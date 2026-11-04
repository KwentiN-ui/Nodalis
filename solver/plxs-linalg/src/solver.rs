//! Direct solvers on `faer`: LDLᵀ for symmetric systems. The symbolic analysis (ordering,
//! elimination tree, supernodes; `faer`'s) is done once per pattern and reused for every
//! numeric factorization with the same pattern. Supernodal structures are factorized by the
//! tree-parallel method of [`crate::supernodal`], small (simplicial) ones by
//! `faer`.

use faer::Side;
use faer::dyn_stack::{MemBuffer, MemStack};
use faer::linalg::cholesky::ldlt::factor::LdltRegularization;
use faer::sparse::linalg::SupernodalThreshold;
use faer::sparse::linalg::cholesky::{
    CholeskySymbolicParams, SymbolicCholesky, SymbolicCholeskyRaw, SymmetricOrdering,
    factorize_symbolic_cholesky, supernodal::SupernodalLdltRef,
};
use faer::sparse::{SparseColMatRef, SymbolicSparseColMatRef};

use plxs_core::{Error, Result};

use crate::supernodal::{self, Regularization, TreeSymbolic, Workspace};
use crate::{Pattern, SparseMatrix};

/// A direct or iterative solver of `K x = b`.
pub trait LinearSolver {
    /// Symbolic analysis (ordering, supernodes) of a pattern; reused by every factorization
    /// until the next `analyze`.
    fn analyze(&mut self, pattern: &Pattern) -> Result<()>;
    /// Numeric factorization of the values on the last analyzed pattern.
    fn factorize(&mut self, matrix: &SparseMatrix) -> Result<()>;
    /// Solves `K x = b` in place for one right-hand side.
    fn solve(&self, rhs: &mut [f64]) -> Result<()>;
    /// Number of negative pivots of the last factorization (for the Sturm check of the
    /// eigensolvers), when the solver knows it.
    fn inertia(&self) -> Option<usize>;
}

/// The index type of the factor's structure. The pattern of the matrix fits 32 bits, the
/// factor of a large model does not: a million equations of quadratic tetrahedra fill it
/// with more than 2³¹ entries, which `faer` refuses for `u32` (`IndexOverflow`).
pub(crate) type FactorIndex = u64;

/// `PLXS_LDLT_TRACE=1`: the time since `clock` of each phase of the analysis.
pub(crate) fn trace(clock: &std::time::Instant, what: &str) {
    if std::env::var("PLXS_LDLT_TRACE").is_ok() {
        eprintln!("ldlt: {what} after {:.2} s", clock.elapsed().as_secs_f64());
    }
}

/// Systems of at least this many equations are ordered by nested dissection.
const DISSECTION_FROM: usize = 3000;

/// Sparse LDLᵀ (supernodal) from `faer`, ordered by nested dissection or AMD.
pub struct LdltSolver {
    par: faer::Par,
    /// Positions of the unknowns of the next `analyze`, for the geometric dissection.
    coordinates: Option<Vec<[f64; 3]>>,
    symbolic: Option<SymbolicCholesky<FactorIndex>>,
    /// The pattern of `analyzed` with indices of the factor's type, for `faer`.
    wide: (Vec<FactorIndex>, Vec<FactorIndex>),
    /// The pattern `symbolic` belongs to: an `analyze` of the same pattern is skipped (the
    /// Newton iterations of a step and the steps of a model share one).
    analyzed: Option<Pattern>,
    reused: u32,
    /// Elimination tree and permuted structure for the tree-parallel factorization, when
    /// the symbolic analysis chose the supernodal structure.
    tree: Option<TreeSymbolic>,
    /// Scratch buffers of the factorization, reused over refactorizations.
    workspace: Workspace,
    /// When `faer` chooses the supernodal over the simplicial structure.
    supernodal_threshold: SupernodalThreshold,
    values: Vec<f64>,
    factorized: bool,
    /// Relative threshold below which a pivot is replaced by `±threshold · scale` (its sign
    /// kept); `0.0` means none. Indefinite matrices (`K − σM` with `σ` inside the spectrum,
    /// the Sturm checks of the eigensolvers) can meet pivots that are exactly zero without
    /// being singular, which LDLᵀ without pivoting cannot pass otherwise.
    regularization: f64,
    /// Whether `values` holds the memory of the analyzed factor
    /// ([`Self::estimate_factor_bytes`] analyzes without it).
    allocated: bool,
}

/// The start of the message of a factorization that did not fit into the memory.
pub const OUT_OF_MEMORY: &str = "not enough memory";

/// Whether `error` is a factorization that did not fit into the memory.
pub fn is_out_of_memory(error: &Error) -> bool {
    matches!(error, Error::Numerical { message, .. }
        if message.starts_with(OUT_OF_MEMORY) || message.starts_with("out of memory"))
}

impl LdltSolver {
    /// A solver that uses `threads` threads of the global rayon pool (1: sequential).
    pub fn new(threads: usize) -> Self {
        let par = if threads <= 1 {
            faer::Par::Seq
        } else {
            faer::Par::rayon(threads)
        };
        Self {
            par,
            coordinates: None,
            symbolic: None,
            wide: (Vec::new(), Vec::new()),
            analyzed: None,
            reused: 0,
            tree: None,
            workspace: Workspace::default(),
            supernodal_threshold: SupernodalThreshold::AUTO,
            values: Vec::new(),
            factorized: false,
            regularization: 0.0,
            allocated: false,
        }
    }

    /// The pivots `D` of the last factorization, indexed like the unknowns of the matrix
    /// (not in the elimination order). A pivot that is zero or tiny against the diagonal
    /// entry of its unknown marks a singular direction: what is left of that unknown's
    /// stiffness once the unknowns eliminated before it took theirs.
    pub fn pivots(&self) -> Option<Vec<f64>> {
        let symbolic = self.symbolic.as_ref().filter(|_| self.factorized)?;
        let n = symbolic.nrows();
        // D in the elimination order.
        let mut eliminated = vec![0.0; n];
        match symbolic.raw() {
            SymbolicCholeskyRaw::Simplicial(simplicial) => {
                // The first stored entry of every column of the factor is its diagonal, D_j.
                let col_ptr = simplicial.col_ptr();
                for (j, d) in eliminated.iter_mut().enumerate() {
                    *d = self.values[col_ptr[j] as usize];
                }
            }
            SymbolicCholeskyRaw::Supernodal(supernodal) => {
                let ldlt = SupernodalLdltRef::new(supernodal, &self.values);
                let begin = supernodal.supernode_begin();
                for (s, &first) in begin.iter().enumerate().take(supernodal.n_supernodes()) {
                    let block = ldlt.supernode(s).val();
                    let first = first as usize;
                    for j in 0..block.ncols() {
                        eliminated[first + j] = block[(j, j)];
                    }
                }
            }
        }
        Some(match symbolic.perm() {
            Some(perm) => {
                let (forward, _) = perm.arrays();
                let mut pivots = vec![0.0; n];
                for (new, &old) in forward.iter().enumerate() {
                    pivots[old as usize] = eliminated[new];
                }
                pivots
            }
            None => eliminated,
        })
    }

    /// How many `analyze` calls were skipped because the pattern was the analyzed one.
    pub fn analyses_reused(&self) -> u32 {
        self.reused
    }

    /// Always uses the supernodal (tree-parallel) factorization, also for systems that
    /// `faer` would factorize column by column; for tests of the supernodal path on small
    /// matrices.
    pub fn force_supernodal(mut self) -> Self {
        self.supernodal_threshold = SupernodalThreshold::FORCE_SUPERNODAL;
        self
    }

    /// Gives the positions of the unknowns (one per equation of the pattern analyzed next);
    /// the ordering then bisects geometrically, which fills far less on meshes. Taken by the
    /// next `analyze`.
    pub fn set_coordinates(&mut self, coordinates: Option<Vec<[f64; 3]>>) {
        self.coordinates = coordinates;
    }

    /// Nonzeros of the factor after `analyze`, a measure of the work of a factorization.
    pub fn factor_nonzeros(&self) -> Option<usize> {
        self.symbolic.as_ref().map(|s| s.len_val())
    }

    /// Replaces pivots smaller than `relative` times the largest diagonal entry by that
    /// value with the pivot's sign (dynamic regularization), for indefinite matrices.
    /// `0.0` switches it off (the default: a zero pivot is then an error).
    pub fn set_regularization(&mut self, relative: f64) {
        self.regularization = relative;
    }

    /// The relative pivot threshold of [`Self::set_regularization`].
    pub fn regularization(&self) -> f64 {
        self.regularization
    }

    /// Forgets the values of the last factorization (the analysis and the factor's memory
    /// stay): `solve` refuses until the next `factorize`. For a solver lent to another
    /// computation that factorized other values.
    pub fn discard_factor(&mut self) {
        self.factorized = false;
    }

    /// Whether the solver holds a factor (`solve` works).
    pub fn is_factorized(&self) -> bool {
        self.factorized
    }

    /// The pattern with the factor's index type.
    fn widen(pattern: &Pattern) -> (Vec<FactorIndex>, Vec<FactorIndex>) {
        let wide = |v: &[u32]| v.iter().map(|&i| i as FactorIndex).collect();
        (wide(pattern.col_ptr()), wide(pattern.row_idx()))
    }

    fn symbolic_matrix(
        n: usize,
        (col_ptr, row_idx): &(Vec<FactorIndex>, Vec<FactorIndex>),
    ) -> SymbolicSparseColMatRef<'_, FactorIndex> {
        SymbolicSparseColMatRef::new_checked(n, n, col_ptr, None, row_idx)
    }
}

fn numerical(message: impl Into<String>) -> Error {
    Error::Numerical {
        step: 0,
        increment: 0,
        message: message.into(),
    }
}

impl LdltSolver {
    /// Bytes the factor of `pattern` will take (its values; the structure is small beside
    /// them), from the symbolic analysis alone: the factor's memory is not taken yet. The
    /// analysis is kept, so an [`LinearSolver::analyze`] of the same pattern only allocates.
    pub fn estimate_factor_bytes(&mut self, pattern: &Pattern) -> Result<u64> {
        self.analyze_with(pattern, false)?;
        Ok(self.factor_nonzeros().unwrap_or(0) as u64 * 8)
    }

    /// The symbolic analysis of `pattern`, and with `allocate` the memory of its factor.
    fn analyze_with(&mut self, pattern: &Pattern, allocate: bool) -> Result<()> {
        if self.symbolic.is_some() && self.analyzed.as_ref() == Some(pattern) {
            // The factor of the previous values stays valid until the next factorize; the
            // ordering of the pattern stays too.
            self.coordinates = None;
            self.reused += 1;
            if allocate && !self.allocated {
                self.allocate()?;
            }
            return Ok(());
        }
        let clock = std::time::Instant::now();
        let wide = Self::widen(pattern);
        let matrix = Self::symbolic_matrix(pattern.dimension(), &wide);
        // Nested dissection for large systems (it fills three-dimensional meshes far less
        // than AMD, see `ordering`), AMD for small ones, where it is as good and cheaper.
        let coordinates = self.coordinates.take();
        // `PLXS_ORDERING=amd|nd` forces one ordering, for measurements.
        let dissect = match std::env::var("PLXS_ORDERING").as_deref() {
            Ok("amd") => false,
            Ok("nd") => true,
            _ => pattern.dimension() >= DISSECTION_FROM,
        };
        let dissection =
            dissect.then(|| crate::ordering::nested_dissection(pattern, coordinates.as_deref()));
        trace(&clock, "ordering");
        let inverse = dissection.as_deref().map(crate::ordering::inverse);
        let wide_perm =
            |p: Vec<u32>| -> Vec<FactorIndex> { p.into_iter().map(|i| i as FactorIndex).collect() };
        let (dissection, inverse) = (dissection.map(wide_perm), inverse.map(wide_perm));
        let ordering =
            match (&dissection, &inverse) {
                (Some(perm), Some(inv)) => SymmetricOrdering::Custom(
                    faer::perm::PermRef::new_checked(perm, inv, pattern.dimension()),
                ),
                _ => SymmetricOrdering::Amd,
            };
        let symbolic = factorize_symbolic_cholesky(
            matrix,
            Side::Upper,
            ordering,
            CholeskySymbolicParams {
                supernodal_flop_ratio_threshold: self.supernodal_threshold,
                ..Default::default()
            },
        )
        .map_err(|e| numerical(format!("symbolic factorization failed: {e:?}")))?;
        trace(&clock, "symbolic factorization");
        self.values = Vec::new();
        self.allocated = false;
        self.workspace.clear();
        self.tree = match symbolic.raw() {
            SymbolicCholeskyRaw::Supernodal(raw) => {
                let inverse = symbolic.perm().map(|perm| perm.arrays().1);
                Some(TreeSymbolic::new(raw, pattern, inverse))
            }
            SymbolicCholeskyRaw::Simplicial(_) => None,
        };
        trace(&clock, "supernodal tree");
        self.symbolic = Some(symbolic);
        self.wide = wide;
        self.analyzed = Some(pattern.clone());
        self.factorized = false;
        if allocate {
            self.allocate()?;
        }
        Ok(())
    }

    /// Takes (and touches) the memory of the analyzed factor. A factor larger than the
    /// memory is an error the run reports, not an abort of the process (the built-in solver
    /// runs inside Nodalis).
    fn allocate(&mut self) -> Result<()> {
        let symbolic =
            (self.symbolic.as_ref()).ok_or_else(|| numerical("allocate before analyze"))?;
        let (len, n) = (symbolic.len_val(), symbolic.nrows());
        self.values = Vec::new();
        self.values.try_reserve_exact(len).map_err(|_| {
            numerical(format!(
                "{OUT_OF_MEMORY}: the factor of the {n} equations needs {:.1} GB",
                len as f64 * 8.0 / 1e9
            ))
        })?;
        // The allocation is mapped lazily; writing it once here takes its page faults in one
        // thread. Taken by the threads of the factorization at once they cost several times
        // more (the kernel serializes them; measured in the cloud VM: 1.9 GB of factor 1.1 s
        // here against about 3 s inside the parallel factorization).
        self.values.resize(len, 0.0);
        self.allocated = true;
        Ok(())
    }
}

impl LinearSolver for LdltSolver {
    fn analyze(&mut self, pattern: &Pattern) -> Result<()> {
        self.analyze_with(pattern, true)
    }

    fn factorize(&mut self, matrix: &SparseMatrix) -> Result<()> {
        if self.symbolic.is_some() && !self.allocated {
            self.allocate()?;
        }
        let symbolic = self
            .symbolic
            .as_ref()
            .ok_or_else(|| numerical("factorize before analyze"))?;
        let pattern = matrix.pattern();
        let threshold = if self.regularization > 0.0 {
            let largest = matrix
                .diagonal()
                .iter()
                .fold(0.0_f64, |m, v| m.max(v.abs()))
                .max(f64::MIN_POSITIVE);
            self.regularization * largest
        } else {
            0.0
        };
        // The tree-parallel factorization for supernodal structures (`PLXS_FACTOR=faer`
        // falls back to faer's own, for measurements);
        // faer's simplicial factorization for the small systems it chose that for.
        let own = std::env::var("PLXS_FACTOR").as_deref() != Ok("faer");
        match (symbolic.raw(), &self.tree) {
            (SymbolicCholeskyRaw::Supernodal(raw), Some(tree)) if own => {
                supernodal::factorize(
                    raw,
                    tree,
                    matrix.values(),
                    &mut self.values,
                    Regularization {
                        signs: None,
                        delta: threshold,
                        epsilon: threshold,
                    },
                    self.par,
                    &self.workspace,
                )
                .map_err(|e| match e {
                    supernodal::FactorError::Cancelled => Error::Cancelled,
                    supernodal::FactorError::Ldlt(e) => {
                        numerical(format!("the stiffness matrix is singular: {e:?}"))
                    }
                })?;
            }
            _ => {
                let a = SparseColMatRef::new(
                    Self::symbolic_matrix(pattern.dimension(), &self.wide),
                    matrix.values(),
                );
                let scratch =
                    symbolic.factorize_numeric_ldlt_scratch::<f64>(self.par, Default::default());
                let mut buffer = MemBuffer::try_new(scratch)
                    .map_err(|_| numerical("out of memory in the factorization"))?;
                symbolic
                    .factorize_numeric_ldlt::<f64>(
                        &mut self.values,
                        a,
                        Side::Upper,
                        LdltRegularization {
                            dynamic_regularization_signs: None,
                            dynamic_regularization_delta: threshold,
                            dynamic_regularization_epsilon: threshold,
                        },
                        self.par,
                        MemStack::new(&mut buffer),
                        Default::default(),
                    )
                    .map_err(|e| numerical(format!("the stiffness matrix is singular: {e:?}")))?;
            }
        }
        self.factorized = true;
        Ok(())
    }

    fn solve(&self, rhs: &mut [f64]) -> Result<()> {
        let symbolic = self
            .symbolic
            .as_ref()
            .filter(|_| self.factorized)
            .ok_or_else(|| numerical("solve before factorize"))?;
        let n = symbolic.nrows();
        assert_eq!(rhs.len(), n, "right-hand side of the wrong size");
        // The tree-parallel substitutions (`PLXS_SOLVE=faer` takes faer's own).
        if let (SymbolicCholeskyRaw::Supernodal(raw), Some(tree), true) = (
            symbolic.raw(),
            &self.tree,
            self.par.degree() > 1 && std::env::var("PLXS_SOLVE").as_deref() != Ok("faer"),
        ) {
            let mut x = match symbolic.perm() {
                Some(perm) => perm
                    .arrays()
                    .0
                    .iter()
                    .map(|&old| rhs[old as usize])
                    .collect(),
                None => rhs.to_vec(),
            };
            supernodal::solve(raw, tree, &self.values, &mut x, self.par.degree());
            match symbolic.perm() {
                Some(perm) => {
                    for (&old, &value) in perm.arrays().0.iter().zip(&x) {
                        rhs[old as usize] = value;
                    }
                }
                None => rhs.copy_from_slice(&x),
            }
            return Ok(());
        }
        let factor = faer::sparse::linalg::cholesky::LdltRef::new(symbolic, &self.values);
        let scratch = symbolic.solve_in_place_scratch::<f64>(1, self.par);
        let mut buffer =
            MemBuffer::try_new(scratch).map_err(|_| numerical("out of memory in the solve"))?;
        let mut x = faer::MatMut::from_column_major_slice_mut(rhs, n, 1);
        factor.solve_in_place_with_conj(
            faer::Conj::No,
            x.as_mut(),
            self.par,
            MemStack::new(&mut buffer),
        );
        Ok(())
    }

    /// The number of negative entries of `D`: by Sylvester's law of inertia the number of
    /// negative eigenvalues of the factorized matrix, which is what the Sturm sequence
    /// check of the eigensolvers counts (`K − σM` has as many negative pivots as there are
    /// eigenvalues below `σ`).
    fn inertia(&self) -> Option<usize> {
        let symbolic = self.symbolic.as_ref().filter(|_| self.factorized)?;
        let negative = match symbolic.raw() {
            SymbolicCholeskyRaw::Simplicial(simplicial) => {
                // The first stored entry of every column of the factor is its diagonal, D_j.
                let col_ptr = simplicial.col_ptr();
                (0..simplicial.nrows())
                    .filter(|&j| self.values[col_ptr[j] as usize] < 0.0)
                    .count()
            }
            SymbolicCholeskyRaw::Supernodal(supernodal) => {
                let ldlt = SupernodalLdltRef::new(supernodal, &self.values);
                (0..supernodal.n_supernodes())
                    .map(|s| {
                        let block = ldlt.supernode(s).val();
                        (0..block.ncols()).filter(|&j| block[(j, j)] < 0.0).count()
                    })
                    .sum()
            }
        };
        Some(negative)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[test]
    fn ldlt_solves_a_small_spd_system() {
        // A = [[4, 1, 0], [1, 3, 1], [0, 1, 2]], x = [1, 2, 3] -> b = [6, 10, 8].
        let pattern = Arc::new(Pattern::from_blocks(3, [&[0u32, 1][..], &[1, 2][..]]));
        let mut a = SparseMatrix::zeros(pattern.clone());
        a.add(0, 0, 4.0);
        a.add(1, 1, 3.0);
        a.add(2, 2, 2.0);
        a.add(0, 1, 1.0);
        a.add(2, 1, 1.0);
        let mut solver = LdltSolver::new(1);
        solver.analyze(&pattern).unwrap();
        solver.factorize(&a).unwrap();
        let mut b = vec![6.0, 10.0, 8.0];
        solver.solve(&mut b).unwrap();
        for (x, expected) in b.iter().zip([1.0, 2.0, 3.0]) {
            assert!((x - expected).abs() < 1e-12, "{b:?}");
        }
        // Refactorize with the same pattern and other values; analyzing the same pattern
        // again keeps the symbolic factorization.
        solver.analyze(&pattern).unwrap();
        assert_eq!(solver.analyses_reused(), 1);
        a.add(0, 0, 4.0);
        solver.factorize(&a).unwrap();
        let mut b = vec![10.0, 10.0, 8.0];
        solver.solve(&mut b).unwrap();
        for (x, expected) in b.iter().zip([1.0, 2.0, 3.0]) {
            assert!((x - expected).abs() < 1e-12, "{b:?}");
        }
    }

    #[test]
    fn inertia_counts_negative_eigenvalues() {
        // A = [[1, 2, 0], [2, -3, 1], [0, 1, 2]]: eigenvalues -3.98, 1.33, 2.65 -> one negative.
        let pattern = Arc::new(Pattern::from_blocks(3, [&[0u32, 1][..], &[1, 2][..]]));
        let mut a = SparseMatrix::zeros(pattern.clone());
        a.add(0, 0, 1.0);
        a.add(1, 1, -3.0);
        a.add(2, 2, 2.0);
        a.add(0, 1, 2.0);
        a.add(1, 2, 1.0);
        let mut solver = LdltSolver::new(1);
        solver.analyze(&pattern).unwrap();
        solver.factorize(&a).unwrap();
        assert_eq!(solver.inertia(), Some(1));
        // Shifted by -5: all eigenvalues negative.
        let mut b = a.clone();
        for i in 0..3 {
            b.add(i, i, -5.0);
        }
        solver.factorize(&b).unwrap();
        assert_eq!(solver.inertia(), Some(3));
        // A larger banded matrix that faer factorizes supernodally: tridiagonal with 2 on
        // the diagonal and -1 off it has eigenvalues 2 - 2 cos(kπ/(n+1)) > 0; shifted by -1
        // the eigenvalues below 1 are those with k < (n+1)/3.
        let n = 400usize;
        let blocks: Vec<[u32; 2]> = (0..n as u32 - 1).map(|i| [i, i + 1]).collect();
        let pattern = Arc::new(Pattern::from_blocks(n, blocks.iter().map(|b| &b[..])));
        let mut t = SparseMatrix::zeros(pattern.clone());
        for i in 0..n as u32 {
            t.add(i, i, 1.0);
            if i + 1 < n as u32 {
                t.add(i, i + 1, -1.0);
            }
        }
        let mut solver = LdltSolver::new(1);
        // Without pivoting the natural order meets an exact zero pivot (1 - 1/1 = 0);
        // regularization lets the factorization pass with the right signs.
        solver.analyze(&pattern).unwrap();
        assert!(solver.factorize(&t).is_err());
        solver.set_regularization(1e-12);
        solver.factorize(&t).unwrap();
        let expected = (1..=n)
            .filter(|&k| {
                2.0 - 2.0 * (k as f64 * std::f64::consts::PI / (n as f64 + 1.0)).cos() < 1.0
            })
            .count();
        assert_eq!(solver.inertia(), Some(expected));
    }

    #[test]
    fn pivots_show_the_singular_unknown() {
        // A chain of three springs, held at no end: singular once; regularization lets the
        // factorization pass and the last pivot in the elimination order is zero. Both the
        // simplicial and the tree-parallel supernodal factorization.
        let pattern = Arc::new(Pattern::from_blocks(3, [&[0u32, 1][..], &[1, 2][..]]));
        let mut a = SparseMatrix::zeros(pattern.clone());
        for (i, j) in [(0, 1), (1, 2)] {
            a.add(i, i, 1.0);
            a.add(j, j, 1.0);
            a.add(i, j, -1.0);
        }
        for supernodal in [false, true] {
            let mut solver = LdltSolver::new(1);
            if supernodal {
                solver = solver.force_supernodal();
            }
            solver.analyze(&pattern).unwrap();
            solver.set_regularization(1e-14);
            solver.factorize(&a).unwrap();
            let pivots = solver.pivots().unwrap();
            let diagonal = a.diagonal();
            let zero = (0..3).filter(|&j| pivots[j] / diagonal[j] < 1e-10).count();
            assert_eq!(zero, 1, "supernodal {supernodal}: {pivots:?}");
            // Held at one end: no zero pivot.
            let mut held = a.clone();
            held.add(0, 0, 1.0);
            solver.factorize(&held).unwrap();
            let pivots = solver.pivots().unwrap();
            assert!(
                (0..3).all(|j| pivots[j] > 0.1),
                "supernodal {supernodal}: {pivots:?}"
            );
        }
    }

    #[test]
    fn singular_matrices_are_reported() {
        let pattern = Arc::new(Pattern::from_blocks(2, [&[0u32, 1][..]]));
        let mut a = SparseMatrix::zeros(pattern.clone());
        a.add(0, 0, 1.0);
        a.add(0, 1, 1.0);
        a.add(1, 1, 1.0);
        let mut solver = LdltSolver::new(1);
        solver.analyze(&pattern).unwrap();
        assert!(solver.factorize(&a).is_err());
    }
}
