//! Direct solvers on `faer`: LDLᵀ for symmetric systems. The symbolic analysis (AMD
//! ordering, elimination tree, supernodes) is done once per pattern and reused for every
//! numeric factorization with the same pattern.

use faer::Side;
use faer::dyn_stack::{MemBuffer, MemStack};
use faer::linalg::cholesky::ldlt::factor::LdltRegularization;
use faer::sparse::linalg::cholesky::{
    CholeskySymbolicParams, SymbolicCholesky, SymmetricOrdering, factorize_symbolic_cholesky,
};
use faer::sparse::{SparseColMatRef, SymbolicSparseColMatRef};

use plxs_core::{Error, Result};

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

/// Sparse LDLᵀ (supernodal, AMD ordering) from `faer`.
pub struct LdltSolver {
    par: faer::Par,
    symbolic: Option<SymbolicCholesky<u32>>,
    /// The pattern `symbolic` belongs to: an `analyze` of the same pattern is skipped (the
    /// Newton iterations of a step and the steps of a model share one).
    analyzed: Option<Pattern>,
    reused: u32,
    values: Vec<f64>,
    factorized: bool,
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
            symbolic: None,
            analyzed: None,
            reused: 0,
            values: Vec::new(),
            factorized: false,
        }
    }

    /// How many `analyze` calls were skipped because the pattern was the analyzed one.
    pub fn analyses_reused(&self) -> u32 {
        self.reused
    }

    fn symbolic_matrix(pattern: &Pattern) -> SymbolicSparseColMatRef<'_, u32> {
        SymbolicSparseColMatRef::new_checked(
            pattern.dimension(),
            pattern.dimension(),
            pattern.col_ptr(),
            None,
            pattern.row_idx(),
        )
    }
}

fn numerical(message: impl Into<String>) -> Error {
    Error::Numerical {
        step: 0,
        increment: 0,
        message: message.into(),
    }
}

impl LinearSolver for LdltSolver {
    fn analyze(&mut self, pattern: &Pattern) -> Result<()> {
        if self.symbolic.is_some() && self.analyzed.as_ref() == Some(pattern) {
            // The factor of the previous values stays valid until the next factorize.
            self.reused += 1;
            return Ok(());
        }
        let matrix = Self::symbolic_matrix(pattern);
        let symbolic = factorize_symbolic_cholesky(
            matrix,
            Side::Upper,
            SymmetricOrdering::Amd,
            CholeskySymbolicParams::default(),
        )
        .map_err(|e| numerical(format!("symbolic factorization failed: {e:?}")))?;
        self.values = vec![0.0; symbolic.len_val()];
        self.symbolic = Some(symbolic);
        self.analyzed = Some(pattern.clone());
        self.factorized = false;
        Ok(())
    }

    fn factorize(&mut self, matrix: &SparseMatrix) -> Result<()> {
        let symbolic = self
            .symbolic
            .as_ref()
            .ok_or_else(|| numerical("factorize before analyze"))?;
        let pattern = matrix.pattern();
        let a = SparseColMatRef::new(Self::symbolic_matrix(pattern), matrix.values());
        let scratch = symbolic.factorize_numeric_ldlt_scratch::<f64>(self.par, Default::default());
        let mut buffer = MemBuffer::try_new(scratch)
            .map_err(|_| numerical("out of memory in the factorization"))?;
        symbolic
            .factorize_numeric_ldlt::<f64>(
                &mut self.values,
                a,
                Side::Upper,
                LdltRegularization {
                    dynamic_regularization_signs: None,
                    dynamic_regularization_delta: 0.0,
                    dynamic_regularization_epsilon: 0.0,
                },
                self.par,
                MemStack::new(&mut buffer),
                Default::default(),
            )
            .map_err(|e| numerical(format!("the stiffness matrix is singular: {e:?}")))?;
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

    fn inertia(&self) -> Option<usize> {
        None
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
