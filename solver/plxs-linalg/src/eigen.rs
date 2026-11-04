//! Eigensolvers on `faer`: Lanczos with full reorthogonalization and thick restart
//! (Wu & Simon, "Thick-restart Lanczos method for large symmetric eigenvalue problems",
//! SIAM J. Matrix Anal. Appl. 22, 2000) on a shift-inverted operator, with a Sturm
//! sequence check (Sylvester's law of inertia on the LDLᵀ of `K − σM`) that no eigenvalue
//! was missed. Textbook material (Parlett, "The Symmetric Eigenvalue Problem"; Bathe,
//! "Finite Element Procedures", chapter 11); no ARPACK or CalculiX code was consulted.
//!
//! Two problems are served:
//!
//! * **Frequencies**: `K x = λ M x` with `M` positive semi-definite. The operator is
//!   `(K − σM)⁻¹ M`, self-adjoint in the `M` inner product; its eigenvalues `θ = 1/(λ − σ)`
//!   are largest for the `λ` nearest the shift, which Lanczos finds first. Eigenvectors are
//!   mass normalized (`xᵀ M x = 1`).
//! * **Buckling**: `(K + λ K_σ) x = 0` with `K` positive definite. The operator is
//!   `K⁻¹ K_σ`, self-adjoint in the `K` inner product, eigenvalues `μ = −1/λ`; the smallest
//!   positive buckling factors are the most negative `μ`, extreme eigenvalues that Lanczos
//!   finds first. Eigenvectors come out with `xᵀ K x = 1` and are rescaled by the caller.
//!
//! Limitation: the shifted matrices are factorized by LDLᵀ without pivoting (dynamic
//! regularization of tiny pivots keeps the signs for the Sturm count). A shift that makes
//! `K − σM` meet a vanishing pivot without being singular is a measure-zero event for finite
//! element matrices; a Bunch-Kaufman variant is the remedy if it ever shows up.
//!
//! Multiple (degenerate) eigenvalues: a single Lanczos run finds one vector of a
//! multiple eigenvalue; the Sturm count then reports the missing copies, a fresh random
//! vector orthogonal to the basis is injected and the iteration continues until the count
//! agrees.

use plxs_core::{Error, Result};

use crate::{LdltSolver, LinearSolver, PcgSolver, SparseMatrix};

/// Eigenpairs, sorted by eigenvalue (frequencies ascending; buckling factors ascending
/// among the positive ones).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct EigenPairs {
    pub values: Vec<f64>,
    /// One vector per eigenvalue, in the DOF numbering of the matrices.
    pub vectors: Vec<Vec<f64>>,
    /// The largest residual `‖K x − λ M x‖ / ‖K x‖` (or `‖(K + λ K_σ) x‖ / ‖K x‖`).
    pub residual: f64,
    /// Number of Lanczos steps (operator applications) taken.
    pub steps: usize,
    /// Whether the Sturm sequence check confirmed the count.
    pub sturm_confirmed: bool,
}

/// The operator a Lanczos run applies: `y = (K − σM)⁻¹ M x` for frequencies, `K⁻¹ K_σ x`
/// for buckling.
type Operator<'a> = dyn FnMut(&[f64], &mut [f64]) -> Result<()> + 'a;

#[derive(Clone, Debug, PartialEq)]
pub struct LanczosOptions {
    /// Relative residual of a Ritz pair (`β |y_last| / |θ|`) below which it counts as
    /// converged.
    pub tolerance: f64,
    /// Size of the Krylov basis before a restart; `None` picks `max(2·count + 16, 32)`.
    pub max_basis: Option<usize>,
    /// Restarts before giving up.
    pub max_restarts: usize,
    /// Rounds of random injection after a Sturm count that disagrees.
    pub max_sturm_rounds: usize,
    pub seed: u64,
}

impl Default for LanczosOptions {
    fn default() -> Self {
        Self {
            tolerance: 1e-10,
            max_basis: None,
            max_restarts: 200,
            max_sturm_rounds: 6,
            seed: 0x5eed_1234_abcd_0001,
        }
    }
}

/// The `count` smallest eigenpairs of `K x = λ M x` (shift-invert Lanczos on LDLᵀ) with a
/// Sturm sequence check, as the interface document specifies.
pub trait EigenSolver {
    fn solve(
        &mut self,
        k: &SparseMatrix,
        m: &SparseMatrix,
        count: usize,
        shift: f64,
    ) -> Result<EigenPairs>;
}

pub struct ShiftInvertLanczos {
    solver: LdltSolver,
    /// The inner solver when the shifted systems are solved by conjugate gradients
    /// ([`Self::set_iterative`]) instead of the factorization.
    iterative: Option<PcgSolver>,
    pub options: LanczosOptions,
}

impl ShiftInvertLanczos {
    pub fn new(threads: usize, options: LanczosOptions) -> Self {
        let mut solver = LdltSolver::new(threads);
        solver.set_regularization(1e-12);
        Self {
            solver,
            iterative: None,
            options,
        }
    }

    /// Solves the shifted systems of the Lanczos steps by conjugate gradients with an
    /// incomplete Cholesky preconditioner instead of factorizing them: far less memory, for
    /// models whose factor does not fit. The shifted matrix must then be positive definite
    /// (frequencies: a shift at or below the spectrum; buckling: `K`), and without a
    /// factorization there is no Sturm count. In its place every run ends with rounds of a
    /// fresh random start vector orthogonal to the found eigenvectors, until a round finds
    /// no new eigenvalue among the wanted ones (the missing copies of a multiple eigenvalue
    /// show up there); [`EigenPairs::sturm_confirmed`] stays false.
    pub fn set_iterative(&mut self, iterative: bool) {
        self.iterative = iterative.then(PcgSolver::new);
    }

    pub fn is_iterative(&self) -> bool {
        self.iterative.is_some()
    }

    /// The direct solver, for an estimate of its factor before the choice.
    pub fn direct_mut(&mut self) -> &mut LdltSolver {
        &mut self.solver
    }

    /// `K x = λ M x`: the `count` eigenvalues nearest to `shift` (the smallest ones for
    /// `shift` at or below the spectrum), ascending, with mass normalized vectors.
    pub fn frequencies(
        &mut self,
        k: &SparseMatrix,
        m: &SparseMatrix,
        count: usize,
        shift: f64,
    ) -> Result<EigenPairs> {
        let n = k.dimension();
        let count = count.min(n);
        if count == 0 {
            return Ok(EigenPairs::default());
        }
        let shifted = SparseMatrix::linear_combination(k, 1.0, m, -shift);
        let basis = self.basis_size(count);
        if let Some(pcg) = self.iterative.as_mut() {
            pcg.analyze(shifted.pattern())?;
            pcg.factorize(&shifted)?;
            let pcg = &*pcg;
            let mut op = |x: &[f64], y: &mut [f64]| -> Result<()> {
                m.multiply(x, y);
                pcg.solve(y)
            };
            let b_mul = |x: &[f64], y: &mut [f64]| m.multiply(x, y);
            let (values, vectors, steps) = confirmed_by_rounds(
                n,
                count,
                basis,
                &self.options,
                &mut op,
                &b_mul,
                |theta: f64| theta.abs(),
                |theta| shift + 1.0 / theta,
                |a, b| a.total_cmp(&b),
            )?;
            let residual = residual_norm(k, m, &values, &vectors, |lambda| -lambda);
            return Ok(EigenPairs {
                values,
                vectors,
                residual,
                steps,
                sturm_confirmed: false,
            });
        }
        self.solver.analyze(k.pattern())?;
        self.solver.factorize(&shifted).map_err(|e| match e {
            Error::Numerical { message, .. } => Error::Numerical {
                step: 0,
                increment: 0,
                message: format!(
                    "{message}; the stiffness matrix is singular at the shift {shift:e} (free structure or insufficient constraints)"
                ),
            },
            other => other,
        })?;
        let below_shift = self.solver.inertia().unwrap_or(0);
        let mut scratch = vec![0.0; n];
        let mut lanczos = Lanczos::new(n, self.basis_size(count), self.options.seed);
        let mut rounds = 0;
        let (values, vectors) = loop {
            {
                let solver = &self.solver;
                let mut op = |x: &[f64], y: &mut [f64]| -> Result<()> {
                    m.multiply(x, y);
                    solver.solve(y)
                };
                let b_mul = |x: &[f64], y: &mut [f64]| m.multiply(x, y);
                lanczos.run(count, &mut op, &b_mul, |theta| theta.abs(), &self.options)?;
            }
            let mut pairs: Vec<(f64, Vec<f64>)> = lanczos
                .wanted(count, |theta| theta.abs())
                .into_iter()
                .map(|(theta, x)| (shift + 1.0 / theta, x))
                .collect();
            pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
            let values: Vec<f64> = pairs.iter().map(|p| p.0).collect();

            // Sturm check: the number of eigenvalues in (lo, hi] must be `count`.
            let margin = |v: f64| v.abs().max(1e-300) * 1e-7 + f64::MIN_POSITIVE;
            let hi = values[count - 1] + margin(values[count - 1]);
            let below_hi = self.count_below(k, m, hi, &mut scratch)?;
            let lo_value = values[0] - margin(values[0]);
            let below_lo = if lo_value >= shift {
                below_shift
            } else {
                self.count_below(k, m, lo_value, &mut scratch)?
            };
            let actual = below_hi.saturating_sub(below_lo);
            // Everything the Sturm count sees in the interval must be among the found
            // eigenvalues (the extra ones of earlier rounds included: a multiple eigenvalue
            // at the upper end is found completely and then truncated).
            let found = values
                .iter()
                .filter(|v| **v > lo_value && **v <= hi)
                .count();
            if actual <= found || rounds >= self.options.max_sturm_rounds {
                // Restore the factorization of the shifted matrix for later use.
                self.solver.factorize(&shifted)?;
                let mut pairs = pairs;
                pairs.truncate(count);
                lanczos.sturm_confirmed = actual == found;
                break (
                    pairs.iter().map(|p| p.0).collect::<Vec<f64>>(),
                    pairs.into_iter().map(|p| p.1).collect::<Vec<Vec<f64>>>(),
                );
            }
            // Eigenvalues were missed (multiple eigenvalues, typically): continue with a
            // fresh random vector orthogonal to the basis, asking for the missing ones too.
            rounds += 1;
            self.solver.factorize(&shifted)?;
            lanczos.lock_and_restart(self.options.tolerance);
            lanczos.extra_wanted += actual - found;
        };
        let residual = residual_norm(k, m, &values, &vectors, |lambda| -lambda);
        Ok(EigenPairs {
            values,
            vectors,
            residual,
            steps: lanczos.steps,
            sturm_confirmed: lanczos.sturm_confirmed,
        })
    }

    /// `(K + λ K_σ) x = 0`: the `count` smallest positive buckling factors `λ`, ascending,
    /// with `xᵀ K x = 1`. When fewer positive factors exist, negative ones (load reversal)
    /// follow by magnitude.
    pub fn buckling(
        &mut self,
        k: &SparseMatrix,
        k_sigma: &SparseMatrix,
        count: usize,
    ) -> Result<EigenPairs> {
        let n = k.dimension();
        let count = count.min(n);
        if count == 0 {
            return Ok(EigenPairs::default());
        }
        // Most negative μ = −1/λ first; positive μ (negative λ) only when nothing else is left.
        let preference = |theta: f64| {
            if theta < 0.0 { -theta } else { -1e300 + theta }
        };
        // Positive factors ascending, then negative ones by magnitude.
        let order = |a: f64, b: f64| {
            let key = |v: f64| if v > 0.0 { (0, v) } else { (1, -v) };
            key(a).partial_cmp(&key(b)).unwrap()
        };
        let basis = self.basis_size(count);
        if let Some(pcg) = self.iterative.as_mut() {
            pcg.analyze(k.pattern())?;
            pcg.factorize(k)?;
            let pcg = &*pcg;
            let mut op = |x: &[f64], y: &mut [f64]| -> Result<()> {
                k_sigma.multiply(x, y);
                pcg.solve(y)
            };
            let b_mul = |x: &[f64], y: &mut [f64]| k.multiply(x, y);
            let (values, vectors, steps) = confirmed_by_rounds(
                n,
                count,
                basis,
                &self.options,
                &mut op,
                &b_mul,
                preference,
                |theta| -1.0 / theta,
                order,
            )?;
            let residual = residual_norm(k, k_sigma, &values, &vectors, |lambda| lambda);
            return Ok(EigenPairs {
                values,
                vectors,
                residual,
                steps,
                sturm_confirmed: false,
            });
        }
        self.solver.analyze(k.pattern())?;
        self.solver.factorize(k).map_err(|e| match e {
            Error::Numerical { message, .. } => Error::Numerical {
                step: 0,
                increment: 0,
                message: format!("{message}; the model is probably not sufficiently constrained"),
            },
            other => other,
        })?;
        if self.solver.inertia().unwrap_or(0) > 0 {
            return Err(Error::Numerical {
                step: 0,
                increment: 0,
                message: "the stiffness matrix is not positive definite; buckling needs a stable reference state".into(),
            });
        }
        let mut scratch = vec![0.0; n];
        let mut lanczos = Lanczos::new(n, self.basis_size(count), self.options.seed);
        let mut rounds = 0;
        let (values, vectors) = loop {
            {
                let solver = &self.solver;
                let mut op = |x: &[f64], y: &mut [f64]| -> Result<()> {
                    k_sigma.multiply(x, y);
                    solver.solve(y)
                };
                let b_mul = |x: &[f64], y: &mut [f64]| k.multiply(x, y);
                lanczos.run(count, &mut op, &b_mul, preference, &self.options)?;
            }
            let mut pairs: Vec<(f64, Vec<f64>)> = lanczos
                .wanted(count, preference)
                .into_iter()
                .map(|(theta, x)| (-1.0 / theta, x))
                .collect();
            pairs.sort_by(|a, b| order(a.0, b.0));
            let values: Vec<f64> = pairs.iter().map(|p| p.0).collect();
            let positive = values.iter().filter(|v| **v > 0.0).count().min(count);
            let (actual, found) = if positive > 0 {
                let hi = values[positive - 1] * (1.0 + 1e-7);
                // Eigenvalues λ in (0, hi]: negative pivots of K + hi·K_σ (K itself has none).
                let shifted = SparseMatrix::linear_combination(k, 1.0, k_sigma, hi);
                self.solver.factorize(&shifted)?;
                let actual = self.solver.inertia().unwrap_or(positive);
                let found = values.iter().filter(|v| **v > 0.0 && **v <= hi).count();
                (actual, found)
            } else {
                (0, 0)
            };
            let _ = &mut scratch;
            if actual <= found || rounds >= self.options.max_sturm_rounds {
                self.solver.factorize(k)?;
                let mut pairs = pairs;
                pairs.truncate(count);
                lanczos.sturm_confirmed = actual == found;
                break (
                    pairs.iter().map(|p| p.0).collect::<Vec<f64>>(),
                    pairs.into_iter().map(|p| p.1).collect::<Vec<Vec<f64>>>(),
                );
            }
            rounds += 1;
            self.solver.factorize(k)?;
            lanczos.lock_and_restart(self.options.tolerance);
            lanczos.extra_wanted += actual - found;
        };
        let residual = residual_norm(k, k_sigma, &values, &vectors, |lambda| lambda);
        Ok(EigenPairs {
            values,
            vectors,
            residual,
            steps: lanczos.steps,
            sturm_confirmed: lanczos.sturm_confirmed,
        })
    }

    fn basis_size(&self, count: usize) -> usize {
        self.options
            .max_basis
            .unwrap_or_else(|| (2 * count + 16).max(32))
    }

    /// Number of eigenvalues of `K x = λ M x` below `sigma` (Sturm sequence).
    fn count_below(
        &mut self,
        k: &SparseMatrix,
        m: &SparseMatrix,
        sigma: f64,
        _scratch: &mut [f64],
    ) -> Result<usize> {
        let shifted = SparseMatrix::linear_combination(k, 1.0, m, -sigma);
        self.solver.factorize(&shifted)?;
        Ok(self.solver.inertia().unwrap_or(0))
    }
}

impl EigenSolver for ShiftInvertLanczos {
    fn solve(
        &mut self,
        k: &SparseMatrix,
        m: &SparseMatrix,
        count: usize,
        shift: f64,
    ) -> Result<EigenPairs> {
        self.frequencies(k, m, count, shift)
    }
}

/// Lanczos runs on `op` until a round with a fresh random start vector (orthogonal to the
/// converged eigenvectors) and one more wanted eigenpair changes none of the `count`
/// wanted eigenvalues: the check without a Sturm count of the iterative inner solver.
/// `value` maps a Ritz value to the eigenvalue, `order` sorts the eigenvalues. Returns
/// values, vectors and the number of Lanczos steps.
#[allow(clippy::too_many_arguments)]
fn confirmed_by_rounds(
    n: usize,
    count: usize,
    basis: usize,
    options: &LanczosOptions,
    op: &mut Operator<'_>,
    b_mul: &dyn Fn(&[f64], &mut [f64]),
    preference: impl Fn(f64) -> f64 + Copy,
    value: impl Fn(f64) -> f64,
    order: impl Fn(f64, f64) -> std::cmp::Ordering,
) -> Result<(Vec<f64>, Vec<Vec<f64>>, usize)> {
    let mut lanczos = Lanczos::new(n, basis, options.seed);
    let mut previous: Option<Vec<f64>> = None;
    let mut rounds = 0;
    loop {
        lanczos.run(count, op, b_mul, preference, options)?;
        let mut pairs: Vec<(f64, Vec<f64>)> = lanczos
            .wanted(count, preference)
            .into_iter()
            .map(|(theta, x)| (value(theta), x))
            .collect();
        pairs.sort_by(|a, b| order(a.0, b.0));
        pairs.truncate(count);
        let values: Vec<f64> = pairs.iter().map(|p| p.0).collect();
        let same = previous.as_ref().is_some_and(|p| {
            p.len() == values.len()
                && p.iter()
                    .zip(&values)
                    .all(|(a, b)| (a - b).abs() <= 1e-6 * a.abs().max(b.abs()))
        });
        let exhausted = lanczos.locked.len() + lanczos.k() >= n;
        if same || exhausted || rounds >= options.max_sturm_rounds {
            return Ok((
                values,
                pairs.into_iter().map(|p| p.1).collect(),
                lanczos.steps,
            ));
        }
        previous = Some(values);
        rounds += 1;
        lanczos.lock_and_restart(options.tolerance);
        lanczos.extra_wanted += 1;
    }
}

/// `max_i ‖A x_i + f(λ_i) B x_i‖ / ‖A x_i‖`.
fn residual_norm(
    a: &SparseMatrix,
    b: &SparseMatrix,
    values: &[f64],
    vectors: &[Vec<f64>],
    factor: impl Fn(f64) -> f64,
) -> f64 {
    let n = a.dimension();
    let mut ax = vec![0.0; n];
    let mut bx = vec![0.0; n];
    let mut worst = 0.0_f64;
    for (&lambda, x) in values.iter().zip(vectors) {
        a.multiply(x, &mut ax);
        b.multiply(x, &mut bx);
        let f = factor(lambda);
        let num: f64 = ax.iter().zip(&bx).map(|(a, b)| (a + f * b).powi(2)).sum();
        let den: f64 = ax.iter().map(|a| a * a).sum::<f64>().max(f64::MIN_POSITIVE);
        worst = worst.max((num / den).sqrt());
    }
    worst
}

/// The Lanczos process on a self-adjoint operator in a `B` inner product, with a dense
/// projected matrix (full reorthogonalization makes it tridiagonal up to rounding, but
/// the full projection is kept, which also covers the coupling rows after a thick restart
/// and after an injected vector).
struct Lanczos {
    n: usize,
    max_basis: usize,
    /// `B`-orthonormal basis vectors.
    basis: Vec<Vec<f64>>,
    /// `B` times the basis vectors.
    b_basis: Vec<Vec<f64>>,
    /// Projected matrix `Vᵀ B A V`, row-major `max_basis × max_basis`, valid `k × k`.
    t: Vec<f64>,
    /// Next basis vector (unnormalized residual) and its `B`-norm; `None` after a
    /// breakdown until a random vector is injected.
    next: Option<(Vec<f64>, f64)>,
    /// Converged eigenvectors taken out of the iteration (locked): deflated from every
    /// new vector, so that a fresh Lanczos run finds the eigenvalues that are still
    /// missing (the other copies of a multiple eigenvalue).
    locked: Vec<Vec<f64>>,
    b_locked: Vec<Vec<f64>>,
    locked_theta: Vec<f64>,
    rng: u64,
    steps: usize,
    restarts: usize,
    /// Additional eigenvalues wanted beyond `count` after a Sturm check found some missing.
    extra_wanted: usize,
    sturm_confirmed: bool,
}

impl Lanczos {
    fn new(n: usize, max_basis: usize, seed: u64) -> Self {
        let max_basis = max_basis.min(n).max(1);
        let mut this = Self {
            n,
            max_basis,
            basis: Vec::with_capacity(max_basis + 1),
            b_basis: Vec::with_capacity(max_basis + 1),
            t: vec![0.0; (max_basis + 1) * (max_basis + 1)],
            next: None,
            locked: Vec::new(),
            b_locked: Vec::new(),
            locked_theta: Vec::new(),
            rng: seed,
            steps: 0,
            restarts: 0,
            extra_wanted: 0,
            sturm_confirmed: false,
        };
        this.next = Some((this.random_vector(), 0.0));
        this
    }

    fn k(&self) -> usize {
        self.basis.len()
    }

    fn stride(&self) -> usize {
        self.max_basis + 1
    }

    fn random_vector(&mut self) -> Vec<f64> {
        (0..self.n)
            .map(|_| {
                // xorshift64*
                self.rng ^= self.rng >> 12;
                self.rng ^= self.rng << 25;
                self.rng ^= self.rng >> 27;
                let bits = self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D);
                (bits >> 11) as f64 / (1u64 << 53) as f64 - 0.5
            })
            .collect()
    }

    /// Locks the converged Ritz pairs of the current basis (they leave the iteration as
    /// exact eigenvectors and are deflated from all later vectors) and starts a fresh
    /// Lanczos run from a random vector. Used when the Sturm count reports missing
    /// eigenvalues (other copies of a multiple eigenvalue, which one Krylov sequence
    /// cannot separate) and after a breakdown (invariant subspace found).
    fn lock_and_restart(&mut self, tolerance: f64) {
        if self.k() > 0 {
            let (thetas, ys, residuals) = self.ritz();
            for (i, y) in ys.iter().enumerate() {
                if residuals[i] <= tolerance * thetas[i].abs().max(f64::MIN_POSITIVE) {
                    let mut x = vec![0.0; self.n];
                    let mut bx = vec![0.0; self.n];
                    for (c, &yc) in y.iter().enumerate() {
                        axpy(yc, &self.basis[c], &mut x);
                        axpy(yc, &self.b_basis[c], &mut bx);
                    }
                    self.locked.push(x);
                    self.b_locked.push(bx);
                    self.locked_theta.push(thetas[i]);
                }
            }
        }
        self.basis.clear();
        self.b_basis.clear();
        self.t.iter_mut().for_each(|v| *v = 0.0);
        let v = self.random_vector();
        self.next = Some((v, 0.0));
    }

    /// Removes the components along the locked eigenvectors and the basis.
    fn deflate(&self, v: &mut [f64]) {
        for (q, bq) in self.locked.iter().zip(&self.b_locked) {
            let h = dot(bq, v);
            axpy(-h, q, v);
        }
    }

    /// One Lanczos step: appends the next vector to the basis, applies the operator and
    /// projects. Returns false on a breakdown (invariant subspace found).
    fn step(&mut self, op: &mut Operator<'_>, b_mul: &dyn Fn(&[f64], &mut [f64])) -> Result<bool> {
        let Some((mut v, mut beta)) = self.next.take() else {
            return Ok(false);
        };
        let mut bv = vec![0.0; self.n];
        if beta <= 0.0 {
            // A fresh (start or injected) vector: orthogonalize and normalize in B.
            for _ in 0..2 {
                self.deflate(&mut v);
                for (q, bq) in self.basis.iter().zip(&self.b_basis) {
                    let h = dot(bq, &v);
                    axpy(-h, q, &mut v);
                }
            }
            b_mul(&v, &mut bv);
            beta = dot(&bv, &v).max(0.0).sqrt();
            if beta <= 0.0 {
                return Ok(false);
            }
        } else {
            // The residual of the previous step is B-orthogonal to the basis up to rounding
            // relative to the vector it was computed from; after normalization that error
            // is amplified by 1/β, so orthogonalize once more.
            self.deflate(&mut v);
            for (q, bq) in self.basis.iter().zip(&self.b_basis) {
                let h = dot(bq, &v);
                axpy(-h, q, &mut v);
            }
            b_mul(&v, &mut bv);
            let actual = dot(&bv, &v).max(0.0).sqrt();
            if actual > 0.0 {
                beta = actual;
            }
        }
        scale(1.0 / beta, &mut v);
        scale(1.0 / beta, &mut bv);
        self.basis.push(v);
        self.b_basis.push(bv);
        let k = self.k();
        let j = k - 1;
        let stride = self.stride();

        // a = A v_j, projected onto the basis twice (full reorthogonalization).
        let mut a = vec![0.0; self.n];
        op(&self.basis[j], &mut a)?;
        self.steps += 1;
        let mut coefficients = vec![0.0; k];
        let mut ba = vec![0.0; self.n];
        b_mul(&a, &mut ba);
        let norm_a = dot(&ba, &a).max(0.0).sqrt();
        let mut beta_next = 0.0;
        // Gram-Schmidt passes until the remainder is not much smaller than the vector it
        // came from (the DGKS criterion), at least two.
        for pass in 0..4 {
            self.deflate(&mut a);
            for (i, coefficient) in coefficients.iter_mut().enumerate().take(k) {
                let h = dot(&self.b_basis[i], &a);
                axpy(-h, &self.basis[i], &mut a);
                *coefficient += h;
            }
            b_mul(&a, &mut ba);
            beta_next = dot(&ba, &a).max(0.0).sqrt();
            if pass >= 1 && beta_next > 0.5 * norm_a {
                break;
            }
            if beta_next <= 1e-14 * norm_a {
                break;
            }
        }
        for (i, &h) in coefficients.iter().enumerate() {
            self.t[j * stride + i] = h;
            self.t[i * stride + j] = h;
        }
        if beta_next <= 1e-14 * norm_a.max(f64::MIN_POSITIVE) {
            self.next = None;
            return Ok(false);
        }
        self.next = Some((a, beta_next));
        Ok(true)
    }

    /// Eigenpairs of the projected matrix: `(θ_i, y_i, residual_i)` with the residual
    /// estimate `β |y_i[k−1]|`.
    fn ritz(&self) -> (Vec<f64>, Vec<Vec<f64>>, Vec<f64>) {
        let k = self.k();
        let stride = self.stride();
        let mut t = faer::Mat::<f64>::zeros(k, k);
        for i in 0..k {
            for j in 0..k {
                t[(i, j)] = 0.5 * (self.t[i * stride + j] + self.t[j * stride + i]);
            }
        }
        let evd = t
            .self_adjoint_eigen(faer::Side::Lower)
            .expect("eigendecomposition of the projected matrix");
        let beta = self.next.as_ref().map_or(0.0, |(_, b)| *b);
        let mut thetas = Vec::with_capacity(k);
        let mut ys = Vec::with_capacity(k);
        let mut residuals = Vec::with_capacity(k);
        for c in 0..k {
            let y: Vec<f64> = (0..k).map(|r| evd.U()[(r, c)]).collect();
            thetas.push(evd.S()[c]);
            residuals.push(beta * y[k - 1].abs());
            ys.push(y);
        }
        (thetas, ys, residuals)
    }

    /// Indices of the Ritz pairs ordered by preference (most wanted first).
    fn order(thetas: &[f64], preference: &dyn Fn(f64) -> f64) -> Vec<usize> {
        let mut order: Vec<usize> = (0..thetas.len()).collect();
        order.sort_by(|&a, &b| preference(thetas[b]).total_cmp(&preference(thetas[a])));
        order
    }

    /// Runs steps and restarts until the `count + extra_wanted` most wanted Ritz pairs
    /// have converged.
    fn run(
        &mut self,
        count: usize,
        op: &mut Operator<'_>,
        b_mul: &dyn Fn(&[f64], &mut [f64]),
        preference: impl Fn(f64) -> f64,
        options: &LanczosOptions,
    ) -> Result<()> {
        let wanted = (count + self.extra_wanted).min(self.n);
        let cancel = plxs_core::CancelToken::current();
        loop {
            // Extend the basis.
            while self.k() < self.max_basis {
                if cancel
                    .as_ref()
                    .is_some_and(plxs_core::CancelToken::is_cancelled)
                {
                    return Err(Error::Cancelled);
                }
                if !self.step(op, b_mul)? {
                    // Invariant subspace: everything in it is exact; more vectors need a
                    // fresh start direction.
                    if self.converged(wanted, &preference, options.tolerance) {
                        return Ok(());
                    }
                    if self.locked.len() + self.k() >= self.n {
                        return Ok(());
                    }
                    self.lock_and_restart(options.tolerance);
                    continue;
                }
                if self.converged(wanted, &preference, options.tolerance) {
                    return Ok(());
                }
            }
            if self.converged(wanted, &preference, options.tolerance) {
                return Ok(());
            }
            if self.locked.len() + self.k() >= self.n {
                // The whole space is spanned: the Ritz pairs are exact.
                return Ok(());
            }
            if self.restarts >= options.max_restarts {
                return Err(Error::Numerical {
                    step: 0,
                    increment: 0,
                    message: format!(
                        "the eigenvalue iteration did not converge after {} restarts",
                        self.restarts
                    ),
                });
            }
            // Thick restart: keep the wanted Ritz vectors plus a buffer, leaving room for
            // new vectors.
            let keep = (wanted + wanted / 2 + 4)
                .min(self.max_basis.saturating_sub(4))
                .max(1);
            self.restart(keep, &preference);
        }
    }

    /// Whether the `wanted` most wanted eigenpairs (locked ones and Ritz pairs of the
    /// basis together) have converged.
    fn converged(&self, wanted: usize, preference: &dyn Fn(f64) -> f64, tolerance: f64) -> bool {
        if self.locked.len() + self.k() < wanted {
            return false;
        }
        let (mut thetas, _, mut residuals) = if self.k() > 0 {
            self.ritz()
        } else {
            (Vec::new(), Vec::new(), Vec::new())
        };
        thetas.extend_from_slice(&self.locked_theta);
        residuals.extend(std::iter::repeat_n(0.0, self.locked.len()));
        let order = Self::order(&thetas, preference);
        order
            .iter()
            .take(wanted)
            .all(|&i| residuals[i] <= tolerance * thetas[i].abs().max(f64::MIN_POSITIVE))
    }

    /// Replaces the basis by the `keep` most wanted Ritz vectors (thick restart).
    fn restart(&mut self, keep: usize, preference: &dyn Fn(f64) -> f64) {
        let (thetas, ys, residuals) = self.ritz();
        let order = Self::order(&thetas, preference);
        let keep = keep.min(order.len());
        let k = self.k();
        let mut basis = Vec::with_capacity(self.max_basis + 1);
        let mut b_basis = Vec::with_capacity(self.max_basis + 1);
        let mut t = vec![0.0; self.t.len()];
        let stride = self.stride();
        for (new, &i) in order.iter().take(keep).enumerate() {
            let y = &ys[i];
            let mut q = vec![0.0; self.n];
            let mut bq = vec![0.0; self.n];
            for (c, &yc) in y.iter().enumerate() {
                axpy(yc, &self.basis[c], &mut q);
                axpy(yc, &self.b_basis[c], &mut bq);
            }
            basis.push(q);
            b_basis.push(bq);
            t[new * stride + new] = thetas[i];
            // Coupling of the kept Ritz vector to the next vector: β y_i[k−1], with the
            // sign of the component.
            let s = residuals[i] * y[k - 1].signum();
            t[keep * stride + new] = s;
            t[new * stride + keep] = s;
        }
        self.basis = basis;
        self.b_basis = b_basis;
        self.t = t;
        self.restarts += 1;
        // `next` stays: the residual vector of the old basis, still B-orthogonal to the
        // kept Ritz vectors.
    }

    /// The `count` most wanted eigenpairs (plus the extra ones): locked vectors and Ritz
    /// vectors of the basis, `(θ, x)`.
    fn wanted(&self, count: usize, preference: impl Fn(f64) -> f64) -> Vec<(f64, Vec<f64>)> {
        let (mut thetas, ys, _) = if self.k() > 0 {
            self.ritz()
        } else {
            (Vec::new(), Vec::new(), Vec::new())
        };
        let ritz_count = thetas.len();
        thetas.extend_from_slice(&self.locked_theta);
        let order = Self::order(&thetas, &preference);
        let take = (count + self.extra_wanted).min(order.len());
        order
            .iter()
            .take(take)
            .map(|&i| {
                if i < ritz_count {
                    (thetas[i], self.vector(&ys[i]))
                } else {
                    (thetas[i], self.locked[i - ritz_count].clone())
                }
            })
            .collect()
    }

    /// `V y`.
    fn vector(&self, y: &[f64]) -> Vec<f64> {
        let mut x = vec![0.0; self.n];
        for (c, &yc) in y.iter().enumerate() {
            axpy(yc, &self.basis[c], &mut x);
        }
        x
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

fn axpy(alpha: f64, x: &[f64], y: &mut [f64]) {
    for (yi, xi) in y.iter_mut().zip(x) {
        *yi += alpha * xi;
    }
}

fn scale(alpha: f64, x: &mut [f64]) {
    for xi in x.iter_mut() {
        *xi *= alpha;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Pattern;
    use std::sync::Arc;

    /// A chain of `n` masses and springs fixed at both ends: K tridiagonal (2, -1), M = I·m.
    /// Eigenvalues (k/m)·(2 − 2 cos(jπ/(n+1))), each simple.
    fn chain(n: usize, spring: f64, mass: f64) -> (SparseMatrix, SparseMatrix) {
        let blocks: Vec<[u32; 2]> = (0..n as u32 - 1).map(|i| [i, i + 1]).collect();
        let pattern = Arc::new(Pattern::from_blocks(n, blocks.iter().map(|b| &b[..])));
        let mut k = SparseMatrix::zeros(pattern.clone());
        let mut m = SparseMatrix::zeros(pattern);
        for i in 0..n as u32 {
            k.add(i, i, 2.0 * spring);
            m.add(i, i, mass);
            if i + 1 < n as u32 {
                k.add(i, i + 1, -spring);
            }
        }
        (k, m)
    }

    fn exact_chain(n: usize, spring: f64, mass: f64) -> Vec<f64> {
        let mut v: Vec<f64> = (1..=n)
            .map(|j| {
                spring / mass
                    * (2.0 - 2.0 * (j as f64 * std::f64::consts::PI / (n as f64 + 1.0)).cos())
            })
            .collect();
        v.sort_by(f64::total_cmp);
        v
    }

    #[test]
    fn smallest_eigenvalues_of_a_chain() {
        let n = 300;
        let (k, m) = chain(n, 1000.0, 0.5);
        let exact = exact_chain(n, 1000.0, 0.5);
        let mut solver = ShiftInvertLanczos::new(1, LanczosOptions::default());
        let pairs = solver.frequencies(&k, &m, 8, 0.0).unwrap();
        assert_eq!(pairs.values.len(), 8);
        for (v, e) in pairs.values.iter().zip(&exact) {
            assert!((v - e).abs() <= 1e-9 * e, "{v} vs {e}");
        }
        assert!(pairs.sturm_confirmed);
        assert!(pairs.residual < 1e-8, "{}", pairs.residual);
        // Mass normalized.
        for x in &pairs.vectors {
            let mut mx = vec![0.0; n];
            m.multiply(x, &mut mx);
            assert!((dot(x, &mx) - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn a_shift_inside_the_spectrum_finds_the_nearest_eigenvalues() {
        let n = 200;
        let (k, m) = chain(n, 1.0, 1.0);
        let exact = exact_chain(n, 1.0, 1.0);
        // (Exactly 1.0 would make K − σM meet a zero pivot in the natural order, which
        // LDLᵀ without pivoting cannot pass; see the module documentation.)
        let shift = 1.05;
        let mut solver = ShiftInvertLanczos::new(1, LanczosOptions::default());
        let pairs = solver.frequencies(&k, &m, 6, shift).unwrap();
        let mut nearest = exact.clone();
        nearest.sort_by(|a, b| (a - shift).abs().total_cmp(&(b - shift).abs()));
        let mut nearest: Vec<f64> = nearest[..6].to_vec();
        nearest.sort_by(f64::total_cmp);
        for (v, e) in pairs.values.iter().zip(&nearest) {
            assert!((v - e).abs() <= 1e-9, "{v} vs {e}");
        }
        assert!(pairs.sturm_confirmed);
    }

    #[test]
    fn multiple_eigenvalues_are_all_found() {
        // Two identical chains side by side: every eigenvalue is double.
        let n = 60;
        let blocks: Vec<[u32; 2]> = (0..n as u32 - 1)
            .flat_map(|i| [[i, i + 1], [i + n as u32, i + n as u32 + 1]])
            .collect();
        let pattern = Arc::new(Pattern::from_blocks(2 * n, blocks.iter().map(|b| &b[..])));
        let mut k = SparseMatrix::zeros(pattern.clone());
        let mut m = SparseMatrix::zeros(pattern);
        for c in 0..2 {
            for i in 0..n as u32 {
                let g = i + c * n as u32;
                k.add(g, g, 2.0);
                m.add(g, g, 1.0);
                if i + 1 < n as u32 {
                    k.add(g, g + 1, -1.0);
                }
            }
        }
        let exact = exact_chain(n, 1.0, 1.0);
        let mut solver = ShiftInvertLanczos::new(1, LanczosOptions::default());
        let pairs = solver.frequencies(&k, &m, 7, 0.0).unwrap();
        let expected = [
            exact[0], exact[0], exact[1], exact[1], exact[2], exact[2], exact[3],
        ];
        for (v, e) in pairs.values.iter().zip(&expected) {
            assert!(
                (v - e).abs() <= 1e-9,
                "{:?} vs {:?}",
                pairs.values,
                expected
            );
        }
        assert!(pairs.sturm_confirmed);
        // The two vectors of a pair are M-orthogonal.
        let mut mx = vec![0.0; 2 * n];
        m.multiply(&pairs.vectors[0], &mut mx);
        assert!(dot(&pairs.vectors[1], &mx).abs() < 1e-8);
    }

    #[test]
    fn the_iterative_inner_solver_finds_the_same_eigenvalues() {
        let n = 300;
        let (k, m) = chain(n, 1000.0, 0.5);
        let exact = exact_chain(n, 1000.0, 0.5);
        let mut solver = ShiftInvertLanczos::new(1, LanczosOptions::default());
        solver.set_iterative(true);
        let pairs = solver.frequencies(&k, &m, 8, 0.0).unwrap();
        assert_eq!(pairs.values.len(), 8);
        for (v, e) in pairs.values.iter().zip(&exact) {
            assert!((v - e).abs() <= 1e-8 * e, "{v} vs {e}");
        }
        assert!(!pairs.sturm_confirmed);
        assert!(pairs.residual < 1e-7, "{}", pairs.residual);

        // Two identical chains: every eigenvalue double; the rounds with a fresh start
        // vector find the second copies without a Sturm count.
        let n = 60;
        let blocks: Vec<[u32; 2]> = (0..n as u32 - 1)
            .flat_map(|i| [[i, i + 1], [i + n as u32, i + n as u32 + 1]])
            .collect();
        let pattern = Arc::new(Pattern::from_blocks(2 * n, blocks.iter().map(|b| &b[..])));
        let mut k = SparseMatrix::zeros(pattern.clone());
        let mut m = SparseMatrix::zeros(pattern);
        for c in 0..2 {
            for i in 0..n as u32 {
                let g = i + c * n as u32;
                k.add(g, g, 2.0);
                m.add(g, g, 1.0);
                if i + 1 < n as u32 {
                    k.add(g, g + 1, -1.0);
                }
            }
        }
        let exact = exact_chain(n, 1.0, 1.0);
        let pairs = solver.frequencies(&k, &m, 7, 0.0).unwrap();
        let expected = [
            exact[0], exact[0], exact[1], exact[1], exact[2], exact[2], exact[3],
        ];
        for (v, e) in pairs.values.iter().zip(&expected) {
            assert!(
                (v - e).abs() <= 1e-8,
                "{:?} vs {:?}",
                pairs.values,
                expected
            );
        }

        // Buckling: the same factors as with the factorization.
        let n = 40;
        let (k, _) = chain(n, 100.0, 1.0);
        let mut k_sigma = k.clone();
        k_sigma.fill_zero();
        for i in 0..n as u32 {
            k_sigma.add(i, i, -(1.0 + i as f64 / n as f64));
        }
        let direct = ShiftInvertLanczos::new(1, LanczosOptions::default())
            .buckling(&k, &k_sigma, 4)
            .unwrap();
        let iterative = solver.buckling(&k, &k_sigma, 4).unwrap();
        for (a, b) in iterative.values.iter().zip(&direct.values) {
            assert!((a - b).abs() <= 1e-8 * b.abs(), "{a} vs {b}");
        }
    }

    #[test]
    fn buckling_of_a_discrete_column() {
        // K = chain stiffness, K_σ = −(chain with spring 1)·s: (K − λ s K_1) x = 0 has
        // λ_j = spring_K / s (both tridiagonal with the same eigenvectors): all factors equal
        // the ratio when both chains have the same stiffness pattern. Use different end
        // conditions instead: K_σ = −diag(1..n)/n·s gives distinct factors found by a dense
        // reference.
        let n = 40;
        let (k, _) = chain(n, 100.0, 1.0);
        let mut k_sigma = k.clone();
        k_sigma.fill_zero();
        for i in 0..n as u32 {
            k_sigma.add(i, i, -(1.0 + i as f64 / n as f64));
        }
        // Dense reference: eigenvalues of K + λ K_σ singular <=> K x = λ (−K_σ) x.
        let mut kd = faer::Mat::<f64>::zeros(n, n);
        for i in 0..n {
            for j in 0..n {
                kd[(i, j)] = k.get(i as u32, j as u32);
            }
        }
        let mut d_inv_sqrt = faer::Mat::<f64>::zeros(n, n);
        for i in 0..n {
            d_inv_sqrt[(i, i)] = 1.0 / (-k_sigma.get(i as u32, i as u32)).sqrt();
        }
        let scaled = &d_inv_sqrt * &kd * &d_inv_sqrt;
        let mut exact = scaled.self_adjoint_eigenvalues(faer::Side::Lower).unwrap();
        exact.sort_by(f64::total_cmp);
        let mut solver = ShiftInvertLanczos::new(1, LanczosOptions::default());
        let pairs = solver.buckling(&k, &k_sigma, 4).unwrap();
        for (v, e) in pairs.values.iter().zip(&exact) {
            assert!(
                (v - e).abs() <= 1e-8 * e,
                "{:?} vs {:?}",
                pairs.values,
                &exact[..4]
            );
        }
        assert!(pairs.sturm_confirmed);
        assert!(pairs.residual < 1e-8);
    }
}
