//! The iterative solver: conjugate gradients preconditioned by an incomplete Cholesky
//! factorization without fill, IC(0), for symmetric positive definite systems.
//!
//! It needs a fraction of the memory of the direct solver (the matrix twice and one
//! triangle of the same pattern, no fill), which is what it is for: models whose factor
//! would not fit. It solves only positive definite systems; contact, eigenvalue problems
//! and unsymmetric tangents stay with the direct solver.
//!
//! Sources: Hestenes and Stiefel's conjugate gradient method in the preconditioned form of
//! Saad, "Iterative Methods for Sparse Linear Systems", 2nd ed., algorithm 9.1; IC(0) as in
//! Saad section 10.3 (row-wise, the pattern of the lower triangle of the matrix); the
//! diagonal shift on breakdown after Manteuffel, "An incomplete factorization technique for
//! positive definite linear systems", Math. Comp. 34 (1980). No CalculiX code was consulted.
//!
//! The matrix-vector product and the vector operations run in parallel on the current
//! rayon pool; the triangular solves of the preconditioner are sequential (each row needs
//! the ones before it).

use rayon::prelude::*;

use plxs_core::{Error, Result};

use crate::solver::LinearSolver;
use crate::{Pattern, SparseMatrix};

/// The iteration stops when `|r| <= TOLERANCE |b|` (Euclidean norms).
pub const TOLERANCE: f64 = 1e-12;

/// Vectors shorter than this are handled by one thread.
const PARALLEL_FROM: usize = 8192;

/// How the last solve went.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PcgStats {
    pub iterations: usize,
    /// `|r| / |b|` at the end.
    pub relative_residual: f64,
}

/// Preconditioned conjugate gradients with an IC(0) preconditioner.
#[derive(Default)]
pub struct PcgSolver {
    /// The pattern of the last `analyze`, the upper triangle in columns, which read as rows
    /// is the lower triangle: row `i` holds the columns `k <= i`, the diagonal last.
    pattern: Option<std::sync::Arc<Pattern>>,
    /// The whole matrix in rows, for the product: row starts, columns, and for each entry
    /// its position in the values of the upper triangle.
    full_ptr: Vec<u32>,
    full_col: Vec<u32>,
    full_src: Vec<u32>,
    full_val: Vec<f64>,
    /// The incomplete factor `L` on the pattern of the lower triangle (`A ≈ L Lᵀ`).
    factor: Vec<f64>,
    /// The relative diagonal shift the factorization needed (0 when none).
    shift: f64,
    factorized: bool,
    max_iterations: Option<usize>,
}

impl PcgSolver {
    pub fn new() -> Self {
        Self::default()
    }

    /// Limits the iterations of a solve (default: the dimension, at least 1000, at most
    /// 100 000).
    pub fn set_max_iterations(&mut self, max: usize) {
        self.max_iterations = Some(max);
    }

    /// Entries of the preconditioner after `analyze`, its memory in values.
    pub fn preconditioner_nonzeros(&self) -> Option<usize> {
        self.pattern.as_ref().map(|p| p.nnz())
    }

    /// The relative diagonal shift `α` of the last factorization: the preconditioner is
    /// the incomplete factor of `A + α diag(A)`; 0 when it was not needed.
    pub fn shift(&self) -> f64 {
        self.shift
    }

    /// Solves `A x = b` in place and tells how many iterations it took.
    pub fn solve_counted(&self, rhs: &mut [f64]) -> Result<PcgStats> {
        let pattern = (self.pattern.as_ref())
            .filter(|_| self.factorized)
            .ok_or_else(|| numerical("solve before factorize"))?;
        let n = pattern.dimension();
        assert_eq!(rhs.len(), n, "right-hand side of the wrong size");
        let b = rhs.to_vec();
        let b_norm = norm(&b);
        rhs.iter_mut().for_each(|x| *x = 0.0);
        if b_norm == 0.0 {
            return Ok(PcgStats {
                iterations: 0,
                relative_residual: 0.0,
            });
        }
        if !b_norm.is_finite() {
            return Err(numerical("the right-hand side is not finite"));
        }
        let max_iterations = self
            .max_iterations
            .unwrap_or_else(|| n.clamp(1000, 100_000));
        let x = rhs;
        let mut r = b;
        let mut z = vec![0.0; n];
        self.precondition(&r, &mut z);
        let mut p = z.clone();
        let mut q = vec![0.0; n];
        let mut rz = dot(&r, &z);
        let mut residual = 1.0;
        for iteration in 1..=max_iterations {
            self.product(&p, &mut q);
            let pq = dot(&p, &q);
            if pq <= 0.0 || !pq.is_finite() {
                return Err(numerical(
                    "the matrix is not positive definite, which the iterative solver needs; \
                     choose the direct equation solver",
                ));
            }
            let alpha = rz / pq;
            axpy(alpha, &p, x);
            axpy(-alpha, &q, &mut r);
            residual = norm(&r) / b_norm;
            if residual <= TOLERANCE {
                return Ok(PcgStats {
                    iterations: iteration,
                    relative_residual: residual,
                });
            }
            self.precondition(&r, &mut z);
            let rz_new = dot(&r, &z);
            let beta = rz_new / rz;
            rz = rz_new;
            // p = z + beta p
            zip_mut(&mut p, &z, |p, z| *p = z + beta * *p);
        }
        Err(numerical(format!(
            "the iterative solver did not converge in {max_iterations} iterations \
             (relative residual {residual:.3e}); choose the direct equation solver"
        )))
    }

    /// `y = A x` with the whole matrix in rows, rows in parallel.
    fn product(&self, x: &[f64], y: &mut [f64]) {
        let row = |(i, y): (usize, &mut f64)| {
            let (start, end) = (self.full_ptr[i] as usize, self.full_ptr[i + 1] as usize);
            *y = (self.full_col[start..end].iter())
                .zip(&self.full_val[start..end])
                .map(|(&c, &a)| a * x[c as usize])
                .sum();
        };
        if y.len() < PARALLEL_FROM {
            y.iter_mut().enumerate().for_each(row);
        } else {
            y.par_iter_mut()
                .enumerate()
                .with_min_len(1024)
                .for_each(row);
        }
    }

    /// `z = (L Lᵀ)⁻¹ r`.
    fn precondition(&self, r: &[f64], z: &mut [f64]) {
        let pattern = self.pattern.as_ref().expect("analyzed");
        let (ptr, col, l) = (pattern.col_ptr(), pattern.row_idx(), &self.factor);
        let n = pattern.dimension();
        // L y = r, row by row; the diagonal is the last entry of each row.
        for i in 0..n {
            let (start, end) = (ptr[i] as usize, ptr[i + 1] as usize);
            let mut s = r[i];
            for p in start..end - 1 {
                s -= l[p] * z[col[p] as usize];
            }
            z[i] = s / l[end - 1];
        }
        // Lᵀ z = y, rows of L read as columns of Lᵀ, from the last.
        for i in (0..n).rev() {
            let (start, end) = (ptr[i] as usize, ptr[i + 1] as usize);
            let zi = z[i] / l[end - 1];
            z[i] = zi;
            for p in start..end - 1 {
                z[col[p] as usize] -= l[p] * zi;
            }
        }
    }

    /// IC(0) of `A + shift diag(A)`; `false` on a pivot that is not positive.
    fn incomplete_cholesky(&mut self, values: &[f64], shift: f64) -> bool {
        let pattern = self.pattern.as_ref().expect("analyzed");
        let (ptr, col) = (pattern.col_ptr(), pattern.row_idx());
        let n = pattern.dimension();
        let l = &mut self.factor;
        l.copy_from_slice(values);
        // Row i of L scattered into a dense work vector while it is computed.
        let mut work = vec![0.0; n];
        for i in 0..n {
            let (start, end) = (ptr[i] as usize, ptr[i + 1] as usize);
            let diagonal = end - 1;
            debug_assert_eq!(col[diagonal] as usize, i);
            for p in start..diagonal {
                let k = col[p] as usize;
                let (k_start, k_end) = (ptr[k] as usize, ptr[k + 1] as usize);
                // L[i,k] = (A[i,k] - Σ_{m<k} L[i,m] L[k,m]) / L[k,k]; work holds L[i,m]
                // for the m < k computed so far and zero elsewhere.
                let mut s = l[p];
                for q in k_start..k_end - 1 {
                    s -= l[q] * work[col[q] as usize];
                }
                let value = s / l[k_end - 1];
                l[p] = value;
                work[k] = value;
            }
            let mut d = l[diagonal] * (1.0 + shift);
            for p in start..diagonal {
                d -= l[p] * l[p];
                work[col[p] as usize] = 0.0;
            }
            if d <= 0.0 || !d.is_finite() {
                return false;
            }
            l[diagonal] = d.sqrt();
        }
        true
    }
}

impl LinearSolver for PcgSolver {
    fn analyze(&mut self, pattern: &Pattern) -> Result<()> {
        if self.pattern.as_deref() == Some(pattern) {
            // The Newton iterations of a step keep their pattern.
            return Ok(());
        }
        let n = pattern.dimension();
        let (ptr, rows) = (pattern.col_ptr(), pattern.row_idx());
        // Entries per row of the whole matrix: column j of the upper triangle gives row j
        // (its lower part) and, mirrored, one entry in each row i < j.
        let mut count = vec![0u32; n + 1];
        for j in 0..n {
            for &i in &rows[ptr[j] as usize..ptr[j + 1] as usize] {
                count[j + 1] += 1;
                if i as usize != j {
                    count[i as usize + 1] += 1;
                }
            }
        }
        for i in 0..n {
            count[i + 1] += count[i];
        }
        let nnz = count[n] as usize;
        let mut next = count.clone();
        let mut full_col = vec![0u32; nnz];
        let mut full_src = vec![0u32; nnz];
        for j in 0..n {
            let (start, end) = (ptr[j] as usize, ptr[j + 1] as usize);
            for (k, &i) in rows.iter().enumerate().take(end).skip(start) {
                let i = i as usize;
                let at = next[j] as usize;
                full_col[at] = i as u32;
                full_src[at] = k as u32;
                next[j] += 1;
                if i != j {
                    let at = next[i] as usize;
                    full_col[at] = j as u32;
                    full_src[at] = k as u32;
                    next[i] += 1;
                }
            }
        }
        self.full_ptr = count;
        self.full_col = full_col;
        self.full_src = full_src;
        self.full_val = vec![0.0; nnz];
        self.factor = vec![0.0; pattern.nnz()];
        self.pattern = Some(std::sync::Arc::new(pattern.clone()));
        self.factorized = false;
        Ok(())
    }

    fn factorize(&mut self, matrix: &SparseMatrix) -> Result<()> {
        let pattern =
            (self.pattern.as_ref()).ok_or_else(|| numerical("factorize before analyze"))?;
        if **matrix.pattern() != **pattern {
            return Err(numerical("factorize with another pattern than analyzed"));
        }
        let values = matrix.values();
        let n = pattern.dimension();
        for i in 0..n {
            let diagonal = pattern.col_ptr()[i + 1] as usize - 1;
            if values[diagonal] <= 0.0 || !values[diagonal].is_finite() {
                return Err(numerical(format!(
                    "the matrix is not positive definite (diagonal entry {} is {:.3e}), \
                     which the iterative solver needs",
                    i + 1,
                    values[diagonal]
                )));
            }
        }
        let src = &self.full_src;
        self.full_val
            .par_iter_mut()
            .zip(src.par_iter())
            .with_min_len(4096)
            .for_each(|(v, &s)| *v = values[s as usize]);
        // A positive definite matrix can still break IC(0) down; shift the diagonal until
        // it does not (Manteuffel). Without a usable factor fall back to the diagonal.
        let mut shift = 0.0;
        let mut factored = self.incomplete_cholesky(values, shift);
        let mut next = 1e-3;
        while !factored && next < 2.0 {
            shift = next;
            factored = self.incomplete_cholesky(values, shift);
            next *= 4.0;
        }
        self.shift = shift;
        if !factored {
            // Jacobi: L = sqrt(diag(A)), off-diagonal entries zero.
            let pattern = self.pattern.as_ref().expect("analyzed");
            self.factor.iter_mut().for_each(|v| *v = 0.0);
            for i in 0..n {
                let diagonal = pattern.col_ptr()[i + 1] as usize - 1;
                self.factor[diagonal] = values[diagonal].sqrt();
            }
            self.shift = f64::INFINITY;
        }
        self.factorized = true;
        Ok(())
    }

    fn solve(&self, rhs: &mut [f64]) -> Result<()> {
        self.solve_counted(rhs).map(|_| ())
    }

    fn inertia(&self) -> Option<usize> {
        None
    }
}

fn numerical(message: impl Into<String>) -> Error {
    Error::Numerical {
        step: 0,
        increment: 0,
        message: message.into(),
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    if a.len() < PARALLEL_FROM {
        a.iter().zip(b).map(|(x, y)| x * y).sum()
    } else {
        (a.par_chunks(4096).zip(b.par_chunks(4096)))
            .map(|(a, b)| a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>())
            .sum()
    }
}

fn norm(a: &[f64]) -> f64 {
    dot(a, a).sqrt()
}

/// `y += alpha x`.
fn axpy(alpha: f64, x: &[f64], y: &mut [f64]) {
    zip_mut(y, x, |y, x| *y += alpha * x);
}

fn zip_mut(a: &mut [f64], b: &[f64], f: impl Fn(&mut f64, f64) + Sync + Send) {
    if a.len() < PARALLEL_FROM {
        a.iter_mut().zip(b).for_each(|(a, &b)| f(a, b));
    } else {
        (a.par_chunks_mut(4096).zip(b.par_chunks(4096)))
            .for_each(|(a, b)| a.iter_mut().zip(b).for_each(|(a, &b)| f(a, b)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LdltSolver;
    use std::sync::Arc;

    /// The stiffness of a chain of `n` springs fixed at the start, with every node also
    /// tied to the one two places on, so that the pattern has more than a band of one.
    fn chain(n: usize) -> SparseMatrix {
        let blocks: Vec<[u32; 2]> = (0..n as u32 - 1)
            .map(|i| [i, i + 1])
            .chain((0..n as u32 - 2).map(|i| [i, i + 2]))
            .collect();
        let pattern = Arc::new(Pattern::from_blocks(n, blocks.iter().map(|b| &b[..])));
        let mut a = SparseMatrix::zeros(pattern);
        for (e, block) in blocks.iter().enumerate() {
            let k = 1.0 + (e % 7) as f64;
            a.add_block(block, &[k, -k, -k, k]);
        }
        a.add(0, 0, 10.0);
        a
    }

    #[test]
    fn pcg_matches_the_direct_solver() {
        for n in [5, 300, 20_000] {
            let a = chain(n);
            let b: Vec<f64> = (0..n).map(|i| ((i * 37) % 11) as f64 - 5.0).collect();
            let mut direct = LdltSolver::new(1);
            direct.analyze(a.pattern()).unwrap();
            direct.factorize(&a).unwrap();
            let mut expected = b.clone();
            direct.solve(&mut expected).unwrap();

            let mut pcg = PcgSolver::new();
            pcg.analyze(a.pattern()).unwrap();
            pcg.factorize(&a).unwrap();
            let mut x = b.clone();
            let stats = pcg.solve_counted(&mut x).unwrap();
            assert!(stats.relative_residual <= TOLERANCE, "{stats:?}");
            let scale = expected.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            for (x, e) in x.iter().zip(&expected) {
                assert!((x - e).abs() <= 1e-7 * scale, "n = {n}: {x} vs {e}");
            }
        }
    }

    #[test]
    fn ic0_of_a_tridiagonal_matrix_is_exact() {
        // No fill in a tridiagonal factor, so IC(0) is the Cholesky factor and PCG
        // converges in one iteration.
        let n = 50;
        let blocks: Vec<[u32; 2]> = (0..n as u32 - 1).map(|i| [i, i + 1]).collect();
        let pattern = Arc::new(Pattern::from_blocks(n, blocks.iter().map(|b| &b[..])));
        let mut a = SparseMatrix::zeros(pattern);
        for block in &blocks {
            a.add_block(block, &[2.0, -1.0, -1.0, 2.0]);
        }
        let mut pcg = PcgSolver::new();
        pcg.analyze(a.pattern()).unwrap();
        pcg.factorize(&a).unwrap();
        assert_eq!(pcg.shift(), 0.0);
        let mut x = vec![1.0; n];
        let stats = pcg.solve_counted(&mut x).unwrap();
        assert_eq!(stats.iterations, 1, "{stats:?}");
    }

    #[test]
    fn indefinite_matrices_are_refused() {
        let pattern = Arc::new(Pattern::from_blocks(2, [&[0u32, 1][..]]));
        let mut a = SparseMatrix::zeros(pattern);
        a.add(0, 0, 1.0);
        a.add(0, 1, 2.0);
        a.add(1, 1, 1.0);
        let mut pcg = PcgSolver::new();
        pcg.analyze(a.pattern()).unwrap();
        // IC(0) breaks down at every shift tried; the diagonal preconditioner then lets CG
        // see the negative curvature.
        pcg.factorize(&a).unwrap();
        let mut x = vec![1.0, -1.0];
        let error = pcg.solve_counted(&mut x).unwrap_err().to_string();
        assert!(error.contains("not positive definite"), "{error}");

        a.add(1, 1, -3.0);
        assert!(pcg.factorize(&a).is_err());
    }

    #[test]
    fn a_zero_load_gives_zero_without_iterating() {
        let a = chain(10);
        let mut pcg = PcgSolver::new();
        pcg.analyze(a.pattern()).unwrap();
        pcg.factorize(&a).unwrap();
        let mut x = vec![0.0; 10];
        assert_eq!(pcg.solve_counted(&mut x).unwrap().iterations, 0);
    }
}
