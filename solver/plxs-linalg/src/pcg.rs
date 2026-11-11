//! The iterative solver: conjugate gradients for symmetric positive definite systems,
//! preconditioned by algebraic multigrid ([`crate::amg`], the default) or by an incomplete
//! Cholesky factorization without fill, IC(0).
//!
//! It needs a fraction of the memory of the direct solver (the matrix in full storage and
//! a multigrid hierarchy of a fraction of its size), which is what it is for: models
//! whose factor would not fit. It solves only positive definite systems; eigenvalue
//! problems and unsymmetric tangents stay with the direct solver.
//!
//! Sources: Hestenes and Stiefel's conjugate gradient method in the preconditioned form of
//! Saad, "Iterative Methods for Sparse Linear Systems", 2nd ed., algorithm 9.1; IC(0) as in
//! Saad section 10.3 (row-wise, the pattern of the lower triangle of the matrix); the
//! diagonal shift on breakdown after Manteuffel, "An incomplete factorization technique for
//! positive definite linear systems", Math. Comp. 34 (1980). No CalculiX code was consulted.
//!
//! The matrix-vector product, the vector operations and the multigrid cycle run in
//! parallel on the current rayon pool; the triangular solves of IC(0) are sequential (each
//! row needs the ones before it). `PLXS_PRECONDITIONER=ic0|amg` chooses for measurements.

use std::sync::atomic::{AtomicUsize, Ordering};

use rayon::prelude::*;

use plxs_core::{Error, Result};

use crate::amg::{Amg, AmgOptions, Unknowns};
use crate::bsr::Bsr;
use crate::solver::LinearSolver;
use crate::{Pattern, SparseMatrix};

/// The iteration stops when `|r| <= TOLERANCE |b|` (Euclidean norms).
pub const TOLERANCE: f64 = 1e-12;
/// `PLXS_AMG_REBUILD=1` rebuilds the hierarchy at every factorization (measurements).
fn reuse_disabled() -> bool {
    std::env::var("PLXS_AMG_REBUILD").is_ok()
}

/// Vectors shorter than this are handled by one thread.
const PARALLEL_FROM: usize = 8192;

/// Systems of at least this many unknowns are stored in blocks when the unknowns are
/// known; the scalar storage is simpler for the small ones.
const BLOCKED_FROM: usize = 3000;

/// How the last solve went.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PcgStats {
    pub iterations: usize,
    /// `|r| / |b|` at the end.
    pub relative_residual: f64,
}

/// Which preconditioner the solver builds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Preconditioner {
    /// Smoothed aggregation multigrid (one V-cycle).
    Multigrid,
    /// Incomplete Cholesky without fill.
    IncompleteCholesky,
}

impl Preconditioner {
    /// The default, or what `PLXS_PRECONDITIONER` asks for.
    fn from_environment() -> Self {
        match std::env::var("PLXS_PRECONDITIONER").as_deref() {
            Ok("ic0") | Ok("IC0") => Self::IncompleteCholesky,
            _ => Self::Multigrid,
        }
    }
}

/// Preconditioned conjugate gradients.
pub struct PcgSolver {
    /// The pattern of the last `analyze`, the upper triangle in columns, which read as rows
    /// is the lower triangle: row `i` holds the columns `k <= i`, the diagonal last.
    pattern: Option<std::sync::Arc<Pattern>>,
    /// The whole matrix in rows (scalar, or in `3 × 3` blocks padded to three unknowns per
    /// node when the unknowns are known), for the product, and for each of its stored
    /// values the position in the values of the upper triangle (`u32::MAX`: a padding
    /// entry, fixed at analysis).
    full: Bsr,
    full_src: Vec<u32>,
    /// Blocked storage: the padded index of every unknown, and the padded dimension.
    slot_of: Vec<u32>,
    padded: usize,
    /// The unknowns the storage was analyzed for.
    analyzed_unknowns: Option<Unknowns>,
    choice: Preconditioner,
    /// The multigrid hierarchy of the last factorization.
    amg: Option<Amg>,
    /// What the unknowns are (nodes, components, positions), for the aggregation and the
    /// rigid body modes of the multigrid; without it the hierarchy is scalar.
    unknowns: Option<Unknowns>,
    /// The incomplete factor `L` on the pattern of the lower triangle (`A ≈ L Lᵀ`).
    factor: Vec<f64>,
    /// The relative diagonal shift the factorization needed (0 when none).
    shift: f64,
    factorized: bool,
    max_iterations: Option<usize>,
    /// Whether a solve gives up early when the residual stalls ([`Stall`]): when another
    /// solver stands by.
    give_up_on_stall: bool,
    /// Iterations of the first solve after the hierarchy was built, and of the last
    /// solve (0: none yet); a factorization keeps the hierarchy while the count stays
    /// reasonable.
    iterations_after_build: AtomicUsize,
    last_iterations: AtomicUsize,
    /// The seconds the hierarchy took to build and the seconds per iteration of the last
    /// solve (as bits of an `f64`): what a stale hierarchy costs against a new one.
    build_seconds: f64,
    seconds_per_iteration: std::sync::atomic::AtomicU64,
    /// Whether the last factorization kept the hierarchy of the one before.
    reused: bool,
    /// The diagonal of the matrix the hierarchy was built on (padded numbering).
    built_diagonal: Vec<f64>,
    /// The slow modes of the preconditioned operator learned from the first solve after
    /// the hierarchy was built, deflated from the solves after it.
    deflation: std::sync::Mutex<Option<Deflation>>,
    /// The slowest modes of the first solve with a hierarchy of the rigid body modes
    /// alone: near-nullspace candidates of every later hierarchy of this pattern. `true`
    /// while a hierarchy without them is in use and they are to go into the next build.
    candidates: std::sync::Mutex<Vec<Vec<f64>>>,
    candidates_pending: std::sync::atomic::AtomicBool,
}

impl Default for PcgSolver {
    fn default() -> Self {
        Self {
            pattern: None,
            full: Bsr::default(),
            full_src: Vec::new(),
            slot_of: Vec::new(),
            padded: 0,
            analyzed_unknowns: None,
            choice: Preconditioner::from_environment(),
            amg: None,
            unknowns: None,
            factor: Vec::new(),
            shift: 0.0,
            factorized: false,
            max_iterations: None,
            give_up_on_stall: false,
            iterations_after_build: AtomicUsize::new(0),
            last_iterations: AtomicUsize::new(0),
            build_seconds: 0.0,
            seconds_per_iteration: std::sync::atomic::AtomicU64::new(0),
            reused: false,
            built_diagonal: Vec::new(),
            deflation: std::sync::Mutex::new(None),
            candidates: std::sync::Mutex::new(Vec::new()),
            candidates_pending: std::sync::atomic::AtomicBool::new(false),
        }
    }
}

impl PcgSolver {
    pub fn new() -> Self {
        Self::default()
    }

    /// Chooses the preconditioner (default: multigrid, or `PLXS_PRECONDITIONER`).
    pub fn set_preconditioner(&mut self, choice: Preconditioner) {
        self.choice = choice;
    }

    /// Tells the solver what the unknowns of the next systems are, before `analyze`;
    /// `None` makes the storage and the multigrid scalar (every unknown its own block, the
    /// constant as nullspace).
    pub fn set_unknowns(&mut self, unknowns: Option<Unknowns>) {
        self.unknowns = unknowns;
    }

    /// Whether the preconditioner is the hierarchy of an earlier factorization, refreshed
    /// for the current values (see [`LinearSolver::factorize`]).
    pub fn is_reused(&self) -> bool {
        self.reused
    }

    /// Builds the multigrid hierarchy anew for the current values: after a solve with a
    /// kept hierarchy that did not converge. Fails like a factorization when no hierarchy
    /// can be built.
    pub fn rebuild_preconditioner(&mut self) -> Result<()> {
        if self.choice != Preconditioner::Multigrid || !self.factorized {
            return Ok(());
        }
        self.build_multigrid()
    }

    /// The hierarchy of the current values of `full`.
    fn build_multigrid(&mut self) -> Result<()> {
        self.amg = None;
        *self.deflation.get_mut().unwrap_or_else(|e| e.into_inner()) = None;
        self.reused = false;
        self.iterations_after_build.store(0, Ordering::Relaxed);
        self.built_diagonal = self.full.diagonal();
        let unknowns = self.padded_unknowns();
        let started = std::time::Instant::now();
        let candidates =
            std::mem::take(self.candidates.get_mut().unwrap_or_else(|e| e.into_inner()));
        self.candidates_pending.store(false, Ordering::Relaxed);
        #[allow(unused_mut)]
        let mut amg = Amg::build(
            &self.full,
            unknowns.as_ref(),
            &candidates,
            AmgOptions::default(),
        );
        *self.candidates.get_mut().unwrap_or_else(|e| e.into_inner()) = candidates;
        #[allow(unused_mut)]
        let mut amg = amg?;
        #[cfg(feature = "gpu")]
        amg.attach_operator(&self.full);
        self.build_seconds = started.elapsed().as_secs_f64();
        self.amg = Some(amg);
        self.factorized = true;
        Ok(())
    }

    /// The deflation for the solves to come, from the Lanczos vectors `lanczos` of a
    /// finished solve and its tridiagonal: the Ritz vectors of the Ritz values below
    /// [`DEFLATE_BELOW`], at most [`DEFLATE_MAX`] of them; `None` when there are none.
    fn learn_deflation(
        &self,
        ritz: &RitzTrace,
        lanczos: &[Vec<f32>],
        trace: bool,
    ) -> Option<Deflation> {
        let m = ritz.alphas.len().min(lanczos.len());
        if m < 2 {
            return None;
        }
        let (diag, off) = ritz.tridiagonal();
        let (values, vectors) = tridiagonal_eigen(&diag[..m], &off[..m]);
        let largest = values.iter().copied().fold(0.0_f64, f64::max);
        let chosen: Vec<usize> = (0..m)
            .filter(|&j| values[j] > 0.0 && values[j] < DEFLATE_BELOW * largest)
            .take(DEFLATE_MAX)
            .collect();
        if chosen.is_empty() {
            return None;
        }
        let n = self.padded;
        let d = chosen.len();
        let mut w = vec![0.0; d * n];
        for (slot, &j) in chosen.iter().enumerate() {
            let y = &vectors[j * m..(j + 1) * m];
            let out = &mut w[slot * n..(slot + 1) * n];
            for (k, v) in lanczos.iter().take(m).enumerate() {
                let c = y[k];
                if c != 0.0 {
                    out.par_chunks_mut(4096)
                        .zip(v.par_chunks(4096))
                        .for_each(|(o, v)| {
                            o.iter_mut().zip(v).for_each(|(o, &v)| *o += c * v as f64)
                        });
                }
            }
            let norm = norm(out);
            if norm > 0.0 {
                out.iter_mut().for_each(|v| *v /= norm);
            }
        }
        // Lanczos without reorthogonalization loses its orthogonality once a Ritz value
        // converges and then produces copies of it (a buckling matrix near its shift: pairs
        // of Ritz values 1.5e-5, 1.5e-5), and copies make `E` singular. Orthonormalize
        // (modified Gram-Schmidt) and drop what is spanned already.
        let mut basis: Vec<f64> = Vec::with_capacity(d * n);
        let mut kept = 0;
        for j in 0..d {
            let mut col = w[j * n..(j + 1) * n].to_vec();
            for i in 0..kept {
                let q = &basis[i * n..(i + 1) * n];
                let c = dot(q, &col);
                axpy(-c, q, &mut col);
            }
            let remaining = norm(&col);
            if remaining < DEFLATE_INDEPENDENT {
                continue;
            }
            col.iter_mut().for_each(|v| *v /= remaining);
            basis.extend_from_slice(&col);
            kept += 1;
        }
        if kept == 0 {
            return None;
        }
        let (d, w) = (kept, basis);
        let mut defl = Deflation {
            d,
            w,
            aw: vec![0.0; d * n],
            chol: Vec::new(),
        };
        if !defl.refresh(self) {
            return None;
        }
        if trace {
            let shown: Vec<String> = chosen
                .iter()
                .map(|&j| format!("{:.2e}", values[j]))
                .collect();
            eprintln!(
                "pcg: deflating {d} modes with ritz values {}",
                shown.join(" ")
            );
        }
        Some(defl)
    }

    /// Whether the matrix is stored in `3 × 3` blocks.
    pub fn is_blocked(&self) -> bool {
        self.full.br == 3
    }

    /// A line describing the preconditioner of the last factorization.
    pub fn preconditioner(&self) -> String {
        match &self.amg {
            Some(amg) => {
                let sizes = amg.sizes();
                let levels: Vec<String> = sizes.iter().map(|n| n.to_string()).collect();
                format!(
                    "algebraic multigrid in single precision, {} levels ({}), \
                     {:.2} x the matrix{}",
                    amg.levels(),
                    levels.join(" > "),
                    // Memory: single precision values against the double precision matrix.
                    amg.nonzeros() as f64 / (2 * self.full.nnz().max(1)) as f64,
                    if self.reused { ", kept" } else { "" }
                ) + &amg
                    .placement()
                    .map(|p| format!(", {p}"))
                    .unwrap_or_default()
            }
            None if self.shift.is_infinite() => {
                "diagonal (incomplete Cholesky broke down)".to_owned()
            }
            None if self.shift > 0.0 => {
                format!(
                    "incomplete Cholesky, diagonal shifted by {:.0e}",
                    self.shift
                )
            }
            None => "incomplete Cholesky".to_owned(),
        }
    }

    /// Limits the iterations of a solve (default: the dimension, at least 1000, at most
    /// 100 000).
    pub fn set_max_iterations(&mut self, max: usize) {
        self.max_iterations = Some(max);
    }

    /// Gives a solve up as soon as the residual stalls ([`Stall`]) instead of iterating
    /// up to the limit: for a caller that has another solver to fall back on.
    pub fn set_give_up_on_stall(&mut self, give_up: bool) {
        self.give_up_on_stall = give_up;
    }

    /// Entries of the preconditioner, its memory in values.
    pub fn preconditioner_nonzeros(&self) -> Option<usize> {
        match &self.amg {
            Some(amg) => Some(amg.nonzeros()),
            None => self.pattern.as_ref().map(|p| p.nnz()),
        }
    }

    /// The relative diagonal shift `α` of the last factorization: the preconditioner is
    /// the incomplete factor of `A + α diag(A)`; 0 when it was not needed.
    pub fn shift(&self) -> f64 {
        self.shift
    }

    /// Solves `A x = b` in place and tells how many iterations it took.
    pub fn solve_counted(&self, rhs: &mut [f64]) -> Result<PcgStats> {
        self.iterate(rhs, None)
    }

    /// `x ≈ A⁻¹ b` in place, to `|r| <= tolerance |b|` or after `max_iterations`,
    /// whichever comes first: a preconditioner for an iteration outside (GMRES on the
    /// unsymmetric tangent of sliding friction), which needs only a few digits. Fails only
    /// when the matrix turns out indefinite or the iteration produces no finite numbers,
    /// never for missing the tolerance; the stats tell what it reached.
    pub fn solve_loosely(
        &self,
        rhs: &mut [f64],
        tolerance: f64,
        max_iterations: usize,
    ) -> Result<PcgStats> {
        self.iterate(rhs, Some((tolerance, max_iterations)))
    }

    /// The conjugate gradients: to [`TOLERANCE`] within the solver's iteration limit
    /// (`loose: None`, failing beyond it), or to `loose`'s tolerance and limit, keeping
    /// the iterate it ends with.
    fn iterate(&self, rhs: &mut [f64], loose: Option<(f64, usize)>) -> Result<PcgStats> {
        let pattern = (self.pattern.as_ref())
            .filter(|_| self.factorized)
            .ok_or_else(|| numerical(plxs_core::Failure::Internal, "solve before factorize"))?;
        assert_eq!(
            rhs.len(),
            pattern.dimension(),
            "right-hand side of the wrong size"
        );
        let n = self.padded;
        let b = self.pad(rhs);
        let b_norm = norm(&b);
        rhs.iter_mut().for_each(|x| *x = 0.0);
        if b_norm == 0.0 {
            return Ok(PcgStats {
                iterations: 0,
                relative_residual: 0.0,
            });
        }
        if !b_norm.is_finite() {
            return Err(numerical(
                plxs_core::Failure::IterativeSolver,
                "the right-hand side is not finite",
            ));
        }
        let mut max_iterations = self
            .max_iterations
            .unwrap_or_else(|| n.clamp(1000, 100_000));
        let tolerance = loose.map_or(TOLERANCE, |(tolerance, _)| tolerance);
        if let Some((_, limit)) = loose {
            max_iterations = limit.max(1);
        } else if self.reused {
            // A kept hierarchy that no longer fits its matrix shows soon: give up early,
            // the caller rebuilds it ([`Self::rebuild_preconditioner`]).
            let after_build = self.iterations_after_build.load(Ordering::Relaxed);
            max_iterations = max_iterations.min((3 * after_build).max(100));
        }
        let trace = std::env::var("PLXS_PCG_TRACE").is_ok();
        let cancel = plxs_core::CancelToken::current();
        // The first solve with a fresh hierarchy of a pattern learns the near-nullspace
        // candidates of the next hierarchy from its Lanczos vectors (see [`Deflation`]),
        // which only the processor's loop collects.
        #[cfg(feature = "gpu")]
        let learning = loose.is_none()
            && self.amg.is_some()
            && candidate_count() > 0
            && !self.candidates_pending.load(Ordering::Relaxed)
            && self
                .candidates
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .is_empty()
            && self
                .deflation
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .is_none();
        #[cfg(feature = "gpu")]
        if let Some(gpu) = self.amg.as_ref().and_then(Amg::gpu_cycle)
            && loose.is_none()
            && !learning
        {
            let started = std::time::Instant::now();
            let mut x = vec![0.0; n];
            let cancelled = || {
                cancel
                    .as_ref()
                    .is_some_and(plxs_core::CancelToken::is_cancelled)
            };
            let mut stall = Stall::new(self.give_up_on_stall);
            match gpu.solve(
                &b,
                &mut x,
                TOLERANCE,
                max_iterations,
                &cancelled,
                &mut |i, r| stall.stalls(i, r),
            ) {
                Ok(Err(error)) => return Err(error),
                Ok(Ok(crate::gpu::CgOutcome::Converged {
                    iterations,
                    residual,
                })) => {
                    if trace {
                        eprintln!(
                            "pcg: {iterations} iterations on the graphics card, {:.2} s",
                            started.elapsed().as_secs_f64()
                        );
                    }
                    return Ok(self.converged(
                        &x,
                        rhs,
                        iterations,
                        residual,
                        started.elapsed().as_secs_f64(),
                    ));
                }
                Ok(Ok(crate::gpu::CgOutcome::Indefinite)) => return Err(indefinite()),
                Ok(Ok(crate::gpu::CgOutcome::NotConverged {
                    residual,
                    iterations,
                })) => {
                    return Err(not_converged(iterations, residual, self.reused));
                }
                Err(e) => {
                    // The card failed: the processor solves, from here on.
                    if let Some(amg) = self.amg.as_ref() {
                        amg.note_gpu_failure(format!("failed: {e}"));
                    }
                }
            }
        }
        let mut x_padded = vec![0.0; n];
        let x = &mut x_padded;
        let mut r = b;
        let mut z = vec![0.0; n];
        self.precondition(&r, &mut z)?;
        let mut p = z.clone();
        let mut q = vec![0.0; n];
        let mut rz = dot(&r, &z);
        // The Lanczos vectors of the solve (single precision), from which the slowest
        // modes of the preconditioned operator are taken after it (see [`Deflation`]).
        let mut deflation = self.deflation.lock().unwrap_or_else(|e| e.into_inner());
        let collect = deflation.is_none() && loose.is_none() && self.amg.is_some();
        let mut lanczos: Vec<Vec<f32>> = Vec::new();
        if collect {
            lanczos.push(z.iter().map(|v| (v / rz.sqrt()) as f32).collect());
        }
        // Deflated: the iteration solves the projected system `P A x̃ = P b` (whose
        // residuals stay in the complement of `A W` in every iteration, which keeps the
        // rounding out of the deflated space), and the solution of the original system is
        // `x = W E⁻¹ Wᵀ b + Pᵀ x̃` at the end.
        // The right-hand side, to check a deflated solution against at the end.
        let b0: Option<Vec<f64>> = deflation.as_ref().map(|_| r.clone());
        let mu_b = deflation.as_ref().map(|defl| {
            let mu = defl.solve_e(&defl.transpose_product(&defl.w, &r));
            defl.add_combination(&defl.aw, &mu, -1.0, &mut r);
            mu
        });
        if deflation.is_some() {
            self.precondition(&r, &mut z)?;
            p.copy_from_slice(&z);
            rz = dot(&r, &z);
        }
        let finish = |x: &mut [f64]| {
            if let (Some(defl), Some(mu_b)) = (deflation.as_ref(), mu_b.as_ref()) {
                let mu_x = defl.solve_e(&defl.transpose_product(&defl.aw, x));
                let mu: Vec<f64> = mu_b.iter().zip(&mu_x).map(|(b, x)| b - x).collect();
                defl.add_combination(&defl.w, &mu, 1.0, x);
            }
        };
        let mut residual = 1.0;
        let mut stall = Stall::new(self.give_up_on_stall);
        let (mut t_product, mut t_precondition, mut t_vectors) = (0.0, 0.0, 0.0);
        // The Lanczos tridiagonal of the preconditioned operator: its eigenvalues (Ritz
        // values) show what slows the iteration down (trace) and pick the modes to
        // deflate.
        let mut ritz = RitzTrace::new(trace);
        let clock = std::time::Instant::now();
        let mut last = 0.0;
        let mut lap = |t: &mut f64| {
            let now = clock.elapsed().as_secs_f64();
            *t += now - last;
            last = now;
        };
        for iteration in 1..=max_iterations {
            // Every iteration: on large systems one takes a good part of a second.
            if cancel
                .as_ref()
                .is_some_and(plxs_core::CancelToken::is_cancelled)
            {
                return Err(plxs_core::Error::Cancelled);
            }
            if trace {
                lap(&mut t_vectors);
            }
            self.product(&p, &mut q);
            if let Some(defl) = deflation.as_ref() {
                defl.project(&mut q);
            }
            if trace {
                lap(&mut t_product);
            }
            let pq = dot(&p, &q);
            if pq <= 0.0 || !pq.is_finite() {
                return Err(indefinite());
            }
            let alpha = rz / pq;
            ritz.alpha(alpha);
            axpy(alpha, &p, x);
            axpy(-alpha, &q, &mut r);
            residual = norm(&r) / b_norm;
            if residual <= tolerance {
                if trace {
                    eprintln!(
                        "pcg: {iteration} iterations, product {t_product:.2} s, \
                         preconditioner {t_precondition:.2} s, vectors {t_vectors:.2} s"
                    );
                    ritz.report();
                }
                finish(x);
                if let Some(b0) = b0.as_ref() {
                    // The projected residual converged; the solution also carries
                    // `W E⁻¹ Wᵀ b`, which rounding in a badly conditioned `E` can spoil.
                    self.product(x, &mut q);
                    let gap: f64 = b0.iter().zip(&q).map(|(b, q)| (b - q) * (b - q)).sum();
                    let true_residual = gap.sqrt() / b_norm;
                    if true_residual > DEFLATE_CHECK * tolerance || !true_residual.is_finite() {
                        if trace {
                            eprintln!(
                                "pcg: the deflated solve left a true residual of {true_residual:.2e}, solving again without the deflation"
                            );
                        }
                        *deflation = None;
                        drop(deflation);
                        self.unpad(b0, rhs);
                        return self.iterate(rhs, loose);
                    }
                }
                // Only a solve that kept every Lanczos vector teaches: the Ritz pairs of
                // a truncated tridiagonal are not eigenpairs (a buckling matrix near its
                // shift, 348 iterations: the 24 "modes" of its first 96 vectors made the
                // deflated operator indefinite).
                if collect && lanczos.len() >= DEFLATE_FROM_ITERATIONS && lanczos.len() == iteration
                {
                    let learned = self.learn_deflation(&ritz, &lanczos, trace);
                    if let Some(defl) = learned.as_ref() {
                        let mut candidates =
                            self.candidates.lock().unwrap_or_else(|e| e.into_inner());
                        if candidates.is_empty() {
                            // The slowest modes become near-nullspace candidates of the
                            // next hierarchy of this pattern.
                            let count = candidate_count().min(defl.d);
                            for j in (defl.d - count..defl.d).rev() {
                                candidates.push(defl.w[j * n..(j + 1) * n].to_vec());
                            }
                            if count > 0 {
                                self.candidates_pending.store(true, Ordering::Relaxed);
                            }
                        }
                    }
                    *deflation = learned;
                }
                if loose.is_some() {
                    // Not the count of a solve to the tolerance, which the kept hierarchy
                    // is judged by.
                    return Ok(self.unconverged(x, rhs, iteration, residual));
                }
                return Ok(self.converged(
                    x,
                    rhs,
                    iteration,
                    residual,
                    clock.elapsed().as_secs_f64(),
                ));
            }
            if stall.stalls(iteration, residual) {
                return Err(not_converged(iteration, residual, self.reused));
            }
            if trace {
                lap(&mut t_vectors);
            }
            self.precondition(&r, &mut z)?;
            if trace {
                lap(&mut t_precondition);
            }
            let rz_new = dot(&r, &z);
            if trace && rz_new <= 0.0 {
                eprintln!(
                    "pcg: iteration {iteration}: r.z = {rz_new:.3e} <= 0, the preconditioner is not positive definite here"
                );
            }
            if collect && lanczos.len() < DEFLATE_COLLECT_MAX && rz_new > 0.0 {
                let sign = if lanczos.len() % 2 == 1 { -1.0 } else { 1.0 };
                let scale = sign / rz_new.sqrt();
                lanczos.push(z.iter().map(|v| (v * scale) as f32).collect());
            }
            // Polak-Ribière form of beta (flexible conjugate gradients),
            // `z_new · (r_new - r_old) / (z_old · r_old)` with `r_new - r_old = -alpha q`:
            // one more dot product, and the directions stay conjugate when the
            // preconditioner is not exactly the same operator in every iteration (its
            // single precision rounding). Equal to `rz_new / rz` for a fixed operator.
            let beta = -alpha * dot(&z, &q) / rz;
            // The Lanczos relation holds for the Fletcher-Reeves form.
            ritz.beta(rz_new / rz);
            rz = rz_new;
            // p = z + beta p
            zip_mut(&mut p, &z, |p, z| *p = z + beta * *p);
        }
        if loose.is_some() && residual.is_finite() {
            finish(x);
            return Ok(self.unconverged(x, rhs, max_iterations, residual));
        }
        Err(not_converged(max_iterations, residual, self.reused))
    }

    /// The iterate `x` (padded) of a loose solve into `rhs`, without the bookkeeping of
    /// [`Self::converged`].
    fn unconverged(
        &self,
        x: &[f64],
        rhs: &mut [f64],
        iterations: usize,
        residual: f64,
    ) -> PcgStats {
        self.unpad(x, rhs);
        PcgStats {
            iterations,
            relative_residual: residual,
        }
    }

    /// The solution `x` (padded) into `rhs`, and the bookkeeping of a converged solve.
    fn converged(
        &self,
        x: &[f64],
        rhs: &mut [f64],
        iterations: usize,
        residual: f64,
        seconds: f64,
    ) -> PcgStats {
        self.unpad(x, rhs);
        self.last_iterations.store(iterations, Ordering::Relaxed);
        if self.iterations_after_build.load(Ordering::Relaxed) == 0 {
            self.iterations_after_build
                .store(iterations, Ordering::Relaxed);
        }
        if iterations > 0 {
            self.seconds_per_iteration
                .store((seconds / iterations as f64).to_bits(), Ordering::Relaxed);
        }
        PcgStats {
            iterations,
            relative_residual: residual,
        }
    }

    /// `y = A x` with the whole matrix in rows, rows in parallel.
    fn product(&self, x: &[f64], y: &mut [f64]) {
        self.full.multiply(x, y);
    }

    /// `z = M⁻¹ r` (padded vectors).
    fn precondition(&self, r: &[f64], z: &mut [f64]) -> Result<()> {
        match &self.amg {
            Some(amg) => amg.apply(r, z),
            None => {
                // The incomplete factor lives in the original numbering.
                let mut r_u = vec![0.0; self.slot_of.len()];
                let mut z_u = vec![0.0; self.slot_of.len()];
                self.unpad(r, &mut r_u);
                self.incomplete_cholesky_solve(&r_u, &mut z_u);
                z.iter_mut().for_each(|v| *v = 0.0);
                for (i, &slot) in self.slot_of.iter().enumerate() {
                    z[slot as usize] = z_u[i];
                }
                Ok(())
            }
        }
    }

    /// The unknowns in the padded numbering: a padding unknown belongs to its node without
    /// a component, so that it has no rigid body mode and stays decoupled.
    fn padded_unknowns(&self) -> Option<Unknowns> {
        let u = self.unknowns.as_ref()?;
        if u.node.len() != self.slot_of.len() {
            return None;
        }
        if !self.is_blocked() {
            let mut renumbered = u.clone();
            for (i, &slot) in self.slot_of.iter().enumerate() {
                let slot = slot as usize;
                renumbered.node[slot] = u.node[i];
                renumbered.component[slot] = u.component[i];
                renumbered.coordinates[slot] = u.coordinates[i];
                if let Some(&body) = u.body.get(i) {
                    renumbered.body[slot] = body;
                }
            }
            return Some(renumbered);
        }
        let nodes = self.padded / 3;
        let bodies_known = u.body.len() == u.node.len();
        let mut padded = Unknowns {
            node: (0..nodes as u32).flat_map(|b| [b, b, b]).collect(),
            component: vec![u8::MAX; self.padded],
            coordinates: vec![[0.0; 3]; self.padded],
            body: vec![0; if bodies_known { self.padded } else { 0 }],
        };
        for (i, &slot) in self.slot_of.iter().enumerate() {
            let slot = slot as usize;
            padded.component[slot] = u.component[i];
            let node = slot / 3;
            for c in 0..3 {
                padded.coordinates[3 * node + c] = u.coordinates[i];
                if bodies_known {
                    padded.body[3 * node + c] = u.body[i];
                }
            }
        }
        Some(padded)
    }

    /// The vector of the unknowns in the solver's (renumbered, padded) numbering, zero
    /// on the padding.
    fn pad(&self, v: &[f64]) -> Vec<f64> {
        let mut padded = vec![0.0; self.padded];
        for (i, &slot) in self.slot_of.iter().enumerate() {
            padded[slot as usize] = v[i];
        }
        padded
    }

    fn unpad(&self, padded: &[f64], v: &mut [f64]) {
        for (i, &slot) in self.slot_of.iter().enumerate() {
            v[i] = padded[slot as usize];
        }
    }

    /// Scalar storage: every entry of the upper triangle mirrored into rows, the unknowns
    /// renumbered by reverse Cuthill-McKee.
    fn analyze_scalar(&mut self, pattern: &Pattern) {
        let n = pattern.dimension();
        let (ptr, rows) = (pattern.col_ptr(), pattern.row_idx());
        let mut adjacency: Vec<Vec<u32>> = vec![Vec::new(); n];
        for j in 0..n {
            for &i in &rows[ptr[j] as usize..ptr[j + 1] as usize] {
                if i as usize != j {
                    adjacency[j].push(i);
                    adjacency[i as usize].push(j as u32);
                }
            }
        }
        let slot = reverse_cuthill_mckee(&adjacency);
        drop(adjacency);
        // Entries per row of the whole matrix: column j of the upper triangle gives row j
        // (its lower part) and, mirrored, one entry in each row i < j.
        let mut count = vec![0u32; n + 1];
        for j in 0..n {
            for &i in &rows[ptr[j] as usize..ptr[j + 1] as usize] {
                count[slot[j] as usize + 1] += 1;
                if i as usize != j {
                    count[slot[i as usize] as usize + 1] += 1;
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
            let sj = slot[j] as usize;
            for (k, &i) in rows.iter().enumerate().take(end).skip(start) {
                let si = slot[i as usize] as usize;
                let at = next[sj] as usize;
                full_col[at] = si as u32;
                full_src[at] = k as u32;
                next[sj] += 1;
                if si != sj {
                    let at = next[si] as usize;
                    full_col[at] = sj as u32;
                    full_src[at] = k as u32;
                    next[si] += 1;
                }
            }
        }
        self.full = Bsr {
            brows: n,
            bcols: n,
            br: 1,
            bc: 1,
            ptr: count.iter().map(|&c| c as usize).collect(),
            col: full_col,
            val: vec![0.0; nnz],
        };
        self.full_src = full_src;
        self.slot_of = slot;
        self.padded = n;
    }

    /// Blocked storage: the unknowns padded to three per node, in `3 × 3` blocks.
    fn analyze_blocked(&mut self, pattern: &Pattern, unknowns: &Unknowns) {
        let n = pattern.dimension();
        let (ptr, rows) = (pattern.col_ptr(), pattern.row_idx());
        // Nodes in the order of their first unknown.
        let max_node = unknowns
            .node
            .iter()
            .copied()
            .max()
            .map_or(0, |m| m as usize + 1);
        let mut dense = vec![u32::MAX; max_node];
        let mut nodes = 0u32;
        let mut node_of = Vec::with_capacity(n);
        for &node in &unknowns.node {
            let slot = &mut dense[node as usize];
            if *slot == u32::MAX {
                *slot = nodes;
                nodes += 1;
            }
            node_of.push(*slot);
        }
        let nodes = nodes as usize;
        // Block columns per block row, from the entries of the upper triangle and their
        // mirror images.
        let mut lists: Vec<Vec<u32>> = vec![Vec::new(); nodes];
        for j in 0..n {
            let bj = node_of[j];
            for &i in &rows[ptr[j] as usize..ptr[j + 1] as usize] {
                let bi = node_of[i as usize];
                lists[bj as usize].push(bi);
                if bi != bj {
                    lists[bi as usize].push(bj);
                }
            }
        }
        for list in &mut lists {
            list.sort_unstable();
            list.dedup();
        }
        // Nodes renumbered by reverse Cuthill-McKee for the locality of the product.
        let adjacency: Vec<Vec<u32>> = lists
            .iter()
            .enumerate()
            .map(|(b, list)| list.iter().copied().filter(|&c| c as usize != b).collect())
            .collect();
        let renumbered = reverse_cuthill_mckee(&adjacency);
        drop(adjacency);
        let node_of: Vec<u32> = node_of.iter().map(|&b| renumbered[b as usize]).collect();
        let slot_of: Vec<u32> = (0..n)
            .map(|i| 3 * node_of[i] + unknowns.component[i] as u32)
            .collect();
        let mut by_new: Vec<Vec<u32>> = vec![Vec::new(); nodes];
        for (old, list) in lists.into_iter().enumerate() {
            let mut list: Vec<u32> = list.into_iter().map(|c| renumbered[c as usize]).collect();
            list.sort_unstable();
            by_new[renumbered[old] as usize] = list;
        }
        let mut bptr = Vec::with_capacity(nodes + 1);
        let mut bcol = Vec::new();
        bptr.push(0);
        for list in &by_new {
            bcol.extend_from_slice(list);
            bptr.push(bcol.len());
        }
        drop(by_new);
        let blocks = bcol.len();
        let mut val = vec![0.0; 9 * blocks];
        let mut src = vec![u32::MAX; 9 * blocks];
        let position = |bi: u32, bj: u32| -> usize {
            let (s, e) = (bptr[bi as usize], bptr[bi as usize + 1]);
            s + bcol[s..e].binary_search(&bj).expect("block in the pattern")
        };
        for j in 0..n {
            let (bj, cj) = (node_of[j], unknowns.component[j] as usize);
            let (start, end) = (ptr[j] as usize, ptr[j + 1] as usize);
            for (k, &row) in rows.iter().enumerate().take(end).skip(start) {
                let i = row as usize;
                let (bi, ci) = (node_of[i], unknowns.component[i] as usize);
                // Row i, column j of the full matrix and its mirror image.
                src[9 * position(bi, bj) + 3 * ci + cj] = k as u32;
                src[9 * position(bj, bi) + 3 * cj + ci] = k as u32;
            }
        }
        // The padding: unit diagonal entries, coupled with nothing.
        let mut present = vec![false; 3 * nodes];
        for &slot in &slot_of {
            present[slot as usize] = true;
        }
        for b in 0..nodes {
            let d = position(b as u32, b as u32);
            for c in 0..3 {
                if !present[3 * b + c] {
                    val[9 * d + 4 * c] = 1.0;
                }
            }
        }
        self.full = Bsr {
            brows: nodes,
            bcols: nodes,
            br: 3,
            bc: 3,
            ptr: bptr,
            col: bcol,
            val,
        };
        self.full_src = src;
        self.slot_of = slot_of;
        self.padded = 3 * nodes;
    }

    /// `z = (L Lᵀ)⁻¹ r`.
    fn incomplete_cholesky_solve(&self, r: &[f64], z: &mut [f64]) {
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
        if self.pattern.as_deref() == Some(pattern) && self.analyzed_unknowns == self.unknowns {
            // The Newton iterations of a step keep their pattern.
            return Ok(());
        }
        let n = pattern.dimension();
        let blocked = self.unknowns.clone().filter(|u| {
            u.node.len() == n && n >= BLOCKED_FROM && u.component.iter().all(|&c| c < 3)
        });
        match &blocked {
            Some(unknowns) => self.analyze_blocked(pattern, unknowns),
            None => self.analyze_scalar(pattern),
        }
        self.factor = vec![0.0; pattern.nnz()];
        self.pattern = Some(std::sync::Arc::new(pattern.clone()));
        self.analyzed_unknowns = self.unknowns.clone();
        self.amg = None;
        *self.deflation.get_mut().unwrap_or_else(|e| e.into_inner()) = None;
        self.candidates
            .get_mut()
            .unwrap_or_else(|e| e.into_inner())
            .clear();
        self.candidates_pending.store(false, Ordering::Relaxed);
        self.reused = false;
        self.iterations_after_build.store(0, Ordering::Relaxed);
        self.factorized = false;
        Ok(())
    }

    fn factorize(&mut self, matrix: &SparseMatrix) -> Result<()> {
        let pattern = (self.pattern.as_ref())
            .ok_or_else(|| numerical(plxs_core::Failure::Internal, "factorize before analyze"))?;
        if **matrix.pattern() != **pattern {
            return Err(numerical(
                plxs_core::Failure::Internal,
                "factorize with another pattern than analyzed",
            ));
        }
        let values = matrix.values();
        let n = pattern.dimension();
        for i in 0..n {
            let diagonal = pattern.col_ptr()[i + 1] as usize - 1;
            if values[diagonal] <= 0.0 || !values[diagonal].is_finite() {
                return Err(numerical(
                    plxs_core::Failure::NotPositiveDefinite,
                    format!(
                        "the matrix is not positive definite (diagonal entry {} is {:.3e}), \
                     which the iterative solver needs",
                        i + 1,
                        values[diagonal]
                    ),
                ));
            }
        }
        let src = &self.full_src;
        self.full
            .val
            .par_iter_mut()
            .zip(src.par_iter())
            .with_min_len(4096)
            .for_each(|(v, &s)| {
                if s != u32::MAX {
                    *v = values[s as usize]
                }
            });
        if self.choice == Preconditioner::Multigrid {
            // The hierarchy of the previous factorization (same pattern) serves on while
            // the matrix changed little (the diagonal, in the Euclidean norm; a Newton
            // iteration moves it a little, another system of the same procedure a lot)
            // and while its staleness costs less than a new hierarchy: the iterations
            // the last solve took beyond the first solve after the build (what a new
            // hierarchy is expected to take), at the last solve's pace, over the next
            // `STALE_HORIZON` solves, against the time of the build.
            let after_build = self.iterations_after_build.load(Ordering::Relaxed);
            let last = self.last_iterations.load(Ordering::Relaxed);
            let pace = f64::from_bits(self.seconds_per_iteration.load(Ordering::Relaxed));
            let stale_cost = STALE_HORIZON * last.saturating_sub(after_build) as f64 * pace;
            let cheaper_to_keep = stale_cost <= self.build_seconds;
            let similar = || {
                let now = self.full.diagonal();
                let (mut change, mut scale) = (0.0, 0.0);
                for (n, b) in now.iter().zip(&self.built_diagonal) {
                    change += (n - b) * (n - b);
                    scale += b * b;
                }
                now.len() == self.built_diagonal.len() && change <= 0.25 * 0.25 * scale
            };
            let pending = self.candidates_pending.load(Ordering::Relaxed);
            let keep = after_build > 0 && cheaper_to_keep && !pending && !reuse_disabled();
            if std::env::var("PLXS_PCG_TRACE").is_ok() && after_build > 0 {
                eprintln!(
                    "pcg: hierarchy {}: last solve {last} iterations, first after build {after_build}, {pace:.3} s per iteration, build {:.2} s",
                    if keep { "kept" } else { "rebuilt" },
                    self.build_seconds
                );
            }
            if let Some(amg) = self.amg.as_mut().filter(|_| keep && similar()) {
                amg.refresh(&self.full);
                #[cfg(feature = "gpu")]
                amg.attach_operator(&self.full);
                self.reused = true;
                self.factorized = true;
                // The deflated modes stay, their products with the new matrix follow it.
                let mut deflation = std::mem::take(&mut self.deflation);
                if let Some(defl) = deflation
                    .get_mut()
                    .unwrap_or_else(|e| e.into_inner())
                    .as_mut()
                    && !defl.refresh(self)
                {
                    *deflation.get_mut().unwrap_or_else(|e| e.into_inner()) = None;
                }
                self.deflation = deflation;
                return Ok(());
            }
            match self.build_multigrid() {
                Ok(()) => return Ok(()),
                // A hierarchy that cannot be built (coarsening stalled, indefinite coarse
                // system): the incomplete factorization serves instead.
                Err(Error::Numerical { .. }) => {}
                Err(error) => return Err(error),
            }
        }
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

/// The reverse Cuthill-McKee numbering of a graph given by its adjacency lists (without
/// self loops): `new[old]`. Neighbouring vertices get nearby numbers, which keeps the
/// vector of a product in the cache.
fn reverse_cuthill_mckee(adjacency: &[Vec<u32>]) -> Vec<u32> {
    let n = adjacency.len();
    let degree: Vec<u32> = adjacency.iter().map(|a| a.len() as u32).collect();
    let mut visited = vec![false; n];
    let mut order: Vec<u32> = Vec::with_capacity(n);
    let mut by_degree: Vec<u32> = (0..n as u32).collect();
    by_degree.sort_by_key(|&v| degree[v as usize]);
    let bfs = |start: u32, visited: &mut [bool], order: &mut Vec<u32>| {
        let first = order.len();
        order.push(start);
        visited[start as usize] = true;
        let mut head = first;
        let mut neighbours: Vec<u32> = Vec::new();
        while head < order.len() {
            let v = order[head] as usize;
            head += 1;
            neighbours.clear();
            neighbours.extend(
                adjacency[v]
                    .iter()
                    .copied()
                    .filter(|&w| !visited[w as usize]),
            );
            neighbours.sort_by_key(|&w| degree[w as usize]);
            for &w in &neighbours {
                visited[w as usize] = true;
                order.push(w);
            }
        }
        first
    };
    for &candidate in &by_degree {
        if visited[candidate as usize] {
            continue;
        }
        // A pseudo-peripheral start: the last vertex of a breadth-first search from the
        // vertex of least degree, twice.
        let mut start = candidate;
        for _ in 0..2 {
            let mut trial_visited = visited.clone();
            let mut trial = Vec::new();
            bfs(start, &mut trial_visited, &mut trial);
            start = *trial.last().expect("the start itself");
        }
        bfs(start, &mut visited, &mut order);
    }
    order.reverse();
    let mut new = vec![0u32; n];
    for (position, &old) in order.iter().enumerate() {
        new[old as usize] = position as u32;
    }
    new
}

/// Iterations between the checks of [`Stall`].
const STALL_CHECK: usize = 100;

/// How far back [`Stall`] compares the residual.
const STALL_SPAN: usize = 200;

/// The test for an iteration that does not get anywhere: from [`STALL_SPAN`] iterations
/// on, every [`STALL_CHECK`] iterations, the residual has to be at most half of what it
/// was [`STALL_SPAN`] iterations before (the start counts as 1). A solve that is to reach
/// [`TOLERANCE`] within a few thousand iterations is far faster than that; a model the
/// preconditioner does not fit (thin expanded shells, a badly conditioned mesh) is not,
/// and stalls for its whole limit otherwise, minutes on a large model, before the direct
/// solver takes over.
pub(crate) struct Stall {
    enabled: bool,
    /// The residual at every check so far, from iteration 0.
    marks: Vec<f64>,
}

impl Stall {
    pub(crate) fn new(enabled: bool) -> Self {
        Self {
            enabled,
            marks: vec![1.0],
        }
    }

    /// Whether the iteration stalls, given the residual after `iteration` iterations
    /// (called every iteration, or every few on the graphics card: a check is taken at
    /// the first call at or after its iteration).
    pub(crate) fn stalls(&mut self, iteration: usize, residual: f64) -> bool {
        if !self.enabled || iteration < self.marks.len() * STALL_CHECK {
            return false;
        }
        self.marks.push(residual);
        let back = STALL_SPAN / STALL_CHECK;
        let checks = self.marks.len() - 1;
        // A residual that is not a number stalls too.
        checks >= back && (residual.is_nan() || residual > 0.5 * self.marks[checks - back])
    }
}

/// A solve collects at most this many Lanczos vectors (single precision, one per
/// iteration) for the deflation.
/// Over how many solves a kept hierarchy is expected to serve on: its extra iterations
/// over that many solves are weighed against the time of a new build.
const STALE_HORIZON: f64 = 2.0;

/// How many Lanczos vectors a learning solve keeps (single precision).
const DEFLATE_COLLECT_MAX: usize = 96;
/// A solve shorter than this teaches nothing worth deflating.
const DEFLATE_FROM_ITERATIONS: usize = 30;
/// Ritz values below this fraction of the largest one are deflated: a hierarchy that fits
/// its matrix has no eigenvalue of `M⁻¹ A` below about 0.17 (a solid block of quadratic
/// tetrahedra); the slow modes of a thin flange or a contact interface sit at 0.02 to 0.1.
const DEFLATE_BELOW: f64 = 0.15;
/// At most this many modes are deflated (two vectors of the system's length each).
const DEFLATE_MAX: usize = 24;
/// A Ritz vector whose part outside the span of the ones before it is shorter than this
/// (of a unit vector) is a copy of them and is dropped.
const DEFLATE_INDEPENDENT: f64 = 1e-3;
/// A deflated solve whose true residual exceeds the tolerance by this factor is repeated
/// without the deflation.
const DEFLATE_CHECK: f64 = 1e3;
/// How many of the slowest modes become near-nullspace candidates of the next hierarchy
/// (`PLXS_AMG_CANDIDATES`).
fn candidate_count() -> usize {
    std::env::var("PLXS_AMG_CANDIDATES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(3)
}

/// Deflation of the slowest modes of the preconditioned operator (Saad, Yeung, Erhel,
/// Guyomarc'h, "A deflated version of the conjugate gradient algorithm", SIAM J. Sci.
/// Comput. 21 (2000)): the Newton iterations of a step solve a sequence of systems with
/// one multigrid hierarchy, and the first solve shows, in its Lanczos tridiagonal, which
/// modes the hierarchy reduces slowly (the bending of a thin flange, the modes of a body
/// held by contact only). Their Ritz vectors `W` are taken out of the solves after it,
/// in the projected form of Frank and Vuik, "On the construction of deflation-based
/// preconditioners", SIAM J. Sci. Comput. 23 (2001): the conjugate gradients solve
/// `P A x̃ = P b` with `P = I − A W E⁻¹ Wᵀ`, `E = Wᵀ A W`, whose operator has the deflated
/// eigenvalues replaced by zero, and `x = W E⁻¹ Wᵀ b + Pᵀ x̃` solves the original system.
/// The multigrid stays as it is; `A W` and `E` follow the matrix of every factorization.
struct Deflation {
    d: usize,
    /// `d` vectors of the system's length, one after the other.
    w: Vec<f64>,
    /// `A W`, in the same layout.
    aw: Vec<f64>,
    /// The Cholesky factor (lower, row major, `d × d`) of `E = Wᵀ A W`.
    chol: Vec<f64>,
}

impl Deflation {
    /// Recomputes `A W` and `E` for the solver's current matrix; `false` when `E` is not
    /// positive definite (the modes no longer fit).
    fn refresh(&mut self, solver: &PcgSolver) -> bool {
        let n = solver.padded;
        let d = self.d;
        for j in 0..d {
            let (w, aw) = (
                &self.w[j * n..(j + 1) * n],
                &mut self.aw[j * n..(j + 1) * n],
            );
            solver.product(w, aw);
        }
        let mut e = vec![0.0; d * d];
        for i in 0..d {
            for j in 0..=i {
                let v = dot(&self.w[i * n..(i + 1) * n], &self.aw[j * n..(j + 1) * n]);
                e[i * d + j] = v;
                e[j * d + i] = v;
            }
        }
        // Cholesky, lower.
        let mut l = vec![0.0; d * d];
        for i in 0..d {
            for j in 0..=i {
                let mut sum = e[i * d + j];
                for k in 0..j {
                    sum -= l[i * d + k] * l[j * d + k];
                }
                if i == j {
                    // A pivot far below the diagonal: two modes nearly the same, `E⁻¹`
                    // would amplify rounding into the solution.
                    if sum <= 1e-10 * e[i * d + i] || !sum.is_finite() {
                        return false;
                    }
                    l[i * d + i] = sum.sqrt();
                } else {
                    l[i * d + j] = sum / l[j * d + j];
                }
            }
        }
        self.chol = l;
        true
    }

    /// `E⁻¹ y`.
    fn solve_e(&self, y: &[f64]) -> Vec<f64> {
        let d = self.d;
        let l = &self.chol;
        let mut x = y.to_vec();
        for i in 0..d {
            for k in 0..i {
                x[i] -= l[i * d + k] * x[k];
            }
            x[i] /= l[i * d + i];
        }
        for i in (0..d).rev() {
            for k in i + 1..d {
                x[i] -= l[k * d + i] * x[k];
            }
            x[i] /= l[i * d + i];
        }
        x
    }

    /// `Vᵀ x` for the `d` vectors `V` (`w` or `aw`).
    fn transpose_product(&self, v: &[f64], x: &[f64]) -> Vec<f64> {
        let n = x.len();
        (0..self.d)
            .into_par_iter()
            .map(|j| dot(&v[j * n..(j + 1) * n], x))
            .collect()
    }

    /// `out += sign · V μ`.
    fn add_combination(&self, v: &[f64], mu: &[f64], sign: f64, out: &mut [f64]) {
        let n = out.len();
        let d = self.d;
        out.par_chunks_mut(2048).enumerate().for_each(|(c, out)| {
            let start = c * 2048;
            for j in 0..d {
                let coefficient = sign * mu[j];
                if coefficient == 0.0 {
                    continue;
                }
                let v = &v[j * n + start..j * n + start + out.len()];
                out.iter_mut()
                    .zip(v)
                    .for_each(|(o, v)| *o += coefficient * v);
            }
        });
    }

    /// `q ← P q = q − A W E⁻¹ Wᵀ q`: takes the part of a residual (or of `A p`) in the
    /// deflated space out.
    fn project(&self, q: &mut [f64]) {
        let mu = self.solve_e(&self.transpose_product(&self.w, q));
        self.add_combination(&self.aw, &mu, -1.0, q);
    }
}

/// The eigenvalues and (row major, one per row) eigenvectors of the symmetric tridiagonal
/// matrix with diagonal `diag` and off-diagonal `off` (`off[k]` couples `k-1` and `k`):
/// the implicit QL algorithm with Wilkinson shifts.
fn tridiagonal_eigen(diag: &[f64], off: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let m = diag.len();
    let mut d = diag.to_vec();
    // e[k] couples k and k+1.
    let mut e: Vec<f64> = (0..m)
        .map(|k| off.get(k + 1).copied().unwrap_or(0.0))
        .collect();
    // z holds the eigenvectors as columns: z[i*m + j] is component i of vector j.
    let mut z = vec![0.0; m * m];
    for i in 0..m {
        z[i * m + i] = 1.0;
    }
    for l in 0..m {
        let mut iterations = 0;
        loop {
            let mut mm = l;
            while mm + 1 < m {
                let dd = d[mm].abs() + d[mm + 1].abs();
                if e[mm].abs() <= f64::EPSILON * dd {
                    break;
                }
                mm += 1;
            }
            if mm == l {
                break;
            }
            iterations += 1;
            if iterations > 60 {
                break;
            }
            let mut g = (d[l + 1] - d[l]) / (2.0 * e[l]);
            let mut r = g.hypot(1.0);
            g = d[mm] - d[l] + e[l] / (g + r.copysign(g));
            let (mut s, mut c, mut p) = (1.0, 1.0, 0.0);
            let mut i = mm;
            let mut underflow = false;
            while i > l {
                i -= 1;
                let f = s * e[i];
                let b = c * e[i];
                r = f.hypot(g);
                e[i + 1] = r;
                if r == 0.0 {
                    d[i + 1] -= p;
                    e[mm] = 0.0;
                    underflow = true;
                    break;
                }
                s = f / r;
                c = g / r;
                g = d[i + 1] - p;
                r = (d[i] - g) * s + 2.0 * c * b;
                p = s * r;
                d[i + 1] = g + p;
                g = c * r - b;
                for k in 0..m {
                    let f = z[k * m + i + 1];
                    z[k * m + i + 1] = s * z[k * m + i] + c * f;
                    z[k * m + i] = c * z[k * m + i] - s * f;
                }
            }
            if underflow {
                continue;
            }
            d[l] -= p;
            e[l] = g;
            e[mm] = 0.0;
        }
    }
    // Rows: vector j is z[.., j].
    let vectors: Vec<f64> = (0..m)
        .flat_map(|j| (0..m).map(move |i| (i, j)))
        .map(|(i, j)| z[i * m + j])
        .collect();
    (d, vectors)
}

/// The Lanczos tridiagonal `T` built from the conjugate gradient coefficients
/// (`T_kk = 1/α_k + β_{k-1}/α_{k-1}`, `T_{k,k+1} = √β_k / α_k`), whose eigenvalues
/// approximate those of the preconditioned operator `M⁻¹ A`; `PLXS_PCG_TRACE` prints the
/// extreme ones.
struct RitzTrace {
    enabled: bool,
    alphas: Vec<f64>,
    betas: Vec<f64>,
}

impl RitzTrace {
    fn new(enabled: bool) -> Self {
        Self {
            enabled,
            alphas: Vec::new(),
            betas: Vec::new(),
        }
    }

    fn alpha(&mut self, alpha: f64) {
        self.alphas.push(alpha);
    }

    fn beta(&mut self, beta: f64) {
        self.betas.push(beta);
    }

    /// The diagonal and off-diagonal (`off[k]` couples `k-1` and `k`) of `T`.
    fn tridiagonal(&self) -> (Vec<f64>, Vec<f64>) {
        let m = self.alphas.len();
        let mut diag = vec![0.0; m];
        let mut off = vec![0.0; m];
        for k in 0..m {
            diag[k] = 1.0 / self.alphas[k];
            if k > 0 {
                diag[k] += self.betas[k - 1] / self.alphas[k - 1];
                off[k] = self.betas[k - 1].max(0.0).sqrt() / self.alphas[k - 1];
            }
        }
        (diag, off)
    }

    fn report(&self) {
        let m = self.alphas.len();
        if !self.enabled || m < 2 {
            return;
        }
        let (diag, off) = self.tridiagonal();
        let values: Vec<f64> = tridiagonal_eigenvalues(&diag, &off)
            .into_iter()
            .filter(|v| *v > 0.0)
            .collect();
        let m = values.len();
        if m == 0 {
            return;
        }
        let low: Vec<String> = values.iter().take(8).map(|v| format!("{v:.2e}")).collect();
        let high: Vec<String> = values
            .iter()
            .rev()
            .take(4)
            .map(|v| format!("{v:.2e}"))
            .collect();
        eprintln!(
            "pcg: ritz values of M^-1 A: smallest {} ... largest {} (condition {:.1e})",
            low.join(" "),
            high.join(" "),
            values[m - 1] / values[0].max(f64::MIN_POSITIVE)
        );
    }
}

/// The eigenvalues (ascending) of the symmetric tridiagonal matrix with diagonal `diag`
/// and off-diagonal `off` (`off[k]` couples `k-1` and `k`), by bisection on the Sturm
/// count.
fn tridiagonal_eigenvalues(diag: &[f64], off: &[f64]) -> Vec<f64> {
    let m = diag.len();
    let count_below = |x: f64| {
        let mut count = 0;
        let mut q = 1.0;
        for k in 0..m {
            let o = if k > 0 { off[k] * off[k] } else { 0.0 };
            q = diag[k] - x - if q != 0.0 { o / q } else { o / 1e-300 };
            if q < 0.0 {
                count += 1;
            }
        }
        count
    };
    let radius = (0..m)
        .map(|k| diag[k].abs() + off[k].abs() + off.get(k + 1).map_or(0.0, |o| o.abs()))
        .fold(0.0_f64, f64::max);
    (0..m)
        .map(|i| {
            let (mut lo, mut hi) = (-radius, radius);
            for _ in 0..200 {
                let mid = 0.5 * (lo + hi);
                if count_below(mid) > i {
                    hi = mid;
                } else {
                    lo = mid;
                }
                if hi - lo <= 1e-12 * radius {
                    break;
                }
            }
            0.5 * (lo + hi)
        })
        .collect()
}

fn indefinite() -> Error {
    numerical(
        plxs_core::Failure::NotPositiveDefinite,
        "the matrix is not positive definite, which the iterative solver needs; \
         choose the direct equation solver",
    )
}

fn not_converged(iterations: usize, residual: f64, reused: bool) -> Error {
    numerical(
        plxs_core::Failure::IterativeSolver,
        format!(
            "the iterative solver did not converge in {iterations} iterations \
         (relative residual {residual:.3e}){}",
            if reused {
                " with the kept preconditioner"
            } else {
                "; choose the direct equation solver"
            }
        ),
    )
}

fn numerical(failure: plxs_core::Failure, message: impl Into<String>) -> Error {
    Error::numerical(failure, message)
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
    #[test]
    fn a_stalling_iteration_is_given_up_after_two_hundred_iterations() {
        // Above the start after 200 iterations (the corrugated tube of expanded shells).
        let mut stall = super::Stall::new(true);
        assert!((1..200).all(|i| !stall.stalls(i, 50.0)));
        assert!(stall.stalls(200, 50.0));
        // Halving every 200 iterations goes on.
        let mut stall = super::Stall::new(true);
        let residual = |i: usize| 0.5f64.powf(i as f64 / 190.0);
        assert!((1..3000).all(|i| !stall.stalls(i, residual(i))));
        // Checks taken late (the graphics card reads every few iterations) still count.
        let mut stall = super::Stall::new(true);
        assert!(!stall.stalls(104, 0.3));
        assert!(!stall.stalls(204, 0.4));
        assert!(stall.stalls(304, 0.2));
        // Off: never.
        let mut stall = super::Stall::new(false);
        assert!((1..3000).all(|i| !stall.stalls(i, 10.0)));
    }

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
    fn tridiagonal_eigen_decomposes_the_matrix() {
        let diag = [2.0, 3.0, 1.0, 4.0, 2.5];
        let off = [0.0, 0.5, -0.7, 0.3, 0.9];
        let (values, vectors) = tridiagonal_eigen(&diag, &off);
        let m = 5;
        let mut sorted = values.clone();
        sorted.sort_by(f64::total_cmp);
        let sturm = tridiagonal_eigenvalues(&diag, &off);
        for (a, b) in sorted.iter().zip(&sturm) {
            assert!((a - b).abs() < 1e-9, "{sorted:?} vs {sturm:?}");
        }
        for j in 0..m {
            let y = &vectors[j * m..(j + 1) * m];
            for i in 0..m {
                let mut t = diag[i] * y[i];
                if i > 0 {
                    t += off[i] * y[i - 1];
                }
                if i + 1 < m {
                    t += off[i + 1] * y[i + 1];
                }
                assert!((t - values[j] * y[i]).abs() < 1e-9, "vector {j}");
            }
            let norm: f64 = y.iter().map(|v| v * v).sum();
            assert!((norm - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn deflated_conjugate_gradients_match_the_direct_solver() {
        let n = 20_000;
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
        let plain = pcg.solve_counted(&mut b.clone()).unwrap();
        // Any A-independent vectors deflate correctly; smooth ones (the slow modes of a
        // multigrid are smooth) also save iterations.
        let d = 3;
        let w: Vec<f64> = (0..d)
            .flat_map(|j| {
                (0..n).map(move |i| ((i as f64) * (j + 1) as f64 * 1e-4).sin() + 0.1 * j as f64)
            })
            .collect();
        let mut defl = Deflation {
            d,
            w,
            aw: vec![0.0; d * n],
            chol: Vec::new(),
        };
        assert!(defl.refresh(&pcg));
        *pcg.deflation.lock().unwrap() = Some(defl);
        let mut x = b.clone();
        let deflated = pcg.solve_counted(&mut x).unwrap();
        assert!(deflated.relative_residual <= TOLERANCE, "{deflated:?}");
        let scale = expected.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        for (x, e) in x.iter().zip(&expected) {
            assert!((x - e).abs() <= 1e-7 * scale, "{x} vs {e}");
        }
        assert!(
            deflated.iterations <= plain.iterations,
            "{deflated:?} vs {plain:?}"
        );
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
    fn the_hierarchy_is_kept_over_factorizations_with_the_same_pattern() {
        // Newton iterations: the values change a little, the hierarchy serves on.
        let n = 20_000;
        let a = chain(n);
        let mut pcg = PcgSolver::new();
        pcg.analyze(a.pattern()).unwrap();
        pcg.factorize(&a).unwrap();
        assert!(
            !pcg.preconditioner().contains("kept"),
            "{}",
            pcg.preconditioner()
        );
        let mut x = vec![1.0; n];
        pcg.solve_counted(&mut x).unwrap();
        let mut b = SparseMatrix::zeros(a.pattern().clone());
        for (v, w) in b.values_mut().iter_mut().zip(a.values()) {
            *v = 1.1 * w;
        }
        pcg.factorize(&b).unwrap();
        assert!(
            pcg.preconditioner().contains("kept"),
            "{}",
            pcg.preconditioner()
        );
        let mut direct = LdltSolver::new(1);
        direct.analyze(b.pattern()).unwrap();
        direct.factorize(&b).unwrap();
        let mut expected = vec![1.0; n];
        direct.solve(&mut expected).unwrap();
        let mut x = vec![1.0; n];
        let stats = pcg.solve_counted(&mut x).unwrap();
        assert!(stats.relative_residual <= TOLERANCE, "{stats:?}");
        let scale = expected.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        for (x, e) in x.iter().zip(&expected) {
            assert!((x - e).abs() <= 1e-7 * scale, "{x} vs {e}");
        }
    }

    #[test]
    fn a_much_changed_matrix_gets_a_new_hierarchy() {
        // Another system of the same pattern (the diagonal three times as large): the
        // hierarchy of the first would not fit, it is built anew.
        let n = 20_000;
        let a = chain(n);
        let mut pcg = PcgSolver::new();
        pcg.analyze(a.pattern()).unwrap();
        pcg.factorize(&a).unwrap();
        let mut x = vec![1.0; n];
        pcg.solve_counted(&mut x).unwrap();
        let mut b = SparseMatrix::zeros(a.pattern().clone());
        for (i, (v, w)) in b.values_mut().iter_mut().zip(a.values()).enumerate() {
            *v = if a.pattern().row_idx()[i] as usize == column_of(a.pattern(), i) {
                3.0 * w
            } else {
                *w
            };
        }
        pcg.factorize(&b).unwrap();
        assert!(!pcg.is_reused(), "{}", pcg.preconditioner());
        let mut x = vec![1.0; n];
        let stats = pcg.solve_counted(&mut x).unwrap();
        assert!(stats.relative_residual <= TOLERANCE, "{stats:?}");
    }

    /// The column of the `k`-th stored value of `pattern`.
    fn column_of(pattern: &Pattern, k: usize) -> usize {
        pattern.col_ptr().partition_point(|&p| p as usize <= k) - 1
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
