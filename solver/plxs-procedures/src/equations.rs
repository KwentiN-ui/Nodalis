//! The equation solver of a step: the direct LDLᵀ the session keeps, conjugate gradients
//! when the step asks for an iterative solver (`SOLVER=ITERATIVE CHOLESKY` or `ITERATIVE
//! SCALING`), or the automatic choice between them when it names no solver.
//!
//! The procedures whose systems are symmetric positive definite use it: linear and
//! nonlinear static steps (with contact; the unsymmetric part of sliding friction is
//! resolved by GMRES around it), `*VISCO`, implicit `*DYNAMIC` steps and heat transfer. The
//! eigenvalue procedures solve directly.
//!
//! The automatic choice (no `SOLVER=`) solves iteratively from [`AUTOMATIC_FROM`] equations
//! on when the unknowns are translations or a scalar field only (a solid or heat transfer
//! model, where the multigrid converges in a few dozen iterations and beats the
//! factorization by far); models with rotations (shells, beams), procedures that solve
//! many systems with one factorization (the eigenvalue iteration, implicit dynamics) and
//! small models stay direct while the factor fits into the memory, and so do models of
//! shells, beams or trusses expanded into solids (translations only, but bending through
//! elements a fraction of a millimetre thin: the multigrid stalls on them, see
//! [`Self::for_model`]): for a system of at least
//! [`AUTOMATIC_FROM`] equations the symbolic analysis tells the size of the factor before
//! its memory is taken, and beyond [`MEMORY_SHARE`] of the available memory the step solves
//! iteratively. Each choice falls back to the other once when it fails: a factor that does
//! not fit after all goes iterative, an iteration that breaks down or does not converge
//! within [`AUTOMATIC_MAX_ITERATIONS`] (an indefinite tangent near a limit point, a
//! mechanism held only by contact, a badly conditioned model) goes direct.

use plxs_core::step::{EquationSolver, SolverMode};
use plxs_core::{Error, Progress, ProgressEvent, Result};
use std::sync::Arc;

use plxs_linalg::{
    LdltSolver, LinearSolver, Pattern, PcgSolver, SparseMatrix, Unknowns, is_out_of_memory,
};

/// The automatic choice considers the iterative solver from this many equations on;
/// smaller systems always fit and the direct solver is faster on them.
pub const AUTOMATIC_FROM: usize = 50_000;

/// The automatic choice gives the iterative solver this many iterations before it hands
/// the system to the direct solver (a solid model converges in well under a hundred).
pub const AUTOMATIC_MAX_ITERATIONS: usize = 2000;

/// The automatic choice solves iteratively when the factor (with a quarter more for the
/// work space of the factorization) would take more than this share of the available
/// memory.
pub const MEMORY_SHARE: f64 = 0.6;

/// GMRES on the unsymmetric tangent stops at this relative residual.
pub const UNSYMMETRIC_TOLERANCE: f64 = 1e-10;

/// A GMRES solution with at most this relative residual is good enough for the Newton
/// iteration; beyond it the equation solver tries harder (see
/// [`Equations::solve_unsymmetric`]).
pub const UNSYMMETRIC_ACCEPTED: f64 = 1e-6;

/// The iterative solver as the preconditioner of GMRES: the relative residual it stops
/// at, and its iterations at most. One digit is enough: the flexible GMRES takes a few
/// more outer iterations, each far cheaper (a contact gasket sliding with friction solved
/// 40 % faster than with 1e-4, most of the work went into the inner digits).
pub const PRECONDITIONER_TOLERANCE: f64 = 1e-1;
pub const PRECONDITIONER_ITERATIONS: usize = 100;

/// A kept factor is replaced by one of the current tangent once a solve with it took more
/// conjugate gradient iterations than are worth about half a factorization: the square
/// root of the factor's nonzeros over this, within these bounds. Twice as many and the
/// solve is given up for a factorization at once.
pub const KEPT_FACTOR_SOLVES: f64 = 3000.0;
pub const KEPT_FACTOR_FEWEST: usize = 3;
pub const KEPT_FACTOR_ITERATIONS: usize = 20;

/// The relative residual the conjugate gradients on a kept factor stop at when the
/// procedure asked for no tolerance: as good as the direct solve they stand in for.
pub const KEPT_FACTOR_TOLERANCE: f64 = 1e-10;

/// The pace of the factorization in the estimate of the automatic choice, floating point
/// operations per second and thread (measured with 4 threads: a bracket of 591 000
/// equations, 1.05e12 operations, 9.3 s, about 28 billion per thread; a strip of 167 000,
/// 1.3e10 operations, 0.21 s, about 16 billion).
pub const FACTOR_FLOPS_PER_THREAD: f64 = 25e9;

/// The pace at which the factor's memory is taken (written once), bytes per second
/// (4.2 GB in 3 s in the cloud container).
pub const TOUCH_BYTES_PER_SECOND: f64 = 1.4e9;

/// Without a way to ask the platform for its free memory, the automatic choice assumes
/// this much.
const ASSUMED_MEMORY: u64 = 8 << 30;

/// The solver of a step's systems; the direct one is borrowed for each call so that it can
/// live in the procedure context.
pub struct Equations {
    mode: SolverMode,
    /// The current choice; `None` until the first [`Self::prepare`] decides automatically.
    iterative: Option<bool>,
    pcg: Option<PcgSolver>,
    /// Automatic and iterative: the prepared matrix with the positions of its unknowns, to
    /// factorize it after all when the iteration fails (ordered by the positions, as the
    /// direct solver is ordered otherwise: without them the factor fills many times more).
    /// Only the pattern is kept: the values are in the iterative solver's own copy of the
    /// matrix, which gives them back (a second copy of a large matrix costs hundreds of
    /// megabytes for a fall-back that is rarely taken).
    kept: Option<(Arc<Pattern>, Vec<[f64; 3]>)>,
    /// Whether the automatic choice already fell back once (it does not switch back).
    fell_back: bool,
    /// The memory the automatic choice works with, `None`: ask the platform.
    available: Option<u64>,
    /// [`AUTOMATIC_FROM`], lowered by tests.
    automatic_from: usize,
    /// Automatic: direct while the factor fits, even for a solid model (a procedure that
    /// solves many systems with one factorization).
    prefer_direct: bool,
    /// The relative residual the next solves need (`None`: all the solver gives), see
    /// [`Self::set_tolerance`].
    tolerance: Option<f64>,
    /// Direct: whether the factor of an earlier tangent may serve on as the preconditioner
    /// of conjugate gradients on the current one, see [`Self::keeping_factors`].
    keep_factors: bool,
    /// The pattern of the matrix the direct solver's factor belongs to.
    factored: Option<Arc<Pattern>>,
    /// The prepared matrix when the factor is that of an earlier one.
    stale: Option<SparseMatrix>,
    /// Whether the next prepared matrix is factorized whatever the factor at hand.
    refresh: bool,
    /// Whether a slow iteration was weighed against the direct solver already (once).
    probed: bool,
}

impl Equations {
    pub fn new(choice: EquationSolver) -> Self {
        let mode = choice.mode();
        Self {
            mode,
            iterative: match mode {
                SolverMode::Automatic => None,
                SolverMode::Direct => Some(false),
                SolverMode::Iterative => Some(true),
            },
            pcg: None,
            kept: None,
            fell_back: false,
            available: None,
            automatic_from: AUTOMATIC_FROM,
            prefer_direct: false,
            tolerance: None,
            keep_factors: false,
            factored: None,
            stale: None,
            refresh: false,
            probed: false,
        }
    }

    /// The direct solver keeps its factor over the systems to come while it still serves:
    /// for a procedure whose matrices change little from one system to the next (the
    /// Newton iterations of a static step). A system prepared while a factor of the same
    /// pattern is at hand is not factorized; conjugate gradients solve it with the kept
    /// factor as the preconditioner, in a handful of iterations of one substitution and
    /// one product each where a factorization costs a hundred substitutions. The
    /// solution is that of the current matrix (to the tolerance asked for), so the Newton
    /// iteration converges as with a fresh factor, unlike a modified Newton iteration.
    /// The tangent is factorized anew when the iteration breaks down (an indefinite
    /// tangent or factor), or takes more iterations than a share of a
    /// factorization is worth ([`KEPT_FACTOR_SOLVES`]).
    pub fn keeping_factors(mut self) -> Self {
        self.keep_factors = std::env::var("PLXS_KEEP_FACTOR").as_deref() != Ok("0");
        self
    }

    /// The relative residual `|r| / |b|` the solves from now on have to reach; `None`:
    /// all the solver gives (the direct solve, the iterative solver's own tolerance). The
    /// direct solver with a fresh factor ignores it.
    pub fn set_tolerance(&mut self, tolerance: Option<f64>) {
        self.tolerance = tolerance;
    }

    /// The automatic choice stays direct while the factor fits into the memory, whatever
    /// the model: for a procedure that solves many systems with one factorization (the
    /// eigenvalue iteration, implicit dynamics), where the factorization pays off.
    pub fn preferring_direct(mut self) -> Self {
        self.prefer_direct = true;
        self
    }

    /// The automatic choice for a model of `model`: a model whose shells, membranes, beams
    /// or trusses were expanded into solids is solved directly while its factor fits (its
    /// unknowns are translations only, yet the multigrid built on them does not fit the
    /// bending of the thin expanded elements and stalls: a corrugated tube of 0.3 mm
    /// shells, 367k equations, did not get below its initial residual in 2000 iterations,
    /// 80 s, while the factorization took 4 s).
    pub fn for_model(mut self, model: &plxs_core::Model) -> Self {
        if model.expanded_structure {
            self.prefer_direct = true;
        }
        self
    }

    /// Whether the systems are solved iteratively (in automatic mode: the choice so far,
    /// direct before the first system).
    pub fn is_iterative(&self) -> bool {
        self.iterative == Some(true)
    }

    /// Whether a numerical error of this solver can mean an unconstrained model: a direct
    /// factorization's zero pivot can, the iterative solver's messages say what failed.
    pub fn hints_constraints(&self) -> bool {
        !self.is_iterative()
    }

    /// Analyzes (when `analyze`, i.e. the pattern may have changed) and factorizes `k`, or
    /// builds the preconditioner; `unknowns` tells what the unknowns are (nodes and
    /// positions), for the ordering of the direct solver and the multigrid of the
    /// iterative one.
    pub fn prepare(
        &mut self,
        direct: &mut LdltSolver,
        k: &SparseMatrix,
        unknowns: impl FnOnce() -> Unknowns,
        analyze: bool,
        progress: &mut dyn Progress,
    ) -> Result<()> {
        let unknowns = unknowns();
        let mut analyze = analyze;
        if self.iterative.is_none() {
            let iterative = self.choose(direct, k, &unknowns, progress)?;
            self.iterative = Some(iterative);
        }
        if self.is_iterative() {
            match self.prepare_iterative(k, &unknowns, progress) {
                Ok(()) => return Ok(()),
                Err(error) if self.may_fall_back(&error) => {
                    progress.event(ProgressEvent::Warning(format!(
                        "iterative solver: {}; solving directly instead",
                        message(&error)
                    )));
                    self.switch(false);
                    analyze = true;
                }
                Err(error) => return Err(error),
            }
        }
        // A factor of an earlier matrix of this pattern serves on, see `keeping_factors`.
        if self.keep_factors
            && !self.refresh
            && direct.is_factorized()
            && (self.factored.as_ref())
                .is_some_and(|p| Arc::ptr_eq(p, k.pattern()) || **p == **k.pattern())
        {
            self.stale = Some(k.clone());
            return Ok(());
        }
        self.stale = None;
        let started = std::time::Instant::now();
        let direct_result = (|| {
            if analyze {
                direct.set_coordinates(Some(unknowns.coordinates.clone()));
                direct.analyze(k.pattern())?;
                progress.event(ProgressEvent::Info(format!(
                    "factor: {} nonzeros, {:.2e} flops, ordered in {:.2} s",
                    direct.factor_nonzeros().unwrap_or(0),
                    direct.factor_flops().unwrap_or(0.0),
                    started.elapsed().as_secs_f64()
                )));
            }
            direct.factorize(k)
        })();
        match direct_result {
            Ok(()) => {
                self.factored = Some(k.pattern().clone());
                self.refresh = false;
                progress.event(ProgressEvent::Info(format!(
                    "stiffness factorized in {:.2} s",
                    started.elapsed().as_secs_f64()
                )));
                Ok(())
            }
            Err(error) if is_out_of_memory(&error) && self.may_fall_back(&error) => {
                progress.event(ProgressEvent::Warning(format!(
                    "{}; solving iteratively instead",
                    message(&error)
                )));
                self.switch(true);
                self.prepare_iterative(k, &unknowns, progress)
            }
            Err(error) if is_out_of_memory(&error) && self.mode == SolverMode::Direct => {
                Err(with_message(error, |m| {
                    format!("{m}; the iterative or automatic equation solver needs far less")
                }))
            }
            Err(error) => Err(error),
        }
    }

    /// Solves `K x = b` in place with the last prepared matrix.
    pub fn solve(
        &mut self,
        direct: &mut LdltSolver,
        rhs: &mut [f64],
        progress: &mut dyn Progress,
    ) -> Result<()> {
        let started = std::time::Instant::now();
        if self.is_iterative() {
            let pcg = self.pcg.as_mut().expect("prepared");
            pcg.set_tolerance(self.tolerance.unwrap_or(plxs_linalg::pcg::TOLERANCE));
            // The right-hand side, for a second attempt (the solve works in place).
            let b = (self.kept.is_some() || pcg.is_reused()).then(|| rhs.to_vec());
            // The automatic choice weighs a slow iteration against the direct solver once
            // (see `weigh_probe`).
            pcg.set_probe(
                self.mode == SolverMode::Automatic
                    && !self.fell_back
                    && !self.probed
                    && self.kept.is_some(),
            );
            let mut result = pcg.solve_counted(rhs);
            if result.is_err()
                && let Some(probe) = pcg.take_probe()
            {
                self.probed = true;
                let b = b.as_ref().expect("kept for the second attempt");
                rhs.copy_from_slice(b);
                if self.weigh_probe(direct, probe, progress)? {
                    // Solved directly below.
                    return self.solve_after_switch(direct, rhs, started, progress);
                }
                let pcg = self.pcg.as_mut().expect("prepared");
                pcg.set_probe(false);
                result = pcg.solve_counted(rhs);
            }
            let pcg = self.pcg.as_mut().expect("prepared");
            if result.is_err() && pcg.is_reused() {
                // The hierarchy kept from the previous factorization does not fit this
                // matrix after all: build it anew and try once more.
                rhs.copy_from_slice(b.as_ref().expect("kept for the second attempt"));
                pcg.rebuild_preconditioner()?;
                result = pcg.solve_counted(rhs);
            }
            let b = b.filter(|_| self.kept.is_some());
            match result {
                Ok(stats) => {
                    progress.event(ProgressEvent::LinearSolve {
                        iterations: stats.iterations as u32,
                        relative_residual: stats.relative_residual,
                        seconds: started.elapsed().as_secs_f64(),
                        quiet: false,
                    });
                    return Ok(());
                }
                Err(error) if self.may_fall_back(&error) && b.is_some() => {
                    let (pattern, coordinates) = self.kept.take().expect("kept for the fall-back");
                    let k = (self.pcg.as_ref())
                        .and_then(|pcg| pcg.matrix(pattern))
                        .expect("the iterative solver holds the prepared matrix");
                    direct.set_coordinates(Some(coordinates));
                    if !self.factor_fits(direct, &k, &error, progress)? {
                        return Err(with_message(error, |m| {
                            format!(
                                "{m}; the factor of the direct solver does not fit into \
                                 the memory either: support the model better (bodies held \
                                 only by contact or weak springs slow the iteration down)"
                            )
                        }));
                    }
                    progress.event(ProgressEvent::Warning(format!(
                        "iterative solver: {}; solving directly instead",
                        message(&error)
                    )));
                    self.switch(false);
                    direct.analyze(k.pattern())?;
                    direct.factorize(&k)?;
                    rhs.copy_from_slice(&b.expect("checked"));
                }
                Err(error) => return Err(error),
            }
        }
        if let Some(k) = self.stale.take() {
            let b = rhs.to_vec();
            let tolerance = self.tolerance.unwrap_or(KEPT_FACTOR_TOLERANCE);
            // A solve with the factor costs about its nonzeros, the factorization about
            // their power 1.5: the iterations worth a share of a factorization (counted,
            // not timed, so that a run repeats itself).
            let budget = direct
                .factor_nonzeros()
                .map_or(KEPT_FACTOR_FEWEST, |nonzeros| {
                    ((nonzeros as f64).sqrt() / KEPT_FACTOR_SOLVES) as usize
                });
            let budget = budget.clamp(KEPT_FACTOR_FEWEST, KEPT_FACTOR_ITERATIONS);
            match kept_factor_solve(direct, &k, rhs, tolerance, 2 * budget) {
                Ok((iterations, relative_residual)) => {
                    if iterations > budget {
                        self.refresh = true;
                    }
                    progress.event(ProgressEvent::LinearSolve {
                        iterations: iterations as u32,
                        relative_residual,
                        seconds: started.elapsed().as_secs_f64(),
                        quiet: true,
                    });
                    return Ok(());
                }
                Err(Error::Cancelled) => return Err(Error::Cancelled),
                Err(_) => {
                    // The kept factor does not fit this matrix: its own factor solves.
                    rhs.copy_from_slice(&b);
                    direct.factorize(&k)?;
                    self.refresh = false;
                }
            }
        }
        direct.solve(rhs)?;
        progress.event(ProgressEvent::Info(format!(
            "solved in {:.2} s",
            started.elapsed().as_secs_f64()
        )));
        Ok(())
    }

    /// Solves the unsymmetric system `A x = b` (`apply(v, out) = A v`) whose symmetric
    /// part is the last prepared matrix `k`: GMRES preconditioned by `k`, for the tangent
    /// of sliding friction.
    ///
    /// Iterative, `k` preconditions by a loose solve with the multigrid
    /// ([`PRECONDITIONER_TOLERANCE`], at most [`PRECONDITIONER_ITERATIONS`]): one digit
    /// suffices, GMRES resolves the rest. When GMRES still misses [`UNSYMMETRIC_ACCEPTED`]
    /// the multigrid is built anew for `k`, then the automatic choice factorizes `k`
    /// directly, but only when the factor fits into the memory; failing all that, the best
    /// iterate goes on to the Newton iteration with a warning.
    pub fn solve_unsymmetric(
        &mut self,
        direct: &mut LdltSolver,
        k: &SparseMatrix,
        unknowns: impl FnOnce() -> Unknowns,
        apply: &dyn Fn(&[f64], &mut [f64]),
        b: &[f64],
        progress: &mut dyn Progress,
    ) -> Result<Vec<f64>> {
        // GMRES is preconditioned by the factor of `k` itself.
        if self.stale.take().is_some() {
            direct.factorize(k)?;
            self.refresh = false;
        }
        let attempt = |equations: &Self, direct: &LdltSolver| {
            let precondition = |v: &mut [f64]| equations.apply_inverse(direct, v);
            plxs_linalg::gmres(apply, &precondition, b, UNSYMMETRIC_TOLERANCE, 60, 300)
        };
        let mut result = attempt(self, direct);
        if !accepted(&result) && self.is_iterative() {
            if let Some(pcg) = self.pcg.as_mut() {
                pcg.rebuild_preconditioner()?;
            }
            result = attempt(self, direct);
        }
        if !accepted(&result) && self.is_iterative() {
            let error = match &result {
                Err(error) => error.clone(),
                Ok((_, residual)) => plxs_core::Error::numerical(
                    plxs_core::Failure::IterativeSolver,
                    format!(
                        "GMRES on the unsymmetric tangent of sliding friction reached only a \
                         relative residual of {residual:.1e}"
                    ),
                ),
            };
            if self.fall_back_to_direct(direct, k, unknowns, &error, progress)? {
                result = attempt(self, direct);
            }
        }
        match result {
            Ok((x, residual)) => {
                if residual > UNSYMMETRIC_ACCEPTED {
                    progress.event(ProgressEvent::Warning(format!(
                        "GMRES on the unsymmetric tangent of sliding friction reached only a \
                         relative residual of {residual:.1e}; the Newton iteration goes on \
                         with it"
                    )));
                }
                Ok(x)
            }
            Err(error) => Err(error),
        }
    }

    /// After the iterative solver failed as the preconditioner of GMRES, factorizes `k`
    /// directly from now on, as [`Self::solve`] falls back. Returns false (and does
    /// nothing) unless the automatic choice may still fall back and the factor fits into
    /// the memory.
    fn fall_back_to_direct(
        &mut self,
        direct: &mut LdltSolver,
        k: &SparseMatrix,
        unknowns: impl FnOnce() -> Unknowns,
        error: &Error,
        progress: &mut dyn Progress,
    ) -> Result<bool> {
        if !self.is_iterative() || !self.may_fall_back(error) {
            return Ok(false);
        }
        direct.set_coordinates(Some(unknowns().coordinates));
        if !self.factor_fits(direct, k, error, progress)? {
            return Ok(false);
        }
        progress.event(ProgressEvent::Warning(format!(
            "iterative solver: {}; solving directly instead",
            message(error)
        )));
        self.switch(false);
        direct.analyze(k.pattern())?;
        direct.factorize(k)?;
        Ok(true)
    }

    /// Weighs the rest of a slow iteration (`probe`) against the direct solver: the
    /// symbolic analysis tells the factor's memory and floating point operations, from
    /// which [`FACTOR_FLOPS_PER_THREAD`] and [`TOUCH_BYTES_PER_SECOND`] estimate the time
    /// of the factorization; the iteration would take its predicted iterations at its
    /// pace so far. Switches to the direct solver (and factorizes) when that is cheaper
    /// and the factor fits; `true` then.
    fn weigh_probe(
        &mut self,
        direct: &mut LdltSolver,
        probe: plxs_linalg::PcgProbe,
        progress: &mut dyn Progress,
    ) -> Result<bool> {
        let Some((pattern, coordinates)) = self.kept.clone() else {
            return Ok(false);
        };
        let started = std::time::Instant::now();
        direct.set_coordinates(Some(coordinates));
        let bytes = direct.estimate_factor_bytes(&pattern)?;
        let flops = direct.factor_flops().unwrap_or(0.0);
        let threads = rayon::current_num_threads().max(1) as f64;
        let direct_seconds =
            flops / (FACTOR_FLOPS_PER_THREAD * threads) + bytes as f64 / TOUCH_BYTES_PER_SECOND;
        let iterative_seconds = probe.predicted as f64 * probe.seconds_per_iteration;
        let need = bytes.saturating_add(bytes / 4);
        let available = self
            .available
            .or_else(plxs_linalg::available_memory)
            .unwrap_or(ASSUMED_MEMORY);
        let fits = (need as f64) <= MEMORY_SHARE * available as f64;
        let switch = fits && direct_seconds < iterative_seconds;
        progress.event(ProgressEvent::Info(format!(
            "automatic equation solver: the iteration would take about {} iterations, \
             {iterative_seconds:.1} s; the direct solver about {direct_seconds:.1} s with a \
             factor of {:.1} GB{} (analyzed in {:.2} s); solving {}",
            probe.predicted,
            need as f64 / 1e9,
            if fits {
                ""
            } else {
                ", more than fits into the memory"
            },
            started.elapsed().as_secs_f64(),
            if switch { "directly" } else { "iteratively on" }
        )));
        if !switch {
            return Ok(false);
        }
        let k = (self.pcg.as_ref())
            .and_then(|pcg| pcg.matrix(pattern))
            .expect("the iterative solver holds the prepared matrix");
        self.switch(false);
        direct.analyze(k.pattern())?;
        direct.factorize(&k)?;
        Ok(true)
    }

    /// The direct solve of `rhs` right after the switch of [`Self::weigh_probe`].
    fn solve_after_switch(
        &mut self,
        direct: &mut LdltSolver,
        rhs: &mut [f64],
        started: std::time::Instant,
        progress: &mut dyn Progress,
    ) -> Result<()> {
        direct.solve(rhs)?;
        progress.event(ProgressEvent::Info(format!(
            "solved in {:.2} s",
            started.elapsed().as_secs_f64()
        )));
        Ok(())
    }

    /// Whether the factor of `k` fits into the memory, for a fall-back from the iterative
    /// solver after `error`; tells the monitor when it does not. `direct` has its
    /// coordinates.
    fn factor_fits(
        &mut self,
        direct: &mut LdltSolver,
        k: &SparseMatrix,
        error: &Error,
        progress: &mut dyn Progress,
    ) -> Result<bool> {
        let mut quiet = FirstOrWarnings {
            inner: progress,
            all: false,
        };
        if !factor_exceeds_memory(direct, k, self.available, &mut quiet)? {
            return Ok(true);
        }
        progress.event(ProgressEvent::Warning(format!(
            "iterative solver: {}; the direct solver would take over, but its factor does \
             not fit into the memory",
            message(error)
        )));
        // Once is enough: the next failure does not estimate again.
        self.fell_back = true;
        self.kept = None;
        Ok(false)
    }

    /// `v ≈ K⁻¹ v` with the last prepared matrix, without the fall-back of
    /// [`Self::solve`]: the preconditioner of GMRES (sliding friction). Iterative, a loose
    /// solve that fails only when the matrix is not positive definite.
    pub fn apply_inverse(&self, direct: &LdltSolver, v: &mut [f64]) -> Result<()> {
        match (&self.pcg, self.is_iterative()) {
            (Some(pcg), true) => pcg
                .solve_loosely(v, PRECONDITIONER_TOLERANCE, PRECONDITIONER_ITERATIONS)
                .map(|_| ()),
            _ => direct.solve(v),
        }
    }

    /// Lowers the size from which the automatic choice considers the iterative solver and
    /// sets the memory it works with (tests).
    #[doc(hidden)]
    pub fn set_automatic_limits(&mut self, from: usize, available: u64) {
        self.automatic_from = from;
        self.available = Some(available);
    }

    fn choose(
        &mut self,
        direct: &mut LdltSolver,
        k: &SparseMatrix,
        unknowns: &Unknowns,
        progress: &mut dyn Progress,
    ) -> Result<bool> {
        let n = k.dimension();
        if n < self.automatic_from {
            return Ok(false);
        }
        // Translations or a scalar field only: the multigrid converges fast, and the
        // direct solver stands by.
        let solid = !unknowns.component.is_empty() && unknowns.component.iter().all(|&c| c < 3);
        if solid && !self.prefer_direct {
            progress.event(ProgressEvent::Info(format!(
                "automatic equation solver: {n} equations of translations only; solving \
                 iteratively (the direct solver takes over if the iteration fails)"
            )));
            return Ok(true);
        }
        direct.set_coordinates(Some(unknowns.coordinates.clone()));
        factor_exceeds_memory(direct, k, self.available, progress)
    }

    fn prepare_iterative(
        &mut self,
        k: &SparseMatrix,
        unknowns: &Unknowns,
        progress: &mut dyn Progress,
    ) -> Result<()> {
        let started = std::time::Instant::now();
        let pcg = self.pcg.get_or_insert_with(PcgSolver::new);
        if self.mode == SolverMode::Automatic {
            pcg.set_max_iterations(AUTOMATIC_MAX_ITERATIONS);
            // The direct solver stands by (once): an iteration that stalls hands over
            // early instead of after the whole limit.
            pcg.set_give_up_on_stall(!self.fell_back);
        }
        pcg.set_unknowns(Some(unknowns.clone()));
        // Skipped for the pattern (and unknowns) analyzed last.
        pcg.analyze(k.pattern())?;
        pcg.factorize(k)?;
        progress.event(ProgressEvent::Info(format!(
            "iterative solver: preconditioner {}, {} nonzeros, built in {:.2} s",
            pcg.preconditioner(),
            pcg.preconditioner_nonzeros().unwrap_or(0),
            started.elapsed().as_secs_f64()
        )));
        if self.mode == SolverMode::Automatic && !self.fell_back {
            self.kept = Some((k.pattern().clone(), unknowns.coordinates.clone()));
        }
        Ok(())
    }

    /// Whether the automatic choice may still switch to the other solver after `error`.
    fn may_fall_back(&self, error: &Error) -> bool {
        self.mode == SolverMode::Automatic
            && !self.fell_back
            && matches!(error, Error::Numerical { .. })
    }

    fn switch(&mut self, iterative: bool) {
        self.iterative = Some(iterative);
        self.fell_back = true;
        self.kept = None;
        if !iterative {
            // The preconditioner's memory is not needed any more.
            self.pcg = None;
        }
    }
}

/// Solves `K x = b` in place by conjugate gradients preconditioned with the factor
/// `direct` holds (that of an earlier matrix of the same pattern), to the relative residual
/// `tolerance`. Returns the iterations and the relative residual; fails when the matrix or
/// the factor turns out not to be positive definite or the iteration does not get there
/// within `limit` iterations.
fn kept_factor_solve(
    direct: &LdltSolver,
    k: &SparseMatrix,
    rhs: &mut [f64],
    tolerance: f64,
    limit: usize,
) -> Result<(usize, f64)> {
    let failed = |what: &str| Error::numerical(plxs_core::Failure::IterativeSolver, what);
    let dot = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(a, b)| a * b).sum::<f64>();
    let n = rhs.len();
    let b_norm = dot(rhs, rhs).sqrt();
    if b_norm == 0.0 {
        return Ok((0, 0.0));
    }
    if !b_norm.is_finite() {
        return Err(failed("the right-hand side is not finite"));
    }
    let cancel = plxs_core::CancelToken::current();
    let mut r = rhs.to_vec();
    let mut x = vec![0.0; n];
    let mut z = r.clone();
    direct.solve(&mut z)?;
    let mut p = z.clone();
    let mut q = vec![0.0; n];
    let mut rz = dot(&r, &z);
    for iteration in 1..=limit {
        if cancel
            .as_ref()
            .is_some_and(plxs_core::CancelToken::is_cancelled)
        {
            return Err(Error::Cancelled);
        }
        k.multiply(&p, &mut q);
        let pq = dot(&p, &q);
        if rz <= 0.0 || pq <= 0.0 || !pq.is_finite() {
            return Err(failed("not positive definite with the kept factor"));
        }
        let alpha = rz / pq;
        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * q[i];
        }
        let residual = dot(&r, &r).sqrt() / b_norm;
        if residual <= tolerance {
            rhs.copy_from_slice(&x);
            return Ok((iteration, residual));
        }
        z.copy_from_slice(&r);
        direct.solve(&mut z)?;
        let rz_new = dot(&r, &z);
        let beta = rz_new / rz;
        rz = rz_new;
        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
    }
    Err(failed("the kept factor does not precondition this matrix"))
}

fn message(error: &Error) -> String {
    match error {
        Error::Numerical { message, .. } => message.clone(),
        other => other.to_string(),
    }
}

fn accepted(result: &Result<(Vec<f64>, f64)>) -> bool {
    matches!(result, Ok((_, residual)) if *residual <= UNSYMMETRIC_ACCEPTED)
}

fn with_message(error: Error, f: impl FnOnce(&str) -> String) -> Error {
    error.map_message(|message| f(&message))
}

/// The automatic choice for a system of `k`'s pattern: whether its factor (estimated by
/// the symbolic analysis on `direct`, which keeps it) with a quarter more for the work
/// space exceeds [`MEMORY_SHARE`] of the available memory (`available`, else the
/// platform's). Tells the monitor what it found.
/// The unknowns of the free displacement DOFs `free` (three per node) of `model`, with the
/// nodes' positions and bodies, for the solvers (the ordering of the direct one, the
/// multigrid of the iterative one; see [`plxs_core::Model::node_bodies`]).
pub fn displacement_unknowns(model: &plxs_core::Model, free: &[u32]) -> Unknowns {
    Unknowns::from_free(free, 3, &model.nodes.coordinates)
        .with_bodies(&model.node_bodies())
        .with_midsides(&model.midside_ends())
        .with_fibres(&model.fibre_of_nodes())
        .with_roles(&model.node_roles())
}

pub fn factor_exceeds_memory(
    direct: &mut LdltSolver,
    k: &SparseMatrix,
    available: Option<u64>,
    progress: &mut dyn Progress,
) -> Result<bool> {
    let n = k.dimension();
    let started = std::time::Instant::now();
    let factor = direct.estimate_factor_bytes(k.pattern())?;
    let need = factor.saturating_add(factor / 4);
    let (available, known) = match available.or_else(plxs_linalg::available_memory) {
        Some(bytes) => (bytes, true),
        None => (ASSUMED_MEMORY, false),
    };
    let iterative = need as f64 > MEMORY_SHARE * available as f64;
    progress.event(ProgressEvent::Info(format!(
        "automatic equation solver: the factor of {n} equations needs {:.1} GB, {:.1} GB {}; \
         solving {} (estimated in {:.2} s)",
        need as f64 / 1e9,
        available as f64 / 1e9,
        if known { "available" } else { "assumed" },
        if iterative { "iteratively" } else { "directly" },
        started.elapsed().as_secs_f64()
    )));
    Ok(iterative)
}

/// Passes the warnings of a solver on, and its other messages only when `all`: for
/// procedures that solve many systems a step (Newton iterations, time increments).
pub struct FirstOrWarnings<'a> {
    pub inner: &'a mut dyn Progress,
    pub all: bool,
}

impl Progress for FirstOrWarnings<'_> {
    fn event(&mut self, event: ProgressEvent) {
        match event {
            // The monitor plots every solve, the output tells only the first.
            ProgressEvent::LinearSolve {
                iterations,
                relative_residual,
                seconds,
                ..
            } => self.inner.event(ProgressEvent::LinearSolve {
                iterations,
                relative_residual,
                seconds,
                quiet: !self.all,
            }),
            event if self.all || matches!(event, ProgressEvent::Warning(_)) => {
                self.inner.event(event)
            }
            _ => {}
        }
    }
}

/// A numerical error of the linear solver, placed in the step and increment it happened in.
pub fn in_increment(error: Error, step: u32, increment: u32) -> Error {
    error.in_increment(step, increment)
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use plxs_core::progress::NoProgress;
    use plxs_linalg::Pattern;

    use super::*;

    /// The tridiagonal matrix of a chain of `n` springs fixed at one end, scaled by
    /// `sign` on the diagonal of the last unknown (negative: indefinite).
    fn chain(n: usize, last: f64) -> SparseMatrix {
        let blocks: Vec<Vec<u32>> = (0..n - 1).map(|i| vec![i as u32, i as u32 + 1]).collect();
        let pattern = Pattern::from_blocks(n, blocks.iter().map(|b| b.as_slice()));
        let mut k = SparseMatrix::zeros(Arc::new(pattern));
        for i in 0..n {
            k.add(i as u32, i as u32, if i + 1 == n { last } else { 2.0 });
            if i + 1 < n {
                k.add(i as u32, i as u32 + 1, -1.0);
            }
        }
        k
    }

    fn coordinates(n: usize) -> impl FnOnce() -> Unknowns {
        move || {
            let free: Vec<u32> = (0..n as u32).collect();
            let coordinates: Vec<[f64; 3]> = (0..n).map(|i| [i as f64, 0.0, 0.0]).collect();
            Unknowns::from_free(&free, 1, &coordinates)
        }
    }

    fn residual(k: &SparseMatrix, x: &[f64], b: &[f64]) -> f64 {
        let mut kx = vec![0.0; x.len()];
        k.multiply(x, &mut kx);
        kx.iter()
            .zip(b)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max)
    }

    fn solve(equations: &mut Equations, k: &SparseMatrix) -> Vec<f64> {
        let n = k.dimension();
        let mut direct = LdltSolver::new(1);
        equations
            .prepare(&mut direct, k, coordinates(n), true, &mut NoProgress)
            .unwrap();
        let b: Vec<f64> = (0..n).map(|i| 1.0 + i as f64).collect();
        let mut x = b.clone();
        equations
            .solve(&mut direct, &mut x, &mut NoProgress)
            .unwrap();
        assert!(
            residual(k, &x, &b) < 1e-8,
            "residual {}",
            residual(k, &x, &b)
        );
        x
    }

    /// Prepares and solves `k` with `direct`; whether a kept factor served, and the
    /// residual relative to the right-hand side.
    fn solve_keeping(
        equations: &mut Equations,
        direct: &mut LdltSolver,
        k: &SparseMatrix,
    ) -> (bool, f64) {
        let n = k.dimension();
        equations
            .prepare(direct, k, coordinates(n), true, &mut NoProgress)
            .unwrap();
        let kept = equations.stale.is_some();
        let b: Vec<f64> = (0..n).map(|i| 1.0 + i as f64).collect();
        let mut x = b.clone();
        equations.solve(direct, &mut x, &mut NoProgress).unwrap();
        (kept, residual(k, &x, &b) / (n as f64))
    }

    #[test]
    fn a_kept_factor_solves_the_current_matrix() {
        let n = 200;
        let mut equations = Equations::new(EquationSolver::Spooles).keeping_factors();
        let mut direct = LdltSolver::new(1);
        let (kept, relative) = solve_keeping(&mut equations, &mut direct, &chain(n, 2.0));
        assert!(!kept && relative < 1e-9);
        // A stiffer end: the factor of the first matrix stays, the solution is that of
        // the second.
        let (kept, relative) = solve_keeping(&mut equations, &mut direct, &chain(n, 3.0));
        assert!(kept && relative < 1e-9, "{relative}");
        // With a tolerance the solve stops early, within that tolerance.
        equations.set_tolerance(Some(1e-3));
        let (kept, relative) = solve_keeping(&mut equations, &mut direct, &chain(n, 2.5));
        assert!(kept && relative < 1e-3, "{relative}");
    }

    #[test]
    fn an_indefinite_matrix_is_factorized_instead_of_iterated_on_a_kept_factor() {
        let n = 50;
        let mut equations = Equations::new(EquationSolver::Spooles).keeping_factors();
        let mut direct = LdltSolver::new(1);
        solve_keeping(&mut equations, &mut direct, &chain(n, 2.0));
        // Conjugate gradients stop at the negative curvature; the factorization of the
        // matrix itself still solves it.
        let (kept, relative) = solve_keeping(&mut equations, &mut direct, &chain(n, -1.0));
        assert!(kept && relative < 1e-9, "{relative}");
        // And is the factor at hand from then on.
        let (_, relative) = solve_keeping(&mut equations, &mut direct, &chain(n, -1.0));
        assert!(relative < 1e-9, "{relative}");
    }

    #[test]
    fn automatic_is_iterative_for_large_systems_of_translations() {
        let k = chain(200, 1.0);
        let mut equations = Equations::new(EquationSolver::Default);
        equations.set_automatic_limits(10, 1 << 30);
        let x = solve(&mut equations, &k);
        assert!(equations.is_iterative());
        let mut direct = Equations::new(EquationSolver::Pastix);
        let reference = solve(&mut direct, &k);
        for (a, b) in x.iter().zip(&reference) {
            assert!((a - b).abs() <= 1e-8 * b.abs().max(1.0));
        }
        // Small systems do not even estimate.
        let mut equations = Equations::new(EquationSolver::Default);
        equations.set_automatic_limits(1000, 0);
        solve(&mut equations, &k);
        assert!(!equations.is_iterative());
    }

    #[test]
    fn automatic_is_direct_while_the_factor_fits_when_direct_is_preferred() {
        let k = chain(200, 1.0);
        let mut equations = Equations::new(EquationSolver::Default).preferring_direct();
        equations.set_automatic_limits(10, 1 << 30);
        solve(&mut equations, &k);
        assert!(!equations.is_iterative());
        // Unless the factor does not fit.
        let mut equations = Equations::new(EquationSolver::Default).preferring_direct();
        equations.set_automatic_limits(10, 100);
        solve(&mut equations, &k);
        assert!(equations.is_iterative());
    }

    #[test]
    fn automatic_is_direct_for_expanded_shells_while_the_factor_fits() {
        // Translations only, but of shells expanded into solids: the memory decides.
        let k = chain(200, 1.0);
        let mut model = plxs_core::Model::default();
        model.expanded_structure = true;
        let mut equations = Equations::new(EquationSolver::Default).for_model(&model);
        equations.set_automatic_limits(10, 1 << 30);
        solve(&mut equations, &k);
        assert!(!equations.is_iterative());
        let mut equations = Equations::new(EquationSolver::Default).for_model(&model);
        equations.set_automatic_limits(10, 100);
        solve(&mut equations, &k);
        assert!(equations.is_iterative());
        // A solid model goes iterative.
        let mut equations =
            Equations::new(EquationSolver::Default).for_model(&plxs_core::Model::default());
        equations.set_automatic_limits(10, 1 << 30);
        solve(&mut equations, &k);
        assert!(equations.is_iterative());
    }

    #[test]
    fn automatic_is_direct_for_unknowns_with_rotations() {
        // Six unknowns per node (shells, beams): the memory decides, and it fits.
        let n = 198;
        let k = chain(n, 1.0);
        let mut equations = Equations::new(EquationSolver::Default);
        equations.set_automatic_limits(10, 1 << 30);
        let mut direct = LdltSolver::new(1);
        let unknowns = || {
            let free: Vec<u32> = (0..n as u32).collect();
            let coordinates: Vec<[f64; 3]> = (0..n / 6).map(|i| [i as f64, 0.0, 0.0]).collect();
            Unknowns::from_free(&free, 6, &coordinates)
        };
        equations
            .prepare(&mut direct, &k, unknowns, true, &mut NoProgress)
            .unwrap();
        assert!(!equations.is_iterative());
    }

    #[test]
    fn automatic_goes_iterative_when_the_factor_does_not_fit() {
        let k = chain(200, 1.0);
        let mut equations = Equations::new(EquationSolver::Default);
        equations.set_automatic_limits(10, 100);
        let x = solve(&mut equations, &k);
        assert!(equations.is_iterative());
        let mut direct = Equations::new(EquationSolver::Pastix);
        let reference = solve(&mut direct, &k);
        for (a, b) in x.iter().zip(&reference) {
            assert!((a - b).abs() <= 1e-8 * b.abs().max(1.0));
        }
    }

    #[test]
    fn automatic_falls_back_to_direct_on_an_indefinite_matrix() {
        // Positive diagonal (the preconditioner builds), but indefinite: CG breaks down.
        let k = chain(200, 0.2);
        let mut equations = Equations::new(EquationSolver::Default);
        equations.set_automatic_limits(10, 1 << 30);
        solve(&mut equations, &k);
        assert!(!equations.is_iterative());
        // Unless the factor does not fit: then the failure says so.
        let mut equations = Equations::new(EquationSolver::Default);
        equations.set_automatic_limits(10, 100);
        let mut direct = LdltSolver::new(1);
        equations
            .prepare(&mut direct, &k, coordinates(200), true, &mut NoProgress)
            .unwrap();
        let mut x = vec![1.0; 200];
        let error = equations
            .solve(&mut direct, &mut x, &mut NoProgress)
            .unwrap_err();
        assert!(error.to_string().contains("does not fit"), "{error}");
        assert!(equations.is_iterative());
        // Asked for explicitly, the iterative solver reports the failure.
        let mut iterative = Equations::new(EquationSolver::IterativeCholesky);
        let mut direct = LdltSolver::new(1);
        iterative
            .prepare(&mut direct, &k, coordinates(200), true, &mut NoProgress)
            .unwrap();
        let mut x = vec![1.0; 200];
        assert!(
            iterative
                .solve(&mut direct, &mut x, &mut NoProgress)
                .is_err()
        );
    }

    #[test]
    fn an_unsymmetric_system_is_preconditioned_by_a_loose_iteration() {
        // GMRES around the multigrid's loose solve (sliding friction on a large solid
        // model): the inner iteration stops at a few digits, GMRES gets all of them.
        let n = 400;
        let k = chain(n, 1.0);
        let mut equations = Equations::new(EquationSolver::Default);
        equations.set_automatic_limits(10, 100);
        let mut direct = LdltSolver::new(1);
        equations
            .prepare(&mut direct, &k, coordinates(n), true, &mut NoProgress)
            .unwrap();
        assert!(equations.is_iterative());
        let apply = |v: &[f64], out: &mut [f64]| {
            k.multiply(v, out);
            out[3] += 0.5 * v[170];
            out[250] -= 0.3 * v[20];
        };
        let b: Vec<f64> = (0..n).map(|i| 1.0 + (i % 7) as f64).collect();
        let x = equations
            .solve_unsymmetric(&mut direct, &k, coordinates(n), &apply, &b, &mut NoProgress)
            .unwrap();
        let mut ax = vec![0.0; n];
        apply(&x, &mut ax);
        let r = ax
            .iter()
            .zip(&b)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt();
        let b_norm = b.iter().map(|b| b * b).sum::<f64>().sqrt();
        assert!(
            r <= UNSYMMETRIC_ACCEPTED * b_norm,
            "relative residual {}",
            r / b_norm
        );
        assert!(equations.is_iterative());
    }

    /// The five-point Laplacian of an `m` x `m` grid held at its first node, with the
    /// unknowns at the grid points; `last` on the diagonal of the last unknown.
    fn grid(m: usize, last: f64) -> (SparseMatrix, Vec<[f64; 3]>) {
        let n = m * m;
        let index = |i: usize, j: usize| (i * m + j) as u32;
        let mut blocks = Vec::new();
        for i in 0..m {
            for j in 0..m {
                if j + 1 < m {
                    blocks.push(vec![index(i, j), index(i, j + 1)]);
                }
                if i + 1 < m {
                    blocks.push(vec![index(i, j), index(i + 1, j)]);
                }
            }
        }
        let pattern = Pattern::from_blocks(n, blocks.iter().map(|b| b.as_slice()));
        let mut k = SparseMatrix::zeros(Arc::new(pattern));
        for block in &blocks {
            k.add(block[0], block[1], -1.0);
        }
        for d in 0..n {
            k.add(d as u32, d as u32, if d + 1 == n { last } else { 4.0 });
        }
        k.add(0, 0, 4.0);
        let coordinates = (0..n)
            .map(|d| [(d / m) as f64, (d % m) as f64, 0.0])
            .collect();
        (k, coordinates)
    }

    #[test]
    fn the_direct_solver_taking_over_orders_by_the_positions_of_the_unknowns() {
        // Indefinite: CG breaks down in the solve, and the direct solver takes over. Its
        // ordering bisects the grid by the positions, as when it solves from the start
        // (without them a fine mesh of thin shells filled the memory).
        let (k, coordinates) = grid(60, 0.5);
        let n = k.dimension();
        let unknowns = || Unknowns::from_free(&(0..n as u32).collect::<Vec<_>>(), 1, &coordinates);
        let factor = |mut equations: Equations| {
            let mut direct = LdltSolver::new(1);
            equations.set_automatic_limits(10, 1 << 30);
            equations
                .prepare(&mut direct, &k, unknowns, true, &mut NoProgress)
                .unwrap();
            let mut x = vec![1.0; n];
            equations
                .solve(&mut direct, &mut x, &mut NoProgress)
                .unwrap();
            assert!(!equations.is_iterative());
            direct.factor_nonzeros().unwrap()
        };
        let taken_over = factor(Equations::new(EquationSolver::Default));
        let direct = factor(Equations::new(EquationSolver::Pastix));
        assert_eq!(taken_over, direct);
    }
}
