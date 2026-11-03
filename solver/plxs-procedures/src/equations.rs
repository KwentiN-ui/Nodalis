//! The equation solver of a step: the direct LDLᵀ the session keeps, conjugate gradients
//! when the step asks for an iterative solver (`SOLVER=ITERATIVE CHOLESKY` or `ITERATIVE
//! SCALING`), or the automatic choice between them when it names no solver.
//!
//! The procedures whose systems are symmetric positive definite use it: linear and
//! nonlinear static steps (with contact; the unsymmetric part of sliding friction is
//! resolved by GMRES around it), `*VISCO`, implicit `*DYNAMIC` steps and heat transfer. The
//! eigenvalue procedures solve directly.
//!
//! The automatic choice (no `SOLVER=`) is direct while the factor fits into the memory: for
//! a system of at least [`AUTOMATIC_FROM`] equations the symbolic analysis tells the size of
//! the factor before its memory is taken, and beyond [`MEMORY_SHARE`] of the available
//! memory the step solves iteratively. Each choice falls back to the other once when it
//! fails: a factor that does not fit after all goes iterative, an iteration that breaks
//! down or does not converge (an indefinite tangent near a limit point, a mechanism held
//! only by contact) goes direct.

use plxs_core::step::{EquationSolver, SolverMode};
use plxs_core::{Error, Progress, ProgressEvent, Result};
use plxs_linalg::{LdltSolver, LinearSolver, PcgSolver, SparseMatrix, is_out_of_memory};

/// The automatic choice considers the iterative solver from this many equations on;
/// smaller systems always fit and the direct solver is faster on them.
pub const AUTOMATIC_FROM: usize = 50_000;

/// The automatic choice solves iteratively when the factor (with a quarter more for the
/// work space of the factorization) would take more than this share of the available
/// memory.
pub const MEMORY_SHARE: f64 = 0.6;

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
    /// Automatic and iterative: the prepared matrix, to factorize it after all when the
    /// iteration fails.
    kept: Option<SparseMatrix>,
    /// Whether the automatic choice already fell back once (it does not switch back).
    fell_back: bool,
    /// The memory the automatic choice works with, `None`: ask the platform.
    available: Option<u64>,
    /// [`AUTOMATIC_FROM`], lowered by tests.
    automatic_from: usize,
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
        }
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
    /// builds the preconditioner; `coordinates` are the positions of the unknowns, for the
    /// ordering of the direct solver.
    pub fn prepare(
        &mut self,
        direct: &mut LdltSolver,
        k: &SparseMatrix,
        coordinates: impl FnOnce() -> Vec<[f64; 3]>,
        analyze: bool,
        progress: &mut dyn Progress,
    ) -> Result<()> {
        let mut coordinates = Some(coordinates);
        let mut analyze = analyze;
        if self.iterative.is_none() {
            let iterative = self.choose(direct, k, &mut coordinates, progress)?;
            self.iterative = Some(iterative);
        }
        if self.is_iterative() {
            match self.prepare_iterative(k, progress) {
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
        let started = std::time::Instant::now();
        let direct_result = (|| {
            if analyze {
                if let Some(coordinates) = coordinates.take() {
                    direct.set_coordinates(Some(coordinates()));
                }
                direct.analyze(k.pattern())?;
                progress.event(ProgressEvent::Info(format!(
                    "factor: {} nonzeros, ordered in {:.2} s",
                    direct.factor_nonzeros().unwrap_or(0),
                    started.elapsed().as_secs_f64()
                )));
            }
            direct.factorize(k)
        })();
        match direct_result {
            Ok(()) => {
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
                self.prepare_iterative(k, progress)
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
            let b = self.kept.is_some().then(|| rhs.to_vec());
            let pcg = self.pcg.as_ref().expect("prepared");
            match pcg.solve_counted(rhs) {
                Ok(stats) => {
                    progress.event(ProgressEvent::Info(format!(
                        "iterative solver: {} iterations, relative residual {:.1e}, {:.2} s",
                        stats.iterations,
                        stats.relative_residual,
                        started.elapsed().as_secs_f64()
                    )));
                    return Ok(());
                }
                Err(error) if self.may_fall_back(&error) && b.is_some() => {
                    progress.event(ProgressEvent::Warning(format!(
                        "iterative solver: {}; solving directly instead",
                        message(&error)
                    )));
                    let k = self.kept.take().expect("kept for the fall-back");
                    self.switch(false);
                    direct.analyze(k.pattern())?;
                    direct.factorize(&k)?;
                    rhs.copy_from_slice(&b.expect("checked"));
                }
                Err(error) => return Err(error),
            }
        }
        direct.solve(rhs)?;
        progress.event(ProgressEvent::Info(format!(
            "solved in {:.2} s",
            started.elapsed().as_secs_f64()
        )));
        Ok(())
    }

    /// `v ← K⁻¹ v` with the last prepared matrix, without the fall-back of
    /// [`Self::solve`]: the preconditioner of GMRES (sliding friction).
    pub fn apply_inverse(&self, direct: &LdltSolver, v: &mut [f64]) -> Result<()> {
        match (&self.pcg, self.is_iterative()) {
            (Some(pcg), true) => pcg.solve(v),
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
        coordinates: &mut Option<impl FnOnce() -> Vec<[f64; 3]>>,
        progress: &mut dyn Progress,
    ) -> Result<bool> {
        let n = k.dimension();
        if n < self.automatic_from {
            return Ok(false);
        }
        let started = std::time::Instant::now();
        if let Some(coordinates) = coordinates.take() {
            direct.set_coordinates(Some(coordinates()));
        }
        let factor = direct.estimate_factor_bytes(k.pattern())?;
        let need = factor.saturating_add(factor / 4);
        let (available, known) = match self.available.or_else(plxs_linalg::available_memory) {
            Some(bytes) => (bytes, true),
            None => (ASSUMED_MEMORY, false),
        };
        let iterative = need as f64 > MEMORY_SHARE * available as f64;
        progress.event(ProgressEvent::Info(format!(
            "automatic equation solver: the factor of {n} equations needs {:.1} GB, {:.1} GB \
             {}; solving {} (estimated in {:.2} s)",
            need as f64 / 1e9,
            available as f64 / 1e9,
            if known { "available" } else { "assumed" },
            if iterative { "iteratively" } else { "directly" },
            started.elapsed().as_secs_f64()
        )));
        Ok(iterative)
    }

    fn prepare_iterative(&mut self, k: &SparseMatrix, progress: &mut dyn Progress) -> Result<()> {
        let started = std::time::Instant::now();
        let pcg = self.pcg.get_or_insert_with(PcgSolver::new);
        // Skipped for the pattern analyzed last.
        pcg.analyze(k.pattern())?;
        pcg.factorize(k)?;
        let shift = pcg.shift();
        let kind = if shift.is_infinite() {
            "diagonal (incomplete Cholesky broke down)".to_owned()
        } else if shift > 0.0 {
            format!("incomplete Cholesky, diagonal shifted by {shift:.0e}")
        } else {
            "incomplete Cholesky".to_owned()
        };
        progress.event(ProgressEvent::Info(format!(
            "iterative solver: preconditioner {kind}, {} nonzeros, built in {:.2} s",
            pcg.preconditioner_nonzeros().unwrap_or(0),
            started.elapsed().as_secs_f64()
        )));
        if self.mode == SolverMode::Automatic && !self.fell_back {
            self.kept = Some(k.clone());
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

fn message(error: &Error) -> String {
    match error {
        Error::Numerical { message, .. } => message.clone(),
        other => other.to_string(),
    }
}

fn with_message(error: Error, f: impl FnOnce(&str) -> String) -> Error {
    match error {
        Error::Numerical {
            step,
            increment,
            message,
        } => Error::Numerical {
            step,
            increment,
            message: f(&message),
        },
        other => other,
    }
}

/// Passes the warnings of a solver on, and its other messages only when `all`: for
/// procedures that solve many systems a step (Newton iterations, time increments).
pub struct FirstOrWarnings<'a> {
    pub inner: &'a mut dyn Progress,
    pub all: bool,
}

impl Progress for FirstOrWarnings<'_> {
    fn event(&mut self, event: ProgressEvent) {
        if self.all || matches!(event, ProgressEvent::Warning(_)) {
            self.inner.event(event);
        }
    }
}

/// A numerical error of the linear solver, placed in the step and increment it happened in.
pub fn in_increment(error: Error, step: u32, increment: u32) -> Error {
    match error {
        Error::Numerical { message, .. } => Error::Numerical {
            step,
            increment,
            message,
        },
        other => other,
    }
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

    fn coordinates(n: usize) -> impl FnOnce() -> Vec<[f64; 3]> {
        move || (0..n).map(|i| [i as f64, 0.0, 0.0]).collect()
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

    #[test]
    fn automatic_is_direct_while_the_factor_fits() {
        let k = chain(200, 1.0);
        let mut equations = Equations::new(EquationSolver::Default);
        equations.set_automatic_limits(10, 1 << 30);
        solve(&mut equations, &k);
        assert!(!equations.is_iterative());
        // Small systems do not even estimate.
        let mut equations = Equations::new(EquationSolver::Default);
        equations.set_automatic_limits(1000, 0);
        solve(&mut equations, &k);
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
        equations.set_automatic_limits(10, 100);
        solve(&mut equations, &k);
        assert!(!equations.is_iterative());
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
}
