//! The equation solver of a step: the direct LDLᵀ the session keeps, or conjugate gradients
//! when the step asks for an iterative solver (`SOLVER=ITERATIVE CHOLESKY` or `ITERATIVE
//! SCALING`). Only the procedures whose systems are symmetric positive definite use the
//! iterative one: linear static steps and heat transfer; all others solve directly, as
//! CalculiX does.

use plxs_core::{Error, Progress, ProgressEvent, Result, step::EquationSolver};
use plxs_linalg::{LdltSolver, LinearSolver, PcgSolver, SparseMatrix};

/// The solver of a step's systems; the direct one is borrowed for each call so that it can
/// live in the procedure context.
pub struct Equations {
    iterative: Option<PcgSolver>,
}

impl Equations {
    pub fn new(choice: EquationSolver) -> Self {
        Self {
            iterative: choice.is_iterative().then(PcgSolver::new),
        }
    }

    pub fn is_iterative(&self) -> bool {
        self.iterative.is_some()
    }

    /// Analyzes (when `analyze`, i.e. the pattern changed) and factorizes `k`, or builds
    /// the preconditioner; `coordinates` are the positions of the unknowns, for the
    /// ordering of the direct solver.
    pub fn prepare(
        &mut self,
        direct: &mut LdltSolver,
        k: &SparseMatrix,
        coordinates: impl FnOnce() -> Vec<[f64; 3]>,
        analyze: bool,
        progress: &mut dyn Progress,
    ) -> Result<()> {
        let started = std::time::Instant::now();
        match &mut self.iterative {
            Some(pcg) => {
                if analyze {
                    pcg.analyze(k.pattern())?;
                }
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
            }
            None => {
                if analyze {
                    direct.set_coordinates(Some(coordinates()));
                    direct.analyze(k.pattern())?;
                    progress.event(ProgressEvent::Info(format!(
                        "factor: {} nonzeros, ordered in {:.2} s",
                        direct.factor_nonzeros().unwrap_or(0),
                        started.elapsed().as_secs_f64()
                    )));
                }
                direct.factorize(k)?;
                progress.event(ProgressEvent::Info(format!(
                    "stiffness factorized in {:.2} s",
                    started.elapsed().as_secs_f64()
                )));
            }
        }
        Ok(())
    }

    /// Solves `K x = b` in place with the last prepared matrix.
    pub fn solve(
        &self,
        direct: &LdltSolver,
        rhs: &mut [f64],
        progress: &mut dyn Progress,
    ) -> Result<()> {
        let started = std::time::Instant::now();
        match &self.iterative {
            Some(pcg) => {
                let stats = pcg.solve_counted(rhs)?;
                progress.event(ProgressEvent::Info(format!(
                    "iterative solver: {} iterations, relative residual {:.1e}, {:.2} s",
                    stats.iterations,
                    stats.relative_residual,
                    started.elapsed().as_secs_f64()
                )));
            }
            None => {
                direct.solve(rhs)?;
                progress.event(ProgressEvent::Info(format!(
                    "solved in {:.2} s",
                    started.elapsed().as_secs_f64()
                )));
            }
        }
        Ok(())
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
