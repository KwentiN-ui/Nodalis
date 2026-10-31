//! Progress events a session reports while it computes. The command line prints them in
//! CalculiX's wording (PrePoLix's monitor reads that text), PrePoLix will consume them
//! directly in phase 2.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use serde::{Deserialize, Serialize};

/// Lets the caller stop a run: the procedures look at it between increments and iterations
/// and end the step with `StepOutcome::Cancelled`. Clones share the flag.
#[derive(Clone, Debug, Default)]
pub struct CancelToken(Arc<AtomicBool>);

impl CancelToken {
    pub fn new() -> Self {
        Self::default()
    }

    /// Asks the run to stop at the next opportunity.
    pub fn cancel(&self) {
        self.0.store(true, Ordering::Relaxed);
    }

    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }
}

/// Receives the events of a run. Implementations must be cheap: events come per iteration.
pub trait Progress {
    fn event(&mut self, event: ProgressEvent);
}

/// A progress sink that drops everything.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoProgress;

impl Progress for NoProgress {
    fn event(&mut self, _: ProgressEvent) {}
}

impl<F: FnMut(ProgressEvent)> Progress for F {
    fn event(&mut self, event: ProgressEvent) {
        self(event)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ProgressEvent {
    /// A step starts (`STEP n` on CalculiX's stdout), 1-based.
    Step { step: u32 },
    /// An increment starts (`increment n attempt m`).
    Increment {
        step: u32,
        increment: u32,
        attempt: u32,
        /// Size of the increment (`increment size=`).
        size: f64,
        /// Time at the end of the increment (`actual step time=`).
        step_time: f64,
        total_time: f64,
    },
    /// A Newton iteration finished (`iteration n`) with its residual and correction norms.
    Iteration {
        step: u32,
        increment: u32,
        iteration: u32,
        /// Largest residual force and the DOF/node it occurs at.
        residual: f64,
        /// Largest increment of the solution in this iteration.
        correction: f64,
        converged: bool,
    },
    /// An increment converged (`.sta` row).
    IncrementDone {
        step: u32,
        increment: u32,
        attempt: u32,
        iterations: u32,
        total_time: f64,
        step_time: f64,
        size: f64,
    },
    /// An increment was cut back after failing to converge.
    Cutback {
        step: u32,
        increment: u32,
        attempt: u32,
        reason: String,
    },
    /// An eigenvalue was found (frequency and buckle steps).
    Eigenvalue {
        step: u32,
        mode: u32,
        /// The eigenvalue itself (omega^2 or the buckling factor).
        value: f64,
    },
    /// A step finished.
    StepDone { step: u32, converged: bool },
    /// Something the user should know but that does not stop the run (`*WARNING`).
    Warning(String),
    /// Information lines (`*INFO`), such as the number of equations.
    Info(String),
    /// The job finished (`Job finished`).
    JobDone,
}
