//! Errors of the solver, with the name of the item of the input they refer to, so that
//! Nodalis can point at the tree entry and the command line can print them in CalculiX's
//! wording (`*ERROR reading *BOUNDARY: ...`).

use serde::{Deserialize, Serialize};

pub type Result<T> = std::result::Result<T, Error>;

/// What the solver could not do. Every variant names the item (keyword, set, material, step)
/// it refers to where there is one.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, thiserror::Error)]
pub enum Error {
    /// The input file could not be read or does not make sense. `keyword` is the keyword the
    /// error was found in (`*BOUNDARY`), `message` what is wrong.
    #[error("*ERROR reading {keyword}: {message}")]
    Input {
        keyword: String,
        message: String,
        /// File and line of the input, when known.
        location: Option<Location>,
    },
    /// A keyword the solver does not know (CalculiX's wording, `solver_check` looks for it).
    #[error("*ERROR in calinput: keyword {keyword} cannot be interpreted")]
    UnknownKeyword {
        keyword: String,
        location: Option<Location>,
    },
    /// A reference to something the model does not define.
    #[error("*ERROR in {keyword}: {kind} {name} has not been defined")]
    UndefinedReference {
        /// Keyword or item that holds the reference.
        keyword: String,
        /// What is missing: `node set`, `element set`, `surface`, `material`, `amplitude`,
        /// `surface interaction`, `node`, `element`.
        kind: String,
        name: String,
    },
    /// Something is defined twice where only one definition is allowed.
    #[error("*ERROR in {keyword}: {kind} {name} is defined twice")]
    Duplicate {
        keyword: String,
        kind: String,
        name: String,
    },
    /// The model is complete but the solver cannot compute it yet or at all.
    #[error("*ERROR: {0}")]
    Unsupported(String),
    /// A procedure did not converge or a matrix is singular. `failure` says which, so that
    /// Nodalis can explain it without reading the message.
    #[error("*ERROR: {message} (step {step}, increment {increment})")]
    Numerical {
        step: u32,
        increment: u32,
        failure: Failure,
        message: String,
    },
    /// A file of the job could not be read or written.
    #[error("*ERROR: {path}: {message}")]
    Io { path: String, message: String },
    /// The caller stopped the run (see [`crate::CancelToken`]) while a long computation was
    /// under way; the procedures end the step as cancelled.
    #[error("*INFO: the run was stopped")]
    Cancelled,
    /// An error about one item of the analysis (a section, a tie, a step), which the
    /// validation names so that Nodalis can point at what the item came from without
    /// reading the message.
    #[error("{error}")]
    Concerning {
        item: crate::diagnostic::ItemRef,
        error: Box<Error>,
    },
}

/// What went wrong in an [`Error::Numerical`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Failure {
    /// The increments of a nonlinear step (or of a heat transfer step) did not converge.
    NoConvergence(Convergence),
    /// The step needs more increments than `INC` of `*STEP` allows.
    IncrementLimit,
    /// The stiffness (or conductivity) matrix is singular: something moves freely.
    Singular,
    /// The matrix is not positive definite where it has to be: for the iterative equation
    /// solver, the multigrid, the reference state of a buckling step.
    NotPositiveDefinite,
    /// The factor or another part of the solution does not fit into the memory.
    OutOfMemory,
    /// The iterative equation solver did not converge.
    IterativeSolver,
    /// The eigenvalue iteration did not converge or found no eigenvalues.
    Eigenvalues,
    /// An undamped mode is excited at its eigenfrequency (steady state dynamics).
    Resonance,
    /// A misuse inside the solver (a bug, not a property of the model).
    Internal,
}

/// How the increments of a step failed to converge.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Convergence {
    /// Why the last attempt was given up.
    pub cause: Divergence,
    /// Why no further attempt was made.
    pub stop: Stop,
    /// Contact springs exist in the model.
    pub contact: bool,
    /// The contact status (closed, open, sticking, sliding) still changed in the last
    /// iteration of the last attempt.
    pub contact_changing: bool,
}

/// Why an attempt at an increment was given up.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Divergence {
    /// The residual force grew in two consecutive iterations.
    ResidualGrows,
    /// The iteration limit was reached.
    IterationLimit,
    /// The convergence rate predicts more iterations than allowed.
    TooSlow,
    /// The solution correction is not finite.
    NotFinite,
    /// The creep strain changed more than `CETOL` in the increment.
    CreepTolerance,
    /// The temperatures of a heat transfer step did not converge.
    Temperatures,
    /// The contact of a heat transfer step with contact did not settle.
    ThermalContact,
    /// The equations of an iteration could not be solved (see the message).
    Equations,
}

/// Why no smaller increment was tried.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Stop {
    /// More cutbacks in one increment than allowed.
    Cutbacks,
    /// The increment would fall below the minimum.
    MinimumIncrement,
    /// The increment size is fixed (`DIRECT`).
    FixedIncrement,
}

/// Where in the input an error was found.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Location {
    pub file: String,
    /// 1-based line number.
    pub line: usize,
}

impl Error {
    pub fn input(keyword: impl Into<String>, message: impl Into<String>) -> Self {
        Error::Input {
            keyword: keyword.into(),
            message: message.into(),
            location: None,
        }
    }

    /// A numerical error outside of a step and increment ([`Error::in_increment`] places it).
    pub fn numerical(failure: Failure, message: impl Into<String>) -> Self {
        Error::Numerical {
            step: 0,
            increment: 0,
            failure,
            message: message.into(),
        }
    }

    /// A numerical error placed in the step and increment it happened in; other errors as
    /// they are.
    pub fn in_increment(self, step: u32, increment: u32) -> Self {
        match self {
            Error::Numerical {
                failure, message, ..
            } => Error::Numerical {
                step,
                increment,
                failure,
                message,
            },
            other => other,
        }
    }

    /// The message of a numerical error changed by `f`; other errors as they are.
    pub fn map_message(self, f: impl FnOnce(String) -> String) -> Self {
        match self {
            Error::Numerical {
                step,
                increment,
                failure,
                message,
            } => Error::Numerical {
                step,
                increment,
                failure,
                message: f(message),
            },
            other => other,
        }
    }

    /// The kind of a numerical error.
    pub fn failure(&self) -> Option<Failure> {
        match self.plain() {
            Error::Numerical { failure, .. } => Some(*failure),
            _ => None,
        }
    }

    pub fn undefined(
        keyword: impl Into<String>,
        kind: impl Into<String>,
        name: impl Into<String>,
    ) -> Self {
        Error::UndefinedReference {
            keyword: keyword.into(),
            kind: kind.into(),
            name: name.into(),
        }
    }

    /// The error with its location in the input, if it has none yet.
    pub fn at(mut self, file: &str, line: usize) -> Self {
        match &mut self {
            Error::Input { location, .. } | Error::UnknownKeyword { location, .. }
                if location.is_none() =>
            {
                *location = Some(Location {
                    file: file.to_string(),
                    line,
                });
            }
            _ => {}
        }
        self
    }

    /// The error as about `item`, unless it names an item already.
    pub fn concerning(self, item: crate::diagnostic::ItemRef) -> Self {
        match self {
            Error::Concerning { .. } => self,
            error => Error::Concerning {
                item,
                error: Box::new(error),
            },
        }
    }

    /// The item of the analysis the error is about, when the validation named one.
    pub fn item(&self) -> Option<&crate::diagnostic::ItemRef> {
        match self {
            Error::Concerning { item, .. } => Some(item),
            _ => None,
        }
    }

    /// The error without the item it is about.
    pub fn plain(&self) -> &Error {
        match self {
            Error::Concerning { error, .. } => error.plain(),
            error => error,
        }
    }

    /// The location of the error in the input, when it has one.
    pub fn location(&self) -> Option<&Location> {
        match self {
            Error::Concerning { error, .. } => error.location(),
            Error::Input { location, .. } | Error::UnknownKeyword { location, .. } => {
                location.as_ref()
            }
            _ => None,
        }
    }
}
