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
    /// A procedure did not converge or a matrix is singular.
    #[error("*ERROR: {message} (step {step}, increment {increment})")]
    Numerical {
        step: u32,
        increment: u32,
        message: String,
    },
    /// A file of the job could not be read or written.
    #[error("*ERROR: {path}: {message}")]
    Io { path: String, message: String },
    /// An error about one item of the analysis (a section, a tie, a step), which the
    /// validation names so that Nodalis can point at what the item came from without
    /// reading the message.
    #[error("{error}")]
    Concerning {
        item: crate::diagnostic::ItemRef,
        error: Box<Error>,
    },
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
