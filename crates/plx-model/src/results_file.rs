//! The results of another analysis a model takes, such as the global results of a submodel:
//! by the analysis they come from, or a file the user picked. The project file keeps which
//! analysis; the file of its results the run reads is made for each run and not saved.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// The results file a model takes results from.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ResultsFile {
    /// The results of the analysis block `analysis` of the project, through a link: the
    /// project hands the model a copy of them for the run, `copy`, which is not saved but
    /// made again when the project is opened.
    Linked {
        analysis: u64,
        #[serde(skip)]
        copy: Option<PathBuf>,
    },
    /// A file the user picked.
    File(PathBuf),
}

impl Default for ResultsFile {
    /// No file picked yet.
    fn default() -> Self {
        Self::File(PathBuf::new())
    }
}

impl ResultsFile {
    /// The results of the analysis block `analysis` with the copy for the run.
    pub fn linked(analysis: u64, copy: PathBuf) -> Self {
        Self::Linked {
            analysis,
            copy: Some(copy),
        }
    }

    /// The file the run reads, if there is one.
    pub fn path(&self) -> Option<&Path> {
        match self {
            Self::Linked { copy, .. } => copy.as_deref(),
            Self::File(path) => (!path.as_os_str().is_empty()).then_some(path.as_path()),
        }
    }

    /// The analysis block the results come from, for linked ones.
    pub fn analysis(&self) -> Option<u64> {
        match self {
            Self::Linked { analysis, .. } => Some(*analysis),
            Self::File(_) => None,
        }
    }

    /// What the results are, for messages: the file, or that they come through a link.
    pub fn label(&self) -> String {
        match (self, self.path()) {
            (_, Some(path)) => path.display().to_string(),
            (Self::Linked { .. }, None) => "the linked results".into(),
            (Self::File(_), None) => "no file".into(),
        }
    }

    /// Why the run cannot read the results, if it cannot.
    pub fn missing(&self) -> Option<String> {
        match (self, self.path()) {
            (_, Some(path)) if path.is_file() => None,
            (_, Some(path)) => Some(format!("Result file {} does not exist", path.display())),
            (Self::Linked { .. }, None) => {
                Some("The linked results are not there: solve the analysis they come from".into())
            }
            (Self::File(_), None) => Some("No result file chosen".into()),
        }
    }

    /// The same results read from `file`, e.g. a copy beside the input of the run.
    pub fn with_path(&self, file: PathBuf) -> Self {
        match self {
            Self::Linked { analysis, .. } => Self::linked(*analysis, file),
            Self::File(_) => Self::File(file),
        }
    }
}

impl From<PathBuf> for ResultsFile {
    /// A file the user picked.
    fn from(path: PathBuf) -> Self {
        Self::File(path)
    }
}

impl From<&str> for ResultsFile {
    /// A file the user picked.
    fn from(path: &str) -> Self {
        Self::File(path.into())
    }
}
