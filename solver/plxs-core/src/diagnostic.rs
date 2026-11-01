//! What the model check of the solver finds: typed, with the nodes and the step it concerns,
//! so that PrePoLix can point at the parts in its tree instead of reading the solver's text.

use serde::{Deserialize, Serialize};

use crate::mesh::NodeId;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Severity {
    Info,
    /// The run goes through, but the results may be wrong or not what was meant.
    Warning,
    /// The run would stop or its results would be useless.
    Error,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum DiagnosticCode {
    /// The stiffness matrix of a step is singular: the nodes can move as a rigid body
    /// without resistance. `motions` are the free motions, see [`Diagnostic::motions`].
    RigidBodyMotion,
    /// The stiffness matrix is singular, but the free motion deforms the structure: a
    /// mechanism (truss nodes on a line, hinges), not a rigid body.
    Mechanism,
    /// The stiffness matrix is nearly singular: a part is held far more weakly than the
    /// rest (very soft springs, extreme material or element size ratios, wrong units).
    NearlySingular,
    /// Slave nodes of a tie are farther than its tolerance from the master surface and are
    /// not tied.
    TieNodesNotTied { tie: String },
    /// The input cannot be read or refers to something undefined.
    Input,
    /// The model is complete, but the solver cannot compute it (yet).
    Unsupported,
    /// Another error of the solver.
    Solver,
    /// Size of the factorized stiffness matrix: equations and memory.
    FactorSize { equations: usize, bytes: u64 },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Diagnostic {
    pub code: DiagnosticCode,
    pub severity: Severity,
    /// 1-based step the finding belongs to; `None` for the model as a whole.
    pub step: Option<u32>,
    /// Name of the step (`*STEP` has none in the input: "Step n").
    pub step_name: Option<String>,
    /// What was found, in words, for the job output.
    pub message: String,
    /// The nodes concerned (user ids of the input), e.g. those that move freely.
    pub nodes: Vec<NodeId>,
    /// Free rigid body motions, one per singular direction: translation `[0..3]` and
    /// rotation `[3..6]` about [`Self::center`] that fit the motion best.
    pub motions: Vec<[f64; 6]>,
    /// Point the rotations of `motions` are about (centroid of `nodes`).
    pub center: [f64; 3],
}

impl Diagnostic {
    pub fn new(code: DiagnosticCode, severity: Severity, message: impl Into<String>) -> Self {
        Self {
            code,
            severity,
            step: None,
            step_name: None,
            message: message.into(),
            nodes: Vec::new(),
            motions: Vec::new(),
            center: [0.0; 3],
        }
    }

    /// An error of the solver as a diagnostic.
    pub fn from_error(error: &crate::Error) -> Self {
        use crate::Error;
        let code = match error {
            Error::Input { .. }
            | Error::UnknownKeyword { .. }
            | Error::UndefinedReference { .. }
            | Error::Duplicate { .. } => DiagnosticCode::Input,
            Error::Unsupported(_) => DiagnosticCode::Unsupported,
            Error::Numerical { .. } | Error::Io { .. } => DiagnosticCode::Solver,
        };
        Self::new(code, Severity::Error, error.to_string())
    }
}
