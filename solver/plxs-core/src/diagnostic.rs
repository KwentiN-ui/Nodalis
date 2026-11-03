//! What the model check of the solver finds: typed, with the nodes and the step it concerns,
//! so that Nodalis can point at the parts in its tree instead of reading the solver's text.

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

/// An item of the analysis a diagnostic is about, by its name or its index in the
/// [`crate::Analysis`]. Nodalis keeps a table from these to the items of its model they
/// were lowered from, so a finding points at the item in the tree without the message
/// being read.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum ItemRef {
    /// An element by its id.
    Element(u32),
    /// A material by its name.
    Material(String),
    /// An amplitude by its name.
    Amplitude(String),
    /// A section by its index in `Model::sections`.
    Section(usize),
    /// A tie by its name.
    Tie(String),
    /// A rigid body by its index in `Constraints::rigid_bodies`.
    RigidBody(usize),
    /// A contact pair by its index in `Contact::pairs`.
    ContactPair(usize),
    /// A step by its 0-based index in `Analysis::steps`.
    Step(usize),
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
    /// The item of the analysis the finding is about, when there is one.
    #[serde(default)]
    pub item: Option<ItemRef>,
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
            item: None,
        }
    }

    /// The diagnostic as about `item`.
    pub fn concerning(mut self, item: ItemRef) -> Self {
        self.item = Some(item);
        self
    }

    /// An error of the solver as a diagnostic.
    pub fn from_error(error: &crate::Error) -> Self {
        use crate::Error;
        let code = match error.plain() {
            Error::Concerning { .. } => unreachable!("plain() unwraps the item"),
            Error::Input { .. }
            | Error::UnknownKeyword { .. }
            | Error::UndefinedReference { .. }
            | Error::Duplicate { .. } => DiagnosticCode::Input,
            Error::Unsupported(_) => DiagnosticCode::Unsupported,
            Error::Numerical { .. } | Error::Io { .. } => DiagnosticCode::Solver,
        };
        let diagnostic = Self::new(code, Severity::Error, error.to_string());
        match error.item() {
            Some(item) => diagnostic.concerning(item.clone()),
            None => diagnostic,
        }
    }
}
