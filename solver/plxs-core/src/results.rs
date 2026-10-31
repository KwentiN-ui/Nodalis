//! Results of a run in memory: what the `.frd` and `.dat` writers serialize and what
//! PrePoLix will read directly in phase 2.

use serde::{Deserialize, Serialize};

use crate::mesh::NodeId;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Results {
    /// One entry per written increment (or mode).
    pub increments: Vec<ResultsIncrement>,
    /// History rows (`.dat`), in the order they are written.
    pub history: Vec<HistoryBlock>,
}

/// The kind of increment, as the `.frd` header tells it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum IncrementKind {
    /// A time increment: `value` is the total time (what CalculiX writes into the `.frd`).
    Time,
    /// A mode: `value` is the eigenfrequency (Hz) or, for `*BUCKLE`, the load factor.
    Frequency,
    Buckling,
    /// A complex mode: `value` is the real part of the frequency.
    ComplexFrequency,
}

/// Fields written for one increment of one step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResultsIncrement {
    pub step: u32,
    pub increment: u32,
    pub kind: IncrementKind,
    pub value: f64,
    /// Total time at the increment (time increments only).
    pub total_time: f64,
    /// Nodal diameter of a cyclic symmetric mode.
    pub nodal_diameter: Option<u32>,
    pub fields: Vec<ResultField>,
}

/// A nodal result field as written to the `.frd`: a CalculiX field name (`DISP`, `STRESS`,
/// `FORC`...), component names (`D1`, `SXX`...) and the values per component, in the order
/// of `nodes` (the output mesh: expanded nodes of shells and beams included, hence an own
/// id list rather than the model's dense order).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResultField {
    pub name: String,
    pub components: Vec<String>,
    pub nodes: Vec<NodeId>,
    /// `values[component][node]`.
    pub values: Vec<Vec<f64>>,
}

/// A block of the `.dat` file: a quantity for a set at a time, one row per node, element or
/// integration point.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HistoryBlock {
    pub step: u32,
    pub increment: u32,
    /// The heading as CalculiX writes it (`displacements (vx,vy,vz) for set TIP and time ...`
    /// without the time), which `plx-io::dat` recognises.
    pub quantity: String,
    pub set: String,
    pub time: f64,
    /// Rows: the id (node, element, or 0 for totals) and the values.
    pub rows: Vec<(u32, Vec<f64>)>,
}
