//! The state of a model between steps and increments: what a restart stores and what a
//! state link of the Nodalis graph hands from one node to the next. Data only; the
//! procedures fill it.

use serde::{Deserialize, Serialize};

/// Values at every node of the model in dense node order ([`crate::Nodes`]), one row per
/// node with `per_node` entries (3 for displacements, 1 for temperatures).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct NodalField {
    pub per_node: usize,
    pub values: Vec<f64>,
}

impl NodalField {
    pub fn zeros(nodes: usize, per_node: usize) -> Self {
        Self {
            per_node,
            values: vec![0.0; nodes * per_node],
        }
    }

    pub fn node(&self, index: usize) -> &[f64] {
        &self.values[index * self.per_node..(index + 1) * self.per_node]
    }

    pub fn node_mut(&mut self, index: usize) -> &mut [f64] {
        &mut self.values[index * self.per_node..(index + 1) * self.per_node]
    }
}

/// Values at the integration points of every element: `offsets[e]..offsets[e+1]` are the
/// entries of element `e` (dense element order), `per_point` values per integration point.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct IntegrationPointField {
    pub per_point: usize,
    pub offsets: Vec<u32>,
    pub values: Vec<f64>,
}

/// State variables of the material laws at every integration point (plastic strain tensor,
/// equivalent plastic strain, back stress, creep strain), laid out like
/// [`IntegrationPointField`] with the number of variables the material of the element needs.
pub type MaterialState = IntegrationPointField;

/// Where a contact slave node or integration point stands: open, sticking or sliding, with
/// the current gap and pressure.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ContactPointState {
    pub status: ContactStatus,
    pub gap: f64,
    pub pressure: f64,
    /// Tangential slip accumulated so far.
    pub slip: [f64; 2],
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContactStatus {
    #[default]
    Open,
    Stick,
    Slip,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct State {
    /// Number of steps computed so far; the next step is `steps_done + 1`.
    pub steps_done: u32,
    /// Total time at the end of the last increment.
    pub total_time: f64,
    /// Displacements (and rotations where nodes have them: 6 per node when any element
    /// carries rotations, 3 otherwise).
    pub displacements: NodalField,
    pub velocities: Option<NodalField>,
    pub accelerations: Option<NodalField>,
    pub temperatures: Option<NodalField>,
    /// Stresses at the integration points (6 per point).
    pub stresses: IntegrationPointField,
    /// Total strains at the integration points.
    pub strains: IntegrationPointField,
    pub material: MaterialState,
    /// Per contact pair, per slave node or integration point.
    pub contact: Vec<Vec<ContactPointState>>,
    /// Contact pairs switched off by `*MODEL CHANGE`, by index into
    /// [`crate::Contact::pairs`].
    pub inactive_pairs: Vec<u32>,
    /// Which steps stored modes (`*FREQUENCY, STORAGE=YES`) and how many: what later modal
    /// steps need.
    pub stored_modes: Option<StoredModes>,
    /// Internal energy density at the integration points at the end of a dynamic step,
    /// which CalculiX carries over into the `ENER` of the next dynamic step.
    #[serde(default)]
    pub energy: Option<IntegrationPointField>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StoredModes {
    pub step: u32,
    pub count: u32,
    /// Eigenvalues (omega^2).
    pub eigenvalues: Vec<f64>,
    /// Mode shapes, mass normalised, each in the layout of `displacements`.
    pub modes: Vec<NodalField>,
    /// Whether the modes were computed with cyclic symmetry (nodal diameters).
    pub nodal_diameters: Option<Vec<u32>>,
}

/// A snapshot for a restart or a state link: the state with a fingerprint of the model it
/// belongs to, so that a changed model is not resumed with a stale state, and what the
/// session carries from one step to the next besides the state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Snapshot {
    /// Stable hash of the model without its sets, surfaces and amplitudes (the parts a
    /// continuing model may add to); a session refuses a snapshot of another model.
    pub model_fingerprint: u64,
    pub state: State,
    /// The time-average force of the steps so far, the scale of the residual check of the
    /// next nonlinear step; `None` before the first nonlinear step. Without it a resumed
    /// run would not converge exactly as the run from the start.
    #[serde(default)]
    pub average_force: Option<f64>,
}
