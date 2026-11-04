//! The data model of the Nodalis solver (`plxs`): the intermediate representation (IR) that
//! every other solver crate works on, and nothing else.
//!
//! The IR is the contract between the crates of `solver/`: `plxs-input` fills it from a
//! CalculiX input file, Nodalis itself will fill it directly from its model in phase 2, and
//! the elements, constraints, contact, procedures and output crates read it. It holds **data,
//! validation and errors only**; no element matrix, no solver and no file format lives here.
//!
//! Rules (see `docs/solver/SCHNITTSTELLEN.md`):
//!
//! - User ids (node and element numbers of the input) stay; the solver computes on dense
//!   indices ([`Nodes::index`], [`Model::element_index`]) and writes the ids out again.
//! - Everything is `Clone + Serialize + Deserialize` so that models, states and results can be
//!   handed between nodes of the Nodalis graph and stored for a restart.
//! - Names of sets, surfaces, materials, interactions and amplitudes are stored as CalculiX
//!   reads them: upper case ([`normalized_name`]).
//! - The IR covers the whole CalculiX inventory Nodalis writes (concept section 1), also what
//!   is computed only in later milestones, so that the shape of the data does not change while
//!   the crates fill up. Changing it is a PR of its own.
//! - A step holds what is in force during it: the `OP=NEW` semantics of CalculiX are resolved
//!   by the reader, procedures never look at earlier steps.
//!
//! Sources: CalculiX CrunchiX user's manual 2.23 (keyword chapter) for the meaning of each
//! field. No CalculiX source code was consulted or translated (its licence, GPL-2.0-only, is not
//! compatible with this project's GPL-3.0-or-later).

pub mod amplitude;
pub mod constraints;
pub mod contact;
pub mod diagnostic;
pub mod element;
pub mod error;
pub mod material;
pub mod mesh;
pub mod model;
pub mod progress;
pub mod results;
pub mod section;
pub mod state;
pub mod step;
pub mod validate;

pub use amplitude::Amplitude;
pub use constraints::{Constraints, Equation, RigidBody, Submodel, Tie, Transform};
pub use contact::{Contact, ContactPair, SurfaceBehavior, SurfaceInteraction};
pub use diagnostic::{Diagnostic, DiagnosticCode, ItemRef, Severity};
pub use element::{Element, ElementFamily, ElementType};
pub use error::{Error, Result};
pub use material::Material;
pub use mesh::{ElementId, NodeId, Nodes, Sets, Surface, SurfaceFace};
pub use model::{Analysis, InitialConditions, Model, PhysicalConstants};
pub use progress::{CancelToken, Progress, ProgressEvent};
pub use results::{Results, ResultsIncrement};
pub use section::Section;
pub use state::State;
pub use step::{Boundary, Loads, Outputs, Procedure, Step};
pub use validate::validate;

/// A name as CalculiX stores it: upper case, surrounding blanks removed. Set, surface,
/// material, interaction and amplitude names are compared this way.
pub fn normalized_name(name: &str) -> String {
    name.trim().to_ascii_uppercase()
}
