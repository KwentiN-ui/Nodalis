//! Constraints of the PrePoLix solver: single-point constraints, `*EQUATION`, `*RIGID BODY`
//! (linearized, small rotations), `*TIE` (tied contact, not cyclic) and `*TRANSFORM`, all
//! eliminated by a transformation `u = T v + g` (concept section 4.3): `K_red = Tᵀ K T`
//! stays symmetric positive definite, so the LDLᵀ solver applies unchanged.
//!
//! - [`ModelConstraints::new`] prepares the model-level constraints once: ties are projected
//!   (and their slave nodes adjusted), rigid bodies and equations become linear
//!   constraints on the primary DOFs (`node * 3 + d`, in the local system at transformed
//!   nodes).
//! - [`ModelConstraints::step_set`] adds a step's SPCs and eliminates everything into a
//!   [`ConstraintSet`], re-pivoting constraints whose dependent DOF is held otherwise
//!   instead of stopping as CalculiX does.
//!
//! Sources: CalculiX manual (keywords `*EQUATION`, `*RIGID BODY`, `*TIE`, `*TRANSFORM`,
//! section "Boundary conditions / Multiple point constraints"), Cook et al., "Concepts and
//! Applications of Finite Element Analysis", 4th ed., chapter 9; Wriggers, "Computational
//! Contact Mechanics", chapter 4. No CalculiX code was consulted.

pub mod elimination;
pub mod set;
pub mod tie;
pub mod transform;

pub use elimination::{ConstraintSet, Eliminator, LinearConstraint, Policy};
pub use set::ModelConstraints;
pub use transform::Rotation;
