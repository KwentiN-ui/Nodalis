//! Element formulations of the PrePoLix solver: shape functions and integration rules of
//! all families at one place ([`shape`]) and the solid elements ([`solid`]). Shells, beams,
//! plane elements, springs and gaps follow in later milestones (concept section 4.1).
//!
//! Sources: the CalculiX manual for node, face and integration-point conventions; textbooks
//! (Zienkiewicz & Taylor; Hughes; Bathe) for the formulations. No CalculiX code was
//! consulted.

pub mod nonlinear;
pub mod shape;
pub mod solid;

pub use nonlinear::PointKinematics;
pub use shape::{Face, Family, IntegrationRule};
pub use solid::{Formulation, SolidElement, solid};
