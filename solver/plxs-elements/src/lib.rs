//! Element formulations of the PrePoLix solver: shape functions and integration rules of
//! all families at one place ([`shape`]) and the solid elements ([`solid`]). Plane stress,
//! plane strain and axisymmetric elements ([`plane`]) are expanded into solids as CalculiX
//! does, and so are shells, membranes, beams and trusses ([`expand`]). Springs and gaps
//! follow in later milestones (concept section 4.1).
//!
//! Sources: the CalculiX manual for node, face and integration-point conventions; textbooks
//! (Zienkiewicz & Taylor; Hughes; Bathe) for the formulations. No CalculiX code was
//! consulted.

pub mod expand;
pub mod plane;
pub mod shape;
pub mod solid;
pub mod u1;

pub use plane::{PlaneKind, expanded_type, plane};
pub use shape::{Face, Family, IntegrationRule};
pub use solid::{Formulation, ReportedNodal, SolidElement, solid};
