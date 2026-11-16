//! Particle analyses of the Nodalis solver: the material point method (MPM) on the
//! processor, for materials that flow, break apart and merge again, such as pastes, soft
//! solids and fluids moved by rigid tools. Concept: `KONZEPT_ROADMAP_MULTIPHYSIK.md` and
//! `STUFENKONZEPT_1_2_PARTIKEL_MPM.md` (stages 1 and 2).
//!
//! The input is a [`plxs_core::ParticleModel`]: material bodies given as closed surfaces,
//! which [`solve`] fills with particles, rigid walls given the same way, the box the material
//! moves in and its resolution. The results are frames of the particles with their fields
//! and the history of the energies and of the loads on the walls.
//!
//! Implemented from the publications cited in the modules; no code of other MPM solvers was
//! consulted.

pub mod distance;
pub mod law;
pub mod math;
mod solver;

pub use solver::{Status, solve};
