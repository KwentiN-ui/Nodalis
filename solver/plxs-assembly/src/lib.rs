//! Assembly of the PrePoLix solver: degree-of-freedom numbering, the element table of a
//! model (formulation, nodes, material of every element), the sparsity pattern, parallel
//! assembly of the stiffness matrix and of the load vector, single-point constraints and
//! the reduction to the free degrees of freedom.
//!
//! M1 covers solid elements with three displacements per node; [`heat`] adds the
//! temperature problem (one DOF per node) of the thermal procedures. Rotational degrees of
//! freedom (shells, beams), multi-point constraints and contact come in M2/M3 and extend
//! the [`DofMap`] and the reduction ([`Reduction`] is what `ConstraintSet` of the interface
//! document will generalize).
//!
//! No CalculiX code was consulted.

pub mod dofs;
pub mod elements;
pub mod heat;
pub mod loads;
pub mod stiffness;

pub use dofs::{DofMap, Reduction, Spc};
pub use elements::{ElementTable, IntegrationPointLayout};
pub use heat::{HeatTable, assemble_heat, heat_loads, temperature_spc};
pub use loads::load_vector;
pub use stiffness::{assemble_stiffness, build_pattern};
