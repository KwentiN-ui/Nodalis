//! Sparse linear algebra of the PrePoLix solver: the sparsity pattern of a symmetric
//! system, the matrix that fills it, and the direct solver on top of `faer`.
//!
//! Only `faer` and `rayon` are used for numerics (concept decision 7). The pattern is the
//! upper triangle (including the diagonal) of a symmetric matrix in compressed column form
//! with `u32` indices and `f64` values; it is computed once per step and reused over the
//! iterations of that step, which is what makes a reused symbolic factorization possible.
//!
//! Sources: the sparse LDLᵀ factorization is `faer`'s (supernodal with AMD ordering); the
//! matrix layout is the textbook compressed-column format. No CalculiX code was consulted.

pub mod dense;
pub mod eigen;
pub mod pattern;
pub mod solver;

pub use eigen::{EigenPairs, EigenSolver, LanczosOptions, ShiftInvertLanczos};
pub use pattern::{Pattern, SparseMatrix};
pub use solver::{LdltSolver, LinearSolver};
