//! Sparse linear algebra of the Nodalis solver: the sparsity pattern of a symmetric
//! system, the matrix that fills it, the direct solver on top of `faer` and the iterative
//! one (conjugate gradients with an incomplete Cholesky preconditioner, [`pcg`]).
//!
//! Only `faer` and `rayon` are used for numerics (concept decision 7). The pattern is the
//! upper triangle (including the diagonal) of a symmetric matrix in compressed column form
//! with `u32` indices and `f64` values; it is computed once per step and reused over the
//! iterations of that step, which is what makes a reused symbolic factorization possible.
//!
//! Sources: the symbolic analysis of the sparse LDLᵀ factorization is `faer`'s (ordered by
//! the nested dissection of [`ordering`] for large systems, AMD for small ones); the numeric
//! factorization of supernodal structures is the tree-parallel one of [`supernodal`] on
//! `faer`'s dense kernels; the matrix layout is the textbook compressed-column format. No
//! CalculiX code was consulted.

pub mod amg;
pub mod bsr;
pub mod dense;
pub mod eigen;
pub mod gmres;
#[cfg(feature = "gpu")]
pub mod gpu;
pub mod memory;
pub mod ordering;
mod patch;
pub mod pattern;
pub mod pcg;
pub mod solver;
pub mod supernodal;

pub use amg::{Amg, AmgOptions, GpuMode, Unknowns};
pub use gmres::gmres;
pub use memory::available_memory;

pub use eigen::{EigenPairs, EigenSolver, LanczosOptions, ShiftInvertLanczos};
pub use pattern::{Pattern, SparseMatrix};
pub use pcg::{PcgSolver, PcgStats};
pub use solver::{LdltSolver, LinearSolver, OUT_OF_MEMORY, is_out_of_memory};
