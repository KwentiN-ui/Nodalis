//! Regression verification of the Nodalis solver: a corpus of fixtures (input files
//! Nodalis's writer produces, plus the examples of `testdata/`) with result snapshots
//! computed by plxs, and the comparison of what a solver computes for them. The snapshots
//! replaced CalculiX's golden files after plxs had matched them (stage 7 of the CalculiX
//! replacement); a fixture plxs did not match carries a `known_gap` instead.
//!
//! Layout (`solver/fixtures/<area>/`):
//!
//! - `<name>.inp`: the input, as Nodalis writes it.
//! - `<name>.toml`: what the fixture tests ([`Meta`]): area, tolerance class, milestone,
//!   the keywords and element types it uses, and where it came from.
//! - `<name>.ref.frd.gz`: the snapshot `.frd` of plxs, gzip-compressed (flate2 is pure Rust and in the
//!   workspace already; the concept named zstd, which needs a C library).
//! - `<name>.ref.dat`: the snapshot `.dat`, when the input requests history output.
//!
//! Comparison works on what Nodalis reads, not on text: both files go through
//! `plx_io::frd::read_frd_bytes` and `plx_io::dat::parse_dat`, then the meshes must match
//! exactly (class D: node and element ids, element types, expansion), the increments must
//! line up (step, increment, kind, value), and every raw component of every field must agree
//! within the tolerance of the fixture's class relative to the largest absolute value of the
//! component (see [`Class::tolerance`]).
//!
//! `cargo test -p plxs-verify` checks the corpus itself (every input parses with
//! `plxs-input`, every snapshot decodes and matches itself, the feature matrix in
//! `docs/solver/FIXTURES.md` is current). Running a solver over the corpus is the
//! `plxs-fixtures` binary (`cargo run -p plxs-verify --bin plxs-fixtures -- run --solver
//! target/debug/plxs`) and, for the milestone threads, tests that call [`run_corpus`] on their
//! fixtures.

pub mod classify;
pub mod compare;
pub mod corpus;
pub mod reference;
pub mod report;
#[cfg(test)]
mod tests;

pub use classify::{Class, Meta, Stage, classify};
pub use compare::{
    Comparison, compare_dat, compare_dat_except, compare_dat_with_noise, compare_frd,
    compare_frd_except, compare_frd_with, compare_frd_with_noise,
};
pub use corpus::{Fixture, FixtureResults, corpus, fixtures_root, load_reference};
pub use report::{Outcome, Report, run_corpus, run_fixture};
