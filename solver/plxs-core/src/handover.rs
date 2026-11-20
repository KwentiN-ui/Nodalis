//! What an analysis takes over from another one: the results of a global model for a
//! submodel, the modes of a frequency step and temperatures. The program that makes the
//! analysis gives them as values; only an input file names a file instead.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use serde::{Deserialize, Serialize};

use crate::element::ElementType;

/// Where the values an analysis takes over come from.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Source<T> {
    /// A file an input file names, read when the step needs it; a relative path is relative
    /// to the directory of the job.
    Path(PathBuf),
    /// The values themselves.
    Loaded(Arc<T>),
}

impl<T> Source<T> {
    pub fn loaded(values: T) -> Self {
        Source::Loaded(Arc::new(values))
    }

    /// The file, for a source read from one.
    pub fn path(&self) -> Option<&Path> {
        match self {
            Source::Path(path) => Some(path),
            Source::Loaded(_) => None,
        }
    }
}

/// The mesh and the displacements of a global model, for the nodes of a submodel.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct GlobalResults {
    /// Node id to coordinates.
    pub nodes: BTreeMap<u32, [f64; 3]>,
    /// The solid elements to interpolate in, with their node ids in the order of the
    /// solver (CalculiX's).
    pub elements: Vec<(ElementType, Vec<u32>)>,
    /// The displacements at the end of every step, numbered from 1: node id to `U1..U3`.
    pub displacements: BTreeMap<u32, BTreeMap<u32, [f64; 3]>>,
}

/// Temperatures of another analysis at the end of its steps, numbered from 1: node id to
/// temperature.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct NodalTemperatures {
    pub steps: BTreeMap<u32, BTreeMap<u32, f64>>,
}

/// The modes of a frequency step of another analysis, for modal steps without a frequency
/// step of their own.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct NodalModes {
    /// Eigenvalues (omega^2), one per mode.
    pub eigenvalues: Vec<f64>,
    /// The mass normalised mode shapes: node id to `U1..U3`.
    pub modes: Vec<BTreeMap<u32, [f64; 3]>>,
    /// The nodal diameter of every mode, for modes computed with cyclic symmetry.
    pub nodal_diameters: Option<Vec<u32>>,
}
