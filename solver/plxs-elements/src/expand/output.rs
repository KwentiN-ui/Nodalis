//! From the results of the expanded model to what CalculiX writes: the `.frd` mesh holds
//! the nodes of the (expanded) elements only, not the original nodes of shells and beams
//! nor helper nodes of knots and mean rotations.

use std::collections::{BTreeMap, HashSet};

use plxs_core::mesh::NodeId;
use plxs_core::results::Results;
use plxs_core::{Model, Step};

/// How results of the solver model become the written ones.
#[derive(Clone, Debug, Default)]
pub struct OutputMap {
    /// Expanded nodes per original node and expansion (empty without expansion).
    pub groups: BTreeMap<NodeId, Vec<Vec<NodeId>>>,
}

impl OutputMap {
    pub fn identity(_model: &Model) -> Self {
        Self::default()
    }

    pub fn expanded(
        _original: &Model,
        _solver: &Model,
        groups: BTreeMap<NodeId, Vec<Vec<NodeId>>>,
        _steps: &[Step],
    ) -> Self {
        Self { groups }
    }

    /// The model and results as written: nodes that belong to no element are left out of
    /// the mesh and of every field.
    pub fn apply(&self, model: &Model, results: &Results) -> (Model, Results) {
        let used: HashSet<NodeId> = model
            .elements
            .iter()
            .flat_map(|e| e.nodes.iter().copied())
            .collect();
        if used.len() == model.nodes.len() {
            return (model.clone(), results.clone());
        }
        let mut out = model.clone();
        let mut nodes = plxs_core::mesh::Nodes::default();
        for (id, x) in model.nodes.iter() {
            if used.contains(&id) {
                nodes.insert(id, x);
            }
        }
        out.nodes = nodes;
        let mut results = results.clone();
        for increment in &mut results.increments {
            for field in &mut increment.fields {
                let keep: Vec<bool> = field.nodes.iter().map(|n| used.contains(n)).collect();
                field.nodes = field
                    .nodes
                    .iter()
                    .zip(&keep)
                    .filter(|(_, k)| **k)
                    .map(|(n, _)| *n)
                    .collect();
                for component in &mut field.values {
                    *component = component
                        .iter()
                        .zip(&keep)
                        .filter(|(_, k)| **k)
                        .map(|(v, _)| *v)
                        .collect();
                }
            }
        }
        (out, results)
    }
}
