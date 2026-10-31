//! From the results of the expanded model to what CalculiX writes: the `.frd` mesh holds
//! the nodes of the written elements only, not helper nodes of knots and mean rotations.
//!
//! Shells and beams are written expanded (the solids on the expanded nodes). Trusses and
//! membranes are written as themselves on their original nodes (ccx 2.21 does so): there
//! the forces are the sums over the expanded nodes, the displacements those of the original
//! node, and every other field the mean.

use std::collections::{BTreeMap, HashMap, HashSet};

use plxs_core::mesh::NodeId;
use plxs_core::results::Results;
use plxs_core::{Element, ElementFamily, ElementType, Model, Step};

/// How results of the solver model become the written ones.
#[derive(Clone, Debug, Default)]
pub struct OutputMap {
    /// Used expanded nodes per original node and expansion (empty without expansion).
    pub groups: BTreeMap<NodeId, Vec<Vec<NodeId>>>,
    /// Elements written as themselves: index into the model's elements and the original.
    native: Vec<(usize, Element)>,
    /// U1 nodes and the helper nodes of their rotations.
    rotations: BTreeMap<NodeId, NodeId>,
}

impl OutputMap {
    pub fn identity(_model: &Model) -> Self {
        Self::default()
    }

    pub fn expanded(
        original: &Model,
        _solver: &Model,
        groups: BTreeMap<NodeId, Vec<Vec<NodeId>>>,
        _steps: &[Step],
    ) -> Self {
        let native = original
            .elements
            .iter()
            .enumerate()
            .filter(|(_, e)| {
                matches!(
                    e.kind.family(),
                    ElementFamily::Truss | ElementFamily::Membrane
                )
            })
            .map(|(i, e)| (i, e.clone()))
            .collect();
        Self {
            groups,
            native,
            rotations: BTreeMap::new(),
        }
    }

    /// With U1 beams: CalculiX writes no element for them, their nodes with six
    /// displacements and forces (the rotations and moments from the helper nodes), and no
    /// element fields there.
    pub fn with_rotations(mut self, rotations: BTreeMap<NodeId, NodeId>) -> Self {
        self.rotations = rotations;
        self
    }

    /// The model and results as written: native elements back on their original nodes,
    /// and nodes that belong to no written element left out of the mesh and every field.
    pub fn apply(&self, model: &Model, results: &Results) -> (Model, Results) {
        let mut out = model.clone();
        let mut results = results.clone();
        if !self.native.is_empty() {
            let mut native_nodes: Vec<NodeId> = Vec::new();
            for (index, element) in &self.native {
                out.elements[*index] = element.clone();
                native_nodes.extend(element.nodes.iter().copied());
            }
            native_nodes.sort_unstable();
            native_nodes.dedup();
            for increment in &mut results.increments {
                // Displacements stay the original node's own (the mean constraint's value).
                for field in increment.fields.iter_mut().filter(|f| f.name != "DISP") {
                    let position: HashMap<NodeId, usize> = field
                        .nodes
                        .iter()
                        .enumerate()
                        .map(|(i, &n)| (n, i))
                        .collect();
                    let sum = field.name == "FORC";
                    for &node in &native_nodes {
                        let (Some(&target), Some(groups)) =
                            (position.get(&node), self.groups.get(&node))
                        else {
                            continue;
                        };
                        let sources: Vec<usize> = groups
                            .iter()
                            .flatten()
                            .filter_map(|n| position.get(n).copied())
                            .collect();
                        if sources.is_empty() {
                            continue;
                        }
                        for component in &mut field.values {
                            let total: f64 = sources.iter().map(|&s| component[s]).sum();
                            component[target] = if sum {
                                total
                            } else {
                                total / sources.len() as f64
                            };
                        }
                    }
                }
            }
        }
        if !self.rotations.is_empty() {
            out.elements.retain(|e| e.kind != ElementType::U1);
            for increment in &mut results.increments {
                for field in &mut increment.fields {
                    let names: [&str; 3] = match field.name.as_str() {
                        "DISP" => ["D4", "D5", "D6"],
                        "FORC" => ["F4", "F5", "F6"],
                        _ => continue,
                    };
                    let position: HashMap<NodeId, usize> = field
                        .nodes
                        .iter()
                        .enumerate()
                        .map(|(i, &n)| (n, i))
                        .collect();
                    let first = field.values.len().min(3);
                    let mut extra = vec![vec![0.0; field.nodes.len()]; 3];
                    for (i, node) in field.nodes.iter().enumerate() {
                        let Some(p) = self.rotations.get(node).and_then(|h| position.get(h)) else {
                            continue;
                        };
                        for (k, column) in extra.iter_mut().enumerate() {
                            column[i] = field.values.get(k).map_or(0.0, |c| c[*p]);
                        }
                    }
                    field.values.truncate(first);
                    field.values.extend(extra);
                    field.components.truncate(first);
                    field.components.extend(names.iter().map(|n| n.to_string()));
                }
            }
        }
        let in_elements: HashSet<NodeId> = out
            .elements
            .iter()
            .flat_map(|e| e.nodes.iter().copied())
            .collect();
        let mut used = in_elements.clone();
        used.extend(self.rotations.keys().copied());
        if used.len() == out.nodes.len() {
            return (out, results);
        }
        let mut nodes = plxs_core::mesh::Nodes::default();
        for (id, x) in out.nodes.iter() {
            if used.contains(&id) {
                nodes.insert(id, x);
            }
        }
        out.nodes = nodes;
        for increment in &mut results.increments {
            for field in &mut increment.fields {
                // Element fields only where elements are.
                let nodal = matches!(field.name.as_str(), "DISP" | "FORC");
                let keep: Vec<bool> = field
                    .nodes
                    .iter()
                    .map(|n| {
                        if nodal {
                            used.contains(n)
                        } else {
                            in_elements.contains(n)
                        }
                    })
                    .collect();
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
        for increment in &mut results.increments {
            increment.fields.retain(|f| !f.nodes.is_empty());
        }
        (out, results)
    }
}
