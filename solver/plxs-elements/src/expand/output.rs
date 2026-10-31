//! From the results of the expanded model to what CalculiX writes: the `.frd` mesh holds
//! the nodes of the written elements only, not helper nodes of knots and mean rotations.
//!
//! Shells and beams are written expanded (the solids on the expanded nodes), or with
//! `OUTPUT=2D` as themselves. Trusses and membranes are always written as themselves on
//! their original nodes (ccx 2.21 does so). On such a node the forces are the sums over the
//! expanded nodes and every other field the through-section mean (shells Simpson over the
//! thickness, beams the mean of the four corners), averaged over the expansions meeting
//! there; displacements of a node where several expansions meet are its own.

use std::collections::{BTreeMap, HashMap, HashSet};

use plxs_core::mesh::NodeId;
use plxs_core::results::Results;
use plxs_core::step::OutputDimension;
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
    /// Through-section mean weights per original node and expansion.
    means: BTreeMap<NodeId, Vec<Vec<(NodeId, f64)>>>,
}

impl OutputMap {
    pub fn identity(_model: &Model) -> Self {
        Self::default()
    }

    pub fn expanded(
        original: &Model,
        _solver: &Model,
        groups: BTreeMap<NodeId, Vec<Vec<NodeId>>>,
        steps: &[Step],
    ) -> Self {
        // `OUTPUT=2D` on any field output writes all shells and beams unexpanded.
        let two_d = steps.iter().any(|s| {
            s.outputs
                .node_file
                .iter()
                .chain(&s.outputs.el_file)
                .any(|f| f.output == OutputDimension::TwoD)
        });
        let native = original
            .elements
            .iter()
            .enumerate()
            .filter(|(_, e)| match e.kind.family() {
                ElementFamily::Truss | ElementFamily::Membrane => true,
                ElementFamily::Shell | ElementFamily::Beam => two_d,
                _ => false,
            })
            .map(|(i, e)| (i, e.clone()))
            .collect();
        Self {
            groups,
            native,
            rotations: BTreeMap::new(),
            means: BTreeMap::new(),
        }
    }

    /// The through-section mean weights of the expansions (see the module docs).
    pub fn with_means(mut self, means: BTreeMap<NodeId, Vec<Vec<(NodeId, f64)>>>) -> Self {
        self.means = means;
        self
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
                for field in increment.fields.iter_mut() {
                    let position: HashMap<NodeId, usize> = field
                        .nodes
                        .iter()
                        .enumerate()
                        .map(|(i, &n)| (n, i))
                        .collect();
                    let sum = field.name == "FORC";
                    let disp = field.name == "DISP";
                    for &node in &native_nodes {
                        let Some(groups) = self.groups.get(&node) else {
                            continue;
                        };
                        // Displacements of a knot are the node's own.
                        if disp && groups.len() > 1 {
                            continue;
                        }
                        // Per value: (field index, weight).
                        let sources: Vec<(usize, f64)> = if sum {
                            groups
                                .iter()
                                .flatten()
                                .filter_map(|n| position.get(n).map(|&p| (p, 1.0)))
                                .collect()
                        } else {
                            let Some(means) = self.means.get(&node) else {
                                continue;
                            };
                            let complete: Vec<&Vec<(NodeId, f64)>> = means
                                .iter()
                                .filter(|m| m.iter().all(|(n, _)| position.contains_key(n)))
                                .collect();
                            let share = 1.0 / complete.len().max(1) as f64;
                            complete
                                .into_iter()
                                .flatten()
                                .map(|(n, w)| (position[n], w * share))
                                .collect()
                        };
                        if sources.is_empty() {
                            continue;
                        }
                        let target = match position.get(&node) {
                            Some(&t) => t,
                            None => {
                                field.nodes.push(node);
                                for component in &mut field.values {
                                    component.push(0.0);
                                }
                                field.nodes.len() - 1
                            }
                        };
                        for component in &mut field.values {
                            component[target] =
                                sources.iter().map(|&(s, w)| w * component[s]).sum();
                        }
                    }
                    // Nodes in ascending order, as written.
                    if field.nodes.windows(2).any(|w| w[0] > w[1]) {
                        let mut order: Vec<usize> = (0..field.nodes.len()).collect();
                        order.sort_by_key(|&i| field.nodes[i]);
                        field.nodes = order.iter().map(|&i| field.nodes[i]).collect();
                        for component in &mut field.values {
                            *component = order.iter().map(|&i| component[i]).collect();
                        }
                    }
                }
            }
        }
        if !self.rotations.is_empty() {
            // U1 elements stay on their two nodes only (the frd writer skips the type but
            // writes the nodes of every element).
            for e in out
                .elements
                .iter_mut()
                .filter(|e| e.kind == ElementType::U1)
            {
                if e.nodes.len() == 4 {
                    e.nodes = vec![e.nodes[0], e.nodes[2]];
                }
            }
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
            .filter(|e| e.kind != ElementType::U1)
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
