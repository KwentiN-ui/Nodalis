//! The multi-point constraints of a model, prepared once (ties projected and adjusted,
//! rigid bodies and equations written as linear constraints on the primary DOFs), and the
//! per-step [`ConstraintSet`] that adds the step's SPCs and eliminates everything.

use std::collections::HashMap;

use plxs_core::Model;
use plxs_core::Result;
use plxs_core::constraints::RigidBodyNodes;
use plxs_core::mesh::NodeRef;

use crate::elimination::{Added, ConstraintSet, Eliminator, LinearConstraint, Policy};
use crate::tie;
use crate::transform::{Rotation, node_rotations};

/// Degrees of freedom per node of the primary numbering (`node * 3 + d`).
const PER_NODE: u32 = 3;

/// The model-level constraints, independent of the step.
#[derive(Clone, Debug, Default)]
pub struct ModelConstraints {
    /// Local systems of transformed nodes (dense node index).
    pub rotations: HashMap<u32, Rotation>,
    /// Rigid bodies, then equations, then ties, in input order within each kind.
    pub constraints: Vec<LinearConstraint>,
    /// What each constraint came from (`origin` indexes here).
    pub labels: Vec<String>,
    /// Messages for the job output (untied slave nodes, unsupported DOFs).
    pub warnings: Vec<String>,
    /// Number of slave nodes moved on to their master surface.
    pub adjusted: usize,
}

impl ModelConstraints {
    /// Prepares the constraints of a model. Moves the slave nodes of ties with `ADJUST` on
    /// to their master surfaces (the model's coordinates change, as in CalculiX).
    pub fn new(model: &mut Model) -> Result<ModelConstraints> {
        let mut mc = ModelConstraints::default();
        // Ties first: adjusting changes coordinates that rigid bodies and cylindrical
        // systems use.
        let mut tie_constraints = Vec::new();
        for t in model.constraints.ties.clone() {
            if t.cyclic_symmetry || t.multistage {
                // Cyclic symmetry is a separate milestone (M3); the tie only names the sides.
                continue;
            }
            let (couplings, missed) = tie::couplings(model, &t, &model.nodes.coordinates)?;
            if !missed.is_empty() {
                let ids: Vec<String> = missed
                    .iter()
                    .take(10)
                    .map(|&i| model.nodes.ids[i as usize].to_string())
                    .collect();
                mc.warnings.push(format!(
                    "*WARNING in *TIE {}: {} slave node(s) are too far away from the master surface and are not tied: {}{}",
                    t.name,
                    missed.len(),
                    ids.join(", "),
                    if missed.len() > 10 { ", ..." } else { "" }
                ));
            }
            if t.adjust {
                for c in &couplings {
                    if c.distance > 0.0 {
                        model.nodes.coordinates[c.slave as usize] = c.point;
                        mc.adjusted += 1;
                    }
                }
            }
            let origin = mc.label(format!("*TIE {}", t.name));
            tie_constraints.push((origin, couplings));
        }
        mc.rotations = node_rotations(model, &mut mc.warnings);

        // Rigid bodies.
        for (k, body) in model.constraints.rigid_bodies.iter().enumerate() {
            let origin = mc.label(format!("*RIGID BODY {}", k + 1));
            let (Some(reference), Some(rot)) = (
                model.nodes.index(body.reference_node),
                model.nodes.index(body.rotation_node),
            ) else {
                return Err(plxs_core::Error::undefined(
                    "*RIGID BODY",
                    "node",
                    format!("{} or {}", body.reference_node, body.rotation_node),
                ));
            };
            let x_ref = model.nodes.coordinates[reference as usize];
            for member in rigid_body_nodes(model, &body.nodes) {
                if member == reference || member == rot {
                    continue;
                }
                let x = model.nodes.coordinates[member as usize];
                let r = [x[0] - x_ref[0], x[1] - x_ref[1], x[2] - x_ref[2]];
                // u_i - u_ref - θ × r = 0, θ the "translations" of the rotation node.
                let rows: [[(u32, f64); 2]; 3] = [
                    [(1, -r[2]), (2, r[1])],
                    [(2, -r[0]), (0, r[2])],
                    [(0, -r[1]), (1, r[0])],
                ];
                for (d, theta) in rows.iter().enumerate() {
                    let mut global = vec![
                        (member * PER_NODE + d as u32, 1.0),
                        (reference * PER_NODE + d as u32, -1.0),
                    ];
                    for &(k, a) in theta {
                        if a != 0.0 {
                            global.push((rot * PER_NODE + k, a));
                        }
                    }
                    mc.push_global(
                        &global,
                        member * PER_NODE + d as u32,
                        Policy::Repivot,
                        origin,
                    );
                }
            }
        }

        // Equations: terms refer to local directions at transformed nodes.
        for (k, equation) in model.constraints.equations.iter().enumerate() {
            let origin = mc.label(format!("*EQUATION {}", k + 1));
            let mut terms = Vec::with_capacity(equation.terms.len());
            let mut skipped = false;
            for term in &equation.terms {
                let ids = match &term.node {
                    NodeRef::Node(id) => vec![*id],
                    NodeRef::Set(name) => {
                        model.sets.node_sets.get(name).cloned().unwrap_or_default()
                    }
                };
                if ids.len() != 1 || !(1..=3).contains(&term.dof) {
                    skipped = true;
                    continue;
                }
                let Some(index) = model.nodes.index(ids[0]) else {
                    return Err(plxs_core::Error::undefined(
                        "*EQUATION",
                        "node",
                        ids[0].to_string(),
                    ));
                };
                terms.push((index * PER_NODE + u32::from(term.dof) - 1, term.coefficient));
            }
            if skipped {
                mc.warnings.push(format!(
                    "*WARNING in *EQUATION {}: terms on degrees of freedom other than 1-3 or on node sets with more than one node are not supported and were left out",
                    k + 1
                ));
            }
            if let Some(&(preferred, _)) = terms.first() {
                mc.constraints.push(LinearConstraint {
                    terms,
                    preferred,
                    policy: Policy::Repivot,
                    origin,
                });
            }
        }

        // Ties: slave DOFs held otherwise stay untied, as in CalculiX.
        for (origin, couplings) in tie_constraints {
            for c in couplings {
                for d in 0..PER_NODE {
                    let mut global = vec![(c.slave * PER_NODE + d, 1.0)];
                    for &(m, w) in &c.masters {
                        if w != 0.0 {
                            global.push((m * PER_NODE + d, -w));
                        }
                    }
                    mc.push_global(
                        &global,
                        c.slave * PER_NODE + d,
                        Policy::KeepPrevious,
                        origin,
                    );
                }
            }
        }
        Ok(mc)
    }

    fn label(&mut self, label: String) -> u32 {
        self.labels.push(label);
        self.labels.len() as u32 - 1
    }

    /// Adds a constraint given on global DOFs, rewritten in the primary DOFs.
    fn push_global(&mut self, global: &[(u32, f64)], preferred: u32, policy: Policy, origin: u32) {
        let mut terms: Vec<(u32, f64)> = Vec::with_capacity(global.len());
        for &(dof, a) in global {
            let (node, d) = (dof / PER_NODE, (dof % PER_NODE) as usize);
            match self.rotations.get(&node) {
                None => add_term(&mut terms, dof, a),
                Some(r) => {
                    for (k, &rdk) in r[d].iter().enumerate() {
                        if rdk != 0.0 {
                            add_term(&mut terms, node * PER_NODE + k as u32, a * rdk);
                        }
                    }
                }
            }
        }
        let preferred = match self.rotations.get(&(preferred / PER_NODE)) {
            None => preferred,
            Some(r) => {
                let d = (preferred % PER_NODE) as usize;
                let k = (0..3)
                    .max_by(|&a, &b| r[d][a].abs().total_cmp(&r[d][b].abs()))
                    .unwrap();
                preferred / PER_NODE * PER_NODE + k as u32
            }
        };
        self.constraints.push(LinearConstraint {
            terms,
            preferred,
            policy,
            origin,
        });
    }

    pub fn is_empty(&self) -> bool {
        self.constraints.is_empty() && self.rotations.is_empty()
    }

    pub fn rotation(&self, node: u32) -> Option<&Rotation> {
        self.rotations.get(&node)
    }

    /// The constraint set of a step: the SPCs (`spc_dofs`, primary DOFs, i.e. local at
    /// transformed nodes) first, then the model's MPCs. `active[v]` marks the DOFs elements
    /// use. The prescribed values are passed later to [`ConstraintSet::particular`] in the
    /// order of `spc_dofs`.
    pub fn step_set(&self, nodes: usize, active: &[bool], spc_dofs: &[u32]) -> ConstraintSet {
        let n = nodes * PER_NODE as usize;
        let mut elimination = Eliminator::new(n);
        let mut warnings = Vec::new();
        for (s, &dof) in spc_dofs.iter().enumerate() {
            elimination.add(&[(dof, 1.0)], &[(s as u32, -1.0)], dof, Policy::Repivot);
        }
        let mut repivoted: HashMap<u32, usize> = HashMap::new();
        let mut conflicts: HashMap<u32, usize> = HashMap::new();
        for c in &self.constraints {
            match elimination.add(&c.terms, &[], c.preferred, c.policy) {
                Added::Repivoted { .. } => *repivoted.entry(c.origin).or_default() += 1,
                Added::Conflict => *conflicts.entry(c.origin).or_default() += 1,
                _ => {}
            }
        }
        let mut origins: Vec<_> = conflicts.into_iter().collect();
        origins.sort_unstable();
        for (origin, count) in origins {
            warnings.push(format!(
                "*WARNING in {}: {count} constraint(s) contradict earlier boundary conditions or constraints and were left out",
                self.labels[origin as usize]
            ));
        }
        let mut notes = Vec::new();
        let mut origins: Vec<_> = repivoted.into_iter().collect();
        origins.sort_unstable();
        for (origin, count) in origins {
            notes.push(format!(
                "{}: {count} dependent degree(s) of freedom were held otherwise; the constraint(s) were re-pivoted",
                self.labels[origin as usize]
            ));
        }
        let rotation = |node: u32| self.rotations.get(&node).copied();
        ConstraintSet::from_elimination(&elimination, active, &rotation, warnings, notes)
    }
}

fn add_term(terms: &mut Vec<(u32, f64)>, dof: u32, a: f64) {
    match terms.iter_mut().find(|(t, _)| *t == dof) {
        Some((_, v)) => *v += a,
        None => terms.push((dof, a)),
    }
}

/// The dense nodes of a rigid body: a node set, or the nodes of an element set.
fn rigid_body_nodes(model: &Model, nodes: &RigidBodyNodes) -> Vec<u32> {
    let ids: Vec<u32> = match nodes {
        RigidBodyNodes::NodeSet(name) => {
            model.sets.node_sets.get(name).cloned().unwrap_or_default()
        }
        RigidBodyNodes::ElementSet(name) => {
            let mut ids = Vec::new();
            let mut seen = std::collections::HashSet::new();
            for &e in model.sets.element_sets.get(name).into_iter().flatten() {
                if let Some(element) = model.element(e) {
                    for &n in &element.nodes {
                        if seen.insert(n) {
                            ids.push(n);
                        }
                    }
                }
            }
            ids
        }
    };
    ids.iter().filter_map(|&id| model.nodes.index(id)).collect()
}
