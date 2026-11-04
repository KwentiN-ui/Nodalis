//! The multi-point constraints of a model, prepared once (ties projected and adjusted,
//! rigid bodies and equations written as linear constraints on the primary DOFs), and the
//! per-step [`ConstraintSet`] that adds the step's SPCs and eliminates everything.

use std::collections::HashMap;

use plxs_core::Result;
use plxs_core::constraints::RigidBodyNodes;
use plxs_core::mesh::NodeRef;
use plxs_core::{ElementFamily, Model};

use crate::cyclic::CyclicSymmetry;
use crate::elimination::{Added, ConstraintSet, Eliminator, LinearConstraint, Policy};
use crate::tie;
use crate::transform::{Rotation, node_rotations};

/// Degrees of freedom per node of the primary numbering (`node * 3 + d`).
const PER_NODE: u32 = 3;

/// Offset of the two expanded layers of a plane node in a rigid body (any positive value
/// gives the same constraints).
const PLANE_HALF_THICKNESS: f64 = 0.5;

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
    /// The cyclic symmetry of the model, if it has one.
    pub cyclic: Option<CyclicSymmetry>,
    /// Its nodal diameter 0 constraints (`u_s = R u_m`), which every step but a frequency
    /// step with `*SELECT CYCLIC SYMMETRY MODES` uses.
    pub cyclic_real: Vec<LinearConstraint>,
    /// Slave nodes (dense indices) of each tie that are too far from its master surface.
    pub untied: Vec<(String, Vec<u32>)>,
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
                // A cyclic symmetry tie only names the sides (see `cyclic`).
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
                mc.untied.push((t.name.clone(), missed.clone()));
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
        let plane = plane_nodes(model);

        // Cyclic symmetry: the axis is oriented so that the slave side lies at +φ.
        let label = model
            .constraints
            .cyclic_symmetry
            .as_ref()
            .map(|c| format!("*TIE {} (cyclic symmetry)", c.tie));
        if let Some(label) = label {
            let origin = mc.label(label);
            if let Some(cyclic) = CyclicSymmetry::new(model, origin)? {
                if !cyclic.missed.is_empty() {
                    let ids: Vec<String> = cyclic
                        .missed
                        .iter()
                        .take(10)
                        .map(|&i| model.nodes.ids[i as usize].to_string())
                        .collect();
                    mc.warnings.push(format!(
                        "*WARNING in *CYCLIC SYMMETRY MODEL: {} slave node(s) find no master face after the rotation by the sector angle and are not coupled: {}{}",
                        cyclic.missed.len(),
                        ids.join(", "),
                        if cyclic.missed.len() > 10 { ", ..." } else { "" }
                    ));
                }
                // The input's axis points the way the sectors follow each other, so that
                // the output and later steps need not orient it again.
                if let Some(definition) = model.constraints.cyclic_symmetry.as_mut() {
                    let p = cyclic.origin;
                    definition.axis = [
                        p,
                        [
                            p[0] + cyclic.axis[0],
                            p[1] + cyclic.axis[1],
                            p[2] + cyclic.axis[2],
                        ],
                    ];
                }
                mc.cyclic_real = cyclic
                    .real_constraints()
                    .into_iter()
                    .map(|(terms, preferred)| {
                        primary(
                            &mc.rotations,
                            u32::MAX,
                            &terms,
                            preferred,
                            Policy::KeepPrevious,
                            origin,
                        )
                    })
                    .collect();
                mc.cyclic = Some(cyclic);
            }
        }

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
                // A plane stress or plane strain node joins the rigid body in its plane
                // only: its out-of-plane DOF (the thickness change of plane stress) stays
                // out, as the ccx results show. The node is taken at z = ±t/2 so that the
                // body cannot rotate about x or y and its reference node does not move out
                // of plane (any positive t gives the same constraints).
                let in_plane = plane[member as usize];
                let layers: &[f64] = if in_plane {
                    &[PLANE_HALF_THICKNESS, -PLANE_HALF_THICKNESS]
                } else {
                    &[0.0]
                };
                for &offset in layers {
                    let r = [r[0], r[1], r[2] + offset];
                    // u_i - u_ref - θ × r = 0, θ the "translations" of the rotation node.
                    let rows: [[(u32, f64); 2]; 3] = [
                        [(1, -r[2]), (2, r[1])],
                        [(2, -r[0]), (0, r[2])],
                        [(0, -r[1]), (1, r[0])],
                    ];
                    for (d, theta) in rows.iter().enumerate() {
                        let own = d < 2 || !in_plane;
                        let mut global = vec![(reference * PER_NODE + d as u32, -1.0)];
                        if own {
                            global.insert(0, (member * PER_NODE + d as u32, 1.0));
                        }
                        for &(k, a) in theta {
                            if a != 0.0 {
                                global.push((rot * PER_NODE + k, a));
                            }
                        }
                        let preferred = if own { member } else { reference };
                        mc.push_global(
                            &global,
                            preferred * PER_NODE + d as u32,
                            Policy::Repivot,
                            origin,
                        );
                    }
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
                // An edge of a plane element ties the in-plane displacements; the
                // out-of-plane DOF (thickness change) of each side stays its own.
                let dofs = if c.planar { 2 } else { PER_NODE };
                for d in 0..dofs {
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
        let c = primary(&self.rotations, u32::MAX, global, preferred, policy, origin);
        self.constraints.push(c);
    }

    pub fn is_empty(&self) -> bool {
        self.constraints.is_empty() && self.rotations.is_empty() && self.cyclic_real.is_empty()
    }

    /// The dense nodes any constraint refers to, ascending and unique. Nodes outside every
    /// element (rigid body reference and rotation nodes, remote points) take their
    /// stiffness from these constraints alone, so their DOFs must not be held at zero.
    pub fn referenced_nodes(&self) -> Vec<u32> {
        let mut nodes: Vec<u32> = self
            .constraints
            .iter()
            .flat_map(|c| c.terms.iter().map(|&(dof, _)| dof / PER_NODE))
            .collect();
        nodes.sort_unstable();
        nodes.dedup();
        nodes
    }

    pub fn rotation(&self, node: u32) -> Option<&Rotation> {
        self.rotations.get(&node)
    }

    /// Carries the forces at the dependent DOFs of the multi-point constraints over to the
    /// nodes that have no element and take their force through the constraints alone:
    /// with the multiplier `λ = f_p / a_p` of the dependent DOF `p`, such a term `i` gets
    /// `-a_i λ` (rigid bodies: all their terms). `f` are the internal forces of the
    /// elements at every DOF (global directions, three per node); afterwards a reference
    /// point reports the force and moment it takes, and the pre-tension node of a bolt the
    /// bolt's force, as CalculiX reports them. `active[v]` marks the DOFs elements use.
    /// Constraints whose dependent DOF is held (`held`, primary DOFs) are left alone: there
    /// the force is a reaction already.
    pub fn transfer_forces(&self, f: &mut [f64], held: &[u32], active: &[bool]) {
        if self.constraints.is_empty() {
            return;
        }
        let held: std::collections::HashSet<u32> = held.iter().copied().collect();
        // Into the primary (local) directions of transformed nodes and back afterwards.
        let turn = |f: &mut [f64], to_local: bool| {
            for (&node, r) in &self.rotations {
                let at = node as usize * 3;
                let Some(v) = f.get(at..at + 3).map(|v| [v[0], v[1], v[2]]) else {
                    continue;
                };
                for d in 0..3 {
                    f[at + d] = (0..3)
                        .map(|k| {
                            if to_local {
                                r[k][d] * v[k]
                            } else {
                                r[d][k] * v[k]
                            }
                        })
                        .sum();
                }
            }
        };
        turn(f, true);
        let rigid = |c: &LinearConstraint| {
            (self.labels.get(c.origin as usize)).is_some_and(|l| l.starts_with("*RIGID BODY"))
        };
        for c in &self.constraints {
            let rigid = rigid(c);
            let p = c.preferred;
            if held.contains(&p) {
                continue;
            }
            let Some(&(_, a_p)) = c.terms.iter().find(|&&(v, _)| v == p) else {
                continue;
            };
            let Some(&f_p) = f.get(p as usize) else {
                continue;
            };
            if a_p == 0.0 || f_p == 0.0 {
                continue;
            }
            let lambda = f_p / a_p;
            for &(v, a) in &c.terms {
                if v != p
                    && (rigid || !active.get(v as usize).copied().unwrap_or(false))
                    && let Some(fv) = f.get_mut(v as usize)
                {
                    *fv -= a * lambda;
                }
            }
        }
        turn(f, false);
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
        for c in self.constraints.iter().chain(&self.cyclic_real) {
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

    /// The constraint set of a frequency step of nodal diameter `k` in a cyclic symmetric
    /// model, on the doubled numbering: the real part of the mode on the primary DOFs
    /// `0..3·nodes`, the imaginary part on `3·nodes..6·nodes`. Both copies carry the SPCs
    /// (`spc_dofs`, homogeneous) and the model's MPCs; the cyclic constraints couple them
    /// (see [`crate::cyclic`]). `active` is given for one copy.
    pub fn cyclic_step_set(
        &self,
        nodes: usize,
        active: &[bool],
        spc_dofs: &[u32],
        k: u32,
    ) -> Option<ConstraintSet> {
        let cyclic = self.cyclic.as_ref()?;
        let n = nodes * PER_NODE as usize;
        let offset = n as u32;
        let mut elimination = Eliminator::new(2 * n);
        for copy in [0, offset] {
            for (s, &dof) in spc_dofs.iter().enumerate() {
                elimination.add(
                    &[(copy + dof, 1.0)],
                    &[(s as u32, -1.0)],
                    copy + dof,
                    Policy::Repivot,
                );
            }
        }
        let mut conflicts: HashMap<u32, usize> = HashMap::new();
        for copy in [0, offset] {
            for c in &self.constraints {
                let terms: Vec<(u32, f64)> = c.terms.iter().map(|&(d, a)| (copy + d, a)).collect();
                if let Added::Conflict = elimination.add(&terms, &[], copy + c.preferred, c.policy)
                {
                    *conflicts.entry(c.origin).or_default() += 1;
                }
            }
        }
        for (terms, preferred) in cyclic.complex_constraints(k, offset) {
            let c = primary(
                &self.rotations,
                nodes as u32,
                &terms,
                preferred,
                Policy::KeepPrevious,
                cyclic.origin_label,
            );
            if let Added::Conflict = elimination.add(&c.terms, &[], c.preferred, c.policy) {
                *conflicts.entry(c.origin).or_default() += 1;
            }
        }
        let mut warnings = Vec::new();
        let mut origins: Vec<_> = conflicts.into_iter().collect();
        origins.sort_unstable();
        for (origin, count) in origins {
            warnings.push(format!(
                "*WARNING in {}: {count} constraint(s) contradict earlier boundary conditions or constraints and were left out",
                self.labels[origin as usize]
            ));
        }
        let mut doubled_active = active.to_vec();
        doubled_active.resize(n, false);
        doubled_active.extend_from_within(..);
        let rotation = |node: u32| self.rotations.get(&(node % nodes as u32)).copied();
        Some(ConstraintSet::from_elimination(
            &elimination,
            &doubled_active,
            &rotation,
            warnings,
            Vec::new(),
        ))
    }
}

/// Nodes used only by plane stress and plane strain elements (dense index).
fn plane_nodes(model: &Model) -> Vec<bool> {
    let n = model.nodes.len();
    let (mut plane, mut other) = (vec![false; n], vec![false; n]);
    for element in &model.elements {
        let flags = match element.kind.family() {
            ElementFamily::PlaneStress | ElementFamily::PlaneStrain => &mut plane,
            _ => &mut other,
        };
        for &id in &element.nodes {
            if let Some(i) = model.nodes.index(id) {
                flags[i as usize] = true;
            }
        }
    }
    plane.iter().zip(&other).map(|(&p, &o)| p && !o).collect()
}

/// A constraint given on global DOFs (`node * 3 + d`), rewritten in the primary DOFs (local
/// directions at transformed nodes). `wrap` maps the node numbers of a doubled numbering
/// back on to the model's (`u32::MAX` for none).
fn primary(
    rotations: &HashMap<u32, Rotation>,
    wrap: u32,
    global: &[(u32, f64)],
    preferred: u32,
    policy: Policy,
    origin: u32,
) -> LinearConstraint {
    let rotation_of = |node: u32| rotations.get(&(node % wrap));
    let mut terms: Vec<(u32, f64)> = Vec::with_capacity(global.len());
    for &(dof, a) in global {
        let (node, d) = (dof / PER_NODE, (dof % PER_NODE) as usize);
        match rotation_of(node) {
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
    let preferred = match rotation_of(preferred / PER_NODE) {
        None => preferred,
        Some(r) => {
            let d = (preferred % PER_NODE) as usize;
            let k = (0..3)
                .max_by(|&a, &b| r[d][a].abs().total_cmp(&r[d][b].abs()))
                .unwrap();
            preferred / PER_NODE * PER_NODE + k as u32
        }
    };
    LinearConstraint {
        terms,
        preferred,
        policy,
        origin,
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
