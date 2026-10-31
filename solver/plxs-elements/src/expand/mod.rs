//! Expansion of shell, membrane, beam and truss elements into solid elements, the way the
//! CalculiX manual describes it (sections "Shell elements", "Beam elements" and "Expansion
//! of the one-dimensional and two-dimensional elements"): every node of a structural
//! element gets new nodes along its normal (shells: bottom, middle, top) or across the
//! section (beams: eight), the element becomes a solid on these nodes, and multi-point
//! constraints tie the original node to its expansion:
//!
//! * without a knot, the original node follows the mean of the expanded edge nodes
//!   (`u_bottom + u_top - 2 u_0 = 0` for shells, the four corners for beams); loads and
//!   translational boundary conditions on the original node act through this constraint;
//! * a rotation boundary condition or moment on such a node becomes a *mean rotation*
//!   constraint about the axis, with a helper node whose first DOF is the angle;
//! * a node where several normals meet, or whose three rotations are all held, becomes a
//!   *knot*: an expandable rigid body around the original node (translation) with a helper
//!   node for the rotation vector and one for the uniform expansion, linearized.
//!
//! The result is a model with solid elements only (plus elements the solver computes
//! natively), so the solid pipeline computes it unchanged. [`OutputMap`] maps the results
//! back to what CalculiX writes: the expanded mesh without the original and helper nodes.
//!
//! The rules were taken from the manual; where it is silent (node numbering, which
//! constraint ccx generates in a case) they were inferred by comparing ccx results of small
//! models (black box). No CalculiX code was consulted.

mod beam;
mod output;
mod shell;

use std::collections::{BTreeMap, HashMap, HashSet};

use plxs_core::constraints::{Equation, EquationTerm};
use plxs_core::mesh::{ElementRef, NodeId, NodeRef};
use plxs_core::section::SectionKind;
use plxs_core::step::{Boundary, ConcentratedLoad, DistributedLoadKind};
use plxs_core::{Analysis, ElementFamily, ElementType, Error, Model, Result};

pub use output::OutputMap;

/// A model after the expansion, and how to map its results back.
#[derive(Clone, Debug)]
pub struct Expansion {
    /// What the solver computes.
    pub analysis: Analysis,
    pub output: OutputMap,
}

/// Whether an analysis has elements that need the expansion.
pub fn needs_expansion(model: &Model) -> bool {
    model.elements.iter().any(|e| expands(e.kind))
}

fn expands(kind: ElementType) -> bool {
    matches!(
        kind.family(),
        ElementFamily::Shell | ElementFamily::Membrane | ElementFamily::Beam
    )
}

/// Expands the structural elements of an analysis. An analysis without them is returned
/// unchanged with an identity output map.
pub fn expand(analysis: Analysis) -> Result<Expansion> {
    if !needs_expansion(&analysis.model) {
        let output = OutputMap::identity(&analysis.model);
        return Ok(Expansion { analysis, output });
    }
    Expander::new(analysis)?.run()
}

/// What kind of expansion a group of elements at a node is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    /// Shells and membranes: three nodes along the normal.
    Shell,
    /// Beams: eight nodes around the section.
    Beam,
}

/// One expansion of a node: the elements (index, local node) sharing a normal (and
/// thickness, offset, section), and the new nodes.
#[derive(Clone, Debug)]
struct Group {
    kind: Kind,
    members: Vec<(usize, usize)>,
    /// Shell normal, or for beams the local 1- and 2-directions.
    normal: [f64; 3],
    axis1: [f64; 3],
    /// Shell thickness and offset, beam section key.
    thickness: f64,
    offset: f64,
    beam: Option<beam::BeamSection>,
    /// New node ids: 3 for shells (bottom, middle, top), 8 for beams.
    nodes: Vec<NodeId>,
    /// Whether an element uses the new node (unused ones are not in the model).
    used: Vec<bool>,
}

/// The per-element data the expansion needs.
#[derive(Clone, Debug)]
struct Structural {
    /// Index into `model.elements`.
    element: usize,
    kind: Kind,
    /// Normal at every local node (shells) or local 2-direction (beams).
    normals: Vec<[f64; 3]>,
    /// Beam tangents at the local nodes.
    tangents: Vec<[f64; 3]>,
    thickness: f64,
    offset: f64,
    beam: Option<beam::BeamSection>,
}

struct Expander {
    analysis: Analysis,
    structural: Vec<Structural>,
    /// Groups per original node.
    groups: BTreeMap<NodeId, Vec<Group>>,
    next_id: NodeId,
    equations: Vec<Equation>,
    /// Knot helper nodes per original node: (rotation node, expansion node).
    knots: HashMap<NodeId, (NodeId, NodeId)>,
    /// Mean rotation helper nodes per (original node, axis 0..3).
    mean_rotations: HashMap<(NodeId, usize), NodeId>,
    /// Helper DOFs that need a zero boundary condition in every step.
    helper_fixed: Vec<(NodeId, u8)>,
}

impl Expander {
    fn new(analysis: Analysis) -> Result<Self> {
        let model = &analysis.model;
        let next_id = model.nodes.ids.iter().copied().max().unwrap_or(0) + 1;
        let mut structural = Vec::new();
        for (index, element) in model.elements.iter().enumerate() {
            if !expands(element.kind) {
                continue;
            }
            let section = element
                .section
                .and_then(|s| model.sections.get(s as usize))
                .ok_or_else(|| {
                    Error::input("*ELEMENT", format!("element {} has no section", element.id))
                })?;
            let coords = element_coordinates(model, index)?;
            let s = match (&section.kind, element.kind.family()) {
                (
                    SectionKind::Shell {
                        thickness,
                        offset,
                        nodal_thickness,
                        ..
                    },
                    ElementFamily::Shell | ElementFamily::Membrane,
                ) => {
                    if *nodal_thickness {
                        return Err(Error::Unsupported(
                            "*NODAL THICKNESS on shells is not supported".into(),
                        ));
                    }
                    Structural {
                        element: index,
                        kind: Kind::Shell,
                        normals: shell::normals(element.kind, &coords, element.id)?,
                        tangents: Vec::new(),
                        thickness: *thickness,
                        offset: *offset,
                        beam: None,
                    }
                }
                (
                    SectionKind::Membrane {
                        thickness, offset, ..
                    },
                    ElementFamily::Shell | ElementFamily::Membrane,
                ) => Structural {
                    element: index,
                    kind: Kind::Shell,
                    normals: shell::normals(element.kind, &coords, element.id)?,
                    tangents: Vec::new(),
                    thickness: *thickness,
                    offset: *offset,
                    beam: None,
                },
                (
                    SectionKind::Beam {
                        profile,
                        normal,
                        offset1,
                        offset2,
                        ..
                    },
                    ElementFamily::Beam,
                ) => {
                    let section = beam::BeamSection::new(profile, *offset1, *offset2)?;
                    let (tangents, normals) =
                        beam::directions(element.kind, &coords, *normal, element.id)?;
                    Structural {
                        element: index,
                        kind: Kind::Beam,
                        normals,
                        tangents,
                        thickness: 0.0,
                        offset: 0.0,
                        beam: Some(section),
                    }
                }
                _ => {
                    return Err(Error::input(
                        "*ELEMENT",
                        format!(
                            "element {} of type {} has an unsuitable section",
                            element.id, element.kind
                        ),
                    ));
                }
            };
            structural.push(s);
        }
        Ok(Self {
            analysis,
            structural,
            groups: BTreeMap::new(),
            next_id,
            equations: Vec::new(),
            knots: HashMap::new(),
            mean_rotations: HashMap::new(),
            helper_fixed: Vec::new(),
        })
    }

    fn run(mut self) -> Result<Expansion> {
        self.group_normals();
        self.number_nodes();
        let original = self.analysis.model.clone();
        self.replace_elements()?;
        self.drop_unused_nodes();
        let knot_nodes = self.knot_nodes();
        self.constraints(&knot_nodes);
        self.remap_steps(&original)?;
        let mut model = std::mem::take(&mut self.analysis.model);
        model.constraints.equations.extend(self.equations.drain(..));
        let groups: BTreeMap<NodeId, Vec<Vec<NodeId>>> = self
            .groups
            .iter()
            .map(|(&n, g)| (n, g.iter().map(|g| g.nodes.clone()).collect()))
            .collect();
        self.analysis.model = model;
        let output = OutputMap::expanded(&original, &self.analysis.model, groups, &self.analysis.steps);
        Ok(Expansion {
            analysis: self.analysis,
            output,
        })
    }

    /// Groups the structural elements at every node by normal, thickness, offset and
    /// section, the reference element being the one with the lowest number (manual).
    fn group_normals(&mut self) {
        let model = &self.analysis.model;
        // (node -> list of (element id, structural index, local node)).
        let mut at_node: BTreeMap<NodeId, Vec<(u32, usize, usize)>> = BTreeMap::new();
        for (s, st) in self.structural.iter().enumerate() {
            let element = &model.elements[st.element];
            for (local, &node) in element.nodes.iter().enumerate() {
                at_node
                    .entry(node)
                    .or_default()
                    .push((element.id, s, local));
            }
        }
        for (node, mut list) in at_node {
            list.sort_by_key(|x| (x.0, x.2));
            let mut left: Vec<(u32, usize, usize)> = list;
            let mut groups: Vec<Group> = Vec::new();
            while !left.is_empty() {
                let (_, s0, l0) = left[0];
                let reference = &self.structural[s0];
                let n0 = reference.normals[l0];
                let t0 = reference.tangents.get(l0).copied();
                let mut members = Vec::new();
                let mut rest = Vec::new();
                let mut sum_n = [0.0; 3];
                let mut sum_t = [0.0; 3];
                for &(id, s, l) in &left {
                    let st = &self.structural[s];
                    let same = st.kind == reference.kind
                        && model.elements[st.element].kind.family()
                            == model.elements[reference.element].kind.family()
                        && st.thickness == reference.thickness
                        && st.offset == reference.offset
                        && st.beam == reference.beam
                        && angle_below(st.normals[l], n0, NORMAL_ANGLE)
                        && match (st.tangents.get(l), t0) {
                            (Some(&t), Some(t0)) => angle_below(t, t0, NORMAL_ANGLE),
                            _ => true,
                        };
                    if same {
                        members.push((st.element, l));
                        sum_n = add(sum_n, st.normals[l]);
                        if let Some(&t) = st.tangents.get(l) {
                            sum_t = add(sum_t, t);
                        }
                    } else {
                        rest.push((id, s, l));
                    }
                }
                let normal = normalize(sum_n);
                let (normal, axis1) = if reference.kind == Kind::Beam {
                    beam::local_axes(normalize(sum_t), normal)
                } else {
                    (normal, [0.0; 3])
                };
                groups.push(Group {
                    kind: reference.kind,
                    members,
                    normal,
                    axis1,
                    thickness: reference.thickness,
                    offset: reference.offset,
                    beam: reference.beam.clone(),
                    nodes: Vec::new(),
                    used: Vec::new(),
                });
                left = rest;
            }
            self.groups.insert(node, groups);
        }
    }

    /// New node ids: nodes in ascending order, per group three (shell) or eight (beam)
    /// consecutive numbers after the largest id of the input (CalculiX numbers the same
    /// way; the `.frd` shows these ids).
    fn number_nodes(&mut self) {
        let model = &mut self.analysis.model;
        for (&node, groups) in self.groups.iter_mut() {
            let x = model.nodes.coordinates(node).unwrap_or([0.0; 3]);
            for g in groups.iter_mut() {
                let positions = match g.kind {
                    Kind::Shell => shell::positions(x, g.normal, g.thickness, g.offset),
                    Kind::Beam => g.beam.as_ref().unwrap().positions(x, g.axis1, g.normal),
                };
                for p in positions {
                    let id = self.next_id;
                    self.next_id += 1;
                    model.nodes.insert(id, p);
                    g.nodes.push(id);
                }
            }
        }
    }

    /// The expanded node `slot` of local node `local` of an element.
    fn expanded(&self, node: NodeId, element: usize, local: usize, slot: usize) -> NodeId {
        let groups = &self.groups[&node];
        let g = groups
            .iter()
            .find(|g| g.members.contains(&(element, local)))
            .expect("every structural element node is in a group");
        g.nodes[slot]
    }

    /// Replaces every structural element by its solid on the expanded nodes and converts
    /// the sections.
    fn replace_elements(&mut self) -> Result<()> {
        let mut converted: HashSet<u32> = HashSet::new();
        for st in self.structural.clone() {
            let element = self.analysis.model.elements[st.element].clone();
            let (kind, nodes) = match st.kind {
                Kind::Shell => {
                    let solid = shell::solid_type(element.kind);
                    let layout = shell::layout(element.kind);
                    let nodes = layout
                        .iter()
                        .map(|&(local, slot)| {
                            self.expanded(element.nodes[local], st.element, local, slot)
                        })
                        .collect();
                    (solid, nodes)
                }
                Kind::Beam => {
                    let solid = beam::solid_type(element.kind);
                    let layout = beam::layout(element.kind);
                    let nodes = layout
                        .iter()
                        .map(|&(local, slot)| {
                            self.expanded(element.nodes[local], st.element, local, slot)
                        })
                        .collect();
                    (solid, nodes)
                }
            };
            let e = &mut self.analysis.model.elements[st.element];
            e.kind = kind;
            e.nodes = nodes;
            if let Some(s) = e.section
                && converted.insert(s)
            {
                let section = &mut self.analysis.model.sections[s as usize];
                let material = section.kind.material().unwrap_or_default().to_string();
                section.kind = SectionKind::Solid {
                    material,
                    thickness: None,
                };
            }
        }
        Ok(())
    }

    /// Expanded nodes no element uses (the middle node of shell midside nodes, the edge
    /// midpoints of B31 nodes) keep their numbers but are not part of the model.
    fn drop_unused_nodes(&mut self) {
        let model = &mut self.analysis.model;
        let used: HashSet<NodeId> = model
            .elements
            .iter()
            .flat_map(|e| e.nodes.iter().copied())
            .collect();
        let mut unused: HashSet<NodeId> = HashSet::new();
        for groups in self.groups.values_mut() {
            for g in groups.iter_mut() {
                g.used = g.nodes.iter().map(|n| used.contains(n)).collect();
                unused.extend(g.nodes.iter().filter(|n| !used.contains(n)));
            }
        }
        if unused.is_empty() {
            return;
        }
        let mut nodes = plxs_core::mesh::Nodes::default();
        for (id, x) in model.nodes.iter() {
            if !unused.contains(&id) {
                nodes.insert(id, x);
            }
        }
        model.nodes = nodes;
    }

    /// Nodes that become knots: several expansions meet, or all three rotations are held
    /// in some step (CalculiX then uses an expandable rigid body instead of mean rotation
    /// constraints, which cannot hold the drilling rotation).
    fn knot_nodes(&self) -> HashSet<NodeId> {
        let mut knots: HashSet<NodeId> = self
            .groups
            .iter()
            .filter(|(_, g)| g.len() > 1)
            .map(|(&n, _)| n)
            .collect();
        let model = &self.analysis.model;
        for step in &self.analysis.steps {
            let mut held: HashMap<NodeId, [bool; 3]> = HashMap::new();
            for b in &step.boundaries {
                for node in model.nodes_of(&b.region) {
                    if !self.groups.contains_key(&node) {
                        continue;
                    }
                    let h = held.entry(node).or_default();
                    for d in b.first_dof.max(4)..=b.last_dof.min(6) {
                        h[(d - 4) as usize] = true;
                    }
                }
            }
            knots.extend(held.into_iter().filter(|(_, h)| h.iter().all(|&x| x)).map(|(n, _)| n));
        }
        knots
    }

    fn helper_node(&mut self, at: [f64; 3]) -> NodeId {
        let id = self.next_id;
        self.next_id += 1;
        self.analysis.model.nodes.insert(id, at);
        id
    }

    /// The constraints between original and expanded nodes.
    fn constraints(&mut self, knots: &HashSet<NodeId>) {
        let nodes: Vec<NodeId> = self.groups.keys().copied().collect();
        for node in nodes {
            if knots.contains(&node) {
                self.knot(node);
            } else {
                self.mean(node);
            }
        }
    }

    /// `u_0 = mean of the edge nodes` per direction, the original node first so that it
    /// becomes the dependent DOF unless it is held.
    fn mean(&mut self, node: NodeId) {
        let g = &self.groups[&node][0];
        let edge: Vec<NodeId> = match g.kind {
            Kind::Shell => vec![g.nodes[0], g.nodes[2]],
            Kind::Beam => beam::edge_slots().iter().map(|&s| g.nodes[s]).collect(),
        };
        let weight = 1.0 / edge.len() as f64;
        for dof in 1..=3u8 {
            let mut terms = vec![EquationTerm {
                node: NodeRef::Node(node),
                dof,
                coefficient: -1.0,
            }];
            for &e in &edge {
                terms.push(EquationTerm {
                    node: NodeRef::Node(e),
                    dof,
                    coefficient: weight,
                });
            }
            self.equations.push(Equation { terms });
        }
    }

    /// The linearized expandable rigid body of a knot: every expanded node follows
    /// `u_i = u_0 + θ × r_i + α r_i`, `r_i` from the centre of the expanded nodes.
    fn knot(&mut self, node: NodeId) {
        let model = &self.analysis.model;
        let members: Vec<NodeId> = self.groups[&node]
            .iter()
            .flat_map(|g| g.nodes.iter().zip(&g.used).filter(|(_, u)| **u).map(|(n, _)| *n))
            .collect();
        let coords: Vec<[f64; 3]> = members
            .iter()
            .map(|&n| model.nodes.coordinates(n).unwrap())
            .collect();
        let centre = scale(coords.iter().fold([0.0; 3], |a, &c| add(a, c)), 1.0 / coords.len() as f64);
        let x0 = model.nodes.coordinates(node).unwrap();
        let rotation = self.helper_node(x0);
        let expansion = self.helper_node(x0);
        self.helper_fixed.push((expansion, 2));
        self.helper_fixed.push((expansion, 3));
        self.knots.insert(node, (rotation, expansion));
        for (&m, &c) in members.iter().zip(&coords) {
            let r = sub(c, centre);
            for d in 0..3usize {
                // (θ × r)_d = θ_{d+1} r_{d+2} - θ_{d+2} r_{d+1}
                let (a, b) = ((d + 1) % 3, (d + 2) % 3);
                let mut terms = vec![
                    term(m, d as u8 + 1, 1.0),
                    term(node, d as u8 + 1, -1.0),
                ];
                if r[b] != 0.0 {
                    terms.push(term(rotation, a as u8 + 1, -r[b]));
                }
                if r[a] != 0.0 {
                    terms.push(term(rotation, b as u8 + 1, r[a]));
                }
                if r[d] != 0.0 {
                    terms.push(term(expansion, 1, -r[d]));
                }
                self.equations.push(Equation { terms });
            }
        }
    }

    /// The helper node of a mean rotation constraint about global axis `axis` at a node
    /// without knot, created with its constraint on first use.
    fn mean_rotation(&mut self, node: NodeId, axis: usize) -> Option<NodeId> {
        if let Some(&h) = self.mean_rotations.get(&(node, axis)) {
            return Some(h);
        }
        let model = &self.analysis.model;
        let g = &self.groups[&node][0];
        let participants: Vec<NodeId> = (g.nodes.iter().zip(&g.used))
            .filter(|(_, u)| **u)
            .map(|(n, _)| *n)
            .collect();
        let coords: Vec<[f64; 3]> = participants
            .iter()
            .map(|&n| model.nodes.coordinates(n).unwrap())
            .collect();
        let centre = scale(
            coords.iter().fold([0.0; 3], |a, &c| add(a, c)),
            1.0 / coords.len() as f64,
        );
        let mut a = [0.0; 3];
        a[axis] = 1.0;
        // Linearization of the mean of sin(angle) about the axis at the reference state:
        // Σ_i (a × r_i⊥)/|r_i⊥|² · (u_i - u_c) / N = φ.
        let mut gs = Vec::new();
        for &c in &coords {
            let r = sub(c, centre);
            let rp = sub(r, scale(a, dot(r, a)));
            let len2 = dot(rp, rp);
            gs.push(if len2 > 1e-20 * (1.0 + dot(r, r)) {
                scale(cross(a, rp), 1.0 / len2)
            } else {
                [0.0; 3]
            });
        }
        let count = gs.iter().filter(|g| dot(**g, **g) > 0.0).count();
        if count == 0 {
            return None;
        }
        let mean_g = scale(gs.iter().fold([0.0; 3], |s, &g| add(s, g)), 1.0 / gs.len() as f64);
        let x0 = model.nodes.coordinates(node).unwrap();
        let helper = self.helper_node(a);
        let _ = x0;
        self.helper_fixed.push((helper, 2));
        self.helper_fixed.push((helper, 3));
        let mut terms = Vec::new();
        let n = count as f64;
        for (&p, &gi) in participants.iter().zip(&gs) {
            let c = sub(gi, mean_g);
            for d in 0..3 {
                if c[d].abs() > 1e-14 * (1.0 + dot(gi, gi).sqrt()) {
                    terms.push(term(p, d as u8 + 1, c[d] / n));
                }
            }
        }
        terms.push(term(helper, 1, -1.0));
        self.equations.push(Equation { terms });
        self.mean_rotations.insert((node, axis), helper);
        Some(helper)
    }

    /// Boundary conditions and loads on rotations go to the knot or mean rotation helper
    /// nodes; pressures on structural elements to the faces of their solids.
    fn remap_steps(&mut self, original: &Model) -> Result<()> {
        let mut steps = std::mem::take(&mut self.analysis.steps);
        for step in &mut steps {
            let mut boundaries = Vec::new();
            for b in &step.boundaries {
                let (translation, rotation) = split_dofs(b.first_dof, b.last_dof);
                if let Some((f, l)) = translation {
                    boundaries.push(Boundary {
                        first_dof: f,
                        last_dof: l,
                        ..b.clone()
                    });
                }
                if let Some((f, l)) = rotation {
                    for node in original.nodes_of(&b.region) {
                        for d in f..=l {
                            if let Some((helper, dof)) = self.rotation_dof(node, d) {
                                boundaries.push(Boundary {
                                    region: NodeRef::Node(helper),
                                    first_dof: dof,
                                    last_dof: dof,
                                    ..b.clone()
                                });
                            }
                        }
                    }
                }
            }
            let mut loads = Vec::new();
            for c in &step.loads.concentrated {
                if c.dof <= 3 {
                    // Forces on nodes without knot act on the expanded nodes directly.
                    let nodes = original.nodes_of(&c.region);
                    if !nodes.iter().any(|n| self.distributes(*n)) {
                        loads.push(c.clone());
                        continue;
                    }
                    for node in nodes {
                        if !self.distributes(node) {
                            loads.push(ConcentratedLoad {
                                region: NodeRef::Node(node),
                                ..c.clone()
                            });
                            continue;
                        }
                        for (target, weight) in self.force_shares(node) {
                            loads.push(ConcentratedLoad {
                                region: NodeRef::Node(target),
                                value: c.value * weight,
                                ..c.clone()
                            });
                        }
                    }
                    continue;
                }
                for node in original.nodes_of(&c.region) {
                    if let Some((helper, dof)) = self.rotation_dof(node, c.dof) {
                        loads.push(ConcentratedLoad {
                            region: NodeRef::Node(helper),
                            dof,
                            ..c.clone()
                        });
                    }
                }
            }
            step.boundaries = boundaries;
            step.loads.concentrated = loads;
            for load in &mut step.loads.distributed {
                self.remap_pressure(original, &mut load.region, &mut load.kind)?;
            }
        }
        // Helper DOFs without meaning are held in every step.
        for step in &mut steps {
            for &(node, dof) in &self.helper_fixed {
                step.boundaries.push(Boundary {
                    region: NodeRef::Node(node),
                    first_dof: dof,
                    last_dof: dof,
                    value: None,
                    amplitude: None,
                    fixed: false,
                    submodel_step: None,
                    load_case: None,
                });
            }
        }
        self.analysis.steps = steps;
        Ok(())
    }

    /// Whether a force on the node is distributed to its expanded nodes (expanded nodes
    /// without knot).
    fn distributes(&self, node: NodeId) -> bool {
        self.groups.contains_key(&node) && !self.knots.contains_key(&node)
    }

    /// The expanded nodes of a node without knot and their shares of a force on it. For a
    /// shell node with all three expanded nodes in use (a corner of a quadratic element)
    /// ccx applies -1/2, 2, -1/2 to bottom, middle and top (the `FORC` output of ccx shows
    /// exactly these values), otherwise half to bottom and top. Beams: a quarter to each
    /// corner.
    fn force_shares(&self, node: NodeId) -> Vec<(NodeId, f64)> {
        let g = &self.groups[&node][0];
        match g.kind {
            Kind::Shell if g.used[1] => vec![
                (g.nodes[0], -0.5),
                (g.nodes[1], 2.0),
                (g.nodes[2], -0.5),
            ],
            Kind::Shell => vec![(g.nodes[0], 0.5), (g.nodes[2], 0.5)],
            Kind::Beam => beam::edge_slots()
                .iter()
                .map(|&s| (g.nodes[s], 0.25))
                .collect(),
        }
    }

    /// Where rotation DOF `d` (4..6) of an original node lives: the knot's rotation node
    /// or a mean rotation helper; `None` for nodes without rotations.
    fn rotation_dof(&mut self, node: NodeId, d: u8) -> Option<(NodeId, u8)> {
        if let Some(&(rotation, _)) = self.knots.get(&node) {
            return Some((rotation, d - 3));
        }
        if !self.groups.contains_key(&node) {
            return None;
        }
        self.mean_rotation(node, (d - 4) as usize).map(|h| (h, 1))
    }

    fn remap_pressure(
        &self,
        original: &Model,
        region: &mut ElementRef,
        kind: &mut DistributedLoadKind,
    ) -> Result<()> {
        let ids = original.elements_of(region);
        let structural = ids.iter().any(|&id| {
            original
                .element(id)
                .is_some_and(|e| expands(e.kind))
        });
        if !structural {
            return Ok(());
        }
        let family = original.element(ids[0]).map(|e| e.kind.family());
        match kind {
            // Shells: `P` is a pressure on the bottom face, acting along the normal.
            DistributedLoadKind::Other { label, values }
                if label == "P" && matches!(family, Some(ElementFamily::Shell | ElementFamily::Membrane)) =>
            {
                *kind = DistributedLoadKind::Pressure {
                    face: 1,
                    value: values.first().copied().unwrap_or(0.0),
                };
            }
            DistributedLoadKind::Pressure { face, .. } if family == Some(ElementFamily::Beam) => {
                return Err(Error::Unsupported(format!(
                    "pressure P{face} on beam elements is not supported yet"
                )));
            }
            _ => {}
        }
        Ok(())
    }
}

/// Normals within this angle (degrees) count as one.
const NORMAL_ANGLE: f64 = 0.5;

fn split_dofs(first: u8, last: u8) -> (Option<(u8, u8)>, Option<(u8, u8)>) {
    let translation = (first <= 3).then(|| (first, last.min(3)));
    let rotation = (last >= 4 && first <= 6).then(|| (first.max(4), last.min(6)));
    let other = (last > 6).then_some(());
    let _ = other;
    (translation, rotation)
}

fn term(node: NodeId, dof: u8, coefficient: f64) -> EquationTerm {
    EquationTerm {
        node: NodeRef::Node(node),
        dof,
        coefficient,
    }
}

fn element_coordinates(model: &Model, index: usize) -> Result<Vec<[f64; 3]>> {
    let element = &model.elements[index];
    element
        .nodes
        .iter()
        .map(|&id| {
            model.nodes.coordinates(id).ok_or_else(|| {
                Error::undefined("*ELEMENT", "node", format!("{id} (element {})", element.id))
            })
        })
        .collect()
}

fn angle_below(a: [f64; 3], b: [f64; 3], degrees: f64) -> bool {
    dot(a, b) >= degrees.to_radians().cos() * (dot(a, a) * dot(b, b)).sqrt()
}

pub(crate) fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub(crate) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub(crate) fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub(crate) fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(crate) fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub(crate) fn normalize(a: [f64; 3]) -> [f64; 3] {
    let l = dot(a, a).sqrt();
    if l > 0.0 { scale(a, 1.0 / l) } else { a }
}
