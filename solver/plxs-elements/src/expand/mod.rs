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
use plxs_core::step::{Boundary, ConcentratedLoad, DistributedLoadKind, Follower};
use plxs_core::{Analysis, ElementFamily, ElementType, Error, Model, Result};

pub use beam::section_rule;
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
    model
        .elements
        .iter()
        .any(|e| expands(e.kind) || e.kind == ElementType::U1)
}

fn expands(kind: ElementType) -> bool {
    // M3D3 and M3D4 are computed natively (`crate::membrane`).
    !crate::membrane::is_native(kind)
        && matches!(
            kind.family(),
            ElementFamily::Shell
                | ElementFamily::Membrane
                | ElementFamily::Beam
                | ElementFamily::Truss
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
    /// Trusses: each node also gets a mean rotation constraint holding the twist about
    /// the axis, which nothing else resists.
    truss: bool,
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
    truss: bool,
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
    /// Nodes of U1 beams and the helper node carrying their three rotations.
    u1_rotations: BTreeMap<NodeId, NodeId>,
    /// Local axes of structural nodes in a `*TRANSFORM` (rows: local x, y, z).
    local_axes: HashMap<NodeId, [[f64; 3]; 3]>,
    /// Transformed DOFs already turned into equations.
    local_done: HashSet<(NodeId, u8)>,
}

impl Expander {
    fn new(analysis: Analysis) -> Result<Self> {
        let model = &analysis.model;
        // CalculiX numbers one new node per node of a `*TRANSFORM` before the expanded ones.
        let transformed: HashSet<NodeId> = model
            .constraints
            .transforms
            .iter()
            .flat_map(|t| model.nodes_of(&NodeRef::Set(t.nset.clone())))
            .collect();
        // And one per node of a `*RIGID BODY` (ccx 2.21: the first expanded id moves by
        // the size of the rigid body's node set).
        let rigid: usize = model
            .constraints
            .rigid_bodies
            .iter()
            .map(|body| {
                let mut nodes: Vec<NodeId> = match &body.nodes {
                    plxs_core::constraints::RigidBodyNodes::NodeSet(set) => {
                        model.nodes_of(&NodeRef::Set(set.clone()))
                    }
                    plxs_core::constraints::RigidBodyNodes::ElementSet(set) => model
                        .elements_of(&plxs_core::mesh::ElementRef::Set(set.clone()))
                        .iter()
                        .filter_map(|&id| model.element(id))
                        .flat_map(|e| e.nodes.iter().copied())
                        .collect(),
                };
                nodes.sort_unstable();
                nodes.dedup();
                nodes.len()
            })
            .sum();
        let next_id = model.nodes.ids.iter().copied().max().unwrap_or(0)
            + 1
            + transformed.len() as u32
            + rigid as u32;
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
                        truss: false,
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
                    truss: false,
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
                        truss: false,
                        thickness: 0.0,
                        offset: 0.0,
                        beam: Some(section),
                    }
                }
                // Trusses: like a B31 beam with a square section of the given area (manual,
                // "Two-node truss element"; ccx 2.21 results match this beam exactly).
                (
                    SectionKind::Solid {
                        thickness: Some(area),
                        ..
                    },
                    ElementFamily::Truss,
                ) => {
                    let side = area.sqrt();
                    let section = beam::BeamSection::new(
                        &plxs_core::section::BeamProfile::Rect { a: side, b: side },
                        0.0,
                        0.0,
                    )?;
                    let axis = normalize(sub(coords[1], coords[0]));
                    let n1 = if axis[2].abs() < 0.9 {
                        [0.0, 0.0, -1.0]
                    } else {
                        [1.0, 0.0, 0.0]
                    };
                    let (tangents, normals) =
                        beam::directions(ElementType::B31, &coords, n1, element.id)?;
                    Structural {
                        element: index,
                        kind: Kind::Beam,
                        normals,
                        tangents,
                        truss: true,
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
            u1_rotations: BTreeMap::new(),
            local_axes: HashMap::new(),
            local_done: HashSet::new(),
        })
    }

    fn run(mut self) -> Result<Expansion> {
        self.group_normals();
        self.number_nodes();
        let original = self.analysis.model.clone();
        self.replace_elements()?;
        self.drop_unused_nodes();
        self.initial_motion();
        self.u1_nodes()?;
        self.transforms()?;
        let knot_nodes = self.knot_nodes();
        self.constraints(&knot_nodes);
        self.remap_steps(&original)?;
        let mut model = std::mem::take(&mut self.analysis.model);
        model.constraints.equations.append(&mut self.equations);
        model.expanded_structure = !self.structural.is_empty();
        model.shell_knots = self
            .knots
            .iter()
            .filter_map(|(&node, &(rotation, expansion))| {
                let member = self.groups[&node]
                    .iter()
                    .flat_map(|g| g.nodes.iter().zip(&g.used))
                    .find(|(_, u)| **u)
                    .map(|(&n, _)| n)?;
                Some([node, rotation, expansion, member])
            })
            .collect();
        model.shell_knots.sort_unstable();
        model.shell_fibres = self
            .groups
            .values()
            .flatten()
            .filter(|g| g.kind == Kind::Shell)
            .map(|g| {
                g.nodes
                    .iter()
                    .zip(&g.used)
                    .filter(|(_, u)| **u)
                    .map(|(n, _)| *n)
                    .collect::<Vec<_>>()
            })
            .filter(|f| f.len() >= 2)
            .collect();
        let groups: BTreeMap<NodeId, Vec<Vec<NodeId>>> = self
            .groups
            .iter()
            .map(|(&n, g)| {
                let used = g.iter().map(|g| {
                    g.nodes
                        .iter()
                        .zip(&g.used)
                        .filter(|(_, u)| **u)
                        .map(|(n, _)| *n)
                        .collect()
                });
                (n, used.collect())
            })
            .collect();
        // Through-section means of every expansion: shells Simpson over the thickness
        // (bottom, middle, top; without middle node the mean of both faces), beams the mean
        // of the four corners (what ccx writes on the original nodes).
        let means: BTreeMap<NodeId, Vec<Vec<(NodeId, f64)>>> = self
            .groups
            .iter()
            .map(|(&n, g)| {
                let per = g.iter().map(|g| match g.kind {
                    Kind::Shell if g.used.get(1) == Some(&true) => vec![
                        (g.nodes[0], 1.0 / 6.0),
                        (g.nodes[1], 4.0 / 6.0),
                        (g.nodes[2], 1.0 / 6.0),
                    ],
                    Kind::Shell => vec![(g.nodes[0], 0.5), (g.nodes[2], 0.5)],
                    Kind::Beam => g.nodes[..4].iter().map(|&k| (k, 0.25)).collect(),
                });
                (n, per.collect())
            })
            .collect();
        self.analysis.model = model;
        let output = OutputMap::expanded(
            &original,
            &self.analysis.model,
            groups,
            &self.analysis.steps,
        )
        .with_means(means)
        .with_rotations(self.u1_rotations.clone());
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
                        && st.truss == reference.truss
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
                    truss: reference.truss,
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
                    let as_beam = if st.truss {
                        ElementType::B31
                    } else {
                        element.kind
                    };
                    let solid = beam::solid_type(as_beam);
                    let layout = beam::layout(as_beam);
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
            // Beams integrated on their own section points keep the beam section.
            let round = st
                .beam
                .as_ref()
                .is_some_and(|b| !st.truss && section_rule(kind, &b.profile).is_some());
            if let Some(s) = e.section
                && !round
                && converted.insert(s)
            {
                let section = &mut self.analysis.model.sections[s as usize];
                let material = section.kind.material().unwrap_or_default().to_string();
                section.kind = SectionKind::Solid {
                    material,
                    thickness: None,
                    orientation: None,
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

    /// `*INITIAL CONDITIONS, TYPE=VELOCITY / DISPLACEMENT` on a structural node move its
    /// whole section: the original node depends on its expansion through the constraints,
    /// so the values go to the expanded nodes, translations as they are and rotations
    /// (DOFs 4 to 6) as the rigid motion `ω × r` of the section about the node.
    fn initial_motion(&mut self) {
        let model = &mut self.analysis.model;
        let moved = |entries: &[(NodeRef, u8, f64)], model: &plxs_core::model::Model| {
            let mut added: Vec<(NodeRef, u8, f64)> = Vec::new();
            for (region, dof, value) in entries {
                for node in model.nodes_of(region) {
                    let Some(groups) = self.groups.get(&node) else {
                        continue;
                    };
                    let Some(x0) = model.nodes.coordinates(node) else {
                        continue;
                    };
                    for group in groups {
                        for (&id, _) in group.nodes.iter().zip(&group.used).filter(|(_, u)| **u) {
                            match dof {
                                1..=3 => added.push((NodeRef::Node(id), *dof, *value)),
                                4..=6 => {
                                    let Some(x) = model.nodes.coordinates(id) else {
                                        continue;
                                    };
                                    let r = [x[0] - x0[0], x[1] - x0[1], x[2] - x0[2]];
                                    let mut w = [0.0; 3];
                                    w[*dof as usize - 4] = *value;
                                    let v = [
                                        w[1] * r[2] - w[2] * r[1],
                                        w[2] * r[0] - w[0] * r[2],
                                        w[0] * r[1] - w[1] * r[0],
                                    ];
                                    for (k, c) in v.iter().enumerate() {
                                        if *c != 0.0 {
                                            added.push((NodeRef::Node(id), k as u8 + 1, *c));
                                        }
                                    }
                                }
                                _ => {}
                            }
                        }
                    }
                }
            }
            added
        };
        let velocities = moved(&model.initial.velocities, model);
        let displacements = moved(&model.initial.displacements, model);
        // Rotations add up per node and component (several DOFs 4 to 6 on one node).
        let merge = |into: &mut Vec<(NodeRef, u8, f64)>, added: Vec<(NodeRef, u8, f64)>| {
            let mut sums: BTreeMap<(NodeId, u8), f64> = BTreeMap::new();
            for (region, dof, value) in added {
                if let NodeRef::Node(id) = region {
                    *sums.entry((id, dof)).or_default() += value;
                }
            }
            into.extend(
                sums.into_iter()
                    .map(|((id, dof), v)| (NodeRef::Node(id), dof, v)),
            );
        };
        merge(&mut model.initial.velocities, velocities);
        merge(&mut model.initial.displacements, displacements);
    }

    /// U1 beams keep their nodes; the rotations of every U1 node live on a helper node,
    /// which the element lists after the node (`a, rotation of a, b, rotation of b`).
    fn u1_nodes(&mut self) -> Result<()> {
        let mut ids: Vec<NodeId> = Vec::new();
        for e in &self.analysis.model.elements {
            if e.kind == ElementType::U1 {
                ids.extend(e.nodes.iter().copied());
            }
        }
        ids.sort_unstable();
        ids.dedup();
        for n in ids {
            if self.groups.contains_key(&n) {
                return Err(Error::Unsupported(format!(
                    "node {n} joins a U1 beam to shells or beams, which is not supported yet"
                )));
            }
            let x = self.analysis.model.nodes.coordinates(n).unwrap_or([0.0; 3]);
            let helper = self.helper_node(x);
            self.u1_rotations.insert(n, helper);
        }
        for e in &mut self.analysis.model.elements {
            if e.kind == ElementType::U1 {
                e.nodes = e
                    .nodes
                    .iter()
                    .flat_map(|n| [*n, self.u1_rotations[n]])
                    .collect();
            }
        }
        Ok(())
    }

    /// `*TRANSFORM` on structural nodes: their boundary conditions and loads act in the
    /// local axes, which the expansion turns into equations and mean rotations about the
    /// local axes. The transform then leaves the model. Rectangular systems only.
    fn transforms(&mut self) -> Result<()> {
        let model = &mut self.analysis.model;
        let mut kept = Vec::new();
        for t in std::mem::take(&mut model.constraints.transforms) {
            let nodes = model.nodes_of(&NodeRef::Set(t.nset.clone()));
            let structural = nodes
                .iter()
                .filter(|n| self.groups.contains_key(n) || self.u1_rotations.contains_key(n))
                .count();
            if structural == 0 {
                kept.push(t);
                continue;
            }
            if structural != nodes.len() {
                return Err(Error::Unsupported(format!(
                    "*TRANSFORM on set {} mixes shell or beam nodes with others",
                    t.nset
                )));
            }
            if t.kind != plxs_core::constraints::TransformKind::Rectangular {
                return Err(Error::Unsupported(
                    "cylindrical *TRANSFORM on shell or beam nodes is not supported yet".into(),
                ));
            }
            let x = normalize(t.a);
            let z = normalize(cross(t.a, t.b));
            let y = cross(z, x);
            for n in nodes {
                self.local_axes.insert(n, [x, y, z]);
            }
        }
        model.constraints.transforms = kept;
        Ok(())
    }

    /// `Σ dir_i u(node, i) = 0`, the largest coefficient first (the dependent DOF).
    fn directional_equation(&mut self, node: NodeId, dir: [f64; 3]) {
        let mut order = [0usize, 1, 2];
        order.sort_by(|&a, &b| dir[b].abs().total_cmp(&dir[a].abs()));
        let largest = dir[order[0]].abs();
        let terms = order
            .iter()
            .filter(|&&i| dir[i].abs() > 1e-12 * largest)
            .map(|&i| term(node, i as u8 + 1, dir[i]))
            .collect();
        self.equations.push(Equation { terms });
    }

    /// A boundary condition on a DOF of a transformed structural node: translations become
    /// an equation on the node, rotations a mean rotation about the local axis (or an
    /// equation on the knot's rotation node). Returns a boundary on a helper DOF, if any.
    fn local_boundary(&mut self, node: NodeId, d: u8, b: &Boundary) -> Result<Option<Boundary>> {
        let axes = self.local_axes[&node];
        let prescribed = b.value.unwrap_or(0.0);
        if d <= 3 {
            if prescribed != 0.0 {
                return Err(Error::Unsupported(
                    "nonzero boundary values in a *TRANSFORM on shell or beam nodes".into(),
                ));
            }
            if self.local_done.insert((node, d)) {
                self.directional_equation(node, axes[d as usize - 1]);
            }
            return Ok(None);
        }
        let dir = axes[d as usize - 4];
        let rotation_node = self
            .knots
            .get(&node)
            .map(|k| k.0)
            .or_else(|| self.u1_rotations.get(&node).copied());
        if let Some(r) = rotation_node {
            if prescribed != 0.0 {
                return Err(Error::Unsupported(
                    "nonzero boundary values in a *TRANSFORM on shell or beam nodes".into(),
                ));
            }
            if self.local_done.insert((node, d)) {
                self.directional_equation(r, dir);
            }
            return Ok(None);
        }
        let key = (node, 10 + (d as usize - 4));
        let helper = match self.mean_rotations.get(&key) {
            Some(&h) => Some(h),
            None => {
                let h = self.mean_rotation_about(node, dir);
                if let Some(h) = h {
                    self.mean_rotations.insert(key, h);
                }
                h
            }
        };
        Ok(helper.map(|h| Boundary {
            region: NodeRef::Node(h),
            first_dof: 1,
            last_dof: 1,
            ..b.clone()
        }))
    }

    /// Nodes that become knots: several expansions meet, or all three rotations of a shell
    /// node are held in some step (CalculiX then uses an expandable rigid body instead of
    /// mean rotation constraints, which cannot hold the drilling rotation).
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
                    // Beams hold their rotations by mean rotation constraints (the section
                    // has extent about every axis); shells cannot about the drilling axis.
                    match self.groups.get(&node) {
                        Some(g) if g.iter().all(|g| g.kind == Kind::Shell) => {}
                        _ => continue,
                    }
                    // In local axes ccx takes every held rotation as a mean rotation about
                    // its axis (manual, "Applying the SPC's to the expanded structure").
                    if self.local_axes.contains_key(&node) {
                        continue;
                    }
                    let h = held.entry(node).or_default();
                    for d in b.first_dof.max(4)..=b.last_dof.min(6) {
                        h[(d - 4) as usize] = true;
                    }
                }
            }
            knots.extend(
                held.into_iter()
                    .filter(|(_, h)| h.iter().all(|&x| x))
                    .map(|(n, _)| n),
            );
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
    /// Original nodes in node surfaces (tie slaves): their mean constraint makes an
    /// expanded node dependent, so that a tie can still take the original node (ccx 2.21
    /// ties the original node, the expanded nodes follow on average).
    fn surface_nodes(&self, knots: &HashSet<NodeId>) -> HashSet<NodeId> {
        let mut out = HashSet::new();
        for surface in self.analysis.model.sets.surfaces.values() {
            if let plxs_core::mesh::Surface::Node(ids) = surface {
                out.extend(
                    ids.iter()
                        .filter(|n| self.groups.contains_key(n) && !knots.contains(n)),
                );
            }
        }
        out
    }

    fn constraints(&mut self, knots: &HashSet<NodeId>) {
        let held = self.held_translations();
        let in_surfaces = self.surface_nodes(knots);
        let nodes: Vec<NodeId> = self.groups.keys().copied().collect();
        for node in nodes {
            if knots.contains(&node) {
                self.knot(node);
            } else {
                self.mean(node, in_surfaces.contains(&node));
                if held.contains(&node) {
                    self.tie_edge_midpoints(node);
                }
                let g = &self.groups[&node][0];
                if g.truss {
                    let axis = cross(g.axis1, g.normal);
                    if let Some(h) = self.mean_rotation_about(node, normalize(axis)) {
                        self.helper_fixed.push((h, 1));
                    }
                }
            }
        }
    }

    /// Structural nodes with a translational boundary condition in some step.
    fn held_translations(&self) -> HashSet<NodeId> {
        let model = &self.analysis.model;
        let mut held = HashSet::new();
        for step in &self.analysis.steps {
            for b in &step.boundaries {
                if b.first_dof <= 3 {
                    held.extend(
                        model
                            .nodes_of(&b.region)
                            .into_iter()
                            .filter(|n| self.groups.contains_key(n)),
                    );
                }
            }
        }
        held
    }

    /// At a held end node of a quadratic beam the edge midpoints of the section follow the
    /// node (ccx results show them exactly at the prescribed values; the corners only
    /// on average, as the mean constraint says).
    fn tie_edge_midpoints(&mut self, node: NodeId) {
        let g = &self.groups[&node][0];
        if g.kind != Kind::Beam || !g.used[4] {
            return;
        }
        let mids: Vec<NodeId> = (4..8).map(|s| g.nodes[s]).collect();
        // Along the beam each midpoint follows the node; in the section plane opposite
        // midpoints move symmetrically about it (an affine map of the section: Poisson
        // contraction and distortion stay free), as ccx 2.21 shows.
        let (n1, n2) = (normalize(g.axis1), normalize(g.normal));
        let t = normalize(cross(n1, n2));
        let along = |d: [f64; 3], nodes: &[(NodeId, f64)]| -> Equation {
            let mut terms = Vec::new();
            for &(n, w) in nodes {
                for (k, &c) in d.iter().enumerate() {
                    if c.abs() > 1e-12 {
                        terms.push(term(n, k as u8 + 1, c * w));
                    }
                }
            }
            Equation { terms }
        };
        for &m in &mids {
            self.equations.push(along(t, &[(m, 1.0), (node, -1.0)]));
        }
        for (a, b) in [(mids[0], mids[2]), (mids[1], mids[3])] {
            for d in [n1, n2] {
                self.equations
                    .push(along(d, &[(a, 0.5), (b, 0.5), (node, -1.0)]));
            }
        }
    }

    /// `u_0 = mean of the edge nodes` per direction, the original node first so that it
    /// becomes the dependent DOF unless it is held.
    fn mean(&mut self, node: NodeId, edge_dependent: bool) {
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
            if edge_dependent {
                terms.swap(0, 1);
            }
            self.equations.push(Equation { terms });
        }
    }

    /// The linearized expandable rigid body of a knot: every expanded node follows
    /// `u_i = u_0 + θ × r_i + S r_i`, `r_i` from the centre of the expanded nodes. The
    /// stretch `S` is isotropic (`α I`, one parameter) for shell knots and knots of
    /// several expansions; for the knot of a single beam expansion it is the stretch in the
    /// section plane linearized at the undeformed state, `s1 e1 e1 + s2 e2 e2` (the manual's
    /// beam knot: the shear angle of the principal directions has no first-order effect).
    fn knot(&mut self, node: NodeId) {
        let model = &self.analysis.model;
        let groups = &self.groups[&node];
        let members: Vec<NodeId> = groups
            .iter()
            .flat_map(|g| {
                g.nodes
                    .iter()
                    .zip(&g.used)
                    .filter(|(_, u)| **u)
                    .map(|(n, _)| *n)
            })
            .collect();
        let beam_axes = (groups.len() == 1 && groups[0].kind == Kind::Beam)
            .then(|| (groups[0].axis1, groups[0].normal));
        let coords: Vec<[f64; 3]> = members
            .iter()
            .map(|&n| model.nodes.coordinates(n).unwrap())
            .collect();
        let centre = scale(
            coords.iter().fold([0.0; 3], |a, &c| add(a, c)),
            1.0 / coords.len() as f64,
        );
        let x0 = model.nodes.coordinates(node).unwrap();
        let rotation = self.helper_node(x0);
        let expansion = self.helper_node(x0);
        if beam_axes.is_none() {
            self.helper_fixed.push((expansion, 2));
        }
        self.helper_fixed.push((expansion, 3));
        self.knots.insert(node, (rotation, expansion));
        for (&m, &c) in members.iter().zip(&coords) {
            let r = sub(c, centre);
            for d in 0..3usize {
                // (θ × r)_d = θ_{d+1} r_{d+2} - θ_{d+2} r_{d+1}
                let (a, b) = ((d + 1) % 3, (d + 2) % 3);
                let mut terms = vec![term(m, d as u8 + 1, 1.0), term(node, d as u8 + 1, -1.0)];
                if r[b] != 0.0 {
                    terms.push(term(rotation, a as u8 + 1, -r[b]));
                }
                if r[a] != 0.0 {
                    terms.push(term(rotation, b as u8 + 1, r[a]));
                }
                match beam_axes {
                    None => {
                        if r[d] != 0.0 {
                            terms.push(term(expansion, 1, -r[d]));
                        }
                    }
                    Some((e1, e2)) => {
                        let (c1, c2) = (dot(r, e1) * e1[d], dot(r, e2) * e2[d]);
                        if c1.abs() > 1e-14 * (1.0 + dot(r, r).sqrt()) {
                            terms.push(term(expansion, 1, -c1));
                        }
                        if c2.abs() > 1e-14 * (1.0 + dot(r, r).sqrt()) {
                            terms.push(term(expansion, 2, -c2));
                        }
                    }
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
        let mut a = [0.0; 3];
        a[axis] = 1.0;
        let helper = self.mean_rotation_about(node, a)?;
        self.mean_rotations.insert((node, axis), helper);
        Some(helper)
    }

    /// A mean rotation constraint about the unit vector `a`; returns its helper node.
    fn mean_rotation_about(&mut self, node: NodeId, a: [f64; 3]) -> Option<NodeId> {
        let model = &self.analysis.model;
        let g = &self.groups[&node][0];
        // Shells: bottom and top (the middle node lies on the axis); beams: the corners.
        let participants: Vec<NodeId> = match g.kind {
            Kind::Shell => vec![g.nodes[0], g.nodes[2]],
            Kind::Beam => beam::edge_slots().iter().map(|&s| g.nodes[s]).collect(),
        };
        let coords: Vec<[f64; 3]> = participants
            .iter()
            .map(|&n| model.nodes.coordinates(n).unwrap())
            .collect();
        let centre = scale(
            coords.iter().fold([0.0; 3], |a, &c| add(a, c)),
            1.0 / coords.len() as f64,
        );
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
        let mean_g = scale(
            gs.iter().fold([0.0; 3], |s, &g| add(s, g)),
            1.0 / gs.len() as f64,
        );
        let helper = self.helper_node(a);
        self.helper_fixed.push((helper, 2));
        self.helper_fixed.push((helper, 3));
        let mut terms = Vec::new();
        let n = count as f64;
        for (&p, &gi) in participants.iter().zip(&gs) {
            let c = sub(gi, mean_g);
            for (d, &cd) in c.iter().enumerate() {
                if cd.abs() > 1e-14 * (1.0 + dot(gi, gi).sqrt()) {
                    terms.push(term(p, d as u8 + 1, cd / n));
                }
            }
        }
        terms.push(term(helper, 1, -1.0));
        self.equations.push(Equation { terms });
        Some(helper)
    }

    /// Boundary conditions and loads on rotations go to the knot or mean rotation helper
    /// nodes; pressures on structural elements to the faces of their solids.
    fn remap_steps(&mut self, original: &Model) -> Result<()> {
        let mut steps = std::mem::take(&mut self.analysis.steps);
        for step in &mut steps {
            let mut boundaries = Vec::new();
            // Boundaries on transformed nodes act in local axes; the others go on per node
            // when a set mixes both.
            let mut global: Vec<Boundary> = Vec::new();
            for b in &step.boundaries {
                let nodes = original.nodes_of(&b.region);
                if !nodes.iter().any(|n| self.local_axes.contains_key(n)) {
                    global.push(b.clone());
                    continue;
                }
                for node in nodes {
                    if !self.local_axes.contains_key(&node) {
                        global.push(Boundary {
                            region: NodeRef::Node(node),
                            ..b.clone()
                        });
                        continue;
                    }
                    for d in b.first_dof..=b.last_dof.min(6) {
                        if let Some(h) = self.local_boundary(node, d, b)? {
                            boundaries.push(h);
                        }
                    }
                }
            }
            for b in &global {
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
            let mut concentrated: Vec<ConcentratedLoad> = Vec::new();
            for c in &step.loads.concentrated {
                let nodes = original.nodes_of(&c.region);
                if c.dof > 6 || !nodes.iter().any(|n| self.local_axes.contains_key(n)) {
                    concentrated.push(c.clone());
                    continue;
                }
                for node in nodes {
                    let Some(axes) = self.local_axes.get(&node) else {
                        concentrated.push(ConcentratedLoad {
                            region: NodeRef::Node(node),
                            ..c.clone()
                        });
                        continue;
                    };
                    let (dir, base) = if c.dof <= 3 {
                        (axes[c.dof as usize - 1], 1)
                    } else {
                        (axes[c.dof as usize - 4], 4)
                    };
                    for (i, &v) in dir.iter().enumerate() {
                        if v != 0.0 {
                            concentrated.push(ConcentratedLoad {
                                region: NodeRef::Node(node),
                                dof: base + i as u8,
                                value: c.value * v,
                                ..c.clone()
                            });
                        }
                    }
                }
            }
            for c in &concentrated {
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
            // Every node of a follower load names the same nodes: they are mapped once.
            let mut followers: Vec<(NodeRef, Follower)> = Vec::new();
            for load in &mut loads {
                if let Some(Follower::Nodes(nodes)) = &load.follower {
                    let mapped = match followers.iter().find(|(n, _)| n == nodes) {
                        Some((_, f)) => f.clone(),
                        None => {
                            let f = self.follower(original, nodes);
                            followers.push((nodes.clone(), f.clone()));
                            f
                        }
                    };
                    load.follower = Some(mapped);
                }
            }
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
            Kind::Shell if g.used[1] => {
                vec![(g.nodes[0], -0.5), (g.nodes[1], 2.0), (g.nodes[2], -0.5)]
            }
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
        if let Some(&helper) = self.u1_rotations.get(&node) {
            return Some((helper, d - 3));
        }
        if let Some(&(rotation, _)) = self.knots.get(&node) {
            return Some((rotation, d - 3));
        }
        if !self.groups.contains_key(&node) {
            return None;
        }
        self.mean_rotation(node, (d - 4) as usize).map(|h| (h, 1))
    }

    /// What a follower load on nodes of the original model turns with in the expanded one:
    /// a single node with a knot or with U1 rotations with its rotation node, nodes of
    /// beams and shells with the nodes they expand to.
    fn follower(&mut self, original: &Model, nodes: &NodeRef) -> Follower {
        let ids = original.nodes_of(nodes);
        if let [single] = ids[..] {
            if let Some(&helper) = self.u1_rotations.get(&single) {
                return Follower::Rotation(helper);
            }
            if let Some(&(rotation, _)) = self.knots.get(&single) {
                return Follower::Rotation(rotation);
            }
        }
        if !ids.iter().any(|n| self.groups.contains_key(n)) {
            return Follower::Nodes(nodes.clone());
        }
        let mut expanded = Vec::new();
        // An expanded node stays in the model without motion of its own (its results come
        // from the nodes it expands to): only those take part.
        for id in ids {
            match self.groups.get(&id) {
                Some(groups) => {
                    for group in groups {
                        let used = group.nodes.iter().zip(&group.used).filter(|(_, u)| **u);
                        expanded.extend(used.map(|(&n, _)| n));
                    }
                }
                None => expanded.push(id),
            }
        }
        expanded.sort_unstable();
        expanded.dedup();
        let sets = &mut self.analysis.model.sets.node_sets;
        let name = (1..)
            .map(|k| format!("PLXS-FOLLOWER-{k}"))
            .find(|name| !sets.contains_key(name))
            .unwrap_or_default();
        sets.insert(name.clone(), expanded);
        Follower::Nodes(NodeRef::Set(name))
    }

    fn remap_pressure(
        &self,
        original: &Model,
        region: &mut ElementRef,
        kind: &mut DistributedLoadKind,
    ) -> Result<()> {
        let ids = original.elements_of(region);
        let structural = ids
            .iter()
            .any(|&id| original.element(id).is_some_and(|e| expands(e.kind)));
        if !structural {
            return Ok(());
        }
        let family = original.element(ids[0]).map(|e| e.kind.family());
        match kind {
            // Shells: `P` is a pressure on the bottom face, acting along the normal.
            DistributedLoadKind::Other { label, values }
                if label == "P"
                    && matches!(family, Some(ElementFamily::Shell | ElementFamily::Membrane)) =>
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

/// Normals (and beam tangents) within this angle (degrees) of the reference element's
/// count as one. The CalculiX manual ("Shell elements", "Beam elements") takes 0.5° only
/// for normals the user gave with `*NORMAL`, which plxs does not read, and 20° for the
/// normals computed from the geometry: with 0.5° nearly every node of a curved quadratic
/// shell mesh became a knot (adjacent elements' normals differ by several degrees), which
/// multiplied the nodes and constraints of the expanded model.
const NORMAL_ANGLE: f64 = 20.0;

/// A range of degrees of freedom, first and last.
type DofRange = Option<(u8, u8)>;

/// The translational and the rotational part of a DOF range.
fn split_dofs(first: u8, last: u8) -> (DofRange, DofRange) {
    let translation = (first <= 3).then(|| (first, last.min(3)));
    let rotation = (last >= 4 && first <= 6).then(|| (first.max(4), last.min(6)));
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
