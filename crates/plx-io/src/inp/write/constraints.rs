//! Constraints written as CalculiX elements, the way the `CalculixFileWriter` splits
//! them: springs to ground become `SPRING1` elements with one `*Spring` section per node
//! and direction. Compression only supports on faces of solids or 2D elements become a
//! `*Compression only` surface of the solver (gaps at the integration points of the faces:
//! `GAPUNI` elements at the nodes leave the corners of quadratic faces without load and the
//! stresses checkered); those on shells become `GAPUNI` elements to fixed ground nodes.
//! Springs to a surface become `SPRING2` elements from each node of
//! the region to a new node that `*Equation`s tie to the closest point of the master
//! surface. Contact regions of no separation become one `*Equation` per slave node that
//! holds it to the closest point of the master normal to the surface.

use std::collections::BTreeMap;

use plx_mesh::{ElementId, FeMesh, NodeId};
use plx_model::{
    CompressionOnly, Constraint, ContactBehavior, ContactRegion, FeModel, ModelSpace, Region,
    SideRoles, Spring, SpringLayout,
};

use super::{Members, Sets, WriteError, empty, number};

/// What the constraints add to the input file besides the mesh.
#[derive(Default)]
pub(super) struct Generated {
    /// New nodes, appended to the `*Node` block.
    pub nodes: Vec<(NodeId, [f64; 3])>,
    /// `*Element` blocks of spring and gap elements.
    pub elements: Vec<ElementBlock>,
    /// `*Spring` and `*Gap` sections.
    pub sections: Vec<GeneratedSection>,
    /// `*Equation`s, written under the "Constraints" title after the ties.
    pub equations: Vec<Vec<super::dofs::Term>>,
    /// Holds the ground nodes of the gap elements in every step: the node set and the
    /// number of translations held.
    pub boundary: Option<(String, usize)>,
    /// The dummy plastic material that makes the solver solve gaps nonlinearly, by
    /// its name.
    pub material: Option<String>,
    /// `*Compression only` supports on element surfaces.
    pub compression_only: Vec<plxs_core::contact::CompressionOnly>,
    /// The surface of each compression only support written as a surface support, which a
    /// contact history output prints the contact forces of.
    pub support_surfaces: Vec<(plx_model::Id, String)>,
}

/// An `*Element` block of generated elements.
pub(super) struct ElementBlock {
    pub kind: &'static str,
    pub elset: Option<String>,
    pub elements: Vec<(ElementId, Vec<NodeId>)>,
}

/// A section of generated elements.
pub(super) enum GeneratedSection {
    /// `*Spring`: the degrees of freedom (one for SPRING1, two for SPRING2) and the
    /// stiffness.
    Spring {
        elset: String,
        dofs: Vec<u8>,
        stiffness: f64,
        /// `Stabilize=`, see [`Spring::stabilize`]; zero for none.
        stabilize: f64,
    },
    /// `*Gap`: clearance, direction, and spring stiffness and tensile force if given.
    Gap {
        elset: String,
        clearance: f64,
        direction: [f64; 3],
        properties: Option<(f64, f64)>,
    },
}

/// Adds the elements, sections and equations of the active springs and supports to `out`,
/// which holds what other constraints generated before; ties are written as `*Tie` by the
/// caller. Their equations come last for the dependent degrees of freedom (see
/// [`super::dofs`]).
pub(super) fn springs(
    sets: &mut Sets,
    model: &FeModel,
    out: &mut Generated,
) -> Result<(), WriteError> {
    let mesh = sets.mesh;
    let mut writer = Writer {
        next_node: (mesh.node_ids().iter().copied())
            .chain(out.nodes.iter().map(|(id, _)| *id))
            .max()
            .map_or(1, |n| n + 1),
        out: std::mem::take(out),
        next_element: mesh
            .elements()
            .iter()
            .map(|e| e.id)
            .max()
            .map_or(1, |e| e + 1),
        equations: Vec::new(),
        gaps: Vec::new(),
        ground: Vec::new(),
        dofs: if model.properties.space.is_2d() { 2 } else { 3 },
        space: model.properties.space,
        stabilize: 0.0,
    };
    let mut nonlinear = false;
    // The lowered model keeps the active regions of no separation only.
    let mut held = std::collections::BTreeSet::new();
    for region in &model.contact_regions {
        if region.active && region.behavior == ContactBehavior::NoSeparation {
            writer.no_separation(mesh, region, &mut held)?;
        }
    }
    for constraint in model.constraints.iter().filter(|c| c.active()) {
        match constraint {
            Constraint::Spring(spring) => {
                writer.stabilize = spring.stabilize;
                writer.spring(sets, spring)?;
                writer.stabilize = 0.0;
            }
            Constraint::CompressionOnly(support) => {
                writer.compression_only(sets, support)?;
                nonlinear |= support.nonlinear;
            }
            // Written as `*Rigid body` by the caller.
            Constraint::RigidBody(_) | Constraint::Joint(_) | Constraint::PointMass(_) => {}
            Constraint::Tie(_) | Constraint::NodeTie(_) => {}
        }
    }
    *out = writer.out;
    if !writer.gaps.is_empty() {
        out.elements.push(ElementBlock {
            kind: "GAPUNI",
            elset: None,
            elements: (writer.gaps.iter())
                .map(|(element, nodes)| (*element, nodes.to_vec()))
                .collect(),
        });
        let set = sets.unique("Internal_All_Compression_Only_Constraints_NodeSet");
        sets.node_sets.push((set.clone(), writer.ground));
        out.boundary = Some((set, writer.dofs));
    }
    if nonlinear {
        out.material = Some(sets.unique("Internal_compression_only-1"));
    }
    for (item, mut equation) in writer.equations {
        let movable = |n: NodeId| mesh.node(n).is_some();
        if sets.dofs.claim_equation(&item, &mut equation, movable) {
            out.equations.push(equation);
        }
    }
    Ok(())
}

struct Writer {
    out: Generated,
    next_node: NodeId,
    next_element: ElementId,
    /// Linear equations as (node, degree of freedom, factor) terms, the dependent term first,
    /// with the name of the constraint or contact region they belong to.
    equations: Vec<(String, Vec<super::dofs::Term>)>,
    /// Gap elements with their ground and surface node.
    gaps: Vec<(ElementId, [NodeId; 2])>,
    ground: Vec<NodeId>,
    /// Translational degrees of freedom of the model: ccx crashes on more than two in 2D.
    dofs: usize,
    space: ModelSpace,
    /// `Stabilize=` of the springs written now ([`Spring::stabilize`]).
    stabilize: f64,
}

impl Writer {
    fn node(&mut self, coords: [f64; 3]) -> NodeId {
        let id = self.next_node;
        self.next_node += 1;
        self.out.nodes.push((id, coords));
        id
    }

    fn element(&mut self) -> ElementId {
        let id = self.next_element;
        self.next_element += 1;
        id
    }

    /// Springs of one constraint and one element block with all of them, like
    /// point springs. Each group of springs shares the stiffness, an element set and a
    /// `*Spring` section per direction; it is (name, nodes of each spring, stiffness).
    fn springs(
        &mut self,
        sets: &mut Sets,
        name: &str,
        groups: Vec<(String, Vec<Vec<NodeId>>, [f64; 3])>,
    ) {
        self.springs_in(sets, name, groups, self.dofs);
    }

    /// [`Writer::springs`] in the first `dofs` degrees of freedom.
    fn springs_in(
        &mut self,
        sets: &mut Sets,
        name: &str,
        groups: Vec<(String, Vec<Vec<NodeId>>, [f64; 3])>,
        dofs: usize,
    ) {
        let two_nodes = (groups.first())
            .and_then(|(_, springs, _)| springs.first())
            .is_some_and(|nodes| nodes.len() == 2);
        let mut block = Vec::new();
        for (group, springs, stiffness) in groups {
            for (dof, k) in (1..).zip(stiffness).take(dofs).filter(|(_, k)| *k != 0.0) {
                let mut elements = Vec::new();
                for nodes in &springs {
                    let element = self.element();
                    elements.push(element);
                    block.push((element, nodes.clone()));
                }
                let set = sets.unique(&format!("{group}_DOF_{dof}"));
                sets.element_sets
                    .push((set.clone(), Members::Ids(elements)));
                let dof = dof as u8;
                self.out.sections.push(GeneratedSection::Spring {
                    elset: set,
                    dofs: if two_nodes { vec![dof, dof] } else { vec![dof] },
                    stiffness: k,
                    stabilize: self.stabilize,
                });
            }
        }
        if block.is_empty() {
            return;
        }
        let kind = if two_nodes { "SPRING2" } else { "SPRING1" };
        let set = sets.unique(&format!("{name}_All"));
        self.out.elements.push(ElementBlock {
            kind,
            elset: Some(set),
            elements: block,
        });
    }

    fn spring(&mut self, sets: &mut Sets, spring: &Spring) -> Result<(), WriteError> {
        let Some(layout) = spring.layout() else {
            return Err(WriteError::InvalidConstraint {
                item: spring.name.clone(),
                reason: "both ends are World".into(),
            });
        };
        let name = super::name(&spring.name);
        let selection = match layout {
            SpringLayout::SelectionToWorld { selection }
            | SpringLayout::SelectionToSurface { selection, .. }
            | SpringLayout::SelectionToPoint { selection, .. } => selection,
            SpringLayout::PointToWorld { point } => {
                // Translations on the reference node, rotations on the rotation node.
                let ([reference, rotation], _) = super::point_nodes(sets, &spring.name, point)?;
                self.point_springs(sets, &name, spring, [vec![reference], vec![rotation]]);
                return Ok(());
            }
            SpringLayout::PointToPoint { first, second } => {
                let ([r1, w1], _) = super::point_nodes(sets, &spring.name, first)?;
                let ([r2, w2], _) = super::point_nodes(sets, &spring.name, second)?;
                if r1 == r2 {
                    return Err(WriteError::InvalidConstraint {
                        item: spring.name.clone(),
                        reason: "both ends are the same point".into(),
                    });
                }
                self.point_springs(sets, &name, spring, [vec![r1, r2], vec![w1, w2]]);
                return Ok(());
            }
        };
        let weights = spring_shares(sets.mesh, spring, selection);
        if weights.is_empty() {
            return Err(empty(&spring.name, "nodes"));
        }
        let surface = match layout {
            SpringLayout::SelectionToSurface { surface, .. } => surface,
            SpringLayout::SelectionToPoint { point, .. } => {
                // Every node gets springs to the point's reference node.
                let ([reference, _], _) = super::point_nodes(sets, &spring.name, point)?;
                let springs = (weights.iter())
                    .filter(|&(&node, _)| node != reference)
                    .map(|(&node, w)| {
                        let k = spring.stiffness.map(|k| k * w);
                        (format!("{name}_{node}"), vec![vec![node, reference]], k)
                    })
                    .collect();
                self.springs(sets, &name, springs);
                return Ok(());
            }
            _ => {
                // Springs of equal stiffness share their sections.
                let first = weights.values().next().copied().unwrap_or(0.0);
                let springs = if weights.values().all(|&w| w == first) {
                    let nodes = weights.keys().map(|&node| vec![node]).collect();
                    vec![(name.clone(), nodes, spring.stiffness.map(|k| k * first))]
                } else {
                    (weights.iter())
                        .map(|(&node, w)| {
                            let k = spring.stiffness.map(|k| k * w);
                            (format!("{name}_{node}"), vec![vec![node]], k)
                        })
                        .collect()
                };
                self.springs(sets, &name, springs);
                return Ok(());
            }
        };
        let mesh = sets.mesh;
        let target = Target::new(mesh, &surface.faces(mesh));
        if target.faces.is_empty() {
            return Err(empty(&spring.name, "element faces on the second end"));
        }
        let mut springs = Vec::new();
        for (&node, &w) in &weights {
            let Some(coords) = mesh.node(node) else {
                continue;
            };
            let Some((point, terms)) = target.closest(coords) else {
                continue;
            };
            // A node on both surfaces needs no spring; a node on top of a target node is
            // connected to it directly.
            let other = match terms.as_slice() {
                [(other, _)] if *other == node => continue,
                [(other, _)] => *other,
                _ => {
                    let other = self.node(point);
                    for dof in 1..=3 {
                        let mut equation = vec![(other, dof, 1.0)];
                        equation.extend(terms.iter().map(|&(n, f)| (n, dof, -f)));
                        self.equations.push((spring.name.clone(), equation));
                    }
                    other
                }
            };
            let k = spring.stiffness.map(|k| k * w);
            springs.push((format!("{name}_{node}"), vec![vec![node, other]], k));
        }
        self.springs(sets, &name, springs);
        Ok(())
    }

    /// Springs of a point to ground (one node each) or between two points (two nodes each):
    /// the translational ones on the reference nodes, the rotational ones on the rotation
    /// nodes, whose displacements are the point's rotations. A plane model turns about z
    /// only, the third degree of freedom of the rotation node; an axisymmetric one not at
    /// all.
    fn point_springs(
        &mut self,
        sets: &mut Sets,
        name: &str,
        spring: &Spring,
        [reference, rotation]: [Vec<NodeId>; 2],
    ) {
        let [r1, r2, r3] = spring.acting_rotational();
        let rotational = match self.space {
            ModelSpace::ThreeD => [r1, r2, r3],
            ModelSpace::PlaneStress | ModelSpace::PlaneStrain => [0.0, 0.0, r3],
            ModelSpace::Axisymmetric => [0.0; 3],
        };
        let springs = vec![(name.to_string(), vec![reference], spring.stiffness)];
        self.springs(sets, name, springs);
        let rotations = vec![(format!("{name}_Rotation"), vec![rotation], rotational)];
        self.springs_in(sets, &format!("{name}_Rotation"), rotations, 3);
    }

    /// One equation per slave node within the tolerance of the master side: the slave node
    /// moves with the closest point of the master normal to the surface, while it slides
    /// freely along it. The slave node's largest component of the normal is the dependent
    /// term. Nodes `held` by an earlier region are left out, as a node can be the dependent
    /// one of a single equation only.
    fn no_separation(
        &mut self,
        mesh: &FeMesh,
        region: &ContactRegion,
        held: &mut std::collections::BTreeSet<NodeId>,
    ) -> Result<(), WriteError> {
        let (master, slave) = match region.settings.sides {
            Some(SideRoles::BIsMaster) => (&region.side_b, &region.side_a),
            _ => (&region.side_a, &region.side_b),
        };
        let target = Target::new(mesh, &master.faces(mesh));
        let slave_faces = slave.faces(mesh);
        if target.faces.is_empty() || slave_faces.is_empty() {
            return Err(empty(&region.name, "element faces on both sides"));
        }
        let mut normals: BTreeMap<NodeId, [f64; 3]> = BTreeMap::new();
        for &(element, face) in &slave_faces {
            let Some(normal) = face_normal(mesh, element, face) else {
                continue;
            };
            for node in face_node_ids(mesh, element, face) {
                let sum = normals.entry(node).or_default();
                for k in 0..3 {
                    sum[k] += normal[k];
                }
            }
        }
        let tolerance = region.settings.tolerance.unwrap_or(f64::INFINITY);
        for (node, normal) in normals {
            let Some(coords) = mesh.node(node) else {
                continue;
            };
            let Some((point, terms)) = target.closest(coords) else {
                continue;
            };
            let gap = sub(point, coords);
            if dot(gap, gap).sqrt() > tolerance || held.contains(&node) {
                continue;
            }
            if let [(other, _)] = terms.as_slice()
                && *other == node
            {
                continue;
            }
            let normal = normalized(normal);
            let dofs = 1..=self.dofs as u8;
            let Some(dependent) = (dofs.clone()).max_by(|&i, &j| {
                normal[usize::from(i) - 1]
                    .abs()
                    .total_cmp(&normal[usize::from(j) - 1].abs())
            }) else {
                continue;
            };
            let component = |dof: u8| normal[usize::from(dof) - 1];
            let mut equation = vec![(node, dependent, component(dependent))];
            for dof in dofs.clone().filter(|&d| d != dependent) {
                if component(dof).abs() > 1e-9 {
                    equation.push((node, dof, component(dof)));
                }
            }
            for &(other, weight) in &terms {
                for dof in dofs.clone() {
                    let factor = -weight * component(dof);
                    if factor.abs() > 1e-9 {
                        equation.push((other, dof, factor));
                    }
                }
            }
            held.insert(node);
            self.equations.push((region.name.clone(), equation));
        }
        Ok(())
    }

    fn compression_only(
        &mut self,
        sets: &mut Sets,
        support: &CompressionOnly,
    ) -> Result<(), WriteError> {
        let mesh = sets.mesh;
        let faces = support.region.faces(mesh);
        let (weights, area) = node_areas(mesh, &faces, false, true);
        if weights.is_empty() {
            return Err(empty(&support.name, "element faces"));
        }
        let on_shells = (faces.iter()).any(|(e, _)| mesh.element(*e).is_none_or(|e| e.is_shell()));
        if !on_shells {
            // The stiffness of the whole surface, or of every node, as a stiffness per area;
            // the tensile force of CalculiX's gaps has no counterpart.
            let stiffness = support.spring_stiffness.map(|k| {
                if support.each_node {
                    k * weights.len() as f64 / area
                } else {
                    k / area
                }
            });
            let surface = sets.surface(&support.name, "Surface", &support.region)?;
            self.out
                .support_surfaces
                .push((support.id, surface.clone()));
            self.out
                .compression_only
                .push(plxs_core::contact::CompressionOnly {
                    // The solver model knows its surfaces by their normalized names.
                    surface: super::ir_name(&surface),
                    clearance: support.clearance,
                    stiffness,
                    flipped: support.flipped,
                });
            return Ok(());
        }
        // Normal of each node: the mean of the outward normals of its faces.
        let mut normals: BTreeMap<NodeId, [f64; 3]> = BTreeMap::new();
        for &(element, face) in &faces {
            let Some(normal) = outward_normal(mesh, element, face) else {
                continue;
            };
            for node in face_node_ids(mesh, element, face) {
                let sum = normals.entry(node).or_default();
                for k in 0..3 {
                    sum[k] += normal[k];
                }
            }
        }
        // A value given is spread over the nodes by area, or acts whole at every node; one
        // not given is CalculiX's default for each gap.
        let share = |value: Option<f64>, default: f64, w: f64| match value {
            Some(v) if support.each_node => v,
            Some(v) => v * w / area,
            None => default,
        };
        let name = super::name(&support.name);
        // Gaps of the same direction and properties, such as those of a flat face, share an
        // element set and a `*Gap` section: the solver makes a material of each section.
        type Line = (String, [f64; 3], Option<(f64, f64)>);
        let mut sections: Vec<(Line, Vec<ElementId>)> = Vec::new();
        let mut by_line: BTreeMap<String, usize> = BTreeMap::new();
        for (&node, &w) in weights.iter().filter(|(_, w)| **w != 0.0) {
            let (Some(coords), Some(normal)) = (mesh.node(node), normals.get(&node)) else {
                continue;
            };
            // The gap points the way the support pushes, from the ground node
            // to the surface node, so that it closes when the surface moves towards the
            // ground: into the material from outside, or out of it when flipped.
            let normal = support.push_direction(normalized(*normal));
            // Rounded, so that the gaps of a flat face get the same direction.
            let direction = normal.map(|x| (x * 1e9).round() / 1e9 + 0.0);
            let ground = self.node(std::array::from_fn(|k| {
                coords[k] - support.offset * normal[k]
            }));
            self.ground.push(ground);
            let element = self.element();
            self.gaps.push((element, [ground, node]));
            let values = (support.spring_stiffness.is_some() || support.tensile_force.is_some())
                .then(|| {
                    (
                        share(
                            support.spring_stiffness,
                            CompressionOnly::DEFAULT_STIFFNESS,
                            w,
                        ),
                        share(
                            support.tensile_force,
                            CompressionOnly::DEFAULT_TENSILE_FORCE,
                            w,
                        ),
                    )
                });
            let properties = values
                .map(|(k, f)| format!(", , {}, {}", number(k), number(f)))
                .unwrap_or_default();
            let line = format!(
                "{}, {}, {}, {}{properties}",
                number(support.clearance),
                number(direction[0]),
                number(direction[1]),
                number(direction[2])
            );
            let index = *by_line.entry(line.clone()).or_insert_with(|| {
                sections.push(((line, direction, values), Vec::new()));
                sections.len() - 1
            });
            sections[index].1.push(element);
        }
        for (count, ((_, direction, properties), elements)) in (1..).zip(sections) {
            let set = sets.unique(&format!("{name}_ElementSet-{count}"));
            sets.element_sets
                .push((set.clone(), Members::Ids(elements)));
            self.out.sections.push(GeneratedSection::Gap {
                elset: set,
                clearance: support.clearance,
                direction,
                properties,
            });
        }
        Ok(())
    }
}

/// The factor of a spring's stiffness at each node of its region: like a force on faces by
/// area (lumped, so that every node gets a positive share), on edges by length and on points
/// in equal shares, or 1 at every node with `each_node`. Nodes with no share get no spring.
fn spring_shares(mesh: &FeMesh, spring: &Spring, region: &Region) -> BTreeMap<NodeId, f64> {
    if spring.each_node {
        return region.nodes(mesh).into_iter().map(|n| (n, 1.0)).collect();
    }
    let faces = region.faces(mesh);
    let shares: Vec<(NodeId, f64)> = if !faces.is_empty() {
        let (weights, area) = node_areas(mesh, &faces, false, true);
        let scale = if spring.per_area { 1.0 } else { 1.0 / area };
        weights.into_iter().map(|(n, w)| (n, w * scale)).collect()
    } else {
        let edges = region.edges(mesh);
        if !edges.is_empty() {
            (super::edge_traction_forces(mesh, &edges, [1.0, 0.0, 0.0], false).into_iter())
                .map(|(n, share)| (n, share[0]))
                .collect()
        } else {
            let nodes = region.nodes(mesh);
            let share = 1.0 / nodes.len().max(1) as f64;
            nodes.into_iter().map(|n| (n, share)).collect()
        }
    };
    shares.into_iter().filter(|(_, w)| *w != 0.0).collect()
}

/// Share of the area of the faces that each node takes and the total area.
/// Within a face the share follows the equivalent
/// nodal forces of a constant pressure: equal parts on linear faces, only midside nodes on
/// quadratic triangles, and -1/12 per corner and 1/3 per midside node on quadratic
/// quadrilaterals. With `lumped` quadratic faces share their area as the diagonal of their
/// mass matrix instead (Hinton-Rock-Zienkiewicz lumping): 1/19 per corner and 16/57 per
/// midside node on triangles, 3/76 and 4/19 on quadrilaterals. Stiffness spread like a
/// pressure would be negative at the corners of quadratic quadrilaterals.
pub(super) fn node_areas(
    mesh: &FeMesh,
    faces: &[(ElementId, u8)],
    axisymmetric: bool,
    lumped: bool,
) -> (BTreeMap<NodeId, f64>, f64) {
    let mut weights: BTreeMap<NodeId, f64> = BTreeMap::new();
    let mut total_area = 0.0;
    for &(element, face) in faces {
        let Some(element) = mesh.element(element) else {
            continue;
        };
        let Some(topology) = (face as usize)
            .checked_sub(1)
            .and_then(|f| element.faces().get(f))
        else {
            continue;
        };
        let node = |local: usize| element.nodes.get(local).copied();
        let point = |local: usize| node(local).and_then(|id| mesh.node(id));
        let corners: Vec<[f64; 3]> = topology.corners.iter().filter_map(|&l| point(l)).collect();
        if corners.len() != topology.corners.len() {
            continue;
        }
        let quadratic = element.shape.is_quadratic() && !topology.mids.is_empty();
        // Edges of 2D elements: their length, or for axisymmetric ones length times radius.
        if let [a, b] = corners[..] {
            let ends = [topology.corners[0], topology.corners[1]];
            let mid = topology.mids.first().copied().filter(|_| quadratic);
            let (nodes, edge) = match mid.and_then(|m| Some((m, point(m)?))) {
                Some((m, pm)) => (
                    vec![ends[0], ends[1], m],
                    super::edge_weights(&[a, b, pm], axisymmetric),
                ),
                None => (ends.to_vec(), super::edge_weights(&[a, b], axisymmetric)),
            };
            total_area += edge.iter().sum::<f64>();
            for (local, weight) in nodes.into_iter().zip(edge) {
                if let Some(id) = node(local) {
                    *weights.entry(id).or_default() += weight;
                }
            }
            continue;
        }
        let area = super::polygon_area(&corners);
        total_area += area;
        let (corner_weight, mid_weight) = match (corners.len(), quadratic) {
            (3, true) if lumped => (1.0 / 19.0, 16.0 / 57.0),
            (4, true) if lumped => (3.0 / 76.0, 4.0 / 19.0),
            (3, true) => (0.0, 1.0 / 3.0),
            (4, true) => (-1.0 / 12.0, 1.0 / 3.0),
            (n, _) => (1.0 / n as f64, 0.0),
        };
        for &local in topology.corners {
            if let Some(id) = node(local) {
                *weights.entry(id).or_default() += area * corner_weight;
            }
        }
        if quadratic {
            for &local in topology.mids {
                if let Some(id) = node(local) {
                    *weights.entry(id).or_default() += area * mid_weight;
                }
            }
        }
    }
    (weights, total_area)
}

/// Nodes of an element face: corners, then midside nodes of quadratic elements.
pub(super) fn face_node_ids(mesh: &FeMesh, element: ElementId, face: u8) -> Vec<NodeId> {
    let Some(element) = mesh.element(element) else {
        return Vec::new();
    };
    let Some(topology) = (face as usize)
        .checked_sub(1)
        .and_then(|f| element.faces().get(f))
    else {
        return Vec::new();
    };
    let mids: &[usize] = if element.shape.is_quadratic() {
        topology.mids
    } else {
        &[]
    };
    (topology.corners.iter().chain(mids))
        .filter_map(|&l| element.nodes.get(l).copied())
        .collect()
}

/// Unit normal of an element face pointing out of a solid element; for a shell, the normal
/// of its first face.
pub(super) fn outward_normal(mesh: &FeMesh, element: ElementId, face: u8) -> Option<[f64; 3]> {
    let e = mesh.element(element)?;
    let topology = e.faces().get(usize::from(face).checked_sub(1)?)?;
    let corners: Vec<[f64; 3]> = (topology.corners.iter())
        .map(|&l| e.nodes.get(l).and_then(|&n| mesh.node(n)))
        .collect::<Option<_>>()?;
    let mut normal = [0.0; 3];
    for (i, a) in corners.iter().enumerate() {
        let b = corners[(i + 1) % corners.len()];
        normal[0] += a[1] * b[2] - a[2] * b[1];
        normal[1] += a[2] * b[0] - a[0] * b[2];
        normal[2] += a[0] * b[1] - a[1] * b[0];
    }
    if e.shape.family() == plx_mesh::ElementFamily::Solid {
        let all: Vec<[f64; 3]> = e.nodes.iter().filter_map(|&n| mesh.node(n)).collect();
        let centre = |points: &[[f64; 3]]| -> [f64; 3] {
            std::array::from_fn(|k| points.iter().map(|p| p[k]).sum::<f64>() / points.len() as f64)
        };
        let (face_centre, element_centre) = (centre(&corners), centre(&all));
        let outward: f64 = (0..3)
            .map(|k| normal[k] * (face_centre[k] - element_centre[k]))
            .sum();
        if outward < 0.0 {
            normal = normal.map(|v| -v);
        }
    }
    let length = dot(normal, normal).sqrt();
    (length > 0.0).then(|| normal.map(|v| v / length))
}

/// [`outward_normal`] of a face; of an edge of a 2D element the normal in the plane of the
/// model pointing out of the element.
fn face_normal(mesh: &FeMesh, element: ElementId, face: u8) -> Option<[f64; 3]> {
    if let Some(normal) = outward_normal(mesh, element, face) {
        return Some(normal);
    }
    let e = mesh.element(element)?;
    let topology = e.faces().get(usize::from(face).checked_sub(1)?)?;
    let [a, b] = topology.corners[..] else {
        return None;
    };
    let (a, b) = (mesh.node(*e.nodes.get(a)?)?, mesh.node(*e.nodes.get(b)?)?);
    let mut normal = [b[1] - a[1], a[0] - b[0], 0.0];
    let all: Vec<[f64; 3]> = e.nodes.iter().filter_map(|&n| mesh.node(n)).collect();
    let centre: [f64; 3] =
        std::array::from_fn(|k| all.iter().map(|p| p[k]).sum::<f64>() / all.len() as f64);
    if dot(normal, sub(a, centre)) < 0.0 {
        normal = normal.map(|v| -v);
    }
    let length = dot(normal, normal).sqrt();
    (length > 0.0).then(|| normal.map(|v| v / length))
}

fn normalized(v: [f64; 3]) -> [f64; 3] {
    let length = dot(v, v).sqrt();
    if length > 0.0 {
        v.map(|x| x / length)
    } else {
        [0.0, 0.0, 1.0]
    }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// Nodes with their shape function value at a point.
type Terms = Vec<(NodeId, f64)>;

/// A face of the target surface: its corner coordinates, its nodes (corners, then midside
/// nodes on quadratic faces) and its bounding box.
struct TargetFace {
    corners: Vec<[f64; 3]>,
    nodes: Vec<NodeId>,
    min: [f64; 3],
    max: [f64; 3],
}

/// The master surface of a spring connection, searched for the closest point of each slave
/// node.
struct Target {
    faces: Vec<TargetFace>,
}

impl Target {
    fn new(mesh: &FeMesh, faces: &[(ElementId, u8)]) -> Self {
        let faces = faces
            .iter()
            .filter_map(|&(element, face)| {
                let e = mesh.element(element)?;
                let topology = e.faces().get(usize::from(face).checked_sub(1)?)?;
                let corners: Vec<[f64; 3]> = (topology.corners.iter())
                    .map(|&l| e.nodes.get(l).and_then(|&n| mesh.node(n)))
                    .collect::<Option<_>>()?;
                let nodes = face_node_ids(mesh, element, face);
                let min =
                    std::array::from_fn(|k| corners.iter().map(|c| c[k]).fold(f64::MAX, f64::min));
                let max =
                    std::array::from_fn(|k| corners.iter().map(|c| c[k]).fold(f64::MIN, f64::max));
                Some(TargetFace {
                    corners,
                    nodes,
                    min,
                    max,
                })
            })
            .collect();
        Self { faces }
    }

    /// Closest point of the surface to `p` and the shape function values of the face's
    /// nodes there, without the nodes whose value is zero.
    fn closest(&self, p: [f64; 3]) -> Option<([f64; 3], Terms)> {
        let mut best: Option<(f64, usize, [f64; 3])> = None;
        for (index, face) in self.faces.iter().enumerate() {
            // Distance to the bounding box is a lower bound of the distance to the face.
            let gap: f64 = (0..3)
                .map(|k| {
                    (face.min[k] - p[k])
                        .max(p[k] - face.max[k])
                        .max(0.0)
                        .powi(2)
                })
                .sum();
            if best.is_some_and(|(d, ..)| gap >= d) {
                continue;
            }
            let c = &face.corners;
            let triangles: &[[usize; 3]] = if c.len() == 4 {
                &[[0, 1, 2], [0, 2, 3]]
            } else {
                &[[0, 1, 2]]
            };
            for t in triangles {
                let q = closest_on_triangle(p, c[t[0]], c[t[1]], c[t[2]]);
                let d = dot(sub(q, p), sub(q, p));
                if best.is_none_or(|(b, ..)| d < b) {
                    best = Some((d, index, q));
                }
            }
        }
        let (_, index, point) = best?;
        let face = &self.faces[index];
        let values = shape_functions(&face.corners, face.nodes.len(), point);
        let terms: Vec<(NodeId, f64)> = (face.nodes.iter().copied())
            .zip(values)
            .filter(|(_, v)| v.abs() > 1e-9)
            .collect();
        // On a node all others vanish.
        match terms.as_slice() {
            [(node, v)] if (v - 1.0).abs() < 1e-9 => Some((point, vec![(*node, 1.0)])),
            _ => Some((point, terms)),
        }
    }
}

/// Closest point of triangle `abc` to `p` (Ericson, Real-Time Collision Detection 5.1.5).
fn closest_on_triangle(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> [f64; 3] {
    let at = |s: f64, from: [f64; 3], d: [f64; 3]| -> [f64; 3] {
        std::array::from_fn(|k| from[k] + s * d[k])
    };
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return at(d1 / (d1 - d3), a, ab);
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return at(d2 / (d2 - d6), a, ac);
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return at((d4 - d3) / ((d4 - d3) + (d5 - d6)), b, sub(c, b));
    }
    let denominator = 1.0 / (va + vb + vc);
    let (v, w) = (vb * denominator, vc * denominator);
    std::array::from_fn(|k| a[k] + ab[k] * v + ac[k] * w)
}

/// Values of the shape functions of a face with `count` nodes (3, 6, 4 or 8, in CalculiX's
/// order) at a point on it. The local coordinates are found on the face spanned by the
/// corners.
fn shape_functions(corners: &[[f64; 3]], count: usize, p: [f64; 3]) -> Vec<f64> {
    if corners.len() == 3 {
        let [l1, l2, l3] = barycentric(p, corners[0], corners[1], corners[2]);
        if count == 6 {
            vec![
                l1 * (2.0 * l1 - 1.0),
                l2 * (2.0 * l2 - 1.0),
                l3 * (2.0 * l3 - 1.0),
                4.0 * l1 * l2,
                4.0 * l2 * l3,
                4.0 * l3 * l1,
            ]
        } else {
            vec![l1, l2, l3]
        }
    } else {
        let (xi, eta) = quad_coordinates(corners, p);
        let signs = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)];
        if count == 8 {
            let mut values: Vec<f64> = signs
                .iter()
                .map(|(s, t)| 0.25 * (1.0 + s * xi) * (1.0 + t * eta) * (s * xi + t * eta - 1.0))
                .collect();
            values.extend([
                0.5 * (1.0 - xi * xi) * (1.0 - eta),
                0.5 * (1.0 + xi) * (1.0 - eta * eta),
                0.5 * (1.0 - xi * xi) * (1.0 + eta),
                0.5 * (1.0 - xi) * (1.0 - eta * eta),
            ]);
            values
        } else {
            (signs.iter())
                .map(|(s, t)| 0.25 * (1.0 + s * xi) * (1.0 + t * eta))
                .collect()
        }
    }
}

fn barycentric(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> [f64; 3] {
    let (v0, v1, v2) = (sub(b, a), sub(c, a), sub(p, a));
    let (d00, d01, d11) = (dot(v0, v0), dot(v0, v1), dot(v1, v1));
    let (d20, d21) = (dot(v2, v0), dot(v2, v1));
    let denominator = d00 * d11 - d01 * d01;
    if denominator == 0.0 {
        return [1.0, 0.0, 0.0];
    }
    let v = (d11 * d20 - d01 * d21) / denominator;
    let w = (d00 * d21 - d01 * d20) / denominator;
    [1.0 - v - w, v, w]
}

/// Local coordinates (xi, eta) in [-1, 1] of a point on a bilinear quadrilateral, by
/// Gauss-Newton iteration.
fn quad_coordinates(c: &[[f64; 3]], p: [f64; 3]) -> (f64, f64) {
    let (mut xi, mut eta) = (0.0_f64, 0.0_f64);
    for _ in 0..20 {
        let n = [
            0.25 * (1.0 - xi) * (1.0 - eta),
            0.25 * (1.0 + xi) * (1.0 - eta),
            0.25 * (1.0 + xi) * (1.0 + eta),
            0.25 * (1.0 - xi) * (1.0 + eta),
        ];
        let dxi = [
            -0.25 * (1.0 - eta),
            0.25 * (1.0 - eta),
            0.25 * (1.0 + eta),
            -0.25 * (1.0 + eta),
        ];
        let deta = [
            -0.25 * (1.0 - xi),
            -0.25 * (1.0 + xi),
            0.25 * (1.0 + xi),
            0.25 * (1.0 - xi),
        ];
        let combine = |w: &[f64; 4]| -> [f64; 3] {
            std::array::from_fn(|k| (0..4).map(|i| w[i] * c[i][k]).sum())
        };
        let (x, t1, t2) = (combine(&n), combine(&dxi), combine(&deta));
        let r = sub(p, x);
        let (a11, a12, a22) = (dot(t1, t1), dot(t1, t2), dot(t2, t2));
        let (b1, b2) = (dot(t1, r), dot(t2, r));
        let det = a11 * a22 - a12 * a12;
        if det.abs() < 1e-300 {
            break;
        }
        let dx = (a22 * b1 - a12 * b2) / det;
        let de = (a11 * b2 - a12 * b1) / det;
        xi = (xi + dx).clamp(-1.0, 1.0);
        eta = (eta + de).clamp(-1.0, 1.0);
        if dx.abs() + de.abs() < 1e-12 {
            break;
        }
    }
    (xi, eta)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shape_functions_sum_to_one_and_hit_nodes() {
        let tri = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]];
        let quad = [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        for (corners, count) in [(&tri[..], 3), (&tri[..], 6), (&quad[..], 4), (&quad[..], 8)] {
            let values = shape_functions(corners, count, [0.5, 0.3, 0.0]);
            assert_eq!(values.len(), count);
            assert!((values.iter().sum::<f64>() - 1.0).abs() < 1e-12);
            let at_corner = shape_functions(corners, count, corners[1]);
            assert!((at_corner[1] - 1.0).abs() < 1e-9, "{at_corner:?}");
        }
        // Midside node of the first edge of a quadratic quadrilateral.
        let mid = shape_functions(&quad, 8, [1.0, 0.0, 0.0]);
        assert!((mid[4] - 1.0).abs() < 1e-9, "{mid:?}");
    }

    #[test]
    fn closest_point_on_a_triangle() {
        let (a, b, c) = ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        let near = |p: [f64; 3], q: [f64; 3]| (0..3).all(|k| (p[k] - q[k]).abs() < 1e-12);
        assert!(near(
            closest_on_triangle([0.2, 0.2, 5.0], a, b, c),
            [0.2, 0.2, 0.0]
        ));
        assert!(near(closest_on_triangle([-1.0, -1.0, 0.0], a, b, c), a));
        assert!(near(
            closest_on_triangle([0.5, -2.0, 1.0], a, b, c),
            [0.5, 0.0, 0.0]
        ));
        assert!(near(
            closest_on_triangle([1.0, 1.0, 0.0], a, b, c),
            [0.5, 0.5, 0.0]
        ));
    }
}
