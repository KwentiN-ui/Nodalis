//! Prescribed displacements and rotations of a selection. The solver holds the nodes of the
//! selection directly, or, where the selection should turn although its nodes have no
//! rotations (faces and edges of solids), a point, by default at the centre of the selection,
//! that the selection follows as a rigid body or on average (deformable), as Ansys' remote
//! displacement does. The point exists only in the input file.

use std::collections::{BTreeMap, BTreeSet};

use plx_mesh::{ElementFamily, ElementId, FeMesh, NodeId};
use serde::{Deserialize, Serialize};

use crate::{BoundaryKind, FeModel, ModelSpace, Region};

/// How the selection follows its prescribed displacements, Ansys' "Behavior".
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Behavior {
    /// Chosen from the selection, see [`Displacement::formulation`].
    #[default]
    Automatic,
    /// Every node is held directly.
    EachNode,
    /// The selection keeps its shape and follows a point (`*RIGID BODY`).
    Rigid,
    /// A point moves with the weighted mean of the selection, which may deform: Nastran's
    /// RBE3, written as `*EQUATION`s.
    Deformable,
}

impl Behavior {
    pub const ALL: [Behavior; 4] = [
        Self::Automatic,
        Self::EachNode,
        Self::Rigid,
        Self::Deformable,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Automatic => "Automatic",
            Self::EachNode => "Each node",
            Self::Rigid => "Rigid",
            Self::Deformable => "Deformable",
        }
    }
}

/// How the selection follows a point.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum RemoteBehavior {
    #[default]
    Rigid,
    Deformable,
}

/// How a displacement is written for CalculiX, see [`Displacement::formulation`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Formulation {
    /// `*BOUNDARY` on every node of the selection (a reference point: on its nodes).
    Nodes,
    /// `*BOUNDARY` on a point the selection follows.
    Point(RemoteBehavior),
}

/// Prescribed displacements and rotations of a selection.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Displacement {
    /// Displacements (U1..U3) and rotations (UR1..UR3) in the directions of `system`;
    /// `None` leaves one free.
    pub values: [Option<f64>; 6],
    pub behavior: Behavior,
    /// Coordinate system of the location and of the directions; `None` is the global one.
    pub system: Option<crate::Id>,
    /// Coordinates of the point in `system`; `None` is the centre of the selection. Setting
    /// it asks for a point.
    pub location: Option<[f64; 3]>,
    /// Only the nodes within this distance of the point take part, Ansys' pinball region.
    /// Setting it asks for a point.
    pub pinball: Option<f64>,
    /// A time table per degree of freedom that replaces its value in `values`: the value
    /// follows the table over the step time instead of the amplitude of the boundary
    /// condition. The degree of freedom must be held (`Some` in `values`); the number there
    /// is kept for when the table is removed.
    #[serde(default, skip_serializing_if = "crate::no_tables")]
    pub tables: [Option<crate::ValueTable>; 6],
}

/// A remote displacement resolved on a mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct RemotePoint {
    pub position: [f64; 3],
    /// The local x, y and z directions at the point; `None` for the global ones.
    pub axes: Option<[[f64; 3]; 3]>,
    /// The nodes that follow the point.
    pub nodes: Vec<NodeId>,
    /// Weight of each node in a deformable coupling: its share of the face areas.
    pub weights: Vec<f64>,
}

/// A degree of freedom of an equation: of a node of the mesh, or the translation or rotation
/// of the remote point in a global direction 0..3.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum RemoteDof {
    Node(NodeId, usize),
    Translation(usize),
    Rotation(usize),
}

impl FeModel {
    /// The displacement boundary condition `i` of step `s`.
    pub fn displacement(&self, s: usize, i: usize) -> Option<&Displacement> {
        match &self.steps.get(s)?.boundary_conditions.get(i)?.kind {
            BoundaryKind::Displacement(displacement) => Some(displacement),
            _ => None,
        }
    }

    /// How the displacement boundary condition `i` of step `s` is written. The point of a
    /// displacement exists in all steps of the input file, so one that needs a point in some
    /// active step gets it in all of them: the boundary conditions of the same name and
    /// region in the other steps decide too.
    pub fn formulation(&self, mesh: &FeMesh, s: usize, i: usize) -> Option<Formulation> {
        let step = self.steps.get(s)?;
        let bc = step.boundary_conditions.get(i)?;
        let BoundaryKind::Displacement(displacement) = &bc.kind else {
            return None;
        };
        if displacement.behavior != Behavior::Automatic {
            return Some(displacement.formulation(self, mesh, &bc.region, &step.name, &bc.name));
        }
        let point = (self.steps.iter().filter(|other| other.active))
            .flat_map(|other| other.boundary_conditions.iter().map(move |b| (other, b)))
            .filter(|(other, b)| {
                b.active
                    && b.name == bc.name
                    && b.region == bc.region
                    && other.kind.supports_boundary(&b.kind)
            })
            .chain(std::iter::once((step, bc)))
            .any(|(other, b)| match &b.kind {
                BoundaryKind::Displacement(d) if d.behavior == Behavior::Automatic => matches!(
                    d.formulation(self, mesh, &b.region, &other.name, &b.name),
                    Formulation::Point(_)
                ),
                _ => false,
            });
        Some(if point {
            Formulation::Point(RemoteBehavior::Rigid)
        } else {
            Formulation::Nodes
        })
    }

    /// The first displacement boundary condition of the id that is written with a point,
    /// of any step, with the index of its step.
    pub fn remote_displacement(
        &self,
        mesh: &FeMesh,
        id: crate::Id,
    ) -> Option<(usize, &crate::BoundaryCondition, &Displacement)> {
        self.steps.iter().enumerate().find_map(|(s, step)| {
            let (i, bc) =
                (step.boundary_conditions.iter().enumerate()).find(|(_, bc)| bc.id == id)?;
            let BoundaryKind::Displacement(displacement) = &bc.kind else {
                return None;
            };
            matches!(self.formulation(mesh, s, i), Some(Formulation::Point(_))).then_some((
                s,
                bc,
                displacement,
            ))
        })
    }

    /// The active steps in which the point of a displacement with `region` acts: those with
    /// an active displacement on a point of the same region and coupling. Displacements of
    /// several steps share that point.
    pub fn point_steps(
        &self,
        mesh: &FeMesh,
        region: &Region,
        displacement: &Displacement,
    ) -> BTreeSet<usize> {
        let mut steps = BTreeSet::new();
        for (s, step) in self.steps.iter().enumerate().filter(|(_, s)| s.active) {
            let acts = (step.boundary_conditions.iter().enumerate()).any(|(i, bc)| {
                bc.active
                    && step.kind.supports_boundary(&bc.kind)
                    && bc.region == *region
                    && matches!(&bc.kind, BoundaryKind::Displacement(d) if d.same_point(displacement))
                    && matches!(self.formulation(mesh, s, i), Some(Formulation::Point(_)))
            });
            if acts {
                steps.insert(s);
            }
        }
        steps
    }

    /// The nodes a point of a displacement with `region` leaves out of its coupling: those
    /// that boundary conditions hold directly in a step in which the point acts, where the
    /// equations of the point and the boundary conditions would bind the same degree of
    /// freedom. Supports of the other steps do not count: there the point is not held and
    /// the solver takes its equations around the held nodes.
    pub fn point_excluded_nodes(
        &self,
        mesh: &FeMesh,
        region: &Region,
        displacement: &Displacement,
    ) -> BTreeSet<NodeId> {
        let steps = self.point_steps(mesh, region, displacement);
        self.directly_held_nodes_in(mesh, |s| steps.contains(&s))
    }

    /// The nodes that boundary conditions hold directly in the active steps `steps` takes,
    /// mechanical degrees of freedom only. A point leaves them out of its coupling: an
    /// equation may not depend on a degree of freedom a boundary condition holds.
    pub fn directly_held_nodes_in(
        &self,
        mesh: &FeMesh,
        steps: impl Fn(usize) -> bool,
    ) -> BTreeSet<NodeId> {
        let mut held = BTreeSet::new();
        for (s, step) in
            (self.steps.iter().enumerate()).filter(|(s, step)| step.active && steps(*s))
        {
            for (i, bc) in step.boundary_conditions.iter().enumerate() {
                if !bc.active
                    || !step.kind.supports_boundary(&bc.kind)
                    || bc.region.reference_point().is_some()
                {
                    continue;
                }
                let direct = match &bc.kind {
                    BoundaryKind::Displacement(d) => {
                        self.formulation(mesh, s, i) == Some(Formulation::Nodes)
                            && d.values[..3].iter().any(Option::is_some)
                    }
                    BoundaryKind::Submodel { dofs, .. } => dofs[..3].iter().any(|&d| d),
                    BoundaryKind::Symmetry { .. } => true,
                    BoundaryKind::Temperature(_) => false,
                };
                if direct {
                    held.extend(bc.region.nodes(mesh));
                }
            }
        }
        held
    }

    /// The translations (directions 1 to 3) that boundary conditions hold directly at each
    /// node in any active step, as CalculiX sees them. An equation may not depend on them
    /// and a rigid body may not take their nodes ("dependent side of a MPC and a SPC").
    /// A displacement in a coordinate system of its own and a symmetry plane that is no
    /// coordinate plane hold their directions in local axes, which counts as all three.
    pub fn held_translations(&self, mesh: &FeMesh) -> BTreeMap<NodeId, [bool; 3]> {
        self.held_translations_in(mesh, |_| true)
    }

    /// [`Self::held_translations`] in the active steps `steps` takes.
    pub fn held_translations_in(
        &self,
        mesh: &FeMesh,
        steps: impl Fn(usize) -> bool,
    ) -> BTreeMap<NodeId, [bool; 3]> {
        let space = self.properties.space;
        let mut held: BTreeMap<NodeId, [bool; 3]> = BTreeMap::new();
        for (s, step) in
            (self.steps.iter().enumerate()).filter(|(s, step)| step.active && steps(*s))
        {
            for (i, bc) in step.boundary_conditions.iter().enumerate() {
                if !bc.active
                    || !step.kind.supports_boundary(&bc.kind)
                    || bc.region.reference_point().is_some()
                {
                    continue;
                }
                let directions = match &bc.kind {
                    BoundaryKind::Displacement(d) => {
                        if self.formulation(mesh, s, i) != Some(Formulation::Nodes) {
                            continue;
                        }
                        let given = std::array::from_fn(|k| d.values[k].is_some());
                        if d.system.is_some() && given.iter().any(|&g| g) {
                            [true; 3]
                        } else {
                            given
                        }
                    }
                    BoundaryKind::Submodel { dofs, .. } => [dofs[0], dofs[1], dofs[2]],
                    BoundaryKind::Symmetry { normal, .. } => {
                        let axis = crate::symmetry_plane(mesh, &bc.region, space, *normal)
                            .ok()
                            .and_then(|plane| plane.axis());
                        match axis {
                            Some(axis) => std::array::from_fn(|k| k == axis),
                            None => [true; 3],
                        }
                    }
                    BoundaryKind::Temperature(_) => continue,
                };
                if !directions.iter().any(|&d| d) {
                    continue;
                }
                for node in bc.region.nodes(mesh) {
                    let entry = held.entry(node).or_default();
                    for k in 0..3 {
                        entry[k] |= directions[k];
                    }
                }
            }
        }
        held
    }
}

impl Displacement {
    pub fn new(values: [Option<f64>; 6]) -> Self {
        Self {
            values,
            behavior: Behavior::default(),
            system: None,
            location: None,
            pinball: None,
            tables: Default::default(),
        }
    }

    /// The table of degree of freedom `d` (0..6), if it is held and follows one.
    pub fn table(&self, d: usize) -> Option<&crate::ValueTable> {
        self.values[d].and(self.tables[d].as_ref())
    }

    /// Whether degree of freedom `d` is held at zero throughout the step.
    pub fn still(&self, d: usize) -> bool {
        match self.table(d) {
            Some(table) => !table.moves(),
            None => self.values[d] == Some(0.0),
        }
    }

    /// Whether degree of freedom `d` is held and moves at some time of the step.
    pub fn moves(&self, d: usize) -> bool {
        self.values[d].is_some() && !self.still(d)
    }

    /// A displacement whose selection follows a rigid point.
    pub fn rigid(values: [Option<f64>; 6]) -> Self {
        Self {
            behavior: Behavior::Rigid,
            ..Self::new(values)
        }
    }

    /// How this displacement alone is written; [`FeModel::formulation`] looks at the other
    /// steps as well. `step` and `name` find exposed rotations, which count as turning:
    /// the formulation does not change with a parameter.
    ///
    /// Automatically, the nodes are held directly when they have rotations of their own
    /// (beam and shell nodes), when the selection is a single node or a reference point,
    /// in axisymmetric models, and when all rotations of the model space are held at zero.
    /// A point at the centre is needed for a face or edge of solids that may turn or is
    /// turned, and for a location or pinball region; it is rigid then.
    pub fn formulation(
        &self,
        model: &FeModel,
        mesh: &FeMesh,
        region: &Region,
        step: &str,
        name: &str,
    ) -> Formulation {
        let point = match self.behavior {
            Behavior::EachNode => return Formulation::Nodes,
            Behavior::Rigid => RemoteBehavior::Rigid,
            Behavior::Deformable => RemoteBehavior::Deformable,
            Behavior::Automatic => RemoteBehavior::Rigid,
        };
        if region.reference_point().is_some() {
            return Formulation::Nodes;
        }
        if self.behavior != Behavior::Automatic || self.location.is_some() || self.pinball.is_some()
        {
            return Formulation::Point(point);
        }
        let space = model.properties.space;
        let rotations: Vec<usize> = (space.remote_dofs().iter().copied())
            .filter(|&d| d >= 3)
            .collect();
        if rotations.is_empty() {
            return Formulation::Nodes;
        }
        let exposed = (model.exposed.iter()).any(|e| {
            matches!(&e.path.item, crate::ItemRef::BoundaryCondition { step: s, name: n }
                if s == step && n == name)
                && crate::Field::ROTATION.contains(&e.path.field)
        });
        let still = rotations.iter().all(|&d| self.still(d));
        if still && !exposed {
            return Formulation::Nodes;
        }
        let nodes = region.nodes(mesh);
        if nodes.len() < 2 {
            return Formulation::Nodes;
        }
        let expanded = expanded_nodes(mesh);
        if nodes.iter().all(|n| expanded.contains(n)) {
            return Formulation::Nodes;
        }
        Formulation::Point(point)
    }

    /// The same point and coupling, whatever the values: displacements of several steps
    /// share it.
    pub fn same_point(&self, other: &Self) -> bool {
        self.behavior == other.behavior
            && self.system == other.system
            && self.location == other.location
            && self.pinball == other.pinball
    }

    /// The values of the degrees of freedom of a point the model space allows, the others
    /// free.
    pub fn held(&self, space: ModelSpace) -> [Option<f64>; 6] {
        let mut held = [None; 6];
        for &d in space.remote_dofs() {
            held[d] = self.values[d];
        }
        held
    }

    /// The point, its directions and the nodes following it.
    pub fn resolve(
        &self,
        model: &FeModel,
        mesh: &FeMesh,
        region: &Region,
        excluded: &BTreeSet<NodeId>,
    ) -> Result<RemotePoint, String> {
        let system = (model.coordinate_system_or_global(self.system))
            .ok_or("The coordinate system does not exist")?;
        let position = match self.location {
            Some(local) => system.global(local)?,
            None => (region.centroid(mesh))
                .ok_or_else(|| "The selection contains no nodes.".to_string())?,
        };
        let axes = system.directions_at(position)?;
        let global = (0..3).all(|i| (0..3).all(|k| (axes[i][k] - f64::from(i == k)).abs() < 1e-12));
        let within = |node: NodeId| match (self.pinball, mesh.node(node)) {
            (Some(radius), Some(x)) => distance(x, position) <= radius,
            (None, Some(_)) => true,
            (_, None) => false,
        };
        let nodes: Vec<NodeId> = region
            .nodes(mesh)
            .into_iter()
            .filter(|&n| within(n))
            .collect();
        if nodes.is_empty() {
            return Err(match self.pinball {
                Some(_) => "No node of the selection lies within the pinball region.".into(),
                None => "The selection contains no nodes.".into(),
            });
        }
        // Nodes another boundary condition holds directly stay out of the coupling.
        let nodes: Vec<NodeId> = (nodes.iter().copied())
            .filter(|n| !excluded.contains(n))
            .collect();
        if nodes.is_empty() {
            return Err("Other boundary conditions hold all nodes of the selection.".into());
        }
        let areas = face_areas(mesh, &region.faces(mesh));
        let weights = (nodes.iter())
            .map(|n| areas.get(n).copied().unwrap_or(0.0))
            .collect::<Vec<_>>();
        // Without faces, or nodes beside them, every node counts alike.
        let weights = if weights.iter().all(|&w| w > 0.0) {
            weights
        } else {
            vec![1.0; nodes.len()]
        };
        Ok(RemotePoint {
            position,
            axes: (!global).then_some(axes),
            nodes,
            weights,
        })
    }
}

impl RemotePoint {
    /// The `*EQUATION`s of a deformable behavior, each as its terms with the dependent node
    /// first: a node degree of freedom that no other equation has, so the solver needs no
    /// cascading.
    pub fn equations(&self, mesh: &FeMesh, space: ModelSpace) -> Vec<Vec<(RemoteDof, f64)>> {
        reduce(self.averaging_rows(mesh, space))
    }

    /// Nastran's RBE3: the point moves with the weighted mean translation of the nodes and
    /// turns with their weighted mean rotation about their centre c,
    /// J theta = sum w r x u with J = sum w (|r|^2 I - r r^T), r = x - c, and
    /// u_point = sum w u + theta x (point - c).
    fn averaging_rows(&self, mesh: &FeMesh, space: ModelSpace) -> Vec<Vec<(RemoteDof, f64)>> {
        let total: f64 = self.weights.iter().sum();
        let points: Vec<(NodeId, f64, [f64; 3])> = (self.nodes.iter().zip(&self.weights))
            .filter_map(|(&n, &w)| Some((n, w / total, mesh.node(n)?)))
            .collect();
        let c = points
            .iter()
            .fold([0.0; 3], |s, (_, w, x)| [0, 1, 2].map(|k| s[k] + w * x[k]));
        let d: [f64; 3] = std::array::from_fn(|k| self.position[k] - c[k]);
        let mut rows = Vec::new();
        let translations = if space.is_2d() { 2 } else { 3 };
        let turns: &[usize] = match space {
            ModelSpace::ThreeD => &[0, 1, 2],
            ModelSpace::PlaneStress | ModelSpace::PlaneStrain => &[2],
            ModelSpace::Axisymmetric => &[],
        };
        for k in 0..translations {
            let mut row = vec![(RemoteDof::Translation(k), 1.0)];
            row.extend(points.iter().map(|&(n, w, _)| (RemoteDof::Node(n, k), -w)));
            // - (theta x d)_k
            let (a, b) = ((k + 1) % 3, (k + 2) % 3);
            if turns.contains(&a) {
                row.push((RemoteDof::Rotation(a), -d[b]));
            }
            if turns.contains(&b) {
                row.push((RemoteDof::Rotation(b), d[a]));
            }
            rows.push(row);
        }
        let r: Vec<[f64; 3]> = points
            .iter()
            .map(|(_, _, x)| [0, 1, 2].map(|k| x[k] - c[k]))
            .collect();
        for &k in turns {
            let mut row = Vec::new();
            for &l in turns {
                let j: f64 = (points.iter().zip(&r))
                    .map(|((_, w, _), r)| {
                        let squared = if k == l {
                            r.iter().map(|v| v * v).sum()
                        } else {
                            0.0
                        };
                        w * (squared - r[k] * r[l])
                    })
                    .sum();
                row.push((RemoteDof::Rotation(l), j));
            }
            // - (r x u)_k = -(r_a u_b - r_b u_a)
            let (a, b) = ((k + 1) % 3, (k + 2) % 3);
            for ((n, w, _), r) in points.iter().zip(&r) {
                if b < translations {
                    row.push((RemoteDof::Node(*n, b), -w * r[a]));
                }
                if a < translations {
                    row.push((RemoteDof::Node(*n, a), w * r[b]));
                }
            }
            rows.push(row);
        }
        rows
    }
}

/// Reduced row echelon form of the equations in the node degrees of freedom: each keeps
/// the node degree of freedom with the largest factor as its dependent one, scaled to 1 and
/// removed from all other equations. Equations without node terms are left out.
fn reduce(rows: Vec<Vec<(RemoteDof, f64)>>) -> Vec<Vec<(RemoteDof, f64)>> {
    let mut rows: Vec<BTreeMap<RemoteDof, f64>> = rows
        .into_iter()
        .map(|row| {
            let mut map = BTreeMap::new();
            for (dof, value) in row {
                *map.entry(dof).or_insert(0.0) += value;
            }
            map
        })
        .collect();
    // Rows each node degree of freedom appears in, so elimination only visits those.
    let mut users: BTreeMap<RemoteDof, Vec<usize>> = BTreeMap::new();
    for (i, row) in rows.iter().enumerate() {
        for dof in row.keys() {
            users.entry(*dof).or_default().push(i);
        }
    }
    let largest = (rows.iter().flat_map(|r| r.values())).fold(0.0_f64, |m, v| m.max(v.abs()));
    let tolerance = 1e-10 * largest;
    let mut pivots = Vec::new();
    for i in 0..rows.len() {
        let pivot = (rows[i].iter())
            .filter(|(dof, v)| matches!(dof, RemoteDof::Node(..)) && v.abs() > tolerance)
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
            .map(|(dof, v)| (*dof, *v));
        let Some((pivot, value)) = pivot else {
            continue;
        };
        for v in rows[i].values_mut() {
            *v /= value;
        }
        let source = rows[i].clone();
        for j in users.get(&pivot).cloned().unwrap_or_default() {
            if j == i {
                continue;
            }
            let Some(f) = rows[j].remove(&pivot) else {
                continue;
            };
            for (dof, v) in source.iter().filter(|(dof, _)| **dof != pivot) {
                let entry = rows[j].entry(*dof).or_insert_with(|| {
                    users.entry(*dof).or_default().push(j);
                    0.0
                });
                *entry -= f * v;
            }
        }
        pivots.push((i, pivot));
    }
    (pivots.into_iter())
        .map(|(i, pivot)| {
            let mut terms = vec![(pivot, 1.0)];
            terms.extend(
                (rows[i].iter())
                    .filter(|(dof, v)| **dof != pivot && v.abs() > tolerance)
                    .map(|(dof, v)| (*dof, *v)),
            );
            terms
        })
        .collect()
}

/// The nodes of line (beam, truss), shell and membrane elements, which the solver expands into
/// solid elements. CalculiX 2.23 refuses a `*RIGID BODY` on such a node that is not a knot
/// ("nodes belonging to beam elements must not be subject to a RIGID MPC"), so a rigid
/// coupling takes them with [`rigid_rows`] instead.
pub fn expanded_nodes(mesh: &FeMesh) -> BTreeSet<NodeId> {
    (mesh.elements().iter())
        .filter(|e| e.shape.family() == ElementFamily::Line || e.is_shell())
        .flat_map(|e| e.nodes.iter().copied())
        .collect()
}

/// Rigid coupling of nodes to a point for small rotations, u = u_point + theta x (x - point),
/// one equation per node and direction with the node's degree of freedom first. The same
/// as a `*RIGID BODY` in a linear analysis, where CalculiX linearises its rigid MPCs too;
/// rotations of beam and shell nodes are not coupled, as by a `*RIGID BODY` either.
pub fn rigid_rows(
    mesh: &FeMesh,
    nodes: &[NodeId],
    position: [f64; 3],
) -> Vec<Vec<(RemoteDof, f64)>> {
    let mut rows = Vec::new();
    for &node in nodes {
        let Some(x) = mesh.node(node) else {
            continue;
        };
        let r: [f64; 3] = std::array::from_fn(|k| x[k] - position[k]);
        for k in 0..3 {
            // - (theta x r)_k = -(theta_a r_b - theta_b r_a)
            let (a, b) = ((k + 1) % 3, (k + 2) % 3);
            let row = [
                (RemoteDof::Node(node, k), 1.0),
                (RemoteDof::Translation(k), -1.0),
                (RemoteDof::Rotation(a), -r[b]),
                (RemoteDof::Rotation(b), r[a]),
            ];
            rows.push(row.into_iter().filter(|(_, v)| *v != 0.0).collect());
        }
    }
    rows
}

/// Share of the face areas (edge lengths of 2D faces) of each node of the faces.
fn face_areas(mesh: &FeMesh, faces: &[(ElementId, u8)]) -> BTreeMap<NodeId, f64> {
    let mut areas = BTreeMap::new();
    for &(element, face) in faces {
        let Some(element) = mesh.element(element) else {
            continue;
        };
        let Some(topology) = element.faces().get(usize::from(face).saturating_sub(1)) else {
            continue;
        };
        let corners: Vec<[f64; 3]> = (topology.corners.iter())
            .filter_map(|&local| mesh.node(*element.nodes.get(local)?))
            .collect();
        let area = match corners[..] {
            [a, b] => distance(a, b),
            [a, ..] if corners.len() >= 3 => (corners[1..].windows(2))
                .map(|pair| {
                    let u: [f64; 3] = std::array::from_fn(|k| pair[0][k] - a[k]);
                    let v: [f64; 3] = std::array::from_fn(|k| pair[1][k] - a[k]);
                    let c = [
                        u[1] * v[2] - u[2] * v[1],
                        u[2] * v[0] - u[0] * v[2],
                        u[0] * v[1] - u[1] * v[0],
                    ];
                    0.5 * c.iter().map(|x| x * x).sum::<f64>().sqrt()
                })
                .sum(),
            _ => 0.0,
        };
        let quadratic = element.shape.is_quadratic();
        let nodes: Vec<NodeId> = (topology.corners.iter())
            .chain(if quadratic { topology.mids } else { &[] })
            .filter_map(|&local| element.nodes.get(local).copied())
            .collect();
        for node in &nodes {
            *areas.entry(*node).or_insert(0.0) += area / nodes.len() as f64;
        }
    }
    areas
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|k| (a[k] - b[k]).powi(2)).sum::<f64>().sqrt()
}
