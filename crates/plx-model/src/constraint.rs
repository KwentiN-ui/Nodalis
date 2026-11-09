//! The spring, joint and support constraints, with springs and joints between ground,
//! selections and points. Ties are in [`crate::contact`].
//!
//! Like everything in the model they store what the user defined. The spring and gap elements,
//! extra nodes and equations the solver needs for them are generated when the input file is
//! written.

use serde::{Deserialize, Serialize};

use crate::{Region, RemoteBehavior};

/// Nodes tied to each other, nodalis's node tie: the ends of beams or trusses meeting at a
/// point, which have nodes of their own since parts share no nodes. Every node of the region
/// moves with the first. Node ties are listed with the contact pairs; a contact detection
/// finds them at the line ends.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NodeTie {
    pub name: String,
    #[serde(default = "crate::active")]
    pub active: bool,
    pub region: Region,
    /// Ties the rotations too, a rigid joint between beams; a hinge otherwise. Nodes
    /// without rotations (trusses, solids) are tied in their translations only either way.
    pub rotations: bool,
    /// The contact detection that found the tie at line ends; `None` for one made by hand.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub detected_by: Option<crate::Id>,
    /// Changed by the user after its detection: detecting again keeps it as it is.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub edited: bool,
}

impl NodeTie {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            region: Region::Nodes(Vec::new()),
            rotations: true,
            detected_by: None,
            edited: false,
        }
    }
}

/// Nodes coupled to a reference point. Rigid, they move as one rigid body with the point
/// (`*RIGID BODY`); deformable, the point follows their weighted mean motion (Nastran's
/// RBE3, written as equations) and the selection keeps its own stiffness, which suits a
/// point that only tracks where a face goes. Boundary conditions and loads on the
/// reference point drive the selection: its translations and rotations, forces and moments.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigidBody {
    pub name: String,
    #[serde(default = "crate::active")]
    pub active: bool,
    /// Hidden constraints have no symbol in the 3D view unless selected.
    #[serde(default = "crate::active")]
    pub visible: bool,
    pub region: Region,
    /// The reference point ([`crate::ReferencePoint`]) the body is driven by.
    pub reference_point: crate::Id,
    /// Whether the selection keeps its shape or the point follows its mean motion.
    #[serde(default)]
    pub behavior: RemoteBehavior,
}

impl RigidBody {
    pub fn new(name: impl Into<String>, reference_point: crate::Id) -> Self {
        Self {
            name: name.into(),
            active: true,
            visible: true,
            region: Region::Faces(Vec::new()),
            reference_point,
            behavior: RemoteBehavior::Rigid,
        }
    }
}

/// One end of a spring or joint: fixed ground, a region of the mesh or a point.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SpringEnd {
    /// Fixed ground.
    World,
    /// Nodes, edges or faces of the mesh, or a reference point ([`Region::ReferencePoint`])
    /// or the point a boundary condition or remote force creates ([`Region::RemotePoint`]).
    Region(Region),
}

impl SpringEnd {
    pub fn region(&self) -> Option<&Region> {
        match self {
            SpringEnd::World => None,
            SpringEnd::Region(region) => Some(region),
        }
    }

    pub fn region_mut(&mut self) -> Option<&mut Region> {
        match self {
            SpringEnd::World => None,
            SpringEnd::Region(region) => Some(region),
        }
    }

    /// A reference point or the point of a boundary condition or remote force, which has
    /// rotations.
    pub fn point(&self) -> Option<&Region> {
        self.region().filter(|r| is_point(r))
    }

    /// Nodes, edges or faces of the mesh.
    pub fn selection(&self) -> Option<&Region> {
        self.region().filter(|r| !is_point(r))
    }
}

/// A reference point or the point of a boundary condition or remote force.
pub fn is_point(region: &Region) -> bool {
    matches!(region, Region::ReferencePoint(_) | Region::RemotePoint(_))
}

/// What a spring connects, from its two ends, which the user may give in either order.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SpringLayout<'a> {
    /// Springs from the nodes of a selection to ground (`SPRING1`).
    SelectionToWorld { selection: &'a Region },
    /// Springs from the nodes of a selection, the first end, to the closest points of the
    /// faces of the other selection (`SPRING2`).
    SelectionToSurface {
        selection: &'a Region,
        surface: &'a Region,
    },
    /// Springs from the nodes of a selection to a point (`SPRING2` to its reference node).
    SelectionToPoint {
        selection: &'a Region,
        point: &'a Region,
    },
    /// Translational and rotational springs from a point to ground.
    PointToWorld { point: &'a Region },
    /// Translational and rotational springs between two points.
    PointToPoint {
        first: &'a Region,
        second: &'a Region,
    },
}

/// Springs between two ends, each of them ground ("World"), a selection of the mesh or a
/// point: a reference point or the point a boundary condition or remote force creates.
/// They act in the global directions. Translational springs connect the nodes of a
/// selection, like a force the stiffness is that of the whole selection and spread over
/// its nodes, on faces by area, on edges by length and on points in equal shares. Between
/// points and ground the springs hold the rotations too.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Spring {
    pub name: String,
    #[serde(default = "crate::active")]
    pub active: bool,
    /// Hidden constraints have no symbol in the 3D view unless selected.
    #[serde(default = "crate::active")]
    pub visible: bool,
    /// The two ends. With a selection on both, the first gets the springs and the second
    /// is the surface they end on.
    pub ends: [SpringEnd; 2],
    /// K1..K3 of the whole selection in force per length, or per area of its faces (force
    /// per volume) with `per_area`, or of every node with `each_node`; a zero adds no spring
    /// in that direction.
    pub stiffness: [f64; 3],
    /// Rotational stiffness about the global axes in moment per radian, for springs between
    /// points and ground only ([`Spring::takes_rotations`]).
    #[serde(default)]
    pub rotational: [f64; 3],
    pub per_area: bool,
    /// The whole stiffness acts at every node instead of being spread over the nodes.
    pub each_node: bool,
    /// The springs also resist the motion within each increment of a static step with
    /// this factor times their stiffness, a force that starts from zero in every increment
    /// (`*Spring, Stabilize=`): the weak springs of the parts held only by contact
    /// ([`FeModel::weak_springs`](crate::FeModel::weak_springs)), which are not saved.
    #[serde(skip)]
    pub stabilize: f64,
}

impl Spring {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            visible: true,
            ends: [
                SpringEnd::Region(Region::Nodes(Vec::new())),
                SpringEnd::World,
            ],
            stiffness: [0.0; 3],
            rotational: [0.0; 3],
            per_area: false,
            each_node: false,
            stabilize: 0.0,
        }
    }

    /// Springs from the nodes of `region` to ground.
    pub fn to_world(name: impl Into<String>, region: Region, stiffness: [f64; 3]) -> Self {
        Self {
            ends: [SpringEnd::Region(region), SpringEnd::World],
            stiffness,
            ..Self::new(name)
        }
    }

    /// What the spring connects; `None` with ground on both ends.
    pub fn layout(&self) -> Option<SpringLayout<'_>> {
        let [a, b] = &self.ends;
        Some(match (a, b) {
            (SpringEnd::World, SpringEnd::World) => return None,
            (SpringEnd::World, SpringEnd::Region(r)) | (SpringEnd::Region(r), SpringEnd::World) => {
                if is_point(r) {
                    SpringLayout::PointToWorld { point: r }
                } else {
                    SpringLayout::SelectionToWorld { selection: r }
                }
            }
            (SpringEnd::Region(a), SpringEnd::Region(b)) => match (is_point(a), is_point(b)) {
                (false, false) => SpringLayout::SelectionToSurface {
                    selection: a,
                    surface: b,
                },
                (false, true) => SpringLayout::SelectionToPoint {
                    selection: a,
                    point: b,
                },
                (true, false) => SpringLayout::SelectionToPoint {
                    selection: b,
                    point: a,
                },
                (true, true) => SpringLayout::PointToPoint {
                    first: a,
                    second: b,
                },
            },
        })
    }

    /// The selection whose nodes get the springs, if an end is one.
    pub fn selection(&self) -> Option<&Region> {
        match self.layout()? {
            SpringLayout::SelectionToWorld { selection }
            | SpringLayout::SelectionToSurface { selection, .. }
            | SpringLayout::SelectionToPoint { selection, .. } => Some(selection),
            SpringLayout::PointToWorld { .. } | SpringLayout::PointToPoint { .. } => None,
        }
    }

    /// The springs connect points and ground only, whose rotations they hold too; nodes of
    /// a selection have no rotations to connect.
    pub fn takes_rotations(&self) -> bool {
        matches!(
            self.layout(),
            Some(SpringLayout::PointToWorld { .. } | SpringLayout::PointToPoint { .. })
        )
    }

    /// The stiffness is per area of the faces of the selection.
    pub fn is_per_area(&self) -> bool {
        self.per_area && !self.each_node && self.selection().is_some()
    }

    /// Target surface and selection of springs between two selections, master before slave.
    pub fn master_slave(&self) -> Option<[&Region; 2]> {
        match self.layout()? {
            SpringLayout::SelectionToSurface { selection, surface } => Some([surface, selection]),
            _ => None,
        }
    }

    /// The rotational stiffness that acts: none where the spring takes no rotations.
    pub fn acting_rotational(&self) -> [f64; 3] {
        if self.takes_rotations() {
            self.rotational
        } else {
            [0.0; 3]
        }
    }
}

/// The kind of a joint: which relative motions of its ends it leaves free, in its frame,
/// whose z axis is the axis of a hinge, cylinder or slider and the normal of a planar
/// joint.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum JointKind {
    /// The ends move as one.
    Fixed,
    /// A hinge: the ends turn against each other about z.
    #[default]
    Revolute,
    /// Turning about and sliding along z.
    Cylindrical,
    /// Sliding along z without turning.
    Slider,
    /// A ball joint: turning freely, no sliding.
    Spherical,
    /// Sliding in the x-y plane and turning about z.
    Planar,
    /// Free and held degrees of freedom chosen one by one.
    General,
}

impl JointKind {
    pub const ALL: [JointKind; 7] = [
        Self::Fixed,
        Self::Revolute,
        Self::Cylindrical,
        Self::Slider,
        Self::Spherical,
        Self::Planar,
        Self::General,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Fixed => "Fixed",
            Self::Revolute => "Revolute",
            Self::Cylindrical => "Cylindrical",
            Self::Slider => "Slider",
            Self::Spherical => "Spherical",
            Self::Planar => "Planar",
            Self::General => "General",
        }
    }

    /// The free relative degrees of freedom (Ux, Uy, Uz, URx, URy, URz in the joint frame);
    /// `None` for a general joint, which keeps its own.
    pub fn free(self) -> Option<[bool; 6]> {
        Some(match self {
            Self::Fixed => [false; 6],
            Self::Revolute => [false, false, false, false, false, true],
            Self::Cylindrical => [false, false, true, false, false, true],
            Self::Slider => [false, false, true, false, false, false],
            Self::Spherical => [false, false, false, true, true, true],
            Self::Planar => [true, true, false, false, false, true],
            Self::General => return None,
        })
    }
}

/// The axis of a joint: the axis a revolute, cylindrical or slider joint turns about or
/// slides along, the normal of a planar joint; the z axis of the joint frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum JointAxis {
    /// The z axis of the coordinate system.
    #[default]
    CoordinateSystem,
    /// The mean normal of the faces of End 1 (of End 2 when End 1 has none), e.g. the face
    /// where a hub sits on its shaft.
    FaceNormal,
    /// The axis of the cylindrical faces of End 1 (of End 2 when End 1 has none), e.g. a
    /// bore or a pin.
    CylinderAxis,
    /// A direction in the coordinate system.
    Direction([f64; 3]),
}

impl JointAxis {
    pub const CHOICES: [JointAxis; 4] = [
        JointAxis::FaceNormal,
        JointAxis::CylinderAxis,
        JointAxis::CoordinateSystem,
        JointAxis::Direction([0.0, 0.0, 1.0]),
    ];

    pub fn label(self) -> &'static str {
        match self {
            JointAxis::CoordinateSystem => "Z of the coordinate system",
            JointAxis::FaceNormal => "Face normal",
            JointAxis::CylinderAxis => "Cylinder axis",
            JointAxis::Direction(_) => "Direction",
        }
    }

    /// The same choice, whatever the direction.
    pub fn same_kind(self, other: JointAxis) -> bool {
        std::mem::discriminant(&self) == std::mem::discriminant(&other)
    }
}

/// A joint between two ends, each ground ("World"), a selection of the mesh or a point (a
/// reference point or the point a boundary condition or remote force creates): like a
/// Displacement/Rotation that holds the motion of the second end relative to the first
/// instead of to ground. The joint frame sits at `location` with the directions of
/// `system` and turns with the first end. A selection follows a point of its own at the
/// joint, rigidly or on average (`behavior`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Joint {
    pub name: String,
    #[serde(default = "crate::active")]
    pub active: bool,
    /// Hidden constraints have no symbol in the 3D view unless selected.
    #[serde(default = "crate::active")]
    pub visible: bool,
    pub ends: [SpringEnd; 2],
    pub kind: JointKind,
    /// The free degrees of freedom of a general joint, see [`JointKind::free`].
    pub free: [bool; 6],
    /// How a selection end follows its point.
    pub behavior: crate::RemoteBehavior,
    /// Coordinate system of the location and the joint directions; `None` is the global
    /// one.
    #[serde(default)]
    pub system: Option<crate::Id>,
    /// Coordinates of the joint in `system`; `None` is the centre of the first end (of the
    /// second with ground as the first).
    pub location: Option<[f64; 3]>,
    /// The joint axis (3D only; a joint in a plane turns about the normal of the plane).
    #[serde(default)]
    pub axis: JointAxis,
}

impl Joint {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            visible: true,
            ends: [
                SpringEnd::Region(Region::Faces(Vec::new())),
                SpringEnd::Region(Region::Faces(Vec::new())),
            ],
            kind: JointKind::Revolute,
            free: [false; 6],
            behavior: crate::RemoteBehavior::Rigid,
            system: None,
            location: None,
            axis: JointAxis::FaceNormal,
        }
    }

    /// The faces the axis is taken from: of the first end with faces.
    pub fn axis_faces(&self, mesh: &plx_mesh::FeMesh) -> Vec<(plx_mesh::ElementId, u8)> {
        (self.ends.iter())
            .filter_map(|end| end.selection())
            .map(|region| region.faces(mesh))
            .find(|faces| !faces.is_empty())
            .unwrap_or_default()
    }

    /// The free relative degrees of freedom, from the kind or chosen one by one.
    pub fn free_dofs(&self) -> [bool; 6] {
        self.kind.free().unwrap_or(self.free)
    }

    /// The end whose centre is the default location: the first, or the second with ground
    /// as the first.
    pub fn locating_end(&self) -> Option<&Region> {
        self.ends[0].region().or(self.ends[1].region())
    }
}

impl crate::FeModel {
    /// The global position of a point: a reference point, or the point of a
    /// Displacement/Rotation condition or remote force.
    pub fn point_position(&self, mesh: &plx_mesh::FeMesh, point: &Region) -> Option<[f64; 3]> {
        match point {
            Region::ReferencePoint(id) => self.reference_point(*id).map(|p| p.position),
            Region::RemotePoint(id) => {
                if let Some((_, bc, remote)) = self.remote_displacement(mesh, *id) {
                    let excluded = self.point_excluded_nodes(mesh, &bc.region, remote);
                    return (remote.resolve(self, mesh, &bc.region, &excluded).ok())
                        .map(|p| p.position);
                }
                let (region, force) = self.remote_force_of(*id)?;
                (self.remote_force_nodes(mesh, region, force).ok()).map(|(p, _)| p.position)
            }
            _ => None,
        }
    }

    /// Where a joint is, globally: its location, or the centre of its first end (of the
    /// second with ground as the first).
    pub fn joint_origin(&self, mesh: &plx_mesh::FeMesh, joint: &Joint) -> Result<[f64; 3], String> {
        let system = (self.coordinate_system_or_global(joint.system))
            .ok_or("The coordinate system does not exist")?;
        if let Some(local) = joint.location {
            return system.global(local);
        }
        let end = joint.locating_end().ok_or("Both ends are World")?;
        if is_point(end) {
            self.point_position(mesh, end)
                .ok_or_else(|| format!("{} has no position", end.describe(self)))
        } else {
            end.centroid(mesh)
                .ok_or_else(|| "The selection contains no nodes.".to_string())
        }
    }

    /// The joint frame: its origin and its x, y and z directions, z along the joint axis
    /// and x as close to the x direction of the coordinate system as the axis allows.
    pub fn joint_frame(
        &self,
        mesh: &plx_mesh::FeMesh,
        joint: &Joint,
    ) -> Result<([f64; 3], [[f64; 3]; 3]), String> {
        let origin = self.joint_origin(mesh, joint)?;
        let system = (self.coordinate_system_or_global(joint.system))
            .ok_or("The coordinate system does not exist")?;
        let directions = system.directions_at(origin)?;
        if self.properties.space.is_2d() {
            return Ok((origin, directions));
        }
        let axis = match joint.axis {
            JointAxis::CoordinateSystem => return Ok((origin, directions)),
            JointAxis::Direction(d) => {
                [0, 1, 2].map(|k| (0..3).map(|i| d[i] * directions[i][k]).sum::<f64>())
            }
            JointAxis::FaceNormal => match joint.axis_faces(mesh) {
                faces if !faces.is_empty() => face_normal(mesh, &faces)?,
                // Nodes picked on a mesh without faces: the normal of the plane they lie in.
                _ => plane_normal(mesh, joint)?,
            },
            JointAxis::CylinderAxis => cylinder_axis(mesh, &joint.axis_faces(mesh))?,
        };
        Ok((origin, frame_about(axis, directions)?))
    }
}

/// Gap elements on a surface that only take pressure (`GAPUNI`), a
/// compression only support.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CompressionOnly {
    pub name: String,
    #[serde(default = "crate::active")]
    pub active: bool,
    /// Hidden constraints have no symbol in the 3D view unless selected.
    #[serde(default = "crate::active")]
    pub visible: bool,
    pub region: Region,
    /// Distance the surface can move away before the support takes load.
    pub clearance: f64,
    /// Stiffness of the closed gaps of the whole surface, or of each gap with `each_node`;
    /// `None` is CalculiX's default for each gap, practically rigid.
    pub spring_stiffness: Option<f64>,
    /// Tensile force at negative infinity of the whole surface, or of each gap with
    /// `each_node`; `None` is CalculiX's default for each gap.
    pub tensile_force: Option<f64>,
    /// Stiffness and tensile force act at every node instead of being spread over the
    /// nodes by area.
    #[serde(default)]
    pub each_node: bool,
    /// The support pushes the surface outwards, from inside the material, instead of
    /// pushing onto it from outside.
    #[serde(default)]
    pub flipped: bool,
    /// Distance by which the ground node of each gap lies outside the surface.
    pub offset: f64,
    /// Solves the support nonlinearly; otherwise it is linearized in a linear step.
    pub nonlinear: bool,
}

impl CompressionOnly {
    /// Default spring stiffness of the gap elements.
    pub const DEFAULT_STIFFNESS: f64 = 1e12;
    /// Default tensile force of the gap elements at large clearances.
    pub const DEFAULT_TENSILE_FORCE: f64 = 1e-3;

    /// The direction the support pushes a face with the given outward normal in.
    pub fn push_direction(&self, outward: [f64; 3]) -> [f64; 3] {
        if self.flipped {
            outward
        } else {
            outward.map(|v| -v)
        }
    }
}

/// The faces as `(area, unit normal)` per triangle, the normals outward of solids (the
/// corners of a CalculiX solid face turn about the inward normal).
fn face_triangles(
    mesh: &plx_mesh::FeMesh,
    faces: &[(plx_mesh::ElementId, u8)],
) -> Vec<(f64, [f64; 3])> {
    let mut out = Vec::new();
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
        if corners.len() < 3 {
            continue;
        }
        let a = corners[0];
        for pair in corners[1..].windows(2) {
            let n = cross3(sub3(pair[1], a), sub3(pair[0], a));
            let length = norm3(n);
            if length > 0.0 {
                out.push((0.5 * length, n.map(|c| c / length)));
            }
        }
    }
    out
}

/// The mean normal of faces that are (nearly) flat.
fn face_normal(
    mesh: &plx_mesh::FeMesh,
    faces: &[(plx_mesh::ElementId, u8)],
) -> Result<[f64; 3], String> {
    let triangles = face_triangles(mesh, faces);
    if triangles.is_empty() {
        return Err("The face normal needs faces at End 1 or End 2.".into());
    }
    let area: f64 = triangles.iter().map(|(a, _)| a).sum();
    let sum = (triangles.iter()).fold([0.0; 3], |s, (a, n)| [0, 1, 2].map(|k| s[k] + a * n[k]));
    let length = norm3(sum);
    if length < 0.5 * area {
        return Err("The faces have no common normal: choose flat faces.".into());
    }
    Ok(sum.map(|c| c / length))
}

/// The normal of the plane the nodes of the first selection end lie in.
fn plane_normal(mesh: &plx_mesh::FeMesh, joint: &Joint) -> Result<[f64; 3], String> {
    let points: Vec<[f64; 3]> = (joint.ends.iter())
        .filter_map(|end| end.selection())
        .map(|region| region.nodes(mesh))
        .find(|nodes| nodes.len() >= 3)
        .unwrap_or_default()
        .into_iter()
        .filter_map(|id| mesh.node(id))
        .collect();
    if points.len() < 3 {
        return Err("The face normal needs faces at End 1 or End 2.".into());
    }
    let count = points.len() as f64;
    let centre = (points.iter()).fold([0.0; 3], |s, p| [0, 1, 2].map(|k| s[k] + p[k] / count));
    let mut scatter = [[0.0; 6]; 6];
    for p in &points {
        let d = sub3(*p, centre);
        for i in 0..3 {
            for j in 0..3 {
                scatter[i][j] += d[i] * d[j];
            }
        }
    }
    let (values, vectors) = crate::checks::eigen(&scatter, 3);
    let mut order = [0, 1, 2];
    order.sort_by(|&i, &j| values[i].total_cmp(&values[j]));
    if values[order[0]] > 1e-6 * values[order[2]] || values[order[1]] < 1e-6 * values[order[2]] {
        return Err("The nodes do not lie in a plane: choose flat faces.".into());
    }
    let v = vectors[order[0]];
    Ok(along_positive([v[0], v[1], v[2]]))
}

/// The direction pointing along the positive global axis it is closest to, normalized.
fn along_positive(mut axis: [f64; 3]) -> [f64; 3] {
    let largest = (0..3)
        .max_by(|&i, &j| axis[i].abs().total_cmp(&axis[j].abs()))
        .unwrap_or(2);
    if axis[largest] < 0.0 {
        axis = axis.map(|c| -c);
    }
    let length = norm3(axis);
    axis.map(|c| c / length)
}

/// The axis of cylindrical (or conical) faces: the direction all their normals are
/// perpendicular to.
fn cylinder_axis(
    mesh: &plx_mesh::FeMesh,
    faces: &[(plx_mesh::ElementId, u8)],
) -> Result<[f64; 3], String> {
    let triangles = face_triangles(mesh, faces);
    if triangles.is_empty() {
        return Err("The cylinder axis needs faces at End 1 or End 2.".into());
    }
    let mut scatter = [[0.0; 6]; 6];
    for (a, n) in &triangles {
        for i in 0..3 {
            for j in 0..3 {
                scatter[i][j] += a * n[i] * n[j];
            }
        }
    }
    let (values, vectors) = crate::checks::eigen(&scatter, 3);
    let order = {
        let mut order = [0, 1, 2];
        order.sort_by(|&i, &j| values[i].total_cmp(&values[j]));
        order
    };
    // Normals all around an axis: one direction without any normal component, two with.
    if values[order[0]] > 0.05 * values[order[2]] || values[order[1]] < 0.05 * values[order[2]] {
        return Err("The faces are not cylindrical: choose the faces of a bore or a pin.".into());
    }
    let v = vectors[order[0]];
    Ok(along_positive([v[0], v[1], v[2]]))
}

/// An orthonormal frame with z along `axis` and x as close to the first of `directions` as
/// it can be (the second when the axis is along the first).
fn frame_about(axis: [f64; 3], directions: [[f64; 3]; 3]) -> Result<[[f64; 3]; 3], String> {
    let length = norm3(axis);
    if length < 1e-12 {
        return Err("The joint axis has no direction.".into());
    }
    let z = axis.map(|c| c / length);
    let project = |d: [f64; 3]| {
        let along = dot3(d, z);
        let x = [0, 1, 2].map(|k| d[k] - along * z[k]);
        let l = norm3(x);
        (l > 1e-6).then(|| x.map(|c| c / l))
    };
    let x = (project(directions[0]))
        .or_else(|| project(directions[1]))
        .ok_or("The joint axis has no direction.")?;
    Ok([x, cross3(z, x), z])
}

fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm3(a: [f64; 3]) -> f64 {
    dot3(a, a).sqrt()
}

#[cfg(test)]
mod joint_axis_tests {
    use super::*;
    use plx_mesh::FeMesh;

    /// The faces of `mesh` whose corners all satisfy `on`.
    fn faces_where(mesh: &FeMesh, on: impl Fn([f64; 3]) -> bool) -> Region {
        let mut faces = Vec::new();
        for element in mesh.elements() {
            for (f, topology) in element.faces().iter().enumerate() {
                let all = (topology.corners.iter())
                    .all(|&l| mesh.node(element.nodes[l]).is_some_and(&on));
                if all {
                    faces.push((element.id, f as u8 + 1));
                }
            }
        }
        Region::Faces(faces)
    }

    fn frame(mesh: &FeMesh, region: Region, axis: JointAxis) -> Result<[[f64; 3]; 3], String> {
        let joint = Joint {
            ends: [SpringEnd::Region(region), SpringEnd::World],
            axis,
            ..Joint::new("Joint-1")
        };
        crate::FeModel::default()
            .joint_frame(mesh, &joint)
            .map(|(_, axes)| axes)
    }

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        (0..3).all(|k| (a[k] - b[k]).abs() < 1e-9)
    }

    #[test]
    fn the_axis_is_the_normal_of_a_flat_face() {
        let mesh = plx_mesh::structured::box_mesh([0.0; 3], [1.0; 3], [2; 3], "C3D8", false, "P");
        let side = faces_where(&mesh, |p| (p[0] - 1.0).abs() < 1e-12);
        let axes = frame(&mesh, side, JointAxis::FaceNormal).unwrap();
        assert!(close(axes[2], [1.0, 0.0, 0.0]), "{axes:?}");
        let other = faces_where(&mesh, |p| p[0].abs() < 1e-12);
        let flipped = frame(&mesh, other, JointAxis::FaceNormal).unwrap();
        assert!(close(flipped[2], [-1.0, 0.0, 0.0]), "{flipped:?}");
        // x as close to global x as the axis allows: global y here; a right-handed frame.
        assert!(close(axes[0], [0.0, 1.0, 0.0]), "{axes:?}");
        assert!(close(axes[1], [0.0, 0.0, 1.0]), "{axes:?}");
        // Faces all around have no common normal.
        let all = faces_where(&mesh, |p| {
            p.iter().any(|c| c.abs() < 1e-12 || (c - 1.0).abs() < 1e-12)
        });
        assert!(frame(&mesh, all, JointAxis::FaceNormal).is_err());
    }

    #[test]
    fn the_axis_of_a_tube_is_found_from_its_outer_faces() {
        let a = [1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0];
        let b = [2.0 / 3.0, 1.0 / 3.0, -2.0 / 3.0];
        let c = cross3(a, b);
        // A box along x, r from 1 to 1.5, t around: bent into a tube about `a`.
        let mut mesh = plx_mesh::structured::box_mesh(
            [0.0, 1.0, 0.0],
            [2.0, 0.5, std::f64::consts::TAU],
            [2, 1, 24],
            "C3D8",
            false,
            "P",
        );
        for id in mesh.node_ids().to_vec() {
            let [x, r, t] = mesh.node(id).unwrap();
            let p = [0, 1, 2].map(|k| x * a[k] + r * t.cos() * b[k] + r * t.sin() * c[k]);
            mesh.set_node(id, p);
        }
        let radius = |p: [f64; 3]| {
            let along = dot3(p, a);
            norm3([0, 1, 2].map(|k| p[k] - along * a[k]))
        };
        let outer = faces_where(&mesh, |p| (radius(p) - 1.5).abs() < 1e-9);
        let axes = frame(&mesh, outer.clone(), JointAxis::CylinderAxis).unwrap();
        assert!((0..3).all(|k| (axes[2][k] - a[k]).abs() < 1e-6), "{axes:?}");
        assert!(frame(&mesh, outer, JointAxis::FaceNormal).is_err());
        // A flat end is no cylinder.
        let end = faces_where(&mesh, |p| dot3(p, a).abs() < 1e-9);
        assert!(frame(&mesh, end.clone(), JointAxis::CylinderAxis).is_err());
        let axes = frame(&mesh, end, JointAxis::FaceNormal).unwrap();
        assert!((dot3(axes[2], a).abs() - 1.0).abs() < 1e-9, "{axes:?}");
    }

    #[test]
    fn picked_nodes_give_the_normal_of_their_plane() {
        let mesh = plx_mesh::structured::box_mesh([0.0; 3], [1.0; 3], [2; 3], "C3D8", false, "P");
        let nodes = (mesh.node_ids().iter().copied())
            .filter(|&id| mesh.node(id).is_some_and(|p| (p[2] - 1.0).abs() < 1e-12))
            .collect();
        let axes = frame(&mesh, Region::Nodes(nodes), JointAxis::FaceNormal).unwrap();
        assert!(close(axes[2], [0.0, 0.0, 1.0]), "{axes:?}");
        let all = Region::Nodes(mesh.node_ids().to_vec());
        assert!(frame(&mesh, all, JointAxis::FaceNormal).is_err());
    }

    #[test]
    fn a_direction_and_the_coordinate_system_give_the_axis_directly() {
        let mesh = plx_mesh::structured::box_mesh([0.0; 3], [1.0; 3], [1; 3], "C3D8", false, "P");
        let side = faces_where(&mesh, |p| p[0].abs() < 1e-12);
        let axes = frame(&mesh, side.clone(), JointAxis::Direction([0.0, 2.0, 0.0])).unwrap();
        assert!(close(axes[2], [0.0, 1.0, 0.0]), "{axes:?}");
        let axes = frame(&mesh, side, JointAxis::CoordinateSystem).unwrap();
        assert!(close(axes[2], [0.0, 0.0, 1.0]), "{axes:?}");
    }
}
