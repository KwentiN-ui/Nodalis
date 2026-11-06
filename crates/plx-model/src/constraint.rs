//! The spring, joint and support constraints, with springs and joints between ground,
//! selections and points. Ties are in [`crate::contact`].
//!
//! Like everything in the model they store what the user defined. The spring and gap elements,
//! extra nodes and equations the solver needs for them are generated when the input file is
//! written.

use serde::{Deserialize, Serialize};

use crate::Region;

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
    pub detected_by: Option<String>,
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

/// Nodes that move as one rigid body with a reference point (`*RIGID BODY`).
/// Boundary conditions and loads on the reference point drive the body:
/// its translations and rotations, forces and moments.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigidBody {
    pub name: String,
    #[serde(default = "crate::active")]
    pub active: bool,
    /// Hidden constraints have no symbol in the 3D view unless selected.
    #[serde(default = "crate::active")]
    pub visible: bool,
    pub region: Region,
    /// Name of the reference point ([`crate::ReferencePoint`]) the body is driven by.
    pub reference_point: String,
}

impl RigidBody {
    pub fn new(name: impl Into<String>, reference_point: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            visible: true,
            region: Region::Faces(Vec::new()),
            reference_point: reference_point.into(),
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
    /// Coordinate system of the location and the joint directions, by name.
    pub system: String,
    /// Coordinates of the joint in `system`; `None` is the centre of the first end (of the
    /// second with ground as the first).
    pub location: Option<[f64; 3]>,
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
            system: crate::features::GLOBAL.to_string(),
            location: None,
        }
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
            Region::ReferencePoint(name) => self.reference_point(name).map(|p| p.position),
            Region::RemotePoint(name) => {
                if let Some((_, bc, remote)) = self.remote_displacement(mesh, name) {
                    let excluded = self.point_excluded_nodes(mesh, &bc.region, remote);
                    return (remote.resolve(self, mesh, &bc.region, &excluded).ok())
                        .map(|p| p.position);
                }
                let (region, force) = self.remote_force_named(name)?;
                (self.remote_force_nodes(mesh, region, force).ok()).map(|(p, _)| p.position)
            }
            _ => None,
        }
    }

    /// Where a joint is, globally: its location, or the centre of its first end (of the
    /// second with ground as the first).
    pub fn joint_origin(&self, mesh: &plx_mesh::FeMesh, joint: &Joint) -> Result<[f64; 3], String> {
        let system = (self.coordinate_system_or_global(&joint.system))
            .ok_or_else(|| format!("Coordinate System {} does not exist", joint.system))?;
        if let Some(local) = joint.location {
            return system.global(local);
        }
        let end = joint.locating_end().ok_or("Both ends are World")?;
        if is_point(end) {
            self.point_position(mesh, end)
                .ok_or_else(|| format!("{} has no position", end.describe()))
        } else {
            end.centroid(mesh)
                .ok_or_else(|| "The selection contains no nodes.".to_string())
        }
    }

    /// The joint frame: its origin and the x, y and z directions of its coordinate system
    /// there.
    pub fn joint_frame(
        &self,
        mesh: &plx_mesh::FeMesh,
        joint: &Joint,
    ) -> Result<([f64; 3], [[f64; 3]; 3]), String> {
        let origin = self.joint_origin(mesh, joint)?;
        let system = (self.coordinate_system_or_global(&joint.system))
            .ok_or_else(|| format!("Coordinate System {} does not exist", joint.system))?;
        Ok((origin, system.directions_at(origin)?))
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
