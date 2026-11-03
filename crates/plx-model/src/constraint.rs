//! The spring and support constraints, with springs also between two surfaces. Ties
//! are in [`crate::contact`].
//!
//! Like everything in the model they store what the user defined. The spring and gap elements,
//! extra nodes and equations CalculiX needs for them are generated when the input file is
//! written.

use serde::{Deserialize, Serialize};

use crate::Region;

/// Nodes tied to each other, nodalis's node tie: the ends of beams or trusses meeting at a
/// point, which have nodes of their own since parts share no nodes. Every node of the region
/// moves with the first. Node ties are listed with the contact pairs, as the contact search
/// finds them.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NodeTie {
    pub name: String,
    #[serde(default = "crate::active")]
    pub active: bool,
    pub region: Region,
    /// Ties the rotations too, a rigid joint between beams; a hinge otherwise. Nodes
    /// without rotations (trusses, solids) are tied in their translations only either way.
    pub rotations: bool,
}

impl NodeTie {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            region: Region::Nodes(Vec::new()),
            rotations: true,
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

/// Springs in the global directions from the nodes of a region to ground (`SPRING1`), or to
/// the closest points of a target surface (`SPRING2`). One spring
/// constraint covers the point and surface springs: like a force, the stiffness is
/// that of the whole region and spread over its nodes, on faces by area, on edges by length
/// and on points in equal shares.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Spring {
    pub name: String,
    #[serde(default = "crate::active")]
    pub active: bool,
    /// Hidden constraints have no symbol in the 3D view unless selected.
    #[serde(default = "crate::active")]
    pub visible: bool,
    /// The nodes that get springs: the slave side of a connection to a surface.
    pub region: Region,
    /// The surface the springs end on, the master side; `None` connects them to ground.
    pub master: Option<Region>,
    /// K1..K3 of the whole region in force per length, or per area of its faces (force per
    /// volume) with `per_area`, or of every node with `each_node`; a zero adds no spring
    /// in that direction.
    pub stiffness: [f64; 3],
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
            region: Region::Nodes(Vec::new()),
            master: None,
            stiffness: [0.0; 3],
            per_area: false,
            each_node: false,
        }
    }

    /// The stiffness is per area of the faces of the region.
    pub fn is_per_area(&self) -> bool {
        self.per_area && !self.each_node
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
