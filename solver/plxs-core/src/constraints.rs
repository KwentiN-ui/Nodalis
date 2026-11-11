//! Multi-point constraints and related model data (concept section 1.3). Single-point
//! constraints (`*BOUNDARY`) belong to the steps, see [`crate::step::Boundary`].

use serde::{Deserialize, Serialize};

use crate::mesh::{NodeId, NodeRef};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Constraints {
    pub equations: Vec<Equation>,
    pub rigid_bodies: Vec<RigidBody>,
    /// Joints between two points, Nodalis's own keyword `*JOINT`.
    #[serde(default)]
    pub joints: Vec<Joint>,
    pub ties: Vec<Tie>,
    pub cyclic_symmetry: Option<CyclicSymmetryModel>,
    pub transforms: Vec<Transform>,
    pub submodel: Option<Submodel>,
    /// Beam and shell normals given explicitly (`*NORMAL`), none from Nodalis.
    pub normals: Vec<crate::section::NodeNormal>,
}

/// `*EQUATION`: sum of coefficient * u(node, dof) = 0. The first term's DOF is the dependent
/// one CalculiX eliminates; the solver may re-pivot when it is held otherwise.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Equation {
    pub terms: Vec<EquationTerm>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EquationTerm {
    pub node: NodeRef,
    pub dof: u8,
    pub coefficient: f64,
}

/// `*RIGID BODY`: the nodes of a set move as a rigid body with the translation of the
/// reference node and the rotation (vector) of the rotation node.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigidBody {
    pub nodes: RigidBodyNodes,
    pub reference_node: NodeId,
    pub rotation_node: NodeId,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RigidBodyNodes {
    NodeSet(String),
    ElementSet(String),
}

/// `*JOINT` (no CalculiX keyword): a kinematic joint between two points, each the
/// reference and rotation node of a rigid body or of a deformable coupling, or ground.
/// The joint holds the relative motion of the second end against the first in the fixed
/// directions of its frame: translations of the material points at `origin`, and the
/// components of the relative rotation. The frame turns with the first end (in large
/// rotations, NLGEOM); with ground as the first end it stays in place.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Joint {
    pub name: String,
    /// Reference and rotation node of each end; `None` for ground.
    pub ends: [Option<[NodeId; 2]>; 2],
    /// Where the joint is, undeformed.
    pub origin: [f64; 3],
    /// The local x, y and z directions of the joint frame (global components, orthonormal).
    pub axes: [[f64; 3]; 3],
    /// Held relative degrees of freedom: translations along x, y, z, then rotations about
    /// them.
    pub fixed: [bool; 6],
}

/// `*TIE`: the slave surface follows the master surface (nodes within the position
/// tolerance are projected and coupled by multi-point constraints).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Tie {
    pub name: String,
    pub slave: String,
    pub master: String,
    pub position_tolerance: Option<f64>,
    /// `ADJUST=NO` leaves the slave nodes where they are instead of moving them on to the
    /// master surface.
    pub adjust: bool,
    /// The tie couples the two sides of a cyclic symmetry sector (`CYCLIC SYMMETRY`).
    pub cyclic_symmetry: bool,
    /// `MULTISTAGE`, not written by Nodalis.
    pub multistage: bool,
}

/// `*CYCLIC SYMMETRY MODEL`: the sector in the model is one of `sectors` around the axis
/// through the two points; `ngraph` sectors are expanded in the output.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CyclicSymmetryModel {
    pub sectors: u32,
    pub ngraph: u32,
    pub tie: String,
    /// Two points on the axis of symmetry.
    pub axis: [[f64; 3]; 2],
    /// `ELSET`, when the sector is only part of the model.
    pub elset: Option<String>,
    /// `CHECK=NO`.
    pub check: bool,
}

/// `*TRANSFORM`: a local system for the degrees of freedom of a node set, rectangular (`R`,
/// points give the local x axis and a point in the local xy plane) or cylindrical (`C`, the
/// points lie on the axis).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Transform {
    pub nset: String,
    pub kind: TransformKind,
    pub a: [f64; 3],
    pub b: [f64; 3],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TransformKind {
    Rectangular,
    Cylindrical,
}

/// `*SUBMODEL, TYPE=NODE`: the nodes of the given sets take their displacements (or
/// temperatures) from the results of a global model, read from `input`, in the steps whose
/// `*BOUNDARY, SUBMODEL` names them.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Submodel {
    pub input: String,
    pub node_sets: Vec<String>,
    /// `GLOBAL ELSET`: the elements of the global model to interpolate in.
    pub global_elset: Option<String>,
}
