//! Sections: what gives elements their material and their geometry beyond the nodes.

use serde::{Deserialize, Serialize};

use crate::mesh::NodeId;

/// A section on an element set. `elset` is upper case; `material` too.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Section {
    pub elset: String,
    pub kind: SectionKind,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SectionKind {
    /// `*SOLID SECTION`: solids, 2D elements (with their thickness), trusses (with their
    /// cross-section area in the same field).
    Solid {
        material: String,
        /// Thickness of plane stress/strain elements or area of trusses, when given.
        thickness: Option<f64>,
    },
    /// `*SHELL SECTION`.
    Shell {
        material: String,
        thickness: f64,
        /// Offset of the reference surface in units of the thickness.
        offset: f64,
        /// Thickness read from the nodes (`NODAL THICKNESS`) instead; not written by Nodalis.
        nodal_thickness: bool,
    },
    /// `*MEMBRANE SECTION`.
    Membrane {
        material: String,
        thickness: f64,
        offset: f64,
    },
    /// `*BEAM SECTION`, also with `SECTION=GENERAL` on Nodalis's `U1` user elements
    /// ([`BeamProfile::General`]).
    Beam {
        material: String,
        profile: BeamProfile,
        /// The direction (in the data) that fixes the orientation of the profile, the first
        /// local axis as CalculiX defines it.
        normal: [f64; 3],
        offset1: f64,
        offset2: f64,
    },
    /// `*SPRING` on `SPRING1`/`SPRING2` elements.
    Spring(Spring),
    /// `*GAP` on `GAPUNI` elements.
    Gap(Gap),
}

impl SectionKind {
    /// The material of the section, if it has one (springs and gaps have none).
    pub fn material(&self) -> Option<&str> {
        match self {
            SectionKind::Solid { material, .. }
            | SectionKind::Shell { material, .. }
            | SectionKind::Membrane { material, .. }
            | SectionKind::Beam { material, .. } => Some(material),
            SectionKind::Spring(_) | SectionKind::Gap(_) => None,
        }
    }
}

/// Beam profiles, with the dimensions of CalculiX's `SECTION=` types and the general values
/// of Nodalis's `U1` beam.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum BeamProfile {
    /// `SECTION=RECT`: widths in the two local directions.
    Rect { a: f64, b: f64 },
    /// `SECTION=CIRC`: radii (an ellipse when they differ).
    Circ { a: f64, b: f64 },
    /// `SECTION=PIPE`: outer radius and wall thickness.
    Pipe { radius: f64, thickness: f64 },
    /// `SECTION=BOX`: outer dimensions and the four wall thicknesses.
    Box {
        a: f64,
        b: f64,
        t1: f64,
        t2: f64,
        t3: f64,
        t4: f64,
    },
    /// General values of a Timoshenko beam: area, second moments about the local axes
    /// (`i11`, `i22`, product `i12`), torsion constant and the shear areas; what the U1
    /// data lines carry.
    General {
        area: f64,
        i11: f64,
        i12: f64,
        i22: f64,
        torsion: f64,
        shear_area_1: Option<f64>,
        shear_area_2: Option<f64>,
        /// `CONTACT RADIUS=` (own parameter): node-to-surface contact keeps the beam's axis
        /// this far from the master surface, the radius of a rope.
        #[serde(default)]
        contact_radius: Option<f64>,
    },
}

/// `*SPRING`: the stiffness of a set of spring elements. A `SPRING1` acts on one DOF of one
/// node, a `SPRING2` between one DOF of each of its two nodes. The stiffness is constant or
/// a force / displacement table (`NONLINEAR`) per temperature.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Spring {
    pub dof1: u8,
    pub dof2: u8,
    pub stiffness: SpringStiffness,
    /// `ORIENTATION` is not written by Nodalis; springs act along global axes.
    pub orientation: Option<String>,
    /// `STABILIZE` (own parameter): the spring also resists the displacement within each
    /// increment of a nonlinear static step with this factor times its stiffness, a force
    /// that starts from zero in every increment and so leaves none behind. Zero: none.
    #[serde(default)]
    pub stabilize: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SpringStiffness {
    Linear(Vec<crate::material::TemperatureValue>),
    /// (force, displacement, temperature) rows.
    Nonlinear(Vec<(f64, f64, f64)>),
}

/// `*GAP` on `GAPUNI` elements: a compression-only link between two nodes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Gap {
    /// Initial clearance.
    pub clearance: f64,
    /// Direction of the gap (from the first to the second node).
    pub direction: [f64; 3],
    /// Stiffness in the normal direction when given (CalculiX otherwise uses a default).
    pub stiffness: Option<f64>,
    /// Friction coefficient when given.
    pub friction: Option<f64>,
}

/// `*NORMAL`, which Nodalis does not write: a normal at a node of a beam or shell element.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct NodeNormal {
    pub element: crate::mesh::ElementId,
    pub node: NodeId,
    pub normal: [f64; 3],
}
