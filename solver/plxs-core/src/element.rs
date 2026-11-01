//! Elements and their types: everything Nodalis writes (concept section 1.1).

use serde::{Deserialize, Serialize};

use crate::mesh::{ElementId, NodeId};

/// An element: user id, type, its nodes in CalculiX order, and the section (index into
/// [`crate::Model::sections`]) assigned to it, once sections are resolved.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Element {
    pub id: ElementId,
    pub kind: ElementType,
    pub nodes: Vec<NodeId>,
    /// Index of the section that holds this element's material and geometry; `None` until
    /// [`crate::validate`] assigned it, or for elements that need none (springs, gaps get
    /// theirs by their own keywords).
    pub section: Option<u32>,
}

/// What an element type is, for anything that does not care about the exact type.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ElementFamily {
    /// Three-dimensional continuum.
    Solid,
    /// Plane stress.
    PlaneStress,
    /// Plane strain.
    PlaneStrain,
    /// Axisymmetric.
    Axisymmetric,
    Shell,
    Membrane,
    Beam,
    Truss,
    Spring,
    Gap,
    /// A user element (`*USER ELEMENT`); Nodalis writes `U1` for general beam sections.
    User,
}

/// The CalculiX element types Nodalis writes, spelled as in the input.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[allow(clippy::upper_case_acronyms)]
pub enum ElementType {
    // Solids.
    C3D4,
    C3D10,
    C3D10T,
    C3D6,
    C3D15,
    C3D8,
    C3D8R,
    C3D8I,
    C3D20,
    C3D20R,
    // Plane stress.
    CPS3,
    CPS4,
    CPS4R,
    CPS6,
    CPS8,
    CPS8R,
    // Plane strain.
    CPE3,
    CPE4,
    CPE4R,
    CPE6,
    CPE8,
    CPE8R,
    // Axisymmetric.
    CAX3,
    CAX4,
    CAX4R,
    CAX6,
    CAX8,
    CAX8R,
    // Shells.
    S3,
    S4,
    S4R,
    S6,
    S8,
    S8R,
    // Membranes.
    M3D3,
    M3D4,
    M3D4R,
    M3D6,
    // Beams and trusses.
    B31,
    B32,
    B32R,
    T3D2,
    // Springs and gaps.
    SPRING1,
    SPRING2,
    GAPUNI,
    /// General beam (`*USER ELEMENT, TYPE=U1`), two nodes, six DOF each.
    U1,
}

impl ElementType {
    /// The type for its name in the input (case-insensitive), if Nodalis writes it.
    pub fn parse(name: &str) -> Option<Self> {
        use ElementType::*;
        Some(match name.trim().to_ascii_uppercase().as_str() {
            "C3D4" => C3D4,
            "C3D10" => C3D10,
            "C3D10T" => C3D10T,
            "C3D6" => C3D6,
            "C3D15" => C3D15,
            "C3D8" => C3D8,
            "C3D8R" => C3D8R,
            "C3D8I" => C3D8I,
            "C3D20" => C3D20,
            "C3D20R" => C3D20R,
            "CPS3" => CPS3,
            "CPS4" => CPS4,
            "CPS4R" => CPS4R,
            "CPS6" => CPS6,
            "CPS8" => CPS8,
            "CPS8R" => CPS8R,
            "CPE3" => CPE3,
            "CPE4" => CPE4,
            "CPE4R" => CPE4R,
            "CPE6" => CPE6,
            "CPE8" => CPE8,
            "CPE8R" => CPE8R,
            "CAX3" => CAX3,
            "CAX4" => CAX4,
            "CAX4R" => CAX4R,
            "CAX6" => CAX6,
            "CAX8" => CAX8,
            "CAX8R" => CAX8R,
            "S3" => S3,
            "S4" => S4,
            "S4R" => S4R,
            "S6" => S6,
            "S8" => S8,
            "S8R" => S8R,
            "M3D3" => M3D3,
            "M3D4" => M3D4,
            "M3D4R" => M3D4R,
            "M3D6" => M3D6,
            "B31" => B31,
            "B32" => B32,
            "B32R" => B32R,
            "T3D2" => T3D2,
            "SPRING1" => SPRING1,
            "SPRING2" => SPRING2,
            "GAPUNI" => GAPUNI,
            "U1" => U1,
            _ => return None,
        })
    }

    /// The name as written in the input.
    pub fn name(self) -> &'static str {
        use ElementType::*;
        match self {
            C3D4 => "C3D4",
            C3D10 => "C3D10",
            C3D10T => "C3D10T",
            C3D6 => "C3D6",
            C3D15 => "C3D15",
            C3D8 => "C3D8",
            C3D8R => "C3D8R",
            C3D8I => "C3D8I",
            C3D20 => "C3D20",
            C3D20R => "C3D20R",
            CPS3 => "CPS3",
            CPS4 => "CPS4",
            CPS4R => "CPS4R",
            CPS6 => "CPS6",
            CPS8 => "CPS8",
            CPS8R => "CPS8R",
            CPE3 => "CPE3",
            CPE4 => "CPE4",
            CPE4R => "CPE4R",
            CPE6 => "CPE6",
            CPE8 => "CPE8",
            CPE8R => "CPE8R",
            CAX3 => "CAX3",
            CAX4 => "CAX4",
            CAX4R => "CAX4R",
            CAX6 => "CAX6",
            CAX8 => "CAX8",
            CAX8R => "CAX8R",
            S3 => "S3",
            S4 => "S4",
            S4R => "S4R",
            S6 => "S6",
            S8 => "S8",
            S8R => "S8R",
            M3D3 => "M3D3",
            M3D4 => "M3D4",
            M3D4R => "M3D4R",
            M3D6 => "M3D6",
            B31 => "B31",
            B32 => "B32",
            B32R => "B32R",
            T3D2 => "T3D2",
            SPRING1 => "SPRING1",
            SPRING2 => "SPRING2",
            GAPUNI => "GAPUNI",
            U1 => "U1",
        }
    }

    pub fn family(self) -> ElementFamily {
        use ElementType::*;
        match self {
            C3D4 | C3D10 | C3D10T | C3D6 | C3D15 | C3D8 | C3D8R | C3D8I | C3D20 | C3D20R => {
                ElementFamily::Solid
            }
            CPS3 | CPS4 | CPS4R | CPS6 | CPS8 | CPS8R => ElementFamily::PlaneStress,
            CPE3 | CPE4 | CPE4R | CPE6 | CPE8 | CPE8R => ElementFamily::PlaneStrain,
            CAX3 | CAX4 | CAX4R | CAX6 | CAX8 | CAX8R => ElementFamily::Axisymmetric,
            S3 | S4 | S4R | S6 | S8 | S8R => ElementFamily::Shell,
            M3D3 | M3D4 | M3D4R | M3D6 => ElementFamily::Membrane,
            B31 | B32 | B32R => ElementFamily::Beam,
            T3D2 => ElementFamily::Truss,
            SPRING1 | SPRING2 => ElementFamily::Spring,
            GAPUNI => ElementFamily::Gap,
            U1 => ElementFamily::User,
        }
    }

    /// Number of nodes of the element.
    pub fn node_count(self) -> usize {
        use ElementType::*;
        match self {
            C3D4 => 4,
            C3D10 | C3D10T => 10,
            C3D6 => 6,
            C3D15 => 15,
            C3D8 | C3D8R | C3D8I => 8,
            C3D20 | C3D20R => 20,
            CPS3 | CPE3 | CAX3 | S3 | M3D3 => 3,
            CPS4 | CPS4R | CPE4 | CPE4R | CAX4 | CAX4R | S4 | S4R | M3D4 | M3D4R => 4,
            CPS6 | CPE6 | CAX6 | S6 | M3D6 => 6,
            CPS8 | CPS8R | CPE8 | CPE8R | CAX8 | CAX8R | S8 | S8R => 8,
            B31 | T3D2 | SPRING2 | GAPUNI | U1 => 2,
            B32 | B32R => 3,
            SPRING1 => 1,
        }
    }

    /// Whether the element has midside nodes.
    pub fn is_quadratic(self) -> bool {
        use ElementType::*;
        matches!(
            self,
            C3D10
                | C3D10T
                | C3D15
                | C3D20
                | C3D20R
                | CPS6
                | CPS8
                | CPS8R
                | CPE6
                | CPE8
                | CPE8R
                | CAX6
                | CAX8
                | CAX8R
                | S6
                | S8
                | S8R
                | M3D6
                | B32
                | B32R
        )
    }

    /// Degrees of freedom per node the element carries: 3 translations, 6 with rotations,
    /// 1 for the temperature-only case is decided by the procedure, not here.
    pub fn dofs_per_node(self) -> u8 {
        match self.family() {
            ElementFamily::Shell | ElementFamily::Beam | ElementFamily::User => 6,
            ElementFamily::PlaneStress
            | ElementFamily::PlaneStrain
            | ElementFamily::Axisymmetric => 2,
            ElementFamily::Spring | ElementFamily::Gap => 3,
            _ => 3,
        }
    }
}

impl std::fmt::Display for ElementType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}
