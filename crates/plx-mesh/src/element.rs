/// Geometric shape of an element, independent of its formulation (C3D8 vs. C3D8R, S4 vs. CPS4 …).
///
/// Node numbering follows CalculiX/Abaqus: corner nodes first, then midside nodes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ElementShape {
    Line2,
    Line3,
    Tri3,
    Tri6,
    Quad4,
    Quad8,
    Tet4,
    Tet10,
    Wedge6,
    Wedge15,
    Hex8,
    Hex20,
}

/// What kind of entity an element is, used for display and skin extraction.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ElementFamily {
    Line,
    Surface,
    Solid,
}

/// One face of an element: corner nodes in order, followed by midside nodes
/// (`mids[i]` lies between `corners[i]` and `corners[i + 1]`). Indices are 0-based local nodes;
/// `mids` only exist on quadratic shapes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceTopology {
    pub corners: &'static [usize],
    pub mids: &'static [usize],
}

const fn face(corners: &'static [usize], mids: &'static [usize]) -> FaceTopology {
    FaceTopology { corners, mids }
}

const TET_FACES: [FaceTopology; 4] = [
    face(&[0, 1, 2], &[4, 5, 6]),
    face(&[0, 3, 1], &[7, 8, 4]),
    face(&[1, 3, 2], &[8, 9, 5]),
    face(&[2, 3, 0], &[9, 7, 6]),
];

const WEDGE_FACES: [FaceTopology; 5] = [
    face(&[0, 1, 2], &[6, 7, 8]),
    face(&[3, 5, 4], &[11, 10, 9]),
    face(&[0, 3, 4, 1], &[12, 9, 13, 6]),
    face(&[1, 4, 5, 2], &[13, 10, 14, 7]),
    face(&[2, 5, 3, 0], &[14, 11, 12, 8]),
];

const HEX_FACES: [FaceTopology; 6] = [
    face(&[0, 1, 2, 3], &[8, 9, 10, 11]),
    face(&[4, 7, 6, 5], &[15, 14, 13, 12]),
    face(&[0, 4, 5, 1], &[16, 12, 17, 8]),
    face(&[1, 5, 6, 2], &[17, 13, 18, 9]),
    face(&[2, 6, 7, 3], &[18, 14, 19, 10]),
    face(&[3, 7, 4, 0], &[19, 15, 16, 11]),
];

const TRI_EDGES: [FaceTopology; 3] = [
    face(&[0, 1], &[3]),
    face(&[1, 2], &[4]),
    face(&[2, 0], &[5]),
];

const QUAD_EDGES: [FaceTopology; 4] = [
    face(&[0, 1], &[4]),
    face(&[1, 2], &[5]),
    face(&[2, 3], &[6]),
    face(&[3, 0], &[7]),
];

const TRI_FACE: [FaceTopology; 1] = [face(&[0, 1, 2], &[3, 4, 5])];
const QUAD_FACE: [FaceTopology; 1] = [face(&[0, 1, 2, 3], &[4, 5, 6, 7])];

/// Faces of a shell or membrane in CalculiX order: S1 the bottom (NEG) and S2 the top (POS)
/// of the element, both listed with the element's own node order, then S3 and up its edges.
const TRI_SHELL_FACES: [FaceTopology; 5] = [
    TRI_FACE[0],
    TRI_FACE[0],
    TRI_EDGES[0],
    TRI_EDGES[1],
    TRI_EDGES[2],
];
const QUAD_SHELL_FACES: [FaceTopology; 6] = [
    QUAD_FACE[0],
    QUAD_FACE[0],
    QUAD_EDGES[0],
    QUAD_EDGES[1],
    QUAD_EDGES[2],
    QUAD_EDGES[3],
];

impl ElementShape {
    /// Maps a CalculiX/Abaqus element type name to its shape; `None` for unsupported types.
    pub fn from_type_name(name: &str) -> Option<Self> {
        let name = name.trim().to_ascii_uppercase();
        let digits = |prefix: &str| -> Option<u32> {
            let rest = name.strip_prefix(prefix)?;
            let end = rest
                .find(|c: char| !c.is_ascii_digit())
                .unwrap_or(rest.len());
            rest[..end].parse().ok()
        };
        if let Some(n) = digits("C3D").or_else(|| digits("F3D")) {
            return match n {
                4 => Some(Self::Tet4),
                10 => Some(Self::Tet10),
                6 => Some(Self::Wedge6),
                15 => Some(Self::Wedge15),
                8 => Some(Self::Hex8),
                20 => Some(Self::Hex20),
                _ => None,
            };
        }
        let surface = ["S", "M3D", "CPS", "CPE", "CAX"]
            .iter()
            .find_map(|prefix| digits(prefix));
        if let Some(n) = surface {
            return match n {
                3 => Some(Self::Tri3),
                6 => Some(Self::Tri6),
                4 => Some(Self::Quad4),
                8 => Some(Self::Quad8),
                _ => None,
            };
        }
        // U1 is the user element the solver has general beam sections for, a 2-node beam.
        if name.starts_with("B31")
            || name.starts_with("T3D2")
            || name.starts_with("T2D2")
            || name == "U1"
        {
            return Some(Self::Line2);
        }
        if name.starts_with("B32") || name.starts_with("T3D3") || name.starts_with("T2D3") {
            return Some(Self::Line3);
        }
        None
    }

    pub fn node_count(self) -> usize {
        match self {
            Self::Line2 => 2,
            Self::Line3 | Self::Tri3 => 3,
            Self::Quad4 | Self::Tet4 => 4,
            Self::Tri6 | Self::Wedge6 => 6,
            Self::Quad8 | Self::Hex8 => 8,
            Self::Tet10 => 10,
            Self::Wedge15 => 15,
            Self::Hex20 => 20,
        }
    }

    pub fn family(self) -> ElementFamily {
        match self {
            Self::Line2 | Self::Line3 => ElementFamily::Line,
            Self::Tri3 | Self::Tri6 | Self::Quad4 | Self::Quad8 => ElementFamily::Surface,
            _ => ElementFamily::Solid,
        }
    }

    pub fn is_quadratic(self) -> bool {
        matches!(
            self,
            Self::Line3 | Self::Tri6 | Self::Quad8 | Self::Tet10 | Self::Wedge15 | Self::Hex20
        )
    }

    /// Faces in CalculiX face order (S1, S2, …); a surface element has exactly one face.
    pub fn faces(self) -> &'static [FaceTopology] {
        let all: &'static [FaceTopology] = match self {
            Self::Line2 | Self::Line3 => &[],
            Self::Tri3 | Self::Tri6 => &TRI_FACE,
            Self::Quad4 | Self::Quad8 => &QUAD_FACE,
            Self::Tet4 | Self::Tet10 => &TET_FACES,
            Self::Wedge6 | Self::Wedge15 => &WEDGE_FACES,
            Self::Hex8 | Self::Hex20 => &HEX_FACES,
        };
        all
    }

    /// Faces of a shell or membrane element of this shape in CalculiX order, see
    /// [`crate::Element::faces`]; other shapes have their [`Self::faces`].
    pub fn shell_faces(self) -> &'static [FaceTopology] {
        match self {
            Self::Tri3 | Self::Tri6 => &TRI_SHELL_FACES,
            Self::Quad4 | Self::Quad8 => &QUAD_SHELL_FACES,
            _ => self.faces(),
        }
    }

    /// Edges of a triangle or quadrilateral in CalculiX order, the faces of plane elements;
    /// other shapes have none.
    pub fn edges(self) -> &'static [FaceTopology] {
        match self {
            Self::Tri3 | Self::Tri6 => &TRI_EDGES,
            Self::Quad4 | Self::Quad8 => &QUAD_EDGES,
            _ => &[],
        }
    }

    /// Local node chains of a line element, as segments for drawing.
    pub fn line_segments(self) -> &'static [[usize; 2]] {
        match self {
            Self::Line2 => &[[0, 1]],
            Self::Line3 => &[[0, 1], [1, 2]],
            _ => &[],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn type_names_map_to_shapes() {
        let cases = [
            ("C3D4", ElementShape::Tet4),
            ("c3d10", ElementShape::Tet10),
            ("C3D8R", ElementShape::Hex8),
            ("C3D8I", ElementShape::Hex8),
            ("C3D20R", ElementShape::Hex20),
            ("C3D15", ElementShape::Wedge15),
            ("S4R", ElementShape::Quad4),
            ("S8R", ElementShape::Quad8),
            ("CPS3", ElementShape::Tri3),
            ("CAX8R", ElementShape::Quad8),
            ("M3D6", ElementShape::Tri6),
            ("B31", ElementShape::Line2),
            ("U1", ElementShape::Line2),
            ("B32R", ElementShape::Line3),
            ("T3D2", ElementShape::Line2),
        ];
        for (name, shape) in cases {
            assert_eq!(ElementShape::from_type_name(name), Some(shape), "{name}");
        }
        for name in ["SPRINGA", "DASHPOTA", "MASS", "GAPUNI", "C3D9", "DCOUP3D"] {
            assert_eq!(ElementShape::from_type_name(name), None, "{name}");
        }
    }

    #[test]
    fn face_tables_reference_valid_nodes() {
        use ElementShape::*;
        for shape in [
            Tri3, Tri6, Quad4, Quad8, Tet4, Tet10, Wedge6, Wedge15, Hex8, Hex20,
        ] {
            for f in shape.faces() {
                assert_eq!(f.corners.len(), f.mids.len());
                let max = if shape.is_quadratic() {
                    f.corners.iter().chain(f.mids).max()
                } else {
                    f.corners.iter().max()
                };
                assert!(*max.unwrap() < shape.node_count(), "{shape:?}");
            }
        }
    }

    #[test]
    fn every_solid_edge_is_shared_by_two_faces_of_the_element() {
        use std::collections::HashMap;
        for shape in [ElementShape::Tet4, ElementShape::Wedge6, ElementShape::Hex8] {
            let mut edges: HashMap<(usize, usize), i32> = HashMap::new();
            for f in shape.faces() {
                for i in 0..f.corners.len() {
                    let (a, b) = (f.corners[i], f.corners[(i + 1) % f.corners.len()]);
                    *edges.entry((a, b)).or_default() += 1;
                    *edges.entry((b, a)).or_default() -= 1;
                }
            }
            // Consistently oriented faces traverse each edge once in each direction.
            assert!(edges.values().all(|&n| n == 0), "{shape:?}");
        }
    }
}
