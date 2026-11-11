//! Jacobian determinant of elements at their integration points, the check the solver makes
//! before it computes the stiffness ("nonpositive jacobian determinant in element"). An
//! element with a determinant of zero or less is inverted or so distorted that its mapping
//! folds over, for example a quadratic tetrahedron whose midside node was moved past a
//! corner.

use crate::element::ElementShape;
use crate::mesh::{Element, FeMesh};

/// 1/sqrt(3), the Gauss points of the two point rule.
const G: f64 = 0.577_350_269_189_625_8;

impl Element {
    /// Smallest Jacobian determinant at the integration points, or `None` for elements
    /// the solver does not check this way (beams, shells) or with missing nodes. Plane
    /// elements are checked in the xy plane: CalculiX expects them counterclockwise.
    pub fn min_jacobian(&self, mesh: &FeMesh) -> Option<f64> {
        self.min_jacobian_at(|node| mesh.node(node))
    }

    /// [`Element::min_jacobian`] with the nodes where `position` puts them, e.g. moved
    /// before the mesh itself is.
    pub fn min_jacobian_at(
        &self,
        position: impl Fn(crate::NodeId) -> Option<[f64; 3]>,
    ) -> Option<f64> {
        let dim = if self.shape.is_solid() {
            3
        } else if self.is_plane() && self.shape.family() == crate::ElementFamily::Surface {
            2
        } else {
            return None;
        };
        let count = self.shape.node_count();
        let mut coords = [[0.0; 3]; 20];
        if self.nodes.len() < count {
            return None;
        }
        for (c, &node) in coords.iter_mut().zip(&self.nodes[..count]) {
            *c = position(node)?;
        }
        derivatives(self.shape)
            .iter()
            .map(|point| {
                let mut j = [[0.0; 3]; 3];
                for (d, c) in point.iter().zip(&coords) {
                    for i in 0..dim {
                        for (k, dk) in d.iter().enumerate().take(dim) {
                            j[i][k] += dk * c[i];
                        }
                    }
                }
                if dim == 3 {
                    determinant3(&j)
                } else {
                    j[0][0] * j[1][1] - j[0][1] * j[1][0]
                }
            })
            .reduce(f64::min)
    }
}

/// Derivatives of the shape functions by the natural coordinates at each integration point,
/// computed once per shape.
fn derivatives(shape: ElementShape) -> &'static [Vec<[f64; 3]>] {
    use std::sync::OnceLock;
    const SHAPES: [ElementShape; 10] = [
        ElementShape::Tri3,
        ElementShape::Tri6,
        ElementShape::Quad4,
        ElementShape::Quad8,
        ElementShape::Tet4,
        ElementShape::Tet10,
        ElementShape::Wedge6,
        ElementShape::Wedge15,
        ElementShape::Hex8,
        ElementShape::Hex20,
    ];
    static TABLES: OnceLock<Vec<Vec<Vec<[f64; 3]>>>> = OnceLock::new();
    let tables = TABLES.get_or_init(|| {
        SHAPES
            .iter()
            .map(|&shape| {
                let points: &[[f64; 3]] = match shape {
                    ElementShape::Tet4 | ElementShape::Tet10 => &TET_POINTS,
                    ElementShape::Wedge6 | ElementShape::Wedge15 => &WEDGE_POINTS,
                    ElementShape::Tri3 | ElementShape::Tri6 => &TRI_POINTS,
                    ElementShape::Quad4 | ElementShape::Quad8 => &QUAD_POINTS,
                    _ => &HEX_POINTS,
                };
                points
                    .iter()
                    .map(|&p| point_derivatives(shape, p))
                    .collect()
            })
            .collect()
    });
    SHAPES
        .iter()
        .position(|&s| s == shape)
        .map_or(&[], |i| tables[i].as_slice())
}

/// dN/dxi of every node at the natural coordinates `p`, from central differences of the
/// shape functions (polynomials of degree three at most, so the differences are exact to
/// rounding).
pub(crate) fn point_derivatives(shape: ElementShape, p: [f64; 3]) -> Vec<[f64; 3]> {
    const H: f64 = 1e-5;
    let mut out = vec![[0.0; 3]; shape.node_count()];
    let mut plus = [0.0; 20];
    let mut minus = [0.0; 20];
    for k in 0..3 {
        let mut a = p;
        let mut b = p;
        a[k] += H;
        b[k] -= H;
        shape_functions(shape, a, &mut plus);
        shape_functions(shape, b, &mut minus);
        for (node, d) in out.iter_mut().enumerate() {
            d[k] = (plus[node] - minus[node]) / (2.0 * H);
        }
    }
    out
}

impl ElementShape {
    fn is_solid(self) -> bool {
        matches!(self.family(), crate::ElementFamily::Solid)
    }
}

const TET_POINTS: [[f64; 3]; 4] = {
    let a = 0.585_410_196_624_968_5;
    let b = 0.138_196_601_125_010_5;
    [[b, b, b], [a, b, b], [b, a, b], [b, b, a]]
};

const WEDGE_POINTS: [[f64; 3]; 6] = {
    let (a, b) = (1.0 / 6.0, 2.0 / 3.0);
    [
        [a, a, -G],
        [b, a, -G],
        [a, b, -G],
        [a, a, G],
        [b, a, G],
        [a, b, G],
    ]
};

const HEX_POINTS: [[f64; 3]; 8] = [
    [-G, -G, -G],
    [G, -G, -G],
    [G, G, -G],
    [-G, G, -G],
    [-G, -G, G],
    [G, -G, G],
    [G, G, G],
    [-G, G, G],
];

const TRI_POINTS: [[f64; 3]; 3] = {
    let (a, b) = (1.0 / 6.0, 2.0 / 3.0);
    [[a, a, 0.0], [b, a, 0.0], [a, b, 0.0]]
};

const QUAD_POINTS: [[f64; 3]; 4] = [[-G, -G, 0.0], [G, -G, 0.0], [G, G, 0.0], [-G, G, 0.0]];

fn determinant3(m: &[[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// Natural coordinates of the corners of hexahedra and quadrilaterals, in CalculiX order.
const HEX_CORNERS: [[f64; 3]; 8] = [
    [-1.0, -1.0, -1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, -1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [1.0, -1.0, 1.0],
    [1.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0],
];

/// Corners joined by the midside nodes of a C3D20, in CalculiX order (nodes 9 to 20).
const HEX_EDGES: [[usize; 2]; 12] = [
    [0, 1],
    [1, 2],
    [2, 3],
    [3, 0],
    [4, 5],
    [5, 6],
    [6, 7],
    [7, 4],
    [0, 4],
    [1, 5],
    [2, 6],
    [3, 7],
];

/// Corners joined by the midside nodes of a C3D10 (nodes 5 to 10).
const TET_EDGES: [[usize; 2]; 6] = [[0, 1], [1, 2], [2, 0], [0, 3], [1, 3], [2, 3]];

/// Corners joined by the midside nodes of a C3D15 (nodes 7 to 15).
const WEDGE_EDGES: [[usize; 2]; 9] = [
    [0, 1],
    [1, 2],
    [2, 0],
    [3, 4],
    [4, 5],
    [5, 3],
    [0, 3],
    [1, 4],
    [2, 5],
];

/// Values of the shape functions at `p` written to `n`, node order as in CalculiX.
pub(crate) fn shape_functions(shape: ElementShape, p: [f64; 3], n: &mut [f64; 20]) {
    let [r, s, t] = p;
    match shape {
        ElementShape::Tet4 | ElementShape::Tet10 => {
            let l = [1.0 - r - s - t, r, s, t];
            if shape == ElementShape::Tet4 {
                n[..4].copy_from_slice(&l);
            } else {
                for i in 0..4 {
                    n[i] = l[i] * (2.0 * l[i] - 1.0);
                }
                for (k, [a, b]) in TET_EDGES.iter().enumerate() {
                    n[4 + k] = 4.0 * l[*a] * l[*b];
                }
            }
        }
        ElementShape::Tri3 | ElementShape::Tri6 => {
            let l = [1.0 - r - s, r, s];
            if shape == ElementShape::Tri3 {
                n[..3].copy_from_slice(&l);
            } else {
                for i in 0..3 {
                    n[i] = l[i] * (2.0 * l[i] - 1.0);
                }
                for (k, [a, b]) in [[0, 1], [1, 2], [2, 0]].iter().enumerate() {
                    n[3 + k] = 4.0 * l[*a] * l[*b];
                }
            }
        }
        ElementShape::Wedge6 | ElementShape::Wedge15 => {
            let l = [1.0 - r - s, r, s];
            let z = |corner: usize| if corner < 3 { -1.0 } else { 1.0 };
            for corner in 0..6 {
                let li = l[corner % 3];
                let zi = z(corner);
                n[corner] = if shape == ElementShape::Wedge6 {
                    0.5 * li * (1.0 + zi * t)
                } else {
                    0.5 * li * (2.0 * li - 1.0) * (1.0 + zi * t) - 0.5 * li * (1.0 - t * t)
                };
            }
            if shape == ElementShape::Wedge15 {
                for (k, [a, b]) in WEDGE_EDGES.iter().enumerate() {
                    n[6 + k] = if k < 6 {
                        2.0 * l[a % 3] * l[b % 3] * (1.0 + z(*a) * t)
                    } else {
                        l[a % 3] * (1.0 - t * t)
                    };
                }
            }
        }
        ElementShape::Hex8 | ElementShape::Quad4 => {
            let corners = if shape == ElementShape::Hex8 { 8 } else { 4 };
            for (i, c) in HEX_CORNERS.iter().take(corners).enumerate() {
                let w = if corners == 8 {
                    0.125 * (1.0 + c[2] * t)
                } else {
                    0.25
                };
                n[i] = w * (1.0 + c[0] * r) * (1.0 + c[1] * s);
            }
        }
        ElementShape::Hex20 => {
            for (i, c) in HEX_CORNERS.iter().enumerate() {
                n[i] = 0.125
                    * (1.0 + c[0] * r)
                    * (1.0 + c[1] * s)
                    * (1.0 + c[2] * t)
                    * (c[0] * r + c[1] * s + c[2] * t - 2.0);
            }
            for (k, [a, b]) in HEX_EDGES.iter().enumerate() {
                let m: [f64; 3] =
                    std::array::from_fn(|i| 0.5 * (HEX_CORNERS[*a][i] + HEX_CORNERS[*b][i]));
                let factor = |x: f64, mi: f64| {
                    if mi == 0.0 { 1.0 - x * x } else { 1.0 + mi * x }
                };
                n[8 + k] = 0.25 * factor(r, m[0]) * factor(s, m[1]) * factor(t, m[2]);
            }
        }
        ElementShape::Quad8 => {
            for (i, c) in HEX_CORNERS.iter().take(4).enumerate() {
                n[i] = 0.25 * (1.0 + c[0] * r) * (1.0 + c[1] * s) * (c[0] * r + c[1] * s - 1.0);
            }
            for (k, [a, b]) in [[0, 1], [1, 2], [2, 3], [3, 0]].iter().enumerate() {
                let m: [f64; 2] =
                    std::array::from_fn(|i| 0.5 * (HEX_CORNERS[*a][i] + HEX_CORNERS[*b][i]));
                n[4 + k] = if m[0] == 0.0 {
                    0.5 * (1.0 - r * r) * (1.0 + m[1] * s)
                } else {
                    0.5 * (1.0 + m[0] * r) * (1.0 - s * s)
                };
            }
        }
        ElementShape::Line2 | ElementShape::Line3 => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn element(shape: ElementShape, type_name: &str, coords: &[[f64; 3]]) -> (FeMesh, Element) {
        let mut mesh = FeMesh::default();
        for (i, c) in coords.iter().enumerate() {
            mesh.set_node(i as u32 + 1, *c);
        }
        let element = Element {
            id: 1,
            type_name: type_name.into(),
            shape,
            nodes: (1..=coords.len() as u32).collect(),
        };
        (mesh, element)
    }

    fn mid(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
        std::array::from_fn(|i| 0.5 * (a[i] + b[i]))
    }

    fn quadratic(corners: &[[f64; 3]], edges: &[[usize; 2]]) -> Vec<[f64; 3]> {
        let mut coords = corners.to_vec();
        coords.extend(edges.iter().map(|[a, b]| mid(corners[*a], corners[*b])));
        coords
    }

    fn unit_hex() -> Vec<[f64; 3]> {
        HEX_CORNERS
            .iter()
            .map(|c| c.map(|x| 0.5 * (x + 1.0)))
            .collect()
    }

    const TET: [[f64; 3]; 4] = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];

    fn check(shape: ElementShape, type_name: &str, coords: &[[f64; 3]]) -> f64 {
        let (mesh, element) = element(shape, type_name, coords);
        element.min_jacobian(&mesh).unwrap()
    }

    #[test]
    fn regular_elements_have_their_volume_ratio_as_determinant() {
        // The reference tetrahedron and the unit cube map with a constant Jacobian.
        assert!((check(ElementShape::Tet4, "C3D4", &TET) - 1.0).abs() < 1e-6);
        let tet10 = quadratic(&TET, &TET_EDGES);
        assert!((check(ElementShape::Tet10, "C3D10", &tet10) - 1.0).abs() < 1e-6);
        let hex = unit_hex();
        assert!((check(ElementShape::Hex8, "C3D8", &hex) - 0.125).abs() < 1e-6);
        let hex20 = quadratic(&hex, &HEX_EDGES);
        assert!((check(ElementShape::Hex20, "C3D20", &hex20) - 0.125).abs() < 1e-6);
        let wedge = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 2.0],
            [1.0, 0.0, 2.0],
            [0.0, 1.0, 2.0],
        ];
        assert!((check(ElementShape::Wedge6, "C3D6", &wedge) - 1.0).abs() < 1e-6);
        let wedge15 = quadratic(&wedge, &WEDGE_EDGES);
        assert!((check(ElementShape::Wedge15, "C3D15", &wedge15) - 1.0).abs() < 1e-6);
        let quad = &unit_hex()[..4];
        assert!((check(ElementShape::Quad4, "CPS4", quad) - 0.25).abs() < 1e-6);
        let quad8 = quadratic(quad, &[[0, 1], [1, 2], [2, 3], [3, 0]]);
        assert!((check(ElementShape::Quad8, "CPS8", &quad8) - 0.25).abs() < 1e-6);
        let tri6 = quadratic(&TET[..3], &[[0, 1], [1, 2], [2, 0]]);
        assert!((check(ElementShape::Tri6, "CPE6", &tri6) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn inverted_and_folded_elements_are_found() {
        let mut inverted = TET;
        inverted.swap(1, 2);
        assert!(check(ElementShape::Tet4, "C3D4", &inverted) < 0.0);
        // A midside node pulled past the opposite corner folds the element.
        let mut tet10 = quadratic(&TET, &TET_EDGES);
        tet10[4] = [1.5, 1.5, 1.5];
        assert!(check(ElementShape::Tet10, "C3D10", &tet10) <= 0.0);
        // A node of the cube moved through the opposite face, as in the CalculiX test.
        let mut hex = unit_hex();
        hex[6] = [-1.0, -1.0, -1.0];
        assert!(check(ElementShape::Hex8, "C3D8", &hex) <= 0.0);
        // A clockwise plane element.
        let mut quad = unit_hex()[..4].to_vec();
        quad.swap(1, 3);
        assert!(check(ElementShape::Quad4, "CPS4", &quad) < 0.0);
    }

    #[test]
    fn shells_and_beams_are_not_checked() {
        let (mesh, element) = element(ElementShape::Quad4, "S4", &unit_hex()[..4]);
        assert_eq!(element.min_jacobian(&mesh), None);
    }
}
