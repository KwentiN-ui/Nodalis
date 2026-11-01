//! Shape functions, integration rules and face tables of the element families, at one
//! place for everything that needs them (stiffness, loads, extrapolation, later contact).
//!
//! Node numbering and natural coordinates follow the CalculiX manual (section "Element
//! types"): hexahedra with corners 1-4 at ζ = -1 and 5-8 at ζ = +1, midside nodes 9-12 on the
//! bottom, 13-16 on the top, 17-20 on the vertical edges; tetrahedra with the midside nodes
//! 5 (1-2), 6 (2-3), 7 (3-1), 8 (1-4), 9 (2-4), 10 (3-4); wedges with 1-3 at the bottom, 4-6
//! at the top, 7-9 bottom edges, 10-12 top edges, 13-15 vertical edges. Faces are numbered
//! S1..S6 as in the manual. Formulas are the textbook ones (Zienkiewicz & Taylor, The Finite
//! Element Method, chapter 8; Hughes, The Finite Element Method, chapter 3).

/// The geometric families of solid elements.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Family {
    Hex8,
    Hex20,
    Tet4,
    Tet10,
    Wedge6,
    Wedge15,
}

impl Family {
    pub fn node_count(self) -> usize {
        match self {
            Family::Hex8 => 8,
            Family::Hex20 => 20,
            Family::Tet4 => 4,
            Family::Tet10 => 10,
            Family::Wedge6 => 6,
            Family::Wedge15 => 15,
        }
    }

    /// Natural coordinates of the nodes.
    pub fn node_coordinates(self) -> &'static [[f64; 3]] {
        match self {
            Family::Hex8 => &HEX8_NODES,
            Family::Hex20 => &HEX20_NODES,
            Family::Tet4 => &TET4_NODES,
            Family::Tet10 => &TET10_NODES,
            Family::Wedge6 => &WEDGE6_NODES,
            Family::Wedge15 => &WEDGE15_NODES,
        }
    }

    /// Shape functions and their derivatives with respect to the natural coordinates at a
    /// point: `n[i]` and `dn[i] = [dN_i/dξ, dN_i/dη, dN_i/dζ]`.
    pub fn shape(self, p: [f64; 3], n: &mut [f64], dn: &mut [[f64; 3]]) {
        match self {
            Family::Hex8 => hex8(p, n, dn),
            Family::Hex20 => hex20(p, n, dn),
            Family::Tet4 => tet4(p, n, dn),
            Family::Tet10 => tet10(p, n, dn),
            Family::Wedge6 => wedge6(p, n, dn),
            Family::Wedge15 => wedge15(p, n, dn),
        }
    }

    /// Faces in CalculiX order (S1 first): the local node indices of each face, corners
    /// first (counter-clockwise seen from outside the element), then the midside nodes
    /// between consecutive corners.
    pub fn faces(self) -> &'static [Face] {
        match self {
            Family::Hex8 | Family::Hex20 => &HEX_FACES,
            Family::Tet4 | Family::Tet10 => &TET_FACES,
            Family::Wedge6 | Family::Wedge15 => &WEDGE_FACES,
        }
    }

    pub fn is_quadratic(self) -> bool {
        matches!(self, Family::Hex20 | Family::Tet10 | Family::Wedge15)
    }
}

/// A face of a solid element.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Face {
    pub corners: &'static [usize],
    pub mids: &'static [usize],
}

const fn face(corners: &'static [usize], mids: &'static [usize]) -> Face {
    Face { corners, mids }
}

/// Faces as the CalculiX manual lists them for C3D8/C3D20: S1 = 1-2-3-4, S2 = 5-8-7-6,
/// S3 = 1-5-6-2, S4 = 2-6-7-3, S5 = 3-7-8-4, S6 = 4-8-5-1 (node numbers 1-based; here
/// 0-based). The corner order of S1 here is reversed against the manual so that every face
/// is counter-clockwise from outside.
pub const HEX_FACES: [Face; 6] = [
    face(&[0, 3, 2, 1], &[11, 10, 9, 8]),
    face(&[4, 5, 6, 7], &[12, 13, 14, 15]),
    face(&[0, 1, 5, 4], &[8, 17, 12, 16]),
    face(&[1, 2, 6, 5], &[9, 18, 13, 17]),
    face(&[2, 3, 7, 6], &[10, 19, 14, 18]),
    face(&[3, 0, 4, 7], &[11, 16, 15, 19]),
];

/// C3D4/C3D10: S1 = 1-2-3, S2 = 1-4-2, S3 = 2-4-3, S4 = 3-4-1.
pub const TET_FACES: [Face; 4] = [
    face(&[0, 2, 1], &[6, 5, 4]),
    face(&[0, 1, 3], &[4, 8, 7]),
    face(&[1, 2, 3], &[5, 9, 8]),
    face(&[2, 0, 3], &[6, 7, 9]),
];

/// C3D6/C3D15: S1 = 1-2-3, S2 = 4-5-6, S3 = 1-2-5-4, S4 = 2-3-6-5, S5 = 3-1-4-6.
pub const WEDGE_FACES: [Face; 5] = [
    face(&[0, 2, 1], &[8, 7, 6]),
    face(&[3, 4, 5], &[9, 10, 11]),
    face(&[0, 1, 4, 3], &[6, 13, 9, 12]),
    face(&[1, 2, 5, 4], &[7, 14, 10, 13]),
    face(&[2, 0, 3, 5], &[8, 12, 11, 14]),
];

const HEX8_NODES: [[f64; 3]; 8] = [
    [-1.0, -1.0, -1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, -1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [1.0, -1.0, 1.0],
    [1.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0],
];

const HEX20_NODES: [[f64; 3]; 20] = [
    [-1.0, -1.0, -1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, -1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [1.0, -1.0, 1.0],
    [1.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0],
    [0.0, -1.0, -1.0],
    [1.0, 0.0, -1.0],
    [0.0, 1.0, -1.0],
    [-1.0, 0.0, -1.0],
    [0.0, -1.0, 1.0],
    [1.0, 0.0, 1.0],
    [0.0, 1.0, 1.0],
    [-1.0, 0.0, 1.0],
    [-1.0, -1.0, 0.0],
    [1.0, -1.0, 0.0],
    [1.0, 1.0, 0.0],
    [-1.0, 1.0, 0.0],
];

const TET4_NODES: [[f64; 3]; 4] = [
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
];

const TET10_NODES: [[f64; 3]; 10] = [
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.5, 0.0, 0.0],
    [0.5, 0.5, 0.0],
    [0.0, 0.5, 0.0],
    [0.0, 0.0, 0.5],
    [0.5, 0.0, 0.5],
    [0.0, 0.5, 0.5],
];

const WEDGE6_NODES: [[f64; 3]; 6] = [
    [0.0, 0.0, -1.0],
    [1.0, 0.0, -1.0],
    [0.0, 1.0, -1.0],
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 1.0],
    [0.0, 1.0, 1.0],
];

const WEDGE15_NODES: [[f64; 3]; 15] = [
    [0.0, 0.0, -1.0],
    [1.0, 0.0, -1.0],
    [0.0, 1.0, -1.0],
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 1.0],
    [0.0, 1.0, 1.0],
    [0.5, 0.0, -1.0],
    [0.5, 0.5, -1.0],
    [0.0, 0.5, -1.0],
    [0.5, 0.0, 1.0],
    [0.5, 0.5, 1.0],
    [0.0, 0.5, 1.0],
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
];

fn hex8(p: [f64; 3], n: &mut [f64], dn: &mut [[f64; 3]]) {
    let [x, y, z] = p;
    for (i, c) in HEX8_NODES.iter().enumerate() {
        let (a, b, d) = (1.0 + c[0] * x, 1.0 + c[1] * y, 1.0 + c[2] * z);
        n[i] = 0.125 * a * b * d;
        dn[i] = [
            0.125 * c[0] * b * d,
            0.125 * c[1] * a * d,
            0.125 * c[2] * a * b,
        ];
    }
}

fn hex20(p: [f64; 3], n: &mut [f64], dn: &mut [[f64; 3]]) {
    let [x, y, z] = p;
    for (i, c) in HEX20_NODES.iter().enumerate() {
        let (a, b, d) = (1.0 + c[0] * x, 1.0 + c[1] * y, 1.0 + c[2] * z);
        if i < 8 {
            let s = c[0] * x + c[1] * y + c[2] * z - 2.0;
            n[i] = 0.125 * a * b * d * s;
            dn[i] = [
                0.125 * c[0] * b * d * (s + a),
                0.125 * c[1] * a * d * (s + b),
                0.125 * c[2] * a * b * (s + d),
            ];
        } else if c[0] == 0.0 {
            n[i] = 0.25 * (1.0 - x * x) * b * d;
            dn[i] = [
                -0.5 * x * b * d,
                0.25 * (1.0 - x * x) * c[1] * d,
                0.25 * (1.0 - x * x) * b * c[2],
            ];
        } else if c[1] == 0.0 {
            n[i] = 0.25 * a * (1.0 - y * y) * d;
            dn[i] = [
                0.25 * c[0] * (1.0 - y * y) * d,
                -0.5 * y * a * d,
                0.25 * a * (1.0 - y * y) * c[2],
            ];
        } else {
            n[i] = 0.25 * a * b * (1.0 - z * z);
            dn[i] = [
                0.25 * c[0] * b * (1.0 - z * z),
                0.25 * a * c[1] * (1.0 - z * z),
                -0.5 * z * a * b,
            ];
        }
    }
}

fn tet4(p: [f64; 3], n: &mut [f64], dn: &mut [[f64; 3]]) {
    let [x, y, z] = p;
    n[0] = 1.0 - x - y - z;
    n[1] = x;
    n[2] = y;
    n[3] = z;
    dn[0] = [-1.0, -1.0, -1.0];
    dn[1] = [1.0, 0.0, 0.0];
    dn[2] = [0.0, 1.0, 0.0];
    dn[3] = [0.0, 0.0, 1.0];
}

fn tet10(p: [f64; 3], n: &mut [f64], dn: &mut [[f64; 3]]) {
    let [x, y, z] = p;
    // Volume coordinates and their gradients.
    let l = [1.0 - x - y - z, x, y, z];
    let dl: [[f64; 3]; 4] = [
        [-1.0, -1.0, -1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];
    for i in 0..4 {
        n[i] = l[i] * (2.0 * l[i] - 1.0);
        for k in 0..3 {
            dn[i][k] = (4.0 * l[i] - 1.0) * dl[i][k];
        }
    }
    const EDGES: [(usize, usize); 6] = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)];
    for (m, &(a, b)) in EDGES.iter().enumerate() {
        n[4 + m] = 4.0 * l[a] * l[b];
        for k in 0..3 {
            dn[4 + m][k] = 4.0 * (dl[a][k] * l[b] + l[a] * dl[b][k]);
        }
    }
}

fn wedge6(p: [f64; 3], n: &mut [f64], dn: &mut [[f64; 3]]) {
    let [x, y, z] = p;
    let l = [1.0 - x - y, x, y];
    let dl: [[f64; 2]; 3] = [[-1.0, -1.0], [1.0, 0.0], [0.0, 1.0]];
    for i in 0..3 {
        let (lo, hi) = (0.5 * (1.0 - z), 0.5 * (1.0 + z));
        n[i] = l[i] * lo;
        n[i + 3] = l[i] * hi;
        dn[i] = [dl[i][0] * lo, dl[i][1] * lo, -0.5 * l[i]];
        dn[i + 3] = [dl[i][0] * hi, dl[i][1] * hi, 0.5 * l[i]];
    }
}

fn wedge15(p: [f64; 3], n: &mut [f64], dn: &mut [[f64; 3]]) {
    let [x, y, z] = p;
    let l = [1.0 - x - y, x, y];
    let dl: [[f64; 2]; 3] = [[-1.0, -1.0], [1.0, 0.0], [0.0, 1.0]];
    let (lo, hi, mid) = (0.5 * (1.0 - z), 0.5 * (1.0 + z), 1.0 - z * z);
    // Corners: N = L (2L - 1) (1 ∓ ζ)/2 - L (1 - ζ²)/2.
    for i in 0..3 {
        let li = l[i];
        n[i] = li * (2.0 * li - 1.0) * lo - 0.5 * li * mid;
        n[i + 3] = li * (2.0 * li - 1.0) * hi - 0.5 * li * mid;
        for k in 0..2 {
            dn[i][k] = dl[i][k] * ((4.0 * li - 1.0) * lo - 0.5 * mid);
            dn[i + 3][k] = dl[i][k] * ((4.0 * li - 1.0) * hi - 0.5 * mid);
        }
        dn[i][2] = -0.5 * li * (2.0 * li - 1.0) + li * z;
        dn[i + 3][2] = 0.5 * li * (2.0 * li - 1.0) + li * z;
    }
    // Edge midsides in the triangles: N = 2 L_a L_b (1 ∓ ζ).
    const EDGES: [(usize, usize); 3] = [(0, 1), (1, 2), (2, 0)];
    for (m, &(a, b)) in EDGES.iter().enumerate() {
        let lab = l[a] * l[b];
        n[6 + m] = 2.0 * lab * (1.0 - z);
        n[9 + m] = 2.0 * lab * (1.0 + z);
        for k in 0..2 {
            let d = dl[a][k] * l[b] + l[a] * dl[b][k];
            dn[6 + m][k] = 2.0 * d * (1.0 - z);
            dn[9 + m][k] = 2.0 * d * (1.0 + z);
        }
        dn[6 + m][2] = -2.0 * lab;
        dn[9 + m][2] = 2.0 * lab;
    }
    // Vertical midsides: N = L (1 - ζ²).
    for i in 0..3 {
        n[12 + i] = l[i] * mid;
        dn[12 + i] = [dl[i][0] * mid, dl[i][1] * mid, -2.0 * z * l[i]];
    }
}

/// An integration rule: points in natural coordinates with their weights.
#[derive(Clone, Debug, PartialEq)]
pub struct IntegrationRule {
    pub points: Vec<[f64; 3]>,
    pub weights: Vec<f64>,
}

impl IntegrationRule {
    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Gauss-Legendre `order`^3 on the cube `[-1, 1]^3`.
    pub fn hex(order: usize) -> Self {
        let (p, w) = gauss_legendre(order);
        let mut points = Vec::with_capacity(order * order * order);
        let mut weights = Vec::with_capacity(order * order * order);
        // Innermost loop over ξ, as the tables of the manuals do.
        for k in 0..order {
            for j in 0..order {
                for i in 0..order {
                    points.push([p[i], p[j], p[k]]);
                    weights.push(w[i] * w[j] * w[k]);
                }
            }
        }
        Self { points, weights }
    }

    /// One point at the centre of the cube, weight 8.
    pub fn hex_reduced() -> Self {
        Self {
            points: vec![[0.0; 3]],
            weights: vec![8.0],
        }
    }

    /// One point at the centroid of the tetrahedron.
    pub fn tet1() -> Self {
        Self {
            points: vec![[0.25; 3]],
            weights: vec![1.0 / 6.0],
        }
    }

    /// Four points of the tetrahedron (degree 2).
    pub fn tet4() -> Self {
        let a = 0.585_410_196_624_968_5;
        let b = 0.138_196_601_125_010_5;
        Self {
            points: vec![[b, b, b], [a, b, b], [b, a, b], [b, b, a]],
            weights: vec![1.0 / 24.0; 4],
        }
    }

    /// Two points of the wedge: triangle centroid, ζ = ±1/√3.
    pub fn wedge2() -> Self {
        let g = 1.0 / 3.0_f64.sqrt();
        Self {
            points: vec![[1.0 / 3.0, 1.0 / 3.0, -g], [1.0 / 3.0, 1.0 / 3.0, g]],
            weights: vec![0.5, 0.5],
        }
    }

    /// Nine points of the wedge: three in the triangle times three along ζ.
    pub fn wedge9() -> Self {
        let (p, w) = gauss_legendre(3);
        let tri = [
            [1.0 / 6.0, 1.0 / 6.0],
            [2.0 / 3.0, 1.0 / 6.0],
            [1.0 / 6.0, 2.0 / 3.0],
        ];
        let mut points = Vec::new();
        let mut weights = Vec::new();
        for k in 0..3 {
            for t in &tri {
                points.push([t[0], t[1], p[k]]);
                weights.push(w[k] / 6.0);
            }
        }
        Self { points, weights }
    }

    /// Fourteen points of the tetrahedron (degree 5, positive weights; Keast 1986), for
    /// the consistent mass matrix of the quadratic tetrahedron whose integrand is of
    /// degree four.
    pub fn tet14() -> Self {
        // Three orbits: two of four points (λ, λ, λ, 1 - 3λ) and one of six points
        // (λ, λ, 1/2 - λ, 1/2 - λ); weights for the reference volume 1/6.
        let orbit4 = |l: f64, w: f64, points: &mut Vec<[f64; 3]>, weights: &mut Vec<f64>| {
            let m = 1.0 - 3.0 * l;
            points.extend([[l, l, l], [m, l, l], [l, m, l], [l, l, m]]);
            weights.extend([w / 6.0; 4]);
        };
        let orbit6 = |l: f64, w: f64, points: &mut Vec<[f64; 3]>, weights: &mut Vec<f64>| {
            let m = 0.5 - l;
            points.extend([
                [l, l, m],
                [l, m, l],
                [m, l, l],
                [l, m, m],
                [m, l, m],
                [m, m, l],
            ]);
            weights.extend([w / 6.0; 6]);
        };
        let mut points = Vec::with_capacity(14);
        let mut weights = Vec::with_capacity(14);
        orbit4(
            0.310_885_919_263_300_6,
            0.112_687_925_718_015_7,
            &mut points,
            &mut weights,
        );
        orbit4(
            0.092_735_250_310_891_2,
            0.073_493_043_116_361_95,
            &mut points,
            &mut weights,
        );
        orbit6(
            0.454_496_295_874_350_4,
            0.042_546_020_777_081_47,
            &mut points,
            &mut weights,
        );
        Self { points, weights }
    }

    /// Eighteen points of the wedge: the six-point triangle rule of degree four (Dunavant
    /// 1985) times three Gauss points along ζ, for the consistent mass matrix of the
    /// quadratic wedge.
    pub fn wedge18() -> Self {
        let (p, w) = gauss_legendre(3);
        let (a, wa) = (0.445_948_490_915_965, 0.223_381_589_678_011);
        let (b, wb) = (0.091_576_213_509_771, 0.109_951_743_655_322);
        let tri = [
            ([a, a], wa),
            ([1.0 - 2.0 * a, a], wa),
            ([a, 1.0 - 2.0 * a], wa),
            ([b, b], wb),
            ([1.0 - 2.0 * b, b], wb),
            ([b, 1.0 - 2.0 * b], wb),
        ];
        let mut points = Vec::with_capacity(18);
        let mut weights = Vec::with_capacity(18);
        for k in 0..3 {
            for (t, wt) in &tri {
                points.push([t[0], t[1], p[k]]);
                weights.push(w[k] * wt / 2.0);
            }
        }
        Self { points, weights }
    }
}

/// Gauss-Legendre points and weights on `[-1, 1]` for orders 1 to 3.
pub fn gauss_legendre(order: usize) -> (Vec<f64>, Vec<f64>) {
    match order {
        1 => (vec![0.0], vec![2.0]),
        2 => {
            let g = 1.0 / 3.0_f64.sqrt();
            (vec![-g, g], vec![1.0, 1.0])
        }
        3 => {
            let g = (0.6_f64).sqrt();
            (vec![-g, 0.0, g], vec![5.0 / 9.0, 8.0 / 9.0, 5.0 / 9.0])
        }
        _ => panic!("Gauss-Legendre order {order} not tabulated"),
    }
}

/// Shape functions of a face (a quadrilateral with 4 or 8 nodes, a triangle with 3 or 6)
/// at a point of its natural coordinates: values and derivatives with respect to the two
/// face coordinates. Node order: corners, then midside nodes between consecutive corners.
pub fn face_shape(
    corners: usize,
    quadratic: bool,
    p: [f64; 2],
    n: &mut [f64],
    dn: &mut [[f64; 2]],
) {
    let [x, y] = p;
    if corners == 4 {
        const C: [[f64; 2]; 4] = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
        if !quadratic {
            for (i, c) in C.iter().enumerate() {
                n[i] = 0.25 * (1.0 + c[0] * x) * (1.0 + c[1] * y);
                dn[i] = [
                    0.25 * c[0] * (1.0 + c[1] * y),
                    0.25 * c[1] * (1.0 + c[0] * x),
                ];
            }
        } else {
            for (i, c) in C.iter().enumerate() {
                let (a, b) = (1.0 + c[0] * x, 1.0 + c[1] * y);
                let s = c[0] * x + c[1] * y - 1.0;
                n[i] = 0.25 * a * b * s;
                dn[i] = [0.25 * c[0] * b * (s + a), 0.25 * c[1] * a * (s + b)];
            }
            const M: [[f64; 2]; 4] = [[0.0, -1.0], [1.0, 0.0], [0.0, 1.0], [-1.0, 0.0]];
            for (m, c) in M.iter().enumerate() {
                let i = 4 + m;
                if c[0] == 0.0 {
                    n[i] = 0.5 * (1.0 - x * x) * (1.0 + c[1] * y);
                    dn[i] = [-x * (1.0 + c[1] * y), 0.5 * (1.0 - x * x) * c[1]];
                } else {
                    n[i] = 0.5 * (1.0 + c[0] * x) * (1.0 - y * y);
                    dn[i] = [0.5 * c[0] * (1.0 - y * y), -y * (1.0 + c[0] * x)];
                }
            }
        }
    } else {
        let l = [1.0 - x - y, x, y];
        let dl: [[f64; 2]; 3] = [[-1.0, -1.0], [1.0, 0.0], [0.0, 1.0]];
        if !quadratic {
            n[..3].copy_from_slice(&l);
            dn[..3].copy_from_slice(&dl);
        } else {
            for i in 0..3 {
                n[i] = l[i] * (2.0 * l[i] - 1.0);
                dn[i] = [(4.0 * l[i] - 1.0) * dl[i][0], (4.0 * l[i] - 1.0) * dl[i][1]];
            }
            const EDGES: [(usize, usize); 3] = [(0, 1), (1, 2), (2, 0)];
            for (m, &(a, b)) in EDGES.iter().enumerate() {
                n[3 + m] = 4.0 * l[a] * l[b];
                dn[3 + m] = [
                    4.0 * (dl[a][0] * l[b] + l[a] * dl[b][0]),
                    4.0 * (dl[a][1] * l[b] + l[a] * dl[b][1]),
                ];
            }
        }
    }
}

/// Integration points of a face: Gauss 2x2 or 3x3 on a quadrilateral, 3 or 7 points on a
/// triangle (degrees 2 and 5).
pub fn face_rule(corners: usize, quadratic: bool) -> (Vec<[f64; 2]>, Vec<f64>) {
    if corners == 4 {
        let (p, w) = gauss_legendre(if quadratic { 3 } else { 2 });
        let mut points = Vec::new();
        let mut weights = Vec::new();
        for j in 0..p.len() {
            for i in 0..p.len() {
                points.push([p[i], p[j]]);
                weights.push(w[i] * w[j]);
            }
        }
        (points, weights)
    } else if !quadratic {
        (
            vec![
                [1.0 / 6.0, 1.0 / 6.0],
                [2.0 / 3.0, 1.0 / 6.0],
                [1.0 / 6.0, 2.0 / 3.0],
            ],
            vec![1.0 / 6.0; 3],
        )
    } else {
        // Radon's 7-point rule, degree 5.
        let a = 0.470_142_064_105_115;
        let b = 0.101_286_507_323_456;
        let wa = 0.132_394_152_788_506 / 2.0;
        let wb = 0.125_939_180_544_827 / 2.0;
        (
            vec![
                [1.0 / 3.0, 1.0 / 3.0],
                [a, a],
                [1.0 - 2.0 * a, a],
                [a, 1.0 - 2.0 * a],
                [b, b],
                [1.0 - 2.0 * b, b],
                [b, 1.0 - 2.0 * b],
            ],
            vec![0.225 / 2.0, wa, wa, wa, wb, wb, wb],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn check_partition_and_nodes(family: Family) {
        let count = family.node_count();
        let mut n = vec![0.0; count];
        let mut dn = vec![[0.0; 3]; count];
        // Kronecker property at the nodes.
        for (i, &p) in family.node_coordinates().iter().enumerate() {
            family.shape(p, &mut n, &mut dn);
            for (j, &v) in n.iter().enumerate() {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (v - expected).abs() < 1e-12,
                    "{family:?} N{j} at node {i}: {v}"
                );
            }
        }
        // Partition of unity and zero derivative sum at an interior point.
        let p = match family {
            Family::Hex8 | Family::Hex20 => [0.3, -0.2, 0.6],
            _ => [0.2, 0.3, 0.1],
        };
        family.shape(p, &mut n, &mut dn);
        assert!((n.iter().sum::<f64>() - 1.0).abs() < 1e-12, "{family:?}");
        for k in 0..3 {
            let s: f64 = dn.iter().map(|d| d[k]).sum();
            assert!(s.abs() < 1e-12, "{family:?} derivative sum {k}: {s}");
        }
        // Derivatives against finite differences.
        let h = 1e-6;
        for k in 0..3 {
            let mut pp = p;
            pp[k] += h;
            let mut np = vec![0.0; count];
            let mut dd = vec![[0.0; 3]; count];
            family.shape(pp, &mut np, &mut dd);
            let mut pm = p;
            pm[k] -= h;
            let mut nm = vec![0.0; count];
            family.shape(pm, &mut nm, &mut dd);
            for i in 0..count {
                let fd = (np[i] - nm[i]) / (2.0 * h);
                assert!(
                    (fd - dn[i][k]).abs() < 1e-7,
                    "{family:?} dN{i}/d{k}: {} vs {fd}",
                    dn[i][k]
                );
            }
        }
    }

    #[test]
    fn shape_functions_interpolate() {
        for family in [
            Family::Hex8,
            Family::Hex20,
            Family::Tet4,
            Family::Tet10,
            Family::Wedge6,
            Family::Wedge15,
        ] {
            check_partition_and_nodes(family);
        }
    }

    #[test]
    fn rules_integrate_the_volume() {
        let volume = |r: &IntegrationRule| r.weights.iter().sum::<f64>();
        assert!((volume(&IntegrationRule::hex(2)) - 8.0).abs() < 1e-12);
        assert!((volume(&IntegrationRule::hex(3)) - 8.0).abs() < 1e-12);
        assert!((volume(&IntegrationRule::tet4()) - 1.0 / 6.0).abs() < 1e-12);
        assert!((volume(&IntegrationRule::wedge2()) - 1.0).abs() < 1e-12);
        assert!((volume(&IntegrationRule::wedge9()) - 1.0).abs() < 1e-12);
        let (_, w) = face_rule(3, true);
        assert!((w.iter().sum::<f64>() - 0.5).abs() < 1e-12);
        let (_, w) = face_rule(4, true);
        assert!((w.iter().sum::<f64>() - 4.0).abs() < 1e-12);
    }

    #[test]
    fn faces_are_outward() {
        // Every face's corner order gives an outward normal (cross product of the first two
        // edges points away from the centroid) on the reference element.
        for family in [Family::Hex20, Family::Tet10, Family::Wedge15] {
            let nodes = family.node_coordinates();
            let centroid = nodes[..family.faces()[0].corners.len().max(4)]
                .iter()
                .fold([0.0; 3], |acc, n| {
                    [acc[0] + n[0], acc[1] + n[1], acc[2] + n[2]]
                });
            let count = match family {
                Family::Hex20 => 8.0,
                Family::Tet10 => 4.0,
                Family::Wedge15 => 6.0,
                _ => unreachable!(),
            };
            let centroid = centroid.map(|c| c / count);
            for (f, face) in family.faces().iter().enumerate() {
                let c = face.corners;
                let a = nodes[c[0]];
                let b = nodes[c[1]];
                let d = nodes[c[2]];
                let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                let e2 = [d[0] - a[0], d[1] - a[1], d[2] - a[2]];
                let n = [
                    e1[1] * e2[2] - e1[2] * e2[1],
                    e1[2] * e2[0] - e1[0] * e2[2],
                    e1[0] * e2[1] - e1[1] * e2[0],
                ];
                let out = [a[0] - centroid[0], a[1] - centroid[1], a[2] - centroid[2]];
                let dot = n[0] * out[0] + n[1] * out[1] + n[2] * out[2];
                assert!(dot > 0.0, "{family:?} face {} is not outward", f + 1);
                // Midside nodes lie between consecutive corners.
                for (m, &mid) in face.mids.iter().enumerate() {
                    let (p, q) = (nodes[c[m]], nodes[c[(m + 1) % c.len()]]);
                    for k in 0..3 {
                        assert!(
                            (nodes[mid][k] - 0.5 * (p[k] + q[k])).abs() < 1e-12,
                            "{family:?} face {} mid {m}",
                            f + 1
                        );
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod rule_tests {
    use super::IntegrationRule;

    fn factorial(n: usize) -> f64 {
        (1..=n).map(|k| k as f64).product()
    }

    fn integrate(rule: &IntegrationRule, a: usize, b: usize, c: usize) -> f64 {
        rule.points
            .iter()
            .zip(&rule.weights)
            .map(|(p, w)| w * p[0].powi(a as i32) * p[1].powi(b as i32) * p[2].powi(c as i32))
            .sum()
    }

    #[test]
    fn tet14_integrates_degree_five_exactly() {
        let rule = IntegrationRule::tet14();
        assert!((rule.weights.iter().sum::<f64>() - 1.0 / 6.0).abs() < 1e-14);
        for a in 0..=5 {
            for b in 0..=5 - a {
                for c in 0..=5 - a - b {
                    let q = integrate(&rule, a, b, c);
                    let exact =
                        factorial(a) * factorial(b) * factorial(c) / factorial(a + b + c + 3);
                    assert!(
                        (q - exact).abs() < 1e-12,
                        "x^{a} y^{b} z^{c}: {q} vs {exact}"
                    );
                }
            }
        }
    }

    #[test]
    fn wedge18_integrates_degree_four_exactly() {
        let rule = IntegrationRule::wedge18();
        assert!((rule.weights.iter().sum::<f64>() - 1.0).abs() < 1e-14);
        for a in 0..=4 {
            for b in 0..=4 - a {
                for c in 0..=4 {
                    let q = integrate(&rule, a, b, c);
                    let tri = factorial(a) * factorial(b) / factorial(a + b + 2);
                    let line = if c % 2 == 0 {
                        2.0 / (c as f64 + 1.0)
                    } else {
                        0.0
                    };
                    let exact = tri * line;
                    assert!(
                        (q - exact).abs() < 1e-12,
                        "x^{a} y^{b} z^{c}: {q} vs {exact}"
                    );
                }
            }
        }
    }
}
