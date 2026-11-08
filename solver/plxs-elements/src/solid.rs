//! Three-dimensional continuum elements: C3D4, C3D10, C3D10T, C3D6, C3D15, C3D8, C3D8R,
//! C3D8I, C3D20, C3D20R. Isoparametric displacement formulation with Gauss integration
//! (Zienkiewicz & Taylor, chapters 5 and 8; Hughes, chapter 3); the incompatible modes of
//! C3D8I after Wilson et al. (Taylor, Beresford & Wilson 1976) with the mode derivatives taken
//! through the centroid Jacobian but, as CalculiX, without the det J0 / det J scaling;
//! condensed at element level.
//!
//! Integration points as the CalculiX manual lists them: C3D8 and C3D20R 2×2×2, C3D20
//! 3×3×3, C3D8R one point, C3D4 one point, C3D10 four points, C3D6 two points, C3D15 nine
//! points. Strains and stresses are computed at these points; `extrapolation` maps them to
//! the nodes.
//!
//! Vector conventions: nodal displacements `[u1x, u1y, u1z, u2x, ...]`; strain and stress
//! as `[xx, yy, zz, xy, yz, zx]` with engineering shear strains (γ) in the element
//! formulation.

use plxs_core::ElementType;

use crate::plane::{Expansion, PlaneKind};
use crate::shape::{Family, IntegrationRule, face_rule, face_shape};

/// How the element integrates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Formulation {
    /// Full displacement formulation.
    Standard,
    /// One integration point (C3D8R).
    Reduced,
    /// Incompatible modes (C3D8I).
    Incompatible,
}

#[derive(Clone, Debug)]
pub struct SolidElement {
    pub kind: ElementType,
    pub family: Family,
    pub rule: IntegrationRule,
    pub formulation: Formulation,
    /// `extrapolation[node * ips + ip]`: integration-point values to nodal values.
    extrapolation: Vec<f64>,
    /// For plane and axisymmetric elements: how the 2D element expands into this 3D one.
    /// The public methods then take and return values of the 2D nodes.
    expansion: Option<Box<Expansion>>,
    /// `dN/dξ` of every node at every point of the rule: constant on the reference
    /// element, so computed once.
    reference_gradients: Vec<Vec<[f64; 3]>>,
}

/// `dN/dξ` of all nodes at every point of `rule`.
fn reference_gradients(family: Family, rule: &IntegrationRule) -> Vec<Vec<[f64; 3]>> {
    let count = family.node_count();
    rule.points
        .iter()
        .map(|&p| {
            let mut n = vec![0.0; count];
            let mut dn = vec![[0.0; 3]; count];
            family.shape(p, &mut n, &mut dn);
            dn
        })
        .collect()
}

/// Nodal values of a plane or axisymmetric element as CalculiX reports them, see
/// [`SolidElement::reported_nodal`].
#[derive(Clone, Debug, PartialEq)]
pub struct ReportedNodal {
    /// Per 2D node.
    pub displacements: Vec<[f64; 3]>,
    /// Per 2D node.
    pub forces: Vec<[f64; 3]>,
    /// The element's contribution to the solver's force vector, 3 per 2D node.
    pub condensed: Vec<f64>,
}

/// The solid element of a type, `None` for types that are no solids.
pub fn solid(kind: ElementType) -> Option<SolidElement> {
    use ElementType::*;
    let (family, rule, formulation) = match kind {
        C3D8 => (Family::Hex8, IntegrationRule::hex(2), Formulation::Standard),
        C3D8R => (
            Family::Hex8,
            IntegrationRule::hex_reduced(),
            Formulation::Reduced,
        ),
        C3D8I => (
            Family::Hex8,
            IntegrationRule::hex(2),
            Formulation::Incompatible,
        ),
        C3D20 => (
            Family::Hex20,
            IntegrationRule::hex(3),
            Formulation::Standard,
        ),
        C3D20R => (
            Family::Hex20,
            IntegrationRule::hex(2),
            Formulation::Standard,
        ),
        C3D4 => (Family::Tet4, IntegrationRule::tet1(), Formulation::Standard),
        C3D10 | C3D10T => (
            Family::Tet10,
            IntegrationRule::tet4(),
            Formulation::Standard,
        ),
        C3D6 => (
            Family::Wedge6,
            IntegrationRule::wedge2(),
            Formulation::Standard,
        ),
        C3D15 => (
            Family::Wedge15,
            IntegrationRule::wedge9(),
            Formulation::Standard,
        ),
        _ => return None,
    };
    let extrapolation = extrapolation_matrix(family, &rule);
    Some(SolidElement {
        kind,
        family,
        reference_gradients: reference_gradients(family, &rule),
        rule,
        formulation,
        extrapolation,
        expansion: None,
    })
}

/// Matrix `nodes × ips` that extrapolates integration-point values to the nodes.
///
/// The schemes reproduce what CalculiX writes, inferred by fitting its nodal stresses of a
/// single distorted element under many load cases against its integration-point stresses
/// (black-box use; no CalculiX source was consulted):
/// * 2×2×2 hexahedra: the trilinear Lagrange polynomial through the Gauss points, with the
///   weights rounded to about four digits as CalculiX tabulates them.
/// * 3×3×3 hexahedra: the least-squares fit of the element's own shape functions through
///   the 27 points (for C3D20 the serendipity functions; the triquadratic Lagrange
///   polynomial differs by percent on distorted elements), rounded to five decimals.
/// * tetrahedra: the linear polynomial through the four points, corner weights rounded to
///   five decimals.
/// * wedges: linear in the triangle; along the axis CalculiX's two-point weights (which
///   alternate with the node number, see below) or the least-squares line through the
///   three planes of the nine-point rule.
/// * one-point rules: the value itself.
fn extrapolation_matrix(family: Family, rule: &IntegrationRule) -> Vec<f64> {
    let nodes = family.node_coordinates();
    let ips = rule.len();
    let mut m = vec![0.0; nodes.len() * ips];
    if matches!(family, Family::Hex8 | Family::Hex20) && ips == 27 {
        // CalculiX tabulates these weights to five decimals.
        return least_squares_extrapolation(family, rule)
            .into_iter()
            .map(|w| (w * 1e5).round() / 1e5)
            .collect();
    }
    for (a, node) in nodes.iter().enumerate() {
        let row = &mut m[a * ips..(a + 1) * ips];
        match (family, ips) {
            (_, 1) => row[0] = 1.0,
            (Family::Hex8 | Family::Hex20, 8) => {
                // CalculiX's tables, indexed by the number of axes on which the node and
                // the point lie on the same side (the trilinear Lagrange values rounded to
                // about four digits).
                const CORNER: [f64; 4] = [-0.04904, 0.183, -0.683, 2.549];
                const MID: [f64; 3] = [0.06698, -0.25, 0.933];
                let axes: Vec<usize> = (0..3).filter(|&k| node[k] != 0.0).collect();
                for (g, p) in rule.points.iter().enumerate() {
                    let same = axes.iter().filter(|&&k| node[k] * p[k] > 0.0).count();
                    row[g] = if axes.len() == 3 {
                        CORNER[same]
                    } else {
                        MID[same]
                    };
                }
            }
            (Family::Tet4 | Family::Tet10, 4) => {
                // The linear polynomial through the four points (point g lies nearest to
                // corner g), with CalculiX's corner weights rounded to five decimals:
                // 1.92705 and -0.30902 instead of 1.927051 and -0.309017. The rows sum to
                // 0.99999, so a uniform stress comes out 1e-5 low, as in ccx. Midside nodes
                // take the mean of their two corners. Fitted to ccx nodal values on
                // distorted elements (residual at the .frd's print resolution).
                const TET_CORNER: [f64; 2] = [1.927_05, -0.309_02];
                let n = [1.0 - node[0] - node[1] - node[2], node[0], node[1], node[2]];
                for (g, w) in row.iter_mut().enumerate() {
                    *w = (0..4).map(|c| n[c] * TET_CORNER[usize::from(c != g)]).sum();
                }
            }
            (Family::Wedge6 | Family::Wedge15, 2) => {
                // CalculiX's weights: 1.1455 and -0.1455 (points at ±√0.6), and the point
                // they favour alternates with the local node number rather than following
                // the node's axial position — reproduced here so the nodal values match.
                let (near, far) = (1.1455, -0.1455);
                let (w0, w1) = if a % 2 == 0 { (near, far) } else { (far, near) };
                row[0] = w0;
                row[1] = w1;
            }
            (Family::Wedge6 | Family::Wedge15, 9) => {
                let a2: f64 = 0.6;
                // Least-squares line through the planes at ζ = -√0.6, 0, √0.6.
                let lz = |x: f64, g: f64| -> f64 {
                    if g < 0.0 {
                        1.0 / 3.0 - x / (2.0 * a2.sqrt())
                    } else if g > 0.0 {
                        1.0 / 3.0 + x / (2.0 * a2.sqrt())
                    } else {
                        1.0 / 3.0
                    }
                };
                let l1 = 2.0 * (node[0] - 1.0 / 6.0);
                let l2 = 2.0 * (node[1] - 1.0 / 6.0);
                let tri = [1.0 - l1 - l2, l1, l2];
                for (g, p) in rule.points.iter().enumerate() {
                    row[g] = tri[g % 3] * lz(node[2], p[2]);
                }
            }
            _ => unreachable!("no extrapolation for {family:?} with {ips} points"),
        }
    }
    m
}

/// Nodal values `v` minimizing `Σ_ip (N(ξ_ip)·v − value_ip)²`: `(NᵀN)⁻¹Nᵀ`.
fn least_squares_extrapolation(family: Family, rule: &IntegrationRule) -> Vec<f64> {
    let n = family.node_count();
    let ips = rule.len();
    let mut shape = vec![0.0; ips * n];
    let mut dn = vec![[0.0; 3]; n];
    for (g, p) in rule.points.iter().enumerate() {
        family.shape(*p, &mut shape[g * n..(g + 1) * n], &mut dn);
    }
    let mut ntn = vec![0.0; n * n];
    for g in 0..ips {
        let row = &shape[g * n..(g + 1) * n];
        for a in 0..n {
            for b in 0..n {
                ntn[a * n + b] += row[a] * row[b];
            }
        }
    }
    // Solve (NᵀN) M = Nᵀ column by column; `solve_dense` takes `b` as `n` columns of length
    // `m`, so pass Nᵀ as an `ips`-column matrix.
    let mut nt = vec![0.0; n * ips];
    for g in 0..ips {
        for a in 0..n {
            nt[a * ips + g] = shape[g * n + a];
        }
    }
    solve_dense(&mut ntn, n, &nt, ips)
}

/// Jacobian, its determinant and the Cartesian shape function derivatives at a point.
pub(crate) struct Point {
    pub(crate) det: f64,
    /// `dndx[i] = [dN_i/dx, dN_i/dy, dN_i/dz]`.
    pub(crate) dndx: Vec<[f64; 3]>,
    pub(crate) n: Vec<f64>,
}

/// The Jacobian determinant and `dN/dx` from `dN/dξ` and the node coordinates.
fn cartesian_gradients(dn: &[[f64; 3]], coords: &[[f64; 3]]) -> (f64, Vec<[f64; 3]>) {
    let j = jacobian(dn, coords);
    let (det, inv) = invert3(&j);
    // dN/dξ_r = Σ_c J_rc dN/dx_c with J_rc = dx_c/dξ_r, so dN/dx_c = Σ_r inv[c][r] dN/dξ_r.
    let dndx = dn
        .iter()
        .map(|d| {
            let mut g = [0.0; 3];
            for (c, gc) in g.iter_mut().enumerate() {
                *gc = d[0] * inv[c][0] + d[1] * inv[c][1] + d[2] * inv[c][2];
            }
            g
        })
        .collect();
    (det, dndx)
}

fn jacobian(dn: &[[f64; 3]], coords: &[[f64; 3]]) -> [[f64; 3]; 3] {
    let mut j = [[0.0; 3]; 3];
    for (d, x) in dn.iter().zip(coords) {
        for (r, row) in j.iter_mut().enumerate() {
            for (c, v) in row.iter_mut().enumerate() {
                *v += d[r] * x[c];
            }
        }
    }
    j
}

fn invert3(j: &[[f64; 3]; 3]) -> (f64, [[f64; 3]; 3]) {
    let det = j[0][0] * (j[1][1] * j[2][2] - j[1][2] * j[2][1])
        - j[0][1] * (j[1][0] * j[2][2] - j[1][2] * j[2][0])
        + j[0][2] * (j[1][0] * j[2][1] - j[1][1] * j[2][0]);
    let inv = [
        [
            (j[1][1] * j[2][2] - j[1][2] * j[2][1]) / det,
            (j[0][2] * j[2][1] - j[0][1] * j[2][2]) / det,
            (j[0][1] * j[1][2] - j[0][2] * j[1][1]) / det,
        ],
        [
            (j[1][2] * j[2][0] - j[1][0] * j[2][2]) / det,
            (j[0][0] * j[2][2] - j[0][2] * j[2][0]) / det,
            (j[0][2] * j[1][0] - j[0][0] * j[1][2]) / det,
        ],
        [
            (j[1][0] * j[2][1] - j[1][1] * j[2][0]) / det,
            (j[0][1] * j[2][0] - j[0][0] * j[2][1]) / det,
            (j[0][0] * j[1][1] - j[0][1] * j[1][0]) / det,
        ],
    ];
    (det, inv)
}

impl SolidElement {
    /// The element with another integration rule and extrapolation
    /// (`extrapolation[node * ips + ip]`), for the section points of expanded pipe beams.
    pub fn with_rule(mut self, rule: IntegrationRule, extrapolation: Vec<f64>) -> Self {
        assert_eq!(extrapolation.len(), rule.len() * self.node_count());
        self.extrapolation = extrapolation;
        self.reference_gradients = reference_gradients(self.family, &rule);
        self.rule = rule;
        self
    }

    pub fn node_count(&self) -> usize {
        match &self.expansion {
            Some(x) => x.nodes,
            None => self.family.node_count(),
        }
    }

    pub(crate) fn set_expansion(&mut self, expansion: Expansion, extrapolation: Vec<f64>) {
        self.expansion = Some(Box::new(expansion));
        self.extrapolation = extrapolation;
    }

    /// The expansion of a plane or axisymmetric element.
    pub fn expansion(&self) -> Option<&Expansion> {
        self.expansion.as_deref()
    }

    /// Whether the third DOF of the element's nodes takes part in the stiffness (not for
    /// plane and axisymmetric elements, whose nodes move in their plane only).
    pub fn uses_out_of_plane(&self) -> bool {
        self.expansion.is_none()
    }

    /// Whether the element is a plane stress element: its material must make the thickness
    /// stress zero (`plxs_materials::plane_stress`).
    pub fn is_plane_stress(&self) -> bool {
        self.expansion.as_deref().map(|x| x.kind) == Some(PlaneKind::Stress)
    }

    /// For plane and axisymmetric elements: displacements and nodal forces (`K u`) of the
    /// 2D nodes as CalculiX reports them (see [`crate::plane`]), and the nodal forces as the
    /// solver's own vector holds them (`Tᵀ K₃ T u`), so that the caller can swap one for
    /// the other. `None` for solids, whose nodes report what they compute.
    pub fn reported_nodal(
        &self,
        coords: &[[f64; 3]],
        d: &[[f64; 6]; 6],
        u: &[f64],
    ) -> Option<ReportedNodal> {
        let x = self.expansion.as_deref()?;
        let coords3 = x.coordinates(coords);
        let mut k3 = Vec::new();
        self.stiffness_3d(&coords3, d, &mut k3);
        let u3 = x.expand(u);
        let n3 = u3.len();
        let f3: Vec<f64> = (0..n3)
            .map(|i| (0..n3).map(|j| k3[i * n3 + j] * u3[j]).sum())
            .collect();
        let mut condensed = vec![0.0; self.dof_count()];
        x.condense_vector(&f3, &mut condensed);
        Some(ReportedNodal {
            displacements: x.reported_displacements(u),
            forces: x.reported_forces(&f3),
            condensed,
        })
    }

    /// Number of faces (edges of 2D elements) that can carry a pressure.
    pub fn face_count(&self) -> usize {
        match &self.expansion {
            Some(x) => x.edges(),
            None => self.family.faces().len(),
        }
    }

    pub fn dof_count(&self) -> usize {
        3 * self.node_count()
    }

    pub fn integration_points(&self) -> usize {
        self.rule.len()
    }

    /// `extrapolation[node * ips + ip]`.
    pub fn extrapolation(&self) -> &[f64] {
        &self.extrapolation
    }

    pub(crate) fn point(&self, p: [f64; 3], coords: &[[f64; 3]]) -> Point {
        let count = self.family.node_count();
        let mut n = vec![0.0; count];
        let mut dn = vec![[0.0; 3]; count];
        self.family.shape(p, &mut n, &mut dn);
        let (det, dndx) = cartesian_gradients(&dn, coords);
        Point { det, dndx, n }
    }

    /// The Jacobian determinant and the Cartesian shape function derivatives at integration
    /// point `ip` of the rule (from the stored reference derivatives).
    pub(crate) fn rule_point(&self, ip: usize, coords: &[[f64; 3]]) -> (f64, Vec<[f64; 3]>) {
        cartesian_gradients(&self.reference_gradients[ip], coords)
    }

    /// Whether a three-dimensional element is turned inside out or folded: its Jacobian
    /// determinant is zero or negative at an integration point. Its stiffness then has
    /// negative eigenvalues, an implicit solution diverges and an explicit one grows without
    /// bound. Plane and axisymmetric elements are not checked (`false`).
    pub fn inverted(&self, coords: &[[f64; 3]]) -> bool {
        self.expansion.is_none()
            && (0..self.rule.points.len()).any(|ip| self.rule_point(ip, coords).0 <= 0.0)
    }

    /// Element volume (of the full circumference for axisymmetric elements).
    pub fn volume(&self, coords: &[[f64; 3]]) -> f64 {
        if let Some(x) = &self.expansion {
            return x.scale * self.volume_3d(&x.coordinates(coords));
        }
        self.volume_3d(coords)
    }

    fn volume_3d(&self, coords: &[[f64; 3]]) -> f64 {
        self.rule
            .points
            .iter()
            .zip(&self.rule.weights)
            .map(|(&p, &w)| self.point(p, coords).det * w)
            .sum()
    }

    /// Fills the strain-displacement matrix `b` (6 × 3n, row-major) from Cartesian
    /// derivatives.
    pub(crate) fn fill_b(dndx: &[[f64; 3]], b: &mut [f64]) {
        let cols = 3 * dndx.len();
        b.iter_mut().for_each(|v| *v = 0.0);
        for (i, g) in dndx.iter().enumerate() {
            let (x, y, z) = (3 * i, 3 * i + 1, 3 * i + 2);
            b[x] = g[0];
            b[cols + y] = g[1];
            b[2 * cols + z] = g[2];
            b[3 * cols + x] = g[1];
            b[3 * cols + y] = g[0];
            b[4 * cols + y] = g[2];
            b[4 * cols + z] = g[1];
            b[5 * cols + x] = g[2];
            b[5 * cols + z] = g[0];
        }
    }

    /// The incompatible-mode strain matrix `g` (6 × 9) at a point: the modes
    /// `1 - ξ²`, `1 - η²`, `1 - ζ²` with the derivatives taken through the Jacobian at
    /// the centroid. CalculiX does not scale them by `det J0 / det J` (Taylor's patch-test
    /// fix): the oracle stiffness of a distorted element matches the unscaled form to 4e-6
    /// and differs from the scaled one by 2e-3, so the scaling is left out.
    fn fill_g(p: [f64; 3], inv0: &[[f64; 3]; 3], g: &mut [f64]) {
        let dndx = Self::mode_gradients(p, inv0);
        Self::fill_b(&dndx, g);
    }

    /// Cartesian derivatives of the three incompatible modes at a point (see `fill_g`).
    pub(crate) fn mode_gradients(p: [f64; 3], inv0: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
        let dmodes = [
            [-2.0 * p[0], 0.0, 0.0],
            [0.0, -2.0 * p[1], 0.0],
            [0.0, 0.0, -2.0 * p[2]],
        ];
        let mut dndx = [[0.0; 3]; 3];
        for (m, d) in dmodes.iter().enumerate() {
            for c in 0..3 {
                dndx[m][c] = d[0] * inv0[c][0] + d[1] * inv0[c][1] + d[2] * inv0[c][2];
            }
        }
        dndx
    }

    /// Stiffness matrix (row-major `3n × 3n`) for the material matrix `d` (6 × 6, acting
    /// on engineering strains).
    pub fn stiffness(&self, coords: &[[f64; 3]], d: &[[f64; 6]; 6], out: &mut Vec<f64>) {
        if let Some(x) = &self.expansion {
            let mut k3 = Vec::new();
            self.stiffness_3d(&x.coordinates(coords), d, &mut k3);
            let n = self.dof_count();
            out.clear();
            out.resize(n * n, 0.0);
            x.condense_matrix(&k3, out);
            return;
        }
        self.stiffness_3d(coords, d, out);
    }

    pub(crate) fn dof_count_3d(&self) -> usize {
        3 * self.family.node_count()
    }

    fn stiffness_3d(&self, coords: &[[f64; 3]], d: &[[f64; 6]; 6], out: &mut Vec<f64>) {
        let ndof = self.dof_count_3d();
        out.clear();
        out.resize(ndof * ndof, 0.0);
        let mut b = vec![0.0; 6 * ndof];
        if self.formulation == Formulation::Incompatible {
            self.stiffness_incompatible(coords, d, out, &mut b);
            return;
        }
        if self.formulation == Formulation::Reduced {
            self.stiffness_reduced(coords, d, out, &mut b);
            return;
        }
        for (&p, &w) in self.rule.points.iter().zip(&self.rule.weights) {
            let point = self.point(p, coords);
            Self::fill_b(&point.dndx, &mut b);
            accumulate_btdb(&b, d, ndof, point.det * w, out);
        }
    }

    /// C3D8R as the uniform-strain hexahedron of Flanagan and Belytschko (1981): the
    /// stiffness of the mean strain-displacement matrix over the element volume (the
    /// 2×2×2 average of B) plus the orthogonal hourglass control with the stiffness
    /// CalculiX uses, which was identified from its output: `ε = 0.004 E V Σ_j b_j·b_j`
    /// per hourglass mode (independent of Poisson's ratio), so that
    /// `K_hg = ε/8 Σ_α γ_α γ_αᵀ ⊗ I`, with CalculiX's hourglass vectors (see below).
    fn stiffness_reduced(
        &self,
        coords: &[[f64; 3]],
        d: &[[f64; 6]; 6],
        out: &mut [f64],
        b: &mut [f64],
    ) {
        let ndof = self.dof_count_3d();
        let (volume, mean) = self.mean_gradients(coords);
        Self::fill_b(&mean, b);
        accumulate_btdb(b, d, ndof, volume, out);
        self.hourglass_stiffness(coords, d, &mean, volume, out);
    }

    /// Adds the hourglass control stiffness of the uniform-strain hexahedron to `out`
    /// (Young's modulus from the isotropic material matrix).
    pub(crate) fn hourglass_stiffness(
        &self,
        coords: &[[f64; 3]],
        d: &[[f64; 6]; 6],
        mean: &[[f64; 3]],
        volume: f64,
        out: &mut [f64],
    ) {
        let ndof = self.dof_count_3d();
        let mu = d[3][3];
        let lambda = d[0][1];
        let young = mu * (3.0 * lambda + 2.0 * mu) / (lambda + mu);
        let bb: f64 = mean
            .iter()
            .map(|m| m[0] * m[0] + m[1] * m[1] + m[2] * m[2])
            .sum();
        let epsilon = 0.004 * young * volume * bb / 8.0;
        const H: [[f64; 8]; 4] = [
            [1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0],
            [1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0],
            [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0],
            [-1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0],
        ];
        for h in &H {
            // Flanagan-Belytschko: γ = h − Σ_j (h·x_j) b_j, orthogonal to the linear
            // displacement fields. CalculiX subtracts the term of the global z coordinate
            // only (j = 3), which was identified from its stiffness matrices of distorted and
            // rotated single elements (fit to the six digits of its output; with all three
            // terms the hourglass stiffness differs by percent on such elements). Linear
            // fields in x and y therefore excite its hourglass control slightly; the same
            // is done here, so that the results match.
            let mut gamma = *h;
            for j in 2..3 {
                let hx: f64 = (0..8).map(|a| h[a] * coords[a][j]).sum();
                for a in 0..8 {
                    gamma[a] -= hx * mean[a][j];
                }
            }
            for a in 0..8 {
                for c in 0..8 {
                    let v = epsilon * gamma[a] * gamma[c];
                    for k in 0..3 {
                        out[(3 * a + k) * ndof + 3 * c + k] += v;
                    }
                }
            }
        }
    }

    /// Volume and volume-averaged Cartesian shape function derivatives (the uniform-strain
    /// `B` of Flanagan and Belytschko), integrated with 2×2×2 points (exact for the
    /// trilinear hexahedron).
    pub(crate) fn mean_gradients(&self, coords: &[[f64; 3]]) -> (f64, Vec<[f64; 3]>) {
        let full = IntegrationRule::hex(2);
        let mut volume = 0.0;
        let mut mean = vec![[0.0; 3]; 8];
        for (&p, &w) in full.points.iter().zip(&full.weights) {
            let point = self.point(p, coords);
            let f = point.det * w;
            volume += f;
            for (m, g) in mean.iter_mut().zip(&point.dndx) {
                for c in 0..3 {
                    m[c] += f * g[c];
                }
            }
        }
        for m in mean.iter_mut() {
            for c in m.iter_mut() {
                *c /= volume;
            }
        }
        (volume, mean)
    }

    /// Condensation data of the incompatible modes: `K_aa^-1 K_au` (9 × 3n) so that the
    /// mode amplitudes are `α = -K_aa^-1 K_au u`.
    pub(crate) fn incompatible_coupling(
        &self,
        coords: &[[f64; 3]],
        d: &[[f64; 6]; 6],
    ) -> (Vec<f64>, [[f64; 3]; 3]) {
        let ndof = self.dof_count_3d();
        let mut b = vec![0.0; 6 * ndof];
        let mut g = vec![0.0; 6 * 9];
        let centre = self.point([0.0; 3], coords);
        let j0 = {
            let mut n = vec![0.0; 8];
            let mut dn = vec![[0.0; 3]; 8];
            self.family.shape([0.0; 3], &mut n, &mut dn);
            jacobian(&dn, coords)
        };
        let (_, inv0) = invert3(&j0);
        let mut kaa = vec![0.0; 81];
        let mut kau = vec![0.0; 9 * ndof];
        for (&p, &w) in self.rule.points.iter().zip(&self.rule.weights) {
            let point = self.point(p, coords);
            Self::fill_b(&point.dndx, &mut b);
            Self::fill_g(p, &inv0, &mut g);
            let f = point.det * w;
            accumulate_btdb(&g, d, 9, f, &mut kaa);
            accumulate_atdb(&g, 9, &b, ndof, d, f, &mut kau);
        }
        let _ = centre;
        // Solve K_aa X = K_au for X (9 × ndof) by Gaussian elimination with pivoting.
        let x = solve_dense(&mut kaa, 9, &kau, ndof);
        (x, inv0)
    }

    fn stiffness_incompatible(
        &self,
        coords: &[[f64; 3]],
        d: &[[f64; 6]; 6],
        out: &mut [f64],
        b: &mut [f64],
    ) {
        let ndof = self.dof_count_3d();
        let (x, inv0) = self.incompatible_coupling(coords, d);
        let mut g = vec![0.0; 6 * 9];
        let mut kau = vec![0.0; 9 * ndof];
        for (&p, &w) in self.rule.points.iter().zip(&self.rule.weights) {
            let point = self.point(p, coords);
            Self::fill_b(&point.dndx, b);
            Self::fill_g(p, &inv0, &mut g);
            let f = point.det * w;
            accumulate_btdb(b, d, ndof, f, out);
            accumulate_atdb(&g, 9, b, ndof, d, f, &mut kau);
        }
        // K = K_uu - K_ua K_aa^-1 K_au = K_uu - K_auᵀ X.
        for i in 0..ndof {
            for j in 0..ndof {
                let mut s = 0.0;
                for m in 0..9 {
                    s += kau[m * ndof + i] * x[m * ndof + j];
                }
                out[i * ndof + j] -= s;
            }
        }
    }

    /// Engineering strains at the integration points for nodal displacements `u`.
    pub fn strains(&self, coords: &[[f64; 3]], d: &[[f64; 6]; 6], u: &[f64]) -> Vec<[f64; 6]> {
        if let Some(x) = &self.expansion {
            let mut e = self.strains_3d(&x.coordinates(coords), d, &x.expand(u));
            if x.kind != PlaneKind::Axisymmetric {
                // No transverse shear in the plane (its value is round-off of the equal
                // in-plane displacements of the layers).
                for e in &mut e {
                    e[4] = 0.0;
                    e[5] = 0.0;
                }
            }
            return e;
        }
        self.strains_3d(coords, d, u)
    }

    fn strains_3d(&self, coords: &[[f64; 3]], d: &[[f64; 6]; 6], u: &[f64]) -> Vec<[f64; 6]> {
        let ndof = self.dof_count_3d();
        let mut b = vec![0.0; 6 * ndof];
        if self.formulation == Formulation::Reduced {
            // The uniform strain of the element (CalculiX's stress at the one point follows it,
            // which differs from B at the centre for hexahedra that are no parallelepipeds).
            let (_, mean) = self.mean_gradients(coords);
            Self::fill_b(&mean, &mut b);
            let e: [f64; 6] =
                std::array::from_fn(|r| (0..ndof).map(|c| b[r * ndof + c] * u[c]).sum());
            return vec![e];
        }
        let incompatible = (self.formulation == Formulation::Incompatible).then(|| {
            let (x, inv0) = self.incompatible_coupling(coords, d);
            let mut alpha = [0.0; 9];
            for (m, a) in alpha.iter_mut().enumerate() {
                *a = -(0..ndof).map(|j| x[m * ndof + j] * u[j]).sum::<f64>();
            }
            (alpha, inv0)
        });
        let mut g = vec![0.0; 6 * 9];
        self.rule
            .points
            .iter()
            .map(|&p| {
                let point = self.point(p, coords);
                Self::fill_b(&point.dndx, &mut b);
                let mut e = [0.0; 6];
                for (r, er) in e.iter_mut().enumerate() {
                    *er = (0..ndof).map(|c| b[r * ndof + c] * u[c]).sum();
                }
                if let Some((alpha, inv0)) = &incompatible {
                    Self::fill_g(p, inv0, &mut g);
                    for (r, er) in e.iter_mut().enumerate() {
                        *er += (0..9).map(|c| g[r * 9 + c] * alpha[c]).sum::<f64>();
                    }
                }
                e
            })
            .collect()
    }

    /// Internal nodal forces `∫ Bᵀ σ dV` for stresses at the integration points.
    pub fn internal_force(&self, coords: &[[f64; 3]], stresses: &[[f64; 6]], out: &mut [f64]) {
        if let Some(x) = &self.expansion {
            let mut f3 = vec![0.0; self.dof_count_3d()];
            self.internal_force_3d(&x.coordinates(coords), stresses, &mut f3);
            out.iter_mut().for_each(|v| *v = 0.0);
            x.condense_vector(&f3, out);
            return;
        }
        self.internal_force_3d(coords, stresses, out);
    }

    fn internal_force_3d(&self, coords: &[[f64; 3]], stresses: &[[f64; 6]], out: &mut [f64]) {
        let ndof = self.dof_count_3d();
        out.iter_mut().for_each(|v| *v = 0.0);
        let mut b = vec![0.0; 6 * ndof];
        if self.formulation == Formulation::Reduced {
            let (volume, mean) = self.mean_gradients(coords);
            Self::fill_b(&mean, &mut b);
            for (c, o) in out.iter_mut().enumerate() {
                *o = volume
                    * (0..6)
                        .map(|r| b[r * ndof + c] * stresses[0][r])
                        .sum::<f64>();
            }
            return;
        }
        for ((&p, &w), s) in self
            .rule
            .points
            .iter()
            .zip(&self.rule.weights)
            .zip(stresses)
        {
            let point = self.point(p, coords);
            Self::fill_b(&point.dndx, &mut b);
            let f = point.det * w;
            for (c, o) in out.iter_mut().enumerate() {
                *o += f * (0..6).map(|r| b[r * ndof + c] * s[r]).sum::<f64>();
            }
        }
    }

    /// Equivalent nodal forces of a pressure on face `face` (1-based CalculiX number),
    /// positive into the element, added to `out` (3 per node of the element).
    /// For plane and axisymmetric elements `face` is the edge (P1..P4 of the input).
    pub fn face_load(&self, coords: &[[f64; 3]], face: u8, pressure: f64, out: &mut [f64]) {
        if let Some(x) = &self.expansion {
            let mut f3 = vec![0.0; self.dof_count_3d()];
            self.face_load_3d(&x.coordinates(coords), x.face(face), pressure, &mut f3);
            x.condense_vector(&f3, out);
            return;
        }
        self.face_load_3d(coords, face, pressure, out);
    }

    fn face_load_3d(&self, coords: &[[f64; 3]], face: u8, pressure: f64, out: &mut [f64]) {
        let faces = self.family.faces();
        let f = &faces[face as usize - 1];
        let quadratic = self.family.is_quadratic();
        let nodes: Vec<usize> = if quadratic {
            f.corners.iter().chain(f.mids).copied().collect()
        } else {
            f.corners.to_vec()
        };
        let (points, weights) = self.face_integration(f.corners.len(), quadratic);
        let mut n = vec![0.0; nodes.len()];
        let mut dn = vec![[0.0; 2]; nodes.len()];
        for (p, w) in points.iter().zip(weights) {
            face_shape(f.corners.len(), quadratic, *p, &mut n, &mut dn);
            let mut t1 = [0.0; 3];
            let mut t2 = [0.0; 3];
            for (i, &node) in nodes.iter().enumerate() {
                for k in 0..3 {
                    t1[k] += dn[i][0] * coords[node][k];
                    t2[k] += dn[i][1] * coords[node][k];
                }
            }
            let normal = [
                t1[1] * t2[2] - t1[2] * t2[1],
                t1[2] * t2[0] - t1[0] * t2[2],
                t1[0] * t2[1] - t1[1] * t2[0],
            ];
            for (i, &node) in nodes.iter().enumerate() {
                for k in 0..3 {
                    out[3 * node + k] -= pressure * n[i] * normal[k] * w;
                }
            }
        }
    }

    /// The integration points of a face for pressure loads. CalculiX does not always take
    /// the full rule on quadrilateral faces; found by comparing its displacements under
    /// pressure on trapezoidal faces (where the rules differ; on parallelograms they agree):
    /// one point at the centre for C3D6 and C3D8R, 2×2 for C3D8/C3D8I and also for C3D20R
    /// and C3D15 (whose 8-node faces are thus integrated reduced), 3×3 for C3D20.
    fn face_integration(&self, corners: usize, quadratic: bool) -> (Vec<[f64; 2]>, Vec<f64>) {
        if corners == 4 {
            let one_point =
                self.family == Family::Wedge6 || self.formulation == Formulation::Reduced;
            if one_point {
                return (vec![[0.0, 0.0]], vec![4.0]);
            }
            let full = self.family == Family::Hex20 && self.rule.len() == 27;
            return face_rule(4, quadratic && full);
        }
        face_rule(corners, quadratic)
    }

    /// Equivalent nodal forces of a body force per unit volume `f(x)`, added to `out`.
    pub fn body_force(
        &self,
        coords: &[[f64; 3]],
        f: impl Fn([f64; 3]) -> [f64; 3],
        out: &mut [f64],
    ) {
        if let Some(x) = &self.expansion {
            let mut f3 = vec![0.0; self.dof_count_3d()];
            self.body_force_3d(&x.coordinates(coords), f, &mut f3);
            x.condense_vector(&f3, out);
            return;
        }
        self.body_force_3d(coords, f, out);
    }

    fn body_force_3d(
        &self,
        coords: &[[f64; 3]],
        f: impl Fn([f64; 3]) -> [f64; 3],
        out: &mut [f64],
    ) {
        for (&p, &w) in self.rule.points.iter().zip(&self.rule.weights) {
            let point = self.point(p, coords);
            let mut x = [0.0; 3];
            for (ni, c) in point.n.iter().zip(coords) {
                for k in 0..3 {
                    x[k] += ni * c[k];
                }
            }
            let force = f(x);
            let factor = point.det * w;
            for (i, ni) in point.n.iter().enumerate() {
                for k in 0..3 {
                    out[3 * i + k] += ni * force[k] * factor;
                }
            }
        }
    }

    /// Coordinates of the integration points.
    pub fn integration_point_coordinates(&self, coords: &[[f64; 3]]) -> Vec<[f64; 3]> {
        let expanded;
        let coords = match &self.expansion {
            Some(x) => {
                expanded = x.coordinates(coords);
                &expanded[..]
            }
            None => coords,
        };
        self.rule
            .points
            .iter()
            .map(|&p| {
                let point = self.point(p, coords);
                let mut x = [0.0; 3];
                for (ni, c) in point.n.iter().zip(coords) {
                    for k in 0..3 {
                        x[k] += ni * c[k];
                    }
                }
                x
            })
            .collect()
    }
}

/// `out += f · Bᵀ D B` for `b` of size 6 × n.
fn accumulate_btdb(b: &[f64], d: &[[f64; 6]; 6], n: usize, f: f64, out: &mut [f64]) {
    // db = D B (6 × n).
    let mut db = vec![0.0; 6 * n];
    for r in 0..6 {
        for c in 0..n {
            let mut s = 0.0;
            for k in 0..6 {
                s += d[r][k] * b[k * n + c];
            }
            db[r * n + c] = s;
        }
    }
    for i in 0..n {
        for j in i..n {
            let mut s = 0.0;
            for k in 0..6 {
                s += b[k * n + i] * db[k * n + j];
            }
            out[i * n + j] += f * s;
            if i != j {
                out[j * n + i] += f * s;
            }
        }
    }
}

/// `out += f · Aᵀ D B` for `a` of size 6 × m and `b` of size 6 × n (`out` is m × n).
fn accumulate_atdb(
    a: &[f64],
    m: usize,
    b: &[f64],
    n: usize,
    d: &[[f64; 6]; 6],
    f: f64,
    out: &mut [f64],
) {
    let mut db = vec![0.0; 6 * n];
    for r in 0..6 {
        for c in 0..n {
            let mut s = 0.0;
            for k in 0..6 {
                s += d[r][k] * b[k * n + c];
            }
            db[r * n + c] = s;
        }
    }
    for i in 0..m {
        for j in 0..n {
            let mut s = 0.0;
            for k in 0..6 {
                s += a[k * m + i] * db[k * n + j];
            }
            out[i * n + j] += f * s;
        }
    }
}

/// Solves `A X = B` for a dense `m × m` matrix `a` (destroyed) and `B` of size `m × n`,
/// Gaussian elimination with partial pivoting.
fn solve_dense(a: &mut [f64], m: usize, b: &[f64], n: usize) -> Vec<f64> {
    let mut x = b.to_vec();
    for k in 0..m {
        let pivot = (k..m)
            .max_by(|&i, &j| a[i * m + k].abs().total_cmp(&a[j * m + k].abs()))
            .unwrap();
        if pivot != k {
            for c in 0..m {
                a.swap(k * m + c, pivot * m + c);
            }
            for c in 0..n {
                x.swap(k * n + c, pivot * n + c);
            }
        }
        let akk = a[k * m + k];
        for i in k + 1..m {
            let factor = a[i * m + k] / akk;
            if factor == 0.0 {
                continue;
            }
            for c in k..m {
                a[i * m + c] -= factor * a[k * m + c];
            }
            for c in 0..n {
                x[i * n + c] -= factor * x[k * n + c];
            }
        }
    }
    for k in (0..m).rev() {
        for c in 0..n {
            let mut s = x[k * n + c];
            for j in k + 1..m {
                s -= a[k * m + j] * x[j * n + c];
            }
            x[k * n + c] = s / a[k * m + k];
        }
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    fn isotropic(e: f64, nu: f64) -> [[f64; 6]; 6] {
        let l = e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
        let g = e / (2.0 * (1.0 + nu));
        let mut d = [[0.0; 6]; 6];
        for i in 0..3 {
            d[i][..3].fill(l);
            d[i][i] = l + 2.0 * g;
            d[i + 3][i + 3] = g;
        }
        d
    }

    fn unit_cube(family: Family) -> Vec<[f64; 3]> {
        family
            .node_coordinates()
            .iter()
            .map(|c| [0.5 * (c[0] + 1.0), 0.5 * (c[1] + 1.0), 0.5 * (c[2] + 1.0)])
            .collect()
    }

    fn reference(family: Family) -> Vec<[f64; 3]> {
        family.node_coordinates().to_vec()
    }

    fn all_kinds() -> Vec<ElementType> {
        use ElementType::*;
        vec![C3D8, C3D8R, C3D8I, C3D20, C3D20R, C3D4, C3D10, C3D6, C3D15]
    }

    #[test]
    fn a_folded_element_is_inverted() {
        for kind in all_kinds() {
            let e = solid(kind).unwrap();
            let coords = match e.family {
                Family::Hex8 | Family::Hex20 => unit_cube(e.family),
                _ => reference(e.family),
            };
            assert!(!e.inverted(&coords), "{kind}");
            // Mirrored: turned inside out.
            let mirrored: Vec<[f64; 3]> = coords.iter().map(|c| [-c[0], c[1], c[2]]).collect();
            assert!(e.inverted(&mirrored), "{kind}");
        }
        // A brick whose top face is pushed through its bottom on one side only: positive
        // at some integration points, negative at others.
        let e = solid(ElementType::C3D8).unwrap();
        let mut coords = unit_cube(Family::Hex8);
        for c in coords.iter_mut().filter(|c| c[2] > 0.5 && c[0] > 0.5) {
            c[2] = -2.0;
        }
        assert!(e.inverted(&coords));
    }

    #[test]
    fn volumes_are_right() {
        for kind in all_kinds() {
            let e = solid(kind).unwrap();
            let coords = match e.family {
                Family::Hex8 | Family::Hex20 => unit_cube(e.family),
                _ => reference(e.family),
            };
            let expected = match e.family {
                Family::Hex8 | Family::Hex20 => 1.0,
                Family::Tet4 | Family::Tet10 => 1.0 / 6.0,
                Family::Wedge6 | Family::Wedge15 => 1.0,
            };
            assert!((e.volume(&coords) - expected).abs() < 1e-12, "{kind}");
        }
    }

    #[test]
    fn stiffness_is_symmetric_and_rigid_body_free() {
        let d = isotropic(1000.0, 0.3);
        for kind in all_kinds() {
            let e = solid(kind).unwrap();
            let mut coords = match e.family {
                Family::Hex8 | Family::Hex20 => unit_cube(e.family),
                _ => reference(e.family),
            };
            // Distort a little so that the Jacobian is not constant.
            for (i, c) in coords.iter_mut().enumerate() {
                c[0] += 0.05 * (i as f64 * 0.7).sin();
                c[2] += 0.04 * (i as f64 * 1.3).cos();
            }
            let n = e.dof_count();
            let mut k = Vec::new();
            e.stiffness(&coords, &d, &mut k);
            for i in 0..n {
                for j in 0..n {
                    assert!(
                        (k[i * n + j] - k[j * n + i]).abs() < 1e-9 * d[0][0],
                        "{kind}"
                    );
                }
            }
            // Translation and small rotation give no forces (on distorted C3D8R elements
            // only translation: CalculiX's hourglass vectors, which the solver follows, are
            // not orthogonal to linear fields in x and y).
            let mode_count = if kind == ElementType::C3D8R { 3 } else { 6 };
            let modes: Vec<Vec<f64>> = (0..mode_count)
                .map(|m| {
                    (0..e.node_count())
                        .flat_map(|a| {
                            let x = coords[a];
                            match m {
                                0 => [1.0, 0.0, 0.0],
                                1 => [0.0, 1.0, 0.0],
                                2 => [0.0, 0.0, 1.0],
                                3 => [0.0, -x[2], x[1]],
                                4 => [x[2], 0.0, -x[0]],
                                _ => [-x[1], x[0], 0.0],
                            }
                        })
                        .collect()
                })
                .collect();
            for u in &modes {
                for i in 0..n {
                    let f: f64 = (0..n).map(|j| k[i * n + j] * u[j]).sum();
                    assert!(
                        f.abs() < 1e-8 * d[0][0],
                        "{kind}: rigid mode gives force {f}"
                    );
                }
            }
            // A uniform strain field gives uniform strains at the integration points. Not for
            // C3D8I: without Taylor's det J0 / det J scaling (CalculiX leaves it out) the
            // incompatible modes pick up part of a uniform strain in a distorted element.
            if kind == ElementType::C3D8I {
                continue;
            }
            let u: Vec<f64> = coords
                .iter()
                .flat_map(|x| {
                    [
                        0.01 * x[0] + 0.002 * x[1],
                        0.003 * x[1],
                        -0.004 * x[2] + 0.001 * x[0],
                    ]
                })
                .collect();
            for s in e.strains(&coords, &d, &u) {
                let expected = [0.01, 0.003, -0.004, 0.002, 0.0, 0.001];
                for (a, b) in s.iter().zip(expected) {
                    assert!((a - b).abs() < 1e-10, "{kind}: strain {s:?}");
                }
            }
        }
    }

    #[test]
    fn pressure_gives_force_times_area() {
        for kind in all_kinds() {
            let e = solid(kind).unwrap();
            let coords = match e.family {
                Family::Hex8 | Family::Hex20 => unit_cube(e.family),
                _ => reference(e.family),
            };
            for (f, face) in e.family.faces().iter().enumerate() {
                let mut out = vec![0.0; e.dof_count()];
                e.face_load(&coords, f as u8 + 1, 2.0, &mut out);
                let total: [f64; 3] =
                    [0, 1, 2].map(|k| (0..e.node_count()).map(|a| out[3 * a + k]).sum());
                // Outward normal of the face from the corners.
                let c = face.corners;
                let (a, b, d) = (coords[c[0]], coords[c[1]], coords[c[2]]);
                let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                let e2 = [d[0] - a[0], d[1] - a[1], d[2] - a[2]];
                let n = [
                    e1[1] * e2[2] - e1[2] * e2[1],
                    e1[2] * e2[0] - e1[0] * e2[2],
                    e1[0] * e2[1] - e1[1] * e2[0],
                ];
                let area = if c.len() == 4 {
                    (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt()
                } else {
                    0.5 * (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt()
                };
                let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                for k in 0..3 {
                    let expected = -2.0 * area * n[k] / len;
                    assert!(
                        (total[k] - expected).abs() < 1e-12,
                        "{kind} face {}: {total:?}",
                        f + 1
                    );
                }
            }
        }
    }

    #[test]
    fn extrapolation_reproduces_linear_fields() {
        // A field linear in the natural coordinates is reproduced at the nodes by every
        // rule with more than one point: exactly, up to CalculiX's four-digit tables for
        // the 2×2×2 hexahedra and its five-decimal tetrahedron weights, and only in the plane for the two-point wedge rule, whose
        // axial weights follow CalculiX rather than the rule's points.
        for kind in all_kinds() {
            let e = solid(kind).unwrap();
            if e.integration_points() == 1 {
                continue;
            }
            let field = |p: [f64; 3]| 1.0 + 2.0 * p[0] - p[1] + 0.5 * p[2];
            let ips: Vec<f64> = e.rule.points.iter().map(|&p| field(p)).collect();
            let ipn = ips.len();
            let hex8_points = matches!(e.family, Family::Hex8 | Family::Hex20) && ipn == 8;
            let wedge_plane = matches!(e.family, Family::Wedge6 | Family::Wedge15) && ipn == 2;
            let hex27_points = matches!(e.family, Family::Hex8 | Family::Hex20) && ipn == 27;
            let tet_points = matches!(e.family, Family::Tet4 | Family::Tet10);
            let tolerance = if hex8_points || wedge_plane {
                2e-3
            } else if hex27_points || tet_points {
                1e-4
            } else {
                1e-10
            };
            for (a, node) in e.family.node_coordinates().iter().enumerate() {
                let v: f64 = (0..ipn)
                    .map(|g| e.extrapolation[a * ipn + g] * ips[g])
                    .sum();
                let expected = if wedge_plane {
                    // Constant in the plane: the centroid value; axially CalculiX's weights
                    // favour the first point on even nodes and the second on odd ones.
                    let sign = if a % 2 == 0 { -1.0 } else { 1.0 };
                    1.0 + 2.0 / 3.0 - 1.0 / 3.0 + 0.5 * sign * 1.291 / 3.0_f64.sqrt()
                } else {
                    field(*node)
                };
                assert!(
                    (v - expected).abs() < tolerance,
                    "{kind} node {a}: {v} vs {expected}"
                );
            }
        }
    }

    #[test]
    fn extrapolation_weights_match_calculix_tables() {
        // The 2×2×2 corner weights CalculiX tabulates (fitted from its output).
        let e = solid(ElementType::C3D8).unwrap();
        let mut corner: Vec<f64> = e.extrapolation[0..8].to_vec();
        corner.sort_by(|a, b| a.total_cmp(b));
        assert_eq!(corner[7], 2.549);
        assert_eq!(corner[0], -0.683);
        assert_eq!(corner[6], 0.183);
        assert_eq!(corner[3], -0.04904);
        let e = solid(ElementType::C3D20R).unwrap();
        let mut mid: Vec<f64> = e.extrapolation[8 * 8..9 * 8].to_vec();
        mid.sort_by(|a, b| a.total_cmp(b));
        assert_eq!(mid[7], 0.933);
        assert_eq!(mid[0], -0.25);
        assert_eq!(mid[4], 0.06698);
        // Each row of every extrapolation matrix sums to one.
        for kind in all_kinds() {
            let e = solid(kind).unwrap();
            let ipn = e.integration_points();
            for a in 0..e.family.node_count() {
                let sum: f64 = e.extrapolation[a * ipn..(a + 1) * ipn].iter().sum();
                assert!((sum - 1.0).abs() < 2e-3, "{kind} node {a}: row sum {sum}");
            }
        }
    }

    #[test]
    fn reduced_integration_matches_calculix_hourglass_stiffness() {
        // CalculiX's C3D8R stiffness of a 2 × 0.5 × 0.2 box: the one-point stiffness plus
        // 11.7 on each of the twelve hourglass modes (identified from its output with
        // E = 1000, ν = 0.3).
        let d = isotropic(1000.0, 0.3);
        let e = solid(ElementType::C3D8R).unwrap();
        let coords: Vec<[f64; 3]> = Family::Hex8
            .node_coordinates()
            .iter()
            .map(|c| [1.0 * (c[0] + 1.0), 0.25 * (c[1] + 1.0), 0.1 * (c[2] + 1.0)])
            .collect();
        let mut k = Vec::new();
        e.stiffness(&coords, &d, &mut k);
        let h = [1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0];
        let mut u = [0.0; 24];
        for a in 0..8 {
            u[3 * a + 2] = h[a];
        }
        let f: Vec<f64> = (0..24)
            .map(|i| (0..24).map(|j| k[i * 24 + j] * u[j]).sum())
            .collect();
        for i in 0..24 {
            assert!(
                (f[i] - 11.7 * u[i]).abs() < 1e-6,
                "dof {i}: {} vs {}",
                f[i],
                11.7 * u[i]
            );
        }
    }

    #[test]
    fn incompatible_modes_bend_better_than_c3d8() {
        // A single element in pure bending: C3D8I reproduces the beam solution much more
        // closely than C3D8 (which locks in shear).
        let d = isotropic(1000.0, 0.0);
        let coords: Vec<[f64; 3]> = Family::Hex8
            .node_coordinates()
            .iter()
            .map(|c| [5.0 * (c[0] + 1.0), 0.5 * (c[1] + 1.0), 0.5 * (c[2] + 1.0)])
            .collect();
        // Pure bending displacement field about y: u = -κ x z, w = κ x²/2 (plus the
        // Poisson-free part) — apply it on all nodes and compare the energy with the exact
        // one (M² L / 2EI); C3D8I should be exact, C3D8 stiffer.
        let kappa = 0.01;
        let u: Vec<f64> = coords
            .iter()
            .flat_map(|x| {
                let z = x[2] - 0.5;
                [-kappa * x[0] * z, 0.0, kappa * x[0] * x[0] / 2.0]
            })
            .collect();
        let energy = |kind: ElementType| {
            let e = solid(kind).unwrap();
            let n = e.dof_count();
            let mut k = Vec::new();
            e.stiffness(&coords, &d, &mut k);
            let mut s = 0.0;
            for i in 0..n {
                for j in 0..n {
                    s += u[i] * k[i * n + j] * u[j];
                }
            }
            0.5 * s
        };
        let exact = 0.5 * 1000.0 * (1.0 / 12.0) * kappa * kappa * 10.0;
        let e8 = energy(ElementType::C3D8);
        let e8i = energy(ElementType::C3D8I);
        assert!((e8i - exact).abs() < 1e-3 * exact, "C3D8I {e8i} vs {exact}");
        assert!(e8 > 1.5 * exact, "C3D8 {e8} vs {exact}");
    }
}

// ---------------------------------------------------------------------------------------
// Mass and geometric (initial stress, initial displacement) stiffness, M2 T7. Formulas from
// Zienkiewicz & Taylor (consistent mass, chapter 16 of volume 1 / chapter 2 of volume 2)
// and Bathe, "Finite Element Procedures", section 6.3 (total Lagrangian formulation:
// linear strain-displacement matrix B_L = B_0 + B_1(u) and the initial stress matrix
// ∫ Gᵀ S G dV). No CalculiX code was consulted.
// ---------------------------------------------------------------------------------------

impl SolidElement {
    /// The integration rule of the consistent mass matrix: the element's own rule, as
    /// CalculiX does it (the eigenfrequencies of the C3D20R fixtures agree to seven digits
    /// with the 2×2×2 rule and differ by up to 4e-5 with the full 3×3×3 rule; inferred
    /// from its output, no source consulted). One-point rules (C3D8R, C3D4) would leave
    /// the mass matrix rank deficient and use the full rule of their family instead.
    pub fn mass_rule(&self) -> IntegrationRule {
        if self.rule.len() > 1 {
            return self.rule.clone();
        }
        match self.family {
            Family::Hex8 | Family::Hex20 => IntegrationRule::hex(2),
            Family::Tet4 => IntegrationRule::tet4(),
            Family::Tet10 => IntegrationRule::tet14(),
            Family::Wedge6 => IntegrationRule::wedge2(),
            Family::Wedge15 => IntegrationRule::wedge18(),
        }
    }

    /// The integration rule CalculiX's direct time integration (`*DYNAMIC`) takes for the
    /// consistent mass matrix: the full rule of the family, also for reduced elements
    /// (C3D20R with 3×3×3: the transient response of the C3D20R cantilever agrees with
    /// ccx 2.21 to the digits of the `.frd` only so, while its eigenfrequencies need the
    /// element's own 2×2×2 rule, see [`SolidElement::mass_rule`]; inferred from its output).
    pub fn dynamic_mass_rule(&self) -> IntegrationRule {
        match self.family {
            Family::Hex8 => IntegrationRule::hex(2),
            Family::Hex20 => IntegrationRule::hex(3),
            _ => self.mass_rule(),
        }
    }

    /// The element with the full integration of its family where it is reduced (C3D20R
    /// with 3×3×3 points like C3D20, C3D8R with 2×2×2 like C3D8), `None` otherwise. CalculiX
    /// takes these for the matrix of the initial accelerations of a `*DYNAMIC` step (the
    /// accelerations of single-DOF C3D8R and C3D20R oracles agree with ccx 2.21 to all
    /// printed digits only so; inferred from its output). Only stiffness and mass are
    /// meaningful on the result.
    pub fn fully_integrated(&self) -> Option<SolidElement> {
        let (rule, formulation) = match (self.family, self.formulation) {
            (Family::Hex20, Formulation::Standard) if self.rule.len() < 27 => {
                (IntegrationRule::hex(3), Formulation::Standard)
            }
            (Family::Hex8, Formulation::Reduced) => {
                (IntegrationRule::hex(2), Formulation::Standard)
            }
            _ => return None,
        };
        let mut full = self.clone();
        full.extrapolation = vec![0.0; rule.len() * self.family.node_count()];
        full.reference_gradients = reference_gradients(self.family, &rule);
        full.rule = rule;
        full.formulation = formulation;
        Some(full)
    }

    /// The volume of the element and the volume average of its shape function gradients
    /// as a row over its DOFs (`g[3a + c] = (1/V) ∫ ∂N_a/∂x_c dV`), in the reference
    /// configuration: `g · v` is the mean volumetric strain rate of the nodal velocities
    /// `v`, and `V (g · v) g` the nodal forces of a unit pressure rate (the linear bulk
    /// viscosity of explicit time integration). Plane and axisymmetric elements average over
    /// their expanded element and return the row on their 2D DOFs and the volume of the
    /// thickness or the full circumference.
    pub fn volume_mean_gradients(&self, coords: &[[f64; 3]]) -> (f64, Vec<f64>) {
        let expanded;
        let (coords3, x) = match self.expansion.as_deref() {
            Some(x) => {
                expanded = x.coordinates(coords);
                (expanded.as_slice(), Some(x))
            }
            None => (coords, None),
        };
        let rule = self.mass_rule();
        let nodes = self.family.node_count();
        let mut g3 = vec![0.0; 3 * nodes];
        let mut volume = 0.0;
        for (&p, &w) in rule.points.iter().zip(&rule.weights) {
            let point = self.point(p, coords3);
            let dv = point.det * w;
            volume += dv;
            for (a, d) in point.dndx.iter().enumerate() {
                for c in 0..3 {
                    g3[3 * a + c] += dv * d[c];
                }
            }
        }
        if volume > 0.0 {
            g3.iter_mut().for_each(|g| *g /= volume);
        }
        match x {
            None => (volume, g3),
            Some(x) => {
                let mut g2 = vec![0.0; self.dof_count()];
                x.condense_vector(&g3, &mut g2);
                g2.iter_mut().for_each(|g| *g /= x.scale);
                (volume * x.scale, g2)
            }
        }
    }

    /// Consistent mass matrix (row-major `3n × 3n`) for the density `rho`:
    /// `M[(a,i),(b,j)] = δ_ij ∫ ρ N_a N_b dV`, with the rule of [`SolidElement::mass_rule`].
    pub fn mass(&self, coords: &[[f64; 3]], rho: f64, out: &mut Vec<f64>) {
        self.mass_with(coords, rho, &self.mass_rule(), out);
    }

    /// [`SolidElement::mass`] with another integration rule.
    pub fn mass_with(
        &self,
        coords: &[[f64; 3]],
        rho: f64,
        rule: &IntegrationRule,
        out: &mut Vec<f64>,
    ) {
        if let Some(x) = &self.expansion {
            // Plane and axisymmetric elements: the mass of the expanded element, condensed
            // onto the 2D nodes like the stiffness.
            let mut m3 = Vec::new();
            self.mass_3d(&x.coordinates(coords), rho, rule, &mut m3);
            let n = self.dof_count();
            out.clear();
            out.resize(n * n, 0.0);
            x.condense_matrix(&m3, out);
            return;
        }
        self.mass_3d(coords, rho, rule, out);
    }

    fn mass_3d(&self, coords: &[[f64; 3]], rho: f64, rule: &IntegrationRule, out: &mut Vec<f64>) {
        let ndof = self.dof_count_3d();
        let nodes = self.family.node_count();
        out.clear();
        out.resize(ndof * ndof, 0.0);
        for (&p, &w) in rule.points.iter().zip(&rule.weights) {
            let point = self.point(p, coords);
            let f = rho * point.det * w;
            for a in 0..nodes {
                for b in a..nodes {
                    let v = f * point.n[a] * point.n[b];
                    for k in 0..3 {
                        out[(3 * a + k) * ndof + 3 * b + k] += v;
                        if a != b {
                            out[(3 * b + k) * ndof + 3 * a + k] += v;
                        }
                    }
                }
            }
        }
    }

    /// Initial stress (geometric) stiffness `∫ Gᵀ S G dV` (row-major `3n × 3n`) for the
    /// stresses `[xx, yy, zz, xy, yz, zx]` at the element's integration points:
    /// `K_σ[(a,i),(b,j)] = δ_ij ∇N_aᵀ S ∇N_b`.
    pub fn geometric_stiffness(
        &self,
        coords: &[[f64; 3]],
        stresses: &[[f64; 6]],
        out: &mut Vec<f64>,
    ) {
        let ndof = self.dof_count();
        let nodes = self.node_count();
        out.clear();
        out.resize(ndof * ndof, 0.0);
        for ((&p, &w), s) in self
            .rule
            .points
            .iter()
            .zip(&self.rule.weights)
            .zip(stresses)
        {
            let point = self.point(p, coords);
            let f = point.det * w;
            let sm = [[s[0], s[3], s[5]], [s[3], s[1], s[4]], [s[5], s[4], s[2]]];
            // S ∇N_b for every node once.
            let sg: Vec<[f64; 3]> = point
                .dndx
                .iter()
                .map(|g| {
                    let mut r = [0.0; 3];
                    for (i, ri) in r.iter_mut().enumerate() {
                        *ri = sm[i][0] * g[0] + sm[i][1] * g[1] + sm[i][2] * g[2];
                    }
                    r
                })
                .collect();
            for a in 0..nodes {
                let ga = point.dndx[a];
                for b in a..nodes {
                    let v = f * (ga[0] * sg[b][0] + ga[1] * sg[b][1] + ga[2] * sg[b][2]);
                    for k in 0..3 {
                        out[(3 * a + k) * ndof + 3 * b + k] += v;
                        if a != b {
                            out[(3 * b + k) * ndof + 3 * a + k] += v;
                        }
                    }
                }
            }
        }
    }

    /// Engineering strain increment of the total Lagrangian formulation for a displacement
    /// increment `du` about the displaced state `u`: `B_L(u) du` with `B_L = B_0 + B_1(u)`
    /// (Bathe, section 6.3.4). The incompatible modes of C3D8I follow `du` as in
    /// [`Self::strains`]. Without displacement this is [`Self::strains`].
    pub fn linearized_strains(
        &self,
        coords: &[[f64; 3]],
        d: &[[f64; 6]; 6],
        u: &[f64],
        du: &[f64],
    ) -> Vec<[f64; 6]> {
        let ndof = self.dof_count();
        let nodes = self.node_count();
        let mut bl = vec![0.0; 6 * ndof];
        let incompatible = (self.formulation == Formulation::Incompatible).then(|| {
            let (x, inv0) = self.incompatible_coupling(coords, d);
            let mut alpha = [0.0; 9];
            for (m, a) in alpha.iter_mut().enumerate() {
                *a = -(0..ndof).map(|j| x[m * ndof + j] * du[j]).sum::<f64>();
            }
            (alpha, inv0)
        });
        let mut g = vec![0.0; 6 * 9];
        self.rule
            .points
            .iter()
            .map(|&p| {
                let point = self.point(p, coords);
                Self::fill_b(&point.dndx, &mut bl);
                Self::add_b1(&point.dndx, nodes, u, &mut bl);
                let mut e = [0.0; 6];
                for (r, er) in e.iter_mut().enumerate() {
                    *er = (0..ndof).map(|c| bl[r * ndof + c] * du[c]).sum();
                }
                if let Some((alpha, inv0)) = &incompatible {
                    Self::fill_g(p, inv0, &mut g);
                    for (r, er) in e.iter_mut().enumerate() {
                        *er += (0..9).map(|c| g[r * 9 + c] * alpha[c]).sum::<f64>();
                    }
                }
                e
            })
            .collect()
    }

    /// Adds the initial displacement part `B_1(u)` of the total Lagrangian strain matrix
    /// to `b` (row-major `6 × 3n`, which holds `B_0` on entry).
    fn add_b1(dndx: &[[f64; 3]], nodes: usize, u: &[f64], b: &mut [f64]) {
        let ndof = 3 * nodes;
        // Displacement gradient l[i][j] = ∂u_i/∂x_j.
        let mut l = [[0.0; 3]; 3];
        for (a, g) in dndx.iter().enumerate() {
            for (i, row) in l.iter_mut().enumerate() {
                for (lij, gj) in row.iter_mut().zip(g) {
                    *lij += u[3 * a + i] * gj;
                }
            }
        }
        for (a, g) in dndx.iter().enumerate() {
            for (i, li) in l.iter().enumerate() {
                let c = 3 * a + i;
                b[c] += li[0] * g[0];
                b[ndof + c] += li[1] * g[1];
                b[2 * ndof + c] += li[2] * g[2];
                b[3 * ndof + c] += li[0] * g[1] + li[1] * g[0];
                b[4 * ndof + c] += li[1] * g[2] + li[2] * g[1];
                b[5 * ndof + c] += li[0] * g[2] + li[2] * g[0];
            }
        }
    }

    /// The initial displacement stiffness of the total Lagrangian formulation for the
    /// nodal displacements `u`: `∫ (B_Lᵀ D B_L − B_0ᵀ D B_0) dV` with `B_L = B_0 + B_1(u)`
    /// (Bathe, section 6.3.4), added to `out` (row-major `3n × 3n`). Together with
    /// [`Self::stiffness`] and [`Self::geometric_stiffness`] this is the tangent stiffness
    /// at a deformed state, what a perturbation step needs.
    pub fn initial_displacement_stiffness(
        &self,
        coords: &[[f64; 3]],
        d: &[[f64; 6]; 6],
        u: &[f64],
        out: &mut [f64],
    ) {
        let ndof = self.dof_count();
        let nodes = self.node_count();
        let mut b0 = vec![0.0; 6 * ndof];
        let mut bl = vec![0.0; 6 * ndof];
        for (&p, &w) in self.rule.points.iter().zip(&self.rule.weights) {
            let point = self.point(p, coords);
            Self::fill_b(&point.dndx, &mut b0);
            bl.copy_from_slice(&b0);
            Self::add_b1(&point.dndx, nodes, u, &mut bl);
            let f = point.det * w;
            accumulate_btdb(&bl, d, ndof, f, out);
            accumulate_btdb(&b0, d, ndof, -f, out);
        }
    }
}

#[cfg(test)]
mod mass_tests {
    use super::*;

    fn unit_cube() -> Vec<[f64; 3]> {
        Family::Hex8
            .node_coordinates()
            .iter()
            .map(|p| [0.5 * (p[0] + 1.0), 0.5 * (p[1] + 1.0), 0.5 * (p[2] + 1.0)])
            .collect()
    }

    #[test]
    fn mass_matrix_sums_to_the_element_mass() {
        for kind in [
            ElementType::C3D8,
            ElementType::C3D8R,
            ElementType::C3D20R,
            ElementType::C3D20,
        ] {
            let element = solid(kind).unwrap();
            let coords: Vec<[f64; 3]> = element
                .family
                .node_coordinates()
                .iter()
                .map(|p| [0.5 * (p[0] + 1.0), 0.5 * (p[1] + 1.0), 0.5 * (p[2] + 1.0)])
                .collect();
            let mut m = Vec::new();
            element.mass(&coords, 7.85e-9, &mut m);
            let n = element.dof_count();
            // Rigid translation in x: eᵀ M e = ρ V.
            let mut total = 0.0;
            for a in 0..n / 3 {
                for b in 0..n / 3 {
                    total += m[(3 * a) * n + 3 * b];
                }
            }
            assert!((total - 7.85e-9).abs() < 1e-20, "{kind}: {total}");
            for i in 0..n {
                for j in 0..n {
                    assert!((m[i * n + j] - m[j * n + i]).abs() < 1e-24);
                }
            }
        }
        for (kind, volume) in [
            (ElementType::C3D4, 1.0 / 6.0),
            (ElementType::C3D10, 1.0 / 6.0),
            (ElementType::C3D6, 1.0),
            (ElementType::C3D15, 1.0),
        ] {
            let element = solid(kind).unwrap();
            let coords = element.family.node_coordinates().to_vec();
            let mut m = Vec::new();
            element.mass(&coords, 2.0, &mut m);
            let n = element.dof_count();
            let mut total = 0.0;
            for a in 0..n / 3 {
                for b in 0..n / 3 {
                    total += m[(3 * a + 1) * n + 3 * b + 1];
                }
            }
            assert!((total - 2.0 * volume).abs() < 1e-12, "{kind}: {total}");
        }
    }

    #[test]
    fn geometric_stiffness_of_a_uniaxially_stressed_cube() {
        // A unit cube under σ_xx = s: K_σ = s ∫ ∂N_a/∂x ∂N_b/∂x dV ⊗ I. The rigid-body
        // translations are in its kernel; a uniform stretch in x has the energy s/2·(Δl/l)².
        let element = solid(ElementType::C3D8).unwrap();
        let coords = unit_cube();
        let s = 100.0;
        let stresses = vec![[s, 0.0, 0.0, 0.0, 0.0, 0.0]; 8];
        let mut k = Vec::new();
        element.geometric_stiffness(&coords, &stresses, &mut k);
        let n = 24;
        let translation: Vec<f64> = (0..n).map(|i| if i % 3 == 1 { 1.0 } else { 0.0 }).collect();
        let stretch: Vec<f64> = (0..8).flat_map(|a| [coords[a][0], 0.0, 0.0]).collect();
        let quad = |v: &[f64]| -> f64 {
            (0..n)
                .map(|i| v[i] * (0..n).map(|j| k[i * n + j] * v[j]).sum::<f64>())
                .sum()
        };
        assert!(quad(&translation).abs() < 1e-12);
        assert!((quad(&stretch) - s).abs() < 1e-10, "{}", quad(&stretch));
    }

    #[test]
    fn initial_displacement_stiffness_vanishes_without_displacement() {
        let element = solid(ElementType::C3D20R).unwrap();
        let coords: Vec<[f64; 3]> = element.family.node_coordinates().to_vec();
        let d = [[1.0; 6]; 6];
        let u = vec![0.0; 60];
        let mut out = vec![0.0; 3600];
        element.initial_displacement_stiffness(&coords, &d, &u, &mut out);
        assert!(out.iter().all(|v| v.abs() < 1e-15));
    }
}
