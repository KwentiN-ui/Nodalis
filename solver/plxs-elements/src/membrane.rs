//! Membranes computed natively in their own surface: the three-node `M3D3` (one point,
//! constant strain) and the four-node `M3D4` (2 × 2 Gauss points), plane stress, no
//! bending stiffness.
//!
//! Observed with ccx 2.21 as a black box (no CalculiX code consulted):
//! - `M3D3` gives exactly the constant-strain plane stress triangle, including nodal
//!   stresses (element values averaged at the nodes) and strains with the thickness strain
//!   `ε33 = -ν / (1 - ν) (ε11 + ε22)`.
//! - `M3D4` has no stiffness out of its plane: a simply supported strip under a transverse
//!   load is singular in ccx, while the C3D8/C3D8I layer it would expand to carries it by
//!   bending. In its plane it agrees with the plane stress quadrilateral with 2 × 2 points
//!   up to a term of the order (thickness / element size)² (it is a C3D8 layer whose
//!   thickness may change there; 5e-4 for a 10 × 10 × 1 element in in-plane bending).
//! - Under `NLGEOM` a prestressed `M3D4` strip under pressure deflects like a string,
//!   `w = p L² / (8 N)`, to the digits of the `.frd` with a total Lagrangian membrane
//!   (Green-Lagrange strain, St. Venant-Kirchhoff plane stress) and a pressure that follows
//!   the deformed surface.
//!
//! - Out of its plane `M3D4` is not exactly free in ccx either: a node between a fixed
//!   edge and an edge tied to a moving solid lies on the straight line between them, as a
//!   tiny stiffness against the slope `∇w` would put it (and a transverse load on a free
//!   membrane gives displacements of 1e10). plxs adds such a stiffness,
//!   [`OUT_OF_PLANE`] `E t ∫ ∇N·∇N dA` on the normal displacement, so that a membrane
//!   without prestress is not singular.
//!
//! The total Lagrangian membrane follows Bathe, "Finite Element Procedures", section 6.3
//! (initial stress matrix `∫ ∇Nᵀ S ∇N dA` on every displacement component).

use plxs_core::ElementType;

use crate::expand::{cross, dot, normalize};

/// Whether an element type is computed as a native membrane (the others of the membrane
/// family are expanded to solids).
pub fn is_native(kind: ElementType) -> bool {
    matches!(kind, ElementType::M3D3 | ElementType::M3D4)
}

/// Relative stiffness against the slope of the normal displacement (see the module
/// documentation): far below any prestress a real membrane carries (σ / E ≥ 1e-6), large
/// enough for a direct solver.
pub const OUT_OF_PLANE: f64 = 1e-9;

/// Integration points (natural coordinates) and weights of a membrane with `nodes` nodes.
fn rule(nodes: usize) -> Vec<([f64; 2], f64)> {
    if nodes == 3 {
        vec![([1.0 / 3.0, 1.0 / 3.0], 0.5)]
    } else {
        let g = 1.0 / 3.0_f64.sqrt();
        // CalculiX's order of the 2 × 2 points: ξ fastest.
        vec![
            ([-g, -g], 1.0),
            ([g, -g], 1.0),
            ([-g, g], 1.0),
            ([g, g], 1.0),
        ]
    }
}

/// Shape functions and their derivatives with respect to the natural coordinates.
fn shape(nodes: usize, p: [f64; 2]) -> (Vec<f64>, Vec<[f64; 2]>) {
    let [r, s] = p;
    if nodes == 3 {
        (
            vec![1.0 - r - s, r, s],
            vec![[-1.0, -1.0], [1.0, 0.0], [0.0, 1.0]],
        )
    } else {
        let c = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
        let n = c
            .iter()
            .map(|[a, b]| 0.25 * (1.0 + a * r) * (1.0 + b * s))
            .collect();
        let dn = c
            .iter()
            .map(|[a, b]| [0.25 * a * (1.0 + b * s), 0.25 * b * (1.0 + a * r)])
            .collect();
        (n, dn)
    }
}

/// Tangents `∂x/∂ξ`, `∂x/∂η` of positions `x` at a point.
fn tangents(x: &[[f64; 3]], dn: &[[f64; 2]]) -> ([f64; 3], [f64; 3]) {
    let mut a = [[0.0; 3]; 2];
    for (xa, d) in x.iter().zip(dn) {
        for (k, ak) in a.iter_mut().enumerate() {
            for c in 0..3 {
                ak[c] += d[k] * xa[c];
            }
        }
    }
    (a[0], a[1])
}

/// The reference geometry at an integration point.
struct Point {
    /// Derivatives with respect to the local in-plane axes e1, e2.
    dndx: Vec<[f64; 2]>,
    /// Local frame: rows e1, e2, normal.
    frame: [[f64; 3]; 3],
    /// Reference area times the weight.
    da: f64,
}

fn point(x: &[[f64; 3]], p: [f64; 2], weight: f64) -> Option<Point> {
    let (_, dn) = shape(x.len(), p);
    let (a1, a2) = tangents(x, &dn);
    let normal = cross(a1, a2);
    let area = dot(normal, normal).sqrt();
    if area <= 1e-12 * dot(a1, a1).max(1e-300).sqrt() * dot(a2, a2).max(1e-300).sqrt() {
        return None;
    }
    let normal = normalize(normal);
    let e1 = normalize(a1);
    let e2 = cross(normal, e1);
    // J[i][j] = a_i · e_j; ∂N/∂ξ_i = Σ_j J[i][j] ∂N/∂X_j.
    let j = [[dot(a1, e1), dot(a1, e2)], [dot(a2, e1), dot(a2, e2)]];
    let det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
    if det <= 0.0 {
        return None;
    }
    let inv = [
        [j[1][1] / det, -j[0][1] / det],
        [-j[1][0] / det, j[0][0] / det],
    ];
    let dndx = dn
        .iter()
        .map(|d| {
            [
                inv[0][0] * d[0] + inv[0][1] * d[1],
                inv[1][0] * d[0] + inv[1][1] * d[1],
            ]
        })
        .collect();
    Some(Point {
        dndx,
        frame: [e1, e2, normal],
        da: area * weight,
    })
}

fn plane_stress(young: f64, poisson: f64) -> [[f64; 3]; 3] {
    let f = young / (1.0 - poisson * poisson);
    [
        [f, f * poisson, 0.0],
        [f * poisson, f, 0.0],
        [0.0, 0.0, f * (1.0 - poisson) / 2.0],
    ]
}

/// A membrane element: reference node positions, elastic constants and thickness.
#[derive(Clone, Debug)]
pub struct Membrane {
    pub coordinates: Vec<[f64; 3]>,
    pub young: f64,
    pub poisson: f64,
    pub thickness: f64,
}

/// Kinematics at a point: the in-plane gradient columns `f1`, `f2` (global) and the
/// strain `[E11, E22, 2 E12]` in the local axes.
struct Kinematics {
    f: [[f64; 3]; 2],
    strain: [f64; 3],
}

impl Membrane {
    /// `None` for a degenerate element or another node count than 3 or 4.
    pub fn new(
        coordinates: Vec<[f64; 3]>,
        young: f64,
        poisson: f64,
        thickness: f64,
    ) -> Option<Self> {
        if !matches!(coordinates.len(), 3 | 4) {
            return None;
        }
        let m = Membrane {
            coordinates,
            young,
            poisson,
            thickness,
        };
        m.points().map(|_| m)
    }

    pub fn node_count(&self) -> usize {
        self.coordinates.len()
    }

    pub fn dof_count(&self) -> usize {
        3 * self.coordinates.len()
    }

    fn points(&self) -> Option<Vec<Point>> {
        rule(self.node_count())
            .into_iter()
            .map(|(p, w)| point(&self.coordinates, p, w))
            .collect()
    }

    /// Kinematics of the displacements `u` (3 per node) at a point; small strains measure
    /// the in-plane strain in the reference axes.
    fn kinematics(&self, p: &Point, u: &[f64], nlgeom: bool) -> Kinematics {
        if nlgeom {
            let mut f = [[0.0; 3]; 2];
            for (a, (x, d)) in self.coordinates.iter().zip(&p.dndx).enumerate() {
                for (k, fk) in f.iter_mut().enumerate() {
                    for c in 0..3 {
                        fk[c] += d[k] * (x[c] + u[3 * a + c]);
                    }
                }
            }
            let strain = [
                0.5 * (dot(f[0], f[0]) - 1.0),
                0.5 * (dot(f[1], f[1]) - 1.0),
                dot(f[0], f[1]),
            ];
            Kinematics { f, strain }
        } else {
            let f = [p.frame[0], p.frame[1]];
            let mut strain = [0.0; 3];
            for (a, d) in p.dndx.iter().enumerate() {
                let ua = [u[3 * a], u[3 * a + 1], u[3 * a + 2]];
                strain[0] += d[0] * dot(f[0], ua);
                strain[1] += d[1] * dot(f[1], ua);
                strain[2] += d[0] * dot(f[1], ua) + d[1] * dot(f[0], ua);
            }
            Kinematics { f, strain }
        }
    }

    /// Strain-displacement matrix (3 × 3n) of the kinematics.
    fn b_matrix(p: &Point, k: &Kinematics) -> Vec<[f64; 3]> {
        // Per DOF column: its row values for E11, E22, 2 E12.
        let mut b = Vec::with_capacity(3 * p.dndx.len());
        for d in &p.dndx {
            for c in 0..3 {
                b.push([
                    d[0] * k.f[0][c],
                    d[1] * k.f[1][c],
                    d[0] * k.f[1][c] + d[1] * k.f[0][c],
                ]);
            }
        }
        b
    }

    /// Internal nodal forces (into `f`, 3n) and, when asked, the tangent stiffness
    /// (row-major 3n × 3n) at the displacements `u`. Without `nlgeom` the force is `K u`
    /// with the linear stiffness.
    pub fn internal(&self, u: &[f64], nlgeom: bool, f: &mut [f64], k: Option<&mut Vec<f64>>) {
        let ndof = self.dof_count();
        let d = plane_stress(self.young, self.poisson);
        let points = self.points().unwrap_or_default();
        let mut k = k;
        if let Some(k) = k.as_deref_mut() {
            k.clear();
            k.resize(ndof * ndof, 0.0);
        }
        f.iter_mut().for_each(|v| *v = 0.0);
        for p in &points {
            let kin = self.kinematics(p, u, nlgeom);
            let s: [f64; 3] =
                std::array::from_fn(|i| (0..3).map(|j| d[i][j] * kin.strain[j]).sum());
            let b = Self::b_matrix(p, &kin);
            let w = p.da * self.thickness;
            for (fi, bi) in f.iter_mut().zip(&b) {
                *fi += w * (bi[0] * s[0] + bi[1] * s[1] + bi[2] * s[2]);
            }
            // The stiffness against the slope of the normal displacement.
            let alpha = OUT_OF_PLANE * self.young * w;
            let normal = p.frame[2];
            let slope: [f64; 2] = std::array::from_fn(|k| {
                p.dndx
                    .iter()
                    .enumerate()
                    .map(|(a, d)| d[k] * dot(normal, [u[3 * a], u[3 * a + 1], u[3 * a + 2]]))
                    .sum()
            });
            for (a, d) in p.dndx.iter().enumerate() {
                let g = alpha * (d[0] * slope[0] + d[1] * slope[1]);
                for c in 0..3 {
                    f[3 * a + c] += g * normal[c];
                }
            }
            let Some(k) = k.as_deref_mut() else {
                continue;
            };
            let db: Vec<[f64; 3]> = b
                .iter()
                .map(|bj| std::array::from_fn(|i| (0..3).map(|c| d[i][c] * bj[c]).sum()))
                .collect();
            for (i, bi) in b.iter().enumerate() {
                for (j, dbj) in db.iter().enumerate() {
                    k[i * ndof + j] += w * (bi[0] * dbj[0] + bi[1] * dbj[1] + bi[2] * dbj[2]);
                }
            }
            let nodes = self.node_count();
            if nlgeom {
                for a in 0..nodes {
                    let da = p.dndx[a];
                    for c in 0..nodes {
                        let dc = p.dndx[c];
                        let g = w
                            * (s[0] * da[0] * dc[0]
                                + s[1] * da[1] * dc[1]
                                + s[2] * (da[0] * dc[1] + da[1] * dc[0]));
                        for x in 0..3 {
                            k[(3 * a + x) * ndof + 3 * c + x] += g;
                        }
                    }
                }
            }
            for a in 0..nodes {
                for c in 0..nodes {
                    let (da, dc) = (p.dndx[a], p.dndx[c]);
                    let g = alpha * (da[0] * dc[0] + da[1] * dc[1]);
                    for x in 0..3 {
                        for y in 0..3 {
                            k[(3 * a + x) * ndof + 3 * c + y] += g * normal[x] * normal[y];
                        }
                    }
                }
            }
        }
    }

    /// The linear stiffness (row-major 3n × 3n).
    pub fn stiffness(&self) -> Vec<f64> {
        let mut k = Vec::new();
        let mut f = vec![0.0; self.dof_count()];
        self.internal(&vec![0.0; self.dof_count()], false, &mut f, Some(&mut k));
        k
    }

    /// Strain and stress tensors in global axes (`[xx, yy, zz, xy, yz, zx]`, tensor shear)
    /// at the integration points. Small strains: the linear strain and stress. `nlgeom`:
    /// the Green-Lagrange strain (reference axes) and the Cauchy stress.
    pub fn point_values(&self, u: &[f64], nlgeom: bool) -> Vec<([f64; 6], [f64; 6])> {
        let d = plane_stress(self.young, self.poisson);
        let nu = self.poisson;
        self.points()
            .unwrap_or_default()
            .iter()
            .map(|p| {
                let kin = self.kinematics(p, u, nlgeom);
                let e = kin.strain;
                let s: [f64; 3] = std::array::from_fn(|i| (0..3).map(|j| d[i][j] * e[j]).sum());
                let e33 = -nu / (1.0 - nu) * (e[0] + e[1]);
                let strain = [
                    [e[0], e[2] / 2.0, 0.0],
                    [e[2] / 2.0, e[1], 0.0],
                    [0.0, 0.0, e33],
                ];
                let stress_local = [[s[0], s[2], 0.0], [s[2], s[1], 0.0], [0.0, 0.0, 0.0]];
                let r = p.frame;
                let strain = to_global(&strain, &r);
                let stress = if nlgeom {
                    // σ = F S Fᵀ / det F with F = f1 ⊗ e1 + f2 ⊗ e2 + λ3 m ⊗ n.
                    let m = cross(kin.f[0], kin.f[1]);
                    let area = dot(m, m).sqrt();
                    let m = normalize(m);
                    let lambda3 = (1.0 + 2.0 * e33).max(0.0).sqrt();
                    let det = area * lambda3;
                    let cols = [
                        kin.f[0],
                        kin.f[1],
                        [m[0] * lambda3, m[1] * lambda3, m[2] * lambda3],
                    ];
                    // F in global axes: F_ij = Σ_k cols[k]_i r[k]_j; F S Fᵀ with S local
                    // equals Σ_kl cols[k] S_kl cols[l]ᵀ.
                    let mut sigma = [[0.0; 3]; 3];
                    for (i, row) in sigma.iter_mut().enumerate() {
                        for (j, v) in row.iter_mut().enumerate() {
                            for k in 0..3 {
                                for l in 0..3 {
                                    *v += cols[k][i] * stress_local[k][l] * cols[l][j];
                                }
                            }
                            *v /= det;
                        }
                    }
                    [
                        sigma[0][0],
                        sigma[1][1],
                        sigma[2][2],
                        sigma[0][1],
                        sigma[1][2],
                        sigma[2][0],
                    ]
                } else {
                    to_global(&stress_local, &r)
                };
                (strain, stress)
            })
            .collect()
    }

    /// [`Membrane::point_values`] extrapolated to the nodes (constant for the triangle,
    /// bilinear from the 2 × 2 points for the quadrilateral).
    pub fn nodal_values(&self, u: &[f64], nlgeom: bool) -> Vec<([f64; 6], [f64; 6])> {
        let points = self.point_values(u, nlgeom);
        if points.len() == 1 {
            return vec![points[0]; self.node_count()];
        }
        let s = 3.0_f64.sqrt();
        let corners = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
        let at = rule(4);
        corners
            .iter()
            .map(|c| {
                let mut out = ([0.0; 6], [0.0; 6]);
                for ((g, _), v) in at.iter().zip(&points) {
                    // The bilinear function that is 1 at point g and 0 at the others,
                    // evaluated at the corner.
                    let w =
                        0.25 * (1.0 + g[0].signum() * c[0] * s) * (1.0 + g[1].signum() * c[1] * s);
                    for k in 0..6 {
                        out.0[k] += w * v.0[k];
                        out.1[k] += w * v.1[k];
                    }
                }
                out
            })
            .collect()
    }

    /// Consistent mass matrix (row-major 3n × 3n) for the density `rho`, with the element's
    /// own points (three points for the triangle, whose one-point rule would leave it
    /// rank deficient).
    pub fn mass(&self, rho: f64) -> Vec<f64> {
        let nodes = self.node_count();
        let ndof = 3 * nodes;
        let mut m = vec![0.0; ndof * ndof];
        let points: Vec<([f64; 2], f64)> = if nodes == 3 {
            vec![
                ([1.0 / 6.0, 1.0 / 6.0], 1.0 / 6.0),
                ([2.0 / 3.0, 1.0 / 6.0], 1.0 / 6.0),
                ([1.0 / 6.0, 2.0 / 3.0], 1.0 / 6.0),
            ]
        } else {
            rule(nodes)
        };
        for (q, w) in points {
            let (n, dn) = shape(nodes, q);
            let (a1, a2) = tangents(&self.coordinates, &dn);
            let c = cross(a1, a2);
            let f = rho * self.thickness * w * dot(c, c).sqrt();
            for a in 0..nodes {
                for b in 0..nodes {
                    for k in 0..3 {
                        m[(3 * a + k) * ndof + 3 * b + k] += f * n[a] * n[b];
                    }
                }
            }
        }
        m
    }

    /// Nodal forces (into `f`, added) of a body force per unit mass `g(x)` for the density
    /// `rho`, on the reference geometry.
    pub fn body_force(&self, rho: f64, g: impl Fn([f64; 3]) -> [f64; 3], f: &mut [f64]) {
        let nodes = self.node_count();
        for (q, w) in rule(nodes) {
            let (n, dn) = shape(nodes, q);
            let (a1, a2) = tangents(&self.coordinates, &dn);
            let c = cross(a1, a2);
            let mut x = [0.0; 3];
            for (na, xa) in n.iter().zip(&self.coordinates) {
                for k in 0..3 {
                    x[k] += na * xa[k];
                }
            }
            let gx = g(x);
            let factor = rho * self.thickness * w * dot(c, c).sqrt();
            for (a, na) in n.iter().enumerate() {
                for k in 0..3 {
                    f[3 * a + k] += factor * na * gx[k];
                }
            }
        }
    }

    /// Nodal forces (into `f`, added) of a pressure `p` along the element normal
    /// `∂x/∂ξ × ∂x/∂η` (CalculiX's `P`/`P1`: pushing from the bottom side), on the
    /// reference surface or, with `u`, on the deformed one.
    pub fn pressure(&self, u: Option<&[f64]>, p: f64, f: &mut [f64]) {
        let x: Vec<[f64; 3]> = match u {
            Some(u) => self
                .coordinates
                .iter()
                .enumerate()
                .map(|(a, x)| [x[0] + u[3 * a], x[1] + u[3 * a + 1], x[2] + u[3 * a + 2]])
                .collect(),
            None => self.coordinates.clone(),
        };
        for (q, w) in rule(x.len()) {
            let (n, dn) = shape(x.len(), q);
            let (a1, a2) = tangents(&x, &dn);
            let normal = cross(a1, a2);
            for (a, na) in n.iter().enumerate() {
                for c in 0..3 {
                    f[3 * a + c] += p * w * na * normal[c];
                }
            }
        }
    }
}

/// `R m Rᵀ` as `[xx, yy, zz, xy, yz, zx]` for a local tensor `m` and frame rows `r`.
fn to_global(m: &[[f64; 3]; 3], r: &[[f64; 3]; 3]) -> [f64; 6] {
    let g = |i: usize, j: usize| -> f64 {
        let mut v = 0.0;
        for a in 0..3 {
            for c in 0..3 {
                v += m[a][c] * r[a][i] * r[c][j];
            }
        }
        v
    };
    [g(0, 0), g(1, 1), g(2, 2), g(0, 1), g(1, 2), g(2, 0)]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::expand::sub;

    fn tri() -> Membrane {
        Membrane::new(
            vec![[0.0, 0.0, 0.0], [2.0, 0.5, 1.0], [0.3, 1.5, -0.4]],
            210000.0,
            0.3,
            2.0,
        )
        .unwrap()
    }

    fn quad() -> Membrane {
        Membrane::new(
            vec![
                [0.0, 0.0, 0.0],
                [2.0, 0.2, 0.5],
                [2.3, 1.8, 0.4],
                [-0.1, 1.5, 0.1],
            ],
            210000.0,
            0.3,
            2.0,
        )
        .unwrap()
    }

    #[test]
    fn rigid_motions_are_free_and_uniform_strain_is_exact() {
        let m = tri();
        let x = &m.coordinates;
        let k = m.stiffness();
        let w = [0.1, -0.2, 0.3];
        let mut u = [0.0; 9];
        for (n, xn) in x.iter().enumerate() {
            let r = cross(w, *xn);
            for d in 0..3 {
                u[3 * n + d] = r[d] + [1.0, 2.0, 3.0][d];
            }
        }
        let largest = k.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        for i in 0..9 {
            let f: f64 = (0..9).map(|j| k[i * 9 + j] * u[j]).sum();
            // Up to the tiny stiffness against the slope of the normal displacement.
            assert!(f.abs() < 1e-8 * largest, "{i}: {f}");
        }
        // Uniform in-plane stretch along the first edge: uniaxial strain along e1.
        let e1 = normalize(sub(x[1], x[0]));
        let mut u = [0.0; 9];
        for (n, xn) in x.iter().enumerate() {
            let s = 1e-3 * dot(sub(*xn, x[0]), e1);
            for d in 0..3 {
                u[3 * n + d] = s * e1[d];
            }
        }
        let (strain, stress) = m.point_values(&u, false)[0];
        let trace = strain[0] + strain[1] + strain[2];
        // ε11 = 1e-3, ε22 = 0, ε33 = -0.3/0.7e-3.
        assert!((trace - (1e-3 - 0.3 / 0.7 * 1e-3)).abs() < 1e-12);
        let s_trace = stress[0] + stress[1] + stress[2];
        assert!((s_trace - 210000.0 / 0.91 * 1e-3 * 1.3).abs() < 1e-6);
    }

    /// The tangent is the derivative of the internal force for both shapes, in a stretched
    /// and rotated state.
    #[test]
    fn tangent_is_the_derivative_of_the_internal_force() {
        for m in [tri(), quad()] {
            let n = m.dof_count();
            let u: Vec<f64> = (0..n)
                .map(|i| 0.02 * ((i as f64) * 1.3).sin() + 0.05 * ((i % 3) as f64))
                .collect();
            let mut f = vec![0.0; n];
            let mut k = Vec::new();
            m.internal(&u, true, &mut f, Some(&mut k));
            let scale = k.iter().fold(0.0_f64, |a, v| a.max(v.abs()));
            let h = 1e-7;
            for j in 0..n {
                let (mut up, mut um) = (u.clone(), u.clone());
                up[j] += h;
                um[j] -= h;
                let (mut fp, mut fm) = (vec![0.0; n], vec![0.0; n]);
                m.internal(&up, true, &mut fp, None);
                m.internal(&um, true, &mut fm, None);
                for i in 0..n {
                    let fd = (fp[i] - fm[i]) / (2.0 * h);
                    assert!(
                        (fd - k[i * n + j]).abs() < 1e-6 * scale,
                        "{i},{j}: {fd} vs {}",
                        k[i * n + j]
                    );
                }
            }
        }
    }

    /// A rigid rotation leaves no strain and (up to the tiny out-of-plane stiffness) no
    /// force under NLGEOM; out of its plane the unstressed membrane has only that.
    #[test]
    fn rigid_rotation_is_strain_free_and_there_is_no_bending_stiffness() {
        let m = quad();
        let (c, s) = (0.6_f64.cos(), 0.6_f64.sin());
        let u: Vec<f64> = m
            .coordinates
            .iter()
            .flat_map(|x| [c * x[0] - s * x[1] - x[0], s * x[0] + c * x[1] - x[1], 0.0])
            .collect();
        let mut f = vec![0.0; 12];
        m.internal(&u, true, &mut f, None);
        // E t A = 210000 * 2 * 3: forces of a 1e-3 strain would be about 1e3.
        assert!(f.iter().all(|v| v.abs() < 1e-2), "{f:?}");
        let flat = Membrane::new(
            vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            1000.0,
            0.3,
            0.1,
        )
        .unwrap();
        let k = flat.stiffness();
        for a in 0..4 {
            let kzz = k[(3 * a + 2) * 12 + 3 * a + 2];
            let kxx = k[(3 * a) * 12 + 3 * a];
            assert!(kzz > 0.0 && kzz < 1e-8 * kxx, "{kzz} vs {kxx}");
        }
    }

    /// Pressure: total force p A along the normal; uniform in-plane stress is extrapolated
    /// to the nodes unchanged.
    #[test]
    fn pressure_and_nodal_values_of_the_quadrilateral() {
        let m = Membrane::new(
            vec![
                [0.0, 0.0, 0.0],
                [4.0, 0.0, 0.0],
                [4.0, 2.0, 0.0],
                [0.0, 2.0, 0.0],
            ],
            1000.0,
            0.25,
            1.0,
        )
        .unwrap();
        let mut f = vec![0.0; 12];
        m.pressure(None, 0.5, &mut f);
        let fz: f64 = (0..4).map(|a| f[3 * a + 2]).sum();
        assert!((fz - 4.0).abs() < 1e-12);
        let u: Vec<f64> = m
            .coordinates
            .iter()
            .flat_map(|x| [1e-3 * x[0], 0.0, 0.0])
            .collect();
        for (strain, stress) in m.nodal_values(&u, false) {
            assert!((strain[0] - 1e-3).abs() < 1e-14);
            assert!((stress[0] - 1000.0 / (1.0 - 0.0625) * 1e-3).abs() < 1e-10);
        }
    }
}
