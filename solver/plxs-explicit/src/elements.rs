//! The elements as the card computes them, matrix-free: for every element its DOFs, the
//! Cartesian derivatives of the shape functions at its integration points (constant in the
//! total Lagrangian description), its elastic matrix, its bulk viscosity and, for the
//! uniform-strain hexahedron, its hourglass control. Elements of the same shape (nodes,
//! points, hourglass control) form a [`Family`], one dispatch per step.
//!
//! The internal force of an element at a point is `w Σ_c σ_kc ∂N_a/∂X_c` on DOF `(a, k)`
//! with the small strain of the displacements relative to the element's first node, the
//! viscous force `c V (g · v) g` plus the hourglass viscosity. Under NLGEOM the stress is
//! `P = F S` with `S = D E` of the Green-Lagrange strain `E`, and the viscosity takes the
//! volume and the mean gradients of the current configuration: from the points' `F` for
//! the fully integrated elements (`J F⁻ᵀ ∇N`), from the current nodal coordinates with
//! 2 × 2 × 2 points for the uniform-strain hexahedron, as the processor does. [`Elements::forces`] is
//! the same in double precision on the processor, the reference of the tests.

// Index loops read as the tensor formulas they implement.
#![allow(clippy::needless_range_loop)]

/// The hourglass control of one uniform-strain hexahedron (C3D8R).
#[derive(Clone, Debug, PartialEq)]
pub struct Hourglass {
    /// `ε` of the stiffness `ε Σ_α γ_α γ_αᵀ ⊗ I` with `γ_α = h_α − (h_α · z) ∂N/∂z`.
    pub stiffness: f64,
    /// `κ` of the viscosity `κ Σ_α γ_α (γ_α · v)` with `γ_α = h_α − Σ_j (h_α · x_j) g_j`.
    pub viscosity: f64,
    /// The nodal coordinates.
    pub coordinates: Vec<[f64; 3]>,
}

/// State variables per integration point (those of the processor's J2 law: equivalent
/// plastic strain, plastic strain, back stress, creep rate).
pub const STATE_SIZE: usize = 14;

/// A material law of the kernels.
// A model has a handful of materials: their size does not matter.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug, PartialEq)]
pub enum Material {
    /// `σ = D ε` (Saint Venant-Kirchhoff under NLGEOM).
    Elastic([[f64; 6]; 6]),
    /// Small-strain J2 plasticity, isotropic and linear kinematic hardening (radial return
    /// as the processor's `J2Plastic`, without creep).
    J2 {
        shear: f64,
        bulk: f64,
        /// Yield stress against equivalent plastic strain.
        isotropic: Vec<(f64, f64)>,
        /// The back stress's uniaxial stress against equivalent plastic strain, or empty.
        kinematic: Vec<(f64, f64)>,
        /// At finite strains (the processor's `FiniteStrainJ2`, under NLGEOM): Hencky
        /// elasticity in the logarithmic elastic strain, the return mapping on the Kirchhoff
        /// stress, the plastic slots of the state holding `C_p⁻¹ − 1`.
        finite: bool,
    },
    /// A hyperelastic strain energy (the processor's `Hyperelastic`) of the Green-Lagrange
    /// strain: the second Piola-Kirchhoff stress, no state variables.
    Hyperelastic {
        isochoric: Isochoric,
        volumetric: Volumetric,
    },
}

/// The isochoric part of a hyperelastic strain energy, as `*HYPERELASTIC` defines it.
#[derive(Clone, Debug, PartialEq)]
pub enum Isochoric {
    /// `Σ C_ij (Ī1 − 3)^i (Ī2 − 3)^j` as `(i, j, C_ij)` (at most nine terms, `i + j ≤ 3`).
    Polynomial(Vec<(i32, i32, f64)>),
    /// Arruda-Boyce: `μ Σ_k c_k / λm^(2k−2) (Ī1^k − 3^k)`, five terms.
    ArrudaBoyce { mu: f64, locking: f64 },
    /// Ogden: `Σ 2 μ_p / α_p² (Σ_a λ̄_a^α_p − 3)` as `(μ_p, α_p)` (at most three).
    Ogden(Vec<(f64, f64)>),
}

/// The volumetric part `U(J)` of a hyperelastic strain energy.
#[derive(Clone, Debug, PartialEq)]
pub enum Volumetric {
    /// `Σ (J − 1)^(2k) / D_k` holding `1 / D_k` (at most three).
    Polynomial(Vec<f64>),
    /// `((J² − 1) / 2 − ln J) / D` holding `1 / D`.
    ArrudaBoyce(f64),
}

/// The coefficients of the inverse Langevin series in the Arruda-Boyce energy.
pub(crate) const ARRUDA_BOYCE: [f64; 5] = [
    0.5,
    1.0 / 20.0,
    11.0 / 1050.0,
    19.0 / 7000.0,
    519.0 / 673_750.0,
];

/// Value and slope of a piecewise linear curve, constant beyond its ends.
fn curve(points: &[(f64, f64)], x: f64) -> (f64, f64) {
    let Some(&(x0, y0)) = points.first() else {
        return (0.0, 0.0);
    };
    if x <= x0 {
        return (y0, if x < x0 { 0.0 } else { slope_after(points) });
    }
    for w in points.windows(2) {
        if x <= w[1].0 {
            let slope = (w[1].1 - w[0].1) / (w[1].0 - w[0].0);
            return (w[0].1 + (x - w[0].0) * slope, slope);
        }
    }
    (points[points.len() - 1].1, 0.0)
}

fn slope_after(points: &[(f64, f64)]) -> f64 {
    match points {
        [a, b, ..] => (b.1 - a.1) / (b.0 - a.0),
        _ => 0.0,
    }
}

impl Material {
    /// The stress of the engineering strain; `state` (J2) advances.
    pub fn stress(&self, strain: &[f64; 6], state: Option<&mut [f64]>) -> [f64; 6] {
        match self {
            Material::Elastic(d) => {
                std::array::from_fn(|r| d[r].iter().zip(strain).map(|(a, b)| a * b).sum())
            }
            Material::Hyperelastic {
                isochoric,
                volumetric,
            } => hyperelastic(isochoric, volumetric, strain),
            Material::J2 {
                shear,
                bulk,
                isotropic,
                kinematic,
                finite,
            } => {
                let state = state.expect("J2 has state variables");
                let law = Radial {
                    mu: *shear,
                    bulk: *bulk,
                    isotropic,
                    kinematic,
                };
                if *finite {
                    law.finite(strain, state)
                } else {
                    law.small(strain, state)
                }
            }
        }
    }
}

/// The radial return of `J2Plastic` (rate independent).
struct Radial<'a> {
    mu: f64,
    bulk: f64,
    isotropic: &'a [(f64, f64)],
    kinematic: &'a [(f64, f64)],
}

type Mat3 = [[f64; 3]; 3];

impl Radial<'_> {
    /// The small-strain law of the engineering strain.
    fn small(&self, strain: &[f64; 6], state: &mut [f64]) -> [f64; 6] {
        let e = strain;
        let elastic: [f64; 6] = std::array::from_fn(|k| {
            let total = if k < 3 { e[k] } else { 0.5 * e[k] };
            total - state[1 + k]
        });
        let (stress, increment) = self.ret(&elastic, state);
        if let Some(increment) = increment {
            for k in 0..6 {
                state[1 + k] += increment[k];
            }
        }
        stress
    }

    /// The finite-strain law of the Green-Lagrange strain (engineering components): the
    /// second Piola-Kirchhoff stress. The stretch `U = 1 + A`, its inverse `1 + A'`, the
    /// trial `b_e = U C_p⁻¹ U` and the new `C_p⁻¹ = U⁻¹ b_e U⁻¹` are formed from their
    /// differences to the identity, as the card does in single precision.
    fn finite(&self, strain: &[f64; 6], state: &mut [f64]) -> [f64; 6] {
        let e = strain;
        let two_e = [
            [2.0 * e[0], e[3], e[5]],
            [e[3], 2.0 * e[1], e[4]],
            [e[5], e[4], 2.0 * e[2]],
        ];
        let a = spectral(&two_e, |l| l / (1.0 + (1.0 + l).sqrt()));
        let ai = spectral(&two_e, |l| {
            let s = (1.0 + l).sqrt();
            -l / (1.0 + s) / s
        });
        let p = matrix(&std::array::from_fn(|k| state[1 + k]));
        let b = around(&a, &p);
        let trial = vector(&spectral(&b, |l| 0.5 * l.ln_1p()));
        let (tau, increment) = self.ret(&trial, state);
        if let Some(increment) = increment {
            let elastic: [f64; 6] = std::array::from_fn(|k| trial[k] - increment[k]);
            let be = spectral(&matrix(&elastic), |l| (2.0 * l).exp_m1());
            let cp = vector(&around(&ai, &be));
            state[1..7].copy_from_slice(&cp);
        }
        let tau = matrix(&tau);
        let mut s = tau;
        let at = multiply(&ai, &tau);
        let ata = multiply(&at, &ai);
        for i in 0..3 {
            for j in 0..3 {
                s[i][j] += at[i][j] + at[j][i] + ata[i][j];
            }
        }
        vector(&s)
    }

    /// The stress of the elastic trial strain (tensor components) and, where it flows, the
    /// plastic strain increment; the equivalent plastic strain and the back stress advance.
    fn ret(&self, elastic: &[f64; 6], state: &mut [f64]) -> ([f64; 6], Option<[f64; 6]>) {
        let (mu, isotropic, kinematic) = (self.mu, self.isotropic, self.kinematic);
        let trace = elastic[0] + elastic[1] + elastic[2];
        let mean = trace / 3.0;
        let s_trial: [f64; 6] =
            std::array::from_fn(|k| 2.0 * mu * (elastic[k] - if k < 3 { mean } else { 0.0 }));
        let xi: [f64; 6] = std::array::from_fn(|k| s_trial[k] - state[7 + k]);
        let xi_norm = (xi[0] * xi[0]
            + xi[1] * xi[1]
            + xi[2] * xi[2]
            + 2.0 * (xi[3] * xi[3] + xi[4] * xi[4] + xi[5] * xi[5]))
            .sqrt();
        let q = 1.5_f64.sqrt() * xi_norm;
        let peeq = state[0];
        let yield_stress = curve(isotropic, peeq).0;
        let hydrostatic = self.bulk * trace;
        let mut stress = s_trial;
        let mut flow = None;
        if q - yield_stress > 1e-9 * yield_stress.max(0.0) && xi_norm > 0.0 {
            let kappa0 = curve(kinematic, peeq).0;
            let overstress = |dg: f64| {
                q - 3.0 * mu * dg
                    - (curve(kinematic, peeq + dg).0 - kappa0)
                    - curve(isotropic, peeq + dg).0
            };
            let hardening = |dg: f64| curve(kinematic, peeq + dg).1 + curve(isotropic, peeq + dg).1;
            let (mut lo, mut hi) = (0.0, q / (3.0 * mu));
            let mut dg = (q - yield_stress) / (3.0 * mu + hardening(0.0));
            for _ in 0..100 {
                let g = overstress(dg);
                if g.abs() <= 1e-12 * q.max(1.0) {
                    break;
                }
                if g > 0.0 {
                    lo = dg;
                } else {
                    hi = dg;
                }
                let next = dg + g / (3.0 * mu + hardening(dg));
                dg = if next > lo && next < hi {
                    next
                } else {
                    0.5 * (lo + hi)
                };
            }
            let dg = dg.clamp(0.0, q / (3.0 * mu));
            let delta_kappa = curve(kinematic, peeq + dg).0 - kappa0;
            let mut increment = [0.0; 6];
            for k in 0..6 {
                let n = xi[k] / xi_norm;
                increment[k] = dg * 1.5_f64.sqrt() * n;
                state[7 + k] += delta_kappa * (2.0_f64 / 3.0).sqrt() * n;
                stress[k] -= 2.0 * mu * increment[k];
            }
            state[0] = peeq + dg;
            flow = Some(increment);
        }
        for v in stress.iter_mut().take(3) {
            *v += hydrostatic;
        }
        (stress, flow)
    }
}

/// The second Piola-Kirchhoff stress of a hyperelastic energy for the Green-Lagrange strain
/// (engineering components), as the processor's `Hyperelastic` (Holzapfel, sections 6.4 and
/// 6.5) but formed from `M = C − 1`, so that single precision on the card holds small
/// strains: `det C − 1 = tr M + I₂(M) + det M`, `C⁻¹ − 1 = (M² − (1 + tr M) M − det M) / det C`
/// (Cayley-Hamilton) and the isochoric stress written without the identity's share.
fn hyperelastic(isochoric: &Isochoric, volumetric: &Volumetric, e: &[f64; 6]) -> [f64; 6] {
    let m = [
        [2.0 * e[0], e[3], e[5]],
        [e[3], 2.0 * e[1], e[4]],
        [e[5], e[4], 2.0 * e[2]],
    ];
    let mm = multiply(&m, &m);
    let tr = m[0][0] + m[1][1] + m[2][2];
    let tr2 = mm[0][0] + mm[1][1] + mm[2][2];
    let i2 = 0.5 * (tr * tr - tr2);
    let det_m = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    let dm1 = tr + i2 + det_m;
    let det = 1.0 + dm1;
    if det.is_nan() || det <= 0.0 {
        return [f64::NAN; 6];
    }
    let j = det.sqrt();
    let jm1 = dm1 / (1.0 + j);
    let pressure = j * match volumetric {
        Volumetric::Polynomial(inverse) => (inverse.iter().enumerate())
            .map(|(k, a)| {
                let power = 2 * k as i32 + 2;
                a * f64::from(power) * jm1.powi(power - 1)
            })
            .sum(),
        Volumetric::ArrudaBoyce(a) => a * jm1 * (j + 1.0) / j,
    };
    let log_j = 0.5 * dm1.ln_1p();
    let n: Mat3 = std::array::from_fn(|a| {
        std::array::from_fn(|b| {
            let identity = if a == b { 1.0 } else { 0.0 };
            (mm[a][b] - (1.0 + tr) * m[a][b] - det_m * identity) / det
        })
    });
    let s: Mat3 = match isochoric {
        Isochoric::Ogden(terms) => {
            let (values, v) = eigen(&m);
            let mut tau = [pressure; 3];
            for &(mu, alpha) in terms {
                let p = values.map(|l| (alpha * (0.5 * l.ln_1p() - log_j / 3.0)).exp_m1());
                let mean = (p[0] + p[1] + p[2]) / 3.0;
                for a in 0..3 {
                    tau[a] += 2.0 * mu / alpha * (p[a] - mean);
                }
            }
            std::array::from_fn(|r| {
                std::array::from_fn(|q| {
                    (0..3)
                        .map(|a| tau[a] / (1.0 + values[a]) * v[r][a] * v[q][a])
                        .sum()
                })
            })
        }
        invariant => {
            let j23m1 = (-2.0 / 3.0 * log_j).exp_m1();
            let j43m1 = (-4.0 / 3.0 * log_j).exp_m1();
            let j23 = 1.0 + j23m1;
            // Ī1 − 3 and Ī2 − 3.
            let x = 3.0 * j23m1 + j23 * tr;
            let y = 3.0 * j43m1 + (1.0 + j43m1) * (2.0 * tr + i2);
            let (w1, w2) = match invariant {
                Isochoric::Polynomial(terms) => {
                    let (mut w1, mut w2) = (0.0, 0.0);
                    for &(i, k, c) in terms {
                        if i > 0 {
                            w1 += c * f64::from(i) * x.powi(i - 1) * y.powi(k);
                        }
                        if k > 0 {
                            w2 += c * f64::from(k) * x.powi(i) * y.powi(k - 1);
                        }
                    }
                    (w1, w2)
                }
                Isochoric::ArrudaBoyce { mu, locking } => {
                    let i1 = 3.0 + x;
                    let w1 = (ARRUDA_BOYCE.iter().enumerate())
                        .map(|(k, c)| {
                            mu * c / locking.powi(2 * k as i32) * (k + 1) as f64 * i1.powi(k as i32)
                        })
                        .sum();
                    (w1, 0.0)
                }
                Isochoric::Ogden(_) => unreachable!(),
            };
            // S̄ = a 1 − b C; S = J^(−2/3) (S̄ − ⅓ (S̄ : C) C⁻¹) + p C⁻¹.
            let a = 2.0 * (w1 + (3.0 + x) * w2);
            let b = 2.0 * w2 * j23;
            let share = b / 3.0 * (tr + tr2) - (a - b) * tr / 3.0;
            std::array::from_fn(|r| {
                std::array::from_fn(|q| {
                    let identity = if r == q { 1.0 } else { 0.0 };
                    let dev = -(a - b) * n[r][q] - b * m[r][q] + share * (identity + n[r][q]);
                    j23 * dev + pressure * (identity + n[r][q])
                })
            })
        }
    };
    vector(&s)
}

/// The symmetric matrix of tensor components `[xx, yy, zz, xy, yz, zx]`.
fn matrix(t: &[f64; 6]) -> Mat3 {
    [[t[0], t[3], t[5]], [t[3], t[1], t[4]], [t[5], t[4], t[2]]]
}

fn vector(m: &Mat3) -> [f64; 6] {
    [m[0][0], m[1][1], m[2][2], m[0][1], m[1][2], m[2][0]]
}

fn multiply(a: &Mat3, b: &Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}

/// `(1 + a) (1 + b) (1 + a) − 1` of symmetric `a`, `b`, without forming the identity.
fn around(a: &Mat3, b: &Mat3) -> Mat3 {
    let aa = multiply(a, a);
    let ab = multiply(a, b);
    let aba = multiply(&ab, a);
    std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            2.0 * a[i][j] + b[i][j] + aa[i][j] + ab[i][j] + ab[j][i] + aba[i][j]
        })
    })
}

/// `f` of the eigenvalues of a symmetric matrix (cyclic Jacobi rotations, as the card).
fn spectral(m: &Mat3, f: impl Fn(f64) -> f64) -> Mat3 {
    let (values, v) = eigen(m);
    let fa = values.map(f);
    std::array::from_fn(|i| {
        std::array::from_fn(|j| (0..3).map(|a| fa[a] * v[i][a] * v[j][a]).sum())
    })
}

/// Eigenvalues and eigenvectors (columns) of a symmetric matrix.
fn eigen(m: &Mat3) -> ([f64; 3], Mat3) {
    let mut a = *m;
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        let scale = a[0][0].abs() + a[1][1].abs() + a[2][2].abs() + off;
        if off <= 1e-15 * scale {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q] == 0.0 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = if theta == 0.0 {
                1.0
            } else {
                theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt())
            };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for row in a.iter_mut() {
                let (akp, akq) = (row[p], row[q]);
                row[p] = c * akp - s * akq;
                row[q] = s * akp + c * akq;
            }
            for k in 0..3 {
                let (apk, aqk) = (a[p][k], a[q][k]);
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
            for row in v.iter_mut() {
                let (vkp, vkq) = (row[p], row[q]);
                row[p] = c * vkp - s * vkq;
                row[q] = s * vkp + c * vkq;
            }
        }
    }
    ([a[0][0], a[1][1], a[2][2]], v)
}

/// One element.
#[derive(Clone, Debug, PartialEq)]
pub struct Element {
    /// Three per node.
    pub dofs: Vec<u32>,
    /// Per integration point: the volume it stands for and the shape function derivatives
    /// per node.
    pub points: Vec<(f64, Vec<[f64; 3]>)>,
    pub material: Material,
    /// With a material that has state variables: the index of the first one of its first
    /// point in the material state ([`STATE_SIZE`] per point, the points in a row).
    pub state: Option<usize>,
    /// The bulk viscosity `c V` and the mean gradients `g` (three per node).
    pub bulk: f64,
    pub mean: Vec<f64>,
    /// Only with eight nodes and one point.
    pub hourglass: Option<Hourglass>,
}

/// Elements of one shape, element by element.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Family {
    pub nodes: usize,
    pub points: usize,
    pub hourglass: bool,
    pub count: usize,
    /// `count × 3 nodes`.
    pub dofs: Vec<u32>,
    /// Per element and point the volume, then the derivatives node by node:
    /// `count × points × (1 + 3 nodes)`.
    pub gradients: Vec<f64>,
    /// Per element `c V`, then `g`: `count × (1 + 3 nodes)`.
    pub viscosity: Vec<f64>,
    /// Per element `ε`, `κ`, then the coordinates relative to the first node:
    /// `count × (2 + 3 nodes)`, empty without hourglass control.
    pub control: Vec<f64>,
    /// Per element its index into [`Elements::materials`].
    pub material: Vec<u32>,
    /// Whether an element has state variables, and per element the index of its first one
    /// (`u32::MAX` for elements without).
    pub history: bool,
    pub state: Vec<u32>,
}

impl Family {
    /// Values per element and point in [`Family::gradients`].
    pub fn point_size(&self) -> usize {
        1 + 3 * self.nodes
    }
}

/// The elements of a model.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Elements {
    pub families: Vec<Family>,
    /// The different materials.
    pub materials: Vec<Material>,
    /// Large deformations (total Lagrangian: Green-Lagrange strain, Saint Venant-Kirchhoff
    /// stress, the bulk and hourglass viscosity in the current configuration).
    pub nlgeom: bool,
}

/// The hourglass base vectors of the eight-node brick (the same set as the elements').
pub(crate) const HOURGLASS_BASE: [[f64; 8]; 4] = [
    [1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0],
    [1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0],
    [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0],
    [-1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0],
];

impl Elements {
    /// Adds an element to the family of its shape. `Err` when it does not fit the kernels.
    pub fn push(&mut self, element: Element) -> Result<(), String> {
        let nodes = element.dofs.len() / 3;
        let points = element.points.len();
        if nodes == 0
            || element.dofs.len() != 3 * nodes
            || points == 0
            || element.points.iter().any(|(_, g)| g.len() != nodes)
            || element.mean.len() != 3 * nodes
        {
            return Err("an element of unexpected shape".to_owned());
        }
        if nodes > MAX_NODES {
            return Err(format!("an element of {nodes} nodes"));
        }
        let hourglass = element.hourglass.is_some();
        if let Some(h) = &element.hourglass
            && (nodes != 8 || points != 1 || h.coordinates.len() != 8)
        {
            return Err("hourglass control for an element other than C3D8R".to_owned());
        }
        let state = match (&element.material, element.state) {
            (Material::Elastic(_) | Material::Hyperelastic { .. }, _) => u32::MAX,
            (Material::J2 { .. }, Some(state)) if state < u32::MAX as usize => state as u32,
            (Material::J2 { .. }, _) => {
                return Err("a plastic element without state variables".to_owned());
            }
        };
        let material = match self.materials.iter().position(|m| *m == element.material) {
            Some(i) => i,
            None => {
                self.materials.push(element.material);
                self.materials.len() - 1
            }
        } as u32;
        let family = match self
            .families
            .iter()
            .position(|f| f.nodes == nodes && f.points == points && f.hourglass == hourglass)
        {
            Some(i) => &mut self.families[i],
            None => {
                self.families.push(Family {
                    nodes,
                    points,
                    hourglass,
                    ..Default::default()
                });
                self.families.last_mut().expect("pushed")
            }
        };
        family.count += 1;
        family.dofs.extend_from_slice(&element.dofs);
        for (w, g) in &element.points {
            family.gradients.push(*w);
            family.gradients.extend(g.iter().flatten());
        }
        family.viscosity.push(element.bulk);
        family.viscosity.extend_from_slice(&element.mean);
        family.material.push(material);
        family.state.push(state);
        family.history |= state != u32::MAX;
        if let Some(h) = element.hourglass {
            family.control.push(h.stiffness);
            family.control.push(h.viscosity);
            let x0 = h.coordinates[0];
            for x in &h.coordinates {
                family.control.extend((0..3).map(|j| x[j] - x0[j]));
            }
        }
        Ok(())
    }

    /// The number of elements.
    pub fn len(&self) -> usize {
        self.families.iter().map(|f| f.count).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The internal and the viscous forces at the displacements `u` and velocities `v`,
    /// in double precision; the material state advances in place.
    pub fn forces(&self, u: &[f64], v: &[f64], state: &mut [f64]) -> (Vec<f64>, Vec<f64>) {
        let mut internal = vec![0.0; u.len()];
        let mut viscous = vec![0.0; u.len()];
        for family in &self.families {
            let nd = 3 * family.nodes;
            for e in 0..family.count {
                let dofs = &family.dofs[e * nd..(e + 1) * nd];
                let ue: Vec<f64> = dofs
                    .iter()
                    .enumerate()
                    .map(|(k, &d)| u[d as usize] - u[dofs[k % 3] as usize])
                    .collect();
                let ve: Vec<f64> = dofs.iter().map(|&d| v[d as usize]).collect();
                let mut f = vec![0.0; nd];
                let mut fv = vec![0.0; nd];
                let material = &self.materials[family.material[e] as usize];
                let size = family.point_size();
                // The current volume and `V g` of the fully integrated elements.
                let mut current = (0.0, vec![0.0; nd]);
                let mut reference = 0.0;
                for ip in 0..family.points {
                    let at = (e * family.points + ip) * size;
                    let w = family.gradients[at];
                    let g = &family.gradients[at + 1..at + size];
                    reference += w;
                    let history = (family.state[e] != u32::MAX).then(|| {
                        let at = family.state[e] as usize + ip * STATE_SIZE;
                        &mut state[at..at + STATE_SIZE]
                    });
                    let stress = |strain: &[f64; 6]| material.stress(strain, history);
                    point_forces(w, g, &ue, stress, self.nlgeom, &mut f, &mut current);
                }
                let visc = &family.viscosity[e * (1 + nd)..(e + 1) * (1 + nd)];
                let control = family
                    .hourglass
                    .then(|| &family.control[e * (2 + nd)..(e + 1) * (2 + nd)]);
                let moved: Vec<f64>;
                let (volume, mean): (f64, &[f64]) = match (self.nlgeom, control) {
                    (false, _) => (reference, &visc[1..]),
                    (true, Some(control)) => {
                        let x: Vec<[f64; 3]> = (0..8)
                            .map(|a| {
                                std::array::from_fn(|j| control[2 + 3 * a + j] + ue[3 * a + j])
                            })
                            .collect();
                        let (volume, g) = brick_mean_gradients(&x);
                        moved = g;
                        (volume, &moved)
                    }
                    (true, None) => {
                        moved = current.1.iter().map(|g| g / current.0).collect();
                        (current.0, &moved)
                    }
                };
                let q = visc[0] / reference
                    * volume
                    * mean.iter().zip(&ve).map(|(g, v)| g * v).sum::<f64>();
                for (o, g) in fv.iter_mut().zip(mean) {
                    *o += q * g;
                }
                if let Some(control) = control {
                    let g = &family.gradients[e * size + 1..(e + 1) * size];
                    hourglass_forces(control, g, mean, &ue, &ve, self.nlgeom, &mut f, &mut fv);
                }
                for (k, &dof) in dofs.iter().enumerate() {
                    internal[dof as usize] += f[k];
                    viscous[dof as usize] += fv[k];
                }
            }
        }
        (internal, viscous)
    }
}

/// The most nodes of an element the kernels take (C3D20).
pub const MAX_NODES: usize = 20;

/// Adds `w Bᵀ σ` of one point with the derivatives `g` (three per node) to `f`, and under
/// NLGEOM the current volume `w J` and `w J F⁻ᵀ ∇N` to `current`.
fn point_forces(
    w: f64,
    g: &[f64],
    ue: &[f64],
    stress: impl FnOnce(&[f64; 6]) -> [f64; 6],
    nlgeom: bool,
    f: &mut [f64],
    current: &mut (f64, Vec<f64>),
) {
    let nodes = g.len() / 3;
    let mut h = [[0.0; 3]; 3];
    for a in 0..nodes {
        for (k, hk) in h.iter_mut().enumerate() {
            for (c, hkc) in hk.iter_mut().enumerate() {
                *hkc += ue[3 * a + k] * g[3 * a + c];
            }
        }
    }
    // `Σ_k H_ki H_kj` under NLGEOM.
    let hh = |i: usize, j: usize| {
        if nlgeom {
            (0..3).map(|k| h[k][i] * h[k][j]).sum::<f64>()
        } else {
            0.0
        }
    };
    let strain = [
        h[0][0] + 0.5 * hh(0, 0),
        h[1][1] + 0.5 * hh(1, 1),
        h[2][2] + 0.5 * hh(2, 2),
        h[0][1] + h[1][0] + hh(0, 1),
        h[1][2] + h[2][1] + hh(1, 2),
        h[2][0] + h[0][2] + hh(2, 0),
    ];
    let s = stress(&strain);
    let sm = [[s[0], s[3], s[5]], [s[3], s[1], s[4]], [s[5], s[4], s[2]]];
    // P = F S (F = I under small strains).
    let mut p = sm;
    let mut fm = [[0.0; 3]; 3];
    if nlgeom {
        for k in 0..3 {
            for c in 0..3 {
                fm[k][c] = f64::from(u8::from(k == c)) + h[k][c];
            }
        }
        for k in 0..3 {
            for c in 0..3 {
                p[k][c] = (0..3).map(|m| fm[k][m] * sm[m][c]).sum();
            }
        }
    }
    for a in 0..nodes {
        for k in 0..3 {
            f[3 * a + k] += w * (0..3).map(|c| p[k][c] * g[3 * a + c]).sum::<f64>();
        }
    }
    if nlgeom {
        // J F⁻ᵀ is the cofactor matrix of F.
        let cof = cofactors(&fm);
        let det: f64 = (0..3).map(|c| fm[0][c] * cof[0][c]).sum();
        current.0 += w * det;
        for a in 0..nodes {
            for k in 0..3 {
                current.1[3 * a + k] += w * (0..3).map(|c| cof[k][c] * g[3 * a + c]).sum::<f64>();
            }
        }
    }
}

/// The cofactor matrix `det(m) m⁻ᵀ`.
fn cofactors(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut c = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            let (i1, i2) = ((i + 1) % 3, (i + 2) % 3);
            let (j1, j2) = ((j + 1) % 3, (j + 2) % 3);
            c[i][j] = m[i1][j1] * m[i2][j2] - m[i1][j2] * m[i2][j1];
        }
    }
    c
}

/// The trilinear brick's natural node coordinates.
pub(crate) const CORNERS: [[f64; 3]; 8] = [
    [-1.0, -1.0, -1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, -1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [1.0, -1.0, 1.0],
    [1.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0],
];

/// Volume and volume mean shape function derivatives (three per node) of a trilinear
/// brick at the nodal positions `x`, with 2 × 2 × 2 points (exact).
fn brick_mean_gradients(x: &[[f64; 3]]) -> (f64, Vec<f64>) {
    let p = 1.0 / 3.0_f64.sqrt();
    let mut volume = 0.0;
    let mut sum = vec![0.0; 24];
    for gauss in &CORNERS {
        let xi = [p * gauss[0], p * gauss[1], p * gauss[2]];
        // dN_a/dξ_m.
        let dn: Vec<[f64; 3]> = CORNERS
            .iter()
            .map(|c| {
                let f = |i: usize| 1.0 + c[i] * xi[i];
                [
                    c[0] * f(1) * f(2) / 8.0,
                    c[1] * f(0) * f(2) / 8.0,
                    c[2] * f(0) * f(1) / 8.0,
                ]
            })
            .collect();
        // J_km = ∂x_k/∂ξ_m.
        let mut j = [[0.0; 3]; 3];
        for (xa, da) in x.iter().zip(&dn) {
            for k in 0..3 {
                for m in 0..3 {
                    j[k][m] += xa[k] * da[m];
                }
            }
        }
        let cof = cofactors(&j);
        volume += (0..3).map(|m| j[0][m] * cof[0][m]).sum::<f64>();
        // det J ∂N/∂x = cof(J) ∂N/∂ξ.
        for (a, da) in dn.iter().enumerate() {
            for k in 0..3 {
                sum[3 * a + k] += (0..3).map(|m| cof[k][m] * da[m]).sum::<f64>();
            }
        }
    }
    sum.iter_mut().for_each(|g| *g /= volume);
    (volume, sum)
}

/// The hourglass stiffness and viscosity of a C3D8R: `control` is `ε`, `κ` and the
/// relative coordinates, `g` the point's derivatives, `mean` the mean gradients (of the
/// current configuration under NLGEOM, where the viscosity also takes the current
/// coordinates).
#[allow(clippy::too_many_arguments)]
fn hourglass_forces(
    control: &[f64],
    g: &[f64],
    mean: &[f64],
    ue: &[f64],
    ve: &[f64],
    nlgeom: bool,
    f: &mut [f64],
    fv: &mut [f64],
) {
    let (epsilon, kappa, x) = (control[0], control[1], &control[2..]);
    let moved: Vec<f64> = x
        .iter()
        .zip(ue)
        .map(|(x, u)| if nlgeom { x + u } else { *x })
        .collect();
    for h in &HOURGLASS_BASE {
        let hx: Vec<f64> = (0..3)
            .map(|j| (0..8).map(|a| h[a] * x[3 * a + j]).sum())
            .collect();
        let hm: Vec<f64> = (0..3)
            .map(|j| (0..8).map(|a| h[a] * moved[3 * a + j]).sum())
            .collect();
        let stiff: Vec<f64> = (0..8).map(|a| h[a] - hx[2] * g[3 * a + 2]).collect();
        let visc: Vec<f64> = (0..8)
            .map(|a| h[a] - (0..3).map(|j| hm[j] * mean[3 * a + j]).sum::<f64>())
            .collect();
        for k in 0..3 {
            let q: f64 = (0..8).map(|a| stiff[a] * ue[3 * a + k]).sum();
            let r: f64 = (0..8).map(|a| visc[a] * ve[3 * a + k]).sum();
            for a in 0..8 {
                f[3 * a + k] += epsilon * stiff[a] * q;
                fv[3 * a + k] += kappa * visc[a] * r;
            }
        }
    }
}
