//! Isotropic hyperelasticity (`*HYPERELASTIC`): the strain energy models of rubber-like
//! materials with the split of the energy into an isochoric and a volumetric part.
//!
//! The elements hand in the Green-Lagrange strain `E`; the law forms `C = 1 + 2E` and
//! returns the second Piola-Kirchhoff stress `S = 2 ∂W/∂C`:
//!
//! - invariant based models (polynomial, Mooney-Rivlin, Neo-Hooke, Yeoh, Arruda-Boyce):
//!   `S̄ = 2 [(W₁ + Ī1 W₂) 1 - W₂ C̄]`, `S = J^(-2/3) (S̄ - ⅓ (S̄ : C) C⁻¹) + J U'(J) C⁻¹`
//!   (Holzapfel, *Nonlinear Solid Mechanics*, 2000, section 6.4);
//! - the Ogden model in principal stretches: Kirchhoff principal stresses
//!   `τ_a = Σ_p 2 μ_p / α_p (λ̄_a^α_p - ⅓ Σ_b λ̄_b^α_p) + J U'(J)` and
//!   `S = Σ_a τ_a / λ_a² N_a ⊗ N_a` (Ogden, *Non-Linear Elastic Deformations*, 1984;
//!   Holzapfel, section 6.5).
//!
//! The strain energies and the meaning of the constants follow the CalculiX manual
//! (`*HYPERELASTIC`), so constants fitted for CalculiX or Abaqus carry over. The tangent
//! `dS/dE` is taken by central differences of the stress (twelve extra evaluations per
//! integration point, which cost little next to the element). No CalculiX code was
//! consulted.
//!
//! In geometrically linear steps the law sees the small strain in place of `E`, which is
//! the linear elastic law with the initial moduli for small strains; the procedures switch
//! NLGEOM on for steps that deform hyperelastic materials, as CalculiX does.

// Index loops over 3 × 3 and 6 × 6 tensors read like the formulas.
#![allow(clippy::needless_range_loop)]

use plxs_core::material::{Hyperelastic as Definition, HyperelasticModel, Material};
use plxs_core::{Error, Result};

use crate::finite::eigen_symmetric;
use crate::{LinearElastic, MaterialInput, MaterialLaw, MaterialOutput};

type Mat3 = [[f64; 3]; 3];

/// The coefficients of the first five terms of the inverse Langevin series in the
/// Arruda-Boyce potential.
const ARRUDA_BOYCE: [f64; 5] = [
    0.5,
    1.0 / 20.0,
    11.0 / 1050.0,
    19.0 / 7000.0,
    519.0 / 673_750.0,
];

/// The volumetric part `U(J)` of the strain energy.
#[derive(Clone, Debug, PartialEq)]
enum Volumetric {
    /// `Σ (J - 1)^(2k) / D_k`, holding `1 / D_k` (zero for a missing term).
    Polynomial(Vec<f64>),
    /// `((J² - 1) / 2 - ln J) / D`, holding `1 / D`.
    ArrudaBoyce(f64),
}

impl Volumetric {
    fn energy(&self, j: f64) -> f64 {
        match self {
            Self::Polynomial(inverse) => (inverse.iter().enumerate())
                .map(|(k, a)| a * (j - 1.0).powi(2 * (k as i32 + 1)))
                .sum(),
            Self::ArrudaBoyce(a) => a * (0.5 * (j * j - 1.0) - j.ln()),
        }
    }

    /// `dU/dJ`.
    fn derivative(&self, j: f64) -> f64 {
        match self {
            Self::Polynomial(inverse) => (inverse.iter().enumerate())
                .map(|(k, a)| {
                    let power = 2 * (k as i32 + 1);
                    a * f64::from(power) * (j - 1.0).powi(power - 1)
                })
                .sum(),
            Self::ArrudaBoyce(a) => a * (j - 1.0 / j),
        }
    }

    /// The initial bulk modulus `U''(1)`.
    fn bulk_modulus(&self) -> f64 {
        match self {
            Self::Polynomial(inverse) => 2.0 * inverse.first().copied().unwrap_or(0.0),
            Self::ArrudaBoyce(a) => 2.0 * a,
        }
    }
}

/// The isochoric part of the strain energy.
#[derive(Clone, Debug, PartialEq)]
enum Isochoric {
    /// `Σ C_ij (Ī1 - 3)^i (Ī2 - 3)^j` as `(i, j, C_ij)`.
    Polynomial(Vec<(i32, i32, f64)>),
    /// `μ Σ_k c_k / λm^(2k-2) (Ī1^k - 3^k)`.
    ArrudaBoyce { mu: f64, locking: f64 },
    /// `Σ 2 μ_p / α_p² (Σ_a λ̄_a^α_p - 3)` as `(μ_p, α_p)`.
    Ogden(Vec<(f64, f64)>),
}

impl Isochoric {
    /// `W` and its derivatives `W₁ = ∂W/∂Ī1`, `W₂ = ∂W/∂Ī2` of an invariant based model.
    fn invariant_derivatives(&self, i1: f64, i2: f64) -> (f64, f64, f64) {
        match self {
            Self::Polynomial(terms) => {
                let (a, b) = (i1 - 3.0, i2 - 3.0);
                let mut w = 0.0;
                let mut w1 = 0.0;
                let mut w2 = 0.0;
                for &(i, j, c) in terms {
                    w += c * a.powi(i) * b.powi(j);
                    if i > 0 {
                        w1 += c * f64::from(i) * a.powi(i - 1) * b.powi(j);
                    }
                    if j > 0 {
                        w2 += c * f64::from(j) * a.powi(i) * b.powi(j - 1);
                    }
                }
                (w, w1, w2)
            }
            Self::ArrudaBoyce { mu, locking } => {
                let mut w = 0.0;
                let mut w1 = 0.0;
                for (k, c) in ARRUDA_BOYCE.iter().enumerate() {
                    let power = k as i32 + 1;
                    let factor = mu * c / locking.powi(2 * k as i32);
                    w += factor * (i1.powi(power) - 3.0_f64.powi(power));
                    w1 += factor * f64::from(power) * i1.powi(power - 1);
                }
                (w, w1, 0.0)
            }
            Self::Ogden(_) => (0.0, 0.0, 0.0),
        }
    }

    /// The initial shear modulus.
    fn shear_modulus(&self) -> f64 {
        match self {
            Self::Polynomial(terms) => {
                2.0 * (terms.iter())
                    .filter(|(i, j, _)| i + j == 1)
                    .map(|(_, _, c)| c)
                    .sum::<f64>()
            }
            Self::ArrudaBoyce { .. } => 2.0 * self.invariant_derivatives(3.0, 3.0).1,
            Self::Ogden(terms) => terms.iter().map(|(mu, _)| mu).sum(),
        }
    }
}

/// A hyperelastic material law.
#[derive(Clone, Debug, PartialEq)]
pub struct Hyperelastic {
    isochoric: Isochoric,
    volumetric: Volumetric,
    initial: LinearElastic,
}

fn determinant(a: &Mat3) -> f64 {
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
}

fn inverse(a: &Mat3, det: f64) -> Mat3 {
    let mut inv = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            let (i1, i2) = ((j + 1) % 3, (j + 2) % 3);
            let (j1, j2) = ((i + 1) % 3, (i + 2) % 3);
            inv[i][j] = (a[i1][j1] * a[i2][j2] - a[i1][j2] * a[i2][j1]) / det;
        }
    }
    inv
}

/// `C = 1 + 2E` of an engineering Green-Lagrange strain.
fn right_cauchy_green(e: &[f64; 6]) -> Mat3 {
    [
        [1.0 + 2.0 * e[0], e[3], e[5]],
        [e[3], 1.0 + 2.0 * e[1], e[4]],
        [e[5], e[4], 1.0 + 2.0 * e[2]],
    ]
}

fn to_vector(m: &Mat3) -> [f64; 6] {
    [
        m[0][0],
        m[1][1],
        m[2][2],
        0.5 * (m[0][1] + m[1][0]),
        0.5 * (m[1][2] + m[2][1]),
        0.5 * (m[2][0] + m[0][2]),
    ]
}

impl Hyperelastic {
    /// The law of a `*HYPERELASTIC` definition. The constants must give a positive
    /// initial shear and bulk modulus (Nodalis has no hybrid elements, so `D1 = 0`, full
    /// incompressibility, is refused).
    pub fn new(definition: &Definition) -> std::result::Result<Self, String> {
        let model = definition.model;
        let c = &definition.constants;
        if c.len() != model.constant_count() {
            return Err(format!(
                "{} constants expected, {} given",
                model.constant_count(),
                c.len()
            ));
        }
        let d = |from: usize| -> Vec<f64> {
            c[from..]
                .iter()
                .map(|&d| if d == 0.0 { 0.0 } else { 1.0 / d })
                .collect()
        };
        let polynomial_terms = |n: usize, reduced: bool| -> Vec<(i32, i32, f64)> {
            let mut terms = Vec::new();
            let mut k = 0;
            for order in 1..=n as i32 {
                if reduced {
                    terms.push((order, 0, c[k]));
                    k += 1;
                } else {
                    for j in 0..=order {
                        terms.push((order - j, j, c[k]));
                        k += 1;
                    }
                }
            }
            terms
        };
        let (isochoric, volumetric) = match model {
            HyperelasticModel::NeoHooke => (
                Isochoric::Polynomial(vec![(1, 0, c[0])]),
                Volumetric::Polynomial(d(1)),
            ),
            HyperelasticModel::MooneyRivlin => (
                Isochoric::Polynomial(vec![(1, 0, c[0]), (0, 1, c[1])]),
                Volumetric::Polynomial(d(2)),
            ),
            HyperelasticModel::Polynomial(n) => {
                if !(1..=3).contains(&n) {
                    return Err(format!("N={n}: orders 1 to 3 are supported"));
                }
                let terms = polynomial_terms(n as usize, false);
                let count = terms.len();
                (
                    Isochoric::Polynomial(terms),
                    Volumetric::Polynomial(d(count)),
                )
            }
            HyperelasticModel::ReducedPolynomial(n) => {
                if !(1..=3).contains(&n) {
                    return Err(format!("N={n}: orders 1 to 3 are supported"));
                }
                (
                    Isochoric::Polynomial(polynomial_terms(n as usize, true)),
                    Volumetric::Polynomial(d(n as usize)),
                )
            }
            HyperelasticModel::Yeoh => (
                Isochoric::Polynomial(polynomial_terms(3, true)),
                Volumetric::Polynomial(d(3)),
            ),
            HyperelasticModel::Ogden(n) => {
                if !(1..=3).contains(&n) {
                    return Err(format!("N={n}: orders 1 to 3 are supported"));
                }
                let n = n as usize;
                let terms: Vec<(f64, f64)> = (0..n).map(|p| (c[2 * p], c[2 * p + 1])).collect();
                if terms.iter().any(|(_, alpha)| *alpha == 0.0) {
                    return Err("an exponent alpha is zero".into());
                }
                (Isochoric::Ogden(terms), Volumetric::Polynomial(d(2 * n)))
            }
            HyperelasticModel::ArrudaBoyce => {
                if c[1] <= 0.0 {
                    return Err("the locking stretch must be positive".into());
                }
                (
                    Isochoric::ArrudaBoyce {
                        mu: c[0],
                        locking: c[1],
                    },
                    Volumetric::ArrudaBoyce(if c[2] == 0.0 { 0.0 } else { 1.0 / c[2] }),
                )
            }
        };
        let mu = isochoric.shear_modulus();
        let bulk = volumetric.bulk_modulus();
        if mu.is_nan() || mu <= 0.0 {
            return Err(format!(
                "the initial shear modulus {mu} is not positive; check the signs of the constants"
            ));
        }
        if bulk.is_nan() || bulk <= 0.0 {
            return Err(
                "D1 must be positive: fully incompressible materials need hybrid elements, \
                 which Nodalis does not have; give a small D1 instead"
                    .into(),
            );
        }
        let young = 9.0 * bulk * mu / (3.0 * bulk + mu);
        let poisson = (3.0 * bulk - 2.0 * mu) / (2.0 * (3.0 * bulk + mu));
        Ok(Self {
            isochoric,
            volumetric,
            initial: LinearElastic { young, poisson },
        })
    }

    /// The law of a material with a `*HYPERELASTIC` definition.
    pub fn of(material: &Material) -> Result<Option<Self>> {
        let Some(definition) = &material.hyperelastic else {
            return Ok(None);
        };
        Self::new(definition).map(Some).map_err(|message| {
            Error::input(
                "*HYPERELASTIC",
                format!("material {}: {message}", material.name),
            )
        })
    }

    /// The elastic constants of the material for small strains.
    pub fn initial(&self) -> LinearElastic {
        self.initial
    }

    /// The strain energy density per undeformed volume for an engineering Green-Lagrange
    /// strain.
    pub fn energy(&self, strain: &[f64; 6]) -> f64 {
        let c = right_cauchy_green(strain);
        let j = determinant(&c).max(1e-300).sqrt();
        let volumetric = self.volumetric.energy(j);
        match &self.isochoric {
            Isochoric::Ogden(terms) => {
                let (eigenvalues, _) = eigen_symmetric(&c);
                let scale = j.powf(-1.0 / 3.0);
                let stretches = eigenvalues.map(|l| l.max(1e-300).sqrt() * scale);
                let iso: f64 = (terms.iter())
                    .map(|&(mu, alpha)| {
                        2.0 * mu / (alpha * alpha)
                            * (stretches.iter().map(|l| l.powf(alpha)).sum::<f64>() - 3.0)
                    })
                    .sum();
                iso + volumetric
            }
            invariant => {
                let (i1, i2) = Self::isochoric_invariants(&c, j);
                invariant.invariant_derivatives(i1, i2).0 + volumetric
            }
        }
    }

    fn isochoric_invariants(c: &Mat3, j: f64) -> (f64, f64) {
        let i1 = c[0][0] + c[1][1] + c[2][2];
        let trace_c2: f64 = (0..3)
            .flat_map(|a| (0..3).map(move |b| (a, b)))
            .map(|(a, b)| c[a][b] * c[b][a])
            .sum();
        let i2 = 0.5 * (i1 * i1 - trace_c2);
        (i1 * j.powf(-2.0 / 3.0), i2 * j.powf(-4.0 / 3.0))
    }

    /// The second Piola-Kirchhoff stress for an engineering Green-Lagrange strain.
    pub fn second_piola_kirchhoff(&self, strain: &[f64; 6]) -> [f64; 6] {
        let c = right_cauchy_green(strain);
        let det = determinant(&c);
        if det.is_nan() || det <= 0.0 {
            // An inverted point: no energy is defined. A huge volumetric pressure pushes
            // the Newton iteration back (it cuts the increment back if it cannot).
            return [f64::NAN; 6];
        }
        let j = det.sqrt();
        let pressure = j * self.volumetric.derivative(j);
        let c_inv = inverse(&c, det);
        let s = match &self.isochoric {
            Isochoric::Ogden(terms) => {
                let (eigenvalues, vectors) = eigen_symmetric(&c);
                let scale = j.powf(-1.0 / 3.0);
                let stretches = eigenvalues.map(|l| l.max(1e-300).sqrt() * scale);
                let mut tau = [pressure; 3];
                for &(mu, alpha) in terms {
                    let powers = stretches.map(|l| l.powf(alpha));
                    let mean = (powers[0] + powers[1] + powers[2]) / 3.0;
                    for (t, p) in tau.iter_mut().zip(powers) {
                        *t += 2.0 * mu / alpha * (p - mean);
                    }
                }
                let mut s = [[0.0; 3]; 3];
                for a in 0..3 {
                    let principal = tau[a] / eigenvalues[a];
                    for (r, row) in s.iter_mut().enumerate() {
                        for (q, value) in row.iter_mut().enumerate() {
                            *value += principal * vectors[r][a] * vectors[q][a];
                        }
                    }
                }
                s
            }
            invariant => {
                let (i1, i2) = Self::isochoric_invariants(&c, j);
                let (_, w1, w2) = invariant.invariant_derivatives(i1, i2);
                let j23 = j.powf(-2.0 / 3.0);
                // S̄ = 2 [(W₁ + Ī1 W₂) 1 - W₂ C̄] with C̄ = J^(-2/3) C.
                let mut s_bar = [[0.0; 3]; 3];
                for a in 0..3 {
                    for b in 0..3 {
                        let identity = if a == b { 1.0 } else { 0.0 };
                        s_bar[a][b] = 2.0 * ((w1 + i1 * w2) * identity - w2 * j23 * c[a][b]);
                    }
                }
                let contraction: f64 = (0..3)
                    .flat_map(|a| (0..3).map(move |b| (a, b)))
                    .map(|(a, b)| s_bar[a][b] * c[a][b])
                    .sum();
                let mut s = [[0.0; 3]; 3];
                for a in 0..3 {
                    for b in 0..3 {
                        s[a][b] = j23 * (s_bar[a][b] - contraction / 3.0 * c_inv[a][b])
                            + pressure * c_inv[a][b];
                    }
                }
                s
            }
        };
        to_vector(&s)
    }
}

impl MaterialLaw for Hyperelastic {
    fn state_size(&self) -> usize {
        0
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn stress(&self, input: &MaterialInput, _state: &mut [f64]) -> MaterialOutput {
        let stress = self.second_piola_kirchhoff(&input.strain);
        // Central differences on the engineering strain; the tangent of a potential is
        // symmetric, so the two halves are averaged.
        let h = 1e-6;
        let mut tangent = [[0.0; 6]; 6];
        for column in 0..6 {
            let mut plus = input.strain;
            let mut minus = input.strain;
            plus[column] += h;
            minus[column] -= h;
            let (sp, sm) = (
                self.second_piola_kirchhoff(&plus),
                self.second_piola_kirchhoff(&minus),
            );
            for row in 0..6 {
                tangent[row][column] = (sp[row] - sm[row]) / (2.0 * h);
            }
        }
        for row in 0..6 {
            for column in row + 1..6 {
                let mean = 0.5 * (tangent[row][column] + tangent[column][row]);
                tangent[row][column] = mean;
                tangent[column][row] = mean;
            }
        }
        if tangent.iter().flatten().any(|v| !v.is_finite()) {
            // A point within the difference step of turning inside out: treated as
            // inverted, so the iteration cuts the increment back instead of factorizing a
            // tangent without numbers.
            return MaterialOutput {
                stress: [f64::NAN; 6],
                tangent,
            };
        }
        MaterialOutput { stress, tangent }
    }

    fn elastic_matrix(&self) -> [[f64; 6]; 6] {
        self.initial.matrix()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn law(model: HyperelasticModel, constants: &[f64]) -> Hyperelastic {
        Hyperelastic::new(&Definition {
            model,
            constants: constants.to_vec(),
        })
        .unwrap()
    }

    fn all_laws() -> Vec<Hyperelastic> {
        vec![
            law(HyperelasticModel::NeoHooke, &[0.5, 0.02]),
            law(HyperelasticModel::MooneyRivlin, &[0.4, 0.1, 0.02]),
            law(
                HyperelasticModel::Polynomial(2),
                &[0.4, 0.1, 0.01, -0.005, 0.002, 0.02, 0.0],
            ),
            law(
                HyperelasticModel::Yeoh,
                &[0.5, -0.02, 0.003, 0.02, 0.0, 0.0],
            ),
            law(
                HyperelasticModel::Ogden(3),
                &[0.6, 1.3, 0.002, 5.0, -0.01, -2.0, 0.02, 0.0, 0.0],
            ),
            law(HyperelasticModel::ArrudaBoyce, &[0.5, 2.5, 0.02]),
        ]
    }

    /// A general strain of some 20 %.
    const STRAIN: [f64; 6] = [0.21, -0.08, 0.05, 0.12, -0.07, 0.04];

    #[test]
    fn the_stress_is_twice_the_derivative_of_the_energy() {
        for law in all_laws() {
            let s = law.second_piola_kirchhoff(&STRAIN);
            for k in 0..6 {
                let h = 1e-6;
                let mut plus = STRAIN;
                let mut minus = STRAIN;
                plus[k] += h;
                minus[k] -= h;
                // dW/dε_k with engineering strains is S_k for normal and shear components.
                let fd = (law.energy(&plus) - law.energy(&minus)) / (2.0 * h);
                assert!(
                    (fd - s[k]).abs() < 1e-7,
                    "{law:?} component {k}: {fd} vs {}",
                    s[k]
                );
            }
        }
    }

    #[test]
    fn the_undeformed_state_is_stress_free_and_has_the_initial_moduli() {
        for law in all_laws() {
            let out = law.stress(
                &MaterialInput {
                    strain: [0.0; 6],
                    temperature: 0.0,
                    dt: 1.0,
                    time: 1.0,
                    creep_active: false,
                },
                &mut [],
            );
            assert!(out.stress.iter().all(|s| s.abs() < 1e-12), "{law:?}");
            let d = law.initial().matrix();
            for r in 0..6 {
                for c in 0..6 {
                    assert!(
                        (out.tangent[r][c] - d[r][c]).abs() < 1e-6 * d[0][0],
                        "{law:?} [{r}][{c}]: {} vs {}",
                        out.tangent[r][c],
                        d[r][c]
                    );
                }
            }
        }
    }

    #[test]
    fn the_initial_moduli_follow_the_constants() {
        let neo = law(HyperelasticModel::NeoHooke, &[0.5, 0.02]);
        let mu = neo.initial().shear_modulus();
        let bulk = neo.initial().bulk_modulus();
        assert!((mu - 1.0).abs() < 1e-12);
        assert!((bulk - 100.0).abs() < 1e-9);
        let ogden = law(
            HyperelasticModel::Ogden(2),
            &[0.6, 1.3, 0.4, 5.0, 0.02, 0.0],
        );
        assert!((ogden.initial().shear_modulus() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn incompressible_uniaxial_tension_of_neo_hooke_follows_the_textbook() {
        // Nominal stress of an incompressible Neo-Hooke bar: P = 2 C10 (λ - λ⁻²).
        let c10 = 0.5;
        let law = law(HyperelasticModel::NeoHooke, &[c10, 1e-6]);
        let stretch = 2.0_f64;
        let lateral = 1.0 / stretch.sqrt();
        let green = |l: f64| 0.5 * (l * l - 1.0);
        let strain = [
            green(stretch),
            green(lateral),
            green(lateral),
            0.0,
            0.0,
            0.0,
        ];
        let s = law.second_piola_kirchhoff(&strain);
        // The lateral stress vanishes only for the exact incompressible stretch; the
        // pressure from D1 adds the same amount to all Kirchhoff stresses, so subtract it.
        let tau_axial = s[0] * stretch * stretch;
        let tau_lateral = s[1] * lateral * lateral;
        let nominal = (tau_axial - tau_lateral) / stretch;
        let expected = 2.0 * c10 * (stretch - 1.0 / (stretch * stretch));
        assert!((nominal - expected).abs() < 1e-9, "{nominal} vs {expected}");
    }

    #[test]
    fn ogden_with_alpha_two_is_neo_hooke() {
        let neo = law(HyperelasticModel::NeoHooke, &[0.5, 0.02]);
        let ogden = law(HyperelasticModel::Ogden(1), &[1.0, 2.0, 0.02]);
        let mooney = law(HyperelasticModel::MooneyRivlin, &[0.3, 0.2, 0.02]);
        let ogden2 = law(
            HyperelasticModel::Ogden(2),
            &[0.6, 2.0, 0.4, -2.0, 0.02, 0.0],
        );
        let (a, b) = (
            neo.second_piola_kirchhoff(&STRAIN),
            ogden.second_piola_kirchhoff(&STRAIN),
        );
        let (c, d) = (
            mooney.second_piola_kirchhoff(&STRAIN),
            ogden2.second_piola_kirchhoff(&STRAIN),
        );
        for k in 0..6 {
            assert!((a[k] - b[k]).abs() < 1e-10, "{a:?} vs {b:?}");
            assert!((c[k] - d[k]).abs() < 1e-10, "{c:?} vs {d:?}");
        }
    }

    #[test]
    fn the_tangent_is_symmetric_and_matches_the_stress() {
        for law in all_laws() {
            let input = MaterialInput {
                strain: STRAIN,
                temperature: 0.0,
                dt: 1.0,
                time: 1.0,
                creep_active: false,
            };
            let out = law.stress(&input, &mut []);
            for c in 0..6 {
                let h = 1e-5;
                let mut plus = STRAIN;
                plus[c] += h;
                let mut minus = STRAIN;
                minus[c] -= h;
                let sp = law.second_piola_kirchhoff(&plus);
                let sm = law.second_piola_kirchhoff(&minus);
                for r in 0..6 {
                    let fd = (sp[r] - sm[r]) / (2.0 * h);
                    assert!((fd - out.tangent[r][c]).abs() < 1e-5, "{law:?} [{r}][{c}]");
                    assert_eq!(out.tangent[r][c], out.tangent[c][r]);
                }
            }
        }
    }

    #[test]
    fn invalid_constants_are_refused() {
        let new = |model, constants: &[f64]| {
            Hyperelastic::new(&Definition {
                model,
                constants: constants.to_vec(),
            })
        };
        assert!(new(HyperelasticModel::NeoHooke, &[0.5, 0.0]).is_err());
        assert!(new(HyperelasticModel::NeoHooke, &[-0.5, 0.02]).is_err());
        assert!(new(HyperelasticModel::NeoHooke, &[0.5]).is_err());
        assert!(new(HyperelasticModel::Ogden(1), &[1.0, 0.0, 0.02]).is_err());
        assert!(new(HyperelasticModel::Polynomial(4), &[0.0; 18]).is_err());
    }

    #[test]
    fn a_point_on_the_verge_of_inverting_has_no_stress() {
        // C33 = 1 + 2 E33 = 1e-7: positive, but the difference step of the tangent crosses
        // zero. Its tangent would have no numbers, so the point counts as inverted and the
        // iteration cuts the increment back.
        let law = law(HyperelasticModel::NeoHooke, &[0.5, 0.02]);
        let mut strain = [0.0; 6];
        strain[2] = 0.5 * (1e-7 - 1.0);
        assert!(
            law.second_piola_kirchhoff(&strain)
                .iter()
                .all(|v| v.is_finite())
        );
        let input = MaterialInput {
            strain,
            temperature: 0.0,
            dt: 1.0,
            time: 1.0,
            creep_active: false,
        };
        let out = law.stress(&input, &mut []);
        assert!(out.stress.iter().all(|v| v.is_nan()));
    }
}
