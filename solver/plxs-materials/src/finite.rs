//! J2 plasticity and creep at finite strains for NLGEOM steps: the multiplicative split
//! `F = F_e F_p` with a hyperelastic (Hencky) response in the logarithmic elastic strain and
//! the exponential-map return mapping (Simo & Hughes, *Computational Inelasticity*, chapter
//! 9; Simo 1992). The hardening curves then relate the von Mises Kirchhoff stress to the
//! equivalent logarithmic plastic strain, which is what the CalculiX manual says its
//! `*PLASTIC` tables mean under large strains.
//!
//! The elements work in the total Lagrangian frame and hand in the Green-Lagrange strain
//! `E`; the law forms `C = 1 + 2E`, the stretch `U = √C` (any `F` with `FᵀF = C` gives the
//! same second Piola-Kirchhoff stress for an isotropic material), the trial elastic left
//! Cauchy-Green tensor `b_e = U C_p⁻¹ U` from the plastic right Cauchy-Green tensor stored
//! in the state, its logarithm, the small-strain return mapping on the Kirchhoff stress, the
//! update `b_e = exp(2 ε_e)` and `C_p⁻¹ = U⁻¹ b_e U⁻¹`, and finally `S = U⁻¹ τ U⁻¹`. The
//! tangent `dS/dE` is taken by central differences of this map (twelve extra evaluations
//! per integration point; an analytical tangent is a later refinement).
//!
//! State variables as in [`crate::plastic`]: equivalent plastic strain, `C_p⁻¹ - 1` (six
//! tensor components, so that zeros are the virgin state), back stress (Lagrangian frame),
//! creep rate.

use crate::plastic::{BACK, J2Plastic, PLASTIC, STATE_SIZE, deviator};
use crate::{MaterialInput, MaterialLaw, MaterialOutput};

type Mat3 = [[f64; 3]; 3];

/// Eigenvalues and eigenvectors (columns) of a symmetric 3 × 3 matrix by cyclic Jacobi
/// rotations.
pub fn eigen_symmetric(a: &Mat3) -> ([f64; 3], Mat3) {
    let mut a = *a;
    let mut v: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        let scale = a[0][0].abs() + a[1][1].abs() + a[2][2].abs();
        if off <= 1e-15 * scale.max(1e-300) {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() <= 1e-300 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for row in a.iter_mut() {
                let (akp, akq) = (row[p], row[q]);
                row[p] = c * akp - s * akq;
                row[q] = s * akp + c * akq;
            }
            let (row_p, row_q) = (a[p], a[q]);
            for ((apk, aqk), k) in row_p.iter().zip(&row_q).zip(0..3) {
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

fn to_matrix(t: &[f64; 6]) -> Mat3 {
    [[t[0], t[3], t[5]], [t[3], t[1], t[4]], [t[5], t[4], t[2]]]
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

fn multiply(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut c = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            c[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    c
}

/// `f` applied to the eigenvalues of a symmetric matrix.
fn spectral(m: &Mat3, f: impl Fn(f64) -> f64) -> Mat3 {
    let (values, v) = eigen_symmetric(m);
    let mut out = [[0.0; 3]; 3];
    for (a, &lambda) in values.iter().enumerate() {
        let fa = f(lambda);
        for i in 0..3 {
            for j in 0..3 {
                out[i][j] += fa * v[i][a] * v[j][a];
            }
        }
    }
    out
}

/// The finite-strain version of a [`J2Plastic`] law.
#[derive(Clone, Debug, PartialEq)]
pub struct FiniteStrainJ2 {
    inner: J2Plastic,
}

impl FiniteStrainJ2 {
    pub fn new(inner: J2Plastic) -> Self {
        Self { inner }
    }

    /// The second Piola-Kirchhoff stress for the Green-Lagrange strain `e` (engineering
    /// components) from the state at the start of the increment; `state` is updated. Also
    /// returns whether the point flowed; without `allow_flow` it responds elastically.
    fn evaluate(
        &self,
        e: &[f64; 6],
        input: &MaterialInput,
        state: &mut [f64],
        allow_flow: bool,
    ) -> ([f64; 6], bool) {
        let c = to_matrix(&[
            1.0 + 2.0 * e[0],
            1.0 + 2.0 * e[1],
            1.0 + 2.0 * e[2],
            e[3],
            e[4],
            e[5],
        ]);
        let u = spectral(&c, |l| l.max(1e-300).sqrt());
        let u_inv = spectral(&c, |l| 1.0 / l.max(1e-300).sqrt());
        let mut cp_inv = to_matrix(&[
            state[PLASTIC],
            state[PLASTIC + 1],
            state[PLASTIC + 2],
            state[PLASTIC + 3],
            state[PLASTIC + 4],
            state[PLASTIC + 5],
        ]);
        for (i, row) in cp_inv.iter_mut().enumerate() {
            row[i] += 1.0;
        }
        let b_trial = multiply(&multiply(&u, &cp_inv), &u);
        // Logarithmic elastic trial strain.
        let eps_trial = spectral(&b_trial, |l| 0.5 * l.max(1e-300).ln());
        let eps_trial_v = to_vector(&eps_trial);
        let (tau, plastic_increment, _) = self.inner.kernel(&eps_trial_v, state, input, allow_flow);
        let flowed = plastic_increment.iter().any(|v| *v != 0.0);
        let eps_e: [f64; 6] = std::array::from_fn(|k| eps_trial_v[k] - plastic_increment[k]);
        let b_e = spectral(&to_matrix(&eps_e), |x| (2.0 * x).exp());
        let cp_inv_new = multiply(&multiply(&u_inv, &b_e), &u_inv);
        let cp_v = to_vector(&cp_inv_new);
        for k in 0..6 {
            state[PLASTIC + k] = cp_v[k] - if k < 3 { 1.0 } else { 0.0 };
        }
        (
            to_vector(&multiply(&multiply(&u_inv, &to_matrix(&tau)), &u_inv)),
            flowed,
        )
    }
}

impl MaterialLaw for FiniteStrainJ2 {
    fn state_size(&self) -> usize {
        STATE_SIZE
    }

    fn is_nonlinear(&self) -> bool {
        true
    }

    fn elastic_matrix(&self) -> [[f64; 6]; 6] {
        self.inner.elastic_matrix()
    }

    /// The stress alone: one evaluation instead of the thirteen of the tangent's central
    /// differences.
    fn stress_without_tangent(
        &self,
        input: &MaterialInput,
        state: &mut [f64],
    ) -> ([f64; 6], [f64; 6]) {
        (
            self.evaluate(&input.strain, input, state, true).0,
            input.strain,
        )
    }

    // Symmetrising a 6×6 array by index pairs reads better than iterator chains here.
    #[allow(clippy::needless_range_loop)]
    fn stress(&self, input: &MaterialInput, state: &mut [f64]) -> MaterialOutput {
        let start: Vec<f64> = state.to_vec();
        let (stress, flowed) = self.evaluate(&input.strain, input, state, true);
        // A point that does not flow (inside the yield surface, or on it without a strain
        // increment) gets the tangent of the elastic branch: central differences across the
        // surface would mix in the plastic branch and soften an unloading increment.
        let mut tangent = [[0.0; 6]; 6];
        let scale = input.strain.iter().fold(1.0_f64, |m, v| m.max(v.abs()));
        let h = 1e-7 * scale;
        for j in 0..6 {
            let mut plus = input.strain;
            plus[j] += h;
            let mut minus = input.strain;
            minus[j] -= h;
            let mut sp = start.clone();
            let mut sm = start.clone();
            let (s_plus, _) = self.evaluate(&plus, input, &mut sp, flowed);
            let (s_minus, _) = self.evaluate(&minus, input, &mut sm, flowed);
            for i in 0..6 {
                tangent[i][j] = (s_plus[i] - s_minus[i]) / (2.0 * h);
            }
        }
        // Symmetrise what round-off left unsymmetric.
        for i in 0..6 {
            for j in 0..i {
                let v = 0.5 * (tangent[i][j] + tangent[j][i]);
                tangent[i][j] = v;
                tangent[j][i] = v;
            }
        }
        let _ = (deviator, BACK);
        MaterialOutput { stress, tangent }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LinearElastic;
    use plxs_core::material::{Elastic, ElasticRow, Hardening, Material, Plastic, PlasticRow};

    fn steel() -> Material {
        Material {
            name: "STEEL".into(),
            elastic: Some(Elastic {
                rows: vec![ElasticRow {
                    young: 210_000.0,
                    poisson: 0.3,
                    temperature: 0.0,
                }],
            }),
            plastic: Some(Plastic {
                hardening: Hardening::Isotropic,
                rows: vec![
                    PlasticRow {
                        stress: 235.0,
                        plastic_strain: 0.0,
                        temperature: 0.0,
                    },
                    PlasticRow {
                        stress: 400.0,
                        plastic_strain: 0.2,
                        temperature: 0.0,
                    },
                ],
            }),
            ..Material::default()
        }
    }

    fn law() -> FiniteStrainJ2 {
        let material = steel();
        let elastic = LinearElastic::of(&material, 0.0).unwrap();
        FiniteStrainJ2::new(J2Plastic::of(&material, elastic, 0.0).unwrap())
    }

    fn input(strain: [f64; 6]) -> MaterialInput {
        MaterialInput {
            strain,
            temperature: 0.0,
            dt: 1.0,
            time: 1.0,
            creep_active: false,
        }
    }

    fn green_lagrange(f: &Mat3) -> [f64; 6] {
        let mut c = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                c[i][j] = (0..3).map(|k| f[k][i] * f[k][j]).sum();
            }
        }
        [
            0.5 * (c[0][0] - 1.0),
            0.5 * (c[1][1] - 1.0),
            0.5 * (c[2][2] - 1.0),
            c[0][1],
            c[1][2],
            c[2][0],
        ]
    }

    #[test]
    fn jacobi_eigen_decomposition_reconstructs_the_matrix() {
        let a: Mat3 = [[4.0, 1.0, -2.0], [1.0, 3.0, 0.5], [-2.0, 0.5, 6.0]];
        let (values, v) = eigen_symmetric(&a);
        for i in 0..3 {
            for j in 0..3 {
                let r: f64 = (0..3).map(|k| values[k] * v[i][k] * v[j][k]).sum();
                assert!((r - a[i][j]).abs() < 1e-12, "{r} vs {}", a[i][j]);
            }
        }
    }

    /// Uniaxial tension with free lateral contraction: the Kirchhoff stress follows the
    /// hardening curve in the logarithmic plastic strain, and plastic flow conserves the
    /// volume (J = exp(3 ε_e,vol)).
    #[test]
    fn uniaxial_stretch_follows_the_curve_in_logarithmic_strain() {
        let law = law();
        let stretch = 1.1_f64; // 10 % logarithmic strain, far into the plastic range
        let mut lateral = 1.0 / stretch.sqrt();
        let mut state = vec![0.0; STATE_SIZE];
        let mut s = [0.0; 6];
        for _ in 0..60 {
            let f: Mat3 = [
                [stretch, 0.0, 0.0],
                [0.0, lateral, 0.0],
                [0.0, 0.0, lateral],
            ];
            let mut trial = vec![0.0; STATE_SIZE];
            let out = law.stress(&input(green_lagrange(&f)), &mut trial);
            s = out.stress;
            // Newton on the lateral stretch for zero lateral stress.
            let de = lateral; // dE_22/dλ_2 = λ_2
            let d = (out.tangent[1][1] + out.tangent[1][2]) * de;
            let correction = -s[1] / d;
            lateral += correction;
            if correction.abs() < 1e-13 {
                state = trial;
                break;
            }
        }
        assert!(s[1].abs() < 1e-8 && s[2].abs() < 1e-8, "{s:?}");
        // Cauchy stress σ = λ_1 S_11 / (λ_2 λ_3) ... with J = λ_1 λ_2²: σ_11 = λ_1² S_11 / J.
        let j = stretch * lateral * lateral;
        let sigma = stretch * stretch * s[0] / j;
        let tau = j * sigma;
        let peeq = state[0];
        let on_curve = 235.0 + (400.0 - 235.0) / 0.2 * peeq;
        assert!(
            (tau - on_curve).abs() < 1e-6 * on_curve,
            "{tau} vs {on_curve}"
        );
        // Logarithmic strain = elastic + plastic.
        let log_strain = stretch.ln();
        let elastic = tau / 210_000.0;
        assert!(
            (log_strain - elastic - peeq).abs() < 1e-8,
            "{log_strain} vs {} + {peeq}",
            elastic
        );
        // Plastic isochoric: J = exp(elastic volumetric strain).
        let volumetric = (1.0 - 2.0 * 0.3) * tau / 210_000.0;
        assert!(
            (j - volumetric.exp()).abs() < 1e-8,
            "{j} vs {}",
            volumetric.exp()
        );
    }

    #[test]
    fn the_tangent_is_the_derivative_of_the_stress_in_the_plastic_range() {
        let law = law();
        // A plastic state from a first increment, then a further plastic increment.
        let mut state = vec![0.0; STATE_SIZE];
        let f0: Mat3 = [[1.01, 0.0, 0.0], [0.0, 0.995, 0.0], [0.0, 0.0, 0.995]];
        law.stress(&input(green_lagrange(&f0)), &mut state);
        let f: Mat3 = [[1.02, 0.01, 0.0], [0.0, 0.99, 0.004], [0.0, 0.0, 0.99]];
        let e = green_lagrange(&f);
        let out = law.stress(&input(e), &mut state.clone());
        let h = 1e-6;
        for j in 0..6 {
            let (mut plus, mut minus) = (e, e);
            plus[j] += h;
            minus[j] -= h;
            let sp = law.stress(&input(plus), &mut state.clone()).stress;
            let sm = law.stress(&input(minus), &mut state.clone()).stress;
            for i in 0..6 {
                let fd = (sp[i] - sm[i]) / (2.0 * h);
                assert!(
                    (fd - out.tangent[i][j]).abs() <= 1e-4 * 210_000.0,
                    "d S{i}/d E{j}: tangent {} vs finite difference {fd}",
                    out.tangent[i][j]
                );
            }
        }
    }

    /// The converged state at the start of an increment sits on the yield surface: its
    /// tangent is the elastic branch's (the one of a slightly unloaded strain), not a
    /// central difference across the surface (regression: unloading a yielded body under
    /// NLGEOM diverged in its first increment).
    #[test]
    fn a_converged_plastic_point_starts_the_next_increment_elastically() {
        use crate::plastic::PEEQ;
        let law = law();
        let f: Mat3 = [[1.02, 0.0, 0.0], [0.0, 0.994, 0.0], [0.0, 0.0, 0.994]];
        let e = green_lagrange(&f);
        let mut state = vec![0.0; STATE_SIZE];
        law.stress(&input(e), &mut state);
        assert!(state[PEEQ] > 0.0, "the test strain must yield");
        let mut next = state.clone();
        let at_start = law.stress(&input(e), &mut next);
        assert!((next[PEEQ] - state[PEEQ]).abs() < 1e-15, "no further flow");
        let mut unloaded = e;
        unloaded[0] -= 1e-4;
        let mut inside = state.clone();
        let elastic = law.stress(&input(unloaded), &mut inside);
        assert_eq!(
            inside[PEEQ], state[PEEQ],
            "the unloaded strain must be elastic"
        );
        for i in 0..6 {
            for j in 0..6 {
                assert!(
                    (at_start.tangent[i][j] - elastic.tangent[i][j]).abs() < 1e-3 * 210_000.0,
                    "D[{i}][{j}]: {} at the start vs {} elastic",
                    at_start.tangent[i][j],
                    elastic.tangent[i][j]
                );
            }
        }
    }

    #[test]
    fn a_rigid_rotation_of_a_plastic_state_keeps_the_stress_objective() {
        let law = law();
        let f: Mat3 = [[1.05, 0.02, 0.0], [0.0, 0.98, 0.0], [0.0, 0.0, 0.99]];
        let mut state_a = vec![0.0; STATE_SIZE];
        let s_a = law.stress(&input(green_lagrange(&f)), &mut state_a).stress;
        assert!(state_a[0] > 0.0);
        let (sn, cs) = 0.4_f64.sin_cos();
        let r: Mat3 = [[cs, -sn, 0.0], [sn, cs, 0.0], [0.0, 0.0, 1.0]];
        let rf = multiply(&r, &f);
        let mut state_b = vec![0.0; STATE_SIZE];
        let s_b = law.stress(&input(green_lagrange(&rf)), &mut state_b).stress;
        for k in 0..6 {
            assert!((s_a[k] - s_b[k]).abs() < 1e-9 * 300.0, "{s_a:?} vs {s_b:?}");
        }
        assert!((state_a[0] - state_b[0]).abs() < 1e-12);
    }

    #[test]
    fn small_strains_agree_with_the_small_strain_law() {
        let material = steel();
        let elastic = LinearElastic::of(&material, 0.0).unwrap();
        let small = J2Plastic::of(&material, elastic, 0.0).unwrap();
        let finite = FiniteStrainJ2::new(small.clone());
        let strain = [0.003, -0.001, 0.0002, 0.002, -0.0015, 0.0007];
        let mut sa = vec![0.0; STATE_SIZE];
        let mut sb = vec![0.0; STATE_SIZE];
        let a = small.stress(&input(strain), &mut sa);
        let b = finite.stress(&input(strain), &mut sb);
        assert!(sa[0] > 0.0);
        // The finite-strain law returns the second Piola-Kirchhoff stress; compare the
        // Cauchy stress σ = U S U / J, which differs from the small-strain stress by the
        // order of strain² times the elastic moduli (a few MPa here).
        let c = to_matrix(&[
            1.0 + 2.0 * strain[0],
            1.0 + 2.0 * strain[1],
            1.0 + 2.0 * strain[2],
            strain[3],
            strain[4],
            strain[5],
        ]);
        let u = spectral(&c, |l| l.sqrt());
        let (lc, _) = eigen_symmetric(&c);
        let j = (lc[0] * lc[1] * lc[2]).sqrt();
        let sigma = to_vector(&multiply(&multiply(&u, &to_matrix(&b.stress)), &u));
        for k in 0..6 {
            assert!(
                (a.stress[k] - sigma[k] / j).abs() < 2.0 * 160_000.0 * 0.003 * 0.003 + 1.0,
                "{:?} vs {:?}",
                a.stress,
                sigma
            );
        }
        assert!((sa[0] - sb[0]).abs() < 1e-5, "{} vs {}", sa[0], sb[0]);
        for i in 0..6 {
            for j in 0..6 {
                assert!(
                    (a.tangent[i][j] - b.tangent[i][j]).abs() < 2e-2 * 210_000.0,
                    "D[{i}][{j}]: {} vs {}",
                    a.tangent[i][j],
                    b.tangent[i][j]
                );
            }
        }
    }
}
