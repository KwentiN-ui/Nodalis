//! Plane stress: the thickness stress `σzz` is zero at every integration point.
//!
//! Plane stress elements move in their plane only (their strains have no transverse shear
//! and no thickness strain from the displacements); the thickness strain `εzz` is a local
//! unknown of each integration point that makes `σzz` vanish. For Hooke's law that is the
//! static condensation of the elasticity matrix ([`condense`]); for any other law
//! ([`PlaneStress`]) a Newton iteration on `εzz` with the law's own tangent, whose
//! condensed form is the tangent the element sees (Simo & Hughes, *Computational
//! Inelasticity*, 1998, section 3.4.2 on plane stress return mapping by condensation of the
//! three-dimensional algorithm; de Borst, "The zero-normal-stress condition in plasticity",
//! Computers & Structures 40, 1991). Under NLGEOM the strain is the Green-Lagrange strain
//! and the stress the second Piola-Kirchhoff stress, whose `Szz` vanishes with the Cauchy
//! `σzz` because the deformation gradient of an in-plane motion has no coupling between
//! the plane and its normal.
//!
//! Index 2 of the engineering vectors `[xx, yy, zz, xy, yz, zx]` is the thickness
//! direction.

use crate::{MaterialInput, MaterialLaw, MaterialOutput};

/// The thickness component of the strain and stress vectors.
const ZZ: usize = 2;

/// The matrix `D` condensed to plane stress: `D_ij - D_i3 D_3j / D_33` with the thickness
/// row and column zero, so that `D ε` has no thickness stress whatever `εzz` is.
pub fn condense(d: &[[f64; 6]; 6]) -> [[f64; 6]; 6] {
    let mut out = *d;
    let pivot = d[ZZ][ZZ];
    if pivot == 0.0 || !pivot.is_finite() {
        return out;
    }
    for i in 0..6 {
        for j in 0..6 {
            out[i][j] = d[i][j] - d[i][ZZ] * d[ZZ][j] / pivot;
        }
    }
    out[ZZ] = [0.0; 6];
    for row in &mut out {
        row[ZZ] = 0.0;
    }
    out
}

/// The thickness strain of an elastic plane stress point: the `εzz` that makes the thickness
/// stress of the full elasticity matrix `d` vanish for the in-plane strains of `strain`
/// (its `εzz` is ignored) and the isotropic thermal strain `thermal`.
pub fn thickness_strain(d: &[[f64; 6]; 6], strain: &[f64; 6], thermal: f64) -> f64 {
    let pivot = d[ZZ][ZZ];
    if pivot == 0.0 || !pivot.is_finite() {
        return strain[ZZ];
    }
    let mut stress = 0.0;
    for (k, e) in strain.iter().enumerate() {
        if k == ZZ {
            continue;
        }
        let mechanical = if k < 3 { e - thermal } else { *e };
        stress += d[ZZ][k] * mechanical;
    }
    thermal - stress / pivot
}

/// A material law under plane stress: the thickness strain found per point so that the
/// thickness stress is zero, the tangent condensed.
pub struct PlaneStress {
    inner: Box<dyn MaterialLaw>,
}

/// Iterations of the thickness strain before the point gives up (keeps the last value).
const MAX_ITERATIONS: usize = 50;

impl PlaneStress {
    pub fn new(inner: Box<dyn MaterialLaw>) -> Self {
        Self { inner }
    }

    /// Stress, condensed tangent and the strain with the thickness strain that makes the
    /// thickness stress vanish.
    fn solve(&self, input: &MaterialInput, state: &mut [f64]) -> (MaterialOutput, [f64; 6]) {
        let elastic = self.inner.elastic_matrix();
        let mut strain = input.strain;
        // No transverse shear in the plane (round-off of the element's kinematics).
        strain[4] = 0.0;
        strain[5] = 0.0;
        strain[ZZ] = thickness_strain(&elastic, &strain, 0.0);
        let start = state.to_vec();
        // The scale of the stresses this law gives: what the thickness stress is measured
        // against (the strains are dimensionless, the moduli carry the unit).
        let modulus = elastic[ZZ][ZZ].abs().max(f64::MIN_POSITIVE);
        // Newton's method on the thickness strain, safeguarded by bisection: the thickness
        // stress grows with the thickness strain, so every evaluation narrows the bracket
        // `(low, high)` around the root. A Green-Lagrange thickness strain of -1/2 is a
        // thickness of zero; where the law gives no numbers (a point squeezed flat) the
        // root lies above.
        let (mut low, mut high) = (-0.5, f64::INFINITY);
        let inside = |low: f64, high: f64| {
            if high.is_finite() {
                0.5 * (low + high)
            } else if low < 0.0 {
                0.5 * low
            } else {
                low + low.max(1.0)
            }
        };
        if strain[ZZ].is_nan() || strain[ZZ] <= low + 0.05 {
            strain[ZZ] = inside(low, high);
        }
        let mut output = self.evaluate(input, &strain, state);
        for _ in 0..MAX_ITERATIONS {
            let residual = output.stress[ZZ];
            if !residual.is_finite() {
                low = strain[ZZ];
                strain[ZZ] = inside(low, high);
            } else {
                let scale = output.stress.iter().fold(0.0_f64, |m, s| m.max(s.abs()));
                if residual.abs() <= 1e-12 * scale.max(1e-12 * modulus) {
                    break;
                }
                if residual > 0.0 {
                    high = strain[ZZ];
                } else {
                    low = strain[ZZ];
                }
                let slope = output.tangent[ZZ][ZZ];
                let slope = if slope.is_finite() && slope > 0.0 {
                    slope
                } else {
                    modulus
                };
                let next = strain[ZZ] - residual / slope;
                strain[ZZ] = if next > low && next < high {
                    next
                } else {
                    inside(low, high)
                };
            }
            state.copy_from_slice(&start);
            output = self.evaluate(input, &strain, state);
        }
        if output.stress[ZZ].is_finite() {
            output.stress[ZZ] = 0.0;
            output.stress[4] = 0.0;
            output.stress[5] = 0.0;
        }
        output.tangent = condense(&output.tangent);
        (output, strain)
    }

    fn evaluate(
        &self,
        input: &MaterialInput,
        strain: &[f64; 6],
        state: &mut [f64],
    ) -> MaterialOutput {
        self.inner.stress(
            &MaterialInput {
                strain: *strain,
                ..*input
            },
            state,
        )
    }
}

impl MaterialLaw for PlaneStress {
    fn state_size(&self) -> usize {
        self.inner.state_size()
    }

    fn is_nonlinear(&self) -> bool {
        self.inner.is_nonlinear()
    }

    fn stress(&self, input: &MaterialInput, state: &mut [f64]) -> MaterialOutput {
        self.solve(input, state).0
    }

    fn stress_and_strain(
        &self,
        input: &MaterialInput,
        state: &mut [f64],
    ) -> (MaterialOutput, [f64; 6]) {
        self.solve(input, state)
    }

    fn elastic_matrix(&self) -> [[f64; 6]; 6] {
        condense(&self.inner.elastic_matrix())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Hyperelastic, J2Plastic, LinearElastic, isotropic_matrix};
    use plxs_core::material::{
        self, Elastic, ElasticRow, Hardening, HyperelasticModel, Material, Plastic, PlasticRow,
    };

    fn input(strain: [f64; 6]) -> MaterialInput {
        MaterialInput {
            strain,
            temperature: 0.0,
            dt: 1.0,
            time: 1.0,
            creep_active: false,
        }
    }

    #[test]
    fn the_condensed_matrix_is_the_textbook_plane_stress_matrix() {
        let (e, nu) = (210_000.0, 0.3);
        let d = condense(&isotropic_matrix(e, nu));
        let f = e / (1.0 - nu * nu);
        assert!((d[0][0] - f).abs() < 1e-6 * f);
        assert!((d[0][1] - f * nu).abs() < 1e-6 * f);
        assert!((d[3][3] - e / (2.0 * (1.0 + nu))).abs() < 1e-6 * f);
        assert!(d[2].iter().all(|v| *v == 0.0));
        assert!(d.iter().all(|row| row[2] == 0.0));
        // Uniaxial stress: εyy = εzz = -ν εxx.
        let ezz = thickness_strain(
            &isotropic_matrix(e, nu),
            &[1e-3, -nu * 1e-3, 0.0, 0.0, 0.0, 0.0],
            0.0,
        );
        assert!((ezz + nu * 1e-3).abs() < 1e-15);
        // Free thermal expansion: no stress, εzz = α ΔT.
        let ezz = thickness_strain(
            &isotropic_matrix(e, nu),
            &[2e-4, 2e-4, 0.0, 0.0, 0.0, 0.0],
            2e-4,
        );
        assert!((ezz - 2e-4).abs() < 1e-15);
    }

    #[test]
    fn an_elastic_law_gives_the_condensed_stress() {
        let law = PlaneStress::new(Box::new(LinearElastic {
            young: 1000.0,
            poisson: 0.25,
        }));
        let eps = [1e-3, 4e-4, 0.0, 3e-4, 0.0, 0.0];
        let (out, strain) = law.stress_and_strain(&input(eps), &mut []);
        let d = condense(&isotropic_matrix(1000.0, 0.25));
        for (r, row) in d.iter().enumerate() {
            let expected: f64 = row.iter().zip(&eps).map(|(a, b)| a * b).sum();
            assert!((out.stress[r] - expected).abs() < 1e-12, "{r}");
        }
        assert_eq!(out.stress[2], 0.0);
        assert!((strain[2] + 0.25 / 0.75 * 1.4e-3).abs() < 1e-15);
    }

    #[test]
    fn a_plastic_law_yields_in_uniaxial_plane_stress_at_its_yield_stress() {
        let material = Material {
            name: "STEEL".into(),
            elastic: Some(Elastic {
                rows: vec![ElasticRow {
                    young: 200_000.0,
                    poisson: 0.3,
                    temperature: 0.0,
                }],
            }),
            plastic: Some(Plastic {
                hardening: Hardening::Isotropic,
                rows: vec![PlasticRow {
                    stress: 200.0,
                    plastic_strain: 0.0,
                    temperature: 0.0,
                }],
            }),
            ..Material::default()
        };
        let elastic = LinearElastic {
            young: 200_000.0,
            poisson: 0.3,
        };
        let law = PlaneStress::new(Box::new(J2Plastic::of(&material, elastic, 0.0).unwrap()));
        let mut state = vec![0.0; law.state_size()];
        // Strain εxx = 5 times the yield strain; εyy free (found by bisection on σyy = 0).
        let exx = 5.0 * 200.0 / 200_000.0;
        let syy = |eyy: f64| {
            let mut s = vec![0.0; law.state_size()];
            law.stress(&input([exx, eyy, 0.0, 0.0, 0.0, 0.0]), &mut s)
                .stress[1]
        };
        let (mut lo, mut hi) = (-exx, 0.0);
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if syy(mid) > 0.0 {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        let eyy = 0.5 * (lo + hi);
        let (out, strain) =
            law.stress_and_strain(&input([exx, eyy, 0.0, 0.0, 0.0, 0.0]), &mut state);
        assert_eq!(out.stress[2], 0.0);
        assert!((out.stress[0] - 200.0).abs() < 1e-6, "{:?}", out.stress);
        // Plastic flow is volume preserving: the thickness strain tends to -εxx/2.
        assert!(
            strain[2] < -0.4 * exx && strain[2] > -0.5 * exx,
            "{strain:?}"
        );
    }

    #[test]
    fn the_condensed_tangent_is_the_derivative_of_the_plane_stress() {
        let material = Material {
            name: "RUBBER".into(),
            hyperelastic: Some(material::Hyperelastic {
                model: HyperelasticModel::NeoHooke,
                constants: vec![10.0, 0.02],
            }),
            ..Material::default()
        };
        let hyper = Hyperelastic::of(&material).unwrap().unwrap();
        let law = PlaneStress::new(Box::new(hyper));
        let eps = [0.2, -0.05, 0.0, 0.1, 0.0, 0.0];
        let out = law.stress(&input(eps), &mut []);
        let h = 1e-6;
        for c in [0, 1, 3] {
            let mut plus = eps;
            let mut minus = eps;
            plus[c] += h;
            minus[c] -= h;
            let sp = law.stress(&input(plus), &mut []).stress;
            let sm = law.stress(&input(minus), &mut []).stress;
            for r in [0, 1, 3] {
                let fd = (sp[r] - sm[r]) / (2.0 * h);
                assert!(
                    (out.tangent[r][c] - fd).abs() < 1e-4 * out.tangent[0][0].abs(),
                    "{r} {c}: {} vs {fd}",
                    out.tangent[r][c]
                );
            }
        }
        assert_eq!(out.stress[2], 0.0);
    }
}
