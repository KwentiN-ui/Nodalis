//! J2 (von Mises) plasticity with isotropic, kinematic and combined hardening from the
//! `*PLASTIC` tables, and Norton creep (`*CREEP, LAW=NORTON`) as a viscoplastic overstress
//! law on the same yield surface (a zero yield surface without `*PLASTIC`, as the CalculiX
//! manual states). Small-strain additive split; under NLGEOM the Green-Lagrange strain takes
//! the place of the small strain (procedures hand it in), which is exact for the elastic
//! part and an approximation for plastic strains of a few percent.
//!
//! Algorithm: elastic predictor, radial return on the deviator (Simo & Hughes, Box 3.2,
//! extended to piecewise-linear hardening curves by a safeguarded Newton iteration on the
//! consistency condition) and Prager's linear kinematic hardening rule with the modulus
//! taken from the kinematic curve. With creep active the plastic multiplier satisfies the
//! Perzyna-type overstress relation `Δγ = Δt A (q - σ_y)^n t^m` (Simo & Hughes 2.7), whose
//! rate-independent limit is the return mapping. The consistent tangent follows from
//! differentiating the return map (Simo & Hughes 3.3.3), with `dΔγ/dq_trial` from the
//! scalar equation actually solved.
//!
//! State variables per integration point (`STATE_SIZE`), in CalculiX's order so that a
//! future `SDV` output can hand them out: equivalent plastic strain (1), plastic strain
//! tensor (6, tensor shear components), back stress (6); plus the equivalent creep strain
//! rate of the last increment (1) for the `CETOL` time step control.

use plxs_core::material::{Hardening, Material};
use plxs_core::{Error, Result};

use crate::{LinearElastic, MaterialInput, MaterialLaw, MaterialOutput};

/// State variables per integration point.
pub const STATE_SIZE: usize = 14;
pub(crate) const PEEQ: usize = 0;
pub(crate) const PLASTIC: usize = 1;
pub(crate) const BACK: usize = 7;
pub(crate) const CREEP_RATE: usize = 13;

/// Relative distance to the yield surface below which a trial state counts as on the
/// surface and elastic (far above the round-off of the return mapping, far below any real
/// plastic strain increment).
const YIELD_TOLERANCE: f64 = 1e-9;

/// A hardening curve σ(ε̄ᵖ) at one temperature: piecewise linear, constant beyond its ends.
#[derive(Clone, Debug, PartialEq)]
struct Curve {
    /// (equivalent plastic strain, stress), ascending in strain.
    points: Vec<(f64, f64)>,
}

impl Curve {
    fn value(&self, strain: f64) -> f64 {
        let p = &self.points;
        if strain <= p[0].0 {
            return p[0].1;
        }
        for w in p.windows(2) {
            if strain <= w[1].0 {
                let t = (strain - w[0].0) / (w[1].0 - w[0].0);
                return w[0].1 + t * (w[1].1 - w[0].1);
            }
        }
        p[p.len() - 1].1
    }

    /// The slope at `strain` (of the segment the strain lies in; zero beyond the ends).
    fn slope(&self, strain: f64) -> f64 {
        let p = &self.points;
        if strain < p[0].0 {
            return 0.0;
        }
        for w in p.windows(2) {
            if strain <= w[1].0 {
                return (w[1].1 - w[0].1) / (w[1].0 - w[0].0);
            }
        }
        0.0
    }
}

/// Norton creep constants at the temperature.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Norton {
    a: f64,
    n: f64,
    m: f64,
}

/// The J2 law of one material at one temperature.
#[derive(Clone, Debug, PartialEq)]
pub struct J2Plastic {
    elastic: LinearElastic,
    /// Isotropic hardening: yield stress against equivalent plastic strain. `None` is a
    /// zero yield surface (creep without `*PLASTIC`).
    isotropic: Option<Curve>,
    /// Kinematic hardening: the uniaxial stress the back stress contributes, against the
    /// equivalent plastic strain (the kinematic curve minus its first stress).
    kinematic: Option<Curve>,
    creep: Option<Norton>,
}

impl J2Plastic {
    /// Builds the law from the material's `*PLASTIC` and `*CREEP` tables at `temperature`
    /// (the curve of the nearest tabulated temperature, interpolated between two).
    pub fn of(material: &Material, elastic: LinearElastic, temperature: f64) -> Result<Self> {
        let mut isotropic = None;
        let mut kinematic = None;
        if let Some(plastic) = &material.plastic {
            let curve = curve_at(
                plastic
                    .rows
                    .iter()
                    .map(|r| (r.temperature, r.plastic_strain, r.stress)),
                temperature,
            )
            .ok_or_else(|| {
                Error::input(
                    "*PLASTIC",
                    format!("material {} has an empty *PLASTIC table", material.name),
                )
            })?;
            if curve.points.windows(2).any(|w| w[1].0 <= w[0].0) {
                return Err(Error::input(
                    "*PLASTIC",
                    format!(
                        "the plastic strains of material {} must increase along the curve",
                        material.name
                    ),
                ));
            }
            match plastic.hardening {
                Hardening::Isotropic => isotropic = Some(curve),
                // Combined hardening takes its isotropic curve from *CYCLIC HARDENING, which
                // Nodalis does not write: the yield stress then stays at the first point.
                Hardening::Kinematic | Hardening::Combined => {
                    let first = curve.points[0].1;
                    isotropic = Some(Curve {
                        points: vec![(0.0, first)],
                    });
                    kinematic = Some(Curve {
                        points: curve.points.iter().map(|&(e, s)| (e, s - first)).collect(),
                    });
                }
            }
        }
        let creep = match &material.creep {
            Some(c) => {
                let at = |f: fn(&plxs_core::material::CreepRow) -> f64| {
                    plxs_core::material::interpolate(
                        &c.rows
                            .iter()
                            .map(|r| plxs_core::material::TemperatureValue {
                                value: f(r),
                                temperature: r.temperature,
                            })
                            .collect::<Vec<_>>(),
                        temperature,
                    )
                };
                match (at(|r| r.a), at(|r| r.n), at(|r| r.m)) {
                    (Some(a), Some(n), Some(m)) => Some(Norton { a, n, m }),
                    _ => {
                        return Err(Error::input(
                            "*CREEP",
                            format!("material {} has an empty *CREEP table", material.name),
                        ));
                    }
                }
            }
            None => None,
        };
        Ok(Self {
            elastic,
            isotropic,
            kinematic,
            creep,
        })
    }

    fn yield_stress(&self, peeq: f64) -> f64 {
        self.isotropic.as_ref().map_or(0.0, |c| c.value(peeq))
    }

    fn yield_slope(&self, peeq: f64) -> f64 {
        self.isotropic.as_ref().map_or(0.0, |c| c.slope(peeq))
    }

    fn kinematic_stress(&self, peeq: f64) -> f64 {
        self.kinematic.as_ref().map_or(0.0, |c| c.value(peeq))
    }

    fn kinematic_slope(&self, peeq: f64) -> f64 {
        self.kinematic.as_ref().map_or(0.0, |c| c.slope(peeq))
    }

    /// Solves for the plastic multiplier of the increment. `overstress(Δγ)` is
    /// `q_trial - 3μΔγ - Δκ(Δγ) - σ_y(ε̄ + Δγ)`, decreasing in Δγ. Returns the multiplier
    /// and `dΔγ/dq_trial` for the tangent.
    fn multiplier(&self, q_trial: f64, peeq: f64, mu: f64, input: &MaterialInput) -> (f64, f64) {
        let kappa0 = self.kinematic_stress(peeq);
        let overstress = |dg: f64| {
            q_trial
                - 3.0 * mu * dg
                - (self.kinematic_stress(peeq + dg) - kappa0)
                - self.yield_stress(peeq + dg)
        };
        let hardening = |dg: f64| self.kinematic_slope(peeq + dg) + self.yield_slope(peeq + dg);
        // Rate-independent limit: the root of the overstress in [0, q_trial / 3μ].
        let (mut lo, mut hi) = (0.0, q_trial / (3.0 * mu));
        let mut dg = (q_trial - self.yield_stress(peeq)) / (3.0 * mu + hardening(0.0));
        for _ in 0..100 {
            let g = overstress(dg);
            if g.abs() <= 1e-12 * q_trial.max(1.0) {
                break;
            }
            if g > 0.0 {
                lo = dg;
            } else {
                hi = dg;
            }
            let step = g / (3.0 * mu + hardening(dg));
            let next = dg + step;
            dg = if next > lo && next < hi {
                next
            } else {
                0.5 * (lo + hi)
            };
        }
        let dg_rate_independent = dg.clamp(0.0, q_trial / (3.0 * mu));
        let creep = match self.creep {
            Some(c) if input.creep_active && input.dt > 0.0 => c,
            _ => {
                let h = hardening(dg_rate_independent);
                return (dg_rate_independent, 1.0 / (3.0 * mu + h));
            }
        };
        // Viscoplastic: Δγ = c (overstress)^n with c = Δt A t^m. Root of
        // h(Δγ) = Δγ - c overstress(Δγ)^n in [0, Δγ_rate_independent].
        let time_factor = if creep.m == 0.0 {
            1.0
        } else {
            input.time.max(1e-300).powf(creep.m)
        };
        let c = input.dt * creep.a * time_factor;
        let (mut lo, mut hi) = (0.0, dg_rate_independent);
        let mut dg = 0.5 * hi;
        let mut g = overstress(dg);
        for _ in 0..200 {
            let gp = g.max(0.0);
            let h = dg - c * gp.powf(creep.n);
            if h.abs() <= 1e-14 * hi.max(1e-300) {
                break;
            }
            if h < 0.0 {
                lo = dg;
            } else {
                hi = dg;
            }
            let dh = 1.0 + c * creep.n * gp.powf(creep.n - 1.0) * (3.0 * mu + hardening(dg));
            let next = dg - h / dh;
            dg = if next > lo && next < hi && dh.is_finite() {
                next
            } else {
                0.5 * (lo + hi)
            };
            g = overstress(dg);
            if hi - lo <= 1e-15 * dg_rate_independent.max(1e-300) {
                break;
            }
        }
        let gp = overstress(dg).max(0.0);
        let cn = c * creep.n * gp.powf(creep.n - 1.0);
        let ddg_dq = cn / (1.0 + cn * (3.0 * mu + hardening(dg)));
        (dg, ddg_dq)
    }
}

/// The hardening curve at a temperature from rows `(temperature, strain, stress)`: the
/// rows of the nearest tabulated temperatures, interpolated point by point in temperature.
fn curve_at(rows: impl Iterator<Item = (f64, f64, f64)>, temperature: f64) -> Option<Curve> {
    let mut by_temperature: Vec<(f64, Vec<(f64, f64)>)> = Vec::new();
    for (t, e, s) in rows {
        match by_temperature.iter_mut().find(|(bt, _)| *bt == t) {
            Some((_, points)) => points.push((e, s)),
            None => by_temperature.push((t, vec![(e, s)])),
        }
    }
    if by_temperature.is_empty() {
        return None;
    }
    by_temperature.sort_by(|a, b| a.0.total_cmp(&b.0));
    let curve = |points: &[(f64, f64)]| Curve {
        points: points.to_vec(),
    };
    let (first, last) = (
        &by_temperature[0],
        &by_temperature[by_temperature.len() - 1],
    );
    if by_temperature.len() == 1 || temperature <= first.0 {
        return Some(curve(&first.1));
    }
    if temperature >= last.0 {
        return Some(curve(&last.1));
    }
    let upper = by_temperature.iter().position(|(t, _)| *t > temperature)?;
    let (a, b) = (&by_temperature[upper - 1], &by_temperature[upper]);
    let t = (temperature - a.0) / (b.0 - a.0);
    let (ca, cb) = (curve(&a.1), curve(&b.1));
    // Interpolate the stress at the union of the strain abscissae.
    let mut strains: Vec<f64> = a.1.iter().chain(&b.1).map(|p| p.0).collect();
    strains.sort_by(|x, y| x.total_cmp(y));
    strains.dedup();
    Some(Curve {
        points: strains
            .iter()
            .map(|&e| (e, ca.value(e) + t * (cb.value(e) - ca.value(e))))
            .collect(),
    })
}

pub(crate) fn deviator(t: &[f64; 6]) -> [f64; 6] {
    let mean = (t[0] + t[1] + t[2]) / 3.0;
    [t[0] - mean, t[1] - mean, t[2] - mean, t[3], t[4], t[5]]
}

/// The norm of a symmetric tensor in 6-vector form (tensor shear components).
pub(crate) fn norm(t: &[f64; 6]) -> f64 {
    (t[0] * t[0] + t[1] * t[1] + t[2] * t[2] + 2.0 * (t[3] * t[3] + t[4] * t[4] + t[5] * t[5]))
        .sqrt()
}

impl MaterialLaw for J2Plastic {
    fn state_size(&self) -> usize {
        STATE_SIZE
    }

    fn is_nonlinear(&self) -> bool {
        self.isotropic.is_some() || self.creep.is_some()
    }

    fn elastic_matrix(&self) -> [[f64; 6]; 6] {
        self.elastic.matrix()
    }

    fn stress(&self, input: &MaterialInput, state: &mut [f64]) -> MaterialOutput {
        let e = &input.strain;
        // Elastic trial strain (tensor shear components).
        let mut elastic = [e[0], e[1], e[2], 0.5 * e[3], 0.5 * e[4], 0.5 * e[5]];
        for (k, v) in elastic.iter_mut().enumerate() {
            *v -= state[PLASTIC + k];
        }
        let (stress, plastic_increment, tangent) = self.kernel(&elastic, state, input, true);
        for (k, d) in plastic_increment.iter().enumerate() {
            state[PLASTIC + k] += d;
        }
        MaterialOutput { stress, tangent }
    }
}

impl J2Plastic {
    /// The return mapping for an elastic trial strain (tensor components): the stress, the
    /// plastic strain increment (tensor components) and the consistent tangent on
    /// engineering strains. Updates the equivalent plastic strain, the back stress and the
    /// creep rate in `state`; the plastic strain slots are the caller's. Without
    /// `allow_flow` the response is the elastic one (the elastic branch of a point on the
    /// yield surface, for one-sided tangents).
    pub(crate) fn kernel(
        &self,
        elastic: &[f64; 6],
        state: &mut [f64],
        input: &MaterialInput,
        allow_flow: bool,
    ) -> ([f64; 6], [f64; 6], [[f64; 6]; 6]) {
        let mu = self.elastic.shear_modulus();
        let bulk = self.elastic.bulk_modulus();
        let trace = elastic[0] + elastic[1] + elastic[2];
        let dev = deviator(elastic);
        let s_trial: [f64; 6] = std::array::from_fn(|k| 2.0 * mu * dev[k]);
        let mut xi: [f64; 6] = std::array::from_fn(|k| s_trial[k] - state[BACK + k]);
        let xi_norm = norm(&xi);
        let q_trial = (1.5_f64).sqrt() * xi_norm;
        let peeq = state[PEEQ];

        // Creep acts on a zero yield surface only when a *CREEP card is present; without a
        // *PLASTIC card and with creep switched off the material is elastic.
        let yields = self.isotropic.is_some() || (self.creep.is_some() && input.creep_active);
        let f_trial = q_trial - self.yield_stress(peeq);
        let hydrostatic = bulk * trace;
        // A point that sits on the yield surface without a strain increment (the converged
        // state at the start of an increment, where the return mapping left it within
        // round-off) is elastic: whether the next increment loads or unloads it is not
        // known yet, and the elastoplastic tangent would make an unloading increment's
        // first Newton step overshoot by the ratio of the elastic to the hardening modulus.
        let on_surface = YIELD_TOLERANCE * self.yield_stress(peeq).max(0.0);
        if !yields || !allow_flow || f_trial <= on_surface || xi_norm <= 0.0 {
            let mut stress = s_trial;
            for v in stress.iter_mut().take(3) {
                *v += hydrostatic;
            }
            state[CREEP_RATE] = 0.0;
            return (stress, [0.0; 6], self.elastic.matrix());
        }

        let (dg, ddg_dq) = self.multiplier(q_trial, peeq, mu, input);
        for v in xi.iter_mut() {
            *v /= xi_norm;
        }
        let n = xi;
        let root32 = (1.5_f64).sqrt();
        let delta_kappa = self.kinematic_stress(peeq + dg) - self.kinematic_stress(peeq);
        let mut stress = [0.0; 6];
        let mut plastic_increment = [0.0; 6];
        for k in 0..6 {
            let dep = dg * root32 * n[k];
            plastic_increment[k] = dep;
            state[BACK + k] += delta_kappa * (2.0_f64 / 3.0).sqrt() * n[k];
            stress[k] = s_trial[k] - 2.0 * mu * dep;
        }
        for v in stress.iter_mut().take(3) {
            *v += hydrostatic;
        }
        state[PEEQ] = peeq + dg;
        state[CREEP_RATE] = if self.creep.is_some() && input.creep_active && input.dt > 0.0 {
            dg / input.dt
        } else {
            0.0
        };

        // Consistent tangent on engineering strains:
        // D = (K 1⊗1 + 2μθ I_dev) S - 2μ θ̄ n⊗n, S = diag(1,1,1,½,½,½).
        let theta = 1.0 - 3.0 * mu * dg / q_trial;
        let theta_bar = 3.0 * mu * ddg_dq - (1.0 - theta);
        let mut tangent = [[0.0; 6]; 6];
        for i in 0..6 {
            for j in 0..6 {
                let mut v = 0.0;
                if i < 3 && j < 3 {
                    v += bulk;
                    v += 2.0 * mu * theta * (if i == j { 1.0 } else { 0.0 } - 1.0 / 3.0);
                } else if i == j {
                    v += 2.0 * mu * theta * 0.5;
                }
                v -= 2.0 * mu * theta_bar * n[i] * n[j];
                tangent[i][j] = v;
            }
        }
        (stress, plastic_increment, tangent)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plxs_core::material::{Creep, CreepRow, Elastic, ElasticRow, Plastic, PlasticRow};

    fn steel(hardening: Hardening, creep: bool) -> Material {
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
                hardening,
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
            creep: creep.then(|| Creep {
                rows: vec![CreepRow {
                    a: 1e-15,
                    n: 5.0,
                    m: 0.0,
                    temperature: 0.0,
                }],
            }),
            ..Material::default()
        }
    }

    fn law(material: &Material) -> J2Plastic {
        let elastic = LinearElastic::of(material, 0.0).unwrap();
        J2Plastic::of(material, elastic, 0.0).unwrap()
    }

    /// Uniaxial stress for a uniaxial strain history: the lateral strains are iterated so
    /// that the lateral stresses vanish.
    fn uniaxial(law: &J2Plastic, state: &mut [f64], axial: f64, input: MaterialInput) -> f64 {
        let mut lateral = -0.3 * axial;
        let mut out = None;
        for _ in 0..50 {
            let mut trial = state.to_vec();
            let o = law.stress(
                &MaterialInput {
                    strain: [axial, lateral, lateral, 0.0, 0.0, 0.0],
                    ..input
                },
                &mut trial,
            );
            // Newton on the lateral strain with the tangent's lateral block.
            let d = o.tangent[1][1] + o.tangent[1][2];
            let correction = -o.stress[1] / d;
            lateral += correction;
            if correction.abs() < 1e-14 {
                state.copy_from_slice(&trial);
                out = Some(o.stress[0]);
                break;
            }
            out = Some(o.stress[0]);
            if correction.abs() < 1e-14 {
                state.copy_from_slice(&trial);
            }
        }
        out.unwrap()
    }

    fn static_input(strain: [f64; 6]) -> MaterialInput {
        MaterialInput {
            strain,
            temperature: 0.0,
            dt: 1.0,
            time: 1.0,
            creep_active: false,
        }
    }

    #[test]
    fn isotropic_hardening_follows_the_table_in_uniaxial_tension() {
        let law = law(&steel(Hardening::Isotropic, false));
        let mut state = vec![0.0; STATE_SIZE];
        // Elastic: below yield.
        let s = uniaxial(&law, &mut state, 0.001, static_input([0.0; 6]));
        assert!((s - 210.0).abs() < 1e-9, "{s}");
        assert_eq!(state[PEEQ], 0.0);
        // Plastic: total strain = elastic + plastic at 300 MPa on the curve.
        let target = 300.0;
        let peeq = (target - 235.0) / (400.0 - 235.0) * 0.2;
        let axial = target / 210_000.0 + peeq;
        let s = uniaxial(&law, &mut state, axial, static_input([0.0; 6]));
        assert!((s - target).abs() < 1e-6, "{s}");
        assert!((state[PEEQ] - peeq).abs() < 1e-12, "{}", state[PEEQ]);
        // Plastic strain is isochoric.
        let trace = state[PLASTIC] + state[PLASTIC + 1] + state[PLASTIC + 2];
        assert!(trace.abs() < 1e-14);
    }

    #[test]
    fn kinematic_hardening_shows_the_bauschinger_effect() {
        let law = law(&steel(Hardening::Kinematic, false));
        let mut state = vec![0.0; STATE_SIZE];
        let peeq = (300.0 - 235.0) / (400.0 - 235.0) * 0.2;
        let axial = 300.0 / 210_000.0 + peeq;
        let s = uniaxial(&law, &mut state, axial, static_input([0.0; 6]));
        assert!((s - 300.0).abs() < 1e-6, "{s}");
        // Unloading by 2 σ_y reaches the reverse yield point elastically...
        let elastic_range = 2.0 * 235.0;
        let s = uniaxial(
            &law,
            &mut state.clone(),
            axial - elastic_range / 210_000.0 * 0.999,
            static_input([0.0; 6]),
        );
        assert!((s - (300.0 - 0.999 * elastic_range)).abs() < 1e-6, "{s}");
        assert!((state[PEEQ] - peeq).abs() < 1e-12);
        // ... and a little beyond it yields again (isotropic hardening would not).
        let s = uniaxial(
            &law,
            &mut state,
            axial - elastic_range / 210_000.0 * 1.2,
            static_input([0.0; 6]),
        );
        assert!(s > 300.0 - 1.2 * elastic_range + 1.0, "{s}");
        assert!(state[PEEQ] > peeq + 1e-6);
    }

    #[test]
    fn creep_relaxes_by_nortons_law_and_is_off_in_static_steps() {
        let mut material = steel(Hardening::Isotropic, true);
        material.plastic = None;
        let law = law(&material);
        let mut state = vec![0.0; STATE_SIZE];
        let axial = 100.0 / 210_000.0;
        let s = uniaxial(&law, &mut state, axial, static_input([0.0; 6]));
        assert!((s - 100.0).abs() < 1e-9, "static step: {s}");
        assert_eq!(state[PEEQ], 0.0);
        // A visco increment at constant strain: the stress relaxes, Δγ = Δt A q^n at the
        // end of the increment (backward Euler).
        let dt = 50.0;
        let s = uniaxial(
            &law,
            &mut state,
            axial,
            MaterialInput {
                dt,
                time: 51.0,
                creep_active: true,
                ..static_input([0.0; 6])
            },
        );
        assert!(s < 100.0 && s > 50.0, "{s}");
        let expected = dt * 1e-15 * s.powi(5);
        assert!(
            (state[PEEQ] - expected).abs() < 1e-12 * expected.max(1e-9),
            "{} vs {expected}",
            state[PEEQ]
        );
        assert!((state[CREEP_RATE] - expected / dt).abs() < 1e-20);
    }

    #[test]
    fn the_tangent_is_the_derivative_of_the_stress() {
        for (hardening, creep) in [
            (Hardening::Isotropic, false),
            (Hardening::Kinematic, false),
            (Hardening::Combined, true),
        ] {
            let law = law(&steel(hardening, creep));
            let state0 = vec![0.0; STATE_SIZE];
            let strain = [0.004, -0.001, 0.0005, 0.003, -0.002, 0.001];
            let input = MaterialInput {
                strain,
                temperature: 0.0,
                dt: 2.0,
                time: 3.0,
                creep_active: true,
            };
            let mut state = state0.clone();
            let base = law.stress(&input, &mut state);
            assert!(state[PEEQ] > 0.0, "the test strain must yield");
            let h = 1e-7;
            for j in 0..6 {
                let mut plus = input;
                plus.strain[j] += h;
                let mut minus = input;
                minus.strain[j] -= h;
                let sp = law.stress(&plus, &mut state0.clone()).stress;
                let sm = law.stress(&minus, &mut state0.clone()).stress;
                for i in 0..6 {
                    let fd = (sp[i] - sm[i]) / (2.0 * h);
                    assert!(
                        (fd - base.tangent[i][j]).abs() < 1e-5 * 210_000.0,
                        "{hardening:?} creep {creep}: D[{i}][{j}] = {} but finite difference {fd}",
                        base.tangent[i][j]
                    );
                }
            }
        }
    }

    /// The converged state at the start of an increment sits on the yield surface: its
    /// tangent is the elastic one, so that an unloading increment starts with the right
    /// stiffness (regression: unloading a yielded body diverged in its first increment).
    #[test]
    fn a_converged_plastic_point_starts_the_next_increment_elastically() {
        for hardening in [Hardening::Isotropic, Hardening::Kinematic] {
            let law = law(&steel(hardening, false));
            let input = static_input([0.004, -0.0012, -0.0012, 0.0, 0.0, 0.0]);
            let mut state = vec![0.0; STATE_SIZE];
            let loaded = law.stress(&input, &mut state);
            assert!(state[PEEQ] > 0.0, "the test strain must yield");
            // The next increment's first evaluation: same strain, committed state.
            let mut next = state.clone();
            let again = law.stress(&input, &mut next);
            assert_eq!(next[PEEQ], state[PEEQ], "{hardening:?}: no further flow");
            for i in 0..6 {
                assert!((again.stress[i] - loaded.stress[i]).abs() < 1e-9 * 235.0);
            }
            assert_eq!(again.tangent, law.elastic_matrix(), "{hardening:?}");
            // Unloading from there stays elastic.
            let mut unload = input;
            unload.strain[0] -= 0.0005;
            let mut after = state.clone();
            let out = law.stress(&unload, &mut after);
            assert_eq!(after[PEEQ], state[PEEQ]);
            assert_eq!(out.tangent, law.elastic_matrix());
        }
    }
}
