//! The stress update at a material point.
//!
//! Solids and pastes carry the elastic left Cauchy-Green tensor `b_e`. A step pushes it
//! forward with the increment of the deformation gradient, `b_e ← f b_e fᵀ` with
//! `f = I + Δt ∇v`, and the stress follows from the logarithmic (Hencky) elastic strain
//! `ε = ½ ln b_e` as the Kirchhoff stress `τ = K tr ε I + 2G dev ε` (Simo & Hughes,
//! *Computational Inelasticity*, 1998, chapter 9; the same frame as the finite-strain J2 law of
//! `plxs-materials`).
//!
//! The paste flows after Herschel-Bulkley: above the yield stress the equivalent shear
//! stress (`√J₂` of the deviator) is `τ_y + k γ̇ⁿ`. The return mapping of the deviator solves
//! the scalar equation `τ_tr − G Δγ = τ_y + k (Δγ/Δt)ⁿ` for the plastic shear increment
//! `Δγ`, a backward Euler step of a Perzyna-type overstress law, so the viscous part needs
//! no time step limit of its own (the formulation of Yue et al. 2015 for shear-dependent
//! foams, here derived on the Hencky strain). Plastic flow keeps the volume.
//!
//! The fluid has no elastic shear: its stress is the pressure of the volume ratio plus the
//! Newtonian viscous stress of the strain rate. It carries no tension (a free surface
//! opens instead).

use plxs_core::particles::ParticleLaw;
use plxs_materials::finite::eigen_symmetric;

use crate::math::{IDENTITY, Mat3, contract, deviator, mat_mul, sym, transpose};

/// The state of a material point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointState {
    /// The elastic left Cauchy-Green tensor (identity for a fluid).
    pub b: Mat3,
    /// The volume ratio `det F`.
    pub j: f64,
    /// The equivalent plastic shear strain `∫ γ̇_p dt`.
    pub plastic: f64,
}

impl Default for PointState {
    fn default() -> Self {
        Self {
            b: IDENTITY,
            j: 1.0,
            plastic: 0.0,
        }
    }
}

/// What a step of the stress update gives.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PointUpdate {
    /// The Kirchhoff stress `J σ`.
    pub tau: Mat3,
    /// Energy dissipated in the step per unit initial volume.
    pub dissipated: f64,
    /// The elastic energy stored per unit initial volume.
    pub stored: f64,
    /// The equivalent shear rate `√(2 D':D')`.
    pub shear_rate: f64,
}

/// A material law with its moduli as the solver uses them (bulk modulus possibly lowered
/// for a slower wave speed).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Law {
    pub density: f64,
    pub bulk: f64,
    pub shear: f64,
    pub kind: Kind,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    Elastic,
    ViscoPlastic {
        yield_stress: f64,
        consistency: f64,
        exponent: f64,
    },
    Fluid {
        viscosity: f64,
    },
}

impl Law {
    pub fn new(density: f64, law: &ParticleLaw) -> Self {
        let (bulk, shear) = law.moduli();
        let kind = match *law {
            ParticleLaw::Elastic { .. } => Kind::Elastic,
            ParticleLaw::ViscoPlastic {
                yield_stress,
                consistency,
                exponent,
                ..
            } => Kind::ViscoPlastic {
                yield_stress,
                consistency,
                exponent,
            },
            ParticleLaw::Fluid { viscosity, .. } => Kind::Fluid { viscosity },
        };
        Self {
            density,
            bulk,
            shear,
            kind,
        }
    }

    /// The speed of the elastic (pressure) waves.
    pub fn wave_speed(&self) -> f64 {
        ((self.bulk + 4.0 / 3.0 * self.shear) / self.density).sqrt()
    }

    /// The viscosity that limits an explicit time step (the fluid's; zero otherwise).
    pub fn explicit_viscosity(&self) -> f64 {
        match self.kind {
            Kind::Fluid { viscosity } => viscosity,
            _ => 0.0,
        }
    }

    /// The law with its waves slowed to `speed` (if they are faster): the bulk modulus is
    /// lowered, and the shear modulus too where the bulk modulus alone cannot reach it.
    /// Returns the law and the factor the speed was divided by.
    pub fn slowed(&self, speed: f64) -> (Self, f64) {
        let real = self.wave_speed();
        if speed.is_nan() || speed <= 0.0 || speed >= real {
            return (*self, 1.0);
        }
        let target = self.density * speed * speed;
        let mut law = *self;
        let shear_part = 4.0 / 3.0 * self.shear;
        if target > 1.25 * shear_part {
            law.bulk = target - shear_part;
        } else {
            let f = target / (self.bulk + shear_part);
            law.bulk *= f;
            law.shear *= f;
        }
        (law, real / law.wave_speed())
    }

    /// One step: the state at the start, the increment `f` of the deformation gradient, the
    /// volume ratio the stress takes (`det F` at the end, or with F-bar the average over the
    /// neighbourhood), the velocity gradient and the time step. `factor` scales the yield
    /// stress, the consistency and the viscosity (their change with the temperature). The
    /// state is advanced; its own volume ratio follows `f`.
    pub fn update(
        &self,
        state: &mut PointState,
        f: &Mat3,
        j: f64,
        gradient: &Mat3,
        dt: f64,
        factor: f64,
    ) -> PointUpdate {
        let own = state.j * crate::math::det(f);
        let d = sym(gradient);
        let dd = deviator(&d);
        let shear_rate = (2.0 * contract(&dd, &dd)).sqrt();
        match self.kind {
            Kind::Fluid { viscosity } => {
                let viscosity = viscosity * factor;
                // No tension: the volume never grows past its initial one.
                let j = j.min(1.0);
                state.j = own.min(1.0);
                let pressure_tau = self.bulk * j.ln();
                let mut tau = [[0.0; 3]; 3];
                for (a, row) in tau.iter_mut().enumerate() {
                    for (b, value) in row.iter_mut().enumerate() {
                        *value = 2.0 * viscosity * j * dd[a][b];
                    }
                    row[a] += pressure_tau;
                }
                let dissipated = 2.0 * viscosity * j * contract(&dd, &dd) * dt;
                let stored = 0.5 * self.bulk * j.ln().powi(2);
                PointUpdate {
                    tau,
                    dissipated,
                    stored,
                    shear_rate,
                }
            }
            Kind::Elastic | Kind::ViscoPlastic { .. } => {
                let mut b = mat_mul(&mat_mul(f, &state.b), &transpose(f));
                // The volume of the elastic part is the total one (plastic flow keeps it).
                let det_b = crate::math::det(&b);
                if det_b > 0.0 {
                    let s = (j * j / det_b).cbrt();
                    for row in &mut b {
                        for v in row {
                            *v *= s;
                        }
                    }
                }
                let (lambda, vectors) = eigen_symmetric(&b);
                let strain = lambda.map(|l| 0.5 * l.max(1e-300).ln());
                let volumetric = strain[0] + strain[1] + strain[2];
                let mut dev = strain.map(|e| e - volumetric / 3.0);
                let dev_norm = (dev[0] * dev[0] + dev[1] * dev[1] + dev[2] * dev[2]).sqrt();
                let g = self.shear;
                let mut dissipated = 0.0;
                if let Kind::ViscoPlastic {
                    yield_stress,
                    consistency,
                    exponent,
                } = self.kind
                {
                    let (yield_stress, consistency) = (yield_stress * factor, consistency * factor);
                    let trial = std::f64::consts::SQRT_2 * g * dev_norm;
                    if trial > yield_stress && g > 0.0 {
                        let dgamma =
                            plastic_increment(trial, yield_stress, g, consistency, exponent, dt);
                        let new = trial - g * dgamma;
                        dev = dev.map(|e| e * new / trial);
                        state.plastic += dgamma;
                        dissipated = new * dgamma;
                    }
                }
                let strain: [f64; 3] = std::array::from_fn(|a| dev[a] + volumetric / 3.0);
                let tau_principal = dev.map(|e| 2.0 * g * e + self.bulk * volumetric);
                let mut tau = [[0.0; 3]; 3];
                let mut b_new = [[0.0; 3]; 3];
                for a in 0..3 {
                    let e2 = (2.0 * strain[a]).exp();
                    for r in 0..3 {
                        for c in 0..3 {
                            let vv = vectors[r][a] * vectors[c][a];
                            tau[r][c] += tau_principal[a] * vv;
                            b_new[r][c] += e2 * vv;
                        }
                    }
                }
                state.b = b_new;
                state.j = own;
                let stored = g * (dev[0] * dev[0] + dev[1] * dev[1] + dev[2] * dev[2])
                    + 0.5 * self.bulk * volumetric * volumetric;
                PointUpdate {
                    tau,
                    dissipated,
                    stored,
                    shear_rate,
                }
            }
        }
    }
}

/// The plastic shear increment `Δγ ≥ 0` with `trial − G Δγ = yield + k (Δγ/Δt)ⁿ`; `trial`
/// is above `yield`. The left side falls and the right side rises with `Δγ`, so the root
/// is unique and lies below `(trial − yield)/G`.
pub fn plastic_increment(trial: f64, yield_stress: f64, g: f64, k: f64, n: f64, dt: f64) -> f64 {
    let over = trial - yield_stress;
    if k <= 0.0 {
        return over / g;
    }
    if (n - 1.0).abs() < 1e-12 {
        return over / (g + k / dt);
    }
    let residual = |x: f64| over - g * x - k * (x / dt).powf(n);
    let (mut lo, mut hi) = (0.0, over / g);
    let mut x = 0.5 * hi;
    for _ in 0..100 {
        let r = residual(x);
        if r > 0.0 {
            lo = x;
        } else {
            hi = x;
        }
        let slope = g + k * n / dt * (x / dt).max(1e-300).powf(n - 1.0);
        let mut next = x + r / slope;
        if !(next > lo && next < hi) {
            next = 0.5 * (lo + hi);
        }
        if (next - x).abs() <= 1e-14 * hi.max(1e-300) {
            return next;
        }
        x = next;
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn herschel_bulkley_root() {
        let (trial, y, g, k, n, dt) = (500.0, 100.0, 1e4, 50.0, 0.4, 1e-4);
        let x = plastic_increment(trial, y, g, k, n, dt);
        let r = trial - g * x - y - k * (x / dt).powf(n);
        assert!(r.abs() < 1e-8, "{r}");
    }

    /// Steady simple shear at the rate γ̇ ends at the Bingham stress `τ_y + η γ̇`.
    #[test]
    fn bingham_simple_shear() {
        let law = Law::new(
            1000.0,
            &ParticleLaw::ViscoPlastic {
                shear: 1e5,
                bulk: 1e6,
                yield_stress: 200.0,
                consistency: 10.0,
                exponent: 1.0,
            },
        );
        let rate = 5.0;
        let dt = 1e-5;
        let l = [[0.0, rate, 0.0], [0.0; 3], [0.0; 3]];
        let f = [[1.0, rate * dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut state = PointState::default();
        let mut out = PointUpdate::default();
        for _ in 0..20000 {
            out = law.update(&mut state, &f, 1.0, &l, dt, 1.0);
        }
        let expected = 200.0 + 10.0 * rate;
        assert!(
            (out.tau[0][1] - expected).abs() < 0.01 * expected,
            "{:?}",
            out.tau
        );
        assert!((out.shear_rate - rate).abs() < 1e-9);
    }
}
