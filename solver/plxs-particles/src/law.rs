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
//! The granular material flows after the μ(I) rheology of Jop, Forterre & Pouliquen (2006):
//! a Drucker-Prager cone whose friction rises with the inertial number
//! `I = γ̇ d √(ρ/p)` from its static value to a limit. The return mapping is the same scalar
//! equation as for the paste, `τ_tr − G Δγ = p μ(I(Δγ/Δt))`, at the pressure of the trial
//! state, as Dunatunga & Kamrin (2015) do it in their MPM; plastic flow keeps the volume. Like
//! the fluid it carries no tension: grains that move apart are free of stress until they
//! touch again.
//!
//! The fluid has no elastic shear: its stress is the pressure of the volume ratio plus the
//! Newtonian viscous stress of the strain rate. It carries no tension (a free surface
//! opens instead).

use plxs_core::particles::{FlowTables, ParticleLaw, table_value};
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
    Granular {
        friction: f64,
        friction_limit: f64,
        grain: f64,
        inertial: f64,
        /// The fine grains of a mixture of two sizes (zero: one size).
        fine_grain: f64,
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
            ParticleLaw::Granular {
                friction,
                friction_limit,
                grain,
                inertial,
                fine_grain,
                ..
            } => Kind::Granular {
                friction,
                friction_limit,
                grain,
                inertial,
                fine_grain,
            },
        };
        Self {
            density,
            bulk,
            shear,
            kind,
        }
    }

    /// The law with the values of its flow at the temperature `t`: those that `tables`
    /// give over the temperature, the others as they are.
    pub fn at_temperature(&self, tables: &FlowTables, t: f64) -> Self {
        let mut law = *self;
        let at = |value: &mut f64, table: &[[f64; 2]]| {
            if !table.is_empty() {
                *value = table_value(table, t);
            }
        };
        match &mut law.kind {
            Kind::ViscoPlastic {
                yield_stress,
                consistency,
                ..
            } => {
                at(yield_stress, &tables.yield_stress);
                at(consistency, &tables.consistency);
            }
            Kind::Fluid { viscosity } => at(viscosity, &tables.viscosity),
            Kind::Elastic | Kind::Granular { .. } => {}
        }
        law
    }

    /// The law of a mixture of two grain sizes with the volume fraction `fine` of the fine
    /// ones: its grain diameter is the mean of the two. Other laws stay as they are.
    pub fn mixed(&self, fine: f64) -> Self {
        let mut law = *self;
        if let Kind::Granular {
            grain, fine_grain, ..
        } = &mut law.kind
            && *fine_grain > 0.0
        {
            *grain = fine * *fine_grain + (1.0 - fine) * *grain;
        }
        law
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
    /// neighbourhood), the velocity gradient and the time step. The state is advanced; its
    /// own volume ratio follows `f`.
    pub fn update(
        &self,
        state: &mut PointState,
        f: &Mat3,
        j: f64,
        gradient: &Mat3,
        dt: f64,
    ) -> PointUpdate {
        let own = state.j * crate::math::det(f);
        let d = sym(gradient);
        let dd = deviator(&d);
        let shear_rate = (2.0 * contract(&dd, &dd)).sqrt();
        match self.kind {
            Kind::Fluid { viscosity } => {
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
            Kind::Elastic | Kind::ViscoPlastic { .. } | Kind::Granular { .. } => {
                // Grains carry no tension: the volume of their stress never grows past the
                // initial one.
                let granular = matches!(self.kind, Kind::Granular { .. });
                let (j, own) = if granular {
                    (j.min(1.0), own.min(1.0))
                } else {
                    (j, own)
                };
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
                if let Kind::Granular {
                    friction,
                    friction_limit,
                    grain,
                    inertial,
                    ..
                } = self.kind
                {
                    let trial = std::f64::consts::SQRT_2 * g * dev_norm;
                    let pressure = -self.bulk * volumetric;
                    let rate = Inertial {
                        friction,
                        friction_limit,
                        inertial,
                        per_rate: grain * (self.density / pressure.max(1e-300)).sqrt(),
                    };
                    if g > 0.0 && trial > friction * pressure {
                        let dgamma = granular_increment(trial, pressure, g, &rate, dt);
                        let new = trial - g * dgamma;
                        dev = if trial > 0.0 {
                            dev.map(|e| e * new / trial)
                        } else {
                            dev
                        };
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

/// The friction of the μ(I) rheology over the shear rate at a pressure: `per_rate` is
/// `d √(ρ/p)`, so that `I = γ̇ per_rate`.
pub struct Inertial {
    pub friction: f64,
    pub friction_limit: f64,
    pub inertial: f64,
    pub per_rate: f64,
}

impl Inertial {
    /// The friction coefficient and its derivative at the shear rate `rate`.
    fn at(&self, rate: f64) -> (f64, f64) {
        let rise = self.friction_limit - self.friction;
        if rise <= 0.0 || self.inertial <= 0.0 {
            return (self.friction, 0.0);
        }
        let i = rate * self.per_rate;
        let s = self.inertial + i;
        (
            self.friction + rise * i / s,
            rise * self.inertial / (s * s) * self.per_rate,
        )
    }
}

/// The plastic shear increment `Δγ ≥ 0` with `trial − G Δγ = p μ(I(Δγ/Δt))`; `trial` is
/// above `p μ_s`. The left side falls and the right side rises with `Δγ`, so the root is
/// unique and lies below `(trial − p μ_s)/G`. Without pressure the shear stress is gone.
pub fn granular_increment(trial: f64, pressure: f64, g: f64, mu: &Inertial, dt: f64) -> f64 {
    let hi_start = (trial - pressure.max(0.0) * mu.friction) / g;
    if pressure <= 0.0 {
        return trial / g;
    }
    let residual = |x: f64| trial - g * x - pressure * mu.at(x / dt).0;
    let (mut lo, mut hi) = (0.0, hi_start);
    let mut x = hi;
    for _ in 0..100 {
        let r = residual(x);
        if r > 0.0 {
            lo = x;
        } else {
            hi = x;
        }
        let slope = g + pressure * mu.at(x / dt).1 / dt;
        let mut next = x + r / slope;
        if !(next > lo && next < hi) {
            next = 0.5 * (lo + hi);
        }
        if (next - x).abs() <= 1e-14 * hi_start.max(1e-300) {
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

    /// Each value of the flow follows its own table over the temperature; a value without
    /// one keeps the law's.
    #[test]
    fn values_over_temperature() {
        let paste = Law::new(
            1000.0,
            &ParticleLaw::ViscoPlastic {
                shear: 1e4,
                bulk: 1e5,
                yield_stress: 200.0,
                consistency: 10.0,
                exponent: 0.5,
            },
        );
        let tables = FlowTables {
            yield_stress: vec![[0.0, 300.0], [40.0, 100.0]],
            consistency: Vec::new(),
            viscosity: vec![[0.0, 99.0]],
        };
        let at = |t| match paste.at_temperature(&tables, t).kind {
            Kind::ViscoPlastic {
                yield_stress,
                consistency,
                ..
            } => (yield_stress, consistency),
            _ => unreachable!(),
        };
        assert_eq!(at(10.0), (250.0, 10.0));
        assert_eq!(at(-5.0), (300.0, 10.0));
        assert_eq!(at(60.0), (100.0, 10.0));
        let fluid = Law::new(
            1000.0,
            &ParticleLaw::Fluid {
                bulk: 1e5,
                viscosity: 1.0,
            },
        );
        let tables = FlowTables {
            viscosity: vec![[20.0, 1.0], [40.0, 0.5]],
            ..Default::default()
        };
        assert_eq!(
            fluid.at_temperature(&tables, 30.0).explicit_viscosity(),
            0.75
        );
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
            out = law.update(&mut state, &f, 1.0, &l, dt);
        }
        let expected = 200.0 + 10.0 * rate;
        assert!(
            (out.tau[0][1] - expected).abs() < 0.01 * expected,
            "{:?}",
            out.tau
        );
        assert!((out.shear_rate - rate).abs() < 1e-9);
    }

    /// Steady simple shear under a pressure ends at the shear stress `μ(I) p` of the rate.
    #[test]
    fn granular_simple_shear() {
        let (friction, friction_limit, grain, inertial) = (0.38, 0.64, 0.5e-3, 0.28);
        let law = Law::new(
            1500.0,
            &ParticleLaw::Granular {
                shear: 1e6,
                bulk: 2e6,
                friction,
                friction_limit,
                grain,
                inertial,
                fine_grain: 0.0,
                fine_fraction: 0.0,
            },
        );
        // A compression of 1 % holds the pressure; the shear rate is so high that I
        // matters.
        let (rate, dt, j) = (50.0, 1e-6, 0.99f64);
        let l = [[0.0, rate, 0.0], [0.0; 3], [0.0; 3]];
        let f = [[1.0, rate * dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut state = PointState {
            j,
            ..Default::default()
        };
        let mut out = PointUpdate::default();
        for _ in 0..20000 {
            out = law.update(&mut state, &f, j, &l, dt);
        }
        let pressure = -(out.tau[0][0] + out.tau[1][1] + out.tau[2][2]) / 3.0;
        assert!((pressure - 2e6 * -j.ln()).abs() < 1e-6 * pressure);
        let i = rate * grain * (1500.0 / pressure).sqrt();
        let mu = friction + (friction_limit - friction) * i / (inertial + i);
        let shear = out.tau[0][1];
        assert!(
            (shear - mu * pressure).abs() < 0.01 * mu * pressure,
            "{shear} {mu} {pressure}"
        );
    }

    /// Grains pulled apart carry no stress.
    #[test]
    fn granular_no_tension() {
        let law = Law::new(
            1500.0,
            &ParticleLaw::Granular {
                shear: 1e6,
                bulk: 2e6,
                friction: 0.5,
                friction_limit: 0.5,
                grain: 1e-3,
                inertial: 0.3,
                fine_grain: 0.0,
                fine_fraction: 0.0,
            },
        );
        let f = [[1.01, 0.002, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut state = PointState::default();
        let out = law.update(&mut state, &f, 1.01, &[[0.0; 3]; 3], 1e-4);
        let norm = contract(&out.tau, &out.tau).sqrt();
        assert!(norm < 1e-6, "{:?}", out.tau);
        assert!(state.j <= 1.0);
    }
}
