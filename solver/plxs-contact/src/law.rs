//! Pressure-overclosure relations of `*SURFACE BEHAVIOR` and the Coulomb friction of
//! `*FRICTION`, as CalculiX's penalty contact applies them. Everything here was derived from
//! the CalculiX manual (keywords `*SURFACE BEHAVIOR`, `*FRICTION`, sections "Node-to-Face
//! Penalty Contact" and "Face-to-Face Penalty Contact") and from runs of ccx 2.21 as a black
//! box; no CalculiX code was consulted. What was observed:
//!
//! - **Face-to-face penalty** evaluates the relation only for overclosures `d > 0`: linear
//!   `p = K d`, exponential `p = p0 · 10^(2 d / c0)` (so `c0` is the clearance at which the
//!   pressure is `p0 / 100`), tabular by linear interpolation; `σ∞` and `c0` of the linear
//!   relation are ignored. `HARD` is linear with `K = 50 E`.
//! - **Node-to-face penalty** smooths the linear relation so that it is differentiable at
//!   zero and tends to the tension `-σ∞` for large clearances:
//!   `p(d) = K d / 2 + (K d / π) · atan(K d / (π σ∞))`; with `σ∞ = 0` (not given) CalculiX
//!   takes `σ∞ = E / 70000` of the slave's material (3 MPa for steel in N/mm). The
//!   exponential and tabular relations are evaluated unsmoothed for clearances as well
//!   (tabular: zero below the first point, constant beyond the last). A contact spring
//!   exists while the clearance is below a cutoff: `5 c0` when `c0` is given, else
//!   `10⁻³ √A` with `A` the area of the slave node (exponential: `c0`).
//! - **Tied** keeps the normal and tangential relative displacement with the stiffness `K`
//!   and the stick slope; it never separates or slides.
//! - **Friction**: the tangential traction grows with the stick slope `λ` until it reaches
//!   `μ p` and slides from there (Coulomb, regularised); without a stick slope CalculiX
//!   takes `E / 2` of the first material.

use plxs_core::contact::{Friction, SurfaceBehavior};

/// Penalty factor of hard mortar contact relative to `E / h`.
pub const MORTAR_STIFFNESS: f64 = 1e4;

/// A pressure-overclosure relation resolved with the model's defaults.
#[derive(Clone, Debug, PartialEq)]
pub enum Law {
    /// `K`, `σ∞` (resolved, used for node-to-face only).
    Linear {
        k: f64,
        sigma_inf: f64,
    },
    Exponential {
        c0: f64,
        p0: f64,
    },
    /// `(overclosure, pressure)` sorted by overclosure.
    Tabular(Vec<(f64, f64)>),
    Tied {
        k: f64,
    },
}

impl Law {
    /// Resolves a behavior: `HARD` becomes `LINEAR` with `K = 50 E`, a missing `σ∞` becomes
    /// `E / 70000` (`young` is the slave material's modulus).
    pub fn resolve(behavior: Option<&SurfaceBehavior>, young: f64) -> Law {
        let default_sigma = young / 70_000.0;
        match behavior {
            None | Some(SurfaceBehavior::Hard) => Law::Linear {
                k: 50.0 * young,
                sigma_inf: default_sigma,
            },
            Some(SurfaceBehavior::Linear { k, sigma_inf, .. }) => Law::Linear {
                k: *k,
                sigma_inf: match sigma_inf {
                    Some(s) if *s > 0.0 => *s,
                    _ => default_sigma,
                },
            },
            Some(SurfaceBehavior::Exponential { c0, p0 }) => Law::Exponential { c0: *c0, p0: *p0 },
            Some(SurfaceBehavior::Tabular(rows)) => {
                let mut points: Vec<(f64, f64)> = rows.iter().map(|&(p, d)| (d, p)).collect();
                points.sort_by(|a, b| a.0.total_cmp(&b.0));
                Law::Tabular(points)
            }
            Some(SurfaceBehavior::Tied { k }) => Law::Tied { k: *k },
        }
    }

    /// The relation of a `TYPE=MORTAR` pair: without a behavior (or `HARD`) the exact,
    /// non-penetrating contact of CalculiX's Lagrange multipliers, approximated by a penalty
    /// so stiff that the remaining overclosure is negligible against the deformation
    /// (`K = MORTAR_STIFFNESS · E / h`, `h` the typical face size); a given relation as for
    /// face-to-face contact.
    pub fn resolve_mortar(behavior: Option<&SurfaceBehavior>, young: f64, typical: f64) -> Law {
        match behavior {
            None | Some(SurfaceBehavior::Hard) => Law::Linear {
                k: MORTAR_STIFFNESS * young / typical.max(1e-300),
                sigma_inf: young / 70_000.0,
            },
            Some(b) => Law::resolve(Some(b), young),
        }
    }

    pub fn is_tied(&self) -> bool {
        matches!(self, Law::Tied { .. })
    }

    /// Pressure and its derivative at the overclosure `d` for a node-to-face spring
    /// (smoothed linear relation, tension within the cutoff).
    pub fn node_pressure(&self, d: f64) -> (f64, f64) {
        match self {
            Law::Linear { k, sigma_inf } => {
                let kd = k * d;
                let x = kd / (std::f64::consts::PI * sigma_inf);
                let p = 0.5 * kd + kd / std::f64::consts::PI * x.atan();
                let dp = 0.5 * k + k / std::f64::consts::PI * (x.atan() + x / (1.0 + x * x));
                (p, dp)
            }
            Law::Exponential { c0, p0 } => exponential(*c0, *p0, d),
            Law::Tabular(points) => tabular(points, d),
            Law::Tied { k } => (k * d, *k),
        }
    }

    /// The linear relation `p = K d` with a rounded start, compression only: `K d²/(2δ)` up
    /// to `d = δ`, `K (d − δ/2)` beyond (other relations as for a face-to-face point). The
    /// relation is convex and has a continuous slope: Newton's iterations for a light body
    /// resting on a stiff spring (gravity alone presses a part by picometres onto another)
    /// approach the overclosure from above instead of jumping between a closed spring and an
    /// open one, which with a kink at zero they do as long as the overclosure is below the
    /// accuracy of a correction.
    pub fn rounded_pressure(&self, d: f64, delta: f64) -> (f64, f64) {
        match self {
            Law::Linear { k, .. } if delta > 0.0 => {
                if d <= 0.0 {
                    (0.0, 0.0)
                } else if d < delta {
                    (0.5 * k * d * d / delta, k * d / delta)
                } else {
                    (k * (d - 0.5 * delta), *k)
                }
            }
            _ => self.face_pressure(d),
        }
    }

    /// The energy `∫₀ᵈ p` of [`Law::rounded_pressure`].
    pub fn rounded_energy(&self, d: f64, delta: f64) -> f64 {
        match self {
            Law::Linear { k, .. } if delta > 0.0 => {
                if d <= 0.0 {
                    0.0
                } else if d < delta {
                    k * d * d * d / (6.0 * delta)
                } else {
                    k * (0.5 * d * d - 0.5 * delta * d + delta * delta / 6.0)
                }
            }
            _ => self.face_energy(d),
        }
    }

    /// Pressure and its derivative at the overclosure `d` for a face-to-face integration
    /// point: compression only, nothing for a clearance.
    pub fn face_pressure(&self, d: f64) -> (f64, f64) {
        match self {
            Law::Tied { k } => (k * d, *k),
            _ if d <= 0.0 => (0.0, 0.0),
            Law::Linear { k, .. } => (k * d, *k),
            Law::Exponential { c0, p0 } => exponential(*c0, *p0, d),
            Law::Tabular(points) => tabular(points, d),
        }
    }

    /// The energy `∫₀ᵈ p` of the relation (face-to-face evaluation).
    pub fn face_energy(&self, d: f64) -> f64 {
        match self {
            Law::Tied { k } => 0.5 * k * d * d,
            _ if d <= 0.0 => 0.0,
            Law::Linear { k, .. } => 0.5 * k * d * d,
            Law::Exponential { c0, p0 } => {
                let b = 2.0 * std::f64::consts::LN_10 / c0;
                p0 / b * ((b * d).exp() - 1.0)
            }
            Law::Tabular(points) => {
                // Trapezoids up to d.
                let mut e = 0.0;
                let mut last = (0.0, 0.0);
                for &(x, p) in points {
                    if x >= d {
                        let (p_d, _) = tabular(points, d);
                        e += 0.5 * (last.1 + p_d) * (d - last.0);
                        return e;
                    }
                    e += 0.5 * (last.1 + p) * (x - last.0);
                    last = (x, p);
                }
                e + last.1 * (d - last.0)
            }
        }
    }

    /// The distance from the master surface within which a node-to-face spring exists.
    pub fn node_cutoff(&self, c0: Option<f64>, area: f64) -> f64 {
        match self {
            Law::Exponential { c0, .. } => *c0,
            _ => match c0 {
                Some(c) if c > 0.0 => 5.0 * c,
                _ => 1e-3 * area.abs().sqrt(),
            },
        }
    }
}

fn exponential(c0: f64, p0: f64, d: f64) -> (f64, f64) {
    let b = 2.0 * std::f64::consts::LN_10 / c0;
    let p = p0 * (b * d).exp();
    (p, b * p)
}

/// The tabular relation: zero below the first point, linear interpolation, constant beyond
/// the last point. Outside the table the derivative is that of the nearest segment, so
/// that a spring at zero pressure (or saturated) still has a stiffness for the next
/// iteration instead of leaving the body unsupported.
fn tabular(points: &[(f64, f64)], d: f64) -> (f64, f64) {
    let Some(&first) = points.first() else {
        return (0.0, 0.0);
    };
    let slope = |w: &[(f64, f64)]| {
        let ((x0, p0), (x1, p1)) = (w[0], w[1]);
        if x1 > x0 { (p1 - p0) / (x1 - x0) } else { 0.0 }
    };
    if points.len() == 1 {
        return (if d < first.0 { 0.0 } else { first.1 }, 0.0);
    }
    if d < first.0 {
        return (0.0, slope(&points[..2]));
    }
    for w in points.windows(2) {
        if d <= w[1].0 {
            let s = slope(w);
            return (w[0].1 + s * (d - w[0].0), s);
        }
    }
    let n = points.len();
    (points[n - 1].1, slope(&points[n - 2..]))
}

/// Coulomb friction with a stick slope.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrictionLaw {
    pub mu: f64,
    /// Tangential stiffness per area before sliding (`λ`).
    pub stick: f64,
}

impl FrictionLaw {
    /// Resolves `*FRICTION`; a missing stick slope is `E / 2`.
    pub fn resolve(friction: Option<&Friction>, young: f64) -> Option<FrictionLaw> {
        friction.map(|f| FrictionLaw {
            mu: f.coefficient,
            stick: match f.stick_slope {
                Some(s) if s > 0.0 => s,
                _ => 0.5 * young,
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn smoothed_linear_relation_matches_calculix_samples() {
        // ccx 2.21, K = 1000, σ∞ = 5: (d, p) observed.
        let law = Law::Linear {
            k: 1000.0,
            sigma_inf: 5.0,
        };
        for (d, p) in [
            (0.02, 15.7614),
            (0.01, 6.80449),
            (-0.02, -4.23845),
            (-0.5, -4.99836),
            (0.0005, 0.255063),
        ] {
            let (got, _) = law.node_pressure(d);
            assert!(
                (got - p).abs() < 2e-3 * p.abs().max(0.1),
                "d={d}: {got} vs {p}"
            );
        }
        // The derivative is the slope of the relation.
        let h = 1e-7;
        for d in [-0.03, -0.001, 0.0, 0.004, 0.05] {
            let (_, dp) = law.node_pressure(d);
            let fd = (law.node_pressure(d + h).0 - law.node_pressure(d - h).0) / (2.0 * h);
            assert!(
                (dp - fd).abs() < 1e-4 * dp.abs().max(1.0),
                "d={d}: {dp} vs {fd}"
            );
        }
    }

    #[test]
    fn exponential_relation_matches_calculix_samples() {
        let law = Law::Exponential { c0: 0.5, p0: 10.0 };
        for (d, p) in [
            (0.1, 25.1188),
            (-0.1, 3.98107),
            (0.3, 158.487),
            (-0.49, 0.109648),
        ] {
            let (got, _) = law.node_pressure(d);
            assert!((got - p).abs() < 1e-4 * p, "d={d}: {got} vs {p}");
        }
        assert_eq!(law.face_pressure(-0.1), (0.0, 0.0));
    }

    #[test]
    fn tabular_relation_interpolates_and_saturates() {
        let law = Law::resolve(
            Some(&SurfaceBehavior::Tabular(vec![
                (0.0, 0.0),
                (10.0, 0.01),
                (15.0, 0.02),
                (100.0, 0.1),
            ])),
            210_000.0,
        );
        assert!((law.node_pressure(0.005).0 - 5.0).abs() < 1e-12);
        assert!((law.node_pressure(0.05).0 - 46.875).abs() < 1e-12);
        assert!((law.node_pressure(0.15).0 - 100.0).abs() < 1e-12);
        assert_eq!(law.node_pressure(-0.005).0, 0.0);
    }

    #[test]
    fn hard_is_stiff_linear_and_cutoffs_follow_the_observed_rules() {
        let law = Law::resolve(Some(&SurfaceBehavior::Hard), 210_000.0);
        assert_eq!(
            law,
            Law::Linear {
                k: 1.05e7,
                sigma_inf: 3.0
            }
        );
        assert!((law.node_cutoff(None, 25.0) - 0.005).abs() < 1e-15);
        assert!((law.node_cutoff(Some(0.1), 25.0) - 0.5).abs() < 1e-15);
    }
}
