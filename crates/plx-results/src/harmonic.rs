//! The results of a steady state dynamics step as the vibration they describe.
//!
//! The solver writes the response to a harmonic load `F cos Ωt` at every frequency as a
//! complex amplitude `z = a + i b`: the real part `a` in the usual fields (`DISP`,
//! `STRESS`, ...) and the imaginary part `b` in the fields with an `I` appended (`DISPI`,
//! `STRESSI`, `TOSTRAII`, `FORCI`). The value over time is
//!
//! `x(t) = Re(z e^(iΩt)) = a cos Ωt − b sin Ωt = A cos(Ωt − θ)`
//!
//! with the amplitude `A = |z|` and the phase lag `θ` behind the load. Without damping the
//! response is in phase or in antiphase with the load and `b` is zero.
//!
//! Nodalis shows these results as the vibration: for every field with an imaginary part
//! [`add_amplitudes`] adds the amplitude (`DISP_AMPLITUDE`, ...) and the phase lag
//! (`DISP_PHASE`, ...), and [`at_phase`] gives the values at any phase angle `Ωt` of the
//! period.

use crate::{Component, Field, FieldRank, Increment, TensorKind};

/// The suffix of the field of the amplitudes of a field.
pub const AMPLITUDE: &str = "_AMPLITUDE";
/// The suffix of the field of the phase lags (degrees) of a field.
pub const PHASE: &str = "_PHASE";

/// The name of the imaginary part of a field: `I` appended, as CalculiX names them in the
/// eight characters of the `.frd` file (`TOSTRAIN` becomes `TOSTRAII`).
pub fn imaginary_name(real: &str) -> String {
    match real {
        "TOSTRAIN" => "TOSTRAII".into(),
        _ => format!("{real}I"),
    }
}

/// The imaginary part of `real` in `increment`.
pub fn imaginary<'a>(increment: &'a Increment, real: &str) -> Option<&'a Field> {
    let name = imaginary_name(real);
    // `DISPI` itself has no imaginary part, though `DISPII` would be its name.
    (name != real).then(|| increment.field(&name)).flatten()
}

/// Whether `field` of `increment` is the imaginary part of another of its fields.
pub fn is_imaginary(increment: &Increment, field: &str) -> bool {
    (increment.fields.iter()).any(|f| f.name != field && imaginary_name(&f.name) == field)
}

/// The fields of `increment` that have an imaginary part, by name.
fn complex_fields(increment: &Increment) -> Vec<String> {
    (increment.fields.iter())
        .filter(|f| !f.name.ends_with(AMPLITUDE) && !f.name.ends_with(PHASE))
        .filter(|f| !is_imaginary(increment, &f.name))
        .filter(|f| imaginary(increment, &f.name).is_some())
        .map(|f| f.name.clone())
        .collect()
}

/// The raw components of `real` with those of the same name in `imaginary` (zero where it
/// has none).
fn pairs<'a>(
    real: &'a Field,
    imaginary: Option<&'a Field>,
) -> Vec<(&'a Component, Option<&'a [f32]>)> {
    (real.components.iter())
        .filter(|c| !c.derived)
        .map(|c| {
            let partner = imaginary
                .and_then(|f| f.component(&c.name))
                .filter(|p| p.values.len() == c.values.len())
                .map(|p| p.values.as_slice());
            (c, partner)
        })
        .collect()
}

/// The field `real` at the phase angle `phase` (radians) of the period: `a cos − b sin` of
/// every raw component, with the derived components (magnitude, equivalent and principal
/// values) of these values.
pub fn at_phase(real: &Field, imaginary: Option<&Field>, phase: f32) -> Field {
    let (cos, sin) = (phase.cos(), phase.sin());
    let components = pairs(real, imaginary)
        .into_iter()
        .map(|(c, b)| Component {
            values: match b {
                Some(b) => (c.values.iter().zip(b))
                    .map(|(a, b)| a * cos - b * sin)
                    .collect(),
                None => c.values.iter().map(|a| a * cos).collect(),
            },
            ..c.clone()
        })
        .collect();
    let mut field = Field {
        name: real.name.clone(),
        components,
        rank: real.rank,
    };
    crate::add_derived_components(&mut field);
    field
}

/// The largest value of `sqrt(Q(a cos φ − b sin φ))` over a period, for a quadratic form
/// `Q` with the bilinear form `B`: `Q(a) cos² + Q(b) sin² − 2 B(a, b) sin cos` peaks at
/// `(Q(a) + Q(b)) / 2 + sqrt(((Q(a) − Q(b)) / 2)² + B(a, b)²)`.
fn peak_of_quadratic(qa: f64, qb: f64, bab: f64) -> f32 {
    let mean = 0.5 * (qa + qb);
    let half = 0.5 * (qa - qb);
    (mean + (half * half + bab * bab).sqrt()).max(0.0).sqrt() as f32
}

/// The bilinear form of the von Mises equivalent, `B(t, t)` its square.
fn mises_form(a: [f64; 6], b: [f64; 6]) -> f64 {
    let d = |t: [f64; 6]| [t[0] - t[1], t[1] - t[2], t[2] - t[0]];
    let (da, db) = (d(a), d(b));
    0.5 * (da[0] * db[0]
        + da[1] * db[1]
        + da[2] * db[2]
        + 6.0 * (a[3] * b[3] + a[4] * b[4] + a[5] * b[5]))
}

/// The amplitudes of `real` with the imaginary part `imaginary`: of every raw component
/// `|a + i b|`, and the largest magnitude (vectors) or equivalent value (tensors) over the
/// period.
pub fn amplitudes(real: &Field, imaginary: Option<&Field>) -> Field {
    let pairs = pairs(real, imaginary);
    let count = pairs.first().map_or(0, |(c, _)| c.values.len());
    let column = |k: usize, b: bool| -> Vec<f64> {
        let (c, im) = &pairs[k];
        match (b, im) {
            (false, _) => c.values.iter().map(|&v| f64::from(v)).collect(),
            (true, Some(im)) => im.iter().map(|&v| f64::from(v)).collect(),
            (true, None) => vec![0.0; count],
        }
    };
    let derived = |name: &str, values: Vec<f32>| Component {
        name: name.to_string(),
        values,
        derived: true,
        out_of_plane: false,
    };
    let mut components: Vec<Component> = Vec::new();
    match real.rank {
        FieldRank::Vector if pairs.len() == 3 => {
            let (a, b): (Vec<_>, Vec<_>) =
                (0..3).map(|k| (column(k, false), column(k, true))).unzip();
            let all = (0..count)
                .map(|i| {
                    let qa: f64 = (0..3).map(|k| a[k][i] * a[k][i]).sum();
                    let qb: f64 = (0..3).map(|k| b[k][i] * b[k][i]).sum();
                    let bab: f64 = (0..3).map(|k| a[k][i] * b[k][i]).sum();
                    peak_of_quadratic(qa, qb, bab)
                })
                .collect();
            components.push(derived("ALL", all));
        }
        FieldRank::Tensor(kind) if pairs.len() == 6 => {
            let (a, b): (Vec<_>, Vec<_>) =
                (0..6).map(|k| (column(k, false), column(k, true))).unzip();
            let factor = if kind == TensorKind::Strain {
                2.0 / 3.0
            } else {
                1.0
            };
            let equivalent = (0..count)
                .map(|i| {
                    let ta = [0, 1, 2, 3, 4, 5].map(|k| a[k][i]);
                    let tb = [0, 1, 2, 3, 4, 5].map(|k| b[k][i]);
                    let peak = peak_of_quadratic(
                        mises_form(ta, ta),
                        mises_form(tb, tb),
                        mises_form(ta, tb),
                    );
                    factor * peak
                })
                .collect();
            let name = if kind == TensorKind::Strain {
                "EQUIVALENT"
            } else {
                "MISES"
            };
            components.push(derived(name, equivalent));
        }
        _ => {}
    }
    components.extend(pairs.iter().map(|(c, b)| {
        Component {
            values: match b {
                Some(b) => (c.values.iter().zip(*b))
                    .map(|(a, b)| a.hypot(*b))
                    .collect(),
                None => c.values.iter().map(|a| a.abs()).collect(),
            },
            ..(*c).clone()
        }
    }));
    Field {
        name: format!("{}{AMPLITUDE}", real.name),
        components,
        rank: real.rank,
    }
}

/// The phase lags of the raw components of `real` behind the load, in degrees from −180
/// to 180: `θ` of `A cos(Ωt − θ)`. A component that hardly moves (below a thousandth of
/// the largest amplitude of the field) has the lag 0, not the angle of round-off.
pub fn phases(real: &Field, imaginary: Option<&Field>) -> Field {
    let pairs = pairs(real, imaginary);
    let largest = (pairs.iter())
        .flat_map(|(c, b)| {
            (c.values.iter().enumerate()).map(move |(i, a)| a.hypot(b.map_or(0.0, |b| b[i])))
        })
        .filter(|v| v.is_finite())
        .fold(0.0_f32, f32::max);
    let tiny = 1e-3 * largest;
    let components = pairs
        .iter()
        .map(|(c, b)| Component {
            values: (c.values.iter().enumerate())
                .map(|(i, &a)| {
                    let b = b.map_or(0.0, |b| b[i]);
                    if !(a.is_finite() && b.is_finite()) {
                        f32::NAN
                    } else if a.hypot(b) <= tiny {
                        0.0
                    } else {
                        -b.atan2(a).to_degrees()
                    }
                })
                .collect(),
            ..(*c).clone()
        })
        .collect();
    Field {
        name: format!("{}{PHASE}", real.name),
        components,
        rank: FieldRank::Scalar,
    }
}

/// Adds the amplitude and phase fields of every field with an imaginary part in front of
/// the fields of a steady state increment, in the order of the fields; nothing when they
/// are there already. Returns whether it added any.
pub fn add_amplitudes(increment: &mut Increment) -> bool {
    if (increment.fields.iter()).any(|f| f.name.ends_with(AMPLITUDE)) {
        return false;
    }
    let mut added = Vec::new();
    for name in complex_fields(increment) {
        let real = increment.field(&name).unwrap();
        let im = imaginary(increment, &name);
        added.push(amplitudes(real, im));
        added.push(phases(real, im));
    }
    if added.is_empty() {
        return false;
    }
    added.append(&mut increment.fields);
    increment.fields = added;
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AnalysisKind;

    fn component(name: &str, values: Vec<f32>) -> Component {
        Component {
            name: name.into(),
            values,
            derived: false,
            out_of_plane: false,
        }
    }

    fn vector(name: &str, values: [[f32; 2]; 3]) -> Field {
        let mut field = Field::new(
            name,
            ["U1", "U2", "U3"]
                .iter()
                .zip(values)
                .map(|(n, v)| component(n, v.to_vec()))
                .collect(),
        );
        crate::add_derived_components(&mut field);
        field
    }

    fn increment() -> Increment {
        // Node 0 swings along x in phase, node 1 in a circle (x and y a quarter apart).
        let real = vector("DISP", [[2.0, 1.0], [0.0, 0.0], [0.0, 0.0]]);
        let mut imaginary = vector("DISPI", [[0.0, 0.0], [0.0, -1.0], [0.0, 0.0]]);
        imaginary.rank = FieldRank::Scalar;
        imaginary.components.retain(|c| !c.derived);
        Increment {
            step: 1,
            increment: 1,
            kind: AnalysisKind::Dynamic,
            value: 10.0,
            fields: vec![real, imaginary],
            nodal_diameter: None,
        }
    }

    #[test]
    fn names_of_imaginary_parts() {
        assert_eq!(imaginary_name("DISP"), "DISPI");
        assert_eq!(imaginary_name("TOSTRAIN"), "TOSTRAII");
        assert_eq!(imaginary_name("ROTATION"), "ROTATIONI");
        let increment = increment();
        assert!(is_imaginary(&increment, "DISPI"));
        assert!(!is_imaginary(&increment, "DISP"));
    }

    #[test]
    fn amplitude_and_phase_lag_of_a_circular_and_a_straight_swing() {
        let mut increment = increment();
        assert!(add_amplitudes(&mut increment));
        assert!(!add_amplitudes(&mut increment), "added once");
        let names: Vec<&str> = increment.fields.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names, ["DISP_AMPLITUDE", "DISP_PHASE", "DISP", "DISPI"]);
        let amplitude = increment.field("DISP_AMPLITUDE").unwrap();
        assert_eq!(amplitude.component("U1").unwrap().values, [2.0, 1.0]);
        assert_eq!(amplitude.component("U2").unwrap().values, [0.0, 1.0]);
        // The circle has the radius 1 at every moment.
        let all = &amplitude.component("ALL").unwrap().values;
        assert!(
            (all[0] - 2.0).abs() < 1e-6 && (all[1] - 1.0).abs() < 1e-6,
            "{all:?}"
        );
        let phase = increment.field("DISP_PHASE").unwrap();
        // b = -1: y = sin Ωt = cos(Ωt - 90°), a quarter period behind the load.
        assert_eq!(phase.component("U2").unwrap().values, [0.0, 90.0]);
        assert_eq!(phase.component("U1").unwrap().values, [0.0, 0.0]);
    }

    #[test]
    fn values_at_a_phase_angle() {
        let increment = increment();
        let real = increment.field("DISP").unwrap();
        let im = imaginary(&increment, "DISP");
        let quarter = at_phase(real, im, std::f32::consts::FRAC_PI_2);
        let u1 = &quarter.component("U1").unwrap().values;
        let u2 = &quarter.component("U2").unwrap().values;
        assert!(u1.iter().all(|v| v.abs() < 1e-6), "{u1:?}");
        assert!((u2[1] - 1.0).abs() < 1e-6, "{u2:?}");
        assert!((quarter.component("ALL").unwrap().values[1] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn the_mises_amplitude_is_the_peak_over_the_period() {
        let tensor = |name: &str, s: [f32; 6]| {
            let names = ["S11", "S22", "S33", "S12", "S23", "S13"];
            Field::new(
                name,
                names
                    .iter()
                    .zip(s)
                    .map(|(n, v)| component(n, vec![v]))
                    .collect(),
            )
        };
        // σ11 = cos Ωt, σ22 = sin Ωt: sampled peak of the equivalent.
        let real = tensor("STRESS", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let im = tensor("STRESSI", [0.0, -1.0, 0.0, 0.0, 0.0, 0.0]);
        let amplitude = amplitudes(&real, Some(&im));
        let peak = (0..3600)
            .map(|k| {
                let field = at_phase(&real, Some(&im), k as f32 * 0.1_f32.to_radians());
                field.component("MISES").unwrap().values[0]
            })
            .fold(0.0, f32::max);
        let mises = amplitude.component("MISES").unwrap().values[0];
        assert!((mises - peak).abs() < 1e-4, "{mises} vs {peak}");
    }
}
