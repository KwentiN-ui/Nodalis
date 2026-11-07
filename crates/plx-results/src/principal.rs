//! Principal directions of tensor fields, and the vector sources that directional displays
//! read: a vector field such as `DISP` or `FLUX`, or the principal direction of a tensor field
//! such as `STRESS`, which is a line field: a direction without a sign.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{FieldRank, Increment, principal_values};

/// Which principal value of a tensor, and so which principal direction.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Principal {
    #[default]
    Max,
    Mid,
    Min,
    /// The principal value of the largest magnitude, Max where it pulls and Min where it
    /// pushes: the load path through tension and compression alike, as SGN_MAX_ABS_PRI.
    AbsMax,
}

impl Principal {
    pub const ALL: [Principal; 4] = [
        Principal::Max,
        Principal::Mid,
        Principal::Min,
        Principal::AbsMax,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Principal::Max => "Max",
            Principal::Mid => "Mid",
            Principal::Min => "Min",
            Principal::AbsMax => "Abs Max",
        }
    }

    /// Which of the principal values, ordered largest first, this one is: Max or Min for
    /// [`Principal::AbsMax`], whichever is larger in magnitude (Max on a tie).
    pub fn pick(self, [p1, _, p3]: [f64; 3]) -> Principal {
        match self {
            Principal::AbsMax if p3.abs() > p1.abs() => Principal::Min,
            Principal::AbsMax => Principal::Max,
            which => which,
        }
    }

    /// Index into the principal values ordered largest first.
    fn index(self, values: [f64; 3]) -> usize {
        match self.pick(values) {
            Principal::Max | Principal::AbsMax => 0,
            Principal::Mid => 1,
            Principal::Min => 2,
        }
    }
}

/// Position of tensor component (row, column) among xx, yy, zz, xy, yz, zx.
const SLOT: [[usize; 3]; 3] = [[0, 3, 5], [3, 1, 4], [5, 4, 2]];

/// Principal values closer than this share their directions, relative to the largest magnitude.
const REPEATED: f64 = 1e-6;

/// Unit eigenvector of the symmetric tensor for a simple eigenvalue: T - lambda I has rank 2,
/// so the cross product of two of its rows that are not parallel is normal to both.
fn eigenvector(t: [f64; 6], lambda: f64) -> DVec3 {
    let rows = [0, 1, 2].map(|i| {
        let mut row = DVec3::from(SLOT[i].map(|k| t[k]));
        row[i] -= lambda;
        row
    });
    let crosses = [
        rows[0].cross(rows[1]),
        rows[0].cross(rows[2]),
        rows[1].cross(rows[2]),
    ];
    let best = (crosses.into_iter())
        .max_by(|a, b| a.length_squared().total_cmp(&b.length_squared()))
        .unwrap_or(DVec3::ZERO);
    best.try_normalize().unwrap_or(DVec3::NAN)
}

/// Some unit vector normal to `v`.
fn normal_to(v: DVec3) -> DVec3 {
    v.any_orthonormal_vector()
}

/// Turns the direction so that its largest component is positive: the sign of an eigenvector
/// is arbitrary, and a fixed rule keeps the contours of its components from flipping between
/// neighbouring nodes.
fn signed(v: DVec3) -> DVec3 {
    let largest = (0..3)
        .max_by(|&a, &b| v[a].abs().total_cmp(&v[b].abs()))
        .unwrap_or(0);
    if v[largest] < 0.0 { -v } else { v }
}

/// The principal direction of a symmetric tensor (xx, yy, zz, xy, yz, zx) as a unit vector,
/// and its principal value. Where that value is repeated, the direction is any one of the
/// plane it shares with the other one, the three directions staying normal to each other;
/// where all three are equal, it is undefined (NaN).
pub fn principal_direction(t: [f64; 6], which: Principal) -> ([f64; 3], f64) {
    let values = principal_values(t);
    let [p1, p2, p3] = values;
    let which = which.pick(values);
    let value = values[which.index(values)];
    let scale = values.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let tolerance = REPEATED * scale;
    if !scale.is_finite() || p1 - p3 <= tolerance {
        return ([f64::NAN; 3], value);
    }
    let direction = if p1 - p2 <= tolerance {
        let min = eigenvector(t, p3);
        let max = normal_to(min);
        match which {
            Principal::Mid => min.cross(max),
            Principal::Min => min,
            Principal::Max | Principal::AbsMax => max,
        }
    } else if p2 - p3 <= tolerance {
        let max = eigenvector(t, p1);
        let mid = normal_to(max);
        match which {
            Principal::Mid => mid,
            Principal::Min => max.cross(mid),
            Principal::Max | Principal::AbsMax => max,
        }
    } else {
        eigenvector(t, value)
    };
    (signed(direction).to_array(), value)
}

/// How clearly a principal direction is defined: the gap between its principal value and the
/// nearest other one over the largest magnitude, from 0 (repeated value, any direction in a
/// plane or all) to 2 (equal and opposite neighbours, such as pure shear's Max and Min have 1).
/// [`Principal::AbsMax`] also turns by 90 degrees where Max and Min are equally large, so
/// there it is the gap between their magnitudes too.
pub fn anisotropy([p1, p2, p3]: [f64; 3], which: Principal) -> f64 {
    let scale = p1.abs().max(p2.abs()).max(p3.abs());
    if !scale.is_finite() {
        return f64::NAN;
    }
    if scale == 0.0 {
        return 0.0;
    }
    let gap = match which.pick([p1, p2, p3]) {
        Principal::Max | Principal::AbsMax => p1 - p2,
        Principal::Mid => (p1 - p2).min(p2 - p3),
        Principal::Min => p2 - p3,
    };
    let switch = match which {
        Principal::AbsMax => (p1.abs() - p3.abs()).abs(),
        _ => f64::INFINITY,
    };
    gap.min(switch) / scale
}

/// Where a directional display takes its directions from.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum VectorSource {
    /// The raw components of a vector field.
    Vector { field: String },
    /// A principal direction of a tensor field: a direction without a sign.
    Principal { field: String, which: Principal },
}

impl VectorSource {
    pub fn field(&self) -> &str {
        match self {
            VectorSource::Vector { field } | VectorSource::Principal { field, .. } => field,
        }
    }

    /// Whether the directions have no sign, so that they are drawn without arrow heads.
    pub fn is_line_field(&self) -> bool {
        matches!(self, VectorSource::Principal { .. })
    }

    pub fn label(&self) -> String {
        match self {
            VectorSource::Vector { field } => field.clone(),
            VectorSource::Principal { field, which } => {
                format!("Principal {} of {field}", which.label())
            }
        }
    }

    /// The sources the increment offers: each vector field, and the three principal
    /// directions of each tensor field.
    pub fn available(increment: &Increment) -> Vec<VectorSource> {
        let mut sources = Vec::new();
        for field in &increment.fields {
            let field = field.name.clone();
            let source = VectorSource::Vector { field };
            if source.raw_values(increment).is_some() {
                sources.push(source);
                continue;
            }
            for which in Principal::ALL {
                let field = source.field().to_string();
                let principal = VectorSource::Principal { field, which };
                if principal.raw_values(increment).is_some() {
                    sources.push(principal);
                }
            }
        }
        sources
    }

    /// The raw components the directions follow from, per node: x, y, z of a vector or the
    /// six components of a tensor. `None` if the increment has no such field.
    pub fn raw_values<'a>(&self, increment: &'a Increment) -> Option<Vec<&'a [f32]>> {
        let field = increment.field(self.field())?;
        let raw = field.raw_values();
        let fits = match self {
            VectorSource::Vector { .. } => field.rank == FieldRank::Vector && raw.len() == 3,
            VectorSource::Principal { .. } => field.rank.is_tensor() && raw.len() == 6,
        };
        fits.then_some(raw)
    }

    /// The direction as a unit vector, NaN where it is undefined, and its magnitude at every
    /// node: the length of the vector, or the principal value with its sign.
    pub fn sample(&self, increment: &Increment) -> Option<Vec<([f32; 3], f32)>> {
        let raw = self.raw_values(increment)?;
        let count = raw.iter().map(|r| r.len()).min().unwrap_or(0);
        let at = |n: usize, k: usize| raw[k][n] as f64;
        let samples = (0..count).map(|n| match self {
            VectorSource::Vector { .. } => {
                let v = DVec3::new(at(n, 0), at(n, 1), at(n, 2));
                let direction = v.try_normalize().unwrap_or(DVec3::NAN);
                (direction.as_vec3().to_array(), v.length() as f32)
            }
            VectorSource::Principal { which, .. } => {
                let tensor = [0, 1, 2, 3, 4, 5].map(|k| at(n, k));
                let (direction, value) = principal_direction(tensor, *which);
                (direction.map(|v| v as f32), value as f32)
            }
        });
        Some(samples.collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{AnalysisKind, Component, Field};

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        (DVec3::from(a) - DVec3::from(b)).length() < 1e-9
    }

    /// Checks T v = lambda v.
    fn is_eigenpair(t: [f64; 6], v: [f64; 3], lambda: f64) -> bool {
        let tv = [0, 1, 2].map(|i| (0..3).map(|j| t[SLOT[i][j]] * v[j]).sum::<f64>());
        close(tv, v.map(|x| x * lambda)) && (DVec3::from(v).length() - 1.0).abs() < 1e-12
    }

    #[test]
    fn uniaxial_stress_points_along_the_load() {
        let t = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let (max, value) = principal_direction(t, Principal::Max);
        assert!(close(max, [1.0, 0.0, 0.0]) && value == 100.0);
        // The repeated zero values: any two directions normal to the load and each other.
        let (mid, _) = principal_direction(t, Principal::Mid);
        let (min, _) = principal_direction(t, Principal::Min);
        for v in [mid, min] {
            assert!(v[0].abs() < 1e-12 && is_eigenpair(t, v, 0.0));
        }
        assert!(DVec3::from(mid).dot(DVec3::from(min)).abs() < 1e-12);
        let values = principal_values(t);
        // The closed form of the principal values is good to about 1e-8 at repeated ones.
        assert!((anisotropy(values, Principal::Max) - 1.0).abs() < 1e-7);
        assert!(anisotropy(values, Principal::Min).abs() < 1e-7);
    }

    #[test]
    fn pure_shear_turns_the_directions_by_45_degrees() {
        let t = [0.0, 0.0, 0.0, 50.0, 0.0, 0.0];
        let h = 0.5f64.sqrt();
        let (max, value) = principal_direction(t, Principal::Max);
        assert!(close(max, [h, h, 0.0]) && (value - 50.0).abs() < 1e-9);
        let (mid, _) = principal_direction(t, Principal::Mid);
        assert!(close(mid, [0.0, 0.0, 1.0]));
        let (min, value) = principal_direction(t, Principal::Min);
        // The largest component is turned positive.
        assert!(close(min, [h, -h, 0.0]) || close(min, [-h, h, 0.0]));
        assert!(min.iter().cloned().fold(f64::MIN, f64::max) > 0.0);
        assert!((value + 50.0).abs() < 1e-9);
        let values = principal_values(t);
        for which in [Principal::Max, Principal::Mid, Principal::Min] {
            assert!((anisotropy(values, which) - 1.0).abs() < 1e-9);
        }
        // Tension and compression are equally large: Abs Max could take either.
        assert!(anisotropy(values, Principal::AbsMax) < 1e-9);
    }

    #[test]
    fn abs_max_follows_tension_and_compression() {
        // Uniaxial tension along x, then compression along y.
        let (tension, value) =
            principal_direction([80.0, 0.0, 0.0, 0.0, 0.0, 0.0], Principal::AbsMax);
        assert_eq!((tension, value), ([1.0, 0.0, 0.0], 80.0));
        let pushed = [0.0, -80.0, 0.0, 0.0, 0.0, 0.0];
        let (compression, value) = principal_direction(pushed, Principal::AbsMax);
        assert_eq!((compression, value), ([0.0, 1.0, 0.0], -80.0));
        // Max alone has no direction there: 0 is repeated.
        assert!(anisotropy(principal_values(pushed), Principal::Max) < 1e-9);
        assert_eq!(anisotropy(principal_values(pushed), Principal::AbsMax), 1.0);
    }

    #[test]
    fn hydrostatic_state_has_no_direction() {
        let t = [-7.0, -7.0, -7.0, 0.0, 0.0, 0.0];
        for which in Principal::ALL {
            let (direction, value) = principal_direction(t, which);
            assert!(direction.iter().all(|v| v.is_nan()) && value == -7.0);
            assert_eq!(anisotropy(principal_values(t), which), 0.0);
        }
        let ([x, ..], _) = principal_direction([0.0; 6], Principal::Max);
        assert!(x.is_nan(), "a stress-free node has no direction");
        assert_eq!(anisotropy([0.0; 3], Principal::Max), 0.0);
    }

    #[test]
    fn general_tensor_gives_orthonormal_eigenvectors() {
        let t = [40.0, -10.0, 25.0, 12.0, -7.0, 3.0];
        let values = principal_values(t);
        let directions = Principal::ALL.map(|which| {
            let (v, lambda) = principal_direction(t, which);
            assert!(is_eigenpair(t, v, lambda), "{which:?}");
            assert_eq!(lambda, values[which.index(values)]);
            DVec3::from(v)
        });
        assert_eq!(
            directions[3], directions[0],
            "40 outweighs the smallest value"
        );
        assert!(directions[0].dot(directions[1]).abs() < 1e-9);
        assert!(directions[1].dot(directions[2]).abs() < 1e-9);
    }

    fn increment() -> Increment {
        let column = |name: &str, values: [f32; 2]| Component {
            name: name.into(),
            values: values.to_vec(),
            derived: false,
            out_of_plane: false,
        };
        let stress = ["S11", "S22", "S33", "S12", "S23", "S13"]
            .iter()
            .enumerate()
            .map(|(k, n)| column(n, if k == 1 { [0.0, -20.0] } else { [0.0; 2] }))
            .collect();
        let disp = vec![
            column("U1", [3.0, 0.0]),
            column("U2", [0.0, 0.0]),
            column("U3", [4.0, 0.0]),
        ];
        let mut fields = vec![Field::new("DISP", disp), Field::new("STRESS", stress)];
        fields.iter_mut().for_each(crate::add_derived_components);
        Increment {
            step: 1,
            increment: 1,
            kind: AnalysisKind::Static,
            value: 1.0,
            fields,
            nodal_diameter: None,
        }
    }

    #[test]
    fn sources_of_vector_and_tensor_fields() {
        let increment = increment();
        let sources = VectorSource::available(&increment);
        assert_eq!(sources.len(), 5);
        assert_eq!(sources[0].label(), "DISP");
        assert_eq!(sources[1].label(), "Principal Max of STRESS");
        assert!(!sources[0].is_line_field() && sources[4].is_line_field());
        assert_eq!(sources[4].label(), "Principal Abs Max of STRESS");

        let disp = sources[0].sample(&increment).unwrap();
        assert_eq!(disp[0], ([0.6, 0.0, 0.8], 5.0));
        assert!(disp[1].0[0].is_nan(), "no direction without displacement");

        let min = VectorSource::Principal {
            field: "STRESS".into(),
            which: Principal::Min,
        };
        let samples = min.sample(&increment).unwrap();
        assert!(samples[0].0[0].is_nan());
        assert_eq!(samples[1], ([0.0, 1.0, 0.0], -20.0));
        assert_eq!(min.raw_values(&increment).unwrap().len(), 6);
        let wrong = VectorSource::Vector {
            field: "STRESS".into(),
        };
        assert!(wrong.sample(&increment).is_none());
    }
}
