//! Amplitudes: time curves that scale boundary conditions and loads, tabular
//! amplitudes. A boundary condition or load refers to one by name; without one CalculiX
//! ramps the value linearly over a static step and applies it at once in a heat transfer
//! step.

use serde::{Deserialize, Serialize};

/// A tabular amplitude (`*AMPLITUDE`): factors over time, linear in between and constant
/// beyond the first and last point.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Amplitude {
    /// The internal identity other items refer to; the user sees only the name.
    pub id: crate::Id,
    pub name: String,
    /// Whether the times count from the start of each step or of the analysis.
    pub time_span: AmplitudeTime,
    /// Added to every time (`SHIFTX`).
    pub shift_time: f64,
    /// Added to every factor (`SHIFTY`).
    pub shift_amplitude: f64,
    /// Time and factor of each point, in order of time.
    pub points: Vec<[f64; 2]>,
}

impl Amplitude {
    /// A new amplitude with the single point at zero.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            time_span: AmplitudeTime::Step,
            shift_time: 0.0,
            shift_amplitude: 0.0,
            points: vec![[0.0, 0.0]],
        }
    }

    /// The factor at a time, as CalculiX interpolates it, with both shifts applied.
    pub fn value_at(&self, time: f64) -> f64 {
        let time = time - self.shift_time;
        let value = match self.points.as_slice() {
            [] => 0.0,
            [first, ..] if time <= first[0] => first[1],
            [.., last] if time >= last[0] => last[1],
            points => points
                .windows(2)
                .find(|w| time <= w[1][0])
                .map_or(0.0, |w| {
                    let [[t0, a0], [t1, a1]] = [w[0], w[1]];
                    if t1 > t0 {
                        a0 + (a1 - a0) * (time - t0) / (t1 - t0)
                    } else {
                        a1
                    }
                }),
        };
        value + self.shift_amplitude
    }

    /// Why the solver would reject the points, if it would: it needs at least one point and
    /// times that do not decrease.
    pub fn points_problem(&self) -> Option<String> {
        if self.points.is_empty() {
            return Some("The amplitude needs at least one point".into());
        }
        if self.points.iter().flatten().any(|v| !v.is_finite()) {
            return Some("The table contains invalid numbers".into());
        }
        (self.points.windows(2))
            .position(|w| w[1][0] < w[0][0])
            .map(|i| format!("The time in row {} is smaller than before", i + 2))
    }
}

/// The time an amplitude's points refer to (`TIME=`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum AmplitudeTime {
    /// Step time, CalculiX's default.
    #[default]
    Step,
    /// Total time of the analysis (`TIME=TOTAL TIME`).
    Total,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolates_linearly_and_holds_the_ends() {
        let mut amplitude = Amplitude::new("A");
        amplitude.points = vec![[0.0, 0.0], [1.0, 2.0], [3.0, 2.0]];
        assert_eq!(amplitude.value_at(-1.0), 0.0);
        assert_eq!(amplitude.value_at(0.5), 1.0);
        assert_eq!(amplitude.value_at(2.0), 2.0);
        assert_eq!(amplitude.value_at(10.0), 2.0);
        amplitude.shift_time = 1.0;
        amplitude.shift_amplitude = 0.5;
        assert_eq!(amplitude.value_at(1.5), 1.5);
    }

    #[test]
    fn rejects_decreasing_times() {
        let mut amplitude = Amplitude::new("A");
        assert_eq!(amplitude.points_problem(), None);
        amplitude.points = vec![[0.0, 0.0], [2.0, 1.0], [1.0, 1.0]];
        assert!(amplitude.points_problem().unwrap().contains("row 3"));
        amplitude.points.clear();
        assert!(amplitude.points_problem().is_some());
    }
}
