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

/// A value that changes over the step: the time table that a component of a boundary
/// condition or load can take instead of a fixed value, e.g. a displacement in mm over the
/// step time. Linear between the points and constant beyond the first and last, like an
/// amplitude, but with the values themselves rather than factors. Each component has a table
/// of its own, so one boundary condition describes a motion in several directions.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ValueTable {
    /// Step time and value of each point, in order of time.
    pub points: Vec<[f64; 2]>,
}

impl ValueTable {
    /// A table that starts at zero and reaches `value` at `time`, as the default ramp does.
    pub fn ramp(value: f64, time: f64) -> Self {
        Self {
            points: vec![[0.0, 0.0], [time, value]],
        }
    }

    /// The value at a step time.
    pub fn value_at(&self, time: f64) -> f64 {
        self.as_amplitude("").value_at(time)
    }

    /// Why the solver would reject the points, if it would.
    pub fn points_problem(&self) -> Option<String> {
        self.as_amplitude("")
            .points_problem()
            .map(|p| p.replace("The amplitude", "The table"))
    }

    /// Whether any point has a value other than zero.
    pub fn moves(&self) -> bool {
        self.points.iter().any(|p| p[1] != 0.0)
    }

    /// The amplitude the solver takes for the table: the same points, in step time, as
    /// factors of a value of 1.
    pub fn as_amplitude(&self, name: impl Into<String>) -> Amplitude {
        Amplitude {
            id: crate::Id::new(),
            name: name.into(),
            time_span: AmplitudeTime::Step,
            shift_time: 0.0,
            shift_amplitude: 0.0,
            points: self.points.clone(),
        }
    }
}

/// Whether no component has a table, which leaves the field out of the project file.
pub fn no_tables<const N: usize>(tables: &[Option<ValueTable>; N]) -> bool {
    tables.iter().all(Option::is_none)
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
    fn a_table_holds_values_and_names_its_problems() {
        let table = ValueTable {
            points: vec![[0.0, 0.0], [1.0, -0.8], [2.0, 0.8]],
        };
        assert_eq!(table.value_at(0.5), -0.4);
        assert_eq!(table.value_at(5.0), 0.8);
        assert!(table.moves());
        assert!(!ValueTable::ramp(0.0, 1.0).moves());
        assert_eq!(ValueTable::ramp(2.0, 4.0).value_at(1.0), 0.5);
        let backwards = ValueTable {
            points: vec![[1.0, 0.0], [0.0, 1.0]],
        };
        assert!(backwards.points_problem().unwrap().contains("row 2"));
        let empty = ValueTable { points: Vec::new() };
        assert_eq!(
            empty.points_problem().as_deref(),
            Some("The table needs at least one point")
        );
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
