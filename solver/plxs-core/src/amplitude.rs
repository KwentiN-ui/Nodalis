//! `*AMPLITUDE`: a time table that scales loads and boundary values, or a signal the
//! solver evaluates itself (a pulse, a sine, a sweep) to excite a structure in dynamics.

use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Amplitude {
    /// Upper case name.
    pub name: String,
    /// `TIME=TOTAL TIME`: the table is read with the total time, otherwise with the step time.
    pub total_time: bool,
    /// `SHIFTX`, `SHIFTY`: added to every time and value.
    pub shift_x: f64,
    pub shift_y: f64,
    /// (time, value) points in ascending time. Not used when the amplitude is a signal.
    pub points: Vec<(f64, f64)>,
    /// `SHAPE=`: a signal evaluated exactly at every time instead of the table. Always
    /// written: the restart file (postcard) cannot skip fields.
    #[serde(default)]
    pub signal: Option<Signal>,
}

/// Samples per period at which the corners of a signal are placed, where a procedure
/// interpolates the amplitude linearly between corners (the explicit card, increments of a
/// nonlinear step): the peak of a sine drops by 1 - cos(pi / 48), 0.2 %.
pub const SIGNAL_SAMPLES_PER_PERIOD: f64 = 48.0;

/// A signal with values between -1 and 1 that is zero outside `start..end` (`SHAPE=`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Signal {
    /// `SHAPE=PULSE`: a half sine of length `width` from `start`, the smooth stand-in for a
    /// Dirac impulse: its area is `2 width / pi`.
    Pulse { start: f64, width: f64 },
    /// `SHAPE=SINE`: `sin(2 pi f (t - start) + phase)` from `start` to `end`.
    Sine {
        start: f64,
        end: f64,
        frequency: f64,
        /// In radians.
        phase: f64,
    },
    /// `SHAPE=CHIRP`: a sine whose frequency sweeps from `from` at `start` to `to` at `end`,
    /// linearly or, with `logarithmic` (`SWEEP=LOG`), by the same factor per time.
    Chirp {
        start: f64,
        end: f64,
        from: f64,
        to: f64,
        logarithmic: bool,
    },
}

impl Signal {
    /// The time the signal ends.
    pub fn end(&self) -> f64 {
        match *self {
            Signal::Pulse { start, width } => start + width,
            Signal::Sine { end, .. } | Signal::Chirp { end, .. } => end,
        }
    }

    pub fn start(&self) -> f64 {
        match *self {
            Signal::Pulse { start, .. }
            | Signal::Sine { start, .. }
            | Signal::Chirp { start, .. } => start,
        }
    }

    /// The value at a time.
    pub fn value(&self, t: f64) -> f64 {
        if t < self.start() || t > self.end() {
            return 0.0;
        }
        match *self {
            Signal::Pulse { start, width } => {
                if width > 0.0 {
                    (PI * (t - start) / width).sin()
                } else {
                    0.0
                }
            }
            Signal::Sine {
                start,
                frequency,
                phase,
                ..
            } => (2.0 * PI * frequency * (t - start) + phase).sin(),
            Signal::Chirp { .. } => self.phase(t).sin(),
        }
    }

    /// The phase of a chirp at a time inside it, in radians.
    fn phase(&self, t: f64) -> f64 {
        let Signal::Chirp {
            start,
            end,
            from,
            to,
            logarithmic,
        } = *self
        else {
            return 0.0;
        };
        let length = end - start;
        let tau = t - start;
        if length <= 0.0 {
            return 0.0;
        }
        let ratio = if from > 0.0 { to / from } else { 0.0 };
        if logarithmic && ratio > 0.0 && (ratio - 1.0).abs() > 1e-12 {
            let k = ratio.ln() / length;
            2.0 * PI * from * ((k * tau).exp() - 1.0) / k
        } else {
            2.0 * PI * (from * tau + (to - from) * tau * tau / (2.0 * length))
        }
    }

    /// The frequency of the signal at a time inside it: of a pulse the half sine's.
    pub fn frequency_at(&self, t: f64) -> f64 {
        match *self {
            Signal::Pulse { width, .. } => 0.5 / width.max(f64::MIN_POSITIVE),
            Signal::Sine { frequency, .. } => frequency,
            Signal::Chirp {
                start,
                end,
                from,
                to,
                logarithmic,
            } => {
                let s = ((t - start) / (end - start).max(f64::MIN_POSITIVE)).clamp(0.0, 1.0);
                if logarithmic && from > 0.0 && to > 0.0 {
                    from * (to / from).powf(s)
                } else {
                    from + (to - from) * s
                }
            }
        }
    }

    /// The highest frequency the signal reaches.
    pub fn highest_frequency(&self) -> f64 {
        match *self {
            Signal::Chirp { from, to, .. } => from.max(to),
            _ => self.frequency_at(self.start()),
        }
    }

    /// Times at which a linear interpolation of the signal stays close to it:
    /// [`SIGNAL_SAMPLES_PER_PERIOD`] per period of the frequency it has there, from its start
    /// to its end, both included; at most `limit` of them, evenly spread when that cuts.
    pub fn corners(&self, limit: usize) -> Vec<f64> {
        let (start, end) = (self.start(), self.end());
        if end.is_nan() || start.is_nan() || end <= start || limit < 2 {
            return vec![start];
        }
        let mut times = vec![start];
        let mut t = start;
        while t < end && times.len() < limit {
            let f = self.frequency_at(t).max(f64::MIN_POSITIVE);
            let dt = 1.0 / (SIGNAL_SAMPLES_PER_PERIOD * f);
            // A hair before the end counts as the end.
            t = if end - (t + dt) < 1e-9 * (end - start) {
                end
            } else {
                t + dt
            };
            times.push(t);
        }
        if *times.last().unwrap() < end {
            // Too many: evenly spread instead.
            let n = limit - 1;
            return (0..=n)
                .map(|i| start + (end - start) * i as f64 / n as f64)
                .collect();
        }
        times
    }
}

impl Amplitude {
    /// The value at a time: linear between the points, constant before the first and after
    /// the last (CalculiX holds the last value).
    pub fn value(&self, time: f64) -> f64 {
        let t = time - self.shift_x;
        if let Some(signal) = &self.signal {
            return signal.value(t) + self.shift_y;
        }
        let points = &self.points;
        let Some(&(t0, v0)) = points.first() else {
            return self.shift_y;
        };
        let &(t1, v1) = points.last().unwrap_or(&(t0, v0));
        let value = if t <= t0 {
            v0
        } else if t >= t1 {
            v1
        } else {
            // Long tables (sampled signals) are searched by halves.
            let upper = points
                .partition_point(|&(pt, _)| pt <= t)
                .clamp(1, points.len() - 1);
            let (ta, va) = points[upper - 1];
            let (tb, vb) = points[upper];
            if tb == ta {
                vb
            } else {
                va + (t - ta) / (tb - ta) * (vb - va)
            }
        };
        value + self.shift_y
    }

    /// The times of the amplitude's own time axis (before the shift) at which it bends: the
    /// points of a table, the corners of a signal ([`Signal::corners`]).
    pub fn corner_times(&self) -> Vec<f64> {
        match &self.signal {
            Some(signal) => signal.corners(MAX_SIGNAL_CORNERS),
            None => self.points.iter().map(|&(t, _)| t).collect(),
        }
    }
}

/// The most corners a signal gives, which bounds the increments a nonlinear step cuts and
/// the chunks of the explicit card.
pub const MAX_SIGNAL_CORNERS: usize = 1_000_000;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_amplitude_ramps_and_holds() {
        let amplitude = Amplitude {
            name: "A".into(),
            total_time: false,
            shift_x: 0.0,
            shift_y: 0.0,
            points: vec![(0.0, 0.0), (1.0, 2.0), (2.0, 2.0)],
            signal: None,
        };
        assert_eq!(amplitude.value(-1.0), 0.0);
        assert_eq!(amplitude.value(0.5), 1.0);
        assert_eq!(amplitude.value(1.5), 2.0);
        assert_eq!(amplitude.value(9.0), 2.0);
    }

    #[test]
    fn signals_are_zero_outside_and_follow_their_formula_inside() {
        let pulse = Signal::Pulse {
            start: 0.1,
            width: 0.02,
        };
        assert_eq!(pulse.value(0.05), 0.0);
        assert!((pulse.value(0.11) - 1.0).abs() < 1e-12);
        assert_eq!(pulse.value(0.2), 0.0);
        let sine = Signal::Sine {
            start: 1.0,
            end: 2.0,
            frequency: 10.0,
            phase: 0.0,
        };
        assert!((sine.value(1.025) - 1.0).abs() < 1e-12);
        assert_eq!(sine.value(2.5), 0.0);
        let chirp_of = |logarithmic| Signal::Chirp {
            start: 0.0,
            end: 1.0,
            from: 10.0,
            to: 100.0,
            logarithmic,
        };
        let chirp = chirp_of(false);
        assert!((chirp.frequency_at(0.5) - 55.0).abs() < 1e-12);
        // The phase grows by 2 pi times the frequency: a numeric derivative matches.
        let h = 1e-7;
        for t in [0.1, 0.5, 0.9] {
            for log in [false, true] {
                let c = chirp_of(log);
                let rate = (c.phase(t + h) - c.phase(t - h)) / (2.0 * h) / (2.0 * PI);
                assert!((rate - c.frequency_at(t)).abs() < 1e-4 * c.frequency_at(t));
            }
        }
        let log = chirp_of(true);
        assert!((log.frequency_at(0.5) - 10.0_f64.powf(1.5)).abs() < 1e-9);
    }

    #[test]
    fn a_signal_amplitude_is_evaluated_exactly_and_gives_corners() {
        let amplitude = Amplitude {
            name: "S".into(),
            total_time: false,
            shift_x: 0.0,
            shift_y: 0.0,
            points: Vec::new(),
            signal: Some(Signal::Sine {
                start: 0.0,
                end: 1.0,
                frequency: 2.0,
                phase: 0.0,
            }),
        };
        assert!((amplitude.value(0.125) - 1.0).abs() < 1e-12);
        let corners = amplitude.corner_times();
        assert_eq!(corners.len(), 97);
        assert_eq!(*corners.last().unwrap(), 1.0);
        let capped = Signal::Sine {
            start: 0.0,
            end: 1.0,
            frequency: 1e3,
            phase: 0.0,
        }
        .corners(11);
        assert_eq!(capped.len(), 11);
        assert!((capped[5] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn long_tables_interpolate_like_short_ones() {
        let points: Vec<(f64, f64)> = (0..1000).map(|i| (i as f64, (i * i) as f64)).collect();
        let amplitude = Amplitude {
            name: "T".into(),
            total_time: false,
            shift_x: 0.0,
            shift_y: 0.0,
            points,
            signal: None,
        };
        assert_eq!(amplitude.value(10.5), 110.5);
        assert_eq!(amplitude.value(998.0), 996004.0);
    }
}
