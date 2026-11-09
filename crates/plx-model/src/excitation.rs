//! Excitation: a force that follows a signal (a short impulse, a sine, a frequency sweep) to
//! shake a structure in a dynamic step. One load with a choice of signal; the solver
//! evaluates the signal itself at every time step, so no table has to sample it.

use serde::{Deserialize, Serialize};

use plxs_core::amplitude::Signal as IrSignal;

/// The force of an [`crate::LoadKind::Excitation`]: the peak force on the region, spread
/// like a [`crate::LoadKind::Force`], times the signal.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Excitation {
    /// The peak force; the signal swings between it and its negative.
    pub value: [f64; 3],
    /// The whole force at every node of the region instead of spread over it.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub each_node: bool,
    pub signal: Signal,
}

impl Default for Excitation {
    fn default() -> Self {
        Self {
            value: [0.0; 3],
            each_node: false,
            signal: Signal::default(),
        }
    }
}

/// How an excitation varies over the step time. All times count from the start of the step;
/// a signal without an end runs to the end of the step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Signal {
    /// A short half sine of length `width` from `time`: the smooth stand-in for a Dirac
    /// impulse, which excites every frequency up to about `1.5 / width` at once.
    Impulse { time: f64, width: f64 },
    /// A sine of one frequency; the phase in degrees.
    Sine {
        frequency: f64,
        phase: f64,
        start: f64,
        end: Option<f64>,
    },
    /// A sine whose frequency sweeps from `from` to `to` between start and end.
    Chirp {
        from: f64,
        to: f64,
        sweep: Sweep,
        start: f64,
        end: Option<f64>,
    },
}

impl Default for Signal {
    fn default() -> Self {
        Signal::Impulse {
            time: 0.0,
            width: 1e-3,
        }
    }
}

/// How a chirp's frequency moves from start to end.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Sweep {
    /// By the same number of hertz per second.
    #[default]
    Linear,
    /// By the same factor per second: as long per octave at low as at high frequencies.
    Logarithmic,
}

impl Sweep {
    pub const ALL: [Sweep; 2] = [Sweep::Linear, Sweep::Logarithmic];

    pub fn label(self) -> &'static str {
        match self {
            Sweep::Linear => "Linear",
            Sweep::Logarithmic => "Logarithmic",
        }
    }
}

/// The kinds of signal, in the order the dialog offers them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SignalKind {
    Impulse,
    Sine,
    Chirp,
}

impl SignalKind {
    pub const ALL: [SignalKind; 3] = [SignalKind::Impulse, SignalKind::Sine, SignalKind::Chirp];

    pub fn label(self) -> &'static str {
        match self {
            SignalKind::Impulse => "Impulse",
            SignalKind::Sine => "Sine",
            SignalKind::Chirp => "Chirp",
        }
    }

    /// A signal of this kind with the settings of `from` that it shares (start, end).
    pub fn signal(self, from: &Signal) -> Signal {
        let start = from.start();
        let end = from.end();
        match self {
            SignalKind::Impulse => match from {
                Signal::Impulse { .. } => from.clone(),
                _ => Signal::Impulse {
                    time: start,
                    width: 1e-3,
                },
            },
            SignalKind::Sine => match from {
                Signal::Sine { .. } => from.clone(),
                _ => Signal::Sine {
                    frequency: 10.0,
                    phase: 0.0,
                    start,
                    end: if matches!(from, Signal::Impulse { .. }) {
                        None
                    } else {
                        end
                    },
                },
            },
            SignalKind::Chirp => match from {
                Signal::Chirp { .. } => from.clone(),
                _ => Signal::Chirp {
                    from: 1.0,
                    to: 100.0,
                    sweep: Sweep::Linear,
                    start,
                    end: if matches!(from, Signal::Impulse { .. }) {
                        None
                    } else {
                        end
                    },
                },
            },
        }
    }
}

impl Signal {
    pub fn kind(&self) -> SignalKind {
        match self {
            Signal::Impulse { .. } => SignalKind::Impulse,
            Signal::Sine { .. } => SignalKind::Sine,
            Signal::Chirp { .. } => SignalKind::Chirp,
        }
    }

    /// The step time the signal starts.
    pub fn start(&self) -> f64 {
        match *self {
            Signal::Impulse { time, .. } => time,
            Signal::Sine { start, .. } | Signal::Chirp { start, .. } => start,
        }
    }

    /// The end the user set; an impulse ends after its width.
    pub fn end(&self) -> Option<f64> {
        match *self {
            Signal::Impulse { time, width } => Some(time + width),
            Signal::Sine { end, .. } | Signal::Chirp { end, .. } => end,
        }
    }

    /// The signal the solver evaluates, in a step of length `duration`.
    pub fn to_ir(&self, duration: f64) -> IrSignal {
        match *self {
            Signal::Impulse { time, width } => IrSignal::Pulse { start: time, width },
            Signal::Sine {
                frequency,
                phase,
                start,
                end,
            } => IrSignal::Sine {
                start,
                end: end.unwrap_or(duration),
                frequency,
                phase: phase.to_radians(),
            },
            Signal::Chirp {
                from,
                to,
                sweep,
                start,
                end,
            } => IrSignal::Chirp {
                start,
                end: end.unwrap_or(duration),
                from,
                to,
                logarithmic: sweep == Sweep::Logarithmic,
            },
        }
    }

    /// The factor of the peak force at a step time, between -1 and 1.
    pub fn value_at(&self, time: f64, duration: f64) -> f64 {
        self.to_ir(duration).value(time)
    }

    /// The highest frequency the signal excites: the end of a sweep, the sine's frequency,
    /// for an impulse the first zero of its spectrum, `1.5 / width`.
    pub fn highest_frequency(&self) -> f64 {
        match *self {
            Signal::Impulse { width, .. } => 1.5 / width.max(f64::MIN_POSITIVE),
            Signal::Sine { frequency, .. } => frequency,
            Signal::Chirp { from, to, .. } => from.max(to),
        }
    }

    /// The length the automatic time step of a dynamic step takes a tenth of, like the
    /// shortest segment of an amplitude: the width of an impulse, half a period of the
    /// highest frequency of a sine or sweep (20 steps per period).
    pub fn feature_length(&self) -> f64 {
        match *self {
            Signal::Impulse { width, .. } => width,
            _ => 0.5 / self.highest_frequency().max(f64::MIN_POSITIVE),
        }
    }

    /// The impulse (force times time) the signal carries per unit of peak force: the area
    /// of the half sine, `2 width / pi`. Only an impulse has a net one.
    pub fn impulse_per_force(&self) -> Option<f64> {
        match *self {
            Signal::Impulse { width, .. } => Some(2.0 * width / std::f64::consts::PI),
            _ => None,
        }
    }

    /// Why the solver would reject the signal in a step of length `duration`, if it would.
    pub fn problem(&self, duration: f64) -> Option<String> {
        let numbers: Vec<f64> = match *self {
            Signal::Impulse { time, width } => vec![time, width],
            Signal::Sine {
                frequency,
                phase,
                start,
                end,
            } => [frequency, phase, start].into_iter().chain(end).collect(),
            Signal::Chirp {
                from,
                to,
                start,
                end,
                ..
            } => [from, to, start].into_iter().chain(end).collect(),
        };
        if numbers.iter().any(|v| !v.is_finite()) {
            return Some("The signal contains invalid numbers".into());
        }
        match *self {
            Signal::Impulse { width, .. } if width <= 0.0 => {
                return Some("The width of the impulse must be positive".into());
            }
            Signal::Sine { frequency, .. } if frequency <= 0.0 => {
                return Some("The frequency must be positive".into());
            }
            Signal::Chirp { from, to, .. } if from < 0.0 || to < 0.0 || from.max(to) <= 0.0 => {
                return Some("The frequencies must be positive".into());
            }
            Signal::Chirp {
                from,
                to,
                sweep: Sweep::Logarithmic,
                ..
            } if from <= 0.0 || to <= 0.0 => {
                return Some("A logarithmic sweep needs frequencies above zero".into());
            }
            _ => {}
        }
        let ir = self.to_ir(duration);
        if ir.end() <= ir.start() {
            return Some("The signal must end after it starts".into());
        }
        if self.start() >= duration {
            return Some("The signal starts after the end of the step".into());
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_open_end_runs_to_the_end_of_the_step() {
        let sine = Signal::Sine {
            frequency: 2.0,
            phase: 90.0,
            start: 0.0,
            end: None,
        };
        assert!((sine.value_at(0.0, 1.0) - 1.0).abs() < 1e-12);
        assert!((sine.value_at(1.0, 1.0) - 1.0).abs() < 1e-9);
        assert_eq!(sine.value_at(1.5, 1.0), 0.0);
        assert_eq!(sine.feature_length(), 0.25);
    }

    #[test]
    fn an_impulse_carries_the_area_of_its_half_sine() {
        let impulse = Signal::Impulse {
            time: 0.1,
            width: 0.01,
        };
        let n = 10_000;
        let h = 0.01 / n as f64;
        let area: f64 = (0..n)
            .map(|i| impulse.value_at(0.1 + (i as f64 + 0.5) * h, 1.0) * h)
            .sum();
        assert!((area - impulse.impulse_per_force().unwrap()).abs() < 1e-9);
        assert_eq!(impulse.feature_length(), 0.01);
        assert!((impulse.highest_frequency() - 150.0).abs() < 1e-9);
    }

    #[test]
    fn problems_are_named() {
        let duration = 1.0;
        assert_eq!(Signal::default().problem(duration), None);
        let zero = Signal::Impulse {
            time: 0.0,
            width: 0.0,
        };
        assert!(zero.problem(duration).unwrap().contains("width"));
        let log = Signal::Chirp {
            from: 0.0,
            to: 100.0,
            sweep: Sweep::Logarithmic,
            start: 0.0,
            end: None,
        };
        assert!(log.problem(duration).unwrap().contains("logarithmic"));
        let late = Signal::Sine {
            frequency: 1.0,
            phase: 0.0,
            start: 2.0,
            end: None,
        };
        assert!(late.problem(duration).is_some());
    }

    #[test]
    fn switching_the_kind_keeps_start_and_end() {
        let sine = Signal::Sine {
            frequency: 5.0,
            phase: 0.0,
            start: 0.2,
            end: Some(0.8),
        };
        let chirp = SignalKind::Chirp.signal(&sine);
        assert_eq!((chirp.start(), chirp.end()), (0.2, Some(0.8)));
        assert_eq!(SignalKind::Sine.signal(&sine), sine);
    }
}
