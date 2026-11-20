//! The settings of a step that decide how its solution runs, in words: how long the step
//! lasts, how many result frames and increments it takes, whether it is geometrically
//! nonlinear, how many modes it computes. The model tree shows them as a short line under
//! the step, the details on hover.

use crate::{
    Damping, DynamicMethod, FeModel, HeatTransferStep, IncrementSize, ModalDamping, StaticStep,
    Step, StepKind, TimeStep,
};

/// The solution settings of a step: a line short enough for the model tree, and one line
/// per setting for its tooltip.
#[derive(Clone, Debug, PartialEq)]
pub struct StepSettings {
    /// "Nlgeom, 1 s, 10 frames, Δt 0.1 s".
    pub line: String,
    /// "Duration: 1 s", "Result frames: 10", ...
    pub details: Vec<String>,
}

impl FeModel {
    /// What the step at `index` is solved with, see [`StepSettings`].
    pub fn step_settings(&self, index: usize) -> StepSettings {
        self.settings_of(&self.steps[index])
    }

    /// What `step` is solved with, see [`StepSettings`].
    pub fn settings_of(&self, step: &Step) -> StepSettings {
        let nlgeom = self.step_nlgeom(&step.kind);
        // Large deformations the materials switch on, not the step's own setting.
        let by_materials = nlgeom && !step.kind.nlgeom();
        let mut s = Summary::default();
        match &step.kind {
            StepKind::Static(settings) => {
                s.detail("Static".into());
                s.increments(settings, nlgeom, by_materials);
            }
            StepKind::Visco(settings) => {
                s.detail(format!(
                    "Visco, creep tolerance {}",
                    number(settings.creep_tolerance)
                ));
                s.increments(&settings.increments, nlgeom, by_materials);
            }
            StepKind::HeatTransfer(settings) => s.heat("Heat transfer", settings, nlgeom),
            StepKind::CoupledTempDisp(settings) => {
                s.heat("Coupled temperature-displacement", settings, nlgeom)
            }
            StepKind::UncoupledTempDisp(settings) => {
                s.heat("Uncoupled temperature-displacement", settings, nlgeom)
            }
            StepKind::Dynamic(settings) => {
                let (short, method) = match settings.method {
                    DynamicMethod::Direct => ("Implicit", settings.method.label().to_string()),
                    DynamicMethod::Modal { modes, .. } => (
                        "Modal",
                        format!("{}, {modes} modes", settings.method.label()),
                    ),
                    DynamicMethod::Explicit => ("Explicit", settings.method.label().into()),
                    DynamicMethod::Automatic => {
                        ("Implicit/explicit", settings.method.label().into())
                    }
                };
                s.nlgeom(nlgeom, by_materials);
                if matches!(settings.method, DynamicMethod::Direct) {
                    s.detail(format!("Dynamic, {method}"));
                } else {
                    s.both(short.into(), format!("Dynamic, {method}"));
                }
                if let DynamicMethod::Modal { modes, .. } = settings.method {
                    s.line.push(format!("{modes} modes"));
                }
                s.time(settings.duration, settings.frames);
                if settings.method.uses_explicit() {
                    s.both(
                        "Δt from mesh".into(),
                        "Time step: below the stability limit of the mesh".into(),
                    );
                    if let Some(factor) = settings.mass_scaling {
                        s.both(
                            "mass scaling".into(),
                            format!("Mass scaling to a time step of {} s", number(factor)),
                        );
                    }
                } else {
                    let interval = self.amplitude_interval(step, settings.duration);
                    let integration = settings.integration(interval);
                    let count = integration.increments(settings);
                    let dt = number(integration.increment);
                    s.both(
                        format!("{count} × {dt} s"),
                        format!(
                            "Time steps: {count} of {dt} s{}",
                            match settings.time_step {
                                TimeStep::Fixed(_) => " (fixed)",
                                TimeStep::Automatic => " (automatic)",
                            }
                        ),
                    );
                }
                match settings.damping {
                    Damping::None => s.detail("Damping: none".into()),
                    Damping::Ratio { zeta, .. } => {
                        let percent = number(zeta * 100.0);
                        s.both(
                            format!("ζ {percent} %"),
                            format!("Damping ratio: {percent} %"),
                        );
                    }
                    Damping::Rayleigh(r) => s.both(
                        "Rayleigh".into(),
                        format!(
                            "Rayleigh damping: alpha {}, beta {}",
                            number(r.alpha),
                            number(r.beta)
                        ),
                    ),
                }
            }
            StepKind::Frequency(settings) => {
                s.detail("Frequency".into());
                let modes = settings.num_frequencies;
                s.both(format!("{modes} modes"), format!("Eigenmodes: {modes}"));
                let range = match (settings.lower_frequency, settings.upper_frequency) {
                    (Some(low), Some(high)) => {
                        Some(format!("{} to {} Hz", number(low), number(high)))
                    }
                    (Some(low), None) => Some(format!("above {} Hz", number(low))),
                    (None, Some(high)) => Some(format!("up to {} Hz", number(high))),
                    (None, None) => None,
                };
                if let Some(range) = range {
                    s.both(range.clone(), format!("Frequencies: {range}"));
                }
                if let Some([low, high]) = settings.nodal_diameters {
                    s.both(
                        format!("ND {low} to {high}"),
                        format!("Nodal diameters: {low} to {high}"),
                    );
                }
            }
            StepKind::ComplexFrequency(settings) => {
                s.detail("Complex frequency".into());
                let modes = settings.num_frequencies;
                s.both(
                    format!("{modes} modes"),
                    format!(
                        "Complex eigenmodes: {modes}, from {} eigenmodes",
                        settings.num_modes
                    ),
                );
                let speed = number(settings.speed);
                s.both(format!("{speed} rad/s"), format!("Speed: {speed} rad/s"));
                if settings.coriolis {
                    s.both("Coriolis".into(), "Coriolis forces: on".into());
                }
            }
            StepKind::Buckle(settings) => {
                s.detail("Buckle".into());
                let factors = settings.num_factors;
                s.both(
                    format!(
                        "{factors} buckling factor{}",
                        if factors == 1 { "" } else { "s" }
                    ),
                    format!("Buckling factors: {factors}"),
                );
            }
            StepKind::SteadyStateDynamics(settings) => {
                let kind = if settings.harmonic {
                    "Harmonic"
                } else {
                    "Periodic"
                };
                s.both(kind.into(), format!("Steady state dynamics, {kind}"));
                let range = format!(
                    "{} to {} Hz",
                    number(settings.lower_frequency),
                    number(settings.upper_frequency)
                );
                s.both(range.clone(), format!("Frequencies: {range}"));
                let points = settings.data_points;
                s.both(
                    format!("{points} points"),
                    format!("Points per interval: {points}"),
                );
                let modes = settings.modes;
                s.both(format!("{modes} modes"), format!("Eigenmodes: {modes}"));
                match &settings.damping {
                    Some(ModalDamping::Constant(zeta)) => {
                        let percent = number(zeta * 100.0);
                        s.both(
                            format!("ζ {percent} %"),
                            format!("Damping ratio: {percent} %"),
                        );
                    }
                    Some(ModalDamping::Direct(_)) => {
                        s.detail("Damping: ratios per mode range".into())
                    }
                    Some(ModalDamping::Rayleigh(_)) => s.detail("Damping: Rayleigh".into()),
                    None => s.detail("Damping: none".into()),
                }
            }
            StepKind::Sensitivity(settings) => {
                s.detail("Sensitivity".into());
                let of: Vec<&str> = [
                    (settings.mass, "mass"),
                    (settings.strain_energy, "strain energy"),
                    (settings.eigenfrequency, "eigenfrequencies"),
                ]
                .into_iter()
                .filter_map(|(on, name)| on.then_some(name))
                .collect();
                let of = if of.is_empty() {
                    "nothing".to_string()
                } else {
                    of.join(", ")
                };
                s.both(format!("Of {of}"), format!("Sensitivities of: {of}"));
            }
            StepKind::ParticleFlow(settings) => {
                s.detail("Particle flow".into());
                s.time(settings.duration, settings.frames);
                if settings.heat {
                    s.both("heat".into(), "Temperature: computed".into());
                }
            }
        }
        s.details.append(&mut s.last);
        StepSettings {
            line: s.line.join(", "),
            details: s.details,
        }
    }
}

#[derive(Default)]
struct Summary {
    line: Vec<String>,
    details: Vec<String>,
    /// Details listed after all others, such as the large deformations, which lead the line.
    last: Vec<String>,
}

impl Summary {
    fn detail(&mut self, detail: String) {
        self.details.push(detail);
    }

    fn both(&mut self, short: String, detail: String) {
        self.line.push(short);
        self.details.push(detail);
    }

    fn time(&mut self, duration: f64, frames: u32) {
        let duration = number(duration);
        self.both(format!("{duration} s"), format!("Duration: {duration} s"));
        let frames = frames.max(1);
        self.both(
            format!("{frames} frame{}", if frames == 1 { "" } else { "s" }),
            format!("Result frames: {frames}"),
        );
    }

    fn nlgeom(&mut self, nlgeom: bool, by_materials: bool) {
        if nlgeom {
            self.line.push("Nlgeom".into());
            self.last.push(if by_materials {
                "Large deformations: on (hyperelastic materials)".into()
            } else {
                "Large deformations: on".into()
            });
        } else {
            self.last.push("Large deformations: off".into());
        }
    }

    /// Duration, frames and increments of a static, visco or heat transfer step.
    fn increments(&mut self, settings: &StaticStep, nlgeom: bool, by_materials: bool) {
        // First, so that a narrow tree does not cut it off.
        self.nlgeom(nlgeom, by_materials);
        self.time(settings.duration, settings.frames);
        let plan = settings.plan(nlgeom);
        let initial = number(plan.initial);
        match settings.control.size {
            IncrementSize::Fixed(_) => {
                // Shortened to end on the frames: at least one per frame.
                let per_frame = (settings.frame_interval() / plan.initial - 1e-9)
                    .ceil()
                    .max(1.0);
                let count = (per_frame * f64::from(plan.frames)).min(u32::MAX as f64) as u64;
                self.both(
                    format!("{count} × {initial} s"),
                    format!("Increments: {count} fixed of {initial} s"),
                );
            }
            IncrementSize::Automatic => self.both(
                format!("Δt {initial} s"),
                format!(
                    "Increments: automatic, first {initial} s, between {} s and {} s",
                    number(plan.minimum),
                    number(plan.maximum)
                ),
            ),
        }
        if let Some(limit) = plan.limit {
            self.both(
                format!("max {limit} inc."),
                format!("Increment limit: {limit}"),
            );
        }
    }

    fn heat(&mut self, name: &str, settings: &HeatTransferStep, nlgeom: bool) {
        let state = if settings.steady_state {
            "Steady state"
        } else {
            "Transient"
        };
        self.both(state.into(), format!("{name}, {}", state.to_lowercase()));
        self.increments(&settings.increments, nlgeom, false);
        if let Some(deltmx) = settings.deltmx {
            self.detail(format!(
                "Largest temperature change per increment: {}",
                number(deltmx)
            ));
        }
    }
}

/// A value with at most four significant digits: 0.1, 2.5e-4, 1200.
fn number(value: f64) -> String {
    if value == 0.0 || !value.is_finite() {
        return format!("{value}");
    }
    let magnitude = value.abs();
    if (1e-3..1e6).contains(&magnitude) {
        let decimals = (3 - magnitude.log10().floor() as i32).max(0) as usize;
        let text = format!("{value:.decimals$}");
        if text.contains('.') {
            text.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            text
        }
    } else {
        let text = format!("{value:.3e}");
        let (mantissa, exponent) = text.split_once('e').unwrap_or((&text, "0"));
        let mantissa = mantissa.trim_end_matches('0').trim_end_matches('.');
        format!("{mantissa}e{exponent}")
    }
}

#[cfg(test)]
mod tests {
    use super::number;
    use crate::{FeModel, IncrementSize, StaticStep, Step, StepKind};

    fn model(kind: StepKind) -> FeModel {
        let mut model = FeModel::default();
        let mut step = Step::new_static("Step-1");
        step.kind = kind;
        model.steps.push(step);
        model
    }

    #[test]
    fn numbers_are_short() {
        assert_eq!(number(0.1), "0.1");
        assert_eq!(number(1.0), "1");
        assert_eq!(number(1e-5), "1e-5");
        assert_eq!(number(2.5e-4), "2.5e-4");
        assert_eq!(number(1000.0), "1000");
        assert_eq!(number(0.333_333_3), "0.3333");
        assert_eq!(number(123.456), "123.5");
    }

    #[test]
    fn a_static_step_shows_duration_frames_and_increments() {
        let step = StaticStep {
            duration: 1.0,
            frames: 10,
            nlgeom: true,
            ..StaticStep::default()
        };
        let settings = model(StepKind::Static(step.clone())).step_settings(0);
        assert_eq!(settings.line, "Nlgeom, 1 s, 10 frames, Δt 0.1 s");
        assert!(
            settings
                .details
                .contains(&"Large deformations: on".to_string())
        );

        let mut fixed = step;
        fixed.nlgeom = false;
        fixed.control.size = IncrementSize::Fixed(0.05);
        fixed.control.limit = Some(500);
        let settings = model(StepKind::Static(fixed)).step_settings(0);
        assert_eq!(settings.line, "1 s, 10 frames, 20 × 0.05 s, max 500 inc.");
    }
}
