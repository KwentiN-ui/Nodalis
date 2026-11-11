//! The steps of results as the user defined them: the steps of the FE model with their
//! names, kinds and time periods, recorded when the analysis was solved ([`StepRecord`]),
//! and how the increments of the results fall into them ([`ResultSteps`]).
//!
//! The solver numbers its steps differently from the model: deactivated steps are left out,
//! a step with a moving load is solved as one step per position, and an analysis that
//! continues another one solves the steps it took over first. The records tie the solver's
//! steps back to the model's. Results without records, such as a `.frd` file of another
//! program, get one step per solver step, named by its number.

use serde::{Deserialize, Serialize};

use crate::{AnalysisKind, Increment};

/// The kind of a step, as far as the results show it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum StepCategory {
    Static,
    Visco,
    HeatTransfer,
    CoupledTempDisp,
    UncoupledTempDisp,
    Dynamic,
    SteadyStateDynamics,
    Frequency,
    ComplexFrequency,
    Buckle,
    Sensitivity,
    /// Results that do not tell.
    Unknown,
}

/// What a step computes, for its colour.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepFamily {
    /// Static and creep steps.
    Mechanical,
    /// Displacements over time with inertia, and harmonic responses.
    Dynamic,
    /// Temperatures, alone or with displacements.
    Thermal,
    /// Eigenmodes and buckling modes.
    Modal,
    Other,
}

impl StepCategory {
    pub fn of(kind: &plx_model::StepKind) -> Self {
        use plx_model::StepKind as K;
        match kind {
            K::Static(_) => Self::Static,
            K::Frequency(_) => Self::Frequency,
            K::ComplexFrequency(_) => Self::ComplexFrequency,
            K::Buckle(_) => Self::Buckle,
            K::HeatTransfer(_) => Self::HeatTransfer,
            K::CoupledTempDisp(_) => Self::CoupledTempDisp,
            K::UncoupledTempDisp(_) => Self::UncoupledTempDisp,
            K::Dynamic(_) => Self::Dynamic,
            K::Visco(_) => Self::Visco,
            K::SteadyStateDynamics(_) => Self::SteadyStateDynamics,
            K::Sensitivity(_) => Self::Sensitivity,
        }
    }

    /// The kind as the increments of results without records tell it.
    pub fn of_analysis(kind: AnalysisKind) -> Self {
        match kind {
            AnalysisKind::Static => Self::Static,
            AnalysisKind::Frequency => Self::Frequency,
            AnalysisKind::ComplexFrequency => Self::ComplexFrequency,
            AnalysisKind::Buckling => Self::Buckle,
            AnalysisKind::Dynamic => Self::Dynamic,
            AnalysisKind::Sensitivity => Self::Sensitivity,
            AnalysisKind::Other(_) => Self::Unknown,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Static => "Static",
            Self::Visco => "Visco",
            Self::HeatTransfer => "Heat transfer",
            Self::CoupledTempDisp => "Coupled temperature-displacement",
            Self::UncoupledTempDisp => "Uncoupled temperature-displacement",
            Self::Dynamic => "Dynamic",
            Self::SteadyStateDynamics => "Steady state dynamics",
            Self::Frequency => "Frequency",
            Self::ComplexFrequency => "Complex frequency",
            Self::Buckle => "Buckle",
            Self::Sensitivity => "Sensitivity",
            Self::Unknown => "Step",
        }
    }

    pub fn family(self) -> StepFamily {
        match self {
            Self::Static | Self::Visco => StepFamily::Mechanical,
            Self::Dynamic | Self::SteadyStateDynamics => StepFamily::Dynamic,
            Self::HeatTransfer | Self::CoupledTempDisp | Self::UncoupledTempDisp => {
                StepFamily::Thermal
            }
            Self::Frequency | Self::ComplexFrequency | Self::Buckle => StepFamily::Modal,
            Self::Sensitivity | Self::Unknown => StepFamily::Other,
        }
    }

    /// Whether the step runs over a time period, which its increments divide.
    pub fn is_timed(self) -> bool {
        matches!(
            self,
            Self::Static
                | Self::Visco
                | Self::HeatTransfer
                | Self::CoupledTempDisp
                | Self::UncoupledTempDisp
                | Self::Dynamic
                | Self::Unknown
        )
    }
}

/// A step of the FE model as it was solved.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StepRecord {
    pub name: String,
    pub category: StepCategory,
    /// Computed with large deformations.
    pub nlgeom: bool,
    /// A linear perturbation of the state of the step before it.
    pub perturbation: bool,
    /// Length of the step in time, for a step that runs over time.
    pub time_period: Option<f64>,
    /// The solver's steps it was solved as, numbered from 1: several for the positions of a
    /// moving load.
    pub solver_steps: Vec<u32>,
    /// The analysis the step was taken over from, for an analysis that continues another.
    pub inherited_from: Option<String>,
    /// For a step that leaves the state as it found it: the state it was computed about,
    /// "End of Preload" or "Unloaded model".
    pub base: Option<String>,
}

impl StepRecord {
    /// The record of the step `kind` of the model, solved as `solver_steps`.
    pub fn new(name: &str, kind: &plx_model::StepKind, solver_steps: Vec<u32>) -> Self {
        use plx_model::StepKind as K;
        let time_period = match kind {
            K::Static(s) => Some(s.duration),
            K::HeatTransfer(s) | K::CoupledTempDisp(s) | K::UncoupledTempDisp(s) => {
                Some(s.increments.duration)
            }
            K::Dynamic(s) => Some(s.duration),
            K::Visco(s) => Some(s.increments.duration),
            _ => None,
        };
        let perturbation = match kind {
            K::Frequency(_) | K::Buckle(_) => kind.base().is_some_and(|b| b.is_preloaded()),
            _ => false,
        };
        Self {
            name: name.to_owned(),
            category: StepCategory::of(kind),
            nlgeom: kind.nlgeom(),
            perturbation,
            time_period: time_period.filter(|t| t.is_finite() && *t > 0.0),
            solver_steps,
            inherited_from: None,
            base: None,
        }
    }
}

/// How the solver got through an increment.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct IncrementRecord {
    /// Solver step, from 1.
    pub step: u32,
    pub increment: u32,
    /// Attempts until it converged: 1 without a cutback.
    pub attempts: u32,
    /// Newton iterations of the attempt that converged.
    pub iterations: u32,
    pub step_time: f64,
    pub total_time: f64,
    /// Integrated explicitly (a step with automatic switching).
    #[serde(default)]
    pub explicit: bool,
}

/// A step of the results with the increments that belong to it.
#[derive(Clone, Debug, PartialEq)]
pub struct ResultStep {
    pub name: String,
    pub category: StepCategory,
    pub nlgeom: bool,
    pub perturbation: bool,
    pub time_period: Option<f64>,
    pub inherited_from: Option<String>,
    /// The state a probing step was computed about, see [`StepRecord::base`].
    pub base: Option<String>,
    pub solver_steps: Vec<u32>,
    /// Indices into the increments of the results, in order.
    pub increments: Vec<usize>,
    /// Total time at the start and at the end of the step; the same for a step that does
    /// not run over time.
    pub start_time: f64,
    pub end_time: f64,
    /// False for the step the solver stopped in.
    pub converged: bool,
    /// Whether the name is only the number, for results without records.
    pub numbered: bool,
    /// The spans of total time the solver integrated explicitly in a step that switches
    /// between implicit and explicit integration.
    pub explicit: Vec<(f64, f64)>,
}

impl ResultStep {
    /// Whether the step is solved once per position of a moving load.
    pub fn has_positions(&self) -> bool {
        self.solver_steps.len() > 1
    }

    /// What one entry of the step is called: an increment, a mode, a frequency, a response,
    /// or a position of a moving load.
    pub fn entry_noun(&self) -> &'static str {
        if self.has_positions() {
            return "Position";
        }
        match self.category {
            StepCategory::Frequency | StepCategory::ComplexFrequency | StepCategory::Buckle => {
                "Mode"
            }
            StepCategory::SteadyStateDynamics => "Frequency",
            StepCategory::Sensitivity => "Response",
            _ => "Increment",
        }
    }

    /// The kind with its options, e.g. "Static, NLGEOM" or "Frequency, about the end of
    /// Preload".
    pub fn description(&self) -> String {
        let mut text = self.category.label().to_owned();
        if self.nlgeom {
            text.push_str(", NLGEOM");
        }
        match &self.base {
            Some(base) => text.push_str(&format!(", about the {}", lower_first(base))),
            None if self.perturbation => text.push_str(", perturbation"),
            None => {}
        }
        if self.has_positions() {
            text.push_str(&format!(
                ", moving load in {} positions",
                self.solver_steps.len()
            ));
        }
        text
    }

    pub fn is_timed(&self) -> bool {
        self.category.is_timed()
    }
}

/// Where an increment lies in the steps.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct IncrementPlace {
    /// Index into [`ResultSteps::steps`].
    pub step: usize,
    /// Position in [`ResultStep::increments`].
    pub index: usize,
    /// Time since the start of the step and total time, for a step that runs over time.
    pub step_time: Option<f64>,
    pub total_time: Option<f64>,
    /// Attempts and iterations, when the solver reported them.
    pub attempts: Option<u32>,
    pub iterations: Option<u32>,
    /// Integrated explicitly (a step with automatic switching).
    pub explicit: bool,
}

/// The steps of results and where each increment lies in them.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ResultSteps {
    pub steps: Vec<ResultStep>,
    /// For each increment of the results.
    places: Vec<IncrementPlace>,
}

impl ResultSteps {
    /// The steps of `increments`: those of `records`, or one per solver step for increments
    /// no record covers. `convergence` gives the times and attempts the solver reported;
    /// `unconverged` is the solver step the run stopped in.
    pub fn new(
        increments: &[Increment],
        records: &[StepRecord],
        convergence: &[IncrementRecord],
        unconverged: Option<u32>,
    ) -> Self {
        // The record of each solver step.
        let record_of = |step: u32| records.iter().position(|r| r.solver_steps.contains(&step));
        // The steps in the order of their first solver step: the records, and a numbered
        // step for each solver step without one.
        enum Source {
            Record(usize),
            Number(u32),
        }
        let mut sources: Vec<(u32, Source)> = (records.iter().enumerate())
            .map(|(i, r)| {
                let first = r.solver_steps.iter().copied().min().unwrap_or(u32::MAX);
                (first, Source::Record(i))
            })
            .collect();
        let mut numbered: Vec<u32> = (increments.iter())
            .map(|i| i.step)
            .filter(|&s| record_of(s).is_none())
            .collect();
        numbered.dedup();
        numbered.sort_unstable();
        numbered.dedup();
        sources.extend(numbered.iter().map(|&s| (s, Source::Number(s))));
        sources.sort_by_key(|(first, _)| *first);

        let mut steps: Vec<ResultStep> = (sources.iter())
            .map(|(_, source)| match source {
                Source::Record(i) => {
                    let r = &records[*i];
                    ResultStep {
                        name: r.name.clone(),
                        category: r.category,
                        nlgeom: r.nlgeom,
                        perturbation: r.perturbation,
                        time_period: r.time_period,
                        inherited_from: r.inherited_from.clone(),
                        base: r.base.clone(),
                        solver_steps: r.solver_steps.clone(),
                        increments: Vec::new(),
                        start_time: 0.0,
                        end_time: 0.0,
                        converged: true,
                        numbered: false,
                        explicit: Vec::new(),
                    }
                }
                Source::Number(n) => {
                    let kind = (increments.iter()).find(|i| i.step == *n).map(|i| i.kind);
                    ResultStep {
                        name: format!("Step {n}"),
                        category: kind.map_or(StepCategory::Unknown, StepCategory::of_analysis),
                        nlgeom: false,
                        perturbation: false,
                        time_period: None,
                        inherited_from: None,
                        base: None,
                        solver_steps: vec![*n],
                        increments: Vec::new(),
                        start_time: 0.0,
                        end_time: 0.0,
                        converged: true,
                        numbered: true,
                        explicit: Vec::new(),
                    }
                }
            })
            .collect();
        let step_of = |steps: &[ResultStep], solver: u32| {
            steps.iter().position(|s| s.solver_steps.contains(&solver))
        };
        let owners: Vec<Option<usize>> =
            increments.iter().map(|i| step_of(&steps, i.step)).collect();
        for (index, owner) in owners.iter().enumerate() {
            if let Some(owner) = owner {
                steps[*owner].increments.push(index);
            }
        }
        if let Some(stopped) = unconverged
            && let Some(step) = step_of(&steps, stopped)
        {
            steps[step].converged = false;
        }

        // The times: the total time runs on through the steps that run over time.
        let reported = |inc: &Increment| {
            (convergence.iter()).find(|c| c.step == inc.step && c.increment == inc.increment)
        };
        let mut places = vec![IncrementPlace::default(); increments.len()];
        let mut clock = 0.0_f64;
        for (s, step) in steps.iter_mut().enumerate() {
            let timed = step.is_timed();
            // The start from what the solver reported, else where the step before ended.
            let start = (step.increments.first())
                .and_then(|&i| reported(&increments[i]))
                .map_or(clock, |c| c.total_time - c.step_time);
            let start = if start.is_finite() { start } else { clock };
            step.start_time = if timed { start } else { clock };
            let mut reached = step.start_time;
            // The end of the increment before, for the explicit spans.
            let mut before_time = step.start_time;
            // The increments give the total time, or the step time when one of them lies
            // before the start of the step.
            let step_times =
                (step.increments.iter()).any(|&i| before(increments[i].value, step.start_time));
            for (position, &i) in step.increments.iter().enumerate() {
                let inc = &increments[i];
                let report = reported(inc);
                let (step_time, total_time) = if !timed {
                    (None, None)
                } else if let Some(c) = report {
                    (Some(c.step_time), Some(c.total_time))
                } else if step_times {
                    (Some(inc.value), Some(step.start_time + inc.value))
                } else {
                    (Some(inc.value - step.start_time), Some(inc.value))
                };
                if let Some(total) = total_time {
                    reached = reached.max(total);
                }
                // Event frames between the increments have no report and do not break a
                // span.
                if let (Some(c), Some(total)) = (report, total_time) {
                    if c.explicit {
                        match step.explicit.last_mut() {
                            Some(span) if span.1 >= before_time => span.1 = total,
                            _ => step.explicit.push((before_time, total)),
                        }
                    }
                    before_time = total;
                }
                places[i] = IncrementPlace {
                    step: s,
                    index: position,
                    step_time,
                    total_time,
                    attempts: report.map(|c| c.attempts),
                    iterations: report.map(|c| c.iterations),
                    explicit: report.is_some_and(|c| c.explicit),
                };
            }
            step.end_time = match step.time_period {
                Some(period) if timed => step.start_time + period,
                _ => reached,
            };
            clock = clock.max(step.end_time);
        }
        Self { steps, places }
    }

    /// Where the increment `index` of the results lies; `None` for an index out of range.
    pub fn place(&self, index: usize) -> Option<&IncrementPlace> {
        self.places.get(index)
    }

    /// The step the increment `index` belongs to.
    pub fn step_of(&self, index: usize) -> Option<&ResultStep> {
        self.steps.get(self.place(index)?.step)
    }

    /// The step the solver step `solver` belongs to.
    pub fn of_solver_step(&self, solver: u32) -> Option<usize> {
        (self.steps.iter()).position(|s| s.solver_steps.contains(&solver))
    }

    /// Step times and total times of the time `values` of the solver step `solver`. Results
    /// give either: the built-in solver's history output the step time, its increments and
    /// CalculiX the total time; values before the start of the step are step times. `None`
    /// for a step that does not run over time.
    pub fn times_of(&self, solver: u32, values: &[f64]) -> Option<Vec<(f64, f64)>> {
        let step = self.steps.get(self.of_solver_step(solver)?)?;
        if !step.is_timed() {
            return None;
        }
        let start = step.start_time;
        let step_times = values.iter().any(|&v| before(v, start));
        Some(
            (values.iter())
                .map(|&v| {
                    if step_times {
                        (v, start + v)
                    } else {
                        (v - start, v)
                    }
                })
                .collect(),
        )
    }

    /// The increment shown when step `step` is chosen: its last one, the state at its end.
    pub fn entry_increment(&self, step: usize) -> Option<usize> {
        self.steps.get(step)?.increments.last().copied()
    }
}

/// Whether the time `value` lies before `start`, beyond rounding.
fn before(value: f64, start: f64) -> bool {
    value < start - 1e-9 * start.abs().max(1.0)
}

/// `text` with a lower-case first letter: "End of Preload" in a sentence.
fn lower_first(text: &str) -> String {
    let mut chars = text.chars();
    match chars.next() {
        Some(first) => first.to_lowercase().chain(chars).collect(),
        None => String::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inc(step: u32, increment: u32, kind: AnalysisKind, value: f64) -> Increment {
        Increment {
            step,
            increment,
            kind,
            value,
            fields: Vec::new(),
            nodal_diameter: None,
        }
    }

    fn record(
        name: &str,
        category: StepCategory,
        period: Option<f64>,
        steps: &[u32],
    ) -> StepRecord {
        StepRecord {
            name: name.into(),
            category,
            nlgeom: false,
            perturbation: false,
            time_period: period,
            solver_steps: steps.to_vec(),
            inherited_from: None,
            base: None,
        }
    }

    #[test]
    fn results_without_records_get_a_step_per_solver_step() {
        let increments = [
            inc(1, 1, AnalysisKind::Static, 0.5),
            inc(1, 2, AnalysisKind::Static, 1.0),
            inc(2, 1, AnalysisKind::Static, 1.25),
            inc(2, 2, AnalysisKind::Static, 2.0),
            inc(3, 1, AnalysisKind::Frequency, 112.0),
        ];
        let steps = ResultSteps::new(&increments, &[], &[], None);
        let names: Vec<&str> = steps.steps.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, ["Step 1", "Step 2", "Step 3"]);
        assert_eq!(steps.steps[1].increments, [2, 3]);
        assert_eq!(steps.steps[2].category, StepCategory::Frequency);
        assert_eq!(steps.steps[2].entry_noun(), "Mode");
        // The second step starts where the first ended.
        let place = steps.place(2).unwrap();
        assert_eq!((place.step, place.index), (1, 0));
        assert_eq!(place.step_time, Some(0.25));
        assert_eq!(place.total_time, Some(1.25));
        assert_eq!(steps.steps[1].start_time, 1.0);
        assert_eq!(steps.steps[2].start_time, 2.0);
        assert_eq!(steps.place(4).unwrap().total_time, None);
    }

    #[test]
    fn records_name_the_steps_and_join_the_positions_of_a_moving_load() {
        let increments = [
            inc(1, 1, AnalysisKind::Static, 1.0),
            inc(2, 1, AnalysisKind::Static, 2.0),
            inc(3, 1, AnalysisKind::Static, 3.0),
            inc(4, 1, AnalysisKind::Static, 4.0),
        ];
        let records = [
            record("Mount", StepCategory::Static, Some(1.0), &[1]),
            record("Crane travel", StepCategory::Static, Some(1.0), &[2, 3, 4]),
        ];
        let steps = ResultSteps::new(&increments, &records, &[], None);
        assert_eq!(steps.steps.len(), 2);
        let travel = &steps.steps[1];
        assert_eq!(travel.name, "Crane travel");
        assert_eq!(travel.increments, [1, 2, 3]);
        assert!(travel.has_positions());
        assert_eq!(travel.entry_noun(), "Position");
        assert_eq!(steps.step_of(3).unwrap().name, "Crane travel");
        assert_eq!(steps.entry_increment(0), Some(0));
        assert_eq!(steps.entry_increment(1), Some(3));
    }

    #[test]
    fn a_step_without_increments_is_kept_and_the_stopped_step_is_marked() {
        let increments = [
            inc(1, 1, AnalysisKind::Static, 1.0),
            inc(2, 1, AnalysisKind::Static, 1.3),
        ];
        let records = [
            record("Preload", StepCategory::Static, Some(1.0), &[1]),
            record("Overload", StepCategory::Static, Some(1.0), &[2]),
            record("Unload", StepCategory::Static, Some(1.0), &[3]),
        ];
        let steps = ResultSteps::new(&increments, &records, &[], Some(2));
        assert!(steps.steps[0].converged);
        assert!(!steps.steps[1].converged);
        // The time period gives the end, also of the step that stopped early.
        assert_eq!(steps.steps[1].end_time, 2.0);
        assert!(steps.steps[2].increments.is_empty());
    }

    #[test]
    fn reported_times_and_attempts_are_taken() {
        let increments = [
            inc(1, 1, AnalysisKind::Dynamic, 0.1),
            inc(2, 3, AnalysisKind::Frequency, 300.0),
        ];
        let convergence = [IncrementRecord {
            step: 1,
            increment: 1,
            attempts: 2,
            iterations: 5,
            step_time: 0.1,
            total_time: 4.1,
            explicit: false,
        }];
        let records = [
            record("Shake", StepCategory::Dynamic, Some(0.5), &[1]),
            record("Modes", StepCategory::Frequency, None, &[2]),
        ];
        let steps = ResultSteps::new(&increments, &records, &convergence, None);
        let place = steps.place(0).unwrap();
        let near = |a: f64, b: f64| (a - b).abs() < 1e-12;
        assert!(near(steps.steps[0].start_time, 4.0));
        assert_eq!((place.attempts, place.iterations), (Some(2), Some(5)));
        assert!(near(steps.steps[0].end_time, 4.5));
        // A modal step takes no time.
        assert!(near(steps.steps[1].start_time, 4.5));
        assert!(near(steps.steps[1].end_time, 4.5));
    }

    /// A step with automatic switching: the increments the solver reported as explicit
    /// make spans of total time, from the increment before; an event frame without a report
    /// between them does not break a span.
    #[test]
    fn explicit_increments_make_spans_of_the_step() {
        let increments: Vec<Increment> = [0.1, 0.2, 0.25, 0.3, 0.4, 0.5]
            .iter()
            .enumerate()
            .map(|(k, t)| inc(1, k as u32 + 1, AnalysisKind::Dynamic, *t))
            .collect();
        let explicit = [false, true, false, true, false, true];
        let convergence: Vec<IncrementRecord> = increments
            .iter()
            .zip(explicit)
            // Increment 3 (0.25) is an event frame without a report.
            .filter(|(i, _)| i.increment != 3)
            .map(|(i, explicit)| IncrementRecord {
                step: 1,
                increment: i.increment,
                attempts: 1,
                iterations: 1,
                step_time: i.value,
                total_time: i.value,
                explicit,
            })
            .collect();
        let records = [record("Drop", StepCategory::Dynamic, Some(0.5), &[1])];
        let steps = ResultSteps::new(&increments, &records, &convergence, None);
        let spans = &steps.steps[0].explicit;
        assert_eq!(spans.len(), 2, "{spans:?}");
        let near = |a: f64, b: f64| (a - b).abs() < 1e-12;
        assert!(near(spans[0].0, 0.1) && near(spans[0].1, 0.3), "{spans:?}");
        assert!(near(spans[1].0, 0.4) && near(spans[1].1, 0.5), "{spans:?}");
        assert!(steps.place(1).unwrap().explicit);
        assert!(!steps.place(4).unwrap().explicit);
    }

    #[test]
    fn a_frequency_step_after_a_perturbed_preload_describes_itself() {
        let mut modes = record("Modes", StepCategory::Frequency, None, &[2]);
        modes.perturbation = true;
        let increments = [inc(2, 1, AnalysisKind::Frequency, 10.0)];
        let steps = ResultSteps::new(&increments, &[modes.clone()], &[], None);
        assert_eq!(steps.steps[0].description(), "Frequency, perturbation");
        // The record names its base.
        modes.base = Some("End of Preload".into());
        let steps = ResultSteps::new(&increments, &[modes], &[], None);
        assert_eq!(
            steps.steps[0].description(),
            "Frequency, about the end of Preload"
        );
    }
}
