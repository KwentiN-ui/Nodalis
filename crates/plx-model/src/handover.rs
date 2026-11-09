//! What a step takes over from the steps before it and what it gives to the steps after it,
//! in words. This is the one place the step dialog, the model tree and the step overview
//! read it from, so that every step says the same about the same chain.
//!
//! The steps of an analysis are a chain. A *continuing* step (static, visco, dynamic, heat
//! transfer) starts from the state the steps before it leave and passes its own end state
//! on. A *probing* step (frequency, buckle, complex frequency, steady state dynamics,
//! sensitivity) reads a state, the unloaded model or that of the steps before it (its
//! [`Base`]), and leaves the state as it found it: the step after it starts where the
//! steps before the probing step ended.

use crate::{
    Base, Constraint, DynamicMethod, FeModel, InitialConditionKind, LoadKind, RemoteBehavior,
    StepKind,
};

impl StepKind {
    /// Whether the step passes its end state on to the step after it. A probing step
    /// (eigenmodes, buckling factors, harmonic response, sensitivities) leaves the state
    /// as it found it.
    pub fn continues_state(&self) -> bool {
        matches!(
            self,
            StepKind::Static(_)
                | StepKind::Visco(_)
                | StepKind::Dynamic(_)
                | StepKind::HeatTransfer(_)
                | StepKind::CoupledTempDisp(_)
                | StepKind::UncoupledTempDisp(_)
        )
    }

    /// What the step (or the eigenmodes it superposes) is computed about, where the user
    /// chooses it: frequency and buckle steps, and the modes of a modal dynamic step.
    pub fn base(&self) -> Option<Base> {
        match self {
            StepKind::Frequency(settings) => Some(settings.base),
            StepKind::Buckle(settings) => Some(settings.base),
            StepKind::ComplexFrequency(settings) => Some(settings.base),
            StepKind::SteadyStateDynamics(settings) => Some(settings.base),
            StepKind::Dynamic(settings) => match settings.method {
                DynamicMethod::Modal { base, .. } => Some(base),
                _ => None,
            },
            _ => None,
        }
    }

    pub fn base_mut(&mut self) -> Option<&mut Base> {
        match self {
            StepKind::Frequency(settings) => Some(&mut settings.base),
            StepKind::Buckle(settings) => Some(&mut settings.base),
            StepKind::ComplexFrequency(settings) => Some(&mut settings.base),
            StepKind::SteadyStateDynamics(settings) => Some(&mut settings.base),
            StepKind::Dynamic(settings) => match &mut settings.method {
                DynamicMethod::Modal { base, .. } => Some(base),
                _ => None,
            },
            _ => None,
        }
    }

    /// Whether the step leaves eigenmodes behind that a later sensitivity step reads: a
    /// frequency step that stores them.
    pub fn stores_modes(&self) -> bool {
        matches!(self, StepKind::Frequency(settings) if settings.storage)
    }
}

/// How a step stands in the chain of steps of its analysis.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Handover {
    /// The active step whose end state the step starts from; `None` for the first one.
    pub previous: Option<usize>,
    /// "Unloaded model", "Initial conditions (Spin)", "End of Preload (explicit)".
    pub starts_from: String,
    /// What carries over from the steps before: "displacements, stresses, ...".
    pub carried: String,
    /// Deactivated steps between the previous one and this one, which are left out.
    pub skipped: Vec<String>,
    /// What a probing step, or the modes of a modal dynamic step, are computed about, with
    /// the words for the choice: ("End of Preload", "stiffening by its stresses").
    pub base: Option<Base>,
    /// The name of the choice [`Base::PreviousState`] in this step: "End of Preload", or
    /// `None` when no step before leaves a state.
    pub previous_state: Option<String>,
    /// What other steps produced that the step reads: "Eigenmodes of Freq-1".
    pub uses: Vec<String>,
    /// How the loads start: "Loads change linearly from their values at the end of Preload".
    pub loads: Option<String>,
    /// Loads of the previous step the step does not have, which ramp down to zero.
    pub removed_loads: Vec<String>,
    /// Temperatures a step before was given and this one is not, which go back to the
    /// initial temperatures: "Temperatures of Heat-up go back to the initial ones".
    pub temperatures: Option<String>,
    /// What holds in every step: constraints, contact, bolts, the couplings of remote
    /// forces of other steps.
    pub model_wide: Vec<String>,
    /// What the next active step gets: "Its end state to Unload", "Nothing: Unload starts
    /// from the end of Preload".
    pub passes_on: String,
}

impl Handover {
    /// The first line the model tree shows under the step.
    pub fn summary(&self) -> String {
        match self.base {
            Some(Base::Unloaded) if self.previous.is_none() || !self.continues() => {
                "Base: Unloaded model".into()
            }
            Some(Base::PreviousState) if !self.continues() => match &self.previous_state {
                Some(state) => format!("Base: {state}"),
                None => "Base: Unloaded model".into(),
            },
            _ => format!("Starts from: {}", self.starts_from),
        }
    }

    fn continues(&self) -> bool {
        self.passes_on.starts_with("Its end state")
    }
}

/// The ending of a step's name with its integration, so that a dynamic step names how its
/// state was found.
fn method_suffix(kind: &StepKind) -> &'static str {
    match kind {
        StepKind::Dynamic(d) if d.method.is_explicit() => " (explicit)",
        StepKind::Dynamic(d) if matches!(d.method, DynamicMethod::Automatic) => " (automatic)",
        StepKind::Dynamic(d) if matches!(d.method, DynamicMethod::Direct) => " (implicit)",
        StepKind::Dynamic(_) => " (modal)",
        _ => "",
    }
}

impl FeModel {
    /// What the step at `index` takes over and passes on.
    pub fn step_handover(&self, index: usize) -> Handover {
        self.handover(index, &self.steps[index])
    }

    /// What `step` takes over and passes on at `index`, the step being edited there or a
    /// new one at `steps.len()`.
    pub fn handover(&self, index: usize, step: &crate::Step) -> Handover {
        let kind = &step.kind;
        let index = index.min(self.steps.len());
        let before = &self.steps[..index];
        let after = self.steps.get(index + 1..).unwrap_or(&[]);
        let previous = (0..index)
            .rev()
            .find(|&s| before[s].active && before[s].kind.continues_state());
        let skipped: Vec<String> = (previous.map_or(0, |p| p + 1)..index)
            .filter(|&s| !before[s].active)
            .map(|s| before[s].name.clone())
            .collect();
        let previous_name = previous.map(|p| {
            let step = &before[p];
            format!("End of {}{}", step.name, method_suffix(&step.kind))
        });

        // The initial conditions that act at the start: temperatures always, velocities in
        // the first dynamic step.
        let dynamic_before =
            (before.iter()).any(|s| s.active && matches!(s.kind, StepKind::Dynamic(_)));
        let conditions: Vec<&str> = (self.initial_conditions.iter())
            .filter(|c| c.active)
            .filter(|c| match c.kind {
                InitialConditionKind::Temperature(_) => previous.is_none(),
                _ => matches!(kind, StepKind::Dynamic(_)) && !dynamic_before,
            })
            .map(|c| c.name.as_str())
            .collect();
        let probing = !kind.continues_state();
        // Eigenmodes linked from another analysis were computed there, about its state.
        let linked_modes = matches!(kind, StepKind::SteadyStateDynamics(_))
            && self.properties.stored_modes.is_some();
        let base = kind.base().filter(|_| !linked_modes);
        let starts_unloaded = previous.is_none() || (probing && base == Some(Base::Unloaded));
        let mut starts_from = if starts_unloaded {
            "Unloaded model".to_owned()
        } else {
            previous_name.clone().unwrap_or_default()
        };
        if !conditions.is_empty() {
            let with = format!("initial conditions {}", conditions.join(", "));
            starts_from = if previous.is_none() {
                format!("Unloaded model with {with}")
            } else {
                format!("{starts_from} with {with}")
            };
        }

        let carried = if starts_unloaded {
            String::new()
        } else {
            let mut parts: Vec<String> = Vec::new();
            let previous_step = &before[previous.unwrap_or(0)];
            if previous_step.kind.is_mechanical() {
                parts.push("displacements, stresses, plastic and creep strains".into());
                if !self.contact_regions.is_empty() || !self.contact_pairs.is_empty() {
                    parts.push("contact status".into());
                }
                if matches!(previous_step.kind, StepKind::Dynamic(_)) {
                    parts.push("velocities".into());
                }
            }
            // Temperatures: those a heat transfer step solved for, or those a mechanical
            // step was given.
            let last_thermal = (before.iter().take(previous.unwrap_or(0) + 1).rev())
                .filter(|s| s.active)
                .find(|s| s.kind.is_thermal() || s.defined_fields.iter().any(|f| f.active));
            if let Some(step) = last_thermal {
                parts.push(format!("temperatures of {}", step.name));
            }
            parts.join(", ")
        };

        // Removed loads ramp down to zero in a static or visco step after one: the solver
        // moves every load from its value at the end of the previous step to the value of
        // this step.
        let mut loads = None;
        let mut removed_loads = Vec::new();
        if let Some(p) = previous
            && matches!(kind, StepKind::Static(_) | StepKind::Visco(_))
            && matches!(before[p].kind, StepKind::Static(_) | StepKind::Visco(_))
        {
            let previous_step = &before[p];
            let acting =
                |load: &crate::Load| load.active && !matches!(load.kind, LoadKind::Moving(_));
            if previous_step.loads.iter().any(acting) {
                loads = Some(format!(
                    "Loads change linearly from their values at the end of {}",
                    previous_step.name
                ));
                removed_loads = (previous_step.loads.iter())
                    .filter(|l| acting(l))
                    .filter(|l| !step.loads.iter().any(|o| o.active && o.id == l.id))
                    .map(|l| l.name.clone())
                    .collect();
            }
        }

        let temperatures = self.temperature_source(index, step).map(|s| {
            format!(
                "Temperatures of {} go back to the initial ones",
                self.steps[s].name
            )
        });

        let mut uses = Vec::new();
        match kind {
            StepKind::SteadyStateDynamics(settings) => {
                uses.push(match &self.properties.stored_modes {
                    Some(file) => format!("Eigenmodes linked from {}", file.display()),
                    None => format!("Its own {} eigenmodes", settings.modes),
                });
            }
            StepKind::Sensitivity(_) => match (0..index).rev().find(|&s| before[s].active) {
                Some(s) => uses.push(format!("Solution of {}", before[s].name)),
                None => uses.push("Solution of the step before it (none)".into()),
            },
            StepKind::ComplexFrequency(settings) => {
                uses.push(if settings.stiffening {
                    format!(
                        "Its own {} eigenmodes, preloaded by the rotation",
                        settings.num_modes
                    )
                } else {
                    format!("Its own {} eigenmodes", settings.num_modes)
                });
            }
            _ => {}
        }

        let mut model_wide = Vec::new();
        let constraints = (self.constraints.iter())
            .filter(|c| constraint_active(c))
            .count()
            + self.ties.len()
            + self.node_ties.len();
        if constraints > 0 {
            model_wide.push(plural(constraints, "constraint"));
        }
        let contact = self.contact_regions.len() + self.contact_pairs.len();
        if contact > 0 {
            model_wide.push(plural(contact, "contact region"));
        }
        if !self.bolts.is_empty() {
            model_wide.push(plural(self.bolts.len(), "bolt"));
        }
        // The rigid point of a remote force keeps its selection rigid in every step.
        for (s, i, _, remote) in self.remote_forces() {
            if s != index && remote.behavior == RemoteBehavior::Rigid {
                let step = &self.steps[s];
                model_wide.push(format!(
                    "rigid coupling of {} ({})",
                    step.loads[i].name, step.name
                ));
            }
        }

        let next = (after.iter()).find(|s| s.active).map(|s| s.name.as_str());
        let passes_on = match (kind.continues_state(), next) {
            (true, Some(next)) => format!("Its end state to {next}"),
            (true, None) => "Its end state (no step follows)".into(),
            (false, Some(next)) => match &previous_name {
                Some(state) => format!("Nothing: {next} starts from the {}", lower_first(state)),
                None => format!("Nothing: {next} starts from the unloaded model"),
            },
            (false, None) => "Nothing (no step follows)".into(),
        };

        Handover {
            previous,
            starts_from,
            carried,
            skipped,
            base,
            previous_state: previous_name,
            uses,
            loads,
            removed_loads,
            temperatures,
            model_wide,
            passes_on,
        }
    }

    /// The step whose defined temperatures `step` at `index` sets back to the initial
    /// ones: a step that passes its state on and defines no temperatures of its own does
    /// not keep those a step before it was given (after the last heat transfer step, whose
    /// temperatures are state and carry on). `None` when there are none to set back.
    pub fn temperature_source(&self, index: usize, step: &crate::Step) -> Option<usize> {
        if !step.kind.continues_state()
            || !step.kind.supports_defined_fields()
            || step.defined_fields.iter().any(|f| f.active)
        {
            return None;
        }
        for s in (0..index.min(self.steps.len())).rev() {
            let before = &self.steps[s];
            if !before.active {
                continue;
            }
            if before.kind.is_thermal() {
                return None;
            }
            if before.kind.supports_defined_fields()
                && before.defined_fields.iter().any(|f| f.active)
            {
                return Some(s);
            }
        }
        None
    }
}

fn constraint_active(constraint: &Constraint) -> bool {
    constraint.active()
}

fn plural(count: usize, noun: &str) -> String {
    if count == 1 {
        format!("1 {noun}")
    } else {
        format!("{count} {noun}s")
    }
}

/// "End of Preload" in a sentence: "end of Preload".
fn lower_first(text: &str) -> String {
    let mut chars = text.chars();
    match chars.next() {
        Some(first) => first.to_lowercase().chain(chars).collect(),
        None => String::new(),
    }
}

#[cfg(test)]
mod tests {
    use crate::{Base, BuckleStep, FeModel, FrequencyStep, Load, LoadKind, Region, Step, StepKind};

    fn chain() -> FeModel {
        let mut fe = FeModel::default();
        let mut preload = Step::new_static("Preload");
        preload.loads.push(Load::new(
            "Pressure",
            Region::Parts(Vec::new()),
            LoadKind::Pressure(1.0),
        ));
        fe.steps.push(preload);
        let mut modes = Step::new_frequency("Modes");
        if let StepKind::Frequency(settings) = &mut modes.kind {
            settings.base = Base::PreviousState;
        }
        fe.steps.push(modes);
        fe.steps.push(Step::new_static("Unload"));
        fe
    }

    #[test]
    fn a_first_step_starts_from_the_unloaded_model() {
        let fe = chain();
        let handover = fe.step_handover(0);
        assert_eq!(handover.previous, None);
        assert_eq!(handover.starts_from, "Unloaded model");
        assert_eq!(handover.passes_on, "Its end state to Modes");
        assert_eq!(handover.summary(), "Starts from: Unloaded model");
    }

    #[test]
    fn a_probing_step_names_its_base_and_passes_nothing_on() {
        let fe = chain();
        let handover = fe.step_handover(1);
        assert_eq!(handover.base, Some(Base::PreviousState));
        assert_eq!(handover.previous_state.as_deref(), Some("End of Preload"));
        assert_eq!(handover.summary(), "Base: End of Preload");
        assert_eq!(
            handover.passes_on,
            "Nothing: Unload starts from the end of Preload"
        );
    }

    #[test]
    fn an_unloaded_base_starts_from_the_unloaded_model() {
        let mut fe = chain();
        fe.steps[1].kind = StepKind::Buckle(BuckleStep::default());
        let handover = fe.step_handover(1);
        assert_eq!(handover.starts_from, "Unloaded model");
        assert_eq!(handover.summary(), "Base: Unloaded model");
    }

    #[test]
    fn a_step_after_a_probing_one_continues_the_step_before_it() {
        let fe = chain();
        let handover = fe.step_handover(2);
        assert_eq!(handover.previous, Some(0));
        assert_eq!(handover.starts_from, "End of Preload");
        assert!(handover.carried.starts_with("displacements, stresses"));
        assert_eq!(
            handover.loads.as_deref(),
            Some("Loads change linearly from their values at the end of Preload")
        );
        assert_eq!(handover.removed_loads, ["Pressure"]);
        assert_eq!(handover.passes_on, "Its end state (no step follows)");
    }

    #[test]
    fn complex_frequency_and_steady_state_steps_compute_their_own_modes() {
        let mut fe = chain();
        fe.steps[1].kind = StepKind::ComplexFrequency(crate::ComplexFrequencyStep::default());
        let handover = fe.step_handover(1);
        assert_eq!(handover.summary(), "Base: Unloaded model");
        assert_eq!(
            handover.uses,
            ["Its own 20 eigenmodes, preloaded by the rotation"]
        );
        assert_eq!(
            handover.passes_on,
            "Nothing: Unload starts from the end of Preload"
        );
        fe.steps[1].kind = StepKind::SteadyStateDynamics(crate::SteadyStateDynamicsStep::default());
        let handover = fe.step_handover(1);
        assert_eq!(handover.uses, ["Its own 20 eigenmodes"]);
        assert_eq!(handover.base, Some(Base::Unloaded));
        // Eigenmodes of another analysis: no base of its own.
        fe.properties.stored_modes = Some("modes.eig".into());
        let handover = fe.step_handover(1);
        assert_eq!(handover.uses, ["Eigenmodes linked from modes.eig"]);
        assert_eq!(handover.base, None);
    }

    #[test]
    fn a_deactivated_step_is_skipped() {
        let mut fe = chain();
        fe.steps.insert(1, Step::new_static("Settle"));
        fe.steps[1].active = false;
        let handover = fe.step_handover(3);
        assert_eq!(handover.previous, Some(0));
        assert_eq!(handover.skipped, ["Settle"]);
    }

    #[test]
    fn a_new_step_at_the_end_starts_from_the_last_one() {
        let fe = chain();
        let mut step = Step::new_frequency("New");
        step.kind = StepKind::Frequency(FrequencyStep::default());
        let handover = fe.handover(fe.steps.len(), &step);
        assert_eq!(handover.previous, Some(2));
        assert_eq!(handover.summary(), "Base: Unloaded model");
        assert_eq!(handover.passes_on, "Nothing (no step follows)");
    }
}
