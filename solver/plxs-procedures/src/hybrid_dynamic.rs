//! `*DYNAMIC, INTEGRATION=AUTOMATIC`: a dynamic step that integrates implicitly while the
//! motion is smooth and explicitly through impacts and wherever the implicit increments
//! would have to become tiny.
//!
//! The step runs in parts ([`Segment`]) on the frame grid of the step (its increments).
//! Each part is computed by one of the two integrators on the state the part before it
//! ended with, the way a step continues the step before it: displacements, velocities,
//! stresses, material and contact state are taken over, the accelerations are computed
//! again from the integrator's own mass.
//!
//! - **Implicit to explicit**: an implicit part stops where a sub-increment would be
//!   shorter than `SWITCH=` times the stable explicit time step (`k`, default 30: an
//!   implicit increment costs about as much as 30 to 100 explicit time steps), where an
//!   impact would have to be split that far, or where an increment does not converge even
//!   split. The state of the last converged increment goes on explicitly to the end of
//!   the frame it lies in.
//! - **Explicit to implicit**: at the end of an explicit part an implicit part is tried
//!   again. When it stops again before it got as far as the explicit part before it, the
//!   explicit parts become longer: 2, 4, 8 and at most 16 frames before the next attempt. Converged implicit increments of
//!   an attempt are kept (they are correct), so a failed attempt costs only the increment
//!   it stopped in.
//!
//! The decisions depend on the model only, so the same model switches at the same times.
//!
//! No CalculiX code was consulted (CalculiX has no such procedure).

use plxs_core::step::{Integration, Procedure as ProcedureKind};
use plxs_core::{Error, Progress, ProgressEvent, Result, Step};

use crate::dynamic::short;
use crate::{Procedure, ProcedureContext, Segment, StepOutcome};

/// The longest explicit part between two attempts to go back to implicit, in frames.
const MAX_EXPLICIT_FRAMES: u32 = 16;

pub struct HybridDynamic;

impl Procedure for HybridDynamic {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        let ProcedureKind::Dynamic(parameters) = &step.procedure else {
            return Err(Error::Unsupported(
                "HybridDynamic needs a *DYNAMIC step".into(),
            ));
        };
        let Integration::Automatic {
            explicit,
            switch_factor,
        } = parameters.integration
        else {
            return Err(Error::Unsupported(
                "HybridDynamic needs INTEGRATION=AUTOMATIC".into(),
            ));
        };
        let with = |integration: Integration| {
            let mut clone = step.clone();
            if let ProcedureKind::Dynamic(d) = &mut clone.procedure {
                d.integration = integration;
            }
            clone
        };
        let mut implicit_step = with(Integration::Implicit);
        let mut explicit_step = with(Integration::Explicit(explicit));

        // A linear model without contact has nothing that would make the implicit
        // increments short: it stays implicit (and linear).
        let nonlinear =
            step.nlgeom || ctx.table.has_nonlinear_material() || !ctx.contact.is_empty();
        if !nonlinear || switch_factor <= 0.0 {
            progress.event(ProgressEvent::Info(if switch_factor <= 0.0 {
                "automatic: switch factor 0, the step stays implicit".into()
            } else {
                "automatic: linear model without contact, the step stays implicit".into()
            }));
            return if nonlinear {
                crate::nonlinear_dynamic::NonlinearDynamic.run(ctx, &implicit_step, progress)
            } else {
                crate::dynamic::DynamicProcedure.run(ctx, &implicit_step, progress)
            };
        }

        let increments = parameters.increments;
        let period = increments.period();
        if period <= 0.0 {
            return Err(Error::input("*DYNAMIC", "the step period must be positive"));
        }
        let interval = increments.initial().min(period);
        if interval <= 0.0 {
            return Err(Error::input(
                "*DYNAMIC",
                "the initial increment must be positive",
            ));
        }
        let stable = crate::explicit_dynamic::stable_time_step_estimate(ctx, step, &explicit)?;
        let shortest = switch_factor * stable;
        progress.event(ProgressEvent::Info(format!(
            "automatic: implicit while its increments stay above {} s ({} times the stable \
             explicit time step of {} s), explicit below",
            short(shortest),
            short(switch_factor),
            short(stable)
        )));

        // What every part starts from as the step does.
        let energy = ctx.state.energy.clone();
        let steps_done = ctx.state.steps_done;
        let n = ctx.table.dofs.len();
        let mut segment = Segment {
            start: 0.0,
            end: period,
            first: true,
            previous: if ctx.state.displacements.values.len() == n {
                ctx.state.displacements.values.clone()
            } else {
                vec![0.0; n]
            },
            shortest,
            ..Segment::default()
        };
        let tolerance = 1e-9 * period;
        let mut implicit = true;
        // An implicit attempt after an explicit part (not the start of the step).
        let mut attempt = false;
        // Frames of the next explicit part.
        let mut explicit_frames = 1u32;
        let mut explicit_time = 0.0;
        let mut switches = 0u32;
        loop {
            ctx.state.energy.clone_from(&energy);
            ctx.state.steps_done = steps_done;
            segment.stopped = None;
            let outcome = if implicit {
                segment.end = period;
                ctx.segment = Some(segment.clone());
                crate::nonlinear_dynamic::NonlinearDynamic.run(ctx, &implicit_step, progress)
            } else {
                let frame = (segment.start / interval + 1e-9).floor();
                segment.end = ((frame + f64::from(explicit_frames)) * interval).min(period);
                ctx.segment = Some(segment.clone());
                crate::explicit_dynamic::ExplicitDynamic.run(ctx, &explicit_step, progress)
            };
            let done = ctx.segment.take().expect("the part");
            match outcome {
                Ok(StepOutcome::Converged) => {}
                other => {
                    ctx.segment = None;
                    return other;
                }
            }
            // The damping is fitted once. With rigid body or mechanism modes the implicit
            // parts keep the stiffness-proportional damping they fitted and the explicit
            // parts have none (mass-proportional damping would brake the free motion).
            if let Some(rayleigh) = done.rigid_damping {
                for (clone, keep) in [(&mut implicit_step, true), (&mut explicit_step, false)] {
                    if let ProcedureKind::Dynamic(d) = &mut clone.procedure {
                        d.damping_ratio = None;
                        d.rayleigh = keep.then_some(rayleigh);
                    }
                }
            }
            if let Some(lowest) = done.lowest {
                for clone in [&mut implicit_step, &mut explicit_step] {
                    if let ProcedureKind::Dynamic(d) = &mut clone.procedure
                        && let Some(ratio) = d.damping_ratio.as_mut()
                    {
                        ratio.lower.get_or_insert(lowest);
                    }
                }
            }
            if !implicit {
                explicit_time += done.reached - segment.start;
            }
            let from = segment.start;
            segment = Segment {
                start: done.reached,
                first: false,
                lowest: None,
                rigid_damping: None,
                ..done
            };
            if segment.start >= period - tolerance {
                break;
            }
            if implicit {
                let reason = segment.stopped.take().unwrap_or_default();
                // An attempt that stopped before it got as far as the explicit part before
                // it: the explicit parts grow before the next one.
                let ran = segment.start - from;
                let failed = attempt && ran < f64::from(explicit_frames) * interval - tolerance;
                explicit_frames = if failed {
                    (explicit_frames * 2).min(MAX_EXPLICIT_FRAMES)
                } else {
                    1
                };
                if ran > tolerance || !attempt {
                    switches += 1;
                    progress.event(ProgressEvent::Info(format!(
                        "automatic: explicit from {} s: {reason}",
                        short(segment.start)
                    )));
                }
                implicit = false;
            } else {
                implicit = true;
                attempt = true;
            }
        }
        ctx.segment = None;
        progress.event(ProgressEvent::Info(format!(
            "automatic: {} of {} s explicit, {switches} switch{} to explicit",
            short(explicit_time),
            short(period),
            if switches == 1 { "" } else { "es" }
        )));
        Ok(StepOutcome::Converged)
    }
}
