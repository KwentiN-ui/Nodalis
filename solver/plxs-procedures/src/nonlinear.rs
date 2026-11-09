//! `*STATIC` with geometric or material nonlinearity and `*VISCO`: Newton-Raphson iterations
//! per increment with CalculiX's automatic incrementation rules (the manual's section
//! "Convergence criteria" and the `*CONTROLS` defaults): residual check against the average
//! element force, solution check against the incremental displacement, cutbacks on
//! divergence or too many iterations, growth by 1.5 after two fast increments, reduction
//! by 0.75 after a slow one, the creep strain tolerance `CETOL` in visco steps. Exact
//! equality of CalculiX's increment sequence is not a goal (concept section 4.5); the
//! defaults make the sequences agree where the problem is benign.
//!
//! No CalculiX code was consulted.

use std::time::Instant;

use crate::profile::Phase;
use plxs_assembly::discrete::{DiscreteElement, DiscreteKind};
use plxs_assembly::{
    NonlinearInput, Spc, add_membrane_pressures, assemble_nonlinear, load_vector,
    loads::add_carried_loads, membrane_pressures, rotate_concentrated_loads,
};
use plxs_constraints::{ConstraintSet, ReducedPattern};
use plxs_core::step::{DefaultAmplitude, Incrementation, Procedure as ProcedureKind};
use plxs_core::{Convergence, Divergence, Error, Progress, ProgressEvent, Result, Step, Stop};
use plxs_linalg::SparseMatrix;
use plxs_linalg::Unknowns;
use plxs_output::{CvgRow, StaRow};

use crate::equations::{Equations, in_increment};
use crate::fields::write_increment;
use crate::{Procedure, ProcedureContext, StepOutcome};

/// CalculiX's `*CONTROLS` defaults (the manual's section on convergence criteria).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Controls {
    /// Iteration after which the residual is checked for increasing twice in a row (I_0).
    pub divergence_check_after: u32,
    /// Iteration after which the logarithmic convergence estimate is applied (I_R).
    pub estimate_after: u32,
    /// Iteration after which the relaxed residual tolerance applies (I_P).
    pub relaxed_after: u32,
    /// Maximum number of iterations per attempt (I_C).
    pub max_iterations: u32,
    /// Iterations above which the next increment is reduced (I_L).
    pub slow_iterations: u32,
    /// Iterations below or at which an increment counts as fast (I_G).
    pub fast_iterations: u32,
    /// Maximum cutbacks per increment (I_S).
    pub max_cutbacks: u32,
    /// Cutback factors: divergence (D_f), too many iterations predicted (D_C), slow
    /// convergence for the next increment (D_B); growth factor (D_D).
    pub cutback_divergence: f64,
    pub cutback_slow: f64,
    pub reduce_slow: f64,
    pub grow: f64,
    /// Residual tolerance relative to the average force (R_α), its relaxed value (R_P), the
    /// value under which an increment counts as linear (R_l).
    pub residual: f64,
    pub residual_relaxed: f64,
    pub residual_linear: f64,
    /// Solution correction tolerance relative to the incremental solution (C_α) and under
    /// zero flux (C_zero), the zero-flux threshold (ε_α).
    pub correction: f64,
    pub correction_zero_flux: f64,
    pub zero_flux: f64,
    /// Average force at the start of the first step (q_0).
    pub initial_average_force: f64,
}

impl Default for Controls {
    fn default() -> Self {
        Self {
            divergence_check_after: 4,
            estimate_after: 8,
            relaxed_after: 9,
            max_iterations: 16,
            slow_iterations: 10,
            fast_iterations: 4,
            max_cutbacks: 5,
            cutback_divergence: 0.25,
            cutback_slow: 0.5,
            reduce_slow: 0.75,
            grow: 1.5,
            residual: 0.005,
            residual_relaxed: 0.02,
            residual_linear: 1e-8,
            correction: 0.01,
            correction_zero_flux: 1e-3,
            zero_flux: 1e-5,
            initial_average_force: 0.01,
        }
    }
}

impl Controls {
    /// The controls of a step: the defaults, changed by `*CONTROLS, PARAMETERS=TIME
    /// INCREMENTATION` or `FIELD` where the input has them.
    pub fn of_step(step: &Step) -> Self {
        let mut c = Self::default();
        let Some(controls) = &step.controls else {
            return c;
        };
        let v = |i: usize| controls.values.get(i).copied().filter(|v| *v != 0.0);
        let int = |i: usize| v(i).map(|x| x.max(1.0) as u32);
        match controls.parameters.to_ascii_uppercase().as_str() {
            p if p.contains("TIME") => {
                c.divergence_check_after = int(0).unwrap_or(c.divergence_check_after);
                c.estimate_after = int(1).unwrap_or(c.estimate_after);
                c.relaxed_after = int(2).unwrap_or(c.relaxed_after);
                c.max_iterations = int(3).unwrap_or(c.max_iterations);
                c.slow_iterations = int(4).unwrap_or(c.slow_iterations);
                c.fast_iterations = int(5).unwrap_or(c.fast_iterations);
                c.max_cutbacks = int(7).unwrap_or(c.max_cutbacks);
                c.cutback_divergence = v(10).unwrap_or(c.cutback_divergence);
                c.cutback_slow = v(11).unwrap_or(c.cutback_slow);
                c.reduce_slow = v(12).unwrap_or(c.reduce_slow);
                c.grow = v(16).unwrap_or(c.grow);
            }
            p if p.contains("FIELD") => {
                c.residual = v(0).unwrap_or(c.residual);
                c.correction = v(1).unwrap_or(c.correction);
                c.initial_average_force = v(2).unwrap_or(c.initial_average_force);
                c.residual_relaxed = v(4).unwrap_or(c.residual_relaxed);
                c.zero_flux = v(5).unwrap_or(c.zero_flux);
                c.correction_zero_flux = v(6).unwrap_or(c.correction_zero_flux);
                c.residual_linear = v(7).unwrap_or(c.residual_linear);
            }
            _ => {}
        }
        c
    }
}

/// Why an attempt at an increment failed.
enum Failure {
    /// The residual grew in two consecutive iterations, the iteration limit was hit or the
    /// correction is not finite; whether the contact status changed in the last iteration.
    Diverging(Divergence, String, bool),
    /// The logarithmic estimate predicts too many iterations.
    TooSlow(String, bool),
    /// The creep strain changed more than `CETOL` allows (visco steps).
    CreepTolerance { error: f64 },
}

/// A converged attempt.
struct Converged {
    iterations: u32,
    /// The contact springs at the converged displacements.
    contact: Option<plxs_contact::Evaluation>,
    u: Vec<f64>,
    output: plxs_assembly::NonlinearOutput,
    average_force: f64,
}

pub struct NonlinearStatic;

impl Procedure for NonlinearStatic {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        ctx.table.require_elastic()?;
        let (increments, visco, cetol, solver): (Incrementation, bool, f64, _) =
            match &step.procedure {
                ProcedureKind::Static(s) => (s.increments, false, 0.0, s.solver),
                ProcedureKind::Visco(v) => (v.increments, true, v.cetol, v.solver),
                _ => {
                    return Err(Error::Unsupported(
                        "NonlinearStatic needs a *STATIC or *VISCO step".into(),
                    ));
                }
            };
        // CalculiX applies the loads of a *VISCO step at once (step loading).
        let mut step = step.clone();
        if visco && step.default_amplitude == DefaultAmplitude::Ramp {
            step.default_amplitude = DefaultAmplitude::Step;
        }
        let step = &step;
        let controls = Controls::of_step(step);
        let mut equations = Equations::new(solver);

        let period = increments.period();
        if period <= 0.0 {
            return Err(Error::input(
                step.procedure.keyword(),
                "the step period must be positive",
            ));
        }
        let maximum = increments
            .maximum
            .filter(|m| *m > 0.0)
            .unwrap_or(1e30)
            .min(period);
        // The first increment respects the largest one as every later increment does.
        let mut dt = increments.initial().min(period).min(maximum);
        if dt <= 0.0 {
            return Err(Error::input(
                step.procedure.keyword(),
                "the initial increment must be positive",
            ));
        }
        let minimum = increments
            .minimum
            .unwrap_or(f64::INFINITY)
            .min(dt)
            .min(1e-5 * period);
        let mut frames = crate::frames::Frames::of(&increments, period);
        let direct = increments.direct;
        let total_dofs = ctx.table.dofs.len();
        let start_total = ctx.state.total_time;
        let temperature = 0.0;

        // Material state with the layout of this model.
        if ctx.state.material.values.len()
            != plxs_assembly::nonlinear::material_state(ctx.table)
                .values
                .len()
        {
            ctx.state.material = plxs_assembly::nonlinear::material_state(ctx.table);
        }
        if ctx.state.displacements.values.len() != total_dofs {
            ctx.state.displacements = plxs_core::state::NodalField::zeros(ctx.table.dofs.nodes, 3);
        }

        // The prescribed DOFs of the step do not change within it; their values may. SPCs
        // and multi-point constraints are eliminated together, u = T v + g, like the linear
        // procedure does.
        let constraints = ctx.constraints;
        let spc_end = Spc::of_step(
            ctx.model,
            step,
            &ctx.table.dofs,
            period,
            start_total + period,
            period,
        )?;
        let active = ctx.table.active_dofs();
        let set = constraints.step_set(ctx.model.nodes.len(), &active, &spc_end.dofs);
        for warning in &set.warnings {
            progress.event(ProgressEvent::Warning(warning.clone()));
        }
        for note in &set.notes {
            progress.event(ProgressEvent::Info(note.clone()));
        }
        progress.event(ProgressEvent::Info(format!(
            "number of equations: {}",
            set.reduced_len()
        )));
        let discrete: Vec<&DiscreteElement> = ctx.table.discrete.iter().collect();
        // Rigid bodies follow finite rotations under NLGEOM.
        let large_rotations = step.nlgeom && constraints.has_finite_rotations();
        let mut k = SparseMatrix::zeros(ctx.pattern.clone());
        // The pattern of `Tᵀ K T` and where each entry of `K` lands, kept over the Newton
        // iterations and increments while the matrix pattern and `T` stay.
        let mut reducer: Option<ReducedPattern> = None;
        // Contact: *MODEL CHANGE / *CHANGE FRICTION of the step, then a contact state per
        // slave point.
        for warning in ctx.contact.begin_step(step, ctx.state) {
            progress.event(ProgressEvent::Warning(warning));
        }
        let has_contact = ctx.contact.is_active();
        // Displacements at the end of the previous step (`*BOUNDARY, FIXED`).
        let previous = ctx.state.displacements.values.clone();
        let rotation = |node: u32| constraints.rotation(node).copied();
        let prescribed =
            |ctx: &ProcedureContext<'_>, step_time: f64, total: f64| -> Result<Vec<f64>> {
                let mut spc =
                    Spc::of_step(ctx.model, step, &ctx.table.dofs, step_time, total, period)?;
                spc.start_from_previous(
                    ctx.model,
                    step,
                    &ctx.table.dofs,
                    &previous,
                    &rotation,
                    step_time,
                    period,
                );
                spc.apply_fixed(ctx.model, step, &ctx.table.dofs, &previous, &rotation);
                Ok(set.particular(&spc.values))
            };

        let mut average_force = ctx.average_force.unwrap_or(controls.initial_average_force);
        let mut step_force_sum = 0.0;
        let mut step_force_count = 0u32;
        let mut time = 0.0_f64;
        let mut increment = 0u32;
        let mut previous_fast = false;
        let mut warned = false;
        let carried = ctx.state.loads.clone();
        let mut applied: Vec<f64> = Vec::new();
        // Increments end at the points of the amplitudes (and value tables) the step follows,
        // so a motion that changes direction is applied as given; the increment size cut to
        // reach a point is restored afterwards, like that of a frame.
        let breakpoints = step.amplitude_times(ctx.model, start_total, period);
        // Concentrated loads that turn with the structure, under NLGEOM only.
        let followers = if step.nlgeom {
            plxs_assembly::FollowerLoads::new(ctx.model, &ctx.table.dofs, step)
        } else {
            plxs_assembly::FollowerLoads::default()
        };
        while time < period * (1.0 - 1e-12) {
            if ctx.cancel.is_cancelled() {
                return Ok(StepOutcome::Cancelled);
            }
            increment += 1;
            if increment > step.max_increments {
                return Ok(StepOutcome::Diverged {
                    increment,
                    failure: plxs_core::Failure::IncrementLimit,
                    message: format!(
                        "more than INC={} increments are needed; increase INC on *STEP",
                        step.max_increments
                    ),
                });
            }
            let mut attempt = 0u32;
            // The planned size of an increment shortened to end on a frame, which the next
            // increment resumes.
            let mut planned: Option<f64>;
            let converged = loop {
                attempt += 1;
                let full = dt;
                if dt > period - time {
                    dt = period - time;
                }
                if let Some(frames) = &frames {
                    dt = frames.limit(time, dt);
                }
                let tolerance = period * 1e-9;
                if let Some(&next) = breakpoints.iter().find(|&&t| t > time + tolerance)
                    && time + dt > next + tolerance
                {
                    dt = next - time;
                }
                planned = (dt < full).then_some(full);
                let step_time = time + dt;
                let total = start_total + step_time;
                progress.event(ProgressEvent::Increment {
                    step: ctx.step_number,
                    increment,
                    attempt,
                    size: dt,
                    step_time,
                    total_time: total,
                });
                let (mut f_ext, warnings) =
                    load_vector(ctx.model, ctx.table, step, step_time, total, period)?;
                rotate_concentrated_loads(
                    ctx.model,
                    constraints,
                    &ctx.table.dofs,
                    step,
                    step_time,
                    total,
                    period,
                    &mut f_ext,
                );
                // Loads move from their values at the end of the previous step to the ones
                // of this step (an unloading step must not drop them at once).
                add_carried_loads(&mut f_ext, carried.as_deref(), step, step_time, period);
                applied.clone_from(&f_ext);
                // Under NLGEOM pressures on membranes follow the deformed surface (as in
                // CalculiX, see `plxs_elements::membrane`): the reference forces leave the
                // load vector and the assembly adds the current ones.
                let follower = if step.nlgeom {
                    membrane_pressures(ctx.model, ctx.table, step, step_time, total, period)?
                } else {
                    Vec::new()
                };
                if !follower.is_empty() {
                    let mut reference = vec![0.0; f_ext.len()];
                    add_membrane_pressures(ctx.table, &follower, None, &mut reference);
                    for (f, r) in f_ext.iter_mut().zip(&reference) {
                        *f -= r;
                    }
                }
                if !warned {
                    warned = true;
                    for warning in warnings {
                        progress.event(ProgressEvent::Warning(warning));
                    }
                }
                // The particular solutions of the constraints at the start and the end of
                // the increment: their difference is the prescribed motion of the increment.
                let g_start = prescribed(ctx, time, start_total + time)?;
                let g_end = prescribed(ctx, step_time, total)?;
                let turning = followers.at(ctx.model, step, step_time, total, period);
                let input = NonlinearInput {
                    u: &[],
                    nlgeom: step.nlgeom,
                    dt,
                    time: total,
                    creep_active: visco,
                    temperature,
                    follower: &follower,
                    turning: (!turning.is_empty()).then_some(&turning),
                };
                // Contact: candidate master faces from the start of the increment and a
                // matrix pattern with every spring they allow (the last one while it does,
                // so that the solver keeps its symbolic analysis).
                if has_contact {
                    let u0 = ctx.state.displacements.values.clone();
                    let blocks = ctx.contact.prepare(&ctx.model.nodes.coordinates, &u0);
                    let pattern = crate::contact::cached_pattern(
                        ctx.table,
                        &blocks,
                        &mut ctx.contact_pattern,
                    );
                    k = SparseMatrix::zeros(pattern);
                }
                let result = Self::attempt(
                    ctx,
                    progress,
                    &controls,
                    &set,
                    &discrete,
                    &g_start,
                    &g_end,
                    &f_ext,
                    &mut k,
                    input,
                    increment,
                    attempt,
                    step_time,
                    average_force,
                    direct,
                    (visco && cetol > 0.0).then_some(cetol),
                    &mut equations,
                    large_rotations.then_some((active.as_slice(), spc_end.dofs.as_slice())),
                    &mut reducer,
                )?;
                match result {
                    Ok(converged) => break converged,
                    Err(failure) => {
                        let (factor, reason, cause, contact_changing) = match &failure {
                            Failure::Diverging(cause, why, changing) => {
                                (controls.cutback_divergence, why.clone(), *cause, *changing)
                            }
                            Failure::TooSlow(why, changing) => (
                                controls.cutback_slow,
                                why.clone(),
                                Divergence::TooSlow,
                                *changing,
                            ),
                            Failure::CreepTolerance { error } => (
                                (0.8 * cetol / error).clamp(0.1, 0.9),
                                format!(
                                    "creep strain change {error:.3e} exceeds CETOL={cetol:.3e}"
                                ),
                                Divergence::CreepTolerance,
                                false,
                            ),
                        };
                        let failed = |stop| {
                            plxs_core::Failure::NoConvergence(Convergence {
                                cause,
                                stop,
                                contact: has_contact,
                                contact_changing,
                            })
                        };
                        if direct {
                            return Ok(StepOutcome::Diverged {
                                increment,
                                failure: failed(Stop::FixedIncrement),
                                message: format!("{reason}; the increment size is fixed (DIRECT)"),
                            });
                        }
                        if attempt > controls.max_cutbacks {
                            return Ok(StepOutcome::Diverged {
                                increment,
                                failure: failed(Stop::Cutbacks),
                                message: format!(
                                    "{reason}; more than {} cutbacks in one increment",
                                    controls.max_cutbacks
                                ),
                            });
                        }
                        progress.event(ProgressEvent::Cutback {
                            step: ctx.step_number,
                            increment,
                            attempt,
                            reason: reason.clone(),
                        });
                        dt *= factor;
                        if dt < minimum {
                            return Ok(StepOutcome::Diverged {
                                increment,
                                failure: failed(Stop::MinimumIncrement),
                                message: format!(
                                    "{reason}; the increment size {dt:.3e} fell below the minimum {minimum:.3e}"
                                ),
                            });
                        }
                    }
                }
            };

            // Accept the increment.
            let step_time = time + dt;
            let total = start_total + step_time;
            time = step_time;
            step_force_sum += converged.average_force;
            step_force_count += 1;
            average_force = step_force_sum / f64::from(step_force_count);
            ctx.average_force = Some(average_force);
            if let Some(evaluation) = converged.contact.clone() {
                ctx.state.contact = evaluation.states();
                ctx.contact.last = Some(evaluation);
            }
            let Converged {
                iterations,
                u,
                output,
                ..
            } = converged;
            // The forces the constraints carry to their independent nodes, for the output.
            let mut forces = output.f_int.clone();
            let rotated = large_rotations.then(|| ctx.constraints.at_rotations(&u));
            rotated.as_ref().unwrap_or(ctx.constraints).transfer_forces(
                &mut forces,
                &spc_end.dofs,
                &active,
            );
            ctx.state.displacements.per_node = 3;
            ctx.state.displacements.values.clear();
            ctx.state.displacements.values.extend_from_slice(&u);
            ctx.state.stresses = output.stresses;
            ctx.state.strains = output.strains;
            ctx.state.material = output.material;
            ctx.state.loads = Some(applied.clone());
            ctx.state.total_time = total;
            let last = time >= period * (1.0 - 1e-12);
            let started = Instant::now();
            let write = match &mut frames {
                Some(frames) => frames.reached(time) || last,
                None => crate::static_linear::writes_increment_at(step, increment, last),
            };
            if write {
                write_increment(
                    ctx,
                    step,
                    increment,
                    step_time,
                    total,
                    &u,
                    &forces,
                    &ctx.state.stresses.clone(),
                    &ctx.state.strains.clone(),
                    Some(&ctx.state.material.clone()),
                )?;
            }
            crate::history::write_history(
                ctx.results,
                ctx.model,
                ctx.table,
                step,
                ctx.step_number,
                increment,
                step_time,
                &u,
                &forces,
                &ctx.state.stresses,
                &ctx.state.strains,
                Some(&ctx.state.material),
            );
            if let Some(evaluation) = &ctx.contact.last {
                for request in &step.outputs.contact_print {
                    let blocks = plxs_contact::output::history_blocks(
                        &ctx.contact,
                        evaluation,
                        request,
                        &ctx.model.nodes.ids,
                        ctx.step_number,
                        increment,
                        total,
                    );
                    ctx.results.history.extend(blocks);
                }
            }
            ctx.profile.add(Phase::Results, started);
            ctx.sta.push(StaRow {
                step: ctx.step_number,
                increment,
                attempt,
                iterations,
                total_time: total,
                step_time,
                increment_size: dt,
            });
            progress.event(ProgressEvent::IncrementDone {
                step: ctx.step_number,
                increment,
                attempt,
                iterations,
                total_time: total,
                step_time,
                size: dt,
            });

            // The size of the next increment.
            if let Some(full) = planned {
                dt = full;
            }
            if !direct && !last {
                let fast = iterations <= controls.fast_iterations;
                if iterations > controls.slow_iterations {
                    dt *= controls.reduce_slow;
                } else if fast && previous_fast {
                    dt *= controls.grow;
                }
                previous_fast = fast;
                dt = dt.min(maximum);
            }
        }
        ctx.state.total_time = start_total + period;
        ctx.state.steps_done = ctx.step_number;
        Ok(StepOutcome::Converged)
    }
}

impl NonlinearStatic {
    /// Adds the springs and the closed gaps to the tangent and their forces to `f_int`;
    /// returns which discrete elements act (springs always, gaps when closed at `u`).
    pub(crate) fn add_discrete(
        discrete: &[&DiscreteElement],
        u: &[f64],
        k: &mut SparseMatrix,
        f_int: &mut [f64],
    ) -> Vec<bool> {
        let mut acting = Vec::with_capacity(discrete.len());
        let mut clearance = vec![0.0; f_int.len()];
        for element in discrete {
            let block = match element.kind {
                DiscreteKind::Spring { .. } => element.spring_stiffness(),
                DiscreteKind::Gap { .. } => element
                    .gap_opening(u)
                    .is_some_and(|o| o <= 0.0)
                    .then(|| element.closed_stiffness()),
            };
            acting.push(block.is_some());
            let Some(block) = block else {
                continue;
            };
            let m = element.dofs.len();
            k.add_block(&element.dofs, &block);
            let ue: Vec<f64> = element.dofs.iter().map(|&d| u[d as usize]).collect();
            for (a, &dof) in element.dofs.iter().enumerate() {
                let f: f64 = (0..m).map(|b| block[a * m + b] * ue[b]).sum();
                f_int[dof as usize] += f;
            }
            // A closed gap's force k (d + n·Δu) has the constant part k d: the load-side
            // term of the linear procedure, here part of the internal force.
            element.add_clearance_force(&mut clearance);
        }
        for (f, c) in f_int.iter_mut().zip(&clearance) {
            *f -= c;
        }
        acting
    }

    /// One attempt at an increment: Newton iterations from the state at the start of the
    /// increment. `Ok(Err(..))` is a failed attempt the caller cuts back.
    #[allow(clippy::too_many_arguments)]
    fn attempt(
        ctx: &mut ProcedureContext<'_>,
        progress: &mut dyn Progress,
        controls: &Controls,
        set: &ConstraintSet,
        discrete: &[&DiscreteElement],
        g_start: &[f64],
        g_end: &[f64],
        f_ext: &[f64],
        k: &mut SparseMatrix,
        input: NonlinearInput<'_>,
        increment: u32,
        attempt: u32,
        step_time: f64,
        average_force: f64,
        direct: bool,
        cetol: Option<f64>,
        equations: &mut Equations,
        large_rotations: Option<(&[bool], &[u32])>,
        reducer: &mut Option<ReducedPattern>,
    ) -> Result<std::result::Result<Converged, Failure>> {
        let total_dofs = ctx.table.dofs.len();
        // Start from the converged displacements, u = T v + g_start. A change of the
        // prescribed values enters through the first iteration as a linear prediction,
        // `Tᵀ K T Δv = Tᵀ (r - K Δg)` with the tangent of the converged state, instead of
        // moving the prescribed nodes alone and leaving the first Newton iteration a
        // heavily distorted trial state.
        let mut u = ctx.state.displacements.values.clone();
        u.resize(total_dofs, 0.0);
        let delta_g: Vec<f64> = g_end.iter().zip(g_start).map(|(e, s)| e - s).collect();
        let mut pending_g = delta_g.iter().any(|d| *d != 0.0);
        let u_start: Vec<f64> = u.iter().zip(&delta_g).map(|(u, d)| u + d).collect();
        let mut k_delta = vec![0.0; total_dofs];
        let mut residuals: Vec<f64> = Vec::new();
        let mut last_correction = f64::INFINITY;
        let mut previous_correction = f64::INFINITY;
        let mut acting: Option<Vec<u8>> = None;
        // The last iteration in which the contact status changed.
        let mut last_change: Option<u32> = None;
        let mut iteration = 0u32;
        loop {
            let started = Instant::now();
            let mut output = assemble_nonlinear(
                ctx.table,
                ctx.model,
                &NonlinearInput { u: &u, ..input },
                ctx.state,
                Some(k),
            );
            let mut status: Vec<u8> = Self::add_discrete(discrete, &u, k, &mut output.f_int)
                .into_iter()
                .map(u8::from)
                .collect();
            let closed_gaps = discrete
                .iter()
                .zip(&status)
                .filter(|(e, x)| **x > 0 && matches!(e.kind, DiscreteKind::Gap { .. }))
                .count() as u32;
            // Contact springs at the current displacements; a change of the contact status
            // (closed, sticking, sliding) needs another iteration like a gap that changed.
            let contact = if !ctx.contact.is_empty() {
                let evaluation = ctx.contact.evaluate(
                    &ctx.model.nodes.coordinates,
                    &u,
                    &ctx.state.contact,
                    iteration == 0,
                );
                for element in &evaluation.elements {
                    k.add_block(&element.dofs, &element.ke);
                    for (&dof, f) in element.dofs.iter().zip(&element.fe) {
                        output.f_int[dof as usize] += f;
                    }
                }
                status.extend(evaluation.status_key());
                Some(evaluation)
            } else {
                None
            };
            let contact_elements = closed_gaps + contact.as_ref().map_or(0, |e| e.active as u32);
            ctx.profile.add(Phase::Assembly, started);
            let gaps_changed = acting.as_ref().is_some_and(|a| *a != status);
            acting = Some(status);
            if gaps_changed {
                last_change = Some(iteration);
            }
            // The contact status changed in one of the last three iterations.
            let changing = last_change.is_some_and(|i| iteration - i <= 2);
            let flux = if output.flux_count > 0 {
                output.flux_sum / output.flux_count as f64
            } else {
                0.0
            };
            // The average force of the step so far, with this increment's value included,
            // is the scale of the residual check.
            let scale = if ctx.average_force.is_some() || ctx.step_number > 1 {
                0.5 * (average_force + flux)
            } else {
                flux.max(average_force)
            };
            if pending_g {
                k.multiply(&delta_g, &mut k_delta);
            }
            let residual_full: Vec<f64> = (0..total_dofs)
                .map(|d| f_ext[d] - output.f_int[d] - if pending_g { k_delta[d] } else { 0.0 })
                .collect();
            // With rigid bodies under finite rotations, the constraints linearized at `u`.
            let rotated = large_rotations.map(|(active, held)| {
                ctx.constraints
                    .at_rotations(&u)
                    .step_set(ctx.model.nodes.len(), active, held)
            });
            let set = rotated.as_ref().unwrap_or(set);
            let started = Instant::now();
            let residual = set.reduce_rhs(&residual_full);
            ctx.profile.add(Phase::Reduction, started);
            let r_max = residual.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            // An inverted element of a hyperelastic material has no stress (NaN, which the
            // maximum above skips): the increment is cut back before the tangent is
            // factorized.
            if residual.iter().any(|r| !r.is_finite()) {
                return Ok(Err(Failure::Diverging(
                    Divergence::NotFinite,
                    "an element is turned inside out (the residual is not finite)".into(),
                    false,
                )));
            }
            let du_max = u
                .iter()
                .zip(&u_start)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0_f64, f64::max);
            residuals.push(r_max);

            if iteration > 0 {
                ctx.cvg.push(CvgRow {
                    step: ctx.step_number,
                    increment,
                    attempt,
                    iteration,
                    contact_elements,
                    residual: r_max,
                    correction: last_correction,
                    step_time,
                    increment_size: input.dt,
                });
                let zero_flux =
                    scale <= controls.zero_flux * average_force.max(1e-300) || scale == 0.0;
                let residual_ok = if zero_flux {
                    true
                } else {
                    let tolerance = if iteration > controls.relaxed_after {
                        controls.residual_relaxed
                    } else {
                        controls.residual
                    };
                    r_max <= tolerance * scale
                };
                let linear = !zero_flux && r_max <= controls.residual_linear * scale;
                let correction_ok = if zero_flux {
                    last_correction <= controls.correction_zero_flux * du_max || du_max == 0.0
                } else {
                    last_correction <= controls.correction * du_max
                        || linear
                        || (previous_correction.is_finite()
                            && previous_correction > 0.0
                            && last_correction * last_correction / previous_correction
                                <= controls.correction * du_max)
                };
                // CalculiX always iterates at least twice; the second iteration confirms
                // the first one's residual and makes class B comparisons less noisy. A gap
                // that opened or closed in this iteration needs another one as well.
                let converged = residual_ok && correction_ok && iteration >= 2 && !gaps_changed;
                progress.event(ProgressEvent::Iteration {
                    step: ctx.step_number,
                    increment,
                    iteration,
                    residual: r_max,
                    correction: last_correction,
                    converged,
                });
                if converged {
                    if let Some(cetol) = cetol {
                        let error = input.dt * output.max_creep_rate_change;
                        if error > cetol {
                            return Ok(Err(Failure::CreepTolerance { error }));
                        }
                    }
                    return Ok(Ok(Converged {
                        iterations: iteration,
                        u,
                        output,
                        average_force: flux,
                        contact,
                    }));
                }
                // Divergence and slow-convergence checks (not across a change of the gap
                // status, which restarts the convergence).
                let n = residuals.len();
                if !direct
                    && !gaps_changed
                    && iteration >= controls.divergence_check_after
                    && n >= 3
                    && residuals[n - 1] > residuals[n - 2]
                    && residuals[n - 2] > residuals[n - 3]
                {
                    return Ok(Err(Failure::Diverging(
                        Divergence::ResidualGrows,
                        "the residual force increased in two consecutive iterations".into(),
                        changing,
                    )));
                }
                if iteration >= controls.estimate_after && n >= 2 && !direct && !gaps_changed {
                    let (r1, r0) = (residuals[n - 1], residuals[n - 2]);
                    let target = controls.residual * scale;
                    if r1 > 0.0 && r0 > 0.0 && r1 < r0 && target > 0.0 {
                        let estimate = f64::from(iteration) + (target / r1).ln() / (r1 / r0).ln();
                        if estimate > f64::from(controls.max_iterations) {
                            return Ok(Err(Failure::TooSlow(
                                format!(
                                    "about {} iterations would be needed to converge",
                                    estimate.ceil()
                                ),
                                changing,
                            )));
                        }
                    }
                }
                if iteration >= controls.max_iterations {
                    return Ok(Err(Failure::Diverging(
                        Divergence::IterationLimit,
                        format!(
                            "no convergence after {} iterations",
                            controls.max_iterations
                        ),
                        changing,
                    )));
                }
            }

            // Solve Tᵀ K T Δv = Tᵀ r and correct u = T v + g.
            iteration += 1;
            let started = Instant::now();
            let k_reduced = set.reduce_cached(k, reducer);
            ctx.profile.add(Phase::Reduction, started);
            let started = Instant::now();
            let coordinates = || Unknowns::from_free(&set.free, 3, &ctx.model.nodes.coordinates);
            // The solver's messages would repeat in every iteration: only those of the first
            // system of the step and the warnings (fall-backs of the automatic choice).
            let mut sink = crate::equations::FirstOrWarnings {
                inner: &mut *progress,
                all: increment == 1 && attempt == 1 && iteration == 1,
            };
            equations
                .prepare(&mut ctx.solver, &k_reduced, coordinates, true, &mut sink)
                .map_err(|e| match e {
                    Error::Numerical {
                        failure, message, ..
                    } => Error::Numerical {
                        step: ctx.step_number,
                        increment,
                        failure,
                        message: if equations.hints_constraints() {
                            format!("{message}; the model is probably not sufficiently constrained")
                        } else {
                            message
                        },
                    },
                    other => other,
                })?;
            ctx.profile.add(Phase::Factorization, started);
            let started = Instant::now();
            let unsymmetric: Vec<&plxs_contact::ContactElement> = contact
                .as_ref()
                .map(|e| {
                    e.elements
                        .iter()
                        .filter(|e| !e.unsymmetric.is_empty())
                        .collect()
                })
                .unwrap_or_default();
            let dv = if unsymmetric.is_empty() {
                let mut dv = residual;
                let mut sink = crate::equations::FirstOrWarnings {
                    inner: &mut *progress,
                    all: false,
                };
                equations
                    .solve(&mut ctx.solver, &mut dv, &mut sink)
                    .map_err(|e| in_increment(e, ctx.step_number, increment))?;
                dv
            } else {
                // Sliding friction makes the tangent unsymmetric: GMRES on the full
                // tangent, preconditioned by the factorized symmetric part.
                let zero = vec![0.0; total_dofs];
                let solver = &ctx.solver;
                let apply = |v: &[f64], out: &mut [f64]| {
                    k_reduced.multiply(v, out);
                    let full = set.expand(v, &zero);
                    let mut e = vec![0.0; total_dofs];
                    for element in &unsymmetric {
                        element.apply_unsymmetric(&full, &mut e);
                    }
                    for (o, r) in out.iter_mut().zip(set.reduce_rhs(&e)) {
                        *o += r;
                    }
                };
                let precondition = |v: &mut [f64]| equations.apply_inverse(solver, v);
                let result = plxs_linalg::gmres(&apply, &precondition, &residual, 1e-10, 60, 300);
                match result {
                    Ok((dv, _)) => dv,
                    // The multigrid's own iteration as the preconditioner may miss its
                    // tolerance; the factorization stands in, as for a symmetric system.
                    Err(error)
                        if equations.fall_back_to_direct(
                            &mut ctx.solver,
                            &k_reduced,
                            coordinates,
                            &error,
                            &mut *progress,
                        )? =>
                    {
                        let solver = &ctx.solver;
                        let precondition = |v: &mut [f64]| equations.apply_inverse(solver, v);
                        plxs_linalg::gmres(&apply, &precondition, &residual, 1e-10, 60, 300)
                            .map_err(|e| in_increment(e, ctx.step_number, increment))?
                            .0
                    }
                    Err(error) => return Err(in_increment(error, ctx.step_number, increment)),
                }
            };
            ctx.profile.add(Phase::Solve, started);
            let mut du = set.expand(&dv, &vec![0.0; total_dofs]);
            // A correction that drives contact points deep into the master (a body held
            // only by weak springs until it touches) is shortened; see
            // `ContactModel::step_limit`.
            if !ctx.contact.is_empty() {
                let full: Vec<f64> = if pending_g {
                    du.iter().zip(&delta_g).map(|(a, b)| a + b).collect()
                } else {
                    du.clone()
                };
                let work: f64 = du.iter().zip(&residual_full).map(|(d, r)| d * r).sum();
                let factor = ctx.contact.step_limit(
                    &ctx.model.nodes.coordinates,
                    &u,
                    &full,
                    &ctx.state.contact,
                    work,
                );
                if factor < 1.0 {
                    for d in &mut du {
                        *d *= factor;
                    }
                }
            }
            previous_correction = last_correction;
            last_correction = du.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            if !last_correction.is_finite() {
                return Ok(Err(Failure::Diverging(
                    Divergence::NotFinite,
                    "the solution correction is not finite".into(),
                    false,
                )));
            }
            for (ui, d) in u.iter_mut().zip(&du) {
                *ui += d;
            }
            if pending_g {
                for (ui, d) in u.iter_mut().zip(&delta_g) {
                    *ui += d;
                }
                pending_g = false;
            }
            if let Some((_, held)) = large_rotations {
                ctx.constraints.project_rigid(&mut u, held);
            }
        }
    }
}
