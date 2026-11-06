//! `*DYNAMIC` with `NLGEOM` or nonlinear materials: the α-method of Hilber, Hughes and
//! Taylor of [`crate::dynamic`] with Newton iterations in every increment.
//!
//! The equation of motion at the shifted time,
//!
//! ```text
//! M a₁ + (1 + α)(C v₁ + f(u₁)) − α (C v₀ + f(u₀)) = (1 + α) p₁ − α p₀
//! ```
//!
//! with the internal forces `f(u)` of the nonlinear assembly (total Lagrange under
//! `NLGEOM`, plastic and creeping materials), the external loads `p` and Newmark's updates
//! `a₁ = (u₁ − ũ)/(βΔt²)`, `v₁ = ṽ + γΔt a₁` from the predictors `ũ`, `ṽ`. Newton solves
//! for `u₁` with the tangent `M/(βΔt²) + (1+α)(γ/(βΔt) C + K_T(u₁))`. The mass matrix
//! belongs to the undeformed configuration (constant under total Lagrange); Rayleigh
//! damping `α_R M v + β_R ḟ` damps the rate of the internal forces, `(f(u₁) − f(u₀))/Δt`
//! over the increment (`β_R K v` for small motions). A finite rigid rotation (a pendulum
//! swinging about a joint) does not change the internal forces and is not braked, which it
//! would be by `K₀ v` like by a viscous fluid; and unlike `K_T(u) v` the force has the
//! exact Newton tangent `β_R K_T/Δt`.
//!
//! Rigid bodies follow finite rotations: their constraints are linearized at the current
//! rotation in every iteration and their members put where the rotation takes them (see
//! [`plxs_constraints::ModelConstraints::at_rotations`]).
//!
//! Contact pairs (node to surface, surface to surface, with friction; also the reason a
//! small-deformation dynamic step comes here) add their penalty springs to the internal
//! forces: `f_c(u)` enters the equation of motion α-weighted like `f(u)`, the springs are
//! evaluated from the contact states at the start of every (sub-)increment (the friction
//! slips), and an iteration in which the contact status changed is never the last. Rayleigh
//! damping leaves the springs out: `β_R ḟ_c` would hold a body that bounces off a surface as
//! if glued (the contact force falls to zero as it lifts off). An impact is resolved by the
//! adaptive sub-increments: the jump of the accelerations raises the local error estimate,
//! and an increment whose Newton iterations do not settle the contact set is split.
//!
//! An increment that does not converge is split in halves (up to 64 parts); the result
//! frames stay on the increments of the step.
//!
//! No CalculiX code was consulted.

use plxs_assembly::discrete::DiscreteElement;
use plxs_assembly::{
    MassRule, NonlinearInput, Spc, assemble_mass_with, assemble_nonlinear, assemble_stiffness,
    load_vector, rotate_concentrated_loads,
};
use plxs_constraints::{ConstraintSet, ModelConstraints};
use plxs_contact::Evaluation;
use plxs_core::state::{ContactPointState, IntegrationPointField, NodalField};
use plxs_core::step::{Procedure as ProcedureKind, Rayleigh};
use plxs_core::{Convergence, Divergence, Error, Progress, ProgressEvent, Result, Step, Stop};
use plxs_linalg::{SparseMatrix, Unknowns};
use plxs_output::{CvgRow, StaRow};

use crate::dynamic::{
    Energies, Motion, Scheme, combine, ctx_max_increments, energy_requests, fit_damping,
    initial_acceleration_matrix, initial_values, matrix_factors, multiply, numerical, short,
    write_energies, write_increment,
};
use crate::equations::{Equations, FirstOrWarnings, in_increment};
use crate::fields::{internal_energy_density, nodal_average_scalar};
use crate::nonlinear::NonlinearStatic;
use crate::{Procedure, ProcedureContext, StepOutcome};

pub struct NonlinearDynamic;

/// Newton iterations per attempt before the increment is split.
const MAX_ITERATIONS: u32 = 16;
/// How often an increment may be halved.
const MAX_SPLITS: u32 = 6;
/// How often an increment may be halved for an impact (see [`IMPACT_ENERGY`]): down to a
/// few nanoseconds of a millisecond, only around the impact.
const MAX_IMPACT_SPLITS: u32 = 16;
/// Largest error of the contact's energy in a (sub-)increment relative to the energy of the
/// motion (kinetic energy, work of the loads, energy of the springs).
const IMPACT_ENERGY: f64 = 1e-3;
/// Iterations above which the following increments are computed in twice as many parts,
/// and at or below which in half as many: large rotations per increment converge slowly
/// and lose accuracy.
const SLOW_ITERATIONS: u32 = 8;
const FAST_ITERATIONS: u32 = 3;
/// Largest local error of a sub-increment relative to its motion (Zienkiewicz and Xie's
/// estimate for Newmark's method), and how many parts an increment may take for it.
const ACCURACY: f64 = 0.01;
const ACCURACY_PARTS: u32 = 8;
/// Residual tolerance relative to the largest force of the increment (external, inertial
/// or the average element force) and correction tolerance relative to the motion of the
/// increment.
const RESIDUAL: f64 = 1e-3;
const CORRECTION: f64 = 1e-4;

/// The state at a time of the step: the motion, the forces the next increment starts from
/// and the results at the integration points.
#[derive(Clone)]
struct At {
    time: f64,
    motion: Motion,
    /// External loads and internal forces.
    load: Vec<f64>,
    internal: Vec<f64>,
    /// The damping forces (mass part `α_R M v`, stiffness part `β_R ḟ`).
    damping: Vec<f64>,
    stresses: IntegrationPointField,
    strains: IntegrationPointField,
    material: IntegrationPointField,
    /// Accumulated energies (internal: work of the internal and contact forces).
    energies: Energies,
    /// The forces of the contact springs (empty without contact), the states of the slave
    /// points (friction slips) and the springs of the last evaluation.
    contact: Vec<f64>,
    /// The part of `contact` that is damping, the part from the normal springs alone and
    /// the energy these store.
    contact_damping: Vec<f64>,
    contact_normal: Vec<f64>,
    contact_energy: f64,
    contact_states: Vec<Vec<ContactPointState>>,
    evaluation: Option<Evaluation>,
}

/// What the increments share.
struct Setup<'s> {
    step: &'s Step,
    scheme: Scheme,
    rayleigh: Rayleigh,
    m: SparseMatrix,
    k0: SparseMatrix,
    /// The tangent and the system matrix, reused between iterations.
    k: SparseMatrix,
    set: ConstraintSet,
    spc_dofs: Vec<u32>,
    active: Vec<bool>,
    /// Rigid bodies under finite rotations.
    large_rotations: bool,
    previous: Vec<f64>,
    start_total: f64,
    period: f64,
    discrete: Vec<&'s DiscreteElement>,
    energies_wanted: bool,
    /// Whether contact pairs are in force; the mass matrix on the pattern with the contact
    /// springs of the current sub-increment.
    has_contact: bool,
    m_contact: Option<SparseMatrix>,
    /// The size of the model (diagonal of its bounding box).
    size: f64,
}

/// Why an attempt failed.
struct Failed {
    cause: Divergence,
    reason: String,
    /// The contact status changed in one of the last iterations.
    contact_changing: bool,
}

impl Procedure for NonlinearDynamic {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        let ProcedureKind::Dynamic(parameters) = &step.procedure else {
            return Err(Error::Unsupported(
                "NonlinearDynamic needs a *DYNAMIC step".into(),
            ));
        };
        if parameters.explicit {
            return Err(Error::Unsupported(format!(
                "procedure not implemented yet: *DYNAMIC, EXPLICIT (step {})",
                step.name
            )));
        }
        let increments = parameters.increments;
        let period = increments.period();
        if period <= 0.0 {
            return Err(Error::input("*DYNAMIC", "the step period must be positive"));
        }
        let initial = increments.initial().min(period);
        if initial <= 0.0 {
            return Err(Error::input(
                "*DYNAMIC",
                "the initial increment must be positive",
            ));
        }
        ctx.table.require_elastic()?;
        let scheme = Scheme::of(parameters);
        let n = ctx.table.dofs.len();
        let start_total = ctx.state.total_time;

        let started = std::time::Instant::now();
        if ctx.stiffness.is_none() {
            let mut k = SparseMatrix::zeros(ctx.pattern.clone());
            assemble_stiffness(ctx.table, ctx.model, &mut k);
            ctx.stiffness = Some(k);
        }
        let k0 = ctx.stiffness.clone().unwrap();
        let mut m = SparseMatrix::zeros(ctx.pattern.clone());
        assemble_mass_with(ctx.table, ctx.model, MassRule::Full, &mut m)?;
        progress.event(ProgressEvent::Info(format!(
            "stiffness and mass assembled in {:.2} s",
            started.elapsed().as_secs_f64()
        )));
        progress.event(ProgressEvent::Info(format!(
            "{}: time step {} s resolves vibrations up to {} Hz",
            if step.nlgeom || ctx.table.has_nonlinear_material() {
                "large deformations"
            } else {
                "contact"
            },
            short(initial),
            short(1.0 / (10.0 * initial))
        )));
        // Contact: *MODEL CHANGE / *CHANGE FRICTION of the step, then a contact state per
        // slave point.
        for warning in ctx.contact.begin_step(step, ctx.state) {
            progress.event(ProgressEvent::Warning(warning));
        }
        let has_contact = ctx.contact.is_active();

        // Material state and displacements with the layout of this model.
        let layout = plxs_assembly::nonlinear::material_state(ctx.table);
        if ctx.state.material.values.len() != layout.values.len() {
            ctx.state.material = layout;
        }

        let spc_end = Spc::of_step(
            ctx.model,
            step,
            &ctx.table.dofs,
            period,
            start_total + period,
            period,
        )?;
        let active = ctx.table.active_dofs();
        let set = ctx
            .constraints
            .step_set(ctx.model.nodes.len(), &active, &spc_end.dofs);
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
        let rayleigh = match parameters.damping_ratio {
            Some(ratio) => fit_damping(ctx, &set, &k0, &m, ratio, parameters.solver, progress)?,
            None => {
                let r = parameters.rayleigh.unwrap_or_default();
                if r.alpha != 0.0 || r.beta != 0.0 {
                    progress.event(ProgressEvent::Info(format!(
                        "Rayleigh damping alpha = {}, beta = {}",
                        short(r.alpha),
                        short(r.beta)
                    )));
                }
                r
            }
        };
        let large_rotations = step.nlgeom && ctx.constraints.has_finite_rotations();
        let table: &plxs_assembly::ElementTable = ctx.table;
        let mut setup = Setup {
            step,
            scheme,
            rayleigh,
            k: SparseMatrix::zeros(ctx.pattern.clone()),
            m,
            k0,
            set,
            spc_dofs: spc_end.dofs.clone(),
            active,
            large_rotations,
            previous: ctx.state.displacements.values.clone(),
            start_total,
            period,
            discrete: table.discrete.iter().collect(),
            energies_wanted: !energy_requests(step).is_empty(),
            has_contact,
            m_contact: None,
            size: model_size(&ctx.model.nodes.coordinates),
        };
        let mut equations = Equations::new(parameters.solver).preferring_direct();

        // The motion the step starts from.
        let mut motion = Motion {
            u: if ctx.state.displacements.values.len() == n {
                ctx.state.displacements.values.clone()
            } else {
                vec![0.0; n]
            },
            v: match &ctx.state.velocities {
                Some(v) if v.values.len() == n => v.values.clone(),
                _ => initial_values(ctx.model, ctx.table, &ctx.model.initial.velocities),
            },
            a: vec![0.0; n],
        };
        if ctx.state.velocities.is_none()
            && ctx.state.steps_done == 0
            && !ctx.model.initial.displacements.is_empty()
        {
            motion.u = initial_values(ctx.model, ctx.table, &ctx.model.initial.displacements);
        }
        let (load, warnings) = load_at(ctx, step, 0.0, start_total, period)?;
        for warning in warnings {
            progress.event(ProgressEvent::Warning(warning));
        }
        let at_start = assemble_nonlinear(
            ctx.table,
            ctx.model,
            &input(step, &motion.u, initial, start_total),
            ctx.state,
            None,
        );
        let mut internal = at_start.f_int;
        NonlinearStatic::add_discrete(&setup.discrete, &motion.u, &mut setup.k, &mut internal);
        // The contact forces at the start (a body resting on another).
        let mut contact_states = ctx.state.contact.clone();
        let (contact_start, contact_normal_start, contact_energy_start, evaluation) = if has_contact
        {
            ctx.contact.prepare(&ctx.model.nodes.coordinates, &motion.u);
            let evaluation =
                evaluate_contact(ctx, &motion.u, &motion.v, &contact_states, false, 0.0, 0.0);
            contact_states = evaluation.states();
            let springs = contact_springs(&evaluation, n, 0.0, None);
            (springs.0, springs.2, springs.3, Some(evaluation))
        } else {
            (vec![0.0; n], vec![0.0; n], 0.0, None)
        };

        // The initial velocities are damped with the initial stiffness (`ḟ = K₀ v`).
        let cv_start = damping(&setup, &setup.k0, &motion.v);
        // Initial accelerations from the equilibrium at the start, with the matrix the
        // linear procedure takes (a tenth of the first increment keeps it regular where the
        // mass alone is singular).
        {
            let r = combine(
                &[
                    (1.0, &load),
                    (-1.0, &internal),
                    (-1.0, &contact_start),
                    (-1.0, &cv_start),
                ],
                n,
            );
            let (c_m, c_k) = matrix_factors(scheme, Rayleigh::default(), initial / 10.0, true);
            let matrix =
                initial_acceleration_matrix(ctx.table, ctx.model, &ctx.pattern, c_m, c_k, true)?;
            let set = constraint_set(ctx, &setup, &motion.u);
            let reduced = set.reduce(&matrix);
            let coordinates = || Unknowns::from_free(&set.free, 3, &ctx.model.nodes.coordinates);
            equations
                .prepare(&mut ctx.solver, &reduced, coordinates, true, progress)
                .map_err(|e| numerical(e, ctx.step_number, 0, &equations))?;
            let mut a = set.reduce_rhs(&r);
            equations
                .solve(
                    &mut ctx.solver,
                    &mut a,
                    &mut FirstOrWarnings {
                        inner: &mut *progress,
                        all: false,
                    },
                )
                .map_err(|e| in_increment(e, ctx.step_number, 0))?;
            motion.a = set.expand(&a, &vec![0.0; n]);
        }

        let kinetic = 0.5 * dot(&motion.v, &multiply(&setup.m, &motion.v));
        let mut state = At {
            time: 0.0,
            internal: internal.clone(),
            damping: cv_start,
            energies: Energies {
                internal: 0.5 * dot(&motion.u, &internal),
                kinetic,
                external_work: 0.0,
                dissipation: 0.0,
            },
            motion,
            load,
            contact: contact_start,
            contact_damping: vec![0.0; n],
            contact_normal: contact_normal_start,
            contact_energy: contact_energy_start,
            contact_states,
            evaluation,
            stresses: ctx.state.stresses.clone(),
            strains: ctx.state.strains.clone(),
            material: ctx.state.material.clone(),
        };
        if state.stresses.values.is_empty() {
            state.stresses = ctx.table.layout.field(6);
            state.strains = ctx.table.layout.field(6);
        }

        let carried = match &ctx.state.energy {
            Some(e) => Some(e.clone()),
            None if ctx.state.steps_done > 0 && !ctx.state.stresses.values.is_empty() => Some(
                internal_energy_density(&ctx.state.stresses, &ctx.state.strains),
            ),
            None => None,
        };
        let carried_nodal = carried
            .as_ref()
            .map(|e| nodal_average_scalar(ctx.table, ctx.model, e));
        let requests = energy_requests(step);

        let maximum = increments
            .maximum
            .filter(|m| *m > 0.0)
            .unwrap_or(f64::INFINITY);
        let mut size = initial;
        let mut parts = 1u32;
        let mut increment = 0u32;
        while state.time < period * (1.0 - 1e-12) {
            if ctx.cancel.is_cancelled() {
                return Ok(StepOutcome::Cancelled);
            }
            increment += 1;
            if increment > ctx_max_increments(step) {
                return Err(Error::Numerical {
                    step: ctx.step_number,
                    increment,
                    failure: plxs_core::Failure::IncrementLimit,
                    message: format!(
                        "*ERROR: increment {increment} exceeds the maximum number of increments ({}) of *STEP, INC=",
                        ctx_max_increments(step)
                    ),
                });
            }
            if !increments.direct && increment > 2 {
                size = (size * 1.5).min(maximum);
            }
            let (last, t1) = if increments.direct {
                let end = increment as f64 * size;
                let last = end >= period * (1.0 - 1e-9);
                (last, if last { period } else { end })
            } else {
                let last = size >= (period - state.time) * (1.0 - 1e-9);
                (last, if last { period } else { state.time + size })
            };
            let total = start_total + t1;
            progress.event(ProgressEvent::Increment {
                step: ctx.step_number,
                increment,
                attempt: 1,
                size: t1 - state.time,
                step_time: t1,
                total_time: total,
            });
            // The increment in `parts` equal sub-increments; computed again in more parts
            // when one of them is too coarse for the motion.
            let from_time = state.time;
            let (next, iterations) = loop {
                let mut current = state.clone();
                let mut iterations = 0u32;
                let mut slowest = 0u32;
                let mut largest_error = 0.0_f64;
                let mut failure = None;
                let mut too_coarse = false;
                for part in 1..=parts {
                    let end = if part == parts {
                        t1
                    } else {
                        from_time + (t1 - from_time) * f64::from(part) / f64::from(parts)
                    };
                    match advance(
                        ctx,
                        &mut setup,
                        &mut equations,
                        progress,
                        &current,
                        end,
                        increment,
                        (0, 0),
                    )? {
                        Ok((next, its)) => {
                            let error = local_error(scheme, &setup.active, &current, &next);
                            largest_error = largest_error.max(error);
                            if error > ACCURACY && parts < ACCURACY_PARTS {
                                too_coarse = true;
                                break;
                            }
                            current = next;
                            iterations += its;
                            slowest = slowest.max(its);
                        }
                        Err(failed) => {
                            failure = Some(failed);
                            break;
                        }
                    }
                }
                if too_coarse {
                    parts *= 2;
                    progress.event(ProgressEvent::Info(format!(
                        "the motion needs a shorter time step: increment {increment} is computed in {parts} parts"
                    )));
                    continue;
                }
                if let Some(failed) = failure {
                    return Ok(StepOutcome::Diverged {
                        increment,
                        failure: plxs_core::Failure::NoConvergence(Convergence {
                            cause: failed.cause,
                            stop: Stop::Cutbacks,
                            contact: setup.has_contact,
                            contact_changing: failed.contact_changing,
                        }),
                        message: format!(
                            "{}; no convergence even with the increment split into {} parts",
                            failed.reason,
                            parts << MAX_SPLITS
                        ),
                    });
                }
                if slowest > SLOW_ITERATIONS && parts < 1 << MAX_SPLITS {
                    parts *= 2;
                    progress.event(ProgressEvent::Info(format!(
                        "slow convergence: the next increments are computed in {parts} parts"
                    )));
                } else if slowest <= FAST_ITERATIONS && largest_error < ACCURACY / 8.0 && parts > 1
                {
                    parts /= 2;
                }
                break (current, iterations);
            };
            let dt = t1 - state.time;
            state = next;

            if let Some(evaluation) = &state.evaluation {
                ctx.state.contact.clone_from(&state.contact_states);
                ctx.contact.last = Some(evaluation.clone());
            }
            // Outputs: the internal and contact forces carried to the reference points.
            let mut reactions = state.internal.clone();
            if setup.has_contact {
                for (r, c) in reactions.iter_mut().zip(&state.contact) {
                    *r += c;
                }
            }
            let constraints = rotated(ctx.constraints, &setup, &state.motion.u);
            constraints
                .as_ref()
                .unwrap_or(ctx.constraints)
                .transfer_forces(&mut reactions, &setup.spc_dofs, &setup.active);
            if !requests.is_empty() {
                write_energies(
                    ctx.results,
                    &requests,
                    &state.energies,
                    (ctx.step_number, increment, total),
                );
            }
            if crate::static_linear::writes_increment(
                step,
                increment,
                if last { increment } else { u32::MAX },
            ) {
                write_increment(
                    ctx,
                    step,
                    increment,
                    total,
                    &state.motion,
                    &reactions,
                    (&state.stresses, &state.strains),
                    carried_nodal.as_deref(),
                    Some(&state.material),
                );
            }
            crate::history::write_history_with_velocities(
                ctx.results,
                ctx.model,
                ctx.table,
                step,
                (ctx.step_number, increment, total),
                (&state.motion.u, Some(&state.motion.v)),
                &reactions,
                &state.stresses,
                &state.strains,
                Some(&state.material),
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
            ctx.sta.push(StaRow {
                step: ctx.step_number,
                increment,
                attempt: 1,
                iterations,
                total_time: total,
                step_time: t1,
                increment_size: dt,
            });
            progress.event(ProgressEvent::IncrementDone {
                step: ctx.step_number,
                increment,
                attempt: 1,
                iterations,
                total_time: total,
                step_time: t1,
                size: dt,
            });
        }

        let mut energy = internal_energy_density(&state.stresses, &state.strains);
        if let Some(c) = &carried {
            for (e, c) in energy.values.iter_mut().zip(&c.values) {
                *e += c;
            }
        }
        ctx.state.energy = Some(energy);
        ctx.state.displacements = NodalField {
            per_node: 3,
            values: state.motion.u,
        };
        ctx.state.velocities = Some(NodalField {
            per_node: 3,
            values: state.motion.v,
        });
        ctx.state.accelerations = Some(NodalField {
            per_node: 3,
            values: state.motion.a,
        });
        ctx.state.loads = None;
        ctx.state.stresses = state.stresses;
        ctx.state.strains = state.strains;
        ctx.state.material = state.material;
        ctx.state.total_time = start_total + period;
        ctx.state.steps_done = ctx.step_number;
        Ok(StepOutcome::Converged)
    }
}

/// The local error of the step from `from` to `to` relative to the largest displacement of
/// the step: `|β − 1/6| Δt² max|a₁ − a₀| / max|u₁ − u₀|` (Zienkiewicz and Xie, 1991), over
/// the DOFs of elements (`active`; not the rotations of rigid bodies, which have other
/// units).
fn local_error(scheme: Scheme, active: &[bool], from: &At, to: &At) -> f64 {
    let dt = to.time - from.time;
    let largest = |a: &[f64], b: &[f64]| {
        a.iter()
            .zip(b)
            .zip(active)
            .filter(|(_, active)| **active)
            .map(|((x, y), _)| (x - y).abs())
            .fold(0.0_f64, f64::max)
    };
    let motion = largest(&to.motion.u, &from.motion.u);
    if motion == 0.0 {
        return 0.0;
    }
    (scheme.beta - 1.0 / 6.0).abs() * dt * dt * largest(&to.motion.a, &from.motion.a) / motion
}

fn input<'a>(step: &Step, u: &'a [f64], dt: f64, time: f64) -> NonlinearInput<'a> {
    NonlinearInput {
        u,
        nlgeom: step.nlgeom,
        dt,
        time,
        creep_active: false,
        temperature: 0.0,
        follower: &[],
    }
}

/// Damping of the contact springs, as a fraction of the time step: `c = η Δt dp/dd` (see
/// [`plxs_contact::Damping`]). A slave node on a stiff penalty spring is a small mass on a
/// stiff spring: with a consistent mass matrix it bounces off the master at the frequency
/// of the spring, a few time steps per period, and the contact force chatters between zero
/// and several times its mean (the impact of a bar on a wall). Numerical damping of the time
/// integration hardly reaches this frequency. `η = 1` damps it about critically
/// (`ζ ≈ ω Δt / 2`) and leaves motions much slower than the time step alone (`ζ ≪ 1`), like
/// the waves of the impact.
const CONTACT_DAMPING: f64 = 1.0;

/// The springs of a contact evaluation in the tangent `k` (scaled by `scale`): their
/// forces over all DOFs, separately the part of them from their damping and that from the
/// normal springs alone, and the energy the normal springs store.
fn contact_springs(
    evaluation: &Evaluation,
    n: usize,
    scale: f64,
    mut k: Option<&mut SparseMatrix>,
) -> (Vec<f64>, Vec<f64>, Vec<f64>, f64) {
    let mut f = vec![0.0; n];
    let mut fd = vec![0.0; n];
    let mut normal = vec![0.0; n];
    let mut block = Vec::new();
    for element in &evaluation.elements {
        if let Some(k) = k.as_deref_mut() {
            block.clear();
            block.extend(element.ke.iter().map(|k| k * scale));
            k.add_block(&element.dofs, &block);
        }
        for (a, &dof) in element.dofs.iter().enumerate() {
            f[dof as usize] += element.fe[a];
        }
        for (&dof, d) in element.dofs.iter().zip(&element.damping) {
            fd[dof as usize] += d;
        }
        for (&dof, d) in element.dofs.iter().zip(&element.normal) {
            normal[dof as usize] += d;
        }
    }
    let energy = evaluation
        .points
        .iter()
        .flatten()
        .map(|p| p.normal_energy)
        .sum();
    (f, fd, normal, energy)
}

/// The contact springs at `u` from the states `start`, damped with the velocities `v`
/// (`damping` the coefficient as a time, `rate` = dv/du).
fn evaluate_contact(
    ctx: &mut ProcedureContext<'_>,
    u: &[f64],
    v: &[f64],
    start: &[Vec<ContactPointState>],
    predictor: bool,
    damping: f64,
    rate: f64,
) -> Evaluation {
    ctx.contact.damping = Some(plxs_contact::Damping {
        velocities: v.to_vec(),
        coefficient: damping,
        rate,
    });
    let evaluation = ctx
        .contact
        .evaluate(&ctx.model.nodes.coordinates, u, start, predictor);
    ctx.contact.damping = None;
    evaluation
}

/// The diagonal of the bounding box of the nodes.
fn model_size(coordinates: &[[f64; 3]]) -> f64 {
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for c in coordinates {
        for d in 0..3 {
            low[d] = low[d].min(c[d]);
            high[d] = high[d].max(c[d]);
        }
    }
    (0..3)
        .map(|d| (high[d] - low[d]).max(0.0).powi(2))
        .sum::<f64>()
        .sqrt()
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// The damping forces `C v` with the stiffness `k`: at the start of the step.
fn damping(setup: &Setup<'_>, k: &SparseMatrix, x: &[f64]) -> Vec<f64> {
    let Rayleigh { alpha, beta } = setup.rayleigh;
    if alpha == 0.0 && beta == 0.0 {
        return vec![0.0; x.len()];
    }
    let mx = multiply(&setup.m, x);
    let kx = multiply(k, x);
    combine(&[(alpha, &mx), (beta, &kx)], x.len())
}

/// The external loads at step time `time`.
fn load_at(
    ctx: &ProcedureContext<'_>,
    step: &Step,
    time: f64,
    start_total: f64,
    period: f64,
) -> Result<(Vec<f64>, Vec<String>)> {
    let total = start_total + time;
    let (mut f, warnings) = load_vector(ctx.model, ctx.table, step, time, total, period)?;
    rotate_concentrated_loads(
        ctx.model,
        ctx.constraints,
        &ctx.table.dofs,
        step,
        time,
        total,
        period,
        &mut f,
    );
    Ok((f, warnings))
}

/// The prescribed part `g` of the displacements at step time `time`.
fn prescribed(ctx: &ProcedureContext<'_>, setup: &Setup<'_>, time: f64) -> Result<Vec<f64>> {
    let mut spc = Spc::of_step(
        ctx.model,
        setup.step,
        &ctx.table.dofs,
        time,
        setup.start_total + time,
        setup.period,
    )?;
    spc.apply_fixed(
        ctx.model,
        setup.step,
        &ctx.table.dofs,
        &setup.previous,
        &|node| ctx.constraints.rotation(node).copied(),
    );
    Ok(setup.set.particular(&spc.values))
}

/// The constraints linearized at `u` when rigid bodies rotate finitely.
fn rotated(
    constraints: &ModelConstraints,
    setup: &Setup<'_>,
    u: &[f64],
) -> Option<ModelConstraints> {
    setup.large_rotations.then(|| constraints.at_rotations(u))
}

/// The constraint set of an iteration at `u`.
fn constraint_set(ctx: &ProcedureContext<'_>, setup: &Setup<'_>, u: &[f64]) -> ConstraintSet {
    match rotated(ctx.constraints, setup, u) {
        Some(constraints) => {
            constraints.step_set(ctx.model.nodes.len(), &setup.active, &setup.spc_dofs)
        }
        None => setup.set.clone(),
    }
}

/// Integrates from `from` to step time `t1`; a failed attempt is split into two halves
/// (`cuts` such splits so far, at most [`MAX_SPLITS`]), and so is an attempt in which a
/// contact closes or opens too abruptly (`depth` splits of any kind so far, at most
/// [`MAX_IMPACT_SPLITS`]). Returns the state at `t1` and the iterations it took.
#[allow(clippy::too_many_arguments)]
fn advance(
    ctx: &mut ProcedureContext<'_>,
    setup: &mut Setup<'_>,
    equations: &mut Equations,
    progress: &mut dyn Progress,
    from: &At,
    t1: f64,
    increment: u32,
    (depth, cuts): (u32, u32),
) -> Result<std::result::Result<(At, u32), Failed>> {
    let (reason, cut) = match attempt(ctx, setup, equations, progress, from, t1, increment, depth)?
    {
        Ok((done, its, impact)) if impact <= IMPACT_ENERGY || depth >= MAX_IMPACT_SPLITS => {
            return Ok(Ok((done, its)));
        }
        Ok(_) => (None, 0),
        Err(failed) if cuts >= MAX_SPLITS => return Ok(Err(failed)),
        Err(failed) => (Some(failed.reason), 1),
    };
    // An impact is refined silently (it may take a dozen halvings).
    if let Some(reason) = reason {
        progress.event(ProgressEvent::Cutback {
            step: ctx.step_number,
            increment,
            attempt: depth + 1,
            reason: format!("{reason}; the increment is split in two"),
        });
    }
    let middle = 0.5 * (from.time + t1);
    let next = (depth + 1, cuts + cut);
    let (half, first) = match advance(
        ctx, setup, equations, progress, from, middle, increment, next,
    )? {
        Ok(done) => done,
        Err(failed) => return Ok(Err(failed)),
    };
    match advance(ctx, setup, equations, progress, &half, t1, increment, next)? {
        Ok((end, second)) => Ok(Ok((end, first + second))),
        Err(failed) => Ok(Err(failed)),
    }
}

/// Newton iterations for the state at `t1` from `from`.
#[allow(clippy::too_many_arguments)]
fn attempt(
    ctx: &mut ProcedureContext<'_>,
    setup: &mut Setup<'_>,
    equations: &mut Equations,
    progress: &mut dyn Progress,
    from: &At,
    t1: f64,
    increment: u32,
    depth: u32,
) -> Result<std::result::Result<(At, u32, f64), Failed>> {
    let n = ctx.table.dofs.len();
    let dt = t1 - from.time;
    let total = setup.start_total + t1;
    let Scheme { alpha, beta, gamma } = setup.scheme;
    let c1 = 1.0 / (beta * dt * dt);
    let step = setup.step;
    let (load, _) = load_at(ctx, step, t1, setup.start_total, setup.period)?;

    let m0 = &from.motion;
    let u_tilde = combine(
        &[(1.0, &m0.u), (dt, &m0.v), (dt * dt * (0.5 - beta), &m0.a)],
        n,
    );
    let v_tilde = combine(&[(1.0, &m0.v), (dt * (1.0 - gamma), &m0.a)], n);
    // The part of the residual that does not change in the iterations.
    let cv0 = &from.damping;
    let mut fixed = combine(
        &[
            (1.0 + alpha, &load),
            (-alpha, &from.load),
            (alpha, cv0),
            (alpha, &from.internal),
        ],
        n,
    );
    if setup.has_contact {
        for (f, c) in fixed.iter_mut().zip(&from.contact) {
            *f += alpha * c;
        }
        // Candidate master faces from the start of the sub-increment and a matrix pattern
        // with every spring they allow (kept while it does, see `contact::cached_pattern`).
        let blocks = ctx
            .contact
            .prepare(&ctx.model.nodes.coordinates, &from.motion.u);
        let pattern = crate::contact::cached_pattern(ctx.table, &blocks, &mut ctx.contact_pattern);
        if !std::sync::Arc::ptr_eq(setup.k.pattern(), &pattern) {
            setup.k = SparseMatrix::zeros(pattern.clone());
            setup.m_contact = Some(setup.m.embed(pattern));
        }
    }
    let (c_m, _) = matrix_factors(setup.scheme, setup.rayleigh, dt, false);
    // Damping and mass part of the tangent (c_m); the stiffness part is (1+α) K_T plus the
    // stiffness-proportional damping (1+α) β_R K_T/Δt.
    let Rayleigh {
        alpha: alpha_r,
        beta: beta_r,
    } = setup.rayleigh;

    // From the start of the increment; the change of the prescribed values enters the
    // first iteration as a linear prediction (as in the static procedure).
    let g0 = prescribed(ctx, setup, from.time)?;
    let g1 = prescribed(ctx, setup, t1)?;
    let delta_g: Vec<f64> = g1.iter().zip(&g0).map(|(e, s)| e - s).collect();
    let mut pending_g = delta_g.iter().any(|d| *d != 0.0);
    // The material state the increment starts from.
    ctx.state.material.clone_from(&from.material);
    let mut u = m0.u.clone();
    // With contact the iterations start from the predictor `ũ` (the motion continued at the
    // velocity and acceleration of the start): a body that slides or flies on is then
    // evaluated where it will be, not held by the friction or the contact status of where it
    // was. Prescribed DOFs keep their start; they move with the first correction.
    if setup.has_contact {
        u.clone_from(&u_tilde);
        for &dof in &setup.spc_dofs {
            u[dof as usize] = m0.u[dof as usize];
        }
        if setup.large_rotations {
            ctx.constraints.project_rigid(&mut u, &setup.spc_dofs);
        }
    }
    let mut last_correction = f64::INFINITY;
    let mut iteration = 0u32;
    // The contact status of the last iteration and the last iteration in which it changed.
    let mut status: Option<Vec<u8>> = None;
    let mut last_change: Option<u32> = None;
    loop {
        let started = std::time::Instant::now();
        let mut output = assemble_nonlinear(
            ctx.table,
            ctx.model,
            &input(step, &u, dt, total),
            ctx.state,
            Some(&mut setup.k),
        );
        let gaps: Vec<u8> =
            NonlinearStatic::add_discrete(&setup.discrete, &u, &mut setup.k, &mut output.f_int)
                .into_iter()
                .map(u8::from)
                .collect();
        // Contact springs at `u` from the states at the start of the sub-increment. Their
        // stiffness enters the tangent with (1 + α) only (no stiffness-proportional
        // damping), which the scaling undoes for the factor of `setup.k` below.
        let a1 = combine(&[(c1, &u), (-c1, &u_tilde)], n);
        let v1 = combine(&[(1.0, &v_tilde), (gamma * dt, &a1)], n);
        // The springs are damped (see `CONTACT_DAMPING`) with the velocities of `u`.
        let evaluation = setup.has_contact.then(|| {
            evaluate_contact(
                ctx,
                &u,
                &v1,
                &from.contact_states,
                iteration == 0,
                CONTACT_DAMPING * dt,
                gamma / (beta * dt),
            )
        });
        let (fc, fd, fn1, energy1) = match &evaluation {
            Some(evaluation) => {
                contact_springs(evaluation, n, 1.0 / (1.0 + beta_r / dt), Some(&mut setup.k))
            }
            None => (Vec::new(), Vec::new(), Vec::new(), 0.0),
        };
        let mut key = gaps;
        if let Some(evaluation) = &evaluation {
            key.extend(evaluation.status_key());
        }
        let changed = status.as_ref().is_some_and(|s| *s != key);
        status = Some(key);
        if changed {
            last_change = Some(iteration);
        }
        let changing = last_change.is_some_and(|i| iteration - i <= 2);
        ctx.profile.add(crate::profile::Phase::Assembly, started);
        let ma = multiply(&setup.m, &a1);
        let cv1 = if alpha_r == 0.0 && beta_r == 0.0 {
            vec![0.0; n]
        } else {
            let mv = multiply(&setup.m, &v1);
            combine(
                &[
                    (alpha_r, &mv),
                    (beta_r / dt, &output.f_int),
                    (-beta_r / dt, &from.internal),
                ],
                n,
            )
        };
        // The tangent: c_m M + (1+α)(1 + β_R/Δt) K_T.
        let tangent = SparseMatrix::linear_combination(
            setup.m_contact.as_ref().unwrap_or(&setup.m),
            c_m,
            &setup.k,
            (1.0 + alpha) * (1.0 + beta_r / dt),
        );
        let mut residual_full = combine(
            &[
                (1.0, &fixed),
                (-1.0, &ma),
                (-(1.0 + alpha), &cv1),
                (-(1.0 + alpha), &output.f_int),
            ],
            n,
        );
        for (r, c) in residual_full.iter_mut().zip(&fc) {
            *r -= (1.0 + alpha) * c;
        }
        if pending_g {
            let td = multiply(&tangent, &delta_g);
            for (r, t) in residual_full.iter_mut().zip(&td) {
                *r -= t;
            }
        }
        let set = constraint_set(ctx, setup, &u);
        let residual = set.reduce_rhs(&residual_full);
        let r_max = residual.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        let largest = |x: &[f64]| x.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        let flux = if output.flux_count > 0 {
            output.flux_sum / output.flux_count as f64
        } else {
            0.0
        };
        // A body in free flight has none of these forces: the round-off of its inertial
        // forces against the momentum it carries (`M v / Δt`) is the floor.
        let momentum = 1e-8 * largest(&multiply(&setup.m, &v1)) / dt;
        let scale = flux
            .max(largest(&load))
            .max(largest(&ma))
            .max(largest(&fc))
            .max(momentum);
        let motion = u
            .iter()
            .zip(&m0.u)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0_f64, f64::max);
        if iteration > 0 {
            ctx.cvg.push(CvgRow {
                step: ctx.step_number,
                increment,
                attempt: depth + 1,
                iteration,
                contact_elements: evaluation.as_ref().map_or(0, |e| e.active as u32),
                residual: r_max,
                correction: last_correction,
                step_time: t1,
                increment_size: dt,
            });
            let residual_ok = r_max <= RESIDUAL * scale || r_max == 0.0;
            // A body at rest moves by nothing in an increment: a correction at the round-off
            // of the model's size is converged as well.
            let correction_ok = last_correction <= CORRECTION * motion
                || last_correction <= 1e-13 * setup.size
                || motion == 0.0
                || r_max <= 1e-10 * scale;
            // An iteration that opened or closed a contact point needs another one.
            let converged = residual_ok && correction_ok && !changed;
            progress.event(ProgressEvent::Iteration {
                step: ctx.step_number,
                increment,
                iteration,
                residual: r_max,
                correction: last_correction,
                converged,
            });
            if converged {
                let du = combine(&[(1.0, &u), (-1.0, &m0.u)], n);
                let mut energies = from.energies.clone();
                let mean_internal = combine(&[(0.5, &from.internal), (0.5, &output.f_int)], n);
                let mean_load = combine(&[(0.5, &from.load), (0.5, &load)], n);
                energies.internal += dot(&mean_internal, &du);
                energies.external_work += dot(&mean_load, &du);
                let kinetic = if setup.energies_wanted || setup.has_contact {
                    0.5 * dot(&v1, &multiply(&setup.m, &v1))
                } else {
                    0.0
                };
                // The error of the contact's energy: the work of the normal springs over the
                // increment by the trapezoidal rule of the time integration against the
                // energy they store. A point that closes or opens within a coarse increment
                // makes it large (an impact computed with steps longer than it lasts gives
                // energy), and the increment is computed in parts.
                let mut impact = 0.0;
                if setup.has_contact {
                    // The springs store energy, their damping dissipates it, the friction
                    // takes the rest.
                    let mean_damping = combine(&[(0.5, &from.contact_damping), (0.5, &fd)], n);
                    let mean_normal = combine(&[(0.5, &from.contact_normal), (0.5, &fn1)], n);
                    let mean_other = combine(
                        &[
                            (0.5, &from.contact),
                            (0.5, &fc),
                            (-1.0, &mean_damping),
                            (-1.0, &mean_normal),
                        ],
                        n,
                    );
                    let stored = energy1 - from.contact_energy;
                    energies.internal += stored + dot(&mean_other, &du);
                    energies.dissipation += dot(&mean_damping, &du);
                    let error = (dot(&mean_normal, &du) - stored).abs();
                    let reference = from
                        .energies
                        .kinetic
                        .max(kinetic)
                        .max(energies.external_work.abs())
                        .max(energy1)
                        .max(from.contact_energy);
                    impact = if reference > 0.0 {
                        error / reference
                    } else {
                        0.0
                    };
                }
                if setup.energies_wanted || setup.has_contact {
                    energies.kinetic = kinetic;
                }
                if setup.energies_wanted {
                    energies.dissipation += 0.5 * dt * (dot(&m0.v, cv0) + dot(&v1, &cv1));
                }
                return Ok(Ok((
                    At {
                        time: t1,
                        motion: Motion { u, v: v1, a: a1 },
                        load,
                        internal: output.f_int,
                        damping: cv1,
                        stresses: output.stresses,
                        strains: output.strains,
                        material: output.material,
                        energies,
                        contact_states: evaluation
                            .as_ref()
                            .map_or_else(|| from.contact_states.clone(), Evaluation::states),
                        contact: if setup.has_contact { fc } else { vec![0.0; n] },
                        contact_damping: if setup.has_contact { fd } else { vec![0.0; n] },
                        contact_normal: if setup.has_contact { fn1 } else { vec![0.0; n] },
                        contact_energy: energy1,
                        evaluation,
                    },
                    iteration,
                    impact,
                )));
            }
            if iteration >= MAX_ITERATIONS {
                return Ok(Err(Failed {
                    cause: Divergence::IterationLimit,
                    reason: format!("no convergence after {MAX_ITERATIONS} iterations"),
                    contact_changing: changing,
                }));
            }
        }

        // Solve Tᵀ S T Δv = Tᵀ r.
        iteration += 1;
        let started = std::time::Instant::now();
        let reduced = set.reduce(&tangent);
        let coordinates = || Unknowns::from_free(&set.free, 3, &ctx.model.nodes.coordinates);
        let mut sink = FirstOrWarnings {
            inner: &mut *progress,
            all: increment == 1 && depth == 0 && iteration == 1,
        };
        equations
            .prepare(&mut ctx.solver, &reduced, coordinates, true, &mut sink)
            .map_err(|e| numerical(e, ctx.step_number, increment, equations))?;
        ctx.profile
            .add(crate::profile::Phase::Factorization, started);
        let unsymmetric: Vec<&plxs_contact::ContactElement> = evaluation
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
            equations
                .solve(
                    &mut ctx.solver,
                    &mut dv,
                    &mut FirstOrWarnings {
                        inner: &mut *progress,
                        all: false,
                    },
                )
                .map_err(|e| in_increment(e, ctx.step_number, increment))?;
            dv
        } else {
            // Sliding friction makes the tangent unsymmetric: GMRES on the full tangent,
            // preconditioned by the factorized symmetric part.
            let zero = vec![0.0; n];
            let solver = &ctx.solver;
            let apply = |v: &[f64], out: &mut [f64]| {
                reduced.multiply(v, out);
                let full = set.expand(v, &zero);
                let mut e = vec![0.0; n];
                for element in &unsymmetric {
                    element.apply_unsymmetric(&full, &mut e);
                }
                for (o, r) in out.iter_mut().zip(set.reduce_rhs(&e)) {
                    *o += (1.0 + alpha) * r;
                }
            };
            let precondition = |v: &mut [f64]| equations.apply_inverse(solver, v);
            plxs_linalg::gmres(&apply, &precondition, &residual, 1e-10, 60, 300)?.0
        };
        let mut du = set.expand(&dv, &vec![0.0; n]);
        // A correction that drives contact points deep into the master (a body arriving
        // fast in a sub-increment it was still free in) is shortened, see
        // `ContactModel::step_limit`.
        if setup.has_contact {
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
                &from.contact_states,
                work,
            );
            if factor < 1.0 {
                for d in &mut du {
                    *d *= factor;
                }
            }
        }
        last_correction = du.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        if !last_correction.is_finite() {
            return Ok(Err(Failed {
                cause: Divergence::NotFinite,
                reason: "the solution correction is not finite".into(),
                contact_changing: changing,
            }));
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
        if setup.large_rotations {
            ctx.constraints.project_rigid(&mut u, &setup.spc_dofs);
        }
    }
}
