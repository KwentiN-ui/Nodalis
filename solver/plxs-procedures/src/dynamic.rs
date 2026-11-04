//! `*DYNAMIC`: direct time integration of `M a + C v + K u = f(t)` (concept section 4.5,
//! M3 T13).
//!
//! **Implicit** (default): the α-method of Hilber, Hughes and Taylor as the CalculiX manual
//! describes it (section "*DYNAMIC"): with `α ∈ [−1/3, 0]` (default −0.05),
//! `β = (1 − α)²/4`, `γ = 1/2 − α`, the equation of motion is enforced at the shifted time
//!
//! ```text
//! M a₁ + (1 + α)(C v₁ + K u₁) − α (C v₀ + K u₀) = (1 + α) f₁ − α f₀
//! ```
//!
//! with Newmark's updates `u₁ = u₀ + Δt v₀ + Δt²((1/2 − β) a₀ + β a₁)` and
//! `v₁ = v₀ + Δt((1 − γ) a₀ + γ a₁)`. The unknown is the displacement at the end of the
//! increment; prescribed displacements and multi-point constraints are eliminated with the
//! step's [`plxs_constraints::ConstraintSet`] (`u = T v + g`). The initial acceleration
//! comes from the equilibrium at the start of the step, the way CalculiX computes it (see
//! the comment in `integrate`). The mass matrix is consistent with the full integration rule
//! of each element ([`MassRule::Full`], C3D20R with 3×3×3 points), unlike the eigenvalue
//! procedures.
//!
//! **Increments**: `DIRECT` keeps the initial size; otherwise the size grows by 1.5 from the
//! third increment on up to the maximum size, as CalculiX does for a linear step.
//!
//! **Explicit** (`EXPLICIT`) and `NLGEOM` are refused as not implemented yet.
//!
//! **Damping**: Rayleigh `C = α_R M + β_R K` from `*DAMPING` in the step.
//!
//! **Initial conditions**: `*INITIAL CONDITIONS, TYPE=VELOCITY` (and `DISPLACEMENT`) set
//! the state before the first step that needs it; later steps start from the state the
//! previous step left (displacements, velocities, accelerations).
//!
//! Outputs: `U`, `V` (`VELO`), `S`, `E`, `RF` (internal forces `K u`), `ENER` (strain
//! energy density `½ σ:ε`), the stress error estimator, as the `.frd` fields of CalculiX in
//! its order.
//!
//! No CalculiX code was consulted.

use plxs_assembly::{
    ElementTable, MassRule, Spc, assemble_mass_with, assemble_stiffness, load_vector,
    rotate_concentrated_loads,
};
use plxs_core::mesh::NodeRef;
use plxs_core::results::{IncrementKind, ResultField, ResultsIncrement};
use plxs_core::state::{IntegrationPointField, NodalField};
use plxs_core::step::{Dynamic, Procedure as ProcedureKind};
use plxs_core::{Error, Model, Progress, ProgressEvent, Result, Step};
use plxs_linalg::SparseMatrix;
use plxs_output::StaRow;
use rayon::prelude::*;

use crate::equations::{Equations, FirstOrWarnings, in_increment};
use crate::fields::{build_fields, internal_energy_density, nodal_average_scalar, node_order};
use crate::static_linear::{integration_point_results, writes_increment};
use crate::{Procedure, ProcedureContext, StepOutcome};

pub struct DynamicProcedure;

/// Integration parameters of one increment size.
#[derive(Clone, Copy, Debug)]
struct Scheme {
    alpha: f64,
    beta: f64,
    gamma: f64,
}

impl Scheme {
    fn of(parameters: &Dynamic) -> Scheme {
        Scheme::new(parameters.alpha.unwrap_or(-0.05))
    }

    fn new(alpha: f64) -> Scheme {
        let alpha = alpha.clamp(-1.0 / 3.0, 0.0);
        Scheme {
            alpha,
            beta: (1.0 - alpha).powi(2) / 4.0,
            gamma: 0.5 - alpha,
        }
    }
}

/// The state of the time integration at the start of an increment.
pub(crate) struct Motion {
    pub(crate) u: Vec<f64>,
    pub(crate) v: Vec<f64>,
    pub(crate) a: Vec<f64>,
}

/// Values given per node (or node set) and DOF, as `*INITIAL CONDITIONS` lists them.
fn initial_values(model: &Model, table: &ElementTable, entries: &[(NodeRef, u8, f64)]) -> Vec<f64> {
    let mut out = vec![0.0; table.dofs.len()];
    for (region, dof, value) in entries {
        let nodes: Vec<u32> = match region {
            NodeRef::Node(id) => vec![*id],
            NodeRef::Set(name) => model.sets.node_sets.get(name).cloned().unwrap_or_default(),
        };
        if !(1..=6).contains(dof) {
            continue;
        }
        for id in nodes {
            if let Some(index) = model.nodes.index(id)
                && let Some(d) = table.dofs.dof(index, *dof as usize - 1)
            {
                out[d as usize] = *value;
            }
        }
    }
    out
}

/// The linear combination `Σ cₖ xₖ` of vectors of length `n`.
pub(crate) fn combine(terms: &[(f64, &[f64])], n: usize) -> Vec<f64> {
    let mut out = vec![0.0; n];
    for (c, x) in terms {
        if *c == 0.0 {
            continue;
        }
        for (o, v) in out.iter_mut().zip(x.iter()) {
            *o += c * v;
        }
    }
    out
}

pub(crate) fn multiply(m: &SparseMatrix, x: &[f64]) -> Vec<f64> {
    let mut y = vec![0.0; x.len()];
    m.multiply(x, &mut y);
    y
}

impl Procedure for DynamicProcedure {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        let ProcedureKind::Dynamic(parameters) = &step.procedure else {
            return Err(Error::Unsupported(
                "DynamicProcedure needs a *DYNAMIC step".into(),
            ));
        };
        if parameters.explicit {
            // ccx 2.21's explicit scheme updates the displacements with the trapezoidal rule
            // of the velocities and is not the β = 0 member of the α-method; it is not
            // identified yet (single-DOF oracles in the M3 T13 notes).
            return Err(Error::Unsupported(format!(
                "procedure not implemented yet: *DYNAMIC, EXPLICIT (step {})",
                step.name
            )));
        }
        if step.nlgeom {
            return Err(Error::Unsupported(format!(
                "procedure not implemented yet: *DYNAMIC, NLGEOM (step {})",
                step.name
            )));
        }
        if parameters.increments.period() <= 0.0 {
            return Err(Error::input("*DYNAMIC", "the step period must be positive"));
        }
        if parameters.increments.initial() <= 0.0 {
            return Err(Error::input(
                "*DYNAMIC",
                "the initial increment must be positive",
            ));
        }
        let scheme = Scheme::of(parameters);
        let rayleigh = parameters.rayleigh.unwrap_or_default();

        // Matrices: K once per model (cached), M consistent with the full rule.
        let started = std::time::Instant::now();
        if ctx.stiffness.is_none() {
            let mut k = SparseMatrix::zeros(ctx.pattern.clone());
            ctx.table.require_elastic()?;
            assemble_stiffness(ctx.table, ctx.model, &mut k);
            ctx.stiffness = Some(k);
        }
        let k = ctx.stiffness.take().unwrap();
        let mut m = SparseMatrix::zeros(ctx.pattern.clone());
        assemble_mass_with(ctx.table, ctx.model, MassRule::Full, &mut m)?;
        progress.event(ProgressEvent::Info(format!(
            "stiffness and mass assembled in {:.2} s",
            started.elapsed().as_secs_f64()
        )));
        let outcome = self.integrate(
            ctx,
            step,
            progress,
            &k,
            &m,
            scheme,
            rayleigh,
            &parameters.increments,
            parameters.solver,
        );
        ctx.stiffness = Some(k);
        outcome
    }
}

impl DynamicProcedure {
    #[allow(clippy::too_many_arguments)]
    fn integrate(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
        k: &SparseMatrix,
        m: &SparseMatrix,
        scheme: Scheme,
        rayleigh: plxs_core::step::Rayleigh,
        increments: &plxs_core::step::Incrementation,
        solver: plxs_core::step::EquationSolver,
    ) -> Result<StepOutcome> {
        let n = ctx.table.dofs.len();
        // `K + c M` (and the matrix of the initial accelerations) is positive definite: the
        // iterative solver can take the step's systems.
        let mut equations = Equations::new(solver);
        let period = increments.period();
        let initial = increments.initial().min(period);
        let maximum = increments
            .maximum
            .filter(|m| *m > 0.0)
            .unwrap_or(f64::INFINITY);
        let start_total = ctx.state.total_time;
        let Scheme { alpha, beta, gamma } = scheme;
        let damping = |x: &[f64]| -> Vec<f64> {
            if rayleigh.alpha == 0.0 && rayleigh.beta == 0.0 {
                return vec![0.0; x.len()];
            }
            let mx = multiply(m, x);
            let kx = multiply(k, x);
            combine(&[(rayleigh.alpha, &mx), (rayleigh.beta, &kx)], x.len())
        };

        // Constraints: the DOFs prescribed in this step do not change; their values may.
        let spc0 = Spc::of_step(ctx.model, step, &ctx.table.dofs, 0.0, start_total, period)?;
        let active = ctx.table.active_dofs();
        let set = ctx
            .constraints
            .step_set(ctx.model.nodes.len(), &active, &spc0.dofs);
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
        let previous = ctx.state.displacements.values.clone();
        let spc_at = |time: f64| -> Result<Vec<f64>> {
            let mut spc = Spc::of_step(
                ctx.model,
                step,
                &ctx.table.dofs,
                time,
                start_total + time,
                period,
            )?;
            spc.apply_fixed(ctx.model, step, &ctx.table.dofs, &previous, &|node| {
                ctx.constraints.rotation(node).copied()
            });
            Ok(set.particular(&spc.values))
        };
        let load_at = |time: f64| -> Result<(Vec<f64>, Vec<String>)> {
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
        };

        // The motion the step starts from.
        let fresh = ctx.state.velocities.is_none();
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
        if fresh && ctx.state.steps_done == 0 && !ctx.model.initial.displacements.is_empty() {
            motion.u = initial_values(ctx.model, ctx.table, &ctx.model.initial.displacements);
        }
        let (f0, warnings) = load_at(0.0)?;
        for warning in warnings {
            progress.event(ProgressEvent::Warning(warning));
        }
        // Initial acceleration from the equilibrium at the start of the step. CalculiX does
        // not solve `M a₀ = f₀ − C v₀ − K u₀` but takes the system matrix of a tenth of the
        // first increment in the acceleration form, `(M + (1+α)βh² K) a₀ =
        // f₀ − C v₀ − K u₀` with `h = Δt/10` but without the damping term, and in it the fully integrated stiffness and
        // mass of reduced elements (C3D20R as C3D20, C3D8R as C3D8). Inferred from single-DOF
        // runs of ccx 2.21 with several Δt and α (the factor of K is (1+α)β/100 to seven
        // digits) and from C3D8R/C3D20R oracles; no source consulted. This also keeps `a₀`
        // finite where the mass matrix alone is singular. Prescribed DOFs start without
        // acceleration.
        {
            let started = std::time::Instant::now();
            // The internal forces of the start state with the same full integration
            // (a static preload: ten times closer to ccx than with K u₀).
            let ku = if motion.u.iter().any(|v| *v != 0.0) {
                let k_full = initial_acceleration_matrix(
                    ctx.table,
                    ctx.model,
                    &ctx.pattern,
                    0.0,
                    1.0,
                    false,
                )?;
                multiply(&k_full, &motion.u)
            } else {
                vec![0.0; n]
            };
            let cv = damping(&motion.v);
            let r = combine(&[(1.0, &f0), (-1.0, &ku), (-1.0, &cv)], n);
            // Damping does not enter this matrix (single-DOF oracles with mass- and
            // stiffness-proportional damping).
            let undamped = plxs_core::step::Rayleigh::default();
            let (c_m, c_k) = matrix_factors(scheme, undamped, initial / 10.0, true);
            let a0_matrix =
                initial_acceleration_matrix(ctx.table, ctx.model, &ctx.pattern, c_m, c_k, true)?;
            let a_red = set.reduce(&a0_matrix);
            let coordinates = || {
                (set.free.iter())
                    .map(|&dof| ctx.model.nodes.coordinates[dof as usize / 3])
                    .collect()
            };
            equations
                .prepare(&mut ctx.solver, &a_red, coordinates, true, progress)
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
            progress.event(ProgressEvent::Info(format!(
                "initial accelerations in {:.2} s",
                started.elapsed().as_secs_f64()
            )));
        }

        // Increments: `DIRECT` keeps the initial size; otherwise CalculiX grows a linear
        // dynamic step's increment by 1.5 from the third increment on (each converges in
        // its two counted iterations), up to the maximum size, and cuts the last one to the
        // step end (observed with ccx 2.21 on the cantilever: 2.5e-5, 2.5e-5, 3.75e-5, ...).
        // CalculiX integrates the internal energy of a dynamic step from a zero state on
        // top of the energy the steps before left (observed with ccx 2.21: after a static
        // preload `ENER` is the static energy plus ½σ:ε of the dynamic state). Linear
        // elastic, that is the carried energy plus ½σ:ε.
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
        let mut factored_dt: Option<f64> = None;
        let mut f_prev = f0;
        let mut stresses = ctx.table.layout.field(6);
        let mut strains = ctx.table.layout.field(6);
        let mut reactions = vec![0.0; n];
        let mut time = 0.0;
        let mut size = initial;
        let mut increment = 0u32;
        while time < period * (1.0 - 1e-12) {
            if ctx.cancel.is_cancelled() {
                return Ok(StepOutcome::Cancelled);
            }
            increment += 1;
            if increment > ctx_max_increments(step) {
                return Err(Error::Numerical {
                    step: ctx.step_number,
                    increment,
                    message: format!(
                        "*ERROR: increment {increment} exceeds the maximum number of increments ({}) of *STEP, INC=",
                        ctx_max_increments(step)
                    ),
                });
            }
            if !increments.direct && increment > 2 {
                size = (size * 1.5).min(maximum);
            }
            let remaining = period - time;
            let last = size >= remaining * (1.0 - 1e-9);
            let dt = if last { remaining } else { size };
            let t1 = time + dt;
            let total = start_total + t1;
            progress.event(ProgressEvent::Increment {
                step: ctx.step_number,
                increment,
                attempt: 1,
                size: dt,
                step_time: t1,
                total_time: total,
            });

            // Effective matrix A = c_m M + c_k K (Rayleigh damping folded in).
            let c1 = 1.0 / (beta * dt * dt);
            let (c_m, c_k) = matrix_factors(scheme, rayleigh, dt, false);
            let a_matrix = SparseMatrix::linear_combination(m, c_m, k, c_k);
            if factored_dt.is_none_or(|d| (d - dt).abs() > 1e-12 * dt) {
                let started = std::time::Instant::now();
                let a_red = set.reduce(&a_matrix);
                let coordinates = || {
                    (set.free.iter())
                        .map(|&dof| ctx.model.nodes.coordinates[dof as usize / 3])
                        .collect()
                };
                equations
                    .prepare(
                        &mut ctx.solver,
                        &a_red,
                        coordinates,
                        true,
                        &mut FirstOrWarnings {
                            inner: &mut *progress,
                            all: false,
                        },
                    )
                    .map_err(|e| numerical(e, ctx.step_number, increment, &equations))?;
                progress.event(ProgressEvent::Info(format!(
                    "system matrix factorized in {:.2} s",
                    started.elapsed().as_secs_f64()
                )));
                factored_dt = Some(dt);
            }

            let (f1, _) = load_at(t1)?;
            let g1 = spc_at(t1)?;
            // Predictors.
            let u_tilde = combine(
                &[
                    (1.0, &motion.u),
                    (dt, &motion.v),
                    (dt * dt * (0.5 - beta), &motion.a),
                ],
                n,
            );
            let v_tilde = combine(&[(1.0, &motion.v), (dt * (1.0 - gamma), &motion.a)], n);
            let ku0 = multiply(k, &motion.u);
            let cv0 = damping(&motion.v);
            let f_eff = combine(
                &[
                    (1.0 + alpha, &f1),
                    (-alpha, &f_prev),
                    (alpha, &ku0),
                    (alpha, &cv0),
                ],
                n,
            );
            // A u₁ = f_eff + c1 M ũ − (1+α) C (ṽ − γΔt c1 ũ)
            let mu = multiply(m, &u_tilde);
            let damp_arg = combine(&[(1.0, &v_tilde), (-gamma * dt * c1, &u_tilde)], n);
            let cd = damping(&damp_arg);
            let r = combine(&[(1.0, &f_eff), (c1, &mu), (-(1.0 + alpha), &cd)], n);
            let ag = multiply(&a_matrix, &g1);
            let mut x = set.reduce_rhs(&combine(&[(1.0, &r), (-1.0, &ag)], n));
            equations
                .solve(
                    &mut ctx.solver,
                    &mut x,
                    &mut FirstOrWarnings {
                        inner: &mut *progress,
                        all: false,
                    },
                )
                .map_err(|e| in_increment(e, ctx.step_number, increment))?;
            let u1 = set.expand(&x, &g1);
            let a1 = combine(&[(c1, &u1), (-c1, &u_tilde)], n);
            let v1 = combine(&[(1.0, &v_tilde), (gamma * dt, &a1)], n);

            k.multiply(&u1, &mut reactions);
            ctx.constraints
                .transfer_forces(&mut reactions, &spc0.dofs, &active);
            integration_point_results(ctx.table, ctx.model, &u1, &mut strains, &mut stresses);
            motion = Motion {
                u: u1,
                v: v1,
                a: a1,
            };
            f_prev = f1;
            time = t1;

            progress.event(ProgressEvent::Iteration {
                step: ctx.step_number,
                increment,
                iteration: 1,
                residual: 0.0,
                correction: 0.0,
                converged: true,
            });
            if writes_increment(step, increment, if last { increment } else { u32::MAX }) {
                write_increment(
                    ctx,
                    step,
                    increment,
                    total,
                    &motion,
                    &reactions,
                    (&stresses, &strains),
                    carried_nodal.as_deref(),
                );
            }
            crate::history::write_history_with_velocities(
                ctx.results,
                ctx.model,
                ctx.table,
                step,
                (ctx.step_number, increment, total),
                (&motion.u, Some(&motion.v)),
                &reactions,
                &stresses,
                &strains,
                None,
            );
            // CalculiX counts a linear increment as two iterations.
            ctx.sta.push(StaRow {
                step: ctx.step_number,
                increment,
                attempt: 1,
                iterations: 2,
                total_time: total,
                step_time: t1,
                increment_size: dt,
            });
            progress.event(ProgressEvent::IncrementDone {
                step: ctx.step_number,
                increment,
                attempt: 1,
                iterations: 2,
                total_time: total,
                step_time: t1,
                size: dt,
            });
        }

        ctx.state.displacements = NodalField {
            per_node: 3,
            values: motion.u,
        };
        ctx.state.velocities = Some(NodalField {
            per_node: 3,
            values: motion.v,
        });
        ctx.state.accelerations = Some(NodalField {
            per_node: 3,
            values: motion.a,
        });
        let mut energy = internal_energy_density(&stresses, &strains);
        if let Some(c) = &carried {
            for (e, c) in energy.values.iter_mut().zip(&c.values) {
                *e += c;
            }
        }
        ctx.state.energy = Some(energy);
        ctx.state.stresses = stresses;
        ctx.state.strains = strains;
        ctx.state.total_time = start_total + period;
        ctx.state.steps_done = ctx.step_number;
        Ok(StepOutcome::Converged)
    }
}

/// `c_m M + c_k K` with the fully integrated stiffness and mass of reduced elements (and
/// the springs and linear blocks of the stiffness): the matrix of the initial accelerations
/// (`system`), or with `c_m = 0, c_k = 1` the stiffness for their internal forces.
fn initial_acceleration_matrix(
    table: &ElementTable,
    model: &Model,
    pattern: &std::sync::Arc<plxs_linalg::Pattern>,
    c_m: f64,
    c_k: f64,
    system: bool,
) -> Result<SparseMatrix> {
    let mut out = SparseMatrix::zeros(pattern.clone());
    let blocks: Vec<Result<Vec<f64>>> = table
        .entries
        .par_iter()
        .map(|entry| {
            let own = table.formulation(entry);
            let full = own.fully_integrated();
            let formulation = full.as_ref().unwrap_or(own);
            let coords = table.coordinates(model, entry);
            let rho = entry.density.ok_or_else(|| missing_density(model, entry))?;
            let mut ke = Vec::new();
            formulation.stiffness(&coords, table.material_matrix(entry), &mut ke);
            let mut me = Vec::new();
            formulation.mass_with(&coords, rho, &formulation.dynamic_mass_rule(), &mut me);
            // C3D4 enters with a quarter of its stiffness (single-DOF oracle with three Δt:
            // the factor of K is (1+α)β/400 instead of /100; observed, not explained).
            let c_k = if system && own.family == plxs_elements::Family::Tet4 {
                0.25 * c_k
            } else {
                c_k
            };
            Ok(ke.iter().zip(&me).map(|(k, m)| c_k * k + c_m * m).collect())
        })
        .collect();
    for (entry, block) in table.entries.iter().zip(blocks) {
        out.add_block(&entry.dofs, &block?);
    }
    for block in &table.linear_blocks {
        let scaled: Vec<f64> = block.stiffness.iter().map(|v| c_k * v).collect();
        out.add_block(&block.dofs, &scaled);
    }
    for element in &table.discrete {
        if let Some(ke) = element.spring_stiffness() {
            let scaled: Vec<f64> = ke.iter().map(|v| c_k * v).collect();
            out.add_block(&element.dofs, &scaled);
        }
    }
    Ok(out)
}

fn missing_density(model: &Model, entry: &plxs_assembly::elements::ElementEntry) -> Error {
    Error::input(
        "*DENSITY",
        format!(
            "element {} has no density; a dynamic analysis needs *DENSITY for every material",
            model.elements[entry.element as usize].id
        ),
    )
}

/// The factors of `M` and `K` in the system matrix of an increment of size `dt`: in the
/// displacement form `A = M/(βΔt²) + (1+α)γ/(βΔt) C + (1+α) K` of the increments, in the
/// acceleration form `A = M + (1+α)γΔt C + (1+α)βΔt² K` of the initial accelerations
/// (`acceleration_form`); `C = α_R M + β_R K`.
fn matrix_factors(
    scheme: Scheme,
    rayleigh: plxs_core::step::Rayleigh,
    dt: f64,
    acceleration_form: bool,
) -> (f64, f64) {
    let Scheme { alpha, beta, gamma } = scheme;
    let (m, c, k) = if acceleration_form {
        (
            1.0,
            (1.0 + alpha) * gamma * dt,
            (1.0 + alpha) * beta * dt * dt,
        )
    } else {
        let c1 = 1.0 / (beta * dt * dt);
        (c1, (1.0 + alpha) * gamma * dt * c1, 1.0 + alpha)
    };
    (m + c * rayleigh.alpha, k + c * rayleigh.beta)
}

/// `*STEP, INC=`: the largest number of increments of the step.
fn ctx_max_increments(step: &Step) -> u32 {
    if step.max_increments == 0 {
        100
    } else {
        step.max_increments
    }
}

fn numerical(e: Error, step: u32, increment: u32, equations: &Equations) -> Error {
    match e {
        Error::Numerical { message, .. } => Error::Numerical {
            step,
            increment,
            message: if equations.hints_constraints() {
                format!("{message}; the model is probably not sufficiently constrained")
            } else {
                message
            },
        },
        other => other,
    }
}

/// Appends the fields of a dynamic increment: those of a static increment plus `VELO`
/// after `DISP`, as CalculiX orders them, and `ENER` on top of the `carried` nodal energy.
#[allow(clippy::too_many_arguments)]
pub(crate) fn write_increment(
    ctx: &mut ProcedureContext<'_>,
    step: &Step,
    increment: u32,
    total: f64,
    motion: &Motion,
    reactions: &[f64],
    (stresses, strains): (&IntegrationPointField, &IntegrationPointField),
    carried: Option<&[f64]>,
) {
    let mut fields = build_fields(
        ctx.table, ctx.model, step, &motion.u, reactions, stresses, strains,
    );
    if let (Some(carried), Some(ener)) = (carried, fields.iter_mut().find(|f| f.name == "ENER")) {
        let order = node_order(ctx.model);
        for (value, &(_, i)) in ener.values[0].iter_mut().zip(&order) {
            *value += carried[i as usize];
        }
    }
    let variables = crate::fields::requested_variables(step);
    let has = |v: &str| variables.iter().any(|x| x == v);
    let order = node_order(ctx.model);
    if has("V") {
        let zero = vec![0.0; motion.v.len()];
        let (v, _) = ctx.table.reported(ctx.model, &motion.v, &zero);
        let field = ResultField {
            name: "VELO".into(),
            components: vec!["V1".into(), "V2".into(), "V3".into()],
            nodes: order.iter().map(|&(id, _)| id).collect(),
            values: (0..3)
                .map(|k| order.iter().map(|&(_, i)| v[i as usize * 3 + k]).collect())
                .collect(),
        };
        let at = fields
            .iter()
            .position(|f| f.name == "DISP")
            .map_or(0, |p| p + 1);
        fields.insert(at, field);
    }
    ctx.results.increments.push(ResultsIncrement {
        step: ctx.step_number,
        increment,
        kind: IncrementKind::Dynamic,
        value: total,
        total_time: total,
        nodal_diameter: None,
        fields,
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hht_parameters_follow_alpha() {
        let s = Scheme::new(-0.05);
        assert!((s.beta - 0.275625).abs() < 1e-15);
        assert!((s.gamma - 0.55).abs() < 1e-15);
        let s = Scheme::new(0.0);
        assert_eq!((s.beta, s.gamma), (0.25, 0.5));
    }

    #[test]
    fn matrix_factors_of_the_displacement_and_acceleration_forms() {
        let s = Scheme::new(-0.1);
        let dt = 1e-3;
        let undamped = plxs_core::step::Rayleigh::default();
        // Displacement form: M/(β Δt²) + (1+α) K.
        let (c_m, c_k) = matrix_factors(s, undamped, dt, false);
        assert!((c_m - 1.0 / (s.beta * dt * dt)).abs() < 1e-6 * c_m);
        assert!((c_k - 0.9).abs() < 1e-12);
        // Acceleration form: M + (1+α) β Δt² K.
        let (c_m, c_k) = matrix_factors(s, undamped, dt, true);
        assert_eq!(c_m, 1.0);
        assert!((c_k - 0.9 * s.beta * dt * dt).abs() < 1e-18);
    }

    #[test]
    fn combine_adds_scaled_vectors() {
        let a = [1.0, 2.0];
        let b = [3.0, -1.0];
        assert_eq!(combine(&[(2.0, &a), (-1.0, &b)], 2), vec![-1.0, 5.0]);
    }
}
