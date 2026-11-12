//! `*HEAT TRANSFER` (steady state and transient) and the temperature-displacement
//! procedures `*COUPLED TEMPERATURE-DISPLACEMENT` and `*UNCOUPLED TEMPERATURE-DISPLACEMENT`
//! for small displacements.
//!
//! Thermal problem per increment (backward Euler for transient steps, Lewis et al. chapter
//! 7): `(K(T) + C/Δt) T_{n+1} = q(T_{n+1}) + C/Δt T_n`, solved by Newton's method with the
//! tangent of films and radiation; linear problems take one solve. As in CalculiX, the
//! capacity term acts on the rates of the free temperatures only: prescribed temperatures
//! jump to their value without a capacity reaction.
//!
//! Mechanical problem: with small displacements the temperature does not depend on the
//! displacements (no mechanical heat sources), so coupled and uncoupled steps both solve
//! the temperatures of an increment first and then the linear static equilibrium with the
//! thermal strains of these temperatures. Geometric nonlinearity (`NLGEOM`) is not
//! supported here.
//!
//! Contact: the mechanical problem of an increment becomes nonlinear (Newton's method with
//! the contact springs on the elastic stiffness). Closed contact points conduct heat with
//! the `*GAP CONDUCTANCE` of their interaction (per unit area, interpolated in the contact
//! pressure). The two problems are staggered: the contact state of the displacements at
//! the temperatures of the start of an increment decides which points conduct during the
//! increment; heat transfer steps take the contact state of the current displacements.
//!
//! Time incrementation as CalculiX does it (observed with ccx, documented in
//! `docs/solver/SCHNITTSTELLEN.md`): `DIRECT` keeps the initial increment; otherwise the
//! increment grows by 1.5 when the last two increments converged in at most 4 iterations
//! and the last one was not cut back, failed increments are cut to a quarter, and in
//! transient steps with `DELTMX` an increment whose largest temperature change exceeds
//! `DELTMX` is repeated with `0.85 · Δt · DELTMX / ΔT_max`.
//!
//! No CalculiX code was consulted.

use plxs_assembly::{
    ElementTable, HeatTable, Reduction, Spc, assemble_heat, assemble_stiffness, heat_loads,
    load_vector, temperature_spc,
};
use plxs_core::progress::NoProgress;
use plxs_core::results::{IncrementKind, ResultField, ResultsIncrement};
use plxs_core::state::{IntegrationPointField, NodalField};
use plxs_core::step::{DefaultAmplitude, HeatTransfer, Procedure as ProcedureKind};
use plxs_core::{
    Convergence, Divergence, Error, Failure, Model, Progress, ProgressEvent, Result, Step, Stop,
};
use std::sync::Arc;

use plxs_contact::ThermalLink;
use plxs_core::contact::GapConductance;
use plxs_linalg::Unknowns;
use plxs_linalg::{LdltSolver, Pattern, SparseMatrix};
use plxs_output::StaRow;

use crate::equations::{Equations, in_increment};
use crate::fields::{nodal_average, write_increment};
use crate::static_linear::{integration_point_results, writes_increment};
use crate::thermal::{ThermalStrains, initial_temperatures};
use crate::{Procedure, ProcedureContext, StepOutcome};

pub struct HeatTransferProcedure;

/// Iterations at most before an increment is cut back.
const MAX_ITERATIONS: u32 = 30;
/// Growth of the increment after easy increments, CalculiX' default.
const GROWTH: f64 = 1.5;
/// Cutback factor after a failed increment.
const CUTBACK: f64 = 0.25;

impl Procedure for HeatTransferProcedure {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        let (parameters, mechanical) = match &step.procedure {
            ProcedureKind::HeatTransfer(p) => (p, false),
            ProcedureKind::CoupledTemperatureDisplacement(p)
            | ProcedureKind::UncoupledTemperatureDisplacement(p) => (p, true),
            _ => {
                return Err(Error::Unsupported(
                    "HeatTransferProcedure needs a heat transfer step".into(),
                ));
            }
        };
        if step.nlgeom && mechanical {
            return Err(Error::Unsupported(format!(
                "procedure not implemented yet: {}, NLGEOM (step {})",
                step.procedure.keyword(),
                step.name
            )));
        }
        let heat = HeatTable::new(ctx.model, ctx.table)?;
        let transient = !parameters.steady_state;
        heat.check(ctx.model, ctx.table, transient)?;
        let mut run = HeatRun::new(ctx, step, parameters, &heat, mechanical)?;
        run.increments(ctx, step, progress)
    }
}

/// The state of a heat transfer step while it runs.
struct HeatRun<'h> {
    heat: &'h HeatTable,
    transient: bool,
    mechanical: bool,
    ramp: bool,
    direct: bool,
    period: f64,
    initial: f64,
    minimum: f64,
    maximum: f64,
    /// Result frames the increments end on (`FRAMES=`).
    frames: Option<crate::frames::Frames>,
    deltmx: Option<f64>,
    nonlinear: bool,
    k: SparseMatrix,
    c: Option<SparseMatrix>,
    tangent: SparseMatrix,
    solver: LdltSolver,
    /// Direct or iterative, as the step asks, for the temperatures.
    equations: Equations,
    /// The same for the displacements of coupled steps (their direct solver is the
    /// context's).
    mechanical_equations: Equations,
    /// Node positions, for the ordering of the factorization.
    coordinates: Vec<[f64; 3]>,
    /// Free set and matrix values of the last factorization.
    factorized: Option<(Vec<u32>, Vec<f64>)>,
    assembled: bool,
    /// Prescribed mechanical DOFs of the last mechanical factorization.
    mechanical_spc: Option<Vec<u32>>,
    /// Whether contact pairs are in force in the step.
    contact: bool,
    /// The closed contact points that conduct heat in the current increment.
    links: Vec<ThermalLink>,
}

/// A converged increment of the thermal problem.
struct ThermalIncrement {
    t: Vec<f64>,
    iterations: u32,
    free: Vec<u32>,
}

impl<'h> HeatRun<'h> {
    fn new(
        ctx: &ProcedureContext<'_>,
        step: &Step,
        p: &HeatTransfer,
        heat: &'h HeatTable,
        mechanical: bool,
    ) -> Result<Self> {
        let transient = !p.steady_state;
        let period = p.increments.period();
        let maximum = p
            .increments
            .maximum
            .filter(|m| *m > 0.0)
            .unwrap_or(period)
            .min(period);
        // The first increment respects the largest one as every later increment does.
        let initial = p.increments.initial().min(maximum);
        if period <= 0.0 || initial <= 0.0 {
            return Err(Error::input(
                step.procedure.keyword(),
                "the step period and the initial increment must be positive",
            ));
        }
        let capacity_varies = ctx.model.materials.iter().any(|m| {
            m.density.as_ref().is_some_and(|d| d.rows.len() > 1)
                || m.specific_heat.as_ref().is_some_and(|c| c.rows.len() > 1)
        });
        let nonlinear = heat.temperature_dependent
            || (transient && capacity_varies)
            || !step.loads.radiations.is_empty();
        Ok(Self {
            heat,
            transient,
            mechanical,
            // Steady-state steps ramp their loads like static ones, transient steps apply
            // them at once.
            ramp: !transient && step.default_amplitude == DefaultAmplitude::Ramp,
            direct: p.direct || p.increments.direct,
            period,
            initial,
            minimum: p.increments.minimum.unwrap_or(initial * 1e-5),
            maximum,
            frames: crate::frames::Frames::of(&p.increments, period),
            deltmx: p.deltmx.filter(|d| *d > 0.0),
            nonlinear,
            k: SparseMatrix::zeros(heat.pattern.clone()),
            c: transient.then(|| SparseMatrix::zeros(heat.pattern.clone())),
            tangent: SparseMatrix::zeros(heat.pattern.clone()),
            solver: LdltSolver::new(ctx.threads),
            equations: Equations::new(p.solver),
            mechanical_equations: Equations::new(p.solver).for_model(ctx.model),
            coordinates: ctx.model.nodes.coordinates.clone(),
            factorized: None,
            assembled: false,
            mechanical_spc: None,
            contact: false,
            links: Vec::new(),
        })
    }

    fn increments(
        &mut self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        let model = ctx.model;
        let start_total = ctx.state.total_time;
        let mut t = match &ctx.state.temperatures {
            Some(field) if field.values.len() == model.nodes.len() => field.values.clone(),
            _ => initial_temperatures(model),
        };
        let start = t.clone();
        let thermal = if self.mechanical {
            ThermalStrains::new(model, ctx.table, self.heat)
        } else {
            None
        };
        if !ctx.contact.is_empty() {
            for warning in ctx.contact.begin_step(step, ctx.state) {
                progress.event(ProgressEvent::Warning(warning));
            }
            self.contact = ctx.contact.is_active();
        }

        let mut step_time = 0.0;
        let mut dt = self.initial;
        let mut increment = 0u32;
        // Iterations of the increments done so far in this step.
        let mut done: Vec<u32> = Vec::new();
        let end = self.period * (1.0 - 1e-9);
        while step_time < end {
            if ctx.cancel.is_cancelled() {
                return Ok(StepOutcome::Cancelled);
            }
            increment += 1;
            if increment > step.max_increments {
                return Err(Error::Numerical {
                    step: ctx.step_number,
                    increment,
                    failure: Failure::IncrementLimit,
                    message: format!(
                        "the step needs more than INC={} increments",
                        step.max_increments
                    ),
                });
            }
            if self.contact {
                // The contact state that conducts heat in this increment.
                if self.mechanical {
                    let size = dt.min(self.period - step_time);
                    let time = step_time + size;
                    self.mechanics(
                        ctx,
                        step,
                        thermal.as_ref(),
                        &t,
                        time,
                        start_total + time,
                        increment,
                        false,
                        false,
                    )?;
                } else {
                    let u = displacements(ctx);
                    ctx.contact.prepare(&model.nodes.coordinates, &u);
                    let evaluation = ctx.contact.evaluate(
                        &model.nodes.coordinates,
                        &u,
                        &ctx.state.contact,
                        false,
                    );
                    ctx.contact.last = Some(evaluation);
                }
                let links = match &ctx.contact.last {
                    Some(evaluation) => ctx.contact.thermal_links(
                        &model.nodes.coordinates,
                        &displacements(ctx),
                        evaluation,
                    ),
                    None => Vec::new(),
                };
                self.set_links(links);
            }
            let mut attempt = 0u32;
            // The planned size of an increment shortened to end on a frame, which the next
            // increment resumes.
            let mut planned: Option<f64>;
            let (accepted, size) = loop {
                attempt += 1;
                let mut size = dt.min(self.period - step_time);
                if let Some(frames) = &self.frames {
                    size = frames.limit(step_time, size);
                }
                planned = (size < dt).then_some(dt);
                let time = if self.period - step_time - size <= 1e-9 * self.period {
                    self.period
                } else {
                    step_time + size
                };
                let total = start_total + time;
                progress.event(ProgressEvent::Increment {
                    step: ctx.step_number,
                    increment,
                    attempt,
                    size,
                    step_time: time,
                    total_time: total,
                });
                let result = self.thermal_increment(
                    ctx, step, &t, &start, time, total, size, increment, progress,
                );
                let reject = match result {
                    Ok(inc) => {
                        let change = inc
                            .free
                            .iter()
                            .map(|&n| (inc.t[n as usize] - t[n as usize]).abs())
                            .fold(0.0_f64, f64::max);
                        match self.deltmx {
                            Some(limit) if self.transient && !self.direct && change > limit => {
                                Some((
                                    size * 0.85 * limit / change,
                                    None,
                                    format!(
                                        "largest temperature change {change:.4e} exceeds DELTMX={limit}"
                                    ),
                                ))
                            }
                            _ => break (inc, size),
                        }
                    }
                    Err(Error::Numerical {
                        failure, message, ..
                    }) if !self.direct => Some((size * CUTBACK, Some(failure), message)),
                    Err(e) => return Err(e),
                };
                let (smaller, failure, reason) = reject.unwrap();
                progress.event(ProgressEvent::Cutback {
                    step: ctx.step_number,
                    increment,
                    attempt,
                    reason: reason.clone(),
                });
                if smaller < self.minimum {
                    // A failure of the equations stays what it is; one of the iteration
                    // (or the temperature change) is no convergence of the step.
                    let failure = match failure {
                        Some(Failure::NoConvergence(c)) => Failure::NoConvergence(Convergence {
                            stop: Stop::MinimumIncrement,
                            ..c
                        }),
                        Some(other) => other,
                        None => Failure::NoConvergence(Convergence {
                            cause: Divergence::Temperatures,
                            stop: Stop::MinimumIncrement,
                            contact: self.contact,
                            contact_changing: false,
                        }),
                    };
                    return Err(Error::Numerical {
                        step: ctx.step_number,
                        increment,
                        failure,
                        message: format!(
                            "{reason}; the increment size {smaller:.4e} would fall below the minimum {:.4e}",
                            self.minimum
                        ),
                    });
                }
                dt = smaller;
            };
            let time = if self.period - step_time - size <= 1e-9 * self.period {
                self.period
            } else {
                step_time + size
            };
            let total = start_total + time;
            t = accepted.t;
            let iterations = accepted.iterations;
            ctx.state.temperatures = Some(NodalField {
                per_node: 1,
                values: t.clone(),
            });

            // The mechanical part and the results of the increment.
            let last = (self.period - time).abs() <= 1e-9 * self.period;
            let write = match &mut self.frames {
                Some(frames) => frames.reached(time) || last,
                None => writes_increment(step, increment, u32::MAX) || last,
            };
            if self.mechanical {
                self.mechanics(
                    ctx,
                    step,
                    thermal.as_ref(),
                    &t,
                    time,
                    total,
                    increment,
                    write,
                    true,
                )?;
            } else if write {
                let fields = Vec::new();
                ctx.results.increments.push(ResultsIncrement {
                    step: ctx.step_number,
                    increment,
                    kind: IncrementKind::Time,
                    value: time,
                    total_time: total,
                    nodal_diameter: None,
                    fields,
                });
            }
            if write {
                self.thermal_fields(ctx, step, &t)?;
                if let Some(last) = ctx.results.increments.last_mut()
                    && self.transient
                {
                    last.kind = IncrementKind::Dynamic;
                }
            }
            self.thermal_history(ctx, step, increment, time, &t)?;
            ctx.sta.push(StaRow {
                step: ctx.step_number,
                increment,
                attempt,
                iterations,
                total_time: total,
                step_time: time,
                increment_size: size,
            });
            progress.event(ProgressEvent::IncrementDone {
                step: ctx.step_number,
                increment,
                attempt,
                iterations,
                total_time: total,
                step_time: time,
                size,
            });
            step_time = time;
            done.push(iterations);

            // The next increment.
            dt = planned.unwrap_or(size);
            if !self.direct
                && attempt == 1
                && done.len() >= 2
                && done[done.len() - 2..].iter().all(|&i| i <= 4)
            {
                dt *= GROWTH;
            }
            dt = dt.min(self.maximum);
        }
        ctx.state.total_time = start_total + self.period;
        ctx.state.steps_done = ctx.step_number;
        Ok(StepOutcome::Converged)
    }

    /// Solves the temperatures at the end of an increment from `t_old`.
    #[allow(clippy::too_many_arguments)]
    fn thermal_increment(
        &mut self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        t_old: &[f64],
        start: &[f64],
        time: f64,
        total: f64,
        size: f64,
        increment: u32,
        progress: &mut dyn Progress,
    ) -> Result<ThermalIncrement> {
        let model = ctx.model;
        let table = ctx.table;
        let nodes = model.nodes.len();
        let spc = temperature_spc(model, step, start, time, total, self.period, self.ramp);
        let reduction = Reduction::new(nodes, spc);
        let mut t = reduction.prescribed_vector(nodes);
        for &dof in &reduction.free {
            t[dof as usize] = t_old[dof as usize];
        }
        let mut kt = vec![0.0; nodes];
        let mut ct = vec![0.0; nodes];
        let mut iteration = 0u32;
        loop {
            iteration += 1;
            if !self.assembled || self.nonlinear {
                assemble_heat(model, table, self.heat, &t, &mut self.k, self.c.as_mut())?;
                add_links(model, &self.links, &t, &mut self.k);
                self.assembled = true;
            }
            let q = heat_loads(
                model,
                table,
                step,
                &t,
                time,
                total,
                self.period,
                self.ramp,
                &mut self.tangent,
            )?;
            self.k.multiply(&t, &mut kt);
            let mut residual: Vec<f64> = q.iter().zip(&kt).map(|(q, k)| q - k).collect();
            if let Some(c) = &self.c {
                // Capacity on the rates of the free temperatures only.
                let mut rate = vec![0.0; nodes];
                for &dof in &reduction.free {
                    rate[dof as usize] = (t[dof as usize] - t_old[dof as usize]) / size;
                }
                c.multiply(&rate, &mut ct);
                for (r, c) in residual.iter_mut().zip(&ct) {
                    *r -= c;
                }
            }
            let mut rhs = reduction.reduce_vector(&residual);
            let residual_norm = rhs.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            if !reduction.free.is_empty() {
                self.factorize(&reduction.free, size)?;
                self.equations
                    .solve(&mut self.solver, &mut rhs, &mut NoProgress)
                    .map_err(|e| in_increment(e, ctx.step_number, increment))?;
            }
            for (&dof, d) in reduction.free.iter().zip(&rhs) {
                t[dof as usize] += d;
            }
            let correction = rhs.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            let scale = t.iter().fold(1.0_f64, |m, v| m.max(v.abs()));
            let converged = !self.nonlinear || correction <= 1e-10 * scale;
            progress.event(ProgressEvent::Iteration {
                step: ctx.step_number,
                increment,
                iteration,
                residual: residual_norm,
                correction,
                residual_limit: None,
                correction_limit: self.nonlinear.then_some(1e-10 * scale),
                converged,
            });
            if converged {
                // CalculiX counts the check of the linear solution as a second iteration.
                return Ok(ThermalIncrement {
                    t,
                    iterations: iteration + 1,
                    free: reduction.free,
                });
            }
            if iteration >= MAX_ITERATIONS || !correction.is_finite() {
                return Err(Error::Numerical {
                    step: ctx.step_number,
                    increment,
                    failure: Failure::NoConvergence(Convergence {
                        cause: if correction.is_finite() {
                            Divergence::Temperatures
                        } else {
                            Divergence::NotFinite
                        },
                        stop: Stop::FixedIncrement,
                        contact: self.contact,
                        contact_changing: false,
                    }),
                    message: format!(
                        "no convergence of the temperatures after {iteration} iterations (largest correction {correction:.4e})"
                    ),
                });
            }
        }
    }

    /// Factorizes `K + tangent + C/Δt` on the free temperatures unless the same matrix
    /// was factorized last.
    fn factorize(&mut self, free: &[u32], size: f64) -> Result<()> {
        let mut a = self.k.clone();
        for (v, t) in a.values_mut().iter_mut().zip(self.tangent.values()) {
            *v += t;
        }
        if let Some(c) = &self.c {
            for (v, c) in a.values_mut().iter_mut().zip(c.values()) {
                *v += c / size;
            }
        }
        let same_free = self.factorized.as_ref().is_some_and(|(f, _)| f == free);
        if same_free
            && self
                .factorized
                .as_ref()
                .is_some_and(|(_, v)| v == a.values())
        {
            return Ok(());
        }
        let reduced = a.restrict(free);
        let coordinates = || Unknowns::from_free(free, 1, &self.coordinates);
        self.equations.prepare(
            &mut self.solver,
            &reduced,
            coordinates,
            !same_free,
            &mut NoProgress,
        )?;
        self.factorized = Some((free.to_vec(), a.values().to_vec()));
        Ok(())
    }

    /// Solves the static equilibrium of an increment with the thermal strains of `t` and
    /// appends its mechanical fields when `write`.
    #[allow(clippy::too_many_arguments)]
    fn mechanics(
        &mut self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        thermal: Option<&ThermalStrains<'_>>,
        t: &[f64],
        time: f64,
        total: f64,
        increment: u32,
        write: bool,
        record: bool,
    ) -> Result<()> {
        if self.contact {
            return self.mechanics_with_contact(
                ctx, step, thermal, t, time, total, increment, write, record,
            );
        }
        let model = ctx.model;
        let table = ctx.table;
        let total_dofs = table.dofs.len();
        if ctx.stiffness.is_none() {
            let mut k = SparseMatrix::zeros(ctx.pattern.clone());
            table.require_elastic()?;
            assemble_stiffness(table, model, &mut k);
            ctx.stiffness = Some(k);
        }
        let k = ctx.stiffness.take().unwrap();
        let result = (|| -> Result<()> {
            let spc = Spc::of_step(model, step, &table.dofs, time, total, self.period)?;
            let reduction = Reduction::new(total_dofs, spc);
            if self.mechanical_spc.as_ref() != Some(&reduction.spc.dofs) {
                let k_free = k.restrict(&reduction.free);
                let coordinates =
                    || crate::equations::displacement_unknowns(model, &reduction.free);
                let iterative = self.mechanical_equations.is_iterative();
                self.mechanical_equations
                    .prepare(&mut ctx.solver, &k_free, coordinates, true, &mut NoProgress)
                    .map_err(|e| match e {
                        Error::Numerical {
                            failure, message, ..
                        } => Error::Numerical {
                            step: ctx.step_number,
                            increment,
                            failure,
                            message: if iterative {
                                message
                            } else {
                                format!(
                                    "{message}; the model is probably not sufficiently constrained"
                                )
                            },
                        },
                        other => other,
                    })?;
                self.mechanical_spc = Some(reduction.spc.dofs.clone());
            }
            let (mut f, _) = load_vector(model, table, step, time, total, self.period)?;
            let eps_th = thermal.map(|th| th.strains(t));
            let f_th = match (thermal, &eps_th) {
                (Some(th), Some(_)) => Some(th.load(t)),
                _ => None,
            };
            if let Some(f_th) = &f_th {
                for (a, b) in f.iter_mut().zip(f_th) {
                    *a += b;
                }
            }
            let u_p = reduction.prescribed_vector(total_dofs);
            let mut k_up = vec![0.0; total_dofs];
            k.multiply(&u_p, &mut k_up);
            let rhs_full: Vec<f64> = f.iter().zip(&k_up).map(|(f, ku)| f - ku).collect();
            let mut rhs = reduction.reduce_vector(&rhs_full);
            self.mechanical_equations
                .solve(&mut ctx.solver, &mut rhs, &mut NoProgress)
                .map_err(|e| in_increment(e, ctx.step_number, increment))?;
            let mut u = u_p;
            reduction.expand_into(&rhs, &mut u);
            // Nodal forces: the internal forces ∫ Bᵀ σ dV = K u - f_th.
            let mut forces = vec![0.0; total_dofs];
            k.multiply(&u, &mut forces);
            if let Some(f_th) = &f_th {
                for (a, b) in forces.iter_mut().zip(f_th) {
                    *a -= b;
                }
            }
            let mut stresses = table.layout.field(6);
            let mut strains = table.layout.field(6);
            integration_point_results(table, model, &u, &mut strains, &mut stresses);
            if let (Some(th), Some(e)) = (thermal, &eps_th) {
                th.correct_stresses(e, &mut strains, &mut stresses);
            }
            ctx.state.displacements.per_node = 3;
            ctx.state.displacements.values.clear();
            ctx.state.displacements.values.extend_from_slice(&u);
            ctx.state.stresses = stresses.clone();
            ctx.state.strains = strains.clone();
            ctx.state.loads = None;
            if write {
                write_increment(
                    ctx, step, increment, time, total, &u, &forces, &stresses, &strains, None,
                )?;
            }
            if !record {
                return Ok(());
            }
            crate::history::write_history(
                ctx.results,
                model,
                table,
                step,
                ctx.step_number,
                increment,
                time,
                &u,
                &forces,
                &stresses,
                &strains,
                None,
            );
            Ok(())
        })();
        ctx.stiffness = Some(k);
        result
    }

    /// Uses `links` for the conduction across the contact from now on: the temperature
    /// pattern grows when they couple nodes it does not hold yet.
    fn set_links(&mut self, links: Vec<ThermalLink>) {
        let pattern = self.k.pattern().clone();
        if !links
            .iter()
            .all(|l| crate::contact::covers(&pattern, &l.nodes))
        {
            let n = pattern.dimension();
            let mut columns: Vec<Vec<u32>> = (0..n).map(|c| pattern.column(c).to_vec()).collect();
            for link in &links {
                for &a in &link.nodes {
                    for &b in &link.nodes {
                        // Upper triangle: rows above the diagonal per column.
                        if a <= b {
                            columns[b as usize].push(a);
                        }
                    }
                }
            }
            for column in &mut columns {
                column.sort_unstable();
                column.dedup();
            }
            let grown = Arc::new(Pattern::from_columns(n, columns));
            self.k = SparseMatrix::zeros(grown.clone());
            self.tangent = SparseMatrix::zeros(grown.clone());
            self.c = self.c.as_ref().map(|_| SparseMatrix::zeros(grown));
            self.factorized = None;
        }
        self.links = links;
        self.assembled = false;
    }

    /// The static equilibrium of an increment with contact: Newton's method on the elastic
    /// stiffness plus the contact springs, the thermal strains of `t` as loads. Stores the
    /// converged contact state; appends the mechanical fields when `write` and the history
    /// output when `record`.
    #[allow(clippy::too_many_arguments)]
    fn mechanics_with_contact(
        &mut self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        thermal: Option<&ThermalStrains<'_>>,
        t: &[f64],
        time: f64,
        total: f64,
        increment: u32,
        write: bool,
        record: bool,
    ) -> Result<()> {
        let model = ctx.model;
        let table = ctx.table;
        let total_dofs = table.dofs.len();
        table.require_elastic()?;
        let coords = &model.nodes.coordinates;
        let mut u = displacements(ctx);
        let blocks = ctx.contact.prepare(coords, &u);
        let pattern = crate::contact::cached_pattern(table, &blocks, &mut ctx.contact_pattern);
        let mut k = SparseMatrix::zeros(pattern);
        assemble_stiffness(table, model, &mut k);
        let spc = Spc::of_step(model, step, &table.dofs, time, total, self.period)?;
        let reduction = Reduction::new(total_dofs, spc);
        for (dof, value) in reduction.prescribed_vector(total_dofs).iter().enumerate() {
            if reduction.to_reduced[dof] == u32::MAX {
                u[dof] = *value;
            }
        }
        let (mut f, _) = load_vector(model, table, step, time, total, self.period)?;
        let eps_th = thermal.map(|th| th.strains(t));
        let f_th = match (thermal, &eps_th) {
            (Some(th), Some(_)) => Some(th.load(t)),
            _ => None,
        };
        if let Some(f_th) = &f_th {
            for (a, b) in f.iter_mut().zip(f_th) {
                *a += b;
            }
        }
        let iterative = self.mechanical_equations.is_iterative();
        let mut previous_status: Option<Vec<u8>> = None;
        let mut forces = vec![0.0; total_dofs];
        let mut iteration = 0u32;
        let evaluation = loop {
            iteration += 1;
            let evaluation = ctx
                .contact
                .evaluate(coords, &u, &ctx.state.contact, iteration == 1);
            let mut a = k.clone();
            k.multiply(&u, &mut forces);
            if let Some(f_th) = &f_th {
                for (a, b) in forces.iter_mut().zip(f_th) {
                    *a -= b;
                }
            }
            for element in &evaluation.elements {
                a.add_block(&element.dofs, &element.ke);
                for (&dof, value) in element.dofs.iter().zip(&element.fe) {
                    forces[dof as usize] += value;
                }
            }
            let residual: Vec<f64> = f
                .iter()
                .zip(&forces)
                .zip(
                    f_th.as_deref()
                        .map_or(&[][..], |v| v)
                        .iter()
                        .chain(std::iter::repeat(&0.0)),
                )
                .map(|((f, r), th)| f - th - r)
                .collect();
            let mut rhs = reduction.reduce_vector(&residual);
            let a_free = a.restrict(&reduction.free);
            let coordinates = || Unknowns::from_free(&reduction.free, 3, coords);
            self.mechanical_equations
                .prepare(&mut ctx.solver, &a_free, coordinates, true, &mut NoProgress)
                .map_err(|e| match e {
                    Error::Numerical {
                        failure, message, ..
                    } => Error::Numerical {
                        step: ctx.step_number,
                        increment,
                        failure,
                        message: if iterative {
                            message
                        } else {
                            format!("{message}; the model is probably not sufficiently constrained")
                        },
                    },
                    other => other,
                })?;
            self.mechanical_equations
                .solve(&mut ctx.solver, &mut rhs, &mut NoProgress)
                .map_err(|e| in_increment(e, ctx.step_number, increment))?;
            for (&dof, d) in reduction.free.iter().zip(&rhs) {
                u[dof as usize] += d;
            }
            let correction = rhs.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            let scale = u.iter().fold(0.0_f64, |m, v| m.max(v.abs())).max(1e-30);
            let status = evaluation.status_key();
            let settled = previous_status.as_ref() == Some(&status);
            previous_status = Some(status);
            if settled && correction <= 1e-8 * scale {
                break ctx.contact.evaluate(coords, &u, &ctx.state.contact, false);
            }
            if iteration >= 3 * MAX_ITERATIONS || !correction.is_finite() {
                return Err(Error::Numerical {
                    step: ctx.step_number,
                    increment,
                    failure: Failure::NoConvergence(Convergence {
                        cause: if correction.is_finite() {
                            Divergence::ThermalContact
                        } else {
                            Divergence::NotFinite
                        },
                        stop: Stop::FixedIncrement,
                        contact: true,
                        contact_changing: !settled,
                    }),
                    message: format!(
                        "no convergence of the contact after {iteration} iterations (largest correction {correction:.4e})"
                    ),
                });
            }
        };
        // The factorization holds contact springs: the linear path must factorize anew.
        self.mechanical_spc = None;
        // Nodal forces: internal and contact forces at the converged displacements.
        k.multiply(&u, &mut forces);
        if let Some(f_th) = &f_th {
            for (a, b) in forces.iter_mut().zip(f_th) {
                *a -= b;
            }
        }
        for element in &evaluation.elements {
            for (&dof, value) in element.dofs.iter().zip(&element.fe) {
                forces[dof as usize] += value;
            }
        }
        if record {
            ctx.state.contact = evaluation.states();
        }
        ctx.contact.last = Some(evaluation);
        let mut stresses = table.layout.field(6);
        let mut strains = table.layout.field(6);
        integration_point_results(table, model, &u, &mut strains, &mut stresses);
        if let (Some(th), Some(e)) = (thermal, &eps_th) {
            th.correct_stresses(e, &mut strains, &mut stresses);
        }
        ctx.state.displacements.per_node = 3;
        ctx.state.displacements.values.clear();
        ctx.state.displacements.values.extend_from_slice(&u);
        ctx.state.stresses = stresses.clone();
        ctx.state.strains = strains.clone();
        ctx.state.loads = None;
        if write {
            write_increment(
                ctx, step, increment, time, total, &u, &forces, &stresses, &strains, None,
            )?;
        }
        if record {
            crate::history::write_history(
                ctx.results,
                model,
                table,
                step,
                ctx.step_number,
                increment,
                time,
                &u,
                &forces,
                &stresses,
                &strains,
                None,
            );
        }
        Ok(())
    }

    /// The `.dat` blocks of the thermal output requests.
    fn thermal_history(
        &mut self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        increment: u32,
        time: f64,
        t: &[f64],
    ) -> Result<()> {
        if step.outputs.node_print.is_empty() && step.outputs.el_print.is_empty() {
            return Ok(());
        }
        let mut kt = vec![0.0; t.len()];
        self.k.multiply(t, &mut kt);
        let flux = if crate::history::wants_heat_flux(step) {
            Some(heat_flux(ctx.model, ctx.table, self.heat, t)?)
        } else {
            None
        };
        crate::history::write_thermal_history(
            ctx.results,
            ctx.model,
            ctx.table,
            step,
            ctx.step_number,
            increment,
            time,
            t,
            &kt,
            flux.as_ref(),
        );
        Ok(())
    }

    /// Adds the thermal fields (`NDTEMP`, `FLUX`, `RFL`) to the last written increment in
    /// CalculiX' order: the temperatures after the displacements, the fluxes at the end.
    fn thermal_fields(
        &mut self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        t: &[f64],
    ) -> Result<()> {
        let variables = crate::fields::requested_variables(step);
        let has = |v: &str| variables.iter().any(|x| x == v);
        let model = ctx.model;
        let table = ctx.table;
        let order = crate::fields::node_order(model);
        let ids: Vec<u32> = order.iter().map(|&(id, _)| id).collect();
        let scalar = |name: &str, component: &str, values: &[f64]| ResultField {
            name: name.into(),
            components: vec![component.into()],
            nodes: ids.clone(),
            values: vec![order.iter().map(|&(_, i)| values[i as usize]).collect()],
        };
        let mut front = Vec::new();
        let mut back = Vec::new();
        if has("NT") {
            front.push(scalar("NDTEMP", "T", t));
        }
        if has("HFL") {
            let flux = heat_flux(model, table, self.heat, t)?;
            let (average, _) = nodal_average(table, model, &flux);
            back.push(ResultField {
                name: "FLUX".into(),
                components: vec!["F1".into(), "F2".into(), "F3".into()],
                nodes: ids.clone(),
                values: (0..3)
                    .map(|k| order.iter().map(|&(_, i)| average[i as usize][k]).collect())
                    .collect(),
            });
        }
        if has("RFL") {
            // CalculiX' RFL is the conduction K·T at every node (prescribed nodes: the heat
            // that leaves through the boundary).
            // Nodes of plane and axisymmetric elements report 0, as CalculiX does.
            let mut kt = vec![0.0; t.len()];
            self.k.multiply(t, &mut kt);
            zero_on_expanded(ctx.table, &mut kt);
            back.push(scalar("RFL", "RFL", &kt));
        }
        let Some(last) = ctx.results.increments.last_mut() else {
            return Ok(());
        };
        let at = last
            .fields
            .iter()
            .position(|f| f.name == "DISP")
            .map_or(0, |p| p + 1);
        for (offset, field) in front.into_iter().enumerate() {
            last.fields.insert(at + offset, field);
        }
        last.fields.extend(back);
        Ok(())
    }
}

/// The displacements of the state, zero where none were solved yet.
fn displacements(ctx: &ProcedureContext<'_>) -> Vec<f64> {
    let total = ctx.table.dofs.len();
    let values = &ctx.state.displacements.values;
    if values.len() == total {
        values.clone()
    } else {
        vec![0.0; total]
    }
}

/// The conductance per unit area of a closed contact point: `*GAP CONDUCTANCE` of its
/// interaction, linear in the contact pressure between the rows of the first temperature
/// (constant beyond them), at the mean temperature of the link's nodes for tables with
/// several temperatures.
fn conductance(model: &Model, link: &ThermalLink, t: &[f64]) -> f64 {
    let Some(interaction) = model.contact.pairs.get(link.pair).and_then(|pair| {
        model
            .contact
            .interactions
            .iter()
            .find(|i| i.name == pair.interaction)
    }) else {
        return 0.0;
    };
    match &interaction.gap_conductance {
        None => 0.0,
        Some(GapConductance::Constant(h)) => *h,
        Some(GapConductance::Table(rows)) => {
            let mean = link.nodes.iter().map(|&n| t[n as usize]).sum::<f64>()
                / link.nodes.len().max(1) as f64;
            // The rows of the temperature closest to the mean.
            let temperature = rows
                .iter()
                .map(|r| r.2)
                .min_by(|a, b| (a - mean).abs().total_cmp(&(b - mean).abs()))
                .unwrap_or(0.0);
            let mut points: Vec<(f64, f64)> = rows
                .iter()
                .filter(|r| r.2 == temperature)
                .map(|r| (r.1, r.0))
                .collect();
            points.sort_by(|a, b| a.0.total_cmp(&b.0));
            let p = link.pressure;
            match points.as_slice() {
                [] => 0.0,
                [only] => only.1,
                _ if p <= points[0].0 => points[0].1,
                _ if p >= points[points.len() - 1].0 => points[points.len() - 1].1,
                _ => {
                    let i = points.windows(2).position(|w| p <= w[1].0).unwrap_or(0);
                    let (a, b) = (points[i], points[i + 1]);
                    a.1 + (b.1 - a.1) * (p - a.0) / (b.0 - a.0)
                }
            }
        }
    }
}

/// Adds the conduction across the closed contact points: `h A c cᵀ` per link.
fn add_links(model: &Model, links: &[ThermalLink], t: &[f64], k: &mut SparseMatrix) {
    for link in links {
        let h = conductance(model, link, t) * link.area;
        if h == 0.0 {
            continue;
        }
        let m = link.nodes.len();
        let mut block = vec![0.0; m * m];
        for i in 0..m {
            for j in 0..m {
                block[i * m + j] = h * link.coefficients[i] * link.coefficients[j];
            }
        }
        k.add_block(&link.nodes, &block);
    }
}

/// The heat flux `q = -k ∇T` at the integration points of every element (3 components,
/// stored in a field with 6 per point so that the stress extrapolation applies).
pub fn heat_flux(
    model: &Model,
    table: &ElementTable,
    heat: &HeatTable,
    t: &[f64],
) -> Result<IntegrationPointField> {
    let mut field = table.layout.field(6);
    for (index, entry) in table.entries.iter().enumerate() {
        let formulation = table.formulation(entry);
        let coords = table.coordinates(model, entry);
        let te: Vec<f64> = entry.nodes.iter().map(|&n| t[n as usize]).collect();
        let material = &model.materials[heat.material[index] as usize];
        let rows = material
            .conductivity
            .as_ref()
            .map(|c| c.rows.as_slice())
            .unwrap_or(&[]);
        let tp = formulation.material_temperatures(&formulation.rule, &te);
        let start = table.layout.offsets[index] as usize;
        for (p, (g, temperature)) in formulation
            .temperature_gradients(&coords, &te)
            .into_iter()
            .zip(tp)
            .enumerate()
        {
            let k = plxs_core::material::interpolate(rows, temperature).unwrap_or(0.0);
            let at = (start + p) * 6;
            for (v, gc) in field.values[at..at + 3].iter_mut().zip(g) {
                *v = -k * gc;
            }
        }
    }
    Ok(field)
}

/// Sets the values of nodes of plane and axisymmetric elements to zero: CalculiX writes no
/// RFL for them (checked with ccx 2.21 on CPS4, CPS8, CPE4 and CAX4).
fn zero_on_expanded(table: &ElementTable, values: &mut [f64]) {
    for entry in &table.entries {
        if table.formulation(entry).expansion().is_some() {
            for &n in &entry.nodes {
                values[n as usize] = 0.0;
            }
        }
    }
}
