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
use plxs_core::results::{IncrementKind, ResultField, ResultsIncrement};
use plxs_core::state::{IntegrationPointField, NodalField};
use plxs_core::step::{DefaultAmplitude, HeatTransfer, Procedure as ProcedureKind};
use plxs_core::{Error, Model, Progress, ProgressEvent, Result, Step};
use plxs_linalg::{LdltSolver, LinearSolver, SparseMatrix};
use plxs_output::StaRow;

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
        heat.check(ctx.model, transient)?;
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
    deltmx: Option<f64>,
    nonlinear: bool,
    k: SparseMatrix,
    c: Option<SparseMatrix>,
    tangent: SparseMatrix,
    solver: LdltSolver,
    /// Free set and matrix values of the last factorization.
    factorized: Option<(Vec<u32>, Vec<f64>)>,
    assembled: bool,
    /// Prescribed mechanical DOFs of the last mechanical factorization.
    mechanical_spc: Option<Vec<u32>>,
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
        let initial = p.increments.initial().min(period);
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
            maximum: p.increments.maximum.unwrap_or(period).min(period),
            deltmx: p.deltmx.filter(|d| *d > 0.0),
            nonlinear,
            k: SparseMatrix::zeros(heat.pattern.clone()),
            c: transient.then(|| SparseMatrix::zeros(heat.pattern.clone())),
            tangent: SparseMatrix::zeros(heat.pattern.clone()),
            solver: LdltSolver::new(ctx.threads),
            factorized: None,
            assembled: false,
            mechanical_spc: None,
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

        let mut step_time = 0.0;
        let mut dt = self.initial;
        let mut increment = 0u32;
        // Iterations of the increments done so far in this step.
        let mut done: Vec<u32> = Vec::new();
        let end = self.period * (1.0 - 1e-9);
        while step_time < end {
            increment += 1;
            if increment > step.max_increments {
                return Err(Error::Numerical {
                    step: ctx.step_number,
                    increment,
                    message: format!(
                        "the step needs more than INC={} increments",
                        step.max_increments
                    ),
                });
            }
            let mut attempt = 0u32;
            let (accepted, size) = loop {
                attempt += 1;
                let size = dt.min(self.period - step_time);
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
                                    format!(
                                        "largest temperature change {change:.4e} exceeds DELTMX={limit}"
                                    ),
                                ))
                            }
                            _ => break (inc, size),
                        }
                    }
                    Err(Error::Numerical { message, .. }) if !self.direct => {
                        Some((size * CUTBACK, message))
                    }
                    Err(e) => return Err(e),
                };
                let (smaller, reason) = reject.unwrap();
                progress.event(ProgressEvent::Cutback {
                    step: ctx.step_number,
                    increment,
                    attempt,
                    reason: reason.clone(),
                });
                if smaller < self.minimum {
                    return Err(Error::Numerical {
                        step: ctx.step_number,
                        increment,
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
            let write = writes_increment(step, increment, u32::MAX)
                || (self.period - time).abs() <= 1e-9 * self.period;
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
            dt = size;
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
                self.solver.solve(&mut rhs)?;
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
        if !same_free {
            self.solver.analyze(reduced.pattern())?;
        }
        self.solver.factorize(&reduced)?;
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
    ) -> Result<()> {
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
                ctx.solver.analyze(k_free.pattern())?;
                ctx.solver.factorize(&k_free).map_err(|e| match e {
                    Error::Numerical { message, .. } => Error::Numerical {
                        step: ctx.step_number,
                        increment,
                        message: format!(
                            "{message}; the model is probably not sufficiently constrained"
                        ),
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
            ctx.solver.solve(&mut rhs)?;
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
                th.correct_stresses(e, &mut stresses);
            }
            ctx.state.displacements.per_node = 3;
            ctx.state.displacements.values.clear();
            ctx.state.displacements.values.extend_from_slice(&u);
            ctx.state.stresses = stresses.clone();
            ctx.state.strains = strains.clone();
            if write {
                write_increment(
                    ctx, step, increment, time, total, &u, &forces, &stresses, &strains,
                )?;
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
            );
            Ok(())
        })();
        ctx.stiffness = Some(k);
        result
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
            let mut kt = vec![0.0; t.len()];
            self.k.multiply(t, &mut kt);
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
