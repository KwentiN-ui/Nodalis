//! `*STATIC` without geometric nonlinearity: one linear solve at the end of the step, loads
//! and prescribed displacements scaled by their amplitudes at the step's end time.

use crate::profile::Phase;
use plxs_assembly::discrete::{DiscreteElement, DiscreteKind};
use plxs_assembly::{
    ElementTable, Spc, assemble_stiffness, load_vector, rotate_concentrated_loads,
};
use plxs_core::state::IntegrationPointField;
use plxs_core::step::Incrementation;
use plxs_core::step::Procedure as ProcedureKind;
use plxs_core::{Error, Model, Progress, ProgressEvent, Result, Step};
use plxs_linalg::SparseMatrix;
use plxs_output::StaRow;
use rayon::prelude::*;

use crate::equations::{Equations, in_increment};
use crate::fields::write_increment;
use crate::{Procedure, ProcedureContext, StepOutcome};

pub struct StaticLinear;

impl Procedure for StaticLinear {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        let ProcedureKind::Static(parameters) = &step.procedure else {
            return Err(Error::Unsupported(
                "StaticLinear needs a *STATIC step".into(),
            ));
        };
        // A linear step is solved once per result frame (`FRAMES=`, Nodalis), with the
        // one factorization; without frames once, at its end: CalculiX ignores the initial
        // increment and `DIRECT` of a purely linear *STATIC step and reports a single
        // increment of the size of the period (checked with ccx 2.21).
        let period = parameters.increments.period();
        if period <= 0.0 {
            return Err(Error::input("*STATIC", "the step period must be positive"));
        }
        let count = parameters.increments.frames.unwrap_or(1).max(1);
        let framed = parameters.increments.frames.is_some();

        // Stiffness: assembled once per model, reused by every linear step.
        if ctx.stiffness.is_none() {
            let started = std::time::Instant::now();
            let mut k = SparseMatrix::zeros(ctx.pattern.clone());
            ctx.table.require_elastic()?;
            assemble_stiffness(ctx.table, ctx.model, &mut k);
            ctx.profile.add(crate::profile::Phase::Assembly, started);
            ctx.stiffness = Some(k);
            progress.event(ProgressEvent::Info(format!(
                "stiffness assembled in {:.2} s",
                started.elapsed().as_secs_f64()
            )));
        }
        let k = ctx.stiffness.take().unwrap();
        let outcome = Self::increments(ctx, step, progress, &k, period, framed, count);
        ctx.stiffness = Some(k);
        outcome
    }
}

impl StaticLinear {
    #[allow(clippy::too_many_arguments)]
    fn increments(
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
        k: &SparseMatrix,
        period: f64,
        framed: bool,
        count: u32,
    ) -> Result<StepOutcome> {
        let total_dofs = ctx.table.dofs.len();

        // The prescribed DOFs do not change within the step; their values may. SPCs and
        // multi-point constraints are eliminated together: u = T v + g.
        let spc = Spc::of_step(
            ctx.model,
            step,
            &ctx.table.dofs,
            period,
            ctx.state.total_time + period,
            period,
        )?;
        let active = ctx.table.active_dofs();
        let set = ctx
            .constraints
            .step_set(ctx.model.nodes.len(), &active, &spc.dofs);
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
        let gaps: Vec<&DiscreteElement> = ctx
            .table
            .discrete
            .iter()
            .filter(|e| matches!(e.kind, DiscreteKind::Gap { .. }))
            .collect();
        let solver_choice = match &step.procedure {
            ProcedureKind::Static(p) => p.solver,
            _ => Default::default(),
        };
        let mut equations = Equations::new(solver_choice).for_model(ctx.model);
        // Gap status the current factorization belongs to (all open before the first).
        let mut factored: Option<Vec<bool>> = None;
        let mut closed: Vec<bool> = gaps
            .iter()
            .map(|g| {
                g.gap_opening(&vec![0.0; total_dofs])
                    .is_some_and(|o| o <= 0.0)
            })
            .collect();

        // Displacements of a global model at the submodel's driven nodes.
        let submodel =
            crate::submodel::SubmodelValues::of_step(ctx.model, step, &ctx.work_dir, progress)?;
        let start_total = ctx.state.total_time;
        // Displacements at the end of the previous step (`*BOUNDARY, FIXED`).
        let previous = ctx.state.displacements.values.clone();
        // Temperatures (`*INITIAL CONDITIONS`, `*TEMPERATURE`, an earlier heat step) and
        // the thermal strains they cause.
        let temperatures =
            crate::thermal::StepTemperatures::of_step(ctx.model, step, ctx.state, &ctx.work_dir)?;
        let heat = if temperatures.is_active() {
            Some(plxs_assembly::HeatTable::new(ctx.model, ctx.table)?)
        } else {
            None
        };
        let thermal = heat
            .as_ref()
            .and_then(|h| crate::thermal::ThermalStrains::new(ctx.model, ctx.table, h));
        let mut u = vec![0.0; total_dofs];
        let mut stresses = ctx.table.layout.field(6);
        let mut strains = ctx.table.layout.field(6);
        let mut reactions = vec![0.0; total_dofs];
        for increment in 1..=count {
            if ctx.cancel.is_cancelled() {
                return Ok(StepOutcome::Cancelled);
            }
            let time = Incrementation::frame_time(period, count, increment);
            let size = time - Incrementation::frame_time(period, count, increment - 1);
            let total = start_total + time;
            progress.event(ProgressEvent::Increment {
                step: ctx.step_number,
                increment,
                attempt: 1,
                size,
                step_time: time,
                total_time: total,
            });

            let mut spc = Spc::of_step(ctx.model, step, &ctx.table.dofs, time, total, period)?;
            spc.apply_fixed(ctx.model, step, &ctx.table.dofs, &previous, &|node| {
                ctx.constraints.rotation(node).copied()
            });
            submodel.apply(
                ctx.model,
                step,
                &ctx.table.dofs,
                &mut spc,
                time,
                total,
                period,
                &|node| ctx.constraints.rotation(node).copied(),
            );
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
            // What a following static step ramps its loads from (without the thermal part).
            let applied = f.clone();
            let t = temperatures
                .is_active()
                .then(|| temperatures.at(ctx.model, step, time, total, period));
            let thermal_strains = thermal
                .as_ref()
                .zip(t.as_ref())
                .map(|(th, t)| th.strains(t));
            let f_th = thermal.as_ref().zip(t.as_ref()).map(|(th, t)| th.load(t));
            if let Some(f_th) = &f_th {
                for (a, b) in f.iter_mut().zip(f_th) {
                    *a += b;
                }
            }
            if increment == 1 {
                for warning in warnings {
                    progress.event(ProgressEvent::Warning(warning));
                }
            }
            let g = set.particular(&spc.values);

            // Gaps change the stiffness with their status: solve until it settles
            // (contact without friction, so the result does not depend on the path).
            let mut iterations = 0u32;
            let mut v;
            let mut k_used;
            loop {
                iterations += 1;
                k_used = if gaps.is_empty() {
                    None
                } else {
                    let mut kc = k.clone();
                    for (gap, &c) in gaps.iter().zip(&closed) {
                        if c {
                            kc.add_block(&gap.dofs, &gap.closed_stiffness());
                        }
                    }
                    Some(kc)
                };
                let k_now = k_used.as_ref().unwrap_or(k);
                if factored.as_ref() != Some(&closed) {
                    let started = std::time::Instant::now();
                    let k_red = set.reduce(k_now);
                    ctx.profile.add(Phase::Reduction, started);
                    let coordinates =
                        || crate::equations::displacement_unknowns(ctx.model, &set.free);
                    let factorization = std::time::Instant::now();
                    equations
                        .prepare(&mut ctx.solver, &k_red, coordinates, true, progress)
                        .map_err(|e| match e {
                            Error::Numerical {
                                    failure, message, ..
                            } => Error::Numerical {
                                step: ctx.step_number,
                                increment,
                                failure,
                                message: if equations.is_iterative() {
                                    message
                                } else {
                                    format!(
                                        "{message}; the model is probably not sufficiently constrained"
                                    )
                                },
                            },
                            other => other,
                        })?;
                    ctx.profile.add(Phase::Factorization, factorization);
                    factored = Some(closed.clone());
                }
                let mut f_now = f.clone();
                for (gap, &c) in gaps.iter().zip(&closed) {
                    if c {
                        gap.add_clearance_force(&mut f_now);
                    }
                }
                let mut k_g = vec![0.0; total_dofs];
                k_now.multiply(&g, &mut k_g);
                let rhs_full: Vec<f64> = f_now.iter().zip(&k_g).map(|(f, kg)| f - kg).collect();
                v = set.reduce_rhs(&rhs_full);
                let started = std::time::Instant::now();
                equations
                    .solve(&mut ctx.solver, &mut v, progress)
                    .map_err(|e| in_increment(e, ctx.step_number, increment))?;
                ctx.profile.add(Phase::Solve, started);
                u = set.expand(&v, &g);
                let status: Vec<bool> = gaps
                    .iter()
                    .map(|gap| gap.gap_opening(&u).is_some_and(|o| o <= 0.0))
                    .collect();
                if status == closed || iterations >= 100 {
                    break;
                }
                closed = status;
            }
            let rhs = v;

            // Nodal forces as CalculiX reports them (`RF`): the internal forces K u at every
            // node, which are the reactions at prescribed DOFs, the applied loads at loaded
            // nodes and zero (up to round-off) elsewhere.
            k_used.as_ref().unwrap_or(k).multiply(&u, &mut reactions);
            // With thermal strains the internal forces are K u - f_th.
            if let Some(f_th) = &f_th {
                for (a, b) in reactions.iter_mut().zip(f_th) {
                    *a -= b;
                }
            }
            // The forces the constraints carry to their independent nodes.
            ctx.constraints
                .transfer_forces(&mut reactions, &spc.dofs, &active);

            progress.event(ProgressEvent::Iteration {
                step: ctx.step_number,
                increment,
                iteration: iterations,
                residual: 0.0,
                correction: rhs.iter().fold(0.0_f64, |m, v| m.max(v.abs())),
                residual_limit: None,
                correction_limit: None,
                converged: true,
            });

            let results_started = std::time::Instant::now();
            integration_point_results(ctx.table, ctx.model, &u, &mut strains, &mut stresses);
            if let (Some(th), Some(e)) = (&thermal, &thermal_strains) {
                th.correct_stresses(e, &mut strains, &mut stresses);
            }
            if let Some(t) = &t {
                ctx.state.temperatures = Some(plxs_core::state::NodalField {
                    per_node: 1,
                    values: t.clone(),
                });
            }
            ctx.state.displacements.per_node = 3;
            ctx.state.displacements.values.clear();
            ctx.state.displacements.values.extend_from_slice(&u);
            ctx.state.stresses = stresses.clone();
            ctx.state.strains = strains.clone();
            ctx.state.loads = Some(applied);

            if framed || writes_increment(step, increment, count) {
                write_increment(
                    ctx, step, increment, time, total, &u, &reactions, &stresses, &strains, None,
                )?;
                crate::thermal::insert_temperatures(ctx, step, t.as_deref());
            }
            crate::history::write_history(
                ctx.results,
                ctx.model,
                ctx.table,
                step,
                ctx.step_number,
                increment,
                total,
                &u,
                &reactions,
                &stresses,
                &strains,
                None,
            );
            for request in &step.outputs.contact_print {
                ctx.results.history.extend(crate::support::history_blocks(
                    ctx.model,
                    request,
                    &u,
                    ctx.step_number,
                    increment,
                    total,
                ));
            }
            ctx.profile.add(Phase::Results, results_started);
            ctx.sta.push(StaRow {
                step: ctx.step_number,
                increment,
                attempt: 1,
                iterations,
                total_time: total,
                step_time: time,
                increment_size: size,
            });
            progress.event(ProgressEvent::IncrementDone {
                step: ctx.step_number,
                increment,
                attempt: 1,
                iterations,
                total_time: total,
                step_time: time,
                size,
            });
        }
        ctx.state.total_time = start_total + period;
        ctx.state.steps_done = ctx.step_number;
        Ok(StepOutcome::Converged)
    }
}

/// Whether the field output of an increment is written: every `FREQUENCY`-th increment
/// (the smallest frequency of the requests), the last one always.
pub(crate) fn writes_increment(step: &Step, increment: u32, count: u32) -> bool {
    writes_increment_at(step, increment, increment == count)
}

/// Whether the field output of an increment is written when `last` says it is the last one.
pub fn writes_increment_at(step: &Step, increment: u32, last: bool) -> bool {
    let frequency = step
        .outputs
        .node_file
        .iter()
        .chain(&step.outputs.el_file)
        .chain(&step.outputs.contact_file)
        .map(|o| o.frequency.max(1))
        .min()
        .unwrap_or(1);
    last || increment.is_multiple_of(frequency)
}

/// Strains and stresses at the integration points of every element.
/// Strains and stresses of one element at its integration points.
type ElementFields = (Vec<[f64; 6]>, Vec<[f64; 6]>);

pub fn integration_point_results(
    table: &ElementTable,
    model: &Model,
    u: &[f64],
    strains: &mut IntegrationPointField,
    stresses: &mut IntegrationPointField,
) {
    let per_element: Vec<ElementFields> = table
        .entries
        .par_iter()
        .map(|entry| {
            let formulation = table.formulation(entry);
            let coords = table.coordinates(model, entry);
            let d = table.material_matrix(entry);
            let ue = table.gather(entry, u);
            let mut e = formulation.strains(&coords, d, &ue);
            // Plane stress: the thickness strain that makes the thickness stress zero (the
            // displacements leave it at zero; thermal strains add theirs later).
            for eps in &mut e {
                if let Some(ezz) = table.thickness_strain(entry, eps, 0.0) {
                    eps[2] = ezz;
                }
            }
            let s: Vec<[f64; 6]> = e
                .iter()
                .map(|eps| {
                    let mut sigma = [0.0; 6];
                    for (r, sr) in sigma.iter_mut().enumerate() {
                        *sr = (0..6).map(|c| d[r][c] * eps[c]).sum();
                    }
                    sigma
                })
                .collect();
            (e, s)
        })
        .collect();
    for (index, (e, s)) in per_element.iter().enumerate() {
        let start = table.layout.offsets[index] as usize;
        for (p, (eps, sigma)) in e.iter().zip(s).enumerate() {
            let at = (start + p) * 6;
            // Tensor shear strains in the state (half the engineering ones).
            strains.values[at..at + 6].copy_from_slice(&[
                eps[0],
                eps[1],
                eps[2],
                0.5 * eps[3],
                0.5 * eps[4],
                0.5 * eps[5],
            ]);
            stresses.values[at..at + 6].copy_from_slice(sigma);
        }
    }
}
