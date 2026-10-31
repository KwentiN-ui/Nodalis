//! `*STATIC` without geometric nonlinearity: one linear solve at the end of the step, loads
//! and prescribed displacements scaled by their amplitudes at the step's end time.

use plxs_assembly::discrete::{DiscreteElement, DiscreteKind};
use plxs_assembly::{
    ElementTable, Spc, assemble_stiffness, load_vector, rotate_concentrated_loads,
};
use plxs_core::state::IntegrationPointField;
use plxs_core::step::Procedure as ProcedureKind;
use plxs_core::{Error, Model, Progress, ProgressEvent, Result, Step};
use plxs_linalg::{LinearSolver, SparseMatrix};
use plxs_output::StaRow;
use rayon::prelude::*;

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
        // A linear step is solved once, at its end: CalculiX ignores the initial increment
        // and `DIRECT` of a purely linear *STATIC step and reports a single increment of
        // the size of the period (checked with ccx 2.21; the increments only show with
        // NLGEOM or other nonlinearities, which are not this procedure's).
        let period = parameters.increments.period();
        if period <= 0.0 {
            return Err(Error::input("*STATIC", "the step period must be positive"));
        }
        let (initial, count) = (period, 1);

        // Stiffness: assembled once per model, reused by every linear step.
        if ctx.stiffness.is_none() {
            let started = std::time::Instant::now();
            let mut k = SparseMatrix::zeros(ctx.pattern.clone());
            assemble_stiffness(ctx.table, ctx.model, &mut k);
            ctx.stiffness = Some(k);
            progress.event(ProgressEvent::Info(format!(
                "stiffness assembled in {:.2} s",
                started.elapsed().as_secs_f64()
            )));
        }
        let k = ctx.stiffness.take().unwrap();
        let outcome = Self::increments(ctx, step, progress, &k, period, initial, count);
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
        initial: f64,
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
        // Gap status the current factorization belongs to (all open before the first).
        let mut factored: Option<Vec<bool>> = None;
        let mut closed: Vec<bool> = gaps
            .iter()
            .map(|g| {
                g.gap_opening(&vec![0.0; total_dofs])
                    .is_some_and(|o| o <= 0.0)
            })
            .collect();

        let start_total = ctx.state.total_time;
        // Displacements at the end of the previous step (`*BOUNDARY, FIXED`).
        let previous = ctx.state.displacements.values.clone();
        let mut u = vec![0.0; total_dofs];
        let mut stresses = ctx.table.layout.field(6);
        let mut strains = ctx.table.layout.field(6);
        let mut reactions = vec![0.0; total_dofs];
        for increment in 1..=count {
            let time = (f64::from(increment) * initial).min(period);
            let size = time - (f64::from(increment - 1) * initial).min(period);
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
                    ctx.solver.analyze(k_red.pattern())?;
                    ctx.solver.factorize(&k_red).map_err(|e| match e {
                        Error::Numerical { message, .. } => Error::Numerical {
                            step: ctx.step_number,
                            increment,
                            message: format!(
                                "{message}; the model is probably not sufficiently constrained"
                            ),
                        },
                        other => other,
                    })?;
                    progress.event(ProgressEvent::Info(format!(
                        "stiffness factorized in {:.2} s",
                        started.elapsed().as_secs_f64()
                    )));
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
                ctx.solver.solve(&mut v)?;
                progress.event(ProgressEvent::Info(format!(
                    "solved in {:.2} s",
                    started.elapsed().as_secs_f64()
                )));
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

            progress.event(ProgressEvent::Iteration {
                step: ctx.step_number,
                increment,
                iteration: iterations,
                residual: 0.0,
                correction: rhs.iter().fold(0.0_f64, |m, v| m.max(v.abs())),
                converged: true,
            });

            integration_point_results(ctx.table, ctx.model, &u, &mut strains, &mut stresses);
            ctx.state.displacements.per_node = 3;
            ctx.state.displacements.values.clear();
            ctx.state.displacements.values.extend_from_slice(&u);
            ctx.state.stresses = stresses.clone();
            ctx.state.strains = strains.clone();

            if writes_increment(step, increment, count) {
                write_increment(
                    ctx, step, increment, time, total, &u, &reactions, &stresses, &strains,
                )?;
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
            );
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
fn writes_increment(step: &Step, increment: u32, count: u32) -> bool {
    let frequency = step
        .outputs
        .node_file
        .iter()
        .chain(&step.outputs.el_file)
        .chain(&step.outputs.contact_file)
        .map(|o| o.frequency.max(1))
        .min()
        .unwrap_or(1);
    increment == count || increment.is_multiple_of(frequency)
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
            let e = formulation.strains(&coords, d, &ue);
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
