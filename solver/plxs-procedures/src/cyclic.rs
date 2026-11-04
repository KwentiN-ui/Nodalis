//! Frequency steps of cyclic symmetric models with `*SELECT CYCLIC SYMMETRY MODES` (concept
//! section 4.3, M3 T15).
//!
//! For every nodal diameter `k` from `NMIN` to `NMAX` (default 0 to `N/2`) the eigenvalue
//! problem is solved on two copies of the sector, the real part `a` and the imaginary part
//! `b` of the complex mode, coupled by the cyclic constraints of
//! `plxs_constraints::cyclic` (`c_s = e^{-ikφ} R(φ) c_m`). Every eigenvalue comes twice:
//! with `(a, b)` also `(−b, a)` is a mode, the same complex mode times `i`. CalculiX lists
//! both in the `.dat` and writes only the first of each pair to the `.frd`, its real part as
//! `DISP`, `STRESS`, ... and its imaginary part as `DISPI`, `STRESSI`, ...; so does this
//! procedure. Modes are normalized to `aᵀ M a + bᵀ M b = 1`.
//!
//! The `.dat` gets per nodal diameter the eigenvalue table (with the nodal diameter as first
//! column), the participation factors, effective modal masses and total effective mass on
//! the reduced unknowns of both copies, and the turning direction of the modes: a mode of
//! nodal diameter `k` travels around the axis in the direction the sectors follow each
//! other, which CalculiX reports as backward against the opposite of that axis.
//!
//! No CalculiX code was consulted.

use std::sync::Arc;

use plxs_assembly::{Spc, assemble_mass};
use plxs_core::results::{IncrementKind, ResultField, ResultsIncrement};
use plxs_core::state::{NodalField, StoredModes};
use plxs_core::step::Frequency;
use plxs_core::{Error, Progress, ProgressEvent, Result, Step};
use plxs_linalg::{Pattern, SparseMatrix};
use plxs_output::{EigenTable, StaRow, imaginary_field_name};

use crate::eigen::{
    increment_results, lanczos_modes, preload_displacements, reference_stiffness, restrict_to_free,
    rigid_body_motions,
};
use crate::fields::build_fields;
use crate::history::write_table;
use crate::{ProcedureContext, StepOutcome};

/// The block-diagonal matrix `diag(A, A)`.
pub(crate) fn doubled(a: &SparseMatrix) -> SparseMatrix {
    let pattern = a.pattern();
    let n = pattern.dimension();
    let mut columns: Vec<Vec<u32>> = Vec::with_capacity(2 * n);
    for copy in [0, n as u32] {
        for j in 0..n {
            columns.push(pattern.column(j).iter().map(|&i| i + copy).collect());
        }
    }
    let doubled_pattern = Arc::new(Pattern::from_columns(2 * n, columns));
    let mut out = SparseMatrix::zeros(doubled_pattern.clone());
    let values = a.values();
    let col_ptr = pattern.col_ptr();
    let out_values = out.values_mut();
    for copy in [0, n as u32] {
        for (j, &start) in col_ptr.iter().take(n).enumerate() {
            let start = start as usize;
            for (offset, &i) in pattern.column(j).iter().enumerate() {
                let pos = doubled_pattern
                    .position(i + copy, j as u32 + copy)
                    .expect("the doubled pattern holds every entry");
                out_values[pos] = values[start + offset];
            }
        }
    }
    out
}

/// Appends the fields of the imaginary part to those of the real part, each after its
/// partner, with CalculiX's names (`DISP`, `DISPI`, `STRESS`, `STRESSI`, ...).
fn interleave(real: Vec<ResultField>, imaginary: Vec<ResultField>) -> Vec<ResultField> {
    let mut out = Vec::with_capacity(2 * real.len());
    for (r, mut i) in real.into_iter().zip(imaginary) {
        i.name = imaginary_field_name(&i.name);
        out.push(r);
        out.push(i);
    }
    out
}

pub(crate) fn cyclic_frequency(
    ctx: &mut ProcedureContext<'_>,
    step: &Step,
    parameters: &Frequency,
    progress: &mut dyn Progress,
) -> Result<StepOutcome> {
    let Some(cyclic) = ctx.constraints.cyclic.as_ref() else {
        return Err(Error::input(
            "*SELECT CYCLIC SYMMETRY MODES",
            "the model has no *CYCLIC SYMMETRY MODEL",
        ));
    };
    let modes = parameters.cyclic_modes.unwrap_or_default();
    let highest = cyclic.max_diameter();
    let nmin = modes.nmin.unwrap_or(0).min(highest);
    let nmax = modes.nmax.unwrap_or(highest).min(highest);
    if nmax < nmin {
        return Err(Error::input(
            "*SELECT CYCLIC SYMMETRY MODES",
            format!("NMAX={nmax} is smaller than NMIN={nmin}"),
        ));
    }
    let reference_axis = cyclic.axis.map(|c| -c);
    let count = parameters.count.max(1) as usize;
    let (k, perturbed) = reference_stiffness(ctx, step, progress)?;
    let u_pre = preload_displacements(ctx, perturbed);
    let mut m = SparseMatrix::zeros(ctx.pattern.clone());
    assemble_mass(ctx.table, ctx.model, &mut m)?;
    let k2 = doubled(&k);
    let m2 = doubled(&m);
    drop(k);

    let spc = Spc::of_step(ctx.model, step, &ctx.table.dofs, 1.0, 1.0, 1.0)?;
    let active = ctx.table.active_dofs();
    let nodes = ctx.model.nodes.len();
    let total_dofs = ctx.table.dofs.len();
    let rigid: Vec<Vec<f64>> = rigid_body_motions(ctx.model, total_dofs)
        .into_iter()
        .map(|mut r| {
            r.resize(2 * total_dofs, 0.0);
            r
        })
        .collect();
    let zeros = vec![0.0; 2 * total_dofs];
    let mut stresses = ctx.table.layout.field(6);
    let mut strains = ctx.table.layout.field(6);
    let mut forces = vec![0.0; total_dofs];
    let mut stored_values = Vec::new();
    let mut stored_modes = Vec::new();
    let mut stored_diameters = Vec::new();
    let upper = parameters.upper.filter(|f| *f > 0.0);
    let mut global_mode = 0u32;
    for diameter in nmin..=nmax {
        let Some(set) = ctx
            .constraints
            .cyclic_step_set(nodes, &active, &spc.dofs, diameter)
        else {
            unreachable!("the model has a cyclic symmetry");
        };
        for warning in &set.warnings {
            progress.event(ProgressEvent::Warning(warning.clone()));
        }
        let k_free = set.reduce(&k2);
        let m_free = set.reduce(&m2);
        progress.event(ProgressEvent::Info(format!(
            "nodal diameter {diameter}: number of equations: {}; {count} eigenvalues requested",
            set.reduced_len()
        )));
        let pairs = lanczos_modes(
            ctx,
            &k_free,
            &m_free,
            count,
            parameters.lower,
            parameters.solver,
            // Both copies of the sector at the nodes' positions.
            (set.free.iter())
                .map(|&d| ctx.model.nodes.coordinates[(d as usize % total_dofs) / 3])
                .collect(),
            progress,
        )?;

        let rigid_free: Vec<Vec<f64>> = rigid
            .iter()
            .map(|r| restrict_to_free(&set, ctx.constraints, r))
            .collect();
        let m_r: Vec<Vec<f64>> = rigid_free
            .iter()
            .map(|r| {
                let mut mr = vec![0.0; set.reduced_len()];
                m_free.multiply(r, &mut mr);
                mr
            })
            .collect();
        let total_effective: Vec<f64> = rigid_free
            .iter()
            .zip(&m_r)
            .map(|(r, mr)| r.iter().zip(mr).map(|(a, b)| a * b).sum())
            .collect();

        let mut eigen_rows = Vec::new();
        let mut participation_rows = Vec::new();
        let mut modal_mass_rows = Vec::new();
        let mut modal_mass_total = vec![0.0; 6];
        let mut turning_rows = vec![(0, reference_axis.to_vec())];
        let mut mode = 0u32;
        for (lambda, v) in pairs.values.iter().zip(&pairs.vectors) {
            let (omega, omega_im) = if *lambda >= 0.0 {
                (lambda.sqrt(), 0.0)
            } else {
                (0.0, (-lambda).sqrt())
            };
            let frequency = omega / (2.0 * std::f64::consts::PI);
            if upper.is_some_and(|f| frequency > f) {
                break;
            }
            mode += 1;
            global_mode += 1;
            progress.event(ProgressEvent::Eigenvalue {
                step: ctx.step_number,
                mode: global_mode,
                value: *lambda,
            });
            eigen_rows.push((
                mode,
                vec![f64::from(diameter), *lambda, omega, frequency, omega_im],
            ));
            let gamma: Vec<f64> = m_r
                .iter()
                .map(|mr| v.iter().zip(mr).map(|(a, b)| a * b).sum::<f64>())
                .collect();
            let effective: Vec<f64> = gamma.iter().map(|g| g * g).collect();
            for (t, e) in modal_mass_total.iter_mut().zip(&effective) {
                *t += e;
            }
            participation_rows.push((mode, gamma));
            modal_mass_rows.push((mode, effective));
            turning_rows.push((mode, vec![f64::from(diameter), -1.0]));

            // Only the first mode of each pair is written (the second is the same complex
            // mode times i).
            if mode.is_multiple_of(2) {
                continue;
            }
            let x = set.expand(v, &zeros);
            let (a, b) = x.split_at(total_dofs);
            let mut parts = Vec::with_capacity(2);
            for part in [a, b] {
                increment_results(
                    ctx.table,
                    ctx.model,
                    &u_pre,
                    part,
                    &mut strains,
                    &mut stresses,
                    &mut forces,
                );
                parts.push(build_fields(
                    ctx.table, ctx.model, step, part, &forces, &stresses, &strains,
                ));
            }
            let imaginary = parts.pop().unwrap();
            let real = parts.pop().unwrap();
            ctx.results.increments.push(ResultsIncrement {
                step: ctx.step_number,
                increment: global_mode,
                kind: IncrementKind::Frequency,
                value: frequency,
                total_time: ctx.state.total_time,
                nodal_diameter: Some(diameter),
                fields: interleave(real, imaginary),
            });
            if parameters.storage {
                stored_values.push(*lambda);
                stored_diameters.push(diameter);
                stored_modes.push(NodalField {
                    per_node: 3,
                    values: x,
                });
            }
        }
        modal_mass_rows.push((0, modal_mass_total));
        write_table(
            ctx.results,
            ctx.step_number,
            EigenTable::CYCLIC_EIGENVALUES,
            eigen_rows,
        );
        write_table(
            ctx.results,
            ctx.step_number,
            EigenTable::PARTICIPATION,
            participation_rows,
        );
        write_table(
            ctx.results,
            ctx.step_number,
            EigenTable::EFFECTIVE_MODAL_MASS,
            modal_mass_rows,
        );
        write_table(
            ctx.results,
            ctx.step_number,
            EigenTable::TOTAL_EFFECTIVE_MASS,
            vec![(0, total_effective)],
        );
        write_table(
            ctx.results,
            ctx.step_number,
            EigenTable::CYCLIC_TURNING_DIRECTION,
            turning_rows,
        );
    }
    if parameters.storage {
        // The modes on both copies (real part, then imaginary part) with their diameters.
        ctx.state.stored_modes = Some(StoredModes {
            step: ctx.step_number,
            count: stored_values.len() as u32,
            eigenvalues: stored_values,
            modes: stored_modes,
            nodal_diameters: Some(stored_diameters),
        });
    }
    ctx.sta.push(StaRow {
        step: ctx.step_number,
        increment: 1,
        attempt: 1,
        iterations: 1,
        total_time: ctx.state.total_time,
        step_time: 0.0,
        increment_size: 0.0,
    });
    ctx.state.steps_done = ctx.step_number;
    Ok(StepOutcome::Converged)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn doubled_matrix_repeats_the_blocks() {
        let p = Arc::new(Pattern::from_blocks(2, [&[0u32, 1][..]]));
        let mut a = SparseMatrix::zeros(p);
        a.add_block(&[0, 1], &[2.0, -1.0, -1.0, 3.0]);
        let d = doubled(&a);
        assert_eq!(d.dimension(), 4);
        assert_eq!(d.get(2, 2), 2.0);
        assert_eq!(d.get(2, 3), -1.0);
        assert_eq!(d.get(3, 3), 3.0);
        assert_eq!(d.get(0, 2), 0.0);
    }
}
