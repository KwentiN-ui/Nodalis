//! `*FREQUENCY` and `*BUCKLE` (concept section 4.5, M2 T7).
//!
//! **Frequency**: `K φ = ω² M φ` with the consistent mass matrix, solved by shift-invert
//! Lanczos (`plxs_linalg::ShiftInvertLanczos`) on the free degrees of freedom; homogeneous
//! single-point constraints (the CalculiX manual: boundary conditions that may be nonzero
//! from earlier steps are set to zero). With `PERTURBATION` the stiffness is the tangent
//! stiffness of the state left by the last step (initial displacement and initial stress
//! stiffness, manual section "*STEP"). Modes are mass normalized; the `.dat` gets the
//! eigenvalue table, participation factors `Γ_j = φᵀ M r_j` for the six unit rigid body
//! motions `r_j` (translations along and rotations about the global axes through the
//! origin), the effective modal masses `Γ_j²` with their totals and the total effective
//! mass `r_jᵀ M r_j` (manual section "Frequency analysis", equations 242–244). Both
//! participation factors and total effective mass are evaluated on the free degrees of
//! freedom, as CalculiX's totals show (the mass of the fixed nodes is missing there).
//!
//! **Buckle**: a linear static solution for the step's loads gives the stresses `σ_b`
//! and their initial stress stiffness `K_σ`; `(K + λ K_σ) φ = 0` is solved for the
//! smallest positive factors `λ` (manual: "only the stress stiffness due to the buckling
//! load is taken into account and not the large deformation stiffness"). With
//! `PERTURBATION`, `K` is the tangent stiffness of the previous state. The `.frd` gets the
//! static solution as the first data set of the step (value 0) and then one set per mode
//! with the factor as its value; the `.dat` the factor table.
//!
//! No CalculiX code was consulted.

use plxs_assembly::{
    ElementTable, Spc, add_membrane_geometric_stiffness, assemble_geometric_stiffness,
    assemble_mass, assemble_stiffness, assemble_tangent_stiffness, load_vector,
};
use plxs_constraints::{ConstraintSet, ModelConstraints};
use plxs_core::results::{IncrementKind, ResultsIncrement};
use plxs_core::state::{IntegrationPointField, NodalField, StoredModes};
use plxs_core::step::{EquationSolver, Procedure as ProcedureKind, SolverMode};
use plxs_core::{Error, Model, Progress, ProgressEvent, Result, Step};
use plxs_linalg::Unknowns;
use plxs_linalg::{LanczosOptions, ShiftInvertLanczos, SparseMatrix};
use plxs_output::{EigenTable, StaRow};
use rayon::prelude::*;

use crate::equations::{Equations, in_increment};
use crate::fields::build_fields;
use crate::history::write_table;
use crate::{Procedure, ProcedureContext, StepOutcome};

pub struct FrequencyProcedure;
pub struct BuckleProcedure;

/// The stiffness matrix a perturbation step linearizes about: the tangent stiffness of
/// the state when a step was computed before, the linear stiffness otherwise (which is
/// also what a non-perturbative step uses; it is cached in the context).
pub(crate) fn reference_stiffness(
    ctx: &mut ProcedureContext<'_>,
    step: &Step,
    progress: &mut dyn Progress,
) -> Result<(SparseMatrix, bool)> {
    let started = std::time::Instant::now();
    let perturbed = step.perturbation
        && ctx.state.steps_done > 0
        && ctx.state.stresses.values.len() == ctx.table.layout.total() * 6
        && ctx.state.displacements.values.len() == ctx.table.dofs.len();
    let k = if perturbed {
        let mut k = SparseMatrix::zeros(ctx.pattern.clone());
        assemble_tangent_stiffness(
            ctx.table,
            ctx.model,
            &ctx.state.displacements.values,
            &ctx.state.stresses,
            &mut k,
        )?;
        crate::complex::add_spin_softening(ctx.model, ctx.table, step, &mut k)?;
        progress.event(ProgressEvent::Info(format!(
            "tangent stiffness of the preloaded state assembled in {:.2} s",
            started.elapsed().as_secs_f64()
        )));
        k
    } else {
        if ctx.stiffness.is_none() {
            let mut k = SparseMatrix::zeros(ctx.pattern.clone());
            assemble_stiffness(ctx.table, ctx.model, &mut k);
            ctx.stiffness = Some(k);
            progress.event(ProgressEvent::Info(format!(
                "stiffness assembled in {:.2} s",
                started.elapsed().as_secs_f64()
            )));
        }
        ctx.stiffness.clone().unwrap()
    };
    Ok((k, perturbed))
}

/// The displacements a perturbation step linearizes about; empty when the step is not
/// perturbed (the strains are then the linear ones).
pub(crate) fn preload_displacements(ctx: &ProcedureContext<'_>, perturbed: bool) -> Vec<f64> {
    if perturbed {
        ctx.state.displacements.values.clone()
    } else {
        Vec::new()
    }
}

/// Strain and stress pairs at the integration points of one element and its internal forces.
type ElementResults = (Vec<([f64; 6], [f64; 6])>, Vec<f64>);

/// Strains, stresses and nodal forces of a displacement field `du` of an eigenvalue step
/// as CalculiX writes them. With a preloaded state `u` they are linearized about it (total
/// Lagrangian): strains `ε = B_L(u) du`, stresses `σ = D ε`; with `u` empty they are the
/// linear ones (`B_L = B_0`). The nodal forces are the internal forces `∫ B_0ᵀ σ dV` in
/// both cases (for the reference solution of a preloaded buckling step this is not the load
/// vector: the initial stress stiffness of the preload is missing, which CalculiX's output
/// shares to all printed digits). The fields are returned in the state's layout.
pub(crate) fn increment_results(
    table: &ElementTable,
    model: &Model,
    u: &[f64],
    du: &[f64],
    strains: &mut IntegrationPointField,
    stresses: &mut IntegrationPointField,
    forces: &mut [f64],
) {
    let per_element: Vec<ElementResults> = table
        .entries
        .par_iter()
        .map(|entry| {
            let formulation = table.formulation(entry);
            let coords = table.coordinates(model, entry);
            let d = table.material_matrix(entry);
            let due = table.gather(entry, du);
            let strains = if u.is_empty() {
                formulation.strains(&coords, d, &due)
            } else {
                let ue = table.gather(entry, u);
                formulation.linearized_strains(&coords, d, &ue, &due)
            };
            let points: Vec<([f64; 6], [f64; 6])> = strains
                .into_iter()
                .map(|eps| {
                    let mut sigma = [0.0; 6];
                    for (r, sr) in sigma.iter_mut().enumerate() {
                        *sr = (0..6).map(|c| d[r][c] * eps[c]).sum();
                    }
                    (eps, sigma)
                })
                .collect();
            let sigma: Vec<[f64; 6]> = points.iter().map(|(_, s)| *s).collect();
            let mut fe = vec![0.0; entry.dofs.len()];
            formulation.internal_force(&coords, &sigma, &mut fe);
            (points, fe)
        })
        .collect();
    forces.iter_mut().for_each(|v| *v = 0.0);
    for (index, ((points, fe), entry)) in per_element.iter().zip(&table.entries).enumerate() {
        let start = table.layout.offsets[index] as usize;
        for (p, (eps, sigma)) in points.iter().enumerate() {
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
        for (dof, f) in entry.dofs.iter().zip(fe) {
            forces[*dof as usize] += f;
        }
    }
}

fn lanczos_options(accuracy: Option<f64>) -> LanczosOptions {
    let mut options = LanczosOptions::default();
    if let Some(accuracy) = accuracy {
        // CalculiX's "accuracy" is ARPACK's tolerance; the Ritz residual criterion here is
        // stricter, so never loosen it beyond the default.
        options.tolerance = options.tolerance.min(accuracy.abs().max(1e-14));
    }
    options
}

/// The homogeneous constraints of an eigenvalue step: its single-point constraints set to
/// zero (the CalculiX manual: boundary conditions that may be nonzero from earlier steps
/// are set to zero) and the model's multi-point constraints, eliminated together.
pub(crate) fn homogeneous_constraints(
    ctx: &ProcedureContext<'_>,
    step: &Step,
    progress: &mut dyn Progress,
) -> Result<ConstraintSet> {
    let spc = Spc::of_step(ctx.model, step, &ctx.table.dofs, 1.0, 1.0, 1.0)?;
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
    Ok(set)
}

/// A vector on all primary DOFs (global components) restricted to the reduced unknowns of
/// a constraint set: the value of every independent DOF, in the local system of a
/// transformed node. `values` may cover several copies of the model (cyclic symmetry).
pub(crate) fn restrict_to_free(
    set: &ConstraintSet,
    constraints: &ModelConstraints,
    values: &[f64],
) -> Vec<f64> {
    let nodes = (values.len() / 3).max(1) as u32;
    set.free
        .iter()
        .map(|&dof| {
            let (node, k) = (dof / 3, (dof % 3) as usize);
            match constraints.rotation(node % nodes) {
                None => values[dof as usize],
                Some(r) => (0..3)
                    .map(|d| r[d][k] * values[(node * 3) as usize + d])
                    .sum(),
            }
        })
        .collect()
}

/// The lowest `count` eigenpairs of `K φ = λ M φ` (above the lower frequency bound when
/// given) by shift-invert Lanczos; a singular stiffness (free structure) moves the shift
/// below the spectrum.
#[allow(clippy::too_many_arguments)]
pub(crate) fn lanczos_modes(
    ctx: &ProcedureContext<'_>,
    k_free: &SparseMatrix,
    m_free: &SparseMatrix,
    count: usize,
    lower: Option<f64>,
    solver: EquationSolver,
    coordinates: Vec<[f64; 3]>,
    progress: &mut dyn Progress,
) -> Result<plxs_linalg::EigenPairs> {
    let lower_shift = lower
        .filter(|f| *f > 0.0)
        .map(|f| (2.0 * std::f64::consts::PI * f).powi(2));
    let mut lanczos = ShiftInvertLanczos::new(ctx.threads, lanczos_options(None));
    // The positions of the unknowns make the nested dissection of the direct solver
    // geometric. Without them it bisects along level structures, which on a 600k-equation
    // mesh took minutes instead of seconds and filled more.
    lanczos.direct_mut().set_coordinates(Some(coordinates));
    let started = std::time::Instant::now();
    let mut shifts = vec![lower_shift.unwrap_or(0.0)];
    if lower_shift.is_none() {
        let scale = k_free.diagonal().iter().sum::<f64>()
            / m_free.diagonal().iter().sum::<f64>().max(f64::MIN_POSITIVE);
        shifts.extend([-1e-6 * scale, -1e-4 * scale, -1e-2 * scale]);
    }
    // A shift above zero (a lower frequency bound) makes K − σM indefinite, which the
    // iterative inner solver cannot take.
    let definite = lower_shift.is_none();
    let pairs = with_eigen_solver(
        &mut lanczos,
        solver,
        k_free,
        definite,
        progress,
        |lanczos, progress| {
            let mut last_error = None;
            for &shift in &shifts {
                match lanczos.frequencies(k_free, m_free, count, shift) {
                    Ok(p) => return Ok(p),
                    // A singular stiffness (free structure): the factorization says so, the
                    // iteration breaks down or does not converge.
                    Err(Error::Numerical { message, .. })
                        if (message.contains("singular") || lanczos.is_iterative())
                            && lower_shift.is_none() =>
                    {
                        progress.event(ProgressEvent::Info(format!(
                            "shift {shift:e}: {message}; trying a negative shift"
                        )));
                        last_error = Some(message);
                    }
                    Err(e) => return Err(e),
                }
            }
            Err(Error::Numerical {
                step: ctx.step_number,
                increment: 1,
                message: last_error.unwrap_or_else(|| "no eigenvalues found".into()),
            })
        },
    )?;
    let iterative = lanczos.is_iterative();
    progress.event(ProgressEvent::Info(format!(
        "{} eigenvalues in {:.2} s ({} Lanczos steps, residual {:.1e}{})",
        pairs.values.len(),
        started.elapsed().as_secs_f64(),
        pairs.steps,
        pairs.residual,
        if iterative {
            ", iterative inner solver, confirmed by restarts"
        } else if pairs.sturm_confirmed {
            ", Sturm check passed"
        } else {
            ", Sturm check NOT confirmed"
        }
    )));
    if !pairs.sturm_confirmed && !iterative {
        progress.event(ProgressEvent::Warning(
            "the Sturm sequence check could not confirm that no eigenvalue was missed".into(),
        ));
    }
    Ok(pairs)
}

/// Runs `solve` on `lanczos` with the inner solver the step's equation solver asks for:
/// the factorization, or conjugate gradients (only when the shifted matrices are positive
/// definite, `definite`). The automatic choice takes the iterative one when the factor of
/// `k` would not fit into the memory and falls back once to the other on a numerical
/// failure.
fn with_eigen_solver<T>(
    lanczos: &mut ShiftInvertLanczos,
    solver: EquationSolver,
    k: &SparseMatrix,
    definite: bool,
    progress: &mut dyn Progress,
    mut solve: impl FnMut(&mut ShiftInvertLanczos, &mut dyn Progress) -> Result<T>,
) -> Result<T> {
    let iterative = match solver.mode() {
        SolverMode::Direct => false,
        SolverMode::Iterative if !definite => {
            progress.event(ProgressEvent::Warning(
                "the iterative equation solver needs the shift at or below the spectrum \
                 (no lower frequency bound); solving directly"
                    .into(),
            ));
            false
        }
        SolverMode::Iterative => true,
        SolverMode::Automatic => {
            definite
                && k.dimension() >= crate::equations::AUTOMATIC_FROM
                && crate::equations::factor_exceeds_memory(lanczos.direct_mut(), k, None, progress)?
        }
    };
    lanczos.set_iterative(iterative);
    match solve(lanczos, progress) {
        Err(error @ Error::Numerical { .. })
            if solver.mode() == SolverMode::Automatic
                && (iterative || (definite && plxs_linalg::is_out_of_memory(&error))) =>
        {
            let message = match &error {
                Error::Numerical { message, .. } => message.clone(),
                _ => unreachable!(),
            };
            progress.event(ProgressEvent::Warning(format!(
                "{message}; solving {} instead",
                if iterative { "directly" } else { "iteratively" }
            )));
            lanczos.set_iterative(!iterative);
            solve(lanczos, progress)
        }
        other => other,
    }
}

/// The six unit rigid body motions on the full DOF vector: translations along the axes
/// and rotations about the axes through the origin.
pub(crate) fn rigid_body_motions(model: &Model, dofs: usize) -> Vec<Vec<f64>> {
    let mut motions = vec![vec![0.0; dofs]; 6];
    for (i, x) in model.nodes.coordinates.iter().enumerate() {
        let at = 3 * i;
        for k in 0..3 {
            motions[k][at + k] = 1.0;
        }
        // ω × x for unit rotations about x, y, z.
        motions[3][at + 1] = -x[2];
        motions[3][at + 2] = x[1];
        motions[4][at] = x[2];
        motions[4][at + 2] = -x[0];
        motions[5][at] = -x[1];
        motions[5][at + 1] = x[0];
    }
    motions
}

impl Procedure for FrequencyProcedure {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        let ProcedureKind::Frequency(parameters) = &step.procedure else {
            return Err(Error::Unsupported(
                "FrequencyProcedure needs a *FREQUENCY step".into(),
            ));
        };
        if parameters.cyclic_modes.is_some() && ctx.constraints.cyclic.is_some() {
            return crate::cyclic::cyclic_frequency(ctx, step, parameters, progress);
        }
        let count = parameters.count.max(1) as usize;
        let (k, perturbed) = reference_stiffness(ctx, step, progress)?;
        let u_pre = preload_displacements(ctx, perturbed);
        let mut m = SparseMatrix::zeros(ctx.pattern.clone());
        assemble_mass(ctx.table, ctx.model, &mut m)?;
        let reduction = homogeneous_constraints(ctx, step, progress)?;
        let k_free = reduction.reduce(&k);
        let m_free = reduction.reduce(&m);
        progress.event(ProgressEvent::Info(format!(
            "number of equations: {}; {count} eigenvalues requested",
            reduction.reduced_len()
        )));
        let pairs = lanczos_modes(
            ctx,
            &k_free,
            &m_free,
            count,
            parameters.lower,
            parameters.solver,
            Unknowns::from_free(&reduction.free, 3, &ctx.model.nodes.coordinates).coordinates,
            progress,
        )?;

        // Keep the modes within the requested range.
        let upper = parameters.upper.filter(|f| *f > 0.0);
        let total_dofs = ctx.table.dofs.len();
        let rigid = rigid_body_motions(ctx.model, total_dofs);
        let rigid_free: Vec<Vec<f64>> = rigid
            .iter()
            .map(|r| restrict_to_free(&reduction, ctx.constraints, r))
            .collect();
        let mut m_r: Vec<Vec<f64>> = Vec::with_capacity(6);
        for r in &rigid_free {
            let mut mr = vec![0.0; reduction.reduced_len()];
            m_free.multiply(r, &mut mr);
            m_r.push(mr);
        }
        let total_effective: Vec<f64> = rigid_free
            .iter()
            .zip(&m_r)
            .map(|(r, mr)| r.iter().zip(mr).map(|(a, b)| a * b).sum())
            .collect();

        let mut eigen_rows = Vec::new();
        let mut participation_rows = Vec::new();
        let mut modal_mass_rows = Vec::new();
        let mut modal_mass_total = vec![0.0; 6];
        let mut stored_values = Vec::new();
        let mut stored_modes = Vec::new();
        let mut stresses = ctx.table.layout.field(6);
        let mut strains = ctx.table.layout.field(6);
        let mut forces = vec![0.0; total_dofs];
        let mut mode_number = 0u32;
        let zeros = vec![0.0; total_dofs];
        for (lambda, phi_free) in pairs.values.iter().zip(&pairs.vectors) {
            let (omega, omega_im) = if *lambda >= 0.0 {
                (lambda.sqrt(), 0.0)
            } else {
                (0.0, (-lambda).sqrt())
            };
            let frequency = omega / (2.0 * std::f64::consts::PI);
            if upper.is_some_and(|f| frequency > f) {
                break;
            }
            mode_number += 1;
            progress.event(ProgressEvent::Eigenvalue {
                step: ctx.step_number,
                mode: mode_number,
                value: *lambda,
            });
            eigen_rows.push((mode_number, vec![*lambda, omega, frequency, omega_im]));
            let gamma: Vec<f64> = m_r
                .iter()
                .map(|mr| phi_free.iter().zip(mr).map(|(a, b)| a * b).sum::<f64>())
                .collect();
            let effective: Vec<f64> = gamma.iter().map(|g| g * g).collect();
            for (t, e) in modal_mass_total.iter_mut().zip(&effective) {
                *t += e;
            }
            participation_rows.push((mode_number, gamma));
            modal_mass_rows.push((mode_number, effective));

            let phi = reduction.expand(phi_free, &zeros);
            increment_results(
                ctx.table,
                ctx.model,
                &u_pre,
                &phi,
                &mut strains,
                &mut stresses,
                &mut forces,
            );
            let fields = build_fields(
                ctx.table, ctx.model, step, &phi, &forces, &stresses, &strains,
            );
            ctx.results.increments.push(ResultsIncrement {
                step: ctx.step_number,
                increment: mode_number,
                kind: IncrementKind::Frequency,
                value: frequency,
                total_time: ctx.state.total_time,
                nodal_diameter: None,
                fields,
            });
            if parameters.storage {
                stored_values.push(*lambda);
                stored_modes.push(NodalField {
                    per_node: 3,
                    values: phi,
                });
            }
        }
        modal_mass_rows.push((0, modal_mass_total));
        write_table(
            ctx.results,
            ctx.step_number,
            EigenTable::EIGENVALUES,
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
        if parameters.storage {
            ctx.state.stored_modes = Some(StoredModes {
                step: ctx.step_number,
                count: mode_number,
                eigenvalues: stored_values,
                modes: stored_modes,
                nodal_diameters: None,
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
}

impl Procedure for BuckleProcedure {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        let ProcedureKind::Buckle(parameters) = &step.procedure else {
            return Err(Error::Unsupported(
                "BuckleProcedure needs a *BUCKLE step".into(),
            ));
        };
        let count = parameters.count.max(1) as usize;
        let (k, perturbed) = reference_stiffness(ctx, step, progress)?;
        let reduction = homogeneous_constraints(ctx, step, progress)?;
        let total_dofs = ctx.table.dofs.len();
        let zeros = vec![0.0; total_dofs];
        let k_free = reduction.reduce(&k);
        progress.event(ProgressEvent::Info(format!(
            "number of equations: {}; {count} buckling factors requested",
            reduction.reduced_len()
        )));

        // Static solution for the step's loads (fully applied).
        let (f, warnings) = load_vector(
            ctx.model,
            ctx.table,
            step,
            1.0,
            ctx.state.total_time + 1.0,
            1.0,
        )?;
        for warning in warnings {
            progress.event(ProgressEvent::Warning(warning));
        }
        progress.event(ProgressEvent::Increment {
            step: ctx.step_number,
            increment: 1,
            attempt: 1,
            size: 1.0,
            step_time: 1.0,
            total_time: ctx.state.total_time + 1.0,
        });
        let started = std::time::Instant::now();
        // One factorization serves every iteration step: direct while it fits.
        let mut equations = Equations::new(parameters.solver).preferring_direct();
        let coordinates = || Unknowns::from_free(&reduction.free, 3, &ctx.model.nodes.coordinates);
        equations
            .prepare(&mut ctx.solver, &k_free, coordinates, true, progress)
            .map_err(|e| match e {
                Error::Numerical { message, .. } => Error::Numerical {
                    step: ctx.step_number,
                    increment: 1,
                    message: if equations.hints_constraints() {
                        format!("{message}; the model is probably not sufficiently constrained")
                    } else {
                        message
                    },
                },
                other => other,
            })?;
        let mut u_free = reduction.reduce_rhs(&f);
        equations
            .solve(&mut ctx.solver, &mut u_free, progress)
            .map_err(|e| in_increment(e, ctx.step_number, 1))?;
        progress.event(ProgressEvent::Info(format!(
            "reference solution in {:.2} s",
            started.elapsed().as_secs_f64()
        )));
        let u = reduction.expand(&u_free, &zeros);
        let mut stresses = ctx.table.layout.field(6);
        let mut strains = ctx.table.layout.field(6);
        let mut forces = vec![0.0; total_dofs];
        let u_pre = preload_displacements(ctx, perturbed);
        increment_results(
            ctx.table,
            ctx.model,
            &u_pre,
            &u,
            &mut strains,
            &mut stresses,
            &mut forces,
        );
        let fields = build_fields(ctx.table, ctx.model, step, &u, &forces, &stresses, &strains);
        ctx.results.increments.push(ResultsIncrement {
            step: ctx.step_number,
            increment: 0,
            kind: IncrementKind::Buckling,
            value: 0.0,
            total_time: ctx.state.total_time,
            nodal_diameter: None,
            fields,
        });

        // Initial stress stiffness of the buckling load and the eigenvalue problem.
        let mut k_sigma = SparseMatrix::zeros(ctx.pattern.clone());
        assemble_geometric_stiffness(ctx.table, ctx.model, &stresses, &mut k_sigma)?;
        add_membrane_geometric_stiffness(ctx.table, &u, &mut k_sigma);
        let k_sigma_free = reduction.reduce(&k_sigma);
        let started = std::time::Instant::now();
        let mut lanczos =
            ShiftInvertLanczos::new(ctx.threads, lanczos_options(parameters.accuracy));
        // The reference solution analyzed the same pattern with the node positions: the
        // Lanczos solver takes that solver over, with its ordering and the memory of its
        // factor, instead of ordering again and holding a second factor.
        let regularization = ctx.solver.regularization();
        std::mem::swap(lanczos.direct_mut(), &mut ctx.solver);
        lanczos.direct_mut().set_regularization(1e-12);
        // Taken by a fresh analysis only (the reference solution solved iteratively).
        lanczos.direct_mut().set_coordinates(Some(
            Unknowns::from_free(&reduction.free, 3, &ctx.model.nodes.coordinates).coordinates,
        ));
        let pairs = with_eigen_solver(
            &mut lanczos,
            parameters.solver,
            &k_free,
            true,
            progress,
            |lanczos, _| lanczos.buckling(&k_free, &k_sigma_free, count),
        );
        std::mem::swap(lanczos.direct_mut(), &mut ctx.solver);
        ctx.solver.set_regularization(regularization);
        // It holds the factor of a shifted matrix now, not of the stiffness.
        ctx.solver.discard_factor();
        let pairs = pairs?;
        let iterative = lanczos.is_iterative();
        progress.event(ProgressEvent::Info(format!(
            "{} buckling factors in {:.2} s ({} Lanczos steps, residual {:.1e}{})",
            pairs.values.len(),
            started.elapsed().as_secs_f64(),
            pairs.steps,
            pairs.residual,
            if iterative {
                ", iterative inner solver, confirmed by restarts"
            } else if pairs.sturm_confirmed {
                ", Sturm check passed"
            } else {
                ", Sturm check NOT confirmed"
            }
        )));
        if !pairs.sturm_confirmed && !iterative {
            progress.event(ProgressEvent::Warning(
                "the Sturm sequence check could not confirm that no buckling factor was missed"
                    .into(),
            ));
        }
        let mut rows = Vec::new();
        for (mode, (factor, phi_free)) in pairs.values.iter().zip(&pairs.vectors).enumerate() {
            let mode = mode as u32 + 1;
            progress.event(ProgressEvent::Eigenvalue {
                step: ctx.step_number,
                mode,
                value: *factor,
            });
            rows.push((mode, vec![*factor]));
            let mut phi = reduction.expand(phi_free, &zeros);
            normalize_buckling_mode(&k, &mut phi, &mut forces);
            // CalculiX derives the stresses of a buckling mode from the linear strains even
            // with a preload (the reference solution above and the modes of a perturbed
            // frequency step use the linearized ones); with the linearized strains the
            // shear stresses of `vorspannung-beulen` differ from CalculiX by a quarter,
            // with the linear ones by 1e-3 at the free end, where the reference modes
            // carry a trace of the second mode of the almost degenerate pair.
            increment_results(
                ctx.table,
                ctx.model,
                &[],
                &phi,
                &mut strains,
                &mut stresses,
                &mut forces,
            );
            let fields = build_fields(
                ctx.table, ctx.model, step, &phi, &forces, &stresses, &strains,
            );
            ctx.results.increments.push(ResultsIncrement {
                step: ctx.step_number,
                increment: mode,
                kind: IncrementKind::Buckling,
                value: *factor,
                total_time: ctx.state.total_time,
                nodal_diameter: None,
                fields,
            });
        }
        write_table(ctx.results, ctx.step_number, EigenTable::BUCKLING, rows);
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
}

/// Scales a buckling mode as CalculiX writes it: to unit strain energy in the reference
/// stiffness, `φᵀ K φ = 1` (checked against the reference files of `beulform` and
/// `vorspannung-beulen`, whose modes have exactly that norm). `work` is scratch of the
/// vector's length.
fn normalize_buckling_mode(k: &SparseMatrix, phi: &mut [f64], work: &mut [f64]) {
    k.multiply(phi, work);
    let energy: f64 = work.iter().zip(phi.iter()).map(|(a, b)| a * b).sum();
    if energy > 0.0 {
        let scale = 1.0 / energy.sqrt();
        for v in phi.iter_mut() {
            *v *= scale;
        }
    }
}
