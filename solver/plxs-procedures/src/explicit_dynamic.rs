//! `*DYNAMIC, INTEGRATION=EXPLICIT`: central differences with a lumped mass matrix
//! (concept `KONZEPT_EXPLICIT_DYNAMICS.md`, section 2).
//!
//! Every time step costs the internal forces of the elements and a division per degree of
//! freedom: no equation system, no Newton iterations, nothing that can fail to converge.
//! The price is the time step, which must stay below the time a wave needs to cross the
//! smallest element. Impacts, crashes and drop tests, which the implicit integration can
//! only follow with tiny increments anyway, are its field.
//!
//! **Integration** (Newmark β = 0, γ = 1/2 with staggered velocities and a variable step):
//!
//! ```text
//! a_n       = M_L⁻¹ (p_n − f_int(u_n) − f_v(v_{n−½}) − f_d(v_{n−½}))
//! v_{n+½}   = v_{n−½} + (Δt_n + Δt_{n+1})/2 · a_n
//! u_{n+1}   = u_n + Δt_{n+1} v_{n+½}
//! ```
//!
//! with the internal forces of the nonlinear assembly (total Lagrange under `NLGEOM`, the
//! material laws updated once per step: the return mapping of plasticity is strain driven
//! and needs no iteration), the external loads `p`, the bulk viscosity `f_v` and the
//! damping `f_d`. Constraints are eliminated as in the implicit procedures, `u = T q + g`:
//! the unknowns `q` accelerate with the reduced mass `Tᵀ M_L T`, which stays diagonal for
//! single-point constraints and is factorized once per step where multi-point constraints
//! couple DOFs (a sparse LDLᵀ of small blocks). Prescribed displacements `g(t)` move their
//! DOFs exactly; their acceleration enters the coupled DOFs as `M_L g̈`.
//!
//! **Lumped mass**: the diagonal of the consistent element mass (full integration rule),
//! scaled to the element mass per direction (Hinton, Rock and Zienkiewicz 1976). Row sums
//! would be zero or negative at the corner nodes of quadratic elements; the scaled diagonal
//! stays positive for every family.
//!
//! **Stable time step**: `Δt = 2/ω_max` with `ω_max` bounded by the largest eigenfrequency
//! of the elements (Hughes: `ω_max ≤ max_e ω_e`), each from `M_L,e⁻¹ K_e` by a few power
//! iterations (with Gershgorin's bound as a ceiling). It holds for every family, for
//! distorted elements, hourglass stiffness and for the thickness direction of expanded
//! shells. A safety factor of 0.9 and the damping at `ω_max` (`Δt (√(1+ξ²) − ξ)`) reduce it.
//!
//! **Bulk viscosity**: a linear pressure `q = b₁ ρ c_d L_e ε̇_vol` (b₁ = 0.06) on the mean
//! volumetric strain rate of every element damps the ringing of the highest modes; its
//! work goes into the dissipated energy.
//!
//! **Damping**: a damping ratio damps the mass-proportional part only (`α_R = 2ζω₁` at the
//! lowest elastic eigenfrequency): stiffness-proportional damping would shrink the time
//! step to nothing at the highest frequencies the mesh carries. Rayleigh coefficients are
//! taken as given, `β_R` with the time step reduced for it (and a note).
//!
//! **Output**: the result frames lie on the increments the step asks for (`initial` of the
//! incrementation is the frame interval); within a frame the time steps are equal, so that
//! the last one ends exactly on the frame. The velocities of a frame are those of the full
//! step, `v_n = v_{n−½} + Δt_n/2 a_n`. Energies: kinetic, internal (work of the internal
//! forces), external work (including that of prescribed displacements), dissipated (bulk
//! viscosity and damping); their balance error is reported.
//!
//! No CalculiX code was consulted.

use plxs_assembly::nonlinear::material_state;
use plxs_assembly::{ElementTable, Spc, load_vector, rotate_concentrated_loads};
use plxs_constraints::ConstraintSet;
use plxs_core::state::{IntegrationPointField, NodalField};
use plxs_core::step::{
    DefaultAmplitude, ExplicitControls, Integration, Procedure as ProcedureKind,
};
use plxs_core::{Error, Model, Progress, ProgressEvent, Result, Step};
use plxs_linalg::{LdltSolver, LinearSolver, Pattern, SparseMatrix};
use plxs_materials::{MaterialInput, STATE_SIZE};
use plxs_output::StaRow;
use rayon::prelude::*;
use std::sync::Arc;

use crate::dynamic::{
    Energies, Motion, energy_requests, initial_values, short, write_energies, write_increment,
};
use crate::fields::{internal_energy_density, nodal_average_scalar};
use crate::profile::Phase;
use crate::{Procedure, ProcedureContext, StepOutcome};

pub struct ExplicitDynamic;

/// Fraction of the stable time step that is taken.
const SAFETY: f64 = 0.9;
/// Linear bulk viscosity: the damping ratio it gives the highest mode of an element.
const BULK_VISCOSITY: f64 = 0.06;
/// Power iterations for the largest eigenvalue of an element.
const POWER_ITERATIONS: usize = 40;
/// Relative energy balance error above which the step warns.
const ENERGY_WARNING: f64 = 0.05;
/// Time steps between two checks of the cancel token within a frame.
const CANCEL_STEPS: u64 = 1000;
/// Seconds between two progress lines within a long step.
const PROGRESS_SECONDS: f64 = 2.0;

/// What the elements give the explicit integration once per step: the lumped mass, the
/// stable time step of every element and its bulk viscosity.
pub(crate) struct ElementData {
    /// Lumped mass of every DOF.
    pub(crate) mass: Vec<f64>,
    /// The highest eigenfrequency (rad per time) of every entry of the element table.
    pub(crate) omega: Vec<f64>,
    /// The bulk viscosity of every entry: `f = c V (g · v) g`.
    pub(crate) viscosity: Vec<Viscosity>,
    /// The highest eigenfrequency of the other elements (membranes, springs, linear
    /// blocks) and what it belongs to.
    pub(crate) other_omega: f64,
}

/// The linear bulk viscosity of an element: coefficient times volume, and the mean
/// gradient row (see `SolidElement::mean_gradients`).
#[derive(Clone, Debug, Default)]
pub(crate) struct Viscosity {
    pub(crate) cv: f64,
    pub(crate) g: Vec<f64>,
}

/// HRZ lumping of a consistent element mass (row-major `ndof × ndof`, three DOFs per
/// node): the diagonal, scaled per direction to the element's mass.
pub(crate) fn hrz(me: &[f64], ndof: usize) -> Vec<f64> {
    let mut out = vec![0.0; ndof];
    for k in 0..3 {
        let mut total = 0.0;
        let mut diagonal = 0.0;
        for a in (k..ndof).step_by(3) {
            diagonal += me[a * ndof + a];
            for b in (k..ndof).step_by(3) {
                total += me[a * ndof + b];
            }
        }
        if diagonal <= 0.0 {
            continue;
        }
        for a in (k..ndof).step_by(3) {
            out[a] = me[a * ndof + a] * total / diagonal;
        }
    }
    out
}

/// The largest eigenfrequency `ω` of `K x = ω² M x` for a symmetric `k` (row-major) and a
/// diagonal `m`, over the DOFs with mass: power iterations on `M^-½ K M^-½`, never above
/// Gershgorin's bound. DOFs without mass are left out (they carry no stiffness in a
/// well-formed element: the out-of-plane DOFs of plane elements).
pub(crate) fn largest_frequency(k: &[f64], m: &[f64]) -> f64 {
    let n = m.len();
    let dofs: Vec<usize> = (0..n).filter(|&i| m[i] > 0.0).collect();
    if dofs.is_empty() {
        return 0.0;
    }
    let s: Vec<f64> = dofs.iter().map(|&i| 1.0 / m[i].sqrt()).collect();
    let a = |i: usize, j: usize| k[dofs[i] * n + dofs[j]] * s[i] * s[j];
    let r = dofs.len();
    let gershgorin = (0..r)
        .map(|i| (0..r).map(|j| a(i, j).abs()).sum::<f64>())
        .fold(0.0_f64, f64::max);
    if gershgorin <= 0.0 {
        return 0.0;
    }
    // A start with every sign pattern in it: alternating plus a deterministic ripple.
    let mut x: Vec<f64> = (0..r)
        .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 } + 0.3 * ((i as f64) * 1.7).sin())
        .collect();
    let mut y = vec![0.0; r];
    let mut rayleigh = 0.0;
    for _ in 0..POWER_ITERATIONS {
        let norm = x.iter().map(|v| v * v).sum::<f64>().sqrt();
        if norm == 0.0 {
            break;
        }
        x.iter_mut().for_each(|v| *v /= norm);
        for (i, yi) in y.iter_mut().enumerate() {
            *yi = (0..r).map(|j| a(i, j) * x[j]).sum();
        }
        rayleigh = x.iter().zip(&y).map(|(a, b)| a * b).sum::<f64>();
        std::mem::swap(&mut x, &mut y);
    }
    // The Rayleigh quotient approaches the largest eigenvalue from below: a tenth more
    // covers a slow convergence, Gershgorin's bound caps it.
    (1.1 * rayleigh).min(gershgorin).max(0.0).sqrt()
}

/// The lumped mass, the element frequencies and the bulk viscosities of a model.
pub(crate) fn element_data(
    table: &ElementTable,
    model: &Model,
    nlgeom: bool,
) -> Result<ElementData> {
    struct PerElement {
        lumped: Vec<f64>,
        omega: f64,
        viscosity: Viscosity,
    }
    let per: Vec<Result<PerElement>> = table
        .entries
        .par_iter()
        .map(|entry| {
            let rho = entry
                .density
                .ok_or_else(|| missing_density(model, entry.element))?;
            let formulation = table.formulation(entry);
            let coords = table.coordinates(model, entry);
            let ndof = formulation.dof_count();
            let mut me = Vec::new();
            formulation.mass_with(&coords, rho, &formulation.dynamic_mass_rule(), &mut me);
            let lumped = hrz(&me, ndof);
            let d = table.law(entry, nlgeom).elastic_matrix();
            let mut ke = Vec::new();
            formulation.stiffness(&coords, &d, &mut ke);
            let omega = largest_frequency(&ke, &lumped);
            // c = b₁ ρ c_d L_e with L_e = c_d Δt_e: b₁ ρ c_d² Δt_e, c_d² = (λ + 2μ)/ρ.
            let modulus = (0..3).map(|i| d[i][i]).fold(0.0_f64, f64::max);
            let (volume, g) = formulation.volume_mean_gradients(&coords);
            let dt_e = if omega > 0.0 { 2.0 / omega } else { 0.0 };
            Ok(PerElement {
                lumped,
                omega,
                viscosity: Viscosity {
                    cv: BULK_VISCOSITY * modulus * dt_e * volume,
                    g,
                },
            })
        })
        .collect();
    let mut mass = vec![0.0; table.dofs.len()];
    let mut omega = Vec::with_capacity(per.len());
    let mut viscosity = Vec::with_capacity(per.len());
    for (entry, element) in table.entries.iter().zip(per) {
        let element = element?;
        for (&dof, m) in entry.dofs.iter().zip(&element.lumped) {
            mass[dof as usize] += m;
        }
        omega.push(element.omega);
        viscosity.push(element.viscosity);
    }
    let mut other_omega = 0.0_f64;
    for membrane in &table.membranes {
        let rho = membrane
            .density
            .ok_or_else(|| missing_density(model, membrane.element))?;
        let lumped = hrz(&membrane.formulation.mass(rho), membrane.dofs.len());
        for (&dof, m) in membrane.dofs.iter().zip(&lumped) {
            mass[dof as usize] += m;
        }
        other_omega = other_omega.max(largest_frequency(
            &membrane.formulation.stiffness(),
            &lumped,
        ));
    }
    // Springs and constant blocks have no mass of their own: their frequency on the masses
    // of the nodes they join (Gershgorin's bound of `M^-½ K M^-½`).
    let bound = |dofs: &[u32], k: &[f64]| -> f64 {
        let n = dofs.len();
        let mut largest = 0.0_f64;
        for i in 0..n {
            let mi = mass[dofs[i] as usize];
            if mi <= 0.0 {
                continue;
            }
            let row: f64 = (0..n)
                .filter(|&j| mass[dofs[j] as usize] > 0.0)
                .map(|j| k[i * n + j].abs() / (mi * mass[dofs[j] as usize]).sqrt())
                .sum();
            largest = largest.max(row);
        }
        largest.sqrt()
    };
    for block in &table.linear_blocks {
        other_omega = other_omega.max(bound(&block.dofs, &block.stiffness));
    }
    for element in &table.discrete {
        let k = element
            .spring_stiffness()
            .unwrap_or_else(|| element.closed_stiffness());
        other_omega = other_omega.max(bound(&element.dofs, &k));
    }
    Ok(ElementData {
        mass,
        omega,
        viscosity,
        other_omega,
    })
}

fn missing_density(model: &Model, element: u32) -> Error {
    Error::input(
        "*DENSITY",
        format!(
            "element {} has no density; a dynamic analysis needs *DENSITY for every material",
            model.elements[element as usize].id
        ),
    )
}

/// How the external loads vary over the step: a sum of reference vectors times their
/// amplitude (one per amplitude name), which costs a vector sum per time step instead of
/// an assembly; or assembled in every step where that does not reproduce them.
struct LoadHistory {
    groups: Vec<(Option<String>, Vec<f64>)>,
    per_step: bool,
}

impl LoadHistory {
    fn new(ctx: &ProcedureContext<'_>, step: &Step, start_total: f64, period: f64) -> Result<Self> {
        let mut names: Vec<Option<String>> = Vec::new();
        let loads = &step.loads;
        for name in loads
            .concentrated
            .iter()
            .map(|l| &l.amplitude)
            .chain(loads.distributed.iter().map(|l| &l.amplitude))
        {
            if !names.contains(name) {
                names.push(name.clone());
            }
        }
        let mut groups = Vec::new();
        for name in names {
            let mut only = step.clone();
            only.default_amplitude = DefaultAmplitude::Step;
            only.loads.concentrated.retain(|l| l.amplitude == name);
            only.loads.distributed.retain(|l| l.amplitude == name);
            for l in &mut only.loads.concentrated {
                l.amplitude = None;
            }
            for l in &mut only.loads.distributed {
                l.amplitude = None;
            }
            let (f, _) = load_at(ctx, &only, 0.0, start_total, period)?;
            groups.push((name, f));
        }
        let mut history = Self {
            groups,
            per_step: false,
        };
        // The decomposition must give the loads the assembly gives.
        for fraction in [0.0, 0.37, 1.0] {
            let time = fraction * period;
            let (direct, _) = load_at(ctx, step, time, start_total, period)?;
            let summed = history.at(ctx, step, time, start_total, period)?;
            let scale = direct.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            let error = direct
                .iter()
                .zip(&summed)
                .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs()));
            if error > 1e-9 * scale.max(f64::MIN_POSITIVE) {
                history.per_step = true;
                break;
            }
        }
        Ok(history)
    }

    fn at(
        &self,
        ctx: &ProcedureContext<'_>,
        step: &Step,
        time: f64,
        start_total: f64,
        period: f64,
    ) -> Result<Vec<f64>> {
        if self.per_step {
            return Ok(load_at(ctx, step, time, start_total, period)?.0);
        }
        let mut f = vec![0.0; ctx.table.dofs.len()];
        for (name, reference) in &self.groups {
            let factor = plxs_assembly::dofs::amplitude_factor(
                ctx.model,
                step,
                name.as_deref(),
                time,
                start_total + time,
                period,
            );
            if factor != 0.0 {
                for (o, r) in f.iter_mut().zip(reference) {
                    *o += factor * r;
                }
            }
        }
        Ok(f)
    }
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

/// The reduced mass `Tᵀ M_L T`: its inverse diagonal, or its factorization where
/// constraints couple DOFs.
enum ReducedMass {
    Diagonal(Vec<f64>),
    Factored(Box<LdltSolver>),
}

impl ReducedMass {
    fn new(ctx: &ProcedureContext<'_>, set: &ConstraintSet, mass: &[f64]) -> Result<Self> {
        let n = mass.len();
        let pattern = Arc::new(Pattern::from_columns(
            n,
            (0..n as u32).map(|j| vec![j]).collect(),
        ));
        let mut m = SparseMatrix::zeros(pattern);
        m.values_mut().copy_from_slice(mass);
        let reduced = set.reduce(&m);
        let r = set.reduced_len();
        // A reduced unknown without mass cannot accelerate under a force.
        let diagonal: Vec<f64> = (0..r as u32)
            .map(|j| {
                reduced
                    .pattern()
                    .position(j, j)
                    .map_or(0.0, |p| reduced.values()[p])
            })
            .collect();
        if let Some(j) = diagonal.iter().position(|d| *d <= 0.0) {
            let dof = set.free[j] as usize;
            let node = ctx.model.nodes.ids[dof / 3];
            return Err(Error::input(
                "*DYNAMIC",
                format!(
                    "explicit integration: node {node} (DOF {}) has no mass; every moving node \
                     needs mass from elements with a density",
                    dof % 3 + 1
                ),
            ));
        }
        if reduced.pattern().nnz() == r {
            return Ok(ReducedMass::Diagonal(
                diagonal.iter().map(|d| 1.0 / d).collect(),
            ));
        }
        let mut solver = LdltSolver::new(ctx.threads);
        solver.analyze(reduced.pattern())?;
        solver.factorize(&reduced)?;
        Ok(ReducedMass::Factored(Box::new(solver)))
    }

    /// `x ← (Tᵀ M_L T)⁻¹ x`.
    fn solve(&self, x: &mut [f64]) -> Result<()> {
        match self {
            ReducedMass::Diagonal(inverse) => {
                x.iter_mut().zip(inverse).for_each(|(v, m)| *v *= m);
                Ok(())
            }
            ReducedMass::Factored(solver) => solver.solve(x),
        }
    }
}

/// What a time step needs from the elements: the internal forces and the bulk viscosity at
/// `u` and `v`, with the material state updated in place.
struct Kernel<'a> {
    table: &'a ElementTable,
    model: &'a Model,
    nlgeom: bool,
    viscosity: &'a [Viscosity],
    /// Offsets of every entry's DOFs in the element force buffers.
    offsets: Vec<usize>,
    fe: Vec<f64>,
    fv: Vec<f64>,
    /// A matrix for the discrete elements to add their (unused) stiffness to.
    scratch: SparseMatrix,
}

/// The forces of a time step.
struct Forces {
    internal: Vec<f64>,
    viscous: Vec<f64>,
}

impl<'a> Kernel<'a> {
    fn new(ctx: &ProcedureContext<'a>, step: &Step, data: &'a ElementData) -> Self {
        let mut offsets = Vec::with_capacity(ctx.table.entries.len() + 1);
        let mut total = 0;
        for entry in &ctx.table.entries {
            offsets.push(total);
            total += entry.dofs.len();
        }
        offsets.push(total);
        Self {
            table: ctx.table,
            model: ctx.model,
            nlgeom: step.nlgeom,
            viscosity: &data.viscosity,
            offsets,
            fe: vec![0.0; total],
            fv: vec![0.0; total],
            scratch: SparseMatrix::zeros(ctx.pattern.clone()),
        }
    }

    /// The internal and viscous forces at `u`, `v` (time step `dt` ending at total time
    /// `time`); the material state advances in place. With `fields` the stresses and
    /// strains of the integration points are written there.
    fn forces(
        &mut self,
        u: &[f64],
        v: &[f64],
        dt: f64,
        time: f64,
        material: &mut [f64],
        fields: Option<(&mut IntegrationPointField, &mut IntegrationPointField)>,
    ) -> Forces {
        let table = self.table;
        let model = self.model;
        let nlgeom = self.nlgeom;
        let layout = &table.layout;
        let n = table.dofs.len();
        // Per entry: its force slices and its slice of the material state (contiguous in
        // the order of the entries).
        let mut fe_slices: Vec<&mut [f64]> = Vec::with_capacity(table.entries.len());
        let mut fv_slices: Vec<&mut [f64]> = Vec::with_capacity(table.entries.len());
        {
            let mut rest_e: &mut [f64] = &mut self.fe;
            let mut rest_v: &mut [f64] = &mut self.fv;
            for w in self.offsets.windows(2) {
                let len = w[1] - w[0];
                let (a, b) = std::mem::take(&mut rest_e).split_at_mut(len);
                fe_slices.push(a);
                rest_e = b;
                let (a, b) = std::mem::take(&mut rest_v).split_at_mut(len);
                fv_slices.push(a);
                rest_v = b;
            }
        }
        let mut material_slices: Vec<&mut [f64]> = Vec::with_capacity(table.entries.len());
        {
            let mut rest: &mut [f64] = material;
            for (index, entry) in table.entries.iter().enumerate() {
                let start = layout.offsets[index] as usize;
                let points = table.formulation(entry).integration_points();
                let end = layout
                    .offsets
                    .get(index + 1)
                    .map_or(start + points, |e| *e as usize);
                let (a, b) = std::mem::take(&mut rest).split_at_mut((end - start) * STATE_SIZE);
                material_slices.push(a);
                rest = b;
            }
        }
        let want_fields = fields.is_some();
        let mut point_fields: Vec<(Vec<f64>, Vec<f64>)> = Vec::new();
        let outputs: Vec<Option<(Vec<f64>, Vec<f64>)>> = table
            .entries
            .par_iter()
            .zip(fe_slices.par_iter_mut())
            .zip(fv_slices.par_iter_mut())
            .zip(material_slices.par_iter_mut())
            .zip(self.viscosity.par_iter())
            .map(|((((entry, fe), fv), state), viscosity)| {
                let formulation = table.formulation(entry);
                let law = table.law(entry, nlgeom);
                let coords = table.coordinates(model, entry);
                let ue = table.gather(entry, u);
                let points = formulation.integration_points();
                let mut stresses = if want_fields {
                    vec![0.0; 6 * points]
                } else {
                    Vec::new()
                };
                let mut strains = stresses.clone();
                fe.iter_mut().for_each(|f| *f = 0.0);
                let d_elastic = law.elastic_matrix();
                formulation.nonlinear(
                    &coords,
                    &ue,
                    nlgeom,
                    &d_elastic,
                    |ip, kinematics: &plxs_elements::PointKinematics| {
                        let slice = &mut state[ip * STATE_SIZE..(ip + 1) * STATE_SIZE];
                        let (output, e) = law.stress_and_strain(
                            &MaterialInput {
                                strain: kinematics.strain,
                                temperature: 0.0,
                                dt,
                                time,
                                creep_active: false,
                            },
                            slice,
                        );
                        if want_fields {
                            let sigma = if nlgeom {
                                let mut kinematics = *kinematics;
                                if e[2] != kinematics.strain[2] {
                                    kinematics.deformation_gradient[2][2] =
                                        (1.0 + 2.0 * e[2]).max(0.0).sqrt();
                                }
                                kinematics.cauchy(&output.stress)
                            } else {
                                output.stress
                            };
                            stresses[ip * 6..(ip + 1) * 6].copy_from_slice(&sigma);
                            strains[ip * 6..(ip + 1) * 6].copy_from_slice(&[
                                e[0],
                                e[1],
                                e[2],
                                0.5 * e[3],
                                0.5 * e[4],
                                0.5 * e[5],
                            ]);
                        }
                        (output.stress, output.tangent)
                    },
                    fe,
                    None,
                );
                // Bulk viscosity on the mean volumetric strain rate.
                let rate: f64 = entry
                    .dofs
                    .iter()
                    .zip(&viscosity.g)
                    .map(|(&dof, g)| g * v[dof as usize])
                    .sum();
                let q = viscosity.cv * rate;
                for (f, g) in fv.iter_mut().zip(&viscosity.g) {
                    *f = q * g;
                }
                want_fields.then_some((stresses, strains))
            })
            .collect();
        if want_fields {
            point_fields = outputs.into_iter().map(|o| o.unwrap_or_default()).collect();
        }
        let mut internal = vec![0.0; n];
        let mut viscous = vec![0.0; n];
        for (index, entry) in table.entries.iter().enumerate() {
            let range = self.offsets[index]..self.offsets[index + 1];
            for ((&dof, f), q) in entry
                .dofs
                .iter()
                .zip(&self.fe[range.clone()])
                .zip(&self.fv[range])
            {
                internal[dof as usize] += f;
                viscous[dof as usize] += q;
            }
        }
        if let Some((stresses, strains)) = fields {
            for (index, (s, e)) in point_fields.iter().enumerate() {
                let start = layout.offsets[index] as usize;
                let points = s.len() / 6;
                stresses.values[start * 6..(start + points) * 6].copy_from_slice(s);
                strains.values[start * 6..(start + points) * 6].copy_from_slice(e);
            }
        }
        // Constant blocks (U1), native membranes and the discrete elements.
        for block in table.linear_blocks.iter().filter(|b| !b.membrane) {
            let m = block.dofs.len();
            for (i, &dof) in block.dofs.iter().enumerate() {
                internal[dof as usize] += (0..m)
                    .map(|j| block.stiffness[i * m + j] * u[block.dofs[j] as usize])
                    .sum::<f64>();
            }
        }
        for membrane in &table.membranes {
            let mut fe = vec![0.0; membrane.dofs.len()];
            membrane
                .formulation
                .internal(&membrane.gather(u), nlgeom, &mut fe, None);
            for (&dof, f) in membrane.dofs.iter().zip(&fe) {
                internal[dof as usize] += f;
            }
        }
        if !table.discrete.is_empty() {
            let discrete: Vec<&plxs_assembly::discrete::DiscreteElement> =
                table.discrete.iter().collect();
            crate::nonlinear::NonlinearStatic::add_discrete(
                &discrete,
                u,
                &mut self.scratch,
                &mut internal,
            );
        }
        Forces { internal, viscous }
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// The prescribed part `g` of the displacements at step time `time`.
#[allow(clippy::too_many_arguments)]
fn prescribed(
    ctx: &ProcedureContext<'_>,
    step: &Step,
    set: &ConstraintSet,
    previous: &[f64],
    time: f64,
    start_total: f64,
    period: f64,
) -> Result<Vec<f64>> {
    let mut spc = Spc::of_step(
        ctx.model,
        step,
        &ctx.table.dofs,
        time,
        start_total + time,
        period,
    )?;
    spc.apply_fixed(ctx.model, step, &ctx.table.dofs, previous, &|node| {
        ctx.constraints.rotation(node).copied()
    });
    Ok(set.particular(&spc.values))
}

/// The settings of an explicit step and what follows from them.
pub(crate) struct Timing {
    /// The stable time step (with the safety factor and the damping).
    pub(crate) stable: f64,
    /// The element that limits it (dense index), `None` for the other elements.
    pub(crate) limiting: Option<u32>,
}

/// The stable time step from the element frequencies, reduced for the bulk viscosity and
/// the stiffness-proportional damping `beta`.
pub(crate) fn stable_time_step(data: &ElementData, beta: f64) -> Timing {
    let mut largest = data.other_omega;
    let mut limiting = None;
    for (index, &omega) in data.omega.iter().enumerate() {
        if omega > largest {
            largest = omega;
            limiting = Some(index as u32);
        }
    }
    if largest <= 0.0 {
        return Timing {
            stable: f64::INFINITY,
            limiting,
        };
    }
    let xi = BULK_VISCOSITY + 0.5 * beta * largest;
    Timing {
        stable: SAFETY * 2.0 / largest * ((1.0 + xi * xi).sqrt() - xi),
        limiting,
    }
}

impl Procedure for ExplicitDynamic {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        let ProcedureKind::Dynamic(parameters) = &step.procedure else {
            return Err(Error::Unsupported(
                "ExplicitDynamic needs a *DYNAMIC step".into(),
            ));
        };
        let controls: ExplicitControls = match parameters.integration {
            Integration::Explicit(c) => c,
            _ => {
                return Err(Error::Unsupported(
                    "ExplicitDynamic needs INTEGRATION=EXPLICIT".into(),
                ));
            }
        };
        let increments = parameters.increments;
        let period = increments.period();
        if period <= 0.0 {
            return Err(Error::input("*DYNAMIC", "the step period must be positive"));
        }
        // The frame interval: the increments the step asks for.
        let interval = increments.initial().min(period);
        if interval <= 0.0 {
            return Err(Error::input(
                "*DYNAMIC",
                "the initial increment must be positive",
            ));
        }
        if let Some(target) = controls.mass_scaling
            && target > 0.0
        {
            progress.event(ProgressEvent::Warning(
                "explicit: mass scaling is not implemented yet; the step runs without it".into(),
            ));
        }
        ctx.table.require_elastic()?;
        if !ctx.contact.is_empty() {
            return Err(Error::Unsupported(format!(
                "contact in explicit *DYNAMIC steps is not implemented yet (step {})",
                step.name
            )));
        }
        let n = ctx.table.dofs.len();
        let start_total = ctx.state.total_time;

        // Lumped mass, element frequencies, bulk viscosity.
        let started = std::time::Instant::now();
        let data = element_data(ctx.table, ctx.model, step.nlgeom)?;
        ctx.profile.add(Phase::Assembly, started);

        // Constraints: `u = T q + g`.
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
            "number of unknowns: {} (explicit, no equation system)",
            set.reduced_len()
        )));
        let started = std::time::Instant::now();
        let reduced_mass = ReducedMass::new(ctx, &set, &data.mass)?;
        ctx.profile.add(Phase::Factorization, started);
        if let ReducedMass::Factored(_) = reduced_mass {
            progress.event(ProgressEvent::Info(
                "explicit: constraints couple the mass; the reduced mass is factorized once".into(),
            ));
        }

        // Damping.
        let (alpha_r, beta_r) = explicit_damping(ctx, step, &set, progress)?;

        let timing = stable_time_step(&data, beta_r);
        let mut dt_max = timing.stable;
        if let Some(fixed) = controls.time_step.filter(|t| *t > 0.0) {
            if fixed > timing.stable {
                progress.event(ProgressEvent::Warning(format!(
                    "explicit: the time step {} s is above the stable one {} s; the stable one is taken",
                    short(fixed),
                    short(timing.stable)
                )));
            } else {
                dt_max = fixed;
            }
        }
        if !dt_max.is_finite() {
            dt_max = interval;
        }
        let total_steps = (period / dt_max).ceil();
        progress.event(ProgressEvent::Info(format!(
            "explicit: stable time step {} s{}, about {} time steps for {} s",
            short(timing.stable),
            match timing.limiting {
                Some(index) => format!(
                    " (element {})",
                    ctx.model.elements[ctx.table.entries[index as usize].element as usize].id
                ),
                None if data.other_omega > 0.0 => " (springs or membranes)".into(),
                None => String::new(),
            },
            short(total_steps),
            short(period)
        )));
        if !ctx
            .table
            .formulations
            .iter()
            .any(|f| f.integration_points() == 1 || format!("{:?}", f.kind).ends_with('R'))
        {
            progress.event(ProgressEvent::Info(
                "explicit: reduced integration (S4R, S8R, C3D8R) makes explicit steps several times faster"
                    .into(),
            ));
        }

        // Material state with the layout of this model.
        let layout = material_state(ctx.table);
        if ctx.state.material.values.len() != layout.values.len() {
            ctx.state.material = layout;
        }
        let mut material = ctx.state.material.values.clone();
        let material_template = IntegrationPointField {
            values: Vec::new(),
            ..ctx.state.material.clone()
        };

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
        let previous = ctx.state.displacements.values.clone();
        let varying = step.boundaries.iter().any(|b| {
            b.amplitude.is_some()
                || (step.default_amplitude == DefaultAmplitude::Ramp
                    && b.value.is_some_and(|v| v != 0.0))
        });
        let g_at = |ctx: &ProcedureContext<'_>, time: f64| {
            prescribed(ctx, step, &set, &previous, time, start_total, period)
        };
        let mut g_cur = g_at(ctx, 0.0)?;
        let varying = varying || g_cur != g_at(ctx, period)?;
        let mut g_prev = g_cur.clone();

        let (_, warnings) = load_at(ctx, step, 0.0, start_total, period)?;
        for warning in warnings {
            progress.event(ProgressEvent::Warning(warning));
        }
        let loads = LoadHistory::new(ctx, step, start_total, period)?;

        // The reduced unknowns and their velocities.
        let free: Vec<usize> = set.free.iter().map(|&d| d as usize).collect();
        let mut q: Vec<f64> = free.iter().map(|&d| motion.u[d]).collect();
        let mut qdot: Vec<f64> = free.iter().map(|&d| motion.v[d]).collect();
        let mut u = set.expand(&q, &g_cur);
        let mut v_half = set.expand(&qdot, &vec![0.0; n]);
        let mut kernel = Kernel::new(ctx, step, &data);
        let mut stresses = ctx.table.layout.field(6);
        let mut strains = ctx.table.layout.field(6);
        let started = std::time::Instant::now();
        let mut forces = kernel.forces(
            &u,
            &v_half,
            dt_max,
            start_total,
            &mut material,
            Some((&mut stresses, &mut strains)),
        );
        ctx.profile.add(Phase::Assembly, started);
        let mut p = loads.at(ctx, step, 0.0, start_total, period)?;
        let mass = &data.mass;
        let damping_force = |v: &[f64], f_int: &[f64], f_int_prev: &[f64], dt: f64| -> Vec<f64> {
            (0..n)
                .map(|i| {
                    let mut f = alpha_r * mass[i] * v[i];
                    if beta_r != 0.0 && dt > 0.0 {
                        f += beta_r * (f_int[i] - f_int_prev[i]) / dt;
                    }
                    f
                })
                .collect()
        };
        let mut f_damp = damping_force(&v_half, &forces.internal, &forces.internal, 0.0);
        // The acceleration at the start.
        let accelerate = |r: &[f64]| -> Result<Vec<f64>> {
            let mut a = set.reduce_rhs(r);
            reduced_mass.solve(&mut a)?;
            Ok(a)
        };
        let residual = |p: &[f64], forces: &Forces, f_damp: &[f64], g_acc: Option<&[f64]>| {
            (0..n)
                .map(|i| {
                    p[i] - forces.internal[i]
                        - forces.viscous[i]
                        - f_damp[i]
                        - g_acc.map_or(0.0, |g| mass[i] * g[i])
                })
                .collect::<Vec<f64>>()
        };
        let mut a_red = accelerate(&residual(&p, &forces, &f_damp, None))?;
        let zero = vec![0.0; n];
        let mut a_full = set.expand(&a_red, &zero);

        let kinetic = |v: &[f64]| 0.5 * v.iter().zip(mass).map(|(v, m)| m * v * v).sum::<f64>();
        let mut energies = Energies {
            internal: 0.5 * dot(&u, &forces.internal),
            kinetic: kinetic(&v_half),
            external_work: 0.0,
            dissipation: 0.0,
        };
        let initial_energy = energies.internal + energies.kinetic;
        let mut largest_energy = initial_energy.abs();
        // The reactions (`M a + f − p` on the prescribed DOFs) for the work of prescribed
        // displacements.
        let reaction =
            |a_full: &[f64], g_acc: &[f64], p: &[f64], forces: &Forces, f_damp: &[f64]| {
                (0..n)
                    .map(|i| {
                        mass[i] * (a_full[i] + g_acc[i])
                            + forces.internal[i]
                            + forces.viscous[i]
                            + f_damp[i]
                            - p[i]
                    })
                    .collect::<Vec<f64>>()
            };
        let mut g_acc = vec![0.0; n];
        let mut reactions_prev = if varying {
            reaction(&a_full, &g_acc, &p, &forces, &f_damp)
        } else {
            Vec::new()
        };

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

        let frames = ((period / interval) - 1e-9).ceil().max(1.0) as u32;
        let mut time = 0.0;
        let mut h_prev = 0.0;
        let mut steps_done: u64 = 0;
        let mut last_info = std::time::Instant::now();
        let run_started = std::time::Instant::now();
        let mut balance_warned = false;
        for frame in 1..=frames {
            if ctx.cancel.is_cancelled() {
                return Ok(StepOutcome::Cancelled);
            }
            let t_frame = if frame == frames {
                period
            } else {
                f64::from(frame) * interval
            };
            let steps = ((t_frame - time) / dt_max * (1.0 - 1e-12)).ceil().max(1.0) as u64;
            let h = (t_frame - time) / steps as f64;
            progress.event(ProgressEvent::Increment {
                step: ctx.step_number,
                increment: frame,
                attempt: 1,
                size: t_frame - time,
                step_time: t_frame,
                total_time: start_total + t_frame,
            });
            for s in 0..steps {
                if s > 0 && s % CANCEL_STEPS == 0 && ctx.cancel.is_cancelled() {
                    return Ok(StepOutcome::Cancelled);
                }
                let last_of_frame = s + 1 == steps;
                let t_next = if last_of_frame { t_frame } else { time + h };
                let started = std::time::Instant::now();
                let g_next = if varying {
                    g_at(ctx, t_next)?
                } else {
                    g_cur.clone()
                };
                // Velocities and displacements.
                let c = 0.5 * (h_prev + h);
                for (qd, a) in qdot.iter_mut().zip(&a_red) {
                    *qd += c * a;
                }
                for (qi, qd) in q.iter_mut().zip(&qdot) {
                    *qi += h * qd;
                }
                let u_next = set.expand(&q, &g_next);
                let mut v_next = set.expand(&qdot, &zero);
                if varying {
                    for ((v, gn), gc) in v_next.iter_mut().zip(&g_next).zip(&g_cur) {
                        *v += (gn - gc) / h;
                    }
                }
                ctx.profile.add(Phase::Vectors, started);
                // Forces at the new time.
                let started = std::time::Instant::now();
                let fields = last_of_frame.then_some((&mut stresses, &mut strains));
                let new_forces = kernel.forces(
                    &u_next,
                    &v_next,
                    h,
                    start_total + t_next,
                    &mut material,
                    fields,
                );
                ctx.profile.add(Phase::Assembly, started);
                let started = std::time::Instant::now();
                let p_next = loads.at(ctx, step, t_next, start_total, period)?;
                ctx.profile.add(Phase::Loads, started);
                let started = std::time::Instant::now();
                let f_damp_next = if alpha_r != 0.0 || beta_r != 0.0 {
                    damping_force(&v_next, &new_forces.internal, &forces.internal, h)
                } else {
                    vec![0.0; n]
                };
                if varying {
                    let h_back = if h_prev > 0.0 { h_prev } else { h };
                    for i in 0..n {
                        g_acc[i] = ((g_next[i] - g_cur[i]) / h - (g_cur[i] - g_prev[i]) / h_back)
                            / (0.5 * (h + h_back));
                    }
                    if h_prev == 0.0 {
                        g_acc.iter_mut().for_each(|g| *g = 0.0);
                    }
                }
                let r = residual(
                    &p_next,
                    &new_forces,
                    &f_damp_next,
                    varying.then_some(&g_acc[..]),
                );
                a_red = accelerate(&r)?;
                // Energies over the step (trapezoidal rule of the forces).
                let mut w_int = 0.0;
                let mut w_ext = 0.0;
                let mut w_diss = 0.0;
                for i in 0..n {
                    let du = u_next[i] - u[i];
                    if du == 0.0 {
                        continue;
                    }
                    w_int += du * 0.5 * (forces.internal[i] + new_forces.internal[i]);
                    w_ext += du * 0.5 * (p[i] + p_next[i]);
                    w_diss += du
                        * 0.5
                        * (forces.viscous[i] + new_forces.viscous[i] + f_damp[i] + f_damp_next[i]);
                }
                energies.internal += w_int;
                energies.external_work += w_ext;
                energies.dissipation += w_diss;
                if varying {
                    a_full = set.expand(&a_red, &zero);
                    let reactions = reaction(&a_full, &g_acc, &p_next, &new_forces, &f_damp_next);
                    for i in 0..n {
                        let dg = g_next[i] - g_cur[i];
                        if dg != 0.0 {
                            // The prescribed motion's work counts as external work: the
                            // reaction forces of the reduction, minus the loads there.
                            energies.external_work += dg * 0.5 * (reactions_prev[i] + reactions[i]);
                        }
                    }
                    reactions_prev = reactions;
                }
                g_prev = std::mem::replace(&mut g_cur, g_next);
                u = u_next;
                v_half = v_next;
                forces = new_forces;
                p = p_next;
                f_damp = f_damp_next;
                h_prev = h;
                time = t_next;
                steps_done += 1;
                ctx.profile.add(Phase::Vectors, started);
            }

            // The frame: full-step velocities and the accelerations.
            let started = std::time::Instant::now();
            a_full = set.expand(&a_red, &g_acc);
            let v_full: Vec<f64> = v_half
                .iter()
                .zip(&a_full)
                .map(|(v, a)| v + 0.5 * h * a)
                .collect();
            energies.kinetic = kinetic(&v_full);
            let total_energy = energies.kinetic + energies.internal + energies.dissipation;
            largest_energy = largest_energy
                .max(energies.kinetic)
                .max(energies.internal.abs())
                .max(energies.external_work.abs());
            let balance = (total_energy - initial_energy - energies.external_work).abs()
                / largest_energy.max(f64::MIN_POSITIVE);
            if balance > ENERGY_WARNING && !balance_warned && largest_energy > 0.0 {
                balance_warned = true;
                progress.event(ProgressEvent::Warning(format!(
                    "explicit: the energy balance is off by {} % at {} s; the time step may be \
                     too large for the model",
                    short(100.0 * balance),
                    short(time)
                )));
            }
            let state_motion = Motion {
                u: u.clone(),
                v: v_full,
                a: a_full.clone(),
            };
            let mut reactions: Vec<f64> = (0..n)
                .map(|i| forces.internal[i] + forces.viscous[i] + f_damp[i])
                .collect();
            ctx.constraints
                .transfer_forces(&mut reactions, &spc_end.dofs, &active);
            let total = start_total + time;
            if !requests.is_empty() {
                write_energies(
                    ctx.results,
                    &requests,
                    &energies,
                    (ctx.step_number, frame, total),
                );
            }
            let mut current_material = material_template.clone();
            current_material.values.clone_from(&material);
            if crate::static_linear::writes_increment(step, frame, frames) {
                write_increment(
                    ctx,
                    step,
                    frame,
                    total,
                    &state_motion,
                    &reactions,
                    (&stresses, &strains),
                    carried_nodal.as_deref(),
                    Some(&current_material),
                );
            }
            crate::history::write_history_with_velocities(
                ctx.results,
                ctx.model,
                ctx.table,
                step,
                (ctx.step_number, frame, total),
                (&state_motion.u, Some(&state_motion.v)),
                &reactions,
                &stresses,
                &strains,
                Some(&current_material),
            );
            ctx.profile.add(Phase::Results, started);
            ctx.sta.push(StaRow {
                step: ctx.step_number,
                increment: frame,
                attempt: 1,
                iterations: steps.min(u64::from(u32::MAX)) as u32,
                total_time: total,
                step_time: time,
                increment_size: h,
            });
            progress.event(ProgressEvent::IncrementDone {
                step: ctx.step_number,
                increment: frame,
                attempt: 1,
                iterations: steps.min(u64::from(u32::MAX)) as u32,
                total_time: total,
                step_time: time,
                size: h,
            });
            if last_info.elapsed().as_secs_f64() > PROGRESS_SECONDS || frame == frames {
                last_info = std::time::Instant::now();
                let elapsed = run_started.elapsed().as_secs_f64();
                let left = elapsed * (period - time) / time.max(f64::MIN_POSITIVE);
                progress.event(ProgressEvent::Info(format!(
                    "explicit: {} s of {} s, {steps_done} time steps of {} s, energy balance {} %{}",
                    short(time),
                    short(period),
                    short(h),
                    short(100.0 * balance),
                    if frame < frames {
                        format!(", about {:.0} s left", left)
                    } else {
                        String::new()
                    }
                )));
            }
            motion = state_motion;
        }

        let mut energy = internal_energy_density(&stresses, &strains);
        if let Some(c) = &carried {
            for (e, c) in energy.values.iter_mut().zip(&c.values) {
                *e += c;
            }
        }
        ctx.state.energy = Some(energy);
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
        ctx.state.loads = None;
        ctx.state.stresses = stresses;
        ctx.state.strains = strains;
        ctx.state.material.values = material;
        ctx.state.total_time = start_total + period;
        ctx.state.steps_done = ctx.step_number;
        Ok(StepOutcome::Converged)
    }
}

/// The damping of an explicit step: `(α_R, β_R)`. A damping ratio damps the mass-
/// proportional part only, at the lowest elastic eigenfrequency.
fn explicit_damping(
    ctx: &mut ProcedureContext<'_>,
    step: &Step,
    set: &ConstraintSet,
    progress: &mut dyn Progress,
) -> Result<(f64, f64)> {
    let ProcedureKind::Dynamic(parameters) = &step.procedure else {
        return Ok((0.0, 0.0));
    };
    if let Some(ratio) = parameters.damping_ratio {
        let lower = match ratio.lower {
            Some(lower) => Some(lower),
            None => {
                let started = std::time::Instant::now();
                if ctx.stiffness.is_none() {
                    let mut k = SparseMatrix::zeros(ctx.pattern.clone());
                    plxs_assembly::assemble_stiffness(ctx.table, ctx.model, &mut k);
                    ctx.stiffness = Some(k);
                }
                let k = ctx.stiffness.clone().expect("assembled");
                let mut m = SparseMatrix::zeros(ctx.pattern.clone());
                plxs_assembly::assemble_mass_with(
                    ctx.table,
                    ctx.model,
                    plxs_assembly::MassRule::Full,
                    &mut m,
                )?;
                ctx.profile.add(Phase::Assembly, started);
                let fitted = crate::dynamic::fit_damping(
                    ctx,
                    set,
                    &k,
                    &m,
                    ratio,
                    parameters.solver,
                    progress,
                )?;
                // α = 2ζ ω₁ω₂/(ω₁+ω₂), β = 2ζ/(ω₁+ω₂): ω₁ = α/(β ω₂); α = 0 marks rigid
                // body modes, which mass-proportional damping would brake.
                let w2 = std::f64::consts::TAU * ratio.upper;
                (fitted.alpha > 0.0 && fitted.beta > 0.0)
                    .then(|| fitted.alpha / (fitted.beta * w2) / std::f64::consts::TAU)
            }
        };
        return Ok(match lower {
            Some(f1) if f1 > 0.0 => {
                let alpha = 2.0 * ratio.ratio * std::f64::consts::TAU * f1;
                progress.event(ProgressEvent::Info(format!(
                    "explicit: damping {} % at {} Hz as mass-proportional damping (alpha = {}); \
                     stiffness-proportional damping would shrink the time step",
                    short(100.0 * ratio.ratio),
                    short(f1),
                    short(alpha)
                )));
                (alpha, 0.0)
            }
            _ => {
                progress.event(ProgressEvent::Info(
                    "explicit: rigid body or mechanism modes: the damping ratio is not applied \
                     (mass-proportional damping would brake the free motion)"
                        .into(),
                ));
                (0.0, 0.0)
            }
        });
    }
    let r = parameters.rayleigh.unwrap_or_default();
    if r.alpha != 0.0 || r.beta != 0.0 {
        progress.event(ProgressEvent::Info(format!(
            "Rayleigh damping alpha = {}, beta = {}",
            short(r.alpha),
            short(r.beta)
        )));
    }
    if r.beta > 0.0 {
        progress.event(ProgressEvent::Info(
            "explicit: stiffness-proportional damping (beta) reduces the stable time step".into(),
        ));
    }
    Ok((r.alpha, r.beta))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hrz_keeps_the_mass_and_stays_positive() {
        // Consistent mass of a 3-node bar (quadratic) along x, per direction.
        let base = [[4.0, -1.0, 2.0], [-1.0, 4.0, 2.0], [2.0, 2.0, 16.0]];
        let ndof = 9;
        let mut me = vec![0.0; ndof * ndof];
        for a in 0..3 {
            for b in 0..3 {
                for k in 0..3 {
                    me[(3 * a + k) * ndof + 3 * b + k] = base[a][b] / 30.0;
                }
            }
        }
        let lumped = hrz(&me, ndof);
        let total: f64 = (0..ndof).step_by(3).map(|a| lumped[a]).sum();
        assert!((total - 1.0).abs() < 1e-12, "{total}");
        assert!(lumped.iter().all(|m| *m > 0.0));
        assert!((lumped[0] - 1.0 / 6.0).abs() < 1e-12);
    }

    #[test]
    fn largest_frequency_of_two_masses_on_a_spring() {
        let k = [2.0, -2.0, -2.0, 2.0];
        let m = [1.0, 3.0];
        // ω² = k (1/m1 + 1/m2) = 2 · 4/3.
        let omega = largest_frequency(&k, &m);
        let exact = (8.0_f64 / 3.0).sqrt();
        assert!(
            omega >= exact * 0.999 && omega <= exact * 1.05,
            "{omega} vs {exact}"
        );
    }
}
