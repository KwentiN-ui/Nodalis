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
use crate::shell_mass::{self, FibreMass, Fibres};
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
    /// The lumped mass of every entry, one after the other in the order of the entries.
    pub(crate) lumped: Vec<f64>,
    /// Selective mass scaling of expanded shells: per entry its factor and fibres (see
    /// [`crate::shell_mass`]), and the nodes of every fibre.
    pub(crate) shells: Vec<Option<ShellEntry>>,
    pub(crate) fibre_nodes: Vec<Vec<u32>>,
}

/// An expanded shell element under selective mass scaling.
#[derive(Clone, Debug)]
pub(crate) struct ShellEntry {
    pub(crate) alpha: f64,
    /// (fibre, the element's local nodes in fibre order, their positions in the fibre).
    pub(crate) fibres: Vec<shell_mass::ElementFibre>,
}

impl ElementData {
    /// The mass the fibres of the scaled shells add, `None` without them; from the lumped
    /// masses as they are (after a mass scaling).
    pub(crate) fn fibre_mass(&self, table: &ElementTable) -> Option<FibreMass> {
        if self.shells.iter().all(Option::is_none) {
            return None;
        }
        let mut out = FibreMass::new(self.fibre_nodes.clone());
        let mut offset = 0;
        for (entry, shell) in table.entries.iter().zip(&self.shells) {
            let lumped = &self.lumped[offset..offset + entry.dofs.len()];
            offset += entry.dofs.len();
            let Some(shell) = shell else { continue };
            for (fibre, local, positions) in &shell.fibres {
                let masses: Vec<f64> = local.iter().map(|&a| lumped[a * 3]).collect();
                out.add(
                    *fibre,
                    positions,
                    &shell_mass::fibre_block(shell.alpha, &masses),
                );
            }
        }
        Some(out)
    }
}

/// The highest frequency of an element on its lumped mass, plus the fibre blocks of a
/// scaled shell.
fn element_frequency(ke: &[f64], lumped: &[f64], shell: Option<&ShellEntry>) -> f64 {
    match shell {
        Some(shell) => {
            let (m, _) = shell_mass::element_mass(lumped, &shell.fibres, shell.alpha);
            shell_mass::largest_frequency_dense(ke, &m, lumped.len())
        }
        None => largest_frequency(ke, lumped),
    }
}

/// The linear bulk viscosity of an element, `f = c V (g · v) g` with the mean gradient row
/// `g` (see `SolidElement::volume_mean_gradients`), and its hourglass viscosity.
#[derive(Clone, Debug, Default)]
pub(crate) struct Viscosity {
    /// `c`: `b₁ ρ c_d² Δt_e`, per volume.
    pub(crate) rate: f64,
    /// Volume and mean gradients in the reference configuration.
    pub(crate) volume: f64,
    pub(crate) g: Vec<f64>,
    /// The coefficient of the viscous hourglass control (C3D8R), zero for other elements.
    pub(crate) hourglass: f64,
}

/// The hourglass base vectors of an eight-node brick (Flanagan and Belytschko 1981): the
/// nodal patterns `ξη`, `ηζ`, `ζξ` and `ξηζ`, which the single integration point does not
/// see.
const HOURGLASS_BASE: [[f64; 8]; 4] = [
    [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0],
    [1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0],
    [1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0],
    [-1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0],
];

/// Coefficient of the viscous hourglass control: `f = κ ρ c_d V^⅔ / 4 · Σ γ (γ · v)`
/// (the LS-DYNA standard, κ = 0.1).
const HOURGLASS_VISCOSITY: f64 = 0.1;

/// The viscous hourglass forces of a C3D8R at nodal positions `x` (eight nodes) with mean
/// gradients `g` (24, node-major), velocities `v` (24), added to `f`: the hourglass
/// patterns made orthogonal to the linear fields, `γ_i = h_i − (h_i · x) b`.
fn hourglass_forces(x: &[[f64; 3]], g: &[f64], v: &[f64], coefficient: f64, f: &mut [f64]) {
    for h in &HOURGLASS_BASE {
        let mut hx = [0.0; 3];
        for (a, xa) in x.iter().enumerate() {
            for j in 0..3 {
                hx[j] += h[a] * xa[j];
            }
        }
        let mut gamma = [0.0; 8];
        for a in 0..8 {
            gamma[a] = h[a] - (0..3).map(|j| hx[j] * g[3 * a + j]).sum::<f64>();
        }
        for j in 0..3 {
            let q: f64 = (0..8).map(|a| gamma[a] * v[3 * a + j]).sum();
            for a in 0..8 {
                f[3 * a + j] += coefficient * gamma[a] * q;
            }
        }
    }
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
    selective: bool,
) -> Result<ElementData> {
    struct PerElement {
        lumped: Vec<f64>,
        omega: f64,
        viscosity: Viscosity,
        shell: Option<ShellEntry>,
    }
    let fibres = if selective {
        Fibres::of(model)
    } else {
        Fibres::of(&Model::default())
    };
    let entry_fibres = shell_mass::entry_fibres(table, &fibres);
    let per: Vec<Result<PerElement>> = table
        .entries
        .par_iter()
        .zip(entry_fibres.into_par_iter())
        .map(|(entry, element_fibres)| {
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
            // The thickness modes of a shell: their own frequency for the bulk viscosity,
            // the scaled one for the time step.
            let omega_own = largest_frequency(&ke, &lumped);
            let shell = (!element_fibres.is_empty())
                .then(|| ShellEntry {
                    alpha: shell_mass::alpha(&coords, &element_fibres),
                    fibres: element_fibres,
                })
                .filter(|s| s.alpha > 0.0);
            let omega = match &shell {
                Some(shell) => element_frequency(&ke, &lumped, Some(shell)).min(omega_own),
                None => omega_own,
            };
            // c = b₁ ρ c_d L_e with L_e = c_d Δt_e: b₁ ρ c_d² Δt_e, c_d² = (λ + 2μ)/ρ.
            let modulus = (0..3).map(|i| d[i][i]).fold(0.0_f64, f64::max);
            let (volume, g) = formulation.volume_mean_gradients(&coords);
            let dt_e = if omega_own > 0.0 {
                2.0 / omega_own
            } else {
                0.0
            };
            let hourglass = if formulation.kind == plxs_core::element::ElementType::C3D8R
                && coords.len() == 8
                && ndof == 24
            {
                HOURGLASS_VISCOSITY * rho * (modulus / rho).sqrt() * volume.powf(2.0 / 3.0) / 4.0
            } else {
                0.0
            };
            Ok(PerElement {
                lumped,
                omega,
                viscosity: Viscosity {
                    rate: BULK_VISCOSITY * modulus * dt_e,
                    volume,
                    g,
                    hourglass,
                },
                shell,
            })
        })
        .collect();
    let mut mass = vec![0.0; table.dofs.len()];
    let mut omega = Vec::with_capacity(per.len());
    let mut viscosity = Vec::with_capacity(per.len());
    let mut lumped = Vec::new();
    let mut shells = Vec::with_capacity(per.len());
    for (entry, element) in table.entries.iter().zip(per) {
        let element = element?;
        shells.push(element.shell);
        for (&dof, m) in entry.dofs.iter().zip(&element.lumped) {
            mass[dof as usize] += m;
        }
        lumped.extend_from_slice(&element.lumped);
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
        lumped,
        shells,
        fibre_nodes: fibres.nodes,
    })
}

/// The highest eigenfrequency of every entry in the configuration `u` (NLGEOM: elements
/// that are squeezed get stiffer and their stable time step shorter), with the elastic
/// stiffness and the lumped masses of `data` (which do not change).
fn current_frequencies(
    table: &ElementTable,
    model: &Model,
    nlgeom: bool,
    u: &[f64],
    data: &ElementData,
    offsets: &[usize],
) -> Vec<f64> {
    table
        .entries
        .par_iter()
        .enumerate()
        .map(|(index, entry)| {
            let formulation = table.formulation(entry);
            let coords = table.coordinates(model, entry);
            let ue = table.gather(entry, u);
            if ue.len() != 3 * coords.len() {
                return data.omega[index];
            }
            let current: Vec<[f64; 3]> = coords
                .iter()
                .enumerate()
                .map(|(a, x)| [x[0] + ue[3 * a], x[1] + ue[3 * a + 1], x[2] + ue[3 * a + 2]])
                .collect();
            let d = table.law(entry, nlgeom).elastic_matrix();
            let mut ke = Vec::new();
            formulation.stiffness(&current, &d, &mut ke);
            let omega = element_frequency(
                &ke,
                &data.lumped[offsets[index]..offsets[index + 1]],
                data.shells[index].as_ref(),
            );
            // An inverted element has no meaningful frequency: keep the reference one.
            if omega.is_finite() && omega > 0.0 {
                omega
            } else {
                data.omega[index]
            }
        })
        .collect()
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
    /// The step times at which an amplitude of the loads has a corner, ascending.
    corners: Vec<f64>,
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
        let mut corners: Vec<f64> = names
            .iter()
            .flatten()
            .filter_map(|name| ctx.model.amplitude(name))
            .flat_map(|a| {
                let offset = a.shift_x - if a.total_time { start_total } else { 0.0 };
                a.points.iter().map(move |&(t, _)| t + offset)
            })
            .filter(|&t| t > 0.0 && t < period)
            .collect();
        corners.sort_by(f64::total_cmp);
        corners.dedup();
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
            corners,
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

    /// The first corner of an amplitude after `time`.
    fn next_corner(&self, time: f64) -> Option<f64> {
        self.corners.iter().copied().find(|&t| t > time)
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
    fn new(
        ctx: &ProcedureContext<'_>,
        set: &ConstraintSet,
        mass: &[f64],
        fibres: Option<&FibreMass>,
    ) -> Result<Self> {
        let n = mass.len();
        let columns = match fibres {
            Some(f) => f.columns(n),
            None => (0..n as u32).map(|j| vec![j]).collect(),
        };
        let pattern = Arc::new(Pattern::from_columns(n, columns));
        let mut m = SparseMatrix::zeros(pattern.clone());
        {
            let values = m.values_mut();
            for (j, &mj) in mass.iter().enumerate() {
                if let Some(p) = pattern.position(j as u32, j as u32) {
                    values[p] = mj;
                }
            }
        }
        if let Some(f) = fibres {
            f.add_to(&mut m);
        }
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

impl Forces {
    /// Contact springs count as internal forces, their damping as viscous.
    fn add_contact(&mut self, springs: &[f64], damping: &[f64]) {
        for (f, c) in self.internal.iter_mut().zip(springs) {
            *f += c;
        }
        for (f, c) in self.viscous.iter_mut().zip(damping) {
            *f += c;
        }
    }
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
                        let (stress, e) = law.stress_without_tangent(
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
                                kinematics.cauchy(&stress)
                            } else {
                                stress
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
                        // No tangent is assembled (`ke` is `None`).
                        (stress, d_elastic)
                    },
                    fe,
                    None,
                );
                // Bulk viscosity on the mean volumetric strain rate, under NLGEOM in the
                // current configuration (a spinning element must not see a rate).
                let current: Option<Vec<[f64; 3]>> =
                    (nlgeom && ue.len() == 3 * coords.len()).then(|| {
                        coords
                            .iter()
                            .enumerate()
                            .map(|(a, x)| {
                                [x[0] + ue[3 * a], x[1] + ue[3 * a + 1], x[2] + ue[3 * a + 2]]
                            })
                            .collect()
                    });
                let moved;
                let (volume, g): (f64, &[f64]) = match &current {
                    Some(x) => {
                        moved = formulation.volume_mean_gradients(x);
                        (moved.0, &moved.1)
                    }
                    None => (viscosity.volume, &viscosity.g),
                };
                let ve: Vec<f64> = entry.dofs.iter().map(|&dof| v[dof as usize]).collect();
                let rate: f64 = ve.iter().zip(g).map(|(v, g)| g * v).sum();
                let q = viscosity.rate * volume * rate;
                for (f, g) in fv.iter_mut().zip(g) {
                    *f = q * g;
                }
                if viscosity.hourglass > 0.0 {
                    hourglass_forces(
                        current.as_deref().unwrap_or(&coords),
                        g,
                        &ve,
                        viscosity.hourglass,
                        fv,
                    );
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

/// Power iterations for the highest frequency of the reduced system.
const REDUCED_ITERATIONS: usize = 30;

/// The highest frequency of the reduced system `Tᵀ K T x = ω² Tᵀ M_L T x` (elastic
/// stiffness), from power iterations on `M_red⁻¹ K_red` and a margin for their slow
/// convergence from below.
fn reduced_frequency(
    ctx: &ProcedureContext<'_>,
    set: &ConstraintSet,
    reduced_mass: &ReducedMass,
    mass: &[f64],
    fibres: Option<&FibreMass>,
) -> Result<f64> {
    let n = ctx.table.dofs.len();
    let r = set.reduced_len();
    if r == 0 {
        return Ok(0.0);
    }
    let mut k = SparseMatrix::zeros(ctx.pattern.clone());
    plxs_assembly::assemble_stiffness(ctx.table, ctx.model, &mut k);
    let k = set.reduce(&k);
    let zero = vec![0.0; n];
    let mass_times = |x: &[f64]| -> Vec<f64> {
        let full = set.expand(x, &zero);
        let mut weighted: Vec<f64> = full.iter().zip(mass).map(|(x, m)| x * m).collect();
        if let Some(f) = fibres {
            f.add_times(&full, &mut weighted);
        }
        set.reduce_rhs(&weighted)
    };
    let mut x: Vec<f64> = (0..r)
        .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 } + 0.3 * ((i as f64) * 1.7).sin())
        .collect();
    let mut kx = vec![0.0; r];
    let mut lambda = 0.0_f64;
    for _ in 0..REDUCED_ITERATIONS {
        k.multiply(&x, &mut kx);
        let mx = mass_times(&x);
        let (num, den) = (dot(&x, &kx), dot(&x, &mx));
        if den <= 0.0 {
            break;
        }
        lambda = num / den;
        let mut next = kx.clone();
        reduced_mass.solve(&mut next)?;
        let norm = next.iter().map(|v| v * v).sum::<f64>().sqrt();
        if norm <= 0.0 || norm.is_nan() {
            break;
        }
        x = next.into_iter().map(|v| v / norm).collect();
    }
    Ok((1.1 * lambda).max(0.0).sqrt())
}

/// What mass scaling added.
struct MassScaling {
    /// Elements that got mass, the mass added and the mass of the model before.
    elements: usize,
    added: f64,
    total: f64,
    /// The element with the largest factor (dense entry index) and the factor.
    largest: Option<(usize, f64)>,
}

/// Mass scaling to the stable time step `target`: every element whose own stable time
/// step (with the bulk viscosity and the factor `contact_step`) is shorter gets the mass
/// that makes it `target`, `(target / Δt_e)²` times its own, on its nodes in the
/// proportion of its lumped mass. Its frequency, bulk and hourglass viscosity follow.
fn scale_masses(
    table: &ElementTable,
    data: &mut ElementData,
    target: f64,
    contact_step: f64,
) -> MassScaling {
    let mut out = MassScaling {
        elements: 0,
        added: 0.0,
        total: 0.0,
        largest: None,
    };
    let mut offset = 0;
    for (index, entry) in table.entries.iter().enumerate() {
        let ndof = entry.dofs.len();
        let lumped = &mut data.lumped[offset..offset + ndof];
        offset += ndof;
        // The mass of the element: the sum over one direction.
        let own: f64 = lumped.iter().step_by(3).sum();
        out.total += own;
        let omega = data.omega[index];
        let dt = stable_time_step_of(&[omega], 0.0, 0.0).stable * contact_step;
        if dt >= target || dt.is_nan() || dt <= 0.0 {
            continue;
        }
        let factor = (target / dt).powi(2);
        for (m, &dof) in lumped.iter_mut().zip(&entry.dofs) {
            data.mass[dof as usize] += (factor - 1.0) * *m;
            *m *= factor;
        }
        let root = factor.sqrt();
        data.omega[index] = omega / root;
        let viscosity = &mut data.viscosity[index];
        viscosity.rate *= root;
        viscosity.hourglass *= root;
        out.elements += 1;
        out.added += (factor - 1.0) * own;
        if out.largest.is_none_or(|(_, f)| factor > f) {
            out.largest = Some((index, factor));
        }
    }
    out
}

/// Tells what the selective mass scaling of the shells did.
fn report_shell_scaling(data: &ElementData, progress: &mut dyn Progress) {
    let scaled: Vec<f64> = data.shells.iter().flatten().map(|s| s.alpha).collect();
    if scaled.is_empty() {
        return;
    }
    let largest = scaled.iter().copied().fold(0.0_f64, f64::max);
    progress.event(ProgressEvent::Info(format!(
        "explicit: {} shell elements with the inertia of their thickness modes scaled (up to {} times; translations unchanged)",
        scaled.len(),
        short(1.0 + largest)
    )));
}

/// Above this fraction of added mass mass scaling warns.
const MASS_SCALING_WARNING: f64 = 0.05;

fn report_mass_scaling(
    ctx: &ProcedureContext<'_>,
    scaling: &MassScaling,
    target: f64,
    progress: &mut dyn Progress,
) {
    if scaling.elements == 0 {
        progress.event(ProgressEvent::Info(format!(
            "explicit: mass scaling to {} s adds no mass (every element is stable at it)",
            short(target)
        )));
        return;
    }
    let fraction = scaling.added / scaling.total.max(f64::MIN_POSITIVE);
    let largest = scaling.largest.map_or(String::new(), |(index, factor)| {
        format!(
            "; most on element {} ({} times its mass)",
            ctx.model.elements[ctx.table.entries[index].element as usize].id,
            short(factor)
        )
    });
    let text = format!(
        "explicit: mass scaling to {} s adds {} ({} % of the model's mass) to {} elements{}",
        short(target),
        short(scaling.added),
        short(100.0 * fraction),
        scaling.elements,
        largest
    );
    progress.event(if fraction > MASS_SCALING_WARNING {
        ProgressEvent::Warning(format!(
            "{text}; above {} % the added inertia changes the response noticeably",
            100.0 * MASS_SCALING_WARNING
        ))
    } else {
        ProgressEvent::Info(text)
    });
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
    stable_time_step_of(&data.omega, data.other_omega, beta)
}

/// The stable time step an explicit integration of `step` would take, from the elements
/// (without the check of the constraints), for the switch of an automatic step.
pub(crate) fn stable_time_step_estimate(
    ctx: &ProcedureContext<'_>,
    step: &Step,
    controls: &ExplicitControls,
) -> Result<f64> {
    let mut data = element_data(
        ctx.table,
        ctx.model,
        step.nlgeom,
        !controls.full_shell_inertia,
    )?;
    let contact_step = if ctx.contact.is_empty() {
        1.0
    } else {
        CONTACT_STEP
    };
    if let Some(target) = controls.mass_scaling.filter(|t| *t > 0.0) {
        scale_masses(ctx.table, &mut data, target, contact_step);
    }
    Ok(stable_time_step(&data, 0.0).stable * contact_step)
}

/// [`stable_time_step`] from the element frequencies `omega` and that of the other
/// elements.
fn stable_time_step_of(omega: &[f64], other_omega: f64, beta: f64) -> Timing {
    let mut largest = other_omega;
    let mut limiting = None;
    for (index, &omega) in omega.iter().enumerate() {
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

/// How stiff a contact spring may be against the masses it joins: `ω_c² Δt² ≤ 0.7`, with
/// `ω_c` from Gershgorin's bound of `M^-½ K_c M^-½`. On its own such a spring would be
/// stable up to `ω Δt = 2`; together with the element at the same node (`ω Δt ≈ 1.7`) and
/// the time step shortened by [`CONTACT_STEP`] the node stays below it.
const CONTACT_STIFFNESS: f64 = 0.7;

/// The factor on the stable time step while contact pairs are in force.
const CONTACT_STEP: f64 = 0.9;

/// Damping of the contact springs as a fraction of the time step (`c = η Δt dp/dd`, see
/// `plxs_contact::Damping`): `ζ ≈ η ω_c Δt / 2 ≈ 0.2` for a spring at the stiffness limit,
/// enough to keep a slave node from chattering on its spring.
const CONTACT_DAMPING: f64 = 0.5;

/// Under NLGEOM the stable time step is found anew every so many time steps.
const NLGEOM_CHECK_STEPS: u64 = 100;

/// Most event frames (contact closing or opening) between two result frames.
const MAX_EVENT_FRAMES: u64 = 32;

/// The contact of an explicit step: the springs of `plxs_contact` evaluated at every time
/// step from the states of the step before (the friction slips), their stiffness capped
/// where it would be too stiff for the time step.
struct ExplicitContact {
    /// The nodes of the pairs and their displacements when the candidates were found.
    nodes: Vec<u32>,
    prepared: Vec<f64>,
    /// How far those nodes may move before the candidates are searched again.
    reach: f64,
    states: Vec<Vec<plxs_core::state::ContactPointState>>,
    /// DOFs whose motion is prescribed: they count as infinitely heavy.
    fixed: Vec<bool>,
    /// The last evaluation (with the capped stiffness), for the outputs.
    last: Option<plxs_contact::Evaluation>,
    /// Points whose stiffness was capped, and the smallest factor.
    capped: usize,
    smallest: f64,
}

impl ExplicitContact {
    fn new(ctx: &mut ProcedureContext<'_>, u: &[f64], fixed: Vec<bool>) -> Self {
        let nodes = ctx.contact.nodes();
        ctx.contact.prepare(&ctx.model.nodes.coordinates, u);
        ctx.contact.stiffness_scale = ctx
            .contact
            .pairs
            .iter()
            .map(|p| {
                let count = if p.mortar_nodes.is_empty() {
                    p.points.len()
                } else {
                    p.mortar_nodes.len()
                };
                vec![1.0; count]
            })
            .collect();
        let prepared = Self::gather(&nodes, u);
        Self {
            nodes,
            prepared,
            reach: 0.5 * ctx.contact.search_reach(),
            states: ctx.state.contact.clone(),
            fixed,
            last: None,
            capped: 0,
            smallest: 1.0,
        }
    }

    fn gather(nodes: &[u32], u: &[f64]) -> Vec<f64> {
        nodes
            .iter()
            .flat_map(|&n| (0..3).map(move |d| u[n as usize * 3 + d]))
            .collect()
    }

    /// The forces of the springs at `u` (velocities `v`, time step `h`): the springs and,
    /// separately, their damping. The states advance to those at `u`.
    fn forces(
        &mut self,
        ctx: &mut ProcedureContext<'_>,
        u: &[f64],
        v: &[f64],
        h: f64,
        mass: &[f64],
    ) -> (Vec<f64>, Vec<f64>) {
        let n = u.len();
        let coords = &ctx.model.nodes.coordinates;
        // A new search for candidates once a node moved more than the search covers.
        let current = Self::gather(&self.nodes, u);
        let moved = current
            .as_chunks::<3>()
            .0
            .iter()
            .zip(self.prepared.as_chunks::<3>().0)
            .map(|(a, b)| {
                ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
            })
            .fold(0.0_f64, f64::max);
        if moved >= self.reach || moved.is_nan() {
            ctx.contact.prepare(coords, u);
            self.prepared = current;
        }
        ctx.contact.damping = Some(plxs_contact::Damping {
            velocities: v.to_vec(),
            coefficient: CONTACT_DAMPING * h,
            rate: 0.0,
        });
        let mut evaluation = ctx.contact.evaluate(coords, u, &self.states, false);
        ctx.contact.damping = None;
        let mut springs = vec![0.0; n];
        let mut damping = vec![0.0; n];
        let limit = CONTACT_STIFFNESS / (h * h);
        // Gershgorin's bound of all springs together on the masses of their nodes, per
        // DOF: a node under several springs (the integration points of a face) takes
        // their sum.
        let moving = |dof: u32| !self.fixed[dof as usize] && mass[dof as usize] > 0.0;
        let mut rows = vec![0.0_f64; n];
        for element in &evaluation.elements {
            let m = element.dofs.len();
            for i in 0..m {
                let di = element.dofs[i];
                if !moving(di) {
                    continue;
                }
                rows[di as usize] += (0..m)
                    .filter(|&j| moving(element.dofs[j]))
                    .map(|j| {
                        element.ke[i * m + j].abs()
                            / (mass[di as usize] * mass[element.dofs[j] as usize]).sqrt()
                    })
                    .sum::<f64>();
            }
        }
        for element in &mut evaluation.elements {
            let largest = element
                .dofs
                .iter()
                .map(|&d| rows[d as usize])
                .fold(0.0_f64, f64::max);
            if largest > limit {
                let factor = limit / largest;
                let (pair, point) = (element.source.0 as usize, element.source.1 as usize);
                if let Some(scale) = ctx
                    .contact
                    .stiffness_scale
                    .get_mut(pair)
                    .and_then(|s| s.get_mut(point))
                {
                    if *scale == 1.0 {
                        self.capped += 1;
                    }
                    *scale *= factor;
                    self.smallest = self.smallest.min(*scale);
                }
                // This step's force with the lowered stiffness as well (a fast impact
                // overcloses by `v Δt` in its first step).
                element.fe.iter_mut().for_each(|f| *f *= factor);
                element.damping.iter_mut().for_each(|f| *f *= factor);
                element.normal.iter_mut().for_each(|f| *f *= factor);
                element.ke.iter_mut().for_each(|k| *k *= factor);
            }
            for (a, &dof) in element.dofs.iter().enumerate() {
                let d = element.damping.get(a).copied().unwrap_or(0.0);
                springs[dof as usize] += element.fe[a] - d;
                damping[dof as usize] += d;
            }
        }
        self.states = evaluation.states();
        self.last = Some(evaluation);
        (springs, damping)
    }

    /// Whether some point is closed now that was open before, or the other way round.
    fn changed(
        before: &[Vec<plxs_core::state::ContactPointState>],
        after: &[Vec<plxs_core::state::ContactPointState>],
    ) -> bool {
        use plxs_core::state::ContactStatus;
        before
            .iter()
            .flatten()
            .zip(after.iter().flatten())
            .any(|(a, b)| (a.status == ContactStatus::Open) != (b.status == ContactStatus::Open))
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
        ctx.table.require_elastic()?;
        // A part of a step that switches between the integrators: from `start` to `end` of
        // the step time, on the state the part before it ended with.
        let segment = ctx.segment.clone();
        let (start, end) = segment.as_ref().map_or((0.0, period), |s| (s.start, s.end));
        // Contact: *MODEL CHANGE / *CHANGE FRICTION of the step, a state per slave point.
        if segment.as_ref().is_none_or(|s| s.first) {
            for warning in ctx.contact.begin_step(step, ctx.state) {
                progress.event(ProgressEvent::Warning(warning));
            }
        }
        let has_contact = ctx.contact.is_active();
        let n = ctx.table.dofs.len();
        let start_total = ctx.state.total_time - start;

        // Lumped mass, element frequencies, bulk viscosity.
        let started = std::time::Instant::now();
        let mut data = element_data(
            ctx.table,
            ctx.model,
            step.nlgeom,
            !controls.full_shell_inertia,
        )?;
        ctx.profile.add(Phase::Assembly, started);
        if let Some(target) = controls.mass_scaling.filter(|t| *t > 0.0) {
            let contact_step = if has_contact { CONTACT_STEP } else { 1.0 };
            let scaling = scale_masses(ctx.table, &mut data, target, contact_step);
            report_mass_scaling(ctx, &scaling, target, progress);
        }
        let fibre_mass = data.fibre_mass(ctx.table);
        report_shell_scaling(&data, progress);

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
        let mut set = ctx
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
        let mut reduced_mass = ReducedMass::new(ctx, &set, &data.mass, fibre_mass.as_ref())?;
        ctx.profile.add(Phase::Factorization, started);
        if let ReducedMass::Factored(_) = reduced_mass {
            progress.event(ProgressEvent::Info(
                "explicit: constraints couple the mass; the reduced mass is factorized once".into(),
            ));
            // The element bound holds for the elements on their own masses; constraints
            // that carry stiffness to lighter nodes (a tie, an equation) can raise the
            // highest frequency of the whole: check it on the reduced system.
            let started = std::time::Instant::now();
            let omega =
                reduced_frequency(ctx, &set, &reduced_mass, &data.mass, fibre_mass.as_ref())?;
            ctx.profile.add(Phase::Assembly, started);
            let elements = data.omega.iter().copied().fold(data.other_omega, f64::max);
            if omega > elements {
                progress.event(ProgressEvent::Info(format!(
                    "explicit: the constraints raise the highest frequency to {} rad/s (elements: {} rad/s)",
                    short(omega),
                    short(elements)
                )));
                data.other_omega = omega;
            }
        }

        // Damping.
        let (alpha_r, beta_r) = explicit_damping(ctx, step, &set, progress)?;

        let contact_step = if has_contact { CONTACT_STEP } else { 1.0 };
        let mut timing = stable_time_step(&data, beta_r);
        timing.stable *= contact_step;
        let fixed_step = controls
            .time_step
            .filter(|t| *t > 0.0 && *t <= timing.stable);
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
        let previous = match &segment {
            Some(s) => s.previous.clone(),
            None => ctx.state.displacements.values.clone(),
        };
        // Rigid bodies and joints turning by finite angles (NLGEOM): the constraints are
        // linearized at the current rotations in every time step and the members put
        // where the rotation takes them.
        let finite_rotations = step.nlgeom && ctx.constraints.has_finite_rotations();
        let rotated_set = |ctx: &ProcedureContext<'_>, u: &[f64]| {
            ctx.constraints
                .at_rotations(u)
                .step_set(ctx.model.nodes.len(), &active, &spc_end.dofs)
        };
        if finite_rotations && motion.u.iter().any(|v| *v != 0.0) {
            set = rotated_set(ctx, &motion.u);
            reduced_mass = ReducedMass::new(ctx, &set, &data.mass, fibre_mass.as_ref())?;
        }
        let varying = step.boundaries.iter().any(|b| {
            b.amplitude.is_some()
                || (step.default_amplitude == DefaultAmplitude::Ramp
                    && b.value.is_some_and(|v| v != 0.0))
        });
        let g_at = |ctx: &ProcedureContext<'_>, set: &ConstraintSet, time: f64| {
            prescribed(ctx, step, set, &previous, time, start_total, period)
        };
        let mut g_cur = g_at(ctx, &set, start)?;
        let varying = varying || g_cur != g_at(ctx, &set, period)?;
        let mut g_prev = g_cur.clone();

        let (_, warnings) = load_at(ctx, step, start, start_total, period)?;
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
            start_total + start,
            &mut material,
            Some((&mut stresses, &mut strains)),
        );
        let mut contact = has_contact.then(|| {
            let mut fixed = vec![false; n];
            for &dof in &spc_end.dofs {
                if let Some(f) = fixed.get_mut(dof as usize) {
                    *f = true;
                }
            }
            ExplicitContact::new(ctx, &u, fixed)
        });
        if let Some(contact) = contact.as_mut() {
            let (springs, damping) = contact.forces(ctx, &u, &v_half, dt_max, &data.mass);
            forces.add_contact(&springs, &damping);
        }
        ctx.profile.add(Phase::Assembly, started);
        let mut p = loads.at(ctx, step, start, start_total, period)?;
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
        let accelerate =
            |set: &ConstraintSet, reduced_mass: &ReducedMass, r: &[f64]| -> Result<Vec<f64>> {
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
        let mut a_red = accelerate(&set, &reduced_mass, &residual(&p, &forces, &f_damp, None))?;
        let zero = vec![0.0; n];
        let mut a_full = set.expand(&a_red, &zero);

        let kinetic = |v: &[f64]| {
            0.5 * v.iter().zip(mass).map(|(v, m)| m * v * v).sum::<f64>()
                + fibre_mass.as_ref().map_or(0.0, |f| f.energy(v))
        };
        let mut energies = Energies {
            internal: 0.5 * dot(&u, &forces.internal),
            kinetic: kinetic(&v_half),
            external_work: segment.as_ref().map_or(0.0, |s| s.external_work),
            dissipation: segment.as_ref().map_or(0.0, |s| s.dissipation),
        };
        // The balance counts from the start of this part.
        let initial_energy =
            energies.internal + energies.kinetic + energies.dissipation - energies.external_work;
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

        // The time steps on the graphics card.
        let mut gpu = if contact.is_none()
            && !varying
            && beta_r == 0.0
            && set.is_selection()
            && matches!(reduced_mass, ReducedMass::Diagonal(_))
            && ctx.table.membranes.is_empty()
            && ctx.table.discrete.is_empty()
        {
            gpu_engine(ctx, step, &kernel, &set, &data.mass, alpha_r, progress)
        } else {
            None
        };
        let frames = ((period / interval) - 1e-9).ceil().max(1.0) as u32;
        let event_frames = parameters.event_frames && has_contact;
        // The frames of this part: from the first frame end after its start to its end.
        let first_frame = ((start / interval) + 1e-9).floor() as u32 + 1;
        let last_frame = (((end / interval) - 1e-9).ceil().max(1.0) as u32).min(frames);
        let mut time = start;
        let mut h_prev = 0.0;
        let mut h;
        let mut steps_done: u64 = 0;
        let mut last_info = std::time::Instant::now();
        let run_started = std::time::Instant::now();
        let mut balance_warned = false;
        let mut balance = 0.0;
        // The result frames written so far (the frames of the step and the event frames).
        let mut record: u32 = segment.as_ref().map_or(0, |s| s.record);
        // An event in the last time step: the next one is written as an event frame.
        let mut event_pending = false;
        let mut omega_now: Option<Vec<f64>> = None;
        for frame in first_frame..=last_frame {
            if ctx.cancel.is_cancelled() {
                ctx.contact.stiffness_scale.clear();
                return Ok(StepOutcome::Cancelled);
            }
            let t_frame = if frame == frames {
                period
            } else {
                f64::from(frame) * interval
            }
            .min(end);
            progress.event(ProgressEvent::Increment {
                step: ctx.step_number,
                increment: record + 1,
                attempt: 1,
                size: t_frame - time,
                step_time: t_frame,
                total_time: start_total + t_frame,
            });
            let frame_start = time;
            let mut frame_steps: u64 = 0;
            let mut events_in_frame: u64 = 0;
            let mut last_event_step: Option<u64> = None;
            // On the graphics card: all time steps of the frame but the last, which the
            // processor computes with the fields of the frame. Under NLGEOM the card stops
            // where the processor would check the stable time step on the deformed elements
            // (every `NLGEOM_CHECK_STEPS` steps), and starts again with the new step.
            while let Some(engine) = gpu.as_mut() {
                let remaining = t_frame - time;
                let count = (remaining / dt_max * (1.0 - 1e-12)).ceil().max(1.0) as usize;
                if count <= 1 {
                    break;
                }
                {
                    let h = remaining / count as f64;
                    let started = std::time::Instant::now();
                    let visc: Vec<f64> = (0..n).map(|i| forces.viscous[i] + f_damp[i]).collect();
                    let a_now = set.expand(&a_red, &zero);
                    // The material state on the card, when elements have state variables.
                    let state_of = |engine: &mut plxs_explicit::Engine| -> std::result::Result<
                        Option<Vec<f64>>,
                        String,
                    > {
                        if material.is_empty() {
                            return Ok(None);
                        }
                        let mut state = material.clone();
                        engine.read_state(&mut state)?;
                        Ok(Some(state))
                    };
                    let check = step.nlgeom.then_some(NLGEOM_CHECK_STEPS as usize);
                    let offset = steps_done as usize;
                    let run = |engine: &mut plxs_explicit::Engine| -> std::result::Result<
                        GpuRun,
                        String,
                    > {
                        engine.load(&plxs_explicit::Start {
                            u: &u,
                            v: &v_half,
                            a: &a_now,
                            internal: &forces.internal,
                            viscous: &visc,
                            loads: &p,
                            material: &material,
                        })?;
                        let mut from = p.clone();
                        let mut done = 0;
                        while done < count - 1 {
                            // The card interpolates the loads linearly over a chunk: a chunk
                            // stays within a straight piece of the amplitudes, the time step
                            // across a corner is a chunk of its own.
                            let mut chunk = (count - 1 - done).min(plxs_explicit::MAX_CHUNK);
                            let t_begin = time + done as f64 * h;
                            if let Some(corner) = loads.next_corner(t_begin + 1e-9 * h) {
                                let within = ((corner - t_begin) / h + 1e-9).floor().max(1.0);
                                if within < chunk as f64 {
                                    chunk = within as usize;
                                }
                            }
                            if let Some(every) = check {
                                chunk = chunk.min(every - (offset + done) % every);
                            }
                            let steps: Vec<(f64, f64)> = (0..chunk)
                                .map(|j| {
                                    let c = if done + j == 0 { 0.5 * (h_prev + h) } else { h };
                                    (h, c)
                                })
                                .collect();
                            let t_end = time + (done + chunk) as f64 * h;
                            let to = loads
                                .at(ctx, step, t_end, start_total, period)
                                .map_err(|e| e.to_string())?;
                            engine.run(&steps, &from, &to)?;
                            from = to;
                            done += chunk;
                            if let Some(every) = check
                                && (offset + done).is_multiple_of(every)
                                && done < count - 1
                            {
                                let reached = engine.read()?;
                                let omega = current_frequencies(
                                    ctx.table,
                                    ctx.model,
                                    true,
                                    &reached.u,
                                    &data,
                                    &kernel.offsets,
                                );
                                let now = stable_time_step_of(&omega, data.other_omega, beta_r);
                                let mut dt = (now.stable * contact_step)
                                    .min(fixed_step.unwrap_or(f64::INFINITY));
                                if !dt.is_finite() {
                                    dt = interval;
                                }
                                return Ok(GpuRun {
                                    reached,
                                    material: state_of(engine)?,
                                    loads: from,
                                    taken: done,
                                    checked: Some((dt, omega)),
                                });
                            }
                        }
                        let reached = engine.read()?;
                        Ok(GpuRun {
                            reached,
                            material: state_of(engine)?,
                            loads: from,
                            taken: done,
                            checked: None,
                        })
                    };
                    let outcome = run(engine);
                    let mut again = false;
                    match outcome {
                        Ok(GpuRun {
                            reached,
                            material: state,
                            loads: p_reached,
                            taken,
                            checked,
                        }) => {
                            if let Some(state) = state {
                                material = state;
                            }
                            if let Some((dt, omega)) = checked {
                                again = true;
                                dt_max = dt;
                                omega_now = Some(omega);
                            }
                            let taken = taken as u64;
                            u = reached.u;
                            v_half = reached.v;
                            q = free.iter().map(|&d| u[d]).collect();
                            qdot = free.iter().map(|&d| v_half[d]).collect();
                            a_red = free.iter().map(|&d| reached.a[d]).collect();
                            energies.internal += reached.internal_work;
                            energies.external_work += reached.external_work;
                            energies.dissipation += reached.dissipation;
                            time += taken as f64 * h;
                            h_prev = h;
                            steps_done += taken;
                            frame_steps += taken;
                            p = p_reached;
                            ctx.profile.add(Phase::Solve, started);
                            let started = std::time::Instant::now();
                            forces = kernel.forces(
                                &u,
                                &v_half,
                                h,
                                start_total + time,
                                &mut material,
                                None,
                            );
                            f_damp =
                                damping_force(&v_half, &forces.internal, &forces.internal, 0.0);
                            ctx.profile.add(Phase::Assembly, started);
                        }
                        Err(e) => {
                            progress.event(ProgressEvent::Warning(format!(
                                "explicit: the graphics card failed ({e}); the time steps go on on the processor"
                            )));
                            gpu = None;
                        }
                    }
                    if !again {
                        break;
                    }
                }
            }
            loop {
                if frame_steps > 0
                    && frame_steps.is_multiple_of(CANCEL_STEPS)
                    && ctx.cancel.is_cancelled()
                {
                    ctx.contact.stiffness_scale.clear();
                    return Ok(StepOutcome::Cancelled);
                }
                // Under NLGEOM the elements change their stable time step as they deform.
                if step.nlgeom && steps_done > 0 && steps_done.is_multiple_of(NLGEOM_CHECK_STEPS) {
                    let started = std::time::Instant::now();
                    let omega =
                        current_frequencies(ctx.table, ctx.model, true, &u, &data, &kernel.offsets);
                    let now = stable_time_step_of(&omega, data.other_omega, beta_r);
                    dt_max = (now.stable * contact_step).min(fixed_step.unwrap_or(f64::INFINITY));
                    if !dt_max.is_finite() {
                        dt_max = interval;
                    }
                    omega_now = Some(omega);
                    ctx.profile.add(Phase::Assembly, started);
                }
                // Equal steps to the end of the frame.
                let remaining = t_frame - time;
                let count = (remaining / dt_max * (1.0 - 1e-12)).ceil().max(1.0);
                h = remaining / count;
                let last_of_frame = count <= 1.0;
                let t_next = if last_of_frame { t_frame } else { time + h };
                let write_event = event_pending && !last_of_frame;
                event_pending = false;
                let started = std::time::Instant::now();
                let g_next = if varying {
                    g_at(ctx, &set, t_next)?
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
                // With finite rotations: the members on their rotated places, the
                // constraints linearized there, and the acceleration `Ṫ q̇` the turning
                // constraints give the members at constant `q̇` (centripetal).
                let mut turning: Option<Vec<f64>> = None;
                let u_next = if finite_rotations {
                    let mut u_next = set.expand(&q, &g_next);
                    ctx.constraints.project_rigid(&mut u_next, &spc_end.dofs);
                    let next_set = rotated_set(ctx, &u_next);
                    if next_set.free != set.free {
                        return Err(Error::Unsupported(
                            "explicit: the rotated constraints chose other independent DOFs".into(),
                        ));
                    }
                    let before = set.expand(&qdot, &zero);
                    let after = next_set.expand(&qdot, &zero);
                    turning = Some(
                        before
                            .iter()
                            .zip(&after)
                            .map(|(b, a)| (a - b) / h)
                            .collect(),
                    );
                    set = next_set;
                    let started = std::time::Instant::now();
                    reduced_mass = ReducedMass::new(ctx, &set, &data.mass, fibre_mass.as_ref())?;
                    ctx.profile.add(Phase::Factorization, started);
                    u_next
                } else {
                    set.expand(&q, &g_next)
                };
                let mut v_next = set.expand(&qdot, &zero);
                if varying {
                    for ((v, gn), gc) in v_next.iter_mut().zip(&g_next).zip(&g_cur) {
                        *v += (gn - gc) / h;
                    }
                }
                ctx.profile.add(Phase::Vectors, started);
                // Forces at the new time.
                let started = std::time::Instant::now();
                let fields =
                    (last_of_frame || write_event).then_some((&mut stresses, &mut strains));
                let mut new_forces = kernel.forces(
                    &u_next,
                    &v_next,
                    h,
                    start_total + t_next,
                    &mut material,
                    fields,
                );
                ctx.profile.add(Phase::Assembly, started);
                if let Some(contact) = contact.as_mut() {
                    let started = std::time::Instant::now();
                    let before = event_frames.then(|| contact.states.clone());
                    let (springs, damping) = contact.forces(ctx, &u_next, &v_next, h, &data.mass);
                    new_forces.add_contact(&springs, &damping);
                    // A contact that closed or opened: the next time step is an event frame
                    // (at most `MAX_EVENT_FRAMES` per frame, spread over it).
                    let expected = ((t_frame - frame_start) / h).max(1.0) as u64;
                    if let Some(before) = before
                        && events_in_frame < MAX_EVENT_FRAMES
                        && last_event_step
                            .is_none_or(|last| frame_steps >= last + expected / MAX_EVENT_FRAMES)
                        && ExplicitContact::changed(&before, &contact.states)
                    {
                        event_pending = true;
                        events_in_frame += 1;
                        last_event_step = Some(frame_steps);
                    }
                    ctx.profile.add(Phase::Contact, started);
                }
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
                let extra: Option<Vec<f64>> = match &turning {
                    Some(t) => Some(
                        t.iter()
                            .zip(&g_acc)
                            .map(|(t, g)| t + if varying { *g } else { 0.0 })
                            .collect(),
                    ),
                    None => varying.then(|| g_acc.clone()),
                };
                let r = residual(&p_next, &new_forces, &f_damp_next, extra.as_deref());
                a_red = accelerate(&set, &reduced_mass, &r)?;
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
                frame_steps += 1;
                ctx.profile.add(Phase::Vectors, started);

                if last_of_frame || write_event {
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
                    balance = (total_energy - initial_energy - energies.external_work).abs()
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
                    motion = Motion {
                        u: u.clone(),
                        v: v_full,
                        a: a_full.clone(),
                    };
                    let mut reactions: Vec<f64> = (0..n)
                        .map(|i| forces.internal[i] + forces.viscous[i] + f_damp[i])
                        .collect();
                    ctx.constraints
                        .transfer_forces(&mut reactions, &spc_end.dofs, &active);
                    if let Some(contact) = &contact {
                        ctx.state.contact.clone_from(&contact.states);
                        ctx.contact.last.clone_from(&contact.last);
                    }
                    record += 1;
                    let total = start_total + time;
                    if !requests.is_empty() {
                        write_energies(
                            ctx.results,
                            &requests,
                            &energies,
                            (ctx.step_number, record, total),
                        );
                    }
                    let mut current_material = material_template.clone();
                    current_material.values.clone_from(&material);
                    if write_event || crate::static_linear::writes_increment(step, frame, frames) {
                        write_increment(
                            ctx,
                            step,
                            record,
                            total,
                            &motion,
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
                        (ctx.step_number, record, total),
                        (&motion.u, Some(&motion.v)),
                        &reactions,
                        &stresses,
                        &strains,
                        Some(&current_material),
                    );
                    if let Some(evaluation) = contact.as_ref().and_then(|c| c.last.as_ref()) {
                        for request in &step.outputs.contact_print {
                            let blocks = plxs_contact::output::history_blocks(
                                &ctx.contact,
                                evaluation,
                                request,
                                &ctx.model.nodes.ids,
                                ctx.step_number,
                                record,
                                total,
                            );
                            ctx.results.history.extend(blocks);
                        }
                    }
                    ctx.profile.add(Phase::Results, started);
                }
                if last_of_frame {
                    break;
                }
            }

            ctx.sta.push(StaRow {
                step: ctx.step_number,
                increment: record,
                attempt: 1,
                iterations: frame_steps.min(u64::from(u32::MAX)) as u32,
                total_time: start_total + time,
                step_time: time,
                increment_size: h,
            });
            progress.event(ProgressEvent::IncrementDone {
                step: ctx.step_number,
                increment: record,
                attempt: 1,
                iterations: frame_steps.min(u64::from(u32::MAX)) as u32,
                total_time: start_total + time,
                step_time: time,
                size: h,
            });
            if last_info.elapsed().as_secs_f64() > PROGRESS_SECONDS || frame == last_frame {
                last_info = std::time::Instant::now();
                let elapsed = run_started.elapsed().as_secs_f64();
                let left = elapsed * (end - time) / (time - start).max(f64::MIN_POSITIVE);
                progress.event(ProgressEvent::Info(format!(
                    "explicit: {} s of {} s, {steps_done} time steps{}, energy balance {} %{}",
                    short(time),
                    short(period),
                    if omega_now.is_some() {
                        format!(", now {} s each", short(h))
                    } else {
                        format!(" of {} s", short(h))
                    },
                    short(100.0 * balance),
                    if frame < last_frame {
                        format!(", about {:.0} s left", left)
                    } else {
                        String::new()
                    }
                )));
            }
        }
        if let Some(contact) = &contact
            && contact.capped > 0
        {
            progress.event(ProgressEvent::Info(format!(
                "explicit: the contact stiffness of {} points was lowered for the time step \
                 (down to {} of the relation's)",
                contact.capped,
                short(contact.smallest)
            )));
        }
        ctx.contact.stiffness_scale.clear();

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
        ctx.state.total_time = start_total + end;
        ctx.state.steps_done = ctx.step_number;
        if let Some(s) = ctx.segment.as_mut() {
            s.record = record;
            s.reached = time;
            s.external_work = energies.external_work;
            s.dissipation = energies.dissipation;
        }
        Ok(StepOutcome::Converged)
    }
}

// The kernels on the card keep the state variables of the processor's J2 law.
const _: () = assert!(plxs_explicit::STATE_SIZE == STATE_SIZE);

/// What a run of time steps on the graphics card reached: the state, the loads at its end,
/// the time steps taken and, under NLGEOM, the last check of the stable time step (the
/// step and the element frequencies).
struct GpuRun {
    reached: plxs_explicit::Reached,
    /// The material state, when elements have state variables.
    material: Option<Vec<f64>>,
    loads: Vec<f64>,
    taken: usize,
    checked: Option<(f64, Vec<f64>)>,
}

/// Models of at least this many DOFs go to a discrete graphics card in the automatic
/// mode (`PLXS_GPU` unset); smaller ones are not worth the transfers.
const GPU_FROM: usize = 30_000;

/// The engine for the time steps of a linear model on the graphics card
/// ([`plxs_explicit`]), when the mode (`PLXS_GPU`, the setting of the application) and the
/// card allow: the element kernels or else the stiffness and the viscous matrix, the
/// inverse lumped mass on the free DOFs.
fn gpu_engine(
    ctx: &mut ProcedureContext<'_>,
    step: &Step,
    kernel: &Kernel<'_>,
    set: &ConstraintSet,
    mass: &[f64],
    alpha: f64,
    progress: &mut dyn Progress,
) -> Option<plxs_explicit::Engine> {
    use plxs_linalg::amg::GpuMode;
    let n = ctx.table.dofs.len();
    let mode = GpuMode::from_environment();
    let any = match mode {
        GpuMode::Off => return None,
        GpuMode::Automatic if n < GPU_FROM => return None,
        GpuMode::Automatic => false,
        GpuMode::On => true,
    };
    let started = std::time::Instant::now();
    let mut inverse = vec![0.0; n];
    for &dof in &set.free {
        let m = mass[dof as usize];
        if m > 0.0 {
            inverse[dof as usize] = 1.0 / m;
        }
    }
    // Element by element where all elements have kernels, otherwise by the matrices.
    let opened = match gpu_elements(ctx, kernel, step.nlgeom) {
        Some(elements) => plxs_explicit::Engine::new(
            &plxs_explicit::Setup {
                forces: plxs_explicit::ForceModel::Elements(&elements),
                inverse_mass: &inverse,
                alpha,
            },
            any,
        )
        .map(|engine| {
            (
                engine,
                "element forces in single precision, state in double precision",
            )
        }),
        // Large deformations need the kernels.
        None if step.nlgeom || ctx.table.has_nonlinear_material() => Err(
            "large deformations and nonlinear materials need element kernels, which not all \
             elements and materials have yet"
                .to_owned(),
        ),
        None => gpu_matrices(ctx, kernel, &inverse, alpha, any).map(|engine| {
            (
                engine,
                "forces in single precision from the motion within a frame, state in double \
                 precision",
            )
        }),
    };
    match opened {
        Ok((engine, how)) => {
            ctx.profile.add(Phase::Assembly, started);
            progress.event(ProgressEvent::Info(format!(
                "explicit: time steps on {} ({how})",
                engine.device()
            )));
            Some(engine)
        }
        Err(e) => {
            if mode == GpuMode::On || n >= GPU_FROM {
                progress.event(ProgressEvent::Info(format!(
                    "explicit: time steps on the processor ({e})"
                )));
            }
            None
        }
    }
}

/// The elements for the kernels on the card, when every element and its material have one
/// (solid elements whose displacement gradient is the sum of nodal gradients, elastic or
/// small-strain J2 materials, no constant blocks).
fn gpu_elements(
    ctx: &ProcedureContext<'_>,
    kernel: &Kernel<'_>,
    nlgeom: bool,
) -> Option<plxs_explicit::Elements> {
    // `PLXS_GPU_FORCES=matrices`: the matrices anyway, for measurements (until the
    // kernels cover all elements and the matrices go).
    if !ctx.table.linear_blocks.is_empty()
        || std::env::var("PLXS_GPU_FORCES").is_ok_and(|v| v.eq_ignore_ascii_case("matrices"))
    {
        return None;
    }
    let mut elements = plxs_explicit::Elements {
        nlgeom,
        ..Default::default()
    };
    for (index, (entry, viscosity)) in ctx.table.entries.iter().zip(kernel.viscosity).enumerate() {
        let formulation = ctx.table.formulation(entry);
        let coords = ctx.table.coordinates(ctx.model, entry);
        let law = ctx.table.law(entry, nlgeom);
        let elastic = law.elastic_matrix();
        let (material, state) = match law.kernel_law()? {
            plxs_materials::KernelLaw::Elastic => (plxs_explicit::Material::Elastic(elastic), None),
            plxs_materials::KernelLaw::J2 {
                shear,
                bulk,
                isotropic,
                kinematic,
            } => (
                plxs_explicit::Material::J2 {
                    shear,
                    bulk,
                    isotropic,
                    kinematic,
                },
                Some(ctx.table.layout.offsets[index] as usize * STATE_SIZE),
            ),
        };
        let gradients = formulation.nodal_gradients(&coords, &elastic)?;
        if entry.dofs.len() != 3 * coords.len() || viscosity.g.len() != entry.dofs.len() {
            return None;
        }
        let hourglass = match (gradients.hourglass, viscosity.hourglass > 0.0) {
            (Some(stiffness), _) => Some(plxs_explicit::Hourglass {
                stiffness,
                viscosity: viscosity.hourglass,
                coordinates: coords.clone(),
            }),
            (None, false) => None,
            (None, true) => return None,
        };
        elements
            .push(plxs_explicit::Element {
                dofs: entry.dofs.clone(),
                points: gradients.points,
                material,
                state,
                bulk: viscosity.rate * viscosity.volume,
                mean: viscosity.g.clone(),
                hourglass,
            })
            .ok()?;
    }
    (!elements.is_empty()).then_some(elements)
}

/// The engine with the stiffness and the viscous matrix (bulk viscosity and hourglass
/// control, linear in the velocities under small strains).
fn gpu_matrices(
    ctx: &mut ProcedureContext<'_>,
    kernel: &Kernel<'_>,
    inverse: &[f64],
    alpha: f64,
    any: bool,
) -> std::result::Result<plxs_explicit::Engine, String> {
    let n = ctx.table.dofs.len();
    if ctx.stiffness.is_none() {
        let mut k = SparseMatrix::zeros(ctx.pattern.clone());
        plxs_assembly::assemble_stiffness(ctx.table, ctx.model, &mut k);
        ctx.stiffness = Some(k);
    }
    let k = ctx.stiffness.as_ref().expect("assembled");
    let mut entries = Vec::with_capacity(2 * k.values().len());
    let pattern = k.pattern();
    for col in 0..n {
        let start = pattern.col_ptr()[col] as usize;
        for (offset, &row) in pattern.column(col).iter().enumerate() {
            let value = k.values()[start + offset];
            entries.push((row, col as u32, value));
            if row as usize != col {
                entries.push((col as u32, row, value));
            }
        }
    }
    let stiffness = plxs_explicit::Csr::from_entries(n, entries);
    // The viscous matrix element by element: the bulk viscosity `c V g gᵀ`, the hourglass
    // control column by column.
    let mut entries = Vec::new();
    for (entry, viscosity) in ctx.table.entries.iter().zip(kernel.viscosity) {
        let dofs = &entry.dofs;
        let m = dofs.len();
        let factor = viscosity.rate * viscosity.volume;
        let mut ce = vec![0.0; m * m];
        if factor != 0.0 && viscosity.g.len() == m {
            for a in 0..m {
                for b in 0..m {
                    ce[a * m + b] += factor * viscosity.g[a] * viscosity.g[b];
                }
            }
        }
        if viscosity.hourglass > 0.0 {
            let coords = ctx.table.coordinates(ctx.model, entry);
            let mut unit = vec![0.0; m];
            for b in 0..m {
                unit[b] = 1.0;
                let mut column = vec![0.0; m];
                hourglass_forces(
                    &coords,
                    &viscosity.g,
                    &unit,
                    viscosity.hourglass,
                    &mut column,
                );
                unit[b] = 0.0;
                for a in 0..m {
                    ce[a * m + b] += column[a];
                }
            }
        }
        for a in 0..m {
            for b in 0..m {
                if ce[a * m + b] != 0.0 {
                    entries.push((dofs[a], dofs[b], ce[a * m + b]));
                }
            }
        }
    }
    let viscous = plxs_explicit::Csr::from_entries(n, entries);
    plxs_explicit::Engine::new(
        &plxs_explicit::Setup {
            forces: plxs_explicit::ForceModel::Matrices {
                stiffness: &stiffness,
                viscous: &viscous,
            },
            inverse_mass: inverse,
            alpha,
        },
        any,
    )
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
                let lower = (fitted.alpha > 0.0 && fitted.beta > 0.0)
                    .then(|| fitted.alpha / (fitted.beta * w2) / std::f64::consts::TAU);
                if let Some(segment) = ctx.segment.as_mut() {
                    segment.lowest = lower;
                }
                lower
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
