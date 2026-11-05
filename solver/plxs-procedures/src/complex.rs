//! `*COMPLEX FREQUENCY` (M3 T14): the eigenmodes of a rotating structure with Coriolis
//! forces, in the space of the modes a `*FREQUENCY, STORAGE=YES` step stored.
//!
//! With mass-normalised modes `Φ`, their eigenvalues `Λ = diag(ω²)` and the Coriolis
//! matrix `G = 2 ∫ ρ Nᵀ Ω̃ N dV` (`Ω̃ x = ω × x`, the angular velocity of the `CENTRIF`
//! load), the modal coordinates obey `q̈ + Γ q̇ + Λ q = 0` with the skew-symmetric
//! `Γ = Φᵀ G Φ`. With `z = (q, q̇)` this is `ż = A z`, `A = [[0, I], [−Λ, −Γ]]`, a real
//! general matrix whose eigenvalues `s = σ + iμ` come in conjugate pairs: `μ > 0` is the
//! frequency of a complex mode, `σ` its growth rate (zero while the mode is stable).
//!
//! `Λ` may hold negative entries: the spin softening of the stored perturbation modes
//! makes the rigid motions across the axis of a free rotor `−Ω²`, and in the rotating
//! frame the Coriolis forces turn them into two modes at exactly `Ω`. Clamping them to
//! zero (as a `Λ^½` formulation would) gives `0` and `2Ω` instead.
//!
//! As CalculiX prints them (observed with ccx 2.21 on the `rotor` fixtures, no source
//! consulted): the modal coordinates of a mode are scaled so that the largest is 1 (its
//! "participation factors"), the mode `x = Φ c` goes to the `.frd` as its real part
//! (`DISP`, stresses and strains of it) and as magnitude and phase (`PDISP`, phase
//! `atan(Im/Re)` in degrees, plus 180° where the real part is negative); the `.dat` gets
//! the frequencies and the turning direction of every mode about the rotation axis.

use plxs_assembly::ElementTable;
use plxs_core::mesh::ElementRef;
use plxs_core::results::{HistoryBlock, IncrementKind, ModeKind, ResultsIncrement};
use plxs_core::step::{DistributedLoadKind, Procedure as ProcedureKind};
use plxs_core::{Error, Model, Progress, ProgressEvent, Result, Step};
use plxs_linalg::dense::general_eigen;

use crate::fields::{add_mode, build_fields, node_order};
use crate::modal::stored_modes;
use crate::static_linear::integration_point_results;
use crate::{Procedure, ProcedureContext, StepOutcome};

pub struct ComplexFrequencyProcedure;

/// The rotation of the step's `CENTRIF` load: angular velocity vector and the dense
/// indices of the elements it acts on.
struct Rotation {
    omega: [f64; 3],
    axis: [f64; 3],
    entries: Vec<u32>,
}

fn rotation(model: &Model, table: &ElementTable, step: &Step) -> Option<Rotation> {
    let mut found: Option<Rotation> = None;
    for load in &step.loads.distributed {
        let DistributedLoadKind::Centrifugal {
            omega_squared,
            axis,
            ..
        } = &load.kind
        else {
            continue;
        };
        let len = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if len == 0.0 {
            continue;
        }
        let unit = axis.map(|c| c / len);
        let speed = omega_squared.max(0.0).sqrt();
        let elements: Vec<u32> = match &load.region {
            ElementRef::Element(id) => vec![*id],
            ElementRef::Set(name) => model
                .sets
                .element_sets
                .get(name)
                .cloned()
                .unwrap_or_default(),
        };
        let rotation = found.get_or_insert(Rotation {
            omega: unit.map(|c| c * speed),
            axis: unit,
            entries: Vec::new(),
        });
        for id in elements {
            if let Some(index) = model.element_index(id) {
                let entry = table.entry_of[index as usize];
                if entry != u32::MAX && !rotation.entries.contains(&entry) {
                    rotation.entries.push(entry);
                }
            }
        }
    }
    found
}

/// Adds the spin softening `−ω² ∫ ρ Nᵀ (I − a aᵀ) N dV` of every `CENTRIF` load of the
/// step to `k`: the change of the centrifugal force with the displacement, which CalculiX
/// takes into the stiffness of a perturbation step about a rotating state (the rotor
/// fixtures: without it the frequencies are 1.3e-4 too high).
pub(crate) fn add_spin_softening(
    model: &Model,
    table: &ElementTable,
    step: &Step,
    k: &mut plxs_linalg::SparseMatrix,
) -> Result<()> {
    for load in &step.loads.distributed {
        let DistributedLoadKind::Centrifugal {
            omega_squared,
            axis,
            ..
        } = &load.kind
        else {
            continue;
        };
        let len = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if len == 0.0 || *omega_squared == 0.0 {
            continue;
        }
        let a = axis.map(|c| c / len);
        let elements: Vec<u32> = match &load.region {
            ElementRef::Element(id) => vec![*id],
            ElementRef::Set(name) => model
                .sets
                .element_sets
                .get(name)
                .cloned()
                .unwrap_or_default(),
        };
        for id in elements {
            let Some(index) = model.element_index(id) else {
                continue;
            };
            let entry_index = table.entry_of[index as usize];
            if entry_index == u32::MAX {
                continue;
            }
            let entry = &table.entries[entry_index as usize];
            let formulation = table.formulation(entry);
            let nodes = entry.nodes.len();
            let dofs = formulation.dof_count();
            let Some(rho) = entry.density else {
                continue;
            };
            if dofs != 3 * nodes {
                return Err(Error::Unsupported(format!(
                    "spin softening of a CENTRIF load on element type {} is not implemented yet",
                    formulation.kind
                )));
            }
            let coords = table.coordinates(model, entry);
            let mut me = Vec::new();
            formulation.mass(&coords, rho, &mut me);
            let mut block = vec![0.0; dofs * dofs];
            for p in 0..nodes {
                for q in 0..nodes {
                    let m_pq = me[(3 * p) * dofs + 3 * q];
                    for i in 0..3 {
                        for j in 0..3 {
                            let projector = if i == j { 1.0 } else { 0.0 } - a[i] * a[j];
                            block[(3 * p + i) * dofs + 3 * q + j] =
                                -omega_squared * m_pq * projector;
                        }
                    }
                }
            }
            k.add_block(&entry.dofs, &block);
        }
    }
    Ok(())
}

/// `G x` for the Coriolis matrix of the rotating elements, element by element:
/// `(G x)_a = Σ_b 2 m_ab ω × x_b` with the scalar consistent mass `m_ab`.
fn coriolis_times(
    model: &Model,
    table: &ElementTable,
    rotation: &Rotation,
    x: &[f64],
) -> Result<Vec<f64>> {
    let mut out = vec![0.0; x.len()];
    let w = rotation.omega;
    for &index in &rotation.entries {
        let entry = &table.entries[index as usize];
        let formulation = table.formulation(entry);
        let nodes = entry.nodes.len();
        let dofs = formulation.dof_count();
        if dofs != 3 * nodes {
            return Err(Error::Unsupported(format!(
                "procedure not implemented yet: *COMPLEX FREQUENCY, CORIOLIS on element type {}",
                formulation.kind
            )));
        }
        let rho = entry.density.ok_or_else(|| {
            Error::input(
                "*DENSITY",
                format!(
                    "element {} needs a density for the Coriolis forces",
                    table.element_id(model, index)
                ),
            )
        })?;
        let coords = table.coordinates(model, entry);
        let mut me = Vec::new();
        formulation.mass(&coords, rho, &mut me);
        for a in 0..nodes {
            let mut f = [0.0; 3];
            for b in 0..nodes {
                let m_ab = me[(3 * a) * dofs + 3 * b];
                if m_ab == 0.0 {
                    continue;
                }
                let xb: [f64; 3] = std::array::from_fn(|c| x[entry.dofs[3 * b + c] as usize]);
                let cross = [
                    w[1] * xb[2] - w[2] * xb[1],
                    w[2] * xb[0] - w[0] * xb[2],
                    w[0] * xb[1] - w[1] * xb[0],
                ];
                for c in 0..3 {
                    f[c] += 2.0 * m_ab * cross[c];
                }
            }
            for c in 0..3 {
                out[entry.dofs[3 * a + c] as usize] += f[c];
            }
        }
    }
    Ok(out)
}

/// A complex mode in modal coordinates: frequency `mu` (rad/time), growth rate `sigma`
/// (per time, zero for a stable mode) and coefficients.
struct ComplexMode {
    mu: f64,
    sigma: f64,
    re: Vec<f64>,
    im: Vec<f64>,
}

/// The complex modes of `q̈ + Γ q̇ + Λ q = 0`, ascending in frequency, at most `count`:
/// one of every conjugate pair `σ ± iμ`, and of a real pair `±σ` (a diverging motion,
/// frequency 0) the growing one.
fn complex_modes(lambda: &[f64], gamma: &[f64], count: usize) -> Vec<ComplexMode> {
    let m = lambda.len();
    // Rigid motions come out of the eigensolver as ±1e-7 of the largest eigenvalue; as
    // negative ones they would be diverging motions. Spin softening is far above that.
    let largest = lambda.iter().fold(0.0, |a: f64, l| a.max(l.abs()));
    let lambda: Vec<f64> = lambda
        .iter()
        .map(|&l| if l.abs() <= 1e-6 * largest { 0.0 } else { l })
        .collect();
    // A = [[0, I], [−Λ, −Γ]], 2m × 2m.
    let n2 = 2 * m;
    let mut a = vec![0.0; n2 * n2];
    for i in 0..m {
        a[i * n2 + m + i] = 1.0;
        a[(m + i) * n2 + i] = -lambda[i];
        for j in 0..m {
            a[(m + i) * n2 + m + j] = -gamma[i * m + j];
        }
    }
    let (values, vectors) = general_eigen(n2, &a);
    let scale = values
        .iter()
        .map(|(re, im)| re.hypot(*im))
        .fold(0.0, f64::max);
    // A double root with one eigenvector (the free mass above) is only accurate to about
    // the square root of the machine precision, so growth rates below that are noise.
    let tolerance = 1e-9 * scale;
    let noise = 1e-5 * scale;
    // A rigid motion is a double eigenvalue 0 of `A`: one mode for every two of them.
    let mut zeros = 0;
    let mut modes: Vec<ComplexMode> = values
        .iter()
        .zip(&vectors)
        .filter(|((sigma, mu), _)| {
            if sigma.hypot(*mu) <= noise {
                zeros += 1;
                zeros % 2 == 1
            } else {
                *mu > tolerance || (mu.abs() <= tolerance && *sigma > 0.0)
            }
        })
        .map(|((sigma, mu), z)| ComplexMode {
            mu: if *mu > tolerance { *mu } else { 0.0 },
            sigma: if sigma.abs() > noise { *sigma } else { 0.0 },
            re: z[..m].iter().map(|c| c.0).collect(),
            im: z[..m].iter().map(|c| c.1).collect(),
        })
        .collect();
    modes.sort_by(|a, b| a.mu.total_cmp(&b.mu).then(b.sigma.total_cmp(&a.sigma)));
    modes.truncate(count);
    modes
}

/// Scales the coefficients so that the largest is 1 (real), as CalculiX prints them.
fn normalise(mode: &mut ComplexMode) {
    let (mut best, mut size) = (0, -1.0);
    for j in 0..mode.re.len() {
        let s = mode.re[j].hypot(mode.im[j]);
        if s > size * (1.0 + 1e-9) {
            best = j;
            size = s;
        }
    }
    if size <= 0.0 {
        return;
    }
    // Divide by c = re + i im: (a + ib)(re − i im)/|c|².
    let (cr, ci) = (mode.re[best], mode.im[best]);
    let d = cr * cr + ci * ci;
    for j in 0..mode.re.len() {
        let (a, b) = (mode.re[j], mode.im[j]);
        mode.re[j] = (a * cr + b * ci) / d;
        mode.im[j] = (b * cr - a * ci) / d;
    }
    mode.re[best] = 1.0;
    mode.im[best] = 0.0;
}

/// CalculiX's phase in degrees: `atan(Im/Re)`, plus 180° where the real part is negative.
pub(crate) fn phase(re: f64, im: f64) -> f64 {
    if re == 0.0 {
        return if im > 0.0 {
            90.0
        } else if im < 0.0 {
            -90.0
        } else {
            0.0
        };
    }
    let p = (im / re).atan().to_degrees();
    if re < 0.0 { p + 180.0 } else { p }
}

impl Procedure for ComplexFrequencyProcedure {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        let ProcedureKind::ComplexFrequency(parameters) = &step.procedure else {
            return Err(Error::Unsupported(
                "ComplexFrequencyProcedure needs a *COMPLEX FREQUENCY step".into(),
            ));
        };
        const KEYWORD: &str = "*COMPLEX FREQUENCY";
        if parameters.fluid {
            return Err(Error::Unsupported(format!(
                "procedure not implemented yet: *COMPLEX FREQUENCY, FLUID (step {})",
                step.name
            )));
        }
        let modes = stored_modes(ctx, KEYWORD)?.clone();
        let m = modes.modes.len();
        let n = ctx.table.dofs.len();
        let count = if parameters.count == 0 {
            m
        } else {
            (parameters.count as usize).min(m)
        };
        let rotation = rotation(ctx.model, ctx.table, step);
        if parameters.coriolis && rotation.is_none() {
            return Err(Error::input(
                KEYWORD,
                "CORIOLIS needs a CENTRIF load for the rotational speed",
            ));
        }
        // Γ = Φᵀ G Φ.
        let mut gamma = vec![0.0; m * m];
        if parameters.coriolis
            && let Some(rotation) = &rotation
        {
            for j in 0..m {
                let gphi = coriolis_times(ctx.model, ctx.table, rotation, &modes.modes[j].values)?;
                for i in 0..m {
                    gamma[i * m + j] = modes.modes[i]
                        .values
                        .iter()
                        .zip(&gphi)
                        .map(|(a, b)| a * b)
                        .sum();
                }
            }
            // Exactly skew-symmetric.
            for i in 0..m {
                for j in 0..i {
                    let v = 0.5 * (gamma[i * m + j] - gamma[j * m + i]);
                    gamma[i * m + j] = v;
                    gamma[j * m + i] = -v;
                }
                gamma[i * m + i] = 0.0;
            }
        }
        let mut complex = complex_modes(&modes.eigenvalues, &gamma, count);
        for mode in &mut complex {
            normalise(mode);
        }
        progress.event(ProgressEvent::Info(format!(
            "{} complex modes from {m} stored modes",
            complex.len()
        )));

        // Physical modes, results and the turning direction about the axis.
        let axis = rotation.as_ref().map_or([1.0, 0.0, 0.0], |r| r.axis);
        let order = node_order(ctx.model);
        let zero = vec![0.0; n];
        let mut stresses = ctx.table.layout.field(6);
        let mut strains = ctx.table.layout.field(6);
        let mut eigen_rows = Vec::new();
        let mut turning_rows = Vec::new();
        for (index, mode) in complex.iter().enumerate() {
            let number = index as u32 + 1;
            // The physical mode is mass normalised (`xᴴ M x = 1`, observed: twice the
            // orbit area of the participation-scaled mode for a circular whirl).
            let size = mode
                .re
                .iter()
                .zip(&mode.im)
                .map(|(a, b)| a * a + b * b)
                .sum::<f64>()
                .sqrt()
                .max(f64::MIN_POSITIVE);
            let mut x_re = vec![0.0; n];
            let mut x_im = vec![0.0; n];
            for (j, phi) in modes.modes.iter().enumerate() {
                let (a, b) = (mode.re[j] / size, mode.im[j] / size);
                for (k, v) in phi.values.iter().enumerate() {
                    x_re[k] += a * v;
                    x_im[k] += b * v;
                }
            }
            let frequency = mode.mu / (2.0 * std::f64::consts::PI);
            add_mode(
                ctx,
                ModeKind::ComplexFrequency,
                number,
                mode.mu,
                frequency,
                None,
                &x_re,
                Some(&x_im),
            );
            // The fields CalculiX writes for the mode, where the step asks for them.
            let mut fields = Vec::new();
            if crate::fields::writes_fields(step) {
                integration_point_results(ctx.table, ctx.model, &x_re, &mut strains, &mut stresses);
                fields = build_fields(
                    ctx.table, ctx.model, step, &x_re, &zero, &stresses, &strains,
                );
            }
            fields.retain(|f| f.name != "FORC" && f.name != "ENER");
            let (re, _) = ctx.table.reported(ctx.model, &x_re, &zero);
            let (im, _) = ctx.table.reported(ctx.model, &x_im, &zero);
            let variables = crate::fields::requested_variables(step);
            if variables.iter().any(|v| v == "PU") {
                let at = fields
                    .iter()
                    .position(|f| f.name == "ERROR")
                    .unwrap_or(fields.len());
                fields.insert(
                    at,
                    crate::modal::magnitude_phase(
                        "PDISP",
                        &["MAG1", "MAG2", "MAG3", "PHA1", "PHA2", "PHA3"],
                        &order,
                        3,
                        &re,
                        &im,
                    ),
                );
            }
            // The orbit x(t) = Re(x e^{iμt}) turns with −(Re x × Im x) about the axis.
            let mut turn = 0.0;
            for node in 0..re.len() / 3 {
                let a = &re[node * 3..node * 3 + 3];
                let b = &im[node * 3..node * 3 + 3];
                let cross = [
                    a[1] * b[2] - a[2] * b[1],
                    a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0],
                ];
                turn -= cross[0] * axis[0] + cross[1] * axis[1] + cross[2] * axis[2];
            }
            eigen_rows.push((number, vec![mode.mu, frequency, mode.sigma]));
            turning_rows.push((number, vec![if turn >= 0.0 { 1.0 } else { -1.0 }]));
            if fields.is_empty() {
                continue;
            }
            ctx.results.increments.push(ResultsIncrement {
                step: ctx.step_number,
                increment: number,
                kind: IncrementKind::ComplexFrequency,
                value: frequency,
                total_time: ctx.state.total_time,
                nodal_diameter: None,
                fields,
            });
        }
        let table_block = |quantity: &str, rows: Vec<(u32, Vec<f64>)>| HistoryBlock {
            step: ctx.step_number,
            increment: 0,
            quantity: quantity.to_string(),
            set: String::new(),
            time: 0.0,
            rows,
        };
        ctx.results.history.push(table_block(
            plxs_output::EigenTable::COMPLEX_EIGENVALUES,
            eigen_rows,
        ));
        if rotation.is_some() {
            turning_rows.push((0, axis.to_vec()));
            ctx.results.history.push(table_block(
                plxs_output::EigenTable::TURNING_DIRECTION,
                turning_rows,
            ));
        }
        ctx.state.steps_done = ctx.step_number;
        Ok(StepOutcome::Converged)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn without_coriolis_the_complex_modes_are_the_real_ones() {
        let modes = complex_modes(&[4.0, 9.0], &[0.0; 4], 2);
        assert_eq!(modes.len(), 2);
        assert!((modes[0].mu - 2.0).abs() < 1e-12);
        assert!((modes[1].mu - 3.0).abs() < 1e-12);
    }

    #[test]
    fn coriolis_splits_a_degenerate_pair() {
        // Two equal modes coupled by Γ = [[0, g], [−g, 0]]: μ² ∓ gμ − ω² = 0.
        let (w2, g) = (100.0, 2.0);
        let modes = complex_modes(&[w2, w2], &[0.0, g, -g, 0.0], 2);
        let lower = (-g + (g * g + 4.0 * w2).sqrt()) / 2.0;
        let upper = (g + (g * g + 4.0 * w2).sqrt()) / 2.0;
        assert!((modes[0].mu - lower).abs() < 1e-10, "{}", modes[0].mu);
        assert!((modes[1].mu - upper).abs() < 1e-10, "{}", modes[1].mu);
        let mut m = ComplexMode {
            mu: modes[0].mu,
            sigma: 0.0,
            re: modes[0].re.clone(),
            im: modes[0].im.clone(),
        };
        normalise(&mut m);
        // A circular whirl: both coefficients of modulus 1, a quarter period apart.
        assert!((m.re[1].hypot(m.im[1]) - 1.0).abs() < 1e-9);
        assert!(m.re[1].abs() < 1e-9);
    }

    #[test]
    fn a_spin_softened_rigid_motion_circles_at_the_speed_of_rotation() {
        // A free mass in the rotating frame: Λ = −Ω² (spin softening), Γ = 2Ω [[0, 1], [−1, 0]]
        // (Coriolis). Seen from the rotating frame a mass at rest circles at Ω, so both
        // modes lie at exactly Ω; clamping Λ to zero would give 0 and 2Ω.
        let omega = 1675.5;
        let modes = complex_modes(
            &[-omega * omega, -omega * omega],
            &[0.0, 2.0 * omega, -2.0 * omega, 0.0],
            2,
        );
        assert_eq!(modes.len(), 2);
        for mode in &modes {
            // A double root with a single eigenvector: accurate to about √ε only.
            assert!((mode.mu - omega).abs() < 1e-4 * omega, "{}", mode.mu);
            assert_eq!(mode.sigma, 0.0);
        }
    }

    #[test]
    fn a_slightly_negative_rigid_mode_is_not_diverging() {
        let modes = complex_modes(&[-0.6, 1.0e8], &[0.0; 4], 2);
        assert_eq!(modes.len(), 2);
        assert!(
            modes[0].mu.abs() < 1e-3 && modes[0].sigma == 0.0,
            "{} {}",
            modes[0].mu,
            modes[0].sigma
        );
        assert!((modes[1].mu - 1.0e4).abs() < 1e-6);
    }

    #[test]
    fn a_diverging_motion_is_a_growing_mode_without_frequency() {
        // Λ < 0 without Coriolis: q̈ = Ω² q, growing as e^{Ωt}.
        let modes = complex_modes(&[-4.0, 9.0], &[0.0; 4], 2);
        assert_eq!(modes.len(), 2);
        assert_eq!(modes[0].mu, 0.0);
        assert!((modes[0].sigma - 2.0).abs() < 1e-12, "{}", modes[0].sigma);
        assert!((modes[1].mu - 3.0).abs() < 1e-12);
        assert_eq!(modes[1].sigma, 0.0);
    }

    #[test]
    fn phases_as_calculix_prints_them() {
        assert!((phase(-2.71162, 2.152) - 141.56).abs() < 0.01);
        assert!((phase(1.0, -1.0) + 45.0).abs() < 1e-12);
        assert!((phase(-1.0, -1.0) - 225.0).abs() < 1e-12);
    }
}
