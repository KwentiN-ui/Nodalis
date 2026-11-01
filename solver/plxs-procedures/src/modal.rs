//! Mode-superposition procedures on the modes a `*FREQUENCY, STORAGE=YES` step stored
//! (M3 T14): `*MODAL DYNAMICS` (transient) and `*STEADY STATE DYNAMICS` (harmonic), both
//! with `*MODAL DAMPING`.
//!
//! Transient: every mode `j` obeys `q̈ + 2ζω q̇ + ω² q = φⱼᵀ f(t)` with mass-normalised
//! modes; the load is linear between the increment ends and every increment is integrated
//! exactly (the matrix exponential of the mode's first-order system with the load and its
//! slope as extra states), so the increment size only decides where results are written.
//! Displacements and velocities are the sums of the modes; stresses, strains and the
//! internal forces `K u` follow from the displacements as in a static increment.
//!
//! Steady state: for every excitation frequency `Ω` the modal amplitudes are
//! `qⱼ = φⱼᵀ f / (ω² − Ω² + 2iζωΩ)`; the real and imaginary parts go to `DISP`/`DISPI`
//! and the other fields with an `I`, as CalculiX writes them. The frequencies follow
//! CalculiX's grid: the range is split at the eigenfrequencies inside it, each interval
//! gets `points` frequencies, denser towards its ends with `bias`.
//!
//! No CalculiX code was consulted; the conventions are from the CalculiX manual and
//! black-box runs (docs/solver/SCHNITTSTELLEN.md, "Stand nach M3 T14").

use plxs_assembly::{
    MassRule, Spc, assemble_mass_with, assemble_stiffness, load_vector,
    rotate_concentrated_loads,
};
use plxs_core::results::{IncrementKind, ResultsIncrement};
use plxs_core::state::{NodalField, StoredModes};
use plxs_core::step::{ModalDamping, Procedure as ProcedureKind};
use plxs_core::{Error, Progress, ProgressEvent, Result, Step};
use plxs_linalg::SparseMatrix;

use crate::dynamic::{Motion, multiply, write_increment};
use crate::fields::build_fields;
use crate::static_linear::{integration_point_results, writes_increment};
use crate::{Procedure, ProcedureContext, StepOutcome};

pub struct ModalDynamicsProcedure;

/// The stored modes a modal step works on, or the error CalculiX gives without them.
pub(crate) fn stored_modes<'a>(ctx: &'a ProcedureContext<'_>, keyword: &str) -> Result<&'a StoredModes> {
    let modes = ctx.state.stored_modes.as_ref().ok_or_else(|| {
        Error::input(
            keyword,
            "needs the modes of a preceding *FREQUENCY step with STORAGE=YES",
        )
    })?;
    if modes.nodal_diameters.is_some() {
        return Err(Error::Unsupported(format!(
            "procedure not implemented yet: {keyword} on cyclic symmetric modes"
        )));
    }
    let n = ctx.table.dofs.len();
    if modes.modes.iter().any(|m| m.values.len() != n) {
        return Err(Error::input(
            keyword,
            "the stored modes belong to another model",
        ));
    }
    Ok(modes)
}

/// The damping ratio of every mode: `*MODAL DAMPING` ranges (modes outside them are
/// undamped) or Rayleigh, `ζ = α/(2ω) + βω/2`.
pub(crate) fn damping_ratios(damping: Option<&ModalDamping>, eigenvalues: &[f64]) -> Vec<f64> {
    eigenvalues
        .iter()
        .enumerate()
        .map(|(j, &lambda)| {
            let mode = j as u32 + 1;
            match damping {
                None => 0.0,
                Some(ModalDamping::Direct(rows)) => rows
                    .iter()
                    .rev()
                    .find(|(first, last, _)| (*first..=*last).contains(&mode))
                    .map_or(0.0, |r| r.2),
                Some(ModalDamping::Rayleigh { alpha, beta }) => {
                    let omega = lambda.max(0.0).sqrt();
                    if omega > 0.0 {
                        alpha / (2.0 * omega) + beta * omega / 2.0
                    } else {
                        0.0
                    }
                }
            }
        })
        .collect()
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// `exp(A)` of a small dense matrix (row major) by scaling and squaring of the Taylor
/// series; accurate to round-off for the 4×4 matrices of a mode increment.
fn expm(a: &[f64; 16]) -> [f64; 16] {
    let norm = (0..4)
        .map(|i| (0..4).map(|j| a[i * 4 + j].abs()).sum::<f64>())
        .fold(0.0, f64::max);
    let squarings = if norm > 0.5 {
        (norm / 0.5).log2().ceil() as i32
    } else {
        0
    };
    let scale = 0.5f64.powi(squarings);
    let x: Vec<f64> = a.iter().map(|v| v * scale).collect();
    let mul = |p: &[f64; 16], q: &[f64]| -> [f64; 16] {
        let mut r = [0.0; 16];
        for i in 0..4 {
            for k in 0..4 {
                let pik = p[i * 4 + k];
                if pik != 0.0 {
                    for j in 0..4 {
                        r[i * 4 + j] += pik * q[k * 4 + j];
                    }
                }
            }
        }
        r
    };
    let mut result = [0.0; 16];
    let mut term = [0.0; 16];
    for i in 0..4 {
        result[i * 5] = 1.0;
        term[i * 5] = 1.0;
    }
    for k in 1..=20 {
        let next = mul(&term, &x);
        for (t, n) in term.iter_mut().zip(next) {
            *t = n / k as f64;
        }
        for (r, t) in result.iter_mut().zip(&term) {
            *r += t;
        }
    }
    for _ in 0..squarings {
        result = mul(&result, &result);
    }
    result
}

/// The exact increment of one mode: `(q, q̇)` after `dt` from `(q₀, q̇₀)` with the modal
/// force linear from `p0` to `p1`.
#[derive(Clone, Copy, Debug)]
struct ModeStep {
    e: [f64; 16],
}

impl ModeStep {
    fn new(omega2: f64, zeta: f64, dt: f64) -> ModeStep {
        let omega = omega2.max(0.0).sqrt();
        // States (q, q̇, p, Δp): q̈ = p − ω² q − 2ζω q̇, ṗ = Δp/Δt.
        let mut a = [0.0; 16];
        a[1] = dt;
        a[4] = -omega2 * dt;
        a[5] = -2.0 * zeta * omega * dt;
        a[6] = dt;
        a[11] = 1.0;
        ModeStep { e: expm(&a) }
    }

    fn advance(&self, q: f64, v: f64, p0: f64, p1: f64) -> (f64, f64) {
        let z = [q, v, p0, p1 - p0];
        let row = |i: usize| (0..4).map(|j| self.e[i * 4 + j] * z[j]).sum::<f64>();
        (row(0), row(1))
    }
}

/// Loads at a step time with the step's amplitudes, concentrated loads in local systems
/// rotated, as the other procedures build them.
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

/// Refuses prescribed non-zero displacements (base excitation), which the modal
/// procedures do not compute yet.
fn require_homogeneous_constraints(
    ctx: &ProcedureContext<'_>,
    step: &Step,
    keyword: &str,
    period: f64,
) -> Result<()> {
    let spc = Spc::of_step(
        ctx.model,
        step,
        &ctx.table.dofs,
        period,
        ctx.state.total_time + period,
        period,
    )?;
    if spc.values.iter().any(|v| *v != 0.0) {
        return Err(Error::Unsupported(format!(
            "procedure not implemented yet: {keyword} with prescribed non-zero displacements \
             (base motion, step {})",
            step.name
        )));
    }
    Ok(())
}

fn ensure_stiffness(ctx: &mut ProcedureContext<'_>) -> Result<SparseMatrix> {
    if ctx.stiffness.is_none() {
        let mut k = SparseMatrix::zeros(ctx.pattern.clone());
        ctx.table.require_elastic()?;
        assemble_stiffness(ctx.table, ctx.model, &mut k);
        ctx.stiffness = Some(k);
    }
    Ok(ctx.stiffness.take().unwrap())
}

/// `Σ cⱼ φⱼ`.
fn superpose(modes: &[NodalField], c: &[f64], n: usize) -> Vec<f64> {
    let mut out = vec![0.0; n];
    for (mode, &cj) in modes.iter().zip(c) {
        if cj != 0.0 {
            for (o, v) in out.iter_mut().zip(&mode.values) {
                *o += cj * v;
            }
        }
    }
    out
}

impl Procedure for ModalDynamicsProcedure {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        let ProcedureKind::ModalDynamics(parameters) = &step.procedure else {
            return Err(Error::Unsupported(
                "ModalDynamicsProcedure needs a *MODAL DYNAMICS step".into(),
            ));
        };
        const KEYWORD: &str = "*MODAL DYNAMICS";
        if parameters.steady_state {
            return Err(Error::Unsupported(format!(
                "procedure not implemented yet: *MODAL DYNAMICS, STEADY STATE (step {})",
                step.name
            )));
        }
        let dt = parameters.increment;
        let period = parameters.period;
        if dt <= 0.0 || period <= 0.0 {
            return Err(Error::input(
                KEYWORD,
                "the time increment and the period must be positive",
            ));
        }
        let modes = stored_modes(ctx, KEYWORD)?.clone();
        require_homogeneous_constraints(ctx, step, KEYWORD, period)?;
        let n = ctx.table.dofs.len();
        let zeta = damping_ratios(parameters.damping.as_ref(), &modes.eigenvalues);
        let start_total = ctx.state.total_time;
        progress.event(ProgressEvent::Info(format!(
            "modal dynamics on {} modes",
            modes.modes.len()
        )));

        // Modal coordinates of the motion the step starts from: q = Φᵀ M u.
        let u0 = &ctx.state.displacements.values;
        let v0 = ctx.state.velocities.as_ref().map(|v| v.values.clone());
        let starts_at_rest = u0.iter().all(|x| *x == 0.0)
            && v0.as_ref().is_none_or(|v| v.iter().all(|x| *x == 0.0));
        let (mut q, mut qd) = if starts_at_rest || u0.len() != n {
            (vec![0.0; modes.modes.len()], vec![0.0; modes.modes.len()])
        } else {
            let mut m = SparseMatrix::zeros(ctx.pattern.clone());
            assemble_mass_with(ctx.table, ctx.model, MassRule::Element, &mut m)?;
            let mu = multiply(&m, u0);
            let mv = v0.map(|v| multiply(&m, &v)).unwrap_or(vec![0.0; n]);
            (
                modes.modes.iter().map(|p| dot(&p.values, &mu)).collect(),
                modes.modes.iter().map(|p| dot(&p.values, &mv)).collect(),
            )
        };

        let k = ensure_stiffness(ctx)?;
        let (f0, warnings) = load_at(ctx, step, 0.0, start_total, period)?;
        for warning in warnings {
            progress.event(ProgressEvent::Warning(warning));
        }
        let modal_force =
            |f: &[f64]| -> Vec<f64> { modes.modes.iter().map(|p| dot(&p.values, f)).collect() };
        let mut p_prev = modal_force(&f0);
        let count = ((period / dt) * (1.0 - 1e-9)).ceil().max(1.0) as u32;
        let mut stresses = ctx.table.layout.field(6);
        let mut strains = ctx.table.layout.field(6);
        let mut reactions = vec![0.0; n];
        let mut steps: Vec<ModeStep> = Vec::new();
        let mut stepped_dt = f64::NAN;
        let mut time = 0.0;
        let mut motion = Motion {
            u: vec![0.0; n],
            v: vec![0.0; n],
            a: vec![0.0; n],
        };
        for increment in 1..=count {
            let t1 = if increment == count {
                period
            } else {
                increment as f64 * dt
            };
            let h = t1 - time;
            let total = start_total + t1;
            progress.event(ProgressEvent::Increment {
                step: ctx.step_number,
                increment,
                attempt: 1,
                size: h,
                step_time: t1,
                total_time: total,
            });
            if stepped_dt.is_nan() || (h - stepped_dt).abs() > 1e-12 * h {
                steps = modes
                    .eigenvalues
                    .iter()
                    .zip(&zeta)
                    .map(|(&w2, &z)| ModeStep::new(w2, z, h))
                    .collect();
                stepped_dt = h;
            }
            let (f1, _) = load_at(ctx, step, t1, start_total, period)?;
            let p1 = modal_force(&f1);
            for j in 0..q.len() {
                let (qj, vj) = steps[j].advance(q[j], qd[j], p_prev[j], p1[j]);
                q[j] = qj;
                qd[j] = vj;
            }
            p_prev = p1;
            time = t1;
            motion.u = superpose(&modes.modes, &q, n);
            motion.v = superpose(&modes.modes, &qd, n);
            k.multiply(&motion.u, &mut reactions);
            integration_point_results(ctx.table, ctx.model, &motion.u, &mut strains, &mut stresses);
            progress.event(ProgressEvent::Iteration {
                step: ctx.step_number,
                increment,
                iteration: 1,
                residual: 0.0,
                correction: 0.0,
                converged: true,
            });
            if writes_increment(step, increment, count) {
                write_increment(
                    ctx,
                    step,
                    increment,
                    total,
                    &motion,
                    &reactions,
                    (&stresses, &strains),
                    None,
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
            );
            progress.event(ProgressEvent::IncrementDone {
                step: ctx.step_number,
                increment,
                attempt: 1,
                iterations: 1,
                total_time: total,
                step_time: t1,
                size: h,
            });
        }
        ctx.stiffness = Some(k);
        ctx.state.displacements = NodalField {
            per_node: 3,
            values: motion.u,
        };
        ctx.state.velocities = Some(NodalField {
            per_node: 3,
            values: motion.v,
        });
        ctx.state.stresses = stresses;
        ctx.state.strains = strains;
        ctx.state.total_time = start_total + period;
        ctx.state.steps_done = ctx.step_number;
        Ok(StepOutcome::Converged)
    }
}

pub struct SteadyStateProcedure;

/// The excitation frequencies (Hz) of a steady state step: the range split at the
/// eigenfrequencies inside it, `points` frequencies per interval (ends shared), spaced
/// `f = (f₁+f₂)/2 + (f₂−f₁)/2 · sign(y)|y|^(1/bias)` with `y` evenly in [−1, 1], so that
/// `bias > 1` crowds them towards the interval ends.
pub(crate) fn excitation_frequencies(
    lower: f64,
    upper: f64,
    points: u32,
    bias: f64,
    eigenfrequencies: &[f64],
) -> Vec<f64> {
    let (lower, upper) = if lower <= upper {
        (lower, upper)
    } else {
        (upper, lower)
    };
    let mut ends = vec![lower];
    let mut inside: Vec<f64> = eigenfrequencies
        .iter()
        .copied()
        .filter(|f| *f > lower && *f < upper)
        .collect();
    inside.sort_by(f64::total_cmp);
    for f in inside {
        if f - ends.last().unwrap() > 1e-9 * upper.abs().max(1.0) {
            ends.push(f);
        }
    }
    ends.push(upper);
    let points = points.max(2);
    let bias = if bias > 0.0 { bias } else { 1.0 };
    let mut out = vec![lower];
    for pair in ends.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if b - a <= 0.0 {
            continue;
        }
        for k in 1..points {
            let y = -1.0 + 2.0 * k as f64 / (points - 1) as f64;
            let shaped = y.signum() * y.abs().powf(1.0 / bias);
            out.push(if k == points - 1 {
                b
            } else {
                0.5 * (a + b) + 0.5 * (b - a) * shaped
            });
        }
    }
    out
}

impl Procedure for SteadyStateProcedure {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome> {
        let ProcedureKind::SteadyStateDynamics(parameters) = &step.procedure else {
            return Err(Error::Unsupported(
                "SteadyStateProcedure needs a *STEADY STATE DYNAMICS step".into(),
            ));
        };
        const KEYWORD: &str = "*STEADY STATE DYNAMICS";
        if !parameters.harmonic {
            return Err(Error::Unsupported(format!(
                "procedure not implemented yet: *STEADY STATE DYNAMICS, HARMONIC=NO (step {})",
                step.name
            )));
        }
        let modes = stored_modes(ctx, KEYWORD)?.clone();
        require_homogeneous_constraints(ctx, step, KEYWORD, 1.0)?;
        let n = ctx.table.dofs.len();
        let zeta = damping_ratios(parameters.damping.as_ref(), &modes.eigenvalues);
        let omegas: Vec<f64> = modes.eigenvalues.iter().map(|l| l.max(0.0).sqrt()).collect();
        let eigenfrequencies: Vec<f64> = omegas
            .iter()
            .map(|w| w / (2.0 * std::f64::consts::PI))
            .collect();
        let frequencies = excitation_frequencies(
            parameters.lower,
            parameters.upper,
            parameters.points,
            parameters.bias,
            &eigenfrequencies,
        );
        progress.event(ProgressEvent::Info(format!(
            "steady state dynamics on {} modes at {} frequencies",
            modes.modes.len(),
            frequencies.len()
        )));
        let k = ensure_stiffness(ctx)?;
        let mut stresses = ctx.table.layout.field(6);
        let mut strains = ctx.table.layout.field(6);
        let mut reactions = vec![0.0; n];
        for (index, &frequency) in frequencies.iter().enumerate() {
            let increment = index as u32 + 1;
            // Amplitudes of a steady state step are functions of the frequency.
            let (f, warnings) = load_at(ctx, step, frequency, frequency, frequency)?;
            if index == 0 {
                for warning in warnings {
                    progress.event(ProgressEvent::Warning(warning));
                }
            }
            let big_omega = 2.0 * std::f64::consts::PI * frequency;
            let (mut re, mut im) = (Vec::new(), Vec::new());
            for (j, mode) in modes.modes.iter().enumerate() {
                let p = dot(&mode.values, &f);
                // q = p / (ω² − Ω² + 2iζωΩ)
                let a = omegas[j] * omegas[j] - big_omega * big_omega;
                let b = 2.0 * zeta[j] * omegas[j] * big_omega;
                let d = a * a + b * b;
                if d == 0.0 {
                    return Err(Error::Numerical {
                        step: ctx.step_number,
                        increment,
                        message: format!(
                            "undamped mode {} excited at its eigenfrequency {frequency}",
                            j + 1
                        ),
                    });
                }
                re.push(p * a / d);
                im.push(-p * b / d);
            }
            let mut fields = Vec::new();
            let mut imaginary = Vec::new();
            for (c, out, suffix) in [(&re, &mut fields, false), (&im, &mut imaginary, true)] {
                let u = superpose(&modes.modes, c, n);
                k.multiply(&u, &mut reactions);
                integration_point_results(ctx.table, ctx.model, &u, &mut strains, &mut stresses);
                let mut part =
                    build_fields(ctx.table, ctx.model, step, &u, &reactions, &stresses, &strains);
                if suffix {
                    part.retain(|f| f.name != "ERROR" && f.name != "ENER");
                    for field in &mut part {
                        field.name = match field.name.as_str() {
                            "TOSTRAIN" => "TOSTRAII".to_string(),
                            other => format!("{other}I"),
                        };
                    }
                }
                *out = part;
                if !suffix {
                    crate::history::write_history(
                        ctx.results,
                        ctx.model,
                        ctx.table,
                        step,
                        ctx.step_number,
                        increment,
                        frequency,
                        &u,
                        &reactions,
                        &stresses,
                        &strains,
                    );
                }
            }
            fields.extend(imaginary);
            ctx.results.increments.push(ResultsIncrement {
                step: ctx.step_number,
                increment,
                kind: IncrementKind::SteadyState,
                value: frequency,
                total_time: frequency,
                nodal_diameter: None,
                fields,
            });
        }
        ctx.stiffness = Some(k);
        ctx.state.steps_done = ctx.step_number;
        Ok(StepOutcome::Converged)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A fine Runge-Kutta reference for one mode under a linear load.
    fn rk4(omega2: f64, zeta: f64, dt: f64, q: f64, v: f64, p0: f64, p1: f64) -> (f64, f64) {
        let omega = omega2.sqrt();
        let f = |t: f64, q: f64, v: f64| {
            let p = p0 + (p1 - p0) * t / dt;
            (v, p - omega2 * q - 2.0 * zeta * omega * v)
        };
        let steps = 20_000;
        let h = dt / steps as f64;
        let (mut q, mut v) = (q, v);
        for i in 0..steps {
            let t = i as f64 * h;
            let (k1q, k1v) = f(t, q, v);
            let (k2q, k2v) = f(t + h / 2.0, q + h / 2.0 * k1q, v + h / 2.0 * k1v);
            let (k3q, k3v) = f(t + h / 2.0, q + h / 2.0 * k2q, v + h / 2.0 * k2v);
            let (k4q, k4v) = f(t + h, q + h * k3q, v + h * k3v);
            q += h / 6.0 * (k1q + 2.0 * k2q + 2.0 * k3q + k4q);
            v += h / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
        }
        (q, v)
    }

    #[test]
    fn a_mode_increment_is_exact_for_a_linear_load() {
        for (omega2, zeta, dt) in [
            (2.7e7, 0.1, 2.5e-5),
            (6.6e9, 0.02, 2.5e-5),
            (1.0, 0.0, 0.3),
            (0.0, 0.0, 0.5),
            (4.0, 1.5, 0.2),
        ] {
            let step = ModeStep::new(omega2, zeta, dt);
            let (q, v) = step.advance(0.3, -2.0, 1.5, -0.7);
            let (rq, rv) = rk4(omega2, zeta, dt, 0.3, -2.0, 1.5, -0.7);
            assert!((q - rq).abs() < 1e-9 * (1.0 + rq.abs()), "{omega2} {zeta}: {q} vs {rq}");
            assert!((v - rv).abs() < 1e-8 * (1.0 + rv.abs()), "{omega2} {zeta}: {v} vs {rv}");
        }
    }

    #[test]
    fn modal_damping_ranges_and_rayleigh() {
        let eigen = [4.0, 16.0, 64.0];
        let direct = ModalDamping::Direct(vec![(1, 2, 0.05), (2, 2, 0.1)]);
        assert_eq!(damping_ratios(Some(&direct), &eigen), vec![0.05, 0.1, 0.0]);
        let rayleigh = ModalDamping::Rayleigh {
            alpha: 2.0,
            beta: 0.5,
        };
        let z = damping_ratios(Some(&rayleigh), &eigen);
        assert!((z[0] - (2.0 / 4.0 + 0.5)).abs() < 1e-15);
        assert_eq!(damping_ratios(None, &eigen), vec![0.0; 3]);
    }

    #[test]
    fn steady_state_frequencies_split_at_the_eigenfrequencies() {
        // The modal fixture: 700 to 900 Hz with the first bending pair at 834.26 Hz.
        let f = excitation_frequencies(700.0, 900.0, 21, 1.0, &[834.2596031, 834.2596031, 5005.2]);
        assert_eq!(f.len(), 41);
        assert!((f[1] - 706.7129802).abs() < 1e-6, "{}", f[1]);
        assert!((f[20] - 834.2596031).abs() < 1e-9);
        assert_eq!(f[40], 900.0);
    }
}
