//! Comparison of parsed results: the structure must be identical (class D), the values
//! within the tolerance of the fixture's class.
//!
//! Eigenmodes are compared as what they are, bases of eigenspaces: the sign of a mode is
//! arbitrary, and modes whose eigenvalues agree within the tolerance (the two bending modes
//! of a square column) span a subspace in which any orthonormal basis is as right as any
//! other. Consecutive modes of a step with such eigenvalues form a cluster; the mixing
//! matrix that maps the candidate's displacement modes onto the reference's is determined by
//! least squares, must be orthogonal (the normalization agrees), and is applied to every
//! other field of the cluster (stresses, strains and forces are linear in the mode). In the
//! `.dat` the participation factors of a cluster are compared by their root sum of squares
//! per direction and the effective modal masses by their sums, which do not depend on the
//! basis either. Mode shapes get ten times the class tolerance: CalculiX's Lanczos iteration
//! converges the eigenvectors to about 1e-5 (the eigenvalues, second order in the vector
//! error, keep the class tolerance). The last mode of a step may be one of a degenerate pair
//! whose partner was not requested; when its shape cannot be matched it is noted, not
//! failed.

use std::collections::BTreeMap;

use plx_io::dat::DatImport;
use plx_io::frd::FrdImport;
use plx_results::history_output::HistorySet;
use plx_results::{AnalysisKind, Increment};

use crate::Class;

/// Modes whose eigenvalues agree within this relative gap are compared as one subspace.
const MODE_CLUSTER_GAP: f64 = 1e-3;
/// Mode shapes (every field of a mode increment) get this multiple of the class tolerance.
const MODE_SHAPE_FACTOR: f64 = 10.0;

/// Values below this are round-off in every unit system PrePoLix uses (CalculiX prints
/// e.g. reaction forces of 1e-14 where plxs computes exact zeros).
pub const ABSOLUTE_FLOOR: f64 = 1e-7;

/// The increments of a list with only the last static increment of every step: the
/// increments of the nonlinear path may differ from CalculiX's, its end may not. Eigenvalue,
/// sensitivity and other increments stay.
fn static_step_ends(list: &[Increment]) -> Vec<&Increment> {
    let mut out: Vec<&Increment> = Vec::new();
    for i in list {
        match out.last_mut() {
            Some(l) if i.kind == AnalysisKind::Static && l.kind == i.kind && l.step == i.step => {
                *l = i
            }
            _ => out.push(i),
        }
    }
    out
}

/// The largest finite absolute value of several lists.
fn largest<'a>(lists: impl IntoIterator<Item = &'a [f64]>) -> f64 {
    lists
        .into_iter()
        .flatten()
        .map(|v| v.abs())
        .filter(|v| v.is_finite())
        .fold(0.0, f64::max)
}

/// What a comparison found.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Comparison {
    /// Differences in structure or beyond tolerance, as messages for the report.
    pub failures: Vec<String>,
    /// What was deliberately not compared, and why.
    pub notes: Vec<String>,
    /// Modes (step, increment) whose shape could not be matched because they end a step
    /// (see the module documentation); their participation factors are not compared either.
    pub unmatched_modes: Vec<(u32, u32)>,
    /// The largest relative deviation of a value (relative to the largest absolute value of
    /// its component), over all compared components.
    pub max_relative: f64,
    /// Number of values compared.
    pub compared: usize,
}

impl Comparison {
    pub fn ok(&self) -> bool {
        self.failures.is_empty()
    }

    fn fail(&mut self, message: impl Into<String>) {
        if self.failures.len() < 50 {
            self.failures.push(message.into());
        } else if self.failures.len() == 50 {
            self.failures.push("... more failures not listed".into());
        }
    }

    /// Compares two value lists of a component; returns false when they differ in length.
    /// Deviations are relative to the largest absolute reference value of the component, at
    /// least `floor` (the norm of the whole field, so that a component that is pure
    /// round-off, such as the shear stress of a uniaxial state, is judged against the
    /// field) and at least [`ABSOLUTE_FLOOR`].
    fn values(
        &mut self,
        what: &str,
        reference: &[f64],
        candidate: &[f64],
        tolerance: f64,
        floor: f64,
    ) -> bool {
        if reference.len() != candidate.len() {
            self.fail(format!(
                "{what}: {} values in the reference, {} in the candidate",
                reference.len(),
                candidate.len()
            ));
            return false;
        }
        let scale = reference
            .iter()
            .map(|v| v.abs())
            .filter(|v| v.is_finite())
            .fold(0.0_f64, f64::max)
            .max(floor)
            .max(ABSOLUTE_FLOOR);
        let mut worst = 0.0_f64;
        let mut worst_index = 0;
        for (i, (&r, &c)) in reference.iter().zip(candidate).enumerate() {
            if !r.is_finite() || !c.is_finite() {
                if r.is_finite() != c.is_finite() {
                    self.fail(format!(
                        "{what}: entry {i} is {r} in the reference and {c} in the candidate"
                    ));
                    return false;
                }
                continue;
            }
            let relative = (r - c).abs() / scale;
            if relative > worst {
                worst = relative;
                worst_index = i;
            }
        }
        self.compared += reference.len();
        self.max_relative = self.max_relative.max(worst);
        if worst > tolerance {
            self.fail(format!(
                "{what}: deviation {worst:.2e} relative to {scale:.4e} exceeds {tolerance:.0e} (entry {worst_index}: {} vs {})",
                reference[worst_index], candidate[worst_index]
            ));
            return false;
        }
        true
    }
}

/// Compares two `.frd` imports: mesh exactly, increments and raw field components within
/// the class tolerance.
pub fn compare_frd(reference: &FrdImport, candidate: &FrdImport, class: Class) -> Comparison {
    compare_frd_with(reference, candidate, class, &[], &BTreeMap::new())
}

/// [`compare_frd`] leaving out the field components named in `undetermined`
/// (`"DISP U1"`) and the values of whole fields named there (`"FORC"`, a fixture's
/// documented deviations; such a field may also be missing in the candidate).
pub fn compare_frd_except(
    reference: &FrdImport,
    candidate: &FrdImport,
    class: Class,
    undetermined: &[String],
) -> Comparison {
    compare_frd_with(reference, candidate, class, undetermined, &BTreeMap::new())
}

/// [`compare_frd`] with absolute noise levels per field name (`noise` of the fixture's
/// `.toml`): deviations below the level pass whatever the field's magnitude. Fields of
/// round-off size in both files (stresses of a free thermal expansion) need it.
pub fn compare_frd_with_noise(
    reference: &FrdImport,
    candidate: &FrdImport,
    class: Class,
    noise: &BTreeMap<String, f64>,
) -> Comparison {
    compare_frd_with(reference, candidate, class, &[], noise)
}

/// [`compare_frd`] leaving out the `undetermined` components and with the `noise` levels.
pub fn compare_frd_with(
    reference: &FrdImport,
    candidate: &FrdImport,
    class: Class,
    undetermined: &[String],
    noise: &BTreeMap<String, f64>,
) -> Comparison {
    let mut c = Comparison::default();
    let tolerance = class.tolerance();

    // Mesh (class D).
    let (rm, cm) = (&reference.mesh, &candidate.mesh);
    if rm.node_ids() != cm.node_ids() {
        c.fail(format!(
            "mesh: node ids differ ({} nodes in the reference, {} in the candidate)",
            rm.node_count(),
            cm.node_count()
        ));
    } else {
        let flat = |m: &plx_mesh::FeMesh| -> Vec<f64> {
            m.coords().iter().flat_map(|c| c.iter().copied()).collect()
        };
        c.values("mesh: node coordinates", &flat(rm), &flat(cm), 1e-6, 0.0);
    }
    let elements = |m: &plx_mesh::FeMesh| -> Vec<(u32, String, Vec<u32>)> {
        m.elements()
            .iter()
            .map(|e| (e.id, e.type_name.clone(), e.nodes.clone()))
            .collect()
    };
    let (re, ce) = (elements(rm), elements(cm));
    if re != ce {
        if re.len() != ce.len() {
            c.fail(format!(
                "mesh: {} elements in the reference, {} in the candidate",
                re.len(),
                ce.len()
            ));
        } else if let Some((r, k)) = re.iter().zip(&ce).find(|(r, k)| r != k) {
            c.fail(format!(
                "mesh: element {} is {} {:?} in the reference and {} {} {:?} in the candidate",
                r.0, r.1, r.2, k.0, k.1, k.2
            ));
        }
    }

    // Increments. Class B (nonlinear) fixtures compare static steps at their end only: the
    // path there may need another number of increments than CalculiX (the concept asks for
    // the same order of magnitude), and the increments after the onset of yielding differ
    // by more than the tolerance even when their number happens to agree. The other classes
    // and the other kinds of increments compare every increment.
    let step_ends;
    let (references, candidates): (&[Increment], &[Increment]) = if class == Class::B {
        let (rn, kn) = (reference.increments.len(), candidate.increments.len());
        if rn > 3 * kn.max(1) || kn > 3 * rn.max(1) {
            c.fail(format!(
                "{rn} increments in the reference, {kn} in the candidate (more than a factor of 3 apart)"
            ));
        }
        let (r, k) = (
            static_step_ends(&reference.increments),
            static_step_ends(&candidate.increments),
        );
        let steps = |list: &[&Increment]| -> Vec<(u32, AnalysisKind)> {
            list.iter().map(|i| (i.step, i.kind)).collect()
        };
        if steps(&r) != steps(&k) {
            c.fail(format!(
                "increments {:?} in the reference, {:?} in the candidate",
                steps(&r),
                steps(&k)
            ));
            return c;
        }
        // The increment numbers of a step end may differ; the candidate takes the
        // reference's so that the structure compares equal.
        step_ends = (
            r.iter().map(|&i| i.clone()).collect::<Vec<_>>(),
            k.iter()
                .zip(&r)
                .map(|(&k, r)| {
                    let mut k = k.clone();
                    k.increment = r.increment;
                    k
                })
                .collect::<Vec<_>>(),
        );
        (&step_ends.0, &step_ends.1)
    } else {
        if reference.increments.len() != candidate.increments.len() {
            c.fail(format!(
                "{} increments in the reference, {} in the candidate",
                reference.increments.len(),
                candidate.increments.len()
            ));
        }
        (&reference.increments, &candidate.increments)
    };
    let count = references.len().min(candidates.len());
    let mut i = 0;
    while i < count {
        let r = &references[i];
        let k = &candidates[i];
        let label = format!("step {} increment {}", r.step, r.increment);
        if (r.step, r.increment, r.kind, r.nodal_diameter)
            != (k.step, k.increment, k.kind, k.nodal_diameter)
        {
            c.fail(format!(
                "{label}: candidate has step {} increment {} ({:?})",
                k.step, k.increment, k.kind
            ));
            i += 1;
            continue;
        }
        c.values(
            &format!("{label}: value"),
            &[r.value],
            &[k.value],
            tolerance,
            0.0,
        );
        let Some((r_fields, k_fields)) = same_fields(&mut c, &label, r, k, undetermined) else {
            i += 1;
            continue;
        };
        if r.kind == AnalysisKind::Sensitivity {
            let has_senfreq = |i: &Increment| i.fields.iter().any(|f| f.name == "SENFREQ");
            let mut end = i + 1;
            if has_senfreq(r) {
                while end < count {
                    let (next, kn) = (&references[end], &candidates[end]);
                    let close =
                        (next.value - r.value).abs() <= MODE_CLUSTER_GAP * r.value.abs().max(1e-12);
                    if !(next.kind == r.kind
                        && next.step == r.step
                        && has_senfreq(next)
                        && close
                        && (next.step, next.increment, next.kind)
                            == (kn.step, kn.increment, kn.kind))
                    {
                        break;
                    }
                    end += 1;
                }
            }
            let mut structured = true;
            for j in i + 1..end {
                let (rj, kj) = (&references[j], &candidates[j]);
                let label = format!("step {} increment {}", rj.step, rj.increment);
                c.values(
                    &format!("{label}: value"),
                    &[rj.value],
                    &[kj.value],
                    tolerance,
                    0.0,
                );
                structured &= same_fields(&mut c, &label, rj, kj, undetermined).is_some();
            }
            if structured {
                compare_sensitivities(
                    &mut c,
                    &references[i..end],
                    &candidates[i..end],
                    tolerance,
                    undetermined,
                );
            }
            i = end;
            continue;
        }
        if is_mode(r) {
            // The cluster: the following modes of the step whose eigenvalues agree with
            // this one within the tolerance (in the reference).
            let mut end = i + 1;
            while end < count {
                let next = &references[end];
                let close =
                    (next.value - r.value).abs() <= MODE_CLUSTER_GAP * r.value.abs().max(1e-12);
                if !(is_mode(next) && next.step == r.step && next.kind == r.kind && close) {
                    break;
                }
                let kn = &candidates[end];
                if (next.step, next.increment, next.kind) != (kn.step, kn.increment, kn.kind) {
                    break;
                }
                end += 1;
            }
            let mut all_structured = true;
            for j in i + 1..end {
                let (rj, kj) = (&references[j], &candidates[j]);
                let label = format!("step {} increment {}", rj.step, rj.increment);
                c.values(
                    &format!("{label}: value"),
                    &[rj.value],
                    &[kj.value],
                    tolerance,
                    0.0,
                );
                all_structured &= same_fields(&mut c, &label, rj, kj, undetermined).is_some();
            }
            if all_structured {
                let last_of_step = references.get(end).is_none_or(|next| {
                    next.step != r.step || next.kind != r.kind || !is_mode(next)
                });
                compare_mode_cluster(
                    &mut c,
                    &references[i..end],
                    &candidates[i..end],
                    tolerance * MODE_SHAPE_FACTOR,
                    last_of_step,
                    undetermined,
                    noise,
                );
            }
            i = end;
            continue;
        }
        for (rf, kf) in r_fields.iter().zip(&k_fields) {
            // The stress error estimator is a CalculiX-internal heuristic (its numbers do
            // not follow the manual's description); only its presence is compared, see
            // docs/solver/SCHNITTSTELLEN.md.
            if rf.name == "ERROR" || undetermined.contains(&rf.name) {
                continue;
            }
            let to_f64 = |v: &[f32]| -> Vec<f64> { v.iter().map(|&x| f64::from(x)).collect() };
            let raw: Vec<Vec<f64>> = rf
                .components
                .iter()
                .filter(|c| !c.derived)
                .map(|c| to_f64(&c.values))
                .collect();
            let floor = largest(raw.iter().map(|v| v.as_slice()))
                .max(noise.get(&rf.name).map_or(0.0, |n| n / tolerance));
            for rc in rf.components.iter().filter(|c| !c.derived) {
                let Some(kc) = kf.component(&rc.name) else {
                    c.fail(format!("{label}: {} has no component {}", rf.name, rc.name));
                    continue;
                };
                if undetermined.contains(&format!("{} {}", rf.name, rc.name)) {
                    continue;
                }
                c.values(
                    &format!("{label}: {} {}", rf.name, rc.name),
                    &to_f64(&rc.values),
                    &to_f64(&kc.values),
                    tolerance,
                    floor,
                );
            }
        }
        i += 1;
    }
    c
}

/// Compares the data sets of a sensitivity step: one set, or the sets of a cluster of
/// repeated eigenfrequencies (`SENFREQ`). Sensitivities are relative quantities whose size
/// depends on the unit system (mass sensitivities in mm and t are about 1e-9), so every
/// component is compared after dividing both sides by the reference's largest value. In a
/// cluster, the derivative of every single mode depends on the basis the eigensolver picked
/// in the eigenspace; only their sum (the trace of the derivative matrix) does not, so the
/// sum of `DFDN` over the cluster is compared and the normalized filtered fields are not.
fn compare_sensitivities(
    c: &mut Comparison,
    reference: &[Increment],
    candidate: &[Increment],
    tolerance: f64,
    undetermined: &[String],
) {
    let first = &reference[0];
    let scaled =
        |values: &[f64], scale: f64| -> Vec<f64> { values.iter().map(|v| v / scale).collect() };
    let scale_of = |values: &[f64]| -> f64 {
        let s = largest([values]);
        if s > 0.0 { s } else { 1.0 }
    };
    if reference.len() == 1 {
        let label = format!("step {} increment {}", first.step, first.increment);
        for (rf, kf) in first.fields.iter().zip(&candidate[0].fields) {
            if undetermined.contains(&rf.name) {
                continue;
            }
            for rc in rf.components.iter().filter(|c| !c.derived) {
                let Some(kc) = kf.component(&rc.name) else {
                    c.fail(format!("{label}: {} has no component {}", rf.name, rc.name));
                    continue;
                };
                if undetermined.contains(&format!("{} {}", rf.name, rc.name)) {
                    continue;
                }
                let (rv, kv) = (to_f64(&rc.values), to_f64(&kc.values));
                let scale = scale_of(&rv);
                c.values(
                    &format!(
                        "{label}: {} {} (relative to its largest value)",
                        rf.name, rc.name
                    ),
                    &scaled(&rv, scale),
                    &scaled(&kv, scale),
                    tolerance,
                    0.0,
                );
            }
        }
        return;
    }
    let label = format!(
        "step {} sensitivity sets {}-{} (repeated eigenvalue {})",
        first.step,
        first.increment,
        reference[reference.len() - 1].increment,
        first.value
    );
    if undetermined.iter().any(|u| u == "SENFREQ") {
        return;
    }
    let sum = |increments: &[Increment]| -> Option<Vec<f64>> {
        let mut total: Option<Vec<f64>> = None;
        for increment in increments {
            let field = increment.fields.iter().find(|f| f.name == "SENFREQ")?;
            let values = to_f64(&field.component("DFDN")?.values);
            match &mut total {
                Some(t) if t.len() == values.len() => {
                    t.iter_mut().zip(&values).for_each(|(a, b)| *a += b)
                }
                Some(_) => return None,
                None => total = Some(values),
            }
        }
        total
    };
    match (sum(reference), sum(candidate)) {
        (Some(r), Some(k)) => {
            let scale = scale_of(&r);
            c.values(
                &format!("{label}: sum of SENFREQ DFDN (relative to its largest value)"),
                &scaled(&r, scale),
                &scaled(&k, scale),
                tolerance,
                0.0,
            );
        }
        _ => c.fail(format!("{label}: SENFREQ DFDN missing")),
    }
    c.notes.push(format!(
        "{label}: DFDN of the single modes and DFDNFIL depend on the basis of the eigenspace; only the sum of DFDN is compared"
    ));
}

fn to_f64(v: &[f32]) -> Vec<f64> {
    v.iter().map(|&x| f64::from(x)).collect()
}

/// An eigenmode increment (modes count from one; increment zero of a buckling step is the
/// static reference solution).
fn is_mode(increment: &Increment) -> bool {
    matches!(
        increment.kind,
        AnalysisKind::Frequency | AnalysisKind::Buckling
    ) && increment.increment > 0
}

/// Checks that two increments carry the same fields; the names are structure (class D).
fn same_fields<'a>(
    c: &mut Comparison,
    label: &str,
    r: &'a Increment,
    k: &'a Increment,
    undetermined: &[String],
) -> Option<(Vec<&'a plx_results::Field>, Vec<&'a plx_results::Field>)> {
    // A field of a documented deviation may be missing in the candidate (CalculiX's
    // internal stresses of U1 beams, which plxs does not write).
    let missing_ok = |f: &&plx_results::Field| {
        !(undetermined.contains(&f.name) && !k.fields.iter().any(|g| g.name == f.name))
    };
    let r_fields: Vec<&plx_results::Field> = r.fields.iter().filter(missing_ok).collect();
    let k_fields: Vec<&plx_results::Field> = k.fields.iter().collect();
    let names = |fields: &[&plx_results::Field]| -> Vec<String> {
        fields.iter().map(|f| f.name.clone()).collect()
    };
    if names(&r_fields) != names(&k_fields) {
        c.fail(format!(
            "{label}: fields {:?} in the reference, {:?} in the candidate",
            names(&r_fields),
            names(&k_fields)
        ));
        return None;
    }
    Some((r_fields, k_fields))
}

/// Compares a cluster of modes with (nearly) equal eigenvalues as bases of one subspace:
/// the mixing matrix `A` with `R ≈ C A` (columns are the modes' displacement vectors) is
/// fitted by least squares, must be orthogonal within the tolerance, and maps every field
/// of the candidate onto the reference's.
fn compare_mode_cluster(
    c: &mut Comparison,
    reference: &[Increment],
    candidate: &[Increment],
    tolerance: f64,
    last_of_step: bool,
    undetermined: &[String],
    noise: &BTreeMap<String, f64>,
) {
    let m = reference.len();
    let first = &reference[0];
    let label = if m == 1 {
        format!("step {} increment {}", first.step, first.increment)
    } else {
        format!(
            "step {} increments {}-{}",
            first.step,
            first.increment,
            reference[m - 1].increment
        )
    };
    let displacement = |i: &Increment| -> Option<Vec<f64>> {
        let field = i.fields.iter().find(|f| f.name == "DISP")?;
        let mut v = Vec::new();
        for comp in field.components.iter().filter(|c| !c.derived) {
            v.extend(comp.values.iter().map(|&x| f64::from(x)));
        }
        Some(v)
    };
    let (Some(r0), Some(k0)) = (displacement(first), displacement(&candidate[0])) else {
        c.fail(format!(
            "{label}: modes without a DISP field cannot be compared"
        ));
        return;
    };
    let n = r0.len();
    if k0.len() != n {
        c.fail(format!(
            "{label}: DISP has {n} values in the reference, {} in the candidate",
            k0.len()
        ));
        return;
    }
    let mut r_modes = vec![r0];
    let mut k_modes = vec![k0];
    for (rj, kj) in reference.iter().zip(candidate).skip(1) {
        match (displacement(rj), displacement(kj)) {
            (Some(r), Some(k)) if r.len() == n && k.len() == n => {
                r_modes.push(r);
                k_modes.push(k);
            }
            _ => {
                c.fail(format!("{label}: DISP of the modes differ in size"));
                return;
            }
        }
    }
    // Normal equations: (CᵀC) A = CᵀR, with m ≤ a handful.
    let dot = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>();
    let mut ctc = vec![0.0; m * m];
    let mut ctr = vec![0.0; m * m];
    for i in 0..m {
        for j in 0..m {
            ctc[i * m + j] = dot(&k_modes[i], &k_modes[j]);
            ctr[i * m + j] = dot(&k_modes[i], &r_modes[j]);
        }
    }
    let Some(a) = solve_dense(ctc, &ctr, m) else {
        c.fail(format!(
            "{label}: the candidate's modes are linearly dependent"
        ));
        return;
    };
    // Orthogonality of the mixing: both bases are normalized the same way (mass or
    // reference stiffness) exactly when AᵀA = I.
    let mut worst = 0.0_f64;
    for i in 0..m {
        for j in 0..m {
            let v: f64 = (0..m).map(|k| a[k * m + i] * a[k * m + j]).sum();
            let expected = if i == j { 1.0 } else { 0.0 };
            worst = worst.max((v - expected).abs());
        }
    }
    if worst > tolerance {
        if last_of_step {
            c.notes.push(format!(
                "{label}: the shape of the last mode of the step is not compared: its eigenvalue may be degenerate with one that was not requested, and the candidate's mode does not span the reference's (mixing deviates from orthogonal by {worst:.2e})"
            ));
            c.unmatched_modes
                .extend(reference.iter().map(|i| (i.step, i.increment)));
            return;
        }
        c.fail(format!(
            "{label}: the mixing of the candidate's modes onto the reference's is not orthogonal (deviation {worst:.2e}); the modes are normalized differently or span another subspace"
        ));
    }
    c.compared += m * m;
    c.max_relative = c.max_relative.max(worst);
    // Every raw component of every field, mixed.
    for rf in &first.fields {
        if rf.name == "ERROR" || undetermined.contains(&rf.name) {
            continue;
        }
        fn field<'a>(i: &'a Increment, name: &str) -> Option<&'a plx_results::Field> {
            i.fields.iter().find(|g| g.name == name)
        }
        // The floor of the field: its largest raw value over all modes of the cluster.
        let floor = largest(
            reference
                .iter()
                .filter_map(|i| field(i, &rf.name))
                .flat_map(|g| g.components.iter().filter(|c| !c.derived))
                .map(|c| to_f64(&c.values))
                .collect::<Vec<_>>()
                .iter()
                .map(|v| v.as_slice()),
        )
        .max(noise.get(&rf.name).map_or(0.0, |n| n / tolerance));
        for rc in rf.components.iter().filter(|c| !c.derived) {
            if undetermined.contains(&format!("{} {}", rf.name, rc.name)) {
                continue;
            }
            let Some(reference_values) = reference
                .iter()
                .map(|i| {
                    field(i, &rf.name)?
                        .component(&rc.name)
                        .map(|c| to_f64(&c.values))
                })
                .collect::<Option<Vec<_>>>()
            else {
                c.fail(format!(
                    "{label}: {} {} is missing in a mode of the reference",
                    rf.name, rc.name
                ));
                continue;
            };
            let Some(candidate_values) = candidate
                .iter()
                .map(|i| {
                    field(i, &rf.name)?
                        .component(&rc.name)
                        .map(|c| to_f64(&c.values))
                })
                .collect::<Option<Vec<_>>>()
            else {
                c.fail(format!("{label}: {} has no component {}", rf.name, rc.name));
                continue;
            };
            let len = reference_values[0].len();
            if candidate_values
                .iter()
                .chain(&reference_values)
                .any(|v| v.len() != len)
            {
                c.fail(format!(
                    "{label}: {} {} differs in size between the modes",
                    rf.name, rc.name
                ));
                continue;
            }
            for (j, rv) in reference_values.iter().enumerate() {
                let mixed: Vec<f64> = (0..len)
                    .map(|p| (0..m).map(|i| a[i * m + j] * candidate_values[i][p]).sum())
                    .collect();
                let what = if m == 1 {
                    format!("{label}: {} {}", rf.name, rc.name)
                } else {
                    format!(
                        "{label}: {} {} (mode {})",
                        rf.name, rc.name, reference[j].increment
                    )
                };
                c.values(&what, rv, &mixed, tolerance, floor);
            }
        }
    }
}

/// Solves `M X = B` for `X` (all `m × m`, row-major) by Gaussian elimination with partial
/// pivoting; `None` when `M` is singular.
fn solve_dense(mut matrix: Vec<f64>, rhs: &[f64], m: usize) -> Option<Vec<f64>> {
    let mut x = rhs.to_vec();
    for col in 0..m {
        let pivot = (col..m).max_by(|&a, &b| {
            matrix[a * m + col]
                .abs()
                .total_cmp(&matrix[b * m + col].abs())
        })?;
        if matrix[pivot * m + col].abs() < 1e-300 {
            return None;
        }
        if pivot != col {
            for j in 0..m {
                matrix.swap(col * m + j, pivot * m + j);
                x.swap(col * m + j, pivot * m + j);
            }
        }
        for row in 0..m {
            if row == col {
                continue;
            }
            let factor = matrix[row * m + col] / matrix[col * m + col];
            if factor == 0.0 {
                continue;
            }
            for j in 0..m {
                matrix[row * m + j] -= factor * matrix[col * m + j];
                x[row * m + j] -= factor * x[col * m + j];
            }
        }
    }
    for row in 0..m {
        let d = matrix[row * m + row];
        for j in 0..m {
            x[row * m + j] /= d;
        }
    }
    Some(x)
}

/// Compares two `.dat` imports: the same sets, fields, components and entries, values
/// within the class tolerance. `unmatched_modes` are the modes (step, number) whose shape
/// the `.frd` comparison could not match (see [`Comparison::unmatched_modes`]); their
/// participation factors and effective modal masses are left out.
pub fn compare_dat(
    reference: &DatImport,
    candidate: &DatImport,
    class: Class,
    unmatched_modes: &[(u32, u32)],
) -> Comparison {
    compare_dat_except(reference, candidate, class, unmatched_modes, &[])
}

/// [`compare_dat`] leaving out the history components named in `undetermined` as
/// `"<FIELD> <COMPONENT>"` (`"GRADIENT_LENGTH STRAINENERGY"`, a fixture's documented
/// deviations).
pub fn compare_dat_except(
    reference: &DatImport,
    candidate: &DatImport,
    class: Class,
    unmatched_modes: &[(u32, u32)],
    undetermined: &[String],
) -> Comparison {
    compare_dat_with_noise(
        reference,
        candidate,
        class,
        unmatched_modes,
        undetermined,
        &BTreeMap::new(),
    )
}

/// [`compare_dat_except`] with absolute noise levels per history field name (`noise` of the
/// fixture's `.toml`, e.g. `MOMENT_ABOUT_CG`, a difference of large moments that is pure
/// round-off when the contact force acts at its centre).
pub fn compare_dat_with_noise(
    reference: &DatImport,
    candidate: &DatImport,
    class: Class,
    unmatched_modes: &[(u32, u32)],
    undetermined: &[String],
    noise: &BTreeMap<String, f64>,
) -> Comparison {
    let mut c = Comparison::default();
    let tolerance = class.tolerance();
    // CalculiX prints the blocks in the order of the `*NODE PRINT`/`*EL PRINT` requests;
    // the order carries no information, so the sets are matched by name.
    let names = |d: &DatImport| -> Vec<String> {
        let mut names: Vec<String> = d.sets.iter().map(|s| s.name.clone()).collect();
        names.sort();
        names
    };
    if names(reference) != names(candidate) {
        c.fail(format!(
            "history sets {:?} in the reference, {:?} in the candidate",
            names(reference),
            names(candidate)
        ));
        return c;
    }
    for rs in &reference.sets {
        let Some(ks) = candidate.sets.iter().find(|s| s.name == rs.name) else {
            continue;
        };
        let label = format!("history {}", rs.name);
        let rows = |s: &plx_results::history_output::HistorySet| -> Vec<(u32, u32)> {
            s.rows.iter().map(|r| (r.0, r.1)).collect()
        };
        if rows(rs) != rows(ks) {
            c.fail(format!("{label}: rows {:?} vs {:?}", rows(rs), rows(ks)));
            continue;
        }
        let times = |s: &plx_results::history_output::HistorySet| -> Vec<f64> {
            s.rows.iter().map(|r| r.2).collect()
        };
        c.values(
            &format!("{label}: times"),
            &times(rs),
            &times(ks),
            tolerance,
            0.0,
        );
        let fields = |s: &plx_results::history_output::HistorySet| -> Vec<String> {
            s.fields.iter().map(|f| f.name.clone()).collect()
        };
        if fields(rs) != fields(ks) {
            c.fail(format!(
                "{label}: fields {:?} vs {:?}",
                fields(rs),
                fields(ks)
            ));
            continue;
        }
        for (rf, kf) in rs.fields.iter().zip(&ks.fields) {
            let floor = largest(
                rf.components
                    .iter()
                    .flat_map(|c| c.entries.iter().map(|e| e.values.as_slice())),
            )
            .max(noise.get(&rf.name).map_or(0.0, |n| n / tolerance));
            for rc in &rf.components {
                let Some(kc) = kf.component(&rc.name) else {
                    c.fail(format!("{label}: {} has no component {}", rf.name, rc.name));
                    continue;
                };
                let entries =
                    |comp: &plx_results::history_output::HistoryComponent| -> Vec<String> {
                        comp.entries.iter().map(|e| e.name.clone()).collect()
                    };
                if entries(rc) != entries(kc) {
                    c.fail(format!(
                        "{label}: {} {} entries {:?} vs {:?}",
                        rf.name,
                        rc.name,
                        entries(rc),
                        entries(kc)
                    ));
                    continue;
                }
                if undetermined.contains(&format!("{} {}", rf.name, rc.name)) {
                    c.notes.push(format!(
                        "{label}: {} {} not compared (documented deviation)",
                        rf.name, rc.name
                    ));
                    continue;
                }
                for (re, ke) in rc.entries.iter().zip(&kc.entries) {
                    let what = format!("{label}: {} {} {}", rf.name, rc.name, re.name);
                    match rf.name.as_str() {
                        "PARTICIPATION_FACTORS" => {
                            let clusters = mode_clusters(rs, unmatched_modes);
                            c.values(
                                &format!("{what} (root sum of squares per eigenvalue)"),
                                &root_sum_squares(&re.values, &clusters),
                                &root_sum_squares(&ke.values, &clusters),
                                tolerance,
                                floor,
                            );
                        }
                        "EFFECTIVE_MODAL_MASS" => {
                            let clusters = mode_clusters(rs, unmatched_modes);
                            c.values(
                                &format!("{what} (sum per eigenvalue)"),
                                &sums(&re.values, &clusters),
                                &sums(&ke.values, &clusters),
                                tolerance,
                                floor,
                            );
                        }
                        // The `.dat` reader averages the tables of all modes of an
                        // EIGENFREQUENCY response into one row, and the gradient length of
                        // a single one of repeated eigenvalues depends on the basis of the
                        // eigenspace (see `compare_sensitivities`).
                        "GRADIENT_LENGTH" if rc.name == "EIGENFREQUENCY" => {
                            c.notes.push(format!(
                                "{what}: not compared (mean over the modes, basis dependent for repeated eigenvalues)"
                            ));
                        }
                        // Responses and gradient lengths relative to themselves: a mass
                        // gradient in mm and t is about 1e-8.
                        "DESIGN_RESPONSE" | "GRADIENT_LENGTH" => {
                            let scale = largest([re.values.as_slice()]);
                            let scale = if scale > 0.0 { scale } else { 1.0 };
                            let rel =
                                |v: &[f64]| -> Vec<f64> { v.iter().map(|x| x / scale).collect() };
                            c.values(
                                &format!("{what} (relative)"),
                                &rel(&re.values),
                                &rel(&ke.values),
                                tolerance,
                                0.0,
                            );
                        }
                        _ => {
                            c.values(&what, &re.values, &ke.values, tolerance, floor);
                        }
                    }
                }
            }
        }
    }
    c
}

/// Groups the rows of an eigenvalue table (one per mode, the third entry its frequency or
/// buckling factor) into runs of consecutive modes whose eigenvalues agree within
/// [`MODE_CLUSTER_GAP`]; returns the row range of every run, leaving out runs that hold an
/// unmatched mode.
fn mode_clusters(set: &HistorySet, unmatched_modes: &[(u32, u32)]) -> Vec<std::ops::Range<usize>> {
    let mut runs = Vec::new();
    let mut start = 0;
    for i in 1..=set.rows.len() {
        let close = i < set.rows.len()
            && set.rows[i].0 == set.rows[start].0
            && (set.rows[i].2 - set.rows[start].2).abs()
                <= MODE_CLUSTER_GAP * set.rows[start].2.abs().max(1e-12);
        if !close {
            let run = start..i;
            if !set.rows[run.clone()]
                .iter()
                .any(|row| unmatched_modes.contains(&(row.0, row.1)))
            {
                runs.push(run);
            }
            start = i;
        }
    }
    runs
}

fn root_sum_squares(values: &[f64], runs: &[std::ops::Range<usize>]) -> Vec<f64> {
    runs.iter()
        .map(|run| {
            values[run.start.min(values.len())..run.end.min(values.len())]
                .iter()
                .map(|x| x * x)
                .sum::<f64>()
                .sqrt()
        })
        .collect()
}

fn sums(values: &[f64], runs: &[std::ops::Range<usize>]) -> Vec<f64> {
    runs.iter()
        .map(|run| {
            values[run.start.min(values.len())..run.end.min(values.len())]
                .iter()
                .sum::<f64>()
        })
        .collect()
}
