//! Comparison of parsed results: the structure must be identical (class D), the values
//! within the tolerance of the fixture's class.

use plx_io::dat::DatImport;
use plx_io::frd::FrdImport;

use crate::Class;

/// What a comparison found.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Comparison {
    /// Differences in structure or beyond tolerance, as messages for the report.
    pub failures: Vec<String>,
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

    /// Compares two value lists of a component relative to the largest absolute value of
    /// the reference; returns false when they differ in length.
    fn values(&mut self, what: &str, reference: &[f64], candidate: &[f64], tolerance: f64) -> bool {
        let scale = largest(reference);
        self.values_scaled(what, reference, candidate, tolerance, scale)
    }

    /// Compares two value lists of a component relative to `scale` (the largest absolute
    /// value of the whole field, so that a component that is zero up to round-off, such as
    /// the lateral stress of a bar in tension, is not compared against its own noise).
    fn values_scaled(
        &mut self,
        what: &str,
        reference: &[f64],
        candidate: &[f64],
        tolerance: f64,
        scale: f64,
    ) -> bool {
        if reference.len() != candidate.len() {
            self.fail(format!(
                "{what}: {} values in the reference, {} in the candidate",
                reference.len(),
                candidate.len()
            ));
            return false;
        }
        let scale = scale.max(largest(reference)).max(1e-12);
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

/// The last increment of every step of a list.
fn last_of_step(list: &[plx_results::Increment]) -> Vec<&plx_results::Increment> {
    let mut out: Vec<&plx_results::Increment> = Vec::new();
    for i in list {
        match out.last_mut() {
            Some(l) if l.step == i.step => *l = i,
            _ => out.push(i),
        }
    }
    out
}

/// The largest finite absolute value of a list.
fn largest(values: &[f64]) -> f64 {
    values
        .iter()
        .map(|v| v.abs())
        .filter(|v| v.is_finite())
        .fold(0.0_f64, f64::max)
}

/// Compares two `.frd` imports: mesh exactly, increments and raw field components within
/// the class tolerance.
pub fn compare_frd(reference: &FrdImport, candidate: &FrdImport, class: Class) -> Comparison {
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
        c.values("mesh: node coordinates", &flat(rm), &flat(cm), 1e-6);
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

    // Increments. Class B (nonlinear) fixtures are compared at the end of every step only:
    // the path there may need another number of increments than CalculiX (the concept asks
    // for the same order of magnitude), and the increments after the onset of yielding
    // differ by more than the tolerance even when their number happens to agree. The other
    // classes compare every increment.
    let structure = |i: &plx_results::Increment| (i.step, i.increment, i.kind, i.nodal_diameter);
    let same_structure = reference.increments.len() == candidate.increments.len()
        && reference
            .increments
            .iter()
            .zip(&candidate.increments)
            .all(|(r, k)| {
                structure(r) == structure(k)
                    && (r.value - k.value).abs() <= tolerance * r.value.abs().max(1e-12)
            });
    let per_increment = same_structure && class != Class::B;
    let pairs: Vec<(&plx_results::Increment, &plx_results::Increment)> = if per_increment {
        reference
            .increments
            .iter()
            .zip(&candidate.increments)
            .collect()
    } else if class == Class::B {
        let (rn, kn) = (reference.increments.len(), candidate.increments.len());
        if rn > 3 * kn.max(1) || kn > 3 * rn.max(1) {
            c.fail(format!(
                "{rn} increments in the reference, {kn} in the candidate (more than a factor of 3 apart)"
            ));
        }
        let (r, k) = (
            last_of_step(&reference.increments),
            last_of_step(&candidate.increments),
        );
        if r.len() != k.len() || r.iter().zip(&k).any(|(a, b)| a.step != b.step) {
            c.fail(format!(
                "steps {:?} in the reference, {:?} in the candidate",
                r.iter().map(|i| i.step).collect::<Vec<_>>(),
                k.iter().map(|i| i.step).collect::<Vec<_>>()
            ));
            return c;
        }
        r.into_iter().zip(k).collect()
    } else {
        if reference.increments.len() != candidate.increments.len() {
            c.fail(format!(
                "{} increments in the reference, {} in the candidate",
                reference.increments.len(),
                candidate.increments.len()
            ));
        }
        reference
            .increments
            .iter()
            .zip(&candidate.increments)
            .collect()
    };
    for (r, k) in pairs {
        let label = if per_increment {
            format!("step {} increment {}", r.step, r.increment)
        } else {
            format!(
                "step {} end (increment {} vs {})",
                r.step, r.increment, k.increment
            )
        };
        if same_structure && structure(r) != structure(k) {
            c.fail(format!(
                "{label}: candidate has step {} increment {} ({:?})",
                k.step, k.increment, k.kind
            ));
            continue;
        }
        c.values(
            &format!("{label}: value"),
            &[r.value],
            &[k.value],
            tolerance,
        );
        fn names(i: &plx_results::Increment) -> Vec<&str> {
            i.fields.iter().map(|f| f.name.as_str()).collect()
        }
        if names(r) != names(k) {
            c.fail(format!(
                "{label}: fields {:?} in the reference, {:?} in the candidate",
                names(r),
                names(k)
            ));
            continue;
        }
        for (rf, kf) in r.fields.iter().zip(&k.fields) {
            // The stress error estimator is a CalculiX-internal heuristic (its numbers do
            // not follow the manual's description); only its presence is compared, see
            // docs/solver/SCHNITTSTELLEN.md.
            if rf.name == "ERROR" {
                continue;
            }
            let to_f64 = |v: &[f32]| -> Vec<f64> { v.iter().map(|&x| f64::from(x)).collect() };
            let field_scale = rf
                .components
                .iter()
                .filter(|c| !c.derived)
                .map(|c| largest(&to_f64(&c.values)))
                .fold(0.0_f64, f64::max);
            for rc in rf.components.iter().filter(|c| !c.derived) {
                let Some(kc) = kf.component(&rc.name) else {
                    c.fail(format!("{label}: {} has no component {}", rf.name, rc.name));
                    continue;
                };
                c.values_scaled(
                    &format!("{label}: {} {}", rf.name, rc.name),
                    &to_f64(&rc.values),
                    &to_f64(&kc.values),
                    tolerance,
                    field_scale,
                );
            }
        }
    }
    c
}

/// Compares two `.dat` imports: the same sets, fields, components and entries, values
/// within the class tolerance.
pub fn compare_dat(reference: &DatImport, candidate: &DatImport, class: Class) -> Comparison {
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
                let field_scale = rf
                    .components
                    .iter()
                    .flat_map(|c| c.entries.iter())
                    .map(|e| largest(&e.values))
                    .fold(0.0_f64, f64::max);
                for (re, ke) in rc.entries.iter().zip(&kc.entries) {
                    c.values_scaled(
                        &format!("{label}: {} {} {}", rf.name, rc.name, re.name),
                        &re.values,
                        &ke.values,
                        tolerance,
                        field_scale,
                    );
                }
            }
        }
    }
    c
}
