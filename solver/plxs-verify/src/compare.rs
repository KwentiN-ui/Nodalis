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

    /// Compares two value lists of a component; returns false when they differ in length.
    fn values(&mut self, what: &str, reference: &[f64], candidate: &[f64], tolerance: f64) -> bool {
        self.values_with_floor(what, reference, candidate, tolerance, 1e-12)
    }

    /// As [`Self::values`], with the scale of the component at least `floor`.
    fn values_with_floor(
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
            .max(floor);
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
    compare_frd_except(reference, candidate, class, &[])
}

/// Like [`compare_frd`], but the values of the named fields (a fixture's documented
/// deviations) are not compared; their presence and components still are.
pub fn compare_frd_except(
    reference: &FrdImport,
    candidate: &FrdImport,
    class: Class,
    skipped: &[String],
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

    // Increments.
    if reference.increments.len() != candidate.increments.len() {
        c.fail(format!(
            "{} increments in the reference, {} in the candidate",
            reference.increments.len(),
            candidate.increments.len()
        ));
    }
    for (r, k) in reference.increments.iter().zip(&candidate.increments) {
        let label = format!("step {} increment {}", r.step, r.increment);
        if (r.step, r.increment, r.kind, r.nodal_diameter)
            != (k.step, k.increment, k.kind, k.nodal_diameter)
        {
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
        // A field of a documented deviation may be missing in the candidate (CalculiX's
        // internal stresses of U1 beams, which plxs does not write).
        let missing_ok = |f: &&plx_results::Field| {
            !(skipped.contains(&f.name) && !k.fields.iter().any(|g| g.name == f.name))
        };
        let r_fields: Vec<&plx_results::Field> = r.fields.iter().filter(missing_ok).collect();
        let names = |fields: &[&plx_results::Field]| -> Vec<String> {
            fields.iter().map(|f| f.name.clone()).collect()
        };
        let k_fields: Vec<&plx_results::Field> = k.fields.iter().collect();
        if names(&r_fields) != names(&k_fields) {
            c.fail(format!(
                "{label}: fields {:?} in the reference, {:?} in the candidate",
                names(&r_fields),
                names(&k_fields)
            ));
            continue;
        }
        for (rf, kf) in r_fields.iter().zip(&k_fields) {
            // The stress error estimator is a CalculiX-internal heuristic (its numbers do
            // not follow the manual's description); only its presence is compared, see
            // docs/solver/SCHNITTSTELLEN.md.
            if rf.name == "ERROR" || skipped.contains(&rf.name) {
                continue;
            }
            // Values are compared relative to the largest value of the whole field (the
            // "Feldnorm" of the concept, section 6.2), not of their own component: components
            // of vectors and tensors mix under a change of axes, and small components (shear
            // stresses beside large normal stresses, out-of-plane values of plane models that
            // CalculiX writes as round-off noise) would otherwise be held to the noise of
            // CalculiX's own output (its nodal values of quadratic wedges scatter by about
            // 1e-6 of the field).
            let field_scale = rf
                .components
                .iter()
                .filter(|c| !c.derived)
                .flat_map(|c| c.values.iter())
                .map(|v| f64::from(v.abs()))
                .filter(|v| v.is_finite())
                .fold(0.0_f64, f64::max);
            let floor = field_scale.max(1e-12);
            for rc in rf.components.iter().filter(|c| !c.derived) {
                let Some(kc) = kf.component(&rc.name) else {
                    c.fail(format!("{label}: {} has no component {}", rf.name, rc.name));
                    continue;
                };
                let to_f64 = |v: &[f32]| -> Vec<f64> { v.iter().map(|&x| f64::from(x)).collect() };
                c.values_with_floor(
                    &format!("{label}: {} {}", rf.name, rc.name),
                    &to_f64(&rc.values),
                    &to_f64(&kc.values),
                    tolerance,
                    floor,
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
    let names = |d: &DatImport| -> Vec<String> { d.sets.iter().map(|s| s.name.clone()).collect() };
    if names(reference) != names(candidate) {
        c.fail(format!(
            "history sets {:?} in the reference, {:?} in the candidate",
            names(reference),
            names(candidate)
        ));
        return c;
    }
    for (rs, ks) in reference.sets.iter().zip(&candidate.sets) {
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
                for (re, ke) in rc.entries.iter().zip(&kc.entries) {
                    c.values(
                        &format!("{label}: {} {} {}", rf.name, rc.name, re.name),
                        &re.values,
                        &ke.values,
                        tolerance,
                    );
                }
            }
        }
    }
    c
}
