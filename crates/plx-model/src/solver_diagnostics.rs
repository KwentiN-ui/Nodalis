//! The findings of the solver's model check (Nodalis solver), attached to the tree: free
//! motions to the boundary conditions of their step and to the parts that move, untied
//! nodes to the analysis and their parts. The solver found them exactly; the geometric
//! estimates of [`crate::checks`] are only a quick hint while editing.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use plx_mesh::FeMesh;
use plxs_core::{Diagnostic, DiagnosticCode, ItemRef, Severity as SolverSeverity};

use crate::checks::{Finding, Problem, describe_motions};
use crate::{FeModel, ModelItem, ModelSpace};

/// The item of the model each item of a lowered analysis came from, filled by the lowering
/// (`plx_io::inp::lower`): a diagnostic of the solver that names an [`ItemRef`] points at
/// the item in the tree through it, without its message being read.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Origins(BTreeMap<ItemRef, ModelItem>);

impl Origins {
    pub fn insert(&mut self, item: ItemRef, origin: ModelItem) {
        self.0.insert(item, origin);
    }

    /// The item of the model `item` came from.
    pub fn get(&self, item: &ItemRef) -> Option<ModelItem> {
        self.0.get(item).copied()
    }

    pub fn iter(&self) -> impl Iterator<Item = (&ItemRef, &ModelItem)> {
        self.0.iter()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// The findings of the solver's diagnostics. Information (sizes, free modes of a frequency
/// step) gives none. `origins` leads the items the solver names back to the model's.
pub fn diagnose_solver_check(
    diagnostics: &[Diagnostic],
    mesh: &FeMesh,
    fe: &FeModel,
    origins: &Origins,
) -> Vec<Finding> {
    let mut findings = Vec::new();
    let parts_of = parts_of_nodes(mesh);
    let parts_named = |nodes: &[u32]| -> (Vec<usize>, String) {
        let parts: BTreeSet<usize> = (nodes.iter())
            .filter_map(|n| parts_of.get(n))
            .flatten()
            .copied()
            .collect();
        let parts: Vec<usize> = parts.into_iter().collect();
        let names: Vec<&str> = (parts.iter())
            .filter_map(|&p| mesh.parts.get(p).map(|p| p.name.as_str()))
            .collect();
        let what = if names.is_empty() {
            format!("{} node(s)", nodes.len())
        } else if names.len() > 5 {
            format!("{} and {} more", names[..5].join(", "), names.len() - 5)
        } else {
            names.join(", ")
        };
        (parts, what)
    };
    // The input holds the active steps only, in order.
    let active: Vec<usize> = (fe.steps.iter().enumerate())
        .filter(|(_, s)| s.active)
        .map(|(i, _)| i)
        .collect();
    let has_contact = !fe.contact_pairs.is_empty() || !fe.contact_regions.is_empty();
    let space = fe.properties.space;
    for diagnostic in diagnostics {
        if diagnostic.severity == SolverSeverity::Info {
            continue;
        }
        match &diagnostic.code {
            DiagnosticCode::RigidBodyMotion | DiagnosticCode::Mechanism => {
                // The step of the model the solver's step came from; without the table
                // (another input file) the active steps in order.
                let step = (diagnostic.step).and_then(|s| {
                    let index = (s as usize).checked_sub(1)?;
                    match origins.get(&ItemRef::Step(index)) {
                        Some(ModelItem::Step(step)) => Some(step),
                        _ if origins.is_empty() => active.get(index).copied(),
                        _ => None,
                    }
                });
                let problem = if has_contact && diagnostic.severity == SolverSeverity::Warning {
                    Problem::HeldOnlyByContact
                } else if diagnostic.code == DiagnosticCode::Mechanism {
                    Problem::Mechanism
                } else {
                    Problem::RigidBodyMotion
                };
                let (parts, what) = parts_named(&diagnostic.nodes);
                let how = if diagnostic.code == DiagnosticCode::Mechanism {
                    format!("{} free motion(s)", diagnostic.motions.len())
                } else {
                    describe_motions(space, &free_motions(space, &diagnostic.motions))
                };
                let step_name = step.map_or_else(
                    || diagnostic.step_name.clone().unwrap_or_default(),
                    |s| fe.steps[s].name.clone(),
                );
                let detail = format!(
                    "{step_name}: {what}: {how} (found by the solver, nodes {})",
                    sample(&diagnostic.nodes)
                );
                findings.push(Finding::new(
                    step.map_or(ModelItem::Analysis, ModelItem::BoundaryConditions),
                    problem,
                    detail.clone(),
                ));
                for part in parts {
                    findings.push(Finding::new(ModelItem::Part(part), problem, detail.clone()));
                }
            }
            DiagnosticCode::TieNodesNotTied { .. } => {
                let (parts, what) = parts_named(&diagnostic.nodes);
                let detail = format!(
                    "{} slave node(s) of {what} not tied: nodes {}",
                    diagnostic.nodes.len(),
                    sample(&diagnostic.nodes)
                );
                findings.push(Finding::new(
                    ModelItem::Analysis,
                    Problem::TieNodesNotTied,
                    detail.clone(),
                ));
                if let Some(item) = (diagnostic.item.as_ref()).and_then(|i| origins.get(i)) {
                    findings.push(Finding::new(item, Problem::TieNodesNotTied, detail.clone()));
                }
                for part in parts {
                    findings.push(Finding::new(
                        ModelItem::Part(part),
                        Problem::TieNodesNotTied,
                        detail.clone(),
                    ));
                }
            }
            DiagnosticCode::Input | DiagnosticCode::Unsupported | DiagnosticCode::Solver => {
                findings.push(Finding::new(
                    ModelItem::Analysis,
                    Problem::SolverError,
                    diagnostic.message.clone(),
                ));
                // The item the solver names: an element by its part, the rest by the
                // table of the lowering.
                let item = match &diagnostic.item {
                    Some(ItemRef::Element(id)) => (mesh.parts.iter())
                        .position(|p| p.elements.contains(id))
                        .map(ModelItem::Part),
                    Some(item) => origins.get(item),
                    None => None,
                };
                if let Some(item) = item.filter(|&i| i != ModelItem::Analysis) {
                    findings.push(Finding::new(
                        item,
                        Problem::SolverError,
                        diagnostic.message.clone(),
                    ));
                }
            }
            DiagnosticCode::NearlySingular | DiagnosticCode::FactorSize { .. } => {}
        }
    }
    findings
}

/// Parts by index that contain each node.
fn parts_of_nodes(mesh: &FeMesh) -> HashMap<u32, Vec<usize>> {
    let mut parts: HashMap<u32, Vec<usize>> = HashMap::new();
    for (p, part) in mesh.parts.iter().enumerate() {
        for element in part.elements.iter().filter_map(|&e| mesh.element(e)) {
            for &node in &element.nodes {
                let of = parts.entry(node).or_default();
                if of.last() != Some(&p) {
                    of.push(p);
                }
            }
        }
    }
    parts
}

fn sample(nodes: &[u32]) -> String {
    let shown: Vec<String> = nodes.iter().take(5).map(u32::to_string).collect();
    if nodes.len() > 5 {
        format!("{}, ...", shown.join(", "))
    } else {
        shown.join(", ")
    }
}

/// The solver's free motions (translation x, y, z, rotation x, y, z) as orthonormal vectors
/// over the rigid body unknowns of the model space, the way [`describe_motions`] reads them:
/// 3D all six, plane X, Y and rotation about Z, axisymmetric the axial displacement.
fn free_motions(space: ModelSpace, motions: &[[f64; 6]]) -> Vec<[f64; 6]> {
    let unknowns: &[usize] = match space {
        ModelSpace::ThreeD => &[0, 1, 2, 3, 4, 5],
        ModelSpace::Axisymmetric => &[1],
        _ => &[0, 1, 5],
    };
    let mut basis: Vec<[f64; 6]> = Vec::new();
    for motion in motions {
        let mut v = [0.0; 6];
        for (i, &u) in unknowns.iter().enumerate() {
            v[i] = motion[u];
        }
        // Translations and rotations have different units: weigh them alike.
        let norm = |v: &[f64; 6]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
        let n = norm(&v);
        if n == 0.0 {
            continue;
        }
        v.iter_mut().for_each(|x| *x /= n);
        for b in &basis {
            let dot: f64 = (0..6).map(|i| v[i] * b[i]).sum();
            (0..6).for_each(|i| v[i] -= dot * b[i]);
        }
        let n = norm(&v);
        if n > 1e-6 {
            v.iter_mut().for_each(|x| *x /= n);
            basis.push(v);
        }
    }
    basis
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn six_free_motions_are_completely_free() {
        let motions: Vec<[f64; 6]> = (0..6)
            .map(|i| std::array::from_fn(|j| if i == j { 1.0 } else { 0.0 }))
            .collect();
        let free = free_motions(ModelSpace::ThreeD, &motions);
        assert_eq!(free.len(), 6);
        assert_eq!(
            describe_motions(ModelSpace::ThreeD, &free),
            "completely free, nothing holds them"
        );
    }

    #[test]
    fn a_turn_about_z_is_named() {
        let free = free_motions(ModelSpace::ThreeD, &[[0.2, -0.1, 0.0, 0.0, 0.0, 1.0]]);
        assert_eq!(
            describe_motions(ModelSpace::ThreeD, &free),
            "rotation about Z free"
        );
    }

    /// The items the solver names lead to the model's through the lowering's table: an
    /// untied tie to the contact region it stands for, a step to the model's step of that
    /// analysis step, which skips deactivated ones.
    #[test]
    fn the_solver_items_lead_back_to_the_tree() {
        let mut fe = FeModel::default();
        for name in ["Step-1", "Off", "Step-3"] {
            let mut step = crate::Step::new_static(name);
            step.active = name != "Off";
            fe.steps.push(step);
        }
        let mut origins = Origins::default();
        origins.insert(ItemRef::Tie("GLUE".into()), ModelItem::ContactRegion(4));
        origins.insert(ItemRef::Step(0), ModelItem::Step(0));
        origins.insert(ItemRef::Step(1), ModelItem::Step(2));
        let untied = Diagnostic::new(
            DiagnosticCode::TieNodesNotTied { tie: "GLUE".into() },
            SolverSeverity::Warning,
            "not tied",
        )
        .concerning(ItemRef::Tie("GLUE".into()));
        let mut free = Diagnostic::new(
            DiagnosticCode::RigidBodyMotion,
            SolverSeverity::Error,
            "free",
        );
        free.step = Some(2);
        free.motions = vec![[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]];
        let findings = diagnose_solver_check(&[untied, free], &FeMesh::default(), &fe, &origins);
        let items: Vec<ModelItem> = findings.iter().map(|f| f.item).collect();
        assert!(items.contains(&ModelItem::ContactRegion(4)), "{items:?}");
        assert!(
            items.contains(&ModelItem::BoundaryConditions(2)),
            "{items:?}"
        );
    }
}
