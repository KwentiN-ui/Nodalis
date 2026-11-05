//! The findings of the solver's model check (Nodalis solver), attached to the tree: free
//! motions to the boundary conditions of their step and to the parts that move, untied
//! nodes to the analysis and their parts. The solver found them exactly; the geometric
//! estimates of [`crate::checks`] are only a quick hint while editing.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use plx_mesh::FeMesh;
use plxs_core::{
    Diagnostic, DiagnosticCode, Divergence, Failure, ItemRef, Severity as SolverSeverity, Stop,
};

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
            DiagnosticCode::Numerical(failure) => {
                let (problem, detail) = explain_failure(*failure, diagnostic, fe, has_contact);
                findings.push(Finding::new(ModelItem::Analysis, problem, detail.clone()));
                // The step it happened in, through the lowering's table.
                let step = (diagnostic.step).and_then(|s| {
                    let index = (s as usize).checked_sub(1)?;
                    match origins.get(&ItemRef::Step(index)) {
                        Some(ModelItem::Step(step)) => Some(step),
                        _ if origins.is_empty() => active.get(index).copied(),
                        _ => None,
                    }
                });
                if let Some(step) = step {
                    findings.push(Finding::new(ModelItem::Step(step), problem, detail));
                }
            }
            DiagnosticCode::Input
            | DiagnosticCode::Unsupported
            | DiagnosticCode::Files
            | DiagnosticCode::Solver => {
                let problem = match diagnostic.code {
                    DiagnosticCode::Input => Problem::SolverInput,
                    DiagnosticCode::Unsupported => Problem::SolverUnsupported,
                    DiagnosticCode::Files => Problem::SolverFiles,
                    _ => Problem::SolverError,
                };
                findings.push(Finding::new(
                    ModelItem::Analysis,
                    problem,
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
                    findings.push(Finding::new(item, problem, diagnostic.message.clone()));
                }
            }
            DiagnosticCode::NearlySingular | DiagnosticCode::FactorSize { .. } => {}
        }
    }
    findings
}

/// The problem a numerical failure of the solver stands for, and the detail: the solver's
/// message without its `*ERROR: ` and what it means for this model.
fn explain_failure(
    failure: Failure,
    diagnostic: &Diagnostic,
    fe: &FeModel,
    has_contact: bool,
) -> (Problem, String) {
    let message = diagnostic.message.trim();
    let message = message.strip_prefix("*ERROR:").unwrap_or(message).trim();
    let with = |meaning: &str| {
        if meaning.is_empty() {
            message.to_string()
        } else {
            format!("{message}. {meaning}")
        }
    };
    match failure {
        Failure::NoConvergence(c) => {
            let stop = match c.stop {
                Stop::Cutbacks => "smaller increments did not help either",
                Stop::MinimumIncrement => "the increment fell below the minimum",
                Stop::FixedIncrement => "the increment size is fixed (Incrementation Direct)",
            };
            if matches!(
                c.cause,
                Divergence::Temperatures | Divergence::ThermalContact
            ) {
                let what = if c.cause == Divergence::ThermalContact {
                    "The contact of the heat transfer step did not settle"
                } else {
                    "The temperatures did not settle"
                };
                return (Problem::HeatNoConvergence, with(&format!("{what}; {stop}")));
            }
            let plastic = fe.materials.iter().any(|m| m.plastic.is_some());
            let cause = match c.cause {
                Divergence::ResidualGrows => {
                    if plastic {
                        "The residual force grew: the load may exceed the strength of the \
                         structure (plastic collapse) or its buckling load"
                    } else {
                        "The residual force grew: the load may exceed the buckling load, or \
                         the structure snaps through"
                    }
                }
                Divergence::IterationLimit => "The iterations ran out before the forces balanced",
                Divergence::TooSlow => "The iterations converged too slowly to finish in time",
                Divergence::NotFinite => {
                    "The solution grew without bound: a part moves without resistance or \
                     a material constant is wrong"
                }
                Divergence::CreepTolerance => {
                    "The creep strain changes faster than the creep tolerance allows, even \
                     in small increments"
                }
                Divergence::Equations => "The equations of an iteration could not be solved",
                Divergence::Temperatures | Divergence::ThermalContact => unreachable!(),
            };
            if c.contact && c.contact_changing {
                (
                    Problem::ContactNoConvergence,
                    with(&format!(
                        "The contact status was still changing in the last iterations; {stop}"
                    )),
                )
            } else if c.contact || has_contact {
                (
                    Problem::NoConvergence,
                    with(&format!(
                        "{cause}; {stop}. The contact status had settled, so the cause is \
                         more likely the load, the material or the supports than the contact"
                    )),
                )
            } else {
                (Problem::NoConvergence, with(&format!("{cause}; {stop}")))
            }
        }
        Failure::IncrementLimit => (Problem::IncrementLimit, with("")),
        Failure::Singular => (
            Problem::RigidBodyMotion,
            with("A part or a degree of freedom is not held: the stiffness matrix is singular"),
        ),
        Failure::NotPositiveDefinite => (Problem::NotPositiveDefinite, with("")),
        Failure::OutOfMemory => (Problem::OutOfMemory, with("")),
        Failure::IterativeSolver => (Problem::IterativeSolverFailed, with("")),
        Failure::Eigenvalues => (Problem::EigenvaluesNotFound, with("")),
        Failure::Resonance => (Problem::UndampedResonance, with("")),
        Failure::Internal => (Problem::SolverError, with("")),
    }
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

    fn failed(failure: Failure, step: Option<u32>) -> Diagnostic {
        let mut d = Diagnostic::new(
            DiagnosticCode::Numerical(failure),
            SolverSeverity::Error,
            "*ERROR: no convergence: no convergence after 16 iterations; more than 5 cutbacks \
             in one increment (step 1, increment 1)",
        );
        d.step = step;
        d
    }

    fn convergence(contact: bool, contact_changing: bool) -> Failure {
        Failure::NoConvergence(plxs_core::Convergence {
            cause: Divergence::IterationLimit,
            stop: Stop::Cutbacks,
            contact,
            contact_changing,
        })
    }

    /// A failed increment whose contact status still changed is a contact problem, at the
    /// analysis and at its step; with a settled contact it is the general one, and the
    /// detail says why the contact is not the likely cause.
    #[test]
    fn a_failed_increment_is_explained_by_its_contact_status() {
        let mut fe = FeModel::default();
        fe.steps.push(crate::Step::new_static("Step-1"));
        let mut origins = Origins::default();
        origins.insert(ItemRef::Step(0), ModelItem::Step(0));
        let mesh = FeMesh::default();
        let found = diagnose_solver_check(
            &[failed(convergence(true, true), Some(1))],
            &mesh,
            &fe,
            &origins,
        );
        let items: Vec<(ModelItem, Problem)> = found.iter().map(|f| (f.item, f.problem)).collect();
        assert_eq!(
            items,
            vec![
                (ModelItem::Analysis, Problem::ContactNoConvergence),
                (ModelItem::Step(0), Problem::ContactNoConvergence),
            ]
        );
        assert!(
            found[0]
                .detail
                .starts_with("no convergence: no convergence after 16"),
            "{}",
            found[0].detail
        );
        assert!(
            found[0].detail.contains("still changing"),
            "{}",
            found[0].detail
        );

        let found = diagnose_solver_check(
            &[failed(convergence(true, false), Some(1))],
            &mesh,
            &fe,
            &origins,
        );
        assert_eq!(found[0].problem, Problem::NoConvergence);
        assert!(
            found[0].detail.contains("had settled"),
            "{}",
            found[0].detail
        );
    }

    /// Every kind of failure has its own explanation; only a failure inside the solver is
    /// the general solver error.
    #[test]
    fn every_failure_has_its_own_problem() {
        let fe = FeModel::default();
        let mesh = FeMesh::default();
        let origins = Origins::default();
        let problem = |failure: Failure| {
            diagnose_solver_check(&[failed(failure, None)], &mesh, &fe, &origins)[0].problem
        };
        for (failure, expected) in [
            (convergence(false, false), Problem::NoConvergence),
            (Failure::IncrementLimit, Problem::IncrementLimit),
            (Failure::Singular, Problem::RigidBodyMotion),
            (Failure::NotPositiveDefinite, Problem::NotPositiveDefinite),
            (Failure::OutOfMemory, Problem::OutOfMemory),
            (Failure::IterativeSolver, Problem::IterativeSolverFailed),
            (Failure::Eigenvalues, Problem::EigenvaluesNotFound),
            (Failure::Resonance, Problem::UndampedResonance),
            (Failure::Internal, Problem::SolverError),
            (
                Failure::NoConvergence(plxs_core::Convergence {
                    cause: Divergence::Temperatures,
                    stop: Stop::MinimumIncrement,
                    contact: false,
                    contact_changing: false,
                }),
                Problem::HeatNoConvergence,
            ),
        ] {
            assert_eq!(problem(failure), expected, "{failure:?}");
        }
        for (code, expected) in [
            (DiagnosticCode::Input, Problem::SolverInput),
            (DiagnosticCode::Unsupported, Problem::SolverUnsupported),
            (DiagnosticCode::Files, Problem::SolverFiles),
        ] {
            let d = Diagnostic::new(code, SolverSeverity::Error, "*ERROR: x");
            let found = diagnose_solver_check(&[d], &mesh, &fe, &origins);
            assert_eq!(found[0].problem, expected);
        }
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
