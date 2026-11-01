//! The model check of the solver: everything that can be found before computing, as typed
//! [`Diagnostic`]s. PrePoLix runs it for "Check Model" and when a run fails; it reads the
//! model like a run, prepares the session and factorizes the stiffness matrix of every
//! structural step with a distinct set of supports once, which finds every free motion
//! exactly (no geometric guess) and names the nodes that move.
//!
//! The rank test: LDLᵀ without pivoting leaves a pivot of (numerically) zero for each
//! direction the supports do not hold; dynamic regularization lets the factorization pass
//! it. Solving with the regularized factor for a unit load on such an unknown returns the
//! free motion, scaled by the inverse of the tiny pivot, so it stands out from everything
//! held. A rigid body motion fitted to it tells a part that moves as a whole (rigid body
//! motion) from a structure that deforms without resistance (mechanism). Textbook material
//! (e.g. Felippa, "Introduction to Finite Element Methods", on rank deficiency); no
//! CalculiX code was consulted.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;
use std::time::Instant;

use plxs_assembly::{Spc, assemble_stiffness};
use plxs_core::step::Procedure;
use plxs_core::{
    Analysis, Diagnostic, DiagnosticCode, NodeId, Progress, ProgressEvent, Severity, Step,
};
use plxs_linalg::{LdltSolver, LinearSolver, SparseMatrix};

use crate::{JobOptions, Session, SessionOptions};

/// A pivot smaller than this fraction of its unknown's diagonal entry counts as zero: the
/// unknown has no stiffness of its own once the others are eliminated. Round-off leaves
/// about 1e-16 to 1e-12 for a true zero; held but slender structures stay far above.
const ZERO_PIVOT: f64 = 1e-10;

/// Singular directions followed per step; a model with more has dozens of free parts,
/// which the first ones show well enough.
const MAX_DIRECTIONS: usize = 60;

/// Share of the largest movement above which a node counts as moving with a free motion.
const MOVING: f64 = 1e-3;

/// A free motion whose rigid body fit leaves more than this share unexplained deforms the
/// structure: a mechanism.
const RIGID_FIT: f64 = 0.05;

/// What the check found.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Check {
    pub diagnostics: Vec<Diagnostic>,
    /// Equations of the largest step checked.
    pub equations: usize,
    pub seconds: f64,
}

impl Check {
    pub fn has_errors(&self) -> bool {
        self.diagnostics
            .iter()
            .any(|d| d.severity == Severity::Error)
    }
}

/// Reads an input text and checks it, see [`check_analysis`].
pub fn check_input(
    input: &str,
    base: Option<&Path>,
    options: &JobOptions,
    progress: &mut dyn Progress,
) -> Check {
    let started = Instant::now();
    let reading = match plxs_input::read_inp_str(input, base) {
        Ok(reading) => reading,
        Err(errors) => {
            return Check {
                diagnostics: errors.iter().map(Diagnostic::from_error).collect(),
                equations: 0,
                seconds: started.elapsed().as_secs_f64(),
            };
        }
    };
    let mut check = check_analysis(reading.analysis, options, progress);
    check.seconds = started.elapsed().as_secs_f64();
    check
}

/// Checks a parsed analysis: expansion, constraints (ties), and the rank of the stiffness
/// matrix of every structural step.
pub fn check_analysis(
    analysis: Analysis,
    options: &JobOptions,
    progress: &mut dyn Progress,
) -> Check {
    let started = Instant::now();
    let mut check = Check::default();
    let original_ids: Vec<NodeId> = analysis.model.nodes.ids.clone();
    let expansion = match plxs_elements::expand::expand(analysis) {
        Ok(expansion) => expansion,
        Err(error) => {
            check.diagnostics.push(Diagnostic::from_error(&error));
            return check;
        }
    };
    // Expanded nodes stand for the node of the shell or beam they came from.
    let mut original_of: HashMap<NodeId, NodeId> = HashMap::new();
    for (&node, groups) in &expansion.output.groups {
        for &expanded in groups.iter().flatten() {
            original_of.insert(expanded, node);
        }
    }
    for &id in &original_ids {
        original_of.insert(id, id);
    }
    let analysis = expansion.analysis;
    let has_contact = !analysis.model.contact.pairs.is_empty();
    let session_options = SessionOptions {
        threads: options.threads,
        pool: options.pool.clone(),
        work_dir: options.work_dir.clone(),
        cancel: options.cancel.clone(),
    };
    let session = match Session::new(analysis.model, session_options) {
        Ok(session) => session,
        Err(error) => {
            check.diagnostics.push(Diagnostic::from_error(&error));
            return check;
        }
    };
    // Nodes whose DOFs are displacements of the structure: not the rotation nodes of rigid
    // bodies (their "displacements" are angles) and not the helper nodes of the expansion.
    let mut structural: Vec<bool> = (session.model.nodes.ids.iter())
        .map(|id| original_of.contains_key(id))
        .collect();
    for body in &session.model.constraints.rigid_bodies {
        if let Some(i) = session.model.nodes.index(body.rotation_node) {
            structural[i as usize] = false;
        }
    }
    let ids = |dense: &[u32]| -> Vec<NodeId> {
        let mut out: Vec<NodeId> = dense
            .iter()
            .filter_map(|&i| original_of.get(&session.model.nodes.ids[i as usize]))
            .copied()
            .collect();
        out.sort_unstable();
        out.dedup();
        out
    };
    for (tie, missed) in &session.constraints.untied {
        let mut diagnostic = Diagnostic::new(
            DiagnosticCode::TieNodesNotTied { tie: tie.clone() },
            Severity::Warning,
            format!(
                "*TIE {tie}: {} slave node(s) are farther than the position tolerance from the master surface and are not tied",
                missed.len()
            ),
        );
        diagnostic.nodes = ids(missed);
        check.diagnostics.push(diagnostic);
    }

    // One rank test per distinct set of held DOFs; the stiffness is the same for all.
    let mut stiffness: Option<SparseMatrix> = None;
    let mut tested: Vec<(Vec<u32>, usize)> = Vec::new();
    for (index, step) in analysis.steps.iter().enumerate() {
        if options.cancel.is_cancelled() {
            break;
        }
        let number = index as u32 + 1;
        let severity = match &step.procedure {
            Procedure::Static(_)
            | Procedure::Visco(_)
            | Procedure::Buckle(_)
            | Procedure::Dynamic(_)
            | Procedure::CoupledTemperatureDisplacement(_)
            | Procedure::UncoupledTemperatureDisplacement(_)
            | Procedure::Sensitivity(_) => Severity::Error,
            // Free-free modes are legitimate: their frequency is zero.
            Procedure::Frequency(_) | Procedure::ComplexFrequency(_) => Severity::Info,
            Procedure::ModalDynamics(_) | Procedure::SteadyStateDynamics(_) => Severity::Warning,
            Procedure::NoAnalysis | Procedure::HeatTransfer(_) => continue,
        };
        // Contact may hold what the supports do not; the rank test sees no contact.
        let severity = if has_contact && severity == Severity::Error {
            Severity::Warning
        } else {
            severity
        };
        let spc = match Spc::of_step(&session.model, step, &session.table.dofs, 1.0, 1.0, 1.0) {
            Ok(spc) => spc,
            Err(error) => {
                check
                    .diagnostics
                    .push(at_step(Diagnostic::from_error(&error), step, number));
                continue;
            }
        };
        let mut held = spc.dofs.clone();
        held.sort_unstable();
        if let Some((_, earlier)) = tested.iter().find(|(h, _)| *h == held) {
            // Same supports as an earlier step: repeat its findings for this step, if the
            // severity differs only.
            let repeated: Vec<Diagnostic> = (check.diagnostics.iter())
                .filter(|d| d.step == Some(*earlier as u32) && is_rank(&d.code))
                .map(|d| {
                    let mut d = at_step(d.clone(), step, number);
                    d.severity = severity;
                    d
                })
                .collect();
            check.diagnostics.extend(repeated);
            continue;
        }
        tested.push((held, number as usize));
        if stiffness.is_none() {
            if let Err(error) = session.table.require_elastic() {
                check.diagnostics.push(Diagnostic::from_error(&error));
                break;
            }
            let mut k = SparseMatrix::zeros(session.pattern.clone());
            session
                .pool
                .install(|| assemble_stiffness(&session.table, &session.model, &mut k));
            stiffness = Some(k);
        }
        let k = stiffness.as_ref().unwrap();
        let found = session
            .pool
            .install(|| rank_test(&session, step, &spc.dofs, k));
        match found {
            Ok(test) => {
                check.equations = check.equations.max(test.equations);
                progress.event(ProgressEvent::Info(format!(
                    "model check of step {number}: {} equations, {} singular direction(s)",
                    test.equations,
                    test.directions.len()
                )));
                if index == 0 || check.equations == test.equations {
                    let mut size = Diagnostic::new(
                        DiagnosticCode::FactorSize {
                            equations: test.equations,
                            bytes: test.factor_bytes,
                        },
                        Severity::Info,
                        format!(
                            "{} equations, factorized stiffness matrix {:.0} MB",
                            test.equations,
                            test.factor_bytes as f64 / 1e6
                        ),
                    );
                    size.step = Some(number);
                    check
                        .diagnostics
                        .retain(|d| !matches!(d.code, DiagnosticCode::FactorSize { .. }));
                    check.diagnostics.push(size);
                }
                for group in group_directions(&test.directions, &structural) {
                    let mut diagnostic =
                        describe(&session, &test.directions, &group, severity, has_contact);
                    diagnostic.nodes = ids(&group.nodes);
                    check.diagnostics.push(at_step(diagnostic, step, number));
                }
            }
            Err(error) => {
                check
                    .diagnostics
                    .push(at_step(Diagnostic::from_error(&error), step, number));
            }
        }
    }
    check.seconds = started.elapsed().as_secs_f64();
    check
}

/// A diagnostic as one line of the job output, in the style of CalculiX's messages
/// (`*ERROR`, `*WARNING`, `*INFO`), with up to ten of its nodes.
pub fn format_diagnostic(diagnostic: &Diagnostic) -> String {
    let level = match diagnostic.severity {
        Severity::Error => "*ERROR",
        Severity::Warning => "*WARNING",
        Severity::Info => "*INFO",
    };
    let step = diagnostic
        .step
        .map_or_else(String::new, |s| format!(" (step {s})"));
    let message = diagnostic.message.trim_start_matches("*ERROR: ");
    let mut line = format!("{level} in model check{step}: {message}");
    if !diagnostic.nodes.is_empty() {
        let shown: Vec<String> = (diagnostic.nodes.iter().take(10))
            .map(|n| n.to_string())
            .collect();
        line += &format!(
            "; nodes {}{}",
            shown.join(", "),
            if diagnostic.nodes.len() > 10 {
                ", ..."
            } else {
                ""
            }
        );
    }
    line
}

fn is_rank(code: &DiagnosticCode) -> bool {
    matches!(
        code,
        DiagnosticCode::RigidBodyMotion | DiagnosticCode::Mechanism
    )
}

fn at_step(mut diagnostic: Diagnostic, step: &Step, number: u32) -> Diagnostic {
    diagnostic.step = Some(number);
    diagnostic.step_name = Some(step.name.clone());
    diagnostic
}

/// The singular directions of a step's stiffness matrix.
struct RankTest {
    equations: usize,
    factor_bytes: u64,
    /// Displacement (all DOFs of the session, per node 3) of each free motion, scaled to a
    /// largest component of 1.
    directions: Vec<Vec<f64>>,
}

fn rank_test(
    session: &Session,
    _step: &Step,
    held: &[u32],
    k: &SparseMatrix,
) -> plxs_core::Result<RankTest> {
    let active = session.table.active_dofs();
    let set = session
        .constraints
        .step_set(session.model.nodes.len(), &active, held);
    let k_red = set.reduce(k);
    let equations = set.reduced_len();
    let mut solver = LdltSolver::new(session.threads);
    solver.set_coordinates(Some(
        (set.free.iter())
            .map(|&dof| session.model.nodes.coordinates[dof as usize / 3])
            .collect(),
    ));
    solver.analyze(k_red.pattern())?;
    let factor_bytes = solver.factor_nonzeros().unwrap_or(0) as u64 * 8;
    // Zero pivots are replaced by a tiny one so that the factorization passes.
    solver.set_regularization(1e-14);
    solver.factorize(&k_red)?;
    let pivots = solver.pivots().unwrap_or_default();
    let diagonal = k_red.diagonal();
    let mut singular: Vec<(usize, f64)> = (0..equations)
        .filter_map(|j| {
            let scale = diagonal[j].abs().max(f64::MIN_POSITIVE);
            let ratio = pivots[j] / scale;
            (ratio < ZERO_PIVOT).then_some((j, ratio))
        })
        .collect();
    singular.sort_by(|a, b| a.1.total_cmp(&b.1));
    singular.truncate(MAX_DIRECTIONS);
    let zeros = vec![0.0; set_len(session)];
    let mut directions = Vec::with_capacity(singular.len());
    for (j, _) in singular {
        let mut v = vec![0.0; equations];
        v[j] = 1.0;
        solver.solve(&mut v)?;
        let mut u = set.expand(&v, &zeros);
        // Displacement DOFs only: per node 3.
        u.truncate(set_len(session));
        let largest = u.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
        if largest <= 0.0 || !largest.is_finite() {
            continue;
        }
        for x in &mut u {
            *x /= largest;
        }
        directions.push(u);
    }
    Ok(RankTest {
        equations,
        factor_bytes,
        directions,
    })
}

fn set_len(session: &Session) -> usize {
    session.model.nodes.len() * 3
}

/// Free motions that move the same nodes, with the nodes they move.
struct Group {
    nodes: Vec<u32>,
    directions: Vec<usize>,
}

fn moving_nodes(direction: &[f64], structural: &[bool]) -> Vec<u32> {
    (0..direction.len() / 3)
        .filter(|&n| structural[n])
        .filter(|&n| {
            let u = &direction[n * 3..n * 3 + 3];
            (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt() > MOVING
        })
        .map(|n| n as u32)
        .collect()
}

fn group_directions(directions: &[Vec<f64>], structural: &[bool]) -> Vec<Group> {
    // Directions that share a moving node belong together (union-find).
    let moving: Vec<Vec<u32>> = directions
        .iter()
        .map(|d| moving_nodes(d, structural))
        .collect();
    let mut parent: Vec<usize> = (0..directions.len()).collect();
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let mut owner: HashMap<u32, usize> = HashMap::new();
    for (d, nodes) in moving.iter().enumerate() {
        for &n in nodes {
            if let Some(&other) = owner.get(&n) {
                let (a, b) = (root(&mut parent, d), root(&mut parent, other));
                if a != b {
                    parent[a.max(b)] = a.min(b);
                }
            } else {
                owner.insert(n, d);
            }
        }
    }
    let mut groups: BTreeMap<usize, Group> = BTreeMap::new();
    for (d, nodes) in moving.into_iter().enumerate() {
        let r = root(&mut parent, d);
        let group = groups.entry(r).or_insert(Group {
            nodes: Vec::new(),
            directions: Vec::new(),
        });
        group.directions.push(d);
        group.nodes.extend(nodes);
    }
    groups
        .into_values()
        .filter(|g| !g.nodes.is_empty())
        .map(|mut g| {
            g.nodes.sort_unstable();
            g.nodes.dedup();
            g
        })
        .collect()
}

/// Least-squares rigid body motion `u = t + θ × (x − c)` of a displacement field over
/// `nodes`, and the share of the field it leaves unexplained.
fn rigid_fit(
    coordinates: &[[f64; 3]],
    direction: &[f64],
    nodes: &[u32],
    center: [f64; 3],
) -> ([f64; 6], f64) {
    // Normal equations A p = b with p = (t, θ); the rows per node are
    // [I, −[r]×] with r = x − c.
    let mut a = [[0.0_f64; 6]; 6];
    let mut b = [0.0_f64; 6];
    let mut total = 0.0;
    let rows = |r: [f64; 3]| -> [[f64; 6]; 3] {
        [
            [1.0, 0.0, 0.0, 0.0, r[2], -r[1]],
            [0.0, 1.0, 0.0, -r[2], 0.0, r[0]],
            [0.0, 0.0, 1.0, r[1], -r[0], 0.0],
        ]
    };
    for &n in nodes {
        let x = coordinates[n as usize];
        let r = [x[0] - center[0], x[1] - center[1], x[2] - center[2]];
        let m = rows(r);
        let u = &direction[n as usize * 3..n as usize * 3 + 3];
        for i in 0..3 {
            total += u[i] * u[i];
            for p in 0..6 {
                b[p] += m[i][p] * u[i];
                for q in 0..6 {
                    a[p][q] += m[i][p] * m[i][q];
                }
            }
        }
    }
    let p = solve6(a, b);
    let mut residual = 0.0;
    for &n in nodes {
        let x = coordinates[n as usize];
        let r = [x[0] - center[0], x[1] - center[1], x[2] - center[2]];
        let m = rows(r);
        let u = &direction[n as usize * 3..n as usize * 3 + 3];
        for i in 0..3 {
            let fit: f64 = (0..6).map(|q| m[i][q] * p[q]).sum();
            residual += (u[i] - fit).powi(2);
        }
    }
    let unexplained = if total > 0.0 {
        (residual / total).sqrt()
    } else {
        0.0
    };
    (p, unexplained)
}

/// Solves a symmetric positive semi-definite 6×6 system; directions without data (a group
/// on a line cannot fix the rotation about it) get zero.
fn solve6(mut a: [[f64; 6]; 6], mut b: [f64; 6]) -> [f64; 6] {
    let scale = (0..6)
        .map(|i| a[i][i])
        .fold(0.0_f64, f64::max)
        .max(f64::MIN_POSITIVE);
    let mut skip = [false; 6];
    for k in 0..6 {
        if a[k][k] <= 1e-12 * scale {
            skip[k] = true;
            continue;
        }
        let pivot_row = a[k];
        for i in k + 1..6 {
            let f = a[i][k] / pivot_row[k];
            for (aij, akj) in a[i].iter_mut().zip(pivot_row).skip(k) {
                *aij -= f * akj;
            }
            b[i] -= f * b[k];
        }
    }
    let mut x = [0.0; 6];
    for k in (0..6).rev() {
        if skip[k] {
            continue;
        }
        let s: f64 = (k + 1..6).map(|j| a[k][j] * x[j]).sum();
        x[k] = (b[k] - s) / a[k][k];
    }
    x
}

/// Names a group of free motions: rigid body motion or mechanism, and how it moves.
fn describe(
    session: &Session,
    directions: &[Vec<f64>],
    group: &Group,
    severity: Severity,
    has_contact: bool,
) -> Diagnostic {
    let coordinates = &session.model.nodes.coordinates;
    let count = group.nodes.len() as f64;
    let mut center = [0.0; 3];
    for &n in &group.nodes {
        for (c, x) in center.iter_mut().zip(coordinates[n as usize]) {
            *c += x / count;
        }
    }
    let mut motions = Vec::new();
    let mut worst = 0.0_f64;
    for &d in &group.directions {
        let (motion, unexplained) = rigid_fit(coordinates, &directions[d], &group.nodes, center);
        worst = worst.max(unexplained);
        motions.push(motion);
    }
    let rigid = worst <= RIGID_FIT;
    let code = if rigid {
        DiagnosticCode::RigidBodyMotion
    } else {
        DiagnosticCode::Mechanism
    };
    let what = if rigid {
        "can move as a rigid body"
    } else {
        "can move without resistance (mechanism)"
    };
    let mut message = format!(
        "{} node(s) {what}: {} free motion(s), the stiffness matrix is singular",
        group.nodes.len(),
        group.directions.len()
    );
    if has_contact {
        message.push_str("; unless the contact holds them");
    }
    let mut diagnostic = Diagnostic::new(code, severity, message);
    diagnostic.motions = motions;
    diagnostic.center = center;
    diagnostic
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_rigid_rotation_is_explained_and_a_bending_is_not() {
        let coordinates: Vec<[f64; 3]> = (0..10)
            .map(|i| [i as f64, 0.5 * (i % 2) as f64, 0.0])
            .collect();
        let nodes: Vec<u32> = (0..10).collect();
        let center = [4.5, 0.25, 0.0];
        // Rotation about z through the center: u = θ × r.
        let rotation: Vec<f64> = coordinates
            .iter()
            .flat_map(|x| [-(x[1] - center[1]), x[0] - center[0], 0.0])
            .collect();
        let (motion, unexplained) = rigid_fit(&coordinates, &rotation, &nodes, center);
        assert!(unexplained < 1e-12, "{unexplained}");
        assert!((motion[5] - 1.0).abs() < 1e-12, "{motion:?}");
        // A sine across the line: no rigid motion fits it.
        let bending: Vec<f64> = coordinates
            .iter()
            .flat_map(|x| [0.0, (x[0] * std::f64::consts::PI / 9.0).sin(), 0.0])
            .collect();
        let (_, unexplained) = rigid_fit(&coordinates, &bending, &nodes, center);
        assert!(unexplained > RIGID_FIT, "{unexplained}");
    }
}
