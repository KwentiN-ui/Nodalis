//! Self test of the solver from the settings: checks that it runs and computes correct
//! results.
//!
//! The test solves small cantilevers through the same path as a real analysis (model, input
//! file writer, run, results), comparing with beam theory.

use std::path::Path;

use plx_io::frd::FrdImport;
use plx_mesh::{Element, ElementId, ElementShape, FeMesh, NodeId, Part};
use plx_model::{
    BoundaryCondition, BoundaryKind, Elastic, FeModel, LinearSolverChoice, Load, LoadKind,
    Material, Region, Section, SectionKind, Step,
};
use plx_solve::{JobStatus, Run};

pub(crate) const LENGTH: f64 = 100.0;
const WIDTH: f64 = 10.0;
const HEIGHT: f64 = 10.0;
const YOUNG: f64 = 210_000.0;
const POISSON: f64 = 0.3;
/// Total tip force in -z.
const TIP_FORCE: f64 = 100.0;
const PRESSURE: f64 = 2.0;

/// Outcome of one check.
#[derive(Clone, Debug, PartialEq)]
pub struct CheckResult {
    pub name: &'static str,
    pub passed: bool,
    pub message: String,
}

/// Outcome of the self test.
#[derive(Clone, Debug, PartialEq)]
pub struct Report {
    pub checks: Vec<CheckResult>,
}

/// What the self test is doing, for the window shown while it runs.
#[derive(Clone, Debug, Default)]
pub struct Progress {
    /// The running step, e.g. "Solving cantilever with pressure".
    pub step: String,
    /// How far the step has come, if it has countable parts: (done, total).
    pub count: Option<(usize, usize)>,
}

/// A [`Progress`] the test thread updates and the window reads.
pub type SharedProgress = std::sync::Arc<std::sync::Mutex<Progress>>;

fn set_step(progress: &SharedProgress, step: &str, count: Option<(usize, usize)>) {
    if let Ok(mut p) = progress.lock() {
        p.step = step.to_string();
        p.count = count;
    }
}

/// Runs all checks one after another on a pool of `threads` threads (0: all cores).
pub fn run(threads: u32, work_dir: &Path, progress: &SharedProgress) -> Report {
    set_step(progress, "Starting the solver", None);
    let mut checks = vec![check(
        "Start solver",
        true,
        format!("Nodalis solver {}", plx_solve::VERSION),
    )];
    let dir = work_dir.join("SelfTest");
    set_step(progress, "Solving cantilever with end load", None);
    checks.push(tip_force(threads, &dir, LinearSolverChoice::Direct));
    set_step(progress, "Solving cantilever with pressure", None);
    checks.push(pressure(threads, &dir));
    set_step(
        progress,
        "Solving cantilever with the iterative solver",
        None,
    );
    checks.push(tip_force(threads, &dir, LinearSolverChoice::Iterative));
    let _ = std::fs::remove_dir_all(&dir);
    Report { checks }
}

fn check(name: &'static str, passed: bool, message: String) -> CheckResult {
    CheckResult {
        name,
        passed,
        message,
    }
}

/// Cantilever with a surface traction on the free end: the tip deflection must match
/// Timoshenko beam theory and the reactions must balance the load.
fn tip_force(threads: u32, dir: &Path, equations: LinearSolverChoice) -> CheckResult {
    let (name, job) = match equations {
        LinearSolverChoice::Direct => ("Cantilever with end load", "Cantilever"),
        LinearSolverChoice::Iterative => ("Iterative equation solver", "Iterative"),
    };
    let mesh = beam_mesh();
    let tip = faces_where(&mesh, |p| p[0] == LENGTH);
    let mut model = beam_model(
        &mesh,
        LoadKind::force([0.0, 0.0, -TIP_FORCE]),
        Region::Faces(tip),
    );
    for step in &mut model.steps {
        if let plx_model::StepKind::Static(settings) = &mut step.kind {
            settings.solver = equations;
        }
    }
    let frd = match solve(threads, dir, job, &mesh, &model) {
        Ok(frd) => frd,
        Err(message) => return check(name, false, message),
    };
    let tip_nodes = nodes_where(&mesh, |p| p[0] == LENGTH);
    let deflection = -mean(&frd, "DISP", "U3", &tip_nodes);
    let inertia = WIDTH * HEIGHT.powi(3) / 12.0;
    let shear_modulus = YOUNG / (2.0 * (1.0 + POISSON));
    let expected = TIP_FORCE * LENGTH.powi(3) / (3.0 * YOUNG * inertia)
        + TIP_FORCE * LENGTH / (5.0 / 6.0 * shear_modulus * WIDTH * HEIGHT);
    let deviation = (deflection / expected - 1.0).abs();
    let reaction = reaction(&frd, &mesh, "F3");
    let balanced = (reaction - TIP_FORCE).abs() < 1e-3 * TIP_FORCE;
    let passed = deviation < 0.03 && balanced;
    check(
        name,
        passed,
        format!(
            "Deflection {deflection:.4} mm (beam theory {expected:.4} mm, deviation {:.1} %), reaction {reaction:.3} N (load {TIP_FORCE} N)",
            deviation * 100.0
        ),
    )
}

/// Pressure on the free end: the axial reactions must balance pressure times area. (Loads on
/// clamped nodes do not show up as reactions, so the end face is used, not the top.)
fn pressure(threads: u32, dir: &Path) -> CheckResult {
    const NAME: &str = "Cantilever with pressure";
    let mesh = beam_mesh();
    let tip = faces_where(&mesh, |p| p[0] == LENGTH);
    let model = beam_model(&mesh, LoadKind::Pressure(PRESSURE), Region::Faces(tip));
    let frd = match solve(threads, dir, "Pressure", &mesh, &model) {
        Ok(frd) => frd,
        Err(message) => return check(NAME, false, message),
    };
    let expected = PRESSURE * WIDTH * HEIGHT;
    let reaction = reaction(&frd, &mesh, "F1");
    let passed = (reaction - expected).abs() < 1e-3 * expected;
    check(
        NAME,
        passed,
        format!("Reaction {reaction:.3} N (pressure times area {expected} N)"),
    )
}

fn solve(
    threads: u32,
    dir: &Path,
    name: &str,
    mesh: &FeMesh,
    model: &FeModel,
) -> Result<FrdImport, String> {
    let input = plx_io::inp::write_inp(mesh, model, "Nodalis self-test")
        .map_err(|e| format!("Input file not written: {e}"))?;
    let mut job = Run::start(threads, dir, name, &input).map_err(|e| format!("Start: {e}"))?;
    let status = job.wait();
    let output = job.new_output();
    if status != JobStatus::Completed {
        let error = output
            .iter()
            .find(|l| l.contains("*ERROR"))
            .map_or_else(String::new, |l| format!(": {}", l.trim()));
        return Err(format!(
            "Analysis {}{error}",
            crate::analysis::Analysis::status_text(status)
        ));
    }
    let solved = job.solver_results().ok_or("no results")?;
    Ok(plx_io::solver::import(&solved))
}

/// Mean of a result component over nodes.
fn mean(frd: &FrdImport, field: &str, component: &str, nodes: &[NodeId]) -> f64 {
    let values = component_values(frd, field, component, nodes);
    values.iter().sum::<f64>() / values.len().max(1) as f64
}

/// Sum of the reaction forces at the clamped end.
fn reaction(frd: &FrdImport, mesh: &FeMesh, component: &str) -> f64 {
    let fixed = nodes_where(mesh, |p| p[0] == 0.0);
    component_values(frd, "FORC", component, &fixed)
        .iter()
        .sum()
}

fn component_values(frd: &FrdImport, field: &str, component: &str, nodes: &[NodeId]) -> Vec<f64> {
    let Some(values) = frd
        .increments
        .last()
        .and_then(|i| i.field(field))
        .and_then(|f| f.component(component))
        .map(|c| &c.values)
    else {
        return vec![f64::NAN];
    };
    nodes
        .iter()
        .map(|&node| {
            frd.mesh
                .node_index(node)
                .and_then(|i| values.get(i))
                .map_or(f64::NAN, |&v| f64::from(v))
        })
        .collect()
}

/// Solves the beam under a tip load in `dir`, with a history output
/// of the fixed nodes; returns its results file (`.ndlr`).
#[cfg(test)]
pub(crate) fn solve_beam_built_in(dir: &Path) -> std::path::PathBuf {
    let mesh = beam_mesh();
    let tip = faces_where(&mesh, |p| p[0] == LENGTH);
    let mut fe = beam_model(&mesh, LoadKind::force([0.0, 0.0, -1.0]), Region::Faces(tip));
    let fixed = (mesh.node_ids().iter().copied()).filter(|&n| mesh.node(n).unwrap()[0] == 0.0);
    let output = plx_model::HistoryOutput::node("H-Output-1", Region::Nodes(fixed.collect()));
    fe.steps[0].history_outputs.push(output);
    let input = plx_io::inp::write_inp(&mesh, &fe, "built-in").unwrap();
    let mut run = Run::start(1, dir, "Beam", &input).unwrap();
    assert_eq!(run.wait(), JobStatus::Completed, "{:?}", run.new_output());
    run.results().unwrap()
}

pub(crate) fn beam_model(mesh: &FeMesh, load: LoadKind, region: Region) -> FeModel {
    let mut step = Step::new_static("Step-1");
    step.boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Fixed-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(nodes_where(mesh, |p| p[0] == 0.0)),
        kind: BoundaryKind::displacement([Some(0.0); 6]),
        amplitude: None,
    });
    step.loads.push(Load {
        distribution: None,
        name: "Load-1".into(),
        active: true,
        visible: true,
        region,
        kind: load,
        amplitude: None,
        factor_amplitude: None,
    });
    FeModel {
        properties: Default::default(),
        materials: vec![Material {
            name: "Steel".into(),
            density: None,
            elastic: Some(Elastic {
                young: YOUNG,
                poisson: POISSON,
            }),
            ..Default::default()
        }],
        sections: vec![Section {
            name: "Section-1".into(),
            material: "Steel".into(),
            region: Region::Parts(vec!["BEAM".into()]),
            thickness: 1.0,
            kind: SectionKind::Solid,
        }],
        steps: vec![step],
        ..FeModel::default()
    }
}

/// The beam as 10 x 2 x 2 quadratic hexahedra (C3D20R), x along the length.
pub(crate) fn beam_mesh() -> FeMesh {
    const DIVISIONS: [u32; 3] = [10, 2, 2];
    let size = [LENGTH, WIDTH, HEIGHT];
    // Nodes on a lattice of half element size; the id encodes the lattice position.
    let points = DIVISIONS.map(|d| 2 * d + 1);
    let id = |p: [u32; 3]| 1 + p[0] + points[0] * (p[1] + points[1] * p[2]);
    let mut mesh = FeMesh::default();
    let add = |mesh: &mut FeMesh, p: [u32; 3]| {
        let coords = [0, 1, 2].map(|k| size[k] * f64::from(p[k]) / f64::from(points[k] - 1));
        mesh.set_node(id(p), coords);
        id(p)
    };
    // CalculiX's C3D20 node order relative to the element's corner, in half element sizes.
    const LOCAL: [[u32; 3]; 20] = [
        [0, 0, 0],
        [2, 0, 0],
        [2, 2, 0],
        [0, 2, 0],
        [0, 0, 2],
        [2, 0, 2],
        [2, 2, 2],
        [0, 2, 2],
        [1, 0, 0],
        [2, 1, 0],
        [1, 2, 0],
        [0, 1, 0],
        [1, 0, 2],
        [2, 1, 2],
        [1, 2, 2],
        [0, 1, 2],
        [0, 0, 1],
        [2, 0, 1],
        [2, 2, 1],
        [0, 2, 1],
    ];
    let mut elements = Vec::new();
    for k in 0..DIVISIONS[2] {
        for j in 0..DIVISIONS[1] {
            for i in 0..DIVISIONS[0] {
                let corner = [2 * i, 2 * j, 2 * k];
                let nodes = LOCAL
                    .iter()
                    .map(|l| add(&mut mesh, [0, 1, 2].map(|d| corner[d] + l[d])))
                    .collect();
                let element_id = elements.len() as ElementId + 1;
                mesh.add_element(Element {
                    id: element_id,
                    type_name: "C3D20R".into(),
                    shape: ElementShape::Hex20,
                    nodes,
                })
                .expect("20 nodes per element");
                elements.push(element_id);
            }
        }
    }
    mesh.parts.push(Part {
        name: "BEAM".into(),
        elements,
    });
    mesh
}

fn nodes_where(mesh: &FeMesh, on: impl Fn([f64; 3]) -> bool) -> Vec<NodeId> {
    mesh.node_ids()
        .iter()
        .zip(mesh.coords())
        .filter(|&(_, &p)| on(p))
        .map(|(&id, _)| id)
        .collect()
}

/// Element faces whose corners all lie where `on` holds.
pub(crate) fn faces_where(mesh: &FeMesh, on: impl Fn([f64; 3]) -> bool) -> Vec<(ElementId, u8)> {
    let mut faces = Vec::new();
    for element in mesh.elements() {
        for (index, face) in element.shape.faces().iter().enumerate() {
            let inside = face
                .corners
                .iter()
                .all(|&l| mesh.node(element.nodes[l]).is_some_and(&on));
            if inside {
                faces.push((element.id, index as u8 + 1));
            }
        }
    }
    faces
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn beam_mesh_is_a_closed_brick() {
        let mesh = beam_mesh();
        assert_eq!(mesh.element_count(), 40);
        assert!(mesh.missing_nodes().is_empty());
        // 21 x 5 x 5 lattice points minus face and body centres of each element.
        assert_eq!(
            mesh.node_count(),
            21 * 5 * 5 - 40 - (10 * 2 * 3 + 10 * 3 * 2 + 11 * 2 * 2)
        );
        assert_eq!(faces_where(&mesh, |p| p[0] == LENGTH).len(), 4);
        assert_eq!(faces_where(&mesh, |p| p[2] == HEIGHT).len(), 20);
    }

    #[test]
    fn the_solver_passes() {
        let dir = std::env::temp_dir().join(format!("plx-selftest-{}", std::process::id()));
        let report = run(1, &dir, &Default::default());
        let _ = std::fs::remove_dir_all(&dir);
        for result in &report.checks {
            eprintln!("{}: {}", result.name, result.message);
        }
        assert_eq!(report.checks.len(), 4, "{report:#?}");
        assert!(report.checks.iter().all(|r| r.passed), "{report:#?}");
    }
}
