//! Self test of the solver from the settings: the built-in PrePoLix solver, or the CalculiX
//! executable, the shipped one or the user's own. Checks that the solver runs and computes
//! correct results.
//!
//! The test reads the version, solves small cantilevers through the same path as a real
//! analysis (model, input file writer, run, results reader), comparing with beam theory, and
//! for CalculiX checks that the executable knows every keyword PrePoLix writes.

use std::collections::BTreeMap;
use std::path::Path;
use std::process::Command;

use plx_io::frd::{FrdImport, read_frd};
use plx_job::{Job, JobStatus};
use plx_mesh::{Element, ElementId, ElementShape, FeMesh, NodeId, Part};
use plx_model::{
    BoundaryCondition, BoundaryKind, Elastic, FeModel, LinearSolverChoice, Load, LoadKind,
    Material, Region, Section, SectionKind, Step,
};
use plx_solve::{Engine, Run};

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
    /// Result of [`known_keywords`] for all keywords PrePoLix writes, if CalculiX started.
    pub keywords: Option<BTreeMap<String, bool>>,
    /// What [`version_text`] read, if CalculiX started.
    pub version: Option<String>,
}

/// What the self test is doing, for the window shown while it runs.
#[derive(Clone, Debug, Default)]
pub struct Progress {
    /// The running step, e.g. "Testing keywords".
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

/// Runs all checks one after another; stops after the version check if CalculiX does not
/// start at all. The built-in solver has no executable and no keyword probe: it solves the
/// cantilevers only.
pub fn run(
    engine: Engine,
    solver: &plx_job::Solver,
    work_dir: &Path,
    progress: &SharedProgress,
) -> Report {
    set_step(progress, "Starting the solver", None);
    let version = match engine {
        Engine::CalculiX => version(&solver.executable),
        Engine::PrePoLix => check(
            "Start solver",
            true,
            format!("PrePoLix solver {}, built in", plx_solve::VERSION),
        ),
    };
    let started = version.passed;
    let mut checks = vec![version];
    let mut keywords = None;
    if started {
        let dir = work_dir.join("SelfTest");
        set_step(progress, "Solving cantilever with end load", None);
        checks.push(tip_force(engine, solver, &dir, LinearSolverChoice::Direct));
        set_step(progress, "Solving cantilever with pressure", None);
        checks.push(pressure(engine, solver, &dir));
        set_step(
            progress,
            "Solving cantilever with the iterative solver",
            None,
        );
        checks.push(tip_force(
            engine,
            solver,
            &dir,
            LinearSolverChoice::Iterative,
        ));
        let _ = std::fs::remove_dir_all(&dir);
    }
    if started && engine == Engine::CalculiX {
        let written = (plx_io::inp::keywords::WRITTEN_KEYWORDS.iter())
            .map(|(k, _)| k.to_string())
            .collect::<Vec<_>>();
        let on_keyword = |done, total| set_step(progress, "Checking keywords", Some((done, total)));
        keywords = known_keywords(solver, work_dir, &written, &on_keyword);
        checks.push(keyword_check(keywords.as_ref(), written.len()));
    }
    Report {
        checks,
        keywords,
        version: match engine {
            Engine::CalculiX => started.then(|| version_text(&solver.executable)).flatten(),
            Engine::PrePoLix => Some(plx_solve::VERSION.to_owned()),
        },
    }
}

/// Lists the features whose keywords the executable does not know.
fn keyword_check(known: Option<&BTreeMap<String, bool>>, count: usize) -> CheckResult {
    const NAME: &str = "Keywords";
    let Some(known) = known.filter(|k| !k.is_empty()) else {
        return check(
            NAME,
            false,
            "Not checked: CalculiX did not read the test input".into(),
        );
    };
    let unknown = unknown_of(
        known,
        plx_io::inp::keywords::WRITTEN_KEYWORDS
            .iter()
            .map(|(k, _)| *k),
    );
    if unknown.is_empty() {
        return check(
            NAME,
            true,
            format!("All {count} keywords PrePoLix writes are known"),
        );
    }
    check(
        NAME,
        false,
        format!(
            "Unknown to this CalculiX, so these features cannot be computed:\n{}",
            by_feature(&unknown).join("\n")
        ),
    )
}

/// The keywords that `known` marks as unknown.
pub fn unknown_of<'a>(
    known: &BTreeMap<String, bool>,
    keywords: impl IntoIterator<Item = &'a str>,
) -> Vec<String> {
    use plx_io::inp::keywords::normalized;
    (keywords.into_iter())
        .filter(|k| known.get(&normalized(k)) == Some(&false))
        .map(str::to_string)
        .collect()
}

/// One entry per feature, "Feature (*KEYWORD, *KEYWORD)", for a list of keywords; keywords
/// of no known feature stand alone.
pub fn by_feature(keywords: &[String]) -> Vec<String> {
    let mut groups: Vec<(&str, Vec<&str>)> = Vec::new();
    for keyword in keywords {
        let feature = plx_io::inp::keywords::feature(keyword).unwrap_or("");
        match groups.iter_mut().find(|(f, _)| *f == feature) {
            Some((_, list)) => list.push(keyword),
            None => groups.push((feature, vec![keyword])),
        }
    }
    let groups = groups.into_iter().map(|(feature, list)| match feature {
        "" => list.join(", "),
        feature => format!("{feature} ({})", list.join(", ")),
    });
    groups.collect()
}

/// Whether the executable knows each keyword, by [`plx_io::inp::keywords::normalized`] name.
/// CalculiX cannot list its keywords, so each one is read alone from an input file: an unknown
/// keyword gives the warning "Card image cannot be interpreted", a known one another message
/// (it is out of place there) or none. A keyword whose test does not run is left out; `None`
/// if CalculiX does not start. A few keywords crash CalculiX when they stand alone and get the
/// context they need first, see [`keyword_test_input`].
pub fn known_keywords(
    solver: &plx_job::Solver,
    work_dir: &Path,
    keywords: &[String],
    progress: &dyn Fn(usize, usize),
) -> Option<BTreeMap<String, bool>> {
    let dir = work_dir.join("Keywordtest");
    let mut known = BTreeMap::new();
    for (index, keyword) in keywords.iter().enumerate() {
        progress(index, keywords.len());
        let Ok(mut job) = Job::start(solver, &dir, "Keyword", &keyword_test_input(keyword)) else {
            let _ = std::fs::remove_dir_all(&dir);
            return None;
        };
        let started = std::time::Instant::now();
        while job.poll() == JobStatus::Running {
            if started.elapsed() > std::time::Duration::from_secs(10) {
                job.kill();
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        let output = job.new_output();
        // Without the banner something else ran or CalculiX crashed: no verdict.
        if job.poll() == JobStatus::Killed || !output.iter().any(|l| l.contains("CalculiX")) {
            continue;
        }
        let unknown = output.iter().any(|l| l.contains("cannot be interpreted"));
        known.insert(plx_io::inp::keywords::normalized(keyword), !unknown);
    }
    let _ = std::fs::remove_dir_all(&dir);
    Some(known)
}

/// The input file that tests `keyword`: the keyword alone, or, where CalculiX 2.23 crashes on
/// that, after the model it reads. `*CYCLIC SYMMETRY MODEL` needs nodes and a cyclic tie; with
/// them CalculiX stops with an error instead of a segmentation fault (on Windows a crash
/// report window).
fn keyword_test_input(keyword: &str) -> String {
    use plx_io::inp::keywords::normalized;
    if normalized(keyword) == normalized("*CYCLIC SYMMETRY MODEL") {
        format!("{CYCLIC_SYMMETRY_MODEL}{keyword}, N=4, TIE=T\n0, 0, 0, 0, 0, 1\n")
    } else {
        format!("{keyword}\n")
    }
}

/// One element with the two surfaces and the cyclic tie `*CYCLIC SYMMETRY MODEL` refers to.
const CYCLIC_SYMMETRY_MODEL: &str = "*Node
1, 0, 0, 0
2, 1, 0, 0
3, 1, 1, 0
4, 0, 1, 0
5, 0, 0, 1
6, 1, 0, 1
7, 1, 1, 1
8, 0, 1, 1
*Element, Type=C3D8, Elset=E
1, 1, 2, 3, 4, 5, 6, 7, 8
*Surface, Name=A, Type=Node
1
*Surface, Name=B, Type=Node
2
*Tie, Name=T, Cyclic symmetry
A, B
";

fn check(name: &'static str, passed: bool, message: String) -> CheckResult {
    CheckResult {
        name,
        passed,
        message,
    }
}

/// The version `ccx -v` prints, e.g. "2.21".
pub fn version_text(executable: &Path) -> Option<String> {
    let output = plx_job::hide_console(&mut Command::new(executable))
        .arg("-v")
        .output()
        .ok()?;
    let text = String::from_utf8_lossy(&output.stdout);
    text.lines()
        .find_map(|l| l.trim().strip_prefix("This is Version"))
        .map(|v| v.trim().to_string())
}

/// `ccx -v` prints e.g. "This is Version 2.21".
fn version(executable: &Path) -> CheckResult {
    const NAME: &str = "Start executable";
    if let Some(message) = crate::settings::missing_executable(executable) {
        return check(NAME, false, message);
    }
    match plx_job::hide_console(&mut Command::new(executable))
        .arg("-v")
        .output()
    {
        Ok(output) => {
            let text = String::from_utf8_lossy(&output.stdout);
            match text
                .lines()
                .find_map(|l| l.trim().strip_prefix("This is Version"))
            {
                Some(version) => check(NAME, true, format!("CalculiX {}", version.trim())),
                None => check(
                    NAME,
                    false,
                    format!(
                        "{} starts but reports no CalculiX version",
                        executable.display()
                    ),
                ),
            }
        }
        Err(error) => check(
            NAME,
            false,
            format!("{} not started: {error}", executable.display()),
        ),
    }
}

/// Cantilever with a surface traction on the free end: the tip deflection must match
/// Timoshenko beam theory and the reactions must balance the load.
fn tip_force(
    engine: Engine,
    solver: &plx_job::Solver,
    dir: &Path,
    equations: LinearSolverChoice,
) -> CheckResult {
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
    let frd = match solve(engine, solver, dir, job, &mesh, &model) {
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
fn pressure(engine: Engine, solver: &plx_job::Solver, dir: &Path) -> CheckResult {
    const NAME: &str = "Cantilever with pressure";
    let mesh = beam_mesh();
    let tip = faces_where(&mesh, |p| p[0] == LENGTH);
    let model = beam_model(&mesh, LoadKind::Pressure(PRESSURE), Region::Faces(tip));
    let frd = match solve(engine, solver, dir, "Pressure", &mesh, &model) {
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
    engine: Engine,
    solver: &plx_job::Solver,
    dir: &Path,
    name: &str,
    mesh: &FeMesh,
    model: &FeModel,
) -> Result<FrdImport, String> {
    let input = plx_io::inp::write_inp(mesh, model, "PrePoLix self-test")
        .map_err(|e| format!("Input file not written: {e}"))?;
    let mut job =
        Run::start(engine, solver, dir, name, &input).map_err(|e| format!("Start: {e}"))?;
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
    let path = job.results().ok_or("no results file written")?;
    read_frd(&path).map_err(|e| format!("Results not readable: {e}"))
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
    fn missing_program_fails_the_first_check_only() {
        let solver = plx_job::Solver {
            executable: "plx-does-not-exist".into(),
            threads: 1,
        };
        let report = run(
            Engine::CalculiX,
            &solver,
            &std::env::temp_dir(),
            &Default::default(),
        );
        assert_eq!(report.checks.len(), 1);
        assert!(!report.checks[0].passed);
        assert_eq!(report.keywords, None);
    }

    #[test]
    fn installed_calculix_passes() {
        if Command::new("ccx").arg("-v").output().is_err() {
            eprintln!("ccx not found, test skipped");
            return;
        }
        let dir = std::env::temp_dir().join(format!("plx-selftest-{}", std::process::id()));
        let report = run(
            Engine::CalculiX,
            &plx_job::Solver::default(),
            &dir,
            &Default::default(),
        );
        let _ = std::fs::remove_dir_all(&dir);
        for result in &report.checks {
            eprintln!("{}: {}", result.name, result.message);
        }
        assert_eq!(report.checks.len(), 5);
        assert!(report.checks.iter().all(|r| r.passed), "{report:#?}");
    }

    /// The self test with the built-in solver needs no program and runs everywhere.
    #[test]
    fn built_in_solver_passes() {
        let dir = std::env::temp_dir().join(format!("plx-selftest-builtin-{}", std::process::id()));
        let solver = plx_job::Solver {
            executable: "unused".into(),
            threads: 1,
        };
        let report = run(Engine::PrePoLix, &solver, &dir, &Default::default());
        let _ = std::fs::remove_dir_all(&dir);
        for result in &report.checks {
            eprintln!("{}: {}", result.name, result.message);
        }
        assert_eq!(report.checks.len(), 4, "{report:#?}");
        assert!(report.checks.iter().all(|r| r.passed), "{report:#?}");
        assert_eq!(report.keywords, None);
        assert_eq!(report.version.as_deref(), Some(plx_solve::VERSION));
    }

    /// The self test with PrePoLix's own solver as a program (the CalculiX path):
    /// `PLXS=target/debug/plxs cargo test` after `cargo build -p plxs-cli` (skipped when the
    /// variable is not set).
    #[test]
    fn plxs_passes() {
        let Ok(executable) = std::env::var("PLXS") else {
            eprintln!("PLXS not set, test skipped");
            return;
        };
        let solver = plx_job::Solver {
            executable: executable.into(),
            threads: 1,
        };
        let dir = std::env::temp_dir().join(format!("plx-selftest-plxs-{}", std::process::id()));
        let report = run(Engine::CalculiX, &solver, &dir, &Default::default());
        let _ = std::fs::remove_dir_all(&dir);
        for result in &report.checks {
            eprintln!("{}: {}", result.name, result.message);
        }
        assert_eq!(report.checks.len(), 5);
        assert!(report.checks.iter().all(|r| r.passed), "{report:#?}");
    }

    #[test]
    fn unknown_keywords_are_found() {
        if Command::new("ccx").arg("-v").output().is_err() {
            eprintln!("ccx not found, test skipped");
            return;
        }
        let dir = std::env::temp_dir().join(format!("plx-keywordtest-{}", std::process::id()));
        let keywords = [
            "*Static".to_string(),
            "*NOT A KEYWORD".into(),
            "*node print".into(),
        ];
        let known =
            known_keywords(&plx_job::Solver::default(), &dir, &keywords, &|_, _| {}).unwrap();
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(known.len(), 3, "{known:?}");
        let unknown = unknown_of(&known, keywords.iter().map(String::as_str));
        assert_eq!(unknown, ["*NOT A KEYWORD"]);
    }

    #[test]
    fn cyclic_symmetry_test_does_not_crash_calculix() {
        if Command::new("ccx").arg("-v").output().is_err() {
            eprintln!("ccx not found, test skipped");
            return;
        }
        let dir = std::env::temp_dir().join(format!("plx-cyclictest-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let input = keyword_test_input("*Cyclic symmetry model");
        std::fs::write(dir.join("Keyword.inp"), input).unwrap();
        let output = Command::new("ccx")
            .arg("Keyword")
            .current_dir(&dir)
            .output()
            .unwrap();
        let _ = std::fs::remove_dir_all(&dir);
        let text = String::from_utf8_lossy(&output.stdout);
        // Alone the keyword ends in a segmentation fault; here CalculiX stops with an error.
        assert!(output.status.code().is_some(), "{text}");
        assert!(text.contains("*ERROR"), "{text}");
        assert!(!text.contains("cannot be interpreted"), "{text}");
    }

    #[test]
    fn missing_keywords_are_listed_by_feature() {
        let keywords = [
            "*SENSITIVITY",
            "*OBJECTIVE",
            "*MY KEYWORD",
            "*CYCLIC SYMMETRY MODEL",
        ]
        .map(String::from);
        assert_eq!(
            by_feature(&keywords),
            [
                "Sensitivity step (*SENSITIVITY, *OBJECTIVE)",
                "*MY KEYWORD",
                "Cyclic symmetry (*CYCLIC SYMMETRY MODEL)"
            ]
        );
        let known = BTreeMap::from([
            ("*STATIC".to_string(), true),
            ("*BUCKLE".to_string(), false),
        ]);
        let result = keyword_check(Some(&known), 2);
        assert!(!result.passed);
        assert!(
            result.message.ends_with("Buckle step (*BUCKLE)"),
            "{}",
            result.message
        );
        assert!(!keyword_check(None, 2).passed);
    }
}
