//! The meshing worker: the program started with `--mesh-worker <folder> <program id>` reads
//! its `task.ron` there and the `request.ron` of the job the task names, meshes the parts of
//! the groups it takes, each into `part-K.ron` in its own folder, says it is done in
//! `result.ron`, and ends when the program does: on Unix when its input is closed. Several
//! workers of one job share its groups, each group meshed by one of them.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use plx_mesh::FeMesh;
use plx_model::{Geometry, UnitSystem};
use serde::Serialize;

/// The worker's request, as `meshing::WorkerRequest` writes it.
#[derive(Serialize)]
struct Request {
    library: Option<PathBuf>,
    geometry: Geometry,
    parts: Vec<plx_model::PartRef>,
    groups: Vec<Vec<usize>>,
}

/// One worker's task, as `meshing::WorkerTask` writes it.
#[derive(Serialize)]
struct Task {
    job: PathBuf,
    first: Vec<usize>,
    take_groups: bool,
}

/// The worker's answer, as `meshing::WorkerResult`.
type WorkerResult = Result<(), String>;

/// One part's outcome, as `meshing::PartOutcome`.
type PartOutcome = Result<(FeMesh, Vec<String>), String>;

fn plate() -> Option<Geometry> {
    geometry("platte_mit_loch.step")
}

fn geometry(file: &str) -> Option<Geometry> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(file);
    match plx_mesher::import_cad(&path, UnitSystem::Unitless) {
        Ok(import) => Some(import.geometry),
        Err(error) if std::env::var_os("NODALIS_REQUIRE_GMSH").is_none() => {
            eprintln!("Gmsh not available, test skipped: {error}");
            None
        }
        Err(error) => panic!("Gmsh is required: {error}"),
    }
}

fn folder(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("nodalis-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// A job of all parts of the geometry in `dir`, each part a group, and the task of a worker
/// in the same folder that takes groups.
fn write_request(dir: &Path, geometry: Geometry) {
    let parts: Vec<plx_model::PartRef> = (plx_mesher::geometry_parts(&geometry).unwrap())
        .into_iter()
        .map(|p| p.id)
        .collect();
    let request = Request {
        library: plx_mesher::library_path(),
        groups: (0..parts.len()).map(|k| vec![k]).collect(),
        parts,
        geometry,
    };
    std::fs::write(dir.join("request.ron"), ron::to_string(&request).unwrap()).unwrap();
    write_task(dir, dir, &[], true);
}

fn write_task(dir: &Path, job: &Path, first: &[usize], take_groups: bool) {
    let task = Task {
        job: job.to_path_buf(),
        first: first.to_vec(),
        take_groups,
    };
    std::fs::write(dir.join("task.ron"), ron::to_string(&task).unwrap()).unwrap();
}

fn worker(dir: &Path) -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_nodalis"));
    command
        .arg("--mesh-worker")
        .arg(dir)
        .arg(std::process::id().to_string())
        .stdin(Stdio::piped())
        .stdout(Stdio::null());
    command
}

#[test]
fn the_worker_meshes_the_geometry() {
    let Some(geometry) = plate() else {
        return;
    };
    let dir = folder("mesh-worker");
    write_request(&dir, geometry);
    // The input stays open while it meshes, as the program keeps it; `wait` would close it.
    let mut child = worker(&dir).spawn().unwrap();
    let _input = child.stdin.take();
    let status = child.wait().unwrap();
    assert!(status.success(), "{status}");
    // The result was renamed into place whole.
    assert!(!dir.join("result.tmp").exists());
    let text = std::fs::read_to_string(dir.join("result.ron")).unwrap();
    let result: WorkerResult = ron::from_str(&text).unwrap();
    result.unwrap();
    let text = std::fs::read_to_string(dir.join("part-0.ron")).unwrap();
    let outcome: PartOutcome = ron::from_str(&text).unwrap();
    assert!(outcome.unwrap().0.element_count() > 100);
    assert!(!dir.join("part-1.ron").exists());
    // It said which part it was on.
    assert_eq!(std::fs::read_to_string(dir.join("current")).unwrap(), "0 0");
    let _ = std::fs::remove_dir_all(&dir);
}

/// Two workers of one job mesh each of its parts once between them, and a worker that only
/// gets the rest of a crashed one's group meshes just that.
#[test]
fn workers_share_the_parts_of_a_job() {
    let Some(mut geometry) = geometry("zwei_bloecke.step") else {
        return;
    };
    geometry.meshing.max_size = 5.0;
    let job = folder("mesh-job");
    write_request(&job, geometry);
    let dirs = [job.join("a"), job.join("b")];
    let mut children = Vec::new();
    for dir in &dirs {
        std::fs::create_dir_all(dir).unwrap();
        write_task(dir, &job, &[], true);
        let mut child = worker(dir).spawn().unwrap();
        children.push((child.stdin.take(), child));
    }
    for (_input, child) in &mut children {
        assert!(child.wait().unwrap().success());
    }
    for k in 0..2 {
        assert!(job.join(format!("group-{k}")).exists());
        let file = format!("part-{k}.ron");
        let by: Vec<&PathBuf> = dirs.iter().filter(|d| d.join(&file).exists()).collect();
        assert_eq!(by.len(), 1, "part {k} meshed by {by:?}");
        let text = std::fs::read_to_string(by[0].join(&file)).unwrap();
        let outcome: PartOutcome = ron::from_str(&text).unwrap();
        assert!(outcome.unwrap().0.element_count() > 10);
    }
    // All groups are taken: a third worker meshes only what its task names.
    let rest = job.join("c");
    std::fs::create_dir_all(&rest).unwrap();
    write_task(&rest, &job, &[1], true);
    let mut child = worker(&rest).spawn().unwrap();
    let _input = child.stdin.take();
    assert!(child.wait().unwrap().success());
    assert!(rest.join("part-1.ron").exists());
    assert!(!rest.join("part-0.ron").exists());
    let _ = std::fs::remove_dir_all(&job);
}

/// A worker whose program ends, crashed or killed, stops meshing at once.
#[test]
fn the_worker_stops_when_its_input_closes() {
    let Some(mut geometry) = plate() else {
        return;
    };
    // Fine enough to mesh for minutes.
    geometry.meshing.max_size /= 40.0;
    geometry.meshing.min_size = 0.0;
    let dir = folder("mesh-worker-closed");
    write_request(&dir, geometry);
    let mut child = worker(&dir).spawn().unwrap();
    std::thread::sleep(Duration::from_millis(1500));
    assert!(child.try_wait().unwrap().is_none(), "still meshing");
    let closed = Instant::now();
    child.stdin.take().unwrap().flush().unwrap();
    let status = child.wait().unwrap();
    assert!(!status.success());
    assert!(closed.elapsed() < Duration::from_secs(5));
    assert!(!dir.join("result.ron").exists());
    let _ = std::fs::remove_dir_all(&dir);
}
