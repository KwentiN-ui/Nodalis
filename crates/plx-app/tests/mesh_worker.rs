//! The meshing worker: the program started with `--mesh-worker <folder> <program id>` meshes
//! the geometry in `request.ron` there into `result.ron`, and ends when the program does: on
//! Unix when its input is closed.

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
    parts: Option<Vec<String>>,
}

type WorkerResult = Result<Vec<(FeMesh, Vec<String>)>, String>;

fn plate() -> Option<Geometry> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/platte_mit_loch.step");
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

fn write_request(dir: &Path, geometry: Geometry) {
    let request = Request {
        library: plx_mesher::library_path(),
        geometry,
        parts: None,
    };
    std::fs::write(dir.join("request.ron"), ron::to_string(&request).unwrap()).unwrap();
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
    let meshes = result.unwrap();
    assert_eq!(meshes.len(), 1);
    assert!(meshes[0].0.element_count() > 100);
    let _ = std::fs::remove_dir_all(&dir);
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
