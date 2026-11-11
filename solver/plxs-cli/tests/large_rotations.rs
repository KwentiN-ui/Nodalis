//! Finite rotations under NLGEOM: a steel plate (plane stress, 100 × 20 mm, 10 × 2 CPS4)
//! whose left edge is a rigid body.
//!
//! - Static: the rigid body is turned by 90° about z, with the plate's own weight acting.
//!   The plate turns with it as a whole and keeps its length (with linearized rigid body
//!   constraints its members would move along the tangent and the plate would be pulled
//!   apart).
//! - Dynamic: the rigid body is held at its reference point, free to turn, and gravity
//!   makes the plate a physical pendulum released horizontally. A quarter period, the time
//!   it takes to hang straight down, is `T/4 = K(sin 45°) / ω₀` with `ω₀² = g d / i²`
//!   (`d` the distance of the centre of mass from the pivot, `i²` the squared radius of
//!   gyration about the pivot); the plate keeps its length throughout.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

const LENGTH: f64 = 0.1;
const HEIGHT: f64 = 0.02;
/// The node in the middle of the right edge, whose motion is printed.
const TIP: usize = 1 + 10 + 11;

/// Nodes on an 11 × 3 grid, the rigid body's reference node 100 in the middle of the left
/// edge and its rotation node 101; `step` follows the model.
fn plate(step: &str) -> String {
    let mut s = String::from("*Node, Nset=NALL\n");
    let id = |i: usize, j: usize| 1 + i + 11 * j;
    for j in 0..3 {
        for i in 0..11 {
            writeln!(
                s,
                "{}, {}, {}, 0",
                id(i, j),
                LENGTH * i as f64 / 10.0,
                HEIGHT * j as f64 / 2.0
            )
            .unwrap();
        }
    }
    writeln!(
        s,
        "100, 0, {}, 0\n101, 0, {}, 0",
        HEIGHT / 2.0,
        HEIGHT / 2.0
    )
    .unwrap();
    s.push_str("*Element, Type=CPS4, Elset=EALL\n");
    let mut e = 0;
    for j in 0..2 {
        for i in 0..10 {
            e += 1;
            writeln!(
                s,
                "{e}, {}, {}, {}, {}",
                id(i, j),
                id(i + 1, j),
                id(i + 1, j + 1),
                id(i, j + 1)
            )
            .unwrap();
        }
    }
    writeln!(
        s,
        "*Nset, Nset=LEFT\n{}, {}, {}",
        id(0, 0),
        id(0, 1),
        id(0, 2)
    )
    .unwrap();
    s.push_str(
        "*Material, Name=STEEL\n*Elastic\n2.1e11, 0.3\n*Density\n7850\n\
         *Solid section, Elset=EALL, Material=STEEL\n0.01\n\
         *Rigid body, Nset=LEFT, Ref node=100, Rot node=101\n",
    );
    s.push_str(step);
    s
}

/// Runs `input` and returns the printed displacements of the tip per time:
/// `(time, [u1, u2])`.
fn run(dir: &Path, job: &str, input: &str) -> Vec<(f64, [f64; 2])> {
    std::fs::write(dir.join(format!("{job}.inp")), input).unwrap();
    let output = Command::new(plxs())
        .arg(job)
        .arg("--threads")
        .arg("2")
        .current_dir(dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned();
    assert!(output.status.success(), "{job} failed:\n{log}");
    assert!(log.contains("Job finished"), "{job}:\n{log}");
    let dat = std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap();
    let mut rows = Vec::new();
    let mut time = None;
    for line in dat.lines() {
        if line.contains("displacements (vx,vy,vz)") {
            time = line.split_whitespace().last().and_then(|t| t.parse().ok());
        } else if let Some(t) = time {
            let values: Vec<f64> = line
                .split_whitespace()
                .filter_map(|v| v.parse().ok())
                .collect();
            if values.len() == 4 {
                rows.push((t, [values[1], values[2]]));
                time = None;
            }
        }
    }
    assert!(!rows.is_empty(), "no displacements in\n{dat}");
    rows
}

fn work_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("plxs-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// The tip relative to the pivot: angle in degrees from +x and distance.
fn polar(u: [f64; 2]) -> (f64, f64) {
    let (x, y) = (LENGTH + u[0], u[1]);
    (y.atan2(x).to_degrees(), x.hypot(y))
}

#[test]
fn a_rigid_body_turned_by_ninety_degrees_carries_the_plate_along() {
    let dir = work_dir("rigid-turn");
    let step = format!(
        "*Step, Nlgeom, Inc=100\n*Static\n0.1, 1\n*Boundary\n100, 1, 2, 0\n101, 3, 3, {}\n\
         *Dload\nEALL, GRAV, 9.81, 0, -1, 0\n*Node print, Nset=TIPNODE\nU\n*End step\n",
        std::f64::consts::FRAC_PI_2
    );
    let input = plate(&format!("*Nset, Nset=TIPNODE\n{TIP}\n{step}"));
    let rows = run(&dir, "turn", &input);
    let _ = std::fs::remove_dir_all(&dir);
    let (_, u) = rows.last().unwrap();
    let (angle, distance) = polar(*u);
    // Its own weight bends the plate by a few tenths of a micrometre on the way and
    // shortens it by nanometres when it stands upright.
    assert!((angle - 90.0).abs() < 1e-3, "{angle}° ({u:?})");
    assert!((distance - LENGTH).abs() < 1e-6 * LENGTH, "{distance}");
}

#[test]
fn a_plate_hinged_at_a_rigid_edge_swings_as_a_pendulum() {
    let dir = work_dir("pendulum");
    let g = 9.81;
    let step = format!(
        "*Step, Nlgeom, Inc=1000\n*Dynamic, Direct\n0.005, 0.2\n*Boundary\n100, 1, 2, 0\n\
         *Dload\nEALL, GRAV, {g}, 0, -1, 0\n*Node print, Nset=TIPNODE\nU\n*End step\n"
    );
    let input = plate(&format!("*Nset, Nset=TIPNODE\n{TIP}\n{step}"));
    let rows = run(&dir, "pendulum", &input);
    let _ = std::fs::remove_dir_all(&dir);
    // Radius of gyration about the pivot and the quarter period released at 90°:
    // K(sin 45°) = 1.854075 (complete elliptic integral of the first kind).
    let d = LENGTH / 2.0;
    let i2 = (LENGTH * LENGTH + HEIGHT * HEIGHT) / 12.0 + d * d;
    let omega = (g * d / i2).sqrt();
    let quarter = 1.854_075 / omega;
    let mut crossing = None;
    let mut previous: Option<(f64, f64)> = None;
    for (t, u) in &rows {
        let (angle, distance) = polar(*u);
        assert!(
            (distance - LENGTH).abs() < 1e-4 * LENGTH,
            "t = {t}: the plate is {distance} long"
        );
        if let Some((t0, a0)) = previous
            && crossing.is_none()
            && a0 > -90.0
            && angle <= -90.0
        {
            crossing = Some(t0 + (a0 + 90.0) / (a0 - angle) * (t - t0));
        }
        previous = Some((*t, angle));
    }
    let crossing = crossing.unwrap_or_else(|| panic!("never hangs down: {rows:?}"));
    assert!(
        (crossing - quarter).abs() < 5e-3 * quarter,
        "hangs down at {crossing} s, expected {quarter} s"
    );
}
