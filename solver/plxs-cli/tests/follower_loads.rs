//! Follower loads (`*CLOAD, FOLLOWER=`) on a steel plate (plane stress, 100 × 20 mm,
//! 10 × 2 CPS4) whose left edge is a rigid body.
//!
//! - Static: the rigid body is turned by 90° about z. A force at the right end of the
//!   plate that follows the rotation node, the left edge or the whole plate turns with it:
//!   the reaction at the reference node turns by 90° too; a force without FOLLOWER keeps
//!   its direction.
//! - Dynamic: the plate turns freely about its reference node, driven by a force at the
//!   right edge that stays normal to the plate: a constant moment, so the angle grows as
//!   `½ α t²` with `α = F L / I` (implicit and explicit time integration).

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
    rows(dir, job, input, "displacements (vx,vy,vz)")
}

/// Runs `input` and returns the first node's values of the printed table `title` per time.
fn rows(dir: &Path, job: &str, input: &str, title: &str) -> Vec<(f64, [f64; 2])> {
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
        if line.contains(title) {
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

/// The force of the reaction at the reference node after turning the rigid body by 90°
/// with a force of 1000 N along x at the right end of the plate, following `follower`
/// (empty: none).
fn reaction_after_turning(job: &str, follower: &str) -> [f64; 2] {
    let dir = work_dir(job);
    let step = format!(
        "*Step, Nlgeom, Inc=100\n*Static\n0.1, 1\n*Boundary\n100, 1, 2, 0\n101, 3, 3, {}\n\
         *Cload{follower}\n{TIP}, 1, 1000\n*Node print, Nset=REFNODE\nRF\n*End step\n",
        std::f64::consts::FRAC_PI_2
    );
    let input = plate(&format!(
        "*Nset, Nset=REFNODE\n100\n*Nset, Nset=PLATE, Generate\n1, 33, 1\n{step}"
    ));
    let rows = rows(&dir, job, &input, "forces (fx,fy,fz)");
    let _ = std::fs::remove_dir_all(&dir);
    rows.last().unwrap().1
}

#[test]
fn a_follower_force_turns_with_the_rigid_body_it_acts_on() {
    for (job, follower) in [
        ("rot", ", Follower rotation=101"),
        ("edge", ", Follower=LEFT"),
        ("plate", ", Follower=PLATE"),
    ] {
        let rf = reaction_after_turning(job, follower);
        // The plate stretches by micrometres under the force; the iterations end within
        // the residual tolerance (the load stiffness is not in the tangent).
        assert!(
            rf[0].abs() < 0.1 && (rf[1] + 1000.0).abs() < 0.1,
            "{job}: {rf:?}"
        );
    }
    let rf = reaction_after_turning("fixed", "");
    assert!((rf[0] + 1000.0).abs() < 0.1 && rf[1].abs() < 0.1, "{rf:?}");
}

/// The angle of the plate (degrees) per time, turned about its hinge by a force normal to
/// the plate at its right edge.
fn swing(job: &str, procedure: &str, period: f64, force: f64) -> Vec<(f64, f64)> {
    let dir = work_dir(job);
    let share = force / 3.0;
    let step = format!(
        "*Step, Nlgeom, Inc=100000\n{procedure}\n{}, {period}\n*Boundary\n100, 1, 2, 0\n\
         *Cload, Follower=RIGHT\nRIGHT, 2, {share}\n*Node print, Nset=TIPNODE\nU\n*End step\n",
        period / 20.0
    );
    let right = format!("*Nset, Nset=RIGHT\n11, 22, 33\n*Nset, Nset=TIPNODE\n{TIP}\n");
    let rows = run(&dir, job, &plate(&format!("{right}{step}")));
    let _ = std::fs::remove_dir_all(&dir);
    rows.into_iter().map(|(t, u)| (t, polar(u).0)).collect()
}

#[test]
fn a_follower_force_normal_to_a_hinged_plate_turns_it_at_constant_acceleration() {
    let mass = 7850.0 * LENGTH * HEIGHT * 0.01;
    let d = LENGTH / 2.0;
    let inertia = mass * ((LENGTH * LENGTH + HEIGHT * HEIGHT) / 12.0 + d * d);
    let period = 0.02;
    // A quarter turn at the end of the period.
    let alpha = 2.0 * std::f64::consts::FRAC_PI_2 / (period * period);
    let force = alpha * inertia / LENGTH;
    for (job, procedure) in [
        ("implicit", "*Dynamic, Direct"),
        ("explicit", "*Dynamic, Integration=Explicit"),
    ] {
        let rows = swing(job, procedure, period, force);
        for (t, angle) in rows.iter().filter(|(t, _)| *t > 0.25 * period) {
            let expected = (0.5 * alpha * t * t).to_degrees();
            assert!(
                (angle - expected).abs() < 0.02 * expected,
                "{job}, t = {t}: {angle}°, expected {expected}°"
            );
        }
        let (_, last) = rows.last().unwrap();
        assert!((last - 90.0).abs() < 2.0, "{job}: {last}°");
    }
}

/// A beam (B31, 100 mm, 10 × 5 mm) whose clamped end is turned by 90° about z, with a
/// force of `force` N along x at its free end, following `follower` (empty: none): the
/// displacement of the free end.
fn beam_tip(job: &str, force: f64, follower: &str) -> [f64; 2] {
    let dir = work_dir(job);
    let mut s = String::from("*Node\n");
    for i in 0..=10 {
        writeln!(s, "{}, {}, 0, 0", i + 1, 10.0 * i as f64).unwrap();
    }
    s.push_str("*Element, Type=B31, Elset=BEAM\n");
    for i in 1..=10 {
        writeln!(s, "{i}, {i}, {}", i + 1).unwrap();
    }
    write!(
        s,
        "*Nset, Nset=TIPNODE\n11\n*Material, Name=STEEL\n*Elastic\n210000, 0.3\n\
         *Beam section, Elset=BEAM, Material=STEEL, Section=RECT\n10, 5\n0, 0, 1\n\
         *Step, Nlgeom, Inc=100\n*Static\n0.1, 1\n*Boundary\n1, 1, 5, 0\n1, 6, 6, {}\n\
         *Cload{follower}\n11, 1, {force}\n*Node print, Nset=TIPNODE\nU\n*End step\n",
        std::f64::consts::FRAC_PI_2
    )
    .unwrap();
    let rows = run(&dir, job, &s);
    let _ = std::fs::remove_dir_all(&dir);
    rows.last().unwrap().1
}

#[test]
fn a_follower_force_at_a_beam_node_turns_with_the_beam() {
    let turned = beam_tip("beam-free", 0.0, "");
    let change = |u: [f64; 2]| [u[0] - turned[0], u[1] - turned[1]];
    // The beam node expands to the corners of its section, which turn with it: the force
    // pulls along the turned beam and stretches it.
    let pulled = change(beam_tip("beam-tip", 100.0, ", Follower=TIPNODE"));
    assert!(pulled[0].abs() < 1e-3 && pulled[1] > 0.0, "{pulled:?}");
    // Without FOLLOWER it acts across the turned beam and bends it.
    let bent = change(beam_tip("beam-fixed", 100.0, ""));
    assert!(bent[0] > 0.1, "{bent:?}");
}
