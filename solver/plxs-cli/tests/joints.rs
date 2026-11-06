//! Joints (`*JOINT`) between two steel plates (plane stress, 100 × 20 mm each, 10 × 2
//! CPS4): plate A is clamped at its left edge, plate B lies to its right. The facing edges
//! are rigid bodies, A's with its reference node at the joint, the middle of the gap, B's
//! with its reference node in the middle of B, so that the joint reaches B by a lever.
//!
//! - A fixed joint makes the two plates one: the tip of B moves as when both edges are one
//!   rigid body.
//! - A revolute joint turned by one radian under NLGEOM, the plates under their own
//!   weight: plate B swings about the joint as a whole and the joint stays closed (with
//!   linearized constraints B's nodes would move along the tangent, a tenth of its length
//!   off at this angle).

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

const LENGTH: f64 = 0.1;
const HEIGHT: f64 = 0.02;

/// Node `i, j` of plate `p` (0 = A, 1 = B).
fn id(p: usize, i: usize, j: usize) -> usize {
    1 + i + 11 * j + 100 * p
}

/// The node in the middle of the right edge of plate B, whose motion is printed.
fn tip() -> usize {
    id(1, 10, 1)
}

/// The two plates, the clamp and the edge sets; `rest` follows.
fn plates(rest: &str) -> String {
    let mut s = String::from("*Node, Nset=NALL\n");
    for p in 0..2 {
        for j in 0..3 {
            for i in 0..11 {
                writeln!(
                    s,
                    "{}, {}, {}, 0",
                    id(p, i, j),
                    LENGTH * (p as f64 + i as f64 / 10.0),
                    HEIGHT * j as f64 / 2.0
                )
                .unwrap();
            }
        }
    }
    // A's point at the joint, B's in the middle of B.
    for (n, x) in [
        (300, LENGTH),
        (301, LENGTH),
        (302, 1.5 * LENGTH),
        (303, 1.5 * LENGTH),
    ] {
        writeln!(s, "{n}, {x}, {}, 0", HEIGHT / 2.0).unwrap();
    }
    s.push_str("*Element, Type=CPS4, Elset=EALL\n");
    let mut e = 0;
    for p in 0..2 {
        for j in 0..2 {
            for i in 0..10 {
                e += 1;
                writeln!(
                    s,
                    "{e}, {}, {}, {}, {}",
                    id(p, i, j),
                    id(p, i + 1, j),
                    id(p, i + 1, j + 1),
                    id(p, i, j + 1)
                )
                .unwrap();
            }
        }
    }
    let edge = |p: usize, i: usize| {
        (0..3)
            .map(|j| id(p, i, j).to_string())
            .collect::<Vec<_>>()
            .join(", ")
    };
    writeln!(s, "*Nset, Nset=CLAMP\n{}", edge(0, 0)).unwrap();
    writeln!(s, "*Nset, Nset=AEDGE\n{}", edge(0, 10)).unwrap();
    writeln!(s, "*Nset, Nset=BEDGE\n{}", edge(1, 0)).unwrap();
    writeln!(s, "*Nset, Nset=BOTH\n{}, {}", edge(0, 10), edge(1, 0)).unwrap();
    writeln!(s, "*Nset, Nset=TIPNODE\n{}", tip()).unwrap();
    s.push_str(
        "*Material, Name=STEEL\n*Elastic\n2.1e11, 0.3\n*Density\n7850\n\
         *Solid section, Elset=EALL, Material=STEEL\n0.01\n",
    );
    s.push_str(rest);
    s
}

/// The facing edges as rigid bodies with points 300/301 (A, at the joint) and 302/303 (B,
/// half a plate away), joined by a joint holding `fixed`.
fn joined(fixed: &str) -> String {
    format!(
        "*Rigid body, Nset=AEDGE, Ref node=300, Rot node=301\n\
         *Rigid body, Nset=BEDGE, Ref node=302, Rot node=303\n\
         *Joint, Name=J\n300, 301, 302, 303\n{LENGTH}, {}, 0, 1, 0, 0, 0, 1, 0\n{fixed}\n",
        HEIGHT / 2.0
    )
}

/// Runs `input` and returns the printed displacements of the tip per time.
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
    let dir = std::env::temp_dir().join(format!("plxs-joint-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
fn a_fixed_joint_makes_the_plates_one() {
    let dir = work_dir("fixed");
    let step = format!(
        "*Step\n*Static\n*Boundary\nCLAMP, 1, 2, 0\n*Cload\n{}, 2, -1000\n\
         *Node print, Nset=TIPNODE\nU\n*End step\n",
        tip()
    );
    let jointed = plates(&format!("{}{step}", joined("1, 2, 6")));
    let one = plates(&format!(
        "*Rigid body, Nset=BOTH, Ref node=300, Rot node=301\n{step}"
    ));
    let a = run(&dir, "jointed", &jointed);
    let b = run(&dir, "one", &one);
    let _ = std::fs::remove_dir_all(&dir);
    let (ua, ub) = (a.last().unwrap().1, b.last().unwrap().1);
    assert!(ub[1] < -1e-6, "the plate bends: {ub:?}");
    for d in 0..2 {
        assert!(
            (ua[d] - ub[d]).abs() < 1e-9 * ub[1].abs(),
            "{ua:?} vs {ub:?}"
        );
    }
}

#[test]
fn a_revolute_joint_turns_the_second_plate_as_a_whole() {
    let dir = work_dir("revolute");
    let angle = 1.0_f64;
    let step = format!(
        "*Nset, Nset=AREF\n300\n*Step, Nlgeom, Inc=100\n*Static\n0.1, 1\n*Boundary\n\
         CLAMP, 1, 2, 0\n303, 3, 3, {angle}\n*Dload\nEALL, GRAV, 9.81, 0, -1, 0\n\
         *Node print, Nset=TIPNODE\nU\n\
         *Node print, Nset=AREF\nU\n*End step\n"
    );
    let input = plates(&format!("{}{step}", joined("1, 2")));
    let rows = run(&dir, "revolute", &input);
    let _ = std::fs::remove_dir_all(&dir);
    // The rows alternate between the tip and the reference node of A.
    let tip_u = rows[rows.len() - 2].1;
    let a_ref = rows[rows.len() - 1].1;
    // The tip lies a plate length from the joint, turned by the angle; the weight of B
    // bends the plates by tenths of a micrometre.
    let (x, y) = (LENGTH + tip_u[0] - a_ref[0], tip_u[1] - a_ref[1]);
    let turned = y.atan2(x);
    let distance = x.hypot(y);
    assert!(
        (turned - angle).abs() < 1e-5,
        "turned by {turned} ({tip_u:?})"
    );
    assert!((distance - LENGTH).abs() < 1e-5 * LENGTH, "{distance}");
    // Plate A carries B's weight at the joint: it sags by less than a micrometre.
    assert!(a_ref[1] < 0.0 && a_ref[1] > -1e-6, "{a_ref:?}");
}

#[test]
fn a_damped_pendulum_on_a_joint_swings_freely() {
    let dir = work_dir("pendulum");
    let step_of = |dt: f64, damping: &str| {
        format!(
            "*Step, Nlgeom, Inc=10000\n*Dynamic, Direct\n{dt}, 0.3\n{damping}*Boundary\n\
             CLAMP, 1, 2, 0\n*Dload\nEALL, GRAV, 9.81, 0, -1, 0\n\
             *Node print, Nset=TIPNODE\nU\n*End step\n"
        )
    };
    let step = |damping: &str| step_of(0.002, damping);
    let free = run(
        &dir,
        "free",
        &plates(&format!("{}{}", joined("1, 2"), step(""))),
    );
    // Stiffness-proportional damping as given, and as the automatic fit puts it.
    let given = run(
        &dir,
        "given",
        &plates(&format!(
            "{}{}",
            joined("1, 2"),
            step("*Damping, Beta=1.3e-4\n")
        )),
    );
    let fitted = run(
        &dir,
        "fitted",
        &plates(&format!(
            "{}{}",
            joined("1, 2"),
            step("*Damping, Ratio=0.02, Upper=50\n")
        )),
    );
    // Damping as strong as the automatic fit gives for a time step of the swing, the time
    // step as long (β_R about a tenth of Δt): with the damping force K_T(u) v the Newton
    // iterations stalled, and the swing fell behind by half a plate length.
    let strong = run(
        &dir,
        "strong",
        &plates(&format!(
            "{}{}",
            joined("1, 2"),
            step_of(0.03, "*Damping, Beta=2e-3\n")
        )),
    );
    let _ = std::fs::remove_dir_all(&dir);
    // Plate B falls from the horizontal about the joint; after 0.3 s it hangs down, its tip
    // a plate length left of where it started.
    let end = free.last().unwrap().1;
    assert!(end[0] < -0.9 * LENGTH * 2.0, "the pendulum swings: {end:?}");
    // Damping that acted on the rigid rotation (the initial stiffness, or mass-proportional
    // damping) let it creep down by millimetres instead. The stiffness-proportional damping
    // still acts on the pendulum's own restoring stiffness (the weight turning with it),
    // little when weak and a few percent of the swing when strong.
    for (name, rows, tolerance) in [
        ("given", &given, 0.02),
        ("fitted", &fitted, 0.02),
        ("strong", &strong, 0.25),
    ] {
        let u = rows.last().unwrap().1;
        for d in 0..2 {
            assert!(
                (u[d] - end[d]).abs() < tolerance * LENGTH,
                "{name}: {u:?} vs undamped {end:?}"
            );
        }
    }
}
