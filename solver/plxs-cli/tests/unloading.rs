//! Unloading a body that yielded: a 3x3x3 C3D8 cube clamped on one face and pulled on the
//! other with 150 per node (about 267 MPa against a yield stress of 200), then a second
//! static step that lowers or removes the load. This diverged in the first increment of
//! the second step (with and without NLGEOM): the loads jumped to their new value at the
//! start of the step, so cutbacks did not make the increment smaller, and the converged
//! points on the yield surface started the increment with the elastoplastic tangent, which
//! made the unloading Newton step overshoot.
//!
//! Checked here: the second step converges, its first increment without a cutback, and the
//! load goes linearly from its value at the end of the first step to the new one (as in
//! CalculiX): the reaction at half the step is the mean of the two.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

/// The cube with `second` as the load cards of the second step.
fn cube(nlgeom: bool, second: &str) -> String {
    let mut s = String::from("*NODE, NSET=NALL\n");
    let id = |i: usize, j: usize, k: usize| 1 + i + 4 * j + 16 * k;
    for k in 0..4 {
        for j in 0..4 {
            for i in 0..4 {
                writeln!(s, "{}, {i}, {j}, {k}", id(i, j, k)).unwrap();
            }
        }
    }
    s.push_str("*ELEMENT, TYPE=C3D8, ELSET=EALL\n");
    let mut e = 0;
    for k in 0..3 {
        for j in 0..3 {
            for i in 0..3 {
                e += 1;
                writeln!(
                    s,
                    "{e}, {}, {}, {}, {}, {}, {}, {}, {}",
                    id(i, j, k),
                    id(i + 1, j, k),
                    id(i + 1, j + 1, k),
                    id(i, j + 1, k),
                    id(i, j, k + 1),
                    id(i + 1, j, k + 1),
                    id(i + 1, j + 1, k + 1),
                    id(i, j + 1, k + 1),
                )
                .unwrap();
            }
        }
    }
    let face = |i: usize| -> String {
        let mut out = String::new();
        for k in 0..4 {
            for j in 0..4 {
                writeln!(out, "{},", id(i, j, k)).unwrap();
            }
        }
        out
    };
    let step = if nlgeom { "*STEP, NLGEOM" } else { "*STEP" };
    write!(
        s,
        "*NSET, NSET=FIX\n{}*NSET, NSET=PULL\n{}\
*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n*PLASTIC\n200, 0\n400, 0.2\n\
*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL\n\
{step}\n*STATIC\n0.25, 1\n*BOUNDARY\nFIX, 1, 3\n*CLOAD\nPULL, 1, 150\n\
*NODE PRINT, NSET=FIX, TOTALS=ONLY\nRF\n*END STEP\n\
{step}\n*STATIC\n0.5, 1\n{second}*NODE PRINT, NSET=FIX, TOTALS=ONLY\nRF\n*END STEP\n",
        face(0),
        face(3)
    )
    .unwrap();
    s
}

/// Runs the job and returns (the `.sta` rows of step 2, the total x reactions by time).
fn run(dir: &Path, job: &str, input: &str) -> (Vec<Vec<f64>>, Vec<(f64, f64)>) {
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
    let sta = std::fs::read_to_string(dir.join(format!("{job}.sta"))).unwrap();
    let rows = (sta.lines())
        .map(|l| {
            l.split_whitespace()
                .filter_map(|t| t.parse::<f64>().ok())
                .collect::<Vec<_>>()
        })
        .filter(|r| r.len() == 7 && r[0] == 2.0)
        .collect();
    let dat = std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap();
    let mut totals = Vec::new();
    let mut time = None;
    for line in dat.lines() {
        if line.contains("total force") {
            time = line.split_whitespace().last().and_then(|t| t.parse().ok());
        } else if let (Some(t), Some(fx)) = (
            time,
            line.split_whitespace()
                .next()
                .and_then(|v| v.parse::<f64>().ok()),
        ) {
            totals.push((t, fx));
            time = None;
        }
    }
    (rows, totals)
}

#[test]
fn unloading_a_yielded_cube_converges_and_ramps_the_load_down() {
    let dir = std::env::temp_dir().join(format!("plxs-unloading-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let pulled = 16.0 * 150.0;
    for nlgeom in [false, true] {
        for (name, cards, end) in [
            ("removed", "*CLOAD, OP=NEW\n", 0.0),
            ("lowered", "*CLOAD\nPULL, 1, 100\n", 16.0 * 100.0),
            ("raised", "*CLOAD\nPULL, 1, 160\n", 16.0 * 160.0),
        ] {
            let job = format!("{name}-{}", if nlgeom { "nlgeom" } else { "small" });
            let (rows, totals) = run(&dir, &job, &cube(nlgeom, cards));
            assert!(!rows.is_empty(), "{job}: no increment of step 2");
            // Columns: step, increment, attempt, iterations, total, step time, increment.
            assert_eq!(rows[0][2], 1.0, "{job}: the first increment was cut back");
            assert_eq!(rows.last().unwrap()[5], 1.0, "{job}: step 2 did not finish");
            // The reaction carries the applied load: half way through step 2 it is the
            // mean of the load at the end of step 1 and the new one.
            // The `.dat` labels its blocks with the step time: step 2 starts where the
            // time goes back.
            let split = (1..totals.len())
                .find(|&i| totals[i].0 < totals[i - 1].0)
                .unwrap_or_else(|| panic!("{job}: no block of step 2: {totals:?}"));
            let at = |step: usize, t: f64| {
                let range = if step == 1 {
                    &totals[..split]
                } else {
                    &totals[split..]
                };
                range
                    .iter()
                    .find(|(time, _)| (time - t).abs() < 1e-9)
                    .map(|(_, fx)| -fx)
                    .unwrap_or_else(|| {
                        panic!("{job}: no reaction at {t} in step {step}: {totals:?}")
                    })
            };
            let tolerance = 1e-3 * pulled;
            assert!(
                (at(1, 1.0) - pulled).abs() < tolerance,
                "{job}: {}",
                at(1, 1.0)
            );
            let half = 0.5 * (pulled + end);
            assert!(
                (at(2, 0.5) - half).abs() < tolerance,
                "{job}: reaction {} at half of step 2, expected {half}",
                at(2, 0.5)
            );
            assert!(
                (at(2, 1.0) - end).abs() < tolerance,
                "{job}: {}",
                at(2, 1.0)
            );
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
}
