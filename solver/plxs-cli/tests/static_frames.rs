//! Result frames of a static step (`*STATIC, FRAMES=`): the increments end on the frames,
//! only the frames are written, the first increment respects the largest one, and a linear
//! step is solved once per frame. A 3x3x3 C3D8 cube clamped on one face and pulled on the
//! other, elastic or (pulled beyond yield) elastoplastic.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

/// The cube with `procedure` (the `*STATIC` card and its data line) in its step.
fn cube(plastic: bool, step: &str, procedure: &str) -> String {
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
    let plasticity = if plastic {
        "*PLASTIC\n200, 0\n400, 0.2\n"
    } else {
        ""
    };
    write!(
        s,
        "*NSET, NSET=FIX\n{}*NSET, NSET=PULL\n{}\
*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n{plasticity}\
*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL\n\
{step}\n{procedure}*BOUNDARY\nFIX, 1, 3\n*CLOAD\nPULL, 1, 150\n\
*NODE FILE\nU\n*NODE PRINT, NSET=FIX, TOTALS=ONLY\nRF\n*END STEP\n",
        face(0),
        face(3)
    )
    .unwrap();
    s
}

struct Run {
    /// The `.sta` rows: step, increment, attempt, iterations, total, step time, size.
    increments: Vec<Vec<f64>>,
    /// The times of the result frames in the `.frd`.
    frames: Vec<f64>,
}

fn run(job: &str, input: &str) -> Run {
    let dir: &Path = &std::env::temp_dir().join(format!("plxs-frames-{}", std::process::id()));
    std::fs::create_dir_all(dir).unwrap();
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
    let increments = (sta.lines())
        .map(|l| {
            l.split_whitespace()
                .filter_map(|t| t.parse::<f64>().ok())
                .collect::<Vec<_>>()
        })
        .filter(|r| r.len() == 7)
        .collect();
    let frd = std::fs::read_to_string(dir.join(format!("{job}.frd"))).unwrap();
    let frames = (frd.lines())
        .filter(|l| l.trim_start().starts_with("100CL"))
        .filter_map(|l| l.split_whitespace().nth(2)?.parse().ok())
        .collect();
    Run { increments, frames }
}

fn assert_times(job: &str, times: &[f64], expected: &[f64]) {
    assert_eq!(times.len(), expected.len(), "{job}: {times:?}");
    for (t, e) in times.iter().zip(expected) {
        assert!(
            (t - e).abs() < 1e-9,
            "{job}: {times:?} instead of {expected:?}"
        );
    }
}

#[test]
fn nonlinear_increments_end_on_the_frames_and_only_frames_are_written() {
    // Plastic: the nonlinear procedure. A first increment of 0.3 grows by half; the frames
    // at 0.25, 0.5, 0.75 and 1 cut the increments that would pass them.
    let run_ = run(
        "plastic-frames",
        &cube(true, "*STEP", "*STATIC, FRAMES=4\n0.3, 1\n"),
    );
    assert_times("plastic-frames", &run_.frames, &[0.25, 0.5, 0.75, 1.0]);
    let ends: Vec<f64> = run_.increments.iter().map(|r| r[5]).collect();
    for frame in [0.25, 0.5, 0.75, 1.0] {
        assert!(
            ends.iter().any(|t| (t - frame).abs() < 1e-9),
            "no increment ends on {frame}: {ends:?}"
        );
    }
}

#[test]
fn the_first_increment_respects_the_largest_one() {
    // Initial 0.5, largest 0.1: every increment, the first one too, is at most 0.1.
    let run_ = run(
        "plastic-largest",
        &cube(
            true,
            "*STEP, INC=1000",
            "*STATIC, FRAMES=2\n0.5, 1, , 0.1\n",
        ),
    );
    let sizes: Vec<f64> = run_.increments.iter().map(|r| r[6]).collect();
    assert!(sizes.len() >= 10, "{sizes:?}");
    assert!(sizes.iter().all(|&s| s <= 0.1 + 1e-12), "{sizes:?}");
    assert_times("plastic-largest", &run_.frames, &[0.5, 1.0]);
}

#[test]
fn a_linear_step_is_solved_once_per_frame() {
    let run_ = run(
        "elastic-frames",
        &cube(false, "*STEP", "*STATIC, FRAMES=5\n1, 2\n"),
    );
    assert_times("elastic-frames", &run_.frames, &[0.4, 0.8, 1.2, 1.6, 2.0]);
    assert_eq!(run_.increments.len(), 5);
    // Without frames once, at the end.
    let once = run("elastic-once", &cube(false, "*STEP", "*STATIC\n0.1, 2\n"));
    assert_times("elastic-once", &once.frames, &[2.0]);
}

#[test]
fn a_large_increment_limit_lets_a_long_step_run_to_its_end() {
    // 200 fixed increments, more than the default INC of 100: Nodalis writes the limit
    // only when the user sets one, as the largest number otherwise.
    let run_ = run(
        "fixed-many",
        &cube(
            true,
            "*STEP, INC=4294967295",
            "*STATIC, DIRECT, FRAMES=4\n0.005, 1\n",
        ),
    );
    assert_eq!(run_.increments.len(), 200);
    assert_times("fixed-many", &run_.frames, &[0.25, 0.5, 0.75, 1.0]);
}
