//! A ball pressed onto a plate by a force (axisymmetric, frictionless face-to-face
//! contact), held only by weak springs until it touches, as Nodalis writes such a model.
//! The ball touches the plate in one point at the start, so the first Newton iteration saw
//! no contact stiffness (the predictor reaches only points within the cutoff) and moved the
//! ball through the plate by its rigid-body displacement on the weak springs. Every point
//! within reach was then deeply closed, the contact set shrank by a few points per
//! iteration, and the increment ran out of iterations; cutbacks changed nothing because the
//! fly-through does not depend on the load.
//!
//! Checked here: the first increment converges without a cutback in a few iterations and
//! the plate carries the load.

use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

/// Ball of radius 5 mm (its lower 3 mm, 2 mm wide) on a 4 mm x 2 mm plate, CAX4, SI units.
fn model() -> String {
    let radius = 5e-3;
    let (bx, by) = (16, 12);
    let (px, py) = (32, 8);
    let mut s = String::from("*NODE\n");
    let ball = |i: usize, j: usize| 1 + i + (bx + 1) * j;
    for j in 0..=by {
        for i in 0..=bx {
            let x = 2e-3 * i as f64 / bx as f64;
            let bottom = radius - (radius * radius - x * x).sqrt();
            let y = bottom + (3e-3 - bottom) * j as f64 / by as f64;
            writeln!(s, "{}, {x:e}, {y:e}", ball(i, j)).unwrap();
        }
    }
    let first = 1 + (bx + 1) * (by + 1);
    let plate = |i: usize, j: usize| first + i + (px + 1) * j;
    for j in 0..=py {
        for i in 0..=px {
            let x = 4e-3 * i as f64 / px as f64;
            let y = -2e-3 + 2e-3 * j as f64 / py as f64;
            writeln!(s, "{}, {x:e}, {y:e}", plate(i, j)).unwrap();
        }
    }
    let mut e = 0;
    s.push_str("*ELEMENT, TYPE=CAX4, ELSET=BALL\n");
    for j in 0..by {
        for i in 0..bx {
            e += 1;
            let n = [
                ball(i, j),
                ball(i + 1, j),
                ball(i + 1, j + 1),
                ball(i, j + 1),
            ];
            writeln!(s, "{e}, {}, {}, {}, {}", n[0], n[1], n[2], n[3]).unwrap();
        }
    }
    let ball_bottom: Vec<usize> = (1..=bx).collect();
    s.push_str("*ELEMENT, TYPE=CAX4, ELSET=PLATE\n");
    let mut plate_top = Vec::new();
    for j in 0..py {
        for i in 0..px {
            e += 1;
            let n = [
                plate(i, j),
                plate(i + 1, j),
                plate(i + 1, j + 1),
                plate(i, j + 1),
            ];
            writeln!(s, "{e}, {}, {}, {}, {}", n[0], n[1], n[2], n[3]).unwrap();
            if j == py - 1 {
                plate_top.push(e);
            }
        }
    }
    // Weak springs on every ball node, one set per direction.
    s.push_str("*ELEMENT, TYPE=SPRING1, ELSET=WEAK\n");
    for n in 1..first {
        e += 1;
        writeln!(s, "{e}, {n}").unwrap();
    }
    s.push_str("*SPRING, ELSET=WEAK\n1\n0.1\n");
    s.push_str("*ELEMENT, TYPE=SPRING1, ELSET=WEAK2\n");
    for n in 1..first {
        e += 1;
        writeln!(s, "{e}, {n}").unwrap();
    }
    s.push_str("*SPRING, ELSET=WEAK2\n2\n0.1\n");
    s.push_str("*SURFACE, NAME=SLAVE, TYPE=ELEMENT\n");
    for f in &ball_bottom {
        writeln!(s, "{f}, S1").unwrap();
    }
    s.push_str("*SURFACE, NAME=MASTER, TYPE=ELEMENT\n");
    for f in &plate_top {
        writeln!(s, "{f}, S3").unwrap();
    }
    s.push_str("*NSET, NSET=FIX\n");
    for i in 0..=px {
        writeln!(s, "{},", plate(i, 0)).unwrap();
    }
    s.push_str("*NSET, NSET=TOP\n");
    for i in 0..=bx {
        writeln!(s, "{},", ball(i, by)).unwrap();
    }
    s.push_str(
        "*MATERIAL, NAME=STEEL\n*ELASTIC\n2.1e11, 0.3\n\
*SOLID SECTION, ELSET=BALL, MATERIAL=STEEL\n*SOLID SECTION, ELSET=PLATE, MATERIAL=STEEL\n\
*SURFACE INTERACTION, NAME=SMOOTH\n*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=LINEAR\n1e16, 3e6\n\
*CONTACT PAIR, INTERACTION=SMOOTH, TYPE=SURFACE TO SURFACE\nSLAVE, MASTER\n\
*STEP, INC=100\n*STATIC\n*BOUNDARY\nFIX, 1, 2, 0\n*CLOAD\nTOP, 2, -20\n\
*NODE PRINT, NSET=FIX, TOTALS=ONLY\nRF\n*END STEP\n",
    );
    s
}

#[test]
fn a_ball_held_by_weak_springs_is_pressed_onto_a_plate() {
    let dir = std::env::temp_dir().join(format!("plxs-hertz-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("hertz.inp"), model()).unwrap();
    let output = Command::new(plxs())
        .arg("hertz")
        .arg("--threads")
        .arg("2")
        .current_dir(&dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned();
    assert!(output.status.success(), "hertz failed:\n{log}");
    let sta = std::fs::read_to_string(dir.join("hertz.sta")).unwrap();
    // Columns: step, increment, attempt, iterations, total, step time, increment.
    let rows: Vec<Vec<f64>> = (sta.lines())
        .map(|l| {
            l.split_whitespace()
                .filter_map(|t| t.parse::<f64>().ok())
                .collect::<Vec<_>>()
        })
        .filter(|r| r.len() == 7)
        .collect();
    assert_eq!(rows.len(), 1, "one increment expected:\n{sta}");
    assert_eq!(rows[0][2], 1.0, "the increment was cut back:\n{sta}");
    assert!(rows[0][3] <= 10.0, "too many iterations:\n{sta}");
    // The plate carries the load of the 17 top nodes.
    let dat = std::fs::read_to_string(dir.join("hertz.dat")).unwrap();
    let fy: f64 = dat
        .lines()
        .skip_while(|l| !l.contains("total force"))
        .skip(1)
        .find(|l| !l.trim().is_empty())
        .and_then(|l| l.split_whitespace().nth(1))
        .and_then(|v| v.parse().ok())
        .unwrap_or_else(|| panic!("no total reaction:\n{dat}"));
    assert!((fy - 17.0 * 20.0).abs() < 1e-2 * 340.0, "reaction {fy}");
    let _ = std::fs::remove_dir_all(&dir);
}
