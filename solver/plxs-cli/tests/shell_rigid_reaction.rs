//! A shell edge coupled rigidly to a point by equations (the way a rigid displacement or
//! reference point takes shell nodes, which `*RIGID BODY` refuses): the point reports the
//! force that moves the edge. The shell's nodes have no element of their own, the solver's
//! expanded nodes take the element forces; the force has to come back to the shell nodes
//! before the equations carry it on to the point.

use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

/// The total force printed for `set`.
fn total(dat: &str, set: &str) -> [f64; 3] {
    let header = format!("total force (fx,fy,fz) for set {set} ");
    let block = dat.split(&header).nth(1).expect("a total force block");
    let line = block
        .lines()
        .map(str::trim)
        .find(|l| !l.is_empty() && !l.starts_with("and"));
    let values: Vec<f64> = line
        .unwrap()
        .split_whitespace()
        .map(|v| v.parse().unwrap())
        .collect();
    [values[0], values[1], values[2]]
}

#[test]
fn rigid_point_of_a_shell_edge_reports_the_force() {
    // A strip of 300 x 30 mm, 2 mm thick, of 10 x 2 S4, clamped at x = 0; the edge at
    // x = 300 follows a point at its centre lifted by 1 mm.
    let (length, width, columns, rows) = (300.0, 30.0, 10u32, 2u32);
    let id = |i: u32, j: u32| 1 + i * (rows + 1) + j;
    let y = |j: u32| width * f64::from(j) / f64::from(rows) - width / 2.0;
    let mut s = String::from("*NODE\n");
    for i in 0..=columns {
        for j in 0..=rows {
            let x = length * f64::from(i) / f64::from(columns);
            writeln!(s, "{}, {x}, {}, 0", id(i, j), y(j)).unwrap();
        }
    }
    s.push_str("1001, 300, 0, 0\n1002, 300, 0, 0\n*ELEMENT, TYPE=S4, ELSET=PLATE\n");
    let mut e = 1;
    for i in 0..columns {
        for j in 0..rows {
            let n = [id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)];
            writeln!(s, "{e}, {}, {}, {}, {}", n[0], n[1], n[2], n[3]).unwrap();
            e += 1;
        }
    }
    s.push_str(
        "*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n\
         *SHELL SECTION, ELSET=PLATE, MATERIAL=STEEL\n2.0\n*NSET, NSET=FIXED\n",
    );
    let fixed: Vec<String> = (0..=rows).map(|j| id(0, j).to_string()).collect();
    writeln!(s, "{}", fixed.join(", ")).unwrap();
    s.push_str("*NSET, NSET=RP\n1001\n*EQUATION\n");
    // u = u_point + theta x r with r = (0, y, 0).
    for j in 0..=rows {
        let n = id(columns, j);
        writeln!(s, "2\n{n}, 1, 1.0, 1001, 1, -1.0").unwrap();
        if y(j) == 0.0 {
            writeln!(s, "2\n{n}, 2, 1.0, 1001, 2, -1.0").unwrap();
            writeln!(s, "2\n{n}, 3, 1.0, 1001, 3, -1.0").unwrap();
        } else {
            writeln!(s, "3\n{n}, 1, 1.0, 1001, 1, -1.0, 1002, 3, {}", y(j)).unwrap();
            writeln!(s, "2\n{n}, 2, 1.0, 1001, 2, -1.0").unwrap();
            writeln!(s, "3\n{n}, 3, 1.0, 1001, 3, -1.0, 1002, 1, {}", -y(j)).unwrap();
        }
    }
    s.push_str(
        "*STEP\n*STATIC\n*BOUNDARY\nFIXED, 1, 6\n1001, 3, 3, 1.0\n\
         *NODE PRINT, NSET=RP, TOTALS=YES\nRF\n\
         *NODE PRINT, NSET=FIXED, TOTALS=YES\nRF\n*END STEP\n",
    );
    let dir = std::env::temp_dir().join(format!("plxs-shell-rigid-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("strip.inp"), s).unwrap();
    let output = Command::new(plxs())
        .args(["strip", "--threads", "2"])
        .current_dir(&dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned();
    assert!(output.status.success(), "strip failed:\n{log}");
    let dat = std::fs::read_to_string(dir.join("strip.dat")).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    let point = total(&dat, "RP");
    let clamp = total(&dat, "FIXED");
    // A cantilever lifted at its tip: 3 E I / L^3 = 0.467 N as a beam, a little stiffer as
    // a plate (E / (1 - nu^2) = 0.513 N at most).
    assert!(
        point[2] > 0.46 && point[2] < 0.52,
        "point: {point:?}\n{dat}"
    );
    // Equilibrium with the clamped end.
    assert!(
        (point[2] + clamp[2]).abs() < 1e-6 * point[2],
        "point {point:?}, clamp {clamp:?}"
    );
}
