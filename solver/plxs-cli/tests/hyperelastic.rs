//! Hyperelastic materials end to end: a unit cube of one C3D8 element on symmetry supports,
//! stretched to twice its length by a prescribed displacement in a step without NLGEOM
//! (the solver switches it on for hyperelastic materials, as CalculiX does). The deformation
//! is homogeneous, so the reaction force must be the analytic one of the compressible
//! Neo-Hooke model: with `F = diag(λ, t, t)` and `J = λ t²`, the Cauchy stresses are
//! `σ11 - σ22 = 2 C10 J^(-5/3) (λ² - t²)` and `σ22 = 0` fixes `t` through
//! `2 C10 J^(-5/3) (λ² - t²) / 3 = 2 (J - 1) / D1`; the force is `σ11 t²`.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

fn cube(material: &str, step: &str) -> String {
    let mut s = String::from("*NODE, NSET=NALL\n");
    let corners = [
        [0, 0, 0],
        [1, 0, 0],
        [1, 1, 0],
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 1],
        [1, 1, 1],
        [0, 1, 1],
    ];
    for (i, c) in corners.iter().enumerate() {
        writeln!(s, "{}, {}, {}, {}", i + 1, c[0], c[1], c[2]).unwrap();
    }
    s.push_str("*ELEMENT, TYPE=C3D8, ELSET=EALL\n1, 1, 2, 3, 4, 5, 6, 7, 8\n");
    let set = |axis: usize, value: i32| -> String {
        (corners.iter().enumerate())
            .filter(|(_, c)| c[axis] == value)
            .map(|(i, _)| format!("{},", i + 1))
            .collect::<Vec<_>>()
            .join(" ")
    };
    write!(
        s,
        "*NSET, NSET=X0\n{}\n*NSET, NSET=Y0\n{}\n*NSET, NSET=Z0\n{}\n*NSET, NSET=X1\n{}\n\
*MATERIAL, NAME=RUBBER\n{material}\
*SOLID SECTION, ELSET=EALL, MATERIAL=RUBBER\n{step}",
        set(0, 0),
        set(1, 0),
        set(2, 0),
        set(0, 1)
    )
    .unwrap();
    s
}

const STRETCH: &str = "*STEP, INC=200\n*STATIC\n0.1, 1\n*BOUNDARY\nX0, 1\nY0, 2\nZ0, 3\nX1, 1, 1, 1.0\n\
*NODE PRINT, NSET=X0, TOTALS=ONLY\nRF\n*END STEP\n";

/// Runs a job and returns the total x reactions at the end of the steps and the log.
fn run(dir: &Path, job: &str, input: &str) -> (Vec<f64>, String) {
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
    let dat = std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap();
    let mut totals = Vec::new();
    let mut take = false;
    for line in dat.lines() {
        if line.contains("total force") {
            take = true;
        } else if take && let Some(fx) = line.split_whitespace().next().and_then(|v| v.parse().ok())
        {
            totals.push(fx);
            take = false;
        }
    }
    (totals, log)
}

/// The analytic force of the compressible Neo-Hooke cube stretched by `stretch`.
fn neo_hooke_force(c10: f64, d1: f64, stretch: f64) -> f64 {
    let deviator = |t: f64| {
        let j = stretch * t * t;
        2.0 * c10 * j.powf(-5.0 / 3.0) * (stretch * stretch - t * t)
    };
    let residual = |t: f64| deviator(t) / 3.0 - 2.0 * (stretch * t * t - 1.0) / d1;
    let (mut lo, mut hi) = (0.1, 1.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if residual(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let t = 0.5 * (lo + hi);
    deviator(t) * t * t
}

#[test]
fn a_stretched_hyperelastic_cube_carries_the_analytic_force() {
    let dir = std::env::temp_dir().join(format!("plxs-hyperelastic-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let (c10, d1) = (0.5, 0.06);
    let expected = neo_hooke_force(c10, d1, 2.0);
    // The same energy written as four models: Neo-Hooke, Mooney-Rivlin with C01 = 0, a
    // one-term Ogden with alpha = 2 and mu = 2 C10, the reduced polynomial of order 1.
    for (job, material) in [
        ("neo", format!("*HYPERELASTIC, NEO HOOKE\n{c10}, {d1}\n")),
        (
            "mooney",
            format!("*HYPERELASTIC, MOONEY-RIVLIN\n{c10}, 0, {d1}\n"),
        ),
        (
            "ogden",
            format!("*HYPERELASTIC, OGDEN, N=1\n{}, 2, {d1}\n", 2.0 * c10),
        ),
        (
            "reduced",
            format!("*HYPERELASTIC, REDUCED POLYNOMIAL, N=1\n{c10}, {d1}\n"),
        ),
    ] {
        let (totals, log) = run(&dir, job, &cube(&material, STRETCH));
        assert!(log.contains("NLGEOM switched on"), "{job}:\n{log}");
        let force = -totals.last().copied().unwrap_or(f64::NAN);
        assert!(
            (force - expected).abs() < 1e-4 * expected,
            "{job}: force {force}, expected {expected}"
        );
    }
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn frequencies_of_a_hyperelastic_body_use_the_initial_moduli() {
    let dir = std::env::temp_dir().join(format!("plxs-hyper-frequency-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let step = "*STEP\n*FREQUENCY\n3\n*BOUNDARY\nX0, 1, 3\n*END STEP\n";
    // C10 = 0.5, nu = 0.45: mu = 1, K = 2 mu (1 + nu) / (3 (1 - 2 nu)), E = 2 mu (1 + nu).
    let nu: f64 = 0.45;
    let bulk = 2.0 * (1.0 + nu) / (3.0 * (1.0 - 2.0 * nu));
    let hyper = format!(
        "*HYPERELASTIC, NEO HOOKE\n0.5, {}\n*DENSITY\n1e-9\n",
        2.0 / bulk
    );
    let linear = format!("*ELASTIC\n{}, {nu}\n*DENSITY\n1e-9\n", 2.0 * (1.0 + nu));
    let frequencies = |job: &str, material: &str| -> Vec<f64> {
        let input = cube(material, step);
        std::fs::write(dir.join(format!("{job}.inp")), input).unwrap();
        let output = Command::new(plxs())
            .arg(job)
            .current_dir(&dir)
            .output()
            .unwrap();
        assert!(output.status.success(), "{job} failed");
        let dat = std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap();
        // Rows of the eigenvalue table: mode, eigenvalue, rad/time, cycles/time, imaginary.
        (dat.lines())
            .filter_map(|l| {
                let v: Vec<f64> = l
                    .split_whitespace()
                    .filter_map(|t| t.parse().ok())
                    .collect();
                (v.len() == 5 && v[0].fract() == 0.0 && v[0] >= 1.0).then(|| v[1])
            })
            .collect()
    };
    let (a, b) = (frequencies("hyper", &hyper), frequencies("linear", &linear));
    assert!(!a.is_empty() && a.len() == b.len(), "{a:?} vs {b:?}");
    for (x, y) in a.iter().zip(&b) {
        assert!((x - y).abs() < 1e-8 * y.abs(), "{a:?} vs {b:?}");
    }
    let _ = std::fs::remove_dir_all(&dir);
}
