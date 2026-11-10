//! General beams (U1) under large deformations: the corotational beam of
//! `plxs_elements::corotational`, checked against closed-form solutions.
//!
//! - A cantilever rolled up by an end moment `M = π EI / L` into a half circle (the tip ends
//!   at `x = 0`, `y = 2L/π`).
//! - A rope pulled by `T` between two supports `2a` apart and pushed sideways by `δ` in the
//!   middle needs `F = 2 T δ / (a - λ tanh(a/λ))` with `λ = sqrt(EI / T)`, more than the
//!   ideal rope's `2 T δ / a` (the formula of the wire rope report).
//! - The eigenfrequencies of a beam under tension `T`, pinned at both ends:
//!   `f_n = n / (2L) sqrt(T / (ρA)) sqrt(1 + n²π² EI / (T L²))`.
//! - The buckling load of a cantilever column, `π² EI / (4 L²)`.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

fn work_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("plxs-rope-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// A straight beam of `n` U1 elements along x from 0 to `length`, nodes 1 to n + 1 in the
/// set NALL, with the section values.
fn beam(n: usize, length: f64, young: f64, area: f64, inertia: f64, density: f64) -> String {
    let mut s = String::from("*Node, Nset=NALL\n");
    for i in 0..=n {
        writeln!(s, "{}, {}, 0, 0", i + 1, length * i as f64 / n as f64).unwrap();
    }
    s.push_str(
        "*User element, Type=U1, Nodes=2, Integration points=2, Maxdof=6\n\
         *Element, Type=U1, Elset=BEAM\n",
    );
    for i in 0..n {
        writeln!(s, "{}, {}, {}", i + 1, i + 1, i + 2).unwrap();
    }
    write!(
        s,
        "*Material, Name=ROPE\n*Elastic\n{young}, 0.3\n*Density\n{density}\n\
         *Beam section, Elset=BEAM, Material=ROPE, Section=GENERAL\n\
         {area}, {inertia}, 0, {inertia}, 0.8333333\n0, 0, 1\n"
    )
    .unwrap();
    s
}

/// Runs `input` and returns the dat file.
fn run(job: &str, input: &str) -> String {
    let dir = work_dir(job);
    std::fs::write(dir.join(format!("{job}.inp")), input).unwrap();
    let output = Command::new(plxs())
        .arg(job)
        .arg("--threads")
        .arg("2")
        .current_dir(&dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned();
    assert!(output.status.success(), "{job} failed:\n{log}");
    assert!(log.contains("Job finished"), "{job}:\n{log}");
    read(&dir, job)
}

fn read(dir: &Path, job: &str) -> String {
    std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap()
}

/// The values of `node` in the last table whose title contains `title`.
fn last_row(dat: &str, title: &str, node: usize) -> Vec<f64> {
    let mut row = None;
    let mut inside = false;
    for line in dat.lines() {
        if line.contains(title) {
            inside = true;
            continue;
        }
        let values: Vec<f64> = line
            .split_whitespace()
            .filter_map(|v| v.parse().ok())
            .collect();
        if inside && values.first() == Some(&(node as f64)) {
            row = Some(values[1..].to_vec());
            inside = false;
        }
    }
    row.unwrap_or_else(|| panic!("no {title} of node {node} in\n{dat}"))
}

/// The cycles per time of the modes in the eigenvalue output.
fn frequencies(dat: &str) -> Vec<f64> {
    let start = dat.find("E I G E N V A L U E").expect("eigenvalue output");
    (dat[start..].lines())
        .skip(6)
        .map_while(|line| {
            let v: Vec<f64> = line
                .split_whitespace()
                .filter_map(|v| v.parse().ok())
                .collect();
            (v.len() == 5).then(|| v[3])
        })
        .collect()
}

#[test]
fn an_end_moment_rolls_a_cantilever_into_a_half_circle() {
    let (n, length, young, inertia) = (20, 1000.0, 210000.0, 1000.0);
    let moment = std::f64::consts::PI * young * inertia / length;
    let input = format!(
        "{}*Boundary\n1, 1, 6, 0\n*Step, Nlgeom\n*Static\n0.1, 1\n*Cload\n{}, 6, {moment}\n\
         *Node print, Nset=NALL\nU\n*End step\n",
        beam(n, length, young, 100.0, inertia, 7.85e-9),
        n + 1
    );
    let tip = last_row(&run("circle", &input), "displacements", n + 1);
    assert!((tip[0] + length).abs() < 1e-3 * length, "{tip:?}");
    let up = 2.0 * length / std::f64::consts::PI;
    assert!((tip[1] - up).abs() < 3e-3 * up, "{tip:?}");
}

#[test]
fn bending_stiffness_raises_the_force_to_deflect_a_rope_by_lambda_over_a() {
    // The example of the report: 20 kN, EA = 1.1e7 N, EI = 2e7 N mm², a = 150 mm.
    let (a, n, tension, young) = (150.0, 80, 20000.0, 1.1e5);
    let (area, inertia) = (1.1e7 / young, 2e7 / young);
    let delta = 2.0;
    let middle = n / 2 + 1;
    let input = format!(
        "{}*Boundary\nNALL, 3, 5, 0\n1, 1, 2, 0\n{end}, 2, 2, 0\n\
         *Step, Nlgeom\n*Static\n*Cload\n{end}, 1, {tension}\n*End step\n\
         *Step, Nlgeom\n*Static\n0.25, 1\n*Boundary\n{middle}, 2, 2, {}\n\
         *Node print, Nset=NALL\nRF\n*End step\n",
        beam(n, 2.0 * a, young, area, inertia, 7.85e-9),
        -delta,
        end = n + 1,
    );
    let force = -last_row(&run("three-point", &input), "forces", middle)[1];
    let lambda = (young * inertia / tension).sqrt();
    let expected = 2.0 * tension * delta / (a - lambda * (a / lambda).tanh());
    assert!(
        (force - expected).abs() < 5e-3 * expected,
        "{force} vs {expected}"
    );
    // 27 % above the ideal rope.
    assert!(force > 1.25 * 2.0 * tension * delta / a);
}

#[test]
fn tension_raises_the_eigenfrequencies_of_a_pinned_beam() {
    let (n, length, young, area, inertia, density) = (40, 1000.0, 210000.0, 100.0, 1000.0, 7.85e-9);
    let tension = 1000.0;
    let input = format!(
        "{}*Boundary\nNALL, 3, 5, 0\n1, 1, 2, 0\n{end}, 2, 2, 0\n\
         *Step, Nlgeom\n*Static\n*Cload\n{end}, 1, {tension}\n*End step\n\
         *Step, Perturbation\n*Frequency\n3\n*End step\n",
        beam(n, length, young, area, inertia, density),
        end = n + 1,
    );
    let found = frequencies(&run("string", &input));
    for (k, f) in found.iter().enumerate() {
        let k = (k + 1) as f64;
        let pi = std::f64::consts::PI;
        let expected = k / (2.0 * length)
            * (tension / (density * area)).sqrt()
            * (1.0 + k * k * pi * pi * young * inertia / (tension * length * length)).sqrt();
        assert!(
            (f - expected).abs() < 1e-2 * expected,
            "mode {k}: {f} vs {expected}"
        );
    }
    assert_eq!(found.len(), 3);
}

#[test]
fn a_cantilever_column_buckles_at_the_euler_load() {
    let (n, length, young, inertia) = (40, 1000.0, 210000.0, 1000.0);
    let input = format!(
        "{}*Boundary\n1, 1, 6, 0\n*Step\n*Buckle\n2\n*Cload\n{}, 1, -1\n*End step\n",
        beam(n, length, young, 100.0, inertia, 7.85e-9),
        n + 1
    );
    let dat = run("buckle", &input);
    let start = dat.find("B U C K L I N G").expect("buckling output");
    let factor: f64 = dat[start..]
        .lines()
        .find_map(|l| {
            let v: Vec<f64> = l
                .split_whitespace()
                .filter_map(|v| v.parse().ok())
                .collect();
            (v.len() == 2 && v[0] == 1.0).then(|| v[1])
        })
        .unwrap();
    let euler = std::f64::consts::PI.powi(2) * young * inertia / (4.0 * length * length);
    assert!((factor - euler).abs() < 1e-3 * euler, "{factor} vs {euler}");
}

/// The rows of the last block `name` of the frd file in `dir`: node and values.
fn frd_block(dir: &Path, job: &str, name: &str) -> Vec<(usize, Vec<f64>)> {
    let frd = std::fs::read_to_string(dir.join(format!("{job}.frd"))).unwrap();
    let lines: Vec<&str> = frd.lines().collect();
    let start = (lines.iter())
        .rposition(|l| l.starts_with(&format!(" -4  {name} ")))
        .unwrap_or_else(|| panic!("no {name} in the frd file"));
    (lines[start + 1..].iter())
        .take_while(|l| !l.starts_with(" -3"))
        .filter(|l| l.starts_with(" -1"))
        .map(|l| {
            let node = l[3..13].trim().parse().unwrap();
            let values = (l[13..].as_bytes().chunks(12))
                .map(|c| std::str::from_utf8(c).unwrap().trim().parse().unwrap())
                .collect();
            (node, values)
        })
        .collect()
}

#[test]
fn a_beam_writes_its_normal_stress_and_section_forces() {
    // A cantilever pulled along its axis and loaded across at the tip: N / A along it in
    // STRESS, and in SECFORC the normal force and the bending moment Q (L - x).
    let (n, length, area, pull, across) = (10, 1000.0, 100.0, 1000.0, 1.0);
    let input = format!(
        "{}*Boundary\n1, 1, 6, 0\n*Step\n*Static\n*Cload\n{tip}, 1, {pull}\n{tip}, 2, {across}\n\
         *Node file\nU\n*El file\nS\n*End step\n",
        beam(n, length, 210000.0, area, 1000.0, 7.85e-9),
        tip = n + 1
    );
    let job = "section-forces";
    let dir = work_dir(job);
    std::fs::write(dir.join(format!("{job}.inp")), input).unwrap();
    let output = Command::new(plxs())
        .arg(job)
        .current_dir(&dir)
        .output()
        .unwrap();
    assert!(output.status.success());
    let stress = frd_block(&dir, job, "STRESS");
    let forces = frd_block(&dir, job, "SECFORC");
    assert_eq!(forces.len(), n + 1);
    for (node, s) in &stress {
        assert!(
            (s[0] - pull / area).abs() < 1e-3 * pull / area,
            "{node}: {s:?}"
        );
    }
    for (node, f) in &forces {
        let x = length * (*node - 1) as f64 / n as f64;
        assert!((f[0] - pull).abs() < 1e-3 * pull, "{node}: {f:?}");
        let moment = across * (length - x);
        assert!(
            (f[1] - moment).abs() < 1e-3 * across * length,
            "{node}: {f:?} vs {moment}"
        );
    }
}
