//! `*TIE` on plane and axisymmetric elements. CalculiX cannot serve as the reference: version
//! 2.23 leaves the expanded slave nodes of a 2D tie (partly) uncoupled. Instead:
//!
//! * linear elements: the displacements equal those of the same model extruded into solids
//!   with the out-of-plane displacement held (plane strain exactly), whose tie is verified
//!   against CalculiX in the fixture corpus. With quadratic faces the extrusion is no exact
//!   reference: the tie of its midplane nodes lets the solids vary across the thickness.
//! * patch test, linear and quadratic, plane strain and axisymmetric: under uniaxial stress
//!   the exact solution is linear, every element reproduces it, so the tie must as well.
//!
//! Two non-matching meshes: a master block A (x 1..11, y 0..4, 4 × 2 quadrilaterals) on a
//! slave block B (x 3..9, y -6..0, 3 × 3 cells split into triangles), B's bottom held, A's
//! top displaced.

use std::collections::HashMap;

use plxs_core::progress::NoProgress;
use plxs_session::{JobOptions, run_job_keeping};

#[derive(Clone, Copy)]
struct Mesh {
    quadratic: bool,
}

/// Grid node id of a block: `base + i + (m·nx + 1)·j`, `m` = 2 for quadratic meshes.
fn grid(base: u32, nx: usize, m: usize) -> impl Fn(usize, usize) -> u32 {
    move |i, j| base + (i + (m * nx + 1) * j) as u32
}

/// Grid nodes of A (`base` 1) and B (`base` 1001) with their coordinates.
fn nodes(mesh: Mesh) -> Vec<(u32, f64, f64)> {
    let m = if mesh.quadratic { 2 } else { 1 };
    let mut out = Vec::new();
    for (base, x0, x1, y0, y1, nx, ny) in [
        (1, 1.0, 11.0, 0.0, 4.0, 4, 2),
        (1001, 3.0, 9.0, -6.0, 0.0, 3, 3),
    ] {
        let id = grid(base, nx, m);
        for j in 0..=m * ny {
            for i in 0..=m * nx {
                let x = x0 + (x1 - x0) * i as f64 / (m * nx) as f64;
                let y = y0 + (y1 - y0) * j as f64 / (m * ny) as f64;
                out.push((id(i, j), x, y));
            }
        }
    }
    out
}

/// The 2D elements: (id, A or B, nodes in CalculiX order).
fn elements(mesh: Mesh) -> Vec<(u32, bool, Vec<u32>)> {
    let m = if mesh.quadratic { 2 } else { 1 };
    let mut out = Vec::new();
    let a = grid(1, 4, m);
    for j in 0..2 {
        for i in 0..4 {
            let (i, j) = (m * i, m * j);
            let mut n = vec![a(i, j), a(i + m, j), a(i + m, j + m), a(i, j + m)];
            if mesh.quadratic {
                n.extend([a(i + 1, j), a(i + 2, j + 1), a(i + 1, j + 2), a(i, j + 1)]);
            }
            out.push((out.len() as u32 + 1, true, n));
        }
    }
    let b = grid(1001, 3, m);
    for j in 0..3 {
        for i in 0..3 {
            let (i, j) = (m * i, m * j);
            let (p, q, r, s) = (b(i, j), b(i + m, j), b(i + m, j + m), b(i, j + m));
            let (mut t1, mut t2) = (vec![p, q, r], vec![p, r, s]);
            if mesh.quadratic {
                t1.extend([b(i + 1, j), b(i + 2, j + 1), b(i + 1, j + 1)]);
                t2.extend([b(i + 1, j + 1), b(i + 1, j + 2), b(i, j + 1)]);
            }
            out.push((101 + out.len() as u32, false, t1));
            out.push((101 + out.len() as u32, false, t2));
        }
    }
    out
}

fn sets(mesh: Mesh, layers: &[u32]) -> String {
    let m = if mesh.quadratic { 2 } else { 1 };
    let (a, b) = (grid(1, 4, m), grid(1001, 3, m));
    let mut s = String::from("*NSET, NSET=FIX\n");
    for &l in layers {
        for i in 0..=3 * m {
            s += &format!("{},\n", b(i, 0) + l);
        }
    }
    s += "*NSET, NSET=TOP\n";
    for &l in layers {
        for i in 0..=4 * m {
            s += &format!("{},\n", a(i, 2 * m) + l);
        }
    }
    s
}

const MATERIALS: &str = "*MATERIAL, NAME=STEEL\n*ELASTIC\n210000., 0.3\n\
*MATERIAL, NAME=ALU\n*ELASTIC\n70000., 0.33\n";

const STEP: &str = "*TIE, NAME=T1\nSLAVE, MASTER\n*STEP\n*STATIC\n*BOUNDARY\nFIX, 1, 2\n\
TOP, 1, 1, 0.01\nTOP, 2, 2, -0.02\n*NODE FILE\nU\n*END STEP\n";

fn plane(mesh: Mesh) -> String {
    plane_with(mesh, "CPE", MATERIALS, STEP)
}

/// The plane model with elements `<prefix>4/3` or `<prefix>8/6`.
fn plane_with(mesh: Mesh, prefix: &str, materials: &str, step: &str) -> String {
    let (quad, tri) = if mesh.quadratic {
        (format!("{prefix}8"), format!("{prefix}6"))
    } else {
        (format!("{prefix}4"), format!("{prefix}3"))
    };
    let mut s = String::from("*NODE\n");
    for (id, x, y) in nodes(mesh) {
        s += &format!("{id}, {x}, {y}, 0\n");
    }
    let all = elements(mesh);
    for (kind, set, in_a) in [(quad, "A", true), (tri, "B", false)] {
        s += &format!("*ELEMENT, TYPE={kind}, ELSET={set}\n");
        for (id, _, n) in all.iter().filter(|e| e.1 == in_a) {
            s += &format!("{id}, {}\n", join(n));
        }
    }
    s += &sets(mesh, &[0]);
    // A's bottom edges (S1: corners 1-2) are the master, B's top edges the slave: the
    // second triangle of each top cell, edge S2 (corners 2-3).
    s += "*SURFACE, NAME=MASTER, TYPE=ELEMENT\n1, S1\n2, S1\n3, S1\n4, S1\n";
    s += "*SURFACE, NAME=SLAVE, TYPE=ELEMENT\n";
    for (id, a, _) in all.iter().rev().take(6).step_by(2) {
        assert!(!a);
        s += &format!("{id}, S2\n");
    }
    s += materials;
    if prefix == "CAX" {
        s += "*SOLID SECTION, ELSET=A, MATERIAL=STEEL\n*SOLID SECTION, ELSET=B, MATERIAL=ALU\n";
    } else {
        s += "*SOLID SECTION, ELSET=A, MATERIAL=STEEL\n1.\n*SOLID SECTION, ELSET=B, MATERIAL=ALU\n1.\n";
    }
    s + step
}

/// Offset of the back layer of the extrusion.
const BACK: u32 = 10000;

/// The linear model extruded by 1 along z: C3D8 and C3D6.
fn extruded() -> String {
    let mesh = Mesh { quadratic: false };
    let mut s = String::from("*NODE\n");
    for (id, x, y) in nodes(mesh) {
        s += &format!("{id}, {x}, {y}, 0\n{}, {x}, {y}, 1\n", id + BACK);
    }
    let all = elements(mesh);
    for (kind, set, in_a) in [("C3D8", "A", true), ("C3D6", "B", false)] {
        s += &format!("*ELEMENT, TYPE={kind}, ELSET={set}\n");
        for (id, _, n) in all.iter().filter(|e| e.1 == in_a) {
            let back: Vec<u32> = n.iter().map(|v| v + BACK).collect();
            s += &format!("{id}, {}, {}\n", join(n), join(&back));
        }
    }
    s += &sets(mesh, &[0, BACK]);
    s += "*NSET, NSET=ALL\n";
    for (id, _, _) in nodes(mesh) {
        s += &format!("{id},\n{},\n", id + BACK);
    }
    // A's bottom faces: S3 (nodes 1-2-6-5) of the hexahedra; B's top nodes the slave.
    s += "*SURFACE, NAME=MASTER, TYPE=ELEMENT\n1, S3\n2, S3\n3, S3\n4, S3\n";
    s += "*SURFACE, NAME=SLAVE, TYPE=NODE\n";
    let b = grid(1001, 3, 1);
    for layer in [0, BACK] {
        for i in 0..=3 {
            s += &format!("{}\n", b(i, 3) + layer);
        }
    }
    s += MATERIALS;
    s += "*SOLID SECTION, ELSET=A, MATERIAL=STEEL\n*SOLID SECTION, ELSET=B, MATERIAL=ALU\n";
    s + &STEP.replace("FIX, 1, 2\n", "FIX, 1, 2\nALL, 3, 3\n")
}

fn join(n: &[u32]) -> String {
    n.iter().map(u32::to_string).collect::<Vec<_>>().join(", ")
}

/// Displacements (u1, u2) by node id.
fn solve(input: &str) -> HashMap<u32, [f64; 2]> {
    let dir = std::env::temp_dir();
    let options = JobOptions {
        threads: 1,
        write_files: false,
        ..JobOptions::new(&dir, "tie-2d")
    };
    let mut kept = None;
    run_job_keeping(input, None, &options, &mut NoProgress, &mut kept)
        .unwrap_or_else(|e| panic!("{e}\n{input}"));
    let (_, results) = kept.expect("results");
    let field = (results.increments.last().unwrap().fields.iter())
        .find(|f| f.name == "DISP")
        .unwrap();
    (field.nodes.iter().enumerate())
        .map(|(k, &id)| (id, [field.values[0][k], field.values[1][k]]))
        .collect()
}

#[test]
fn linear_plane_tie_matches_the_extruded_solid() {
    let mesh = Mesh { quadratic: false };
    let flat = solve(&plane(mesh));
    let solid = solve(&extruded());
    let scale = flat
        .values()
        .map(|u| u[0].abs().max(u[1].abs()))
        .fold(0.0, f64::max);
    assert!(scale > 1e-3, "nothing moved: {scale}");
    // B's top moves with A: the tie holds (a free B would stay at rest).
    let moved = flat[&grid(1001, 3, 1)(1, 3)];
    assert!(moved[1] < -1e-3, "{moved:?}");
    for (id, u) in &flat {
        for layer in [0, BACK] {
            let v = solid[&(id + layer)];
            for d in 0..2 {
                assert!(
                    (u[d] - v[d]).abs() <= 1e-6 * scale,
                    "node {id} (layer {layer}) u{}: plane {} vs extruded {}",
                    d + 1,
                    u[d],
                    v[d]
                );
            }
        }
    }
}

/// Uniaxial stress `σ` along y in both blocks (one material): the exact displacements.
fn uniaxial(prefix: &str, x: f64, y: f64) -> [f64; 2] {
    let (e, nu, sigma) = (210000.0, 0.3, -50.0);
    let (exx, eyy) = if prefix == "CAX" {
        (-nu * sigma / e, sigma / e)
    } else {
        (-nu * (1.0 + nu) * sigma / e, (1.0 - nu * nu) * sigma / e)
    };
    [exx * x, eyy * y]
}

/// The patch test: A held at the exact solution, B's bottom held in y (and x at the axis
/// side for plane strain); B's other nodes, the slave nodes among them, must follow.
fn patch(mesh: Mesh, prefix: &str) {
    let same = "*MATERIAL, NAME=STEEL\n*ELASTIC\n210000., 0.3\n\
*MATERIAL, NAME=ALU\n*ELASTIC\n210000., 0.3\n";
    let all = nodes(mesh);
    let mut step = String::from("*TIE, NAME=T1\nSLAVE, MASTER\n*STEP\n*STATIC\n*BOUNDARY\n");
    let bottom = all
        .iter()
        .filter(|n| n.0 > 1000 && n.2 == -6.0)
        .map(|n| n.0)
        .collect::<Vec<_>>();
    for &(id, x, y) in &all {
        let u = uniaxial(prefix, x, y);
        if id < 1000 {
            step += &format!("{id}, 1, 1, {}\n{id}, 2, 2, {}\n", u[0], u[1]);
        } else if bottom.contains(&id) {
            step += &format!("{id}, 2, 2, {}\n", u[1]);
            if id == bottom[0] {
                step += &format!("{id}, 1, 1, {}\n", u[0]);
            }
        }
    }
    step += "*NODE FILE\nU\n*END STEP\n";
    let result = solve(&plane_with(mesh, prefix, same, &step));
    let scale = 1e-3;
    // Element nodes only: the centres of A's quadratic cells belong to no element.
    let used: std::collections::HashSet<u32> =
        elements(mesh).into_iter().flat_map(|e| e.2).collect();
    // The axisymmetric expansion reports the radial displacement as the x displacement of
    // its nodes at ±1° (averaged with the midplane node where there is one): up to cos 1°
    // times the radial displacement, prescribed ones too. The axial displacement is exact.
    let radial = if prefix == "CAX" { 2e-4 } else { 1e-12 };
    for &(id, x, y) in all.iter().filter(|n| used.contains(&n.0)) {
        let exact = uniaxial(prefix, x, y);
        let u = result[&id];
        for d in 0..2 {
            let tolerance = if d == 0 {
                radial * exact[0].abs() + 1e-12 * scale
            } else {
                1e-9 * scale
            };
            assert!(
                (u[d] - exact[d]).abs() <= tolerance,
                "{prefix} node {id} u{}: {} vs exact {}",
                d + 1,
                u[d],
                exact[d]
            );
        }
    }
}

#[test]
fn plane_ties_pass_the_patch_test() {
    for quadratic in [false, true] {
        patch(Mesh { quadratic }, "CPE");
    }
}

#[test]
fn axisymmetric_ties_pass_the_patch_test() {
    for quadratic in [false, true] {
        patch(Mesh { quadratic }, "CAX");
    }
}
