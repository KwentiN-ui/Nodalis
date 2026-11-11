//! Ties on the faces and edges of native membranes (`M3D3`, `M3D4`): a membrane strip
//! pulled through solid blocks glued on its faces, and a strip glued with its edge to the
//! side of a block. The free part of the strip carries the whole force in uniaxial tension,
//! so it stretches by `F L / (E t w)`.

use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::Command;

const YOUNG: f64 = 210000.0;
const FORCE: f64 = 1000.0;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

/// Runs `input` and returns the log and the dat file.
fn run(job: &str, input: &str) -> (String, String) {
    let dir = std::env::temp_dir().join(format!("plxs-membrane-tie-{job}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
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
    let dat = std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap();
    if std::env::var_os("KEEP").is_none() {
        let _ = std::fs::remove_dir_all(&dir);
    }
    (log, dat)
}

/// The displacements of `node` in the last displacement table.
fn displacement(dat: &str, node: u32) -> [f64; 3] {
    let mut row = None;
    for line in dat.lines() {
        let v: Vec<f64> = line
            .split_whitespace()
            .filter_map(|v| v.parse().ok())
            .collect();
        if v.len() == 4 && v[0] == f64::from(node) {
            row = Some([v[1], v[2], v[3]]);
        }
    }
    row.unwrap_or_else(|| panic!("no displacement of node {node} in\n{dat}"))
}

/// Nodes of a grid in a plane z = const: id `start + i + (nx + 1) j`.
fn grid(s: &mut String, start: u32, nx: u32, ny: u32, x: [f64; 2], y: [f64; 2], z: f64) {
    for j in 0..=ny {
        for i in 0..=nx {
            let px = x[0] + (x[1] - x[0]) * f64::from(i) / f64::from(nx);
            let py = y[0] + (y[1] - y[0]) * f64::from(j) / f64::from(ny);
            writeln!(s, "{}, {px}, {py}, {z}", start + i + (nx + 1) * j).unwrap();
        }
    }
}

/// Hexahedra between two grids `lo` and `hi` of `n x n` cells.
fn hexes(s: &mut String, first: u32, n: u32, lo: u32, hi: u32) {
    let id = |base: u32, i: u32, j: u32| base + i + (n + 1) * j;
    for j in 0..n {
        for i in 0..n {
            let corners = |b| {
                [
                    id(b, i, j),
                    id(b, i + 1, j),
                    id(b, i + 1, j + 1),
                    id(b, i, j + 1),
                ]
            };
            let nodes: Vec<String> = (corners(lo).into_iter().chain(corners(hi)))
                .map(|n| n.to_string())
                .collect();
            writeln!(s, "{}, {}", first + i + n * j, nodes.join(", ")).unwrap();
        }
    }
}

/// A strip 20 × 10 × 1 in z = 0, held at x = 0, with a block 10 × 10 × 5 above and one
/// below on its half x > 10 (meshes not matching the strip's). The upper block's bottom
/// is the slave of the strip's face SPOS, the strip's face SNEG the slave of the lower
/// block's top. Both blocks are pulled at x = 20.
fn sandwich(kind: &str) -> String {
    let mut s = String::from("*Heading\nmembrane sandwich\n*Node\n");
    grid(&mut s, 1, 4, 2, [0.0, 20.0], [0.0, 10.0], 0.0);
    grid(&mut s, 101, 3, 3, [10.0, 20.0], [0.0, 10.0], 0.5);
    grid(&mut s, 201, 3, 3, [10.0, 20.0], [0.0, 10.0], 5.5);
    grid(&mut s, 301, 3, 3, [10.0, 20.0], [0.0, 10.0], -0.5);
    grid(&mut s, 401, 3, 3, [10.0, 20.0], [0.0, 10.0], -5.5);
    writeln!(s, "*Element, Type={kind}, Elset=STRIP").unwrap();
    let node = |i: u32, j: u32| 1 + i + 5 * j;
    let mut e = 1;
    let mut under = Vec::new();
    for j in 0..2 {
        for i in 0..4 {
            let q = [
                node(i, j),
                node(i + 1, j),
                node(i + 1, j + 1),
                node(i, j + 1),
            ];
            let mut write = |nodes: &[u32]| {
                let n: Vec<String> = nodes.iter().map(|n| n.to_string()).collect();
                writeln!(s, "{e}, {}", n.join(", ")).unwrap();
                if i >= 2 {
                    under.push(e.to_string());
                }
                e += 1;
            };
            if kind == "M3D3" {
                write(&[q[0], q[1], q[2]]);
                write(&[q[0], q[2], q[3]]);
            } else {
                write(&q);
            }
        }
    }
    s.push_str("*Element, Type=C3D8, Elset=UPPER\n");
    hexes(&mut s, 101, 3, 101, 201);
    s.push_str("*Element, Type=C3D8, Elset=LOWER\n");
    hexes(&mut s, 201, 3, 401, 301);
    // Flat: the strip before the blocks and the blocks' outer faces (a membrane holds
    // nothing across its plane, the strip's nodes under the blocks follow the blocks).
    let flat: Vec<String> = (1..=15)
        .filter(|n| (n - 1) % 5 < 2)
        .chain((201..217).chain(401..417))
        .map(|n| n.to_string())
        .collect();
    let load: Vec<String> = [101, 201, 301, 401]
        .iter()
        .flat_map(|b| (0..4).map(move |j| (b + 3 + 4 * j).to_string()))
        .collect();
    write!(
        s,
        "*Elset, Elset=UNDER\n{}\n\
         *Surface, Name=UPPER_BOTTOM, Type=Element\nUPPER, S1\n\
         *Surface, Name=LOWER_TOP, Type=Element\nLOWER, S2\n\
         *Surface, Name=STRIP_POS, Type=Element\nUNDER, SPOS\n\
         *Surface, Name=STRIP_NEG, Type=Element\nUNDER, SNEG\n\
         *Nset, Nset=HELD\n1, 6, 11\n*Nset, Nset=FLAT\n{}\n*Nset, Nset=LOAD\n{}\n\
         *Material, Name=STEEL\n*Elastic\n{YOUNG}, 0.3\n\
         *Membrane section, Elset=STRIP, Material=STEEL\n1\n\
         *Solid section, Elset=UPPER, Material=STEEL\n\
         *Solid section, Elset=LOWER, Material=STEEL\n\
         *Tie, Name=ON_TOP\nUPPER_BOTTOM, STRIP_POS\n\
         *Tie, Name=BELOW\nSTRIP_NEG, LOWER_TOP\n\
         *Step\n*Static\n*Boundary\nHELD, 1, 1, 0\n1, 2, 2, 0\nFLAT, 3, 3, 0\n\
         *Cload\nLOAD, 1, {}\n*Node print, Nset=NALL\nU\n*End step\n",
        under.join(", "),
        flat.join(", "),
        load.join(", "),
        FORCE / load.len() as f64,
    )
    .unwrap();
    s.replace("*Node\n", "*Node, Nset=NALL\n")
}

fn stretch() -> f64 {
    FORCE * 10.0 / (YOUNG * 1.0 * 10.0)
}

#[test]
fn blocks_glued_on_both_faces_of_a_membrane_pull_it() {
    for kind in ["M3D4", "M3D3"] {
        let (log, dat) = run(&format!("sandwich-{kind}"), &sandwich(kind));
        assert!(!log.contains("not tied"), "{kind}:\n{log}");
        // The corners of the strip at x = 10 and the blocks' nodes above and below them
        // move alike.
        for (strip, block) in [(3, 101), (3, 301), (13, 113), (13, 313), (15, 116)] {
            let (s, u) = (displacement(&dat, strip), displacement(&dat, block));
            for d in 0..3 {
                assert!((u[d] - s[d]).abs() < 1e-9, "{kind} {block}: {u:?} {s:?}");
            }
        }
        // The free half stretches as a bar; the blocks hold the strip's contraction
        // near x = 10, hence the tolerance.
        let stretched = (1..=15)
            .filter(|n| (n - 1) % 5 == 2)
            .map(|n| displacement(&dat, n)[0])
            .sum::<f64>()
            / 3.0;
        assert!(
            (stretched - stretch()).abs() < 0.03 * stretch(),
            "{kind}: {stretched} against {}",
            stretch()
        );
    }
}

/// A strip 10 × 10 × 1 whose edge x = 10 (faces S4) is the master of the side x = 10 of a
/// block 4 × 10 × 1 held at x = 14; the strip is pulled at x = 0.
fn edge() -> String {
    let mut s = String::from("*Heading\nmembrane edge\n*Node, Nset=NALL\n");
    grid(&mut s, 1, 2, 2, [0.0, 10.0], [0.0, 10.0], 0.0);
    grid(&mut s, 101, 2, 3, [10.0, 14.0], [0.0, 10.0], -0.5);
    grid(&mut s, 201, 2, 3, [10.0, 14.0], [0.0, 10.0], 0.5);
    s.push_str("*Element, Type=M3D4, Elset=STRIP\n");
    let node = |i: u32, j: u32| 1 + i + 3 * j;
    for j in 0..2 {
        for i in 0..2 {
            writeln!(
                s,
                "{}, {}, {}, {}, {}",
                1 + i + 2 * j,
                node(i, j),
                node(i + 1, j),
                node(i + 1, j + 1),
                node(i, j + 1)
            )
            .unwrap();
        }
    }
    s.push_str("*Element, Type=C3D8, Elset=BLOCK\n");
    let id = |b: u32, i: u32, j: u32| b + i + 3 * j;
    for j in 0..3 {
        for i in 0..2 {
            let c = |b| {
                [
                    id(b, i, j),
                    id(b, i + 1, j),
                    id(b, i + 1, j + 1),
                    id(b, i, j + 1),
                ]
            };
            let nodes: Vec<String> = (c(101).into_iter().chain(c(201)))
                .map(|n| n.to_string())
                .collect();
            writeln!(s, "{}, {}", 101 + i + 2 * j, nodes.join(", ")).unwrap();
        }
    }
    let held: Vec<String> = [101, 201]
        .iter()
        .flat_map(|b| (0..4).map(move |j| (b + 2 + 3 * j).to_string()))
        .collect();
    write!(
        s,
        "*Elset, Elset=STRIP_EDGE\n2, 4\n*Elset, Elset=BLOCK_SIDE\n101, 103, 105\n\
         *Surface, Name=MASTER, Type=Element\nSTRIP_EDGE, S4\n\
         *Surface, Name=SLAVE, Type=Element\nBLOCK_SIDE, S6\n\
         *Nset, Nset=HELD\n{}\n*Nset, Nset=PULLED\n1, 4, 7\n\
         *Nset, Nset=STRIP\n1, 2, 3, 4, 5, 6, 7, 8, 9\n\
         *Material, Name=STEEL\n*Elastic\n{YOUNG}, 0.3\n\
         *Membrane section, Elset=STRIP, Material=STEEL\n1\n\
         *Solid section, Elset=BLOCK, Material=STEEL\n\
         *Tie, Name=EDGE\nSLAVE, MASTER\n\
         *Step\n*Static\n*Boundary\nHELD, 1, 3, 0\nSTRIP, 3, 3, 0\n\
         *Cload\n1, 1, {q}\n4, 1, {h}\n7, 1, {q}\n*Node print, Nset=NALL\nU\n*End step\n",
        held.join(", "),
        q = -FORCE / 4.0,
        h = -FORCE / 2.0,
    )
    .unwrap();
    s
}

#[test]
fn a_membrane_edge_holds_the_side_of_a_block() {
    let (log, dat) = run("edge", &edge());
    assert!(!log.contains("not tied"), "{log}");
    // The block's nodes above and below the strip's edge node 6 (x = 10, y = 5) follow it
    // in all directions: an edge of a membrane is tied in 3D, not only in a plane.
    let at_edge = displacement(&dat, 6);
    let block = |b: u32, j: u32| displacement(&dat, b + 3 * j);
    for (b, j) in [(101, 0), (201, 0), (101, 3), (201, 3)] {
        let corner = displacement(&dat, if j == 0 { 3 } else { 9 });
        let u = block(b, j);
        for d in 0..3 {
            assert!((u[d] - corner[d]).abs() < 1e-9, "{b} {j}: {u:?} {corner:?}");
        }
    }
    assert!(at_edge[0] < 0.0, "{at_edge:?}");
    let stretched = [(1, 3), (4, 6), (7, 9)]
        .iter()
        .map(|&(free, tied)| displacement(&dat, tied)[0] - displacement(&dat, free)[0])
        .sum::<f64>()
        / 3.0;
    assert!(
        (stretched - stretch()).abs() < 0.03 * stretch(),
        "{stretched} against {}",
        stretch()
    );
}
