//! The normals of shell elements at a node are averaged when they lie within 20° of each
//! other (the CalculiX manual's rule for normals computed from the geometry; 0.5° holds
//! only for normals given by `*NORMAL`). A strip of `S4` folded by a few degrees from
//! element to element, as a coarse mesh of a curved shell is, expands into one set of
//! nodes along the normal per node; a sharper fold becomes a knot with one set per element.

use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

/// Number of nodes in the `.frd` of a cantilever strip of `columns` elements whose
/// segments turn by `fold` degrees each (about the y axis), clamped at the first two
/// columns of nodes and loaded at the tip.
fn expanded_nodes(job: &str, columns: u32, fold: f64) -> usize {
    let mut s = String::from("*NODE\n");
    let (mut x, mut z, mut angle) = (0.0f64, 0.0f64, 0.0f64);
    for i in 0..=columns {
        for j in 0..2u32 {
            writeln!(s, "{}, {x}, {}, {z}", 1 + i * 2 + j, f64::from(j) * 10.0).unwrap();
        }
        x += 10.0 * angle.to_radians().cos();
        z += 10.0 * angle.to_radians().sin();
        angle += fold;
    }
    s.push_str("*ELEMENT, TYPE=S4, ELSET=STRIP\n");
    for i in 0..columns {
        let a = 1 + i * 2;
        writeln!(s, "{}, {}, {}, {}, {}", i + 1, a, a + 2, a + 3, a + 1).unwrap();
    }
    let tip = 1 + columns * 2;
    write!(
        s,
        "*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n\
         *SHELL SECTION, ELSET=STRIP, MATERIAL=STEEL\n1.0\n\
         *NSET, NSET=FIXED\n1, 2, 3, 4\n\
         *STEP\n*STATIC\n*BOUNDARY\nFIXED, 1, 3\n\
         *CLOAD\n{tip}, 3, 1.0\n{}, 3, 1.0\n\
         *NODE FILE\nU\n*END STEP\n",
        tip + 1
    )
    .unwrap();
    let dir = std::env::temp_dir().join(format!("plxs-shell-normals-{job}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(format!("{job}.inp")), s).unwrap();
    let output = Command::new(plxs())
        .arg(job)
        .arg("--threads")
        .arg("2")
        .current_dir(&dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned();
    assert!(output.status.success(), "{job} failed:\n{log}");
    let frd = std::fs::read_to_string(dir.join(format!("{job}.frd"))).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    let block = frd
        .split("\n    2C")
        .nth(1)
        .expect("a node block")
        .split("\n -3")
        .next()
        .unwrap();
    block.lines().filter(|l| l.starts_with(" -1")).count()
}

#[test]
fn slightly_folded_elements_share_one_normal() {
    // 6 elements, 14 nodes, each turned by 5° against its neighbour: no knot, every node
    // expands into a bottom and a top node (S4 becomes C3D8I).
    assert_eq!(expanded_nodes("gentle", 6, 5.0), 14 * 2);
}

#[test]
fn sharp_folds_become_knots() {
    // Turned by 30° each: the 10 nodes between two elements expand once per element.
    assert_eq!(expanded_nodes("sharp", 6, 30.0), 14 * 2 + 10 * 2);
}
