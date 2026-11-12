//! The iterative solver on quadratic elements: its multigrid coarsens the mesh to the corner
//! nodes first. With aggregates of the quadratic mesh itself a block of 20-node bricks took
//! twice the iterations of the same block in 8-node bricks (40 on this one), and a part
//! swept into bricks over a million unknowns four times those of its tetrahedra.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

const CELLS: usize = 10;
const SIDE: f64 = 100.0;

/// A steel cube of `CELLS`³ C3D20, clamped at x = 0 and pulled sideways at its far face.
fn cube() -> String {
    // Nodes on the grid of half cells; a brick has none at its face and body centres.
    let mut ids: BTreeMap<[usize; 3], usize> = BTreeMap::new();
    let mut s = String::from("*NODE\n");
    let n = 2 * CELLS;
    for k in 0..=n {
        for j in 0..=n {
            for i in 0..=n {
                if [i, j, k].iter().filter(|&&c| c % 2 == 1).count() > 1 {
                    continue;
                }
                let id = ids.len() + 1;
                ids.insert([i, j, k], id);
                let at = |c: usize| SIDE * c as f64 / n as f64;
                writeln!(s, "{id}, {}, {}, {}", at(i), at(j), at(k)).unwrap();
            }
        }
    }
    // Corners, then the midside nodes of the bottom, the top and the vertical edges.
    const LOCAL: [[usize; 3]; 20] = [
        [0, 0, 0],
        [2, 0, 0],
        [2, 2, 0],
        [0, 2, 0],
        [0, 0, 2],
        [2, 0, 2],
        [2, 2, 2],
        [0, 2, 2],
        [1, 0, 0],
        [2, 1, 0],
        [1, 2, 0],
        [0, 1, 0],
        [1, 0, 2],
        [2, 1, 2],
        [1, 2, 2],
        [0, 1, 2],
        [0, 0, 1],
        [2, 0, 1],
        [2, 2, 1],
        [0, 2, 1],
    ];
    s.push_str("*ELEMENT, TYPE=C3D20, ELSET=EALL\n");
    let mut element = 0;
    for k in 0..CELLS {
        for j in 0..CELLS {
            for i in 0..CELLS {
                element += 1;
                let nodes: Vec<String> = LOCAL
                    .iter()
                    .map(|l| ids[&[2 * i + l[0], 2 * j + l[1], 2 * k + l[2]]].to_string())
                    .collect();
                writeln!(s, "{element}, {},", nodes[..10].join(", ")).unwrap();
                writeln!(s, "{}", nodes[10..].join(", ")).unwrap();
            }
        }
    }
    let face = |x: usize| -> Vec<String> {
        (ids.iter().filter(|(at, _)| at[0] == x))
            .map(|(_, id)| id.to_string())
            .collect()
    };
    let far = face(n);
    writeln!(s, "*NSET, NSET=FIX\n{}", face(0).join(",\n")).unwrap();
    writeln!(s, "*NSET, NSET=FAR\n{}", far.join(",\n")).unwrap();
    s.push_str("*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n");
    s.push_str("*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL\n*BOUNDARY\nFIX, 1, 3\n");
    s.push_str("*STEP\n*STATIC, SOLVER=ITERATIVE CHOLESKY\n");
    writeln!(s, "*CLOAD\nFAR, 3, {}", -1000.0 / far.len() as f64).unwrap();
    s.push_str("*NODE FILE\nU\n*END STEP\n");
    s
}

#[test]
fn bricks_with_midside_nodes_converge_like_linear_ones() {
    let dir = std::env::temp_dir().join(format!("plxs-quadratic-multigrid-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("cube.inp"), cube()).unwrap();
    let output = Command::new(plxs())
        .arg("cube")
        .arg("--threads")
        .arg("2")
        .current_dir(&dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned()
        + &String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.success() && log.contains("Job finished"),
        "{log}"
    );
    // "iterative solver: 22 iterations, relative residual ..."
    let iterations: usize = (log.lines())
        .find_map(|line| {
            let rest = line.split("iterative solver: ").nth(1)?;
            rest.split(" iterations").next()?.parse().ok()
        })
        .unwrap_or_else(|| panic!("no iteration count:\n{log}"));
    assert!(iterations <= 28, "{iterations} iterations:\n{log}");
    if std::env::var_os("PLXS_KEEP").is_none() {
        let _ = std::fs::remove_dir_all(&dir);
    }
}
