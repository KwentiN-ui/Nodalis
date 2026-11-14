//! The iterative solver on expanded shells: its multigrid first gathers every fibre of
//! expanded nodes (bottom, middle, top) into one shell node with three translations and two
//! rotations, and a knot into one node with its translations and rotation vector. With
//! aggregates of the expanded nodes themselves bending was lost on the coarse levels, and
//! the iterations grew with the slenderness of the elements: a corrugated tube of `S8R` 0.3
//! thick took five times the iterations it takes now, and a large one did not converge.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

/// A corrugated steel tube of `S8R`, `around` elements around and `along` elements along
/// (eight per wave), radius 100, wave amplitude 8 and length 20, 0.3 thick: clamped at one
/// end and pulled sideways at the other.
fn tube(around: usize, along: usize, solver: &str) -> String {
    use std::f64::consts::PI;
    const LENGTH: f64 = 20.0;
    let height = along as f64 * LENGTH / 8.0;
    let mut ids: BTreeMap<(usize, usize), usize> = BTreeMap::new();
    let mut s = String::from("*NODE\n");
    // Nodes on the grid of half elements, `i` around and `j` along.
    let mut node = |i: usize, j: usize, s: &mut String| -> usize {
        let i = i % (2 * around);
        if let Some(&id) = ids.get(&(i, j)) {
            return id;
        }
        let id = ids.len() + 1;
        ids.insert((i, j), id);
        let angle = PI * i as f64 / around as f64;
        let z = height * j as f64 / (2 * along) as f64;
        let r = 100.0 + 8.0 * (2.0 * PI * z / LENGTH).sin();
        writeln!(s, "{id}, {}, {}, {z}", r * angle.cos(), r * angle.sin()).unwrap();
        id
    };
    let mut elements = Vec::new();
    for e in 0..around {
        for f in 0..along {
            let (i, j) = (2 * e, 2 * f);
            let corners = [(i, j), (i + 2, j), (i + 2, j + 2), (i, j + 2)];
            let midsides = [(i + 1, j), (i + 2, j + 1), (i + 1, j + 2), (i, j + 1)];
            let nodes: Vec<String> = (corners.iter().chain(&midsides))
                .map(|&(i, j)| node(i, j, &mut s).to_string())
                .collect();
            elements.push(nodes.join(", "));
        }
    }
    s.push_str("*ELEMENT, TYPE=S8R, ELSET=TUBE\n");
    for (k, nodes) in elements.iter().enumerate() {
        writeln!(s, "{}, {nodes}", k + 1).unwrap();
    }
    let ring =
        |j: usize| -> Vec<String> { (0..2 * around).map(|i| ids[&(i, j)].to_string()).collect() };
    let top = ring(2 * along);
    writeln!(s, "*NSET, NSET=FIX\n{}", ring(0).join(",\n")).unwrap();
    writeln!(s, "*NSET, NSET=TOP\n{}", top.join(",\n")).unwrap();
    s.push_str("*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n");
    s.push_str("*SHELL SECTION, ELSET=TUBE, MATERIAL=STEEL\n0.3\n*BOUNDARY\nFIX, 1, 3\n");
    writeln!(s, "*STEP\n*STATIC, SOLVER={solver}").unwrap();
    writeln!(s, "*CLOAD\nTOP, 2, {}", 10.0 / top.len() as f64).unwrap();
    s.push_str("*NODE FILE\nU\n*END STEP\n");
    s
}

/// The log of the run and the largest displacement magnitude in the `.frd`.
fn run(job: &str, input: &str) -> (String, f64) {
    let dir =
        std::env::temp_dir().join(format!("plxs-shell-multigrid-{job}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(format!("{job}.inp")), input).unwrap();
    let output = Command::new(plxs())
        .arg(job)
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
    let frd = std::fs::read_to_string(dir.join(format!("{job}.frd"))).unwrap();
    if std::env::var_os("PLXS_KEEP").is_none() {
        let _ = std::fs::remove_dir_all(&dir);
    }
    let block = frd.split("DISP").nth(1).expect("a displacement block");
    let largest = (block.lines())
        .take_while(|l| !l.starts_with(" -3"))
        .filter(|l| l.starts_with(" -1"))
        .map(|l| {
            let u: Vec<f64> = (0..3)
                .map(|c| l[13 + 12 * c..25 + 12 * c].trim().parse().unwrap())
                .collect();
            (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt()
        })
        .fold(0.0, f64::max);
    (log, largest)
}

#[test]
fn corrugated_shell_tube_converges() {
    let (log, iterative) = run("iterative", &tube(24, 16, "ITERATIVE CHOLESKY"));
    // "iterative solver: 22 iterations, relative residual ..."
    let iterations: usize = (log.lines())
        .find_map(|line| {
            let rest = line.split("iterative solver: ").nth(1)?;
            rest.split(" iterations").next()?.parse().ok()
        })
        .unwrap_or_else(|| panic!("no iteration count:\n{log}"));
    assert!(iterations <= ITERATIONS, "{iterations} iterations:\n{log}");
    let (_, direct) = run("direct", &tube(24, 16, "PASTIX"));
    assert!(direct > 0.0, "no displacement");
    assert!(
        (iterative - direct).abs() <= 1e-4 * direct,
        "iterative {iterative}, direct {direct}"
    );
}

/// 1081 with the fibres, 5278 with aggregates of the expanded nodes.
const ITERATIONS: usize = 1500;
