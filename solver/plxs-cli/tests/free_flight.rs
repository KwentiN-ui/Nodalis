//! A body in free flight under NLGEOM: no load, no support, nothing in contact, only an
//! initial velocity. Its internal forces are differences of coordinates far larger than its
//! (zero) strains, and their round-off stayed above the residual tolerance, which in free
//! flight is set by the round-off of the momentum alone: every increment was split down to
//! the limit (a projectile flying towards a can took thousands of iterations before it got
//! there). A correction at the round-off of the coordinates now ends the iteration.

use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

const CELLS: usize = 20;
const BAR: f64 = 0.1;
const SIDE: f64 = 0.01;
const V0: f64 = 138.9;

/// A steel bar (`CELLS` C3D8 along x) flying sideways at `V0`, stiffness-proportional
/// damping only (mass-proportional damping would brake the flight).
fn bar() -> String {
    let mut s = String::from("*Node, Nset=BARN\n");
    let id = |i: usize, c: usize| 1 + 4 * i + c;
    for i in 0..=CELLS {
        for (c, (y, z)) in [(0.0, 0.0), (SIDE, 0.0), (SIDE, SIDE), (0.0, SIDE)]
            .iter()
            .enumerate()
        {
            writeln!(
                s,
                "{}, {}, {y}, {z}",
                id(i, c),
                BAR * i as f64 / CELLS as f64
            )
            .unwrap();
        }
    }
    s.push_str("*Element, Type=C3D8, Elset=BAR\n");
    for i in 0..CELLS {
        let nodes: Vec<String> = (0..4)
            .map(|c| id(i, c))
            .chain((0..4).map(|c| id(i + 1, c)))
            .map(|n| n.to_string())
            .collect();
        writeln!(s, "{}, {}", i + 1, nodes.join(", ")).unwrap();
    }
    writeln!(
        s,
        "*Material, Name=STEEL\n*Elastic\n2.1e11, 0.3\n*Density\n7850\n\
         *Solid section, Elset=BAR, Material=STEEL\n\
         *Initial conditions, Type=Velocity\nBARN, 2, {V0}\n\
         *Step, Nlgeom, Inc=1000\n*Dynamic, Direct\n1e-3, 1e-2\n\
         *Damping, Alpha=0, Beta=1e-6\n\
         *Node print, Nset=BARN\nU\n*End step"
    )
    .unwrap();
    s
}

#[test]
fn a_body_in_free_flight_converges_at_once() {
    let dir = std::env::temp_dir().join(format!("plxs-free-flight-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("flight.inp"), bar()).unwrap();
    let output = Command::new(plxs())
        .arg("flight")
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
    // Ten increments of one or two iterations (it took thousands).
    let iterations = log
        .lines()
        .filter(|l| l.trim().starts_with("iteration"))
        .count();
    assert!(iterations <= 40, "{iterations} iterations:\n{log}");
    // The bar flew the whole way, stress-free.
    let dat = std::fs::read_to_string(dir.join("flight.dat")).unwrap();
    let last: Vec<f64> = (dat.lines().rev())
        .find_map(|line| {
            let values: Vec<f64> = (line.split_whitespace())
                .filter_map(|v| v.parse().ok())
                .collect();
            (values.len() == 4).then_some(values)
        })
        .unwrap();
    let expected = V0 * 1e-2;
    assert!(
        (last[2] - expected).abs() < 1e-9 * expected && last[1].abs() < 1e-9,
        "{last:?}"
    );
    if std::env::var_os("PLXS_KEEP").is_none() {
        let _ = std::fs::remove_dir_all(&dir);
    }
}
