//! Frequency bounds of a `*FREQUENCY` step on a free body. The number of eigenfrequencies
//! counts from the lower bound: ten modes from 1 Hz on a free bar are ten elastic modes, not
//! six rigid body modes dropped from the ten lowest. An upper bound replaces the number by
//! every mode between the bounds.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

const CELLS: usize = 20;
const BAR: f64 = 0.1;
const SIDE: f64 = 0.01;

/// A free steel bar of `CELLS` C3D8 along x, no support.
fn bar(step: &str) -> String {
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
         *Solid section, Elset=BAR, Material=STEEL\n*Step\n{step}\n*End step"
    )
    .unwrap();
    s
}

/// Runs `step` on the bar and returns the eigenfrequencies of the `.dat` file.
fn frequencies(dir: &Path, job: &str, step: &str) -> Vec<f64> {
    std::fs::write(dir.join(format!("{job}.inp")), bar(step)).unwrap();
    let output = Command::new(plxs())
        .arg(job)
        .arg("--threads")
        .arg("2")
        .current_dir(dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned()
        + &String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.success() && log.contains("Job finished"),
        "{log}"
    );
    let dat = std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap();
    dat.lines()
        .skip_while(|l| !l.contains("E I G E N V A L U E   O U T P U T"))
        .skip(1)
        .take_while(|l| !l.contains("P A R T I C I P A T I O N"))
        .filter_map(|l| {
            let v: Vec<f64> = l
                .split_whitespace()
                .filter_map(|w| w.parse().ok())
                .collect();
            (v.len() == 5 && v[0].fract() == 0.0 && v[0] >= 1.0).then(|| v[3])
        })
        .collect()
}

#[test]
fn the_number_of_modes_counts_from_the_lower_bound() {
    let dir = std::env::temp_dir().join(format!("plxs-frequency-bounds-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    // From zero: six rigid body modes first.
    let all = frequencies(&dir, "all", "*Frequency\n16");
    assert_eq!(all.len(), 16, "{all:?}");
    assert!(all[..6].iter().all(|f| *f < 1.0), "{all:?}");
    assert!(all[6] > 100.0, "{all:?}");
    // From 1 Hz: ten elastic modes, the same as the 7th to 16th from zero (those of the
    // double pairs of the bending modes a little less exact with the singular stiffness).
    let above = frequencies(&dir, "above", "*Frequency\n10, 1.");
    assert_eq!(above.len(), 10, "{above:?}");
    for (a, b) in above.iter().zip(&all[6..]) {
        assert!((a - b).abs() <= 1e-4 * b, "{above:?} vs {all:?}");
    }
    // An upper bound: every mode between the bounds, whatever the number says.
    let upper = 0.5 * (all[11] + all[12]);
    let between = frequencies(&dir, "between", &format!("*Frequency\n2, 1., {upper}"));
    assert_eq!(between.len(), 6, "{between:?} below {upper}");
    for (a, b) in between.iter().zip(&all[6..]) {
        assert!((a - b).abs() <= 1e-4 * b, "{between:?} vs {all:?}");
    }
    let _ = std::fs::remove_dir_all(&dir);
}
