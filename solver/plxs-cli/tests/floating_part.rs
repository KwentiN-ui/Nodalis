//! A part held only by contact, pushed along its contact face: the solver says which part
//! floats and along what direction, instead of only that the increment did not converge.
//! A screw whose head was given no contact with its seat jumped back and forth by ten
//! millimetres in every iteration of a prescribed turn, and the log said nothing but that
//! the residual force did not fall.
//!
//! Checked here: a block pressed onto a fixed base and pushed sideways without friction
//! fails, and the failure names the block and the direction it slides in; with enough
//! friction to stick it finishes and says nothing about floating.

use std::path::PathBuf;
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

/// Runs `input` as job `job`; returns whether it finished all increments and the log.
fn run(job: &str, input: &str) -> (bool, String) {
    let dir = std::env::temp_dir().join(format!("plxs-floating-{job}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
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
    let sta = std::fs::read_to_string(dir.join(format!("{job}.sta"))).unwrap_or_default();
    let finished = (sta.lines().rev())
        .find_map(|l| {
            let f: Vec<f64> = (l.split_whitespace())
                .filter_map(|t| t.parse().ok())
                .collect();
            (f.len() == 7).then(|| f[5])
        })
        .is_some_and(|t| t > 1.0 - 1e-9);
    let _ = std::fs::remove_dir_all(&dir);
    (finished, log)
}

/// A block (element 2) on a fixed base (element 1), pressed down by 100 N and pushed along
/// +x by `push` per top node, with the friction lines `friction` in its contact.
fn block(push: f64, friction: &str) -> String {
    format!(
        r"*NODE
1, 0, 0
2, 10, 0
3, 10, 2
4, 0, 2
11, 2, 2
12, 6, 2
13, 6, 6
14, 2, 6
*ELEMENT, TYPE=CPE4, ELSET=BASE
1, 1, 2, 3, 4
*ELEMENT, TYPE=CPE4, ELSET=BLOCK
2, 11, 12, 13, 14
*SOLID SECTION, ELSET=BASE, MATERIAL=STEEL
1.
*SOLID SECTION, ELSET=BLOCK, MATERIAL=STEEL
1.
*MATERIAL, NAME=STEEL
*ELASTIC
210000, 0.3
*SURFACE, NAME=TOP
1, S3
*SURFACE, NAME=BOT
2, S1
*CONTACT PAIR, INTERACTION=I1, TYPE=SURFACE TO SURFACE
BOT, TOP
*SURFACE INTERACTION, NAME=I1
*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=LINEAR
1e6
{friction}*NSET, NSET=FIX
1, 2
*NSET, NSET=TOPN
13, 14
*BOUNDARY
FIX, 1, 2
*STEP, NLGEOM, INC=20
*STATIC
0.1, 1.0
*CLOAD
TOPN, 2, -50
TOPN, 1, {push}
*NODE PRINT, NSET=TOPN
U
*END STEP
",
    )
}

#[test]
fn a_block_sliding_off_without_friction_is_named() {
    let (finished, log) = run("free", &block(10.0, ""));
    assert!(!finished, "{log}");
    let error = (log.lines())
        .find(|l| l.starts_with("*ERROR"))
        .unwrap_or_else(|| panic!("no error:\n{log}"));
    assert!(error.contains("held only by contact"), "{error}");
    assert!(error.contains("element 2"), "{error}");
    assert!(error.contains("along (1.00, "), "{error}");
}

#[test]
fn a_block_that_sticks_is_no_floating_part() {
    let (finished, log) = run("stick", &block(10.0, "*FRICTION\n1.0\n"));
    assert!(finished, "{log}");
    assert!(!log.contains("held only by contact"), "{log}");
}
