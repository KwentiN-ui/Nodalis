//! A linear Dynamic step by modal superposition against direct time integration: the
//! motion of a cantilever under a step load with Rayleigh damping, and from an initial
//! velocity, agree in the tip displacement and velocity and in the energies when the
//! modes cover the motion (20 modes of a beam under a ramped tip load).

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

fn work_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("plxs-modal-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Runs `input` and returns the `.dat` file and the log.
fn run(dir: &Path, job: &str, input: &str) -> (String, String) {
    std::fs::write(dir.join(format!("{job}.inp")), input).unwrap();
    let output = Command::new(plxs())
        .arg(job)
        .arg("--threads")
        .arg("2")
        .current_dir(dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned()
        + &String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "{job} failed:\n{log}");
    assert!(log.contains("Job finished"), "{job}:\n{log}");
    let dat = std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap();
    (dat, log)
}

/// The blocks of a `.dat` file whose heading contains `heading`: `(time, rows)`.
fn blocks(dat: &str, heading: &str) -> Vec<(f64, Vec<Vec<f64>>)> {
    let mut out: Vec<(f64, Vec<Vec<f64>>)> = Vec::new();
    let mut inside = false;
    for line in dat.lines() {
        if line.contains(heading) {
            let time = line
                .split_whitespace()
                .last()
                .and_then(|t| t.parse().ok())
                .unwrap();
            out.push((time, Vec::new()));
            inside = true;
        } else if inside {
            let values: Vec<f64> = line
                .split_whitespace()
                .filter_map(|v| v.parse().ok())
                .collect();
            if values.is_empty() {
                if !out.last().unwrap().1.is_empty() {
                    inside = false;
                }
            } else {
                out.last_mut().unwrap().1.push(values);
            }
        }
    }
    out
}

const DURATION: f64 = 0.05;
const DT: f64 = 2.5e-4;

/// A steel cantilever 1 m x 0.1 m x 0.1 m of 20 x 2 x 2 C3D8, clamped at x = 0, with the
/// outputs of a Dynamic step; `step` is the step block.
fn cantilever(step: &str, initial_velocity: bool) -> String {
    let (nx, ny, nz) = (20, 2, 2);
    let nid = |i: usize, j: usize, k: usize| 1 + i + (nx + 1) * (j + (ny + 1) * k);
    let mut s = String::from("*NODE\n");
    for k in 0..=nz {
        for j in 0..=ny {
            for i in 0..=nx {
                let _ = writeln!(
                    s,
                    "{}, {}, {}, {}",
                    nid(i, j, k),
                    i as f64 / nx as f64,
                    0.1 * j as f64 / ny as f64,
                    0.1 * k as f64 / nz as f64
                );
            }
        }
    }
    s.push_str("*ELEMENT, TYPE=C3D8, ELSET=EALL\n");
    let mut e = 0;
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                e += 1;
                let _ = writeln!(
                    s,
                    "{e}, {}, {}, {}, {}, {}, {}, {}, {}",
                    nid(i, j, k),
                    nid(i + 1, j, k),
                    nid(i + 1, j + 1, k),
                    nid(i, j + 1, k),
                    nid(i, j, k + 1),
                    nid(i + 1, j, k + 1),
                    nid(i + 1, j + 1, k + 1),
                    nid(i, j + 1, k + 1)
                );
            }
        }
    }
    s.push_str("*NSET, NSET=FIX\n");
    for k in 0..=nz {
        for j in 0..=ny {
            let _ = writeln!(s, "{}", nid(0, j, k));
        }
    }
    s.push_str("*NSET, NSET=TIP\n");
    for k in 0..=nz {
        for j in 0..=ny {
            let _ = writeln!(s, "{}", nid(nx, j, k));
        }
    }
    // The load rises over 5 ms: a jump excites every mode of the beam.
    s.push_str("*AMPLITUDE, NAME=RISE\n0, 0, 0.005, 1, 1, 1\n");
    s.push_str(
        "*MATERIAL, NAME=STEEL\n*ELASTIC\n2.1e11, 0.3\n*DENSITY\n7850\n\
         *SOLID SECTION, ELSET=EALL, MATERIAL=STEEL\n",
    );
    if initial_velocity {
        // A smooth velocity field, (x/L)^2 m/s, mostly in the first bending mode: a uniform
        // one has a jump at the clamp, whose high modes the numerical damping of the direct
        // integration removes while the superposition keeps them exactly.
        s.push_str("*INITIAL CONDITIONS, TYPE=VELOCITY\n");
        for k in 0..=nz {
            for j in 0..=ny {
                for i in 1..=nx {
                    let x = i as f64 / nx as f64;
                    let _ = writeln!(s, "{}, 3, {}", nid(i, j, k), x * x);
                }
            }
        }
    }
    s.push_str(step);
    s
}

fn outputs() -> String {
    "*BOUNDARY\nFIX, 1, 3\n*NODE FILE, FREQUENCY=10\nU, V, A\n*EL FILE, FREQUENCY=10\nS\n\
     *NODE PRINT, NSET=TIP\nU, V\n*EL PRINT, ELSET=EALL, TOTALS=ONLY\nALLIE, ALLKE, ALLWK, ALLVD\n\
     *END STEP\n"
        .into()
}

fn direct(load: bool) -> String {
    let mut s = format!("*STEP, INC=1000\n*DYNAMIC, DIRECT\n{DT}, {DURATION}\n");
    if load {
        s.push_str("*DAMPING, ALPHA=0, BETA=1e-5\n*CLOAD, AMPLITUDE=RISE\nTIP, 3, -111.111111\n");
    }
    s + &outputs()
}

fn modal(load: bool, modes: u32) -> String {
    let mut s = format!(
        "*STEP\n*FREQUENCY, STORAGE=YES\n{modes}\n*BOUNDARY\nFIX, 1, 3\n*END STEP\n\
         *STEP, INC=1000\n*MODAL DYNAMICS\n{DT}, {DURATION}\n"
    );
    if load {
        s.push_str(
            "*MODAL DAMPING, RAYLEIGH\n, , 0, 1e-5\n*CLOAD, AMPLITUDE=RISE\nTIP, 3, -111.111111\n",
        );
    }
    s + &outputs()
}

/// A quantity over time: `(time, value)` per increment.
type Series = Vec<(f64, f64)>;

/// The mean z displacement and velocity of the tip over time.
fn tip(dat: &str) -> (Series, Series) {
    let mean = |rows: &[Vec<f64>]| rows.iter().map(|r| r[3]).sum::<f64>() / rows.len() as f64;
    let u = blocks(dat, "displacements")
        .iter()
        .map(|(t, rows)| (*t, mean(rows)))
        .collect();
    let v = blocks(dat, "velocities")
        .iter()
        .map(|(t, rows)| (*t, mean(rows)))
        .collect();
    (u, v)
}

fn energy(dat: &str, name: &str) -> Vec<f64> {
    blocks(dat, name)
        .iter()
        .map(|(_, rows)| rows[0][0])
        .collect()
}

/// The largest difference of two series relative to the largest value of the first.
fn deviation(a: &[(f64, f64)], b: &[(f64, f64)]) -> f64 {
    assert_eq!(a.len(), b.len());
    let scale = a.iter().map(|(_, v)| v.abs()).fold(0.0, f64::max);
    a.iter()
        .zip(b)
        .map(|((ta, va), (tb, vb))| {
            assert!((ta - tb).abs() < 1e-9, "times differ: {ta} vs {tb}");
            (va - vb).abs()
        })
        .fold(0.0, f64::max)
        / scale
}

fn compare(name: &str, load: bool) {
    let dir = work_dir(name);
    let (direct_dat, _) = run(&dir, "direct", &cantilever(&direct(load), !load));
    let (modal_dat, log) = run(&dir, "modal", &cantilever(&modal(load, 20), !load));
    assert!(log.contains("the highest mode lies at"), "{log}");
    let (ud, vd) = tip(&direct_dat);
    let (um, vm) = tip(&modal_dat);
    assert_eq!(ud.len(), (DURATION / DT).round() as usize);
    // The direct integration elongates the periods of the higher modes and damps them
    // numerically (it loses 1 % of the energy over the step, the superposition none), so
    // the curves drift apart by a few percent of the amplitude.
    let (du, dv) = (deviation(&ud, &um), deviation(&vd, &vm));
    assert!(
        du < 0.1,
        "{name}: tip displacements differ by {du:.4} of the amplitude"
    );
    // The velocity carries the higher modes more strongly than the displacement.
    let v_tolerance = if load { 0.1 } else { 0.2 };
    assert!(
        dv < v_tolerance,
        "{name}: tip velocities differ by {dv:.4} of the amplitude"
    );
    // The energies: both methods account for the same work, strain and kinetic energy.
    for quantity in [
        "internal energy",
        "kinetic energy",
        "external work",
        "damping dissipation",
    ] {
        let (ed, em) = (energy(&direct_dat, quantity), energy(&modal_dat, quantity));
        assert_eq!(ed.len(), em.len(), "{quantity}");
        let scale = ed.iter().map(|e| e.abs()).fold(0.0, f64::max).max(1e-12);
        let worst = ed
            .iter()
            .zip(&em)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max)
            / scale;
        assert!(
            worst < 0.08,
            "{name}: {quantity} differs by {worst:.4} (direct {ed:?}, modal {em:?})"
        );
    }
    // The modal step writes the frames with accelerations like the direct one.
    let frd = std::fs::read_to_string(dir.join("modal.frd")).unwrap();
    assert!(
        frd.contains(" -4  ACCE"),
        "no accelerations in the modal frames"
    );
    if std::env::var_os("PLXS_KEEP").is_none() {
        let _ = std::fs::remove_dir_all(&dir);
    }
}

#[test]
fn modal_superposition_follows_a_damped_cantilever_under_a_step_load() {
    compare("load", true);
}

#[test]
fn modal_superposition_takes_up_an_initial_velocity() {
    compare("velocity", false);
}
