//! `*DYNAMIC, INTEGRATION=EXPLICIT` against closed-form and implicit solutions.
//!
//! - A bar moving at `v₀` towards its fixed end (`ν = 0`, so that the wave is
//!   one-dimensional): the support takes `ρ c v₀ A` until the wave has run to the free end
//!   and back (`2L/c`), the free end keeps `−v₀` until `L/c` and leaves at `+v₀` after it,
//!   and the energy stays where it was.
//! - A cantilever released from its static deflection under a tip load (a static step, then
//!   the explicit one without the load): it swings at its first eigenfrequency (from a
//!   `*FREQUENCY` step of the same mesh) about its unloaded position, with the static
//!   deflection as amplitude.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

fn work_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("plxs-explicit-{name}-{}", std::process::id()));
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

fn clean(dir: &Path) {
    if std::env::var_os("PLXS_KEEP").is_none() {
        let _ = std::fs::remove_dir_all(dir);
    }
}

/// The blocks of a `.dat` file whose heading contains `heading`: `(time, rows)` with the
/// numbers of every row.
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

/// A box of `nx × ny × nz` C3D8 elements, `lx × ly × lz`, from the origin. Node ids
/// `1 + i + (nx+1)(j + (ny+1) k)`; sets `END0` (x = 0), `END1` (x = lx), `ALL`.
fn bar_mesh(nx: usize, ny: usize, nz: usize, size: [f64; 3], kind: &str) -> String {
    let id = |i: usize, j: usize, k: usize| 1 + i + (nx + 1) * (j + (ny + 1) * k);
    let mut s = String::from("*Node, Nset=ALL\n");
    for k in 0..=nz {
        for j in 0..=ny {
            for i in 0..=nx {
                writeln!(
                    s,
                    "{}, {}, {}, {}",
                    id(i, j, k),
                    size[0] * i as f64 / nx as f64,
                    size[1] * j as f64 / ny as f64,
                    size[2] * k as f64 / nz as f64
                )
                .unwrap();
            }
        }
    }
    writeln!(s, "*Element, Type={kind}, Elset=EALL").unwrap();
    let mut e = 0;
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                e += 1;
                writeln!(
                    s,
                    "{e}, {}, {}, {}, {}, {}, {}, {}, {}",
                    id(i, j, k),
                    id(i + 1, j, k),
                    id(i + 1, j + 1, k),
                    id(i, j + 1, k),
                    id(i, j, k + 1),
                    id(i + 1, j, k + 1),
                    id(i + 1, j + 1, k + 1),
                    id(i, j + 1, k + 1)
                )
                .unwrap();
            }
        }
    }
    for (name, i) in [("END0", 0), ("END1", nx)] {
        writeln!(s, "*Nset, Nset={name}").unwrap();
        for k in 0..=nz {
            for j in 0..=ny {
                writeln!(s, "{},", id(i, j, k)).unwrap();
            }
        }
    }
    s
}

const E: f64 = 2.1e11;
const RHO: f64 = 7850.0;

#[test]
fn a_wave_runs_through_a_bar_and_back() {
    let (length, side, cells) = (0.1, 0.01, 40);
    let v0 = 1.0;
    let c = (E / RHO).sqrt();
    let run_time = 2.0 * length / c;
    let period = 1.5 * run_time;
    let frames = 90;
    let mut input = bar_mesh(cells, 1, 1, [length, side, side], "C3D8");
    writeln!(
        input,
        "*Material, Name=STEEL\n*Elastic\n{E}, 0\n*Density\n{RHO}\n\
         *Solid section, Elset=EALL, Material=STEEL\n\
         *Initial conditions, Type=Velocity\nALL, 1, {}\n\
         *Step\n*Dynamic, Integration=Explicit\n{}, {period}\n\
         *Boundary\nEND0, 1, 3, 0\n\
         *Node print, Nset=END0, Totals=Only\nRF\n\
         *Node print, Nset=END1\nV\n\
         *El print, Elset=EALL, Totals=Only\nALLIE, ALLKE, ALLVD, ALLWK\n*End step",
        -v0,
        period / frames as f64
    )
    .unwrap();
    let dir = work_dir("bar");
    let (dat, log) = run(&dir, "bar", &input);
    clean(&dir);
    assert!(log.contains("stable time step"), "{log}");
    let forces = blocks(&dat, "total force");
    let velocities = blocks(&dat, "velocities");
    assert_eq!(forces.len(), frames, "{log}");
    let expected = RHO * c * v0 * side * side;
    // The plateau between the start and the return of the wave (the coarse mesh spreads
    // the front over a few elements).
    for (t, rows) in &forces {
        let f = rows[0][0].abs();
        if *t > 0.15 * run_time && *t < 0.85 * run_time {
            assert!(
                (f - expected).abs() < 0.05 * expected,
                "support force {f} N at {t} s, expected {expected} N\n{log}"
            );
        }
    }
    // The free end: −v₀ before the wave arrives, +v₀ after it reflected.
    let free_end = |t0: f64| {
        let (_, rows) = velocities
            .iter()
            .min_by(|a, b| (a.0 - t0).abs().total_cmp(&(b.0 - t0).abs()))
            .unwrap();
        rows.iter().map(|r| r[1]).sum::<f64>() / rows.len() as f64
    };
    let before = free_end(0.3 * run_time);
    let after = free_end(0.75 * run_time);
    assert!(
        (before + v0).abs() < 0.03 * v0,
        "free end at {before} m/s before"
    );
    assert!(
        (after - v0).abs() < 0.1 * v0,
        "free end at {after} m/s after"
    );
    // Energy: no loads, the support does no work.
    let energy = |name: &str| blocks(&dat, name).last().map(|(_, r)| r[0][0]).unwrap();
    // The support's nodes stop at once: their half element of mass is not part of it.
    let moving = 1.0 - 0.5 / cells as f64;
    let kinetic0 = 0.5 * RHO * length * side * side * v0 * v0 * moving;
    let (internal, kinetic, damped) = (
        energy("total model internal energy"),
        energy("total model kinetic energy"),
        energy("total model damping dissipation"),
    );
    let balance = (internal + kinetic + damped - kinetic0).abs() / kinetic0;
    assert!(
        balance < 0.01,
        "energy {internal} + {kinetic} + {damped} J, started with {kinetic0} J"
    );
    // The bulk viscosity smears the sharp wave front.
    assert!(
        damped < 0.1 * kinetic0,
        "the bulk viscosity took {damped} J"
    );
}

#[test]
fn a_released_cantilever_swings_at_its_eigenfrequency() {
    let (length, side) = (0.2, 0.02);
    let mesh = bar_mesh(20, 2, 2, [length, side, side], "C3D8I");
    let material = format!(
        "*Material, Name=STEEL\n*Elastic\n{E}, 0.3\n*Density\n{RHO}\n\
         *Solid section, Elset=EALL, Material=STEEL\n"
    );
    let dir = work_dir("cantilever");
    // The first bending eigenfrequency of the mesh.
    let (_, log) = run(
        &dir,
        "modes",
        &format!("{mesh}{material}*Boundary\nEND0, 1, 3, 0\n*Step\n*Frequency\n2\n*End step\n"),
    );
    let frequency = log
        .lines()
        .filter_map(|l| l.split("frequency").nth(1))
        .filter_map(|r| r.split_whitespace().find_map(|w| w.parse::<f64>().ok()))
        .next();
    // The frequencies are in the .dat file as well (cycles/time in the third column).
    let dat_modes = std::fs::read_to_string(dir.join("modes.dat")).unwrap();
    let f1 = dat_modes
        .lines()
        .skip_while(|l| !l.contains("E I G E N V A L U E   O U T P U T"))
        .filter_map(|l| {
            let v: Vec<f64> = l
                .split_whitespace()
                .filter_map(|w| w.parse().ok())
                .collect();
            (v.len() >= 4 && v[0] == 1.0).then(|| v[3])
        })
        .next()
        .or(frequency)
        .expect("first eigenfrequency");
    let swing = 2.5 / f1;
    let tip_load = 1000.0;
    let (dat, log) = run(
        &dir,
        "swing",
        &format!(
            "{mesh}{material}*Boundary\nEND0, 1, 3, 0\n\
             *Step\n*Static\n*Cload\nEND1, 2, {}\n*Node print, Nset=END1\nU\n*End step\n\
             *Step\n*Dynamic, Integration=Explicit\n{}, {swing}\n*Cload, Op=New\n\
             *Node print, Nset=END1\nU\n*End step\n",
            tip_load / 9.0,
            swing / 250.0
        ),
    );
    clean(&dir);
    let tip: Vec<(f64, f64)> = blocks(&dat, "displacements")
        .iter()
        .map(|(t, rows)| {
            (
                *t,
                rows.iter().map(|r| r[2]).sum::<f64>() / rows.len() as f64,
            )
        })
        .collect();
    let deflection = tip[0].1;
    assert!(deflection > 0.0, "{dat}");
    // Zero crossings (downward and upward) after the release at t = 1.
    let crossings: Vec<f64> = tip
        .windows(2)
        .filter(|w| w[0].0 >= 1.0 && w[0].1 * w[1].1 < 0.0)
        .map(|w| w[0].0 + (w[1].0 - w[0].0) * w[0].1 / (w[0].1 - w[1].1))
        .collect();
    assert!(crossings.len() >= 4, "{crossings:?}\n{log}");
    let half_periods = (crossings.len() - 1) as f64;
    let measured = half_periods / (2.0 * (crossings.last().unwrap() - crossings[0]));
    assert!(
        (measured - f1).abs() < 0.03 * f1,
        "swings at {measured} Hz, eigenfrequency {f1} Hz\n{log}"
    );
    let lowest = tip.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    // The static shape is mostly, not only, the first mode (and the bulk viscosity takes a
    // little): the swing comes back to within a few per cent of the release.
    assert!(
        (lowest + deflection).abs() < 0.08 * deflection,
        "swings down to {lowest}, released at {deflection}"
    );
}
