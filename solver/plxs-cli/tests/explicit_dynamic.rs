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

// ---------------------------------------------------------------------------------------
// Stage 2: contact, plasticity, large deformations, hourglass control

/// A steel bar (`cells` C3D8 along x, one across, 0.1 m × 10 mm × 10 mm) moving at `v0`
/// towards a fixed steel brick at `gap` from its end, with a contact pair of type `kind`.
fn bar_on_wall(kind: &str, cells: usize, gap: f64, v0: f64, frame: f64, period: f64) -> String {
    let (bar, side) = (0.1, 0.01);
    let mut s = String::from("*Node\n");
    let id = |i: usize, c: usize| 1 + 4 * i + c;
    let corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)];
    for i in 0..=cells {
        for (c, (y, z)) in corners.iter().enumerate() {
            writeln!(
                s,
                "{}, {}, {y}, {z}",
                id(i, c),
                bar * i as f64 / cells as f64
            )
            .unwrap();
        }
    }
    let x0 = bar + gap;
    for (k, x) in [x0, x0 + 0.01].iter().enumerate() {
        for (c, (y, z)) in [(-0.01, -0.01), (0.02, -0.01), (0.02, 0.02), (-0.01, 0.02)]
            .iter()
            .enumerate()
        {
            writeln!(s, "{}, {x}, {y}, {z}", 1001 + 4 * k + c).unwrap();
        }
    }
    s.push_str("*Element, Type=C3D8, Elset=BAR\n");
    for i in 0..cells {
        let n: Vec<usize> = (0..2)
            .flat_map(|k| (0..4).map(move |c| id(i + k, c)))
            .collect();
        writeln!(
            s,
            "{}, {}",
            i + 1,
            n.iter()
                .map(|n| n.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        )
        .unwrap();
    }
    s.push_str(
        "*Element, Type=C3D8, Elset=WALL\n1001, 1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008\n\
         *Elset, Elset=EALL\nBAR, WALL\n*Nset, Nset=WALLN, Generate\n1001, 1008, 1\n",
    );
    writeln!(s, "*Nset, Nset=BARN, Generate\n1, {}, 1", 4 * cells + 4).unwrap();
    writeln!(
        s,
        "*Elset, Elset=BAREND\n{cells}\n*Surface, Name=SLAVE, Type=Element\nBAREND, S2\n\
         *Surface, Name=MASTER, Type=Element\nWALL, S1\n\
         *Material, Name=STEEL\n*Elastic\n{E}, 0\n*Density\n{RHO}\n\
         *Solid section, Elset=EALL, Material=STEEL\n\
         *Surface interaction, Name=SI\n*Surface behavior, Pressure-overclosure=Linear\n1e15\n\
         *Contact pair, Interaction=SI, Type={kind}\nSLAVE, MASTER\n\
         *Initial conditions, Type=Velocity\nBARN, 1, {v0}\n\
         *Step\n*Dynamic, Integration=Explicit\n{frame}, {period}\n\
         *Boundary\nWALLN, 1, 3, 0\n\
         *Node print, Nset=WALLN, Totals=Only\nRF\n\
         *Node print, Nset=BARN\nV\n\
         *El print, Elset=EALL, Totals=Only\nALLIE, ALLKE, ALLVD, ALLWK\n*End step"
    )
    .unwrap();
    s
}

/// The bar of [`bar_on_wall`] hits the wall: the contact lasts `2L/c` and carries
/// `ρ c v₀ A`, the bar leaves at `−v₀` and keeps its energy.
fn check_bar_on_wall(kind: &str, job: &str) {
    let (bar, side, cells, gap, v0) = (0.1, 0.01, 20, 1e-5, 1.0);
    let c = (E / RHO).sqrt();
    let duration = 2.0 * bar / c;
    let arrival = gap / v0;
    let period = arrival + 1.5 * duration;
    let frames = 120;
    let input = bar_on_wall(kind, cells, gap, v0, period / frames as f64, period);
    let dir = work_dir(job);
    let (dat, log) = run(&dir, job, &input);
    clean(&dir);
    let forces = blocks(&dat, "total force");
    assert_eq!(forces.len(), frames, "{log}");
    let expected = RHO * c * v0 * side * side;
    let mut impulse = 0.0;
    let (mut t_prev, mut f_prev) = (0.0, 0.0);
    let mut contact = (None, None);
    for (t, rows) in &forces {
        let f = rows[0][0].abs();
        impulse += 0.5 * (f + f_prev) * (t - t_prev);
        if f > 1e-3 * expected {
            contact.0.get_or_insert(*t);
            contact.1 = Some(*t);
        }
        (t_prev, f_prev) = (*t, f);
        if (t - (arrival + 0.2 * duration)) * (t - (arrival + 0.7 * duration)) < 0.0 {
            assert!(
                (f - expected).abs() < 0.08 * expected,
                "{kind}: contact force {f} N at {t} s, expected {expected} N\n{log}"
            );
        }
    }
    let (first, last) = (contact.0.unwrap(), contact.1.unwrap());
    assert!(
        ((last - first) - duration).abs() < 0.15 * duration,
        "{kind}: contact from {first} to {last} s, expected {duration} s"
    );
    let mass = RHO * bar * side * side;
    assert!(
        (impulse - 2.0 * mass * v0).abs() < 0.05 * 2.0 * mass * v0,
        "{kind}: impulse {impulse}, expected {}",
        2.0 * mass * v0
    );
    let (_, rows) = blocks(&dat, "velocities").pop().unwrap();
    let rebound = rows.iter().map(|r| r[1]).sum::<f64>() / rows.len() as f64;
    assert!(
        (rebound + v0).abs() < 0.05 * v0,
        "{kind}: the bar leaves at {rebound} m/s"
    );
    let energy = |name: &str| blocks(&dat, name).last().map(|(_, r)| r[0][0]).unwrap();
    let kinetic0 = 0.5 * mass * v0 * v0;
    let (internal, kinetic, damped) = (
        energy("total model internal energy"),
        energy("total model kinetic energy"),
        energy("total model damping dissipation"),
    );
    assert!(
        (internal + kinetic + damped - kinetic0).abs() < 0.02 * kinetic0,
        "{kind}: energy {internal} + {kinetic} + {damped} J, started with {kinetic0} J"
    );
    assert!(
        damped < 0.1 * kinetic0,
        "{kind}: the damping took {damped} J of {kinetic0} J"
    );
}

#[test]
fn an_elastic_bar_bounces_off_a_wall_node_to_surface() {
    check_bar_on_wall("Node to surface", "wall-n2s");
}

#[test]
fn an_elastic_bar_bounces_off_a_wall_surface_to_surface() {
    check_bar_on_wall("Surface to surface", "wall-s2s");
}

/// A block (2 × 2 CPE4, 10 mm) sliding at `v0` on a steel plate under gravity stops after
/// `v0² / (2 μ g)`.
#[test]
fn a_block_sliding_on_a_plate_stops_by_friction() {
    let (v0, mu, g) = (0.3, 0.3, 9.81);
    let stop = v0 / (mu * g);
    let distance = v0 * v0 / (2.0 * mu * g);
    let mut s = String::from("*Node\n");
    for j in 0..2 {
        for i in 0..=20 {
            writeln!(
                s,
                "{}, {}, {}, 0",
                1 + i + 21 * j,
                0.005 * i as f64,
                0.005 * j as f64
            )
            .unwrap();
        }
    }
    for j in 0..=2 {
        for i in 0..=2 {
            writeln!(
                s,
                "{}, {}, {}, 0",
                1001 + i + 3 * j,
                0.01 + 0.005 * i as f64,
                0.005 + 0.005 * j as f64
            )
            .unwrap();
        }
    }
    s.push_str("*Element, Type=CPE4, Elset=PLATE\n");
    for i in 0..20 {
        writeln!(s, "{}, {}, {}, {}, {}", i + 1, 1 + i, 2 + i, 23 + i, 22 + i).unwrap();
    }
    s.push_str("*Element, Type=CPE4, Elset=BLOCK\n");
    for j in 0..2 {
        for i in 0..2 {
            let n = 1001 + i + 3 * j;
            writeln!(
                s,
                "{}, {n}, {}, {}, {}",
                101 + i + 2 * j,
                n + 1,
                n + 4,
                n + 3
            )
            .unwrap();
        }
    }
    writeln!(
        s,
        "*Elset, Elset=EALL\nPLATE, BLOCK\n*Nset, Nset=BOTTOM, Generate\n1, 21, 1\n\
         *Nset, Nset=BLOCKN, Generate\n1001, 1009, 1\n*Nset, Nset=CORNER\n1001\n\
         *Elset, Elset=BLOCKLOW\n101, 102\n*Surface, Name=SLAVE, Type=Element\nBLOCKLOW, S1\n\
         *Surface, Name=MASTER, Type=Element\nPLATE, S3\n\
         *Material, Name=STEEL\n*Elastic\n{E}, 0.3\n*Density\n{RHO}\n\
         *Solid section, Elset=EALL, Material=STEEL\n1\n\
         *Surface interaction, Name=SI\n*Surface behavior, Pressure-overclosure=Linear\n1e15\n\
         *Friction\n{mu}, 1e13\n\
         *Contact pair, Interaction=SI, Type=Surface to surface\nSLAVE, MASTER\n\
         *Initial conditions, Type=Velocity\nBLOCKN, 1, {v0}\n\
         *Step\n*Dynamic, Integration=Explicit\n0.005, 0.06\n*Boundary\nBOTTOM, 1, 2, 0\n\
         *Dload\nEALL, GRAV, {g}, 0, -1, 0\n\
         *Node print, Nset=CORNER\nU\n*End step"
    )
    .unwrap();
    let dir = work_dir("slide");
    let (dat, log) = run(&dir, "slide", &s);
    clean(&dir);
    let path: Vec<(f64, [f64; 2])> = blocks(&dat, "displacements")
        .into_iter()
        .map(|(t, rows)| (t, [rows[0][1], rows[0][2]]))
        .collect();
    assert!(!path.is_empty(), "{log}");
    for (t, u) in &path {
        let expected = if *t < stop {
            v0 * t - 0.5 * mu * g * t * t
        } else {
            distance
        };
        assert!(
            (u[0] - expected).abs() < 0.03 * distance,
            "t = {t}: the block slid {} m, expected {expected} m\n{log}",
            u[0]
        );
        assert!(
            u[1].abs() < 2e-6,
            "t = {t}: the block moved by {} m vertically",
            u[1]
        );
    }
}

/// A bar hitting its support faster than it can take elastically (`ρ c v₀ > σ_y`): the
/// support carries the yield force `σ_y A` (perfect plasticity, ν = 0, uniaxial stress: the
/// support holds the bar along its axis only, so that it can widen as it yields), and the
/// energy the bar loses is in the internal energy.
#[test]
fn a_bar_hitting_its_support_yields() {
    let (length, side, cells) = (0.1, 0.01, 40);
    let (v0, yield_stress) = (20.0, 300e6);
    let c = (E / RHO).sqrt();
    assert!(RHO * c * v0 > 2.0 * yield_stress);
    let run_time = 2.0 * length / c;
    let period = run_time;
    let frames = 60;
    let mut input = bar_mesh(cells, 1, 1, [length, side, side], "C3D8");
    writeln!(
        input,
        "*Material, Name=STEEL\n*Elastic\n{E}, 0\n*Plastic\n{yield_stress}, 0\n*Density\n{RHO}\n\
         *Solid section, Elset=EALL, Material=STEEL\n\
         *Initial conditions, Type=Velocity\nALL, 1, {}\n\
         *Step\n*Dynamic, Integration=Explicit\n{}, {period}\n\
         *Boundary\nEND0, 1, 1, 0\n\
         *Node print, Nset=END0, Totals=Only\nRF\n\
         *El print, Elset=EALL, Totals=Only\nALLIE, ALLKE, ALLVD, ALLWK\n*End step",
        -v0,
        period / frames as f64
    )
    .unwrap();
    let dir = work_dir("plastic");
    let (dat, log) = run(&dir, "plastic", &input);
    clean(&dir);
    // The widening end rings (its lateral inertia) about the yield force, less and less.
    let expected = yield_stress * side * side;
    let late: Vec<f64> = blocks(&dat, "total force")
        .into_iter()
        .filter(|(t, _)| *t > 0.3 * run_time)
        .map(|(_, rows)| rows[0][0].abs())
        .collect();
    let mean = late.iter().sum::<f64>() / late.len() as f64;
    assert!(
        (mean - expected).abs() < 0.05 * expected,
        "support force {mean} N on average, expected the yield force {expected} N\n{log}"
    );
    for f in &late {
        assert!(
            (f - expected).abs() < 0.2 * expected,
            "support force {f} N, expected the yield force {expected} N"
        );
    }
    let energy = |name: &str| blocks(&dat, name).last().map(|(_, r)| r[0][0]).unwrap();
    let kinetic0 = 0.5 * RHO * length * side * side * v0 * v0 * (1.0 - 0.5 / cells as f64);
    let (internal, kinetic, damped) = (
        energy("total model internal energy"),
        energy("total model kinetic energy"),
        energy("total model damping dissipation"),
    );
    assert!(
        (internal + kinetic + damped - kinetic0).abs() < 0.01 * kinetic0,
        "energy {internal} + {kinetic} + {damped} J, started with {kinetic0} J"
    );
    // The elastic energy of a bar at the yield stress is far below what it lost.
    let elastic = yield_stress * yield_stress / (2.0 * E) * length * side * side;
    assert!(internal > 3.0 * elastic, "internal energy {internal} J");
}

/// A cube spinning about its axis under NLGEOM: a rigid rotation, which neither the bulk
/// viscosity (in the current configuration) nor anything else may slow down.
#[test]
fn a_spinning_cube_keeps_its_energy_under_nlgeom() {
    let side = 0.01;
    let omega = 2000.0;
    let mut input = bar_mesh(2, 2, 2, [side, side, side], "C3D8R");
    // Rotation about the cube's axis parallel to z: v = ω × (x − x_c).
    let mut velocities = String::new();
    for k in 0..=2 {
        for j in 0..=2 {
            for i in 0..=2 {
                let id = 1 + i + 3 * (j + 3 * k);
                let (x, y) = (
                    side * i as f64 / 2.0 - side / 2.0,
                    side * j as f64 / 2.0 - side / 2.0,
                );
                writeln!(
                    velocities,
                    "{id}, 1, {}\n{id}, 2, {}",
                    -omega * y,
                    omega * x
                )
                .unwrap();
            }
        }
    }
    // Half a turn.
    let period = std::f64::consts::PI / omega;
    writeln!(
        input,
        "*Material, Name=STEEL\n*Elastic\n{E}, 0.3\n*Density\n{RHO}\n\
         *Solid section, Elset=EALL, Material=STEEL\n\
         *Initial conditions, Type=Velocity\n{velocities}\
         *Step, Nlgeom\n*Dynamic, Integration=Explicit\n{}, {period}\n\
         *El print, Elset=EALL, Totals=Only\nALLIE, ALLKE, ALLVD, ALLWK\n*End step",
        period / 10.0
    )
    .unwrap();
    let dir = work_dir("spin");
    let (dat, log) = run(&dir, "spin", &input);
    clean(&dir);
    let series = |name: &str| -> Vec<f64> {
        blocks(&dat, name)
            .into_iter()
            .map(|(_, r)| r[0][0])
            .collect()
    };
    let kinetic = series("total model kinetic energy");
    let damped = series("total model damping dissipation");
    // The lumped kinetic energy: an eighth of an element's mass at each of its nodes.
    let element_mass = RHO * (side / 2.0).powi(3);
    let mut kinetic0 = 0.0;
    for k in 0..=2_i32 {
        for j in 0..=2_i32 {
            for i in 0..=2_i32 {
                let elements = [i, j, k]
                    .iter()
                    .map(|&c| if c == 1 { 2.0 } else { 1.0 })
                    .product::<f64>();
                let r2 = ((i - 1).pow(2) + (j - 1).pow(2)) as f64 * (side / 2.0).powi(2);
                kinetic0 += 0.5 * elements * element_mass / 8.0 * omega * omega * r2;
            }
        }
    }
    let last = *kinetic.last().unwrap();
    assert!(
        (last - kinetic0).abs() < 0.01 * kinetic0,
        "kinetic energy {last} J after half a turn, {kinetic0} J at the start\n{log}"
    );
    let lost = *damped.last().unwrap();
    assert!(
        lost < 1e-3 * kinetic0,
        "the viscosity took {lost} J of {kinetic0} J"
    );
}

/// The wave of the first test with reduced integration (C3D8R and its hourglass control):
/// the same support force and energy.
#[test]
fn a_wave_runs_through_a_bar_of_reduced_elements() {
    let (length, side, cells) = (0.1, 0.01, 40);
    let v0 = 1.0;
    let c = (E / RHO).sqrt();
    let run_time = 2.0 * length / c;
    let frames = 40;
    let mut input = bar_mesh(cells, 2, 2, [length, side, side], "C3D8R");
    writeln!(
        input,
        "*Material, Name=STEEL\n*Elastic\n{E}, 0\n*Density\n{RHO}\n\
         *Solid section, Elset=EALL, Material=STEEL\n\
         *Initial conditions, Type=Velocity\nALL, 1, {}\n\
         *Step\n*Dynamic, Integration=Explicit\n{}, {run_time}\n\
         *Boundary\nEND0, 1, 3, 0\n\
         *Node print, Nset=END0, Totals=Only\nRF\n\
         *El print, Elset=EALL, Totals=Only\nALLIE, ALLKE, ALLVD, ALLWK\n*End step",
        -v0,
        run_time / frames as f64
    )
    .unwrap();
    let dir = work_dir("reduced");
    let (dat, log) = run(&dir, "reduced", &input);
    clean(&dir);
    let expected = RHO * c * v0 * side * side;
    for (t, rows) in blocks(&dat, "total force") {
        if t > 0.15 * run_time && t < 0.85 * run_time {
            let f = rows[0][0].abs();
            assert!(
                (f - expected).abs() < 0.05 * expected,
                "support force {f} N at {t} s, expected {expected} N\n{log}"
            );
        }
    }
    let energy = |name: &str| blocks(&dat, name).last().map(|(_, r)| r[0][0]).unwrap();
    let kinetic0 = 0.5 * RHO * length * side * side * v0 * v0 * (1.0 - 0.5 / cells as f64);
    let (internal, kinetic, damped) = (
        energy("total model internal energy"),
        energy("total model kinetic energy"),
        energy("total model damping dissipation"),
    );
    assert!(
        (internal + kinetic + damped - kinetic0).abs() < 0.01 * kinetic0,
        "energy {internal} + {kinetic} + {damped} J, started with {kinetic0} J"
    );
    assert!(damped < 0.1 * kinetic0, "the viscosities took {damped} J");
}

/// A rubber block (Neo-Hooke, NLGEOM) hitting its support at a tenth of its wave speed:
/// large strains, and the energy balance holds.
#[test]
fn a_rubber_block_bounces_with_large_strains() {
    let (length, side) = (0.02, 0.01);
    let (c10, d1): (f64, f64) = (0.4e6, 1e-8);
    let rho: f64 = 1100.0;
    let bulk = 2.0 / d1;
    let c = ((bulk + 4.0 / 3.0 * 2.0 * c10) / rho).sqrt();
    let v0 = 5.0;
    let period = 4.0 * length / (2.0 * c10 * 3.0 / rho).sqrt();
    let mut input = bar_mesh(8, 2, 2, [length, side, side], "C3D8");
    writeln!(
        input,
        "*Material, Name=RUBBER\n*Hyperelastic, Neo Hooke\n{c10}, {d1}\n*Density\n{rho}\n\
         *Solid section, Elset=EALL, Material=RUBBER\n\
         *Initial conditions, Type=Velocity\nALL, 1, {}\n\
         *Step, Nlgeom\n*Dynamic, Integration=Explicit\n{}, {period}\n\
         *Boundary\nEND0, 1, 3, 0\n\
         *El print, Elset=EALL, Totals=Only\nALLIE, ALLKE, ALLVD, ALLWK\n*End step",
        -v0,
        period / 20.0
    )
    .unwrap();
    let dir = work_dir("rubber");
    let (dat, log) = run(&dir, "rubber", &input);
    clean(&dir);
    assert!(c > 10.0 * v0);
    let energy = |name: &str| blocks(&dat, name).last().map(|(_, r)| r[0][0]).unwrap();
    let kinetic0 = 0.5 * rho * length * side * side * v0 * v0 * (1.0 - 0.5 / 8.0);
    let (internal, kinetic, damped) = (
        energy("total model internal energy"),
        energy("total model kinetic energy"),
        energy("total model damping dissipation"),
    );
    assert!(
        (internal + kinetic + damped - kinetic0).abs() < 0.02 * kinetic0,
        "energy {internal} + {kinetic} + {damped} J, started with {kinetic0} J\n{log}"
    );
}

/// `EVENTS` in an explicit step: frames where the contact closes and opens, between the
/// evenly spaced ones, in order of time.
#[test]
fn an_explicit_impact_logs_frames_at_the_contact_events() {
    let (bar, gap, v0) = (0.1, 1e-5, 1.0);
    let c = (E / RHO).sqrt();
    let duration = 2.0 * bar / c;
    let arrival = gap / v0;
    let period = arrival + 1.5 * duration;
    let frame = period / 4.0;
    let input = bar_on_wall("Node to surface", 20, gap, v0, frame, period).replace(
        "*Dynamic, Integration=Explicit\n",
        "*Dynamic, Integration=Explicit, Events\n",
    );
    let dir = work_dir("events");
    let (dat, log) = run(&dir, "events", &input);
    clean(&dir);
    let forces = blocks(&dat, "total force");
    let times: Vec<f64> = forces.iter().map(|(t, _)| *t).collect();
    assert!(
        times.windows(2).all(|w| w[1] > w[0]),
        "the frames are not in order of time: {times:?}\n{log}"
    );
    let on_grid = |t: f64| ((t / frame).round() * frame - t).abs() < 1e-5 * frame;
    let grid = times.iter().filter(|&&t| on_grid(t)).count();
    assert_eq!(grid, 4, "{times:?}\n{log}");
    assert!(times.len() - grid >= 2, "no event frames: {times:?}\n{log}");
    // The first frame with a wall force lies right after the arrival (within a few time
    // steps, against a frame interval of a hundred).
    let first = forces
        .iter()
        .find(|(_, rows)| rows[0][0].abs() > 1.0)
        .map(|(t, _)| *t)
        .expect("the wall is hit");
    assert!(
        first > arrival && first - arrival < 0.08 * frame,
        "the first frame with contact is at {first}, the arrival at {arrival}\n{log}"
    );
}

// ---------------------------------------------------------------------------------------
// Stage 3: rigid bodies under finite rotations, mass scaling

/// A steel plate (plane stress, 100 × 20 mm, 10 × 2 CPS4, 10 mm thick) whose left edge is
/// a rigid body held at its reference point, released horizontally under gravity: a
/// physical pendulum that hangs straight down after `K(sin 45°) / ω₀`, and keeps its
/// length while it turns.
#[test]
fn a_plate_hinged_at_a_rigid_edge_swings_as_a_pendulum() {
    let (length, height, g) = (0.1, 0.02, 9.81);
    let id = |i: usize, j: usize| 1 + i + 11 * j;
    let mut s = String::from("*Node, Nset=NALL\n");
    for j in 0..3 {
        for i in 0..11 {
            writeln!(
                s,
                "{}, {}, {}, 0",
                id(i, j),
                length * i as f64 / 10.0,
                height * j as f64 / 2.0
            )
            .unwrap();
        }
    }
    writeln!(
        s,
        "100, 0, {}, 0\n101, 0, {}, 0",
        height / 2.0,
        height / 2.0
    )
    .unwrap();
    s.push_str("*Element, Type=CPS4, Elset=EALL\n");
    let mut e = 0;
    for j in 0..2 {
        for i in 0..10 {
            e += 1;
            writeln!(
                s,
                "{e}, {}, {}, {}, {}",
                id(i, j),
                id(i + 1, j),
                id(i + 1, j + 1),
                id(i, j + 1)
            )
            .unwrap();
        }
    }
    let d = length / 2.0;
    let i2 = (length * length + height * height) / 12.0 + d * d;
    let quarter = 1.854_075 / (g * d / i2).sqrt();
    let period = 1.1 * quarter;
    writeln!(
        s,
        "*Nset, Nset=LEFT\n{}, {}, {}\n*Nset, Nset=TIPNODE\n{}\n\
         *Material, Name=STEEL\n*Elastic\n{E}, 0.3\n*Density\n{RHO}\n\
         *Solid section, Elset=EALL, Material=STEEL\n0.01\n\
         *Rigid body, Nset=LEFT, Ref node=100, Rot node=101\n\
         *Step, Nlgeom\n*Dynamic, Integration=Explicit\n{}, {period}\n*Boundary\n100, 1, 2, 0\n\
         *Dload\nEALL, GRAV, {g}, 0, -1, 0\n*Node print, Nset=TIPNODE\nU\n*End step",
        id(0, 0),
        id(0, 1),
        id(0, 2),
        id(10, 1),
        period / 110.0
    )
    .unwrap();
    let dir = work_dir("pendulum");
    let (dat, log) = run(&dir, "pendulum", &s);
    clean(&dir);
    let mut crossing = None;
    let mut previous: Option<(f64, f64)> = None;
    for (t, rows) in blocks(&dat, "displacements") {
        let (x, y) = (length + rows[0][1], rows[0][2]);
        let (angle, distance) = (y.atan2(x).to_degrees(), x.hypot(y));
        assert!(
            (distance - length).abs() < 1e-3 * length,
            "t = {t}: the plate is {distance} long\n{log}"
        );
        if let Some((t0, a0)) = previous
            && crossing.is_none()
            && a0 > -90.0
            && angle <= -90.0
        {
            crossing = Some(t0 + (a0 + 90.0) / (a0 - angle) * (t - t0));
        }
        previous = Some((t, angle));
    }
    let crossing = crossing.unwrap_or_else(|| panic!("never hangs down\n{log}"));
    assert!(
        (crossing - quarter).abs() < 5e-3 * quarter,
        "hangs down at {crossing} s, expected {quarter} s"
    );
}

/// The tip of a cantilever (20 × 2 × 2 C3D8I, 200 × 20 × 20 mm) whose first element
/// layer is thin (a tenth of the others), released from a static tip load: its zero
/// crossings. `scaling` is the `MASS SCALING` target.
fn released_cantilever(job: &str, scaling: Option<f64>) -> (Vec<f64>, String) {
    let (length, side, cells) = (0.2, 0.02, 20);
    let mut mesh = bar_mesh(cells, 2, 2, [length, side, side], "C3D8I");
    // Station 1 close to the support: x = cell/10.
    let cell = length / cells as f64;
    let mut moved = String::new();
    for line in mesh.lines() {
        let fields: Vec<&str> = line.split(", ").collect();
        if fields.len() == 4
            && let (Ok(id), Ok(x)) = (fields[0].parse::<usize>(), fields[1].parse::<f64>())
            && id > 0
            && (x - cell).abs() < 1e-12
        {
            writeln!(moved, "{id}, {}, {}, {}", 0.1 * cell, fields[2], fields[3]).unwrap();
        } else {
            writeln!(moved, "{line}").unwrap();
        }
    }
    mesh = moved;
    let swing = 0.007;
    let input = format!(
        "{mesh}*Material, Name=STEEL\n*Elastic\n{E}, 0.3\n*Density\n{RHO}\n\
         *Solid section, Elset=EALL, Material=STEEL\n*Boundary\nEND0, 1, 3, 0\n\
         *Step\n*Static\n*Cload\nEND1, 2, {}\n*End step\n\
         *Step\n*Dynamic, Integration=Explicit{}\n{}, {swing}\n*Cload, Op=New\n\
         *Node print, Nset=END1\nU\n*End step\n",
        1000.0 / 9.0,
        scaling.map_or(String::new(), |t| format!(", Mass scaling={t}")),
        swing / 400.0
    );
    let dir = work_dir(job);
    let (dat, log) = run(&dir, job, &input);
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
    let crossings = tip
        .windows(2)
        .filter(|w| w[0].1 * w[1].1 < 0.0)
        .map(|w| w[0].0 + (w[1].0 - w[0].0) * w[0].1 / (w[0].1 - w[1].1))
        .collect();
    (crossings, log)
}

/// Mass scaling to the time step of the regular elements adds mass to the thin layer at
/// the support only: a small fraction of the model, and the swing keeps its frequency.
#[test]
fn mass_scaling_lifts_the_time_step_of_a_thin_layer_only() {
    let (plain, log) = released_cantilever("unscaled", None);
    let stable = |log: &str| -> f64 {
        log.lines()
            .find_map(|l| l.split("stable time step ").nth(1))
            .and_then(|r| r.split_whitespace().next())
            .and_then(|v| v.parse().ok())
            .unwrap()
    };
    let dt = stable(&log);
    // About the stable time step of the regular elements.
    let target = 2.5 * dt;
    let (scaled, scaled_log) = released_cantilever("scaled", Some(target));
    assert!(scaled_log.contains("mass scaling to"), "{scaled_log}");
    assert!(
        stable(&scaled_log) > 2.0 * dt,
        "time step {} s with mass scaling, {dt} s without\n{scaled_log}",
        stable(&scaled_log)
    );
    let added = scaled_log
        .lines()
        .find_map(|l| {
            l.split("% of the model")
                .next()
                .filter(|_| l.contains("mass scaling to"))
        })
        .and_then(|l| l.rsplit('(').next())
        .and_then(|v| v.trim().parse::<f64>().ok())
        .unwrap();
    assert!(
        added < 5.0,
        "mass scaling added {added} %\n{scaled_log}\n{log}"
    );
    assert!(
        plain.len() >= 4 && scaled.len() >= 4,
        "{plain:?} {scaled:?}"
    );
    let frequency = |c: &[f64]| (c.len() - 1) as f64 / (2.0 * (c[c.len() - 1] - c[0]));
    let (f0, f1) = (frequency(&plain), frequency(&scaled));
    assert!(
        (f1 - f0).abs() < 0.01 * f0,
        "swings at {f1} Hz with mass scaling, {f0} Hz without"
    );
}
