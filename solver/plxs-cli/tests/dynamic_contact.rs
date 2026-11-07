//! Contact in `*DYNAMIC` steps, against closed-form solutions.
//!
//! - Impact of an elastic bar on a rigid wall (3D, node to surface, surface to surface and
//!   mortar): a bar of length `L` hitting the wall at `v₀` stays in contact for the time a
//!   wave takes to run to the free end and back, `2L/c` with `c = √(E/ρ)`, and leaves it at
//!   `−v₀` (the wave theory of St. Venant; without lateral contraction, `ν = 0`). The
//!   impulse the wall takes is `2 m v₀`.
//! - A block sliding on a fixed plate under gravity (plane strain, friction `μ`): it
//!   decelerates by `μ g` and stops after `v₀/(μ g)` at the distance `v₀²/(2 μ g)`.
//! - A block dropped on a plate (plane strain) with time steps far longer than the impact:
//!   it falls freely until `√(2h/g)`, does not pass through the plate, bounces and never
//!   rises above where it fell from.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

fn work_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("plxs-dyncontact-{name}-{}", std::process::id()));
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

// ---------------------------------------------------------------------------------------
// Bar impact

const E: f64 = 2.1e11;
const RHO: f64 = 7850.0;
const BAR: f64 = 0.1;
const SIDE: f64 = 0.01;
const CELLS: usize = 20;
const GAP: f64 = 1e-5;
const V0: f64 = 1.0;
/// Contact stiffness (Pa/m): stiff enough that the overclosure under the impact pressure
/// `ρ c v₀` (a few hundredths of a micrometre) leaves the waves in the bar alone.
const PENALTY: f64 = 1e15;

/// A steel bar (`CELLS` C3D8 along x, one across) moving at `V0` towards a fixed steel
/// block at `GAP` from its end.
fn bar_impact(kind: &str, period: f64, dt: f64) -> String {
    let mut s = String::from("*Node\n");
    let id = |i: usize, c: usize| 1 + 4 * i + c;
    let corners = [(0.0, 0.0), (SIDE, 0.0), (SIDE, SIDE), (0.0, SIDE)];
    for i in 0..=CELLS {
        for (c, (y, z)) in corners.iter().enumerate() {
            writeln!(
                s,
                "{}, {}, {y}, {z}",
                id(i, c),
                BAR * i as f64 / CELLS as f64
            )
            .unwrap();
        }
    }
    // The wall: one brick 10 mm thick, 30 mm square, centred on the bar's axis.
    let x0 = BAR + GAP;
    let mut wall = Vec::new();
    for (k, x) in [x0, x0 + 0.01].iter().enumerate() {
        for (c, (y, z)) in [(-0.01, -0.01), (0.02, -0.01), (0.02, 0.02), (-0.01, 0.02)]
            .iter()
            .enumerate()
        {
            let n = 1001 + 4 * k + c;
            writeln!(s, "{n}, {x}, {y}, {z}").unwrap();
            wall.push(n);
        }
    }
    s.push_str("*Element, Type=C3D8, Elset=BAR\n");
    for i in 0..CELLS {
        writeln!(
            s,
            "{}, {}, {}, {}, {}, {}, {}, {}, {}",
            i + 1,
            id(i, 0),
            id(i, 1),
            id(i, 2),
            id(i, 3),
            id(i + 1, 0),
            id(i + 1, 1),
            id(i + 1, 2),
            id(i + 1, 3)
        )
        .unwrap();
    }
    s.push_str(
        "*Element, Type=C3D8, Elset=WALL\n1001, 1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008\n",
    );
    s.push_str("*Elset, Elset=EALL\nBAR, WALL\n*Nset, Nset=WALLN\n");
    for n in &wall {
        writeln!(s, "{n},").unwrap();
    }
    s.push_str("*Nset, Nset=BARN\n");
    for i in 0..=CELLS {
        for c in 0..4 {
            writeln!(s, "{},", id(i, c)).unwrap();
        }
    }
    // Slave: the bar's end face (x = BAR, face 2 of the last element); master: the wall
    // face at x = x0 (face 1 of the brick).
    writeln!(
        s,
        "*Elset, Elset=BAREND\n{CELLS}\n*Surface, Name=SLAVE, Type=Element\nBAREND, S2\n\
         *Surface, Name=MASTER, Type=Element\nWALL, S1"
    )
    .unwrap();
    writeln!(
        s,
        "*Material, Name=STEEL\n*Elastic\n{E}, 0\n*Density\n{RHO}\n\
         *Solid section, Elset=BAR, Material=STEEL\n*Solid section, Elset=WALL, Material=STEEL\n\
         *Surface interaction, Name=SI\n*Surface behavior, Pressure-overclosure=Linear\n{PENALTY}\n\
         *Contact pair, Interaction=SI, Type={kind}\nSLAVE, MASTER\n\
         *Initial conditions, Type=Velocity\nBARN, 1, {V0}\n\
         *Step, Inc=100000\n*Dynamic, Direct\n{dt}, {period}\n\
         *Boundary\nWALLN, 1, 3, 0\n\
         *Node print, Nset=WALLN, Totals=Only\nRF\n\
         *Node print, Nset=BARN\nV\n\
         *El print, Elset=EALL, Totals=Only\nALLIE, ALLKE, ALLVD\n*End step"
    )
    .unwrap();
    s
}

/// The momentum per mass of the bar: the mean of `v₁` with the end stations at half
/// weight (the row sums of the mass matrix of a uniform mesh).
fn mean_velocity(rows: &[Vec<f64>]) -> f64 {
    let mut sum = 0.0;
    let mut weight = 0.0;
    for row in rows {
        let node = row[0] as usize;
        let station = (node - 1) / 4;
        let w = if station == 0 || station == CELLS {
            0.5
        } else {
            1.0
        };
        sum += w * row[1];
        weight += w;
    }
    sum / weight
}

fn check_bar_impact(kind: &str, job: &str) {
    let c = (E / RHO).sqrt();
    let duration = 2.0 * BAR / c;
    let arrival = GAP / V0;
    let dt = 2.5e-7;
    let period = arrival + 1.5 * duration;
    let dir = work_dir(job);
    let (dat, log) = run(&dir, job, &bar_impact(kind, period, dt));
    if std::env::var_os("PLXS_KEEP").is_none() {
        let _ = std::fs::remove_dir_all(&dir);
    }
    let forces = blocks(&dat, "total force");
    let velocities = blocks(&dat, "velocities");
    assert!(!forces.is_empty() && !velocities.is_empty(), "{dat}\n{log}");
    // Contact time and impulse from the wall's reaction.
    let mut first = None;
    let mut last = None;
    let mut impulse = 0.0;
    let mut t_prev = 0.0;
    let mut f_prev = 0.0;
    for (t, rows) in &forces {
        // The wall pushes back: its reaction points along +x (towards the wall's support).
        let f = rows[0][0].abs();
        impulse += 0.5 * (f + f_prev) * (t - t_prev);
        if f > 1e-3 * RHO * c * V0 * SIDE * SIDE {
            first.get_or_insert(*t);
            last = Some(*t);
        }
        t_prev = *t;
        f_prev = f;
    }
    let mass = RHO * BAR * SIDE * SIDE;
    let (first, last) = (first.unwrap(), last.unwrap());
    let contact = last - first;
    // The coarse mesh disperses the wave that releases the bar: the last part of the
    // contact is a decaying tail.
    assert!(
        (first - arrival).abs() < 4.0 * dt,
        "{kind}: contact starts at {first}, expected {arrival}"
    );
    assert!(
        (contact - duration).abs() < 0.15 * duration,
        "{kind}: contact lasts {contact} s, expected {duration} s"
    );
    assert!(
        (impulse - 2.0 * mass * V0).abs() < 0.05 * 2.0 * mass * V0,
        "{kind}: impulse {impulse}, expected {}",
        2.0 * mass * V0
    );
    // In the middle of the contact the wall takes the pressure `ρ c v₀` (the springs are
    // damped: no chatter).
    let plateau: Vec<f64> = forces
        .iter()
        .filter(|(t, _)| (t - (arrival + 0.2 * duration)) * (t - (arrival + 0.6 * duration)) < 0.0)
        .map(|(_, rows)| rows[0][0].abs())
        .collect();
    let expected = RHO * c * V0 * SIDE * SIDE;
    for f in &plateau {
        assert!(
            (f - expected).abs() < 0.05 * expected,
            "{kind}: contact force {f} N in the middle of the impact, expected {expected} N"
        );
    }
    let (_, rows) = velocities.last().unwrap();
    let rebound = mean_velocity(rows);
    assert!(
        (rebound + V0).abs() < 0.05 * V0,
        "{kind}: the bar leaves at {rebound} m/s, expected {}",
        -V0
    );
    // Energy: the bar leaves with (almost) all of it, the contact damping takes little.
    let energy = |name: &str| {
        blocks(&dat, name)
            .last()
            .map(|(_, rows)| rows[0][0])
            .unwrap()
    };
    let kinetic0 = 0.5 * mass * V0 * V0;
    let (internal, kinetic, damped) = (
        energy("total model internal energy"),
        energy("total model kinetic energy"),
        energy("total model damping dissipation"),
    );
    assert!(
        damped < 0.05 * kinetic0,
        "{kind}: the contact damping took {damped} J of {kinetic0} J"
    );
    assert!(
        (internal + kinetic + damped - kinetic0).abs() < 0.02 * kinetic0,
        "{kind}: energy {internal} + {kinetic} + {damped} J, started with {kinetic0} J"
    );
}

#[test]
fn an_elastic_bar_bounces_off_a_wall_node_to_surface() {
    check_bar_impact("Node to surface", "bar-n2s");
}

#[test]
fn an_elastic_bar_bounces_off_a_wall_surface_to_surface() {
    check_bar_impact("Surface to surface", "bar-s2s");
}

// ---------------------------------------------------------------------------------------
// A block on a plate (plane strain)

const G: f64 = 9.81;
/// The block: 10 mm square, 2 × 2 CPE4, its lower left corner at (10 mm, 5 mm + gap).
const BLOCK: f64 = 0.01;
/// The node at the block's lower left corner.
const CORNER: usize = 1001;

/// A steel plate 100 × 5 mm (20 × 1 CPE4, fixed at the bottom) with the block above it at
/// `gap`, under gravity; `contact` is the interaction's keywords after its name, `extra`
/// goes before the step, `step` is the `*DYNAMIC` data line.
fn block_on_plate(gap: f64, contact: &str, extra: &str, step: &str) -> String {
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
                CORNER + i + 3 * j,
                0.01 + 0.005 * i as f64,
                0.005 + gap + 0.005 * j as f64
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
            let n = CORNER + i + 3 * j;
            writeln!(
                s,
                "{}, {}, {}, {}, {}",
                101 + i + 2 * j,
                n,
                n + 1,
                n + 4,
                n + 3
            )
            .unwrap();
        }
    }
    s.push_str("*Elset, Elset=EALL\nPLATE, BLOCK\n*Nset, Nset=BOTTOM, Generate\n1, 21, 1\n");
    s.push_str("*Nset, Nset=BLOCKN, Generate\n1001, 1009, 1\n*Nset, Nset=CORNER\n1001\n");
    s.push_str(
        "*Elset, Elset=BLOCKLOW\n101, 102\n*Surface, Name=SLAVE, Type=Element\nBLOCKLOW, S1\n\
         *Surface, Name=MASTER, Type=Element\nPLATE, S3\n",
    );
    writeln!(
        s,
        "*Material, Name=STEEL\n*Elastic\n{E}, 0.3\n*Density\n{RHO}\n\
         *Solid section, Elset=EALL, Material=STEEL\n1\n\
         *Surface interaction, Name=SI\n{contact}\n\
         *Contact pair, Interaction=SI, Type=Surface to surface\nSLAVE, MASTER\n{extra}\
         *Step, Inc=100000\n*Dynamic, Direct\n{step}\n*Boundary\nBOTTOM, 1, 2, 0\n\
         *Dload\nEALL, GRAV, {G}, 0, -1, 0\n\
         *Node print, Nset=CORNER\nU\n\
         *El print, Elset=EALL, Totals=Only\nALLIE, ALLKE, ALLWK, ALLVD\n*End step"
    )
    .unwrap();
    s
}

/// The displacements of the block's corner: `(time, [u1, u2])`.
fn corner(dat: &str) -> Vec<(f64, [f64; 2])> {
    blocks(dat, "displacements")
        .into_iter()
        .map(|(t, rows)| (t, [rows[0][1], rows[0][2]]))
        .collect()
}

#[test]
fn a_block_sliding_on_a_plate_stops_by_friction() {
    let (v0, mu) = (0.3, 0.3);
    let stop = v0 / (mu * G);
    let distance = v0 * v0 / (2.0 * mu * G);
    let contact = format!(
        "*Surface behavior, Pressure-overclosure=Linear\n{PENALTY}\n*Friction\n{mu}, {}",
        PENALTY * 0.01
    );
    let input = block_on_plate(
        0.0,
        &contact,
        &format!("*Initial conditions, Type=Velocity\nBLOCKN, 1, {v0}\n"),
        "0.001, 0.15",
    );
    let dir = work_dir("slide");
    let (dat, _) = run(&dir, "slide", &input);
    if std::env::var_os("PLXS_KEEP").is_none() {
        let _ = std::fs::remove_dir_all(&dir);
    }
    let path = corner(&dat);
    for (t, u) in &path {
        let expected = if *t < stop {
            v0 * t - 0.5 * mu * G * t * t
        } else {
            distance
        };
        assert!(
            (u[0] - expected).abs() < 0.02 * distance,
            "t = {t}: the block slid {} m, expected {expected} m",
            u[0]
        );
        // It stays on the plate.
        assert!(u[1].abs() < 1e-6, "t = {t}: the block lifted by {} m", u[1]);
    }
    let (_, end) = path.last().unwrap();
    assert!(
        (end[0] - distance).abs() < 0.01 * distance,
        "stops after {} m, expected {distance} m",
        end[0]
    );
}

#[test]
fn a_block_dropped_on_a_plate_lands_in_time_and_gains_no_energy() {
    let h = 0.001;
    let landing = (2.0 * h / G).sqrt();
    let contact = format!("*Surface behavior, Pressure-overclosure=Linear\n{PENALTY}");
    // Time steps of a millisecond against an impact of microseconds.
    let input = block_on_plate(h, &contact, "", "0.001, 0.04");
    let dir = work_dir("drop");
    let (dat, _) = run(&dir, "drop", &input);
    if std::env::var_os("PLXS_KEEP").is_none() {
        let _ = std::fs::remove_dir_all(&dir);
    }
    let path = corner(&dat);
    // Free fall until it lands.
    for (t, u) in path.iter().filter(|(t, _)| *t < landing) {
        let expected = -0.5 * G * t * t;
        assert!(
            (u[1] - expected).abs() < 1e-3 * h,
            "t = {t}: fell {} m, expected {expected} m",
            u[1]
        );
    }
    // Never through the plate.
    let lowest = path.iter().map(|(_, u)| u[1]).fold(f64::INFINITY, f64::min);
    assert!(lowest > -h - 1e-6, "the block sank to {lowest} m");
    // It bounces, but never above the height it fell from (an impact computed with steps
    // longer than it lasts must not give energy; the increments are split around it).
    let after: Vec<f64> = path
        .iter()
        .filter(|(t, _)| *t > landing + 0.002)
        .map(|(_, u)| u[1])
        .collect();
    let highest = after.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    assert!(highest < 1e-3 * h, "the block bounced to {highest} m");
    assert!(highest > -0.7 * h, "the block hardly bounced: {highest} m");
    // No energy from nowhere: kinetic + internal never exceeds the work of gravity.
    let series = |name: &str| -> Vec<f64> {
        blocks(&dat, name)
            .into_iter()
            .map(|(_, rows)| rows[0][0])
            .collect()
    };
    let (internal, kinetic, work) = (
        series("total model internal energy"),
        series("total model kinetic energy"),
        series("total model external work"),
    );
    let mass = RHO * BLOCK * BLOCK;
    let scale = mass * G * h;
    for ((i, k), w) in internal.iter().zip(&kinetic).zip(&work) {
        assert!(
            i + k - w < 0.01 * scale,
            "energy {i} + {k} J exceeds the work {w} J of gravity"
        );
    }
}

/// The block falls 50 mm and the first time step (120 ms) takes it 20 mm below the plate's
/// top, through the 5 mm plate: at the start of the step the plate was far beyond the
/// master faces the block's points could touch. The step converged with the block below
/// the plate and not one spring (it fell through); such an increment is split until the
/// contact is seen.
#[test]
fn a_block_reaching_the_plate_within_one_increment_does_not_fall_through() {
    let h = 0.05;
    let contact = format!("*Surface behavior, Pressure-overclosure=Linear\n{PENALTY}");
    let input = block_on_plate(h, &contact, "", "0.12, 0.24");
    let dir = work_dir("drop-through");
    let (dat, log) = run(&dir, "drop-through", &input);
    if std::env::var_os("PLXS_KEEP").is_none() {
        let _ = std::fs::remove_dir_all(&dir);
    }
    let path = corner(&dat);
    let lowest = path.iter().map(|(_, u)| u[1]).fold(f64::INFINITY, f64::min);
    assert!(lowest > -h - 1e-4, "the block sank to {lowest} m\n{log}");
}

#[test]
fn an_elastic_bar_bounces_off_a_wall_mortar() {
    check_bar_impact("Mortar", "bar-mortar");
}

/// `*DYNAMIC, EVENTS`: with time steps longer than the impact, the results get frames at
/// the moment the contact closes and at the sub-increments of the impact, between the
/// evenly spaced increments.
#[test]
fn an_impact_logs_frames_at_the_contact_events() {
    let c = (E / RHO).sqrt();
    let duration = 2.0 * BAR / c;
    let arrival = GAP / V0;
    // Two time steps over the contact; the first frame lies before the arrival.
    let dt = 0.6 * duration;
    let period = 4.0 * dt;
    let input = bar_impact("Node to surface", period, dt)
        .replace("*Dynamic, Direct\n", "*Dynamic, Direct, Events\n");
    let dir = work_dir("bar-events");
    let (dat, log) = run(&dir, "bar-events", &input);
    if std::env::var_os("PLXS_KEEP").is_none() {
        let _ = std::fs::remove_dir_all(&dir);
    }
    let forces = blocks(&dat, "total force");
    let times: Vec<f64> = forces.iter().map(|(t, _)| *t).collect();
    assert!(
        times.windows(2).all(|w| w[1] > w[0]),
        "the frames are not in order of time: {times:?}\n{log}"
    );
    // The four increments and the events of the impact.
    // The `.dat` prints seven significant digits.
    let on_grid = |t: f64| ((t / dt).round() * dt - t).abs() < 1e-5 * dt;
    let grid = times.iter().filter(|&&t| on_grid(t)).count();
    let events = times.len() - grid;
    assert_eq!(grid, 4, "{times:?}\n{log}");
    assert!(events >= 4, "only {events} event frames: {times:?}\n{log}");
    // The contact closes at the arrival: the first frame with a wall force lies right
    // after it, far closer than a time step.
    let threshold = 1e-3 * RHO * c * V0 * SIDE * SIDE;
    let first = forces
        .iter()
        .find(|(_, rows)| rows[0][0].abs() > threshold)
        .map(|(t, _)| *t)
        .expect("the wall is hit");
    assert!(
        first > arrival && first - arrival < 0.05 * dt,
        "the first frame with contact is at {first}, the arrival at {arrival} (dt {dt})\n{log}"
    );
    // The frame before it shows no contact yet.
    let before = times.iter().rfind(|&&t| t < first).unwrap();
    assert!(first - before < 0.1 * dt, "{times:?}");
}
