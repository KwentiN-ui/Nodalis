//! Ropes on sheaves: node-to-surface contact of general beams with a contact radius
//! (`*BEAM SECTION, CONTACT RADIUS=`), the rope's axis kept that far from the master.
//!
//! - A sheave under a rope pulled by `T` lifts it by `δ` in the middle of supports `2a`
//!   apart, a deflection of 3°: the rope touches it in one point (the report's argument:
//!   `κ₀ = α / 2λ < 1 / R`) and needs `F = 2 T δ / (a - λ tanh(a/λ))` as when the middle node
//!   is moved.
//! - A rope wrapped by 90° around a fixed drum, held at one end and pulled at the other,
//!   slides with the tension ratio of Euler-Eytelwein, `e^(-μφ)` (the benchmark of
//!   Konyukhov and Shala 2022); without friction both ends carry the same tension.
//! - A rope in a groove of its own radius, held out of the plane by nothing but the groove,
//!   carries the force of the first case and stays in the groove's middle.

use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

/// Radius of the rope and of the drum; the rope's axis runs at their sum from the centre.
const ROPE: f64 = 8.0;
const DRUM: f64 = 100.0;
/// Rope modulus, metallic area and the second moment of EI = 2e7 N mm².
const YOUNG: f64 = 1.1e5;
const AREA: f64 = 100.0;
const INERTIA: f64 = 2e7 / YOUNG;

/// Runs `input`, returns the dat file.
fn run(job: &str, input: &str) -> String {
    let dir = std::env::temp_dir().join(format!("plxs-rope-contact-{job}-{}", std::process::id()));
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
    assert!(output.status.success(), "{job} failed:\n{log}");
    assert!(log.contains("Job finished"), "{job}:\n{log}");
    std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap()
}

/// The values of `node` in the last table whose title contains `title`.
fn last_row(dat: &str, title: &str, node: usize) -> Vec<f64> {
    let mut row = None;
    let mut inside = false;
    for line in dat.lines() {
        if line.contains(title) {
            inside = true;
            continue;
        }
        let values: Vec<f64> = line
            .split_whitespace()
            .filter_map(|v| v.parse().ok())
            .collect();
        if inside && values.first() == Some(&(node as f64)) {
            row = Some(values[1..].to_vec());
            inside = false;
        }
    }
    row.unwrap_or_else(|| panic!("no {title} of node {node} in\n{dat}"))
}

/// The rope through the points (nodes 1 to n in the set ROPEN), its section with the
/// contact radius, and a drum: 10 mm thick and 10 mm wide, outer radius [`DRUM`] about the
/// origin from -45° to 135° in steps of half a degree (C3D8, nodes in DRUMN, its outer face
/// the surface DRUMS).
fn rope_and_drum(points: &[[f64; 2]], centre: [f64; 2]) -> String {
    let mut s = String::from("*Node, Nset=ROPEN\n");
    for (i, [x, y]) in points.iter().enumerate() {
        writeln!(s, "{}, {x}, {y}, 0", i + 1).unwrap();
    }
    s.push_str("*Node, Nset=DRUMN\n");
    let steps = 360;
    let id = |j: usize, outer: usize, top: usize| 1001 + 4 * j + 2 * outer + top;
    for j in 0..=steps {
        let angle = (-45.0 + 180.0 * j as f64 / steps as f64).to_radians();
        for (outer, radius) in [DRUM - 10.0, DRUM].into_iter().enumerate() {
            for (top, z) in [-5.0, 5.0].into_iter().enumerate() {
                let (x, y) = (
                    centre[0] + radius * angle.cos(),
                    centre[1] + radius * angle.sin(),
                );
                writeln!(s, "{}, {x}, {y}, {z}", id(j, outer, top)).unwrap();
            }
        }
    }
    s.push_str(
        "*User element, Type=U1, Nodes=2, Integration points=2, Maxdof=6\n\
         *Element, Type=U1, Elset=ROPE\n",
    );
    for i in 1..points.len() {
        writeln!(s, "{i}, {i}, {}", i + 1).unwrap();
    }
    s.push_str("*Element, Type=C3D8, Elset=DRUM\n");
    for j in 0..steps {
        let corners = [
            id(j, 0, 0),
            id(j, 1, 0),
            id(j + 1, 1, 0),
            id(j + 1, 0, 0),
            id(j, 0, 1),
            id(j, 1, 1),
            id(j + 1, 1, 1),
            id(j + 1, 0, 1),
        ];
        let corners: Vec<String> = corners.iter().map(|c| c.to_string()).collect();
        writeln!(s, "{}, {}", 5001 + j, corners.join(", ")).unwrap();
    }
    write!(
        s,
        "*Material, Name=ROPE\n*Elastic\n{YOUNG}, 0.3\n*Material, Name=STEEL\n*Elastic\n210000, 0.3\n\
         *Beam section, Elset=ROPE, Material=ROPE, Section=GENERAL, Contact radius={ROPE}\n\
         {AREA}, {INERTIA}, 0, {INERTIA}, 0.8333333\n0, 0, 1\n\
         *Solid section, Elset=DRUM, Material=STEEL\n\
         *Surface, Name=DRUMS, Type=Element\nDRUM, S4\n*Surface, Name=ROPES, Type=Node\nROPEN\n"
    )
    .unwrap();
    s
}

/// The contact pair of the rope on the drum with friction `mu`.
fn contact(mu: f64) -> String {
    let friction = if mu > 0.0 {
        format!("*Friction\n{mu}, 10000\n")
    } else {
        String::new()
    };
    format!(
        "*Surface interaction, Name=SI\n*Surface behavior, Pressure-overclosure=Hard\n{friction}\
         *Contact pair, Interaction=SI, Type=Node to surface\nROPES, DRUMS\n"
    )
}

#[test]
fn a_sheave_deflecting_a_rope_by_three_degrees_touches_it_in_one_point() {
    let (a, n, tension) = (150.0, 80, 20000.0);
    // The rope's axis lies the rope's radius above the top of the sheave; 0.05 mm of gap.
    let points: Vec<[f64; 2]> = (0..=n)
        .map(|i| [2.0 * a * i as f64 / n as f64, 0.0])
        .collect();
    let gap = 0.05;
    let centre = [a, -(DRUM + ROPE + gap)];
    let lift = a * 1.5_f64.to_radians().tan();
    let middle = n / 2 + 1;
    let input = format!(
        "{}{}*Boundary\nROPEN, 3, 5, 0\n1, 1, 2, 0\n{end}, 2, 2, 0\nDRUMN, 1, 3, 0\n\
         *Step, Nlgeom\n*Static\n*Cload\n{end}, 1, {tension}\n*End step\n\
         *Step, Nlgeom\n*Static\n0.1, 1\n*Boundary\nDRUMN, 2, 2, {}\n\
         *Node print, Nset=ROPEN\nRF, U\n*End step\n",
        rope_and_drum(&points, centre),
        contact(0.0),
        lift + gap,
        end = n + 1,
    );
    let dat = run("sheave", &input);
    let force = -last_row(&dat, "forces", 1)[1] - last_row(&dat, "forces", n + 1)[1];
    let lambda = (YOUNG * INERTIA / tension).sqrt();
    let expected = 2.0 * tension * lift / (a - lambda * (a / lambda).tanh());
    assert!(
        (force - expected).abs() < 1e-2 * expected,
        "{force} vs {expected}"
    );
    // The axis stays the rope's radius above the sheave, which touches it in the middle
    // only: its neighbours hang below the top of the lifted sheave plus the radius.
    let top = last_row(&dat, "displacements", middle)[1];
    assert!((top - lift).abs() < 1e-2 * lift, "{top} vs {lift}");
    for node in [middle - 1, middle + 1] {
        assert!(last_row(&dat, "displacements", node)[1] < top);
    }
}

/// The tensions at the held and the pulled end of a rope wrapped by 90° around the drum.
fn wrapped(job: &str, mu: f64) -> (f64, f64) {
    let axis = DRUM + ROPE;
    // From the left along the top, a quarter round the drum, then straight down.
    let mut points: Vec<[f64; 2]> = (0..20).map(|i| [-200.0 + 10.0 * i as f64, axis]).collect();
    for k in 0..=30 {
        let angle = std::f64::consts::FRAC_PI_2 * (1.0 - k as f64 / 30.0);
        points.push([axis * angle.cos(), axis * angle.sin()]);
    }
    points.extend((1..=20).map(|i| [axis, -10.0 * i as f64]));
    let end = points.len();
    let input = format!(
        "{}{}*Boundary\nROPEN, 3, 5, 0\n1, 1, 2, 0\nDRUMN, 1, 3, 0\n{end}, 1, 1, 0\n\
         *Step, Nlgeom, Inc=1000\n*Static\n0.1, 1\n*Boundary\n{end}, 2, 2, -1\n\
         *Node print, Nset=ROPEN\nRF\n*End step\n",
        rope_and_drum(&points, [0.0, 0.0]),
        contact(mu),
    );
    let dat = run(job, &input);
    (
        -last_row(&dat, "forces", 1)[0],
        -last_row(&dat, "forces", end)[1],
    )
}

#[test]
fn a_rope_sliding_round_a_frictionless_drum_keeps_its_tension() {
    let (held, pulled) = wrapped("drum-frictionless", 0.0);
    assert!(pulled > 1e4, "{pulled}");
    assert!((held / pulled - 1.0).abs() < 1e-2, "{held} vs {pulled}");
}

#[test]
fn a_rope_sliding_round_a_drum_keeps_the_euler_eytelwein_ratio() {
    let mu = 0.2;
    let (held, pulled) = wrapped("drum-friction", mu);
    let expected = (-mu * std::f64::consts::FRAC_PI_2).exp();
    assert!(
        (held / pulled - expected).abs() < 2e-2 * expected,
        "{} vs {expected}",
        held / pulled
    );
}

/// A sheave with a round groove of the rope's own radius about the circle of radius
/// [`DRUM`] + [`ROPE`] around `centre`, from 70° to 110°: C3D8 4 mm thick behind the groove
/// over ±80° across it (nodes in DRUMN, the groove the surface DRUMS); `half` keeps only the
/// side z >= 0, the half of a model cut by its symmetry plane z = 0.
fn grooved_sheave(centre: [f64; 2], half: bool) -> String {
    let steps = 80;
    let (across, from) = if half { (8, 0.0) } else { (16, -80.0) };
    let axis = DRUM + ROPE;
    let id = |j: usize, depth: usize, k: usize| 1001 + 2 * (across + 1) * j + 2 * k + depth;
    let mut s = String::from("*Node, Nset=DRUMN\n");
    for j in 0..=steps {
        let angle = (70.0 + 40.0 * j as f64 / steps as f64).to_radians();
        for k in 0..=across {
            let phi = (from + 160.0 * k as f64 / 16.0).to_radians();
            // depth 1 is the groove, depth 0 the back of the sheave behind it.
            for (depth, rho) in [ROPE + 4.0, ROPE].into_iter().enumerate() {
                let radius = axis - rho * phi.cos();
                let (x, y) = (
                    centre[0] + radius * angle.cos(),
                    centre[1] + radius * angle.sin(),
                );
                writeln!(s, "{}, {x}, {y}, {}", id(j, depth, k), rho * phi.sin()).unwrap();
            }
        }
    }
    s.push_str("*Element, Type=C3D8, Elset=DRUM\n");
    let mut e = 5001;
    for j in 0..steps {
        for k in 0..across {
            let corners = [
                id(j, 0, k),
                id(j, 1, k),
                id(j + 1, 1, k),
                id(j + 1, 0, k),
                id(j, 0, k + 1),
                id(j, 1, k + 1),
                id(j + 1, 1, k + 1),
                id(j + 1, 0, k + 1),
            ];
            let corners: Vec<String> = corners.iter().map(|c| c.to_string()).collect();
            writeln!(s, "{e}, {}", corners.join(", ")).unwrap();
            e += 1;
        }
    }
    s
}

/// The rope of [`a_rope_in_a_groove_of_its_own_radius_is_held_sideways_by_both_flanks`]
/// lifted by the grooved sheave; returns the force on the sheave and the displacement of the
/// rope's middle. `half` models the half z >= 0 only: half a sheave, a rope of half the
/// section under half the tension, held on the symmetry plane.
fn groove(job: &str, half: bool) -> (f64, Vec<f64>) {
    let (a, n) = (150.0, 80);
    let share = if half { 0.5 } else { 1.0 };
    let (area, inertia, tension) = (share * AREA, share * INERTIA, share * 20000.0);
    let mut s = String::from("*Node, Nset=ROPEN\n");
    for i in 0..=n {
        writeln!(s, "{}, {}, 0, 0", i + 1, 2.0 * a * i as f64 / n as f64).unwrap();
    }
    let gap = 0.05;
    s.push_str(&grooved_sheave([a, -(DRUM + ROPE + gap)], half));
    s.push_str(
        "*User element, Type=U1, Nodes=2, Integration points=2, Maxdof=6\n\
         *Element, Type=U1, Elset=ROPE\n",
    );
    for i in 1..=n {
        writeln!(s, "{i}, {i}, {}", i + 1).unwrap();
    }
    write!(
        s,
        "*Material, Name=ROPE\n*Elastic\n{YOUNG}, 0.3\n*Material, Name=STEEL\n*Elastic\n210000, 0.3\n\
         *Beam section, Elset=ROPE, Material=ROPE, Section=GENERAL, Contact radius={ROPE}\n\
         {area}, {inertia}, 0, {inertia}, 0.8333333\n0, 0, 1\n\
         *Solid section, Elset=DRUM, Material=STEEL\n\
         *Surface, Name=DRUMS, Type=Element\nDRUM, S4\n*Surface, Name=ROPES, Type=Node\nROPEN\n"
    )
    .unwrap();
    let lift = a * 1.5_f64.to_radians().tan();
    let middle = n / 2 + 1;
    // The symmetry plane z = 0 holds the rope's out-of-plane displacement and its rotations
    // about x and y.
    let held = if half {
        format!("ROPEN, 3, 5, 0\n1, 1, 2, 0\n{end}, 2, 2, 0\n", end = n + 1)
    } else {
        format!("1, 1, 4, 0\n{end}, 2, 3, 0\n", end = n + 1)
    };
    let input = format!(
        "{s}{}*Boundary\n{held}DRUMN, 1, 3, 0\n\
         *Step, Nlgeom\n*Static\n*Cload\n{end}, 1, {tension}\n*End step\n\
         *Step, Nlgeom\n*Static\n0.1, 1\n*Boundary\nDRUMN, 2, 2, {}\n\
         *Node print, Nset=ROPEN\nRF, U\n*End step\n",
        contact(0.0),
        lift + gap,
        end = n + 1,
    );
    let dat = run(job, &input);
    let force = -last_row(&dat, "forces", 1)[1] - last_row(&dat, "forces", n + 1)[1];
    (force, last_row(&dat, "displacements", middle))
}

/// The force a rope under 20 kN needs to be lifted by the sheave of the groove tests.
fn groove_force() -> (f64, f64) {
    let (a, tension) = (150.0, 20000.0);
    let lift = a * 1.5_f64.to_radians().tan();
    let lambda = (YOUNG * INERTIA / tension).sqrt();
    (
        2.0 * tension * lift / (a - lambda * (a / lambda).tanh()),
        lift,
    )
}

#[test]
fn a_rope_in_a_groove_of_its_own_radius_is_held_sideways_by_both_flanks() {
    // As the sheave lifting the rope by 3°, but the sheave has a groove that fits the rope
    // and nothing else holds the rope out of the plane: only one end against twisting.
    // The rope touches the groove all across; with only its axis node against the nearest
    // groove face it was pushed from flank to flank and the solver cut back to nothing.
    let (force, top) = groove("groove", false);
    let (expected, lift) = groove_force();
    assert!(
        (force - expected).abs() < 2e-2 * expected,
        "{force} vs {expected}"
    );
    assert!((top[1] - lift).abs() < 2e-2 * lift, "{top:?} vs {lift}");
    // Held in the middle of the groove.
    assert!(top[2].abs() < 1e-3 * lift, "{top:?}");
}

#[test]
fn a_rope_on_the_symmetry_plane_of_a_half_sheave_carries_half_the_force() {
    // The groove cut along the rope's axis: the rope lies on the cut, half its circumference
    // against the sheave, the other half off the model, held by the symmetry plane.
    let (force, top) = groove("half-groove", true);
    let (expected, lift) = groove_force();
    let expected = 0.5 * expected;
    assert!(
        (force - expected).abs() < 2e-2 * expected,
        "{force} vs {expected}"
    );
    assert!((top[1] - lift).abs() < 2e-2 * lift, "{top:?} vs {lift}");
}
