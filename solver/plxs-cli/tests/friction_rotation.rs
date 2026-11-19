//! Friction on parts that turn and slide far (screws, pins in bores), plane strain, mm:
//!
//! - A disc pressed into a ring (interference) and turned by half a turn: the friction
//!   moment stays the same all the way round. Friction measured on the total displacement
//!   projected on the tangent of now saw a chord, `r sin φ`, instead of the arc: the moment
//!   fell to half at a quarter turn and turned against the motion beyond.
//! - A block pressed on a foundation and pushed along it: it slides with `μ N`, and an
//!   increment of steady sliding converges in a few iterations (the points that slid keep
//!   sliding in the predictor instead of holding the block with the stick stiffness).

use std::f64::consts::PI;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

const YOUNG: f64 = 210_000.0;

/// A structured grid of CPE4 (`ni` × `nj` cells) through `f(a, b)` for `a, b` in `[0, 1]`,
/// counter-clockwise; nodes from `first`, elements from `first_element`. Returns the
/// node ids by `(i, j)` (a closed ring when `closed`: column `ni` is column 0).
#[allow(clippy::too_many_arguments)]
fn grid(
    s: &mut String,
    elements: &mut String,
    first: usize,
    first_element: &mut usize,
    ni: usize,
    nj: usize,
    closed: bool,
    f: &dyn Fn(f64, f64) -> [f64; 2],
) -> impl Fn(usize, usize) -> usize + use<> {
    let columns = if closed { ni } else { ni + 1 };
    let id = move |i: usize, j: usize| first + (i % columns) + columns * j;
    for j in 0..=nj {
        for i in 0..columns {
            let [x, y] = f(i as f64 / ni as f64, j as f64 / nj as f64);
            writeln!(s, "{}, {x:.12}, {y:.12}, 0", id(i, j)).unwrap();
        }
    }
    for j in 0..nj {
        for i in 0..ni {
            writeln!(
                elements,
                "{}, {}, {}, {}, {}",
                *first_element,
                id(i, j),
                id(i + 1, j),
                id(i + 1, j + 1),
                id(i, j + 1)
            )
            .unwrap();
            *first_element += 1;
        }
    }
    id
}

fn contact(mu: f64, edge: f64) -> String {
    // As Nodalis writes it: K = 10 E / h, stick slope K / 100.
    let k = 10.0 * YOUNG / edge;
    format!(
        "*Surface interaction, Name=SI\n*Surface behavior, Pressure-overclosure=Linear\n\
         {k}, 3\n*Friction\n{mu}, {}\n\
         *Contact pair, Interaction=SI, Type=Surface to surface\nSLAVE, MASTER\n",
        0.01 * k
    )
}

/// Disc (r 2..5.005, rigid inner edge) in a ring (r 5..8, outer edge held), turned by
/// `angle` in `increments`.
fn disc_in_ring(mu: f64, angle: f64, increments: usize) -> String {
    let (n_disc, n_ring) = (32, 41);
    let mut s = String::from("*Node, Nset=NALL\n");
    let mut disc = String::from("*Element, Type=CPE4, Elset=DISC\n");
    let mut ring = String::from("*Element, Type=CPE4, Elset=RING\n");
    let mut e = 1;
    // Angle along a, radius along b: clockwise in (a, b); swap to keep the cells
    // counter-clockwise (radius first).
    let d = grid(&mut s, &mut disc, 1, &mut e, n_disc, 3, true, &|a, b| {
        let r = 2.0 + 3.005 * b;
        [r * (-2.0 * PI * a).cos(), r * (-2.0 * PI * a).sin()]
    });
    let r = grid(&mut s, &mut ring, 1001, &mut e, n_ring, 3, true, &|a, b| {
        let r = 5.0 + 3.0 * b;
        [r * (-2.0 * PI * a).cos(), r * (-2.0 * PI * a).sin()]
    });
    s.push_str("9001, 0, 0, 0\n9002, 0, 0, 0\n");
    s.push_str(&disc);
    s.push_str(&ring);
    s.push_str("*Nset, Nset=INNER\n");
    for i in 0..n_disc {
        writeln!(s, "{},", d(i, 0)).unwrap();
    }
    s.push_str("*Nset, Nset=OUTER\n");
    for i in 0..n_ring {
        writeln!(s, "{},", r(i, 3)).unwrap();
    }
    s.push_str("*Elset, Elset=DISC_OUT, Generate\n");
    writeln!(s, "{}, {}", 1 + 2 * n_disc, 3 * n_disc).unwrap();
    s.push_str("*Elset, Elset=RING_IN, Generate\n");
    writeln!(s, "{}, {}", 3 * n_disc + 1, 3 * n_disc + n_ring).unwrap();
    s.push_str(
        "*Surface, Name=SLAVE, Type=Element\nDISC_OUT, S3\n\
         *Surface, Name=MASTER, Type=Element\nRING_IN, S1\n",
    );
    write!(
        s,
        "*Material, Name=STEEL\n*Elastic\n{YOUNG}, 0.3\n\
         *Solid section, Elset=DISC, Material=STEEL\n1\n\
         *Solid section, Elset=RING, Material=STEEL\n1\n{}\
         *Rigid body, Nset=INNER, Ref node=9001, Rot node=9002\n*Nset, Nset=TURN\n9002\n\
         *Boundary\nOUTER, 1, 2\n9001, 1, 2\n9002, 1, 2\n\
         *Step, Nlgeom, Inc=1000\n*Static\n1, 1\n*Boundary\n9002, 3, 3, 0\n*End step\n\
         *Step, Nlgeom, Inc=1000\n*Static\n{}, 1, 1e-6, {}\n*Boundary\n9002, 3, 3, {}\n\
         *Node print, Nset=TURN\nRF\n*End step\n",
        contact(mu, 2.0 * PI * 5.0 / n_ring as f64),
        1.0 / increments as f64,
        1.0 / increments as f64,
        angle.to_radians(),
    )
    .unwrap();
    s
}

/// Block 10 × 4 (13 × 3 cells) on a foundation 40 × 4 (16 × 4 cells), pressed down by
/// 0.01 at its top, then pushed along by 10.
fn block_on_foundation(mu: f64, increments: usize) -> String {
    let mut s = String::from("*Node, Nset=NALL\n");
    let mut found = String::from("*Element, Type=CPE4, Elset=FOUND\n");
    let mut block = String::from("*Element, Type=CPE4, Elset=BLOCK\n");
    let mut e = 1;
    let f = grid(&mut s, &mut found, 1, &mut e, 16, 4, false, &|a, b| {
        [-10.0 + 40.0 * a, -4.0 + 4.0 * b]
    });
    let b = grid(&mut s, &mut block, 1001, &mut e, 13, 3, false, &|a, b| {
        [10.0 * a, 4.0 * b]
    });
    s.push_str(&found);
    s.push_str(&block);
    s.push_str("*Nset, Nset=BOTTOM\n");
    for i in 0..=16 {
        writeln!(s, "{},", f(i, 0)).unwrap();
    }
    s.push_str("*Nset, Nset=TOP\n");
    for i in 0..=13 {
        writeln!(s, "{},", b(i, 3)).unwrap();
    }
    s.push_str("*Elset, Elset=FOUND_TOP, Generate\n49, 64\n");
    s.push_str("*Elset, Elset=BLOCK_BOTTOM, Generate\n65, 77\n");
    s.push_str(
        "*Surface, Name=SLAVE, Type=Element\nBLOCK_BOTTOM, S1\n\
         *Surface, Name=MASTER, Type=Element\nFOUND_TOP, S3\n",
    );
    write!(
        s,
        "*Material, Name=STEEL\n*Elastic\n{YOUNG}, 0.3\n\
         *Solid section, Elset=FOUND, Material=STEEL\n1\n\
         *Solid section, Elset=BLOCK, Material=STEEL\n1\n{}\
         *Boundary\nBOTTOM, 1, 2\n\
         *Step, Nlgeom, Inc=1000\n*Static\n1, 1\n*Boundary\nTOP, 1, 1, 0\nTOP, 2, 2, -0.01\n\
         *End step\n\
         *Step, Nlgeom, Inc=1000\n*Static\n{}, 1, 1e-6, {}\n*Boundary\nTOP, 1, 1, 10\n\
         TOP, 2, 2, -0.01\n*Node print, Nset=TOP, Totals=Only\nRF\n*End step\n",
        contact(mu, 2.5),
        1.0 / increments as f64,
        1.0 / increments as f64,
    )
    .unwrap();
    s
}

struct Run {
    /// Per printed time: the first force row (x, y, z) of the last `*NODE PRINT` block.
    forces: Vec<(f64, [f64; 3])>,
    /// Iterations per step.
    iterations: Vec<u32>,
}

fn run(dir: &Path, job: &str, input: &str) -> Run {
    std::fs::write(dir.join(format!("{job}.inp")), input).unwrap();
    let output = Command::new(plxs())
        .arg(job)
        .arg("--threads")
        .arg("2")
        .current_dir(dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned();
    assert!(
        output.status.success() && log.contains("Job finished"),
        "{job} failed:\n{log}"
    );
    let mut iterations = Vec::new();
    for line in log.lines() {
        let line = line.trim();
        if line.starts_with("STEP") {
            iterations.push(0);
        } else if line.starts_with("iteration")
            && let Some(last) = iterations.last_mut()
        {
            *last += 1;
        }
    }
    let dat = std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap();
    let mut forces = Vec::new();
    let mut time = None;
    for line in dat.lines() {
        if line.contains("forces (fx,fy,fz)") || line.contains("total force (fx,fy,fz)") {
            time = line.split_whitespace().last().and_then(|t| t.parse().ok());
        } else if let Some(t) = time {
            let values: Vec<f64> = line
                .split_whitespace()
                .filter_map(|v| v.parse().ok())
                .collect();
            if values.len() >= 3 {
                let v = &values[values.len() - 3..];
                forces.push((t, [v[0], v[1], v[2]]));
                time = None;
            }
        }
    }
    assert!(!forces.is_empty(), "no forces in\n{dat}");
    Run { forces, iterations }
}

fn work_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("plxs-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
fn the_friction_moment_of_a_disc_turned_half_round_in_a_ring_stays_the_same() {
    let dir = work_dir("friction-turn");
    let run = run(&dir, "turn", &disc_in_ring(0.2, 180.0, 20));
    let _ = std::fs::remove_dir_all(&dir);
    // The moment about z on the rotation node (its third "force").
    let moments: Vec<(f64, f64)> = run.forces.iter().map(|(t, f)| (*t, f[2])).collect();
    let first = moments[0].1;
    assert!(first > 0.0, "{moments:?}");
    for (t, m) in &moments {
        assert!(
            (m - first).abs() < 0.01 * first,
            "t = {t}: moment {m} against {first} at the start: {moments:?}"
        );
    }
    // Turned by 9° per increment: the first iteration moves the disc along the tangents of
    // the turn and presses it into the ring by the chord's error, which the following
    // iterations take back (about eleven per increment now).
    assert!(run.iterations[1] <= 20 * 13, "{:?}", run.iterations);
}

#[test]
fn a_block_pushed_along_slides_with_mu_n_in_few_iterations() {
    let dir = work_dir("friction-slide");
    let mu = 0.3;
    let run = run(&dir, "slide", &block_on_foundation(mu, 20));
    let _ = std::fs::remove_dir_all(&dir);
    // The reactions of the top: pressed down (negative) and pushed along.
    for (t, [fx, fy, _]) in &run.forces {
        assert!(*fy < 0.0, "{:?}", run.forces);
        assert!(
            (fx - mu * fy.abs()).abs() < 0.01 * mu * fy.abs(),
            "t = {t}: pushed with {fx} at a normal force of {fy}"
        );
    }
    assert!(run.iterations[1] <= 20 * 3, "{:?}", run.iterations);
}
