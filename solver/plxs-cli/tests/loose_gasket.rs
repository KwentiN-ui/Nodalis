//! A rubber gasket in a groove, pressed by a plate that then lifts off and slides away, as
//! in a fitted Ogden O-ring model that stopped converging twice on its way:
//!
//! - Pressed into the kink between the fillet of the groove bottom and its flank, a contact
//!   point lay beneath both master faces, its foot point clamped to their shared corner.
//!   Both faces were equally close, and the tie went to the one whose normal pushed the
//!   point over to the other side, where the other face won: the point flipped between
//!   them in every Newton iteration and the residual alternated between two values until
//!   the increment was cut back to nothing. The normal there is now the direction to the
//!   corner, which turns continuously from one face's normal to the other's.
//! - Once the plate lifted off, the gasket lay loose in its groove, held only by its weak
//!   springs: a residual force far below the tolerance moved it by tenths of a millimetre
//!   and every iteration threw it against another wall. Its springs now also resist the
//!   motion within each increment (`*SPRING, STABILIZE`), with no force left over.
//!
//! Checked here: a soft block pressed into a shallow V stays centred and the step finishes,
//! and a block that a plate presses into a groove, lets go of and slides past finishes the
//! step with its stabilized weak springs and stays in the groove.

use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

/// Runs `input` as job `job`; returns whether it finished, the log, the `.sta` rows and the
/// `.dat` text.
fn run(job: &str, input: &str) -> (bool, String, Vec<Vec<f64>>, String) {
    let dir = std::env::temp_dir().join(format!("plxs-gasket-{job}-{}", std::process::id()));
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
    // Columns: step, increment, attempt, iterations, total, step time, increment.
    let rows: Vec<Vec<f64>> = (sta.lines())
        .map(|l| {
            l.split_whitespace()
                .filter_map(|t| t.parse::<f64>().ok())
                .collect::<Vec<_>>()
        })
        .filter(|r| r.len() == 7)
        .collect();
    let dat = std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap_or_default();
    let _ = std::fs::remove_dir_all(&dir);
    (output.status.success(), log, rows, dat)
}

/// The displacements of the last block of `*NODE PRINT` output in the `.dat`.
fn last_displacement(dat: &str) -> [f64; 2] {
    let line = (dat.lines().rev())
        .find(|l| {
            let f: Vec<&str> = l.split_whitespace().collect();
            f.len() == 4 && f.iter().all(|v| v.parse::<f64>().is_ok())
        })
        .unwrap_or_else(|| panic!("no displacements:\n{dat}"));
    let f: Vec<f64> = line
        .split_whitespace()
        .map(|v| v.parse().unwrap())
        .collect();
    [f[1], f[2]]
}

/// Nodes of a structured grid of `nx` x `ny` CPE4 elements over `[x0, x1] x [y0, y1]` (the
/// height of the bottom row raised by `bottom(x)`), numbered from `first`.
struct Grid {
    first: usize,
    nx: usize,
}

impl Grid {
    fn node(&self, i: usize, j: usize) -> usize {
        self.first + i + (self.nx + 1) * j
    }
}

#[allow(clippy::too_many_arguments)]
fn grid(
    s: &mut String,
    first: usize,
    [x0, x1]: [f64; 2],
    [y0, y1]: [f64; 2],
    nx: usize,
    ny: usize,
    top: impl Fn(f64) -> f64,
) -> Grid {
    for j in 0..=ny {
        for i in 0..=nx {
            let x = x0 + (x1 - x0) * i as f64 / nx as f64;
            let y = y0 + (y1 + top(x) - y0) * j as f64 / ny as f64;
            writeln!(s, "{}, {x:e}, {y:e}", first + i + (nx + 1) * j).unwrap();
        }
    }
    Grid { first, nx }
}

/// The elements of `grid` as an element set `name` from element `first` on; returns the
/// element of each (i, j).
fn elements(s: &mut String, grid: &Grid, ny: usize, name: &str, first: usize) -> Vec<Vec<usize>> {
    writeln!(s, "*ELEMENT, TYPE=CPE4, ELSET={name}").unwrap();
    let mut ids = vec![vec![0; ny]; grid.nx];
    let mut e = first;
    for j in 0..ny {
        for (i, column) in ids.iter_mut().enumerate() {
            let n = [
                grid.node(i, j),
                grid.node(i + 1, j),
                grid.node(i + 1, j + 1),
                grid.node(i, j + 1),
            ];
            writeln!(s, "{e}, {}, {}, {}, {}", n[0], n[1], n[2], n[3]).unwrap();
            column[j] = e;
            e += 1;
        }
    }
    ids
}

/// A soft block (1 x 0.5) pressed by 0.1 into a stiff base whose top is a V with flanks at
/// 5 degrees, frictionless node-to-surface contact. The block's middle bottom node sits on
/// the bottom of the V and goes in beneath the kink; only the V holds the block sideways.
fn v_kink() -> String {
    let slope = 5f64.to_radians().tan();
    let mut s = String::from("*NODE\n");
    let base = grid(&mut s, 1, [-1.0, 1.0], [-0.5, 0.0], 8, 4, |x| {
        x.abs() * slope
    });
    let block = grid(&mut s, 100, [-0.5, 0.5], [0.0, 0.5], 8, 4, |_| 0.0);
    let base_elements = elements(&mut s, &base, 4, "BASE", 1);
    let block_elements = elements(&mut s, &block, 4, "BLOCK", 100);
    s.push_str("*SURFACE, NAME=SLAVE, TYPE=ELEMENT\n");
    for column in &block_elements {
        writeln!(s, "{}, S1", column[0]).unwrap();
    }
    s.push_str("*SURFACE, NAME=MASTER, TYPE=ELEMENT\n");
    for column in &base_elements {
        writeln!(s, "{}, S3", column[3]).unwrap();
    }
    s.push_str("*NSET, NSET=FIX\n");
    for i in 0..=8 {
        writeln!(s, "{},", base.node(i, 0)).unwrap();
    }
    s.push_str("*NSET, NSET=TOP\n");
    for i in 0..=8 {
        writeln!(s, "{},", block.node(i, 4)).unwrap();
    }
    writeln!(s, "*NSET, NSET=MIDDLE\n{},", block.node(4, 0)).unwrap();
    s.push_str(
        "*MATERIAL, NAME=SOFT\n*ELASTIC\n1000, 0.3\n*MATERIAL, NAME=HARD\n*ELASTIC\n1e6, 0.3\n\
*SOLID SECTION, ELSET=BLOCK, MATERIAL=SOFT\n*SOLID SECTION, ELSET=BASE, MATERIAL=HARD\n\
*SURFACE INTERACTION, NAME=SMOOTH\n*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=LINEAR\n1e6\n\
*CONTACT PAIR, INTERACTION=SMOOTH, TYPE=NODE TO SURFACE\nSLAVE, MASTER\n\
*STEP, NLGEOM, INC=100\n*STATIC\n0.25, 1\n*BOUNDARY\nFIX, 1, 2, 0\nTOP, 2, 2, -0.1\n\
*NODE PRINT, NSET=MIDDLE\nU\n*END STEP\n",
    );
    s
}

#[test]
fn a_block_pressed_into_a_shallow_v_stays_centred() {
    let (finished, log, rows, dat) = run("vkink", &v_kink());
    assert!(finished, "the step did not finish:\n{log}");
    assert_eq!(rows.last().unwrap()[5], 1.0, "{rows:?}");
    let cutbacks = rows.iter().filter(|r| r[2] > 1.0).count();
    assert!(cutbacks <= 1, "cut back {cutbacks} times: {rows:?}");
    assert!(
        rows.iter().all(|r| r[3] <= 6.0),
        "too many iterations: {rows:?}"
    );
    // Before: it drifted sideways by the noise that decided between the two faces.
    let [x, y] = last_displacement(&dat);
    assert!(x.abs() < 1e-9, "the middle node moved sideways by {x}");
    assert!(y < 0.0 && y > -1e-3, "the middle node went in by {y}");
}

/// A rubber block (1 x 1, Neo-Hooke) in a groove 1.2 wide, held only by contact and by weak
/// springs to ground (`stabilize` on their `*SPRING`), pressed 0.2 down by a plate, which
/// then lifts 0.4 and slides 0.5 sideways; friction 0.5. Bulging, the block touches the
/// walls of the groove.
fn gasket(stabilize: &str) -> String {
    let mut s = String::from("*NODE\n");
    let bottom = grid(&mut s, 1, [-0.8, 0.8], [-0.3, 0.0], 16, 3, |_| 0.0);
    let left = grid(&mut s, 101, [-0.8, -0.6], [0.0, 0.9], 2, 12, |_| 0.0);
    let right = grid(&mut s, 201, [0.6, 0.8], [0.0, 0.9], 2, 12, |_| 0.0);
    let block = grid(&mut s, 301, [-0.5, 0.5], [0.0, 1.0], 10, 10, |_| 0.0);
    let plate = grid(&mut s, 501, [-0.55, 0.55], [1.0, 1.2], 22, 2, |_| 0.0);
    let bottom_elements = elements(&mut s, &bottom, 3, "BOTTOM", 1);
    let left_elements = elements(&mut s, &left, 12, "LEFT", 101);
    let right_elements = elements(&mut s, &right, 12, "RIGHT", 201);
    let block_elements = elements(&mut s, &block, 10, "BLOCK", 301);
    let plate_elements = elements(&mut s, &plate, 2, "PLATE", 501);
    let block_nodes: Vec<usize> = (0..=10)
        .flat_map(|j| (0..=10).map(move |i| (i, j)))
        .map(|(i, j)| block.node(i, j))
        .collect();
    // Weak springs as Nodalis writes them: 1e-6 E L spread over the nodes.
    let k = 1e-6 * 1e3 / block_nodes.len() as f64;
    let mut e = 1000;
    for dof in 1..=2 {
        writeln!(s, "*ELEMENT, TYPE=SPRING1, ELSET=WEAK{dof}").unwrap();
        for n in &block_nodes {
            writeln!(s, "{e}, {n}").unwrap();
            e += 1;
        }
        writeln!(s, "*SPRING, ELSET=WEAK{dof}{stabilize}\n{dof}\n{k:e}").unwrap();
    }
    s.push_str("*SURFACE, NAME=GASKET, TYPE=ELEMENT\n");
    for (i, column) in block_elements.iter().enumerate() {
        writeln!(s, "{}, S1\n{}, S3", column[0], column[9]).unwrap();
        writeln!(
            s,
            "{}, S4\n{}, S2",
            block_elements[0][i], block_elements[9][i]
        )
        .unwrap();
    }
    s.push_str("*SURFACE, NAME=GROOVE, TYPE=ELEMENT\n");
    for column in &bottom_elements {
        writeln!(s, "{}, S3", column[2]).unwrap();
    }
    for j in 0..12 {
        writeln!(
            s,
            "{}, S2\n{}, S4",
            left_elements[1][j], right_elements[0][j]
        )
        .unwrap();
    }
    s.push_str("*SURFACE, NAME=PLATE, TYPE=ELEMENT\n");
    for column in &plate_elements {
        writeln!(s, "{}, S1", column[0]).unwrap();
    }
    s.push_str("*NSET, NSET=FIX\n");
    for (grid, nx, ny) in [(&bottom, 16, 3), (&left, 2, 12), (&right, 2, 12)] {
        for j in 0..=ny {
            for i in 0..=nx {
                writeln!(s, "{},", grid.node(i, j)).unwrap();
            }
        }
    }
    s.push_str("*NSET, NSET=PRESS\n");
    for i in 0..=22 {
        writeln!(s, "{},", plate.node(i, 2)).unwrap();
    }
    writeln!(s, "*NSET, NSET=CENTRE\n{},", block.node(5, 5)).unwrap();
    s.push_str(
        "*MATERIAL, NAME=RUBBER\n*HYPERELASTIC, NEO HOOKE\n200, 0.0002\n\
*MATERIAL, NAME=STEEL\n*ELASTIC\n2e5, 0.3\n\
*SOLID SECTION, ELSET=BLOCK, MATERIAL=RUBBER\n*SOLID SECTION, ELSET=BOTTOM, MATERIAL=STEEL\n\
*SOLID SECTION, ELSET=LEFT, MATERIAL=STEEL\n*SOLID SECTION, ELSET=RIGHT, MATERIAL=STEEL\n\
*SOLID SECTION, ELSET=PLATE, MATERIAL=STEEL\n\
*SURFACE INTERACTION, NAME=ROUGH\n*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=LINEAR\n1e5, 1\n\
*FRICTION\n0.5, 1e3\n\
*CONTACT PAIR, INTERACTION=ROUGH, TYPE=SURFACE TO SURFACE\nGASKET, GROOVE\n\
*CONTACT PAIR, INTERACTION=ROUGH, TYPE=SURFACE TO SURFACE\nGASKET, PLATE\n\
*AMPLITUDE, NAME=SIDEWAYS\n0, 0, 1, 0, 2, 0.5\n*AMPLITUDE, NAME=DOWN\n0, 0, 1, -0.2, 2, 0.2\n\
*STEP, NLGEOM, INC=1000\n*STATIC\n0.05, 2, 1e-5, 0.05\n*BOUNDARY\nFIX, 1, 2, 0\n\
*BOUNDARY, AMPLITUDE=SIDEWAYS\nPRESS, 1, 1, 1\n*BOUNDARY, AMPLITUDE=DOWN\nPRESS, 2, 2, 1\n\
*NODE PRINT, NSET=CENTRE\nU\n*END STEP\n",
    );
    s
}

#[test]
fn a_gasket_the_plate_lets_go_of_stays_in_its_groove() {
    let (finished, log, rows, dat) = run("gasket", &gasket(", STABILIZE=10"));
    assert!(finished, "the step did not finish:\n{log}");
    assert_eq!(rows.last().unwrap()[5], 2.0, "{rows:?}");
    let cutbacks = rows.iter().filter(|r| r[2] > 1.0).count();
    assert!(cutbacks <= 2, "cut back {cutbacks} times: {rows:?}");
    // Let go of, it springs back: dragged along by the plate against the wall at most
    // (0.1 away), on the bottom of the groove.
    let [x, y] = last_displacement(&dat);
    assert!(
        x > 0.0 && x < 0.1 + 1e-3 && y.abs() < 0.01,
        "the gasket moved by ({x}, {y})"
    );
}

/// Without the stabilization the loose gasket stops the step once the plate lifted off: the
/// test above checks what the stabilization changes.
#[test]
fn a_gasket_on_weak_springs_alone_stops_once_the_plate_lifted_off() {
    let (finished, _, rows, _) = run("gasket-weak", &gasket(""));
    let reached = rows.last().map_or(0.0, |r| r[5]);
    assert!(
        !finished && reached > 1.4 && reached < 2.0,
        "the step ended at {reached} (finished: {finished}): {rows:?}"
    );
}
