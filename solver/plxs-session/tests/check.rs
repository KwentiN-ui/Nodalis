//! The model check finds free motions exactly and names the nodes that move.

use plxs_core::progress::NoProgress;
use plxs_core::{DiagnosticCode, Severity};
use plxs_session::{JobOptions, check_input};

/// A block of `nx × ny × nz` C3D8 elements of size `h`, starting at `x0` along x, with node
/// and element ids offset by `offset`; node sets `<NAME>X0` (face x = x0) and `<NAME>ALL`.
fn block(name: &str, nx: usize, ny: usize, nz: usize, h: f64, x0: f64, offset: u32) -> String {
    let id = |i: usize, j: usize, k: usize| offset + 1 + (i + (nx + 1) * (j + (ny + 1) * k)) as u32;
    let mut s = String::from("*NODE\n");
    for k in 0..=nz {
        for j in 0..=ny {
            for i in 0..=nx {
                s += &format!(
                    "{}, {}, {}, {}\n",
                    id(i, j, k),
                    x0 + i as f64 * h,
                    j as f64 * h,
                    k as f64 * h
                );
            }
        }
    }
    s += &format!("*ELEMENT, TYPE=C3D8, ELSET={name}\n");
    let mut e = offset;
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                e += 1;
                s += &format!(
                    "{e}, {}, {}, {}, {}, {}, {}, {}, {}\n",
                    id(i, j, k),
                    id(i + 1, j, k),
                    id(i + 1, j + 1, k),
                    id(i, j + 1, k),
                    id(i, j, k + 1),
                    id(i + 1, j, k + 1),
                    id(i + 1, j + 1, k + 1),
                    id(i, j + 1, k + 1)
                );
            }
        }
    }
    s += &format!("*NSET, NSET={name}X0\n");
    for k in 0..=nz {
        for j in 0..=ny {
            s += &format!("{},\n", id(0, j, k));
        }
    }
    s += &format!(
        "*MATERIAL, NAME=M{name}\n*ELASTIC\n210000, 0.3\n*SOLID SECTION, ELSET={name}, MATERIAL=M{name}\n"
    );
    s
}

fn options() -> JobOptions {
    JobOptions {
        threads: 1,
        write_files: false,
        ..JobOptions::new(std::env::temp_dir(), "check")
    }
}

fn step(boundary: &str) -> String {
    format!("{boundary}*STEP\n*STATIC\n*END STEP\n")
}

fn rank(check: &plxs_session::Check) -> Vec<&plxs_core::Diagnostic> {
    check
        .diagnostics
        .iter()
        .filter(|d| {
            matches!(
                d.code,
                DiagnosticCode::RigidBodyMotion | DiagnosticCode::Mechanism
            )
        })
        .collect()
}

#[test]
fn a_held_block_has_no_free_motion() {
    let input = block("A", 4, 2, 2, 1.0, 0.0, 0) + &step("*BOUNDARY\nAX0, 1, 3\n");
    let check = check_input(&input, None, &options(), &mut NoProgress);
    assert!(rank(&check).is_empty(), "{:#?}", check.diagnostics);
    assert!(!check.has_errors());
    assert!(
        check
            .diagnostics
            .iter()
            .any(|d| matches!(d.code, DiagnosticCode::FactorSize { .. }))
    );
}

#[test]
fn a_free_block_moves_in_six_ways() {
    let input = block("A", 3, 2, 2, 1.0, 0.0, 0) + &step("");
    let check = check_input(&input, None, &options(), &mut NoProgress);
    let found = rank(&check);
    assert_eq!(found.len(), 1, "{:#?}", check.diagnostics);
    assert_eq!(found[0].code, DiagnosticCode::RigidBodyMotion);
    assert_eq!(found[0].severity, Severity::Error);
    assert_eq!(found[0].motions.len(), 6);
    assert_eq!(found[0].nodes.len(), 4 * 3 * 3);
    assert_eq!(found[0].step, Some(1));
}

#[test]
fn only_the_unsupported_block_is_named() {
    // Two blocks side by side, not connected; only A is held.
    let input = block("A", 2, 2, 2, 1.0, 0.0, 0)
        + &block("B", 2, 2, 2, 1.0, 5.0, 1000)
        + &step("*BOUNDARY\nAX0, 1, 3\n");
    let check = check_input(&input, None, &options(), &mut NoProgress);
    let found = rank(&check);
    assert_eq!(found.len(), 1, "{:#?}", check.diagnostics);
    assert_eq!(found[0].motions.len(), 6);
    assert!(
        found[0].nodes.iter().all(|&n| n > 1000),
        "{:?}",
        found[0].nodes
    );
    assert_eq!(found[0].nodes.len(), 27);
}

#[test]
fn a_block_held_along_a_line_turns_about_it() {
    // Nodes on the edge x = 0, y = 0 held in all directions: the block turns about it.
    let mut input = block("A", 2, 2, 2, 1.0, 0.0, 0);
    input += "*NSET, NSET=EDGE\n1, 10, 19\n";
    input += &step("*BOUNDARY\nEDGE, 1, 3\n");
    let check = check_input(&input, None, &options(), &mut NoProgress);
    let found = rank(&check);
    assert_eq!(found.len(), 1, "{:#?}", check.diagnostics);
    assert_eq!(found[0].code, DiagnosticCode::RigidBodyMotion);
    assert_eq!(found[0].motions.len(), 1);
    // The motion is a rotation about z (the edge runs along z).
    let m = found[0].motions[0];
    let rotation = (m[3] * m[3] + m[4] * m[4] + m[5] * m[5]).sqrt();
    assert!(m[5].abs() > 0.99 * rotation, "{m:?}");
}

#[test]
fn a_slender_held_bar_is_not_singular() {
    // 200:1, clamped at one end: soft in bending, but held.
    let input = block("A", 200, 1, 1, 1.0, 0.0, 0) + &step("*BOUNDARY\nAX0, 1, 3\n");
    let check = check_input(&input, None, &options(), &mut NoProgress);
    assert!(rank(&check).is_empty(), "{:#?}", check.diagnostics);
}

#[test]
fn free_modes_are_only_information_in_a_frequency_step() {
    let mut input =
        block("A", 2, 2, 2, 1.0, 0.0, 0).replace("*ELASTIC", "*DENSITY\n7.8e-9\n*ELASTIC");
    input += "*STEP\n*FREQUENCY\n10\n*END STEP\n";
    let check = check_input(&input, None, &options(), &mut NoProgress);
    let found = rank(&check);
    assert_eq!(found.len(), 1, "{:#?}", check.diagnostics);
    assert_eq!(found[0].severity, Severity::Info);
    assert!(!check.has_errors());
}

#[test]
fn input_errors_are_diagnostics() {
    let check = check_input(
        "*NODE\n1, 0, 0, 0\n*CLOUD\n",
        None,
        &options(),
        &mut NoProgress,
    );
    assert!(check.has_errors());
    assert_eq!(check.diagnostics[0].code, DiagnosticCode::Input);
}

#[test]
fn free_shells_are_named_by_their_own_nodes() {
    // A plate of 2 × 2 S4 shells, nothing held: the expanded solid moves, the shell nodes
    // are named.
    let mut input = String::from("*NODE\n");
    for j in 0..3 {
        for i in 0..3 {
            input += &format!("{}, {}, {}, 0\n", 1 + i + 3 * j, i, j);
        }
    }
    input += "*ELEMENT, TYPE=S4, ELSET=PLATE\n1, 1, 2, 5, 4\n2, 2, 3, 6, 5\n3, 4, 5, 8, 7\n4, 5, 6, 9, 8\n\
        *MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n\
        *SHELL SECTION, ELSET=PLATE, MATERIAL=STEEL\n0.1\n*STEP\n*STATIC\n*END STEP\n";
    let check = check_input(&input, None, &options(), &mut NoProgress);
    let found = rank(&check);
    assert_eq!(found.len(), 1, "{:#?}", check.diagnostics);
    assert_eq!(found[0].nodes, (1..=9).collect::<Vec<u32>>());
    assert_eq!(found[0].motions.len(), 6);
}

#[test]
fn untied_slave_nodes_are_named() {
    // Block B 3 units away from A: its x0 face cannot be tied to A's far face.
    let mut input = block("A", 2, 2, 2, 1.0, 0.0, 0) + &block("B", 2, 2, 2, 1.0, 5.0, 1000);
    input += "*SURFACE, NAME=SB, TYPE=NODE\nBX0,\n*ELSET, ELSET=AFAR\n2, 4, 6, 8\n\
        *SURFACE, NAME=SA\nAFAR, S4\n*TIE, NAME=GLUE, POSITION TOLERANCE=0.1\nSB, SA\n";
    input += &step("*BOUNDARY\nAX0, 1, 3\nBX0, 1, 3\n");
    let check = check_input(&input, None, &options(), &mut NoProgress);
    let untied: Vec<_> = (check.diagnostics.iter())
        .filter(|d| matches!(&d.code, DiagnosticCode::TieNodesNotTied { tie } if tie == "GLUE"))
        .collect();
    assert_eq!(untied.len(), 1, "{:#?}", check.diagnostics);
    assert_eq!(untied[0].nodes.len(), 9);
    assert!(untied[0].nodes.iter().all(|&n| n > 1000));
}
