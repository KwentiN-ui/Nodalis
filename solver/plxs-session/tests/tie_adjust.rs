//! A `*TIE` whose ADJUST pulls slave nodes across their own elements: the error about the
//! elements turned inside out names the tie, so that the user looks at the tie's surfaces
//! instead of the mesh.
//!
//! A thin slave plate (z 0.9..1, one C3D8) lies above a master block (z 0..0.85): with a
//! position tolerance of 0.2 the plate's top nodes (0.15 above the block) are moved on to
//! the block's top face, below the plate's bottom nodes.

use plxs_core::progress::NoProgress;
use plxs_session::{JobOptions, run_job_keeping};

fn input(tolerance: f64) -> String {
    format!(
        "*Node
1, 0, 0, 0
2, 1, 0, 0
3, 1, 1, 0
4, 0, 1, 0
5, 0, 0, 0.85
6, 1, 0, 0.85
7, 1, 1, 0.85
8, 0, 1, 0.85
11, 0, 0, 0.9
12, 1, 0, 0.9
13, 1, 1, 0.9
14, 0, 1, 0.9
15, 0, 0, 1
16, 1, 0, 1
17, 1, 1, 1
18, 0, 1, 1
*Element, Type=C3D8, Elset=EALL
1, 1, 2, 3, 4, 5, 6, 7, 8
2, 11, 12, 13, 14, 15, 16, 17, 18
*Nset, Nset=FIX
1, 2, 3, 4
*Elset, Elset=TOP
1
*Elset, Elset=PLATE
2
*Surface, Name=MASTER, Type=Element
TOP, S2
*Surface, Name=SLAVE, Type=Element
PLATE, S1
PLATE, S2
*Material, Name=STEEL
*Elastic
210000, 0.3
*Solid section, Elset=EALL, Material=STEEL
*Tie, Name=GLUE, Position tolerance={tolerance}
SLAVE, MASTER
*Step
*Static
*Boundary
FIX, 1, 3
*Cload
15, 3, 1
*End step
"
    )
}

fn run(input: &str) -> Result<(), String> {
    let options = JobOptions {
        threads: 1,
        write_files: false,
        ..JobOptions::new(std::env::temp_dir(), "tie-adjust")
    };
    let mut kept = None;
    run_job_keeping(input, None, &options, &mut NoProgress, &mut kept)
        .map(|_| ())
        .map_err(|e| e.to_string())
}

#[test]
fn a_tie_that_turns_its_slave_elements_inside_out_is_named() {
    let error = run(&input(0.2)).expect_err("the plate is turned inside out");
    assert!(error.contains("turned inside out"), "{error}");
    assert!(error.contains("*TIE GLUE moved"), "{error}");
}

#[test]
fn a_tie_that_only_closes_the_gap_below_solves() {
    // Only the plate's bottom nodes, 0.05 above the block, are moved.
    run(&input(0.1)).unwrap();
}
