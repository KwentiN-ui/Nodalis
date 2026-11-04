//! A stop of the run ends a long computation at once, not after it: the factorization of a
//! large stiffness matrix looks at the cancel token before every supernode.

use std::time::{Duration, Instant};

use plxs_session::{JobOptions, JobOutcome, run_job};

/// A cube of `n³` C3D8 elements, fixed at x = 0 and pulled at x = n.
fn cube(n: usize) -> String {
    let id = |i: usize, j: usize, k: usize| 1 + i + (n + 1) * (j + (n + 1) * k);
    let mut s = String::from("*NODE, NSET=NALL\n");
    for k in 0..=n {
        for j in 0..=n {
            for i in 0..=n {
                s += &format!("{}, {i}, {j}, {k}\n", id(i, j, k));
            }
        }
    }
    s += "*ELEMENT, TYPE=C3D8, ELSET=EALL\n";
    let mut e = 0;
    for k in 0..n {
        for j in 0..n {
            for i in 0..n {
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
    let face = |i: usize| {
        let mut ids = String::new();
        for k in 0..=n {
            for j in 0..=n {
                ids += &format!("{},\n", id(i, j, k));
            }
        }
        ids
    };
    s += &format!("*NSET, NSET=FIX\n{}*NSET, NSET=PULL\n{}", face(0), face(n));
    s += "*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL\n";
    s += "*BOUNDARY\nFIX, 1, 3\n*STEP\n*STATIC\n*CLOAD\nPULL, 1, 1\n*END STEP\n";
    s
}

#[test]
fn a_stop_ends_a_long_factorization_at_once() {
    // About 28k equations, whose factorization takes seconds unoptimized; the stop comes
    // as soon as the factorization starts (after the ordering is reported).
    let input = cube(20);
    let options = JobOptions {
        threads: 2,
        write_files: false,
        ..JobOptions::new(std::env::temp_dir(), "plxs-cancel-test")
    };
    let cancel = options.cancel.clone();
    let mut stopped = None;
    let mut progress = |event: plxs_core::ProgressEvent| {
        if let plxs_core::ProgressEvent::Info(text) = &event
            && text.starts_with("factor:")
            && stopped.is_none()
        {
            cancel.cancel();
            stopped = Some(Instant::now());
        }
    };
    let outcome = run_job(&input, None, &options, &mut progress).unwrap();
    let ended = Instant::now();
    assert_eq!(outcome, JobOutcome::Cancelled);
    let stopped = stopped.expect("the factorization started");
    let delay = ended - stopped;
    assert!(
        delay < Duration::from_millis(500),
        "the run ended {delay:?} after the stop"
    );
}
