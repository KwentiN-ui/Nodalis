//! The step cache: a run takes over the steps of the run before that did not change and
//! gives the same results as the run of all steps from the start; a changed step and the
//! steps after it are computed again.

use std::path::{Path, PathBuf};

use plxs_core::{ProgressEvent, Results};
use plxs_session::{JobOptions, StepCache, run_job_keeping};

/// A cube of `n³` C3D8 elements, with a plastic steel, fixed at x = 0; the face x = n is `PULL`.
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
    s += "*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n*PLASTIC\n200, 0\n400, 0.2\n*DENSITY\n7.85e-9\n*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL\n";
    s
}

const PULL: &str = "*STEP, NLGEOM\n*STATIC\n0.25, 1\n*BOUNDARY\nFIX, 1, 3\n*CLOAD\nPULL, 1, 150\n\
    *NODE FILE\nU\n*EL FILE\nS, PEEQ\n*NODE PRINT, NSET=PULL\nU\n*END STEP\n";
const MODES: &str = "*STEP\n*FREQUENCY, STORAGE=YES\n3\n*NODE FILE\nU\n*END STEP\n";
const FURTHER: &str = "*STEP, NLGEOM\n*STATIC\n0.5, 1\n*CLOAD\nPULL, 1, 165\n*NODE FILE\nU\n\
    *EL FILE\nS, PEEQ\n*NODE PRINT, NSET=PULL\nU\n*END STEP\n";

fn fresh(test: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("plxs-step-cache-{test}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Runs `steps` on the cube in `dir`, with the step cache `dir/job.steps` or without; the
/// results and the info lines of the run.
fn run(dir: &Path, steps: &[&str], cached: bool) -> (Results, Vec<String>) {
    let input = cube(3) + &steps.concat();
    let options = JobOptions {
        threads: 2,
        write_files: false,
        step_cache: cached.then(|| StepCache {
            dir: dir.join("job.steps"),
            salt: 1,
        }),
        ..JobOptions::new(dir, "job")
    };
    let mut info = Vec::new();
    let mut kept = None;
    run_job_keeping(
        &input,
        None,
        &options,
        &mut |event: ProgressEvent| {
            if let ProgressEvent::Info(text) = event {
                info.push(text);
            }
        },
        &mut kept,
    )
    .unwrap();
    (kept.unwrap().1, info)
}

fn taken_over(info: &[String]) -> Option<&String> {
    info.iter().find(|line| line.contains("since the last run"))
}

/// The case of the request: a step is added after a computed one.
#[test]
fn an_added_step_continues_after_the_steps_computed_before() {
    let dir = fresh("added");
    let (first, info) = run(&dir, &[PULL], true);
    assert!(taken_over(&info).is_none());
    let (cached, info) = run(&dir, &[PULL, FURTHER, MODES], true);
    assert!(
        taken_over(&info).is_some_and(|l| l.contains("steps 1 to 1")),
        "{info:?}"
    );
    let (full, _) = run(&fresh("added-full"), &[PULL, FURTHER, MODES], false);
    assert_eq!(cached, full);
    // The first step's results are those of the first run.
    let step = |r: &Results| r.increments.iter().filter(|i| i.step == 1).count();
    assert_eq!(step(&first), step(&full));
    assert!(first.increments.iter().all(|i| full.increments.contains(i)));
    // Nothing changed: no step is computed.
    let (again, info) = run(&dir, &[PULL, FURTHER, MODES], true);
    assert!(
        taken_over(&info).is_some_and(|l| l.contains("no step changed")),
        "{info:?}"
    );
    assert_eq!(again, full);
    let _ = std::fs::remove_dir_all(&dir);
}

/// A change in a step leaves the steps before it taken over; a change in the first step
/// (or the model) computes everything again.
#[test]
fn a_changed_step_is_computed_again_with_the_steps_after_it() {
    let dir = fresh("changed");
    run(&dir, &[PULL, FURTHER, MODES], true);
    let fewer_modes = MODES.replace("\n3\n", "\n2\n");
    let (cached, info) = run(&dir, &[PULL, FURTHER, &fewer_modes], true);
    assert!(
        taken_over(&info).is_some_and(|l| l.contains("steps 1 to 2")),
        "{info:?}"
    );
    let (full, _) = run(
        &fresh("changed-full"),
        &[PULL, FURTHER, &fewer_modes],
        false,
    );
    assert_eq!(cached, full);

    let slower = FURTHER.replace("0.5, 1", "0.4, 1");
    let (cached, info) = run(&dir, &[PULL, &slower, &fewer_modes], true);
    assert!(
        taken_over(&info).is_some_and(|l| l.contains("steps 1 to 1")),
        "{info:?}"
    );
    let (full, _) = run(
        &fresh("changed-full"),
        &[PULL, &slower, &fewer_modes],
        false,
    );
    assert_eq!(cached, full);

    let harder = PULL.replace("150\n", "140\n");
    let (cached, info) = run(&dir, &[&harder, FURTHER, &fewer_modes], true);
    assert!(taken_over(&info).is_none(), "{info:?}");
    let (full, _) = run(
        &fresh("changed-full"),
        &[&harder, FURTHER, &fewer_modes],
        false,
    );
    assert_eq!(cached, full);
    // Only the steps of the last run are kept.
    let kept = std::fs::read_dir(dir.join("job.steps")).unwrap().count();
    assert_eq!(kept, 2 * 3);
    let _ = std::fs::remove_dir_all(&dir);
}

/// Another solver build does not take over the steps.
#[test]
fn another_solver_build_computes_everything() {
    let dir = fresh("build");
    run(&dir, &[PULL], true);
    let input = cube(3) + PULL;
    let options = JobOptions {
        threads: 2,
        write_files: false,
        step_cache: Some(StepCache {
            dir: dir.join("job.steps"),
            salt: 2,
        }),
        ..JobOptions::new(&dir, "job")
    };
    let mut info = Vec::new();
    run_job_keeping(
        &input,
        None,
        &options,
        &mut |event: ProgressEvent| {
            if let ProgressEvent::Info(text) = event {
                info.push(text);
            }
        },
        &mut None,
    )
    .unwrap();
    assert!(taken_over(&info).is_none(), "{info:?}");
    let _ = std::fs::remove_dir_all(&dir);
}

/// Sets and amplitudes defined for a later step (the model holds them all) leave the steps
/// before it taken over.
#[test]
fn sets_and_amplitudes_of_a_later_step_keep_the_steps_before_it() {
    let dir = fresh("named");
    run(&dir, &[PULL], true);
    let named = "*NSET, NSET=CORNER\n64\n*AMPLITUDE, NAME=RAMP\n0, 0, 1, 1\n";
    let sideways = "*STEP, NLGEOM\n*STATIC\n0.5, 1\n*CLOAD, AMPLITUDE=RAMP\nCORNER, 2, 5\n\
        *NODE FILE\nU\n*END STEP\n";
    let (cached, info) = run(&dir, &[named, PULL, sideways], true);
    assert!(
        taken_over(&info).is_some_and(|l| l.contains("steps 1 to 1")),
        "{info:?}"
    );
    let (full, _) = run(&fresh("named-full"), &[named, PULL, sideways], false);
    assert_eq!(cached, full);
    // A changed set that the first step does not use keeps it too, one it uses does not.
    let (_, info) = run(
        &dir,
        &[&named.replace("\n64\n", "\n63\n"), PULL, sideways],
        true,
    );
    assert!(
        taken_over(&info).is_some_and(|l| l.contains("steps 1 to 1")),
        "{info:?}"
    );
    let _ = std::fs::remove_dir_all(&dir);
}
