use super::*;
use crate::{Job, JobStatus, Solver};

const MODEL: &str = "\
*Heading
Restart test
*Node
1, 0, 0, 0
2, 1, 0, 0
3, 1, 1, 0
4, 0, 1, 0
5, 0, 0, 1
6, 1, 0, 1
7, 1, 1, 1
8, 0, 1, 1
*Element, Type=C3D8, Elset=EALL
1, 1, 2, 3, 4, 5, 6, 7, 8
*Nset, Nset=FIX
1, 2, 3, 4
*Nset, Nset=TOP
5, 6, 7, 8
*Material, Name=STEEL
*Elastic
210000, 0.3
*Density
7.85e-9
*Solid section, Elset=EALL, Material=STEEL
*Boundary
FIX, 1, 3
";

/// A nonlinear static step pulling the top with `force`.
fn static_step(name: &str, force: f64) -> String {
    format!(
        "**\n** Step: {name}\n**\n*Step, Nlgeom\n*Static\n0.5, 1\n*Cload\nTOP, 3, {force}\n\
         *Node file\nU\n*El file\nS\n*Node print, Nset=TOP, Totals=Only\nRF\n*End step\n"
    )
}

const FREQUENCY: &str = "*Step, Perturbation\n*Frequency\n3\n*Node file\nU\n*End step\n";

fn work_dir(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("plx-restart-{name}-{}", std::process::id()))
}

fn ccx_installed() -> bool {
    let found = std::process::Command::new("ccx").arg("-v").output().is_ok();
    if !found {
        eprintln!("ccx not found, test skipped");
    }
    found
}

#[test]
fn steps_over_time_write_their_state() {
    let input = format!(
        "{MODEL}{}{}{FREQUENCY}{}",
        static_step("Step-1", 100.0),
        static_step("Step-2", 200.0),
        static_step("Step-4", 300.0)
    );
    let chain = Chain::new(&input, true, 0);
    assert_eq!(chain.steps.len(), 4);
    // The frequency step ends the steps whose state is kept.
    assert_eq!(chain.restartable, 2);
    assert_eq!(chain.input.matches("*Restart, Write").count(), 2);
    assert_eq!(
        Chain::new(&input, false, 0).input,
        input,
        "without states the input stays as it is"
    );
    // Comments, such as step names, do not count; the steps after a change differ too.
    let renamed = input.replace("Step: Step-2", "Step: Loading");
    assert_eq!(Chain::new(&renamed, true, 0).steps, chain.steps);
    let changed = input.replace("TOP, 3, 200", "TOP, 3, 250");
    let other = Chain::new(&changed, true, 0);
    assert_eq!(other.steps[0], chain.steps[0]);
    assert_ne!(other.steps[1], chain.steps[1]);
    assert_ne!(other.steps[3], chain.steps[3]);
    // The heading is no part of the model; another file read by the job is.
    let headed = input.replace("Restart test", "Other heading");
    assert_eq!(Chain::new(&headed, true, 0).steps, chain.steps);
    assert_ne!(Chain::new(&input, true, 1).steps[0], chain.steps[0]);
    // The input of a restart reads the state and holds the steps after.
    let restart = chain.restart_input(2);
    assert!(restart.contains("*Restart, Read, Step=2"));
    assert!(!restart.contains("*Node\n"));
    assert_eq!(restart.matches("*Step").count(), 2);
    // A file with its own restart keywords is left alone.
    let own = input.replacen("*End step", "*Restart, Write\n*End step", 1);
    assert_eq!(Chain::new(&own, true, 0).restartable, 0);
}

#[test]
fn results_of_a_restart_join_those_before() {
    // Data sets numbered through the file, as CalculiX writes them.
    let frd = |steps: &[u32]| {
        let mut text = "    1C\n    2C  nodes\n -3\n".to_string();
        for (i, step) in steps.iter().enumerate() {
            let n = i + 1;
            text += &format!("    1PSTEP{n:>26}{:>12}{step:>12}\n", 1);
            text += &format!(
                "  100CL{:>5}{:>12}{:>12}{:>20}{:>2}{n:>5}\n -3\n",
                100 + n,
                1.0,
                8,
                "",
                0
            );
        }
        text + " 9999\n"
    };
    let joined = join_frd(frd(&[1, 1, 2, 3]).as_bytes(), frd(&[3, 4]).as_bytes(), 2);
    assert_eq!(String::from_utf8(joined).unwrap(), frd(&[1, 1, 2, 3, 4]));

    let dat = |steps: &[(u32, f64)]| {
        let mut text = String::new();
        for (step, time) in steps {
            text += &format!("\n   S T E P       {step}\n\n INCREMENT 1\n\n");
            text += &format!(" displacements (vx,vy,vz) for set A and time  {time:E}\n\n 1 0.0\n");
        }
        text
    };
    let old = dat(&[(1, 1.0), (2, 2.0), (3, 3.0)]);
    let new = dat(&[(3, 2.5)]);
    let joined = join_dat(old.as_bytes(), new.as_bytes(), 2, 2.0);
    assert_eq!(
        String::from_utf8(joined).unwrap(),
        dat(&[(1, 1.0), (2, 2.0), (3, 2.5)])
    );
    // Without step marks the time tells the steps apart.
    let unmarked = |text: String| {
        (text.lines())
            .filter(|l| !l.contains("S T E P"))
            .map(|l| format!("{l}\n"))
            .collect::<String>()
    };
    let joined = join_dat(
        unmarked(old).as_bytes(),
        unmarked(new.clone()).as_bytes(),
        2,
        2.0,
    );
    assert_eq!(
        String::from_utf8(joined).unwrap(),
        unmarked(dat(&[(1, 1.0), (2, 2.0)])) + &unmarked(new)
    );
}

#[test]
fn the_status_file_tells_the_finished_steps() {
    let sta = "\
SUMMARY OF JOB INFORMATION
  STEP      INC     ATT  ITRS     TOT TIME     STEP TIME      INC TIME
     2          1     1     2  0.150000E+01  0.500000E+00  0.500000E+00
     2          2     1     2  0.200000E+01  0.100000E+01  0.500000E+00
     3          1     1     2  0.250000E+01  0.500000E+00  0.500000E+00
";
    assert_eq!(finished_steps(sta, 1, true), [(2, 2.0), (3, 2.5)]);
    // The step a failed run stopped in did not finish.
    assert_eq!(finished_steps(sta, 1, false), [(2, 2.0)]);
}

/// Runs the job as Nodalis does: from a state if the index has one, recording the run.
fn run(dir: &Path, input: &str) -> (Plan, String) {
    run_after(dir, input, &[])
}

/// Runs the job, from a state of its own or of the jobs in `upstream`.
fn run_after(dir: &Path, input: &str, upstream: &[(PathBuf, String)]) -> (Plan, String) {
    let solver = Solver::default();
    let chain = Chain::new(input, true, 0);
    let mut jobs = vec![(dir.to_path_buf(), "Job".to_string())];
    jobs.extend(upstream.iter().cloned());
    let plan = plan(&chain, "ccx", &jobs);
    let mut index = StateIndex::read(dir, "Job");
    let text = match &plan {
        Plan::Restart {
            after,
            file,
            results,
            ..
        } => {
            prepare(dir, "Job", file, results).unwrap();
            chain.restart_input(*after)
        }
        Plan::Full(_) => chain.input.clone(),
    };
    let mut job = Job::start(&solver, dir, "Job", &text).unwrap();
    let status = job.wait();
    assert_eq!(status, JobStatus::Completed, "{:?}", job.new_output());
    let after = match &plan {
        Plan::Restart { after, time, .. } => {
            join_results(dir, "Job", *after, *time).unwrap();
            *after
        }
        Plan::Full(_) => 0,
    };
    let record = Run {
        after,
        completed: true,
        results: true,
        sta: std::fs::read_to_string(dir.join("Job.sta")).unwrap(),
    };
    index.record(&chain, "ccx", dir, "Job", &record).unwrap();
    let frd = std::fs::read_to_string(dir.join("Job.frd")).unwrap();
    (plan, frd)
}

/// The result lines of a results file, without the date and time of the run.
fn results(frd: &str) -> Vec<&str> {
    frd.lines().filter(|l| !l.starts_with("    1U")).collect()
}

#[test]
fn a_changed_step_is_solved_from_the_state_before() {
    if !ccx_installed() {
        return;
    }
    let dir = work_dir("chain");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let steps = |last: f64| {
        format!(
            "{MODEL}{}{}{}{FREQUENCY}",
            static_step("Step-1", 100.0),
            static_step("Step-2", 200.0),
            static_step("Step-3", last)
        )
    };
    let (plan, _) = run(&dir, &steps(300.0));
    assert!(matches!(plan, Plan::Full(_)), "{plan:?}");
    // Only the third step changed: the run starts after the second.
    let (plan, restarted) = run(&dir, &steps(400.0));
    assert!(matches!(plan, Plan::Restart { after: 2, .. }), "{plan:?}");
    let dat = std::fs::read_to_string(dir.join("Job.dat")).unwrap();
    assert_eq!(dat.matches("S T E P").count(), 4, "{dat}");
    // The same results as a run of all steps.
    let full = work_dir("full");
    let _ = std::fs::remove_dir_all(&full);
    let (_, solved) = run(&full, &steps(400.0));
    assert_eq!(results(&restarted), results(&solved));
    // Another job that continues after the second step starts from this job's state.
    let other = work_dir("continued");
    let _ = std::fs::remove_dir_all(&other);
    let (plan, continued) = run_after(&other, &steps(450.0), &[(dir.clone(), "Job".into())]);
    assert!(matches!(plan, Plan::Restart { after: 2, .. }), "{plan:?}");
    let full_other = work_dir("full-other");
    let _ = std::fs::remove_dir_all(&full_other);
    let (_, solved_other) = run(&full_other, &steps(450.0));
    assert_eq!(results(&continued), results(&solved_other));
    std::fs::remove_dir_all(other).unwrap();
    std::fs::remove_dir_all(full_other).unwrap();
    // The input file of the run is the full one, and the states of the steps that changed
    // are gone.
    let index = StateIndex::read(&dir, "Job");
    let mut steps_kept: Vec<usize> = (index.files.iter())
        .flat_map(|f| f.states.iter().map(|s| s.step))
        .collect();
    steps_kept.sort();
    assert_eq!(steps_kept, [1, 2, 3]);
    assert!(!dir.join("Job.rin").exists());
    // A new set for the steps is model definition, which a restart cannot add.
    let with_set = steps(500.0).replace("*Material", "*Nset, Nset=CORNER\n5\n*Material");
    let (plan, _) = run(&dir, &with_set);
    assert!(matches!(plan, Plan::Full(_)), "{plan:?}");
    // Another CalculiX does not read the states.
    let chain = Chain::new(&with_set, true, 0);
    let index = StateIndex::read(&dir, "Job");
    assert!(matches!(
        index.plan(&chain, "other", &dir, "Job"),
        Plan::Full(_)
    ));
    std::fs::remove_dir_all(dir).unwrap();
    std::fs::remove_dir_all(full).unwrap();
}

#[test]
fn the_oldest_states_go_first() {
    let project = work_dir("limit");
    let job = project.join("Analysis-1");
    std::fs::create_dir_all(&job).unwrap();
    for (i, name) in ["Job.state-0.rout", "Job.state-1.rout", "Job.frd"]
        .iter()
        .enumerate()
    {
        std::fs::write(job.join(name), vec![0u8; 100]).unwrap();
        // Distinct modification times.
        let time = std::time::UNIX_EPOCH + std::time::Duration::from_secs(1000 + i as u64);
        std::fs::File::options()
            .write(true)
            .open(job.join(name))
            .unwrap()
            .set_modified(time)
            .unwrap();
    }
    let notes = limit_states(&project, 150);
    assert_eq!(notes.len(), 1);
    assert!(!job.join("Job.state-0.rout").exists());
    assert!(job.join("Job.state-1.rout").exists());
    assert!(job.join("Job.frd").exists());
    std::fs::remove_dir_all(project).unwrap();
}
