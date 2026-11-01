//! `*RESTART, WRITE` and `*RESTART, READ`: a run that continues from the state after step
//! `k` of an earlier run gives the same results for the steps after `k` as the run of all
//! steps from the start. Covered: creep (material state), moving loads and amplitudes over
//! total time (carry-over of loads, total time), locked and unlocked constraints; a
//! snapshot of another model is refused, new sets are allowed.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;

use plxs_verify::fixtures_root;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

fn run(dir: &Path, job: &str) -> String {
    let output = Command::new(plxs())
        .arg(job)
        .arg("--threads")
        .arg("2")
        .current_dir(dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned();
    assert!(output.status.success(), "{job} failed:\n{log}");
    log
}

/// Line indices of the `*STEP` cards.
fn step_starts(lines: &[&str]) -> Vec<usize> {
    (lines.iter().enumerate())
        .filter(|(_, l)| {
            let l = l.trim_start().to_ascii_uppercase();
            l.starts_with("*STEP") && !l.starts_with("**")
        })
        .map(|(i, _)| i)
        .collect()
}

type Blocks = BTreeMap<(u32, u32, String), Vec<f64>>;

/// The result blocks of an `.frd` by (step, increment, field), values in file order.
fn blocks(path: &Path) -> Blocks {
    let text = std::fs::read_to_string(path).unwrap();
    let mut out = Blocks::new();
    let mut step: Option<(u32, u32)> = None;
    let mut key = None;
    for line in text.lines() {
        if let Some(rest) = line.strip_prefix("    1PSTEP") {
            let f: Vec<u32> = rest
                .split_whitespace()
                .filter_map(|t| t.parse().ok())
                .collect();
            step = Some((f[2], f[1]));
            key = None;
        } else if let (Some((s, i)), true) = (step, line.starts_with(" -4")) {
            let name = line.split_whitespace().nth(1).unwrap().to_string();
            key = Some((s, i, name));
            out.entry(key.clone().unwrap()).or_default();
        } else if let (Some(k), true) = (&key, line.starts_with(" -1") || line.starts_with(" -2")) {
            let values = out.get_mut(k).unwrap();
            let mut at = 13;
            while at + 12 <= line.len() {
                values.push(line[at..at + 12].trim().parse().unwrap());
                at += 12;
            }
        }
    }
    out
}

/// Runs the fixture in full with `*RESTART, WRITE` in every step, then continues from
/// each step and compares.
fn check(area: &str, name: &str) {
    let source = fixtures_root().join(area).join(format!("{name}.inp"));
    let text = std::fs::read_to_string(&source).unwrap();
    let dir = std::env::temp_dir().join(format!("plxs-restart-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    // Files the input refers to (temperature fields) sit next to it.
    for entry in std::fs::read_dir(source.parent().unwrap()).unwrap() {
        let path = entry.unwrap().path();
        if path.extension().is_some_and(|e| e == "frd") {
            std::fs::copy(&path, dir.join(path.file_name().unwrap())).unwrap();
        }
    }
    let lines: Vec<&str> = text.lines().collect();
    let starts = step_starts(&lines);
    let mut full = String::new();
    for (i, line) in lines.iter().enumerate() {
        full.push_str(line);
        full.push('\n');
        if starts.contains(&i) {
            full.push_str("*Restart, Write\n");
        }
    }
    std::fs::write(dir.join("full.inp"), &full).unwrap();
    run(&dir, "full");
    let reference = blocks(&dir.join("full.frd"));
    for k in 1..starts.len() {
        let job = format!("from{k}");
        let continuing = format!(
            "** continues after step {k}\n*Restart, Read, Step={k}\n{}\n",
            lines[starts[k]..].join("\n")
        );
        std::fs::write(dir.join(format!("{job}.inp")), continuing).unwrap();
        std::fs::copy(dir.join("full.rout"), dir.join(format!("{job}.rin"))).unwrap();
        let log = run(&dir, &job);
        assert!(log.contains(&format!(" STEP {}", k + 1)), "{log}");
        let restarted = blocks(&dir.join(format!("{job}.frd")));
        let expected: Vec<_> = reference.keys().filter(|key| key.0 > k as u32).collect();
        assert_eq!(
            restarted.keys().collect::<Vec<_>>(),
            expected,
            "{name}: result blocks after a restart from step {k}"
        );
        for (key, values) in &restarted {
            let full = &reference[key];
            assert_eq!(values.len(), full.len(), "{name} {key:?}");
            let scale = full.iter().fold(0.0f64, |m, v| m.max(v.abs())).max(1e-30);
            for (a, b) in values.iter().zip(full) {
                assert!(
                    (a - b).abs() <= 1e-6 * scale,
                    "{name} from step {k}, {key:?}: {a} vs {b}"
                );
            }
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn creep_continues_with_its_material_state() {
    check("nonlinear", "kriechen");
}

#[test]
fn moving_loads_continue_with_the_carried_loads() {
    check("loads", "moving");
}

#[test]
fn amplitudes_over_total_time_continue_at_the_time_reached() {
    check("loads", "amplitude_total_time");
}

#[test]
fn locked_constraints_continue() {
    check("constraints", "gelockert");
    check("constraints", "verriegelt");
}

#[test]
fn a_restart_file_of_another_model_is_refused() {
    let dir = std::env::temp_dir().join(format!("plxs-restart-other-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let model = |e: &str| {
        format!(
            "*Node, Nset=ALL\n1, 0, 0, 0\n2, 1, 0, 0\n3, 0, 1, 0\n4, 0, 0, 1\n\
             *Element, Type=C3D4, Elset=E\n1, 1, 2, 3, 4\n*Material, Name=M\n*Elastic\n{e}, 0.3\n\
             *Solid section, Elset=E, Material=M\n*Nset, Nset=BASE\n1, 2, 3\n"
        )
    };
    let step =
        "*Step\n*Static\n*Boundary\nBASE, 1, 3\n*Cload\n4, 3, 1.\n*Node file\nU\n*End step\n";
    let with_write = step.replace("*Static\n", "*Static\n*Restart, Write\n");
    std::fs::write(dir.join("a.inp"), format!("{}{with_write}", model("1000."))).unwrap();
    run(&dir, "a");
    // The restart file holds the model: a continuing input without one runs.
    std::fs::write(dir.join("b.inp"), format!("*Restart, Read\n{step}")).unwrap();
    std::fs::copy(dir.join("a.rout"), dir.join("b.rin")).unwrap();
    let log = run(&dir, "b");
    assert!(log.contains(" STEP 2"), "{log}");
    // A snapshot handed to a changed model through the session API is refused.
    let file = plxs_session::restart::RestartFile::read(&dir.join("a.rout")).unwrap();
    let snapshot = file.after(None).unwrap().clone();
    let mut other = plxs_input::read_inp_str(&format!("{}{step}", model("2000.")), None)
        .unwrap()
        .analysis;
    let error =
        plxs_session::Session::resume(other.model.clone(), snapshot.clone(), Default::default())
            .err()
            .expect("another material must be refused");
    assert!(error.to_string().contains("another model"), "{error}");
    // New sets are allowed.
    other = plxs_input::read_inp_str(
        &format!("{}*Nset, Nset=EXTRA\n4\n{step}", model("1000.")),
        None,
    )
    .unwrap()
    .analysis;
    assert!(
        plxs_session::Session::resume(other.model, snapshot.clone(), Default::default()).is_ok()
    );
    // And the bytes of a snapshot come back unchanged.
    let bytes = plxs_session::restart::encode_snapshot(&snapshot).unwrap();
    assert_eq!(
        plxs_session::restart::decode_snapshot(&bytes).unwrap(),
        snapshot
    );
    assert!(plxs_session::restart::decode_snapshot(b"PLXSSNAP\x09\0\0\0").is_err());
    let _ = std::fs::remove_dir_all(&dir);
}
