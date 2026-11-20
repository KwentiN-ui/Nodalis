//! What an analysis takes over from another one, given as values instead of a file: the
//! modes of a frequency step, the displacements of a global model for a submodel and
//! temperatures. The run gives the same results as with the file an input file names.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use plxs_core::handover::{GlobalResults, NodalModes, NodalTemperatures, Source};
use plxs_core::step::DefinedFieldKind;
use plxs_core::{Analysis, ProgressEvent, Results};
use plxs_session::{JobOptions, run_analysis_keeping, run_job_keeping};

/// A bar of `n` C3D8 elements along x, 1 x 1 in section, of a steel that expands with heat;
/// fixed at x = 0 (`FIX`), the end x = n is `END`.
fn bar(n: usize) -> String {
    let id = |i: usize, j: usize, k: usize| 1 + i + (n + 1) * (j + 2 * k);
    let mut s = String::from("*NODE, NSET=NALL\n");
    for k in 0..2 {
        for j in 0..2 {
            for i in 0..=n {
                s += &format!("{}, {i}, {j}, {k}\n", id(i, j, k));
            }
        }
    }
    s += "*ELEMENT, TYPE=C3D8, ELSET=EALL\n";
    for i in 0..n {
        s += &format!(
            "{}, {}, {}, {}, {}, {}, {}, {}, {}\n",
            i + 1,
            id(i, 0, 0),
            id(i + 1, 0, 0),
            id(i + 1, 1, 0),
            id(i, 1, 0),
            id(i, 0, 1),
            id(i + 1, 0, 1),
            id(i + 1, 1, 1),
            id(i, 1, 1)
        );
    }
    let face = |i: usize| -> String {
        let ids: Vec<String> = [(0, 0), (1, 0), (0, 1), (1, 1)]
            .iter()
            .map(|&(j, k)| id(i, j, k).to_string())
            .collect();
        ids.join(", ") + "\n"
    };
    s += &format!("*NSET, NSET=FIX\n{}*NSET, NSET=END\n{}", face(0), face(n));
    s += "*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n*DENSITY\n7.85e-9\n\
        *EXPANSION\n1.2e-5\n*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL\n";
    s
}

const FIXED: &str = "*BOUNDARY\nFIX, 1, 3\n";
const MODES: &str = "*STEP\n*FREQUENCY, STORAGE=YES\n4\n*NODE FILE\nU\n*END STEP\n";
const PULL: &str = "*STEP\n*STATIC\n*CLOAD\nEND, 1, 100\n*NODE FILE\nU\n*END STEP\n";
const MODAL: &str = "*STEP\n*MODAL DYNAMICS\n1e-4, 1e-3\n*CLOAD\nEND, 2, 10\n*NODE FILE\nU\n\
    *END STEP\n";

fn fresh(test: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("plxs-handover-{test}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn options(dir: &Path, write_files: bool) -> JobOptions {
    JobOptions {
        threads: 1,
        write_files,
        ..JobOptions::new(dir, "job")
    }
}

/// Runs an input text in `dir`, writing the result files (`job.frd`, `job.eig`).
fn run_text(dir: &Path, input: &str) {
    let mut kept = None;
    run_job_keeping(input, None, &options(dir, true), &mut |_| {}, &mut kept).unwrap();
}

/// Runs `analysis` in `dir`: its results and warnings.
fn run(dir: &Path, analysis: Analysis) -> (Results, Vec<String>) {
    let mut warnings = Vec::new();
    let mut kept = None;
    run_analysis_keeping(
        analysis,
        &options(dir, false),
        &mut |event: ProgressEvent| {
            if let ProgressEvent::Warning(text) = event {
                warnings.push(text);
            }
        },
        &mut kept,
    )
    .unwrap();
    (kept.unwrap().results, warnings)
}

fn analysis(input: &str) -> Analysis {
    plxs_input::read_inp_str(input, None).unwrap().analysis
}

/// The analysis of `input` read as `dir/job.inp`, which takes the modes of `dir/job.eig`.
fn analysis_in(dir: &Path, input: &str) -> Analysis {
    let path = dir.join("job.inp");
    plxs_input::read_inp_str(input, Some(&path))
        .unwrap()
        .analysis
}

/// The modes of `job.eig` by node id, as the program that made the analysis gives them.
fn modes_by_id(eig: &Path, analysis: &Analysis) -> NodalModes {
    let stored: plxs_core::state::StoredModes =
        postcard::from_bytes(&std::fs::read(eig).unwrap()).unwrap();
    let ids = &analysis.model.nodes.ids;
    NodalModes {
        eigenvalues: stored.eigenvalues.clone(),
        modes: (stored.modes.iter())
            .map(|field| {
                (ids.iter().enumerate())
                    .map(|(i, &id)| (id, [0, 1, 2].map(|d| field.node(i)[d])))
                    .collect()
            })
            .collect(),
        nodal_diameters: stored.nodal_diameters.clone(),
    }
}

#[test]
fn given_modes_run_a_modal_step_as_those_of_the_eig_file() {
    let dir = fresh("modes");
    run_text(&dir, &(bar(4) + FIXED + MODES));
    let eig = dir.join("job.eig");
    let from_file = analysis_in(&dir, &(bar(4) + FIXED + MODAL));
    assert_eq!(
        from_file.model.stored_modes,
        Some(Source::Path(eig.clone()))
    );
    let given = modes_by_id(&eig, &from_file);
    let mut from_values = from_file.clone();
    from_values.model.stored_modes = Some(Source::loaded(given.clone()));
    let (expected, _) = run(&dir, from_file);
    let (results, warnings) = run(&dir, from_values);
    assert!(warnings.is_empty(), "{warnings:?}");
    assert!(!results.increments.is_empty());
    assert_eq!(results, expected);
    // Modes without a node of the model are not used: the modal step has none.
    let mut partial = given;
    for shape in &mut partial.modes {
        shape.remove(&1);
    }
    let mut missing = analysis_in(&dir, &(bar(4) + FIXED + MODAL));
    missing.model.stored_modes = Some(Source::loaded(partial));
    let mut warnings = Vec::new();
    let failed = run_analysis_keeping(
        missing,
        &options(&dir, false),
        &mut |event: ProgressEvent| {
            if let ProgressEvent::Warning(text) = event {
                warnings.push(text);
            }
        },
        &mut None,
    );
    assert!(failed.is_err());
    assert!(
        warnings
            .iter()
            .any(|w| w.contains("no values at 1 of the 20 nodes")),
        "{warnings:?}"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn given_global_results_drive_a_submodel_as_those_of_the_frd_file() {
    let dir = fresh("submodel");
    run_text(&dir, &(bar(4) + FIXED + PULL));
    // The submodel is the half of the bar at the free end, its cut face driven.
    let submodel = |input: &str| {
        let mut text = bar(4).replace("*NSET, NSET=FIX", "*NSET, NSET=CUT");
        text += &format!("*SUBMODEL, TYPE=NODE, INPUT=\"{input}\"\nCUT\n");
        text + "*STEP\n*STATIC\n*BOUNDARY, SUBMODEL, STEP=1\nCUT, 1, 3\n*NODE FILE\nU\n*END STEP\n"
    };
    let from_file = analysis(&submodel("job.frd"));
    let text = std::fs::read_to_string(dir.join("job.frd")).unwrap();
    let global: GlobalResults = plxs_procedures::submodel::read_global_frd(&text).unwrap();
    let mut from_values = from_file.clone();
    from_values
        .model
        .constraints
        .submodel
        .as_mut()
        .unwrap()
        .global = Source::loaded(global);
    let (expected, _) = run(&dir, from_file);
    let (results, _) = run(&dir, from_values);
    assert!(!results.increments.is_empty());
    assert_eq!(results, expected);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn given_temperatures_load_a_step_as_those_of_the_frd_file() {
    let dir = fresh("temperatures");
    let input = bar(2) + FIXED;
    let ids: Vec<u32> = analysis(&input).model.nodes.ids.clone();
    // 20 degrees at the start, then 20 + 10 x at node x.
    let temperature = |id: u32| 20.0 + 10.0 * f64::from((id - 1) % 3);
    let mut frd = String::from("    1PSTEP                         1           1           1\n");
    frd += " -4  NDTEMP      1    1\n";
    for &id in &ids {
        frd += &format!(" -1{id:>10}{:>12.5e}\n", temperature(id));
    }
    frd += " -3\n";
    std::fs::write(dir.join("temperatures.frd"), frd).unwrap();
    let text = input
        + "*INITIAL CONDITIONS, TYPE=TEMPERATURE\nNALL, 20\n*STEP\n*STATIC\n\
           *TEMPERATURE, FILE=temperatures.frd, BSTEP=1\n*NODE FILE\nU\n*END STEP\n";
    let from_file = analysis(&text);
    let values: BTreeMap<u32, f64> = ids.iter().map(|&id| (id, temperature(id))).collect();
    let mut from_values = from_file.clone();
    for field in &mut from_values.steps[0].fields {
        if let DefinedFieldKind::TemperatureFrom { source, .. } = &mut field.kind {
            *source = Source::loaded(NodalTemperatures {
                steps: BTreeMap::from([(1, values.clone())]),
            });
        }
    }
    let (expected, _) = run(&dir, from_file);
    let (results, _) = run(&dir, from_values);
    let moved = (results.increments.last())
        .and_then(|i| i.fields.iter().find(|f| f.frd_name() == "DISP"))
        .is_some_and(|f| f.values.iter().flatten().any(|v| v.abs() > 1e-9));
    assert!(moved, "the temperatures load the bar");
    assert_eq!(results, expected);
    let _ = std::fs::remove_dir_all(&dir);
}
