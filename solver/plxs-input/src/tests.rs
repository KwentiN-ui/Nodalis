use std::path::{Path, PathBuf};

use plxs_core::mesh::NodeRef;
use plxs_core::step::{DistributedLoadKind, Procedure};
use plxs_core::{Analysis, Error};

use super::read_inp_str;

fn repo() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn read(text: &str) -> Result<Analysis, Vec<Error>> {
    read_inp_str(text, None).map(|r| r.analysis)
}

/// A cube on steel with a fixed face, as PrePoLix writes it, with `steps` in place of the step.
fn cube(steps: &str) -> String {
    format!(
        "*Heading\ncube\n*Node\n1, 0, 0, 0\n2, 1, 0, 0\n3, 1, 1, 0\n4, 0, 1, 0\n5, 0, 0, 1\n6, 1, 0, 1\n7, 1, 1, 1\n8, 0, 1, 1\n\
*Element, Type=C3D8, Elset=EALL\n1, 1, 2, 3, 4, 5, 6, 7, 8\n*Nset, Nset=FIX\n1, 2, 3, 4\n*Nset, Nset=TOP\n5, 6, 7, 8\n\
*Surface, Name=TOP_S2, Type=Element\nEALL, S2\n*Material, Name=Steel\n*Elastic\n210000, 0.3\n*Density\n7.85e-9\n\
*Solid section, Elset=EALL, Material=Steel\n*Amplitude, Name=Ramp\n0, 0, 1, 1\n{steps}"
    )
}

fn inp_files(dir: &Path) -> Vec<PathBuf> {
    let mut files = Vec::new();
    for entry in std::fs::read_dir(dir).unwrap().flatten() {
        let path = entry.path();
        if path.is_dir() {
            files.extend(inp_files(&path));
        } else if path.extension().is_some_and(|e| e == "inp") {
            files.push(path);
        }
    }
    files.sort();
    files
}

#[test]
fn every_example_of_testdata_reads() {
    let files = inp_files(&repo().join("testdata"));
    assert!(files.len() >= 9, "{files:?}");
    for file in files {
        let reading =
            super::read_inp(&file).unwrap_or_else(|e| panic!("{}: {e:?}", file.display()));
        let plx = plx_io::inp::read_inp(&file).unwrap();
        let model = &reading.analysis.model;
        assert_eq!(
            model.nodes.len(),
            plx.mesh.node_count(),
            "{}",
            file.display()
        );
        // plx-io skips springs and gaps, which the solver reads.
        assert_eq!(
            model.elements.len() - plx.skipped_elements.len(),
            plx.mesh.element_count(),
            "{}",
            file.display()
        );
        for (name, nodes) in &plx.mesh.node_sets {
            assert_eq!(
                model.sets.node_sets.get(name),
                Some(nodes),
                "{}: set {name}",
                file.display()
            );
        }
    }
}

/// Every fixture of the solver corpus, which the `plx-io` writer tests produced, reads
/// without error; its nodes, elements and sets agree with what `plx-io` reads.
#[test]
fn every_fixture_of_the_corpus_reads() {
    let files = inp_files(&repo().join("solver/fixtures"));
    assert!(files.len() >= 150, "{} fixtures", files.len());
    for file in files {
        let reading =
            super::read_inp(&file).unwrap_or_else(|e| panic!("{}: {e:?}", file.display()));
        let plx = plx_io::inp::read_inp(&file).unwrap();
        let model = &reading.analysis.model;
        assert_eq!(
            model.nodes.len(),
            plx.mesh.node_count(),
            "{}",
            file.display()
        );
        assert_eq!(
            model.elements.len() - plx.skipped_elements.len(),
            plx.mesh.element_count(),
            "{}",
            file.display()
        );
        for (name, nodes) in &plx.mesh.node_sets {
            // Set members that are no nodes are dropped by the solver, as CalculiX does.
            let nodes: Vec<u32> = nodes
                .iter()
                .copied()
                .filter(|&n| plx.mesh.node_index(n).is_some())
                .collect();
            assert_eq!(
                model.sets.node_sets.get(name),
                Some(&nodes),
                "{}: set {name}",
                file.display()
            );
        }
        for (name, elements) in &plx.mesh.element_sets {
            let plx_elements: Vec<u32> = elements
                .iter()
                .copied()
                .filter(|&e| model.element_index(e).is_some())
                .collect();
            assert_eq!(
                model.sets.element_sets.get(name),
                Some(&plx_elements),
                "{}: set {name}",
                file.display()
            );
        }
        assert!(!reading.analysis.steps.is_empty(), "{}", file.display());
        for step in &reading.analysis.steps {
            assert!(
                !step.outputs.node_file.is_empty()
                    || matches!(step.procedure, Procedure::NoAnalysis),
                "{}: step {} has no field output",
                file.display(),
                step.name
            );
        }
    }
}

#[test]
fn boundaries_and_loads_carry_over_unless_reset() {
    let analysis = read(&cube(
        "*Step\n*Static\n*Boundary\nFIX, 1, 3\n*Cload\nTOP, 3, -1\n*Node file\nU\n*End step\n\
*Step\n*Static\n*Cload\nTOP, 1, 2\n*End step\n\
*Step\n*Static\n*Boundary, op=New\nFIX, 1, 1\n*Cload, op=New\n*Dload\nEALL, P2, 5\n*End step\n",
    ))
    .unwrap();
    let steps = &analysis.steps;
    assert_eq!(steps.len(), 3);
    assert_eq!(steps[0].name, "Step-1");
    assert_eq!(steps[1].boundaries.len(), 1);
    assert_eq!(steps[1].loads.concentrated.len(), 2, "kept and added");
    assert_eq!(
        steps[1].outputs.node_file.len(),
        1,
        "output requests carry over"
    );
    assert_eq!(steps[2].boundaries.len(), 1);
    assert_eq!(steps[2].boundaries[0].last_dof, 1);
    assert!(
        steps[2].loads.concentrated.is_empty(),
        "OP=NEW drops the loads"
    );
    assert!(matches!(
        steps[2].loads.distributed[0].kind,
        DistributedLoadKind::Pressure { face: 2, value } if value == 5.0
    ));
}

/// The concentrated load of a step at a node and DOF, with sets resolved.
fn cload_at(analysis: &Analysis, step: usize, node: u32, dof: u8) -> f64 {
    let model = &analysis.model;
    analysis.steps[step]
        .loads
        .concentrated
        .iter()
        .filter(|l| l.dof == dof && model.nodes_of(&l.region).contains(&node))
        .map(|l| l.value)
        .sum()
}

/// What CalculiX 2.21 does with loads redefined without OP=NEW (the numbers were checked
/// against ccx runs of the same cards on a cantilever, see `carry.rs`): a new step replaces
/// the carried value at the node and DOF it names, within a step the cards add up.
#[test]
fn a_new_cload_replaces_the_carried_one_at_its_nodes_only() {
    let analysis = read(&cube(
        "*Step\n*Static\n*Boundary\nFIX, 1, 3\n*Cload\nTOP, 3, -100\n*Cload\n5, 3, -1\n*End step\n\
*Step\n*Static\n*Cload\n6, 3, -50\n*Cload\n6, 3, -20\n*Cload\n7, 1, 4\n*End step\n",
    ))
    .unwrap();
    // Step 1: the two cards on node 5 add up.
    assert_eq!(cload_at(&analysis, 0, 5, 3), -101.0);
    assert_eq!(cload_at(&analysis, 0, 6, 3), -100.0);
    // Step 2: node 6 replaced by -50 - 20, nodes 5, 7, 8 keep theirs, node 7 adds DOF 1.
    assert_eq!(cload_at(&analysis, 1, 5, 3), -101.0);
    assert_eq!(cload_at(&analysis, 1, 6, 3), -70.0);
    assert_eq!(cload_at(&analysis, 1, 7, 3), -100.0);
    assert_eq!(cload_at(&analysis, 1, 8, 3), -100.0);
    assert_eq!(cload_at(&analysis, 1, 7, 1), 4.0);
}

#[test]
fn a_new_dload_replaces_the_carried_one_of_its_kind() {
    let analysis = read(&cube(
        "*Step\n*Static\n*Boundary\nFIX, 1, 3\n*Dload\nEALL, P2, 1\n\
*Dload\nEALL, Grav, 9810, 0, 0, -1\n*Dload\nEALL, Grav, 9810, 0, -1, 0\n\
*Dload\nEALL, Centrif, 100, 0, 0, 0, 0, 0, 1\n*End step\n\
*Step\n*Static\n*Dload\n1, P2, 3\n*Dload\nEALL, Grav, 19620, 0, 0, -1\n\
*Dload\nEALL, Centrif, 400, 0, 0, 0, 0, 1, 0\n*End step\n",
    ))
    .unwrap();
    let step = &analysis.steps[1];
    let pressures: Vec<f64> = (step.loads.distributed.iter())
        .filter_map(|l| match l.kind {
            DistributedLoadKind::Pressure { value, .. } => Some(value),
            _ => None,
        })
        .collect();
    assert_eq!(pressures, [3.0]);
    let gravity: Vec<(f64, [f64; 3])> = (step.loads.distributed.iter())
        .filter_map(|l| match l.kind {
            DistributedLoadKind::Gravity {
                magnitude,
                direction,
            } => Some((magnitude, direction)),
            _ => None,
        })
        .collect();
    // The gravity in z is replaced, the one in y stays.
    assert_eq!(gravity.len(), 2, "{gravity:?}");
    assert!(gravity.contains(&(9810.0, [0.0, -1.0, 0.0])));
    assert!(gravity.contains(&(19620.0, [0.0, 0.0, -1.0])));
    // A centrifugal load about another axis is ignored like CalculiX does, with a warning.
    let centrifugal: Vec<f64> = (step.loads.distributed.iter())
        .filter_map(|l| match l.kind {
            DistributedLoadKind::Centrifugal { omega_squared, .. } => Some(omega_squared),
            _ => None,
        })
        .collect();
    assert_eq!(centrifugal, [100.0]);
    let warnings = read_inp_str(
        &cube(
            "*Step\n*Static\n*Dload\nEALL, Centrif, 100, 0, 0, 0, 0, 0, 1\n*End step\n\
*Step\n*Static\n*Dload\nEALL, Centrif, 400, 0, 0, 0, 0, 1, 0\n*End step\n",
        ),
        None,
    )
    .unwrap()
    .warnings;
    assert!(
        warnings.iter().any(|w| w.contains("CENTRIF")),
        "{warnings:?}"
    );
}

/// Loads following a step-time amplitude go on with their value at the end of their step;
/// total-time amplitudes keep acting (ccx 2.21).
#[test]
fn carried_loads_keep_their_final_value_of_a_step_time_amplitude() {
    let analysis = read(&cube(
        "*Amplitude, Name=Total, Time=Total time\n0, 0, 4, 4\n\
*Step\n*Static\n0.5, 0.5\n*Boundary\nFIX, 1, 3\n*Boundary, Amplitude=Ramp\n5, 3, 3, 0.1\n\
*Cload, Amplitude=Ramp\n6, 3, -100\n*Cload, Amplitude=Total\n7, 3, -100\n\
*Dload, Amplitude=Ramp\nEALL, P2, 2\n*End step\n\
*Step\n*Static\n2, 2\n*End step\n",
    ))
    .unwrap();
    let step = &analysis.steps[1];
    assert_eq!(cload_at(&analysis, 1, 6, 3), -50.0);
    assert_eq!(cload_at(&analysis, 1, 7, 3), -100.0);
    let total = step.loads.concentrated.iter().find(|l| l.value == -100.0);
    assert_eq!(total.unwrap().amplitude.as_deref(), Some("TOTAL"));
    assert!(matches!(
        step.loads.distributed[0].kind,
        DistributedLoadKind::Pressure { value, .. } if value == 1.0
    ));
    assert_eq!(step.loads.distributed[0].amplitude, None);
    let lift = step
        .boundaries
        .iter()
        .find(|b| b.first_dof == 3 && b.value.is_some());
    assert_eq!(lift.unwrap().value, Some(0.05));
    assert_eq!(lift.unwrap().amplitude, None);
}

#[test]
fn a_perturbation_step_does_not_carry_its_loads_on() {
    let analysis = read(&cube(
        "*Step\n*Static\n*Boundary\nFIX, 1, 3\n*Cload\nTOP, 3, -1\n*Node file\nU\n*End step\n\
*Step, Perturbation\n*Frequency\n3\n*Cload\nTOP, 1, 7\n*End step\n\
*Step\n*Static\n*End step\n",
    ))
    .unwrap();
    assert_eq!(analysis.steps[1].loads.concentrated.len(), 2);
    assert_eq!(analysis.steps[2].loads.concentrated.len(), 1);
    assert!(matches!(&analysis.steps[1].procedure, Procedure::Frequency(f) if f.count == 3));
    assert!(analysis.steps[1].perturbation);
}

#[test]
fn names_are_upper_case_and_references_resolve() {
    let analysis = read(&cube(
        "*Step\n*Static\n*Boundary\nfix, 1, 3\n*Cload, Amplitude=ramp\ntop, 3, -1\n*Dload\neall, Grav, 9810, 0, 0, -1\n*End step\n",
    ))
    .unwrap();
    let step = &analysis.steps[0];
    assert_eq!(step.boundaries[0].region, NodeRef::Set("FIX".into()));
    assert_eq!(
        step.loads.concentrated[0].amplitude.as_deref(),
        Some("RAMP")
    );
    assert!(matches!(
        step.loads.distributed[0].kind,
        DistributedLoadKind::Gravity { magnitude, direction: [0.0, 0.0, -1.0] } if magnitude == 9810.0
    ));
    assert_eq!(analysis.model.elements[0].section, Some(0));
}

#[test]
fn errors_use_calculix_wording() {
    let errors = read(&cube(
        "*Step\n*Static\n*Boundary\nNOWHERE, 1, 3\n*End step\n",
    ))
    .unwrap_err();
    assert_eq!(
        errors[0].to_string(),
        "*ERROR in *BOUNDARY: node set NOWHERE (step Step-1) has not been defined"
    );
    let errors = read(&cube("*Step\n*Static\n*Hyperfoam\n*End step\n")).unwrap_err();
    assert!(
        errors[0].to_string().contains("not supported by plxs"),
        "{}",
        errors[0]
    );
    let errors = read(&cube("*Step\n*Static\n*Nonsense\n*End step\n")).unwrap_err();
    assert_eq!(
        errors[0].to_string(),
        "*ERROR in calinput: keyword *NONSENSE cannot be interpreted"
    );
    let errors = read(&cube(
        "*Step\n*Static\n*Boundary, Foo=1\nFIX, 1\n*End step\n",
    ))
    .unwrap_err();
    assert_eq!(
        errors[0].to_string(),
        "*ERROR reading *BOUNDARY: unknown parameter FOO"
    );
    assert!(errors[0].location().is_some());
    let errors = read(&cube("*Step\n*Static\n*Boundary\nFIX, 1, 3\n")).unwrap_err();
    assert!(
        errors[0].to_string().contains("no *END STEP"),
        "{}",
        errors[0]
    );
}

#[test]
fn equations_and_generated_sets_are_read() {
    let analysis = read(&cube(
        "*Nset, Nset=GEN, Generate\n1, 7, 2\n*Equation\n3\n5, 1, 1, 6, 1, -0.5,\n7, 1, -0.5\n\
*Step\n*Static\n*Boundary\nFIX, 1, 3\n*End step\n",
    ))
    .unwrap();
    assert_eq!(analysis.model.sets.node_sets["GEN"], [1, 3, 5, 7]);
    let equation = &analysis.model.constraints.equations[0];
    assert_eq!(equation.terms.len(), 3);
    assert_eq!(equation.terms[2].coefficient, -0.5);
}

#[test]
fn a_no_analysis_step_reads_the_model_only() {
    let analysis = read(&cube("*Step\n*No analysis\n*End step\n")).unwrap();
    assert!(matches!(analysis.steps[0].procedure, Procedure::NoAnalysis));
}
