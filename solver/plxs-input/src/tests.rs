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

/// A cube on steel with a fixed face, as Nodalis writes it, with `steps` in place of the step.
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
/// The heat loads follow the same rule as `*CLOAD`/`*DLOAD` (checked against ccx 2.21 on a
/// single C3D8, see `carry.rs`): a new step replaces the carried flux at the same node, the
/// same label on the same element and the same film or radiation face.
#[test]
fn new_heat_loads_replace_the_carried_ones_of_their_kind() {
    let analysis = read(&cube(
        "*Physical constants, Absolute zero=-273.15, Stefan Boltzmann=5.67e-8\n\
*Step\n*Heat transfer, Steady state\n*Boundary\nFIX, 11, 11, 20\n*Cflux\nTOP, 11, 10\n\
*Dflux\nEALL, S2, 10\n*Dflux\nEALL, BF, 4\n*Film\nEALL, F2, 40, 7\n*Radiate\nEALL, R2, 400, 0.8\n\
*End step\n*Step\n*Heat transfer, Steady state\n*Cflux\n7, 11, 5\n*Cflux\n7, 11, 1\n\
*Dflux\n1, S2, 3\n*Film\n1, F3, 60, 2\n*Radiate\n1, R2, 300, 0.5\n*End step\n",
    ))
    .unwrap();
    let model = &analysis.model;
    let loads = &analysis.steps[1].loads;
    let cflux = |node: u32| -> f64 {
        loads
            .concentrated_flux
            .iter()
            .filter(|l| model.nodes_of(&l.region).contains(&node))
            .map(|l| l.value)
            .sum()
    };
    // Node 7 replaced by 5 + 1, the other nodes of TOP keep their 10.
    assert_eq!(cflux(7), 6.0);
    assert_eq!(cflux(5), 10.0);
    assert_eq!(cflux(8), 10.0);
    // S2 replaced, BF kept.
    let dflux: Vec<_> = loads
        .distributed_flux
        .iter()
        .map(|l| (l.kind, l.value))
        .collect();
    assert_eq!(dflux.len(), 2);
    assert!(dflux.contains(&(plxs_core::step::FluxKind::Body, 4.0)));
    assert!(dflux.contains(&(plxs_core::step::FluxKind::Face(2), 3.0)));
    // A film on another face adds, the radiation on the same face is replaced.
    let films: Vec<_> = loads
        .films
        .iter()
        .map(|f| (f.face, f.coefficient))
        .collect();
    assert_eq!(films, vec![(2, 7.0), (3, 2.0)]);
    let radiations: Vec<_> = loads.radiations.iter().map(|r| r.emissivity).collect();
    assert_eq!(radiations, vec![0.5]);
}

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
        errors[0]
            .to_string()
            .contains("is not supported by the solver yet"),
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
fn joints_read_and_render_back() {
    let text = cube(
        "*Node\n9, 0.5, 0.5, 1\n10, 0.5, 0.5, 1\n*Joint, Name=Hinge\n9, 10, 0, 0\n\
0.5, 0.5, 1, 0, 2, 0, -1, 0, 0\n1, 2, 3, 4, 5\n\
*Step\n*Static\n*Boundary\nFIX, 1, 3\n*End step\n",
    );
    let analysis = read(&text).unwrap();
    let joint = &analysis.model.constraints.joints[0];
    assert_eq!(joint.name, "HINGE");
    assert_eq!(joint.ends, [Some([9, 10]), None]);
    // The directions are normalized; z follows from x and y.
    assert_eq!(
        joint.axes,
        [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    );
    assert_eq!(joint.fixed, [true, true, true, true, true, false]);
    let again = read(&super::render(&analysis)).unwrap();
    assert_eq!(
        again.model.constraints.joints,
        analysis.model.constraints.joints
    );
    let parallel = text.replace("-1, 0, 0\n1, 2", "0, 3, 0\n1, 2");
    assert!(read(&parallel).is_err());
}

#[test]
fn the_stabilization_of_springs_reads_and_renders_back() {
    use plxs_core::section::SectionKind;
    let text = cube(
        "*Element, Type=SPRING1, Elset=WEAK\n2, 5\n\
*Spring, Elset=WEAK, Stabilize=1000\n1\n0.5\n\
*Step\n*Static\n*Boundary\nFIX, 1, 3\n*End step\n",
    );
    let stabilize = |analysis: &plxs_core::Analysis| {
        (analysis.model.sections.iter())
            .find_map(|s| match &s.kind {
                SectionKind::Spring(spring) => Some(spring.stabilize),
                _ => None,
            })
            .unwrap()
    };
    let analysis = read(&text).unwrap();
    assert_eq!(stabilize(&analysis), 1000.0);
    let rendered = super::render(&analysis);
    assert!(rendered.contains(", Stabilize=1000"), "{rendered}");
    assert_eq!(stabilize(&read(&rendered).unwrap()), 1000.0);
    let without = text.replace(", Stabilize=1000", "");
    assert_eq!(stabilize(&read(&without).unwrap()), 0.0);
    assert!(read(&text.replace("Stabilize=1000", "Stabilize=-1")).is_err());
}

#[test]
fn event_frames_of_a_dynamic_step_read_and_render_back() {
    let text = cube("*Step\n*Dynamic, Direct, Events\n0.001, 0.1\n*End step\n");
    let analysis = read(&text).unwrap();
    let Procedure::Dynamic(dynamic) = &analysis.steps[0].procedure else {
        panic!("not a dynamic step");
    };
    assert!(dynamic.event_frames);
    let rendered = super::render(&analysis);
    assert!(
        rendered.contains("*Dynamic, Direct, Events\n"),
        "{rendered}"
    );
    let plain = read(&cube("*Step\n*Dynamic, Direct\n0.001, 0.1\n*End step\n")).unwrap();
    let Procedure::Dynamic(dynamic) = &plain.steps[0].procedure else {
        panic!("not a dynamic step");
    };
    assert!(!dynamic.event_frames);
}

#[test]
fn a_no_analysis_step_reads_the_model_only() {
    let analysis = read(&cube("*Step\n*No analysis\n*End step\n")).unwrap();
    assert!(matches!(analysis.steps[0].procedure, Procedure::NoAnalysis));
}

/// Every fixture of the solver corpus, rendered as text and read again, is the same
/// analysis: the renderer says everything the reader took from the file.
#[test]
fn every_fixture_renders_as_text_that_reads_back_the_same() {
    let files = inp_files(&repo().join("solver/fixtures"));
    assert!(files.len() > 200, "{}", files.len());
    let mut failed = Vec::new();
    for file in files {
        let mut first = super::read_inp(&file)
            .unwrap_or_else(|e| panic!("{}: {e:?}", file.display()))
            .analysis;
        first.model.stored_modes_file = None;
        let text = super::render(&first);
        assert!(
            super::unwritable_names(&first).is_empty(),
            "{}",
            file.display()
        );
        let again = match read_inp_str(&text, None) {
            Ok(reading) => reading.analysis,
            Err(e) => {
                failed.push(format!("{}: {e:?}", file.display()));
                continue;
            }
        };
        if again != first {
            failed.push(format!(
                "{}: {}",
                file.display(),
                first_difference(&first, &again)
            ));
        }
    }
    assert!(
        failed.is_empty(),
        "{} differ:\n{}",
        failed.len(),
        failed.join("\n")
    );
}

/// Where two analyses differ first, for the message of a failed comparison.
fn first_difference(a: &Analysis, b: &Analysis) -> String {
    fn walk(path: &str, a: &serde_json::Value, b: &serde_json::Value) -> Option<String> {
        use serde_json::Value;
        match (a, b) {
            (Value::Object(x), Value::Object(y)) => {
                for (key, value) in x {
                    let other = y.get(key).unwrap_or(&Value::Null);
                    if let Some(d) = walk(&format!("{path}.{key}"), value, other) {
                        return Some(d);
                    }
                }
                None
            }
            (Value::Array(x), Value::Array(y)) => {
                for (i, (p, q)) in x.iter().zip(y).enumerate() {
                    if let Some(d) = walk(&format!("{path}[{i}]"), p, q) {
                        return Some(d);
                    }
                }
                (x.len() != y.len()).then(|| format!("{path}: {} vs {} entries", x.len(), y.len()))
            }
            _ => (a != b).then(|| format!("{path}: {a} vs {b}")),
        }
    }
    let (a, b) = (
        serde_json::to_value(a).unwrap(),
        serde_json::to_value(b).unwrap(),
    );
    walk("", &a, &b).unwrap_or_else(|| "only in the indices".into())
}

#[test]
fn numbers_render_exactly_and_short() {
    use super::render::number;
    for value in [
        0.0,
        1.0,
        -2.5,
        0.1,
        1e-9,
        7.85e-9,
        210000.0,
        1.0 / 3.0,
        f64::MAX,
        1e300,
    ] {
        let text = number(value);
        assert_eq!(text.parse::<f64>().unwrap(), value, "{text}");
        assert!(text.len() <= 24, "{text}");
    }
    assert_eq!(number(210000.0), "210000");
    assert_eq!(number(7.85e-9), "7.85e-9");
}

#[test]
fn hyperelastic_models_are_read_and_rendered_back() {
    use plxs_core::material::HyperelasticModel as H;
    for (card, model, count) in [
        ("*Hyperelastic, Neo Hooke\n0.5, 0.01\n", H::NeoHooke, 2),
        (
            "*Hyperelastic, Mooney-Rivlin\n0.4, 0.1, 0.01, 20\n",
            H::MooneyRivlin,
            3,
        ),
        (
            "*Hyperelastic, Yeoh\n0.5, -0.01, 0.001, 0.01, 0, 0\n",
            H::Yeoh,
            6,
        ),
        (
            "*Hyperelastic, Polynomial, N=2\n0.4, 0.1, 0.01, 0, 0.002, 0.01, 0\n",
            H::Polynomial(2),
            7,
        ),
        (
            "*Hyperelastic, Ogden, N=3\n0.6, 1.3, 0.002, 5, -0.01, -2, 0.01, 0\n0\n",
            H::Ogden(3),
            9,
        ),
        (
            "*Hyperelastic, Arruda-Boyce\n0.5, 7, 0.01\n",
            H::ArrudaBoyce,
            3,
        ),
    ] {
        let text = cube("*Step\n*Static\n*Boundary\nFIX, 1, 3\n*End step\n")
            .replace("*Elastic\n210000, 0.3\n", card);
        let analysis = read(&text).unwrap_or_else(|e| panic!("{card}: {e:?}"));
        let hyper = analysis.model.materials[0].hyperelastic.clone().unwrap();
        assert_eq!(hyper.model, model, "{card}");
        assert_eq!(hyper.constants.len(), count, "{card}");
        let again = read(&super::render(&analysis)).unwrap();
        assert_eq!(again.model.materials, analysis.model.materials, "{card}");
    }
    let missing = cube("*Step\n*Static\n*End step\n")
        .replace("*Elastic\n210000, 0.3\n", "*Hyperelastic\n0.5, 0.01\n");
    assert!(read(&missing).is_err());
    let temperatures = cube("*Step\n*Static\n*End step\n").replace(
        "*Elastic\n210000, 0.3\n",
        "*Hyperelastic, Neo Hooke\n0.5, 0.01, 20\n0.4, 0.01, 80\n",
    );
    assert!(read(&temperatures).is_err());
}

/// Signal amplitudes read with their parameters, render back the same and reject what the
/// solver cannot evaluate.
#[test]
fn signal_amplitudes_read_and_render_back() {
    use plxs_core::amplitude::Signal;
    let step = "*Amplitude, Name=Kick, Shape=Pulse\n0.1, 0.002\n\
*Amplitude, Name=Shake, Shape=Sine\n0, 1, 50, 90\n\
*Amplitude, Name=Sweep, Shape=Chirp, Sweep=Log\n0.2, 0.8, 10, 1000\n\
*Step\n*Dynamic, Direct\n0.001, 1\n*Boundary\nFIX, 1, 3, 0\n*Cload\n5, 3, -1, , SWEEP\n*End step\n";
    let analysis = read(&cube(step)).unwrap();
    let signal = |name: &str| analysis.model.amplitude(name).unwrap().signal.clone();
    assert_eq!(
        signal("KICK"),
        Some(Signal::Pulse {
            start: 0.1,
            width: 0.002
        })
    );
    let Some(Signal::Sine { phase, .. }) = signal("SHAKE") else {
        panic!("{:?}", signal("SHAKE"))
    };
    assert!((phase - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    assert!((analysis.model.amplitude("SHAKE").unwrap().value(0.0) - 1.0).abs() < 1e-12);
    assert!(matches!(
        signal("SWEEP"),
        Some(Signal::Chirp {
            logarithmic: true,
            ..
        })
    ));
    let again = read(&super::render(&analysis)).unwrap();
    for name in ["KICK", "SHAKE", "SWEEP"] {
        let a = signal(name).unwrap();
        let b = again.model.amplitude(name).unwrap().signal.clone().unwrap();
        for t in [0.0, 0.1, 0.101, 0.3, 0.55, 0.9] {
            assert!((a.value(t) - b.value(t)).abs() < 1e-9, "{name} at {t}");
        }
    }
    for wrong in [
        "*Amplitude, Name=A, Shape=Pulse\n0.1, 0\n",
        "*Amplitude, Name=A, Shape=Sine\n0, 1, 50\n",
        "*Amplitude, Name=A, Shape=Chirp, Sweep=Log\n0, 1, 0, 100\n",
        "*Amplitude, Name=A, Shape=Sine\n1, 0.5, 50, 0\n",
        "*Amplitude, Name=A, Shape=Square\n0, 1\n",
        "*Amplitude, Name=A, Sweep=Log\n0, 0, 1, 1\n",
    ] {
        assert!(read(&cube(wrong)).is_err(), "{wrong}");
    }
}

#[test]
fn anisotropic_elasticity_and_orientations_read_and_render_back() {
    use plxs_core::material::AnisotropicElastic;
    use plxs_core::section::SectionKind;
    let steps = "*Step\n*Static\n*Boundary\nFIX, 1, 3\n*End step\n";
    let text = cube(steps)
        .replace(
            "*Elastic\n210000, 0.3\n",
            "*Elastic, Type=Engineering constants\n3000, 2000, 1000, 0.3, 0.2, 0.25, 800, 600,\n400\n",
        )
        .replace(
            "*Solid section, Elset=EALL, Material=Steel\n",
            "*Orientation, Name=Turned\n0, 1, 0, -1, 0, 0\n\
*Solid section, Elset=EALL, Material=Steel, Orientation=Turned\n",
        );
    let check = |analysis: &plxs_core::Analysis| {
        let material = &analysis.model.materials[0];
        assert!(material.elastic.is_none());
        assert_eq!(
            material.anisotropic,
            Some(AnisotropicElastic::EngineeringConstants([
                3000.0, 2000.0, 1000.0, 0.3, 0.2, 0.25, 800.0, 600.0, 400.0
            ]))
        );
        let SectionKind::Solid { orientation, .. } = &analysis.model.sections[0].kind else {
            panic!("no solid section");
        };
        assert_eq!(orientation.as_deref(), Some("TURNED"));
        let axes = analysis
            .model
            .orientation("TURNED")
            .unwrap()
            .axes()
            .unwrap();
        assert_eq!(axes[2], [0.0, 0.0, 1.0]);
    };
    let analysis = read(&text).unwrap();
    check(&analysis);
    let rendered = super::render(&analysis);
    check(&read(&rendered).unwrap());
    let aniso = cube(steps).replace(
        "*Elastic\n210000, 0.3\n",
        "*Elastic, Type=Aniso\n1, 0, 1, 0, 0, 1, 0, 0,\n0, 1, 0, 0, 0, 0, 1, 0,\n0, 0, 0, 0, 1\n",
    );
    let analysis = read(&aniso).unwrap();
    assert!(matches!(
        analysis.model.materials[0].anisotropic,
        Some(AnisotropicElastic::Anisotropic(_))
    ));
    assert!(read(&text.replace("Name=Turned\n0, 1, 0, -1", "Name=Turned\n0, 1, 0, 0")).is_err());
    assert!(read(&text.replace("Name=Turned", "Name=Turned, System=Cylindrical")).is_err());
}
