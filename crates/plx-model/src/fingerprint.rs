//! Fingerprints of what the cells of an analysis are made from, as Ansys Workbench tells which
//! cells are up to date. A cell keeps the fingerprint of its inputs from when it was last
//! made; when the inputs' fingerprint differs now, the cell is out of date and is made again,
//! while cells whose inputs did not change are kept. Unlike a "changed" flag a fingerprint
//! cannot be forgotten to be set.

use plx_mesh::{FeMesh, Fnv};

use crate::{ContactRegion, FeModel, Geometry, MeshSetupKind, ModelSpace};

/// The inputs of the solution: everything that goes into the input file the solver runs, the
/// mesh and the model definition. Items only used to evaluate results, result paths and
/// planes, are left out: they never need a new solution.
pub fn solve_input(mesh: &FeMesh, fe: &FeModel) -> u64 {
    steps_input(mesh, fe, fe.steps.len())
}

/// The inputs of the results of each step, in the order of the steps: the model and the
/// steps up to it. The solver solves the steps one after the other, so steps after a step never
/// change its results. The last one is [`solve_input`].
pub fn step_inputs(mesh: &FeMesh, fe: &FeModel) -> Vec<u64> {
    (1..=fe.steps.len())
        .map(|count| steps_input(mesh, fe, count))
        .collect()
}

/// The inputs of the results of the first `count` steps: [`solve_input`] of the model with
/// only these steps.
pub fn steps_input(mesh: &FeMesh, fe: &FeModel, count: usize) -> u64 {
    let mut hash = Fnv::new();
    hash.add(&mesh.input_fingerprint().to_le_bytes());
    let FeModel {
        properties,
        materials,
        sections,
        cross_sections,
        constraints,
        surface_interactions,
        contact_pairs,
        ties,
        node_ties,
        contact_regions,
        contact_detections: _,
        contact_defaults,
        bolts,
        amplitudes,
        distributions,
        initial_conditions,
        steps,
        reference_points,
        coordinate_systems,
        planes,
        result_planes: _,
        result_paths: _,
        mesh_sets,
        element_types,
        exposed: _,
    } = fe;
    // The global results of a submodel count by their folder and file name, which change
    // with the global results they are; where the work directory is does not count.
    let mut properties = properties.clone();
    let short = |path: &std::path::Path| -> std::path::PathBuf {
        let parts: Vec<_> = path.components().rev().take(2).collect();
        parts.into_iter().rev().collect()
    };
    properties.global_results = properties.global_results.as_deref().map(short);
    // So do the files of other analyses' results the model starts from.
    properties.stored_modes = properties.stored_modes.as_deref().map(short);
    if let Some((file, _)) = &mut properties.initial_temperatures {
        *file = short(file);
    }
    hash.add_debug(&properties);
    hash.add_debug(materials);
    hash.add_debug(sections);
    if !cross_sections.is_empty() {
        hash.add_debug(cross_sections);
    }
    hash.add_debug(constraints);
    hash.add_debug(surface_interactions);
    hash.add_debug(contact_pairs);
    hash.add_debug(ties);
    // Who found a node tie and whether it was changed since is no input of the solver.
    for tie in node_ties {
        hash.add_debug(&(&tie.name, tie.active, &tie.region, tie.rotations));
    }
    if !contact_regions.is_empty() {
        // Who found a region and whether it was changed since is no input of the solver.
        let regions: Vec<_> = (contact_regions.iter())
            .map(|r| ContactRegion {
                detected_by: None,
                edited: false,
                ..r.clone()
            })
            .collect();
        hash.add_debug(&regions);
        hash.add_debug(contact_defaults);
    }
    if !bolts.is_empty() {
        hash.add_debug(bolts);
    }
    hash.add_debug(amplitudes);
    hash.add_debug(distributions);
    hash.add_debug(initial_conditions);
    // Temperatures read from results files count by their folder and file name too.
    let mut steps = steps[..count.min(steps.len())].to_vec();
    for field in steps.iter_mut().flat_map(|s| s.defined_fields.iter_mut()) {
        if let crate::DefinedFieldKind::TemperatureFromFile { file, .. } = &mut field.kind {
            *file = short(file);
        }
    }
    hash.add_debug(&steps);
    hash.add_debug(reference_points);
    hash.add_debug(coordinate_systems);
    hash.add_debug(planes);
    hash.add_debug(mesh_sets);
    hash.add_debug(element_types);
    hash.finish()
}

/// The inputs of the mesh of one part of the geometry: the shapes, the model space and the
/// meshing settings that apply to the part. Changing the settings of one part leaves the
/// meshes of the others up to date.
pub fn mesh_input(geometry: &Geometry, space: ModelSpace, part: &str) -> u64 {
    let mut hash = Fnv::new();
    hash.add_str(&geometry.brep);
    hash.add_debug(&geometry.part_names);
    hash.add_debug(&space);
    hash.add_str(part);
    hash.add_debug(&geometry.parameters(part));
    hash.add_debug(&geometry.algorithms(part));
    for item in &geometry.mesh_items {
        // A local mesh size applies to faces and edges of any part.
        let applies = match &item.kind {
            MeshSetupKind::LocalMeshSize { .. } => true,
            kind => (kind.parts().unwrap_or_default().iter()).any(|p| p.eq_ignore_ascii_case(part)),
        };
        if applies {
            hash.add_debug(&item.kind);
        }
    }
    hash.finish()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MeshSetupItem, MeshingParameters, ResultPath, Step};

    fn geometry() -> Geometry {
        Geometry {
            source: "block.step".into(),
            brep: "shapes".into(),
            meshing: MeshingParameters::default(),
            mesh_items: Vec::new(),
            part_names: Vec::new(),
            refinement: 1.0,
            signatures: Vec::new(),
        }
    }

    #[test]
    fn result_paths_leave_the_solution_up_to_date() {
        let mesh = FeMesh::default();
        let mut fe = FeModel::default();
        let before = solve_input(&mesh, &fe);
        fe.result_paths.push(ResultPath::new("Path-1"));
        assert_eq!(solve_input(&mesh, &fe), before);
        fe.steps.push(Step::new_static("Step-1"));
        assert_ne!(solve_input(&mesh, &fe), before);
    }

    #[test]
    fn later_steps_leave_the_results_of_earlier_ones_up_to_date() {
        let mesh = FeMesh::default();
        let mut fe = FeModel::default();
        fe.steps.push(Step::new_static("Step-1"));
        let before = step_inputs(&mesh, &fe);
        assert_eq!(before, [solve_input(&mesh, &fe)]);
        fe.steps.push(Step::new_static("Step-2"));
        let after = step_inputs(&mesh, &fe);
        assert_eq!(after[0], before[0]);
        assert_eq!(after[1], solve_input(&mesh, &fe));
        assert_ne!(after[1], before[0]);
        // A changed earlier step changes the results of the later ones too.
        fe.steps[0].name = "Load".into();
        let changed = step_inputs(&mesh, &fe);
        assert!(changed[0] != after[0] && changed[1] != after[1]);
        // So does the model.
        fe.steps[0].name = "Step-1".into();
        fe.materials.push(Default::default());
        assert_ne!(step_inputs(&mesh, &fe)[0], before[0]);
    }

    #[test]
    fn global_results_count_by_folder_and_name() {
        let mesh = FeMesh::default();
        let mut fe = FeModel::default();
        fe.properties.global_results = Some("/tmp/a/Global-1/global.frd".into());
        let before = solve_input(&mesh, &fe);
        fe.properties.global_results = Some("/home/b/Global-1/global.frd".into());
        assert_eq!(solve_input(&mesh, &fe), before);
        fe.properties.global_results = Some("/home/b/Global-2/global.frd".into());
        assert_ne!(solve_input(&mesh, &fe), before);
    }

    #[test]
    fn temperatures_from_files_count_by_folder_and_name() {
        let mesh = FeMesh::default();
        let mut fe = FeModel::default();
        let mut step = Step::new_static("Step-1");
        let field = |file: &str| crate::DefinedField {
            distribution: None,
            name: "Temperatures".into(),
            active: true,
            region: crate::Region::Parts(Vec::new()),
            kind: crate::DefinedFieldKind::TemperatureFromFile {
                file: file.into(),
                step: 1,
            },
            amplitude: None,
        };
        step.defined_fields.push(field("/tmp/a/Global-1/heat.frd"));
        fe.steps.push(step);
        let before = solve_input(&mesh, &fe);
        fe.steps[0].defined_fields[0] = field("/home/b/Global-1/heat.frd");
        assert_eq!(solve_input(&mesh, &fe), before);
        fe.steps[0].defined_fields[0] = field("/home/b/Global-2/heat.frd");
        assert_ne!(solve_input(&mesh, &fe), before);
    }

    #[test]
    fn meshing_settings_of_one_part_leave_the_others_up_to_date() {
        let mut geometry = geometry();
        let space = ModelSpace::default();
        let (a, b) = (
            mesh_input(&geometry, space, "Solid-1"),
            mesh_input(&geometry, space, "Solid-2"),
        );
        geometry.mesh_items.push(MeshSetupItem {
            name: "Meshing_Parameters-1".into(),
            kind: MeshSetupKind::MeshingParameters {
                parts: vec!["Solid-1".into()],
                parameters: MeshingParameters {
                    max_size: 1.0,
                    ..Default::default()
                },
            },
        });
        assert_ne!(mesh_input(&geometry, space, "Solid-1"), a);
        assert_eq!(mesh_input(&geometry, space, "Solid-2"), b);
        // New shapes need every part meshed again.
        geometry.brep = "other shapes".into();
        assert_ne!(mesh_input(&geometry, space, "Solid-2"), b);
    }

    #[test]
    fn a_refinement_puts_every_part_out_of_date() {
        let mut geometry = geometry();
        let space = ModelSpace::default();
        let before = mesh_input(&geometry, space, "SOLID-1");
        geometry.refinement = 1.5;
        assert_ne!(mesh_input(&geometry, space, "SOLID-1"), before);
    }
}
