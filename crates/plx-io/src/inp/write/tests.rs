use std::path::PathBuf;
use std::process::Command;

use plx_mesh::CadEntity;
use plx_model::{
    BeamOrientation, BeamProfile, BeamSection, Behavior, BoundaryCondition, CrossSection,
    DefinedField, DefinedFieldKind, Displacement, DynamicStep, Elastic, EquationSolver, Hardening,
    Load, Material, ModalDamping, ModalDynamicsStep, NodeTie, Plastic, PlasticPoint,
    ReferencePoint, RigidBody, Section, SectionKind, SteadyStateDynamicsStep, UnitSystem,
};

use super::*;
use crate::frd::{FrdImport, read_frd};
use crate::inp::{read_inp, read_inp_str};

fn testdata(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name)
}

/// The cantilever of `kragbalken_c3d8.inp`, set up the way the GUI would.
fn cantilever(load: Load) -> (FeMesh, FeModel) {
    analysis("kragbalken_c3d8.inp", load)
}

/// A test model with steel on part EALL, held at node set FIX.
fn analysis(file: &str, load: Load) -> (FeMesh, FeModel) {
    let mesh = read_inp(&testdata(file)).unwrap().mesh;
    let mut step = Step::new_static("Step-1");
    step.boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Fixed-1".into(),
        active: true,
        visible: true,
        region: Region::NodeSet("FIX".into()),
        kind: BoundaryKind::displacement([Some(0.0); 6]),
        amplitude: None,
    });
    step.loads.push(load);
    let model = FeModel {
        properties: Default::default(),
        materials: vec![Material {
            name: "Steel".into(),
            density: Some(7.85e-9),
            elastic: Some(Elastic {
                young: 210_000.0,
                poisson: 0.3,
            }),
            ..Default::default()
        }],
        sections: vec![Section {
            name: "Section-1".into(),
            material: "Steel".into(),
            region: Region::Parts(vec!["EALL".into()]),
            thickness: 1.0,
            kind: SectionKind::Solid,
        }],
        steps: vec![step],
        ..FeModel::default()
    };
    (mesh, model)
}

fn tip_force() -> Load {
    Load {
        distribution: None,
        name: "Force-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(vec![99]),
        kind: LoadKind::force([0.0, 0.0, -100.0]),
        amplitude: None,
        factor_amplitude: None,
    }
}

#[test]
fn writes_mesh_and_analysis_that_read_back() {
    let (mesh, model) = cantilever(tip_force());
    let text = write_inp(&mesh, &model, "Kragbalken").unwrap();
    for line in [
        "** Nodes +++++++++++++++++++++++++++++++++++++++++++++++++++\n",
        "*Node\n1, 0.00000000E0, 0.00000000E0, 0.00000000E0\n",
        "*Element, Type=C3D8, Elset=EALL\n",
        "*Elset, Elset=Internal_Selection-1_Section-1\nEALL\n",
        "*Nset, Nset=Internal_Selection-1_Force-1\n99\n",
        "*Material, Name=Steel\n*Density\n0.00000000785\n*Elastic\n210000, 0.3\n",
        "*Solid section, Elset=Internal_Selection-1_Section-1, Material=Steel\n",
        "*Step\n*Static\n",
        "*Boundary, op=New\n",
        "*Boundary\nFIX, 1, 6, 0\n",
        "*Cload, op=New\n*Dload, op=New\n",
        "*Cload\nInternal_Selection-1_Force-1, 3, -100\n",
        "*Node file\nRF, U\n",
        "*El file\nS, E, NOE\n",
        "*End step\n",
    ] {
        assert!(text.contains(line), "missing {line:?} in\n{text}");
    }
    let read = read_inp_str(&text, None).unwrap().mesh;
    assert_eq!(read.node_count(), mesh.node_count());
    assert_eq!(read.elements(), mesh.elements());
    assert_eq!(read.node_sets["FIX"], mesh.node_sets["FIX"]);
    assert_eq!(read.surfaces["TIP"], mesh.surfaces["TIP"]);
    // The part set must not be written twice.
    assert_eq!(read.element_sets["EALL"].len(), 40);
}

#[test]
fn field_outputs_are_written_with_their_options_like_prepomax_does() {
    let (mesh, mut model) = cantilever(tip_force());
    let outputs = &mut model.steps[0].field_outputs;
    outputs[0].last_iterations = true;
    outputs[0].global = false;
    outputs[1].contact_elements = true;
    outputs[1].dimension = OutputDimension::ThreeD;
    outputs[1].variables = vec!["S".into(), "ERR".into()];
    outputs.push(FieldOutput::new(
        "CF_Output-1",
        OutputKind::Contact,
        &["CDIS", "CSTR"],
    ));
    let mut off = FieldOutput::new("NF_Output-1", OutputKind::Node, &["V"]);
    off.active = false;
    outputs.push(off);
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        // The node output follows the element output's form.
        "** Name: NF-Output-1\n*Node file, Last iterations, Output=3D, Global=No\nRF, U\n",
        "** Name: EF-Output-1\n*El file, Contact elements, Output=3D\nS, ERR\n",
        "** Name: CF_Output-1\n*Contact file\nCDIS, CSTR\n",
        "** Name: NF_Output-1: Deactivated\n",
    ] {
        assert!(text.contains(line), "missing {line:?} in\n{text}");
    }
}

#[test]
fn calculix_accepts_the_options_of_field_outputs() {
    let (mesh, mut model) = cantilever(tip_force());
    let outputs = &mut model.steps[0].field_outputs;
    outputs[0].last_iterations = true;
    outputs[0].global = false;
    outputs[1].dimension = OutputDimension::ThreeD;
    outputs[1].variables.push("ERR".into());
    let Some(frd) = run_ccx("feldausgabe", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    assert!(node_value(&frd, "DISP", "U3", 99) < 0.0);
    assert!(frd.increments.last().unwrap().field("ERROR").is_some());
}

#[test]
fn the_chosen_solver_is_written_with_the_procedure() {
    let (mesh, mut model) = cantilever(tip_force());
    model.resolve_default_solver(EquationSolver::Pardiso);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Step\n*Static, Solver=Pardiso\n"), "{text}");
    let StepKind::Static(settings) = &mut model.steps[0].kind else {
        unreachable!()
    };
    settings.solver = EquationSolver::IterativeCholesky;
    settings.incrementation = Incrementation::Direct;
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Static, Solver=Iterative Cholesky, Direct\n1, 1\n"),
        "{text}"
    );
    // A solver chosen in the step is kept.
    model.resolve_default_solver(EquationSolver::Pardiso);
    let StepKind::Static(settings) = &model.steps[0].kind else {
        unreachable!()
    };
    assert_eq!(settings.solver, EquationSolver::IterativeCholesky);
}

#[test]
fn picked_faces_become_one_element_set_per_face_number() {
    let load = Load {
        distribution: None,
        name: "Pressure 1".into(),
        active: true,
        visible: true,
        region: Region::Faces(vec![(10, 4), (20, 4), (1, 6)]),
        kind: LoadKind::Pressure(2.5),
        amplitude: None,
        factor_amplitude: None,
    };
    let (mesh, model) = cantilever(load);
    let text = write_inp(&mesh, &model, "").unwrap();
    let side = "Internal-1_Internal_Selection-1_Pressure_1";
    for line in [
        format!("*Elset, Elset={side}_S4\n10, 20\n"),
        format!("*Elset, Elset={side}_S6\n1\n"),
        format!("*Surface, Name=Internal_Selection-1_Pressure_1, Type=Element\n{side}_S4, S4\n"),
        format!("*Dload\n{side}_S4, P4, 2.5\n{side}_S6, P6, 2.5\n"),
    ] {
        assert!(text.contains(&line), "missing {line:?} in\n{text}");
    }
}

#[test]
fn empty_regions_and_unknown_materials_are_errors() {
    let (mesh, mut model) = cantilever(Load {
        distribution: None,
        name: "Force-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(Vec::new()),
        kind: LoadKind::force([1.0, 0.0, 0.0]),
        amplitude: None,
        factor_amplitude: None,
    });
    assert_eq!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::EmptyRegion {
            item: "Force-1".into(),
            what: "nodes"
        })
    );
    model.sections[0].material = "Alu".into();
    assert!(matches!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::UnknownMaterial { .. })
    ));
}

/// Runs CalculiX on the input file, if it is installed; CI does not have it.
fn run_ccx(name: &str, text: &str) -> Option<FrdImport> {
    dump_fixture(name, text);
    let dir = ccx_dir(name)?;
    let frd = run_ccx_in(&dir, name, text);
    remove_ccx_dir(&dir);
    Some(frd)
}

/// Keeps the input file as a fixture of the solver corpus (`solver/fixtures`, see
/// `scripts/solver_reference.sh`) when `PLXS_FIXTURE_DIR` names the directory to collect them
/// in. Two tests must not share a name with different inputs.
fn dump_fixture(name: &str, text: &str) {
    let Some(dir) = std::env::var_os("PLXS_FIXTURE_DIR") else {
        return;
    };
    let dir = PathBuf::from(dir);
    std::fs::create_dir_all(&dir).unwrap();
    let name = name.to_ascii_lowercase().replace(' ', "-");
    let path = dir.join(format!("{name}.inp"));
    if let Ok(existing) = std::fs::read_to_string(&path) {
        assert_eq!(
            existing, text,
            "fixture {name} written twice with different inputs"
        );
    }
    std::fs::write(path, text).unwrap();
}

/// A fresh directory for a CalculiX run, `None` without CalculiX.
fn ccx_dir(name: &str) -> Option<PathBuf> {
    if Command::new("ccx").arg("-v").output().is_err() {
        eprintln!("ccx not found, test skipped");
        return None;
    }
    let dir = std::env::temp_dir().join(format!("plx-write-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    Some(dir)
}

/// Runs CalculiX on the input file in the directory and reads the results.
fn run_ccx_in(dir: &std::path::Path, name: &str, text: &str) -> FrdImport {
    std::fs::write(dir.join(format!("{name}.inp")), text).unwrap();
    let output = Command::new("ccx")
        .args(["-i", name])
        .current_dir(dir)
        .env("OMP_NUM_THREADS", "1")
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout);
    assert!(output.status.success() && !log.contains("*ERROR"), "{log}");
    read_frd(&dir.join(format!("{name}.frd"))).unwrap()
}

/// Removes the files of a run; PLX_KEEP_CCX keeps them for a look at them.
fn remove_ccx_dir(dir: &std::path::Path) {
    if std::env::var_os("PLX_KEEP_CCX").is_none() {
        let _ = std::fs::remove_dir_all(dir);
    }
}

fn node_value(frd: &FrdImport, field: &str, component: &str, node: NodeId) -> f64 {
    let index = frd.mesh.node_index(node).unwrap();
    let increment = frd.increments.last().unwrap();
    let field = increment.field(field).unwrap();
    f64::from(field.component(component).unwrap().values[index])
}

/// The value at a node of `mesh`, which CalculiX may have expanded: the mean over the nodes
/// it was expanded to, or with `sum` their sum, as of forces.
fn expanded_value(
    frd: &FrdImport,
    mesh: &FeMesh,
    (field, component): (&str, &str),
    node: NodeId,
    sum: bool,
) -> f64 {
    let expanded = crate::frd::expanded_nodes(mesh, &frd.mesh);
    let indices = crate::frd::node_indices(&frd.mesh, &expanded, node);
    assert!(!indices.is_empty(), "no node {node} in the results");
    let increment = frd.increments.last().unwrap();
    let values = &increment
        .field(field)
        .unwrap()
        .component(component)
        .unwrap()
        .values;
    let total: f64 = indices.iter().map(|&i| f64::from(values[i])).sum();
    if sum {
        total
    } else {
        total / indices.len() as f64
    }
}

#[test]
fn calculix_reproduces_the_reference_cantilever() {
    let (mesh, model) = cantilever(tip_force());
    let Some(frd) = run_ccx("kragbalken", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let reference = read_frd(&testdata("kragbalken_c3d8.frd")).unwrap();
    let ours = node_value(&frd, "DISP", "U3", 99);
    let expected = node_value(&reference, "DISP", "U3", 99);
    assert!(
        (ours - expected).abs() < 1e-6 * expected.abs(),
        "{ours} != {expected}"
    );
}

/// A plate 100 x 10 of S8 shells, 2 thick, clamped at x = 0 and loaded by 1 N at its tip.
fn shell_plate() -> (FeMesh, FeModel) {
    let (length, width, thickness) = (100.0, 10.0, 2.0);
    let mesh = rectangle(0.0, length, width, 20, 2);
    let tip = nodes_at(&mesh, 0, length);
    assert_eq!(tip.len(), 5);
    // A force on points is shared by their nodes.
    let mut model = plane_model(
        ModelSpace::ThreeD,
        1.0,
        vec![(nodes_at(&mesh, 0, 0.0), [Some(0.0); 6])],
        LoadKind::force([0.0, 0.0, -1.0]),
        Region::Nodes(tip),
    );
    model.sections[0].kind = SectionKind::Shell {
        thickness,
        offset: 0.0,
    };
    (mesh, model)
}

/// A rectangle of linear shells in the x-y plane, `nx` by `ny` quads `S4` or, with
/// `triangles`, each quad split into two `S3`; one part PLATE.
fn linear_rectangle(width: f64, height: f64, nx: u32, ny: u32, triangles: bool) -> FeMesh {
    let mut mesh = FeMesh::default();
    let id = |i: u32, j: u32| j * (nx + 1) + i + 1;
    for j in 0..=ny {
        for i in 0..=nx {
            let (x, y) = (
                width * f64::from(i) / f64::from(nx),
                height * f64::from(j) / f64::from(ny),
            );
            mesh.set_node(id(i, j), [x, y, 0.0]);
        }
    }
    let mut part = plx_mesh::Part {
        name: "PLATE".into(),
        elements: Vec::new(),
    };
    let mut next = 1;
    for j in 0..ny {
        for i in 0..nx {
            let corners = [id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)];
            let elements: Vec<(&str, plx_mesh::ElementShape, Vec<NodeId>)> = if triangles {
                vec![
                    (
                        "S3",
                        plx_mesh::ElementShape::Tri3,
                        vec![corners[0], corners[1], corners[2]],
                    ),
                    (
                        "S3",
                        plx_mesh::ElementShape::Tri3,
                        vec![corners[0], corners[2], corners[3]],
                    ),
                ]
            } else {
                vec![("S4", plx_mesh::ElementShape::Quad4, corners.to_vec())]
            };
            for (type_name, shape, nodes) in elements {
                mesh.add_element(plx_mesh::Element {
                    id: next,
                    type_name: type_name.into(),
                    shape,
                    nodes,
                })
                .unwrap();
                part.elements.push(next);
                next += 1;
            }
        }
    }
    mesh.parts.push(part);
    mesh
}

/// A membrane strip of 100 x 10 x 2 in the x-y plane, held at x = 0 and pulled by a force
/// of 1 along x at x = 100. Membranes carry no lateral load, so all nodes are held in z.
fn membrane_strip(type_name: &str) -> (FeMesh, FeModel) {
    let (length, width) = (100.0, 10.0);
    let mesh = linear_rectangle(length, width, 20, 2, type_name == "M3D3");
    let tip = nodes_at(&mesh, 0, length);
    let mut model = plane_model(
        ModelSpace::ThreeD,
        1.0,
        vec![
            (nodes_at(&mesh, 0, 0.0), FIX_X),
            (vec![1], FIX_Y),
            (
                mesh.node_ids().to_vec(),
                [None, None, Some(0.0), None, None, None],
            ),
        ],
        LoadKind::force([1.0, 0.0, 0.0]),
        Region::Nodes(tip),
    );
    model.sections[0].kind = SectionKind::Membrane {
        thickness: 2.0,
        offset: 0.0,
    };
    model.set_element_type("PLATE", type_name);
    (mesh, model)
}

/// The cantilever with its tip face as a rigid body driven by reference point RP-1 at the
/// centre of the face, and the load or boundary condition given on the point.
fn rigid_tip(load: Option<Load>, bc: Option<BoundaryCondition>) -> (FeMesh, FeModel) {
    let (mesh, mut model) = cantilever(tip_force());
    model.steps[0].loads.clear();
    model.steps[0].loads.extend(load);
    model.steps[0].boundary_conditions.extend(bc);
    model
        .reference_points
        .push(ReferencePoint::new("RP-1", [100.0, 5.0, 5.0]));
    let mut body = RigidBody::new("Rigid_Body-1", "RP-1");
    body.region = Region::Surface("TIP".into());
    model.constraints.push(Constraint::RigidBody(body));
    (mesh, model)
}

fn moment(moment: [f64; 3]) -> Load {
    Load {
        distribution: None,
        name: "Moment-1".into(),
        active: true,
        visible: true,
        region: Region::ReferencePoint("RP-1".into()),
        kind: LoadKind::Moment(moment),
        amplitude: None,
        factor_amplitude: None,
    }
}

#[test]
fn a_rigid_body_and_its_reference_point_are_written_like_prepomax_does() {
    let bc = BoundaryCondition {
        distribution: None,
        name: "Turn-1".into(),
        active: true,
        visible: true,
        region: Region::ReferencePoint("RP-1".into()),
        kind: BoundaryKind::displacement([Some(0.0), Some(0.0), None, None, None, Some(0.01)]),
        amplitude: None,
    };
    let (mesh, model) = rigid_tip(Some(moment([0.0, 0.0, 1000.0])), Some(bc));
    let text = write_inp(&mesh, &model, "").unwrap();
    // The reference and rotation node follow the mesh nodes, both on the point.
    assert!(
        text.contains("99, 1.00000000E2, 1.00000000E1, 1.00000000E1\n100, 1.00000000E2, 5.00000000E0, 5.00000000E0\n101, 1.00000000E2, 5.00000000E0, 5.00000000E0\n"),
        "{text}"
    );
    assert!(
        text.contains("*Nset, Nset=Internal_Selection-1_RP-1\n100\n"),
        "{text}"
    );
    assert!(
        text.contains("*Rigid body, Nset=Internal-1_TIP, Ref node=100, Rot node=101\n"),
        "{text}"
    );
    // Translations on the reference node, rotations on the rotation node.
    assert!(
        text.contains("*Boundary\nInternal_Selection-1_RP-1, 1, 2, 0\n101, 3, 3, 0.01\n"),
        "{text}"
    );
    assert!(text.contains("*Cload\n101, 3, 1000\n"), "{text}");
}

/// A force on points is the total: the nodes share it equally, a reference point of a
/// rigid body takes it whole. Applied to all nodes, each node takes all of it.
#[test]
fn a_force_on_points_is_shared_by_the_nodes() {
    let mut on_nodes = tip_force();
    on_nodes.region = Region::Nodes(vec![96, 97, 98, 99]);
    let (mesh, model) = cantilever(on_nodes);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Cload\nInternal_Selection-1_Force-1, 3, -25\n"),
        "{text}"
    );
    // At each node, like PrePoMax's concentrated force, every node takes the whole force.
    let mut each = tip_force();
    each.region = Region::Nodes(vec![96, 97, 98, 99]);
    each.kind = LoadKind::Force {
        value: [0.0, 0.0, -100.0],
        each_node: true,
    };
    let (mesh, model) = cantilever(each);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Cload\nInternal_Selection-1_Force-1, 3, -100\n"),
        "{text}"
    );
    let mut on_point = tip_force();
    on_point.region = Region::ReferencePoint("RP-1".into());
    let (mesh, model) = rigid_tip(Some(on_point), None);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Cload\nInternal_Selection-1_RP-1, 3, -100\n"),
        "{text}"
    );
}

/// A moment at the reference point bends the beam: the reactions at the clamped end balance
/// it and the rigid tip face stays plane.
#[test]
fn calculix_bends_the_cantilever_by_a_moment_on_the_rigid_tip() {
    let m = 10_000.0;
    let (mesh, model) = rigid_tip(Some(moment([0.0, 0.0, m])), None);
    let Some(frd) = run_ccx("moment", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let reaction: f64 = (mesh.node_sets["FIX"].iter())
        .map(|&node| {
            let y = mesh.node(node).unwrap()[1];
            (y - 5.0) * node_value(&frd, "FORC", "F1", node)
        })
        .sum();
    assert!((reaction - m).abs() < 1e-3 * m, "{reaction} != {m}");
    let tip = |y: f64| -> Vec<f64> {
        (1..=99)
            .filter(|&id| mesh.node(id).is_some_and(|c| c[0] == 100.0 && c[1] == y))
            .map(|id| node_value(&frd, "DISP", "U1", id))
            .collect()
    };
    let (top, bottom, middle) = (tip(10.0), tip(0.0), tip(5.0));
    assert_eq!(top.len(), 3);
    let rotation = (bottom[0] - top[0]) / 10.0;
    assert!(rotation > 1e-4, "{rotation}");
    for (u_top, u_bottom) in top.iter().zip(&bottom) {
        assert!(
            (u_top + u_bottom).abs() < 1e-6 * rotation,
            "{u_top} {u_bottom}"
        );
    }
    assert!(
        middle.iter().all(|u| u.abs() < 1e-6 * rotation),
        "{middle:?}"
    );
}

/// A rotation prescribed at the reference point turns the rigid tip face about it.
#[test]
fn calculix_turns_the_rigid_tip_by_the_rotation_of_its_reference_point() {
    let angle = 0.01;
    let bc = BoundaryCondition {
        distribution: None,
        name: "Turn-1".into(),
        active: true,
        visible: true,
        region: Region::ReferencePoint("RP-1".into()),
        kind: BoundaryKind::displacement([None, None, None, None, None, Some(angle)]),
        amplitude: None,
    };
    let (mesh, model) = rigid_tip(None, Some(bc));
    let Some(frd) = run_ccx("drehung", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    for id in (1..=99).filter(|&id| mesh.node(id).is_some_and(|c| c[0] == 100.0)) {
        let y = mesh.node(id).unwrap()[1];
        let u1 = node_value(&frd, "DISP", "U1", id);
        let expected = -angle * (y - 5.0);
        assert!((u1 - expected).abs() < 1e-7, "{u1} != {expected}");
    }
}

/// A displacement held at every node.
fn each_node(values: [Option<f64>; 6]) -> BoundaryKind {
    BoundaryKind::Displacement(Displacement {
        behavior: Behavior::EachNode,
        ..Displacement::new(values)
    })
}

fn remote(name: &str, values: [Option<f64>; 6]) -> BoundaryCondition {
    BoundaryCondition {
        distribution: None,
        name: name.into(),
        active: true,
        visible: true,
        region: Region::Surface("TIP".into()),
        kind: BoundaryKind::Displacement(Displacement::rigid(values)),
        amplitude: None,
    }
}

/// A remote displacement on the tip face, changed by `edit`.
fn remote_with(
    values: [Option<f64>; 6],
    edit: impl FnOnce(&mut Displacement),
) -> BoundaryCondition {
    let mut bc = remote("Remote-1", values);
    if let BoundaryKind::Displacement(r) = &mut bc.kind {
        edit(r);
    }
    bc
}

/// The cantilever without its load, with the boundary condition at the tip.
fn remote_cantilever(bc: BoundaryCondition) -> (FeMesh, FeModel) {
    let (mesh, mut model) = cantilever(tip_force());
    model.steps[0].loads.clear();
    model.steps[0].boundary_conditions.push(bc);
    (mesh, model)
}

/// A system turned by 90 degrees about z: its x axis is the global y axis.
fn turned_system(model: &mut FeModel) {
    let mut cs = plx_model::CoordinateSystem::new("Turned");
    cs.point_x = [0.0, 1.0, 0.0];
    cs.point_xy = [-1.0, 0.0, 0.0];
    model.coordinate_systems.push(cs);
}

/// The weighted mean rotation of the nodes of a deformable remote point about axis k,
/// J^-1 sum w r x u restricted to axis k when J is diagonal, as the equations define it.
fn mean_rotation(mesh: &FeMesh, model: &FeModel, frd: &FrdImport, k: usize) -> f64 {
    let bc = &model.steps[0].boundary_conditions[1];
    let BoundaryKind::Displacement(remote) = &bc.kind else {
        unreachable!()
    };
    let point = remote
        .resolve(model, mesh, &bc.region, &Default::default())
        .unwrap();
    let total: f64 = point.weights.iter().sum();
    let x: Vec<[f64; 3]> = point.nodes.iter().map(|&n| mesh.node(n).unwrap()).collect();
    let c: [f64; 3] = std::array::from_fn(|i| {
        (x.iter().zip(&point.weights))
            .map(|(x, w)| w * x[i])
            .sum::<f64>()
            / total
    });
    let (a, b) = ((k + 1) % 3, (k + 2) % 3);
    let (mut moment, mut inertia) = (0.0, 0.0);
    for ((&node, x), w) in point.nodes.iter().zip(&x).zip(&point.weights) {
        let u = ["U1", "U2", "U3"].map(|c| node_value(frd, "DISP", c, node));
        let r: [f64; 3] = std::array::from_fn(|i| x[i] - c[i]);
        moment += w * (r[a] * u[b] - r[b] * u[a]);
        inertia += w * (r[a] * r[a] + r[b] * r[b]);
    }
    moment / inertia
}

#[test]
fn calculix_turns_a_deformable_remote_point_with_the_mean_of_its_face() {
    let angle = 0.01;
    let values = [Some(0.0), Some(0.0), Some(0.0), None, None, Some(angle)];
    let (mesh, model) = remote_cantilever(remote_with(values, |r| {
        r.behavior = Behavior::Deformable;
    }));
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(!text.contains("*Rigid body"), "{text}");
    assert_eq!(text.matches("*Equation").count(), 6, "{text}");
    let Some(frd) = run_ccx("remote-deformable", &text) else {
        return;
    };
    let turned = mean_rotation(&mesh, &model, &frd, 2);
    assert!((turned - angle).abs() < 1e-6 * angle, "{turned} != {angle}");
}

/// A solid beam on two end faces held in their translations with free rotations is simply
/// supported: the faces turn about their centres. Held at each node, the same supports clamp
/// the ends nearly.
#[test]
fn calculix_supports_a_solid_beam_on_two_hinged_faces() {
    // 100 x 10 x 10, 1000 N at midspan: F L^3 / (48 E I) plus shear F L / (4 k G A).
    let force = Load {
        region: Region::Nodes(vec![477, 508, 539, 570, 601]),
        kind: LoadKind::force([0.0, 0.0, -1000.0]),
        ..tip_force()
    };
    let (mesh, mut model) = analysis("kragbalken_c3d20r.inp", force);
    let bending = 1000.0 * 100f64.powi(3) / (48.0 * 210_000.0 * 1e4 / 12.0);
    let shear = 1000.0 * 100.0 / (4.0 * 5.0 / 6.0 * 210_000.0 / 2.6 * 100.0);
    let expected = bending + shear;
    let step = &mut model.steps[0];
    // The left face also stops the twist about the beam axis.
    step.boundary_conditions[0].kind =
        BoundaryKind::displacement([Some(0.0), Some(0.0), Some(0.0), Some(0.0), None, None]);
    step.boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Roller-1".into(),
        active: true,
        visible: true,
        region: Region::NodeSet("TIP".into()),
        kind: BoundaryKind::displacement([None, Some(0.0), Some(0.0), None, None, None]),
        amplitude: None,
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    assert_eq!(text.matches("*Rigid body").count(), 2, "{text}");
    let Some(frd) = run_ccx("hinged-beam", &text) else {
        return;
    };
    let sag = -node_value(&frd, "DISP", "U3", 539);
    assert!(
        (sag - expected).abs() < 0.02 * expected,
        "{sag} != {expected}"
    );
    for bc in &mut model.steps[0].boundary_conditions {
        if let BoundaryKind::Displacement(d) = &mut bc.kind {
            d.behavior = Behavior::EachNode;
        }
    }
    let text = write_inp(&mesh, &model, "").unwrap();
    let frd = run_ccx("clamped-beam", &text).unwrap();
    let sag = -node_value(&frd, "DISP", "U3", 539);
    assert!(sag < 0.6 * expected, "{sag} not stiffer than {expected}");
}

/// Each node held in a coordinate system of its own: a `*Transform` of the nodes.
#[test]
fn calculix_holds_each_node_in_its_coordinate_system() {
    let shift = 0.1;
    let values = [Some(shift), Some(0.0), Some(0.0), None, None, None];
    let (mesh, mut model) = remote_cantilever(remote_with(values, |r| {
        r.behavior = Behavior::EachNode;
        r.system = "Turned".into();
    }));
    turned_system(&mut model);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(!text.contains("*Equation"), "{text}");
    assert!(!text.contains("*Rigid body"), "{text}");
    assert!(
        text.contains("*Transform, Nset=Internal-1_TIP, Type=R\n0, 1, 0, -1, 0, 0\n"),
        "{text}"
    );
    let Some(frd) = run_ccx("each-node-system", &text) else {
        return;
    };
    for id in [11, 22, 33, 44, 55, 66, 77, 88, 99] {
        let u = ["U1", "U2", "U3"].map(|c| node_value(&frd, "DISP", c, id));
        assert!(
            u[0].abs() < 1e-6 && (u[1] - shift).abs() < 1e-6 && u[2].abs() < 1e-6,
            "{id}: {u:?}"
        );
    }
}

/// A face of solids held in its translations and free to turn becomes a hinge about its
/// centre; held in all directions it is clamped node by node. Nodes that another boundary
/// condition holds directly stay out of the point.
#[test]
fn a_face_free_to_turn_follows_a_rigid_point() {
    let hinge = [Some(0.0), Some(0.0), Some(0.0), None, None, None];
    let (mesh, mut model) = remote_cantilever(remote("Hinge-1", hinge));
    if let BoundaryKind::Displacement(d) = &mut model.steps[0].boundary_conditions[1].kind {
        d.behavior = Behavior::Automatic;
    }
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Rigid body, Nset=Internal-1_TIP,"), "{text}");
    assert!(
        text.contains("** Name: Hinge-1\n*Boundary\n100, 1, 3, 0\n**"),
        "{text}"
    );
    // Clamped: each node.
    model.steps[0].boundary_conditions[1].kind = BoundaryKind::displacement([Some(0.0); 6]);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(!text.contains("*Rigid body"), "{text}");
    assert!(
        text.contains("** Name: Hinge-1\n*Boundary\nInternal-1_TIP, 1, 6, 0\n"),
        "{text}"
    );
    // A node of the face held by another boundary condition leaves the point.
    model.steps[0].boundary_conditions[1].kind = BoundaryKind::displacement(hinge);
    let node = Region::Surface("TIP".into()).nodes(&mesh)[0];
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Pin-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(vec![node]),
        kind: BoundaryKind::displacement([None, Some(0.0), None, None, None, None]),
        amplitude: None,
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Rigid body, Nset=Internal_Selection"),
        "{text}"
    );
    let Some(_) = run_ccx("hinge-and-pin", &text) else {
        return;
    };
}

/// In a coordinate system turned about z, U1 is the global y direction and UR1 a rotation
/// about the global y axis, for rigid and deformable points alike.
#[test]
fn calculix_moves_a_remote_point_in_its_coordinate_system() {
    let (mesh, mut model) = remote_cantilever(remote_with(
        [
            Some(0.1),
            Some(0.0),
            Some(0.0),
            Some(0.0),
            Some(0.0),
            Some(0.0),
        ],
        |r| r.system = "Turned".into(),
    ));
    turned_system(&mut model);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Transform, Nset=Internal_Selection-1_Remote-1_Point, Type=R\n"),
        "{text}"
    );
    let Some(frd) = run_ccx("remote-system", &text) else {
        return;
    };
    for id in [11, 55, 99] {
        let u = ["U1", "U2", "U3"].map(|c| node_value(&frd, "DISP", c, id));
        assert!(
            u[0].abs() < 1e-6 && (u[1] - 0.1).abs() < 1e-6 && u[2].abs() < 1e-6,
            "{u:?}"
        );
    }
    let angle = 0.01;
    let bc = &mut model.steps[0].boundary_conditions[1];
    if let BoundaryKind::Displacement(r) = &mut bc.kind {
        r.behavior = Behavior::Deformable;
        r.values = [
            Some(0.0),
            Some(0.0),
            Some(0.0),
            Some(angle),
            Some(0.0),
            Some(0.0),
        ];
    }
    let text = write_inp(&mesh, &model, "").unwrap();
    let Some(frd) = run_ccx("remote-system-deformable", &text) else {
        return;
    };
    let turned = mean_rotation(&mesh, &model, &frd, 1);
    assert!((turned - angle).abs() < 1e-6 * angle, "{turned} != {angle}");
}

/// A point at the lower edge of the tip face: the pinball region takes the four nodes
/// within 6 of it, three on the edge and the middle node of the face.
#[test]
fn a_remote_point_takes_its_location_and_pinball_region() {
    let (mesh, model) = remote_cantilever(remote_with([Some(0.0); 6], |r| {
        r.location = Some([100.0, 5.0, 0.0]);
        r.pinball = Some(6.0);
    }));
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("100, 1.00000000E2, 5.00000000E0, 0.00000000E0\n"),
        "{text}"
    );
    assert!(
        text.contains("*Nset, Nset=Internal_Selection-1_Remote-1\n11, 22, 33, 55\n"),
        "{text}"
    );
    // A pinball region without nodes cannot be written.
    let (mesh, model) = remote_cantilever(remote_with([Some(0.0); 6], |r| {
        r.location = Some([0.0; 3]);
        r.pinball = Some(1.0);
    }));
    assert!(matches!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::InvalidBoundary { .. })
    ));
}

#[test]
fn a_remote_displacement_drives_its_region_from_a_point_at_its_centre() {
    let (mesh, mut model) = cantilever(tip_force());
    model.steps[0].loads.clear();
    let values = [Some(0.0), Some(0.0), None, None, None, Some(0.01)];
    (model.steps[0].boundary_conditions).push(remote("Remote-1", values));
    // A second step on the same region shares the point and its rigid body.
    let mut second = model.steps[0].clone();
    second.name = "Step-2".into();
    second.boundary_conditions[1].name = "Remote-2".into();
    model.steps.push(second);
    let text = write_inp(&mesh, &model, "").unwrap();
    // Reference and rotation node at the centre of the tip face.
    assert!(
        text.contains(
            "100, 1.00000000E2, 5.00000000E0, 5.00000000E0
101, 1.00000000E2, 5.00000000E0, 5.00000000E0
"
        ),
        "{text}"
    );
    assert_eq!(text.matches("*Rigid body").count(), 1, "{text}");
    assert!(
        text.contains(
            "** Name: Remote-1: Remote point\n*Rigid body, Nset=Internal-1_TIP, Ref node=100, Rot node=101\n"
        ),
        "{text}"
    );
    for name in ["Remote-1", "Remote-2"] {
        let expected = format!("** Name: {name}\n*Boundary\n100, 1, 2, 0\n101, 3, 3, 0.01\n");
        assert!(text.contains(&expected), "{expected}\n{text}");
    }
    // Without nodes there is no centre.
    model.steps[0].boundary_conditions[1].region = Region::Nodes(Vec::new());
    assert!(matches!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::InvalidBoundary { .. })
    ));
}

/// The region of a remote displacement moves rigidly with its point: turned about the
/// centre of the tip face and shifted along y.
#[test]
fn calculix_moves_the_region_of_a_remote_displacement_rigidly() {
    let (angle, shift) = (0.01, 0.1);
    let (mesh, mut model) = cantilever(tip_force());
    model.steps[0].loads.clear();
    let values = [None, Some(shift), None, None, None, Some(angle)];
    (model.steps[0].boundary_conditions).push(remote("Remote-1", values));
    let Some(frd) = run_ccx("remote", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let tip: Vec<_> = (1..=99)
        .filter(|&id| mesh.node(id).is_some_and(|c| c[0] == 100.0))
        .collect();
    assert_eq!(tip.len(), 9);
    for id in tip {
        let y = mesh.node(id).unwrap()[1];
        let u1 = node_value(&frd, "DISP", "U1", id);
        let u2 = node_value(&frd, "DISP", "U2", id);
        let expected = -angle * (y - 5.0);
        assert!((u1 - expected).abs() < 1e-7, "{u1} != {expected}");
        assert!((u2 - shift).abs() < 1e-7, "{u2} != {shift}");
    }
}

/// Steel S235 with a linear hardening to 400 MPa at 20 % plastic strain.
fn s235(mesh_model: &mut FeModel) {
    mesh_model.materials[0].plastic = Some(Plastic {
        hardening: Hardening::Isotropic,
        points: vec![
            PlasticPoint {
                stress: 235.0,
                plastic_strain: 0.0,
                temperature: 0.0,
            },
            PlasticPoint {
                stress: 400.0,
                plastic_strain: 0.2,
                temperature: 0.0,
            },
        ],
    });
}

#[test]
fn a_plastic_material_is_written_like_prepomax_does() {
    let (mesh, mut model) = cantilever(tip_force());
    s235(&mut model);
    model.materials[0].plastic.as_mut().unwrap().hardening = Hardening::Kinematic;
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains(
            "*Elastic\n210000, 0.3\n*Plastic, Hardening=Kinematic\n235, 0, 0\n400, 0.2, 0\n"
        ),
        "{text}"
    );
}

/// A bar pulled beyond its yield stress: the stress stays on the hardening curve and the
/// equivalent plastic strain is where the curve reaches the applied stress.
#[test]
fn calculix_yields_the_bar_pulled_beyond_its_yield_stress() {
    let stress = 300.0;
    let load = Load {
        distribution: None,
        name: "Pressure-1".into(),
        active: true,
        visible: true,
        region: Region::Surface("TIP".into()),
        kind: LoadKind::Pressure(-stress),
        amplitude: None,
        factor_amplitude: None,
    };
    let (mesh, mut model) = cantilever(load);
    s235(&mut model);
    // Symmetry supports instead of the clamp: a clamped end restrains the lateral
    // contraction, and with the small plastic tangent modulus that reaches along the
    // whole bar; held this way the bar is in uniaxial tension.
    let (min, max) = mesh.bounds().unwrap();
    let nodes_at = |axis: usize, value: f64| -> Vec<NodeId> {
        (1..=99)
            .filter(|&id| mesh.node(id).is_some_and(|c| c[axis] == value))
            .collect()
    };
    let hold = |axis: usize| {
        let mut dofs = [None; 6];
        dofs[axis] = Some(0.0);
        BoundaryCondition {
            distribution: None,
            name: format!("Symmetry-{}", axis + 1),
            active: true,
            visible: true,
            region: Region::Nodes(nodes_at(axis, min[axis])),
            kind: each_node(dofs),
            amplitude: None,
        }
    };
    model.steps[0].boundary_conditions = vec![hold(0), hold(1), hold(2)];
    model.steps[0].field_outputs[1]
        .variables
        .push("PEEQ".into());
    let Some(frd) = run_ccx("plastisch", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let nodes = nodes_at(0, max[0]);
    assert_eq!(nodes.len(), 9);
    let expected_strain = (stress - 235.0) / (400.0 - 235.0) * 0.2;
    for node in nodes {
        let s = node_value(&frd, "STRESS", "S11", node);
        assert!((s - stress).abs() < 0.02 * stress, "{s} != {stress}");
        let peeq = node_value(&frd, "PE", "PE", node);
        assert!(
            (peeq - expected_strain).abs() < 0.05 * expected_strain,
            "{peeq} != {expected_strain}"
        );
    }
}

/// The element faces at x = 50 of the elements left of the cut: a pre-tension section
/// through the middle of the bar.
fn cut_faces(mesh: &FeMesh) -> Vec<(ElementId, u8)> {
    (mesh.elements().iter())
        .flat_map(|e| {
            (1..=6u8).filter_map(move |f| {
                let corners = e.shape.faces()[usize::from(f) - 1].corners;
                let at_cut = (corners.iter()).all(|&l| mesh.node(e.nodes[l]).unwrap()[0] == 50.0);
                let left = (e.nodes.iter()).all(|&n| mesh.node(n).unwrap()[0] <= 50.0);
                (at_cut && left).then_some((e.id, f))
            })
        })
        .collect()
}

fn pre_tension(value: f64, by_displacement: bool) -> (FeMesh, FeModel) {
    let (mesh, mut model) = cantilever(tip_force());
    let faces = cut_faces(&mesh);
    assert_eq!(faces.len(), 4);
    model.steps[0].loads = vec![Load {
        distribution: None,
        name: "Pre_Tension-1".into(),
        active: true,
        visible: true,
        region: Region::Faces(faces),
        kind: LoadKind::PreTension {
            value,
            by_displacement,
            lock: false,
            direction: None,
        },
        amplitude: None,
        factor_amplitude: None,
    }];
    // Both ends held: the bolt is clamped between them.
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Fixed-2".into(),
        active: true,
        visible: true,
        region: Region::Surface("TIP".into()),
        kind: BoundaryKind::displacement([Some(0.0); 6]),
        amplitude: None,
    });
    (mesh, model)
}

#[test]
fn a_shell_section_is_written_like_prepomax_does() {
    let (mesh, mut model) = shell_plate();
    model.sections[0].kind = SectionKind::Shell {
        thickness: 2.0,
        offset: 0.5,
    };
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Element, Type=S8, Elset=PLATE\n"), "{text}");
    assert!(
        text.contains(
            "*Shell section, Elset=Internal_Selection-1_Section-1, Material=Steel, Offset=0.5\n2\n"
        ),
        "{text}"
    );
    // Results on the solids CalculiX expands the shells to: at the shell nodes (2D) the
    // stresses would lack the bending.
    assert!(text.contains("*Node file\nRF, U\n"), "{text}");
    assert!(text.contains("*El file\nS, E, NOE\n"), "{text}");
    // Asked for 2D, the node outputs follow the element outputs.
    let mut two_d = model.clone();
    for output in &mut two_d.steps[0].field_outputs {
        if output.kind == OutputKind::Element {
            output.dimension = OutputDimension::TwoD;
        }
    }
    let text = write_inp(&mesh, &two_d, "").unwrap();
    assert!(text.contains("*Node file, Output=2D\nRF, U\n"), "{text}");
    assert!(text.contains("*El file, Output=2D\nS, E, NOE\n"), "{text}");
    // A shell section on solid elements is refused before CalculiX would be.
    let (solid_mesh, mut solid) = cantilever(tip_force());
    solid.sections[0].kind = SectionKind::Shell {
        thickness: 2.0,
        offset: 0.0,
    };
    assert!(matches!(
        write_inp(&solid_mesh, &solid, ""),
        Err(WriteError::InvalidSection { .. })
    ));
}

#[test]
fn a_membrane_section_is_written_for_membrane_elements() {
    let (mesh, mut model) = membrane_strip("M3D4R");
    model.sections[0].kind = SectionKind::Membrane {
        thickness: 2.0,
        offset: 0.5,
    };
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Element, Type=M3D4R, Elset=PLATE\n"),
        "{text}"
    );
    assert!(
        text.contains(
            "*Membrane section, Elset=Internal_Selection-1_Section-1, Material=Steel, \
             Offset=0.5\n2\n"
        ),
        "{text}"
    );
    // Results on the expanded solids, as for shells.
    assert!(text.contains("*Node file\nRF, U\n"), "{text}");
    // Linear shells under a membrane section are written as membranes without choosing an
    // element type; the mesh keeps its shells.
    model.element_types.clear();
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Element, Type=M3D4, Elset=PLATE\n"),
        "{text}"
    );
    assert!(mesh.elements().iter().all(|e| e.type_name == "S4"));
    // Quadratic shells under a membrane section, and membranes under a shell or solid
    // section, are refused before CalculiX would be.
    let (quadratic, mut plate) = shell_plate();
    plate.sections[0].kind = model.sections[0].kind.clone();
    assert!(matches!(
        write_inp(&quadratic, &plate, ""),
        Err(WriteError::InvalidSection { .. })
    ));
    for kind in [
        SectionKind::Solid,
        SectionKind::Shell {
            thickness: 2.0,
            offset: 0.0,
        },
    ] {
        let (mesh, mut model) = membrane_strip("M3D4");
        model.sections[0].kind = kind;
        assert!(matches!(
            write_inp(&mesh, &model, ""),
            Err(WriteError::InvalidSection { .. })
        ));
    }
}

/// A membrane strip under tension stretches by F L / (E A) at mid-length, away from the
/// unevenly loaded tip, with each membrane element type a part can have.
#[test]
fn calculix_stretches_a_membrane_strip() {
    // S4R is written as M3D4R.
    for (chosen, type_name) in [
        ("M3D3", "M3D3"),
        ("M3D4", "M3D4"),
        ("M3D4R", "M3D4R"),
        ("S4R", "M3D4R"),
    ] {
        let (mesh, model) = membrane_strip(chosen);
        let text = write_inp(&mesh, &model, "").unwrap();
        assert!(text.contains(&format!("Type={type_name},")), "{text}");
        let name = format!("membran-{}", chosen.to_ascii_lowercase());
        let Some(frd) = run_ccx(&name, &text) else {
            return;
        };
        let expected = 50.0 / (210_000.0 * 10.0 * 2.0);
        for node in nodes_at(&mesh, 0, 50.0) {
            let u = node_value(&frd, "DISP", "U1", node);
            assert!(
                (u - expected).abs() < 0.01 * expected,
                "{type_name}: {u} != {expected}"
            );
        }
    }
}

/// A membrane has no bending stiffness: stretched first, it carries a lateral pressure like
/// a taut string, sagging by p L^2 / (8 N) at mid-span with the tension N per width, in a
/// geometrically nonlinear analysis.
#[test]
fn calculix_carries_pressure_on_a_pretensioned_membrane() {
    let (length, width, thickness) = (100.0, 10.0, 2.0);
    let mesh = linear_rectangle(length, width, 20, 2, false);
    let (start, end) = (nodes_at(&mesh, 0, 0.0), nodes_at(&mesh, 0, length));
    let stretch = 0.1;
    let pressure = 0.01;
    // Held across the strip everywhere, so it is in plane strain along y.
    let hold_y = (
        mesh.node_ids().to_vec(),
        [None, Some(0.0), None, None, None, None],
    );
    let mut model = plane_model(
        ModelSpace::ThreeD,
        1.0,
        vec![
            (
                start.clone(),
                [Some(0.0), None, Some(0.0), None, None, None],
            ),
            (
                end.clone(),
                [Some(stretch), None, Some(0.0), None, None, None],
            ),
            hold_y,
        ],
        LoadKind::Pressure(pressure),
        Region::Faces(mesh.elements().iter().map(|e| (e.id, 1)).collect()),
    );
    model.sections[0].kind = SectionKind::Membrane {
        thickness,
        offset: 0.0,
    };
    model.set_element_type("PLATE", "M3D4");
    // Step 1 stretches the flat strip, which has no lateral stiffness yet; step 2 adds the
    // pressure.
    let mut stretch_step = model.steps[0].clone();
    stretch_step.name = "Step-0".into();
    stretch_step.loads.clear();
    for step in [&mut stretch_step, &mut model.steps[0]] {
        if let StepKind::Static(s) = &mut step.kind {
            s.nlgeom = true;
        }
    }
    model.steps.insert(0, stretch_step);
    let text = write_inp(&mesh, &model, "").unwrap();
    let Some(frd) = run_ccx("membran-vorgespannt", &text) else {
        return;
    };
    let (e, nu) = (210_000.0, 0.3);
    let tension = e / (1.0 - nu * nu) * stretch / length * thickness;
    let expected = pressure * length * length / (8.0 * tension);
    let middle = nodes_at(&mesh, 0, length / 2.0);
    let sag: f64 = middle
        .iter()
        .map(|&n| node_value(&frd, "DISP", "U3", n))
        .sum::<f64>()
        / middle.len() as f64;
    assert!(
        (sag.abs() - expected).abs() < 0.05 * expected,
        "{sag} != {expected}"
    );
}

/// The shell plate bends like a beam: F L^3 / (3 E I) at the tip.
#[test]
fn calculix_bends_the_shell_plate_like_a_beam() {
    let (mesh, model) = shell_plate();
    let Some(frd) = run_ccx("schale", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let inertia = 10.0 * 2.0f64.powi(3) / 12.0;
    let expected = -100.0f64.powi(3) / (3.0 * 210_000.0 * inertia);
    let tip = nodes_at(&mesh, 0, 100.0);
    let deflection: f64 = tip
        .iter()
        .map(|&n| expanded_value(&frd, &mesh, ("DISP", "U3"), n, false))
        .sum::<f64>()
        / tip.len() as f64;
    assert!(
        (deflection - expected).abs() < 0.03 * expected.abs(),
        "{deflection} != {expected}"
    );
}

/// The tip edge of the shell plate as segments between its corner nodes, ordered along y.
fn shell_plate_tip_edge(mesh: &FeMesh) -> Vec<[NodeId; 2]> {
    let mut tip = nodes_at(mesh, 0, 100.0);
    tip.sort_by(|&a, &b| mesh.node(a).unwrap()[1].total_cmp(&mesh.node(b).unwrap()[1]));
    // Corners at y = 0, 5 and 10, midside nodes between them.
    vec![[tip[0], tip[2]], [tip[2], tip[4]]]
}

#[test]
fn surface_traction_on_an_edge_spreads_the_total_force_by_length() {
    let (mesh, _) = shell_plate();
    let edges = Region::Edges(shell_plate_tip_edge(&mesh)).edges(&mesh);
    assert_eq!(edges.len(), 2);
    assert!(
        edges.iter().all(|e| e.len() == 3),
        "quadratic edges keep their midside node"
    );
    let nodal = edge_traction_forces(&mesh, &edges, [0.0, 0.0, -12.0], false);
    let mut tip = nodes_at(&mesh, 0, 100.0);
    tip.sort_by(|&a, &b| mesh.node(a).unwrap()[1].total_cmp(&mesh.node(b).unwrap()[1]));
    // Two quadratic edges of equal length: 1/6, 4/6, 1/6 of each half.
    let expected = [-1.0, -4.0, -2.0, -4.0, -1.0];
    for (node, f) in tip.iter().zip(expected) {
        assert!(
            (nodal[node][2] - f).abs() < 1e-9,
            "{node}: {:?}",
            nodal[node]
        );
    }
    assert_eq!(nodal.len(), 5);
}

/// A total force on the tip edge bends the plate like the same force at the tip nodes.
#[test]
fn calculix_bends_the_shell_plate_by_a_traction_on_its_edge() {
    let (mesh, mut model) = shell_plate();
    let load = &mut model.steps[0].loads[0];
    load.kind = LoadKind::force([0.0, 0.0, -1.0]);
    load.region = Region::Edges(shell_plate_tip_edge(&mesh));
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Cload"), "{text}");
    let Some(frd) = run_ccx("schalenkante", &text) else {
        return;
    };
    let reaction: f64 = (nodes_at(&mesh, 0, 0.0).iter())
        .map(|&n| expanded_value(&frd, &mesh, ("FORC", "F3"), n, true))
        .sum();
    assert!((reaction - 1.0).abs() < 1e-4, "{reaction}");
    let inertia = 10.0 * 2.0f64.powi(3) / 12.0;
    let expected = -100.0f64.powi(3) / (3.0 * 210_000.0 * inertia);
    let tip = nodes_at(&mesh, 0, 100.0);
    let deflection: f64 = (tip.iter())
        .map(|&n| expanded_value(&frd, &mesh, ("DISP", "U3"), n, false))
        .sum::<f64>()
        / tip.len() as f64;
    assert!(
        (deflection - expected).abs() < 0.03 * expected.abs(),
        "{deflection} != {expected}"
    );
}

#[test]
fn a_pre_tension_load_cuts_the_mesh_and_ties_the_sides_by_equations() {
    let (mesh, mut model) = pre_tension(1000.0, false);
    // A second step keeps the bolt at its length instead of the force.
    let mut second = model.steps[0].clone();
    second.name = "Step-2".into();
    second.loads[0].kind = LoadKind::PreTension {
        value: 0.0,
        by_displacement: false,
        lock: true,
        direction: Some([1.0, 0.0, 0.0]),
    };
    model.steps.push(second);
    let text = write_inp(&mesh, &model, "").unwrap();
    // The 9 nodes of the cut get copies 100 to 108, the pre-tension node is 109.
    assert!(!text.contains("*Pre-tension section"), "{text}");
    assert_eq!(text.matches("** Name: Pre_Tension-1\n*Equation").count(), 1);
    assert_eq!(text.matches("*Equation\n").count(), 27);
    assert!(text.contains("*Equation\n3\n100, 1, 1, "), "{text}");
    assert_eq!(text.matches(", 109, 1, -1\n").count(), 9);
    assert!(text.contains("108, 5.00000000E1, "), "{text}");
    assert!(text.contains("*Cload\n109, 1, 1000\n"), "{text}");
    assert!(text.contains("*Boundary, Fixed\n109, 1, 1\n"), "{text}");
}

/// A bar 100 x 10 x 10 along x of C3D10 tetrahedra, six per cube of 10 x 5 x 5, with node set
/// FIX at x = 0 and TIP at x = 100, in part EALL.
fn tet_bar() -> FeMesh {
    let (nx, ny) = (10i64, 2i64);
    let (dx, dy) = (10.0, 5.0);
    let mut mesh = FeMesh::default();
    // Nodes on the half grid, numbered by position.
    let id = |i: i64, j: i64, k: i64| (((i * (2 * ny + 1) + j) * (2 * ny + 1) + k) + 1) as u32;
    let mut used = std::collections::BTreeSet::new();
    let mut part = plx_mesh::Part {
        name: "EALL".into(),
        elements: Vec::new(),
    };
    // Kuhn's six tetrahedra around the diagonal 0-6 of a cube, the same split in every cube.
    const TETS: [[usize; 4]; 6] = [
        [0, 1, 2, 6],
        [0, 2, 3, 6],
        [0, 3, 7, 6],
        [0, 7, 4, 6],
        [0, 4, 5, 6],
        [0, 5, 1, 6],
    ];
    let mut element = 0;
    for i in 0..nx {
        for j in 0..ny {
            for k in 0..ny {
                let c = [
                    (i, j, k),
                    (i + 1, j, k),
                    (i + 1, j + 1, k),
                    (i, j + 1, k),
                    (i, j, k + 1),
                    (i + 1, j, k + 1),
                    (i + 1, j + 1, k + 1),
                    (i, j + 1, k + 1),
                ]
                .map(|(a, b, c)| (2 * a, 2 * b, 2 * c));
                for tet in TETS {
                    let mut p = tet.map(|l| c[l]);
                    let x = |q: (i64, i64, i64)| {
                        [
                            q.0 as f64 * dx / 2.0,
                            q.1 as f64 * dy / 2.0,
                            q.2 as f64 * dy / 2.0,
                        ]
                    };
                    let [a, b, cc, d] = p.map(x);
                    let e = |u: [f64; 3], v: [f64; 3]| [v[0] - u[0], v[1] - u[1], v[2] - u[2]];
                    let (u, v, w) = (e(a, b), e(a, cc), e(a, d));
                    let volume = (u[1] * v[2] - u[2] * v[1]) * w[0]
                        + (u[2] * v[0] - u[0] * v[2]) * w[1]
                        + (u[0] * v[1] - u[1] * v[0]) * w[2];
                    if volume < 0.0 {
                        p.swap(1, 2);
                    }
                    let mid = |m: usize, n: usize| {
                        (
                            (p[m].0 + p[n].0) / 2,
                            (p[m].1 + p[n].1) / 2,
                            (p[m].2 + p[n].2) / 2,
                        )
                    };
                    let all = [
                        p[0],
                        p[1],
                        p[2],
                        p[3],
                        mid(0, 1),
                        mid(1, 2),
                        mid(2, 0),
                        mid(0, 3),
                        mid(1, 3),
                        mid(2, 3),
                    ];
                    for q in all {
                        if used.insert(q) {
                            mesh.set_node(id(q.0, q.1, q.2), x(q));
                        }
                    }
                    element += 1;
                    mesh.add_element(plx_mesh::Element {
                        id: element,
                        type_name: "C3D10".into(),
                        shape: plx_mesh::ElementShape::Tet10,
                        nodes: all.iter().map(|q| id(q.0, q.1, q.2)).collect(),
                    })
                    .unwrap();
                    part.elements.push(element);
                }
            }
        }
    }
    mesh.parts.push(part);
    for (name, x) in [("FIX", 0.0), ("TIP", 100.0)] {
        let ids = (mesh.node_ids().iter().copied())
            .filter(|&n| mesh.node(n).unwrap()[0] == x)
            .collect();
        mesh.node_sets.insert(name.into(), ids);
    }
    mesh
}

/// The faces at x = 50 of the tetrahedra left of the cut.
fn tet_cut_faces(mesh: &FeMesh) -> Vec<(ElementId, u8)> {
    (mesh.elements().iter())
        .flat_map(|e| {
            (1..=4u8).filter_map(move |f| {
                let corners = e.shape.faces()[usize::from(f) - 1].corners;
                let at_cut = (corners.iter()).all(|&l| mesh.node(e.nodes[l]).unwrap()[0] == 50.0);
                let left = (e.nodes.iter()).all(|&n| mesh.node(n).unwrap()[0] <= 50.0);
                (at_cut && left).then_some((e.id, f))
            })
        })
        .collect()
}

/// CalculiX's own `*PRE-TENSION SECTION` leaves the tetrahedra that touch the cut with only
/// an edge or a corner on both sides, and most of the preload goes through them (PrePoMax
/// users add a thin layer of wedge elements for this). The cut through the mesh carries all
/// of it, also across a cut whose normal is not along an axis of the bar.
#[test]
fn calculix_preloads_a_tetrahedral_bar_across_the_cut() {
    let force = 1000.0;
    for (name, direction) in [
        ("tet-vorspannung", None),
        ("tet-schraeg", Some([1.0, 0.2, 0.0])),
    ] {
        let mesh = tet_bar();
        let faces = tet_cut_faces(&mesh);
        assert_eq!(faces.len(), 4 * 2);
        let (_, mut model) = pre_tension(force, false);
        model.steps[0].loads[0].region = Region::Faces(faces);
        model.steps[0].loads[0].kind = LoadKind::PreTension {
            value: force,
            by_displacement: false,
            lock: false,
            direction,
        };
        model.steps[0].boundary_conditions[1].region = Region::NodeSet("TIP".into());
        let text = write_inp(&mesh, &model, "").unwrap();
        let Some(frd) = run_ccx(name, &text) else {
            return;
        };
        let reaction: f64 = (mesh.node_sets["FIX"].iter())
            .map(|&node| node_value(&frd, "FORC", "F1", node))
            .sum();
        let expected = match direction {
            // The preload opens the cut along the given direction while the sides cannot
            // slide: the work of the force on the gap is done by the bar's force along x.
            Some([x, y, z]) => force * f64::hypot(f64::hypot(x, y), z) / x,
            None => force,
        };
        assert!(
            (reaction.abs() - expected).abs() < 0.02 * expected,
            "{name}: {reaction}"
        );
        if direction.is_none() {
            for id in (mesh.node_ids().iter().copied())
                .filter(|&id| (20.0..=80.0).contains(&mesh.node(id).unwrap()[0]))
            {
                let s = node_value(&frd, "STRESS", "S11", id);
                assert!((s - 10.0).abs() < 0.5, "node {id}: {s}");
            }
        }
    }
}

/// A preload across the middle of the clamped bar puts the whole bar under that tension.
#[test]
fn calculix_preloads_the_clamped_bar_across_the_cut() {
    let force = 1000.0;
    let (mesh, model) = pre_tension(force, false);
    let Some(frd) = run_ccx("vorspannung", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let reaction: f64 = (mesh.node_sets["FIX"].iter())
        .map(|&node| node_value(&frd, "FORC", "F1", node))
        .sum();
    assert!((reaction.abs() - force).abs() < 1e-3 * force, "{reaction}");
    // 1000 N on the 10 x 10 mm section, away from the clamped ends.
    for id in (1..=99).filter(|&id| mesh.node(id).is_some_and(|c| c[0] == 50.0)) {
        let s = node_value(&frd, "STRESS", "S11", id);
        assert!((s - 10.0).abs() < 0.3, "{s}");
    }
}

/// A bolt locked in the step after its preload keeps its length and so its force; a
/// shortening of 0 instead loosens it, as the shortening counts from the start.
#[test]
fn calculix_keeps_the_preload_of_a_locked_bolt() {
    let force = 1000.0;
    for (name, lock, expected) in [("verriegelt", true, force), ("gelockert", false, 0.0)] {
        let (mesh, mut model) = pre_tension(force, false);
        let mut second = model.steps[0].clone();
        second.name = "Step-2".into();
        second.loads[0].kind = LoadKind::PreTension {
            value: 0.0,
            by_displacement: true,
            lock,
            direction: None,
        };
        model.steps.push(second);
        let Some(frd) = run_ccx(name, &write_inp(&mesh, &model, "").unwrap()) else {
            return;
        };
        let reaction: f64 = (mesh.node_sets["FIX"].iter())
            .map(|&node| node_value(&frd, "FORC", "F1", node))
            .sum();
        assert!(
            (reaction.abs() - expected).abs() < 1e-3 * force,
            "{name}: {reaction}"
        );
    }
}

/// Shortening the bolt by a prescribed displacement stretches the bar by that much.
#[test]
fn calculix_shortens_the_bolt_by_the_prescribed_displacement() {
    let shortening = 0.01;
    let (mesh, model) = pre_tension(shortening, true);
    let Some(frd) = run_ccx("verkuerzung", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    // The two halves overlap by the shortening: strain 0.01 / 100 in a bar held at both
    // ends, 21 MPa, 2100 N.
    let reaction: f64 = (mesh.node_sets["FIX"].iter())
        .map(|&node| node_value(&frd, "FORC", "F1", node))
        .sum();
    let expected = shortening / 100.0 * 210_000.0 * 100.0;
    assert!(
        (reaction.abs() - expected).abs() < 0.02 * expected,
        "{reaction} != {expected}"
    );
}

#[test]
fn calculix_pressure_on_a_surface_balances_the_reactions() {
    let load = Load {
        distribution: None,
        name: "Pressure-1".into(),
        active: true,
        visible: true,
        region: Region::Surface("TIP".into()),
        kind: LoadKind::Pressure(2.0),
        amplitude: None,
        factor_amplitude: None,
    };
    let (mesh, model) = cantilever(load);
    let Some(frd) = run_ccx("druck", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let reaction: f64 = mesh.node_sets["FIX"]
        .iter()
        .map(|&node| node_value(&frd, "FORC", "F1", node))
        .sum();
    // 2 MPa on the 10 x 10 mm tip face.
    assert!((reaction.abs() - 200.0).abs() < 1e-3, "{reaction}");
}

#[test]
fn calculix_reads_quadratic_elements_over_two_lines() {
    const NODE: NodeId = 141;
    const FORCE: [f64; 3] = [0.0, -500.0, 0.0];
    let load = Load {
        distribution: None,
        name: "Force-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(vec![NODE]),
        kind: LoadKind::force(FORCE),
        amplitude: None,
        factor_amplitude: None,
    };
    let (mesh, model) = analysis("block_c3d20r.inp", load);
    let text = write_inp(&mesh, &model, "").unwrap();
    let element = mesh.elements()[0]
        .nodes
        .iter()
        .map(u32::to_string)
        .collect::<Vec<_>>();
    let first = format!(
        "1, {},\n{}\n",
        element[..15].join(", "),
        element[15..].join(", ")
    );
    assert!(text.contains(&first), "{first}");
    let Some(frd) = run_ccx("block", &text) else {
        return;
    };
    let reference = read_frd(&testdata("block_c3d20r.frd")).unwrap();
    for component in ["U1", "U2", "U3"] {
        let ours = node_value(&frd, "DISP", component, NODE);
        let expected = node_value(&reference, "DISP", component, NODE);
        assert!(
            (ours - expected).abs() <= 1e-6 * expected.abs().max(1e-9),
            "{ours} != {expected}"
        );
    }
}

#[test]
fn surface_traction_spreads_the_total_force_by_area() {
    let mesh = &read_inp(&testdata("block_c3d20r.inp")).unwrap().mesh;
    // Every element face on the x = min side of the block.
    let min_x = mesh.bounds().unwrap().0[0];
    let faces: Vec<(ElementId, u8)> = mesh
        .elements()
        .iter()
        .flat_map(|e| {
            (1..=6u8).filter_map(move |f| {
                let topology = e.shape.faces()[usize::from(f) - 1];
                let on_side = topology
                    .corners
                    .iter()
                    .all(|&l| mesh.node(e.nodes[l]).unwrap()[0] == min_x);
                on_side.then_some((e.id, f))
            })
        })
        .collect();
    assert!(!faces.is_empty());
    let nodal = traction_forces(mesh, &faces, [0.0, -500.0, 30.0], false);
    let total = nodal
        .values()
        .fold([0.0; 3], |sum, f| [0, 1, 2].map(|k| sum[k] + f[k]));
    assert!(
        (total[1] + 500.0).abs() < 1e-9 && (total[2] - 30.0).abs() < 1e-9,
        "{total:?}"
    );
    // Quadratic quads: corners pull against the load direction, midside nodes carry it.
    assert!(nodal.values().any(|f| f[1] > 0.0) && nodal.values().any(|f| f[1] < 0.0));
}

#[test]
fn calculix_balances_a_surface_traction() {
    let load = Load {
        distribution: None,
        name: "Surface_Traction-1".into(),
        active: true,
        visible: true,
        region: Region::Surface("TIP".into()),
        kind: LoadKind::force([0.0, -80.0, 0.0]),
        amplitude: None,
        factor_amplitude: None,
    };
    let (mesh, model) = cantilever(load);
    let text = write_inp(&mesh, &model, "").unwrap();
    let Some(frd) = run_ccx("traktion", &text) else {
        return;
    };
    let reaction: f64 = mesh.node_sets["FIX"]
        .iter()
        .map(|&node| node_value(&frd, "FORC", "F2", node))
        .sum();
    assert!((reaction - 80.0).abs() < 1e-3, "{reaction}");
}

/// Position of the title `name` among the top-level keywords.
fn top_title(tree: &[Keyword], name: &str) -> usize {
    tree.iter()
        .position(|k| k.kind == KeywordKind::Title(name.into()))
        .unwrap()
}

/// Runs CalculiX like [`run_ccx`] and returns the `.dat` file.
fn run_ccx_dat(name: &str, text: &str) -> Option<String> {
    if Command::new("ccx").arg("-v").output().is_err() {
        eprintln!("ccx not found, test skipped");
        return None;
    }
    let dir = std::env::temp_dir().join(format!("plx-write-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(format!("{name}.inp")), text).unwrap();
    let output = Command::new("ccx")
        .args(["-i", name])
        .current_dir(&dir)
        .env("OMP_NUM_THREADS", "1")
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout);
    assert!(output.status.success() && !log.contains("*ERROR"), "{log}");
    let dat = std::fs::read_to_string(dir.join(format!("{name}.dat"))).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    Some(dat)
}

#[test]
fn check_model_replaces_the_procedures_by_no_analysis() {
    let (mesh, mut model) = cantilever(tip_force());
    let text = write_check_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Step\n*No analysis\n"), "{text}");
    assert!(!text.contains("*Static"), "{text}");
    // The loads are still checked.
    assert!(text.contains("*Cload\nInternal_Selection-1_Force-1, 3, -100\n"));
    // A deactivated step stays a comment, and without an active step PrePoMax's CheckModel
    // step is added.
    model.steps[0].active = false;
    let text = write_check_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("** Name: StaticStep: Deactivated\n"),
        "{text}"
    );
    assert!(text.contains("** CheckModel"), "{text}");
    assert_eq!(text.matches("*No analysis").count(), 1, "{text}");
    model.steps.clear();
    let text = write_check_inp(&mesh, &model, "").unwrap();
    let step = text.find("*Step\n*No analysis\n").expect(&text);
    assert!(text[step..].contains("*End step\n"), "{text}");
}

/// The cantilever of `kragbalken_c3d20r.inp` with a frequency step after the static one.
fn frequency_analysis() -> (FeMesh, FeModel) {
    let (mesh, mut model) = analysis("kragbalken_c3d20r.inp", tip_force());
    let mut step = Step::new_frequency("Step-2");
    step.boundary_conditions = model.steps[0].boundary_conditions.clone();
    // A load left in a frequency step must not reach CalculiX.
    step.loads = model.steps[0].loads.clone();
    model.steps.push(step);
    (mesh, model)
}

#[test]
fn a_frequency_step_is_written_like_prepomax_does() {
    let (mesh, mut model) = frequency_analysis();
    let text = write_inp(&mesh, &model, "").unwrap();
    let frequency = &text[text.find("** Step-2").unwrap()..];
    for line in [
        "*Step\n*Frequency\n10\n",
        "*Boundary\nFIX, 1, 6, 0\n",
        "*Node file\nU\n",
        "*El file\nS, E, NOE\n",
    ] {
        assert!(frequency.contains(line), "missing {line:?} in\n{frequency}");
    }
    assert!(!frequency.contains("*Cload"), "{frequency}");
    let StepKind::Frequency(settings) = &mut model.steps[1].kind else {
        unreachable!()
    };
    settings.perturbation = true;
    settings.storage = true;
    settings.num_frequencies = 4;
    settings.upper_frequency = Some(5000.0);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Step, Perturbation\n*Frequency, Storage=Yes\n4, 0, 5000\n"),
        "{text}"
    );
    let StepKind::Frequency(settings) = &mut model.steps[1].kind else {
        unreachable!()
    };
    settings.lower_frequency = Some(100.0);
    settings.upper_frequency = None;
    model.resolve_default_solver(EquationSolver::Spooles);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Frequency, Solver=Spooles, Storage=Yes\n4, 100\n"),
        "{text}"
    );
}

#[test]
fn calculix_finds_the_bending_frequency_of_the_cantilever() {
    let (mesh, mut model) = frequency_analysis();
    // The preload of the static step is small; CalculiX must accept the perturbation step.
    let StepKind::Frequency(settings) = &mut model.steps[1].kind else {
        unreachable!()
    };
    settings.perturbation = true;
    let Some(frd) = run_ccx("eigenfrequenz", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let modes: Vec<f64> = (frd.increments.iter())
        .filter(|i| i.kind == plx_results::AnalysisKind::Frequency)
        .map(|i| i.value)
        .collect();
    assert_eq!(modes.len(), 10, "{modes:?}");
    // Euler-Bernoulli: f1 = 1.875² / (2π L²) √(EI / ρA) = 835 Hz for the 10 x 10 x 100 steel
    // beam; shear makes the real beam a little softer. The square section bends alike in
    // both directions.
    let euler = 1.875_f64.powi(2) / (2.0 * std::f64::consts::PI * 100.0_f64.powi(2))
        * (210_000.0 * 10.0_f64.powi(4) / 12.0 / (7.85e-9 * 100.0)).sqrt();
    for mode in &modes[..2] {
        assert!(
            (0.95 * euler..1.001 * euler).contains(mode),
            "{mode} Hz vs. {euler} Hz"
        );
    }
}

/// The cantilever with a dynamic step of the given duration, increments of `increment`,
/// the tip force applied at once (a stepped amplitude) and the given damping.
fn dynamic_cantilever(
    duration: f64,
    increment: f64,
    damping: Option<plx_model::RayleighDamping>,
) -> (FeMesh, FeModel) {
    let (mesh, mut model) = analysis("kragbalken_c3d20r.inp", tip_force());
    model
        .amplitudes
        .push(amplitude("Stepped", vec![[0.0, 1.0], [1.0, 1.0]]));
    let step = &mut model.steps[0];
    step.loads[0].amplitude = Some("Stepped".into());
    step.field_outputs = FieldOutput::dynamic_defaults();
    step.kind = StepKind::Dynamic(DynamicStep {
        increments: StaticStep {
            incrementation: Incrementation::Direct,
            initial_increment: increment,
            time_period: duration,
            ..StaticStep::default()
        },
        damping,
        ..DynamicStep::default()
    });
    (mesh, model)
}

#[test]
fn a_dynamic_step_is_written_like_prepomax_does() {
    let damping = plx_model::RayleighDamping {
        alpha: 0.0,
        beta: 4e-5,
    };
    let (mesh, mut model) = dynamic_cantilever(1.5e-3, 2.5e-5, Some(damping));
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*Step, Inc=100\n*Dynamic, Direct\n0.000025, 0.0015\n*Damping, Alpha=0, Beta=0.00004\n",
        "*Cload, Amplitude=Stepped\n",
        "*Node file\nRF, U, V\n",
        "*El file\nS, E, ENER, NOE\n",
    ] {
        assert!(text.contains(line), "missing {line:?} in\n{text}");
    }
    let StepKind::Dynamic(settings) = &mut model.steps[0].kind else {
        unreachable!()
    };
    settings.damping = None;
    settings.alpha = -0.1;
    settings.procedure = plx_model::DynamicProcedure::Explicit;
    settings.increments.nlgeom = true;
    settings.increments.incrementation = Incrementation::Automatic;
    settings.increments.solver = EquationSolver::Spooles;
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains(
            "*Step, Nlgeom, Inc=100\n*Dynamic, Solver=Spooles, Alpha=-0.1, Explicit=3\n\
             0.000025, 0.0015, 0.00001, 1.00000000E30\n"
        ),
        "{text}"
    );
    assert!(!text.contains("*Damping"), "{text}");
    model.steps[0].active = false;
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("** Name: DynamicStep: Deactivated\n"),
        "{text}"
    );
}

/// The tip deflection over time of a dynamic increment: (time, U3 at node 99).
fn tip_history(frd: &FrdImport) -> Vec<(f64, f64)> {
    let index = frd.mesh.node_index(99).unwrap();
    (frd.increments.iter())
        .filter(|i| i.kind == plx_results::AnalysisKind::Dynamic)
        .map(|i| {
            let u3 = i.field("DISP").unwrap().component("U3").unwrap().values[index];
            (i.value, f64::from(u3))
        })
        .collect()
}

/// A force applied at once makes the cantilever swing about its static deflection: the
/// first trough is twice the static deflection, half a period of the first mode after the
/// start. Stiffness-proportional damping with a ratio of 0.1 lowers the trough to about
/// 1 + e^(-0.1 pi) = 1.73 times the static deflection.
#[test]
fn calculix_swings_the_cantilever_about_its_static_deflection() {
    let (mesh, reference) = analysis("kragbalken_c3d20r.inp", tip_force());
    let Some(frd) = run_ccx(
        "dynamik_statisch",
        &write_inp(&mesh, &reference, "").unwrap(),
    ) else {
        return;
    };
    let static_u3 = node_value(&frd, "DISP", "U3", 99);
    assert!(static_u3 < 0.0, "{static_u3}");
    // 60 increments over 1.25 periods of the 810 Hz bending mode.
    let (mesh, model) = dynamic_cantilever(1.5e-3, 2.5e-5, None);
    let frd = run_ccx("dynamik", &write_inp(&mesh, &model, "").unwrap()).unwrap();
    let history = tip_history(&frd);
    assert_eq!(history.len(), 60, "{history:?}");
    let (trough_time, trough) = (history.iter().copied())
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .unwrap();
    let ratio = trough / static_u3;
    assert!((1.8..2.1).contains(&ratio), "trough {ratio} x static");
    let half_period = 0.5 / 810.0;
    assert!(
        (trough_time - half_period).abs() < 0.15 * half_period,
        "trough at {trough_time} s, expected {half_period} s"
    );
    let omega = 2.0 * std::f64::consts::PI * 810.0;
    let damping = plx_model::RayleighDamping {
        alpha: 0.0,
        beta: 2.0 * 0.1 / omega,
    };
    let (mesh, model) = dynamic_cantilever(1.5e-3, 2.5e-5, Some(damping));
    let frd = run_ccx("dynamik_gedaempft", &write_inp(&mesh, &model, "").unwrap()).unwrap();
    let damped = (tip_history(&frd).iter()).map(|h| h.1).fold(0.0, f64::min);
    let damped_ratio = damped / static_u3;
    assert!(
        (1.55..1.9).contains(&damped_ratio) && damped_ratio < ratio - 0.08,
        "damped trough {damped_ratio} x static, undamped {ratio}"
    );
}

/// The cantilever with a frequency step that stores its modes, a modal dynamics step with
/// the tip force applied at once, and a steady state sweep of the tip force around the
/// first bending mode; both modal steps damped as given.
fn modal_cantilever(damping: Option<ModalDamping>) -> (FeMesh, FeModel) {
    let (mesh, mut model) = analysis("kragbalken_c3d20r.inp", tip_force());
    model
        .amplitudes
        .push(amplitude("Stepped", vec![[0.0, 1.0], [1.0, 1.0]]));
    let supports = model.steps[0].boundary_conditions.clone();
    let mut force = model.steps[0].loads[0].clone();
    force.amplitude = Some("Stepped".into());
    let mut frequency = Step::new_frequency("Step-1");
    frequency.boundary_conditions = supports.clone();
    let StepKind::Frequency(settings) = &mut frequency.kind else {
        unreachable!()
    };
    settings.storage = true;
    settings.num_frequencies = 6;
    let mut modal = Step::new_modal_dynamics("Step-2");
    modal.boundary_conditions = supports.clone();
    modal.loads = vec![force];
    modal.kind = StepKind::ModalDynamics(ModalDynamicsStep {
        increment: 2.5e-5,
        time_period: 1.5e-3,
        damping: damping.clone(),
        ..ModalDynamicsStep::default()
    });
    let mut sweep = Step::new_steady_state_dynamics("Step-3");
    sweep.boundary_conditions = supports;
    sweep.loads = model.steps[0].loads.clone();
    sweep.kind = StepKind::SteadyStateDynamics(SteadyStateDynamicsStep {
        lower_frequency: 700.0,
        upper_frequency: 900.0,
        data_points: 21,
        bias: 1.0,
        damping,
        ..SteadyStateDynamicsStep::default()
    });
    model.steps = vec![frequency, modal, sweep];
    (mesh, model)
}

/// The cantilever of `kragbalken_c3d20r.inp` in a buckle step, pressed along its axis by
/// 21 N shared by the 21 nodes of its free end.
fn buckle_analysis() -> (FeMesh, FeModel) {
    let (mesh, mut model) = analysis(
        "kragbalken_c3d20r.inp",
        Load {
            distribution: None,
            name: "Force-1".into(),
            active: true,
            visible: true,
            region: Region::NodeSet("TIP".into()),
            kind: LoadKind::force([-21.0, 0.0, 0.0]),
            amplitude: None,
            factor_amplitude: None,
        },
    );
    let mut step = Step::new_buckle("Step-1");
    step.boundary_conditions = model.steps[0].boundary_conditions.clone();
    step.loads = model.steps[0].loads.clone();
    model.steps = vec![step];
    (mesh, model)
}

#[test]
fn modal_steps_are_written_like_prepomax_does() {
    let (mesh, mut model) = modal_cantilever(Some(ModalDamping::Constant(0.02)));
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*Step\n*Frequency, Storage=Yes\n6\n",
        "*Step, Inc=100\n*Modal dynamics\n0.000025, 0.0015\n*Modal damping\n1, 1000000, 0.02\n",
        "*Cload, Amplitude=Stepped\n",
        "*Step\n*Steady state dynamics\n700, 900, 21, 1\n*Modal damping\n1, 1000000, 0.02\n",
        "*Node file\nRF, U, V\n",
    ] {
        assert!(text.contains(line), "missing {line:?} in\n{text}");
    }
    let StepKind::ModalDynamics(settings) = &mut model.steps[1].kind else {
        unreachable!()
    };
    settings.time_period = 0.01;
    settings.steady_state = true;
    settings.solver = EquationSolver::Spooles;
    settings.damping = Some(ModalDamping::Direct(vec![
        plx_model::ModeDamping {
            lowest: 1,
            highest: 2,
            ratio: 0.05,
        },
        plx_model::ModeDamping {
            lowest: 3,
            highest: 6,
            ratio: 0.1,
        },
    ]));
    let StepKind::SteadyStateDynamics(settings) = &mut model.steps[2].kind else {
        unreachable!()
    };
    settings.harmonic = false;
    settings.fourier_terms = 5;
    settings.time_upper = 0.002;
    settings.damping = Some(ModalDamping::Rayleigh(plx_model::RayleighDamping {
        alpha: 10.0,
        beta: 4e-5,
    }));
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*Step, Inc=100\n*Modal dynamics, Solver=Spooles, Steady state\n0.000025, 0.01\n\
         *Modal damping\n1, 2, 0.05\n3, 6, 0.1\n",
        "*Step\n*Steady state dynamics, Harmonic=No\n700, 900, 21, 1, 5, 0, 0.002\n\
         *Modal damping, Rayleigh\n , , 10, 0.00004\n",
    ] {
        assert!(text.contains(line), "missing {line:?} in\n{text}");
    }
    // The increment count grows with the time period.
    let StepKind::ModalDynamics(settings) = &mut model.steps[1].kind else {
        unreachable!()
    };
    settings.steady_state = false;
    settings.time_period = 0.01;
    assert_eq!(settings.increments(), 401);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Step, Inc=401\n*Modal dynamics"), "{text}");
    model.steps[1].active = false;
    model.steps[2].active = false;
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("** Name: ModalDynamicsStep: Deactivated\n"),
        "{text}"
    );
    assert!(
        text.contains("** Name: SteadyStateDynamicsStep: Deactivated\n"),
        "{text}"
    );
}

#[test]
fn a_buckle_step_is_written_like_prepomax_does() {
    let (mesh, mut model) = buckle_analysis();
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*Step\n*Buckle\n1, 0.0001\n",
        "*Boundary\nFIX, 1, 6, 0\n",
        "*Cload\nTIP, 1, -1\n",
        "*Node file\nRF, U\n",
    ] {
        assert!(text.contains(line), "missing {line:?} in\n{text}");
    }
    let StepKind::Buckle(settings) = &mut model.steps[0].kind else {
        unreachable!()
    };
    settings.perturbation = true;
    settings.num_factors = 3;
    settings.accuracy = 0.01;
    model.resolve_default_solver(EquationSolver::Spooles);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Step, Perturbation\n*Buckle, Solver=Spooles\n3, 0.01\n"),
        "{text}"
    );
}

/// The modal superposition swings the tip like the direct integration does, damped by the
/// modal damping ratio of 0.1 to about 1.73 times the static deflection; the harmonic
/// sweep peaks near the first bending mode at about 1 / (2 zeta) = 5 times the static
/// deflection, a little less where the 10 Hz grid misses the resonance.
#[test]
fn calculix_superposes_the_stored_modes_of_the_cantilever() {
    let (mesh, reference) = analysis("kragbalken_c3d20r.inp", tip_force());
    let Some(frd) = run_ccx("modal_statisch", &write_inp(&mesh, &reference, "").unwrap()) else {
        return;
    };
    let static_u3 = node_value(&frd, "DISP", "U3", 99);
    let (mesh, model) = modal_cantilever(Some(ModalDamping::Constant(0.1)));
    let frd = run_ccx("modal", &write_inp(&mesh, &model, "").unwrap()).unwrap();
    let index = frd.mesh.node_index(99).unwrap();
    let tip = |i: &plx_results::Increment| {
        f64::from(i.field("DISP").unwrap().component("U3").unwrap().values[index])
    };
    let modes: Vec<f64> = (frd.increments.iter())
        .filter(|i| i.kind == plx_results::AnalysisKind::Frequency)
        .map(|i| i.value)
        .collect();
    assert_eq!(modes.len(), 6, "{modes:?}");
    assert!((780.0..835.0).contains(&modes[0]), "{modes:?}");
    let history: Vec<f64> = (frd.increments.iter())
        .filter(|i| i.step == 2)
        .map(tip)
        .collect();
    assert_eq!(history.len(), 60, "{history:?}");
    let trough = history.iter().copied().fold(0.0, f64::min) / static_u3;
    assert!((1.55..1.9).contains(&trough), "trough {trough} x static");
    // Each frequency point carries the real part (DISP) and the imaginary part (DISPI);
    // the magnitude peaks at resonance.
    let imaginary = |i: &plx_results::Increment| {
        (i.field("DISPI"))
            .and_then(|f| f.component("U3"))
            .map_or(0.0, |c| f64::from(c.values[index]))
    };
    let sweep: Vec<(f64, f64)> = (frd.increments.iter())
        .filter(|i| i.step == 3)
        .map(|i| (i.value, tip(i).hypot(imaginary(i))))
        .collect();
    assert!(sweep.len() >= 21, "{}", sweep.len());
    let mut peak = 0.0_f64;
    let mut at = 0.0;
    for (frequency, magnitude) in &sweep {
        if *magnitude > peak {
            peak = *magnitude;
            at = *frequency;
        }
    }
    let ratio = peak / static_u3.abs();
    assert!(
        (4.0..5.6).contains(&ratio),
        "peak {ratio} x static at {at} Hz"
    );
    assert!(
        (at - modes[0]).abs() < 12.0,
        "peak at {at} Hz, mode at {}",
        modes[0]
    );
}

#[test]
fn calculix_finds_the_euler_load_of_the_cantilever() {
    let (mesh, mut model) = buckle_analysis();
    let StepKind::Buckle(settings) = &mut model.steps[0].kind else {
        unreachable!()
    };
    settings.num_factors = 2;
    let Some(frd) = run_ccx("beulen", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let buckling: Vec<_> = (frd.increments.iter())
        .filter(|i| i.kind == plx_results::AnalysisKind::Buckling)
        .collect();
    // The static solution of the reference load comes first as increment 0, then one
    // increment per buckling mode with its factor and mode shape.
    let (reference, buckling) = buckling.split_first().unwrap();
    assert_eq!((reference.increment, reference.value), (0, 0.0));
    assert!(reference.field("STRESS").is_some());
    let factors: Vec<f64> = buckling.iter().map(|i| i.value).collect();
    assert_eq!(factors.len(), 2, "{factors:?}");
    let modes: Vec<u32> = buckling.iter().map(|i| i.increment).collect();
    assert_eq!(modes, [1, 2]);
    assert!(buckling.iter().all(|i| i.displacements().is_some()));
    // Euler case 1: P = π² EI / (4 L²) for the 10 x 10 x 100 steel beam, against the 21 N
    // of the load; shear makes the real beam a little softer. The square section buckles
    // alike in both directions.
    let euler = std::f64::consts::PI.powi(2) * 210_000.0 * 10.0_f64.powi(4)
        / 12.0
        / (4.0 * 100.0_f64.powi(2))
        / 21.0;
    for factor in &factors {
        assert!(
            (0.95 * euler..1.001 * euler).contains(factor),
            "{factor} vs. {euler}"
        );
    }
}

#[test]
fn the_buckling_mode_exports_as_a_deformed_mesh() {
    let (mesh, model) = buckle_analysis();
    let Some(frd) = run_ccx("beulform", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let mode = frd.increments.iter().find(|i| i.increment == 1).unwrap();
    let displacements = mode.displacements().unwrap();
    let text = write_deformed_mesh_inp(&frd.mesh, &displacements, 0.5, "Beulform").unwrap();
    // Only the mesh: no material, section or step.
    for keyword in ["*Material", "*Solid section", "*Step", "*Boundary"] {
        assert!(!text.contains(keyword), "{keyword} in\n{text}");
    }
    let read = read_inp_str(&text, None).unwrap().mesh;
    assert_eq!(read.node_ids(), frd.mesh.node_ids());
    assert_eq!(read.element_count(), frd.mesh.element_count());
    for ((moved, start), u) in read
        .coords()
        .iter()
        .zip(frd.mesh.coords())
        .zip(&displacements)
    {
        for k in 0..3 {
            let expected = start[k] + 0.5 * f64::from(u[k]);
            assert!(
                (moved[k] - expected).abs() < 1e-6,
                "{moved:?} vs. {start:?} + {u:?}"
            );
        }
    }
    // The free end moves sideways in the mode, the clamped end stays.
    let tip = frd.mesh.node_index(41).unwrap();
    let fixed = frd.mesh.node_index(1).unwrap();
    assert!(read.coords()[tip] != frd.mesh.coords()[tip]);
    assert_eq!(read.coords()[fixed], frd.mesh.coords()[fixed]);
}

#[test]
fn a_preloaded_buckle_step_counts_its_modes_from_one() {
    let (mesh, mut model) = buckle_analysis();
    // A static step with a side load before the buckle step, taken over as preload.
    let mut preload = Step::new_static("Step-1");
    preload.boundary_conditions = model.steps[0].boundary_conditions.clone();
    preload.loads.push(tip_force());
    model.steps.insert(0, preload);
    model.steps[1].name = "Step-2".into();
    let StepKind::Buckle(settings) = &mut model.steps[1].kind else {
        unreachable!()
    };
    settings.perturbation = true;
    let Some(frd) = run_ccx("vorspannung-beulen", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let ids: Vec<_> = (frd.increments.iter())
        .map(|i| (i.step, i.increment, i.kind))
        .collect();
    use plx_results::AnalysisKind::{Buckling, Static};
    assert_eq!(ids, [(1, 1, Static), (2, 0, Buckling), (2, 1, Buckling)]);
}

#[test]
fn deactivated_items_are_left_out_as_comments() {
    let (mesh, mut model) = cantilever(tip_force());
    model.steps[0].loads.push(Load {
        distribution: None,
        name: "Empty-1".into(),
        active: false,
        visible: true,
        // Not even an empty region stops the export of a deactivated item.
        region: Region::Nodes(Vec::new()),
        kind: LoadKind::force([1.0, 0.0, 0.0]),
        amplitude: None,
        factor_amplitude: None,
    });
    model.steps[0].loads[0].active = false;
    model.steps[0].boundary_conditions[0].active = false;
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*Boundary, op=New\n** Name: Fixed-1: Deactivated\n",
        "*Cload, op=New\n*Dload, op=New\n** Name: Force-1: Deactivated\n** Name: Empty-1: Deactivated\n",
    ] {
        assert!(text.contains(line), "missing {line:?} in\n{text}");
    }
    assert!(!text.contains("FIX, 1, 6, 0"), "{text}");
    assert!(!text.contains("_Force-1"), "{text}");
}

#[test]
fn a_deactivated_step_is_written_as_comments_only() {
    let (mesh, mut model) = frequency_analysis();
    model.steps[0].active = false;
    let text = write_inp(&mesh, &model, "").unwrap();
    let step = &text[text.find("** Step-1").unwrap()..text.find("** Step-2").unwrap()];
    for line in [
        "** Name: Step-1: Deactivated\n** Name: StaticStep: Deactivated\n",
        "** Name: Fixed-1: Deactivated\n",
        "** Name: Force-1: Deactivated\n",
        "** Name: NF-Output-1: Deactivated\n",
    ] {
        assert!(step.contains(line), "missing {line:?} in\n{step}");
    }
    let keywords = step
        .lines()
        .filter(|l| l.starts_with('*') && !l.starts_with("**"));
    assert_eq!(keywords.count(), 0, "{step}");
    // The node set of the load in the deactivated step is not written either.
    assert!(!text.contains("_Force-1"), "{text}");
    // CalculiX runs the remaining frequency step alone.
    let Some(frd) = run_ccx("deaktiviert", &text) else {
        return;
    };
    assert!(
        (frd.increments.iter()).all(|i| i.kind == plx_results::AnalysisKind::Frequency),
        "{:?}",
        frd.increments.len()
    );
}

/// A rectangle of quadratic quadrilaterals in the x-y plane from (`x0`, 0) to
/// (`x0 + width`, `height`), `nx` by `ny` elements, typed as shells (`S8`) so the model space
/// decides the element type. Elements count row by row from 1.
fn rectangle(x0: f64, width: f64, height: f64, nx: u32, ny: u32) -> FeMesh {
    let mut mesh = FeMesh::default();
    let columns = 2 * nx + 1;
    let id = |i: u32, j: u32| j * columns + i + 1;
    for j in 0..=2 * ny {
        for i in 0..=2 * nx {
            if i % 2 == 1 && j % 2 == 1 {
                continue;
            }
            let x = x0 + width * f64::from(i) / f64::from(2 * nx);
            let y = height * f64::from(j) / f64::from(2 * ny);
            mesh.set_node(id(i, j), [x, y, 0.0]);
        }
    }
    let mut part = plx_mesh::Part {
        name: "PLATE".into(),
        elements: Vec::new(),
    };
    for row in 0..ny {
        for column in 0..nx {
            let (i, j) = (2 * column, 2 * row);
            let element = row * nx + column + 1;
            let nodes = vec![
                id(i, j),
                id(i + 2, j),
                id(i + 2, j + 2),
                id(i, j + 2),
                id(i + 1, j),
                id(i + 2, j + 1),
                id(i + 1, j + 2),
                id(i, j + 1),
            ];
            mesh.add_element(plx_mesh::Element {
                id: element,
                type_name: "S8".into(),
                shape: plx_mesh::ElementShape::Quad8,
                nodes,
            })
            .unwrap();
            part.elements.push(element);
        }
    }
    mesh.parts.push(part);
    mesh
}

/// Nodes of the mesh where `coordinate` of the axis is `value`.
fn nodes_at(mesh: &FeMesh, axis: usize, value: f64) -> Vec<NodeId> {
    (mesh.node_ids().iter().zip(mesh.coords()))
        .filter(|(_, c)| (c[axis] - value).abs() < 1e-9)
        .map(|(&id, _)| id)
        .collect()
}

/// Edges of the rectangle's elements lying on the line where `axis` is `value`, as faces.
fn edges_at(mesh: &FeMesh, axis: usize, value: f64) -> Vec<(ElementId, u8)> {
    let mut faces = Vec::new();
    for element in mesh.elements() {
        for (k, edge) in element.shape.edges().iter().enumerate() {
            let on = (edge.corners.iter())
                .all(|&l| (mesh.node(element.nodes[l]).unwrap()[axis] - value).abs() < 1e-9);
            if on {
                faces.push((element.id, k as u8 + 1));
            }
        }
    }
    faces
}

/// A 2D model of the rectangle: steel, `thickness`, the given boundary conditions and load.
fn plane_model(
    space: ModelSpace,
    thickness: f64,
    bcs: Vec<(Vec<NodeId>, [Option<f64>; 6])>,
    load: LoadKind,
    region: Region,
) -> FeModel {
    let mut step = Step::new_static("Step-1");
    for (k, (nodes, values)) in bcs.into_iter().enumerate() {
        step.boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: format!("Displacement_Rotation-{}", k + 1),
            active: true,
            visible: true,
            region: Region::Nodes(nodes),
            kind: each_node(values),
            amplitude: None,
        });
    }
    step.loads.push(Load {
        distribution: None,
        name: "Load-1".into(),
        active: true,
        visible: true,
        region,
        kind: load,
        amplitude: None,
        factor_amplitude: None,
    });
    FeModel {
        properties: plx_model::ModelProperties {
            space,
            ..Default::default()
        },
        materials: vec![Material {
            name: "Steel".into(),
            density: None,
            elastic: Some(Elastic {
                young: 210_000.0,
                poisson: 0.3,
            }),
            ..Default::default()
        }],
        sections: vec![Section {
            name: "Section-1".into(),
            material: "Steel".into(),
            region: Region::Parts(vec!["PLATE".into()]),
            thickness,
            kind: SectionKind::Solid,
        }],
        steps: vec![step],
        ..FeModel::default()
    }
}

const FIX_X: [Option<f64>; 6] = [Some(0.0), None, None, None, None, None];
const FIX_Y: [Option<f64>; 6] = [None, Some(0.0), None, None, None, None];

#[test]
fn two_d_models_write_their_element_types_thickness_and_dofs() {
    let mesh = rectangle(0.0, 10.0, 2.0, 2, 1);
    let left = nodes_at(&mesh, 0, 0.0);
    let mut model = plane_model(
        ModelSpace::PlaneStress,
        2.5,
        vec![(left, FIX_X)],
        LoadKind::force([1.0, 2.0, 3.0]),
        Region::Nodes(vec![5]),
    );
    model.steps[0].boundary_conditions[0].kind = BoundaryKind::displacement([Some(0.0); 6]);
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*Element, Type=CPS8, Elset=PLATE\n",
        "*Solid section, Elset=Internal_Selection-1_Section-1, Material=Steel\n2.5\n",
        ", 1, 2, 0\n",
        "*Cload\nInternal_Selection-1_Load-1, 1, 1\nInternal_Selection-1_Load-1, 2, 2\n**\n",
    ] {
        assert!(text.contains(line), "{line:?} missing in\n{text}");
    }
    model.properties.space = ModelSpace::Axisymmetric;
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Element, Type=CAX8, Elset=PLATE\n"));
    assert!(text.contains("Material=Steel\n**"), "{text}");
}

#[test]
fn edge_tractions_follow_length_and_radius() {
    // One quadratic edge from x = 1 to x = 3: by length 1/6, 4/6, 1/6.
    let weights = edge_weights(&[[1.0, 0.0, 0.0], [3.0, 0.0, 0.0], [2.0, 0.0, 0.0]], false);
    for (w, expected) in weights.iter().zip([2.0 / 6.0, 2.0 / 6.0, 8.0 / 6.0]) {
        assert!((w - expected).abs() < 1e-12, "{weights:?}");
    }
    // Times the radius: the integral of r over the edge is 4.
    let weights = edge_weights(&[[1.0, 0.0, 0.0], [3.0, 0.0, 0.0]], true);
    assert!((weights.iter().sum::<f64>() - 4.0).abs() < 1e-12);
    assert!((weights[0] - 5.0 / 3.0).abs() < 1e-12, "{weights:?}");
}

/// Plane stress: a strip pulled by a surface traction stretches by sigma L / E.
#[test]
fn calculix_stretches_a_plane_stress_strip() {
    let (length, height, thickness, force) = (10.0, 2.0, 2.0, 400.0);
    let mesh = rectangle(0.0, length, height, 5, 2);
    let model = plane_model(
        ModelSpace::PlaneStress,
        thickness,
        vec![(nodes_at(&mesh, 0, 0.0), FIX_X), (vec![1], FIX_Y)],
        LoadKind::force([force, 0.0, 0.0]),
        Region::Faces(edges_at(&mesh, 0, length)),
    );
    let Some(frd) = run_ccx(
        "ebener-spannungszustand",
        &write_inp(&mesh, &model, "").unwrap(),
    ) else {
        return;
    };
    let stress = force / (height * thickness);
    let expected = stress * length / 210_000.0;
    for node in nodes_at(&mesh, 0, length) {
        let u = node_value(&frd, "DISP", "U1", node);
        assert!((u - expected).abs() < 1e-5 * expected, "{u} != {expected}");
        let s = node_value(&frd, "STRESS", "S11", node);
        assert!((s - stress).abs() < 1e-3 * stress, "{s} != {stress}");
    }
}

/// Plane strain: the same strip under pressure is stiffer by 1 - nu^2 and has a stress
/// nu sigma across its plane.
#[test]
fn calculix_stretches_a_plane_strain_strip() {
    let (length, height, pressure) = (10.0, 2.0, -100.0);
    let mesh = rectangle(0.0, length, height, 5, 2);
    let model = plane_model(
        ModelSpace::PlaneStrain,
        1.0,
        vec![(nodes_at(&mesh, 0, 0.0), FIX_X), (vec![1], FIX_Y)],
        LoadKind::Pressure(pressure),
        Region::Faces(edges_at(&mesh, 0, length)),
    );
    let Some(frd) = run_ccx(
        "ebener-verzerrungszustand",
        &write_inp(&mesh, &model, "").unwrap(),
    ) else {
        return;
    };
    let expected = (1.0 - 0.3 * 0.3) * -pressure * length / 210_000.0;
    for node in nodes_at(&mesh, 0, length) {
        let u = node_value(&frd, "DISP", "U1", node);
        assert!((u - expected).abs() < 1e-5 * expected, "{u} != {expected}");
        let szz = node_value(&frd, "STRESS", "S33", node);
        assert!((szz - 30.0).abs() < 1e-2, "{szz}");
    }
}

/// Axisymmetric: a thick-walled tube under internal pressure, held axially, widens as Lamé's
/// solution for plane strain says.
#[test]
fn calculix_widens_a_tube_under_internal_pressure() {
    let (a, b, pressure) = (10.0, 20.0, 100.0);
    let mesh = rectangle(a, b - a, 2.0, 10, 1);
    let model = plane_model(
        ModelSpace::Axisymmetric,
        1.0,
        vec![(mesh.node_ids().to_vec(), FIX_Y)],
        LoadKind::Pressure(pressure),
        Region::Faces(edges_at(&mesh, 0, a)),
    );
    let Some(frd) = run_ccx("rohr-innendruck", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let (e, nu) = (210_000.0, 0.3);
    let lame = |r: f64| {
        (1.0 + nu) / e * a * a * pressure / (b * b - a * a) * ((1.0 - 2.0 * nu) * r + b * b / r)
    };
    for r in [a, b] {
        let expected = lame(r);
        for node in nodes_at(&mesh, 0, r) {
            let u = node_value(&frd, "DISP", "U1", node);
            assert!(
                (u - expected).abs() < 2e-3 * expected,
                "r = {r}: {u} != {expected}"
            );
        }
    }
}

/// Axisymmetric: a total axial force on the end of a tube is the force on the whole
/// revolution, so the axial stress is F / (pi (b^2 - a^2)).
#[test]
fn calculix_pulls_a_tube_by_its_total_force() {
    let (a, b, height, force) = (10.0, 12.0, 5.0, 10_000.0);
    let mesh = rectangle(a, b - a, height, 2, 3);
    let model = plane_model(
        ModelSpace::Axisymmetric,
        1.0,
        vec![(nodes_at(&mesh, 1, 0.0), FIX_Y)],
        LoadKind::force([0.0, force, 0.0]),
        Region::Faces(edges_at(&mesh, 1, height)),
    );
    let Some(frd) = run_ccx("rohr-zug", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let stress = force / (std::f64::consts::PI * (b * b - a * a));
    for node in mesh.node_ids() {
        let s = node_value(&frd, "STRESS", "S22", *node);
        assert!(
            (s - stress).abs() < 1e-3 * stress,
            "{node}: {s} != {stress}"
        );
    }
}

type Faces = Vec<(ElementId, u8)>;

/// Two blocks of 10 x 10 x 10 mm stacked along z, parts LOWER and UPPER, each 2 x 2 x 1
/// C3D8 elements with own nodes, the upper one `gap` above the lower one. Returns the mesh
/// with the top faces of LOWER and the bottom faces of UPPER.
fn two_blocks(gap: f64) -> (FeMesh, Faces, Faces) {
    let mut mesh = FeMesh::default();
    let mut faces = [Vec::new(), Vec::new()];
    for (block, z0) in [(0u32, 0.0), (1, 10.0 + gap)] {
        let node = |i: u32, j: u32, k: u32| 100 * block + 1 + i + 3 * j + 9 * k;
        for k in 0..2 {
            for j in 0..3 {
                for i in 0..3 {
                    let coords = [
                        5.0 * f64::from(i),
                        5.0 * f64::from(j),
                        z0 + 10.0 * f64::from(k),
                    ];
                    mesh.set_node(node(i, j, k), coords);
                }
            }
        }
        let mut elements = Vec::new();
        for j in 0..2 {
            for i in 0..2 {
                let id = 100 * block + 1 + i + 2 * j;
                let mut nodes = Vec::new();
                for k in 0..2 {
                    nodes.extend([node(i, j, k), node(i + 1, j, k), node(i + 1, j + 1, k)]);
                    nodes.push(node(i, j + 1, k));
                }
                mesh.add_element(plx_mesh::Element {
                    id,
                    type_name: "C3D8".into(),
                    shape: plx_mesh::ElementShape::Hex8,
                    nodes,
                })
                .unwrap();
                elements.push(id);
                // Top face S2 of the lower block, bottom face S1 of the upper one.
                faces[block as usize].push((id, if block == 0 { 2 } else { 1 }));
            }
        }
        let name = if block == 0 { "LOWER" } else { "UPPER" };
        mesh.parts.push(plx_mesh::Part {
            name: name.into(),
            elements,
        });
    }
    let [lower, upper] = faces;
    (mesh, lower, upper)
}

/// Steel on both blocks, the lower one held at its bottom and 10 MPa pressing on the top of
/// the upper one.
fn stacked_blocks(mesh: &FeMesh) -> FeModel {
    let mut model = FeModel::default();
    let bottom: Vec<NodeId> = (mesh.node_ids().iter().zip(mesh.coords()))
        .filter(|(_, c)| c[2] == 0.0)
        .map(|(&id, _)| id)
        .collect();
    let mut step = Step::new_static("Step-1");
    step.boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Fixed-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(bottom),
        kind: BoundaryKind::displacement([Some(0.0); 6]),
        amplitude: None,
    });
    step.loads.push(Load {
        distribution: None,
        name: "Pressure-1".into(),
        active: true,
        visible: true,
        region: Region::Faces((101..=104).map(|e| (e, 2)).collect()),
        kind: LoadKind::Pressure(10.0),
        amplitude: None,
        factor_amplitude: None,
    });
    model.materials.push(Material {
        name: "Steel".into(),
        density: None,
        elastic: Some(Elastic {
            young: 210_000.0,
            poisson: 0.3,
        }),
        ..Default::default()
    });
    model.sections.push(Section {
        name: "Section-1".into(),
        material: "Steel".into(),
        region: Region::Parts(vec!["LOWER".into(), "UPPER".into()]),
        thickness: 1.0,
        kind: SectionKind::Solid,
    });
    model.steps.push(step);
    model
}

/// A bolt split into two parts and tied at its cut: the preload goes through the tie, whichever
/// side of it the pre-tension faces are on. A tie to the faces of the cut side would bind the
/// other part to the copies of the cut nodes and bypass the preload.
#[test]
fn calculix_preloads_a_bolt_split_into_two_tied_parts() {
    let force = 1000.0;
    // Faces picked on the FE mesh, and on the CAD geometry, whose nodes are the original ones.
    for (cut_is_master, on_geometry) in [(true, false), (false, false), (true, true), (false, true)]
    {
        let (mut mesh, lower, upper) = two_blocks(0.0);
        for (face, faces) in [(1, &lower), (2, &upper)] {
            let nodes: BTreeSet<NodeId> = (faces.iter())
                .flat_map(|&(e, f)| constraints::face_node_ids(&mesh, e, f))
                .collect();
            mesh.cad.faces.insert(face, faces.clone());
            mesh.cad
                .nodes
                .insert(CadEntity::Face(face), nodes.into_iter().collect());
        }
        let side = |face: i32, faces: &Faces| {
            if on_geometry {
                Region::Geometry(vec![CadEntity::Face(face)])
            } else {
                Region::Faces(faces.clone())
            }
        };
        let mut model = stacked_blocks(&mesh);
        let ends: Vec<NodeId> = (mesh.node_ids().iter().zip(mesh.coords()))
            .filter(|(_, c)| c[2] == 0.0 || c[2] == 20.0)
            .map(|(&id, _)| id)
            .collect();
        let step = &mut model.steps[0];
        step.boundary_conditions[0].region = Region::Nodes(ends.clone());
        step.loads = vec![Load {
            distribution: None,
            name: "Pre_Tension-1".into(),
            active: true,
            visible: true,
            region: Region::Faces(lower.clone()),
            kind: LoadKind::PreTension {
                value: force,
                by_displacement: false,
                lock: false,
                direction: None,
            },
            amplitude: None,
            factor_amplitude: None,
        }];
        let mut tie = plx_model::Tie::new("Tie-1");
        (tie.master, tie.slave) = if cut_is_master {
            (side(1, &lower), side(2, &upper))
        } else {
            (side(2, &upper), side(1, &lower))
        };
        model.ties.push(tie);
        let text = write_inp(&mesh, &model, "").unwrap();
        // The cut nodes 19 to 27 of the lower block are the slave nodes.
        assert!(text.contains("Slave, Type=Node\n"), "{text}");
        let name = format!("geteilt-{cut_is_master}-{on_geometry}");
        let Some(frd) = run_ccx(&name, &text) else {
            return;
        };
        let reaction: f64 = (ends.iter())
            .filter(|&&n| mesh.node(n).unwrap()[2] == 0.0)
            .map(|&node| node_value(&frd, "FORC", "F3", node))
            .sum();
        assert!(
            (reaction.abs() - force).abs() < 1e-3 * force,
            "{name}: {reaction}"
        );
    }
}

/// Two blocks clamped at their far ends with their faces of the cut on the geometry, and a
/// bolt across them: the bolt and a bonded contact region on the same faces.
fn bolted_blocks(preload: f64) -> (FeMesh, FeModel, Vec<NodeId>) {
    let (mut mesh, lower, upper) = two_blocks(0.0);
    for (face, faces) in [(1, &lower), (2, &upper)] {
        let nodes: BTreeSet<NodeId> = (faces.iter())
            .flat_map(|&(e, f)| constraints::face_node_ids(&mesh, e, f))
            .collect();
        mesh.cad.faces.insert(face, faces.clone());
        mesh.cad
            .nodes
            .insert(CadEntity::Face(face), nodes.into_iter().collect());
    }
    let mut model = stacked_blocks(&mesh);
    let ends: Vec<NodeId> = (mesh.node_ids().iter().zip(mesh.coords()))
        .filter(|(_, c)| c[2] == 0.0 || c[2] == 20.0)
        .map(|(&id, _)| id)
        .collect();
    model.steps[0].boundary_conditions[0].region = Region::Nodes(ends.clone());
    model.steps[0].loads.clear();
    let mut second = model.steps[0].clone();
    second.name = "Step-2".into();
    model.steps.push(second);
    let mut region = plx_model::ContactRegion::new(
        "Bonded - LOWER to UPPER",
        plx_model::ContactBehavior::Bonded,
    );
    region.side_a = Region::Geometry(vec![CadEntity::Face(2)]);
    region.side_b = Region::Geometry(vec![CadEntity::Face(1)]);
    model.contact_regions.push(region);
    let mut bolt = plx_model::Bolt::new("Bolt-1");
    bolt.side_a = Region::Geometry(vec![CadEntity::Face(2)]);
    bolt.side_b = Region::Geometry(vec![CadEntity::Face(1)]);
    bolt.preload = plx_model::Preload::Force(preload);
    model.bolts.push(bolt);
    let bottom = (ends.iter().copied())
        .filter(|&n| mesh.node(n).unwrap()[2] == 0.0)
        .collect();
    (mesh, model, bottom)
}

#[test]
fn a_bolt_replaces_the_contact_region_on_its_cut_and_locks_after_its_preload() {
    let (mesh, model, _) = bolted_blocks(1000.0);
    assert_eq!(
        model.replaced_contact_regions(&mesh),
        [(0, 0)].into_iter().collect()
    );
    let (lowered, origin) = model.lower_bolts(&mesh);
    assert!(!lowered.contact_regions[0].active);
    // The cut is on one side, tied to the other one.
    assert_eq!(lowered.ties.len(), 1);
    assert_eq!(origin.bolt_of(plx_model::ModelItem::Tie(0)), Some(0));
    let kinds: Vec<_> = (lowered.steps.iter())
        .map(|s| s.loads[0].kind.clone())
        .collect();
    assert!(matches!(
        kinds[0],
        LoadKind::PreTension {
            value: 1000.0,
            lock: false,
            ..
        }
    ));
    assert!(matches!(kinds[1], LoadKind::PreTension { lock: true, .. }));
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Cload\n"), "{text}");
    assert!(text.contains("*Boundary, Fixed\n"), "{text}");
    assert!(
        text.contains("** Name: Bonded - LOWER to UPPER: Deactivated"),
        "{text}"
    );
    assert!(text.contains("*Tie, Name=Bolt-1_Cut"), "{text}");
}

#[test]
fn a_bolt_follows_the_actions_of_its_steps() {
    let (mesh, mut model, _) = bolted_blocks(1000.0);
    model.steps[1].bolt_changes.push(plx_model::BoltChange {
        bolt: "Bolt-1".into(),
        action: plx_model::BoltAction::Open,
    });
    let (lowered, _) = model.lower_bolts(&mesh);
    assert_eq!(lowered.steps[1].loads, []);
    model.steps[1].bolt_changes[0].action = plx_model::BoltAction::Adjust(0.01);
    let (lowered, _) = model.lower_bolts(&mesh);
    assert!(matches!(
        lowered.steps[1].loads[0].kind,
        LoadKind::PreTension {
            value: 0.01,
            by_displacement: true,
            lock: false,
            ..
        }
    ));
    // A step that takes no forces is skipped: the preload goes into the next one.
    model.steps[1].bolt_changes.clear();
    model.steps.insert(0, Step::new_frequency("Modes"));
    assert_eq!(model.bolt_action(&model.bolts[0], 0), None);
    assert_eq!(
        model.bolt_action(&model.bolts[0], 1),
        Some(plx_model::BoltAction::Load(1000.0))
    );
    assert_eq!(
        model.bolt_action(&model.bolts[0], 2),
        Some(plx_model::BoltAction::Lock)
    );
}

/// The preload of the bolt across two blocks clamped at their ends goes through the blocks
/// in full, and stays when the bolt is locked in the next step. The bonded contact region on
/// the same faces would make CalculiX loop forever if it were written too.
#[test]
fn calculix_preloads_the_bolt_across_two_blocks() {
    let force = 1000.0;
    let (mesh, model, bottom) = bolted_blocks(force);
    let text = write_inp(&mesh, &model, "").unwrap();
    let Some((frd, dat)) = run_ccx_frd_dat("schraube", &text) else {
        return;
    };
    let reaction: f64 = (bottom.iter())
        .map(|&node| node_value(&frd, "FORC", "F3", node))
        .sum();
    assert!((reaction + force).abs() < 1e-3 * force, "{reaction}");
    // The bolt force is printed under the bolt's name, in the preload step and while locked.
    let names: Vec<&str> = dat.sets.iter().map(|s| s.name.as_str()).collect();
    let bolt = (dat.sets.iter())
        .find(|s| s.name == "BOLT-1")
        .unwrap_or_else(|| panic!("no bolt force in {names:?}"));
    let forces = &bolt
        .field("FORCES")
        .unwrap()
        .component("RF1")
        .unwrap()
        .entries[0];
    let steps: Vec<u32> = bolt.rows.iter().map(|r| r.0).collect();
    assert_eq!(steps, [1, 2], "{:?}", bolt.rows);
    for value in &forces.values {
        assert!((value - force).abs() < 1e-3 * force, "{:?}", forces.values);
    }
}

/// A bolt meshed in one piece is cut at the faces inside it, without a tie.
#[test]
fn calculix_preloads_a_bolt_cut_inside_its_mesh() {
    let force = 1000.0;
    let (mesh, mut model) = pre_tension(force, false);
    let LoadKind::PreTension { .. } = model.steps[0].loads[0].kind else {
        unreachable!()
    };
    let faces = model.steps[0].loads.remove(0).region;
    let mut bolt = plx_model::Bolt::new("Bolt-1");
    bolt.side_a = faces;
    bolt.preload = plx_model::Preload::Force(force);
    model.bolts.push(bolt);
    let resolved = model.resolve_bolt(&model.bolts[0], &mesh).unwrap();
    assert_eq!(resolved.tied_to, None);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(!text.contains("*Tie"), "{text}");
    let Some(frd) = run_ccx("schraube-ganz", &text) else {
        return;
    };
    let reaction: f64 = (mesh.node_sets["FIX"].iter())
        .map(|&node| node_value(&frd, "FORC", "F1", node))
        .sum();
    assert!((reaction.abs() - force).abs() < 1e-3 * force, "{reaction}");
}

#[test]
fn the_model_check_reports_what_keeps_a_bolt_from_working() {
    use plx_model::{MeshCheck, ModelItem, Problem};
    let problems = |model: &FeModel, mesh: &FeMesh| -> Vec<(ModelItem, Problem)> {
        (model
            .check(mesh, &MeshCheck::new(mesh, model.properties.space))
            .into_iter())
        .map(|f| (f.item, f.problem))
        .collect()
    };
    let (mesh, mut model, _) = bolted_blocks(1000.0);
    assert_eq!(problems(&model, &mesh), []);
    model.bolts[0].preload = plx_model::Preload::Force(0.0);
    assert_eq!(
        problems(&model, &mesh),
        [(ModelItem::Bolt(0), Problem::BoltWithoutPreload)]
    );
    model.bolts[0].preload = plx_model::Preload::Force(1000.0);
    let mut tie = plx_model::Tie::new("Tie-1");
    tie.master = model.bolts[0].side_a.clone();
    tie.slave = model.bolts[0].side_b.clone();
    model.ties.push(tie);
    assert!(problems(&model, &mesh).contains(&(ModelItem::Bolt(0), Problem::BoltFacesUsedTwice)));
    model.ties.clear();
    model.bolts[0].side_a = Region::Faces(Vec::new());
    model.bolts[0].side_b = Region::Faces(Vec::new());
    assert!(problems(&model, &mesh).contains(&(ModelItem::Bolt(0), Problem::InvalidBolt)));
}

#[test]
fn a_tie_writes_its_surfaces_slave_first() {
    let (mesh, lower, upper) = two_blocks(0.0);
    let mut model = stacked_blocks(&mesh);
    let mut tie = plx_model::Tie::new("Tie-1");
    tie.master = Region::Faces(lower);
    tie.slave = Region::Faces(upper);
    tie.position_tolerance = Some(0.5);
    model.ties.push(tie);
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*Surface, Name=Internal_Selection-1_Tie-1_Master, Type=Element\n",
        "*Elset, Elset=Internal-1_Internal_Selection-1_Tie-1_Slave_S1\n101, 102, 103, 104\n",
        "*Tie, Name=Tie-1, Position tolerance=0.5\n\
         Internal_Selection-1_Tie-1_Slave, Internal_Selection-1_Tie-1_Master\n",
    ] {
        assert!(text.contains(line), "{line:?} missing in\n{text}");
    }
    let constraints = top_title(&model_keywords(&mesh, &model, "").unwrap(), "Constraints");
    assert_eq!(constraints, 11);
}

#[test]
fn surface_interactions_and_contact_pairs_are_written_like_prepomax() {
    let (mesh, lower, upper) = two_blocks(0.0);
    let mut model = stacked_blocks(&mesh);
    model.surface_interactions.push(SurfaceInteraction {
        name: "Surface_Interaction-1".into(),
        properties: vec![
            InteractionProperty::SurfaceBehavior(SurfaceBehavior::Linear {
                k: 1e7,
                sigma_inf: 2.86,
                c0: None,
            }),
            InteractionProperty::Friction(plx_model::Friction {
                coefficient: 0.2,
                stick_slope: Some(5000.0),
            }),
            InteractionProperty::GapConductance(GapConductance::Constant(3.0)),
        ],
    });
    let mut pair = ContactPair::new("Contact_Pair-1", "Surface_Interaction-1");
    pair.master = Region::Faces(lower);
    pair.slave = Region::Faces(upper);
    pair.method = ContactMethod::NodeToSurface;
    pair.small_sliding = true;
    pair.adjust = true;
    model.contact_pairs.push(pair.clone());
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*Surface interaction, Name=Surface_Interaction-1\n\
         *Surface behavior, Pressure-overclosure=Linear\n10000000, 2.86\n\
         *Friction\n0.2, 5000\n*Gap conductance\n3\n",
        "*Contact pair, Interaction=Surface_Interaction-1, Type=Node to surface, \
         Small sliding, Adjust=0\n\
         Internal_Selection-1_Contact_Pair-1_Slave, Internal_Selection-1_Contact_Pair-1_Master\n",
    ] {
        assert!(text.contains(line), "{line:?} missing in\n{text}");
    }
    // Deactivated, the pair keeps its place as a comment and gets no surfaces.
    model.contact_pairs[0].active = false;
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("** Name: Contact_Pair-1: Deactivated\n"));
    assert!(!text.contains("Contact_Pair-1_Master"));
    // A pair needs its interaction.
    model.contact_pairs[0] = ContactPair::new("Contact_Pair-1", "Gone");
    assert!(matches!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::UnknownInteraction { .. })
    ));
}

/// Sum of a component of a nodal field over nodes.
fn node_sum(frd: &FrdImport, field: &str, component: &str, nodes: &[NodeId]) -> f64 {
    nodes
        .iter()
        .map(|&n| node_value(frd, field, component, n))
        .sum()
}

#[test]
fn calculix_carries_the_load_across_a_tie() {
    // A small gap within the position tolerance; adjust closes it.
    let (mesh, lower, upper) = two_blocks(0.01);
    let mut model = stacked_blocks(&mesh);
    let mut tie = plx_model::Tie::new("Tie-1");
    tie.master = Region::Faces(lower);
    tie.slave = Region::Faces(upper);
    tie.position_tolerance = Some(0.05);
    model.ties.push(tie);
    let Some(frd) = run_ccx("tie", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let bottom: Vec<NodeId> = (1..=9).collect();
    let reaction = node_sum(&frd, "FORC", "F3", &bottom);
    // 10 MPa on 10 x 10 mm.
    assert!((reaction - 1000.0).abs() < 1.0, "{reaction}");
    // The tied faces move together: middle node of the lower top and the upper bottom.
    let below = node_value(&frd, "DISP", "U3", 14);
    let above = node_value(&frd, "DISP", "U3", 105);
    assert!(
        (below - above).abs() < 1e-3 * below.abs(),
        "{below} != {above}"
    );
}

#[test]
fn calculix_carries_the_load_across_a_contact() {
    // PrePoMax's default hard contact and a linear spring.
    let linear = SurfaceBehavior::Linear {
        k: 1e7,
        sigma_inf: 2.86,
        c0: None,
    };
    for (name, behavior) in [("hart", SurfaceBehavior::Hard), ("linear", linear)] {
        let (mesh, lower, upper) = two_blocks(0.0);
        let mut model = stacked_blocks(&mesh);
        // Held sideways at its top, the upper block rests on the lower one only by contact.
        let top: Vec<NodeId> = (119..=127).collect();
        model.steps[0].boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Side-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(top),
            kind: each_node([Some(0.0), Some(0.0), None, None, None, None]),
            amplitude: None,
        });
        model.surface_interactions.push(SurfaceInteraction {
            name: "Surface_Interaction-1".into(),
            properties: vec![InteractionProperty::SurfaceBehavior(behavior)],
        });
        let mut pair = ContactPair::new("Contact_Pair-1", "Surface_Interaction-1");
        pair.master = Region::Faces(lower);
        pair.slave = Region::Faces(upper);
        model.contact_pairs.push(pair);
        let text = write_inp(&mesh, &model, "").unwrap();
        let Some(frd) = run_ccx(&format!("kontakt-{name}"), &text) else {
            return;
        };
        let bottom: Vec<NodeId> = (1..=9).collect();
        let reaction = node_sum(&frd, "FORC", "F3", &bottom);
        assert!((reaction - 1000.0).abs() < 10.0, "{name}: {reaction}");
    }
}

/// A tied contact carries shear too. Its stiffness along the surfaces is the stick slope of
/// the friction; without one CalculiX 2.23 does not take its default (half Young's modulus of
/// the first material) and gives up, so the writer puts in the stiffness of the tie.
#[test]
fn calculix_carries_shear_across_a_tied_contact_without_stick_slope() {
    let (mesh, lower, upper) = two_blocks(0.0);
    let mut model = stacked_blocks(&mesh);
    // The top of the upper block is pushed sideways.
    let top: Vec<NodeId> = (110..=118).collect();
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Shift-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(top.clone()),
        kind: BoundaryKind::displacement([Some(0.001), None, None, None, None, None]),
        amplitude: None,
    });
    model.surface_interactions.push(SurfaceInteraction {
        name: "Tied".into(),
        properties: vec![
            InteractionProperty::SurfaceBehavior(SurfaceBehavior::Tied { k: 1e7 }),
            InteractionProperty::Friction(plx_model::Friction::default()),
        ],
    });
    let mut pair = ContactPair::new("Contact_Pair-1", "Tied");
    pair.master = Region::Faces(lower);
    pair.slave = Region::Faces(upper);
    model.contact_pairs.push(pair);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Friction\n0.1, 10000000\n"), "{text}");
    let Some(frd) = run_ccx("kontakt-tied", &text) else {
        return;
    };
    let bottom: Vec<NodeId> = (1..=9).collect();
    let shear = node_sum(&frd, "FORC", "F1", &bottom);
    let pushed = node_sum(&frd, "FORC", "F1", &top);
    assert!(shear < -10.0, "{shear}");
    assert!(
        (shear + pushed).abs() < 1e-3 * pushed.abs(),
        "{shear} {pushed}"
    );
}

/// The stacked blocks with a contact pair of the upper block's bottom on the lower block's top,
/// its surface interaction made of `properties`.
fn blocks_in_contact(
    properties: Vec<InteractionProperty>,
    method: ContactMethod,
) -> (FeMesh, FeModel) {
    let (mesh, lower, upper) = two_blocks(0.0);
    let mut model = stacked_blocks(&mesh);
    model.surface_interactions.push(SurfaceInteraction {
        name: "Surface_Interaction-1".into(),
        properties,
    });
    let mut pair = ContactPair::new("Contact_Pair-1", "Surface_Interaction-1");
    pair.master = Region::Faces(lower);
    pair.slave = Region::Faces(upper);
    pair.method = method;
    model.contact_pairs.push(pair);
    (mesh, model)
}

/// CalculiX makes a hard contact linear with K = 50 E and takes E/2 as stick slope, both not
/// stiffnesses per length: in SI units they are 1000 times softer than in mm. The writer puts
/// in K = 10 E/h (h the mean edge length of the slave faces, 5 here) and K/100.
#[test]
fn hard_contact_and_stick_slope_are_written_per_element_size() {
    let hard = InteractionProperty::SurfaceBehavior(SurfaceBehavior::Hard);
    let friction = InteractionProperty::Friction(plx_model::Friction::default());
    let (mesh, model) = blocks_in_contact(
        vec![hard.clone(), friction.clone()],
        ContactMethod::SurfaceToSurface,
    );
    let text = write_inp(&mesh, &model, "").unwrap();
    let expected = "*Surface interaction, Name=Surface_Interaction-1\n\
                    ** Hard contact as linear, K = 10 E/h, E = 210000, h = 5\n\
                    *Surface behavior, Pressure-overclosure=Linear\n420000, 3\n\
                    *Friction\n0.1, 4200\n";
    assert!(text.contains(expected), "{text}");
    // A linear behavior's own K sets the stick slope; the user's stick slope stays.
    let linear = InteractionProperty::SurfaceBehavior(SurfaceBehavior::Linear {
        k: 1e6,
        sigma_inf: 2.86,
        c0: None,
    });
    let (mesh, model) = blocks_in_contact(
        vec![linear, friction.clone()],
        ContactMethod::SurfaceToSurface,
    );
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Friction\n0.1, 10000\n"), "{text}");
    // Mortar contact is truly hard without a surface behavior.
    let (mesh, model) = blocks_in_contact(vec![hard.clone()], ContactMethod::Mortar);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(!text.contains("*Surface behavior"), "{text}");
    // Without contact pairs nothing scales the stiffness: CalculiX's defaults.
    let (mesh, mut model) = blocks_in_contact(vec![hard, friction], ContactMethod::Massless);
    model.contact_pairs.clear();
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Surface behavior, Pressure-overclosure=Hard\n*Friction\n0.1, 105000\n"),
        "{text}"
    );
}

/// The same model in mm and in SI units (as PrePoLix writes) gives the same contact
/// penetration and the same slip of a tied contact.
#[test]
fn calculix_gives_the_same_contact_in_mm_and_in_m() {
    let hard = vec![InteractionProperty::SurfaceBehavior(SurfaceBehavior::Hard)];
    let tied = vec![
        InteractionProperty::SurfaceBehavior(SurfaceBehavior::Tied { k: 1e7 }),
        InteractionProperty::Friction(plx_model::Friction::default()),
    ];
    // Middle node of the lower block's top and of the upper block's bottom.
    let (below, above) = (14, 105);
    for (name, properties) in [("hard", hard), ("tied", tied)] {
        let (mesh, mut model) = blocks_in_contact(properties, ContactMethod::SurfaceToSurface);
        // 100 MPa shear on the top of the upper block, 10 kN on 10 x 10 mm.
        model.steps[0].loads.push(Load {
            distribution: None,
            name: "Shear-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes((110..=118).collect()),
            kind: LoadKind::force([1e4, 0.0, 0.0]),
            amplitude: None,
            factor_amplitude: None,
        });
        if name == "hard" {
            // Held sideways at its top, the upper block rests on the lower one by contact.
            model.steps[0].loads.pop();
            model.steps[0].boundary_conditions.push(BoundaryCondition {
                distribution: None,
                name: "Side-1".into(),
                active: true,
                visible: true,
                region: Region::Nodes((110..=118).collect()),
                kind: each_node([Some(0.0), Some(0.0), None, None, None, None]),
                amplitude: None,
            });
        }
        model.properties.units = UnitSystem::MmTonSC;
        let mut model_m = model.clone();
        let mut mesh_m = mesh.clone();
        model_m.convert_units(UnitSystem::MKgSC);
        mesh_m.scale(1e-3);
        let mut gaps = Vec::new();
        for (unit, mesh, model) in [("mm", &mesh, &model), ("m", &mesh_m, &model_m)] {
            let text = write_inp(mesh, model, "").unwrap();
            let Some(frd) = run_ccx(&format!("kontakt-{name}-{unit}"), &text) else {
                return;
            };
            let component = if name == "hard" { "U3" } else { "U1" };
            let gap = node_value(&frd, "DISP", component, above)
                - node_value(&frd, "DISP", component, below);
            let elastic = node_value(&frd, "DISP", component, below);
            let to_mm = if unit == "m" { 1e3 } else { 1.0 };
            gaps.push((gap * to_mm, elastic * to_mm));
        }
        let [(mm, elastic), (m, _)] = gaps[..] else {
            unreachable!()
        };
        assert!(
            (m - mm).abs() < 1e-3 * mm.abs(),
            "{name}: {mm} mm != {m} mm"
        );
        // Penetration and slip are small against the deformation of the lower block.
        assert!(
            mm.abs() < 0.2 * elastic.abs(),
            "{name}: {mm} against {elastic}"
        );
        eprintln!("{name}: gap {mm} mm, lower block {elastic} mm");
    }
}

/// Mortar contact without a surface behavior is truly hard.
#[test]
fn calculix_carries_the_load_across_a_true_hard_mortar_contact() {
    let hard = vec![InteractionProperty::SurfaceBehavior(SurfaceBehavior::Hard)];
    let (mesh, mut model) = blocks_in_contact(hard, ContactMethod::Mortar);
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Side-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes((110..=118).collect()),
        kind: each_node([Some(0.0), Some(0.0), None, None, None, None]),
        amplitude: None,
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    let Some(frd) = run_ccx("kontakt-mortar", &text) else {
        return;
    };
    let bottom: Vec<NodeId> = (1..=9).collect();
    let reaction = node_sum(&frd, "FORC", "F3", &bottom);
    assert!((reaction - 1000.0).abs() < 10.0, "{reaction}");
}

/// Plane strain: a strip of 4 x 1 resting on another of the same size only by contact on
/// their edges carries the pressure on its top into the supports of the lower one.
#[test]
fn calculix_carries_the_load_across_a_contact_in_2d() {
    let (width, pressure) = (4.0, 50.0);
    let mut mesh = rectangle(0.0, width, 1.0, 4, 1);
    // The upper strip: the same rectangle with own nodes and elements, one higher.
    let upper = rectangle(0.0, width, 1.0, 4, 1);
    for (&id, c) in upper.node_ids().iter().zip(upper.coords()) {
        mesh.set_node(id + 1000, [c[0], c[1] + 1.0, c[2]]);
    }
    for element in upper.elements() {
        let mut element = element.clone();
        element.id += 100;
        element.nodes.iter_mut().for_each(|n| *n += 1000);
        mesh.add_element(element).unwrap();
    }
    mesh.parts.push(plx_mesh::Part {
        name: "UPPER".into(),
        elements: (101..=104).collect(),
    });
    let on_line = |y: f64, upper: bool| -> Vec<(ElementId, u8)> {
        (edges_at(&mesh, 1, y).into_iter())
            .filter(|&(element, _)| (element > 100) == upper)
            .collect()
    };
    let (lower_top, upper_bottom, upper_top) =
        (on_line(1.0, false), on_line(1.0, true), on_line(2.0, true));
    let bottom = nodes_at(&mesh, 1, 0.0);
    // Held sideways at its top, the upper strip rests on the lower one only by contact.
    let top: Vec<NodeId> = (nodes_at(&mesh, 1, 2.0).into_iter())
        .filter(|&n| n > 1000)
        .collect();
    let mut model = plane_model(
        ModelSpace::PlaneStrain,
        1.0,
        vec![(bottom.clone(), FIX_Y), (vec![1], FIX_X), (top, FIX_X)],
        LoadKind::Pressure(pressure),
        Region::Faces(upper_top),
    );
    model.sections[0].region = Region::Parts(vec!["PLATE".into(), "UPPER".into()]);
    model.surface_interactions.push(SurfaceInteraction {
        name: "Surface_Interaction-1".into(),
        properties: vec![InteractionProperty::SurfaceBehavior(SurfaceBehavior::Hard)],
    });
    let mut pair = ContactPair::new("Contact_Pair-1", "Surface_Interaction-1");
    pair.master = Region::Faces(lower_top);
    pair.slave = Region::Faces(upper_bottom);
    model.contact_pairs.push(pair);
    let Some(frd) = run_ccx("kontakt-2d", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let reaction = node_sum(&frd, "FORC", "F2", &bottom);
    let expected = pressure * width;
    assert!(
        (reaction - expected).abs() < 0.01 * expected,
        "{reaction} != {expected}"
    );
}

/// Two steel blocks of 1 x 1 x 1 hexahedra, `a` from z = 0 to 1 and `b` from z = 1.1 to 2.1,
/// with `n` elements per edge each. Returns the mesh and the bottom and top faces of
/// each block.
struct Blocks {
    mesh: FeMesh,
    a_bottom: Vec<(ElementId, u8)>,
    a_top: Vec<(ElementId, u8)>,
    b_bottom: Vec<(ElementId, u8)>,
    b_top: Vec<(ElementId, u8)>,
}

fn blocks(na: u32, nb: u32) -> Blocks {
    use plx_mesh::{Element, ElementShape, Part};
    let mut mesh = FeMesh::default();
    let mut next_node = 1;
    let mut next_element = 1;
    let mut block = |mesh: &mut FeMesh, n: u32, z0: f64, name: &str| {
        let mut ids = BTreeMap::new();
        for k in 0..=n {
            for j in 0..=n {
                for i in 0..=n {
                    let h = 1.0 / f64::from(n);
                    mesh.set_node(
                        next_node,
                        [f64::from(i) * h, f64::from(j) * h, z0 + f64::from(k) * h],
                    );
                    ids.insert((i, j, k), next_node);
                    next_node += 1;
                }
            }
        }
        let (mut bottom, mut top, mut elements) = (Vec::new(), Vec::new(), Vec::new());
        for k in 0..n {
            for j in 0..n {
                for i in 0..n {
                    let corner = |di, dj, dk| ids[&(i + di, j + dj, k + dk)];
                    let nodes = vec![
                        corner(0, 0, 0),
                        corner(1, 0, 0),
                        corner(1, 1, 0),
                        corner(0, 1, 0),
                        corner(0, 0, 1),
                        corner(1, 0, 1),
                        corner(1, 1, 1),
                        corner(0, 1, 1),
                    ];
                    mesh.add_element(Element {
                        id: next_element,
                        type_name: "C3D8".into(),
                        shape: ElementShape::Hex8,
                        nodes,
                    })
                    .unwrap();
                    if k == 0 {
                        bottom.push((next_element, 1));
                    }
                    if k == n - 1 {
                        top.push((next_element, 2));
                    }
                    elements.push(next_element);
                    next_element += 1;
                }
            }
        }
        mesh.parts.push(Part {
            name: name.into(),
            elements,
        });
        (bottom, top)
    };
    let (a_bottom, a_top) = block(&mut mesh, na, 0.0, "A");
    let (b_bottom, b_top) = block(&mut mesh, nb, 1.1, "B");
    Blocks {
        mesh,
        a_bottom,
        a_top,
        b_bottom,
        b_top,
    }
}

impl Blocks {
    /// The nodes of the bottom face of block b.
    fn b_bottom_nodes(&self) -> Vec<NodeId> {
        Region::Faces(self.b_bottom.clone()).nodes(&self.mesh)
    }
}

/// Steel on both blocks and one static step with a surface traction on top of block `b`,
/// block `a` held at its bottom.
fn blocks_model(
    blocks: &Blocks,
    constraints: Vec<plx_model::Constraint>,
    force: [f64; 3],
) -> FeModel {
    let mut step = Step::new_static("Step-1");
    step.boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Fixed-1".into(),
        active: true,
        visible: true,
        region: Region::Faces(blocks.a_bottom.clone()),
        kind: BoundaryKind::displacement([Some(0.0); 6]),
        amplitude: None,
    });
    step.loads.push(Load {
        distribution: None,
        name: "Surface_Traction-1".into(),
        active: true,
        visible: true,
        region: Region::Faces(blocks.b_top.clone()),
        kind: LoadKind::force(force),
        amplitude: None,
        factor_amplitude: None,
    });
    FeModel {
        materials: vec![Material {
            name: "Steel".into(),
            density: None,
            elastic: Some(Elastic {
                young: 210_000.0,
                poisson: 0.3,
            }),
            ..Default::default()
        }],
        sections: vec![Section {
            name: "Section-1".into(),
            material: "Steel".into(),
            region: Region::Parts(vec!["A".into(), "B".into()]),
            thickness: 1.0,
            kind: SectionKind::Solid,
        }],
        constraints,
        steps: vec![step],
        ..FeModel::default()
    }
}

/// Mean displacement component of the top nodes of block `b`.
fn mean_top(blocks: &Blocks, frd: &FrdImport, component: &str) -> f64 {
    let nodes = Region::Faces(blocks.b_top.clone()).nodes(&blocks.mesh);
    let sum: f64 = nodes
        .iter()
        .map(|&n| node_value(frd, "DISP", component, n))
        .sum();
    sum / nodes.len() as f64
}

/// A spring constraint on the region with the stiffness of the whole region.
fn spring(name: &str, region: Region, stiffness: [f64; 3]) -> plx_model::Spring {
    plx_model::Spring {
        region,
        stiffness,
        ..plx_model::Spring::new(name)
    }
}

#[test]
fn calculix_carries_a_block_on_springs_between_two_surfaces() {
    // Different meshes on the two sides: the springs end on points between nodes.
    let blocks = blocks(2, 3);
    let bearing = plx_model::Spring {
        master: Some(Region::Faces(blocks.a_top.clone())),
        ..spring(
            "Bearing-1",
            Region::Faces(blocks.b_bottom.clone()),
            [1000.0, 1000.0, 2000.0],
        )
    };
    let load = [0.0, 0.0, 100.0];
    let model = blocks_model(&blocks, vec![Constraint::Spring(bearing.clone())], load);
    let text = write_inp(&blocks.mesh, &model, "").unwrap();
    assert!(
        text.contains("*Element, Type=SPRING2, Elset=Bearing-1_All\n"),
        "{text}"
    );
    assert!(text.contains("*Equation\n"));
    let Some(frd) = run_ccx("feder-flaeche", &text) else {
        return;
    };
    // 100 N on 2000 N/mm, the steel itself barely stretches.
    let lift = mean_top(&blocks, &frd, "U3");
    assert!((lift - 0.05).abs() < 0.001, "{lift}");
    // Applied to the 16 nodes of the bottom individually: 32000 N/mm, so stiff that the
    // steel of the two blocks adds about 0.001 mm.
    let each = plx_model::Spring {
        each_node: true,
        ..bearing
    };
    let model = blocks_model(&blocks, vec![Constraint::Spring(each)], load);
    let frd = run_ccx(
        "feder-flaeche-einzeln",
        &write_inp(&blocks.mesh, &model, "").unwrap(),
    );
    let lift = mean_top(&blocks, &frd.unwrap(), "U3");
    let springs = 100.0 / 32000.0;
    assert!(lift > springs && lift < springs + 0.0012, "{lift}");
}

#[test]
fn calculix_carries_a_block_on_springs_to_ground() {
    let blocks = blocks(2, 2);
    // 1000 N/mm of the whole face, spread by area.
    let face = spring(
        "Spring-1",
        Region::Faces(blocks.b_bottom.clone()),
        [1000.0, 1000.0, 1000.0],
    );
    // 100 N/mm at each of the nine nodes of the bottom of block b.
    let nodes = Region::Nodes(blocks.b_bottom_nodes());
    let each = plx_model::Spring {
        each_node: true,
        ..spring("Spring-2", nodes.clone(), [0.0, 0.0, 100.0])
    };
    let springs = vec![Constraint::Spring(face), Constraint::Spring(each)];
    let model = blocks_model(&blocks, springs, [0.0, 0.0, 95.0]);
    let text = write_inp(&blocks.mesh, &model, "").unwrap();
    assert!(text.contains("*Element, Type=SPRING1, Elset=Spring-2_All\n"));
    assert!(
        text.contains("*Spring, Elset=Spring-2_DOF_3\n3\n100.\n"),
        "{text}"
    );
    let Some(frd) = run_ccx("federn", &text) else {
        return;
    };
    let lift = mean_top(&blocks, &frd, "U3");
    assert!((lift - 0.05).abs() < 0.001, "{lift}");
    // On nodes the stiffness is shared equally: 900 N/mm in all.
    let shared = spring("Spring-1", nodes, [0.0, 0.0, 900.0]);
    let model = blocks_model(&blocks, vec![Constraint::Spring(shared)], [0.0, 0.0, 90.0]);
    let text = write_inp(&blocks.mesh, &model, "").unwrap();
    assert!(
        text.contains("*Spring, Elset=Spring-1_DOF_3\n3\n100.\n"),
        "{text}"
    );
    let lift = mean_top(&blocks, &run_ccx("federn-knoten", &text).unwrap(), "U3");
    assert!((lift - 0.1).abs() < 0.001, "{lift}");
}

#[test]
fn springs_on_edges_are_spread_by_length() {
    let blocks = blocks(2, 2);
    // The bottom front edge of block b: two element edges, the middle node takes half.
    let bottom = blocks.b_bottom_nodes();
    let edge_nodes: Vec<NodeId> = (bottom.iter().copied())
        .filter(|&n| blocks.mesh.node(n).unwrap()[1] == 0.0)
        .collect();
    let edges = edge_nodes.windows(2).map(|w| [w[0], w[1]]).collect();
    let line = spring("Spring-1", Region::Edges(edges), [0.0, 0.0, 400.0]);
    let model = blocks_model(&blocks, vec![Constraint::Spring(line)], [0.0; 3]);
    let text = write_inp(&blocks.mesh, &model, "").unwrap();
    let [a, b, c] = edge_nodes[..] else {
        panic!("{edge_nodes:?}")
    };
    assert!(
        text.contains(&format!("Spring-1_{a}_DOF_3\n3\n100.\n")),
        "{text}"
    );
    assert!(
        text.contains(&format!("Spring-1_{b}_DOF_3\n3\n200.\n")),
        "{text}"
    );
    assert!(
        text.contains(&format!("Spring-1_{c}_DOF_3\n3\n100.\n")),
        "{text}"
    );
}

#[test]
fn springs_of_a_2d_model_have_no_z_direction() {
    let blocks = blocks(2, 2);
    let region = Region::Faces(blocks.b_bottom.clone());
    let spring = Constraint::Spring(spring("Spring-1", region, [1.0, 2.0, 3.0]));
    let mut model = blocks_model(&blocks, vec![spring], [0.0, 0.0, 0.0]);
    let text = write_inp(&blocks.mesh, &model, "").unwrap();
    assert!(text.contains("_DOF_3"), "{text}");
    // ccx crashes on a spring in a degree of freedom above 2 in a 2D model.
    model.properties.space = ModelSpace::PlaneStress;
    let text = write_inp(&blocks.mesh, &model, "").unwrap();
    assert!(text.contains("_DOF_2"), "{text}");
    assert!(!text.contains("_DOF_3"), "{text}");
}

#[test]
fn calculix_compression_only_support_takes_only_pressure() {
    use plx_model::CompressionOnly;
    let blocks = blocks(2, 2);
    let support = |force: [f64; 3], edit: &dyn Fn(&mut CompressionOnly)| {
        let springs = Constraint::Spring(spring(
            "Spring-1",
            Region::Faces(blocks.b_bottom.clone()),
            [1000.0, 1000.0, 1000.0],
        ));
        let mut gaps = CompressionOnly {
            name: "Compression_Only-1".into(),
            active: true,
            visible: true,
            region: Region::Faces(blocks.b_bottom.clone()),
            clearance: 0.0,
            spring_stiffness: None,
            tensile_force: None,
            each_node: false,
            flipped: false,
            offset: 0.0,
            nonlinear: true,
        };
        edit(&mut gaps);
        let model = blocks_model(
            &blocks,
            vec![springs, Constraint::CompressionOnly(gaps)],
            force,
        );
        write_inp(&blocks.mesh, &model, "").unwrap()
    };
    let up = [0.0, 0.0, 100.0];
    let down = [0.0, 0.0, -100.0];
    let pulled = support(up, &|_| {});
    assert!(pulled.contains("*Element, Type=GAPUNI\n"));
    assert!(pulled.contains("*Plastic\n"));
    let Some(frd) = run_ccx("druck-nur-zug", &pulled) else {
        return;
    };
    // Pulled away, only the springs hold the block.
    let lift = mean_top(&blocks, &frd, "U3");
    assert!((lift - 0.1).abs() < 0.002, "{lift}");
    let frd = run_ccx("druck-nur-druck", &support(down, &|_| {})).unwrap();
    // Pushed down, the gaps carry the load.
    let sink = mean_top(&blocks, &frd, "U3");
    assert!(sink.abs() < 0.002, "{sink}");

    // Flipped, the support pushes the bottom face up from inside the block: it holds the
    // block pulled up and lets it sink.
    let flip = |c: &mut CompressionOnly| c.flipped = true;
    let frd = run_ccx("druck-nur-umgedreht-zug", &support(up, &flip)).unwrap();
    let lift = mean_top(&blocks, &frd, "U3");
    assert!(lift.abs() < 0.002, "{lift}");
    let frd = run_ccx("druck-nur-umgedreht-druck", &support(down, &flip)).unwrap();
    let sink = mean_top(&blocks, &frd, "U3");
    assert!((sink + 0.1).abs() < 0.002, "{sink}");

    // A stiffness given is that of the whole face, spread over its nodes: 1000 of the gaps
    // and 1000 of the springs.
    let soft = |c: &mut CompressionOnly| c.spring_stiffness = Some(1000.0);
    let text = support(down, &soft);
    let frd = run_ccx("druck-nur-verteilt", &text).unwrap();
    let sink = mean_top(&blocks, &frd, "U3");
    assert!((sink + 0.05).abs() < 0.001, "{sink}");
    // Applied to the 9 nodes individually, each gap has the whole stiffness. The gaps of
    // the flat face then share one section, as CalculiX makes a material of each.
    let each = |c: &mut CompressionOnly| {
        c.spring_stiffness = Some(1000.0);
        c.each_node = true;
    };
    let text = support(down, &each);
    assert_eq!(text.matches(", , 1000, ").count(), 1, "{text}");
    assert_eq!(text.matches("*Gap, Elset=").count(), 1, "{text}");
    let gaps = text.split("*Element, Type=GAPUNI\n").nth(1).unwrap();
    assert_eq!(gaps.lines().take_while(|l| !l.starts_with('*')).count(), 9);
    let frd = run_ccx("druck-nur-einzeln", &text).unwrap();
    let sink = mean_top(&blocks, &frd, "U3");
    assert!((sink + 0.01).abs() < 0.001, "{sink}");
}

/// A straight chain of `n` line elements along `axis` from 0 to `length`, numbered from 1;
/// quadratic ones have a midside node.
fn line_chain(n: u32, axis: usize, length: f64, quadratic: bool) -> FeMesh {
    use plx_mesh::{Element, ElementShape, Part};
    let mut mesh = FeMesh::default();
    let per_element = if quadratic { 2 } else { 1 };
    let count = n * per_element;
    for i in 0..=count {
        let mut coords = [0.0; 3];
        coords[axis] = length * f64::from(i) / f64::from(count);
        mesh.set_node(i + 1, coords);
    }
    let mut elements = Vec::new();
    for e in 0..n {
        let first = e * per_element + 1;
        let (shape, type_name, nodes) = if quadratic {
            (
                ElementShape::Line3,
                "B32",
                vec![first, first + 1, first + 2],
            )
        } else {
            (ElementShape::Line2, "B31", vec![first, first + 1])
        };
        mesh.add_element(Element {
            id: e + 1,
            type_name: type_name.into(),
            shape,
            nodes,
        })
        .unwrap();
        elements.push(e + 1);
    }
    mesh.parts.push(Part {
        name: "BEAM".into(),
        elements,
    });
    mesh
}

/// Steel on part BEAM with the given section, node 1 fixed and `load` at the last node.
fn line_model(mesh: &FeMesh, kind: SectionKind, load: LoadKind) -> FeModel {
    let tip = *mesh.node_ids().last().unwrap();
    let mut step = Step::new_static("Step-1");
    step.boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Fixed-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(vec![1]),
        kind: BoundaryKind::displacement([Some(0.0); 6]),
        amplitude: None,
    });
    step.loads.push(Load {
        distribution: None,
        name: "Force-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(vec![tip]),
        kind: load,
        amplitude: None,
        factor_amplitude: None,
    });
    FeModel {
        cross_sections: line_cross_sections(),
        materials: vec![Material {
            name: "Steel".into(),
            density: None,
            elastic: Some(Elastic {
                young: 210_000.0,
                poisson: 0.3,
            }),
            conductivity: None,
            specific_heat: None,
            expansion: None,
            plastic: None,
            creep: None,
        }],
        sections: vec![Section {
            name: "Beam-1".into(),
            material: "Steel".into(),
            region: Region::Parts(vec!["BEAM".into()]),
            thickness: 1.0,
            kind,
        }],
        steps: vec![step],
        ..FeModel::default()
    }
}

/// The cross-sections of [`line_model`], by name.
fn line_cross_sections() -> Vec<CrossSection> {
    let profiles = [
        ("Rect", BeamProfile::Rect { a: 10.0, b: 5.0 }),
        (
            "Pipe",
            BeamProfile::Pipe {
                radius: 5.0,
                thickness: 1.0,
            },
        ),
        ("Circ", BeamProfile::Circ { radius: 2.0 }),
        ("Box", BeamProfile::ALL[3].clone()),
    ];
    (profiles.into_iter())
        .map(|(name, profile)| CrossSection {
            name: name.into(),
            profile,
        })
        .collect()
}

/// A beam section of a cross-section of [`line_cross_sections`].
fn beam(cross_section: &str) -> BeamSection {
    BeamSection {
        cross_section: cross_section.into(),
        ..BeamSection::DEFAULT
    }
}

fn rect_beam(orientation: BeamOrientation) -> SectionKind {
    SectionKind::Beam(BeamSection {
        orientation,
        ..beam("Rect")
    })
}

#[test]
fn beam_sections_type_their_elements_and_write_the_normal() {
    let mesh = line_chain(2, 0, 100.0, true);
    let model = line_model(
        &mesh,
        rect_beam(BeamOrientation::Direction([0.0, 1.0, 0.0])),
        LoadKind::force([0.0, -100.0, 0.0]),
    );
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*Element, Type=B32, Elset=BEAM\n1, 1, 2, 3\n2, 3, 4, 5\n",
        "*Beam section, Elset=Internal_Selection-1_Beam-1, Material=Steel, Section=RECT\n\
         10, 5\n0, 1, 0\n",
        "*Boundary\nInternal_Selection-1_Fixed-1, 1, 6, 0\n",
    ] {
        assert!(text.contains(line), "{line} missing in\n{text}");
    }
    // Pipes and boxes need B32R and carry the offsets.
    let mut model = model;
    model.sections[0].kind = SectionKind::Beam(BeamSection {
        offset: [0.5, 0.0],
        ..beam("Pipe")
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Element, Type=B32R, Elset=BEAM\n"), "{text}");
    // CalculiX 2.21 reads the set name of a pipe or box 20 characters wide, so it is short.
    assert!(text.contains("*Elset, Elset=Beam-1\n1, 2\n"), "{text}");
    assert!(
        text.contains(
            "*Beam section, Elset=Beam-1, Material=Steel, Section=PIPE, Offset1=0.5\n\
             5, 1\n0, 0, 1\n"
        ),
        "{text}"
    );
    // A circle is written by its diameter, as CalculiX reads it.
    model.sections[0].kind = SectionKind::Beam(beam("Circ"));
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("Section=CIRC\n4, 4\n0, 0, 1\n"), "{text}");
}

/// An I-beam is no beam section of CalculiX: it is written as a general section of the user
/// element U1, which bends like the textbook beam, and read back from its comment.
#[test]
fn general_profiles_are_written_for_user_element_u1() {
    let mesh = line_chain(10, 0, 1000.0, false);
    let i_beam = BeamProfile::I {
        h: 100.0,
        b: 50.0,
        tw: 4.0,
        tf: 6.0,
    };
    let mut model = line_model(
        &mesh,
        SectionKind::Beam(BeamSection {
            orientation: BeamOrientation::Direction([0.0, 0.0, 1.0]),
            ..beam("I")
        }),
        LoadKind::force([0.0, 0.0, 100.0]),
    );
    model.cross_sections.push(CrossSection {
        name: "I".into(),
        profile: i_beam.clone(),
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*User element, Type=U1, Nodes=2, Integration points=2, Maxdof=6\n\
         *Element, Type=U1, Elset=BEAM\n1, 1, 2\n",
        "Material=Steel, Section=GENERAL\n",
        "** Profile: I(h:100.0,b:50.0,tw:4.0,tf:6.0); Normal: 0, 0, 1\n",
    ] {
        assert!(text.contains(line), "{line} missing in\n{text}");
    }
    let import = read_inp_str(&text, None).unwrap();
    assert_eq!(import.cross_sections[0].profile, i_beam);
    assert_eq!(
        import.sections[0].kind,
        SectionKind::Beam(BeamSection {
            cross_section: import.cross_sections[0].name.clone(),
            orientation: BeamOrientation::Direction([0.0, 0.0, 1.0]),
            offset: [0.0; 2],
        })
    );
    assert_eq!(import.mesh.element(1).unwrap().type_name, "U1");
    // A tip force in the 1-direction (z) bends it about its weak axis, with shear in the web.
    let p = i_beam.properties();
    let g = 210_000.0 / 2.6;
    let expected = 100.0 * 1000.0_f64.powi(3) / (3.0 * 210_000.0 * p.i22)
        + 100.0 * 1000.0 / (i_beam.shear_factor() * g * p.area);
    if let Some(frd) = run_ccx("u1-beam", &text) {
        let sag = node_value(&frd, "DISP", "U3", 11);
        assert!(
            (sag - expected).abs() < 0.02 * expected,
            "{sag} != {expected}"
        );
    }
    // An angle is written about its principal axes: the normal turned about the beam.
    let angle = BeamProfile::L {
        h: 80.0,
        b: 40.0,
        t: 6.0,
    };
    model.cross_sections.last_mut().unwrap().profile = angle.clone();
    let text = write_inp(&mesh, &model, "").unwrap();
    let (sin, cos) = angle.principal_turn().sin_cos();
    let general = format!("Section=GENERAL\n{}, ", number(angle.data_line()[0]));
    let at = text.find(&general).expect(&text);
    let written: Vec<f64> = (text[at..].lines().nth(2).unwrap().split(','))
        .map(|v| v.trim().parse().unwrap())
        .collect();
    // The 2-direction is x cross z = -y.
    for (w, e) in written.iter().zip([0.0, -sin, cos]) {
        assert!((w - e).abs() < 1e-9, "{written:?}");
    }
    assert!(text.contains("; Normal: 0, 0, 1\n"), "{text}");
    let import = read_inp_str(&text, None).unwrap();
    assert_eq!(import.cross_sections[0].profile, angle);
    if let Some(frd) = run_ccx("u1-angle", &text) {
        // Skew bending: the force along u deflects by the inverse of the moment tensor.
        let p = angle.properties();
        let (uu, vv, uv) = (p.i22, p.i11, p.i12);
        let bending = 100.0 * 1000.0_f64.powi(3) / (3.0 * 210_000.0) * vv / (uu * vv - uv * uv);
        let expected = bending + 100.0 * 1000.0 / (angle.shear_factor() * g * p.area);
        let sag = node_value(&frd, "DISP", "U3", 11);
        assert!(
            (sag - expected).abs() < 0.02 * expected,
            "{sag} != {expected}"
        );
        // and sideways along v, the 2-direction -y.
        let side = -uv / vv * bending;
        let lateral = -node_value(&frd, "DISP", "U2", 11);
        assert!(
            (lateral - side).abs() < 0.02 * side.abs(),
            "{lateral} != {side}"
        );
    }
}

/// A sketched profile goes the way of the I-beam: a general section of U1, its polygons in
/// the comment, which may be longer than a line of CalculiX.
#[test]
fn sketched_profiles_are_written_for_user_element_u1() {
    let mesh = line_chain(10, 0, 1000.0, false);
    // A hollow rectangle with an off-centre hole, so that the centroid is not the centre.
    let sketch = BeamProfile::Sketch(plx_model::Sketch {
        loops: vec![
            vec![[0.0, 0.0], [50.0, 0.0], [50.0, 100.0], [0.0, 100.0]],
            vec![[5.0, 5.0], [45.0, 5.0], [45.0, 80.0], [5.0, 80.0]],
        ],
    });
    let mut model = line_model(
        &mesh,
        SectionKind::Beam(BeamSection {
            orientation: BeamOrientation::Direction([0.0, 0.0, 1.0]),
            ..beam("Sketch")
        }),
        LoadKind::force([0.0, 0.0, 100.0]),
    );
    model.cross_sections.push(CrossSection {
        name: "Sketch".into(),
        profile: sketch.clone(),
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("Section=GENERAL\n"), "{text}");
    let comment = (text.lines()).find(|l| l.starts_with("** Profile: Sketch("));
    assert!(comment.is_some_and(|c| c.len() > 132), "{text}");
    let import = read_inp_str(&text, None).unwrap();
    assert_eq!(import.cross_sections[0].profile, sketch);
    if let Some(frd) = run_ccx("u1-sketch", &text) {
        let p = sketch.properties();
        let g = 210_000.0 / 2.6;
        let expected = 100.0 * 1000.0_f64.powi(3) / (3.0 * 210_000.0 * p.i22)
            + 100.0 * 1000.0 / (sketch.shear_factor() * g * p.area);
        let sag = node_value(&frd, "DISP", "U3", 11);
        assert!(
            (sag - expected).abs() < 0.02 * expected,
            "{sag} != {expected}"
        );
    }
    // A round one with many points, far wider than CalculiX reads a line.
    let round = BeamProfile::Sketch(plx_model::Sketch::of(&BeamProfile::Circ { radius: 0.0123 }));
    model.cross_sections.last_mut().unwrap().profile = round.clone();
    let text = write_inp(&mesh, &model, "").unwrap();
    let import = read_inp_str(&text, None).unwrap();
    assert_eq!(import.cross_sections[0].profile, round);
    // CalculiX skips the comment however long it is.
    run_ccx("u1-round-sketch", &text);
}

#[test]
fn automatic_normals_split_a_frame_into_groups() {
    use plx_mesh::{Element, ElementShape};
    // An L: two elements along x, then two up along z.
    let mut mesh = line_chain(2, 0, 100.0, false);
    mesh.set_node(4, [100.0, 0.0, 50.0]);
    mesh.set_node(5, [100.0, 0.0, 100.0]);
    for (id, nodes) in [(3, vec![3, 4]), (4, vec![4, 5])] {
        mesh.add_element(Element {
            id,
            type_name: "B31".into(),
            shape: ElementShape::Line2,
            nodes,
        })
        .unwrap();
        mesh.parts[0].elements.push(id);
    }
    let model = line_model(
        &mesh,
        rect_beam(BeamOrientation::Automatic),
        LoadKind::force([100.0, 0.0, 0.0]),
    );
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*Elset, Elset=Internal_Selection-1_Beam-1\n1, 2\n",
        "*Elset, Elset=Internal_Selection-2_Beam-1\n3, 4\n",
        "*Beam section, Elset=Internal_Selection-1_Beam-1, Material=Steel, Section=RECT\n\
         10, 5\n0, 0, 1\n\
         *Beam section, Elset=Internal_Selection-2_Beam-1, Material=Steel, Section=RECT\n\
         10, 5\n1, 0, 0\n",
    ] {
        assert!(text.contains(line), "{line} missing in\n{text}");
    }
    // A given normal parallel to the columns is refused.
    let mut model = model;
    model.sections[0].kind = rect_beam(BeamOrientation::Direction([0.0, 0.0, 1.0]));
    let error = write_inp(&mesh, &model, "").unwrap_err();
    assert_eq!(
        error,
        WriteError::InvalidSection {
            item: "Beam-1".into(),
            reason: "The normal is parallel to the axis of element 3".into()
        }
    );
}

#[test]
fn sections_must_fit_their_elements() {
    let mesh = line_chain(2, 0, 100.0, false);
    let mut model = line_model(
        &mesh,
        SectionKind::Solid,
        LoadKind::force([0.0, -100.0, 0.0]),
    );
    let error = write_inp(&mesh, &model, "").unwrap_err();
    assert!(
        matches!(&error, WriteError::InvalidSection { item, reason }
            if item == "Beam-1" && reason.contains("line element")),
        "{error}"
    );
    model.sections[0].kind = SectionKind::Beam(beam("Box"));
    let error = write_inp(&mesh, &model, "").unwrap_err();
    assert!(
        matches!(&error, WriteError::InvalidSection { reason, .. } if reason.contains("B32R")),
        "{error}"
    );
    let (solid_mesh, mut solid_model) = cantilever(tip_force());
    solid_model.sections[0].kind = SectionKind::Truss { area: 1.0 };
    let error = write_inp(&solid_mesh, &solid_model, "").unwrap_err();
    assert!(
        matches!(&error, WriteError::InvalidSection { reason, .. }
            if reason.contains("not a line element")),
        "{error}"
    );
}

#[test]
fn trusses_are_written_as_t3d2_with_translations_only() {
    let mesh = line_chain(2, 0, 100.0, true);
    let model = line_model(
        &mesh,
        SectionKind::Truss { area: 50.0 },
        LoadKind::force([1000.0, 0.0, 0.0]),
    );
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        // The midside nodes of the quadratic lines are dropped.
        "*Element, Type=T3D2, Elset=BEAM\n1, 1, 3\n2, 3, 5\n",
        "*Solid section, Elset=Internal_Selection-1_Beam-1, Material=Steel\n50\n",
        "*Boundary\nInternal_Selection-1_Fixed-1, 1, 3, 0\n",
    ] {
        assert!(text.contains(line), "{line} missing in\n{text}");
    }
}

/// Two chains of `n` line elements each along x, part BEAM from 0 to 50 with nodes from 1
/// and part BEAM2 from 50 to 100 with nodes from 101, meeting at x = 50 with a node each.
fn two_chains(n: u32, quadratic: bool) -> FeMesh {
    use plx_mesh::{Element, ElementShape, Part};
    let mut mesh = line_chain(n, 0, 50.0, quadratic);
    let per_element = if quadratic { 2 } else { 1 };
    let count = n * per_element;
    for i in 0..=count {
        let x = 50.0 + 50.0 * f64::from(i) / f64::from(count);
        mesh.set_node(101 + i, [x, 0.0, 0.0]);
    }
    let mut elements = Vec::new();
    for e in 0..n {
        let first = 101 + e * per_element;
        let (shape, type_name, nodes) = if quadratic {
            (
                ElementShape::Line3,
                "B32",
                vec![first, first + 1, first + 2],
            )
        } else {
            (ElementShape::Line2, "B31", vec![first, first + 1])
        };
        mesh.add_element(Element {
            id: 101 + e,
            type_name: type_name.into(),
            shape,
            nodes,
        })
        .unwrap();
        elements.push(101 + e);
    }
    mesh.parts.push(Part {
        name: "BEAM2".into(),
        elements,
    });
    mesh
}

/// The two chains with their section on both parts and a node tie at x = 50.
fn tied_chains(n: u32, quadratic: bool, kind: SectionKind, load: LoadKind) -> (FeMesh, FeModel) {
    let mesh = two_chains(n, quadratic);
    let mut model = line_model(&mesh, kind, load);
    model.sections[0].region = Region::Parts(vec!["BEAM".into(), "BEAM2".into()]);
    let middle = n * if quadratic { 2 } else { 1 } + 1;
    model.node_ties.push(NodeTie {
        region: Region::Nodes(vec![middle, 101]),
        ..NodeTie::new("Node_Tie-1")
    });
    (mesh, model)
}

#[test]
fn node_ties_merge_the_nodes_or_hinge_beams_with_equations() {
    let (mesh, mut model) = tied_chains(
        2,
        false,
        rect_beam(BeamOrientation::Automatic),
        LoadKind::force([0.0, -100.0, 0.0]),
    );
    // Tied rigidly, the second beam starts at the first one's end node; the node set of a
    // boundary condition on the merged node names the node that stays.
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Held".into(),
        active: true,
        visible: true,
        amplitude: None,
        region: Region::Nodes(vec![101]),
        kind: BoundaryKind::displacement([None, None, Some(0.0), None, None, None]),
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*Element, Type=B31, Elset=BEAM2\n101, 3, 102\n102, 102, 103\n",
        "** Name: Node_Tie-1\n** Node 3 (merged)\n",
        "*Nset, Nset=Internal_Selection-1_Held\n3\n",
    ] {
        assert!(text.contains(line), "{line} missing in\n{text}");
    }
    assert!(!text.contains("*Node\n101,") && !text.contains("\n101, 50, 0, 0\n"));
    assert!(!text.contains("*Equation"));
    // A hinge between beams keeps the nodes and ties their translations.
    model.node_ties[0].rotations = false;
    let text = write_inp(&mesh, &model, "").unwrap();
    // Node 101 is held in z: that equation depends on node 3 instead, which CalculiX
    // would otherwise refuse ("dependent side of a MPC and a SPC").
    let mut expected: String = (1..=2)
        .map(|dof| format!("*Equation\n2\n101, {dof}, 1, 3, {dof}, -1\n"))
        .collect();
    expected.push_str("*Equation\n2\n3, 3, -1, 101, 3, 1\n");
    assert_eq!(
        crate::inp::resolved_overlaps(&write_inp(&mesh, &model, "").unwrap()),
        ["Node_Tie-1: 1 equation on another degree of freedom (the first one is held already)"]
    );
    assert!(
        text.contains(&format!("** Name: Node_Tie-1\n{expected}")) && !text.contains("101, 4, 1"),
        "{text}"
    );
    assert!(text.contains("*Element, Type=B31, Elset=BEAM2\n101, 101, 102\n"));
    // Trusses have no rotations: a hinge is the same as a rigid tie, one node.
    model.sections[0].kind = SectionKind::Truss { area: 50.0 };
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Element, Type=T3D2, Elset=BEAM2\n101, 3, 102\n"));
    assert!(!text.contains("*Equation"));
    // A deactivated tie is a comment, an empty one an error.
    model.node_ties[0] = NodeTie {
        active: false,
        ..NodeTie::new("Node_Tie-1")
    };
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("** Name: Node_Tie-1: Deactivated\n"));
    assert!(text.contains("*Element, Type=T3D2, Elset=BEAM2\n101, 101, 102\n"));
    model.node_ties[0] = NodeTie::new("Node_Tie-1");
    assert!(matches!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::EmptyRegion { .. })
    ));
}

/// Two beams of five elements each tied rigidly at their ends bend like one cantilever.
#[test]
fn calculix_bends_two_beams_tied_at_a_node_like_one() {
    let expected = cantilever_deflection(100.0, 100.0, 10.0 * 125.0 / 12.0);
    let (mesh, model) = tied_chains(
        5,
        false,
        rect_beam(BeamOrientation::Automatic),
        LoadKind::force([0.0, -100.0, 0.0]),
    );
    let Some(frd) = run_ccx("balken_node_tie", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let deflection = -min_u2(&frd);
    assert!(
        (deflection - expected).abs() < 0.005 * expected,
        "{deflection} instead of {expected}"
    );
}

/// Two beams clamped at their far ends and hinged together at x = 50 move together there,
/// but bend less stiffly than tied rigidly: a hinge carries no moment.
#[test]
fn calculix_hinges_two_beams_at_a_node() {
    let deflection = |rotations: bool| {
        let (mesh, mut model) = tied_chains(
            5,
            false,
            rect_beam(BeamOrientation::Automatic),
            LoadKind::force([0.0, -100.0, 0.0]),
        );
        model.node_ties[0].rotations = rotations;
        let step = &mut model.steps[0];
        step.boundary_conditions[0].region = Region::Nodes(vec![1, 106]);
        step.loads[0].region = Region::Nodes(vec![3]);
        let name = if rotations {
            "balken_starr"
        } else {
            "balken_gelenk"
        };
        let frd = run_ccx(name, &write_inp(&mesh, &model, "").unwrap())?;
        // The expanded nodes at x = 50 of both beams move alike; tied rigidly, the beams
        // share the node there and CalculiX expands it once.
        let at_joint: Vec<f64> = (frd.mesh.node_ids().iter().zip(frd.mesh.coords()))
            .filter(|(_, c)| (c[0] - 50.0).abs() < 1e-6)
            .map(|(&n, _)| node_value(&frd, "DISP", "U2", n))
            .collect();
        assert_eq!(
            at_joint.len(),
            if rotations { 4 } else { 8 },
            "{at_joint:?}"
        );
        let mean = at_joint.iter().sum::<f64>() / at_joint.len() as f64;
        assert!(
            at_joint
                .iter()
                .all(|u| (u - mean).abs() < 1e-3 * mean.abs()),
            "{at_joint:?}"
        );
        Some(-mean)
    };
    let (Some(hinged), Some(rigid)) = (deflection(false), deflection(true)) else {
        return;
    };
    assert!(hinged > 1.2 * rigid && rigid > 0.0, "{hinged} vs {rigid}");
}

/// Two trusses tied at their ends stretch like one bar of their whole length.
#[test]
fn calculix_stretches_two_trusses_tied_at_a_node_like_one() {
    let (mesh, mut model) = tied_chains(
        5,
        true,
        SectionKind::Truss { area: 50.0 },
        LoadKind::force([1000.0, 0.0, 0.0]),
    );
    // A straight chain of trusses is a mechanism sideways; hold the nodes in the axis.
    let nodes: Vec<NodeId> = (mesh.node_ids().iter().copied())
        .filter(|&n| n % 2 == 1)
        .collect();
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Sideways".into(),
        active: true,
        visible: true,
        amplitude: None,
        region: Region::Nodes(nodes),
        kind: BoundaryKind::displacement([None, Some(0.0), Some(0.0), None, None, None]),
    });
    let Some(frd) = run_ccx("stab_node_tie", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let expected = 1000.0 * 100.0 / (210_000.0 * 50.0);
    let stretch = node_value(&frd, "DISP", "U1", 111);
    assert!(
        (stretch - expected).abs() < 1e-6 * expected,
        "{stretch} instead of {expected}"
    );
}

/// Deflection of a cantilever of length `l` under a tip force `f` after beam theory.
fn cantilever_deflection(f: f64, l: f64, inertia: f64) -> f64 {
    f * l.powi(3) / (3.0 * 210_000.0 * inertia)
}

/// The most negative `U2` in the results: the tip of a beam bent down.
fn min_u2(frd: &FrdImport) -> f64 {
    let increment = frd.increments.last().unwrap();
    let u2 = increment.field("DISP").unwrap().component("U2").unwrap();
    f64::from(u2.values.iter().copied().fold(f32::MAX, f32::min))
}

#[test]
fn calculix_bends_a_rectangular_beam_like_beam_theory() {
    // Rectangle 10 high (1-direction z) and 5 wide, loaded in y: I = 10 * 5^3 / 12.
    let expected = cantilever_deflection(100.0, 100.0, 10.0 * 125.0 / 12.0);
    for (quadratic, tolerance) in [(false, 0.005), (true, 0.03)] {
        let mesh = line_chain(10, 0, 100.0, quadratic);
        let model = line_model(
            &mesh,
            rect_beam(BeamOrientation::Automatic),
            LoadKind::force([0.0, -100.0, 0.0]),
        );
        let name = if quadratic {
            "balken_b32"
        } else {
            "balken_b31"
        };
        let Some(frd) = run_ccx(name, &write_inp(&mesh, &model, "").unwrap()) else {
            return;
        };
        // CalculiX expands the beam to solids with new node numbers.
        assert!(frd.mesh.element_count() == 10 && frd.mesh.node(1).is_none());
        let deflection = -min_u2(&frd);
        assert!(
            (deflection - expected).abs() < tolerance * expected,
            "{name}: {deflection} instead of {expected}"
        );
    }
}

#[test]
fn a_rigid_remote_displacement_couples_beam_nodes_by_equations() {
    // CalculiX 2.23 stops on beam nodes in a *Rigid body ("nodes belonging to beam elements
    // must not be subject to a RIGID MPC"); they are coupled by equations instead.
    let mesh = line_chain(4, 0, 100.0, true);
    let tip = *mesh.node_ids().last().unwrap();
    let mut model = line_model(
        &mesh,
        rect_beam(BeamOrientation::Automatic),
        LoadKind::force([0.0, 0.0, 0.0]),
    );
    model.steps[0].loads.clear();
    // The point 20 above the tip, its rotations held: the tip moves like the point.
    let values = [None, Some(-1.0), None, Some(0.0), Some(0.0), Some(0.0)];
    let mut bc = remote_with(values, |r| r.location = Some([100.0, 0.0, 20.0]));
    bc.region = Region::Nodes(vec![tip]);
    model.steps[0].boundary_conditions.push(bc);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(!text.contains("*Rigid body"), "{text}");
    let (reference, rotation) = (tip + 1, tip + 2);
    // u_tip = u_point + theta x (0, 0, -20)
    for expected in [
        format!("*Equation\n3\n{tip}, 1, 1, {reference}, 1, -1, {rotation}, 2, 20\n"),
        format!("*Equation\n3\n{tip}, 2, 1, {reference}, 2, -1, {rotation}, 1, -20\n"),
        format!("*Equation\n2\n{tip}, 3, 1, {reference}, 3, -1\n"),
    ] {
        assert!(text.contains(&expected), "{expected}\n{text}");
    }
    if let Some(frd) = run_ccx("balken_remote_starr", &text) {
        let u2 = min_u2(&frd);
        assert!((u2 + 1.0).abs() < 1e-6, "{u2}");
    }

    // The same for a rigid body constraint on the tip.
    model.steps[0].boundary_conditions.truncate(1);
    model
        .reference_points
        .push(ReferencePoint::new("RP-1", [100.0, 0.0, 0.0]));
    let mut body = RigidBody::new("Rigid_Body-1", "RP-1");
    body.region = Region::Nodes(vec![tip]);
    model.constraints.push(Constraint::RigidBody(body));
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(!text.contains("*Rigid body"), "{text}");
    assert!(
        text.contains(&format!("*Equation\n2\n{tip}, 1, 1, {reference}, 1, -1\n")),
        "{text}"
    );
}

#[test]
fn a_rigid_body_keeps_its_solid_nodes_and_couples_beam_nodes_by_equations() {
    // The cantilever's tip face and a beam node beside it in one rigid body.
    let (mut mesh, mut model) = rigid_tip(None, None);
    let start = mesh.node_ids().iter().max().unwrap() + 1;
    mesh.set_node(start, [100.0, 5.0, 5.0]);
    mesh.set_node(start + 1, [150.0, 5.0, 5.0]);
    let id = mesh.elements().iter().map(|e| e.id).max().unwrap() + 1;
    mesh.add_element(plx_mesh::Element {
        id,
        type_name: "B31".into(),
        shape: plx_mesh::ElementShape::Line2,
        nodes: vec![start, start + 1],
    })
    .unwrap();
    let tip = Region::Surface("TIP".into()).nodes(&mesh);
    let Constraint::RigidBody(body) = &mut model.constraints[0] else {
        unreachable!()
    };
    body.region = Region::Nodes([tip.clone(), vec![start]].concat());
    let text = write_inp(&mesh, &model, "").unwrap();
    assert_eq!(text.matches("*Rigid body").count(), 1, "{text}");
    let set = text
        .split("*Rigid body, Nset=")
        .nth(1)
        .and_then(|rest| rest.split(',').next())
        .unwrap();
    let members = text
        .split(&format!("*Nset, Nset={set}\n"))
        .nth(1)
        .unwrap()
        .split('*')
        .next()
        .unwrap();
    assert!(!members.contains(&start.to_string()), "{members}");
    assert_eq!(text.matches("*Equation\n").count(), 3, "{text}");
    assert!(
        text.contains(&format!("*Equation\n2\n{start}, 1, 1, ")),
        "{text}"
    );
}

#[test]
fn calculix_bends_a_pipe_like_beam_theory() {
    let (r, t) = (5.0f64, 1.0f64);
    let inertia = std::f64::consts::PI * (r.powi(4) - (r - t).powi(4)) / 4.0;
    let expected = cantilever_deflection(100.0, 100.0, inertia);
    let mesh = line_chain(10, 0, 100.0, true);
    let model = line_model(
        &mesh,
        // The pipe of line_cross_sections.
        SectionKind::Beam(beam("Pipe")),
        LoadKind::force([0.0, -100.0, 0.0]),
    );
    let Some(frd) = run_ccx("rohr_b32r", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let deflection = -min_u2(&frd);
    assert!(
        (deflection - expected).abs() < 0.01 * expected,
        "{deflection} instead of {expected}"
    );
}

#[test]
fn calculix_stretches_a_truss_by_f_l_over_e_a() {
    let mesh = line_chain(5, 0, 100.0, true);
    let mut model = line_model(
        &mesh,
        SectionKind::Truss { area: 50.0 },
        LoadKind::force([1000.0, 0.0, 0.0]),
    );
    // A straight chain of trusses is a mechanism sideways; hold the nodes in the axis.
    let nodes: Vec<NodeId> = mesh
        .node_ids()
        .iter()
        .copied()
        .filter(|&n| n % 2 == 1)
        .collect();
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Sideways".into(),
        active: true,
        visible: true,
        region: Region::Nodes(nodes),
        kind: BoundaryKind::displacement([None, Some(0.0), Some(0.0), None, None, None]),
        amplitude: None,
    });
    let Some(frd) = run_ccx("stab_t3d2", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let expected = 1000.0 * 100.0 / (210_000.0 * 50.0);
    let stretch = node_value(&frd, "DISP", "U1", 11);
    assert!(
        (stretch - expected).abs() < 1e-6 * expected,
        "{stretch} instead of {expected}"
    );
}

/// The cantilever bar (x from 0 to 100, 5 x 5 cross-section) as a thermal model of steel in
/// mm, t, s: conductivity 50 W/(m K), specific heat 460 J/(kg K), expansion 1e-5 from 20, and
/// a step of the given kind.
fn thermal_bar(kind: StepKind, bcs: Vec<BoundaryCondition>, loads: Vec<Load>) -> (FeMesh, FeModel) {
    let (mesh, mut model) = cantilever(tip_force());
    let material = &mut model.materials[0];
    material.conductivity = Some(CONDUCTIVITY);
    material.specific_heat = Some(SPECIFIC_HEAT);
    material.expansion = Some(plx_model::Expansion {
        coefficient: 1e-5,
        zero_temperature: 20.0,
    });
    let step = &mut model.steps[0];
    step.field_outputs = match kind {
        StepKind::HeatTransfer(_) => FieldOutput::heat_transfer_defaults(),
        _ => FieldOutput::coupled_defaults(),
    };
    step.kind = kind;
    step.boundary_conditions = bcs;
    step.loads = loads;
    (mesh, model)
}

const CONDUCTIVITY: f64 = 50.0;
const SPECIFIC_HEAT: f64 = 4.6e8;

fn temperature(name: &str, region: Region, value: f64) -> BoundaryCondition {
    BoundaryCondition {
        distribution: None,
        name: name.into(),
        active: true,
        visible: true,
        region,
        kind: BoundaryKind::Temperature(value),
        amplitude: None,
    }
}

fn heat_load(name: &str, region: Region, kind: LoadKind) -> Load {
    Load {
        distribution: None,
        name: name.into(),
        active: true,
        visible: true,
        region,
        kind,
        amplitude: None,
        factor_amplitude: None,
    }
}

fn steady() -> StepKind {
    StepKind::HeatTransfer(HeatTransferStep::default())
}

/// Temperatures at the nodes of the cross-section at x.
fn temperatures_at(mesh: &FeMesh, frd: &FrdImport, x: f64) -> Vec<f64> {
    let nodes = nodes_at(mesh, 0, x);
    assert!(!nodes.is_empty());
    (nodes.iter())
        .map(|&n| node_value(frd, "NDTEMP", "T", n))
        .collect()
}

#[track_caller]
fn assert_all_close(values: &[f64], expected: f64, tolerance: f64) {
    for &v in values {
        assert!(
            (v - expected).abs() <= tolerance,
            "{v} != {expected} ({values:?})"
        );
    }
}

#[test]
fn writes_heat_transfer_keywords() {
    let (mesh, mut model) = thermal_bar(
        StepKind::HeatTransfer(HeatTransferStep {
            steady_state: false,
            deltmx: Some(5.0),
            ..HeatTransferStep::default()
        }),
        vec![
            temperature("Temperature-1", Region::NodeSet("FIX".into()), 20.0),
            BoundaryCondition {
                distribution: None,
                name: "Fixed-1".into(),
                active: true,
                visible: true,
                region: Region::NodeSet("FIX".into()),
                kind: BoundaryKind::displacement([Some(0.0); 6]),
                amplitude: None,
            },
        ],
        vec![
            heat_load(
                "Flux-1",
                Region::Nodes(vec![99]),
                LoadKind::ConcentratedFlux(3.0),
            ),
            heat_load(
                "Film-1",
                Region::Surface("TIP".into()),
                LoadKind::Film {
                    sink: 25.0,
                    coefficient: 0.01,
                },
            ),
            heat_load(
                "Body-1",
                Region::Parts(vec!["EALL".into()]),
                LoadKind::BodyFlux(0.5),
            ),
            tip_force(),
        ],
    );
    model.properties.absolute_zero = Some(-273.15);
    model.properties.stefan_boltzmann = Some(5.67e-11);
    model.initial_conditions.push(plx_model::InitialCondition {
        distribution: None,
        name: "Initial_Temperature-1".into(),
        active: true,
        region: Region::Parts(vec!["EALL".into()]),
        kind: plx_model::InitialConditionKind::Temperature(20.0),
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    for expected in [
        "*Physical constants, Absolute zero=-273.15, Stefan Boltzmann=0.0000000000567\n",
        "*Expansion, Zero=20\n0.00001\n*Conductivity\n50\n*Specific heat\n460000000\n",
        "** Name: Initial_Temperature-1\n*Initial conditions, Type=Temperature\n",
        "*Heat transfer, Deltmx=5\n",
        "** Name: Temperature-1\n*Boundary\nFIX, 11, 11, 20\n",
        // Displacements and forces do not act in a heat transfer step.
        "** Name: Fixed-1: Deactivated\n",
        "** Name: Force-1: Deactivated\n",
        "*Cflux, op=New\n*Dflux, op=New\n*Film, op=New\n",
        "*Cflux\nInternal_Selection-1_Flux-1, 11, 3\n",
        "*Dflux\nInternal_Selection-1_Body-1, BF, 0.5\n",
        "*Film\nInternal-1_TIP_S4, F4, 25, 0.01\n",
        "*Node file\nNT, RFL\n",
        "*El file\nHFL\n",
    ] {
        assert!(text.contains(expected), "{expected}\n{text}");
    }
    assert!(
        !text.contains("Cload") && !text.contains("Radiate"),
        "{text}"
    );
}

/// Steady conduction along the bar, held at 0 at x = 0: a heat flux q into the far end
/// gives T = q x / k.
#[test]
fn calculix_conducts_heat_along_a_bar() {
    let q = 1.0;
    let (mesh, model) = thermal_bar(
        steady(),
        vec![temperature(
            "Temperature-1",
            Region::NodeSet("FIX".into()),
            0.0,
        )],
        vec![heat_load(
            "Surface_Flux-1",
            Region::Surface("TIP".into()),
            LoadKind::SurfaceFlux(q),
        )],
    );
    let Some(frd) = run_ccx("waermeleitung", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    for x in [50.0, 100.0] {
        assert_all_close(&temperatures_at(&mesh, &frd, x), q * x / CONDUCTIVITY, 1e-6);
    }
    let hfl = node_value(&frd, "FLUX", "F1", 99);
    assert!((hfl.abs() - q).abs() < 1e-3, "{hfl}");
}

fn defined_temperature(name: &str, region: Region, value: f64) -> DefinedField {
    DefinedField {
        distribution: None,
        name: name.into(),
        active: true,
        region,
        kind: DefinedFieldKind::Temperature(value),
        amplitude: None,
    }
}

fn temperature_from_file(name: &str, file: &str, step: u32) -> DefinedField {
    DefinedField {
        distribution: None,
        name: name.into(),
        active: true,
        region: Region::Nodes(Vec::new()),
        kind: DefinedFieldKind::TemperatureFromFile {
            file: PathBuf::from(file),
            step,
        },
        amplitude: None,
    }
}

#[test]
fn a_defined_temperature_is_written_like_prepomax_does() {
    let (mesh, mut model) = thermal_bar(
        StepKind::Static(StaticStep::default()),
        Vec::new(),
        Vec::new(),
    );
    model.steps[0].defined_fields = vec![
        defined_temperature(
            "Defined_Temperature-1",
            Region::Parts(vec!["EALL".into()]),
            120.0,
        ),
        temperature_from_file("Defined_Temperature-2", "/results/heat run.frd", 2),
    ];
    model.steps[0].defined_fields[1].active = false;
    let text = write_inp(&mesh, &model, "").unwrap();
    let expected = "** Name: Defined_Temperature-1\n*Temperature\n\
                    Internal_Selection-1_Defined_Temperature-1, 120\n\
                    ** Name: Defined_Temperature-2: Deactivated\n";
    assert!(text.contains(expected), "{text}");
    model.steps[0].defined_fields[1].active = true;
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("** Name: Defined_Temperature-2\n*Temperature, File=heat run.frd, BStep=2\n"),
        "{text}"
    );
    assert_eq!(
        model.result_files(),
        [std::path::Path::new("/results/heat run.frd")]
    );
    // A thermal step solves for the temperatures and leaves the fields out.
    model.steps[0].kind = steady();
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("** Name: Defined_Temperature-1: Deactivated\n"),
        "{text}"
    );
    assert!(!text.contains("*Temperature"), "{text}");
    assert!(model.result_files().is_empty());
    model.steps[0].kind = StepKind::Static(StaticStep::default());
    model.steps[0].defined_fields[1].kind = DefinedFieldKind::TemperatureFromFile {
        file: PathBuf::from("a,b.frd"),
        step: 1,
    };
    assert!(matches!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::InvalidResultFile { .. })
    ));
}

/// A heat transfer run gives the bar a gradient from 20 at x = 0 to 120 at the tip; the
/// static run reads it from the frd file and the bar grows by alpha times the integral of
/// T - 20 over its length, 1e-5 * 100^2 / 2.
#[test]
fn calculix_strains_the_bar_by_the_temperatures_of_a_result_file() {
    let (mesh, heat) = thermal_bar(
        steady(),
        vec![
            temperature("Temperature-1", Region::NodeSet("FIX".into()), 20.0),
            temperature("Temperature-2", Region::Surface("TIP".into()), 120.0),
        ],
        Vec::new(),
    );
    let Some(dir) = ccx_dir("waermedehnung") else {
        return;
    };
    run_ccx_in(&dir, "heat", &write_inp(&mesh, &heat, "").unwrap());
    let (_, mut model) = thermal_bar(
        StepKind::Static(StaticStep::default()),
        Vec::new(),
        Vec::new(),
    );
    // CalculiX refuses a *Temperature without an initial temperature.
    model.initial_conditions.push(plx_model::InitialCondition {
        distribution: None,
        name: "Initial_Temperature-1".into(),
        active: true,
        region: Region::Parts(vec!["EALL".into()]),
        kind: plx_model::InitialConditionKind::Temperature(20.0),
    });
    // Held at x = 0 in x only, and against rigid motion at two nodes of that face, so the
    // bar expands freely: the corner holds y and z, the node along y from it holds z.
    let fix = nodes_at(&mesh, 0, 0.0);
    let at = |n: NodeId| mesh.node(n).unwrap();
    let corner = (fix.iter().copied())
        .min_by(|&a, &b| (at(a)[1] + at(a)[2]).total_cmp(&(at(b)[1] + at(b)[2])))
        .unwrap();
    let along_y = (fix.iter().copied())
        .filter(|&n| (at(n)[2] - at(corner)[2]).abs() < 1e-9 && n != corner)
        .max_by(|&a, &b| at(a)[1].total_cmp(&at(b)[1]))
        .unwrap();
    let hold = |name: &str, nodes: Vec<NodeId>, held: [bool; 3]| BoundaryCondition {
        distribution: None,
        name: name.into(),
        active: true,
        visible: true,
        region: Region::Nodes(nodes),
        kind: BoundaryKind::displacement([
            held[0].then_some(0.0),
            held[1].then_some(0.0),
            held[2].then_some(0.0),
            None,
            None,
            None,
        ]),
        amplitude: None,
    };
    model.steps[0].boundary_conditions = vec![
        hold("Axial", fix.clone(), [true, false, false]),
        hold("Corner", vec![corner], [false, true, true]),
        hold("Edge", vec![along_y], [false, false, true]),
    ];
    model.steps[0].defined_fields = vec![temperature_from_file(
        "Defined_Temperature-1",
        dir.join("heat.frd").to_str().unwrap(),
        1,
    )];
    let frd = run_ccx_in(&dir, "static", &write_inp(&mesh, &model, "").unwrap());
    remove_ccx_dir(&dir);
    assert_all_close(&temperatures_at(&mesh, &frd, 100.0), 120.0, 1e-6);
    let tip = nodes_at(&mesh, 0, 100.0);
    let growth: Vec<f64> = (tip.iter())
        .map(|&n| node_value(&frd, "DISP", "U1", n))
        .collect();
    assert_all_close(&growth, 1e-5 * 100.0 * 100.0 / 2.0, 1e-3);
}

/// Heat generated in the bar flows out at x = 0: T = Q (L x - x^2 / 2) / k.
#[test]
fn calculix_conducts_body_heat() {
    let (body, length) = (0.01, 100.0);
    let (mesh, model) = thermal_bar(
        steady(),
        vec![temperature(
            "Temperature-1",
            Region::NodeSet("FIX".into()),
            0.0,
        )],
        vec![heat_load(
            "Body_Flux-1",
            Region::Parts(vec!["EALL".into()]),
            LoadKind::BodyFlux(body),
        )],
    );
    let Some(frd) = run_ccx("koerperwaerme", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    for x in [50.0, 100.0] {
        let expected = body * (length * x - x * x / 2.0) / CONDUCTIVITY;
        assert_all_close(&temperatures_at(&mesh, &frd, x), expected, 1e-3 * expected);
    }
}

/// The far end gives its heat to the surroundings by convection: k T_L / L = h (T_s - T_L).
#[test]
fn calculix_cools_the_bar_by_convection() {
    let (h, sink, length) = (0.5, 100.0, 100.0);
    let (mesh, model) = thermal_bar(
        steady(),
        vec![temperature(
            "Temperature-1",
            Region::NodeSet("FIX".into()),
            0.0,
        )],
        vec![heat_load(
            "Film-1",
            Region::Surface("TIP".into()),
            LoadKind::Film {
                sink,
                coefficient: h,
            },
        )],
    );
    let Some(frd) = run_ccx("konvektion", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let expected = h * sink / (h + CONDUCTIVITY / length);
    assert_all_close(
        &temperatures_at(&mesh, &frd, length),
        expected,
        1e-6 * expected,
    );
}

/// The far end radiates to the surroundings: k (T_0 - T_L) / L = e s (T_L^4 - T_s^4) in
/// kelvin, with the physical constants of the model.
#[test]
fn calculix_cools_the_bar_by_radiation() {
    let (hot, sink, emissivity, length) = (500.0, 20.0, 0.8, 100.0);
    let (mesh, mut model) = thermal_bar(
        steady(),
        vec![temperature(
            "Temperature-1",
            Region::NodeSet("FIX".into()),
            hot,
        )],
        vec![heat_load(
            "Radiation-1",
            Region::Surface("TIP".into()),
            LoadKind::Radiation { sink, emissivity },
        )],
    );
    let (zero, sigma) =
        plx_model::ModelProperties::standard_constants(model.properties.units).unwrap();
    model.properties.absolute_zero = Some(zero);
    model.properties.stefan_boltzmann = Some(sigma);
    // Radiation is nonlinear; CalculiX iterates from the initial temperature.
    model.initial_conditions.push(plx_model::InitialCondition {
        distribution: None,
        name: "Initial_Temperature-1".into(),
        active: true,
        region: Region::Parts(vec!["EALL".into()]),
        kind: plx_model::InitialConditionKind::Temperature(hot),
    });
    let Some(frd) = run_ccx("strahlung", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let kelvin = |t: f64| t - zero;
    let balance = |t: f64| {
        CONDUCTIVITY * (hot - t) / length
            - emissivity * sigma * (kelvin(t).powi(4) - kelvin(sink).powi(4))
    };
    let (mut low, mut high) = (sink, hot);
    for _ in 0..100 {
        let mid = 0.5 * (low + high);
        if balance(mid) > 0.0 {
            low = mid;
        } else {
            high = mid;
        }
    }
    assert_all_close(
        &temperatures_at(&mesh, &frd, length),
        low,
        1e-3 * (hot - low),
    );
}

/// Without the physical constants CalculiX rejects the radiation, and the model check explains
/// its message.
#[test]
fn calculix_without_physical_constants_rejects_radiation() {
    let (mesh, model) = thermal_bar(
        steady(),
        vec![temperature(
            "Temperature-1",
            Region::NodeSet("FIX".into()),
            500.0,
        )],
        vec![heat_load(
            "Radiation-1",
            Region::Surface("TIP".into()),
            LoadKind::Radiation {
                sink: 20.0,
                emissivity: 0.8,
            },
        )],
    );
    let Some(dir) = ccx_dir("ohne-konstanten") else {
        return;
    };
    std::fs::write(
        dir.join("radiate.inp"),
        write_inp(&mesh, &model, "").unwrap(),
    )
    .unwrap();
    let output = Command::new("ccx")
        .args(["-i", "radiate"])
        .current_dir(&dir)
        .env("OMP_NUM_THREADS", "1")
        .output()
        .unwrap();
    remove_ccx_dir(&dir);
    let lines: Vec<String> = (String::from_utf8_lossy(&output.stdout).lines())
        .map(String::from)
        .collect();
    let findings = plx_model::diagnose_solver_output(&lines, &mesh);
    assert!(
        (findings.iter()).any(|f| f.problem == plx_model::Problem::NoPhysicalConstants),
        "{lines:?}"
    );
}

/// A bar heated at x = 0 and insulated elsewhere warms up as the series solution of the
/// heat equation says: T(L, t) / T_0 = 1 - sum 4 (-1)^n / ((2n+1) pi) exp(-l_n^2 a t).
#[test]
fn calculix_warms_the_bar_transiently() {
    let (hot, length, density) = (100.0, 100.0, 7.85e-9);
    let mut settings = HeatTransferStep {
        steady_state: false,
        ..HeatTransferStep::default()
    };
    // Thermal diffusivity.
    let a = CONDUCTIVITY / (density * SPECIFIC_HEAT);
    // Half way to the end temperature at the far end.
    let period = 0.4 * length * length / a;
    settings.increments.incrementation = Incrementation::Direct;
    settings.increments.time_period = period;
    settings.increments.initial_increment = period / 200.0;
    settings.increments.max_increments = 1000;
    let (mesh, mut model) = thermal_bar(
        StepKind::HeatTransfer(settings),
        vec![temperature(
            "Temperature-1",
            Region::NodeSet("FIX".into()),
            hot,
        )],
        Vec::new(),
    );
    model.initial_conditions.push(plx_model::InitialCondition {
        distribution: None,
        name: "Initial_Temperature-1".into(),
        active: true,
        region: Region::Parts(vec!["EALL".into()]),
        kind: plx_model::InitialConditionKind::Temperature(0.0),
    });
    let Some(frd) = run_ccx("instationaer", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let series: f64 = (0..50)
        .map(|n| {
            let k = f64::from(2 * n + 1);
            let lambda = k * std::f64::consts::PI / (2.0 * length);
            let sign = if n % 2 == 0 { 1.0 } else { -1.0 };
            4.0 * sign / (k * std::f64::consts::PI) * (-lambda * lambda * a * period).exp()
        })
        .sum();
    let expected = hot * (1.0 - series);
    assert!(expected > 20.0 && expected < 80.0, "{expected}");
    assert_all_close(&temperatures_at(&mesh, &frd, length), expected, 0.02 * hot);
}

/// A bar held only against rigid body motion and heated evenly by 100 from its stress free
/// temperature grows by alpha dT L and stays free of stress.
#[test]
fn calculix_expands_the_bar_in_a_coupled_step() {
    expand_bar(
        StepKind::CoupledTempDisp(HeatTransferStep::default()),
        "waermedehnung",
    );
}

/// The uncoupled step solves the temperatures first and the displacements from them; the
/// bar grows as in the coupled step.
#[test]
fn calculix_expands_the_bar_in_an_uncoupled_step() {
    let inp = expand_bar(
        StepKind::UncoupledTempDisp(HeatTransferStep::default()),
        "waermedehnung-entkoppelt",
    );
    assert!(inp.contains("*Uncoupled temperature-displacement, Steady state\n"));
}

/// Writes and runs the heated bar in a step of the given kind; returns the input file.
fn expand_bar(kind: StepKind, name: &str) -> String {
    let length = 100.0;
    let (mesh, _) = cantilever(tip_force());
    let at = |p: [f64; 3]| -> NodeId {
        let index = (mesh.coords().iter())
            .position(|c| (0..3).all(|k| (c[k] - p[k]).abs() < 1e-9))
            .unwrap();
        mesh.node_ids()[index]
    };
    let support = |name: &str, nodes: Vec<NodeId>, values: [Option<f64>; 6]| BoundaryCondition {
        distribution: None,
        name: name.into(),
        active: true,
        visible: true,
        region: Region::Nodes(nodes),
        kind: each_node(values),
        amplitude: None,
    };
    let (origin, y, z) = (at([0.0; 3]), at([0.0, 5.0, 0.0]), at([0.0, 0.0, 5.0]));
    let (mesh, mut model) = thermal_bar(
        kind,
        vec![
            support("Support-1", nodes_at(&mesh, 0, 0.0), FIX_X),
            support(
                "Support-2",
                vec![origin],
                [None, Some(0.0), Some(0.0), None, None, None],
            ),
            support(
                "Support-3",
                vec![y],
                [None, None, Some(0.0), None, None, None],
            ),
            support(
                "Support-4",
                vec![z],
                [None, Some(0.0), None, None, None, None],
            ),
            temperature("Temperature-1", Region::NodeSet("NALL".into()), 120.0),
        ],
        Vec::new(),
    );
    model.initial_conditions.push(plx_model::InitialCondition {
        distribution: None,
        name: "Initial_Temperature-1".into(),
        active: true,
        region: Region::Parts(vec!["EALL".into()]),
        kind: plx_model::InitialConditionKind::Temperature(20.0),
    });
    let inp = write_inp(&mesh, &model, "").unwrap();
    let Some(frd) = run_ccx(name, &inp) else {
        return inp;
    };
    let expected = 1e-5 * 100.0 * length;
    for node in nodes_at(&mesh, 0, length) {
        let u = node_value(&frd, "DISP", "U1", node);
        assert!((u - expected).abs() < 1e-6 * expected, "{u} != {expected}");
        let s = node_value(&frd, "STRESS", "S11", node);
        assert!(s.abs() < 1e-6, "{s}");
    }
    inp
}

/// In a plane stress model the faces are edges: a strip held at 0 at x = 0 and cooled by a
/// film at its far edge reaches k T_L / L = h (T_s - T_L), as the bar does.
#[test]
fn calculix_cools_a_plane_strip_by_convection() {
    let (length, h, sink) = (10.0, 5.0, 100.0);
    let mesh = rectangle(0.0, length, 2.0, 5, 2);
    let mut model = plane_model(
        ModelSpace::PlaneStress,
        2.0,
        Vec::new(),
        LoadKind::Film {
            sink,
            coefficient: h,
        },
        Region::Faces(edges_at(&mesh, 0, length)),
    );
    model.materials[0].conductivity = Some(CONDUCTIVITY);
    let step = &mut model.steps[0];
    step.kind = steady();
    step.field_outputs = FieldOutput::heat_transfer_defaults();
    step.boundary_conditions = vec![temperature(
        "Temperature-1",
        Region::Nodes(nodes_at(&mesh, 0, 0.0)),
        0.0,
    )];
    let text = write_inp(&mesh, &model, "").unwrap();
    let Some(frd) = run_ccx("film-2d", &text) else {
        return;
    };
    let expected = h * sink / (h + CONDUCTIVITY / length);
    assert_all_close(
        &temperatures_at(&mesh, &frd, length),
        expected,
        1e-6 * expected,
    );
}

fn amplitude(name: &str, points: Vec<[f64; 2]>) -> plx_model::Amplitude {
    plx_model::Amplitude {
        points,
        ..plx_model::Amplitude::new(name)
    }
}

#[test]
fn writes_amplitudes_and_their_references() {
    let mut force = tip_force();
    force.amplitude = Some("Ramp up".into());
    let (mesh, mut model) = cantilever(force);
    let mut ramp = amplitude(
        "Ramp up",
        vec![[0.0, 0.0], [0.2, 0.5], [0.4, 1.0], [0.6, 1.0], [1.0, 0.25]],
    );
    ramp.time_span = plx_model::AmplitudeTime::Total;
    ramp.shift_time = 0.1;
    ramp.shift_amplitude = -0.5;
    model.amplitudes.push(ramp);
    model.amplitudes.push(amplitude("Half", vec![[0.0, 0.5]]));
    let step = &mut model.steps[0];
    step.boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Displacement-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(vec![99]),
        kind: BoundaryKind::displacement([None, Some(0.5), None, None, None, None]),
        amplitude: Some("Half".into()),
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    for expected in [
        "** Amplitudes ",
        "*Amplitude, Name=Ramp_up, Time=Total time, Shiftx=0.1, Shifty=-0.5\n\
         0, 0, 0.2, 0.5, 0.4, 1, 0.6, 1\n1, 0.25\n",
        "*Amplitude, Name=Half\n0, 0.5\n",
        "** Name: Fixed-1\n*Boundary\nFIX, 1, 6, 0\n",
        "** Name: Displacement-1\n*Boundary, Amplitude=Half\n",
        "*Cload, Amplitude=Ramp_up\nInternal_Selection-1_Force-1, 3, -100\n",
    ] {
        assert!(text.contains(expected), "{expected}\n{text}");
    }
    // The amplitudes precede the steps, as CalculiX reads them before their references.
    assert!(text.find("*Amplitude,").unwrap() < text.find("*Step").unwrap());

    model.steps[0].loads[0].amplitude = Some("Gone".into());
    assert_eq!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::UnknownAmplitude {
            item: "Force-1".into(),
            amplitude: "Gone".into(),
        })
    );
    model.steps[0].loads[0].amplitude = None;
    model.amplitudes[1].points.clear();
    assert!(matches!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::InvalidAmplitude { .. })
    ));
}

#[test]
fn writes_both_amplitudes_of_films_and_radiation() {
    let mut film = heat_load(
        "Film-1",
        Region::Surface("TIP".into()),
        LoadKind::Film {
            sink: 25.0,
            coefficient: 0.01,
        },
    );
    film.amplitude = Some("Sink".into());
    film.factor_amplitude = Some("Coefficient".into());
    let mut radiation = heat_load(
        "Radiation-1",
        Region::Surface("TIP".into()),
        LoadKind::Radiation {
            sink: 25.0,
            emissivity: 0.8,
        },
    );
    radiation.factor_amplitude = Some("Coefficient".into());
    let mut flux = heat_load(
        "Flux-1",
        Region::Nodes(vec![99]),
        LoadKind::ConcentratedFlux(3.0),
    );
    flux.amplitude = Some("Sink".into());
    let mut surface_flux = heat_load(
        "Surface_Flux-1",
        Region::Surface("TIP".into()),
        LoadKind::SurfaceFlux(1.0),
    );
    surface_flux.amplitude = Some("Sink".into());
    let mut temperature = temperature("Temperature-1", Region::NodeSet("FIX".into()), 20.0);
    temperature.amplitude = Some("Sink".into());
    let (mesh, mut model) = thermal_bar(
        steady(),
        vec![temperature],
        vec![film, radiation, flux, surface_flux],
    );
    model.properties.absolute_zero = Some(-273.15);
    model.properties.stefan_boltzmann = Some(5.67e-11);
    model.amplitudes = vec![
        amplitude("Sink", vec![[0.0, 1.0], [1.0, 2.0]]),
        amplitude("Coefficient", vec![[0.0, 1.0]]),
    ];
    let text = write_inp(&mesh, &model, "").unwrap();
    for expected in [
        "*Boundary, Amplitude=Sink\nFIX, 11, 11, 20\n",
        "*Film, Amplitude=Sink, Film amplitude=Coefficient\n",
        "*Radiate, Radiation amplitude=Coefficient\n",
        "*Cflux, Amplitude=Sink\n",
        "*Dflux, Amplitude=Sink\n",
    ] {
        assert!(text.contains(expected), "{expected}\n{text}");
    }
    // CalculiX takes every one of them.
    run_ccx("amplitudes_thermal", &text);
}

/// A force that follows an amplitude up to its full value at half the step and back to half
/// of it at the end: the cantilever bends as far as with the full force at first, then half.
/// The deflection is small, so the geometrically nonlinear steps stay within a percent of
/// the linear solution.
#[test]
fn calculix_scales_a_load_by_its_amplitude() {
    let (mesh, reference) = cantilever(tip_force());
    let Some(full) = run_ccx(
        "amplitude_reference",
        &write_inp(&mesh, &reference, "").unwrap(),
    ) else {
        return;
    };
    let full = node_value(&full, "DISP", "U3", 99);
    let mut model = reference.clone();
    model.amplitudes.push(amplitude(
        "Up_and_down",
        vec![[0.0, 0.0], [0.5, 1.0], [1.0, 0.5]],
    ));
    model.steps[0].loads[0].amplitude = Some("Up_and_down".into());
    let StepKind::Static(settings) = &mut model.steps[0].kind else {
        unreachable!()
    };
    // A linear static step is solved once, at its end; increments need a nonlinear one.
    settings.nlgeom = true;
    settings.incrementation = Incrementation::Direct;
    settings.initial_increment = 0.25;
    let frd = run_ccx("amplitude_force", &write_inp(&mesh, &model, "").unwrap()).unwrap();
    let index = frd.mesh.node_index(99).unwrap();
    let u3: Vec<f64> = (frd.increments.iter())
        .filter_map(|i| i.field("DISP"))
        .map(|f| f64::from(f.component("U3").unwrap().values[index]))
        .collect();
    assert_eq!(u3.len(), 4, "{u3:?}");
    for (value, factor) in u3.iter().zip([0.5, 1.0, 0.75, 0.5]) {
        assert!(
            (value - factor * full).abs() < 1e-2 * full.abs(),
            "{u3:?} instead of {factor} * {full}"
        );
    }
}

/// A temperature held by an amplitude that ends at half: the bar settles at half the value.
#[test]
fn calculix_scales_a_temperature_by_its_amplitude() {
    let mut held = temperature("Temperature-1", Region::NodeSet("FIX".into()), 100.0);
    held.amplitude = Some("Half".into());
    let (mesh, mut model) = thermal_bar(steady(), vec![held], Vec::new());
    model
        .amplitudes
        .push(amplitude("Half", vec![[0.0, 1.0], [1.0, 0.5]]));
    let Some(frd) = run_ccx(
        "amplitude_temperature",
        &write_inp(&mesh, &model, "").unwrap(),
    ) else {
        return;
    };
    assert_all_close(&temperatures_at(&mesh, &frd, 100.0), 50.0, 1e-6);
}

/// Beyond its last point an amplitude keeps the last value, as [`plx_model::Amplitude::value_at`]
/// assumes for the dialog's curve.
#[test]
fn calculix_holds_an_amplitude_after_its_last_point() {
    let (mesh, reference) = cantilever(tip_force());
    let Some(full) = run_ccx(
        "amplitude_hold_ref",
        &write_inp(&mesh, &reference, "").unwrap(),
    ) else {
        return;
    };
    let full = node_value(&full, "DISP", "U3", 99);
    let mut model = reference;
    model
        .amplitudes
        .push(amplitude("Early", vec![[0.0, 0.0], [0.5, 1.0]]));
    model.steps[0].loads[0].amplitude = Some("Early".into());
    let frd = run_ccx("amplitude_hold", &write_inp(&mesh, &model, "").unwrap()).unwrap();
    let held = node_value(&frd, "DISP", "U3", 99);
    assert!(
        (held - full).abs() < 1e-6 * full.abs(),
        "{held} instead of {full}"
    );
}

/// The tip deflection `U3` at node 99 at the end of every step.
fn tip_deflection_per_step(frd: &FrdImport) -> Vec<f64> {
    let index = frd.mesh.node_index(99).unwrap();
    let mut out: Vec<(u32, f64)> = Vec::new();
    for increment in &frd.increments {
        let Some(u3) = increment.field("DISP").and_then(|f| f.component("U3")) else {
            continue;
        };
        let value = f64::from(u3.values[index]);
        match out.last_mut() {
            Some((step, last)) if *step == increment.step => *last = value,
            _ => out.push((increment.step, value)),
        }
    }
    out.into_iter().map(|(_, v)| v).collect()
}

/// A `TIME=TOTAL TIME` amplitude with both shifts over three linear steps of different
/// periods: each step is solved once at its end, with the amplitude read at the total time.
#[test]
fn calculix_reads_a_total_time_amplitude_with_shifts_across_steps() {
    let (mesh, mut model) = cantilever(tip_force());
    let mut total = amplitude("Total", vec![[0.0, 0.0], [1.0, 0.5], [3.0, 1.0]]);
    total.time_span = plx_model::AmplitudeTime::Total;
    total.shift_time = 0.25;
    total.shift_amplitude = 0.1;
    model.amplitudes.push(total.clone());
    model.steps[0].loads[0].amplitude = Some("Total".into());
    let periods = [0.5, 1.5, 2.0];
    let first = model.steps[0].clone();
    model.steps.clear();
    for (n, period) in (1..).zip(periods) {
        let mut step = first.clone();
        step.name = format!("Step-{n}");
        let StepKind::Static(settings) = &mut step.kind else {
            unreachable!()
        };
        // The initial increment does not matter: a linear step is solved once.
        settings.incrementation = Incrementation::Automatic;
        settings.time_period = period;
        settings.initial_increment = period / 2.0;
        model.steps.push(step);
    }
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Amplitude, Name=Total, Time=Total time, Shiftx=0.25, Shifty=0.1\n"),
        "{text}"
    );
    let Some(frd) = run_ccx("amplitude_total_time", &text) else {
        return;
    };
    let u3 = tip_deflection_per_step(&frd);
    assert_eq!(u3.len(), 3, "{u3:?}");
    // The deflections are in the ratio of the amplitude at the total times 0.5, 2 and 4.
    let factors = [0.5, 2.0, 4.0].map(|t| total.value_at(t));
    for (value, factor) in u3.iter().zip(factors) {
        let expected = u3[0] * factor / factors[0];
        assert!(
            (value - expected).abs() < 1e-5 * expected.abs(),
            "{u3:?} instead of the ratios of {factors:?}"
        );
    }
}

/// Pressure, gravity, a centrifugal load and a prescribed displacement, each scaled by a
/// step-time amplitude, in a step that ends before the amplitudes do; then a second step
/// that writes them again and reads them at its own end.
#[test]
fn calculix_scales_distributed_loads_and_displacements_by_amplitudes() {
    let pressure = Load {
        distribution: None,
        name: "Pressure-1".into(),
        active: true,
        visible: true,
        region: Region::Faces(vec![(10, 4), (20, 4), (30, 4), (40, 4)]),
        kind: LoadKind::Pressure(2.0),
        amplitude: Some("Rise".into()),
        factor_amplitude: None,
    };
    let (mesh, mut model) = cantilever(pressure);
    let mut gravity = body_load(LoadKind::Gravity([0.0, -9810.0, 0.0]));
    gravity.name = "Gravity-1".into();
    gravity.amplitude = Some("Rise".into());
    let mut spin = body_load(LoadKind::Centrifugal {
        point: [0.0, 5.0, 5.0],
        axis: [0.0, 0.0, 1.0],
        speed: 100.0,
    });
    spin.name = "Spin-1".into();
    spin.amplitude = Some("Fall".into());
    let step = &mut model.steps[0];
    step.loads.push(gravity);
    step.loads.push(spin);
    step.boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Lift-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(vec![99]),
        kind: BoundaryKind::displacement([None, None, Some(0.01), None, None, None]),
        amplitude: Some("Fall".into()),
    });
    let StepKind::Static(settings) = &mut step.kind else {
        unreachable!()
    };
    settings.incrementation = Incrementation::Automatic;
    settings.time_period = 0.5;
    settings.initial_increment = 0.5;
    let mut second = model.steps[0].clone();
    second.name = "Step-2".into();
    let StepKind::Static(settings) = &mut second.kind else {
        unreachable!()
    };
    settings.time_period = 2.0;
    settings.initial_increment = 1.0;
    model.steps.push(second);
    model.amplitudes = vec![
        amplitude("Rise", vec![[0.0, 0.0], [1.0, 1.0], [2.0, 3.0]]),
        amplitude("Fall", vec![[0.0, 2.0], [2.0, 0.5]]),
    ];
    let text = write_inp(&mesh, &model, "").unwrap();
    for expected in [
        "*Dload, Amplitude=Rise\n",
        "*Boundary, Amplitude=Fall\nInternal_Selection-1_Lift-1, 3, 3, 0.01\n",
    ] {
        assert!(text.contains(expected), "{expected}\n{text}");
    }
    let Some(frd) = run_ccx("amplitude_loads", &text) else {
        return;
    };
    // The prescribed tip displacement follows its amplitude at each step's end.
    let u3 = tip_deflection_per_step(&frd);
    assert_eq!(u3.len(), 2, "{u3:?}");
    for (value, factor) in u3.iter().zip([1.625, 0.5]) {
        assert!((value - 0.01 * factor).abs() < 1e-9, "{u3:?}");
    }
}

/// Gravity on quadratic elements: the consistent body force of the C3D20R cantilever adds up
/// to its weight.
#[test]
fn calculix_weighs_a_quadratic_cantilever() {
    let (mesh, model) = analysis(
        "kragbalken_c3d20r.inp",
        body_load(LoadKind::Gravity([0.0, 0.0, -9810.0])),
    );
    let Some(frd) = run_ccx("gravity_c3d20r", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let weight = 7.85e-9 * 100.0 * 100.0 * 9810.0;
    let reaction = node_sum(&frd, "FORC", "F3", &fixed_nodes(&mesh));
    assert!(
        reaction > 0.9 * weight && reaction < weight,
        "{reaction} vs {weight}"
    );
}

#[test]
fn history_outputs_become_print_keywords_with_their_sets() {
    let (mesh, mut model) = cantilever(tip_force());
    let step = &mut model.steps[0];
    let mut nodes = HistoryOutput::node("NH_Output-1", Region::Nodes(vec![99]));
    nodes.totals = Totals::Yes;
    step.history_outputs.push(nodes);
    let mut elements = HistoryOutput::element("EH_Output-1", Region::Parts(vec!["EALL".into()]));
    elements.variables = vec!["S".into(), "EVOL".into()];
    step.history_outputs.push(elements);
    let mut off = HistoryOutput::node("NH_Output-2", Region::NodeSet("FIX".into()));
    off.active = false;
    step.history_outputs.push(off);
    let mut empty = HistoryOutput::node("NH_Output-3", Region::NodeSet("FIX".into()));
    empty.variables.clear();
    step.history_outputs.push(empty);
    let text = write_inp(&mesh, &model, "").unwrap();
    for expected in [
        "*Nset, Nset=Internal_Selection-1_NH_Output-1\n99\n",
        "*Elset, Elset=Internal_Selection-1_EH_Output-1\nEALL\n",
        "** History outputs +++++++++++++++++++++++++++++++++++++++++\n**\n\
         ** Name: NH_Output-1\n\
         *Node print, Nset=Internal_Selection-1_NH_Output-1, Totals=Yes\nRF, U\n\
         ** Name: EH_Output-1\n\
         *El print, Elset=Internal_Selection-1_EH_Output-1\nS, EVOL\n\
         ** Name: NH_Output-2: Deactivated\n**\n",
    ] {
        assert!(text.contains(expected), "{expected}\nmissing in\n{text}");
    }
    assert!(!text.contains("NH_Output-3"));
    // A deactivated step lists its history outputs as comments.
    model.steps[0].active = false;
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("** Name: NH_Output-1: Deactivated\n"));
    assert!(!text.contains("*Node print"));
}

#[test]
fn contact_forces_need_an_active_contact_pair() {
    let (mesh, mut model) = cantilever(tip_force());
    let mut output = HistoryOutput::contact("CH_Output-1", "Contact_Pair-1");
    model.steps[0].history_outputs.push(output.clone());
    // Without contact forces no surfaces are needed.
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("** Name: CH_Output-1\n*Contact print\nCDIS, CSTR\n"));
    output.variables.push("CF".into());
    model.steps[0].history_outputs[0] = output;
    assert_eq!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::UnknownContactPair {
            item: "CH_Output-1".into(),
            pair: "Contact_Pair-1".into(),
        })
    );
}

#[test]
fn calculix_prints_the_history_outputs_into_the_dat_file() {
    let (mesh, mut model) = cantilever(tip_force());
    let step = &mut model.steps[0];
    let mut reactions = HistoryOutput::node("NH_Output-1", Region::NodeSet("FIX".into()));
    reactions.variables = vec!["RF".into()];
    reactions.totals = Totals::Only;
    step.history_outputs.push(reactions);
    step.history_outputs
        .push(HistoryOutput::node("NH_Output-2", Region::Nodes(vec![99])));
    let mut volume = HistoryOutput::element("EH_Output-1", Region::Parts(vec!["EALL".into()]));
    volume.variables = vec!["S".into(), "EVOL".into()];
    volume.totals = Totals::Yes;
    step.history_outputs.push(volume);
    let Some(dat) = run_ccx_dat("historie", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let import = crate::dat::parse_dat(&dat);
    assert!(import.warnings.is_empty(), "{:?}", import.warnings);
    let set = |name: &str| import.sets.iter().find(|s| s.name == name).unwrap();
    let total = set("FIX").field("TOTAL_FORCE").unwrap();
    let rf3 = &total.component("RF3").unwrap().entries[0].values;
    assert!((rf3[0] - 100.0).abs() < 1e-6, "{rf3:?}");
    let tip = set("NH_OUTPUT-2").field("DISPLACEMENTS").unwrap();
    let u3 = tip.component("U3").unwrap();
    assert_eq!(u3.entries[0].name, "99");
    assert!(u3.entries[0].values[0] < 0.0);
    let elements = set("EH_OUTPUT-1");
    assert!(
        elements
            .field("STRESSES")
            .unwrap()
            .component("MISES")
            .is_some()
    );
    let volume = elements.field("TOTAL_VOLUME").unwrap().components[0].entries[0].values[0];
    let bounds = mesh.bounds().unwrap();
    let expected: f64 = (0..3).map(|i| bounds.1[i] - bounds.0[i]).product();
    assert!(
        (volume - expected).abs() < 1e-6 * expected,
        "{volume} {expected}"
    );
}

#[test]
fn calculix_prints_the_contact_force_of_a_pair() {
    let (mesh, lower, upper) = two_blocks(0.0);
    let mut model = stacked_blocks(&mesh);
    let top: Vec<NodeId> = (119..=127).collect();
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Side-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(top),
        kind: each_node([Some(0.0), Some(0.0), None, None, None, None]),
        amplitude: None,
    });
    model.surface_interactions.push(SurfaceInteraction {
        name: "Surface_Interaction-1".into(),
        properties: vec![InteractionProperty::SurfaceBehavior(SurfaceBehavior::Hard)],
    });
    let mut pair = ContactPair::new("Contact_Pair-1", "Surface_Interaction-1");
    pair.master = Region::Faces(lower);
    pair.slave = Region::Faces(upper);
    model.contact_pairs.push(pair);
    let mut output = HistoryOutput::contact("CH_Output-1", "Contact_Pair-1");
    output.variables = ["CDIS", "CSTR", "CNUM", "CF"].map(String::from).to_vec();
    model.steps[0].history_outputs.push(output);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains(
        "*Contact print, Master=Internal_Selection-1_Contact_Pair-1_Master, \
         Slave=Internal_Selection-1_Contact_Pair-1_Slave\nCDIS, CSTR, CNUM, CF\n"
    ));
    let Some(dat) = run_ccx_dat("kontakt-historie", &text) else {
        return;
    };
    let import = crate::dat::parse_dat(&dat);
    assert!(import.warnings.is_empty(), "{:?}", import.warnings);
    let pair = import
        .sets
        .iter()
        .find(|s| s.name == "CONTACT_PAIR-1")
        .unwrap();
    let force = pair
        .field("TOTAL_SURFACE_FORCE")
        .unwrap()
        .component("FZ")
        .unwrap();
    let fz = *force.entries[0].values.last().unwrap();
    // 10 MPa on 10 x 10 mm press the blocks together.
    assert!((fz.abs() - 1000.0).abs() < 10.0, "{fz}");
    let all = (import.sets.iter())
        .find(|s| s.name == crate::dat::ALL_CONTACT_ELEMENTS)
        .unwrap();
    assert!(all.field("CONTACT_STRESS").is_some());
    assert!(all.field("TOTAL_NUMBER_OF_CONTACT_ELEMENTS").is_some());
}

/// The free cantilever without supports or loads, moving from an initial velocity in a
/// short Dynamic step with direct increments.
fn coasting_cantilever(kind: plx_model::InitialConditionKind) -> (FeMesh, FeModel) {
    let (mesh, mut model) = dynamic_cantilever(1e-3, 1e-4, None);
    let step = &mut model.steps[0];
    step.boundary_conditions.clear();
    step.loads.clear();
    model.initial_conditions.push(plx_model::InitialCondition {
        distribution: None,
        name: "Initial_Velocity-1".into(),
        active: true,
        region: Region::Parts(vec!["EALL".into()]),
        kind,
    });
    (mesh, model)
}

#[test]
fn initial_velocities_are_written_like_prepomax_does() {
    use plx_model::InitialConditionKind;
    let (mesh, mut model) =
        coasting_cantilever(InitialConditionKind::Velocity([0.0, 0.0, -1000.0]));
    model.initial_conditions[0].region = Region::NodeSet("FIX".into());
    let tip = mesh.node(99).unwrap();
    model.initial_conditions.push(plx_model::InitialCondition {
        distribution: None,
        name: "Initial_Angular_Velocity-1".into(),
        active: true,
        region: Region::Nodes(vec![99]),
        kind: InitialConditionKind::AngularVelocity {
            point: [0.0, 0.0, 5.0],
            axis: [0.0, 2.0, 0.0],
            speed: 10.0,
        },
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    // y x (x, y, z - 5) = (z - 5, 0, -x): the tip moves down and, off the axis, along x.
    let expected = format!(
        "** Name: Initial_Velocity-1\n*Initial conditions, Type=Velocity\nFIX, 3, -1000\n\
         ** Name: Initial_Angular_Velocity-1\n*Initial conditions, Type=Velocity\n\
         99, 1, {}\n99, 3, {}\n",
        number(10.0 * (tip[2] - 5.0)),
        number(-10.0 * tip[0])
    );
    assert!(text.contains(&expected), "missing {expected:?} in\n{text}");
    assert!(
        !text.contains("FIX, 1,"),
        "zero components are left out:\n{text}"
    );
}

/// A free body keeps its initial velocity: after 1 ms at 1000 mm/s the tip has moved
/// 1 mm; rotating at 10 rad/s about the root, the tip at the length L has moved 10 L t.
#[test]
fn calculix_starts_the_cantilever_from_its_initial_velocity() {
    use plx_model::InitialConditionKind;
    let (mesh, model) = coasting_cantilever(InitialConditionKind::Velocity([0.0, 0.0, -1000.0]));
    let Some(frd) = run_ccx(
        "anfangsgeschwindigkeit",
        &write_inp(&mesh, &model, "").unwrap(),
    ) else {
        return;
    };
    let u3 = node_value(&frd, "DISP", "U3", 99);
    assert!((u3 + 1.0).abs() < 0.02, "tip moved {u3} mm, expected -1");
    let tip = mesh.node(99).unwrap();
    let (mesh, model) = coasting_cantilever(InitialConditionKind::AngularVelocity {
        point: [0.0, tip[1], tip[2]],
        axis: [0.0, 1.0, 0.0],
        speed: 10.0,
    });
    let frd = run_ccx("anfangsdrehung", &write_inp(&mesh, &model, "").unwrap()).unwrap();
    let u3 = node_value(&frd, "DISP", "U3", 99);
    let expected = -10.0 * tip[0] * 1e-3;
    assert!(
        (u3 - expected).abs() < 0.02 * expected.abs(),
        "tip moved {u3} mm, expected {expected}"
    );
}

fn body_load(kind: LoadKind) -> Load {
    Load {
        distribution: None,
        name: "Body-1".into(),
        active: true,
        visible: true,
        region: Region::Parts(vec!["EALL".into()]),
        kind,
        amplitude: None,
        factor_amplitude: None,
    }
}

#[test]
fn gravity_and_centrifugal_loads_are_written_like_prepomax() {
    let (mesh, model) = cantilever(body_load(LoadKind::Gravity([0.0, 0.0, -9810.0])));
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains(
            "** Name: Body-1\n*Dload\nInternal_Selection-1_Body-1, Grav, 9810, 0, 0, -1\n"
        ),
        "{text}"
    );
    let (mesh, model) = cantilever(body_load(LoadKind::Centrifugal {
        point: [0.0, 5.0, 5.0],
        axis: [0.0, 0.0, 2.0],
        speed: 100.0,
    }));
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Dload\nInternal_Selection-1_Body-1, Centrif, 10000, 0, 5, 5, 0, 0, 1\n"),
        "{text}"
    );
    let (mesh, model) = cantilever(body_load(LoadKind::Gravity([0.0; 3])));
    assert!(matches!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::InvalidLoad { .. })
    ));
}

/// The nodes of the fixed end of the cantilever.
fn fixed_nodes(mesh: &FeMesh) -> Vec<NodeId> {
    mesh.node_sets.get("FIX").unwrap().to_vec()
}

#[test]
fn calculix_reactions_balance_the_weight_and_the_centrifugal_force() {
    // Steel beam 10 x 10 x 100 mm, 7.85e-9 t/mm³: weight 0.077 N under 9810 mm/s².
    let (mesh, model) = cantilever(body_load(LoadKind::Gravity([0.0, 0.0, -9810.0])));
    let Some(frd) = run_ccx("gravity", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let weight = 7.85e-9 * 100.0 * 100.0 * 9810.0;
    // CalculiX's nodal forces hold the reaction and the applied body force at a node; the
    // fixed nodes carry the weight of half of the first of the ten element layers.
    let expected = weight * (1.0 - 0.5 / 10.0);
    let reaction = node_sum(&frd, "FORC", "F3", &fixed_nodes(&mesh));
    assert!(
        (reaction - expected).abs() < 1e-3 * weight,
        "{reaction} vs {expected}"
    );
    // Rotating about the z axis through the fixed end: the centrifugal force on the beam is
    // rho A omega² L² / 2, pulling it away from the axis.
    let speed = 100.0;
    let (mesh, model) = cantilever(body_load(LoadKind::Centrifugal {
        point: [0.0, 5.0, 5.0],
        axis: [0.0, 0.0, 1.0],
        speed,
    }));
    let Some(frd) = run_ccx("centrifugal", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let pull = 7.85e-9 * 100.0 * speed * speed * 100.0_f64.powi(2) / 2.0;
    // The fixed nodes take a third of the force of the first layer (1/100 of the pull).
    let expected = pull * (1.0 - 1.0 / 300.0);
    let reaction = node_sum(&frd, "FORC", "F1", &fixed_nodes(&mesh));
    assert!(
        (reaction + expected).abs() < 1e-3 * pull,
        "{reaction} vs {expected}"
    );
}

/// The cantilever spinning about its own axis like a shaft: a static step with the
/// centrifugal load, a frequency step storing its modes and a complex frequency step with
/// the Coriolis forces, as CalculiX wants them.
fn rotor_analysis(speed: f64) -> (FeMesh, FeModel) {
    let (mesh, mut model) = analysis(
        "kragbalken_c3d8.inp",
        body_load(LoadKind::Centrifugal {
            point: [0.0, 5.0, 5.0],
            axis: [1.0, 0.0, 0.0],
            speed,
        }),
    );
    let mut frequency = Step::new_frequency("Step-2");
    frequency.boundary_conditions = model.steps[0].boundary_conditions.clone();
    let StepKind::Frequency(settings) = &mut frequency.kind else {
        unreachable!()
    };
    settings.perturbation = true;
    settings.storage = true;
    settings.num_frequencies = 4;
    model.steps.push(frequency);
    let mut complex = Step::new_complex_frequency("Step-3");
    complex.boundary_conditions = model.steps[0].boundary_conditions.clone();
    let StepKind::ComplexFrequency(settings) = &mut complex.kind else {
        unreachable!()
    };
    settings.num_frequencies = 4;
    model.steps.push(complex);
    (mesh, model)
}

/// The half x <= 50 of the cantilever as a submodel: held at x = 0, its cut at x = 50
/// driven by the global results with `dofs`.
fn cantilever_submodel(dofs: [bool; 6]) -> (FeMesh, FeModel) {
    let text = std::fs::read_to_string(testdata("kragbalken_c3d8.inp")).unwrap();
    let mesh = read_inp_str(&text, None).unwrap().mesh;
    let x = |id: NodeId| mesh.node(id).unwrap()[0];
    let mut sub = String::new();
    let mut block = "";
    for line in text.lines() {
        if line.starts_with('*') {
            let kept = ["*NODE,", "*ELEMENT,", "*NSET, NSET=FIX"];
            block = if kept.iter().any(|k| line.starts_with(k)) {
                line
            } else {
                ""
            };
            if !block.is_empty() {
                sub.push_str(line);
                sub.push('\n');
            }
            continue;
        }
        let ids: Vec<NodeId> = (line.split(',').map(str::trim))
            .filter_map(|v| v.parse().ok())
            .collect();
        let keep = if block.starts_with("*NODE") {
            x(ids[0]) <= 50.0
        } else if block.starts_with("*ELEMENT") {
            ids[1..].iter().all(|&n| x(n) <= 50.0)
        } else {
            !block.is_empty()
        };
        if keep {
            sub.push_str(line);
            sub.push('\n');
        }
    }
    let mesh = read_inp_str(&sub, None).unwrap().mesh;
    let cut: Vec<NodeId> = (mesh.node_ids().iter().copied())
        .filter(|&n| (mesh.node(n).unwrap()[0] - 50.0).abs() < 1e-9)
        .collect();
    let (_, mut model) = cantilever(tip_force());
    let step = &mut model.steps[0];
    step.loads.clear();
    step.boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Submodel-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(cut),
        kind: BoundaryKind::Submodel { step: 1, dofs },
        amplitude: None,
    });
    model.properties.kind = plx_model::ModelKind::Submodel;
    model.properties.global_results = Some(PathBuf::from("/somewhere/global.frd"));
    (mesh, model)
}

#[test]
fn a_complex_frequency_step_is_written_like_prepomax_does() {
    let (mesh, mut model) = rotor_analysis(100.0);
    let text = write_inp(&mesh, &model, "").unwrap();
    let complex = &text[text.find("** Step-3").unwrap()..];
    for line in [
        "*Step, Perturbation\n*Complex frequency, Coriolis\n4\n",
        "*Boundary\nFIX, 1, 6, 0\n",
        "*Node file\nU, PU\n",
    ] {
        assert!(complex.contains(line), "missing {line:?} in\n{complex}");
    }
    assert!(!complex.contains("*Dload"), "{complex}");
    let StepKind::ComplexFrequency(settings) = &mut model.steps[2].kind else {
        unreachable!()
    };
    settings.coriolis = false;
    let StepKind::Frequency(settings) = &mut model.steps[1].kind else {
        unreachable!()
    };
    settings.perturbation = false;
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Step\n*Complex frequency\n4\n"), "{text}");
}

#[test]
fn the_complex_frequency_step_takes_perturbation_from_the_stored_frequency_step() {
    // CalculiX stops ("the .eig-file was created without perturbation info") unless both
    // steps agree; a rotor without prestress in the frequency step must solve as well.
    for perturbation in [false, true] {
        let (mesh, mut model) = rotor_analysis(100.0);
        let StepKind::Frequency(settings) = &mut model.steps[1].kind else {
            unreachable!()
        };
        settings.perturbation = perturbation;
        let text = write_inp(&mesh, &model, "").unwrap();
        let complex = &text[text.find("** Step-3").unwrap()..];
        let header = if perturbation {
            "*Step, Perturbation\n"
        } else {
            "*Step\n"
        };
        assert!(
            complex.contains(&format!("{header}*Complex frequency, Coriolis\n")),
            "{complex}"
        );
        let Some(frd) = run_ccx(&format!("rotor-perturbation-{perturbation}"), &text) else {
            return;
        };
        let whirling = (frd.increments.iter())
            .filter(|i| i.kind == plx_results::AnalysisKind::ComplexFrequency)
            .count();
        assert_eq!(whirling, 4);
    }
}

#[test]
fn calculix_splits_the_bending_modes_of_the_rotating_cantilever() {
    // At 100 rad/s about its axis the two bending modes of the square beam split into a
    // backward and a forward whirl around the frequency of the standing beam.
    let (mesh, model) = rotor_analysis(100.0);
    let Some(frd) = run_ccx("rotor", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let standing: Vec<f64> = (frd.increments.iter())
        .filter(|i| i.kind == plx_results::AnalysisKind::Frequency)
        .map(|i| i.value)
        .collect();
    let whirling: Vec<&plx_results::Increment> = (frd.increments.iter())
        .filter(|i| i.kind == plx_results::AnalysisKind::ComplexFrequency)
        .collect();
    assert_eq!(standing.len(), 4, "{standing:?}");
    assert_eq!(
        whirling.iter().map(|i| i.increment).collect::<Vec<_>>(),
        [1, 2, 3, 4]
    );
    let f1 = standing[0];
    let (backward, forward) = (whirling[0].value, whirling[1].value);
    assert!(
        backward < f1 && f1 < forward,
        "{backward} < {f1} < {forward}"
    );
    // The real part of the mode shape and its magnitudes and phases for the whirl animation.
    let mode = whirling[0];
    assert!(mode.field("DISP").is_some());
    let pdisp = mode.field("PDISP").unwrap();
    for name in ["MAG1", "MAG2", "MAG3", "PHA1", "PHA2", "PHA3"] {
        assert!(pdisp.component(name).is_some(), "{name} missing");
    }
    let tip = frd.mesh.node_index(99).unwrap();
    let magnitude = |k: &str| f64::from(pdisp.component(k).unwrap().values[tip]);
    assert!(magnitude("MAG2").hypot(magnitude("MAG3")) > 0.0);
}

#[test]
fn calculix_prints_the_complex_frequencies_and_the_whirl_direction() {
    let (mesh, model) = rotor_analysis(100.0);
    let Some(dat) = run_ccx_dat("rotor_dat", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let import = crate::dat::parse_dat(&dat);
    let set = import.sets.iter().find(|s| s.name == "STEP_3").unwrap();
    let output = set.field("EIGENVALUE_OUTPUT").unwrap();
    let whirl = &output.component("TURNING_DIRECTION").unwrap().entries[0].values;
    assert_eq!(whirl.len(), 4, "{whirl:?}");
    assert!(whirl.iter().all(|w| w.abs() == 1.0), "{whirl:?}");
    assert!(whirl.contains(&1.0) && whirl.contains(&-1.0), "{whirl:?}");
    assert_eq!(
        output.component("FREQUENCY").unwrap().entries[0]
            .values
            .len(),
        4
    );
}

#[test]
fn submodels_read_the_cut_displacements_of_the_global_results() {
    let (mesh, mut model) = cantilever_submodel([true, false, true, false, false, false]);
    let text = write_inp(&mesh, &model, "").unwrap();
    let heading = text.find("*Heading").unwrap();
    let submodel = text
        .find("*Submodel, Type=Node, Input=\"global.frd\"\nInternal_Selection-1_Submodel-1\n")
        .expect(&text);
    assert!(heading < submodel && submodel < text.find("*Node").unwrap());
    assert!(text.contains(
        "** Name: Submodel-1\n*Boundary, Submodel, Step=1\nInternal_Selection-1_Submodel-1, 1, 1\n\
         Internal_Selection-1_Submodel-1, 3, 3\n"
    ));

    // A general model has no global results, as in PrePoMax.
    model.properties.kind = plx_model::ModelKind::General;
    assert!(matches!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::NoGlobalResults { item }) if item == "Submodel-1"
    ));
    // Without an active submodel boundary condition nothing is read.
    model.steps[0].boundary_conditions[1].active = false;
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(!text.contains("*Submodel"));
}

#[test]
fn calculix_submodel_follows_the_global_cantilever() {
    let (mesh, mut model) = cantilever_submodel([true, true, true, false, false, false]);
    let name = "submodel";
    let dir = std::env::temp_dir().join(format!("plx-write-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::copy(testdata("kragbalken_c3d8.frd"), dir.join("global.frd")).unwrap();
    model.properties.global_results = Some(dir.join("global.frd"));
    let Some(frd) = run_ccx(name, &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let global = read_frd(&testdata("kragbalken_c3d8.frd")).unwrap();
    // The submodel has the global mesh of its half, so it reproduces the global solution
    // inside, not only at the cut.
    for node in mesh.node_ids().iter().copied() {
        for component in ["U1", "U2", "U3"] {
            let ours = node_value(&frd, "DISP", component, node);
            let expected = node_value(&global, "DISP", component, node);
            assert!(
                (ours - expected).abs() <= 1e-4 * expected.abs().max(1e-3),
                "node {node} {component}: {ours} != {expected}"
            );
        }
    }
}

/// The shell plate cut in half along its length at y = 5 with a symmetry boundary condition,
/// turned by `angle` degrees about z, with the tip force of the full plate shared out.
fn half_shell_plate(angle: f64) -> (FeMesh, FeModel, NodeId) {
    let mut mesh = rectangle(0.0, 100.0, 5.0, 20, 1);
    let tip = nodes_at(&mesh, 0, 100.0);
    let edge = nodes_at(&mesh, 1, 5.0);
    let clamped = nodes_at(&mesh, 0, 0.0);
    let corner = (tip.iter().copied())
        .find(|&n| mesh.node(n).unwrap()[1] == 0.0)
        .unwrap();
    turn(&mut mesh, angle);
    let mut model = plane_model(
        ModelSpace::ThreeD,
        1.0,
        vec![(clamped, [Some(0.0); 6])],
        // The node on the symmetry plane carries half the share of the two others.
        LoadKind::force([0.0, 0.0, -0.4]),
        Region::Nodes(tip.iter().copied().filter(|n| !edge.contains(n)).collect()),
    );
    let mut middle = model.steps[0].loads[0].clone();
    middle.name = "Load-2".into();
    middle.kind = LoadKind::force([0.0, 0.0, -0.1]);
    middle.region = Region::Nodes(tip.iter().copied().filter(|n| edge.contains(n)).collect());
    model.steps[0].loads.push(middle);
    model.sections[0].kind = SectionKind::Shell {
        thickness: 2.0,
        offset: 0.0,
    };
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Symmetry-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(edge),
        kind: BoundaryKind::Symmetry { normal: None },
        amplitude: None,
    });
    (mesh, model, corner)
}

#[test]
fn a_symmetry_plane_holds_the_normal_displacement_and_the_in_plane_rotations() {
    let (mesh, model, _) = half_shell_plate(0.0);
    let text = write_inp(&mesh, &model, "").unwrap();
    let expected = "** Name: Symmetry-1\n*Boundary\nInternal_Selection-1_Symmetry-1, 2, 2, 0\n\
                    Internal_Selection-1_Symmetry-1, 4, 4, 0\n\
                    Internal_Selection-1_Symmetry-1, 6, 6, 0\n";
    assert!(text.contains(expected), "{text}");
    assert!(!text.contains("*Transform"), "{text}");

    // A skewed plane gets a local system with x along its normal.
    let (mesh, model, _) = half_shell_plate(30.0);
    let text = write_inp(&mesh, &model, "").unwrap();
    let expected = "*Boundary\nInternal_Selection-1_Symmetry-1, 1, 1, 0\n\
                    Internal_Selection-1_Symmetry-1, 5, 5, 0\n\
                    Internal_Selection-1_Symmetry-1, 6, 6, 0\n";
    assert!(text.contains(expected), "{text}");
    let transform = text
        .split("*Transform, Nset=Internal_Selection-1_Symmetry-1, Type=R\n")
        .nth(1)
        .expect("transform missing");
    let values: Vec<f64> = (transform.lines().next().unwrap().split(','))
        .map(|v| v.trim().parse().unwrap())
        .collect();
    // The normal of the plane y = 5 turned by 30 degrees, and a vector in the plane.
    let (s, c) = 30f64.to_radians().sin_cos();
    let normal = [-s, c, 0.0];
    let sign = values[1].signum();
    for k in 0..3 {
        assert!((sign * values[k] - normal[k]).abs() < 1e-7, "{values:?}");
    }
    let along: f64 = (0..3).map(|k| values[3 + k] * normal[k]).sum();
    assert!(along.abs() < 1e-7, "{values:?}");

    // A single node fixes no plane.
    let (mesh, mut model, corner) = half_shell_plate(0.0);
    model.steps[0].boundary_conditions[1].region = Region::Nodes(vec![corner]);
    assert!(matches!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::NoSymmetryPlane { .. })
    ));
}

/// Half the shell plate with a symmetry plane deflects like the full plate, also when the
/// plane is no coordinate plane.
#[test]
fn calculix_bends_the_half_shell_plate_like_the_full_one() {
    let (mesh, model) = shell_plate();
    let Some(full) = run_ccx("symmetrie-voll", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let corner = (nodes_at(&mesh, 0, 100.0).into_iter())
        .find(|&n| mesh.node(n).unwrap()[1] == 0.0)
        .unwrap();
    let expected = expanded_value(&full, &mesh, ("DISP", "U3"), corner, false);
    // CalculiX joins the expanded shell nodes of a *Transform by a knot, which makes the
    // only element across this narrow plate a little softer than without.
    for (angle, tolerance) in [(0.0, 1e-4), (30.0, 1e-2)] {
        let (mesh, model, corner) = half_shell_plate(angle);
        let text = write_inp(&mesh, &model, "").unwrap();
        let half = run_ccx(&format!("symmetrie-{angle}"), &text).unwrap();
        let ours = expanded_value(&half, &mesh, ("DISP", "U3"), corner, false);
        assert!(
            (ours - expected).abs() < tolerance * expected.abs(),
            "{angle} degrees: {ours} != {expected}"
        );
    }
    // Without the rotations the half plate would be softer.
    let (mesh, mut model, corner) = half_shell_plate(0.0);
    let edge = model.steps[0].boundary_conditions[1].region.clone();
    model.steps[0].boundary_conditions[1] = BoundaryCondition {
        region: edge,
        kind: BoundaryKind::displacement([None, Some(0.0), None, None, None, None]),
        ..model.steps[0].boundary_conditions[1].clone()
    };
    let text = write_inp(&mesh, &model, "").unwrap();
    let free = run_ccx("symmetrie-frei", &text).unwrap();
    let softer = expanded_value(&free, &mesh, ("DISP", "U3"), corner, false);
    assert!(
        softer.abs() > 1.002 * expected.abs(),
        "{softer} vs {expected}"
    );
}

/// Turns the mesh by `angle` degrees about the z axis.
fn turn(mesh: &mut FeMesh, angle: f64) {
    let (s, c) = angle.to_radians().sin_cos();
    for (id, x) in (mesh.node_ids().to_vec())
        .into_iter()
        .zip(mesh.coords().to_vec())
    {
        mesh.set_node(id, [c * x[0] - s * x[1], s * x[0] + c * x[1], x[2]]);
    }
}

/// Solids have no rotations; a skewed symmetry plane holds them like a coordinate plane.
#[test]
fn calculix_holds_solids_in_a_skewed_symmetry_plane_alike() {
    let mut deflections = Vec::new();
    for angle in [0.0, 30.0] {
        let (mut mesh, mut model) = cantilever(tip_force());
        let side = nodes_at(&mesh, 1, 0.0);
        turn(&mut mesh, angle);
        model.steps[0].boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Symmetry-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(side),
            kind: BoundaryKind::Symmetry { normal: None },
            amplitude: None,
        });
        let text = write_inp(&mesh, &model, "").unwrap();
        assert_eq!(text.contains("*Transform"), angle != 0.0, "{text}");
        let Some(frd) = run_ccx(&format!("symmetrie-solid-{angle}"), &text) else {
            return;
        };
        deflections.push(node_value(&frd, "DISP", "U3", 99));
    }
    let [aligned, skewed] = deflections[..] else {
        unreachable!()
    };
    assert!(
        (aligned - skewed).abs() < 1e-5 * aligned.abs(),
        "{aligned} != {skewed}"
    );
}

/// A 2D model holds the line of a symmetry plane in its normal direction only.
#[test]
fn calculix_holds_a_2d_model_in_a_skewed_symmetry_line_alike() {
    let mut tips = Vec::new();
    for angle in [0.0, 30.0] {
        let mut mesh = rectangle(0.0, 10.0, 2.0, 5, 1);
        let left = nodes_at(&mesh, 0, 0.0);
        let bottom = nodes_at(&mesh, 1, 0.0);
        let tip = (nodes_at(&mesh, 0, 10.0).into_iter())
            .find(|&n| mesh.node(n).unwrap()[1] == 2.0)
            .unwrap();
        turn(&mut mesh, angle);
        let (s, c) = angle.to_radians().sin_cos();
        // Pulls along the plate, so that the line moves along itself only.
        let mut model = plane_model(
            ModelSpace::PlaneStress,
            1.0,
            // Held in both directions, which a turned system at the corner keeps.
            vec![(left, [Some(0.0), Some(0.0), None, None, None, None])],
            LoadKind::force([100.0 * c, 100.0 * s, 0.0]),
            Region::Nodes(vec![tip]),
        );
        model.steps[0].boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Symmetry-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(bottom),
            kind: BoundaryKind::Symmetry { normal: None },
            amplitude: None,
        });
        let text = write_inp(&mesh, &model, "").unwrap();
        if angle == 0.0 {
            assert!(text.contains("Symmetry-1, 2, 2, 0\n**"), "{text}");
        }
        let Some(frd) = run_ccx(&format!("symmetrie-2d-{angle}"), &text) else {
            return;
        };
        let u = ["U1", "U2"].map(|c| node_value(&frd, "DISP", c, tip));
        tips.push(c * u[0] + s * u[1]);
    }
    let [aligned, skewed] = tips[..] else {
        unreachable!()
    };
    assert!(aligned > 0.0);
    assert!(
        (aligned - skewed).abs() < 1e-5 * aligned.abs(),
        "{aligned} != {skewed}"
    );
}

/// The element types a part can be given instead of its own, see
/// `plx_model::element_type_options`, are written and run in CalculiX.
#[test]
fn calculix_runs_the_element_types_chosen_for_a_part() {
    let solid = |file: &str, load_node: NodeId| {
        let (mesh, mut model) = analysis(file, tip_force());
        model.steps[0].loads[0].region = Region::Nodes(vec![load_node]);
        (mesh, model)
    };
    let mut cases = Vec::new();
    for type_name in ["C3D8R", "C3D8I"] {
        cases.push((solid("kragbalken_c3d8.inp", 99), "EALL", type_name));
    }
    cases.push((solid("kragbalken_c3d20r.inp", 99), "EALL", "C3D20"));
    cases.push((solid("wuerfel_c3d10.inp", 336), "EALL", "C3D10T"));
    cases.push((shell_plate(), "PLATE", "S8R"));
    let plane = |space| {
        let mesh = rectangle(0.0, 10.0, 2.0, 5, 2);
        let fixed = vec![(nodes_at(&mesh, 0, 0.0), FIX_X), (vec![1], FIX_Y)];
        let tip = Region::Faces(edges_at(&mesh, 0, 10.0));
        let model = plane_model(space, 1.0, fixed, LoadKind::force([100.0, 0.0, 0.0]), tip);
        (mesh, model)
    };
    cases.push((plane(ModelSpace::PlaneStrain), "PLATE", "CPE8R"));
    // Chosen in plane stress, written for the axisymmetric model the model became.
    let (mesh, mut model) = plane(ModelSpace::PlaneStress);
    model.set_element_type("PLATE", "CPS8R");
    model.properties.space = ModelSpace::Axisymmetric;
    cases.push(((mesh, model), "PLATE", "CAX8R"));
    for ((mesh, mut model), part, type_name) in cases {
        model.set_element_type(part, type_name);
        let text = write_inp(&mesh, &model, "").unwrap();
        let block = format!("*Element, Type={type_name}, Elset={part}\n");
        assert!(text.contains(&block), "{block:?} missing in\n{text}");
        // The mesh itself is not changed by writing it.
        assert!(mesh.elements().iter().all(|e| e.type_name != type_name));
        let name = format!("elementtyp-{}", type_name.to_ascii_lowercase());
        let Some(frd) = run_ccx(&name, &text) else {
            return;
        };
        let increment = frd.increments.last().unwrap();
        let values = &increment.field("DISP").unwrap().components[0].values;
        assert!(values.iter().all(|v| v.is_finite()), "{type_name}");
        assert!(values.iter().any(|v| *v != 0.0), "{type_name}");
    }
}

/// The cantilever with a wheel driving along its top face from x = 10 to 90 in five steps.
fn moving_cantilever() -> (FeMesh, FeModel) {
    let mesh = read_inp(&testdata("kragbalken_c3d8.inp")).unwrap().mesh;
    let top = mesh.coords().iter().map(|c| c[2]).fold(f64::MIN, f64::max);
    let mut faces = Vec::new();
    for element in mesh.elements() {
        for (f, topology) in element.faces().iter().enumerate() {
            let on_top =
                (topology.corners.iter()).all(|&l| mesh.node(element.nodes[l]).unwrap()[2] == top);
            if on_top {
                faces.push((element.id, f as u8 + 1));
            }
        }
    }
    let load = Load {
        distribution: None,
        name: "Moving-1".into(),
        active: true,
        visible: true,
        region: Region::Faces(faces),
        kind: LoadKind::Moving(plx_model::MovingLoad {
            start: 10.0,
            end: 90.0,
            positions: 5,
            patch_length: 10.0,
            patch_width: 10.0,
            wheels: vec![plx_model::Wheel {
                along: 0.0,
                across: 5.0,
                force: [0.0, 0.0, -100.0],
            }],
            ..Default::default()
        }),
        amplitude: None,
        factor_amplitude: None,
    };
    let (_, mut model) = cantilever(load);
    // Another load stays the same in every position.
    model.steps[0].loads.insert(0, tip_force());
    (mesh, model)
}

#[test]
fn a_moving_load_writes_a_step_per_position() {
    let (mesh, model) = moving_cantilever();
    let text = write_inp(&mesh, &model, "").unwrap();
    assert_eq!(text.matches("\n*Step").count(), 5, "{text}");
    assert!(text.contains("** Moving load position 1 of 5, Moving-1 at 10\n*Step"));
    assert!(text.contains("** Moving load position 5 of 5, Moving-1 at 90\n*Step"));
    // Each step holds the tip force and the wheel's 100 N spread over the top face.
    for block in text.split("\n*Step").skip(1) {
        let moving = block.split("** Name: Moving-1\n*Cload\n").nth(1).unwrap();
        let total: f64 = (moving.lines())
            .take_while(|l| !l.starts_with('*'))
            .map(|l| l.split(", ").nth(2).unwrap().parse::<f64>().unwrap())
            .sum();
        assert!((total + 100.0).abs() < 1e-9, "{total}");
        assert!(block.contains("** Name: Force-1\n*Cload\n"));
    }
    // The node sets of other items are written once.
    assert_eq!(text.matches("*Nset, Nset=Internal_Selection").count(), 1);
    // Check Model checks every position without solving it.
    let check = write_check_inp(&mesh, &model, "").unwrap();
    assert_eq!(check.matches("*No analysis").count(), 5);
    assert!(!check.contains("*Static"));
}

#[test]
fn a_moving_load_off_its_faces_is_left_out() {
    let (mesh, mut model) = moving_cantilever();
    let LoadKind::Moving(moving) = &mut model.steps[0].loads[1].kind else {
        unreachable!()
    };
    moving.end = 200.0;
    let text = write_inp(&mesh, &model, "").unwrap();
    let last = text.split("\n*Step").last().unwrap();
    assert!(
        last.contains("** Name: Moving-1: No wheel on the region\n"),
        "{last}"
    );
}

#[test]
fn moving_load_stations_follow_the_steps_the_writer_gives() {
    let (mesh, mut model) = moving_cantilever();
    // A plain step before, a deactivated one in between and one after the moving load.
    let mut moving = model.steps.remove(0);
    let mut plain = moving.clone();
    plain
        .loads
        .retain(|l| !matches!(l.kind, LoadKind::Moving(_)));
    let mut off = plain.clone();
    off.name = "Off".into();
    off.active = false;
    moving.name = "Step-2".into();
    let mut last = plain.clone();
    last.name = "Step-3".into();
    model.steps = vec![plain, off, moving, last];
    let text = write_inp(&mesh, &model, "").unwrap();
    assert_eq!(text.matches("\n*Step").count(), 7, "{text}");
    let stations = plx_model::load_stations(&model);
    assert_eq!(stations.len(), 1);
    assert_eq!(stations[0].load, "Moving-1");
    assert_eq!(
        stations[0].steps,
        [(2, 10.0), (3, 30.0), (4, 50.0), (5, 70.0), (6, 90.0)]
    );
}

#[test]
fn calculix_bends_the_cantilever_more_as_the_moving_load_drives_out() {
    let (mesh, mut model) = moving_cantilever();
    model.steps[0].loads.remove(0);
    let Some(frd) = run_ccx("moving", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let index = frd.mesh.node_index(99).unwrap();
    let tip: Vec<f64> = (frd.increments.iter())
        .filter_map(|i| i.field("DISP"))
        .map(|f| f64::from(f.component("U3").unwrap().values[index]))
        .collect();
    assert_eq!(tip.len(), 5, "{tip:?}");
    assert!(tip.windows(2).all(|w| w[1] < w[0] && w[1] < 0.0), "{tip:?}");
}

/// The cantilever with a sensitivity step after its static step, the design region on the
/// faces of the elements at the fixed end.
fn sensitivity_cantilever(settings: SensitivityStep) -> (FeMesh, FeModel) {
    let (mesh, mut model) = cantilever(tip_force());
    let mut step = Step::new_sensitivity("Step-2");
    step.kind = StepKind::Sensitivity(SensitivityStep {
        design_region: Region::NodeSet("NALL".into()),
        ..settings
    });
    // Supports and loads of the static step stay; the sensitivity step writes none.
    step.boundary_conditions = model.steps[0].boundary_conditions.clone();
    model.steps.push(step);
    (mesh, model)
}

/// Runs CalculiX and reads the `.frd` and `.dat` files.
fn run_ccx_frd_dat(name: &str, text: &str) -> Option<(FrdImport, crate::dat::DatImport)> {
    let dir = ccx_dir(name)?;
    let frd = run_ccx_in(&dir, name, text);
    let dat = crate::dat::read_dat(&dir.join(format!("{name}.dat"))).unwrap();
    remove_ccx_dir(&dir);
    Some((frd, dat))
}

#[test]
fn a_sensitivity_step_writes_design_variables_and_responses() {
    let (mesh, mut model) = sensitivity_cantilever(SensitivityStep {
        filter_radius: Some(15.0),
        ..SensitivityStep::default()
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    let design = text
        .find("*Design variables, Type=Coordinate\nNALL\n")
        .unwrap();
    assert!(design < text.find("*Step").unwrap(), "{text}");
    let step = &text[text.find("** Step-2").unwrap()..];
    for line in [
        "*Step\n*Sensitivity\n",
        "*Design response, Name=MASS\nMASS, Internal_Selection-1_Step-2\n",
        "*Design response, Name=STRAINENERGY\nSTRAINENERGY, Internal_Selection-1_Step-2\n",
        "*Filter, Type=Linear\n15\n",
        "*Node file\nSEN\n",
    ] {
        assert!(step.contains(line), "missing {line:?} in\n{step}");
    }
    // The step evaluates the static solution: no supports, loads or their resets.
    assert!(!step.contains("*Boundary"), "{step}");
    assert!(!step.contains("*Cload"), "{step}");
    // Eigenfrequencies come alone, without an element set.
    let StepKind::Sensitivity(settings) = &mut model.steps[1].kind else {
        unreachable!()
    };
    (
        settings.mass,
        settings.strain_energy,
        settings.eigenfrequency,
    ) = (false, false, true);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Design response, Name=EIGENFREQUENCY\nEIGENFREQUENCY\n"),
        "{text}"
    );
}

/// The mass is density times volume, the strain energy of the static step half the work of
/// the tip force; CalculiX prints both with their gradients and writes the sensitivities.
#[test]
fn calculix_computes_the_sensitivities_of_mass_and_strain_energy() {
    let (mesh, model) = sensitivity_cantilever(SensitivityStep::default());
    let Some((frd, dat)) = run_ccx_frd_dat("sensitivitaet", &write_inp(&mesh, &model, "").unwrap())
    else {
        return;
    };
    let index = frd.mesh.node_index(99).unwrap();
    let disp = frd.increments[0].field("DISP").unwrap();
    let tip = -f64::from(disp.component("U3").unwrap().values[index]);
    let set = dat.sets.iter().find(|s| s.name == "STEP_2").unwrap();
    let response = |name: &str| {
        let field = set.field("DESIGN_RESPONSE").unwrap();
        field.component(name).unwrap().entries[0].values[0]
    };
    let mass = 7.85e-9 * 100.0 * 10.0 * 10.0;
    assert!(
        (response("MASS") - mass).abs() < 1e-6 * mass,
        "{}",
        response("MASS")
    );
    let energy = 0.5 * 100.0 * tip;
    let strain_energy = response("STRAINENERGY");
    assert!(
        (strain_energy - energy).abs() < 1e-3 * energy,
        "{strain_energy} != {energy}"
    );
    let sensitivity = (frd.increments.iter())
        .find(|i| i.step == 2 && i.kind == plx_results::AnalysisKind::Sensitivity)
        .unwrap();
    for field in ["NORM", "SENMASS", "SENENER"] {
        assert!(sensitivity.field(field).is_some(), "{field}");
    }
    let sen = sensitivity
        .field("SENENER")
        .unwrap()
        .component("DFDN")
        .unwrap();
    assert!(sen.values.iter().any(|v| *v != 0.0));
}

/// After a frequency step, the sensitivities of the eigenfrequencies come as one increment
/// per mode with its frequency.
#[test]
fn calculix_computes_the_sensitivities_of_the_eigenfrequencies() {
    let (mesh, mut model) = cantilever(tip_force());
    let mut frequency = Step::new_frequency("Step-1");
    frequency.boundary_conditions = model.steps[0].boundary_conditions.clone();
    let StepKind::Frequency(settings) = &mut frequency.kind else {
        unreachable!()
    };
    settings.num_frequencies = 4;
    // CalculiX reads the modes from the .eig file.
    settings.storage = true;
    let mut step = Step::new_sensitivity("Step-2");
    step.kind = StepKind::Sensitivity(SensitivityStep {
        design_region: Region::NodeSet("NALL".into()),
        mass: false,
        strain_energy: false,
        eigenfrequency: true,
        filter_radius: None,
    });
    model.steps = vec![frequency, step];
    let Some(frd) = run_ccx(
        "sensitivitaet-frequenz",
        &write_inp(&mesh, &model, "").unwrap(),
    ) else {
        return;
    };
    let frequencies: Vec<f64> = (frd.increments.iter())
        .filter(|i| i.step == 1 && i.kind == plx_results::AnalysisKind::Frequency)
        .map(|i| i.value)
        .collect();
    let modes: Vec<_> = (frd.increments.iter())
        .filter(|i| i.step == 2 && i.kind == plx_results::AnalysisKind::Sensitivity)
        .collect();
    assert_eq!(modes.len(), 4);
    for (mode, f) in modes.iter().zip(&frequencies) {
        assert!((mode.value - f).abs() < 1e-3 * f, "{} vs. {f}", mode.value);
        assert!(mode.field("SENFREQ").is_some());
    }
}

#[test]
fn a_visco_step_and_a_creep_law_are_written() {
    let (mesh, mut model) = cantilever(tip_force());
    model.materials[0].creep = Some(plx_model::Creep {
        a: 1e-15,
        n: 5.0,
        m: 0.0,
    });
    let mut step = Step::new_visco("Step-2");
    step.boundary_conditions = model.steps[0].boundary_conditions.clone();
    step.loads = model.steps[0].loads.clone();
    model.steps.push(step);
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*Elastic\n210000, 0.3\n*Creep, Law=Norton\n1.00000000E-15, 5, 0\n",
        "*Step, Inc=1000\n*Visco, Cetol=0.0001\n1, 1000, 0.00001, 1.00000000E30\n",
        "*El file\nS, E, PEEQ, NOE\n",
    ] {
        assert!(text.contains(line), "{line:?} missing in\n{text}");
    }
}

/// A bar under a constant stress creeps by Norton's law: after the time t of the visco
/// step its creep strain is A stress^n t.
#[test]
fn calculix_creeps_the_bar_by_nortons_law() {
    let stress = 100.0;
    let load = Load {
        distribution: None,
        name: "Pressure-1".into(),
        active: true,
        visible: true,
        region: Region::Surface("TIP".into()),
        kind: LoadKind::Pressure(-stress),
        amplitude: None,
        factor_amplitude: None,
    };
    let (mesh, mut model) = cantilever(load);
    let creep = plx_model::Creep {
        a: 1e-15,
        n: 5.0,
        m: 0.0,
    };
    model.materials[0].creep = Some(creep);
    // Held on three symmetry planes, so that the bar is in uniaxial tension.
    let (min, max) = mesh.bounds().unwrap();
    let nodes_at = |axis: usize, value: f64| -> Vec<NodeId> {
        (1..=99)
            .filter(|&id| mesh.node(id).is_some_and(|c| c[axis] == value))
            .collect()
    };
    let hold = |axis: usize| {
        let mut dofs = [None; 6];
        dofs[axis] = Some(0.0);
        BoundaryCondition {
            distribution: None,
            name: format!("Symmetry-{}", axis + 1),
            active: true,
            visible: true,
            region: Region::Nodes(nodes_at(axis, min[axis])),
            kind: each_node(dofs),
            amplitude: None,
        }
    };
    model.steps[0].boundary_conditions = vec![hold(0), hold(1), hold(2)];
    model.steps[0].field_outputs = FieldOutput::visco_defaults();
    // The static step loads the bar at once; the visco step keeps the load for 1000 s.
    let mut step = Step::new_visco("Step-2");
    step.boundary_conditions = model.steps[0].boundary_conditions.clone();
    step.loads = model.steps[0].loads.clone();
    model.steps.push(step);
    let Some(frd) = run_ccx("kriechen", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let time = 1000.0;
    let expected = creep.a * stress.powf(creep.n) * time;
    let last = frd.increments.last().unwrap();
    assert_eq!(last.step, 2);
    // The analysis time runs on from the end of the static step.
    assert!(
        (last.value - 1.0 - time).abs() < 1e-6 * time,
        "{}",
        last.value
    );
    let elastic = stress / 210_000.0;
    let index = |node| frd.mesh.node_index(node).unwrap();
    for node in nodes_at(0, max[0]) {
        let s = last
            .field("STRESS")
            .unwrap()
            .component("S11")
            .unwrap()
            .values[index(node)];
        assert!((f64::from(s) - stress).abs() < 0.01 * stress, "{s}");
        let peeq = last.field("PE").unwrap().component("PE").unwrap().values[index(node)];
        let peeq = f64::from(peeq);
        assert!(
            (peeq - expected).abs() < 0.02 * expected,
            "{peeq} != {expected}"
        );
        let strain = last
            .field("TOSTRAIN")
            .unwrap()
            .component("E11")
            .unwrap()
            .values[index(node)];
        let strain = f64::from(strain);
        assert!(
            (strain - elastic - expected).abs() < 0.02 * expected,
            "{strain} != {elastic} + {expected}"
        );
    }
    // The static step does not creep.
    let first = frd.increments.iter().find(|i| i.step == 1).unwrap();
    let peeq = first.field("PE").unwrap().component("PE").unwrap().values[index(1)];
    assert!(f64::from(peeq).abs() < 1e-3 * expected, "{peeq}");
}

/// An annular steel disk of C3D8 elements, 20 to 60 mm in radius and 4 mm thick, held at
/// its bore: the whole disk, or one of `sectors` sectors with its two cut faces.
fn disk(sectors: u32, full: bool) -> (FeMesh, FeModel, Faces, Faces) {
    let (nr, nt, nz) = (6, 4, 2);
    let angle = std::f64::consts::TAU / f64::from(sectors);
    let columns = if full { nt * sectors } else { nt + 1 };
    let id = |i: u32, j: u32, k: u32| 1 + i + (nr + 1) * ((j % columns) + columns * k);
    let mut text = String::from("*Node\n");
    for k in 0..=nz {
        for j in 0..columns {
            for i in 0..=nr {
                let (r, t) = (
                    20.0 + 40.0 * f64::from(i) / f64::from(nr),
                    angle * f64::from(j) / f64::from(nt),
                );
                let z = 4.0 * f64::from(k) / f64::from(nz);
                text += &format!("{}, {}, {}, {z}\n", id(i, j, k), r * t.cos(), r * t.sin());
            }
        }
    }
    text += "*Element, Type=C3D8, Elset=EALL\n";
    let (mut left, mut right, mut element) = (Vec::new(), Vec::new(), 0);
    let rows = if full { nt * sectors } else { nt };
    for k in 0..nz {
        for j in 0..rows {
            for i in 0..nr {
                element += 1;
                let nodes = [
                    id(i, j, k),
                    id(i + 1, j, k),
                    id(i + 1, j + 1, k),
                    id(i, j + 1, k),
                    id(i, j, k + 1),
                    id(i + 1, j, k + 1),
                    id(i + 1, j + 1, k + 1),
                    id(i, j + 1, k + 1),
                ]
                .map(|n| n.to_string());
                text += &format!("{element}, {}\n", nodes.join(", "));
                // Face S3 runs through nodes 1, 2, 6, 5, face S5 through 3, 4, 8, 7.
                if j == 0 {
                    left.push((element, 3));
                }
                if j == rows - 1 {
                    right.push((element, 5));
                }
            }
        }
    }
    let bore: Vec<String> = (0..=nz)
        .flat_map(|k| (0..columns).map(move |j| id(0, j, k).to_string()))
        .collect();
    text += &format!("*Nset, Nset=FIX\n{}\n", bore.join(", "));
    let mesh = read_inp_str(&text, None).unwrap().mesh;
    let mut model = cantilever(tip_force()).1;
    let mut step = Step::new_frequency("Step-1");
    step.boundary_conditions = model.steps[0].boundary_conditions.clone();
    model.steps = vec![step];
    (mesh, model, left, right)
}

#[test]
fn a_cyclic_symmetry_tie_is_written_like_prepomax_does() {
    let (mesh, mut model, left, right) = disk(12, false);
    let mut tie = plx_model::Tie::new("Tie-1");
    tie.master = Region::Faces(left);
    tie.slave = Region::Faces(right);
    tie.cyclic = Some(plx_model::CyclicSymmetry::default());
    model.ties.push(tie);
    // The number of sectors follows from the 30 degrees between the cut faces, and all
    // of them show by default.
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Cyclic symmetry model, N=12, Tie=Tie-1, Ngraph=12\n0, 0, 0, 0, 0, 1\n"),
        "{text}"
    );
    model.ties[0].cyclic.as_mut().unwrap().display = plx_model::SectorDisplay::Single;
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in [
        "*Tie, Name=Tie-1, Cyclic symmetry\n",
        "*Cyclic symmetry model, N=12, Tie=Tie-1\n0, 0, 0, 0, 0, 1\n",
        "*Frequency\n20\n*Select cyclic symmetry modes\n",
    ] {
        assert!(text.contains(line), "{line:?} missing in\n{text}");
    }
    let StepKind::Frequency(settings) = &mut model.steps[0].kind else {
        unreachable!()
    };
    settings.nodal_diameters = Some([1, 3]);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Select cyclic symmetry modes, Nmin=1, Nmax=3\n"),
        "{text}"
    );
    // The bore nodes of the slave side follow the master side; CalculiX refuses them in
    // a boundary condition as well.
    assert!(text.contains("*Boundary\nInternal_Selection-1_Fixed-1, 1, 6, 0\n"));
    let set = "*Nset, Nset=Internal_Selection-1_Fixed-1\n";
    let nodes = &text[text.find(set).unwrap() + set.len()..];
    let nodes: Vec<&str> = nodes.lines().next().unwrap().split(", ").collect();
    assert_eq!(nodes.len(), 3 * 4, "{nodes:?}");
    // Cut faces that are not a whole fraction of the circle about the axis do not export.
    model.ties[0].cyclic.as_mut().unwrap().axis_direction = [1.0, 0.0, 0.0];
    assert!(matches!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::InvalidCyclicSymmetry { .. })
    ));
}

/// The sector computes the frequencies of the whole disk, sorted by nodal diameter.
#[test]
fn calculix_finds_the_disk_frequencies_on_a_sector() {
    let (mesh, mut model, _, _) = disk(12, true);
    let Some(dir) = ccx_dir("zyklisch") else {
        return;
    };
    let full = run_ccx_in(&dir, "voll", &write_inp(&mesh, &model, "").unwrap());
    let whole: Vec<f64> = full.increments.iter().map(|i| i.value).collect();
    let (mesh, sector_model, left, right) = disk(12, false);
    model = sector_model;
    let mut tie = plx_model::Tie::new("Tie-1");
    tie.master = Region::Faces(left);
    tie.slave = Region::Faces(right);
    tie.cyclic = Some(plx_model::CyclicSymmetry::default());
    model.ties.push(tie);
    let StepKind::Frequency(settings) = &mut model.steps[0].kind else {
        unreachable!()
    };
    settings.num_frequencies = 2;
    settings.nodal_diameters = Some([0, 2]);
    let frd = run_ccx_in(&dir, "sektor", &write_inp(&mesh, &model, "").unwrap());
    let dat = std::fs::read_to_string(dir.join("sektor.dat")).unwrap();
    remove_ccx_dir(&dir);
    // The results are expanded to the whole disk.
    assert_eq!(frd.mesh.node_count(), 12 * mesh.node_count());
    let modes: Vec<(Option<u32>, u32, f64)> = (frd.increments.iter())
        .map(|i| (i.nodal_diameter, i.increment, i.value))
        .collect();
    let diameters: Vec<_> = modes.iter().map(|m| m.0).collect();
    assert_eq!(diameters, [0, 0, 1, 1, 2, 2].map(Some), "{modes:?}");
    let increments: Vec<_> = modes.iter().map(|m| m.1).collect();
    assert_eq!(increments, [1, 2, 3, 4, 5, 6], "{modes:?}");
    // The lowest frequency of each nodal diameter is one of the whole disk.
    for (_, _, f) in modes.iter().step_by(2) {
        assert!(
            whole.iter().any(|w| (w - f).abs() < 1e-4 * f),
            "{f} Hz not in {whole:?}"
        );
    }
    // The .dat file numbers the modes on through the nodal diameters, as the .frd does.
    let import = crate::dat::parse_dat(&dat);
    let set = import.sets.iter().find(|s| s.name == "STEP_1").unwrap();
    let field = (set.fields.iter())
        .find(|f| f.name == "EIGENVALUE_OUTPUT")
        .unwrap();
    let nd = (field.components.iter())
        .find(|c| c.name == "NODAL_DIAMETER")
        .unwrap();
    let rows: Vec<f64> = nd.entries[0].values.clone();
    assert_eq!(rows, [0.0, 0.0, 1.0, 1.0, 2.0, 2.0], "{rows:?}");
}

/// A steel block of 2 x 2 hexes, 10 x 10 x 10, from z = 10 to 20, standing on a vertical
/// plate of 2 x 2 elements of the type, 10 x 10 and 1 thick, in the plane x = 5, held at its
/// bottom. Tie-1 glues the top edges of the plate (S5) to the bottom of the block (S1), and
/// a pressure of 10 on the block's top pushes 1000 N through the plate.
fn block_on_plate(plate_type: &str) -> (FeMesh, FeModel) {
    let mut mesh = FeMesh::default();
    let block = |i: u32, j: u32, k: u32| 1 + i + 3 * j + 9 * k;
    for k in 0..2 {
        for j in 0..3 {
            for i in 0..3 {
                let coords = [
                    5.0 * f64::from(i),
                    5.0 * f64::from(j),
                    10.0 + 10.0 * f64::from(k),
                ];
                mesh.set_node(block(i, j, k), coords);
            }
        }
    }
    for j in 0..2 {
        for i in 0..2 {
            let mut nodes = Vec::new();
            for k in 0..2 {
                nodes.extend([block(i, j, k), block(i + 1, j, k)]);
                nodes.extend([block(i + 1, j + 1, k), block(i, j + 1, k)]);
            }
            mesh.add_element(plx_mesh::Element {
                id: 1 + i + 2 * j,
                type_name: "C3D8".into(),
                shape: plx_mesh::ElementShape::Hex8,
                nodes,
            })
            .unwrap();
        }
    }
    let plate = |j: u32, k: u32| 101 + j + 3 * k;
    for k in 0..3 {
        for j in 0..3 {
            mesh.set_node(plate(j, k), [5.0, 5.0 * f64::from(j), 5.0 * f64::from(k)]);
        }
    }
    for k in 0..2 {
        for j in 0..2 {
            mesh.add_element(plx_mesh::Element {
                id: 201 + j + 2 * k,
                type_name: plate_type.into(),
                shape: plx_mesh::ElementShape::Quad4,
                nodes: vec![
                    plate(j, k),
                    plate(j + 1, k),
                    plate(j + 1, k + 1),
                    plate(j, k + 1),
                ],
            })
            .unwrap();
        }
    }
    for (name, elements) in [
        ("BLOCK", vec![1, 2, 3, 4]),
        ("PLATE", vec![201, 202, 203, 204]),
    ] {
        mesh.parts.push(plx_mesh::Part {
            name: name.into(),
            elements,
        });
    }
    let mut step = Step::new_static("Step-1");
    let bc = |name: &str, nodes: Vec<NodeId>, kind| BoundaryCondition {
        distribution: None,
        name: name.into(),
        active: true,
        visible: true,
        region: Region::Nodes(nodes),
        kind,
        amplitude: None,
    };
    step.boundary_conditions.push(bc(
        "Fixed-1",
        (101..=103).collect(),
        BoundaryKind::displacement([Some(0.0); 6]),
    ));
    // A membrane holds the block in its plane only.
    let sideways = BoundaryKind::displacement([Some(0.0), Some(0.0), None, None, None, None]);
    step.boundary_conditions
        .push(bc("Guided-1", (10..=18).collect(), sideways));
    step.loads.push(Load {
        distribution: None,
        name: "Pressure-1".into(),
        active: true,
        visible: true,
        region: Region::Faces((1..=4).map(|e| (e, 2)).collect()),
        kind: LoadKind::Pressure(10.0),
        amplitude: None,
        factor_amplitude: None,
    });
    let mut model = FeModel::default();
    model.materials.push(Material {
        name: "Steel".into(),
        elastic: Some(Elastic {
            young: 210_000.0,
            poisson: 0.3,
        }),
        ..Default::default()
    });
    let plate_kind = if plate_type.starts_with('M') {
        SectionKind::Membrane {
            thickness: 1.0,
            offset: 0.0,
        }
    } else {
        SectionKind::Shell {
            thickness: 1.0,
            offset: 0.0,
        }
    };
    for (name, part, kind) in [
        ("Section-1", "BLOCK", SectionKind::Solid),
        ("Section-2", "PLATE", plate_kind),
    ] {
        model.sections.push(Section {
            name: name.into(),
            material: "Steel".into(),
            region: Region::Parts(vec![part.into()]),
            thickness: 1.0,
            kind,
        });
    }
    let mut tie = plx_model::Tie::new("Tie-1");
    tie.master = Region::Faces((1..=4).map(|e| (e, 1)).collect());
    tie.slave = Region::Faces(vec![(203, 5), (204, 5)]);
    model.ties.push(tie);
    model.steps.push(step);
    (mesh, model)
}

#[test]
fn the_edge_of_a_shell_or_membrane_tied_to_a_solid_carries_its_load() {
    for plate_type in ["S4", "M3D4"] {
        let (mesh, model) = block_on_plate(plate_type);
        let text = write_inp(&mesh, &model, "").unwrap();
        // CalculiX ties membrane edges by their nodes only.
        let slave = if plate_type == "S4" {
            "*Surface, Name=Internal_Selection-1_Tie-1_Slave, Type=Element\n\
             Internal-1_Internal_Selection-1_Tie-1_Slave_S5, S5\n"
        } else {
            "*Surface, Name=Internal_Selection-1_Tie-1_Slave, Type=Node\n\
             Internal-1_Internal_Selection-1_Tie-1_Slave\n"
        };
        assert!(
            text.contains(slave),
            "{plate_type}: {slave:?} missing in\n{text}"
        );
        let Some(frd) = run_ccx(&format!("edge-tie-{plate_type}"), &text) else {
            return;
        };
        // 1000 N through 10 x 1 of plate 10 long, and the block: about 0.0052 down.
        let u3 = node_value(&frd, "DISP", "U3", 14);
        assert!((-0.007..-0.004).contains(&u3), "{plate_type}: U3 = {u3}");
    }
}

#[test]
fn calculix_bends_a_box_of_a_mesh_block_held_and_loaded_on_its_sides() {
    // A box 100 x 10 x 10 held at its side X- and pressed down on its side Z+ by 1 MPa:
    // a cantilever under q = 10 N/mm, whose tip sinks by q L^4 / (8 E I).
    let primitive = plx_model::MeshPrimitive::Box {
        origin: [0.0; 3],
        size: [100.0, 10.0, 10.0],
        divisions: [20, 2, 2],
        element: "C3D20R".into(),
    };
    let mesh = primitive.generate(ModelSpace::ThreeD, "BOX");
    let load = Load {
        distribution: None,
        name: "Pressure-1".into(),
        active: true,
        visible: true,
        region: Region::Surface("Z+".into()),
        kind: LoadKind::Pressure(1.0),
        amplitude: None,
        factor_amplitude: None,
    };
    let (_, mut model) = cantilever(load);
    model.sections[0].region = Region::Parts(vec!["BOX".into()]);
    model.steps[0].boundary_conditions[0].region = Region::NodeSet("X-".into());
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Surface, Name=Z+, Type=Element\n"), "{text}");
    let Some(frd) = run_ccx("box-block", &text) else {
        return;
    };
    let increment = frd.increments.last().unwrap();
    let u3 = increment.field("DISP").unwrap().component("U3").unwrap();
    let tip = f64::from(u3.values.iter().copied().fold(f32::MAX, f32::min));
    let inertia = 10.0 * 10.0_f64.powi(3) / 12.0;
    let expected = -10.0 * 100.0_f64.powi(4) / (8.0 * 210_000.0 * inertia);
    assert!(
        (tip - expected).abs() < 0.03 * expected.abs(),
        "{tip} against {expected}"
    );
}

#[test]
fn calculix_bends_a_rectangle_of_a_mesh_block_held_and_loaded_on_its_edges() {
    // The box above as a plate in plane stress, 10 thick, pressed down on its edge Y+.
    let primitive = plx_model::MeshPrimitive::Rectangle {
        origin: [0.0; 2],
        size: [100.0, 10.0],
        divisions: [20, 2],
        quadratic: true,
        reduced: true,
    };
    let space = ModelSpace::PlaneStress;
    let mesh = primitive.generate(space, "RECTANGLE");
    let mut model = plane_model(
        space,
        10.0,
        Vec::new(),
        LoadKind::Pressure(1.0),
        Region::Surface("Y+".into()),
    );
    model.sections[0].region = Region::Parts(vec!["RECTANGLE".into()]);
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Fixed-1".into(),
        active: true,
        visible: true,
        region: Region::NodeSet("X-".into()),
        kind: BoundaryKind::displacement([Some(0.0); 6]),
        amplitude: None,
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    let Some(frd) = run_ccx("rectangle-block", &text) else {
        return;
    };
    let inertia = 10.0 * 10.0_f64.powi(3) / 12.0;
    let expected = -10.0 * 100.0_f64.powi(4) / (8.0 * 210_000.0 * inertia);
    let tip = min_u2(&frd);
    assert!(
        (tip - expected).abs() < 0.03 * expected.abs(),
        "{tip} against {expected}"
    );
}

/// The point of a remote displacement in a plane model turns about z: the right end of a
/// strip held at its left end turns about its centre.
#[test]
fn calculix_turns_the_remote_point_of_a_plane_model() {
    let angle = 0.01;
    let mesh = rectangle(0.0, 10.0, 2.0, 5, 2);
    let mut model = plane_model(
        ModelSpace::PlaneStress,
        1.0,
        vec![(nodes_at(&mesh, 0, 0.0), [Some(0.0); 6])],
        LoadKind::force([0.0, 0.0, 0.0]),
        Region::Nodes(vec![1]),
    );
    model.steps[0].loads.clear();
    let values = [
        Some(0.0),
        Some(0.0),
        Some(1.0),
        Some(1.0),
        Some(1.0),
        Some(angle),
    ];
    let mut bc = remote("Remote-1", values);
    bc.region = Region::Nodes(nodes_at(&mesh, 0, 10.0));
    model.steps[0].boundary_conditions.push(bc);
    let text = write_inp(&mesh, &model, "").unwrap();
    // U3, UR1 and UR2 do not exist in a plane model.
    assert!(
        text.contains("** Name: Remote-1\n*Boundary\n56, 1, 2, 0\n57, 3, 3, 0.01\n**"),
        "{text}"
    );
    let Some(frd) = run_ccx("remote-2d", &text) else {
        return;
    };
    for node in nodes_at(&mesh, 0, 10.0) {
        let y = mesh.node(node).unwrap()[1];
        let u1 = node_value(&frd, "DISP", "U1", node);
        let u2 = node_value(&frd, "DISP", "U2", node);
        let expected = -angle * (y - 1.0);
        assert!((u1 - expected).abs() < 1e-7, "{u1} != {expected}");
        assert!(u2.abs() < 1e-7, "{u2}");
    }
}

/// The reference point of a rigid body in a plane model turns about z with a Displacement/
/// Rotation boundary condition: UR3 is the third degree of freedom of its rotation node.
#[test]
fn calculix_turns_the_reference_point_of_a_plane_model() {
    let angle = 0.01;
    let mesh = rectangle(0.0, 10.0, 2.0, 5, 2);
    let mut model = plane_model(
        ModelSpace::PlaneStress,
        1.0,
        vec![(nodes_at(&mesh, 0, 0.0), [Some(0.0); 6])],
        LoadKind::force([0.0, 0.0, 0.0]),
        Region::Nodes(vec![1]),
    );
    model.steps[0].loads.clear();
    model
        .reference_points
        .push(ReferencePoint::new("RP-1", [10.0, 1.0, 0.0]));
    let mut body = RigidBody::new("Rigid_Body-1", "RP-1");
    body.region = Region::Nodes(nodes_at(&mesh, 0, 10.0));
    model.constraints.push(Constraint::RigidBody(body));
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Turn-1".into(),
        active: true,
        visible: true,
        region: Region::ReferencePoint("RP-1".into()),
        // U3, UR1 and UR2 do not exist in a plane model and are left out.
        kind: BoundaryKind::displacement([
            Some(0.0),
            Some(0.0),
            Some(1.0),
            Some(1.0),
            Some(1.0),
            Some(angle),
        ]),
        amplitude: None,
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("** Name: Turn-1\n*Boundary\n") && text.contains(", 3, 3, 0.01\n"),
        "{text}"
    );
    let Some(frd) = run_ccx("referenzpunkt-2d", &text) else {
        return;
    };
    for node in nodes_at(&mesh, 0, 10.0) {
        let y = mesh.node(node).unwrap()[1];
        let u1 = node_value(&frd, "DISP", "U1", node);
        let u2 = node_value(&frd, "DISP", "U2", node);
        let expected = -angle * (y - 1.0);
        assert!((u1 - expected).abs() < 1e-7, "{u1} != {expected}");
        assert!(u2.abs() < 1e-7, "{u2}");
    }
}

/// A deformable point of a plane model turns the end of the strip by the mean of its nodes.
#[test]
fn calculix_turns_a_deformable_remote_point_of_a_plane_model() {
    let angle = 0.01;
    let mesh = rectangle(0.0, 10.0, 2.0, 5, 2);
    let mut model = plane_model(
        ModelSpace::PlaneStress,
        1.0,
        vec![(nodes_at(&mesh, 0, 0.0), [Some(0.0); 6])],
        LoadKind::force([0.0, 0.0, 0.0]),
        Region::Nodes(vec![1]),
    );
    model.steps[0].loads.clear();
    let mut remote = Displacement::new([Some(0.0), Some(0.0), None, None, None, Some(angle)]);
    remote.behavior = Behavior::Deformable;
    let mut bc = self::remote("Remote-1", [None; 6]);
    bc.kind = BoundaryKind::Displacement(remote);
    bc.region = Region::Nodes(nodes_at(&mesh, 0, 10.0));
    model.steps[0].boundary_conditions.push(bc);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(!text.contains("*Rigid body"), "{text}");
    let Some(frd) = run_ccx("remote-2d-deformable", &text) else {
        return;
    };
    let nodes = nodes_at(&mesh, 0, 10.0);
    let (mut moment, mut inertia) = (0.0, 0.0);
    for &node in &nodes {
        let y = mesh.node(node).unwrap()[1] - 1.0;
        moment -= y * node_value(&frd, "DISP", "U1", node);
        inertia += y * y;
    }
    let turned = moment / inertia;
    assert!((turned - angle).abs() < 1e-5 * angle, "{turned} != {angle}");
}

/// Every behavior runs on the midside nodes of a quadratic mesh; the deformable point turns
/// by the mean rotation of the end face.
#[test]
fn calculix_drives_a_quadratic_end_face_with_every_behavior() {
    let angle = 0.01;
    for behavior in [Behavior::Rigid, Behavior::Deformable, Behavior::EachNode] {
        let (mesh, mut model) = analysis("kragbalken_c3d20r.inp", tip_force());
        model.steps[0].loads.clear();
        let mut bc = remote_with(
            [Some(0.0), Some(0.0), Some(0.01), None, None, Some(angle)],
            |r| r.behavior = behavior,
        );
        bc.region = Region::NodeSet("TIP".into());
        model.steps[0].boundary_conditions.push(bc);
        let text = write_inp(&mesh, &model, "").unwrap();
        let Some(frd) = run_ccx(&format!("remote-c3d20r-{}", behavior.label()), &text) else {
            return;
        };
        // The shift along z: of every node, or of their weighted mean when deformable.
        let bc = &model.steps[0].boundary_conditions[1];
        let BoundaryKind::Displacement(remote) = &bc.kind else {
            unreachable!()
        };
        let point = remote
            .resolve(&model, &mesh, &bc.region, &Default::default())
            .unwrap();
        let u3: Vec<f64> = (point.nodes.iter())
            .map(|&n| node_value(&frd, "DISP", "U3", n))
            .collect();
        let total: f64 = point.weights.iter().sum();
        let mean = u3
            .iter()
            .zip(&point.weights)
            .map(|(u, w)| u * w)
            .sum::<f64>()
            / total;
        assert!((mean - 0.01).abs() < 1e-6, "{}: {mean}", behavior.label());
        if behavior != Behavior::Deformable {
            assert!(u3.iter().all(|u| (u - 0.01).abs() < 1e-6), "{u3:?}");
        }
        if behavior == Behavior::Deformable {
            let turned = mean_rotation(&mesh, &model, &frd, 2);
            assert!((turned - angle).abs() < 1e-5 * angle, "{turned} != {angle}");
        }
    }
}

#[test]
fn a_point_history_output_prints_its_reference_and_rotation_node() {
    let (mesh, mut model) = rigid_tip(None, None);
    (model.steps[0].history_outputs).push(HistoryOutput::node(
        "NH_Output-1",
        Region::ReferencePoint("RP-1".into()),
    ));
    let mut temperatures =
        HistoryOutput::node("NH_Output-2", Region::ReferencePoint("RP-1".into()));
    temperatures.variables = vec!["NT".into()];
    model.steps[0].history_outputs.push(temperatures);
    let text = write_inp(&mesh, &model, "").unwrap();
    for expected in [
        "*Nset, Nset=Internal_Selection-1_NH_Output-1\n100\n",
        "*Nset, Nset=Internal_Rotation-1_NH_Output-1\n101\n",
        "** Name: NH_Output-1\n\
         *Node print, Nset=Internal_Selection-1_NH_Output-1\nRF, U\n\
         *Node print, Nset=Internal_Rotation-1_NH_Output-1\nRF, U\n\
         ** Name: NH_Output-2\n\
         *Node print, Nset=Internal_Selection-1_NH_Output-2\nNT\n**\n",
    ] {
        assert!(text.contains(expected), "{expected}\nmissing in\n{text}");
    }
    // Without a rigid body the point has no node.
    model.constraints.clear();
    assert_eq!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::UnknownReferencePoint {
            item: "NH_Output-1".into(),
            point: "RP-1".into(),
        })
    );
}

#[test]
fn a_remote_point_history_output_prints_the_nodes_of_the_point() {
    let values = [Some(0.0), None, None, None, None, None];
    let (mesh, mut model) = remote_cantilever(remote_with(values, |r| {
        r.behavior = Behavior::Deformable;
    }));
    (model.steps[0].history_outputs).push(HistoryOutput::node(
        "NH_Output-1",
        Region::RemotePoint("Remote-1".into()),
    ));
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Nset, Nset=Internal_Selection-1_NH_Output-1\n100\n"),
        "{text}"
    );
    assert!(
        text.contains("*Nset, Nset=Internal_Rotation-1_NH_Output-1\n101\n"),
        "{text}"
    );
    // A displacement held at each node has no point.
    if let BoundaryKind::Displacement(d) = &mut model.steps[0].boundary_conditions[1].kind {
        d.behavior = Behavior::EachNode;
    }
    assert_eq!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::UnknownRemotePoint {
            item: "NH_Output-1".into(),
            point: "Remote-1".into(),
        })
    );
    if let BoundaryKind::Displacement(d) = &mut model.steps[0].boundary_conditions[1].kind {
        d.behavior = Behavior::Deformable;
    }
    // A deactivated remote displacement has no point.
    model.steps[0].boundary_conditions[1].active = false;
    assert_eq!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::UnknownRemotePoint {
            item: "NH_Output-1".into(),
            point: "Remote-1".into(),
        })
    );
}

/// The moment and rotation of a point turned by 0.01 about z, from the `.dat` file, and the
/// moment of the reactions at the clamped end about the point's axis, which balances it.
fn point_moment(
    mesh: &FeMesh,
    mut model: FeModel,
    region: Region,
    name: &str,
) -> Option<(f64, f64, f64)> {
    let step = &mut model.steps[0];
    step.history_outputs
        .push(HistoryOutput::node("NH_Output-1", region));
    let mut clamp = HistoryOutput::node("NH_Output-2", Region::NodeSet("FIX".into()));
    clamp.variables = vec!["RF".into()];
    step.history_outputs.push(clamp);
    let dat = run_ccx_dat(name, &write_inp(mesh, &model, "").unwrap())?;
    let import = crate::dat::parse_dat(&dat);
    assert!(import.warnings.is_empty(), "{:?}", import.warnings);
    let set = |name: &str| import.sets.iter().find(|s| s.name == name).unwrap();
    let value = |field: &str, component: &str| {
        let component = set("NH_OUTPUT-1")
            .field(field)
            .unwrap()
            .component(component)
            .unwrap();
        // Shown under the reference node, as the forces.
        let node = &set("NH_OUTPUT-1").field("FORCES").unwrap().components[0].entries[0].name;
        assert_eq!(component.entries[0].name, *node);
        *component.entries[0].values.last().unwrap()
    };
    let (moment, rotation) = (value("MOMENTS", "RM3"), value("ROTATIONS", "UR3"));
    assert!(value("FORCES", "RF1").abs() < 1e-6 * moment.abs());
    let forces = set("FIX")
        .field("FORCES")
        .unwrap()
        .component("RF1")
        .unwrap();
    let clamp: f64 = (forces.entries.iter())
        .map(|e| {
            let y = mesh.node(e.name.parse().unwrap()).unwrap()[1];
            -(y - 5.0) * e.values.last().unwrap()
        })
        .sum();
    Some((moment, rotation, clamp))
}

/// A rotation prescribed at a reference point needs a moment, which its history output
/// prints with the rotation; the reactions at the clamped end balance it.
#[test]
fn calculix_prints_the_moment_and_rotation_of_a_reference_point() {
    let bc = BoundaryCondition {
        distribution: None,
        name: "Turn-1".into(),
        active: true,
        visible: true,
        region: Region::ReferencePoint("RP-1".into()),
        kind: BoundaryKind::displacement([None, None, None, None, None, Some(0.01)]),
        amplitude: None,
    };
    let (mesh, model) = rigid_tip(None, Some(bc));
    let region = Region::ReferencePoint("RP-1".into());
    let Some((moment, rotation, clamp)) = point_moment(&mesh, model, region, "rp-moment") else {
        return;
    };
    assert!((rotation - 0.01).abs() < 1e-9, "{rotation}");
    assert!(moment > 1.0, "{moment}");
    assert!((moment + clamp).abs() < 1e-3 * moment, "{moment} {clamp}");
}

#[test]
fn calculix_prints_the_moment_and_rotation_of_a_remote_point() {
    let values = [None, None, None, None, None, Some(0.01)];
    let (mesh, model) = remote_cantilever(remote("Remote-1", values));
    let region = Region::RemotePoint("Remote-1".into());
    let Some((moment, rotation, clamp)) = point_moment(&mesh, model, region, "remote-moment")
    else {
        return;
    };
    assert!((rotation - 0.01).abs() < 1e-9, "{rotation}");
    assert!(moment > 1.0, "{moment}");
    assert!((moment + clamp).abs() < 1e-3 * moment, "{moment} {clamp}");
}

/// CalculiX prints zero forces at the point of equations: a deformable remote point prints
/// its displacements and rotations only.
#[test]
fn calculix_prints_the_rotation_of_a_deformable_remote_point() {
    let values = [None, None, None, None, None, Some(0.01)];
    let (mesh, mut model) = remote_cantilever(remote_with(values, |r| {
        r.behavior = Behavior::Deformable;
    }));
    (model.steps[0].history_outputs).push(HistoryOutput::node(
        "NH_Output-1",
        Region::RemotePoint("Remote-1".into()),
    ));
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains(
            "*Node print, Nset=Internal_Selection-1_NH_Output-1\nU\n\
             *Node print, Nset=Internal_Rotation-1_NH_Output-1\nU\n"
        ),
        "{text}"
    );
    let Some(dat) = run_ccx_dat("remote-deformable-rotation", &text) else {
        return;
    };
    let import = crate::dat::parse_dat(&dat);
    let set = import
        .sets
        .iter()
        .find(|s| s.name == "NH_OUTPUT-1")
        .unwrap();
    assert!(set.field("FORCES").is_none() && set.field("MOMENTS").is_none());
    let ur3 = set.field("ROTATIONS").unwrap().component("UR3").unwrap();
    assert!((ur3.entries[0].values[0] - 0.01).abs() < 1e-9);
}

/// Springs and gaps share a face's area lumped: a quadratic quadrilateral spread like a
/// pressure gives its corners negative shares, a negative stiffness.
#[test]
fn stiffness_is_spread_with_positive_shares_on_quadratic_faces() {
    let mesh = read_inp(&testdata("block_c3d20r.inp")).unwrap().mesh;
    let element = mesh.elements()[0].id;
    let faces: Vec<(ElementId, u8)> = (1..=6).map(|f| (element, f)).collect();
    let (pressure, area) = constraints::node_areas(&mesh, &faces, false, false);
    assert!(pressure.values().any(|&w| w < 0.0));
    let (lumped, lumped_area) = constraints::node_areas(&mesh, &faces, false, true);
    assert_eq!(lumped.len(), 20);
    assert!(lumped.values().all(|&w| w > 0.0), "{lumped:?}");
    let sum: f64 = lumped.values().sum();
    assert!((sum - area).abs() < 1e-9 * area && lumped_area == area);
}

/// The stacked blocks joined by a contact region of the behavior between the top of the
/// lower block (side A) and the bottom of the upper one (side B).
fn blocks_joined_by_region(gap: f64, behavior: plx_model::ContactBehavior) -> (FeMesh, FeModel) {
    let (mesh, lower, upper) = two_blocks(gap);
    let mut model = stacked_blocks(&mesh);
    let mut region = plx_model::ContactRegion::new("Region-1", behavior);
    region.side_a = Region::Faces(lower);
    region.side_b = Region::Faces(upper);
    model.contact_regions.push(region);
    (mesh, model)
}

/// The nodes of the top of the upper block, pushed sideways by `shift` along x.
fn push_top(model: &mut FeModel, shift: f64) -> Vec<NodeId> {
    let top: Vec<NodeId> = (110..=118).collect();
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Shift-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(top.clone()),
        kind: BoundaryKind::displacement([Some(shift), None, None, None, None, None]),
        amplitude: None,
    });
    top
}

#[test]
fn a_bonded_region_is_a_tie_with_a_tolerance_of_the_mesh_size() {
    let (mesh, model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::Bonded);
    let text = write_inp(&mesh, &model, "").unwrap();
    // Faces of 5 x 5 mm: a quarter of it.
    assert!(
        text.contains(
            "*Tie, Name=Region-1, Position tolerance=1.25\n\
             Internal_Selection-1_Region-1_Slave, Internal_Selection-1_Region-1_Master\n"
        ),
        "{text}"
    );
    assert!(!text.contains("*Contact pair"));
    // Equal meshes and materials: the larger area is master, else side A.
    let resolved = model.resolve_contacts(&mesh);
    assert!(resolved.items[0].a_is_master);
}

/// A region a detection found keeps at least the distance the detection searched within as
/// its automatic tolerance: a detection on the geometry finds faces further apart than a
/// quarter of the mesh size, and CalculiX would leave their nodes untied ("too far away").
#[test]
fn calculix_ties_a_detected_gap_wider_than_a_quarter_of_the_mesh_size() {
    let (mesh, mut model) = blocks_joined_by_region(1.5, plx_model::ContactBehavior::Bonded);
    let mut detection = plx_model::ContactDetection::new("Detection-1");
    detection.tolerance = Some(2.0);
    model.contact_detections.push(detection);
    model.contact_regions[0].detected_by = Some("Detection-1".into());
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Tie, Name=Region-1, Position tolerance=2\n"),
        "{text}"
    );
    let resolved = model.resolve_contacts(&mesh);
    assert!(
        (resolved.items[0].why.iter())
            .any(|w| w.starts_with("Tolerance: the distance the detection")),
        "{:?}",
        resolved.items[0].why
    );
    // Not found by a detection: a quarter of the mesh size.
    model.contact_regions[0].detected_by = None;
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("Position tolerance=1.25\n"), "{text}");
    model.contact_regions[0].detected_by = Some("Detection-1".into());
    let Some(frd) = run_ccx(
        "region-detected-gap",
        &write_inp(&mesh, &model, "").unwrap(),
    ) else {
        return;
    };
    let bottom: Vec<NodeId> = (1..=9).collect();
    let reaction = node_sum(&frd, "FORC", "F3", &bottom);
    assert!((reaction - 1000.0).abs() < 1.0, "{reaction}");
}

#[test]
fn bonded_regions_sharing_slave_nodes_become_one_tie() {
    let (mesh, lower, upper) = two_blocks(0.0);
    let mut model = stacked_blocks(&mesh);
    for (i, half) in [[0, 1], [2, 3]].into_iter().enumerate() {
        let mut region = plx_model::ContactRegion::new(
            format!("Region-{}", i + 1),
            plx_model::ContactBehavior::Bonded,
        );
        region.side_a = Region::Faces(half.iter().map(|&k| lower[k]).collect());
        region.side_b = Region::Faces(half.iter().map(|&k| upper[k]).collect());
        region.settings.sides = Some(plx_model::SideRoles::AIsMaster);
        model.contact_regions.push(region);
    }
    let (lowered, origin) = model.lower_contacts(&mesh);
    assert_eq!(lowered.ties.len(), 1);
    assert_eq!(origin.tie_regions, [vec![0, 1]]);
    assert_eq!(lowered.ties[0].slave.faces(&mesh).len(), 4);
}

#[test]
fn calculix_carries_the_load_across_a_bonded_region() {
    // A small gap within the tolerance; the adjustment closes it.
    let (mesh, model) = blocks_joined_by_region(0.01, plx_model::ContactBehavior::Bonded);
    let Some(frd) = run_ccx("region-bonded", &write_inp(&mesh, &model, "").unwrap()) else {
        return;
    };
    let bottom: Vec<NodeId> = (1..=9).collect();
    let reaction = node_sum(&frd, "FORC", "F3", &bottom);
    assert!((reaction - 1000.0).abs() < 1.0, "{reaction}");
    let (below, above) = (
        node_value(&frd, "DISP", "U3", 14),
        node_value(&frd, "DISP", "U3", 105),
    );
    assert!(
        (below - above).abs() < 1e-3 * below.abs(),
        "{below} {above}"
    );
}

/// Penetration of the middle node of the upper block's bottom into the lower block, from
/// the results of the stacked blocks.
fn penetration(frd: &FrdImport) -> f64 {
    node_value(frd, "DISP", "U3", 14) - node_value(frd, "DISP", "U3", 105)
}

/// A frictionless region carries the pressure; the penetration is the pressure over the
/// stiffness 10 E / h, the same fraction of the elastic compression in millimetres and in
/// metres.
#[test]
fn calculix_carries_the_load_across_a_frictionless_region_in_any_units() {
    let mut fractions = Vec::new();
    for (name, scale) in [("mm", 1.0), ("m", 1e-3)] {
        let (mut mesh, mut model) =
            blocks_joined_by_region(0.0, plx_model::ContactBehavior::Frictionless);
        for (&id, c) in mesh.clone().node_ids().iter().zip(mesh.clone().coords()) {
            mesh.set_node(id, c.map(|v| v * scale));
        }
        // Stresses in Pa instead of MPa with lengths in m.
        let stress = 1.0 / (scale * scale);
        model.materials[0].elastic.as_mut().unwrap().young *= stress;
        model.steps[0].loads[0].kind = LoadKind::Pressure(10.0 * stress);
        // Held sideways at its top, the upper block rests on the lower one only by contact.
        model.steps[0].boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Side-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes((110..=118).collect()),
            kind: each_node([Some(0.0), Some(0.0), None, None, None, None]),
            amplitude: None,
        });
        let text = write_inp(&mesh, &model, "").unwrap();
        assert!(text.contains("*Surface behavior, Pressure-overclosure=Linear\n"));
        assert!(!text.contains("*Friction"));
        let Some(frd) = run_ccx(&format!("region-frictionless-{name}"), &text) else {
            return;
        };
        let bottom: Vec<NodeId> = (1..=9).collect();
        let reaction = node_sum(&frd, "FORC", "F3", &bottom);
        let force = 1000.0 * stress * scale * scale;
        assert!(
            (reaction - force).abs() < 1e-2 * force,
            "{name}: {reaction}"
        );
        let pressure = 10.0 * stress;
        let young = 210_000.0 * stress;
        let k = 10.0 * young / (5.0 * scale);
        let expected = pressure / k;
        // The middle node carries more than the mean pressure.
        let penetration = penetration(&frd);
        assert!(
            (penetration - expected).abs() < 0.5 * expected,
            "{name}: {penetration} {expected}"
        );
        // Elastic compression of the upper block, 10 thick.
        fractions.push(penetration / (pressure * 10.0 * scale / young));
    }
    if let [mm, m] = fractions[..] {
        assert!((mm - m).abs() < 1e-3 * mm, "{mm} {m}");
    }
}

/// Pushed sideways under the pressure, a frictional region carries mu times the normal force,
/// a rough one and a bonded one the whole shear.
#[test]
fn calculix_slides_frictional_regions_and_holds_rough_and_bonded_ones() {
    use plx_model::{BondedMethod, ContactBehavior};
    let cases = [
        ("frictional", ContactBehavior::Frictional { mu: 0.15 }, None),
        ("rough", ContactBehavior::Rough, None),
        ("tied", ContactBehavior::Bonded, Some(BondedMethod::Contact)),
        ("tie", ContactBehavior::Bonded, None),
    ];
    let mut shears = Vec::new();
    for (name, behavior, method) in cases {
        let (mesh, mut model) = blocks_joined_by_region(0.0, behavior);
        model.contact_regions[0].settings.bonded_method = method;
        // Pressure and push grow together, so does the friction limit.
        let top = push_top(&mut model, 0.004);
        let text = write_inp(&mesh, &model, "").unwrap();
        let Some(frd) = run_ccx(&format!("region-{name}"), &text) else {
            return;
        };
        let pushed = node_sum(&frd, "FORC", "F1", &top);
        shears.push((name, pushed));
    }
    let shear = |wanted: &str| shears.iter().find(|(n, _)| *n == wanted).unwrap().1;
    // 10 MPa on 10 x 10 mm.
    let frictional = shear("frictional");
    assert!((frictional - 150.0).abs() < 0.05 * 150.0, "{shears:?}");
    // Without sliding, the push of 4 um shears the blocks elastically, more than mu N.
    for name in ["rough", "tied", "tie"] {
        assert!(shear(name) > 1.5 * 150.0, "{shears:?}");
    }
    // The tied contact gives way little compared with the tie. The rough contact is softer:
    // the push bends the stack enough to lift an edge of the interface.
    assert!(
        (shear("tied") - shear("tie")).abs() < 0.01 * shear("tie"),
        "{shears:?}"
    );
}

#[test]
fn calculix_takes_mortar_contact_regions() {
    let (mesh, mut model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::Frictionless);
    model.contact_defaults.formulation = Some(ContactMethod::Mortar);
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Side-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes((110..=118).collect()),
        kind: each_node([Some(0.0), Some(0.0), None, None, None, None]),
        amplitude: None,
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    // Mortar contact is hard without a surface behavior.
    assert!(!text.contains("Pressure-overclosure=Hard"), "{text}");
    assert!(text.contains("Type=Mortar"));
    let Some(frd) = run_ccx("region-mortar", &text) else {
        return;
    };
    let bottom: Vec<NodeId> = (1..=9).collect();
    let reaction = node_sum(&frd, "FORC", "F3", &bottom);
    assert!((reaction - 1000.0).abs() < 10.0, "{reaction}");
}

#[test]
fn contact_regions_take_the_formulation_of_the_pairs_defined_by_hand() {
    use plx_model::{BondedMethod, ContactBehavior, MeshCheck, ModelItem, Problem, ResolvedKind};
    let (mesh, mut model) = blocks_joined_by_region(0.0, ContactBehavior::Bonded);
    model.contact_regions[0].settings.bonded_method = Some(BondedMethod::Contact);
    let resolved = model.resolve_contacts(&mesh);
    assert_eq!(resolved.formulation, ContactMethod::SurfaceToSurface);
    assert!(matches!(
        resolved.items[0].kind,
        ResolvedKind::Pair {
            behavior: SurfaceBehavior::Tied { .. },
            ..
        }
    ));
    // A pair defined by hand sets the formulation; a tied contact needs surface to surface,
    // so the region falls back to a tie and says why.
    let mut pair = ContactPair::new("Manual", "Hard");
    pair.method = ContactMethod::NodeToSurface;
    model.surface_interactions.push(SurfaceInteraction {
        name: "Hard".into(),
        properties: vec![InteractionProperty::SurfaceBehavior(SurfaceBehavior::Hard)],
    });
    model.contact_pairs.push(pair);
    let resolved = model.resolve_contacts(&mesh);
    assert_eq!(resolved.formulation, ContactMethod::NodeToSurface);
    assert!(matches!(resolved.items[0].kind, ResolvedKind::Tie { .. }));
    assert!(
        resolved.items[0]
            .why
            .iter()
            .any(|w| w.contains("needs Surface to surface")),
        "{:?}",
        resolved.items[0].why
    );
    // Another formulation chosen for the regions: the pair defined by hand differs.
    model.contact_regions[0].behavior = ContactBehavior::Frictionless;
    model.contact_defaults.formulation = Some(ContactMethod::SurfaceToSurface);
    let check = MeshCheck::new(&mesh, model.properties.space);
    let differ: Vec<ModelItem> = (model.check(&mesh, &check).into_iter())
        .filter(|f| f.problem == Problem::ContactFormulationsDiffer)
        .map(|f| f.item)
        .collect();
    assert_eq!(differ, [ModelItem::ContactPair(0)]);
}

#[test]
fn findings_on_what_a_region_becomes_belong_to_the_region() {
    use plx_model::{ContactBehavior, MeshCheck, ModelItem, Problem};
    let (mesh, mut model) = blocks_joined_by_region(0.0, ContactBehavior::Bonded);
    model.contact_regions[0].side_b = Region::Parts(vec!["GONE".into()]);
    let check = MeshCheck::new(&mesh, model.properties.space);
    let missing: Vec<ModelItem> = (model.check(&mesh, &check).into_iter())
        .filter(|f| f.problem == Problem::MissingReference)
        .map(|f| f.item)
        .collect();
    assert_eq!(missing, [ModelItem::ContactRegion(0)]);
    // Mortar contact in a frequency step.
    let (mesh, mut model) = blocks_joined_by_region(0.0, ContactBehavior::Frictionless);
    model.contact_defaults.formulation = Some(ContactMethod::Mortar);
    let mut frequency = Step::new_static("Step-2");
    frequency.kind = StepKind::Frequency(Default::default());
    model.steps.push(frequency);
    let mortar: Vec<ModelItem> = (model.check(&mesh, &check).into_iter())
        .filter(|f| f.problem == Problem::MortarNotSupported)
        .map(|f| f.item)
        .collect();
    assert_eq!(mortar, [ModelItem::Step(1)]);
}

/// The stacked blocks with a detection of all parts, and their skins.
fn detected_blocks(gap: f64) -> (FeMesh, FeModel, Vec<plx_mesh::PartSkin>) {
    let (mesh, _, _) = two_blocks(gap);
    let mut model = stacked_blocks(&mesh);
    model
        .contact_detections
        .push(plx_model::ContactDetection::new("Contacts"));
    let skins = (mesh.parts.iter())
        .map(|p| plx_mesh::extract_part_skin(&mesh, p, 30.0))
        .collect();
    (mesh, model, skins)
}

#[test]
fn a_detection_finds_the_touching_blocks_and_keeps_regions_the_user_changed() {
    let (mesh, mut model, skins) = detected_blocks(0.01);
    // 0.5 % of the diagonal (about 0.11) is less than half the element size of 5 to 10.
    let auto = plx_model::auto_detection_tolerance(&mesh, &skins, &[true, true]);
    assert!(auto > 2.0 && auto < 5.0, "{auto}");
    let report = model
        .detect_contacts(0, &mesh, &skins, &[true, true])
        .unwrap();
    assert_eq!((report.added, report.replaced), (1, 0));
    let region = &model.contact_regions[0];
    assert_eq!(region.name, "Bonded - LOWER to UPPER");
    assert_eq!(region.behavior, plx_model::ContactBehavior::Bonded);
    assert_eq!(region.detected_by.as_deref(), Some("Contacts"));
    assert_eq!(region.side_a.faces(&mesh).len(), 4);
    assert_eq!(region.side_b.faces(&mesh).len(), 4);
    assert_eq!(region.settings.tolerance, Some(auto));
    // Detecting again replaces what it found.
    let report = model
        .detect_contacts(0, &mesh, &skins, &[true, true])
        .unwrap();
    assert_eq!((report.added, report.replaced), (1, 1));
    assert_eq!(model.contact_regions.len(), 1);
    // A region changed by the user stays, and nothing is added for its parts.
    model.contact_regions[0].behavior = plx_model::ContactBehavior::Frictionless;
    model.contact_regions[0].edited = true;
    let report = model
        .detect_contacts(0, &mesh, &skins, &[true, true])
        .unwrap();
    assert_eq!((report.added, report.replaced), (0, 0));
    assert_eq!(report.kept, ["Bonded - LOWER to UPPER"]);
    assert_eq!(model.contact_regions.len(), 1);
    // Apart beyond the tolerance it is no longer found, but kept.
    let (far, _, _) = two_blocks(1.0);
    let far_skins: Vec<_> = (far.parts.iter())
        .map(|p| plx_mesh::extract_part_skin(&far, p, 30.0))
        .collect();
    model.contact_detections[0].tolerance = Some(0.5);
    let report = model
        .detect_contacts(0, &far, &far_skins, &[true, true])
        .unwrap();
    assert_eq!(report.added, 0);
    assert_eq!(report.lost, ["Bonded - LOWER to UPPER"]);
    // Parts left out of the detection are not searched.
    model.contact_regions.clear();
    model.contact_detections[0].tolerance = None;
    model.contact_detections[0].parts = vec!["LOWER".into()];
    let report = model
        .detect_contacts(0, &mesh, &skins, &[true, true])
        .unwrap();
    assert_eq!(report.added, 0);
}

#[test]
fn calculix_carries_the_load_across_detected_contacts() {
    let (mesh, mut model, skins) = detected_blocks(0.01);
    model.detect_contacts(0, &mesh, &skins, &[true, true]);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Tie, Name=Bonded_-_LOWER_to_UPPER"),
        "{text}"
    );
    let Some(frd) = run_ccx("detected-bonded", &text) else {
        return;
    };
    let bottom: Vec<NodeId> = (1..=9).collect();
    let reaction = node_sum(&frd, "FORC", "F3", &bottom);
    assert!((reaction - 1000.0).abs() < 1.0, "{reaction}");
}

#[test]
fn the_contact_check_tells_closed_and_open_regions_apart() {
    use plx_model::ContactStatus::*;
    // The tolerance of a bonded region of 5 x 5 faces is 1.25.
    for (gap, status) in [
        (0.0, Closed),
        (1.0, Closed),
        (-0.2, Closed),
        (3.0, NearOpen),
        (20.0, FarOpen),
    ] {
        let (mesh, model) = blocks_joined_by_region(gap, plx_model::ContactBehavior::Bonded);
        let check = model.check_contacts(&mesh)[0].clone().unwrap();
        assert_eq!(check.status, status, "{gap}");
        assert_eq!(check.slave_nodes, 9);
        assert!((check.tolerance - 1.25).abs() < 1e-9);
        match status {
            Closed if gap < 0.0 => {
                assert!(
                    (check.max_penetration.unwrap() - 0.2).abs() < 1e-9,
                    "{check:?}"
                );
            }
            Closed => assert!((check.max_gap.unwrap() - gap).abs() < 1e-9, "{check:?}"),
            NearOpen => assert!((check.min_gap.unwrap() - gap).abs() < 1e-9, "{check:?}"),
            _ => assert_eq!(check.touching, 0),
        }
        let open = (model.check(
            &mesh,
            &plx_model::MeshCheck::new(&mesh, model.properties.space),
        ))
        .into_iter()
        .any(|f| f.problem == plx_model::Problem::ContactRegionOpen);
        assert_eq!(open, matches!(status, NearOpen | FarOpen), "{gap}");
    }
    // Half the upper block beside the lower one: partly closed.
    let (mut mesh, model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::Frictionless);
    for (&id, c) in mesh.clone().node_ids().iter().zip(mesh.clone().coords()) {
        if id > 100 {
            mesh.set_node(id, [c[0] + 5.0, c[1], c[2]]);
        }
    }
    let check = model.check_contacts(&mesh)[0].clone().unwrap();
    assert_eq!(check.status, PartlyClosed, "{check:?}");
    assert_eq!(check.touching, 6);
}

/// Contact regions written as contact pairs bring contact results for the check after
/// solving; ties have none.
#[test]
fn contact_regions_write_contact_results() {
    let (mesh, model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::Frictionless);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Contact file\nCDIS, CSTR\n"), "{text}");
    let (mesh_tie, tie) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::Bonded);
    assert!(
        !write_inp(&mesh_tie, &tie, "")
            .unwrap()
            .contains("*Contact file")
    );
    let Some(frd) = run_ccx("region-contact-results", &text) else {
        return;
    };
    // 10 MPa on the upper block, carried by contact pressure at its bottom.
    let pressure = node_value(&frd, "CONTACT", "CPRESS", 105);
    assert!(pressure > 5.0 && pressure < 20.0, "{pressure}");
}

/// No separation holds the upper block to the lower one normal to the surfaces, one
/// equation per slave node, while it slides freely along them: pulled up and pushed
/// sideways at its top, the lower block takes the pull but no shear.
#[test]
fn calculix_holds_a_region_of_no_separation_together_and_lets_it_slide() {
    let (mesh, mut model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::NoSeparation);
    model.contact_regions[0].settings.sides = Some(plx_model::SideRoles::AIsMaster);
    model.steps[0].loads[0].kind = LoadKind::Pressure(-10.0);
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Side-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes((110..=118).collect()),
        kind: each_node([Some(0.01), Some(0.0), None, None, None, None]),
        amplitude: None,
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(!text.contains("*Contact pair") && !text.contains("*Tie"));
    // The nine nodes of the upper block's bottom, each on a node of the lower block's top.
    assert_eq!(text.matches("*Equation\n2\n").count(), 9, "{text}");
    assert!(
        text.contains("*Equation\n2\n105, 3, -1, 14, 3, 1\n"),
        "{text}"
    );
    let Some(frd) = run_ccx("region-no-separation", &text) else {
        return;
    };
    let bottom: Vec<NodeId> = (1..=9).collect();
    let pull = node_sum(&frd, "FORC", "F3", &bottom);
    assert!((pull + 1000.0).abs() < 1.0, "{pull}");
    let shear = node_sum(&frd, "FORC", "F1", &bottom);
    assert!(shear.abs() < 1.0, "{shear}");
    assert!(
        (node_value(&frd, "DISP", "U3", 14) - node_value(&frd, "DISP", "U3", 105)).abs() < 1e-9
    );
    let slide = node_value(&frd, "DISP", "U1", 105) - node_value(&frd, "DISP", "U1", 14);
    assert!((slide - 0.01).abs() < 1e-3, "{slide}");
}

/// A boundary condition on slave nodes of a region of no separation, the case CalculiX
/// aborted on ("dependent side of a MPC and a SPC"): the upper block's side x = 0 is held
/// in z, also at the slave nodes on it. Their equations depend on the master node below
/// instead, which the side holds then too, as the side itself is held.
#[test]
fn calculix_solves_no_separation_on_nodes_held_by_a_boundary_condition() {
    let (mesh, mut model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::NoSeparation);
    model.contact_regions[0].settings.sides = Some(plx_model::SideRoles::AIsMaster);
    let side: Vec<NodeId> = [101, 104, 107, 110, 113, 116].into();
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Side-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes(side),
        kind: each_node([None, None, Some(0.0), None, None, None]),
        amplitude: None,
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    // Nodes 101, 104 and 107 sit on the master nodes 10, 13 and 16.
    for (slave, master) in [(101, 10), (104, 13), (107, 16)] {
        assert!(
            text.contains(&format!("*Equation\n2\n{master}, 3, 1, {slave}, 3, -1\n")),
            "{text}"
        );
    }
    assert!(
        text.contains("*Equation\n2\n105, 3, -1, 14, 3, 1\n"),
        "{text}"
    );
    assert_eq!(
        crate::inp::resolved_overlaps(&text),
        ["Region-1: 3 equations on another degree of freedom (the first one is held already)"]
    );
    let Some(frd) = run_ccx("region-no-separation-held", &text) else {
        return;
    };
    assert!(node_value(&frd, "DISP", "U3", 10).abs() < 1e-12);
    let (below, above) = (
        node_value(&frd, "DISP", "U3", 14),
        node_value(&frd, "DISP", "U3", 105),
    );
    assert!(
        below < 0.0 && (below - above).abs() < 1e-9,
        "{below} {above}"
    );
}

/// A rigid body leaves out the nodes a boundary condition holds, as the point of a remote
/// displacement does: CalculiX refuses them in it.
#[test]
fn calculix_solves_a_rigid_body_on_nodes_held_by_a_boundary_condition() {
    let (mesh, mut model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::Bonded);
    model.steps[0].loads.clear();
    model
        .reference_points
        .push(ReferencePoint::new("RP-1", [5.0, 5.0, 20.0]));
    let mut body = RigidBody::new("Rigid_Body-1", "RP-1");
    body.region = Region::Nodes((110..=118).collect());
    model.constraints.push(Constraint::RigidBody(body));
    for (name, region, values) in [
        (
            "Corner-1",
            Region::Nodes(vec![110]),
            [None, None, Some(0.0)],
        ),
        (
            "Lift-1",
            Region::ReferencePoint("RP-1".into()),
            [Some(0.0), Some(0.0), Some(-0.001)],
        ),
    ] {
        let [x, y, z] = values;
        model.steps[0].boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: name.into(),
            active: true,
            visible: true,
            region,
            kind: each_node([x, y, z, Some(0.0), Some(0.0), Some(0.0)]),
            amplitude: None,
        });
    }
    let text = write_inp(&mesh, &model, "").unwrap();
    assert_eq!(
        crate::inp::resolved_overlaps(&text),
        ["Rigid_Body-1: 1 node left out (held already)"]
    );
    let Some(frd) = run_ccx("rigid-body-held", &text) else {
        return;
    };
    assert!(node_value(&frd, "DISP", "U3", 110).abs() < 1e-12);
    assert!((node_value(&frd, "DISP", "U3", 114) + 0.001).abs() < 1e-9);
}

/// An offset replaces the geometric gap: a negative one presses blocks that just touch
/// apart like an interference fit, the pressure from the overlap over the compliance of
/// both blocks and the contact.
#[test]
fn calculix_presses_regions_apart_by_a_negative_offset() {
    let (mesh, mut model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::Frictionless);
    model.contact_regions[0].settings.interface =
        Some(plx_model::InterfaceTreatment::Offset(-0.01));
    model.steps[0].loads.clear();
    model.steps[0].boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Top-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes((110..=118).collect()),
        kind: BoundaryKind::displacement([Some(0.0); 6]),
        amplitude: None,
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains(
            "*Clearance, Master=Internal_Selection-1_Region-1_Master, \
             Slave=Internal_Selection-1_Region-1_Slave, Value=-0.01\n"
        ),
        "{text}"
    );
    assert!(!text.contains("Adjust="));
    let Some(frd) = run_ccx("region-offset", &text) else {
        return;
    };
    // 0.01 = p (10/E + 10/E + h/(10 E)) without the lateral restraint, about 100 MPa.
    let bottom: Vec<NodeId> = (1..=9).collect();
    let force = node_sum(&frd, "FORC", "F3", &bottom);
    assert!(force > 5000.0 && force < 20_000.0, "{force}");
    assert!(node_value(&frd, "DISP", "U3", 14) < 0.0);
    assert!(node_value(&frd, "DISP", "U3", 105) > 0.0);
}

/// A tie or another formulation leaves the offset out and says why.
#[test]
fn an_offset_needs_a_surface_to_surface_contact() {
    let (mesh, mut model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::Bonded);
    model.contact_regions[0].settings.interface = Some(plx_model::InterfaceTreatment::Offset(0.1));
    let resolved = model.resolve_contacts(&mesh);
    assert!(
        resolved.items[0]
            .why
            .iter()
            .any(|w| w.contains("No offset"))
    );
    model.contact_regions[0].behavior = plx_model::ContactBehavior::Frictionless;
    model.contact_defaults.formulation = Some(plx_model::ContactMethod::NodeToSurface);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(!text.contains("*Clearance"));
}

/// In a coupled step heat flows across a contact region: its contact pair conducts 100 k / h,
/// so that the temperature hardly jumps at the contact. Hot at the bottom and cold at the
/// top, the blocks meet at about the mean.
#[test]
fn calculix_conducts_heat_across_contact_regions_in_coupled_steps() {
    let (mesh, mut model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::Frictionless);
    let material = &mut model.materials[0];
    material.conductivity = Some(CONDUCTIVITY);
    material.specific_heat = Some(SPECIFIC_HEAT);
    let step = &mut model.steps[0];
    step.kind = StepKind::CoupledTempDisp(HeatTransferStep::default());
    step.field_outputs = FieldOutput::coupled_defaults();
    step.boundary_conditions.extend([
        temperature("Hot", Region::Nodes((1..=9).collect()), 100.0),
        temperature("Cold", Region::Nodes((110..=118).collect()), 0.0),
    ]);
    step.boundary_conditions.push(BoundaryCondition {
        distribution: None,
        name: "Side-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes((110..=118).collect()),
        kind: each_node([Some(0.0), Some(0.0), None, None, None, None]),
        amplitude: None,
    });
    model.initial_conditions.push(plx_model::InitialCondition {
        distribution: None,
        name: "Initial-1".into(),
        active: true,
        region: Region::Parts(vec!["LOWER".into(), "UPPER".into()]),
        kind: plx_model::InitialConditionKind::Temperature(50.0),
    });
    let resolved = model.resolve_contacts(&mesh);
    let plx_model::ResolvedKind::Pair { conductance, .. } = resolved.items[0].kind else {
        panic!("{:?}", resolved.items[0].kind);
    };
    assert_eq!(conductance, Some(100.0 * CONDUCTIVITY / 5.0));
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Gap conductance\n1000\n"), "{text}");
    let Some(frd) = run_ccx("region-conductance", &text) else {
        return;
    };
    let (below, above) = (
        node_value(&frd, "NDTEMP", "T", 14),
        node_value(&frd, "NDTEMP", "T", 105),
    );
    assert!((below - 50.0).abs() < 5.0, "{below}");
    assert!((below - above).abs() < 2.0, "{below} {above}");
}

/// The equations of no separation hold for small displacements only; with large
/// deformations the model check warns.
#[test]
fn no_separation_with_large_deformations_is_a_warning() {
    use plx_model::{MeshCheck, ModelItem, Problem};
    let (mesh, mut model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::NoSeparation);
    let check = MeshCheck::new(&mesh, model.properties.space);
    let warned = |model: &FeModel| {
        (model.check(&mesh, &check).iter()).any(|f| {
            f.problem == Problem::NoSeparationLargeDeformation
                && f.item == ModelItem::ContactRegion(0)
        })
    };
    assert!(!warned(&model));
    if let StepKind::Static(s) = &mut model.steps[0].kind {
        s.nlgeom = true;
    }
    assert!(warned(&model));
    let resolved = model.resolve_contacts(&mesh);
    assert!(
        resolved.items[0]
            .why
            .iter()
            .any(|w| w.contains("small displacements"))
    );
}

/// A step after `model`'s first, its loads left out and its boundary conditions those of the
/// first and `extra`.
fn next_step(model: &FeModel, name: &str, extra: Vec<BoundaryCondition>) -> Step {
    let mut step = model.steps[0].clone();
    step.name = name.into();
    step.loads.clear();
    step.boundary_conditions.extend(extra);
    step
}

/// A step switches a contact region off: pressed on before, the upper block held at its top
/// no longer loads the lower one. The next step switches it on again and the pressure gets
/// through.
#[test]
fn calculix_switches_contact_regions_off_and_on_in_steps() {
    use plx_model::{ContactBehavior, ContactState};
    let (mesh, mut model) = blocks_joined_by_region(0.0, ContactBehavior::Frictionless);
    let side = |values| BoundaryCondition {
        distribution: None,
        name: "Side-1".into(),
        active: true,
        visible: true,
        region: Region::Nodes((110..=118).collect()),
        kind: each_node(values),
        amplitude: None,
    };
    model.steps[0]
        .boundary_conditions
        .push(side([Some(0.0), Some(0.0), None, None, None, None]));
    let on = ContactState {
        active: true,
        friction: None,
    };
    let off = ContactState {
        active: false,
        ..on
    };
    let mut free = next_step(&model, "Step-2", Vec::new());
    free.boundary_conditions[1] = side([Some(0.0), Some(0.0), Some(0.0), None, None, None]);
    free.set_contact_state("Region-1", on, off);
    assert_eq!(free.contact_changes.len(), 1);
    let mut pressed = model.steps[0].clone();
    pressed.name = "Step-3".into();
    pressed.set_contact_state("Region-1", off, on);
    model.steps.push(free);
    let region = &model.contact_regions[0];
    assert_eq!(region.state_after(&model.steps), off);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains(
            "** Name: Region-1\n*Model change, Type=Contact pair, Remove\n\
             Internal_Selection-1_Region-1_Slave, Internal_Selection-1_Region-1_Master\n"
        ),
        "{text}"
    );
    let bottom: Vec<NodeId> = (1..=9).collect();
    let Some(frd) = run_ccx("region-switched-off", &text) else {
        return;
    };
    let reaction = node_sum(&frd, "FORC", "F3", &bottom);
    assert!(reaction.abs() < 1.0, "{reaction}");
    model.steps.push(pressed);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(text.contains("*Model change, Type=Contact pair, Add\n"));
    let Some(frd) = run_ccx("region-switched-on", &text) else {
        return;
    };
    let reaction = node_sum(&frd, "FORC", "F3", &bottom);
    assert!((reaction - 1000.0).abs() < 10.0, "{reaction}");
}

/// A step raises the friction of a sliding region: pushed further, the block slides at the
/// new coefficient times the normal force. The region gets an interaction of its own, as
/// the change applies to the whole interaction.
#[test]
fn calculix_changes_the_friction_of_a_region_in_a_step() {
    use plx_model::{ContactBehavior, ContactState};
    let (mesh, mut model) = blocks_joined_by_region(0.0, ContactBehavior::Frictional { mu: 0.15 });
    let mut other = model.contact_regions[0].clone();
    other.name = "Region-2".into();
    other.active = false;
    model.contact_regions.push(other);
    let top = push_top(&mut model, 0.004);
    let mut step = model.steps[0].clone();
    step.name = "Step-2".into();
    if let BoundaryKind::Displacement(d) = &mut step.boundary_conditions[1].kind {
        d.values[0] = Some(0.008);
    }
    let before = model.contact_regions[0].state_after(&model.steps);
    step.set_contact_state(
        "Region-1",
        before,
        ContactState {
            friction: Some(0.3),
            ..before
        },
    );
    model.steps.push(step);
    let (lowered, _) = model.lower_contacts(&mesh);
    let interactions: Vec<&str> = (lowered.contact_pairs.iter())
        .map(|p| p.interaction.as_str())
        .collect();
    assert_eq!(interactions, ["PLX_Frictional", "PLX_Frictional-2"]);
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(
        text.contains("*Change friction, Interaction=PLX_Frictional\n*Friction\n0.3, 4200\n"),
        "{text}"
    );
    let Some(frd) = run_ccx("region-friction-change", &text) else {
        return;
    };
    let pushed = node_sum(&frd, "FORC", "F1", &top);
    assert!((pushed - 300.0).abs() < 0.05 * 300.0, "{pushed}");
}

/// A tie cannot be switched in a step: a bonded region switched becomes a tied contact,
/// unless chosen as a tie, which the model check reports, as it reports no separation.
#[test]
fn switched_bonded_regions_become_tied_contacts() {
    use plx_model::{
        BondedMethod, ContactBehavior, ContactState, MeshCheck, ModelItem, Problem, ResolvedKind,
    };
    let (mesh, mut model) = blocks_joined_by_region(0.0, ContactBehavior::Bonded);
    let mut step = next_step(&model, "Step-2", Vec::new());
    let on = ContactState {
        active: true,
        friction: None,
    };
    step.set_contact_state(
        "Region-1",
        on,
        ContactState {
            active: false,
            ..on
        },
    );
    model.steps.push(step);
    let resolved = model.resolve_contacts(&mesh);
    assert!(matches!(
        resolved.items[0].kind,
        ResolvedKind::Pair {
            behavior: SurfaceBehavior::Tied { .. },
            ..
        }
    ));
    let check = MeshCheck::new(&mesh, model.properties.space);
    let ignored = |model: &FeModel| {
        (model.check(&mesh, &check).iter()).any(|f| {
            f.problem == Problem::ContactChangeIgnored && f.item == ModelItem::ContactRegion(0)
        })
    };
    assert!(!ignored(&model));
    model.contact_regions[0].settings.bonded_method = Some(BondedMethod::Constraint);
    assert!(ignored(&model));
    assert!(
        !write_inp(&mesh, &model, "")
            .unwrap()
            .contains("*Model change")
    );
    model.contact_regions[0].behavior = ContactBehavior::NoSeparation;
    assert!(ignored(&model));
}

/// CalculiX reads names of at most 80 characters: the sets of a region with a long name are
/// shortened, and the `.dat` file read beside the input file shows the full name again.
#[test]
fn long_set_names_are_shortened_and_restored_in_the_history_output() {
    let (mesh, mut model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::Frictionless);
    let long = "Bonded - SPUR_GEAR__20_TEETH, SPUR_GEAR__60_TEETH-2 to SPUR_GEAR__20_TEETH-2";
    model.contact_regions[0].name = long.to_string();
    let mut output = HistoryOutput::contact("CH_Output-1", long);
    output.variables = ["CF".to_string()].to_vec();
    model.steps[0].history_outputs.push(output);
    let text = write_inp(&mesh, &model, "").unwrap();
    for line in text.lines().filter(|l| !l.starts_with("**")) {
        for part in line.split(',') {
            let value = part.split_once('=').map_or(part, |(_, v)| v).trim();
            assert!(value.len() <= 80, "{line}");
        }
    }
    assert!(text.contains(super::SHORTENED_NAME), "{text}");
    for behavior in [
        plx_model::ContactBehavior::Bonded,
        plx_model::ContactBehavior::Frictional { mu: 0.2 },
    ] {
        let mut model = model.clone();
        model.contact_regions[0].behavior = behavior;
        model.steps[0].history_outputs.clear();
        let text = write_inp(&mesh, &model, "").unwrap();
        for line in text.lines().filter(|l| !l.starts_with("**")) {
            for part in line.split(',') {
                let value = part.split_once('=').map_or(part, |(_, v)| v).trim();
                assert!(value.len() <= 80, "{behavior:?}: {line}");
            }
        }
    }
    let Some(dat) = run_ccx_dat("lange-namen", &text) else {
        return;
    };
    let import = crate::dat::parse_dat(&crate::dat::restore_shortened_names(&dat, &text));
    let names: Vec<&String> = import.sets.iter().map(|s| &s.name).collect();
    let expected = "BONDED_-_SPUR_GEAR__20_TEETH__SPUR_GEAR__60_TEETH-2_TO_SPUR_GEAR__20_TEETH-2";
    assert!(names.iter().any(|n| *n == expected), "{names:?}");
}

/// A name made from the definition follows the type of the region; a name of the user
/// stays.
#[test]
fn a_name_from_the_definition_follows_the_type() {
    let (mesh, mut model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::Bonded);
    let region = &mut model.contact_regions[0];
    let custom = region.name.clone();
    region.set_behavior(plx_model::ContactBehavior::Frictionless);
    assert_eq!(region.name, custom);
    region.name = region.name_from_definition(&mesh);
    assert!(
        region.name.starts_with("Frictionless - "),
        "{}",
        region.name
    );
    let rest = region.name["Frictionless".len()..].to_string();
    region.set_behavior(plx_model::ContactBehavior::Frictional { mu: 0.2 });
    assert_eq!(region.name, format!("Frictional{rest}"));
    // Another coefficient is the same type.
    region.set_behavior(plx_model::ContactBehavior::Frictional { mu: 0.3 });
    assert_eq!(region.name, format!("Frictional{rest}"));
}

fn equation(name: &str, text: &str) -> plx_model::Distribution {
    plx_model::Distribution {
        name: name.into(),
        kind: plx_model::DistributionKind::Equation(text.into()),
    }
}

/// A pressure scaled by a distribution takes its value at the center of every face: y / 10
/// over the 10 x 10 tip face is 0.5 on average, so 2 MPa push with 100 N.
#[test]
fn calculix_pressure_follows_its_distribution() {
    let load = Load {
        distribution: Some("Rising".into()),
        name: "Pressure-1".into(),
        active: true,
        visible: true,
        region: Region::Surface("TIP".into()),
        kind: LoadKind::Pressure(2.0),
        amplitude: None,
        factor_amplitude: None,
    };
    let (mesh, mut model) = cantilever(load);
    model.properties.units = UnitSystem::MKgSC;
    model.distributions.push(equation("Rising", "=y / 10"));
    let text = write_inp(&mesh, &model, "").unwrap();
    // One line per face, no surface sets.
    assert!(!text.contains("Internal_Selection-1_Pressure-1"), "{text}");
    let faces = Region::Surface("TIP".into()).faces(&mesh);
    let (element, face) = faces[0];
    let center = face_center(&mesh, element, face).unwrap();
    let line = format!("\n{element}, P{face}, {}\n", number(2.0 * center[1] / 10.0));
    assert!(text.contains(&line), "{line}\n{text}");
    let Some(frd) = run_ccx("druck-verteilt", &text) else {
        return;
    };
    let reaction: f64 = mesh.node_sets["FIX"]
        .iter()
        .map(|&node| node_value(&frd, "FORC", "F1", node))
        .sum();
    assert!((reaction.abs() - 100.0).abs() < 1e-3, "{reaction}");
}

#[test]
fn distributed_temperatures_are_written_per_node() {
    let (mesh, mut model) = thermal_bar(
        steady(),
        vec![temperature(
            "Temperature-1",
            Region::NodeSet("FIX".into()),
            50.0,
        )],
        Vec::new(),
    );
    model.properties.units = UnitSystem::MKgSC;
    model.distributions.push(equation("Half", "=0.5"));
    model.steps[0].boundary_conditions[0].distribution = Some("Half".into());
    model.initial_conditions.push(plx_model::InitialCondition {
        distribution: Some("Half".into()),
        name: "Initial_Temperature-1".into(),
        active: true,
        region: Region::Nodes(vec![1, 2]),
        kind: plx_model::InitialConditionKind::Temperature(20.0),
    });
    let text = write_inp(&mesh, &model, "").unwrap();
    for expected in [
        "*Initial conditions, Type=Temperature\n1, 10\n2, 10\n",
        "** Name: Temperature-1\n*Boundary\n1, 11, 11, 25\n",
    ] {
        assert!(text.contains(expected), "{expected}\n{text}");
    }
    assert!(
        !text.contains("Internal_Selection-1_Temperature-1"),
        "{text}"
    );
    model.distributions[0].kind = plx_model::DistributionKind::Equation("=x +".into());
    assert!(matches!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::InvalidDistribution { .. })
    ));
    model.distributions.clear();
    assert!(matches!(
        write_inp(&mesh, &model, ""),
        Err(WriteError::UnknownDistribution { .. })
    ));
}

/// A defined temperature of 20 + x over the bar, given as 120 times a distribution, makes it
/// grow by alpha times the integral of T - 20, as the same field read from a heat transfer
/// run does.
#[test]
fn calculix_strains_the_bar_by_a_distributed_temperature() {
    let (mesh, mut model) = thermal_bar(
        StepKind::Static(StaticStep::default()),
        Vec::new(),
        Vec::new(),
    );
    model.properties.units = UnitSystem::MKgSC;
    model
        .distributions
        .push(equation("Gradient", "=(20 + x) / 120"));
    model.initial_conditions.push(plx_model::InitialCondition {
        distribution: None,
        name: "Initial_Temperature-1".into(),
        active: true,
        region: Region::Parts(vec!["EALL".into()]),
        kind: plx_model::InitialConditionKind::Temperature(20.0),
    });
    let mut field = defined_temperature(
        "Defined_Temperature-1",
        Region::Parts(vec!["EALL".into()]),
        120.0,
    );
    field.distribution = Some("Gradient".into());
    model.steps[0].defined_fields = vec![field];
    // Held at x = 0 in x, and against rigid motion at the corner and an edge node.
    let hold = |name: &str, nodes: Vec<NodeId>, held: [bool; 3]| BoundaryCondition {
        distribution: None,
        name: name.into(),
        active: true,
        visible: true,
        region: Region::Nodes(nodes),
        kind: BoundaryKind::displacement([
            held[0].then_some(0.0),
            held[1].then_some(0.0),
            held[2].then_some(0.0),
            None,
            None,
            None,
        ]),
        amplitude: None,
    };
    // Node 1 is the corner at the origin, node 12 lies along y from it.
    model.steps[0].boundary_conditions = vec![
        hold("Axial", nodes_at(&mesh, 0, 0.0), [true, false, false]),
        hold("Corner", vec![1], [false, true, true]),
        hold("Edge", vec![12], [false, false, true]),
    ];
    let Some(frd) = run_ccx(
        "verteilte-temperatur",
        &write_inp(&mesh, &model, "").unwrap(),
    ) else {
        return;
    };
    assert_all_close(&temperatures_at(&mesh, &frd, 100.0), 120.0, 1e-6);
    let growth: Vec<f64> = (nodes_at(&mesh, 0, 100.0).iter())
        .map(|&n| node_value(&frd, "DISP", "U1", n))
        .collect();
    assert_all_close(&growth, 1e-5 * 100.0 * 100.0 / 2.0, 1e-3);
}

/// The upper block rests on the lower one by a frictionless contact only: nothing holds it
/// sideways, and with an initial gap nothing holds it at all before the gap closes. CalculiX
/// gives up on both with "too many cutbacks"; weak springs to ground hold the block and take
/// about a millionth of the load.
#[test]
fn calculix_solves_a_part_held_only_by_contact_with_weak_springs() {
    for (name, gap, nlgeom) in [("spalt", 0.01, false), ("spalt-nlgeom", 0.01, true)] {
        let (mesh, mut model) =
            blocks_joined_by_region(gap, plx_model::ContactBehavior::Frictionless);
        if let plx_model::StepKind::Static(settings) = &mut model.steps[0].kind {
            settings.nlgeom = nlgeom;
        }
        let text = write_inp(&mesh, &model, "").unwrap();
        // The upper block: 1e-6 E L with L = 10 mm, spread over its 18 nodes.
        assert!(
            text.contains("*Element, Type=SPRING1, Elset=PLX_Weak_Springs_UPPER_All\n"),
            "{text}"
        );
        assert!(
            text.contains("*Spring, Elset=PLX_Weak_Springs_UPPER_DOF_3\n3\n1.16666667E-1\n"),
            "{text}"
        );
        model.contact_defaults.weak_springs = false;
        let without = write_inp(&mesh, &model, "").unwrap();
        assert!(!without.contains("SPRING1"), "{without}");
        model.contact_defaults.weak_springs = true;
        let Some(frd) = run_ccx(&format!("weak-springs-{name}"), &text) else {
            return;
        };
        let bottom: Vec<NodeId> = (1..=9).collect();
        let reaction = node_sum(&frd, "FORC", "F3", &bottom);
        assert!((reaction - 1000.0).abs() < 0.1, "{name}: {reaction}");
    }
}

/// A bonded region whose contact values a history output prints becomes a tied contact,
/// since a tie has none: CalculiX prints the 1000 N the pressure pushes through it.
#[test]
fn calculix_prints_the_contact_forces_of_a_bonded_region() {
    let (mesh, mut model) = blocks_joined_by_region(0.0, plx_model::ContactBehavior::Bonded);
    let mut output = HistoryOutput::contact("CH_Output-1", "Region-1");
    output.variables = ["CDIS", "CSTR", "CF"].map(String::from).to_vec();
    model.steps[0].history_outputs.push(output);
    let resolved = model.resolve_contacts(&mesh);
    assert!(
        matches!(resolved.items[0].kind, plx_model::ResolvedKind::Pair { .. }),
        "{:?}",
        resolved.items[0]
    );
    assert!(
        (model
            .check(
                &mesh,
                &plx_model::MeshCheck::new(&mesh, model.properties.space)
            )
            .iter())
        .all(|f| f.problem != plx_model::Problem::ContactOutputIgnored)
    );
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(!text.contains("*Tie"), "{text}");
    assert!(text.contains("*Contact print, Master="), "{text}");
    let Some(dat) = run_ccx_dat("region-kontaktkraft", &text) else {
        return;
    };
    let import = crate::dat::parse_dat(&dat);
    let region = (import.sets.iter())
        .find(|s| s.name == "REGION-1")
        .unwrap_or_else(|| {
            panic!(
                "{:?}",
                import.sets.iter().map(|s| &s.name).collect::<Vec<_>>()
            )
        });
    let force = (region.field("TOTAL_SURFACE_FORCE").unwrap())
        .component("FZ")
        .unwrap();
    let fz = *force.entries[0].values.last().unwrap();
    // 10 MPa on 10 x 10 mm.
    assert!((fz.abs() - 1000.0).abs() < 10.0, "{fz}");
}

/// Of a region written as a tie or as equations CalculiX has no contact values: the
/// history output is left out with a warning instead of failing the input file.
#[test]
fn contact_outputs_of_regions_without_contact_elements_are_left_out() {
    for (behavior, method) in [
        (
            plx_model::ContactBehavior::Bonded,
            Some(plx_model::BondedMethod::Constraint),
        ),
        (plx_model::ContactBehavior::NoSeparation, None),
    ] {
        let (mesh, mut model) = blocks_joined_by_region(0.0, behavior);
        model.contact_regions[0].settings.bonded_method = method;
        let mut output = HistoryOutput::contact("CH_Output-1", "Region-1");
        output.variables = ["CF".to_string()].to_vec();
        model.steps[0].history_outputs.push(output);
        let text = write_inp(&mesh, &model, "").unwrap();
        assert!(!text.contains("*Contact print"), "{behavior:?}");
        assert!(
            (model
                .check(
                    &mesh,
                    &plx_model::MeshCheck::new(&mesh, model.properties.space)
                )
                .iter())
            .any(|f| f.problem == plx_model::Problem::ContactOutputIgnored
                && f.item == plx_model::ModelItem::ContactRegion(0)),
            "{behavior:?}"
        );
    }
}

/// A bolt shank of radius 5 and length 40, its axis tilted out of z: wedges around the axis
/// and a ring of hexahedra, 12 around and 8 along, as part EALL. Both ends are held (node
/// set FIX at the start, TIP at the end); the bolt is picked by its cylindrical faces.
fn shank_bolt(preload: f64) -> (FeMesh, FeModel) {
    let (around, along, radius, length) = (12, 8, 5.0, 40.0);
    let (tilt, turn) = (0.4_f64, 0.7_f64);
    // The axis z turned about x by `tilt`, then about z by `turn`.
    let rotate = |[x, y, z]: [f64; 3]| -> [f64; 3] {
        let (y, z) = (
            y * tilt.cos() - z * tilt.sin(),
            y * tilt.sin() + z * tilt.cos(),
        );
        let (x, y) = (
            x * turn.cos() - y * turn.sin(),
            x * turn.sin() + y * turn.cos(),
        );
        [x, y, z]
    };
    let per_layer = 1 + 2 * around;
    let node = |layer: usize, ring: usize, i: usize| -> usize {
        1 + layer * per_layer
            + if ring == 0 {
                0
            } else {
                1 + (ring - 1) * around + i % around
            }
    };
    let mut text = String::from("*Node\n");
    for layer in 0..=along {
        let z = length * layer as f64 / along as f64;
        let mut points = vec![(node(layer, 0, 0), [0.0, 0.0, z])];
        for ring in 1..=2 {
            for i in 0..around {
                let angle = std::f64::consts::TAU * i as f64 / around as f64;
                let r = radius * ring as f64 / 2.0;
                points.push((node(layer, ring, i), [r * angle.cos(), r * angle.sin(), z]));
            }
        }
        for (id, p) in points {
            let [x, y, z] = rotate(p);
            let _ = writeln!(text, "{id}, {x}, {y}, {z}");
        }
    }
    let mut wedges = String::from("*Element, Type=C3D6, Elset=EALL\n");
    let mut hexes = String::from("*Element, Type=C3D8, Elset=EALL\n");
    let mut shank = Vec::new();
    let mut id = 1;
    for layer in 0..along {
        for i in 0..around {
            let (b, t) = (layer, layer + 1);
            let _ = writeln!(
                wedges,
                "{id}, {}, {}, {}, {}, {}, {}",
                node(b, 0, 0),
                node(b, 1, i),
                node(b, 1, i + 1),
                node(t, 0, 0),
                node(t, 1, i),
                node(t, 1, i + 1)
            );
            id += 1;
            let _ = writeln!(
                hexes,
                "{id}, {}, {}, {}, {}, {}, {}, {}, {}",
                node(b, 1, i),
                node(b, 2, i),
                node(b, 2, i + 1),
                node(b, 1, i + 1),
                node(t, 1, i),
                node(t, 2, i),
                node(t, 2, i + 1),
                node(t, 1, i + 1)
            );
            // Face 4 (nodes 2, 6, 7, 3) lies on the cylinder.
            shank.push((id as ElementId, 4));
            id += 1;
        }
    }
    let ends = |layer: usize| -> String {
        let ids: Vec<String> = (0..per_layer)
            .map(|k| (1 + layer * per_layer + k).to_string())
            .collect();
        ids.join(",\n")
    };
    let _ = write!(
        text,
        "{wedges}{hexes}*Nset, Nset=FIX\n{}\n*Nset, Nset=TIP\n{}\n",
        ends(0),
        ends(along)
    );
    let mesh = crate::inp::read_inp_str(&text, None).unwrap().mesh;
    let (_, mut model) = cantilever(tip_force());
    let step = &mut model.steps[0];
    step.loads.clear();
    let mut tip = step.boundary_conditions[0].clone();
    tip.name = "Fixed-2".into();
    tip.region = Region::NodeSet("TIP".into());
    step.boundary_conditions.push(tip);
    let mut second = model.steps[0].clone();
    second.name = "Step-2".into();
    model.steps.push(second);
    let mut bolt = plx_model::Bolt::new("Bolt-1");
    bolt.side_a = Region::Faces(shank);
    bolt.preload = plx_model::Preload::Force(preload);
    model.bolts.push(bolt);
    (mesh, model)
}

/// Picked by the cylindrical faces of its shank, a bolt is cut across the middle along the
/// element faces and preloaded along its axis.
#[test]
fn a_bolt_picked_by_its_shank_is_cut_across_its_middle() {
    let force = 1000.0;
    let (mesh, model) = shank_bolt(force);
    let resolved = model.resolve_bolt(&model.bolts[0], &mesh).unwrap();
    // 12 wedges and 12 hexahedra below the middle.
    assert_eq!(resolved.cut_faces.len(), 24, "{resolved:?}");
    assert_eq!(resolved.tied_to, None);
    let axis = resolved.shank_axis.unwrap();
    let (tilt, turn) = (0.4_f64, 0.7_f64);
    let expected = [
        tilt.sin() * turn.sin(),
        -tilt.sin() * turn.cos(),
        tilt.cos(),
    ];
    let along: f64 = axis.iter().zip(expected).map(|(a, b)| a * b).sum();
    assert!((along.abs() - 1.0).abs() < 1e-9, "{axis:?}");
    // The cut faces lie in the middle: all their nodes half way along the axis.
    for &(e, f) in &resolved.cut_faces {
        for n in constraints::face_node_ids(&mesh, e, f) {
            let p = mesh.node(n).unwrap();
            let t: f64 = p.iter().zip(expected).map(|(a, b)| a * b).sum();
            assert!((t - 20.0).abs() < 1e-9, "{t}");
        }
    }
    let text = write_inp(&mesh, &model, "").unwrap();
    assert!(!text.contains("*Tie"), "{text}");
    let Some((frd, dat)) = run_ccx_frd_dat("schraube-schaft", &text) else {
        return;
    };
    let reaction: [f64; 3] = std::array::from_fn(|i| {
        (mesh.node_sets["FIX"].iter())
            .map(|&node| node_value(&frd, "FORC", ["F1", "F2", "F3"][i], node))
            .sum()
    });
    // The bolt is in tension: the support at its start pulls it away from the middle.
    let axial: f64 = reaction.iter().zip(expected).map(|(a, b)| a * b).sum();
    assert!((axial + force).abs() < 1e-3 * force, "{reaction:?}");
    let bolt = dat.sets.iter().find(|s| s.name == "BOLT-1").unwrap();
    let forces = &bolt
        .field("FORCES")
        .unwrap()
        .component("RF1")
        .unwrap()
        .entries[0];
    for value in &forces.values {
        assert!((value - force).abs() < 1e-3 * force, "{:?}", forces.values);
    }
}

/// A shank bonded into its hole cannot stretch: the model check warns at the bolt.
#[test]
fn the_model_check_warns_of_a_shank_bonded_into_its_hole() {
    use plx_model::{MeshCheck, ModelItem, Problem};
    let (mesh, mut model) = shank_bolt(1000.0);
    let problems = |model: &FeModel| -> Vec<(ModelItem, Problem)> {
        (model
            .check(&mesh, &MeshCheck::new(&mesh, model.properties.space))
            .into_iter())
        .map(|f| (f.item, f.problem))
        .collect()
    };
    assert_eq!(problems(&model), []);
    let mut region = plx_model::ContactRegion::new("Bore", plx_model::ContactBehavior::Bonded);
    region.side_a = model.bolts[0].side_a.clone();
    region.side_b = Region::NodeSet("TIP".into());
    model.contact_regions.push(region);
    assert_eq!(
        problems(&model),
        [(ModelItem::Bolt(0), Problem::BoltHeldAtCut)]
    );
    // Sliding in its hole, the shank stretches.
    model.contact_regions[0].behavior = plx_model::ContactBehavior::Frictionless;
    assert_eq!(problems(&model), []);
}
