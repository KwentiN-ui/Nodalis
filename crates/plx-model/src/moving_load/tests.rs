use plx_mesh::{Element, ElementShape};

use super::*;

/// A flat plate in the x-y plane from x = 0 to 1000 and y = -100 to 100 of 10 x 2
/// shell elements, linear or quadratic, and its faces.
fn plate(quadratic: bool) -> (FeMesh, Vec<(ElementId, u8)>) {
    let mut mesh = FeMesh::default();
    // Nodes on a grid twice as fine, so that quadratic elements find their midside nodes.
    let id = |i: usize, j: usize| (i * 5 + j + 1) as NodeId;
    for i in 0..=20 {
        for j in 0..=4 {
            mesh.set_node(id(i, j), [i as f64 * 50.0, -100.0 + j as f64 * 50.0, 0.0]);
        }
    }
    let mut faces = Vec::new();
    for e in 0..10 {
        for f in 0..2 {
            let (i, j) = (2 * e, 2 * f);
            let mut nodes = vec![id(i, j), id(i + 2, j), id(i + 2, j + 2), id(i, j + 2)];
            if quadratic {
                nodes.extend([
                    id(i + 1, j),
                    id(i + 2, j + 1),
                    id(i + 1, j + 2),
                    id(i, j + 1),
                ]);
            }
            let element = (e * 2 + f + 1) as ElementId;
            let (type_name, shape) = if quadratic {
                ("S8R", ElementShape::Quad8)
            } else {
                ("S4", ElementShape::Quad4)
            };
            mesh.add_element(Element {
                id: element,
                type_name: type_name.into(),
                shape,
                nodes,
            })
            .unwrap();
            faces.push((element, 1));
        }
    }
    (mesh, faces)
}

/// The default moving load with its lengths in millimetres, as the plate's are.
fn mm_default() -> MovingLoad {
    let mut load = MovingLoad::default();
    for length in [
        &mut load.start,
        &mut load.end,
        &mut load.patch_length,
        &mut load.patch_width,
    ] {
        *length *= 1000.0;
    }
    for wheel in &mut load.wheels {
        wheel.along *= 1000.0;
    }
    load
}

fn one_wheel(position: f64) -> MovingLoad {
    MovingLoad {
        start: position,
        end: position,
        positions: 1,
        wheels: vec![Wheel {
            along: 0.0,
            across: 0.0,
            force: [0.0, 0.0, -1000.0],
        }],
        ..mm_default()
    }
}

/// Total force and its point of action in x and y.
fn resultant(mesh: &FeMesh, frame: &MovingFrame) -> (f64, f64, f64) {
    let total: f64 = frame.forces.values().map(|f| f[2]).sum();
    let moment = |k: usize| -> f64 {
        (frame.forces.iter())
            .map(|(&n, f)| f[2] * mesh.node(n).unwrap()[k])
            .sum::<f64>()
            / total
    };
    (total, moment(0), moment(1))
}

#[test]
fn stations_are_spread_evenly_from_start_to_end() {
    let load = MovingLoad {
        start: 2100.0,
        end: 30650.0,
        positions: 50,
        ..mm_default()
    };
    let stations = load.stations();
    assert_eq!(stations.len(), 50);
    assert_eq!(stations[0], 2100.0);
    assert!((stations[49] - 30650.0).abs() < 1e-9);
    assert!((stations[1] - 2100.0 - 28550.0 / 49.0).abs() < 1e-9);
}

#[test]
fn the_whole_wheel_force_acts_at_the_wheel() {
    let model = FeModel::default();
    for quadratic in [false, true] {
        let (mesh, faces) = plate(quadratic);
        for position in [450.0, 437.0, 50.0] {
            let frame = one_wheel(position)
                .frames(&model, &mesh, &faces, false)
                .unwrap()
                .remove(0);
            let (total, x, y) = resultant(&mesh, &frame);
            assert!(
                (total + 1000.0).abs() < 1e-6,
                "{quadratic} {position}: {total}"
            );
            assert!((x - position).abs() < 1e-6, "{quadratic} {position}: x {x}");
            assert!(y.abs() < 1e-6, "{quadratic} {position}: y {y}");
            assert_eq!(frame.on_region, [1.0]);
        }
    }
}

#[test]
fn a_wheel_leaving_the_surface_carries_only_the_covered_part() {
    let model = FeModel::default();
    let (mesh, faces) = plate(true);
    let frame = one_wheel(1000.0)
        .frames(&model, &mesh, &faces, false)
        .unwrap()
        .remove(0);
    let (total, x, _) = resultant(&mesh, &frame);
    assert!((total + 500.0).abs() < 1e-6, "{total}");
    assert!((x - 975.0).abs() < 1e-6, "{x}");
    assert!((frame.on_region[0] - 0.5).abs() < 1e-12);

    let frame = one_wheel(1200.0)
        .frames(&model, &mesh, &faces, false)
        .unwrap()
        .remove(0);
    assert!(frame.forces.is_empty());
    assert_eq!(frame.on_region, [0.0]);
}

#[test]
fn a_fully_covered_face_gets_the_nodal_forces_of_a_uniform_pressure() {
    // A patch over exactly one quadratic element: corners -1/12, midside nodes 1/3.
    let model = FeModel::default();
    let (mesh, faces) = plate(true);
    let load = MovingLoad {
        patch_length: 100.0,
        wheels: vec![Wheel {
            along: 0.0,
            across: -50.0,
            force: [0.0, 0.0, -1200.0],
        }],
        ..one_wheel(50.0)
    };
    let frame = load.frames(&model, &mesh, &faces, false).unwrap().remove(0);
    // Element 1 spans x 0..100, y -100..0: corner node 1 at (0, -100), midside node 6 at
    // (50, -100).
    assert!(
        (frame.forces[&1][2] - 100.0).abs() < 1e-6,
        "{:?}",
        frame.forces
    );
    assert!(
        (frame.forces[&6][2] + 400.0).abs() < 1e-6,
        "{:?}",
        frame.forces
    );
}

#[test]
fn the_path_follows_the_coordinate_system() {
    // A system turned so that its z axis runs along global x, as the crane runway's system.
    let mut model = FeModel::default();
    let system = crate::CoordinateSystem::from_points(
        "Runway",
        [100.0, 0.0, 0.0],
        [100.0, 1.0, 0.0],
        [100.0, 0.0, 1.0],
    );
    let runway = system.id;
    model.coordinate_systems.push(system);
    let (mesh, faces) = plate(false);
    let load = MovingLoad {
        system: Some(runway),
        travel_axis: 2,
        lateral_axis: 0,
        ..one_wheel(300.0)
    };
    let frame = load.frames(&model, &mesh, &faces, false).unwrap().remove(0);
    let (total, x, _) = resultant(&mesh, &frame);
    assert!((total + 1000.0).abs() < 1e-6);
    assert!((x - 400.0).abs() < 1e-6, "{x}");
}

#[test]
fn problems_are_reported() {
    let model = FeModel::default();
    let mut load = mm_default();
    assert_eq!(load.problem(&model, false), None);
    load.system = Some(crate::Id::new());
    assert!(load.problem(&model, false).is_some());
    load.system = None;
    load.lateral_axis = load.travel_axis;
    assert!(load.problem(&model, false).is_some());
    assert_eq!(load.problem(&model, true), None);
    load.lateral_axis = 1;
    load.wheels.clear();
    assert!(load.problem(&model, false).is_some());
}

#[test]
fn in_a_plane_model_the_wheels_press_on_element_edges() {
    // Ten CPS8 elements from x = 0 to 1000, loaded on their top edges (face 3).
    let mut mesh = FeMesh::default();
    let id = |i: usize, j: usize| (i * 3 + j + 1) as NodeId;
    for i in 0..=20 {
        for j in 0..=2 {
            mesh.set_node(id(i, j), [i as f64 * 50.0, j as f64 * 50.0, 0.0]);
        }
    }
    let mut faces = Vec::new();
    for e in 0..10 {
        let i = 2 * e;
        let nodes = vec![
            id(i, 0),
            id(i + 2, 0),
            id(i + 2, 2),
            id(i, 2),
            id(i + 1, 0),
            id(i + 2, 1),
            id(i + 1, 2),
            id(i, 1),
        ];
        let element = (e + 1) as ElementId;
        mesh.add_element(Element {
            id: element,
            type_name: "CPS8".into(),
            shape: ElementShape::Quad8,
            nodes,
        })
        .unwrap();
        faces.push((element, 3));
    }
    let model = FeModel::default();
    let load = MovingLoad {
        wheels: vec![Wheel {
            along: 0.0,
            across: 0.0,
            force: [0.0, -1000.0, 0.0],
        }],
        ..one_wheel(437.0)
    };
    let frame = load.frames(&model, &mesh, &faces, true).unwrap().remove(0);
    let total: f64 = frame.forces.values().map(|f| f[1]).sum();
    let x: f64 = (frame.forces.iter())
        .map(|(&n, f)| f[1] * mesh.node(n).unwrap()[0])
        .sum::<f64>()
        / total;
    assert!((total + 1000.0).abs() < 1e-6, "{total}");
    assert!((x - 437.0).abs() < 1e-6, "{x}");
    assert!(
        frame
            .forces
            .keys()
            .all(|&n| mesh.node(n).unwrap()[1] == 100.0)
    );
}
