//! These tests need the Gmsh library, see [`gmsh::candidates`]. Without it they pass with a
//! note, unless `NODALIS_REQUIRE_GMSH` is set, as in the CI.

use std::path::PathBuf;

use plx_model::{Algorithm2d, Algorithm3d, MeshSetupItem, MeshSetupKind, UnitSystem};

use super::*;

// The tests read CAD files without a unit system, in millimetres as they were drawn, so that
// the sizes below are millimetres.

fn testdata(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name)
}

/// Whether Gmsh can be used; false if the test should be skipped.
fn gmsh_available() -> bool {
    match self_test() {
        Ok(_) => true,
        Err(error) if std::env::var_os("NODALIS_REQUIRE_GMSH").is_none() => {
            eprintln!("Gmsh not available, test skipped: {error}");
            false
        }
        Err(error) => panic!("Gmsh is required: {error}"),
    }
}

fn corners(mesh: &FeMesh, element: &Element) -> [glam_free::V; 4] {
    [0, 1, 2, 3].map(|k| glam_free::V(mesh.node(element.nodes[k]).unwrap()))
}

/// Just enough vector maths for the checks, without a dependency.
mod glam_free {
    #[derive(Clone, Copy)]
    pub struct V(pub [f64; 3]);

    impl V {
        pub fn sub(self, o: V) -> V {
            V([0, 1, 2].map(|k| self.0[k] - o.0[k]))
        }
        pub fn mid(self, o: V) -> V {
            V([0, 1, 2].map(|k| 0.5 * (self.0[k] + o.0[k])))
        }
        pub fn dot(self, o: V) -> f64 {
            (0..3).map(|k| self.0[k] * o.0[k]).sum()
        }
        pub fn cross(self, o: V) -> V {
            let (a, b) = (self.0, o.0);
            V([
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ])
        }
        pub fn distance(self, o: V) -> f64 {
            let d = self.sub(o);
            d.dot(d).sqrt()
        }
    }
}

/// Volume of a tetrahedron, positive when nodes 1-2-3 run anticlockwise seen from node 4 as
/// CalculiX requires.
fn volume(mesh: &FeMesh, element: &Element) -> f64 {
    let [a, b, c, d] = corners(mesh, element);
    b.sub(a).cross(c.sub(a)).dot(d.sub(a)) / 6.0
}

#[test]
fn step_files_import_with_faces_and_edges() {
    if !gmsh_available() {
        return;
    }
    let import = import_cad(&testdata("platte_mit_loch.step"), UnitSystem::Unitless).unwrap();
    let display = &import.display;
    assert_eq!(display.solids, 1);
    // Four sides, top and bottom with the hole, and the hole's wall.
    assert!(display.faces >= 7, "{} faces", display.faces);
    assert!(display.edges >= 12, "{} edges", display.edges);
    assert_eq!(display.mesh.parts.len(), 1);
    assert_eq!(display.mesh.parts[0].name, "SOLID-1");
    let types: BTreeSet<&str> = (display.mesh.elements().iter())
        .map(|e| e.type_name.as_str())
        .collect();
    assert_eq!(types, BTreeSet::from(["B31", "S3"]));
    let faces: BTreeSet<i32> = (display.entities.iter())
        .filter_map(|e| match e {
            CadEntity::Face(tag) => Some(*tag),
            _ => None,
        })
        .collect();
    assert_eq!(faces.len(), display.faces);
    let first = display.mesh.elements()[0].id;
    assert!(matches!(display.entity(first), Some(CadEntity::Face(_))));
    assert_eq!(display.entity(0), None);

    let geometry = &import.geometry;
    assert_eq!(geometry.source, "platte_mit_loch.step");
    assert!(
        geometry.brep.starts_with("DBRep_DrawableShape"),
        "BREP text"
    );
    // 5 % of the diagonal of 100 x 40 x 10, rounded.
    assert_eq!(geometry.meshing.max_size, 5.0);
}

/// The extent of the geometry's display mesh along x.
fn width(display: &GeometryDisplay) -> f64 {
    let (min, max) = display.mesh.bounds().unwrap();
    max[0] - min[0]
}

#[test]
fn several_files_import_into_one_geometry() {
    if !gmsh_available() {
        return;
    }
    let single = import_cad(&testdata("platte_mit_loch.step"), UnitSystem::Unitless).unwrap();
    let blocks = import_cad(&testdata("zwei_bloecke.step"), UnitSystem::Unitless).unwrap();
    let paths = [
        testdata("platte_mit_loch.step"),
        testdata("zwei_bloecke.step"),
    ];
    let import = import_cad_files(&paths, UnitSystem::Unitless).unwrap();
    let display = &import.display;
    assert_eq!(
        display.solids,
        single.display.solids + blocks.display.solids
    );
    assert_eq!(display.faces, single.display.faces + blocks.display.faces);
    assert_eq!(display.mesh.parts.len(), display.solids);
    assert_eq!(
        import.geometry.source,
        "platte_mit_loch.step, zwei_bloecke.step"
    );
    // A file without geometry names itself in the error.
    let empty = std::env::temp_dir().join("nodalis-leer.brep");
    std::fs::write(&empty, "").unwrap();
    let error = import_cad_files(&[paths[0].clone(), empty.clone()], UnitSystem::Unitless)
        .unwrap_err()
        .to_string();
    let _ = std::fs::remove_file(&empty);
    assert!(error.contains("nodalis-leer.brep"), "{error}");
}

#[test]
fn step_files_import_in_stored_si_units() {
    if !gmsh_available() {
        return;
    }
    // Whatever the unit system shown, the geometry is stored in metres.
    let import = import_cad(&testdata("platte_mit_loch.step"), UnitSystem::MmTonSC).unwrap();
    assert!((width(&import.display) - 0.1).abs() < 1e-6);
    assert_eq!(import.geometry.meshing.max_size, 0.005);
    // Scaled back to millimetres, the plate is as wide as one read in millimetres.
    let scaled = scale_geometry(&import.geometry, 1000.0).unwrap();
    let display = tessellate(&scaled).unwrap();
    assert!((width(&display) - 100.0).abs() < 1e-3);
    assert_eq!(display.faces, import.display.faces);
    // The option does not stick to later imports.
    let again = tessellate(&import.geometry).unwrap();
    assert!((width(&again) - 0.1).abs() < 1e-6);
}

#[test]
fn quadratic_tetrahedra_follow_calculix_numbering() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = import_cad(&testdata("platte_mit_loch.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    geometry.meshing.max_size = 8.0;
    let mesh = generate_mesh(&geometry).unwrap().mesh;
    assert_eq!(mesh.parts.len(), 1);
    assert!(mesh.element_count() > 100);
    let mut total = 0.0;
    for element in mesh.elements() {
        assert_eq!(element.type_name, "C3D10");
        let v = volume(&mesh, element);
        assert!(v > 0.0, "Element {} is inverted", element.id);
        total += v;
        // Straight midside nodes lie halfway along CalculiX's edges 1-2, 2-3, 3-1, 1-4,
        // 2-4, 3-4.
        let node = |k: usize| glam_free::V(mesh.node(element.nodes[k]).unwrap());
        for (mid, (a, b)) in [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)]
            .into_iter()
            .enumerate()
        {
            let expected = node(a).mid(node(b));
            assert!(
                node(4 + mid).distance(expected) < 1e-6,
                "Element {}",
                element.id
            );
        }
    }
    let exact = 100.0 * 40.0 * 10.0 - std::f64::consts::PI * 8.0 * 8.0 * 10.0;
    assert!((total - exact).abs() / exact < 0.01, "Volume {total}");
    assert!(mesh.missing_nodes().is_empty());
}

#[test]
fn every_solid_becomes_a_part_and_size_controls_the_count() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = import_cad(&testdata("zwei_bloecke.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    geometry.meshing.second_order = false;
    geometry.meshing.max_size = 5.0;
    let coarse = generate_mesh(&geometry).unwrap().mesh;
    let names: Vec<&str> = coarse.parts.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, ["SOLID-1", "SOLID-2"]);
    assert!(coarse.elements().iter().all(|e| e.type_name == "C3D4"));
    assert!(coarse.elements().iter().all(|e| volume(&coarse, e) > 0.0));
    // The blocks do not touch, so their meshes share no node.
    let nodes = |part: &Part| -> BTreeSet<NodeId> {
        (part.elements.iter())
            .flat_map(|&id| coarse.element(id).unwrap().nodes.clone())
            .collect()
    };
    assert!(nodes(&coarse.parts[0]).is_disjoint(&nodes(&coarse.parts[1])));

    geometry.meshing.max_size = 1.25;
    let fine = generate_mesh(&geometry).unwrap().mesh;
    assert!(
        fine.element_count() > 3 * coarse.element_count(),
        "{} vs {}",
        fine.element_count(),
        coarse.element_count()
    );
}

#[test]
fn a_remeshed_part_replaces_its_old_mesh_and_leaves_the_others() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = import_cad(&testdata("zwei_bloecke.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    geometry.meshing.second_order = false;
    geometry.meshing.max_size = 5.0;
    assert_eq!(part_names(&geometry).unwrap(), ["SOLID-1", "SOLID-2"]);
    let whole = generate_mesh(&geometry).unwrap().mesh;
    let old_first = whole.parts[0].elements.len();

    geometry.mesh_items.push(MeshSetupItem {
        name: "Meshing_Parameters-1".into(),
        kind: MeshSetupKind::MeshingParameters {
            parts: vec!["SOLID-1".into()],
            parameters: MeshingParameters {
                max_size: 1.25,
                second_order: false,
                ..geometry.meshing
            },
        },
    });
    let part = generate_part_mesh(&geometry, "SOLID-1").unwrap().mesh;
    let names: Vec<&str> = part.parts.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, ["SOLID-1"]);
    let merged = merge_part(&whole, part);
    let names: Vec<&str> = merged.parts.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, ["SOLID-1", "SOLID-2"], "the part keeps its place");
    let new_first = merged.parts[0].elements.len();
    assert!(new_first > 3 * old_first, "{new_first} vs {old_first}");
    assert!(merged.missing_nodes().is_empty());
    // The second block is untouched, with its numbers.
    assert_eq!(merged.parts[1], whole.parts[1]);
    for &id in &whole.parts[1].elements {
        let element = merged.element(id).unwrap();
        assert_eq!(element, whole.element(id).unwrap());
        for &node in &element.nodes {
            assert_eq!(merged.node(node), whole.node(node));
        }
    }
    // The new elements are numbered after all old ones.
    let old_max = whole.elements().iter().map(|e| e.id).max().unwrap();
    assert!(merged.parts[0].elements.iter().all(|&e| e > old_max));
    let used: BTreeSet<NodeId> = (merged.elements().iter())
        .flat_map(|e| e.nodes.iter().copied())
        .collect();
    assert_eq!(used.len(), merged.node_count(), "no orphaned nodes");
    assert!(generate_part_mesh(&geometry, "SOLID-9").is_err());

    // The CAD map follows: the second block's entities keep their numbers, the first
    // block's point into its new mesh.
    let face_of = |mesh: &FeMesh, part: usize| -> BTreeSet<i32> {
        let elements: BTreeSet<ElementId> = mesh.parts[part].elements.iter().copied().collect();
        (mesh.cad.faces.iter())
            .filter(|(_, faces)| faces.iter().all(|(e, _)| elements.contains(e)))
            .map(|(&tag, _)| tag)
            .collect()
    };
    assert_eq!(face_of(&merged, 0), face_of(&whole, 0));
    assert_eq!(face_of(&merged, 1), face_of(&whole, 1));
    assert_eq!(merged.cad.faces.len(), 12);
    for tag in face_of(&whole, 1) {
        assert_eq!(merged.cad.faces[&tag], whole.cad.faces[&tag]);
    }
    for tag in face_of(&merged, 0) {
        let faces = &merged.cad.faces[&tag];
        assert!(faces.len() > whole.cad.faces[&tag].len());
        assert!(faces.iter().all(|(e, _)| *e > old_max));
    }
    for nodes in merged.cad.nodes.values() {
        assert!(nodes.iter().all(|&n| merged.node(n).is_some()));
    }
}

#[test]
fn meshes_record_where_the_cad_entities_lie() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = import_cad(&testdata("platte_mit_loch.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    geometry.meshing.max_size = 8.0;
    let mesh = generate_mesh(&geometry).unwrap().mesh;
    let cad = &mesh.cad;
    let display = tessellate(&geometry).unwrap();
    assert_eq!(cad.faces.len(), display.faces);
    assert_eq!(cad.segments.len(), display.edges);
    assert!(cad.nodes.keys().any(|e| matches!(e, CadEntity::Vertex(_))));
    // Every outer face of the tetrahedra lies on exactly one CAD face.
    let mut count: HashMap<Vec<NodeId>, usize> = HashMap::new();
    for element in mesh.elements() {
        for face in element.faces() {
            let mut corners: Vec<NodeId> = face.corners.iter().map(|&i| element.nodes[i]).collect();
            corners.sort_unstable();
            *count.entry(corners).or_default() += 1;
        }
    }
    let outer = count.values().filter(|&&n| n == 1).count();
    let on_cad: usize = cad.faces.values().map(Vec::len).sum();
    assert_eq!(on_cad, outer);
    // The faces' nodes, midside nodes included, are the CAD face's nodes.
    for (&tag, faces) in &cad.faces {
        let nodes: BTreeSet<NodeId> = (faces.iter())
            .flat_map(|&(e, f)| {
                let element = mesh.element(e).unwrap();
                let face = &element.faces()[usize::from(f) - 1];
                let local = face.corners.iter().chain(face.mids);
                local.map(|&i| element.nodes[i]).collect::<Vec<_>>()
            })
            .collect();
        let recorded: BTreeSet<NodeId> = cad.nodes[&CadEntity::Face(tag)].iter().copied().collect();
        assert_eq!(nodes, recorded, "Face {tag}");
    }
    for (&tag, segments) in &cad.segments {
        let nodes = &cad.nodes[&CadEntity::Edge(tag)];
        // Quadratic segments have a midside node each; the hole's edge is closed.
        let n = 2 * segments.len();
        assert!(nodes.len() == n + 1 || nodes.len() == n, "Edge {tag}");
    }
}

#[test]
fn local_mesh_sizes_refine_faces_and_edges() {
    if !gmsh_available() {
        return;
    }
    let import = import_cad(&testdata("platte_mit_loch.step"), UnitSystem::Unitless).unwrap();
    let mut geometry = import.geometry;
    geometry.meshing.max_size = 8.0;
    geometry.meshing.second_order = false;
    let count = |geometry: &Geometry| {
        (generate_part_mesh(geometry, "SOLID-1").unwrap().mesh).element_count()
    };
    let coarse = count(&geometry);
    // The faces of the plate in the display; the first one is refined.
    let face = (import.display.entities.iter())
        .find_map(|e| match e {
            CadEntity::Face(tag) => Some(*tag),
            _ => None,
        })
        .unwrap();
    let local = |faces: Vec<i32>, edges: Vec<i32>, size| MeshSetupItem {
        name: "Local_Mesh_Size-1".into(),
        kind: MeshSetupKind::LocalMeshSize { faces, edges, size },
    };
    geometry.mesh_items = vec![local(vec![face], vec![], 1.0)];
    let on_face = count(&geometry);
    assert!(on_face > 2 * coarse, "{on_face} vs {coarse}");
    let edge = (import.display.entities.iter())
        .find_map(|e| match e {
            CadEntity::Edge(tag) => Some(*tag),
            _ => None,
        })
        .unwrap();
    geometry.mesh_items = vec![local(vec![], vec![edge], 0.5)];
    let on_edge = count(&geometry);
    assert!(on_edge > coarse, "{on_edge} vs {coarse}");
    // Tags of other parts or faces that do not exist change nothing.
    geometry.mesh_items = vec![local(vec![9999], vec![], 1.0)];
    assert_eq!(count(&geometry), coarse);
}

#[test]
fn every_gmsh_algorithm_gives_valid_tetrahedra() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = import_cad(&testdata("platte_mit_loch.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    geometry.meshing.max_size = 8.0;
    let exact = 100.0 * 40.0 * 10.0 - std::f64::consts::PI * 8.0 * 8.0 * 10.0;
    for (algorithm_2d, algorithm_3d) in [
        (Algorithm2d::MeshAdapt, Algorithm3d::Hxt),
        (Algorithm2d::Delaunay, Algorithm3d::Frontal),
        (Algorithm2d::Automatic, Algorithm3d::Delaunay),
    ] {
        geometry.mesh_items = vec![MeshSetupItem {
            name: "Tetrahedral_Gmsh-1".into(),
            kind: MeshSetupKind::TetrahedralGmsh {
                parts: vec!["SOLID-1".into()],
                algorithm_2d,
                algorithm_3d,
            },
        }];
        let mesh = generate_mesh(&geometry).unwrap().mesh;
        let total: f64 = mesh.elements().iter().map(|e| volume(&mesh, e)).sum();
        assert!(
            mesh.elements().iter().all(|e| volume(&mesh, e) > 0.0),
            "{algorithm_2d:?} {algorithm_3d:?}"
        );
        assert!(
            (total - exact).abs() / exact < 0.01,
            "{algorithm_2d:?} {algorithm_3d:?}: {total}"
        );
    }
}

#[test]
fn invalid_sizes_and_files_are_reported() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = import_cad(&testdata("zwei_bloecke.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    geometry.meshing.max_size = 0.0;
    assert!(generate_mesh(&geometry).is_err());
    assert!(import_cad(&testdata("wuerfel_c3d10.inp"), UnitSystem::Unitless).is_err());
    assert!(import_cad(&testdata("gibt_es_nicht.step"), UnitSystem::Unitless).is_err());
}

#[test]
fn names_are_made_calculix_safe() {
    assert_eq!(
        calculix_name("Bracket v2.1").as_deref(),
        Some("BRACKET_V2_1")
    );
    assert_eq!(calculix_name("  "), None);
    assert_eq!(calculix_name("3D-Teil"), None);
    assert!(is_cad_file(std::path::Path::new("a/B.STEP")));
    assert!(!is_cad_file(std::path::Path::new("a/b.inp")));
}

/// A geometry of one rounded rectangle in the x-y plane, as a 2D model has it.
fn rectangle(origin: [f64; 3], size: [f64; 2], radius: f64) -> Geometry {
    let file = TempFile::new("brep");
    with_gmsh(|gmsh| {
        gmsh.add_rectangle(origin, size, radius)?;
        gmsh.write(&file.0)
    })
    .unwrap();
    Geometry {
        source: "rechteck.brep".into(),
        brep: std::fs::read_to_string(&file.0).unwrap(),
        meshing: MeshingParameters::for_diagonal(size[0].hypot(size[1])),
        mesh_items: Vec::new(),
        part_names: Vec::new(),
        refinement: 1.0,
        signatures: Vec::new(),
    }
}

/// Signed area of a triangle or quadrilateral in the x-y plane, positive counter-clockwise.
fn signed_area(mesh: &FeMesh, element: &Element) -> f64 {
    let n = if element.shape.edges().len() == 3 {
        3
    } else {
        4
    };
    let p: Vec<[f64; 3]> = (0..n)
        .map(|k| mesh.node(element.nodes[k]).unwrap())
        .collect();
    (0..n)
        .map(|k| {
            let (a, b) = (p[k], p[(k + 1) % n]);
            a[0] * b[1] - a[1] * b[0]
        })
        .sum::<f64>()
        / 2.0
}

#[test]
fn faces_outside_solids_are_meshed_as_shell_parts() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = rectangle([10.0, 0.0, 0.0], [20.0, 10.0], 2.0);
    assert_eq!(part_names(&geometry).unwrap(), ["SHELL-1"]);
    let display = tessellate(&geometry).unwrap();
    assert_eq!(display.mesh.parts[0].name, "SHELL-1");

    geometry.meshing.max_size = 2.0;
    let mesh = generate_mesh(&geometry).unwrap().mesh;
    assert_eq!(mesh.parts.len(), 1);
    assert!(mesh.elements().iter().all(|e| e.type_name == "S6"));
    // Midside nodes lie between their corners, as CalculiX numbers them.
    for element in mesh.elements() {
        for edge in element.shape.edges() {
            let p = |l: usize| mesh.node(element.nodes[l]).unwrap();
            let (a, b, m) = (p(edge.corners[0]), p(edge.corners[1]), p(edge.mids[0]));
            let off = (0..3)
                .map(|k| (m[k] - (a[k] + b[k]) / 2.0).abs())
                .fold(0.0, f64::max);
            assert!(off < 0.05, "Element {}: {off}", element.id);
        }
    }
    let area: f64 = mesh
        .elements()
        .iter()
        .map(|e| signed_area(&mesh, e).abs())
        .sum();
    let exact = 200.0 - (4.0 - std::f64::consts::PI) * 4.0;
    assert!((area - exact).abs() / exact < 0.01, "Area {area}");
    assert!(mesh.coords().iter().all(|c| c[2].abs() < 1e-9));
    // The face covers every element; the edges run around it.
    let faces: Vec<(ElementId, u8)> = mesh.cad.faces.values().flatten().copied().collect();
    assert_eq!(faces.len(), mesh.element_count());
    assert!(faces.iter().all(|&(_, f)| f == 1));
    assert_eq!(mesh.cad.segments.len(), 8, "4 sides, 4 fillets");
    assert_eq!(
        mesh.cad_nodes(&[CadEntity::Face(*mesh.cad.faces.keys().next().unwrap())])
            .len(),
        mesh.node_count()
    );

    geometry.meshing.second_order = false;
    geometry.meshing.quad_dominated = true;
    let quads = generate_mesh(&geometry).unwrap().mesh;
    let count = |name: &str| {
        quads
            .elements()
            .iter()
            .filter(|e| e.type_name == name)
            .count()
    };
    assert!(
        count("S4") > 10 * count("S3"),
        "{} S4, {} S3",
        count("S4"),
        count("S3")
    );
    geometry.meshing.second_order = true;
    let quadratic = generate_mesh(&geometry).unwrap().mesh;
    assert!(quadratic.elements().iter().any(|e| e.type_name == "S8"));
}

/// A geometry of a box and, beside it, an L of two lines: one along x, one up along z.
fn box_and_lines() -> Geometry {
    let file = TempFile::new("brep");
    with_gmsh(|gmsh| {
        gmsh.add_box([0.0; 3], [10.0; 3])?;
        let a = gmsh.add_point([20.0, 0.0, 0.0])?;
        let b = gmsh.add_point([50.0, 0.0, 0.0])?;
        let c = gmsh.add_point([50.0, 0.0, 20.0])?;
        gmsh.add_line(a, b)?;
        gmsh.add_line(b, c)?;
        gmsh.write(&file.0)
    })
    .unwrap();
    Geometry {
        source: "rahmen.brep".into(),
        brep: std::fs::read_to_string(&file.0).unwrap(),
        meshing: MeshingParameters {
            max_size: 5.0,
            ..MeshingParameters::default()
        },
        mesh_items: Vec::new(),
        part_names: Vec::new(),
        refinement: 1.0,
        signatures: Vec::new(),
    }
}

#[test]
fn a_long_thin_round_bar_is_displayed_with_few_triangles() {
    if !gmsh_available() {
        return;
    }
    // A bar 6000 long and 5 thick, the size of the fillets along a beam: triangles as wide as
    // long would number hundreds of thousands, and the import would take minutes.
    let file = TempFile::new("brep");
    with_gmsh(|gmsh| {
        let rectangle = gmsh.add_rectangle([0.0; 3], [6000.0, 5.0], 0.0)?;
        gmsh.revolve(
            &[(2, rectangle)],
            [0.0; 3],
            [1.0, 0.0, 0.0],
            std::f64::consts::TAU,
        )?;
        gmsh.remove_only(&[(2, rectangle)])?;
        gmsh.write(&file.0)
    })
    .unwrap();
    let geometry = Geometry {
        source: "stab.brep".into(),
        brep: std::fs::read_to_string(&file.0).unwrap(),
        meshing: MeshingParameters::default(),
        mesh_items: Vec::new(),
        part_names: Vec::new(),
        refinement: 1.0,
        signatures: Vec::new(),
    };
    let display = tessellate(&geometry).unwrap();
    let triangles = display
        .entities
        .iter()
        .filter(|e| matches!(e, CadEntity::Face(_)))
        .count();
    assert!((10..5000).contains(&triangles), "{triangles} triangles");
    // Every face and every edge is shown.
    let shown: BTreeSet<CadEntity> = display.entities.iter().copied().collect();
    assert_eq!(shown.len(), display.faces + display.edges);
}

#[test]
fn edges_outside_faces_are_meshed_as_line_parts() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = box_and_lines();
    assert_eq!(
        part_names(&geometry).unwrap(),
        ["SOLID-1", "LINE-1", "LINE-2"]
    );
    // The display shows the free edges as lines of their own parts, not of the box.
    let display = tessellate(&geometry).unwrap();
    let names: Vec<&str> = display.mesh.parts.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, ["SOLID-1", "LINE-1", "LINE-2"]);
    let line_part = &display.mesh.parts[1];
    assert!(line_part.elements.iter().all(|&id| {
        let element = display.mesh.element(id).unwrap();
        element.shape == ElementShape::Line2
            && matches!(display.entity(id), Some(CadEntity::Edge(_)))
    }));
    assert_eq!(display.edges, 12 + 2);

    // Quadratic lines of 5 units: the 30 long edge gets 6 B32 with the midside node in the
    // middle, the 20 long one 4.
    let mesh = generate_mesh(&geometry).unwrap().mesh;
    assert_eq!(mesh.parts.len(), 3);
    let lines: Vec<&Element> = (mesh.parts[1].elements.iter())
        .map(|&id| mesh.element(id).unwrap())
        .collect();
    assert_eq!(lines.len(), 6, "{lines:?}");
    for element in &lines {
        assert_eq!(element.type_name, "B32");
        let p = |k: usize| mesh.node(element.nodes[k]).unwrap();
        let (a, m, b) = (p(0), p(1), p(2));
        assert!((m[0] - (a[0] + b[0]) / 2.0).abs() < 1e-9, "{element:?}");
        assert!((b[0] - a[0]).abs() - 5.0 < 1e-6);
    }
    assert_eq!(mesh.parts[2].elements.len(), 4);
    // The two lines meet at (50, 0, 0), each with a node of its own there, as parts share
    // no nodes; the CAD map knows the edges, their segments and the vertex at the corner,
    // with both nodes.
    let corner = |part: usize, k: usize| mesh.element(mesh.parts[part].elements[k]).unwrap();
    let ends = [corner(1, 5).nodes[2], corner(2, 0).nodes[0]];
    assert_ne!(ends[0], ends[1]);
    for end in ends {
        assert_eq!(mesh.node(end).unwrap(), [50.0, 0.0, 0.0]);
    }
    let edge_nodes = |edge: i32| mesh.cad.nodes.get(&CadEntity::Edge(edge)).cloned();
    assert_eq!(edge_nodes(13).map(|n| n.len()), Some(13));
    assert_eq!(edge_nodes(14).map(|n| n.len()), Some(9));
    assert_eq!(mesh.cad.segments[&14].len(), 4);
    let vertex_nodes: Vec<Vec<_>> = (mesh.cad.nodes.iter())
        .filter(|(e, n)| matches!(e, CadEntity::Vertex(_)) && n.contains(&ends[0]))
        .map(|(_, n)| n.clone())
        .collect();
    assert_eq!(vertex_nodes, [ends.to_vec()], "{:?}", mesh.cad.nodes);
    // The contact search finds the two ends as one joint.
    let joints = plx_mesh::find_line_joints(&mesh, &[true; 3], 1e-6);
    assert_eq!(joints.len(), 1);
    assert_eq!(joints[0].nodes, ends.to_vec());
    assert_eq!(joints[0].parts, BTreeSet::from([1, 2]));
    // The lines and the box share no nodes; the parts keep their own numbering.
    assert!(
        mesh.parts[0]
            .elements
            .iter()
            .all(|&id| mesh.element(id).unwrap().type_name == "C3D10")
    );

    geometry.meshing.second_order = false;
    let part = generate_part_mesh(&geometry, "LINE-2").unwrap().mesh;
    assert_eq!(part.parts[0].name, "LINE-2");
    assert_eq!(part.element_count(), 4);
    assert!(part.elements().iter().all(|e| e.type_name == "B31"));
    // A local mesh size on the edge refines it.
    geometry.mesh_items.push(plx_model::MeshSetupItem {
        name: "Local_Mesh_Size-1".into(),
        kind: plx_model::MeshSetupKind::LocalMeshSize {
            faces: vec![],
            edges: vec![14],
            size: 1.0,
        },
    });
    let fine = generate_part_mesh(&geometry, "LINE-2").unwrap().mesh;
    assert_eq!(fine.element_count(), 20);
}

#[test]
fn a_deleted_part_leaves_the_others_with_their_names_and_local_sizes() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = import_cad(&testdata("zwei_bloecke.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    geometry.meshing.second_order = false;
    geometry.meshing.max_size = 5.0;
    // A face of each block; the second block's face must follow it into the new numbering.
    let file = TempFile::with_contents("brep", &geometry.brep).unwrap();
    let (first, second, second_box) = with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        let volumes = gmsh.entities(3)?;
        let first = gmsh.adjacencies(3, volumes[0])?.1[0];
        let second = *gmsh.adjacencies(3, volumes[1])?.1.last().unwrap();
        Ok((first, second, gmsh.entity_bounding_box(2, second)?))
    })
    .unwrap();
    geometry.mesh_items.push(MeshSetupItem {
        name: "Local_Mesh_Size-1".into(),
        kind: MeshSetupKind::LocalMeshSize {
            faces: vec![first, second],
            edges: Vec::new(),
            size: 1.0,
        },
    });

    let smaller = delete_part(&geometry, "SOLID-1").unwrap().unwrap();
    // Gmsh alone would call the block that is left SOLID-1.
    assert_eq!(part_names(&smaller).unwrap(), ["SOLID-2"]);
    let mesh = generate_mesh(&smaller).unwrap().mesh;
    let names: Vec<&str> = mesh.parts.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, ["SOLID-2"]);
    let display = tessellate(&smaller).unwrap();
    assert_eq!(display.mesh.parts[0].name, "SOLID-2");
    let MeshSetupKind::LocalMeshSize { faces, .. } = &smaller.mesh_items[0].kind else {
        panic!("local mesh size expected");
    };
    assert_eq!(faces.len(), 1, "the face of the deleted block is dropped");
    let file = TempFile::with_contents("brep", &smaller.brep).unwrap();
    let moved = with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        gmsh.entity_bounding_box(2, faces[0])
    })
    .unwrap();
    assert!(same_box(&moved, &second_box));

    // The mesh of the block that stays follows the new numbers onto the same entities a
    // fresh mesh of the smaller geometry has.
    let (_, tags) = delete_part_renumbered(&geometry, "SOLID-1").unwrap();
    let whole = generate_mesh(&geometry).unwrap().mesh;
    let left = delete_mesh_part(&whole, "SOLID-1");
    let renumbered = left.cad.renumbered(&tags);
    let keys = |map: &CadMap| map.nodes.keys().copied().collect::<Vec<_>>();
    assert_eq!(keys(&renumbered), keys(&mesh.cad));
    assert_eq!(
        renumbered.faces.keys().collect::<Vec<_>>(),
        mesh.cad.faces.keys().collect::<Vec<_>>()
    );

    assert!(delete_part(&smaller, "SOLID-1").is_err());
    assert_eq!(delete_part(&smaller, "SOLID-2").unwrap(), None);
}

#[test]
fn parts_deleted_together_are_renumbered_as_one_by_one() {
    if !gmsh_available() {
        return;
    }
    let geometry = import_cad(&testdata("zwei_bloecke.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    let paths = [testdata("platte_mit_loch.step")];
    let three = add_cad_files(Some(&geometry), &paths, UnitSystem::Unitless, &[])
        .unwrap()
        .import
        .geometry;
    let mut names = part_names(&three).unwrap();
    names.sort();
    assert_eq!(names, ["SOLID-1", "SOLID-2", "SOLID-3"]);

    let deleted = ["SOLID-1".to_string(), "SOLID-3".to_string()];
    let (together, tags) = delete_parts_renumbered(&three, &deleted).unwrap();
    let together = together.unwrap();
    assert_eq!(part_names(&together).unwrap(), ["SOLID-2"]);

    let (first, first_tags) = delete_part_renumbered(&three, "SOLID-1").unwrap();
    let (second, second_tags) = delete_part_renumbered(&first.unwrap(), "SOLID-3").unwrap();
    assert_eq!(part_names(&second.unwrap()).unwrap(), ["SOLID-2"]);
    // The renumbering of both at once is that of one after the other.
    let chained: BTreeMap<_, _> = (first_tags.iter())
        .filter_map(|(old, mid)| Some((*old, *second_tags.get(mid)?)))
        .collect();
    assert!(!tags.is_empty());
    assert_eq!(tags, chained);

    assert!(delete_parts_renumbered(&three, &["SOLID-9".to_string()]).is_err());
    let all = ["SOLID-1", "SOLID-2", "SOLID-3"].map(String::from);
    assert_eq!(delete_parts_renumbered(&three, &all).unwrap().0, None);
}

#[test]
fn the_display_of_parts_that_stay_is_kept_as_tessellated_anew() {
    if !gmsh_available() {
        return;
    }
    let geometry = import_cad(&testdata("zwei_bloecke.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    let paths = [testdata("platte_mit_loch.step")];
    let three = add_cad_files(Some(&geometry), &paths, UnitSystem::Unitless, &[])
        .unwrap()
        .import
        .geometry;
    let display = tessellate(&three).unwrap();
    let deleted = ["SOLID-1".to_string(), "SOLID-3".to_string()];
    let (smaller, tags) = delete_parts_renumbered(&three, &deleted).unwrap();
    let smaller = smaller.unwrap();
    let kept = display
        .clone()
        .without_parts(&deleted, &smaller, &tags)
        .unwrap();
    let fresh = tessellate(&smaller).unwrap();
    assert_eq!(kept.part_dimensions, fresh.part_dimensions);
    assert_eq!(kept.topology, fresh.topology);
    assert_eq!(
        (kept.solids, kept.faces, kept.edges),
        (fresh.solids, fresh.faces, fresh.edges)
    );
    let names = |d: &GeometryDisplay| -> Vec<String> {
        d.mesh.parts.iter().map(|p| p.name.clone()).collect()
    };
    assert_eq!(names(&kept), ["SOLID-2"]);
    assert_eq!(names(&kept), names(&fresh));
    // The same faces and edges, each drawn with the same number of triangles and lines.
    let counts = |d: &GeometryDisplay| -> BTreeMap<CadEntity, usize> {
        let mut counts = BTreeMap::new();
        for element in d.mesh.elements() {
            *counts.entry(d.entity(element.id).unwrap()).or_default() += 1;
        }
        counts
    };
    assert_eq!(counts(&kept), counts(&fresh));
    // A display the smaller geometry does not fit is tessellated anew.
    assert!(
        display
            .without_parts(&deleted[..1], &smaller, &tags)
            .is_none()
    );
}

#[test]
fn mesh_parts_deleted_together_are_deleted_as_one_by_one() {
    if !gmsh_available() {
        return;
    }
    let geometry = import_cad(&testdata("zwei_bloecke.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    let paths = [testdata("platte_mit_loch.step")];
    let mut three = add_cad_files(Some(&geometry), &paths, UnitSystem::Unitless, &[])
        .unwrap()
        .import
        .geometry;
    three.meshing.second_order = false;
    three.meshing.max_size = 5.0;
    let whole = generate_mesh(&three).unwrap().mesh;
    let deleted = ["solid-1".to_string(), "SOLID-3".to_string()];
    let together = delete_mesh_parts(&whole, &deleted);
    let one_by_one = delete_mesh_part(&delete_mesh_part(&whole, "solid-1"), "SOLID-3");
    assert_eq!(together.parts, one_by_one.parts);
    assert_eq!(together.node_ids(), one_by_one.node_ids());
    assert_eq!(together.elements(), one_by_one.elements());
    assert_eq!(together.parts.len(), 1);
}

#[test]
fn a_deleted_mesh_part_takes_its_elements_and_nodes() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = import_cad(&testdata("zwei_bloecke.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    geometry.meshing.second_order = false;
    geometry.meshing.max_size = 5.0;
    let whole = generate_mesh(&geometry).unwrap().mesh;
    let smaller = delete_mesh_part(&whole, "solid-1");
    assert_eq!(smaller.parts, whole.parts[1..]);
    assert_eq!(smaller.element_count(), whole.parts[1].elements.len());
    assert!(smaller.missing_nodes().is_empty());
    assert!(smaller.node_count() < whole.node_count());
}

#[test]
fn added_files_keep_the_old_parts_with_names_sizes_and_faces() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = import_cad(&testdata("zwei_bloecke.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    // After a deletion the remaining block is SOLID-2 and has a local mesh size.
    let file = TempFile::with_contents("brep", &geometry.brep).unwrap();
    let second = with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        let volumes = gmsh.entities(3)?;
        Ok(*gmsh.adjacencies(3, volumes[1])?.1.last().unwrap())
    })
    .unwrap();
    geometry.mesh_items.push(MeshSetupItem {
        name: "Local_Mesh_Size-1".into(),
        kind: MeshSetupKind::LocalMeshSize {
            faces: vec![second],
            edges: Vec::new(),
            size: 1.0,
        },
    });
    let (geometry, tags) = delete_part_renumbered(&geometry, "SOLID-1").unwrap();
    let geometry = geometry.unwrap();
    let MeshSetupKind::LocalMeshSize { faces, .. } = &geometry.mesh_items[0].kind else {
        unreachable!()
    };
    let face = faces[0];
    assert_eq!(tags[&CadEntity::Face(second)], CadEntity::Face(face));

    // A mesh part of the model is called SOLID-1, so the plate gets the next free name.
    let taken = ["SOLID-1".to_string()];
    let paths = [testdata("platte_mit_loch.step")];
    let addition = add_cad_files(Some(&geometry), &paths, UnitSystem::Unitless, &taken).unwrap();
    let combined = &addition.import.geometry;
    assert_eq!(addition.added, ["SOLID-3"]);
    let mut names = part_names(combined).unwrap();
    names.sort();
    assert_eq!(names, ["SOLID-2", "SOLID-3"]);
    assert_eq!(addition.import.display.solids, 2);
    assert_eq!(combined.source, "zwei_bloecke.step, platte_mit_loch.step");
    assert_eq!(combined.meshing, geometry.meshing);
    // Every old face is found again, the local mesh size follows its face.
    let old_faces = with_gmsh(|gmsh| {
        let file = TempFile::with_contents("brep", &geometry.brep)?;
        gmsh.import_shapes(&file.0)?;
        gmsh.entities(2)
    })
    .unwrap();
    for tag in &old_faces {
        assert!(addition.renumbered.contains_key(&CadEntity::Face(*tag)));
    }
    let MeshSetupKind::LocalMeshSize { faces, .. } = &combined.mesh_items[0].kind else {
        unreachable!()
    };
    assert_eq!(
        addition.renumbered[&CadEntity::Face(face)],
        CadEntity::Face(faces[0])
    );
    // The same block imported again lies on the old one but is a new part.
    let again = [testdata("zwei_bloecke.step")];
    let addition = add_cad_files(Some(combined), &again, UnitSystem::Unitless, &[]).unwrap();
    let mut names = part_names(&addition.import.geometry).unwrap();
    names.sort();
    assert_eq!(names, ["SOLID-1", "SOLID-2", "SOLID-3", "SOLID-4"]);
    assert_eq!(addition.added, ["SOLID-1", "SOLID-4"]);

    // Into a model with a mesh but no geometry, the new parts avoid the mesh's names.
    let taken = ["SOLID-1".to_string(), "SOLID-2".to_string()];
    let addition = add_cad_files(None, &again, UnitSystem::Unitless, &taken).unwrap();
    assert_eq!(addition.added, ["SOLID-3", "SOLID-4"]);
    assert_eq!(
        part_names(&addition.import.geometry).unwrap(),
        addition.added
    );
    assert!(addition.renumbered.is_empty());
}

/// A 2D geometry of two rectangles in the x-y plane: a beam (0..40 x 0..5) lying on a block
/// (10..30 x -10..0), touching along y = 0.
fn beam_on_block() -> Geometry {
    let file = TempFile::new("brep");
    with_gmsh(|gmsh| {
        gmsh.add_rectangle([0.0, 0.0, 0.0], [40.0, 5.0], 0.0)?;
        gmsh.add_rectangle([10.0, -10.0, 0.0], [20.0, 10.0], 0.0)?;
        gmsh.write(&file.0)
    })
    .unwrap();
    Geometry {
        source: "balken.brep".into(),
        brep: std::fs::read_to_string(&file.0).unwrap(),
        meshing: MeshingParameters::for_diagonal(40.0),
        mesh_items: Vec::new(),
        part_names: Vec::new(),
        refinement: 1.0,
        signatures: Vec::new(),
    }
}

/// The contact search on a 2D mesh finds the edges along y = 0; they are whole CAD edges,
/// so a contact pair keeps them by geometry and finds them again on a finer mesh.
#[test]
fn contact_edges_found_in_2d_survive_remeshing() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = beam_on_block();
    let mesh_with = |size: f64, geometry: &mut Geometry| {
        geometry.meshing.max_size = size;
        let mut mesh = generate_mesh(geometry).unwrap().mesh;
        plx_model::ModelSpace::PlaneStrain
            .prepare_generated_mesh(&mut mesh)
            .unwrap();
        mesh
    };
    let coarse = mesh_with(5.0, &mut geometry);
    let skins: Vec<_> = (coarse.parts.iter())
        .map(|p| plx_mesh::extract_part_skin(&coarse, p, 30.0))
        .collect();
    let parameters = plx_mesh::SearchParameters {
        distance: 0.01,
        angle_deg: 35.0,
        group_by: plx_mesh::GroupBy::Parts,
        stiffness: Vec::new(),
        filter: plx_mesh::GeometryFilter::default(),
        one_sided: Vec::new(),
        solid_skins: Vec::new(),
    };
    let searched = vec![true; coarse.parts.len()];
    let items = plx_mesh::find_contact_pairs(&coarse, &skins, &searched, &parameters);
    assert_eq!(items.len(), 1, "{items:?}");
    let on_contact = |mesh: &FeMesh, faces: &[(ElementId, u8)]| {
        assert!(!faces.is_empty());
        for &(element, face) in faces {
            let element = mesh.element(element).unwrap();
            let edge = &element.faces()[usize::from(face) - 1];
            for &corner in edge.corners {
                let y = mesh.node(element.nodes[corner]).unwrap()[1];
                assert!(y.abs() < 1e-6, "Edge at y = {y}");
            }
        }
    };
    let mut entities = Vec::new();
    for side in [&items[0].master, &items[0].slave] {
        let faces = plx_mesh::surface_faces(&coarse, &skins, side);
        on_contact(&coarse, &faces);
        let whole = coarse.whole_cad_faces(&faces).expect("whole CAD edges");
        assert!(
            whole.iter().all(|e| matches!(e, CadEntity::Edge(_))),
            "{whole:?}"
        );
        entities.push((whole, faces.len()));
    }
    let fine = mesh_with(1.0, &mut geometry);
    for (whole, coarse_count) in entities {
        let faces = fine.cad_faces(&whole);
        on_contact(&fine, &faces);
        assert!(
            faces.len() > coarse_count,
            "{} <= {coarse_count}",
            faces.len()
        );
    }
}

/// Four blocks meeting at one edge, like the slices of a lug cut along two planes: each
/// block touches two neighbours with one face each, and its two faces at the junction lie in
/// one plane with faces of the blocks beside them. The detection on the geometry finds four
/// regions of one face a side; the borders of the faces, where the triangles of the display
/// end, once joined the faces beside them into the regions and the regions into one, whose
/// ties then pulled nodes across the junction.
#[test]
fn detection_on_geometry_keeps_to_the_faces_that_touch() {
    if !gmsh_available() {
        return;
    }
    let file = TempFile::new("brep");
    with_gmsh(|gmsh| {
        for origin in [[0.0, 0.0], [0.0, 20.0], [30.0, 0.0], [30.0, 20.0]] {
            gmsh.add_box([origin[0], origin[1], 0.0], [30.0, 20.0, 10.0])?;
        }
        gmsh.write(&file.0)
    })
    .unwrap();
    let geometry = Geometry {
        source: "bloecke.brep".into(),
        brep: std::fs::read_to_string(&file.0).unwrap(),
        meshing: MeshingParameters::for_diagonal(60.0),
        mesh_items: Vec::new(),
        part_names: Vec::new(),
        refinement: 1.0,
        signatures: Vec::new(),
    };
    let display = tessellate(&geometry).unwrap();
    let skins: Vec<_> = (display.mesh.parts.iter())
        .map(|part| {
            let mut skin = plx_mesh::extract_part_skin(&display.mesh, part, 180.0);
            for face in &mut skin.faces {
                let id = display.mesh.elements()[face.element].id;
                if let Some(CadEntity::Face(tag)) = display.entity(id) {
                    face.region = tag as usize;
                }
            }
            skin
        })
        .collect();
    let mut model = plx_model::FeModel::default();
    model
        .contact_detections
        .push(plx_model::ContactDetection::new("Contacts"));
    let geometry = plx_model::DetectionGeometry {
        mesh: &display.mesh,
        skins: &skins,
        entities: &display.entities,
        face_edges: &display.topology.face_edges,
        part_dimensions: &display.part_dimensions,
    };
    let visible = vec![true; display.mesh.parts.len()];
    let report = model
        .detect_contacts_on_geometry(0, &geometry, &visible)
        .unwrap();
    assert_eq!(report.added, 4, "{:#?}", model.contact_regions);
    // Each side is the one face of its block at the plane between the two blocks.
    let mut mesh = display.mesh.clone();
    mesh.cad = plx_mesh::display_cad_map(&mesh, &display.entities, &display.topology.face_edges);
    let plane_of = |region: &plx_model::Region| {
        let plx_model::Region::Geometry(entities) = region else {
            panic!("{region:?}");
        };
        assert_eq!(entities.len(), 1, "{entities:?}");
        let nodes = mesh.cad_nodes(entities);
        let points: Vec<[f64; 3]> = nodes.iter().map(|&n| mesh.node(n).unwrap()).collect();
        let flat = |k: usize| points.iter().all(|p| (p[k] - points[0][k]).abs() < 1e-6);
        (flat(0), flat(1), points[0])
    };
    for region in &model.contact_regions {
        let (a, b) = (plane_of(&region.side_a), plane_of(&region.side_b));
        assert_eq!((a.0, a.1), (b.0, b.1), "{}", region.name);
        let k = if a.0 { 0 } else { 1 };
        assert!(
            a.0 != a.1 && (a.2[k] - b.2[k]).abs() < 1e-6,
            "{}",
            region.name
        );
    }
}

/// A face split in the CAD program, here a ring on the upper shaft, is a face of its own on
/// the mesh: its border is drawn and it can be picked although nothing bends there.
#[test]
fn split_faces_keep_their_outline_on_the_mesh() {
    if !gmsh_available() {
        return;
    }
    let geometry = import_cad(
        &testdata("welle_mit_teilflaeche.step"),
        UnitSystem::Unitless,
    )
    .unwrap()
    .geometry;
    let mesh = generate_mesh(&geometry).unwrap().mesh;
    // Two cylinders, two end faces, the shoulder and the ring splitting the upper cylinder.
    assert_eq!(mesh.cad.faces.len(), 7);
    let skin = plx_mesh::extract_part_skin(&mesh, &mesh.parts[0], 30.0);
    let regions: BTreeSet<usize> = skin.faces.iter().map(|f| f.region).collect();
    assert_eq!(regions.len(), 7);
    // The ring's borders at z = 28 and 32 are feature edges.
    let coords = mesh.coords();
    for z in [28.0, 32.0] {
        let on_circle = (skin.edges.iter())
            .filter(|e| e.feature)
            .filter(|e| [e.a, e.b].iter().all(|&n| (coords[n][2] - z).abs() < 1e-6))
            .count();
        assert!(on_circle >= 8, "z = {z}: {on_circle} edges");
    }
}

/// testdata/platte_geteilt.brep: a plate 40 x 20 with two discs of radius 4 split out of it,
/// as the Split of a CAD program leaves them, and beside it a plain plate 20 x 20. Made with
/// Gmsh's Python API: `addRectangle`, two `addDisk`, `fragment` of the plate with the discs,
/// one more `addRectangle` at x = 50, written as BREP, which keeps the shared edges.
#[test]
fn faces_sharing_edges_are_one_shell_part() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = import_cad(&testdata("platte_geteilt.brep"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    // The split plate and its discs are one part, the plate beside it another.
    assert_eq!(part_names(&geometry).unwrap(), ["SHELL-1", "SHELL-2"]);
    let display = tessellate(&geometry).unwrap();
    assert_eq!(display.part_dimensions, [2, 2]);
    let faces_of = |part: &Part| -> BTreeSet<i32> {
        (part.elements.iter())
            .filter_map(|&e| match display.entity(e) {
                Some(CadEntity::Face(face)) => Some(face),
                _ => None,
            })
            .collect()
    };
    assert_eq!(faces_of(&display.mesh.parts[0]).len(), 3);
    assert_eq!(faces_of(&display.mesh.parts[1]).len(), 1);

    geometry.meshing.max_size = 2.0;
    geometry.meshing.second_order = false;
    let mesh = generate_mesh(&geometry).unwrap().mesh;
    let names: Vec<&str> = mesh.parts.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, ["SHELL-1", "SHELL-2"]);
    // Every face keeps its own elements to pick for loads and supports.
    assert_eq!(mesh.cad.faces.len(), 4);
    let split: Vec<i32> = faces_of(&display.mesh.parts[0]).into_iter().collect();
    let area = |face: i32| -> f64 {
        (mesh.cad.faces[&face].iter())
            .map(|&(e, _)| signed_area(&mesh, mesh.element(e).unwrap()).abs())
            .sum()
    };
    let disc = std::f64::consts::PI * 16.0;
    let mut areas: Vec<f64> = split.iter().map(|&f| area(f)).collect();
    areas.sort_by(f64::total_cmp);
    // Straight element sides cut the circles a little short.
    for (got, exact) in areas.iter().zip([disc, disc, 800.0 - 2.0 * disc]) {
        assert!(
            (got - exact).abs() / exact < 0.05,
            "Area {got}, not {exact}"
        );
    }
    // The mesh is conformal: the plate and each disc share the nodes on the circle between
    // them, and no other node lies there twice.
    let nodes = |face: i32| -> BTreeSet<NodeId> {
        mesh.cad_nodes(&[CadEntity::Face(face)])
            .into_iter()
            .collect()
    };
    let plate = *split
        .iter()
        .max_by(|&&a, &&b| area(a).total_cmp(&area(b)))
        .unwrap();
    for &face in split.iter().filter(|&&f| f != plate) {
        let shared = nodes(plate).intersection(&nodes(face)).count();
        assert!(
            shared >= 12,
            "Disc {face} shares {shared} nodes with the plate"
        );
    }
    let coords = mesh.coords();
    let mut places: Vec<[i64; 2]> = (coords.iter())
        .map(|c| [(c[0] * 1e6).round() as i64, (c[1] * 1e6).round() as i64])
        .collect();
    let count = places.len();
    places.sort_unstable();
    places.dedup();
    assert_eq!(places.len(), count, "nodes lie twice");

    let smaller = delete_part(&geometry, "SHELL-1").unwrap().unwrap();
    assert_eq!(part_names(&smaller).unwrap(), ["SHELL-2"]);
}

/// testdata/benannte_teile.step: a STEP file as CAD programs such as Onshape write it, an
/// assembly "Part Studio 1" with the solids "oben" and "Stütze" (written `\X2\00FC\X0\`) and
/// the surface part "Membran". Written by Gmsh, the names set in the text afterwards.
#[test]
fn parts_keep_the_names_of_the_cad_file() {
    if !gmsh_available() {
        return;
    }
    let import = import_cad(&testdata("benannte_teile.step"), UnitSystem::Unitless).unwrap();
    let names = ["OBEN", "STUETZE", "MEMBRAN"];
    assert_eq!(part_names(&import.geometry).unwrap(), names);
    let shown: Vec<&str> = (import.display.mesh.parts.iter())
        .map(|p| p.name.as_str())
        .collect();
    assert_eq!(shown, names);
    // The names belong to the right bodies: "oben" lies above, the membrane in between.
    let lowest = |name: &str| {
        let part = import
            .display
            .mesh
            .parts
            .iter()
            .find(|p| p.name == name)
            .unwrap();
        (part.elements.iter())
            .flat_map(|&id| import.display.mesh.element(id).unwrap().nodes.clone())
            .map(|n| import.display.mesh.node(n).unwrap()[2])
            .fold(f64::INFINITY, f64::min)
    };
    assert!(lowest("OBEN") > 9.0);
    assert!(lowest("STUETZE") < 1.0);
    assert!((lowest("MEMBRAN") - 7.5).abs() < 1e-6);

    let mut geometry = import.geometry;
    geometry.meshing.second_order = false;
    geometry.meshing.max_size = 5.0;
    let mesh = generate_mesh(&geometry).unwrap().mesh;
    let meshed: Vec<&str> = mesh.parts.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(meshed, names);

    // Added to another geometry, the parts keep their names unless one is taken.
    let blocks = import_cad(&testdata("zwei_bloecke.step"), UnitSystem::Unitless).unwrap();
    let paths = [testdata("benannte_teile.step")];
    let taken = ["OBEN".to_string()];
    let addition =
        add_cad_files(Some(&blocks.geometry), &paths, UnitSystem::Unitless, &taken).unwrap();
    assert_eq!(addition.added, ["OBEN-2", "STUETZE", "MEMBRAN"]);
    let addition = add_cad_files(None, &paths, UnitSystem::Unitless, &taken).unwrap();
    assert_eq!(addition.added, ["OBEN-2", "STUETZE", "MEMBRAN"]);
    let shown: Vec<&str> = (addition.import.display.mesh.parts.iter())
        .map(|p| p.name.as_str())
        .collect();
    assert_eq!(shown, ["OBEN-2", "STUETZE", "MEMBRAN"]);
}

#[test]
fn part_names_are_calculix_names() {
    assert_eq!(
        label_name("Shapes/Part Studio 1/2/Stütze").as_deref(),
        Some("STUETZE")
    );
    assert_eq!(
        label_name(
            "Shapes/Open CASCADE STEP translator 7.8 1/1/Open CASCADE STEP translator 7.8 1.1"
        ),
        None
    );
    assert!(is_default_name("SOLID-12"));
    assert!(!is_default_name("SOLID-A"));
    assert!(!is_default_name("OBEN"));
}

fn extrude_item(faces: Vec<i32>, layers: plx_model::ExtrudeLayers) -> MeshSetupItem {
    MeshSetupItem {
        name: "Extrude_Mesh-1".into(),
        kind: MeshSetupKind::ExtrudeMesh(plx_model::ExtrudeMesh {
            parts: vec!["SOLID-1".into()],
            faces,
            layers,
            ..Default::default()
        }),
    }
}

/// Checks that the outer element faces are the faces the CAD map records, with their nodes.
fn check_cad_map(mesh: &FeMesh, display: &GeometryDisplay) {
    let cad = &mesh.cad;
    assert_eq!(cad.faces.len(), display.faces);
    assert_eq!(cad.segments.len(), display.edges);
    let mut count: HashMap<Vec<NodeId>, usize> = HashMap::new();
    for element in mesh.elements() {
        for face in element.faces() {
            let mut corners: Vec<NodeId> = face.corners.iter().map(|&i| element.nodes[i]).collect();
            corners.sort_unstable();
            *count.entry(corners).or_default() += 1;
        }
    }
    let outer = count.values().filter(|&&n| n == 1).count();
    let on_cad: usize = cad.faces.values().map(Vec::len).sum();
    assert_eq!(on_cad, outer);
    for (&tag, faces) in &cad.faces {
        let nodes: BTreeSet<NodeId> = (faces.iter())
            .flat_map(|&(e, f)| {
                let element = mesh.element(e).unwrap();
                let face = &element.faces()[usize::from(f) - 1];
                let mids = if element.shape.is_quadratic() {
                    face.mids
                } else {
                    &[]
                };
                let local = face.corners.iter().chain(mids);
                local.map(|&i| element.nodes[i]).collect::<Vec<_>>()
            })
            .collect();
        let recorded: BTreeSet<NodeId> = cad.nodes[&CadEntity::Face(tag)].iter().copied().collect();
        assert_eq!(nodes, recorded, "Face {tag}");
    }
    for (&tag, segments) in &cad.segments {
        assert!(!segments.is_empty(), "Edge {tag}");
        // An open edge has a node more than segments, a closed one as many.
        assert!(
            cad.nodes[&CadEntity::Edge(tag)].len() >= segments.len(),
            "Edge {tag}"
        );
    }
    assert!(cad.nodes.keys().any(|e| matches!(e, CadEntity::Vertex(_))));
}

#[test]
fn extruded_parts_are_meshed_with_hexahedra_in_layers() {
    if !gmsh_available() {
        return;
    }
    let import = import_cad(&testdata("platte_mit_loch.step"), UnitSystem::Unitless).unwrap();
    let mut geometry = import.geometry;
    geometry.meshing.max_size = 4.0;
    for second_order in [false, true] {
        geometry.meshing.second_order = second_order;
        // The source is found: the face with the hole, swept through the 10 mm.
        geometry.mesh_items = vec![extrude_item(
            vec![],
            plx_model::ExtrudeLayers::NumberOfElements(3),
        )];
        let mesh = generate_part_mesh(&geometry, "SOLID-1").unwrap().mesh;
        let (hex, wedge) = if second_order {
            (ElementShape::Hex20, ElementShape::Wedge15)
        } else {
            (ElementShape::Hex8, ElementShape::Wedge6)
        };
        assert!(
            mesh.elements()
                .iter()
                .all(|e| e.shape == hex || e.shape == wedge)
        );
        let hexes = mesh.elements().iter().filter(|e| e.shape == hex).count();
        assert!(
            hexes * 2 > mesh.element_count(),
            "mostly hexahedra: {hexes}"
        );
        assert_eq!(mesh.element_count() % 3, 0);
        for element in mesh.elements() {
            let jacobian = element.min_jacobian(&mesh).unwrap();
            assert!(jacobian > 0.0, "{} {jacobian}", element.id);
        }
        // Three layers through the thickness of 10.
        let mut heights: Vec<f64> = mesh.coords().iter().map(|c| c[2]).collect();
        heights.sort_by(f64::total_cmp);
        heights.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
        assert_eq!(
            heights.len(),
            if second_order { 7 } else { 4 },
            "{heights:?}"
        );
        check_cad_map(&mesh, &import.display);
    }

    // Full-quad recombination gives hexahedra only; the layers follow the element size.
    geometry.meshing.second_order = false;
    let mut item = extrude_item(vec![], plx_model::ExtrudeLayers::ScaleFactor(0.5));
    if let MeshSetupKind::ExtrudeMesh(extrude) = &mut item.kind {
        extrude.recombination = plx_model::Recombination::BlossomFullQuad;
    }
    geometry.mesh_items = vec![item];
    let mesh = generate_part_mesh(&geometry, "SOLID-1").unwrap().mesh;
    assert!(
        mesh.elements()
            .iter()
            .all(|e| e.shape == ElementShape::Hex8)
    );
    let mut heights: Vec<f64> = mesh.coords().iter().map(|c| c[2]).collect();
    heights.sort_by(f64::total_cmp);
    heights.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    assert_eq!(heights.len(), 6, "10 / (0.5 * 4) = 5 layers");
}

#[test]
fn source_faces_are_checked_to_be_an_end_of_an_extrusion() {
    if !gmsh_available() {
        return;
    }
    let import = import_cad(&testdata("platte_mit_loch.step"), UnitSystem::Unitless).unwrap();
    let mut geometry = import.geometry;
    geometry.meshing.max_size = 5.0;
    let display = &import.display;
    // The faces of the plate by their normal: top and bottom lie in z = const.
    let flat_in = |axis: usize| -> Vec<i32> {
        let mut faces = BTreeSet::new();
        for (element, entity) in display.mesh.elements().iter().zip(&display.entities) {
            let CadEntity::Face(tag) = entity else {
                continue;
            };
            let p: Vec<[f64; 3]> = (element.nodes.iter())
                .map(|&n| display.mesh.node(n).unwrap())
                .collect();
            if p.iter().all(|q| (q[axis] - p[0][axis]).abs() < 1e-9) {
                faces.insert(*tag);
            }
        }
        faces.into_iter().collect()
    };
    let ends = flat_in(2);
    let (top, side) = (ends[0], flat_in(1)[0]);
    geometry.mesh_items = vec![extrude_item(
        vec![top],
        plx_model::ExtrudeLayers::NumberOfElements(2),
    )];
    let mesh = generate_part_mesh(&geometry, "SOLID-1").unwrap().mesh;
    assert!(
        mesh.elements()
            .iter()
            .all(|e| matches!(e.shape, ElementShape::Hex20 | ElementShape::Wedge15))
    );
    // A long side face: the hole goes across the extrusion.
    geometry.mesh_items = vec![extrude_item(
        vec![side],
        plx_model::ExtrudeLayers::NumberOfElements(2),
    )];
    let error = generate_part_mesh(&geometry, "SOLID-1")
        .unwrap_err()
        .to_string();
    assert!(error.contains("not an extrusion"), "{error}");
    // A later Tetrahedral Gmsh item for the part wins.
    geometry.mesh_items = vec![
        extrude_item(vec![top], plx_model::ExtrudeLayers::NumberOfElements(2)),
        MeshSetupItem {
            name: "Tetrahedral_Gmsh-1".into(),
            kind: MeshSetupKind::TetrahedralGmsh {
                parts: vec!["SOLID-1".into()],
                algorithm_2d: Algorithm2d::default(),
                algorithm_3d: Algorithm3d::default(),
            },
        },
    ];
    let mesh = generate_part_mesh(&geometry, "SOLID-1").unwrap().mesh;
    assert!(
        mesh.elements()
            .iter()
            .all(|e| e.shape == ElementShape::Tet10)
    );
}

#[test]
fn meshing_parameters_find_extrusions_and_keep_tetrahedra_for_the_rest() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = import_cad(&testdata("platte_mit_loch.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    geometry.meshing.max_size = 5.0;
    geometry.meshing.hexahedra_where_possible = true;
    let mesh = generate_part_mesh(&geometry, "SOLID-1").unwrap().mesh;
    assert!(
        (mesh.elements().iter())
            .all(|e| matches!(e.shape, ElementShape::Hex20 | ElementShape::Wedge15))
    );
    // A Tetrahedral Gmsh item for the part keeps it tetrahedral.
    geometry.mesh_items = vec![MeshSetupItem {
        name: "Tetrahedral_Gmsh-1".into(),
        kind: MeshSetupKind::TetrahedralGmsh {
            parts: vec!["SOLID-1".into()],
            algorithm_2d: Algorithm2d::default(),
            algorithm_3d: Algorithm3d::default(),
        },
    }];
    let mesh = generate_part_mesh(&geometry, "SOLID-1").unwrap().mesh;
    assert!(
        mesh.elements()
            .iter()
            .all(|e| e.shape == ElementShape::Tet10)
    );

    // A shaft with a split face is no extrusion of one face, and no body of revolution
    // without elements collapsing on its axis: it falls back to tetrahedra.
    let mut shaft = import_cad(
        &testdata("welle_mit_teilflaeche.step"),
        UnitSystem::Unitless,
    )
    .unwrap()
    .geometry;
    shaft.meshing.hexahedra_where_possible = true;
    let mesh = generate_part_mesh(&shaft, "SOLID-1").unwrap().mesh;
    assert!(
        mesh.elements()
            .iter()
            .all(|e| e.shape == ElementShape::Tet10)
    );
}

/// A rectangle in the x-y plane, with corners rounded by `radius`, turned about the y axis.
fn revolved(origin: [f64; 3], size: [f64; 2], radius: f64, angle: f64) -> Geometry {
    let file = TempFile::new("brep");
    with_gmsh(|gmsh| {
        let face = gmsh.add_rectangle(origin, size, radius)?;
        gmsh.revolve(&[(2, face)], [0.0; 3], [0.0, 1.0, 0.0], angle)?;
        if angle >= std::f64::consts::TAU {
            // The rectangle is left inside the closed ring.
            gmsh.remove(&[(2, face)])?;
        }
        gmsh.write(&file.0)
    })
    .unwrap();
    Geometry {
        source: "ring.brep".into(),
        brep: std::fs::read_to_string(&file.0).unwrap(),
        meshing: MeshingParameters::for_diagonal(size[0].hypot(size[1])),
        mesh_items: Vec::new(),
        part_names: Vec::new(),
        refinement: 1.0,
        signatures: Vec::new(),
    }
}

#[test]
fn bodies_of_revolution_are_meshed_with_hexahedra_turned_about_their_axis() {
    if !gmsh_available() {
        return;
    }
    use std::f64::consts::{FRAC_PI_2, TAU};
    // The sharp rings are extrusions as well; the rounded ones only bodies of revolution.
    for (angle, radius) in [(TAU, 0.0), (FRAC_PI_2, 0.0), (FRAC_PI_2, 2.0), (TAU, 2.0)] {
        let mut geometry = revolved([10.0, 0.0, 0.0], [10.0, 30.0], radius, angle);
        assert_eq!(
            part_names(&geometry).unwrap(),
            ["SOLID-1"],
            "{angle} {radius}"
        );
        let display = tessellate(&geometry).unwrap();
        geometry.meshing.max_size = 3.0;
        geometry.meshing.hexahedra_where_possible = true;
        for second_order in [false, true] {
            geometry.meshing.second_order = second_order;
            let mesh = generate_part_mesh(&geometry, "SOLID-1").unwrap().mesh;
            let (hex, wedge) = if second_order {
                (ElementShape::Hex20, ElementShape::Wedge15)
            } else {
                (ElementShape::Hex8, ElementShape::Wedge6)
            };
            assert!(
                (mesh.elements().iter()).all(|e| e.shape == hex || e.shape == wedge),
                "{angle} {radius}"
            );
            let hexes = mesh.elements().iter().filter(|e| e.shape == hex).count();
            assert!(
                hexes * 2 > mesh.element_count(),
                "mostly hexahedra: {hexes}"
            );
            for element in mesh.elements() {
                let jacobian = element.min_jacobian(&mesh).unwrap();
                assert!(jacobian > 0.0, "{angle} {radius} {} {jacobian}", element.id);
            }
            // Every node lies in the ring; midside nodes of an extrusion's circles may sit
            // on their chords.
            for c in mesh.coords() {
                let r = c[0].hypot(c[2]);
                assert!((9.5..=20.0 + 1e-6).contains(&r), "{angle} {radius} {c:?}");
            }
            check_cad_map(&mesh, &display);
        }
    }
}

/// A STEP file of one rounded rectangle 40 x 20 mm at the height `z`, tilted by `tilt`
/// radians about the x axis.
fn flat_step_file(z: f64, tilt: f64) -> TempFile {
    let file = TempFile::new("step");
    with_gmsh(|gmsh| {
        let face = gmsh.add_rectangle([0.0, 0.0, z], [40.0, 20.0], 3.0)?;
        let (c, s) = (tilt.cos(), tilt.sin());
        #[rustfmt::skip]
        let matrix = [
            1.0, 0.0, 0.0, 0.0,
            0.0, c, -s, s * z,
            0.0, s, c, z - c * z,
        ];
        gmsh.affine_transform(&[(2, face)], &matrix)?;
        gmsh.write(&file.0)
    })
    .unwrap();
    file
}

fn z_range(display: &GeometryDisplay) -> (f64, f64) {
    let (min, max) = display.mesh.bounds().unwrap();
    (min[2], max[2])
}

#[test]
fn flat_geometry_is_moved_into_the_xy_plane() {
    if !gmsh_available() {
        return;
    }
    let file = flat_step_file(5.0, 0.0);
    let import = import_cad(&file.0, UnitSystem::Unitless).unwrap();
    let (low, high) = z_range(&import.display);
    assert!(low.abs() < 1e-9 && high.abs() < 1e-9, "{low} {high}");
    assert!((width(&import.display) - 40.0).abs() < 1e-6);
    // The face is moved with its edges, not copied apart from them.
    let level = import_cad(&flat_step_file(0.0, 0.0).0, UnitSystem::Unitless).unwrap();
    assert_eq!(import.display.faces, 1);
    assert_eq!(import.display.edges, level.display.edges);
    assert_eq!(import.notes.len(), 1, "{:?}", import.notes);
    assert!(import.notes[0].contains("z = 5;"), "{:?}", import.notes);
    // In a unit system, the geometry is stored in metres and the note names the unit.
    let import = import_cad(&file.0, UnitSystem::MmTonSC).unwrap();
    let (low, high) = z_range(&import.display);
    assert!(low.abs() < 1e-12 && high.abs() < 1e-12, "{low} {high}");
    assert!(import.notes[0].contains("z = 5 mm"), "{:?}", import.notes);
    // Below the plane, it is moved up.
    let below = import_cad(&flat_step_file(-12.5, 0.0).0, UnitSystem::Unitless).unwrap();
    assert!(z_range(&below.display).0.abs() < 1e-9);
    assert!(below.notes[0].contains("z = -12.5"), "{:?}", below.notes);
}

#[test]
fn geometry_off_a_plane_z_const_stays_put() {
    if !gmsh_available() {
        return;
    }
    // Already in the x-y plane.
    let level = import_cad(&flat_step_file(0.0, 0.0).0, UnitSystem::Unitless).unwrap();
    assert!(level.notes.is_empty(), "{:?}", level.notes);
    // A tilted face is flat, but not parallel to the x-y plane.
    let tilted = import_cad(&flat_step_file(5.0, 0.5).0, UnitSystem::Unitless).unwrap();
    assert!(tilted.notes.is_empty(), "{:?}", tilted.notes);
    let (low, high) = z_range(&tilted.display);
    assert!((low - 5.0).abs() < 1e-6 && (high - 5.0 - 20.0 * 0.5f64.sin()).abs() < 1e-6);
    // Solids keep their place.
    let blocks = import_cad(&testdata("zwei_bloecke.step"), UnitSystem::Unitless).unwrap();
    assert!(blocks.notes.is_empty(), "{:?}", blocks.notes);
}

#[test]
fn flat_files_added_to_a_geometry_are_moved_into_the_xy_plane() {
    if !gmsh_available() {
        return;
    }
    let base = rectangle([100.0, 0.0, 0.0], [30.0, 10.0], 0.0);
    let file = flat_step_file(8.0, 0.0);
    let addition = add_cad_files(
        Some(&base),
        std::slice::from_ref(&file.0),
        UnitSystem::Unitless,
        &[],
    )
    .unwrap();
    assert_eq!(addition.added.len(), 1);
    assert_eq!(
        addition.import.notes.len(),
        1,
        "{:?}",
        addition.import.notes
    );
    let (low, high) = z_range(&addition.import.display);
    assert!(low.abs() < 1e-9 && high.abs() < 1e-9, "{low} {high}");
}

/// A plate of the size beside a small cube, which Gmsh numbers before the plate if it comes
/// first, so that the plate's faces and edges get other tags.
fn plate_and_cube(size: [f64; 3], cube_first: bool) -> Geometry {
    let file = TempFile::new("brep");
    with_gmsh(|gmsh| {
        let cube = || gmsh.add_box([-20.0, 0.0, 0.0], [5.0; 3]);
        if cube_first {
            cube()?;
        }
        gmsh.add_box([0.0; 3], size)?;
        if !cube_first {
            cube()?;
        }
        gmsh.write(&file.0)
    })
    .unwrap();
    with_signatures(Geometry {
        source: "platte.brep".into(),
        brep: std::fs::read_to_string(&file.0).unwrap(),
        meshing: MeshingParameters {
            max_size: 2.0,
            ..MeshingParameters::default()
        },
        mesh_items: Vec::new(),
        part_names: Vec::new(),
        refinement: 1.0,
        signatures: Vec::new(),
    })
    .unwrap()
}

#[test]
fn picked_entities_are_found_in_changed_and_renumbered_shapes() {
    if !gmsh_available() {
        return;
    }
    let old = plate_and_cube([10.0, 10.0, 2.0], false);
    let on_plate = |s: &&plx_model::EntitySignature| s.centroid[0] > 0.0;
    let top = |geometry: &Geometry, z: f64| {
        (geometry.signatures.iter())
            .filter(on_plate)
            .find(|s| {
                s.entity.dim() == 2
                    && (s.centroid[2] - z).abs() < 1e-6
                    && s.kind == "Plane"
                    && s.max[2] - s.min[2] < 1e-3
            })
            .map(|s| s.entity)
            .unwrap()
    };
    let old_top = top(&old, 2.0);
    // The same shapes read again keep their tags.
    let again = plate_and_cube([10.0, 10.0, 2.0], false);
    assert_eq!(again.signatures, old.signatures);
    // A thicker plate, numbered after the cube: every face of both is found, the top face
    // on the new top.
    let mut new = plate_and_cube([10.0, 10.0, 3.0], true);
    let tags = old.tags_in(&new).unwrap();
    let faces = |g: &Geometry| g.signatures.iter().filter(|s| s.entity.dim() == 2).count();
    assert_eq!(faces(&old), 12);
    assert_eq!(tags.keys().filter(|e| e.dim() == 2).count(), 12);
    let new_top = top(&new, 3.0);
    assert_ne!(new_top, old_top, "the plate is numbered after the cube now");
    assert_eq!(tags[&old_top], new_top);
    // Its edges too: the twelve of the plate and those of the cube.
    assert_eq!(tags.keys().filter(|e| e.dim() == 1).count(), 24);
    // A local mesh size on the top face follows it, and the new mesh has it.
    new.mesh_items = vec![MeshSetupItem {
        name: "Local Mesh Size-1".into(),
        kind: MeshSetupKind::LocalMeshSize {
            faces: vec![old_top.tag()],
            edges: Vec::new(),
            size: 0.5,
        },
    }];
    assert_eq!(new.renumber_mesh_items(&tags), 0);
    let MeshSetupKind::LocalMeshSize { faces, .. } = &new.mesh_items[0].kind else {
        unreachable!()
    };
    assert_eq!(faces, &[new_top.tag()]);
    let mesh = generate_mesh(&new).unwrap().mesh;
    let nodes = mesh.cad_nodes(&[tags[&old_top]]);
    assert!(nodes.len() > 20);
    assert!(
        nodes
            .iter()
            .all(|&n| (mesh.node(n).unwrap()[2] - 3.0).abs() < 1e-9)
    );
}

/// A box of 10 mm, its signatures recorded.
fn cube() -> Geometry {
    let file = TempFile::new("brep");
    with_gmsh(|gmsh| {
        gmsh.add_box([0.0; 3], [10.0; 3])?;
        gmsh.write(&file.0)
    })
    .unwrap();
    with_signatures(Geometry {
        source: "wuerfel.brep".into(),
        brep: std::fs::read_to_string(&file.0).unwrap(),
        meshing: MeshingParameters {
            max_size: 2.0,
            ..MeshingParameters::default()
        },
        mesh_items: Vec::new(),
        part_names: Vec::new(),
        refinement: 1.0,
        signatures: Vec::new(),
    })
    .unwrap()
}

/// Volume of the geometry's solids.
fn solid_volume(geometry: &Geometry) -> f64 {
    let file = TempFile::with_contents("brep", &geometry.brep).unwrap();
    with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        let mut volume = 0.0;
        for tag in gmsh.entities(3)? {
            volume += gmsh.mass(3, tag)?;
        }
        Ok(volume)
    })
    .unwrap()
}

#[test]
fn fillets_and_chamfers_change_the_solid_and_keep_its_faces_findable() {
    use plx_model::CadOperationKind::{Chamfer, Fillet};
    if !gmsh_available() {
        return;
    }
    let input = cube();
    // The edge along z at x = y = 10.
    let edge = (input.signatures.iter())
        .find(|s| {
            s.entity.dim() == 1
                && (s.centroid[0] - 10.0).abs() < 1e-6
                && (s.centroid[1] - 10.0).abs() < 1e-6
        })
        .unwrap()
        .entity;
    let rounded = apply_cad_operation(&input, Fillet, &[edge], 2.0).unwrap();
    let removed = (4.0 - std::f64::consts::PI) * 10.0;
    assert!((solid_volume(&rounded) - (1000.0 - removed)).abs() < 1e-3);
    assert_eq!(part_names(&rounded).unwrap(), part_names(&input).unwrap());
    assert!(rounded.signatures.iter().any(|s| s.kind == "Cylinder"));
    let bevelled = apply_cad_operation(&input, Chamfer, &[edge], 2.0).unwrap();
    assert!((solid_volume(&bevelled) - 980.0).abs() < 1e-3);
    // A larger radius: every face of the first result is found in the second, the bottom
    // face where it was, and the new mesh has it.
    let larger = apply_cad_operation(&input, Fillet, &[edge], 3.0).unwrap();
    let tags = rounded.tags_in(&larger).unwrap();
    let faces = rounded.signatures.iter().filter(|s| s.entity.dim() == 2);
    assert_eq!(faces.clone().count(), 7);
    assert!(faces.clone().all(|s| tags.contains_key(&s.entity)));
    let bottom = faces
        .clone()
        .find(|s| s.kind == "Plane" && s.centroid[2].abs() < 1e-6)
        .unwrap()
        .entity;
    let mesh = generate_mesh(&larger).unwrap().mesh;
    let nodes = mesh.cad_nodes(&[tags[&bottom]]);
    assert!(nodes.len() > 10);
    assert!(nodes.iter().all(|&n| mesh.node(n).unwrap()[2].abs() < 1e-9));
    // Too large a radius fails rather than giving a broken solid; so do picks that are no
    // edges of the geometry.
    assert!(apply_cad_operation(&input, Fillet, &[edge], 20.0).is_err());
    assert!(apply_cad_operation(&input, Fillet, &[CadEntity::Edge(99)], 1.0).is_err());
    assert!(apply_cad_operation(&input, Fillet, &[edge], -1.0).is_err());
    // A radius of zero leaves the solid as it is, so that a study can switch it off.
    for kind in [Fillet, Chamfer] {
        let same = apply_cad_operation(&input, kind, &[edge], 0.0).unwrap();
        assert_eq!(
            (same.brep, same.signatures),
            (input.brep.clone(), input.signatures.clone())
        );
    }
}

#[test]
fn push_pull_moves_planar_faces_along_their_normal() {
    use plx_model::CadOperationKind::{Fillet, PushPull};
    if !gmsh_available() {
        return;
    }
    let input = cube();
    let face_at = |geometry: &Geometry, axis: usize, at: f64| {
        (geometry.signatures.iter())
            .find(|s| {
                s.entity.dim() == 2 && s.kind == "Plane" && (s.centroid[axis] - at).abs() < 1e-6
            })
            .unwrap()
            .entity
    };
    let top = face_at(&input, 2, 10.0);
    // Pulled out: the cube grows to 15 high, and its sides stay one face each.
    let pulled = apply_cad_operation(&input, PushPull, &[top], 5.0).unwrap();
    assert!((solid_volume(&pulled) - 1500.0).abs() < 1e-3);
    assert_eq!(part_names(&pulled).unwrap(), part_names(&input).unwrap());
    let faces = pulled.signatures.iter().filter(|s| s.entity.dim() == 2);
    assert_eq!(faces.count(), 6);
    face_at(&pulled, 2, 15.0);
    // Pushed in, and on a side face: the normal points out of the solid either way.
    let pushed = apply_cad_operation(&input, PushPull, &[top], -3.0).unwrap();
    assert!((solid_volume(&pushed) - 700.0).abs() < 1e-3);
    let side = face_at(&input, 0, 0.0);
    let wider = apply_cad_operation(&input, PushPull, &[side, top], 2.0).unwrap();
    assert!((solid_volume(&wider) - 1440.0).abs() < 1e-3);
    // Through the whole solid, a curved face, nothing to move: it fails.
    assert!(apply_cad_operation(&input, PushPull, &[top], -10.0).is_err());
    let same = apply_cad_operation(&input, PushPull, &[top], 0.0).unwrap();
    assert_eq!(same.brep, input.brep);
    let edge = (input.signatures.iter())
        .find(|s| s.entity.dim() == 1)
        .unwrap()
        .entity;
    let rounded = apply_cad_operation(&input, Fillet, &[edge], 2.0).unwrap();
    let curved = (rounded.signatures.iter())
        .find(|s| s.kind == "Cylinder")
        .unwrap()
        .entity;
    let error = apply_cad_operation(&rounded, PushPull, &[curved], 1.0).unwrap_err();
    assert!(error.to_string().contains("planar"), "{error}");
}

#[test]
fn corners_of_flat_faces_are_rounded_and_bevelled() {
    use plx_model::CadOperationKind::{Chamfer, Fillet};
    if !gmsh_available() {
        return;
    }
    // Two squares side by side, sharing the edge at x = 10.
    let file = TempFile::new("brep");
    with_gmsh(|gmsh| {
        gmsh.add_rectangle([0.0; 3], [10.0; 2], 0.0)?;
        gmsh.add_rectangle([10.0, 0.0, 0.0], [10.0; 2], 0.0)?;
        gmsh.remove_all_duplicates()?;
        gmsh.write(&file.0)
    })
    .unwrap();
    let input = with_signatures(Geometry {
        source: "quadrate.brep".into(),
        brep: std::fs::read_to_string(&file.0).unwrap(),
        meshing: MeshingParameters {
            max_size: 2.0,
            ..MeshingParameters::default()
        },
        mesh_items: Vec::new(),
        part_names: Vec::new(),
        refinement: 1.0,
        signatures: Vec::new(),
    })
    .unwrap();
    let vertex = |geometry: &Geometry, x: f64, y: f64| {
        (geometry.signatures.iter())
            .find(|s| {
                s.entity.dim() == 0
                    && (s.centroid[0] - x).abs() < 1e-6
                    && (s.centroid[1] - y).abs() < 1e-6
            })
            .unwrap()
            .entity
    };
    // Area and how many faces each edge bounds.
    let shape = |geometry: &Geometry| {
        let file = TempFile::with_contents("brep", &geometry.brep).unwrap();
        with_gmsh(|gmsh| {
            gmsh.import_shapes(&file.0)?;
            let mut area = 0.0;
            for face in gmsh.entities(2)? {
                area += gmsh.mass(2, face)?;
            }
            let mut shared = 0;
            for edge in gmsh.entities(1)? {
                shared += usize::from(gmsh.adjacencies(1, edge)?.0.len() == 2);
            }
            Ok((gmsh.entities(2)?.len(), area, shared))
        })
        .unwrap()
    };
    let rounded = apply_cad_operation(&input, Fillet, &[vertex(&input, 0.0, 0.0)], 2.0).unwrap();
    let (faces, area, shared) = shape(&rounded);
    assert_eq!((faces, shared), (2, 1));
    assert!(
        (area - (200.0 - (4.0 - std::f64::consts::PI))).abs() < 1e-6,
        "{area}"
    );
    assert_eq!(part_names(&rounded).unwrap(), part_names(&input).unwrap());
    let kinds: Vec<&str> = rounded.signatures.iter().map(|s| s.kind.as_str()).collect();
    assert!(kinds.contains(&"TrimmedCurve"), "{kinds:?}");
    // Two corners of the other square, bevelled.
    let corners = [vertex(&input, 20.0, 10.0), vertex(&input, 20.0, 0.0)];
    let bevelled = apply_cad_operation(&input, Chamfer, &corners, 3.0).unwrap();
    let (faces, area, shared) = shape(&bevelled);
    assert_eq!((faces, shared), (2, 1));
    assert!((area - 191.0).abs() < 1e-6, "{area}");
    // A corner of both squares, and a radius larger than the square: they fail.
    let error = apply_cad_operation(&input, Fillet, &[vertex(&input, 10.0, 0.0)], 1.0)
        .unwrap_err()
        .to_string();
    assert!(error.contains("share"), "{error}");
    assert!(apply_cad_operation(&input, Fillet, &[vertex(&input, 0.0, 0.0)], 20.0).is_err());
    let same = apply_cad_operation(&input, Chamfer, &[vertex(&input, 0.0, 0.0)], 0.0).unwrap();
    assert_eq!(same.brep, input.brep);
}

#[test]
fn quad_dominated_meshing_leaves_solids_whole() {
    if !gmsh_available() {
        return;
    }
    let mut geometry = import_cad(&testdata("zwei_bloecke.step"), UnitSystem::Unitless)
        .unwrap()
        .geometry;
    geometry.meshing.second_order = false;
    geometry.meshing.max_size = 2.0;
    let total = |mesh: &FeMesh| -> f64 { mesh.elements().iter().map(|e| volume(mesh, e)).sum() };
    let triangles = generate_mesh(&geometry).unwrap().mesh;
    geometry.meshing.quad_dominated = true;
    let quads = generate_mesh(&geometry).unwrap().mesh;
    assert!(quads.elements().iter().all(|e| e.type_name == "C3D4"));
    let (a, b) = (total(&triangles), total(&quads));
    assert!((a - b).abs() / a < 1e-9, "Volume {b} instead of {a}");
}
