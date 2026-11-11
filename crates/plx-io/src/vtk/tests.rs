use std::path::PathBuf;

use plx_mesh::{Element, ElementShape, FeMesh, Part};
use plx_results::{AnalysisKind, Component, Field, Increment};

use super::*;

fn testdata(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name)
}

fn out_base() -> PathBuf {
    std::env::var_os("PLX_VTK_OUT").map_or_else(
        || std::env::temp_dir().join(format!("plx-vtk-{}", std::process::id())),
        PathBuf::from,
    )
}

/// A folder for the files of a test; `PLX_VTK_OUT` keeps them in a given folder, to open
/// them in ParaView.
fn out_dir(test: &str) -> PathBuf {
    let dir = out_base().join(test);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// A wedge and a quadratic beam, one part each.
fn mixed_mesh() -> FeMesh {
    let mut mesh = FeMesh::default();
    let points = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [0.0, 1.0, 1.0],
        [0.0, 0.0, 2.0],
        [0.0, 0.0, 3.0],
    ];
    for (i, p) in points.iter().enumerate() {
        mesh.set_node(i as u32 + 1, *p);
    }
    mesh.add_element(Element {
        id: 10,
        type_name: "C3D6".into(),
        shape: ElementShape::Wedge6,
        nodes: vec![1, 2, 3, 4, 5, 6],
    })
    .unwrap();
    mesh.add_element(Element {
        id: 11,
        type_name: "B32".into(),
        shape: ElementShape::Line3,
        nodes: vec![4, 7, 8],
    })
    .unwrap();
    mesh.parts.push(Part {
        bodies: Vec::new(),
        name: "SOLID".into(),
        elements: vec![10],
    });
    mesh.parts.push(Part {
        bodies: Vec::new(),
        name: "BEAM & <more>".into(),
        elements: vec![11],
    });
    mesh
}

fn increment(step: u32, number: u32, value: f64, nodes: usize) -> Increment {
    let component = |name: &str, scale: f32, derived| Component {
        name: name.into(),
        values: (0..nodes)
            .map(|n| scale * n as f32 * value as f32)
            .collect(),
        derived,
        out_of_plane: false,
    };
    let mut disp = Field::new(
        "DISP",
        vec![
            component("U1", 1.0, false),
            component("U2", 2.0, false),
            component("U3", 3.0, false),
            component("ALL", 4.0, true),
        ],
    );
    disp.components[0].values[0] = f32::NAN;
    Increment {
        nodal_diameter: None,
        step,
        increment: number,
        kind: AnalysisKind::Static,
        value,
        fields: vec![disp],
    }
}

#[test]
fn quadratic_lines_put_the_midside_node_last() {
    let cells = Cells::new(&mixed_mesh());
    assert_eq!(cells.types, [13, 21]);
    assert_eq!(cells.connectivity, [0, 1, 2, 3, 4, 5, 3, 7, 6]);
    assert_eq!(cells.offsets, [6, 9]);
    assert_eq!(cells.ids, [10, 11]);
    assert_eq!(cells.parts, [0, 1]);
}

#[test]
fn written_components_form_one_array_and_derived_ones_their_own() {
    let arrays = point_arrays(&increment(1, 1, 1.0, 8), 8);
    let names: Vec<_> = arrays.iter().map(|a| a.name.as_str()).collect();
    assert_eq!(names, ["DISP", "DISP_ALL"]);
    assert_eq!(arrays[0].components, ["U1", "U2", "U3"]);
    // Tuple after tuple: node 1 is U1, U2, U3 of node 0 and so on.
    assert_eq!(arrays[0].values[3..6], [1.0, 2.0, 3.0]);
    assert_eq!(arrays[1].values.len(), 8);
}

#[test]
fn vtu_has_mesh_fields_and_increment() {
    let mesh = mixed_mesh();
    let increment = increment(2, 3, 0.5, 8);
    let text = write_vtu(&mesh, Some(&increment), 0.5, false);
    assert!(text.contains("<Piece NumberOfPoints=\"8\" NumberOfCells=\"2\">"));
    assert!(text.contains(
        "Name=\"DISP\" NumberOfTuples=\"8\" NumberOfComponents=\"3\" ComponentName0=\"U1\""
    ));
    assert!(text.contains("Name=\"DISP_ALL\""));
    assert!(text.contains("Name=\"STEP\""));
    assert!(text.contains("Name=\"TIME\""));
    // Part names are escaped.
    assert!(!text.contains("<more>"));
    let binary = write_vtu(&mesh, Some(&increment), 0.5, true);
    assert!(binary.contains("compressor=\"vtkZLibDataCompressor\""));
    assert!(binary.contains("format=\"binary\""));
    let dir = out_dir("mixed");
    std::fs::write(dir.join("mixed_ascii.vtu"), text).unwrap();
    std::fs::write(dir.join("mixed_binary.vtu"), binary).unwrap();
    for (name, binary) in [("mixed_ascii.vtk", false), ("mixed_binary.vtk", true)] {
        let bytes = write_legacy_vtk(&mesh, Some(&increment), 0.5, binary, "Mixed");
        std::fs::write(dir.join(name), bytes).unwrap();
    }
}

#[test]
fn legacy_vtk_has_all_sections() {
    let mesh = mixed_mesh();
    let increment = increment(1, 1, 1.0, 8);
    let bytes = write_legacy_vtk(&mesh, Some(&increment), 1.0, false, "Test");
    let text = String::from_utf8(bytes).unwrap();
    for section in [
        "# vtk DataFile Version 4.2\nTest\nASCII\nDATASET UNSTRUCTURED_GRID\n",
        "POINTS 8 double",
        "CELLS 2 11",
        "CELL_TYPES 2",
        "CELL_DATA 2",
        "POINT_DATA 8\nFIELD FieldData 3\n",
        "DISP 3 8 float",
        "DISP_ALL 1 8 float",
    ] {
        assert!(text.contains(section), "{section} missing");
    }
}

#[test]
fn series_use_increasing_values_or_their_position() {
    let a = increment(1, 1, 0.5, 1);
    let b = increment(1, 2, 1.0, 1);
    let c = increment(2, 1, 0.5, 1);
    assert_eq!(series_times(&[&a, &b]), (vec![0.5, 1.0], true));
    assert_eq!(series_times(&[&a, &b, &c]), (vec![0.0, 1.0, 2.0], false));
}

#[test]
fn collection_names_drop_their_extension() {
    let pvd = Path::new("/x/Beam.pvd");
    assert_eq!(collection_stem(pvd, VtkFormat::Xml), "Beam");
    let series = Path::new("/x/Beam.vtk.series");
    assert_eq!(collection_stem(series, VtkFormat::Legacy), "Beam");
    assert_eq!(
        collection_stem(Path::new("/x/Beam"), VtkFormat::Legacy),
        "Beam"
    );
}

#[test]
fn several_increments_become_a_collection() {
    let import = crate::frd::read_frd(&testdata("kragbalken_c3d8.frd")).unwrap();
    let nodes = import.mesh.node_count();
    let first = &import.increments[0];
    let mut later = first.clone();
    later.increment += 1;
    later.value += 1.0;
    let increments = [first, &later];
    for (format, binary) in [
        (VtkFormat::Xml, true),
        (VtkFormat::Xml, false),
        (VtkFormat::Legacy, true),
        (VtkFormat::Legacy, false),
    ] {
        let dir = out_dir(&format!("{format:?}_{binary}"));
        let path = dir.join(format!("Kragbalken.{}", format.collection_extension()));
        let summary = export(
            &path,
            format,
            &import.mesh,
            &increments,
            binary,
            "Kragbalken",
        )
        .unwrap();
        assert_eq!(summary.files, 2);
        assert_eq!(summary.physical_times, Some(true));
        let collection = std::fs::read_to_string(&path).unwrap();
        let file = format!("Kragbalken/Kragbalken_0001.{}", format.extension());
        assert!(collection.contains(&file), "{collection}");
        assert!(dir.join(&file).is_file());
        // A single increment goes to the file itself.
        let single = dir.join(format!("Single.{}", format.extension()));
        export(
            &single,
            format,
            &import.mesh,
            &increments[..1],
            binary,
            "Single",
        )
        .unwrap();
        let size = std::fs::metadata(&single).unwrap().len();
        assert!(size as usize > nodes * 3, "{size} bytes");
    }
    if std::env::var_os("PLX_VTK_OUT").is_none() {
        let _ = std::fs::remove_dir_all(out_base());
    }
}

#[test]
fn json_series_and_pvd_list_files_with_times() {
    let files = [
        ("a/a_0000.vtk".to_string(), 0.0),
        ("a/a_0001.vtk".to_string(), 0.25),
    ];
    let series = write_series(&files);
    assert!(series.contains("\"file-series-version\" : \"1.0\""));
    assert!(series.contains("{ \"name\" : \"a/a_0001.vtk\", \"time\" : 0.25 }\n"));
    let pvd = write_pvd(&files);
    assert!(
        pvd.contains("<DataSet timestep=\"0.0\" group=\"\" part=\"0\" file=\"a/a_0000.vtk\"/>")
    );
}

#[test]
fn quadratic_hexahedra_are_written_whole() {
    let import = crate::frd::read_frd(&testdata("block_c3d20r.frd")).unwrap();
    let path = out_dir("hex20").join("block.vtu");
    let increments: Vec<_> = import.increments.iter().take(1).collect();
    export(
        &path,
        VtkFormat::Xml,
        &import.mesh,
        &increments,
        true,
        "Block",
    )
    .unwrap();
    let cells = Cells::new(&import.mesh);
    assert_eq!(cells.len(), import.mesh.element_count());
    assert!(cells.types.iter().all(|&t| t == 25));
}
