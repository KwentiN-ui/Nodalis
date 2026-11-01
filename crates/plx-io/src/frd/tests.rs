use super::*;

fn testdata(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name)
}

fn value(import: &FrdImport, field: &str, component: &str, node: u32) -> f32 {
    let index = import.mesh.node_index(node).unwrap();
    import.increments[0]
        .field(field)
        .unwrap()
        .component(component)
        .unwrap()
        .values[index]
}

#[test]
fn reads_ascii_mesh_and_results() {
    let import = read_frd(&testdata("kragbalken_c3d8.frd")).unwrap();
    assert!(import.warnings.is_empty(), "{:?}", import.warnings);
    assert_eq!(import.mesh.node_count(), 99);
    assert_eq!(import.mesh.element_count(), 40);
    assert_eq!(import.mesh.parts.len(), 1);
    assert_eq!(import.mesh.parts[0].name, "STEEL");
    assert_eq!(import.mesh.node(99), Some([100.0, 10.0, 10.0]));
    assert_eq!(import.date.as_deref(), Some("08.october.2026"));
    assert_eq!(import.time.as_deref(), Some("18:16:41"));

    assert_eq!(import.increments.len(), 1);
    let increment = &import.increments[0];
    assert_eq!((increment.step, increment.increment), (1, 1));
    assert_eq!(increment.kind, AnalysisKind::Static);
    assert_eq!(increment.value, 1.0);
    let names: Vec<&str> = increment.fields.iter().map(|f| f.name.as_str()).collect();
    assert_eq!(names, ["DISP", "STRESS", "ERROR"]);

    let disp: Vec<&str> = increment.fields[0]
        .components
        .iter()
        .map(|c| c.name.as_str())
        .collect();
    assert_eq!(disp, ["ALL", "U1", "U2", "U3"]);
    assert_eq!(value(&import, "DISP", "U3", 99), -1.34512e-1);
    assert_eq!(value(&import, "STRESS", "S33", 99), -1.20928e1);
    assert_eq!(value(&import, "STRESS", "S13", 99), -3.41818);
    assert!(value(&import, "STRESS", "MISES", 99) > 0.0);
    assert_eq!(increment.field("ERROR").unwrap().components[0].name, "STR");
}

#[test]
fn quadratic_hexahedra_get_input_file_node_order() {
    let import = read_frd(&testdata("block_c3d20r.frd")).unwrap();
    let inp = crate::inp::read_inp(&testdata("block_c3d20r.inp")).unwrap();
    for element in inp.mesh.elements() {
        assert_eq!(
            import.mesh.element(element.id).unwrap().nodes,
            element.nodes
        );
    }
}

/// Writes the mesh and the given nodal components as a binary frd with floats of `size` bytes.
fn binary_frd(import: &FrdImport, components: &[(&str, &str)], size: usize) -> Vec<u8> {
    let mesh = &import.mesh;
    let float = |out: &mut Vec<u8>, v: f64| {
        if size == 8 {
            out.extend(v.to_le_bytes());
        } else {
            out.extend((v as f32).to_le_bytes());
        }
    };
    let flag = if size == 8 { 3 } else { 2 };
    let mut out = b"    1C\n    1UMAT    1STEEL\n".to_vec();
    out.extend(format!("    2C{:>30}{:>37}\n", mesh.node_count(), flag).bytes());
    for (&id, xyz) in mesh.node_ids().iter().zip(mesh.coords()) {
        out.extend((id as i32).to_le_bytes());
        for &c in xyz {
            float(&mut out, c);
        }
    }
    out.extend(format!("    3C{:>30}{:>37}\n", mesh.element_count(), flag).bytes());
    for element in mesh.elements() {
        for v in [element.id as i32, 1, 0, 1] {
            out.extend(v.to_le_bytes());
        }
        for &n in &element.nodes {
            out.extend((n as i32).to_le_bytes());
        }
    }
    out.extend(b"    1PSTEP                         1           1           1\n");
    out.extend(
        format!(
            "  100CL  101 1.000000000{:>12}{:20}{:>2}{:>5}{:10}{:>2}\n",
            mesh.node_count(),
            "",
            0,
            1,
            "",
            flag
        )
        .bytes(),
    );
    out.extend(format!(" -4  DISP    {:>5}    1\n", components.len() + 1).bytes());
    for (k, (name, _)) in components.iter().enumerate() {
        let frd_name = ["D1", "D2", "D3"][k];
        let _ = name;
        out.extend(format!(" -5  {frd_name:<8}    1    2{:>5}    0\n", k + 1).bytes());
    }
    out.extend(b" -5  ALL         1    2    0    0    1ALL\n");
    let columns: Vec<&[f32]> = components
        .iter()
        .map(|(field, name)| {
            import.increments[0]
                .field(field)
                .unwrap()
                .component(name)
                .unwrap()
                .values
                .as_slice()
        })
        .collect();
    for (index, &id) in mesh.node_ids().iter().enumerate() {
        out.extend((id as i32).to_le_bytes());
        for column in &columns {
            float(&mut out, column[index] as f64);
        }
    }
    out.extend(b"9999\n");
    out
}

#[test]
fn reads_binary_files_with_single_and_double_precision() {
    let ascii = read_frd(&testdata("kragbalken_c3d8.frd")).unwrap();
    let disp = [("DISP", "U1"), ("DISP", "U2"), ("DISP", "U3")];
    for size in [4, 8] {
        let binary = read_frd_bytes(&binary_frd(&ascii, &disp, size)).unwrap();
        assert_eq!(binary.mesh.node_count(), ascii.mesh.node_count());
        assert_eq!(binary.mesh.element_count(), ascii.mesh.element_count());
        assert_eq!(binary.mesh.parts[0].name, "STEEL");
        for node in [1, 50, 99] {
            assert_eq!(
                value(&binary, "DISP", "U3", node),
                value(&ascii, "DISP", "U3", node)
            );
            assert_eq!(
                value(&binary, "DISP", "ALL", node),
                value(&ascii, "DISP", "ALL", node)
            );
        }
    }
}

#[test]
fn numbers_without_separating_blanks_are_split_by_column() {
    assert_eq!(
        fixed_floats(" -1        99-1.79459E-01-6.31509E-01", 13, 2),
        [-1.79459e-1, -6.31509e-1]
    );
    // Three-digit exponents of old Windows builds make negative numbers 13 columns wide.
    assert_eq!(
        fixed_floats(" -1         1-1.35800E+003 1.0000E+000", 13, 2),
        [-1358.0, 1.0]
    );
}

#[test]
fn truncated_binary_data_is_an_error() {
    let ascii = read_frd(&testdata("kragbalken_c3d8.frd")).unwrap();
    let mut bytes = binary_frd(&ascii, &[("DISP", "U1")], 4);
    bytes.truncate(400);
    assert!(read_frd_bytes(&bytes).is_err());
}

/// CalculiX writes a data set per field of each mode of a cyclic symmetric model, numbered
/// on through the nodal diameters, and only the first of each cosine and sine pair.
#[test]
fn cyclic_symmetric_modes_carry_their_nodal_diameter() {
    let import = read_frd(&testdata("cyclic_sector.frd")).unwrap();
    assert!(import.warnings.is_empty(), "{:?}", import.warnings);
    let modes: Vec<_> = (import.increments.iter())
        .map(|i| (i.increment, i.nodal_diameter, i.value))
        .collect();
    assert_eq!(
        modes,
        [
            (1, Some(1), 5180.798039),
            (2, Some(1), 17183.54392),
            (3, Some(2), 5340.474182),
            (4, Some(2), 27753.78035),
        ]
    );
    let names: Vec<&str> = (import.increments[0].fields.iter())
        .map(|f| f.name.as_str())
        .collect();
    assert_eq!(
        names,
        ["DISP", "DISPI", "STRESS", "STRESSI", "ERROR", "ERRORI"]
    );
}

#[test]
fn gap_sections_of_the_same_name_make_one_part() {
    // CalculiX writes a material "GAP" for each of the four `*Gap` sections.
    let import = read_frd(&testdata("gap_stuetzen.frd")).unwrap();
    assert_eq!(import.materials.values().filter(|m| *m == "GAP").count(), 4);
    let parts: Vec<(&str, usize)> = (import.mesh.parts.iter())
        .map(|p| (p.name.as_str(), p.elements.len()))
        .collect();
    assert_eq!(parts, [("STEEL", 1), ("GAP", 4)]);
}

/// Each node of a curved quadratic shell and of a beam is found again in the solids CalculiX
/// expands them to: the expanded nodes lie around it.
#[test]
fn expanded_nodes_surround_the_nodes_they_come_from() {
    // A strip of S8R bent into a quarter cylinder of radius 5, 1 thick, and a B32 beam.
    let mut text = String::from("*Node\n");
    let (nx, ny) = (4, 1);
    let id = |i: usize, j: usize| j * (2 * nx + 1) + i + 1;
    for j in 0..=2 * ny {
        for i in 0..=2 * nx {
            let angle = std::f64::consts::FRAC_PI_2 * i as f64 / (2 * nx) as f64;
            let (x, z) = (5.0 * angle.cos(), 5.0 * angle.sin());
            text.push_str(&format!("{}, {x}, {}, {z}\n", id(i, j), 2.0 * j as f64));
        }
    }
    text.push_str("100, 0, 10, 0\n101, 2, 10, 0\n102, 4, 10, 0\n");
    text.push_str("*Element, Type=S8R, Elset=Shell\n");
    for e in 0..nx {
        let (a, b) = (2 * e, 0);
        let nodes = [
            id(a, b),
            id(a + 2, b),
            id(a + 2, b + 2),
            id(a, b + 2),
            id(a + 1, b),
            id(a + 2, b + 1),
            id(a + 1, b + 2),
            id(a, b + 1),
        ];
        let list: Vec<String> = nodes.iter().map(ToString::to_string).collect();
        text.push_str(&format!("{}, {}\n", e + 1, list.join(", ")));
    }
    text.push_str(
        "*Element, Type=B32, Elset=Beam\n10, 100, 101, 102\n\
         *Material, Name=S\n*Elastic\n210000, 0.3\n\
         *Shell section, Elset=Shell, Material=S, Offset=0.3\n1\n\
         *Beam section, Elset=Beam, Material=S, Section=RECT\n0.5, 0.3\n0, 0, 1\n\
         *Step\n*Static\n*Boundary\n1, 1, 6\n10, 1, 6\n19, 1, 6\n100, 1, 6\n\
         *Node file\nU\n*El file\nS\n*End step\n",
    );
    let dir = std::env::temp_dir().join(format!("plx-expanded-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("strip.inp"), &text).unwrap();
    plxs_session::run_input_file(&dir, "strip").unwrap_or_else(|e| panic!("{e}"));
    let frd = read_frd(&dir.join("strip.frd")).unwrap();
    let input = crate::inp::read_inp(&dir.join("strip.inp")).unwrap().mesh;
    let _ = std::fs::remove_dir_all(&dir);
    let expanded = expanded_nodes(&input, &frd.mesh);
    // Every expanded node belongs to exactly one original node.
    let all: Vec<NodeId> = expanded.values().flatten().copied().collect();
    let unique: std::collections::BTreeSet<NodeId> = all.iter().copied().collect();
    assert_eq!(all.len(), unique.len());
    assert_eq!(unique.len(), frd.mesh.node_count());
    // The nodes in the middle of the elements' rows belong to none.
    let used: std::collections::BTreeSet<NodeId> = (input.elements().iter())
        .flat_map(|e| e.nodes.iter().copied())
        .collect();
    assert_eq!(
        expanded.keys().copied().collect::<Vec<_>>(),
        used.iter().copied().collect::<Vec<_>>()
    );
    for original in &used {
        let nodes = &expanded[original];
        let q = DVec3::from(input.node(*original).unwrap());
        // Corners of the shell have three nodes through the thickness, mid nodes two; beam
        // nodes eight or four around them.
        assert!(nodes.len() >= 2, "{original}: {nodes:?}");
        let centre = (nodes.iter())
            .map(|&id| DVec3::from(frd.mesh.node(id).unwrap()))
            .sum::<DVec3>()
            / nodes.len() as f64;
        if *original >= 100 {
            assert!((centre - q).length() < 1e-4, "{original}: {centre} vs {q}");
        } else {
            // The elements' normals point to the axis; an offset of 0.3 puts the mid-surface
            // 0.3 off the nodes against the normal, outside the radius of 5.
            let radial = DVec3::new(q.x, 0.0, q.z).normalize();
            let expected = q + radial * 0.3;
            assert!(
                (centre - expected).length() < 1e-3,
                "{original}: {centre} vs {expected}"
            );
        }
    }
}
