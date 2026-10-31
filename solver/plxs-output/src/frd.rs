//! The `.frd` writer: header, mesh (nodes and elements with CalculiX's element type numbers
//! and node order), then one data block per field and increment.

use std::io::{BufWriter, Write};
use std::path::Path;

use plxs_core::results::{IncrementKind, ResultField, Results, ResultsIncrement};
use plxs_core::{ElementType, Error, Model, Result};

use crate::fortran_e12;

/// The frd element type number and the node order of a type (frd lists the vertical
/// midside nodes of quadratic hexahedra and wedges before the top ones).
fn frd_element(kind: ElementType) -> Option<(u8, &'static [usize])> {
    use ElementType::*;
    const HEX8: [usize; 8] = [0, 1, 2, 3, 4, 5, 6, 7];
    const HEX20: [usize; 20] = [
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 16, 17, 18, 19, 12, 13, 14, 15,
    ];
    const TET4: [usize; 4] = [0, 1, 2, 3];
    const TET10: [usize; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
    const WEDGE6: [usize; 6] = [0, 1, 2, 3, 4, 5];
    const WEDGE15: [usize; 15] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 13, 14, 9, 10, 11];
    const TRI3: [usize; 3] = [0, 1, 2];
    const TRI6: [usize; 6] = [0, 1, 2, 3, 4, 5];
    const QUAD4: [usize; 4] = [0, 1, 2, 3];
    const QUAD8: [usize; 8] = [0, 1, 2, 3, 4, 5, 6, 7];
    Some(match kind {
        C3D8 | C3D8R | C3D8I => (1, &HEX8),
        C3D6 => (2, &WEDGE6),
        C3D4 => (3, &TET4),
        C3D20 | C3D20R => (4, &HEX20),
        C3D15 => (5, &WEDGE15),
        C3D10 | C3D10T => (6, &TET10),
        // Plane and axisymmetric elements are written unexpanded, as CalculiX does.
        CPS3 | CPE3 | CAX3 => (7, &TRI3),
        CPS6 | CPE6 | CAX6 => (8, &TRI6),
        CPS4 | CPS4R | CPE4 | CPE4R | CAX4 | CAX4R => (9, &QUAD4),
        CPS8 | CPS8R | CPE8 | CPE8R | CAX8 | CAX8R => (10, &QUAD8),
        M3D3 => (7, &TRI3),
        M3D6 => (8, &TRI6),
        M3D4 | M3D4R => (9, &QUAD4),
        T3D2 | B31 => (11, &[0, 1]),
        // Shells and beams with OUTPUT=2D.
        S3 => (7, &TRI3),
        S6 => (8, &TRI6),
        S4 | S4R => (9, &QUAD4),
        S8 | S8R => (10, &QUAD8),
        B32 | B32R => (12, &[0, 2, 1]),
        // CalculiX writes two-node springs and gaps as line elements, SPRING1 not at all.
        SPRING2 | GAPUNI => (11, &[0, 1]),
        _ => return None,
    })
}

/// The dense indices of the nodes CalculiX writes, ascending by id: those of elements
/// (springs and gaps included, although their elements are not written). Nodes no element
/// uses, such as reference and rotation nodes of rigid bodies, are left out.
pub fn written_nodes(model: &Model) -> Vec<usize> {
    let mut used = vec![false; model.nodes.len()];
    for element in &model.elements {
        for &id in &element.nodes {
            if let Some(i) = model.nodes.index(id) {
                used[i as usize] = true;
            }
        }
    }
    let mut order: Vec<usize> = (0..model.nodes.len()).filter(|&i| used[i]).collect();
    order.sort_by_key(|&i| model.nodes.ids[i]);
    order
}

fn io(path: &Path, e: std::io::Error) -> Error {
    Error::Io {
        path: path.display().to_string(),
        message: e.to_string(),
    }
}

/// Writes the `.frd` of a model and its results. The mesh is the model's (expansion of
/// shells and beams comes with those elements); nodes and elements are written in
/// ascending id order as CalculiX does.
pub fn write_frd(path: &Path, model: &Model, results: &Results, version: &str) -> Result<()> {
    let file = std::fs::File::create(path).map_err(|e| io(path, e))?;
    let mut out = BufWriter::new(file);
    write_frd_to(&mut out, model, results, version).map_err(|e| io(path, e))?;
    out.flush().map_err(|e| io(path, e))
}

pub fn write_frd_to(
    out: &mut impl Write,
    model: &Model,
    results: &Results,
    version: &str,
) -> std::io::Result<()> {
    writeln!(out, "    1C")?;
    writeln!(out, "    1UUSER{:62}", "")?;
    writeln!(out, "    1UDATE{:62}", "")?;
    writeln!(out, "    1UTIME{:62}", "")?;
    writeln!(out, "    1UHOST{:62}", "")?;
    writeln!(out, "    1UPGM{:15}PrePoLix{:40}", "", "")?;
    writeln!(out, "    1UVERSION{:11}Version {version}", "")?;
    writeln!(out, "    1UCOMPILETIME{:55}", "")?;
    writeln!(out, "    1UDIR{:63}", "")?;
    writeln!(out, "    1UDBN{:63}", "")?;
    for (i, material) in model.materials.iter().enumerate() {
        writeln!(out, "    1UMAT{:>5}{:<58}", i + 1, material.name)?;
    }

    // Nodes that belong to elements, ascending by id.
    let node_order = written_nodes(model);
    writeln!(out, "    2C{:>30}{:>38}", node_order.len(), 1)?;
    for &i in &node_order {
        // CalculiX writes the coordinates in single precision; rounding through f32 keeps the
        // last digit the same (2.02577507 is written as 2.02577).
        let c = model.nodes.coordinates[i].map(|v| f64::from(v as f32));
        writeln!(
            out,
            " -1{:>10}{}{}{}",
            model.nodes.ids[i],
            fortran_e12(c[0]),
            fortran_e12(c[1]),
            fortran_e12(c[2])
        )?;
    }
    writeln!(out, " -3")?;

    // Elements, ascending by id, with the material number of their section.
    let mut element_order: Vec<usize> = (0..model.elements.len()).collect();
    element_order.sort_by_key(|&i| model.elements[i].id);
    let material_number = |element: &plxs_core::Element| -> usize {
        element
            .section
            .and_then(|s| model.sections.get(s as usize))
            .and_then(|s| s.kind.material())
            .and_then(|name| model.materials.iter().position(|m| m.name == name))
            .map_or(1, |i| i + 1)
    };
    let written: Vec<usize> = element_order
        .iter()
        .copied()
        .filter(|&i| frd_element(model.elements[i].kind).is_some())
        .collect();
    writeln!(out, "    3C{:>30}{:>38}", written.len(), 1)?;
    for &i in &written {
        let element = &model.elements[i];
        let (kind, order) = frd_element(element.kind).unwrap();
        writeln!(
            out,
            " -1{:>10}{:>5}{:>5}{:>5}",
            element.id,
            kind,
            0,
            material_number(element)
        )?;
        for chunk in order.chunks(10) {
            write!(out, " -2")?;
            for &k in chunk {
                write!(out, "{:>10}", element.nodes[k])?;
            }
            writeln!(out)?;
        }
    }
    writeln!(out, " -3")?;

    let mut set = 0u32;
    for (ordinal, increment) in results.increments.iter().enumerate() {
        for field in &increment.fields {
            set += 1;
            write_block(
                out,
                set,
                ordinal as u32 + 1,
                increment,
                field,
                field.nodes.len(),
            )?;
        }
    }
    writeln!(out, " 9999")?;
    Ok(())
}

/// The time or frequency of a `100CL` line: ten significant digits in fixed notation as
/// CalculiX writes them (`1.000000000`, `10.89062500`), exponential outside that range.
fn format_time(value: f64) -> String {
    let magnitude = value.abs();
    if (1e-3..10.0).contains(&magnitude) {
        format!("{value:>12.9}")
    } else if (10.0..1e9).contains(&magnitude) {
        let digits = magnitude.log10().floor() as usize + 1;
        format!("{value:>12.prec$}", prec = 10 - digits)
    } else {
        fortran_e12(value)
    }
}

/// One result block. `ordinal` counts the written increments over all steps (CalculiX'
/// `100CL` numbering); time increments carry the total time, as in CalculiX.
fn write_block(
    out: &mut impl Write,
    set: u32,
    ordinal: u32,
    increment: &ResultsIncrement,
    field: &ResultField,
    node_count: usize,
) -> std::io::Result<()> {
    let analysis = match increment.kind {
        IncrementKind::Time => 0,
        IncrementKind::Dynamic => 1,
        IncrementKind::Frequency => 2,
        IncrementKind::Buckling => 4,
        IncrementKind::ComplexFrequency => 3,
    };
    writeln!(
        out,
        "    1PSTEP{:>26}{:>12}{:>12}          ",
        set, increment.increment, increment.step
    )?;
    writeln!(
        out,
        "  100CL{:>5}{}{:>12}{:>22}{:>5}{:>12}",
        100 + ordinal,
        format_time(match increment.kind {
            IncrementKind::Time | IncrementKind::Dynamic => increment.total_time,
            _ => increment.value,
        }),
        node_count,
        analysis,
        match increment.kind {
            IncrementKind::Time | IncrementKind::Dynamic => ordinal,
            _ => increment.increment,
        },
        1
    )?;
    let components = field.components.len();
    let (declared, trailer) = match field.name.as_str() {
        "DISP" | "FORC" | "VELO" | "PDISP" | "FLUX" => (components + 1, true),
        _ => (components, false),
    };
    writeln!(out, " -4  {:<8}{:>5}{:>5}", field.name, declared, 1)?;
    for (k, name) in field.components.iter().enumerate() {
        let (kind, c1, c2) = component_flags(&field.name, components, k);
        writeln!(out, " -5  {:<8}{:>5}{:>5}{:>5}{:>5}", name, 1, kind, c1, c2)?;
    }
    if trailer {
        writeln!(out, " -5  ALL     {:>5}{:>5}{:>5}{:>5}    1ALL", 1, 2, 0, 0)?;
    }
    for (n, &node) in field.nodes.iter().enumerate() {
        write!(out, " -1{:>10}", node)?;
        for (k, column) in field.values.iter().enumerate() {
            if k > 0 && k % 6 == 0 {
                write!(out, "\n -2{:>10}", "")?;
            }
            write!(out, "{}", fortran_e12(column[n]))?;
        }
        writeln!(out)?;
    }
    writeln!(out, " -3")
}

/// The type and index columns of a `-5` line: vectors (`1 2 k 0`), tensors (`1 4 i j`),
/// scalars (`1 1 0 0`).
fn component_flags(field: &str, components: usize, k: usize) -> (u8, u8, u8) {
    // Displacements and forces with rotations (U1 nodes): a vector and three scalars.
    if matches!(field, "DISP" | "FORC") && components == 6 {
        return if k < 3 {
            (2, k as u8 + 1, 0)
        } else {
            (1, 0, 0)
        };
    }
    match components {
        3 => (2, k as u8 + 1, 0),
        6 => {
            const IJ: [(u8, u8); 6] = [(1, 1), (2, 2), (3, 3), (1, 2), (2, 3), (3, 1)];
            (4, IJ[k].0, IJ[k].1)
        }
        _ => (1, 0, 0),
    }
}
