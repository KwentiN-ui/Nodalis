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
    Some(match kind {
        C3D8 | C3D8R | C3D8I => (1, &HEX8),
        C3D6 => (2, &WEDGE6),
        C3D4 => (3, &TET4),
        C3D20 | C3D20R => (4, &HEX20),
        C3D15 => (5, &WEDGE15),
        C3D10 | C3D10T => (6, &TET10),
        _ => return None,
    })
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

    // Nodes, ascending by id.
    let mut node_order: Vec<usize> = (0..model.nodes.len()).collect();
    node_order.sort_by_key(|&i| model.nodes.ids[i]);
    writeln!(out, "    2C{:>30}{:>38}", model.nodes.len(), 1)?;
    for &i in &node_order {
        let c = model.nodes.coordinates[i];
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

    // Load case numbers as CalculiX counts them: one per step for time increments, one
    // per mode of a frequency step (the first mode takes the step's number), the static
    // reference solution of a buckle step takes the step's number and its modes count on.
    let mut block = 0u32;
    let mut case = 100u32;
    let mut step_case = 100u32;
    let mut last_step = 0u32;
    for increment in &results.increments {
        if increment.step != last_step {
            last_step = increment.step;
            case += 1;
            step_case = case;
        }
        let this_case = match increment.kind {
            IncrementKind::Time => step_case,
            IncrementKind::Buckling => step_case + increment.increment,
            IncrementKind::Frequency | IncrementKind::ComplexFrequency => {
                step_case + increment.increment.saturating_sub(1)
            }
        };
        case = case.max(this_case);
        for field in &increment.fields {
            block += 1;
            write_block(out, block, this_case, increment, field, model.nodes.len())?;
        }
    }
    writeln!(out, " 9999")?;
    Ok(())
}

/// A number as CalculiX prints the generalized mass and stiffness of a mode: `2.747649E+07`.
fn fortran_e12_6(value: f64) -> String {
    if value == 0.0 || !value.is_finite() {
        return "0.000000E+00".to_string();
    }
    let text = format!("{:.6E}", value.abs());
    let (mantissa, exponent) = text.split_once('E').unwrap();
    let exponent: i32 = exponent.parse().unwrap();
    let sign = if value < 0.0 { "-" } else { "" };
    let exp_sign = if exponent < 0 { "-" } else { "+" };
    format!("{sign}{mantissa}E{exp_sign}{:02}", exponent.abs())
}

/// The value column of a block header: ten significant digits in twelve columns
/// (`1.000000000`, `834.2587061`, `2051.835249`), exponent form outside that range.
fn format_time(value: f64) -> String {
    if (1e-3..1e10).contains(&value.abs()) {
        let decimals = (9 - value.abs().log10().floor() as i32).clamp(0, 9) as usize;
        format!("{value:>12.decimals$}")
    } else {
        fortran_e12(value)
    }
}

fn write_block(
    out: &mut impl Write,
    block: u32,
    case: u32,
    increment: &ResultsIncrement,
    field: &ResultField,
    node_count: usize,
) -> std::io::Result<()> {
    let analysis = match increment.kind {
        IncrementKind::Time => 0,
        IncrementKind::Frequency => 2,
        IncrementKind::Buckling => 4,
        IncrementKind::ComplexFrequency => 3,
    };
    writeln!(
        out,
        "    1PSTEP{:>26}{:>12}{:>12}          ",
        block,
        increment.increment.max(1),
        increment.step
    )?;
    if matches!(increment.kind, IncrementKind::Frequency) {
        // Generalized mass and stiffness of the (mass normalized) mode, nodal diameter,
        // subcase and mode number.
        let omega = 2.0 * std::f64::consts::PI * increment.value;
        let header = |key: &str, value: String| {
            format!("{:<10}{:>26}{:30}", format!("    1P{key}"), value, "")
        };
        writeln!(out, "{}", header("GM", fortran_e12_6(1.0)))?;
        writeln!(out, "{}", header("GK", fortran_e12_6(omega * omega)))?;
        writeln!(
            out,
            "{}",
            header(
                "HID",
                increment
                    .nodal_diameter
                    .map_or("-1".to_string(), |d| d.to_string())
            )
        )?;
        writeln!(out, "{}", header("SUBC", "0".to_string()))?;
        writeln!(out, "{}", header("MODE", increment.increment.to_string()))?;
    }
    let (analysis_name, trailer_width) = match increment.kind {
        IncrementKind::Frequency => ("MODAL", 7),
        _ => ("", 12),
    };
    writeln!(
        out,
        "  100CL{:>5}{}{:>12}{:>22}{:>5}{}{:>width$}",
        case,
        format_time(increment.value),
        node_count,
        analysis,
        case - 100,
        analysis_name,
        1,
        width = trailer_width
    )?;
    let components = field.components.len();
    let (declared, trailer) = match field.name.as_str() {
        "DISP" | "FORC" | "VELO" | "PDISP" => (components + 1, true),
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
    let _ = field;
    match components {
        3 => (2, k as u8 + 1, 0),
        6 => {
            const IJ: [(u8, u8); 6] = [(1, 1), (2, 2), (3, 3), (1, 2), (2, 3), (3, 1)];
            (4, IJ[k].0, IJ[k].1)
        }
        _ => (1, 0, 0),
    }
}
