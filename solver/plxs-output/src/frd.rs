//! The `.frd` writer: header, mesh (nodes and elements with CalculiX's element type numbers
//! and node order), then one data block per field and increment.

use std::io::{BufWriter, Write};
use std::path::Path;

use plxs_core::results::{IncrementKind, ResultField, Results, ResultsIncrement};
use plxs_core::{ElementType, Error, Model, Result};

use crate::fortran_e12;

/// The frd element type number and the node order of a type, `None` for elements the `.frd`
/// leaves out (frd lists the vertical
/// midside nodes of quadratic hexahedra and wedges before the top ones).
pub fn frd_element(kind: ElementType) -> Option<(u8, &'static [usize])> {
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
/// uses, such as reference and rotation nodes of rigid bodies, are left out; so are nodes
/// with nothing but springs to ground (`SPRING1`, which the file has no element for) and the
/// rotation nodes of rigid bodies, whose displacements are rotations, also when springs
/// hold them.
pub fn written_nodes(model: &Model) -> Vec<usize> {
    let used = node_mask(model);
    let mut order: Vec<usize> = (0..model.nodes.len()).filter(|&i| used[i]).collect();
    order.sort_by_key(|&i| model.nodes.ids[i]);
    order
}

/// Which nodes [`written_nodes`] writes, by dense index.
fn node_mask(model: &Model) -> Vec<bool> {
    let mut used = vec![false; model.nodes.len()];
    for element in model
        .elements
        .iter()
        .filter(|e| e.kind != ElementType::SPRING1)
    {
        for &id in &element.nodes {
            if let Some(i) = model.nodes.index(id) {
                used[i as usize] = true;
            }
        }
    }
    for body in &model.constraints.rigid_bodies {
        if body.rotation_node != body.reference_node
            && let Some(i) = model.nodes.index(body.rotation_node)
        {
            used[i as usize] = false;
        }
    }
    used
}

/// The indices of the elements the file holds, ascending by id: those of a type it has,
/// without springs to a node it leaves out (a spring between rotation nodes).
pub fn written_elements(model: &Model) -> Vec<usize> {
    let used = node_mask(model);
    let mut order: Vec<usize> = (0..model.elements.len())
        .filter(|&i| {
            let element = &model.elements[i];
            frd_element(element.kind).is_some()
                && (element.nodes.iter())
                    .all(|&id| model.nodes.index(id).is_some_and(|n| used[n as usize]))
        })
        .collect();
    order.sort_by_key(|&i| model.elements[i].id);
    order
}

/// A cyclic symmetric model and its results expanded to the sectors the `.frd` file holds
/// (`NGRAPH`): the nodes and elements of every further sector, ids shifted, and the fields
/// on all of them, as [`write`] writes them. `None` without cyclic symmetry. The expanded
/// model has no cyclic symmetry left, so it is not expanded twice.
pub fn expand_sectors(model: &Model, results: &Results) -> Option<(Model, Results)> {
    let sectors = Sectors::of(model);
    if sectors.count() == 1 {
        return None;
    }
    let mut expanded = model.clone();
    expanded.constraints.cyclic_symmetry = None;
    let nodes = written_nodes(model);
    for (j, rotation) in sectors.rotations.iter().enumerate().skip(1) {
        let j = j as u32;
        for &i in &nodes {
            expanded.nodes.insert(
                model.nodes.ids[i] + j * sectors.node_offset,
                sectors.rotate_point(rotation, model.nodes.coordinates[i]),
            );
        }
        for element in (model.elements.iter()).filter(|e| frd_element(e.kind).is_some()) {
            let mut copy = element.clone();
            copy.id += j * sectors.element_offset;
            for node in &mut copy.nodes {
                *node += j * sectors.node_offset;
            }
            expanded.elements.push(copy);
        }
    }
    expanded.rebuild_indices();
    let mut results = results.clone();
    for increment in &mut results.increments {
        increment.fields = sectors.expand(increment);
    }
    Some((expanded, results))
}

fn io(path: &Path, e: std::io::Error) -> Error {
    Error::Io {
        path: path.display().to_string(),
        message: e.to_string(),
    }
}

/// How the numbers of an `.frd` are written.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FrdFormat {
    /// 12-column fixed-width text (`E12.5`), what CalculiX writes by default.
    #[default]
    Ascii,
    /// Little-endian binary blocks inside the text frame (format flags of the frd
    /// description: coordinates as 8-byte floats, flag 3; results as 4-byte floats, flag
    /// 2). About a third of the size, read without parsing numbers; same precision for the
    /// results, full precision for the coordinates.
    Binary,
}

/// Writes the `.frd` of a model and its results. The mesh is the model's (expansion of
/// shells and beams comes with those elements); nodes and elements are written in
/// ascending id order as CalculiX does.
pub fn write_frd(path: &Path, model: &Model, results: &Results, version: &str) -> Result<()> {
    write_frd_as(path, model, results, version, FrdFormat::Ascii)
}

/// [`write_frd`] in the given format.
pub fn write_frd_as(
    path: &Path,
    model: &Model,
    results: &Results,
    version: &str,
    format: FrdFormat,
) -> Result<()> {
    let file = std::fs::File::create(path).map_err(|e| io(path, e))?;
    let mut out = BufWriter::new(file);
    write_frd_to_as(&mut out, model, results, version, format).map_err(|e| io(path, e))?;
    out.flush().map_err(|e| io(path, e))
}

pub fn write_frd_to(
    out: &mut impl Write,
    model: &Model,
    results: &Results,
    version: &str,
) -> std::io::Result<()> {
    write_frd_to_as(out, model, results, version, FrdFormat::Ascii)
}

pub fn write_frd_to_as(
    out: &mut impl Write,
    model: &Model,
    results: &Results,
    version: &str,
    format: FrdFormat,
) -> std::io::Result<()> {
    let binary = format == FrdFormat::Binary;
    writeln!(out, "    1C")?;
    writeln!(out, "    1UUSER{:62}", "")?;
    writeln!(out, "    1UDATE{:62}", "")?;
    writeln!(out, "    1UTIME{:62}", "")?;
    writeln!(out, "    1UHOST{:62}", "")?;
    writeln!(out, "    1UPGM{:15}Nodalis {:40}", "", "")?;
    writeln!(out, "    1UVERSION{:11}Version {version}", "")?;
    writeln!(out, "    1UCOMPILETIME{:55}", "")?;
    writeln!(out, "    1UDIR{:63}", "")?;
    writeln!(out, "    1UDBN{:63}", "")?;
    for (i, material) in model.materials.iter().enumerate() {
        writeln!(out, "    1UMAT{:>5}{:<58}", i + 1, material.name)?;
    }

    // Nodes that belong to elements, ascending by id; then the copies of further sectors
    // of a cyclic symmetric model.
    let node_order = written_nodes(model);
    let sectors = Sectors::of(model);
    writeln!(
        out,
        "    2C{:>30}{:>38}",
        node_order.len() * sectors.count(),
        if binary { 3 } else { 1 }
    )?;
    for (j, rotation) in sectors.rotations.iter().enumerate() {
        for &i in &node_order {
            let id = model.nodes.ids[i] + j as u32 * sectors.node_offset;
            let x = sectors.rotate_point(rotation, model.nodes.coordinates[i]);
            if binary {
                out.write_all(&(id as i32).to_le_bytes())?;
                for v in x {
                    out.write_all(&v.to_le_bytes())?;
                }
                continue;
            }
            // CalculiX writes the coordinates in single precision; rounding through f32
            // keeps the last digit the same (2.02577507 is written as 2.02577).
            let c = x.map(|v| f64::from(v as f32));
            writeln!(
                out,
                " -1{:>10}{}{}{}",
                id,
                fortran_e12(c[0]),
                fortran_e12(c[1]),
                fortran_e12(c[2])
            )?;
        }
    }
    if !binary {
        writeln!(out, " -3")?;
    }

    // Elements, ascending by id, with the material number of their section.
    let material_number = |element: &plxs_core::Element| -> usize {
        element
            .section
            .and_then(|s| model.sections.get(s as usize))
            .and_then(|s| s.kind.material())
            .and_then(|name| model.materials.iter().position(|m| m.name == name))
            .map_or(1, |i| i + 1)
    };
    let written = written_elements(model);
    writeln!(
        out,
        "    3C{:>30}{:>38}",
        written.len() * sectors.count(),
        if binary { 3 } else { 1 }
    )?;
    for j in 0..sectors.count() as u32 {
        for &i in &written {
            let element = &model.elements[i];
            let (kind, order) = frd_element(element.kind).unwrap();
            let id = element.id + j * sectors.element_offset;
            if binary {
                for v in [
                    id as i32,
                    i32::from(kind),
                    0,
                    material_number(element) as i32,
                ] {
                    out.write_all(&v.to_le_bytes())?;
                }
                for &k in order {
                    out.write_all(
                        &((element.nodes[k] + j * sectors.node_offset) as i32).to_le_bytes(),
                    )?;
                }
                continue;
            }
            writeln!(
                out,
                " -1{:>10}{:>5}{:>5}{:>5}",
                id,
                kind,
                0,
                material_number(element)
            )?;
            for chunk in order.chunks(10) {
                write!(out, " -2")?;
                for &k in chunk {
                    write!(out, "{:>10}", element.nodes[k] + j * sectors.node_offset)?;
                }
                writeln!(out)?;
            }
        }
    }
    if !binary {
        writeln!(out, " -3")?;
    }

    // `ordinal` counts the written increments over all steps (`100CL`); a mode of a cyclic
    // symmetric model stands for the pair of its real and imaginary solutions, of which
    // CalculiX writes only the first, so it counts twice. Its mode number in the header is
    // its place within the nodal diameter, counted the same way.
    let mut set = 0u32;
    let mut ordinal = 0u32;
    for (index, increment) in results.increments.iter().enumerate() {
        ordinal += 1;
        let mode = match increment.nodal_diameter {
            Some(diameter) => {
                let before = results.increments[..index]
                    .iter()
                    .rev()
                    .take_while(|i| i.step == increment.step)
                    .filter(|i| i.nodal_diameter == Some(diameter))
                    .count() as u32;
                2 * before + 1
            }
            None => increment.increment,
        };
        let fields = sectors.expand(increment);
        for field in &fields {
            set += 1;
            write_block(
                out,
                set,
                ordinal,
                mode,
                sectors.axis.as_ref(),
                increment,
                field,
                field.nodes.len(),
                binary,
            )?;
        }
        if increment.nodal_diameter.is_some() {
            ordinal += 1;
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
/// `100CL` numbering); time increments carry the total time, as in CalculiX. `mode` is the
/// mode number of the `1PMODE` line, `axis` the `1PAX` line of cyclic modes.
#[allow(clippy::too_many_arguments)]
fn write_block(
    out: &mut impl Write,
    set: u32,
    ordinal: u32,
    mode: u32,
    axis: Option<&[f64; 6]>,
    increment: &ResultsIncrement,
    field: &ResultField,
    node_count: usize,
    binary: bool,
) -> std::io::Result<()> {
    let analysis = match increment.kind {
        IncrementKind::Time => 0,
        IncrementKind::Dynamic | IncrementKind::SteadyState => 1,
        IncrementKind::Frequency => 2,
        IncrementKind::Buckling => 4,
        IncrementKind::ComplexFrequency | IncrementKind::Sensitivity => 3,
    };
    writeln!(
        out,
        "    1PSTEP{:>26}{:>12}{:>12}          ",
        set,
        if matches!(
            increment.kind,
            IncrementKind::SteadyState | IncrementKind::ComplexFrequency
        ) {
            0
        } else {
            increment.increment
        },
        increment.step
    )?;
    if matches!(
        increment.kind,
        IncrementKind::Frequency | IncrementKind::ComplexFrequency
    ) {
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
        if let (Some(axis), Some(_)) = (axis, increment.nodal_diameter) {
            write!(out, "    1PAX{:13}", "")?;
            for v in axis {
                write!(out, "{}", fortran_e12(*v))?;
            }
            writeln!(out)?;
        }
        writeln!(out, "{}", header("SUBC", "0".to_string()))?;
        writeln!(out, "{}", header("MODE", mode.to_string()))?;
    }
    let (analysis_name, trailer_width) = match increment.kind {
        IncrementKind::Frequency => ("MODAL", 7),
        _ => ("", 12),
    };
    writeln!(
        out,
        "  100CL{:>5}{}{:>12}{:>22}{:>5}{}{:>width$}",
        100 + ordinal,
        format_time(match increment.kind {
            IncrementKind::Time | IncrementKind::Dynamic => increment.total_time,
            _ => increment.value,
        }),
        node_count,
        analysis,
        ordinal,
        analysis_name,
        if binary { 2 } else { 1 },
        width = trailer_width
    )?;
    let components = field.components.len();
    let (declared, trailer) = match field.name.as_str() {
        "DISP" | "FORC" | "VELO" | "ACCE" | "FLUX" | "NORM" | "DISPI" | "FORCI" => {
            (components + 1, true)
        }
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
    if binary {
        for (n, &node) in field.nodes.iter().enumerate() {
            out.write_all(&(node as i32).to_le_bytes())?;
            for column in &field.values {
                out.write_all(&(column[n] as f32).to_le_bytes())?;
            }
        }
        return Ok(());
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
    // Contact results are six scalars.
    if field == "CONTACT" {
        return (1, 0, 0);
    }
    // Magnitudes and phases of complex displacements (MAG1..3, PHA1..3).
    if field == "PDISP" {
        return (12, k as u8 + 1, 0);
    }
    if field == "PSTRESS" {
        const IJ: [(u8, u8); 6] = [(1, 1), (2, 2), (3, 3), (1, 2), (2, 3), (3, 1)];
        return (14, IJ[k % 6].0, IJ[k % 6].1);
    }
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

/// The name of the imaginary part of a field in the `.frd`: `I` appended (`DISPI`), in
/// place of the last letter when the name has the eight letters the format allows
/// (`TOSTRAIN` becomes `TOSTRAII`), as CalculiX names them.
pub fn imaginary_field_name(name: &str) -> String {
    let mut out: String = name.chars().take(7).collect();
    out.push('I');
    out
}

/// The sectors of a cyclic symmetric model written to the `.frd` (`NGRAPH`): sector `j`
/// is the model rotated by `−jφ` about the axis (the axis points the way the slave side
/// follows the master side, see `plxs_constraints::cyclic`), its node and element ids
/// shifted by `j` times the largest id. Without cyclic symmetry there is one sector, the
/// model itself.
struct Sectors {
    /// One rotation per sector, the identity first.
    rotations: Vec<[[f64; 3]; 3]>,
    origin: [f64; 3],
    angle: f64,
    node_offset: u32,
    element_offset: u32,
    /// The `1PAX` line of cyclic modes: a point on the axis and a second one along the
    /// reference direction (the opposite of the axis).
    axis: Option<[f64; 6]>,
}

impl Sectors {
    fn of(model: &Model) -> Sectors {
        const IDENTITY: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut sectors = Sectors {
            rotations: vec![IDENTITY],
            origin: [0.0; 3],
            angle: 0.0,
            node_offset: model.nodes.ids.iter().copied().max().unwrap_or(0),
            element_offset: model.elements.iter().map(|e| e.id).max().unwrap_or(0),
            axis: None,
        };
        let Some(cyclic) = &model.constraints.cyclic_symmetry else {
            return sectors;
        };
        let [p, q] = cyclic.axis;
        let d = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
        let length = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if length == 0.0 || cyclic.sectors == 0 {
            return sectors;
        }
        let unit = d.map(|c| c / length);
        sectors.origin = p;
        sectors.angle = 2.0 * std::f64::consts::PI / f64::from(cyclic.sectors);
        sectors.axis = Some([
            p[0],
            p[1],
            p[2],
            p[0] - unit[0],
            p[1] - unit[1],
            p[2] - unit[2],
        ]);
        for j in 1..cyclic.ngraph.min(cyclic.sectors) {
            sectors
                .rotations
                .push(rotation_about(unit, -f64::from(j) * sectors.angle));
        }
        sectors
    }

    fn count(&self) -> usize {
        self.rotations.len()
    }

    fn rotate_point(&self, r: &[[f64; 3]; 3], x: [f64; 3]) -> [f64; 3] {
        let o = self.origin;
        let v = apply(r, [x[0] - o[0], x[1] - o[1], x[2] - o[2]]);
        [v[0] + o[0], v[1] + o[1], v[2] + o[2]]
    }

    /// The fields of an increment on all sectors. A real field is rotated with its sector;
    /// a cyclic mode of nodal diameter `k` (fields `X` and `XI`, real and imaginary part
    /// of `c`) is in sector `j` also shifted in phase: `c_j = e^{ikjφ} R(−jφ) c`. Vectors
    /// (three components) and tensors (six) rotate, scalars are copied.
    fn expand(&self, increment: &ResultsIncrement) -> Vec<ResultField> {
        if self.count() == 1 {
            return increment.fields.clone();
        }
        let fields = &increment.fields;
        let mut out: Vec<ResultField> = Vec::with_capacity(fields.len());
        for (index, field) in fields.iter().enumerate() {
            let partner = |name: &str| fields.iter().position(|f| f.name == name);
            // (real part, imaginary part, this field is the imaginary one)
            let real_of = fields
                .iter()
                .position(|f| f.name != field.name && imaginary_field_name(&f.name) == field.name);
            let (real, imaginary, is_imaginary) = match (increment.nodal_diameter, real_of) {
                (Some(_), Some(r)) => (r, Some(index), true),
                (Some(_), None) => (index, partner(&imaginary_field_name(&field.name)), false),
                (None, _) => (index, None, false),
            };
            let mut expanded = field.clone();
            for (j, rotation) in self.rotations.iter().enumerate().skip(1) {
                let phase = increment
                    .nodal_diameter
                    .map_or(0.0, |k| f64::from(k) * j as f64 * self.angle);
                let (sin, cos) = phase.sin_cos();
                let shift = j as u32 * self.node_offset;
                expanded.nodes.extend(field.nodes.iter().map(|n| n + shift));
                let re = &fields[real];
                let im = imaginary.map(|i| &fields[i]);
                let count = field.nodes.len();
                let mut values: Vec<Vec<f64>> = vec![Vec::with_capacity(count); field.values.len()];
                for n in 0..count {
                    let a: Vec<f64> = re.values.iter().map(|c| c[n]).collect();
                    let b: Vec<f64> = match im {
                        Some(f) => f.values.iter().map(|c| c[n]).collect(),
                        None => vec![0.0; a.len()],
                    };
                    // c_j = (cos + i sin) R c: real part cos Ra − sin Rb, imaginary part
                    // sin Ra + cos Rb.
                    let (ra, rb) = (
                        rotate_components(rotation, &a),
                        rotate_components(rotation, &b),
                    );
                    for (k, column) in values.iter_mut().enumerate() {
                        column.push(if is_imaginary {
                            sin * ra[k] + cos * rb[k]
                        } else {
                            cos * ra[k] - sin * rb[k]
                        });
                    }
                }
                for (column, extra) in expanded.values.iter_mut().zip(values) {
                    column.extend(extra);
                }
            }
            out.push(expanded);
        }
        out
    }
}

/// Rotates the components of a node: groups of three as vectors, six as a symmetric tensor
/// (`xx, yy, zz, xy, yz, zx`), anything else unchanged.
fn rotate_components(r: &[[f64; 3]; 3], v: &[f64]) -> Vec<f64> {
    match v.len() {
        3 => apply(r, [v[0], v[1], v[2]]).to_vec(),
        6 => {
            let s = [[v[0], v[3], v[5]], [v[3], v[1], v[4]], [v[5], v[4], v[2]]];
            let mut t = [[0.0; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    t[i][j] = (0..3)
                        .map(|k| (0..3).map(|l| r[i][k] * s[k][l] * r[j][l]).sum::<f64>())
                        .sum();
                }
            }
            vec![t[0][0], t[1][1], t[2][2], t[0][1], t[1][2], t[2][0]]
        }
        _ => v.to_vec(),
    }
}

/// The rotation by `angle` about the unit vector `axis` (Rodrigues).
fn rotation_about(axis: [f64; 3], angle: f64) -> [[f64; 3]; 3] {
    let (s, c) = angle.sin_cos();
    let [x, y, z] = axis;
    let t = 1.0 - c;
    [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
}

fn apply(r: &[[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
        r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
        r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
    ]
}
