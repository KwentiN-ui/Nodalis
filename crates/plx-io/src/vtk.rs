//! Export of results to VTK formats for ParaView and other VTK programs.
//!
//! One increment becomes one unstructured grid: the undeformed mesh with every result field as
//! point data (`DISP` as a 3-component vector, so that ParaView's Warp By Vector deforms the
//! mesh), the node and element numbers, and the part of each element. Two formats:
//!
//! - [`VtkFormat::Xml`], VTK's XML format (`.vtu`), compressed binary or ASCII.
//! - [`VtkFormat::Legacy`], the legacy format (`.vtk`, version 4.2), binary or ASCII.
//!
//! Several increments are written as one file per increment plus a collection file that
//! ParaView opens as a time series, as the ParaView documentation recommends: a ParaView Data
//! file (`.pvd`) for `.vtu` files and a JSON file series (`.vtk.series`, ParaView 5.5 and
//! later) for legacy files, which PVD does not support. Both give every file its time value.

use std::fmt::Write as _;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use base64::Engine as _;
use plx_mesh::{ElementShape, FeMesh};
use plx_results::Increment;

/// The VTK file format of an export.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VtkFormat {
    /// VTK XML unstructured grid (`.vtu`), collected in a `.pvd` file.
    Xml,
    /// Legacy VTK (`.vtk`), collected in a `.vtk.series` file.
    Legacy,
}

impl VtkFormat {
    /// Extension of a file of one increment.
    pub fn extension(self) -> &'static str {
        match self {
            VtkFormat::Xml => "vtu",
            VtkFormat::Legacy => "vtk",
        }
    }

    /// Extension of the collection file of several increments (without the leading dot).
    pub fn collection_extension(self) -> &'static str {
        match self {
            VtkFormat::Xml => "pvd",
            VtkFormat::Legacy => "vtk.series",
        }
    }
}

/// VTK cell type and the order of an element's nodes in it. Node orders of CalculiX (as stored
/// in [`FeMesh`]) and VTK agree except for the quadratic line, whose midside node comes second
/// in CalculiX and last in VTK.
fn vtk_cell(shape: ElementShape) -> (u8, &'static [usize]) {
    match shape {
        ElementShape::Line2 => (3, &[0, 1]),
        ElementShape::Line3 => (21, &[0, 2, 1]),
        ElementShape::Tri3 => (5, &[0, 1, 2]),
        ElementShape::Tri6 => (22, &[0, 1, 2, 3, 4, 5]),
        ElementShape::Quad4 => (9, &[0, 1, 2, 3]),
        ElementShape::Quad8 => (23, &[0, 1, 2, 3, 4, 5, 6, 7]),
        ElementShape::Tet4 => (10, &[0, 1, 2, 3]),
        ElementShape::Tet10 => (24, &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9]),
        ElementShape::Wedge6 => (13, &[0, 1, 2, 3, 4, 5]),
        ElementShape::Wedge15 => (26, &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]),
        ElementShape::Hex8 => (12, &[0, 1, 2, 3, 4, 5, 6, 7]),
        ElementShape::Hex20 => (
            25,
            &[
                0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
            ],
        ),
    }
}

/// The mesh as VTK cells: 0-based point indices, offsets, cell types, element numbers and
/// part indices (-1 for elements in no part).
struct Cells {
    connectivity: Vec<i64>,
    offsets: Vec<i64>,
    types: Vec<u8>,
    ids: Vec<i32>,
    parts: Vec<i32>,
}

impl Cells {
    fn new(mesh: &FeMesh) -> Self {
        let mut part_of = std::collections::HashMap::new();
        for (index, part) in mesh.parts.iter().enumerate() {
            for &id in &part.elements {
                part_of.entry(id).or_insert(index as i32);
            }
        }
        let mut cells = Cells {
            connectivity: Vec::new(),
            offsets: Vec::new(),
            types: Vec::new(),
            ids: Vec::new(),
            parts: Vec::new(),
        };
        for element in mesh.elements() {
            let (kind, order) = vtk_cell(element.shape);
            let nodes: Option<Vec<i64>> = order
                .iter()
                .map(|&k| {
                    let id = *element.nodes.get(k)?;
                    mesh.node_index(id).map(|i| i as i64)
                })
                .collect();
            // Elements with missing nodes cannot be drawn and are left out.
            let Some(nodes) = nodes else { continue };
            cells.connectivity.extend(nodes);
            cells.offsets.push(cells.connectivity.len() as i64);
            cells.types.push(kind);
            cells.ids.push(element.id as i32);
            cells
                .parts
                .push(part_of.get(&element.id).copied().unwrap_or(-1));
        }
        cells
    }

    fn len(&self) -> usize {
        self.types.len()
    }
}

/// A point data array: name, number of components, component names and values (tuple after
/// tuple).
struct PointArray {
    name: String,
    components: Vec<String>,
    values: Vec<f32>,
}

/// Point data of an increment. The components CalculiX wrote form one array per field
/// (`DISP` with `U1`, `U2`, `U3` as a vector, `STRESS` with its six components as a symmetric
/// tensor in VTK's order XX, YY, ZZ, XY, YZ, XZ); computed components such as `MISES` or the
/// principal values are scalar arrays named `FIELD_COMPONENT`.
fn point_arrays(increment: &Increment, nodes: usize) -> Vec<PointArray> {
    let mut arrays = Vec::new();
    for field in &increment.fields {
        let written: Vec<_> = field
            .components
            .iter()
            .filter(|c| !c.derived && c.values.len() == nodes)
            .collect();
        if !written.is_empty() {
            let mut values = Vec::with_capacity(nodes * written.len());
            let columns: Vec<_> = written.iter().map(|c| c.values.get()).collect();
            for node in 0..nodes {
                values.extend(columns.iter().map(|c| c[node]));
            }
            arrays.push(PointArray {
                name: field.name.clone(),
                components: written.iter().map(|c| c.name.clone()).collect(),
                values,
            });
        }
        for component in &field.components {
            if component.derived && component.values.len() == nodes {
                arrays.push(PointArray {
                    name: format!("{}_{}", field.name, component.name),
                    components: vec![component.name.clone()],
                    values: component.values.to_vec(),
                });
            }
        }
    }
    arrays
}

/// Values describing an increment, written as field data of its file: the time value
/// ParaView shows, step and increment number and the increment's own value (time,
/// eigenfrequency or load factor).
fn increment_info(increment: Option<&Increment>, time: f64) -> Vec<(String, f64)> {
    let mut info = vec![("TimeValue".to_string(), time)];
    if let Some(increment) = increment {
        info.push(("STEP".into(), f64::from(increment.step)));
        info.push(("INCREMENT".into(), f64::from(increment.increment)));
        let label = increment
            .kind
            .value_label()
            .replace(' ', "_")
            .to_uppercase();
        info.push((label, increment.value));
    }
    info
}

/// Array names without white space, which the legacy format does not allow.
fn legacy_name(name: &str) -> String {
    let name: String = name
        .chars()
        .map(|c| if c.is_whitespace() { '_' } else { c })
        .collect();
    if name.is_empty() { "_".into() } else { name }
}

/// A number as ASCII VTK reads it; NaN (nodes without a value) is written as `nan`.
fn ascii<T: std::fmt::Display>(out: &mut String, values: impl IntoIterator<Item = T>) {
    for (i, value) in values.into_iter().enumerate() {
        if i > 0 {
            out.push(if i % 9 == 0 { '\n' } else { ' ' });
        }
        let _ = write!(out, "{value}");
    }
    out.push('\n');
}

/// Little-endian bytes of numbers for the XML format.
trait LeBytes: Copy {
    const TYPE: &'static str;
    fn put_le(self, out: &mut Vec<u8>);
    fn put_be(self, out: &mut Vec<u8>);
}

macro_rules! le_bytes {
    ($t:ty, $name:literal) => {
        impl LeBytes for $t {
            const TYPE: &'static str = $name;
            fn put_le(self, out: &mut Vec<u8>) {
                out.extend_from_slice(&self.to_le_bytes());
            }
            fn put_be(self, out: &mut Vec<u8>) {
                out.extend_from_slice(&self.to_be_bytes());
            }
        }
    };
}

le_bytes!(f32, "Float32");
le_bytes!(f64, "Float64");
le_bytes!(i32, "Int32");
le_bytes!(i64, "Int64");
le_bytes!(u8, "UInt8");

/// Writer of a VTK XML unstructured grid.
struct Xml {
    out: String,
    binary: bool,
}

impl Xml {
    /// A `DataArray` element. Binary data is zlib compressed and base64 encoded inline, as one
    /// block with a `UInt64` header (VTK's `vtkZLibDataCompressor`).
    fn array<T: LeBytes + std::fmt::Display>(
        &mut self,
        name: &str,
        components: &[String],
        values: &[T],
    ) {
        let tuples = values.len() / components.len().max(1);
        let _ = write!(
            self.out,
            "<DataArray type=\"{}\" Name=\"{}\" NumberOfTuples=\"{tuples}\"",
            T::TYPE,
            xml(name)
        );
        if components.len() > 1 {
            let _ = write!(self.out, " NumberOfComponents=\"{}\"", components.len());
            for (i, component) in components.iter().enumerate() {
                let _ = write!(self.out, " ComponentName{i}=\"{}\"", xml(component));
            }
        }
        if self.binary {
            let mut raw = Vec::with_capacity(std::mem::size_of_val(values));
            for &value in values {
                value.put_le(&mut raw);
            }
            let mut encoder =
                flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::new(6));
            let _ = encoder.write_all(&raw);
            let compressed = encoder.finish().unwrap_or_default();
            let mut header = Vec::with_capacity(32);
            for value in [
                1,
                raw.len() as u64,
                raw.len() as u64,
                compressed.len() as u64,
            ] {
                header.extend_from_slice(&value.to_le_bytes());
            }
            let engine = base64::engine::general_purpose::STANDARD;
            let _ = writeln!(
                self.out,
                " format=\"binary\">\n{}{}\n</DataArray>",
                engine.encode(header),
                engine.encode(compressed)
            );
        } else {
            self.out.push_str(" format=\"ascii\">\n");
            ascii(&mut self.out, values.iter());
            self.out.push_str("</DataArray>\n");
        }
    }

    /// A string array, always ASCII: the characters of each string as numbers, each string
    /// ended by 0.
    fn strings(&mut self, name: &str, strings: &[String]) {
        let _ = writeln!(
            self.out,
            "<DataArray type=\"String\" Name=\"{}\" NumberOfTuples=\"{}\" format=\"ascii\">",
            xml(name),
            strings.len()
        );
        let codes: Vec<u8> = strings
            .iter()
            .flat_map(|s| s.bytes().chain(std::iter::once(0)))
            .collect();
        ascii(&mut self.out, codes);
        self.out.push_str("</DataArray>\n");
    }
}

fn xml(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('"', "&quot;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

/// The mesh with the results of `increment` (or the mesh alone) as a VTK XML unstructured
/// grid (`.vtu`).
pub fn write_vtu(mesh: &FeMesh, increment: Option<&Increment>, time: f64, binary: bool) -> String {
    let cells = Cells::new(mesh);
    let nodes = mesh.node_count();
    let mut w = Xml {
        out: String::new(),
        binary,
    };
    w.out.push_str("<?xml version=\"1.0\"?>\n");
    w.out.push_str("<VTKFile type=\"UnstructuredGrid\" version=\"1.0\" byte_order=\"LittleEndian\" header_type=\"UInt64\"");
    if binary {
        w.out.push_str(" compressor=\"vtkZLibDataCompressor\"");
    }
    w.out.push_str(">\n<UnstructuredGrid>\n<FieldData>\n");
    for (name, value) in increment_info(increment, time) {
        w.array(&name, &[], &[value]);
    }
    let parts: Vec<String> = mesh.parts.iter().map(|p| p.name.clone()).collect();
    if !parts.is_empty() {
        w.strings("PartNames", &parts);
    }
    w.out.push_str("</FieldData>\n");
    let _ = writeln!(
        w.out,
        "<Piece NumberOfPoints=\"{nodes}\" NumberOfCells=\"{}\">",
        cells.len()
    );
    w.out.push_str("<Points>\n");
    let xyz: Vec<f64> = mesh.coords().iter().flatten().copied().collect();
    w.array("Points", &["X", "Y", "Z"].map(String::from), &xyz);
    w.out.push_str("</Points>\n<Cells>\n");
    w.array("connectivity", &[], &cells.connectivity);
    w.array("offsets", &[], &cells.offsets);
    w.array("types", &[], &cells.types);
    w.out.push_str("</Cells>\n<PointData>\n");
    let ids: Vec<i32> = mesh.node_ids().iter().map(|&id| id as i32).collect();
    w.array("NodeId", &[], &ids);
    if let Some(increment) = increment {
        for array in point_arrays(increment, nodes) {
            w.array(&array.name, &array.components, &array.values);
        }
    }
    w.out.push_str("</PointData>\n<CellData>\n");
    w.array("ElementId", &[], &cells.ids);
    w.array("PartId", &[], &cells.parts);
    w.out
        .push_str("</CellData>\n</Piece>\n</UnstructuredGrid>\n</VTKFile>\n");
    w.out
}

/// Writer of the legacy format: binary data is big-endian and followed by a line break.
struct Legacy {
    out: Vec<u8>,
    binary: bool,
}

impl Legacy {
    fn text(&mut self, text: &str) {
        self.out.extend_from_slice(text.as_bytes());
    }

    fn values<T: LeBytes + std::fmt::Display>(&mut self, values: &[T]) {
        if self.binary {
            for &value in values {
                value.put_be(&mut self.out);
            }
            self.out.push(b'\n');
        } else {
            let mut text = String::new();
            ascii(&mut text, values.iter());
            self.text(&text);
        }
    }

    /// One array of a `FIELD` block.
    fn field_array<T: LeBytes + std::fmt::Display>(
        &mut self,
        name: &str,
        components: usize,
        values: &[T],
    ) {
        let kind = match T::TYPE {
            "Float32" => "float",
            "Float64" => "double",
            "Int64" => "vtktypeint64",
            _ => "int",
        };
        let tuples = values.len() / components.max(1);
        self.text(&format!(
            "{} {components} {tuples} {kind}\n",
            legacy_name(name)
        ));
        self.values(values);
    }
}

/// The mesh with the results of `increment` (or the mesh alone) in the legacy VTK format
/// (`.vtk`, version 4.2).
pub fn write_legacy_vtk(
    mesh: &FeMesh,
    increment: Option<&Increment>,
    time: f64,
    binary: bool,
    title: &str,
) -> Vec<u8> {
    let cells = Cells::new(mesh);
    let nodes = mesh.node_count();
    let mut w = Legacy {
        out: Vec::new(),
        binary,
    };
    // The title is one line of at most 255 characters.
    let title: String = title
        .chars()
        .filter(|c| *c != '\n' && *c != '\r')
        .take(255)
        .collect();
    w.text(&format!(
        "# vtk DataFile Version 4.2\n{title}\n{}\nDATASET UNSTRUCTURED_GRID\n",
        if binary { "BINARY" } else { "ASCII" }
    ));
    let info = increment_info(increment, time);
    w.text(&format!("FIELD FieldData {}\n", info.len()));
    for (name, value) in &info {
        w.field_array(name, 1, &[*value]);
    }
    w.text(&format!("POINTS {nodes} double\n"));
    let xyz: Vec<f64> = mesh.coords().iter().flatten().copied().collect();
    w.values(&xyz);
    // Legacy cells: number of nodes followed by the nodes, for each cell.
    let mut list = Vec::with_capacity(cells.connectivity.len() + cells.len());
    let mut start = 0;
    for &end in &cells.offsets {
        list.push((end - start) as i32);
        list.extend(
            cells.connectivity[start as usize..end as usize]
                .iter()
                .map(|&i| i as i32),
        );
        start = end;
    }
    w.text(&format!("CELLS {} {}\n", cells.len(), list.len()));
    w.values(&list);
    w.text(&format!("CELL_TYPES {}\n", cells.len()));
    let types: Vec<i32> = cells.types.iter().map(|&t| i32::from(t)).collect();
    w.values(&types);
    w.text(&format!("CELL_DATA {}\nFIELD FieldData 2\n", cells.len()));
    w.field_array("ElementId", 1, &cells.ids);
    w.field_array("PartId", 1, &cells.parts);
    let arrays = increment
        .map(|i| point_arrays(i, nodes))
        .unwrap_or_default();
    w.text(&format!(
        "POINT_DATA {nodes}\nFIELD FieldData {}\n",
        arrays.len() + 1
    ));
    let ids: Vec<i32> = mesh.node_ids().iter().map(|&id| id as i32).collect();
    w.field_array("NodeId", 1, &ids);
    for array in &arrays {
        w.field_array(&array.name, array.components.len(), &array.values);
    }
    w.out
}

/// Time values of the increments of a time series. ParaView orders the files of a series by
/// time and needs distinct values: the increments' own values (time, eigenfrequency, load
/// factor) when they increase strictly, as within a static or dynamic step, otherwise the
/// position in the series (0, 1, 2, ...), e.g. for several steps whose time starts again.
/// The flag tells whether the increments' values were used.
pub fn series_times(increments: &[&Increment]) -> (Vec<f64>, bool) {
    let values: Vec<f64> = increments.iter().map(|i| i.value).collect();
    let increasing =
        values.iter().all(|v| v.is_finite()) && values.windows(2).all(|pair| pair[0] < pair[1]);
    if increasing {
        (values, true)
    } else {
        ((0..increments.len()).map(|i| i as f64).collect(), false)
    }
}

/// A ParaView Data collection (`.pvd`) of files and their time values.
pub fn write_pvd(files: &[(String, f64)]) -> String {
    let mut out = String::from(
        "<?xml version=\"1.0\"?>\n<VTKFile type=\"Collection\" version=\"0.1\" byte_order=\"LittleEndian\">\n<Collection>\n",
    );
    for (file, time) in files {
        let _ = writeln!(
            out,
            "  <DataSet timestep=\"{time:?}\" group=\"\" part=\"0\" file=\"{}\"/>",
            xml(file)
        );
    }
    out.push_str("</Collection>\n</VTKFile>\n");
    out
}

/// A ParaView JSON file series (`.vtk.series`) of files and their time values.
pub fn write_series(files: &[(String, f64)]) -> String {
    let mut out = String::from("{\n  \"file-series-version\" : \"1.0\",\n  \"files\" : [\n");
    for (i, (file, time)) in files.iter().enumerate() {
        let name = file.replace('\\', "\\\\").replace('"', "\\\"");
        let comma = if i + 1 < files.len() { "," } else { "" };
        let _ = writeln!(
            out,
            "    {{ \"name\" : \"{name}\", \"time\" : {time:?} }}{comma}"
        );
    }
    out.push_str("  ]\n}\n");
    out
}

/// What [`export`] wrote.
#[derive(Debug)]
pub struct ExportSummary {
    /// The file to open in ParaView: the single file or the collection.
    pub main: PathBuf,
    /// Number of data files.
    pub files: usize,
    /// For a series, whether its time values are the increments' values rather than their
    /// position.
    pub physical_times: Option<bool>,
}

/// Writes the mesh with the given increments. One increment (or none, the mesh alone) goes
/// to `path` itself. Several increments go to one file each in the folder `<name>` next to
/// `path`, named `<name>_0000.vtu` and so on, and `path` becomes their collection, `.pvd` or
/// `.vtk.series` (see [`VtkFormat::collection_extension`]).
pub fn export(
    path: &Path,
    format: VtkFormat,
    mesh: &FeMesh,
    increments: &[&Increment],
    binary: bool,
    title: &str,
) -> std::io::Result<ExportSummary> {
    let write = |path: &Path, increment: Option<&Increment>, time: f64| {
        let bytes = match format {
            VtkFormat::Xml => write_vtu(mesh, increment, time, binary).into_bytes(),
            VtkFormat::Legacy => write_legacy_vtk(mesh, increment, time, binary, title),
        };
        std::fs::write(path, bytes)
    };
    if increments.len() <= 1 {
        let increment = increments.first().copied();
        write(path, increment, increment.map_or(0.0, |i| i.value))?;
        return Ok(ExportSummary {
            main: path.to_path_buf(),
            files: 1,
            physical_times: None,
        });
    }
    let name = collection_stem(path, format);
    let folder = path.with_file_name(&name);
    std::fs::create_dir_all(&folder)?;
    let (times, physical) = series_times(increments);
    let mut files = Vec::with_capacity(increments.len());
    for (k, (increment, &time)) in increments.iter().zip(&times).enumerate() {
        let file = format!("{name}_{k:04}.{}", format.extension());
        write(&folder.join(&file), Some(increment), time)?;
        // Collections refer to their files relative to themselves, with forward slashes on
        // every platform.
        files.push((format!("{name}/{file}"), time));
    }
    let collection = match format {
        VtkFormat::Xml => write_pvd(&files),
        VtkFormat::Legacy => write_series(&files),
    };
    std::fs::write(path, collection)?;
    Ok(ExportSummary {
        main: path.to_path_buf(),
        files: files.len(),
        physical_times: Some(physical),
    })
}

/// The name of a collection file without its extension (`.pvd` or `.vtk.series`).
pub fn collection_stem(path: &Path, format: VtkFormat) -> String {
    let name = path
        .file_name()
        .map_or("results".into(), |n| n.to_string_lossy().into_owned());
    let suffix = format!(".{}", format.collection_extension());
    let stem = if name.len() > suffix.len() && name.to_ascii_lowercase().ends_with(&suffix) {
        &name[..name.len() - suffix.len()]
    } else {
        name.rsplit_once('.')
            .map_or(name.as_str(), |(stem, _)| stem)
    };
    if stem.is_empty() {
        "results".into()
    } else {
        stem.to_string()
    }
}

#[cfg(test)]
mod tests;
