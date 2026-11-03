//! Reader and writer for CalculiX/Abaqus input files (`.inp`).
//!
//! The reader currently takes the mesh: nodes, elements, node and element sets, and surfaces.
//! Every other keyword is skipped and reported in [`InpImport::skipped_keywords`].

mod lines;
mod write;

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

use plx_mesh::{
    Element, ElementFamily, ElementId, ElementShape, FeMesh, NodeId, Part, SurfaceDefinition,
};
use plx_model::{
    BeamOrientation, BeamProfile, BeamSection, CrossSection, Region, Section, SectionKind,
};

pub use write::{
    LowerError, Lowered, ModelError, SHORTENED_NAME, WriteError, lower, lower_unvalidated,
    lower_with_sets_of, lower_with_sets_unvalidated, resolved_overlaps, write_deformed_mesh_inp,
    write_inp, write_inp_with_sets_of,
};

use lines::{Keyword as KeywordLine, SourceLine, fields, is_keyword, parse_f64};

#[derive(Debug, thiserror::Error)]
pub enum InpError {
    #[error("{path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("{file}:{line}: {message}")]
    Parse {
        file: PathBuf,
        line: usize,
        message: String,
    },
}

#[derive(Debug, Default)]
pub struct InpImport {
    pub mesh: FeMesh,
    /// Problems that did not stop the import, ready to show to the user.
    pub warnings: Vec<String>,
    /// Keywords not read yet, with how often they occurred.
    pub skipped_keywords: BTreeMap<String, usize>,
    /// The file itself followed by every included file.
    pub files: Vec<PathBuf>,
    /// Element sets of `*ELEMENT` blocks, which become parts, as the file writes them; set
    /// and part names are upper case like CalculiX reads them.
    pub part_names: BTreeMap<String, String>,
    /// Elements of types the mesh does not hold, such as the springs and gaps that
    /// constraints are written as; skipped like their type.
    pub skipped_elements: BTreeSet<ElementId>,
    /// Shell, membrane and beam sections and the solid sections of line elements (trusses),
    /// on element sets of `mesh`, to draw the elements with their thickness or profile. They stay in
    /// `skipped_keywords`: the model does not take them over.
    pub sections: Vec<Section>,
    /// The profiles of the beam sections in `sections`, each named like its section.
    pub cross_sections: Vec<CrossSection>,
}

pub fn read_inp(path: &Path) -> Result<InpImport, InpError> {
    let mut files = Vec::new();
    let lines = lines::load(path, &mut files)?;
    Reader::new(files).run(&lines)
}

/// Whether the input, with its included files, lets surfaces touch: a contact pair or general
/// contact (`*CONTACT PAIR`, `*CONTACT INCLUSION`). Tied surfaces do not count, they cannot
/// penetrate each other.
pub fn defines_contact(path: &Path) -> Result<bool, InpError> {
    let lines = lines::load(path, &mut Vec::new())?;
    Ok(lines.iter().any(|line| {
        is_keyword(&line.text)
            && matches!(
                KeywordLine::parse(&line.text).name.as_str(),
                "CONTACTPAIR" | "CONTACTINCLUSION"
            )
    }))
}

/// Reads input given as text; `*INCLUDE` paths resolve against `base_dir`.
pub fn read_inp_str(text: &str, base_dir: Option<&Path>) -> Result<InpImport, InpError> {
    let mut files = Vec::new();
    let lines = lines::expand(text, base_dir, PathBuf::from("<text>"), &mut files)?;
    Reader::new(files).run(&lines)
}

enum Block {
    Skip,
    /// Elements of an unsupported type: only their numbers are kept. `continued` is whether
    /// the last line ended in a comma, so the next one holds more of its nodes.
    SkipElements {
        continued: bool,
    },
    Node {
        set: Option<String>,
    },
    Element {
        type_name: String,
        shape: ElementShape,
        set: String,
        pending: Vec<u32>,
        pending_line: usize,
    },
    NodeSet {
        name: String,
        generate: bool,
    },
    ElementSet {
        name: String,
        generate: bool,
    },
    Surface {
        name: String,
        by_nodes: bool,
    },
    /// A section whose data lines give the thickness of shells, the area of trusses or the
    /// profile and normal of beams; `kind` holds the parameters until they are read.
    /// `values` collects the numbers of each data line.
    Section {
        set: String,
        material: String,
        kind: SectionKind,
        /// `SECTION=` of a beam section.
        profile: String,
        values: Vec<Vec<f64>>,
        /// The comment before a general beam section with its profile, see
        /// [`PROFILE_COMMENT`].
        comment: Option<String>,
    },
}

/// The start of the comment Nodalis writes before a general `*Beam section`: CalculiX
/// knows the profile by its area and second moments only, the results draw it from this.
pub(crate) const PROFILE_COMMENT: &str = "** Profile: ";

struct Reader {
    import: InpImport,
    unsupported_elements: BTreeSet<String>,
    duplicate_nodes: usize,
    /// The last profile comment, for the beam section after it.
    profile_comment: Option<String>,
}

impl Reader {
    fn new(files: Vec<PathBuf>) -> Self {
        Self {
            import: InpImport {
                files,
                ..Default::default()
            },
            unsupported_elements: BTreeSet::new(),
            duplicate_nodes: 0,
            profile_comment: None,
        }
    }

    fn run(mut self, lines: &[SourceLine]) -> Result<InpImport, InpError> {
        let mut block = Block::Skip;
        for line in lines {
            if let Some(profile) = line.text.strip_prefix(PROFILE_COMMENT) {
                self.profile_comment = Some(profile.to_owned());
            } else if is_keyword(&line.text) {
                self.finish(&block, line)?;
                block = self.start(KeywordLine::parse(&line.text), line)?;
            } else {
                self.data(&mut block, line)?;
            }
        }
        if let Some(last) = lines.last() {
            self.finish(&block, last)?;
        }
        self.report();
        Ok(self.import)
    }

    fn error(&self, line: &SourceLine, message: impl Into<String>) -> InpError {
        InpError::Parse {
            file: self.import.files[line.file].clone(),
            line: line.number,
            message: message.into(),
        }
    }

    fn start(&mut self, keyword: KeywordLine, line: &SourceLine) -> Result<Block, InpError> {
        let required = |key: &str| {
            keyword
                .param(key)
                .filter(|v| !v.is_empty())
                .map(|v| v.to_ascii_uppercase())
                .ok_or_else(|| self.error(line, format!("*{} without {key}=", keyword.name)))
        };
        Ok(match keyword.name.as_str() {
            "NODE" => Block::Node {
                set: keyword.param("NSET").map(str::to_ascii_uppercase),
            },
            "ELEMENT" => {
                let type_name = required("TYPE")?;
                if let Some(set) = keyword.param("ELSET").filter(|s| !s.is_empty()) {
                    (self.import.part_names)
                        .entry(set.to_ascii_uppercase())
                        .or_insert_with(|| set.to_string());
                }
                match ElementShape::from_type_name(&type_name) {
                    Some(shape) => Block::Element {
                        set: keyword
                            .param("ELSET")
                            .map_or_else(|| type_name.clone(), str::to_ascii_uppercase),
                        type_name,
                        shape,
                        pending: Vec::new(),
                        pending_line: line.number,
                    },
                    None => {
                        self.unsupported_elements.insert(type_name);
                        Block::SkipElements { continued: false }
                    }
                }
            }
            "NSET" => Block::NodeSet {
                name: required("NSET")?,
                generate: keyword.has_flag("GENERATE"),
            },
            "ELSET" => Block::ElementSet {
                name: required("ELSET")?,
                generate: keyword.has_flag("GENERATE"),
            },
            "SURFACE" => Block::Surface {
                name: required("NAME")?,
                by_nodes: keyword
                    .param("TYPE")
                    .is_some_and(|t| t.eq_ignore_ascii_case("NODE")),
            },
            name => {
                let section = match name {
                    "SHELLSECTION" | "MEMBRANESECTION" => {
                        let offset = keyword.param("OFFSET").and_then(parse_f64).unwrap_or(0.0);
                        Some(if name == "SHELLSECTION" {
                            SectionKind::Shell {
                                thickness: 0.0,
                                offset,
                            }
                        } else {
                            SectionKind::Membrane {
                                thickness: 0.0,
                                offset,
                            }
                        })
                    }
                    "SOLIDSECTION" => Some(SectionKind::Truss { area: 0.0 }),
                    "BEAMSECTION" => {
                        let offset =
                            |key: &str| keyword.param(key).and_then(parse_f64).unwrap_or(0.0);
                        Some(SectionKind::Beam(BeamSection {
                            offset: [offset("OFFSET1"), offset("OFFSET2")],
                            ..BeamSection::DEFAULT
                        }))
                    }
                    _ => None,
                };
                let set = keyword.param("ELSET").filter(|s| !s.is_empty());
                let block = match (section, set) {
                    (Some(kind), Some(set)) => Block::Section {
                        set: set.to_ascii_uppercase(),
                        material: keyword.param("MATERIAL").unwrap_or_default().to_string(),
                        kind,
                        profile: keyword
                            .param("SECTION")
                            .unwrap_or_default()
                            .to_ascii_uppercase(),
                        values: Vec::new(),
                        comment: self.profile_comment.take(),
                    },
                    _ => Block::Skip,
                };
                *self
                    .import
                    .skipped_keywords
                    .entry(keyword.name)
                    .or_default() += 1;
                block
            }
        })
    }

    fn data(&mut self, block: &mut Block, line: &SourceLine) -> Result<(), InpError> {
        match block {
            Block::Skip => {}
            Block::SkipElements { continued } => {
                let text = line.text.trim_end();
                if !std::mem::replace(continued, text.ends_with(',')) {
                    let first = fields(text).next();
                    if let Some(id) = first.and_then(parse_id) {
                        self.import.skipped_elements.insert(id);
                    }
                }
            }
            Block::Node { set } => {
                let values: Vec<&str> = fields(&line.text).collect();
                let id =
                    parse_id(values[0]).ok_or_else(|| self.error(line, "Invalid node number"))?;
                let mut coords = [0.0; 3];
                for (axis, value) in values[1..].iter().take(3).enumerate() {
                    if !value.is_empty() {
                        coords[axis] = parse_f64(value).ok_or_else(|| {
                            self.error(line, format!("Invalid coordinate '{value}'"))
                        })?;
                    }
                }
                let mesh = &mut self.import.mesh;
                if mesh.set_node(id, coords) {
                    self.duplicate_nodes += 1;
                }
                if let Some(set) = set {
                    mesh.node_sets.entry(set.clone()).or_default().push(id);
                }
            }
            Block::Element {
                type_name,
                shape,
                set,
                pending,
                pending_line,
            } => {
                if pending.is_empty() {
                    *pending_line = line.number;
                }
                for value in fields(&line.text) {
                    pending
                        .push(parse_id(value).ok_or_else(|| {
                            self.error(line, format!("Invalid number '{value}'"))
                        })?);
                }
                if pending.len() > shape.node_count() + 1 {
                    return Err(self.error(
                        line,
                        format!(
                            "Element {} of type {type_name} has more than {} nodes",
                            pending[0],
                            shape.node_count()
                        ),
                    ));
                }
                if pending.len() == shape.node_count() + 1 {
                    let element = Element {
                        id: pending[0],
                        type_name: type_name.clone(),
                        shape: *shape,
                        nodes: pending[1..].to_vec(),
                    };
                    let id = element.id;
                    let added = self.import.mesh.add_element(element);
                    added.map_err(|e| self.error(line, e.to_string()))?;
                    pending.clear();
                    let mesh = &mut self.import.mesh;
                    mesh.element_sets.entry(set.clone()).or_default().push(id);
                    match mesh.parts.iter_mut().find(|p| p.name == *set) {
                        Some(part) => part.elements.push(id),
                        None => mesh.parts.push(Part {
                            name: set.clone(),
                            elements: vec![id],
                        }),
                    }
                }
            }
            Block::NodeSet { name, generate } => {
                let ids = self.set_members(line, *generate, |mesh, set| {
                    mesh.node_sets.get(set).cloned()
                })?;
                self.import
                    .mesh
                    .node_sets
                    .entry(name.clone())
                    .or_default()
                    .extend(ids);
            }
            Block::ElementSet { name, generate } => {
                let ids = self.set_members(line, *generate, |mesh, set| {
                    mesh.element_sets.get(set).cloned()
                })?;
                self.import
                    .mesh
                    .element_sets
                    .entry(name.clone())
                    .or_default()
                    .extend(ids);
            }
            Block::Surface { name, by_nodes } => self.surface_data(line, name, *by_nodes)?,
            Block::Section {
                set,
                material,
                kind,
                profile,
                values,
                comment,
            } => {
                // A beam section has the dimensions and the normal, the others one line.
                let needed = if matches!(kind, SectionKind::Beam(_)) {
                    2
                } else {
                    1
                };
                if values.len() >= needed {
                    return Ok(());
                }
                values.push(fields(&line.text).map_while(parse_f64).collect());
                if values.len() < needed {
                    return Ok(());
                }
                let mesh = &self.import.mesh;
                let elements = mesh.element_sets.get(set.as_str()).into_iter().flatten();
                let lines = (elements.filter_map(|&id| mesh.element(id)))
                    .any(|e| e.shape.family() == ElementFamily::Line);
                let read = read_section(kind, profile, values, lines, set, comment.as_deref());
                if let Some((kind, profile)) = read {
                    if let Some(profile) = profile {
                        self.import.cross_sections.push(CrossSection {
                            name: set.clone(),
                            profile,
                        });
                    }
                    self.import.sections.push(Section {
                        name: set.clone(),
                        material: material.clone(),
                        region: Region::ElementSet(set.clone()),
                        thickness: plx_model::unit_thickness(),
                        kind,
                    });
                }
            }
        }
        Ok(())
    }

    /// Ids listed on a set data line, either explicitly, as `GENERATE` ranges or via other sets.
    fn set_members(
        &mut self,
        line: &SourceLine,
        generate: bool,
        lookup: impl Fn(&FeMesh, &str) -> Option<Vec<u32>>,
    ) -> Result<Vec<u32>, InpError> {
        let values: Vec<&str> = fields(&line.text).filter(|v| !v.is_empty()).collect();
        if generate {
            let numbers = values
                .iter()
                .map(|v| parse_id(v))
                .collect::<Option<Vec<_>>>()
                .ok_or_else(|| self.error(line, "GENERATE expects numbers"))?;
            let (start, end, step) = match numbers.as_slice() {
                [start, end] => (*start, *end, 1),
                [start, end, step] if *step > 0 => (*start, *end, *step),
                _ => return Err(self.error(line, "GENERATE expects start, end[, increment]")),
            };
            return Ok((start..=end).step_by(step as usize).collect());
        }
        let mut ids = Vec::new();
        for value in values {
            if let Some(id) = parse_id(value) {
                ids.push(id);
            } else {
                match lookup(&self.import.mesh, &value.to_ascii_uppercase()) {
                    Some(members) => ids.extend(members),
                    None => self.warn(line, format!("Set '{value}' is not defined")),
                }
            }
        }
        Ok(ids)
    }

    fn surface_data(
        &mut self,
        line: &SourceLine,
        name: &str,
        by_nodes: bool,
    ) -> Result<(), InpError> {
        let values: Vec<&str> = fields(&line.text).collect();
        let mesh = &self.import.mesh;
        if by_nodes {
            let ids = match parse_id(values[0]) {
                Some(id) => vec![id],
                None => mesh
                    .node_sets
                    .get(&values[0].to_ascii_uppercase())
                    .cloned()
                    .unwrap_or_default(),
            };
            match self
                .import
                .mesh
                .surfaces
                .entry(name.to_string())
                .or_insert_with(|| SurfaceDefinition::Nodes(Vec::new()))
            {
                SurfaceDefinition::Nodes(nodes) => nodes.extend(ids),
                SurfaceDefinition::ElementFaces(_) => self.warn(
                    line,
                    format!("Surface {name} mixes nodes and element faces"),
                ),
            }
            return Ok(());
        }
        let face = values
            .get(1)
            .and_then(|label| face_number(label))
            .ok_or_else(|| self.error(line, "Element face expected, e.g. 'EALL, S2'"))?;
        let elements: Vec<ElementId> = match parse_id(values[0]) {
            Some(id) => vec![id],
            None => match mesh.element_sets.get(&values[0].to_ascii_uppercase()) {
                Some(set) => set.clone(),
                None => {
                    self.warn(line, format!("Set '{}' is not defined", values[0]));
                    Vec::new()
                }
            },
        };
        match self
            .import
            .mesh
            .surfaces
            .entry(name.to_string())
            .or_insert_with(|| SurfaceDefinition::ElementFaces(Vec::new()))
        {
            SurfaceDefinition::ElementFaces(faces) => {
                faces.extend(elements.into_iter().map(|e| (e, face)))
            }
            SurfaceDefinition::Nodes(_) => self.warn(
                line,
                format!("Surface {name} mixes nodes and element faces"),
            ),
        }
        Ok(())
    }

    fn finish(&mut self, block: &Block, at: &SourceLine) -> Result<(), InpError> {
        if let Block::Element {
            pending,
            pending_line,
            type_name,
            ..
        } = block
            && !pending.is_empty()
        {
            let line = SourceLine {
                number: *pending_line,
                ..at.clone()
            };
            return Err(self.error(
                &line,
                format!("Element {} of type {type_name} is incomplete", pending[0]),
            ));
        }
        Ok(())
    }

    fn warn(&mut self, line: &SourceLine, message: String) {
        let file = self.import.files[line.file]
            .file_name()
            .map(|f| f.to_string_lossy().into_owned())
            .unwrap_or_default();
        self.import
            .warnings
            .push(format!("{file}:{}: {message}", line.number));
    }

    fn report(&mut self) {
        let warnings = &mut self.import.warnings;
        for type_name in &self.unsupported_elements {
            warnings.push(format!(
                "Element type {type_name} is not supported yet and was skipped"
            ));
        }
        if self.duplicate_nodes > 0 {
            warnings.push(format!(
                "{} nodes were defined more than once, the last definition applies",
                self.duplicate_nodes
            ));
        }
        let missing = self.import.mesh.missing_nodes();
        if let Some((element, node)) = missing.first() {
            warnings.push(format!(
                "{} element nodes refer to missing nodes (e.g. Element {element}, Node {node})",
                missing.len()
            ));
        }
    }
}

/// The section `kind` stands for with the numbers of its data lines, if they make one;
/// solid sections count only on `lines`, as trusses.
/// The section of a section keyword with its data lines, and the profile of a beam section,
/// whose cross-section is named `set` like the section.
fn read_section(
    kind: &SectionKind,
    profile: &str,
    values: &[Vec<f64>],
    lines: bool,
    set: &str,
    comment: Option<&str>,
) -> Option<(SectionKind, Option<BeamProfile>)> {
    let first = |k: usize| values[0].get(k).copied().filter(|v| *v > 0.0);
    Some(match kind {
        &SectionKind::Shell { offset, .. } => (
            SectionKind::Shell {
                thickness: first(0)?,
                offset,
            },
            None,
        ),
        &SectionKind::Membrane { offset, .. } => (
            SectionKind::Membrane {
                thickness: first(0)?,
                offset,
            },
            None,
        ),
        // Solid sections of solid and plane elements have no area.
        SectionKind::Truss { .. } if lines => (SectionKind::Truss { area: first(0)? }, None),
        SectionKind::Beam(beam) => {
            let profile = match profile {
                "RECT" => BeamProfile::Rect {
                    a: first(0)?,
                    b: first(1)?,
                },
                // The diameter, see BeamProfile::data_line.
                "CIRC" => BeamProfile::Circ {
                    radius: first(0)? / 2.0,
                },
                "PIPE" => BeamProfile::Pipe {
                    radius: first(0)?,
                    thickness: first(1)?,
                },
                "BOX" => BeamProfile::Box {
                    a: first(0)?,
                    b: first(1)?,
                    t: [first(2)?, first(3)?, first(4)?, first(5)?],
                },
                "GENERAL" => {
                    // Nodalis's own comment names the profile and the normal it is drawn
                    // with; otherwise the values: A, I11, I12, I22, k, whose I11 is the
                    // moment of the 1-coordinate squared, see BeamProfile::data_line.
                    if let Some((profile, normal)) = comment.and_then(general_profile) {
                        let beam = SectionKind::Beam(BeamSection {
                            cross_section: set.to_owned(),
                            orientation: BeamOrientation::Direction(normal),
                            offset: beam.offset,
                        });
                        return Some((beam, Some(profile)));
                    }
                    BeamProfile::General {
                        area: first(0)?,
                        i11: first(3)?,
                        i22: first(1)?,
                        k: first(4).unwrap_or(1.0),
                    }
                }
                _ => return None,
            };
            let normal = values[1].get(..3)?;
            let beam = SectionKind::Beam(BeamSection {
                cross_section: set.to_owned(),
                orientation: BeamOrientation::Direction([normal[0], normal[1], normal[2]]),
                offset: beam.offset,
            });
            (beam, Some(profile))
        }
        _ => return None,
    })
}

/// The profile and normal of a [`PROFILE_COMMENT`], without its start.
fn general_profile(comment: &str) -> Option<(BeamProfile, [f64; 3])> {
    let (profile, normal) = comment.split_once("; Normal:")?;
    let profile: BeamProfile = ron::from_str(profile.trim()).ok()?;
    let normal: Vec<f64> = fields(normal).map_while(parse_f64).collect();
    Some((profile, normal.get(..3)?.try_into().ok()?))
}

fn parse_id(field: &str) -> Option<NodeId> {
    field.parse().ok()
}

fn face_number(label: &str) -> Option<u8> {
    let label = label.trim().to_ascii_uppercase();
    let number: u8 = label.strip_prefix('S')?.parse().ok()?;
    (1..=6).contains(&number).then_some(number)
}

#[cfg(test)]
mod tests;
