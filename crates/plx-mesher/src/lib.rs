//! Geometry import and meshing with Gmsh, which brings OpenCASCADE for STEP, IGES and BREP.
//!
//! Gmsh is not linked but loaded at run time from its shared library (see [`gmsh`]), so
//! nodalis starts and works with meshes without it. Releases ship the library next to the
//! program; `scripts/fetch_gmsh.py` fetches it for development.

mod defeature;
mod extrude;
mod ffi;
pub mod gmsh;
mod merge;
mod operation;
mod periodic;
mod revolve;
mod split;
mod stacked;
mod step_names;
mod thicken;

pub use defeature::{Defeatured, defeature, find_blends};
pub use merge::{MergedGeometry, merge_geometries};
pub use operation::{apply_cad_operation, apply_operation};

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU32, Ordering};

pub use plx_mesh::CadEntity;
use plx_mesh::{CadMap, Element, ElementId, ElementShape, FeMesh, NodeId, Part, SurfaceDefinition};
use plx_model::{
    Algorithm2d, Algorithm3d, EntitySignature, Geometry, MeshingParameters, PeriodicFaces, TagMap,
    UnitSystem,
};

use gmsh::{Gmsh, with_gmsh};
pub use gmsh::{GmshError, LibraryInfo, library_path, loaded_library, set_library_path};

/// File extensions of the CAD formats that can be imported.
pub const CAD_EXTENSIONS: [&str; 5] = ["step", "stp", "iges", "igs", "brep"];

/// Gmsh's element type numbers.
const LINE2: i32 = 1;
const LINE3: i32 = 8;
const TRI3: i32 = 2;
const QUAD4: i32 = 3;
const TET4: i32 = 4;
const TRI6: i32 = 9;
const TET10: i32 = 11;
const POINT: i32 = 15;
const QUAD8: i32 = 16;

/// Gmsh numbers the last two midside nodes of a quadratic tetrahedron the other way round
/// than CalculiX (edges 3-2, 3-1 against 2-4, 3-4 in 1-based CalculiX numbering).
const TET10_TO_CALCULIX: [usize; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 9, 8];

/// The faces, edges and vertices a Gmsh message names, such as "Invalid boundary mesh
/// (overlapping facets) on surface 23 surface 35", in the order named.
pub fn named_entities(message: &str) -> Vec<CadEntity> {
    let text = message.to_ascii_lowercase();
    let mut entities = Vec::new();
    let mut words = text.split_whitespace().peekable();
    while let Some(word) = words.next() {
        let word = word.trim_start_matches(|c: char| !c.is_ascii_alphabetic());
        let kind: fn(i32) -> CadEntity = match word {
            "surface" => CadEntity::Face,
            "curve" => CadEntity::Edge,
            "point" => CadEntity::Vertex,
            _ => continue,
        };
        // The tag follows as a whole number, maybe with a comma or bracket after it.
        let Some(next) = words.peek() else {
            break;
        };
        let digits = next.trim_end_matches([',', ')', ';', ':']);
        if let Ok(tag) = digits.parse::<i32>() {
            let entity = kind(tag);
            if !entities.contains(&entity) {
                entities.push(entity);
            }
        }
    }
    entities
}

/// Whether a file is a CAD file by its extension.
pub fn is_cad_file(path: &Path) -> bool {
    path.extension()
        .is_some_and(|e| CAD_EXTENSIONS.iter().any(|c| e.eq_ignore_ascii_case(c)))
}

/// A triangulation of the geometry for display: one part per solid, made of `S3` triangles
/// for its faces and `B31` lines for its edges.
#[derive(Clone, Debug, Default)]
pub struct GeometryDisplay {
    pub mesh: FeMesh,
    /// The CAD face or edge of each display element; element ids count from 1.
    pub entities: Vec<CadEntity>,
    /// The dimension of the CAD shape of each part of the mesh: 3 for a solid, 2 for a face,
    /// 1 for an edge.
    pub part_dimensions: Vec<u8>,
    pub solids: usize,
    pub faces: usize,
    pub edges: usize,
    /// How the CAD faces, edges and vertices meet, for selections that grow across them.
    pub topology: CadTopology,
}

/// The boundary of each CAD face and edge, by tag.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CadTopology {
    /// The edges bounding each face.
    pub face_edges: BTreeMap<i32, Vec<i32>>,
    /// The vertices at the ends of each edge; one for a closed edge.
    pub edge_vertices: BTreeMap<i32, Vec<i32>>,
}

impl GeometryDisplay {
    pub fn entity(&self, element: ElementId) -> Option<CadEntity> {
        self.entities
            .get(usize::try_from(element).ok()?.checked_sub(1)?)
            .copied()
    }

    /// The display of `smaller`, the geometry left after deleting the parts `deleted`, made
    /// from this one rather than tessellated anew, which takes long for a large assembly: the
    /// triangles and lines of the parts that stay are kept, their faces, edges and vertices
    /// renumbered by `tags` as [`delete_parts_renumbered`] gives them. `None` if any of them
    /// was not found again, or if the smaller geometry has a face or edge the display would
    /// lack; then the geometry has to be tessellated.
    pub fn without_parts(
        mut self,
        deleted: &[String],
        smaller: &Geometry,
        tags: &BTreeMap<CadEntity, CadEntity>,
    ) -> Option<GeometryDisplay> {
        let kept: Vec<bool> = (self.mesh.parts.iter())
            .map(|p| !deleted.iter().any(|d| p.name.eq_ignore_ascii_case(d)))
            .collect();
        self.mesh.remove_parts(deleted);
        let mut entities = std::mem::take(&mut self.entities);
        for element in self.mesh.elements() {
            let entity = entities.get_mut(usize::try_from(element.id).ok()?.checked_sub(1)?)?;
            *entity = *tags.get(entity)?;
        }
        let face_tag = |tag: &i32| match tags.get(&CadEntity::Face(*tag))? {
            CadEntity::Face(new) => Some(*new),
            _ => None,
        };
        let edge_tag = |tag: &i32| match tags.get(&CadEntity::Edge(*tag))? {
            CadEntity::Edge(new) => Some(*new),
            _ => None,
        };
        let vertex_tag = |tag: &i32| match tags.get(&CadEntity::Vertex(*tag))? {
            CadEntity::Vertex(new) => Some(*new),
            _ => None,
        };
        let mut topology = CadTopology::default();
        for (face, edges) in &self.topology.face_edges {
            if let Some(face) = face_tag(face) {
                let edges = edges.iter().map(edge_tag).collect::<Option<_>>()?;
                topology.face_edges.insert(face, edges);
            }
        }
        for (edge, vertices) in &self.topology.edge_vertices {
            if let Some(edge) = edge_tag(edge) {
                let vertices = vertices.iter().map(vertex_tag).collect::<Option<_>>()?;
                topology.edge_vertices.insert(edge, vertices);
            }
        }
        // Every face and edge of the smaller geometry is drawn as before.
        let (mut faces, mut edges) = (0, 0);
        for signature in &smaller.signatures {
            match signature.entity {
                CadEntity::Face(tag) if topology.face_edges.contains_key(&tag) => faces += 1,
                CadEntity::Edge(tag) if topology.edge_vertices.contains_key(&tag) => edges += 1,
                CadEntity::Face(_) | CadEntity::Edge(_) => return None,
                CadEntity::Vertex(_) => {}
            }
        }
        if faces != topology.face_edges.len() || edges != topology.edge_vertices.len() {
            return None;
        }
        let part_dimensions: Vec<u8> = (self.part_dimensions.iter().zip(&kept))
            .filter(|(_, kept)| **kept)
            .map(|(dimension, _)| *dimension)
            .collect();
        let solids_gone = (self.part_dimensions.iter().zip(&kept))
            .filter(|(dimension, kept)| **dimension == 3 && !**kept)
            .count();
        Some(GeometryDisplay {
            mesh: self.mesh,
            entities,
            part_dimensions,
            solids: self.solids.checked_sub(solids_gone)?,
            faces,
            edges,
            topology,
        })
    }
}

/// An imported CAD file: the geometry for the project and its display.
#[derive(Debug)]
pub struct CadImport {
    pub geometry: Geometry,
    pub display: GeometryDisplay,
    /// What Gmsh warned about while reading.
    pub warnings: Vec<String>,
    /// What was changed on reading, such as flat geometry moved into the x-y plane.
    pub notes: Vec<String>,
}

/// A mesh generated from the geometry.
#[derive(Debug)]
pub struct GeneratedMesh {
    pub mesh: FeMesh,
    pub warnings: Vec<String>,
}

/// Reads a STEP, IGES or BREP file in the stored SI units, see [`read_files`]. The mesh setup
/// starts with the sizes for the geometry's extent.
pub fn import_cad(path: &Path, units: UnitSystem) -> Result<CadImport, GmshError> {
    import_cad_files(&[path.to_path_buf()], units)
}

/// Reads several STEP, IGES or BREP files into one geometry, the import of
/// several selected files: their parts sit side by side, numbered on from file to file.
pub fn import_cad_files(paths: &[PathBuf], units: UnitSystem) -> Result<CadImport, GmshError> {
    if paths.is_empty() {
        return Err(GmshError::Other("No file selected".into()));
    }
    let brep_file = TempFile::new("brep");
    let (diagonal, warnings, part_names, notes, signatures) = with_gmsh(|gmsh| {
        let notes = read_files(gmsh, paths, units)?;
        let read = part_boxes(gmsh, default_parts(gmsh)?)?;
        gmsh.write(&brep_file.0)?;
        let (min, max) = gmsh.bounding_box()?;
        let diagonal = (0..3)
            .map(|k| (max[k] - min[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        let warnings = gmsh.warnings()?;
        // A BREP file keeps no names: the parts Gmsh reads from it are named after the
        // parts of the CAD files where they lie.
        gmsh.clear()?;
        gmsh.import_shapes(&brep_file.0)?;
        let part_names = names_by_place(part_boxes(gmsh, default_parts(gmsh)?)?, read);
        Ok((diagonal, warnings, part_names, notes, signatures(gmsh)?))
    })?;
    let brep = std::fs::read_to_string(&brep_file.0)
        .map_err(|e| GmshError::Other(format!("{}: {e}", brep_file.0.display())))?;
    let geometry = Geometry {
        source: file_names(paths),
        brep,
        meshing: MeshingParameters::for_diagonal(diagonal),
        mesh_items: Vec::new(),
        part_names,
        refinement: 1.0,
        signatures,
        compounds: Vec::new(),
        suppressed: Vec::new(),
    };
    let display = tessellate(&geometry)?;
    Ok(CadImport {
        geometry,
        display,
        warnings,
        notes,
    })
}

/// Reads CAD files into the open Gmsh model in the stored SI units ([`plx_model::STORED`]):
/// STEP and IGES files know their unit; BREP files have none, and their numbers are taken in
/// the length unit of `units`, the unit system of the model. Without a unit system, nothing
/// is converted and STEP and IGES files are read in millimetres.
///
/// A file whose geometry lies flat in a plane z = const away from z = 0 is moved into the
/// x-y plane, where 2D models lie; the returned notes say which.
fn read_files(gmsh: &Gmsh, paths: &[PathBuf], units: UnitSystem) -> Result<Vec<String>, GmshError> {
    let unit = if units.has_units() { "M" } else { "MM" };
    let brep_scale = units.factor_to(plx_model::Quantity::Length, plx_model::STORED);
    gmsh.set_string("Geometry.OCCTargetUnit", unit)?;
    let mut notes = Vec::new();
    let imported = paths.iter().try_for_each(|path| {
        let scale = if is_step_file(path) || is_iges_file(path) {
            1.0
        } else {
            brep_scale
        };
        gmsh.set_number("Geometry.OCCScaling", scale)?;
        let shapes = gmsh.import_shapes(path);
        gmsh.set_number("Geometry.OCCScaling", 1.0)?;
        let shapes = shapes?;
        if shapes.is_empty() {
            return Err(GmshError::Other(format!(
                "{} contains no geometry",
                path.display()
            )));
        }
        if is_step_file(path) {
            name_step_bodies(gmsh, path, &shapes)?;
        }
        if let Some(z) = move_into_xy_plane(gmsh, &shapes)? {
            let name = path
                .file_name()
                .unwrap_or(path.as_os_str())
                .to_string_lossy();
            let length = plx_model::Quantity::Length;
            let shown = (units.shown(z, length) * 1e9).round() / 1e9;
            let unit = units.unit(length);
            let at = if unit.is_empty() {
                format!("{shown}")
            } else {
                format!("{shown} {unit}")
            };
            notes.push(format!(
                "{name} lies flat in the plane z = {at}; it was moved into the x-y plane."
            ));
        }
        Ok(())
    });
    gmsh.set_string("Geometry.OCCTargetUnit", "MM")?;
    imported.map(|()| notes)
}

/// Moves shapes that lie flat in a plane z = const into the plane z = 0 and returns where
/// they lay; shapes with any extent in z, or already in the x-y plane, stay where they are.
fn move_into_xy_plane(gmsh: &Gmsh, shapes: &[Entity]) -> Result<Option<f64>, GmshError> {
    let (mut min, mut max) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for &(dim, tag) in shapes {
        let (lo, hi) = gmsh.entity_bounding_box(dim, tag)?;
        for k in 0..3 {
            min[k] = min[k].min(lo[k]);
            max[k] = max[k].max(hi[k]);
        }
    }
    let extent = (0..2).map(|k| max[k] - min[k]).fold(0.0, f64::max);
    // OpenCASCADE widens bounding boxes by its confusion tolerance, 1e-7 whatever the unit,
    // so a flat shape is 2e-7 thick.
    let flat = 1e-6 * extent + 4e-7;
    let z = 0.5 * (min[2] + max[2]);
    if !(extent > 0.0 && max[2] - min[2] <= flat && z.abs() > flat) {
        return Ok(None);
    }
    // Only the shapes no other one bounds are moved; their faces, edges and points go along.
    let mut top = Vec::new();
    for &(dim, tag) in shapes {
        if dim == 3 || gmsh.adjacencies(dim, tag)?.0.is_empty() {
            top.push((dim, tag));
        }
    }
    gmsh.translate(&top, [0.0, 0.0, -z])?;
    Ok(Some(z))
}

fn is_iges_file(path: &Path) -> bool {
    (path.extension().and_then(|e| e.to_str()))
        .is_some_and(|e| e.eq_ignore_ascii_case("iges") || e.eq_ignore_ascii_case("igs"))
}

fn is_step_file(path: &Path) -> bool {
    (path.extension().and_then(|e| e.to_str()))
        .is_some_and(|e| e.eq_ignore_ascii_case("step") || e.eq_ignore_ascii_case("stp"))
}

/// Names the solids and free faces Gmsh read from a STEP file after the bodies of the file,
/// where OpenCASCADE left them without a name: the faces of surface parts and the solids of
/// parts with several bodies. Bodies and parts are matched by their order, and only if
/// their numbers agree and every name OpenCASCADE gave is the one of its body; otherwise the
/// parts keep what Gmsh gave them.
fn name_step_bodies(gmsh: &Gmsh, path: &Path, shapes: &[Entity]) -> Result<(), GmshError> {
    let Ok(text) = std::fs::read(path) else {
        return Ok(());
    };
    let bodies = step_names::step_bodies(&String::from_utf8_lossy(&text));
    let mut volumes: Vec<i32> = (shapes.iter())
        .filter(|(dim, _)| *dim == 3)
        .map(|&(_, tag)| tag)
        .collect();
    volumes.sort_unstable();
    let free: BTreeSet<i32> = free_faces(gmsh)?.into_iter().collect();
    let mut faces: Vec<i32> = (shapes.iter())
        .filter(|&&(dim, tag)| dim == 2 && free.contains(&tag))
        .map(|&(_, tag)| tag)
        .collect();
    faces.sort_unstable();
    let solids: Vec<&str> = (bodies.iter())
        .filter(|b| b.faces.is_none())
        .map(|b| b.name.as_str())
        .collect();
    let surfaces: Vec<&str> = (bodies.iter())
        .flat_map(|b| std::iter::repeat_n(b.name.as_str(), b.faces.unwrap_or(0)))
        .collect();
    if solids.len() != volumes.len() {
        return Ok(());
    }
    let mut unnamed = Vec::new();
    for (&volume, &body) in volumes.iter().zip(&solids) {
        let label = gmsh.entity_name(3, volume)?;
        if label.is_empty() {
            unnamed.push((3, volume, body));
        } else if label_name(&label) != label_name(body) {
            log::info!("STEP names not matched: volume {volume} is {label}, body {body}");
            return Ok(());
        }
    }
    if surfaces.len() == faces.len() {
        unnamed.extend(
            faces
                .iter()
                .zip(&surfaces)
                .map(|(&face, &body)| (2, face, body)),
        );
    }
    for (dim, tag, body) in unnamed {
        if label_name(body).is_some() && gmsh.entity_name(dim, tag)?.is_empty() {
            gmsh.set_entity_name(dim, tag, &body.replace('/', "-"))?;
        }
    }
    Ok(())
}

/// The names of `parts` after those of `named` that lie where they do; the others keep
/// theirs.
fn names_by_place(
    parts: Vec<(i32, String, BoundingBox)>,
    named: Vec<(i32, String, BoundingBox)>,
) -> Vec<String> {
    let mut named: Vec<_> = named.into_iter().map(Some).collect();
    (parts.into_iter())
        .map(|(dim, name, bounds)| {
            let found = named.iter_mut().find(|n| {
                n.as_ref()
                    .is_some_and(|(d, _, b)| *d == dim && same_box(b, &bounds))
            });
            found
                .and_then(Option::take)
                .map_or(name, |(_, name, _)| name)
        })
        .collect()
}

/// The files' names, for [`Geometry::source`].
fn file_names(paths: &[PathBuf]) -> String {
    (paths.iter())
        .map(|path| {
            path.file_name().map_or_else(
                || path.display().to_string(),
                |n| n.to_string_lossy().into_owned(),
            )
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// CAD files added to a geometry by [`add_cad_files`].
#[derive(Debug)]
pub struct CadAddition {
    /// The geometry with the old and the new parts.
    pub import: CadImport,
    /// The new tag of every face, edge and vertex of the old geometry, by its old one.
    pub renumbered: BTreeMap<CadEntity, CadEntity>,
    /// Names of the new parts.
    pub added: Vec<String>,
}

/// Adds the parts of CAD files to a geometry: the import into a model that has
/// one, or into a model with a mesh but no geometry (`None`). The parts already there keep
/// their names, local mesh sizes and selections (through [`CadAddition::renumbered`]); the
/// new ones get names neither the geometry nor `taken`, e.g. the mesh parts of the model,
/// uses.
pub fn add_cad_files(
    geometry: Option<&Geometry>,
    paths: &[PathBuf],
    units: UnitSystem,
    taken: &[String],
) -> Result<CadAddition, GmshError> {
    let Some(geometry) = geometry else {
        let mut import = import_cad_files(paths, units)?;
        let mut used: BTreeSet<String> = taken.iter().cloned().collect();
        let kinds = default_part_kinds(&import.geometry)?;
        let added: Vec<String> = (kinds.iter().zip(&import.geometry.part_names))
            .map(|(&dim, name)| {
                let name = if is_default_name(name) {
                    fresh_name(dim, &used)
                } else {
                    unique_name(name, &used)
                };
                used.insert(name.clone());
                name
            })
            .collect();
        if added != import.geometry.part_names {
            import.geometry.part_names.clone_from(&added);
            import.display = tessellate(&import.geometry)?;
        }
        return Ok(CadAddition {
            import,
            renumbered: BTreeMap::new(),
            added,
        });
    };
    if paths.is_empty() {
        return Err(GmshError::Other("No file selected".into()));
    }
    let file = TempFile::with_contents("brep", &geometry.brep)?;
    let combined = TempFile::new("brep");
    let (old_parts, old_signatures, read, warnings, notes) = with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        let parts = part_boxes(gmsh, parts(gmsh, geometry)?)?;
        let old_signatures = signatures(gmsh)?;
        let notes = read_files(gmsh, paths, units)?;
        let read = part_boxes(gmsh, default_parts(gmsh)?)?;
        gmsh.write(&combined.0)?;
        Ok((parts, old_signatures, read, gmsh.warnings()?, notes))
    })?;
    let brep = std::fs::read_to_string(&combined.0)
        .map_err(|e| GmshError::Other(format!("{}: {e}", combined.0.display())))?;
    let (new_parts, new_signatures) = with_gmsh(|gmsh| {
        gmsh.import_shapes(&combined.0)?;
        Ok((part_boxes(gmsh, default_parts(gmsh)?)?, signatures(gmsh)?))
    })?;
    // The old parts are found again by where they lie, the new ones keep the names of the
    // file or are numbered on.
    let read_names = names_by_place(new_parts.clone(), read);
    let mut used: BTreeSet<String> = (old_parts.iter().map(|(_, n, _)| n.clone()))
        .chain(taken.iter().cloned())
        .collect();
    let mut old_parts: Vec<_> = old_parts.into_iter().map(Some).collect();
    let mut names = Vec::with_capacity(new_parts.len());
    let mut added = Vec::new();
    for ((dim, _, bounds), read_name) in new_parts.iter().zip(&read_names) {
        let found = old_parts.iter_mut().find(|old| {
            old.as_ref()
                .is_some_and(|(d, _, b)| d == dim && same_box(b, bounds))
        });
        if let Some((_, name, _)) = found.and_then(Option::take) {
            names.push(name);
            continue;
        }
        let name = if is_default_name(read_name) {
            fresh_name(*dim, &used)
        } else {
            unique_name(read_name, &used)
        };
        used.insert(name.clone());
        names.push(name.clone());
        added.push(name);
    }
    let renumbered = unchanged(&old_signatures, &new_signatures);
    let mut combined = Geometry {
        source: format!("{}, {}", geometry.source, file_names(paths)),
        brep,
        part_names: names,
        signatures: new_signatures,
        ..geometry.clone()
    };
    combined.renumber_mesh_items(&renumbered);
    let display = tessellate(&combined)?;
    Ok(CadAddition {
        import: CadImport {
            geometry: combined,
            display,
            warnings,
            notes,
        },
        renumbered,
        added,
    })
}

/// The geometry enlarged by `factor` about the origin, for a model whose length unit
/// changes. Faces and edges keep their tags.
pub fn scale_geometry(geometry: &Geometry, factor: f64) -> Result<Geometry, GmshError> {
    let file = TempFile::with_contents("brep", &geometry.brep)?;
    let scaled = TempFile::new("brep");
    with_gmsh(|gmsh| {
        gmsh.set_number("Geometry.OCCScaling", factor)?;
        let imported = gmsh.import_shapes(&file.0);
        gmsh.set_number("Geometry.OCCScaling", 1.0)?;
        imported?;
        gmsh.write(&scaled.0)
    })?;
    let brep = std::fs::read_to_string(&scaled.0)
        .map_err(|e| GmshError::Other(format!("{}: {e}", scaled.0.display())))?;
    with_signatures(Geometry {
        brep,
        ..geometry.clone()
    })
}

/// The geometry translated, rotated, mirrored or scaled, with lengths in metres like the
/// geometry, for the Transform block of the project schematic. Translations, rotations and
/// mirrors keep the shapes exact; the parts keep their names and the faces, edges and
/// vertices their tags.
pub fn transform_geometry(
    geometry: &Geometry,
    transform: &plx_mesh::MeshTransform,
) -> Result<Geometry, GmshError> {
    use plx_mesh::MeshTransform;
    if let Some(problem) = transform.problem() {
        return Err(GmshError::Other(problem.trim_end_matches('.').to_owned()));
    }
    let file = TempFile::with_contents("brep", &geometry.brep)?;
    let moved = TempFile::new("brep");
    with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        // Only the shapes no other one bounds are moved; their faces, edges and points go
        // along.
        let mut top = Vec::new();
        for dim in [3, 2, 1, 0] {
            for tag in gmsh.entities(dim)? {
                if dim == 3 || gmsh.adjacencies(dim, tag)?.0.is_empty() {
                    top.push((dim, tag));
                }
            }
        }
        match *transform {
            MeshTransform::Translate(by) => gmsh.translate(&top, by)?,
            MeshTransform::Rotate { point, axis, angle } => {
                gmsh.rotate(&top, point, axis, angle.to_radians())?
            }
            MeshTransform::Mirror { point, normal } => {
                let d = -(0..3).map(|k| normal[k] * point[k]).sum::<f64>();
                gmsh.mirror(&top, [normal[0], normal[1], normal[2], d])?
            }
            MeshTransform::Scale { center, factors } => gmsh.dilate(&top, center, factors)?,
        }
        gmsh.write(&moved.0)
    })?;
    let brep = std::fs::read_to_string(&moved.0)
        .map_err(|e| GmshError::Other(format!("{}: {e}", moved.0.display())))?;
    with_signatures(Geometry {
        brep,
        ..geometry.clone()
    })
}

/// Triangulates the geometry for display, finely enough that curved faces look round.
pub fn tessellate(geometry: &Geometry) -> Result<GeometryDisplay, GmshError> {
    let file = TempFile::with_contents("brep", &geometry.brep)?;
    with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        let (min, max) = gmsh.bounding_box()?;
        let diagonal = (0..3)
            .map(|k| (max[k] - min[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        // The faces take OpenCASCADE's own triangulation, which follows a curved face with
        // long thin triangles where evenly sized ones would number millions, e.g. along the
        // small fillets of a long beam. The edges are sampled to the same deflections.
        let deflection = Deflection {
            linear: diagonal * 1e-4,
            angular: std::f64::consts::TAU / 24.0,
        };
        gmsh.set_number("Mesh.StlLinearDeflection", deflection.linear)?;
        gmsh.set_number("Mesh.StlLinearDeflectionRelative", 0.0)?;
        gmsh.set_number("Mesh.StlAngularDeflection", deflection.angular)?;
        // A face that cannot be triangulated is left out of the display rather than failing
        // the import.
        if let Err(error) = gmsh.import_stl() {
            log::warn!("Geometry display incomplete: {error}");
        }
        let coords = node_coords(gmsh)?;
        let mut boundaries: BTreeMap<i32, Vec<usize>> = BTreeMap::new();
        for face in gmsh.entities(2)? {
            let triangles = gmsh.elements(TRI3, face)?.1;
            boundaries.insert(face, boundary_nodes(&triangles));
        }
        let mut polylines = BTreeMap::new();
        for curve in gmsh.entities(1)? {
            // The edge runs through the nodes the triangulation of its faces has along it,
            // so that the drawn line and the faces agree; the sampling is the fallback.
            let shared = triangulated_curve(gmsh, curve, &boundaries, &coords, deflection)
                .unwrap_or_default();
            let polyline = if shared.len() >= 2 {
                shared
            } else {
                curve_polyline(gmsh, curve, deflection)?
            };
            polylines.insert(curve, polyline);
        }
        display_mesh(gmsh, geometry, &polylines)
    })
}

/// The nodes on the border of a triangulation: those of the triangle sides that belong to
/// one triangle only.
fn boundary_nodes(triangles: &[usize]) -> Vec<usize> {
    let mut sides: HashMap<(usize, usize), u32> = HashMap::new();
    for triangle in triangles.as_chunks::<3>().0 {
        for k in 0..3 {
            let (a, b) = (triangle[k], triangle[(k + 1) % 3]);
            *sides.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    let mut nodes: Vec<usize> = sides
        .into_iter()
        .filter(|&(_, count)| count == 1)
        .flat_map(|((a, b), _)| [a, b])
        .collect();
    nodes.sort_unstable();
    nodes.dedup();
    nodes
}

/// The points of a curve taken from the border nodes of the faces it bounds, in the order
/// along the curve, ends included. Empty if the curve is not found among them.
///
/// The nodes are placed along the curve by a fine sampling of it rather than by Gmsh's
/// closest point, whose parameter is undefined for many points lying on the curve.
fn triangulated_curve(
    gmsh: &Gmsh,
    curve: i32,
    boundaries: &BTreeMap<i32, Vec<usize>>,
    coords: &HashMap<usize, [f64; 3]>,
    deflection: Deflection,
) -> Result<Vec<[f64; 3]>, GmshError> {
    let fine = Deflection {
        linear: deflection.linear * 0.1,
        angular: deflection.angular * 0.25,
    };
    let (parameters, samples) = curve_samples(gmsh, curve, fine)?;
    let (Some(&t0), Some(&t1)) = (parameters.first(), parameters.last()) else {
        return Ok(Vec::new());
    };
    // A node lies on the curve within the sampling's own deflection; nodes of neighbouring
    // edges stay much further away except at the shared ends.
    let reach = 2.0 * fine.linear;
    let (lo, hi) = gmsh.entity_bounding_box(1, curve)?;
    let near = |p: &[f64; 3]| (0..3).all(|k| p[k] >= lo[k] - reach && p[k] <= hi[k] + reach);
    let mut found: Vec<(f64, [f64; 3])> = Vec::new();
    for face in gmsh.adjacencies(1, curve)?.0 {
        let Some(nodes) = boundaries.get(&face) else {
            continue;
        };
        for tag in nodes {
            let Some(&point) = coords.get(tag).filter(|p| near(p)) else {
                continue;
            };
            if let Some((t, gap)) = polyline_parameter(&parameters, &samples, point)
                && gap <= reach
            {
                found.push((t, point));
            }
        }
    }
    if found.is_empty() {
        return Ok(Vec::new());
    }
    // The ends belong to the curve whether or not a node was found there.
    found.sort_by(|a, b| a.0.total_cmp(&b.0));
    let epsilon = (t1 - t0).abs() * 1e-9;
    found.dedup_by(|b, a| (b.0 - a.0).abs() <= epsilon);
    let ends = [samples[0], samples[samples.len() - 1]];
    let mut points: Vec<[f64; 3]> = Vec::with_capacity(found.len() + 2);
    points.push(ends[0]);
    points.extend(
        found
            .iter()
            .filter(|(t, p)| {
                *t > t0 + epsilon
                    && *t < t1 - epsilon
                    && distance(*p, ends[0]) > reach
                    && distance(*p, ends[1]) > reach
            })
            .map(|&(_, p)| p),
    );
    points.push(ends[1]);
    Ok(points)
}

/// The curve parameter of the point of a sampled curve closest to `point`, interpolated
/// along the segment, with the distance to it. None for fewer than two samples.
fn polyline_parameter(
    parameters: &[f64],
    samples: &[[f64; 3]],
    point: [f64; 3],
) -> Option<(f64, f64)> {
    let mut best: Option<(f64, f64)> = None;
    for i in 0..samples.len().saturating_sub(1) {
        let (a, b) = (samples[i], samples[i + 1]);
        let ab = sub(b, a);
        let length2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
        let ap = sub(point, a);
        let s = if length2 > 0.0 {
            ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / length2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let foot = [0, 1, 2].map(|k| a[k] + s * ab[k]);
        let gap = distance(foot, point);
        if best.is_none_or(|(_, g)| gap < g) {
            best = Some((parameters[i] + s * (parameters[i + 1] - parameters[i]), gap));
        }
    }
    best
}

/// How far a display triangulation may stray from the geometry: the distance to it, and the
/// angle between neighbouring segments of a curve.
#[derive(Clone, Copy)]
struct Deflection {
    linear: f64,
    angular: f64,
}

/// Points along a curve, split until every segment keeps within the deflection.
fn curve_polyline(
    gmsh: &Gmsh,
    curve: i32,
    deflection: Deflection,
) -> Result<Vec<[f64; 3]>, GmshError> {
    Ok(curve_samples(gmsh, curve, deflection)?.1)
}

/// The parameters and points along a curve, split until every segment keeps within the
/// deflection.
fn curve_samples(
    gmsh: &Gmsh,
    curve: i32,
    deflection: Deflection,
) -> Result<(Vec<f64>, Vec<[f64; 3]>), GmshError> {
    let (min, max) = gmsh.parameter_bounds(1, curve)?;
    let (Some(&t0), Some(&t1)) = (min.first(), max.first()) else {
        return Ok((Vec::new(), Vec::new()));
    };
    // A few segments to begin with, so that a closed curve does not start as one point.
    const START: usize = 8;
    let mut parameters: Vec<f64> = (0..=START)
        .map(|i| t0 + (t1 - t0) * i as f64 / START as f64)
        .collect();
    let mut points = gmsh.values(1, curve, &parameters)?;
    // Each round halves every segment that strays too far, until none does.
    for _ in 0..16 {
        let middles: Vec<f64> = parameters.windows(2).map(|w| 0.5 * (w[0] + w[1])).collect();
        let middle_points = gmsh.values(1, curve, &middles)?;
        let mut split = vec![false; middles.len()];
        for (i, middle) in middle_points.iter().enumerate() {
            let (a, b) = (points[i], points[i + 1]);
            let chord_middle = [0, 1, 2].map(|k| 0.5 * (a[k] + b[k]));
            // The turn between the halves of the segment.
            let turn = angle(sub(*middle, a), sub(b, *middle));
            split[i] =
                distance(*middle, chord_middle) > deflection.linear || turn > deflection.angular;
        }
        if !split.contains(&true) {
            break;
        }
        let mut new_parameters = Vec::with_capacity(parameters.len() * 2);
        let mut new_points = Vec::with_capacity(points.len() * 2);
        for i in 0..middles.len() {
            new_parameters.push(parameters[i]);
            new_points.push(points[i]);
            if split[i] {
                new_parameters.push(middles[i]);
                new_points.push(middle_points[i]);
            }
        }
        new_parameters.push(t1);
        new_points.push(points[points.len() - 1]);
        parameters = new_parameters;
        points = new_points;
    }
    Ok((parameters, points))
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = sub(a, b);
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// Angle between two vectors; zero if one of them has no length.
fn angle(a: [f64; 3], b: [f64; 3]) -> f64 {
    let (la, lb) = (distance(a, [0.0; 3]), distance(b, [0.0; 3]));
    if la == 0.0 || lb == 0.0 {
        return 0.0;
    }
    let cos = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (la * lb);
    cos.clamp(-1.0, 1.0).acos()
}

/// Meshes every part of the geometry, one after the other. Parts share no
/// nodes, line parts meeting at a point included: the contact search finds such ends and
/// ties them.
pub fn generate_mesh(geometry: &Geometry) -> Result<GeneratedMesh, GmshError> {
    let mut mesh = FeMesh::default();
    let mut warnings = Vec::new();
    for name in part_names(geometry)? {
        let part = generate_part_mesh(geometry, &name)?;
        mesh = merge_part(&mesh, part.mesh);
        warnings.extend(part.warnings);
    }
    mesh.compose_compounds(&geometry.compound_members());
    Ok(GeneratedMesh { mesh, warnings })
}

/// The names of the parts, one per solid, one per shell of faces outside the solids and one per edge
/// outside the faces, as the display and the meshes name them.
pub fn part_names(geometry: &Geometry) -> Result<Vec<String>, GmshError> {
    let file = TempFile::with_contents("brep", &geometry.brep)?;
    with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        Ok(parts(gmsh, geometry)?.into_iter().map(|(_, n)| n).collect())
    })
}

/// The parts of the geometry Gmsh read, as their shapes with the name: the solids, then the
/// shells of faces outside them, then the edges outside the faces. Names the geometry keeps
/// since a part was deleted take precedence over those Gmsh gives.
fn parts(gmsh: &Gmsh, geometry: &Geometry) -> Result<Vec<(Shape, String)>, GmshError> {
    let mut parts = default_parts(gmsh)?;
    if geometry.part_names.len() == parts.len() {
        for ((_, name), kept) in parts.iter_mut().zip(&geometry.part_names) {
            name.clone_from(kept);
        }
    }
    Ok(parts)
}

/// The faces, edges and vertices of the suppressed parts of `geometry` as read into `gmsh`,
/// which the CAD operations leave alone.
fn suppressed_entities(gmsh: &Gmsh, geometry: &Geometry) -> Result<BTreeSet<CadEntity>, GmshError> {
    let mut found = BTreeSet::new();
    if geometry.suppressed.is_empty() {
        return Ok(found);
    }
    for (shape, name) in parts(gmsh, geometry)? {
        if geometry.is_suppressed(&name) {
            found.extend(merge::closure(gmsh, shape.entities())?);
        }
    }
    Ok(found)
}

/// Refuses an operation on picked entities of suppressed parts; `noun` names what was
/// picked.
fn refuse_suppressed(
    gmsh: &Gmsh,
    geometry: &Geometry,
    entities: &[CadEntity],
    noun: &str,
) -> Result<(), GmshError> {
    let suppressed = suppressed_entities(gmsh, geometry)?;
    if entities.iter().any(|e| suppressed.contains(e)) {
        return Err(GmshError::Other(format!(
            "Some of the picked {noun} lie on suppressed parts: unsuppress the parts first"
        )));
    }
    Ok(())
}

/// The parts Gmsh read, as [`parts`], with the names Gmsh gives.
fn default_parts(gmsh: &Gmsh) -> Result<Vec<(Shape, String)>, GmshError> {
    let volumes = gmsh.entities(3)?;
    let shells = shells(gmsh, &free_faces(gmsh)?)?;
    let wires = wires(gmsh, &free_edges(gmsh)?)?;
    let shapes: Vec<Shape> = (volumes.iter().map(|&v| Shape::one(3, v)))
        .chain(shells.into_iter().map(|faces| Shape {
            dim: 2,
            tags: faces,
        }))
        .chain(wires.into_iter().map(|edges| Shape {
            dim: 1,
            tags: edges,
        }))
        .collect();
    let entities: Vec<Entity> = shapes.iter().map(Shape::first).collect();
    let names = entity_part_names(gmsh, &entities)?;
    Ok(shapes.into_iter().zip(names).collect())
}

/// The CAD entities a part is made of: one solid, one edge, or the faces of a shell, which
/// hang together along the edges they share, as the faces of a surface split in the CAD
/// program. Such a shell is meshed as a whole, with the nodes on the shared edges in common,
/// and its faces stay faces of their own to pick.
#[derive(Clone, Debug, PartialEq)]
struct Shape {
    dim: i32,
    /// The tags, in ascending order.
    tags: Vec<i32>,
}

impl Shape {
    fn one(dim: i32, tag: i32) -> Self {
        Self {
            dim,
            tags: vec![tag],
        }
    }

    /// The entity that stands for the part, e.g. for its name: the one with the lowest tag.
    fn first(&self) -> Entity {
        (self.dim, self.tags[0])
    }

    fn entities(&self) -> impl Iterator<Item = Entity> + '_ {
        self.tags.iter().map(|&tag| (self.dim, tag))
    }

    /// The bounding box of all its entities.
    fn bounding_box(&self, gmsh: &Gmsh) -> Result<BoundingBox, GmshError> {
        let (mut min, mut max) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for (dim, tag) in self.entities() {
            let (lo, hi) = gmsh.entity_bounding_box(dim, tag)?;
            for k in 0..3 {
                min[k] = min[k].min(lo[k]);
                max[k] = max[k].max(hi[k]);
            }
        }
        Ok((min, max))
    }
}

/// The faces grouped into shells: faces that share an edge belong to the same shell. The
/// shells come in the order of their lowest face tag, their faces in ascending order.
pub(crate) fn shells(gmsh: &Gmsh, faces: &[i32]) -> Result<Vec<Vec<i32>>, GmshError> {
    // Each face points at another of its shell, up to the one that stands for the shell.
    let mut parent: BTreeMap<i32, i32> = faces.iter().map(|&f| (f, f)).collect();
    fn root(parent: &BTreeMap<i32, i32>, mut face: i32) -> i32 {
        while parent[&face] != face {
            face = parent[&face];
        }
        face
    }
    let mut by_edge: BTreeMap<i32, i32> = BTreeMap::new();
    for &face in faces {
        for edge in gmsh.adjacencies(2, face)?.1 {
            let Some(&other) = by_edge.get(&edge) else {
                by_edge.insert(edge, face);
                continue;
            };
            let (a, b) = (root(&parent, face), root(&parent, other));
            if a != b {
                parent.insert(a.max(b), a.min(b));
            }
        }
    }
    let mut shells: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
    for &face in parent.keys() {
        shells.entry(root(&parent, face)).or_default().push(face);
    }
    Ok(shells.into_values().collect())
}

/// Edges that bound no face, the line parts of a geometry: wires of a STEP file that become
/// beams or trusses.
fn free_edges(gmsh: &Gmsh) -> Result<Vec<i32>, GmshError> {
    let mut in_face = BTreeSet::new();
    for face in gmsh.entities(2)? {
        in_face.extend(gmsh.adjacencies(2, face)?.1);
    }
    Ok((gmsh.entities(1)?.into_iter())
        .filter(|e| !in_face.contains(e))
        .collect())
}

/// The free edges grouped into wires: two edges that meet in a point where no other free edge
/// ends and continue each other without a kink belong together, as the pieces of a composite
/// curve that a CAD program exports as one curve (a straight piece and the arc it runs into,
/// say). Edges that meet at a corner, or where three or more meet, stay apart, as the beams of
/// a frame do. The wires come in the order of their lowest edge tag, their edges ascending.
fn wires(gmsh: &Gmsh, edges: &[i32]) -> Result<Vec<Vec<i32>>, GmshError> {
    /// The angle in which a kink still counts as a continuation: cosine of 5 degrees.
    const CONTINUES: f64 = 0.9961946980917455;
    // The end of an edge: where it lies and where it runs from there.
    struct End {
        edge: i32,
        point: [f64; 3],
        direction: [f64; 3],
    }
    let (min, max) = gmsh.bounding_box()?;
    let extent = (0..3).map(|k| max[k] - min[k]).fold(0.0, f64::max);
    let tolerance = 1e-6 * extent + 1e-9;
    let mut ends = Vec::new();
    for &edge in edges {
        let (lo, hi) = gmsh.parameter_bounds(1, edge)?;
        let (Some(&lo), Some(&hi)) = (lo.first(), hi.first()) else {
            continue;
        };
        let h = 1e-3 * (hi - lo);
        let p = gmsh.values(1, edge, &[lo, lo + h, hi - h, hi])?;
        let [a, b, c, d] = p[..] else { continue };
        ends.push(End {
            edge,
            point: a,
            direction: sub(b, a),
        });
        ends.push(End {
            edge,
            point: d,
            direction: sub(c, d),
        });
    }
    let mut parent: BTreeMap<i32, i32> = edges.iter().map(|&e| (e, e)).collect();
    fn root(parent: &BTreeMap<i32, i32>, mut edge: i32) -> i32 {
        while parent[&edge] != edge {
            edge = parent[&edge];
        }
        edge
    }
    for (k, end) in ends.iter().enumerate() {
        let mut meets = ends
            .iter()
            .enumerate()
            .filter(|(l, other)| *l != k && norm(sub(other.point, end.point)) <= tolerance);
        let (Some((_, other)), None) = (meets.next(), meets.next()) else {
            continue;
        };
        if other.edge == end.edge {
            continue;
        }
        let (a, b) = (end.direction, other.direction);
        let lengths = norm(a) * norm(b);
        if lengths > 0.0 && -(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / lengths >= CONTINUES {
            let (a, b) = (root(&parent, end.edge), root(&parent, other.edge));
            if a != b {
                parent.insert(a.max(b), a.min(b));
            }
        }
    }
    let mut wires: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
    for &edge in parent.keys() {
        wires.entry(root(&parent, edge)).or_default().push(edge);
    }
    Ok(wires.into_values().collect())
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

/// Deletes a part of the geometry; `None` once no
/// part is left. The other parts keep their names. Gmsh numbers the faces and edges anew
/// when it reads the smaller geometry, so the local mesh sizes are renumbered after where
/// the faces and edges lie; those of the deleted part are dropped.
pub fn delete_part(geometry: &Geometry, part: &str) -> Result<Option<Geometry>, GmshError> {
    delete_part_renumbered(geometry, part).map(|(geometry, _)| geometry)
}

/// [`delete_part`] with the new tag of every face, edge and vertex that stays, by its old
/// one, for the mesh and the selections on it.
pub fn delete_part_renumbered(
    geometry: &Geometry,
    part: &str,
) -> Result<(Option<Geometry>, BTreeMap<CadEntity, CadEntity>), GmshError> {
    delete_parts_renumbered(geometry, &[part.to_owned()])
}

/// [`delete_part_renumbered`] for several parts at once, as selected together: Gmsh
/// removes them all and numbers the rest anew once, so the renumbering maps the old tags
/// straight to the final ones.
pub fn delete_parts_renumbered(
    geometry: &Geometry,
    deleted: &[String],
) -> Result<(Option<Geometry>, BTreeMap<CadEntity, CadEntity>), GmshError> {
    let file = TempFile::with_contents("brep", &geometry.brep)?;
    let smaller = TempFile::new("brep");
    let (names, old_signatures) = with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        let parts = parts(gmsh, geometry)?;
        if let Some(part) = (deleted.iter()).find(|d| !parts.iter().any(|(_, n)| n == *d)) {
            return Err(GmshError::Other(format!("The geometry has no part {part}")));
        }
        // Kept with the geometry as Gmsh reads its BREP; made anew for one without them.
        let old_signatures = if geometry.signatures.is_empty() {
            signatures(gmsh)?
        } else {
            geometry.signatures.clone()
        };
        let (gone, kept): (Vec<_>, Vec<_>) =
            (parts.into_iter()).partition(|(_, n)| deleted.contains(n));
        let entities: Vec<Entity> = (gone.iter())
            .flat_map(|(shape, _)| shape.entities())
            .collect();
        gmsh.remove(&entities)?;
        // Only what stays is found again: a face or edge of a deleted part may lie exactly
        // on one that stays, where parts touch, and must not take its place.
        let mut remaining = BTreeSet::new();
        for dim in [0, 1, 2] {
            remaining
                .extend((gmsh.entities(dim)?.into_iter()).filter_map(|t| CadEntity::of(dim, t)));
        }
        let old_signatures: Vec<EntitySignature> = (old_signatures.into_iter())
            .filter(|s| remaining.contains(&s.entity))
            .collect();
        gmsh.write(&smaller.0)?;
        Ok((
            kept.into_iter().map(|(_, n)| n).collect::<Vec<_>>(),
            old_signatures,
        ))
    })?;
    if names.is_empty() {
        return Ok((None, BTreeMap::new()));
    }
    let brep = std::fs::read_to_string(&smaller.0)
        .map_err(|e| GmshError::Other(format!("{}: {e}", smaller.0.display())))?;
    let new_signatures = with_gmsh(|gmsh| {
        gmsh.import_shapes(&smaller.0)?;
        signatures(gmsh)
    })?;
    let renumbered = unchanged(&old_signatures, &new_signatures);
    let mut smaller = Geometry {
        brep,
        part_names: names,
        signatures: new_signatures,
        ..geometry.clone()
    };
    smaller.renumber_mesh_items(&renumbered);
    smaller.remove_from_compounds(deleted);
    smaller.set_suppressed(deleted, false);
    Ok((Some(smaller), renumbered))
}

/// A Gmsh entity as (dimension, tag).
type Entity = (i32, i32);

type BoundingBox = ([f64; 3], [f64; 3]);

/// What every face, edge and vertex of the model looks like, to find them again once Gmsh
/// numbered them anew, see [`plx_model::match_entities`].
fn signatures(gmsh: &Gmsh) -> Result<Vec<EntitySignature>, GmshError> {
    let mut signatures = Vec::new();
    for dim in [0, 1, 2] {
        for tag in gmsh.entities(dim)? {
            let Some(entity) = CadEntity::of(dim, tag) else {
                continue;
            };
            let (min, max) = gmsh.entity_bounding_box(dim, tag)?;
            let (centroid, size) = if dim == 0 {
                (gmsh.point(tag)?, 0.0)
            } else {
                (gmsh.center_of_mass(dim, tag)?, gmsh.mass(dim, tag)?)
            };
            signatures.push(EntitySignature {
                entity,
                kind: gmsh.entity_type(dim, tag)?,
                centroid,
                min,
                max,
                size,
            });
        }
    }
    Ok(signatures)
}

/// The geometry with the signatures of its entities as Gmsh reads its BREP, for a geometry
/// made without them.
pub fn with_signatures(geometry: Geometry) -> Result<Geometry, GmshError> {
    let file = TempFile::with_contents("brep", &geometry.brep)?;
    let signatures = with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        signatures(gmsh)
    })?;
    Ok(Geometry {
        signatures,
        ..geometry
    })
}

/// Whether a part name is a numbered default, as "SOLID-2", rather than one from the file.
fn is_default_name(name: &str) -> bool {
    ["SOLID-", "SHELL-", "LINE-"].iter().any(|prefix| {
        name.strip_prefix(prefix)
            .is_some_and(|n| !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()))
    })
}

/// `name`, or with "-2", "-3" … appended if it is used.
fn unique_name(name: &str, used: &BTreeSet<String>) -> String {
    (1..)
        .map(|n| {
            if n == 1 {
                name.to_owned()
            } else {
                format!("{name}-{n}")
            }
        })
        .find(|candidate| !used.contains(candidate))
        .unwrap_or_default()
}

/// The first "SOLID-n", "SHELL-n" or "LINE-n" for a part of dimension `dim` not yet used.
fn fresh_name(dim: i32, used: &BTreeSet<String>) -> String {
    let prefix = match dim {
        3 => "SOLID",
        2 => "SHELL",
        _ => "LINE",
    };
    (1..)
        .map(|n| format!("{prefix}-{n}"))
        .find(|name| !used.contains(name))
        .unwrap_or_default()
}

/// The dimension of each part of the geometry, in the order of [`parts`].
fn default_part_kinds(geometry: &Geometry) -> Result<Vec<i32>, GmshError> {
    let file = TempFile::with_contents("brep", &geometry.brep)?;
    with_gmsh(|gmsh| {
        gmsh.import_shapes(&file.0)?;
        Ok((default_parts(gmsh)?.into_iter())
            .map(|(shape, _)| shape.dim)
            .collect())
    })
}

/// Dimension and bounding box of every part, as [`parts`] lists them.
fn part_boxes(
    gmsh: &Gmsh,
    parts: Vec<(Shape, String)>,
) -> Result<Vec<(i32, String, BoundingBox)>, GmshError> {
    (parts.into_iter())
        .map(|(shape, name)| Ok((shape.dim, name, shape.bounding_box(gmsh)?)))
        .collect()
}

/// The new tags of the entities that a geometry kept unchanged when parts were added or
/// deleted; the others are left out.
fn unchanged(old: &[EntitySignature], new: &[EntitySignature]) -> TagMap {
    let mut map = plx_model::match_entities(old, new);
    // Only the entities that are still there exactly: those of a deleted part must not be
    // taken for a similar one that stays.
    let by_entity: BTreeMap<CadEntity, &EntitySignature> =
        new.iter().map(|s| (s.entity, s)).collect();
    map.retain(|from, to| {
        let (Some(a), Some(b)) = (old.iter().find(|s| s.entity == *from), by_entity.get(to)) else {
            return false;
        };
        let size = (0..3).map(|k| a.max[k] - a.min[k]).fold(0.0, f64::max);
        let tolerance = 1e-6 * size.max(1e-3);
        a.kind == b.kind
            && (0..3).all(|k| {
                (a.min[k] - b.min[k]).abs() <= tolerance && (a.max[k] - b.max[k]).abs() <= tolerance
            })
    });
    map
}

/// Whether two bounding boxes are those of the same entity, read twice.
fn same_box(a: &BoundingBox, b: &BoundingBox) -> bool {
    let size = (0..3).map(|k| a.1[k] - a.0[k]).fold(0.0, f64::max);
    let tolerance = 1e-6 * size.max(1e-3);
    (0..3).all(|k| (a.0[k] - b.0[k]).abs() <= tolerance && (a.1[k] - b.1[k]).abs() <= tolerance)
}

/// Faces that bound no solid, the shell parts of a geometry such as a 2D cross-section.
fn free_faces(gmsh: &Gmsh) -> Result<Vec<i32>, GmshError> {
    let mut in_solid = BTreeSet::new();
    for volume in gmsh.entities(3)? {
        in_solid.extend(gmsh.adjacencies(3, volume)?.1);
    }
    Ok((gmsh.entities(2)?.into_iter())
        .filter(|f| !in_solid.contains(f))
        .collect())
}

/// Meshes one part with tetrahedra after its meshing parameters, Gmsh algorithms and local
/// mesh sizes. The mesh holds the one part; its numbers count from 1, see [`merge_part`].
pub fn generate_part_mesh(geometry: &Geometry, part: &str) -> Result<GeneratedMesh, GmshError> {
    generate_part_mesh_matching(geometry, part, &[])
}

/// [`generate_part_mesh`] with the cut faces of cyclic symmetries meshed alike, see
/// [`PeriodicFaces`]: the faces of the part that one sector's rotation takes on to each other.
pub fn generate_part_mesh_matching(
    geometry: &Geometry,
    part: &str,
    matching: &[PeriodicFaces],
) -> Result<GeneratedMesh, GmshError> {
    let setup = geometry.parameters(part);
    if !(setup.max_size > 0.0 && setup.min_size >= 0.0) {
        return Err(GmshError::Other(format!(
            "{part}: The max element size must be greater than 0"
        )));
    }
    let file = TempFile::with_contents("brep", &geometry.brep)?;
    with_gmsh(|gmsh| {
        // The sweeps tried in turn, for a part whose meshing parameters ask for hexahedra
        // or whose extrude mesh names no source faces. Each attempt may take the solid out
        // of the model, so each reads the geometry anew.
        let chosen = geometry.extrusion(part);
        let automatic = chosen.is_none() && geometry.finds_hexahedra(part);
        let found = plx_model::ExtrudeMesh {
            parts: vec![part.to_string()],
            ..Default::default()
        };
        let unsourced = chosen.filter(|extrude| extrude.faces.is_empty());
        let mut refused = Vec::new();
        if let Some(extrude) = unsourced.or(automatic.then_some(&found)) {
            for way in [Sweep::Stacked, Sweep::Extruded, Sweep::Revolved] {
                if way == Sweep::Revolved && !automatic {
                    continue;
                }
                let Some(volume) = solid_alone(gmsh, geometry, &file.0, part)? else {
                    gmsh.clear()?;
                    break;
                };
                let result = match way {
                    Sweep::Stacked => {
                        stacked::stacked_part_mesh(gmsh, geometry, part, volume, extrude, matching)?
                    }
                    Sweep::Extruded => {
                        extrude::extrude_part_mesh(gmsh, geometry, part, volume, extrude, matching)?
                    }
                    Sweep::Revolved => revolve::revolve_part_mesh(gmsh, geometry, part, volume)?,
                };
                match result {
                    Ok(mesh) => return Ok(mesh),
                    Err(reason) => {
                        // An extrude mesh is refused by what it was last tried as.
                        if unsourced.is_some() && way == Sweep::Extruded {
                            return Err(GmshError::Other(reason));
                        }
                        // The stack and the body of revolution say what stands against
                        // them; the plain extrusion is a stack of one.
                        if way != Sweep::Extruded {
                            refused.push(reason);
                        }
                        gmsh.clear()?;
                    }
                }
            }
        }
        gmsh.import_shapes(&file.0)?;
        let volumes = gmsh.entities(3)?;
        let Some(shape) = (parts(gmsh, geometry)?.into_iter())
            .find(|(_, n)| n == part)
            .map(|(shape, _)| shape)
        else {
            return Err(GmshError::Other(format!("The geometry has no part {part}")));
        };
        let volume = match shape.dim {
            3 => shape.tags[0],
            2 => return shell_part_mesh(gmsh, geometry, part, &shape.tags),
            _ => return line_part_mesh(gmsh, geometry, part, &shape.tags),
        };
        // The other solids go, so that only this one is meshed.
        let others: Vec<(i32, i32)> = (volumes.iter())
            .filter(|&&v| v != volume)
            .map(|&v| (3, v))
            .collect();
        if !others.is_empty() {
            gmsh.remove(&others)?;
        }
        if let Some(extrude) = chosen {
            return extrude::extrude_part_mesh(gmsh, geometry, part, volume, extrude, matching)?
                .map_err(GmshError::Other);
        }
        let mut options = MeshOptions::of_part(geometry, part);
        // Recombined boundary faces would make Gmsh fill the solid with pyramids next to
        // them, which neither the solver nor the tetrahedra read here take: solids get
        // triangles on their faces whatever the quad-dominated option says.
        options.quads = false;
        options.apply(gmsh)?;
        let faces: BTreeSet<i32> = gmsh.adjacencies(3, volume)?.1.into_iter().collect();
        size_fields(gmsh, &local_sizes(gmsh, geometry, &faces)?, 1.0)?;
        let rotations = periodic::rotations(gmsh, matching)?;
        let faces_list: Vec<i32> = faces.iter().copied().collect();
        periodic::match_rotated(gmsh, 2, &faces_list, &rotations)?;
        gmsh.generate(3)?;
        let order = options.order;
        let mesh = solid_mesh(gmsh, order, &[(volume, part.to_string())])?;
        let mut warnings = gmsh.warnings()?;
        // Why the part got tetrahedra although hexahedra were asked for.
        if !refused.is_empty() {
            let prefix = format!("{part}: ");
            let reasons: Vec<&str> = (refused.iter())
                .map(|reason| reason.strip_prefix(&prefix).unwrap_or(reason))
                .collect();
            warnings.push(format!(
                "{part}: meshed with tetrahedra ({})",
                reasons.join("; ")
            ));
        }
        Ok(GeneratedMesh { mesh, warnings })
    })
}

/// The ways a solid is swept into hexahedra, in the order they are tried.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Sweep {
    /// A stack of extrusions along one direction, see [`stacked`].
    Stacked,
    /// A straight extrusion of faces that need not be planar, see [`extrude`].
    Extruded,
    /// A body of revolution, see [`revolve`].
    Revolved,
}

/// Reads the geometry and leaves only the part's solid; `None` if the part is no solid.
fn solid_alone(
    gmsh: &Gmsh,
    geometry: &Geometry,
    file: &Path,
    part: &str,
) -> Result<Option<i32>, GmshError> {
    gmsh.import_shapes(file)?;
    let Some((Shape { dim: 3, tags }, _)) =
        (parts(gmsh, geometry)?.into_iter()).find(|(_, n)| n == part)
    else {
        return Ok(None);
    };
    let volume = tags[0];
    let others: Vec<(i32, i32)> = (gmsh.entities(3)?.into_iter())
        .filter(|&v| v != volume)
        .map(|v| (3, v))
        .collect();
    if !others.is_empty() {
        gmsh.remove(&others)?;
    }
    Ok(Some(volume))
}

/// Meshes the faces of a shell outside the solids with triangles, or mostly quadrilaterals if
/// its meshing parameters ask for them, typed as shells (`S3`, `S6`, `S4`, `S8`); a 2D model
/// gives them its own types. Faces that share an edge share its nodes. The mesh holds the
/// one part, numbered from 1.
fn shell_part_mesh(
    gmsh: &Gmsh,
    geometry: &Geometry,
    part: &str,
    shell: &[i32],
) -> Result<GeneratedMesh, GmshError> {
    let faces = free_faces(gmsh)?;
    // Only the shell stays, so that only it is meshed.
    let others: Vec<(i32, i32)> = (gmsh.entities(3)?.into_iter().map(|v| (3, v)))
        .chain(faces.iter().filter(|f| !shell.contains(f)).map(|&f| (2, f)))
        .chain(free_edges(gmsh)?.into_iter().map(|e| (1, e)))
        .collect();
    if !others.is_empty() {
        gmsh.remove(&others)?;
    }
    let mut options = MeshOptions::of_faces(geometry, part);
    let mut notes = Vec::new();
    let sizes = local_sizes(gmsh, geometry, &shell.iter().copied().collect())?;
    generate_faces(gmsh, &mut options, shell, &sizes, &mut notes)?;
    let coords = node_coords(gmsh)?;
    let types: [(i32, ElementShape, &str); 2] = if options.order == 2 {
        [
            (TRI6, ElementShape::Tri6, "S6"),
            (QUAD8, ElementShape::Quad8, "S8"),
        ]
    } else {
        [
            (TRI3, ElementShape::Tri3, "S3"),
            (QUAD4, ElementShape::Quad4, "S4"),
        ]
    };
    let mut mesh = FeMesh::default();
    let mut part_elements = Part {
        bodies: Vec::new(),
        name: part.to_string(),
        elements: Vec::new(),
    };
    let kinds = shell
        .iter()
        .flat_map(|&face| types.map(|kind| (face, kind)));
    for (face, (gmsh_type, shape, type_name)) in kinds {
        let (_, nodes) = gmsh.elements(gmsh_type, face)?;
        // Gmsh numbers the nodes of triangles and quadrilaterals as CalculiX does.
        for element in nodes.chunks_exact(shape.node_count()) {
            let mut ids = Vec::with_capacity(element.len());
            for &tag in element {
                let position = coords
                    .get(&tag)
                    .ok_or_else(|| GmshError::Other(format!("Node {tag} is missing")))?;
                let id = node_id(tag)?;
                mesh.set_node(id, *position);
                ids.push(id);
            }
            let id = ElementId::try_from(mesh.element_count() + 1)
                .map_err(|_| GmshError::Other("too many elements".into()))?;
            mesh.add_element(Element {
                id,
                type_name: type_name.into(),
                shape,
                nodes: ids,
            })
            .map_err(|e| GmshError::Other(e.to_string()))?;
            part_elements.elements.push(id);
        }
    }
    if part_elements.elements.is_empty() {
        return Err(GmshError::Other("Gmsh created no elements".into()));
    }
    mesh.parts.push(part_elements);
    mesh.cad = cad_map(gmsh, &mesh, shell, options.order, true)?;
    if let Some(thickening) = geometry.thickening(part) {
        mesh = thicken::thicken(&mesh, part, thickening)
            .map_err(|reason| GmshError::Other(format!("{part}: {reason}")))?;
    }
    let mut warnings = gmsh.warnings()?;
    warnings.extend(notes.into_iter().map(|note| format!("{part}: {note}")));
    Ok(GeneratedMesh { mesh, warnings })
}

/// Meshes an edge outside the faces with 2- or 3-node lines, typed as beams (`B31`, `B32`);
/// the section decides what the solver makes of them. The mesh holds the one part, numbered
/// from 1.
fn line_part_mesh(
    gmsh: &Gmsh,
    geometry: &Geometry,
    part: &str,
    wire: &[i32],
) -> Result<GeneratedMesh, GmshError> {
    let edges = free_edges(gmsh)?;
    // Only the edges of the wire stay, so that only they are meshed.
    let others: Vec<(i32, i32)> = (gmsh.entities(3)?.into_iter().map(|v| (3, v)))
        .chain(free_faces(gmsh)?.into_iter().map(|f| (2, f)))
        .chain(edges.iter().filter(|e| !wire.contains(e)).map(|&e| (1, e)))
        .collect();
    if !others.is_empty() {
        gmsh.remove(&others)?;
    }
    if wire.len() > 1 {
        // The pieces of a wire each come with a vertex of their own at the joints from some
        // CAD programs; one vertex lets them share the node there.
        gmsh.remove_all_duplicates()?;
    }
    let options = MeshOptions::of_part(geometry, part);
    options.apply(gmsh)?;
    for (_, local_edges, size, _) in geometry.local_sizes() {
        let on_wire: Vec<f64> = (wire.iter())
            .filter(|e| local_edges.contains(e))
            .map(|&e| f64::from(e))
            .collect();
        if !on_wire.is_empty() && size.is_finite() && size > 0.0 {
            let field = gmsh.add_field("Constant")?;
            gmsh.set_field_number(field, "VIn", size)?;
            gmsh.set_field_number(field, "VOut", 1e22)?;
            gmsh.set_field_number(field, "IncludeBoundary", 1.0)?;
            gmsh.set_field_numbers(field, "CurvesList", &on_wire)?;
            gmsh.set_background_field(field)?;
        }
    }
    gmsh.generate(1)?;
    let coords = node_coords(gmsh)?;
    let (gmsh_type, shape, type_name) = if options.order == 2 {
        (LINE3, ElementShape::Line3, "B32")
    } else {
        (LINE2, ElementShape::Line2, "B31")
    };
    let mut mesh = FeMesh::default();
    let mut part_elements = Part {
        bodies: Vec::new(),
        name: part.to_string(),
        elements: Vec::new(),
    };
    let mut edge_elements: Vec<(i32, Vec<ElementId>)> = Vec::new();
    for &edge in wire {
        let first = part_elements.elements.len();
        let (_, nodes) = gmsh.elements(gmsh_type, edge)?;
        for element in nodes.chunks_exact(shape.node_count()) {
            // Gmsh lists the end nodes first and the midside node last; the solver wants the
            // midside node in the middle.
            let order: &[usize] = if shape == ElementShape::Line3 {
                &[0, 2, 1]
            } else {
                &[0, 1]
            };
            let mut ids = Vec::with_capacity(element.len());
            for &k in order {
                let tag = element[k];
                let position = coords
                    .get(&tag)
                    .ok_or_else(|| GmshError::Other(format!("Node {tag} is missing")))?;
                let id = node_id(tag)?;
                mesh.set_node(id, *position);
                ids.push(id);
            }
            let id = ElementId::try_from(mesh.element_count() + 1)
                .map_err(|_| GmshError::Other("too many elements".into()))?;
            mesh.add_element(Element {
                id,
                type_name: type_name.into(),
                shape,
                nodes: ids,
            })
            .map_err(|e| GmshError::Other(e.to_string()))?;
            part_elements.elements.push(id);
        }
        edge_elements.push((edge, part_elements.elements[first..].to_vec()));
    }
    if part_elements.elements.is_empty() {
        return Err(GmshError::Other("Gmsh created no elements".into()));
    }
    mesh.parts.push(part_elements);
    mesh.cad = line_cad_map(gmsh, &mesh, &edge_elements)?;
    Ok(GeneratedMesh {
        mesh,
        warnings: gmsh.warnings()?,
    })
}

/// Where the CAD edges of a line part and their vertices lie in the mesh: the nodes and
/// segments of every edge, and the node at each end vertex.
fn line_cad_map(
    gmsh: &Gmsh,
    mesh: &FeMesh,
    edges: &[(i32, Vec<ElementId>)],
) -> Result<CadMap, GmshError> {
    let mut map = CadMap::default();
    for (edge, elements) in edges {
        let mut nodes = BTreeSet::new();
        let mut segments = Vec::new();
        for element in elements.iter().filter_map(|&id| mesh.element(id)) {
            nodes.extend(element.nodes.iter().copied());
            segments.push([element.nodes[0], element.nodes[element.nodes.len() - 1]]);
        }
        map.nodes
            .insert(CadEntity::Edge(*edge), nodes.into_iter().collect());
        map.segments.insert(*edge, segments);
        for vertex in gmsh.adjacencies(1, *edge)?.1 {
            let (_, tags) = gmsh.elements(POINT, vertex)?;
            let nodes: Vec<NodeId> = tags.iter().map(|&t| node_id(t)).collect::<Result<_, _>>()?;
            if !nodes.is_empty() {
                map.nodes.insert(CadEntity::Vertex(vertex), nodes);
            }
        }
    }
    Ok(map)
}

/// A local mesh size on entities of the Gmsh model: their tags, as Gmsh's fields take them.
pub(crate) struct LocalSize {
    pub faces: Vec<f64>,
    pub edges: Vec<f64>,
    pub size: f64,
    /// The depth below the faces to which the size holds, 0 for the faces alone.
    pub depth: f64,
}

/// The local mesh sizes of the mesh setup on the given faces and their edges.
fn local_sizes(
    gmsh: &Gmsh,
    geometry: &Geometry,
    faces: &BTreeSet<i32>,
) -> Result<Vec<LocalSize>, GmshError> {
    let mut edges = BTreeSet::new();
    for &face in faces {
        edges.extend(gmsh.adjacencies(2, face)?.1);
    }
    let own = |tags: &[i32], own: &BTreeSet<i32>| -> Vec<f64> {
        (tags.iter())
            .filter(|t| own.contains(t))
            .map(|&t| f64::from(t))
            .collect()
    };
    Ok(geometry
        .local_sizes()
        .map(|(local_faces, local_edges, size, depth)| LocalSize {
            faces: own(local_faces, faces),
            edges: own(local_edges, &edges),
            size,
            depth,
        })
        .collect())
}

/// Makes the local mesh sizes Gmsh size fields; Gmsh takes the smallest of them and the
/// other size limits. The sizes are multiplied by the factor, see
/// [`MeshOptions::size_factor`]. Returns the fields made, to take them back with.
fn size_fields(gmsh: &Gmsh, sizes: &[LocalSize], factor: f64) -> Result<Vec<i32>, GmshError> {
    let mut fields = Vec::new();
    // The fields of sizes, which the background field takes the smallest of; `fields`
    // also has the distances they are made from.
    let mut size_list = Vec::new();
    for local in sizes {
        if local.size.is_nan()
            || local.size <= 0.0
            || (local.faces.is_empty() && local.edges.is_empty())
        {
            continue;
        }
        let field = gmsh.add_field("Constant")?;
        if !local.faces.is_empty() {
            gmsh.set_field_numbers(field, "SurfacesList", &local.faces)?;
        }
        if !local.edges.is_empty() {
            gmsh.set_field_numbers(field, "CurvesList", &local.edges)?;
        }
        gmsh.set_field_number(field, "VIn", local.size * factor)?;
        gmsh.set_field_number(field, "VOut", 1e22)?;
        gmsh.set_field_number(field, "IncludeBoundary", 1.0)?;
        fields.push(field);
        size_list.push(field);
        // Within the depth below the faces and around the edges the size holds in the
        // volume as well: the distance to them, cut off at the depth.
        if local.depth > 0.0 {
            let distance = gmsh.add_field("Distance")?;
            if !local.faces.is_empty() {
                gmsh.set_field_numbers(distance, "SurfacesList", &local.faces)?;
            }
            if !local.edges.is_empty() {
                gmsh.set_field_numbers(distance, "CurvesList", &local.edges)?;
            }
            gmsh.set_field_number(distance, "Sampling", distance_sampling(gmsh, local)?)?;
            fields.push(distance);
            let threshold = gmsh.add_field("Threshold")?;
            gmsh.set_field_number(threshold, "InField", f64::from(distance))?;
            gmsh.set_field_number(threshold, "SizeMin", local.size * factor)?;
            gmsh.set_field_number(threshold, "SizeMax", 1e22)?;
            gmsh.set_field_number(threshold, "DistMin", local.depth)?;
            // A little beyond the depth, so that the threshold does not divide by zero.
            gmsh.set_field_number(threshold, "DistMax", local.depth * (1.0 + 1e-6))?;
            gmsh.set_field_number(threshold, "StopAtDistMax", 1.0)?;
            fields.push(threshold);
            size_list.push(threshold);
        }
    }
    let background = match size_list.as_slice() {
        [] => return Ok(fields),
        [field] => *field,
        _ => {
            let min = gmsh.add_field("Min")?;
            let list: Vec<f64> = size_list.iter().map(|&f| f64::from(f)).collect();
            gmsh.set_field_numbers(min, "FieldsList", &list)?;
            fields.push(min);
            min
        }
    };
    gmsh.set_background_field(background)?;
    Ok(fields)
}

/// The points per direction a distance field samples the faces and edges of a local size
/// by: enough for a tenth of the depth between them on the largest, within limits that keep
/// the field fast.
fn distance_sampling(gmsh: &Gmsh, local: &LocalSize) -> Result<f64, GmshError> {
    let mut extent = 0.0_f64;
    let entities = (local.faces.iter().map(|&f| (2, f))).chain(local.edges.iter().map(|&e| (1, e)));
    for (dim, tag) in entities {
        let (lo, hi) = gmsh.entity_bounding_box(dim, tag as i32)?;
        for k in 0..3 {
            extent = extent.max(hi[k] - lo[k]);
        }
    }
    Ok((extent / (0.1 * local.depth)).clamp(20.0, 400.0).ceil())
}

/// Faces the quasi-structured algorithm is expected to give more quadrilaterals than this
/// are left to the faster Frontal-Delaunay algorithm: its time grows faster than the mesh,
/// to minutes beyond.
const QUASI_STRUCTURED_LIMIT: f64 = 20_000.0;

/// A quadrilateral whose scaled Jacobian is below this at a corner is badly distorted; a
/// square has 1, a quadrilateral folded over 0 or less.
const DISTORTED_QUAD: f64 = 0.1;

/// An element along a curve longer than this many max element sizes means the mesher did
/// not follow the sizes: local sizes and curvature only ever make elements smaller.
const COARSE_CURVE: f64 = 3.0;

/// Whether a curve of the faces got elements far longer than the max element size: the
/// quasi-structured algorithm does that at times, e.g. on a long face meshed alone among
/// other entities, and leaves square elements that look well shaped but are far too coarse.
fn too_coarse(gmsh: &Gmsh, faces: &[i32], max_size: f64) -> Result<bool, GmshError> {
    let mut curves = BTreeSet::new();
    for &face in faces {
        curves.extend(gmsh.adjacencies(2, face)?.1.into_iter().map(i32::abs));
    }
    for curve in curves {
        let mut count = 0;
        for (gmsh_type, nodes) in [(LINE2, 2), (LINE3, 3)] {
            count += gmsh.elements(gmsh_type, curve)?.1.len() / nodes;
        }
        if count > 0 && gmsh.mass(1, curve)? / count as f64 > COARSE_CURVE * max_size {
            return Ok(true);
        }
    }
    Ok(false)
}

/// The smallest scaled Jacobian at a corner of the quadrilaterals on the faces, `None`
/// without quadrilaterals.
fn worst_quad(gmsh: &Gmsh, faces: &[i32], order: i32) -> Result<Option<f64>, GmshError> {
    use extrude::{cross, dot, norm, sub};
    let coords = node_coords(gmsh)?;
    let (gmsh_type, count) = if order == 2 { (QUAD8, 8) } else { (QUAD4, 4) };
    let mut worst: Option<f64> = None;
    for &face in faces {
        let (_, nodes) = gmsh.elements(gmsh_type, face)?;
        for element in nodes.chunks_exact(count) {
            let mut p = [[0.0; 3]; 4];
            for (corner, tag) in p.iter_mut().zip(element) {
                *corner = *coords
                    .get(tag)
                    .ok_or_else(|| GmshError::Other(format!("Node {tag} is missing")))?;
            }
            let normal = cross(sub(p[2], p[0]), sub(p[3], p[1]));
            let length = norm(normal);
            let mut quality = f64::INFINITY;
            for i in 0..4 {
                let (a, b) = (sub(p[(i + 1) % 4], p[i]), sub(p[(i + 3) % 4], p[i]));
                let scale = norm(a) * norm(b) * length;
                quality = quality.min(if scale > 0.0 {
                    dot(cross(a, b), normal) / scale
                } else {
                    0.0
                });
            }
            worst = Some(worst.map_or(quality, |w| w.min(quality)));
        }
    }
    Ok(worst)
}

/// Meshes the faces left in the Gmsh model after the options and the local mesh sizes. The quasi-structured algorithm is
/// watched: where it is expected to take too long, fails, leaves a badly distorted
/// quadrilateral or elements far coarser than asked for, the faces are meshed with
/// Frontal-Delaunay and recombined instead, unless a distorted mesh comes out worse that
/// way. The notes say so, for the user.
fn generate_faces(
    gmsh: &Gmsh,
    options: &mut MeshOptions,
    faces: &[i32],
    sizes: &[LocalSize],
    notes: &mut Vec<String>,
) -> Result<(), GmshError> {
    if options.quads || options.algorithms.0 == Algorithm2d::FrontalQuads {
        // Gmsh 4.15 crashes with its Frontal-Delaunay for quads on faces closed both ways.
        let mut closed = false;
        for &face in faces {
            closed |= matches!(gmsh.entity_type(2, face)?.as_str(), "Sphere" | "Torus");
        }
        if closed && options.algorithms.0 == Algorithm2d::FrontalQuads {
            options.algorithms.0 = Algorithm2d::FrontalDelaunay;
            notes.push(
                "Frontal-Delaunay for quads cannot mesh spheres and tori: Frontal-Delaunay \
                 is used instead."
                    .into(),
            );
        }
    }
    if options.quads && options.algorithms.0 == Algorithm2d::QuasiStructuredQuad {
        let mut largest: f64 = 0.0;
        for &face in faces {
            largest = largest.max(gmsh.mass(2, face)? / (options.max_size * options.max_size));
        }
        if largest > QUASI_STRUCTURED_LIMIT {
            options.algorithms.0 = Algorithm2d::FrontalDelaunay;
            notes.push(format!(
                "A face would get about {largest:.0} quadrilaterals, too many for the \
                 quasi-structured algorithm: Frontal-Delaunay is used and recombined instead."
            ));
        }
    }
    let run = |options: &MeshOptions| -> Result<Result<(), GmshError>, GmshError> {
        options.apply(gmsh)?;
        let fields = size_fields(gmsh, sizes, options.size_factor())?;
        let result = gmsh.generate(2);
        for field in fields.into_iter().rev() {
            gmsh.remove_field(field)?;
        }
        Ok(result)
    };
    let first = run(options)?;
    if !(options.quads && options.algorithms.0 == Algorithm2d::QuasiStructuredQuad) {
        return first;
    }
    let quality = match &first {
        Ok(()) => worst_quad(gmsh, faces, options.order)?,
        Err(_) => None,
    };
    let coarse = first.is_ok() && too_coarse(gmsh, faces, options.max_size)?;
    if first.is_ok() && !coarse && quality.is_some_and(|q| q >= DISTORTED_QUAD) {
        return Ok(());
    }
    let structured = options.algorithms.0;
    options.algorithms.0 = Algorithm2d::FrontalDelaunay;
    gmsh.clear_mesh()?;
    run(options)??;
    let recombined = worst_quad(gmsh, faces, options.order)?;
    match (first, quality) {
        (Ok(()), Some(quality)) if !coarse && recombined.is_none_or(|r| r < quality) => {
            // The other algorithm is no better: back to the quasi-structured mesh.
            options.algorithms.0 = structured;
            gmsh.clear_mesh()?;
            run(options)?
        }
        (Ok(()), _) if coarse => {
            notes.push(
                "The quasi-structured algorithm left elements far coarser than the max \
                 element size: the faces are meshed with Frontal-Delaunay and recombined \
                 instead."
                    .into(),
            );
            Ok(())
        }
        (Ok(()), _) => {
            notes.push(
                "The quasi-structured algorithm left a badly distorted quadrilateral: the \
                 faces are meshed with Frontal-Delaunay and recombined instead."
                    .into(),
            );
            Ok(())
        }
        (Err(error), _) => {
            notes.push(format!(
                "The quasi-structured algorithm failed ({error}): the faces are meshed with \
                 Frontal-Delaunay and recombined instead."
            ));
            Ok(())
        }
    }
}

/// Puts a newly meshed part into a mesh in place of the part of the same name. The other
/// parts keep their node and element numbers; the new part is numbered after the highest
/// numbers the mesh had, so that nothing that referred to the old part points into the new
/// one.
pub fn merge_part(mesh: &FeMesh, part_mesh: FeMesh) -> FeMesh {
    let replaced: Vec<String> = part_mesh.parts.iter().map(|p| p.name.clone()).collect();
    let is_replaced = |p: &Part| replaced.iter().any(|r| r.eq_ignore_ascii_case(&p.name));
    let removed: BTreeSet<ElementId> = (mesh.parts.iter())
        .filter(|p| is_replaced(p))
        .flat_map(|p| p.elements.iter().copied())
        .collect();
    let node_offset = mesh.node_ids().iter().copied().max().unwrap_or(0);
    let element_offset = mesh.elements().iter().map(|e| e.id).max().unwrap_or(0);

    let mut merged = FeMesh::default();
    let kept: Vec<&Element> = (mesh.elements().iter())
        .filter(|e| !removed.contains(&e.id))
        .collect();
    let used: BTreeSet<NodeId> = kept.iter().flat_map(|e| e.nodes.iter().copied()).collect();
    let removed_nodes: BTreeSet<NodeId> = (mesh.elements().iter())
        .filter(|e| removed.contains(&e.id))
        .flat_map(|e| e.nodes.iter().copied())
        .filter(|n| !used.contains(n))
        .collect();
    for (&id, &coords) in mesh.node_ids().iter().zip(mesh.coords()) {
        if !removed_nodes.contains(&id) {
            merged.set_node(id, coords);
        }
    }
    for element in kept {
        // The elements come from a valid mesh.
        let _ = merged.add_element(element.clone());
    }
    for (&id, &coords) in part_mesh.node_ids().iter().zip(part_mesh.coords()) {
        merged.set_node(id + node_offset, coords);
    }
    for element in part_mesh.elements() {
        let _ = merged.add_element(Element {
            id: element.id + element_offset,
            nodes: element.nodes.iter().map(|n| n + node_offset).collect(),
            ..element.clone()
        });
    }
    let mut new_parts: Vec<Part> = (part_mesh.parts.into_iter())
        .map(|p| Part {
            elements: p.elements.iter().map(|e| e + element_offset).collect(),
            ..p
        })
        .collect();
    for part in &mesh.parts {
        if !is_replaced(part) {
            merged.parts.push(part.clone());
        } else if let Some(index) =
            (new_parts.iter()).position(|p| p.name.eq_ignore_ascii_case(&part.name))
        {
            merged.parts.push(new_parts.remove(index));
        }
    }
    merged.parts.extend(new_parts);
    merged.node_sets = (mesh.node_sets.iter())
        .map(|(name, nodes)| {
            let nodes = nodes.iter().copied().filter(|n| !removed_nodes.contains(n));
            (name.clone(), nodes.collect())
        })
        .collect();
    merged.element_sets = (mesh.element_sets.iter())
        .map(|(name, elements)| {
            let elements = elements.iter().copied().filter(|e| !removed.contains(e));
            (name.clone(), elements.collect())
        })
        .collect();
    merged.surfaces = (mesh.surfaces.iter())
        .map(|(name, surface)| {
            let surface = match surface {
                SurfaceDefinition::ElementFaces(faces) => SurfaceDefinition::ElementFaces(
                    faces
                        .iter()
                        .copied()
                        .filter(|(e, _)| !removed.contains(e))
                        .collect(),
                ),
                SurfaceDefinition::Nodes(nodes) => SurfaceDefinition::Nodes(
                    nodes
                        .iter()
                        .copied()
                        .filter(|n| !removed_nodes.contains(n))
                        .collect(),
                ),
            };
            (name.clone(), surface)
        })
        .collect();
    merged.cad = mesh.cad.without(&removed, &removed_nodes);
    merged
        .cad
        .extend(part_mesh.cad.offset(node_offset, element_offset));
    merged
}

/// Deletes a part of a mesh with its elements and the nodes no other part uses.
/// Sets and surfaces lose them too; the other parts keep their
/// numbers.
pub fn delete_mesh_part(mesh: &FeMesh, part: &str) -> FeMesh {
    delete_mesh_parts(mesh, &[part.to_owned()])
}

/// [`delete_mesh_part`] for several parts at once, as selected together: the mesh is built
/// anew once rather than once per part.
pub fn delete_mesh_parts(mesh: &FeMesh, parts: &[String]) -> FeMesh {
    let mut smaller = mesh.clone();
    smaller.remove_parts(parts);
    smaller
}

/// Loads Gmsh and meshes a cube, for the self test in the settings.
pub fn self_test() -> Result<(LibraryInfo, usize), GmshError> {
    with_gmsh(|gmsh| {
        gmsh.add_box([0.0; 3], [10.0; 3])?;
        let options = MeshOptions {
            max_size: 2.5,
            min_size: 0.0,
            elements_per_2pi: 0.0,
            order: 2,
            straight_midside_nodes: true,
            netgen: false,
            algorithms: Default::default(),
            quads: false,
            recombination: 1.0,
            visible_only: false,
        };
        options.apply(gmsh)?;
        gmsh.generate(3)?;
        let volumes = gmsh.entities(3)?;
        let named: Vec<(i32, String)> =
            volumes.iter().map(|&v| (v, format!("SOLID-{v}"))).collect();
        let mesh = solid_mesh(gmsh, 2, &named)?;
        Ok((gmsh.info().clone(), mesh.element_count()))
    })
}

/// The meshing options nodalis sets.
struct MeshOptions {
    max_size: f64,
    min_size: f64,
    elements_per_2pi: f64,
    order: i32,
    straight_midside_nodes: bool,
    netgen: bool,
    algorithms: (Algorithm2d, Algorithm3d),
    /// Recombines triangles into quadrilaterals where Gmsh can; for faces only, never for
    /// the boundary of a solid meshed with tetrahedra.
    quads: bool,
    /// How triangles are recombined, as Gmsh numbers it; Blossom unless an extrude mesh
    /// chooses another.
    recombination: f64,
    /// Meshes only the entities left visible.
    visible_only: bool,
}

impl MeshOptions {
    /// The options of a part after its meshing parameters and Gmsh algorithms.
    fn of_part(geometry: &Geometry, part: &str) -> Self {
        let setup = geometry.parameters(part);
        let curvature = if setup.elements_per_curvature > 0.0 {
            (std::f64::consts::TAU * setup.elements_per_curvature).round()
        } else {
            0.0
        };
        Self {
            max_size: setup.max_size,
            min_size: setup.min_size.min(setup.max_size),
            elements_per_2pi: curvature,
            order: if setup.second_order { 2 } else { 1 },
            straight_midside_nodes: !setup.midside_nodes_on_geometry,
            netgen: setup.optimize,
            algorithms: geometry.algorithms(part),
            quads: setup.quad_dominated,
            recombination: 1.0,
            visible_only: false,
        }
    }

    /// The options for the faces of a part meshed on their own (a shell or a 2D model):
    /// quadrilaterals come from the quasi-structured algorithm unless a Gmsh item chose one.
    fn of_faces(geometry: &Geometry, part: &str) -> Self {
        let mut options = Self::of_part(geometry, part);
        if options.quads && geometry.chosen_algorithms(part).is_none() {
            options.algorithms.0 = Algorithm2d::QuasiStructuredQuad;
        }
        options
    }

    /// What the sizes handed to Gmsh are multiplied by to get the sizes asked for: the
    /// quasi-structured algorithm ends by splitting every element in four.
    fn size_factor(&self) -> f64 {
        if self.algorithms.0 == Algorithm2d::QuasiStructuredQuad {
            2.0
        } else {
            1.0
        }
    }

    /// Sets every option; Gmsh keeps options between uses.
    fn apply(&self, gmsh: &Gmsh) -> Result<(), GmshError> {
        let (algorithm_2d, algorithm_3d) = self.algorithms;
        // Gmsh's numbers of the algorithms.
        let algorithm_2d = match algorithm_2d {
            Algorithm2d::MeshAdapt => 1.0,
            Algorithm2d::Automatic => 2.0,
            Algorithm2d::Delaunay => 5.0,
            Algorithm2d::FrontalDelaunay => 6.0,
            Algorithm2d::FrontalQuads => 8.0,
            Algorithm2d::PackingOfParallelograms => 9.0,
            Algorithm2d::QuasiStructuredQuad => 11.0,
        };
        let algorithm_3d = match algorithm_3d {
            Algorithm3d::Delaunay => 1.0,
            Algorithm3d::Frontal => 4.0,
            Algorithm3d::Hxt => 10.0,
        };
        let factor = self.size_factor();
        // Half the elements along a curve for the quasi-structured algorithm, but an even
        // number on a full circle and none only if none are asked for.
        let elements_per_2pi = if factor > 1.0 && self.elements_per_2pi > 0.0 {
            (self.elements_per_2pi / factor).ceil().max(4.0)
        } else {
            self.elements_per_2pi
        };
        for (name, value) in [
            ("Mesh.MeshSizeMax", self.max_size * factor),
            ("Mesh.MeshSizeMin", self.min_size),
            ("Mesh.MeshSizeFromCurvature", elements_per_2pi),
            // Gmsh gives the points of a shape sizes of its own when it reads it or removes
            // other shapes; the sizes come from the parameters and size fields only.
            ("Mesh.MeshSizeFromPoints", 0.0),
            ("Mesh.MeshSizeExtendFromBoundary", 1.0),
            ("Mesh.ElementOrder", f64::from(self.order)),
            (
                "Mesh.SecondOrderLinear",
                f64::from(u8::from(self.straight_midside_nodes)),
            ),
            ("Mesh.HighOrderOptimize", 0.0),
            ("Mesh.Algorithm", algorithm_2d),
            ("Mesh.Algorithm3D", algorithm_3d),
            ("Mesh.Optimize", 1.0),
            ("Mesh.OptimizeNetgen", f64::from(u8::from(self.netgen))),
            ("Mesh.RecombineAll", f64::from(u8::from(self.quads))),
            ("Mesh.RecombinationAlgorithm", self.recombination),
            (
                "Mesh.MeshOnlyVisible",
                f64::from(u8::from(self.visible_only)),
            ),
            // Quadratic quadrilaterals with 8 nodes, as CalculiX has them.
            ("Mesh.SecondOrderIncomplete", 1.0),
        ] {
            gmsh.set_number(name, value)?;
        }
        Ok(())
    }
}

/// Node coordinates of the generated mesh by Gmsh node tag.
fn node_coords(gmsh: &Gmsh) -> Result<HashMap<usize, [f64; 3]>, GmshError> {
    let (tags, coords) = gmsh.nodes()?;
    Ok(tags
        .into_iter()
        .zip(coords.as_chunks::<3>().0)
        .map(|(tag, &c)| (tag, c))
        .collect())
}

fn node_id(tag: usize) -> Result<NodeId, GmshError> {
    NodeId::try_from(tag).map_err(|_| GmshError::Other("too many nodes".into()))
}

/// The tetrahedra of the solids given with their part names, one part per solid.
fn solid_mesh(gmsh: &Gmsh, order: i32, volumes: &[(i32, String)]) -> Result<FeMesh, GmshError> {
    if volumes.is_empty() {
        return Err(GmshError::Other("The geometry contains no solids".into()));
    }
    let coords = node_coords(gmsh)?;
    let (gmsh_type, shape, type_name) = if order == 2 {
        (TET10, ElementShape::Tet10, "C3D10")
    } else {
        (TET4, ElementShape::Tet4, "C3D4")
    };
    let mut mesh = FeMesh::default();
    let mut next_id: ElementId = 1;
    for (volume, name) in volumes {
        let (_, nodes) = gmsh.elements(gmsh_type, *volume)?;
        let mut part = Part {
            bodies: Vec::new(),
            name: name.clone(),
            elements: Vec::with_capacity(nodes.len() / shape.node_count()),
        };
        for tet in nodes.chunks_exact(shape.node_count()) {
            let mut ids = Vec::with_capacity(tet.len());
            for k in 0..tet.len() {
                let tag = if order == 2 {
                    tet[TET10_TO_CALCULIX[k]]
                } else {
                    tet[k]
                };
                let position = coords
                    .get(&tag)
                    .ok_or_else(|| GmshError::Other(format!("Node {tag} is missing")))?;
                let id = node_id(tag)?;
                mesh.set_node(id, *position);
                ids.push(id);
            }
            mesh.add_element(Element {
                id: next_id,
                type_name: type_name.into(),
                shape,
                nodes: ids,
            })
            .map_err(|e| GmshError::Other(e.to_string()))?;
            part.elements.push(next_id);
            next_id += 1;
        }
        if !part.elements.is_empty() {
            mesh.parts.push(part);
        }
    }
    if mesh.element_count() == 0 {
        return Err(GmshError::Other("Gmsh created no elements".into()));
    }
    let mut faces = BTreeSet::new();
    for (volume, _) in volumes {
        faces.extend(gmsh.adjacencies(3, *volume)?.1);
    }
    let faces: Vec<i32> = faces.into_iter().collect();
    mesh.cad = cad_map(gmsh, &mesh, &faces, order, false)?;
    Ok(mesh)
}

/// Where the given CAD faces, their edges and vertices lie in a mesh made from Gmsh's nodes:
/// the faces of solid elements on the CAD faces, or for shells the elements themselves.
fn cad_map(
    gmsh: &Gmsh,
    mesh: &FeMesh,
    faces: &[i32],
    order: i32,
    shell: bool,
) -> Result<CadMap, GmshError> {
    let (surface_types, line_type): (&[i32], i32) = match (order, shell) {
        (2, false) => (&[TRI6], LINE3),
        (2, true) => (&[TRI6, QUAD8], LINE3),
        (_, false) => (&[TRI3], LINE2),
        (_, true) => (&[TRI3, QUAD4], LINE2),
    };
    // The solid elements' faces by their sorted corner nodes.
    let mut solid_faces: HashMap<Vec<NodeId>, (ElementId, u8)> = HashMap::new();
    if !shell {
        for element in mesh.elements() {
            for (k, face) in element.faces().iter().enumerate() {
                let mut corners: Vec<NodeId> =
                    face.corners.iter().map(|&i| element.nodes[i]).collect();
                corners.sort_unstable();
                solid_faces.insert(corners, (element.id, (k + 1) as u8));
            }
        }
    }
    let shell_faces: HashMap<Vec<NodeId>, ElementId> = if shell {
        (mesh.elements().iter())
            .map(|e| {
                let mut nodes = e.nodes.clone();
                nodes.sort_unstable();
                (nodes, e.id)
            })
            .collect()
    } else {
        HashMap::new()
    };
    let ids = |tags: &[usize]| -> Result<Vec<NodeId>, GmshError> {
        tags.iter().map(|&t| node_id(t)).collect()
    };
    let mut map = CadMap::default();
    let mut edges = BTreeSet::new();
    for &face in faces {
        let mut nodes = BTreeSet::new();
        let mut on_face = Vec::new();
        for &gmsh_type in surface_types {
            let n = match gmsh_type {
                TRI3 => 3,
                QUAD4 => 4,
                TRI6 => 6,
                _ => 8,
            };
            let (_, tags) = gmsh.elements(gmsh_type, face)?;
            for element in tags.chunks_exact(n) {
                let element = ids(element)?;
                nodes.extend(element.iter().copied());
                let found = if shell {
                    let mut sorted = element.clone();
                    sorted.sort_unstable();
                    shell_faces.get(&sorted).map(|&e| (e, 1))
                } else {
                    let mut corners = element[..3].to_vec();
                    corners.sort_unstable();
                    solid_faces.get(&corners).copied()
                };
                on_face.extend(found);
            }
        }
        map.nodes
            .insert(CadEntity::Face(face), nodes.into_iter().collect());
        map.faces.insert(face, on_face);
        edges.extend(gmsh.adjacencies(2, face)?.1);
    }
    let mut vertices = BTreeSet::new();
    for edge in edges {
        let (_, tags) = gmsh.elements(line_type, edge)?;
        let n = if line_type == LINE3 { 3 } else { 2 };
        let mut nodes = BTreeSet::new();
        let mut segments = Vec::new();
        for segment in tags.chunks_exact(n) {
            let segment = ids(segment)?;
            nodes.extend(segment.iter().copied());
            segments.push([segment[0], segment[1]]);
        }
        map.nodes
            .insert(CadEntity::Edge(edge), nodes.into_iter().collect());
        map.segments.insert(edge, segments);
        vertices.extend(gmsh.adjacencies(1, edge)?.1);
    }
    for vertex in vertices {
        let (_, tags) = gmsh.elements(POINT, vertex)?;
        let nodes = ids(&tags)?;
        if !nodes.is_empty() {
            map.nodes.insert(CadEntity::Vertex(vertex), nodes);
        }
    }
    Ok(map)
}

/// Part names from the names in the CAD file, else "SOLID-n", "SHELL-n" or "LINE-n" after
/// the dimension; upper case and unique among all parts, as CalculiX names are
/// case-insensitive and solids, shells and lines share them.
fn entity_part_names(gmsh: &Gmsh, entities: &[Entity]) -> Result<Vec<String>, GmshError> {
    let mut names = Vec::with_capacity(entities.len());
    let mut used = BTreeSet::new();
    let mut counts = [0; 4];
    for &(dim, tag) in entities {
        counts[dim as usize] += 1;
        let base = label_name(&gmsh.entity_name(dim, tag)?).unwrap_or_else(|| {
            let prefix = match dim {
                3 => "SOLID",
                2 => "SHELL",
                _ => "LINE",
            };
            format!("{prefix}-{}", counts[dim as usize])
        });
        let name = unique_name(&base, &used);
        used.insert(name.clone());
        names.push(name);
    }
    Ok(names)
}

/// The part name for the name Gmsh gives an entity read from a CAD file, a path of labels
/// such as "Shapes/Assembly/1/Bracket": [`calculix_name`] of its last label. OpenCASCADE's
/// own "Open CASCADE STEP translator 7.8 1.2" is no name.
fn label_name(label: &str) -> Option<String> {
    let last = label.rsplit('/').next().unwrap_or_default().trim();
    if last.starts_with("Open CASCADE STEP translator") {
        return None;
    }
    calculix_name(last)
}

/// A name the solver accepts: letters, digits, `-` and `_`, upper case, at most 60 characters.
/// German umlauts are written out, as "Ä" as "AE".
fn calculix_name(label: &str) -> Option<String> {
    let name: String = label
        .trim()
        .chars()
        .flat_map(|c| {
            let written: &[char] = match c {
                'ä' | 'Ä' => &['A', 'E'],
                'ö' | 'Ö' => &['O', 'E'],
                'ü' | 'Ü' => &['U', 'E'],
                'ß' | 'ẞ' => &['S', 'S'],
                _ => &[],
            };
            let one = if c.is_ascii_alphanumeric() || c == '-' {
                c.to_ascii_uppercase()
            } else {
                '_'
            };
            if written.is_empty() {
                vec![one]
            } else {
                written.to_vec()
            }
        })
        .take(60)
        .collect();
    let name = name.trim_matches('_').to_string();
    name.chars()
        .next()
        .is_some_and(|c| c.is_ascii_alphabetic())
        .then_some(name)
}

/// Builds the display mesh from Gmsh's surface mesh: per solid its faces as triangles and its
/// edges as lines. Each shell of faces outside the solids forms one more part, and so does each edge outside
/// any face.
fn display_mesh(
    gmsh: &Gmsh,
    geometry: &Geometry,
    polylines: &BTreeMap<i32, Vec<[f64; 3]>>,
) -> Result<GeometryDisplay, GmshError> {
    let mut coords = node_coords(gmsh)?;
    let surfaces = gmsh.entities(2)?;
    let curves = gmsh.entities(1)?;
    // Parts as (name, faces, edges); the edges of the faces are added below. Every shell
    // of faces outside the solids is a part of its own, as for 2D models, every edge outside
    // the faces a line part.
    let mut groups: Vec<(String, u8, Vec<i32>, Vec<i32>)> = Vec::new();
    for (shape, name) in parts(gmsh, geometry)? {
        let (faces, edges) = match shape.dim {
            3 => (gmsh.adjacencies(3, shape.tags[0])?.1, Vec::new()),
            2 => (shape.tags.clone(), Vec::new()),
            _ => (Vec::new(), shape.tags.clone()),
        };
        groups.push((name, shape.dim as u8, faces, edges));
    }

    // Triangles and edge segments of each entity, fetched once.
    let mut triangles: BTreeMap<i32, Vec<usize>> = BTreeMap::new();
    for &surface in &surfaces {
        triangles.insert(surface, gmsh.elements(TRI3, surface)?.1);
    }
    // The edges get nodes of their own, numbered on after the triangles' nodes.
    let mut next_tag = coords.keys().max().map_or(1, |&tag| tag + 1);
    let mut segments: BTreeMap<i32, Vec<usize>> = BTreeMap::new();
    for (&curve, points) in polylines {
        let tags: Vec<usize> = points
            .iter()
            .map(|&point| {
                coords.insert(next_tag, point);
                next_tag += 1;
                next_tag - 1
            })
            .collect();
        let nodes = tags.windows(2).flatten().copied().collect();
        segments.insert(curve, nodes);
    }

    let mut topology = CadTopology::default();
    for &surface in &surfaces {
        topology
            .face_edges
            .insert(surface, gmsh.adjacencies(2, surface)?.1);
    }
    for &curve in &curves {
        topology
            .edge_vertices
            .insert(curve, gmsh.adjacencies(1, curve)?.1);
    }

    let mut mesh = FeMesh::default();
    let mut entities = Vec::new();
    let mut part_dimensions = Vec::new();
    let mut add = |mesh: &mut FeMesh,
                   part: &mut Part,
                   entity: CadEntity,
                   nodes: &[usize]|
     -> Result<(), GmshError> {
        let (shape, type_name) = match entity {
            CadEntity::Face(_) => (ElementShape::Tri3, "S3"),
            CadEntity::Edge(_) | CadEntity::Vertex(_) => (ElementShape::Line2, "B31"),
        };
        let mut ids = Vec::with_capacity(nodes.len());
        for &tag in nodes {
            let id = node_id(tag)?;
            if let Some(position) = coords.get(&tag) {
                mesh.set_node(id, *position);
            }
            ids.push(id);
        }
        let id = ElementId::try_from(entities.len() + 1)
            .map_err(|_| GmshError::Other("too many elements".into()))?;
        entities.push(entity);
        part.elements.push(id);
        mesh.add_element(Element {
            id,
            type_name: type_name.into(),
            shape,
            nodes: ids,
        })
        .map_err(|e| GmshError::Other(e.to_string()))
    };
    for (name, dimension, faces, own_edges) in groups {
        let mut part = Part {
            bodies: Vec::new(),
            name,
            elements: Vec::new(),
        };
        let mut edges: BTreeSet<i32> = own_edges.into_iter().collect();
        for &face in &faces {
            let nodes = triangles.get(&face).map_or(&[][..], Vec::as_slice);
            for triangle in nodes.as_chunks::<3>().0 {
                add(&mut mesh, &mut part, CadEntity::Face(face), triangle)?;
            }
            edges.extend(topology.face_edges.get(&face).into_iter().flatten());
        }
        for edge in edges {
            let nodes = segments.get(&edge).map_or(&[][..], Vec::as_slice);
            for segment in nodes.as_chunks::<2>().0 {
                add(&mut mesh, &mut part, CadEntity::Edge(edge), segment)?;
            }
        }
        if !part.elements.is_empty() {
            mesh.parts.push(part);
            part_dimensions.push(dimension);
        }
    }
    Ok(GeometryDisplay {
        mesh,
        entities,
        part_dimensions,
        solids: gmsh.entities(3)?.len(),
        faces: surfaces.len(),
        edges: curves.len(),
        topology,
    })
}

/// A file in the temporary directory, deleted when dropped; Gmsh reads and writes only files.
struct TempFile(PathBuf);

impl TempFile {
    fn new(extension: &str) -> Self {
        static COUNTER: AtomicU32 = AtomicU32::new(0);
        let n = COUNTER.fetch_add(1, Ordering::Relaxed);
        Self(std::env::temp_dir().join(format!(
            "nodalis-gmsh-{}-{n}.{extension}",
            std::process::id()
        )))
    }

    fn with_contents(extension: &str, contents: &str) -> Result<Self, GmshError> {
        let file = Self::new(extension);
        std::fs::write(&file.0, contents)
            .map_err(|e| GmshError::Other(format!("{}: {e}", file.0.display())))?;
        Ok(file)
    }
}

impl Drop for TempFile {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod message_tests {
    use super::*;

    #[test]
    fn gmsh_messages_name_their_entities() {
        assert_eq!(
            named_entities(
                "Gmsh: Invalid boundary mesh (overlapping facets) on surface 23 surface 35"
            ),
            [CadEntity::Face(23), CadEntity::Face(35)]
        );
        assert_eq!(
            named_entities("Unable to recover the edge 12 (1/3) on curve 4 (on surface 7)"),
            [CadEntity::Edge(4), CadEntity::Face(7)]
        );
        assert_eq!(
            named_entities("Meshing Surface 5 failed, surface 5"),
            [CadEntity::Face(5)]
        );
        assert_eq!(named_entities("near point 3."), []);
        assert_eq!(named_entities("point 3"), [CadEntity::Vertex(3)]);
        assert!(named_entities("No elements in volume 1").is_empty());
        assert!(named_entities("segment and facet intersect at point (1, 2, 3)").is_empty());
    }
}
