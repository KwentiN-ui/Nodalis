use serde::{Deserialize, Serialize};

use crate::{EntitySignature, TagMap, match_entities};

/// CAD geometry imported from a STEP, IGES or BREP file, the source of the mesh.
///
/// The shapes are kept in OpenCASCADE's BREP format, so that a project still meshes after the
/// original file moved or changed. Each solid is a part; faces and edges are referred to by
/// the tags Gmsh gives them when it reads the BREP, which stay the same between reads.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Geometry {
    /// Name of the imported file, for display.
    pub source: String,
    /// The shapes as an OpenCASCADE BREP file.
    pub brep: String,
    /// Meshing parameters of the parts that no Meshing Parameters item covers.
    pub meshing: MeshingParameters,
    /// The mesh setup items.
    pub mesh_items: Vec<MeshSetupItem>,
    /// Names of the parts, the solids and then the faces outside them, once a part was
    /// deleted: Gmsh numbers the parts that stay anew, while their names must stay. Empty
    /// while the names Gmsh gives are the parts' names.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub part_names: Vec<String>,
    /// Factor on the fineness of the whole mesh: every element size of the mesh setup is
    /// divided by it when the parts are meshed, the sizes as set staying as they are. A value
    /// link of the project schematic, such as a mesh convergence, gives it.
    #[serde(default = "one", skip_serializing_if = "is_one")]
    pub refinement: f64,
    /// What each face, edge and vertex looks like, recorded when the BREP was read, so that
    /// selections and local mesh sizes find their entities again when the shapes change and
    /// Gmsh numbers them anew, see [`Geometry::tags_in`]. Empty if not recorded.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub signatures: Vec<EntitySignature>,
    /// Parts joined into compound parts: each is meshed part by part and becomes one part
    /// of the FE model, its bodies tied where they touch.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub compounds: Vec<CompoundPart>,
    /// The suppressed parts, by id: meshing and the CAD tools leave them alone, as if they
    /// were not there, until they are unsuppressed. A mesh they have already stays.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub suppressed: Vec<crate::Id>,
    /// The parts this analysis meshes itself although a link brings their mesh with them,
    /// by id: the link's mesh of them is left out.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub own_meshes: Vec<crate::Id>,
    /// The identity of each part with its name: everything referring to a part of the
    /// geometry, its suppression, the mesh setup and the compounds, does so by the id, so
    /// that renaming the part keeps it. An analysis taking the geometry through a link takes
    /// the names with the ids, see [`Geometry::renames_since`]. Made with the parts, see
    /// [`Geometry::ensure_ids`].
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub ids: Vec<PartId>,
    /// Whether a link brings the shapes: the analysis then takes the mesh of each part the
    /// link brings meshed, unless it meshes the part itself ([`Geometry::own_meshes`]).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub from_link: bool,
    /// The operations of the Geometry tree, such as a split by plane, in the order they act
    /// on the shapes: a pipeline from [`Geometry::input`] to these shapes, as CAD Operation
    /// blocks before the analysis would be. They act anew whenever the input changes, as
    /// when a link brings new shapes, and when one is changed or deleted.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub operations: Vec<GeometryOperation>,
    /// The shapes before the operations, as imported or as a link brought them; set while
    /// there are operations. Only its shapes count: the mesh setup, suppression and the
    /// like are those of the geometry holding it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub input: Option<Box<Geometry>>,
}

/// An operation of the Geometry tree on the shapes of an analysis, see
/// [`Geometry::operations`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GeometryOperation {
    pub name: String,
    pub operation: crate::CadOperation,
}

/// The id of a part of the geometry, see [`Geometry::ids`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PartId {
    pub name: String,
    pub id: crate::Id,
}

/// Cut faces of a cyclic symmetric sector, whose meshes have to match: the slave faces are
/// the master faces rotated by one sector about the axis. A cyclic symmetry ties every slave
/// node to the master face it lands on; where the nodes of both sides do not coincide, the
/// tie passes the forces on unevenly and the stresses next to the cut faces show stripes.
/// The mesher copies the mesh of the master faces on to the slave faces instead. Taken from
/// the cyclic symmetry ties for meshing ([`crate::FeModel::matching_faces`]), not stored.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PeriodicFaces {
    /// Faces of the geometry (Gmsh tags) on the master side.
    pub master: Vec<i32>,
    /// Faces of the geometry on the slave side.
    pub slave: Vec<i32>,
    /// The cyclic symmetry: its axis.
    pub cyclic: crate::CyclicSymmetry,
}

/// Parts of the geometry joined into one part of the FE model, as the pieces a CAD model
/// is cut into for the mesher. Each piece is meshed on its own; where they touch, the
/// solver input ties them, as a bonded contact would.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CompoundPart {
    /// The id of the part of the FE model it becomes, see [`plx_mesh::Part::id`].
    pub id: crate::Id,
    pub name: String,
    /// The parts of the geometry it joins, by their ids ([`Geometry::ids`]).
    pub parts: Vec<crate::Id>,
}

impl CompoundPart {
    /// A new compound part of the parts of the geometry `parts`, by id.
    pub fn new(name: impl Into<String>, parts: Vec<crate::Id>) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            parts,
        }
    }
}

fn one() -> f64 {
    1.0
}

fn is_one(value: &f64) -> bool {
    *value == 1.0
}

impl Geometry {
    /// The shapes alone, without the operations of the Geometry tree and the input they act
    /// on: what another analysis or block takes through a link, and what the operations
    /// start from.
    pub fn without_operations(mut self) -> Self {
        self.operations.clear();
        self.input = None;
        self
    }

    /// A name for a new operation of the Geometry tree, e.g. "Split by Plane-2".
    pub fn operation_name(&self, label: &str) -> String {
        let taken = |n: &str| {
            self.operations
                .iter()
                .any(|o| o.name.eq_ignore_ascii_case(n))
        };
        (1..)
            .map(|i| format!("{label}-{i}"))
            .find(|n| !taken(n))
            .unwrap_or_default()
    }

    /// The compound part holding the part `part`, by index.
    pub fn compound_of(&self, part: &str) -> Option<usize> {
        self.compound_of_id(self.part_id(part)?)
    }

    /// The compound part holding the part of the id `part`, by index.
    pub fn compound_of_id(&self, part: crate::Id) -> Option<usize> {
        (self.compounds.iter()).position(|c| c.parts.contains(&part))
    }

    /// The compound part of the id `id`, by index.
    pub fn compound_with_id(&self, id: crate::Id) -> Option<usize> {
        (self.compounds.iter()).position(|c| c.id == id)
    }

    /// The compound part named `name`, by index.
    pub fn compound_named(&self, name: &str) -> Option<usize> {
        (self.compounds.iter()).position(|c| c.name.eq_ignore_ascii_case(name))
    }

    /// The name of the part of the FE model that the part `part` of the geometry becomes:
    /// its compound's, or its own.
    pub fn fe_part_name<'a>(&'a self, part: &'a str) -> &'a str {
        match self.compound_of(part) {
            Some(c) => &self.compounds[c].name,
            None => part,
        }
    }

    /// The parts of the geometry the part `name` of the FE model is meshed from: the parts
    /// of a compound, else the part itself.
    pub fn parts_of_fe_part(&self, name: &str) -> Vec<String> {
        match self.compound_named(name) {
            Some(c) => self.member_names(c),
            None => vec![name.to_owned()],
        }
    }

    /// The names of the parts the compound part `compound` (by index) joins.
    pub fn member_names(&self, compound: usize) -> Vec<String> {
        (self.compounds[compound].parts.iter())
            .filter_map(|&id| self.part_name(id))
            .map(str::to_owned)
            .collect()
    }

    /// The compounds as the mesh joins its parts, as (id, name, members), see
    /// [`plx_mesh::FeMesh::compose_compounds`].
    pub fn compound_members(&self) -> Vec<(crate::Id, String, Vec<crate::Id>)> {
        (self.compounds.iter())
            .map(|c| (c.id, c.name.clone(), c.parts.clone()))
            .collect()
    }

    /// Renames the part `old` of the geometry to `new`. Its suppression, the mesh setup
    /// and its compound refer to it by id and stay on it. False if the geometry has no part
    /// of that name: its names are not known without Gmsh, see [`Geometry::part_names`].
    pub fn rename_part(&mut self, old: &str, new: &str) -> bool {
        let Some(at) = (self.part_names.iter()).position(|p| p.eq_ignore_ascii_case(old)) else {
            return false;
        };
        self.ensure_ids();
        self.part_names[at] = new.to_owned();
        for entry in &mut self.ids {
            if entry.name.eq_ignore_ascii_case(old) {
                entry.name = new.to_owned();
            }
        }
        true
    }

    /// Whether `parts`, by id, hold the part named `part`.
    pub fn holds(&self, parts: &[crate::Id], part: &str) -> bool {
        self.part_id(part).is_some_and(|id| parts.contains(&id))
    }

    /// The names of the parts `ids`, of those that are there.
    pub fn part_names_of(&self, ids: &[crate::Id]) -> Vec<String> {
        (ids.iter())
            .filter_map(|&id| self.part_name(id))
            .map(str::to_owned)
            .collect()
    }

    /// The parts of the geometry whose names are known: all of them since the geometry was
    /// imported or changed, empty for a geometry Gmsh names, see [`Geometry::part_names`].
    pub fn known_parts(&self) -> &[String] {
        &self.part_names
    }

    /// Whether the part `part` is suppressed.
    pub fn is_suppressed(&self, part: &str) -> bool {
        self.holds(&self.suppressed, part)
    }

    /// Suppresses the parts `parts`, or unsuppresses them; true if that changed anything.
    pub fn set_suppressed(&mut self, parts: &[String], suppressed: bool) -> bool {
        let ids = self.part_ids_of(parts);
        set_ids(&mut self.suppressed, &ids, suppressed)
    }

    /// The id of the part `name`, if it has one, see [`Geometry::ids`].
    pub fn part_id(&self, name: &str) -> Option<crate::Id> {
        (self.ids.iter())
            .find(|p| p.name.eq_ignore_ascii_case(name))
            .map(|p| p.id)
    }

    /// The name of the part of the id `id`.
    pub fn part_name(&self, id: crate::Id) -> Option<&str> {
        (self.ids.iter())
            .find(|p| p.id == id)
            .map(|p| p.name.as_str())
    }

    /// The ids of the parts `names`, giving parts without one an id: for picking parts by
    /// the names the meshes and Gmsh show.
    pub fn part_ids_of(&mut self, names: &[String]) -> Vec<crate::Id> {
        (names.iter())
            .map(|name| match self.part_id(name) {
                Some(id) => id,
                None => {
                    let id = crate::Id::new();
                    self.ids.push(PartId {
                        name: name.clone(),
                        id,
                    });
                    id
                }
            })
            .collect()
    }

    /// Gives the parts of a mesh Gmsh made of this geometry, and their bodies, the ids of
    /// the parts of the geometry they are the meshes of, by name, and compound parts the
    /// compound's: so that a part meshed anew is the same part to everything referring to
    /// it ([`plx_mesh::Part::id`]).
    pub fn stamp_ids(&self, mesh: &mut plx_mesh::FeMesh) {
        for part in &mut mesh.parts {
            if let Some(c) = self.compound_named(&part.name) {
                part.id = self.compounds[c].id;
            } else if let Some(id) = self.part_id(&part.name) {
                part.id = id;
            }
            for body in &mut part.bodies {
                if let Some(id) = self.part_id(&body.name) {
                    body.id = id;
                }
            }
        }
    }

    /// Gives every part of known name an id, and drops the ids of parts that went.
    pub fn ensure_ids(&mut self) {
        if self.part_names.is_empty() {
            return;
        }
        let names = &self.part_names;
        self.ids
            .retain(|p| names.iter().any(|n| n.eq_ignore_ascii_case(&p.name)));
        for name in &self.part_names {
            if !self.ids.iter().any(|p| p.name.eq_ignore_ascii_case(name)) {
                self.ids.push(PartId {
                    name: name.clone(),
                    id: crate::Id::new(),
                });
            }
        }
    }

    /// Takes the ids of the parts of `from` that this geometry has by the same name, e.g.
    /// after a CAD operation made it of `from`; the other parts get new ones.
    pub fn inherit_ids(&mut self, from: &Geometry) {
        for entry in &from.ids {
            if self.part_id(&entry.name).is_none()
                && self
                    .part_names
                    .iter()
                    .any(|n| n.eq_ignore_ascii_case(&entry.name))
            {
                self.ids.push(entry.clone());
            }
        }
        self.ensure_ids();
    }

    /// Gives the parts of this geometry whose ids `before`, the geometry it replaces, does
    /// not know the id of the part of the same name there that this geometry lacks: shapes
    /// read anew from a file, whose parts have new ids, keep them the same parts to what
    /// refers to them, as [`plx_mesh::FeMesh::adopt_ids`] does for a mesh. Returns the ids
    /// changed, as (old, new), for the meshes of the parts; compounds follow here.
    pub fn adopt_ids(&mut self, before: &Geometry) -> Vec<(crate::Id, crate::Id)> {
        let known = |id: crate::Id, list: &[PartId]| list.iter().any(|p| p.id == id);
        let mut changed = Vec::new();
        for index in 0..self.ids.len() {
            let entry = &self.ids[index];
            if known(entry.id, &before.ids) {
                continue;
            }
            let found = (before.ids.iter()).find(|p| {
                p.name.eq_ignore_ascii_case(&entry.name)
                    && !known(p.id, &self.ids)
                    && !changed.iter().any(|(_, new)| *new == p.id)
            });
            if let Some(found) = found {
                changed.push((entry.id, found.id));
                self.ids[index].id = found.id;
            }
        }
        for compound in &mut self.compounds {
            for part in &mut compound.parts {
                if let Some((_, new)) = changed.iter().find(|(old, _)| old == part) {
                    *part = *new;
                }
            }
        }
        for id in self.suppressed.iter_mut().chain(&mut self.own_meshes) {
            if let Some((_, new)) = changed.iter().find(|(old, _)| old == id) {
                *id = *new;
            }
        }
        changed
    }

    /// The parts renamed since `before`, an older copy of the same shapes, as (old, new)
    /// names: found by their ids.
    pub fn renames_since(&self, before: &Geometry) -> Vec<(String, String)> {
        (self.ids.iter())
            .filter_map(|now| {
                let old = before.ids.iter().find(|p| p.id == now.id)?;
                (!old.name.eq_ignore_ascii_case(&now.name))
                    .then(|| (old.name.clone(), now.name.clone()))
            })
            .collect()
    }

    /// Whether this analysis meshes the part `part` itself although a link brings its mesh.
    pub fn meshes_itself(&self, part: &str) -> bool {
        self.holds(&self.own_meshes, part)
    }

    /// Has this analysis mesh the parts `parts` itself, or take their mesh from the link
    /// that brings them again; true if that changed anything.
    pub fn set_own_meshes(&mut self, parts: &[String], own: bool) -> bool {
        let ids = self.part_ids_of(parts);
        set_ids(&mut self.own_meshes, &ids, own)
    }

    /// Takes parts out of their compounds; compounds left empty go.
    pub fn remove_from_compounds(&mut self, parts: &[String]) {
        let ids: Vec<crate::Id> = parts.iter().filter_map(|p| self.part_id(p)).collect();
        for compound in &mut self.compounds {
            compound.parts.retain(|p| !ids.contains(p));
        }
        self.compounds.retain(|c| !c.parts.is_empty());
    }

    /// The meshing parameters of a part: of the last Meshing Parameters item naming it, else
    /// the defaults; its sizes divided by the refinement.
    pub fn parameters(&self, part: &str) -> MeshingParameters {
        let mut parameters = (self.mesh_items.iter().rev())
            .find_map(|item| match &item.kind {
                MeshSetupKind::MeshingParameters { parts, parameters }
                    if self.holds(parts, part) =>
                {
                    Some(*parameters)
                }
                _ => None,
            })
            .unwrap_or(self.meshing);
        parameters.max_size = self.refined(parameters.max_size);
        parameters.min_size = self.refined(parameters.min_size);
        parameters
    }

    /// An element size of the mesh setup as the parts are meshed with it: divided by the
    /// refinement.
    pub fn refined(&self, size: f64) -> f64 {
        if self.refinement.is_finite() && self.refinement > 0.0 {
            size / self.refinement
        } else {
            size
        }
    }

    /// Gmsh's algorithms for a part: of the last Tetrahedral Gmsh item naming it, else the
    /// defaults.
    pub fn algorithms(&self, part: &str) -> (Algorithm2d, Algorithm3d) {
        self.chosen_algorithms(part).unwrap_or_default()
    }

    /// Gmsh's algorithms of the last Tetrahedral Gmsh item naming the part, if any does.
    pub fn chosen_algorithms(&self, part: &str) -> Option<(Algorithm2d, Algorithm3d)> {
        (self.mesh_items.iter().rev()).find_map(|item| match &item.kind {
            MeshSetupKind::TetrahedralGmsh {
                parts,
                algorithm_2d,
                algorithm_3d,
            } if self.holds(parts, part) => Some((*algorithm_2d, *algorithm_3d)),
            _ => None,
        })
    }

    /// The extrude mesh of a part: the last Extrude Mesh item naming it, unless a later
    /// Tetrahedral Gmsh item names it.
    pub fn extrusion(&self, part: &str) -> Option<&ExtrudeMesh> {
        (self.mesh_items.iter().rev())
            .find_map(|item| match &item.kind {
                MeshSetupKind::ExtrudeMesh(extrude) if self.holds(&extrude.parts, part) => {
                    Some(Some(extrude))
                }
                MeshSetupKind::TetrahedralGmsh { parts, .. } if self.holds(parts, part) => {
                    Some(None)
                }
                _ => None,
            })
            .flatten()
    }

    /// The thickening of a shell part: the last Thicken Shell Mesh item naming it.
    pub fn thickening(&self, part: &str) -> Option<&ThickenShellMesh> {
        (self.mesh_items.iter().rev()).find_map(|item| match &item.kind {
            MeshSetupKind::ThickenShellMesh(thicken) if self.holds(&thicken.parts, part) => {
                Some(thicken)
            }
            _ => None,
        })
    }

    /// Whether the part is meshed with hexahedra if it turns out to be an extrusion or a body
    /// of revolution: its meshing parameters ask for it and no Tetrahedral Gmsh item names it.
    pub fn finds_hexahedra(&self, part: &str) -> bool {
        self.parameters(part).hexahedra_where_possible
            && !self.mesh_items.iter().any(|item| {
                matches!(&item.kind, MeshSetupKind::TetrahedralGmsh { parts, .. } if self.holds(parts, part))
            })
    }

    /// Where the faces, edges and vertices of this geometry are in `new`, the geometry that
    /// takes its place, e.g. a STEP file read again or the result of a CAD operation: `None`
    /// when the shapes are the same, so the tags are, or when either geometry has no
    /// signatures to go by. Entities that cannot be found clearly are left out.
    pub fn tags_in(&self, new: &Geometry) -> Option<TagMap> {
        if self.brep == new.brep || self.signatures.is_empty() || new.signatures.is_empty() {
            return None;
        }
        Some(match_entities(&self.signatures, &new.signatures))
    }

    /// Renumbers the faces and edges the mesh setup items name after `tags`, from
    /// [`Self::tags_in`]; those without a new tag are dropped. Returns how many were.
    pub fn renumber_mesh_items(&mut self, tags: &TagMap) -> usize {
        let mut dropped = 0;
        let mut renumber = |tags_of: fn(i32) -> plx_mesh::CadEntity, list: &mut Vec<i32>| {
            let before = list.len();
            *list = (list.iter())
                .filter_map(|&tag| tags.get(&tags_of(tag)).map(|e| e.tag()))
                .collect();
            dropped += before - list.len();
        };
        for item in &mut self.mesh_items {
            match &mut item.kind {
                MeshSetupKind::LocalMeshSize { faces, edges, .. } => {
                    renumber(plx_mesh::CadEntity::Face, faces);
                    renumber(plx_mesh::CadEntity::Edge, edges);
                }
                MeshSetupKind::ExtrudeMesh(extrude) => {
                    renumber(plx_mesh::CadEntity::Face, &mut extrude.faces);
                }
                _ => {}
            }
        }
        dropped
    }

    /// The local mesh sizes as (faces, edges, size, depth), the sizes divided by the
    /// refinement.
    pub fn local_sizes(&self) -> impl Iterator<Item = (&[i32], &[i32], f64, f64)> {
        self.mesh_items.iter().filter_map(|item| match &item.kind {
            MeshSetupKind::LocalMeshSize {
                faces,
                edges,
                size,
                depth,
            } => Some((
                faces.as_slice(),
                edges.as_slice(),
                self.refined(*size),
                *depth,
            )),
            _ => None,
        })
    }
}

/// Adds `ids` to `list`, or takes them out of it; true if that changed anything.
fn set_ids(list: &mut Vec<crate::Id>, ids: &[crate::Id], add: bool) -> bool {
    let before = list.len();
    if add {
        for id in ids {
            if !list.contains(id) {
                list.push(*id);
            }
        }
    } else {
        list.retain(|p| !ids.contains(p));
    }
    list.len() != before
}

/// How a part is meshed with tetrahedra.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct MeshingParameters {
    pub max_size: f64,
    pub min_size: f64,
    /// Elements per curvature radius; curved faces get smaller elements.
    pub elements_per_curvature: f64,
    /// Quadratic tetrahedra (C3D10) instead of linear ones (C3D4).
    pub second_order: bool,
    /// Midside nodes moved onto curved geometry instead of the straight edge's middle.
    pub midside_nodes_on_geometry: bool,
    /// Improves element quality after meshing with Netgen's optimiser.
    pub optimize: bool,
    /// Faces (2D models and shells) are meshed with quadrilaterals instead of triangles, with
    /// Gmsh's quasi-structured algorithm unless a Gmsh item chooses another.
    pub quad_dominated: bool,
    /// Solids that are straight extrusions or bodies of revolution get a swept mesh of
    /// hexahedra, with the defaults of the Extrude Mesh item for extrusions; the others stay
    /// tetrahedra.
    pub hexahedra_where_possible: bool,
}

impl Default for MeshingParameters {
    fn default() -> Self {
        Self {
            // One metre; imports set sizes for the geometry's extent.
            max_size: 1.0,
            min_size: 0.0,
            elements_per_curvature: 2.0,
            second_order: true,
            midside_nodes_on_geometry: false,
            optimize: true,
            quad_dominated: true,
            hexahedra_where_possible: true,
        }
    }
}

impl MeshingParameters {
    /// The sizes for a new geometry: 5 % and 0.1 % of the bounding box diagonal,
    /// rounded to one significant digit.
    pub fn for_diagonal(diagonal: f64) -> Self {
        Self {
            max_size: round_size(0.05 * diagonal),
            min_size: round_size(0.001 * diagonal),
            ..Self::default()
        }
    }
}

/// An item of the mesh setup, refining how parts are meshed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MeshSetupItem {
    /// The internal identity others refer to; the user sees only the name.
    pub id: crate::Id,
    pub name: String,
    pub kind: MeshSetupKind,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum MeshSetupKind {
    /// Own meshing parameters for some parts.
    MeshingParameters {
        /// The parts, by id.
        parts: Vec<crate::Id>,
        parameters: MeshingParameters,
    },
    /// A smaller element size on faces and edges.
    LocalMeshSize {
        faces: Vec<i32>,
        edges: Vec<i32>,
        size: f64,
        /// The depth below the faces to which the size holds inside the solid, so that a
        /// layer of that thickness (the walls of an infill section) gets elements of the
        /// size; 0 for the faces alone.
        depth: f64,
    },
    /// Gmsh's meshing algorithms for some parts.
    TetrahedralGmsh {
        /// The parts, by id.
        parts: Vec<crate::Id>,
        algorithm_2d: Algorithm2d,
        algorithm_3d: Algorithm3d,
    },
    /// Hexahedra and wedges for parts that are extrusions of some of their faces.
    ExtrudeMesh(ExtrudeMesh),
    /// Solid elements for shells: the face mesh offset along its normals in layers.
    ThickenShellMesh(ThickenShellMesh),
}

impl MeshSetupKind {
    /// The name of the item type, also the prefix of new items' names.
    pub fn type_name(&self) -> &'static str {
        match self {
            MeshSetupKind::MeshingParameters { .. } => "Meshing_Parameters",
            MeshSetupKind::LocalMeshSize { .. } => "Local_Mesh_Size",
            MeshSetupKind::TetrahedralGmsh { .. } => "Tetrahedral_Gmsh",
            MeshSetupKind::ExtrudeMesh(_) => "Extrude_Mesh",
            MeshSetupKind::ThickenShellMesh(_) => "Thicken_Shell_Mesh",
        }
    }

    /// [`MeshSetupKind::parts`] to change.
    pub fn parts_mut(&mut self) -> Option<&mut Vec<crate::Id>> {
        match self {
            MeshSetupKind::MeshingParameters { parts, .. }
            | MeshSetupKind::TetrahedralGmsh { parts, .. } => Some(parts),
            MeshSetupKind::ExtrudeMesh(extrude) => Some(&mut extrude.parts),
            MeshSetupKind::ThickenShellMesh(thicken) => Some(&mut thicken.parts),
            MeshSetupKind::LocalMeshSize { .. } => None,
        }
    }

    /// The parts the item applies to; `None` for a local mesh size, which applies to faces
    /// and edges of any part.
    pub fn parts(&self) -> Option<&[crate::Id]> {
        match self {
            MeshSetupKind::MeshingParameters { parts, .. }
            | MeshSetupKind::TetrahedralGmsh { parts, .. } => Some(parts),
            MeshSetupKind::ExtrudeMesh(extrude) => Some(&extrude.parts),
            MeshSetupKind::ThickenShellMesh(thicken) => Some(&thicken.parts),
            MeshSetupKind::LocalMeshSize { .. } => None,
        }
    }
}

/// Extrude Mesh: the source faces of a part are meshed with Gmsh, mostly with
/// quadrilaterals, and the mesh is swept straight through the part in layers, so that
/// quadrilaterals become hexahedra (C3D8, C3D20) and triangles wedges (C3D6, C3D15). The
/// size of the source mesh comes from the part's meshing parameters and local mesh sizes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ExtrudeMesh {
    /// The parts meshed this way, by id.
    pub parts: Vec<crate::Id>,
    /// The source faces; empty to find for each part a face it is an extrusion of.
    pub faces: Vec<i32>,
    pub algorithm_2d: Algorithm2d,
    pub recombination: Recombination,
    /// How the extrusion is divided into layers of elements.
    pub layers: ExtrudeLayers,
}

impl Default for ExtrudeMesh {
    fn default() -> Self {
        Self {
            parts: Vec::new(),
            faces: Vec::new(),
            algorithm_2d: Algorithm2d::QuasiStructuredQuad,
            recombination: Recombination::Blossom,
            layers: ExtrudeLayers::ScaleFactor(1.0),
        }
    }
}

/// Thicken Shell Mesh: the face mesh of a shell part, made as usual, is offset along its
/// normals in layers, so that triangles become wedges (C3D6, C3D15) and quadrilaterals
/// hexahedra (C3D8, C3D20). The shell's surface lies at offset 0 in the thickness; the solid
/// reaches from `offset - thickness / 2` to `offset + thickness / 2` along the normals.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ThickenShellMesh {
    /// The shell parts thickened, by id.
    pub parts: Vec<crate::Id>,
    pub thickness: f64,
    /// Layers of elements through the thickness.
    pub layers: u32,
    /// Shift of the solid's middle from the shell's surface along the normals: 0 centres the
    /// solid on the surface, half the thickness puts it on the side the normals point to.
    pub offset: f64,
}

impl Default for ThickenShellMesh {
    fn default() -> Self {
        Self {
            parts: Vec::new(),
            thickness: 0.001,
            layers: 2,
            offset: 0.0,
        }
    }
}

impl ThickenShellMesh {
    /// Why the values cannot be meshed, if they cannot.
    pub fn check(&self) -> Result<(), String> {
        if !(self.thickness.is_finite() && self.thickness > 0.0) {
            return Err("The thickness must be greater than 0".into());
        }
        if self.layers == 0 {
            return Err("The number of layers must be at least 1".into());
        }
        if !self.offset.is_finite() {
            return Err("The offset must be a number".into());
        }
        Ok(())
    }
}

/// How Gmsh joins the triangles of a face mesh to quadrilaterals.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Recombination {
    /// Triangles stay, so that an extrusion gives wedges only.
    None,
    Simple,
    #[default]
    Blossom,
    /// Quadrilaterals only: the recombined mesh is split once more, halving its size.
    SimpleFullQuad,
    BlossomFullQuad,
}

/// The element size in the extrude direction.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ExtrudeLayers {
    /// Layers as thick as the part's max. element size times the factor.
    ScaleFactor(f64),
    /// This many layers of the same thickness.
    NumberOfElements(u32),
    /// Layers of the given relative thicknesses, each divided into its number of elements.
    MultiLayered { sizes: Vec<f64>, elements: Vec<u32> },
}

impl ExtrudeLayers {
    /// The heights of the element layers' tops as fractions of the extrusion length,
    /// rising to 1; `max_size` is the part's max. element size, `length` the extrusion's.
    pub fn levels(&self, max_size: f64, length: f64) -> Result<Vec<f64>, String> {
        let even = |n: u32| (1..=n).map(|k| f64::from(k) / f64::from(n)).collect();
        match self {
            ExtrudeLayers::ScaleFactor(factor) => {
                let size = factor * max_size;
                if !(size.is_finite() && size > 0.0) {
                    return Err(
                        "The element size in the extrude direction must be greater than 0".into(),
                    );
                }
                let n = (length / size).round().clamp(1.0, 100_000.0) as u32;
                Ok(even(n))
            }
            ExtrudeLayers::NumberOfElements(n) if *n > 0 => Ok(even(*n)),
            ExtrudeLayers::NumberOfElements(_) => {
                Err("The number of elements must be at least 1".into())
            }
            ExtrudeLayers::MultiLayered { sizes, elements } => {
                if sizes.is_empty() || sizes.len() != elements.len() {
                    return Err("Give as many elements per layer as relative layer sizes".into());
                }
                if sizes.iter().any(|s| !(s.is_finite() && *s > 0.0)) || elements.contains(&0) {
                    return Err("Layer sizes and elements per layer must be positive".into());
                }
                let total: f64 = sizes.iter().sum();
                let mut levels = Vec::new();
                let mut bottom = 0.0;
                for (size, &n) in sizes.iter().zip(elements) {
                    for k in 1..=n {
                        levels.push((bottom + size * f64::from(k) / f64::from(n)) / total);
                    }
                    bottom += size;
                }
                if let Some(last) = levels.last_mut() {
                    *last = 1.0;
                }
                Ok(levels)
            }
        }
    }
}

/// Gmsh's algorithm for meshing the faces.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Algorithm2d {
    MeshAdapt,
    Automatic,
    Delaunay,
    #[default]
    FrontalDelaunay,
    /// Frontal-Delaunay for quadrilaterals, right-angled triangles that recombine well.
    FrontalQuads,
    PackingOfParallelograms,
    QuasiStructuredQuad,
}

/// Gmsh's algorithm for meshing the volume.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Algorithm3d {
    #[default]
    Delaunay,
    Frontal,
    Hxt,
}

/// Rounds to one significant digit, e.g. 0.0347 to 0.03.
fn round_size(size: f64) -> f64 {
    if !(size.is_finite() && size > 0.0) {
        return size;
    }
    let scale = 10f64.powf(size.log10().floor());
    (size / scale).round() * scale
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sizes_follow_the_diagonal() {
        let setup = MeshingParameters::for_diagonal(173.2);
        assert!((setup.max_size - 9.0).abs() < 1e-9, "{}", setup.max_size);
        assert!((setup.min_size - 0.2).abs() < 1e-9, "{}", setup.min_size);
        assert_eq!(round_size(0.0), 0.0);
    }

    #[test]
    fn parts_are_suppressed_by_name_once() {
        let mut geometry = Geometry {
            source: String::new(),
            brep: String::new(),
            meshing: MeshingParameters::default(),
            mesh_items: Vec::new(),
            part_names: Vec::new(),
            refinement: 1.0,
            signatures: Vec::new(),
            compounds: Vec::new(),
            suppressed: Vec::new(),
            own_meshes: Vec::new(),
            ids: Vec::new(),
            from_link: false,
            operations: Vec::new(),
            input: None,
        };
        let parts = ["SOLID-1".to_owned(), "solid-1".to_owned()];
        assert!(geometry.set_suppressed(&parts, true));
        assert_eq!(geometry.suppressed, [geometry.part_id("SOLID-1").unwrap()]);
        assert!(geometry.is_suppressed("Solid-1"));
        assert!(!geometry.set_suppressed(&parts, true));
        assert!(geometry.set_suppressed(&["solid-1".into()], false));
        assert!(!geometry.is_suppressed("SOLID-1"));
    }

    #[test]
    fn items_override_the_defaults_of_their_parts() {
        let fine = MeshingParameters {
            max_size: 1.0,
            ..MeshingParameters::default()
        };
        let mut geometry = Geometry {
            compounds: Vec::new(),
            suppressed: Vec::new(),
            source: "a.step".into(),
            brep: String::new(),
            meshing: MeshingParameters::for_diagonal(100.0),
            mesh_items: Vec::new(),
            part_names: Vec::new(),
            refinement: 1.0,
            signatures: Vec::new(),
            own_meshes: Vec::new(),
            ids: Vec::new(),
            from_link: false,
            operations: Vec::new(),
            input: None,
        };
        let ids = geometry.part_ids_of(&["SOLID-1".into(), "SOLID-2".into()]);
        geometry.mesh_items = vec![
            MeshSetupItem {
                id: crate::Id::new(),
                name: "Meshing_Parameters-1".into(),
                kind: MeshSetupKind::MeshingParameters {
                    parts: vec![ids[0]],
                    parameters: fine,
                },
            },
            MeshSetupItem {
                id: crate::Id::new(),
                name: "Tetrahedral_Gmsh-1".into(),
                kind: MeshSetupKind::TetrahedralGmsh {
                    parts: vec![ids[1]],
                    algorithm_2d: Algorithm2d::Delaunay,
                    algorithm_3d: Algorithm3d::Hxt,
                },
            },
            MeshSetupItem {
                id: crate::Id::new(),
                name: "Local_Mesh_Size-1".into(),
                kind: MeshSetupKind::LocalMeshSize {
                    faces: vec![3],
                    edges: vec![],
                    size: 0.5,
                    depth: 0.0,
                },
            },
        ];
        assert_eq!(geometry.parameters("solid-1"), fine);
        assert_eq!(geometry.parameters("SOLID-2").max_size, 5.0);
        assert_eq!(
            geometry.algorithms("SOLID-2"),
            (Algorithm2d::Delaunay, Algorithm3d::Hxt)
        );
        assert_eq!(geometry.algorithms("SOLID-1"), Default::default());
        let local: Vec<_> = geometry.local_sizes().collect();
        assert_eq!(local, vec![(&[3][..], &[][..], 0.5, 0.0)]);
        // A refinement divides every size; the sizes as set stay.
        geometry.refinement = 2.0;
        assert_eq!(geometry.parameters("SOLID-2").max_size, 2.5);
        let local: Vec<_> = geometry.local_sizes().collect();
        assert_eq!(local, vec![(&[3][..], &[][..], 0.25, 0.0)]);
        assert_eq!(geometry.meshing.max_size, 5.0);
        geometry.refinement = 1.0;
        geometry.mesh_items.clear();
        assert_eq!(geometry.parameters("SOLID-1").max_size, 5.0);
    }

    #[test]
    fn extrude_layers_rise_to_the_top() {
        let levels = ExtrudeLayers::ScaleFactor(0.5).levels(4.0, 10.0).unwrap();
        assert_eq!(levels, vec![0.2, 0.4, 0.6, 0.8, 1.0]);
        assert_eq!(
            ExtrudeLayers::NumberOfElements(2)
                .levels(4.0, 10.0)
                .unwrap(),
            vec![0.5, 1.0]
        );
        let layered = ExtrudeLayers::MultiLayered {
            sizes: vec![1.0, 3.0],
            elements: vec![1, 3],
        };
        assert_eq!(
            layered.levels(4.0, 10.0).unwrap(),
            vec![0.25, 0.5, 0.75, 1.0]
        );
        assert!(
            ExtrudeLayers::NumberOfElements(0)
                .levels(4.0, 10.0)
                .is_err()
        );
        assert!(ExtrudeLayers::ScaleFactor(0.0).levels(4.0, 10.0).is_err());
        let unequal = ExtrudeLayers::MultiLayered {
            sizes: vec![1.0],
            elements: vec![1, 2],
        };
        assert!(unequal.levels(4.0, 10.0).is_err());
    }

    #[test]
    fn projects_from_before_revolutions_keep_their_hexahedra_setting() {
        let parameters: MeshingParameters =
            ron::from_str("(extrude_where_possible: true)").unwrap();
        assert!(parameters.hexahedra_where_possible);
    }

    #[test]
    fn a_later_tetrahedral_item_takes_a_part_back_from_an_extrusion() {
        let extrude = |parts: &[crate::Id]| MeshSetupItem {
            id: crate::Id::new(),
            name: "Extrude_Mesh-1".into(),
            kind: MeshSetupKind::ExtrudeMesh(ExtrudeMesh {
                parts: parts.to_vec(),
                ..ExtrudeMesh::default()
            }),
        };
        let (a, b) = (crate::Id::new(), crate::Id::new());
        let tet = MeshSetupItem {
            id: crate::Id::new(),
            name: "Tetrahedral_Gmsh-1".into(),
            kind: MeshSetupKind::TetrahedralGmsh {
                parts: vec![a],
                algorithm_2d: Algorithm2d::default(),
                algorithm_3d: Algorithm3d::default(),
            },
        };
        let mut geometry = Geometry {
            compounds: Vec::new(),
            suppressed: Vec::new(),
            source: "a.step".into(),
            brep: String::new(),
            meshing: MeshingParameters::default(),
            mesh_items: vec![tet.clone(), extrude(&[a, b])],
            part_names: Vec::new(),
            refinement: 1.0,
            signatures: Vec::new(),
            own_meshes: Vec::new(),
            ids: vec![
                PartId {
                    name: "A".into(),
                    id: a,
                },
                PartId {
                    name: "B".into(),
                    id: b,
                },
            ],
            from_link: false,
            operations: Vec::new(),
            input: None,
        };
        assert!(geometry.extrusion("a").is_some());
        assert!(geometry.extrusion("C").is_none());
        geometry.mesh_items.push(tet);
        assert!(geometry.extrusion("A").is_none());
        assert!(geometry.extrusion("B").is_some());
    }
}
