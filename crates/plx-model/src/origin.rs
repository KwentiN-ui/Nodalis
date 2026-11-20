//! What each part of the mesh is the mesh of: every mesh belongs to a geometry object.
//!
//! A part of an analysis' mesh is either the mesh of a part of its CAD geometry, meshed by
//! Gmsh or taken over through a mesh link, or a mesh read without geometry, such as an input
//! file or a mesh block. Such an imported mesh is a geometry object of its own: the Geometry
//! workspace shows its outline, and it is suppressed, renamed and deleted like a part of the
//! geometry. [`PartOrigins`] records the origin of every part, by the id of the geometry
//! object it is the mesh of (the bodies of a compound part each by its own id; the mesh of
//! a part of the geometry has the id of the part, see [`crate::Geometry::stamp_ids`]), and
//! [`PartOrigins::reconcile`] keeps it complete: a part whose origin is not recorded is an
//! imported mesh, and the origin of a part that went is dropped with it. Renaming an object
//! keeps its origin.

use std::borrow::Cow;
use std::collections::BTreeMap;

use plx_mesh::FeMesh;
use serde::{Deserialize, Serialize};

use crate::Id;

/// Where the mesh of a geometry object comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PartOrigin {
    /// Meshed from a part of the CAD geometry, from the meshing inputs with this
    /// fingerprint ([`crate::fingerprint::mesh_input`]); out of date once they differ.
    Meshed(u64),
    /// The mesh of a part of the CAD geometry brought by a link with the part: the block it
    /// comes from makes it, so it is never out of date here. The analysis can mesh the part
    /// itself instead ([`crate::Geometry::own_meshes`]).
    Linked,
    /// A mesh without CAD shapes, which is its own geometry object: read from a file, made
    /// by a mesh block, or brought by a link.
    Imported {
        /// Whether it is suppressed: left out of the solver input until unsuppressed. Parts
        /// of the CAD geometry are suppressed in [`crate::Geometry::suppressed`].
        suppressed: bool,
        /// Whether a link brings it, so that it changes in the block it comes from.
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        linked: bool,
    },
}

/// The origin of every part of a mesh, by the id of its geometry object, see the
/// [module](self).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct PartOrigins(BTreeMap<Id, PartOrigin>);

/// The names of the geometry objects a part of the mesh is the mesh of: the bodies of a
/// compound part, else the part itself.
pub fn bodies(part: &plx_mesh::Part) -> Vec<&str> {
    if part.bodies.is_empty() {
        vec![part.name.as_str()]
    } else {
        part.bodies.iter().map(|b| b.name.as_str()).collect()
    }
}

/// The ids of the geometry objects a part of the mesh is the mesh of, as [`bodies`].
pub fn objects(part: &plx_mesh::Part) -> Vec<Id> {
    if part.bodies.is_empty() {
        vec![part.id]
    } else {
        part.bodies.iter().map(|b| b.id).collect()
    }
}

/// The geometry objects of `mesh` with their names, as [`bodies`] and [`objects`].
pub fn named_objects(mesh: &FeMesh) -> Vec<(Id, &str)> {
    (mesh.parts.iter())
        .flat_map(|part| objects(part).into_iter().zip(bodies(part)))
        .collect()
}

/// The id of the geometry object `name` of `mesh`, see [`named_objects`].
pub fn object_id(mesh: &FeMesh, name: &str) -> Option<Id> {
    (named_objects(mesh).into_iter())
        .find(|(_, n)| n.eq_ignore_ascii_case(name))
        .map(|(id, _)| id)
}

/// The ids of the geometry objects `names` of `mesh`: parts of one body and bodies of
/// compound parts, see [`bodies`].
pub fn body_ids(mesh: &FeMesh, names: &[String]) -> Vec<Id> {
    (named_objects(mesh).into_iter())
        .filter(|(_, name)| names.iter().any(|n| n.eq_ignore_ascii_case(name)))
        .map(|(id, _)| id)
        .collect()
}

impl PartOrigins {
    /// The origins of the parts of `mesh`, all of them imported meshes.
    pub fn imported(mesh: &FeMesh) -> Self {
        let mut origins = Self::default();
        origins.reconcile(mesh);
        origins
    }

    /// The origin of the geometry object `id`.
    pub fn get(&self, id: Id) -> Option<PartOrigin> {
        self.0.get(&id).copied()
    }

    /// The fingerprint of the meshing inputs the part `id` of the geometry was meshed
    /// from; `None` for a linked or imported mesh.
    pub fn made_from(&self, id: Id) -> Option<u64> {
        match self.get(id)? {
            PartOrigin::Meshed(input) => Some(input),
            _ => None,
        }
    }

    /// Whether `id` is an imported mesh, a geometry object without CAD shapes.
    pub fn is_imported(&self, id: Id) -> bool {
        matches!(self.get(id), Some(PartOrigin::Imported { .. }))
    }

    /// Whether `id` is the mesh of a part of the CAD geometry.
    pub fn is_cad(&self, id: Id) -> bool {
        matches!(
            self.get(id),
            Some(PartOrigin::Meshed(_) | PartOrigin::Linked)
        )
    }

    /// Whether the imported mesh `id` is suppressed.
    pub fn is_suppressed(&self, id: Id) -> bool {
        matches!(
            self.get(id),
            Some(PartOrigin::Imported {
                suppressed: true,
                ..
            })
        )
    }

    /// The ids of the imported meshes, in order.
    pub fn imported_ids(&self) -> impl Iterator<Item = Id> + '_ {
        (self.0.iter())
            .filter(|(_, o)| matches!(o, PartOrigin::Imported { .. }))
            .map(|(k, _)| *k)
    }

    /// Sets the origin of `id`, e.g. of a part of a mesh being put together.
    pub fn insert(&mut self, id: Id, origin: PartOrigin) {
        self.0.insert(id, origin);
    }

    /// Notes that the part `id` of the geometry was meshed from inputs with the
    /// fingerprint `input`.
    pub fn set_meshed(&mut self, id: Id, input: u64) {
        self.0.insert(id, PartOrigin::Meshed(input));
    }

    /// The fingerprints of the parts meshed from the geometry, to follow a change of the
    /// meshing inputs that leaves their meshes as they are.
    pub fn made_from_mut(&mut self) -> impl Iterator<Item = (Id, &mut u64)> {
        (self.0.iter_mut()).filter_map(|(k, o)| match o {
            PartOrigin::Meshed(input) => Some((*k, input)),
            _ => None,
        })
    }

    /// Suppresses the imported meshes among `ids`, or unsuppresses them; true if that
    /// changed anything. Ids of other objects are left alone.
    pub fn set_suppressed(&mut self, ids: &[Id], suppressed: bool) -> bool {
        let mut changed = false;
        for (key, origin) in &mut self.0 {
            if let PartOrigin::Imported { suppressed: s, .. } = origin
                && ids.contains(key)
                && *s != suppressed
            {
                *s = suppressed;
                changed = true;
            }
        }
        changed
    }

    /// The origins as a link hands them on to an analysis that had `before`: the meshes of
    /// CAD parts are made by the source, so they are linked meshes in the target, and meshes
    /// without CAD shapes are linked imported meshes, suppressed there as they were.
    pub fn linked(&self, before: &Self) -> Self {
        Self(
            (self.0.iter())
                .map(|(k, o)| {
                    let o = match o {
                        PartOrigin::Meshed(_) | PartOrigin::Linked => PartOrigin::Linked,
                        PartOrigin::Imported { .. } => PartOrigin::Imported {
                            suppressed: before.is_suppressed(*k),
                            linked: true,
                        },
                    };
                    (*k, o)
                })
                .collect(),
        )
    }

    /// Whether a link brings the mesh of `id`.
    pub fn is_linked(&self, id: Id) -> bool {
        matches!(
            self.get(id),
            Some(PartOrigin::Linked | PartOrigin::Imported { linked: true, .. })
        )
    }

    /// Keeps the origins complete for `mesh`: a part (or body of a compound part) without a
    /// recorded origin is an imported mesh, and the origins of objects without a mesh go.
    /// Returns the ids of the parts that became imported meshes.
    pub fn reconcile(&mut self, mesh: &FeMesh) -> Vec<Id> {
        let mut present: BTreeMap<Id, PartOrigin> = BTreeMap::new();
        let mut added = Vec::new();
        for part in &mesh.parts {
            for body in objects(part) {
                let origin = match self.0.get(&body) {
                    Some(origin) => *origin,
                    None => {
                        added.push(body);
                        PartOrigin::Imported {
                            suppressed: false,
                            linked: false,
                        }
                    }
                };
                present.insert(body, origin);
            }
        }
        self.0 = present;
        added
    }

    /// Makes the origins of the CAD parts agree with who meshes them: in a geometry a link
    /// brings ([`crate::Geometry::from_link`]) the analysis meshes the parts of
    /// [`crate::Geometry::own_meshes`], so a linked mesh of one of them is out of date until
    /// it is meshed here, and the link makes the meshes of the others, so they are linked.
    /// Whether a part's mesh is linked thus follows from the geometry alone.
    pub fn settle(&mut self, geometry: Option<&crate::Geometry>) {
        let Some(geometry) = geometry.filter(|g| g.from_link) else {
            return;
        };
        for (id, origin) in &mut self.0 {
            let own = geometry.own_meshes.contains(id);
            match *origin {
                PartOrigin::Linked if own => *origin = PartOrigin::Meshed(0),
                PartOrigin::Meshed(_) if !own => *origin = PartOrigin::Linked,
                _ => {}
            }
        }
    }

    /// Whether every part of `mesh` has its origin and no origin is without its part.
    pub fn complete_for(&self, mesh: &FeMesh) -> bool {
        let ids: Vec<Id> = mesh.parts.iter().flat_map(objects).collect();
        ids.len() == self.0.len() && ids.iter().all(|id| self.0.contains_key(id))
    }

    /// Moves the origins of the objects whose ids changed, as (old, new), to their new ids,
    /// see [`crate::Geometry::adopt_ids`].
    pub fn rekey(&mut self, changed: &[(Id, Id)]) {
        for (old, new) in changed {
            if let Some(origin) = self.0.remove(old) {
                self.0.insert(*new, origin);
            }
        }
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = (Id, PartOrigin)> + '_ {
        self.0.iter().map(|(k, o)| (*k, *o))
    }
}

/// The meshes of parts of the geometry that `geometry` no longer has, e.g. after a split
/// that cut a part into new ones or a deleted part, by id: deleting a part of the geometry
/// deletes its mesh. Unknown while the parts of the geometry are not known
/// ([`crate::Geometry::known_parts`]).
pub fn orphans(geometry: Option<&crate::Geometry>, origins: &PartOrigins) -> Vec<Id> {
    let known = |id: Id| match geometry {
        None => false,
        Some(g) => g.known_parts().is_empty() || g.part_name(id).is_some(),
    };
    (origins.iter())
        .filter(|(id, origin)| !matches!(origin, PartOrigin::Imported { .. }) && !known(*id))
        .map(|(id, _)| id)
        .collect()
}

/// Deletes the meshes of parts that `geometry` no longer has ([`orphans`]) with the regions
/// of the parts of the FE model that go whole and the items on them alone; returns their
/// names.
pub fn drop_orphans(
    mesh: &mut FeMesh,
    fe: &mut crate::FeModel,
    geometry: Option<&crate::Geometry>,
    origins: &mut PartOrigins,
) -> Vec<String> {
    let orphans = orphans(geometry, origins);
    if orphans.is_empty() {
        return Vec::new();
    }
    let names: Vec<String> = (named_objects(mesh).into_iter())
        .filter(|(id, _)| orphans.contains(id))
        .map(|(_, name)| name.to_owned())
        .collect();
    let gone: Vec<Id> = (mesh.parts.iter())
        .filter(|p| objects(p).iter().all(|b| orphans.contains(b)))
        .map(|p| p.id)
        .collect();
    let full = mesh.clone();
    mesh.remove_bodies(&orphans);
    // The items on them alone, e.g. contacts with them, go with them.
    fe.drop_items_left_empty(&full, mesh);
    fe.remove_parts_from_regions(&gone);
    origins.reconcile(mesh);
    names
}

/// The parts of the FE model that `geometry` has without a mesh: every part of the geometry
/// that is not suppressed belongs to the FE model, meshed or not, so the FE Model lists those
/// without a mesh too. By FE name (a compound part once), in the order of the geometry; a
/// compound part whose mesh lacks only some of its bodies has a mesh and is not among them.
pub fn unmeshed_parts(
    geometry: Option<&crate::Geometry>,
    origins: &PartOrigins,
    mesh: &FeMesh,
) -> Vec<String> {
    let Some(geometry) = geometry else {
        return Vec::new();
    };
    let mut unmeshed: Vec<String> = Vec::new();
    for part in geometry.known_parts() {
        let meshed = geometry
            .part_id(part)
            .is_some_and(|id| origins.get(id).is_some());
        if geometry.is_suppressed(part) || meshed {
            continue;
        }
        let name = geometry.fe_part_name(part);
        let listed = (unmeshed.iter().map(String::as_str))
            .chain(mesh.parts.iter().map(|p| p.name.as_str()))
            .any(|n| n.eq_ignore_ascii_case(name));
        if !listed {
            unmeshed.push(name.to_owned());
        }
    }
    unmeshed
}

/// Whether the geometry object `id` is suppressed: a part of `geometry` suppressed there,
/// or an imported mesh suppressed in `origins`.
pub fn is_suppressed(id: Id, geometry: Option<&crate::Geometry>, origins: &PartOrigins) -> bool {
    origins.is_suppressed(id) || geometry.is_some_and(|g| g.suppressed.contains(&id))
}

/// The geometry objects of `mesh` that are suppressed, by id: parts of the geometry
/// suppressed in `geometry`, imported meshes suppressed in `origins`.
pub fn suppressed_bodies(
    mesh: &FeMesh,
    geometry: Option<&crate::Geometry>,
    origins: &PartOrigins,
) -> Vec<Id> {
    (mesh.parts.iter().flat_map(objects))
        .filter(|&body| is_suppressed(body, geometry, origins))
        .collect()
}

/// Of each part of a mesh, whether it is left out of the analysis: all its geometry objects
/// are suppressed. Contact detection and part filters take it, made by [`LeftOut::of`] from
/// the geometry and the imported meshes with their suppression, so that no search can
/// forget the suppressed parts.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LeftOut(pub(crate) Vec<bool>);

impl LeftOut {
    /// The parts of `mesh` (of an FE model, or the display of a geometry) whose geometry
    /// objects are all suppressed: parts of `geometry` suppressed there, imported meshes
    /// suppressed in `origins`.
    pub fn of(mesh: &FeMesh, geometry: Option<&crate::Geometry>, origins: &PartOrigins) -> Self {
        let parts: Vec<bool> = (mesh.parts.iter())
            .map(|part| {
                (objects(part).into_iter()).all(|body| is_suppressed(body, geometry, origins))
            })
            .collect();
        Self(if parts.contains(&true) {
            parts
        } else {
            Vec::new()
        })
    }

    /// Nothing left out: for a mesh without a geometry or imported meshes, e.g. one read
    /// from an input file, and for the parts of an [`Active`] mesh, which has none left.
    pub fn nothing() -> Self {
        Self::default()
    }

    /// Whether the part `part`, by index, is left out.
    pub fn contains(&self, part: usize) -> bool {
        self.0.get(part).copied().unwrap_or(false)
    }
}

/// What is solved of an analysis: its mesh without the suppressed geometry objects, and its
/// FE model without them in its regions of parts and without the items on them alone.
/// Borrowed while nothing is suppressed.
///
/// The solver input (`plx_io::inp`), the particle model and the fingerprints of what is
/// solved take only this type, and it is made only by [`active_model`], from the geometry
/// and the imported meshes with their suppression: a suppressed object cannot reach the
/// solver by a call that forgets to leave it out. Its fields are private for that.
#[derive(Clone, Debug)]
pub struct Active<'a> {
    mesh: Cow<'a, FeMesh>,
    fe: Cow<'a, crate::FeModel>,
    /// The parts of the mesh left out, whole; bodies of compound parts are left out of
    /// their part.
    left_out: Vec<String>,
}

impl<'a> Active<'a> {
    /// The mesh solved: without the suppressed objects.
    pub fn mesh(&self) -> &FeMesh {
        &self.mesh
    }

    /// The FE model solved: without the suppressed objects in its regions, nor the items on
    /// them alone.
    pub fn fe(&self) -> &crate::FeModel {
        &self.fe
    }

    /// The FE model to change before it is solved, e.g. with the files of linked results;
    /// the mesh stays without the suppressed objects.
    pub fn fe_mut(&mut self) -> &mut crate::FeModel {
        self.fe.to_mut()
    }

    /// The same mesh with the FE model `fe`, a variant of [`Self::fe`] such as the model at
    /// another speed of a sweep.
    pub fn with_fe(&self, fe: crate::FeModel) -> Active<'_> {
        Active {
            mesh: Cow::Borrowed(&*self.mesh),
            fe: Cow::Owned(fe),
            left_out: self.left_out.clone(),
        }
    }

    /// The parts of the mesh left out whole, by name.
    pub fn left_out(&self) -> &[String] {
        &self.left_out
    }

    /// Owns what it borrowed, to be kept beyond the model it was made of.
    pub fn into_owned(self) -> Active<'static> {
        Active {
            mesh: Cow::Owned(self.mesh.into_owned()),
            fe: Cow::Owned(self.fe.into_owned()),
            left_out: self.left_out,
        }
    }
}

/// The mesh and FE model the solver gets, see [`Active`]: a suppressed geometry object
/// takes part in nothing, with its geometry and its mesh.
pub fn active_model<'a>(
    mesh: &'a FeMesh,
    fe: &'a crate::FeModel,
    geometry: Option<&crate::Geometry>,
    origins: &PartOrigins,
) -> Active<'a> {
    let suppressed = suppressed_bodies(mesh, geometry, origins);
    if suppressed.is_empty() {
        return Active {
            mesh: Cow::Borrowed(mesh),
            fe: Cow::Borrowed(fe),
            left_out: Vec::new(),
        };
    }
    let gone = |p: &&plx_mesh::Part| objects(p).iter().all(|b| suppressed.contains(b));
    let left_out: Vec<String> = (mesh.parts.iter().filter(gone))
        .map(|p| p.name.clone())
        .collect();
    let left_out_ids: Vec<Id> = mesh.parts.iter().filter(gone).map(|p| p.id).collect();
    let mut smaller = mesh.clone();
    smaller.remove_bodies(&suppressed);
    let mut fe = fe.clone();
    // An item on suppressed objects alone, e.g. a contact found with a suppressed part
    // before it was suppressed, goes with them; one on others too keeps those.
    fe.drop_items_left_empty(mesh, &smaller);
    fe.remove_parts_from_regions(&left_out_ids);
    Active {
        mesh: Cow::Owned(smaller),
        fe: Cow::Owned(fe),
        left_out,
    }
}

#[cfg(test)]
mod tests {
    use plx_mesh::{Element, ElementShape, Part};

    use super::*;
    use crate::named;

    fn mesh(parts: &[&str]) -> FeMesh {
        let mut mesh = FeMesh::default();
        for (i, name) in parts.iter().enumerate() {
            let base = 4 * i as u32;
            for n in 1..=4 {
                mesh.set_node(base + n, [f64::from(n), i as f64, 0.0]);
            }
            let id = i as u32 + 1;
            mesh.add_element(Element {
                id,
                type_name: "C3D4".into(),
                shape: ElementShape::Tet4,
                nodes: (1..=4).map(|n| base + n).collect(),
            })
            .unwrap();
            mesh.parts.push(Part::with_id(named(name), *name, vec![id]));
        }
        mesh
    }

    #[test]
    fn a_part_without_an_origin_is_an_imported_mesh() {
        let mut origins = PartOrigins::default();
        origins.set_meshed(named("SOLID-1"), 7);
        let added = origins.reconcile(&mesh(&["SOLID-1", "PART-1"]));
        assert_eq!(added, [named("PART-1")]);
        assert_eq!(origins.made_from(named("SOLID-1")), Some(7));
        assert!(origins.is_imported(named("PART-1")));
        assert!(origins.is_cad(named("SOLID-1")));
    }

    #[test]
    fn the_origin_of_a_part_that_went_goes_with_it() {
        let mut origins = PartOrigins::default();
        origins.set_meshed(named("SOLID-1"), 7);
        origins.set_meshed(named("SOLID-2"), 8);
        origins.reconcile(&mesh(&["SOLID-2"]));
        assert_eq!(origins.get(named("SOLID-1")), None);
        assert!(origins.complete_for(&mesh(&["SOLID-2"])));
    }

    #[test]
    fn the_bodies_of_a_compound_part_each_have_their_origin() {
        let mut compound = mesh(&["A", "B"]);
        let bodies = (compound.parts.drain(..))
            .map(|p| plx_mesh::Body {
                id: p.id,
                name: p.name,
                elements: p.elements,
            })
            .collect::<Vec<_>>();
        let mut part = Part::new("C", vec![1, 2]);
        part.bodies = bodies;
        compound.parts.push(part);
        let mut origins = PartOrigins::default();
        origins.set_meshed(named("A"), 1);
        origins.reconcile(&compound);
        assert!(origins.is_cad(named("A")));
        assert!(origins.is_imported(named("B")));
        assert_eq!(origins.get(named("C")), None);
    }

    #[test]
    fn a_suppressed_body_is_left_out_of_its_compound_part() {
        let mut compound = mesh(&["A", "B"]);
        let bodies = (compound.parts.drain(..))
            .map(|p| plx_mesh::Body {
                id: p.id,
                name: p.name,
                elements: p.elements,
            })
            .collect::<Vec<_>>();
        let mut part = Part::new("C", vec![1, 2]);
        part.bodies = bodies;
        compound.parts.push(part);
        let mut origins = PartOrigins::default();
        origins.set_meshed(named("A"), 1);
        origins.set_meshed(named("B"), 2);
        let mut geometry = crate::Geometry {
            source: String::new(),
            brep: String::new(),
            meshing: crate::MeshingParameters::default(),
            mesh_items: Vec::new(),
            part_names: vec!["A".into(), "B".into()],
            refinement: 1.0,
            signatures: Vec::new(),
            compounds: Vec::new(),
            suppressed: vec![named("A")],
            own_meshes: Vec::new(),
            ids: Vec::new(),
            from_link: false,
            operations: Vec::new(),
            input: None,
        };
        let fe = crate::FeModel::default();
        let solved = active_model(&compound, &fe, Some(&geometry), &origins);
        assert!(solved.left_out.is_empty());
        assert_eq!(solved.mesh.parts[0].elements, [2]);
        assert_eq!(solved.mesh.parts[0].bodies.len(), 1);
        assert_eq!(solved.mesh.node_count(), 4);
        geometry.suppressed.push(named("B"));
        let solved = active_model(&compound, &fe, Some(&geometry), &origins);
        assert_eq!(solved.left_out, ["C"]);
        assert!(solved.mesh.parts.is_empty());
    }

    #[test]
    fn only_imported_meshes_are_suppressed_here() {
        let mut origins = PartOrigins::default();
        origins.set_meshed(named("SOLID-1"), 7);
        origins.reconcile(&mesh(&["SOLID-1", "PART-1"]));
        let names = [named("SOLID-1"), named("PART-1")];
        assert!(origins.set_suppressed(&names, true));
        assert!(origins.is_suppressed(named("PART-1")));
        assert!(!origins.is_suppressed(named("SOLID-1")));
        assert!(!origins.set_suppressed(&names, true));
    }

    #[test]
    fn suppressed_objects_are_left_out_of_what_is_solved() {
        let mesh = mesh(&["SOLID-1", "PART-1"]);
        let mut origins = PartOrigins::default();
        origins.set_meshed(named("SOLID-1"), 7);
        origins.reconcile(&mesh);
        let mut fe = crate::FeModel::default();
        fe.mesh_sets.push(crate::MeshSet::new(
            "BOTH",
            crate::SetKind::Elements,
            crate::Region::Parts(mesh.ids_of(&["SOLID-1", "PART-1"])),
        ));
        let mut geometry = crate::Geometry {
            source: String::new(),
            brep: String::new(),
            meshing: crate::MeshingParameters::default(),
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
        let solved = active_model(&mesh, &fe, Some(&geometry), &origins);
        assert!(matches!(solved.mesh, Cow::Borrowed(_)));
        geometry.suppressed.push(named("SOLID-1"));
        origins.set_suppressed(&[named("PART-1")], true);
        let solved = active_model(&mesh, &fe, Some(&geometry), &origins);
        assert_eq!(solved.left_out, ["SOLID-1", "PART-1"]);
        assert_eq!(solved.mesh.element_count(), 0);
        assert_eq!(
            solved.fe.mesh_sets[0].region,
            crate::Region::Parts(Vec::new())
        );
        origins.set_suppressed(&[named("PART-1")], false);
        let solved = active_model(&mesh, &fe, Some(&geometry), &origins);
        assert_eq!(solved.left_out, ["SOLID-1"]);
        assert_eq!(solved.mesh.parts[0].name, "PART-1");
        assert_eq!(solved.mesh.node_count(), 4);
    }

    /// A contact found with a part before it was suppressed reaches the solver with nothing
    /// on one side unless it goes with the part; a contact between parts that stay, and a
    /// load also on a part that stays, are kept.
    #[test]
    fn items_on_suppressed_objects_alone_are_left_out_of_what_is_solved() {
        use crate::Region;
        let mesh = mesh(&["PART-1", "PART-2", "PART-3"]);
        let mut origins = PartOrigins::default();
        origins.reconcile(&mesh);
        let faces = |element: u32| Region::Faces(vec![(element, 1)]);
        let contact = |a: u32, b: u32| crate::ContactRegion {
            side_a: faces(a),
            side_b: faces(b),
            ..crate::ContactRegion::new(format!("Contact {a}-{b}"), crate::ContactBehavior::Bonded)
        };
        let mut fe = crate::FeModel {
            contact_regions: vec![contact(1, 3), contact(1, 2)],
            ..Default::default()
        };
        let mut step = crate::Step::new_static("Step-1");
        for (name, nodes) in [("On 3", vec![9, 10]), ("On 2 and 3", vec![5, 9])] {
            step.boundary_conditions.push(crate::BoundaryCondition::new(
                name,
                Region::Nodes(nodes),
                crate::BoundaryKind::Temperature(20.0),
            ));
        }
        fe.steps.push(step);
        origins.set_suppressed(&[named("PART-3")], true);
        let solved = active_model(&mesh, &fe, None, &origins);
        assert_eq!(solved.left_out(), ["PART-3"]);
        let names = |fe: &crate::FeModel| -> Vec<String> {
            (fe.contact_regions.iter().map(|c| c.name.clone()))
                .chain(
                    fe.steps[0]
                        .boundary_conditions
                        .iter()
                        .map(|b| b.name.clone()),
                )
                .collect()
        };
        assert_eq!(names(solved.fe()), ["Contact 1-2", "On 2 and 3"]);
        // The model itself keeps them for when the part is unsuppressed.
        assert_eq!(names(&fe).len(), 4);
        // What a filter of the detection sees: the suppressed part matches none.
        let left_out = LeftOut::of(&mesh, None, &origins);
        assert_eq!(
            (0..3).map(|p| left_out.contains(p)).collect::<Vec<_>>(),
            [false, false, true]
        );
        let context = fe.part_context(&mesh, &left_out);
        let named = crate::PartFilter {
            include: vec![crate::PartTerm::Part(mesh.parts[2].id)],
            exclude: Vec::new(),
        };
        assert_eq!(named.searched(&context, &[true; 3]), [false; 3]);
    }

    #[test]
    fn the_mesh_of_a_part_the_geometry_no_longer_has_goes() {
        let mut mesh = mesh(&["SOLID-1", "SOLID-2", "PART-1"]);
        let mut origins = PartOrigins::default();
        origins.set_meshed(named("SOLID-1"), 7);
        origins.set_meshed(named("SOLID-2"), 8);
        origins.reconcile(&mesh);
        let geometry = crate::Geometry {
            source: String::new(),
            brep: String::new(),
            meshing: crate::MeshingParameters::default(),
            mesh_items: Vec::new(),
            part_names: vec!["SOLID-2".into(), "SOLID-3".into()],
            refinement: 1.0,
            signatures: Vec::new(),
            compounds: Vec::new(),
            suppressed: Vec::new(),
            own_meshes: Vec::new(),
            ids: ["SOLID-2", "SOLID-3"]
                .map(|name| crate::PartId {
                    name: name.into(),
                    id: named(name),
                })
                .to_vec(),
            from_link: false,
            operations: Vec::new(),
            input: None,
        };
        let mut fe = crate::FeModel::default();
        // A contact with the part that goes goes with it; one between parts that stay stays.
        let faces = |element: u32| crate::Region::Faces(vec![(element, 1)]);
        let contact = |a: u32, b: u32| crate::ContactRegion {
            side_a: faces(a),
            side_b: faces(b),
            ..crate::ContactRegion::new(format!("{a}-{b}"), crate::ContactBehavior::Bonded)
        };
        fe.contact_regions = vec![contact(1, 2), contact(2, 3)];
        let gone = drop_orphans(&mut mesh, &mut fe, Some(&geometry), &mut origins);
        assert_eq!(gone, ["SOLID-1"]);
        assert_eq!(fe.contact_regions.len(), 1);
        assert_eq!(fe.contact_regions[0].name, "2-3");
        let names: Vec<&str> = mesh.parts.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(names, ["SOLID-2", "PART-1"]);
        assert!(origins.complete_for(&mesh));
        // Without geometry, only the imported meshes stay.
        drop_orphans(&mut mesh, &mut fe, None, &mut origins);
        assert_eq!(mesh.parts.len(), 1);
        assert!(origins.is_imported(named("PART-1")));
    }

    #[test]
    fn a_mesh_link_hands_on_meshed_parts_as_linked() {
        let mut origins = PartOrigins::default();
        origins.set_meshed(named("SOLID-1"), 7);
        origins.reconcile(&mesh(&["SOLID-1", "PART-1"]));
        let mut before = PartOrigins::imported(&mesh(&["PART-1"]));
        before.set_suppressed(&[named("PART-1")], true);
        let linked = origins.linked(&before);
        assert_eq!(linked.get(named("SOLID-1")), Some(PartOrigin::Linked));
        assert!(linked.is_imported(named("PART-1")));
        assert!(linked.is_linked(named("PART-1")) && linked.is_linked(named("SOLID-1")));
        // The target keeps its own suppression of a linked object.
        assert!(linked.is_suppressed(named("PART-1")));
        assert_eq!(linked.made_from(named("SOLID-1")), None);
    }

    #[test]
    fn in_linked_shapes_the_own_meshes_decide_what_is_linked() {
        let mut geometry = crate::Geometry {
            source: String::new(),
            brep: String::new(),
            meshing: Default::default(),
            mesh_items: Vec::new(),
            part_names: vec!["OWN".into(), "OTHER".into()],
            refinement: 1.0,
            signatures: Vec::new(),
            compounds: Vec::new(),
            suppressed: Vec::new(),
            own_meshes: vec![named("OWN")],
            ids: Vec::new(),
            from_link: true,
            operations: Vec::new(),
            input: None,
        };
        let mut origins = PartOrigins::default();
        origins.insert(named("OWN"), PartOrigin::Linked);
        origins.insert(named("OTHER"), PartOrigin::Meshed(3));
        origins.settle(Some(&geometry));
        // The link's mesh of a part meshed here is out of date, the own mesh of a part the
        // link meshes is the link's.
        assert_eq!(origins.get(named("OWN")), Some(PartOrigin::Meshed(0)));
        assert_eq!(origins.get(named("OTHER")), Some(PartOrigin::Linked));
        // Shapes of the analysis' own leave the origins alone.
        geometry.from_link = false;
        origins.insert(named("OTHER"), PartOrigin::Meshed(3));
        origins.settle(Some(&geometry));
        assert_eq!(origins.get(named("OTHER")), Some(PartOrigin::Meshed(3)));
    }

    /// Every part of the geometry that is not suppressed is a part of the FE model: those
    /// without a mesh are listed by their FE name, a compound part once.
    #[test]
    fn unmeshed_parts_are_the_unsuppressed_parts_without_a_mesh() {
        let mut origins = PartOrigins::default();
        origins.set_meshed(named("A"), 1);
        let mesh = mesh(&["A"]);
        let geometry = crate::Geometry {
            source: String::new(),
            brep: String::new(),
            meshing: crate::MeshingParameters::default(),
            mesh_items: Vec::new(),
            part_names: ["A", "B", "C", "D", "E"].map(String::from).to_vec(),
            refinement: 1.0,
            signatures: Vec::new(),
            compounds: vec![crate::CompoundPart::new(
                "JOINED",
                vec![crate::named("D"), crate::named("E")],
            )],
            suppressed: vec![named("C")],
            own_meshes: Vec::new(),
            ids: ["A", "B", "C", "D", "E"]
                .map(|name| crate::PartId {
                    name: name.into(),
                    id: crate::named(name),
                })
                .to_vec(),
            from_link: false,
            operations: Vec::new(),
            input: None,
        };
        assert_eq!(
            unmeshed_parts(Some(&geometry), &origins, &mesh),
            ["B", "JOINED"]
        );
        assert!(unmeshed_parts(None, &origins, &mesh).is_empty());
    }
}
