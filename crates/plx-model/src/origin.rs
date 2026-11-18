//! What each part of the mesh is the mesh of: every mesh belongs to a geometry object.
//!
//! A part of an analysis' mesh is either the mesh of a part of its CAD geometry, meshed by
//! Gmsh or taken over through a mesh link, or a mesh read without geometry, such as an input
//! file or a mesh block. Such an imported mesh is a geometry object of its own: the Geometry
//! workspace shows its outline, and it is suppressed, renamed and deleted like a part of the
//! geometry. [`PartOrigins`] records the origin of every part, by the name of the geometry
//! object it is the mesh of (the bodies of a compound part each by its own name), and
//! [`PartOrigins::reconcile`] keeps it complete: a part whose origin is not recorded is an
//! imported mesh, and the origin of a part that went is dropped with it.

use std::borrow::Cow;
use std::collections::BTreeMap;

use plx_mesh::FeMesh;
use serde::{Deserialize, Serialize};

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

/// The origin of every part of a mesh, see the [module](self).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct PartOrigins(BTreeMap<String, PartOrigin>);

/// The names of the geometry objects a part of the mesh is the mesh of: the bodies of a
/// compound part, else the part itself.
pub fn bodies(part: &plx_mesh::Part) -> Vec<&str> {
    if part.bodies.is_empty() {
        vec![part.name.as_str()]
    } else {
        part.bodies.iter().map(|b| b.name.as_str()).collect()
    }
}

impl PartOrigins {
    /// The origins of the parts of `mesh`, all of them imported meshes.
    pub fn imported(mesh: &FeMesh) -> Self {
        let mut origins = Self::default();
        origins.reconcile(mesh);
        origins
    }

    /// The origin of the geometry object `name`, in any case.
    pub fn get(&self, name: &str) -> Option<PartOrigin> {
        (self.0.get(name).copied()).or_else(|| {
            (self.0.iter())
                .find(|(k, _)| k.eq_ignore_ascii_case(name))
                .map(|(_, o)| *o)
        })
    }

    fn key(&self, name: &str) -> Option<String> {
        (self.0.keys())
            .find(|k| k.eq_ignore_ascii_case(name))
            .cloned()
    }

    /// The fingerprint of the meshing inputs the part `name` of the geometry was meshed
    /// from; `None` for a linked or imported mesh.
    pub fn made_from(&self, name: &str) -> Option<u64> {
        match self.get(name)? {
            PartOrigin::Meshed(input) => Some(input),
            _ => None,
        }
    }

    /// Whether `name` is an imported mesh, a geometry object without CAD shapes.
    pub fn is_imported(&self, name: &str) -> bool {
        matches!(self.get(name), Some(PartOrigin::Imported { .. }))
    }

    /// Whether `name` is the mesh of a part of the CAD geometry.
    pub fn is_cad(&self, name: &str) -> bool {
        matches!(
            self.get(name),
            Some(PartOrigin::Meshed(_) | PartOrigin::Linked)
        )
    }

    /// Whether the imported mesh `name` is suppressed.
    pub fn is_suppressed(&self, name: &str) -> bool {
        matches!(
            self.get(name),
            Some(PartOrigin::Imported {
                suppressed: true,
                ..
            })
        )
    }

    /// The names of the imported meshes, in order.
    pub fn imported_names(&self) -> impl Iterator<Item = &str> {
        (self.0.iter())
            .filter(|(_, o)| matches!(o, PartOrigin::Imported { .. }))
            .map(|(k, _)| k.as_str())
    }

    /// Sets the origin of `name`, e.g. of a part of a mesh being put together.
    pub fn insert(&mut self, name: &str, origin: PartOrigin) {
        let key = self.key(name).unwrap_or_else(|| name.to_owned());
        self.0.insert(key, origin);
    }

    /// Notes that the part `name` of the geometry was meshed from inputs with the
    /// fingerprint `input`.
    pub fn set_meshed(&mut self, name: &str, input: u64) {
        let key = self.key(name).unwrap_or_else(|| name.to_owned());
        self.0.insert(key, PartOrigin::Meshed(input));
    }

    /// The fingerprints of the parts meshed from the geometry, to follow a change of the
    /// meshing inputs that leaves their meshes as they are.
    pub fn made_from_mut(&mut self) -> impl Iterator<Item = (&str, &mut u64)> {
        (self.0.iter_mut()).filter_map(|(k, o)| match o {
            PartOrigin::Meshed(input) => Some((k.as_str(), input)),
            _ => None,
        })
    }

    /// Suppresses the imported meshes among `names`, or unsuppresses them; true if that
    /// changed anything. Names of other objects are left alone.
    pub fn set_suppressed(&mut self, names: &[String], suppressed: bool) -> bool {
        let mut changed = false;
        for (key, origin) in &mut self.0 {
            if let PartOrigin::Imported { suppressed: s, .. } = origin
                && names.iter().any(|n| n.eq_ignore_ascii_case(key))
                && *s != suppressed
            {
                *s = suppressed;
                changed = true;
            }
        }
        changed
    }

    /// Follows a renamed geometry object.
    pub fn rename(&mut self, old: &str, new: &str) {
        if let Some(key) = self.key(old)
            && let Some(origin) = self.0.remove(&key)
        {
            self.0.insert(new.to_owned(), origin);
        }
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
                            suppressed: before.is_suppressed(k),
                            linked: true,
                        },
                    };
                    (k.clone(), o)
                })
                .collect(),
        )
    }

    /// Whether a link brings the mesh of `name`.
    pub fn is_linked(&self, name: &str) -> bool {
        matches!(
            self.get(name),
            Some(PartOrigin::Linked | PartOrigin::Imported { linked: true, .. })
        )
    }

    /// Keeps the origins complete for `mesh`: a part (or body of a compound part) without a
    /// recorded origin is an imported mesh, and the origins of objects without a mesh go.
    /// Returns the names of the parts that became imported meshes.
    pub fn reconcile(&mut self, mesh: &FeMesh) -> Vec<String> {
        let mut present: BTreeMap<String, PartOrigin> = BTreeMap::new();
        let mut added = Vec::new();
        for part in &mesh.parts {
            for body in bodies(part) {
                let origin = match self.key(body) {
                    Some(key) => self.0[&key],
                    None => {
                        added.push(body.to_owned());
                        PartOrigin::Imported {
                            suppressed: false,
                            linked: false,
                        }
                    }
                };
                present.insert(body.to_owned(), origin);
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
        for (name, origin) in &mut self.0 {
            let own = geometry.meshes_itself(name);
            match *origin {
                PartOrigin::Linked if own => *origin = PartOrigin::Meshed(0),
                PartOrigin::Meshed(_) if !own => *origin = PartOrigin::Linked,
                _ => {}
            }
        }
    }

    /// Whether every part of `mesh` has its origin and no origin is without its part.
    pub fn complete_for(&self, mesh: &FeMesh) -> bool {
        let names: Vec<&str> = mesh.parts.iter().flat_map(bodies).collect();
        names.len() == self.0.len() && names.iter().all(|n| self.get(n).is_some())
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = (&str, PartOrigin)> {
        self.0.iter().map(|(k, o)| (k.as_str(), *o))
    }
}

/// The meshes of parts of the geometry that `geometry` no longer has, e.g. after a split
/// that cut a part into new ones or a deleted part: deleting a part of the geometry deletes
/// its mesh. Unknown while the names of the geometry's parts are not known
/// ([`crate::Geometry::known_parts`]).
pub fn orphans(geometry: Option<&crate::Geometry>, origins: &PartOrigins) -> Vec<String> {
    let known = |name: &str| match geometry {
        None => false,
        Some(g) => {
            g.known_parts().is_empty()
                || g.known_parts().iter().any(|p| p.eq_ignore_ascii_case(name))
        }
    };
    (origins.iter())
        .filter(|(name, origin)| !matches!(origin, PartOrigin::Imported { .. }) && !known(name))
        .map(|(name, _)| name.to_owned())
        .collect()
}

/// Deletes the meshes of parts that `geometry` no longer has ([`orphans`]) with the regions
/// of the parts of the FE model that go whole; returns their names.
pub fn drop_orphans(
    mesh: &mut FeMesh,
    fe: &mut crate::FeModel,
    geometry: Option<&crate::Geometry>,
    origins: &mut PartOrigins,
) -> Vec<String> {
    let orphans = orphans(geometry, origins);
    if orphans.is_empty() {
        return orphans;
    }
    let gone: Vec<String> = (mesh.parts.iter())
        .filter(|p| {
            bodies(p)
                .iter()
                .all(|b| orphans.iter().any(|o| o.eq_ignore_ascii_case(b)))
        })
        .map(|p| p.name.clone())
        .collect();
    mesh.remove_bodies(&orphans);
    fe.remove_parts_from_regions(&gone);
    origins.reconcile(mesh);
    orphans
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
        if geometry.is_suppressed(part) || origins.get(part).is_some() {
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

/// The names of the geometry objects of `mesh` that are suppressed: parts of the geometry
/// suppressed in `geometry`, imported meshes suppressed in `origins`.
pub fn suppressed_bodies(
    mesh: &FeMesh,
    geometry: Option<&crate::Geometry>,
    origins: &PartOrigins,
) -> Vec<String> {
    (mesh.parts.iter().flat_map(bodies))
        .filter(|body| {
            origins.is_suppressed(body) || geometry.is_some_and(|g| g.is_suppressed(body))
        })
        .map(str::to_owned)
        .collect()
}

/// What is solved of an analysis: its mesh without the suppressed geometry objects, and its
/// FE model without them in its regions of parts. Borrowed while nothing is suppressed.
pub struct Active<'a> {
    pub mesh: Cow<'a, FeMesh>,
    pub fe: Cow<'a, crate::FeModel>,
    /// The parts of the mesh left out, whole; bodies of compound parts are left out of
    /// their part.
    pub left_out: Vec<String>,
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
    let left_out: Vec<String> = (mesh.parts.iter())
        .filter(|p| {
            bodies(p)
                .iter()
                .all(|b| suppressed.iter().any(|s| s.eq_ignore_ascii_case(b)))
        })
        .map(|p| p.name.clone())
        .collect();
    let mut smaller = mesh.clone();
    smaller.remove_bodies(&suppressed);
    let mut fe = fe.clone();
    fe.remove_parts_from_regions(&left_out);
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
            mesh.parts.push(Part::new(*name, vec![id]));
        }
        mesh
    }

    #[test]
    fn a_part_without_an_origin_is_an_imported_mesh() {
        let mut origins = PartOrigins::default();
        origins.set_meshed("SOLID-1", 7);
        let added = origins.reconcile(&mesh(&["SOLID-1", "PART-1"]));
        assert_eq!(added, ["PART-1"]);
        assert_eq!(origins.made_from("SOLID-1"), Some(7));
        assert!(origins.is_imported("PART-1"));
        assert!(origins.is_cad("solid-1"));
    }

    #[test]
    fn the_origin_of_a_part_that_went_goes_with_it() {
        let mut origins = PartOrigins::default();
        origins.set_meshed("SOLID-1", 7);
        origins.set_meshed("SOLID-2", 8);
        origins.reconcile(&mesh(&["SOLID-2"]));
        assert_eq!(origins.get("SOLID-1"), None);
        assert!(origins.complete_for(&mesh(&["SOLID-2"])));
    }

    #[test]
    fn the_bodies_of_a_compound_part_each_have_their_origin() {
        let mut compound = mesh(&["A", "B"]);
        let bodies = (compound.parts.drain(..))
            .map(|p| plx_mesh::Body {
                name: p.name,
                elements: p.elements,
            })
            .collect::<Vec<_>>();
        let mut part = Part::new("C", vec![1, 2]);
        part.bodies = bodies;
        compound.parts.push(part);
        let mut origins = PartOrigins::default();
        origins.set_meshed("A", 1);
        origins.reconcile(&compound);
        assert!(origins.is_cad("A"));
        assert!(origins.is_imported("B"));
        assert_eq!(origins.get("C"), None);
    }

    #[test]
    fn a_suppressed_body_is_left_out_of_its_compound_part() {
        let mut compound = mesh(&["A", "B"]);
        let bodies = (compound.parts.drain(..))
            .map(|p| plx_mesh::Body {
                name: p.name,
                elements: p.elements,
            })
            .collect::<Vec<_>>();
        let mut part = Part::new("C", vec![1, 2]);
        part.bodies = bodies;
        compound.parts.push(part);
        let mut origins = PartOrigins::default();
        origins.set_meshed("A", 1);
        origins.set_meshed("B", 2);
        let mut geometry = crate::Geometry {
            source: String::new(),
            brep: String::new(),
            meshing: crate::MeshingParameters::default(),
            mesh_items: Vec::new(),
            part_names: vec!["A".into(), "B".into()],
            refinement: 1.0,
            signatures: Vec::new(),
            compounds: Vec::new(),
            suppressed: vec!["A".into()],
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
        geometry.suppressed.push("B".into());
        let solved = active_model(&compound, &fe, Some(&geometry), &origins);
        assert_eq!(solved.left_out, ["C"]);
        assert!(solved.mesh.parts.is_empty());
    }

    #[test]
    fn only_imported_meshes_are_suppressed_here() {
        let mut origins = PartOrigins::default();
        origins.set_meshed("SOLID-1", 7);
        origins.reconcile(&mesh(&["SOLID-1", "PART-1"]));
        let names = ["SOLID-1".to_string(), "PART-1".to_string()];
        assert!(origins.set_suppressed(&names, true));
        assert!(origins.is_suppressed("PART-1"));
        assert!(!origins.is_suppressed("SOLID-1"));
        assert!(!origins.set_suppressed(&names, true));
    }

    #[test]
    fn suppressed_objects_are_left_out_of_what_is_solved() {
        let mesh = mesh(&["SOLID-1", "PART-1"]);
        let mut origins = PartOrigins::default();
        origins.set_meshed("SOLID-1", 7);
        origins.reconcile(&mesh);
        let mut fe = crate::FeModel::default();
        fe.mesh_sets.push(crate::MeshSet::new(
            "BOTH",
            crate::SetKind::Elements,
            crate::Region::Parts(vec!["SOLID-1".into(), "PART-1".into()]),
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
        geometry.suppressed.push("SOLID-1".into());
        origins.set_suppressed(&["PART-1".into()], true);
        let solved = active_model(&mesh, &fe, Some(&geometry), &origins);
        assert_eq!(solved.left_out, ["SOLID-1", "PART-1"]);
        assert_eq!(solved.mesh.element_count(), 0);
        assert_eq!(
            solved.fe.mesh_sets[0].region,
            crate::Region::Parts(Vec::new())
        );
        origins.set_suppressed(&["PART-1".into()], false);
        let solved = active_model(&mesh, &fe, Some(&geometry), &origins);
        assert_eq!(solved.left_out, ["SOLID-1"]);
        assert_eq!(solved.mesh.parts[0].name, "PART-1");
        assert_eq!(solved.mesh.node_count(), 4);
    }

    #[test]
    fn the_mesh_of_a_part_the_geometry_no_longer_has_goes() {
        let mut mesh = mesh(&["SOLID-1", "SOLID-2", "PART-1"]);
        let mut origins = PartOrigins::default();
        origins.set_meshed("SOLID-1", 7);
        origins.set_meshed("SOLID-2", 8);
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
            ids: Vec::new(),
            from_link: false,
            operations: Vec::new(),
            input: None,
        };
        let mut fe = crate::FeModel::default();
        let gone = drop_orphans(&mut mesh, &mut fe, Some(&geometry), &mut origins);
        assert_eq!(gone, ["SOLID-1"]);
        let names: Vec<&str> = mesh.parts.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(names, ["SOLID-2", "PART-1"]);
        assert!(origins.complete_for(&mesh));
        // Without geometry, only the imported meshes stay.
        drop_orphans(&mut mesh, &mut fe, None, &mut origins);
        assert_eq!(mesh.parts.len(), 1);
        assert!(origins.is_imported("PART-1"));
    }

    #[test]
    fn a_mesh_link_hands_on_meshed_parts_as_linked() {
        let mut origins = PartOrigins::default();
        origins.set_meshed("SOLID-1", 7);
        origins.reconcile(&mesh(&["SOLID-1", "PART-1"]));
        let mut before = PartOrigins::imported(&mesh(&["PART-1"]));
        before.set_suppressed(&["PART-1".into()], true);
        let linked = origins.linked(&before);
        assert_eq!(linked.get("SOLID-1"), Some(PartOrigin::Linked));
        assert!(linked.is_imported("PART-1"));
        assert!(linked.is_linked("PART-1") && linked.is_linked("SOLID-1"));
        // The target keeps its own suppression of a linked object.
        assert!(linked.is_suppressed("PART-1"));
        assert_eq!(linked.made_from("SOLID-1"), None);
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
            own_meshes: vec!["OWN".into()],
            ids: Vec::new(),
            from_link: true,
            operations: Vec::new(),
            input: None,
        };
        let mut origins = PartOrigins::default();
        origins.insert("OWN", PartOrigin::Linked);
        origins.insert("OTHER", PartOrigin::Meshed(3));
        origins.settle(Some(&geometry));
        // The link's mesh of a part meshed here is out of date, the own mesh of a part the
        // link meshes is the link's.
        assert_eq!(origins.get("OWN"), Some(PartOrigin::Meshed(0)));
        assert_eq!(origins.get("OTHER"), Some(PartOrigin::Linked));
        // Shapes of the analysis' own leave the origins alone.
        geometry.from_link = false;
        origins.insert("OTHER", PartOrigin::Meshed(3));
        origins.settle(Some(&geometry));
        assert_eq!(origins.get("OTHER"), Some(PartOrigin::Meshed(3)));
    }

    /// Every part of the geometry that is not suppressed is a part of the FE model: those
    /// without a mesh are listed by their FE name, a compound part once.
    #[test]
    fn unmeshed_parts_are_the_unsuppressed_parts_without_a_mesh() {
        let mut origins = PartOrigins::default();
        origins.set_meshed("A", 1);
        let mesh = mesh(&["A"]);
        let geometry = crate::Geometry {
            source: String::new(),
            brep: String::new(),
            meshing: crate::MeshingParameters::default(),
            mesh_items: Vec::new(),
            part_names: ["A", "B", "C", "D", "E"].map(String::from).to_vec(),
            refinement: 1.0,
            signatures: Vec::new(),
            compounds: vec![crate::CompoundPart {
                name: "JOINED".into(),
                parts: vec!["D".into(), "E".into()],
            }],
            suppressed: vec!["C".into()],
            own_meshes: Vec::new(),
            ids: Vec::new(),
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
