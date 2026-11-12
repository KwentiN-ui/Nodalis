//! Compound parts: the pieces a CAD model is cut into for the mesher, joined into one part
//! of the FE model ([`crate::CompoundPart`]). Each piece is meshed on its own and becomes a
//! body of the part ([`plx_mesh::Body`]), with nodes of its own. Where the bodies touch,
//! the solver input ties them as a bonded contact region would: [`FeModel::compound_ties`]
//! finds the touching faces with the search of the contact detection, and
//! [`FeModel::lower_contacts`] writes them as ties together with the user's regions. The
//! user sees one part and no ties.

use std::sync::Mutex;

use plx_mesh::{
    FeMesh, Fnv, GeometryFilter, GroupBy, Part, PartSkin, SearchParameters, extract_part_skin,
    facing_faces, find_contact_pairs,
};

use crate::{ContactBehavior, ContactRegion, FeModel, Region};

/// Feature angle of the skins searched, as the model's display uses.
const FEATURE_ANGLE_DEG: f64 = 30.0;

/// Largest angle between faces of two bodies facing each other, as a contact detection's
/// default.
const ANGLE_DEG: f64 = 35.0;

/// A tie between two bodies of a compound part, as a bonded contact region.
#[derive(Clone, Debug, PartialEq)]
pub struct CompoundTie {
    /// The compound part, by index in the mesh.
    pub part: usize,
    pub region: ContactRegion,
}

/// Whether the mesh has a compound part of more than one body, whose bodies are tied.
pub fn has_compound_ties(mesh: &FeMesh) -> bool {
    mesh.parts.iter().any(|p| p.bodies.len() > 1)
}

impl FeModel {
    /// The ties between the bodies of the compound parts: of each pair of touching bodies
    /// a bonded contact region, its master the side the contact detection takes, with the
    /// tolerance a detection on the mesh would have.
    ///
    /// The search runs once for a mesh: the model checks and the input file ask again with
    /// every change of the FE model, and on a large mesh it takes a moment.
    pub fn compound_ties(&self, mesh: &FeMesh) -> Vec<CompoundTie> {
        if !has_compound_ties(mesh) {
            return Vec::new();
        }
        // Materials by element: the sections name the compound part, not its bodies.
        let young_of = self.young_of_elements(mesh);
        let stiffness: Vec<Option<f64>> = (mesh.parts.iter())
            .filter(|p| p.bodies.len() > 1)
            .flat_map(|p| &p.bodies)
            .map(|body| {
                (body.elements.iter())
                    .filter_map(|e| young_of.get(e).copied())
                    .reduce(f64::max)
            })
            .collect();
        let key = key(mesh, &stiffness, self.properties.space.is_2d());
        static FOUND: Mutex<Option<(u64, Vec<CompoundTie>)>> = Mutex::new(None);
        if let Some((seen, ties)) = FOUND.lock().unwrap_or_else(|e| e.into_inner()).as_ref()
            && *seen == key
        {
            return ties.clone();
        }
        let ties = self.search_compound_ties(mesh, stiffness);
        *FOUND.lock().unwrap_or_else(|e| e.into_inner()) = Some((key, ties.clone()));
        ties
    }

    /// [`Self::compound_ties`] found anew; `stiffness` is Young's modulus of each body.
    fn search_compound_ties(&self, mesh: &FeMesh, stiffness: Vec<Option<f64>>) -> Vec<CompoundTie> {
        // The bodies as parts of their own, the parts the search compares.
        let mut bodies = mesh.clone();
        let mut owner = Vec::new();
        bodies.parts = Vec::new();
        for (index, part) in mesh.parts.iter().enumerate() {
            if part.bodies.len() > 1 {
                for body in &part.bodies {
                    bodies
                        .parts
                        .push(Part::new(body.name.clone(), body.elements.clone()));
                    owner.push(index);
                }
            }
        }
        let skins: Vec<PartSkin> = (bodies.parts.iter())
            .map(|part| extract_part_skin(&bodies, part, FEATURE_ANGLE_DEG))
            .collect();
        let filter = if self.properties.space.is_2d() {
            GeometryFilter {
                solid: false,
                shell: false,
                shell_edge: true,
            }
        } else {
            GeometryFilter {
                solid: true,
                shell: true,
                shell_edge: true,
            }
        };
        let mut ties = Vec::new();
        let mut compounds: Vec<usize> = owner.clone();
        compounds.dedup();
        for part in compounds {
            let searched: Vec<bool> = owner.iter().map(|&o| o == part).collect();
            let tolerance = crate::auto_detection_tolerance(&bodies, &skins, &searched);
            let parameters = SearchParameters {
                distance: tolerance,
                angle_deg: ANGLE_DEG,
                group_by: GroupBy::Parts,
                stiffness: stiffness.clone(),
                filter,
                one_sided: Vec::new(),
                solid_skins: Vec::new(),
            };
            for item in find_contact_pairs(&bodies, &skins, &searched, &parameters) {
                // Only the faces lying on the other body: the search pairs whole patches.
                let (master, slave) = facing_faces(
                    &bodies,
                    &skins,
                    &item.master,
                    &item.slave,
                    tolerance,
                    ANGLE_DEG,
                );
                if master.is_empty() || slave.is_empty() {
                    continue;
                }
                let name = format!("{}_{}", mesh.parts[part].name, item.name());
                let mut region = ContactRegion::new(name, ContactBehavior::Bonded);
                region.side_a = Region::Faces(master);
                region.side_b = Region::Faces(slave);
                region.settings.tolerance = Some(tolerance);
                // The bodies stay where they are: a gap between them, such as the play of a
                // pin in its hole, is bridged by the tie. Moving the slave nodes across a gap
                // as wide as their elements would turn those inside out.
                region.settings.interface = Some(crate::InterfaceTreatment::AsIs);
                ties.push(CompoundTie { part, region });
            }
        }
        ties
    }
}

/// What the ties of the compound parts depend on: their bodies, the nodes of their elements
/// and where they are, the stiffness of the bodies and the model space.
fn key(mesh: &FeMesh, stiffness: &[Option<f64>], plane: bool) -> u64 {
    let mut hash = Fnv::new();
    hash.add(&[u8::from(plane)]);
    for value in stiffness {
        hash.add(&value.unwrap_or(f64::NAN).to_le_bytes());
    }
    for (index, part) in mesh.parts.iter().enumerate() {
        if part.bodies.len() < 2 {
            continue;
        }
        hash.add(&index.to_le_bytes());
        hash.add_str(&part.name);
        for body in &part.bodies {
            hash.add_str(&body.name);
            for element in body.elements.iter().filter_map(|&e| mesh.element(e)) {
                hash.add(&element.id.to_le_bytes());
                hash.add_str(&element.type_name);
                for p in element.nodes.iter().filter_map(|&n| mesh.node(n)) {
                    for x in p {
                        hash.add(&x.to_le_bytes());
                    }
                }
            }
        }
    }
    hash.finish()
}

#[cfg(test)]
mod tests {
    use plx_mesh::{Body, ElementShape};

    use super::*;

    /// Two unit cubes of C3D8 side by side along x, each a body of one compound part, with
    /// nodes of their own on the face they share.
    fn two_cubes(gap: f64) -> FeMesh {
        let mut mesh = FeMesh::default();
        let corners = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ];
        for cube in 0..2u32 {
            let shift = cube as f64 * (1.0 + gap);
            for (k, c) in corners.iter().enumerate() {
                mesh.set_node(cube * 8 + k as u32 + 1, [c[0] + shift, c[1], c[2]]);
            }
            mesh.add_element(plx_mesh::Element {
                id: cube + 1,
                type_name: "C3D8".into(),
                shape: ElementShape::Hex8,
                nodes: (1..=8).map(|n| cube * 8 + n).collect(),
            })
            .unwrap();
        }
        let mut part = Part::new("COMPOUND-1", vec![1, 2]);
        part.bodies = vec![
            Body {
                name: "SOLID-1".into(),
                elements: vec![1],
            },
            Body {
                name: "SOLID-2".into(),
                elements: vec![2],
            },
        ];
        mesh.parts.push(part);
        mesh
    }

    #[test]
    fn touching_bodies_of_a_compound_are_tied_on_their_common_face() {
        let mesh = two_cubes(0.0);
        let model = FeModel::default();
        let ties = model.compound_ties(&mesh);
        assert_eq!(ties.len(), 1);
        let tie = &ties[0];
        assert_eq!(tie.part, 0);
        assert_eq!(tie.region.behavior, ContactBehavior::Bonded);
        let nodes = |r: &Region| {
            let mut nodes = r.nodes(&mesh);
            nodes.sort_unstable();
            nodes
        };
        // The face x = 1 of the first cube and x = 1 of the second.
        let (a, b) = (nodes(&tie.region.side_a), nodes(&tie.region.side_b));
        let mut both = [a, b];
        both.sort();
        assert_eq!(both, [vec![2, 3, 6, 7], vec![9, 12, 13, 16]]);
    }

    #[test]
    fn bodies_with_a_gap_are_tied_where_they_are() {
        // A gap of a fifth of the elements, within the tolerance of half their size.
        let mesh = two_cubes(0.2);
        let model = FeModel::default();
        let (lowered, _) = model.lower_contacts(&mesh);
        assert_eq!(lowered.ties.len(), 1);
        assert!(!lowered.ties[0].adjust);
    }

    #[test]
    fn bodies_apart_and_parts_of_one_body_are_not_tied() {
        let model = FeModel::default();
        assert!(model.compound_ties(&two_cubes(0.5)).is_empty());
        let mut mesh = two_cubes(0.0);
        mesh.parts[0].bodies.clear();
        assert!(!has_compound_ties(&mesh));
        assert!(model.compound_ties(&mesh).is_empty());
    }

    #[test]
    fn the_lowered_model_ties_the_bodies_and_the_ties_belong_to_the_part() {
        let mesh = two_cubes(0.0);
        let model = FeModel::default();
        let (lowered, origin) = model.lower_contacts(&mesh);
        assert_eq!(lowered.ties.len(), 1);
        assert!(lowered.contact_regions.is_empty());
        let tie = crate::ModelItem::Tie(0);
        assert_eq!(origin.region_of(tie), None);
        assert_eq!(origin.part_of(tie), Some(0));
    }
}
