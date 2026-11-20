//! Compound parts: the pieces a CAD model is cut into for the mesher, joined into one part
//! of the FE model ([`crate::CompoundPart`]). Each piece is meshed on its own and becomes a
//! body of the part ([`plx_mesh::Body`]), with nodes of its own. Where the bodies touch,
//! the solver input ties them as a bonded contact region would: [`FeModel::compound_ties`]
//! finds the touching faces with the search of the contact detection, and
//! [`FeModel::lower_contacts`] writes them as ties together with the user's regions. The
//! user sees one part and no ties.

use std::cell::OnceCell;
use std::sync::Mutex;

use std::collections::{BTreeSet, HashSet};

use plx_mesh::{
    ElementId, FeMesh, Fnv, GeometryFilter, GroupBy, NodeId, Part, PartSkin, SearchParameters,
    SurfaceFaces, extract_part_skin, find_contact_pairs,
};

use crate::contact_check::FaceSearch;
use crate::{ContactBehavior, ContactRegion, FeModel, Region, SideRoles};

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
        // The pairs of touching bodies: the compound part, the name and the tolerance of the
        // tie, and of both sides the faces lying on the other side and all faces of their
        // surfaces.
        let mut found = Vec::new();
        let faces = SurfaceFaces::new(&bodies, &skins);
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
                let (a, b) = faces.facing(&bodies, &item.master, &item.slave, tolerance, ANGLE_DEG);
                let (whole_a, whole_b) = (faces.of(&item.master), faces.of(&item.slave));
                // Where the bodies share their nodes (shells of a compound meshed together
                // meet at a common edge) they are joined already; a tie there would tie the
                // nodes to themselves. Only the faces across a gap or a mesh that does not
                // match are tied.
                // A side is the surfaces of one body, or of several the search joined.
                let body = |surfaces: &BTreeSet<plx_mesh::SurfaceId>| -> BTreeSet<usize> {
                    surfaces.iter().map(|s| s.0).collect()
                };
                let sides = [
                    Side {
                        bodies: body(&item.master),
                        facing: apart(mesh, &a, &whole_b),
                        whole: apart(mesh, &whole_a, &whole_b),
                    },
                    Side {
                        bodies: body(&item.slave),
                        facing: apart(mesh, &b, &whole_a),
                        whole: apart(mesh, &whole_b, &whole_a),
                    },
                ];
                if sides.iter().all(|s| s.facing.is_empty()) {
                    continue;
                }
                let name = format!("{}_{}", mesh.parts[part].name, item.name());
                found.push((part, name, tolerance, sides));
            }
        }
        // The master of each pair, as that of a bonded region between the two: the coarser
        // mesh, then the stiffer material. A side of which no face lies on the other (a
        // large face a small body sits on a corner of) counts by all its faces.
        let regions: Vec<ContactRegion> = (found.iter())
            .map(|(_, name, _, sides)| {
                let side = |s: &Side| {
                    Region::Faces(if s.facing.is_empty() {
                        s.whole.clone()
                    } else {
                        s.facing.clone()
                    })
                };
                let mut region = ContactRegion::new(name.clone(), ContactBehavior::Bonded);
                region.side_a = side(&sides[0]);
                region.side_b = side(&sides[1]);
                region
            })
            .collect();
        let masters = self.masters_of(&regions, mesh);
        // The skin of each body, for the bodies lying between two others; found when a
        // node is tied across a gap, which the bodies of most compound parts have none of.
        let skin_of: Vec<OnceCell<FaceSearch>> = owner.iter().map(|_| OnceCell::new()).collect();
        let mut ties = Vec::new();
        for ((part, name, tolerance, sides), a_is_master) in found.into_iter().zip(masters) {
            let [a, b] = sides;
            let (master, slave) = if a_is_master { (a, b) } else { (b, a) };
            let others: Vec<usize> = (0..owner.len())
                .filter(|&i| owner[i] == part)
                .filter(|i| !master.bodies.contains(i) && !slave.bodies.contains(i))
                .collect();
            let between = |p: [f64; 3], q: [f64; 3]| {
                others.iter().any(|&i| {
                    let skin = skin_of[i].get_or_init(|| FaceSearch::new(mesh, &faces.of_part(i)));
                    lies_between(skin, p, q)
                })
            };
            let Some((master, slave)) = tied(mesh, master, slave.facing, tolerance, between) else {
                continue;
            };
            let mut region = ContactRegion::new(name, ContactBehavior::Bonded);
            region.side_a = Region::Faces(master);
            region.side_b = slave;
            region.settings.sides = Some(SideRoles::AIsMaster);
            // The nodes tied are chosen here, see [`tied`]: the solver, which measures to
            // the curved faces of quadratic elements, shall not leave any of them out.
            region.settings.tolerance = Some(TOLERANCE_MARGIN * tolerance);
            // The bodies stay where they are: a gap between them, such as the play of a
            // pin in its hole, is bridged by the tie. Moving the slave nodes across a gap
            // as wide as their elements would turn those inside out.
            region.settings.interface = Some(crate::InterfaceTreatment::AsIs);
            ties.push(CompoundTie { part, region });
        }
        ties
    }
}

/// One side of a pair of touching bodies.
struct Side {
    /// Its bodies, by their index among the bodies searched.
    bodies: BTreeSet<usize>,
    /// The faces lying on the other side, see [`plx_mesh::facing_faces`].
    facing: Vec<(ElementId, u8)>,
    /// All faces of the surfaces that touch the other side.
    whole: Vec<(ElementId, u8)>,
}

/// How much further than the tolerance the nodes were chosen within the solver ties: the
/// nodes are chosen by their distance to the master faces flat between their corners, and
/// a quadratic face bulges away from that by a part of its size.
const TOLERANCE_MARGIN: f64 = 1.5;

/// How far beside a master face, relative to its size, a slave node still lies on it: the
/// border of a round common face is a polygon of other corners in each of the two meshes,
/// and the nodes of one lie a little outside the polygon of the other. At most
/// [`BESIDE_TOLERANCE`] of the tolerance, as a master face may be much larger than the
/// elements of the slave.
const BESIDE: f64 = 0.15;
const BESIDE_TOLERANCE: f64 = 0.5;

/// How far beside a master face, relative to its distance, a slave node across a gap
/// still lies above it: around an edge of the master, such as between the faces of a pin
/// in its hole, the closest point is on the edge, and the way to it leans by up to the
/// angle between the faces.
const LEANING: f64 = 0.6;

/// The gap, relative to the tolerance, from which a slave node counts as lying across a
/// gap another body may lie in.
const TOUCHING: f64 = 1e-6;

/// The master faces and the slave side of the tie between two bodies, `None` where
/// nothing is tied.
///
/// A slave face lies on the master when its centre does; its nodes may not. Where the
/// bodies do not cover each other fully, the faces along the border of the common area
/// reach beyond the master, and their outer nodes have no master within the tolerance:
/// the solver would leave them out and warn about each tie. So the slave side is the
/// nodes lying on the master, as picked nodes where some of the faces' nodes do not.
///
/// A node lies on the master within the tolerance above or below one of its faces, not
/// beside it: a node beside the border of the master, tied to the closest point of the
/// border, would be held at a distance along the surfaces, which widens the joint by the
/// tolerance all around and holds the slave against turning about the border.
///
/// A node across a gap another body of the compound lies in is not tied either
/// (`between`, of the node and its closest point of the master): two bodies with a thin
/// sheet between them are tied to the sheet, not to each other through it.
///
/// A master face lies on the slave when its centre does, too, which a face larger than
/// the area the slave touches it in does not: a small body on the corner of a large
/// face. The slave nodes above such a face would find no master. So the master side is
/// also every face of the master's surfaces that is the nearest to a slave node.
fn tied(
    mesh: &FeMesh,
    master: Side,
    slave: Vec<(ElementId, u8)>,
    tolerance: f64,
    between: impl Fn([f64; 3], [f64; 3]) -> bool,
) -> Option<(Vec<(ElementId, u8)>, Region)> {
    let mut candidates = master.facing.clone();
    let facing: HashSet<(ElementId, u8)> = master.facing.iter().copied().collect();
    candidates.extend(master.whole.into_iter().filter(|f| !facing.contains(f)));
    let search = FaceSearch::new(mesh, &candidates);
    let slave = Region::Faces(slave);
    let nodes = slave.nodes(mesh);
    let mut faces: BTreeSet<(ElementId, u8)> = BTreeSet::new();
    let mut near = Vec::new();
    for &node in &nodes {
        let Some(p) = mesh.node(node) else {
            continue;
        };
        let Some(nearest) = search.nearest(p, tolerance) else {
            continue;
        };
        let beside = (BESIDE * nearest.size).min(BESIDE_TOLERANCE * tolerance);
        if nearest.lateral > beside.max(LEANING * nearest.distance) {
            continue;
        }
        if nearest.distance > TOUCHING * tolerance && between(p, nearest.point) {
            continue;
        }
        faces.insert(candidates[nearest.face]);
        near.push(node);
    }
    if near.is_empty() {
        return None;
    }
    // The faces lying on the slave stay masters: the solver ties each node to the closest
    // point of the curved faces, which may be on a neighbour of the nearest flat one.
    faces.extend(facing);
    let slave = if near.len() == nodes.len() {
        slave
    } else {
        Region::Nodes(near)
    };
    Some((faces.into_iter().collect(), slave))
}

/// Whether the body of the skin lies between `p` and `q`: the point half way is in its
/// material or on its skin. A body only touching one of the two points from the side is
/// not between.
fn lies_between(skin: &FaceSearch, p: [f64; 3], q: [f64; 3]) -> bool {
    let middle = [0, 1, 2].map(|k| 0.5 * (p[k] + q[k]));
    let gap = (0..3).map(|k| (q[k] - p[k]).powi(2)).sum::<f64>().sqrt();
    (skin.nearest(middle, gap))
        .is_some_and(|nearest| nearest.inside || nearest.distance <= 1e-6 * gap)
}

/// The faces of `faces` not lying on nodes of the elements of `other`: those whose corners
/// all are nodes the two sides share go.
fn apart(
    mesh: &FeMesh,
    faces: &[(ElementId, u8)],
    other: &[(ElementId, u8)],
) -> Vec<(ElementId, u8)> {
    let across: HashSet<NodeId> = (other.iter())
        .filter_map(|&(e, _)| mesh.element(e))
        .flat_map(|e| e.nodes.iter().copied())
        .collect();
    if across.is_empty() {
        return faces.to_vec();
    }
    (faces.iter().copied())
        .filter(|&(e, face)| {
            let Some(element) = mesh.element(e) else {
                return true;
            };
            let Some(topology) = element.faces().get(usize::from(face).saturating_sub(1)) else {
                return true;
            };
            !(topology.corners.iter()).all(|&k| across.contains(&element.nodes[k]))
        })
        .collect()
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
                // Shared nodes join bodies, see [`apart`].
                for node in &element.nodes {
                    hash.add(&node.to_le_bytes());
                }
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
                id: Default::default(),
                name: "SOLID-1".into(),
                elements: vec![1],
            },
            Body {
                id: Default::default(),
                name: "SOLID-2".into(),
                elements: vec![2],
            },
        ];
        mesh.parts.push(part);
        mesh
    }

    /// A block of C3D8 between the planes `xs`, `ys` and `zs`, its nodes and elements
    /// numbered on from those of the mesh; returns its elements.
    fn block(mesh: &mut FeMesh, xs: &[f64], ys: &[f64], zs: &[f64]) -> Vec<ElementId> {
        let first = mesh.node_ids().iter().copied().max().unwrap_or(0) + 1;
        let (nx, ny) = (xs.len() as u32, ys.len() as u32);
        let node = |i: u32, j: u32, k: u32| first + i + nx * (j + ny * k);
        for (k, &z) in zs.iter().enumerate() {
            for (j, &y) in ys.iter().enumerate() {
                for (i, &x) in xs.iter().enumerate() {
                    mesh.set_node(node(i as u32, j as u32, k as u32), [x, y, z]);
                }
            }
        }
        let mut id = mesh.elements().iter().map(|e| e.id).max().unwrap_or(0);
        let mut elements = Vec::new();
        for k in 0..zs.len() as u32 - 1 {
            for j in 0..ny - 1 {
                for i in 0..nx - 1 {
                    id += 1;
                    mesh.add_element(plx_mesh::Element {
                        id,
                        type_name: "C3D8".into(),
                        shape: ElementShape::Hex8,
                        nodes: vec![
                            node(i, j, k),
                            node(i + 1, j, k),
                            node(i + 1, j + 1, k),
                            node(i, j + 1, k),
                            node(i, j, k + 1),
                            node(i + 1, j, k + 1),
                            node(i + 1, j + 1, k + 1),
                            node(i, j + 1, k + 1),
                        ],
                    })
                    .unwrap();
                    elements.push(id);
                }
            }
        }
        elements
    }

    /// A compound part of the bodies, each the elements of a [`block`].
    fn compound(mesh: &mut FeMesh, bodies: Vec<Vec<ElementId>>) {
        let mut part = Part::new("COMPOUND-1", bodies.concat());
        part.bodies = (bodies.into_iter().enumerate())
            .map(|(i, elements)| Body {
                id: Default::default(),
                name: format!("SOLID-{}", i + 1),
                elements,
            })
            .collect();
        mesh.parts.push(part);
    }

    fn steps(from: f64, step: f64, count: usize) -> Vec<f64> {
        (0..=count).map(|i| from + step * i as f64).collect()
    }

    /// A plate on a block of one element, reaching beyond it along x: the row of wide
    /// elements of the plate above the border of the block has its centres on the block
    /// and its outer nodes far beyond; the rest of the plate is meshed finely, so that the
    /// tolerance is small.
    fn plate_beyond_block() -> FeMesh {
        let mut mesh = FeMesh::default();
        let lower = block(&mut mesh, &[0.0, 4.0], &[0.0, 4.0], &[-4.0, 0.0]);
        let mut xs = vec![1.0, 3.0, 4.9];
        xs.extend(steps(5.0, 0.1, 20));
        let upper = block(&mut mesh, &xs, &steps(0.0, 1.0, 4), &[0.0, 0.5]);
        compound(&mut mesh, vec![lower, upper]);
        mesh
    }

    /// The distance of each slave node of the tie to its master faces.
    fn gaps(mesh: &FeMesh, tie: &crate::Tie) -> Vec<f64> {
        let points: Vec<[f64; 3]> = (tie.slave.nodes(mesh).iter())
            .map(|&n| mesh.node(n).unwrap())
            .collect();
        crate::contact_check::distances(mesh, &tie.master.faces(mesh), &points)
    }

    #[test]
    fn slave_nodes_beyond_the_master_are_left_out_of_the_tie() {
        let mesh = plate_beyond_block();
        let model = FeModel::default();
        let (lowered, _) = model.lower_contacts(&mesh);
        assert_eq!(lowered.ties.len(), 1);
        let tie = &lowered.ties[0];
        // The block of one element is the master, its top face.
        assert_eq!(tie.master.faces(&mesh).len(), 1);
        // Of the plate the nodes at x = 1 and 3, not those at x = 4.9 beyond the block.
        let Region::Nodes(nodes) = &tie.slave else {
            panic!("{:?}", tie.slave);
        };
        assert_eq!(nodes.len(), 10);
        for &n in nodes {
            let [x, _, z] = mesh.node(n).unwrap();
            assert!(x < 3.5 && z == 0.0, "{n}");
        }
        let tolerance = tie.position_tolerance.unwrap();
        assert!(gaps(&mesh, tie).iter().all(|&d| d <= tolerance));
    }

    #[test]
    fn nodes_beside_the_master_are_not_tied() {
        // A finely meshed plate half on a block of one element: its nodes a quarter beside
        // the block are within the tolerance of the border of the block, but not on it.
        let mut mesh = FeMesh::default();
        let lower = block(&mut mesh, &[0.0, 4.0], &[0.0, 4.0], &[-4.0, 0.0]);
        let xs = steps(2.0, 0.25, 16);
        let upper = block(&mut mesh, &xs, &steps(0.0, 1.0, 4), &[0.0, 0.5]);
        compound(&mut mesh, vec![lower, upper]);
        let model = FeModel::default();
        let (lowered, _) = model.lower_contacts(&mesh);
        assert_eq!(lowered.ties.len(), 1);
        let tie = &lowered.ties[0];
        assert!(tie.position_tolerance.unwrap() > 0.25 * TOLERANCE_MARGIN);
        let nodes = tie.slave.nodes(&mesh);
        // The 9 x 5 nodes from x = 2 to 4.
        assert_eq!(nodes.len(), 45);
        assert!(nodes.iter().all(|&n| mesh.node(n).unwrap()[0] <= 4.0));
    }

    #[test]
    fn bodies_are_not_tied_through_a_body_between_them() {
        // A thin sheet between two blocks, thinner than the tolerance: the sheet is tied
        // to both, the blocks are not tied to each other across it.
        let mut mesh = FeMesh::default();
        let unit = steps(0.0, 1.0, 4);
        let left = block(&mut mesh, &[0.0, 4.0], &[0.0, 4.0], &[0.0, 4.0]);
        let sheet = block(&mut mesh, &[4.0, 4.2], &unit, &unit);
        let right = block(
            &mut mesh,
            &[4.2, 6.2, 8.2],
            &[0.0, 2.0, 4.0],
            &[0.0, 2.0, 4.0],
        );
        let of_sheet: HashSet<ElementId> = sheet.iter().copied().collect();
        compound(&mut mesh, vec![left, sheet, right]);
        let model = FeModel::default();
        let ties = model.compound_ties(&mesh);
        assert_eq!(ties.len(), 2, "{ties:?}");
        for tie in &ties {
            assert!(tie.region.settings.tolerance.unwrap() > 0.2);
            let sides = [&tie.region.side_a, &tie.region.side_b];
            let on_sheet = (sides.iter())
                .filter(|side| (side.faces(&mesh).iter()).all(|(e, _)| of_sheet.contains(e)))
                .count();
            assert_eq!(on_sheet, 1, "{tie:?}");
        }
    }

    #[test]
    fn a_small_body_on_the_corner_of_a_large_face_is_tied_to_it() {
        // The centre of the large face is not below the small body.
        let mut mesh = FeMesh::default();
        let lower = block(&mut mesh, &[0.0, 4.0], &[0.0, 4.0], &[-4.0, 0.0]);
        let fine = steps(0.0, 0.25, 4);
        let upper = block(&mut mesh, &fine, &fine, &[0.0, 0.25]);
        compound(&mut mesh, vec![lower, upper]);
        let model = FeModel::default();
        let (lowered, _) = model.lower_contacts(&mesh);
        assert_eq!(lowered.ties.len(), 1);
        let tie = &lowered.ties[0];
        assert_eq!(tie.master.faces(&mesh), [(1, 2)]);
        // All 25 nodes of the bottom of the small body.
        assert_eq!(tie.slave.nodes(&mesh).len(), 25);
        assert!(gaps(&mesh, tie).iter().all(|&d| d == 0.0));
    }

    #[test]
    fn large_faces_under_the_border_of_a_small_body_are_masters_too() {
        // A block of 2 x 2 large faces; the small body covers the first and reaches a
        // little on to the three others, whose centres it does not cover.
        let mut mesh = FeMesh::default();
        let coarse = [0.0, 2.0, 4.0];
        let lower = block(&mut mesh, &coarse, &coarse, &[-2.0, 0.0]);
        let fine = steps(0.0, 0.25, 10);
        let upper = block(&mut mesh, &fine, &fine, &[0.0, 0.25]);
        compound(&mut mesh, vec![lower, upper]);
        let model = FeModel::default();
        let (lowered, _) = model.lower_contacts(&mesh);
        assert_eq!(lowered.ties.len(), 1);
        let tie = &lowered.ties[0];
        assert_eq!(tie.master.faces(&mesh).len(), 4);
        assert!(matches!(tie.slave, Region::Faces(_)));
        assert_eq!(tie.slave.nodes(&mesh).len(), 121);
        assert!(gaps(&mesh, tie).iter().all(|&d| d == 0.0));
    }

    #[test]
    fn ties_sharing_slave_nodes_keep_the_nodes_picked_of_both() {
        // The plate reaching beyond its block, and a second block beside the first below
        // the far end of the plate: both ties have nodes of the plate, and become one.
        let mut mesh = FeMesh::default();
        let lower = block(&mut mesh, &[0.0, 4.0], &[0.0, 4.0], &[-4.0, 0.0]);
        let mut xs = vec![1.0, 3.0, 4.9];
        xs.extend(steps(5.0, 0.1, 20));
        let upper = block(&mut mesh, &xs, &steps(0.0, 1.0, 4), &[0.0, 0.5]);
        let beside = block(&mut mesh, &[4.9, 7.0], &[0.0, 4.0], &[-4.0, 0.0]);
        compound(&mut mesh, vec![lower, upper, beside]);
        let model = FeModel::default();
        let (lowered, _) = model.lower_contacts(&mesh);
        let plate: Vec<&crate::Tie> = (lowered.ties.iter())
            .filter(|t| t.slave.nodes(&mesh).iter().any(|&n| n > 8 && n <= 8 + 230))
            .collect();
        assert_eq!(plate.len(), 1, "{:?}", lowered.ties);
        let tolerance = plate[0].position_tolerance.unwrap();
        let gaps = gaps(&mesh, plate[0]);
        // The whole bottom of the plate: 24 x 5 nodes.
        assert_eq!(gaps.len(), 120);
        assert!(gaps.iter().all(|&d| d <= tolerance));
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
