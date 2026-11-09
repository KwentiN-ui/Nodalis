//! Turning the faces of a closed surface, such as the display of a CAD solid, outwards.

use std::collections::{BTreeSet, HashMap};

use crate::element::ElementFamily;
use crate::mesh::FeMesh;
use crate::skin::extract_part_skin;

/// Turns the triangles and quadrilaterals of part `part` so that their normals point out of
/// the solid they enclose, as the display of a CAD solid does not tell which side of its
/// faces is outside. Returns whether the outside was found for every patch of faces; when
/// not (an open surface, or rays cast from a patch disagree) the mesh is left as it was.
///
/// Faces sharing an edge are first turned alike; each patch connected that way then casts
/// rays from a few of its faces along their normals: a ray that crosses the surface an odd
/// number of times starts inside.
pub fn orient_outward(mesh: &mut FeMesh, part: usize) -> bool {
    let Some(entry) = mesh.parts.get(part) else {
        return false;
    };
    let skin = extract_part_skin(mesh, entry, 180.0);
    let elements = mesh.elements();
    let coords = mesh.coords();
    // The surface faces of the part, by their corners in element order.
    let faces: Vec<(usize, Vec<usize>)> = (skin.faces.iter())
        .filter(|f| elements[f.element].shape.family() == ElementFamily::Surface)
        .filter(|f| f.corners.len() >= 3)
        .map(|f| (f.element, f.corners.clone()))
        .collect();
    if faces.is_empty() {
        return false;
    }
    let triangles: Vec<[[f64; 3]; 3]> = (faces.iter())
        .flat_map(|(_, c)| (1..c.len() - 1).map(move |k| [c[0], c[k], c[k + 1]]))
        .map(|t| t.map(|n| coords[n]))
        .collect();
    // Faces on each edge, with the direction they run along it.
    let mut on_edge: HashMap<[usize; 2], Vec<(usize, bool)>> = HashMap::new();
    for (index, (_, corners)) in faces.iter().enumerate() {
        for k in 0..corners.len() {
            let (a, b) = (corners[k], corners[(k + 1) % corners.len()]);
            on_edge
                .entry([a.min(b), a.max(b)])
                .or_default()
                .push((index, a < b));
        }
    }
    // Patches of faces turned alike: `flip` relative to the first face of the patch.
    let mut patch = vec![usize::MAX; faces.len()];
    let mut flip = vec![false; faces.len()];
    let mut patches: Vec<Vec<usize>> = Vec::new();
    for start in 0..faces.len() {
        if patch[start] != usize::MAX {
            continue;
        }
        let id = patches.len();
        let mut members = vec![start];
        patch[start] = id;
        let mut next = 0;
        while next < members.len() {
            let face = members[next];
            next += 1;
            let corners = &faces[face].1;
            for k in 0..corners.len() {
                let (a, b) = (corners[k], corners[(k + 1) % corners.len()]);
                let Some(sharing) = on_edge.get(&[a.min(b), a.max(b)]) else {
                    continue;
                };
                // Only edges between two faces say how the faces lie to each other.
                if sharing.len() != 2 {
                    continue;
                }
                let forward = (a < b) != flip[face];
                for &(other, other_forward) in sharing {
                    if other == face || patch[other] != usize::MAX {
                        continue;
                    }
                    // Faces turned alike run along a shared edge in opposite directions.
                    flip[other] = other_forward == forward;
                    patch[other] = id;
                    members.push(other);
                }
            }
        }
        patches.push(members);
    }
    // Of each patch, whether it points inwards, from rays cast off its largest faces.
    let area_normal = |corners: &[usize]| {
        let mut n = [0.0; 3];
        for k in 1..corners.len() - 1 {
            let c = cross(
                sub(coords[corners[k]], coords[corners[0]]),
                sub(coords[corners[k + 1]], coords[corners[0]]),
            );
            n = add(n, c);
        }
        scale(n, 0.5)
    };
    let mut inwards = BTreeSet::new();
    for (id, members) in patches.iter().enumerate() {
        let mut by_area: Vec<(f64, usize)> = (members.iter())
            .map(|&f| (length(area_normal(&faces[f].1)), f))
            .filter(|&(area, _)| area > 0.0)
            .collect();
        by_area.sort_by(|a, b| b.0.total_cmp(&a.0));
        let (mut inside, mut outside) = (0usize, 0usize);
        for (ray, &(area, face)) in by_area.iter().take(5).enumerate() {
            let corners = &faces[face].1;
            let mut normal = scale(area_normal(corners), 1.0 / area);
            if flip[face] {
                normal = scale(normal, -1.0);
            }
            let center = scale(
                corners.iter().fold([0.0; 3], |s, &n| add(s, coords[n])),
                1.0 / corners.len() as f64,
            );
            let origin = add(center, scale(normal, 1e-6 * area.sqrt()));
            // Slightly off the normal, so that the ray does not run along an edge.
            let tilt = [0.0123, -0.0217, 0.0179].map(|v| v * (1.0 + ray as f64));
            let direction = normalize(add(normal, tilt));
            let crossings = (triangles.iter())
                .filter(|t| ray_hits(origin, direction, t))
                .count();
            if crossings % 2 == 1 {
                inside += 1;
            } else {
                outside += 1;
            }
        }
        // The majority of the rays decides; a tie leaves the outside unknown.
        if inside == outside {
            return false;
        }
        if inside > outside {
            inwards.insert(id);
        }
    }
    let turned: BTreeSet<_> = (0..faces.len())
        .filter(|&f| flip[f] != inwards.contains(&patch[f]))
        .map(|f| elements[faces[f].0].id)
        .collect();
    mesh.flip_surface_elements(|e| turned.contains(&e.id));
    true
}

/// Whether the ray from `origin` along `direction` crosses the triangle (Möller-Trumbore).
fn ray_hits(origin: [f64; 3], direction: [f64; 3], t: &[[f64; 3]; 3]) -> bool {
    let e1 = sub(t[1], t[0]);
    let e2 = sub(t[2], t[0]);
    let p = cross(direction, e2);
    let det = dot(e1, p);
    if det.abs() < 1e-300 {
        return false;
    }
    let s = sub(origin, t[0]);
    let u = dot(s, p) / det;
    if !(0.0..=1.0).contains(&u) {
        return false;
    }
    let q = cross(s, e1);
    let v = dot(direction, q) / det;
    if v < 0.0 || u + v > 1.0 {
        return false;
    }
    dot(e2, q) / det > 0.0
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn length(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn normalize(a: [f64; 3]) -> [f64; 3] {
    scale(a, 1.0 / length(a))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::element::ElementShape;
    use crate::mesh::{Element, Part};

    /// The six faces of a unit box at `x0` as quadrilaterals, every second one turned in.
    fn box_surface(mesh: &mut FeMesh, x0: f64, first: usize) -> Part {
        let corner = |i: usize| {
            [
                x0 + (i & 1) as f64,
                ((i >> 1) & 1) as f64,
                ((i >> 2) & 1) as f64,
            ]
        };
        for i in 0..8 {
            mesh.set_node((first + i) as u32, corner(i));
        }
        // Outward node order of each face.
        let quads = [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ];
        let mut elements = Vec::new();
        for (k, quad) in quads.iter().enumerate() {
            let mut nodes: Vec<u32> = quad.iter().map(|&i| (first + i) as u32).collect();
            if k % 2 == 1 {
                nodes.reverse();
            }
            let id = (first + k) as u32;
            mesh.add_element(Element {
                id,
                type_name: "S4".into(),
                shape: ElementShape::Quad4,
                nodes,
            })
            .unwrap();
            elements.push(id);
        }
        Part {
            name: format!("BOX{first}"),
            elements,
        }
    }

    fn outward(mesh: &FeMesh, part: usize, x0: f64) -> bool {
        let centre = [x0 + 0.5, 0.5, 0.5];
        mesh.parts[part].elements.iter().all(|&id| {
            let e = mesh.element(id).unwrap();
            let p: Vec<[f64; 3]> = e.nodes.iter().map(|&n| mesh.node(n).unwrap()).collect();
            let n = cross(sub(p[1], p[0]), sub(p[2], p[0]));
            let c = scale(p.iter().fold([0.0; 3], |s, &q| add(s, q)), 0.25);
            dot(n, sub(c, centre)) > 0.0
        })
    }

    #[test]
    fn the_faces_of_a_closed_box_end_up_pointing_out_of_it() {
        let mut mesh = FeMesh::default();
        let a = box_surface(&mut mesh, 0.0, 1);
        let b = box_surface(&mut mesh, 1.0, 101);
        mesh.parts = vec![a, b];
        assert!(orient_outward(&mut mesh, 0));
        assert!(orient_outward(&mut mesh, 1));
        assert!(outward(&mesh, 0, 0.0));
        assert!(outward(&mesh, 1, 1.0));
    }

    #[test]
    fn faces_of_neighbours_in_one_plane_touch_only_while_two_sided() {
        use crate::contact_search::{GeometryFilter, GroupBy, SearchParameters};
        use crate::{extract_part_skin, find_contact_pairs};
        // Two boxes whose faces at y = 0 (and y = 1, z = 0, z = 1) overlap in one plane
        // by 0.2 in x, as faces on a plane of symmetry do where the triangles of their
        // displays overlap; their faces at x = 0.8 and x = 1 are farther apart than the
        // search distance.
        let mut mesh = FeMesh::default();
        let a = box_surface(&mut mesh, 0.0, 1);
        let b = box_surface(&mut mesh, 0.8, 101);
        mesh.parts = vec![a, b];
        let search = |mesh: &FeMesh, one_sided: Vec<bool>| {
            let skins: Vec<_> = (mesh.parts.iter())
                .map(|p| extract_part_skin(mesh, p, 180.0))
                .collect();
            let parameters = SearchParameters {
                distance: 0.01,
                angle_deg: 35.0,
                group_by: GroupBy::Parts,
                stiffness: Vec::new(),
                filter: GeometryFilter {
                    solid: false,
                    shell: true,
                    shell_edge: false,
                },
                one_sided,
                solid_skins: Vec::new(),
            };
            find_contact_pairs(mesh, &skins, &[true, true], &parameters)
        };
        assert_eq!(search(&mesh, Vec::new()).len(), 1);
        assert!(orient_outward(&mut mesh, 0) && orient_outward(&mut mesh, 1));
        assert!(search(&mesh, vec![true, true]).is_empty());
        // Boxes side by side still touch with their faces at x = 1.
        let mut mesh = FeMesh::default();
        let a = box_surface(&mut mesh, 0.0, 1);
        let b = box_surface(&mut mesh, 1.0, 101);
        mesh.parts = vec![a, b];
        assert!(orient_outward(&mut mesh, 0) && orient_outward(&mut mesh, 1));
        assert_eq!(search(&mesh, vec![true, true]).len(), 1);
    }
}
