//! Shell and membrane elements drawn as the solid their section gives them: the surface
//! thickened by the section's thickness on both sides of its mid-surface, which lies off the
//! nodes by the section's offset.

use std::collections::HashMap;

use glam::DVec3;
use plx_mesh::PartSkin;

use crate::contour::NO_VALUE;
use crate::fe::{
    EDGE_COLOR, FEATURE_EDGE_OPACITY, STRAIGHT_TOLERANCE, centre_scalar, face_triangles,
    has_centre, quad8_centre, smooth_vertex_normals,
};
use crate::mesh::{NodeBlend, RenderMesh, Vertex};

/// Thickness of a shell and the offset of its nodes from the mid-surface, as a fraction of the
/// thickness, as CalculiX's `*SHELL SECTION` and `*MEMBRANE SECTION` take them: offset 0.5
/// puts the nodes on the top face, the side the element normal points to.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShellThickness {
    pub thickness: f64,
    pub offset: f64,
}

impl ShellThickness {
    /// Distances of the top and the bottom face from the nodes along the element normal.
    fn faces(self) -> [f64; 2] {
        [
            (0.5 - self.offset) * self.thickness,
            (-0.5 - self.offset) * self.thickness,
        ]
    }
}

/// The solid of the skin faces `shell` gives a thickness: their top and bottom faces, shaded
/// like the part, and side walls where the solid ends, at free edges and where faces meet at
/// a fold or with other thicknesses. The mesh edges of the faces are drawn on both sides.
///
/// Node normals are averaged over the faces of a surface patch like the shading normals of
/// [`crate::part_render_mesh`], so the solid of a curved surface has no gaps. `scalars` holds
/// a normalized contour value per node, as there; the solid takes the value of its node
/// across the thickness.
#[allow(clippy::too_many_arguments)]
pub fn thick_shell_mesh(
    coords: &[[f64; 3]],
    skin: &PartSkin,
    origin: DVec3,
    color: [f32; 3],
    face_color: impl Fn(usize) -> Option<[f32; 3]>,
    shell: impl Fn(usize) -> Option<ShellThickness>,
    smooth_angle_deg: f64,
    scalars: Option<&[f32]>,
) -> RenderMesh {
    let scalar = |node: usize| scalars.map_or(NO_VALUE, |s| s[node]);
    let mut render = RenderMesh::default();
    let shells: Vec<Option<ShellThickness>> = (0..skin.faces.len())
        .map(|index| shell(index).filter(|s| s.thickness > 0.0 && s.thickness.is_finite()))
        .collect();
    if shells.iter().all(Option::is_none) {
        return render;
    }
    let normals = smooth_vertex_normals(coords, skin, smooth_angle_deg);
    // Where the normals of each face start in `normals`.
    let mut first_normal = Vec::with_capacity(skin.faces.len());
    let mut count = 0;
    for face in &skin.faces {
        first_normal.push(count);
        count += face.corners.len() + face.mids.len();
    }
    let render_position = |p: DVec3| (p - origin).as_vec3().to_array();
    // Top and bottom position of each node of a face, corners first, then mids.
    let offset_positions = |index: usize| -> Vec<[DVec3; 2]> {
        let face = &skin.faces[index];
        let Some(shell) = shells[index] else {
            return Vec::new();
        };
        let [top, bottom] = shell.faces();
        (face.corners.iter().chain(&face.mids).enumerate())
            .map(|(k, &node)| {
                let n = DVec3::from(normals[first_normal[index] + k].map(f64::from));
                let p = DVec3::from(coords[node]);
                [p + n * top, p + n * bottom]
            })
            .collect()
    };
    let positions: Vec<Vec<[DVec3; 2]>> = (0..skin.faces.len()).map(offset_positions).collect();

    // Top and bottom faces; the vertices of a face's nodes on each side start at `first`.
    let mut first_vertex = vec![[0u32; 2]; skin.faces.len()];
    for (index, face) in skin.faces.iter().enumerate() {
        if shells[index].is_none() {
            continue;
        }
        let color = face_color(index).unwrap_or(color);
        for side in 0..2 {
            let base = render.vertices.len() as u32;
            first_vertex[index][side] = base;
            let nodes = face.corners.iter().chain(&face.mids);
            for (k, (position, &node)) in positions[index].iter().zip(nodes).enumerate() {
                let n = normals[first_normal[index] + k];
                render.vertices.push(Vertex {
                    position: render_position(position[side]),
                    normal: if side == 0 { n } else { n.map(|c| -c) },
                    color,
                    scalar: scalar(node),
                });
                render.origins.push(NodeBlend::at(node));
            }
            if has_centre(face) {
                let normal = (render.vertices[base as usize..].iter())
                    .map(|v| glam::Vec3::from(v.normal))
                    .sum::<glam::Vec3>()
                    .normalize_or_zero();
                let nodes = face.corners.iter().chain(&face.mids);
                render.vertices.push(Vertex {
                    position: render_position(quad8_centre(
                        positions[index].iter().map(|p| p[side]),
                    )),
                    normal: normal.to_array(),
                    color,
                    scalar: centre_scalar(nodes.map(|&n| scalar(n))),
                });
                render.origins.push(NodeBlend {
                    a: face.mids[0] as u32,
                    b: face.mids[2] as u32,
                    t: 0.5,
                });
            }
            let triangles = face_triangles(face);
            if side == 0 {
                render.triangles.extend(triangles.iter().map(|&k| base + k));
            } else {
                // The bottom faces the other way.
                for triangle in triangles.chunks(3) {
                    render
                        .triangles
                        .extend([2, 1, 0].map(|j| base + triangle[j]));
                }
            }
        }
    }

    // The sides of each edge of the thickened faces: (face, corner) by end nodes.
    let mut edges: HashMap<[usize; 2], Vec<(usize, usize)>> = HashMap::new();
    for (index, face) in skin.faces.iter().enumerate() {
        if shells[index].is_none() {
            continue;
        }
        let n = face.corners.len();
        for k in 0..n {
            let (a, b) = (face.corners[k], face.corners[(k + 1) % n]);
            edges
                .entry([a.min(b), a.max(b)])
                .or_default()
                .push((index, k));
        }
    }
    let feature: HashMap<[usize; 2], bool> = (skin.edges.iter())
        .map(|e| ([e.a.min(e.b), e.a.max(e.b)], e.feature))
        .collect();
    let line = |p: DVec3, opacity: f32| Vertex {
        position: render_position(p),
        normal: [0.0; 3],
        color: EDGE_COLOR,
        scalar: opacity,
    };
    // Whether the mid node of a quadratic edge lies on the line between its ends.
    let straight = |chain: &[[DVec3; 2]]| {
        let [a, m, b] = [chain[0][0], chain[1][0], chain[2][0]];
        (m - (a + b) * 0.5).length() <= STRAIGHT_TOLERANCE * (b - a).length()
    };
    let mut keys: Vec<&[usize; 2]> = edges.keys().collect();
    keys.sort_unstable();
    for key in keys {
        let sides = &edges[key];
        // Node chain of the edge on each face: positions of a, the mid node if any, b.
        let chain = |&(index, k): &(usize, usize)| -> Vec<[DVec3; 2]> {
            let face = &skin.faces[index];
            let n = face.corners.len();
            let mut chain = vec![positions[index][k]];
            if !face.mids.is_empty() {
                chain.push(positions[index][n + k]);
            }
            chain.push(positions[index][(k + 1) % n]);
            chain
        };
        // The nodes of the chain on a face.
        let chain_nodes = |&(index, k): &(usize, usize)| -> Vec<usize> {
            let face = &skin.faces[index];
            let n = face.corners.len();
            let mut nodes = vec![face.corners[k]];
            if !face.mids.is_empty() {
                nodes.push(face.mids[k]);
            }
            nodes.push(face.corners[(k + 1) % n]);
            nodes
        };
        let chains: Vec<Vec<[DVec3; 2]>> = sides.iter().map(chain).collect();
        // Two faces whose solids meet without a step share the edge. Usually the other
        // face runs along it the other way; with its normal flipped, its top is this face's
        // bottom.
        let joined = chains.len() == 2 && {
            let size = shells[sides[0].0].map_or(0.0, |s| s.thickness);
            let close = |p: DVec3, q: DVec3| (p - q).length() <= 1e-6 * size;
            let reversed: Vec<[DVec3; 2]> = chains[1].iter().rev().copied().collect();
            let matches = |other: &[[DVec3; 2]], flip: usize| {
                (chains[0].iter().zip(other))
                    .all(|(p, q)| close(p[0], q[flip]) && close(p[1], q[1 - flip]))
            };
            [0, 1]
                .iter()
                .any(|&flip| matches(&chains[1], flip) || matches(&reversed, flip))
        };
        if joined {
            let chain = &chains[0];
            if feature.get(key).copied().unwrap_or(false) {
                for side in 0..2 {
                    for pair in chain.windows(2) {
                        render.feature_edges.extend([
                            line(pair[0][side], FEATURE_EDGE_OPACITY),
                            line(pair[1][side], FEATURE_EDGE_OPACITY),
                        ]);
                    }
                }
            } else {
                // Mesh edges index the vertices of the first face; a straight edge of a
                // quadratic face is one line through its end nodes.
                let (index, k) = sides[0];
                let n = skin.faces[index].corners.len() as u32;
                let k = k as u32;
                let local: &[u32] = if chain.len() == 3 && !straight(chain) {
                    &[k, n + k, n + k, (k + 1) % n]
                } else {
                    &[k, (k + 1) % n]
                };
                for base in first_vertex[index] {
                    render.mesh_edges.extend(local.iter().map(|&j| base + j));
                }
            }
            continue;
        }
        // The solid ends at this edge: a wall across the thickness of each face.
        for (side, chain) in sides.iter().zip(&chains) {
            let color = face_color(side.0).unwrap_or(color);
            let nodes = chain_nodes(side);
            for (pair, ends) in chain.windows(2).zip(nodes.windows(2)) {
                let [[top_a, bottom_a], [top_b, bottom_b]] = [pair[0], pair[1]];
                let normal = (top_b - top_a)
                    .cross(top_a - bottom_a)
                    .try_normalize()
                    .unwrap_or(DVec3::Z)
                    .as_vec3()
                    .to_array();
                let base = render.vertices.len() as u32;
                for (p, node) in [top_a, top_b, bottom_b, bottom_a]
                    .into_iter()
                    .zip([0, 1, 1, 0])
                {
                    render.vertices.push(Vertex {
                        position: render_position(p),
                        normal,
                        color,
                        scalar: scalar(ends[node]),
                    });
                    render.origins.push(NodeBlend::at(ends[node]));
                }
                render
                    .triangles
                    .extend([0, 1, 2, 0, 2, 3].map(|k| base + k));
                for (a, b) in [(top_a, top_b), (bottom_a, bottom_b)] {
                    render
                        .feature_edges
                        .extend([line(a, FEATURE_EDGE_OPACITY), line(b, FEATURE_EDGE_OPACITY)]);
                }
            }
        }
    }
    render
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_mesh::{Element, ElementShape, FeMesh, Part, extract_part_skin};

    /// A flat square of two by two `S4` elements in the xy plane, normals along +z.
    fn plate() -> FeMesh {
        let mut mesh = FeMesh::default();
        for j in 0..3 {
            for i in 0..3 {
                mesh.set_node(j * 3 + i + 1, [i as f64, j as f64, 0.0]);
            }
        }
        let mut id = 1;
        for j in 0..2 {
            for i in 0..2 {
                let n = j * 3 + i + 1;
                mesh.add_element(Element {
                    id,
                    type_name: "S4".into(),
                    shape: ElementShape::Quad4,
                    nodes: vec![n, n + 1, n + 4, n + 3],
                })
                .unwrap();
                id += 1;
            }
        }
        mesh.parts.push(Part {
            bodies: Vec::new(),
            name: "P".into(),
            elements: vec![1, 2, 3, 4],
        });
        mesh
    }

    fn z_range(render: &RenderMesh) -> (f32, f32) {
        (render.vertices.iter().map(|v| v.position[2]))
            .fold((f32::MAX, f32::MIN), |(lo, hi), z| (lo.min(z), hi.max(z)))
    }

    fn thick(mesh: &FeMesh, shell: Option<ShellThickness>) -> RenderMesh {
        let skin = extract_part_skin(mesh, &mesh.parts[0], 30.0);
        thick_shell_mesh(
            mesh.coords(),
            &skin,
            DVec3::ZERO,
            [1.0; 3],
            |_| None,
            |_| shell,
            60.0,
            None,
        )
    }

    #[test]
    fn offset_moves_the_solid_off_the_nodes() {
        let mesh = plate();
        let centred = thick(
            &mesh,
            Some(ShellThickness {
                thickness: 0.2,
                offset: 0.0,
            }),
        );
        let (lo, hi) = z_range(&centred);
        assert!(
            (lo + 0.1).abs() < 1e-6 && (hi - 0.1).abs() < 1e-6,
            "{lo} {hi}"
        );
        // Offset 0.5: the nodes lie on the top face, the solid below them.
        let top = thick(
            &mesh,
            Some(ShellThickness {
                thickness: 0.2,
                offset: 0.5,
            }),
        );
        let (lo, hi) = z_range(&top);
        assert!((lo + 0.2).abs() < 1e-6 && hi.abs() < 1e-6, "{lo} {hi}");
        let bottom = thick(
            &mesh,
            Some(ShellThickness {
                thickness: 0.2,
                offset: -0.5,
            }),
        );
        let (lo, hi) = z_range(&bottom);
        assert!(lo.abs() < 1e-6 && (hi - 0.2).abs() < 1e-6, "{lo} {hi}");
    }

    #[test]
    fn walls_close_the_free_edges_only() {
        let render = thick(
            &plate(),
            Some(ShellThickness {
                thickness: 0.2,
                offset: 0.0,
            }),
        );
        // Four faces on each side and eight wall quads along the free edges.
        assert_eq!(render.triangles.len(), (2 * 4 * 2 + 8 * 2) * 3);
        // The four inner edges are mesh edges on both sides.
        assert_eq!(render.mesh_edges.len(), 4 * 2 * 2);
        // The outline at the top and the bottom of the walls.
        assert_eq!(render.feature_edges.len(), 8 * 2 * 2);
        // Closed: the wall area is the perimeter times the thickness.
        let area: f32 = (render.triangles.chunks(3))
            .map(|t| {
                let [a, b, c] =
                    [0, 1, 2].map(|k| glam::Vec3::from(render.vertices[t[k] as usize].position));
                (b - a).cross(c - a).length() * 0.5
            })
            .sum();
        assert!((area - (2.0 * 4.0 + 8.0 * 0.2)).abs() < 1e-4, "{area}");
    }

    #[test]
    fn faces_without_thickness_stay_out() {
        assert!(thick(&plate(), None).vertices.is_empty());
        let zero = ShellThickness {
            thickness: 0.0,
            offset: 0.0,
        };
        assert!(thick(&plate(), Some(zero)).vertices.is_empty());
    }

    #[test]
    fn a_flipped_neighbour_with_an_offset_gets_a_step() {
        let mut mesh = plate();
        // Element 2 with its normal along -z.
        let mut flipped = mesh.element(2).unwrap().clone();
        flipped.nodes.reverse();
        let mut rebuilt = FeMesh::default();
        for (index, &c) in mesh.coords().iter().enumerate() {
            rebuilt.set_node(index as u32 + 1, c);
        }
        for id in [1, 3, 4] {
            rebuilt
                .add_element(mesh.element(id).unwrap().clone())
                .unwrap();
        }
        rebuilt.add_element(flipped).unwrap();
        rebuilt.parts = std::mem::take(&mut mesh.parts);
        let shell = ShellThickness {
            thickness: 0.2,
            offset: 0.5,
        };
        let render = thick(&rebuilt, Some(shell));
        // Its solid lies above the plane, the others' below.
        let (lo, hi) = z_range(&render);
        assert!(
            (lo + 0.2).abs() < 1e-6 && (hi - 0.2).abs() < 1e-6,
            "{lo} {hi}"
        );
        // Its two inner edges become walls on both faces, the other two stay mesh edges.
        assert_eq!(render.mesh_edges.len(), 2 * 2 * 2);
        assert_eq!(render.triangles.len(), (2 * 4 * 2 + (8 + 4) * 2) * 3);
    }
}
