use glam::DVec3;
use plx_mesh::{PartSkin, SkinFace, face_normal};

use crate::contour::NO_VALUE;

use crate::mesh::{NodeBlend, RenderMesh, Vertex};

pub(crate) const EDGE_COLOR: [f32; 3] = [0.0, 0.0, 0.0];
/// Opacities of PrePoMax's edge actors; line vertices carry them in [`Vertex::scalar`], the
/// shader draws mesh edges with [`MESH_EDGE_OPACITY`] (`viewport.wgsl` repeats it).
pub const FEATURE_EDGE_OPACITY: f32 = 0.7;
pub const MESH_EDGE_OPACITY: f32 = 0.4;
pub const WIREFRAME_OPACITY: f32 = 0.5;
/// How far the mid node of a straight edge may lie off its centre, relative to its length.
pub(crate) const STRAIGHT_TOLERANCE: f64 = 1e-3;

/// PrePoMax's default part colours (System.Drawing names), cycled by part index.
const PART_COLORS_SRGB: [[u8; 3]; 10] = [
    [245, 245, 220], // Beige
    [143, 188, 143], // DarkSeaGreen
    [240, 230, 140], // Khaki
    [70, 130, 180],  // SteelBlue
    [222, 184, 135], // BurlyWood
    [176, 196, 222], // LightSteelBlue
    [255, 228, 225], // MistyRose
    [233, 150, 122], // DarkSalmon
    [189, 183, 107], // DarkKhaki
    [255, 222, 173], // NavajoWhite
];

/// sRGB colour of the n-th part; the viewport shades in display space like VTK.
pub fn part_color(index: usize) -> [f32; 3] {
    PART_COLORS_SRGB[index % PART_COLORS_SRGB.len()].map(|c| c as f32 / 255.0)
}

/// Builds the GPU-ready geometry of one part from node coordinates (undeformed or deformed).
/// Positions are shifted by `-origin`, so that large coordinates keep their precision in `f32`.
/// `scalars` holds one normalized contour value per node (see [`crate::contour::normalize`]).
/// `face_color` may give a skin face a colour of its own instead of `color`, e.g. to highlight
/// it.
///
/// Faces of the same surface patch that meet at less than `smooth_angle_deg` share averaged
/// vertex normals, so curved surfaces look round; across feature edges the shading stays flat.
/// Faces share the vertex of a node where its normal and colour agree, which keeps large meshes
/// small on the GPU; the mesh edges index the same vertices.
pub fn part_render_mesh(
    coords: &[[f64; 3]],
    skin: &PartSkin,
    origin: DVec3,
    color: [f32; 3],
    face_color: impl Fn(usize) -> Option<[f32; 3]>,
    smooth_angle_deg: f64,
    scalars: Option<&[f32]>,
) -> RenderMesh {
    let scalar = |node: usize| scalars.map_or(NO_VALUE, |s| s[node]);
    let position = |node: usize| (DVec3::from(coords[node]) - origin).as_vec3().to_array();
    let mut render = RenderMesh::default();
    let normals = smooth_vertex_normals(coords, skin, smooth_angle_deg);

    let mut shared = SharedVertices::new(node_span(skin));
    let mut normals = normals.into_iter();
    let mut local = Vec::with_capacity(8);
    for (index, face) in skin.faces.iter().enumerate() {
        let color = face_color(index).unwrap_or(color);
        local.clear();
        for (&node, normal) in face.corners.iter().chain(&face.mids).zip(&mut normals) {
            let vertex = Vertex {
                position: position(node),
                normal,
                color,
                scalar: scalar(node),
            };
            local.push(shared.index(&mut render, node, vertex));
        }
        if has_centre(face) {
            local.push(centre_vertex(
                &mut render,
                &mut shared,
                &local,
                face,
                coords,
                origin,
                color,
                &scalar,
            ));
        }
        (render.triangles).extend(face_triangles(face).iter().map(|&k| local[k as usize]));
    }

    let line_vertex = |node: usize, color: [f32; 3], opacity: f32| Vertex {
        position: position(node),
        normal: [0.0; 3],
        color,
        scalar: opacity,
    };
    for edge in &skin.edges {
        // A straight mesh edge of a quadratic element is one line: there are many of them.
        let straight = |mid: usize| {
            let [a, b, m] = [edge.a, edge.b, mid].map(|n| DVec3::from(coords[n]));
            (m - (a + b) * 0.5).length() <= STRAIGHT_TOLERANCE * (b - a).length()
        };
        let segments: &[[usize; 2]] = match edge.mid {
            Some(mid) if edge.feature || !straight(mid) => &[[edge.a, mid], [mid, edge.b]],
            _ => &[[edge.a, edge.b]],
        };
        for &[a, b] in segments {
            if edge.feature {
                render.feature_edges.extend([
                    line_vertex(a, EDGE_COLOR, FEATURE_EDGE_OPACITY),
                    line_vertex(b, EDGE_COLOR, FEATURE_EDGE_OPACITY),
                ]);
            } else {
                // Any vertex of the node lies in the right place; mesh edges are drawn in
                // their own colour. Edges of faces always have one.
                for node in [a, b] {
                    let index = shared.any(node).unwrap_or_else(|| {
                        let vertex = line_vertex(node, EDGE_COLOR, MESH_EDGE_OPACITY);
                        shared.index(&mut render, node, vertex)
                    });
                    render.mesh_edges.push(index);
                }
            }
        }
    }
    let line_color = color.map(|c| c * 0.5);
    for &[a, b] in &skin.lines {
        render.feature_edges.extend([
            line_vertex(a, line_color, 1.0),
            line_vertex(b, line_color, 1.0),
        ]);
    }
    render
}

/// The node indices a skin uses lie in this range. A part of a large assembly uses few of
/// the nodes of the mesh, mostly numbered in one block, so tables by node are kept to it:
/// tables over all nodes would make building every part as slow as the whole mesh.
pub(crate) fn node_span(skin: &PartSkin) -> std::ops::Range<usize> {
    let faces = skin.faces.iter().flat_map(nodes);
    let edges = (skin.edges.iter()).flat_map(|e| [e.a, e.b].into_iter().chain(e.mid));
    let lines = skin.lines.iter().flatten().copied();
    let (lo, hi) = (faces.chain(edges).chain(lines))
        .fold((usize::MAX, 0), |(lo, hi), n| (lo.min(n), hi.max(n + 1)));
    lo.min(hi)..hi
}

/// The vertices of each node as a linked list: the first one, and per vertex the next.
struct SharedVertices {
    /// The first node of the table.
    offset: usize,
    first: Vec<u32>,
    next: Vec<u32>,
}

impl SharedVertices {
    const NONE: u32 = u32::MAX;

    fn new(nodes: std::ops::Range<usize>) -> Self {
        Self {
            offset: nodes.start,
            first: vec![Self::NONE; nodes.len()],
            next: Vec::new(),
        }
    }

    /// Index of a vertex of the node with the same normal and colour, added if there is none.
    fn index(&mut self, render: &mut RenderMesh, node: usize, vertex: Vertex) -> u32 {
        let slot = node - self.offset;
        let mut index = self.first[slot];
        while index != Self::NONE {
            let other = &render.vertices[index as usize];
            if other.normal == vertex.normal && other.color == vertex.color {
                return index;
            }
            index = self.next[index as usize];
        }
        let index = render.vertices.len() as u32;
        render.vertices.push(vertex);
        render.origins.push(NodeBlend::at(node));
        self.next.push(self.first[slot]);
        self.first[slot] = index;
        index
    }

    /// Index of a new vertex at no node, which no other face shares.
    fn unshared(&mut self, render: &mut RenderMesh, vertex: Vertex, origin: NodeBlend) -> u32 {
        let index = render.vertices.len() as u32;
        render.vertices.push(vertex);
        render.origins.push(origin);
        self.next.push(Self::NONE);
        index
    }

    /// Index of some vertex of the node.
    fn any(&self, node: usize) -> Option<u32> {
        Some(self.first[node - self.offset]).filter(|&i| i != Self::NONE)
    }
}

/// Feature edges of a part as translucent black lines, PrePoMax's wireframe of the undeformed
/// shape behind deformed results.
pub fn wireframe_edges(coords: &[[f64; 3]], skin: &PartSkin, origin: DVec3) -> Vec<Vertex> {
    let vertex = |node: usize| Vertex {
        position: (DVec3::from(coords[node]) - origin).as_vec3().to_array(),
        normal: [0.0; 3],
        color: EDGE_COLOR,
        scalar: WIREFRAME_OPACITY,
    };
    let mut lines = Vec::new();
    for edge in skin.edges.iter().filter(|e| e.feature) {
        let nodes: &[usize] = match edge.mid {
            Some(mid) => &[edge.a, mid, mid, edge.b],
            None => &[edge.a, edge.b],
        };
        lines.extend(nodes.iter().map(|&n| vertex(n)));
    }
    for &[a, b] in &skin.lines {
        lines.extend([vertex(a), vertex(b)]);
    }
    lines
}

/// Every edge of the faces of a part as lines in the colour of the wireframe of the
/// undeformed shape, such as the global model drawn around the results of a submodel.
pub fn mesh_wireframe(coords: &[[f64; 3]], skin: &PartSkin, origin: DVec3) -> Vec<Vertex> {
    let vertex = |node: usize| Vertex {
        position: (DVec3::from(coords[node]) - origin).as_vec3().to_array(),
        normal: [0.0; 3],
        color: EDGE_COLOR,
        scalar: WIREFRAME_OPACITY,
    };
    let mut lines = Vec::new();
    for edge in &skin.edges {
        let nodes: &[usize] = match edge.mid {
            Some(mid) => &[edge.a, mid, mid, edge.b],
            None => &[edge.a, edge.b],
        };
        lines.extend(nodes.iter().map(|&n| vertex(n)));
    }
    for &[a, b] in &skin.lines {
        lines.extend([vertex(a), vertex(b)]);
    }
    lines
}

/// For each face in turn, one normal for each of its corner and mid nodes: the area-weighted
/// average of the normals of all faces of the same patch at that node that deviate from this
/// face by less than the angle.
pub(crate) fn smooth_vertex_normals(
    coords: &[[f64; 3]],
    skin: &PartSkin,
    smooth_angle_deg: f64,
) -> Vec<[f32; 3]> {
    let cos_limit = smooth_angle_deg.to_radians().cos();
    let face_normals: Vec<DVec3> = skin
        .faces
        .iter()
        .map(|f| DVec3::from(face_normal(coords, &f.corners)))
        .collect();
    let weights: Vec<f64> = skin
        .faces
        .iter()
        .map(|f| polygon_area(coords, &f.corners))
        .collect();
    // The faces at each node, as one list in node order: those of node n start at
    // start[n - offset].
    let span = node_span(skin);
    let offset = span.start;
    let mut start = vec![0usize; span.len() + 1];
    for node in skin.faces.iter().flat_map(nodes) {
        start[node - offset + 1] += 1;
    }
    for n in 0..span.len() {
        start[n + 1] += start[n];
    }
    let mut filled = start.clone();
    let mut faces_at_node = vec![0usize; start[span.len()]];
    for (index, face) in skin.faces.iter().enumerate() {
        for node in nodes(face) {
            faces_at_node[filled[node - offset]] = index;
            filled[node - offset] += 1;
        }
    }
    let mut normals = Vec::with_capacity(faces_at_node.len());
    for (face, &own) in skin.faces.iter().zip(&face_normals) {
        for node in nodes(face).map(|n| n - offset) {
            let sum: DVec3 = faces_at_node[start[node]..start[node + 1]]
                .iter()
                .filter(|&&other| {
                    skin.faces[other].region == face.region
                        && own.dot(face_normals[other]) >= cos_limit
                })
                .map(|&other| face_normals[other] * weights[other])
                .sum();
            normals.push(sum.try_normalize().unwrap_or(own).as_vec3().to_array());
        }
    }
    normals
}

fn nodes(face: &SkinFace) -> impl Iterator<Item = usize> + '_ {
    face.corners.iter().chain(&face.mids).copied()
}

fn polygon_area(coords: &[[f64; 3]], corners: &[usize]) -> f64 {
    let p0 = DVec3::from(coords[corners[0]]);
    (1..corners.len() - 1)
        .map(|i| {
            let (a, b) = (
                DVec3::from(coords[corners[i]]),
                DVec3::from(coords[corners[i + 1]]),
            );
            (a - p0).cross(b - p0).length() * 0.5
        })
        .sum::<f64>()
        .max(f64::MIN_POSITIVE)
}

/// Triangles of a face as indices into its corner-then-mid vertex list, followed for a
/// quadratic quad by its centre vertex (see [`has_centre`]).
pub(crate) fn face_triangles(face: &SkinFace) -> &'static [u32] {
    match (face.corners.len(), face.mids.is_empty()) {
        (3, true) => &[0, 1, 2],
        (4, true) => &[0, 1, 2, 0, 2, 3],
        // Corners 0..3, mids 3..6 (m0 between c0 and c1, …).
        (3, false) => &[0, 3, 5, 3, 1, 4, 5, 4, 2, 3, 4, 5],
        // Corners 0..4, mids 4..8, centre 8: a triangle at each corner and four around the
        // centre.
        (4, false) => &[
            0, 4, 7, 4, 1, 5, 5, 2, 6, 6, 3, 7, 4, 5, 8, 5, 6, 8, 6, 7, 8, 7, 4, 8,
        ],
        _ => &[],
    }
}

/// Whether the face is drawn with a vertex at its centre: a quadratic quad. Its inner four
/// mids would otherwise be split by a diagonal between two opposite mids, and a value that
/// is high at both but low in between, such as the von Mises stress across the thickness of
/// a bent shell, would be drawn high along that diagonal (a pattern per element).
pub(crate) fn has_centre(face: &SkinFace) -> bool {
    face.corners.len() == 4 && face.mids.len() == 4
}

/// The value at the centre of a quadratic quad from its corner and mid values, as the
/// element interpolates it: -1/4 of each corner and 1/2 of each mid.
pub(crate) fn quad8_centre<T>(values: impl IntoIterator<Item = T>) -> T
where
    T: std::ops::Add<Output = T> + std::ops::Mul<f64, Output = T> + Default,
{
    (values.into_iter().enumerate()).fold(T::default(), |sum, (k, v)| {
        sum + v * if k < 4 { -0.25 } else { 0.5 }
    })
}

/// The centre contour value of a quadratic quad from its normalized node values: none if a
/// node has none, else kept in the normalized range.
pub(crate) fn centre_scalar(scalars: impl IntoIterator<Item = f32>) -> f32 {
    let scalars: Vec<f32> = scalars.into_iter().collect();
    if scalars.iter().any(|&s| s < 0.0) {
        return NO_VALUE;
    }
    quad8_centre(scalars.iter().map(|&s| f64::from(s))).clamp(0.0, 1.0) as f32
}

/// The centre vertex of a quadratic quad whose eight vertices are `local`: its position
/// and value as the element interpolates them, the mean normal of its vertices.
#[allow(clippy::too_many_arguments)]
fn centre_vertex(
    render: &mut RenderMesh,
    shared: &mut SharedVertices,
    local: &[u32],
    face: &SkinFace,
    coords: &[[f64; 3]],
    origin: DVec3,
    color: [f32; 3],
    scalar: &impl Fn(usize) -> f32,
) -> u32 {
    let nodes: Vec<usize> = nodes(face).collect();
    let position = quad8_centre(nodes.iter().map(|&n| DVec3::from(coords[n])));
    let normal = (local.iter())
        .map(|&v| glam::Vec3::from(render.vertices[v as usize].normal))
        .sum::<glam::Vec3>()
        .normalize_or_zero();
    let vertex = Vertex {
        position: (position - origin).as_vec3().to_array(),
        normal: normal.to_array(),
        color,
        scalar: centre_scalar(nodes.iter().map(|&n| scalar(n))),
    };
    // Directions of LIC between two opposite mids.
    let origin = NodeBlend {
        a: face.mids[0] as u32,
        b: face.mids[2] as u32,
        t: 0.5,
    };
    shared.unshared(render, vertex, origin)
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_mesh::{Element, ElementShape, FeMesh, Part, extract_part_skin};

    fn single_element(shape: ElementShape, coords: &[[f64; 3]]) -> FeMesh {
        let mut mesh = FeMesh::default();
        for (i, c) in coords.iter().enumerate() {
            mesh.set_node(i as u32 + 1, *c);
        }
        mesh.add_element(Element {
            id: 1,
            type_name: format!("{shape:?}"),
            shape,
            nodes: (1..=coords.len() as u32).collect(),
        })
        .unwrap();
        mesh.parts.push(Part {
            name: "P".into(),
            elements: vec![1],
        });
        mesh
    }

    fn triangle_area_sum(render: &RenderMesh) -> f32 {
        render
            .triangles
            .chunks(3)
            .map(|t| {
                let [a, b, c] =
                    [0, 1, 2].map(|k| glam::Vec3::from(render.vertices[t[k] as usize].position));
                (b - a).cross(c - a).length() * 0.5
            })
            .sum()
    }

    #[test]
    fn quadratic_quad_is_split_without_gaps() {
        let coords = [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 2.0, 0.0],
            [0.0, 2.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [1.0, 2.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        let mesh = single_element(ElementShape::Quad8, &coords);
        let skin = extract_part_skin(&mesh, &mesh.parts[0], 30.0);
        let render = part_render_mesh(
            mesh.coords(),
            &skin,
            DVec3::ZERO,
            [1.0; 3],
            |_| None,
            30.0,
            None,
        );
        assert_eq!(render.triangles.len(), 8 * 3);
        assert!((triangle_area_sum(&render) - 4.0).abs() < 1e-5);
        // Four curved boundary edges of two segments each, all on the outline.
        assert_eq!(render.feature_edges.len(), 4 * 2 * 2);
        assert!(render.mesh_edges.is_empty());
    }

    /// The side of a bent shell expanded to a solid: von Mises is high on the top and the
    /// bottom and low in the middle of the thickness. With a diagonal between the top and
    /// bottom mids, the middle of each element was drawn high, the edges low (a pattern
    /// per element); with the centre vertex the middle line stays low throughout.
    #[test]
    fn quadratic_quad_centre_follows_the_element_interpolation() {
        // Corners bottom-left, bottom-right, top-right, top-left; mids bottom, right, top,
        // left. Thickness along y.
        let coords = [
            [0.0, -1.0, 0.0],
            [1.0, -1.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.5, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.5, 1.0, 0.0],
            [0.0, 0.0, 0.0],
        ];
        let scalars = [1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 0.0];
        let mesh = single_element(ElementShape::Quad8, &coords);
        let skin = extract_part_skin(&mesh, &mesh.parts[0], 30.0);
        let render = part_render_mesh(
            mesh.coords(),
            &skin,
            DVec3::ZERO,
            [1.0; 3],
            |_| None,
            30.0,
            Some(&scalars),
        );
        let centre = (render.vertices.iter())
            .find(|v| (v.position[0] - 0.5).abs() < 1e-6 && v.position[1].abs() < 1e-6)
            .expect("a vertex at the centre");
        assert!(centre.scalar.abs() < 1e-6, "centre {}", centre.scalar);
        // No triangle edge joins the top and the bottom mid.
        let bottom_top = |a: &Vertex, b: &Vertex| {
            (a.position[0] - 0.5).abs() < 1e-6
                && (b.position[0] - 0.5).abs() < 1e-6
                && (a.position[1] - b.position[1]).abs() > 1.5
        };
        for t in render.triangles.chunks(3) {
            for (i, j) in [(0, 1), (1, 2), (2, 0)] {
                let (a, b) = (
                    &render.vertices[t[i] as usize],
                    &render.vertices[t[j] as usize],
                );
                assert!(!bottom_top(a, b));
            }
        }
    }

    #[test]
    fn neighbouring_quadratic_quads_share_their_edge_nodes() {
        let mut mesh = FeMesh::default();
        // Two Quad8 side by side along x, sharing the edge at x = 1.
        let xs = [0.0, 0.5, 1.0, 1.5, 2.0];
        let mut id = 0;
        let mut node = |x: f64, y: f64| {
            id += 1;
            mesh.set_node(id, [x, y, 0.0]);
            id
        };
        let bottom: Vec<u32> = xs.iter().map(|&x| node(x, 0.0)).collect();
        let top: Vec<u32> = xs.iter().map(|&x| node(x, 2.0)).collect();
        let middle: Vec<u32> = [0.0, 1.0, 2.0].iter().map(|&x| node(x, 1.0)).collect();
        for (e, k) in [0usize, 2].into_iter().enumerate() {
            let nodes = vec![
                bottom[k],
                bottom[k + 2],
                top[k + 2],
                top[k],
                bottom[k + 1],
                middle[e + 1],
                top[k + 1],
                middle[e],
            ];
            mesh.add_element(Element {
                id: e as u32 + 1,
                type_name: "Quad8".into(),
                shape: ElementShape::Quad8,
                nodes,
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "P".into(),
            elements: vec![1, 2],
        });
        let skin = extract_part_skin(&mesh, &mesh.parts[0], 30.0);
        let render = part_render_mesh(
            mesh.coords(),
            &skin,
            DVec3::ZERO,
            [1.0; 3],
            |_| None,
            30.0,
            None,
        );
        // 13 nodes and a centre per face.
        assert_eq!(render.vertices.len(), 13 + 2);
        assert_eq!(render.origins.len(), render.vertices.len());
        assert_eq!(render.triangles.len(), 2 * 8 * 3);
        assert!((triangle_area_sum(&render) - 4.0).abs() < 1e-5);
    }

    #[test]
    fn centre_scalar_is_missing_where_a_node_has_none() {
        assert_eq!(
            centre_scalar([0.5; 7].into_iter().chain([NO_VALUE])),
            NO_VALUE
        );
        assert!((centre_scalar([0.5; 8]) - 0.5).abs() < 1e-6);
        // Kept in range when the interpolation overshoots.
        assert_eq!(centre_scalar([0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]), 1.0);
    }

    #[test]
    fn quadratic_tet_surface_area_is_preserved() {
        let corners = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let mid = |a: usize, b: usize| [0, 1, 2].map(|k| (corners[a][k] + corners[b][k]) / 2.0);
        let mut coords = corners.to_vec();
        coords.extend([
            mid(0, 1),
            mid(1, 2),
            mid(2, 0),
            mid(0, 3),
            mid(1, 3),
            mid(2, 3),
        ]);
        let mesh = single_element(ElementShape::Tet10, &coords);
        let skin = extract_part_skin(&mesh, &mesh.parts[0], 30.0);
        let render = part_render_mesh(
            mesh.coords(),
            &skin,
            DVec3::ZERO,
            [1.0; 3],
            |_| None,
            30.0,
            None,
        );
        let expected = 3.0 * 0.5 + 3.0_f32.sqrt() / 2.0;
        assert!((triangle_area_sum(&render) - expected).abs() < 1e-5);
    }

    #[test]
    fn positions_are_relative_to_origin() {
        let coords = [[1000.0, 0.0, 0.0], [1001.0, 0.0, 0.0], [1000.0, 1.0, 0.0]];
        let mesh = single_element(ElementShape::Tri3, &coords);
        let skin = extract_part_skin(&mesh, &mesh.parts[0], 30.0);
        let render = part_render_mesh(
            mesh.coords(),
            &skin,
            DVec3::new(1000.0, 0.0, 0.0),
            [1.0; 3],
            |_| None,
            30.0,
            None,
        );
        assert_eq!(render.vertices[1].position, [1.0, 0.0, 0.0]);
    }

    #[test]
    fn normals_are_smoothed_across_shallow_edges_only() {
        // Two triangles folded by 20° along x and a third folded by 90°, all sharing node 0.
        let fold = 20f64.to_radians();
        let coords = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -fold.cos(), fold.sin()],
            [0.0, 0.0, 1.0],
        ];
        let mut mesh = FeMesh::default();
        for (i, c) in coords.iter().enumerate() {
            mesh.set_node(i as u32 + 1, *c);
        }
        for (id, nodes) in [(1, vec![1, 2, 3]), (2, vec![1, 4, 2]), (3, vec![1, 3, 5])] {
            mesh.add_element(Element {
                id,
                type_name: "S3".into(),
                shape: ElementShape::Tri3,
                nodes,
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "P".into(),
            elements: vec![1, 2, 3],
        });
        let skin = extract_part_skin(&mesh, &mesh.parts[0], 30.0);
        let render = part_render_mesh(
            mesh.coords(),
            &skin,
            DVec3::ZERO,
            [1.0; 3],
            |_| None,
            30.0,
            None,
        );
        // Node 0 is the first corner of each face.
        let vertex_at_origin = |face: usize| render.triangles[face * 3] as usize;
        let normal_at_origin =
            |face: usize| glam::Vec3::from(render.vertices[vertex_at_origin(face)].normal);
        // The flat face and the 20° face blend, the 90° face keeps its own normal.
        let blended = normal_at_origin(0);
        assert!(
            (blended.angle_between(glam::Vec3::Z).to_degrees() - 10.0).abs() < 1e-3,
            "{blended}"
        );
        assert!((normal_at_origin(1) - blended).length() < 1e-6);
        assert!((normal_at_origin(2) - glam::Vec3::X).length() < 1e-6);
        // Faces with the same normal at a node share its vertex.
        assert_eq!(vertex_at_origin(0), vertex_at_origin(1));
        assert_ne!(vertex_at_origin(0), vertex_at_origin(2));
    }

    #[test]
    fn faces_share_vertices_and_mesh_edges_index_them() {
        // Two coplanar quadratic triangles sharing the edge from node 1 to 2 (mid node 4).
        let coords = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.5, 0.5, 0.0],
            [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [1.0, 0.5, 0.0],
            [0.5, 1.0, 0.0],
        ];
        let mut mesh = FeMesh::default();
        for (i, c) in coords.iter().enumerate() {
            mesh.set_node(i as u32 + 1, *c);
        }
        for (id, nodes) in [(1, vec![1, 2, 3, 6, 5, 7]), (2, vec![2, 4, 3, 8, 9, 5])] {
            mesh.add_element(Element {
                id,
                type_name: "S6".into(),
                shape: ElementShape::Tri6,
                nodes,
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "P".into(),
            elements: vec![1, 2],
        });
        let skin = extract_part_skin(&mesh, &mesh.parts[0], 30.0);
        let render = part_render_mesh(
            mesh.coords(),
            &skin,
            DVec3::ZERO,
            [1.0; 3],
            |_| None,
            30.0,
            None,
        );
        assert_eq!(render.vertices.len(), coords.len());
        assert!((triangle_area_sum(&render) - 1.0).abs() < 1e-5);
        // The inner edge is the only mesh edge; it is straight, so one line.
        assert_eq!(render.mesh_edges.len(), 2);
        let on_diagonal = |&i: &u32| {
            let [x, y, _] = render.vertices[i as usize].position;
            (x + y - 1.0).abs() < 1e-6
        };
        assert!(render.mesh_edges.iter().all(on_diagonal));
        // Bent, it is two.
        let mut bent = mesh.coords().to_vec();
        bent[4] = [0.55, 0.55, 0.0];
        let render = part_render_mesh(&bent, &skin, DVec3::ZERO, [1.0; 3], |_| None, 30.0, None);
        assert_eq!(render.mesh_edges.len(), 2 * 2);

        // A highlighted face keeps vertices of its own, so its colour does not bleed.
        let red = [1.0, 0.0, 0.0];
        let render = part_render_mesh(
            mesh.coords(),
            &skin,
            DVec3::ZERO,
            [1.0; 3],
            |face| (face == 1).then_some(red),
            30.0,
            None,
        );
        assert_eq!(render.vertices.len(), coords.len() + 3);
        let colors = |face: usize| {
            (render.triangles[face * 12..face * 12 + 12].iter())
                .map(|&i| render.vertices[i as usize].color)
                .collect::<Vec<_>>()
        };
        let (white, highlighted) = (colors(0), colors(1));
        assert!(highlighted.iter().all(|c| *c == red));
        assert!(white.iter().all(|c| *c == [1.0; 3]));
    }

    #[test]
    fn part_colors_cycle() {
        assert_eq!(part_color(0), part_color(PART_COLORS_SRGB.len()));
        assert!(part_color(0).iter().all(|c| (0.0..=1.0).contains(c)));
    }
}
