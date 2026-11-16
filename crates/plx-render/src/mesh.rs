use bytemuck::{Pod, Zeroable};
use glam::Vec3;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Pod, Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 3],
    /// Surfaces: result value normalized to 0..1 for contour plots, negative where there is
    /// none. Lines: opacity.
    pub scalar: f32,
}

/// Where a vertex lies between two nodes, to give it values of the nodes: `a` and `b` are
/// node indices, the value is that of `a` blended towards `b` by `t`. A vertex at a node has
/// `a == b`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NodeBlend {
    pub a: u32,
    pub b: u32,
    pub t: f32,
}

impl NodeBlend {
    /// A vertex at no node, such as one of a section face cut through an element.
    pub const NONE: NodeBlend = NodeBlend {
        a: u32::MAX,
        b: u32::MAX,
        t: 0.0,
    };

    pub fn at(node: usize) -> Self {
        Self {
            a: node as u32,
            b: node as u32,
            t: 0.0,
        }
    }
}

/// Values of the vector source of LIC at a vertex: a vector in the first three, or a
/// symmetric tensor as xx, yy, zz, xy, yz, zx, scaled so that the largest magnitude is about
/// 1. Zero where there is none.
pub type Directions = [f32; 6];

/// Triangulated surface ready for upload, with edges as line lists (two vertices per segment).
#[derive(Clone, Debug, Default)]
pub struct RenderMesh {
    pub vertices: Vec<Vertex>,
    pub triangles: Vec<u32>,
    /// Outline of the part, always drawn; also holds line elements such as beams.
    pub feature_edges: Vec<Vertex>,
    /// Every element edge on the surface, drawn when the mesh is shown: pairs of indices into
    /// `vertices`, drawn in black whatever the vertex colours are.
    pub mesh_edges: Vec<u32>,
    /// Outline of the undeformed shape, drawn behind deformed results.
    pub wireframe_edges: Vec<Vertex>,
    /// Lines drawn [`WIDE_EDGE_PX`](crate::renderer::WIDE_EDGE_PX) wide over everything
    /// else, such as the outline of a selected part.
    pub wide_edges: Vec<Vertex>,
    /// Arrows of a vector field as line lists, drawn as wide lines over the surfaces.
    pub glyphs: Vec<Vertex>,
    /// Per vertex, where it lies between the nodes; may be shorter than `vertices`, the rest
    /// lie at no node.
    pub origins: Vec<NodeBlend>,
    /// Per vertex, the vector source of LIC; empty when the mesh shows none.
    pub directions: Vec<Directions>,
}

impl RenderMesh {
    /// Bytes the mesh takes in memory.
    pub fn bytes(&self) -> usize {
        fn of<T>(items: &Vec<T>) -> usize {
            items.capacity() * std::mem::size_of::<T>()
        }
        of(&self.vertices)
            + of(&self.triangles)
            + of(&self.feature_edges)
            + of(&self.mesh_edges)
            + of(&self.wireframe_edges)
            + of(&self.wide_edges)
            + of(&self.glyphs)
            + of(&self.origins)
            + of(&self.directions)
    }

    /// Adds another mesh to this one, e.g. a transformed copy of a part.
    pub fn append(&mut self, other: RenderMesh) {
        let base = self.vertices.len() as u32;
        // Directions and origins stay parallel to the vertices where either mesh has them.
        if !(self.directions.is_empty() && other.directions.is_empty()) {
            self.directions.resize(base as usize, [0.0; 6]);
            self.directions.extend(&other.directions);
            self.directions
                .resize(base as usize + other.vertices.len(), [0.0; 6]);
        }
        if !other.origins.is_empty() {
            self.origins.resize(base as usize, NodeBlend::NONE);
            self.origins.extend(other.origins);
        }
        self.vertices.extend(other.vertices);
        self.triangles
            .extend(other.triangles.into_iter().map(|index| base + index));
        self.feature_edges.extend(other.feature_edges);
        self.mesh_edges
            .extend(other.mesh_edges.into_iter().map(|index| base + index));
        self.wireframe_edges.extend(other.wireframe_edges);
        self.wide_edges.extend(other.wide_edges);
        self.glyphs.extend(other.glyphs);
    }

    /// Sets the directions of the vertices from values per node, blended along their
    /// [`RenderMesh::origins`]; vertices at no node get none.
    pub fn set_directions(&mut self, nodes: &[Directions]) {
        let value = |node: u32| nodes.get(node as usize).copied().unwrap_or([0.0; 6]);
        let mut directions: Vec<Directions> = (self.origins.iter())
            .map(|o| {
                let (a, b) = (value(o.a), value(o.b));
                std::array::from_fn(|k| a[k] + (b[k] - a[k]) * o.t)
            })
            .collect();
        directions.resize(self.vertices.len(), [0.0; 6]);
        self.directions = directions;
    }

    pub fn bounds(&self) -> Option<(Vec3, Vec3)> {
        let mut positions = self
            .vertices
            .iter()
            .chain(&self.feature_edges)
            .map(|v| Vec3::from(v.position));
        let first = positions.next()?;
        Some(positions.fold((first, first), |(min, max), p| (min.min(p), max.max(p))))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vertex() -> Vertex {
        Vertex {
            position: [0.0; 3],
            normal: [0.0; 3],
            color: [0.0; 3],
            scalar: 0.0,
        }
    }

    #[test]
    fn directions_blend_node_values_and_stay_parallel_to_the_vertices() {
        let nodes = [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [3.0, 2.0, 0.0, 0.0, 0.0, 0.0],
        ];
        let mut mesh = RenderMesh {
            vertices: vec![vertex(); 3],
            origins: vec![NodeBlend::at(0), NodeBlend { a: 0, b: 1, t: 0.5 }],
            ..Default::default()
        };
        mesh.set_directions(&nodes);
        assert_eq!(mesh.directions[0], nodes[0]);
        assert_eq!(mesh.directions[1][..2], [2.0, 1.0]);
        assert_eq!(mesh.directions[2], [0.0; 6], "a vertex at no node");

        // A mesh without directions, such as a thick shell, appends zeros.
        mesh.append(RenderMesh {
            vertices: vec![vertex(); 2],
            ..Default::default()
        });
        assert_eq!(mesh.directions.len(), 5);
        let mut plain = RenderMesh {
            vertices: vec![vertex()],
            ..Default::default()
        };
        plain.append(mesh);
        assert_eq!(plain.directions.len(), 6);
        assert_eq!(plain.directions[1], nodes[0]);
        assert_eq!(plain.origins.len(), 3, "padded up to the appended origins");
    }
}
