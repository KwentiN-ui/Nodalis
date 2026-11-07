//! Plane stress (CPS), plane strain (CPE) and axisymmetric (CAX) elements, expanded the way
//! the CalculiX manual describes them (sections "Eight-node plane stress element",
//! "Eight-node plane strain element", "Eight-node axisymmetric element"): every 2D element
//! is expanded into one layer of a 3D element (CPS8 -> C3D20, CPS8R -> C3D20R, CPS4 ->
//! C3D8, CPS4R -> C3D8R, CPS6 -> C3D15, CPS3 -> C3D6; the same for CPE and CAX), and the
//! expanded nodes are tied to the 2D node:
//!
//! * plane stress and plane strain: the expanded nodes lie at `z = ±t/2` (`t` the section
//!   thickness, 1 by default) and, for quadratic elements, at `z = 0` on the vertical edges;
//!   their in-plane displacements equal the 2D node's, no node moves out of plane;
//! * axisymmetric: the 2D coordinates are radius and axial position; the expansion rotates
//!   the section by ±1° about the y axis (a 2° segment), the expanded nodes move with the
//!   2D node in cylindrical coordinates (radial and axial displacement equal, no
//!   circumferential displacement). Loads and nodal forces count for the full
//!   circumference, so the segment's stiffness is scaled by 360°/2°.
//!
//! The strains of plane stress and plane strain elements therefore have no thickness and no
//! transverse shear part. Plane stress differs from plane strain in the material only: the
//! thickness strain is a local unknown of every integration point that makes the thickness
//! stress zero there (`plxs_materials::plane_stress`), so `σzz`, `σyz` and `σzx` are zero
//! at every point and the element is the exact 2D plane stress element. (The CalculiX manual's
//! plane stress expansion instead lets the expanded nodes off the midplane move out of plane, which makes the thickness
//! stress zero only on average; its integration points then carry thickness stresses of
//! some percent of the in-plane ones.) The third DOF of plane stress, plane strain and
//! axisymmetric nodes is unused ([`SolidElement::uses_out_of_plane`]).
//!
//! Stresses and strains are those of the expanded element at its integration points. The
//! 2D node gets the thickness average of the values extrapolated to its expanded nodes,
//! with Simpson's weights `(bottom + 4 middle + top) / 6` where the 3D element has a node in
//! the midplane and the plain mean of bottom and top elsewhere. That rule reproduces what
//! CalculiX writes; it was found by fitting its nodal stresses of single distorted elements
//! under random displacement fields against its integration-point stresses (black-box use;
//! no CalculiX source was consulted).

use plxs_core::{ElementFamily, ElementType};

use crate::shape::Family;
use crate::solid::{SolidElement, solid};

/// How the 2D element relates to its expansion.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlaneKind {
    Stress,
    Strain,
    Axisymmetric,
}

/// Where an expanded node lies across the thickness.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Layer {
    Bottom,
    Middle,
    Top,
}

impl Layer {
    fn sign(self) -> f64 {
        match self {
            Layer::Bottom => -1.0,
            Layer::Middle => 0.0,
            Layer::Top => 1.0,
        }
    }
}

/// The expansion of a 2D element: what its 3D element is and how its nodes follow the 2D
/// nodes.
#[derive(Clone, Debug)]
pub struct Expansion {
    pub kind: PlaneKind,
    /// Thickness of plane elements; for axisymmetric ones unused.
    pub thickness: f64,
    /// Number of 2D nodes.
    pub nodes: usize,
    /// For every node of the 3D element: the 2D node it belongs to and its layer.
    map: Vec<(usize, Layer)>,
    /// For every DOF of the 3D element (3 per node): the 2D DOF it follows and the factor,
    /// `None` when it is held at zero.
    dofs: Vec<Option<(usize, f64)>>,
    /// Factor on stiffness, forces and volume: 1 for plane elements, 180 for the 2°
    /// segment of axisymmetric ones.
    pub scale: f64,
}

/// Half the opening angle of the axisymmetric segment: 1°.
const HALF_ANGLE: f64 = std::f64::consts::PI / 180.0;

/// The 3D element type a 2D type is expanded into.
pub fn expanded_type(kind: ElementType) -> Option<ElementType> {
    use ElementType::*;
    Some(match kind {
        CPS3 | CPE3 | CAX3 => C3D6,
        CPS4 | CPE4 | CAX4 => C3D8,
        CPS4R | CPE4R | CAX4R => C3D8R,
        CPS6 | CPE6 | CAX6 => C3D15,
        CPS8 | CPE8 | CAX8 => C3D20,
        CPS8R | CPE8R | CAX8R => C3D20R,
        _ => return None,
    })
}

/// The element of a plane stress, plane strain or axisymmetric type with the section
/// thickness `thickness` (`None`: 1), `None` for other types.
pub fn plane(kind: ElementType, thickness: Option<f64>) -> Option<SolidElement> {
    let plane_kind = match kind.family() {
        ElementFamily::PlaneStress => PlaneKind::Stress,
        ElementFamily::PlaneStrain => PlaneKind::Strain,
        ElementFamily::Axisymmetric => PlaneKind::Axisymmetric,
        _ => return None,
    };
    let mut element = solid(expanded_type(kind)?)?;
    let family = element.family;
    let nodes = kind.node_count();
    let map = node_map(family);
    debug_assert_eq!(map.len(), family.node_count());
    let thickness = thickness.filter(|t| *t > 0.0).unwrap_or(1.0);
    let mut expansion = Expansion {
        kind: plane_kind,
        thickness,
        nodes,
        map,
        dofs: Vec::new(),
        scale: if plane_kind == PlaneKind::Axisymmetric {
            std::f64::consts::PI / HALF_ANGLE
        } else {
            1.0
        },
    };
    expansion.dofs = expansion.build_dof_map();
    let extrapolation = expansion.extrapolation(element.extrapolation(), element.rule.len());
    element.kind = kind;
    element.set_expansion(expansion, extrapolation);
    Some(element)
}

/// The 2D node and layer of every node of the 3D element, in the 3D element's order.
fn node_map(family: Family) -> Vec<(usize, Layer)> {
    use Layer::*;
    match family {
        // Corners 1-4 bottom, 5-8 top, midsides 9-12 bottom, 13-16 top, 17-20 middle.
        Family::Hex8 => (0..4)
            .map(|a| (a, Bottom))
            .chain((0..4).map(|a| (a, Top)))
            .collect(),
        Family::Hex20 => (0..4)
            .map(|a| (a, Bottom))
            .chain((0..4).map(|a| (a, Top)))
            .chain((4..8).map(|a| (a, Bottom)))
            .chain((4..8).map(|a| (a, Top)))
            .chain((0..4).map(|a| (a, Middle)))
            .collect(),
        Family::Wedge6 => (0..3)
            .map(|a| (a, Bottom))
            .chain((0..3).map(|a| (a, Top)))
            .collect(),
        Family::Wedge15 => (0..3)
            .map(|a| (a, Bottom))
            .chain((0..3).map(|a| (a, Top)))
            .chain((3..6).map(|a| (a, Bottom)))
            .chain((3..6).map(|a| (a, Top)))
            .chain((0..3).map(|a| (a, Middle)))
            .collect(),
        Family::Tet4 | Family::Tet10 => unreachable!("tetrahedra are no expansions"),
    }
}

impl Expansion {
    /// For every node of the 3D element the 2D node it belongs to. Scalar fields
    /// (temperatures) are the same on all layers of a 2D node.
    pub fn scalar_map(&self) -> Vec<usize> {
        self.map.iter().map(|&(a, _)| a).collect()
    }

    /// Coordinates of the expanded nodes from those of the 2D nodes.
    pub fn coordinates(&self, coords: &[[f64; 3]]) -> Vec<[f64; 3]> {
        self.map
            .iter()
            .map(|&(a, layer)| {
                let c = coords[a];
                let s = layer.sign();
                match self.kind {
                    PlaneKind::Stress | PlaneKind::Strain => {
                        [c[0], c[1], c[2] + 0.5 * s * self.thickness]
                    }
                    PlaneKind::Axisymmetric => {
                        let phi = s * HALF_ANGLE;
                        [c[0] * phi.cos(), c[1], c[0] * phi.sin()]
                    }
                }
            })
            .collect()
    }

    /// For every DOF of the 3D element (3 per node): the 2D DOF it follows and the factor,
    /// `None` when it is held at zero.
    pub(crate) fn dof_map(&self) -> &[Option<(usize, f64)>] {
        &self.dofs
    }

    fn build_dof_map(&self) -> Vec<Option<(usize, f64)>> {
        let mut out = Vec::with_capacity(3 * self.map.len());
        for &(a, layer) in &self.map {
            let s = layer.sign();
            match self.kind {
                PlaneKind::Stress | PlaneKind::Strain => {
                    out.push(Some((3 * a, 1.0)));
                    out.push(Some((3 * a + 1, 1.0)));
                    out.push(None);
                }
                PlaneKind::Axisymmetric => {
                    let phi = s * HALF_ANGLE;
                    let (cx, cz) = (phi.cos(), phi.sin());
                    out.push(Some((3 * a, cx)));
                    out.push(Some((3 * a + 1, 1.0)));
                    out.push((s != 0.0).then_some((3 * a, cz)));
                }
            }
        }
        out
    }

    /// The displacements of the expanded nodes from the 2D ones.
    pub fn expand(&self, u: &[f64]) -> Vec<f64> {
        self.dof_map()
            .iter()
            .map(|m| m.map_or(0.0, |(d, f)| f * u[d]))
            .collect()
    }

    /// Forces on the 2D DOFs from forces on the expanded ones (`Tᵀ f`), scaled to the full
    /// circumference for axisymmetric elements, added to `out`.
    pub fn condense_vector(&self, f3: &[f64], out: &mut [f64]) {
        for (m, value) in self.dof_map().iter().zip(f3) {
            if let Some((d, factor)) = m {
                out[*d] += self.scale * factor * value;
            }
        }
    }

    /// `Tᵀ K T` of a 3D element matrix, scaled like the forces.
    pub fn condense_matrix(&self, k3: &[f64], out: &mut [f64]) {
        let n3 = 3 * self.map.len();
        let n2 = 3 * self.nodes;
        out.iter_mut().for_each(|v| *v = 0.0);
        let map = self.dof_map();
        for (i, mi) in map.iter().enumerate() {
            let Some((di, fi)) = mi else { continue };
            for (j, mj) in map.iter().enumerate() {
                let Some((dj, fj)) = mj else { continue };
                out[di * n2 + dj] += self.scale * fi * fj * k3[i * n3 + j];
            }
        }
    }

    /// The expanded nodes of 2D node `a` with their weights in the thickness average:
    /// Simpson's `(bottom + 4 middle + top) / 6` where the 3D element has a midplane node,
    /// the mean of bottom and top elsewhere.
    fn average_weights(&self, a: usize) -> Vec<(usize, f64)> {
        let of = |layer: Layer| self.map.iter().position(|&(b, l)| b == a && l == layer);
        let (bottom, top) = (of(Layer::Bottom).unwrap(), of(Layer::Top).unwrap());
        match of(Layer::Middle) {
            Some(m) => vec![(bottom, 1.0 / 6.0), (m, 4.0 / 6.0), (top, 1.0 / 6.0)],
            None => vec![(bottom, 0.5), (top, 0.5)],
        }
    }

    /// Displacements of the 2D nodes as CalculiX reports them: the thickness average of the
    /// Cartesian displacements of the expanded nodes (for axisymmetric elements the radial
    /// value is therefore slightly below the 2D node's own, by the cosine of the segment
    /// angle; out of plane it is zero).
    pub fn reported_displacements(&self, u: &[f64]) -> Vec<[f64; 3]> {
        let u3 = self.expand(u);
        (0..self.nodes)
            .map(|a| {
                let mut v = [0.0; 3];
                for (node, w) in self.average_weights(a) {
                    for k in 0..3 {
                        v[k] += w * u3[3 * node + k];
                    }
                }
                v
            })
            .collect()
    }

    /// Nodal forces of the 2D nodes as CalculiX reports them: the sum of the Cartesian
    /// forces on the expanded nodes, scaled to the full circumference. For axisymmetric
    /// elements the radial value also holds the radial parts of the circumferential forces
    /// of the segment faces, so it is not zero at unloaded nodes.
    pub fn reported_forces(&self, f3: &[f64]) -> Vec<[f64; 3]> {
        let mut out = vec![[0.0; 3]; self.nodes];
        for (j, &(a, _)) in self.map.iter().enumerate() {
            for k in 0..3 {
                out[a][k] += self.scale * f3[3 * j + k];
            }
        }
        out
    }

    /// Integration points to 2D nodes: the thickness average of the 3D element's
    /// extrapolation to the expanded nodes.
    fn extrapolation(&self, e3: &[f64], ips: usize) -> Vec<f64> {
        let mut out = vec![0.0; self.nodes * ips];
        for a in 0..self.nodes {
            for (node, w) in self.average_weights(a) {
                for g in 0..ips {
                    out[a * ips + g] += w * e3[node * ips + g];
                }
            }
        }
        out
    }

    /// The face of the 3D element that carries edge `edge` (1-based) of the 2D element.
    pub fn face(&self, edge: u8) -> u8 {
        edge + 2
    }

    /// Number of edges that can carry a distributed load.
    pub fn edges(&self) -> usize {
        match self.map.len() {
            6 | 15 => 3,
            _ => 4,
        }
    }
}
