//! Beams: local directions at the nodes, the eight expanded nodes around the section and
//! the solid element on them (CalculiX manual, "Three-node beam element": B32 becomes
//! C3D20, B32R C3D20R, B31 C3D8I). The vector on the `*BEAM SECTION` card is the
//! 1-direction `n1`; the normal (2-direction) is `n2 = t × n1` and `n1` is made
//! orthogonal again as `n2 × t`.

use plxs_core::section::BeamProfile;
use plxs_core::{ElementType, Error, Result};

use super::{add, cross, dot, normalize, scale, sub};

/// The section data the expansion needs: the outer thicknesses in the 1- and
/// 2-direction, the offsets, and the profile for the integration.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct BeamSection {
    pub profile: BeamProfile,
    /// Thickness in the 1-direction and in the 2-direction.
    pub a: f64,
    pub b: f64,
    pub offset1: f64,
    pub offset2: f64,
}

impl BeamSection {
    pub fn new(profile: &BeamProfile, offset1: f64, offset2: f64) -> Result<Self> {
        let (a, b) = match *profile {
            BeamProfile::Rect { a, b } => (a, b),
            BeamProfile::Circ { a, b } => (a, b),
            BeamProfile::Pipe { radius, .. } => (2.0 * radius, 2.0 * radius),
            BeamProfile::Box { a, b, .. } => (a, b),
            BeamProfile::General { .. } => {
                return Err(Error::input(
                    "*BEAM SECTION",
                    "SECTION=GENERAL is only allowed on U1 elements",
                ));
            }
        };
        if !matches!(profile, BeamProfile::Rect { .. }) {
            return Err(Error::Unsupported(
                "beam sections other than RECT are not implemented yet".into(),
            ));
        }
        Ok(Self {
            profile: profile.clone(),
            a,
            b,
            offset1,
            offset2,
        })
    }

    /// The eight positions around a beam node: corners (-+, --, +-, ++) in (1, 2), then
    /// the edge midpoints (-0, 0-, +0, 0+), as CalculiX numbers them.
    pub fn positions(&self, x: [f64; 3], e1: [f64; 3], e2: [f64; 3]) -> Vec<[f64; 3]> {
        let centre = add(
            x,
            add(
                scale(e1, -self.offset1 * self.a),
                scale(e2, -self.offset2 * self.b),
            ),
        );
        SLOTS
            .iter()
            .map(|&(s1, s2)| {
                add(
                    centre,
                    add(scale(e1, 0.5 * s1 * self.a), scale(e2, 0.5 * s2 * self.b)),
                )
            })
            .collect()
    }
}

/// Section coordinates (in half thicknesses) of the eight expanded nodes.
const SLOTS: [(f64, f64); 8] = [
    (-1.0, 1.0),
    (-1.0, -1.0),
    (1.0, -1.0),
    (1.0, 1.0),
    (-1.0, 0.0),
    (0.0, -1.0),
    (1.0, 0.0),
    (0.0, 1.0),
];

/// The corner slots, which the mean constraint of a node without knot uses.
pub(super) fn edge_slots() -> [usize; 4] {
    [0, 1, 2, 3]
}

pub(super) fn solid_type(kind: ElementType) -> ElementType {
    use ElementType::*;
    match kind {
        B32 => C3D20,
        B32R => C3D20R,
        B31 => C3D8I,
        _ => unreachable!("not a beam: {kind}"),
    }
}

/// The solid's nodes as (local beam node, slot). B32 nodes are end, middle, end.
pub(super) fn layout(kind: ElementType) -> Vec<(usize, usize)> {
    use ElementType::*;
    match kind {
        B31 => vec![(0, 0), (1, 0), (1, 1), (0, 1), (0, 3), (1, 3), (1, 2), (0, 2)],
        B32 | B32R => vec![
            (0, 0),
            (2, 0),
            (2, 1),
            (0, 1),
            (0, 3),
            (2, 3),
            (2, 2),
            (0, 2),
            (1, 0),
            (2, 4),
            (1, 1),
            (0, 4),
            (1, 3),
            (2, 6),
            (1, 2),
            (0, 6),
            (0, 7),
            (2, 7),
            (2, 5),
            (0, 5),
        ],
        _ => unreachable!("not a beam: {kind}"),
    }
}

/// Tangent and 1-direction at every node of a beam element. The tangent follows the
/// element geometry (for B32 the derivative of the quadratic interpolation).
pub(super) fn directions(
    kind: ElementType,
    coords: &[[f64; 3]],
    n1: [f64; 3],
    id: u32,
) -> Result<(Vec<[f64; 3]>, Vec<[f64; 3]>)> {
    let tangents: Vec<[f64; 3]> = match kind {
        ElementType::B31 => {
            let t = normalize(sub(coords[1], coords[0]));
            vec![t, t]
        }
        _ => {
            // x(ξ) = N1 x1 + N2 x2 + N3 x3 with nodes at ξ = -1, 0, 1 (end, middle, end).
            let d = |xi: f64| -> [f64; 3] {
                let dn = [xi - 0.5, -2.0 * xi, xi + 0.5];
                let mut t = [0.0; 3];
                for k in 0..3 {
                    t = add(t, scale(coords[k], dn[k]));
                }
                normalize(t)
            };
            vec![d(-1.0), d(0.0), d(1.0)]
        }
    };
    let mut normals = Vec::with_capacity(tangents.len());
    for &t in &tangents {
        let n2 = cross(t, n1);
        if dot(n2, n2) < 1e-20 * dot(n1, n1).max(1e-300) {
            return Err(Error::input(
                "*BEAM SECTION",
                format!("the 1-direction of element {id} is parallel to its axis"),
            ));
        }
        normals.push(normalize(n2));
    }
    Ok((tangents, normals))
}

/// The local axes of a group from its mean tangent and normal: the normal made
/// orthogonal to the tangent, and the 1-direction `n2 × t`. Returns (n2, n1).
pub(super) fn local_axes(t: [f64; 3], n2: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let n2 = normalize(sub(n2, scale(t, dot(n2, t))));
    let n1 = normalize(cross(n2, t));
    (n2, n1)
}
