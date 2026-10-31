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
        // CIRC: all eight nodes on the ellipse, the corners at 45° (the quadratic solid then
        // approximates the elliptical section; ccx's expanded coordinates show this).
        let round = matches!(self.profile, BeamProfile::Circ { .. });
        SLOTS
            .iter()
            .map(|&(s1, s2)| {
                let (s1, s2) = if round && s1 != 0.0 && s2 != 0.0 {
                    (
                        s1 * std::f64::consts::FRAC_1_SQRT_2,
                        s2 * std::f64::consts::FRAC_1_SQRT_2,
                    )
                } else {
                    (s1, s2)
                };
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
        B31 => vec![
            (0, 0),
            (1, 0),
            (1, 1),
            (0, 1),
            (0, 3),
            (1, 3),
            (1, 2),
            (0, 2),
        ],
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

/// Tangents and normals at the nodes of a beam element.
pub(super) type Directions = (Vec<[f64; 3]>, Vec<[f64; 3]>);

/// Tangent and 1-direction at every node of a beam element. The tangent follows the
/// element geometry (for B32 the derivative of the quadratic interpolation).
pub(super) fn directions(
    kind: ElementType,
    coords: &[[f64; 3]],
    n1: [f64; 3],
    id: u32,
) -> Result<Directions> {
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

/// The integration rule and nodal extrapolation of an expanded beam whose section is not a
/// rectangle, `None` for rectangles (the solid's own rule). Natural coordinates of the
/// expanded solid: ξ along the beam, ζ along the 1-direction, η against the 2-direction.
///
/// PIPE (manual, "Pipe"): eight points equally spaced on the circle of radius
/// `sqrt((r_o² + r_i²) / 2)` starting on the 1-axis, each weighing an eighth of the ring
/// area; along the beam the two Gauss points of the reduced B32R (displacements match ccx).
/// Nodal values: per axial point `a + b ζ + c η + d ζ η` fitted to the eight values by least
/// squares, linear along the beam. ccx's nodal stresses follow this shape but differ by
/// up to 0.3 % (and 11 % in the ζη term); its rule is not known yet.
pub fn section_rule(
    kind: ElementType,
    profile: &BeamProfile,
) -> Option<(crate::shape::IntegrationRule, Vec<f64>)> {
    if kind != ElementType::C3D20R {
        return None;
    }
    // ccx 2.21 integrates B32R with a CIRC section the same way (the 2.11 manual still
    // says reduced Gauss; the oracle matches Gauss-Kronrod exactly).
    if matches!(profile, BeamProfile::Rect { .. } | BeamProfile::Circ { .. }) {
        return Some(gauss_kronrod_rule());
    }
    if let BeamProfile::Box {
        a,
        b,
        t1,
        t2,
        t3,
        t4,
    } = *profile
    {
        let t = [t1, t2, t3, t4];
        return Some(box_rule(a, b, t));
    }
    let BeamProfile::Pipe { radius, thickness } = *profile else {
        return None;
    };
    let inner = radius - thickness;
    let rho = ((radius * radius + inner * inner) / 2.0).sqrt() / radius;
    let ring = std::f64::consts::PI * (radius * radius - inner * inner) / (radius * radius);
    let g = 1.0 / 3f64.sqrt();
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for xi in [-g, g] {
        for k in 0..8 {
            let angle = (45.0 * k as f64).to_radians();
            points.push([xi, -rho * angle.sin(), rho * angle.cos()]);
            weights.push(ring / 8.0);
        }
    }
    let nodes = crate::shape::Family::Hex20.node_coordinates();
    let second = 4.0 * rho * rho; // Σ ζ_k² = Σ η_k² over the eight points
    let mut extrapolation = Vec::with_capacity(nodes.len() * points.len());
    for node in nodes {
        for p in &points {
            let along = 0.5 * (1.0 + node[0] * p[0].signum() / g);
            let across = 1.0 / 8.0
                + node[1] * p[1] / second
                + node[2] * p[2] / second
                + node[1] * node[2] * p[1] * p[2] / rho.powi(4);
            extrapolation.push(along * across);
        }
    }
    Some((
        crate::shape::IntegrationRule { points, weights },
        extrapolation,
    ))
}

/// B32R with a rectangular section (manual, "Three-node beam element"): 2 × 5 × 5
/// Gauss-Kronrod points against hourglassing, two Gauss points along the beam and the
/// five-point Kronrod extension of the two-point Gauss rule across the section (0,
/// ±1/√3, ±√(6/7) with weights 28/45, 27/55, 98/495). Nodal values come from the classical
/// 2 × 2 × 2 Gauss points the scheme contains, with the solid's own extrapolation.
fn gauss_kronrod_rule() -> (crate::shape::IntegrationRule, Vec<f64>) {
    let g = 1.0 / 3f64.sqrt();
    let k = (6.0f64 / 7.0).sqrt();
    let across = [
        (-k, 98.0 / 495.0),
        (-g, 27.0 / 55.0),
        (0.0, 28.0 / 45.0),
        (g, 27.0 / 55.0),
        (k, 98.0 / 495.0),
    ];
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for xi in [-g, g] {
        for &(eta, we) in &across {
            for &(zeta, wz) in &across {
                points.push([xi, eta, zeta]);
                weights.push(we * wz);
            }
        }
    }
    let standard = crate::solid::solid(ElementType::C3D20R).expect("C3D20R");
    let std_points = &standard.rule.points;
    let std_ips = std_points.len();
    let table = standard.extrapolation();
    let nodes = standard.node_count();
    let mut extrapolation = vec![0.0; nodes * points.len()];
    for (q, p) in points.iter().enumerate() {
        if let Some(sp) = std_points
            .iter()
            .position(|s| (0..3).all(|c| (s[c] - p[c]).abs() < 1e-12))
        {
            for a in 0..nodes {
                extrapolation[a * points.len() + q] = table[a * std_ips + sp];
            }
        }
    }
    (
        crate::shape::IntegrationRule { points, weights },
        extrapolation,
    )
}

/// BOX (manual, "Box"): Simpson's rule with five points along the centre line of each of
/// the four walls (16 points, the corners shared), each wall weighted by its thickness;
/// along the beam the two Gauss points. Walls in the order of the section card: at +1, +2,
/// -1 and -2. Nodal values as for the pipe: a bilinear fit across, linear along.
fn box_rule(a: f64, b: f64, t: [f64; 4]) -> (crate::shape::IntegrationRule, Vec<f64>) {
    // Natural coordinates: ζ along the 1-direction, η against the 2-direction.
    let z_plus = 1.0 - t[0] / a;
    let z_minus = -(1.0 - t[2] / a);
    let e_plus2 = -(1.0 - t[1] / b); // wall at +2
    let e_minus2 = 1.0 - t[3] / b; // wall at -2
    // Natural wall thicknesses.
    let (n1p, n2p, n1m, n2m) = (
        2.0 * t[0] / a,
        2.0 * t[1] / b,
        2.0 * t[2] / a,
        2.0 * t[3] / b,
    );
    // Points (η, ζ) with weights: each wall from corner to corner, Simpson 1-4-2-4-1.
    let mut section: Vec<([f64; 2], f64)> = Vec::new();
    let add_point = |p: [f64; 2], w: f64, list: &mut Vec<([f64; 2], f64)>| {
        if let Some(e) = list
            .iter_mut()
            .find(|(q, _)| (q[0] - p[0]).abs() < 1e-12 && (q[1] - p[1]).abs() < 1e-12)
        {
            e.1 += w;
        } else {
            list.push((p, w));
        }
    };
    let simpson = [1.0, 4.0, 2.0, 4.0, 1.0];
    let walls: [([f64; 2], [f64; 2], f64); 4] = [
        ([e_minus2, z_plus], [e_plus2, z_plus], n1p),
        ([e_plus2, z_plus], [e_plus2, z_minus], n2p),
        ([e_plus2, z_minus], [e_minus2, z_minus], n1m),
        ([e_minus2, z_minus], [e_minus2, z_plus], n2m),
    ];
    for (from, to, thickness) in walls {
        let length = ((to[0] - from[0]).powi(2) + (to[1] - from[1]).powi(2)).sqrt();
        let h = length / 4.0;
        for (k, s) in simpson.iter().enumerate() {
            let f = k as f64 / 4.0;
            let p = [
                from[0] + f * (to[0] - from[0]),
                from[1] + f * (to[1] - from[1]),
            ];
            add_point(p, thickness * h / 3.0 * s, &mut section);
        }
    }
    let g = 1.0 / 3f64.sqrt();
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for xi in [-g, g] {
        for (p, w) in &section {
            points.push([xi, p[0], p[1]]);
            weights.push(*w);
        }
    }
    let extrapolation = section_fit_extrapolation(&points, g);
    (
        crate::shape::IntegrationRule { points, weights },
        extrapolation,
    )
}

/// Nodal values from section points: per axial station the least-squares fit of
/// `a + b ζ + c η + d ζ η`, linear along the beam between the stations at ±g.
fn section_fit_extrapolation(points: &[[f64; 3]], g: f64) -> Vec<f64> {
    let nodes = crate::shape::Family::Hex20.node_coordinates();
    let station = |xi: f64| -> Vec<usize> {
        (0..points.len())
            .filter(|&i| (points[i][0] - xi).abs() < 1e-12)
            .collect()
    };
    let mut out = vec![0.0; nodes.len() * points.len()];
    for xi in [-g, g] {
        let ids = station(xi);
        // Normal equations of the fit, 4 × 4.
        let basis = |p: &[f64; 3]| [1.0, p[2], p[1], p[1] * p[2]];
        let mut m = [[0.0; 4]; 4];
        for &i in &ids {
            let f = basis(&points[i]);
            for r in 0..4 {
                for c in 0..4 {
                    m[r][c] += f[r] * f[c];
                }
            }
        }
        let inv = invert4(m);
        for (a, node) in nodes.iter().enumerate() {
            let along = 0.5 * (1.0 + node[0] * xi.signum() / g);
            let fnode = basis(node);
            for &i in &ids {
                let fi = basis(&points[i]);
                let mut w = 0.0;
                for r in 0..4 {
                    for c in 0..4 {
                        w += fnode[r] * inv[r][c] * fi[c];
                    }
                }
                out[a * points.len() + i] = along * w;
            }
        }
    }
    out
}

fn invert4(m: [[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut a = m;
    let mut inv = [[0.0; 4]; 4];
    for (i, row) in inv.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for c in 0..4 {
        let p = (c..4)
            .max_by(|&x, &y| a[x][c].abs().total_cmp(&a[y][c].abs()))
            .unwrap_or(c);
        a.swap(c, p);
        inv.swap(c, p);
        let d = a[c][c];
        for k in 0..4 {
            a[c][k] /= d;
            inv[c][k] /= d;
        }
        for r in 0..4 {
            if r != c {
                let f = a[r][c];
                let (ac, ic) = (a[c], inv[c]);
                for k in 0..4 {
                    a[r][k] -= f * ac[k];
                    inv[r][k] -= f * ic[k];
                }
            }
        }
    }
    inv
}
