//! Shells and membranes: normals at the nodes, the three expanded nodes along the normal
//! and the solid element on them (CalculiX manual, "Eight-node shell element": S8 becomes
//! C3D20, S8R C3D20R, S4 C3D8I, S4R C3D8R, S3 C3D6, S6 C3D15; membranes like the shells of
//! the same shape).

use plxs_core::{ElementType, Error, Result};

use super::{add, cross, normalize, scale};
use crate::shape::face_shape;

/// The solid an expanded shell or membrane becomes.
pub(super) fn solid_type(kind: ElementType) -> ElementType {
    use ElementType::*;
    match kind {
        S8 => C3D20,
        S8R => C3D20R,
        S4 | M3D4 => C3D8I,
        S4R | M3D4R => C3D8R,
        S3 | M3D3 => C3D6,
        S6 | M3D6 => C3D15,
        _ => unreachable!("not a shell: {kind}"),
    }
}

/// The solid's nodes as (local shell node, slot) with slot 0 bottom, 1 middle, 2 top:
/// corners bottom, corners top, midside nodes bottom, midside nodes top, then the middle
/// nodes of the corners (CalculiX's order of the 20- and 15-node solids).
pub(super) fn layout(kind: ElementType) -> Vec<(usize, usize)> {
    use ElementType::*;
    let (corners, quadratic) = shape_of(kind);
    let mut out: Vec<(usize, usize)> = Vec::new();
    out.extend((0..corners).map(|c| (c, 0)));
    out.extend((0..corners).map(|c| (c, 2)));
    if quadratic {
        out.extend((corners..2 * corners).map(|m| (m, 0)));
        out.extend((corners..2 * corners).map(|m| (m, 2)));
        out.extend((0..corners).map(|c| (c, 1)));
    }
    let _ = S8;
    out
}

/// Corner count and whether the element has midside nodes.
pub(super) fn shape_of(kind: ElementType) -> (usize, bool) {
    use ElementType::*;
    match kind {
        S8 | S8R => (4, true),
        S4 | S4R | M3D4 | M3D4R => (4, false),
        S3 | M3D3 => (3, false),
        S6 | M3D6 => (3, true),
        _ => unreachable!("not a shell: {kind}"),
    }
}

/// Natural coordinates of the nodes of a face element.
fn node_coordinates(corners: usize, quadratic: bool) -> Vec<[f64; 2]> {
    let mut p: Vec<[f64; 2]> = if corners == 4 {
        vec![[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]
    } else {
        vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]
    };
    if quadratic {
        for i in 0..corners {
            let (a, b) = (p[i], p[(i + 1) % corners]);
            p.push([0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])]);
        }
    }
    p
}

/// The unit normal of the element at each of its nodes: the cross product of the
/// derivatives of the position with respect to the two natural coordinates there.
pub(super) fn normals(kind: ElementType, coords: &[[f64; 3]], id: u32) -> Result<Vec<[f64; 3]>> {
    let (corners, quadratic) = shape_of(kind);
    let count = if quadratic { 2 * corners } else { corners };
    let mut n = vec![0.0; count];
    let mut dn = vec![[0.0; 2]; count];
    let mut out = Vec::with_capacity(count);
    for p in node_coordinates(corners, quadratic) {
        face_shape(corners, quadratic, p, &mut n, &mut dn);
        let (mut a, mut b) = ([0.0; 3], [0.0; 3]);
        for (k, x) in coords.iter().enumerate().take(count) {
            a = add(a, scale(*x, dn[k][0]));
            b = add(b, scale(*x, dn[k][1]));
        }
        let normal = cross(a, b);
        if super::dot(normal, normal) <= 0.0 {
            return Err(Error::input(
                "*ELEMENT",
                format!("element {id} is degenerate: no normal can be determined"),
            ));
        }
        out.push(normalize(normal));
    }
    Ok(out)
}

/// Bottom, middle and top position of a shell node: the reference surface lies `offset`
/// thicknesses above the middle surface.
pub(super) fn positions(
    x: [f64; 3],
    normal: [f64; 3],
    thickness: f64,
    offset: f64,
) -> Vec<[f64; 3]> {
    let mid = add(x, scale(normal, -offset * thickness));
    vec![
        add(mid, scale(normal, -0.5 * thickness)),
        mid,
        add(mid, scale(normal, 0.5 * thickness)),
    ]
}
