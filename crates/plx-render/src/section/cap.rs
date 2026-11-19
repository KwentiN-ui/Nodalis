//! Section caps of closed surfaces: where a plane cuts the triangulated faces of a CAD solid,
//! the cut lines close into loops, and the area they enclose is filled.
//!
//! The faces of a CAD solid are triangulated each on its own, with nodes of their own along
//! the shared edges at the same positions; the cut points of neighbouring faces are joined
//! by position.

use std::collections::HashMap;

use glam::{DVec2, DVec3};

use super::{SectionCells, corner_count};
use crate::contour::NO_VALUE;
use crate::fe::FEATURE_EDGE_OPACITY;
use crate::mesh::{RenderMesh, Vertex};
use plx_mesh::ElementFamily;

/// Points closer than this share of the cut's size are one point.
const SNAP: f64 = 1e-7;
/// Open ends of the cut lines closer than this share of the cut's size are joined, e.g.
/// across a slightly differing sampling of an edge by its two faces.
const GAP: f64 = 1e-3;

/// The cap where a plane cuts the closed surface of a solid given by its surface cells: the
/// filled area inside the cut lines, facing the removed side, outlined by them.
///
/// `point` and `normal` are in model coordinates; positions are shifted by `-origin` like
/// [`super::section_mesh`].
pub fn cap_mesh(
    cells: &SectionCells,
    coords: &[[f64; 3]],
    origin: DVec3,
    point: DVec3,
    normal: DVec3,
    color: [f32; 3],
) -> RenderMesh {
    let mut render = RenderMesh::default();
    let normal = normal.normalize_or_zero();
    if normal == DVec3::ZERO {
        return render;
    }
    let segments = cut_segments(cells, coords, point, normal);
    if segments.is_empty() {
        return render;
    }
    let loops = loops(&segments);
    if loops.is_empty() {
        return render;
    }
    // In-plane axes: the loops are polygons in (u, v).
    let u = normal.any_orthonormal_vector();
    let v = normal.cross(u);
    let flat: Vec<Vec<DVec2>> = (loops.iter())
        .map(|l| l.iter().map(|p| DVec2::new(p.dot(u), p.dot(v))).collect())
        .collect();
    let face_normal = (-normal).as_vec3().to_array();
    let vertex = |p: DVec3, scalar: f32| Vertex {
        position: (p - origin).as_vec3().to_array(),
        normal: face_normal,
        color,
        scalar,
    };
    let mut earcut = earcut::Earcut::new();
    let mut triangles: Vec<u32> = Vec::new();
    for (outer, holes) in nesting(&flat) {
        let rings: Vec<usize> = std::iter::once(outer).chain(holes).collect();
        let mut hole_starts = Vec::with_capacity(rings.len() - 1);
        let mut data = Vec::new();
        for (k, &ring) in rings.iter().enumerate() {
            if k > 0 {
                hole_starts.push(data.len() as u32);
            }
            data.extend(flat[ring].iter().map(|p| [p.x, p.y]));
        }
        earcut.earcut(data.iter().copied(), &hole_starts, &mut triangles);
        let base = render.vertices.len() as u32;
        let points = rings.iter().flat_map(|&ring| loops[ring].iter().copied());
        render.vertices.extend(points.map(|p| vertex(p, NO_VALUE)));
        render.triangles.extend(triangles.iter().map(|&t| base + t));
    }
    // The cut lines outline the cap like the outline of a solid's section.
    for ring in &loops {
        for (k, &a) in ring.iter().enumerate() {
            let b = ring[(k + 1) % ring.len()];
            render.feature_edges.extend([
                Vertex {
                    color: [0.0; 3],
                    normal: [0.0; 3],
                    ..vertex(a, FEATURE_EDGE_OPACITY)
                },
                Vertex {
                    color: [0.0; 3],
                    normal: [0.0; 3],
                    ..vertex(b, FEATURE_EDGE_OPACITY)
                },
            ]);
        }
    }
    render
}

/// The cut lines of the surface cells, each from one cell.
fn cut_segments(
    cells: &SectionCells,
    coords: &[[f64; 3]],
    point: DVec3,
    normal: DVec3,
) -> Vec<[DVec3; 2]> {
    let mut segments = Vec::new();
    for cell in &cells.cells {
        if cell.shape.family() != ElementFamily::Surface {
            continue;
        }
        let corners = &cell.corners[..corner_count(cell.shape)];
        let position = |k: usize| DVec3::from(coords[corners[k] as usize]);
        let distance = |k: usize| normal.dot(position(k) - point);
        let n = corners.len();
        let mut cut = Vec::with_capacity(2);
        for k in 0..n {
            let (a, b) = (k, (k + 1) % n);
            let (pa, pb, da, db) = (position(a), position(b), distance(a), distance(b));
            if (da < 0.0) == (db < 0.0) {
                continue;
            }
            // The same edge of the neighbouring face, at the same positions, is cut from its
            // lower end too, so both faces find the same point.
            let (p, q, dp, dq) = if pa.to_array() < pb.to_array() {
                (pa, pb, da, db)
            } else {
                (pb, pa, db, da)
            };
            cut.push(p + (q - p) * (dp / (dp - dq)));
        }
        if let [a, b] = cut[..] {
            segments.push([a, b]);
        }
    }
    segments
}

/// The closed loops the cut lines form, as their points in order. Points of neighbouring
/// lines closer than [`SNAP`] of the cut's size are one; open ends are joined across gaps
/// up to [`GAP`] of it, ends left open are dropped with their lines.
fn loops(segments: &[[DVec3; 2]]) -> Vec<Vec<DVec3>> {
    let (mut min, mut max) = (DVec3::splat(f64::INFINITY), DVec3::splat(f64::NEG_INFINITY));
    for p in segments.iter().flatten() {
        min = min.min(*p);
        max = max.max(*p);
    }
    let size = (max - min).length().max(f64::MIN_POSITIVE);
    let snap = size * SNAP;

    // Points merged on a grid of the snap distance, looked up in the neighbouring cells too.
    let mut points: Vec<DVec3> = Vec::new();
    let mut grid: HashMap<[i64; 3], Vec<usize>> = HashMap::new();
    let mut index = |p: DVec3| -> usize {
        let cell = ((p - min) / snap).floor().as_i64vec3().to_array();
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let key = [cell[0] + dx, cell[1] + dy, cell[2] + dz];
                    if let Some(found) = grid.get(&key)
                        && let Some(&i) = found.iter().find(|&&i| points[i].distance(p) <= snap)
                    {
                        return i;
                    }
                }
            }
        }
        points.push(p);
        grid.entry(cell).or_default().push(points.len() - 1);
        points.len() - 1
    };
    let mut edges: Vec<[usize; 2]> = Vec::with_capacity(segments.len());
    for &[a, b] in segments {
        let (a, b) = (index(a), index(b));
        if a != b {
            edges.push([a, b]);
        }
    }
    let mut at: Vec<Vec<usize>> = vec![Vec::new(); points.len()];
    for (e, &[a, b]) in edges.iter().enumerate() {
        at[a].push(e);
        at[b].push(e);
    }

    // Chains of points along unused lines, closed or open.
    let mut used = vec![false; edges.len()];
    let next = |from: usize, used: &mut [bool]| -> Option<usize> {
        let &e = at[from].iter().find(|&&e| !used[e])?;
        used[e] = true;
        let [a, b] = edges[e];
        Some(if a == from { b } else { a })
    };
    let mut closed: Vec<Vec<usize>> = Vec::new();
    let mut open: Vec<Vec<usize>> = Vec::new();
    for e in 0..edges.len() {
        if used[e] {
            continue;
        }
        used[e] = true;
        let [start, second] = edges[e];
        let mut chain = vec![start, second];
        let mut end = second;
        while end != start {
            match next(end, &mut used) {
                Some(p) => {
                    end = p;
                    chain.push(p);
                }
                None => break,
            }
        }
        if end == start {
            chain.pop();
            closed.push(chain);
            continue;
        }
        // Open: grow backwards from the start as well.
        let mut back = Vec::new();
        let mut begin = start;
        while let Some(p) = next(begin, &mut used) {
            begin = p;
            back.push(p);
        }
        back.reverse();
        back.extend(chain);
        open.push(back);
    }
    closed.extend(join_open(open, &points, size * GAP));
    closed
        .into_iter()
        .filter(|chain| chain.len() >= 3)
        .map(|chain| chain.into_iter().map(|i| points[i]).collect())
        .collect()
}

/// Joins open chains end to end where their ends are nearest, up to `gap` apart, until
/// they close.
fn join_open(mut open: Vec<Vec<usize>>, points: &[DVec3], gap: f64) -> Vec<Vec<usize>> {
    let mut closed = Vec::new();
    while let Some(mut chain) = open.pop() {
        loop {
            let (first, last) = (points[chain[0]], points[chain[chain.len() - 1]]);
            let own = first.distance(last);
            // The nearest end of another chain to this chain's last point.
            let nearest = (open.iter().enumerate())
                .flat_map(|(i, other)| {
                    [
                        (i, false, points[other[0]].distance(last)),
                        (i, true, points[other[other.len() - 1]].distance(last)),
                    ]
                })
                .min_by(|a, b| a.2.total_cmp(&b.2));
            match nearest {
                Some((i, reversed, distance)) if distance < own && distance <= gap => {
                    let mut other = open.swap_remove(i);
                    if reversed {
                        other.reverse();
                    }
                    chain.extend(other);
                }
                _ if own <= gap => {
                    closed.push(chain);
                    break;
                }
                _ => break,
            }
        }
    }
    closed
}

/// The loops grouped into polygons with holes, as (outer loop, holes) by index: a loop
/// inside an even number of others bounds material on its inside, one inside an odd number
/// is a hole of the loop it lies directly in.
fn nesting(loops: &[Vec<DVec2>]) -> Vec<(usize, Vec<usize>)> {
    let bounds: Vec<(DVec2, DVec2)> = (loops.iter())
        .map(|l| {
            l.iter().fold(
                (DVec2::splat(f64::INFINITY), DVec2::splat(f64::NEG_INFINITY)),
                |(lo, hi), p| (lo.min(*p), hi.max(*p)),
            )
        })
        .collect();
    let area = |l: &[DVec2]| {
        (0..l.len())
            .map(|k| l[k].perp_dot(l[(k + 1) % l.len()]))
            .sum::<f64>()
            .abs()
            * 0.5
    };
    let areas: Vec<f64> = loops.iter().map(|l| area(l)).collect();
    // The loops each loop lies in, by its first point.
    let containing: Vec<Vec<usize>> = (0..loops.len())
        .map(|i| {
            let p = loops[i][0];
            (0..loops.len())
                .filter(|&j| {
                    let (lo, hi) = bounds[j];
                    j != i
                        && p.cmpge(lo).all()
                        && p.cmple(hi).all()
                        && areas[j] > areas[i]
                        && inside(p, &loops[j])
                })
                .collect()
        })
        .collect();
    let mut polygons: Vec<(usize, Vec<usize>)> = Vec::new();
    let mut polygon_of = vec![usize::MAX; loops.len()];
    for (i, around) in containing.iter().enumerate() {
        if around.len().is_multiple_of(2) {
            polygon_of[i] = polygons.len();
            polygons.push((i, Vec::new()));
        }
    }
    for (i, around) in containing.iter().enumerate() {
        if !around.len().is_multiple_of(2) {
            // The parent is the smallest loop around it.
            let parent = (around.iter().copied()).min_by(|&a, &b| areas[a].total_cmp(&areas[b]));
            if let Some(parent) = parent
                && polygon_of[parent] != usize::MAX
            {
                polygons[polygon_of[parent]].1.push(i);
            }
        }
    }
    polygons
}

/// Whether a point lies inside a polygon, by the even-odd rule.
fn inside(p: DVec2, polygon: &[DVec2]) -> bool {
    let mut inside = false;
    let mut j = polygon.len() - 1;
    for i in 0..polygon.len() {
        let (a, b) = (polygon[i], polygon[j]);
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_mesh::{Element, ElementShape, FeMesh, Part};

    /// The closed surface of a box with a square hole through it along z, each of its faces
    /// with nodes of its own like the display of a CAD solid.
    fn tube() -> (FeMesh, Part) {
        let mut mesh = FeMesh::default();
        let mut part = Part {
            bodies: Vec::new(),
            name: "TUBE".into(),
            elements: Vec::new(),
        };
        let mut node = 0;
        let mut element = 0;
        let mut quad = |mesh: &mut FeMesh, corners: [[f64; 3]; 4]| {
            let ids: Vec<_> = corners
                .iter()
                .map(|&p| {
                    node += 1;
                    mesh.set_node(node, p);
                    node
                })
                .collect();
            for tri in [[0, 1, 2], [0, 2, 3]] {
                element += 1;
                mesh.add_element(Element {
                    id: element,
                    type_name: "S3".into(),
                    shape: ElementShape::Tri3,
                    nodes: tri.iter().map(|&k| ids[k]).collect(),
                })
                .unwrap();
                part.elements.push(element);
            }
        };
        // Outer and inner walls, square rings in x/y, from z = 0 to 1.
        for (r, _) in [(2.0, 0), (1.0, 1)] {
            let c = [[-r, -r], [r, -r], [r, r], [-r, r]];
            for k in 0..4 {
                let (a, b) = (c[k], c[(k + 1) % 4]);
                quad(
                    &mut mesh,
                    [
                        [a[0], a[1], 0.0],
                        [b[0], b[1], 0.0],
                        [b[0], b[1], 1.0],
                        [a[0], a[1], 1.0],
                    ],
                );
            }
        }
        mesh.parts.push(part.clone());
        (mesh, part)
    }

    #[test]
    fn cap_of_a_tube_fills_the_ring_between_its_walls() {
        let (mesh, part) = tube();
        let cells = SectionCells::new(&mesh, &part);
        let cap = cap_mesh(
            &cells,
            mesh.coords(),
            DVec3::ZERO,
            DVec3::new(0.0, 0.0, 0.5),
            DVec3::Z,
            [1.0; 3],
        );
        let area: f64 = cap
            .triangles
            .chunks(3)
            .map(|t| {
                let p = |k: usize| glam::Vec3::from(cap.vertices[t[k] as usize].position);
                ((p(1) - p(0)).cross(p(2) - p(0)).length() * 0.5) as f64
            })
            .sum();
        assert!((area - 12.0).abs() < 1e-5, "ring of 16 - 4, got {area}");
        assert_eq!(cap.feature_edges.len(), 2 * 2 * 8, "both loops outlined");
        assert!(cap.vertices.iter().all(|v| v.normal == [0.0, 0.0, -1.0]));
    }

    #[test]
    fn plane_beside_the_solid_makes_no_cap() {
        let (mesh, part) = tube();
        let cells = SectionCells::new(&mesh, &part);
        let cap = cap_mesh(
            &cells,
            mesh.coords(),
            DVec3::ZERO,
            DVec3::new(0.0, 0.0, 5.0),
            DVec3::Z,
            [1.0; 3],
        );
        assert!(cap.triangles.is_empty());
    }

    #[test]
    fn open_ends_join_across_small_gaps() {
        let p = |x: f64, y: f64| DVec3::new(x, y, 0.0);
        // A square whose last side misses its start by a hair.
        let segments = [
            [p(0.0, 0.0), p(1.0, 0.0)],
            [p(1.0, 0.0), p(1.0, 1.0)],
            [p(1.0, 1.0), p(0.0, 1.0)],
            [p(0.0, 1.0), p(0.0, 1e-5)],
        ];
        let loops = loops(&segments);
        assert_eq!(loops.len(), 1);
        assert_eq!(
            loops[0].len(),
            5,
            "the end near the start stays a point of its own"
        );
    }
}
