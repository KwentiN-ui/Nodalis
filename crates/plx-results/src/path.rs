//! Results along a straight path through the model, e.g. across a wall.
//!
//! Each read-out point is located in the element that contains it and nodal values are
//! interpolated there. Elements are split into tetrahedra (solids) or triangles (shells and
//! 2D models) and the point's barycentric coordinates in the one containing it are the
//! weights. Quadratic tetrahedra and triangles are split at their midside nodes, so their
//! midside values count; other quadratic elements use their corner nodes. The weights only
//! depend on the undeformed mesh and are computed once for all increments.

use plx_mesh::{ElementShape, FeMesh};

/// A read-out point of a path.
#[derive(Clone, Debug, PartialEq)]
pub struct PathPoint {
    /// Distance from the start of the path.
    pub distance: f64,
    pub position: [f64; 3],
    /// Interpolation weights of nodal values, by index into [`FeMesh::coords`]; `None`
    /// outside the mesh.
    pub weights: Option<Vec<(usize, f64)>>,
}

/// Relative tolerance of the barycentric coordinates: points on an element's boundary
/// belong to it.
const INSIDE: f64 = 1e-7;
/// Largest distance of a point from a shell or 2D element, relative to the element's size.
const OFF_PLANE: f64 = 1e-3;

/// Locates the read-out points `(distance, position)` in the elements of the mesh.
pub fn path_points(mesh: &FeMesh, samples: &[(f64, [f64; 3])]) -> Vec<PathPoint> {
    let coords = mesh.coords();
    let mut points: Vec<PathPoint> = samples
        .iter()
        .map(|&(distance, position)| PathPoint {
            distance,
            position,
            weights: None,
        })
        .collect();
    let (Some(first), Some(last)) = (samples.first(), samples.last()) else {
        return points;
    };
    let (start, end) = (first.1, last.1);
    let direction = sub(end, start);
    let length2 = dot(direction, direction);
    // The parameter of every sample along the path, to test only the samples that can lie in
    // an element's bounding box.
    let parameters: Vec<f64> = samples
        .iter()
        .map(|(_, p)| {
            if length2 > 0.0 {
                dot(sub(*p, start), direction) / length2
            } else {
                0.0
            }
        })
        .collect();
    let mut solid_found = vec![false; samples.len()];
    let mut nodes = Vec::with_capacity(20);
    for element in mesh.elements() {
        nodes.clear();
        nodes.extend(element.nodes.iter().map(|&id| mesh.node_index(id)));
        let Some(nodes) = nodes.iter().copied().collect::<Option<Vec<usize>>>() else {
            continue;
        };
        let (low, high) = bounds(coords, &nodes);
        let size = (0..3).map(|k| high[k] - low[k]).fold(0.0, f64::max);
        let margin = size * OFF_PLANE;
        let Some((t0, t1)) = segment_in_box(start, direction, low, high, margin) else {
            continue;
        };
        let range = samples_between(&parameters, t0, t1);
        if range.is_empty() {
            continue;
        }
        if let Some(tets) = tetrahedra(element.shape, &nodes) {
            for i in range {
                if solid_found[i] {
                    continue;
                }
                if let Some(weights) = tets
                    .iter()
                    .find_map(|t| in_tetrahedron(coords, t, points[i].position))
                {
                    points[i].weights = Some(weights);
                    solid_found[i] = true;
                }
            }
        } else if let Some(tris) = triangles(element.shape, &nodes) {
            for i in range {
                // A solid element wins over a shell at the same place.
                if solid_found[i] || points[i].weights.is_some() {
                    continue;
                }
                let position = points[i].position;
                if let Some(weights) = tris.iter().find_map(|t| in_triangle(coords, t, position)) {
                    points[i].weights = Some(weights);
                }
            }
        }
    }
    points
}

/// Interpolated values at the read-out points; `NaN` outside the mesh or where a node has no
/// value.
pub fn interpolate(points: &[PathPoint], values: &[f32]) -> Vec<f64> {
    points
        .iter()
        .map(|p| match &p.weights {
            Some(weights) => weights
                .iter()
                .map(|&(node, w)| values.get(node).map_or(f64::NAN, |&v| v as f64) * w)
                .sum(),
            None => f64::NAN,
        })
        .collect()
}

/// The path as values separated by `separator` with a header, one row per read-out point;
/// values outside the mesh are left empty. A semicolon suits CSV files, a tab spreadsheets.
pub fn to_csv(points: &[PathPoint], values: &[f64], header: &str, separator: char) -> String {
    let s = separator;
    let mut csv = format!("Distance{s}X{s}Y{s}Z{s}{header}\n");
    for (point, value) in points.iter().zip(values) {
        let [x, y, z] = point.position;
        let value = if value.is_finite() {
            value.to_string()
        } else {
            String::new()
        };
        csv.push_str(&format!("{}{s}{x}{s}{y}{s}{z}{s}{value}\n", point.distance));
    }
    csv
}

/// Indices of the samples whose parameter lies in `[t0, t1]`; the parameters increase.
fn samples_between(parameters: &[f64], t0: f64, t1: f64) -> std::ops::Range<usize> {
    let from = parameters.partition_point(|&t| t < t0);
    let to = parameters.partition_point(|&t| t <= t1);
    from..to.max(from)
}

/// Where the segment `start + t * direction`, t in [0, 1], runs through the box widened by
/// `margin`: the range of t, if it does.
fn segment_in_box(
    start: [f64; 3],
    direction: [f64; 3],
    low: [f64; 3],
    high: [f64; 3],
    margin: f64,
) -> Option<(f64, f64)> {
    let (mut t0, mut t1) = (0.0_f64, 1.0_f64);
    for k in 0..3 {
        let (lo, hi) = (low[k] - margin, high[k] + margin);
        if direction[k] == 0.0 {
            if start[k] < lo || start[k] > hi {
                return None;
            }
            continue;
        }
        let (a, b) = (
            (lo - start[k]) / direction[k],
            (hi - start[k]) / direction[k],
        );
        t0 = t0.max(a.min(b));
        t1 = t1.min(a.max(b));
        if t0 > t1 {
            return None;
        }
    }
    Some((t0, t1))
}

fn bounds(coords: &[[f64; 3]], nodes: &[usize]) -> ([f64; 3], [f64; 3]) {
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for &n in nodes {
        for k in 0..3 {
            low[k] = low[k].min(coords[n][k]);
            high[k] = high[k].max(coords[n][k]);
        }
    }
    (low, high)
}

/// A solid element split into tetrahedra, by mesh node indices.
fn tetrahedra(shape: ElementShape, n: &[usize]) -> Option<Vec<[usize; 4]>> {
    let tets = |list: &[[usize; 4]]| list.iter().map(|t| t.map(|i| n[i])).collect();
    Some(match shape {
        ElementShape::Tet4 => vec![[n[0], n[1], n[2], n[3]]],
        // CalculiX's C3D10: midside nodes 4 (0-1), 5 (1-2), 6 (2-0), 7 (0-3), 8 (1-3),
        // 9 (2-3). Four corner tetrahedra and the octahedron split along 6-8.
        ElementShape::Tet10 => tets(&[
            [0, 4, 6, 7],
            [4, 1, 5, 8],
            [6, 5, 2, 9],
            [7, 8, 9, 3],
            [6, 8, 4, 5],
            [6, 8, 5, 9],
            [6, 8, 9, 7],
            [6, 8, 7, 4],
        ]),
        ElementShape::Wedge6 | ElementShape::Wedge15 => {
            tets(&[[0, 1, 2, 3], [1, 2, 3, 4], [2, 3, 4, 5]])
        }
        ElementShape::Hex8 | ElementShape::Hex20 => tets(&[
            [0, 1, 2, 6],
            [0, 2, 3, 6],
            [0, 3, 7, 6],
            [0, 7, 4, 6],
            [0, 4, 5, 6],
            [0, 5, 1, 6],
        ]),
        _ => return None,
    })
}

/// A shell or 2D element split into triangles, by mesh node indices.
fn triangles(shape: ElementShape, n: &[usize]) -> Option<Vec<[usize; 3]>> {
    let tris = |list: &[[usize; 3]]| list.iter().map(|t| t.map(|i| n[i])).collect();
    Some(match shape {
        ElementShape::Tri3 => vec![[n[0], n[1], n[2]]],
        // Midside nodes 3 (0-1), 4 (1-2), 5 (2-0).
        ElementShape::Tri6 => tris(&[[0, 3, 5], [3, 1, 4], [5, 4, 2], [3, 4, 5]]),
        ElementShape::Quad4 | ElementShape::Quad8 => tris(&[[0, 1, 2], [0, 2, 3]]),
        _ => return None,
    })
}

fn in_tetrahedron(coords: &[[f64; 3]], t: &[usize; 4], p: [f64; 3]) -> Option<Vec<(usize, f64)>> {
    let a = coords[t[0]];
    let (e1, e2, e3) = (
        sub(coords[t[1]], a),
        sub(coords[t[2]], a),
        sub(coords[t[3]], a),
    );
    let det = dot(e1, cross(e2, e3));
    if det == 0.0 || !det.is_finite() {
        return None;
    }
    let d = sub(p, a);
    // Cramer's rule for d = l1 e1 + l2 e2 + l3 e3.
    let l1 = dot(d, cross(e2, e3)) / det;
    let l2 = dot(e1, cross(d, e3)) / det;
    let l3 = dot(e1, cross(e2, d)) / det;
    let l0 = 1.0 - l1 - l2 - l3;
    let weights = [l0, l1, l2, l3];
    weights
        .iter()
        .all(|&l| l >= -INSIDE)
        .then(|| (0..4).map(|k| (t[k], weights[k].max(0.0))).collect())
}

fn in_triangle(coords: &[[f64; 3]], t: &[usize; 3], p: [f64; 3]) -> Option<Vec<(usize, f64)>> {
    let a = coords[t[0]];
    let (e1, e2) = (sub(coords[t[1]], a), sub(coords[t[2]], a));
    let normal = cross(e1, e2);
    let area2 = dot(normal, normal);
    if area2 == 0.0 || !area2.is_finite() {
        return None;
    }
    let d = sub(p, a);
    let size = dot(e1, e1).max(dot(e2, e2)).sqrt();
    if dot(d, normal).abs() / area2.sqrt() > OFF_PLANE * size {
        return None;
    }
    let l1 = dot(cross(d, e2), normal) / area2;
    let l2 = dot(cross(e1, d), normal) / area2;
    let l0 = 1.0 - l1 - l2;
    let weights = [l0, l1, l2];
    weights
        .iter()
        .all(|&l| l >= -INSIDE)
        .then(|| (0..3).map(|k| (t[k], weights[k].max(0.0))).collect())
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
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

#[cfg(test)]
mod tests {
    use super::*;
    use plx_mesh::{Element, Part};

    /// `n` unit hexahedra stacked along x.
    fn bar(n: u32) -> FeMesh {
        let mut mesh = FeMesh::default();
        for i in 0..=n {
            for (k, [y, z]) in [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
                .into_iter()
                .enumerate()
            {
                mesh.set_node(i * 4 + k as u32 + 1, [i as f64, y, z]);
            }
        }
        for i in 0..n {
            let a = i * 4 + 1;
            let b = a + 4;
            mesh.add_element(Element {
                id: i + 1,
                type_name: "C3D8".into(),
                shape: ElementShape::Hex8,
                nodes: vec![a, b, b + 1, a + 1, a + 3, b + 3, b + 2, a + 2],
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            bodies: Vec::new(),
            name: "BAR".into(),
            elements: (1..=n).collect(),
        });
        mesh
    }

    fn line(from: [f64; 3], to: [f64; 3], n: usize) -> Vec<(f64, [f64; 3])> {
        let length = dot(sub(to, from), sub(to, from)).sqrt();
        (0..n)
            .map(|i| {
                let t = i as f64 / (n - 1) as f64;
                (
                    t * length,
                    [0, 1, 2].map(|k| from[k] + (to[k] - from[k]) * t),
                )
            })
            .collect()
    }

    #[test]
    fn linear_fields_are_reproduced_along_the_path() {
        let mesh = bar(3);
        // Path from outside the bar through all three elements and out again.
        let samples = line([-0.5, 0.3, 0.6], [3.5, 0.7, 0.2], 41);
        let points = path_points(&mesh, &samples);
        let field: Vec<f32> = (mesh.coords().iter())
            .map(|p| (2.0 * p[0] + p[1] - 3.0 * p[2]) as f32)
            .collect();
        let values = interpolate(&points, &field);
        for ((point, value), (_, p)) in points.iter().zip(&values).zip(&samples) {
            if (0.0..=3.0).contains(&p[0]) {
                let expected = 2.0 * p[0] + p[1] - 3.0 * p[2];
                assert!(
                    (value - expected).abs() < 1e-5,
                    "{p:?}: {value} != {expected}"
                );
            } else {
                assert!(point.weights.is_none() && value.is_nan(), "{p:?}");
            }
        }
    }

    #[test]
    fn quadratic_tetrahedra_use_their_midside_nodes() {
        let mut mesh = FeMesh::default();
        let corners = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let mids = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)];
        let mut all: Vec<[f64; 3]> = corners.to_vec();
        for (a, b) in mids {
            all.push([0, 1, 2].map(|k| (corners[a][k] + corners[b][k]) / 2.0));
        }
        for (i, p) in all.iter().enumerate() {
            mesh.set_node(i as u32 + 1, *p);
        }
        mesh.add_element(Element {
            id: 1,
            type_name: "C3D10".into(),
            shape: ElementShape::Tet10,
            nodes: (1..=10).collect(),
        })
        .unwrap();
        // A value only at the midside node between corners 0 and 1.
        let mut field = vec![0.0_f32; 10];
        field[4] = 1.0;
        let points = path_points(&mesh, &line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 5));
        let values = interpolate(&points, &field);
        let expected = [0.0, 0.5, 1.0, 0.5, 0.0];
        for (v, e) in values.iter().zip(expected) {
            assert!((v - e).abs() < 1e-9, "{values:?}");
        }
        // Every point inside the octahedron is found as well.
        let inner = path_points(&mesh, &line([0.1, 0.1, 0.1], [0.3, 0.3, 0.3], 20));
        assert!(inner.iter().all(|p| p.weights.is_some()));
    }

    #[test]
    fn paths_in_flat_models_lie_in_their_plane() {
        let mut mesh = FeMesh::default();
        for (i, p) in [[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]]
            .iter()
            .enumerate()
        {
            mesh.set_node(i as u32 + 1, [p[0], p[1], 0.0]);
        }
        mesh.add_element(Element {
            id: 1,
            type_name: "CPS4".into(),
            shape: ElementShape::Quad4,
            nodes: vec![1, 2, 3, 4],
        })
        .unwrap();
        let field = [0.0, 2.0, 2.0, 0.0];
        let points = path_points(&mesh, &line([0.0, 0.5, 0.0], [2.0, 0.5, 0.0], 3));
        assert_eq!(interpolate(&points, &field), vec![0.0, 1.0, 2.0]);
        // Beside the plane nothing is found.
        let off = path_points(&mesh, &line([0.0, 0.5, 0.5], [2.0, 0.5, 0.5], 3));
        assert!(off.iter().all(|p| p.weights.is_none()));
    }

    #[test]
    fn csv_leaves_points_outside_empty() {
        let points = vec![
            PathPoint {
                distance: 0.0,
                position: [0.0; 3],
                weights: None,
            },
            PathPoint {
                distance: 1.5,
                position: [1.5, 0.0, 0.0],
                weights: Some(vec![(0, 1.0)]),
            },
        ];
        let csv = to_csv(&points, &[f64::NAN, 4.25], "STRESS MISES", ';');
        assert_eq!(
            csv,
            "Distance;X;Y;Z;STRESS MISES\n0;0;0;0;\n1.5;1.5;0;0;4.25\n"
        );
    }
}
