//! Infill sections at export: every solid element of the region gets the share of its
//! depth below the surface of the region that lies in the walls (perimeters on the sides,
//! solid layers at top and bottom), and a material that mixes the printed material and the
//! homogenized core by that share. The shares are rounded to [`LEVELS`] steps, so that a
//! section writes at most `LEVELS + 1` materials. Faces on a symmetry plane are no surface:
//! the part goes on beyond them.

use std::collections::{BTreeMap, HashMap, HashSet};

use plx_mesh::{ElementFamily, ElementId, FeMesh, NodeId};
use plx_model::InfillSection;

/// Steps of the wall share between the core (0) and the wall (`LEVELS`).
pub const LEVELS: u32 = 8;

/// The elements of an infill region by their wall share in steps of `1 / LEVELS`. Faces
/// whose corners are all `inside` (on a symmetry plane) are not printed surface.
pub fn wall_levels(
    mesh: &FeMesh,
    elements: &[ElementId],
    infill: &InfillSection,
    build: [f64; 3],
    inside: &HashSet<NodeId>,
) -> BTreeMap<u32, Vec<ElementId>> {
    let solids: Vec<&plx_mesh::Element> = (elements.iter())
        .filter_map(|&id| mesh.element(id))
        .filter(|e| e.shape.family() == ElementFamily::Solid)
        .collect();
    // The faces of the region that no second element of it shares are its surface.
    let mut faces: HashMap<Vec<NodeId>, (usize, Vec<NodeId>)> = HashMap::new();
    for element in &solids {
        for face in element.faces() {
            let corners: Vec<NodeId> = face.corners.iter().map(|&k| element.nodes[k]).collect();
            let mut key = corners.clone();
            key.sort_unstable();
            faces.entry(key).or_insert((0, corners)).0 += 1;
        }
    }
    let point = |id: NodeId| mesh.node(id).unwrap_or_default();
    // Triangles of the surface: on the sides (perimeters) or facing up or down (top and
    // bottom layers), by the angle of their normal to the build direction.
    let mut sides = Vec::new();
    let mut tops = Vec::new();
    for (count, corners) in faces.values() {
        if *count != 1 || corners.len() < 3 || corners.iter().all(|n| inside.contains(n)) {
            continue;
        }
        let p: Vec<[f64; 3]> = corners.iter().map(|&n| point(n)).collect();
        for k in 1..p.len() - 1 {
            let tri = [p[0], p[k], p[k + 1]];
            let n = cross(sub(tri[1], tri[0]), sub(tri[2], tri[0]));
            let len = norm(n);
            if len == 0.0 {
                continue;
            }
            if dot(n, build).abs() / len >= std::f64::consts::FRAC_1_SQRT_2 {
                tops.push(tri);
            } else {
                sides.push(tri);
            }
        }
    }
    // Depths beyond the reach matter only for elements wholly in the core: the distance
    // changes by at most an element's size across it.
    let longest = (solids.iter())
        .map(|e| {
            let p: Vec<[f64; 3]> = e.nodes.iter().map(|&n| point(n)).collect();
            let mut longest = 0.0_f64;
            for a in &p {
                for b in &p {
                    longest = longest.max(norm(sub(*a, *b)));
                }
            }
            longest
        })
        .fold(0.0, f64::max);
    let side_grid = Grid::new(&sides, infill.wall_thickness + longest);
    let top_grid = Grid::new(&tops, infill.top_bottom_thickness + longest);
    let mut side_depth: HashMap<NodeId, f64> = HashMap::new();
    let mut top_depth: HashMap<NodeId, f64> = HashMap::new();
    let mut levels: BTreeMap<u32, Vec<ElementId>> = BTreeMap::new();
    for element in &solids {
        let depths = |grid: &Grid, cache: &mut HashMap<NodeId, f64>| -> Vec<f64> {
            (element.nodes.iter())
                .map(|&n| *cache.entry(n).or_insert_with(|| grid.distance(point(n))))
                .collect()
        };
        let side = plx_model::infill::wall_share(
            depths(&side_grid, &mut side_depth),
            infill.wall_thickness,
        );
        let top = plx_model::infill::wall_share(
            depths(&top_grid, &mut top_depth),
            infill.top_bottom_thickness,
        );
        let level = (side.max(top) * LEVELS as f64).round() as u32;
        levels.entry(level).or_default().push(element.id);
    }
    levels
}

/// Triangles in cubes of the size of the reach: the ones within the reach of a point are
/// in its cube and the 26 around it.
struct Grid<'a> {
    triangles: &'a [[[f64; 3]; 3]],
    cell: f64,
    cells: HashMap<[i64; 3], Vec<u32>>,
}

impl<'a> Grid<'a> {
    fn new(triangles: &'a [[[f64; 3]; 3]], reach: f64) -> Self {
        let cell = reach.max(1e-12);
        let mut cells: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
        let index = |v: f64| (v / cell).floor() as i64;
        for (t, tri) in triangles.iter().enumerate() {
            let lo: [i64; 3] =
                std::array::from_fn(|k| index(tri[0][k].min(tri[1][k]).min(tri[2][k])));
            let hi: [i64; 3] =
                std::array::from_fn(|k| index(tri[0][k].max(tri[1][k]).max(tri[2][k])));
            for x in lo[0]..=hi[0] {
                for y in lo[1]..=hi[1] {
                    for z in lo[2]..=hi[2] {
                        cells.entry([x, y, z]).or_default().push(t as u32);
                    }
                }
            }
        }
        Self {
            triangles,
            cell,
            cells,
        }
    }

    /// The distance to the nearest triangle, at most the reach.
    fn distance(&self, p: [f64; 3]) -> f64 {
        let c: [i64; 3] = std::array::from_fn(|k| (p[k] / self.cell).floor() as i64);
        let mut best = self.cell;
        for x in -1..=1 {
            for y in -1..=1 {
                for z in -1..=1 {
                    let Some(list) = self.cells.get(&[c[0] + x, c[1] + y, c[2] + z]) else {
                        continue;
                    };
                    for &t in list {
                        best = best.min(point_triangle(p, &self.triangles[t as usize]));
                    }
                }
            }
        }
        best
    }
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

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// The distance of a point to a triangle (Ericson, *Real-Time Collision Detection*, 5.1.5).
fn point_triangle(p: [f64; 3], [a, b, c]: &[[f64; 3]; 3]) -> f64 {
    let (a, b, c) = (*a, *b, *c);
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    let at = |v: [f64; 3]| norm(sub(p, v));
    let along =
        |o: [f64; 3], d: [f64; 3], t: f64| [o[0] + t * d[0], o[1] + t * d[1], o[2] + t * d[2]];
    if d1 <= 0.0 && d2 <= 0.0 {
        return at(a);
    }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return at(b);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return at(along(a, ab, d1 / (d1 - d3)));
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return at(c);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return at(along(a, ac, d2 / (d2 - d6)));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return at(along(b, sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))));
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    at([
        a[0] + ab[0] * v + ac[0] * w,
        a[1] + ab[1] * v + ac[1] * w,
        a[2] + ab[2] * v + ac[2] * w,
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn faces_on_a_symmetry_plane_are_no_walls() {
        use plx_mesh::{Element, ElementShape};
        // Unit cubes, 3 x 3 x 2; the middle one at the bottom touches the surface only below.
        let mut mesh = FeMesh::default();
        let id = |x: u32, y: u32, z: u32| 1 + x + 4 * y + 16 * z;
        for z in 0..3 {
            for y in 0..4 {
                for x in 0..4 {
                    mesh.set_node(id(x, y, z), [x as f64, y as f64, z as f64]);
                }
            }
        }
        let mut elements = Vec::new();
        for z in 0..2 {
            for y in 0..3 {
                for x in 0..3 {
                    let e = 1 + x + 3 * y + 9 * z;
                    let nodes: [(u32, u32); 4] = [(0, 0), (1, 0), (1, 1), (0, 1)];
                    let nodes: Vec<NodeId> = (0..2)
                        .flat_map(|dz| nodes.map(|(dx, dy)| id(x + dx, y + dy, z + dz)))
                        .collect();
                    (mesh.add_element(Element {
                        id: e,
                        type_name: "C3D8".into(),
                        shape: ElementShape::Hex8,
                        nodes,
                    }))
                    .unwrap();
                    elements.push(e);
                }
            }
        }
        let infill = InfillSection {
            wall_thickness: 0.5,
            top_bottom_thickness: 0.5,
            ..InfillSection::DEFAULT
        };
        let level_of = |inside: &HashSet<NodeId>, element: ElementId| {
            let levels = wall_levels(&mesh, &elements, &infill, [0.0, 0.0, 1.0], inside);
            (levels.into_iter())
                .find(|(_, list)| list.contains(&element))
                .map(|(level, _)| level)
                .unwrap()
        };
        let bottom: HashSet<NodeId> = (0..4)
            .flat_map(|y| (0..4).map(move |x| id(x, y, 0)))
            .collect();
        assert!(level_of(&HashSet::new(), 5) > 0);
        assert_eq!(level_of(&bottom, 5), 0);
    }

    #[test]
    fn point_triangle_distances() {
        let tri = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        assert!((point_triangle([0.2, 0.2, 2.0], &tri) - 2.0).abs() < 1e-12);
        assert!((point_triangle([-1.0, 0.0, 0.0], &tri) - 1.0).abs() < 1e-12);
        assert!((point_triangle([1.0, 1.0, 0.0], &tri) - 0.5_f64.sqrt()).abs() < 1e-12);
    }
}
