//! Hot spot stress (Strukturspannung) by surface stress extrapolation.
//!
//! For each toe node of a [`HotSpot`] definition a straight path runs along the plate surface
//! away from the weld. The nodal stresses of the results are interpolated at the read-out
//! points on it, the chosen stress is computed there and extrapolated back to the toe, as in
//! the IIW recommendations. Read-out points are placed on the surface triangles of the mesh,
//! so their weights only depend on the mesh and are computed once for all increments.

mod definition;
mod grid;

pub use definition::{
    Extrapolation, HotSpot, HotSpotComponent, extrapolated, extrapolation_weights,
};

use plx_mesh::{FeMesh, NodeId, SkinFace};

use crate::{Increment, principal_values};
use grid::Grid;

/// Components of the `STRESS` field in the order xx, yy, zz, xy, yz, zx.
const STRESS_COMPONENTS: [&str; 6] = ["S11", "S22", "S33", "S12", "S23", "S13"];

/// Read-out points further than this fraction of their distance from the straight path are
/// reported: the path left the plate surface, e.g. because it points into the weld.
const GAP_WARNING: f64 = 0.1;

/// A point on a path where the stress is read.
#[derive(Clone, Debug, PartialEq)]
pub struct ReadoutPoint {
    /// Distance from the toe along the path.
    pub distance: f64,
    /// Position on the surface.
    pub position: [f64; 3],
    /// Unit direction of the path at the point, tangent to the surface.
    pub direction: [f64; 3],
    /// Interpolation weights of nodal values, by index into [`FeMesh::coords`].
    pub weights: Vec<(usize, f64)>,
}

/// The path from one toe node.
#[derive(Clone, Debug, PartialEq)]
pub struct HotSpotPath {
    pub node: NodeId,
    /// Index of the toe node in [`FeMesh::coords`].
    pub index: usize,
    pub position: [f64; 3],
    /// Unit direction of the straight path, along the surface away from the weld.
    pub direction: [f64; 3],
    pub points: Vec<ReadoutPoint>,
}

/// Stress read on one path and its hot spot value.
#[derive(Clone, Debug, PartialEq)]
pub struct HotSpotValue {
    pub node: NodeId,
    /// The stress at each read-out point.
    pub readings: Vec<f64>,
    /// The extrapolated stress at the toe.
    pub hot_spot: f64,
}

/// Values of all paths of a definition in one increment.
#[derive(Clone, Debug, PartialEq)]
pub struct IncrementValues {
    pub step: u32,
    pub increment: u32,
    /// Time, eigenfrequency or buckling factor.
    pub value: f64,
    pub values: Vec<HotSpotValue>,
}

impl IncrementValues {
    /// The path with the largest hot spot stress.
    pub fn maximum(&self) -> Option<&HotSpotValue> {
        self.values
            .iter()
            .filter(|v| v.hot_spot.is_finite())
            .max_by(|a, b| a.hot_spot.total_cmp(&b.hot_spot))
    }
}

/// Evaluation of one definition over all increments of a results file.
#[derive(Clone, Debug, PartialEq)]
pub struct HotSpotReport {
    pub name: String,
    pub component: HotSpotComponent,
    pub method: String,
    pub distances: Vec<f64>,
    pub paths: Vec<HotSpotPath>,
    /// Increments with stresses, in file order.
    pub increments: Vec<IncrementValues>,
    pub warnings: Vec<String>,
}

/// A surface triangle; its corners are nodes or the centre of a face, each a weighted sum
/// of nodes.
struct Triangle {
    corners: [[f64; 3]; 3],
    weights: [Vec<(usize, f64)>; 3],
    normal: [f64; 3],
}

/// The surface triangles near some points, to place read-out points on.
struct Surface {
    triangles: Vec<Triangle>,
    grid: Grid,
}

impl Surface {
    /// The triangles of `faces` within `margin` of the box around `points`.
    fn around<'a>(
        coords: &[[f64; 3]],
        faces: impl IntoIterator<Item = &'a SkinFace>,
        points: &[[f64; 3]],
        margin: f64,
    ) -> Self {
        let (mut low, mut high) = bounds(points);
        for k in 0..3 {
            low[k] -= margin;
            high[k] += margin;
        }
        let mut triangles = Vec::new();
        for face in faces {
            let nodes = face.corners.iter().chain(&face.mids);
            let outside = (0..3).any(|k| {
                nodes.clone().all(|&n| coords[n][k] < low[k])
                    || nodes.clone().all(|&n| coords[n][k] > high[k])
            });
            if !outside {
                triangulate(coords, face, &mut triangles);
            }
        }
        let grid = Grid::new(
            &triangles
                .iter()
                .map(|t| bounds(&t.corners))
                .collect::<Vec<_>>(),
        );
        Self { triangles, grid }
    }

    fn closest(&self, point: [f64; 3]) -> Option<(&Triangle, [f64; 3])> {
        closest(&self.triangles, &self.grid, point)
    }

    /// The read-out point on the surface nearest to `distance` along `along` from `start`,
    /// with its distance from the straight path relative to `distance`.
    fn place(
        &self,
        start: [f64; 3],
        along: [f64; 3],
        distance: f64,
    ) -> Option<(ReadoutPoint, f64)> {
        let target = add(start, scale(along, distance));
        let (triangle, [u, v, w]) = self.closest(target)?;
        let [a, b, c] = triangle.corners;
        let position = add(add(scale(a, u), scale(b, v)), scale(c, w));
        let gap = length(sub(position, target)) / distance.max(1e-300);
        let mut weights: Vec<(usize, f64)> = Vec::new();
        for (vertex, factor) in triangle.weights.iter().zip([u, v, w]) {
            for &(n, weight) in vertex {
                match weights.iter_mut().find(|(m, _)| *m == n) {
                    Some((_, total)) => *total += factor * weight,
                    None => weights.push((n, factor * weight)),
                }
            }
        }
        weights.retain(|(_, w)| w.abs() > 1e-12);
        let point = ReadoutPoint {
            distance,
            position,
            direction: in_plane(along, triangle.normal).unwrap_or(along),
            weights,
        };
        Some((point, gap))
    }
}

/// The paths of a definition on the surface given by `faces`, with the warnings found while
/// placing them.
pub fn hot_spot_paths<'a>(
    mesh: &FeMesh,
    faces: impl IntoIterator<Item = &'a SkinFace>,
    hot_spot: &HotSpot,
) -> (Vec<HotSpotPath>, Vec<String>) {
    let mut warnings = Vec::new();
    let coords = mesh.coords();
    let distances = hot_spot.distances();
    let reach = distances.iter().copied().fold(0.0, f64::max);
    let Some(direction) = normalize(hot_spot.direction) else {
        warnings.push(format!("{}: The path direction is zero.", hot_spot.name));
        return (Vec::new(), warnings);
    };
    let mut toe: Vec<(NodeId, usize)> = Vec::new();
    for id in hot_spot.toe.nodes(mesh) {
        match mesh.node_index(id) {
            Some(index) => toe.push((id, index)),
            None => warnings.push(format!(
                "{}: Node {id} is missing in the results.",
                hot_spot.name
            )),
        }
    }
    if toe.is_empty() || reach.is_nan() || reach <= 0.0 {
        return (Vec::new(), warnings);
    }
    // Only faces near the toe can carry read-out points.
    let toe_points: Vec<[f64; 3]> = toe.iter().map(|&(_, index)| coords[index]).collect();
    let surface = Surface::around(coords, faces, &toe_points, 1.5 * reach);
    let toe_grid = Grid::new(&toe_points.iter().map(|&p| (p, p)).collect::<Vec<_>>());

    let mut paths = Vec::new();
    for &(node, index) in &toe {
        let start = coords[index];
        let mut along = direction;
        // Perpendicular to the toe line, where the toe nodes form one.
        if let Some(tangent) = toe_tangent(&toe_points, &toe_grid, start)
            && dot(along, tangent).abs() < 0.95
        {
            along = normalize(sub(along, scale(tangent, dot(along, tangent)))).unwrap_or(along);
        }
        // Into the plate surface, which the first read-out point lies on.
        if let Some((triangle, _)) = surface.closest(add(start, scale(along, distances[0]))) {
            along = in_plane(along, triangle.normal).unwrap_or(along);
        }
        let mut points = Vec::with_capacity(distances.len());
        let mut gap: f64 = 0.0;
        for &distance in &distances {
            let Some((point, off)) = surface.place(start, along, distance) else {
                break;
            };
            gap = gap.max(off);
            points.push(point);
        }
        if points.len() < distances.len() {
            warnings.push(format!(
                "{}: No surface lies on the path of node {node}.",
                hot_spot.name
            ));
            continue;
        }
        if gap > GAP_WARNING {
            warnings.push(format!(
                "{}: The path of node {node} leaves the plate surface by {:.0} % of the \
                 distance; check the direction.",
                hot_spot.name,
                gap * 100.0
            ));
        }
        paths.push(HotSpotPath {
            node,
            index,
            position: start,
            direction: along,
            points,
        });
    }
    (paths, warnings)
}

/// `count` points evenly along a path from its toe to `reach`, to plot the stress along it:
/// the first is the toe node itself, the others lie on the surface like read-out points.
pub fn path_curve<'a>(
    mesh: &FeMesh,
    faces: impl IntoIterator<Item = &'a SkinFace>,
    path: &HotSpotPath,
    reach: f64,
    count: usize,
) -> Vec<ReadoutPoint> {
    let count = count.max(2);
    if !(reach > 0.0 && reach.is_finite()) {
        return Vec::new();
    }
    let surface = Surface::around(mesh.coords(), faces, &[path.position], 1.5 * reach);
    let mut points = vec![ReadoutPoint {
        distance: 0.0,
        position: path.position,
        direction: path.direction,
        weights: vec![(path.index, 1.0)],
    }];
    for i in 1..count {
        let distance = reach * i as f64 / (count - 1) as f64;
        // Points that left the plate are no stress on the path.
        match surface.place(path.position, path.direction, distance) {
            Some((point, gap)) if gap <= GAP_WARNING => points.push(point),
            _ => break,
        }
    }
    points
}

/// Evaluates every definition for every increment with stresses. `faces` are the surface
/// faces of `mesh`, the mesh of the results.
pub fn evaluate(
    mesh: &FeMesh,
    faces: &[&SkinFace],
    hot_spots: &[HotSpot],
    increments: &[Increment],
) -> Vec<HotSpotReport> {
    hot_spots
        .iter()
        .map(|hot_spot| {
            let (paths, mut warnings) = hot_spot_paths(mesh, faces.iter().copied(), hot_spot);
            let distances = hot_spot.distances();
            let weights = extrapolation_weights(&distances);
            let increments: Vec<IncrementValues> = increments
                .iter()
                .filter_map(|increment| {
                    Some(IncrementValues {
                        step: increment.step,
                        increment: increment.increment,
                        value: increment.value,
                        values: evaluate_increment(
                            &paths,
                            hot_spot.component,
                            &weights,
                            increment,
                        )?,
                    })
                })
                .collect();
            if increments.is_empty() && !paths.is_empty() {
                warnings.push(format!(
                    "{}: The results contain no stresses (S).",
                    hot_spot.name
                ));
            }
            HotSpotReport {
                name: hot_spot.name.clone(),
                component: hot_spot.component,
                method: hot_spot.extrapolation.label().to_string(),
                distances,
                paths,
                increments,
                warnings,
            }
        })
        .collect()
}

/// The values of all paths in one increment; `None` without stresses.
pub fn evaluate_increment(
    paths: &[HotSpotPath],
    component: HotSpotComponent,
    weights: &[f64],
    increment: &Increment,
) -> Option<Vec<HotSpotValue>> {
    let columns = stress_columns(increment)?;
    let values = paths
        .iter()
        .map(|path| {
            let readings = read(&columns, &path.points, component);
            let hot_spot = readings.iter().zip(weights).map(|(s, w)| s * w).sum();
            HotSpotValue {
                node: path.node,
                readings,
                hot_spot,
            }
        })
        .collect();
    Some(values)
}

/// The chosen stress at `points` in one increment; `None` without stresses.
pub fn readings(
    points: &[ReadoutPoint],
    component: HotSpotComponent,
    increment: &Increment,
) -> Option<Vec<f64>> {
    Some(read(&stress_columns(increment)?, points, component))
}

fn stress_columns(increment: &Increment) -> Option<Vec<&Vec<f32>>> {
    let field = increment.field("STRESS")?;
    let columns = STRESS_COMPONENTS.map(|name| field.component(name).map(|c| &c.values));
    columns.into_iter().collect()
}

fn read(columns: &[&Vec<f32>], points: &[ReadoutPoint], component: HotSpotComponent) -> Vec<f64> {
    points
        .iter()
        .map(|point| {
            let tensor: [f64; 6] = std::array::from_fn(|k| {
                (point.weights.iter())
                    .map(|&(n, w)| w * columns[k].get(n).map_or(f64::NAN, |&v| v as f64))
                    .sum()
            });
            stress(tensor, component, point.direction)
        })
        .collect()
}

/// The evaluated stress of a tensor (xx, yy, zz, xy, yz, zx); `direction` is the path's.
fn stress(t: [f64; 6], component: HotSpotComponent, direction: [f64; 3]) -> f64 {
    let [xx, yy, zz, xy, yz, zx] = t;
    match component {
        HotSpotComponent::Perpendicular => {
            let [x, y, z] = direction;
            x * x * xx + y * y * yy + z * z * zz + 2.0 * (x * y * xy + y * z * yz + z * x * zx)
        }
        HotSpotComponent::MaxPrincipal => principal_values(t)[0],
        HotSpotComponent::SignedMaxAbsPrincipal => {
            let [p1, _, p3] = principal_values(t);
            if p1.abs() >= p3.abs() { p1 } else { p3 }
        }
    }
}

/// The reports as a table for spreadsheets: semicolons, decimal points, one row per path
/// and increment, UTF-8 with byte order mark so that Excel reads the names right.
pub fn to_csv(reports: &[HotSpotReport]) -> String {
    let points = reports.iter().map(|r| r.distances.len()).max().unwrap_or(0);
    let mut text = String::from("\u{feff}Hot Spot;Method;Component;Step;Increment;Time;Node;X;Y;Z");
    for i in 1..=points {
        text += &format!(";d{i};S{i}");
    }
    text += ";S_hs\n";
    for report in reports {
        for increment in &report.increments {
            for (path, value) in report.paths.iter().zip(&increment.values) {
                let [x, y, z] = path.position;
                text += &format!(
                    "{};{};{};{};{};{};{};{x:.4};{y:.4};{z:.4}",
                    report.name,
                    report.method,
                    report.component.short(),
                    increment.step,
                    increment.increment,
                    increment.value,
                    value.node,
                );
                for i in 0..points {
                    match (report.distances.get(i), value.readings.get(i)) {
                        (Some(d), Some(s)) => text += &format!(";{d:.4};{s:.4}"),
                        _ => text += ";;",
                    }
                }
                text += &format!(";{:.4}\n", value.hot_spot);
            }
        }
    }
    text
}

/// Splits a face into triangles: linear triangles stay as they are, other faces are fanned
/// out from their centre through all corner and midside nodes.
fn triangulate(coords: &[[f64; 3]], face: &SkinFace, triangles: &mut Vec<Triangle>) {
    let (corners, mids) = (&face.corners, &face.mids);
    let quadratic = mids.len() == corners.len();
    let ring: Vec<usize> = if quadratic {
        corners
            .iter()
            .zip(mids)
            .flat_map(|(&c, &m)| [c, m])
            .collect()
    } else {
        corners.clone()
    };
    let node = |n: usize| (coords[n], vec![(n, 1.0)]);
    let mut push = |a: ([f64; 3], Vec<(usize, f64)>), b: ([f64; 3], _), c: ([f64; 3], _)| {
        let normal = normalize(cross(sub(b.0, a.0), sub(c.0, a.0)));
        if let Some(normal) = normal {
            triangles.push(Triangle {
                corners: [a.0, b.0, c.0],
                weights: [a.1, b.1, c.1],
                normal,
            });
        }
    };
    if corners.len() == 3 && !quadratic {
        push(node(ring[0]), node(ring[1]), node(ring[2]));
        return;
    }
    // Value of the face's shape functions at its centre.
    let (corner_weight, mid_weight) = match (corners.len(), quadratic) {
        (3, true) => (-1.0 / 9.0, 4.0 / 9.0),
        (4, true) => (-0.25, 0.5),
        (n, _) => (1.0 / n as f64, 0.0),
    };
    let mut centre_weights: Vec<(usize, f64)> =
        corners.iter().map(|&c| (c, corner_weight)).collect();
    if quadratic {
        centre_weights.extend(mids.iter().map(|&m| (m, mid_weight)));
    }
    let mut centre = [0.0; 3];
    for &(n, w) in &centre_weights {
        centre = add(centre, scale(coords[n], w));
    }
    for i in 0..ring.len() {
        let next = ring[(i + 1) % ring.len()];
        push((centre, centre_weights.clone()), node(ring[i]), node(next));
    }
}

/// The triangle nearest to `point` with the barycentric coordinates of the nearest point.
fn closest<'a>(
    triangles: &'a [Triangle],
    grid: &Grid,
    point: [f64; 3],
) -> Option<(&'a Triangle, [f64; 3])> {
    let nearest = |t: &Triangle| {
        let bary = closest_on_triangle(point, t.corners);
        let [a, b, c] = t.corners;
        let q = add(add(scale(a, bary[0]), scale(b, bary[1])), scale(c, bary[2]));
        (bary, length(sub(q, point)))
    };
    let (index, _) = grid.nearest(point, |i| nearest(&triangles[i]).1)?;
    let triangle = &triangles[index];
    Some((triangle, nearest(triangle).0))
}

/// Lowest and highest corner of the box around `points`.
fn bounds(points: &[[f64; 3]]) -> ([f64; 3], [f64; 3]) {
    let (mut low, mut high) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in points {
        for k in 0..3 {
            low[k] = low[k].min(p[k]);
            high[k] = high[k].max(p[k]);
        }
    }
    (low, high)
}

/// Barycentric coordinates of the point of triangle `abc` nearest to `p` (Ericson,
/// Real-Time Collision Detection, 5.1.5).
fn closest_on_triangle(p: [f64; 3], [a, b, c]: [[f64; 3]; 3]) -> [f64; 3] {
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return [1.0, 0.0, 0.0];
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return [0.0, 1.0, 0.0];
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return [1.0 - v, v, 0.0];
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return [0.0, 0.0, 1.0];
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return [1.0 - w, 0.0, w];
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return [0.0, 1.0 - w, w];
    }
    let denom = 1.0 / (va + vb + vc);
    let (v, w) = (vb * denom, vc * denom);
    [1.0 - v - w, v, w]
}

/// Direction of the toe line at a toe node: towards the nearest other toe node.
fn toe_tangent(toe: &[[f64; 3]], grid: &Grid, here: [f64; 3]) -> Option<[f64; 3]> {
    let distance = |i: usize| match length(sub(toe[i], here)) {
        0.0 => f64::INFINITY,
        d => d,
    };
    let (index, d) = grid.nearest(here, distance)?;
    if d.is_finite() {
        normalize(sub(toe[index], here))
    } else {
        None
    }
}

/// `v` turned into the plane with normal `n`, as a unit vector.
fn in_plane(v: [f64; 3], n: [f64; 3]) -> Option<[f64; 3]> {
    normalize(sub(v, scale(n, dot(v, n))))
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
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

fn length(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn normalize(a: [f64; 3]) -> Option<[f64; 3]> {
    let l = length(a);
    (l > 1e-12 && l.is_finite()).then(|| scale(a, 1.0 / l))
}

#[cfg(test)]
mod tests {
    use plx_mesh::{Element, ElementShape, Part, extract_part_skin};
    use plx_model::Region;

    use super::*;
    use crate::{AnalysisKind, Component, Field};

    /// Plate 0..20 x 0..10 x 0..5 of 2 x 5 x 5 mm hexahedra, nodes numbered from 1.
    fn plate() -> FeMesh {
        let mut mesh = FeMesh::default();
        let (nx, ny, nz) = (10, 2, 1);
        let id = |i: u32, j: u32, k: u32| 1 + i + (nx + 1) * (j + (ny + 1) * k);
        for k in 0..=nz {
            for j in 0..=ny {
                for i in 0..=nx {
                    let p = [2.0 * i as f64, 5.0 * j as f64, 5.0 * k as f64];
                    mesh.set_node(id(i, j, k), p);
                }
            }
        }
        let mut elements = Vec::new();
        for j in 0..ny {
            for i in 0..nx {
                let e = 1 + i + nx * j;
                let bottom = [
                    id(i, j, 0),
                    id(i + 1, j, 0),
                    id(i + 1, j + 1, 0),
                    id(i, j + 1, 0),
                ];
                let top = bottom.map(|n| n + (nx + 1) * (ny + 1));
                mesh.add_element(Element {
                    id: e,
                    type_name: "C3D8".into(),
                    shape: ElementShape::Hex8,
                    nodes: bottom.into_iter().chain(top).collect(),
                })
                .unwrap();
                elements.push(e);
            }
        }
        mesh.parts.push(Part {
            name: "PLATE".into(),
            elements,
        });
        mesh
    }

    /// An increment whose stress is `s11(x)` along x and 50 across.
    fn increment(mesh: &FeMesh, s11: impl Fn(f64) -> f64) -> Increment {
        let column = |f: &dyn Fn([f64; 3]) -> f64| -> Vec<f32> {
            mesh.coords().iter().map(|&p| f(p) as f32).collect()
        };
        let components = STRESS_COMPONENTS
            .iter()
            .enumerate()
            .map(|(k, name)| Component {
                name: name.to_string(),
                values: match k {
                    0 => column(&|p| s11(p[0])),
                    1 => column(&|_| 50.0),
                    _ => column(&|_| 0.0),
                },
                derived: false,
                out_of_plane: false,
            })
            .collect();
        Increment {
            nodal_diameter: None,
            step: 1,
            increment: 1,
            kind: AnalysisKind::Static,
            value: 1.0,
            fields: vec![Field::new("STRESS", components)],
        }
    }

    fn report(mesh: &FeMesh, hot_spot: &HotSpot, increments: &[Increment]) -> HotSpotReport {
        let skin = extract_part_skin(mesh, &mesh.parts[0], 30.0);
        let faces: Vec<&SkinFace> = skin.faces.iter().collect();
        evaluate(mesh, &faces, std::slice::from_ref(hot_spot), increments).remove(0)
    }

    #[test]
    fn linear_stress_rise_is_extrapolated_exactly() {
        let mesh = plate();
        // Toe line along y at x = 2 on the top face; the given direction is tilted out of
        // the plate and gets turned into it.
        let toe = [2, 13, 24].map(|n| n + 33);
        let hot_spot = HotSpot {
            toe: Region::Nodes(toe.to_vec()),
            direction: [1.0, 0.2, 0.3],
            thickness: 5.0,
            ..HotSpot::new("Hot_Spot-1")
        };
        let report = report(&mesh, &hot_spot, &[increment(&mesh, |x| 100.0 + 10.0 * x)]);
        assert!(report.warnings.is_empty(), "{:?}", report.warnings);
        assert_eq!(report.distances, [2.0, 5.0]);
        let values = &report.increments[0].values;
        assert_eq!(values.len(), 3);
        for (path, value) in report.paths.iter().zip(values) {
            assert_eq!(path.position[0], 2.0);
            assert_eq!(path.points[1].position[2], 5.0);
            assert!((value.readings[0] - 140.0).abs() < 1e-6, "{value:?}");
            assert!((value.readings[1] - 170.0).abs() < 1e-6, "{value:?}");
            assert!((value.hot_spot - 120.0).abs() < 1e-6, "{value:?}");
        }
    }

    #[test]
    fn curve_follows_the_stress_along_the_path() {
        let mesh = plate();
        let hot_spot = HotSpot {
            toe: Region::Nodes(vec![35]),
            thickness: 5.0,
            ..HotSpot::new("Hot_Spot-1")
        };
        let skin = extract_part_skin(&mesh, &mesh.parts[0], 30.0);
        let (paths, warnings) = hot_spot_paths(&mesh, &skin.faces, &hot_spot);
        assert!(warnings.is_empty(), "{warnings:?}");
        assert_eq!(paths[0].direction, [1.0, 0.0, 0.0]);
        let curve = path_curve(&mesh, &skin.faces, &paths[0], 10.0, 11);
        assert_eq!(curve.len(), 11);
        assert_eq!(curve[0].weights, [(paths[0].index, 1.0)]);
        let increment = increment(&mesh, |x| 100.0 + 10.0 * x);
        let values = readings(&curve, HotSpotComponent::Perpendicular, &increment).unwrap();
        for (point, value) in curve.iter().zip(values) {
            let x = paths[0].position[0] + point.distance;
            assert!(
                (value - (100.0 + 10.0 * x)).abs() < 1e-6,
                "{point:?}: {value}"
            );
        }
        // Beyond the plate's end the curve stops.
        assert!(path_curve(&mesh, &skin.faces, &paths[0], 40.0, 41).len() < 41);
    }

    #[test]
    fn principal_stress_and_custom_points() {
        let mesh = plate();
        let hot_spot = HotSpot {
            toe: Region::Nodes(vec![34]),
            thickness: 5.0,
            extrapolation: Extrapolation::Custom(vec![2.0, 4.0, 8.0]),
            component: HotSpotComponent::MaxPrincipal,
            ..HotSpot::new("Hot_Spot-1")
        };
        // Below 50 the stress across the path is the largest one.
        let report = report(&mesh, &hot_spot, &[increment(&mesh, |x| 10.0 * x)]);
        let value = &report.increments[0].values[0];
        for (reading, expected) in value.readings.iter().zip([50.0, 50.0, 80.0]) {
            assert!((reading - expected).abs() < 1e-4, "{value:?}");
        }
        // The parabola through (2, 50), (4, 50) and (8, 80) starts at 60.
        assert!((value.hot_spot - 60.0).abs() < 1e-4, "{value:?}");
    }

    #[test]
    fn paths_leaving_the_surface_are_reported() {
        let mesh = plate();
        let hot_spot = HotSpot {
            toe: Region::Nodes(vec![36, 999]),
            direction: [0.0, 0.0, 1.0],
            thickness: 5.0,
            ..HotSpot::new("Hot_Spot-1")
        };
        let report = report(&mesh, &hot_spot, &[]);
        assert_eq!(report.warnings.len(), 3, "{:?}", report.warnings);
        assert!(report.warnings[0].contains("999"));
        assert!(report.warnings[1].contains("direction"));
        assert!(report.warnings[2].contains("no stresses"));
    }

    #[test]
    fn csv_has_a_row_per_path_and_increment() {
        let mesh = plate();
        let hot_spot = HotSpot {
            toe: Region::Nodes(vec![35, 36]),
            thickness: 5.0,
            ..HotSpot::new("Weld top")
        };
        let report = report(&mesh, &hot_spot, &[increment(&mesh, |x| x)]);
        let csv = to_csv(&[report]);
        let lines: Vec<&str> = csv.lines().collect();
        assert_eq!(lines.len(), 3);
        assert!(lines[0].ends_with(";d1;S1;d2;S2;S_hs"));
        assert!(lines[1].starts_with("Weld top;IIW a"));
        assert!(
            lines[2].ends_with(";2.0000;6.0000;5.0000;9.0000;4.0000"),
            "{}",
            lines[2]
        );
    }
}
