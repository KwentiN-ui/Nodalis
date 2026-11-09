//! A load that travels over a surface, such as a vehicle crossing a bridge or a crane on its
//! runway beam. The solver has no such load; the input file gets one static step per position,
//! each with the wheel loads at that position spread over the faces under the wheels.

use std::collections::BTreeMap;

use plx_mesh::{ElementId, FeMesh, NodeId};
use serde::{Deserialize, Serialize};

use crate::{CoordinateSystemKind, FeModel};

/// The origin of a path and its unit directions along and across, in global coordinates.
pub type PathAxes = ([f64; 3], [f64; 3], [f64; 3]);

/// The axis labels of a coordinate system, by index.
pub const AXIS_LABELS: [&str; 3] = ["X", "Y", "Z"];

/// A group of wheels that drives along an axis of a coordinate system. Every wheel presses on
/// a rectangular contact patch; the faces of the region below the patch take its force by
/// area. A wheel that has left the region carries no force there, one that is partly on it
/// the part of its force that the covered part of the patch takes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MovingLoad {
    /// Coordinate system of the path; `None` for the global one.
    pub system: Option<crate::Id>,
    /// Axis of the system the wheels travel along (0 = x, 1 = y, 2 = z).
    pub travel_axis: usize,
    /// Axis of the system across the path, along which the wheels sit side by side.
    pub lateral_axis: usize,
    /// Position of the vehicle's reference point along the travel axis in the first step.
    pub start: f64,
    /// Position in the last step.
    pub end: f64,
    /// Number of positions, each a step of its own.
    pub positions: u32,
    /// Length of a wheel's contact patch along the path.
    pub patch_length: f64,
    /// Width of a contact patch across the path; 2D models have none.
    pub patch_width: f64,
    pub wheels: Vec<Wheel>,
}

/// A wheel relative to the vehicle's reference point, with the force it puts on the ground in
/// global directions.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Wheel {
    /// Offset along the path.
    pub along: f64,
    /// Offset across the path.
    pub across: f64,
    pub force: [f64; 3],
}

impl Default for MovingLoad {
    /// Two wheels 1 m apart on a path along x, as a starting point for the dialog; in SI
    /// units like all stored values.
    fn default() -> Self {
        Self {
            system: None,
            travel_axis: 0,
            lateral_axis: 1,
            start: 0.0,
            end: 1.0,
            positions: 11,
            patch_length: 0.1,
            patch_width: 0.1,
            wheels: vec![
                Wheel {
                    along: -0.5,
                    across: 0.0,
                    force: [0.0; 3],
                },
                Wheel {
                    along: 0.5,
                    across: 0.0,
                    force: [0.0; 3],
                },
            ],
        }
    }
}

/// Where a moving load stands in the steps of the results, by CalculiX step number.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LoadStations {
    /// Name of the load; a load of that name in several steps gives all of their positions.
    pub load: String,
    /// The CalculiX step number and the position of the vehicle's reference point in it.
    pub steps: Vec<(u32, f64)>,
}

/// Where each moving load of the model stands in which step of the input file the writer
/// gives: every active step is a `*Step`, one with moving loads one per position, and a load
/// with fewer positions than another of its step stays at its last one.
pub fn load_stations(model: &FeModel) -> Vec<LoadStations> {
    let mut result: Vec<LoadStations> = Vec::new();
    let mut number = 0;
    for step in model.steps.iter().filter(|s| s.active) {
        let moving: Vec<(&str, Vec<f64>)> = (step.loads.iter())
            .filter(|l| l.active && step.kind.supports_load(&l.kind))
            .filter_map(|l| match &l.kind {
                crate::LoadKind::Moving(m) => Some((l.name.as_str(), m.stations())),
                _ => None,
            })
            .collect();
        let count = moving.iter().map(|(_, s)| s.len()).max().unwrap_or(1);
        for position in 0..count {
            number += 1;
            for (name, stations) in &moving {
                let station = stations[position.min(stations.len() - 1)];
                match result.iter_mut().find(|r| r.load == *name) {
                    Some(r) => r.steps.push((number, station)),
                    None => result.push(LoadStations {
                        load: name.to_string(),
                        steps: vec![(number, station)],
                    }),
                }
            }
        }
    }
    result
}

/// The nodal forces of a moving load at one position.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MovingFrame {
    pub forces: BTreeMap<NodeId, [f64; 3]>,
    /// Share of each wheel's force that acts, by wheel: 1 on the region, 0 off it.
    pub on_region: Vec<f64>,
}

impl MovingLoad {
    /// The position of the vehicle's reference point in each step.
    pub fn stations(&self) -> Vec<f64> {
        let n = self.positions.max(1);
        if n == 1 {
            return vec![self.start];
        }
        let step = (self.end - self.start) / f64::from(n - 1);
        (0..n).map(|i| self.start + step * f64::from(i)).collect()
    }

    /// Why the load cannot be written, if it cannot.
    pub fn problem(&self, model: &FeModel, two_d: bool) -> Option<String> {
        let Some(system) = model.coordinate_system_or_global(self.system) else {
            return Some("The coordinate system does not exist".into());
        };
        if system.kind != CoordinateSystemKind::Rectangular {
            return Some(format!(
                "Coordinate System {} is not rectangular",
                system.name
            ));
        }
        if let Err(reason) = system.axes() {
            return Some(reason);
        }
        if self.travel_axis > 2 || self.lateral_axis > 2 {
            return Some("unknown axis".into());
        }
        if !two_d && self.travel_axis == self.lateral_axis {
            return Some("the path and the lateral direction are the same axis".into());
        }
        if self.positions == 0 {
            return Some("no positions".into());
        }
        if self.wheels.is_empty() {
            return Some("no wheels".into());
        }
        if self.patch_length.is_nan() || self.patch_length <= 0.0 {
            return Some("the contact patch has no length".into());
        }
        if !two_d && (self.patch_width.is_nan() || self.patch_width <= 0.0) {
            return Some("the contact patch has no width".into());
        }
        None
    }

    /// The origin and the unit directions along and across the path in global coordinates.
    pub fn path_axes(&self, model: &FeModel) -> Result<PathAxes, String> {
        let system = (model.coordinate_system_or_global(self.system))
            .ok_or("The coordinate system does not exist")?;
        let axes = system.axes()?;
        let travel = *axes.get(self.travel_axis).ok_or("unknown axis")?;
        let lateral = *axes.get(self.lateral_axis).ok_or("unknown axis")?;
        Ok((system.origin, travel, lateral))
    }

    /// The global centre of every wheel's contact patch at a station, for the symbols.
    pub fn wheel_points(&self, model: &FeModel, station: f64) -> Vec<[f64; 3]> {
        let Ok((origin, travel, lateral)) = self.path_axes(model) else {
            return Vec::new();
        };
        (self.wheels.iter())
            .map(|w| {
                let (u, v) = (station + w.along, w.across);
                std::array::from_fn(|k| origin[k] + u * travel[k] + v * lateral[k])
            })
            .collect()
    }

    /// The nodal forces at every station. The faces are those of the region; in a 2D model
    /// they are element edges and the patch has only a length.
    pub fn frames(
        &self,
        model: &FeModel,
        mesh: &FeMesh,
        faces: &[(ElementId, u8)],
        two_d: bool,
    ) -> Result<Vec<MovingFrame>, String> {
        if let Some(problem) = self.problem(model, two_d) {
            return Err(problem);
        }
        let (origin, travel, lateral) = self.path_axes(model)?;
        let project = |p: [f64; 3]| -> [f64; 2] {
            let d = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]];
            [dot(d, travel), dot(d, lateral)]
        };
        let pieces = face_pieces(mesh, faces, &project);
        let (half_length, half_width) = (self.patch_length / 2.0, self.patch_width / 2.0);
        let patch_area = if two_d {
            self.patch_length
        } else {
            self.patch_length * self.patch_width
        };
        let mut frames = Vec::new();
        for station in self.stations() {
            let mut frame = MovingFrame::default();
            for wheel in &self.wheels {
                let u = station + wheel.along;
                let v = wheel.across;
                let window = if two_d {
                    [
                        u - half_length,
                        u + half_length,
                        f64::NEG_INFINITY,
                        f64::INFINITY,
                    ]
                } else {
                    [
                        u - half_length,
                        u + half_length,
                        v - half_width,
                        v + half_width,
                    ]
                };
                let mut weights: BTreeMap<NodeId, f64> = BTreeMap::new();
                let mut covered = 0.0;
                for piece in pieces.iter().filter(|p| p.overlaps(&window)) {
                    for (node, weight) in piece.weights(&window) {
                        covered += weight;
                        *weights.entry(node).or_default() += weight;
                    }
                }
                // Faces on top of each other, like both flanges of a beam, would carry the
                // wheel twice; the force never exceeds the wheel's.
                let share = (covered / patch_area).min(1.0);
                frame.on_region.push(share);
                if covered <= 0.0 {
                    continue;
                }
                for (node, weight) in weights {
                    let entry = frame.forces.entry(node).or_insert([0.0; 3]);
                    for (value, force) in entry.iter_mut().zip(wheel.force) {
                        *value += force * share * weight / covered;
                    }
                }
            }
            frames.push(frame);
        }
        Ok(frames)
    }
}

/// A face or edge of the region cut into small straight pieces, projected onto the plane of
/// the path: triangles of a face, segments of an edge.
struct FacePieces {
    nodes: Vec<NodeId>,
    kind: PieceKind,
    /// Projected coordinates and parameters of the pieces' corners.
    points: Vec<([f64; 2], [f64; 2])>,
    pieces: Vec<[usize; 3]>,
    bounds: [f64; 4],
}

#[derive(Clone, Copy)]
enum PieceKind {
    Edge { quadratic: bool },
    Triangle { quadratic: bool },
    Quad { quadratic: bool },
}

const DIVISIONS: usize = 4;

fn face_pieces(
    mesh: &FeMesh,
    faces: &[(ElementId, u8)],
    project: &dyn Fn([f64; 3]) -> [f64; 2],
) -> Vec<FacePieces> {
    let mut out = Vec::new();
    for &(element, face) in faces {
        let Some(element) = mesh.element(element) else {
            continue;
        };
        let Some(topology) = (face as usize)
            .checked_sub(1)
            .and_then(|f| element.faces().get(f))
        else {
            continue;
        };
        let quadratic = element.shape.is_quadratic() && !topology.mids.is_empty();
        let locals: Vec<usize> = if quadratic {
            topology
                .corners
                .iter()
                .chain(topology.mids)
                .copied()
                .collect()
        } else {
            topology.corners.to_vec()
        };
        let Some(nodes) = (locals.iter())
            .map(|&l| element.nodes.get(l).copied())
            .collect::<Option<Vec<NodeId>>>()
        else {
            continue;
        };
        let Some(coords) = (nodes.iter())
            .map(|&n| mesh.node(n))
            .collect::<Option<Vec<[f64; 3]>>>()
        else {
            continue;
        };
        let kind = match topology.corners.len() {
            2 => PieceKind::Edge { quadratic },
            3 => PieceKind::Triangle { quadratic },
            4 => PieceKind::Quad { quadratic },
            _ => continue,
        };
        let mut points = Vec::new();
        let mut pieces = Vec::new();
        let mut add_point = |parameter: [f64; 2]| {
            let shape = kind.shape(parameter);
            let position: [f64; 3] =
                std::array::from_fn(|k| shape.iter().zip(&coords).map(|(n, c)| n * c[k]).sum());
            points.push((project(position), parameter));
            points.len() - 1
        };
        let n = DIVISIONS;
        match kind {
            PieceKind::Edge { .. } => {
                let ids: Vec<usize> = (0..=2 * n)
                    .map(|i| add_point([-1.0 + i as f64 / n as f64, 0.0]))
                    .collect();
                for pair in ids.windows(2) {
                    pieces.push([pair[0], pair[1], pair[1]]);
                }
            }
            PieceKind::Triangle { .. } => {
                let mut grid = vec![vec![0; n + 1]; n + 1];
                for (i, row) in grid.iter_mut().enumerate() {
                    for (j, id) in row.iter_mut().enumerate().take(n + 1 - i) {
                        *id = add_point([i as f64 / n as f64, j as f64 / n as f64]);
                    }
                }
                for i in 0..n {
                    for j in 0..n - i {
                        pieces.push([grid[i][j], grid[i + 1][j], grid[i][j + 1]]);
                        if i + j + 1 < n {
                            pieces.push([grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]]);
                        }
                    }
                }
            }
            PieceKind::Quad { .. } => {
                let mut grid = vec![vec![0; n + 1]; n + 1];
                for (i, row) in grid.iter_mut().enumerate() {
                    for (j, id) in row.iter_mut().enumerate() {
                        let s = |k: usize| -1.0 + 2.0 * k as f64 / n as f64;
                        *id = add_point([s(i), s(j)]);
                    }
                }
                for i in 0..n {
                    for j in 0..n {
                        pieces.push([grid[i][j], grid[i + 1][j], grid[i + 1][j + 1]]);
                        pieces.push([grid[i][j], grid[i + 1][j + 1], grid[i][j + 1]]);
                    }
                }
            }
        }
        let mut bounds = [
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ];
        for (p, _) in &points {
            bounds[0] = bounds[0].min(p[0]);
            bounds[1] = bounds[1].max(p[0]);
            bounds[2] = bounds[2].min(p[1]);
            bounds[3] = bounds[3].max(p[1]);
        }
        out.push(FacePieces {
            nodes,
            kind,
            points,
            pieces,
            bounds,
        });
    }
    out
}

impl FacePieces {
    fn overlaps(&self, window: &[f64; 4]) -> bool {
        self.bounds[1] > window[0]
            && self.bounds[0] < window[1]
            && (matches!(self.kind, PieceKind::Edge { .. })
                || (self.bounds[3] > window[2] && self.bounds[2] < window[3]))
    }

    /// Each node's share of the projected area (of an edge, the length) of the face inside
    /// the window: the integral of its shape function over the covered part, so that a fully
    /// covered face gets the equivalent nodal forces of a uniform pressure.
    fn weights(&self, window: &[f64; 4]) -> Vec<(NodeId, f64)> {
        let mut weights = vec![0.0; self.nodes.len()];
        let mut add = |size: f64, parameter: [f64; 2]| {
            for (weight, n) in weights.iter_mut().zip(self.kind.shape(parameter)) {
                *weight += size * n;
            }
        };
        for piece in &self.pieces {
            let corners = piece.map(|i| self.points[i]);
            if let PieceKind::Edge { .. } = self.kind {
                // Two-point Gauss rule on the covered part of the segment.
                let (a, b) = (corners[0], corners[1]);
                let (lo, hi) = (a.0[0].min(b.0[0]), a.0[0].max(b.0[0]));
                let (from, to) = (lo.max(window[0]), hi.min(window[1]));
                if to <= from || hi <= lo {
                    continue;
                }
                let half = (to - from) / 2.0;
                for g in [-1.0 / 3f64.sqrt(), 1.0 / 3f64.sqrt()] {
                    let x = (from + to) / 2.0 + g * half;
                    let t = (x - a.0[0]) / (b.0[0] - a.0[0]);
                    add(half, [a.1[0] + t * (b.1[0] - a.1[0]), 0.0]);
                }
                continue;
            }
            let triangle = corners.map(|c| c.0);
            let polygon = clip(&triangle, window);
            // The parameters vary linearly over the piece, the shape functions at most
            // quadratically: the rule of the edge midpoints integrates them exactly.
            for i in 1..polygon.len().saturating_sub(1) {
                let fan = [polygon[0], polygon[i], polygon[i + 1]];
                let area = triangle_area(fan);
                if area <= 0.0 {
                    continue;
                }
                for (a, b) in [(0, 1), (1, 2), (2, 0)] {
                    let middle = [(fan[a][0] + fan[b][0]) / 2.0, (fan[a][1] + fan[b][1]) / 2.0];
                    let Some(l) = barycentric(triangle, middle) else {
                        continue;
                    };
                    let parameter: [f64; 2] = std::array::from_fn(|k| {
                        l[0] * corners[0].1[k] + l[1] * corners[1].1[k] + l[2] * corners[2].1[k]
                    });
                    add(area / 3.0, parameter);
                }
            }
        }
        (self.nodes.iter().copied())
            .zip(weights)
            .filter(|(_, w)| *w != 0.0)
            .collect()
    }
}

impl PieceKind {
    /// Shape functions at a parameter point, corner nodes first, then midside nodes:
    /// edges and quadrilaterals on [-1, 1], triangles on the unit triangle.
    fn shape(self, [r, s]: [f64; 2]) -> Vec<f64> {
        match self {
            PieceKind::Edge { quadratic: false } => vec![(1.0 - r) / 2.0, (1.0 + r) / 2.0],
            PieceKind::Edge { quadratic: true } => {
                vec![r * (r - 1.0) / 2.0, r * (r + 1.0) / 2.0, 1.0 - r * r]
            }
            PieceKind::Triangle { quadratic } => {
                let l = [1.0 - r - s, r, s];
                if quadratic {
                    vec![
                        l[0] * (2.0 * l[0] - 1.0),
                        l[1] * (2.0 * l[1] - 1.0),
                        l[2] * (2.0 * l[2] - 1.0),
                        4.0 * l[0] * l[1],
                        4.0 * l[1] * l[2],
                        4.0 * l[2] * l[0],
                    ]
                } else {
                    l.to_vec()
                }
            }
            PieceKind::Quad { quadratic } => {
                const CORNERS: [[f64; 2]; 4] = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
                if quadratic {
                    let mut n: Vec<f64> = CORNERS
                        .iter()
                        .map(|[a, b]| (1.0 + r * a) * (1.0 + s * b) * (r * a + s * b - 1.0) / 4.0)
                        .collect();
                    n.extend([
                        (1.0 - r * r) * (1.0 - s) / 2.0,
                        (1.0 + r) * (1.0 - s * s) / 2.0,
                        (1.0 - r * r) * (1.0 + s) / 2.0,
                        (1.0 - r) * (1.0 - s * s) / 2.0,
                    ]);
                    n
                } else {
                    (CORNERS.iter())
                        .map(|[a, b]| (1.0 + r * a) * (1.0 + s * b) / 4.0)
                        .collect()
                }
            }
        }
    }
}

/// The part of a polygon inside the window [x0, x1, y0, y1] (Sutherland-Hodgman).
fn clip(polygon: &[[f64; 2]], window: &[f64; 4]) -> Vec<[f64; 2]> {
    let mut points = polygon.to_vec();
    // Each edge of the window as (axis, limit, keep the side above the limit).
    for (axis, limit, above) in [
        (0, window[0], true),
        (0, window[1], false),
        (1, window[2], true),
        (1, window[3], false),
    ] {
        if !limit.is_finite() || points.is_empty() {
            continue;
        }
        let inside = |p: &[f64; 2]| {
            if above {
                p[axis] >= limit
            } else {
                p[axis] <= limit
            }
        };
        let mut next = Vec::with_capacity(points.len() + 2);
        for i in 0..points.len() {
            let (a, b) = (points[i], points[(i + 1) % points.len()]);
            let (ina, inb) = (inside(&a), inside(&b));
            if ina {
                next.push(a);
            }
            if ina != inb {
                let t = (limit - a[axis]) / (b[axis] - a[axis]);
                next.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
            }
        }
        points = next;
    }
    points
}

fn triangle_area(t: [[f64; 2]; 3]) -> f64 {
    ((t[1][0] - t[0][0]) * (t[2][1] - t[0][1]) - (t[2][0] - t[0][0]) * (t[1][1] - t[0][1])).abs()
        / 2.0
}

fn barycentric(t: [[f64; 2]; 3], p: [f64; 2]) -> Option<[f64; 3]> {
    let det = (t[1][0] - t[0][0]) * (t[2][1] - t[0][1]) - (t[2][0] - t[0][0]) * (t[1][1] - t[0][1]);
    if det.abs() <= 1e-300 {
        return None;
    }
    let l1 =
        ((p[0] - t[0][0]) * (t[2][1] - t[0][1]) - (t[2][0] - t[0][0]) * (p[1] - t[0][1])) / det;
    let l2 =
        ((t[1][0] - t[0][0]) * (p[1] - t[0][1]) - (p[0] - t[0][0]) * (t[1][1] - t[0][1])) / det;
    Some([1.0 - l1 - l2, l1, l2])
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[cfg(test)]
mod tests;
