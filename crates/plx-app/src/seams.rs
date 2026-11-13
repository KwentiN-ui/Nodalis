//! Where the surfaces of different parts, or of the bodies of a compound part, meet. Their
//! meshes share no nodes, so the surface angle selection finds its way across by geometry:
//! along outline edges of one part that lie on outline edges of another, whether their
//! nodes coincide or not.

use std::collections::HashMap;

use glam::DVec3;
use plx_mesh::PartSkin;

/// Two segments are taken to lie on each other where they are this far apart at most, as
/// a share of the shorter one: room for chords of a curve divided differently on each side.
const RELATIVE_TOLERANCE: f64 = 0.05;

/// The outline edges of all parts in a grid, to find those lying on a given edge.
#[derive(Debug, Default)]
pub struct Seams {
    /// Part and end nodes (indices into the mesh's coordinates) of each outline edge.
    edges: Vec<(usize, usize, usize)>,
    /// The outline edges of each part by their end nodes, the smaller first.
    by_nodes: HashMap<(usize, usize, usize), u32>,
    cell: f64,
    grid: HashMap<[i64; 3], Vec<u32>>,
}

impl Seams {
    pub fn new(skins: &[PartSkin], coords: &[[f64; 3]]) -> Self {
        let mut seams = Self::default();
        for (part, skin) in skins.iter().enumerate() {
            for edge in skin.edges.iter().filter(|e| e.feature) {
                let (a, b) = (edge.a.min(edge.b), edge.a.max(edge.b));
                if a == b || b >= coords.len() {
                    continue;
                }
                let index = seams.edges.len() as u32;
                seams.by_nodes.entry((part, a, b)).or_insert(index);
                seams.edges.push((part, a, b));
            }
        }
        if seams.edges.is_empty() {
            return seams;
        }
        // Cells about as large as the typical edge: a long edge spans a few of them.
        let mut lengths: Vec<f64> = (seams.edges.iter())
            .map(|&(_, a, b)| point(coords, a).distance(point(coords, b)))
            .collect();
        let middle = lengths.len() / 2;
        let typical = *lengths.select_nth_unstable_by(middle, f64::total_cmp).1;
        seams.cell = 2.0 * typical.max(f64::MIN_POSITIVE);
        for (index, &(_, a, b)) in seams.edges.iter().enumerate() {
            for cell in seams.cells_along(point(coords, a), point(coords, b)) {
                let entry = seams.grid.entry(cell).or_default();
                if entry.last() != Some(&(index as u32)) {
                    entry.push(index as u32);
                }
            }
        }
        seams
    }

    /// The outline edges, as part and end nodes, lying on the edge from node `a` to `b` of
    /// `part` but not sharing both its nodes, of the parts for which `visible` holds. None
    /// where `a`-`b` is no outline edge: a part can only be left across its outline.
    pub fn along(
        &self,
        part: usize,
        a: usize,
        b: usize,
        coords: &[[f64; 3]],
        visible: impl Fn(usize) -> bool,
    ) -> Vec<(usize, usize, usize)> {
        let key = (part, a.min(b), a.max(b));
        if !self.by_nodes.contains_key(&key) {
            return Vec::new();
        }
        let (p, q) = (point(coords, key.1), point(coords, key.2));
        let mut candidates: Vec<u32> = Vec::new();
        for [x, y, z] in self.cells_along(p, q) {
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(edges) = self.grid.get(&[x + dx, y + dy, z + dz]) {
                            candidates.extend(edges);
                        }
                    }
                }
            }
        }
        candidates.sort_unstable();
        candidates.dedup();
        (candidates.into_iter())
            .map(|i| self.edges[i as usize])
            .filter(|&edge| edge != key && visible(edge.0))
            .filter(|&(_, c, d)| overlap(p, q, point(coords, c), point(coords, d)))
            .collect()
    }

    /// The outline edges, as part, end nodes and the end lying on node `node`, with an end
    /// on `node` of `part`, of the parts for which `visible` holds: where an outline running
    /// through one part goes on in another, or in another body of a compound part. `length`
    /// is the length of the edge arriving there, for the tolerance. The edges of `part` at
    /// `node` itself are among them.
    pub fn at(
        &self,
        node: usize,
        length: f64,
        coords: &[[f64; 3]],
        visible: impl Fn(usize) -> bool,
    ) -> Vec<(usize, usize, usize, usize)> {
        if self.edges.is_empty() {
            return Vec::new();
        }
        let p = point(coords, node);
        let [x, y, z] = self.cells_along(p, p)[0];
        let mut found = Vec::new();
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    for &i in self
                        .grid
                        .get(&[x + dx, y + dy, z + dz])
                        .into_iter()
                        .flatten()
                    {
                        let (part, a, b) = self.edges[i as usize];
                        if !visible(part) {
                            continue;
                        }
                        let other = point(coords, a).distance(point(coords, b));
                        let tolerance = RELATIVE_TOLERANCE * length.min(other);
                        for end in [a, b] {
                            if end == node || point(coords, end).distance(p) <= tolerance {
                                found.push((part, a, b, end));
                            }
                        }
                    }
                }
            }
        }
        found.sort_unstable();
        found.dedup();
        found
    }

    /// The cells of points along the segment, no further apart than half a cell.
    fn cells_along(&self, p: DVec3, q: DVec3) -> Vec<[i64; 3]> {
        let steps = ((p.distance(q) / (0.5 * self.cell)).ceil() as usize).clamp(1, 10_000);
        let mut cells: Vec<[i64; 3]> = (0..=steps)
            .map(|i| {
                let c = (p.lerp(q, i as f64 / steps as f64) / self.cell).floor();
                [c.x as i64, c.y as i64, c.z as i64]
            })
            .collect();
        cells.dedup();
        cells
    }
}

fn point(coords: &[[f64; 3]], node: usize) -> DVec3 {
    DVec3::from(coords[node])
}

/// Whether the segments `p`-`q` and `c`-`d` lie on each other along a stretch: where their
/// ranges overlap they are no further apart than the tolerance, and the stretch is longer.
pub fn overlap(p: DVec3, q: DVec3, c: DVec3, d: DVec3) -> bool {
    let (length, other) = (p.distance(q), c.distance(d));
    if length <= 0.0 || other <= 0.0 {
        return false;
    }
    let tolerance = RELATIVE_TOLERANCE * length.min(other);
    let direction = (q - p) / length;
    // Nearly parallel: a crossing edge has a point on the other too.
    if direction.dot((d - c) / other).abs() < 0.98 {
        return false;
    }
    // The part of `c`-`d` whose projection falls onto `p`-`q`.
    let (tc, td) = ((c - p).dot(direction), (d - p).dot(direction));
    let span = td - tc;
    let at = |t: f64| c.lerp(d, ((t - tc) / span).clamp(0.0, 1.0));
    let (from, to) = (tc.min(td).max(0.0), tc.max(td).min(length));
    if to - from <= tolerance {
        return false;
    }
    let off_line = |x: DVec3| {
        let v = x - p;
        (v - direction * v.dot(direction)).length()
    };
    off_line(at(from)) <= tolerance && off_line(at(to)) <= tolerance
}

/// An end of an edge made of segments: where it is, the direction leaving the edge there
/// and the length of the segment at the end, for the tolerance.
#[derive(Clone, Copy, Debug)]
pub struct EdgeEnd {
    pub edge: i32,
    pub point: DVec3,
    pub leaving: DVec3,
    pub length: f64,
}

impl EdgeEnd {
    /// The end of `edge` at `at`, whose segment there runs on to `next`; none for a
    /// segment of no length.
    pub fn new(edge: i32, at: DVec3, next: DVec3) -> Option<Self> {
        let length = at.distance(next);
        (length > 0.0).then(|| Self {
            edge,
            point: at,
            leaving: (at - next) / length,
            length,
        })
    }

    /// The two ends of an edge through `points` in order; none for fewer than two points.
    pub fn of_polyline(edge: i32, points: &[DVec3]) -> Vec<Self> {
        let n = points.len();
        if n < 2 {
            return Vec::new();
        }
        [
            Self::new(edge, points[0], points[1]),
            Self::new(edge, points[n - 1], points[n - 2]),
        ]
        .into_iter()
        .flatten()
        .collect()
    }
}

/// For each edge the other edges with an end on one of its ends, and how much the path bends
/// from the one into the other there in radians: 0 where the second goes straight on. Ends
/// meet where they are no further apart than the tolerance, as a share of the shorter end
/// segment, so the edges of parts or bodies that share no vertices are found too.
pub fn meeting_ends(ends: &[EdgeEnd]) -> HashMap<i32, Vec<(i32, f64)>> {
    let mut order: Vec<&EdgeEnd> = ends.iter().collect();
    order.sort_by(|a, b| a.point.x.total_cmp(&b.point.x));
    let reach = RELATIVE_TOLERANCE * ends.iter().map(|e| e.length).fold(0.0, f64::max);
    let mut meeting: HashMap<i32, Vec<(i32, f64)>> = HashMap::new();
    for (i, a) in order.iter().enumerate() {
        for b in &order[i + 1..] {
            if b.point.x - a.point.x > reach {
                break;
            }
            if a.edge == b.edge
                || a.point.distance(b.point) > RELATIVE_TOLERANCE * a.length.min(b.length)
            {
                continue;
            }
            // Straight on, the directions leaving the two edges are opposite.
            let bend = (a.leaving.dot(-b.leaving)).clamp(-1.0, 1.0).acos();
            meeting.entry(a.edge).or_default().push((b.edge, bend));
            meeting.entry(b.edge).or_default().push((a.edge, bend));
        }
    }
    meeting
}

/// For each edge made of segments the other edges lying on it along a stretch, see
/// [`overlap`]: where parts meet that do not share their edges. Edges whose boxes overlap
/// are compared segment by segment.
pub fn edge_twins(edges: Vec<(i32, Vec<[DVec3; 2]>)>) -> HashMap<i32, Vec<i32>> {
    struct Segments {
        tag: i32,
        segments: Vec<[DVec3; 2]>,
        min: DVec3,
        max: DVec3,
    }
    let mut lines: Vec<Segments> = (edges.into_iter())
        .filter(|(_, segments)| !segments.is_empty())
        .map(|(tag, segments)| {
            let longest = (segments.iter())
                .map(|s| s[0].distance(s[1]))
                .fold(0.0, f64::max);
            let margin = DVec3::splat(RELATIVE_TOLERANCE * longest);
            let points = segments.iter().flatten();
            let min = points.clone().fold(DVec3::INFINITY, |m, p| m.min(*p)) - margin;
            let max = points.fold(DVec3::NEG_INFINITY, |m, p| m.max(*p)) + margin;
            Segments {
                tag,
                segments,
                min,
                max,
            }
        })
        .collect();
    lines.sort_by(|a, b| a.min.x.total_cmp(&b.min.x));
    let mut twins: HashMap<i32, Vec<i32>> = HashMap::new();
    for (i, a) in lines.iter().enumerate() {
        for b in &lines[i + 1..] {
            if b.min.x > a.max.x {
                break;
            }
            if b.min.cmpgt(a.max).any() || a.min.cmpgt(b.max).any() {
                continue;
            }
            let on_each_other = (a.segments.iter())
                .any(|s| (b.segments.iter()).any(|t| overlap(s[0], s[1], t[0], t[1])));
            if on_each_other {
                twins.entry(a.tag).or_default().push(b.tag);
                twins.entry(b.tag).or_default().push(a.tag);
            }
        }
    }
    twins
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segments_on_each_other() {
        let v = |x: f64, y: f64| DVec3::new(x, y, 0.0);
        // The same segment, and either way round.
        assert!(overlap(v(0.0, 0.0), v(1.0, 0.0), v(0.0, 0.0), v(1.0, 0.0)));
        assert!(overlap(v(0.0, 0.0), v(1.0, 0.0), v(1.0, 0.0), v(0.0, 0.0)));
        // Divided differently: a short piece inside a long one, or shifted half way.
        assert!(overlap(v(0.0, 0.0), v(4.0, 0.0), v(1.0, 0.0), v(2.0, 0.0)));
        assert!(overlap(v(1.0, 0.0), v(2.0, 0.0), v(0.0, 0.0), v(4.0, 0.0)));
        assert!(overlap(
            v(0.0, 0.0),
            v(1.0, 0.0),
            v(0.5, 0.01),
            v(1.5, 0.01)
        ));
        // Touching at an end only, side by side, or crossing.
        assert!(!overlap(v(0.0, 0.0), v(1.0, 0.0), v(1.0, 0.0), v(2.0, 0.0)));
        assert!(!overlap(v(0.0, 0.0), v(1.0, 0.0), v(0.0, 0.2), v(1.0, 0.2)));
        assert!(!overlap(
            v(0.0, 0.0),
            v(1.0, 0.0),
            v(0.5, -0.5),
            v(0.5, 0.5)
        ));
    }

    #[test]
    fn edges_of_touching_parts_are_twins_and_meet() {
        // Edge 1 of one part along x from 0 to 10, edge 2 of a neighbour from 4 to 10 in
        // two pieces, edge 3 from 10 on and edge 4 beside it: only 1 and 2 lie on each other.
        let v = |x: f64, y: f64| DVec3::new(x, y, 0.0);
        let polylines = [
            (1, vec![v(0.0, 0.0), v(10.0, 0.0)]),
            (2, vec![v(4.0, 0.0), v(7.0, 0.0), v(10.0, 0.0)]),
            (3, vec![v(10.0, 1e-3), v(20.0, 0.0)]),
            (4, vec![v(0.0, 1.0), v(10.0, 1.0)]),
        ];
        let twins = edge_twins(
            (polylines.iter())
                .map(|(tag, p)| (*tag, p.windows(2).map(|w| [w[0], w[1]]).collect()))
                .collect(),
        );
        assert_eq!(twins.get(&1), Some(&vec![2]));
        assert_eq!(twins.get(&2), Some(&vec![1]));
        assert_eq!((twins.get(&3), twins.get(&4)), (None, None));
        // At x = 10 edge 3 goes on straight from 1 and 2, though its end lies a little off.
        let ends: Vec<EdgeEnd> = (polylines.iter())
            .flat_map(|(tag, p)| EdgeEnd::of_polyline(*tag, p))
            .collect();
        let meeting = meeting_ends(&ends);
        let bend = |a: i32, b: i32| {
            (meeting.get(&a).into_iter().flatten())
                .find(|(e, _)| *e == b)
                .map(|(_, bend)| *bend)
        };
        assert!(bend(1, 3).is_some_and(|b| b < 1e-3));
        assert!(bend(3, 2).is_some_and(|b| b < 1e-3));
        assert!(bend(1, 2).is_some_and(|b| (b - std::f64::consts::PI).abs() < 1e-3));
        assert_eq!(bend(1, 4), None);
        assert_eq!(meeting.get(&4), None);
    }
}
