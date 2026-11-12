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
}
