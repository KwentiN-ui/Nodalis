//! Continuous collision detection: the first moment a slave point, moving on a straight
//! line over an increment, meets a master face that also moves on straight lines.
//!
//! The springs of an increment only look for master faces near where their points were at
//! the start of it. A body that reaches another within one increment (a projectile hitting
//! a 0.4 mm can wall, a lever swinging into a wall with a long time step) passes through it
//! unseen, or ends so deep that the next increment cannot recover. The time of impact tells
//! where to cut the increment: before it the motion is free, after it the contact is near.
//!
//! The narrow phase is the additive CCD of Li, Kaufman and Jiang ("Codimensional Incremental
//! Potential Contact", ACM Transactions on Graphics 40(4), 2021, algorithm 1): conservative
//! advancement along the straight paths by the current distance over a bound of the
//! relative speed, `t += 0.9 d(t) / l_p`, until the distance falls below a fraction `s` of
//! the start distance. It never steps past the contact (the distance cannot shrink faster
//! than `l_p`) and needs no root finding. Everything is in `f64`. A quadratic face is taken
//! as the flat triangles between its corner, midside and centre points (for the time of
//! impact, not for the contact itself); an edge of a plane model as its straight segments.
//! The broad phase registers the boxes the faces sweep over the increment in a
//! [`FaceGrid`] and asks it with the box each point sweeps.
//!
//! Implemented from the paper; no code of other contact solvers was consulted.

use crate::geometry::{ContactFace, FaceGrid, cross, dot, norm, scale, sub};

/// Stop fraction `s` of the additive CCD: the time of impact is reported when the distance
/// has fallen to this fraction of its start (the paper takes 0.1 for a line search; here it
/// marks the touch).
pub const SEPARATION: f64 = 0.01;

/// Advancement steps of one point and triangle at most: a path that grazes a triangle
/// creeps along it with steps of the distance. Beyond this the reached time is reported
/// (conservative: never later than the touch).
const MAX_STEPS: usize = 500;

/// Flat pieces of a face: its points as weights of the face nodes, and the triangles
/// between them (a segment repeats its last point).
pub struct Pieces {
    /// Per point: (local node, weight) pairs.
    pub points: Vec<Vec<(usize, f64)>>,
    pub triangles: Vec<[usize; 3]>,
}

/// The flat pieces of a face: a linear triangle is itself, a linear quadrilateral four
/// triangles about its centre, a quadratic triangle the four between its corners and
/// midside nodes, a quadratic quadrilateral eight about its centre. An edge of a plane
/// model is one segment (two when quadratic).
pub fn pieces(face: &ContactFace) -> Pieces {
    let node = |i: usize| vec![(i, 1.0)];
    let k = face.corners;
    let m = face.nodes.len();
    let mut points: Vec<Vec<(usize, f64)>> = (0..m).map(node).collect();
    let triangles = match (k, face.quadratic) {
        (2, false) => vec![[0, 1, 1]],
        (2, true) => vec![[0, 2, 2], [2, 1, 1]],
        (3, false) => vec![[0, 1, 2]],
        (3, true) => vec![[0, 3, 5], [3, 1, 4], [5, 4, 2], [3, 4, 5]],
        (4, quadratic) => {
            // The centre as the face's own interpolation (a quadratic face bulges there).
            let mut n = vec![0.0; m];
            let mut dn = vec![[0.0; 2]; m];
            plxs_elements::shape::face_shape(4, quadratic, [0.0, 0.0], &mut n, &mut dn);
            points.push(
                n.iter()
                    .enumerate()
                    .filter(|(_, w)| **w != 0.0)
                    .map(|(i, w)| (i, *w))
                    .collect(),
            );
            let c = m;
            if quadratic {
                (0..4)
                    .flat_map(|e| [[e, 4 + e, c], [4 + e, (e + 1) % 4, c]])
                    .collect()
            } else {
                (0..4).map(|e| [e, (e + 1) % 4, c]).collect()
            }
        }
        _ => Vec::new(),
    };
    Pieces { points, triangles }
}

impl Pieces {
    /// The points of the pieces of `face` in a configuration.
    pub fn positions(&self, face: &ContactFace, config: &[[f64; 3]]) -> Vec<[f64; 3]> {
        self.points
            .iter()
            .map(|weights| {
                let mut x = [0.0; 3];
                for &(i, w) in weights {
                    let c = config[face.nodes[i] as usize];
                    for d in 0..3 {
                        x[d] += w * c[d];
                    }
                }
                x
            })
            .collect()
    }
}

/// Distance of `p` from the segment `a`–`b`.
pub fn point_segment_distance(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = sub(b, a);
    let length = dot(ab, ab);
    let t = if length > 0.0 {
        (dot(sub(p, a), ab) / length).clamp(0.0, 1.0)
    } else {
        0.0
    };
    norm(sub(
        p,
        [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]],
    ))
}

/// Distance of `p` from the triangle `a`, `b`, `c` (a degenerate one is taken as its
/// longest side): the closest point by the triangle's Voronoi regions (Ericson,
/// "Real-Time Collision Detection", 5.1.5).
pub fn point_triangle_distance(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let n = cross(ab, ac);
    let area2 = dot(n, n);
    let scale_sq = dot(ab, ab).max(dot(ac, ac));
    if area2 <= 1e-24 * scale_sq * scale_sq {
        return point_segment_distance(p, a, b)
            .min(point_segment_distance(p, a, c))
            .min(point_segment_distance(p, b, c));
    }
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return norm(ap);
    }
    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return norm(bp);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return point_segment_distance(p, a, b);
    }
    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return norm(cp);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return point_segment_distance(p, a, c);
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return point_segment_distance(p, b, c);
    }
    // Inside: the distance from the plane.
    dot(ap, n).abs() / area2.sqrt()
}

/// The side of the triangle's plane `p` lies on, as a signed distance (a segment of a plane
/// model: the side of its line in the x-y plane).
pub fn signed_distance(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    let mut n = cross(sub(b, a), sub(c, a));
    if dot(n, n) == 0.0 {
        n = cross(sub(b, a), [0.0, 0.0, 1.0]);
    }
    let length = norm(n);
    if length == 0.0 {
        return 0.0;
    }
    dot(sub(p, a), scale(n, 1.0 / length))
}

/// Additive CCD of a point against a triangle (Li, Kaufman, Jiang 2021, algorithm 1 with a
/// zero thickness): `x` the point and the triangle's corners at the start, `p` their
/// displacements over the increment. Returns the fraction of the increment (at most
/// `t_max`) at which the distance has fallen to `s` times its start, conservatively, or
/// `None` when it does not within `t_max`.
pub fn additive_ccd(x: [[f64; 3]; 4], p: [[f64; 3]; 4], s: f64, t_max: f64) -> Option<f64> {
    // Only the relative motion counts: remove the mean.
    let mut mean = [0.0; 3];
    for v in &p {
        for d in 0..3 {
            mean[d] += 0.25 * v[d];
        }
    }
    let p = p.map(|v| sub(v, mean));
    let lengths = p.map(norm);
    // Bound of how fast the distance can change: the point's speed plus the fastest corner.
    let l_p = lengths[0] + lengths[1].max(lengths[2]).max(lengths[3]);
    if l_p <= 0.0 {
        return None;
    }
    let distance = |x: &[[f64; 3]; 4]| point_triangle_distance(x[0], x[1], x[2], x[3]);
    let mut x = x;
    let d0 = distance(&x);
    let g = s * d0;
    let mut t = 0.0;
    let mut t_l = (1.0 - s) * d0 / l_p;
    for _ in 0..MAX_STEPS {
        for (xi, pi) in x.iter_mut().zip(&p) {
            for d in 0..3 {
                xi[d] += t_l * pi[d];
            }
        }
        let d = distance(&x);
        if t > 0.0 && d < g {
            return Some(t);
        }
        t += t_l;
        if t > t_max {
            return None;
        }
        t_l = 0.9 * d / l_p;
    }
    Some(t)
}

/// A swept box: the box of `start` and `end` points.
pub fn swept_box(start: &[[f64; 3]], end: &[[f64; 3]]) -> ([f64; 3], [f64; 3]) {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for x in start.iter().chain(end) {
        for d in 0..3 {
            lo[d] = lo[d].min(x[d]);
            hi[d] = hi[d].max(x[d]);
        }
    }
    (lo, hi)
}

/// The grid of the boxes the master faces sweep: cells of the typical edge length, larger
/// when the faces move far (a face is registered in at most about ten cells per axis).
pub fn swept_grid(boxes: &[([f64; 3], [f64; 3])], typical: f64) -> FaceGrid {
    let sweep = boxes
        .iter()
        .map(|(lo, hi)| (0..3).map(|d| hi[d] - lo[d]).fold(0.0, f64::max))
        .fold(0.0, f64::max);
    FaceGrid::of_boxes(boxes, typical.max(sweep / 10.0), 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    const TRIANGLE: [[f64; 3]; 3] = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];

    #[test]
    fn distances_to_a_triangle() {
        let [a, b, c] = TRIANGLE;
        // Above the inside, beside an edge, beyond a corner.
        assert!((point_triangle_distance([0.2, 0.2, 0.5], a, b, c) - 0.5).abs() < 1e-15);
        assert!((point_triangle_distance([0.5, -1.0, 0.0], a, b, c) - 1.0).abs() < 1e-15);
        let corner = point_triangle_distance([2.0, -1.0, 0.0], a, b, c);
        assert!((corner - 2f64.sqrt()).abs() < 1e-15);
        // Beyond the hypotenuse.
        let d = point_triangle_distance([1.0, 1.0, 0.0], a, b, c);
        assert!((d - 0.5f64.sqrt()).abs() < 1e-15);
        // A degenerate triangle is a segment.
        assert!((point_triangle_distance([0.5, 1.0, 0.0], a, b, b) - 1.0).abs() < 1e-15);
    }

    #[test]
    fn a_point_flying_through_a_triangle_is_caught_before_it() {
        let [a, b, c] = TRIANGLE;
        // From 1 above to 3 below in one increment: it touches at a quarter.
        let x = [[0.2, 0.2, 1.0], a, b, c];
        let p = [[0.0, 0.0, -4.0], [0.0; 3], [0.0; 3], [0.0; 3]];
        let t = additive_ccd(x, p, SEPARATION, 1.0).expect("caught");
        assert!(t <= 0.25 && t > 0.25 * (1.0 - 2.0 * SEPARATION), "{t}");
        // The same relative motion with the triangle flying up.
        let p = [[0.0; 3], [0.0, 0.0, 4.0], [0.0, 0.0, 4.0], [0.0, 0.0, 4.0]];
        let moving = additive_ccd(x, p, SEPARATION, 1.0).expect("caught");
        assert!((moving - t).abs() < 1e-12);
    }

    #[test]
    fn a_point_passing_beside_or_stopping_short_is_not_caught() {
        let [a, b, c] = TRIANGLE;
        // Passes beside the triangle.
        let x = [[2.0, 2.0, 1.0], a, b, c];
        let p = [[0.0, 0.0, -4.0], [0.0; 3], [0.0; 3], [0.0; 3]];
        assert_eq!(additive_ccd(x, p, SEPARATION, 1.0), None);
        // Stops half way.
        let x = [[0.2, 0.2, 1.0], a, b, c];
        let p = [[0.0, 0.0, -0.5], [0.0; 3], [0.0; 3], [0.0; 3]];
        assert_eq!(additive_ccd(x, p, SEPARATION, 1.0), None);
        // Slides along it.
        let x = [[0.2, 0.2, 0.1], a, b, c];
        let p = [[0.5, 0.0, 0.0], [0.0; 3], [0.0; 3], [0.0; 3]];
        assert_eq!(additive_ccd(x, p, SEPARATION, 1.0), None);
    }

    #[test]
    fn a_rotating_triangle_hits_a_point_it_sweeps() {
        // The triangle tips over (one corner goes down by 2): a point under its middle is hit
        // before the end, which the start and end positions alone would not show.
        let x = [
            [0.3, 0.3, -0.2],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        let p = [[0.0; 3], [0.0; 3], [0.0, 0.0, -2.0], [0.0, 0.0, -2.0]];
        let t = additive_ccd(x, p, SEPARATION, 1.0).expect("caught");
        // The plane passes the point when 0.6 * 2 t = 0.2.
        assert!(t <= 1.0 / 6.0 && t > 0.15, "{t}");
    }

    #[test]
    fn quadratic_faces_are_split_into_flat_triangles() {
        let quad8 = ContactFace {
            element: 0,
            element_id: 1,
            face_number: 1,
            nodes: (0..8).collect(),
            corners: 4,
            quadratic: true,
            planar: None,
        };
        let pieces = pieces(&quad8);
        assert_eq!(pieces.triangles.len(), 8);
        // The centre of a flat square from its corners and midside nodes.
        let coords = [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 2.0, 0.0],
            [0.0, 2.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [1.0, 2.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        let x = pieces.positions(&quad8, &coords);
        let centre = x[8];
        assert!(norm(sub(centre, [1.0, 1.0, 0.0])) < 1e-14, "{centre:?}");
        // The triangles cover the square: area 4.
        let area: f64 = pieces
            .triangles
            .iter()
            .map(|t| 0.5 * norm(cross(sub(x[t[1]], x[t[0]]), sub(x[t[2]], x[t[0]]))))
            .sum();
        assert!((area - 4.0).abs() < 1e-14);
    }
}
