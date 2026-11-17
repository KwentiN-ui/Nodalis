//! Closed triangle surfaces as solids: which points lie inside (by the parity of the
//! crossings of a ray along x), and the signed distance field of a rigid body on a lattice
//! of its own, negative inside.
//!
//! The distance is exact within a band of a few lattice spacings around the surface (each
//! triangle visits the lattice points of its box widened by the band) and clamped to the
//! band beyond it, which is all the walls need: the sign anywhere, the distance and the
//! normal near the surface. The sign comes from the crossings of one ray per lattice row,
//! so the surface must be closed; it need not be consistently oriented.
//!
//! Sources: closest point on a triangle after Ericson, *Real-Time Collision Detection*
//! (2005), section 5.1.5; ray parity as in any point-in-polyhedron test.

use plxs_core::particles::TriangleSurface;

use crate::math::{Vec3, add, cross, dot, scale, sub};

/// The crossings of rays along +x with a closed surface, one ray per row of a lattice in
/// y and z.
pub struct ParityRows {
    origin: [f64; 2],
    spacing: f64,
    dims: [usize; 2],
    /// Sorted x of the crossings of each row (`j + dims[0] * k`).
    crossings: Vec<Vec<f64>>,
}

/// A small shift of the rays off the lattice, so that a ray does not pass exactly through
/// an edge or a corner of the triangles (CAD surfaces sit on round coordinates).
const JITTER: [f64; 2] = [1.234_567e-7, 2.345_678e-7];

impl ParityRows {
    /// The rays at `y = origin[0] + j spacing`, `z = origin[1] + k spacing`.
    pub fn new(
        surface: &TriangleSurface,
        origin: [f64; 2],
        spacing: f64,
        dims: [usize; 2],
    ) -> Self {
        let mut crossings = vec![Vec::new(); dims[0] * dims[1]];
        let jitter = [JITTER[0] * spacing, JITTER[1] * spacing];
        for t in &surface.triangles {
            let p = t.map(|i| surface.points[i as usize]);
            let lo = |a: usize| p[0][a].min(p[1][a]).min(p[2][a]);
            let hi = |a: usize| p[0][a].max(p[1][a]).max(p[2][a]);
            let range = |a: usize| {
                let b = a - 1;
                let first = ((lo(a) - origin[b] - jitter[b]) / spacing).ceil().max(0.0) as usize;
                let last = ((hi(a) - origin[b] - jitter[b]) / spacing).floor();
                if last < 0.0 {
                    return first..first;
                }
                first..(last as usize + 1).min(dims[b])
            };
            for k in range(2) {
                for j in range(1) {
                    let y = origin[0] + j as f64 * spacing + jitter[0];
                    let z = origin[1] + k as f64 * spacing + jitter[1];
                    if let Some(x) = ray_x(&p, y, z) {
                        crossings[j + dims[0] * k].push(x);
                    }
                }
            }
        }
        for row in &mut crossings {
            row.sort_by(f64::total_cmp);
        }
        Self {
            origin,
            spacing,
            dims,
            crossings,
        }
    }

    /// Whether the point at `x` on row `(j, k)` lies inside.
    pub fn inside(&self, j: usize, k: usize, x: f64) -> bool {
        let row = &self.crossings[j + self.dims[0] * k];
        row.partition_point(|&c| c < x) % 2 == 1
    }

    pub fn row_of(&self, y: f64, z: f64) -> Option<(usize, usize)> {
        let j = ((y - self.origin[0]) / self.spacing).round();
        let k = ((z - self.origin[1]) / self.spacing).round();
        (j >= 0.0 && k >= 0.0 && (j as usize) < self.dims[0] && (k as usize) < self.dims[1])
            .then_some((j as usize, k as usize))
    }
}

/// Where the ray along x through `(y, z)` crosses the triangle, if it does.
fn ray_x(p: &[Vec3; 3], y: f64, z: f64) -> Option<f64> {
    // Barycentric coordinates in the yz projection.
    let (y0, z0) = (p[0][1], p[0][2]);
    let (y1, z1) = (p[1][1], p[1][2]);
    let (y2, z2) = (p[2][1], p[2][2]);
    let area = (y1 - y0) * (z2 - z0) - (y2 - y0) * (z1 - z0);
    if area.abs() < 1e-300 {
        return None;
    }
    let a = ((y1 - y) * (z2 - z) - (y2 - y) * (z1 - z)) / area;
    let b = ((y2 - y) * (z0 - z) - (y0 - y) * (z2 - z)) / area;
    let c = 1.0 - a - b;
    (a >= 0.0 && b >= 0.0 && c >= 0.0).then(|| a * p[0][0] + b * p[1][0] + c * p[2][0])
}

/// The closest point to `q` on the triangle `p`.
fn closest_on_triangle(q: Vec3, p: &[Vec3; 3]) -> Vec3 {
    let (a, b, c) = (p[0], p[1], p[2]);
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(q, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub(q, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return add(a, scale(ab, d1 / (d1 - d3)));
    }
    let cp = sub(q, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return add(a, scale(ac, d2 / (d2 - d6)));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return add(b, scale(sub(c, b), w));
    }
    let denom = 1.0 / (va + vb + vc);
    add(a, add(scale(ab, vb * denom), scale(ac, vc * denom)))
}

/// The signed distance to a closed surface on a lattice, negative inside.
#[derive(Clone, Debug)]
pub struct DistanceField {
    pub origin: Vec3,
    pub spacing: f64,
    pub dims: [usize; 3],
    values: Vec<f32>,
    /// The distance values are clamped to beyond the band.
    pub band: f64,
}

impl DistanceField {
    /// The field of `surface` on a lattice of `spacing` over its box widened by `margin`,
    /// exact within `band` of the surface. The lattice points lie half a spacing off the
    /// points `anchor + k spacing` (the nodes of a grid of that spacing): those then sit in
    /// the middle of the lattice cells, not on their corners, where the gradient of the
    /// interpolation jumps between the cells around and the normal at a node would hang on
    /// the last bit of its position.
    pub fn new(
        surface: &TriangleSurface,
        spacing: f64,
        margin: f64,
        band: f64,
        anchor: Vec3,
    ) -> Self {
        let (lo, hi) = surface.bounds().unwrap_or(([0.0; 3], [0.0; 3]));
        let origin = std::array::from_fn(|a| {
            let k = ((lo[a] - margin - anchor[a]) / spacing).floor();
            anchor[a] + (k - 0.5) * spacing
        });
        let dims: [usize; 3] =
            std::array::from_fn(|a| ((hi[a] + margin - origin[a]) / spacing).ceil() as usize + 1);
        let index = |i: usize, j: usize, k: usize| i + dims[0] * (j + dims[1] * k);
        let mut distance = vec![band; dims[0] * dims[1] * dims[2]];
        let reach = (band / spacing).ceil() as isize;
        for t in &surface.triangles {
            let p = t.map(|i| surface.points[i as usize]);
            let range = |a: usize| {
                let lo = p[0][a].min(p[1][a]).min(p[2][a]);
                let hi = p[0][a].max(p[1][a]).max(p[2][a]);
                let first = ((lo - origin[a]) / spacing).floor() as isize - reach;
                let last = ((hi - origin[a]) / spacing).ceil() as isize + reach;
                (first.max(0) as usize)..((last + 1).max(0) as usize).min(dims[a])
            };
            let (rx, ry, rz) = (range(0), range(1), range(2));
            for k in rz {
                for j in ry.clone() {
                    for i in rx.clone() {
                        let q = [
                            origin[0] + i as f64 * spacing,
                            origin[1] + j as f64 * spacing,
                            origin[2] + k as f64 * spacing,
                        ];
                        let c = closest_on_triangle(q, &p);
                        let d = crate::math::norm(sub(q, c));
                        let slot = &mut distance[index(i, j, k)];
                        if d < *slot {
                            *slot = d;
                        }
                    }
                }
            }
        }
        let rows = ParityRows::new(surface, [origin[1], origin[2]], spacing, [dims[1], dims[2]]);
        let mut values = vec![0.0f32; distance.len()];
        for k in 0..dims[2] {
            for j in 0..dims[1] {
                for i in 0..dims[0] {
                    let x = origin[0] + i as f64 * spacing;
                    let at = index(i, j, k);
                    let sign = if rows.inside(j, k, x) { -1.0 } else { 1.0 };
                    values[at] = (sign * distance[at]) as f32;
                }
            }
        }
        Self {
            origin,
            spacing,
            dims,
            values,
            band,
        }
    }

    /// The distance and its gradient at `x` by trilinear interpolation, `None` outside the
    /// lattice (far outside the body).
    pub fn sample(&self, x: Vec3) -> Option<(f64, Vec3)> {
        let mut base = [0usize; 3];
        let mut f = [0.0; 3];
        for a in 0..3 {
            let u = (x[a] - self.origin[a]) / self.spacing;
            if !(u >= 0.0 && u < (self.dims[a] - 1) as f64) {
                return None;
            }
            base[a] = u.floor() as usize;
            f[a] = u - base[a] as f64;
        }
        let at = |i: usize, j: usize, k: usize| {
            f64::from(
                self.values
                    [base[0] + i + self.dims[0] * (base[1] + j + self.dims[1] * (base[2] + k))],
            )
        };
        let mut value = 0.0;
        let mut gradient = [0.0; 3];
        for k in 0..2 {
            for j in 0..2 {
                for i in 0..2 {
                    let w = [
                        if i == 1 { f[0] } else { 1.0 - f[0] },
                        if j == 1 { f[1] } else { 1.0 - f[1] },
                        if k == 1 { f[2] } else { 1.0 - f[2] },
                    ];
                    let dw = [
                        if i == 1 { 1.0 } else { -1.0 },
                        if j == 1 { 1.0 } else { -1.0 },
                        if k == 1 { 1.0 } else { -1.0 },
                    ];
                    let v = at(i, j, k);
                    value += w[0] * w[1] * w[2] * v;
                    gradient[0] += dw[0] * w[1] * w[2] * v / self.spacing;
                    gradient[1] += w[0] * dw[1] * w[2] * v / self.spacing;
                    gradient[2] += w[0] * w[1] * dw[2] * v / self.spacing;
                }
            }
        }
        Some((value, gradient))
    }

    /// The distances at the lattice points, `i + dims[0] (j + dims[1] k)`.
    pub fn values(&self) -> &[f32] {
        &self.values
    }

    /// The smallest and largest corner of the lattice.
    pub fn bounds(&self) -> (Vec3, Vec3) {
        let hi = std::array::from_fn(|a| self.origin[a] + (self.dims[a] - 1) as f64 * self.spacing);
        (self.origin, hi)
    }
}

/// The volume enclosed by a closed surface, positive when its triangles turn
/// counterclockwise seen from outside.
pub fn enclosed_volume(surface: &TriangleSurface) -> f64 {
    (surface.triangles.iter())
        .map(|t| {
            let p = t.map(|i| surface.points[i as usize]);
            dot(p[0], cross(p[1], p[2])) / 6.0
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_distance_and_sign() {
        let s = TriangleSurface::cuboid([0.0; 3], [1.0, 2.0, 3.0]);
        assert!((enclosed_volume(&s) - 6.0).abs() < 1e-12);
        let field = DistanceField::new(&s, 0.1, 0.5, 0.4, [0.0; 3]);
        let (d, g) = field.sample([0.5, 1.0, 0.15]).unwrap();
        assert!((d + 0.15).abs() < 1e-6, "{d}");
        assert!((g[2] + 1.0).abs() < 1e-6, "{g:?}");
        let (d, _) = field.sample([1.2, 1.0, 1.5]).unwrap();
        assert!((d - 0.2).abs() < 1e-6, "{d}");
        // Deep inside: clamped to the band, still negative.
        let (d, _) = field.sample([0.5, 1.0, 1.5]).unwrap();
        assert!(d < 0.0);
    }

    #[test]
    fn cylinder_inside() {
        let s = TriangleSurface::cylinder([0.0; 3], [0.0, 0.0, 1.0], 0.5, 64);
        let v = enclosed_volume(&s);
        assert!((v - std::f64::consts::PI * 0.25).abs() < 0.01, "{v}");
        let rows = ParityRows::new(&s, [-1.0, -1.0], 0.05, [41, 41]);
        let (j, k) = rows.row_of(0.0, 0.5).unwrap();
        assert!(rows.inside(j, k, 0.0));
        assert!(rows.inside(j, k, 0.45));
        assert!(!rows.inside(j, k, 0.55));
    }
}
