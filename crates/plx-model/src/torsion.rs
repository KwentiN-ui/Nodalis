//! The torsion constant `J` of a profile drawn as polygons: Saint-Venant torsion, solved for
//! Prandtl's stress function `φ` with `∇²φ = -2` in the material, `φ = 0` on the outline and
//! `φ` constant inside each hole; `J = 2 ∫ φ dA` over the outline, holes included.
//!
//! Minimising `∫ ½ |∇φ|² - 2 φ dA` over the area inside the outline, with one value for all
//! of a hole, gives exactly that: a hole has no gradient and its load `2 A_hole` goes to its
//! one value, which is Bredt's condition of a closed cell. Open thin walls (`b t³ / 3`),
//! solid sections and closed cells come out of the same solve.
//!
//! The polygons are put on a grid of biquadratic elements (9 nodes) whose lines run along
//! every edge parallel to the axes, so that walls parallel to the axes are exact and a thin wall gets the
//! parabola of its stress function across its thickness exactly; slanted and round edges
//! follow the grid in steps. An element belongs to the material, a hole or the outside that
//! most of its area lies in.

/// Elements along the larger side of the profile, besides the lines through the corners.
const ALONG: f64 = 80.0;

/// At most this many elements; a profile with more corners gets coarser elements in between.
const MOST: usize = 8_000;

/// The torsion constant of the region inside `loops[0]` and outside the holes `loops[1..]`;
/// 0 for no area.
pub fn torsion_constant(loops: &[Vec<[f64; 2]>]) -> f64 {
    let Some(outline) = loops.first() else {
        return 0.0;
    };
    let (mut min, mut max) = ([f64::MAX; 2], [f64::MIN; 2]);
    for &[u, v] in outline {
        min = [min[0].min(u), min[1].min(v)];
        max = [max[0].max(u), max[1].max(v)];
    }
    let span = (max[0] - min[0]).max(max[1] - min[1]);
    if !span.is_finite() || span <= 0.0 {
        return 0.0;
    }
    // Lines along the edges parallel to the axes and around the whole: a slanted or round
    // edge is followed in steps anyway.
    let corners = |axis: usize| -> Vec<f64> {
        let mut c = vec![min[axis], max[axis]];
        for points in loops {
            for (k, p) in points.iter().enumerate() {
                let q = points[(k + 1) % points.len()];
                if (p[1 - axis] - q[1 - axis]).abs() > 1e-9 * span
                    && (p[axis] - q[axis]).abs() <= 1e-9 * span
                {
                    c.push(p[axis]);
                }
            }
        }
        c.sort_by(f64::total_cmp);
        c.dedup_by(|a, b| (*a - *b).abs() <= 1e-9 * span);
        c
    };
    let (cu, cv) = (corners(0), corners(1));
    let mut along = ALONG;
    let (mut xs, mut ys) = (lines(&cu, span / along), lines(&cv, span / along));
    while (xs.len() - 1) * (ys.len() - 1) > MOST && along > 4.0 {
        along /= 1.5;
        (xs, ys) = (lines(&cu, span / along), lines(&cv, span / along));
    }
    solve(loops, &xs, &ys)
}

/// Grid lines through the corners `c`, each interval split into elements no longer than `h`,
/// at least two, so that a thin wall has interior nodes across it.
fn lines(c: &[f64], h: f64) -> Vec<f64> {
    let mut out = vec![c[0]];
    for pair in c.windows(2) {
        let length = pair[1] - pair[0];
        let n = ((length / h).ceil() as usize).max(2);
        out.extend((1..=n).map(|k| pair[0] + length * k as f64 / n as f64));
    }
    out
}

/// Where a point lies: `None` outside the outline, `Some(0)` in the material, `Some(k)` in
/// hole `k`.
fn region(point: [f64; 2], loops: &[Vec<[f64; 2]>]) -> Option<usize> {
    if !crate::cross_section::inside(point, &loops[0]) {
        return None;
    }
    Some(
        (loops.iter().enumerate().skip(1))
            .find(|(_, hole)| crate::cross_section::inside(point, hole))
            .map_or(0, |(k, _)| k),
    )
}

/// One-dimensional quadratic element of length `l`: stiffness, mass and load of a unit
/// source, the middle node second.
fn element_1d(l: f64) -> ([[f64; 3]; 3], [[f64; 3]; 3], [f64; 3]) {
    let k =
        [[7.0, -8.0, 1.0], [-8.0, 16.0, -8.0], [1.0, -8.0, 7.0]].map(|r| r.map(|v| v / (3.0 * l)));
    let m = [[4.0, 2.0, -1.0], [2.0, 16.0, 2.0], [-1.0, 2.0, 4.0]].map(|r| r.map(|v| v * l / 30.0));
    let f = [1.0, 4.0, 1.0].map(|v| v * l / 6.0);
    (k, m, f)
}

fn solve(loops: &[Vec<[f64; 2]>], xs: &[f64], ys: &[f64]) -> f64 {
    let (ex, ey) = (xs.len() - 1, ys.len() - 1);
    let (nx, ny) = (2 * ex + 1, 2 * ey + 1);
    let node = |i: usize, j: usize| j * nx + i;
    // The region of each element: the one most of its area lies in, by points spread over
    // it, so that a slanted or round edge cuts its steps about half and half.
    const SAMPLES: usize = 4;
    let cells: Vec<Option<usize>> = (0..ey)
        .flat_map(|j| (0..ex).map(move |i| (i, j)))
        .map(|(i, j)| {
            let mut votes = vec![0usize; loops.len() + 1];
            for s in 0..SAMPLES {
                for t in 0..SAMPLES {
                    let at = |lo: f64, hi: f64, k: usize| {
                        lo + (hi - lo) * (k as f64 + 0.5) / SAMPLES as f64
                    };
                    let point = [at(xs[i], xs[i + 1], s), at(ys[j], ys[j + 1], t)];
                    votes[region(point, loops).map_or(0, |r| r + 1)] += 1;
                }
            }
            let best = (0..votes.len()).max_by_key(|&k| votes[k]).unwrap_or(0);
            best.checked_sub(1)
        })
        .collect();
    // What each node is: fixed at 0 next to the outside, else the one value of a hole it
    // touches, else a value of its own.
    #[derive(Clone, Copy, PartialEq)]
    enum Kind {
        Unset,
        Zero,
        Hole(usize),
        Material,
    }
    let mut kind = vec![Kind::Unset; nx * ny];
    // Beyond the grid is outside.
    for j in 0..ny {
        for i in 0..nx {
            if i == 0 || j == 0 || i == nx - 1 || j == ny - 1 {
                kind[node(i, j)] = Kind::Zero;
            }
        }
    }
    for j in 0..ey {
        for i in 0..ex {
            let cell = cells[j * ex + i];
            for b in 0..3 {
                for a in 0..3 {
                    let n = node(2 * i + a, 2 * j + b);
                    kind[n] = match (kind[n], cell) {
                        (_, None) | (Kind::Zero, _) => Kind::Zero,
                        (Kind::Unset, Some(0)) => Kind::Material,
                        (_, Some(h)) if h > 0 => Kind::Hole(h),
                        (current, _) => current,
                    };
                }
            }
        }
    }
    let holes = loops.len() - 1;
    let mut dof = vec![None::<usize>; nx * ny];
    let mut count = holes;
    for (n, k) in kind.iter().enumerate() {
        dof[n] = match *k {
            Kind::Unset | Kind::Zero => None,
            Kind::Material => {
                count += 1;
                Some(count - 1)
            }
            Kind::Hole(h) => Some(h - 1),
        };
    }
    if count == 0 {
        return 0.0;
    }
    // Assembly, by rows of sorted columns.
    let mut rows: Vec<Vec<(usize, f64)>> = vec![Vec::new(); count];
    let mut load = vec![0.0; count];
    for j in 0..ey {
        for i in 0..ex {
            if cells[j * ex + i].is_none() {
                continue;
            }
            let (kx, mx, fx) = element_1d(xs[i + 1] - xs[i]);
            let (ky, my, fy) = element_1d(ys[j + 1] - ys[j]);
            let local: Vec<(usize, usize, Option<usize>)> = (0..3)
                .flat_map(|b| (0..3).map(move |a| (a, b)))
                .map(|(a, b)| (a, b, dof[node(2 * i + a, 2 * j + b)]))
                .collect();
            for &(a, b, p) in &local {
                let Some(p) = p else { continue };
                load[p] += 2.0 * fx[a] * fy[b];
                for &(c, d, q) in &local {
                    let Some(q) = q else { continue };
                    let value = kx[a][c] * my[b][d] + mx[a][c] * ky[b][d];
                    rows[p].push((q, value));
                }
            }
        }
    }
    let rows: Vec<Vec<(usize, f64)>> = rows
        .into_iter()
        .map(|mut row| {
            row.sort_by_key(|&(q, _)| q);
            let mut merged: Vec<(usize, f64)> = Vec::with_capacity(row.len());
            for (q, v) in row {
                match merged.last_mut() {
                    Some(last) if last.0 == q => last.1 += v,
                    _ => merged.push((q, v)),
                }
            }
            merged
        })
        .collect();
    let phi = conjugate_gradient(&rows, &load);
    phi.iter().zip(&load).map(|(p, f)| p * f).sum()
}

/// Solves the symmetric positive definite system by conjugate gradients with the diagonal as
/// preconditioner.
fn conjugate_gradient(rows: &[Vec<(usize, f64)>], b: &[f64]) -> Vec<f64> {
    let n = b.len();
    let times = |x: &[f64]| -> Vec<f64> {
        rows.iter()
            .map(|row| row.iter().map(|&(q, v)| v * x[q]).sum())
            .collect()
    };
    let diagonal: Vec<f64> = (rows.iter().enumerate())
        .map(|(p, row)| {
            let d = row.iter().find(|&&(q, _)| q == p).map_or(0.0, |&(_, v)| v);
            if d > 0.0 { 1.0 / d } else { 1.0 }
        })
        .collect();
    let dot = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>();
    let mut x = vec![0.0; n];
    let mut r = b.to_vec();
    let mut z: Vec<f64> = r.iter().zip(&diagonal).map(|(r, d)| r * d).collect();
    let mut p = z.clone();
    let mut rz = dot(&r, &z);
    let start = dot(b, b).sqrt();
    if start == 0.0 {
        return x;
    }
    for _ in 0..10 * n.max(10) {
        let q = times(&p);
        let alpha = rz / dot(&p, &q);
        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * q[i];
        }
        if dot(&r, &r).sqrt() <= 1e-9 * start {
            break;
        }
        for i in 0..n {
            z[i] = r[i] * diagonal[i];
        }
        let next = dot(&r, &z);
        let beta = next / rz;
        rz = next;
        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(u0: f64, v0: f64, u1: f64, v1: f64) -> Vec<[f64; 2]> {
        vec![[u0, v0], [u1, v0], [u1, v1], [u0, v1]]
    }

    /// Saint-Venant's series for a rectangle `a` by `b`, `a >= b`.
    fn rectangle(a: f64, b: f64) -> f64 {
        let mut sum = 0.0;
        for k in (1..200).step_by(2) {
            let n = k as f64;
            sum += (n * std::f64::consts::PI * a / (2.0 * b)).tanh() / n.powi(5);
        }
        a * b.powi(3) / 3.0 * (1.0 - 192.0 / std::f64::consts::PI.powi(5) * b / a * sum)
    }

    fn close(value: f64, expected: f64, tolerance: f64) {
        assert!(
            (value - expected).abs() <= tolerance * expected,
            "{value} instead of {expected}"
        );
    }

    #[test]
    fn rectangles_match_saint_venant() {
        close(
            torsion_constant(&[rect(0.0, 0.0, 1.0, 1.0)]),
            rectangle(1.0, 1.0),
            1e-3,
        );
        close(
            torsion_constant(&[rect(0.0, 0.0, 30.0, 10.0)]),
            rectangle(30.0, 10.0),
            1e-3,
        );
        // A thin strip: b t³ / 3 less its ends.
        close(
            torsion_constant(&[rect(0.0, 0.0, 100.0, 1.0)]),
            rectangle(100.0, 1.0),
            2e-3,
        );
    }

    #[test]
    fn a_thin_closed_box_follows_bredt() {
        // 100 by 50 outside, walls 1 thick: Bredt 4 A² / ∮ ds / t over the mid-line.
        let loops = [rect(0.0, 0.0, 100.0, 50.0), rect(1.0, 1.0, 99.0, 49.0)];
        let (a, b): (f64, f64) = (99.0, 49.0);
        let bredt = 4.0 * (a * b).powi(2) / (2.0 * (a + b));
        close(torsion_constant(&loops), bredt, 0.02);
    }

    #[test]
    fn a_disc_and_a_tube_are_polar() {
        let circle = |r: f64| -> Vec<[f64; 2]> {
            (0..96)
                .map(|k| {
                    let t = std::f64::consts::TAU * k as f64 / 96.0;
                    [r * t.cos(), r * t.sin()]
                })
                .collect()
        };
        let pi = std::f64::consts::PI;
        close(torsion_constant(&[circle(10.0)]), pi * 1e4 / 2.0, 0.03);
        let tube = torsion_constant(&[circle(10.0), circle(8.0)]);
        close(tube, pi * (1e4 - 8f64.powi(4)) / 2.0, 0.05);
    }

    #[test]
    fn an_open_section_is_far_softer_than_its_polar_moment() {
        // An I of 200 by 100 with walls 10 thick: about the sum of b t³ / 3.
        let i = vec![
            [0.0, 0.0],
            [100.0, 0.0],
            [100.0, 10.0],
            [55.0, 10.0],
            [55.0, 190.0],
            [100.0, 190.0],
            [100.0, 200.0],
            [0.0, 200.0],
            [0.0, 190.0],
            [45.0, 190.0],
            [45.0, 10.0],
            [0.0, 10.0],
        ];
        let thin = (2.0 * 100.0 + 180.0) * 1000.0 / 3.0;
        let j = torsion_constant(&[i]);
        // The joints of web and flanges add a little to the strips.
        assert!(j > 0.9 * thin && j < 1.25 * thin, "{j} against {thin}");
    }

    #[test]
    fn a_box_girder_with_overhanging_flanges() {
        // Flanges 240 by 10 over webs 8 thick, 15 in from the edges, 630 high: Bredt's
        // 4 A² / ∮ ds / t over the mid-line of the cell, plus a little for the overhangs and
        // the walls' own torsion.
        let outline = vec![
            [0.0, 0.0],
            [240.0, 0.0],
            [240.0, 10.0],
            [225.0, 10.0],
            [225.0, 620.0],
            [240.0, 620.0],
            [240.0, 630.0],
            [0.0, 630.0],
            [0.0, 620.0],
            [15.0, 620.0],
            [15.0, 10.0],
            [0.0, 10.0],
        ];
        let hole = rect(23.0, 10.0, 217.0, 620.0);
        let (a, b): (f64, f64) = (202.0, 620.0);
        let bredt = 4.0 * (a * b).powi(2) / (2.0 * a / 10.0 + 2.0 * b / 8.0);
        let j = torsion_constant(&[outline, hole]);
        assert!(j > bredt && j < 1.03 * bredt, "{j} against {bredt}");
    }
}
