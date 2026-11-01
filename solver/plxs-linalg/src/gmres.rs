//! Restarted GMRES with right preconditioning (Saad, "Iterative Methods for Sparse Linear
//! Systems", 2nd ed., algorithm 9.5), used where a mostly symmetric system has a small
//! unsymmetric part: the factorized symmetric part is the preconditioner, so the iteration
//! only has to resolve the low-rank remainder (sliding friction of contact). No CalculiX
//! code was consulted.

use plxs_core::Result;

/// Solves `A x = b` with `apply(v, out) = A v` and `precondition(v)` replacing `v` by
/// `M⁻¹ v`. Starts from `x = M⁻¹ b`, stops at `‖b − A x‖ ≤ tolerance ‖b‖` or after
/// `max_iterations` operator applications (restart length `restart`) and returns the best
/// iterate with its relative residual.
pub fn gmres(
    apply: &dyn Fn(&[f64], &mut [f64]),
    precondition: &dyn Fn(&mut [f64]) -> Result<()>,
    b: &[f64],
    tolerance: f64,
    restart: usize,
    max_iterations: usize,
) -> Result<(Vec<f64>, f64)> {
    let n = b.len();
    let b_norm = norm(b);
    let mut x = b.to_vec();
    precondition(&mut x)?;
    if b_norm == 0.0 {
        return Ok((x, 0.0));
    }
    let mut work = vec![0.0; n];
    let mut used = 0;
    loop {
        apply(&x, &mut work);
        let r: Vec<f64> = b.iter().zip(&work).map(|(b, a)| b - a).collect();
        let beta = norm(&r);
        let relative = beta / b_norm;
        if relative <= tolerance || used >= max_iterations {
            return Ok((x, relative));
        }
        let m = restart.min(max_iterations - used).max(1);
        let mut v: Vec<Vec<f64>> = Vec::with_capacity(m + 1);
        v.push(r.iter().map(|r| r / beta).collect());
        let mut h = vec![vec![0.0; m]; m + 1];
        let (mut cs, mut sn) = (vec![0.0; m], vec![0.0; m]);
        let mut g = vec![0.0; m + 1];
        g[0] = beta;
        let mut k = 0;
        while k < m {
            let mut z = v[k].clone();
            precondition(&mut z)?;
            let mut w = vec![0.0; n];
            apply(&z, &mut w);
            used += 1;
            for (i, vi) in v.iter().enumerate() {
                let hij = dot(&w, vi);
                h[i][k] = hij;
                for (w, v) in w.iter_mut().zip(vi) {
                    *w -= hij * v;
                }
            }
            let hn = norm(&w);
            h[k + 1][k] = hn;
            for i in 0..k {
                let t = cs[i] * h[i][k] + sn[i] * h[i + 1][k];
                h[i + 1][k] = -sn[i] * h[i][k] + cs[i] * h[i + 1][k];
                h[i][k] = t;
            }
            let d = (h[k][k] * h[k][k] + h[k + 1][k] * h[k + 1][k]).sqrt();
            if d == 0.0 {
                break;
            }
            cs[k] = h[k][k] / d;
            sn[k] = h[k + 1][k] / d;
            h[k][k] = d;
            h[k + 1][k] = 0.0;
            g[k + 1] = -sn[k] * g[k];
            g[k] *= cs[k];
            k += 1;
            if g[k].abs() <= tolerance * b_norm || hn == 0.0 {
                break;
            }
            v.push(w.iter().map(|w| w / hn).collect());
        }
        // Back substitution H y = g, then x += M⁻¹ V y.
        let mut y = vec![0.0; k];
        for i in (0..k).rev() {
            let mut s = g[i];
            for j in i + 1..k {
                s -= h[i][j] * y[j];
            }
            y[i] = s / h[i][i];
        }
        let mut update = vec![0.0; n];
        for (yi, vi) in y.iter().zip(&v) {
            for (u, v) in update.iter_mut().zip(vi) {
                *u += yi * v;
            }
        }
        precondition(&mut update)?;
        for (x, u) in x.iter_mut().zip(&update) {
            *x += u;
        }
        if k == 0 {
            apply(&x, &mut work);
            let r = b
                .iter()
                .zip(&work)
                .map(|(b, a)| (b - a).powi(2))
                .sum::<f64>()
                .sqrt();
            return Ok((x, r / b_norm));
        }
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(a, b)| a * b).sum()
}

fn norm(a: &[f64]) -> f64 {
    dot(a, a).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solves_an_unsymmetric_system_preconditioned_by_its_symmetric_part() {
        // A = S + E with S = tridiag(-1, 4, -1) and a rank-one unsymmetric E.
        let n = 30;
        let apply_s = |v: &[f64], out: &mut [f64]| {
            for i in 0..n {
                out[i] = 4.0 * v[i]
                    - if i > 0 { v[i - 1] } else { 0.0 }
                    - if i + 1 < n { v[i + 1] } else { 0.0 };
            }
        };
        let apply = |v: &[f64], out: &mut [f64]| {
            apply_s(v, out);
            out[3] += 2.5 * v[17];
        };
        // Preconditioner: a few Jacobi-like sweeps would do; use the exact S solve by
        // Thomas algorithm.
        let precondition = |v: &mut [f64]| -> Result<()> {
            let mut c = vec![0.0; n];
            let mut d = v.to_vec();
            c[0] = -1.0 / 4.0;
            d[0] /= 4.0;
            for i in 1..n {
                let m = 4.0 + c[i - 1];
                c[i] = -1.0 / m;
                d[i] = (d[i] + d[i - 1]) / m;
            }
            for i in (0..n - 1).rev() {
                d[i] -= c[i] * d[i + 1];
            }
            v.copy_from_slice(&d);
            Ok(())
        };
        let b: Vec<f64> = (0..n).map(|i| (i as f64).sin() + 1.0).collect();
        let (x, relative) = gmres(&apply, &precondition, &b, 1e-12, 20, 50).unwrap();
        assert!(relative <= 1e-12, "{relative}");
        let mut ax = vec![0.0; n];
        apply(&x, &mut ax);
        for (a, b) in ax.iter().zip(&b) {
            assert!((a - b).abs() < 1e-10);
        }
    }
}
