//! Small dense eigenproblems (faer), for reduced models in modal coordinates.

/// Eigenvalues (ascending) and eigenvectors (columns, each of length `n`) of a real
/// symmetric `n × n` matrix given row major; only its lower triangle is read.
pub fn symmetric_eigen(n: usize, a: &[f64]) -> (Vec<f64>, Vec<Vec<f64>>) {
    assert_eq!(a.len(), n * n, "symmetric_eigen needs an n × n matrix");
    if n == 0 {
        return (Vec::new(), Vec::new());
    }
    let m = faer::Mat::<f64>::from_fn(n, n, |i, j| a[i * n + j]);
    let evd = m
        .self_adjoint_eigen(faer::Side::Lower)
        .expect("eigendecomposition of a dense symmetric matrix");
    let values = (0..n).map(|i| evd.S()[i]).collect();
    let vectors = (0..n)
        .map(|c| (0..n).map(|r| evd.U()[(r, c)]).collect())
        .collect();
    (values, vectors)
}

/// A complex number as `(re, im)`.
pub type Complex = (f64, f64);

/// Eigenvalues and right eigenvectors of a real general (non-symmetric) `n × n` matrix given
/// row major: the values in no particular order, the vectors as columns, one per value.
/// Complex values come in conjugate pairs.
pub fn general_eigen(n: usize, a: &[f64]) -> (Vec<Complex>, Vec<Vec<Complex>>) {
    assert_eq!(a.len(), n * n, "general_eigen needs an n × n matrix");
    if n == 0 {
        return (Vec::new(), Vec::new());
    }
    let m = faer::Mat::<f64>::from_fn(n, n, |i, j| a[i * n + j]);
    let evd = m
        .eigen()
        .expect("eigendecomposition of a dense general matrix");
    let values = (0..n)
        .map(|i| {
            let s = evd.S()[i];
            (s.re, s.im)
        })
        .collect();
    let vectors = (0..n)
        .map(|c| {
            (0..n)
                .map(|r| {
                    let u = evd.U()[(r, c)];
                    (u.re, u.im)
                })
                .collect()
        })
        .collect();
    (values, vectors)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_rotation_has_imaginary_eigenvalues() {
        // [[0, 1], [−1, 0]]: eigenvalues ±i with eigenvectors (1, ±i).
        let (values, vectors) = general_eigen(2, &[0.0, 1.0, -1.0, 0.0]);
        let mut im: Vec<f64> = values.iter().map(|v| v.1).collect();
        im.sort_by(f64::total_cmp);
        assert!((im[0] + 1.0).abs() < 1e-14 && (im[1] - 1.0).abs() < 1e-14);
        assert!(values.iter().all(|v| v.0.abs() < 1e-14));
        for (value, v) in values.iter().zip(&vectors) {
            // A v = s v, first row: v[1] = s v[0].
            let (sr, si) = *value;
            let expect = (sr * v[0].0 - si * v[0].1, sr * v[0].1 + si * v[0].0);
            assert!((v[1].0 - expect.0).abs() < 1e-12 && (v[1].1 - expect.1).abs() < 1e-12);
        }
    }

    #[test]
    fn a_two_by_two_matrix() {
        let (values, vectors) = symmetric_eigen(2, &[2.0, 1.0, 1.0, 2.0]);
        assert!((values[0] - 1.0).abs() < 1e-14 && (values[1] - 3.0).abs() < 1e-14);
        let v = &vectors[1];
        assert!((v[0].abs() - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-14);
        assert!((v[0] - v[1]).abs() < 1e-14);
    }
}
