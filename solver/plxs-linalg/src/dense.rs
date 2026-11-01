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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_two_by_two_matrix() {
        let (values, vectors) = symmetric_eigen(2, &[2.0, 1.0, 1.0, 2.0]);
        assert!((values[0] - 1.0).abs() < 1e-14 && (values[1] - 3.0).abs() < 1e-14);
        let v = &vectors[1];
        assert!((v[0].abs() - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-14);
        assert!((v[0] - v[1]).abs() < 1e-14);
    }
}
