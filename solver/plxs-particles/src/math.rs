//! Small vectors and 3 × 3 matrices.

pub type Vec3 = [f64; 3];
pub type Mat3 = [[f64; 3]; 3];

pub const IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

pub fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn scale(a: Vec3, s: f64) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn norm(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}

pub fn mat_vec(m: &Mat3, v: Vec3) -> Vec3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}

pub fn mat_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut c = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            c[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    c
}

pub fn transpose(a: &Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| a[j][i]))
}

pub fn det(a: &Mat3) -> f64 {
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
}

pub fn trace(a: &Mat3) -> f64 {
    a[0][0] + a[1][1] + a[2][2]
}

/// The symmetric part.
pub fn sym(a: &Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| 0.5 * (a[i][j] + a[j][i])))
}

/// `Σ a_ij b_ij`.
pub fn contract(a: &Mat3, b: &Mat3) -> f64 {
    (0..3).map(|i| dot(a[i], b[i])).sum()
}

/// The deviator of a matrix.
pub fn deviator(a: &Mat3) -> Mat3 {
    let mean = trace(a) / 3.0;
    let mut d = *a;
    for (i, row) in d.iter_mut().enumerate() {
        row[i] -= mean;
    }
    d
}

/// The rotation by the angle `|axis|` about `axis` (Rodrigues).
pub fn rotation(axis: Vec3) -> Mat3 {
    let angle = norm(axis);
    if angle < 1e-300 {
        return IDENTITY;
    }
    let k = scale(axis, 1.0 / angle);
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    [
        [
            c + k[0] * k[0] * t,
            k[0] * k[1] * t - k[2] * s,
            k[0] * k[2] * t + k[1] * s,
        ],
        [
            k[1] * k[0] * t + k[2] * s,
            c + k[1] * k[1] * t,
            k[1] * k[2] * t - k[0] * s,
        ],
        [
            k[2] * k[0] * t - k[1] * s,
            k[2] * k[1] * t + k[0] * s,
            c + k[2] * k[2] * t,
        ],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotation_turns_x_into_y() {
        let r = rotation([0.0, 0.0, std::f64::consts::FRAC_PI_2]);
        let y = mat_vec(&r, [1.0, 0.0, 0.0]);
        assert!((y[0]).abs() < 1e-12 && (y[1] - 1.0).abs() < 1e-12);
        assert!((det(&r) - 1.0).abs() < 1e-12);
    }
}
