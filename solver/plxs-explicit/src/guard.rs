//! The watch over the stable time step under large deformations: an element's highest
//! eigenfrequency goes with the inverse of its shortest dimension, so instead of the
//! frequencies themselves (a power iteration per element) the solver follows the shortest
//! distance between two nodes of every element. While no element's has changed by more than
//! 2.5 % in length ([`GUARD_RATIO`]) since the last exact check, the time step stands. The card and the
//! processor compute the measure the same way: differences in double precision, the squared
//! length in single precision.

use rayon::prelude::*;

/// The squared lengths may change by this factor either way (2.5 % in length) before the
/// frequencies are computed again; the stable time step keeps 10 % in hand.
pub const GUARD_RATIO: f32 = 0.95;

/// The elements the watch follows: for element `e` its nodes `offsets[e]..offsets[e + 1]`,
/// each with its three DOFs and its coordinates.
#[derive(Clone, Debug, Default)]
pub struct Guard {
    pub offsets: Vec<u32>,
    pub dofs: Vec<u32>,
    pub coordinates: Vec<f64>,
}

impl Guard {
    pub fn len(&self) -> usize {
        self.offsets.len().saturating_sub(1)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Adds an element of the nodes at `coordinates` with the DOFs `dofs` (three per node).
    pub fn push(&mut self, coordinates: &[[f64; 3]], dofs: &[u32]) {
        if self.offsets.is_empty() {
            self.offsets.push(0);
        }
        for (x, d) in coordinates.iter().zip(dofs.chunks_exact(3)) {
            self.coordinates.extend_from_slice(x);
            self.dofs.extend_from_slice(d);
        }
        self.offsets.push((self.dofs.len() / 3) as u32);
    }

    /// The shortest squared distance between two nodes of every element at `u`.
    pub fn lengths(&self, u: &[f64]) -> Vec<f32> {
        (0..self.len())
            .into_par_iter()
            .map(|e| self.length(e, u))
            .collect()
    }

    fn length(&self, e: usize, u: &[f64]) -> f32 {
        let (first, last) = (self.offsets[e] as usize, self.offsets[e + 1] as usize);
        let mut shortest = f32::MAX;
        for a in first..last {
            for b in a + 1..last {
                let mut l = 0.0_f32;
                for k in 0..3 {
                    let d = (self.coordinates[3 * b + k] - self.coordinates[3 * a + k])
                        + (u[self.dofs[3 * b + k] as usize] - u[self.dofs[3 * a + k] as usize]);
                    let d = d as f32;
                    l += d * d;
                }
                shortest = shortest.min(l);
            }
        }
        shortest
    }

    /// The largest angle by which a line from an element's first node to one of its other
    /// nodes has turned at `u` (radians).
    pub fn largest_turn(&self, u: &[f64]) -> f64 {
        (0..self.len())
            .into_par_iter()
            .map(|e| {
                let (a, last) = (self.offsets[e] as usize, self.offsets[e + 1] as usize);
                let mut largest = 0.0_f64;
                for b in a + 1..last {
                    let mut r = [0.0; 3];
                    let mut c = [0.0; 3];
                    for k in 0..3 {
                        r[k] = self.coordinates[3 * b + k] - self.coordinates[3 * a + k];
                        c[k] = r[k] + u[self.dofs[3 * b + k] as usize]
                            - u[self.dofs[3 * a + k] as usize];
                    }
                    let cross = [
                        r[1] * c[2] - r[2] * c[1],
                        r[2] * c[0] - r[0] * c[2],
                        r[0] * c[1] - r[1] * c[0],
                    ];
                    let sine =
                        (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
                    let cosine = r[0] * c[0] + r[1] * c[1] + r[2] * c[2];
                    let angle = sine.atan2(cosine);
                    if angle.is_finite() {
                        largest = largest.max(angle);
                    }
                }
                largest
            })
            .reduce(|| 0.0, f64::max)
    }

    /// How far the elements have moved from `reference` (their [`Self::lengths`] at the
    /// last check): the smallest ratio of the squared lengths, now to then or then to now.
    pub fn departure(&self, reference: &[f32], u: &[f64]) -> f32 {
        (0..self.len())
            .into_par_iter()
            .map(|e| ratio(self.length(e, u), reference[e]))
            .reduce(|| 1.0, f32::min)
    }
}

/// `min(now / then, then / now)`, zero for a collapsed element or anything not finite.
pub(crate) fn ratio(now: f32, then: f32) -> f32 {
    if then <= 0.0 {
        return 1.0;
    }
    if now.is_nan() || now <= 0.0 || !now.is_finite() {
        return 0.0;
    }
    (now / then).min(then / now)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_squeezed_element_departs() {
        let mut guard = Guard::default();
        let x = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 2.0, 0.0]];
        guard.push(&x, &[0, 1, 2, 3, 4, 5, 6, 7, 8]);
        let rest = vec![0.0; 9];
        let reference = guard.lengths(&rest);
        assert_eq!(reference, vec![1.0]);
        // A rigid shift does not count.
        let shifted = vec![0.3, -0.2, 0.1, 0.3, -0.2, 0.1, 0.3, -0.2, 0.1];
        assert!((guard.departure(&reference, &shifted) - 1.0).abs() < 1e-6);
        // The short edge squeezed to 0.9: 0.81 of its squared length.
        let mut squeezed = rest.clone();
        squeezed[3] = -0.1;
        assert!((guard.departure(&reference, &squeezed) - 0.81).abs() < 1e-6);
        // Stretched to 1.1: 1 / 1.21.
        squeezed[3] = 0.1;
        assert!((guard.departure(&reference, &squeezed) - 1.0 / 1.21).abs() < 1e-6);
        // Turned by 30° about z (small-strain displacements of the rotation).
        let (c, s) = (30.0_f64.to_radians().cos(), 30.0_f64.to_radians().sin());
        let mut turned = vec![0.0; 9];
        for (n, p) in x.iter().enumerate() {
            turned[3 * n] = c * p[0] - s * p[1] - p[0];
            turned[3 * n + 1] = s * p[0] + c * p[1] - p[1];
        }
        assert!((guard.largest_turn(&turned) - 30.0_f64.to_radians()).abs() < 1e-12);
        assert_eq!(guard.largest_turn(&shifted), 0.0);
    }
}
