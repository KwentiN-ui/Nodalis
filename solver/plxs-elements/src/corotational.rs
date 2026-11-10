//! The corotational two-node beam: the general beam [`U1`](crate::u1) under large
//! displacements and rotations (`NLGEOM`), for ropes and other slender beams whose
//! stiffness values are given rather than taken from a solid cross-section.
//!
//! The element follows Crisfield ("A consistent co-rotational formulation for non-linear,
//! three-dimensional, beam-elements", CMAME 81, 1990): a frame moves with the element, its
//! first axis along the chord between the current nodes and the other two from the mean of
//! the turned nodal triads. In this frame the deformation is small: the extension of the
//! chord and the rotations of the two nodal triads against the frame. The strain energy is
//! that of the linear U1 element on these local values, so the element is exact for rigid
//! motions of any size and reduces to U1 for small displacements.
//!
//! Nodal rotations are total rotation vectors (`R = exp([θ]×)`, Rodrigues' formula), as the
//! rotation nodes of rigid bodies and joints hold them. The internal forces and the tangent
//! are the gradient and the Hessian of the strain energy with respect to these twelve
//! values, computed exactly with hyper-dual numbers (Fike and Alonso, 2011) rather than by
//! hand: the tangent is symmetric and consistent, the work of moments is the work on the
//! rotation vector.

use crate::u1::{self, U1Section};

/// A hyper-dual number `v + a ε₁ + b ε₂ + ab ε₁ε₂` with `ε₁² = ε₂² = 0`: one evaluation
/// carries the value, two first derivatives and the mixed second derivative.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Hd {
    v: f64,
    a: f64,
    b: f64,
    ab: f64,
}

impl Hd {
    const fn constant(v: f64) -> Self {
        Self {
            v,
            a: 0.0,
            b: 0.0,
            ab: 0.0,
        }
    }

    /// `f(self)` from `f`, `f'` and `f''` at the value.
    fn chain(self, f: f64, f1: f64, f2: f64) -> Self {
        Self {
            v: f,
            a: self.a * f1,
            b: self.b * f1,
            ab: self.ab * f1 + self.a * self.b * f2,
        }
    }

    fn sqrt(self) -> Self {
        let s = self.v.sqrt();
        self.chain(s, 0.5 / s, -0.25 / (self.v * s))
    }

    fn sin(self) -> Self {
        let (s, c) = self.v.sin_cos();
        self.chain(s, c, -s)
    }

    fn cos(self) -> Self {
        let (s, c) = self.v.sin_cos();
        self.chain(c, -s, -c)
    }

    fn recip(self) -> Self {
        let r = 1.0 / self.v;
        self.chain(r, -r * r, 2.0 * r * r * r)
    }
}

impl std::ops::Add for Hd {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self {
            v: self.v + o.v,
            a: self.a + o.a,
            b: self.b + o.b,
            ab: self.ab + o.ab,
        }
    }
}

impl std::ops::Sub for Hd {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self {
            v: self.v - o.v,
            a: self.a - o.a,
            b: self.b - o.b,
            ab: self.ab - o.ab,
        }
    }
}

impl std::ops::Neg for Hd {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            v: -self.v,
            a: -self.a,
            b: -self.b,
            ab: -self.ab,
        }
    }
}

impl std::ops::Mul for Hd {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        Self {
            v: self.v * o.v,
            a: self.a * o.v + self.v * o.a,
            b: self.b * o.v + self.v * o.b,
            ab: self.ab * o.v + self.a * o.b + self.b * o.a + self.v * o.ab,
        }
    }
}

impl std::ops::Mul<f64> for Hd {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self {
            v: self.v * s,
            a: self.a * s,
            b: self.b * s,
            ab: self.ab * s,
        }
    }
}

impl std::ops::Div for Hd {
    type Output = Self;
    #[allow(clippy::suspicious_arithmetic_impl)]
    fn div(self, o: Self) -> Self {
        self * o.recip()
    }
}

type V = [Hd; 3];

fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: V, s: Hd) -> V {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn dot(a: V, b: V) -> Hd {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn constant(v: [f64; 3]) -> V {
    v.map(Hd::constant)
}

/// `R(θ) v` with Rodrigues' formula; the coefficients by their series for small angles,
/// where the square root would have no derivative.
fn rotate(theta: V, v: V) -> V {
    let phi2 = dot(theta, theta);
    let (sinc, cosc) = if phi2.v < 1e-6 {
        let p4 = phi2 * phi2;
        let p6 = p4 * phi2;
        (
            Hd::constant(1.0) - phi2 * (1.0 / 6.0) + p4 * (1.0 / 120.0) - p6 * (1.0 / 5040.0),
            Hd::constant(0.5) - phi2 * (1.0 / 24.0) + p4 * (1.0 / 720.0) - p6 * (1.0 / 40320.0),
        )
    } else {
        let phi = phi2.sqrt();
        (phi.sin() / phi, (Hd::constant(1.0) - phi.cos()) / phi2)
    };
    let tv = cross(theta, v);
    add(add(v, scale(tv, sinc)), scale(cross(theta, tv), cosc))
}

/// The rotation vector of a small rotation from its matrix entries `m[i][j] = rᵢ · tⱼ`:
/// the axial vector `sin φ n` of the skew part, scaled to `φ n` by the series of
/// `asin(s) / s` (exact to 1e-8 up to 0.6 rad).
fn small_rotation(m: [[Hd; 3]; 3]) -> V {
    let w = [
        (m[2][1] - m[1][2]) * 0.5,
        (m[0][2] - m[2][0]) * 0.5,
        (m[1][0] - m[0][1]) * 0.5,
    ];
    let s2 = dot(w, w);
    let s4 = s2 * s2;
    let factor = Hd::constant(1.0)
        + s2 * (1.0 / 6.0)
        + s4 * (3.0 / 40.0)
        + s4 * s2 * (5.0 / 112.0)
        + s4 * s4 * (35.0 / 1152.0);
    scale(w, factor)
}

/// A corotational beam between two nodes, set up in its reference configuration.
#[derive(Clone, Debug, PartialEq)]
pub struct CorotationalBeam {
    /// Reference coordinates of the two nodes.
    x0: [[f64; 3]; 2],
    length: f64,
    /// Reference axes: the tangent, the 1- and the 2-direction.
    axes: [[f64; 3]; 3],
    /// The stiffness of the linear element in its own axes on the seven local values
    /// `[extension, θa (3), θb (3)]`, row-major 7 × 7.
    local: [f64; 49],
}

impl CorotationalBeam {
    /// The beam from `xa` to `xb` with the section vector `n1`; `None` like
    /// [`u1::stiffness`].
    pub fn new(xa: [f64; 3], xb: [f64; 3], n1: [f64; 3], section: &U1Section) -> Option<Self> {
        let d = [xb[0] - xa[0], xb[1] - xa[1], xb[2] - xa[2]];
        let length = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let (t, e1, e2) = u1::axes(xa, xb, n1)?;
        // U1 in its own axes: tangent x, 1-direction y, 2-direction z.
        let k = u1::stiffness([0.0; 3], [length, 0.0, 0.0], [0.0, 1.0, 0.0], section)?;
        // The local values in the twelve DOFs of the element in its own axes.
        const AT: [usize; 7] = [6, 3, 4, 5, 9, 10, 11];
        let mut local = [0.0; 49];
        for (i, &a) in AT.iter().enumerate() {
            for (j, &b) in AT.iter().enumerate() {
                local[i * 7 + j] = k[a * 12 + b];
            }
        }
        Some(Self {
            x0: [xa, xb],
            length,
            axes: [t, e1, e2],
            local,
        })
    }

    pub fn length(&self) -> f64 {
        self.length
    }

    /// The seven local values `[extension, θa, θb]` of the DOFs `q = [ua, θa, ub, θb]`.
    fn deformation(&self, q: &[Hd; 12]) -> [Hd; 7] {
        let x =
            |n: usize| -> V { std::array::from_fn(|i| Hd::constant(self.x0[n][i]) + q[6 * n + i]) };
        let theta = |n: usize| -> V { std::array::from_fn(|i| q[6 * n + 3 + i]) };
        let d = sub(x(1), x(0));
        let l2 = dot(d, d);
        let l = l2.sqrt();
        let length = Hd::constant(self.length);
        // l - L without cancellation.
        let extension = (l2 - length * length) / (l + length);
        let r1 = scale(d, l.recip());
        let triads: [[V; 3]; 2] = std::array::from_fn(|n| {
            let th = theta(n);
            self.axes.map(|axis| rotate(th, constant(axis)))
        });
        let p = scale(add(triads[0][1], triads[1][1]), Hd::constant(0.5));
        let r3 = cross(r1, p);
        let r3 = scale(r3, dot(r3, r3).sqrt().recip());
        let r2 = cross(r3, r1);
        let frame = [r1, r2, r3];
        let local = |n: usize| -> V {
            let m: [[Hd; 3]; 3] =
                std::array::from_fn(|i| std::array::from_fn(|j| dot(frame[i], triads[n][j])));
            small_rotation(m)
        };
        let (a, b) = (local(0), local(1));
        [extension, a[0], a[1], a[2], b[0], b[1], b[2]]
    }

    fn energy(&self, q: &[Hd; 12]) -> Hd {
        let d = self.deformation(q);
        let mut e = Hd::default();
        for (i, di) in d.iter().enumerate() {
            let mut row = Hd::default();
            for (j, dj) in d.iter().enumerate() {
                row = row + *dj * self.local[i * 7 + j];
            }
            e = e + *di * row;
        }
        e * 0.5
    }

    /// The gradient and the Hessian (row-major 12 × 12) of `f` at `q`.
    fn derivatives(q: &[f64; 12], f: impl Fn(&[Hd; 12]) -> Hd) -> ([f64; 12], [f64; 144]) {
        let mut g = [0.0; 12];
        let mut h = [0.0; 144];
        for i in 0..12 {
            for j in i..12 {
                let mut x = q.map(Hd::constant);
                x[i].a = 1.0;
                x[j].b = 1.0;
                let e = f(&x);
                h[i * 12 + j] = e.ab;
                h[j * 12 + i] = e.ab;
                if i == j {
                    g[i] = e.a;
                }
            }
        }
        (g, h)
    }

    /// The internal forces (and moments, conjugate to the rotation vectors) for the DOFs
    /// `q = [ua, θa, ub, θb]`, with the tangent stiffness when `k` is given.
    pub fn internal(&self, q: &[f64; 12], f: &mut [f64; 12], k: Option<&mut [f64; 144]>) {
        match k {
            Some(k) => {
                let (g, h) = Self::derivatives(q, |x| self.energy(x));
                *f = g;
                *k = h;
            }
            None => {
                for (i, fi) in f.iter_mut().enumerate() {
                    let mut x = q.map(Hd::constant);
                    x[i].a = 1.0;
                    *fi = self.energy(&x).a;
                }
            }
        }
    }

    /// The local forces `[N, Ma, Mb]` (the derivatives of the strain energy by the seven
    /// local values) for the DOFs `q`.
    pub fn local_forces(&self, q: &[f64; 12]) -> [f64; 7] {
        let d = self.deformation(&q.map(Hd::constant)).map(|x| x.v);
        std::array::from_fn(|i| (0..7).map(|j| self.local[i * 7 + j] * d[j]).sum())
    }

    /// The current axis (unit, from the first node to the second) and the extension
    /// strain `(l - L) / L` for the DOFs `q`;
    /// `linear`: the reference axis and the small strain, as in a step without NLGEOM.
    pub fn axis_and_strain(&self, q: &[f64; 12], linear: bool) -> ([f64; 3], f64) {
        let t = self.axes[0];
        if linear {
            let along: f64 = (0..3).map(|i| (q[6 + i] - q[i]) * t[i]).sum();
            return (t, along / self.length);
        }
        let d: [f64; 3] = std::array::from_fn(|i| self.x0[1][i] + q[6 + i] - self.x0[0][i] - q[i]);
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let axis = if l > 0.0 { d.map(|x| x / l) } else { t };
        (axis, (l - self.length) / self.length)
    }

    /// The section forces at the two nodes for the DOFs `q`: the normal force, the torque
    /// and the bending moments `[N, T, M1, M2]` (about the beam's current 1- and
    /// 2-direction), each as the part of the beam beyond the node (towards the second
    /// node) carries it, so that the elements at a node agree.
    /// `linear`: of the small deformation theory, as in a step without NLGEOM (where the
    /// large deformation's shortening of the chord by the deflection would pull at the
    /// beam).
    pub fn section_forces(&self, q: &[f64; 12], linear: bool) -> [[f64; 4]; 2] {
        let f = if linear {
            // The local values linearised about the reference configuration.
            let mut d = [0.0; 7];
            for (i, &qi) in q.iter().enumerate() {
                let mut x = [Hd::constant(0.0); 12];
                x[i].a = 1.0;
                let column = self.deformation(&x);
                for (dk, c) in d.iter_mut().zip(column) {
                    *dk += c.a * qi;
                }
            }
            std::array::from_fn(|i| (0..7).map(|j| self.local[i * 7 + j] * d[j]).sum())
        } else {
            self.local_forces(q)
        };
        [[f[0], -f[1], -f[2], -f[3]], [f[0], f[4], f[5], f[6]]]
    }

    /// The initial stress stiffness in the reference configuration for the local forces
    /// the DOFs `q` (a small, linear solution) cause: the second derivative of the work of
    /// these forces on the local values, what linear buckling scales.
    pub fn initial_stress_stiffness(&self, q: &[f64; 12]) -> [f64; 144] {
        let forces = self.local_forces(q);
        let (_, h) = Self::derivatives(&[0.0; 12], |x| {
            let d = self.deformation(x);
            let mut w = Hd::default();
            for (di, fi) in d.iter().zip(forces) {
                w = w + *di * fi;
            }
            w
        });
        h
    }

    /// The mass matrix (row-major 12 × 12): the consistent mass of a bar for the
    /// translations, which stays right under rotations of any size, and the rotary inertia
    /// of the section lumped at the nodes. `i11` is the integral of the 1-coordinate
    /// squared, `i22` that of the 2-coordinate.
    pub fn mass(&self, density: f64, area: f64, i11: f64, i22: f64) -> [f64; 144] {
        let mut m = [0.0; 144];
        let m_line = density * area * self.length;
        for a in 0..2 {
            for b in 0..2 {
                let share = if a == b { 2.0 } else { 1.0 } * m_line / 6.0;
                for i in 0..3 {
                    m[(6 * a + i) * 12 + 6 * b + i] = share;
                }
            }
        }
        // Rotary inertia per node about the tangent, the 1- and the 2-axis.
        let half = density * self.length / 2.0;
        let inertia = [(i11 + i22) * half, i22 * half, i11 * half];
        for n in 0..2 {
            for i in 0..3 {
                for j in 0..3 {
                    let v: f64 = (0..3)
                        .map(|k| self.axes[k][i] * inertia[k] * self.axes[k][j])
                        .sum();
                    m[(6 * n + 3 + i) * 12 + 6 * n + 3 + j] = v;
                }
            }
        }
        m
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn section() -> U1Section {
        U1Section {
            area: 100.0,
            i11: 800.0,
            i22: 1200.0,
            shear_factor: 5.0 / 6.0,
            young: 210000.0,
            poisson: 0.3,
        }
    }

    const XA: [f64; 3] = [1.0, 2.0, 3.0];
    const XB: [f64; 3] = [41.0, 32.0, 3.0];
    const N1: [f64; 3] = [0.0, 0.0, 1.0];

    fn largest(k: &[f64]) -> f64 {
        k.iter().fold(0.0_f64, |m, v| m.max(v.abs()))
    }

    #[test]
    fn tangent_at_rest_is_the_linear_element() {
        let beam = CorotationalBeam::new(XA, XB, N1, &section()).unwrap();
        let linear = u1::stiffness(XA, XB, N1, &section()).unwrap();
        let mut f = [0.0; 12];
        let mut k = [0.0; 144];
        beam.internal(&[0.0; 12], &mut f, Some(&mut k));
        assert!(f.iter().all(|v| v.abs() < 1e-9));
        let scale = largest(&linear);
        for i in 0..144 {
            assert!(
                (k[i] - linear[i]).abs() < 1e-9 * scale,
                "{i}: {} vs {}",
                k[i],
                linear[i]
            );
        }
    }

    #[test]
    fn large_rigid_motions_leave_no_force() {
        let beam = CorotationalBeam::new(XA, XB, N1, &section()).unwrap();
        // Turn by 1.7 rad about a skew axis through the origin and shift.
        let theta = [0.6, -1.1, 1.1];
        let turn = |x: [f64; 3]| -> [f64; 3] { rotate(constant(theta), constant(x)).map(|v| v.v) };
        let shift = [5.0, -3.0, 2.0];
        let mut q = [0.0; 12];
        for (n, x) in [XA, XB].into_iter().enumerate() {
            let y = turn(x);
            for i in 0..3 {
                q[6 * n + i] = y[i] + shift[i] - x[i];
                q[6 * n + 3 + i] = theta[i];
            }
        }
        let mut f = [0.0; 12];
        let mut k = [0.0; 144];
        beam.internal(&q, &mut f, Some(&mut k));
        let scale = largest(&u1::stiffness(XA, XB, N1, &section()).unwrap());
        for v in f {
            assert!(v.abs() < 1e-10 * scale, "{f:?}");
        }
        // The tangent there is the turned linear stiffness (no stress, no stress stiffness)
        // up to the factor T(θ) of the rotation vector: the translations agree.
        let linear = u1::stiffness(XA, XB, N1, &section()).unwrap();
        let r: [[f64; 3]; 3] = std::array::from_fn(|j| {
            let mut e = [0.0; 3];
            e[j] = 1.0;
            turn(e)
        });
        // K_uu of the turned element = R K_uu Rᵀ for the a-a block.
        for i in 0..3 {
            for j in 0..3 {
                let mut v = 0.0;
                for p in 0..3 {
                    for s in 0..3 {
                        v += r[p][i] * linear[p * 12 + s] * r[s][j];
                    }
                }
                assert!((k[i * 12 + j] - v).abs() < 1e-8 * scale, "{i} {j}");
            }
        }
    }

    #[test]
    fn forces_are_the_gradient_and_the_tangent_their_derivative() {
        let beam = CorotationalBeam::new(XA, XB, N1, &section()).unwrap();
        let q = [
            0.1, -0.3, 0.2, 0.05, -0.02, 0.3, 1.0, 0.4, -0.5, -0.1, 0.04, 0.2,
        ];
        let mut f = [0.0; 12];
        let mut k = [0.0; 144];
        beam.internal(&q, &mut f, Some(&mut k));
        let mut only = [0.0; 12];
        beam.internal(&q, &mut only, None);
        assert_eq!(f, only);
        let h = 1e-6;
        for j in 0..12 {
            let (mut qp, mut qm) = (q, q);
            qp[j] += h;
            qm[j] -= h;
            let (mut fp, mut fm) = ([0.0; 12], [0.0; 12]);
            beam.internal(&qp, &mut fp, None);
            beam.internal(&qm, &mut fm, None);
            for i in 0..12 {
                let fd = (fp[i] - fm[i]) / (2.0 * h);
                assert!(
                    (fd - k[i * 12 + j]).abs() < 1e-5 * largest(&k),
                    "{i} {j}: {fd} vs {}",
                    k[i * 12 + j]
                );
            }
        }
    }

    #[test]
    fn tension_stiffens_the_transverse_motion_by_n_over_l() {
        // A bar along x pulled by N: moving the end node sideways (rotations held) costs
        // the bending stiffness plus N / L.
        let beam = CorotationalBeam::new([0.0; 3], [100.0, 0.0, 0.0], N1, &section()).unwrap();
        let mut q = [0.0; 12];
        q[6] = 0.01;
        let n = beam.local_forces(&q)[0];
        assert!((n - 210000.0 * 100.0 * 0.01 / 100.0).abs() < 1e-6 * n);
        let ks = beam.initial_stress_stiffness(&q);
        // Transverse translation of node b along y: N / L (rotations are separate DOFs).
        assert!(
            (ks[7 * 12 + 7] - n / 100.0).abs() < 1e-9 * n,
            "{}",
            ks[7 * 12 + 7]
        );
        assert!((ks[12 + 7] + n / 100.0).abs() < 1e-9 * n);
    }

    #[test]
    fn mass_has_the_beam_mass_in_every_direction() {
        let beam = CorotationalBeam::new(XA, XB, N1, &section()).unwrap();
        let m = beam.mass(7.85e-9, 100.0, 800.0, 1200.0);
        let total = 7.85e-9 * 100.0 * beam.length();
        for i in 0..3 {
            let sum: f64 = (0..2)
                .flat_map(|a| (0..2).map(move |b| (a, b)))
                .map(|(a, b)| m[(6 * a + i) * 12 + 6 * b + i])
                .sum();
            assert!((sum - total).abs() < 1e-12 * total);
        }
    }
}
