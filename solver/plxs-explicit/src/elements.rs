//! The elements as the card computes them, matrix-free: for every element its DOFs, the
//! Cartesian derivatives of the shape functions at its integration points (constant in the
//! total Lagrangian description), its elastic matrix, its bulk viscosity and, for the
//! uniform-strain hexahedron, its hourglass control. Elements of the same shape (nodes,
//! points, hourglass control) form a [`Family`], one dispatch per step.
//!
//! The internal force of an element at a point is `w Σ_c σ_kc ∂N_a/∂X_c` on DOF `(a, k)`
//! with the small strain of the displacements relative to the element's first node, the
//! viscous force `c V (g · v) g` plus the hourglass viscosity. Under NLGEOM the stress is
//! `P = F S` with `S = D E` of the Green-Lagrange strain `E`, and the viscosity takes the
//! volume and the mean gradients of the current configuration: from the points' `F` for
//! the fully integrated elements (`J F⁻ᵀ ∇N`), from the current nodal coordinates with
//! 2 × 2 × 2 points for the uniform-strain hexahedron, as the processor does. [`Elements::forces`] is
//! the same in double precision on the processor, the reference of the tests.

// Index loops read as the tensor formulas they implement.
#![allow(clippy::needless_range_loop)]

/// The hourglass control of one uniform-strain hexahedron (C3D8R).
#[derive(Clone, Debug, PartialEq)]
pub struct Hourglass {
    /// `ε` of the stiffness `ε Σ_α γ_α γ_αᵀ ⊗ I` with `γ_α = h_α − (h_α · z) ∂N/∂z`.
    pub stiffness: f64,
    /// `κ` of the viscosity `κ Σ_α γ_α (γ_α · v)` with `γ_α = h_α − Σ_j (h_α · x_j) g_j`.
    pub viscosity: f64,
    /// The nodal coordinates.
    pub coordinates: Vec<[f64; 3]>,
}

/// One element.
#[derive(Clone, Debug, PartialEq)]
pub struct Element {
    /// Three per node.
    pub dofs: Vec<u32>,
    /// Per integration point: the volume it stands for and the shape function derivatives
    /// per node.
    pub points: Vec<(f64, Vec<[f64; 3]>)>,
    /// The elastic matrix on engineering strains.
    pub elastic: [[f64; 6]; 6],
    /// The bulk viscosity `c V` and the mean gradients `g` (three per node).
    pub bulk: f64,
    pub mean: Vec<f64>,
    /// Only with eight nodes and one point.
    pub hourglass: Option<Hourglass>,
}

/// Elements of one shape, element by element.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Family {
    pub nodes: usize,
    pub points: usize,
    pub hourglass: bool,
    pub count: usize,
    /// `count × 3 nodes`.
    pub dofs: Vec<u32>,
    /// Per element and point the volume, then the derivatives node by node:
    /// `count × points × (1 + 3 nodes)`.
    pub gradients: Vec<f64>,
    /// Per element `c V`, then `g`: `count × (1 + 3 nodes)`.
    pub viscosity: Vec<f64>,
    /// Per element `ε`, `κ`, then the coordinates relative to the first node:
    /// `count × (2 + 3 nodes)`, empty without hourglass control.
    pub control: Vec<f64>,
    /// Per element its index into [`Elements::materials`].
    pub material: Vec<u32>,
}

impl Family {
    /// Values per element and point in [`Family::gradients`].
    pub fn point_size(&self) -> usize {
        1 + 3 * self.nodes
    }
}

/// The elements of a model.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Elements {
    pub families: Vec<Family>,
    /// The different elastic matrices.
    pub materials: Vec<[[f64; 6]; 6]>,
    /// Large deformations (total Lagrangian: Green-Lagrange strain, Saint Venant-Kirchhoff
    /// stress, the bulk and hourglass viscosity in the current configuration).
    pub nlgeom: bool,
}

/// The hourglass base vectors of the eight-node brick (the same set as the elements').
pub(crate) const HOURGLASS_BASE: [[f64; 8]; 4] = [
    [1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0],
    [1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0],
    [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0],
    [-1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0],
];

impl Elements {
    /// Adds an element to the family of its shape. `Err` when it does not fit the kernels.
    pub fn push(&mut self, element: Element) -> Result<(), String> {
        let nodes = element.dofs.len() / 3;
        let points = element.points.len();
        if nodes == 0
            || element.dofs.len() != 3 * nodes
            || points == 0
            || element.points.iter().any(|(_, g)| g.len() != nodes)
            || element.mean.len() != 3 * nodes
        {
            return Err("an element of unexpected shape".to_owned());
        }
        if nodes > MAX_NODES {
            return Err(format!("an element of {nodes} nodes"));
        }
        let hourglass = element.hourglass.is_some();
        if let Some(h) = &element.hourglass
            && (nodes != 8 || points != 1 || h.coordinates.len() != 8)
        {
            return Err("hourglass control for an element other than C3D8R".to_owned());
        }
        let material = match self.materials.iter().position(|m| *m == element.elastic) {
            Some(i) => i,
            None => {
                self.materials.push(element.elastic);
                self.materials.len() - 1
            }
        } as u32;
        let family = match self
            .families
            .iter()
            .position(|f| f.nodes == nodes && f.points == points && f.hourglass == hourglass)
        {
            Some(i) => &mut self.families[i],
            None => {
                self.families.push(Family {
                    nodes,
                    points,
                    hourglass,
                    ..Default::default()
                });
                self.families.last_mut().expect("pushed")
            }
        };
        family.count += 1;
        family.dofs.extend_from_slice(&element.dofs);
        for (w, g) in &element.points {
            family.gradients.push(*w);
            family.gradients.extend(g.iter().flatten());
        }
        family.viscosity.push(element.bulk);
        family.viscosity.extend_from_slice(&element.mean);
        family.material.push(material);
        if let Some(h) = element.hourglass {
            family.control.push(h.stiffness);
            family.control.push(h.viscosity);
            let x0 = h.coordinates[0];
            for x in &h.coordinates {
                family.control.extend((0..3).map(|j| x[j] - x0[j]));
            }
        }
        Ok(())
    }

    /// The number of elements.
    pub fn len(&self) -> usize {
        self.families.iter().map(|f| f.count).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The internal and the viscous forces at the displacements `u` and velocities `v`,
    /// in double precision.
    pub fn forces(&self, u: &[f64], v: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let mut internal = vec![0.0; u.len()];
        let mut viscous = vec![0.0; u.len()];
        for family in &self.families {
            let nd = 3 * family.nodes;
            for e in 0..family.count {
                let dofs = &family.dofs[e * nd..(e + 1) * nd];
                let ue: Vec<f64> = dofs
                    .iter()
                    .enumerate()
                    .map(|(k, &d)| u[d as usize] - u[dofs[k % 3] as usize])
                    .collect();
                let ve: Vec<f64> = dofs.iter().map(|&d| v[d as usize]).collect();
                let mut f = vec![0.0; nd];
                let mut fv = vec![0.0; nd];
                let d = &self.materials[family.material[e] as usize];
                let size = family.point_size();
                // The current volume and `V g` of the fully integrated elements.
                let mut current = (0.0, vec![0.0; nd]);
                let mut reference = 0.0;
                for ip in 0..family.points {
                    let at = (e * family.points + ip) * size;
                    let w = family.gradients[at];
                    let g = &family.gradients[at + 1..at + size];
                    reference += w;
                    point_forces(w, g, &ue, d, self.nlgeom, &mut f, &mut current);
                }
                let visc = &family.viscosity[e * (1 + nd)..(e + 1) * (1 + nd)];
                let control = family
                    .hourglass
                    .then(|| &family.control[e * (2 + nd)..(e + 1) * (2 + nd)]);
                let moved: Vec<f64>;
                let (volume, mean): (f64, &[f64]) = match (self.nlgeom, control) {
                    (false, _) => (reference, &visc[1..]),
                    (true, Some(control)) => {
                        let x: Vec<[f64; 3]> = (0..8)
                            .map(|a| {
                                std::array::from_fn(|j| control[2 + 3 * a + j] + ue[3 * a + j])
                            })
                            .collect();
                        let (volume, g) = brick_mean_gradients(&x);
                        moved = g;
                        (volume, &moved)
                    }
                    (true, None) => {
                        moved = current.1.iter().map(|g| g / current.0).collect();
                        (current.0, &moved)
                    }
                };
                let q = visc[0] / reference
                    * volume
                    * mean.iter().zip(&ve).map(|(g, v)| g * v).sum::<f64>();
                for (o, g) in fv.iter_mut().zip(mean) {
                    *o += q * g;
                }
                if let Some(control) = control {
                    let g = &family.gradients[e * size + 1..(e + 1) * size];
                    hourglass_forces(control, g, mean, &ue, &ve, self.nlgeom, &mut f, &mut fv);
                }
                for (k, &dof) in dofs.iter().enumerate() {
                    internal[dof as usize] += f[k];
                    viscous[dof as usize] += fv[k];
                }
            }
        }
        (internal, viscous)
    }
}

/// The most nodes of an element the kernels take (C3D20).
pub const MAX_NODES: usize = 20;

/// Adds `w Bᵀ σ` of one point with the derivatives `g` (three per node) to `f`, and under
/// NLGEOM the current volume `w J` and `w J F⁻ᵀ ∇N` to `current`.
fn point_forces(
    w: f64,
    g: &[f64],
    ue: &[f64],
    d: &[[f64; 6]; 6],
    nlgeom: bool,
    f: &mut [f64],
    current: &mut (f64, Vec<f64>),
) {
    let nodes = g.len() / 3;
    let mut h = [[0.0; 3]; 3];
    for a in 0..nodes {
        for (k, hk) in h.iter_mut().enumerate() {
            for (c, hkc) in hk.iter_mut().enumerate() {
                *hkc += ue[3 * a + k] * g[3 * a + c];
            }
        }
    }
    // `Σ_k H_ki H_kj` under NLGEOM.
    let hh = |i: usize, j: usize| {
        if nlgeom {
            (0..3).map(|k| h[k][i] * h[k][j]).sum::<f64>()
        } else {
            0.0
        }
    };
    let strain = [
        h[0][0] + 0.5 * hh(0, 0),
        h[1][1] + 0.5 * hh(1, 1),
        h[2][2] + 0.5 * hh(2, 2),
        h[0][1] + h[1][0] + hh(0, 1),
        h[1][2] + h[2][1] + hh(1, 2),
        h[2][0] + h[0][2] + hh(2, 0),
    ];
    let s: Vec<f64> = d
        .iter()
        .map(|row| row.iter().zip(&strain).map(|(a, b)| a * b).sum())
        .collect();
    let sm = [[s[0], s[3], s[5]], [s[3], s[1], s[4]], [s[5], s[4], s[2]]];
    // P = F S (F = I under small strains).
    let mut p = sm;
    let mut fm = [[0.0; 3]; 3];
    if nlgeom {
        for k in 0..3 {
            for c in 0..3 {
                fm[k][c] = f64::from(u8::from(k == c)) + h[k][c];
            }
        }
        for k in 0..3 {
            for c in 0..3 {
                p[k][c] = (0..3).map(|m| fm[k][m] * sm[m][c]).sum();
            }
        }
    }
    for a in 0..nodes {
        for k in 0..3 {
            f[3 * a + k] += w * (0..3).map(|c| p[k][c] * g[3 * a + c]).sum::<f64>();
        }
    }
    if nlgeom {
        // J F⁻ᵀ is the cofactor matrix of F.
        let cof = cofactors(&fm);
        let det: f64 = (0..3).map(|c| fm[0][c] * cof[0][c]).sum();
        current.0 += w * det;
        for a in 0..nodes {
            for k in 0..3 {
                current.1[3 * a + k] += w * (0..3).map(|c| cof[k][c] * g[3 * a + c]).sum::<f64>();
            }
        }
    }
}

/// The cofactor matrix `det(m) m⁻ᵀ`.
fn cofactors(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut c = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            let (i1, i2) = ((i + 1) % 3, (i + 2) % 3);
            let (j1, j2) = ((j + 1) % 3, (j + 2) % 3);
            c[i][j] = m[i1][j1] * m[i2][j2] - m[i1][j2] * m[i2][j1];
        }
    }
    c
}

/// The trilinear brick's natural node coordinates.
pub(crate) const CORNERS: [[f64; 3]; 8] = [
    [-1.0, -1.0, -1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, -1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [1.0, -1.0, 1.0],
    [1.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0],
];

/// Volume and volume mean shape function derivatives (three per node) of a trilinear
/// brick at the nodal positions `x`, with 2 × 2 × 2 points (exact).
fn brick_mean_gradients(x: &[[f64; 3]]) -> (f64, Vec<f64>) {
    let p = 1.0 / 3.0_f64.sqrt();
    let mut volume = 0.0;
    let mut sum = vec![0.0; 24];
    for gauss in &CORNERS {
        let xi = [p * gauss[0], p * gauss[1], p * gauss[2]];
        // dN_a/dξ_m.
        let dn: Vec<[f64; 3]> = CORNERS
            .iter()
            .map(|c| {
                let f = |i: usize| 1.0 + c[i] * xi[i];
                [
                    c[0] * f(1) * f(2) / 8.0,
                    c[1] * f(0) * f(2) / 8.0,
                    c[2] * f(0) * f(1) / 8.0,
                ]
            })
            .collect();
        // J_km = ∂x_k/∂ξ_m.
        let mut j = [[0.0; 3]; 3];
        for (xa, da) in x.iter().zip(&dn) {
            for k in 0..3 {
                for m in 0..3 {
                    j[k][m] += xa[k] * da[m];
                }
            }
        }
        let cof = cofactors(&j);
        volume += (0..3).map(|m| j[0][m] * cof[0][m]).sum::<f64>();
        // det J ∂N/∂x = cof(J) ∂N/∂ξ.
        for (a, da) in dn.iter().enumerate() {
            for k in 0..3 {
                sum[3 * a + k] += (0..3).map(|m| cof[k][m] * da[m]).sum::<f64>();
            }
        }
    }
    sum.iter_mut().for_each(|g| *g /= volume);
    (volume, sum)
}

/// The hourglass stiffness and viscosity of a C3D8R: `control` is `ε`, `κ` and the
/// relative coordinates, `g` the point's derivatives, `mean` the mean gradients (of the
/// current configuration under NLGEOM, where the viscosity also takes the current
/// coordinates).
#[allow(clippy::too_many_arguments)]
fn hourglass_forces(
    control: &[f64],
    g: &[f64],
    mean: &[f64],
    ue: &[f64],
    ve: &[f64],
    nlgeom: bool,
    f: &mut [f64],
    fv: &mut [f64],
) {
    let (epsilon, kappa, x) = (control[0], control[1], &control[2..]);
    let moved: Vec<f64> = x
        .iter()
        .zip(ue)
        .map(|(x, u)| if nlgeom { x + u } else { *x })
        .collect();
    for h in &HOURGLASS_BASE {
        let hx: Vec<f64> = (0..3)
            .map(|j| (0..8).map(|a| h[a] * x[3 * a + j]).sum())
            .collect();
        let hm: Vec<f64> = (0..3)
            .map(|j| (0..8).map(|a| h[a] * moved[3 * a + j]).sum())
            .collect();
        let stiff: Vec<f64> = (0..8).map(|a| h[a] - hx[2] * g[3 * a + 2]).collect();
        let visc: Vec<f64> = (0..8)
            .map(|a| h[a] - (0..3).map(|j| hm[j] * mean[3 * a + j]).sum::<f64>())
            .collect();
        for k in 0..3 {
            let q: f64 = (0..8).map(|a| stiff[a] * ue[3 * a + k]).sum();
            let r: f64 = (0..8).map(|a| visc[a] * ve[3 * a + k]).sum();
            for a in 0..8 {
                f[3 * a + k] += epsilon * stiff[a] * q;
                fv[3 * a + k] += kappa * visc[a] * r;
            }
        }
    }
}
