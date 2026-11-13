//! Selective mass scaling of expanded shells for explicit steps (Olovsson, Simonsson and
//! Unosson 2005).
//!
//! A shell expands to a solid with two or three nodes through its thickness (a *fibre*), so
//! the stable time step of an explicit step follows from the thickness, not from the size of
//! the element in its plane: 0.4 mm of steel give about 1e-8 s. The scaling raises the
//! inertia of the motion of the fibre's nodes relative to each other only: per element and
//! fibre with the lumped masses `m` of its nodes
//!
//! ```text
//! ΔM = α (diag(m) − m mᵀ / Σm)   (in each direction)
//! ```
//!
//! which is zero for a common translation of the fibre, so the translational mass, the
//! momentum and the motion of the shell's reference surface stay exact; the thickness
//! stretch, the transverse shear through the thickness and the rotation of the fibre (its
//! rotary inertia) get `1 + α` times their inertia. `α = (s / h)² − 1` with the node
//! spacing `s` in the plane and `h` through the thickness brings the thickness modes down to
//! the in-plane ones. The mass matrix is no longer diagonal but couples only the nodes of a
//! fibre; the reduced mass is factorized once per step.

use std::collections::HashMap;

use plxs_assembly::ElementTable;
use plxs_core::Model;
use plxs_linalg::SparseMatrix;

/// The largest factor `α`: the thickness modes of a shell much thinner than its elements
/// would otherwise get the inertia of a body far larger than the shell.
pub(crate) const MAX_ALPHA: f64 = 1.0e4;

/// A fibre in an element: the fibre, the element's local nodes of it in fibre order, and
/// their positions in the fibre.
pub(crate) type ElementFibre = (u32, Vec<usize>, Vec<u8>);

/// The fibres of a model's expanded shells as node indices, and per node its fibre and
/// position in it.
pub(crate) struct Fibres {
    pub(crate) nodes: Vec<Vec<u32>>,
    of_node: HashMap<u32, (u32, u8)>,
}

impl Fibres {
    pub(crate) fn of(model: &Model) -> Self {
        let nodes: Vec<Vec<u32>> = model
            .shell_fibres
            .iter()
            .map(|fibre| {
                fibre
                    .iter()
                    .filter_map(|&id| model.nodes.index(id))
                    .collect::<Vec<u32>>()
            })
            .filter(|f| f.len() >= 2)
            .collect();
        let mut of_node = HashMap::new();
        for (f, fibre) in nodes.iter().enumerate() {
            for (p, &n) in fibre.iter().enumerate() {
                of_node.insert(n, (f as u32, p as u8));
            }
        }
        Self { nodes, of_node }
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// The fibres an element has two or more nodes of: (fibre, the element's local nodes in
    /// the order of the fibre, their positions in the fibre).
    pub(crate) fn in_element(&self, nodes: &[u32]) -> Vec<ElementFibre> {
        let mut by_fibre: HashMap<u32, Vec<(u8, usize)>> = HashMap::new();
        for (local, n) in nodes.iter().enumerate() {
            if let Some(&(f, p)) = self.of_node.get(n) {
                by_fibre.entry(f).or_default().push((p, local));
            }
        }
        let mut out: Vec<ElementFibre> = by_fibre
            .into_iter()
            .filter(|(_, list)| list.len() >= 2)
            .map(|(f, mut list)| {
                list.sort_unstable();
                (
                    f,
                    list.iter().map(|x| x.1).collect(),
                    list.iter().map(|x| x.0).collect(),
                )
            })
            .collect();
        out.sort_unstable_by_key(|x| x.0);
        out
    }
}

/// The factor `α` of an element from its nodal coordinates and its fibres (local nodes in
/// fibre order): the shortest spacing in the plane over the shortest one through the
/// thickness, squared, less one.
pub(crate) fn alpha(coords: &[[f64; 3]], fibres: &[ElementFibre]) -> f64 {
    let distance = |a: usize, b: usize| {
        let (x, y) = (coords[a], coords[b]);
        ((x[0] - y[0]).powi(2) + (x[1] - y[1]).powi(2) + (x[2] - y[2]).powi(2)).sqrt()
    };
    let mut through = f64::INFINITY;
    for (_, local, _) in fibres {
        for w in local.windows(2) {
            through = through.min(distance(w[0], w[1]));
        }
    }
    let mut plane = f64::INFINITY;
    for (i, (_, a, _)) in fibres.iter().enumerate() {
        for (_, b, _) in &fibres[i + 1..] {
            plane = plane.min(distance(a[0], b[0]));
        }
    }
    if !(through > 0.0 && plane.is_finite()) {
        return 0.0;
    }
    ((plane / through).powi(2) - 1.0).clamp(0.0, MAX_ALPHA)
}

/// The added mass of a fibre in an element, `α (diag(m) − m mᵀ / Σm)` (row-major `k × k`).
pub(crate) fn fibre_block(alpha: f64, m: &[f64]) -> Vec<f64> {
    let k = m.len();
    let total: f64 = m.iter().sum();
    let mut block = vec![0.0; k * k];
    if total <= 0.0 {
        return block;
    }
    for i in 0..k {
        for j in 0..k {
            let diagonal = if i == j { m[i] } else { 0.0 };
            block[i * k + j] = alpha * (diagonal - m[i] * m[j] / total);
        }
    }
    block
}

/// The mass a model's fibres add: per fibre a `k × k` block (the same in every direction).
#[derive(Clone, Debug, Default)]
pub(crate) struct FibreMass {
    pub(crate) nodes: Vec<Vec<u32>>,
    pub(crate) blocks: Vec<Vec<f64>>,
    /// The off-diagonal part of the rotary inertias of point masses: three DOFs of a node
    /// and their 3 × 3 block (zero on the diagonal, which the lumped mass holds).
    pub(crate) dof_blocks: Vec<([u32; 3], [f64; 9])>,
}

impl FibreMass {
    pub(crate) fn new(nodes: Vec<Vec<u32>>) -> Self {
        let blocks = nodes.iter().map(|f| vec![0.0; f.len() * f.len()]).collect();
        Self {
            nodes,
            blocks,
            dof_blocks: Vec::new(),
        }
    }

    /// Adds an element's block of fibre `fibre` (on the fibre positions `positions`).
    pub(crate) fn add(&mut self, fibre: u32, positions: &[u8], block: &[f64]) {
        let k = self.nodes[fibre as usize].len();
        let target = &mut self.blocks[fibre as usize];
        let n = positions.len();
        for (a, &i) in positions.iter().enumerate() {
            for (b, &j) in positions.iter().enumerate() {
                target[i as usize * k + j as usize] += block[a * n + b];
            }
        }
    }

    /// `out += ΔM x` (three DOFs per node).
    pub(crate) fn add_times(&self, x: &[f64], out: &mut [f64]) {
        for (dofs, block) in &self.dof_blocks {
            for (i, &a) in dofs.iter().enumerate() {
                for (j, &b) in dofs.iter().enumerate() {
                    out[a as usize] += block[i * 3 + j] * x[b as usize];
                }
            }
        }
        for (nodes, block) in self.nodes.iter().zip(&self.blocks) {
            let k = nodes.len();
            for (i, &a) in nodes.iter().enumerate() {
                for (j, &b) in nodes.iter().enumerate() {
                    let c = block[i * k + j];
                    if c == 0.0 {
                        continue;
                    }
                    for d in 0..3 {
                        out[a as usize * 3 + d] += c * x[b as usize * 3 + d];
                    }
                }
            }
        }
    }

    /// `½ vᵀ ΔM v`.
    pub(crate) fn energy(&self, v: &[f64]) -> f64 {
        let mut out = vec![0.0; v.len()];
        self.add_times(v, &mut out);
        0.5 * v.iter().zip(&out).map(|(a, b)| a * b).sum::<f64>()
    }

    /// The rows above the diagonal the blocks add to every column (three DOFs per node).
    pub(crate) fn columns(&self, n: usize) -> Vec<Vec<u32>> {
        let mut rows: Vec<Vec<u32>> = (0..n as u32).map(|j| vec![j]).collect();
        for (dofs, _) in &self.dof_blocks {
            for &r in dofs {
                for &c in dofs {
                    if r < c && (c as usize) < n {
                        rows[c as usize].push(r);
                    }
                }
            }
        }
        for nodes in &self.nodes {
            for &a in nodes {
                for &b in nodes {
                    for d in 0..3 {
                        let (r, c) = (a * 3 + d, b * 3 + d);
                        if r < c && (c as usize) < n {
                            rows[c as usize].push(r);
                        }
                    }
                }
            }
        }
        rows
    }

    /// Adds the blocks to `m` (upper triangle on a pattern from [`Self::columns`]).
    pub(crate) fn add_to(&self, m: &mut SparseMatrix) {
        let pattern = m.pattern().clone();
        let values = m.values_mut();
        for (dofs, block) in &self.dof_blocks {
            for (i, &r) in dofs.iter().enumerate() {
                for (j, &c) in dofs.iter().enumerate() {
                    if r < c
                        && let Some(p) = pattern.position(r, c)
                    {
                        values[p] += block[i * 3 + j];
                    }
                }
            }
        }
        for (nodes, block) in self.nodes.iter().zip(&self.blocks) {
            let k = nodes.len();
            for (i, &a) in nodes.iter().enumerate() {
                for (j, &b) in nodes.iter().enumerate() {
                    for d in 0..3 {
                        let (r, c) = (a * 3 + d, b * 3 + d);
                        if r > c {
                            continue;
                        }
                        if let Some(p) = pattern.position(r, c) {
                            values[p] += block[i * k + j];
                        }
                    }
                }
            }
        }
    }
}

/// The highest eigenfrequency of `K x = ω² M x` with a dense, symmetric positive definite
/// `M` (both row-major `n × n`): through the Cholesky factor `M = L Lᵀ`, the one of
/// `L⁻¹ K L⁻ᵀ` on unit masses.
pub(crate) fn largest_frequency_dense(k: &[f64], m: &[f64], n: usize) -> f64 {
    // Cholesky, lower triangle.
    let mut l = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut s = m[i * n + j];
            for p in 0..j {
                s -= l[i * n + p] * l[j * n + p];
            }
            if i == j {
                if s <= 0.0 {
                    return f64::INFINITY;
                }
                l[i * n + i] = s.sqrt();
            } else {
                l[i * n + j] = s / l[j * n + j];
            }
        }
    }
    // Y = L⁻¹ K (forward substitution per column), A = L⁻¹ Yᵀ (K symmetric).
    let forward = |b: &mut [f64], stride: usize, col: usize| {
        for i in 0..n {
            let mut s = b[i * stride + col];
            for p in 0..i {
                s -= l[i * n + p] * b[p * stride + col];
            }
            b[i * stride + col] = s / l[i * n + i];
        }
    };
    let mut y = k.to_vec();
    for col in 0..n {
        forward(&mut y, n, col);
    }
    let mut a = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..n {
            a[i * n + j] = y[j * n + i];
        }
    }
    for col in 0..n {
        forward(&mut a, n, col);
    }
    crate::explicit_dynamic::largest_frequency(&a, &vec![1.0; n])
}

/// The element's dense mass: its lumped mass plus the blocks of its fibres.
pub(crate) fn element_mass(
    lumped: &[f64],
    fibres: &[ElementFibre],
    alpha: f64,
) -> (Vec<f64>, Vec<Vec<f64>>) {
    let n = lumped.len();
    let mut m = vec![0.0; n * n];
    for i in 0..n {
        m[i * n + i] = lumped[i];
    }
    let mut blocks = Vec::with_capacity(fibres.len());
    for (_, local, _) in fibres {
        let masses: Vec<f64> = local.iter().map(|&a| lumped[a * 3]).collect();
        let block = fibre_block(alpha, &masses);
        let k = local.len();
        for (i, &a) in local.iter().enumerate() {
            for (j, &b) in local.iter().enumerate() {
                for d in 0..3 {
                    m[(a * 3 + d) * n + b * 3 + d] += block[i * k + j];
                }
            }
        }
        blocks.push(block);
    }
    (m, blocks)
}

/// The fibres of every entry of `table` (empty for entries without).
pub(crate) fn entry_fibres(table: &ElementTable, fibres: &Fibres) -> Vec<Vec<ElementFibre>> {
    table
        .entries
        .iter()
        .map(|entry| {
            if fibres.is_empty() {
                Vec::new()
            } else {
                fibres.in_element(&entry.nodes)
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fibre_block_leaves_the_translation_alone() {
        let m = [1.0, 4.0, 1.0];
        let block = fibre_block(10.0, &m);
        for i in 0..3 {
            let row: f64 = (0..3).map(|j| block[i * 3 + j]).sum();
            assert!(row.abs() < 1e-12);
        }
        // Opposite motion of top and bottom (a rotation of the fibre): eleven times the
        // kinetic energy.
        let x = [1.0, 0.0, -1.0];
        let base: f64 = (0..3).map(|i| m[i] * x[i] * x[i]).sum();
        let added: f64 = (0..3)
            .map(|i| (0..3).map(|j| x[i] * block[i * 3 + j] * x[j]).sum::<f64>())
            .sum();
        assert!((added - 10.0 * base).abs() < 1e-12);
    }

    #[test]
    fn the_dense_frequency_matches_the_lumped_one() {
        // Two masses on a spring: ω² = k (1/m1 + 1/m2).
        let k = [2.0, -2.0, -2.0, 2.0];
        let m = [1.0, 0.0, 0.0, 3.0];
        let omega = largest_frequency_dense(&k, &m, 2);
        let exact = (8.0_f64 / 3.0).sqrt();
        assert!(omega >= exact * 0.999 && omega <= exact * 1.11, "{omega}");
        // A coupled mass that adds inertia to the relative motion only.
        let block = fibre_block(3.0, &[1.0, 3.0]);
        let m: Vec<f64> = [1.0, 0.0, 0.0, 3.0]
            .iter()
            .zip(&block)
            .map(|(a, b)| a + b)
            .collect();
        let scaled = largest_frequency_dense(&k, &m, 2);
        assert!((scaled / omega - 0.5).abs() < 0.02, "{scaled} vs {omega}");
    }
}
