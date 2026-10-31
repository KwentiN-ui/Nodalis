//! Linear multi-point constraints by elimination: every constraint `Σ c_k u_k = b` makes
//! one of its DOFs dependent, `u_d = (b - Σ_{k≠d} c_k u_k) / c_d`, and the system is
//! reduced with the transformation `u = T u_free + u_0` to `Tᵀ K T u_free = Tᵀ (f - K u_0)`
//! (the elimination method of Cook, Malkus, Plesha & Witt, "Concepts and Applications of
//! Finite Element Analysis", section 9.2; the CalculiX manual describes the same for
//! `*EQUATION`). The matrix stays symmetric positive definite, so LDLᵀ applies.
//!
//! The dependent DOF of a constraint is its first term, unless that DOF is prescribed by a
//! single-point constraint or already dependent in an earlier constraint; then the next
//! term that is still free is taken (re-pivoting instead of CalculiX's error message).
//! Dependents that appear in later constraints are substituted, so the expressions only
//! reference free and prescribed DOFs.
//!
//! No CalculiX code was consulted.

use std::collections::HashMap;

use plxs_core::{Error, Result};
use plxs_linalg::{Pattern, SparseMatrix};

use crate::dofs::{Reduction, Spc};

/// A linear constraint `Σ coefficient · u(dof) = constant` on global DOF numbers.
#[derive(Clone, Debug, PartialEq)]
pub struct Mpc {
    pub terms: Vec<(u32, f64)>,
    pub constant: f64,
}

/// A dependent DOF as a combination of basis DOFs (free or prescribed) plus a constant.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Dependent {
    pub terms: Vec<(u32, f64)>,
    pub constant: f64,
}

/// Coefficients below this fraction of the largest one of a constraint count as zero.
const PIVOT_TOLERANCE: f64 = 1e-12;

impl Reduction {
    /// The reduction for single-point constraints `spc` and multi-point constraints `mpcs`
    /// on `total` global DOFs.
    pub fn with_mpcs(total: usize, spc: Spc, mpcs: &[Mpc]) -> Result<Reduction> {
        if mpcs.is_empty() {
            return Ok(Reduction::new(total, spc));
        }
        let mut dependents: HashMap<u32, Dependent> = HashMap::new();
        let mut order: Vec<u32> = Vec::new();
        for (index, mpc) in mpcs.iter().enumerate() {
            // Substitute the dependents known so far and merge equal DOFs.
            let mut merged: Vec<(u32, f64)> = Vec::new();
            let mut constant = mpc.constant;
            for &(dof, c) in &mpc.terms {
                if dof as usize >= total {
                    return Err(Error::input(
                        "*EQUATION",
                        format!("constraint {} references an unknown degree of freedom", index + 1),
                    ));
                }
                match dependents.get(&dof) {
                    Some(dep) => {
                        constant -= c * dep.constant;
                        for &(d, cd) in &dep.terms {
                            add_term(&mut merged, d, c * cd);
                        }
                    }
                    None => add_term(&mut merged, dof, c),
                }
            }
            let largest = merged.iter().fold(0.0_f64, |m, t| m.max(t.1.abs()));
            merged.retain(|t| t.1.abs() > PIVOT_TOLERANCE * largest);
            // The pivot: the first original term still free, else any free term.
            let free = |dof: u32| !spc.is_fixed(dof) && !dependents.contains_key(&dof);
            let pivot = mpc
                .terms
                .iter()
                .map(|t| t.0)
                .find(|&d| free(d) && merged.iter().any(|t| t.0 == d))
                .or_else(|| merged.iter().map(|t| t.0).find(|&d| free(d)));
            let Some(pivot) = pivot else {
                if merged.iter().all(|t| spc.is_fixed(t.0)) {
                    // Redundant with the prescribed values; CalculiX drops such equations
                    // too when they are consistent. Nothing to eliminate.
                    continue;
                }
                return Err(Error::input(
                    "*EQUATION",
                    format!(
                        "constraint {} has no free degree of freedom left to eliminate",
                        index + 1
                    ),
                ));
            };
            let cp = merged.iter().find(|t| t.0 == pivot).unwrap().1;
            let dependent = Dependent {
                terms: merged
                    .iter()
                    .filter(|t| t.0 != pivot)
                    .map(|&(d, c)| (d, -c / cp))
                    .collect(),
                constant: constant / cp,
            };
            dependents.insert(pivot, dependent);
            order.push(pivot);
        }
        // Resolve dependents that reference DOFs which became dependent later. Every
        // expression only references DOFs that were not dependent when it was made, so
        // resolving in reverse order of creation terminates.
        for &dof in order.iter().rev() {
            let dep = dependents[&dof].clone();
            if dep.terms.iter().all(|t| !dependents.contains_key(&t.0)) {
                continue;
            }
            let mut resolved = Dependent {
                terms: Vec::new(),
                constant: dep.constant,
            };
            for &(d, c) in &dep.terms {
                match dependents.get(&d) {
                    Some(inner) => {
                        resolved.constant += c * inner.constant;
                        for &(e, ce) in &inner.terms {
                            add_term(&mut resolved.terms, e, c * ce);
                        }
                    }
                    None => add_term(&mut resolved.terms, d, c),
                }
            }
            dependents.insert(dof, resolved);
        }
        let mut to_reduced = vec![u32::MAX; total];
        let mut free = Vec::with_capacity(total);
        for dof in 0..total as u32 {
            if !spc.is_fixed(dof) && !dependents.contains_key(&dof) {
                to_reduced[dof as usize] = free.len() as u32;
                free.push(dof);
            }
        }
        let mut list: Vec<(u32, Dependent)> = dependents.into_iter().collect();
        list.sort_by_key(|d| d.0);
        Ok(Reduction {
            free,
            to_reduced,
            spc,
            dependents: list,
        })
    }

    /// Whether the reduction eliminates multi-point constraints.
    pub fn has_mpcs(&self) -> bool {
        !self.dependents.is_empty()
    }

    /// The reduced DOFs and coefficients a global DOF stands for (`T` row restricted to the
    /// free DOFs).
    fn row(&self, dof: u32, out: &mut Vec<(u32, f64)>) {
        out.clear();
        let r = self.to_reduced[dof as usize];
        if r != u32::MAX {
            out.push((r, 1.0));
            return;
        }
        if let Ok(k) = self.dependents.binary_search_by_key(&dof, |d| d.0) {
            for &(d, c) in &self.dependents[k].1.terms {
                let r = self.to_reduced[d as usize];
                if r != u32::MAX {
                    out.push((r, c));
                }
            }
        }
    }

    /// `Tᵀ K T`: the stiffness on the free DOFs.
    pub fn reduce_matrix(&self, k: &SparseMatrix) -> SparseMatrix {
        if !self.has_mpcs() {
            return k.restrict(&self.free);
        }
        let pattern = k.pattern();
        let n = pattern.dimension();
        let (mut lr, mut lc) = (Vec::new(), Vec::new());
        // Pattern of the reduced matrix.
        let mut pairs: Vec<[u32; 2]> = Vec::new();
        for col in 0..n {
            self.row(col as u32, &mut lc);
            if lc.is_empty() {
                continue;
            }
            for &row in pattern.column(col) {
                self.row(row, &mut lr);
                for &(p, _) in &lr {
                    for &(q, _) in &lc {
                        if p != q {
                            pairs.push([p.min(q), p.max(q)]);
                        }
                    }
                }
            }
        }
        pairs.sort_unstable();
        pairs.dedup();
        let reduced = std::sync::Arc::new(Pattern::from_blocks(
            self.free.len(),
            pairs.iter().map(|p| &p[..]),
        ));
        let mut out = SparseMatrix::zeros(reduced);
        let values = k.values();
        let col_ptr = pattern.col_ptr();
        for col in 0..n {
            self.row(col as u32, &mut lc);
            if lc.is_empty() {
                continue;
            }
            let start = col_ptr[col] as usize;
            for (offset, &row) in pattern.column(col).iter().enumerate() {
                let v = values[start + offset];
                if v == 0.0 {
                    continue;
                }
                self.row(row, &mut lr);
                if row as usize == col {
                    for &(p, a) in &lr {
                        for &(q, b) in &lr {
                            if p <= q {
                                out.add(p, q, a * b * v);
                            }
                        }
                    }
                } else {
                    for &(p, a) in &lr {
                        for &(q, b) in &lc {
                            let w = a * b * v;
                            out.add(p, q, if p == q { 2.0 * w } else { w });
                        }
                    }
                }
            }
        }
        out
    }
}

/// The `*EQUATION`s of a model as constraints on global DOFs. Only translational DOFs
/// exist in the solver model (rotations of shells, beams and knots live on helper nodes
/// after the expansion), so a term on DOF 4 to 6 is an error.
pub fn model_mpcs(model: &plxs_core::Model, dofs: &crate::DofMap) -> Result<Vec<Mpc>> {
    use plxs_core::mesh::NodeRef;
    let mut out = Vec::with_capacity(model.constraints.equations.len());
    for equation in &model.constraints.equations {
        let mut terms = Vec::with_capacity(equation.terms.len());
        for term in &equation.terms {
            let NodeRef::Node(id) = &term.node else {
                return Err(Error::input(
                    "*EQUATION",
                    format!("node set {} in an equation is not supported", term.node),
                ));
            };
            let index = model
                .nodes
                .index(*id)
                .ok_or_else(|| Error::undefined("*EQUATION", "node", id.to_string()))?;
            let dof = (1..=3)
                .contains(&term.dof)
                .then(|| dofs.dof(index, term.dof as usize - 1))
                .flatten()
                .ok_or_else(|| {
                    Error::input(
                        "*EQUATION",
                        format!("degree of freedom {} of node {id} does not exist", term.dof),
                    )
                })?;
            terms.push((dof, term.coefficient));
        }
        out.push(Mpc {
            terms,
            constant: 0.0,
        });
    }
    Ok(out)
}

fn add_term(terms: &mut Vec<(u32, f64)>, dof: u32, c: f64) {
    match terms.iter_mut().find(|t| t.0 == dof) {
        Some(t) => t.1 += c,
        None => terms.push((dof, c)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    /// Two springs in series, 0 -[k]- 1 -[k]- 2, with u2 = 2 u1 imposed by a constraint and
    /// u0 fixed: the reduced system is one equation.
    #[test]
    fn eliminates_and_expands() {
        let p = Arc::new(Pattern::from_blocks(3, [&[0u32, 1][..], &[1, 2][..]]));
        let mut k = SparseMatrix::zeros(p);
        k.add_block(&[0, 1], &[1.0, -1.0, -1.0, 1.0]);
        k.add_block(&[1, 2], &[1.0, -1.0, -1.0, 1.0]);
        let spc = Spc {
            dofs: vec![0],
            values: vec![0.0],
        };
        let mpc = Mpc {
            terms: vec![(2, 1.0), (1, -2.0)],
            constant: 0.0,
        };
        let r = Reduction::with_mpcs(3, spc, &[mpc]).unwrap();
        assert_eq!(r.free, vec![1]);
        let kr = r.reduce_matrix(&k);
        // T = [0, 1, 2]ᵀ: Tᵀ K T = 1·2·1 + 2·(-1)·1·2 + 4·1 = 2 - 4 + 4 = 2.
        assert!((kr.get(0, 0) - 2.0).abs() < 1e-12);
        // Force 1 on DOF 2: reduced force 2, u1 = 1, u2 = 2.
        let f = vec![0.0, 0.0, 1.0];
        let rhs = r.reduce_vector(&f);
        assert_eq!(rhs, vec![2.0]);
        let mut u = r.prescribed_vector(3);
        r.expand_into(&[1.0], &mut u);
        assert_eq!(u, vec![0.0, 1.0, 2.0]);
    }

    #[test]
    fn re_pivots_a_prescribed_first_term() {
        let spc = Spc {
            dofs: vec![0],
            values: vec![0.5],
        };
        // u0 - u1 = 0 with u0 prescribed: u1 becomes dependent, equal to 0.5.
        let mpc = Mpc {
            terms: vec![(0, 1.0), (1, -1.0)],
            constant: 0.0,
        };
        let r = Reduction::with_mpcs(3, spc, &[mpc]).unwrap();
        assert_eq!(r.free, vec![2]);
        let u = r.prescribed_vector(3);
        assert_eq!(u, vec![0.5, 0.5, 0.0]);
    }

    #[test]
    fn chains_resolve() {
        // u0 = u1, then u1 = u2: u0 and u1 both follow u2.
        let a = Mpc {
            terms: vec![(0, 1.0), (1, -1.0)],
            constant: 0.0,
        };
        let b = Mpc {
            terms: vec![(1, 1.0), (2, -1.0)],
            constant: 0.0,
        };
        let r = Reduction::with_mpcs(3, Spc::default(), &[a, b]).unwrap();
        assert_eq!(r.free, vec![2]);
        let mut u = r.prescribed_vector(3);
        r.expand_into(&[3.0], &mut u);
        assert_eq!(u, vec![3.0, 3.0, 3.0]);
    }
}
