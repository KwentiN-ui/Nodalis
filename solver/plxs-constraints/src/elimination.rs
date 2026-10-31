//! Elimination of linear constraints: every constraint (SPC or MPC) makes one degree of
//! freedom dependent, `x_p = Σ e_j x_j + Σ h_s r_s` with independent `x_j` and prescribed
//! values `r_s`. The result is the transformation `u = T v + g` from the reduced unknowns
//! `v` to all degrees of freedom, and `K_red = Tᵀ K T`, `f_red = Tᵀ (f − K g)`.
//!
//! Formulation: master-slave elimination as in the CalculiX manual (section "Linear
//! equations" / "Multiple point constraints": the first term of an equation is the
//! dependent one) and Cook, Malkus, Plesha, Witt, "Concepts and Applications of Finite
//! Element Analysis", chapter 9.2 (transformation equations). Unlike CalculiX, a constraint
//! whose dependent DOF is already taken (by an SPC or an earlier MPC) is not an error: the
//! constraint is first rewritten in the independent DOFs and then pivoted on another of its
//! terms, preferably of the same node (re-pivoting as Abaqus does). Redundant constraints
//! are dropped silently, contradicting ones with a warning.
//!
//! No CalculiX code was consulted.

use std::sync::Arc;

use plxs_linalg::{Pattern, SparseMatrix};

/// What to do when the preferred dependent DOF of a constraint is taken already.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Policy {
    /// Rewrite and pivot on another term (`*EQUATION`, `*RIGID BODY`, SPCs).
    Repivot,
    /// Drop the constraint (CalculiX' `*TIE`: slave DOFs that are held otherwise are not
    /// tied).
    KeepPrevious,
}

/// A linear constraint `Σ a_i x_i = 0` on primary DOFs (`node * 3 + d`, local directions at
/// transformed nodes).
#[derive(Clone, Debug, PartialEq)]
pub struct LinearConstraint {
    pub terms: Vec<(u32, f64)>,
    pub preferred: u32,
    pub policy: Policy,
    /// Index into the labels of the constraint source, for messages.
    pub origin: u32,
}

/// What became of a constraint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Added {
    Pivot(u32),
    Repivoted { from: u32, to: u32 },
    Dropped,
    Redundant,
    Conflict,
}

const NONE: u32 = u32::MAX;

/// A row of `T` (reduced column, coefficient) and of `g` (prescribed value, coefficient).
type Row = (Vec<(u32, f64)>, Vec<(u32, f64)>);

#[derive(Clone, Debug, Default)]
struct Expr {
    vars: Vec<(u32, f64)>,
    consts: Vec<(u32, f64)>,
}

/// `target += a * source`, merging equal keys.
fn axpy(target: &mut Vec<(u32, f64)>, source: &[(u32, f64)], a: f64) {
    for &(k, v) in source {
        match target.iter_mut().find(|(t, _)| *t == k) {
            Some((_, tv)) => *tv += a * v,
            None => target.push((k, a * v)),
        }
    }
}

/// Incremental elimination over `n` primary DOFs.
pub struct Eliminator {
    expr_of: Vec<u32>,
    exprs: Vec<Expr>,
    /// For every independent DOF, the dependent ones whose expression may contain it
    /// (possibly stale or duplicated; checked on use).
    users: Vec<Vec<u32>>,
}

impl Eliminator {
    pub fn new(n: usize) -> Self {
        Self {
            expr_of: vec![NONE; n],
            exprs: Vec::new(),
            users: vec![Vec::new(); n],
        }
    }

    pub fn is_dependent(&self, var: u32) -> bool {
        self.expr_of[var as usize] != NONE
    }

    /// Adds `Σ terms + Σ consts = 0`, where `consts` are `(source, coefficient)` pairs of
    /// prescribed values.
    pub fn add(
        &mut self,
        terms: &[(u32, f64)],
        consts: &[(u32, f64)],
        preferred: u32,
        policy: Policy,
    ) -> Added {
        if policy == Policy::KeepPrevious && self.is_dependent(preferred) {
            return Added::Dropped;
        }
        let scale = terms.iter().fold(0.0_f64, |m, &(_, a)| m.max(a.abs()));
        if scale == 0.0 {
            return Added::Redundant;
        }
        let mut vars: Vec<(u32, f64)> = Vec::with_capacity(terms.len());
        let mut cs: Vec<(u32, f64)> = consts.to_vec();
        for &(v, a) in terms {
            match self.expr_of[v as usize] {
                NONE => axpy(&mut vars, &[(v, 1.0)], a),
                e => {
                    let expr = &self.exprs[e as usize];
                    axpy(&mut vars, &expr.vars, a);
                    axpy(&mut cs, &expr.consts, a);
                }
            }
        }
        let tolerance = 1e-10 * scale;
        vars.retain(|&(_, a)| a.abs() > tolerance);
        cs.retain(|&(_, a)| a != 0.0);
        if vars.is_empty() {
            return if cs.iter().all(|&(_, k)| k.abs() <= tolerance) {
                Added::Redundant
            } else {
                Added::Conflict
            };
        }
        let pivot = match vars.iter().position(|&(v, _)| v == preferred) {
            Some(k) => k,
            None => {
                // Re-pivot: the largest coefficient of the same node, else of any node.
                let node = preferred / 3;
                let best = |filter: &dyn Fn(u32) -> bool| {
                    vars.iter()
                        .enumerate()
                        .filter(|(_, (v, _))| filter(*v))
                        .max_by(|a, b| a.1.1.abs().total_cmp(&b.1.1.abs()))
                        .map(|(k, _)| k)
                };
                best(&|v| v / 3 == node)
                    .or_else(|| best(&|_| true))
                    .unwrap()
            }
        };
        let (p, ap) = vars.swap_remove(pivot);
        let expr = Expr {
            vars: vars.iter().map(|&(v, a)| (v, -a / ap)).collect(),
            consts: cs.iter().map(|&(s, k)| (s, -k / ap)).collect(),
        };
        // Substitute the new dependent into the expressions that contain it.
        let mut users = std::mem::take(&mut self.users[p as usize]);
        users.sort_unstable();
        users.dedup();
        for d in users {
            let e = self.expr_of[d as usize];
            if e == NONE {
                continue;
            }
            let target = &mut self.exprs[e as usize];
            let Some(k) = target.vars.iter().position(|&(v, _)| v == p) else {
                continue;
            };
            let (_, c) = target.vars.swap_remove(k);
            axpy(&mut target.vars, &expr.vars, c);
            axpy(&mut target.consts, &expr.consts, c);
            target.vars.retain(|&(_, a)| a != 0.0);
            for &(v, _) in &expr.vars {
                self.users[v as usize].push(d);
            }
        }
        for &(v, _) in &expr.vars {
            self.users[v as usize].push(p);
        }
        self.expr_of[p as usize] = self.exprs.len() as u32;
        self.exprs.push(expr);
        if p == preferred {
            Added::Pivot(p)
        } else {
            Added::Repivoted {
                from: preferred,
                to: p,
            }
        }
    }
}

/// The transformation `u = T v + g` of a step: rows are global DOFs, columns the reduced
/// unknowns, `g` a combination of the prescribed values.
#[derive(Clone, Debug)]
pub struct ConstraintSet {
    n: usize,
    reduced: usize,
    t_ptr: Vec<u32>,
    t_col: Vec<u32>,
    t_val: Vec<f64>,
    g_ptr: Vec<u32>,
    g_src: Vec<u32>,
    g_val: Vec<f64>,
    /// The primary DOF behind every reduced unknown.
    pub free: Vec<u32>,
    /// Constraints left out because they contradict earlier ones.
    pub warnings: Vec<String>,
    /// Constraints re-pivoted because their dependent DOF was held otherwise.
    pub notes: Vec<String>,
}

impl ConstraintSet {
    /// Builds the transformation from an elimination. `active[v]` marks the primary DOFs
    /// that belong to elements; other independent DOFs take part only when a constraint
    /// refers to them, else they are zero (CalculiX ignores DOFs no element uses).
    /// `rotation(node)` gives the local system of transformed nodes.
    pub(crate) fn from_elimination(
        elimination: &Eliminator,
        active: &[bool],
        rotation: &dyn Fn(u32) -> Option<[[f64; 3]; 3]>,
        warnings: Vec<String>,
        notes: Vec<String>,
    ) -> Self {
        let n = elimination.expr_of.len();
        let mut used = active.to_vec();
        used.resize(n, false);
        for expr in &elimination.exprs {
            for &(v, _) in &expr.vars {
                used[v as usize] = true;
            }
        }
        let mut column = vec![NONE; n];
        let mut free = Vec::new();
        for v in 0..n {
            if used[v] && elimination.expr_of[v] == NONE {
                column[v] = free.len() as u32;
                free.push(v as u32);
            }
        }
        // Rows in primary DOFs.
        let primary_row = |v: usize| -> Row {
            match elimination.expr_of[v] {
                NONE if column[v] != NONE => (vec![(column[v], 1.0)], Vec::new()),
                NONE => (Vec::new(), Vec::new()),
                e => {
                    let expr = &elimination.exprs[e as usize];
                    (
                        expr.vars
                            .iter()
                            .map(|&(j, a)| (column[j as usize], a))
                            .collect(),
                        expr.consts.clone(),
                    )
                }
            }
        };
        let mut set = ConstraintSet {
            n,
            reduced: free.len(),
            t_ptr: vec![0],
            t_col: Vec::new(),
            t_val: Vec::new(),
            g_ptr: vec![0],
            g_src: Vec::new(),
            g_val: Vec::new(),
            free,
            warnings,
            notes,
        };
        for node in 0..n / 3 {
            let rows: [Row; 3] = std::array::from_fn(|k| primary_row(node * 3 + k));
            let r = rotation(node as u32);
            for d in 0..3 {
                let (mut t, mut g) = match r {
                    None => rows[d].clone(),
                    Some(r) => {
                        let mut t = Vec::new();
                        let mut g = Vec::new();
                        for (k, (tk, gk)) in rows.iter().enumerate() {
                            if r[d][k] != 0.0 {
                                axpy(&mut t, tk, r[d][k]);
                                axpy(&mut g, gk, r[d][k]);
                            }
                        }
                        (t, g)
                    }
                };
                t.retain(|&(_, a)| a != 0.0);
                g.retain(|&(_, a)| a != 0.0);
                t.sort_unstable_by_key(|&(c, _)| c);
                for (c, a) in t {
                    set.t_col.push(c);
                    set.t_val.push(a);
                }
                for (s, a) in g {
                    set.g_src.push(s);
                    set.g_val.push(a);
                }
                set.t_ptr.push(set.t_col.len() as u32);
                set.g_ptr.push(set.g_src.len() as u32);
            }
        }
        set
    }

    /// Number of all DOFs.
    pub fn len(&self) -> usize {
        self.n
    }

    pub fn is_empty(&self) -> bool {
        self.n == 0
    }

    /// Number of reduced unknowns (equations).
    pub fn reduced_len(&self) -> usize {
        self.reduced
    }

    fn t_row(&self, row: usize) -> impl Iterator<Item = (u32, f64)> + '_ {
        let range = self.t_ptr[row] as usize..self.t_ptr[row + 1] as usize;
        self.t_col[range.clone()]
            .iter()
            .copied()
            .zip(self.t_val[range].iter().copied())
    }

    /// `Tᵀ K T` on a new pattern.
    pub fn reduce(&self, k: &SparseMatrix) -> SparseMatrix {
        let pattern = k.pattern();
        assert_eq!(pattern.dimension(), self.n);
        let m = self.reduced;
        // Pass 1: the pattern.
        let mut rows: Vec<Vec<u32>> = vec![Vec::new(); m];
        for j in 0..self.n {
            if self.t_ptr[j] == self.t_ptr[j + 1] {
                continue;
            }
            for &i in pattern.column(j) {
                for (a, _) in self.t_row(i as usize) {
                    for (b, _) in self.t_row(j) {
                        let (r, c) = if a <= b { (a, b) } else { (b, a) };
                        rows[c as usize].push(r);
                    }
                }
            }
        }
        for list in &mut rows {
            list.sort_unstable();
            list.dedup();
        }
        let reduced_pattern = Arc::new(Pattern::from_columns(m, rows));
        // Pass 2: the values.
        let mut out = SparseMatrix::zeros(reduced_pattern.clone());
        let values = k.values();
        let col_ptr = pattern.col_ptr();
        let out_values = out.values_mut();
        for (j, &start) in col_ptr.iter().take(self.n).enumerate() {
            if self.t_ptr[j] == self.t_ptr[j + 1] {
                continue;
            }
            let start = start as usize;
            for (offset, &i) in pattern.column(j).iter().enumerate() {
                let v = values[start + offset];
                if v == 0.0 {
                    continue;
                }
                let diagonal = i as usize == j;
                for (a, ta) in self.t_row(i as usize) {
                    for (b, tb) in self.t_row(j) {
                        if diagonal && a > b {
                            continue;
                        }
                        let (r, c) = if a <= b { (a, b) } else { (b, a) };
                        // Off-diagonal entries of K stand for K_ij and K_ji: both land on
                        // the same reduced entry, twice on a reduced diagonal.
                        let factor = if !diagonal && a == b { 2.0 } else { 1.0 };
                        let pos = reduced_pattern.position(r, c).unwrap();
                        out_values[pos] += factor * ta * v * tb;
                    }
                }
            }
        }
        out
    }

    /// `Tᵀ f`.
    pub fn reduce_rhs(&self, f: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.reduced];
        for (row, &value) in f.iter().enumerate() {
            if value == 0.0 {
                continue;
            }
            for (c, t) in self.t_row(row) {
                out[c as usize] += t * value;
            }
        }
        out
    }

    /// `g` for the prescribed values (in the order of the SPC DOFs given to `build`).
    pub fn particular(&self, prescribed: &[f64]) -> Vec<f64> {
        (0..self.n)
            .map(|row| {
                let range = self.g_ptr[row] as usize..self.g_ptr[row + 1] as usize;
                self.g_src[range.clone()]
                    .iter()
                    .zip(&self.g_val[range])
                    .map(|(&s, &a)| a * prescribed[s as usize])
                    .sum()
            })
            .collect()
    }

    /// `u = T v + g`.
    pub fn expand(&self, v: &[f64], g: &[f64]) -> Vec<f64> {
        (0..self.n)
            .map(|row| g[row] + self.t_row(row).map(|(c, t)| t * v[c as usize]).sum::<f64>())
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn substitution_and_repivot() {
        // x0 = r0 (SPC), x0 - x1 = 0 (equation on a held DOF -> pivots on x1).
        let mut e = Eliminator::new(6);
        assert_eq!(
            e.add(&[(0, 1.0)], &[(0, -1.0)], 0, Policy::Repivot),
            Added::Pivot(0)
        );
        assert_eq!(
            e.add(&[(0, 1.0), (1, -1.0)], &[], 0, Policy::Repivot),
            Added::Repivoted { from: 0, to: 1 }
        );
        // The same equation again is redundant; x1 = 2 x2 contradicts nothing.
        assert_eq!(
            e.add(&[(1, 1.0), (0, -1.0)], &[], 1, Policy::Repivot),
            Added::Redundant
        );
        assert_eq!(
            e.add(&[(1, 1.0)], &[], 1, Policy::KeepPrevious),
            Added::Dropped
        );
        assert_eq!(e.add(&[(1, 1.0)], &[], 1, Policy::Repivot), Added::Conflict);
        let set =
            ConstraintSet::from_elimination(&e, &[true; 6], &|_| None, Vec::new(), Vec::new());
        // x0 and x1 are dependent: 4 unknowns.
        assert_eq!(set.reduced_len(), 4);
        let g = set.particular(&[2.0]);
        assert_eq!(&g[..3], &[2.0, 2.0, 0.0]);
    }

    #[test]
    fn later_pivot_is_substituted_into_earlier_expressions() {
        // x0 = x1 + x2, then x1 = 3 x2: x0 = 4 x2.
        let mut e = Eliminator::new(3);
        e.add(&[(0, 1.0), (1, -1.0), (2, -1.0)], &[], 0, Policy::Repivot);
        e.add(&[(1, 1.0), (2, -3.0)], &[], 1, Policy::Repivot);
        let set =
            ConstraintSet::from_elimination(&e, &[true; 3], &|_| None, Vec::new(), Vec::new());
        assert_eq!(set.reduced_len(), 1);
        let u = set.expand(&[1.0], &[0.0; 3]);
        assert!((u[0] - 4.0).abs() < 1e-14 && (u[1] - 3.0).abs() < 1e-14 && u[2] == 1.0);
    }

    #[test]
    fn reduction_equals_dense_triple_product() {
        // K on 3 DOFs, constraint x2 = 0.5 x0 + 0.5 x1.
        let p = Arc::new(Pattern::from_blocks(3, [&[0u32, 1, 2][..]]));
        let mut k = SparseMatrix::zeros(p);
        let dense = [[4.0, -1.0, 0.5], [-1.0, 3.0, -2.0], [0.5, -2.0, 5.0]];
        k.add_block(&[0, 1, 2], &dense.concat());
        let mut e = Eliminator::new(3);
        e.add(&[(2, 1.0), (0, -0.5), (1, -0.5)], &[], 2, Policy::Repivot);
        let set =
            ConstraintSet::from_elimination(&e, &[true; 3], &|_| None, Vec::new(), Vec::new());
        let r = set.reduce(&k);
        let t = [[1.0, 0.0], [0.0, 1.0], [0.5, 0.5]];
        for a in 0..2 {
            for b in 0..2 {
                let mut expected = 0.0;
                for i in 0..3 {
                    for j in 0..3 {
                        expected += t[i][a] * dense[i][j] * t[j][b];
                    }
                }
                assert!(
                    (r.get(a as u32, b as u32) - expected).abs() < 1e-12,
                    "{a} {b}"
                );
            }
        }
    }
}
