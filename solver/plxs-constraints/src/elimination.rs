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
use rayon::prelude::*;

/// The reduced positions of the products of `Tᵀ K T` that are not simple, by ranges of
/// columns of `K` (see [`ReducedPattern`]).
type Landing = Vec<(std::ops::Range<usize>, Vec<u32>)>;

/// The pattern of `Tᵀ K T` for the matrices on one pattern and where every entry of `K`
/// lands in it (see [`ConstraintSet::reducer`]). It depends on the structure of `T` only,
/// not on its values: the tangents of rigid bodies under finite rotations, whose `T` is
/// relinearized in every iteration, reuse it too.
#[derive(Clone, Debug)]
pub struct ReducedPattern {
    input: Arc<Pattern>,
    pattern: Arc<Pattern>,
    /// The structure of `T` the reducer was built for.
    t_ptr: Vec<u32>,
    t_col: Vec<u32>,
    /// The rows of DOFs no element uses (see [`ConstraintSet`]), whose entries of `K` are
    /// left out.
    inert: Vec<bool>,
    /// Per entry of the input: its reduced position when it lands on one reduced entry
    /// with factor 1 (an entry between unconstrained DOFs, the usual case), else
    /// `u32::MAX` and the entry is in `general`.
    simple: Vec<u32>,
    /// The other entries, by ranges of columns of the input: the reduced position of each
    /// product `tᵢₐ K_ij tⱼᵦ`, in the order of the entries and for each entry `a` over row
    /// `i` of `T`, `b` over row `j` (on a diagonal entry only `a <= b`). The products are
    /// formed and summed in the same order as a direct `Tᵀ K T` would, so the reduced
    /// matrix is the same to the last bit (a mechanism's solution can hang on round-off of
    /// this size). Only the position is kept (4 bytes a product; a model of expanded
    /// shells has some 300 million of them): the indices into `T` and the factor follow
    /// from the order.
    general: Landing,
}

impl ReducedPattern {
    /// Whether the reducer was built for `pattern`.
    pub fn fits(&self, pattern: &Arc<Pattern>) -> bool {
        Arc::ptr_eq(&self.input, pattern) || *self.input == **pattern
    }

    /// Whether the reducer was built for `pattern` and the structure of `set`'s `T`.
    fn fits_set(&self, pattern: &Arc<Pattern>, set: &ConstraintSet) -> bool {
        self.fits(pattern)
            && self.t_ptr == set.t_ptr
            && self.t_col == set.t_col
            && self.inert == set.inert
    }

    /// The reduced pattern.
    pub fn pattern(&self) -> &Arc<Pattern> {
        &self.pattern
    }

    /// `Tᵀ K T` of `k`, which must be on the pattern the reducer was built for, with the
    /// values `t_val` of `T` (on the structure the reducer was built for).
    fn apply(&self, k: &SparseMatrix, t_val: &[f64]) -> SparseMatrix {
        assert!(
            self.fits(k.pattern()),
            "the matrix is not on the reducer's pattern"
        );
        let mut out = SparseMatrix::zeros(self.pattern.clone());
        let values = k.values();
        let out_values = out.values_mut();
        let input = &*self.input;
        let col_ptr = input.col_ptr();
        let t_range = |row: usize| self.t_ptr[row] as usize..self.t_ptr[row + 1] as usize;
        let inert = |row: usize| self.inert.get(row).copied().unwrap_or(false);
        for (range, positions) in &self.general {
            let mut positions = positions.iter();
            for j in range.clone() {
                if inert(j) {
                    continue;
                }
                let start = col_ptr[j] as usize;
                for (offset, &i) in input.column(j).iter().enumerate() {
                    let entry = start + offset;
                    let v = values[entry];
                    let pos = self.simple[entry];
                    if pos != u32::MAX {
                        out_values[pos as usize] += v;
                        continue;
                    }
                    if inert(i as usize) {
                        continue;
                    }
                    let diagonal = i as usize == j;
                    for ia in t_range(i as usize) {
                        let a = self.t_col[ia];
                        for ib in t_range(j) {
                            let b = self.t_col[ib];
                            if diagonal && a > b {
                                continue;
                            }
                            let factor = if !diagonal && a == b { 2.0 } else { 1.0 };
                            let pos = *positions.next().expect("a position for every product");
                            out_values[pos as usize] += (factor * t_val[ia]) * v * t_val[ib];
                        }
                    }
                }
            }
        }
        out
    }
}

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

/// Smallest coefficient of a loose DOF, relative to the largest of the constraint, that
/// [`Eliminator::add_preferring`] pivots on.
const LOOSE_SHARE: f64 = 1e-3;

/// A compressed index structure by rows: the entries of row `r` are
/// `idx[ptr[r]..ptr[r + 1]]`.
struct Rows {
    ptr: Vec<u32>,
    idx: Vec<u32>,
}

impl Rows {
    fn row(&self, r: usize) -> &[u32] {
        &self.idx[self.ptr[r] as usize..self.ptr[r + 1] as usize]
    }
}

/// The transpose of the structure `ptr`/`idx` of `outer` lists over `0..inner`: for every
/// inner index the outer ones that list it, ascending.
fn transpose(outer: usize, ptr: &[u32], idx: &[u32], inner: usize) -> Rows {
    let mut counts = vec![0u32; inner + 1];
    for &i in idx {
        counts[i as usize + 1] += 1;
    }
    for k in 0..inner {
        counts[k + 1] += counts[k];
    }
    let mut next = counts.clone();
    let mut out = vec![0u32; idx.len()];
    for o in 0..outer {
        for &i in &idx[ptr[o] as usize..ptr[o + 1] as usize] {
            out[next[i as usize] as usize] = o as u32;
            next[i as usize] += 1;
        }
    }
    Rows {
        ptr: counts,
        idx: out,
    }
}

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
        self.add_preferring(terms, consts, preferred, policy, None)
    }

    /// [`Self::add`], but pivoting on a DOF `loose` marks (one no element uses, such as of
    /// a reference point) when the preferred one is not such a DOF and a term is, with the
    /// largest coefficient of those. A distributing coupling, `*EQUATION`s of a region's
    /// nodes and a reference point with the first term on a node of the region, then
    /// makes the point dependent instead of the node: eliminating the node would couple
    /// each of its neighbours to all the region's nodes, a dense block of thousands of
    /// rows in the reduced matrix, while the point has no stiffness to spread. The
    /// constraint is the same either way.
    pub fn add_preferring(
        &mut self,
        terms: &[(u32, f64)],
        consts: &[(u32, f64)],
        preferred: u32,
        policy: Policy,
        loose: Option<&dyn Fn(u32) -> bool>,
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
        let largest = vars.iter().fold(0.0_f64, |m, &(_, a)| m.max(a.abs()));
        let loose_pivot = loose.filter(|loose| !loose(preferred)).and_then(|loose| {
            (vars.iter().enumerate())
                .filter(|(_, (v, a))| loose(*v) && a.abs() >= LOOSE_SHARE * largest)
                .max_by(|a, b| a.1.1.abs().total_cmp(&b.1.1.abs()))
                .map(|(k, _)| k)
        });
        let pivot = match loose_pivot.or_else(|| vars.iter().position(|&(v, _)| v == preferred)) {
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
        if p == preferred || loose_pivot.is_some() {
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
    /// The rows of DOFs no element uses, of nodes such as reference points, in global
    /// directions. Their entries of `K` are zero, so the reduction leaves them out: the row
    /// of `T` of a reference point a distributing coupling makes dependent on thousands of
    /// nodes would else put a dense block of zeros as large into `Tᵀ K T`, from the
    /// diagonal every DOF has in the pattern.
    inert: Vec<bool>,
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
            // Only when `active` covers all DOFs (a cyclic step's doubled numbering gives
            // it for one copy).
            inert: if active.len() == n {
                (0..n)
                    .map(|v| (0..3).all(|k| !active[v / 3 * 3 + k]))
                    .collect()
            } else {
                Vec::new()
            },
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

    /// `T` by rows: the row pointers (`len() + 1`), the columns (reduced unknowns) and
    /// the values.
    pub fn transform(&self) -> (&[u32], &[u32], &[f64]) {
        (&self.t_ptr, &self.t_col, &self.t_val)
    }

    fn t_row(&self, row: usize) -> impl Iterator<Item = (u32, f64)> + '_ {
        let range = self.t_ptr[row] as usize..self.t_ptr[row + 1] as usize;
        self.t_col[range.clone()]
            .iter()
            .copied()
            .zip(self.t_val[range].iter().copied())
    }

    /// Whether row `row` of `T` is the identity row of a free DOF: a single entry 1 on the
    /// DOF's own reduced unknown. Relinearizing the rows of rigid body members never
    /// touches these rows.
    fn is_inert(&self, row: usize) -> bool {
        self.inert.get(row).copied().unwrap_or(false)
    }

    fn is_identity_row(&self, row: usize) -> bool {
        let range = self.t_ptr[row] as usize..self.t_ptr[row + 1] as usize;
        range.len() == 1
            && self.t_val[range.start] == 1.0
            && self.free[self.t_col[range.start] as usize] as usize == row
    }

    /// Whether `T` only selects the free DOFs: every row is the identity row of its own
    /// free DOF or empty (held, or not used by any element), so that `u = q` on the free
    /// DOFs and `u = g` on the others.
    pub fn is_selection(&self) -> bool {
        (0..self.n).all(|row| self.t_ptr[row] == self.t_ptr[row + 1] || self.is_identity_row(row))
    }

    /// `Tᵀ K T` on a new pattern.
    pub fn reduce(&self, k: &SparseMatrix) -> SparseMatrix {
        self.reducer(k.pattern()).apply(k, &self.t_val)
    }

    /// [`Self::reduce`] with the reduced pattern kept in `cache` for the next matrix on
    /// the same pattern (the tangents of Newton iterations): only the values are mapped
    /// then, which is several times cheaper than building the pattern again. The cache
    /// serves any set with the same structure of `T`, whatever its values (the rows of
    /// rigid bodies relinearized at other rotations).
    pub fn reduce_cached(
        &self,
        k: &SparseMatrix,
        cache: &mut Option<ReducedPattern>,
    ) -> SparseMatrix {
        if !cache
            .as_ref()
            .is_some_and(|c| c.fits_set(k.pattern(), self))
        {
            *cache = Some(self.reducer(k.pattern()));
        }
        cache.as_ref().expect("built").apply(k, &self.t_val)
    }

    /// The reduced pattern of `Tᵀ K T` for matrices on `pattern`, with where every entry
    /// of `K` lands.
    ///
    /// Both passes run in parallel: the pattern column by column of the reduced matrix (a
    /// symbolic product with a marker per thread, so that no duplicates are collected:
    /// collecting them per entry of `K` took most of the time and memory of a model with
    /// many constraints, such as expanded shells), the positions in ranges of columns of
    /// `K`, concatenated in their order so that [`ReducedPattern::apply`] sums exactly as
    /// before.
    pub fn reducer(&self, pattern: &Arc<Pattern>) -> ReducedPattern {
        assert_eq!(pattern.dimension(), self.n);
        let reduced_pattern = Arc::new(self.reduced_pattern(pattern));
        let (simple, general) = self.landing(pattern, &reduced_pattern);
        ReducedPattern {
            input: pattern.clone(),
            pattern: reduced_pattern,
            t_ptr: self.t_ptr.clone(),
            t_col: self.t_col.clone(),
            inert: self.inert.clone(),
            simple,
            general,
        }
    }

    /// The pattern of `Tᵀ K T` (upper triangle, with the diagonal of every reduced
    /// unknown): entry `(a, c)`, `a <= c`, is there when some entry `K_pq` of the full
    /// (symmetric) `K` has `T_pa` and `T_qc` in the structure of `T`.
    fn reduced_pattern(&self, pattern: &Pattern) -> Pattern {
        let n = self.n;
        let m = self.reduced;
        // `K` by rows of its upper triangle (the entries `K_qp`, `p > q`, of column `q`
        // of the lower triangle), and `T` by columns.
        let upper_rows = transpose(n, pattern.col_ptr(), pattern.row_idx(), n);
        let t_cols = transpose(n, &self.t_ptr, &self.t_col, m);
        let columns: Vec<Vec<u32>> = (0..m)
            .into_par_iter()
            .with_min_len(256)
            .map_init(
                || vec![u32::MAX; m],
                |mark, c| {
                    let mut rows = vec![c as u32];
                    mark[c] = c as u32;
                    for &q in t_cols.row(c) {
                        let q = q as usize;
                        if self.is_inert(q) {
                            continue;
                        }
                        for &p in pattern.column(q).iter().chain(upper_rows.row(q)) {
                            if self.is_inert(p as usize) {
                                continue;
                            }
                            for &a in &self.t_col[self.t_ptr[p as usize] as usize
                                ..self.t_ptr[p as usize + 1] as usize]
                            {
                                if a as usize <= c && mark[a as usize] != c as u32 {
                                    mark[a as usize] = c as u32;
                                    rows.push(a);
                                }
                            }
                        }
                    }
                    rows.sort_unstable();
                    rows
                },
            )
            .collect();
        let mut col_ptr = Vec::with_capacity(m + 1);
        col_ptr.push(0u32);
        for rows in &columns {
            col_ptr.push(col_ptr.last().copied().unwrap_or(0) + rows.len() as u32);
        }
        let mut row_idx = vec![0u32; col_ptr[m] as usize];
        let mut rest = row_idx.as_mut_slice();
        let mut parts = Vec::with_capacity(m);
        for rows in &columns {
            let (part, tail) = rest.split_at_mut(rows.len());
            parts.push(part);
            rest = tail;
        }
        parts
            .into_par_iter()
            .zip(columns.par_iter())
            .for_each(|(part, rows)| part.copy_from_slice(rows));
        Pattern::from_compressed(m, col_ptr, row_idx)
    }

    /// Where every entry of `K` on `pattern` lands in `reduced` (see [`ReducedPattern`]),
    /// in the order of the entries.
    fn landing(&self, pattern: &Pattern, reduced: &Pattern) -> (Vec<u32>, Landing) {
        let col_ptr = pattern.col_ptr();
        let mut simple = vec![u32::MAX; pattern.nnz()];
        // Ranges of columns of about equal entries, one slice of `simple` each.
        let chunks = (rayon::current_num_threads() * 8).clamp(1, self.n.max(1));
        let per_chunk = pattern.nnz().div_ceil(chunks).max(1);
        let mut ranges = Vec::with_capacity(chunks + 1);
        let mut first = 0;
        while first < self.n {
            let target = col_ptr[first] as usize + per_chunk;
            let mut last = first + 1;
            while last < self.n && (col_ptr[last] as usize) < target {
                last += 1;
            }
            ranges.push(first..last);
            first = last;
        }
        let mut rest = simple.as_mut_slice();
        let mut slices = Vec::with_capacity(ranges.len());
        for range in &ranges {
            let len = (col_ptr[range.end] - col_ptr[range.start]) as usize;
            let (part, tail) = rest.split_at_mut(len);
            slices.push(part);
            rest = tail;
        }
        let general = ranges
            .into_par_iter()
            .zip(slices)
            .map(|(range, simple)| {
                let base = col_ptr[range.start] as usize;
                let mut general = Vec::new();
                for j in range.clone() {
                    if self.t_ptr[j] == self.t_ptr[j + 1] || self.is_inert(j) {
                        continue;
                    }
                    let start = col_ptr[j] as usize;
                    let identity_j = self.is_identity_row(j);
                    for (offset, &i) in pattern.column(j).iter().enumerate() {
                        let entry = start + offset;
                        if self.is_inert(i as usize) {
                            continue;
                        }
                        let diagonal = i as usize == j;
                        if identity_j && self.is_identity_row(i as usize) {
                            // One target with factor 1, whatever the values of the other
                            // rows.
                            let a = self.t_col[self.t_ptr[i as usize] as usize];
                            let b = self.t_col[self.t_ptr[j] as usize];
                            let (r, c) = if a <= b { (a, b) } else { (b, a) };
                            simple[entry - base] = reduced.position(r, c).unwrap() as u32;
                            continue;
                        }
                        // Off-diagonal entries of K stand for K_ij and K_ji: both land on
                        // the same reduced entry, twice on a reduced diagonal (the factor
                        // 2 of `apply`).
                        for (a, _) in self.t_row(i as usize) {
                            for (b, _) in self.t_row(j) {
                                if diagonal && a > b {
                                    continue;
                                }
                                let (r, c) = if a <= b { (a, b) } else { (b, a) };
                                general.push(reduced.position(r, c).unwrap() as u32);
                            }
                        }
                    }
                }
                (range, general)
            })
            .collect();
        (simple, general)
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
    fn a_distributing_coupling_depends_on_its_loose_reference_dofs() {
        // Nodes 0..3 of a region (DOFs 0..3 used by elements), a reference point with two
        // loose DOFs 4 and 5: x0 = (x1 + x2 + x3) / 3 - x4 and x1 = x5 - x4 + x2.
        let loose = |v: u32| v >= 4;
        let mut e = Eliminator::new(6);
        let first = [
            (0, 1.0),
            (1, -1.0 / 3.0),
            (2, -1.0 / 3.0),
            (3, -1.0 / 3.0),
            (4, 1.0),
        ];
        assert_eq!(
            e.add_preferring(&first, &[], 0, Policy::Repivot, Some(&loose)),
            Added::Pivot(4)
        );
        let second = [(1, 1.0), (5, -1.0), (4, 1.0), (2, -1.0)];
        assert_eq!(
            e.add_preferring(&second, &[], 1, Policy::Repivot, Some(&loose)),
            Added::Pivot(5)
        );
        let set = ConstraintSet::from_elimination(
            &e,
            &[true, true, true, true, false, false],
            &|_| None,
            Vec::new(),
            Vec::new(),
        );
        // All four region DOFs stay unknowns: no element row is touched.
        assert_eq!(set.free, vec![0, 1, 2, 3]);
        let u = set.expand(&[3.0, 1.0, 2.0, 6.0], &[0.0; 6]);
        let residual =
            |terms: &[(u32, f64)]| terms.iter().map(|&(v, a)| a * u[v as usize]).sum::<f64>();
        assert!(residual(&first).abs() < 1e-12 && residual(&second).abs() < 1e-12);
        // Without the loose DOFs marked, the preferred one is eliminated as before.
        let mut e = Eliminator::new(6);
        assert_eq!(e.add(&first, &[], 0, Policy::Repivot), Added::Pivot(0));
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
