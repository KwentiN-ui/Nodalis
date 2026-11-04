//! Supernodal LDLᵀ factorization that is parallel over the elimination tree.
//!
//! `faer`'s supernodal factorization parallelizes only the dense operations inside one
//! supernode, so the many small and medium supernodes below the top separators run on one
//! core: measured on a 39³-node hexahedral block (173k equations), four threads gave 1.8×
//! one thread. This module keeps `faer`'s symbolic analysis (ordering, supernode partition,
//! structure of the factor, and the value layout, so that `faer`'s solve and the inertia
//! count work unchanged) and replaces the numeric phase:
//!
//! - **Left-looking** like `faer`'s (Ng and Peyton 1993; Rothberg and Gupta 1991): a
//!   supernode gathers the updates of every descendant whose rows reach into its columns
//!   directly from the descendant's finished factor columns (`L_d D_d L_dᵀ` restricted to
//!   its rows, one dense product per descendant, scattered into the supernode's block), then
//!   factorizes its diagonal block with `faer`'s dense LDLᵀ and solves the block below it.
//!   Nothing beyond the factor itself is kept, unlike a multifrontal method, whose update
//!   matrices needed 1.7 GB on top of the 1.9 GB factor of the block above (measured with a
//!   multifrontal variant of this module, 20 % faster once its memory was paged in, but
//!   slower on a first factorization and a risk for large models).
//! - **Tree parallelism**: the subtrees of a supernode are factorized concurrently with
//!   rayon when they carry enough work; a supernode needs only finished descendants, which
//!   all lie in its own subtree. Small subtrees run sequentially on one task.
//! - **Node parallelism** near the root, where no independent subtrees are left: the
//!   columns of a large supernode are split into blocks that gather their updates in
//!   parallel, and its dense kernels use `faer`'s parallelism.
//!
//! The arithmetic is the same as `faer`'s up to the order of the floating-point additions.
//! Only `faer` and `rayon` are used; nothing of CalculiX or PaStiX was consulted.

use faer::dyn_stack::{MemBuffer, MemStack};
use faer::linalg::cholesky::ldlt::factor::{
    LdltError, LdltRegularization, cholesky_in_place, cholesky_in_place_scratch,
};
use faer::linalg::matmul::triangular::{BlockStructure, matmul as triangular_matmul};
use faer::linalg::triangular_solve::solve_unit_lower_triangular_in_place;
use faer::prelude::*;
use faer::sparse::linalg::cholesky::supernodal::SymbolicSupernodalCholesky;
use faer::{Accum, MatMut, Par};

use crate::solver::FactorIndex;
use rayon::prelude::*;
use std::sync::{Mutex, OnceLock};

use crate::Pattern;

const NONE: u32 = u32::MAX;

/// Flops below which a subtree is factorized sequentially by one task.
const MIN_PARALLEL_SUBTREE: f64 = 2e7;
/// Flops from which the dense kernels of one supernode use `faer`'s parallelism.
const PARALLEL_NODE: f64 = 1e8;

/// Dynamic regularization of the pivots: a pivot with `|d| < epsilon` is replaced by
/// `±delta` (its sign kept, or the sign in `signs` when given, indexed by permuted
/// equation). Zeros switch it off; a zero pivot is then an error.
#[derive(Clone, Copy, Debug, Default)]
pub struct Regularization<'a> {
    pub signs: Option<&'a [i8]>,
    pub delta: f64,
    pub epsilon: f64,
}

/// What the numeric factorization needs beside `faer`'s symbolic structure: the
/// elimination tree of the supernodes, work estimates for the scheduling, and the lower
/// triangle of the permuted matrix as positions into the unpermuted upper-triangle values,
/// so that the assembly reads the values in place.
#[derive(Debug)]
pub struct TreeSymbolic {
    child_ptr: Vec<u32>,
    children: Vec<u32>,
    roots: Vec<u32>,
    /// Flops of the dense operations of each supernode and of its whole subtree.
    node_work: Vec<f64>,
    subtree_work: Vec<f64>,
    total_work: f64,
    /// Flops of the updates a supernode receives from its descendants (left-looking).
    incoming_work: Vec<f64>,
    /// For every supernode the descendants whose rows reach into its columns, ascending.
    updater_ptr: Vec<u32>,
    updaters: Vec<u32>,
    /// Lower triangle of `P A Pᵀ` in compressed columns: for every permuted column the
    /// permuted rows (`>=` column) and the position of the entry in the pattern's values.
    a_col_ptr: Vec<u32>,
    a_row: Vec<u32>,
    a_src: Vec<u32>,
}

impl TreeSymbolic {
    /// Builds the tree for `faer`'s supernodal structure of `pattern` permuted by `perm`
    /// (`perm_inv[original] = permuted`; `None` when the natural order is used).
    pub fn new(
        supernodal: &SymbolicSupernodalCholesky<FactorIndex>,
        pattern: &Pattern,
        perm_inv: Option<&[FactorIndex]>,
    ) -> Self {
        let n = pattern.dimension();
        let n_super = supernodal.n_supernodes();
        let begin = supernodal.supernode_begin();
        let end = supernodal.supernode_end();

        // Supernode of every permuted equation.
        let mut index_to_super = vec![0u32; n];
        for s in 0..n_super {
            index_to_super[begin[s] as usize..end[s] as usize].fill(s as u32);
        }
        // Parent = supernode of the first row below the diagonal block (the rows of a
        // supernode's pattern are sorted ascending).
        let mut parent = vec![NONE; n_super];
        let mut child_count = vec![0u32; n_super + 1];
        let mut roots = Vec::new();
        for (s, parent) in parent.iter_mut().enumerate() {
            match supernodal.supernode(s).pattern().first() {
                Some(&row) => {
                    let p = index_to_super[row as usize];
                    *parent = p;
                    child_count[p as usize + 1] += 1;
                }
                None => roots.push(s as u32),
            }
        }
        let mut child_ptr = child_count;
        for s in 0..n_super {
            child_ptr[s + 1] += child_ptr[s];
        }
        let mut next = child_ptr[..n_super].to_vec();
        let mut children = vec![0u32; child_ptr[n_super] as usize];
        for (s, &p) in parent.iter().enumerate() {
            if p != NONE {
                children[next[p as usize] as usize] = s as u32;
                next[p as usize] += 1;
            }
        }

        // Work: dense LDLᵀ of the c×c block, the triangular solve of the p×c block and the
        // p×p update (lower half), in flops.
        let mut node_work = vec![0.0; n_super];
        let mut subtree_work = vec![0.0; n_super];
        for s in 0..n_super {
            let c = (end[s] - begin[s]) as f64;
            let p = supernodal.supernode(s).pattern().len() as f64;
            node_work[s] = c * c * c / 3.0 + c * c * p + c * p * p;
            subtree_work[s] += node_work[s];
            if parent[s] != NONE {
                // Children are numbered before their parent.
                subtree_work[parent[s] as usize] += subtree_work[s];
            }
        }
        let total_work = roots.iter().map(|&r| subtree_work[r as usize]).sum();

        // Descendants updating each supernode: the distinct supernodes of a supernode's
        // pattern rows (sorted, so equal supernodes are consecutive).
        let mut updater_ptr = vec![0u32; n_super + 1];
        let mut incoming_work = vec![0.0; n_super];
        let mut pairs: Vec<(u32, u32)> = Vec::new();
        for d in 0..n_super {
            let pattern = supernodal.supernode(d).pattern();
            let c_d = (end[d] - begin[d]) as f64;
            let mut k = 0;
            while k < pattern.len() {
                let target = index_to_super[pattern[k] as usize];
                let target_end = end[target as usize];
                let m = pattern[k..].partition_point(|&row| row < target_end);
                let r = (pattern.len() - k) as f64;
                incoming_work[target as usize] += c_d * m as f64 * (r - m as f64 / 2.0);
                pairs.push((target, d as u32));
                updater_ptr[target as usize + 1] += 1;
                k += m;
            }
        }
        for s in 0..n_super {
            updater_ptr[s + 1] += updater_ptr[s];
        }
        let mut next = updater_ptr[..n_super].to_vec();
        let mut updaters = vec![0u32; pairs.len()];
        for (target, d) in pairs {
            updaters[next[target as usize] as usize] = d;
            next[target as usize] += 1;
        }

        // Lower triangle of the permuted matrix as positions into the pattern's values.
        let permuted = |i: u32| perm_inv.map_or(i, |inv| inv[i as usize] as u32);
        let mut a_col_ptr = vec![0u32; n + 1];
        for col in 0..n {
            let pc = permuted(col as u32);
            for &row in pattern.column(col) {
                let pr = permuted(row);
                a_col_ptr[pr.min(pc) as usize + 1] += 1;
            }
        }
        for i in 0..n {
            a_col_ptr[i + 1] += a_col_ptr[i];
        }
        let nnz = a_col_ptr[n] as usize;
        let mut next = a_col_ptr[..n].to_vec();
        let mut a_row = vec![0u32; nnz];
        let mut a_src = vec![0u32; nnz];
        let col_ptr = pattern.col_ptr();
        for (col, &first) in col_ptr[..n].iter().enumerate() {
            let pc = permuted(col as u32);
            for (k, &row) in pattern.column(col).iter().enumerate() {
                let pr = permuted(row);
                let (lo, hi) = (pr.min(pc), pr.max(pc));
                let slot = next[lo as usize] as usize;
                a_row[slot] = hi;
                a_src[slot] = first + k as u32;
                next[lo as usize] += 1;
            }
        }

        Self {
            child_ptr,
            children,
            roots,
            node_work,
            subtree_work,
            total_work,
            incoming_work,
            updater_ptr,
            updaters,
            a_col_ptr,
            a_row,
            a_src,
        }
    }

    fn updaters(&self, s: usize) -> &[u32] {
        &self.updaters[self.updater_ptr[s] as usize..self.updater_ptr[s + 1] as usize]
    }

    fn children(&self, s: usize) -> &[u32] {
        &self.children[self.child_ptr[s] as usize..self.child_ptr[s + 1] as usize]
    }

    /// Flops of one factorization (dense operations only).
    pub fn total_work(&self) -> f64 {
        self.total_work
    }
}

struct Context<'a> {
    supernodal: &'a SymbolicSupernodalCholesky<FactorIndex>,
    tree: &'a TreeSymbolic,
    values: &'a [f64],
    /// The factor's values split into the ranges of the supernodes; the task that
    /// factorizes a supernode takes its range.
    factor: Vec<Mutex<Option<&'a mut [f64]>>>,
    /// The factorized supernodes, read by the supernodes they update (left-looking).
    done: Vec<OnceLock<&'a [f64]>>,
    threads: usize,
    regularization: Regularization<'a>,
    par: Par,
    /// Subtrees with at least this much work are split over tasks.
    parallel_subtree: f64,
    pool: &'a Workspace,
    /// The run's token, looked at before every supernode.
    cancel: Option<plxs_core::CancelToken>,
}

/// Why a factorization ended without a factor.
#[derive(Clone, Copy, Debug)]
pub enum FactorError {
    /// A zero pivot without regularization.
    Ldlt(LdltError),
    /// The run was stopped ([`plxs_core::CancelToken`]).
    Cancelled,
}

impl From<LdltError> for FactorError {
    fn from(error: LdltError) -> Self {
        Self::Ldlt(error)
    }
}

/// Scratch buffers of the dense products, kept from one factorization to the next.
///
/// The allocator hands large blocks back to the operating system when they are freed, and
/// a fresh block costs a page fault per 4 KiB page when it is first written (several
/// microseconds in the cloud VM). Freed buffers go back to the workspace and are handed out
/// again best fit, so a refactorization (Newton iterations) touches no new memory.
#[derive(Debug, Default)]
pub struct Workspace {
    free: Mutex<Vec<Vec<f64>>>,
}

impl Workspace {
    /// A buffer of `len` values with undefined contents.
    fn take(&self, len: usize) -> Vec<f64> {
        if len == 0 {
            return Vec::new();
        }
        let mut free = self.free.lock().unwrap_or_else(|e| e.into_inner());
        let best = (free.iter().enumerate())
            .filter(|(_, b)| b.capacity() >= len)
            .min_by_key(|(_, b)| b.capacity())
            .map(|(i, _)| i);
        let mut buffer = match best {
            Some(i) => free.swap_remove(i),
            None => {
                // Nothing large enough: grow the largest free buffer instead of keeping
                // it beside a new one.
                let largest = (free.iter().enumerate())
                    .max_by_key(|(_, b)| b.capacity())
                    .map(|(i, _)| i);
                largest.map_or_else(Vec::new, |i| free.swap_remove(i))
            }
        };
        drop(free);
        // Only a grown part is written; the contents are overwritten by the caller.
        if buffer.len() > len {
            buffer.truncate(len);
        } else {
            buffer.reserve_exact(len - buffer.len());
            buffer.resize(len, 0.0);
        }
        buffer
    }

    fn give_back(&self, buffer: Vec<f64>) {
        if buffer.capacity() > 0 {
            let mut free = self.free.lock().unwrap_or_else(|e| e.into_inner());
            free.push(buffer);
        }
    }

    /// Releases the buffers (after a new symbolic analysis).
    pub fn clear(&mut self) {
        self.free
            .get_mut()
            .unwrap_or_else(|e| e.into_inner())
            .clear();
    }
}

/// Computes the numeric LDLᵀ factor of the matrix with values `values` (upper triangle,
/// pattern of the symbolic analysis) into `factor` in `faer`'s supernodal layout. Returns
/// the number of regularized pivots. A stop of the run of the calling thread
/// ([`plxs_core::CancelToken::current`]) ends it before the next supernode.
pub fn factorize<'a>(
    supernodal: &'a SymbolicSupernodalCholesky<FactorIndex>,
    tree: &'a TreeSymbolic,
    values: &'a [f64],
    factor: &'a mut [f64],
    regularization: Regularization<'a>,
    par: Par,
    workspace: &'a Workspace,
) -> Result<usize, FactorError> {
    assert_eq!(factor.len(), supernodal.len_val());
    let threads = par.degree().max(1);
    let col_ptr_val = supernodal.col_ptr_for_val();
    let mut rest = factor;
    let mut slots = Vec::with_capacity(supernodal.n_supernodes());
    for s in 0..supernodal.n_supernodes() {
        let (head, tail) = rest.split_at_mut((col_ptr_val[s + 1] - col_ptr_val[s]) as usize);
        slots.push(Mutex::new(Some(head)));
        rest = tail;
    }
    let ctx = Context {
        supernodal,
        tree,
        values,
        factor: slots,
        done: (0..supernodal.n_supernodes())
            .map(|_| OnceLock::new())
            .collect(),
        threads,
        regularization,
        par,
        parallel_subtree: (tree.total_work / (16.0 * threads as f64)).max(MIN_PARALLEL_SUBTREE),
        pool: workspace,
        cancel: plxs_core::CancelToken::current(),
    };
    let counts: Vec<usize> = if threads > 1 {
        tree.roots
            .par_iter()
            .map(|&r| subtree(&ctx, r as usize))
            .collect::<Result<_, _>>()?
    } else {
        tree.roots
            .iter()
            .map(|&r| subtree(&ctx, r as usize))
            .collect::<Result<_, _>>()?
    };
    Ok(counts.into_iter().sum())
}

/// Factorizes the subtree rooted at `s`; returns the number of regularized pivots.
fn subtree(ctx: &Context<'_>, s: usize) -> Result<usize, FactorError> {
    let children = ctx.tree.children(s);
    let parallel =
        ctx.threads > 1 && children.len() > 1 && ctx.tree.subtree_work[s] >= ctx.parallel_subtree;
    if !parallel {
        return sequential(ctx, s);
    }
    let counts: Vec<usize> = children
        .par_iter()
        .map(|&c| subtree(ctx, c as usize))
        .collect::<Result<_, _>>()?;
    Ok(counts.into_iter().sum::<usize>() + node(ctx, s)?)
}

/// Factorizes the subtree rooted at `root` on the calling thread, in post-order without
/// recursion (elimination trees of slender structures are deep).
fn sequential(ctx: &Context<'_>, root: usize) -> Result<usize, FactorError> {
    let mut count = 0;
    let mut stack = vec![(root, false)];
    while let Some((s, expanded)) = stack.pop() {
        if expanded {
            count += node(ctx, s)?;
        } else {
            stack.push((s, true));
            stack.extend(ctx.tree.children(s).iter().map(|&c| (c as usize, false)));
        }
    }
    Ok(count)
}

/// Assembles and factorizes one supernode, left-looking: the updates of all descendants
/// that reach into its columns are applied directly from their finished factor columns.
fn node(ctx: &Context<'_>, s: usize) -> Result<usize, FactorError> {
    if ctx
        .cancel
        .as_ref()
        .is_some_and(plxs_core::CancelToken::is_cancelled)
    {
        return Err(FactorError::Cancelled);
    }
    let supernodal = ctx.supernodal;
    let tree = ctx.tree;
    let s_start = supernodal.supernode_begin()[s] as usize;
    let s_end = supernodal.supernode_end()[s] as usize;
    let c = s_end - s_start;
    let pattern = supernodal.supernode(s).pattern();
    let p = pattern.len();
    let ls_values = (ctx.factor[s].lock().unwrap_or_else(|e| e.into_inner()))
        .take()
        .expect("a supernode is factorized once");
    ls_values.fill(0.0);
    let mut ls = MatMut::from_column_major_slice_mut(&mut *ls_values, c + p, c);
    for j in 0..c {
        let col = s_start + j;
        for k in tree.a_col_ptr[col] as usize..tree.a_col_ptr[col + 1] as usize {
            let row = tree.a_row[k] as usize;
            let i = if row < s_end {
                row - s_start
            } else {
                c + pattern
                    .binary_search(&(row as FactorIndex))
                    .expect("row in the structure")
            };
            ls[(i, j)] += ctx.values[tree.a_src[k] as usize];
        }
    }

    // Updates from the descendants. A large supernode is split into column blocks that
    // take all updates into their own columns in parallel.
    let updaters = tree.updaters(s);
    let blocks = if ctx.threads > 1 && tree.incoming_work[s] >= PARALLEL_NODE {
        (4 * ctx.threads).min(c.div_ceil(32)).max(1)
    } else {
        1
    };
    if blocks == 1 {
        for &d in updaters {
            apply_update(ctx, d as usize, s, 0, ls.rb_mut());
        }
    } else {
        let width = c.div_ceil(blocks);
        let mut parts = Vec::with_capacity(blocks);
        let mut rest = ls.rb_mut();
        let mut first = 0;
        while first < c {
            let w = width.min(c - first);
            let (head, tail) = rest.split_at_col_mut(w);
            parts.push((first, head));
            rest = tail;
            first += w;
        }
        parts.into_par_iter().for_each(|(first, part)| {
            let mut part = part;
            for &d in updaters {
                apply_update(ctx, d as usize, s, first, part.rb_mut());
            }
        });
    }

    let node_par = if ctx.par.degree() > 1 && tree.node_work[s] >= PARALLEL_NODE {
        ctx.par
    } else {
        Par::Seq
    };
    let count = factor_front(ctx, s_start, c, p, ls.rb_mut(), node_par)?;
    let shared: &[f64] = ls_values;
    // The factor outlives the factorization; the reference is only read by ancestors.
    let _ = ctx.done[s].set(shared);
    Ok(count)
}

/// Subtracts the update of the finished descendant `d` from the columns
/// `first..first + target.ncols()` of supernode `s` (its front `target`).
fn apply_update(ctx: &Context<'_>, d: usize, s: usize, first: usize, mut target: MatMut<'_, f64>) {
    let supernodal = ctx.supernodal;
    let s_start = supernodal.supernode_begin()[s] as usize;
    let s_end = supernodal.supernode_end()[s] as usize;
    let c = s_end - s_start;
    let s_pattern = supernodal.supernode(s).pattern();
    let col_lo = (s_start + first) as FactorIndex;
    let col_hi = (s_start + first + target.ncols()) as FactorIndex;

    let d_start = supernodal.supernode_begin()[d] as usize;
    let c_d = supernodal.supernode_end()[d] as usize - d_start;
    let d_pattern = supernodal.supernode(d).pattern();
    let start = d_pattern.partition_point(|&row| row < col_lo);
    let m = d_pattern[start..].partition_point(|&row| row < col_hi);
    if m == 0 {
        return;
    }
    let rows = &d_pattern[start..];
    let r = rows.len();
    let ld_values = ctx.done[d].get().expect("descendants are factorized first");
    let ld = MatRef::from_column_major_slice(ld_values, c_d + d_pattern.len(), c_d);
    let below = ld.subrows(c_d + start, r);
    let mid = below.subrows(0, m);

    // w = D Ld_midᵀ (c_d × m, contiguous), t = Ld_below w (r × m; the upper part of its
    // top m × m block is not needed).
    let mut w = ctx.pool.take(c_d * m);
    let mut w_mat = MatMut::from_column_major_slice_mut(&mut w, c_d, m);
    w_mat.copy_from(mid.transpose());
    for k in 0..c_d {
        let dk = ld[(k, k)];
        w_mat.rb_mut().row_mut(k).iter_mut().for_each(|x| *x *= dk);
    }
    let mut t = ctx.pool.take(r * m);
    let mut t_mat = MatMut::from_column_major_slice_mut(&mut t, r, m);
    let par = Par::Seq;
    {
        let (t_top, t_bot) = t_mat.rb_mut().split_at_row_mut(m);
        triangular_matmul(
            t_top,
            BlockStructure::TriangularLower,
            Accum::Replace,
            mid,
            BlockStructure::Rectangular,
            w_mat.rb(),
            BlockStructure::Rectangular,
            1.0,
            par,
        );
        if r > m {
            faer::linalg::matmul::matmul(
                t_bot,
                Accum::Replace,
                below.subrows(m, r - m),
                w_mat.rb(),
                1.0,
                par,
            );
        }
    }
    ctx.pool.give_back(w);

    // Local rows in the front of s: the diagonal block, then the pattern (both sorted, so
    // the pattern positions are found by one forward walk). Consecutive rows that stay
    // consecutive in the front form runs, which are subtracted as slices.
    let mut runs: Vec<(usize, usize, usize)> = Vec::new();
    let mut k = 0;
    for (i, &row) in rows.iter().enumerate() {
        let l = if (row as usize) < s_end {
            row as usize - s_start
        } else {
            while s_pattern[k] < row {
                k += 1;
            }
            c + k
        };
        match runs.last_mut() {
            Some((src, dst, len)) if *src + *len == i && *dst + *len == l => *len += 1,
            _ => runs.push((i, l, 1)),
        }
    }
    for (j, &row) in rows[..m].iter().enumerate() {
        let col = row as usize - s_start - first;
        let column = t_mat
            .rb()
            .col(j)
            .try_as_col_major()
            .expect("contiguous")
            .as_slice();
        let dst = target
            .rb_mut()
            .col_mut(col)
            .try_as_col_major_mut()
            .expect("contiguous")
            .as_slice_mut();
        for &(src, to, len) in &runs {
            if src + len <= j {
                continue;
            }
            let skip = j.saturating_sub(src);
            let (src, to, len) = (src + skip, to + skip, len - skip);
            for (x, v) in dst[to..to + len].iter_mut().zip(&column[src..src + len]) {
                *x -= v;
            }
        }
    }
    ctx.pool.give_back(t);
}

/// Dense LDLᵀ of the diagonal block of a front, the triangular solve and scaling of the
/// block below it; returns the number of regularized pivots.
fn factor_front(
    ctx: &Context<'_>,
    s_start: usize,
    c: usize,
    p: usize,
    mut ls: MatMut<'_, f64>,
    node_par: Par,
) -> Result<usize, LdltError> {
    let (mut top, mut bot) = ls.rb_mut().split_at_row_mut(c);
    let s_end = s_start + c;
    let regularization = LdltRegularization {
        dynamic_regularization_signs: ctx.regularization.signs.map(|signs| &signs[s_start..s_end]),
        dynamic_regularization_delta: ctx.regularization.delta,
        dynamic_regularization_epsilon: ctx.regularization.epsilon,
    };
    let mut buffer = MemBuffer::new(cholesky_in_place_scratch::<f64>(
        c,
        node_par,
        Default::default(),
    ));
    let info = cholesky_in_place(
        top.rb_mut(),
        regularization,
        node_par,
        MemStack::new(&mut buffer),
        Default::default(),
    )
    .map_err(|LdltError::ZeroPivot { index }| LdltError::ZeroPivot {
        index: index + s_start,
    })?;
    for j in 0..c {
        for i in 0..j {
            top[(i, j)] = 0.0;
        }
    }
    if p > 0 {
        let mut wt = ctx.pool.take(c * p);
        let mut wt_mat = MatMut::from_column_major_slice_mut(&mut wt, c, p);
        wt_mat.copy_from(bot.rb().transpose());
        solve_unit_lower_triangular_in_place(top.rb(), wt_mat.rb_mut(), node_par);
        for j in 0..c {
            let d = top[(j, j)].recip();
            let column = bot
                .rb_mut()
                .col_mut(j)
                .try_as_col_major_mut()
                .expect("contiguous");
            for (i, x) in column.iter_mut().enumerate() {
                *x = wt_mat[(j, i)] * d;
            }
        }
        ctx.pool.give_back(wt);
    }
    Ok(info.dynamic_regularization_count)
}

#[cfg(test)]
mod tests {
    use crate::{LdltSolver, LinearSolver, Pattern, SparseMatrix};
    use std::sync::Arc;

    /// A three-dimensional grid of `n³` points with three unknowns each, coupled like a
    /// hexahedral mesh (all 27 neighbors), with a stiffness-like matrix: diagonally
    /// dominant by `shift` (negative values make it indefinite).
    fn grid(n: usize, shift: f64) -> (Arc<Pattern>, SparseMatrix, Vec<[f64; 3]>) {
        let id = |i: usize, j: usize, k: usize| (i * n + j) * n + k;
        let dofs = 3 * n * n * n;
        let mut blocks = Vec::new();
        for i in 0..n - 1 {
            for j in 0..n - 1 {
                for k in 0..n - 1 {
                    let mut block = Vec::new();
                    for (a, b, c) in
                        (0..8).map(|corner| (corner & 1, (corner >> 1) & 1, corner >> 2))
                    {
                        let node = id(i + a, j + b, k + c);
                        block.extend((0..3).map(|d| (3 * node + d) as u32));
                    }
                    blocks.push(block);
                }
            }
        }
        let pattern = Arc::new(Pattern::from_blocks(dofs, blocks.iter().map(|b| &b[..])));
        let mut a = SparseMatrix::zeros(pattern.clone());
        for col in 0..dofs {
            for &row in pattern.column(col) {
                let row = row as usize;
                let value = if row == col {
                    27.0 + shift
                } else {
                    -1.0 / (1.0 + ((row * 7 + col * 3) % 5) as f64)
                };
                a.add(row as u32, col as u32, value);
            }
        }
        let coordinates = (0..dofs)
            .map(|dof| {
                let node = dof / 3;
                [
                    (node / (n * n)) as f64,
                    ((node / n) % n) as f64,
                    (node % n) as f64,
                ]
            })
            .collect();
        (pattern, a, coordinates)
    }

    fn multiply(a: &SparseMatrix, x: &[f64]) -> Vec<f64> {
        let pattern = a.pattern();
        let mut y = vec![0.0; x.len()];
        let col_ptr = pattern.col_ptr();
        for col in 0..x.len() {
            for (k, &row) in pattern.column(col).iter().enumerate() {
                let v = a.values()[col_ptr[col] as usize + k];
                let row = row as usize;
                y[row] += v * x[col];
                if row != col {
                    y[col] += v * x[row];
                }
            }
        }
        y
    }

    fn check(n: usize, shift: f64, threads: usize, force: bool) {
        let (pattern, a, coordinates) = grid(n, shift);
        let dofs = pattern.dimension();
        let x: Vec<f64> = (0..dofs).map(|i| ((i * 13) % 17) as f64 - 8.0).collect();
        let b = multiply(&a, &x);
        let mut solver = LdltSolver::new(threads);
        if force {
            solver = solver.force_supernodal();
        }
        solver.set_coordinates(Some(coordinates));
        solver.analyze(&pattern).unwrap();
        // Twice: a refactorization reuses the workspace and must give the same factor.
        for _ in 0..2 {
            solver.factorize(&a).unwrap();
            let mut y = b.clone();
            solver.solve(&mut y).unwrap();
            let error = (y.iter().zip(&x))
                .map(|(y, x)| (y - x).abs())
                .fold(0.0, f64::max);
            assert!(
                error < 1e-9,
                "n {n}, shift {shift}, {threads} threads: error {error}"
            );
        }
    }

    #[test]
    fn tree_parallel_factorization_solves_grids() {
        // 8³ points: 1536 equations, AMD and supernodal only when forced; 16³: 12288
        // equations, nested dissection and supernodal on its own.
        for threads in [1, 4] {
            check(8, 0.0, threads, true);
            check(16, 0.0, threads, false);
        }
    }

    #[test]
    fn tree_parallel_factorization_handles_indefinite_matrices() {
        // Shifted into the spectrum: negative pivots, no zero ones.
        check(8, -27.3, 4, true);
    }

    #[test]
    fn small_systems_factorize_supernodally_when_forced() {
        // The 3 × 3 system of the solver tests through the tree-parallel path.
        let pattern = Arc::new(Pattern::from_blocks(3, [&[0u32, 1][..], &[1, 2][..]]));
        let mut a = SparseMatrix::zeros(pattern.clone());
        a.add(0, 0, 4.0);
        a.add(1, 1, 3.0);
        a.add(2, 2, 2.0);
        a.add(0, 1, 1.0);
        a.add(2, 1, 1.0);
        let mut solver = LdltSolver::new(2).force_supernodal();
        solver.analyze(&pattern).unwrap();
        solver.factorize(&a).unwrap();
        let mut b = vec![6.0, 10.0, 8.0];
        solver.solve(&mut b).unwrap();
        for (x, expected) in b.iter().zip([1.0, 2.0, 3.0]) {
            assert!((x - expected).abs() < 1e-12, "{b:?}");
        }
        // A singular matrix is reported, not factorized.
        let pattern = Arc::new(Pattern::from_blocks(2, [&[0u32, 1][..]]));
        let mut a = SparseMatrix::zeros(pattern.clone());
        a.add(0, 0, 1.0);
        a.add(0, 1, 1.0);
        a.add(1, 1, 1.0);
        let mut solver = LdltSolver::new(1).force_supernodal();
        solver.analyze(&pattern).unwrap();
        assert!(solver.factorize(&a).is_err());
    }

    #[test]
    fn a_stopped_run_ends_the_factorization() {
        let pattern = Arc::new(Pattern::from_blocks(3, [&[0u32, 1][..], &[1, 2][..]]));
        let mut a = SparseMatrix::zeros(pattern.clone());
        a.add(0, 0, 4.0);
        a.add(1, 1, 3.0);
        a.add(2, 2, 2.0);
        a.add(0, 1, 1.0);
        a.add(2, 1, 1.0);
        let mut solver = LdltSolver::new(2).force_supernodal();
        solver.analyze(&pattern).unwrap();
        let cancel = plxs_core::CancelToken::new();
        // Outside of a run, and in a run that goes on, the factorization is computed.
        solver.factorize(&a).unwrap();
        cancel.scope(|| solver.factorize(&a)).unwrap();
        cancel.cancel();
        let stopped = cancel.scope(|| solver.factorize(&a));
        assert!(
            matches!(stopped, Err(plxs_core::Error::Cancelled)),
            "{stopped:?}"
        );
        // The token is the run's: once the scope is left, nothing is stopped.
        assert!(plxs_core::CancelToken::current().is_none());
        solver.factorize(&a).unwrap();
    }
}
