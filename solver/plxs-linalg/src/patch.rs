//! Local direct solves where the multigrid hangs (point C2 of the solver review).
//!
//! On a part with a slender feature (a bar of a bracket two elements thick, a thin web)
//! the aggregation multigrid converges well everywhere but there: the error the V-cycle
//! leaves after a few cycles sits almost entirely in a small group of nodes (on the
//! bracket 99.8 % of it in 1 % of the nodes). Such a region is small enough to factorize
//! (10 000 unknowns in a tenth of a second), and solving it exactly inside every cycle
//! removes the slow mode at its source.
//!
//! The region is found from the hierarchy itself, without knowledge of the geometry, and
//! only when a solve needs it: a solve whose rate after [`CHECK_AT`] iterations predicts
//! more than [`SLOW_ITERATIONS`] runs some cycles of the stationary iteration on a random
//! error with the right-hand side zero ([`TEST_ROUNDS`]), which damp everything the
//! hierarchy handles well; what is left is the slow error. The blocks (nodes) that carry
//! most of it, plus a ring of neighbours, form the patch when they are few; otherwise the
//! hierarchy stays as it is. The solve then starts again from its iterate with the
//! patch, and later solves with the hierarchy (Newton iterations) keep it.
//!
//! In the cycle of the finest level the patch acts as a multiplicative Schwarz step on
//! both sides of the coarse correction, which keeps the preconditioner symmetric:
//! pre-smoothing, patch solve on the residual (its effect on the residual updated
//! locally), coarse correction, patch solve on the patch rows' residual, post-smoothing.
//! Both patch steps touch only the patch and its neighbours, a small fraction of a
//! product with the whole matrix.

use std::sync::Arc;

use plxs_core::Result;

use crate::bsr::Bsr;
use crate::solver::LinearSolver;
use crate::{LdltSolver, Pattern, SparseMatrix};

/// The iteration of a solve at which it looks for a slow region.
pub(crate) const CHECK_AT: usize = 12;

/// Cycles of the stationary iteration on a random error, in rounds: the region is looked
/// for after each round, and no further round runs once the contraction stops growing.
pub(crate) const TEST_ROUNDS: [usize; 3] = [12, 4, 4];

/// A test error that contracts by less than this per cycle is no slow error.
pub(crate) const SLOW_CYCLE: f64 = 0.8;

/// It does when its rate so far predicts more than this many iterations to the
/// tolerance (a solid block takes 20 to 30).
pub(crate) const SLOW_ITERATIONS: f64 = 40.0;

/// The patch holds the fewest blocks that carry this share of the test error (squared).
const CAPTURE: f64 = 0.97;

/// Those blocks may be at most this share of all blocks: a slow error spread over the
/// whole part (a plate one element thick) is not local, a patch cannot hold it.
const MOST_SHARE: f64 = 0.03;

/// Rings of neighbours added around the blocks of the error.
const RINGS: usize = 1;

/// At most this many unknowns in the patch (with its rings).
const MOST_UNKNOWNS: usize = 60_000;

/// A factorized group of blocks of the finest operator.
pub(crate) struct Patch {
    /// The block rows of the patch, ascending.
    blocks: Vec<u32>,
    br: usize,
    pattern: Arc<Pattern>,
    /// Per stored entry of the local matrix (upper triangle) its value in the operator.
    src: Vec<usize>,
    solver: LdltSolver,
}

impl Patch {
    /// The patch of the slow error `error` (a vector of `a`'s length); `None` when the
    /// error is not local. `coordinates` are the positions of the unknowns (for the
    /// ordering of the direct solver).
    pub(crate) fn find(
        a: &Bsr,
        error: &[f64],
        coordinates: Option<&[[f64; 3]]>,
        trace: bool,
    ) -> Result<Option<Patch>> {
        let br = a.br;
        let blocks = a.brows;
        if blocks == 0 || a.br != a.bc {
            return Ok(None);
        }
        let mut energy: Vec<(f64, u32)> = (0..blocks)
            .map(|b| {
                let e: f64 = error[b * br..(b + 1) * br].iter().map(|v| v * v).sum();
                (e, b as u32)
            })
            .collect();
        let total: f64 = energy.iter().map(|e| e.0).sum();
        if total <= 0.0 || !total.is_finite() {
            return Ok(None);
        }
        energy.sort_unstable_by(|x, y| y.0.total_cmp(&x.0));
        let mut sum = 0.0;
        let mut core = Vec::new();
        for &(e, b) in &energy {
            if sum >= CAPTURE * total {
                break;
            }
            sum += e;
            core.push(b);
        }
        if core.len() as f64 > MOST_SHARE * blocks as f64 {
            if trace {
                eprintln!(
                    "pcg: slow modes spread over {} of {blocks} nodes: no patch",
                    core.len()
                );
            }
            return Ok(None);
        }
        let mut inside = vec![false; blocks];
        for &b in &core {
            inside[b as usize] = true;
        }
        let core_len = core.len();
        let mut members = core;
        for _ in 0..RINGS {
            let mut ring = Vec::new();
            for &b in &members {
                for &c in a.block_row(b as usize).0 {
                    if !inside[c as usize] {
                        inside[c as usize] = true;
                        ring.push(c);
                    }
                }
            }
            members.extend(ring);
        }
        if members.len() * br > MOST_UNKNOWNS {
            if trace {
                eprintln!(
                    "pcg: slow region of {} unknowns larger than a patch may be",
                    members.len() * br
                );
            }
            return Ok(None);
        }
        members.sort_unstable();
        let started = std::time::Instant::now();
        let patch = Self::new(a, members, coordinates)?;
        if trace {
            eprintln!(
                "pcg: slow modes in {} nodes: patch of {} unknowns factorized in {:.3} s",
                core_len,
                patch.blocks.len() * br,
                started.elapsed().as_secs_f64()
            );
        }
        Ok(Some(patch))
    }

    /// The patch of the block rows `blocks` (ascending), factorized.
    fn new(a: &Bsr, blocks: Vec<u32>, coordinates: Option<&[[f64; 3]]>) -> Result<Patch> {
        let br = a.br;
        let mut local = vec![u32::MAX; a.brows];
        for (p, &b) in blocks.iter().enumerate() {
            local[b as usize] = p as u32;
        }
        let n = blocks.len() * br;
        // Column q·br + cj of the upper triangle: the rows p·br + ci <= it of the blocks
        // (P, Q) of the patch, read from the block (Q, P) of the symmetric operator.
        let mut col_ptr = Vec::with_capacity(n + 1);
        let mut row_idx = Vec::new();
        let mut src = Vec::new();
        col_ptr.push(0u32);
        let size = br * br;
        for (q, &bq) in blocks.iter().enumerate() {
            let (s, e) = (a.ptr[bq as usize], a.ptr[bq as usize + 1]);
            for cj in 0..br {
                let column = q * br + cj;
                for k in s..e {
                    let p = local[a.col[k] as usize];
                    if p == u32::MAX || p as usize > q {
                        continue;
                    }
                    for ci in 0..br {
                        let row = p as usize * br + ci;
                        if row > column {
                            continue;
                        }
                        row_idx.push(row as u32);
                        src.push(k * size + cj * br + ci);
                    }
                }
                col_ptr.push(row_idx.len() as u32);
            }
        }
        let pattern = Arc::new(Pattern::from_compressed(n, col_ptr, row_idx));
        let mut solver = LdltSolver::new(rayon::current_num_threads());
        if let Some(c) = coordinates {
            let positions: Vec<[f64; 3]> = (blocks.iter())
                .flat_map(|&b| (0..br).map(move |i| c[b as usize * br + i]))
                .collect();
            solver.set_coordinates(Some(positions));
        }
        solver.analyze(&pattern)?;
        let mut patch = Patch {
            blocks,
            br,
            pattern,
            src,
            solver,
        };
        patch.refresh(a)?;
        Ok(patch)
    }

    /// Factorizes the patch of new values of the operator (same pattern).
    pub(crate) fn refresh(&mut self, a: &Bsr) -> Result<()> {
        let mut m = SparseMatrix::zeros(self.pattern.clone());
        for (v, &s) in m.values_mut().iter_mut().zip(&self.src) {
            *v = a.val[s];
        }
        self.solver.factorize(&m)
    }

    /// Unknowns of the patch.
    pub(crate) fn len(&self) -> usize {
        self.blocks.len() * self.br
    }

    /// After the pre-smoothing: `x += δ` with `δ` the patch solve of the residual `r`,
    /// and `r -= A δ` (on the patch and its neighbours only).
    pub(crate) fn correct_residual(
        &self,
        a: &Bsr<f32>,
        x: &mut [f32],
        r: &mut [f32],
    ) -> Result<()> {
        let br = self.br;
        let mut delta: Vec<f64> = (self.blocks.iter())
            .flat_map(|&b| (0..br).map(move |i| b as usize * br + i))
            .map(|i| r[i] as f64)
            .collect();
        self.solver.solve(&mut delta)?;
        let size = br * br;
        for (p, &b) in self.blocks.iter().enumerate() {
            let d = &delta[p * br..(p + 1) * br];
            for (i, v) in d.iter().enumerate() {
                x[b as usize * br + i] += *v as f32;
            }
            // Column block b of A times δ_b: rows c of the blocks (b, c), transposed.
            let (s, e) = (a.ptr[b as usize], a.ptr[b as usize + 1]);
            for k in s..e {
                let c = a.col[k] as usize;
                let block = &a.val[k * size..(k + 1) * size];
                // A(c, b) = A(b, c)ᵀ: r_c -= A(b, c)ᵀ δ_b.
                for ci in 0..br {
                    let mut sum = 0.0f64;
                    for (cj, dv) in d.iter().enumerate() {
                        sum += block[cj * br + ci] as f64 * dv;
                    }
                    r[c * br + ci] -= sum as f32;
                }
            }
        }
        Ok(())
    }

    /// After the coarse correction: `x += δ` with `δ` the patch solve of the residual
    /// `b - A x` on the patch rows.
    pub(crate) fn correct(&self, a: &Bsr<f32>, b: &[f32], x: &mut [f32]) -> Result<()> {
        let br = self.br;
        let size = br * br;
        let mut rhs = vec![0.0f64; self.len()];
        for (p, &row) in self.blocks.iter().enumerate() {
            let (s, e) = (a.ptr[row as usize], a.ptr[row as usize + 1]);
            for i in 0..br {
                let mut sum = b[row as usize * br + i] as f64;
                for k in s..e {
                    let c = a.col[k] as usize;
                    let block = &a.val[k * size..(k + 1) * size];
                    for j in 0..br {
                        sum -= block[i * br + j] as f64 * x[c * br + j] as f64;
                    }
                }
                rhs[p * br + i] = sum;
            }
        }
        self.solver.solve(&mut rhs)?;
        for (p, &row) in self.blocks.iter().enumerate() {
            for i in 0..br {
                x[row as usize * br + i] += rhs[p * br + i] as f32;
            }
        }
        Ok(())
    }
}
