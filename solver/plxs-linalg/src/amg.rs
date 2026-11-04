//! Algebraic multigrid by smoothed aggregation, the preconditioner of the iterative
//! solver for large models.
//!
//! The nodes are grouped into aggregates of neighbours (a node's unknowns stay together);
//! on every aggregate the near-nullspace of the operator, the rigid body modes of
//! elasticity (three translations and three rotations, or the constant for a scalar field
//! like temperature), is orthonormalized and becomes the columns of the tentative
//! prolongator, which one damped Jacobi step smooths. The coarse operator is the Galerkin
//! product `Pᵀ A P`; the coarsest system is factorized. One V-cycle with a Chebyshev
//! polynomial smoother (symmetric, so that the preconditioner is symmetric positive
//! definite and conjugate gradients apply) per iteration. Every matrix of the hierarchy
//! is stored in blocks ([`Bsr`]): `3 × 3` per node pair on the finest level, `6 × 6` per
//! aggregate pair above it, `3 × 6` for the prolongator between them. The hierarchy is
//! built in double precision and kept in single precision (the finest operator included),
//! which halves the memory traffic of a cycle; conjugate gradients outside stay in double
//! precision, the preconditioner only has to be a fixed symmetric positive definite
//! operator.
//!
//! Sources: Vaněk, Mandel, Brezina, "Algebraic multigrid by smoothed aggregation for
//! second and fourth order elliptic problems", Computing 56 (1996); the aggregation phases
//! and the smoothing weight `4/(3 λ_max)` follow that paper. The Chebyshev smoother is the
//! three-term recurrence of Saad, "Iterative Methods for Sparse Linear Systems", 2nd ed.,
//! section 12.3, on the upper part of the spectrum of `D⁻¹ A` as in Adams, Brezina, Hu,
//! Tuminaro, "Parallel multigrid smoothing: polynomial versus Gauss-Seidel", JCP 188
//! (2003). No CalculiX code was consulted.

use std::sync::{Arc, Mutex};

use rayon::prelude::*;

use plxs_core::{Error, Result};

use crate::bsr::{BlockAccumulator, Bsr, Scalar};
use crate::solver::LinearSolver;
use crate::{LdltSolver, Pattern, SparseMatrix};

/// What the unknowns of a system are, so that the preconditioner can keep the unknowns of
/// a node together and knows the rigid body modes: for every unknown its node (any dense
/// numbering), its component (0, 1, 2 for the displacements; 0 for a scalar field;
/// `u8::MAX` for a padding unknown without a mode) and the position of its node.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Unknowns {
    pub node: Vec<u32>,
    pub component: Vec<u8>,
    pub coordinates: Vec<[f64; 3]>,
}

impl Unknowns {
    /// The unknowns `free` (ascending global DOF numbers, DOF = node · `per_node` +
    /// component) of nodes at `coordinates`.
    pub fn from_free(free: &[u32], per_node: usize, coordinates: &[[f64; 3]]) -> Self {
        let per_node = per_node.max(1) as u32;
        Self {
            node: free.iter().map(|&d| d / per_node).collect(),
            component: free.iter().map(|&d| (d % per_node) as u8).collect(),
            coordinates: free
                .iter()
                .map(|&d| coordinates[(d / per_node) as usize])
                .collect(),
        }
    }
}

/// Tuning of the hierarchy.
#[derive(Clone, Copy, Debug)]
pub struct AmgOptions {
    /// Two nodes are strongly connected when the Frobenius norm of their coupling block
    /// is at least this fraction of the geometric mean of their diagonal blocks' norms.
    pub strength: f64,
    /// Degree of the Chebyshev smoother (matrix-vector products per smoothing).
    pub degree: usize,
    /// The coarsest level is the first with at most this many unknowns.
    pub coarse_size: usize,
    pub max_levels: usize,
    /// At most this many nodes found an aggregate together; the strongest neighbours of
    /// the root first, the rest joins later.
    pub max_aggregate: usize,
}

impl Default for AmgOptions {
    /// The defaults, each overridable for measurements by an environment variable
    /// (`PLXS_AMG_STRENGTH`, `PLXS_AMG_DEGREE`, `PLXS_AMG_COARSE`, `PLXS_AMG_LEVELS`,
    /// `PLXS_AMG_MAX_AGG`).
    fn default() -> Self {
        fn env<T: std::str::FromStr>(name: &str, default: T) -> T {
            std::env::var(name)
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(default)
        }
        Self {
            strength: env("PLXS_AMG_STRENGTH", 0.01),
            degree: env("PLXS_AMG_DEGREE", 2),
            coarse_size: env("PLXS_AMG_COARSE", 1500),
            max_levels: env("PLXS_AMG_LEVELS", 12),
            max_aggregate: env("PLXS_AMG_MAX_AGG", 27),
        }
    }
}

/// The near-nullspace of a level: `k` vectors of length `n` (row major, `n × k`); the
/// unknowns are grouped by the blocks of the level's matrix.
struct Nullspace {
    k: usize,
    vectors: Vec<f64>,
}

struct Level {
    /// The operator of this level; `None` on the finest level, whose copy is [`Amg::fine`].
    a: Option<Bsr<f32>>,
    inv_diag: Vec<f32>,
    /// Largest eigenvalue of `D⁻¹ A` (estimated, with a margin).
    lambda_max: f64,
    /// Prolongation from the next coarser level and its transpose; `None` on the coarsest.
    p: Option<Bsr<f32>>,
    pt: Option<Bsr<f32>>,
}

struct Work {
    x: Vec<f32>,
    b: Vec<f32>,
    r: Vec<f32>,
    d: Vec<f32>,
    t: Vec<f32>,
    /// Right-hand side and solution of the coarsest level in double precision.
    coarse: Vec<f64>,
}

impl Work {
    fn new(n: usize, coarsest: bool) -> Self {
        Self {
            x: vec![0.0; n],
            b: vec![0.0; n],
            r: vec![0.0; n],
            d: vec![0.0; n],
            t: vec![0.0; n],
            coarse: vec![0.0; if coarsest { n } else { 0 }],
        }
    }
}

/// The multigrid hierarchy of one matrix.
pub struct Amg {
    /// The finest operator in single precision.
    fine: Bsr<f32>,
    levels: Vec<Level>,
    coarse: LdltSolver,
    work: Mutex<Vec<Work>>,
    options: AmgOptions,
    /// Single precision values of all operators and prolongators of the hierarchy, the
    /// copy of the finest operator included.
    nonzeros: usize,
}

impl Amg {
    /// Builds the hierarchy of the symmetric positive definite `a` (full storage, square
    /// blocks). `unknowns`, when given, has one entry per row of `a` and the unknowns of a
    /// block belong to one node.
    pub fn build(a: &Bsr, unknowns: Option<&Unknowns>, options: AmgOptions) -> Result<Amg> {
        let n = a.rows();
        let mut nullspace = match unknowns {
            Some(u) if u.component.len() == n => rigid_body_modes(u),
            _ => Nullspace {
                k: 1,
                vectors: vec![1.0; n],
            },
        };
        let mut levels: Vec<Level> = Vec::new();
        let mut nonzeros = a.nnz();
        let mut current: Option<Bsr> = None;
        let trace = std::env::var("PLXS_AMG_TRACE").is_ok();
        let clock = std::time::Instant::now();
        let mut last = 0.0;
        let mut lap = |what: &str, n: usize| {
            if trace {
                let now = clock.elapsed().as_secs_f64();
                eprintln!("amg: {what} on {n} unknowns: {:.3} s", now - last);
                last = now;
            }
        };
        loop {
            let a_l = current.as_ref().unwrap_or(a);
            let n_l = a_l.rows();
            let inv_diag: Vec<f64> = a_l
                .diagonal()
                .iter()
                .map(|&d| if d > 0.0 { 1.0 / d } else { 1.0 })
                .collect();
            let inv_diag_32: Vec<f32> = inv_diag.iter().map(|&d| d as f32).collect();
            let lambda = largest_eigenvalue(a_l, &inv_diag);
            lap("eigenvalue", n_l);
            let coarsest = n_l <= options.coarse_size || levels.len() + 1 >= options.max_levels;
            let coarser = if coarsest {
                None
            } else {
                let aggregates = aggregate(a_l, options.strength, options.max_aggregate);
                lap("aggregation", n_l);
                let n_c = aggregates.count * nullspace.k;
                if n_c * 10 >= n_l * 9 || n_c == 0 {
                    None
                } else {
                    let (p_t, ns_c) = tentative(a_l, &aggregates, &nullspace);
                    lap("tentative prolongator", n_l);
                    let p = smooth_prolongator(a_l, &inv_diag, lambda, &p_t);
                    let pt = p.transpose();
                    lap("smoothed prolongator", n_l);
                    let a_c = galerkin(a_l, &p, &pt);
                    lap("galerkin product", n_l);
                    if trace {
                        eprintln!(
                            "amg: A {} blocks, P {} blocks, Ac {} x {} blocks",
                            a_l.blocks(),
                            p.blocks(),
                            a_c.brows,
                            a_c.blocks()
                        );
                    }
                    nonzeros += p.nnz() * 2 + a_c.nnz();
                    Some((p, pt, a_c, ns_c))
                }
            };
            match coarser {
                None => {
                    levels.push(Level {
                        a: current.as_ref().map(Bsr::converted),
                        inv_diag: inv_diag_32,
                        lambda_max: lambda,
                        p: None,
                        pt: None,
                    });
                    break;
                }
                Some((p, pt, a_c, ns_c)) => {
                    levels.push(Level {
                        a: current.as_ref().map(Bsr::converted),
                        inv_diag: inv_diag_32,
                        lambda_max: lambda,
                        p: Some(p.converted()),
                        pt: Some(pt.converted()),
                    });
                    current = Some(a_c);
                    nullspace = ns_c;
                }
            }
        }
        // The coarsest system is factorized; when the coarsening stalled on a big level
        // (every connection weak, a pathological matrix) that would cost the memory of
        // the direct solver, so the caller takes another preconditioner.
        let coarsest = current.as_ref().unwrap_or(a);
        if coarsest.rows() > options.coarse_size * 20 {
            return Err(Error::Numerical {
                step: 0,
                increment: 0,
                message: format!(
                    "multigrid: the coarsening stalled at {} unknowns",
                    coarsest.rows()
                ),
            });
        }
        let (pattern, matrix) = upper_triangle(coarsest);
        let mut coarse = LdltSolver::new(1);
        coarse.analyze(&pattern)?;
        let factorized = coarse.factorize(&matrix);
        lap("coarsest factorization", coarsest.rows());
        factorized.map_err(|e| Error::Numerical {
            step: 0,
            increment: 0,
            message: format!("multigrid: the coarsest system is not positive definite ({e})"),
        })?;
        // The Galerkin product of a positive definite matrix is positive definite; a
        // negative pivot means the matrix itself is not, and the smoothers would diverge.
        if coarse.inertia().is_some_and(|negative| negative > 0) {
            return Err(Error::Numerical {
                step: 0,
                increment: 0,
                message: "multigrid: the matrix is not positive definite".to_owned(),
            });
        }
        let last = levels.len() - 1;
        let work = levels
            .iter()
            .enumerate()
            .map(|(l, level)| Work::new(level.inv_diag.len(), l == last))
            .collect();
        let fine = a.converted();
        lap("single precision copy", n);
        Ok(Amg {
            fine,
            levels,
            coarse,
            work: Mutex::new(work),
            options,
            nonzeros,
        })
    }

    pub fn levels(&self) -> usize {
        self.levels.len()
    }

    /// Adapts the hierarchy to new values of the finest operator `a` (same pattern) without
    /// rebuilding it: the smoother of the finest level (its single precision copy, the
    /// inverse diagonal and the eigenvalue estimate) follows the new values, the
    /// prolongators and the coarse operators stay those of the matrix the hierarchy was
    /// built on. Meant for the Newton iterations of a step, whose matrices change
    /// little; the caller watches the iteration counts and rebuilds when they grow.
    pub fn refresh(&mut self, a: &Bsr) {
        let level = &mut self.levels[0];
        let inv_diag: Vec<f64> = a
            .diagonal()
            .iter()
            .map(|&d| if d > 0.0 { 1.0 / d } else { 1.0 })
            .collect();
        level.lambda_max = largest_eigenvalue(a, &inv_diag);
        level.inv_diag = inv_diag.iter().map(|&d| d as f32).collect();
        convert(&a.val, &mut self.fine.val);
    }

    pub fn nonzeros(&self) -> usize {
        self.nonzeros
    }

    /// Sizes of the levels, finest first.
    pub fn sizes(&self) -> Vec<usize> {
        self.levels.iter().map(|l| l.inv_diag.len()).collect()
    }

    /// `z ≈ A⁻¹ r` by one V-cycle, in single precision: the preconditioner only has to
    /// be a fixed symmetric positive definite operator, and the conjugate gradients
    /// outside stay in double precision. The rounding costs about a tenth more iterations
    /// (the iteration no longer keeps its full history of directions), the halved memory
    /// traffic of the cycle outweighs that.
    pub fn apply(&self, r: &[f64], z: &mut [f64]) -> Result<()> {
        let mut work = self.work.lock().unwrap_or_else(|e| e.into_inner());
        if self.levels.len() == 1 {
            // The whole system is factorized: solve it in double precision.
            z.copy_from_slice(r);
            return self.coarse.solve(z);
        }
        convert(r, &mut work[0].b);
        self.cycle(0, &mut work)?;
        convert(&work[0].x, z);
        Ok(())
    }

    fn cycle(&self, l: usize, work: &mut [Work]) -> Result<()> {
        let level = &self.levels[l];
        let a = level.a.as_ref().unwrap_or(&self.fine);
        let (this, rest) = work.split_first_mut().expect("a work space per level");
        let (Some(p), Some(pt)) = (&level.p, &level.pt) else {
            convert(&this.b, &mut this.coarse);
            self.coarse.solve(&mut this.coarse)?;
            convert(&this.coarse, &mut this.x);
            return Ok(());
        };
        smooth(
            level.lambda_max,
            self.options.degree,
            a,
            &level.inv_diag,
            this,
            true,
        );
        a.residual(&this.b, &this.x, &mut this.r);
        let next = &mut rest[0];
        pt.multiply(&this.r, &mut next.b);
        self.cycle(l + 1, rest)?;
        p.multiply(&rest[0].x, &mut this.t);
        add(&mut this.x, &this.t);
        smooth(
            level.lambda_max,
            self.options.degree,
            a,
            &level.inv_diag,
            this,
            false,
        );
        Ok(())
    }
}

/// Chebyshev smoothing of `A x = b` on the upper part of the spectrum of `D⁻¹ A`,
/// `degree` products.
fn smooth(
    lambda_max: f64,
    degree: usize,
    a: &Bsr<f32>,
    inv_diag: &[f32],
    w: &mut Work,
    x_is_zero: bool,
) {
    let lambda_min = lambda_max / 30.0;
    let theta = 0.5 * (lambda_max + lambda_min);
    let delta = 0.5 * (lambda_max - lambda_min);
    let sigma = theta / delta;
    let mut rho_old = 1.0 / sigma;
    if x_is_zero {
        w.r.copy_from_slice(&w.b);
        scaled(&mut w.d, inv_diag, &w.r, (1.0 / theta) as f32);
        w.x.copy_from_slice(&w.d);
    } else {
        a.residual(&w.b, &w.x, &mut w.r);
        scaled(&mut w.d, inv_diag, &w.r, (1.0 / theta) as f32);
        add(&mut w.x, &w.d);
    }
    for _ in 1..degree {
        let rho_new = 1.0 / (2.0 * sigma - rho_old);
        a.multiply(&w.d, &mut w.t);
        sub(&mut w.r, &w.t);
        let (c1, c2) = ((rho_new * rho_old) as f32, (2.0 * rho_new / delta) as f32);
        // d = c1 d + c2 D⁻¹ r; x += d
        update(&mut w.d, &w.r, inv_diag, c1, c2);
        add(&mut w.x, &w.d);
        rho_old = rho_new;
    }
}

const PARALLEL_FROM: usize = 8192;

/// `y = x` in the precision of `y`.
fn convert<S: Scalar, T: Scalar>(x: &[S], y: &mut [T]) {
    if y.len() < PARALLEL_FROM {
        y.iter_mut()
            .zip(x)
            .for_each(|(y, x)| *y = T::from_f64(x.to_f64()));
    } else {
        y.par_chunks_mut(4096)
            .zip(x.par_chunks(4096))
            .for_each(|(y, x)| {
                y.iter_mut()
                    .zip(x)
                    .for_each(|(y, x)| *y = T::from_f64(x.to_f64()))
            });
    }
}

fn add<T: Scalar>(y: &mut [T], x: &[T]) {
    if y.len() < PARALLEL_FROM {
        y.iter_mut().zip(x).for_each(|(y, x)| *y += *x);
    } else {
        y.par_chunks_mut(4096)
            .zip(x.par_chunks(4096))
            .for_each(|(y, x)| y.iter_mut().zip(x).for_each(|(y, x)| *y += *x));
    }
}

fn sub<T: Scalar>(y: &mut [T], x: &[T]) {
    if y.len() < PARALLEL_FROM {
        y.iter_mut().zip(x).for_each(|(y, x)| *y -= *x);
    } else {
        y.par_chunks_mut(4096)
            .zip(x.par_chunks(4096))
            .for_each(|(y, x)| y.iter_mut().zip(x).for_each(|(y, x)| *y -= *x));
    }
}

/// `d = c · diag ∘ r`.
fn scaled<T: Scalar>(d: &mut [T], diag: &[T], r: &[T], c: T) {
    if d.len() < PARALLEL_FROM {
        d.iter_mut()
            .zip(diag)
            .zip(r)
            .for_each(|((d, di), r)| *d = c * *di * *r);
    } else {
        d.par_chunks_mut(4096)
            .zip(diag.par_chunks(4096))
            .zip(r.par_chunks(4096))
            .for_each(|((d, diag), r)| {
                d.iter_mut()
                    .zip(diag)
                    .zip(r)
                    .for_each(|((d, di), r)| *d = c * *di * *r)
            });
    }
}

/// `d = c1 d + c2 diag ∘ r`.
fn update<T: Scalar>(d: &mut [T], r: &[T], diag: &[T], c1: T, c2: T) {
    if d.len() < PARALLEL_FROM {
        d.iter_mut()
            .zip(diag)
            .zip(r)
            .for_each(|((d, di), r)| *d = c1 * *d + c2 * *di * *r);
    } else {
        d.par_chunks_mut(4096)
            .zip(diag.par_chunks(4096))
            .zip(r.par_chunks(4096))
            .for_each(|((d, diag), r)| {
                d.iter_mut()
                    .zip(diag)
                    .zip(r)
                    .for_each(|((d, di), r)| *d = c1 * *d + c2 * *di * *r)
            });
    }
}

/// `out += a · b` for row-major `a` (`m × p`) and `b` (`p × n`).
#[inline]
fn gemm_add(a: &[f64], m: usize, p: usize, b: &[f64], n: usize, out: &mut [f64]) {
    debug_assert_eq!(a.len(), m * p);
    debug_assert_eq!(b.len(), p * n);
    debug_assert_eq!(out.len(), m * n);
    for i in 0..m {
        let out_row = &mut out[i * n..(i + 1) * n];
        for l in 0..p {
            let a_il = a[i * p + l];
            if a_il == 0.0 {
                continue;
            }
            let b_row = &b[l * n..(l + 1) * n];
            for (o, &b_lj) in out_row.iter_mut().zip(b_row) {
                *o += a_il * b_lj;
            }
        }
    }
}

/// The rigid body modes (or the constant, for a scalar field) of the unknowns.
fn rigid_body_modes(u: &Unknowns) -> Nullspace {
    let n = u.component.len();
    let translational = u.component.iter().any(|&c| c > 0 && c < u8::MAX);
    if !translational {
        return Nullspace {
            k: 1,
            vectors: u
                .component
                .iter()
                .map(|&c| if c == u8::MAX { 0.0 } else { 1.0 })
                .collect(),
        };
    }
    // Coordinates relative to the centre, scaled to a unit extent, so that the rotations
    // are of the order of the translations.
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for c in &u.coordinates {
        for d in 0..3 {
            lo[d] = lo[d].min(c[d]);
            hi[d] = hi[d].max(c[d]);
        }
    }
    let centre = [0, 1, 2].map(|d| 0.5 * (lo[d] + hi[d]));
    let extent = (0..3).map(|d| hi[d] - lo[d]).fold(0.0_f64, f64::max);
    let scale = if extent > 0.0 { 1.0 / extent } else { 1.0 };
    let k = 6;
    let mut vectors = vec![0.0; n * k];
    for i in 0..n {
        let c = u.component[i] as usize;
        let x = [0, 1, 2].map(|d| (u.coordinates[i][d] - centre[d]) * scale);
        let row = &mut vectors[i * k..(i + 1) * k];
        if c < 3 {
            row[c] = 1.0;
            // Rotation about axis d moves component c by (e_d × x)_c.
            let rot = [[0.0, -x[2], x[1]], [x[2], 0.0, -x[0]], [-x[1], x[0], 0.0]];
            for d in 0..3 {
                row[3 + d] = rot[d][c];
            }
        }
    }
    Nullspace { k, vectors }
}

/// Aggregates of blocks (nodes): for every block its aggregate, and the blocks of every
/// aggregate (CSR over the aggregates).
struct Aggregates {
    count: usize,
    of_block: Vec<u32>,
    ptr: Vec<usize>,
    blocks: Vec<u32>,
}

/// Groups the blocks of a level by the three aggregation phases on the graph of strong
/// connections (Frobenius norms of the coupling blocks).
fn aggregate(a: &Bsr, strength: f64, max_aggregate: usize) -> Aggregates {
    let blocks = a.brows;
    let size = a.br * a.bc;
    // Squared norms of the diagonal blocks.
    let diag: Vec<f64> = (0..blocks)
        .map(|b| {
            a.diagonal_block(b).map_or(0.0, |k| {
                a.val[size * k..size * (k + 1)].iter().map(|v| v * v).sum()
            })
        })
        .collect();
    // Strong: ‖A_bc‖ ≥ θ √(‖A_bb‖ ‖A_cc‖), with the squared norms as weights.
    let theta4 = strength.powi(4);
    let strong = |b: usize| {
        let (col, val) = a.block_row(b);
        let diag = &diag;
        col.iter().enumerate().filter_map(move |(k, &c)| {
            let c = c as usize;
            let w: f64 = val[size * k..size * (k + 1)].iter().map(|v| v * v).sum();
            (c != b && w * w >= theta4 * diag[b] * diag[c]).then_some((c, w))
        })
    };
    const NONE: u32 = u32::MAX;
    let mut of_block = vec![NONE; blocks];
    let mut count = 0u32;
    // Phase 1: a block whose strong neighbourhood is untouched founds an aggregate of it.
    for b in 0..blocks {
        if of_block[b] != NONE {
            continue;
        }
        if strong(b).all(|(c, _)| of_block[c] == NONE) {
            of_block[b] = count;
            let mut members: Vec<(usize, f64)> = strong(b).collect();
            if members.len() + 1 > max_aggregate {
                members.sort_by(|x, y| y.1.total_cmp(&x.1));
                members.truncate(max_aggregate.saturating_sub(1));
            }
            for (c, _) in members {
                of_block[c] = count;
            }
            count += 1;
        }
    }
    // Phase 2: the rest join the aggregate of their strongest aggregated neighbour (as
    // formed in phase 1, so that aggregates do not grow in chains).
    let phase1 = of_block.clone();
    for (b, aggregate) in of_block.iter_mut().enumerate() {
        if *aggregate != NONE {
            continue;
        }
        let best = strong(b)
            .filter(|&(c, _)| phase1[c] != NONE)
            .max_by(|x, y| x.1.total_cmp(&y.1));
        if let Some((c, _)) = best {
            *aggregate = phase1[c];
        }
    }
    // Phase 3: what is left forms aggregates with its unaggregated strong neighbours.
    for b in 0..blocks {
        if of_block[b] != NONE {
            continue;
        }
        of_block[b] = count;
        let members: Vec<usize> = strong(b)
            .filter(|&(c, _)| of_block[c] == NONE)
            .map(|(c, _)| c)
            .collect();
        for c in members {
            of_block[c] = count;
        }
        count += 1;
    }
    let count = count as usize;
    let mut ptr = vec![0usize; count + 1];
    for &g in &of_block {
        ptr[g as usize + 1] += 1;
    }
    for g in 0..count {
        ptr[g + 1] += ptr[g];
    }
    let mut next = ptr.clone();
    let mut members = vec![0u32; blocks];
    for (b, &g) in of_block.iter().enumerate() {
        members[next[g as usize]] = b as u32;
        next[g as usize] += 1;
    }
    Aggregates {
        count,
        of_block,
        ptr,
        blocks: members,
    }
}

/// The tentative prolongator (blocks of `br × k`, one per block row: the orthonormalized
/// nullspace of the block's aggregate) and the nullspace of the coarse level.
fn tentative(a: &Bsr, agg: &Aggregates, ns: &Nullspace) -> (Bsr, Nullspace) {
    let (br, k) = (a.br, ns.k);
    let blocks = a.brows;
    let mut val = vec![0.0; blocks * br * k];
    let mut coarse = vec![0.0; agg.count * k * k];
    let mut q: Vec<f64> = Vec::new();
    for g in 0..agg.count {
        let members = &agg.blocks[agg.ptr[g]..agg.ptr[g + 1]];
        let m = members.len() * br;
        q.clear();
        for &b in members {
            let b = b as usize;
            q.extend_from_slice(&ns.vectors[b * br * k..(b + 1) * br * k]);
        }
        let r = &mut coarse[g * k * k..(g + 1) * k * k];
        gram_schmidt(&mut q, m, k, r);
        for (position, &b) in members.iter().enumerate() {
            let b = b as usize;
            val[b * br * k..(b + 1) * br * k]
                .copy_from_slice(&q[position * br * k..(position + 1) * br * k]);
        }
    }
    let p_t = Bsr {
        brows: blocks,
        bcols: agg.count,
        br,
        bc: k,
        ptr: (0..=blocks).collect(),
        col: agg.of_block.clone(),
        val,
    };
    (p_t, Nullspace { k, vectors: coarse })
}

/// Orthonormalizes the columns of the row-major `m × k` matrix `q` in place (modified
/// Gram-Schmidt, twice) and writes the `k × k` triangular factor to `r`, so that the
/// input is `Q R`. A column that is (numerically) dependent on the earlier ones becomes
/// zero, with a zero row in `r`.
fn gram_schmidt(q: &mut [f64], m: usize, k: usize, r: &mut [f64]) {
    r.iter_mut().for_each(|v| *v = 0.0);
    let column_norm =
        |q: &[f64], j: usize| (0..m).map(|i| q[i * k + j].powi(2)).sum::<f64>().sqrt();
    for j in 0..k {
        let original = column_norm(q, j);
        for _pass in 0..2 {
            for l in 0..j {
                let dot: f64 = (0..m).map(|i| q[i * k + l] * q[i * k + j]).sum();
                r[l * k + j] += dot;
                for i in 0..m {
                    q[i * k + j] -= dot * q[i * k + l];
                }
            }
        }
        let norm = column_norm(q, j);
        if norm <= 1e-8 * original || norm == 0.0 {
            for i in 0..m {
                q[i * k + j] = 0.0;
            }
            // The column is dropped: its row of R is zero, and the projections onto
            // it are meaningless.
            for l in 0..k {
                r[j * k + l] = 0.0;
            }
            for l in 0..j {
                r[l * k + j] = 0.0;
            }
            continue;
        }
        r[j * k + j] = norm;
        for i in 0..m {
            q[i * k + j] /= norm;
        }
    }
}

/// `λ_max(D⁻¹ A)` by power iteration, with a margin.
fn largest_eigenvalue(a: &Bsr, inv_diag: &[f64]) -> f64 {
    let n = a.rows();
    if n == 0 {
        return 1.0;
    }
    let mut x: Vec<f64> = (0..n)
        .map(|i| 1.0 + 0.5 * ((i * 7919 % 1000) as f64 / 1000.0))
        .collect();
    let mut y = vec![0.0; n];
    let mut lambda = 1.0;
    for _ in 0..12 {
        a.multiply(&x, &mut y);
        y.iter_mut().zip(inv_diag).for_each(|(y, d)| *y *= d);
        let norm = y.iter().map(|v| v * v).sum::<f64>().sqrt();
        if norm == 0.0 || !norm.is_finite() {
            break;
        }
        lambda = norm;
        x.iter_mut().zip(&y).for_each(|(x, y)| *x = y / norm);
    }
    lambda * 1.1
}

/// `P = (I − ω D⁻¹ A) P_t` with `ω = 4 / (3 λ_max)`.
fn smooth_prolongator(a: &Bsr, inv_diag: &[f64], lambda_max: f64, p_t: &Bsr) -> Bsr {
    let omega = 4.0 / (3.0 * lambda_max);
    let (br, k) = (a.br, p_t.bc);
    let size = br * br;
    Bsr::build(
        a.brows,
        p_t.bcols,
        br,
        k,
        || vec![0.0; br * k],
        |i, product, acc| {
            let (col, val) = p_t.block_row(i);
            for (&c, block) in col.iter().zip(val.chunks_exact(br * k)) {
                acc.block(c)
                    .iter_mut()
                    .zip(block)
                    .for_each(|(o, v)| *o += v);
            }
            let (acol, aval) = a.block_row(i);
            for (&j, a_ij) in acol.iter().zip(aval.chunks_exact(size)) {
                let (pcol, pval) = p_t.block_row(j as usize);
                for (&c, p_j) in pcol.iter().zip(pval.chunks_exact(br * k)) {
                    product.iter_mut().for_each(|v| *v = 0.0);
                    gemm_add(a_ij, br, br, p_j, k, product);
                    let out = acc.block(c);
                    for r in 0..br {
                        let w = -omega * inv_diag[i * br + r];
                        for j in 0..k {
                            out[r * k + j] += w * product[r * k + j];
                        }
                    }
                }
            }
        },
    )
}

/// `Pᵀ A P`, with a unit diagonal where a coarse unknown has no coupling at all (the
/// dropped columns of the tentative prolongator).
fn galerkin(a: &Bsr, p: &Bsr, pt: &Bsr) -> Bsr {
    let (br, k) = (a.br, p.bc);
    let mut a_c = Bsr::build(
        p.bcols,
        p.bcols,
        k,
        k,
        || (BlockAccumulator::new(a.brows, k * br), vec![0.0; k * k]),
        |c, (w, product), acc| {
            // W = Pᵀ[c, :] A, blocks of k × br over the fine block columns.
            w.clear();
            let (pcol, pval) = pt.block_row(c);
            for (&i, pt_ci) in pcol.iter().zip(pval.chunks_exact(k * br)) {
                let (acol, aval) = a.block_row(i as usize);
                for (&j, a_ij) in acol.iter().zip(aval.chunks_exact(br * br)) {
                    gemm_add(pt_ci, k, br, a_ij, br, w.block(j));
                }
            }
            // Keep the diagonal block in the pattern.
            let _ = acc.block(c as u32);
            for (j, w_j) in w.entries() {
                let (pcol, pval) = p.block_row(j as usize);
                for (&h, p_jh) in pcol.iter().zip(pval.chunks_exact(br * k)) {
                    product.iter_mut().for_each(|v| *v = 0.0);
                    gemm_add(w_j, k, br, p_jh, k, product);
                    acc.block(h)
                        .iter_mut()
                        .zip(product.iter())
                        .for_each(|(o, v)| *o += v);
                }
            }
        },
    );
    for c in 0..a_c.brows {
        let d = a_c.diagonal_block(c).expect("diagonal kept");
        for r in 0..k {
            let at = k * k * d + r * k + r;
            if a_c.val[at] == 0.0 {
                a_c.val[at] = 1.0;
            }
        }
    }
    a_c
}

/// The upper triangle of a symmetric full matrix as the pattern and values of the direct
/// solver.
fn upper_triangle(a: &Bsr) -> (Pattern, SparseMatrix) {
    let n = a.rows();
    let rows: Vec<Vec<u32>> = (0..n)
        .map(|c| {
            let mut list = Vec::new();
            a.for_row(c, |r, _| {
                if (r as usize) < c {
                    list.push(r);
                }
            });
            list
        })
        .collect();
    let pattern = Pattern::from_columns(n, rows);
    let shared = Arc::new(pattern.clone());
    let mut matrix = SparseMatrix::zeros(shared);
    for c in 0..n {
        a.for_row(c, |r, v| {
            if (r as usize) <= c {
                // Row c of the full matrix: entry (c, r) = (r, c) of the upper triangle.
                matrix.add(r, c as u32, v);
            }
        });
    }
    (pattern, matrix)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gram_schmidt_factorizes() {
        let m = 4;
        let k = 2;
        let a = vec![1.0, 2.0, 0.0, 1.0, 1.0, 0.0, 2.0, 1.0];
        let mut q = a.clone();
        let mut r = vec![0.0; k * k];
        gram_schmidt(&mut q, m, k, &mut r);
        for i in 0..m {
            for j in 0..k {
                let v: f64 = (0..k).map(|l| q[i * k + l] * r[l * k + j]).sum();
                assert!((v - a[i * k + j]).abs() < 1e-12);
            }
        }
        let dot: f64 = (0..m).map(|i| q[i * k] * q[i * k + 1]).sum();
        assert!(dot.abs() < 1e-12);
    }

    #[test]
    fn dependent_columns_are_dropped() {
        let m = 3;
        let k = 2;
        let mut q = vec![1.0, 2.0, 1.0, 2.0, 1.0, 2.0];
        let mut r = vec![0.0; 4];
        gram_schmidt(&mut q, m, k, &mut r);
        assert!(q[1] == 0.0 && q[3] == 0.0 && q[5] == 0.0);
        assert_eq!(r[3], 0.0);
    }

    #[test]
    fn gemm_adds_the_product() {
        // [[1, 2], [3, 4]] · [[1, 0, 2], [0, 1, 1]]
        let a = [1.0, 2.0, 3.0, 4.0];
        let b = [1.0, 0.0, 2.0, 0.0, 1.0, 1.0];
        let mut out = [1.0; 6];
        gemm_add(&a, 2, 2, &b, 3, &mut out);
        assert_eq!(out, [2.0, 3.0, 5.0, 4.0, 5.0, 11.0]);
    }
}
