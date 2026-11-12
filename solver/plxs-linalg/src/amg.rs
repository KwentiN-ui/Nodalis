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
//! A mesh of quadratic elements is coarsened to its corner nodes first, by the
//! interpolation of the linear elements on the same corners (a midside node takes the mean
//! of the ends of its edge), and the aggregation starts on that level, see
//! [`Amg::build`].
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
/// `u8::MAX` for a padding unknown without a mode), the position of its node and,
/// optionally, the body its node belongs to.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Unknowns {
    pub node: Vec<u32>,
    pub component: Vec<u8>,
    pub coordinates: Vec<[f64; 3]>,
    /// The body of every unknown's node (any dense numbering), or empty when the bodies
    /// are not known: then the whole model counts as one. Nodes of different bodies never
    /// share an aggregate of the multigrid (see [`Amg`]).
    pub body: Vec<u32>,
    /// For the unknowns of a midside node the two end nodes of its element edge (numbered
    /// as in `node`), `[u32::MAX; 2]` for those of any other node; empty when the mesh is
    /// not known. The multigrid of a mesh of quadratic elements coarsens to the corner
    /// nodes first (see [`Amg`]).
    pub midside: Vec<[u32; 2]>,
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
            body: Vec::new(),
            midside: Vec::new(),
        }
    }

    /// The same unknowns, with the ends of the edge of every midside node (`of_node`, by
    /// the node numbers of [`Self::node`]; `[u32::MAX; 2]` for the other nodes); see
    /// [`Self::midside`].
    pub fn with_midsides(mut self, of_node: &[[u32; 2]]) -> Self {
        self.midside = self
            .node
            .iter()
            .map(|&n| of_node.get(n as usize).copied().unwrap_or([u32::MAX; 2]))
            .collect();
        self
    }

    /// The same unknowns, with the body of every node (`of_node`, by the node numbers of
    /// [`Self::node`]); see [`Self::body`].
    pub fn with_bodies(mut self, of_node: &[u32]) -> Self {
        self.body = self
            .node
            .iter()
            .map(|&n| of_node.get(n as usize).copied().unwrap_or(0))
            .collect();
        self
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
    /// Whether the cycle may run on the graphics card (`PLXS_GPU`, feature `gpu`).
    pub gpu: GpuMode,
    /// How many near-nullspace vectors the hierarchy may learn from its own slow modes
    /// (adaptive smoothed aggregation), see [`Amg::build`]; 0 turns it off.
    pub adaptive: usize,
    /// Whether a mesh of quadratic elements is coarsened to its corner nodes first, see
    /// [`Amg::build`].
    pub corners: bool,
    /// Whether a region where the cycle is slow gets a direct solve of its own, see
    /// [`crate::patch`] (`PLXS_AMG_PATCH`).
    pub patch: bool,
    /// The corner level with rotations, solved directly (`PLXS_AMG_ROTATIONS`), see
    /// [`Amg::build`].
    pub rotations: Rotations,
}

/// Whether the corner level of a quadratic mesh carries rotations and is solved directly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rotations {
    /// Never (`PLXS_AMG_ROTATIONS=0`, the default: on solid parts the factor of the
    /// level costs far more than the iterations it saves, on flat parts it saves a tenth
    /// of the time at most).
    Off,
    /// When its factor is cheap, that is on flat parts (`PLXS_AMG_ROTATIONS=1`).
    Automatic,
    /// Always, whatever the factor costs (`PLXS_AMG_ROTATIONS=2`, measurements).
    On,
}

/// Where the multigrid cycle runs (`PLXS_GPU=off|on`, automatic otherwise).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GpuMode {
    /// Always on the processor.
    Off,
    /// On a discrete graphics card for large systems, see [`crate::gpu`].
    Automatic,
    /// On the graphics card whenever there is one, of any kind (software renderers
    /// included) and for systems of any size: tests and measurements.
    On,
}

/// The mode of [`GpuMode::set_preferred`]: 0 automatic, 1 off, 2 on.
static PREFERRED: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(0);

impl GpuMode {
    /// `PLXS_GPU` when set, else the mode the application chose with
    /// [`Self::set_preferred`] (automatic by default).
    pub fn from_environment() -> Self {
        match std::env::var("PLXS_GPU")
            .map(|v| v.to_ascii_lowercase())
            .as_deref()
        {
            Ok("off") | Ok("0") | Ok("no") | Ok("false") => Self::Off,
            Ok("on") | Ok("1") | Ok("yes") | Ok("true") => Self::On,
            _ => Self::preferred(),
        }
    }

    /// The mode for the solves started from now on in this process, unless `PLXS_GPU`
    /// says otherwise: the setting "Use graphics card" of the application.
    pub fn set_preferred(mode: GpuMode) {
        let value = match mode {
            Self::Automatic => 0,
            Self::Off => 1,
            Self::On => 2,
        };
        PREFERRED.store(value, std::sync::atomic::Ordering::Relaxed);
    }

    fn preferred() -> Self {
        match PREFERRED.load(std::sync::atomic::Ordering::Relaxed) {
            1 => Self::Off,
            2 => Self::On,
            _ => Self::Automatic,
        }
    }
}

impl Default for AmgOptions {
    /// The defaults, each overridable for measurements by an environment variable
    /// (`PLXS_AMG_STRENGTH`, `PLXS_AMG_DEGREE`, `PLXS_AMG_COARSE`, `PLXS_AMG_LEVELS`,
    /// `PLXS_AMG_MAX_AGG`, `PLXS_AMG_CORNERS`).
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
            gpu: GpuMode::from_environment(),
            adaptive: env("PLXS_AMG_ADAPTIVE", 0),
            corners: env("PLXS_AMG_CORNERS", 1) != 0,
            patch: env("PLXS_AMG_PATCH", 1) != 0,
            rotations: match env("PLXS_AMG_ROTATIONS", 0) {
                0 => Rotations::Off,
                2 => Rotations::On,
                _ => Rotations::Automatic,
            },
        }
    }
}

/// The near-nullspace of a level: `k` vectors of length `n` (row major, `n × k`); the
/// unknowns are grouped by the blocks of the level's matrix.
struct Nullspace {
    k: usize,
    vectors: Vec<f64>,
}

impl Nullspace {
    /// Appends a vector (length `n`) as the last column.
    fn push(&mut self, vector: &[f64]) {
        let n = vector.len();
        let k = self.k + 1;
        let mut vectors = vec![0.0; n * k];
        for i in 0..n {
            vectors[i * k..i * k + self.k]
                .copy_from_slice(&self.vectors[i * self.k..(i + 1) * self.k]);
            vectors[i * k + self.k] = vector[i];
        }
        self.k = k;
        self.vectors = vectors;
    }
}

struct Level {
    /// The operator of this level; `None` on the finest level, whose copy is [`Amg::fine`].
    a: Option<Bsr<f32>>,
    /// Unknowns of the level.
    n: usize,
    /// The inverses of the diagonal blocks (`br × br` each, row-major), the block Jacobi
    /// scaling of the smoother: a scalar diagonal cannot scale a node whose stiffness is
    /// large in one direction only (a gap or contact spring to the ground), and the
    /// smoother then leaves the error in the other directions untouched.
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
    /// The coarsest operator in double precision (what `coarse` factorized), for the
    /// dense inverse on the graphics card.
    #[cfg_attr(not(feature = "gpu"), allow(dead_code))]
    coarsest: Bsr,
    work: Mutex<Vec<Work>>,
    options: AmgOptions,
    /// Single precision values of all operators and prolongators of the hierarchy, the
    /// copy of the finest operator included.
    nonzeros: usize,
    /// The cycle on the graphics card, when it runs there.
    #[cfg(feature = "gpu")]
    gpu: Option<Box<crate::gpu::GpuCycle>>,
    /// Why the cycle runs on the processor although the card was considered (or failed).
    cpu_reason: Mutex<Option<String>>,
    /// The direct solve of the region where the cycle is slow, see [`crate::patch`]
    /// (only on the processor); found by the first solve, see [`Self::set_patch`].
    patch: std::sync::RwLock<Option<crate::patch::Patch>>,
    /// Whether a solve looked for the slow region already.
    patch_tried: std::sync::atomic::AtomicBool,
    /// Why the conjugate gradients stay on the processor although the cycle runs on the
    /// card.
    #[cfg(feature = "gpu")]
    cg_note: Mutex<Option<String>>,
}

/// Cycles of the test for slow modes of a fresh hierarchy (see [`Amg::build`]).
const ADAPTIVE_CYCLES: usize = 30;
/// A hierarchy whose stationary iteration still reduces the energy of a random error by
/// less than this factor per cycle after [`ADAPTIVE_CYCLES`] cycles learns the remaining
/// error as a near-nullspace vector. A solid block of hexahedra contracts by about 0.7
/// per cycle, one of quadratic tetrahedra by about 0.83.
const ADAPTIVE_THRESHOLD: f64 = 0.85;

impl Amg {
    /// Builds the hierarchy of the symmetric positive definite `a` (full storage, square
    /// blocks). `unknowns`, when given, has one entry per row of `a` and the unknowns of a
    /// block belong to one node. `candidates` are further near-nullspace vectors (length
    /// of `a`), the slow modes an earlier solve with a hierarchy of `a` found (see
    /// [`crate::PcgSolver`]).
    ///
    /// Adaptive smoothed aggregation (Brezina, Falgout, MacLachlan, Manteuffel,
    /// McCormick, Ruge, "Adaptive smoothed aggregation (αSA)", SIAM J. Sci. Comput. 25
    /// (2004)): the rigid body modes are the near-nullspace of a solid, but a thin flange
    /// or a layer of one element bends with little energy in a pattern the rigid modes of
    /// an aggregate do not span, and such a hierarchy leaves those modes to the smoother,
    /// which cannot reduce them (a rope pulley: 84 iterations instead of 40). The fresh
    /// hierarchy is therefore tried on a random error with the right-hand side zero;
    /// what [`ADAPTIVE_CYCLES`] cycles leave is the slow mode, and when the energy still
    /// contracts by less than [`ADAPTIVE_THRESHOLD`] per cycle that vector joins the
    /// near-nullspace and the hierarchy is built again, up to `options.adaptive` times.
    ///
    /// Quadratic elements (`unknowns.midside`): the first coarse level is the corner nodes
    /// with the interpolation of the linear elements, unsmoothed, and the aggregation
    /// starts there. Aggregates of the quadratic mesh itself fit it badly, those of the
    /// 20-node brick worst (the smoother with the block diagonal is weak on it, the
    /// largest eigenvalue of `D⁻¹ A` is 5.4 against 2.7 for the 10-node tetrahedron): a
    /// plate with a boss swept into 97 000 bricks, 1.27 million unknowns, took 96
    /// iterations and takes 41, the same part in tetrahedra 46 and 28, a cube of bricks
    /// 44 and 22. The corner level has a quarter (bricks) or an eighth (tetrahedra) of the
    /// unknowns and the sparse operator of the linear mesh, so the hierarchy is no larger
    /// and is built faster. A midside node that something holds from outside its body's
    /// elements stays on the corner level as it is (see `corner_prolongator`).
    pub fn build(
        a: &Bsr,
        unknowns: Option<&Unknowns>,
        candidates: &[Vec<f64>],
        options: AmgOptions,
    ) -> Result<Amg> {
        let n = a.rows();
        let mut nullspace = match unknowns {
            Some(u) if u.component.len() == n => rigid_body_modes(u),
            _ => Nullspace {
                k: 1,
                vectors: vec![1.0; n],
            },
        };
        for candidate in candidates.iter().filter(|c| c.len() == n) {
            nullspace.push(candidate);
        }
        // The body of every block (node) of the level, when the bodies are known: the
        // aggregates never cross from one body into another.
        let bodies: Option<Vec<u32>> = unknowns
            .filter(|u| u.body.len() == n && n > 0)
            .map(|u| (0..a.brows).map(|b| u.body[b * a.br]).collect());
        let padding: Vec<bool> = match unknowns {
            Some(u) if u.component.len() == n => {
                u.component.iter().map(|&c| c == u8::MAX).collect()
            }
            _ => Vec::new(),
        };
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
        if options.corners
            && options.rotations != Rotations::Off
            && let Some(u) = unknowns.filter(|u| u.midside.len() == n)
            && let Some(amg) = Self::linked(a, u, options, trace, &mut lap)?
        {
            return Ok(amg);
        }
        let corners = unknowns
            .filter(|u| options.corners && u.midside.len() == n)
            .and_then(|u| corner_prolongator(a, u, false));
        let corners = corners.as_ref();
        let mut amg = Self::assemble(
            a,
            &nullspace,
            bodies.clone(),
            corners,
            options,
            trace,
            &mut lap,
        )?;
        for round in 0..options.adaptive {
            let Some((vector, factor)) = amg.slow_mode(a, &padding) else {
                break;
            };
            lap("test for slow modes", n);
            if trace {
                eprintln!(
                    "amg: round {}: the error still contracts by {factor:.3} per cycle, the slow \
                     mode becomes near-nullspace vector {}",
                    round + 1,
                    nullspace.k + 1
                );
            }
            nullspace.push(&vector);
            amg = Self::assemble(
                a,
                &nullspace,
                bodies.clone(),
                corners,
                options,
                trace,
                &mut lap,
            )?;
        }
        #[cfg(feature = "gpu")]
        {
            let views: Vec<crate::gpu::LevelView<'_>> = amg
                .levels
                .iter()
                .map(|level| crate::gpu::LevelView {
                    a: level.a.as_ref().unwrap_or(&amg.fine),
                    inv_diag: &level.inv_diag,
                    lambda_max: level.lambda_max,
                    p: level.p.as_ref(),
                    pt: level.pt.as_ref(),
                })
                .collect();
            let placed =
                crate::gpu::GpuCycle::place(&views, &amg.coarsest, options.degree, options.gpu);
            lap("upload to the graphics card", n);
            match placed {
                crate::gpu::Placement::Gpu(cycle) => amg.gpu = Some(cycle),
                crate::gpu::Placement::Cpu(reason) => amg.cpu_reason = Mutex::new(reason),
            }
        }
        Ok(amg)
    }

    /// The slow mode of this hierarchy, when it has one: the error left of a random one
    /// after [`ADAPTIVE_CYCLES`] cycles on `A x = 0`, scaled to unit entries, with the
    /// energy contraction of the last cycle; `None` when the hierarchy contracts the
    /// energy by at least [`ADAPTIVE_THRESHOLD`] per cycle.
    fn slow_mode(&self, a: &Bsr, padding: &[bool]) -> Option<(Vec<f64>, f64)> {
        let (mut x, factor) = self.test_error(a, padding, None, ADAPTIVE_CYCLES)?;
        if factor < ADAPTIVE_THRESHOLD {
            return None;
        }
        let n = x.len();
        let scale = x.iter().map(|v| v * v).sum::<f64>().sqrt() / (n as f64).sqrt();
        if scale == 0.0 || !scale.is_finite() {
            return None;
        }
        x.iter_mut().for_each(|v| *v /= scale);
        Some((x, factor))
    }

    /// The error left of a random one after `cycles` cycles of the stationary iteration
    /// on `A x = 0`, with the energy contraction of the last cycle; `None` for a single
    /// level or when the cycle fails.
    pub(crate) fn test_error(
        &self,
        a: &Bsr,
        padding: &[bool],
        start: Option<Vec<f64>>,
        cycles: usize,
    ) -> Option<(Vec<f64>, f64)> {
        let n = a.rows();
        if n == 0 || self.levels.len() == 1 {
            return None;
        }
        // A deterministic pseudo-random start unless given, zero on the padding.
        let mut x: Vec<f64> = start.filter(|s| s.len() == n).unwrap_or_else(|| {
            (0..n)
                .map(|i| {
                    let h = (i as u64)
                        .wrapping_mul(0x9E37_79B9_7F4A_7C15)
                        .rotate_left(23)
                        ^ 0xD1B5_4A32_D192_ED03;
                    (h >> 11) as f64 / (1u64 << 53) as f64 - 0.5
                })
                .collect()
        });
        if padding.len() == n {
            x.iter_mut().zip(padding).for_each(|(x, &p)| {
                if p {
                    *x = 0.0
                }
            });
        }
        let mut ax = vec![0.0; n];
        let mut z = vec![0.0; n];
        let mut energy_before = 0.0;
        let mut factor = 1.0;
        for cycle in 0..cycles {
            a.multiply(&x, &mut ax);
            let energy: f64 = x.iter().zip(&ax).map(|(x, y)| x * y).sum();
            if !energy.is_finite() || energy <= 0.0 {
                return None;
            }
            if cycle > 0 {
                factor = (energy / energy_before).sqrt();
                if std::env::var("PLXS_AMG_TRACE").is_ok() {
                    eprintln!("amg: test cycle {cycle}: energy contraction {factor:.4}");
                }
            }
            energy_before = energy;
            // x ← x − M⁻¹ A x
            if self.apply(&ax, &mut z).is_err() {
                return None;
            }
            x.iter_mut().zip(&z).for_each(|(x, z)| *x -= z);
        }
        Some((x, factor))
    }

    /// The two-level hierarchy of a quadratic mesh with the corner level with rotations
    /// (see [`linked_prolongator`]) solved directly, when the factor of that level is
    /// cheap enough (or `options.rotations` asks for it whatever it costs); `None`
    /// otherwise.
    fn linked(
        a: &Bsr,
        u: &Unknowns,
        options: AmgOptions,
        trace: bool,
        lap: &mut impl FnMut(&str, usize),
    ) -> Result<Option<Amg>> {
        let n = a.rows();
        let Some(corners) = corner_prolongator(a, u, true) else {
            return Ok(None);
        };
        let p = corners.p;
        let pt = p.transpose();
        lap("linked prolongator", n);
        let mut a_c = galerkin(a, &p, &pt);
        lap("galerkin product", n);
        // A rotation of the whole body moves no midside node: the coarse operator is
        // singular in it (three modes per body). A shift by a small part of the diagonal
        // keeps the factorization definite; the residual restricted from the fine level
        // has no part in these modes (`P v = 0`), and their correction is not
        // prolongated.
        for c in 0..a_c.brows {
            let k = a_c.br;
            let d = a_c.diagonal_block(c).expect("diagonal kept");
            for r in 0..k {
                a_c.val[k * k * d + r * k + r] *= 1.0 + LINKED_SHIFT;
            }
        }
        let (pattern, matrix) = upper_triangle(&a_c);
        lap("linked level upper triangle", a_c.rows());
        let mut coarse = LdltSolver::new(rayon::current_num_threads());
        let positions: Vec<[f64; 3]> = (corners.blocks.iter())
            .flat_map(|&b| std::iter::repeat_n(u.coordinates[b as usize * a.br], a_c.br))
            .collect();
        coarse.set_coordinates(Some(positions));
        coarse.analyze(&pattern)?;
        let flops = coarse.factor_flops().unwrap_or(0.0);
        let factor = coarse.factor_nonzeros().unwrap_or(0);
        lap("linked level analysis", a_c.rows());
        let cheap = flops <= LINKED_FLOPS_PER_NONZERO * a.nnz() as f64
            && factor as f64 <= LINKED_FACTOR_PER_NONZERO * a.nnz() as f64;
        if trace {
            eprintln!(
                "amg: linked corner level {} unknowns, {} blocks; factor {factor} nonzeros, \
                 {flops:.2e} flops against {} nonzeros of the matrix: {}",
                a_c.rows(),
                a_c.blocks(),
                a.nnz(),
                if cheap { "cheap" } else { "too dear" }
            );
        }
        if !cheap && options.rotations != Rotations::On {
            return Ok(None);
        }
        let factorized = coarse.factorize(&matrix);
        lap("linked level factorization", a_c.rows());
        if factorized.is_err() || coarse.inertia().is_some_and(|negative| negative > 0) {
            // Not definite: the matrix itself is not, or the shift fell short; the
            // ordinary hierarchy decides.
            return Ok(None);
        }
        let (inv_diag, inv_chol) = block_diagonal_inverses(a);
        let lambda = largest_eigenvalue(a, &inv_chol);
        lap("eigenvalue", n);
        let n_c = a_c.rows();
        let nonzeros = a.nnz() + p.nnz() * 2 + a_c.nnz();
        let levels = vec![
            Level {
                a: None,
                n,
                inv_diag: inv_diag.iter().map(|&d| d as f32).collect(),
                lambda_max: lambda,
                p: Some(p.converted()),
                pt: Some(pt.converted()),
            },
            Level {
                a: Some(a_c.converted()),
                n: n_c,
                inv_diag: Vec::new(),
                lambda_max: 0.0,
                p: None,
                pt: None,
            },
        ];
        let work = vec![Work::new(n, false), Work::new(n_c, true)];
        let fine = a.converted();
        lap("single precision copy", n);
        Ok(Some(Amg {
            fine,
            levels,
            coarse,
            coarsest: a_c,
            work: Mutex::new(work),
            options,
            nonzeros,
            #[cfg(feature = "gpu")]
            gpu: None,
            cpu_reason: Mutex::new(None),
            patch: Default::default(),
            patch_tried: Default::default(),
            #[cfg(feature = "gpu")]
            cg_note: Mutex::new(None),
        }))
    }

    /// The hierarchy of `a` on the near-nullspace `nullspace`, on the processor.
    fn assemble(
        a: &Bsr,
        nullspace: &Nullspace,
        mut bodies: Option<Vec<u32>>,
        corners: Option<&Corners>,
        options: AmgOptions,
        trace: bool,
        lap: &mut impl FnMut(&str, usize),
    ) -> Result<Amg> {
        let n = a.rows();
        let mut nullspace = Nullspace {
            k: nullspace.k,
            vectors: nullspace.vectors.clone(),
        };
        let mut levels: Vec<Level> = Vec::new();
        let mut nonzeros = a.nnz();
        let mut current: Option<Bsr> = None;
        loop {
            let a_l = current.as_ref().unwrap_or(a);
            let n_l = a_l.rows();
            let (inv_diag, inv_chol) = block_diagonal_inverses(a_l);
            let inv_diag_32: Vec<f32> = inv_diag.iter().map(|&d| d as f32).collect();
            let lambda = largest_eigenvalue(a_l, &inv_chol);
            lap("eigenvalue", n_l);
            let coarsest = n_l <= options.coarse_size || levels.len() + 1 >= options.max_levels;
            let coarser = if coarsest {
                None
            } else if let Some(corners) = corners.filter(|_| levels.is_empty()) {
                let k = nullspace.k;
                let size = a_l.br * k;
                let ns_c = Nullspace {
                    k,
                    vectors: (corners.blocks.iter())
                        .flat_map(|&b| {
                            let b = b as usize;
                            nullspace.vectors[b * size..(b + 1) * size].iter().copied()
                        })
                        .collect(),
                };
                let bodies_c = bodies.as_ref().map(|b| {
                    corners
                        .blocks
                        .iter()
                        .map(|&c| b[c as usize])
                        .collect::<Vec<u32>>()
                });
                let p = corners.p.clone();
                let pt = p.transpose();
                lap("corner prolongator", n_l);
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
                Some((p, pt, a_c, ns_c, bodies_c))
            } else {
                let aggregates = aggregate(
                    a_l,
                    &inv_chol,
                    options.strength,
                    options.max_aggregate,
                    bodies.as_deref(),
                );
                lap("aggregation", n_l);
                let n_c = aggregates.count * nullspace.k;
                if n_c * 10 >= n_l * 9 || n_c == 0 {
                    None
                } else {
                    let bodies_c = bodies.as_ref().map(|b| {
                        (0..aggregates.count)
                            .map(|g| b[aggregates.blocks[aggregates.ptr[g]] as usize])
                            .collect::<Vec<u32>>()
                    });
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
                    Some((p, pt, a_c, ns_c, bodies_c))
                }
            };
            match coarser {
                None => {
                    levels.push(Level {
                        a: current.as_ref().map(Bsr::converted),
                        n: n_l,
                        inv_diag: inv_diag_32,
                        lambda_max: lambda,
                        p: None,
                        pt: None,
                    });
                    break;
                }
                Some((p, pt, a_c, ns_c, bodies_c)) => {
                    levels.push(Level {
                        a: current.as_ref().map(Bsr::converted),
                        n: n_l,
                        inv_diag: inv_diag_32,
                        lambda_max: lambda,
                        p: Some(p.converted()),
                        pt: Some(pt.converted()),
                    });
                    current = Some(a_c);
                    nullspace = ns_c;
                    bodies = bodies_c;
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
                failure: plxs_core::Failure::IterativeSolver,
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
            failure: plxs_core::Failure::NotPositiveDefinite,
            message: format!("multigrid: the coarsest system is not positive definite ({e})"),
        })?;
        // The Galerkin product of a positive definite matrix is positive definite; a
        // negative pivot means the matrix itself is not, and the smoothers would diverge.
        if coarse.inertia().is_some_and(|negative| negative > 0) {
            return Err(Error::Numerical {
                step: 0,
                increment: 0,
                failure: plxs_core::Failure::NotPositiveDefinite,
                message: "multigrid: the matrix is not positive definite".to_owned(),
            });
        }
        let last = levels.len() - 1;
        let work = levels
            .iter()
            .enumerate()
            .map(|(l, level)| Work::new(level.n, l == last))
            .collect();
        let fine = a.converted();
        lap("single precision copy", n);
        let coarsest = current.unwrap_or_else(|| a.clone());
        Ok(Amg {
            fine,
            levels,
            coarse,
            coarsest,
            work: Mutex::new(work),
            options,
            nonzeros,
            #[cfg(feature = "gpu")]
            gpu: None,
            cpu_reason: Mutex::new(None),
            patch: Default::default(),
            patch_tried: Default::default(),
            #[cfg(feature = "gpu")]
            cg_note: Mutex::new(None),
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
        let (inv_diag, inv_chol) = block_diagonal_inverses(a);
        level.lambda_max = largest_eigenvalue(a, &inv_chol);
        level.inv_diag = inv_diag.iter().map(|&d| d as f32).collect();
        convert(&a.val, &mut self.fine.val);
        let patch = self.patch.get_mut().unwrap_or_else(|e| e.into_inner());
        if let Some(p) = patch.as_mut()
            && p.refresh(a).is_err()
        {
            *patch = None;
        }
        #[cfg(feature = "gpu")]
        if let Some(gpu) = self.gpu.as_mut() {
            let level = &self.levels[0];
            gpu.refresh(&self.fine, &level.inv_diag, level.lambda_max);
        }
    }

    /// Gives the cycle on the graphics card the operator in double precision, so that
    /// the conjugate gradients run there too (when the card computes in double
    /// precision); called after every build and refresh.
    #[cfg(feature = "gpu")]
    pub(crate) fn attach_operator(&mut self, a: &Bsr) {
        if let Some(gpu) = self.gpu.as_mut()
            && let Err(e) = gpu.attach(a)
        {
            let mut note = self.cg_note.lock().unwrap_or_else(|e| e.into_inner());
            *note = Some(e);
        }
    }

    /// The cycle on the graphics card when the conjugate gradients run there too;
    /// `None`: the processor solves.
    #[cfg(feature = "gpu")]
    pub(crate) fn gpu_cycle(&self) -> Option<&crate::gpu::GpuCycle> {
        self.gpu.as_deref().filter(|g| g.solves())
    }

    /// Notes that the conjugate gradients on the card failed.
    #[cfg(feature = "gpu")]
    pub(crate) fn note_gpu_failure(&self, reason: String) {
        let mut note = self.cg_note.lock().unwrap_or_else(|e| e.into_inner());
        note.get_or_insert(reason);
    }

    /// Where the cycle runs, for the monitor: the graphics card's name, or why the
    /// processor serves although the card was considered; `None` when it was not.
    pub fn placement(&self) -> Option<String> {
        let reason = self.cpu_reason.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(reason) = reason.as_ref() {
            return Some(format!("V-cycle on the processor ({reason})"));
        }
        #[cfg(feature = "gpu")]
        if let Some(gpu) = self.gpu.as_ref() {
            if gpu.solves() {
                return Some(format!(
                    "conjugate gradients and V-cycle on the graphics card {}",
                    gpu.device_name()
                ));
            }
            let note = self.cg_note.lock().unwrap_or_else(|e| e.into_inner());
            return Some(format!(
                "V-cycle on the graphics card {}{}",
                gpu.device_name(),
                note.as_ref()
                    .map(|n| format!(", conjugate gradients on the processor ({n})"))
                    .unwrap_or_default()
            ));
        }
        None
    }

    pub fn nonzeros(&self) -> usize {
        self.nonzeros
    }

    /// Unknowns of the direct solve of the slow region, when there is one.
    pub fn patch_size(&self) -> Option<usize> {
        let patch = self.patch.read().unwrap_or_else(|e| e.into_inner());
        patch.as_ref().map(|p| p.len())
    }

    /// Whether the cycle may still get a patch: on the processor, allowed by the options
    /// and not looked for yet. Marks it as looked for.
    pub(crate) fn wants_patch(&self) -> bool {
        #[cfg(feature = "gpu")]
        if self.gpu.is_some() {
            return false;
        }
        self.options.patch
            && self.levels.len() > 1
            && !self
                .patch_tried
                .swap(true, std::sync::atomic::Ordering::Relaxed)
    }

    /// Gives the finest level's cycle the direct solve of a slow region.
    pub(crate) fn set_patch(&self, patch: crate::patch::Patch) {
        *self.patch.write().unwrap_or_else(|e| e.into_inner()) = Some(patch);
    }

    /// Sizes of the levels, finest first.
    pub fn sizes(&self) -> Vec<usize> {
        self.levels.iter().map(|l| l.n).collect()
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
        #[cfg(feature = "gpu")]
        if let Some(gpu) = self.gpu.as_ref() {
            match gpu.apply(r, z) {
                Ok(()) => return Ok(()),
                Err(e) => {
                    // Once failed, the processor serves from here on.
                    let mut reason = self.cpu_reason.lock().unwrap_or_else(|e| e.into_inner());
                    reason.get_or_insert_with(|| {
                        format!("the graphics card {} failed: {e}", gpu.device_name())
                    });
                }
            }
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
        let guard = (l == 0).then(|| self.patch.read().unwrap_or_else(|e| e.into_inner()));
        let patch = guard.as_ref().and_then(|g| g.as_ref());
        if let Some(patch) = patch {
            patch.correct_residual(a, &mut this.x, &mut this.r)?;
        }
        let next = &mut rest[0];
        pt.multiply(&this.r, &mut next.b);
        self.cycle(l + 1, rest)?;
        p.multiply(&rest[0].x, &mut this.t);
        add(&mut this.x, &this.t);
        if let Some(patch) = patch {
            patch.correct(a, &this.b, &mut this.x)?;
        }
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

/// Chebyshev smoothing of `A x = b` on the upper part of the spectrum of `D⁻¹ A` (`D` the
/// block diagonal), `degree` products.
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
    let br = a.br;
    if x_is_zero {
        w.r.copy_from_slice(&w.b);
        scaled(&mut w.d, inv_diag, br, &w.r, (1.0 / theta) as f32);
        w.x.copy_from_slice(&w.d);
    } else {
        a.residual(&w.b, &w.x, &mut w.r);
        scaled(&mut w.d, inv_diag, br, &w.r, (1.0 / theta) as f32);
        add(&mut w.x, &w.d);
    }
    for _ in 1..degree {
        let rho_new = 1.0 / (2.0 * sigma - rho_old);
        a.multiply(&w.d, &mut w.t);
        sub(&mut w.r, &w.t);
        let (c1, c2) = ((rho_new * rho_old) as f32, (2.0 * rho_new / delta) as f32);
        // d = c1 d + c2 D⁻¹ r; x += d
        update(&mut w.d, &w.r, inv_diag, br, c1, c2);
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

/// `d = c · D⁻¹ r` with the inverse diagonal blocks `inv` (`br × br` each).
fn scaled<T: Scalar>(d: &mut [T], inv: &[T], br: usize, r: &[T], c: T) {
    block_apply(d, inv, br, r, |_, s| c * s);
}

/// `d = c1 d + c2 D⁻¹ r`.
fn update<T: Scalar>(d: &mut [T], r: &[T], inv: &[T], br: usize, c1: T, c2: T) {
    block_apply(d, inv, br, r, |d, s| c1 * d + c2 * s);
}

/// `d_i = f(d_i, (D⁻¹ r)_i)` block by block, in parallel on large vectors.
fn block_apply<T: Scalar>(
    d: &mut [T],
    inv: &[T],
    br: usize,
    r: &[T],
    f: impl Fn(T, T) -> T + Sync,
) {
    let size = br * br;
    let one = |d: &mut [T], inv: &[T], r: &[T]| {
        for (b, d) in d.chunks_exact_mut(br).enumerate() {
            let inv = &inv[b * size..(b + 1) * size];
            let r = &r[b * br..(b + 1) * br];
            for (i, d) in d.iter_mut().enumerate() {
                let mut s = T::default();
                for (j, &r) in r.iter().enumerate() {
                    s += inv[i * br + j] * r;
                }
                *d = f(*d, s);
            }
        }
    };
    const CHUNK: usize = 2048;
    if d.len() < PARALLEL_FROM {
        one(d, inv, r);
    } else {
        d.par_chunks_mut(CHUNK * br)
            .zip(inv.par_chunks(CHUNK * size))
            .zip(r.par_chunks(CHUNK * br))
            .for_each(|((d, inv), r)| one(d, inv, r));
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

/// The first coarsening of a mesh of quadratic elements: its corner nodes.
struct Corners {
    /// The fine blocks of the corner nodes, ascending: the coarse blocks.
    blocks: Vec<u32>,
    /// The interpolation of the linear elements on the same corners: a corner node takes
    /// its own value, a midside node the mean of the ends of its edge.
    p: Bsr,
}

/// The relative shift of the diagonal of the corner level with rotations (see
/// [`Amg::linked`]).
const LINKED_SHIFT: f64 = 1e-8;

/// The corner level with rotations is solved directly when its factorization costs at
/// most this many floating point operations per nonzero of the matrix ...
const LINKED_FLOPS_PER_NONZERO: f64 = 2000.0;

/// ... and its factor holds at most this many nonzeros per nonzero of the matrix (plates
/// of one and two layers of tetrahedra: 1400 and 2000 operations, 2.0 and 2.2 nonzeros;
/// a bracket 2500 and 2.4, a cube 21 000 and 6.5).
const LINKED_FACTOR_PER_NONZERO: f64 = 2.3;

/// A midside node is held from outside when the blocks of its row within its body add up
/// to more than this fraction of its diagonal block (Frobenius norms).
const HELD: f64 = 0.05;

/// The corner nodes of `a`'s blocks and the interpolation from them, when enough of the
/// nodes are midside nodes for the corner level to be worth its while.
///
/// With `rotations` every corner carries a rotation besides its translation, and a
/// midside node `m` on the edge from `a` to `b` moves by the linked interpolation
/// `u_m = ½ (t_a + t_b) + ½ ω_a × (x_m − x_a) + ½ ω_b × (x_m − x_b)` (for `m` halfway:
/// `¼ (ω_a − ω_b) × (x_b − x_a)`), the interpolation of elements with rotational degrees
/// of freedom. The linear interpolation of the corners locks in bending: a thin wall a
/// few elements thick bends on the corner level only by shearing its elements, and the
/// level is far too stiff for the slow modes of the wall. With the rotations it bends
/// almost freely (a plate one element thick, two levels with exact solves: 49
/// iterations without, 25 with).
fn corner_prolongator(a: &Bsr, u: &Unknowns, rotations: bool) -> Option<Corners> {
    const NONE: u32 = u32::MAX;
    let (blocks, br) = (a.brows, a.br);
    if a.bc != br || blocks == 0 {
        return None;
    }
    let block_of: std::collections::HashMap<u32, u32> =
        (0..blocks).map(|b| (u.node[b * br], b as u32)).collect();
    // A midside node that something holds from outside its body's elements (a spring to
    // the ground, a gap, the penalty of a contact) stays on the corner level as it is:
    // on a curved face the mean of the ends of its edge lies off the face, so the linear
    // interpolation of a motion along the face would move the node against what holds
    // it, and the corner level could not slide at all (a lifting lug on gap springs took
    // 170 to 250 iterations instead of 46 to 76). The elements of a body resist no
    // translation, so the blocks of a node's row within its body add up to zero unless
    // something else holds the node (or a neighbour is prescribed).
    let size = br * br;
    let bodies = (u.body.len() == u.node.len()).then_some(&u.body);
    let held: Vec<bool> = (0..blocks)
        .into_par_iter()
        .map(|b| {
            if u.midside[b * br] == [NONE; 2] {
                return false;
            }
            let (col, val) = a.block_row(b);
            let mut sum = vec![0.0; size];
            let mut diagonal = 0.0;
            for (&c, block) in col.iter().zip(val.chunks_exact(size)) {
                let c = c as usize;
                if c == b {
                    diagonal = block.iter().map(|v| v * v).sum::<f64>();
                }
                if bodies.is_none_or(|body| body[c * br] == body[b * br]) {
                    sum.iter_mut().zip(block).for_each(|(s, v)| *s += v);
                }
            }
            sum.iter().map(|v| v * v).sum::<f64>() > HELD * HELD * diagonal
        })
        .collect();
    let is_corner = |b: usize| u.midside[b * br] == [NONE; 2] || held[b];
    let mut coarse_of = vec![NONE; blocks];
    let mut corners = Vec::new();
    for (b, coarse) in coarse_of.iter_mut().enumerate() {
        if is_corner(b) {
            *coarse = corners.len() as u32;
            corners.push(b as u32);
        }
    }
    if corners.len() * 10 > blocks * 8 {
        return None;
    }
    let bc = if rotations { 6 } else { br };
    if rotations && (br != 3 || u.coordinates.len() != u.node.len()) {
        return None;
    }
    let mut ptr = Vec::with_capacity(blocks + 1);
    let mut col = Vec::new();
    let mut val = Vec::new();
    ptr.push(0);
    // The block `weight · [I, S]` with `S δ = ω × d` (rotations) or `weight · I`.
    let push = |weight: f64, d: [f64; 3], val: &mut Vec<f64>| {
        for r in 0..br {
            for c in 0..bc {
                val.push(if c < br {
                    if r == c { weight } else { 0.0 }
                } else {
                    // (ω × d)_r = ε_rjk ω_j d_k: column j = c - 3.
                    let j = c - br;
                    let k = 3 - r - j;
                    if r == j {
                        0.0
                    } else if (r + 1) % 3 == j {
                        weight * d[k]
                    } else {
                        -weight * d[k]
                    }
                });
            }
        }
    };
    let position = |b: usize| u.coordinates[b * br];
    for b in 0..blocks {
        if is_corner(b) {
            col.push(coarse_of[b]);
            push(1.0, [0.0; 3], &mut val);
        } else {
            let mut ends: Vec<(u32, u32)> = (u.midside[b * br].iter())
                .filter_map(|end| block_of.get(end))
                .map(|&e| (coarse_of[e as usize], e))
                .filter(|&(c, _)| c != NONE)
                .collect();
            ends.sort_unstable();
            ends.dedup();
            let x_m = position(b);
            for (c, e) in ends {
                col.push(c);
                let x_e = position(e as usize);
                let d = if rotations {
                    [x_m[0] - x_e[0], x_m[1] - x_e[1], x_m[2] - x_e[2]]
                } else {
                    [0.0; 3]
                };
                push(0.5, d, &mut val);
            }
        }
        ptr.push(col.len());
    }
    debug_assert_eq!(val.len(), col.len() * br * bc);
    Some(Corners {
        p: Bsr {
            brows: blocks,
            bcols: corners.len(),
            br,
            bc,
            ptr,
            col,
            val,
        },
        blocks: corners,
    })
}

/// Groups the blocks of a level by the three aggregation phases on the graph of strong
/// connections (Frobenius norms of the coupling blocks). Blocks of different bodies
/// (`body`, when given) are never strongly connected: a penalty contact spring between two
/// bodies is as stiff as the elements, yet it holds only the normal direction, and an
/// aggregate across it would force the two bodies to slide together on the coarse levels,
/// although sliding costs no energy. That mode is then left to the smoother, which
/// cannot reduce it, and the iteration count grows several times over.
fn aggregate(
    a: &Bsr,
    inv_chol: &[f64],
    strength: f64,
    max_aggregate: usize,
    body: Option<&[u32]>,
) -> Aggregates {
    let blocks = a.brows;
    let br = a.br;
    let size = br * br;
    // Strong: ‖L_b⁻¹ A_bc L_c⁻ᵀ‖ ≥ θ (Frobenius; the coupling scaled by the block diagonal,
    // `D_b = L_b L_bᵀ`), with the squared norms as weights. Scalar diagonals would call
    // every coupling of a node with a stiff spring to the ground weak, in all directions.
    let theta2 = strength * strength;
    let strong = |b: usize| {
        let (col, val) = a.block_row(b);
        let lb = &inv_chol[b * size..(b + 1) * size];
        col.iter().enumerate().filter_map(move |(k, &c)| {
            let c = c as usize;
            if c == b || body.is_some_and(|body| body[b] != body[c]) {
                return None;
            }
            let block = &val[size * k..size * (k + 1)];
            let lc = &inv_chol[c * size..(c + 1) * size];
            // M = L_b⁻¹ A_bc L_c⁻ᵀ; L⁻¹ lower.
            let mut w = 0.0;
            for i in 0..br {
                for j in 0..br {
                    let mut m = 0.0;
                    for s in 0..=i {
                        let mut t = 0.0;
                        for u in j..br {
                            t += block[s * br + u] * lc[u * br + j];
                        }
                        m += lb[i * br + s] * t;
                    }
                    w += m * m;
                }
            }
            (w >= theta2).then_some((c, w))
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

/// The estimate is scaled by this: an eigenvalue above the smoother's interval is
/// amplified, and the preconditioner would stop being positive definite (a pulley of
/// quadratic tetrahedra whose estimate by 12 power iterations was 4 % low: negative
/// `r·z` in the conjugate gradients, 84 instead of 72 iterations). The Lanczos estimate
/// is within a fraction of a percent after 30 steps, so a small margin suffices; a larger
/// one widens the interval and weakens the smoother (1.2: 88 iterations on that pulley).
const EIGENVALUE_MARGIN: f64 = 1.02;

/// The inverses of the diagonal blocks of `a` (`br × br` each, row-major by block row) and
/// the inverses `L⁻¹` of their Cholesky factors (`D_b = L_b L_bᵀ`, lower). A block that is
/// not positive definite (a zero diagonal) gets the inverses of its diagonal entries (1
/// where that is zero) and their square roots.
fn block_diagonal_inverses(a: &Bsr) -> (Vec<f64>, Vec<f64>) {
    let br = a.br;
    let size = br * br;
    let mut inv = vec![0.0; a.brows * size];
    let mut inv_chol = vec![0.0; a.brows * size];
    let scalar = |block: &[f64], inv: &mut [f64], inv_chol: &mut [f64]| {
        for i in 0..br {
            let d = block[i * br + i];
            let x = if d > 0.0 { 1.0 / d } else { 1.0 };
            inv[i * br + i] = x;
            inv_chol[i * br + i] = x.sqrt();
        }
    };
    let zero = vec![0.0; size];
    for b in 0..a.brows {
        let block = a
            .diagonal_block(b)
            .map_or(&zero[..], |k| &a.val[size * k..size * (k + 1)]);
        let inv = &mut inv[b * size..(b + 1) * size];
        let inv_chol = &mut inv_chol[b * size..(b + 1) * size];
        // Cholesky, lower.
        let mut l = vec![0.0; size];
        let mut ok = true;
        'chol: for i in 0..br {
            for j in 0..=i {
                let mut sum = block[i * br + j];
                for k in 0..j {
                    sum -= l[i * br + k] * l[j * br + k];
                }
                if i == j {
                    if sum <= 1e-14 * block[i * br + i].abs() || !sum.is_finite() {
                        ok = false;
                        break 'chol;
                    }
                    l[i * br + i] = sum.sqrt();
                } else {
                    l[i * br + j] = sum / l[j * br + j];
                }
            }
        }
        if !ok {
            scalar(block, inv, inv_chol);
            continue;
        }
        // L⁻¹ by forward substitution, column by column.
        for c in 0..br {
            for i in c..br {
                let mut sum = if i == c { 1.0 } else { 0.0 };
                for k in c..i {
                    sum -= l[i * br + k] * inv_chol[k * br + c];
                }
                inv_chol[i * br + c] = sum / l[i * br + i];
            }
        }
        // D⁻¹ = L⁻ᵀ L⁻¹.
        for i in 0..br {
            for j in 0..br {
                let mut sum = 0.0;
                for k in i.max(j)..br {
                    sum += inv_chol[k * br + i] * inv_chol[k * br + j];
                }
                inv[i * br + j] = sum;
            }
        }
    }
    (inv, inv_chol)
}

/// `y = L⁻¹ x` block by block (`inv_chol` the lower blocks `L_b⁻¹`).
fn block_lower(inv_chol: &[f64], br: usize, x: &[f64], y: &mut [f64]) {
    let size = br * br;
    for (b, y) in y.chunks_exact_mut(br).enumerate() {
        let l = &inv_chol[b * size..(b + 1) * size];
        let x = &x[b * br..(b + 1) * br];
        for (i, y) in y.iter_mut().enumerate() {
            *y = (0..=i).map(|j| l[i * br + j] * x[j]).sum();
        }
    }
}

/// `y = L⁻ᵀ x` block by block.
fn block_lower_transposed(inv_chol: &[f64], br: usize, x: &[f64], y: &mut [f64]) {
    let size = br * br;
    for (b, y) in y.chunks_exact_mut(br).enumerate() {
        let l = &inv_chol[b * size..(b + 1) * size];
        let x = &x[b * br..(b + 1) * br];
        for (i, y) in y.iter_mut().enumerate() {
            *y = (i..br).map(|j| l[j * br + i] * x[j]).sum();
        }
    }
}

/// `λ_max(D⁻¹ A)` by Lanczos, with a margin (`PLXS_AMG_MARGIN` for measurements).
fn largest_eigenvalue(a: &Bsr, inv_chol: &[f64]) -> f64 {
    let margin: f64 = std::env::var("PLXS_AMG_MARGIN")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(EIGENVALUE_MARGIN);
    largest_eigenvalue_lanczos(a, inv_chol, EIGENVALUE_STEPS) * margin
}

/// Lanczos steps of the eigenvalue estimate of a level.
const EIGENVALUE_STEPS: usize = 30;
/// `λ_max(D⁻¹ A)` as the largest Ritz value of `steps` Lanczos steps on the symmetric
/// `L⁻¹ A L⁻ᵀ` (same spectrum; `D = L Lᵀ` the block diagonal), no reorthogonalization: the
/// extreme eigenvalue converges in a few dozen steps where the power iteration needs
/// hundreds.
fn largest_eigenvalue_lanczos(a: &Bsr, inv_chol: &[f64], steps: usize) -> f64 {
    let n = a.rows();
    if n == 0 {
        return 1.0;
    }
    let br = a.br;
    let mut v: Vec<f64> = (0..n)
        .map(|i| 1.0 + 0.5 * ((i * 7919 % 1000) as f64 / 1000.0))
        .collect();
    let norm = v.iter().map(|x| x * x).sum::<f64>().sqrt();
    v.iter_mut().for_each(|x| *x /= norm);
    let mut v_old = vec![0.0; n];
    let mut w = vec![0.0; n];
    let mut t = vec![0.0; n];
    let mut alphas = Vec::new();
    let mut betas = vec![0.0];
    let mut beta = 0.0;
    for _ in 0..steps {
        // w = L⁻¹ A L⁻ᵀ v
        block_lower_transposed(inv_chol, br, &v, &mut t);
        a.multiply(&t, &mut w);
        let mut scaled_w = vec![0.0; n];
        block_lower(inv_chol, br, &w, &mut scaled_w);
        w.copy_from_slice(&scaled_w);
        let alpha: f64 = w.iter().zip(&v).map(|(w, v)| w * v).sum();
        w.iter_mut()
            .zip(&v)
            .zip(&v_old)
            .for_each(|((w, v), o)| *w -= alpha * v + beta * o);
        alphas.push(alpha);
        beta = w.iter().map(|x| x * x).sum::<f64>().sqrt();
        if beta == 0.0 || !beta.is_finite() {
            break;
        }
        betas.push(beta);
        v_old.copy_from_slice(&v);
        v.iter_mut().zip(&w).for_each(|(v, w)| *v = w / beta);
    }
    // Largest eigenvalue of the tridiagonal by bisection on the Sturm count.
    let m = alphas.len();
    let count_below = |x: f64| {
        let mut count = 0;
        let mut q = 1.0;
        for k in 0..m {
            let o = if k > 0 { betas[k] * betas[k] } else { 0.0 };
            q = alphas[k] - x - if q != 0.0 { o / q } else { o / 1e-300 };
            if q < 0.0 {
                count += 1;
            }
        }
        count
    };
    let radius = (0..m)
        .map(|k| alphas[k].abs() + betas[k].abs() + betas.get(k + 1).map_or(0.0, |o| o.abs()))
        .fold(0.0_f64, f64::max);
    let (mut lo, mut hi) = (-radius, radius);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if count_below(mid) >= m {
            hi = mid;
        } else {
            lo = mid;
        }
        if hi - lo <= 1e-10 * radius {
            break;
        }
    }
    0.5 * (lo + hi)
}

/// `P = (I − ω D⁻¹ A) P_t` with `ω = 4 / (3 λ_max)` and `D` the block diagonal
/// (`inv_diag` its inverse blocks).
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
                    let inv = &inv_diag[i * size..(i + 1) * size];
                    for r in 0..br {
                        for s in 0..br {
                            let w = -omega * inv[r * br + s];
                            if w != 0.0 {
                                for j in 0..k {
                                    out[r * k + j] += w * product[s * k + j];
                                }
                            }
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

    /// The nodes of a chain of quadratic edges along a bent line: corners at the even
    /// positions, midside nodes halfway between, three unknowns each; the stiffness of
    /// springs between all nodes up to two apart (rows that add up to zero), with a
    /// spring to the ground at the first node when `ground`.
    fn quadratic_chain(edges: usize, ground: bool) -> (Bsr, Unknowns) {
        let nodes = 2 * edges + 1;
        let position = |i: usize| {
            let t = i as f64 * 0.5;
            [t, (0.3 * t).sin(), 0.1 * t * t]
        };
        let none = [u32::MAX; 2];
        let unknowns = Unknowns {
            node: (0..nodes as u32).flat_map(|n| [n; 3]).collect(),
            component: (0..nodes).flat_map(|_| [0, 1, 2]).collect(),
            coordinates: (0..nodes).flat_map(|i| [position(i); 3]).collect(),
            body: Vec::new(),
            midside: (0..nodes)
                .flat_map(|i| {
                    let ends = if i % 2 == 1 {
                        [i as u32 - 1, i as u32 + 1]
                    } else {
                        none
                    };
                    [ends; 3]
                })
                .collect(),
        };
        let a = Bsr::build(
            nodes,
            nodes,
            3,
            3,
            || (),
            |i, _, acc| {
                let near = i.saturating_sub(2)..(i + 3).min(nodes);
                let mut diagonal = if ground && i == 0 { 5.0 } else { 0.0 };
                for j in near.clone().filter(|&j| j != i) {
                    let k = 1.0 + ((i + j) % 3) as f64;
                    diagonal += k;
                    let block = acc.block(j as u32);
                    for r in 0..3 {
                        block[r * 3 + r] = -k;
                    }
                }
                let block = acc.block(i as u32);
                for r in 0..3 {
                    block[r * 3 + r] = diagonal;
                }
            },
        );
        (a, unknowns)
    }

    #[test]
    fn the_linked_interpolation_moves_the_midside_nodes_with_a_rigid_rotation() {
        let (a, unknowns) = quadratic_chain(4, false);
        let corners = corner_prolongator(&a, &unknowns, true).unwrap();
        assert_eq!(corners.blocks, [0, 2, 4, 6, 8]);
        let p = &corners.p;
        assert_eq!((p.brows, p.bcols, p.br, p.bc), (9, 5, 3, 6));
        // A rotation w about the origin and a translation s: t_c = s + w × x_c, ω_c = w.
        let (w, s) = ([0.2, -0.7, 0.4], [1.0, 2.0, -3.0]);
        let cross = |a: [f64; 3], b: [f64; 3]| {
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        };
        let x = |b: usize| unknowns.coordinates[b * 3];
        let coarse: Vec<f64> = (corners.blocks.iter())
            .flat_map(|&b| {
                let t = cross(w, x(b as usize));
                [s[0] + t[0], s[1] + t[1], s[2] + t[2], w[0], w[1], w[2]]
            })
            .collect();
        let mut fine = vec![0.0; 27];
        p.multiply(&coarse, &mut fine);
        for b in 0..9 {
            let t = cross(w, x(b));
            for r in 0..3 {
                let expected = s[r] + t[r];
                assert!((fine[b * 3 + r] - expected).abs() < 1e-12, "node {b}");
            }
        }
        // Without rotations the blocks are the translations only.
        let plain = corner_prolongator(&a, &unknowns, false).unwrap();
        assert_eq!(plain.p.bc, 3);
    }

    #[test]
    fn the_corner_level_with_rotations_is_solved_directly_below_the_fine_level() {
        let (a, unknowns) = quadratic_chain(40, true);
        let options = AmgOptions {
            rotations: Rotations::On,
            ..AmgOptions::default()
        };
        let amg = Amg::build(&a, Some(&unknowns), &[], options).unwrap();
        assert_eq!(amg.levels.len(), 2);
        assert_eq!(amg.levels[1].n, 41 * 6);
        // As a preconditioner it is symmetric and positive definite: rᵀ M r > 0 and
        // sᵀ M r = rᵀ M s.
        let n = a.rows();
        let r: Vec<f64> = (0..n).map(|i| ((i * 7) % 5) as f64 - 2.0).collect();
        let t: Vec<f64> = (0..n).map(|i| ((i * 3) % 7) as f64 - 3.0).collect();
        let (mut mr, mut mt) = (vec![0.0; n], vec![0.0; n]);
        amg.apply(&r, &mut mr).unwrap();
        amg.apply(&t, &mut mt).unwrap();
        let dot = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(a, b)| a * b).sum::<f64>();
        assert!(dot(&r, &mr) > 0.0);
        let (tmr, rmt) = (dot(&t, &mr), dot(&r, &mt));
        assert!(
            (tmr - rmt).abs() <= 1e-4 * tmr.abs().max(rmt.abs()),
            "{tmr} {rmt}"
        );
        // Off (the default), the ordinary hierarchy: this system is small enough to be
        // its own coarsest level.
        let amg = Amg::build(&a, Some(&unknowns), &[], AmgOptions::default()).unwrap();
        assert_eq!(amg.levels.len(), 1);
    }

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

    /// Two chains of nodes with full elastic couplings, joined by one stiff spring that
    /// holds a single direction (a penalty contact spring).
    fn two_bodies_in_contact(m: usize) -> (Bsr, Vec<u32>) {
        let n = 2 * m;
        let elastic = |k: f64| -> [f64; 9] { [k, 0.1 * k, 0.0, 0.1 * k, k, 0.0, 0.0, 0.0, k] };
        let mut rows: Vec<Vec<(u32, [f64; 9])>> = vec![Vec::new(); n];
        let mut couple = |i: usize, j: usize, block: [f64; 9]| {
            rows[i].push((j as u32, block));
            rows[j].push((i as u32, block));
            for d in [i, j] {
                rows[d].push((d as u32, block));
            }
        };
        for body in 0..2 {
            for i in body * m..(body + 1) * m - 1 {
                couple(i, i + 1, elastic(-1.0));
            }
        }
        // The contact spring in the direction x, as stiff as the elements.
        let mut spring = [0.0; 9];
        spring[0] = -3.0;
        couple(m - 1, m, spring);
        let mut ptr = vec![0usize];
        let mut col = Vec::new();
        let mut val = Vec::new();
        for (i, row) in rows.iter_mut().enumerate() {
            row.sort_by_key(|(c, _)| *c);
            let mut merged: Vec<(u32, [f64; 9])> = Vec::new();
            for &(c, block) in row.iter() {
                let sign = if c as usize == i { -1.0 } else { 1.0 };
                match merged.last_mut() {
                    Some((last, b)) if *last == c => {
                        b.iter_mut().zip(block).for_each(|(b, v)| *b += sign * v)
                    }
                    _ => merged.push((c, block.map(|v| sign * v))),
                }
            }
            for (c, block) in merged {
                col.push(c);
                val.extend_from_slice(&block);
            }
            ptr.push(col.len());
        }
        let a = Bsr {
            brows: n,
            bcols: n,
            br: 3,
            bc: 3,
            ptr,
            col,
            val,
        };
        let body = (0..n as u32).map(|i| i / m as u32).collect();
        (a, body)
    }

    /// A grounded chain of `n` nodes (3 × 3 blocks), positive definite.
    fn grounded_chain(n: usize) -> Bsr {
        let mut ptr = vec![0usize];
        let mut col = Vec::new();
        let mut val = Vec::new();
        for i in 0..n {
            for j in i.saturating_sub(1)..(i + 2).min(n) {
                col.push(j as u32);
                let mut block = [0.0; 9];
                for c in 0..3 {
                    block[4 * c] = if i == j {
                        let links = (i > 0) as usize + (i + 1 < n) as usize;
                        links as f64 * (1.0 + c as f64) + 1e-3
                    } else {
                        -(1.0 + c as f64)
                    };
                }
                val.extend_from_slice(&block);
            }
            ptr.push(col.len());
        }
        Bsr {
            brows: n,
            bcols: n,
            br: 3,
            bc: 3,
            ptr,
            col,
            val,
        }
    }

    #[test]
    fn a_local_error_makes_a_patch_of_its_nodes_and_their_ring() {
        let a = grounded_chain(400);
        let mut error = vec![1e-4; a.rows()];
        for b in 100..106 {
            for c in 0..3 {
                error[3 * b + c] = 1.0;
            }
        }
        let patch = crate::patch::Patch::find(&a, &error, None, false)
            .unwrap()
            .expect("a local error");
        assert_eq!(patch.len(), 3 * 8);
        // Spread over the whole chain: no patch.
        let spread = vec![1.0; a.rows()];
        assert!(
            crate::patch::Patch::find(&a, &spread, None, false)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn the_cycle_with_a_patch_stays_symmetric() {
        let a = grounded_chain(3000);
        let amg = Amg::build(&a, None, &[], AmgOptions::default()).unwrap();
        assert!(amg.levels() > 1);
        let mut error = vec![0.0; a.rows()];
        error[3 * 1500..3 * 1520].iter_mut().for_each(|v| *v = 1.0);
        let patch = crate::patch::Patch::find(&a, &error, None, false)
            .unwrap()
            .unwrap();
        amg.set_patch(patch);
        let x: Vec<f64> = (0..a.rows())
            .map(|i| ((i * 37) % 11) as f64 - 5.0)
            .collect();
        let y: Vec<f64> = (0..a.rows()).map(|i| ((i * 13) % 7) as f64 - 3.0).collect();
        let (mut mx, mut my) = (vec![0.0; a.rows()], vec![0.0; a.rows()]);
        amg.apply(&x, &mut mx).unwrap();
        amg.apply(&y, &mut my).unwrap();
        let dot = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(a, b)| a * b).sum::<f64>();
        let (xy, yx) = (dot(&my, &x), dot(&mx, &y));
        assert!(
            (xy - yx).abs() <= 1e-4 * xy.abs().max(yx.abs()),
            "{xy} vs {yx}"
        );
        // Positive: the patch adds a correction, it never takes energy away.
        assert!(dot(&mx, &x) > 0.0);
    }

    #[test]
    fn aggregates_stay_within_their_bodies() {
        let (a, body) = two_bodies_in_contact(40);
        let crossing = |agg: &Aggregates| {
            (0..agg.count).any(|g| {
                let members = &agg.blocks[agg.ptr[g]..agg.ptr[g + 1]];
                members.iter().any(|&b| body[b as usize] == 0)
                    && members.iter().any(|&b| body[b as usize] == 1)
            })
        };
        let (_, inv_chol) = block_diagonal_inverses(&a);
        assert!(crossing(&aggregate(&a, &inv_chol, 0.01, 27, None)));
        let agg = aggregate(&a, &inv_chol, 0.01, 27, Some(&body[..]));
        assert!(!crossing(&agg));
        assert_eq!(agg.of_block.len(), 80);
        assert!(agg.count >= 2);
    }

    #[test]
    fn the_block_diagonal_inverses_invert_the_diagonal_blocks() {
        let (a, _) = two_bodies_in_contact(10);
        let (inv, inv_chol) = block_diagonal_inverses(&a);
        let br = a.br;
        let size = br * br;
        for b in 0..a.brows {
            let k = a.diagonal_block(b).expect("a diagonal block");
            let d = &a.val[size * k..size * (k + 1)];
            let (di, li) = (
                &inv[b * size..(b + 1) * size],
                &inv_chol[b * size..(b + 1) * size],
            );
            for i in 0..br {
                for j in 0..br {
                    // D D⁻¹ = I and L⁻ᵀ L⁻¹ = D⁻¹.
                    let product: f64 = (0..br).map(|s| d[i * br + s] * di[s * br + j]).sum();
                    let expected = if i == j { 1.0 } else { 0.0 };
                    assert!(
                        (product - expected).abs() < 1e-9,
                        "block {b}: {product} vs {expected}"
                    );
                    let ltl: f64 = (0..br).map(|s| li[s * br + i] * li[s * br + j]).sum();
                    assert!((ltl - di[i * br + j]).abs() <= 1e-9 * di[i * br + i].abs().max(1.0));
                }
            }
        }
    }

    #[test]
    fn midside_nodes_interpolate_the_ends_of_their_edges() {
        // Five nodes in a row, each coupled to the next: corners 10, 30, 50 with the
        // midside nodes 20 and 40 between them and a midside node 60 whose other end is
        // held (no unknowns, the number 99 no node has).
        let nodes = [10u32, 20, 30, 40, 50, 60];
        let ends = [
            [u32::MAX; 2],
            [10, 30],
            [u32::MAX; 2],
            [50, 30],
            [u32::MAX; 2],
            [50, 99],
        ];
        let unknowns = Unknowns {
            node: nodes.iter().flat_map(|&n| [n, n]).collect(),
            component: nodes.iter().flat_map(|_| [0, 1]).collect(),
            coordinates: (0..6).flat_map(|i| [[i as f64, 0.0, 0.0]; 2]).collect(),
            body: Vec::new(),
            midside: ends.iter().flat_map(|&e| [e, e]).collect(),
        };
        // A chain of springs between neighbours, free at both ends: no translation is
        // resisted. `ground` is a spring to the ground on one node.
        let chain = |ground: Option<usize>| {
            Bsr::build(
                6,
                6,
                2,
                2,
                || (),
                |i, _, acc| {
                    let neighbours = i.saturating_sub(1)..(i + 2).min(6);
                    let held = if ground == Some(i) { 5.0 } else { 0.0 };
                    for j in neighbours.clone() {
                        let v = if i == j {
                            (neighbours.len() - 1) as f64 + held
                        } else {
                            -1.0
                        };
                        acc.block(j as u32).copy_from_slice(&[v, 0.0, 0.0, v]);
                    }
                },
            )
        };
        let a = chain(None);
        let corners = corner_prolongator(&a, &unknowns, false).unwrap();
        assert_eq!(corners.blocks, [0, 2, 4]);
        let p = &corners.p;
        assert_eq!((p.brows, p.bcols, p.br, p.bc), (6, 3, 2, 2));
        let row = |i: usize| {
            let (col, val) = p.block_row(i);
            (col.to_vec(), val.to_vec())
        };
        assert_eq!(row(0), (vec![0], vec![1.0, 0.0, 0.0, 1.0]));
        assert_eq!(
            row(1),
            (vec![0, 1], vec![0.5, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0, 0.5])
        );
        assert_eq!(
            row(3),
            (vec![1, 2], vec![0.5, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0, 0.5])
        );
        assert_eq!(row(5), (vec![2], vec![0.5, 0.0, 0.0, 0.5]));
        // A linear function at the corners is the same function at the midside nodes.
        let coarse = [1.0, 10.0, 3.0, 30.0, 5.0, 50.0];
        let mut fine = [0.0; 12];
        p.multiply(&coarse, &mut fine);
        assert_eq!(
            fine[..10],
            [1.0, 10.0, 2.0, 20.0, 3.0, 30.0, 4.0, 40.0, 5.0, 50.0]
        );

        // A midside node held by a spring to the ground stays on the corner level.
        let held = corner_prolongator(&chain(Some(3)), &unknowns, false).unwrap();
        assert_eq!(held.blocks, [0, 2, 3, 4]);
        let (col, val) = held.p.block_row(3);
        assert_eq!((col, val), (&[2u32][..], &[1.0, 0.0, 0.0, 1.0][..]));

        // A mesh with hardly any midside nodes keeps its aggregates.
        let mut linear = unknowns.clone();
        linear.midside = vec![[u32::MAX; 2]; 12];
        linear.midside[2] = [10, 30];
        linear.midside[3] = [10, 30];
        assert!(corner_prolongator(&a, &linear, false).is_none());
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
