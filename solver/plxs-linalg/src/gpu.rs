//! The V-cycle of the multigrid preconditioner ([`crate::amg`]) on the graphics card, by
//! compute shaders through wgpu (Vulkan, DirectX 12 or Metal, whatever the system has).
//!
//! The hierarchy is uploaded once per build in single precision, the precision the
//! cycle has on the processor too; per application only the residual goes up and the
//! correction comes back, the conjugate gradients outside stay in double precision on
//! the processor. Every operation of a cycle (block products, the Chebyshev smoother's
//! vector updates, the restriction and prolongation) is one dispatch of a fixed list,
//! recorded once and submitted as one command buffer per application. The coarsest
//! system, factorized on the processor, is applied as its dense inverse (a few thousand
//! unknowns at most), so a cycle needs no round trip in its middle.
//!
//! The matrices are stored sliced ("sliced ELLPACK" of blocks, after Kreutzer, Hager,
//! Wellein, Fehske, Bishop, "A unified sparse matrix data format for efficient general
//! sparse matrix-vector multiplication on modern processors with wide SIMD units", SIAM
//! J. Sci. Comput. 36 (2014)): groups of 32 block rows store their `j`-th blocks next
//! to each other, so that neighbouring threads read neighbouring memory; inside windows
//! of 256 block rows the rows are sorted by length to keep the padding small.
//!
//! When the card is used: `PLXS_GPU=off` never, `PLXS_GPU=on` always (any adapter,
//! software renderers included, for tests and measurements), otherwise (automatic) only a
//! discrete graphics card and systems of at least [`GPU_FROM`] unknowns; integrated
//! graphics share the memory bandwidth of the processor and gain nothing. When the
//! hierarchy does not fit into the card's memory, or anything else fails, the cycle
//! stays on the processor and the reason is reported.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

use rayon::prelude::*;
use wgpu::util::DeviceExt;

pub use plxs_gpu::{Detection, detect};
use plxs_gpu::{Double, Gpu, Options, block_on, wgpu};

use crate::amg::GpuMode;
use crate::bsr::{Bsr, Scalar};

/// Automatic mode: systems of at least this many unknowns go to the card.
pub const GPU_FROM: usize = 100_000;
/// Block rows per slice of the sliced storage (`SLICE` in the shaders).
const SLICE: usize = 32;
/// Block rows per window sorted by length.
const WINDOW: usize = 256;
/// The coarsest level is applied as a dense inverse up to this size.
const DENSE_COARSE_MAX: usize = 4096;
const PRODUCT_GROUP: usize = 64;
const VECTOR_GROUP: usize = 256;

/// The device and its pipelines.
struct Context {
    gpu: Gpu,
    product_module: wgpu::ShaderModule,
    product_layout: wgpu::BindGroupLayout,
    product_pipeline_layout: wgpu::PipelineLayout,
    products: Mutex<HashMap<(usize, usize), Arc<wgpu::ComputePipeline>>>,
    vector_layout: wgpu::BindGroupLayout,
    vector: Arc<wgpu::ComputePipeline>,
    /// The pipelines of the conjugate gradients, built at the first use when the card
    /// computes in double precision.
    cg: OnceLock<Result<CgPipelines, String>>,
}

impl std::ops::Deref for Context {
    type Target = Gpu;

    fn deref(&self) -> &Gpu {
        &self.gpu
    }
}

/// The pipelines of the conjugate gradients on the card (auto layouts).
struct CgPipelines {
    /// The double precision product per block shape.
    products: Mutex<HashMap<(usize, usize), Arc<wgpu::ComputePipeline>>>,
    product_module: wgpu::ShaderModule,
    dot: Arc<wgpu::ComputePipeline>,
    scalar: Arc<wgpu::ComputePipeline>,
    update: Arc<wgpu::ComputePipeline>,
    convert: Arc<wgpu::ComputePipeline>,
}

/// The device for discrete cards only (automatic mode) and for any adapter (`On`),
/// opened once per process; `Err` tells why there is none.
static DISCRETE: OnceLock<Result<Context, String>> = OnceLock::new();
static ANY: OnceLock<Result<Context, String>> = OnceLock::new();

fn context(any: bool) -> Result<&'static Context, String> {
    let cell = if any { &ANY } else { &DISCRETE };
    cell.get_or_init(|| open(any))
        .as_ref()
        .map_err(Clone::clone)
}

fn open(any: bool) -> Result<Context, String> {
    let gpu = Gpu::open(Options {
        label: "plxs multigrid",
        any_adapter: any,
        double: Double::IfAvailable,
        timestamps: false,
        memory_share: true,
    })?;
    let device = &gpu.device;
    let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
    let storage = |binding: u32, read_only: bool| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    };
    let uniform = wgpu::BindGroupLayoutEntry {
        binding: 0,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    };
    let mut entries = vec![uniform];
    entries.extend((1..=6).map(|b| storage(b, true)));
    entries.push(storage(7, false));
    let product_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("product"),
        entries: &entries,
    });
    let vector_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("vector"),
        entries: &[
            uniform,
            storage(1, true),
            storage(2, true),
            storage(3, false),
        ],
    });
    let product_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("product"),
        source: wgpu::ShaderSource::Wgsl(include_str!("gpu_product.wgsl").into()),
    });
    let vector_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("vector"),
        source: wgpu::ShaderSource::Wgsl(include_str!("gpu_vector.wgsl").into()),
    });
    let product_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("product"),
        bind_group_layouts: &[Some(&product_layout)],
        immediate_size: 0,
    });
    let vector_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("vector"),
        bind_group_layouts: &[Some(&vector_layout)],
        immediate_size: 0,
    });
    let vector = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("vector"),
        layout: Some(&vector_pipeline_layout),
        module: &vector_module,
        entry_point: Some("vector"),
        compilation_options: Default::default(),
        cache: None,
    });
    if let Some(e) = block_on(scope.pop()) {
        return Err(format!("{}: {e}", gpu.name));
    }
    Ok(Context {
        product_module,
        product_layout,
        product_pipeline_layout,
        products: Mutex::default(),
        vector_layout,
        vector: Arc::new(vector),
        cg: OnceLock::new(),
        gpu,
    })
}

impl Context {
    /// The pipelines of the conjugate gradients, when the card has double precision.
    fn cg(&self) -> Result<&CgPipelines, String> {
        if !self.f64 {
            return Err("no double precision on the card".to_owned());
        }
        self.cg
            .get_or_init(|| {
                let device = &self.device;
                let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
                let module = |label: &str, source: &'static str| {
                    device.create_shader_module(wgpu::ShaderModuleDescriptor {
                        label: Some(label),
                        source: wgpu::ShaderSource::Wgsl(source.into()),
                    })
                };
                let pipeline = |module: &wgpu::ShaderModule, entry: &str| {
                    Arc::new(
                        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                            label: Some(entry),
                            layout: None,
                            module,
                            entry_point: Some(entry),
                            compilation_options: Default::default(),
                            cache: None,
                        }),
                    )
                };
                let pipelines = CgPipelines {
                    products: Mutex::default(),
                    product_module: module("cg product", include_str!("gpu_cg_product.wgsl")),
                    dot: pipeline(&module("dot", include_str!("gpu_cg_dot.wgsl")), "dot"),
                    scalar: pipeline(
                        &module("scalar", include_str!("gpu_cg_scalar.wgsl")),
                        "scalar",
                    ),
                    update: pipeline(
                        &module("update", include_str!("gpu_cg_update.wgsl")),
                        "update",
                    ),
                    convert: pipeline(
                        &module("convert", include_str!("gpu_cg_convert.wgsl")),
                        "convert",
                    ),
                };
                match block_on(scope.pop()) {
                    Some(e) => Err(e.to_string()),
                    None => Ok(pipelines),
                }
            })
            .as_ref()
            .map_err(Clone::clone)
    }

    /// The product pipeline for blocks of `br × bc`.
    fn product(&self, br: usize, bc: usize) -> Arc<wgpu::ComputePipeline> {
        let mut map = self.products.lock().unwrap_or_else(|e| e.into_inner());
        map.entry((br, bc))
            .or_insert_with(|| {
                let constants = [("BR", br as f64), ("BC", bc as f64)];
                Arc::new(
                    self.device
                        .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                            label: Some("product"),
                            layout: Some(&self.product_pipeline_layout),
                            module: &self.product_module,
                            entry_point: Some("product"),
                            compilation_options: wgpu::PipelineCompilationOptions {
                                constants: &constants,
                                zero_initialize_workgroup_memory: false,
                            },
                            cache: None,
                        }),
                )
            })
            .clone()
    }
}

/// A matrix in sliced block storage.
struct Sliced {
    br: usize,
    bc: usize,
    brows: usize,
    start: Vec<u32>,
    rows: Vec<u32>,
    col: Vec<u32>,
    val: Vec<f32>,
    /// For every stored block its block in the source matrix (`u32::MAX`: padding).
    source: Vec<u32>,
}

impl Sliced {
    fn new<T: Scalar>(a: &Bsr<T>) -> Sliced {
        let size = a.br * a.bc;
        let length = |i: usize| a.ptr[i + 1] - a.ptr[i];
        let mut order: Vec<u32> = (0..a.brows as u32).collect();
        for window in order.chunks_mut(WINDOW) {
            window.sort_by_key(|&i| std::cmp::Reverse(length(i as usize)));
        }
        let slices = a.brows.div_ceil(SLICE);
        let mut start = Vec::with_capacity(slices + 1);
        start.push(0u32);
        let mut total = 0usize;
        for s in 0..slices {
            let members = &order[s * SLICE..((s + 1) * SLICE).min(a.brows)];
            let width = members
                .iter()
                .map(|&i| length(i as usize))
                .max()
                .unwrap_or(0);
            total += width * SLICE;
            start.push(total as u32);
        }
        let mut col = vec![0u32; total];
        let mut val = vec![0.0f32; total * size];
        let mut source = vec![u32::MAX; total];
        // Every slice owns a contiguous piece of the storage: filled in parallel.
        let mut pieces = Vec::with_capacity(slices);
        let (mut c, mut v, mut k) = (&mut col[..], &mut val[..], &mut source[..]);
        for s in 0..slices {
            let blocks = (start[s + 1] - start[s]) as usize;
            let (c_s, c_rest) = std::mem::take(&mut c).split_at_mut(blocks);
            let (v_s, v_rest) = std::mem::take(&mut v).split_at_mut(blocks * size);
            let (k_s, k_rest) = std::mem::take(&mut k).split_at_mut(blocks);
            (c, v, k) = (c_rest, v_rest, k_rest);
            pieces.push((s, c_s, v_s, k_s));
        }
        pieces.into_par_iter().for_each(|(s, col, val, source)| {
            for (b, &i) in order[s * SLICE..((s + 1) * SLICE).min(a.brows)]
                .iter()
                .enumerate()
            {
                let i = i as usize;
                for (j, k) in (a.ptr[i]..a.ptr[i + 1]).enumerate() {
                    let at = j * SLICE + b;
                    col[at] = a.col[k];
                    source[at] = k as u32;
                    for (v, &w) in val[at * size..(at + 1) * size]
                        .iter_mut()
                        .zip(&a.val[k * size..(k + 1) * size])
                    {
                        *v = w.to_f64() as f32;
                    }
                }
            }
        });
        Sliced {
            br: a.br,
            bc: a.bc,
            brows: a.brows,
            start,
            rows: order,
            col,
            val,
            source,
        }
    }

    /// The values of `a` (the matrix this was built from, new values) in sliced order.
    fn values_of<T: Scalar>(&self, a: &Bsr<T>) -> Vec<T> {
        let size = self.br * self.bc;
        let mut val = vec![T::default(); self.val.len()];
        for (at, &k) in self.source.iter().enumerate() {
            if k != u32::MAX {
                let k = k as usize;
                val[at * size..(at + 1) * size].copy_from_slice(&a.val[k * size..(k + 1) * size]);
            }
        }
        val
    }
}

/// A sliced matrix on the card.
struct GpuMatrix {
    br: usize,
    bc: usize,
    brows: usize,
    start: wgpu::Buffer,
    rows: wgpu::Buffer,
    col: wgpu::Buffer,
    val: wgpu::Buffer,
}

/// Which Chebyshev coefficient a vector operation carries, to update it when the
/// eigenvalue estimate of its level changes.
#[derive(Clone, Copy)]
enum Coefficient {
    None,
    /// `1/θ` of the first step.
    First,
    /// `(c1, c2)` of step `j` (from 1).
    Step(usize),
}

struct Op {
    pipeline: Arc<wgpu::ComputePipeline>,
    bind_group: wgpu::BindGroup,
    groups: (u32, u32),
    params: wgpu::Buffer,
    /// Parameters as uploaded and what to update on refresh.
    header: [u32; 4],
    level: usize,
    coefficient: Coefficient,
}

/// The first coefficient `1/θ` and the pairs `(c1, c2)` of the Chebyshev smoother of
/// `amg::smooth`, same recurrence.
fn chebyshev(lambda_max: f64, degree: usize) -> (f32, Vec<(f32, f32)>) {
    let lambda_min = lambda_max / 30.0;
    let theta = 0.5 * (lambda_max + lambda_min);
    let delta = 0.5 * (lambda_max - lambda_min);
    let sigma = theta / delta;
    let mut rho_old = 1.0 / sigma;
    let mut steps = Vec::new();
    for _ in 1..degree {
        let rho_new = 1.0 / (2.0 * sigma - rho_old);
        steps.push(((rho_new * rho_old) as f32, (2.0 * rho_new / delta) as f32));
        rho_old = rho_new;
    }
    ((1.0 / theta) as f32, steps)
}

fn params_bytes(header: [u32; 4], c1: f32, c2: f32) -> [u8; 32] {
    let mut out = [0u8; 32];
    for (i, v) in header.iter().enumerate() {
        out[4 * i..4 * i + 4].copy_from_slice(&v.to_le_bytes());
    }
    out[16..20].copy_from_slice(&c1.to_le_bytes());
    out[20..24].copy_from_slice(&c2.to_le_bytes());
    out
}

/// One level of the hierarchy as the cycle needs it (borrowed from [`crate::amg`]).
pub(crate) struct LevelView<'a> {
    pub a: &'a Bsr<f32>,
    pub inv_diag: &'a [f32],
    pub lambda_max: f64,
    pub p: Option<&'a Bsr<f32>>,
    pub pt: Option<&'a Bsr<f32>>,
}

/// The V-cycle on the card.
pub(crate) struct GpuCycle {
    ctx: &'static Context,
    n: usize,
    b0: wgpu::Buffer,
    x0: wgpu::Buffer,
    staging: wgpu::Buffer,
    ops: Vec<Op>,
    degree: usize,
    lambdas: Vec<f64>,
    /// The finest operator: its storage (for new values) and buffers.
    fine: Sliced,
    fine_matrix: GpuMatrix,
    fine_diag: wgpu::Buffer,
    /// The conjugate gradients on the card, once [`GpuCycle::attach`] gave the operator.
    cg: Option<Cg>,
    lock: Mutex<()>,
    failed: AtomicBool,
}

/// Why the cycle runs where it runs, for the monitor.
pub(crate) enum Placement {
    Gpu(Box<GpuCycle>),
    /// The processor; `Some` with the reason when the card was considered.
    Cpu(Option<String>),
}

impl GpuCycle {
    /// Uploads the hierarchy when the mode and the system ask for the card.
    /// `coarsest` is the coarsest operator in double precision.
    pub(crate) fn place(
        levels: &[LevelView<'_>],
        coarsest: &Bsr,
        degree: usize,
        mode: GpuMode,
    ) -> Placement {
        let n = levels[0].a.rows();
        match mode {
            GpuMode::Off => return Placement::Cpu(None),
            GpuMode::Automatic if n < GPU_FROM => return Placement::Cpu(None),
            _ => {}
        }
        if levels.len() < 2 {
            return Placement::Cpu(None);
        }
        let trace = std::env::var("PLXS_AMG_TRACE").is_ok();
        let clock = std::time::Instant::now();
        let ctx = match context(mode == GpuMode::On) {
            Ok(ctx) => ctx,
            Err(reason) => return Placement::Cpu(Some(reason)),
        };
        if trace {
            eprintln!("gpu: device {:.3} s", clock.elapsed().as_secs_f64());
        }
        // A driver or wgpu failure must not take the solver down: the processor serves.
        let uploaded = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            Self::upload(ctx, levels, coarsest, degree)
        }))
        .unwrap_or_else(|_| Err("the upload panicked".to_owned()));
        match uploaded {
            Ok(cycle) => Placement::Gpu(Box::new(cycle)),
            Err(reason) => Placement::Cpu(Some(format!("{}: {reason}", ctx.name))),
        }
    }

    pub(crate) fn device_name(&self) -> &str {
        &self.ctx.name
    }

    fn upload(
        ctx: &'static Context,
        levels: &[LevelView<'_>],
        coarsest: &Bsr,
        degree: usize,
    ) -> Result<GpuCycle, String> {
        if coarsest.rows() > DENSE_COARSE_MAX {
            return Err(format!(
                "the coarsest level has {} unknowns, more than {DENSE_COARSE_MAX}",
                coarsest.rows()
            ));
        }
        let inverse = dense_inverse(coarsest)?;
        let device = &ctx.device;
        let oom = device.push_error_scope(wgpu::ErrorFilter::OutOfMemory);
        let validation = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let limit = ctx
            .limits
            .max_storage_buffer_binding_size
            .min(ctx.limits.max_buffer_size);
        let storage = |label: &str, bytes: &[u8]| -> Result<wgpu::Buffer, String> {
            if bytes.len() as u64 > limit {
                return Err(format!(
                    "a buffer of {} MB exceeds the card's limit of {} MB",
                    bytes.len() >> 20,
                    limit >> 20
                ));
            }
            // Bindings must not be empty.
            let size = (bytes.len() as u64).max(4);
            let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            });
            write(ctx, &buffer, bytes);
            Ok(buffer)
        };
        // Zero-initialized by wgpu.
        let vector = |label: &str, n: usize| -> Result<wgpu::Buffer, String> {
            Ok(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: 4 * n.max(1) as u64,
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            }))
        };
        let matrix = |s: &Sliced| -> Result<GpuMatrix, String> {
            Ok(GpuMatrix {
                br: s.br,
                bc: s.bc,
                brows: s.brows,
                start: storage("start", bytemuck::cast_slice(&s.start))?,
                rows: storage("rows", bytemuck::cast_slice(&s.rows))?,
                col: storage("col", bytemuck::cast_slice(&s.col))?,
                val: storage("val", bytemuck::cast_slice(&s.val))?,
            })
        };
        struct LevelBuffers {
            n: usize,
            a: GpuMatrix,
            diag: wgpu::Buffer,
            p: Option<GpuMatrix>,
            pt: Option<GpuMatrix>,
            x: wgpu::Buffer,
            b: wgpu::Buffer,
            r: wgpu::Buffer,
            d: wgpu::Buffer,
            t: wgpu::Buffer,
        }
        let last = levels.len() - 1;
        let fine = Sliced::new(levels[0].a);
        let mut buffers = Vec::with_capacity(levels.len());
        for (l, level) in levels.iter().enumerate() {
            let n = level.a.rows();
            let a = if l == 0 {
                matrix(&fine)?
            } else if l == last {
                matrix(&Sliced::new(&inverse))?
            } else {
                matrix(&Sliced::new(level.a))?
            };
            buffers.push(LevelBuffers {
                n,
                a,
                diag: storage("diag", bytemuck::cast_slice(level.inv_diag))?,
                p: level.p.map(|p| matrix(&Sliced::new(p))).transpose()?,
                pt: level.pt.map(|p| matrix(&Sliced::new(p))).transpose()?,
                x: vector("x", n)?,
                b: vector("b", n)?,
                r: vector("r", if l == last { 0 } else { n })?,
                d: vector("d", if l == last { 0 } else { n })?,
                t: vector("t", if l == last { 0 } else { n })?,
            });
        }
        let n = buffers[0].n;
        let staging = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("staging"),
            size: 4 * n as u64,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let lambdas: Vec<f64> = levels.iter().map(|l| l.lambda_max).collect();
        let max_groups = ctx.limits.max_compute_workgroups_per_dimension.max(1) as usize;
        let fold = |groups: usize| -> (u32, u32) {
            let gx = groups.clamp(1, max_groups);
            (gx as u32, groups.div_ceil(gx).max(1) as u32)
        };
        let mut ops = Vec::new();
        let uniform = |bytes: &[u8; 32]| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("params"),
                contents: bytes,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            })
        };
        let product = |ops: &mut Vec<Op>,
                       m: &GpuMatrix,
                       x: &wgpu::Buffer,
                       y0: Option<&wgpu::Buffer>,
                       y: &wgpu::Buffer,
                       level: usize| {
            let threads = m.brows.div_ceil(SLICE) * SLICE * m.br;
            let groups = fold(threads.div_ceil(PRODUCT_GROUP));
            let header = [m.brows as u32, y0.is_some() as u32, groups.0, 0];
            let params = uniform(&params_bytes(header, 0.0, 0.0));
            let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("product"),
                layout: &ctx.product_layout,
                entries: &[
                    entry(0, &params),
                    entry(1, &m.start),
                    entry(2, &m.rows),
                    entry(3, &m.col),
                    entry(4, &m.val),
                    entry(5, x),
                    entry(6, y0.unwrap_or(x)),
                    entry(7, y),
                ],
            });
            ops.push(Op {
                pipeline: ctx.product(m.br, m.bc),
                bind_group,
                groups,
                params,
                header,
                level,
                coefficient: Coefficient::None,
            });
        };
        let vector_op = |ops: &mut Vec<Op>,
                         op: u32,
                         n: usize,
                         a: &wgpu::Buffer,
                         diag: Option<&wgpu::Buffer>,
                         v: &wgpu::Buffer,
                         level: usize,
                         coefficient: Coefficient| {
            let groups = fold(n.div_ceil(VECTOR_GROUP));
            // The block size of the inverse diagonal blocks of the level.
            let br = levels[level].a.br as u32;
            let header = [n as u32, op, groups.0, br];
            let (c1, c2) = coefficients(&lambdas, degree, level, coefficient);
            let params = uniform(&params_bytes(header, c1, c2));
            let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("vector"),
                layout: &ctx.vector_layout,
                entries: &[
                    entry(0, &params),
                    entry(1, a),
                    entry(2, diag.unwrap_or(a)),
                    entry(3, v),
                ],
            });
            ops.push(Op {
                pipeline: ctx.vector.clone(),
                bind_group,
                groups,
                params,
                header,
                level,
                coefficient,
            });
        };
        const COPY: u32 = 0;
        const ADD: u32 = 1;
        const SUB: u32 = 2;
        const SCALED: u32 = 3;
        const UPDATE: u32 = 4;
        // The cycle of `Amg::cycle`, unrolled: down the levels, the coarsest, back up.
        let smooth = |ops: &mut Vec<Op>, l: usize, w: &LevelBuffers, zero: bool| {
            if zero {
                vector_op(ops, COPY, w.n, &w.b, None, &w.r, l, Coefficient::None);
                vector_op(
                    ops,
                    SCALED,
                    w.n,
                    &w.r,
                    Some(&w.diag),
                    &w.d,
                    l,
                    Coefficient::First,
                );
                vector_op(ops, COPY, w.n, &w.d, None, &w.x, l, Coefficient::None);
            } else {
                product(ops, &w.a, &w.x, Some(&w.b), &w.r, l);
                vector_op(
                    ops,
                    SCALED,
                    w.n,
                    &w.r,
                    Some(&w.diag),
                    &w.d,
                    l,
                    Coefficient::First,
                );
                vector_op(ops, ADD, w.n, &w.d, None, &w.x, l, Coefficient::None);
            }
            for step in 1..degree {
                product(ops, &w.a, &w.d, None, &w.t, l);
                vector_op(ops, SUB, w.n, &w.t, None, &w.r, l, Coefficient::None);
                vector_op(
                    ops,
                    UPDATE,
                    w.n,
                    &w.r,
                    Some(&w.diag),
                    &w.d,
                    l,
                    Coefficient::Step(step),
                );
                vector_op(ops, ADD, w.n, &w.d, None, &w.x, l, Coefficient::None);
            }
        };
        for l in 0..last {
            let (w, next) = (&buffers[l], &buffers[l + 1]);
            smooth(&mut ops, l, w, true);
            product(&mut ops, &w.a, &w.x, Some(&w.b), &w.r, l);
            let pt =
                w.pt.as_ref()
                    .expect("a restriction below the coarsest level");
            product(&mut ops, pt, &w.r, None, &next.b, l);
        }
        let c = &buffers[last];
        product(&mut ops, &c.a, &c.b, None, &c.x, last);
        for l in (0..last).rev() {
            let (w, next) = (&buffers[l], &buffers[l + 1]);
            let p =
                w.p.as_ref()
                    .expect("a prolongation below the coarsest level");
            product(&mut ops, p, &next.x, None, &w.t, l);
            vector_op(&mut ops, ADD, w.n, &w.t, None, &w.x, l, Coefficient::None);
            smooth(&mut ops, l, w, false);
        }
        // Scopes pop innermost first. A buffer the card had no memory for makes every
        // later use of it a validation error: the shortage is the reason to report.
        let invalid = block_on(validation.pop());
        if let Some(e) = block_on(oom.pop()) {
            return Err(format!("not enough free memory on the card ({e})"));
        }
        if let Some(e) = invalid {
            return Err(e.to_string());
        }
        if let Some(e) = ctx.take_error() {
            return Err(e);
        }
        let level0 = buffers.swap_remove(0);
        Ok(GpuCycle {
            ctx,
            n,
            b0: level0.b,
            x0: level0.x,
            staging,
            ops,
            degree,
            lambdas,
            fine,
            fine_matrix: level0.a,
            fine_diag: level0.diag,
            cg: None,
            lock: Mutex::new(()),
            failed: AtomicBool::new(false),
        })
    }

    /// `z ≈ A⁻¹ r` by one V-cycle on the card.
    pub(crate) fn apply(&self, r: &[f64], z: &mut [f64]) -> Result<(), String> {
        let _guard = self.lock.lock().unwrap_or_else(|e| e.into_inner());
        if self.failed.load(Ordering::Relaxed) {
            return Err("failed before".to_owned());
        }
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.run(r, z)))
            .unwrap_or_else(|_| Err("the cycle panicked".to_owned()));
        if result.is_err() {
            self.failed.store(true, Ordering::Relaxed);
        }
        result
    }

    fn run(&self, r: &[f64], z: &mut [f64]) -> Result<(), String> {
        debug_assert_eq!(r.len(), self.n);
        let b: Vec<f32> = r
            .par_iter()
            .with_min_len(1 << 14)
            .map(|&v| v as f32)
            .collect();
        let (device, queue) = (&self.ctx.device, &self.ctx.queue);
        queue.write_buffer(&self.b0, 0, bytemuck::cast_slice(&b));
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("v-cycle"),
        });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("v-cycle"),
                timestamp_writes: None,
            });
            for op in &self.ops {
                pass.set_pipeline(&op.pipeline);
                pass.set_bind_group(0, &op.bind_group, &[]);
                pass.dispatch_workgroups(op.groups.0, op.groups.1, 1);
            }
        }
        let bytes = 4 * self.n as u64;
        encoder.copy_buffer_to_buffer(&self.x0, 0, &self.staging, 0, bytes);
        queue.submit([encoder.finish()]);
        let (tx, rx) = std::sync::mpsc::channel();
        self.staging
            .map_async(wgpu::MapMode::Read, .., move |result| {
                let _ = tx.send(result);
            });
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|e| e.to_string())?;
        rx.recv()
            .map_err(|e| e.to_string())?
            .map_err(|e| e.to_string())?;
        {
            let view = self
                .staging
                .get_mapped_range(..)
                .map_err(|e| e.to_string())?;
            let x: &[f32] = bytemuck::cast_slice(&view);
            z.par_iter_mut()
                .zip(x.par_iter())
                .with_min_len(1 << 14)
                .for_each(|(z, &x)| *z = x as f64);
        }
        self.staging.unmap();
        if let Some(e) = self.ctx.take_error() {
            return Err(e);
        }
        Ok(())
    }

    /// New values of the finest operator (same pattern), its inverse diagonal and
    /// eigenvalue estimate; see `Amg::refresh`.
    pub(crate) fn refresh(&mut self, fine: &Bsr<f32>, inv_diag: &[f32], lambda_max: f64) {
        let queue = &self.ctx.queue;
        write(
            self.ctx,
            &self.fine_matrix.val,
            bytemuck::cast_slice(&self.fine.values_of(fine)),
        );
        queue.write_buffer(&self.fine_diag, 0, bytemuck::cast_slice(inv_diag));
        self.lambdas[0] = lambda_max;
        for op in self.ops.iter().filter(|op| op.level == 0) {
            if matches!(op.coefficient, Coefficient::None) {
                continue;
            }
            let (c1, c2) = coefficients(&self.lambdas, self.degree, 0, op.coefficient);
            queue.write_buffer(&op.params, 0, &params_bytes(op.header, c1, c2));
        }
    }
}

/// How the conjugate gradients on the card ended.
#[derive(Debug)]
pub(crate) enum CgOutcome {
    Converged {
        iterations: usize,
        residual: f64,
    },
    /// `p·q <= 0`: the matrix is not positive definite.
    Indefinite,
    NotConverged {
        residual: f64,
        /// The iterations done: the limit, or fewer when `stalls` gave up.
        iterations: usize,
    },
}

/// Iterations per submission; the state comes back after each.
const BATCH: usize = 4;
/// Scalars of the iteration (see gpu_cg_scalar.wgsl).
const SCALARS: usize = 16;
const DOT_GROUPS: usize = 256;

/// The conjugate gradients on the card: the double precision copy of the finest
/// operator, the vectors, and the recorded dispatches.
struct Cg {
    val: wgpu::Buffer,
    b: wgpu::Buffer,
    x: wgpu::Buffer,
    scalars: wgpu::Buffer,
    scalars_staging: wgpu::Buffer,
    x_staging: wgpu::Buffer,
    start: Vec<Op>,
    iteration: Vec<Op>,
}

impl GpuCycle {
    /// Gives the cycle the operator in double precision (the matrix the hierarchy was
    /// built on, or new values of it), so that the conjugate gradients run on the card
    /// too. Without double precision on the card nothing happens and the conjugate
    /// gradients stay on the processor.
    pub(crate) fn attach(&mut self, a: &Bsr) -> Result<(), String> {
        if self.failed.load(Ordering::Relaxed)
            || a.blocks() != self.fine.source.iter().filter(|&&k| k != u32::MAX).count()
        {
            return Err("the operator does not match the hierarchy".to_owned());
        }
        let values = self.fine.values_of(a);
        if let Some(cg) = &self.cg {
            write(self.ctx, &cg.val, bytemuck::cast_slice(&values));
            return match self.ctx.take_error() {
                Some(e) => {
                    self.cg = None;
                    Err(e)
                }
                None => Ok(()),
            };
        }
        let built =
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.build_cg(&values)))
                .unwrap_or_else(|_| Err("building the conjugate gradients panicked".to_owned()));
        self.cg = Some(built?);
        Ok(())
    }

    /// Whether the conjugate gradients run on the card.
    pub(crate) fn solves(&self) -> bool {
        self.cg.is_some() && !self.failed.load(Ordering::Relaxed)
    }

    fn build_cg(&self, values: &[f64]) -> Result<Cg, String> {
        let ctx = self.ctx;
        let pipelines = ctx.cg()?;
        let device = &ctx.device;
        let oom = device.push_error_scope(wgpu::ErrorFilter::OutOfMemory);
        let validation = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let n = self.n;
        let usage = wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC;
        let buffer = |label: &str, bytes: u64| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: bytes.max(8),
                usage,
                mapped_at_creation: false,
            })
        };
        let limit = ctx
            .limits
            .max_storage_buffer_binding_size
            .min(ctx.limits.max_buffer_size);
        if 8 * values.len() as u64 > limit {
            return Err(format!(
                "the operator in double precision ({} MB) exceeds the card's limit of {} MB",
                (8 * values.len()) >> 20,
                limit >> 20
            ));
        }
        let val = buffer("val64", 8 * values.len() as u64);
        write(ctx, &val, bytemuck::cast_slice(values));
        let vector = |label: &str| buffer(label, 8 * n as u64);
        let (b, x, r, z, p, q) = (
            vector("b"),
            vector("x"),
            vector("r"),
            vector("z"),
            vector("p"),
            vector("q"),
        );
        let partial = buffer("partial", (8 * 5 * DOT_GROUPS) as u64);
        let scalars = buffer("scalars", (8 * SCALARS) as u64);
        let staging = |label: &str, bytes: u64| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: bytes.max(8),
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        let scalars_staging = staging("scalars staging", (8 * SCALARS) as u64);
        let x_staging = staging("x staging", 8 * n as u64);
        let max_groups = ctx.limits.max_compute_workgroups_per_dimension.max(1) as usize;
        let fold = |groups: usize| -> (u32, u32) {
            let gx = groups.clamp(1, max_groups);
            (gx as u32, groups.div_ceil(gx).max(1) as u32)
        };
        let op = |pipeline: &Arc<wgpu::ComputePipeline>,
                  header: [u32; 4],
                  groups: (u32, u32),
                  buffers: &[&wgpu::Buffer]| {
            let params = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("params"),
                contents: bytemuck::cast_slice(&header),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            });
            let mut entries = vec![entry(0, &params)];
            entries.extend(
                buffers
                    .iter()
                    .enumerate()
                    .map(|(i, b)| entry(i as u32 + 1, b)),
            );
            let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: None,
                layout: &pipeline.get_bind_group_layout(0),
                entries: &entries,
            });
            Op {
                pipeline: pipeline.clone(),
                bind_group,
                groups,
                params,
                header,
                level: usize::MAX,
                coefficient: Coefficient::None,
            }
        };
        let m = &self.fine_matrix;
        let product_pipeline = {
            let mut map = pipelines.products.lock().unwrap_or_else(|e| e.into_inner());
            map.entry((m.br, m.bc))
                .or_insert_with(|| {
                    let constants = [("BR", m.br as f64), ("BC", m.bc as f64)];
                    Arc::new(
                        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                            label: Some("cg product"),
                            layout: None,
                            module: &pipelines.product_module,
                            entry_point: Some("product"),
                            compilation_options: wgpu::PipelineCompilationOptions {
                                constants: &constants,
                                zero_initialize_workgroup_memory: false,
                            },
                            cache: None,
                        }),
                    )
                })
                .clone()
        };
        let product_groups = fold((m.brows.div_ceil(SLICE) * SLICE * m.br).div_ceil(PRODUCT_GROUP));
        let vector_groups = fold(n.div_ceil(VECTOR_GROUP));
        let nn = n as u32;
        let product = |x: &wgpu::Buffer, y: &wgpu::Buffer| {
            op(
                &product_pipeline,
                [m.brows as u32, 0, product_groups.0, 0],
                product_groups,
                &[&m.start, &m.rows, &m.col, &val, x, y],
            )
        };
        let dot = |a: &wgpu::Buffer, b: &wgpu::Buffer, slot: u32| {
            op(
                &pipelines.dot,
                [nn, slot, 0, 0],
                (DOT_GROUPS as u32, 1),
                &[a, b, &partial],
            )
        };
        let scalar = |phase: u32| {
            op(
                &pipelines.scalar,
                [phase, 0, 0, 0],
                (1, 1),
                &[&partial, &scalars],
            )
        };
        let update = |code: u32, q: &wgpu::Buffer| {
            op(
                &pipelines.update,
                [nn, code, vector_groups.0, 0],
                vector_groups,
                &[&scalars, q, &z, &x, &r, &p],
            )
        };
        let to_cycle = op(
            &pipelines.convert,
            [nn, 0, vector_groups.0, 0],
            vector_groups,
            &[&r, &self.b0],
        );
        let from_cycle = || {
            op(
                &pipelines.convert,
                [nn, 1, vector_groups.0, 0],
                vector_groups,
                &[&z, &self.x0],
            )
        };
        let cycle = || self.ops.iter().map(Op::clone_handles);
        // r = b, x = 0, b·b; z = M r; rz; p = z.
        let mut start = vec![update(2, &b), dot(&b, &b, 4), to_cycle.clone_handles()];
        start.extend(cycle());
        start.extend([from_cycle(), dot(&r, &z, 2), scalar(0), update(3, &q)]);
        // q = A p; alpha; x, r; r·r and the check; z = M r; beta; p.
        let mut iteration = vec![
            product(&p, &q),
            dot(&p, &q, 0),
            scalar(1),
            update(0, &q),
            dot(&r, &r, 1),
            scalar(2),
            to_cycle,
        ];
        iteration.extend(cycle());
        iteration.extend([
            from_cycle(),
            dot(&r, &z, 2),
            dot(&z, &q, 3),
            scalar(3),
            update(1, &q),
        ]);
        // Scopes pop innermost first. A buffer the card had no memory for makes every
        // later use of it a validation error: the shortage is the reason to report.
        let invalid = block_on(validation.pop());
        if let Some(e) = block_on(oom.pop()) {
            return Err(format!("not enough free memory on the card ({e})"));
        }
        if let Some(e) = invalid {
            return Err(e.to_string());
        }
        if let Some(e) = ctx.take_error() {
            return Err(e);
        }
        Ok(Cg {
            val,
            b,
            x,
            scalars,
            scalars_staging,
            x_staging,
            start,
            iteration,
        })
    }

    /// Solves `A x = b` by conjugate gradients on the card (the flexible form of
    /// `PcgSolver::solve_counted`, same stopping rule `|r| <= tolerance |b|`). `Err`: the
    /// card failed, the processor has to solve.
    pub(crate) fn solve(
        &self,
        b: &[f64],
        x: &mut [f64],
        tolerance: f64,
        max_iterations: usize,
        cancelled: &dyn Fn() -> bool,
        stalls: &mut dyn FnMut(usize, f64) -> bool,
    ) -> Result<std::result::Result<CgOutcome, plxs_core::Error>, String> {
        let _guard = self.lock.lock().unwrap_or_else(|e| e.into_inner());
        let Some(cg) = self
            .cg
            .as_ref()
            .filter(|_| !self.failed.load(Ordering::Relaxed))
        else {
            return Err("the conjugate gradients are not on the card".to_owned());
        };
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            self.iterate(cg, b, x, tolerance, max_iterations, cancelled, stalls)
        }))
        .unwrap_or_else(|_| Err("the conjugate gradients panicked".to_owned()));
        if result.is_err() {
            self.failed.store(true, Ordering::Relaxed);
        }
        result
    }

    fn submit(&self, ops: &[Op], repeat: usize, read: Option<(&wgpu::Buffer, &wgpu::Buffer, u64)>) {
        let mut encoder = self
            .ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("cg") });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("cg"),
                timestamp_writes: None,
            });
            for _ in 0..repeat {
                for op in ops {
                    pass.set_pipeline(&op.pipeline);
                    pass.set_bind_group(0, &op.bind_group, &[]);
                    pass.dispatch_workgroups(op.groups.0, op.groups.1, 1);
                }
            }
        }
        if let Some((from, to, bytes)) = read {
            encoder.copy_buffer_to_buffer(from, 0, to, 0, bytes);
        }
        self.ctx.queue.submit([encoder.finish()]);
    }

    fn read<T: bytemuck::Pod>(&self, staging: &wgpu::Buffer, out: &mut [T]) -> Result<(), String> {
        let (tx, rx) = std::sync::mpsc::channel();
        staging.map_async(wgpu::MapMode::Read, .., move |result| {
            let _ = tx.send(result);
        });
        self.ctx
            .device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|e| e.to_string())?;
        rx.recv()
            .map_err(|e| e.to_string())?
            .map_err(|e| e.to_string())?;
        {
            let view = staging.get_mapped_range(..).map_err(|e| e.to_string())?;
            let data: &[T] = bytemuck::cast_slice(&view);
            out.copy_from_slice(&data[..out.len()]);
        }
        staging.unmap();
        match self.ctx.take_error() {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn iterate(
        &self,
        cg: &Cg,
        b: &[f64],
        x: &mut [f64],
        tolerance: f64,
        max_iterations: usize,
        cancelled: &dyn Fn() -> bool,
        stalls: &mut dyn FnMut(usize, f64) -> bool,
    ) -> Result<std::result::Result<CgOutcome, plxs_core::Error>, String> {
        let queue = &self.ctx.queue;
        queue.write_buffer(&cg.b, 0, bytemuck::cast_slice(b));
        let mut init = [0.0f64; SCALARS];
        init[7] = tolerance * tolerance;
        init[8] = max_iterations as f64;
        queue.write_buffer(&cg.scalars, 0, bytemuck::cast_slice(&init));
        let bytes = (8 * SCALARS) as u64;
        self.submit(
            &cg.start,
            1,
            Some((&cg.scalars, &cg.scalars_staging, bytes)),
        );
        let mut state = [0.0f64; SCALARS];
        self.read(&cg.scalars_staging, &mut state)?;
        let relative = |state: &[f64; SCALARS]| {
            if state[3] > 0.0 {
                (state[4] / state[3]).sqrt()
            } else {
                0.0
            }
        };
        while state[5] == 0.0 {
            if cancelled() {
                return Ok(Err(plxs_core::Error::Cancelled));
            }
            self.submit(
                &cg.iteration,
                BATCH,
                Some((&cg.scalars, &cg.scalars_staging, bytes)),
            );
            self.read(&cg.scalars_staging, &mut state)?;
            if state[5] == 0.0 && stalls(state[6] as usize, relative(&state)) {
                return Ok(Ok(CgOutcome::NotConverged {
                    residual: relative(&state),
                    iterations: state[6] as usize,
                }));
            }
        }
        let residual = relative(&state);
        let outcome = match state[5] as u32 {
            1 => {
                let n = 8 * self.n as u64;
                let mut encoder = self
                    .ctx
                    .device
                    .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
                encoder.copy_buffer_to_buffer(&cg.x, 0, &cg.x_staging, 0, n);
                queue.submit([encoder.finish()]);
                self.read(&cg.x_staging, x)?;
                CgOutcome::Converged {
                    iterations: state[6] as usize,
                    residual,
                }
            }
            2 => CgOutcome::Indefinite,
            _ => CgOutcome::NotConverged {
                residual,
                iterations: state[6] as usize,
            },
        };
        Ok(Ok(outcome))
    }
}

impl Op {
    /// Another handle to the same dispatch (wgpu objects are reference counted).
    fn clone_handles(&self) -> Op {
        Op {
            pipeline: self.pipeline.clone(),
            bind_group: self.bind_group.clone(),
            groups: self.groups,
            params: self.params.clone(),
            header: self.header,
            level: self.level,
            coefficient: self.coefficient,
        }
    }
}

fn coefficients(lambdas: &[f64], degree: usize, level: usize, c: Coefficient) -> (f32, f32) {
    match c {
        Coefficient::None => (0.0, 0.0),
        Coefficient::First => (chebyshev(lambdas[level], degree).0, 0.0),
        Coefficient::Step(j) => chebyshev(lambdas[level], degree).1[j - 1],
    }
}

/// Writes `bytes` to the start of `buffer` in pieces (see [`Gpu::write`]).
fn write(ctx: &Context, buffer: &wgpu::Buffer, bytes: &[u8]) {
    ctx.gpu.write(buffer, 0, bytes);
}

fn entry(binding: u32, buffer: &wgpu::Buffer) -> wgpu::BindGroupEntry<'_> {
    wgpu::BindGroupEntry {
        binding,
        resource: buffer.as_entire_binding(),
    }
}

/// The inverse of the symmetric positive definite `a` as a dense matrix of `1 × 1`
/// blocks, from its dense `L D Lᵀ` factorization in double precision.
fn dense_inverse(a: &Bsr) -> Result<Bsr<f32>, String> {
    use faer::linalg::solvers::DenseSolveCore;
    let n = a.rows();
    let mut dense = faer::Mat::<f64>::zeros(n, n);
    for i in 0..n {
        a.for_row(i, |j, v| dense[(i, j as usize)] = v);
    }
    let inverse = dense
        .as_ref()
        .ldlt(faer::Side::Lower)
        .map_err(|e| format!("the coarsest system: {e:?}"))?
        .inverse();
    let mut val = Vec::with_capacity(n * n);
    for i in 0..n {
        for j in 0..n {
            val.push(inverse[(i, j)] as f32);
        }
    }
    Ok(Bsr {
        brows: n,
        bcols: n,
        br: 1,
        bc: 1,
        ptr: (0..=n).map(|i| i * n).collect(),
        col: (0..n).flat_map(|_| 0..n as u32).collect(),
        val,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::amg::{Amg, AmgOptions, Unknowns};

    /// A block Laplacian on a grid of `m³` nodes with `3 × 3` couplings (an SPD block
    /// per edge, the diagonal their sum plus a little), as the elasticity matrix of a
    /// solid looks to the multigrid.
    fn grid(m: usize) -> (Bsr, Unknowns) {
        let id = |i: usize, j: usize, k: usize| (i + m * (j + m * k)) as u32;
        let coupling = [2.0, 0.5, 0.1, 0.5, 1.5, 0.3, 0.1, 0.3, 1.0];
        let nodes = m * m * m;
        let a = Bsr::build(
            nodes,
            nodes,
            3,
            3,
            || (),
            |n, (), acc| {
                let (i, j, k) = (n % m, (n / m) % m, n / (m * m));
                let mut neighbours = Vec::new();
                if i > 0 {
                    neighbours.push(id(i - 1, j, k));
                }
                if i + 1 < m {
                    neighbours.push(id(i + 1, j, k));
                }
                if j > 0 {
                    neighbours.push(id(i, j - 1, k));
                }
                if j + 1 < m {
                    neighbours.push(id(i, j + 1, k));
                }
                if k > 0 {
                    neighbours.push(id(i, j, k - 1));
                }
                if k + 1 < m {
                    neighbours.push(id(i, j, k + 1));
                }
                let weight = 1.0 + (n % 5) as f64 * 0.1;
                for &c in &neighbours {
                    let block = acc.block(c);
                    for (b, v) in block.iter_mut().zip(coupling) {
                        *b -= weight * v;
                    }
                }
                let diagonal = acc.block(n as u32);
                for (b, v) in diagonal.iter_mut().zip(coupling) {
                    *b += (neighbours.len() as f64 + 0.01) * weight * v;
                }
            },
        );
        let coordinates: Vec<[f64; 3]> = (0..nodes)
            .map(|n| [(n % m) as f64, ((n / m) % m) as f64, (n / (m * m)) as f64])
            .collect();
        let free: Vec<u32> = (0..3 * nodes as u32).collect();
        (a, Unknowns::from_free(&free, 3, &coordinates))
    }

    #[test]
    fn the_cycle_on_the_card_matches_the_one_on_the_processor() {
        let (a, unknowns) = grid(24);
        let options = |gpu| AmgOptions {
            gpu,
            coarse_size: 300,
            ..AmgOptions::default()
        };
        let cpu = Amg::build(&a, Some(&unknowns), &[], options(GpuMode::Off)).unwrap();
        let mut gpu = Amg::build(&a, Some(&unknowns), &[], options(GpuMode::On)).unwrap();
        let placement = gpu.placement().unwrap_or_default();
        if !placement.contains("graphics card") || placement.contains("processor") {
            eprintln!("skipped, no graphics card: {placement}");
            return;
        }
        assert!(cpu.levels() >= 3, "{:?}", cpu.sizes());
        let n = a.rows();
        let r: Vec<f64> = (0..n).map(|i| ((i * 37) % 23) as f64 - 11.0).collect();
        let compare = |cpu: &Amg, gpu: &Amg| {
            let (mut z_cpu, mut z_gpu) = (vec![0.0; n], vec![0.0; n]);
            cpu.apply(&r, &mut z_cpu).unwrap();
            gpu.apply(&r, &mut z_gpu).unwrap();
            assert!(
                !gpu.placement().unwrap().contains("failed"),
                "{:?}",
                gpu.placement()
            );
            let scale = z_cpu.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            let worst = z_cpu
                .iter()
                .zip(&z_gpu)
                .fold(0.0_f64, |m, (c, g)| m.max((c - g).abs()));
            assert!(
                worst <= 1e-3 * scale,
                "difference {worst:e} at scale {scale:e}"
            );
        };
        compare(&cpu, &gpu);
        // New values of the same pattern: both follow.
        let mut b = a.clone();
        b.val.iter_mut().for_each(|v| *v *= 1.5);
        let mut cpu = cpu;
        cpu.refresh(&b);
        gpu.refresh(&b);
        compare(&cpu, &gpu);
    }

    #[test]
    fn conjugate_gradients_on_the_card_solve_the_system() {
        let (a, unknowns) = grid(16);
        let options = AmgOptions {
            gpu: GpuMode::On,
            coarse_size: 300,
            ..AmgOptions::default()
        };
        let mut amg = Amg::build(&a, Some(&unknowns), &[], options).unwrap();
        amg.attach_operator(&a);
        let Some(gpu) = amg.gpu_cycle() else {
            eprintln!("skipped: {:?}", amg.placement());
            return;
        };
        let n = a.rows();
        let b: Vec<f64> = (0..n).map(|i| ((i * 37) % 23) as f64 - 11.0).collect();
        let mut x = vec![0.0; n];
        let outcome = gpu
            .solve(&b, &mut x, 1e-12, 1000, &|| false, &mut |_, _| false)
            .unwrap()
            .unwrap();
        let CgOutcome::Converged {
            iterations,
            residual,
        } = outcome
        else {
            panic!("did not converge");
        };
        assert!(
            iterations > 1 && residual <= 1e-12,
            "{iterations} {residual:e}"
        );
        let mut r = vec![0.0; n];
        a.residual(&b, &x, &mut r);
        let norm = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
        assert!(norm(&r) <= 1e-11 * norm(&b), "{:e}", norm(&r) / norm(&b));
        // New values (a Newton iteration): the solution follows.
        let mut a2 = a.clone();
        a2.val.iter_mut().for_each(|v| *v *= 2.0);
        amg.refresh(&a2);
        amg.attach_operator(&a2);
        let gpu = amg.gpu_cycle().expect("still on the card");
        let mut y = vec![0.0; n];
        gpu.solve(&b, &mut y, 1e-12, 1000, &|| false, &mut |_, _| false)
            .unwrap()
            .unwrap();
        for (x, y) in x.iter().zip(&y) {
            assert!((x - 2.0 * y).abs() <= 1e-9 * x.abs().max(1.0), "{x} vs {y}");
        }
        // A caller that gives up (a stalling iteration): the card stops at the first
        // check that says so, with the iterations done.
        let mut asked = Vec::new();
        let outcome = gpu
            .solve(&b, &mut y, 1e-12, 1000, &|| false, &mut |i, r| {
                asked.push((i, r));
                i >= 2
            })
            .unwrap()
            .unwrap();
        let CgOutcome::NotConverged { iterations, .. } = outcome else {
            panic!("did not stop: {outcome:?}");
        };
        assert_eq!(iterations, asked.last().unwrap().0);
        assert!(asked.iter().all(|&(_, r)| r.is_finite() && r > 0.0));
    }

    #[test]
    fn sliced_storage_keeps_every_block() {
        let (a, _) = grid(5);
        let s = Sliced::new(&a);
        let kept = s.source.iter().filter(|&&k| k != u32::MAX).count();
        assert_eq!(kept, a.blocks());
        let a32: Bsr<f32> = a.converted();
        assert_eq!(s.values_of(&a32), s.val);
    }
}
