//! The engine behind [`crate::Engine`] with the feature `gpu`.

use plxs_gpu::{Double, Gpu, Options, Use, wgpu};

use crate::elements::{Isochoric, Material, STATE_SIZE, Volumetric};
use crate::{Elements, MAX_CHUNK, Reached, Setup, Start};

/// Values per material in the materials buffer of the element kernels (`MAT` in
/// `elements.wgsl`), see [`materials`].
const MATERIAL_BLOCK: usize = 44;

/// The state variables of one family of elements on the card, value by value
/// (`[(point · STATE_SIZE + s) · count + e]`).
struct History {
    buffer: wgpu::Buffer,
    count: usize,
    points: usize,
    /// Per element the index of its first state variable in the processor's vector.
    state: Vec<u32>,
}

/// The materials buffer: the blocks, then the curves. A block holds the kind (0 elastic,
/// 1 J2, 2 J2 at finite strains, 3 hyperelastic) and
/// - elastic: the elastic matrix at 1..37;
/// - J2: shear and bulk modulus at 37, 38, start and length of the isotropic and the
///   kinematic curve at 39..43 (pairs behind the blocks);
/// - hyperelastic: at 1 the volumetric model (0 polynomial, 1 Arruda-Boyce), its
///   coefficients at 2..5, at 5 the isochoric model (0 polynomial, 1 Arruda-Boyce,
///   2 Ogden), at 6 its number of terms, the terms from 7 (polynomial `(i, j, C_ij)`,
///   Arruda-Boyce `(μ, λm)`, Ogden `(μ_p, α_p)`).
fn materials(materials: &[Material]) -> Result<Vec<f32>, String> {
    let mut out = vec![0.0f32; MATERIAL_BLOCK * materials.len()];
    for (i, m) in materials.iter().enumerate() {
        let at = i * MATERIAL_BLOCK;
        match m {
            Material::Elastic(d) => {
                for (k, v) in d.iter().flatten().enumerate() {
                    out[at + 1 + k] = *v as f32;
                }
            }
            Material::J2 {
                shear,
                bulk,
                isotropic,
                kinematic,
                finite,
            } => {
                out[at] = if *finite { 2.0 } else { 1.0 };
                out[at + 37] = *shear as f32;
                out[at + 38] = *bulk as f32;
                for (j, curve) in [isotropic, kinematic].iter().enumerate() {
                    let start = out.len();
                    out.extend(curve.iter().flat_map(|&(x, y)| [x as f32, y as f32]));
                    out[at + 39 + 2 * j] = start as f32;
                    out[at + 40 + 2 * j] = curve.len() as f32;
                }
            }
            Material::Hyperelastic {
                isochoric,
                volumetric,
            } => {
                out[at] = 3.0;
                let (kind, coefficients) = match volumetric {
                    Volumetric::Polynomial(c) => (0.0, c.clone()),
                    Volumetric::ArrudaBoyce(c) => (1.0, vec![*c]),
                };
                let (model, terms): (f32, Vec<f64>) = match isochoric {
                    Isochoric::Polynomial(t) => (
                        0.0,
                        t.iter()
                            .flat_map(|&(i, j, c)| [f64::from(i), f64::from(j), c])
                            .collect(),
                    ),
                    Isochoric::ArrudaBoyce { mu, locking } => (1.0, vec![*mu, *locking]),
                    Isochoric::Ogden(t) => (2.0, t.iter().flat_map(|&(m, a)| [m, a]).collect()),
                };
                if coefficients.len() > 3 || terms.len() > 27 {
                    return Err("a hyperelastic material with too many terms".to_owned());
                }
                out[at + 1] = kind;
                for (k, c) in coefficients.iter().enumerate() {
                    out[at + 2 + k] = *c as f32;
                }
                out[at + 5] = model;
                out[at + 6] = match isochoric {
                    Isochoric::Polynomial(t) => t.len(),
                    Isochoric::ArrudaBoyce { .. } => 1,
                    Isochoric::Ogden(t) => t.len(),
                } as f32;
                for (k, c) in terms.iter().enumerate() {
                    out[at + 7 + k] = *c as f32;
                }
            }
        }
    }
    Ok(out)
}

const GROUP: usize = 256;
/// Threads per work group of the element kernels.
const ELEMENT_GROUP: usize = 64;

/// The parameters of one step as the shaders read them (`Step`).
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct StepParams {
    h: f64,
    c: f64,
    s: f64,
    alpha: f64,
}

/// The sizes the shaders read (`Dims`; the element kernels name the fields `count`,
/// `groups_x`, `base`).
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Dims {
    n: u32,
    groups_x: u32,
    base: u32,
    pad: u32,
}

/// The kinds of dispatches, as the trace reports them.
const KINDS: [&str; 4] = ["advance", "forces", "gather", "accelerate"];

/// One dispatch of a time step.
struct Pass {
    pipeline: wgpu::ComputePipeline,
    bind: wgpu::BindGroup,
    /// Whether it reads the step's parameters (a dynamic offset).
    step: bool,
    groups: (u32, u32),
    /// Index into [`KINDS`].
    kind: usize,
}

pub(crate) struct Engine {
    pub(crate) gpu: Gpu,
    n: usize,
    alpha: f64,
    stride: u64,
    params: wgpu::Buffer,
    u: wgpu::Buffer,
    v: wgpu::Buffer,
    a: wgpu::Buffer,
    loads: wgpu::Buffer,
    energy: wgpu::Buffer,
    staging: wgpu::Buffer,
    passes: Vec<Pass>,
    histories: Vec<History>,
    /// `PLXS_GPU_TRACE`: the time of every kind of dispatch on the card.
    trace: Option<Trace>,
}

/// The times the card took, from its timestamps: each dispatch of the first time steps of
/// a traced run in a pass of its own.
struct Trace {
    queries: wgpu::QuerySet,
    resolve: wgpu::Buffer,
    read: wgpu::Buffer,
    period: f64,
    /// Time steps per run that are timed (the query set holds at most [`QUERIES`]).
    timed: usize,
    /// Seconds per kind of dispatch over the timed steps; wall clock of the runs; time
    /// steps, timed steps.
    kernels: [f64; 4],
    wall: f64,
    steps: usize,
    timed_steps: usize,
}

/// The most queries of a query set.
const QUERIES: usize = 4096;

impl Drop for Engine {
    fn drop(&mut self) {
        if let Some(t) = &self.trace
            && t.timed_steps > 0
        {
            let kernels: Vec<String> = KINDS
                .iter()
                .zip(t.kernels)
                .filter(|&(_, s)| s > 0.0)
                .map(|(name, s)| format!("{name} {:.3} ms", 1e3 * s / t.timed_steps as f64))
                .collect();
            eprintln!(
                " gpu trace: {} time steps, per step {:.3} ms wall clock, on the card: {}",
                t.steps,
                1e3 * t.wall / t.steps as f64,
                kernels.join(", ")
            );
        }
    }
}

/// The element kernels' data, value by value (`[k * count + e]`).
fn transpose<T: Copy + Default>(values: &[T], count: usize) -> Vec<T> {
    if count == 0 {
        return Vec::new();
    }
    let size = values.len() / count;
    let mut out = vec![T::default(); values.len()];
    for e in 0..count {
        for k in 0..size {
            out[k * count + e] = values[e * size + k];
        }
    }
    out
}

fn single(values: &[f64]) -> Vec<f32> {
    values.iter().map(|&x| x as f32).collect()
}

impl Engine {
    pub(crate) fn new(setup: &Setup<'_>, any: bool) -> Result<Self, String> {
        let n = setup.inverse_mass.len();
        if n == 0 {
            return Err("no unknowns".to_owned());
        }
        let tracing = std::env::var("PLXS_GPU_TRACE").is_ok_and(|v| v != "0");
        let gpu = Gpu::open(Options {
            label: "plxs explicit",
            any_adapter: any,
            double: Double::Required,
            timestamps: tracing,
        })?;
        let too_large = || format!("the model is too large for the buffers of {}", gpu.name);
        if !gpu.fits(8 * 6 * n as u64) {
            return Err(too_large());
        }
        // Out of memory and internal errors as well as validation errors: a buffer the
        // device cannot allocate is invalid without a validation error of its own.
        let memory = gpu.device.push_error_scope(wgpu::ErrorFilter::OutOfMemory);
        let internal = gpu.device.push_error_scope(wgpu::ErrorFilter::Internal);
        let scope = gpu.device.push_error_scope(wgpu::ErrorFilter::Validation);
        let f64s = 8 * n as u64;
        let u = gpu.storage("u", f64s);
        let v = gpu.storage("v", f64s);
        let a = gpu.storage("a", f64s);
        let du = gpu.storage("du", f64s);
        let fs = gpu.storage("forces", 2 * f64s);
        let loads = gpu.storage("loads", 2 * f64s);
        let energy = gpu.storage("energy", 6 * f64s);
        let im = gpu.init("inverse mass", bytemuck::cast_slice(setup.inverse_mass));
        let staging = gpu.staging("staging", 6 * f64s);
        let stride = u64::from(gpu.limits.min_uniform_buffer_offset_alignment)
            .max(std::mem::size_of::<StepParams>() as u64);
        let params = gpu.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("steps"),
            size: stride * MAX_CHUNK as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let groups = gpu.fold(n, GROUP);
        let dims_of = |count: usize, groups: (u32, u32)| {
            gpu.uniform(
                "dims",
                bytemuck::bytes_of(&Dims {
                    n: count as u32,
                    groups_x: groups.0,
                    base: 0,
                    pad: 0,
                }),
            )
        };
        let dims = dims_of(n, groups);
        let step_binding = || {
            wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                buffer: &params,
                offset: 0,
                size: wgpu::BufferSize::new(std::mem::size_of::<StepParams>() as u64),
            })
        };
        use Use::*;
        let mut passes = Vec::new();
        let layout = gpu.layout(
            "advance",
            &[Uniform, Dynamic, Write, Write, Read, Write, Read],
        );
        passes.push(Pass {
            pipeline: gpu.pipeline(include_str!("advance.wgsl"), "advance", &layout, &[]),
            bind: gpu.bind(
                &layout,
                vec![
                    dims.as_entire_binding(),
                    step_binding(),
                    u.as_entire_binding(),
                    v.as_entire_binding(),
                    a.as_entire_binding(),
                    du.as_entire_binding(),
                    im.as_entire_binding(),
                ],
            ),
            step: true,
            groups,
            kind: 0,
        });
        let histories = Self::element_passes(&gpu, setup.elements, n, &u, &v, &fs, &mut passes)?;
        let layout = gpu.layout(
            "accelerate",
            &[Uniform, Dynamic, Read, Write, Read, Read, Read, Read, Write],
        );
        passes.push(Pass {
            pipeline: gpu.pipeline(include_str!("accelerate.wgsl"), "accelerate", &layout, &[]),
            bind: gpu.bind(
                &layout,
                vec![
                    dims.as_entire_binding(),
                    step_binding(),
                    v.as_entire_binding(),
                    a.as_entire_binding(),
                    du.as_entire_binding(),
                    im.as_entire_binding(),
                    loads.as_entire_binding(),
                    fs.as_entire_binding(),
                    energy.as_entire_binding(),
                ],
            ),
            step: true,
            groups,
            kind: 3,
        });
        let timed = (QUERIES / (2 * passes.len())).min(MAX_CHUNK);
        let trace = (gpu.timestamps && timed > 0).then(|| {
            let count = (2 * passes.len() * timed) as u32;
            Trace {
                queries: gpu.device.create_query_set(&wgpu::QuerySetDescriptor {
                    label: Some("trace"),
                    ty: wgpu::QueryType::Timestamp,
                    count,
                }),
                resolve: gpu.device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("trace"),
                    size: 8 * u64::from(count),
                    usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
                    mapped_at_creation: false,
                }),
                read: gpu.staging("trace", 8 * u64::from(count)),
                period: f64::from(gpu.queue.get_timestamp_period()) * 1e-9,
                timed,
                kernels: [0.0; 4],
                wall: 0.0,
                steps: 0,
                timed_steps: 0,
            }
        });
        let validation = plxs_gpu::block_on(scope.pop());
        let internal = plxs_gpu::block_on(internal.pop());
        let memory = plxs_gpu::block_on(memory.pop());
        let errors: Vec<String> = [memory, internal]
            .into_iter()
            .flatten()
            .map(|e| e.to_string())
            .chain(gpu.take_error())
            .chain(validation.map(|e| e.to_string()))
            .collect();
        if !errors.is_empty() {
            // What this run holds on the card, for a failure for want of memory.
            let held = gpu
                .device
                .generate_allocator_report()
                .map(|r| {
                    let mb = |b: u64| b as f64 / 1048576.0;
                    format!(
                        "; this run holds {:.1} MB in {} buffers, {:.1} MB reserved in {} blocks",
                        mb(r.total_allocated_bytes),
                        r.allocations.len(),
                        mb(r.total_reserved_bytes),
                        r.blocks.len()
                    )
                })
                .unwrap_or_default();
            return Err(format!("{}: {}{held}", gpu.name, errors.join("; ")));
        }
        Ok(Self {
            gpu,
            n,
            alpha: setup.alpha,
            stride,
            params,
            u,
            v,
            a,
            loads,
            energy,
            staging,
            passes,
            histories,
            trace,
        })
    }

    /// The dispatches of the element forces: one per family, then the sum per DOF into
    /// `fs`.
    fn element_passes(
        gpu: &Gpu,
        elements: &Elements,
        n: usize,
        u: &wgpu::Buffer,
        v: &wgpu::Buffer,
        fs: &wgpu::Buffer,
        passes: &mut Vec<Pass>,
    ) -> Result<Vec<History>, String> {
        let mut histories = Vec::new();
        use Use::*;
        let too_large = || format!("the model is too large for the buffers of {}", gpu.name);
        // The positions of every DOF's element forces.
        let rows: usize = elements.blocks.iter().map(|b| b.dofs.len()).sum();
        let total: usize = elements
            .families
            .iter()
            .map(|f| 3 * f.nodes * f.count)
            .sum::<usize>()
            + rows;
        if total == 0 {
            return Err("no elements".to_owned());
        }
        if total >= u32::MAX as usize || !gpu.fits(4 * total as u64) {
            return Err(too_large());
        }
        let mut lists: Vec<Vec<u32>> = vec![Vec::new(); n];
        let mut base = 0;
        let mut bases = Vec::with_capacity(elements.families.len());
        for family in &elements.families {
            let nd = 3 * family.nodes;
            for e in 0..family.count {
                for k in 0..nd {
                    let dof = family.dofs[e * nd + k] as usize;
                    if dof >= n {
                        return Err(format!("DOF {dof} of {n}"));
                    }
                    lists[dof].push((base + k * family.count + e) as u32);
                }
            }
            bases.push(base);
            base += nd * family.count;
        }
        // The block rows behind the element forces, and their entries.
        let block_base = base;
        let mut block_ptr = vec![0u32];
        let mut block_columns = Vec::new();
        let mut block_values: Vec<f64> = Vec::new();
        for block in &elements.blocks {
            let m = block.dofs.len();
            for (i, &dof) in block.dofs.iter().enumerate() {
                if dof as usize >= n {
                    return Err(format!("DOF {dof} of {n}"));
                }
                lists[dof as usize].push((block_base + block_ptr.len() - 1) as u32);
                for (j, &col) in block.dofs.iter().enumerate() {
                    let k = block.stiffness[i * m + j];
                    if k != 0.0 {
                        block_columns.push(col);
                        block_values.push(k);
                    }
                }
                block_ptr.push(block_columns.len() as u32);
            }
        }
        let mut ptr = Vec::with_capacity(n + 1);
        ptr.push(0u32);
        let mut positions = Vec::with_capacity(total);
        for list in lists {
            positions.extend(list);
            ptr.push(positions.len() as u32);
        }
        let fint = gpu.storage("element forces", 4 * total as u64);
        let fvis = gpu.storage("element viscous forces", 4 * total as u64);
        let materials = gpu.init(
            "materials",
            bytemuck::cast_slice(&materials(&elements.materials)?),
        );
        let layout = gpu.layout(
            "elements",
            &[
                Uniform, Read, Read, Read, Read, Read, Read, Read, Read, Write, Write, Write,
            ],
        );
        for (family, &base) in elements.families.iter().zip(&bases) {
            if family.count == 0 {
                continue;
            }
            let count = family.count;
            let gradients = transpose(&single(&family.gradients), count);
            if !gpu.fits(4 * gradients.len() as u64) || !gpu.fits(4 * family.control.len() as u64) {
                return Err(too_large());
            }
            let history = if family.history {
                let bytes = (8 * STATE_SIZE * family.points * count) as u64;
                if !gpu.fits(bytes) {
                    return Err(too_large());
                }
                let buffer = gpu.storage("state variables", bytes);
                histories.push(History {
                    buffer: buffer.clone(),
                    count,
                    points: family.points,
                    state: family.state.clone(),
                });
                buffer
            } else {
                gpu.storage("no state variables", 16)
            };
            let source = include_str!("elements.wgsl")
                .replace("__NN__", &family.nodes.to_string())
                .replace("__NP__", &family.points.to_string())
                .replace("__HG__", if family.hourglass { "true" } else { "false" })
                .replace("__NL__", if elements.nlgeom { "true" } else { "false" })
                .replace("__DL__", if family.dense { "true" } else { "false" })
                .replace("__KM__", if family.matrix { "true" } else { "false" })
                .replace("__PS__", &family.point_size().to_string());
            let groups = gpu.fold(count, ELEMENT_GROUP);
            let dims = gpu.uniform(
                "element dims",
                bytemuck::bytes_of(&Dims {
                    n: count as u32,
                    groups_x: groups.0,
                    base: base as u32,
                    pad: 0,
                }),
            );
            passes.push(Pass {
                pipeline: gpu.pipeline(&source, "elements", &layout, &[]),
                bind: gpu.bind(
                    &layout,
                    vec![
                        dims.as_entire_binding(),
                        u.as_entire_binding(),
                        v.as_entire_binding(),
                        gpu.init(
                            "dofs",
                            bytemuck::cast_slice(&transpose(&family.dofs, count)),
                        )
                        .as_entire_binding(),
                        gpu.init("gradients", bytemuck::cast_slice(&gradients))
                            .as_entire_binding(),
                        gpu.init(
                            "viscosity",
                            bytemuck::cast_slice(&transpose(&single(&family.viscosity), count)),
                        )
                        .as_entire_binding(),
                        gpu.init(
                            "hourglass",
                            bytemuck::cast_slice(&transpose(&single(&family.control), count)),
                        )
                        .as_entire_binding(),
                        gpu.init("material", bytemuck::cast_slice(&family.material))
                            .as_entire_binding(),
                        materials.as_entire_binding(),
                        fint.as_entire_binding(),
                        fvis.as_entire_binding(),
                        history.as_entire_binding(),
                    ],
                ),
                step: false,
                groups,
                kind: 1,
            });
        }
        if rows > 0 {
            if !gpu.fits(12 * block_values.len() as u64 + 4) {
                return Err(too_large());
            }
            let groups = gpu.fold(rows, ELEMENT_GROUP);
            let dims = gpu.uniform(
                "block dims",
                bytemuck::bytes_of(&Dims {
                    n: rows as u32,
                    groups_x: groups.0,
                    base: block_base as u32,
                    pad: 0,
                }),
            );
            let layout = gpu.layout("blocks", &[Uniform, Read, Read, Read, Read, Write]);
            // An empty buffer cannot be bound.
            if block_values.is_empty() {
                block_columns.push(0);
                block_values.push(0.0);
            }
            passes.push(Pass {
                pipeline: gpu.pipeline(include_str!("blocks.wgsl"), "blocks", &layout, &[]),
                bind: gpu.bind(
                    &layout,
                    vec![
                        dims.as_entire_binding(),
                        u.as_entire_binding(),
                        gpu.init("block pointers", bytemuck::cast_slice(&block_ptr))
                            .as_entire_binding(),
                        gpu.init("block columns", bytemuck::cast_slice(&block_columns))
                            .as_entire_binding(),
                        gpu.init("block values", bytemuck::cast_slice(&block_values))
                            .as_entire_binding(),
                        fint.as_entire_binding(),
                    ],
                ),
                step: false,
                groups,
                kind: 1,
            });
        }
        let groups = gpu.fold(n, GROUP);
        let dims = gpu.uniform(
            "gather dims",
            bytemuck::bytes_of(&Dims {
                n: n as u32,
                groups_x: groups.0,
                base: positions.len() as u32,
                pad: 0,
            }),
        );
        let layout = gpu.layout("gather", &[Uniform, Read, Read, Read, Read, Write]);
        passes.push(Pass {
            pipeline: gpu.pipeline(include_str!("gather.wgsl"), "gather", &layout, &[]),
            bind: gpu.bind(
                &layout,
                vec![
                    dims.as_entire_binding(),
                    gpu.init("pointers", bytemuck::cast_slice(&ptr))
                        .as_entire_binding(),
                    gpu.init("positions", bytemuck::cast_slice(&positions))
                        .as_entire_binding(),
                    fint.as_entire_binding(),
                    fvis.as_entire_binding(),
                    fs.as_entire_binding(),
                ],
            ),
            step: false,
            groups,
            kind: 2,
        });
        Ok(histories)
    }

    pub(crate) fn load(&mut self, start: &Start<'_>) -> Result<(), String> {
        let n = self.n;
        for x in [
            start.u,
            start.v,
            start.a,
            start.internal,
            start.viscous,
            start.loads,
        ] {
            if x.len() != n {
                return Err(format!("{} values for {n} unknowns", x.len()));
            }
        }
        let q = &self.gpu.queue;
        q.write_buffer(&self.u, 0, bytemuck::cast_slice(start.u));
        q.write_buffer(&self.v, 0, bytemuck::cast_slice(start.v));
        q.write_buffer(&self.a, 0, bytemuck::cast_slice(start.a));
        let bytes = 8 * n as u64;
        q.write_buffer(&self.loads, 0, bytemuck::cast_slice(start.loads));
        q.write_buffer(&self.loads, bytes, bytemuck::cast_slice(start.loads));
        let mut energy = Vec::with_capacity(6 * n);
        energy.extend_from_slice(start.internal);
        energy.extend_from_slice(start.loads);
        energy.extend_from_slice(start.viscous);
        energy.resize(6 * n, 0.0);
        q.write_buffer(&self.energy, 0, bytemuck::cast_slice(&energy));
        for h in &self.histories {
            let mut values = vec![0.0_f64; STATE_SIZE * h.points * h.count];
            for (e, &first) in h.state.iter().enumerate() {
                if first == u32::MAX {
                    continue;
                }
                for k in 0..STATE_SIZE * h.points {
                    let at = first as usize + k;
                    let value = start.material.get(at).ok_or_else(|| {
                        format!("{} state variables, wanted {at}", start.material.len())
                    })?;
                    values[k * h.count + e] = *value;
                }
            }
            self.gpu.write(&h.buffer, 0, bytemuck::cast_slice(&values));
        }
        Ok(())
    }

    /// Writes the state variables on the card into `material`.
    pub(crate) fn read_state(&mut self, material: &mut [f64]) -> Result<(), String> {
        for h in &self.histories {
            let bytes = (8 * STATE_SIZE * h.points * h.count) as u64;
            let staging = self.gpu.staging("state variables", bytes);
            let mut encoder =
                self.gpu
                    .device
                    .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                        label: Some("read state"),
                    });
            encoder.copy_buffer_to_buffer(&h.buffer, 0, &staging, 0, bytes);
            self.gpu.queue.submit([encoder.finish()]);
            let values: Vec<f64> = self
                .gpu
                .map(&staging, |b| bytemuck::cast_slice::<u8, f64>(b).to_vec())?;
            for (e, &first) in h.state.iter().enumerate() {
                if first == u32::MAX {
                    continue;
                }
                for k in 0..STATE_SIZE * h.points {
                    if let Some(slot) = material.get_mut(first as usize + k) {
                        *slot = values[k * h.count + e];
                    }
                }
            }
        }
        match self.gpu.take_error() {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    pub(crate) fn run(
        &mut self,
        steps: &[(f64, f64)],
        from: &[f64],
        to: &[f64],
    ) -> Result<(), String> {
        if steps.is_empty() {
            return Ok(());
        }
        let n = self.n;
        if from.len() != n || to.len() != n {
            return Err("the loads have the wrong length".to_owned());
        }
        let total: f64 = steps.iter().map(|s| s.0).sum();
        let mut params = vec![0u8; self.stride as usize * steps.len()];
        let mut elapsed = 0.0;
        for (k, &(h, c)) in steps.iter().enumerate() {
            elapsed += h;
            let p = StepParams {
                h,
                c,
                s: if total > 0.0 { elapsed / total } else { 1.0 },
                alpha: self.alpha,
            };
            let at = k * self.stride as usize;
            params[at..at + std::mem::size_of::<StepParams>()]
                .copy_from_slice(bytemuck::bytes_of(&p));
        }
        let q = &self.gpu.queue;
        q.write_buffer(&self.params, 0, &params);
        let bytes = 8 * n as u64;
        q.write_buffer(&self.loads, 0, bytemuck::cast_slice(from));
        q.write_buffer(&self.loads, bytes, bytemuck::cast_slice(to));
        let started = std::time::Instant::now();
        let mut encoder = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("explicit"),
            });
        let offset = |k: usize| (k as u64 * self.stride) as u32;
        // The first steps of a traced run: every dispatch in a pass of its own, timed by
        // the card.
        let timed = self.trace.as_ref().map_or(0, |t| t.timed.min(steps.len()));
        if let Some(t) = &self.trace {
            for k in 0..timed {
                for (j, p) in self.passes.iter().enumerate() {
                    let at = (2 * (k * self.passes.len() + j)) as u32;
                    let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                        label: Some("traced"),
                        timestamp_writes: Some(wgpu::ComputePassTimestampWrites {
                            query_set: &t.queries,
                            beginning_of_pass_write_index: Some(at),
                            end_of_pass_write_index: Some(at + 1),
                        }),
                    });
                    pass.set_pipeline(&p.pipeline);
                    let offsets = [offset(k)];
                    pass.set_bind_group(0, &p.bind, if p.step { &offsets[..] } else { &[] });
                    pass.dispatch_workgroups(p.groups.0, p.groups.1, 1);
                }
            }
            let count = (2 * timed * self.passes.len()) as u32;
            encoder.resolve_query_set(&t.queries, 0..count, &t.resolve, 0);
            encoder.copy_buffer_to_buffer(&t.resolve, 0, &t.read, 0, 8 * u64::from(count));
        }
        if timed < steps.len() {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("explicit"),
                timestamp_writes: None,
            });
            for k in timed..steps.len() {
                let offsets = [offset(k)];
                for p in &self.passes {
                    pass.set_pipeline(&p.pipeline);
                    pass.set_bind_group(0, &p.bind, if p.step { &offsets[..] } else { &[] });
                    pass.dispatch_workgroups(p.groups.0, p.groups.1, 1);
                }
            }
        }
        // The parameter buffer is written again by the next run: wait for this one first.
        self.gpu.queue.submit([encoder.finish()]);
        self.gpu.wait()?;
        if let Some(t) = &mut self.trace {
            let kinds: Vec<usize> = self.passes.iter().map(|p| p.kind).collect();
            let times = self.gpu.map(&t.read, |bytes| {
                let stamps: &[u64] = bytemuck::cast_slice(bytes);
                let mut times = [0.0; 4];
                for k in 0..timed * kinds.len() {
                    let ticks = stamps[2 * k + 1].saturating_sub(stamps[2 * k]);
                    times[kinds[k % kinds.len()]] += ticks as f64 * t.period;
                }
                times
            })?;
            for (sum, time) in t.kernels.iter_mut().zip(times) {
                *sum += time;
            }
            t.wall += started.elapsed().as_secs_f64();
            t.steps += steps.len();
            t.timed_steps += timed;
        }
        match self.gpu.take_error() {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    pub(crate) fn read(&mut self) -> Result<Reached, String> {
        let n = self.n;
        let bytes = 8 * n as u64;
        let mut encoder = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("read"),
            });
        encoder.copy_buffer_to_buffer(&self.u, 0, &self.staging, 0, bytes);
        encoder.copy_buffer_to_buffer(&self.v, 0, &self.staging, bytes, bytes);
        encoder.copy_buffer_to_buffer(&self.a, 0, &self.staging, 2 * bytes, bytes);
        encoder.copy_buffer_to_buffer(&self.energy, 3 * bytes, &self.staging, 3 * bytes, 3 * bytes);
        self.gpu.queue.submit([encoder.finish()]);
        let reached = self.gpu.map(&self.staging, |bytes| {
            let data: &[f64] = bytemuck::cast_slice(bytes);
            let sum = |k: usize| data[k * n..(k + 1) * n].iter().sum::<f64>();
            Reached {
                u: data[..n].to_vec(),
                v: data[n..2 * n].to_vec(),
                a: data[2 * n..3 * n].to_vec(),
                internal_work: sum(3),
                external_work: sum(4),
                dissipation: sum(5),
            }
        })?;
        if let Some(e) = self.gpu.take_error() {
            return Err(e);
        }
        if reached.u.iter().any(|x| !x.is_finite()) {
            return Err("the displacements on the graphics card are not finite".to_owned());
        }
        Ok(reached)
    }
}
