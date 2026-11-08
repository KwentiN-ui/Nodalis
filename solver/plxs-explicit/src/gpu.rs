//! The engine behind [`crate::Engine`] with the feature `gpu`.

use plxs_gpu::{Double, Gpu, Options, Use, wgpu};

use crate::contact::{self, CDims, SLOT, STATE};
use crate::elements::{Isochoric, Material, STATE_SIZE, Volumetric};
use crate::{Candidates, Elements, MAX_CHUNK, Ran, Reached, Setup, Sparse, Start};

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
    s_before: f64,
    /// The index of the time step in the run (`contact_gate`).
    index: f64,
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
const KINDS: [&str; 5] = ["advance", "forces", "gather", "accelerate", "contact"];

/// One dispatch of a time step.
struct Pass {
    pipeline: wgpu::ComputePipeline,
    bind: wgpu::BindGroup,
    /// Whether it reads the step's parameters (a dynamic offset).
    step: bool,
    groups: (u32, u32),
    /// Index into [`KINDS`].
    kind: usize,
    /// With contact: the offset of its dispatch in the indirect arguments (all passes but
    /// the gate, which stops the others by zeroing them).
    indirect: Option<u64>,
}

impl Pass {
    fn dispatch(
        &self,
        pass: &mut wgpu::ComputePass<'_>,
        offsets: &[u32],
        args: Option<&wgpu::Buffer>,
    ) {
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.bind, if self.step { offsets } else { &[] });
        match (self.indirect, args) {
            (Some(at), Some(args)) => pass.dispatch_workgroups_indirect(args, at),
            _ => pass.dispatch_workgroups(self.groups.0, self.groups.1, 1),
        }
    }
}

/// The watch over the time step (`guard.wgsl`).
struct GuardGpu {
    pipeline: wgpu::ComputePipeline,
    bind: wgpu::BindGroup,
    groups: (u32, u32),
    reference: wgpu::Buffer,
    result: wgpu::Buffer,
    staging: wgpu::Buffer,
    count: usize,
}

/// The contact on the card (`contact.wgsl`).
struct ContactGpu {
    contact: crate::Contact,
    dims: CDims,
    cdims: wgpu::Buffer,
    ints: wgpu::Buffer,
    reals: wgpu::Buffer,
    coords: wgpu::Buffer,
    state: wgpu::Buffer,
    rows: wgpu::Buffer,
    flags: wgpu::Buffer,
    flag_staging: wgpu::Buffer,
    args: wgpu::Buffer,
    /// Per search: the lists, the positions of the checked nodes, the slots.
    lists: wgpu::Buffer,
    prepared: wgpu::Buffer,
    slots: wgpu::Buffer,
    layout: wgpu::BindGroupLayout,
    gate_layout: wgpu::BindGroupLayout,
    /// The passes: gate, point, rows, cap, gather, check.
    passes: [usize; 6],
}

/// Flags of `contact.wgsl`.
const FLAGS: usize = 8;

/// The most passes of a time step with contact (their indirect dispatches).
const MAX_PASSES: usize = 256;

impl ContactGpu {
    fn new(
        gpu: &Gpu,
        contact: &crate::Contact,
        candidates: &Candidates<'_>,
        mass: &[f64],
    ) -> Result<Self, String> {
        let n = mass.len();
        if contact.coordinates.len() != n || contact.moving.len() != n {
            return Err(format!(
                "contact coordinates for {} DOFs, wanted {n}",
                contact.coordinates.len()
            ));
        }
        let mut dims = CDims {
            n: n as u32,
            events_from: u32::MAX,
            ..Default::default()
        };
        let (ints, reals) = contact::pack(contact, &mut dims)?;
        let points = contact.points.len();
        // The rows per DOF, then `1/√m` of the DOFs that move.
        let mut rows = vec![0.0; 2 * n];
        for (i, (&m, &moving)) in mass.iter().zip(&contact.moving).enumerate() {
            if moving && m > 0.0 {
                rows[n + i] = 1.0 / m.sqrt();
            }
        }
        use Use::*;
        // The gate also writes the indirect dispatches, which the other passes must not
        // bind (a buffer is either storage or dispatch arguments in a dispatch).
        let uses = [
            Uniform, Dynamic, Read, Read, Read, Read, Read, Read, Read, Write, Write, Write, Write,
            Write, Write,
        ];
        let layout = gpu.layout("contact", &uses[..14]);
        let gate_layout = gpu.layout("contact gate", &uses);
        let empty = gpu.storage("contact lists", 16);
        let mut c = Self {
            contact: contact.clone(),
            dims,
            cdims: gpu.buffer(&wgpu::BufferDescriptor {
                label: Some("contact dims"),
                size: std::mem::size_of::<CDims>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }),
            ints: gpu.init("contact faces and points", bytemuck::cast_slice(&ints)),
            reals: gpu.init("contact data", bytemuck::cast_slice(&reals)),
            coords: gpu.init(
                "contact coordinates",
                bytemuck::cast_slice(&contact.coordinates),
            ),
            state: gpu.init(
                "contact state",
                bytemuck::cast_slice(&contact::state_values(points, None)),
            ),
            rows: gpu.init("contact rows", bytemuck::cast_slice(&rows)),
            flags: gpu.buffer(&wgpu::BufferDescriptor {
                label: Some("contact flags"),
                size: 4 * FLAGS as u64,
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            }),
            flag_staging: gpu.staging("contact flags", 4 * FLAGS as u64),
            args: gpu.buffer(&wgpu::BufferDescriptor {
                label: Some("contact dispatches"),
                size: 12 * MAX_PASSES as u64,
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::INDIRECT
                    | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }),
            lists: empty.clone(),
            prepared: empty.clone(),
            slots: empty,
            layout,
            gate_layout,
            passes: [0; 6],
        };
        c.upload(gpu, candidates)?;
        Ok(c)
    }

    /// The buffers of a search.
    fn upload(&mut self, gpu: &Gpu, candidates: &Candidates<'_>) -> Result<(), String> {
        let (lists, slots) = contact::lists(&self.contact, candidates, &mut self.dims)?;
        self.lists = gpu.init("contact lists", bytemuck::cast_slice(&lists));
        self.prepared = gpu.init(
            "contact positions",
            bytemuck::cast_slice(candidates.prepared),
        );
        self.slots = gpu.storage("contact slots", 8 * (SLOT * slots.max(1)) as u64);
        Ok(())
    }

    /// The threads of the passes after the gate: points, gathered nodes, points, gathered
    /// nodes, checked nodes.
    fn counts(&self) -> [usize; 5] {
        let d = &self.dims;
        let points = d.points as usize;
        let nodes = d.gnodes as usize;
        [points, nodes, points, nodes, d.checks as usize]
    }

    fn bind(
        &self,
        gpu: &Gpu,
        params: &wgpu::Buffer,
        u: &wgpu::Buffer,
        v: &wgpu::Buffer,
        fs: &wgpu::Buffer,
    ) -> [wgpu::BindGroup; 2] {
        let resources = || {
            vec![
                self.cdims.as_entire_binding(),
                wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: params,
                    offset: 0,
                    size: wgpu::BufferSize::new(std::mem::size_of::<StepParams>() as u64),
                }),
                self.ints.as_entire_binding(),
                self.reals.as_entire_binding(),
                self.lists.as_entire_binding(),
                self.prepared.as_entire_binding(),
                self.coords.as_entire_binding(),
                u.as_entire_binding(),
                v.as_entire_binding(),
                self.state.as_entire_binding(),
                self.slots.as_entire_binding(),
                self.rows.as_entire_binding(),
                fs.as_entire_binding(),
                self.flags.as_entire_binding(),
            ]
        };
        let mut gate = resources();
        gate.push(self.args.as_entire_binding());
        [
            gpu.bind(&self.layout, resources()),
            gpu.bind(&self.gate_layout, gate),
        ]
    }
}

pub(crate) struct Engine {
    pub(crate) gpu: Gpu,
    /// DOFs and reduced unknowns.
    n: usize,
    r: usize,
    alpha: f64,
    stride: u64,
    params: wgpu::Buffer,
    u: wgpu::Buffer,
    v: wgpu::Buffer,
    q: wgpu::Buffer,
    qd: wgpu::Buffer,
    a: wgpu::Buffer,
    /// The prescribed motion (`expand.wgsl`): g at the chunk's start and end, rate,
    /// acceleration, increment.
    g: wgpu::Buffer,
    /// The reactions of the last step and the work of the prescribed displacements, when
    /// the setup has prescribed motion.
    reaction: Option<wgpu::Buffer>,
    loads: wgpu::Buffer,
    energy: wgpu::Buffer,
    staging: wgpu::Buffer,
    passes: Vec<Pass>,
    histories: Vec<History>,
    fs: wgpu::Buffer,
    contact: Option<ContactGpu>,
    guard: Option<GuardGpu>,
    /// From which time step of a run a contact event stops it.
    pub(crate) events_from: u32,
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
    kernels: [f64; 5],
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
        let n = setup.mass.len();
        let reduction = setup.reduction;
        let r = reduction.len();
        if n == 0 || r == 0 {
            return Err("no unknowns".to_owned());
        }
        if reduction.transform.rows() != n {
            return Err(format!(
                "the constraints have {} rows for {n} DOFs",
                reduction.transform.rows()
            ));
        }
        let tracing = std::env::var("PLXS_GPU_TRACE").is_ok_and(|v| v != "0");
        let gpu = Gpu::open(Options {
            label: "plxs explicit",
            any_adapter: any,
            double: Double::Required,
            timestamps: tracing,
            // The state of a step takes a few hundred MB even for large models, and the
            // budget would refuse the upload and read-back buffers on cards whose shared
            // heap the desktop fills.
            memory_share: false,
        })?;
        let too_large = || format!("the model is too large for the buffers of {}", gpu.name);
        let entries = reduction
            .transform
            .val
            .len()
            .max(reduction.inverse_mass.val.len());
        if !gpu.fits(8 * 6 * n as u64) || !gpu.fits(8 * entries as u64 + 8 * n as u64) {
            return Err(too_large());
        }
        // Out of memory and internal errors as well as validation errors: a buffer the
        // device cannot allocate is invalid without a validation error of its own.
        let memory = gpu.device.push_error_scope(wgpu::ErrorFilter::OutOfMemory);
        let internal = gpu.device.push_error_scope(wgpu::ErrorFilter::Internal);
        let scope = gpu.device.push_error_scope(wgpu::ErrorFilter::Validation);
        let f64s = 8 * n as u64;
        let reduced = 8 * r as u64;
        let u = gpu.storage("u", f64s);
        let v = gpu.storage("v", f64s);
        let du = gpu.storage("du", f64s);
        let q = gpu.storage("q", reduced);
        let qd = gpu.storage("q dot", reduced);
        let a = gpu.storage("a", reduced);
        let rq = gpu.storage("reduced residual", reduced);
        let residual = gpu.storage("residual", f64s);
        let g = gpu.storage("prescribed", 5 * f64s);
        let reaction = setup
            .prescribed_motion
            .then(|| gpu.storage("reactions", 2 * f64s));
        let fs = gpu.storage("forces", 2 * f64s);
        let loads = gpu.storage("loads", 2 * f64s);
        let energy = gpu.storage("energy", 6 * f64s);
        let mass = gpu.init("mass", bytemuck::cast_slice(setup.mass));
        let staging = gpu.staging("staging", 10 * f64s + 3 * reduced);
        let stride = u64::from(gpu.limits.min_uniform_buffer_offset_alignment)
            .max(std::mem::size_of::<StepParams>() as u64);
        let params = gpu.buffer(&wgpu::BufferDescriptor {
            label: Some("steps"),
            // One more for the contact gate after the last time step.
            size: stride * (MAX_CHUNK + 1) as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let groups = gpu.fold(n, GROUP);
        let reduced_groups = gpu.fold(r, GROUP);
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
        let reduced_dims = dims_of(r, reduced_groups);
        let step_binding = || {
            wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                buffer: &params,
                offset: 0,
                size: wgpu::BufferSize::new(std::mem::size_of::<StepParams>() as u64),
            })
        };
        // A sparse matrix as the shaders read it: the row pointers, then the columns; the
        // values. Empty buffers hold one zero.
        let sparse = |name: &str, m: &Sparse| {
            let mut index = m.ptr.clone();
            index.extend_from_slice(&m.col);
            let values = if m.val.is_empty() {
                vec![0.0]
            } else {
                m.val.clone()
            };
            (
                gpu.init(name, bytemuck::cast_slice(&index)),
                gpu.init(name, bytemuck::cast_slice(&values)),
            )
        };
        let transform = sparse("T", &reduction.transform);
        let transposed = sparse("T transposed", &reduction.transform.transpose(r));
        let inverse = sparse("inverse mass", &reduction.inverse_mass);
        use Use::*;
        let mut passes = Vec::new();
        // With contact: the gate first in every time step.
        let mut contact = match setup.contact {
            Some((c, candidates)) => Some(ContactGpu::new(&gpu, c, &candidates, setup.mass)?),
            None => None,
        };
        let contact_bind = contact.as_ref().map(|c| c.bind(&gpu, &params, &u, &v, &fs));
        let contact_pipeline = |layout: &wgpu::BindGroupLayout, entry: &str| {
            gpu.pipeline(include_str!("contact.wgsl"), entry, layout, &[])
        };
        if let (Some(c), Some(bind)) = (&mut contact, &contact_bind) {
            c.passes[0] = passes.len();
            passes.push(Pass {
                pipeline: contact_pipeline(&c.gate_layout, "contact_gate"),
                bind: bind[1].clone(),
                step: true,
                groups: (1, 1),
                kind: 4,
                indirect: None,
            });
        }
        let layout = gpu.layout("advance", &[Uniform, Dynamic, Write, Write, Read]);
        passes.push(Pass {
            pipeline: gpu.pipeline(include_str!("advance.wgsl"), "advance", &layout, &[]),
            bind: gpu.bind(
                &layout,
                vec![
                    reduced_dims.as_entire_binding(),
                    step_binding(),
                    q.as_entire_binding(),
                    qd.as_entire_binding(),
                    a.as_entire_binding(),
                ],
            ),
            step: true,
            groups: reduced_groups,
            kind: 0,
            indirect: None,
        });
        let layout = gpu.layout(
            "expand",
            &[
                Uniform, Dynamic, Read, Read, Write, Read, Read, Write, Write, Write,
            ],
        );
        passes.push(Pass {
            pipeline: gpu.pipeline(include_str!("expand.wgsl"), "expand", &layout, &[]),
            bind: gpu.bind(
                &layout,
                vec![
                    dims.as_entire_binding(),
                    step_binding(),
                    q.as_entire_binding(),
                    qd.as_entire_binding(),
                    g.as_entire_binding(),
                    transform.0.as_entire_binding(),
                    transform.1.as_entire_binding(),
                    u.as_entire_binding(),
                    v.as_entire_binding(),
                    du.as_entire_binding(),
                ],
            ),
            step: true,
            groups,
            kind: 0,
            indirect: None,
        });
        let histories = Self::element_passes(&gpu, setup.elements, n, &u, &v, &fs, &mut passes)?;
        // The springs of the contact into the forces.
        if let (Some(c), Some(bind)) = (&mut contact, &contact_bind) {
            let counts = c.counts();
            for (k, entry) in [
                "contact_point",
                "contact_rows",
                "contact_cap",
                "contact_gather",
                "contact_check",
            ]
            .into_iter()
            .enumerate()
            {
                c.passes[k + 1] = passes.len();
                passes.push(Pass {
                    pipeline: contact_pipeline(&c.layout, entry),
                    bind: bind[0].clone(),
                    step: true,
                    groups: gpu.fold(counts[k], 64),
                    kind: 4,
                    indirect: None,
                });
            }
        }
        let layout = gpu.layout(
            "accelerate",
            &[
                Uniform, Dynamic, Read, Write, Read, Read, Read, Read, Write, Read,
            ],
        );
        passes.push(Pass {
            pipeline: gpu.pipeline(include_str!("accelerate.wgsl"), "accelerate", &layout, &[]),
            bind: gpu.bind(
                &layout,
                vec![
                    dims.as_entire_binding(),
                    step_binding(),
                    v.as_entire_binding(),
                    residual.as_entire_binding(),
                    du.as_entire_binding(),
                    mass.as_entire_binding(),
                    loads.as_entire_binding(),
                    fs.as_entire_binding(),
                    energy.as_entire_binding(),
                    g.as_entire_binding(),
                ],
            ),
            step: true,
            groups,
            kind: 3,
            indirect: None,
        });
        // The reduced residual `Tᵀ r` and the reduced accelerations `M_q⁻¹ Tᵀ r`.
        let layout = gpu.layout("sparse", &[Uniform, Read, Read, Read, Write]);
        let pipeline = gpu.pipeline(include_str!("sparse.wgsl"), "sparse", &layout, &[]);
        for (matrix, x, y) in [(&transposed, &residual, &rq), (&inverse, &rq, &a)] {
            passes.push(Pass {
                pipeline: pipeline.clone(),
                bind: gpu.bind(
                    &layout,
                    vec![
                        reduced_dims.as_entire_binding(),
                        matrix.0.as_entire_binding(),
                        matrix.1.as_entire_binding(),
                        x.as_entire_binding(),
                        y.as_entire_binding(),
                    ],
                ),
                step: false,
                groups: reduced_groups,
                kind: 3,
                indirect: None,
            });
        }
        // The work of the prescribed displacements.
        if let Some(reaction) = &reaction {
            let layout = gpu.layout(
                "react",
                &[Uniform, Read, Read, Read, Read, Read, Read, Write],
            );
            passes.push(Pass {
                pipeline: gpu.pipeline(include_str!("react.wgsl"), "react", &layout, &[]),
                bind: gpu.bind(
                    &layout,
                    vec![
                        dims.as_entire_binding(),
                        a.as_entire_binding(),
                        transform.0.as_entire_binding(),
                        transform.1.as_entire_binding(),
                        mass.as_entire_binding(),
                        residual.as_entire_binding(),
                        g.as_entire_binding(),
                        reaction.as_entire_binding(),
                    ],
                ),
                step: false,
                groups,
                kind: 3,
                indirect: None,
            });
        }
        // With contact every pass but the gate is dispatched indirectly.
        if contact.is_some() && passes.len() > MAX_PASSES {
            return Err(format!(
                "{} passes per time step with contact",
                passes.len()
            ));
        }
        if let Some(c) = &contact {
            for (i, p) in passes.iter_mut().enumerate() {
                if i != c.passes[0] {
                    p.indirect = Some(12 * i as u64);
                }
            }
        }
        let guard = setup.guard.filter(|g| !g.is_empty()).map(|g| {
            let count = g.len();
            let groups = gpu.fold(count, 64);
            let dims = gpu.uniform(
                "guard dims",
                bytemuck::bytes_of(&Dims {
                    n: count as u32,
                    groups_x: groups.0,
                    base: 0,
                    pad: 0,
                }),
            );
            let reference = gpu.storage("guard reference", 4 * count as u64);
            let result = gpu.storage("guard result", 4);
            let layout = gpu.layout("guard", &[Uniform, Read, Read, Read, Read, Read, Write]);
            GuardGpu {
                pipeline: gpu.pipeline(include_str!("guard.wgsl"), "guard", &layout, &[]),
                bind: gpu.bind(
                    &layout,
                    vec![
                        dims.as_entire_binding(),
                        gpu.init("guard offsets", bytemuck::cast_slice(&g.offsets))
                            .as_entire_binding(),
                        gpu.init("guard dofs", bytemuck::cast_slice(&g.dofs))
                            .as_entire_binding(),
                        gpu.init("guard coordinates", bytemuck::cast_slice(&g.coordinates))
                            .as_entire_binding(),
                        u.as_entire_binding(),
                        reference.as_entire_binding(),
                        result.as_entire_binding(),
                    ],
                ),
                groups,
                reference,
                result,
                staging: gpu.staging("guard result", 16),
                count,
            }
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
                resolve: gpu.buffer(&wgpu::BufferDescriptor {
                    label: Some("trace"),
                    size: 8 * u64::from(count),
                    usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
                    mapped_at_creation: false,
                }),
                read: gpu.staging("trace", 8 * u64::from(count)),
                period: f64::from(gpu.queue.get_timestamp_period()) * 1e-9,
                timed,
                kernels: [0.0; 5],
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
                .map(|mut r| {
                    let mb = |b: u64| b as f64 / 1048576.0;
                    r.allocations.sort_by_key(|a| std::cmp::Reverse(a.size));
                    let largest: Vec<String> = r
                        .allocations
                        .iter()
                        .take(4)
                        .map(|a| format!("{} {:.1} MB", a.name, mb(a.size)))
                        .collect();
                    format!(
                        "; this run holds {:.1} MB in {} buffers (largest: {}), {:.1} MB \
                         reserved in {} blocks",
                        mb(r.total_allocated_bytes),
                        r.allocations.len(),
                        largest.join(", "),
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
            r,
            alpha: setup.alpha,
            stride,
            params,
            u,
            v,
            q,
            qd,
            a,
            g,
            reaction,
            loads,
            energy,
            staging,
            passes,
            histories,
            trace,
            fs,
            contact,
            guard,
            events_from: u32::MAX,
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
                indirect: None,
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
                indirect: None,
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
            indirect: None,
        });
        Ok(histories)
    }

    pub(crate) fn load(&mut self, start: &Start<'_>) -> Result<(), String> {
        let (n, r) = (self.n, self.r);
        for x in [
            start.u,
            start.v,
            start.prescribed,
            start.prescribed_rate,
            start.internal,
            start.viscous,
            start.loads,
        ] {
            if x.len() != n {
                return Err(format!("{} values for {n} DOFs", x.len()));
            }
        }
        for x in [start.q, start.qdot, start.a] {
            if x.len() != r {
                return Err(format!("{} values for {r} reduced unknowns", x.len()));
            }
        }
        let q = &self.gpu.queue;
        q.write_buffer(&self.u, 0, bytemuck::cast_slice(start.u));
        q.write_buffer(&self.v, 0, bytemuck::cast_slice(start.v));
        if let Some(c) = &self.contact {
            let values = contact::state_values(c.contact.points.len(), start.contact);
            self.gpu.write(&c.state, 0, bytemuck::cast_slice(&values));
        }
        let bytes = 8 * n as u64;
        // g at the start (also as the end, until a run gives its own), its rate; no
        // acceleration and increment yet.
        let mut g = Vec::with_capacity(5 * n);
        g.extend_from_slice(start.prescribed);
        g.extend_from_slice(start.prescribed);
        g.extend_from_slice(start.prescribed_rate);
        g.resize(5 * n, 0.0);
        self.gpu.write(&self.g, 0, bytemuck::cast_slice(&g));
        if let Some(reaction) = &self.reaction {
            let mut values = vec![0.0; 2 * n];
            if start.reactions.len() == n {
                values[..n].copy_from_slice(start.reactions);
            }
            self.gpu.write(reaction, 0, bytemuck::cast_slice(&values));
        }
        q.write_buffer(&self.q, 0, bytemuck::cast_slice(start.q));
        q.write_buffer(&self.qd, 0, bytemuck::cast_slice(start.qdot));
        q.write_buffer(&self.a, 0, bytemuck::cast_slice(start.a));
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

    /// Uploads the candidates of a new search.
    pub(crate) fn candidates(&mut self, candidates: &Candidates<'_>) -> Result<(), String> {
        let Some(c) = &mut self.contact else {
            return Err("no contact on the card".to_owned());
        };
        c.upload(&self.gpu, candidates)?;
        let bind = c.bind(&self.gpu, &self.params, &self.u, &self.v, &self.fs);
        let counts = c.counts();
        for (k, &at) in c.passes.iter().enumerate() {
            let pass = &mut self.passes[at];
            pass.bind = bind[usize::from(k == 0)].clone();
            if k > 0 {
                pass.groups = self.gpu.fold(counts[k - 1], 64);
            }
        }
        match self.gpu.take_error() {
            Some(e) => Err(e),
            None => Ok(()),
        }
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
        loads: [&[f64]; 2],
        prescribed: [&[f64]; 2],
    ) -> Result<Ran, String> {
        if steps.is_empty() {
            return Ok(Ran::default());
        }
        let n = self.n;
        if loads.iter().chain(&prescribed).any(|x| x.len() != n) {
            return Err(
                "the loads or the prescribed displacements have the wrong length".to_owned(),
            );
        }
        let total: f64 = steps.iter().map(|s| s.0).sum();
        // One more for the gate after the last time step.
        let mut params = vec![0u8; self.stride as usize * (steps.len() + 1)];
        let mut elapsed = 0.0;
        let fraction = |t: f64| if total > 0.0 { t / total } else { 1.0 };
        for (k, &(h, c)) in steps.iter().enumerate() {
            let s_before = if k == 0 { 0.0 } else { fraction(elapsed) };
            elapsed += h;
            let p = StepParams {
                h,
                c,
                s: fraction(elapsed),
                alpha: self.alpha,
                s_before,
                index: k as f64,
            };
            let at = k * self.stride as usize;
            params[at..at + std::mem::size_of::<StepParams>()]
                .copy_from_slice(bytemuck::bytes_of(&p));
        }
        let last = StepParams {
            index: steps.len() as f64,
            ..bytemuck::Zeroable::zeroed()
        };
        let at = steps.len() * self.stride as usize;
        params[at..at + std::mem::size_of::<StepParams>()]
            .copy_from_slice(bytemuck::bytes_of(&last));
        let q = &self.gpu.queue;
        q.write_buffer(&self.params, 0, &params);
        let bytes = 8 * n as u64;
        q.write_buffer(&self.loads, 0, bytemuck::cast_slice(loads[0]));
        q.write_buffer(&self.loads, bytes, bytemuck::cast_slice(loads[1]));
        q.write_buffer(&self.g, 0, bytemuck::cast_slice(prescribed[0]));
        q.write_buffer(&self.g, bytes, bytemuck::cast_slice(prescribed[1]));
        // The contact: the dispatches of all passes, no flags yet.
        if let Some(c) = &mut self.contact {
            c.dims.events_from = self.events_from;
            c.dims.arg_words = 3 * self.passes.len() as u32;
            q.write_buffer(&c.cdims, 0, bytemuck::bytes_of(&c.dims));
            let args: Vec<u32> = self
                .passes
                .iter()
                .flat_map(|p| [p.groups.0, p.groups.1, 1])
                .collect();
            q.write_buffer(&c.args, 0, bytemuck::cast_slice(&args));
            q.write_buffer(&c.flags, 0, bytemuck::cast_slice(&[0u32; FLAGS]));
        }
        let args = self.contact.as_ref().map(|c| &c.args);
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
                    p.dispatch(&mut pass, &[offset(k)], args);
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
                for p in &self.passes {
                    p.dispatch(&mut pass, &[offset(k)], args);
                }
            }
        }
        // The gate once more: whether the last time step opened or closed a contact.
        if let Some(c) = &self.contact {
            {
                let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                    label: Some("contact gate"),
                    timestamp_writes: None,
                });
                self.passes[c.passes[0]].dispatch(&mut pass, &[offset(steps.len())], None);
            }
            encoder.copy_buffer_to_buffer(&c.flags, 0, &c.flag_staging, 0, 4 * FLAGS as u64);
        }
        // The parameter buffer is written again by the next run: wait for this one first.
        self.gpu.queue.submit([encoder.finish()]);
        self.gpu.wait()?;
        if let Some(t) = &mut self.trace {
            let kinds: Vec<usize> = self.passes.iter().map(|p| p.kind).collect();
            let times = self.gpu.map(&t.read, |bytes| {
                let stamps: &[u64] = bytemuck::cast_slice(bytes);
                let mut times = [0.0; 5];
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
        let mut ran = Ran {
            steps: steps.len(),
            ..Default::default()
        };
        if let Some(c) = &self.contact {
            let flags: Vec<u32> = self.gpu.map(&c.flag_staging, |b| {
                bytemuck::cast_slice::<u8, u32>(b).to_vec()
            })?;
            if flags[2] != 0 && (flags[2] as usize) < steps.len() {
                ran.steps = flags[2] as usize;
                ran.search = flags[3] & 1 != 0;
            }
            ran.event = flags[4] != 0;
        }
        match self.gpu.take_error() {
            Some(e) => Err(e),
            None => Ok(ran),
        }
    }

    pub(crate) fn guard_reference(&mut self, reference: &[f32]) -> Result<(), String> {
        let Some(g) = &self.guard else {
            return Err("no watch over the time step on the card".to_owned());
        };
        if reference.len() != g.count {
            return Err(format!(
                "{} reference lengths for {} elements",
                reference.len(),
                g.count
            ));
        }
        self.gpu
            .write(&g.reference, 0, bytemuck::cast_slice(reference));
        Ok(())
    }

    pub(crate) fn departure(&mut self) -> Result<f32, String> {
        let Some(g) = &self.guard else {
            return Err("no watch over the time step on the card".to_owned());
        };
        self.gpu.write(&g.result, 0, bytemuck::bytes_of(&1.0_f32));
        let mut encoder = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("guard"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("guard"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&g.pipeline);
            pass.set_bind_group(0, &g.bind, &[]);
            pass.dispatch_workgroups(g.groups.0, g.groups.1, 1);
        }
        encoder.copy_buffer_to_buffer(&g.result, 0, &g.staging, 0, 4);
        self.gpu.queue.submit([encoder.finish()]);
        let bits = self
            .gpu
            .map(&g.staging, |b| bytemuck::cast_slice::<u8, u32>(&b[..4])[0])?;
        if let Some(e) = self.gpu.take_error() {
            return Err(e);
        }
        Ok(f32::from_bits(bits))
    }

    pub(crate) fn read(&mut self) -> Result<Reached, String> {
        let (n, r) = (self.n, self.r);
        let bytes = 8 * n as u64;
        let reduced = 8 * r as u64;
        let mut encoder = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("read"),
            });
        // u, v, the work per DOF, then q, q̇, a.
        encoder.copy_buffer_to_buffer(&self.u, 0, &self.staging, 0, bytes);
        encoder.copy_buffer_to_buffer(&self.v, 0, &self.staging, bytes, bytes);
        encoder.copy_buffer_to_buffer(&self.energy, 3 * bytes, &self.staging, 2 * bytes, 3 * bytes);
        // The rate of the prescribed displacements; the reactions and their work.
        encoder.copy_buffer_to_buffer(&self.g, 2 * bytes, &self.staging, 5 * bytes, bytes);
        if let Some(reaction) = &self.reaction {
            encoder.copy_buffer_to_buffer(reaction, 0, &self.staging, 6 * bytes, 2 * bytes);
        }
        // The internal and the viscous forces of the last step.
        encoder.copy_buffer_to_buffer(&self.energy, 0, &self.staging, 8 * bytes, bytes);
        encoder.copy_buffer_to_buffer(&self.energy, 2 * bytes, &self.staging, 9 * bytes, bytes);
        let at = 10 * bytes;
        encoder.copy_buffer_to_buffer(&self.q, 0, &self.staging, at, reduced);
        encoder.copy_buffer_to_buffer(&self.qd, 0, &self.staging, at + reduced, reduced);
        encoder.copy_buffer_to_buffer(&self.a, 0, &self.staging, at + 2 * reduced, reduced);
        self.gpu.queue.submit([encoder.finish()]);
        let prescribed = self.reaction.is_some();
        let mut reached = self.gpu.map(&self.staging, |bytes| {
            let data: &[f64] = bytemuck::cast_slice(bytes);
            let sum = |k: usize| data[k * n..(k + 1) * n].iter().sum::<f64>();
            let tail = &data[10 * n..];
            Reached {
                u: data[..n].to_vec(),
                v: data[n..2 * n].to_vec(),
                internal_work: sum(2),
                external_work: sum(3) + if prescribed { sum(7) } else { 0.0 },
                dissipation: sum(4),
                prescribed_rate: data[5 * n..6 * n].to_vec(),
                reactions: if prescribed {
                    data[6 * n..7 * n].to_vec()
                } else {
                    Vec::new()
                },
                q: tail[..r].to_vec(),
                qdot: tail[r..2 * r].to_vec(),
                a: tail[2 * r..3 * r].to_vec(),
                internal: data[8 * n..9 * n].to_vec(),
                viscous: data[9 * n..10 * n].to_vec(),
                contact: None,
            }
        })?;
        if let Some(e) = self.gpu.take_error() {
            return Err(e);
        }
        if let Some(c) = &self.contact
            && !c.contact.points.is_empty()
        {
            let bytes = (8 * STATE * c.contact.points.len()) as u64;
            let staging = self.gpu.staging("contact state", bytes);
            let mut encoder =
                self.gpu
                    .device
                    .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                        label: Some("read contact"),
                    });
            encoder.copy_buffer_to_buffer(&c.state, 0, &staging, 0, bytes);
            self.gpu.queue.submit([encoder.finish()]);
            let points = c.contact.points.len();
            let state = self.gpu.map(&staging, |b| {
                contact::state_of(&bytemuck::cast_slice::<u8, f64>(b)[..STATE * points])
            })?;
            reached.contact = Some(state);
        }
        if reached.u.iter().any(|x| !x.is_finite()) {
            return Err("the displacements on the graphics card are not finite".to_owned());
        }
        Ok(reached)
    }
}
