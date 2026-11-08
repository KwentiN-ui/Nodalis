//! The engine behind [`crate::Engine`] with the feature `gpu`.

use plxs_gpu::{Double, Gpu, Options, Use, wgpu};

use crate::sliced::{SLICE, Sliced};

// A workgroup of the product holds whole slices.
const _: () = assert!(GROUP.is_multiple_of(SLICE));
use crate::{MAX_CHUNK, Reached, Setup, Start};

const GROUP: usize = 256;

/// The parameters of one step as the shaders read them (`Step`).
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct StepParams {
    h: f64,
    c: f64,
    s: f64,
    alpha: f64,
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Dims {
    n: u32,
    groups_x: u32,
    positions: u32,
    pad: u32,
}

pub(crate) struct Engine {
    pub(crate) gpu: Gpu,
    n: usize,
    alpha: f64,
    groups: (u32, u32),
    product_groups: (u32, u32),
    stride: u64,
    params: wgpu::Buffer,
    u: wgpu::Buffer,
    v: wgpu::Buffer,
    a: wgpu::Buffer,
    uref: wgpu::Buffer,
    loads: wgpu::Buffer,
    energy: wgpu::Buffer,
    staging: wgpu::Buffer,
    advance: (wgpu::ComputePipeline, wgpu::BindGroup),
    product: (wgpu::ComputePipeline, wgpu::BindGroup),
    accelerate: (wgpu::ComputePipeline, wgpu::BindGroup),
    /// `PLXS_GPU_TRACE`: the time of every kind of dispatch on the card.
    trace: Option<Trace>,
}

/// The times the card took, from its timestamps: each dispatch of a traced run in a pass of
/// its own.
struct Trace {
    queries: wgpu::QuerySet,
    resolve: wgpu::Buffer,
    read: wgpu::Buffer,
    period: f64,
    /// Seconds in advance, product, accelerate; wall clock of the runs; time steps.
    kernels: [f64; 3],
    wall: f64,
    steps: usize,
}

/// Queries of one run: begin and end of three passes per time step.
const QUERIES: u32 = 6 * MAX_CHUNK as u32;

impl Drop for Engine {
    fn drop(&mut self) {
        if let Some(t) = &self.trace
            && t.steps > 0
        {
            let ms = |s: f64| 1e3 * s / t.steps as f64;
            eprintln!(
                " gpu trace: {} time steps, per step {:.3} ms wall clock, on the card: advance \
                 {:.3} ms, product {:.3} ms, accelerate {:.3} ms",
                t.steps,
                ms(t.wall),
                ms(t.kernels[0]),
                ms(t.kernels[1]),
                ms(t.kernels[2])
            );
        }
    }
}

/// A dispatch of the traced loop: pipeline and bind group, dynamic offsets, work groups.
type TimedPass<'a> = (
    &'a (wgpu::ComputePipeline, wgpu::BindGroup),
    &'a [u32],
    (u32, u32),
);

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
        let sliced = Sliced::new(setup.stiffness, setup.viscous);
        let largest = (8 * 6 * n as u64).max(8 * sliced.val.len() as u64);
        if !gpu.fits(largest) {
            return Err(format!(
                "the model is too large for the buffers of {}",
                gpu.name
            ));
        }
        let scope = gpu.device.push_error_scope(wgpu::ErrorFilter::Validation);
        let storage = |label: &str, bytes: u64| gpu.storage(label, bytes);
        let init = |label: &str, contents: &[u8]| gpu.init(label, contents);
        let f64s = 8 * n as u64;
        let u = storage("u", f64s);
        let v = storage("v", f64s);
        let a = storage("a", f64s);
        let du = storage("du", f64s);
        let uref = storage("uref", f64s);
        let dv = storage("dv", 2 * 4 * n as u64);
        let fs = storage("fs", 2 * 4 * n as u64);
        let loads = storage("loads", 3 * f64s);
        let energy = storage("energy", 6 * f64s);
        let im = init("inverse mass", bytemuck::cast_slice(setup.inverse_mass));
        let slice_start = init("slices", bytemuck::cast_slice(&sliced.start));
        let slice_rows = init("rows", bytemuck::cast_slice(&sliced.rows));
        let slice_col = init("columns", bytemuck::cast_slice(&sliced.col));
        let slice_val = init("values", bytemuck::cast_slice(&sliced.val));
        let positions = sliced.rows.len();
        drop(sliced);
        let staging = gpu.staging("staging", 6 * f64s);
        let device = &gpu.device;
        let stride = u64::from(gpu.limits.min_uniform_buffer_offset_alignment)
            .max(std::mem::size_of::<StepParams>() as u64);
        let params = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("steps"),
            size: stride * MAX_CHUNK as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let groups = gpu.fold(n, GROUP);
        let product_groups = gpu.fold(positions, GROUP);
        let dims_of = |groups: (u32, u32)| {
            gpu.uniform(
                "dims",
                bytemuck::bytes_of(&Dims {
                    n: n as u32,
                    groups_x: groups.0,
                    positions: positions as u32,
                    pad: 0,
                }),
            )
        };
        let dims = dims_of(groups);
        let product_dims = dims_of(product_groups);
        let step_binding = wgpu::BindingResource::Buffer(wgpu::BufferBinding {
            buffer: &params,
            offset: 0,
            size: wgpu::BufferSize::new(std::mem::size_of::<StepParams>() as u64),
        });
        let bind = |layout: &wgpu::BindGroupLayout, resources| gpu.bind(layout, resources);
        let layout = |label: &str, uses: &[Use]| gpu.layout(label, uses);
        let pipeline = |source: &str, entry: &str, layout: &wgpu::BindGroupLayout| {
            gpu.pipeline(source, entry, layout, &[])
        };
        use Use::*;
        let advance_layout = layout(
            "advance",
            &[
                Uniform, Dynamic, Write, Write, Read, Write, Read, Write, Read,
            ],
        );
        let advance = (
            pipeline(include_str!("advance.wgsl"), "advance", &advance_layout),
            bind(
                &advance_layout,
                vec![
                    dims.as_entire_binding(),
                    step_binding.clone(),
                    u.as_entire_binding(),
                    v.as_entire_binding(),
                    a.as_entire_binding(),
                    du.as_entire_binding(),
                    uref.as_entire_binding(),
                    dv.as_entire_binding(),
                    im.as_entire_binding(),
                ],
            ),
        );
        let product_layout = layout("product", &[Uniform, Read, Write, Read, Read, Read, Read]);
        let product = (
            pipeline(include_str!("product.wgsl"), "product", &product_layout),
            bind(
                &product_layout,
                vec![
                    product_dims.as_entire_binding(),
                    dv.as_entire_binding(),
                    fs.as_entire_binding(),
                    slice_start.as_entire_binding(),
                    slice_rows.as_entire_binding(),
                    slice_col.as_entire_binding(),
                    slice_val.as_entire_binding(),
                ],
            ),
        );
        let accelerate_layout = layout(
            "accelerate",
            &[Uniform, Dynamic, Read, Write, Read, Read, Read, Read, Write],
        );
        let accelerate = (
            pipeline(
                include_str!("accelerate.wgsl"),
                "accelerate",
                &accelerate_layout,
            ),
            bind(
                &accelerate_layout,
                vec![
                    dims.as_entire_binding(),
                    step_binding,
                    v.as_entire_binding(),
                    a.as_entire_binding(),
                    du.as_entire_binding(),
                    im.as_entire_binding(),
                    loads.as_entire_binding(),
                    fs.as_entire_binding(),
                    energy.as_entire_binding(),
                ],
            ),
        );
        let trace = gpu.timestamps.then(|| Trace {
            queries: device.create_query_set(&wgpu::QuerySetDescriptor {
                label: Some("trace"),
                ty: wgpu::QueryType::Timestamp,
                count: QUERIES,
            }),
            resolve: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("trace"),
                size: 8 * u64::from(QUERIES),
                usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            }),
            read: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("trace"),
                size: 8 * u64::from(QUERIES),
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }),
            period: f64::from(gpu.queue.get_timestamp_period()) * 1e-9,
            kernels: [0.0; 3],
            wall: 0.0,
            steps: 0,
        });
        if let Some(e) = plxs_gpu::block_on(scope.pop()) {
            return Err(format!("{}: {e}", gpu.name));
        }
        Ok(Self {
            gpu,
            n,
            alpha: setup.alpha,
            groups,
            product_groups,
            stride,
            params,
            u,
            v,
            a,
            uref,
            loads,
            energy,
            staging,
            advance,
            product,
            accelerate,
            trace,
        })
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
        q.write_buffer(&self.uref, 0, bytemuck::cast_slice(start.u));
        q.write_buffer(&self.v, 0, bytemuck::cast_slice(start.v));
        q.write_buffer(&self.a, 0, bytemuck::cast_slice(start.a));
        let bytes = 8 * n as u64;
        q.write_buffer(&self.loads, 0, bytemuck::cast_slice(start.loads));
        q.write_buffer(&self.loads, bytes, bytemuck::cast_slice(start.loads));
        q.write_buffer(&self.loads, 2 * bytes, bytemuck::cast_slice(start.internal));
        let mut energy = Vec::with_capacity(6 * n);
        energy.extend_from_slice(start.internal);
        energy.extend_from_slice(start.loads);
        energy.extend_from_slice(start.viscous);
        energy.resize(6 * n, 0.0);
        q.write_buffer(&self.energy, 0, bytemuck::cast_slice(&energy));
        Ok(())
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
        if let Some(t) = &self.trace {
            // Every dispatch in a pass of its own, timed by the card.
            for k in 0..steps.len() {
                let offset = (k as u64 * self.stride) as u32;
                let passes: [TimedPass<'_>; 3] = [
                    (&self.advance, &[offset], self.groups),
                    (&self.product, &[], self.product_groups),
                    (&self.accelerate, &[offset], self.groups),
                ];
                for (j, (stage, offsets, groups)) in passes.into_iter().enumerate() {
                    let at = (6 * k + 2 * j) as u32;
                    let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                        label: Some("traced"),
                        timestamp_writes: Some(wgpu::ComputePassTimestampWrites {
                            query_set: &t.queries,
                            beginning_of_pass_write_index: Some(at),
                            end_of_pass_write_index: Some(at + 1),
                        }),
                    });
                    pass.set_pipeline(&stage.0);
                    pass.set_bind_group(0, &stage.1, offsets);
                    pass.dispatch_workgroups(groups.0, groups.1, 1);
                }
            }
            let count = 6 * steps.len() as u32;
            encoder.resolve_query_set(&t.queries, 0..count, &t.resolve, 0);
            encoder.copy_buffer_to_buffer(&t.resolve, 0, &t.read, 0, 8 * u64::from(count));
        } else {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("explicit"),
                timestamp_writes: None,
            });
            for k in 0..steps.len() {
                let offset = (k as u64 * self.stride) as u32;
                pass.set_pipeline(&self.advance.0);
                pass.set_bind_group(0, &self.advance.1, &[offset]);
                pass.dispatch_workgroups(self.groups.0, self.groups.1, 1);
                pass.set_pipeline(&self.product.0);
                pass.set_bind_group(0, &self.product.1, &[]);
                pass.dispatch_workgroups(self.product_groups.0, self.product_groups.1, 1);
                pass.set_pipeline(&self.accelerate.0);
                pass.set_bind_group(0, &self.accelerate.1, &[offset]);
                pass.dispatch_workgroups(self.groups.0, self.groups.1, 1);
            }
        }
        // The parameter buffer is written again by the next run: wait for this one first.
        self.gpu.queue.submit([encoder.finish()]);
        self.gpu.wait()?;
        if let Some(t) = &mut self.trace {
            let count = 6 * steps.len();
            let times = self.gpu.map(&t.read, |bytes| {
                let stamps: &[u64] = bytemuck::cast_slice(bytes);
                let mut times = [0.0; 3];
                for k in 0..count / 2 {
                    let ticks = stamps[2 * k + 1].saturating_sub(stamps[2 * k]);
                    times[k % 3] += ticks as f64 * t.period;
                }
                times
            })?;
            for (sum, time) in t.kernels.iter_mut().zip(times) {
                *sum += time;
            }
            t.wall += started.elapsed().as_secs_f64();
            t.steps += steps.len();
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
