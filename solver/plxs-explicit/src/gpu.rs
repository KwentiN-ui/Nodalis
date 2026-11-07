//! The engine behind [`crate::Engine`] with the feature `gpu`.

use std::sync::{Arc, Mutex};

use wgpu::util::DeviceExt;

use crate::{Csr, MAX_CHUNK, Reached, Setup, Start};

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
    pad0: u32,
    pad1: u32,
}

pub(crate) struct Engine {
    pub(crate) name: String,
    device: wgpu::Device,
    queue: wgpu::Queue,
    n: usize,
    alpha: f64,
    groups: (u32, u32),
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
    stiffness: (wgpu::ComputePipeline, wgpu::BindGroup),
    viscous: (wgpu::ComputePipeline, wgpu::BindGroup),
    accelerate: (wgpu::ComputePipeline, wgpu::BindGroup),
    error: Arc<Mutex<Option<String>>>,
}

fn describe(info: &wgpu::AdapterInfo) -> String {
    let kind = match info.device_type {
        wgpu::DeviceType::DiscreteGpu => "discrete",
        wgpu::DeviceType::IntegratedGpu => "integrated",
        wgpu::DeviceType::VirtualGpu => "virtual",
        wgpu::DeviceType::Cpu => "software",
        wgpu::DeviceType::Other => "other",
    };
    format!("{} ({kind})", info.name)
}

/// How a binding is used.
#[derive(Clone, Copy)]
enum Use {
    Uniform,
    /// The step parameters, one block of the parameter buffer per step.
    Step,
    Read,
    Write,
}

fn layout(device: &wgpu::Device, label: &str, uses: &[Use]) -> wgpu::BindGroupLayout {
    let entries: Vec<wgpu::BindGroupLayoutEntry> = uses
        .iter()
        .enumerate()
        .map(|(binding, u)| wgpu::BindGroupLayoutEntry {
            binding: binding as u32,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: match u {
                    Use::Uniform | Use::Step => wgpu::BufferBindingType::Uniform,
                    Use::Read => wgpu::BufferBindingType::Storage { read_only: true },
                    Use::Write => wgpu::BufferBindingType::Storage { read_only: false },
                },
                has_dynamic_offset: matches!(u, Use::Step),
                min_binding_size: None,
            },
            count: None,
        })
        .collect();
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &entries,
    })
}

fn pipeline(
    device: &wgpu::Device,
    source: &'static str,
    entry: &str,
    layout: &wgpu::BindGroupLayout,
) -> wgpu::ComputePipeline {
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some(entry),
        source: wgpu::ShaderSource::Wgsl(source.into()),
    });
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some(entry),
        bind_group_layouts: &[Some(layout)],
        immediate_size: 0,
    });
    device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some(entry),
        layout: Some(&pipeline_layout),
        module: &module,
        entry_point: Some(entry),
        compilation_options: Default::default(),
        cache: None,
    })
}

impl Engine {
    pub(crate) fn new(setup: &Setup<'_>, any: bool) -> Result<Self, String> {
        let n = setup.inverse_mass.len();
        if n == 0 {
            return Err("no unknowns".to_owned());
        }
        let mut descriptor = wgpu::InstanceDescriptor::new_without_display_handle();
        descriptor.backends = wgpu::Backends::PRIMARY;
        let instance = wgpu::Instance::new(descriptor);
        let adapters = pollster::block_on(instance.enumerate_adapters(wgpu::Backends::PRIMARY));
        let rank = |t: wgpu::DeviceType| match t {
            wgpu::DeviceType::DiscreteGpu => 0,
            wgpu::DeviceType::IntegratedGpu => 1,
            wgpu::DeviceType::VirtualGpu => 2,
            wgpu::DeviceType::Other => 3,
            wgpu::DeviceType::Cpu => 4,
        };
        let found: Vec<String> = adapters.iter().map(|a| describe(&a.get_info())).collect();
        let adapter = adapters
            .into_iter()
            .filter(|a| any || a.get_info().device_type == wgpu::DeviceType::DiscreteGpu)
            .min_by_key(|a| rank(a.get_info().device_type))
            .ok_or_else(|| {
                if found.is_empty() {
                    "no graphics card found".to_owned()
                } else {
                    format!("no discrete graphics card (found {})", found.join(", "))
                }
            })?;
        let info = adapter.get_info();
        let name = describe(&info);
        if !adapter.features().contains(wgpu::Features::SHADER_F64) {
            return Err(format!("{name} computes no double precision"));
        }
        let supported = adapter.limits();
        let limits = wgpu::Limits {
            max_storage_buffer_binding_size: supported.max_storage_buffer_binding_size,
            max_buffer_size: supported.max_buffer_size,
            ..Default::default()
        };
        let largest = 8 * 6 * n as u64;
        let largest = largest
            .max(4 * setup.stiffness.val.len() as u64)
            .max(4 * setup.viscous.val.len() as u64);
        if largest > limits.max_storage_buffer_binding_size
            || largest > limits.max_buffer_size
        {
            return Err(format!("the model is too large for the buffers of {name}"));
        }
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("plxs explicit"),
            required_features: wgpu::Features::SHADER_F64,
            required_limits: limits.clone(),
            experimental_features: wgpu::ExperimentalFeatures::disabled(),
            memory_hints: wgpu::MemoryHints::Performance,
            trace: wgpu::Trace::Off,
        }))
        .map_err(|e| format!("{name}: {e}"))?;
        let error: Arc<Mutex<Option<String>>> = Arc::default();
        let sink = error.clone();
        device.on_uncaptured_error(Arc::new(move |e: wgpu::Error| {
            let mut slot = sink.lock().unwrap_or_else(|e| e.into_inner());
            slot.get_or_insert_with(|| e.to_string());
        }));
        let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);

        let storage = |label: &str, bytes: u64| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: bytes.max(16),
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            })
        };
        let init = |label: &str, contents: &[u8]| {
            let padded;
            let contents = if contents.len() < 16 {
                padded = [contents, &[0u8; 16][..16 - contents.len()]].concat();
                &padded[..]
            } else {
                contents
            };
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(label),
                contents,
                usage: wgpu::BufferUsages::STORAGE,
            })
        };
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
        let matrix = |csr: &Csr, label: &str| {
            (
                init(label, bytemuck::cast_slice(&csr.ptr)),
                init(label, bytemuck::cast_slice(&csr.col)),
                init(label, bytemuck::cast_slice(&csr.val)),
            )
        };
        let k = matrix(setup.stiffness, "stiffness");
        let c = matrix(setup.viscous, "viscous");
        let staging = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("staging"),
            size: 6 * f64s,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let stride = u64::from(device.limits().min_uniform_buffer_offset_alignment)
            .max(std::mem::size_of::<StepParams>() as u64);
        let params = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("steps"),
            size: stride * MAX_CHUNK as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let groups = n.div_ceil(GROUP);
        let max_groups = limits.max_compute_workgroups_per_dimension.max(1) as usize;
        let groups_x = groups.min(max_groups);
        let groups = (groups_x as u32, groups.div_ceil(groups_x) as u32);
        let dims = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("dims"),
            contents: bytemuck::bytes_of(&Dims {
                n: n as u32,
                groups_x: groups.0,
                pad0: 0,
                pad1: 0,
            }),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let step_binding = wgpu::BindingResource::Buffer(wgpu::BufferBinding {
            buffer: &params,
            offset: 0,
            size: wgpu::BufferSize::new(std::mem::size_of::<StepParams>() as u64),
        });
        let bind = |layout: &wgpu::BindGroupLayout, resources: Vec<wgpu::BindingResource>| {
            let entries: Vec<wgpu::BindGroupEntry> = resources
                .into_iter()
                .enumerate()
                .map(|(binding, resource)| wgpu::BindGroupEntry {
                    binding: binding as u32,
                    resource,
                })
                .collect();
            device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: None,
                layout,
                entries: &entries,
            })
        };
        use Use::*;
        let advance_layout = layout(
            &device,
            "advance",
            &[Uniform, Step, Write, Write, Read, Write, Read, Write, Read],
        );
        let advance = (
            pipeline(
                &device,
                include_str!("advance.wgsl"),
                "advance",
                &advance_layout,
            ),
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
        let product_layout = layout(
            &device,
            "product",
            &[Uniform, Read, Write, Read, Read, Read],
        );
        let product = |entry: &str, m: &(wgpu::Buffer, wgpu::Buffer, wgpu::Buffer)| {
            (
                pipeline(
                    &device,
                    include_str!("product.wgsl"),
                    entry,
                    &product_layout,
                ),
                bind(
                    &product_layout,
                    vec![
                        dims.as_entire_binding(),
                        dv.as_entire_binding(),
                        fs.as_entire_binding(),
                        m.0.as_entire_binding(),
                        m.1.as_entire_binding(),
                        m.2.as_entire_binding(),
                    ],
                ),
            )
        };
        let stiffness = product("stiffness", &k);
        let viscous = product("viscous", &c);
        let accelerate_layout = layout(
            &device,
            "accelerate",
            &[Uniform, Step, Read, Write, Read, Read, Read, Read, Write],
        );
        let accelerate = (
            pipeline(
                &device,
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
        if let Some(e) = pollster::block_on(scope.pop()) {
            return Err(format!("{name}: {e}"));
        }
        Ok(Self {
            name,
            device,
            queue,
            n,
            alpha: setup.alpha,
            groups,
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
            stiffness,
            viscous,
            accelerate,
            error,
        })
    }

    fn take_error(&self) -> Option<String> {
        self.error.lock().unwrap_or_else(|e| e.into_inner()).take()
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
        let q = &self.queue;
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
        let q = &self.queue;
        q.write_buffer(&self.params, 0, &params);
        let bytes = 8 * n as u64;
        q.write_buffer(&self.loads, 0, bytemuck::cast_slice(from));
        q.write_buffer(&self.loads, bytes, bytemuck::cast_slice(to));
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("explicit"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("explicit"),
                timestamp_writes: None,
            });
            for k in 0..steps.len() {
                let offset = (k as u64 * self.stride) as u32;
                pass.set_pipeline(&self.advance.0);
                pass.set_bind_group(0, &self.advance.1, &[offset]);
                pass.dispatch_workgroups(self.groups.0, self.groups.1, 1);
                for (pipeline, group) in [&self.stiffness, &self.viscous] {
                    pass.set_pipeline(pipeline);
                    pass.set_bind_group(0, group, &[]);
                    pass.dispatch_workgroups(self.groups.0, self.groups.1, 1);
                }
                pass.set_pipeline(&self.accelerate.0);
                pass.set_bind_group(0, &self.accelerate.1, &[offset]);
                pass.dispatch_workgroups(self.groups.0, self.groups.1, 1);
            }
        }
        // The parameter buffer is written again by the next run: wait for this one first.
        self.queue.submit([encoder.finish()]);
        self.device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|e| e.to_string())?;
        match self.take_error() {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    pub(crate) fn read(&mut self) -> Result<Reached, String> {
        let n = self.n;
        let bytes = 8 * n as u64;
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("read"),
            });
        encoder.copy_buffer_to_buffer(&self.u, 0, &self.staging, 0, bytes);
        encoder.copy_buffer_to_buffer(&self.v, 0, &self.staging, bytes, bytes);
        encoder.copy_buffer_to_buffer(&self.a, 0, &self.staging, 2 * bytes, bytes);
        encoder.copy_buffer_to_buffer(&self.energy, 3 * bytes, &self.staging, 3 * bytes, 3 * bytes);
        self.queue.submit([encoder.finish()]);
        let (tx, rx) = std::sync::mpsc::channel();
        self.staging
            .map_async(wgpu::MapMode::Read, .., move |result| {
                let _ = tx.send(result);
            });
        self.device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|e| e.to_string())?;
        rx.recv()
            .map_err(|e| e.to_string())?
            .map_err(|e| e.to_string())?;
        let reached = {
            let view = self
                .staging
                .get_mapped_range(..)
                .map_err(|e| e.to_string())?;
            let data: &[f64] = bytemuck::cast_slice(&view);
            let sum = |k: usize| data[k * n..(k + 1) * n].iter().sum::<f64>();
            Reached {
                u: data[..n].to_vec(),
                v: data[n..2 * n].to_vec(),
                a: data[2 * n..3 * n].to_vec(),
                internal_work: sum(3),
                external_work: sum(4),
                dissipation: sum(5),
            }
        };
        self.staging.unmap();
        if let Some(e) = self.take_error() {
            return Err(e);
        }
        if reached.u.iter().any(|x| !x.is_finite()) {
            return Err("the displacements on the graphics card are not finite".to_owned());
        }
        Ok(reached)
    }
}
