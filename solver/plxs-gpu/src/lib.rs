//! The graphics card as the solvers share it: the choice of the adapter, the device with
//! its error sink, the memory budget, and the small helpers every compute path needs
//! (buffers, uploads in pieces, bind group layouts, pipelines, read-back). The multigrid
//! cycle and the conjugate gradients (`plxs-linalg`) and the explicit time steps
//! (`plxs-explicit`) build on it.
//!
//! When the card is used is decided by the callers (`PLXS_GPU=off|on`, automatic); this
//! crate only opens what they ask for: any adapter (software renderers included, for tests
//! and measurements) or only a discrete graphics card, whose own memory and bandwidth are
//! what the solvers gain from.
//!
//! No CalculiX code was consulted.

use std::sync::{Arc, Mutex};

use wgpu::util::DeviceExt;

pub use wgpu;

/// Percent of the card's memory budget the solver's buffers may take, so that the user
/// interface (which draws on the same card with a device of its own) and the desktop keep
/// theirs: a card with 2 GB filled by a large model made the window's renderer fail and
/// the desktop stutter.
pub const MEMORY_SHARE_PERCENT: u8 = 60;

/// Bytes per piece of an upload ([`Gpu::write`]).
const PIECE: usize = 16 << 20;

/// Waits for a future of wgpu (adapter, device, error scope).
pub fn block_on<F: std::future::Future>(future: F) -> F::Output {
    pollster::block_on(future)
}

/// An adapter as the user reads it, e.g. "NVIDIA GeForce RTX 4070 (discrete)".
pub fn describe(info: &wgpu::AdapterInfo) -> String {
    let kind = match info.device_type {
        wgpu::DeviceType::DiscreteGpu => "discrete",
        wgpu::DeviceType::IntegratedGpu => "integrated",
        wgpu::DeviceType::VirtualGpu => "virtual",
        wgpu::DeviceType::Cpu => "software",
        wgpu::DeviceType::Other => "other",
    };
    format!("{} ({kind})", info.name)
}

fn instance(memory_share: bool) -> wgpu::Instance {
    let mut descriptor = wgpu::InstanceDescriptor::new_without_display_handle();
    descriptor.backends = wgpu::Backends::PRIMARY;
    if memory_share {
        descriptor.memory_budget_thresholds = wgpu::MemoryBudgetThresholds {
            for_resource_creation: Some(MEMORY_SHARE_PERCENT),
            for_device_loss: None,
        };
    }
    wgpu::Instance::new(descriptor)
}

/// What the solver finds for the graphics card, without opening it: for the settings.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Detection {
    /// Every adapter, with its kind, e.g. "NVIDIA GeForce RTX 4070 (discrete)".
    pub adapters: Vec<String>,
    /// The adapter the automatic mode would use: the first discrete one.
    pub chosen: Option<String>,
    /// Whether the chosen one computes in double precision, so that the conjugate
    /// gradients and the explicit time steps run on it too (otherwise only the multigrid
    /// cycle).
    pub double_precision: bool,
    /// The graphics API the chosen one is driven by (Vulkan, DirectX 12, Metal).
    pub backend: Option<String>,
}

/// Lists the graphics adapters and tells which one the automatic mode takes. Cheap
/// against opening a device, but still a driver call: run it off the user interface.
pub fn detect() -> Detection {
    let instance = instance(false);
    let adapters = block_on(instance.enumerate_adapters(wgpu::Backends::PRIMARY));
    let mut detection = Detection {
        adapters: adapters.iter().map(|a| describe(&a.get_info())).collect(),
        ..Default::default()
    };
    // Vulkan and DirectX list the same card twice.
    let mut seen = std::collections::HashSet::new();
    detection.adapters.retain(|a| seen.insert(a.clone()));
    if let Some(adapter) = adapters
        .iter()
        .find(|a| a.get_info().device_type == wgpu::DeviceType::DiscreteGpu)
    {
        let info = adapter.get_info();
        detection.chosen = Some(describe(&info));
        detection.double_precision = adapter.features().contains(wgpu::Features::SHADER_F64);
        detection.backend = Some(info.backend.to_str().to_owned());
    }
    detection
}

/// Double precision in the shaders.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Double {
    /// Without it the card is not opened.
    Required,
    /// Used when the adapter has it ([`Gpu::f64`]).
    IfAvailable,
}

/// What to open.
#[derive(Clone, Copy, Debug)]
pub struct Options {
    /// The device's label, e.g. "plxs multigrid".
    pub label: &'static str,
    /// Any adapter, software renderers included (tests, measurements); otherwise only a
    /// discrete graphics card.
    pub any_adapter: bool,
    pub double: Double,
    /// Timestamp queries, when the adapter has them ([`Gpu::timestamps`]).
    pub timestamps: bool,
}

/// An open device.
pub struct Gpu {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    /// The card, e.g. "NVIDIA GeForce RTX 4070 (discrete)".
    pub name: String,
    /// The limits the device was opened with: the defaults, with the largest buffers and
    /// the most storage buffers per shader the adapter supports.
    pub limits: wgpu::Limits,
    /// Whether the shaders compute in double precision.
    pub f64: bool,
    /// Whether timestamp queries are enabled.
    pub timestamps: bool,
    /// Errors the driver reported outside of error scopes.
    error: Arc<Mutex<Option<String>>>,
}

impl Gpu {
    /// Opens the adapter `options` ask for, the discrete one first. `Err` tells why there
    /// is none, for the output.
    pub fn open(options: Options) -> Result<Self, String> {
        let instance = instance(true);
        let adapters = block_on(instance.enumerate_adapters(wgpu::Backends::PRIMARY));
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
            .filter(|a| {
                options.any_adapter || a.get_info().device_type == wgpu::DeviceType::DiscreteGpu
            })
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
        let f64 = adapter.features().contains(wgpu::Features::SHADER_F64);
        if !f64 && options.double == Double::Required {
            return Err(format!("{name} computes no double precision"));
        }
        let timestamps =
            options.timestamps && adapter.features().contains(wgpu::Features::TIMESTAMP_QUERY);
        let supported = adapter.limits();
        let limits = wgpu::Limits {
            max_storage_buffer_binding_size: supported.max_storage_buffer_binding_size,
            max_buffer_size: supported.max_buffer_size,
            max_storage_buffers_per_shader_stage: supported.max_storage_buffers_per_shader_stage,
            ..Default::default()
        };
        let mut features = wgpu::Features::empty();
        if f64 {
            features |= wgpu::Features::SHADER_F64;
        }
        if timestamps {
            features |= wgpu::Features::TIMESTAMP_QUERY;
        }
        let (device, queue) = block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some(options.label),
            required_features: features,
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
        Ok(Self {
            device,
            queue,
            name,
            limits,
            f64,
            timestamps,
            error,
        })
    }

    /// The first error the driver reported outside of error scopes since the last call.
    pub fn take_error(&self) -> Option<String> {
        self.error.lock().unwrap_or_else(|e| e.into_inner()).take()
    }

    /// Runs `build` in a validation error scope: its error, prefixed with the card.
    pub fn validated<T>(&self, build: impl FnOnce() -> T) -> Result<T, String> {
        let scope = self.device.push_error_scope(wgpu::ErrorFilter::Validation);
        let out = build();
        match block_on(scope.pop()) {
            Some(e) => Err(format!("{}: {e}", self.name)),
            None => Ok(out),
        }
    }

    /// Whether a buffer of `bytes` can be created and bound.
    pub fn fits(&self, bytes: u64) -> bool {
        bytes <= self.limits.max_storage_buffer_binding_size && bytes <= self.limits.max_buffer_size
    }

    /// A storage buffer of `bytes` (at least 16), which copies may fill and read.
    pub fn storage(&self, label: &str, bytes: u64) -> wgpu::Buffer {
        self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: bytes.max(16),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        })
    }

    /// A storage buffer holding `contents` (padded to 16 bytes), uploaded in pieces.
    pub fn init(&self, label: &str, contents: &[u8]) -> wgpu::Buffer {
        if contents.len() <= PIECE {
            let padded;
            let contents = if contents.len() < 16 {
                padded = [contents, &[0u8; 16][..16 - contents.len()]].concat();
                &padded[..]
            } else {
                contents
            };
            return self
                .device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(label),
                    contents,
                    usage: wgpu::BufferUsages::STORAGE,
                });
        }
        let buffer = self.storage(label, contents.len().next_multiple_of(4) as u64);
        self.write(&buffer, 0, contents);
        buffer
    }

    /// A uniform buffer holding `contents`.
    pub fn uniform(&self, label: &str, contents: &[u8]) -> wgpu::Buffer {
        self.device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(label),
                contents,
                usage: wgpu::BufferUsages::UNIFORM,
            })
    }

    /// A buffer the processor reads back, of `bytes`.
    pub fn staging(&self, label: &str, bytes: u64) -> wgpu::Buffer {
        self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: bytes.max(16),
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    }

    /// Writes `bytes` to `buffer` from `offset` in pieces, each staged and submitted on
    /// its own, so that the staging memory stays small (a single write would stage the
    /// whole buffer at once, and the staging heap of a card can be much smaller than its
    /// memory). Uploads in pieces of 16 MB also suit the small staging heap of cards
    /// without resizable BAR.
    pub fn write(&self, buffer: &wgpu::Buffer, offset: u64, bytes: &[u8]) {
        for (i, piece) in bytes.chunks(PIECE).enumerate() {
            self.queue
                .write_buffer(buffer, offset + (i * PIECE) as u64, piece);
            if (i + 1) % 4 == 0 {
                self.queue.submit([]);
                let _ = self.device.poll(wgpu::PollType::wait_indefinitely());
            }
        }
        if bytes.len() > PIECE {
            self.queue.submit([]);
            let _ = self.device.poll(wgpu::PollType::wait_indefinitely());
        }
    }

    /// Waits until the card has done all submitted work.
    pub fn wait(&self) -> Result<(), String> {
        self.device
            .poll(wgpu::PollType::wait_indefinitely())
            .map(|_| ())
            .map_err(|e| e.to_string())
    }

    /// Maps `staging` (filled by submitted copies), hands its bytes to `read` and unmaps
    /// it again.
    pub fn map<T>(
        &self,
        staging: &wgpu::Buffer,
        read: impl FnOnce(&[u8]) -> T,
    ) -> Result<T, String> {
        let (tx, rx) = std::sync::mpsc::channel();
        staging.map_async(wgpu::MapMode::Read, .., move |result| {
            let _ = tx.send(result);
        });
        self.wait()?;
        rx.recv()
            .map_err(|e| e.to_string())?
            .map_err(|e| e.to_string())?;
        let out = {
            let view = staging.get_mapped_range(..).map_err(|e| e.to_string())?;
            read(&view)
        };
        staging.unmap();
        Ok(out)
    }

    /// A bind group layout for compute shaders with the bindings `uses` in order.
    pub fn layout(&self, label: &str, uses: &[Use]) -> wgpu::BindGroupLayout {
        let entries: Vec<wgpu::BindGroupLayoutEntry> = uses
            .iter()
            .enumerate()
            .map(|(binding, u)| wgpu::BindGroupLayoutEntry {
                binding: binding as u32,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: match u {
                        Use::Uniform | Use::Dynamic => wgpu::BufferBindingType::Uniform,
                        Use::Read => wgpu::BufferBindingType::Storage { read_only: true },
                        Use::Write => wgpu::BufferBindingType::Storage { read_only: false },
                    },
                    has_dynamic_offset: matches!(u, Use::Dynamic),
                    min_binding_size: None,
                },
                count: None,
            })
            .collect();
        self.device
            .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some(label),
                entries: &entries,
            })
    }

    /// The compute pipeline of `entry` in the WGSL `source` with one bind group of
    /// `layout`, its overridable `constants` set.
    pub fn pipeline(
        &self,
        source: &str,
        entry: &str,
        layout: &wgpu::BindGroupLayout,
        constants: &[(&str, f64)],
    ) -> wgpu::ComputePipeline {
        let module = self
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some(entry),
                source: wgpu::ShaderSource::Wgsl(source.into()),
            });
        let pipeline_layout = self
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some(entry),
                bind_group_layouts: &[Some(layout)],
                immediate_size: 0,
            });
        self.device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(entry),
                layout: Some(&pipeline_layout),
                module: &module,
                entry_point: Some(entry),
                compilation_options: wgpu::PipelineCompilationOptions {
                    constants,
                    zero_initialize_workgroup_memory: false,
                },
                cache: None,
            })
    }

    /// A bind group of `layout` with `resources` at the bindings 0, 1, ….
    pub fn bind(
        &self,
        layout: &wgpu::BindGroupLayout,
        resources: Vec<wgpu::BindingResource<'_>>,
    ) -> wgpu::BindGroup {
        let entries: Vec<wgpu::BindGroupEntry> = resources
            .into_iter()
            .enumerate()
            .map(|(binding, resource)| wgpu::BindGroupEntry {
                binding: binding as u32,
                resource,
            })
            .collect();
        self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout,
            entries: &entries,
        })
    }

    /// The work groups of `group` threads for `threads` threads, folded into two
    /// dimensions when one would exceed the limit: `(x, y)`; the shaders number a thread
    /// `(wid.y * x + wid.x) * group + local`.
    pub fn fold(&self, threads: usize, group: usize) -> (u32, u32) {
        let max_groups = self.limits.max_compute_workgroups_per_dimension.max(1) as usize;
        let groups = threads.div_ceil(group).max(1);
        let x = groups.min(max_groups);
        (x as u32, groups.div_ceil(x) as u32)
    }
}

/// How a binding of [`Gpu::layout`] is used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Use {
    Uniform,
    /// A uniform with a dynamic offset (e.g. one block of a parameter buffer per step).
    Dynamic,
    Read,
    Write,
}
