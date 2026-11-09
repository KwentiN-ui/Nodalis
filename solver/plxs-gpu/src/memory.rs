//! The card's own memory as the graphics API reports it (Vulkan: memory heaps and
//! `VK_EXT_memory_budget`; DirectX 12: `QueryVideoMemoryInfo`), read through wgpu-hal. The
//! only place of this crate that steps outside of wgpu: the handles come from wgpu's
//! live instance and device and are only read, never freed or written.
#![allow(unsafe_code)]

use crate::Memory;

/// Reads the card's memory through the graphics API; `None` when the API does not tell
/// (Metal, DirectX 12).
pub(crate) enum MemoryQuery {
    Vulkan {
        instance: ash::Instance,
        physical_device: ash::vk::PhysicalDevice,
        /// `VK_EXT_memory_budget` is enabled and the instance speaks Vulkan 1.1.
        budget: bool,
    },
    /// DirectX 12: the adapter is asked at every read.
    #[cfg(windows)]
    Dx12 { adapter: wgpu::Adapter },
}

// The Vulkan handles are plain pointers and dispatch tables; the instance outlives the
// device this is read from, and the query is a pure read of the driver.
unsafe impl Send for MemoryQuery {}
unsafe impl Sync for MemoryQuery {}

impl MemoryQuery {
    pub(crate) fn new(adapter: &wgpu::Adapter, device: &wgpu::Device) -> Option<Self> {
        #[cfg(windows)]
        // SAFETY: only the presence of the DirectX adapter is checked here.
        if unsafe { adapter.as_hal::<wgpu::hal::api::Dx12>() }.is_some() {
            return Some(Self::Dx12 {
                adapter: adapter.clone(),
            });
        }
        #[cfg(not(windows))]
        let _ = adapter;
        // SAFETY: the guard is only read while it lives, and the handles it hands out are
        // owned by the instance wgpu keeps alive as long as the device.
        let hal = unsafe { device.as_hal::<wgpu::hal::api::Vulkan>() }?;
        let shared = hal.shared_instance();
        let budget = shared.instance_api_version() >= ash::vk::API_VERSION_1_1
            && hal
                .enabled_device_extensions()
                .contains(&ash::ext::memory_budget::NAME);
        Some(Self::Vulkan {
            instance: shared.raw_instance().clone(),
            physical_device: hal.raw_physical_device(),
            budget,
        })
    }

    pub(crate) fn read(&self) -> Option<Memory> {
        match self {
            Self::Vulkan {
                instance,
                physical_device,
                budget,
            } => {
                use ash::vk;
                let mut budget_props = vk::PhysicalDeviceMemoryBudgetPropertiesEXT::default();
                // SAFETY: the physical device belongs to the instance, both alive.
                let props = unsafe {
                    if *budget {
                        let mut props = vk::PhysicalDeviceMemoryProperties2::default()
                            .push_next(&mut budget_props);
                        instance
                            .get_physical_device_memory_properties2(*physical_device, &mut props);
                        props.memory_properties
                    } else {
                        instance.get_physical_device_memory_properties(*physical_device)
                    }
                };
                let count = props.memory_heap_count as usize;
                // The largest heap of the card's own memory, where the storage buffers
                // land; the small host-visible one of cards without resizable BAR is
                // also device local but never the largest.
                let (i, heap) = props.memory_heaps[..count]
                    .iter()
                    .enumerate()
                    .filter(|(_, h)| h.flags.contains(vk::MemoryHeapFlags::DEVICE_LOCAL))
                    .max_by_key(|(_, h)| h.size)?;
                let (budget_bytes, used) = if *budget && budget_props.heap_budget[i] > 0 {
                    (budget_props.heap_budget[i], budget_props.heap_usage[i])
                } else {
                    (heap.size, 0)
                };
                Some(Memory {
                    total: heap.size,
                    budget: budget_bytes.min(heap.size),
                    used,
                })
            }
            #[cfg(windows)]
            Self::Dx12 { adapter } => {
                // SAFETY: the guard lives for this read only; the adapter is wgpu's.
                let hal = unsafe { adapter.as_hal::<wgpu::hal::api::Dx12>() }?;
                let adapter = hal.raw_adapter();
                // The local segment group (the card's own memory) is the zero value of
                // `DXGI_MEMORY_SEGMENT_GROUP`, which spares a dependency on `windows`.
                let info = adapter.query_video_memory_info(Default::default()).ok()?;
                // SAFETY: a plain query on a live adapter.
                let total = unsafe { adapter.GetDesc1() }
                    .map(|desc| desc.DedicatedVideoMemory as u64)
                    .unwrap_or(info.Budget);
                let total = total.max(info.Budget);
                Some(Memory {
                    total,
                    budget: info.Budget.min(total),
                    used: info.CurrentUsage,
                })
            }
        }
    }
}
