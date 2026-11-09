//! Prints what the solver finds for the graphics card and its memory, to check the
//! memory share on a real card:
//!
//! ```text
//! cargo run --release -p plxs-gpu --example memory
//! ```
//!
//! Then it creates a storage buffer of 1 GB (or what the share allows), uploads 256 MB
//! in pieces and reads 64 MB back, the way the multigrid cycle does, and reports every
//! error the driver raised.

use plxs_gpu::{Double, Gpu, MEMORY_SHARE_PERCENT, Options, wgpu};

fn main() {
    let detection = plxs_gpu::detect();
    println!("adapters: {}", detection.adapters.join(", "));
    println!(
        "automatic mode takes: {} ({}, double precision: {})",
        detection.chosen.as_deref().unwrap_or("none"),
        detection.backend.as_deref().unwrap_or("-"),
        detection.double_precision
    );
    let gpu = match Gpu::open(Options {
        label: "memory example",
        any_adapter: std::env::var("PLXS_GPU").as_deref() == Ok("on"),
        double: Double::IfAvailable,
        timestamps: false,
        memory_share: true,
    }) {
        Ok(gpu) => gpu,
        Err(reason) => {
            println!("no device: {reason}");
            return;
        }
    };
    let mb = |b: u64| b as f64 / 1048576.0;
    println!("opened: {}", gpu.name);
    match gpu.memory() {
        Some(memory) => println!(
            "memory: {:.0} MB total, {:.0} MB granted to this process, {:.0} MB in use, \
             {:.0} MB free for the solver ({MEMORY_SHARE_PERCENT} %)",
            mb(memory.total),
            mb(memory.budget),
            mb(memory.used),
            mb(memory.free_share())
        ),
        None => println!("memory: the graphics API does not tell (no share applied)"),
    }
    let wanted = 1u64 << 30;
    let size = match gpu.reserve(wanted) {
        Ok(()) => wanted,
        Err(reason) => {
            println!("1024 MB refused: {reason}");
            gpu.memory().map(|m| m.free_share() / 2).unwrap_or(64 << 20)
        }
    };
    let size = size.min(gpu.limits.max_buffer_size).max(1 << 20);
    let storage = gpu.storage("example storage", size);
    println!("storage buffer of {:.0} MB created", mb(size));
    let upload = vec![1u8; (256u64 << 20).min(size) as usize];
    gpu.write(&storage, 0, &upload);
    println!("{:.0} MB uploaded in pieces", mb(upload.len() as u64));
    let read = (64u64 << 20).min(size);
    let staging = gpu.staging("example read-back", read);
    let mut encoder = gpu
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
    encoder.copy_buffer_to_buffer(&storage, 0, &staging, 0, read);
    gpu.queue.submit([encoder.finish()]);
    let slice = staging.slice(..);
    slice.map_async(wgpu::MapMode::Read, |_| {});
    if let Err(e) = gpu.wait() {
        println!("waiting failed: {e}");
    }
    match slice.get_mapped_range() {
        Ok(view) => println!(
            "{:.0} MB read back (first byte {}, expected 1)",
            mb(read),
            view[0]
        ),
        Err(e) => println!("read-back failed: {e}"),
    }
    match gpu.take_error() {
        Some(e) => println!("DRIVER ERROR: {e}"),
        None => println!("no driver error: the card is usable with the memory share"),
    }
}
