//! Screenshot of the 3D view with its annotations, or of another part of the window such as a
//! plot, copied to the clipboard or saved as PNG.
//!
//! egui captures the whole window; the image is cropped to the view afterwards. The capture
//! is requested one frame after the click so that the closing menu is no longer painted.
//!
//! eframe carries out the screenshot command only for the root window: in the window of an
//! analysis, an immediate viewport, [`ImmediateCapture`] takes the capture instead.

use std::path::Path;
use std::sync::Arc;

use egui::{
    ColorImage, Context, FullOutput, RawInput, Rect, UserData, ViewportCommand, ViewportId,
};
use egui_wgpu::{RenderState, wgpu};

/// Where a screenshot goes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    Clipboard,
    File,
}

/// A capture: where it goes and the part of the window it shows.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Request {
    target: Target,
    /// Part of the window in points; `None` is the 3D view.
    region: Option<Rect>,
}

#[derive(Default)]
pub struct Screenshot {
    /// Requested capture, sent with the next frame.
    pending: Option<Request>,
    /// The pass the last capture was sent in.
    sent: Option<u64>,
}

impl Screenshot {
    pub fn request(&mut self, target: Target) {
        self.pending = Some(Request {
            target,
            region: None,
        });
    }

    /// Whether a capture is under way: what should not show in it, such as the floating
    /// bar, stays hidden.
    pub fn capturing(&self, ctx: &Context) -> bool {
        self.pending.is_some()
            || (self.sent).is_some_and(|pass| ctx.cumulative_pass_nr() <= pass + 2)
    }

    /// A capture of `region` of the window, in points, instead of the 3D view.
    pub fn request_region(&mut self, target: Target, region: Rect) {
        self.pending = Some(Request {
            target,
            region: Some(region),
        });
    }

    /// Sends a pending request and delivers captured images; `view` is the 3D view in points.
    pub fn update(&mut self, ctx: &Context, view: Rect, output: &mut Vec<String>) {
        let captured: Vec<_> = ctx.input(|i| {
            i.raw
                .events
                .iter()
                .filter_map(|event| match event {
                    egui::Event::Screenshot {
                        user_data, image, ..
                    } => user_data
                        .data
                        .as_ref()
                        .and_then(|d| d.downcast_ref::<Request>())
                        .map(|request| (*request, image.clone())),
                    _ => None,
                })
                .collect()
        });
        for (request, image) in captured {
            let image = crop(
                &image,
                request.region.unwrap_or(view),
                ctx.pixels_per_point(),
            );
            let name = if request.region.is_some() {
                "plot"
            } else {
                "screenshot"
            };
            deliver(ctx, request.target, image, name, output);
        }
        if let Some(request) = self.pending.take() {
            ctx.send_viewport_cmd(ViewportCommand::Screenshot(UserData::new(request)));
            ctx.request_repaint();
            self.sent = Some(ctx.cumulative_pass_nr());
        }
    }
}

/// The part of the window image covered by `rect`, clamped to the image.
pub fn crop(image: &ColorImage, rect: Rect, pixels_per_point: f32) -> ColorImage {
    let [width, height] = image.size;
    let px = |v: f32, max: usize| ((v * pixels_per_point).round().max(0.0) as usize).min(max);
    let (x0, y0) = (px(rect.min.x, width), px(rect.min.y, height));
    let (x1, y1) = (px(rect.max.x, width), px(rect.max.y, height));
    image.region_by_pixels([x0, y0], [x1 - x0, y1 - y0])
}

/// Copies or saves the image; `name` is what it shows, lower case.
fn deliver(ctx: &Context, target: Target, image: ColorImage, name: &str, output: &mut Vec<String>) {
    let [width, height] = image.size;
    let mut chars = name.chars();
    let title = chars
        .next()
        .map(|c| c.to_uppercase().chain(chars).collect::<String>())
        .unwrap_or_default();
    match target {
        Target::Clipboard => {
            ctx.copy_image(image);
            output.push(format!(
                "{title} ({width} x {height}) copied to the clipboard"
            ));
        }
        Target::File => {
            let Some(path) = rfd::FileDialog::new()
                .set_title(format!("Save {title} As"))
                .add_filter("PNG image", &["png"])
                .set_file_name(format!("{name}.png"))
                .save_file()
            else {
                return;
            };
            let path = if path.extension().is_none() {
                path.with_extension("png")
            } else {
                path
            };
            match write_png(&path, &image) {
                Ok(()) => output.push(format!("{title} saved: {}", path.display())),
                Err(err) => output.push(format!(
                    "{title} could not be saved ({}): {err}",
                    path.display()
                )),
            }
        }
    }
}

fn write_png(path: &Path, image: &ColorImage) -> Result<(), Box<dyn std::error::Error>> {
    let file = std::io::BufWriter::new(std::fs::File::create(path)?);
    let mut encoder = png::Encoder::new(file, image.size[0] as u32, image.size[1] as u32);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    let rgba: Vec<u8> = image
        .pixels
        .iter()
        .flat_map(|c| c.to_srgba_unmultiplied())
        .collect();
    writer.write_image_data(&rgba)?;
    writer.finish()?;
    Ok(())
}

/// Takes the screenshots of immediate viewports, which eframe leaves undone.
///
/// The plugin takes their screenshot commands out of the output, renders the next frame of
/// the viewport into a texture of its own and hands the image to the viewport's next input as
/// [`egui::Event::Screenshot`], just like eframe does for the root window.
pub struct ImmediateCapture {
    render_state: RenderState,
    /// The viewport whose pass is ending, with its size in pixels.
    ending: Option<(ViewportId, [u32; 2])>,
    /// Captures to take from the next frame of a viewport.
    requested: Vec<(ViewportId, UserData)>,
    /// Images to hand to the next input of their viewport.
    captured: Vec<(ViewportId, UserData, Arc<ColorImage>)>,
}

impl ImmediateCapture {
    pub fn new(render_state: RenderState) -> Self {
        Self {
            render_state,
            ending: None,
            requested: Vec::new(),
            captured: Vec::new(),
        }
    }

    /// The window as `output` paints it, `size` in pixels.
    fn render(
        &self,
        ctx: &Context,
        output: &FullOutput,
        size: [u32; 2],
    ) -> Result<ColorImage, String> {
        let RenderState {
            device,
            queue,
            renderer,
            target_format,
            ..
        } = &self.render_state;
        let order = match target_format {
            wgpu::TextureFormat::Rgba8Unorm | wgpu::TextureFormat::Rgba8UnormSrgb => [0, 1, 2, 3],
            wgpu::TextureFormat::Bgra8Unorm | wgpu::TextureFormat::Bgra8UnormSrgb => [2, 1, 0, 3],
            format => return Err(format!("unsupported surface format {format:?}")),
        };
        let [width, height] = size;
        let primitives = ctx.tessellate(output.shapes.clone(), output.pixels_per_point);
        let screen = egui_wgpu::ScreenDescriptor {
            size_in_pixels: size,
            pixels_per_point: output.pixels_per_point,
        };
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("nodalis_screenshot"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: *target_format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let view = texture.create_view(&Default::default());
        let mut encoder = device.create_command_encoder(&Default::default());
        let mut renderer = renderer.write();
        // eframe uploads the textures of this frame only when it paints it, after this.
        for (id, deltas) in &output.textures_delta.set {
            for delta in deltas {
                renderer.update_texture(device, queue, *id, delta);
            }
        }
        let mut commands =
            renderer.update_buffers(device, queue, &mut encoder, &primitives, &screen);
        let [r, g, b, _] = ctx.global_style().visuals.panel_fill.to_array();
        let mut pass = encoder
            .begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("nodalis_screenshot"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: f64::from(r) / 255.0,
                            g: f64::from(g) / 255.0,
                            b: f64::from(b) / 255.0,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                    depth_slice: None,
                })],
                ..Default::default()
            })
            .forget_lifetime();
        renderer.render(&mut pass, &primitives, &screen);
        drop(pass);
        drop(renderer);
        let row = (width * 4).div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
            * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("nodalis_screenshot"),
            size: u64::from(row * height),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        encoder.copy_texture_to_buffer(
            texture.as_image_copy(),
            wgpu::TexelCopyBufferInfo {
                buffer: &buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(row),
                    rows_per_image: None,
                },
            },
            texture.size(),
        );
        commands.push(encoder.finish());
        queue.submit(commands);
        let (tx, rx) = std::sync::mpsc::channel();
        buffer
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |result| {
                tx.send(result).ok();
            });
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|err| err.to_string())?;
        rx.recv()
            .map_err(|err| err.to_string())?
            .map_err(|err| err.to_string())?;
        let slice = buffer.slice(..);
        let data = slice.get_mapped_range().map_err(|err| err.to_string())?;
        let image = pixels(&data, size, row as usize, order);
        drop(data);
        buffer.unmap();
        Ok(image)
    }
}

impl egui::Plugin for ImmediateCapture {
    fn debug_name(&self) -> &'static str {
        "ImmediateCapture"
    }

    fn on_end_pass(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx();
        let size = ctx.input(|i| i.viewport_rect()).size() * ctx.pixels_per_point();
        let size = [size.x.round() as u32, size.y.round() as u32];
        self.ending = Some((ctx.viewport_id(), size));
    }

    fn output_hook(&mut self, ctx: &Context, output: &mut FullOutput) {
        let ending = self.ending.take();
        self.requested.extend(take_immediate_screenshots(output));
        let Some((id, size)) = ending else {
            return;
        };
        if size.contains(&0) || !self.requested.iter().any(|(r, _)| *r == id) {
            return;
        }
        match self.render(ctx, output, size) {
            Ok(image) => {
                let image = Arc::new(image);
                for (r, data) in std::mem::take(&mut self.requested) {
                    if r == id {
                        self.captured.push((id, data, Arc::clone(&image)));
                    } else {
                        self.requested.push((r, data));
                    }
                }
                ctx.request_repaint_of(id);
            }
            Err(err) => {
                log::error!("Screenshot failed: {err}");
                self.requested.retain(|(r, _)| *r != id);
            }
        }
    }

    fn input_hook(&mut self, _ctx: &Context, input: &mut RawInput) {
        let id = input.viewport_id;
        for (r, user_data, image) in std::mem::take(&mut self.captured) {
            if r == id {
                input.events.push(egui::Event::Screenshot {
                    viewport_id: id,
                    user_data,
                    image,
                });
            } else {
                self.captured.push((r, user_data, image));
            }
        }
    }
}

/// Removes the screenshot commands of immediate viewports from `output`, which eframe would
/// keep forever without carrying them out.
fn take_immediate_screenshots(output: &mut FullOutput) -> Vec<(ViewportId, UserData)> {
    let mut taken = Vec::new();
    for (&id, viewport) in &mut output.viewport_output {
        // egui leaves the class of immediate viewports at its default; unlike deferred ones,
        // they have no callback of their own.
        if id == ViewportId::ROOT || viewport.viewport_ui_cb.is_some() {
            continue;
        }
        viewport.commands.retain(|command| match command {
            ViewportCommand::Screenshot(data) => {
                taken.push((id, data.clone()));
                false
            }
            _ => true,
        });
    }
    taken
}

/// The image in `data`, rows of `row` bytes, with the channels of a pixel in `order`.
fn pixels(data: &[u8], [width, height]: [u32; 2], row: usize, order: [usize; 4]) -> ColorImage {
    let pixels = data
        .chunks(row)
        .take(height as usize)
        .flat_map(|line| line[..width as usize * 4].as_chunks::<4>().0)
        .map(|c| {
            egui::Color32::from_rgba_premultiplied(
                c[order[0]],
                c[order[1]],
                c[order[2]],
                c[order[3]],
            )
        })
        .collect();
    ColorImage::new([width as usize, height as usize], pixels)
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::{Color32, ViewportClass, pos2};

    #[test]
    fn crop_is_clamped_and_scaled() {
        let mut image = ColorImage::filled([10, 8], Color32::WHITE);
        image.pixels[2 * 10 + 4] = Color32::RED;
        let part = crop(
            &image,
            Rect::from_min_max(pos2(2.0, 1.0), pos2(100.0, 100.0)),
            2.0,
        );
        assert_eq!(part.size, [6, 6]);
        assert_eq!(part.pixels[0], Color32::RED);
    }

    #[test]
    fn takes_screenshots_of_immediate_viewports_only() {
        let analysis = ViewportId::from_hash_of("analysis");
        let table = ViewportId::from_hash_of("table");
        let callback: Arc<egui::DeferredViewportUiCallback> = Arc::new(|_| {});
        let mut output = FullOutput::default();
        for (id, viewport_ui_cb) in [
            (ViewportId::ROOT, None),
            (analysis, None),
            (table, Some(callback)),
        ] {
            output.viewport_output.insert(
                id,
                egui::ViewportOutput {
                    parent: ViewportId::ROOT,
                    class: ViewportClass::Root,
                    builder: Default::default(),
                    viewport_ui_cb,
                    commands: vec![
                        ViewportCommand::Screenshot(UserData::new(1_u8)),
                        ViewportCommand::Focus,
                    ],
                    repaint_delay: std::time::Duration::ZERO,
                },
            );
        }
        let taken = take_immediate_screenshots(&mut output);
        assert_eq!(taken.len(), 1);
        assert_eq!(taken[0].0, analysis);
        assert_eq!(output.viewport_output[&analysis].commands.len(), 1);
        assert_eq!(output.viewport_output[&ViewportId::ROOT].commands.len(), 2);
        assert_eq!(output.viewport_output[&table].commands.len(), 2);
    }

    #[test]
    fn reads_padded_bgra_rows() {
        // Two pixels per row, padded to three.
        let data = [
            3, 2, 1, 255, 0, 0, 0, 255, 9, 9, 9, 9, //
            0, 0, 255, 255, 0, 255, 0, 255, 9, 9, 9, 9,
        ];
        let image = pixels(&data, [2, 2], 12, [2, 1, 0, 3]);
        assert_eq!(image.size, [2, 2]);
        assert_eq!(image.pixels[0], Color32::from_rgb(1, 2, 3));
        assert_eq!(image.pixels[2], Color32::RED);
        assert_eq!(image.pixels[3], Color32::from_rgb(0, 255, 0));
    }

    #[test]
    fn writes_png() {
        let path = std::env::temp_dir().join(format!("plx-screenshot-{}.png", std::process::id()));
        write_png(&path, &ColorImage::filled([3, 2], Color32::RED)).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        std::fs::remove_file(&path).unwrap();
        assert_eq!(&bytes[1..4], b"PNG");
        assert_eq!(&bytes[16..24], &[0, 0, 0, 3, 0, 0, 0, 2]);
    }
}
