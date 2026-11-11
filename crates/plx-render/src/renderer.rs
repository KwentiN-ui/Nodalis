use bytemuck::{Pod, Zeroable};
use wgpu::util::DeviceExt;

use crate::camera::Camera;
use crate::contour::PALETTE_SLOTS;
use crate::mesh::{Directions, RenderMesh, Vertex};
use crate::section::ClipPlane;

pub const COLOR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
/// The surfaces' projected direction, its mask and the noise, for the LIC pass.
const LIC_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;
const SAMPLE_COUNT: u32 = 4;
/// How far surfaces are pushed back behind their edges, in pixels along the surface: the
/// slope-scaled depth bias moves a face by its depth change over this many pixels. A face seen
/// head on hardly moves, so the edges on the far side of a thin wall stay hidden however far
/// the view is zoomed out; a fixed offset in pixels reached through walls thinner than it.
const EDGE_OFFSET_PX: f32 = 1.5;
/// Constant part of the surface depth bias, in steps of the depth buffer: covers the rounding
/// between a line and the triangle it lies on where the face is seen head on.
const SURFACE_BIAS_CONSTANT: i32 = 2;
/// Width of [`RenderMesh::wide_edges`] in pixels: the outline of a selected part.
pub const WIDE_EDGE_PX: f32 = 3.0;

/// The background gradient: Gainsboro at the top, WhiteSmoke at the bottom.
pub const BACKGROUND: [[f32; 4]; 2] = [
    [220.0 / 255.0, 220.0 / 255.0, 220.0 / 255.0, 1.0],
    [245.0 / 255.0, 245.0 / 255.0, 245.0 / 255.0, 1.0],
];

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Globals {
    view_proj: [[f32; 4]; 4],
    lights: [[f32; 4]; 3],
    view_dir: [f32; 4],
    background_top: [f32; 4],
    background_bottom: [f32; 4],
    /// x, y: target size in pixels; z: width of wide edges in pixels; w: their depth offset
    /// towards the viewer in normalized depth units.
    edge: [f32; 4],
    /// x: number of contour bands, 0 when surfaces show their part colour.
    contour: [f32; 4],
    /// Section view plane: unit normal and `normal · point`.
    clip: [f32; 4],
    /// x: 1 when the section view is on, y: how far behind the plane surfaces still show.
    clip_options: [f32; 4],
    /// Colour of the wireframe of the undeformed shape in rgb.
    wireframe: [f32; 4],
    /// x: LIC source, 0 none, 1 vector, 2 tensor; y: principal value of a tensor, 0 to 2
    /// from the largest, 3 the largest in magnitude; z: grain size in world units; w: size of a pixel in world units.
    lic: [f32; 4],
    /// x: streak length in pixels, y: contrast, z: mix.
    lic_params: [f32; 4],
    palette: [[f32; 4]; PALETTE_SLOTS as usize],
}

/// A vertex or index buffer with its element count; `None` when there is nothing to draw.
struct GpuBuffer {
    buffer: wgpu::Buffer,
    count: u32,
}

struct GpuMesh {
    vertices: Option<GpuBuffer>,
    triangles: Option<GpuBuffer>,
    feature_edges: Option<GpuBuffer>,
    mesh_edges: Option<GpuBuffer>,
    wireframe_edges: Option<GpuBuffer>,
    wide_edges: Option<GpuBuffer>,
    glyphs: Option<GpuBuffer>,
    directions: Option<GpuBuffer>,
    visible: bool,
}

/// What the viewport draws besides the shaded surfaces.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DisplayOptions {
    pub mesh_edges: bool,
    /// Colour surfaces by their normalized vertex scalars in these bands.
    pub contour: Option<ContourPalette>,
    /// Background gradient as RGBA fractions, top and bottom.
    pub background: [[f32; 4]; 2],
    /// Colour of the wireframe of the undeformed shape as RGB fractions; black on the light
    /// background.
    pub wireframe: [f32; 3],
    /// Streaks along the [`RenderMesh::directions`] over the surfaces.
    pub lic: Option<LicDisplay>,
}

/// Line integral convolution: noise fixed on the surfaces is smeared along the directions as
/// projected on the screen, and the streaks modulate the brightness of the surfaces.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LicDisplay {
    pub source: LicSource,
    /// Streak length to either side of a pixel, in pixels.
    pub length: u32,
    /// Size of the noise grains in model units; never finer than a pixel.
    pub grain: f32,
    pub contrast: f32,
    /// Share of the brightness the streaks set, 0 to 1.
    pub mix: f32,
}

/// What the directions of the vertices hold.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LicSource {
    /// A vector, in the first three values.
    Vector,
    /// A symmetric tensor, whose principal direction the streaks follow: 0 for the largest
    /// principal value, 1 the middle, 2 the smallest, 3 the largest in magnitude.
    Principal(u32),
}

/// Colours of the contour bands, minimum first, each covering an equal share of the
/// normalized range; the out-of-range colours of a manual scale are bands of their own.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContourPalette {
    colors: [[f32; 3]; PALETTE_SLOTS as usize],
    count: u32,
}

impl ContourPalette {
    /// The first [`PALETTE_SLOTS`] colours; at least two are needed.
    pub fn new(colors: &[[f32; 3]]) -> Self {
        let mut palette = Self {
            colors: [[0.0; 3]; PALETTE_SLOTS as usize],
            count: colors.len().clamp(2, PALETTE_SLOTS as usize) as u32,
        };
        for (slot, color) in palette.colors.iter_mut().zip(colors) {
            *slot = *color;
        }
        palette
    }
}

impl Default for DisplayOptions {
    fn default() -> Self {
        Self {
            mesh_edges: true,
            contour: None,
            background: BACKGROUND,
            wireframe: [0.0; 3],
            lic: None,
        }
    }
}

struct Targets {
    width: u32,
    height: u32,
    msaa_view: wgpu::TextureView,
    resolve_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    /// Made when LIC is first drawn at this size.
    lic: Option<LicTargets>,
}

/// The scene before the LIC pass and the surfaces' LIC data, multisampled and resolved.
struct LicTargets {
    scene_view: wgpu::TextureView,
    msaa_view: wgpu::TextureView,
    resolve_view: wgpu::TextureView,
    bind_group: wgpu::BindGroup,
}

/// The pipelines of one set of render targets: the colour alone, or with the LIC data.
struct Pipelines {
    background: wgpu::RenderPipeline,
    surface: wgpu::RenderPipeline,
    edge: wgpu::RenderPipeline,
    wireframe: wgpu::RenderPipeline,
    mesh_edge: wgpu::RenderPipeline,
    wide_edge: wgpu::RenderPipeline,
}

/// Renders the 3D scene into an offscreen texture that the GUI shows as an image.
pub struct ViewportRenderer {
    globals: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    /// The same globals with the section view off, for the parts kept out of the section.
    unclipped_globals: wgpu::Buffer,
    unclipped_bind_group: wgpu::BindGroup,
    plain: Pipelines,
    /// Draw the scene for the LIC pass, which then puts it into the colour texture.
    with_lic: Pipelines,
    lic_pipeline: wgpu::RenderPipeline,
    lic_layout: wgpu::BindGroupLayout,
    /// Zero directions for meshes without any while others have them, as long as the
    /// longest of them.
    blank_directions: Option<wgpu::Buffer>,
    parts: Vec<GpuMesh>,
    /// Section faces of the parts, drawn while the section view is on.
    sections: Vec<GpuMesh>,
    clip: Option<ClipPlane>,
    /// Parts, by index, that the section view leaves whole.
    unclipped: Vec<bool>,
    /// Draw only the section faces of the parts and their outlines, e.g. to read results on
    /// the plane.
    sections_only: bool,
    targets: Targets,
    /// Camera and options of the image in the color texture; `None` when the scene changed
    /// since. Drawing a large mesh is expensive, and the GUI repaints for every mouse move.
    drawn: Option<(Camera, DisplayOptions)>,
}

impl ViewportRenderer {
    pub fn new(device: &wgpu::Device) -> Self {
        let shader = device.create_shader_module(wgpu::include_wgsl!("viewport.wgsl"));
        let globals_buffer = |label| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: std::mem::size_of::<Globals>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        let globals = globals_buffer("viewport globals");
        let unclipped_globals = globals_buffer("viewport globals unclipped");
        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("viewport globals"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let globals_bind_group = |buffer: &wgpu::Buffer| {
            device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("viewport globals"),
                layout: &bind_group_layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: buffer.as_entire_binding(),
                }],
            })
        };
        let bind_group = globals_bind_group(&globals);
        let unclipped_bind_group = globals_bind_group(&unclipped_globals);
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("viewport"),
            bind_group_layouts: &[Some(&bind_group_layout)],
            immediate_size: 0,
        });

        let lic_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("viewport lic"),
            entries: &[0, 1].map(|binding| wgpu::BindGroupLayoutEntry {
                binding,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: false },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            }),
        });
        let lic_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("viewport lic"),
            bind_group_layouts: &[Some(&bind_group_layout), Some(&lic_layout)],
            immediate_size: 0,
        });
        let lic_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("viewport lic"),
            layout: Some(&lic_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_lic"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            primitive: Default::default(),
            depth_stencil: None,
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_lic"),
                compilation_options: Default::default(),
                targets: &[Some(COLOR_FORMAT.into())],
            }),
            multiview_mask: None,
            cache: None,
        });

        Self {
            globals,
            bind_group,
            unclipped_globals,
            unclipped_bind_group,
            plain: Pipelines::new(device, &shader, &layout, false),
            with_lic: Pipelines::new(device, &shader, &layout, true),
            lic_pipeline,
            lic_layout,
            blank_directions: None,
            parts: Vec::new(),
            sections: Vec::new(),
            clip: None,
            unclipped: Vec::new(),
            sections_only: false,
            targets: Targets::new(device, 1, 1),
            drawn: None,
        }
    }

    /// Replaces the scene with one mesh per part; all parts start visible.
    pub fn set_parts(&mut self, device: &wgpu::Device, parts: &[RenderMesh]) {
        self.parts = parts.iter().map(|mesh| upload(device, mesh)).collect();
        self.sections.clear();
        self.update_blank_directions(device);
        self.drawn = None;
    }

    /// Replaces the mesh of one part, e.g. when its highlight changed; it stays visible or
    /// hidden as it was.
    pub fn set_part(&mut self, device: &wgpu::Device, index: usize, mesh: &RenderMesh) {
        let Some(part) = self.parts.get_mut(index) else {
            return;
        };
        let visible = part.visible;
        *part = upload(device, mesh);
        part.visible = visible;
        self.update_blank_directions(device);
        self.drawn = None;
    }

    /// Cuts the scene at a plane, or shows it whole again with `None`.
    pub fn set_clip_plane(&mut self, clip: Option<ClipPlane>) {
        if self.clip != clip {
            self.clip = clip;
            self.drawn = None;
        }
    }

    /// Parts, by index, that the section view leaves whole, e.g. a rope running through
    /// cut sheaves.
    pub fn set_unclipped_parts(&mut self, unclipped: &[bool]) {
        if self.unclipped != unclipped {
            self.unclipped = unclipped.to_vec();
            self.drawn = None;
        }
    }

    /// Shows only the section faces and the feature edges of the parts while a section is on.
    pub fn set_sections_only(&mut self, only: bool) {
        if self.sections_only != only {
            self.sections_only = only;
            self.drawn = None;
        }
    }

    /// Section faces of the parts, in the order of the parts.
    pub fn set_sections(&mut self, device: &wgpu::Device, sections: &[RenderMesh]) {
        self.sections = sections.iter().map(|mesh| upload(device, mesh)).collect();
        self.update_blank_directions(device);
        self.drawn = None;
    }

    /// Zero directions for the meshes that have none while LIC draws the others.
    fn update_blank_directions(&mut self, device: &wgpu::Device) {
        let meshes = || self.parts.iter().chain(&self.sections);
        let longest = (meshes())
            .filter(|m| m.directions.is_none())
            .filter_map(|m| m.vertices.as_ref().map(|v| v.count))
            .max();
        let needed = meshes().any(|m| m.directions.is_some());
        self.blank_directions = longest.filter(|_| needed).map(|count| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("blank directions"),
                size: count as u64 * std::mem::size_of::<Directions>() as u64,
                usage: wgpu::BufferUsages::VERTEX,
                mapped_at_creation: false,
            })
        });
    }

    pub fn set_part_visible(&mut self, index: usize, visible: bool) {
        if let Some(part) = self.parts.get_mut(index)
            && part.visible != visible
        {
            part.visible = visible;
            self.drawn = None;
        }
    }

    /// Recreates the render targets if the size changed; returns true when it did.
    pub fn resize(&mut self, device: &wgpu::Device, width: u32, height: u32) -> bool {
        let (width, height) = (width.max(1), height.max(1));
        if self.targets.width == width && self.targets.height == height {
            return false;
        }
        self.targets = Targets::new(device, width, height);
        self.drawn = None;
        true
    }

    /// The resolved color texture, to be registered with the GUI.
    pub fn color_view(&self) -> &wgpu::TextureView {
        &self.targets.resolve_view
    }

    /// Draws the scene into the color texture, unless it already shows this view of it;
    /// returns whether it drew.
    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        camera: &Camera,
        options: DisplayOptions,
    ) -> bool {
        if self.drawn.as_ref() == Some(&(camera.clone(), options)) {
            return false;
        }
        self.drawn = Some((camera.clone(), options));
        let lic = options.lic.filter(|_| self.has_directions());
        // The LIC targets take as much memory as the scene: kept only while LIC shows.
        match lic {
            Some(_) if self.targets.lic.is_none() => {
                self.targets.lic = Some(LicTargets::new(device, &self.targets, &self.lic_layout));
            }
            None => self.targets.lic = None,
            Some(_) => {}
        }
        let pixel_size = camera.pixel_size(self.targets.width as f32, self.targets.height as f32);
        let aspect = self.targets.width as f32 / self.targets.height as f32;
        let globals = Globals {
            view_proj: camera.view_proj(aspect).to_cols_array_2d(),
            lights: camera.light_directions().map(|l| l.extend(0.0).into()),
            view_dir: (-camera.forward()).extend(0.0).into(),
            background_top: options.background[0],
            background_bottom: options.background[1],
            edge: [
                self.targets.width as f32,
                self.targets.height as f32,
                WIDE_EDGE_PX,
                0.5 * WIDE_EDGE_PX
                    * camera.pixel_size(self.targets.width as f32, self.targets.height as f32)
                    / camera.depth_range(),
            ],
            contour: [
                options.contour.map_or(0.0, |p| p.count as f32),
                0.0,
                0.0,
                0.0,
            ],
            clip: self
                .clip
                .map_or([0.0; 4], |c| c.normal.extend(c.normal.dot(c.point)).into()),
            clip_options: [
                if self.clip.is_some() { 1.0 } else { 0.0 },
                // Section faces lie on the plane; a fraction of a pixel keeps them visible.
                0.25 * camera.pixel_size(self.targets.width as f32, self.targets.height as f32),
                0.0,
                0.0,
            ],
            wireframe: [
                options.wireframe[0],
                options.wireframe[1],
                options.wireframe[2],
                0.0,
            ],
            lic: lic.map_or([0.0; 4], |lic| {
                let (source, which) = match lic.source {
                    LicSource::Vector => (1.0, 0.0),
                    LicSource::Principal(which) => (2.0, which.min(3) as f32),
                };
                [source, which, lic.grain, pixel_size]
            }),
            lic_params: lic.map_or([0.0; 4], |lic| {
                [
                    lic.length.max(1) as f32,
                    lic.contrast,
                    lic.mix.clamp(0.0, 1.0),
                    0.0,
                ]
            }),
            palette: palette(options.contour),
        };
        queue.write_buffer(&self.globals, 0, bytemuck::bytes_of(&globals));
        let unclipped = Globals {
            clip_options: [0.0; 4],
            ..globals
        };
        queue.write_buffer(&self.unclipped_globals, 0, bytemuck::bytes_of(&unclipped));

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("viewport"),
        });
        // With LIC the scene goes to a texture of its own, which the LIC pass then streaks
        // into the colour texture.
        let lic_targets = self.targets.lic.as_ref().filter(|_| lic.is_some());
        let pipelines = if lic_targets.is_some() {
            &self.with_lic
        } else {
            &self.plain
        };
        {
            let attachment = |view, resolve_target, clear| {
                Some(wgpu::RenderPassColorAttachment {
                    view,
                    depth_slice: None,
                    resolve_target: Some(resolve_target),
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(clear),
                        store: wgpu::StoreOp::Discard,
                    },
                })
            };
            let attachments = match lic_targets {
                Some(lic) => [
                    attachment(&self.targets.msaa_view, &lic.scene_view, wgpu::Color::WHITE),
                    attachment(&lic.msaa_view, &lic.resolve_view, wgpu::Color::TRANSPARENT),
                ],
                None => [
                    attachment(
                        &self.targets.msaa_view,
                        &self.targets.resolve_view,
                        wgpu::Color::WHITE,
                    ),
                    None,
                ],
            };
            let attachments = if lic_targets.is_some() {
                &attachments[..]
            } else {
                &attachments[..1]
            };
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("viewport"),
                color_attachments: attachments,
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.targets.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Discard,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.set_pipeline(&pipelines.background);
            pass.draw(0..3, 0..1);
            // Section faces belong to their part and share its visibility.
            let sections_only = self.sections_only && self.clip.is_some();
            let unclipped = |index: usize| {
                self.clip.is_some() && self.unclipped.get(index).copied().unwrap_or(false)
            };
            // Meshes with whether they are drawn whole and whether the section cuts them:
            // with only the sections shown, the parts keep just their outline for
            // orientation; a part kept out of the section shows whole and has no faces.
            let sections = self
                .sections
                .iter()
                .zip(&self.parts)
                .enumerate()
                .filter(|_| self.clip.is_some())
                .filter(move |(index, _)| !unclipped(*index))
                .map(|(_, (section, part))| (section, part.visible, true, true));
            let visible = || {
                (self.parts.iter().enumerate())
                    .map(|(index, p)| {
                        let clipped = !unclipped(index);
                        (p, p.visible, !sections_only || !clipped, clipped)
                    })
                    .chain(sections.clone())
                    .filter(|(_, visible, _, _)| *visible)
                    .map(|(p, _, whole, clipped)| (p, whole, clipped))
            };
            let mut bound_clipped = true;
            let mut bind = |pass: &mut wgpu::RenderPass, clipped: bool| {
                if clipped != bound_clipped {
                    bound_clipped = clipped;
                    let group = if clipped {
                        &self.bind_group
                    } else {
                        &self.unclipped_bind_group
                    };
                    pass.set_bind_group(0, group, &[]);
                }
            };
            pass.set_pipeline(&pipelines.surface);
            for (part, _, clipped) in visible().filter(|(_, whole, _)| *whole) {
                if let (Some(vertices), Some(triangles)) = (&part.vertices, &part.triangles) {
                    bind(&mut pass, clipped);
                    pass.set_vertex_buffer(0, vertices.buffer.slice(..));
                    if lic_targets.is_some() {
                        let directions = (part.directions.as_ref().map(|d| &d.buffer))
                            .or(self.blank_directions.as_ref());
                        let Some(directions) = directions else {
                            continue;
                        };
                        pass.set_vertex_buffer(1, directions.slice(..));
                    }
                    pass.set_index_buffer(triangles.buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..triangles.count, 0, 0..1);
                }
            }
            if options.mesh_edges {
                pass.set_pipeline(&pipelines.mesh_edge);
                for (part, _, clipped) in visible().filter(|(_, whole, _)| *whole) {
                    if let (Some(vertices), Some(edges)) = (&part.vertices, &part.mesh_edges) {
                        bind(&mut pass, clipped);
                        pass.set_vertex_buffer(0, vertices.buffer.slice(..));
                        pass.set_index_buffer(edges.buffer.slice(..), wgpu::IndexFormat::Uint32);
                        pass.draw_indexed(0..edges.count, 0, 0..1);
                    }
                }
            }
            pass.set_pipeline(&pipelines.wireframe);
            for (part, _, clipped) in visible().filter(|(_, whole, _)| *whole) {
                if let Some(lines) = &part.wireframe_edges {
                    bind(&mut pass, clipped);
                    pass.set_vertex_buffer(0, lines.buffer.slice(..));
                    pass.draw(0..lines.count, 0..1);
                }
            }
            pass.set_pipeline(&pipelines.edge);
            for (part, _, clipped) in visible() {
                if let Some(lines) = &part.feature_edges {
                    bind(&mut pass, clipped);
                    pass.set_vertex_buffer(0, lines.buffer.slice(..));
                    pass.draw(0..lines.count, 0..1);
                }
            }
            pass.set_pipeline(&pipelines.wide_edge);
            // Arrows are wide lines too, so that their colours read over the surfaces.
            for (part, _, clipped) in visible().filter(|(_, whole, _)| *whole) {
                if let Some(lines) = &part.glyphs {
                    bind(&mut pass, clipped);
                    pass.set_vertex_buffer(0, lines.buffer.slice(..));
                    pass.draw(0..6, 0..lines.count / 2);
                }
            }
            for (part, _, clipped) in visible().filter(|(_, whole, _)| *whole) {
                if let Some(lines) = &part.wide_edges {
                    bind(&mut pass, clipped);
                    pass.set_vertex_buffer(0, lines.buffer.slice(..));
                    pass.draw(0..6, 0..lines.count / 2);
                }
            }
        }
        if let Some(lic) = lic_targets {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("viewport lic"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &self.targets.resolve_view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::WHITE),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&self.lic_pipeline);
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.set_bind_group(1, &lic.bind_group, &[]);
            pass.draw(0..3, 0..1);
        }
        queue.submit([encoder.finish()]);
        true
    }

    /// Whether any mesh carries directions for LIC.
    fn has_directions(&self) -> bool {
        (self.parts.iter().chain(&self.sections)).any(|m| m.directions.is_some())
    }
}

impl Pipelines {
    fn new(
        device: &wgpu::Device,
        shader: &wgpu::ShaderModule,
        layout: &wgpu::PipelineLayout,
        lic: bool,
    ) -> Self {
        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x3, 3 => Float32],
        };
        // Only the surfaces write the LIC data; lines and the background keep what is there.
        let lic_target = |writes: wgpu::ColorWrites| {
            lic.then_some(wgpu::ColorTargetState {
                format: LIC_FORMAT,
                blend: None,
                write_mask: writes,
            })
        };
        let pipeline = |label: &str,
                        vs: &str,
                        fs: &str,
                        buffers: &[Option<wgpu::VertexBufferLayout>],
                        topology: wgpu::PrimitiveTopology,
                        depth: wgpu::DepthStencilState,
                        blend: Option<wgpu::BlendState>| {
            let writes = if fs == "fs_surface_lic" {
                wgpu::ColorWrites::ALL
            } else {
                wgpu::ColorWrites::empty()
            };
            let targets = [
                Some(wgpu::ColorTargetState {
                    format: COLOR_FORMAT,
                    blend,
                    write_mask: wgpu::ColorWrites::ALL,
                }),
                lic_target(writes),
            ];
            let targets = if lic { &targets[..] } else { &targets[..1] };
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(layout),
                vertex: wgpu::VertexState {
                    module: shader,
                    entry_point: Some(vs),
                    compilation_options: Default::default(),
                    buffers,
                },
                primitive: wgpu::PrimitiveState {
                    topology,
                    ..Default::default()
                },
                depth_stencil: Some(depth),
                multisample: wgpu::MultisampleState {
                    count: SAMPLE_COUNT,
                    ..Default::default()
                },
                fragment: Some(wgpu::FragmentState {
                    module: shader,
                    entry_point: Some(fs),
                    compilation_options: Default::default(),
                    targets,
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let depth_state =
            |write: bool, compare: wgpu::CompareFunction, bias: wgpu::DepthBiasState| {
                wgpu::DepthStencilState {
                    format: DEPTH_FORMAT,
                    depth_write_enabled: Some(write),
                    depth_compare: Some(compare),
                    stencil: Default::default(),
                    bias,
                }
            };
        let background = pipeline(
            "viewport background",
            "vs_background",
            "fs_background",
            &[],
            wgpu::PrimitiveTopology::TriangleList,
            depth_state(false, wgpu::CompareFunction::Always, Default::default()),
            None,
        );
        let directions_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Directions>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &wgpu::vertex_attr_array![4 => Float32x3, 5 => Float32x3],
        };
        let (surface_vs, surface_fs, surface_buffers) = if lic {
            (
                "vs_surface_lic",
                "fs_surface_lic",
                vec![Some(vertex_layout.clone()), Some(directions_layout)],
            )
        } else {
            (
                "vs_surface",
                "fs_surface",
                vec![Some(vertex_layout.clone())],
            )
        };
        let surface = pipeline(
            "viewport surfaces",
            surface_vs,
            surface_fs,
            &surface_buffers,
            wgpu::PrimitiveTopology::TriangleList,
            depth_state(
                true,
                wgpu::CompareFunction::Less,
                wgpu::DepthBiasState {
                    constant: SURFACE_BIAS_CONSTANT,
                    slope_scale: EDGE_OFFSET_PX,
                    clamp: 0.0,
                },
            ),
            None,
        );
        let edge = pipeline(
            "viewport edges",
            "vs_edge",
            "fs_edge",
            &[Some(vertex_layout.clone())],
            wgpu::PrimitiveTopology::LineList,
            depth_state(false, wgpu::CompareFunction::LessEqual, Default::default()),
            Some(wgpu::BlendState::ALPHA_BLENDING),
        );
        let wireframe = pipeline(
            "viewport wireframe",
            "vs_wireframe",
            "fs_edge",
            &[Some(vertex_layout.clone())],
            wgpu::PrimitiveTopology::LineList,
            depth_state(false, wgpu::CompareFunction::LessEqual, Default::default()),
            Some(wgpu::BlendState::ALPHA_BLENDING),
        );
        // Mesh edges index the surface vertices and take their colour from the shader.
        let mesh_edge = pipeline(
            "viewport mesh edges",
            "vs_mesh_edge",
            "fs_edge",
            &[Some(vertex_layout)],
            wgpu::PrimitiveTopology::LineList,
            depth_state(false, wgpu::CompareFunction::LessEqual, Default::default()),
            Some(wgpu::BlendState::ALPHA_BLENDING),
        );
        // Wide lines are quads: one instance per segment reads both end vertices of the line
        // list, the vertex shader spreads them across the screen.
        let vertex_size = std::mem::size_of::<Vertex>() as u64;
        let segment_layout = wgpu::VertexBufferLayout {
            array_stride: 2 * vertex_size,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &[
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x3,
                    offset: 0,
                    shader_location: 0,
                },
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x3,
                    offset: std::mem::offset_of!(Vertex, color) as u64,
                    shader_location: 1,
                },
                wgpu::VertexAttribute {
                    format: wgpu::VertexFormat::Float32x3,
                    offset: vertex_size,
                    shader_location: 2,
                },
            ],
        };
        let wide_edge = pipeline(
            "viewport wide edges",
            "vs_wide_edge",
            "fs_edge",
            &[Some(segment_layout)],
            wgpu::PrimitiveTopology::TriangleList,
            depth_state(false, wgpu::CompareFunction::LessEqual, Default::default()),
            Some(wgpu::BlendState::ALPHA_BLENDING),
        );

        Self {
            background,
            surface,
            edge,
            wireframe,
            mesh_edge,
            wide_edge,
        }
    }
}

fn upload(device: &wgpu::Device, mesh: &RenderMesh) -> GpuMesh {
    let buffer = |label: &str, contents: &[u8], count: usize, usage: wgpu::BufferUsages| {
        (count > 0).then(|| GpuBuffer {
            buffer: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(label),
                contents,
                usage,
            }),
            count: count as u32,
        })
    };
    let vertex = wgpu::BufferUsages::VERTEX;
    GpuMesh {
        vertices: buffer(
            "part vertices",
            bytemuck::cast_slice(&mesh.vertices),
            mesh.vertices.len(),
            vertex,
        ),
        triangles: buffer(
            "part triangles",
            bytemuck::cast_slice(&mesh.triangles),
            mesh.triangles.len(),
            wgpu::BufferUsages::INDEX,
        ),
        feature_edges: buffer(
            "part feature edges",
            bytemuck::cast_slice(&mesh.feature_edges),
            mesh.feature_edges.len(),
            vertex,
        ),
        mesh_edges: buffer(
            "part mesh edges",
            bytemuck::cast_slice(&mesh.mesh_edges),
            mesh.mesh_edges.len(),
            wgpu::BufferUsages::INDEX,
        ),
        wireframe_edges: buffer(
            "part wireframe edges",
            bytemuck::cast_slice(&mesh.wireframe_edges),
            mesh.wireframe_edges.len(),
            vertex,
        ),
        wide_edges: buffer(
            "part wide edges",
            bytemuck::cast_slice(&mesh.wide_edges),
            mesh.wide_edges.len(),
            vertex,
        ),
        glyphs: buffer(
            "part glyphs",
            bytemuck::cast_slice(&mesh.glyphs),
            mesh.glyphs.len(),
            vertex,
        ),
        // Only a full set lines up with the vertices.
        directions: buffer(
            "part directions",
            bytemuck::cast_slice(&mesh.directions),
            if mesh.directions.len() == mesh.vertices.len() {
                mesh.directions.len()
            } else {
                0
            },
            vertex,
        ),
        visible: true,
    }
}

fn palette(contour: Option<ContourPalette>) -> [[f32; 4]; PALETTE_SLOTS as usize] {
    let mut palette = [[0.0; 4]; PALETTE_SLOTS as usize];
    if let Some(contour) = contour {
        for (slot, [r, g, b]) in palette.iter_mut().zip(contour.colors) {
            *slot = [r, g, b, 1.0];
        }
    }
    palette
}

impl Targets {
    fn new(device: &wgpu::Device, width: u32, height: u32) -> Self {
        let texture = |label: &str, format, sample_count, usage| {
            device
                .create_texture(&wgpu::TextureDescriptor {
                    label: Some(label),
                    size: wgpu::Extent3d {
                        width,
                        height,
                        depth_or_array_layers: 1,
                    },
                    mip_level_count: 1,
                    sample_count,
                    dimension: wgpu::TextureDimension::D2,
                    format,
                    usage,
                    view_formats: &[],
                })
                .create_view(&Default::default())
        };
        Self {
            width,
            height,
            msaa_view: texture(
                "viewport color msaa",
                COLOR_FORMAT,
                SAMPLE_COUNT,
                wgpu::TextureUsages::RENDER_ATTACHMENT,
            ),
            resolve_view: texture(
                "viewport color",
                COLOR_FORMAT,
                1,
                wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            ),
            depth_view: texture(
                "viewport depth",
                DEPTH_FORMAT,
                SAMPLE_COUNT,
                wgpu::TextureUsages::RENDER_ATTACHMENT,
            ),
            lic: None,
        }
    }
}

impl LicTargets {
    fn new(device: &wgpu::Device, targets: &Targets, layout: &wgpu::BindGroupLayout) -> Self {
        let texture = |label: &str, format, sample_count, usage| {
            device
                .create_texture(&wgpu::TextureDescriptor {
                    label: Some(label),
                    size: wgpu::Extent3d {
                        width: targets.width,
                        height: targets.height,
                        depth_or_array_layers: 1,
                    },
                    mip_level_count: 1,
                    sample_count,
                    dimension: wgpu::TextureDimension::D2,
                    format,
                    usage,
                    view_formats: &[],
                })
                .create_view(&Default::default())
        };
        let read = wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING;
        let scene_view = texture("viewport scene", COLOR_FORMAT, 1, read);
        let resolve_view = texture("viewport lic", LIC_FORMAT, 1, read);
        let msaa_view = texture(
            "viewport lic msaa",
            LIC_FORMAT,
            SAMPLE_COUNT,
            wgpu::TextureUsages::RENDER_ATTACHMENT,
        );
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("viewport lic"),
            layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&scene_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(&resolve_view),
                },
            ],
        });
        Self {
            scene_view,
            msaa_view,
            resolve_view,
            bind_group,
        }
    }
}

#[cfg(test)]
mod tests {
    use wgpu::naga;

    /// The shader compiles only when the GUI starts; check it without a GPU.
    #[test]
    fn shader_is_valid() {
        let module = naga::front::wgsl::parse_str(include_str!("viewport.wgsl"))
            .unwrap_or_else(|e| panic!("{}", e.emit_to_string(include_str!("viewport.wgsl"))));
        let flags = naga::valid::ValidationFlags::all();
        naga::valid::Validator::new(flags, naga::valid::Capabilities::all())
            .validate(&module)
            .unwrap();
    }
}
