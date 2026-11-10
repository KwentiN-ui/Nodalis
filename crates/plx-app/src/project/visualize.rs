//! The blocks of the Visualize category of the project schematic: a Plot block that draws
//! the numbers and series linked to it in the block, as a line, points or a histogram, and a
//! Mesh Preview block that shows a small picture of the mesh linked to it, so that a glance at
//! the schematic shows the geometry beside its analysis without opening it.

use super::*;
use plx_model::PlotKind;

/// The row of a Plot block that chooses what it draws.
const PLOT_TYPE: &str = "Plot type";
/// The input of a histogram with the number of its bins.
pub(super) const BINS: &str = "bins";
/// The most bins of a histogram.
const MAX_BINS: u32 = 200;

/// The size of the picture of a plot or a mesh in its block.
pub(super) const PICTURE: egui::Vec2 = egui::vec2(ROW_WIDTH, 136.0);
/// Pixels of the picture of a mesh per point of the screen; more are drawn and averaged.
const PIXELS_PER_POINT: usize = 2;
const SUPERSAMPLE: usize = 2;

/// A Plot block: what it draws and what links give its inputs.
#[derive(Clone)]
pub(super) struct PlotBlock {
    pub units: UnitSystem,
    pub kind: PlotKind,
    pub bins: u32,
    /// The numbers links give its inputs, by key, each a series; one number is a series of
    /// one value. Brought by every sync, not kept in the file.
    pub inputs: BTreeMap<String, Vec<f64>>,
}

impl PlotBlock {
    pub(super) fn new(units: UnitSystem, kind: PlotKind, bins: u32) -> Self {
        Self {
            units,
            kind,
            bins: bins.clamp(1, MAX_BINS),
            inputs: BTreeMap::new(),
        }
    }

    /// Whether its input `key` takes a series: all but the number of bins.
    pub(super) fn takes_series(&self, key: &str) -> bool {
        key != BINS && self.kind.inputs().iter().any(|(k, _)| *k == key)
    }

    /// Takes the numbers links give its inputs; whether any changed. A linked number of bins
    /// sets them.
    pub(super) fn take_inputs(&mut self, mut given: BTreeMap<String, Vec<f64>>) -> bool {
        let mut changed = false;
        if let Some(&bins) = given.remove(BINS).as_ref().and_then(|b| b.first())
            && bins.is_finite()
        {
            let bins = (bins.round() as u32).clamp(1, MAX_BINS);
            changed |= bins != self.bins;
            self.bins = bins;
        }
        changed |= given != self.inputs;
        self.inputs = given;
        changed
    }
}

/// The data of a Plot block as its body draws it: each input in the units of the block, with
/// its label.
#[derive(Clone, Default)]
pub(super) struct PlotView {
    pub kind: PlotKind,
    pub bins: u32,
    /// The inputs in the order of the kind of plot: values shown, and the name of what gives
    /// them with its unit; `None` while nothing is linked.
    pub inputs: Vec<Option<(Vec<f64>, String)>>,
}

/// The rows of a Plot block: the kind of plot and its inputs, which depend on the kind.
pub(super) fn plot_rows(node: &Node, incoming: &[(Vec<String>, String)]) -> Vec<Row> {
    let Some(plot) = node.as_plot() else {
        return Vec::new();
    };
    let row = |path: Vec<String>, text: String, tip: String| Row {
        path,
        text,
        depth: 0,
        icon: NodeIcon::Setup,
        state: None,
        expander: None,
        input: false,
        output: false,
        linked_from: None,
        collection: false,
        partial: false,
        widget: None,
        tip,
    };
    let mut rows = vec![Row {
        widget: Some(RowWidget::PlotType),
        ..row(
            vec![PLOT_TYPE.to_owned()],
            "Type".into(),
            format!("What the block draws: {}.", plot.kind.describe()),
        )
    }];
    for (key, label) in plot.kind.inputs() {
        let path = vec![links::PARAMETERS.to_owned(), (*key).to_owned()];
        let linked_from = (incoming.iter())
            .find(|(slot, _)| *slot == path)
            .map(|(_, source)| source.clone());
        let (widget, tip) = match *key {
            BINS => (
                Some(RowWidget::Bins),
                "How many bins of equal width the range of the values is split into. Type it, \
                 or drag the pin of a count here."
                    .to_owned(),
            ),
            "x" => (
                None,
                "The values along the horizontal axis: drag the pin of a series here, such as \
                 the Time of a Measure block of a whole history. Without it, Y is drawn over \
                 the number of each value."
                    .to_owned(),
            ),
            "y" => (
                None,
                "The values along the vertical axis: drag the pin of a series here, such as \
                 the Result of a Measure block of a whole history, or of a Math block computing \
                 with one."
                    .to_owned(),
            ),
            _ => (
                None,
                "The values to count: drag the pin of a series here, such as the Result of a \
                 Measure block of the values of a field."
                    .to_owned(),
            ),
        };
        rows.push(Row {
            input: true,
            linked_from,
            icon: NodeIcon::Results,
            widget,
            ..row(path, (*label).to_owned(), tip)
        });
    }
    rows
}

/// The rows of a Mesh Preview block: the mesh it shows.
pub(super) fn preview_rows(node: &Node, incoming: &[(Vec<String>, String)]) -> Vec<Row> {
    let Some(preview) = node.as_preview() else {
        return Vec::new();
    };
    let mesh = [links::MESH.to_owned()];
    let linked_from = (incoming.iter())
        .find(|(slot, _)| links::covers(slot, &mesh))
        .map(|(_, source)| source.clone());
    let count = preview.model.mesh.element_count();
    let tip = if count == 0 {
        "Mesh: drag the Mesh of an analysis or a mesh block here to see it.".to_owned()
    } else {
        format!(
            "Mesh: {count} elements and {} nodes, seen from the side the views of the \
             analysis call isometric.",
            preview.model.mesh.node_count()
        )
    };
    vec![Row {
        path: mesh.to_vec(),
        text: links::MESH.to_owned(),
        depth: 0,
        icon: NodeIcon::Mesh,
        state: None,
        expander: None,
        input: true,
        output: false,
        linked_from,
        collection: false,
        partial: false,
        widget: None,
        tip,
    }]
}

/// A Mesh Preview block: the mesh its link brings.
#[derive(Default)]
pub(super) struct PreviewBlock {
    pub model: LinkedModel,
}

/// The picture of a Mesh Preview block, made on a worker thread whenever its mesh changes.
#[derive(Default)]
pub(super) struct MeshPicture {
    /// The fingerprint of the mesh the texture shows.
    key: u64,
    texture: Option<egui::TextureHandle>,
    /// The fingerprint of the mesh being drawn and where its picture comes.
    drawing: Option<(u64, Receiver<egui::ColorImage>)>,
}

impl ProjectApp {
    /// Adds a Plot block that draws a line until another kind of plot is chosen.
    pub(super) fn add_plot(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = self.unique_name("Plot");
        let block = PlotBlock::new(self.settings.new_model.units, PlotKind::default(), 10);
        self.nodes
            .push(Node::new(id, name.clone(), Block::Plot(block)));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: drag a series to its Y row, such as the Result of a Measure block \
             of a whole history, and its Time to X."
        ));
        id
    }

    /// Adds a Mesh Preview block.
    pub(super) fn add_mesh_preview(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = self.unique_name("Mesh Preview");
        self.nodes.push(Node::new(
            id,
            name.clone(),
            Block::MeshPreview(Box::default()),
        ));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: drag the Mesh of an analysis or a mesh block to its Mesh row."
        ));
        id
    }

    /// The kind of plot of a Plot block; links to inputs it no longer has go.
    pub(super) fn set_plot_kind(&mut self, id: u64, kind: PlotKind) {
        let Some(plot) = self.node(id).and_then(Node::as_plot_mut) else {
            return;
        };
        plot.kind = kind;
        let keys: Vec<&str> = kind.inputs().iter().map(|(k, _)| *k).collect();
        self.links.retain(|l| {
            l.target != id
                || !matches!(links::slot(l), [c, k]
                    if c == links::PARAMETERS && !keys.contains(&k.as_str()))
        });
        self.sync_due = true;
    }

    /// The number of bins of a histogram, typed in its block.
    pub(super) fn set_bins(&mut self, id: u64, bins: u32) {
        if let Some(plot) = self.node(id).and_then(Node::as_plot_mut) {
            plot.bins = bins.clamp(1, MAX_BINS);
        }
    }

    /// What a Plot block draws: its inputs in its units, each named after the block that
    /// gives it.
    pub(super) fn plot_view(&self, node: &Node) -> Option<PlotView> {
        let plot = node.as_plot()?;
        let inputs = (plot.kind.inputs().iter())
            .filter(|(key, _)| *key != BINS)
            .map(|(key, _)| {
                let values = plot.inputs.get(*key)?;
                let link = self.links.iter().find(|l| {
                    l.target == node.id
                        && matches!(links::slot(l), [c, k] if c == links::PARAMETERS && k == key)
                })?;
                let kind = value_kind(&self.nodes, link.source, &link.item);
                let (scale, unit) = shown_scale(kind, plot.units);
                let shown = values.iter().map(|v| v * scale).collect();
                let mut label = self.name_of(link.source);
                if link.item.get(1).is_some_and(|n| n == series::TIME) {
                    label.push_str(" Time");
                }
                if !unit.is_empty() {
                    label.push_str(&format!(" [{unit}]"));
                }
                Some((shown, label))
            })
            .collect();
        Some(PlotView {
            kind: plot.kind,
            bins: plot.bins,
            inputs,
        })
    }

    /// Brings the pictures of the Mesh Preview blocks up to date: a mesh that changed is
    /// drawn anew on a worker thread, a picture that came is loaded as a texture.
    pub(super) fn refresh_pictures(&mut self, ctx: &egui::Context) {
        let ids: BTreeSet<u64> = (self.nodes.iter())
            .filter(|n| n.as_preview().is_some())
            .map(|n| n.id)
            .collect();
        self.pictures.retain(|id, _| ids.contains(id));
        for node in &self.nodes {
            let Some(preview) = node.as_preview() else {
                continue;
            };
            let mesh = &preview.model.mesh;
            let key = mesh_key(mesh);
            let picture = self.pictures.entry(node.id).or_default();
            if let Some((drawn, receiver)) = &picture.drawing {
                match receiver.try_recv() {
                    Ok(image) => {
                        let name = format!("mesh preview {}", node.id);
                        picture.texture =
                            Some(ctx.load_texture(name, image, egui::TextureOptions::LINEAR));
                        picture.key = *drawn;
                        picture.drawing = None;
                    }
                    Err(std::sync::mpsc::TryRecvError::Empty) => ctx.request_repaint(),
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => picture.drawing = None,
                }
            }
            let drawing = picture.drawing.as_ref().map(|(k, _)| *k);
            if key == picture.key || drawing == Some(key) {
                continue;
            }
            if mesh.element_count() == 0 {
                picture.texture = None;
                picture.key = key;
                continue;
            }
            let mesh = mesh.clone();
            // A mesh in the x-y plane is seen from the front, as a 2D analysis shows it.
            let plane = mesh
                .bounds()
                .is_some_and(|(low, high)| high[2] - low[2] <= 0.0);
            let (sender, receiver) = std::sync::mpsc::channel();
            std::thread::spawn(move || {
                let size = [
                    PICTURE.x as usize * PIXELS_PER_POINT,
                    PICTURE.y as usize * PIXELS_PER_POINT,
                ];
                let _ = sender.send(draw_mesh(&mesh, plane, size));
            });
            picture.drawing = Some((key, receiver));
            ctx.request_repaint();
        }
    }

    /// The texture of the picture of a Mesh Preview block, once drawn.
    pub(super) fn picture(&self, id: u64) -> Option<egui::TextureId> {
        self.pictures
            .get(&id)
            .and_then(|p| p.texture.as_ref())
            .map(egui::TextureHandle::id)
    }
}

/// The factor from SI units to the units shown, and the unit, of numbers of `kind`.
fn shown_scale(kind: Option<ValueKind>, units: UnitSystem) -> (f64, String) {
    match kind {
        Some(ValueKind::Quantity(quantity)) => {
            (units.shown(1.0, quantity), units.unit(quantity).to_owned())
        }
        Some(ValueKind::Derived(dimension)) => (1.0, dimension.si()),
        _ => (1.0, String::new()),
    }
}

/// A fingerprint of a mesh cheap enough for every frame: its counts and a sample of its
/// nodes.
fn mesh_key(mesh: &plx_mesh::FeMesh) -> u64 {
    let mut print = plx_mesh::Fnv::new();
    print.add(&mesh.node_count().to_le_bytes());
    print.add(&mesh.element_count().to_le_bytes());
    let coords = mesh.coords();
    let step = (coords.len() / 64).max(1);
    for point in coords.iter().step_by(step) {
        for c in point {
            print.add(&c.to_le_bytes());
        }
    }
    if let Some(last) = mesh.elements().last() {
        for node in &last.nodes {
            print.add(&node.to_le_bytes());
        }
    }
    print.finish()
}

/// A picture of the outer faces of a mesh of `size` pixels on a transparent background, each
/// part in its colour with its feature edges: seen along the space diagonal with Y up like
/// the isometric view of an analysis, a plane mesh from the front.
pub(super) fn draw_mesh(
    mesh: &plx_mesh::FeMesh,
    plane: bool,
    size: [usize; 2],
) -> egui::ColorImage {
    let [width, height] = size.map(|s| s * SUPERSAMPLE);
    // The axes of the screen and the direction towards the viewer.
    let (right, up, toward) = if plane {
        ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])
    } else {
        let s3 = 3f64.sqrt();
        let s2 = 2f64.sqrt();
        let s6 = 6f64.sqrt();
        (
            [1.0 / s2, 0.0, -1.0 / s2],
            [-1.0 / s6, 2.0 / s6, -1.0 / s6],
            [1.0 / s3, 1.0 / s3, 1.0 / s3],
        )
    };
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let all_parts;
    let parts: &[plx_mesh::Part] = if mesh.parts.is_empty() {
        all_parts = [plx_mesh::Part {
            name: String::new(),
            elements: mesh.elements().iter().map(|e| e.id).collect(),
        }];
        &all_parts
    } else {
        &mesh.parts
    };
    let skins: Vec<plx_mesh::PartSkin> = (parts.iter())
        .map(|part| plx_mesh::extract_part_skin(mesh, part, 30.0))
        .collect();
    let coords = mesh.coords();
    let screen: Vec<[f64; 3]> = (coords.iter())
        .map(|&p| [dot(p, right), dot(p, up), dot(p, toward)])
        .collect();
    // The extent of the nodes the skins use, fitted into the picture with a margin.
    let used = skins.iter().flat_map(|skin| {
        (skin.faces.iter().flat_map(|f| f.corners.iter().copied()))
            .chain(skin.lines.iter().flatten().copied())
    });
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for index in used {
        let Some(p) = screen.get(index) else {
            continue;
        };
        for k in 0..3 {
            low[k] = low[k].min(p[k]);
            high[k] = high[k].max(p[k]);
        }
    }
    let mut pixels = vec![[0f32; 4]; width * height];
    if !low[0].is_finite() {
        return downsample(&pixels, width, height);
    }
    let margin = 0.06;
    let span_x = (high[0] - low[0]).max(1e-12);
    let span_y = (high[1] - low[1]).max(1e-12);
    let scale = ((width as f64 * (1.0 - 2.0 * margin)) / span_x)
        .min((height as f64 * (1.0 - 2.0 * margin)) / span_y);
    let center = [0.5 * (low[0] + high[0]), 0.5 * (low[1] + high[1])];
    let to_pixel = |p: [f64; 3]| {
        [
            width as f64 * 0.5 + (p[0] - center[0]) * scale,
            height as f64 * 0.5 - (p[1] - center[1]) * scale,
            p[2],
        ]
    };
    let depth_span = (high[2] - low[2]).max(1e-12);
    let mut depth = vec![f64::NEG_INFINITY; width * height];
    for (index, skin) in skins.iter().enumerate() {
        let color = plx_render::part_color(index);
        for face in &skin.faces {
            let corners: Vec<[f64; 3]> = (face.corners.iter())
                .filter_map(|&c| screen.get(c).copied())
                .collect();
            if corners.len() < 3 {
                continue;
            }
            // Lit from the viewer, both sides alike, so that the picture needs no outward
            // normals.
            let normal = polygon_normal(&corners);
            let light = (0.3 + 0.7 * normal[2].abs()) as f32;
            let shade = [color[0] * light, color[1] * light, color[2] * light, 1.0];
            for i in 1..corners.len() - 1 {
                let triangle = [corners[0], corners[i], corners[i + 1]].map(to_pixel);
                fill_triangle(&mut pixels, &mut depth, width, height, triangle, shade);
            }
        }
        // Edges darker than the faces, the outline of the surface patches only.
        let ink = [0.08, 0.08, 0.09, 1.0];
        let bias = 0.01 * depth_span;
        for edge in skin.edges.iter().filter(|e| e.feature) {
            if let (Some(&a), Some(&b)) = (screen.get(edge.a), screen.get(edge.b)) {
                draw_line(
                    &mut pixels,
                    &depth,
                    width,
                    height,
                    [to_pixel(a), to_pixel(b)],
                    ink,
                    bias,
                    1.0,
                );
            }
        }
        let line_color = [color[0], color[1], color[2], 1.0];
        for &[a, b] in &skin.lines {
            if let (Some(&a), Some(&b)) = (screen.get(a), screen.get(b)) {
                draw_line(
                    &mut pixels,
                    &depth,
                    width,
                    height,
                    [to_pixel(a), to_pixel(b)],
                    line_color,
                    f64::INFINITY,
                    2.5,
                );
            }
        }
    }
    downsample(&pixels, width, height)
}

/// The unit normal of a polygon in screen space (Newell's method).
fn polygon_normal(points: &[[f64; 3]]) -> [f64; 3] {
    let mut n = [0.0; 3];
    for (i, a) in points.iter().enumerate() {
        let b = points[(i + 1) % points.len()];
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if length > 0.0 {
        n.map(|c| c / length)
    } else {
        [0.0, 0.0, 1.0]
    }
}

/// Fills a triangle of pixel coordinates and depth (larger is nearer) where it is nearer
/// than what is drawn.
fn fill_triangle(
    pixels: &mut [[f32; 4]],
    depth: &mut [f64],
    width: usize,
    height: usize,
    [a, b, c]: [[f64; 3]; 3],
    color: [f32; 4],
) {
    let area = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
    if area.abs() < 1e-12 {
        return;
    }
    let x0 = a[0].min(b[0]).min(c[0]).floor().max(0.0) as usize;
    let x1 = (a[0].max(b[0]).max(c[0]).ceil() as usize).min(width.saturating_sub(1));
    let y0 = a[1].min(b[1]).min(c[1]).floor().max(0.0) as usize;
    let y1 = (a[1].max(b[1]).max(c[1]).ceil() as usize).min(height.saturating_sub(1));
    for y in y0..=y1 {
        for x in x0..=x1 {
            let (px, py) = (x as f64 + 0.5, y as f64 + 0.5);
            let w0 = ((b[0] - px) * (c[1] - py) - (c[0] - px) * (b[1] - py)) / area;
            let w1 = ((c[0] - px) * (a[1] - py) - (a[0] - px) * (c[1] - py)) / area;
            let w2 = 1.0 - w0 - w1;
            if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                continue;
            }
            let z = w0 * a[2] + w1 * b[2] + w2 * c[2];
            let at = y * width + x;
            if z > depth[at] {
                depth[at] = z;
                pixels[at] = color;
            }
        }
    }
}

/// Draws a line of `thickness` pixels where it is not behind a face by more than `bias`.
#[allow(clippy::too_many_arguments)]
fn draw_line(
    pixels: &mut [[f32; 4]],
    depth: &[f64],
    width: usize,
    height: usize,
    [a, b]: [[f64; 3]; 2],
    color: [f32; 4],
    bias: f64,
    thickness: f64,
) {
    let length = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
    let steps = (length * 2.0).ceil().max(1.0) as usize;
    let reach = (thickness / 2.0).ceil() as i64;
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let x = a[0] + (b[0] - a[0]) * t;
        let y = a[1] + (b[1] - a[1]) * t;
        let z = a[2] + (b[2] - a[2]) * t;
        for dy in -reach..=reach {
            for dx in -reach..=reach {
                if ((dx * dx + dy * dy) as f64).sqrt() > thickness / 2.0 + 0.25 {
                    continue;
                }
                let (px, py) = (x.floor() as i64 + dx, y.floor() as i64 + dy);
                if px < 0 || py < 0 || px >= width as i64 || py >= height as i64 {
                    continue;
                }
                let at = py as usize * width + px as usize;
                if z + bias >= depth[at] {
                    pixels[at] = color;
                }
            }
        }
    }
}

/// The picture at the size shown: each pixel the mean of a square of drawn ones, which
/// smooths the edges.
fn downsample(pixels: &[[f32; 4]], width: usize, height: usize) -> egui::ColorImage {
    let (w, h) = (width / SUPERSAMPLE, height / SUPERSAMPLE);
    let samples = (SUPERSAMPLE * SUPERSAMPLE) as f32;
    let mut out = Vec::with_capacity(w * h);
    for y in 0..h {
        for x in 0..w {
            let mut sum = [0f32; 4];
            for sy in 0..SUPERSAMPLE {
                for sx in 0..SUPERSAMPLE {
                    let p = pixels[(y * SUPERSAMPLE + sy) * width + x * SUPERSAMPLE + sx];
                    // Premultiplied: a transparent sample adds nothing.
                    for k in 0..3 {
                        sum[k] += p[k] * p[3];
                    }
                    sum[3] += p[3];
                }
            }
            let c = sum.map(|v| ((v / samples).clamp(0.0, 1.0) * 255.0).round() as u8);
            out.push(egui::Color32::from_rgba_premultiplied(
                c[0], c[1], c[2], c[3],
            ));
        }
    }
    egui::ColorImage {
        size: [w, h],
        pixels: out,
        source_size: egui::vec2(w as f32, h as f32),
    }
}

impl Viewer {
    /// The rows of a Plot block that edit it in place: its kind and its bins.
    pub(super) fn edit_plot_row(
        &mut self,
        ui: &mut egui::Ui,
        id: u64,
        row: &Row,
        widget: RowWidget,
        field: egui::Rect,
        tip: &str,
    ) {
        let Some(view) = self.block(id).and_then(|b| b.plot.clone()) else {
            return;
        };
        match widget {
            RowWidget::PlotType => {
                let mut kind = view.kind;
                egui::ComboBox::from_id_salt(("plot type", id))
                    .selected_text(kind.label())
                    .width(field.width())
                    .show_ui(ui, |ui| {
                        for choice in PlotKind::ALL {
                            ui.selectable_value(&mut kind, choice, choice.label())
                                .on_hover_text(choice.describe());
                        }
                    })
                    .response
                    .on_hover_text(tip);
                if kind != view.kind {
                    self.actions.push(Action::SetPlotKind(id, kind));
                }
            }
            RowWidget::Bins => {
                let mut bins = view.bins;
                let drag = crate::numeric::count(&mut bins).range(1..=MAX_BINS);
                number_field(ui, field, row, drag, tip);
                if bins != view.bins {
                    self.actions.push(Action::SetBins(id, bins));
                }
            }
            _ => {}
        }
    }

    /// The body of a Plot block: its plot, or what it needs.
    pub(super) fn show_plot(&self, ui: &mut egui::Ui, view: &PlotView) {
        let (rect, _) = ui.allocate_exact_size(PICTURE, egui::Sense::hover());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 3.0, crate::style::label_fill());
        let ink = crate::style::plot_text();
        let small = egui::FontId::proportional(9.0);
        let first = view.inputs.first().cloned().flatten();
        let second = view.inputs.get(1).cloned().flatten();
        let empty = |text: &str| {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                text,
                egui::FontId::proportional(11.0),
                ink.gamma_multiply(0.6),
            );
        };
        // The area of the data, with room for the numbers of the axes.
        let area = egui::Rect::from_min_max(
            rect.min + egui::vec2(44.0, 16.0),
            rect.max - egui::vec2(8.0, 24.0),
        );
        let (xs, ys, x_label, y_label, bars) = match view.kind {
            PlotKind::Histogram => {
                let Some((values, label)) = first else {
                    return empty("Link a series to Values");
                };
                let bins = plx_model::histogram(&values, view.bins as usize);
                if bins.is_empty() {
                    return empty("No values to count");
                }
                let xs = vec![bins[0].0, bins[bins.len() - 1].1];
                let top = bins.iter().map(|b| b.2).max().unwrap_or(0) as f64;
                (xs, vec![0.0, top.max(1.0)], label, "Count".to_owned(), bins)
            }
            PlotKind::Line | PlotKind::Scatter => {
                let Some((ys, y_label)) = second else {
                    return empty("Link a series to Y");
                };
                let (xs, x_label) = match first {
                    Some((xs, label)) => (xs, label),
                    None => ((1..=ys.len()).map(|i| i as f64).collect(), "Index".into()),
                };
                (xs, ys, x_label, y_label, Vec::new())
            }
        };
        let finite = |v: &[f64]| {
            let f = || v.iter().copied().filter(|v| v.is_finite());
            f().reduce(f64::min).zip(f().reduce(f64::max))
        };
        let (Some((x0, x1)), Some((y0, y1))) = (finite(&xs), finite(&ys)) else {
            return empty("No values to draw");
        };
        let widen = |low: f64, high: f64| {
            if high > low {
                (low, high)
            } else {
                let pad = if low == 0.0 { 1.0 } else { low.abs() * 0.1 };
                (low - pad, high + pad)
            }
        };
        let (x0, x1) = widen(x0, x1);
        let (y0, y1) = widen(y0, y1);
        let at = |x: f64, y: f64| {
            egui::pos2(
                area.left() + ((x - x0) / (x1 - x0)) as f32 * area.width(),
                area.bottom() - ((y - y0) / (y1 - y0)) as f32 * area.height(),
            )
        };
        let grid = egui::Stroke::new(1.0, crate::style::plot_grid());
        painter.rect_stroke(area, 0.0, grid, egui::StrokeKind::Inside);
        let number = |v: f64| crate::results::format_value(v as f32);
        painter.text(
            egui::pos2(area.left() - 3.0, area.top()),
            egui::Align2::RIGHT_TOP,
            number(y1),
            small.clone(),
            ink,
        );
        painter.text(
            egui::pos2(area.left() - 3.0, area.bottom()),
            egui::Align2::RIGHT_BOTTOM,
            number(y0),
            small.clone(),
            ink,
        );
        painter.text(
            egui::pos2(area.left(), area.bottom() + 2.0),
            egui::Align2::LEFT_TOP,
            number(x0),
            small.clone(),
            ink,
        );
        painter.text(
            egui::pos2(area.right(), area.bottom() + 2.0),
            egui::Align2::RIGHT_TOP,
            number(x1),
            small.clone(),
            ink,
        );
        painter.text(
            egui::pos2(area.center().x, rect.bottom() - 2.0),
            egui::Align2::CENTER_BOTTOM,
            &x_label,
            small.clone(),
            ink.gamma_multiply(0.8),
        );
        painter.text(
            egui::pos2(rect.left() + 4.0, rect.top() + 2.0),
            egui::Align2::LEFT_TOP,
            &y_label,
            small.clone(),
            ink.gamma_multiply(0.8),
        );
        let accent = crate::style::highlight();
        match view.kind {
            PlotKind::Histogram => {
                for &(low, high, count) in &bars {
                    let bar = egui::Rect::from_two_pos(at(low, 0.0), at(high, count as f64));
                    painter.rect_filled(bar.shrink2(egui::vec2(0.5, 0.0)), 0.0, accent);
                }
            }
            PlotKind::Line | PlotKind::Scatter => {
                let points: Vec<egui::Pos2> = (xs.iter().zip(&ys))
                    .filter(|(x, y)| x.is_finite() && y.is_finite())
                    .map(|(&x, &y)| at(x, y))
                    .collect();
                if view.kind == PlotKind::Line && points.len() > 1 {
                    painter.add(egui::Shape::line(
                        points.clone(),
                        egui::Stroke::new(1.5, accent),
                    ));
                }
                if view.kind == PlotKind::Scatter || points.len() < 40 {
                    for point in points {
                        painter.circle_filled(point, 2.0, accent);
                    }
                }
                if xs.len() != ys.len() {
                    painter.text(
                        area.right_top() + egui::vec2(-3.0, 2.0),
                        egui::Align2::RIGHT_TOP,
                        format!("X has {}, Y {} values", xs.len(), ys.len()),
                        small,
                        crate::style::warning(),
                    );
                }
            }
        }
    }

    /// The body of a Mesh Preview block: the picture of its mesh, or what it needs.
    pub(super) fn show_picture(
        &self,
        ui: &mut egui::Ui,
        texture: Option<egui::TextureId>,
        has_mesh: bool,
    ) {
        let (rect, _) = ui.allocate_exact_size(PICTURE, egui::Sense::hover());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 3.0, crate::style::label_fill());
        match texture {
            Some(texture) if has_mesh => {
                let uv = egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0));
                painter.image(texture, rect, uv, egui::Color32::WHITE);
            }
            _ => {
                painter.text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    if has_mesh {
                        "Drawing ..."
                    } else {
                        "Link a mesh"
                    },
                    egui::FontId::proportional(11.0),
                    crate::style::plot_text().gamma_multiply(0.6),
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mesh_is_drawn_into_the_middle_of_its_picture() {
        let mesh = plx_mesh::structured::box_mesh(
            [0.0; 3],
            [1.0, 0.5, 0.25],
            [4, 2, 2],
            "C3D8",
            false,
            "BOX",
        );
        let image = draw_mesh(&mesh, false, [100, 60]);
        assert_eq!(image.size, [100, 60]);
        let alpha = |x: usize, y: usize| image.pixels[y * 100 + x].a();
        assert_eq!(alpha(50, 30), 255, "the middle shows the mesh");
        assert_eq!(alpha(0, 0), 0, "the corner stays transparent");
        let plane = draw_mesh(&mesh, true, [100, 60]);
        assert_eq!(plane.pixels[30 * 100 + 50].a(), 255);
    }

    #[test]
    fn a_plot_takes_its_bins_from_a_link() {
        let mut plot = PlotBlock::new(UnitSystem::default(), PlotKind::Histogram, 10);
        assert!(plot.takes_series("values") && !plot.takes_series(BINS));
        let given = BTreeMap::from([
            ("values".to_owned(), vec![1.0, 2.0]),
            (BINS.to_owned(), vec![4.4]),
        ]);
        assert!(plot.take_inputs(given.clone()));
        assert_eq!(plot.bins, 4);
        assert_eq!(plot.inputs.len(), 1);
        assert!(!plot.take_inputs(given));
    }
}
