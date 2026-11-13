//! The blocks of the Visualize category of the project schematic: a Plot block that draws
//! the numbers and series linked to it in the block, as a line, points or a histogram, and a
//! Preview block that shows what is linked to it: a small picture of a mesh, so that a glance
//! at the schematic shows the geometry beside its analysis without opening it, a number, or a
//! series of numbers with its statistics.

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

/// The input of a Preview block before anything is linked to it: a geometry, a mesh or a
/// number goes to it.
pub(super) const PREVIEW_INPUT: &str = "Input";
/// The key of the number or series a Preview block shows.
pub(super) const PREVIEW_VALUE: &str = "Input";

/// The row a link to the Preview block `target` goes to: the Mesh row for a mesh, the
/// Geometry row for a geometry or a part of one, the value row for anything else. `None` if `target` is no Preview block.
pub(super) fn preview_slot(nodes: &[Node], target: u64, item: &[String]) -> Option<Vec<String>> {
    nodes.iter().find(|n| n.id == target)?.as_preview()?;
    let mesh = [links::MESH.to_owned()];
    let geometry = [links::GEOMETRY.to_owned()];
    Some(if links::compatible(item, &mesh) {
        mesh.to_vec()
    } else if links::compatible(item, &geometry) {
        geometry.to_vec()
    } else {
        vec![links::PARAMETERS.to_owned(), PREVIEW_VALUE.to_owned()]
    })
}

/// The row of a Preview block: the geometry, mesh or number it shows, or its open input.
pub(super) fn preview_rows(node: &Node, incoming: &[(Vec<String>, String)]) -> Vec<Row> {
    let Some(preview) = node.as_preview() else {
        return Vec::new();
    };
    let mesh = [links::MESH.to_owned()];
    let geometry = [links::GEOMETRY.to_owned()];
    let value = [links::PARAMETERS.to_owned(), PREVIEW_VALUE.to_owned()];
    let from = |path: &[String]| {
        (incoming.iter())
            .find(|(slot, _)| links::covers(slot, path))
            .map(|(_, source)| source.clone())
    };
    let (path, text, icon, linked_from, tip) = if let Some(source) = from(&mesh) {
        let count = preview.model.mesh.element_count();
        let tip = format!(
            "Mesh: {count} elements and {} nodes, seen from the side the views of the \
             analysis call isometric.",
            preview.model.mesh.node_count()
        );
        (
            mesh.to_vec(),
            links::MESH.to_owned(),
            NodeIcon::Mesh,
            Some(source),
            tip,
        )
    } else if let Some(source) = from(&geometry) {
        let tip = match &preview.model.geometry {
            Some(g) => {
                let parts = g.part_names.len();
                let mut tip = format!("Geometry: {}", g.source);
                if parts > 0 {
                    tip.push_str(&format!(", {parts} parts"));
                }
                tip.push_str(
                    ", its faces and edges as the Geometry workspace shows them, seen from the \
                     side the views of the analysis call isometric.",
                );
                tip
            }
            None => "Geometry: not taken yet.".to_owned(),
        };
        (
            geometry.to_vec(),
            links::GEOMETRY.to_owned(),
            NodeIcon::Geometry,
            Some(source),
            tip,
        )
    } else if let Some(source) = from(&value) {
        let tip = if preview.series {
            "Value: the series linked here, drawn with its statistics."
        } else {
            "Value: the number linked here."
        };
        (
            value.to_vec(),
            "Value".to_owned(),
            NodeIcon::Results,
            Some(source),
            tip.to_owned(),
        )
    } else {
        (
            vec![PREVIEW_INPUT.to_owned()],
            PREVIEW_INPUT.to_owned(),
            NodeIcon::Picture,
            None,
            "Input: drag a geometry, a part of one, a mesh, a number or a series here to see it."
                .to_owned(),
        )
    };
    vec![Row {
        path,
        text,
        depth: 0,
        icon,
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

/// A Preview block: the mesh its link brings, or the number or series. Brought by every
/// sync, not kept in the file.
#[derive(Default)]
pub(super) struct PreviewBlock {
    pub model: LinkedModel,
    /// The numbers linked to it, in SI units; one number is a series of one value.
    pub values: Option<Vec<f64>>,
    /// Whether the link brings a series rather than one number.
    pub series: bool,
}

impl PreviewBlock {
    /// Takes the numbers a link gives it; whether they changed.
    pub(super) fn take_values(&mut self, values: Option<(Vec<f64>, bool)>) -> bool {
        let (values, series) = match values {
            Some((v, s)) => (Some(v), s),
            None => (None, false),
        };
        let changed = values != self.values || series != self.series;
        self.values = values;
        self.series = series;
        changed
    }
}

/// What a Preview block shows of the numbers linked to it.
#[derive(Clone)]
pub(super) struct PreviewView {
    /// The numbers in SI units.
    pub values: Vec<f64>,
    pub series: bool,
    pub kind: Option<ValueKind>,
    pub units: UnitSystem,
    /// The block the numbers come from.
    pub source: String,
}

/// The picture of the mesh or the geometry of a Preview block, made on a worker thread
/// whenever it changes.
#[derive(Default)]
pub(super) struct MeshPicture {
    /// The fingerprint of the mesh or geometry the texture shows.
    key: u64,
    texture: Option<egui::TextureHandle>,
    /// Why the geometry it shows could not be drawn.
    failed: Option<String>,
    /// The fingerprint of what is being drawn and where its picture comes.
    drawing: Option<(u64, Receiver<Result<egui::ColorImage, String>>)>,
}

/// What a Preview block draws a picture of: the mesh or the geometry the link to it brings.
pub(super) enum Shape<'a> {
    Mesh(&'a plx_mesh::FeMesh),
    Geometry(&'a plx_model::Geometry),
}

impl Shape<'_> {
    /// A fingerprint cheap enough for every frame.
    fn key(&self) -> u64 {
        match self {
            Self::Mesh(mesh) => mesh_key(mesh),
            Self::Geometry(geometry) => geometry_key(geometry),
        }
    }
}

/// The shape a Preview block draws, by the link to it: `None` while it takes none, or the
/// link has brought nothing yet.
pub(super) fn preview_shape<'a>(node: &'a Node, all: &[Link]) -> Option<Shape<'a>> {
    let preview = node.as_preview()?;
    let link = (all.iter()).find(|l| {
        l.target == node.id
            && (l.item.first()).is_some_and(|c| c == links::MESH || c == links::GEOMETRY)
    })?;
    if link.item[0] == links::MESH {
        let mesh = &preview.model.mesh;
        (mesh.element_count() > 0).then_some(Shape::Mesh(mesh))
    } else {
        preview.model.geometry.as_ref().map(Shape::Geometry)
    }
}

impl ProjectApp {
    /// Adds a Plot block that draws a line until another kind of plot is chosen.
    pub(super) fn add_plot(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = "Plot".to_string();
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

    /// Adds a Preview block.
    pub(super) fn add_preview(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = "Preview".to_string();
        self.nodes
            .push(Node::new(id, name.clone(), Block::Preview(Box::default())));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: drag a geometry, a mesh, a number or a series to its Input row."
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

    /// What a Preview block shows of the numbers linked to it, if any.
    pub(super) fn preview_view(&self, node: &Node) -> Option<PreviewView> {
        let preview = node.as_preview()?;
        // Linked before the numbers are computed: nothing to show yet.
        let values = preview.values.clone().unwrap_or_default();
        let link = self.links.iter().find(|l| {
            l.target == node.id
                && matches!(links::slot(l), [c, k] if c == links::PARAMETERS && k == PREVIEW_VALUE)
        })?;
        let mut source = self.name_of(link.source);
        if link.item.get(1).is_some_and(|n| n == series::TIME) {
            source.push_str(" Time");
        }
        Some(PreviewView {
            values,
            series: preview.series,
            kind: value_kind(&self.nodes, link.source, &link.item),
            units: self.settings.new_model.units,
            source,
        })
    }

    /// Takes the links to a Preview block other than the one just made: it shows one thing.
    pub(super) fn keep_one_preview_link(&mut self, target: u64, slot: &[String]) {
        if self.node(target).and_then(|n| n.as_preview()).is_none() {
            return;
        }
        let before = self.links.len();
        self.links
            .retain(|l| l.target != target || links::slot(l) == slot);
        if self.links.len() != before {
            self.sync_due = true;
        }
    }

    /// Brings the pictures of the Preview blocks up to date: a mesh or a geometry that
    /// changed is drawn anew on a worker thread, a picture that came is loaded as a texture.
    pub(super) fn refresh_pictures(&mut self, ctx: &egui::Context) {
        let ids: BTreeSet<u64> = (self.nodes.iter())
            .filter(|n| n.as_preview().is_some())
            .map(|n| n.id)
            .collect();
        self.pictures.retain(|id, _| ids.contains(id));
        for node in &self.nodes {
            if node.as_preview().is_none() {
                continue;
            }
            let shape = preview_shape(node, &self.links);
            // Nothing to draw: the key of no shape.
            let key = shape.as_ref().map_or(0, Shape::key);
            let picture = self.pictures.entry(node.id).or_default();
            if let Some((drawn, receiver)) = &picture.drawing {
                match receiver.try_recv() {
                    Ok(made) => {
                        match made {
                            Ok(image) => {
                                let name = format!("mesh preview {}", node.id);
                                picture.texture = Some(ctx.load_texture(
                                    name,
                                    image,
                                    egui::TextureOptions::LINEAR,
                                ));
                                picture.failed = None;
                            }
                            Err(why) => {
                                picture.texture = None;
                                picture.failed = Some(why);
                            }
                        }
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
            let size = [
                PICTURE.x as usize * PIXELS_PER_POINT,
                PICTURE.y as usize * PIXELS_PER_POINT,
            ];
            let (sender, receiver) = std::sync::mpsc::channel();
            match shape {
                None => {
                    picture.texture = None;
                    picture.failed = None;
                    picture.key = key;
                    picture.drawing = None;
                    continue;
                }
                Some(Shape::Mesh(mesh)) => {
                    let mesh = mesh.clone();
                    // A mesh in the x-y plane is seen from the front, as a 2D analysis
                    // shows it.
                    let plane = mesh
                        .bounds()
                        .is_some_and(|(low, high)| high[2] - low[2] <= 0.0);
                    std::thread::spawn(move || {
                        let _ = sender.send(Ok(draw_mesh(&mesh, plane, size)));
                    });
                }
                Some(Shape::Geometry(geometry)) => {
                    let geometry = geometry.clone();
                    // Triangulated as the Geometry workspace does, which takes Gmsh.
                    std::thread::spawn(move || {
                        let made = plx_mesher::tessellate(&geometry)
                            .map(|display| draw_geometry(&display, &geometry.suppressed, size))
                            .map_err(|e| e.to_string());
                        let _ = sender.send(made);
                    });
                }
            }
            picture.drawing = Some((key, receiver));
            ctx.request_repaint();
        }
    }

    /// Why the geometry of a Preview block could not be drawn, if it could not.
    pub(super) fn picture_failure(&self, id: u64) -> Option<String> {
        self.pictures.get(&id).and_then(|p| p.failed.clone())
    }

    /// The texture of the picture of the mesh of a Preview block, once drawn.
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

/// A fingerprint of a geometry cheap enough for every frame: the length of its BREP and a
/// sample of it, its parts' names and which are suppressed.
fn geometry_key(geometry: &plx_model::Geometry) -> u64 {
    let mut print = plx_mesh::Fnv::new();
    print.add_str("geometry");
    let brep = geometry.brep.as_bytes();
    print.add(&brep.len().to_le_bytes());
    let step = (brep.len() / 4096).max(1);
    let sample: Vec<u8> = brep.iter().step_by(step).copied().collect();
    print.add(&sample);
    print.add_debug(&geometry.part_names);
    print.add_debug(&geometry.suppressed);
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
    draw_shapes(mesh, plane, size, None)
}

/// A picture of a geometry as the Geometry workspace shows it, from its display: the faces
/// of each part in its colour, a suppressed part grey, the CAD edges dark, the edges outside
/// any face in the colour of their part. Seen like [`draw_mesh`].
pub(super) fn draw_geometry(
    display: &plx_mesher::GeometryDisplay,
    suppressed: &[String],
    size: [usize; 2],
) -> egui::ColorImage {
    let mesh = &display.mesh;
    let plane = mesh
        .bounds()
        .is_some_and(|(low, high)| high[2] - low[2] <= 0.0);
    let parts = (mesh.parts.iter().enumerate())
        .map(|(index, part)| GeometryPart {
            wire: display.part_dimensions.get(index) == Some(&1),
            suppressed: suppressed.contains(&part.name),
        })
        .collect::<Vec<_>>();
    draw_shapes(mesh, plane, size, Some(&parts))
}

/// How a part of a geometry is drawn.
struct GeometryPart {
    /// Edges only, outside any face: drawn in the colour of the part.
    wire: bool,
    suppressed: bool,
}

/// [`draw_mesh`], or with `geometry` the display mesh of a geometry: its line elements are
/// the CAD edges, and its triangles show no feature edges of their own.
fn draw_shapes(
    mesh: &plx_mesh::FeMesh,
    plane: bool,
    size: [usize; 2],
    geometry: Option<&[GeometryPart]>,
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
            bodies: Vec::new(),
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
        let part = geometry.and_then(|parts| parts.get(index));
        let color = if part.is_some_and(|p| p.suppressed) {
            [0.42, 0.42, 0.44]
        } else {
            plx_render::part_color(index)
        };
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
        let feature = geometry.is_none();
        for edge in skin.edges.iter().filter(|e| feature && e.feature) {
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
        // The CAD edges of a geometry bound its faces, dark like the feature edges of a
        // mesh; beams of a mesh and edges outside any face are drawn in their colour.
        let (line_color, line_bias, thickness) = match part {
            Some(part) if !part.wire => (ink, bias, 1.0),
            _ => ([color[0], color[1], color[2], 1.0], f64::INFINITY, 2.5),
        };
        for &[a, b] in &skin.lines {
            if let (Some(&a), Some(&b)) = (screen.get(a), screen.get(b)) {
                draw_line(
                    &mut pixels,
                    &depth,
                    width,
                    height,
                    [to_pixel(a), to_pixel(b)],
                    line_color,
                    line_bias,
                    thickness,
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

    /// The body of a Preview block: the picture of its mesh, the number or series linked to
    /// it, or what it needs.
    pub(super) fn show_preview(
        &self,
        ui: &mut egui::Ui,
        texture: Option<egui::TextureId>,
        linked: bool,
        shape: bool,
        failed: Option<&str>,
        view: Option<&PreviewView>,
    ) {
        let (rect, _) = ui.allocate_exact_size(PICTURE, egui::Sense::hover());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 3.0, crate::style::label_fill());
        let faint = crate::style::plot_text().gamma_multiply(0.6);
        if let Some(view) = view {
            show_values(&painter, rect, view);
            return;
        }
        match texture {
            Some(texture) if shape => {
                let uv = egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0));
                painter.image(texture, rect, uv, egui::Color32::WHITE);
            }
            _ => {
                let text = match failed {
                    Some(_) if shape => "Cannot draw the geometry",
                    _ if shape => "Drawing ...",
                    _ if linked => "Nothing to show yet",
                    _ => "Link something to see it here",
                };
                let response =
                    ui.interact(rect, ui.id().with("preview failure"), egui::Sense::hover());
                if let Some(why) = failed.filter(|_| shape) {
                    response.on_hover_text(why);
                }
                painter.text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    text,
                    egui::FontId::proportional(11.0),
                    faint,
                );
            }
        }
    }
}

/// Draws a number large, or a series as a line over its index with its statistics below.
fn show_values(painter: &egui::Painter, rect: egui::Rect, view: &PreviewView) {
    let ink = crate::style::plot_text();
    let faint = ink.gamma_multiply(0.6);
    let small = egui::FontId::proportional(10.0);
    let shown = |v: f64| numbers::shown_number(v, view.kind, view.units);
    let finite: Vec<f64> = view
        .values
        .iter()
        .copied()
        .filter(|v| v.is_finite())
        .collect();
    if !view.series {
        let Some(&value) = view.values.first() else {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "No value yet",
                egui::FontId::proportional(11.0),
                faint,
            );
            return;
        };
        painter.text(
            rect.center() - egui::vec2(0.0, 6.0),
            egui::Align2::CENTER_CENTER,
            shown(value),
            egui::FontId::proportional(22.0),
            ink,
        );
        painter.text(
            rect.center_bottom() - egui::vec2(0.0, 8.0),
            egui::Align2::CENTER_BOTTOM,
            &view.source,
            small,
            faint,
        );
        return;
    }
    if finite.is_empty() {
        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "An empty series",
            egui::FontId::proportional(11.0),
            faint,
        );
        return;
    }
    let stats = Stats::of(&finite);
    // The statistics take the lower lines, the line of the series the space above.
    let lines = [
        format!("{} values   mean {}", view.values.len(), shown(stats.mean)),
        format!("min {}   max {}", shown(stats.min), shown(stats.max)),
        format!("std. dev. {}", shown(stats.deviation)),
    ];
    let line_height = 13.0;
    let text_top = rect.bottom() - 6.0 - line_height * lines.len() as f32;
    for (i, line) in lines.iter().enumerate() {
        painter.text(
            egui::pos2(rect.left() + 8.0, text_top + line_height * i as f32),
            egui::Align2::LEFT_TOP,
            line,
            small.clone(),
            ink,
        );
    }
    let area = egui::Rect::from_min_max(
        rect.min + egui::vec2(8.0, 8.0),
        egui::pos2(rect.right() - 8.0, text_top - 6.0),
    );
    painter.rect_stroke(
        area,
        0.0,
        egui::Stroke::new(1.0, crate::style::plot_grid()),
        egui::StrokeKind::Inside,
    );
    let span = (stats.max - stats.min).max(f64::MIN_POSITIVE);
    let count = view.values.len();
    let at = |i: usize, v: f64| {
        let x = if count > 1 {
            i as f32 / (count - 1) as f32
        } else {
            0.5
        };
        let y = if stats.max > stats.min {
            ((v - stats.min) / span) as f32
        } else {
            0.5
        };
        egui::pos2(
            area.left() + 2.0 + x * (area.width() - 4.0),
            area.bottom() - 2.0 - y * (area.height() - 4.0),
        )
    };
    let points: Vec<egui::Pos2> = (view.values.iter().enumerate())
        .filter(|(_, v)| v.is_finite())
        .map(|(i, &v)| at(i, v))
        .collect();
    let line = crate::style::highlight();
    if points.len() == 1 {
        painter.circle_filled(points[0], 2.5, line);
    } else {
        painter.add(egui::Shape::line(points, egui::Stroke::new(1.5, line)));
    }
}

/// The statistics of a series that a Preview block shows.
#[derive(Debug, PartialEq)]
struct Stats {
    min: f64,
    max: f64,
    mean: f64,
    /// The standard deviation of the values as a whole population.
    deviation: f64,
}

impl Stats {
    /// Of finite values, at least one.
    fn of(values: &[f64]) -> Self {
        let n = values.len() as f64;
        let mean = values.iter().sum::<f64>() / n;
        let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n;
        Self {
            min: values.iter().copied().fold(f64::INFINITY, f64::min),
            max: values.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            mean,
            deviation: variance.sqrt(),
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
    fn a_preview_takes_a_geometry_and_draws_its_faces_and_edges() {
        let analysis = |id: u64, geometry: Option<plx_model::Geometry>| {
            let data = file::ProjectAnalysis {
                id,
                name: "Bracket".into(),
                position: [0.0; 2],
                geometry,
                mesh: Default::default(),
                model: Default::default(),
                mesh_inputs: BTreeMap::new(),
                results: None,
            };
            Node::analysis(id, "Bracket".into(), data)
        };
        let geometry = plx_model::Geometry {
            source: "bracket.step".into(),
            brep: "DBRep_DrawableShape".into(),
            meshing: Default::default(),
            mesh_items: Vec::new(),
            part_names: Vec::new(),
            refinement: 1.0,
            signatures: Vec::new(),
            compounds: Vec::new(),
            suppressed: Vec::new(),
        };
        let mut preview = Node::new(2, "Preview".into(), Block::Preview(Box::default()));
        let nodes = vec![analysis(1, Some(geometry.clone())), preview_clone(&preview)];
        // A geometry or a part of it goes to the Geometry row, a mesh to the Mesh row.
        let whole = [links::GEOMETRY.to_owned()];
        let part = [links::GEOMETRY.to_owned(), "BOLT".to_owned()];
        let mesh = [links::MESH.to_owned()];
        assert_eq!(preview_slot(&nodes, 2, &whole), Some(whole.to_vec()));
        assert_eq!(preview_slot(&nodes, 2, &part), Some(whole.to_vec()));
        assert_eq!(preview_slot(&nodes, 2, &mesh), Some(mesh.to_vec()));
        assert_eq!(preview_slot(&nodes, 1, &whole), None);
        let slot = preview_slot(&nodes, 2, &whole).unwrap();
        assert!(refusal(&nodes, &[], 1, &whole, 2, &slot).is_none());
        // The row says what it shows once linked.
        let rows = preview_rows(&preview, &[(whole.to_vec(), "Bracket".to_owned())]);
        assert_eq!(rows[0].text, links::GEOMETRY);
        assert_eq!(rows[0].path, whole);
        // It draws the geometry the link brought, and nothing before.
        let link = Link {
            source: 1,
            target: 2,
            item: whole.to_vec(),
            ..Default::default()
        };
        let links = [link];
        assert!(preview_shape(&preview, &links).is_none());
        if let Block::Preview(block) = &mut preview.block {
            block.model.geometry = Some(geometry);
        }
        assert!(matches!(
            preview_shape(&preview, &links),
            Some(Shape::Geometry(g)) if g.source == "bracket.step"
        ));
        // A mesh it took before is not drawn while the geometry is linked.
        assert!(preview_shape(&preview, &[]).is_none());
    }

    fn preview_clone(node: &Node) -> Node {
        Node::new(node.id, node.name.clone(), Block::Preview(Box::default()))
    }

    #[test]
    fn a_geometry_is_drawn_with_dark_edges() {
        // A square face of two triangles bounded by four edges, and a wire of its own.
        let mut mesh = plx_mesh::FeMesh::default();
        let points = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        for (i, p) in points.iter().enumerate() {
            mesh.set_node(i as u32 + 1, *p);
            mesh.set_node(i as u32 + 11, *p);
        }
        let mut element = |id: u32, shape, nodes: Vec<u32>| {
            mesh.add_element(plx_mesh::Element {
                id,
                type_name: if shape == plx_mesh::ElementShape::Tri3 {
                    "S3"
                } else {
                    "B31"
                }
                .into(),
                shape,
                nodes,
            })
            .unwrap();
        };
        element(1, plx_mesh::ElementShape::Tri3, vec![1, 2, 3]);
        element(2, plx_mesh::ElementShape::Tri3, vec![1, 3, 4]);
        for (k, (a, b)) in [(11, 12), (12, 13), (13, 14), (14, 11)].iter().enumerate() {
            element(3 + k as u32, plx_mesh::ElementShape::Line2, vec![*a, *b]);
        }
        mesh.parts
            .push(plx_mesh::Part::new("PLATE", vec![1, 2, 3, 4, 5, 6]));
        let display = plx_mesher::GeometryDisplay {
            mesh,
            entities: Vec::new(),
            part_dimensions: vec![2],
            solids: 0,
            faces: 1,
            edges: 4,
            topology: Default::default(),
        };
        let image = draw_geometry(&display, &[], [100, 100]);
        let pixel = |x: usize, y: usize| image.pixels[y * 100 + x];
        let face = pixel(50, 50);
        assert_eq!(face.a(), 255, "the middle shows the face");
        let color = plx_render::part_color(0);
        assert!(face.r() > 100 || face.g() > 100 || face.b() > 100);
        assert!((face.r() as f32 / 255.0 - color[0]).abs() < 0.05);
        // The edge along the bottom of the face is dark over it, the margin transparent.
        let light = |p: egui::Color32| p.r() as u32 + p.g() as u32 + p.b() as u32;
        let dark = (80..100).any(|y| {
            let p = pixel(50, y);
            p.a() > 0 && light(p) * 10 < light(face) * 3
        });
        assert!(dark, "the edges are drawn dark");
        assert_eq!(pixel(0, 0).a(), 0);
        // A suppressed part is grey.
        let grey = draw_geometry(&display, &["PLATE".to_owned()], [100, 100]);
        let p = grey.pixels[50 * 100 + 50];
        assert!(p.r().abs_diff(p.g()) < 8 && p.g().abs_diff(p.b()) < 12);
    }

    #[test]
    fn the_statistics_of_a_series() {
        let stats = Stats::of(&[1.0, 2.0, 3.0, 4.0]);
        assert_eq!((stats.min, stats.max, stats.mean), (1.0, 4.0, 2.5));
        assert!((stats.deviation - 1.25f64.sqrt()).abs() < 1e-12);
    }

    #[test]
    fn a_preview_takes_a_number_or_a_series() {
        let mut preview = PreviewBlock::default();
        assert!(preview.take_values(Some((vec![2.0], false))));
        assert!(!preview.take_values(Some((vec![2.0], false))));
        assert!(preview.take_values(Some((vec![2.0], true))));
        assert!(preview.take_values(None));
        assert_eq!((preview.values, preview.series), (None, false));
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
