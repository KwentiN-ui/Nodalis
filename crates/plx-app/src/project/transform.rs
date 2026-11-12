//! The Transform block of the project schematic: the geometry or the mesh linked to it
//! translated, rotated, mirrored or scaled as a whole, like the Transform of mesh parts under
//! FE Model > Mesh Operations, given on to analyses or further blocks. Its lengths are kept
//! in metres like the geometry and the mesh; the result is made again whenever its input or
//! the transform changes, and when the project is opened.

use plx_mesh::MeshTransform;

use crate::mesh_tools::{TransformDraft, transform_form};
use crate::numeric;

use super::*;

/// The row of a transform block that says what it does.
pub(super) const TRANSFORM_ROW: &str = "Transform";

/// A transform block: the transform, the geometry or mesh it takes and what it made of it.
pub(super) struct TransformBlock {
    /// The geometry or mesh as moved, given on through links.
    pub(super) model: LinkedModel,
    pub(super) transform: MeshTransform,
    pub(super) units: UnitSystem,
    /// What the link brings, with its fingerprint; `None` without a link.
    input: Option<(u64, Input)>,
    /// Fingerprint of the input and the transform the result was made from, or why making it
    /// failed.
    made: Option<(u64, Result<(), String>)>,
}

/// What a transform block takes: a geometry, or a mesh with what belongs to it.
enum Input {
    Geometry(plx_model::Geometry),
    Mesh(Box<links::MeshCopy>),
}

impl TransformBlock {
    pub(super) fn new(transform: MeshTransform, units: UnitSystem) -> Self {
        Self {
            model: LinkedModel::default(),
            transform,
            units,
            input: None,
            made: None,
        }
    }

    fn wanted(&self) -> Option<u64> {
        let (input, _) = self.input.as_ref()?;
        let mut print = plx_mesh::Fnv::new();
        print.add(&input.to_le_bytes());
        print.add_debug(&self.transform);
        Some(print.finish())
    }

    pub(super) fn state(&self) -> CellState {
        match (self.wanted(), &self.made) {
            (None, _) => CellState::Missing,
            (Some(wanted), Some((made, result))) if wanted == *made => match result {
                Ok(()) => CellState::UpToDate,
                Err(_) => CellState::Failed,
            },
            _ => CellState::OutOfDate,
        }
    }

    fn error(&self) -> Option<&str> {
        match (&self.made, self.state()) {
            (Some((_, Err(why))), CellState::Failed) => Some(why),
            _ => None,
        }
    }

    /// Whether it takes a mesh rather than a geometry.
    fn takes_mesh(&self) -> bool {
        matches!(self.input, Some((_, Input::Mesh(_))))
    }

    /// What links read of it: its result once made.
    pub(super) fn link_data(&self) -> Option<links::Data<'_>> {
        (self.state() == CellState::UpToDate).then(|| self.model.data())
    }

    /// Makes the result anew for the input as it is.
    fn generate(&mut self) {
        let Some(wanted) = self.wanted() else {
            return;
        };
        let Some((_, input)) = &self.input else {
            return;
        };
        let transform = self.transform;
        let made = match input {
            Input::Geometry(geometry) => plx_mesher::transform_geometry(geometry, &transform)
                .map(|geometry| LinkedModel {
                    geometry: Some(geometry),
                    ..Default::default()
                })
                .map_err(|e| e.to_string()),
            Input::Mesh(copy) => moved_mesh(copy, &transform),
        };
        match made {
            Ok(model) => {
                self.model = model;
                self.made = Some((wanted, Ok(())));
            }
            Err(why) => {
                self.model = LinkedModel::default();
                self.made = Some((wanted, Err(why)));
            }
        }
    }

    /// The transform in words, with its lengths in the block's units.
    fn describe(&self) -> String {
        let length =
            |v: f64| numeric::format_physical(self.units.shown(v, plx_model::Quantity::Length));
        let unit = self.units.unit(plx_model::Quantity::Length);
        let vector = |v: [f64; 3]| {
            format!(
                "({}, {}, {}) {unit}",
                length(v[0]),
                length(v[1]),
                length(v[2])
            )
        };
        let direction = |v: [f64; 3]| {
            format!(
                "({}, {}, {})",
                numeric::format_physical(v[0]),
                numeric::format_physical(v[1]),
                numeric::format_physical(v[2])
            )
        };
        match self.transform {
            MeshTransform::Translate(by) => format!("Translate by {}", vector(by)),
            MeshTransform::Rotate { axis, angle, .. } => {
                format!(
                    "Rotate {}° about {}",
                    numeric::format_physical(angle),
                    direction(axis)
                )
            }
            MeshTransform::Mirror { normal, .. } => {
                format!("Mirror, normal {}", direction(normal))
            }
            MeshTransform::Scale { factors, .. } => format!("Scale by {}", direction(factors)),
        }
    }
}

/// The mesh of `copy` with every node moved, and the geometry it was meshed from moved
/// alike. Its parts stay as they are, as parts read from a file.
fn moved_mesh(copy: &links::MeshCopy, transform: &MeshTransform) -> Result<LinkedModel, String> {
    if let Some(problem) = transform.problem() {
        return Err(problem.trim_end_matches('.').to_owned());
    }
    let geometry = match &copy.geometry {
        Some(geometry) => {
            Some(plx_mesher::transform_geometry(geometry, transform).map_err(|e| e.to_string())?)
        }
        None => None,
    };
    let mut mesh = copy.mesh.clone();
    let parts: Vec<usize> = (0..mesh.parts.len()).collect();
    let faces = mesh.transform_parts(&parts, transform);
    let mut fe = plx_model::FeModel {
        mesh_sets: copy.mesh_sets.clone(),
        element_types: copy.element_types.clone(),
        mesh_operations: copy.mesh_operations.clone(),
        ..Default::default()
    };
    fe.renumber_faces(&faces);
    Ok(LinkedModel {
        geometry,
        mesh,
        fe,
        mesh_inputs: BTreeMap::new(),
    })
}

/// Brings what the transform block `id` takes up to date with the block linked to its
/// Geometry or Mesh row, and makes its result when that or the transform changed.
pub(super) fn take_transform_input(nodes: &mut [Node], all: &[Link], id: u64) {
    let link = all.iter().find(|l| {
        l.target == id
            && l.item
                .first()
                .is_some_and(|c| c == links::GEOMETRY || c == links::MESH)
    });
    let input = match link {
        Some(link) if links::is_geometry_part(link) => {
            let parts = links::linked_parts(all, link.source, id);
            let file = (nodes.iter_mut().find(|n| n.id == link.source)).and_then(Node::as_file_mut);
            match file.map(|f| f.subset(&parts)) {
                Some(Ok(None)) => return,
                Some(Ok(Some(geometry))) => Some(Input::Geometry(geometry)),
                _ => None,
            }
        }
        Some(link) => {
            let source = nodes.iter().find(|n| n.id == link.source);
            // An analysis that is loading in its window gives its data once it has loaded.
            if source.is_some_and(|n| n.link_data().is_none() && n.kind() == Kind::Analysis) {
                return;
            }
            let data = source.and_then(Node::link_data);
            if link.item[0] == links::MESH {
                (data.filter(|d| d.mesh.element_count() > 0))
                    .map(|d| Input::Mesh(Box::new(links::MeshCopy::of(d))))
            } else {
                data.and_then(|d| d.geometry.cloned()).map(Input::Geometry)
            }
        }
        None => None,
    };
    let Some(block) = (nodes.iter_mut().find(|n| n.id == id)).and_then(Node::as_transform_mut)
    else {
        return;
    };
    let input = input.map(|input| {
        let mut print = plx_mesh::Fnv::new();
        match &input {
            Input::Geometry(g) => {
                print.add_str(&g.brep);
                print.add_debug(&g.part_names);
                print.add_debug(&g.compounds);
            }
            Input::Mesh(copy) => {
                let data = links::Data {
                    geometry: copy.geometry.as_ref(),
                    mesh: &copy.mesh,
                    fe: &plx_model::FeModel {
                        mesh_sets: copy.mesh_sets.clone(),
                        element_types: copy.element_types.clone(),
                        mesh_operations: copy.mesh_operations.clone(),
                        ..Default::default()
                    },
                    mesh_inputs: &copy.mesh_inputs,
                };
                print.add(&links::fingerprint(data, &[links::MESH.to_owned()]).to_le_bytes());
                print.add_debug(&copy.geometry.as_ref().map(|g| &g.brep));
            }
        }
        (print.finish(), input)
    });
    let same = match (&block.input, &input) {
        (Some((a, _)), Some((b, _))) => a == b,
        (None, None) => true,
        _ => false,
    };
    if !same {
        block.input = input;
    }
    if block.state() == CellState::OutOfDate {
        block.generate();
    }
}

/// The rows of a transform block: the geometry and the mesh it takes and gives, the one that
/// is linked with its state, and what it does.
pub(super) fn transform_rows(node: &Node, incoming: &[(Vec<String>, String)]) -> Vec<Row> {
    let Some(block) = node.as_transform() else {
        return Vec::new();
    };
    let linked = |path: &[String]| {
        (incoming.iter())
            .find(|(slot, _)| slot.as_slice() == path)
            .map(|(_, source)| source.clone())
    };
    let state = block.state();
    let mut rows = Vec::new();
    for (cell, icon, mesh) in [
        (links::GEOMETRY, NodeIcon::Geometry, false),
        (links::MESH, NodeIcon::Mesh, true),
    ] {
        let path = vec![cell.to_owned()];
        let used = block.input.is_some() && block.takes_mesh() == mesh;
        let mut tip = format!(
            "{cell}: drag the {} of a block here; drag the pin on the right to an analysis or \
             another block for the {} moved.",
            cell.to_lowercase(),
            cell.to_lowercase()
        );
        if used {
            match block.error() {
                Some(why) => tip.push_str(&format!(
                    "\nThe transform failed: {why}.\nWhat comes after it is skipped."
                )),
                None => tip.push_str(&format!("\n{}", state.text())),
            }
        }
        rows.push(Row {
            text: cell.to_owned(),
            depth: 0,
            icon,
            state: used.then_some(state),
            expander: None,
            input: true,
            output: used && block.link_data().is_some(),
            linked_from: linked(&path),
            collection: false,
            partial: false,
            widget: None,
            tip,
            path,
        });
    }
    rows.push(Row {
        text: block.describe(),
        depth: 0,
        icon: NodeIcon::Transform,
        state: None,
        expander: None,
        input: false,
        output: false,
        linked_from: None,
        collection: false,
        partial: false,
        widget: None,
        tip: "What the transform does: double-click the block to edit it.".to_owned(),
        path: vec![TRANSFORM_ROW.to_owned()],
    });
    rows
}

/// The dialog that edits the transform of a block.
pub(super) struct TransformEditor {
    pub(super) id: u64,
    name: String,
    draft: TransformDraft,
    units: UnitSystem,
    error: Option<String>,
}

pub(super) enum TransformEditorResult {
    Open,
    Cancel,
    Ok(MeshTransform),
}

impl TransformEditor {
    pub(super) fn new(id: u64, name: &str, block: &TransformBlock) -> Self {
        Self {
            id,
            name: name.to_owned(),
            draft: TransformDraft::of(&block.transform),
            units: block.units,
            error: None,
        }
    }

    pub(super) fn show(&mut self, ctx: &egui::Context) -> TransformEditorResult {
        let mut result = TransformEditorResult::Open;
        let mut open = true;
        egui::Window::new(format!("Transform: {}", self.name))
            .id(crate::ids::window(ctx, "transform block"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                egui::Grid::new("transform block grid")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| {
                        transform_form(ui, &mut self.draft, false, self.units);
                        ui.label("");
                        ui.weak(
                            "Moves the whole geometry or mesh linked to the block; the \
                             analyses after it take it moved.",
                        );
                        ui.end_row();
                    });
                if let Some(error) = &self.error {
                    ui.add_space(4.0);
                    ui.colored_label(crate::style::error(), error);
                }
                ui.add_space(8.0);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if ui.button("Cancel").clicked() {
                        result = TransformEditorResult::Cancel;
                    }
                    if ui.button("OK").clicked() {
                        let transform = self.draft.transform();
                        match transform.problem() {
                            Some(problem) => self.error = Some(problem.into()),
                            None => result = TransformEditorResult::Ok(transform),
                        }
                    }
                });
            });
        if !open || ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            result = TransformEditorResult::Cancel;
        }
        result
    }
}

impl ProjectApp {
    /// Adds a transform block that moves nothing yet; returns its id.
    pub(super) fn add_transform(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = block_name("Transform");
        let block = TransformBlock::new(MeshTransform::Translate([0.0; 3]), UnitSystem::default());
        self.nodes.push(Node::new(
            id,
            name.clone(),
            Block::Transform(Box::new(block)),
        ));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: drag a geometry or a mesh to it, double-click it to set the \
             transform, and drag its output on to analyses."
        ));
        id
    }

    pub(super) fn edit_transform(&mut self, id: u64) {
        let Some(node) = self.nodes.iter().find(|n| n.id == id) else {
            return;
        };
        if let Some(block) = node.as_transform() {
            self.transform_editor = Some(TransformEditor::new(id, &node.name, block));
        }
    }

    pub(super) fn transform_editor_window(&mut self, ctx: &egui::Context) {
        let Some(editor) = &mut self.transform_editor else {
            return;
        };
        let id = editor.id;
        match editor.show(ctx) {
            TransformEditorResult::Open => {}
            TransformEditorResult::Cancel => self.transform_editor = None,
            TransformEditorResult::Ok(transform) => {
                self.checkpoint(&[]);
                self.transform_editor = None;
                if let Some(block) = self.node(id).and_then(Node::as_transform_mut) {
                    block.transform = transform;
                }
                self.sync_due = true;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_mesh::{Element, ElementShape, Part};

    fn tet_analysis(id: u64) -> Node {
        let mut mesh = plx_mesh::FeMesh::default();
        for (i, p) in [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        .iter()
        .enumerate()
        {
            mesh.set_node(i as u32 + 1, *p);
        }
        mesh.add_element(Element {
            id: 1,
            type_name: "C3D4".into(),
            shape: ElementShape::Tet4,
            nodes: vec![1, 2, 3, 4],
        })
        .unwrap();
        mesh.parts.push(Part::new("TET", vec![1]));
        let data = file::ProjectAnalysis {
            id,
            name: "Tet".into(),
            position: [0.0; 2],
            geometry: None,
            mesh,
            model: Default::default(),
            mesh_inputs: BTreeMap::new(),
            results: None,
        };
        Node::analysis(id, "Tet".into(), data)
    }

    #[test]
    fn a_transform_block_moves_the_mesh_linked_to_it() {
        let block = TransformBlock::new(
            MeshTransform::Translate([0.0, 0.0, 2.0]),
            UnitSystem::default(),
        );
        let mut nodes = vec![
            tet_analysis(1),
            Node::new(2, "Transform".into(), Block::Transform(Box::new(block))),
        ];
        let rows = transform_rows(&nodes[1], &[]);
        assert!(rows.iter().all(|r| r.state.is_none() && !r.output));
        assert_eq!(rows[2].text, "Translate by (0, 0, 2000) mm");
        let link = Link {
            source: 1,
            target: 2,
            item: vec![links::MESH.to_owned()],
            ..Default::default()
        };
        take_transform_input(&mut nodes, &[link], 2);
        let block = nodes[1].as_transform().unwrap();
        assert_eq!(block.state(), CellState::UpToDate);
        let data = block.link_data().unwrap();
        assert_eq!(data.mesh.node(4), Some([0.0, 0.0, 3.0]));
        assert_eq!(data.mesh.parts[0].name, "TET");
        // The mesh row gives it on; the geometry row is not used.
        let rows = transform_rows(&nodes[1], &[]);
        assert!(rows[1].output && !rows[0].output);
        assert_eq!(rows[1].state, Some(CellState::UpToDate));
        // A transform that cannot be applied fails, and gives nothing on.
        nodes[1].as_transform_mut().unwrap().transform = MeshTransform::Scale {
            center: [0.0; 3],
            factors: [0.0, 1.0, 1.0],
        };
        take_transform_input(&mut nodes, &[], 2);
        assert_eq!(nodes[1].as_transform().unwrap().state(), CellState::Missing);
        let link = Link {
            source: 1,
            target: 2,
            item: vec![links::MESH.to_owned()],
            ..Default::default()
        };
        take_transform_input(&mut nodes, &[link], 2);
        let block = nodes[1].as_transform().unwrap();
        assert_eq!(block.state(), CellState::Failed);
        assert!(block.link_data().is_none());
        // Written and read again.
        let written = nodes[1].written([0.0; 2]);
        let read = Node::read(written.unwrap());
        assert_eq!(read.kind(), Kind::Transform);
    }
}
