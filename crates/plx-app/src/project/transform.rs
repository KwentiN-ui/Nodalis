//! The Transform block of the project schematic: the geometry or the mesh linked to it
//! translated, rotated, mirrored or scaled as a whole, like the Transform of mesh parts under
//! FE Model > Mesh Operations, given on to analyses or further blocks. Its lengths are kept
//! in metres like the geometry and the mesh; the result is made again whenever its input or
//! the transform changes, and when the project is opened.

use plx_mesh::MeshTransform;

use crate::numeric;
use plx_model::ValueKind;

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
            _ if transform.problem().is_some() => Err(transform
                .problem()
                .unwrap_or_default()
                .trim_end_matches('.')
                .to_owned()),
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

/// The kinds of transform, as the block's window offers them.
pub(super) const KINDS: [&str; 4] = ["Translate", "Rotate", "Mirror", "Scale"];

fn kind_of(transform: &MeshTransform) -> usize {
    match transform {
        MeshTransform::Translate(_) => 0,
        MeshTransform::Rotate { .. } => 1,
        MeshTransform::Mirror { .. } => 2,
        MeshTransform::Scale { .. } => 3,
    }
}

/// A transform of kind `kind` (an index of [`KINDS`]), keeping the point of `transform`.
fn with_kind(transform: &MeshTransform, kind: usize) -> MeshTransform {
    if kind == kind_of(transform) {
        return *transform;
    }
    let point = match *transform {
        MeshTransform::Translate(_) => [0.0; 3],
        MeshTransform::Rotate { point, .. } | MeshTransform::Mirror { point, .. } => point,
        MeshTransform::Scale { center, .. } => center,
    };
    match kind {
        1 => MeshTransform::Rotate {
            point,
            axis: [0.0, 0.0, 1.0],
            angle: 0.0,
        },
        2 => MeshTransform::Mirror {
            point,
            normal: [0.0, 0.0, 1.0],
        },
        3 => MeshTransform::Scale {
            center: point,
            factors: [1.0; 3],
        },
        _ => MeshTransform::Translate([0.0; 3]),
    }
}

const POINT: [&str; 3] = ["x", "y", "z"];
const OFFSET: [&str; 3] = ["dx", "dy", "dz"];
const AXIS: [&str; 3] = ["ax", "ay", "az"];
const NORMAL: [&str; 3] = ["nx", "ny", "nz"];
const FACTORS: [&str; 3] = ["sx", "sy", "sz"];
pub(super) const ANGLE: &str = "angle";

/// A number of a transform: its key in the Parameters row path, its label, its kind and
/// its value, lengths in metres and the angle in degrees.
pub(super) type TransformValue = (&'static str, String, ValueKind, f64);

/// The numbers of `transform`, each a row of the block that takes a number link.
pub(super) fn transform_values(transform: &MeshTransform) -> Vec<TransformValue> {
    let three = |keys: [&'static str; 3], label: &str, kind: ValueKind, v: [f64; 3]| {
        (0..3)
            .map(|i| {
                (
                    keys[i],
                    format!("{label} {}", POINT[i].to_uppercase()),
                    kind,
                    v[i],
                )
            })
            .collect::<Vec<_>>()
    };
    let (length, factor) = (ValueKind::LENGTH, ValueKind::Factor);
    match *transform {
        MeshTransform::Translate(by) => three(OFFSET, "Offset", length, by),
        MeshTransform::Rotate { point, axis, angle } => {
            let mut values = three(POINT, "Point", length, point);
            values.extend(three(AXIS, "Axis", factor, axis));
            values.push((ANGLE, "Angle".to_owned(), factor, angle));
            values
        }
        MeshTransform::Mirror { point, normal } => {
            let mut values = three(POINT, "Point", length, point);
            values.extend(three(NORMAL, "Normal", factor, normal));
            values
        }
        MeshTransform::Scale { center, factors } => {
            let mut values = three(POINT, "Center", length, center);
            values.extend(three(FACTORS, "Factor", factor, factors));
            values
        }
    }
}

/// Sets the number `key` of `transform`; whether it changed.
pub(super) fn set_transform_value(transform: &mut MeshTransform, key: &str, value: f64) -> bool {
    if !value.is_finite() {
        return false;
    }
    let at = |keys: [&str; 3]| keys.iter().position(|k| *k == key);
    let number = match transform {
        MeshTransform::Translate(by) => at(OFFSET).map(|i| &mut by[i]),
        MeshTransform::Rotate { point, axis, angle } => match (at(POINT), at(AXIS)) {
            (Some(i), _) => Some(&mut point[i]),
            (_, Some(i)) => Some(&mut axis[i]),
            _ => (key == ANGLE).then_some(angle),
        },
        MeshTransform::Mirror { point, normal } => match (at(POINT), at(NORMAL)) {
            (Some(i), _) => Some(&mut point[i]),
            (_, Some(i)) => Some(&mut normal[i]),
            _ => None,
        },
        MeshTransform::Scale { center, factors } => match (at(POINT), at(FACTORS)) {
            (Some(i), _) => Some(&mut center[i]),
            (_, Some(i)) => Some(&mut factors[i]),
            _ => None,
        },
    };
    match number {
        Some(number) if *number != value => {
            *number = value;
            true
        }
        _ => false,
    }
}

/// Whether the pin of a transform block takes what a row of path `cell` gives: a geometry,
/// a part of one or a mesh.
pub(super) fn takes(cell: &str) -> bool {
    cell == links::GEOMETRY || cell == links::MESH
}

/// The cell what transform block `id` gives goes out as, by the link to it: Mesh for a
/// mesh, else Geometry; `None` without a link.
pub(super) fn given(all: &[Link], id: u64) -> Option<&'static str> {
    let link = (all.iter()).find(|l| l.target == id && l.item.first().is_some_and(|c| takes(c)))?;
    Some(if link.item[0] == links::MESH {
        links::MESH
    } else {
        links::GEOMETRY
    })
}

/// The rows of a transform block: the one pin that takes a geometry or a mesh and gives it
/// moved, with its state, and what it does.
pub(super) fn transform_rows(node: &Node, incoming: &[(Vec<String>, String)]) -> Vec<Row> {
    let Some(block) = node.as_transform() else {
        return Vec::new();
    };
    let state = block.state();
    let mut rows = Vec::new();
    // One pin takes a geometry or a mesh, the links being typed, and gives the same kind on.
    // The pin is of the kind of the link to it: Mesh for a mesh, else Geometry.
    let linked = (incoming.iter()).find(|(slot, _)| slot.first().is_some_and(|c| takes(c)));
    let mesh = linked.is_some_and(|(slot, _)| slot[0] == links::MESH);
    let (cell, icon) = if mesh {
        (links::MESH, NodeIcon::Mesh)
    } else {
        (links::GEOMETRY, NodeIcon::Geometry)
    };
    let used = block.input.is_some();
    let mut tip = "Geometry or mesh: drag the geometry, a part or the mesh of a block here; \
                   drag the pin on the right to an analysis or another block for it moved."
        .to_owned();
    if used {
        match block.error() {
            Some(why) => tip.push_str(&format!(
                "\nThe transform failed: {why}.\nWhat comes after it is skipped."
            )),
            None => tip.push_str(&format!("\n{}", state.text())),
        }
    }
    rows.push(Row {
        text: if linked.is_some() {
            cell
        } else {
            "Geometry or Mesh"
        }
        .to_owned(),
        depth: 0,
        icon,
        state: used.then_some(state),
        expander: None,
        input: true,
        output: used && block.link_data().is_some(),
        linked_from: linked.map(|(_, source)| source.clone()),
        collection: false,
        partial: false,
        widget: None,
        tip,
        path: vec![cell.to_owned()],
    });
    rows.push(Row {
        text: KINDS[kind_of(&block.transform)].to_owned(),
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
        tip: format!(
            "{}.\nDouble-click the block to choose translate, rotate, mirror or scale.",
            block.describe()
        ),
        path: vec![TRANSFORM_ROW.to_owned()],
    });
    for (index, (key, label, kind, _)) in transform_values(&block.transform).into_iter().enumerate()
    {
        let path = vec![links::PARAMETERS.to_owned(), key.to_owned()];
        let what = if kind == ValueKind::LENGTH {
            "a length"
        } else if key == ANGLE {
            "in degrees"
        } else {
            "a number"
        };
        rows.push(Row {
            text: label.clone(),
            depth: 0,
            icon: NodeIcon::Setup,
            state: None,
            expander: None,
            input: true,
            output: false,
            linked_from: (incoming.iter())
                .find(|(slot, _)| *slot == path)
                .map(|(_, source)| source.clone()),
            collection: false,
            partial: false,
            widget: Some(RowWidget::TransformValue(index)),
            tip: format!(
                "{label}, {what}. Type it, or drag the pin of a number here: a parameter, a \
                 Math block or an input of a parameter study."
            ),
            path,
        });
    }
    rows
}

/// The dialog that chooses the kind of transform of a block; its numbers are edited in the
/// block, where they can also be linked.
pub(super) struct TransformEditor {
    pub(super) id: u64,
    name: String,
    kind: usize,
    transform: MeshTransform,
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
            kind: kind_of(&block.transform),
            transform: block.transform,
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
                ui.horizontal(|ui| {
                    for (index, kind) in KINDS.iter().enumerate() {
                        ui.radio_value(&mut self.kind, index, *kind);
                    }
                });
                ui.add_space(4.0);
                ui.weak(
                    "Moves the whole geometry or mesh linked to the block. Its numbers are \
                     set in the block, where a parameter or a study can also link to them.",
                );
                ui.add_space(8.0);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if ui.button("Cancel").clicked() {
                        result = TransformEditorResult::Cancel;
                    }
                    if ui.button("OK").clicked() {
                        result = TransformEditorResult::Ok(with_kind(&self.transform, self.kind));
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
            "{name} created: drag a geometry or a mesh to it, set its numbers in the block \
             or double-click it to choose the kind of transform, and drag its output on to analyses."
        ));
        id
    }

    /// Gives the links from transform block `id` to analyses and transform blocks the kind
    /// it gives now, a geometry or a mesh, as its one pin gives what it takes.
    pub(super) fn retype_transform_links(&mut self, id: u64) {
        let Some(cell) = given(&self.links, id) else {
            return;
        };
        let takes_both: BTreeSet<u64> = (self.nodes.iter())
            .filter(|n| matches!(n.kind(), Kind::Analysis | Kind::Transform))
            .map(|n| n.id)
            .collect();
        let mut changed = false;
        for link in self.links.iter_mut().filter(|l| {
            l.source == id
                && takes_both.contains(&l.target)
                && matches!(l.item.as_slice(), [c] if takes(c) && c != cell)
        }) {
            link.item = vec![cell.to_owned()];
            if !link.slot.is_empty() {
                link.slot = vec![cell.to_owned()];
            }
            changed = true;
        }
        if changed {
            self.sync_due = true;
        }
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
                let mut before = Vec::new();
                if let Some(block) = self.node(id).and_then(Node::as_transform_mut) {
                    before = transform_values(&block.transform);
                    block.transform = transform;
                }
                // The numbers of another kind of transform are others: links to those of
                // the kind before go.
                let keys: Vec<&str> = (transform_values(&transform).iter()).map(|v| v.0).collect();
                let name = self.name_of(id);
                let gone: Vec<Link> = (self.links.iter())
                    .filter(|l| {
                        l.target == id
                            && matches!(links::slot(l), [cell, key]
                                if cell == links::PARAMETERS && !keys.contains(&key.as_str()))
                    })
                    .cloned()
                    .collect();
                for link in &gone {
                    let from = self.name_of(link.source);
                    let key = &links::slot(link)[1];
                    let label = (before.iter().find(|v| v.0 == key)).map_or(key.as_str(), |v| &v.1);
                    self.output.push(format!(
                        "{name}: the link from {from} is removed, as the transform has no \
                         {label} any more."
                    ));
                }
                self.links.retain(|l| !gone.contains(l));
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
        assert_eq!(rows[0].text, "Geometry or Mesh");
        assert_eq!(rows[1].text, "Translate");
        assert!(rows[1].tip.starts_with("Translate by (0, 0, 2000) mm"));
        // Each number is a row that takes a number link.
        assert_eq!(rows[4].path, [links::PARAMETERS, "dz"]);
        assert!(rows[4].input);
        // The one pin takes a mesh as well as a geometry.
        let mesh = [links::MESH.to_owned()];
        assert!(refusal(&nodes, &[], 1, &mesh, 2, &rows[0].path).is_none());
        let link = Link {
            source: 1,
            target: 2,
            item: vec![links::MESH.to_owned()],
            ..Default::default()
        };
        take_transform_input(&mut nodes, std::slice::from_ref(&link), 2);
        let block = nodes[1].as_transform().unwrap();
        assert_eq!(block.state(), CellState::UpToDate);
        let data = block.link_data().unwrap();
        assert_eq!(data.mesh.node(4), Some([0.0, 0.0, 3.0]));
        assert_eq!(data.mesh.parts[0].name, "TET");
        // The one pin gives the mesh on, as a mesh.
        let rows = transform_rows(&nodes[1], &[(mesh.to_vec(), "Tet".to_owned())]);
        assert_eq!(rows[0].text, "Mesh");
        assert!(rows[0].output);
        assert_eq!(rows[0].path, mesh);
        assert_eq!(rows[0].state, Some(CellState::UpToDate));
        // A number set in the block or through a link moves it anew.
        let block = nodes[1].as_transform_mut().unwrap();
        assert!(set_transform_value(&mut block.transform, "dz", 1.0));
        take_transform_input(&mut nodes, std::slice::from_ref(&link), 2);
        let data = nodes[1].as_transform().unwrap().link_data().unwrap();
        assert_eq!(data.mesh.node(4), Some([0.0, 0.0, 2.0]));
        // Another kind keeps the point and gives other numbers.
        let rotate = with_kind(&MeshTransform::Translate([1.0; 3]), 1);
        let keys: Vec<_> = transform_values(&rotate).iter().map(|v| v.0).collect();
        assert_eq!(keys, ["x", "y", "z", "ax", "ay", "az", "angle"]);
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
