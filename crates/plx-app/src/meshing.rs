//! Meshing the CAD geometry with Gmsh: the mesh setup with its items, the default
//! meshing parameters and the mesh generation part by part on a worker thread.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::time::Instant;

use plx_mesh::{ElementShape, FeMesh, NodeId};
use plx_mesher::{CadEntity, GeneratedMesh};
use plx_model::{
    Algorithm2d, Algorithm3d, ExtrudeLayers, ExtrudeMesh, Geometry, MeshSetupItem, MeshSetupKind,
    MeshingParameters, Quantity, Recombination, ThickenShellMesh, UnitSystem, next_name,
};
use serde::{Deserialize, Serialize};

use crate::geometry_selection;
use crate::model::{Highlight, Hit, Model};
use crate::numeric;
use crate::selection::{GeometryRegion, History, Operation, PartPicks, Picker, PickerAction};
use crate::viewport::{BoxSelect, Preview};

/// Whether the parameters reach a part of faces, the only parts the quad-dominated option
/// changes: of the parts named, or of all parts if none are; true without a geometry view.
fn reaches_faces(view: Option<&Model>, parts: &BTreeSet<String>) -> bool {
    view.is_none_or(|view| {
        (view.parts.iter())
            .any(|p| p.dimension == Some(2) && (parts.is_empty() || parts.contains(&p.name)))
    })
}

/// The rows of the meshing parameters in a two-column grid. `faces`: whether they reach a
/// part of faces; the quad-dominated option is greyed out otherwise.
fn parameters_ui(ui: &mut egui::Ui, d: &mut MeshingParameters, units: UnitSystem, faces: bool) {
    ui.label("Max. element size");
    ui.add(numeric::quantity(&mut d.max_size, units, Quantity::Length).range(0.0..=f64::MAX));
    ui.end_row();
    ui.label("Min. element size");
    ui.add(numeric::quantity(&mut d.min_size, units, Quantity::Length).range(0.0..=f64::MAX));
    ui.end_row();
    ui.label("Elements per curvature radius")
        .on_hover_text("0 turns off the refinement on curved surfaces.");
    ui.add(numeric::drag_value(&mut d.elements_per_curvature).range(0.0..=100.0));
    ui.end_row();
    ui.label("Mesh type");
    ui.vertical(|ui| {
        ui.checkbox(
            &mut d.second_order,
            "Second order (C3D10, S6/S8, CPS6/CPS8)",
        );
        ui.add_enabled(
            d.second_order,
            egui::Checkbox::new(
                &mut d.midside_nodes_on_geometry,
                "Midside nodes on geometry",
            ),
        );
        ui.checkbox(&mut d.optimize, "Optimize mesh (Netgen)");
        ui.add_enabled(
            faces,
            egui::Checkbox::new(&mut d.quad_dominated, "Quad-dominated surface mesh"),
        )
        .on_hover_text(
            "Mesh faces (2D models and shells) mostly with quadrilaterals. Solids keep \
             triangles on their faces, as tetrahedra need them.",
        )
        .on_disabled_hover_text(
            "Only for faces (2D models and shells): the parts are solids, meshed with \
             tetrahedra, which need triangles on their faces.",
        );
        ui.checkbox(&mut d.hexahedra_where_possible, "Hexahedra where possible")
            .on_hover_text(
                "Parts that are straight extrusions of a face or bodies of revolution get a \
                 swept mesh of hexahedra; the others stay tetrahedra.",
            );
    });
    ui.end_row();
}

/// The open window of the default meshing parameters, which apply to every part that no
/// Meshing Parameters item covers.
pub struct MeshSetupWindow {
    draft: MeshingParameters,
    /// Whether the geometry has a part of faces, for the quad-dominated option.
    faces: bool,
    /// Unit system of the model, the unit of the sizes.
    units: UnitSystem,
}

pub enum MeshSetupResult {
    Open,
    /// Take over the parameters and close.
    Ok(MeshingParameters),
    /// Take over the parameters, close and mesh all parts.
    Mesh(MeshingParameters),
    Cancel,
}

impl MeshSetupWindow {
    pub fn new(setup: &MeshingParameters, units: UnitSystem, view: Option<&Model>) -> Self {
        Self {
            draft: *setup,
            faces: reaches_faces(view, &BTreeSet::new()),
            units,
        }
    }

    pub fn show(&mut self, ctx: &egui::Context) -> MeshSetupResult {
        let mut open = true;
        let mut result = MeshSetupResult::Open;
        egui::Window::new("Default Meshing Parameters")
            .id(crate::ids::window(ctx, "default meshing parameters"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                let (d, units, faces) = (&mut self.draft, self.units, self.faces);
                ui.weak("Apply to all parts without their own Meshing Parameters.");
                egui::Grid::new("mesh size")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| parameters_ui(ui, d, units, faces));
                if d.max_size <= 0.0 {
                    ui.colored_label(
                        crate::style::error(),
                        "The max. element size must be greater than 0.",
                    );
                }
                ui.separator();
                ui.horizontal(|ui| {
                    let valid = d.max_size > 0.0;
                    if ui
                        .add_enabled(valid, egui::Button::new("Mesh All Parts"))
                        .clicked()
                    {
                        result = MeshSetupResult::Mesh(*d);
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("Cancel").clicked() {
                            result = MeshSetupResult::Cancel;
                        }
                        if ui.add_enabled(valid, egui::Button::new("OK")).clicked() {
                            result = MeshSetupResult::Ok(*d);
                        }
                    });
                });
            });
        if !open {
            result = MeshSetupResult::Cancel;
        }
        result
    }
}

/// The item types of "Create Mesh Setup Item" that Nodalis supports.
const TYPES: [&str; 5] = [
    "Meshing Parameters",
    "Local Mesh Size",
    "Tetrahedral Gmsh",
    "Extrude Mesh",
    "Thicken Shell Mesh",
];

fn type_index(kind: &MeshSetupKind) -> usize {
    match kind {
        MeshSetupKind::MeshingParameters { .. } => 0,
        MeshSetupKind::LocalMeshSize { .. } => 1,
        MeshSetupKind::TetrahedralGmsh { .. } => 2,
        MeshSetupKind::ExtrudeMesh(_) => 3,
        MeshSetupKind::ThickenShellMesh(_) => 4,
    }
}

/// A new item of a type with the values of the geometry's defaults.
fn new_kind(index: usize, geometry: &Geometry) -> MeshSetupKind {
    match index {
        0 => MeshSetupKind::MeshingParameters {
            parts: Vec::new(),
            parameters: geometry.meshing,
        },
        1 => MeshSetupKind::LocalMeshSize {
            faces: Vec::new(),
            edges: Vec::new(),
            size: geometry.meshing.max_size / 4.0,
        },
        2 => MeshSetupKind::TetrahedralGmsh {
            parts: Vec::new(),
            algorithm_2d: Algorithm2d::default(),
            algorithm_3d: Algorithm3d::default(),
        },
        3 => MeshSetupKind::ExtrudeMesh(ExtrudeMesh::default()),
        _ => MeshSetupKind::ThickenShellMesh(ThickenShellMesh {
            // A tenth of the element size: about as thin as a sheet next to its mesh.
            thickness: round_thickness(geometry.meshing.max_size / 10.0),
            ..ThickenShellMesh::default()
        }),
    }
}

/// Rounds to one significant digit, so that a proposed thickness reads well.
fn round_thickness(size: f64) -> f64 {
    if !(size.is_finite() && size > 0.0) {
        return ThickenShellMesh::default().thickness;
    }
    let scale = 10f64.powf(size.log10().floor());
    (size / scale).round() * scale
}

const ALGORITHMS_2D: [(Algorithm2d, &str); 4] = [
    (Algorithm2d::FrontalDelaunay, "Frontal-Delaunay"),
    (Algorithm2d::Delaunay, "Delaunay"),
    (Algorithm2d::MeshAdapt, "MeshAdapt"),
    (Algorithm2d::Automatic, "Automatic"),
];

/// The surface algorithms for an extrude mesh, those for quadrilaterals first.
const EXTRUDE_ALGORITHMS_2D: [(Algorithm2d, &str); 7] = [
    (Algorithm2d::FrontalQuads, "Frontal-Delaunay for quads"),
    (
        Algorithm2d::PackingOfParallelograms,
        "Packing of parallelograms",
    ),
    (Algorithm2d::QuasiStructuredQuad, "Quasi-structured quad"),
    (Algorithm2d::FrontalDelaunay, "Frontal-Delaunay"),
    (Algorithm2d::Delaunay, "Delaunay"),
    (Algorithm2d::MeshAdapt, "MeshAdapt"),
    (Algorithm2d::Automatic, "Automatic"),
];

const RECOMBINATIONS: [(Recombination, &str); 5] = [
    (Recombination::None, "None (wedges)"),
    (Recombination::Simple, "Simple"),
    (Recombination::Blossom, "Blossom"),
    (Recombination::SimpleFullQuad, "Simple full-quad"),
    (Recombination::BlossomFullQuad, "Blossom full-quad"),
];

const LAYER_TYPES: [&str; 3] = ["Scale factor", "Number of elements", "Multi-layered"];

fn layer_type(layers: &ExtrudeLayers) -> usize {
    match layers {
        ExtrudeLayers::ScaleFactor(_) => 0,
        ExtrudeLayers::NumberOfElements(_) => 1,
        ExtrudeLayers::MultiLayered { .. } => 2,
    }
}

/// Numbers separated by semicolons or spaces, the way layer sizes are written.
fn list_text<T: ToString>(values: &[T]) -> String {
    values
        .iter()
        .map(|v| format!("{};", v.to_string()))
        .collect::<Vec<_>>()
        .join(" ")
}

fn parse_list<T: std::str::FromStr>(text: &str) -> Option<Vec<T>> {
    (text.split([';', ' ']))
        .filter(|t| !t.trim().is_empty())
        .map(|t| t.trim().replace(',', ".").parse().ok())
        .collect()
}

const ALGORITHMS_3D: [(Algorithm3d, &str); 3] = [
    (Algorithm3d::Delaunay, "Delaunay"),
    (Algorithm3d::Frontal, "Frontal"),
    (Algorithm3d::Hxt, "HXT"),
];

/// The dialog to create or edit a mesh setup item. Parts, and the faces and edges of
/// a local mesh size, are clicked on the geometry.
pub struct MeshItemEditor {
    draft: MeshSetupItem,
    /// Index of the edited item; `None` creates a new one.
    index: Option<usize>,
    /// Names of the other items.
    others: Vec<String>,
    /// Faces and edges of a local mesh size.
    picks: History<CadEntity>,
    /// Parts of the other item types, written to the draft as they change.
    parts: PartPicks,
    /// The selection window, restricted to parts while they are picked.
    picker: Picker,
    error: Option<String>,
    /// Unit system of the model, the unit of the sizes.
    units: UnitSystem,
    /// An extrude mesh's source faces are picked, not found for whole parts.
    source_faces: bool,
    /// An extrude mesh's relative layer sizes and elements per layer as typed.
    layer_text: [String; 2],
    /// The parts of the picked source faces, for the faces they were found for.
    face_parts: (BTreeSet<CadEntity>, Vec<String>),
    /// Names of the geometry's shell parts, the parts a thicken shell mesh takes.
    shells: BTreeSet<String>,
}

pub enum MeshItemResult {
    Open,
    /// The item to store at the index, or to add.
    Ok(Option<usize>, MeshSetupItem),
    /// Like `Ok`, and remesh the parts the item applies to.
    OkRemesh(Option<usize>, MeshSetupItem),
    Cancel,
}

impl MeshItemEditor {
    pub fn create(geometry: &Geometry, units: UnitSystem) -> Self {
        let others: Vec<String> = geometry.mesh_items.iter().map(|i| i.name.clone()).collect();
        let kind = new_kind(0, geometry);
        let name = next_name(kind.type_name(), others.iter().map(String::as_str));
        Self {
            draft: MeshSetupItem { name, kind },
            index: None,
            others,
            picks: History::default(),
            parts: PartPicks::default(),
            picker: Picker::default(),
            error: None,
            units,
            source_faces: false,
            layer_text: ["0.2; 0.6; 0.2;".into(), "2; 4; 2;".into()],
            face_parts: Default::default(),
            shells: BTreeSet::new(),
        }
    }

    /// A new item of the kind at `index` of the dialog's list: meshing parameters of parts,
    /// local mesh size, tetrahedral Gmsh, extrude mesh or thicken shell mesh.
    pub fn create_kind(geometry: &Geometry, units: UnitSystem, index: usize) -> Self {
        let mut editor = Self::create(geometry, units);
        let kind = new_kind(index, geometry);
        editor.draft.name = next_name(kind.type_name(), editor.others.iter().map(String::as_str));
        editor.draft.kind = kind;
        editor
    }

    pub fn edit(geometry: &Geometry, index: usize, units: UnitSystem) -> Option<Self> {
        let draft = geometry.mesh_items.get(index)?.clone();
        let others = (geometry.mesh_items.iter().enumerate())
            .filter(|&(i, _)| i != index)
            .map(|(_, item)| item.name.clone())
            .collect();
        let picks = match &draft.kind {
            MeshSetupKind::LocalMeshSize { faces, edges, .. } => History::from_items(
                (faces.iter().map(|&f| CadEntity::Face(f)))
                    .chain(edges.iter().map(|&e| CadEntity::Edge(e))),
            ),
            MeshSetupKind::ExtrudeMesh(extrude) => {
                History::from_items(extrude.faces.iter().map(|&f| CadEntity::Face(f)))
            }
            _ => History::default(),
        };
        let parts = match &draft.kind {
            MeshSetupKind::ExtrudeMesh(extrude) if !extrude.faces.is_empty() => {
                PartPicks::default()
            }
            kind => PartPicks::from_names(kind.parts().unwrap_or_default().iter().cloned()),
        };
        let mut layer_text = ["0.2; 0.6; 0.2;".to_string(), "2; 4; 2;".to_string()];
        let mut source_faces = false;
        if let MeshSetupKind::ExtrudeMesh(extrude) = &draft.kind {
            source_faces = !extrude.faces.is_empty();
            if let ExtrudeLayers::MultiLayered { sizes, elements } = &extrude.layers {
                layer_text = [list_text(sizes), list_text(elements)];
            }
        }
        Some(Self {
            draft,
            index: Some(index),
            others,
            picks,
            parts,
            picker: Picker::default(),
            error: None,
            units,
            source_faces,
            layer_text,
            face_parts: Default::default(),
            shells: BTreeSet::new(),
        })
    }

    fn title(&self) -> String {
        match self.index {
            Some(_) => format!("Edit Mesh Setup Item: {}", self.draft.name),
            None => "Create Mesh Setup Item".into(),
        }
    }

    /// Whether clicks in the 3D view pick for this dialog.
    pub fn picks(&self) -> bool {
        true
    }

    pub fn show(
        &mut self,
        ctx: &egui::Context,
        geometry: &Geometry,
        view: Option<&Model>,
    ) -> MeshItemResult {
        let mut result = MeshItemResult::Open;
        let mut open = true;
        if let Some(view) = view {
            self.update_face_parts(view);
            self.shells = shell_parts(view);
        }
        let title = self.title();
        let builder = egui::Window::new(title.clone())
            .id(crate::ids::window(ctx, "mesh item editor"))
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::LEFT_TOP)
            .default_pos(ctx.content_rect().left_top() + egui::vec2(300.0, 90.0));
        let window = crate::dock::show(ctx, builder, title, &mut open, |ui| {
            ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
            ui.strong("Type");
            let current = type_index(&self.draft.kind);
            // The list of item types; an edited item keeps its type.
            egui::Frame::group(ui.style())
                .fill(crate::style::window())
                .show(ui, |ui| {
                    ui.set_min_width(360.0);
                    ui.add_enabled_ui(self.index.is_none(), |ui| {
                        for (index, label) in TYPES.into_iter().enumerate() {
                            if ui.selectable_label(index == current, label).clicked()
                                && index != current
                            {
                                self.draft.kind = new_kind(index, geometry);
                                self.draft.name = next_name(
                                    self.draft.kind.type_name(),
                                    self.others.iter().map(String::as_str),
                                );
                                self.picks.clear();
                                self.source_faces = false;
                                self.sync_parts();
                            }
                        }
                    });
                });
            ui.separator();
            ui.strong("Properties");
            egui::Grid::new("mesh item form")
                .max_col_width(crate::dock::column_limit(ui))
                .num_columns(2)
                .spacing([12.0, 6.0])
                .show(ui, |ui| self.form(ui, view));
            if let Some(error) = &self.error {
                ui.colored_label(crate::style::error(), error);
            }
            ui.separator();
            crate::dock::footer(ui, |ui| {
                if ui.button("Cancel").clicked() {
                    result = MeshItemResult::Cancel;
                }
                if ui.button("OK").clicked() {
                    match self.finish() {
                        Ok(item) => result = MeshItemResult::Ok(self.index, item),
                        Err(error) => self.error = Some(error),
                    }
                }
                if ui
                    .button("OK & Remesh")
                    .on_hover_text("Apply the item and mesh the parts it applies to again")
                    .clicked()
                {
                    match self.finish() {
                        Ok(item) => result = MeshItemResult::OkRemesh(self.index, item),
                        Err(error) => self.error = Some(error),
                    }
                }
            });
        });
        if let (Some(window), Some(view)) = (window, view) {
            let anchor = window.response.rect;
            if self.has_parts() {
                let can_undo = self.parts.can_undo();
                if let Some(action) = self.picker.parts_window(ctx, anchor, can_undo) {
                    self.parts.action(view, action);
                    self.sync_parts();
                }
            } else if let Some(region) = self.geometry_region() {
                let can_undo = self.picks.can_undo();
                if let Some(action) = self.picker.geometry_window(ctx, anchor, region, can_undo) {
                    self.entity_action(view, action);
                }
            }
        }
        if !open {
            result = MeshItemResult::Cancel;
        }
        result
    }

    /// Whether the item is made of whole parts.
    fn has_parts(&self) -> bool {
        match self.draft.kind {
            MeshSetupKind::LocalMeshSize { .. } => false,
            MeshSetupKind::ExtrudeMesh(_) => !self.source_faces,
            _ => true,
        }
    }

    /// Whether clicks pick faces or edges rather than parts.
    fn picks_entities(&self) -> bool {
        !self.has_parts()
    }

    /// What the picked CAD entities may be, when they are picked rather than parts.
    fn geometry_region(&self) -> Option<GeometryRegion> {
        match self.draft.kind {
            _ if self.has_parts() => None,
            MeshSetupKind::ExtrudeMesh(_) => Some(GeometryRegion::Faces),
            _ => Some(GeometryRegion::FacesAndEdges),
        }
    }

    /// Applies a button of the selection window to the picked faces and edges.
    fn entity_action(&mut self, view: &Model, action: PickerAction) {
        match action {
            PickerAction::Undo => self.picks.undo(),
            PickerAction::Clear => self.picks.clear(),
            PickerAction::All => {
                let faces = geometry_selection::visible_faces(view);
                self.picks.push(Operation::Replace, faces);
            }
            // The faces of the visible parts not picked take the place of those picked;
            // picked edges stay.
            PickerAction::Invert => {
                let picked = self.picks.items();
                let mut inverted: BTreeSet<CadEntity> = geometry_selection::visible_faces(view)
                    .difference(&picked)
                    .copied()
                    .collect();
                inverted.extend(picked.iter().filter(|e| matches!(e, CadEntity::Edge(_))));
                self.picks.push(Operation::Replace, inverted);
            }
            // The geometry window has no mesh based modes.
            PickerAction::Ids(..) => {}
        }
        self.update_face_parts(view);
    }

    /// Writes the picked parts to the draft.
    fn sync_parts(&mut self) {
        let source_faces = self.source_faces;
        match &mut self.draft.kind {
            MeshSetupKind::MeshingParameters { parts, .. }
            | MeshSetupKind::TetrahedralGmsh { parts, .. } => {
                *parts = self.parts.names().into_iter().collect();
            }
            MeshSetupKind::ThickenShellMesh(thicken) => {
                thicken.parts = self.parts.names().into_iter().collect();
            }
            MeshSetupKind::ExtrudeMesh(extrude) if source_faces => {
                extrude.parts = self.face_parts.1.clone();
            }
            MeshSetupKind::ExtrudeMesh(extrude) => {
                extrude.parts = self.parts.names().into_iter().collect();
            }
            MeshSetupKind::LocalMeshSize { .. } => {}
        }
    }

    /// Finds the parts of the picked source faces once they change.
    fn update_face_parts(&mut self, view: &Model) {
        if !matches!(self.draft.kind, MeshSetupKind::ExtrudeMesh(_)) {
            return;
        }
        let picked = self.picks.items();
        if picked == self.face_parts.0 && !picked.is_empty() {
            return;
        }
        let faces: BTreeSet<i32> = (picked.iter())
            .filter_map(|e| match e {
                CadEntity::Face(tag) => Some(*tag),
                _ => None,
            })
            .collect();
        let parts = parts_of_faces(view, &faces);
        self.face_parts = (picked, parts);
        self.sync_parts();
    }

    fn form(&mut self, ui: &mut egui::Ui, view: Option<&Model>) {
        ui.label("Name");
        ui.text_edit_singleline(&mut self.draft.name);
        ui.end_row();
        match &mut self.draft.kind {
            MeshSetupKind::MeshingParameters { parameters, .. } => {
                parts_ui(ui, &mut self.parts);
                let faces = reaches_faces(view, &self.parts.names());
                parameters_ui(ui, parameters, self.units, faces);
            }
            MeshSetupKind::LocalMeshSize { size, .. } => {
                ui.label("Region");
                ui.vertical(|ui| {
                    let picked = self.picks.items();
                    let faces = picked
                        .iter()
                        .filter(|e| matches!(e, CadEntity::Face(_)))
                        .count();
                    let edges = picked.len() - faces;
                    ui.horizontal(|ui| {
                        if picked.is_empty() {
                            ui.label("Empty");
                        } else {
                            ui.label(format!("{faces} faces, {edges} edges"));
                        }
                        if ui.button("Clear Selection").clicked() {
                            self.picks.clear();
                        }
                    });
                    ui.weak("Click faces and edges in the 3D view.");
                });
                ui.end_row();
                ui.label("Element size");
                ui.add(numeric::quantity(size, self.units, Quantity::Length).range(0.0..=f64::MAX));
                ui.end_row();
            }
            MeshSetupKind::TetrahedralGmsh {
                algorithm_2d,
                algorithm_3d,
                ..
            } => {
                parts_ui(ui, &mut self.parts);
                ui.label("Surface algorithm");
                algorithm_combo(ui, "algorithm 2d", algorithm_2d, &ALGORITHMS_2D);
                ui.end_row();
                ui.label("Volume algorithm");
                algorithm_combo(ui, "algorithm 3d", algorithm_3d, &ALGORITHMS_3D);
                ui.end_row();
            }
            MeshSetupKind::ThickenShellMesh(thicken) => {
                parts_ui(ui, &mut self.parts);
                ui.label("Thickness");
                ui.add(
                    numeric::quantity(&mut thicken.thickness, self.units, Quantity::Length)
                        .range(0.0..=f64::MAX),
                );
                ui.end_row();
                ui.label("Number of layers")
                    .on_hover_text("Layers of solid elements through the thickness.");
                ui.add(crate::numeric::count(&mut thicken.layers).range(1..=1000));
                ui.end_row();
                ui.label("Offset").on_hover_text(
                    "Shift of the solid's middle from the shell along the face normals: 0 \
                     centres it on the shell, half the thickness puts the shell on one side \
                     and minus half the thickness on the other.",
                );
                ui.add(numeric::quantity(
                    &mut thicken.offset,
                    self.units,
                    Quantity::Length,
                ));
                ui.end_row();
            }
            MeshSetupKind::ExtrudeMesh(extrude) => {
                ui.label("Source faces").on_hover_text(
                    "Automatic finds for each part a face it is a straight extrusion of. \
                     Selected: click the faces at one end of the parts.",
                );
                let mut selected = self.source_faces;
                egui::ComboBox::from_id_salt("extrude source")
                    .selected_text(if selected {
                        "Selected faces"
                    } else {
                        "Automatic"
                    })
                    .width(crate::dock::fit_width(ui, 160.0))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut selected, false, "Automatic");
                        ui.selectable_value(&mut selected, true, "Selected faces");
                    });
                ui.end_row();
                if selected != self.source_faces {
                    self.source_faces = selected;
                    self.picks.clear();
                    self.parts.clear();
                    self.face_parts = Default::default();
                }
                if self.source_faces {
                    ui.label("Faces");
                    ui.vertical(|ui| {
                        ui.horizontal(|ui| {
                            let faces = self.picks.items().len();
                            match faces {
                                0 => ui.label("Empty"),
                                1 => ui.label("1 face"),
                                _ => ui.label(format!("{faces} faces")),
                            };
                            if ui.button("Clear Selection").clicked() {
                                self.picks.clear();
                            }
                        });
                        if !self.face_parts.1.is_empty() {
                            ui.weak(format!("Parts: {}", self.face_parts.1.join(", ")));
                        }
                        ui.weak("Click the faces at one end of the parts.");
                    });
                    ui.end_row();
                } else {
                    parts_ui(ui, &mut self.parts);
                }
                ui.label("Surface algorithm");
                algorithm_combo(
                    ui,
                    "extrude algorithm 2d",
                    &mut extrude.algorithm_2d,
                    &EXTRUDE_ALGORITHMS_2D,
                );
                ui.end_row();
                ui.label("Recombine algorithm").on_hover_text(
                    "How triangles are joined to quadrilaterals, which become hexahedra; \
                     triangles become wedges. Full-quad gives quadrilaterals only.",
                );
                algorithm_combo(
                    ui,
                    "extrude recombination",
                    &mut extrude.recombination,
                    &RECOMBINATIONS,
                );
                ui.end_row();
                ui.label("Layers defined by")
                    .on_hover_text("The element size in the extrude direction.");
                let mut kind = layer_type(&extrude.layers);
                egui::ComboBox::from_id_salt("extrude layers")
                    .selected_text(LAYER_TYPES[kind])
                    .width(crate::dock::fit_width(ui, 160.0))
                    .show_ui(ui, |ui| {
                        for (index, label) in LAYER_TYPES.into_iter().enumerate() {
                            ui.selectable_value(&mut kind, index, label);
                        }
                    });
                ui.end_row();
                if kind != layer_type(&extrude.layers) {
                    extrude.layers = match kind {
                        0 => ExtrudeLayers::ScaleFactor(1.0),
                        1 => ExtrudeLayers::NumberOfElements(4),
                        _ => ExtrudeLayers::MultiLayered {
                            sizes: Vec::new(),
                            elements: Vec::new(),
                        },
                    };
                }
                match &mut extrude.layers {
                    ExtrudeLayers::ScaleFactor(factor) => {
                        ui.label("Scale factor").on_hover_text(
                            "Element size in the extrude direction as a multiple of the \
                             part's max. element size.",
                        );
                        ui.add(numeric::drag_value(factor).range(0.0..=1000.0));
                        ui.end_row();
                    }
                    ExtrudeLayers::NumberOfElements(n) => {
                        ui.label("Number of elements");
                        ui.add(crate::numeric::count(n).range(1..=10_000));
                        ui.end_row();
                    }
                    ExtrudeLayers::MultiLayered { .. } => {
                        ui.label("Relative layer sizes")
                            .on_hover_text("For example 0.2; 0.6; 0.2;");
                        ui.text_edit_singleline(&mut self.layer_text[0]);
                        ui.end_row();
                        ui.label("Elements per layer")
                            .on_hover_text("For example 2; 4; 2;");
                        ui.text_edit_singleline(&mut self.layer_text[1]);
                        ui.end_row();
                    }
                }
            }
        }
        self.sync_parts();
    }

    /// The item as entered, or what is missing.
    fn finish(&self) -> Result<MeshSetupItem, String> {
        let mut item = self.draft.clone();
        item.name = item.name.trim().to_string();
        if item.name.is_empty() {
            return Err("The name is missing.".into());
        }
        if self
            .others
            .iter()
            .any(|o| o.eq_ignore_ascii_case(&item.name))
        {
            return Err(format!("The name {} is already in use.", item.name));
        }
        match &mut item.kind {
            MeshSetupKind::MeshingParameters { parts, parameters } => {
                if parts.is_empty() {
                    return Err("No part selected.".into());
                }
                if parameters.max_size <= 0.0 {
                    return Err("The max. element size must be greater than 0.".into());
                }
            }
            MeshSetupKind::TetrahedralGmsh { parts, .. } => {
                if parts.is_empty() {
                    return Err("No part selected.".into());
                }
            }
            MeshSetupKind::ThickenShellMesh(thicken) => {
                if thicken.parts.is_empty() {
                    return Err("No shell part selected.".into());
                }
                if let Some(solid) = (thicken.parts.iter()).find(|p| !self.shells.contains(*p)) {
                    return Err(format!(
                        "{solid} is no shell: only shells can be thickened."
                    ));
                }
                thicken.check().map_err(|e| format!("{e}."))?;
            }
            MeshSetupKind::ExtrudeMesh(extrude) => {
                extrude.faces = if self.source_faces {
                    (self.picks.items().iter())
                        .filter_map(|e| match e {
                            CadEntity::Face(tag) => Some(*tag),
                            _ => None,
                        })
                        .collect()
                } else {
                    Vec::new()
                };
                if self.source_faces && extrude.faces.is_empty() {
                    return Err("No source face selected.".into());
                }
                if extrude.parts.is_empty() {
                    return Err("No part selected.".into());
                }
                if let ExtrudeLayers::MultiLayered { sizes, elements } = &mut extrude.layers {
                    let parsed = (parse_list::<f64>(&self.layer_text[0]))
                        .zip(parse_list::<u32>(&self.layer_text[1]));
                    let Some((s, e)) = parsed else {
                        return Err("Not all layer sizes or element counts are numbers.".into());
                    };
                    (*sizes, *elements) = (s, e);
                }
                extrude
                    .layers
                    .levels(1.0, 1.0)
                    .map_err(|e| format!("{e}."))?;
            }
            MeshSetupKind::LocalMeshSize { faces, edges, size } => {
                let picked = self.picks.items();
                *faces = (picked.iter())
                    .filter_map(|e| match e {
                        CadEntity::Face(tag) => Some(*tag),
                        _ => None,
                    })
                    .collect();
                *edges = (picked.iter())
                    .filter_map(|e| match e {
                        CadEntity::Edge(tag) => Some(*tag),
                        _ => None,
                    })
                    .collect();
                if picked.is_empty() {
                    return Err("No face or edge selected.".into());
                }
                if *size <= 0.0 {
                    return Err("The element size must be greater than 0.".into());
                }
            }
        }
        Ok(item)
    }

    /// A click on the geometry: picks the clicked part, the face or edge for a local mesh
    /// size, or the source face of an extrude mesh.
    pub fn click(&mut self, view: &Model, pick: Option<(&Hit, f32)>, operation: Operation) {
        self.shells = shell_parts(view);
        if self.has_parts() {
            self.parts.click(view, pick.map(|(hit, _)| hit), operation);
            self.sync_parts();
            return;
        }
        let Some(region) = self.geometry_region() else {
            return;
        };
        match pick {
            Some((hit, precision)) => {
                let picked = geometry_selection::pick(view, &self.picker, region, hit, precision);
                if !picked.is_empty() {
                    self.picks.push(operation, picked);
                    self.update_face_parts(view);
                }
            }
            // As in the selection of the FE model, a plain click into empty space clears the
            // selection.
            None if operation == Operation::Replace => self.picks.clear(),
            None => {}
        }
    }

    /// A selection box dragged on the geometry picks the parts, or the faces, inside it.
    pub fn box_select(&mut self, view: &Model, area: &BoxSelect, operation: Operation) {
        if self.has_parts() {
            self.parts.box_select(view, area, operation);
            self.sync_parts();
        } else {
            self.picks
                .push(operation, geometry_selection::pick_box(view, area));
            self.update_face_parts(view);
        }
    }

    /// What a click at the hit would select, for the hover preview.
    pub fn preview(&self, view: &Model, hit: &Hit, precision: f32) -> Preview {
        match self.geometry_region() {
            None => PartPicks::preview(view, hit),
            Some(region) => geometry_selection::preview(view, &self.picker, region, hit, precision),
        }
    }

    /// The parts, faces and edges of the item, shown red on the geometry.
    pub fn highlight(&self, view: &Model) -> Highlight {
        if self.picks_entities() {
            entities_highlight(view, &self.picks.items())
        } else {
            item_highlight(view, &self.draft.kind)
        }
    }
}

fn parts_ui(ui: &mut egui::Ui, parts: &mut PartPicks) {
    ui.label("Parts");
    // The only selection of the dialog: clicks always pick for it.
    parts.ui(ui, true);
    ui.end_row();
}

fn algorithm_combo<T: Copy + PartialEq>(
    ui: &mut egui::Ui,
    id: &str,
    value: &mut T,
    choices: &[(T, &str)],
) {
    let label = (choices.iter())
        .find(|(v, _)| v == value)
        .map_or("", |(_, l)| *l);
    egui::ComboBox::from_id_salt(id)
        .selected_text(label)
        .width(crate::dock::fit_width(ui, 160.0))
        .show_ui(ui, |ui| {
            for &(choice, label) in choices {
                ui.selectable_value(value, choice, label);
            }
        });
}

/// Names of the parts of faces outside the solids.
fn shell_parts(view: &Model) -> BTreeSet<String> {
    (view.parts.iter())
        .filter(|p| p.dimension == Some(2))
        .map(|p| p.name.clone())
        .collect()
}

/// Names of the parts the CAD faces lie on.
fn parts_of_faces(view: &Model, faces: &BTreeSet<i32>) -> Vec<String> {
    if faces.is_empty() {
        return Vec::new();
    }
    (view.mesh.parts.iter().zip(&view.parts))
        .filter(|(part, _)| {
            part.elements.iter().any(
                |&id| matches!(view.cad_entity(id), Some(CadEntity::Face(tag)) if faces.contains(&tag)),
            )
        })
        .map(|(_, info)| info.name.clone())
        .collect()
}

/// The CAD edge near the hit point, else the CAD face under it.
pub fn cad_pick(view: &Model, hit: &Hit, precision: f32) -> Option<CadEntity> {
    let mesh = &view.mesh;
    if hit.line {
        let element = view.skin(hit.part).line_elements[hit.face];
        return view.cad_entity(mesh.elements()[element].id);
    }
    let position = |id: NodeId| mesh.node_index(id).map(|n| view.render_position(n));
    let mut nearest: Option<(f32, CadEntity)> = None;
    for &id in &mesh.parts.get(hit.part)?.elements {
        let Some(element) = mesh.element(id) else {
            continue;
        };
        if element.shape != ElementShape::Line2 {
            continue;
        }
        let (Some(a), Some(b)) = (position(element.nodes[0]), position(element.nodes[1])) else {
            continue;
        };
        let ab = b - a;
        let t =
            ((hit.point - a).dot(ab) / ab.length_squared().max(f32::MIN_POSITIVE)).clamp(0.0, 1.0);
        let distance = (a + ab * t).distance(hit.point);
        if distance <= precision
            && nearest.is_none_or(|(d, _)| distance < d)
            && let Some(entity) = view.cad_entity(id)
        {
            nearest = Some((distance, entity));
        }
    }
    if let Some((_, entity)) = nearest {
        return Some(entity);
    }
    let face = view.skin(hit.part).faces.get(hit.face)?;
    view.cad_entity(mesh.elements()[face.element].id)
}

/// Element faces of the display that show a CAD face.
fn face_items(view: &Model, tags: &BTreeSet<i32>) -> std::collections::HashSet<(u32, u8)> {
    view.cad_face_items(tags)
}

/// Segments of the display that show a CAD edge, as node pairs.
fn edge_segments(view: &Model, tags: &BTreeSet<i32>) -> Vec<[NodeId; 2]> {
    view.cad_edge_segments(tags)
}

/// The outlines of CAD faces and the lines of CAD edges, for the hover preview.
pub fn entities_preview(view: &Model, entities: &BTreeSet<CadEntity>) -> Preview {
    let (mut faces, mut edges) = (BTreeSet::new(), BTreeSet::new());
    for entity in entities {
        match *entity {
            CadEntity::Face(tag) => faces.insert(tag),
            CadEntity::Edge(tag) => edges.insert(tag),
            // The display has no vertices to pick.
            CadEntity::Vertex(_) => false,
        };
    }
    let mut preview = Preview::default();
    if !faces.is_empty() {
        let faces = face_items(view, &faces);
        preview = crate::selection::preview(
            view,
            &crate::selection::Items::Faces(faces.into_iter().collect()),
        );
    }
    let position = |id| view.mesh.node_index(id).map(|n| view.render_position(n));
    preview.lines.extend(
        (edge_segments(view, &edges).into_iter())
            .filter_map(|[a, b]| Some([position(a)?, position(b)?])),
    );
    preview
}

pub fn entities_highlight(view: &Model, entities: &BTreeSet<CadEntity>) -> Highlight {
    let (mut faces, mut edges) = (BTreeSet::new(), BTreeSet::new());
    for entity in entities {
        match *entity {
            CadEntity::Face(tag) => faces.insert(tag),
            CadEntity::Edge(tag) => edges.insert(tag),
            CadEntity::Vertex(_) => false,
        };
    }
    Highlight {
        faces: face_items(view, &faces),
        lines: edge_segments(view, &edges),
        ..Highlight::default()
    }
}

/// How a stored mesh setup item shows on the geometry.
pub fn item_highlight(view: &Model, kind: &MeshSetupKind) -> Highlight {
    match kind {
        MeshSetupKind::MeshingParameters { parts, .. }
        | MeshSetupKind::TetrahedralGmsh { parts, .. }
        | MeshSetupKind::ThickenShellMesh(ThickenShellMesh { parts, .. }) => Highlight {
            parts: (view.parts.iter().enumerate())
                .filter(|(_, p)| parts.contains(&p.name))
                .map(|(i, _)| i)
                .collect(),
            ..Highlight::default()
        },
        MeshSetupKind::LocalMeshSize { faces, edges, .. } => {
            let entities = (faces.iter().map(|&f| CadEntity::Face(f)))
                .chain(edges.iter().map(|&e| CadEntity::Edge(e)))
                .collect();
            entities_highlight(view, &entities)
        }
        MeshSetupKind::ExtrudeMesh(extrude) if !extrude.faces.is_empty() => {
            let entities = extrude.faces.iter().map(|&f| CadEntity::Face(f)).collect();
            entities_highlight(view, &entities)
        }
        MeshSetupKind::ExtrudeMesh(extrude) => Highlight {
            parts: (view.parts.iter().enumerate())
                .filter(|(_, p)| extrude.parts.contains(&p.name))
                .map(|(i, _)| i)
                .collect(),
            ..Highlight::default()
        },
    }
}

/// The meshes of some parts, generated one after the other.
pub struct PartMeshes {
    pub meshes: Vec<GeneratedMesh>,
}

/// The argument that makes the program a meshing worker instead of opening its window.
const WORKER_ARGUMENT: &str = "--mesh-worker";

/// The program meshes run in, set by `main`; without one (in tests) they run on a thread.
static WORKER: std::sync::OnceLock<PathBuf> = std::sync::OnceLock::new();

/// Meshes run in a separate process of `program`, which [`worker_main`] handles, so that
/// cancelling ends them at once: Gmsh itself cannot be interrupted.
pub fn set_worker(program: PathBuf) {
    let _ = WORKER.set(program);
}

/// What the worker meshes, written to `request.ron` in its folder.
#[derive(Serialize, Deserialize)]
struct WorkerRequest {
    /// Gmsh library of the program, so that both mesh alike.
    library: Option<PathBuf>,
    geometry: Geometry,
    /// The parts to mesh, all with `None`.
    parts: Option<Vec<String>>,
}

/// The worker's answer in `result.ron`: per part the mesh and Gmsh's warnings.
type WorkerResult = Result<Vec<(FeMesh, Vec<String>)>, String>;

/// Meshes the named parts, or all with `None`, one after the other.
fn mesh_parts(geometry: &Geometry, parts: Option<Vec<String>>) -> Result<PartMeshes, String> {
    let parts = match parts {
        // A compound part is meshed as the parts of the geometry it joins.
        Some(parts) => {
            let mut expanded: Vec<String> = Vec::new();
            for part in parts.iter().flat_map(|p| geometry.parts_of_fe_part(p)) {
                if !expanded.iter().any(|e| e.eq_ignore_ascii_case(&part)) {
                    expanded.push(part);
                }
            }
            expanded
        }
        None => plx_mesher::part_names(geometry).map_err(|e| e.to_string())?,
    };
    let mut meshes = Vec::with_capacity(parts.len());
    for part in parts {
        let mesh = plx_mesher::generate_part_mesh(geometry, &part).map_err(|e| e.to_string())?;
        meshes.push(mesh);
    }
    Ok(PartMeshes { meshes })
}

/// Runs the meshing worker if the program was started as one, and returns its exit code;
/// `None` for a normal start. The worker ends itself at once, see [`plx_solve::process::end`].
pub fn worker_main() -> Option<i32> {
    let mut args = std::env::args_os().skip(1);
    if args.next()? != WORKER_ARGUMENT {
        return None;
    }
    let dir = PathBuf::from(args.next()?);
    let program = args.next().and_then(|id| id.to_str()?.parse::<u32>().ok());
    // The worker must not mesh on for nobody once the program ends, even if it crashes.
    plx_solve::process::end_with(program);
    let result: WorkerResult = (|| {
        let text = std::fs::read_to_string(dir.join("request.ron")).map_err(|e| e.to_string())?;
        let request: WorkerRequest = ron::from_str(&text).map_err(|e| e.to_string())?;
        if request.library.is_some() {
            plx_mesher::set_library_path(request.library);
        }
        let generated = mesh_parts(&request.geometry, request.parts)?;
        Ok((generated.meshes.into_iter())
            .map(|g| (g.mesh, g.warnings))
            .collect())
    })();
    // Written under another name first, so that the program never reads half a result.
    let written = ron::to_string(&result)
        .map_err(|e| e.to_string())
        .and_then(|text| std::fs::write(dir.join("result.tmp"), text).map_err(|e| e.to_string()))
        .and_then(|()| {
            std::fs::rename(dir.join("result.tmp"), dir.join(RESULT)).map_err(|e| e.to_string())
        });
    plx_solve::process::end(if written.is_ok() { 0 } else { 1 })
}

/// The file the worker leaves its result in, see [`WorkerResult`].
const RESULT: &str = "result.ron";

/// A folder of its own for one worker, removed when dropped.
struct WorkerDir(PathBuf);

impl WorkerDir {
    fn new() -> std::io::Result<Self> {
        static COUNT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let count = COUNT.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("nodalis-mesh-{}-{count}", std::process::id()));
        std::fs::create_dir_all(&dir)?;
        Ok(Self(dir))
    }
}

impl Drop for WorkerDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// Meshes in a worker process of `program` until it is done or `cancel` is set, which kills
/// it.
fn mesh_in_worker(
    program: &Path,
    geometry: Geometry,
    parts: Option<Vec<String>>,
    cancel: &AtomicBool,
) -> Result<PartMeshes, String> {
    let dir = WorkerDir::new().map_err(|e| format!("No folder for the mesher: {e}"))?;
    let request = WorkerRequest {
        library: plx_mesher::library_path(),
        geometry,
        parts,
    };
    let text = ron::to_string(&request).map_err(|e| e.to_string())?;
    std::fs::write(dir.0.join("request.ron"), text)
        .map_err(|e| format!("Cannot write the mesher's input: {e}"))?;
    let mut command = std::process::Command::new(program);
    command
        .arg(WORKER_ARGUMENT)
        .arg(&dir.0)
        .arg(std::process::id().to_string())
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null());
    plx_solve::hide_console(&mut command);
    let mut child = command
        .spawn()
        .map_err(|e| format!("Cannot start the mesher: {e}"))?;
    let result = dir.0.join(RESULT);
    // The result counts once it is there, whether or not the worker has ended yet.
    let text = loop {
        if cancel.load(Ordering::Relaxed) {
            let _ = child.kill();
            let _ = child.wait();
            return Err(CANCELLED.into());
        }
        if let Ok(text) = std::fs::read_to_string(&result) {
            let _ = child.kill();
            let _ = child.wait();
            break text;
        }
        match child.try_wait() {
            // It may have left its result just before it ended.
            Ok(Some(status)) => match std::fs::read_to_string(&result) {
                Ok(text) => break text,
                Err(_) => return Err(format!("The mesher stopped unexpectedly ({status})")),
            },
            Ok(None) => std::thread::sleep(std::time::Duration::from_millis(50)),
            Err(e) => return Err(format!("The mesher could not be watched: {e}")),
        }
    };
    let result: WorkerResult = ron::from_str(&text).map_err(|e| e.to_string())?;
    Ok(PartMeshes {
        meshes: (result?.into_iter())
            .map(|(mesh, warnings)| GeneratedMesh { mesh, warnings })
            .collect(),
    })
}

/// What a cancelled job reports.
pub const CANCELLED: &str = "Meshing cancelled";

/// Mesh generation running on a worker thread, which waits for the worker process. Dropping
/// the job cancels it.
pub struct MeshingJob {
    pub started: Instant,
    /// The geometry as it is meshed, with its meshing settings: what the new meshes are
    /// made from, even if the settings change while they are generated.
    pub geometry: Geometry,
    receiver: Receiver<Result<PartMeshes, String>>,
    cancel: Arc<AtomicBool>,
}

impl MeshingJob {
    /// Meshes the named parts, or all parts with `None`.
    pub fn start(geometry: Geometry, parts: Option<Vec<String>>, ctx: &egui::Context) -> Self {
        let (sender, receiver) = std::sync::mpsc::channel();
        let meshed = geometry.clone();
        let ctx = ctx.clone();
        let cancel = Arc::new(AtomicBool::new(false));
        let cancelled = Arc::clone(&cancel);
        std::thread::spawn(move || {
            let result = match WORKER.get() {
                Some(program) => mesh_in_worker(program, geometry, parts, &cancelled),
                None => mesh_parts(&geometry, parts),
            };
            let _ = sender.send(result);
            ctx.request_repaint();
        });
        Self {
            started: Instant::now(),
            geometry: meshed,
            receiver,
            cancel,
        }
    }

    /// The result once the worker is done.
    pub fn poll(&self) -> Option<Result<PartMeshes, String>> {
        match self.receiver.try_recv() {
            Ok(result) => Some(result),
            Err(std::sync::mpsc::TryRecvError::Empty) => None,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => Some(Err(CANCELLED.into())),
        }
    }
}

impl Drop for MeshingJob {
    fn drop(&mut self) {
        self.cancel.store(true, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use glam::Vec3;

    use super::*;
    use crate::selection::SelectBy;

    fn geometry_view() -> Option<(Geometry, Model)> {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/platte_mit_loch.step");
        match crate::model::load(&path, plx_model::UnitSystem::Unitless) {
            Ok(loaded) => Some((loaded.model.geometry?, loaded.geometry_view?)),
            Err(error) if std::env::var_os("NODALIS_REQUIRE_GMSH").is_none() => {
                eprintln!("Gmsh not available, test skipped: {error}");
                None
            }
            Err(error) => panic!("Gmsh is required: {error}"),
        }
    }

    /// The hit of a ray straight down onto the plate (100 x 40 x 10, centred in render
    /// coordinates) at x, y.
    fn hit(view: &Model, x: f32, y: f32) -> Hit {
        view.pick(Vec3::new(x, y, 50.0), Vec3::NEG_Z).unwrap()
    }

    #[test]
    fn clicks_pick_cad_faces_and_edges_for_a_local_mesh_size() {
        let Some((geometry, view)) = geometry_view() else {
            return;
        };
        let top = cad_pick(&view, &hit(&view, -30.0, 0.0), 0.5);
        assert!(matches!(top, Some(CadEntity::Face(_))), "{top:?}");
        // Close to the long edge at y = -20 the edge wins.
        let edge = cad_pick(&view, &hit(&view, -30.0, -19.8), 0.5);
        assert!(matches!(edge, Some(CadEntity::Edge(_))), "{edge:?}");
        assert_eq!(cad_pick(&view, &hit(&view, 30.0, 0.0), 0.5), top);

        let mut editor = MeshItemEditor::create(&geometry, UnitSystem::MmTonSC);
        editor.draft.kind = new_kind(1, &geometry);
        let (face_hit, edge_hit) = (hit(&view, -30.0, 0.0), hit(&view, -30.0, -19.8));
        editor.click(&view, Some((&face_hit, 0.5)), Operation::Replace);
        editor.click(&view, Some((&edge_hit, 0.5)), Operation::Add);
        let highlight = editor.highlight(&view);
        assert!(!highlight.faces.is_empty() && !highlight.lines.is_empty());
        let preview = editor.preview(&view, &edge_hit, 0.5);
        assert!(!preview.lines.is_empty());
        let item = editor.finish().unwrap();
        let MeshSetupKind::LocalMeshSize { faces, edges, size } = item.kind else {
            panic!("{item:?}");
        };
        assert_eq!((faces.len(), edges.len()), (1, 1));
        assert_eq!(size, geometry.meshing.max_size / 4.0);
        editor.click(&view, None, Operation::Replace);
        assert!(editor.finish().is_err(), "a click into empty space clears");

        // The modes of the selection window: the part takes all its seven faces, the
        // surface angle only the top face, which meets the others at right angles.
        let faces = |editor: &MeshItemEditor| match editor.finish().unwrap().kind {
            MeshSetupKind::LocalMeshSize { faces, edges, .. } => (faces.len(), edges.len()),
            kind => panic!("{kind:?}"),
        };
        editor.picker.select_by = SelectBy::GeometryPart;
        editor.click(&view, Some((&face_hit, 0.5)), Operation::Replace);
        assert_eq!(faces(&editor), (7, 0));
        editor.picker.select_by = SelectBy::GeometrySurfaceAngle;
        editor.click(&view, Some((&face_hit, 0.5)), Operation::Replace);
        assert_eq!(faces(&editor), (1, 0));
        editor.entity_action(&view, PickerAction::Invert);
        assert_eq!(faces(&editor), (6, 0));
        editor.entity_action(&view, PickerAction::Clear);
        assert!(editor.finish().is_err());
        editor.entity_action(&view, PickerAction::All);
        assert_eq!(faces(&editor), (7, 0));
        // The edge angle mode takes the straight edge alone, its neighbours turn by 90°.
        editor.picker.select_by = SelectBy::GeometryEdgeAngle;
        editor.click(&view, Some((&edge_hit, 0.5)), Operation::Replace);
        assert_eq!(faces(&editor), (0, 1));

        // A box takes the faces inside it, dragged to the left also those it crosses.
        let area = |crossing| BoxSelect {
            // A view from above: x and y as they are, depth from 0 to 1 over z = ±100.
            view_proj: glam::Mat4::from_translation(Vec3::new(0.0, 0.0, 0.5))
                * glam::Mat4::from_scale(Vec3::new(1.0 / 60.0, 1.0 / 30.0, 1.0 / 200.0)),
            min: glam::Vec2::new(-1.0, -1.0),
            max: glam::Vec2::new(-0.01, 1.0),
            crossing,
            shift: false,
            ctrl: false,
        };
        editor.box_select(&view, &area(false), Operation::Replace);
        assert_eq!(faces(&editor), (1, 0), "the end face at x = -50");
        editor.box_select(&view, &area(true), Operation::Replace);
        assert_eq!(faces(&editor), (6, 0), "all but the end face at x = 50");
    }

    #[test]
    fn extrude_meshes_pick_source_faces_or_parts() {
        let Some((geometry, view)) = geometry_view() else {
            return;
        };
        let mut editor = MeshItemEditor::create(&geometry, UnitSystem::MmTonSC);
        editor.draft.kind = new_kind(3, &geometry);
        // Automatic: the part is clicked.
        let top = hit(&view, -30.0, 0.0);
        editor.click(&view, Some((&top, 0.5)), Operation::Replace);
        let item = editor.finish().unwrap();
        let MeshSetupKind::ExtrudeMesh(extrude) = &item.kind else {
            panic!("{item:?}");
        };
        assert_eq!(
            (extrude.parts.as_slice(), extrude.faces.len()),
            (&["SOLID-1".to_string()][..], 0)
        );

        // Selected faces: the top face, also when clicked close to an edge; its part follows.
        editor.source_faces = true;
        editor.parts.clear();
        editor.update_face_parts(&view);
        editor.sync_parts();
        assert!(editor.finish().is_err(), "no face yet");
        let near_edge = hit(&view, -30.0, -19.8);
        editor.click(&view, Some((&near_edge, 0.5)), Operation::Replace);
        assert!(!editor.preview(&view, &top, 0.5).lines.is_empty());
        let item = editor.finish().unwrap();
        let MeshSetupKind::ExtrudeMesh(extrude) = &item.kind else {
            panic!("{item:?}");
        };
        assert_eq!(
            extrude.faces,
            vec![
                cad_pick(&view, &top, 0.5)
                    .map(|e| match e {
                        CadEntity::Face(tag) => tag,
                        _ => 0,
                    })
                    .unwrap()
            ]
        );
        assert_eq!(extrude.parts, vec!["SOLID-1".to_string()]);
        assert!(!editor.highlight(&view).faces.is_empty());

        // Multi-layered sizes are typed as a list.
        if let MeshSetupKind::ExtrudeMesh(extrude) = &mut editor.draft.kind {
            extrude.layers = ExtrudeLayers::MultiLayered {
                sizes: Vec::new(),
                elements: Vec::new(),
            };
        }
        editor.layer_text = ["0,2; 0.8".into(), "1; 3;".into()];
        let item = editor.finish().unwrap();
        let MeshSetupKind::ExtrudeMesh(extrude) = &item.kind else {
            panic!("{item:?}");
        };
        assert_eq!(
            extrude.layers,
            ExtrudeLayers::MultiLayered {
                sizes: vec![0.2, 0.8],
                elements: vec![1, 3]
            }
        );
        editor.layer_text[1] = "1; x".into();
        assert!(editor.finish().is_err());
        let mut with_item = geometry.clone();
        with_item.mesh_items.push(item);
        let edited = MeshItemEditor::edit(&with_item, 0, UnitSystem::MmTonSC).unwrap();
        assert!(edited.source_faces);
        assert_eq!(edited.layer_text[0], "0.2; 0.8;");
    }

    #[test]
    fn only_shells_can_be_thickened() {
        let Some((geometry, view)) = geometry_view() else {
            return;
        };
        let mut editor = MeshItemEditor::create_kind(&geometry, UnitSystem::MmTonSC, 4);
        assert_eq!(editor.draft.name, "Thicken_Shell_Mesh-1");
        let MeshSetupKind::ThickenShellMesh(thicken) = &editor.draft.kind else {
            panic!("{:?}", editor.draft.kind);
        };
        assert!(thicken.thickness > 0.0 && thicken.layers == 2 && thicken.offset == 0.0);
        assert!(editor.finish().is_err(), "no part yet");
        // The plate is a solid.
        editor.click(
            &view,
            Some((&hit(&view, -30.0, 0.0), 0.5)),
            Operation::Replace,
        );
        let error = editor.finish().unwrap_err();
        assert!(error.contains("no shell"), "{error}");
        editor.shells.insert("SOLID-1".into());
        let item = editor.finish().unwrap();
        assert_eq!(item.kind.parts().unwrap(), ["SOLID-1"]);
        if let MeshSetupKind::ThickenShellMesh(thicken) = &mut editor.draft.kind {
            thicken.layers = 0;
        }
        assert!(editor.finish().is_err());
    }

    #[test]
    fn faces_named_in_a_gmsh_error_show_red() {
        let Some((_, view)) = geometry_view() else {
            return;
        };
        let message = "Gmsh: Invalid boundary mesh (overlapping facets) on surface 1 curve 2";
        let entities = plx_mesher::named_entities(message).into_iter().collect();
        let highlight = entities_highlight(&view, &entities);
        assert!(!highlight.faces.is_empty() && !highlight.lines.is_empty());
        let face: BTreeSet<_> = (highlight.faces.iter())
            .map(|&(e, _)| view.cad_entity(e))
            .collect();
        assert_eq!(face, BTreeSet::from([Some(CadEntity::Face(1))]));
    }

    #[test]
    fn part_items_take_clicked_parts_and_need_one() {
        let Some((geometry, view)) = geometry_view() else {
            return;
        };
        let mut editor = MeshItemEditor::create(&geometry, UnitSystem::MmTonSC);
        assert_eq!(editor.draft.name, "Meshing_Parameters-1");
        assert!(editor.finish().is_err(), "no part chosen");
        let hit = hit(&view, -30.0, 0.0);
        editor.click(&view, Some((&hit, 0.5)), Operation::Replace);
        let item = editor.finish().unwrap();
        assert!(matches!(
            &item.kind,
            MeshSetupKind::MeshingParameters { parts, .. } if parts == &["SOLID-1"]
        ));
        assert_eq!(item_highlight(&view, &item.kind).parts.len(), 1);
        // Ctrl+click takes it out again, as in every selection.
        editor.click(&view, Some((&hit, 0.5)), Operation::Subtract);
        assert!(editor.finish().is_err());

        let mut with_item = geometry.clone();
        with_item.mesh_items.push(item);
        let edited = MeshItemEditor::edit(&with_item, 0, UnitSystem::MmTonSC).unwrap();
        assert!(edited.finish().is_ok(), "its own name is no clash");
        let mut other = MeshItemEditor::create(&with_item, UnitSystem::MmTonSC);
        assert_eq!(other.draft.name, "Meshing_Parameters-2");
        other.draft.name = "meshing_parameters-1".into();
        other.click(&view, Some((&hit, 0.5)), Operation::Replace);
        assert!(other.finish().is_err(), "names are unique");
    }
}
