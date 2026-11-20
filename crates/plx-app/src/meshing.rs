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
    Algorithm2d, Algorithm3d, ExtrudeLayers, ExtrudeMesh, FeModel, Field, Geometry, ItemRef,
    MeshSetupItem, MeshSetupKind, MeshingParameters, PeriodicFaces, Quantity, Recombination,
    ThickenShellMesh, UnitSystem, next_name,
};
use serde::{Deserialize, Serialize};

use crate::expose;
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
///
/// Each number has the expose button of a surrounding [`expose::Scope`]; an option exposed is
/// 1 for on and 0 for off.
fn parameters_ui(ui: &mut egui::Ui, d: &mut MeshingParameters, units: UnitSystem, faces: bool) {
    ui.label("Max. element size");
    expose::field(ui, Field::MaxElementSize, |ui| {
        ui.add(numeric::quantity(&mut d.max_size, units, Quantity::Length).range(0.0..=f64::MAX))
    });
    ui.end_row();
    ui.label("Min. element size");
    expose::field(ui, Field::MinElementSize, |ui| {
        ui.add(numeric::quantity(&mut d.min_size, units, Quantity::Length).range(0.0..=f64::MAX))
    });
    ui.end_row();
    ui.label("Elements per curvature radius")
        .on_hover_text("0 turns off the refinement on curved surfaces.");
    expose::field(ui, Field::ElementsPerCurvature, |ui| {
        ui.add(numeric::drag_value(&mut d.elements_per_curvature).range(0.0..=100.0))
    });
    ui.end_row();
    ui.label("Mesh type");
    ui.vertical(|ui| {
        expose::field(ui, Field::SecondOrder, |ui| {
            // The element types in the hover text keep the row short enough for its
            // expose button in the docked dialog.
            ui.checkbox(&mut d.second_order, "Second order")
                .on_hover_text("Quadratic elements: C3D10, S6/S8, CPS6/CPS8.")
        });
        let second_order = d.second_order;
        expose::field(ui, Field::MidsideNodes, |ui| {
            ui.add_enabled(
                second_order,
                egui::Checkbox::new(
                    &mut d.midside_nodes_on_geometry,
                    "Midside nodes on geometry",
                ),
            )
        });
        expose::field(ui, Field::Optimize, |ui| {
            ui.checkbox(&mut d.optimize, "Optimize mesh (Netgen)")
        });
        expose::field(ui, Field::Quadrilaterals, |ui| {
            ui.add_enabled(
                faces,
                egui::Checkbox::new(&mut d.quad_dominated, "Quadrilaterals on faces"),
            )
            .on_hover_text(
                "Mesh faces (2D models and shells) with quadrilaterals in a mostly regular \
                 grid, by Gmsh's quasi-structured algorithm; off for triangles. A Tetrahedral \
                 Gmsh item chooses another algorithm, whose triangles are joined to \
                 quadrilaterals. Solids keep triangles on their faces, as tetrahedra need \
                 them.",
            )
            .on_disabled_hover_text(
                "Only for faces (2D models and shells): the parts are solids, meshed with \
                 tetrahedra, which need triangles on their faces.",
            )
        });
        expose::field(ui, Field::Hexahedra, |ui| {
            ui.checkbox(&mut d.hexahedra_where_possible, "Hexahedra where possible")
                .on_hover_text(
                    "Parts that can be swept get hexahedra: stacks of extrusions along one \
                     direction (stepped shafts, plates with bosses, pockets and \
                     counterbores), straight extrusions of a face and bodies of revolution. \
                     The others get tetrahedra, with a warning that says why.",
                )
        });
    });
    ui.end_row();
}

/// The numbers of meshing parameters, each with an expose button.
const PARAMETER_FIELDS: [Field; 8] = [
    Field::MaxElementSize,
    Field::MinElementSize,
    Field::ElementsPerCurvature,
    Field::SecondOrder,
    Field::MidsideNodes,
    Field::Optimize,
    Field::Quadrilaterals,
    Field::Hexahedra,
];

/// The expose buttons of a dialog of the mesh setup: the scope of `item` in `fe`, kept from
/// frame to frame in `kept`, with `available` fields.
fn exposing<R>(
    ui: &mut egui::Ui,
    kept: &mut Option<expose::Scope>,
    fe: &FeModel,
    item: Option<ItemRef>,
    linked: &[(u32, String)],
    available: &[Field],
    form: impl FnOnce(&mut egui::Ui) -> R,
) -> R {
    let mut scope = kept
        .take()
        .unwrap_or_else(|| expose::Scope::new(fe, item, linked));
    scope.available = available.to_vec();
    let result = scope.show(ui, form);
    *kept = Some(scope);
    result
}

/// The open window of the default meshing parameters, which apply to every part that no
/// Meshing Parameters item covers.
pub struct MeshSetupWindow {
    draft: MeshingParameters,
    /// The expose buttons of its numbers, see [`Self::set_linked`].
    expose: Option<expose::Scope>,
    /// The exposed parameters of the analysis that value links give, with where from.
    linked: Vec<(u32, String)>,
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
            expose: None,
            linked: Vec::new(),
            faces: reaches_faces(view, &BTreeSet::new()),
            units,
        }
    }

    /// The exposed parameters value links give, by key, with the blocks they come from: the
    /// window shows those numbers read-only.
    pub fn set_linked(&mut self, linked: Vec<(u32, String)>) {
        if self.linked != linked {
            self.linked = linked;
            self.expose = None;
        }
    }

    /// The fields clicked to be exposed or no longer, to apply once the parameters are
    /// taken over.
    pub fn take_expose(&mut self) -> Option<expose::Scope> {
        self.expose.take()
    }

    pub fn show(&mut self, ctx: &egui::Context, fe: &FeModel) -> MeshSetupResult {
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
                let item = Some(ItemRef::DefaultMeshing);
                exposing(
                    ui,
                    &mut self.expose,
                    fe,
                    item,
                    &self.linked,
                    &PARAMETER_FIELDS,
                    |ui| {
                        egui::Grid::new("mesh size")
                            .num_columns(2)
                            .spacing([12.0, 6.0])
                            .show(ui, |ui| parameters_ui(ui, d, units, faces));
                    },
                );
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
            depth: 0.0,
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
    (Algorithm2d::QuasiStructuredQuad, "Quasi-structured quad"),
    (Algorithm2d::FrontalQuads, "Frontal-Delaunay for quads"),
    (
        Algorithm2d::PackingOfParallelograms,
        "Packing of parallelograms",
    ),
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
    /// The parts of the geometry with their ids, which the item keeps them by.
    part_ids: Vec<plx_model::PartId>,
    /// The id of the edited item, for its exposed parameters.
    original: Option<plx_model::Id<plx_model::MeshSetupItem>>,
    /// The expose buttons of its numbers, see [`Self::set_linked`].
    expose: Option<expose::Scope>,
    /// The exposed parameters of the analysis that value links give, with where from.
    linked: Vec<(u32, String)>,
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
            draft: MeshSetupItem {
                id: plx_model::Id::new(),
                name,
                kind,
            },
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
            part_ids: geometry.ids.clone(),
            original: None,
            expose: None,
            linked: Vec::new(),
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
            // The mesh setup keeps the parts by the ids of the geometry, as the display.
            kind => PartPicks::from_parts(
                (kind.parts().unwrap_or_default().iter())
                    .map(|&id| (id, geometry.part_name(id).unwrap_or_default().to_owned())),
            ),
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
            part_ids: geometry.ids.clone(),
            original: Some(geometry.mesh_items.get(index)?.id),
            expose: None,
            linked: Vec::new(),
        })
    }

    /// The exposed parameters value links give, by key, with the blocks they come from: the
    /// dialog shows those numbers read-only.
    pub fn set_linked(&mut self, linked: Vec<(u32, String)>) {
        if self.linked != linked {
            self.linked = linked;
            self.expose = None;
        }
    }

    /// The fields clicked to be exposed or no longer, with the item they were clicked for
    /// as the geometry had it, to apply once the item is stored.
    pub fn take_expose(&mut self) -> Option<expose::Scope> {
        self.expose.take()
    }

    /// The numbers of the item that have an expose button.
    fn exposable(&self) -> &'static [Field] {
        match self.draft.kind {
            MeshSetupKind::MeshingParameters { .. } => &PARAMETER_FIELDS,
            MeshSetupKind::LocalMeshSize { .. } => &[Field::ElementSize, Field::Depth],
            _ => &[],
        }
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
        fe: &FeModel,
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
            let item = self.original.map(ItemRef::MeshSetup);
            let available = self.exposable();
            let mut kept = self.expose.take();
            let linked = std::mem::take(&mut self.linked);
            exposing(ui, &mut kept, fe, item, &linked, available, |ui| {
                egui::Grid::new("mesh item form")
                    .max_col_width(crate::dock::column_limit(ui))
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| self.form(ui, view));
            });
            self.expose = kept;
            self.linked = linked;
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
                *parts = self.parts.ids().into_iter().collect();
            }
            MeshSetupKind::ThickenShellMesh(thicken) => {
                thicken.parts = self.parts.ids().into_iter().collect();
            }
            MeshSetupKind::ExtrudeMesh(extrude) if source_faces => {
                let ids = &self.part_ids;
                extrude.parts = (self.face_parts.1.iter())
                    .filter_map(|name| ids.iter().find(|p| p.name.eq_ignore_ascii_case(name)))
                    .map(|p| p.id)
                    .collect();
            }
            MeshSetupKind::ExtrudeMesh(extrude) => {
                extrude.parts = self.parts.ids().into_iter().collect();
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
            MeshSetupKind::LocalMeshSize { size, depth, .. } => {
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
                expose::field(ui, Field::ElementSize, |ui| {
                    ui.add(
                        numeric::quantity(size, self.units, Quantity::Length).range(0.0..=f64::MAX),
                    )
                });
                ui.end_row();
                ui.label("Depth").on_hover_text(
                    "How deep below the selected faces the element size holds inside the \
                     solid, for a layer such as the walls of an infill section. 0 refines \
                     the faces alone.",
                );
                expose::field(ui, Field::Depth, |ui| {
                    ui.add(
                        numeric::quantity(depth, self.units, Quantity::Length)
                            .range(0.0..=f64::MAX),
                    )
                });
                ui.end_row();
            }
            MeshSetupKind::TetrahedralGmsh {
                algorithm_2d,
                algorithm_3d,
                ..
            } => {
                parts_ui(ui, &mut self.parts);
                ui.label("Surface algorithm").on_hover_text(
                    "For the faces of solids and for shells and 2D models. Quadrilaterals on \
                     faces are then joined from this algorithm's triangles instead of made \
                     by the quasi-structured algorithm.",
                );
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
                    "Automatic finds how each part is swept: as a stack of extrusions along \
                     one direction, with steps, pockets and holes of any depth, or as the \
                     straight extrusion of a face. Selected: click the faces at one end of \
                     the parts.",
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
                // The quasi-structured algorithm makes its quadrilaterals itself.
                let recombines = extrude.algorithm_2d != Algorithm2d::QuasiStructuredQuad;
                ui.add_enabled_ui(recombines, |ui| {
                    algorithm_combo(
                        ui,
                        "extrude recombination",
                        &mut extrude.recombination,
                        &RECOMBINATIONS,
                    );
                })
                .response
                .on_disabled_hover_text(
                    "The quasi-structured algorithm gives quadrilaterals only, without \
                     joining triangles.",
                );
                ui.end_row();
                ui.label("Layers defined by").on_hover_text(
                    "The element size in the extrude direction. In a stack of \
                         extrusions the layers are counted over the whole height; every \
                         step gets a layer boundary and at least one layer.",
                );
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
                let name = |id: &plx_model::PartRef| {
                    (self.part_ids.iter().find(|p| p.id == *id))
                        .map_or_else(|| id.to_string(), |p| p.name.clone())
                };
                if let Some(solid) =
                    (thicken.parts.iter().map(name)).find(|p| !self.shells.contains(p))
                {
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
            MeshSetupKind::LocalMeshSize {
                faces,
                edges,
                size,
                depth,
            } => {
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
                if *depth < 0.0 || depth.is_nan() {
                    return Err("The depth must not be negative.".into());
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

    /// A selection box dragged on the geometry picks the parts, or the faces and edges,
    /// inside it.
    pub fn box_select(&mut self, view: &Model, area: &BoxSelect, operation: Operation) {
        match self.geometry_region() {
            None => {
                self.parts.box_select(view, area, operation);
                self.sync_parts();
            }
            Some(region) => {
                let picked = geometry_selection::pick_box(view, &self.picker, region, area);
                self.picks.push(operation, picked);
                self.update_face_parts(view);
            }
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
            item_highlight(view, &self.draft.kind, &self.part_ids)
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
    parts_of_entities(view, faces, &BTreeSet::new())
}

/// The names of the parts that hold any of the CAD faces or edges.
pub fn parts_of_entities(
    view: &Model,
    faces: &BTreeSet<i32>,
    edges: &BTreeSet<i32>,
) -> Vec<String> {
    if faces.is_empty() && edges.is_empty() {
        return Vec::new();
    }
    (view.mesh().parts.iter().zip(&view.parts))
        .filter(|(part, _)| {
            part.elements.iter().any(|&id| match view.cad_entity(id) {
                Some(CadEntity::Face(tag)) => faces.contains(&tag),
                Some(CadEntity::Edge(tag)) => edges.contains(&tag),
                _ => false,
            })
        })
        .map(|(_, info)| info.name.clone())
        .collect()
}

/// The CAD edge near the hit point, else the CAD face under it.
pub fn cad_pick(view: &Model, hit: &Hit, precision: f32) -> Option<CadEntity> {
    let mesh = &view.mesh();
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
    let position = |id| view.mesh().node_index(id).map(|n| view.render_position(n));
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

/// How a stored mesh setup item shows on the geometry; `ids` are the parts of the geometry
/// with their ids, which the item keeps them by.
pub fn item_highlight(view: &Model, kind: &MeshSetupKind, ids: &[plx_model::PartId]) -> Highlight {
    let holds = |parts: &[plx_model::PartRef], name: &str| {
        (ids.iter()).any(|p| parts.contains(&p.id) && p.name.eq_ignore_ascii_case(name))
    };
    match kind {
        MeshSetupKind::MeshingParameters { parts, .. }
        | MeshSetupKind::TetrahedralGmsh { parts, .. }
        | MeshSetupKind::ThickenShellMesh(ThickenShellMesh { parts, .. }) => Highlight {
            parts: (view.parts.iter().enumerate())
                .filter(|(_, p)| holds(parts, &p.name))
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
                .filter(|(_, p)| holds(&extrude.parts, &p.name))
                .map(|(i, _)| i)
                .collect(),
            ..Highlight::default()
        },
    }
}

/// The meshes of some parts, generated one after the other.
#[derive(Default)]
pub struct PartMeshes {
    pub meshes: Vec<GeneratedMesh>,
    /// Why the parts that could not be meshed failed, one message each, naming the part:
    /// a part that fails leaves the others to be meshed.
    pub failed: Vec<String>,
    /// Cancelled before all parts were done; the meshes are those finished before.
    pub cancelled: bool,
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

/// What a meshing job meshes, written to `request.ron` in the job's folder for all its
/// workers.
#[derive(Serialize, Deserialize)]
struct WorkerRequest {
    /// Gmsh library of the program, so that both mesh alike.
    library: Option<PathBuf>,
    geometry: Geometry,
    /// The cut faces of cyclic symmetries, meshed alike.
    #[serde(default)]
    matching: Vec<PeriodicFaces>,
    /// The parts to mesh, in order, as [`parts_to_mesh`] lists them.
    parts: Vec<String>,
    /// The parts as the workers take them, see [`part_groups`]: places in `parts`.
    #[serde(default)]
    groups: Vec<Vec<usize>>,
}

/// What one worker does of a job, written to `task.ron` in the worker's own folder.
#[derive(Serialize, Deserialize)]
struct WorkerTask {
    /// The job's folder, with `request.ron` and the marks of the groups taken.
    job: PathBuf,
    /// The parts it meshes first, as places in the request's parts: what is left of a
    /// group whose worker crashed.
    first: Vec<usize>,
    /// Whether it then takes groups of the job, one after the other, until none is left.
    take_groups: bool,
}

/// What became of one part in the worker, in `part-K.ron` for the K-th part of the request:
/// its mesh and Gmsh's warnings, or why it failed.
type PartOutcome = Result<(FeMesh, Vec<String>), String>;

/// The worker's answer in `result.ron` once it is done; an error if it could not mesh at all.
type WorkerResult = Result<(), String>;

/// The parts that are meshed when the named parts are asked for, or all with `None`: a
/// compound part is meshed as the parts of the geometry it joins, and suppressed parts are
/// left out.
fn parts_to_mesh(geometry: &Geometry, parts: Option<Vec<String>>) -> Result<Vec<String>, String> {
    let parts = match parts {
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
    // Suppressed parts are left alone, with the mesh they may have.
    Ok((parts.into_iter())
        .filter(|p| !geometry.is_suppressed(p))
        .collect())
}

/// The parts as the workers of a job take them, as places in `parts`: each part on its own,
/// but the parts of a compound part together, since its shells may be meshed as one, the
/// first of them standing for the others.
fn part_groups(geometry: &Geometry, parts: &[String]) -> Vec<Vec<usize>> {
    let mut groups: Vec<(Option<usize>, Vec<usize>)> = Vec::new();
    for (k, part) in parts.iter().enumerate() {
        let compound = geometry.compound_of(part);
        match groups
            .iter_mut()
            .find(|(of, _)| compound.is_some() && *of == compound)
        {
            Some((_, group)) => group.push(k),
            None => groups.push((compound, vec![k])),
        }
    }
    groups.into_iter().map(|(_, group)| group).collect()
}

/// Meshes the listed parts one after the other, with the cut faces of the cyclic symmetries
/// `matching` meshed alike, and hands each outcome to `outcome` with the part's place in the
/// list; `starting` hears of each part before Gmsh starts on it. A part that cannot be meshed
/// does not stop the others. Parts meshed with an earlier one (the shells of a compound
/// part) are skipped.
fn mesh_list(
    geometry: &Geometry,
    matching: &[PeriodicFaces],
    parts: &[String],
    starting: &mut dyn FnMut(usize),
    outcome: &mut dyn FnMut(usize, Result<GeneratedMesh, String>),
) {
    let mut meshed: Vec<String> = Vec::new();
    for (k, part) in parts.iter().enumerate() {
        if meshed.iter().any(|p| p.eq_ignore_ascii_case(part)) {
            continue;
        }
        starting(k);
        let result = plx_mesher::generate_part_mesh_matching(geometry, part, matching)
            .map_err(|error| named_failure(part, &error.to_string()));
        if let Ok(mesh) = &result {
            meshed.extend(mesh.mesh.parts.iter().map(|p| p.name.clone()));
        }
        outcome(k, result);
    }
}

/// A failure message that begins with the part's name; most messages name it already.
fn named_failure(part: &str, message: &str) -> String {
    if message.starts_with(&format!("{part}: ")) {
        message.to_string()
    } else {
        format!("{part}: {message}")
    }
}

/// Meshes the named parts, or all with `None`, one after the other, with the cut faces of
/// the cyclic symmetries `matching` meshed alike; suppressed parts are left out. A part that
/// cannot be meshed is listed in [`PartMeshes::failed`] and the next one is meshed.
fn mesh_parts(
    geometry: &Geometry,
    matching: &[PeriodicFaces],
    parts: Option<Vec<String>>,
) -> Result<PartMeshes, String> {
    let parts = parts_to_mesh(geometry, parts)?;
    let mut generated = PartMeshes {
        meshes: Vec::with_capacity(parts.len()),
        ..PartMeshes::default()
    };
    mesh_list(
        geometry,
        matching,
        &parts,
        &mut |_| {},
        &mut |_, result| match result {
            Ok(mesh) => generated.meshes.push(mesh),
            Err(message) => generated.failed.push(message),
        },
    );
    Ok(generated)
}

/// Runs the meshing worker if the program was started as one, and returns its exit code;
/// `None` for a normal start. The worker ends itself at once, see [`plx_solve::process::end`].
///
/// It meshes the parts its task names and then takes groups of parts of the job, each by
/// leaving a mark in the job's folder that only one worker can leave, until all are taken:
/// the workers of a job so share the parts among themselves as they get done with them.
///
/// It tells in `current` which part Gmsh works on and leaves each part's outcome in a file of
/// its own as soon as it has it, so that if Gmsh crashes, the program knows which part
/// crashed it and keeps the parts meshed before. Gmsh's messages go to its standard output,
/// which the program reads to tell where in the part Gmsh was.
pub fn worker_main() -> Option<i32> {
    let mut args = std::env::args_os().skip(1);
    if args.next()? != WORKER_ARGUMENT {
        return None;
    }
    let dir = PathBuf::from(args.next()?);
    let program = args.next().and_then(|id| id.to_str()?.parse::<u32>().ok());
    // The worker must not mesh on for nobody once the program ends, even if it crashes.
    plx_solve::process::end_with(program);
    plx_mesher::log_to_terminal();
    let result: WorkerResult = (|| {
        let text = std::fs::read_to_string(dir.join(TASK)).map_err(|e| e.to_string())?;
        let task: WorkerTask = ron::from_str(&text).map_err(|e| e.to_string())?;
        let text = std::fs::read_to_string(task.job.join(REQUEST)).map_err(|e| e.to_string())?;
        let request: WorkerRequest = ron::from_str(&text).map_err(|e| e.to_string())?;
        if request.library.is_some() {
            plx_mesher::set_library_path(request.library.clone());
        }
        let mut written = Ok(());
        // The parts this worker is done with, for the progress shown.
        let done = std::cell::Cell::new(0usize);
        // Meshes the parts at the given places of the request.
        let mut mesh = |places: &[usize]| {
            let names: Vec<String> = (places.iter())
                .filter_map(|&place| request.parts.get(place).cloned())
                .collect();
            mesh_list(
                &request.geometry,
                &request.matching,
                &names,
                &mut |k| {
                    let _ =
                        std::fs::write(dir.join(CURRENT), format!("{} {}", places[k], done.get()));
                },
                &mut |k, result| {
                    done.set(done.get() + 1);
                    let outcome: PartOutcome = result.map(|g| (g.mesh, g.warnings));
                    if written.is_ok() {
                        written = write_whole(&dir, &part_file(places[k]), &outcome);
                    }
                },
            );
        };
        mesh(&task.first);
        if task.take_groups {
            for (g, group) in request.groups.iter().enumerate() {
                if take_group(&task.job, g) {
                    mesh(group);
                }
            }
        }
        written
    })();
    let written = write_whole(&dir, RESULT, &result);
    plx_solve::process::end(if written.is_ok() { 0 } else { 1 })
}

/// Takes the `g`-th group of a job for the worker that asks: true for the first that asks,
/// which leaves a mark in the job's folder, false for all after it.
fn take_group(job: &Path, g: usize) -> bool {
    std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(job.join(format!("group-{g}")))
        .is_ok()
}

/// Writes `value` to `name` in `dir` under another name first, so that the program never
/// reads half of it.
fn write_whole(dir: &Path, name: &str, value: &impl Serialize) -> Result<(), String> {
    let text = ron::to_string(value).map_err(|e| e.to_string())?;
    let temporary = dir.join(format!("{name}.tmp"));
    std::fs::write(&temporary, text).map_err(|e| e.to_string())?;
    std::fs::rename(&temporary, dir.join(name)).map_err(|e| e.to_string())
}

/// The file of a job's [`WorkerRequest`], in the job's folder.
const REQUEST: &str = "request.ron";

/// The file of a worker's [`WorkerTask`], in its own folder.
const TASK: &str = "task.ron";

/// The file the worker leaves its result in, see [`WorkerResult`].
const RESULT: &str = "result.ron";

/// The file that holds the place in the request of the part Gmsh is meshing and, after a
/// space, how many parts the worker meshed before it.
const CURRENT: &str = "current";

/// The file Gmsh's messages go to.
const GMSH_LOG: &str = "gmsh.log";
/// Where the log of the last worker that crashed is kept, in the temporary folder.
const CRASH_LOG: &str = "nodalis-gmsh-crash.log";

/// The file of the K-th part's outcome, see [`PartOutcome`].
fn part_file(k: usize) -> String {
    format!("part-{k}.ron")
}

/// A folder of its own for one worker or one job, removed when dropped.
struct WorkerDir(PathBuf);

impl WorkerDir {
    fn new() -> std::io::Result<Self> {
        static COUNT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let count = COUNT.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("nodalis-mesh-{}-{count}", std::process::id()));
        std::fs::create_dir_all(&dir)?;
        Ok(Self(dir))
    }

    /// A folder with the request of a job in it.
    fn of_job(request: &WorkerRequest) -> Result<Self, String> {
        let dir = Self::new().map_err(|e| format!("No folder for the mesher: {e}"))?;
        let text = ron::to_string(request).map_err(|e| e.to_string())?;
        std::fs::write(dir.0.join(REQUEST), text)
            .map_err(|e| format!("Cannot write the mesher's input: {e}"))?;
        Ok(dir)
    }
}

impl Drop for WorkerDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// The most worker processes a meshing job runs at the same time, from the settings;
/// [`AUTOMATIC_PROCESSES`] for as many as the computer has cores.
static PROCESSES: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

/// [`set_processes`] for as many worker processes as the computer has cores.
pub const AUTOMATIC_PROCESSES: u32 = 0;

/// Sets the most worker processes a meshing job runs at the same time, for the jobs started
/// from now on.
pub fn set_processes(processes: u32) {
    PROCESSES.store(processes, Ordering::Relaxed);
}

/// The worker processes of [`AUTOMATIC_PROCESSES`]: one per processor core.
pub fn automatic_processes() -> usize {
    std::thread::available_parallelism().map_or(1, |n| n.get())
}

/// The worker processes a job of so many groups of parts runs: one per group, up to the
/// number set.
fn worker_count(groups: usize) -> usize {
    let most = match PROCESSES.load(Ordering::Relaxed) {
        AUTOMATIC_PROCESSES => automatic_processes(),
        set => set as usize,
    };
    most.min(groups).max(1)
}

/// What the workers of one job made so far, each of them adding to it.
#[derive(Default)]
struct Outcomes {
    /// The parts done, by their place in the request: a mesh, or why it failed.
    parts: Vec<(usize, Result<GeneratedMesh, String>)>,
    /// Cancelled before all parts were done.
    cancelled: bool,
}

/// Meshes in worker processes of `program` until they are done or `cancel` is set, which
/// kills them. The parts are meshed by several workers at the same time, see
/// [`worker_count`], which take them group by group ([`part_groups`]) as they get done with
/// the ones before: the parts of an assembly are independent of each other, and Gmsh
/// meshes a part on one core. If Gmsh crashes on a part, the part is told as failed with
/// where Gmsh was, and a new worker goes on in place of the one that crashed.
fn mesh_in_worker(
    program: &Path,
    geometry: Geometry,
    matching: Vec<PeriodicFaces>,
    parts: Option<Vec<String>>,
    cancel: &AtomicBool,
    progress: &dyn Fn(MeshProgress),
) -> Result<PartMeshes, String> {
    let parts = parts_to_mesh(&geometry, parts)?;
    let groups = part_groups(&geometry, &parts);
    let workers = worker_count(groups.len());
    let request = WorkerRequest {
        library: plx_mesher::library_path(),
        geometry,
        matching,
        parts,
        groups,
    };
    let job = WorkerDir::of_job(&request)?;
    // The part each worker meshes now, and the parts it is done with.
    let board: std::sync::Mutex<Vec<(Option<usize>, usize)>> =
        std::sync::Mutex::new(vec![(None, 0); workers]);
    let (sender, receiver) = std::sync::mpsc::channel::<MeshProgress>();
    // One failing worker stops the others.
    let failed = AtomicBool::new(false);
    let stopped = || cancel.load(Ordering::Relaxed) || failed.load(Ordering::Relaxed);
    let results: Vec<Result<Outcomes, String>> = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..workers)
            .map(|worker| {
                let (request, job, board, sender) = (&request, &job.0, &board, sender.clone());
                let (stopped, failed) = (&stopped, &failed);
                scope.spawn(move || {
                    let tell = |current: Option<usize>, done: usize| {
                        let mut board = board.lock().unwrap_or_else(|e| e.into_inner());
                        board[worker] = (current, done);
                        let now: Vec<String> = (board.iter())
                            .filter_map(|&(k, _)| request.parts.get(k?).cloned())
                            .collect();
                        let _ = sender.send(MeshProgress {
                            done: board.iter().map(|&(_, done)| done).sum(),
                            total: request.parts.len(),
                            parts: now,
                        });
                    };
                    let result = mesh_as_worker(program, request, job, stopped, &tell);
                    if result.is_err() {
                        failed.store(true, Ordering::Relaxed);
                    }
                    result
                })
            })
            .collect();
        drop(sender);
        // Told from this thread, the one that was asked.
        for now in receiver {
            progress(now);
        }
        (handles.into_iter())
            .map(|handle| {
                handle
                    .join()
                    .unwrap_or_else(|_| Err("A meshing thread stopped unexpectedly".into()))
            })
            .collect()
    });
    let mut all = Outcomes::default();
    for result in results {
        let outcomes = result?;
        all.parts.extend(outcomes.parts);
        all.cancelled |= outcomes.cancelled;
    }
    // In the order of the parts, whichever worker meshed them.
    all.parts.sort_by_key(|(place, _)| *place);
    let mut generated = PartMeshes {
        cancelled: all.cancelled,
        ..PartMeshes::default()
    };
    for (_, outcome) in all.parts {
        match outcome {
            Ok(mesh) => generated.meshes.push(mesh),
            Err(message) => generated.failed.push(message),
        }
    }
    Ok(generated)
}

/// One of the workers of a job: runs worker processes one after the other, a new one
/// whenever Gmsh crashed the one before, until the job has no groups left to take. The part
/// that crashed Gmsh is meshed once more without Netgen, and the new worker begins with
/// what was left of its group.
fn mesh_as_worker(
    program: &Path,
    request: &WorkerRequest,
    job: &Path,
    stopped: &dyn Fn() -> bool,
    current: &dyn Fn(Option<usize>, usize),
) -> Result<Outcomes, String> {
    let mut outcomes = Outcomes::default();
    let mut task = WorkerTask {
        job: job.to_path_buf(),
        first: Vec::new(),
        take_groups: true,
    };
    loop {
        let total = request.parts.len();
        // The parts of the worker processes before this one count as done too.
        let before = outcomes.parts.len();
        let tell = |now: Option<usize>, done: usize| current(now, before + done);
        let crashed = run_worker(program, &task, total, stopped, &tell, &mut outcomes);
        current(None, outcomes.parts.len());
        let Some((k, message)) = crashed? else {
            break;
        };
        if outcomes.cancelled {
            break;
        }
        mesh_without_netgen(program, request, k, message, stopped, &mut outcomes)?;
        if outcomes.cancelled {
            break;
        }
        // What is left of the group of the part that crashed Gmsh: the parts after it, but
        // not those meshed with others.
        let meshed: Vec<&str> = (outcomes.parts.iter())
            .filter_map(|(_, outcome)| outcome.as_ref().ok())
            .flat_map(|m| m.mesh.parts.iter().map(|p| p.name.as_str()))
            .collect();
        let group = (request.groups.iter()).find(|group| group.contains(&k));
        task.first = (group.into_iter().flatten())
            .skip_while(|&&place| place != k)
            .skip(1)
            .copied()
            .filter(|&place| {
                let part = &request.parts[place];
                !meshed.iter().any(|m| m.eq_ignore_ascii_case(part))
            })
            .collect();
    }
    Ok(outcomes)
}

/// Meshes the part at place `k` of the request, on which Gmsh crashed with `message`, once
/// more without the Netgen optimization, which is where Gmsh crashes most; the part fails
/// with `message` if it crashes again or was not optimized anyway.
fn mesh_without_netgen(
    program: &Path,
    request: &WorkerRequest,
    k: usize,
    message: String,
    stopped: &dyn Fn() -> bool,
    outcomes: &mut Outcomes,
) -> Result<(), String> {
    let part = &request.parts[k];
    let parameters = request.geometry.parameters(part);
    if !parameters.optimize {
        outcomes.parts.push((k, Err(message)));
        return Ok(());
    }
    let mut geometry = request.geometry.clone();
    geometry.mesh_items.push(plx_model::MeshSetupItem {
        id: plx_model::Id::new(),
        name: "Without Netgen".into(),
        kind: plx_model::MeshSetupKind::MeshingParameters {
            parts: request.geometry.part_id(part).into_iter().collect(),
            // The parameters the part has, with the sizes before any refinement.
            parameters: plx_model::MeshingParameters {
                optimize: false,
                ..unrefined(&request.geometry, part)
            },
        },
    });
    // A job of its own, of the one part.
    let retry = WorkerRequest {
        library: request.library.clone(),
        geometry,
        matching: request.matching.clone(),
        parts: vec![part.to_string()],
        groups: Vec::new(),
    };
    let job = WorkerDir::of_job(&retry)?;
    let task = WorkerTask {
        job: job.0.clone(),
        first: vec![0],
        take_groups: false,
    };
    let mut again = Outcomes::default();
    run_worker(program, &task, 1, stopped, &|_, _| {}, &mut again)?;
    outcomes.cancelled |= again.cancelled;
    // Only the first failure is told, which is what the user meshed with.
    let mesh = (again.parts.into_iter()).find_map(|(_, outcome)| outcome.ok());
    outcomes.parts.push((
        k,
        match mesh {
            Some(mut mesh) => {
                mesh.warnings.push(format!(
                    "{message}. It was meshed again without Optimize mesh (Netgen), which worked"
                ));
                Ok(mesh)
            }
            None if outcomes.cancelled => Err(message),
            None => Err(format!(
                "{message}. Meshed again without Optimize mesh (Netgen), it failed as well"
            )),
        },
    ));
    Ok(())
}

/// The meshing parameters of `part` as set, before the refinement of the geometry.
fn unrefined(geometry: &Geometry, part: &str) -> plx_model::MeshingParameters {
    let mut parameters = geometry.parameters(part);
    let refinement = geometry.refined(1.0);
    parameters.max_size /= refinement;
    parameters.min_size /= refinement;
    parameters
}

/// Runs one worker process on `task`, of a job of `total` parts, and adds what it made to
/// `outcomes`; returns the place in the request of the part it crashed on, with the message
/// that tells so. `current` hears which part it meshes and how many it meshed before. Once `stopped` says so, it is
/// killed: the parts it finished before are kept and `outcomes` is marked cancelled.
fn run_worker(
    program: &Path,
    task: &WorkerTask,
    total: usize,
    stopped: &dyn Fn() -> bool,
    current: &dyn Fn(Option<usize>, usize),
    outcomes: &mut Outcomes,
) -> Result<Option<(usize, String)>, String> {
    let dir = WorkerDir::new().map_err(|e| format!("No folder for the mesher: {e}"))?;
    let text = ron::to_string(task).map_err(|e| e.to_string())?;
    std::fs::write(dir.0.join(TASK), text)
        .map_err(|e| format!("Cannot write the mesher's input: {e}"))?;
    let log = std::fs::File::create(dir.0.join(GMSH_LOG))
        .map_err(|e| format!("Cannot write the mesher's log: {e}"))?;
    let mut command = std::process::Command::new(program);
    command
        .arg(WORKER_ARGUMENT)
        .arg(&dir.0)
        .arg(std::process::id().to_string())
        .stdin(std::process::Stdio::piped())
        .stdout(log)
        .stderr(std::process::Stdio::null());
    plx_solve::hide_console(&mut command);
    let mut child = command
        .spawn()
        .map_err(|e| format!("Cannot start the mesher: {e}"))?;
    let result = dir.0.join(RESULT);
    // The part it works on and how many it meshed before.
    let read_current = || {
        let text = std::fs::read_to_string(dir.0.join(CURRENT)).ok()?;
        let mut numbers = text.split_whitespace().map(str::parse::<usize>);
        let place = numbers.next()?.ok().filter(|&c| c < total)?;
        Some((place, numbers.next().and_then(Result::ok).unwrap_or(0)))
    };
    let mut told = None;
    let mut cancelled = false;
    // The result counts once it is there, whether or not the worker has ended yet.
    let (text, status) = loop {
        // Which part it works on now, for the progress shown.
        let now = read_current();
        if let Some((place, done)) = now
            && now != told
        {
            told = now;
            current(Some(place), done);
        }
        if stopped() {
            let _ = child.kill();
            let _ = child.wait();
            cancelled = true;
            break (None, None);
        }
        if let Ok(text) = std::fs::read_to_string(&result) {
            let _ = child.kill();
            let _ = child.wait();
            break (Some(text), None);
        }
        match child.try_wait() {
            // It may have left its result just before it ended.
            Ok(Some(status)) => break (std::fs::read_to_string(&result).ok(), Some(status)),
            Ok(None) => std::thread::sleep(std::time::Duration::from_millis(100)),
            Err(e) => return Err(format!("The mesher could not be watched: {e}")),
        }
    };
    outcomes.cancelled |= cancelled;
    // What it finished, whether it ended well or not; parts meshed with an earlier one
    // leave no file.
    let mut finished = BTreeSet::new();
    for k in 0..total {
        let Ok(text) = std::fs::read_to_string(dir.0.join(part_file(k))) else {
            continue;
        };
        let outcome: PartOutcome = ron::from_str(&text).map_err(|e| e.to_string())?;
        outcomes.parts.push((
            k,
            outcome.map(|(mesh, warnings)| GeneratedMesh { mesh, warnings }),
        ));
        finished.insert(k);
    }
    if let Some(text) = text {
        let result: WorkerResult = ron::from_str(&text).map_err(|e| e.to_string())?;
        return result.map(|()| None);
    }
    if cancelled {
        return Ok(None);
    }
    let status = status.map_or_else(String::new, |s| format!(" ({s})"));
    // The part it was meshing when it stopped, unless it stopped before or between the
    // parts.
    let crashed = read_current().map(|(place, _)| place);
    let Some(crashed) = crashed.filter(|c| !finished.contains(c)) else {
        return Err(format!("The mesher stopped unexpectedly{status}"));
    };
    let gmsh_log = std::fs::read_to_string(dir.0.join(GMSH_LOG)).unwrap_or_default();
    let place = (gmsh_log.lines().rev())
        .find_map(plx_mesher::meshing_step)
        .map_or_else(String::new, |step| format!(" while {step}"));
    // Gmsh's own words up to the crash, for a report of it.
    let kept = std::env::temp_dir().join(CRASH_LOG);
    let kept = match std::fs::write(&kept, &gmsh_log) {
        Ok(()) => format!(" (Gmsh's log: {})", kept.display()),
        Err(_) => String::new(),
    };
    Ok(Some((
        crashed,
        format!("Gmsh crashed{place}{status}{kept}"),
    )))
}

/// How far a meshing job is: the parts it works on now, the parts done before them and all
/// it meshes.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MeshProgress {
    pub done: usize,
    pub total: usize,
    /// The parts meshed now, one per worker process.
    pub parts: Vec<String>,
}

impl MeshProgress {
    /// The parts meshed now in a few words: the first, and how many more.
    pub fn now(&self) -> String {
        match &self.parts[..] {
            [] => String::new(),
            [part] => part.clone(),
            [part, more @ ..] => format!("{part} and {} more", more.len()),
        }
    }
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
    /// How far it is; `None` until the first part starts.
    progress: Arc<std::sync::Mutex<Option<MeshProgress>>>,
}

impl MeshingJob {
    /// Meshes the named parts, or all parts with `None`, with the cut faces of the cyclic
    /// symmetries `matching` meshed alike ([`plx_model::FeModel::matching_faces`]).
    pub fn start(
        geometry: Geometry,
        matching: Vec<PeriodicFaces>,
        parts: Option<Vec<String>>,
        ctx: &egui::Context,
    ) -> Self {
        let (sender, receiver) = std::sync::mpsc::channel();
        let meshed = geometry.clone();
        let ctx = ctx.clone();
        let cancel = Arc::new(AtomicBool::new(false));
        let cancelled = Arc::clone(&cancel);
        let progress = Arc::new(std::sync::Mutex::new(None));
        let told = Arc::clone(&progress);
        std::thread::spawn(move || {
            let repaint = ctx.clone();
            let tell = move |now: MeshProgress| {
                *told.lock().unwrap_or_else(|e| e.into_inner()) = Some(now);
                repaint.request_repaint();
            };
            let result = match WORKER.get() {
                Some(program) => {
                    mesh_in_worker(program, geometry, matching, parts, &cancelled, &tell)
                }
                None => mesh_parts(&geometry, &matching, parts),
            };
            let _ = sender.send(result);
            ctx.request_repaint();
        });
        Self {
            started: Instant::now(),
            geometry: meshed,
            receiver,
            cancel,
            progress,
        }
    }

    /// Stops the meshing; the parts finished before are kept, see [`PartMeshes::cancelled`].
    pub fn cancel(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }

    /// Whether [`Self::cancel`] was called.
    pub fn cancelling(&self) -> bool {
        self.cancel.load(Ordering::Relaxed)
    }

    /// How far it is: the part meshed now, with the parts done and all; `None` before the
    /// first part starts.
    pub fn progress(&self) -> Option<MeshProgress> {
        self.progress
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
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

    /// The program the tests of the worker processes start: built with the tests, next to
    /// the folder of the test programs.
    fn program() -> Option<PathBuf> {
        let tests = std::env::current_exe().ok()?;
        let name = format!("nodalis{}", std::env::consts::EXE_SUFFIX);
        let program = tests.parent()?.parent()?.join(name);
        program.is_file().then_some(program)
    }

    /// Several worker processes hand back the meshes one process makes, in the order of
    /// the parts, and tell how far they are.
    #[test]
    fn worker_processes_mesh_the_parts_as_one_does() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/zwei_bloecke.step");
        let loaded = match crate::model::load(&path, plx_model::UnitSystem::Unitless) {
            Ok(loaded) => loaded,
            Err(error) if std::env::var_os("NODALIS_REQUIRE_GMSH").is_none() => {
                eprintln!("Gmsh not available, test skipped: {error}");
                return;
            }
            Err(error) => panic!("Gmsh is required: {error}"),
        };
        let Some(program) = program() else {
            eprintln!("The program is not built, test skipped");
            return;
        };
        let mut geometry = loaded.model.geometry.unwrap();
        geometry.meshing.max_size = 5.0;
        let parts = parts_to_mesh(&geometry, None).unwrap();
        assert_eq!(part_groups(&geometry, &parts), [[0], [1]]);
        let alone = mesh_parts(&geometry, &[], None).unwrap();
        let told = std::sync::Mutex::new(Vec::new());
        let cancel = AtomicBool::new(false);
        let tell = |progress: MeshProgress| told.lock().unwrap().push(progress);
        let shared =
            mesh_in_worker(&program, geometry.clone(), Vec::new(), None, &cancel, &tell).unwrap();
        assert!(shared.failed.is_empty() && !shared.cancelled);
        assert_eq!(shared.meshes.len(), 2);
        for (one, other) in alone.meshes.iter().zip(&shared.meshes) {
            assert_eq!(one.mesh.parts[0].name, other.mesh.parts[0].name);
            assert_eq!(one.mesh.node_ids(), other.mesh.node_ids());
            assert_eq!(one.mesh.coords(), other.mesh.coords());
        }
        let told = told.into_inner().unwrap();
        let last = told.last().unwrap();
        assert_eq!((last.done, last.total), (2, 2));
        assert!(last.parts.is_empty());
        assert!(told.iter().all(|p| p.done + p.parts.len() <= p.total));
        // A cancelled job starts nothing and says so.
        cancel.store(true, Ordering::Relaxed);
        let none = mesh_in_worker(&program, geometry, Vec::new(), None, &cancel, &|_| {}).unwrap();
        assert!(none.cancelled && none.meshes.is_empty());
    }

    #[test]
    fn progress_names_the_first_part_and_counts_the_others() {
        let mut progress = MeshProgress::default();
        assert_eq!(progress.now(), "");
        progress.parts = vec!["A".into()];
        assert_eq!(progress.now(), "A");
        progress.parts = vec!["A".into(), "B".into(), "C".into()];
        assert_eq!(progress.now(), "A and 2 more");
    }

    #[test]
    fn suppressed_parts_are_not_meshed_nor_picked() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/zwei_bloecke.step");
        let loaded = match crate::model::load(&path, plx_model::UnitSystem::Unitless) {
            Ok(loaded) => loaded,
            Err(error) if std::env::var_os("NODALIS_REQUIRE_GMSH").is_none() => {
                eprintln!("Gmsh not available, test skipped: {error}");
                return;
            }
            Err(error) => panic!("Gmsh is required: {error}"),
        };
        let mut geometry = loaded.model.geometry.unwrap();
        let mut view = loaded.geometry_view.unwrap();
        geometry.meshing.max_size = 5.0;
        geometry.set_suppressed(&["SOLID-1".into()], true);
        let meshed = mesh_parts(&geometry, &[], None).unwrap();
        let names: Vec<&str> = (meshed.meshes.iter())
            .flat_map(|m| m.mesh.parts.iter().map(|p| p.name.as_str()))
            .collect();
        assert_eq!(names, ["SOLID-2"]);
        let asked = mesh_parts(&geometry, &[], Some(vec!["SOLID-1".into()])).unwrap();
        assert!(asked.meshes.is_empty());
        // A part that cannot be meshed is told by name, and the others are meshed.
        geometry.set_suppressed(&["SOLID-1".into()], false);
        geometry.mesh_items.push(plx_model::MeshSetupItem {
            id: plx_model::Id::new(),
            name: "Meshing_Parameters-1".into(),
            kind: plx_model::MeshSetupKind::MeshingParameters {
                parts: geometry.part_id("SOLID-1").into_iter().collect(),
                parameters: plx_model::MeshingParameters {
                    max_size: 0.0,
                    ..geometry.meshing
                },
            },
        });
        let partly = mesh_parts(&geometry, &[], None).unwrap();
        let names: Vec<&str> = (partly.meshes.iter())
            .flat_map(|m| m.mesh.parts.iter().map(|p| p.name.as_str()))
            .collect();
        assert_eq!(names, ["SOLID-2"]);
        assert_eq!(partly.failed.len(), 1);
        assert!(
            partly.failed[0].starts_with("SOLID-1: "),
            "{:?}",
            partly.failed
        );
        geometry.mesh_items.pop();
        geometry.set_suppressed(&["SOLID-1".into()], true);
        // In the view it is drawn as its edges and a click goes through it.
        assert!(view.mark_suppressed(&geometry));
        assert!(!view.mark_suppressed(&geometry));
        let first = view.parts.iter().position(|p| p.name == "SOLID-1").unwrap();
        assert!(view.parts[first].suppressed);
        let drawn = view.part_scene_mesh(first).unwrap();
        assert!(drawn.triangles.is_empty() && !drawn.feature_edges.is_empty());
        let (min, max) = view.parts[first].bounds.unwrap();
        let centre = 0.5 * (min + max);
        let above = Vec3::new(centre.x, centre.y, max.z + 100.0);
        assert!(
            view.pick(above, Vec3::NEG_Z)
                .is_none_or(|hit| hit.part != first)
        );
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
        let MeshSetupKind::LocalMeshSize {
            faces, edges, size, ..
        } = item.kind
        else {
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
        editor.picker.select_by = SelectBy::Geometry;
        editor.box_select(&view, &area(false), Operation::Replace);
        assert_eq!(
            faces(&editor),
            (1, 0),
            "the end face at x = -50, its edges with it"
        );
        editor.box_select(&view, &area(true), Operation::Replace);
        assert_eq!(faces(&editor), (6, 0), "all but the end face at x = 50");
        // The edge angle mode boxes edges: those of the end face.
        editor.picker.select_by = SelectBy::GeometryEdgeAngle;
        editor.box_select(&view, &area(false), Operation::Replace);
        assert_eq!(faces(&editor), (0, 4));
        // The part mode the whole part, which only a crossing box takes.
        editor.picker.select_by = SelectBy::GeometryPart;
        editor.box_select(&view, &area(false), Operation::Replace);
        assert!(editor.finish().is_err(), "nothing selected");
        editor.box_select(&view, &area(true), Operation::Replace);
        assert_eq!(faces(&editor), (7, 0));
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
            (&[geometry.part_id("SOLID-1").unwrap()][..], 0)
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
        assert_eq!(extrude.parts, vec![geometry.part_id("SOLID-1").unwrap()]);
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
        assert_eq!(
            item.kind.parts().unwrap(),
            [geometry.part_id("SOLID-1").unwrap()]
        );
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
            MeshSetupKind::MeshingParameters { parts, .. }
                if parts == &[geometry.part_id("SOLID-1").unwrap()]
        ));
        assert_eq!(
            item_highlight(&view, &item.kind, &geometry.ids).parts.len(),
            1
        );
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
