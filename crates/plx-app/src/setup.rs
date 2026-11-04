//! Creating and editing the FE model: the item dialogs for materials, sections, steps,
//! boundary conditions, loads and field outputs, and nodalis's hot spot definitions.
//!
//! Regions are picked in the 3D view while a dialog is open. The user does not
//! need node or element sets for this; the input file writer derives them. Sets can still be
//! defined in the Mesh branch of the tree, by a selection like any region.

use std::collections::{BTreeSet, HashSet};

use egui::Ui;
use plx_mesh::{CadEntity, ElementId, FeMesh, NodeId};
use plx_model::{
    AXIS_LABELS, Amplitude, BeamOrientation, BeamProfile, BeamSection, Behavior, BoundaryCondition,
    BoundaryKind, BuckleStep, ComplexFrequencyStep, Constraint, ContactPair, CrossSection,
    CyclicSymmetry, DefinedField, DefinedFieldKind, Displacement, Distribution, DynamicProcedure,
    DynamicStep, Elastic, FeModel, Field, FieldOutput, Formulation, FrequencyStep, GLOBAL,
    Hardening, HeatTransferStep, HistoryKind, HistoryOutput, Incrementation, InitialCondition,
    InitialConditionKind, ItemRef, LinearSolverChoice, Load, LoadKind, Material, MeshSet,
    ModalDamping, ModalDynamicsStep, ModeDamping, ModelSpace, MovingLoad, NodeTie, OutputDimension,
    OutputKind, PlasticPoint, Quantity, Region, RemoteBehavior, Section, SectionKind,
    SensitivityStep, SetKind, StaticStep, SteadyStateDynamicsStep, Step, StepKind,
    SurfaceInteraction, SymmetryError, Tie, UnitSystem, ViscoStep, Wheel, next_name,
    symmetry_plane,
};

use crate::amplitude_dialog::{self, AmplitudeView, amplitude_row, amplitude_row_with};
use crate::connections::{self, RegionView};
use crate::constraint_dialog::ConstraintDraft;
use crate::contacts::{self, MasterSlave};
use crate::distribution_dialog::{self, DistributionView, distribution_row};
use crate::expose;
use crate::icons::{self, Icon};
use crate::model::{Highlight, Hit, Model};
use crate::motion_preview::MotionPreview;
use crate::numeric;
use crate::selection::{History, Items, Operation, PartPicks, Picker, PickerAction, Target};
use crate::tree::TreeItem;
use crate::viewport::{BoxSelect, Preview};

/// A contact region or a contact pair, by index, whose contact values an output gives.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactItem {
    Region(usize),
    Pair(usize),
}

/// Kinds of items the tree can create.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NewItem {
    Material,
    Section,
    /// A beam profile that beam sections pick.
    CrossSection,
    InitialCondition,
    Step,
    BoundaryCondition(usize),
    Load(usize),
    /// A field output of a step, written into the `.frd` file.
    FieldOutput(usize),
    /// A history output of a step, printed into the `.dat` file.
    HistoryOutput(usize),
    /// A history output of a step printing the contact values of a contact region or pair,
    /// created in the menu of the contact.
    ContactHistoryOutput(usize, ContactItem),
    /// A contact field output of a step, created in the menu of a contact; a step that has
    /// one already opens it.
    ContactFieldOutput(usize),
    /// A defined temperature of a step.
    DefinedField(usize),
    /// A spring, support or rigid body, chosen in the dialog.
    Constraint,
    SurfaceInteraction,
    /// Two surfaces in contact by the behavior the user wants.
    ContactRegion,
    /// A search for contact regions, kept to run it again.
    ContactDetection,
    /// A bolt preloaded across a cut through its shank.
    Bolt,
    ContactPair,
    /// Surfaces glued to each other; listed with the contact pairs.
    Tie,
    /// Nodes tied to each other, the ends of beams; listed with the contact pairs.
    NodeTie,
    /// A time curve for boundary conditions and loads.
    Amplitude,
    /// A factor over the model for pressures and temperatures.
    Distribution,
    /// A field output derived from results, created in the Results tree.
    ResultFieldOutput,
    /// A history output derived from results, created in the Results tree.
    ResultHistoryOutput,
    /// A hot spot definition of results, created in the Results tree.
    ResultHotSpot,
    /// An item of the geometry's mesh setup, created in the Geometry tree.
    MeshSetupItem,
    /// A reference point, coordinate system or result path, edited in its own dialog.
    Feature(crate::features::FeatureKind),
    /// A node set, element set or surface of the mesh, defined by a selection.
    MeshSet(SetKind),
}

/// How a region is given.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Source {
    /// Picked in the 3D view.
    Selection,
    Parts,
    NodeSet,
    ElementSet,
    Surface,
    /// A reference point, which drives a rigid body.
    ReferencePoint,
    /// The point of a remote displacement boundary condition, offered among the reference
    /// points.
    RemotePoint,
}

impl Source {
    fn label(self) -> &'static str {
        match self {
            Source::Selection => "Selection in the 3D view",
            Source::Parts => "Parts",
            Source::NodeSet => "Node Set",
            Source::ElementSet => "Element Set",
            Source::Surface => "Surface",
            Source::ReferencePoint | Source::RemotePoint => "Reference Point",
        }
    }
}

/// A region while it is edited in a dialog.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct RegionDraft {
    sources: &'static [Source],
    pub(crate) source: Source,
    /// Whether picks select nodes or element faces.
    pub(crate) target: Target,
    nodes: History<NodeId>,
    faces: History<(ElementId, u8)>,
    /// Element edges of 3D parts, by their end nodes.
    edges: History<[NodeId; 2]>,
    /// Picks of CAD entities on a mesh generated from geometry. Mixed with picks of nodes or
    /// faces they turn into those, so only one of the histories is in use at a time.
    geometry: History<CadEntity>,
    parts: PartPicks,
    set: String,
}

pub(crate) const NODE_SOURCES: &[Source] = &[Source::Selection, Source::NodeSet, Source::Surface];
pub(crate) const FACE_SOURCES: &[Source] = &[Source::Selection, Source::Surface];
/// Nodes, or the reference point of a rigid body: for boundary conditions and point loads.
pub(crate) const SUPPORT_SOURCES: &[Source] = &[
    Source::Selection,
    Source::NodeSet,
    Source::Surface,
    Source::ReferencePoint,
];
/// Nodes, or the point of a rigid body or a remote displacement: for nodal history outputs,
/// which print the forces and moments of a point too.
pub(crate) const HISTORY_NODE_SOURCES: &[Source] = &[
    Source::Selection,
    Source::NodeSet,
    Source::Surface,
    Source::ReferencePoint,
    Source::RemotePoint,
];
/// Solid elements: whole parts, element sets or the elements of picked faces.
pub(crate) const SOLID_SOURCES: &[Source] = &[Source::Parts, Source::ElementSet, Source::Selection];
pub(crate) const ELEMENT_SOURCES: &[Source] = &[Source::Parts, Source::ElementSet];
/// Nodes or elements of a user-defined set: picked, or of whole parts.
pub(crate) const SET_SOURCES: &[Source] = &[Source::Selection, Source::Parts];
/// Element faces of a user-defined surface.
pub(crate) const SURFACE_SOURCES: &[Source] = &[Source::Selection];
/// Nodes, also those of whole parts, e.g. for the initial temperature of the model.
pub(crate) const NODE_PART_SOURCES: &[Source] = &[
    Source::Selection,
    Source::Parts,
    Source::NodeSet,
    Source::Surface,
];

impl RegionDraft {
    pub(crate) fn new(sources: &'static [Source], target: Target) -> Self {
        Self {
            sources,
            source: sources[0],
            target,
            nodes: History::default(),
            faces: History::default(),
            edges: History::default(),
            geometry: History::default(),
            parts: PartPicks::default(),
            set: String::new(),
        }
    }

    pub(crate) fn from_region(
        region: &Region,
        sources: &'static [Source],
        target: Target,
        mesh: &FeMesh,
    ) -> Self {
        let mut draft = Self::new(sources, target);
        // Picked nodes, faces or CAD entities come from the selection, also where the
        // dialog lists another source first (whole parts for element regions).
        if matches!(
            region,
            Region::Nodes(_) | Region::Faces(_) | Region::Edges(_) | Region::Geometry(_)
        ) && sources.contains(&Source::Selection)
        {
            draft.source = Source::Selection;
        }
        match region {
            Region::Parts(parts) => {
                draft.source = Source::Parts;
                draft.parts = PartPicks::from_names(parts.iter().cloned());
            }
            Region::Nodes(nodes) => draft.nodes = History::from_items(nodes.iter().copied()),
            Region::Faces(faces)
                if matches!(target, Target::Faces | Target::Surfaces | Target::Edges) =>
            {
                draft.faces = History::from_items(faces.iter().copied());
            }
            Region::Edges(edges) if target == Target::Lines => {
                draft.edges = History::from_items(edges.iter().copied());
            }
            Region::Faces(_) | Region::Edges(_) => {
                draft.nodes = History::from_items(region.nodes(mesh));
            }
            Region::Geometry(entities) => {
                draft.geometry = History::from_items(entities.iter().copied());
            }
            Region::NodeSet(set)
            | Region::ElementSet(set)
            | Region::Surface(set)
            | Region::ReferencePoint(set)
            | Region::RemotePoint(set) => {
                draft.source = match region {
                    Region::NodeSet(_) => Source::NodeSet,
                    Region::ElementSet(_) => Source::ElementSet,
                    Region::ReferencePoint(_) => Source::ReferencePoint,
                    Region::RemotePoint(_) => Source::RemotePoint,
                    _ => Source::Surface,
                };
                draft.set = set.clone();
            }
        }
        draft
    }

    /// This region as a draft for another kind of item, which picks with `sources` and
    /// `target`: kept where the new kind takes it, else empty. The flag says that picks were
    /// dropped.
    pub(crate) fn converted(
        &self,
        sources: &'static [Source],
        target: Target,
        mesh: &FeMesh,
    ) -> (Self, bool) {
        let region = self.region();
        let selection = sources.contains(&Source::Selection);
        let kept = !self.is_empty()
            && match &region {
                Region::Nodes(_) => selection && matches!(target, Target::Nodes | Target::Point),
                Region::Faces(_) => {
                    selection && matches!(target, Target::Faces | Target::Surfaces | Target::Edges)
                }
                Region::Edges(_) => selection && target == Target::Lines,
                Region::Geometry(_) => selection,
                Region::Parts(_) => sources.contains(&Source::Parts),
                Region::NodeSet(_) => sources.contains(&Source::NodeSet),
                Region::ElementSet(_) => sources.contains(&Source::ElementSet),
                Region::Surface(_) => sources.contains(&Source::Surface),
                Region::ReferencePoint(_) => sources.contains(&Source::ReferencePoint),
                Region::RemotePoint(_) => sources.contains(&Source::RemotePoint),
            };
        if kept {
            (Self::from_region(&region, sources, target, mesh), false)
        } else {
            (Self::new(sources, target), !self.is_empty())
        }
    }

    pub(crate) fn region(&self) -> Region {
        match self.source {
            Source::Selection if self.geometry.can_undo() => {
                Region::Geometry(self.geometry.items().into_iter().collect())
            }
            Source::Selection => match self.target {
                Target::Nodes | Target::Point => {
                    Region::Nodes(self.nodes.items().into_iter().collect())
                }
                Target::Faces | Target::Surfaces | Target::Edges => {
                    Region::Faces(self.faces.items().into_iter().collect())
                }
                Target::Lines => Region::Edges(self.edges.items().into_iter().collect()),
            },
            Source::Parts => Region::Parts(self.parts.names().into_iter().collect()),
            Source::NodeSet => Region::NodeSet(self.set.clone()),
            Source::ElementSet => Region::ElementSet(self.set.clone()),
            Source::Surface => Region::Surface(self.set.clone()),
            Source::ReferencePoint => Region::ReferencePoint(self.set.clone()),
            Source::RemotePoint => Region::RemotePoint(self.set.clone()),
        }
    }

    pub(crate) fn is_empty(&self) -> bool {
        match self.source {
            Source::Selection if self.geometry.can_undo() => self.geometry.items().is_empty(),
            Source::Selection => match self.target {
                Target::Nodes | Target::Point => self.nodes.items().is_empty(),
                Target::Faces | Target::Surfaces | Target::Edges => self.faces.items().is_empty(),
                Target::Lines => self.edges.items().is_empty(),
            },
            Source::Parts => self.parts.is_empty(),
            _ => self.set.is_empty(),
        }
    }

    pub(crate) fn count(&self) -> usize {
        match self.target {
            Target::Nodes | Target::Point => self.nodes.items().len(),
            Target::Faces | Target::Surfaces | Target::Edges => self.faces.items().len(),
            Target::Lines => self.edges.items().len(),
        }
    }

    pub(crate) fn click(
        &mut self,
        model: &Model,
        picker: &Picker,
        pick: Option<(&Hit, f32)>,
        operation: Operation,
    ) {
        match self.source {
            Source::Parts => self.parts.click(model, pick.map(|(hit, _)| hit), operation),
            Source::Selection => match pick {
                Some((hit, precision)) => {
                    let items = picker.pick(model, hit, self.target, precision);
                    self.take(&model.mesh, items, operation);
                }
                // Nodalis clears the selection on a plain click into empty space.
                None if operation == Operation::Replace => self.clear(),
                None => {}
            },
            _ => {}
        }
    }

    pub(crate) fn clear(&mut self) {
        self.nodes.clear();
        self.faces.clear();
        self.edges.clear();
        self.geometry.clear();
    }

    /// Adds picked items to the selection. CAD entities stay CAD entities unless nodes or
    /// faces are picked too; then they become the nodes or faces they stand for.
    pub(crate) fn take(&mut self, mesh: &FeMesh, items: Items, operation: Operation) {
        let replace = operation == Operation::Replace;
        match items {
            Items::Geometry(entities) if replace || !self.ids_can_undo() => {
                self.nodes.clear();
                self.faces.clear();
                self.edges.clear();
                self.geometry.push(operation, entities);
            }
            Items::Geometry(entities) => {
                let items = Items::Geometry(entities).resolved(mesh, self.target);
                self.take(mesh, items, operation);
            }
            items => {
                if replace {
                    self.geometry.clear();
                } else {
                    self.geometry_to_ids(mesh);
                }
                match items {
                    Items::Nodes(nodes) => self.nodes.push(operation, nodes),
                    Items::Faces(faces) => self.faces.push(operation, faces),
                    Items::Edges(edges) => self.edges.push(operation, edges),
                    Items::Geometry(_) => unreachable!("handled above"),
                }
            }
        }
    }

    /// Turns picked CAD entities into the nodes or faces they stand for.
    fn geometry_to_ids(&mut self, mesh: &FeMesh) {
        if !self.geometry.can_undo() {
            return;
        }
        let entities: Vec<CadEntity> = self.geometry.items().into_iter().collect();
        self.geometry.clear();
        match self.target {
            Target::Nodes | Target::Point => {
                self.nodes = History::from_items(mesh.cad_nodes(&entities))
            }
            Target::Faces | Target::Surfaces | Target::Edges => {
                self.faces = History::from_items(mesh.cad_faces(&entities));
            }
            Target::Lines => {
                self.edges = History::from_items(crate::selection::cad_segments(mesh, &entities));
            }
        }
    }

    fn ids_can_undo(&self) -> bool {
        match self.target {
            Target::Nodes | Target::Point => self.nodes.can_undo(),
            Target::Faces | Target::Surfaces | Target::Edges => self.faces.can_undo(),
            Target::Lines => self.edges.can_undo(),
        }
    }

    pub(crate) fn can_undo(&self) -> bool {
        match self.source {
            Source::Parts => self.parts.can_undo(),
            _ => self.geometry.can_undo() || self.ids_can_undo(),
        }
    }

    /// Whether clicks in the 3D view pick for this region, with the selection window open.
    pub(crate) fn picks(&self) -> bool {
        matches!(self.source, Source::Selection | Source::Parts)
    }

    /// The selection window next to the dialog `anchor` while this region is picked;
    /// regions of parts only pick whole parts.
    pub(crate) fn picker_window(
        &mut self,
        ctx: &egui::Context,
        anchor: egui::Rect,
        picker: &mut Picker,
        model: &Model,
    ) {
        let can_undo = self.can_undo();
        let action = match self.source {
            Source::Selection => picker.window(ctx, anchor, self.target, can_undo),
            Source::Parts => picker.parts_window(ctx, anchor, can_undo),
            _ => None,
        };
        if let Some(action) = action {
            self.action(model, action);
        }
    }

    /// Applies a button of the selection window.
    pub(crate) fn action(&mut self, model: &Model, action: PickerAction) {
        if self.source == Source::Parts {
            return self.parts.action(model, action);
        }
        match action {
            PickerAction::Undo if self.geometry.can_undo() => return self.geometry.undo(),
            PickerAction::Undo | PickerAction::Clear => {}
            // The buttons work on nodes and faces.
            _ => self.geometry_to_ids(&model.mesh),
        }
        match (action, self.target) {
            (PickerAction::Undo, Target::Nodes | Target::Point) => self.nodes.undo(),
            (PickerAction::Undo, Target::Faces | Target::Surfaces | Target::Edges) => {
                self.faces.undo()
            }
            (PickerAction::Undo, Target::Lines) => self.edges.undo(),
            (PickerAction::All, Target::Lines) => {
                let all = crate::selection::visible_segments(model);
                self.edges.push(Operation::Replace, all);
            }
            (PickerAction::Invert, Target::Lines) => {
                let selected = self.edges.items();
                let mut all = crate::selection::visible_segments(model);
                all.retain(|e| !selected.contains(e));
                self.edges.push(Operation::Replace, all);
            }
            // Element ids do not name edges.
            (PickerAction::Ids(..), Target::Lines) => {}
            (PickerAction::Clear, _) => self.clear(),
            (PickerAction::All, Target::Nodes | Target::Point) => {
                self.nodes.push(Operation::Replace, model.visible_nodes());
            }
            (PickerAction::All, Target::Faces | Target::Surfaces) => {
                self.faces.push(Operation::Replace, model.visible_faces());
            }
            (PickerAction::Invert, Target::Nodes | Target::Point) => {
                let selected = self.nodes.items();
                let mut all = model.visible_nodes();
                all.retain(|n| !selected.contains(n));
                self.nodes.push(Operation::Replace, all);
            }
            (PickerAction::All, Target::Edges) => {
                self.faces.push(Operation::Replace, visible_edges(model));
            }
            (PickerAction::Invert, Target::Edges) => {
                let selected = self.faces.items();
                let mut all = visible_edges(model);
                all.retain(|f| !selected.contains(f));
                self.faces.push(Operation::Replace, all);
            }
            (PickerAction::Ids(operation, ids), Target::Edges) => {
                // Ids of elements: their edges on the outline.
                let elements: BTreeSet<ElementId> = ids.into_iter().collect();
                let mut edges = visible_edges(model);
                edges.retain(|(element, _)| elements.contains(element));
                self.faces.push(operation, edges);
            }
            (PickerAction::Invert, Target::Faces | Target::Surfaces) => {
                let selected = self.faces.items();
                let mut all = model.visible_faces();
                all.retain(|f| !selected.contains(f));
                self.faces.push(Operation::Replace, all);
            }
            (PickerAction::Ids(operation, ids), Target::Nodes | Target::Point) => {
                let ids = ids
                    .into_iter()
                    .filter(|&id| model.mesh.node_index(id).is_some())
                    .collect();
                self.nodes.push(operation, ids);
            }
            (PickerAction::Ids(operation, ids), Target::Faces | Target::Surfaces) => {
                // Ids of elements: their faces on the surface.
                let elements: BTreeSet<ElementId> = ids.into_iter().collect();
                let faces = model
                    .visible_faces()
                    .into_iter()
                    .filter(|(element, _)| elements.contains(element))
                    .collect();
                self.faces.push(operation, faces);
            }
        }
    }

    /// The rows of the only region of a dialog, which clicks in the 3D view always fill.
    pub(crate) fn ui(&mut self, ui: &mut Ui, model: &Model) {
        self.ui_labeled(ui, model, "Region", "region", true);
    }

    /// The region's rows under its own label; `id` keeps its widgets apart from those of
    /// another region in the same dialog. `active` tells whether clicks in the 3D view fill
    /// this region; returns whether the user wants them to, by its "..." button or by
    /// changing where the region comes from.
    pub(crate) fn ui_labeled(
        &mut self,
        ui: &mut Ui,
        model: &Model,
        label: &str,
        id: &str,
        active: bool,
    ) -> bool {
        // Top-aligned: the grid centers a cell, which would put the label beside the
        // rows below the source combo box.
        ui.with_layout(egui::Layout::top_down(egui::Align::Min), |ui| {
            ui.label(label)
        });
        let wanted = ui.push_id(id, |ui| self.ui_body(ui, model, active)).inner;
        ui.end_row();
        wanted
    }

    fn ui_body(&mut self, ui: &mut Ui, model: &Model, active: bool) -> bool {
        let mut wanted = false;
        ui.vertical(|ui| {
            let source = self.source;
            egui::ComboBox::from_id_salt("region source")
                .selected_text(self.source.label())
                .width(200.0)
                .show_ui(ui, |ui| {
                    // Remote points are listed with the reference points.
                    for &source in self.sources.iter().filter(|&&s| s != Source::RemotePoint) {
                        let point =
                            source == Source::ReferencePoint && self.source == Source::RemotePoint;
                        if ui
                            .selectable_label(self.source == source || point, source.label())
                            .clicked()
                        {
                            self.source = source;
                        }
                    }
                });
            wanted |= self.source != source;
            match self.source {
                Source::Selection => {
                    ui.horizontal(|ui| {
                        wanted |= pick_button(ui, active);
                        let count = self.count();
                        let geometry = self.geometry.items();
                        let what = match self.target {
                            Target::Nodes | Target::Point => "Nodes",
                            Target::Faces | Target::Surfaces => "Element faces",
                            Target::Edges | Target::Lines => "Element edges",
                        };
                        if !geometry.is_empty() {
                            let entities: Vec<CadEntity> = geometry.into_iter().collect();
                            ui.label(plx_model::describe_entities(&entities));
                        } else if count == 0 {
                            ui.label("Empty");
                        } else {
                            ui.label(format!("{count} {what}"));
                        }
                        if ui.button("Clear Selection").clicked() {
                            self.clear();
                        }
                    });
                    ui.weak("Choose in the \"Selection\" window what a click selects.");
                }
                Source::Parts => wanted |= self.parts.ui(ui, active),
                Source::ReferencePoint | Source::RemotePoint => self.point_ui(ui, model),
                Source::NodeSet | Source::ElementSet | Source::Surface => {
                    let names: Vec<&String> = match self.source {
                        Source::NodeSet => model.mesh.node_sets.keys().collect(),
                        Source::ElementSet => model.mesh.element_sets.keys().collect(),
                        _ => model.mesh.surfaces.keys().collect(),
                    };
                    if names.is_empty() {
                        ui.weak("The mesh contains no such sets.");
                    }
                    egui::ComboBox::from_id_salt("region set")
                        .selected_text(self.set.as_str())
                        .width(200.0)
                        .show_ui(ui, |ui| {
                            for name in names {
                                ui.selectable_value(&mut self.set, name.clone(), name);
                            }
                        });
                }
            }
        });
        wanted
    }

    /// The reference points, and where the region takes them the points of the remote
    /// displacements, by name.
    fn point_ui(&mut self, ui: &mut Ui, model: &Model) {
        let points: Vec<&str> = (model.fe.reference_points.iter())
            .map(|p| p.name.as_str())
            .collect();
        let mut remotes: Vec<&str> = Vec::new();
        if self.sources.contains(&Source::RemotePoint) {
            for (s, step) in model.fe.steps.iter().enumerate() {
                for (i, bc) in step.boundary_conditions.iter().enumerate() {
                    if matches!(
                        model.fe.formulation(&model.mesh, s, i),
                        Some(Formulation::Point(_))
                    ) && !remotes.contains(&bc.name.as_str())
                    {
                        remotes.push(&bc.name);
                    }
                }
            }
        }
        if points.is_empty() && remotes.is_empty() {
            ui.weak(if self.sources.contains(&Source::RemotePoint) {
                "The model has no reference points (Features) and no displacements on a point."
            } else {
                "The model has no reference points (Features)."
            });
        }
        let (mut source, mut set) = (self.source, self.set.clone());
        egui::ComboBox::from_id_salt("region set")
            .selected_text(self.set.as_str())
            .width(200.0)
            .show_ui(ui, |ui| {
                for &name in &points {
                    let chosen = source == Source::ReferencePoint && set == name;
                    if ui.selectable_label(chosen, name).clicked() {
                        (source, set) = (Source::ReferencePoint, name.to_string());
                    }
                }
                if !remotes.is_empty() {
                    if !points.is_empty() {
                        ui.separator();
                    }
                    ui.weak("Displacements on a point");
                }
                for name in remotes {
                    let chosen = source == Source::RemotePoint && set == name;
                    if ui.selectable_label(chosen, name).clicked() {
                        (source, set) = (Source::RemotePoint, name.to_string());
                    }
                }
            });
        (self.source, self.set) = (source, set);
    }

    pub(crate) fn highlight(&self, model: &Model) -> Highlight {
        region_highlight(model, &self.region())
    }

    /// A selection box dragged in the 3D view.
    pub(crate) fn box_select(
        &mut self,
        model: &Model,
        picker: &Picker,
        area: &BoxSelect,
        operation: Operation,
    ) {
        match self.source {
            Source::Selection => {
                let items = picker.pick_box(model, area, self.target);
                self.take(&model.mesh, items, operation);
            }
            Source::Parts => self.parts.box_select(model, area, operation),
            _ => {}
        }
    }

    /// What a click at the hit would select, for the hover preview.
    pub(crate) fn preview(
        &self,
        model: &Model,
        picker: &Picker,
        hit: &Hit,
        precision: f32,
    ) -> Preview {
        match self.source {
            Source::Selection => {
                crate::selection::preview(model, &picker.pick(model, hit, self.target, precision))
            }
            Source::Parts => PartPicks::preview(model, hit),
            _ => Preview::default(),
        }
    }
}

/// The "..." button of a field filled by clicks in the 3D view, shown pressed while it
/// is the field they fill. Returns whether it was clicked.
pub(crate) fn pick_button(ui: &mut Ui, active: bool) -> bool {
    let hint = if active {
        "Clicks in the 3D view select into this field."
    } else {
        "Select into this field in the 3D view"
    };
    let button = egui::Button::new("...").selected(active);
    ui.add(button).on_hover_text(hint).clicked()
}

/// Edges of the outline of the visible parts, the faces of a 2D model.
fn visible_edges(model: &Model) -> BTreeSet<(ElementId, u8)> {
    (model.outline_edges().into_iter())
        .filter(|(part, _, _)| model.parts[*part].visible)
        .map(|(_, face, _)| face)
        .collect()
}

/// Contacts and ties are on element faces or edges of shells, in 2D models on the edges of
/// their elements.
pub(crate) fn contact_target(fe: &FeModel) -> Target {
    if fe.properties.space.is_2d() {
        Target::Edges
    } else {
        Target::Surfaces
    }
}

/// Pressure and surface traction act on element faces, in 2D models on their edges.
pub(crate) fn face_target(fe: &FeModel) -> Target {
    if fe.properties.space.is_2d() {
        Target::Edges
    } else {
        Target::Faces
    }
}

/// The components of a force; 2D models have none along z.
fn force_rows(ui: &mut Ui, force: &mut [f64; 3], two_d: bool, units: UnitSystem) {
    let count = if two_d { 2 } else { 3 };
    let labels = ["F1", "F2", "F3"].into_iter().zip(Field::VECTOR);
    for (value, (label, field)) in force.iter_mut().zip(labels).take(count) {
        ui.label(label);
        expose::field(ui, field, |ui| {
            ui.add(numeric::quantity(value, units, Quantity::Force).speed(1.0))
        });
        ui.end_row();
    }
}

/// Rows of a moving load: the path along an axis of a coordinate system, the contact patch
/// and the wheels relative to the vehicle's reference point.
fn moving_rows(ui: &mut Ui, moving: &mut MovingLoad, fe: &FeModel, two_d: bool, units: UnitSystem) {
    ui.label("Coordinate system");
    egui::ComboBox::from_id_salt("moving load system")
        .selected_text(moving.system.as_str())
        .width(200.0)
        .show_ui(ui, |ui| {
            let names = std::iter::once(GLOBAL)
                .chain(fe.coordinate_systems.iter().map(|c| c.name.as_str()));
            for name in names {
                ui.selectable_value(&mut moving.system, name.to_string(), name);
            }
        });
    ui.end_row();
    let axes = if two_d { 2 } else { 3 };
    let axis_row = |ui: &mut Ui, label: &str, axis: &mut usize, hover: &str| {
        ui.label(label);
        ui.horizontal(|ui| {
            for (index, name) in AXIS_LABELS.iter().enumerate().take(axes) {
                let button =
                    egui::Button::selectable(*axis == index, *name).frame_when_inactive(true);
                if ui.add(button).on_hover_text(hover).clicked() {
                    *axis = index;
                }
            }
        });
        ui.end_row();
    };
    axis_row(
        ui,
        "Path along",
        &mut moving.travel_axis,
        "Axis of the coordinate system the load travels along",
    );
    if !two_d {
        axis_row(
            ui,
            "Across the path",
            &mut moving.lateral_axis,
            "Axis across the path in the running surface: the patch width and the wheel \
             offsets 'Across' are measured along it. The remaining axis is the surface normal.",
        );
        let (travel, across) = (moving.travel_axis, moving.lateral_axis);
        if let Some(normal) = (0..3).find(|&k| travel != across && k != travel && k != across) {
            ui.label("");
            ui.weak(format!(
                "{} is the surface normal; the selected faces set the height.",
                AXIS_LABELS[normal]
            ));
            ui.end_row();
        }
    }
    ui.label("Start");
    ui.add(numeric::quantity(&mut moving.start, units, Quantity::Length).speed(1.0))
        .on_hover_text("Position of the reference point in the first step");
    ui.end_row();
    ui.label("End");
    ui.add(numeric::quantity(&mut moving.end, units, Quantity::Length).speed(1.0))
        .on_hover_text("Position of the reference point in the last step");
    ui.end_row();
    ui.label("Positions");
    ui.add(
        crate::numeric::count(&mut moving.positions)
            .range(1..=10_000)
            .speed(0.2),
    )
    .on_hover_text("Each position is a static step of its own");
    ui.end_row();
    let stations = moving.stations();
    if stations.len() > 1 {
        ui.label("");
        ui.weak(format!(
            "Spacing {} {}",
            numeric::format_physical(units.shown(stations[1] - stations[0], Quantity::Length)),
            units.unit(Quantity::Length)
        ));
        ui.end_row();
    }
    ui.label("Contact patch length");
    ui.add(numeric::quantity(&mut moving.patch_length, units, Quantity::Length).speed(1.0))
        .on_hover_text("Length of each wheel's contact patch along the path");
    ui.end_row();
    if !two_d {
        ui.label("Contact patch width");
        ui.add(numeric::quantity(&mut moving.patch_width, units, Quantity::Length).speed(1.0))
            .on_hover_text("Width of each wheel's contact patch across the path");
        ui.end_row();
    }
    ui.label("Wheels");
    ui.vertical(|ui| {
        let mut remove = None;
        egui::Grid::new("moving load wheels")
            .striped(true)
            .spacing([6.0, 2.0])
            .show(ui, |ui| {
                ui.label("");
                ui.strong("Along");
                if !two_d {
                    ui.strong("Across");
                }
                for label in ["F1", "F2", "F3"].iter().take(if two_d { 2 } else { 3 }) {
                    ui.strong(*label);
                }
                ui.label("");
                ui.end_row();
                for (index, wheel) in moving.wheels.iter_mut().enumerate() {
                    ui.label(format!("{}", index + 1));
                    let width = 70.0;
                    ui.add_sized(
                        [width, 18.0],
                        numeric::quantity(&mut wheel.along, units, Quantity::Length).speed(1.0),
                    );
                    if !two_d {
                        ui.add_sized(
                            [width, 18.0],
                            numeric::quantity(&mut wheel.across, units, Quantity::Length)
                                .speed(1.0),
                        );
                    }
                    for value in wheel.force.iter_mut().take(if two_d { 2 } else { 3 }) {
                        ui.add_sized(
                            [width, 18.0],
                            numeric::quantity(value, units, Quantity::Force).speed(1.0),
                        );
                    }
                    if ui.small_button("Remove").clicked() {
                        remove = Some(index);
                    }
                    ui.end_row();
                }
            });
        if let Some(index) = remove {
            moving.wheels.remove(index);
        }
        ui.horizontal(|ui| {
            if ui.button("Add wheel").clicked() {
                let wheel = moving.wheels.last().copied().unwrap_or(Wheel {
                    along: 0.0,
                    across: 0.0,
                    force: [0.0; 3],
                });
                moving.wheels.push(wheel);
            }
            if ui
                .button("Mirror")
                .on_hover_text(
                    "Adds the wheels mirrored about the reference point along the path, \
                     e.g. the other half of a symmetric axle layout",
                )
                .clicked()
            {
                let mirrored: Vec<Wheel> = (moving.wheels.iter())
                    .filter(|w| w.along != 0.0)
                    .map(|w| Wheel {
                        along: -w.along,
                        ..*w
                    })
                    .collect();
                moving.wheels.extend(mirrored);
            }
        });
    });
    ui.end_row();
    let total: [f64; 3] =
        std::array::from_fn(|k| moving.wheels.iter().map(|w| w.force[k]).sum::<f64>());
    ui.label("");
    ui.weak(format!(
        "Total force ({}) {}",
        (total.iter().take(if two_d { 2 } else { 3 }))
            .map(|f| numeric::format_physical(units.shown(*f, Quantity::Force)))
            .collect::<Vec<_>>()
            .join(", "),
        units.unit(Quantity::Force)
    ));
    ui.end_row();
    for line in [
        "Wheel offsets are measured from the reference point, forces act in global",
        "directions. Select the faces the wheels run on. A wheel off them carries no",
        "force, one partly off the covered part. One static step is written per position.",
    ] {
        ui.label("");
        ui.weak(line);
        ui.end_row();
    }
}

/// Forces of axisymmetric models act on the whole revolution, as in CalculiX.
fn revolution_hint(ui: &mut Ui, axisymmetric: bool) {
    if axisymmetric {
        ui.label("");
        ui.weak("Axisymmetric: force on the whole circumference (360°).");
        ui.end_row();
    }
}

/// Element faces of a 3D model as drawn: faces on the skin, the top of a shell on its one
/// skin face, and the edges of shells as lines.
pub fn shown_faces(
    model: &Model,
    faces: impl IntoIterator<Item = (ElementId, u8)>,
) -> (HashSet<(ElementId, u8)>, Vec<[NodeId; 2]>) {
    let mut shown = HashSet::new();
    let mut edges = BTreeSet::new();
    for (element, face) in faces {
        match model.mesh.element(element) {
            Some(e) if e.is_shell() && face >= 3 => {
                edges.insert((element, face));
            }
            Some(e) if e.is_shell() => {
                shown.insert((element, 1));
            }
            _ => {
                shown.insert((element, face));
            }
        }
    }
    (shown, crate::selection::edge_lines(model, &edges))
}

/// How a region is shown selected in the 3D view.
pub fn region_highlight(model: &Model, region: &Region) -> Highlight {
    let mut highlight = Highlight::default();
    match region {
        Region::Parts(names) => {
            highlight.parts = (model.parts.iter().enumerate())
                .filter(|(_, p)| names.contains(&p.name))
                .map(|(i, _)| i)
                .collect();
        }
        Region::Edges(_) => {
            // Split at the midside node, as the edges of 2D elements.
            for line in region.edges(&model.mesh) {
                match line[..] {
                    [a, b, m] => highlight.lines.extend([[a, m], [m, b]]),
                    [a, b] => highlight.lines.push([a, b]),
                    _ => {}
                }
            }
        }
        Region::Faces(_) | Region::Surface(_) if model.is_plane() => {
            // The faces of 2D elements are edges, drawn as lines.
            let faces = region.faces(&model.mesh).into_iter().collect();
            highlight.lines = crate::selection::edge_lines(model, &faces);
            if highlight.lines.is_empty() {
                highlight.nodes = region.nodes(&model.mesh);
            }
        }
        Region::Faces(_) | Region::Surface(_) => {
            (highlight.faces, highlight.lines) = shown_faces(model, region.faces(&model.mesh));
            if highlight.faces.is_empty() && highlight.lines.is_empty() {
                highlight.nodes = region.nodes(&model.mesh);
            }
        }
        Region::ElementSet(_) => {
            let elements: BTreeSet<ElementId> = region.elements(&model.mesh).into_iter().collect();
            highlight.faces = model
                .skin_faces()
                .filter(|(element, _)| elements.contains(element))
                .collect();
            // Line elements have no faces: they show as lines.
            highlight.lines = (model.line_segments())
                .filter(|(element, _)| elements.contains(element))
                .map(|(_, line)| line)
                .collect();
        }
        Region::Geometry(entities) => return crate::cad_selection::highlight(model, entities),
        // The point is drawn as a feature; the body it drives is the constraint's region.
        Region::ReferencePoint(_) => {}
        // The point of a remote displacement exists only in the input file; its region shows.
        Region::RemotePoint(name) => {
            if let Some((_, bc, _)) = model.fe.remote_displacement(&model.mesh, name) {
                return region_highlight(model, &bc.region);
            }
        }
        Region::Nodes(_) | Region::NodeSet(_) => {
            highlight.nodes = region.nodes(&model.mesh);
            // Faces whose corners are all selected show as faces, as for
            // picked surfaces.
            let selected: std::collections::HashSet<NodeId> =
                highlight.nodes.iter().copied().collect();
            highlight.faces = model
                .skin_faces_with_corners()
                .filter(|(_, corners)| corners.iter().all(|n| selected.contains(n)))
                .map(|(face, _)| face)
                .collect();
            // Likewise segments of line elements between two selected nodes.
            highlight.lines = (model.line_segments())
                .map(|(_, line)| line)
                .filter(|line| line.iter().all(|n| selected.contains(n)))
                .collect();
        }
    }
    highlight
}

/// The item a dialog edits, as a draft that OK copies into the model.
enum Draft {
    Material(Material),
    Section(Section, RegionDraft),
    CrossSection(CrossSection),
    /// The step with the design region of a sensitivity step.
    Step(Step, RegionDraft),
    BoundaryCondition(usize, BoundaryCondition, RegionDraft),
    Load(usize, Load, RegionDraft),
    InitialCondition(InitialCondition, RegionDraft),
    FieldOutput(usize, FieldOutput),
    /// The region draft is unused by contact history outputs.
    HistoryOutput(usize, HistoryOutput, RegionDraft),
    /// The region draft is unused by temperatures read from a file.
    DefinedField(usize, DefinedField, RegionDraft),
    Constraint(ConstraintDraft),
    SurfaceInteraction(SurfaceInteraction, contacts::InteractionView),
    /// A contact region with side A as master and side B as slave of the drafts.
    ContactRegion(plx_model::ContactRegion, MasterSlave, RegionView),
    ContactDetection(plx_model::ContactDetection),
    /// A bolt with side A as master and side B as slave of the drafts, and its action in
    /// each step (`None` automatic).
    Bolt(
        plx_model::Bolt,
        MasterSlave,
        Vec<Option<plx_model::BoltAction>>,
        crate::bolts::BoltView,
    ),
    /// The settings the contact regions share.
    Connections(plx_model::ContactDefaults),
    ContactPair(ContactPair, MasterSlave),
    Tie(Tie, MasterSlave),
    NodeTie(NodeTie, RegionDraft),
    Amplitude(Amplitude, AmplitudeView),
    Distribution(Distribution, DistributionView),
    /// A user-defined set; editing a set read from an input file makes it one, which then
    /// replaces the set of the name given.
    MeshSet(MeshSet, RegionDraft, Option<String>),
}

/// The region clicks in the 3D view pick for, if the dialog has one.
fn draft_region(draft: &Draft) -> Option<&RegionDraft> {
    match draft {
        Draft::Section(_, r)
        | Draft::BoundaryCondition(_, _, r)
        | Draft::Load(_, _, r)
        | Draft::InitialCondition(_, r)
        | Draft::NodeTie(_, r)
        | Draft::MeshSet(_, r, _) => Some(r),
        Draft::ContactPair(_, regions)
        | Draft::Tie(_, regions)
        | Draft::ContactRegion(_, regions, _)
        | Draft::Bolt(_, regions, ..) => Some(regions.current()),
        Draft::Constraint(c) => Some(c.region()),
        Draft::HistoryOutput(_, output, r) if output.kind.region().is_some() => Some(r),
        Draft::DefinedField(_, field, r) if field.kind.takes_region() => Some(r),
        Draft::Step(step, r) if step.kind.design_region().is_some() => Some(r),
        _ => None,
    }
}

fn draft_region_mut(draft: &mut Draft) -> Option<&mut RegionDraft> {
    match draft {
        Draft::Section(_, r)
        | Draft::BoundaryCondition(_, _, r)
        | Draft::Load(_, _, r)
        | Draft::InitialCondition(_, r)
        | Draft::NodeTie(_, r)
        | Draft::MeshSet(_, r, _) => Some(r),
        Draft::ContactPair(_, regions)
        | Draft::Tie(_, regions)
        | Draft::ContactRegion(_, regions, _)
        | Draft::Bolt(_, regions, ..) => Some(regions.current_mut()),
        Draft::Constraint(c) => Some(c.region_mut()),
        Draft::HistoryOutput(_, output, r) if output.kind.region().is_some() => Some(r),
        Draft::DefinedField(_, field, r) if field.kind.takes_region() => Some(r),
        Draft::Step(step, r) if step.kind.design_region().is_some() => Some(r),
        _ => None,
    }
}

/// An open item dialog.
pub struct Editor {
    draft: Draft,
    /// Index of the edited item; `None` creates a new one.
    index: Option<usize>,
    error: Option<String>,
    /// The selection window shown while the region is picked in the 3D view.
    picker: Picker,
    /// The playing preview of a displacement boundary condition's motion.
    motion: Option<MotionPreview>,
    /// The number of sectors of a cyclic symmetry tie, found from its cut faces.
    cyclic: contacts::CyclicSectors,
    /// The expose buttons of its numbers, set up when it is first shown.
    expose: Option<expose::Scope>,
    /// The blocks value links give exposed parameters of the model from, by key.
    linked: Vec<(u32, String)>,
    /// The CAD geometry a contact detection searches before the parts are meshed.
    detection_scope: Option<connections::DetectionScope>,
    /// The step an edited boundary condition or load was in; the dialog can move it to
    /// another step.
    from_step: Option<usize>,
}

pub enum EditorResult {
    Open,
    Ok,
    Cancel,
}

fn names<'a, T: 'a>(items: &'a [T], name: impl Fn(&T) -> &str + 'a) -> Vec<&'a str> {
    items.iter().map(name).collect()
}

const DISPLACEMENT: &str = "Displacement_Rotation";
const TEMPERATURE: &str = "Temperature";
const DEFINED_TEMPERATURE: &str = "Defined_Temperature";
const INITIAL_VELOCITY: &str = "Initial_Velocity";
const INITIAL_ANGULAR_VELOCITY: &str = "Initial_Angular_Velocity";
const SUBMODEL: &str = "Submodel";
const SYMMETRY: &str = "Symmetry";
const FORCE: &str = "Force";
const MOMENT: &str = "Moment";
const PRESSURE: &str = "Pressure";
const PRE_TENSION: &str = "Pre-tension";
const CFLUX: &str = "Concentrated_Flux";
const SURFACE_FLUX: &str = "Surface_Flux";
const BODY_FLUX: &str = "Body_Flux";
const FILM: &str = "Convective_Film";
const RADIATION: &str = "Radiation";
const GRAVITY: &str = "Gravity";
const CENTRIFUGAL: &str = "Centrifugal_Load";
const MOVING: &str = "Moving_Load";

/// Whether the boundary condition dialog offers a preview of the motion: a prescribed
/// displacement that follows an amplitude of the model.
fn previews_motion(bc: &BoundaryCondition, fe: &FeModel) -> bool {
    matches!(bc.kind, BoundaryKind::Displacement(_))
        && bc
            .amplitude
            .as_deref()
            .is_some_and(|a| fe.amplitude(a).is_some())
}

/// The boundary condition kinds of the dialog: label, default name and the kind.
fn boundary_kinds() -> [(&'static str, &'static str, BoundaryKind); 4] {
    [
        // All degrees of freedom held, the most common support.
        (
            "Displacement/Rotation",
            DISPLACEMENT,
            BoundaryKind::displacement([Some(0.0); 6]),
        ),
        ("Temperature", TEMPERATURE, BoundaryKind::Temperature(0.0)),
        (
            "Submodel",
            SUBMODEL,
            BoundaryKind::Submodel {
                step: 1,
                dofs: [true, true, true, false, false, false],
            },
        ),
        (
            "Symmetry",
            SYMMETRY,
            BoundaryKind::Symmetry { normal: None },
        ),
    ]
}

fn boundary_kind_name(kind: &BoundaryKind) -> &'static str {
    match kind {
        BoundaryKind::Displacement(_) => DISPLACEMENT,
        BoundaryKind::Temperature(_) => TEMPERATURE,
        BoundaryKind::Submodel { .. } => SUBMODEL,
        BoundaryKind::Symmetry { .. } => SYMMETRY,
    }
}

/// Rows of a displacement below its values: how the selection follows them (Ansys'
/// behavior) with what the input file gets, the coordinate system of the directions and,
/// for a point, where it is and the pinball region. `names` are the step's and the boundary
/// condition's, for exposed rotations.
fn displacement_rows(
    ui: &mut Ui,
    remote: &mut Displacement,
    region: &Region,
    names: (&str, &str),
    model: &Model,
    units: UnitSystem,
) {
    let fe = &model.fe;
    ui.label("Behavior");
    egui::ComboBox::from_id_salt("displacement behavior")
        .selected_text(remote.behavior.label())
        .width(200.0)
        .show_ui(ui, |ui| {
            for behavior in Behavior::ALL {
                ui.selectable_value(&mut remote.behavior, behavior, behavior.label());
            }
        });
    ui.end_row();
    let formulation = remote.formulation(fe, &model.mesh, region, names.0, names.1);
    let point = matches!(formulation, Formulation::Point(_));
    ui.label("");
    ui.add(
        egui::Label::new(
            egui::RichText::new(formulation_hint(remote, formulation, region, model)).weak(),
        )
        .wrap(),
    );
    ui.end_row();
    ui.label("Coordinate system");
    egui::ComboBox::from_id_salt("remote system")
        .selected_text(remote.system.as_str())
        .width(200.0)
        .show_ui(ui, |ui| {
            let names = std::iter::once(GLOBAL)
                .chain(fe.coordinate_systems.iter().map(|c| c.name.as_str()));
            for name in names {
                ui.selectable_value(&mut remote.system, name.to_string(), name);
            }
        });
    ui.end_row();
    // Location and pinball region belong to a point.
    if !point {
        return;
    }
    let system = fe.coordinate_system_or_global(&remote.system);
    let cylindrical = system
        .as_ref()
        .is_some_and(|s| s.kind == plx_model::CoordinateSystemKind::Cylindrical);
    ui.label("Location");
    ui.horizontal(|ui| {
        if ui.radio(remote.location.is_none(), "Centroid").clicked() {
            remote.location = None;
        }
        if ui.radio(remote.location.is_some(), "Coordinates").clicked() && remote.location.is_none()
        {
            // Starts from the centroid, in the coordinates of the system.
            let centre = region.centroid(&model.mesh).unwrap_or([0.0; 3]);
            let local = (system.as_ref())
                .and_then(|s| s.local(centre).ok())
                .unwrap_or(centre);
            remote.location = Some(if cylindrical {
                [local[0], local[1].to_degrees(), local[2]]
            } else {
                local
            });
        }
    });
    ui.end_row();
    if let Some(location) = &mut remote.location {
        let labels = if cylindrical {
            ["R", "Theta", "Z"]
        } else {
            ["X", "Y", "Z"]
        };
        // A 2D model has no z coordinate.
        let count = if fe.properties.space.is_2d() { 2 } else { 3 };
        for (k, (value, label)) in location.iter_mut().zip(labels).take(count).enumerate() {
            ui.label(label);
            if cylindrical && k == 1 {
                ui.add(numeric::drag_value(value).speed(1.0).suffix(" °"));
            } else {
                expose::field(ui, Field::LOCATION[k], |ui| {
                    ui.add(numeric::quantity(value, units, Quantity::Length).speed(0.1))
                });
            }
            ui.end_row();
        }
    }
    ui.label("Point");
    match remote.resolve(fe, &model.mesh, region, &Default::default()) {
        Ok(point) => {
            let [x, y, z] = point
                .position
                .map(|v| crate::results::format_value(units.shown(v, Quantity::Length) as f32));
            let unit = units.unit(Quantity::Length);
            let text = format!("({x}, {y}, {z}) {unit}");
            let nodes = point.nodes.len();
            ui.label(text.trim_end()).on_hover_text(format!(
                "Global position of the point, followed by {nodes} nodes"
            ));
        }
        Err(error) if region.nodes(&model.mesh).is_empty() => {
            let _ = error;
            ui.weak("Select faces, edges or nodes");
        }
        Err(error) => {
            ui.add(
                egui::Label::new(egui::RichText::new(error).color(crate::style::error())).wrap(),
            );
        }
    }
    ui.end_row();
    ui.label("Pinball region");
    ui.horizontal(|ui| {
        let mut limited = remote.pinball.is_some();
        let switched = ui
            .checkbox(&mut limited, "")
            .on_hover_text("Only the nodes within this distance of the point follow it")
            .changed();
        if switched && limited {
            // Starts from the farthest node of the selection, which still takes them all.
            let all = Displacement {
                pinball: None,
                ..remote.clone()
            };
            let farthest = all
                .resolve(fe, &model.mesh, region, &Default::default())
                .ok()
                .map(|point| {
                    (point.nodes.iter())
                        .filter_map(|&n| model.mesh.node(n))
                        .map(|x| {
                            let d: [f64; 3] = std::array::from_fn(|i| x[i] - point.position[i]);
                            (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
                        })
                        .fold(0.0, f64::max)
                });
            // Rounded up to three digits, so the farthest node stays inside.
            let rounded = farthest.filter(|&r| r > 0.0).map(|r| {
                let scale = 10f64.powi(2 - r.log10().floor() as i32);
                (r * scale).ceil() / scale
            });
            remote.pinball = Some(rounded.unwrap_or(0.0));
        }
        let mut radius = remote.pinball.unwrap_or(0.0);
        expose::field(ui, Field::Pinball, |ui| {
            ui.add_enabled(
                limited,
                numeric::quantity(&mut radius, units, Quantity::Length).speed(0.1),
            )
        });
        remote.pinball = limited.then_some(radius.max(0.0));
    });
    ui.end_row();
}

/// What the input file gets for a displacement, as the line below its behavior says it.
fn formulation_hint(
    displacement: &Displacement,
    formulation: Formulation,
    region: &Region,
    model: &Model,
) -> String {
    let automatic = if displacement.behavior == Behavior::Automatic {
        "Automatic: "
    } else {
        ""
    };
    let text = match formulation {
        _ if region.reference_point().is_some() => {
            "The point and the rigid body it drives are held.".to_string()
        }
        Formulation::Point(RemoteBehavior::Rigid) => {
            "Rigid point at the centre: the selection keeps its shape and turns about it."
                .to_string()
        }
        Formulation::Point(RemoteBehavior::Deformable) => {
            "Point at the centre that moves with the mean of the selection, which deforms."
                .to_string()
        }
        Formulation::Nodes => {
            let nodes = region.nodes(&model.mesh);
            let expanded = plx_model::expanded_nodes(&model.mesh);
            let solid = nodes.iter().any(|n| !expanded.contains(n));
            let turned = displacement.values[3..]
                .iter()
                .any(|v| v.is_some_and(|v| v != 0.0));
            let mut text = format!("Each of the {} nodes held directly.", nodes.len());
            if solid && displacement.values[3..].iter().any(Option::is_some) {
                text.push_str(if turned {
                    " Solid nodes have no rotations: UR has no effect."
                } else {
                    " Held rotations keep it from turning; free them for a hinge."
                });
            }
            text
        }
    };
    format!("{automatic}{text}")
}

/// Rows of a symmetry boundary condition: where the normal of the plane comes from, the
/// plane found for the region and a warning when the region is no plane.
fn symmetry_rows(ui: &mut egui::Ui, normal: &mut Option<[f64; 3]>, region: &Region, model: &Model) {
    let space = model.fe.properties.space;
    let plane = symmetry_plane(&model.mesh, region, space, *normal);
    ui.label("Plane normal");
    ui.horizontal(|ui| {
        if ui.radio(normal.is_none(), "From selection").clicked() {
            *normal = None;
        }
        if ui.radio(normal.is_some(), "Entered").clicked() && normal.is_none() {
            // Starts from the plane found so far.
            *normal = Some(plane.map_or([1.0, 0.0, 0.0], |p| p.normal));
        }
    });
    ui.end_row();
    match normal {
        Some(n) => {
            let count = if space.is_2d() { 2 } else { 3 };
            for (value, label) in n.iter_mut().zip(["N1", "N2", "N3"]).take(count) {
                ui.label(label);
                ui.add(numeric::drag_value(value).speed(0.01));
                ui.end_row();
            }
        }
        None => {
            if let Ok(plane) = plane {
                ui.label("Normal");
                let [x, y, z] = plane.normal.map(|c| if c.abs() < 5e-7 { 0.0 } else { c });
                ui.label(format!("({x:.6}, {y:.6}, {z:.6})"));
                ui.end_row();
            }
        }
    }
    let warning = match plane {
        Err(SymmetryError::NoNodes) => None,
        Err(error) => Some(error.to_string()),
        Ok(plane) => plane.warning(space),
    };
    if let Some(warning) = warning {
        ui.label("");
        let mut text = warning;
        if let Some(first) = text.get_mut(0..1) {
            first.make_ascii_uppercase();
        }
        ui.add(egui::Label::new(egui::RichText::new(text).color(crate::style::error())).wrap());
        ui.end_row();
    }
    ui.label("");
    ui.weak("Holds the displacement normal to the plane and,");
    ui.end_row();
    ui.label("");
    ui.weak("for beams and shells, the rotations about the");
    ui.end_row();
    ui.label("");
    ui.weak("in-plane axes. Select the faces in the cut plane.");
    ui.end_row();
}

/// Rows of a submodel boundary condition: the displacements
/// that follow the global model. Which step of it is read, the link of the global results
/// says; the global results hold no rotations.
fn submodel_rows(ui: &mut egui::Ui, step: u32, dofs: &mut [bool; 6], two_d: bool, fe: &FeModel) {
    // Nodes of 2D models only move in the x-y plane.
    let count = if two_d { 2 } else { 3 };
    for (held, label) in dofs.iter_mut().zip(DOF_LABELS).take(count) {
        ui.label("");
        ui.checkbox(held, label);
        ui.end_row();
    }
    let note = match fe.properties.submodel_input() {
        Some(_) => format!(
            "The displacements of step {step} of the global analysis, the step whose \
             results are linked to the Global results row of this analysis."
        ),
        None => "No global results yet: in the project window, drag the results of a step \
                 of the global analysis to the Global results row of this analysis."
            .into(),
    };
    ui.label("");
    ui.add(egui::Label::new(egui::RichText::new(note).weak()).wrap());
    ui.end_row();
}

const DOF_LABELS: [&str; 6] = ["U1", "U2", "U3", "UR1", "UR2", "UR3"];

/// The load kinds of the dialog, the force first: label, default name and the kind with zero
/// values.
fn load_kinds() -> [(&'static str, &'static str, LoadKind); 12] {
    [
        ("Force", FORCE, LoadKind::force([0.0; 3])),
        ("Moment", MOMENT, LoadKind::Moment([0.0; 3])),
        ("Pressure", PRESSURE, LoadKind::Pressure(0.0)),
        ("Gravity", GRAVITY, LoadKind::Gravity([0.0, 0.0, -9.81])),
        (
            "Centrifugal load",
            CENTRIFUGAL,
            LoadKind::Centrifugal {
                point: [0.0; 3],
                axis: [0.0, 0.0, 1.0],
                speed: 0.0,
            },
        ),
        (
            "Moving Load",
            MOVING,
            LoadKind::Moving(MovingLoad::default()),
        ),
        (
            "Pre-tension (bolt preload)",
            PRE_TENSION,
            LoadKind::PreTension {
                value: 0.0,
                by_displacement: false,
                lock: false,
                direction: None,
            },
        ),
        ("Concentrated Flux", CFLUX, LoadKind::ConcentratedFlux(0.0)),
        ("Surface Flux", SURFACE_FLUX, LoadKind::SurfaceFlux(0.0)),
        ("Body Flux", BODY_FLUX, LoadKind::BodyFlux(0.0)),
        (
            "Film (Convection)",
            FILM,
            LoadKind::Film {
                sink: 20.0,
                coefficient: 0.0,
            },
        ),
        (
            "Radiation",
            RADIATION,
            LoadKind::Radiation {
                sink: 20.0,
                emissivity: 0.8,
            },
        ),
    ]
}

/// The initial condition kinds of the dialog: label, default name and the kind with zero
/// values.
fn initial_condition_kinds() -> [(&'static str, &'static str, InitialConditionKind); 3] {
    [
        (
            "Temperature",
            TEMPERATURE,
            InitialConditionKind::Temperature(20.0),
        ),
        (
            "Translational velocity",
            INITIAL_VELOCITY,
            InitialConditionKind::Velocity([0.0; 3]),
        ),
        (
            "Angular velocity",
            INITIAL_ANGULAR_VELOCITY,
            InitialConditionKind::AngularVelocity {
                point: [0.0; 3],
                axis: [0.0, 0.0, 1.0],
                speed: 0.0,
            },
        ),
    ]
}

fn initial_condition_kind_name(kind: &InitialConditionKind) -> &'static str {
    match kind {
        InitialConditionKind::Temperature(_) => TEMPERATURE,
        InitialConditionKind::Velocity(_) => INITIAL_VELOCITY,
        InitialConditionKind::AngularVelocity { .. } => INITIAL_ANGULAR_VELOCITY,
    }
}

/// The components of a vector; 2D models have none along z.
fn vector_rows(
    ui: &mut Ui,
    vector: &mut [f64; 3],
    labels: [&str; 3],
    two_d: bool,
    quantity: Quantity,
    units: UnitSystem,
) {
    let count = if two_d { 2 } else { 3 };
    let labels = labels.into_iter().zip(Field::VECTOR);
    for (value, (label, field)) in vector.iter_mut().zip(labels).take(count) {
        ui.label(label);
        expose::field(ui, field, |ui| {
            ui.add(numeric::quantity(value, units, quantity).speed(1.0))
        });
        ui.end_row();
    }
}

fn load_kind_name(kind: &LoadKind) -> &'static str {
    match kind {
        LoadKind::Moment(_) => MOMENT,
        LoadKind::Pressure(_) => PRESSURE,
        LoadKind::Force { .. } => FORCE,
        LoadKind::PreTension { .. } => PRE_TENSION,
        LoadKind::ConcentratedFlux(_) => CFLUX,
        LoadKind::SurfaceFlux(_) => SURFACE_FLUX,
        LoadKind::BodyFlux(_) => BODY_FLUX,
        LoadKind::Film { .. } => FILM,
        LoadKind::Radiation { .. } => RADIATION,
        LoadKind::Gravity(_) => GRAVITY,
        LoadKind::Centrifugal { .. } => CENTRIFUGAL,
        LoadKind::Moving(_) => MOVING,
    }
}

/// What a load acts on, which decides how its region is picked.
#[derive(Clone, Copy, PartialEq, Eq)]
enum LoadTarget {
    Nodes,
    Faces,
    Elements,
}

fn load_target(kind: &LoadKind) -> LoadTarget {
    match kind {
        LoadKind::Force { .. } | LoadKind::Moment(_) | LoadKind::ConcentratedFlux(_) => {
            LoadTarget::Nodes
        }
        LoadKind::BodyFlux(_) | LoadKind::Gravity(_) | LoadKind::Centrifugal { .. } => {
            LoadTarget::Elements
        }
        _ => LoadTarget::Faces,
    }
}

/// What a load picks its region with.
fn load_picks(kind: &LoadKind, fe: &FeModel) -> (&'static [Source], Target) {
    match load_target(kind) {
        // A force picks faces, edges or points of the geometry, like a boundary condition.
        // Forces and moments also act on the reference point of a rigid body.
        LoadTarget::Nodes if !kind.is_thermal() => (SUPPORT_SOURCES, Target::Nodes),
        LoadTarget::Nodes => (NODE_SOURCES, Target::Nodes),
        LoadTarget::Faces => (FACE_SOURCES, face_target(fe)),
        LoadTarget::Elements => (SOLID_SOURCES, face_target(fe)),
    }
}

/// An empty region for a load of the kind, or the load's region.
fn load_region(
    kind: &LoadKind,
    region: Option<&Region>,
    fe: &FeModel,
    mesh: &FeMesh,
) -> RegionDraft {
    let (sources, target) = load_picks(kind, fe);
    match region {
        Some(region) => RegionDraft::from_region(region, sources, target, mesh),
        None => RegionDraft::new(sources, target),
    }
}

/// Tells that picks did not fit the kind a dialog was switched to. [`show_dropped_note`]
/// shows it for a few seconds.
pub(crate) fn dropped_note(ui: &Ui, dropped: bool) {
    let until = dropped.then(|| ui.input(|i| i.time) + DROPPED_NOTE_SECONDS);
    ui.data_mut(|d| d.insert_temp(egui::Id::new("dropped note"), until));
}

const DROPPED_NOTE_SECONDS: f64 = 6.0;

/// The hint row left by [`dropped_note`].
pub(crate) fn show_dropped_note(ui: &mut Ui) {
    let until = ui.data(|d| d.get_temp::<Option<f64>>(egui::Id::new("dropped note")));
    if let Some(Some(until)) = until
        && ui.input(|i| i.time) < until
    {
        ui.label("");
        ui.weak("The picks do not fit this type and were cleared.");
        ui.end_row();
        ui.ctx()
            .request_repaint_after(std::time::Duration::from_secs_f64(
                until - ui.input(|i| i.time),
            ));
    }
}

impl Editor {
    pub fn create(kind: NewItem, fe: &FeModel) -> Option<Self> {
        let draft = match kind {
            NewItem::Material => Draft::Material(Material {
                name: next_name("Material", names(&fe.materials, |m| &m.name)),
                density: None,
                elastic: Some(Elastic {
                    young: 0.0,
                    poisson: 0.0,
                }),
                ..Material::default()
            }),
            NewItem::CrossSection => Draft::CrossSection(CrossSection {
                name: next_name(CrossSection::PREFIX, names(&fe.cross_sections, |c| &c.name)),
                // Ten by ten millimetres, in SI units.
                profile: BeamProfile::Rect { a: 0.01, b: 0.01 },
            }),
            NewItem::Section => Draft::Section(
                Section {
                    name: next_name("Solid_Section", names(&fe.sections, |s| &s.name)),
                    material: fe
                        .materials
                        .first()
                        .map(|m| m.name.clone())
                        .unwrap_or_default(),
                    region: Region::Parts(Vec::new()),
                    thickness: plx_model::unit_thickness(),
                    kind: SectionKind::Solid,
                },
                RegionDraft::new(ELEMENT_SOURCES, Target::Faces),
            ),
            NewItem::InitialCondition => {
                let existing = names(&fe.initial_conditions, |i| &i.name);
                Draft::InitialCondition(
                    InitialCondition {
                        distribution: None,
                        name: next_name(TEMPERATURE, existing),
                        active: true,
                        region: Region::Nodes(Vec::new()),
                        kind: InitialConditionKind::Temperature(20.0),
                    },
                    RegionDraft::new(NODE_PART_SOURCES, Target::Nodes),
                )
            }
            NewItem::DefinedField(step) => {
                // A thermal step solves for the temperatures and takes no defined field.
                let target = (fe.steps.get(step)).filter(|s| s.kind.supports_defined_fields())?;
                let existing = names(&target.defined_fields, |f| &f.name);
                Draft::DefinedField(
                    step,
                    DefinedField {
                        distribution: None,
                        name: next_name(DEFINED_TEMPERATURE, existing),
                        active: true,
                        region: Region::Nodes(Vec::new()),
                        kind: DefinedFieldKind::Temperature(20.0),
                        amplitude: None,
                    },
                    RegionDraft::new(NODE_PART_SOURCES, Target::Nodes),
                )
            }
            NewItem::Step => {
                let mut step = Step::new_static(next_name("Step", names(&fe.steps, |s| &s.name)));
                step.kind = StepKind::Static(previous_static(fe));
                Draft::Step(step, RegionDraft::new(NODE_SOURCES, face_target(fe)))
            }
            NewItem::BoundaryCondition(step) => {
                let target = fe.steps.get(step)?;
                let existing = names(&target.boundary_conditions, |b| &b.name);
                // The first kind the step takes: a heat transfer step only temperatures.
                let (_, name, kind) = boundary_kinds()
                    .into_iter()
                    .find(|(_, _, kind)| target.kind.supports_boundary(kind))?;
                Draft::BoundaryCondition(
                    step,
                    BoundaryCondition {
                        distribution: None,
                        name: next_name(name, existing),
                        active: true,
                        visible: true,
                        region: Region::Nodes(Vec::new()),
                        kind,
                        amplitude: None,
                    },
                    RegionDraft::new(SUPPORT_SOURCES, Target::Nodes),
                )
            }
            NewItem::Load(step) => {
                // A frequency step takes no loads.
                let target = fe.steps.get(step).filter(|s| s.kind.supports_loads())?;
                let existing = names(&target.loads, |l| &l.name);
                let (_, name, kind) = load_kinds()
                    .into_iter()
                    .find(|(_, _, kind)| target.kind.supports_load(kind))?;
                Draft::Load(
                    step,
                    Load {
                        distribution: None,
                        name: next_name(name, existing),
                        active: true,
                        visible: true,
                        region: Region::Nodes(Vec::new()),
                        kind: kind.clone(),
                        amplitude: None,
                        factor_amplitude: None,
                    },
                    load_region(&kind, None, fe, &FeMesh::default()),
                )
            }
            NewItem::FieldOutput(step) => {
                let target = fe.steps.get(step)?;
                let kind = OutputKind::Node;
                let output = FieldOutput::new(
                    next_name(kind.prefix(), names(&target.field_outputs, |f| &f.name)),
                    kind,
                    kind.default_variables(),
                );
                Draft::FieldOutput(step, output)
            }
            NewItem::ContactHistoryOutput(step, contact) => {
                let target = fe.steps.get(step)?;
                let pair = match contact {
                    ContactItem::Region(i) => &fe.contact_regions.get(i)?.name,
                    ContactItem::Pair(i) => &fe.contact_pairs.get(i)?.name,
                };
                let mut output = HistoryOutput::contact(
                    next_name("CH_Output", names(&target.history_outputs, |h| &h.name)),
                    pair.clone(),
                );
                // The contact forces too, summed over the slave side.
                output.variables.push("CF".into());
                output.normalize_variables();
                let region = history_region(&output.kind, fe, &FeMesh::default());
                Draft::HistoryOutput(step, output, region)
            }
            NewItem::ContactFieldOutput(step) => {
                let target = fe.steps.get(step)?;
                let kind = OutputKind::Contact;
                let output = FieldOutput::new(
                    next_name(kind.prefix(), names(&target.field_outputs, |f| &f.name)),
                    kind,
                    kind.default_variables(),
                );
                Draft::FieldOutput(step, output)
            }
            NewItem::HistoryOutput(step) => {
                let target = fe.steps.get(step)?;
                let output = HistoryOutput::node(
                    next_name("NH_Output", names(&target.history_outputs, |h| &h.name)),
                    Region::Nodes(Vec::new()),
                );
                let region = history_region(&output.kind, fe, &FeMesh::default());
                Draft::HistoryOutput(step, output, region)
            }
            NewItem::Constraint => Draft::Constraint(ConstraintDraft::new(fe)),
            NewItem::SurfaceInteraction => {
                let existing = names(&fe.surface_interactions, |s| &s.name);
                let interaction = SurfaceInteraction {
                    name: next_name("Surface_Interaction", existing),
                    properties: Vec::new(),
                };
                let view = contacts::InteractionView::new(&interaction);
                Draft::SurfaceInteraction(interaction, view)
            }
            NewItem::ContactRegion => {
                let name = next_name("Contact_Region", names(&fe.contact_regions, |r| &r.name));
                Draft::ContactRegion(
                    plx_model::ContactRegion::new(name, plx_model::ContactBehavior::Bonded),
                    MasterSlave::new(contact_target(fe)),
                    RegionView::default(),
                )
            }
            NewItem::Bolt => {
                let name = next_name("Bolt", names(&fe.bolts, |b| &b.name));
                Draft::Bolt(
                    plx_model::Bolt::new(name),
                    MasterSlave::new(contact_target(fe)),
                    vec![None; fe.steps.len()],
                    Default::default(),
                )
            }
            NewItem::ContactDetection => {
                let existing = names(&fe.contact_detections, |d| &d.name);
                let name = if existing.is_empty() {
                    "Contacts".to_owned()
                } else {
                    next_name("Contacts", existing)
                };
                Draft::ContactDetection(plx_model::ContactDetection::new(name))
            }
            NewItem::ContactPair => {
                let existing = names(&fe.contact_pairs, |c| &c.name);
                let interaction = (fe.surface_interactions.first())
                    .map(|s| s.name.clone())
                    .unwrap_or_default();
                Draft::ContactPair(
                    ContactPair::new(next_name("Contact_Pair", existing), interaction),
                    MasterSlave::new(contact_target(fe)),
                )
            }
            NewItem::Tie => {
                let name = next_name("Tie", names(&fe.ties, |t| &t.name));
                Draft::Tie(Tie::new(name), MasterSlave::new(contact_target(fe)))
            }
            NewItem::NodeTie => {
                let name = next_name("Node_Tie", names(&fe.node_ties, |t| &t.name));
                Draft::NodeTie(
                    NodeTie::new(name),
                    RegionDraft::new(NODE_SOURCES, Target::Nodes),
                )
            }
            NewItem::Amplitude => {
                let name = next_name("Amplitude", names(&fe.amplitudes, |a| &a.name));
                Draft::Amplitude(Amplitude::new(name), AmplitudeView::default())
            }
            NewItem::Distribution => {
                let name = next_name("Distribution", names(&fe.distributions, |d| &d.name));
                Draft::Distribution(Distribution::new(name), DistributionView::default())
            }
            NewItem::ResultFieldOutput
            | NewItem::ResultHistoryOutput
            | NewItem::ResultHotSpot
            | NewItem::MeshSetupItem
            | NewItem::Feature(_)
            | NewItem::MeshSet(_) => {
                return None;
            }
        };
        Some(Self {
            draft,
            index: None,
            error: None,
            picker: Picker::default(),
            motion: None,
            cyclic: Default::default(),
            expose: None,
            linked: Vec::new(),
            detection_scope: None,
            from_step: None,
        })
    }

    /// A dialog for a new node set, element set or surface, named after the sets the mesh
    /// already has.
    pub fn create_set(kind: SetKind, fe: &FeModel, mesh: &FeMesh) -> Self {
        let name = next_name(kind.prefix(), set_names(kind, fe, mesh));
        let region = RegionDraft::new(set_sources(kind), set_target(kind, fe));
        Self {
            draft: Draft::MeshSet(
                MeshSet::new(name, kind, Region::Nodes(Vec::new())),
                region,
                None,
            ),
            index: None,
            error: None,
            picker: Picker::default(),
            motion: None,
            cyclic: Default::default(),
            expose: None,
            linked: Vec::new(),
            detection_scope: None,
            from_step: None,
        }
    }

    /// The dialog of a set of the mesh: of a user-defined one, or of one read from an input
    /// file, which becomes user-defined when its nodes or faces are taken over. Element sets
    /// and node surfaces of an input file cannot be edited.
    fn edit_set(kind: SetKind, name: &str, fe: &FeModel, mesh: &FeMesh) -> Option<Self> {
        let user = (fe.mesh_sets.iter()).position(|s| s.kind == kind && s.name == name);
        let (set, index) = match user {
            Some(i) => (fe.mesh_sets[i].clone(), Some(i)),
            None => {
                let region = match kind {
                    SetKind::Nodes => Region::Nodes(mesh.node_sets.get(name)?.clone()),
                    SetKind::Surface => match mesh.surfaces.get(name)? {
                        plx_mesh::SurfaceDefinition::ElementFaces(faces) => {
                            Region::Faces(faces.clone())
                        }
                        plx_mesh::SurfaceDefinition::Nodes(_) => return None,
                    },
                    SetKind::Elements => return None,
                };
                (MeshSet::new(name, kind, region), None)
            }
        };
        let region =
            RegionDraft::from_region(&set.region, set_sources(kind), set_target(kind, fe), mesh);
        let replaces = index.is_none().then(|| name.to_owned());
        Some(Self {
            draft: Draft::MeshSet(set, region, replaces),
            index,
            error: None,
            picker: Picker::default(),
            motion: None,
            cyclic: Default::default(),
            expose: None,
            linked: Vec::new(),
            detection_scope: None,
            from_step: None,
        })
    }

    pub fn edit(item: &TreeItem, fe: &FeModel, mesh: &FeMesh) -> Option<Self> {
        let (draft, index) = match *item {
            TreeItem::NodeSet(ref name) => return Self::edit_set(SetKind::Nodes, name, fe, mesh),
            TreeItem::ElementSet(ref name) => {
                return Self::edit_set(SetKind::Elements, name, fe, mesh);
            }
            TreeItem::Surface(ref name) => return Self::edit_set(SetKind::Surface, name, fe, mesh),
            TreeItem::Material(i) => (Draft::Material(fe.materials.get(i)?.clone()), i),
            TreeItem::CrossSection(i) => {
                (Draft::CrossSection(fe.cross_sections.get(i)?.clone()), i)
            }
            TreeItem::Section(i) => {
                let section = fe.sections.get(i)?.clone();
                let region =
                    RegionDraft::from_region(&section.region, ELEMENT_SOURCES, Target::Faces, mesh);
                (Draft::Section(section, region), i)
            }
            TreeItem::Step(i) => {
                let step = fe.steps.get(i)?.clone();
                let design = step
                    .kind
                    .design_region()
                    .cloned()
                    .unwrap_or(Region::Nodes(Vec::new()));
                let region = RegionDraft::from_region(&design, NODE_SOURCES, face_target(fe), mesh);
                (Draft::Step(step, region), i)
            }
            TreeItem::BoundaryCondition(s, i) => {
                let bc = fe.steps.get(s)?.boundary_conditions.get(i)?.clone();
                let region =
                    RegionDraft::from_region(&bc.region, SUPPORT_SOURCES, Target::Nodes, mesh);
                (Draft::BoundaryCondition(s, bc, region), i)
            }
            TreeItem::Load(s, i) => {
                let load = fe.steps.get(s)?.loads.get(i)?.clone();
                let region = load_region(&load.kind, Some(&load.region), fe, mesh);
                (Draft::Load(s, load, region), i)
            }
            TreeItem::InitialCondition(i) => {
                let condition = fe.initial_conditions.get(i)?.clone();
                let region = RegionDraft::from_region(
                    &condition.region,
                    NODE_PART_SOURCES,
                    Target::Nodes,
                    mesh,
                );
                (Draft::InitialCondition(condition, region), i)
            }
            TreeItem::FieldOutput(s, i) => {
                let output = fe.steps.get(s)?.field_outputs.get(i)?.clone();
                (Draft::FieldOutput(s, output), i)
            }
            TreeItem::HistoryOutput(s, i) => {
                let output = fe.steps.get(s)?.history_outputs.get(i)?.clone();
                let region = history_region(&output.kind, fe, mesh);
                (Draft::HistoryOutput(s, output, region), i)
            }
            TreeItem::DefinedField(s, i) => {
                let field = fe.steps.get(s)?.defined_fields.get(i)?.clone();
                let region =
                    RegionDraft::from_region(&field.region, NODE_PART_SOURCES, Target::Nodes, mesh);
                (Draft::DefinedField(s, field, region), i)
            }
            TreeItem::Constraint(i) => (
                Draft::Constraint(ConstraintDraft::edit(
                    fe.constraints.get(i)?,
                    face_target(fe),
                    mesh,
                )),
                i,
            ),
            TreeItem::SurfaceInteraction(i) => {
                let interaction = fe.surface_interactions.get(i)?.clone();
                let view = contacts::InteractionView::new(&interaction);
                (Draft::SurfaceInteraction(interaction, view), i)
            }
            TreeItem::ContactPair(i) => {
                let pair = fe.contact_pairs.get(i)?.clone();
                let regions =
                    MasterSlave::from_regions(&pair.master, &pair.slave, contact_target(fe), mesh);
                (Draft::ContactPair(pair, regions), i)
            }
            TreeItem::ContactRegion(i) => {
                let region = fe.contact_regions.get(i)?.clone();
                let sides = MasterSlave::from_regions(
                    &region.side_a,
                    &region.side_b,
                    contact_target(fe),
                    mesh,
                );
                (
                    Draft::ContactRegion(region, sides, RegionView::default()),
                    i,
                )
            }
            TreeItem::ContactDetection(i) => (
                Draft::ContactDetection(fe.contact_detections.get(i)?.clone()),
                i,
            ),
            TreeItem::Bolt(i) => {
                let bolt = fe.bolts.get(i)?.clone();
                let sides =
                    MasterSlave::from_regions(&bolt.side_a, &bolt.side_b, contact_target(fe), mesh);
                let actions = crate::bolts::step_actions(fe, &bolt.name);
                (Draft::Bolt(bolt, sides, actions, Default::default()), i)
            }
            TreeItem::Group("Contacts") => (Draft::Connections(fe.contact_defaults.clone()), 0),
            TreeItem::Tie(i) => {
                let tie = fe.ties.get(i)?.clone();
                let regions =
                    MasterSlave::from_regions(&tie.master, &tie.slave, contact_target(fe), mesh);
                (Draft::Tie(tie, regions), i)
            }
            TreeItem::NodeTie(i) => {
                let tie = fe.node_ties.get(i)?.clone();
                let region =
                    RegionDraft::from_region(&tie.region, NODE_SOURCES, Target::Nodes, mesh);
                (Draft::NodeTie(tie, region), i)
            }
            TreeItem::Amplitude(i) => {
                let amplitude = fe.amplitudes.get(i)?.clone();
                (Draft::Amplitude(amplitude, AmplitudeView::default()), i)
            }
            TreeItem::Distribution(i) => {
                let distribution = fe.distributions.get(i)?.clone();
                (
                    Draft::Distribution(distribution, DistributionView::default()),
                    i,
                )
            }
            _ => return None,
        };
        let from_step = match &draft {
            Draft::BoundaryCondition(s, ..) | Draft::Load(s, ..) => Some(*s),
            _ => None,
        };
        Some(Self {
            draft,
            index: Some(index),
            error: None,
            picker: Picker::default(),
            motion: None,
            cyclic: Default::default(),
            expose: None,
            linked: Vec::new(),
            detection_scope: None,
            from_step,
        })
    }

    /// The name of the item, as the title shows it.
    pub fn name(&self) -> String {
        let title = self.title();
        title
            .split_once(": ")
            .map_or(title.clone(), |(_, name)| name.to_owned())
    }

    /// The name of the step this dialog creates, if it creates one.
    pub fn new_step(&self) -> Option<String> {
        match &self.draft {
            Draft::Step(step, _) if self.index.is_none() => Some(step.name.clone()),
            _ => None,
        }
    }

    pub fn title(&self) -> String {
        let (kind, name): (&str, &str) = match &self.draft {
            Draft::Material(m) => ("Material", &m.name),
            Draft::Section(s, _) => ("Section", &s.name),
            Draft::CrossSection(c) => ("Cross-Section", &c.name),
            Draft::Step(s, _) => ("Step", &s.name),
            Draft::BoundaryCondition(_, b, _) => ("Boundary Condition", &b.name),
            Draft::Load(_, l, _) => ("Load", &l.name),
            Draft::InitialCondition(c, _) => ("Initial Condition", &c.name),
            Draft::FieldOutput(_, f) => ("Field Output", &f.name),
            Draft::HistoryOutput(_, h, _) => ("History Output", &h.name),
            Draft::DefinedField(_, f, _) => ("Defined Field", &f.name),
            Draft::Constraint(c) => ("Constraint", c.name()),
            Draft::SurfaceInteraction(s, _) => ("Surface Interaction", &s.name),
            Draft::ContactPair(c, _) => ("Contact Pair", &c.name),
            Draft::ContactRegion(r, ..) => ("Contact Region", &r.name),
            Draft::ContactDetection(d) => ("Contact Detection", &d.name),
            Draft::Bolt(b, ..) => ("Bolt", &b.name),
            Draft::Connections(_) => return "Connections".into(),
            Draft::Tie(t, _) => ("Tie", &t.name),
            Draft::NodeTie(t, _) => ("Node Tie", &t.name),
            Draft::Amplitude(a, _) => ("Amplitude", &a.name),
            Draft::Distribution(d, _) => ("Distribution", &d.name),
            Draft::MeshSet(set, _, _) => (set.kind.label(), &set.name),
        };
        let action = if self.index.is_some() {
            "Edit"
        } else {
            "Create"
        };
        format!("{action} {kind}: {name}")
    }

    /// The boundary condition or load being edited as it would be applied: its step, its
    /// index (`None` for a new one) and the symbol item, for the 3D view.
    pub fn step_item(&self) -> Option<(usize, Option<usize>, crate::symbols::Item)> {
        let (step, kind, region) = match &self.draft {
            Draft::BoundaryCondition(s, bc, r) => {
                (*s, crate::symbols::Kind::Boundary(bc.kind.clone()), r)
            }
            Draft::Load(s, load, r) => (*s, crate::symbols::Kind::Load(load.kind.clone()), r),
            _ => return None,
        };
        let item = crate::symbols::Item {
            kind,
            region: region.region(),
            selected: true,
        };
        Some((step, self.index, item))
    }

    /// The index of the constraint being edited (`None` for a new one), if a constraint is.
    pub fn editing_constraint(&self) -> Option<Option<usize>> {
        matches!(self.draft, Draft::Constraint(_)).then_some(self.index)
    }

    /// The symbol of the constraint being edited as it would be applied, for the 3D view.
    pub fn constraint_item(&self) -> Option<crate::symbols::Item> {
        match &self.draft {
            Draft::Constraint(c) => c.symbol(),
            _ => None,
        }
    }

    /// The drawn node offsets of the playing motion preview at `now` (seconds of the GUI
    /// clock), if one plays.
    pub fn motion_offsets(&mut self, now: f64, model: &Model) -> Option<Vec<(usize, [f64; 3])>> {
        let Draft::BoundaryCondition(_, bc, region) = &self.draft else {
            return None;
        };
        let preview = self.motion.as_mut()?;
        let BoundaryKind::Displacement(Displacement { values, .. }) = bc.kind else {
            return None;
        };
        let amplitude = model.fe.amplitude(bc.amplitude.as_deref()?)?;
        let values = [values[0], values[1], values[2]];
        Some(preview.frame(now, model, region.region(), values, amplitude))
    }

    /// Whether clicks in the 3D view pick for this dialog.
    pub fn picks(&self) -> bool {
        self.region().is_some_and(RegionDraft::picks)
    }

    fn region(&self) -> Option<&RegionDraft> {
        draft_region(&self.draft)
    }

    /// A click in the 3D view: the hit with the pick tolerance there, or `None` for empty
    /// space.
    pub fn click(&mut self, model: &Model, pick: Option<(&Hit, f32)>, operation: Operation) {
        if let Some(r) = draft_region_mut(&mut self.draft) {
            r.click(model, &self.picker, pick, operation);
        }
    }

    /// A selection box dragged in the 3D view.
    pub fn box_select(&mut self, model: &Model, area: &BoxSelect, operation: Operation) {
        if let Some(r) = draft_region_mut(&mut self.draft) {
            r.box_select(model, &self.picker, area, operation);
        }
    }

    /// What a click at the hit would select, for the hover preview.
    pub fn preview(&self, model: &Model, hit: &Hit, precision: f32) -> Preview {
        self.region()
            .map(|r| r.preview(model, &self.picker, hit, precision))
            .unwrap_or_default()
    }

    /// The region being edited, for the 3D view.
    pub fn highlight(&self, model: &Model) -> Highlight {
        match &self.draft {
            Draft::ContactPair(_, regions)
            | Draft::Tie(_, regions)
            | Draft::ContactRegion(_, regions, _)
            | Draft::Bolt(_, regions, ..) => regions.highlight(model),
            Draft::Constraint(c) => c.highlight(model),
            _ => self
                .region()
                .map_or_else(Highlight::default, |r| r.highlight(model)),
        }
    }

    /// Tells the dialog which exposed parameters of the model value links give, by key, and
    /// from which block.
    /// The parts a contact detection being edited searches; empty for all visible ones.
    pub fn detection_parts(&self) -> Option<&[String]> {
        match &self.draft {
            Draft::ContactDetection(detection) => Some(&detection.parts),
            _ => None,
        }
    }

    pub fn detection_scope(&self) -> Option<&connections::DetectionScope> {
        self.detection_scope.as_ref()
    }

    /// Sets the CAD geometry a contact detection being edited searches, `None` for the mesh.
    pub fn set_detection_scope(&mut self, scope: Option<connections::DetectionScope>) {
        self.detection_scope = scope;
    }

    pub fn set_linked(&mut self, linked: Vec<(u32, String)>) {
        if self.linked != linked {
            self.linked = linked;
            self.expose = None;
        }
    }

    /// The item as it is in the model, for an item whose numbers can be exposed; `None` for
    /// a new one.
    fn original_item(&self, fe: &FeModel) -> Option<ItemRef> {
        let index = self.index?;
        let step = |s: usize| fe.steps.get(s).map(|step| step.name.clone());
        Some(match &self.draft {
            Draft::Material(_) => ItemRef::Material(fe.materials.get(index)?.name.clone()),
            Draft::Section(..) => ItemRef::Section(fe.sections.get(index)?.name.clone()),
            Draft::CrossSection(_) => {
                ItemRef::CrossSection(fe.cross_sections.get(index)?.name.clone())
            }
            Draft::Step(..) => ItemRef::Step(step(index)?),
            Draft::BoundaryCondition(s, ..) => ItemRef::BoundaryCondition {
                name: fe
                    .steps
                    .get(*s)?
                    .boundary_conditions
                    .get(index)?
                    .name
                    .clone(),
                step: step(*s)?,
            },
            Draft::Load(s, ..) => ItemRef::Load {
                name: fe.steps.get(*s)?.loads.get(index)?.name.clone(),
                step: step(*s)?,
            },
            Draft::InitialCondition(..) => {
                ItemRef::InitialCondition(fe.initial_conditions.get(index)?.name.clone())
            }
            Draft::DefinedField(s, ..) => ItemRef::DefinedField {
                name: fe.steps.get(*s)?.defined_fields.get(index)?.name.clone(),
                step: step(*s)?,
            },
            _ => return None,
        })
    }

    /// The item as the dialog has it now, by its new name, in a model of only that item; for
    /// an item whose numbers can be exposed.
    fn draft_item(&self, fe: &FeModel) -> Option<(ItemRef, FeModel)> {
        let mut model = FeModel {
            // The location of a remote displacement depends on the kind of its system.
            coordinate_systems: fe.coordinate_systems.clone(),
            ..FeModel::default()
        };
        let step = |s: usize| {
            let name = fe.steps.get(s)?.name.clone();
            Some(Step {
                kind: fe.steps.get(s)?.kind.clone(),
                ..Step::new_static(name)
            })
        };
        let item = match &self.draft {
            Draft::Material(material) => {
                model.materials.push(material.clone());
                ItemRef::Material(material.name.clone())
            }
            Draft::Section(section, _) => {
                model.sections.push(section.clone());
                ItemRef::Section(section.name.clone())
            }
            Draft::CrossSection(cross_section) => {
                model.cross_sections.push(cross_section.clone());
                ItemRef::CrossSection(cross_section.name.clone())
            }
            Draft::Step(own, _) => {
                model.steps.push(own.clone());
                ItemRef::Step(own.name.clone())
            }
            Draft::BoundaryCondition(s, bc, _) => {
                let mut step = step(*s)?;
                step.boundary_conditions.push(bc.clone());
                let item = ItemRef::BoundaryCondition {
                    step: step.name.clone(),
                    name: bc.name.clone(),
                };
                model.steps.push(step);
                item
            }
            Draft::Load(s, load, _) => {
                let mut step = step(*s)?;
                step.loads.push(load.clone());
                let item = ItemRef::Load {
                    step: step.name.clone(),
                    name: load.name.clone(),
                };
                model.steps.push(step);
                item
            }
            Draft::InitialCondition(condition, _) => {
                model.initial_conditions.push(condition.clone());
                ItemRef::InitialCondition(condition.name.clone())
            }
            Draft::DefinedField(s, field, _) => {
                let mut step = step(*s)?;
                step.defined_fields.push(field.clone());
                let item = ItemRef::DefinedField {
                    step: step.name.clone(),
                    name: field.name.clone(),
                };
                model.steps.push(step);
                item
            }
            _ => return None,
        };
        Some((item, model))
    }

    /// Draws the form with the expose buttons of the numbers that have them.
    fn exposing_form(&mut self, ui: &mut Ui, model: &Model) {
        let Some((item, draft)) = self.draft_item(&model.fe) else {
            return self.form(ui, model);
        };
        let mut scope = match self.expose.take() {
            Some(scope) => scope,
            None => expose::Scope::new(&model.fe, self.original_item(&model.fe), &self.linked),
        };
        scope.available = draft.exposable(&item);
        scope.show(ui, |ui| self.form(ui, model));
        self.expose = Some(scope);
    }

    pub fn show(&mut self, ctx: &egui::Context, model: &Model) -> EditorResult {
        let mut result = EditorResult::Open;
        let mut open = true;
        let title = self.title();
        let mut window = egui::Window::new(title.clone())
            .id(crate::ids::window(ctx, "item editor"))
            .collapsible(false)
            .resizable(false);
        if matches!(self.draft, Draft::Constraint(_)) {
            // The types differ in how much room their rows need; without a fixed width the
            // window would jump between them, and egui never shrinks it again.
            window = window
                .min_width(crate::constraint_dialog::WIDTH)
                .max_width(crate::constraint_dialog::WIDTH);
        }
        let window = window
            .pivot(egui::Align2::LEFT_TOP)
            .default_pos(ctx.content_rect().left_top() + egui::vec2(300.0, 90.0));
        let window = crate::dock::show(ctx, window, title, &mut open, |ui| {
            if let Draft::SurfaceInteraction(interaction, view) = &mut self.draft {
                contacts::interaction_dialog(ui, interaction, view, model.fe.properties.units);
            } else if let Draft::Amplitude(amplitude, view) = &mut self.draft {
                let units = model.fe.properties.units;
                amplitude_dialog::amplitude_form(ui, amplitude, view, units);
            } else {
                egui::Grid::new("item form")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| self.exposing_form(ui, model));
            }
            if let Some(error) = &self.error {
                ui.colored_label(crate::style::error(), error);
            }
            ui.separator();
            crate::dock::footer(ui, |ui| {
                if ui.button("Cancel").clicked() {
                    result = EditorResult::Cancel;
                }
                if ui.button("OK").clicked() {
                    let error =
                        (self.validate(&model.fe).err()).or_else(|| self.validate_set(model).err());
                    self.error = error.or_else(|| self.validate_sectors(model).err());
                    if self.error.is_none() {
                        result = EditorResult::Ok;
                    }
                }
            });
        });
        if let Some(window) = window
            && let Some(region) = draft_region_mut(&mut self.draft)
        {
            region.picker_window(ctx, window.response.rect, &mut self.picker, model);
        }
        if !open {
            result = EditorResult::Cancel;
        }
        result
    }

    fn form(&mut self, ui: &mut Ui, model: &Model) {
        let taken = self.taken(&model.fe);
        let space = model.fe.properties.space;
        let (two_d, axisymmetric) = (space.is_2d(), space == ModelSpace::Axisymmetric);
        let units = model.fe.properties.units;
        match &mut self.draft {
            Draft::Material(material) => material_form(ui, material, units),
            Draft::CrossSection(cross_section) => {
                crate::cross_section::form(ui, cross_section, units);
            }
            Draft::Section(section, region) => {
                name_row(ui, &mut section.name);
                ui.label("Type");
                let before = section.kind.prefix();
                egui::ComboBox::from_id_salt("section kind")
                    .selected_text(section.kind.label())
                    .width(200.0)
                    .show_ui(ui, |ui| {
                        for kind in SectionKind::ALL {
                            // Shells and membranes live in 3D models; 2D elements get a
                            // solid section.
                            if matches!(
                                kind,
                                SectionKind::Shell { .. } | SectionKind::Membrane { .. }
                            ) && two_d
                            {
                                continue;
                            }
                            let selected = kind.prefix() == section.kind.prefix();
                            if ui.selectable_label(selected, kind.label()).clicked() && !selected {
                                section.kind = kind;
                            }
                        }
                    });
                ui.end_row();
                // Another kind keeps a name the user chose and renumbers a default one.
                if section.kind.prefix() != before
                    && (section.name.strip_prefix(before))
                        .and_then(|n| n.strip_prefix('-'))
                        .is_some_and(|n| n.parse::<u32>().is_ok())
                {
                    section.name = next_name(section.kind.prefix(), taken.iter().copied());
                }
                ui.label("Material");
                egui::ComboBox::from_id_salt("section material")
                    .selected_text(section.material.as_str())
                    .width(200.0)
                    .show_ui(ui, |ui| {
                        for material in &model.fe.materials {
                            let name = material.name.clone();
                            ui.selectable_value(&mut section.material, name, &material.name);
                        }
                    });
                ui.end_row();
                let membrane = matches!(section.kind, SectionKind::Membrane { .. });
                match &mut section.kind {
                    SectionKind::Solid => {
                        // Plane stress and plane strain sections have a thickness.
                        if model.fe.properties.space.has_thickness() {
                            ui.label("Thickness");
                            expose::field(ui, Field::Thickness, |ui| {
                                ui.add(
                                    numeric::quantity(
                                        &mut section.thickness,
                                        units,
                                        Quantity::Length,
                                    )
                                    .range(0.0..=f64::MAX),
                                )
                            });
                            ui.end_row();
                        }
                    }
                    SectionKind::Truss { area } => {
                        ui.label("Cross-section area");
                        expose::field(ui, Field::Area, |ui| {
                            ui.add(
                                numeric::quantity(area, units, Quantity::Area)
                                    .range(0.0..=f64::MAX),
                            )
                        });
                        ui.end_row();
                    }
                    SectionKind::Beam(beam) => beam_form(ui, beam, &model.fe, units),
                    SectionKind::Shell { thickness, offset }
                    | SectionKind::Membrane { thickness, offset } => {
                        ui.label("Thickness");
                        expose::field(ui, Field::Thickness, |ui| {
                            ui.add(
                                numeric::quantity(thickness, units, Quantity::Length)
                                    .range(0.0..=f64::MAX),
                            )
                        });
                        ui.end_row();
                        ui.label("Offset");
                        ui.add(numeric::drag_value(offset).speed(0.01).range(-1.0..=1.0));
                        ui.end_row();
                        ui.label("");
                        ui.weak("Offset of the mesh from the mid-surface as a fraction");
                        ui.end_row();
                        ui.label("");
                        ui.weak("of the thickness: 0.5 puts the mesh on the top face.");
                        ui.end_row();
                        if membrane {
                            ui.label("");
                            ui.weak("Membranes carry in-plane forces only. Linear shells are");
                            ui.end_row();
                            ui.label("");
                            ui.weak("written as membranes (M3D3, M3D4); quadratic ones are");
                            ui.end_row();
                            ui.label("");
                            ui.weak("refused: mesh the part with Second order off.");
                            ui.end_row();
                        }
                    }
                }
                region.ui(ui, model);
            }
            Draft::Step(step, region) => {
                let sectors = (model.fe.cyclic_tie())
                    .and_then(|t| t.cyclic_sectors(&model.mesh))
                    .and_then(Result::ok);
                step_form(ui, step, self.index.is_none(), &model.fe, sectors);
                crate::connections::step_contacts(ui, &model.fe, step, self.index);
                if step.kind.design_region().is_some() {
                    region.ui_labeled(ui, model, "Design region", "design region", true);
                    ui.label("");
                    ui.weak("Surface nodes that may move; the solver ignores inner nodes.");
                    ui.end_row();
                }
            }
            Draft::BoundaryCondition(step, bc, region) => {
                step_row(ui, step, &model.fe, |_| true);
                name_row(ui, &mut bc.name);
                ui.label("Type");
                let step_kind = model.fe.steps.get(*step).map(|s| &s.kind);
                let current = boundary_kind_name(&bc.kind);
                let label = (boundary_kinds().into_iter())
                    .find(|(_, name, _)| *name == current)
                    .map_or("", |(label, ..)| label);
                egui::ComboBox::from_id_salt("boundary kind")
                    .selected_text(label)
                    .width(200.0)
                    .show_ui(ui, |ui| {
                        for (label, name, kind) in boundary_kinds() {
                            // Only the kinds the step takes.
                            if step_kind.is_some_and(|s| !s.supports_boundary(&kind)) {
                                continue;
                            }
                            if ui.selectable_label(current == name, label).clicked()
                                && current != name
                            {
                                bc.kind = kind;
                                rename_default(&mut bc.name, current, name, &taken);
                            }
                        }
                    });
                ui.end_row();
                if let BoundaryKind::Temperature(t) = &mut bc.kind {
                    ui.label("Temperature");
                    expose::field(ui, Field::Value, |ui| {
                        ui.add(numeric::quantity(t, units, Quantity::Temperature).speed(1.0))
                    });
                    ui.end_row();
                    distribution_row(ui, "bc distribution", &mut bc.distribution, &model.fe);
                } else {
                    bc.distribution = None;
                }
                // Nodes of 2D models only move in the x-y plane; a point, of a rigid body or
                // of a displacement, also turns about z in a plane model.
                let dofs: Vec<usize> = match &bc.kind {
                    BoundaryKind::Displacement(_) => {
                        model.fe.properties.space.remote_dofs().to_vec()
                    }
                    _ if two_d => vec![0, 1],
                    _ => (0..6).collect(),
                };
                let values = match &mut bc.kind {
                    BoundaryKind::Displacement(displacement) => Some(&mut displacement.values),
                    _ => None,
                };
                if let Some(values) = values {
                    for i in dofs {
                        let (value, label) = (&mut values[i], DOF_LABELS[i]);
                        let mut set = value.is_some();
                        ui.checkbox(&mut set, label);
                        let mut number = value.unwrap_or(0.0);
                        // Displacements are lengths, rotations angles in radian.
                        let quantity = if i < 3 {
                            Quantity::Length
                        } else {
                            Quantity::Angle
                        };
                        let field = numeric::quantity(&mut number, units, quantity).speed(0.01);
                        let path = if i < 3 {
                            Field::VECTOR[i]
                        } else {
                            Field::ROTATION[i - 3]
                        };
                        expose::field(ui, path, |ui| ui.add_enabled(set, field));
                        *value = set.then_some(number);
                        ui.end_row();
                    }
                }
                if let BoundaryKind::Displacement(displacement) = &mut bc.kind {
                    let step = model.fe.steps.get(*step).map_or("", |s| s.name.as_str());
                    let names = (step, bc.name.as_str());
                    displacement_rows(ui, displacement, &region.region(), names, model, units);
                }
                if let BoundaryKind::Submodel { step, dofs } = &mut bc.kind {
                    submodel_rows(ui, *step, dofs, two_d, &model.fe);
                }
                if let BoundaryKind::Symmetry { normal } = &mut bc.kind {
                    symmetry_rows(ui, normal, &region.region(), model);
                }
                if bc.kind.takes_amplitude() {
                    let playing = self.motion.is_some();
                    let playable = previews_motion(bc, &model.fe);
                    let mut toggle = false;
                    amplitude_row_with(
                        ui,
                        "Amplitude",
                        "bc amplitude",
                        &mut bc.amplitude,
                        &model.fe,
                        |ui| {
                            if playable {
                                let (icon, tooltip) = if playing {
                                    (Icon::Stop, "Stop the preview")
                                } else {
                                    (
                                        Icon::Animate,
                                        "Preview the prescribed motion in the 3D view",
                                    )
                                };
                                toggle = icons::dialog_button(ui, icon, tooltip, true).clicked();
                            }
                        },
                    );
                    if !previews_motion(bc, &model.fe) {
                        self.motion = None;
                    } else if toggle {
                        self.motion = match self.motion {
                            Some(_) => None,
                            None => Some(MotionPreview::new(ui.input(|i| i.time), model)),
                        };
                    }
                    if let Some(frame) = self.motion.as_ref().and_then(|m| m.last.as_ref()) {
                        ui.label("");
                        ui.weak(format!(
                            "Time {}, motion drawn {}x",
                            crate::results::format_value(frame.time as f32),
                            crate::results::format_value(frame.scale as f32),
                        ));
                        ui.end_row();
                        if let BoundaryKind::Displacement(displacement) = &bc.kind
                            && displacement.values[3..].iter().any(Option::is_some)
                        {
                            ui.label("");
                            ui.weak("Rotations are not shown in the preview.");
                            ui.end_row();
                        }
                    }
                }
                region.ui(ui, model);
            }
            Draft::Load(step, load, region) => {
                step_row(ui, step, &model.fe, |s| s.kind.supports_loads());
                name_row(ui, &mut load.name);
                ui.label("Type");
                let step_kind = model.fe.steps.get(*step).map(|s| &s.kind);
                let current = load_kind_name(&load.kind);
                let label = (load_kinds().into_iter())
                    .find(|(_, name, _)| *name == current)
                    .map_or("", |(label, ..)| label);
                egui::ComboBox::from_id_salt("load kind")
                    .selected_text(label)
                    .width(200.0)
                    .show_ui(ui, |ui| {
                        for (label, name, kind) in load_kinds() {
                            // Only the kinds the step takes.
                            if step_kind.is_some_and(|s| !s.supports_load(&kind)) {
                                continue;
                            }
                            // A moving load drives over a surface, which a revolution has not.
                            if axisymmetric && matches!(kind, LoadKind::Moving(_)) {
                                continue;
                            }
                            if ui.selectable_label(current == name, label).clicked()
                                && current != name
                            {
                                let moved = load_target(&kind) != load_target(&load.kind);
                                load.kind = kind;
                                rename_default(&mut load.name, current, name, &taken);
                                if moved {
                                    let (sources, target) = load_picks(&load.kind, &model.fe);
                                    let (kept, dropped) =
                                        region.converted(sources, target, &model.mesh);
                                    *region = kept;
                                    dropped_note(ui, dropped);
                                }
                            }
                        }
                    });
                ui.end_row();
                show_dropped_note(ui);
                // Only a pressure varies by a distribution.
                if !matches!(load.kind, LoadKind::Pressure(_)) {
                    load.distribution = None;
                }
                match &mut load.kind {
                    LoadKind::Moment(moment) => {
                        if two_d {
                            ui.label("");
                            ui.weak("Moments need a 3D model.");
                            ui.end_row();
                        } else {
                            let labels = ["M1", "M2", "M3"].into_iter().zip(Field::VECTOR);
                            for (value, (label, field)) in moment.iter_mut().zip(labels) {
                                ui.label(label);
                                expose::field(ui, field, |ui| {
                                    ui.add(
                                        numeric::quantity(value, units, Quantity::Moment)
                                            .speed(1.0),
                                    )
                                });
                                ui.end_row();
                            }
                        }
                        ui.label("");
                        ui.weak(
                            "Acts at the reference point of a rigid body, or at every node of \
                             the region that has rotations (beams, shells).",
                        );
                        ui.end_row();
                    }
                    LoadKind::Pressure(pressure) => {
                        ui.label("Pressure");
                        expose::field(ui, Field::Value, |ui| {
                            ui.add(
                                numeric::quantity(pressure, units, Quantity::Pressure).speed(0.1),
                            )
                        });
                        ui.end_row();
                        let reference = &mut load.distribution;
                        distribution_row(ui, "load distribution", reference, &model.fe);
                    }
                    LoadKind::Force { value, each_node } => {
                        force_rows(ui, value, two_d, units);
                        ui.label("");
                        ui.checkbox(each_node, "Apply to all nodes individually");
                        ui.end_row();
                        ui.label("");
                        ui.weak(if *each_node {
                            "The whole force acts at every node of the region."
                        } else {
                            "Total force: on faces by area, on edges by length, on points in \
                             equal shares."
                        });
                        ui.end_row();
                        revolution_hint(ui, axisymmetric);
                    }
                    LoadKind::PreTension {
                        value,
                        by_displacement,
                        lock,
                        direction,
                    } => {
                        // Force, displacement or lock: one choice of the two flags.
                        let mut by = (*lock, *by_displacement);
                        ui.label("Preload by");
                        ui.horizontal(|ui| {
                            ui.radio_value(&mut by, (false, false), "Force");
                            ui.radio_value(&mut by, (false, true), "Displacement");
                            ui.radio_value(&mut by, (true, *by_displacement), "Lock")
                                .on_hover_text(
                                    "Keeps the length the bolt has at the end of the previous \
                                     step; the bolt force then follows the other loads.",
                                );
                        });
                        (*lock, *by_displacement) = by;
                        ui.end_row();
                        if *lock {
                            ui.label("");
                            ui.weak("Length of the previous step");
                        } else if *by_displacement {
                            ui.label("Shortening");
                            expose::field(ui, Field::Value, |ui| {
                                ui.add(
                                    numeric::quantity(value, units, Quantity::Length).speed(0.001),
                                )
                            });
                        } else {
                            ui.label("Force");
                            expose::field(ui, Field::Value, |ui| {
                                ui.add(numeric::quantity(value, units, Quantity::Force).speed(1.0))
                            });
                        }
                        ui.end_row();
                        let mut given = direction.is_some();
                        ui.label("Direction");
                        ui.checkbox(&mut given, "Given (otherwise the surface normal)");
                        ui.end_row();
                        let mut values = direction.unwrap_or([1.0, 0.0, 0.0]);
                        for (value, label) in values.iter_mut().zip(["    X", "    Y", "    Z"]) {
                            ui.label(label);
                            ui.add_enabled(given, numeric::drag_value(value).speed(0.01));
                            ui.end_row();
                        }
                        *direction = given.then_some(values);
                        for line in [
                            "Select the element faces on one side of a cut through the bolt shank.",
                            "A positive force pulls the two sides together; a displacement shortens the bolt.",
                            "The shortening counts from the start: 0 loosens the bolt.",
                            "Lock in a later step to keep the bolt at its length.",
                        ] {
                            ui.label("");
                            ui.weak(line);
                            ui.end_row();
                        }
                    }
                    LoadKind::ConcentratedFlux(flux) => {
                        ui.label("Heat flux");
                        expose::field(ui, Field::Value, |ui| {
                            ui.add(numeric::physical(flux, units, Quantity::Power))
                        });
                        ui.end_row();
                        ui.label("");
                        ui.weak("The heat flux flows into every node of the region.");
                        ui.end_row();
                    }
                    LoadKind::SurfaceFlux(flux) => {
                        ui.label("Heat flux density");
                        expose::field(ui, Field::Value, |ui| {
                            ui.add(numeric::physical(flux, units, Quantity::HeatFlux))
                        });
                        ui.end_row();
                        ui.label("");
                        ui.weak("Positive: heat flows into the part.");
                        ui.end_row();
                    }
                    LoadKind::BodyFlux(flux) => {
                        ui.label("Heat source");
                        expose::field(ui, Field::Value, |ui| {
                            ui.add(numeric::physical(flux, units, Quantity::PowerPerVolume))
                        });
                        ui.end_row();
                    }
                    LoadKind::Film { sink, coefficient } => {
                        ui.label("Sink temperature");
                        expose::field(ui, Field::Sink, |ui| {
                            ui.add(numeric::quantity(sink, units, Quantity::Temperature).speed(1.0))
                        });
                        ui.end_row();
                        ui.label("Film coefficient");
                        expose::field(ui, Field::FilmCoefficient, |ui| {
                            ui.add(
                                numeric::physical(
                                    coefficient,
                                    units,
                                    Quantity::HeatTransferCoefficient,
                                )
                                .range(0.0..=f64::MAX),
                            )
                        });
                        ui.end_row();
                    }
                    LoadKind::Gravity(acceleration) => {
                        let count = if two_d { 2 } else { 3 };
                        let labels = ["g1", "g2", "g3"].into_iter().zip(Field::VECTOR);
                        for (value, (label, field)) in
                            acceleration.iter_mut().zip(labels).take(count)
                        {
                            ui.label(label);
                            expose::field(ui, field, |ui| {
                                ui.add(
                                    numeric::quantity(value, units, Quantity::Acceleration)
                                        .speed(0.1),
                                )
                            });
                            ui.end_row();
                        }
                        ui.label("");
                        ui.weak("Acceleration vector; the materials need a density.");
                        ui.end_row();
                    }
                    LoadKind::Centrifugal { point, axis, speed } => {
                        ui.label("Rotational speed");
                        expose::field(ui, Field::Speed, |ui| {
                            ui.add(
                                numeric::quantity(speed, units, Quantity::RotationalSpeed)
                                    .speed(1.0),
                            )
                        });
                        ui.end_row();
                        ui.label("");
                        ui.weak(format!(
                            "= {} rpm",
                            numeric::format_physical(*speed * 60.0 / std::f64::consts::TAU)
                        ));
                        ui.end_row();
                        for ((value, label), field) in point
                            .iter_mut()
                            .zip(["Axis point X", "Axis point Y", "Axis point Z"])
                            .zip(Field::LOCATION)
                        {
                            ui.label(label);
                            expose::field(ui, field, |ui| {
                                ui.add(numeric::quantity(value, units, Quantity::Length).speed(0.1))
                            });
                            ui.end_row();
                        }
                        for ((value, label), field) in axis
                            .iter_mut()
                            .zip(["Axis direction X", "Axis direction Y", "Axis direction Z"])
                            .zip(Field::AXIS)
                        {
                            ui.label(label);
                            expose::field(ui, field, |ui| {
                                ui.add(numeric::drag_value(value).speed(0.01))
                            });
                            ui.end_row();
                        }
                        ui.label("");
                        ui.weak(
                            "The solver takes the square of the speed; the materials need a density.",
                        );
                        ui.end_row();
                    }
                    LoadKind::Moving(moving) => moving_rows(ui, moving, &model.fe, two_d, units),
                    LoadKind::Radiation { sink, emissivity } => {
                        ui.label("Sink temperature");
                        expose::field(ui, Field::Sink, |ui| {
                            ui.add(numeric::quantity(sink, units, Quantity::Temperature).speed(1.0))
                        });
                        ui.end_row();
                        ui.label("Emissivity");
                        expose::field(ui, Field::Emissivity, |ui| {
                            ui.add(
                                numeric::drag_value(emissivity)
                                    .range(0.0..=1.0)
                                    .speed(0.01)
                                    .max_decimals(4),
                            )
                        });
                        ui.end_row();
                        let properties = &model.fe.properties;
                        if properties.absolute_zero.is_none()
                            || properties.stefan_boltzmann.is_none()
                        {
                            ui.label("");
                            ui.colored_label(
                                crate::style::error(),
                                "Radiation needs the physical constants\n\
                                 (Model Properties).",
                            );
                            ui.end_row();
                        }
                    }
                }
                // Of a film or radiation the first amplitude scales the sink temperature.
                let factor = load.kind.factor_amplitude_label();
                let label = if factor.is_some() {
                    "Sink temperature amplitude"
                } else {
                    "Amplitude"
                };
                amplitude_row(ui, label, "load amplitude", &mut load.amplitude, &model.fe);
                if let Some(factor) = factor {
                    let label = format!("Amplitude {factor}");
                    let reference = &mut load.factor_amplitude;
                    amplitude_row(ui, &label, "load factor amplitude", reference, &model.fe);
                }
                region.ui(ui, model);
            }
            Draft::InitialCondition(condition, region) => {
                name_row(ui, &mut condition.name);
                ui.label("Type");
                let current = initial_condition_kind_name(&condition.kind);
                let label = (initial_condition_kinds().into_iter())
                    .find(|(_, name, _)| *name == current)
                    .map_or("", |(label, ..)| label);
                egui::ComboBox::from_id_salt("initial condition kind")
                    .selected_text(label)
                    .width(200.0)
                    .show_ui(ui, |ui| {
                        for (label, name, kind) in initial_condition_kinds() {
                            if ui.selectable_label(current == name, label).clicked()
                                && current != name
                            {
                                condition.kind = kind;
                                rename_default(&mut condition.name, current, name, &taken);
                            }
                        }
                    });
                ui.end_row();
                if condition.kind.is_velocity() {
                    condition.distribution = None;
                }
                match &mut condition.kind {
                    InitialConditionKind::Temperature(t) => {
                        ui.label("Temperature");
                        expose::field(ui, Field::Value, |ui| {
                            ui.add(numeric::quantity(t, units, Quantity::Temperature).speed(1.0))
                        });
                        ui.end_row();
                        let reference = &mut condition.distribution;
                        distribution_row(ui, "initial distribution", reference, &model.fe);
                    }
                    InitialConditionKind::Velocity(v) => {
                        vector_rows(ui, v, ["V1", "V2", "V3"], two_d, Quantity::Velocity, units);
                    }
                    InitialConditionKind::AngularVelocity { point, axis, speed } => {
                        let count = if two_d { 2 } else { 3 };
                        let labels = ["Axis point X", "Axis point Y", "Axis point Z"]
                            .into_iter()
                            .zip(Field::LOCATION);
                        for (value, (label, field)) in point.iter_mut().zip(labels).take(count) {
                            ui.label(label);
                            expose::field(ui, field, |ui| {
                                ui.add(numeric::quantity(value, units, Quantity::Length).speed(1.0))
                            });
                            ui.end_row();
                        }
                        if two_d {
                            ui.label("Axis");
                            ui.label("Z");
                            ui.end_row();
                        } else {
                            let labels =
                                ["Axis direction X", "Axis direction Y", "Axis direction Z"]
                                    .into_iter()
                                    .zip(Field::AXIS);
                            for (value, (label, field)) in axis.iter_mut().zip(labels) {
                                ui.label(label);
                                expose::field(ui, field, |ui| {
                                    ui.add(numeric::drag_value(value).speed(0.1))
                                });
                                ui.end_row();
                            }
                        }
                        ui.label("Rotational speed");
                        expose::field(ui, Field::Speed, |ui| {
                            ui.add(
                                numeric::quantity(speed, units, Quantity::RotationalSpeed)
                                    .speed(1.0),
                            )
                        });
                        ui.end_row();
                        ui.label("");
                        ui.weak(format!(
                            "{:.4} rpm; a positive speed turns counter-clockwise about the axis.",
                            *speed * 60.0 / std::f64::consts::TAU
                        ));
                        ui.end_row();
                    }
                }
                region.ui(ui, model);
                ui.label("");
                let hint = match condition.kind {
                    InitialConditionKind::Temperature(_) => {
                        "Temperature before the first step, e.g. for thermal expansion."
                    }
                    _ => "Velocity at the start of a Dynamic step; other steps ignore it.",
                };
                ui.weak(hint);
                ui.end_row();
            }
            Draft::DefinedField(_, field, region) => {
                name_row(ui, &mut field.name);
                ui.label("Type");
                ui.label("Temperature");
                ui.end_row();
                ui.label("Source");
                let by_value = field.kind.takes_region();
                ui.horizontal(|ui| {
                    if ui.radio(by_value, "By value").clicked() && !by_value {
                        field.kind = DefinedFieldKind::Temperature(20.0);
                    }
                    if ui.radio(!by_value, "From result file").clicked() && by_value {
                        field.kind = DefinedFieldKind::TemperatureFromFile {
                            file: Default::default(),
                            step: 1,
                        };
                    }
                });
                ui.end_row();
                if !field.kind.takes_region() {
                    field.distribution = None;
                }
                match &mut field.kind {
                    DefinedFieldKind::Temperature(t) => {
                        ui.label("Temperature");
                        expose::field(ui, Field::Value, |ui| {
                            ui.add(numeric::quantity(t, units, Quantity::Temperature).speed(1.0))
                        });
                        ui.end_row();
                        let reference = &mut field.distribution;
                        distribution_row(ui, "defined field distribution", reference, &model.fe);
                        amplitude_row(
                            ui,
                            "Amplitude",
                            "defined field amplitude",
                            &mut field.amplitude,
                            &model.fe,
                        );
                        region.ui(ui, model);
                    }
                    DefinedFieldKind::TemperatureFromFile { file, step } => {
                        ui.label("Result file");
                        ui.horizontal(|ui| {
                            let name = file.file_name().map_or_else(
                                || "(none)".to_string(),
                                |n| n.to_string_lossy().into_owned(),
                            );
                            ui.label(name).on_hover_text(file.display().to_string());
                            if ui.button("...").clicked() {
                                let mut dialog = rfd::FileDialog::new()
                                    .set_title("Result file of the heat transfer analysis")
                                    .add_filter("CalculiX results (*.frd)", &["frd"]);
                                if let Some(dir) = file.parent().filter(|d| d.is_dir()) {
                                    dialog = dialog.set_directory(dir);
                                }
                                if let Some(picked) = dialog.pick_file() {
                                    *file = picked;
                                }
                            }
                        });
                        ui.end_row();
                        ui.label("Step");
                        ui.add(numeric::drag_value(step).range(1..=9999).speed(0.1));
                        ui.end_row();
                        ui.label("");
                        ui.weak("Temperatures of all nodes from the .frd file of a heat");
                        ui.end_row();
                        ui.label("");
                        ui.weak("transfer analysis on the same mesh (step counted from 1).");
                        ui.end_row();
                        ui.label("");
                        ui.weak(
                            "The file is copied next to the input file when the analysis starts.",
                        );
                        ui.end_row();
                    }
                }
                ui.label("");
                ui.weak("Needs an initial temperature and a thermal expansion of the material.");
                ui.end_row();
            }
            Draft::NodeTie(tie, region) => {
                name_row(ui, &mut tie.name);
                region.ui(ui, model);
                ui.label("Rotations");
                ui.checkbox(&mut tie.rotations, "rigid");
                ui.end_row();
                ui.label("");
                ui.add(
                    egui::Label::new(
                        egui::RichText::new(
                            "All nodes of the region follow the first one. Connects the ends \
                             of beams or trusses of different parts; the contact search \
                             finds them. Rigid also couples the rotations, otherwise the \
                             connection is a hinge; trusses have no rotations.",
                        )
                        .weak(),
                    )
                    .wrap(),
                );
                ui.end_row();
            }
            Draft::FieldOutput(_, output) => {
                field_output_form(ui, output, &taken, self.index.is_none());
            }
            Draft::HistoryOutput(_, output, region) => {
                history_output_form(ui, model, output, region, &taken);
            }
            Draft::Constraint(c) => c.form(ui, model, &taken, self.index.is_none()),
            // Laid out by their own dialogs, see show.
            Draft::SurfaceInteraction(..) | Draft::Amplitude(..) => {}
            Draft::Distribution(distribution, view) => {
                name_row(ui, &mut distribution.name);
                distribution_dialog::distribution_rows(ui, distribution, view, model);
            }
            Draft::ContactPair(pair, regions) => {
                name_row(ui, &mut pair.name);
                contacts::contact_pair_form(ui, model, pair, regions);
            }
            Draft::ContactRegion(region, sides, view) => {
                ui.label("Name");
                ui.horizontal(|ui| {
                    ui.add(egui::TextEdit::singleline(&mut region.name).desired_width(170.0));
                    if ui
                        .small_button("From definition")
                        .on_hover_text("Name the region by its type and the parts of its sides.")
                        .clicked()
                    {
                        (region.side_a, region.side_b) = sides.regions();
                        region.name = region.name_from_definition(&model.mesh);
                    }
                });
                ui.end_row();
                let behavior = region.behavior;
                connections::region_form(ui, model, region, sides, view);
                // A name made from the definition follows the type.
                if region.behavior.name() != behavior.name() {
                    let new = std::mem::replace(&mut region.behavior, behavior);
                    region.set_behavior(new);
                }
            }
            Draft::Bolt(bolt, sides, actions, view) => {
                name_row(ui, &mut bolt.name);
                crate::bolts::bolt_form(ui, model, bolt, sides, actions, view);
            }
            Draft::ContactDetection(detection) => {
                name_row(ui, &mut detection.name);
                connections::detection_form(ui, model, detection, self.detection_scope.as_ref());
            }
            Draft::Connections(defaults) => connections::connections_form(ui, &model.fe, defaults),
            Draft::Tie(tie, regions) => {
                name_row(ui, &mut tie.name);
                contacts::tie_form(ui, model, tie, regions, &mut self.cyclic);
            }
            Draft::MeshSet(set, region, _) => {
                name_row(ui, &mut set.name);
                region.ui(ui, model);
            }
        }
    }

    /// Names of the other items of the same kind, which the draft's name must not repeat.
    fn taken<'a>(&self, fe: &'a FeModel) -> Vec<&'a str> {
        let mut siblings = match &self.draft {
            Draft::Material(_) => names(&fe.materials, |m| &m.name),
            Draft::Section(..) => names(&fe.sections, |s| &s.name),
            Draft::CrossSection(_) => names(&fe.cross_sections, |c| &c.name),
            Draft::Step(..) => names(&fe.steps, |s| &s.name),
            Draft::BoundaryCondition(step, ..) => {
                names(&fe.steps[*step].boundary_conditions, |b| &b.name)
            }
            Draft::Load(step, ..) => names(&fe.steps[*step].loads, |l| &l.name),
            Draft::InitialCondition(..) => names(&fe.initial_conditions, |i| &i.name),
            Draft::FieldOutput(step, _) => names(&fe.steps[*step].field_outputs, |f| &f.name),
            Draft::HistoryOutput(step, ..) => names(&fe.steps[*step].history_outputs, |h| &h.name),
            Draft::DefinedField(step, ..) => names(&fe.steps[*step].defined_fields, |f| &f.name),
            Draft::Constraint(_) => fe.constraints.iter().map(Constraint::name).collect(),
            Draft::SurfaceInteraction(..) => names(&fe.surface_interactions, |s| &s.name),
            Draft::ContactPair(..) => names(&fe.contact_pairs, |c| &c.name),
            Draft::ContactRegion(..) => names(&fe.contact_regions, |r| &r.name),
            Draft::ContactDetection(..) => names(&fe.contact_detections, |d| &d.name),
            Draft::Bolt(..) => names(&fe.bolts, |b| &b.name),
            Draft::Connections(_) => Vec::new(),
            Draft::Tie(..) => names(&fe.ties, |t| &t.name),
            Draft::NodeTie(..) => names(&fe.node_ties, |t| &t.name),
            Draft::Amplitude(..) => names(&fe.amplitudes, |a| &a.name),
            Draft::Distribution(..) => names(&fe.distributions, |d| &d.name),
            // Against the mesh's sets, see validate_set.
            Draft::MeshSet(..) => Vec::new(),
        };
        let moved = match &self.draft {
            Draft::BoundaryCondition(s, ..) | Draft::Load(s, ..) => {
                self.from_step.is_some_and(|from| from != *s)
            }
            _ => false,
        };
        if let Some(index) = self.index.filter(|&i| i < siblings.len() && !moved) {
            siblings.remove(index);
        }
        siblings
    }

    fn validate(&self, fe: &FeModel) -> Result<(), String> {
        let name = match &self.draft {
            Draft::Material(m) => &m.name,
            Draft::Section(s, _) => &s.name,
            Draft::CrossSection(c) => &c.name,
            Draft::Step(s, _) => &s.name,
            Draft::BoundaryCondition(_, b, _) => &b.name,
            Draft::Load(_, l, _) => &l.name,
            Draft::InitialCondition(c, _) => &c.name,
            Draft::FieldOutput(_, f) => &f.name,
            Draft::HistoryOutput(_, h, _) => &h.name,
            Draft::DefinedField(_, f, _) => &f.name,
            Draft::Constraint(c) => c.name(),
            Draft::SurfaceInteraction(s, _) => &s.name,
            Draft::ContactPair(c, _) => &c.name,
            Draft::ContactRegion(r, ..) => &r.name,
            Draft::ContactDetection(d) => &d.name,
            Draft::Bolt(b, ..) => &b.name,
            Draft::Connections(_) => return Ok(()),
            Draft::Tie(t, _) => &t.name,
            Draft::NodeTie(t, _) => &t.name,
            Draft::Amplitude(a, _) => &a.name,
            Draft::Distribution(d, _) => &d.name,
            Draft::MeshSet(set, _, _) => &set.name,
        };
        if name.trim().is_empty() {
            return Err("Please enter a name.".into());
        }
        let duplicate = self
            .taken(fe)
            .iter()
            .any(|other| other.eq_ignore_ascii_case(name));
        if duplicate {
            return Err(format!("The name {name} is already used."));
        }
        if let Draft::DefinedField(_, field, _) = &self.draft
            && let DefinedFieldKind::TemperatureFromFile { file, step } = &field.kind
        {
            if file.as_os_str().is_empty() {
                return Err("Choose the result file (.frd) of a heat transfer analysis.".into());
            }
            if *step == 0 {
                return Err("The step to read counts from 1.".into());
            }
        }
        if let Draft::InitialCondition(condition, _) = &self.draft
            && let Some(problem) = condition.kind.problem()
        {
            return Err(problem);
        }
        if let Draft::Section(section, _) = &self.draft {
            if !fe.materials.iter().any(|m| m.name == section.material) {
                return Err("Please select a material; create one under Materials first.".into());
            }
            match &section.kind {
                SectionKind::Solid => {}
                SectionKind::Truss { area } if !(area.is_finite() && *area > 0.0) => {
                    return Err("The cross-section area must be greater than 0.".into());
                }
                SectionKind::Truss { .. } => {}
                SectionKind::Shell { thickness, .. } | SectionKind::Membrane { thickness, .. }
                    if !(thickness.is_finite() && *thickness > 0.0) =>
                {
                    return Err(format!(
                        "The {} thickness must be greater than 0.",
                        section.kind.label().to_ascii_lowercase()
                    ));
                }
                SectionKind::Shell { .. } | SectionKind::Membrane { .. } => {}
                SectionKind::Beam(beam) => {
                    if fe.cross_section(&beam.cross_section).is_none() {
                        return Err("Please select a cross-section; create one under \
                                    Sections > Cross-Sections first."
                            .into());
                    }
                    if let BeamOrientation::Direction(n) = beam.orientation
                        && n.iter().all(|v| *v == 0.0)
                    {
                        return Err("The normal must not be the zero vector.".into());
                    }
                }
            }
        }
        match &self.draft {
            Draft::Constraint(c) => c.validate()?,
            Draft::ContactPair(pair, regions) => {
                contacts::validate_contact_pair(pair, fe)?;
                regions.validate()?;
            }
            Draft::ContactRegion(region, sides, _) => connections::validate_region(region, sides)?,
            Draft::ContactDetection(detection) => connections::validate_detection(detection)?,
            Draft::Bolt(_, sides, ..) => crate::bolts::validate_bolt(sides)?,
            Draft::Tie(tie, regions) => {
                regions.validate()?;
                if let Some(problem) = tie.cyclic.as_ref().and_then(CyclicSymmetry::problem) {
                    return Err(problem);
                }
            }
            Draft::SurfaceInteraction(interaction, _) => {
                contacts::validate_interaction(interaction)?;
            }
            Draft::Material(material) => {
                if let Some(problem) = material.creep.as_ref().and_then(|c| c.problem()) {
                    return Err(problem.into());
                }
            }
            Draft::CrossSection(cross_section) => {
                if !cross_section.profile.is_valid() {
                    return Err("The dimensions must be greater than 0; walls thinner than \
                                the profile."
                        .into());
                }
            }
            Draft::Amplitude(amplitude, _) => amplitude_dialog::validate(amplitude)?,
            Draft::Distribution(distribution, _) => distribution_dialog::validate(distribution)?,
            Draft::FieldOutput(_, output) => output.check_variables()?,
            Draft::HistoryOutput(_, output, region) => {
                validate_history_output(output, region, fe)?;
            }
            _ => {
                if self.region().is_some_and(RegionDraft::is_empty) {
                    return Err("The region is empty.".into());
                }
            }
        }
        if let Draft::BoundaryCondition(_, bc, _) = &self.draft
            && let BoundaryKind::Symmetry { normal: Some(n) } = bc.kind
            && n.iter().all(|v| *v == 0.0)
        {
            return Err("The normal must not be the zero vector.".into());
        }
        if let Draft::BoundaryCondition(_, bc, _) = &self.draft
            && let BoundaryKind::Submodel { dofs, .. } = bc.kind
        {
            let count = if fe.properties.space.is_2d() { 2 } else { 6 };
            if !dofs[..count].iter().any(|&d| d) {
                return Err("Select at least one degree of freedom.".into());
            }
        }
        if let Draft::Step(step, _) = &self.draft {
            match &step.kind {
                StepKind::Frequency(settings) => validate_frequency_step(settings)?,
                StepKind::Buckle(settings) => validate_buckle_step(settings)?,
                StepKind::HeatTransfer(settings)
                | StepKind::CoupledTempDisp(settings)
                | StepKind::UncoupledTempDisp(settings) => validate_heat_transfer_step(settings)?,
                StepKind::Sensitivity(settings) => {
                    if let Some(problem) = settings.problem() {
                        return Err(problem);
                    }
                }
                StepKind::Dynamic(settings) => {
                    if let Some(problem) = settings.problem() {
                        return Err(problem);
                    }
                }
                StepKind::ModalDynamics(settings) => {
                    if let Some(problem) = settings.problem() {
                        return Err(problem);
                    }
                }
                StepKind::SteadyStateDynamics(settings) => {
                    if let Some(problem) = settings.problem() {
                        return Err(problem);
                    }
                }
                StepKind::Visco(settings) => {
                    if settings.creep_tolerance <= 0.0 {
                        return Err("The creep strain tolerance must be positive.".into());
                    }
                }
                StepKind::Static(_) | StepKind::ComplexFrequency(_) => {}
            }
        }
        Ok(())
    }

    /// The set of an input file the dialog's set takes the place of, which leaves the mesh
    /// with OK.
    pub fn replaced_set(&self) -> Option<(SetKind, &str)> {
        match &self.draft {
            Draft::MeshSet(set, _, Some(name)) => Some((set.kind, name)),
            _ => None,
        }
    }

    /// The name of a set must be new among the mesh's sets of its kind, and an element set
    /// must not take the name of a part, whose elements the input file gathers under it.
    /// Cut faces of a cyclic symmetry tie that do not divide the full circle.
    fn validate_sectors(&mut self, model: &Model) -> Result<(), String> {
        let Draft::Tie(tie, regions) = &self.draft else {
            return Ok(());
        };
        // The form shows why under Sectors.
        match self.cyclic.get(model, tie, regions) {
            Some(Err(_)) => Err("The cyclic symmetry has no valid number of sectors, see \
                                 Sectors."
                .into()),
            _ => Ok(()),
        }
    }

    /// The whole part a cyclic symmetry tie dialog's sector stands for, previewed in the 3D
    /// view.
    pub fn cyclic_preview(&mut self, model: &Model) -> Option<crate::model::CyclicPreview> {
        let Draft::Tie(tie, regions) = &self.draft else {
            return None;
        };
        self.cyclic.preview(model, tie, regions)
    }

    fn validate_set(&self, model: &Model) -> Result<(), String> {
        let Draft::MeshSet(set, _, replaces) = &self.draft else {
            return Ok(());
        };
        // The set's own name, of the user-defined set or of the set of an input file it
        // replaces, is free.
        let own = (self.index.and_then(|i| model.fe.mesh_sets.get(i)))
            .map(|o| o.name.as_str())
            .or(replaces.as_deref());
        let mut taken = set_names(set.kind, &model.fe, &model.mesh);
        taken.retain(|n| Some(*n) != own);
        if set.kind == SetKind::Elements {
            taken.extend(model.mesh.parts.iter().map(|p| p.name.as_str()));
        }
        if taken.iter().any(|n| n.eq_ignore_ascii_case(&set.name)) {
            return Err(format!("The name {} is already used.", set.name));
        }
        if !set
            .name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || "_-.".contains(c))
        {
            return Err("Use letters, digits, '_', '-' and '.' only in the name.".into());
        }
        Ok(())
    }

    /// The index a contact detection of this dialog has once applied to `fe`, to run it.
    pub fn detection(&self, fe: &FeModel) -> Option<usize> {
        match self.draft {
            Draft::ContactDetection(_) => Some(self.index.unwrap_or(fe.contact_detections.len())),
            _ => None,
        }
    }

    /// The tree item a new constraint, surface interaction or contact pair gets once
    /// applied to `fe`, to show it in the tree.
    pub fn new_interaction_item(&self, fe: &FeModel) -> Option<TreeItem> {
        if self.index.is_some() {
            return None;
        }
        match self.draft {
            Draft::Constraint(_) => Some(TreeItem::Constraint(fe.constraints.len())),
            Draft::SurfaceInteraction(..) => {
                Some(TreeItem::SurfaceInteraction(fe.surface_interactions.len()))
            }
            Draft::ContactPair(..) => Some(TreeItem::ContactPair(fe.contact_pairs.len())),
            Draft::ContactRegion(..) => Some(TreeItem::ContactRegion(fe.contact_regions.len())),
            Draft::ContactDetection(..) => {
                Some(TreeItem::ContactDetection(fe.contact_detections.len()))
            }
            Draft::Bolt(..) => Some(TreeItem::Bolt(fe.bolts.len())),
            Draft::Tie(..) => Some(TreeItem::Tie(fe.ties.len())),
            Draft::NodeTie(..) => Some(TreeItem::NodeTie(fe.node_ties.len())),
            Draft::Amplitude(..) => Some(TreeItem::Amplitude(fe.amplitudes.len())),
            Draft::Distribution(..) => Some(TreeItem::Distribution(fe.distributions.len())),
            Draft::MeshSet(ref set, ..) => Some(match set.kind {
                SetKind::Nodes => TreeItem::NodeSet(set.name.clone()),
                SetKind::Elements => TreeItem::ElementSet(set.name.clone()),
                SetKind::Surface => TreeItem::Surface(set.name.clone()),
            }),
            _ => None,
        }
    }

    pub fn apply(mut self, fe: &mut FeModel) {
        // The exposed parameters follow the item, under its new name.
        let exposed = (self.expose.take()).zip(self.draft_item(fe).map(|(item, _)| item));
        self.apply_draft(fe);
        if let Some((scope, item)) = exposed {
            scope.apply(fe, &item);
        }
    }

    fn apply_draft(self, fe: &mut FeModel) {
        fn put<T>(items: &mut Vec<T>, index: Option<usize>, item: T) {
            match index.and_then(|i| items.get_mut(i)) {
                Some(slot) => *slot = item,
                None => items.push(item),
            }
        }
        /// Takes an item out of the step it was in when the dialog moved it to `to`; the
        /// item is then added to `to` as a new one. Returns its index there and the item
        /// taken out.
        fn move_out<T>(
            fe: &mut FeModel,
            from: Option<usize>,
            to: usize,
            index: Option<usize>,
            list: impl Fn(&mut Step) -> &mut Vec<T>,
        ) -> (Option<usize>, Option<T>) {
            match (from, index) {
                (Some(from), Some(i)) if from != to => {
                    let items = fe.steps.get_mut(from).map(list);
                    let taken = items.filter(|l| i < l.len()).map(|l| l.remove(i));
                    (None, taken)
                }
                _ => (index, None),
            }
        }
        let index = self.index;
        let from_step = self.from_step;
        match self.draft {
            Draft::Material(material) => {
                // Sections follow a renamed material.
                if let Some(old) = index.and_then(|i| fe.materials.get(i)) {
                    let old = old.name.clone();
                    for section in fe.sections.iter_mut().filter(|s| s.material == old) {
                        section.material = material.name.clone();
                    }
                }
                put(&mut fe.materials, index, material);
            }
            Draft::Section(mut section, region) => {
                section.region = region.region();
                put(&mut fe.sections, index, section);
            }
            Draft::CrossSection(cross_section) => {
                // Beam sections follow a renamed cross-section.
                if let Some(old) = index.and_then(|i| fe.cross_sections.get(i)) {
                    let old = old.name.clone();
                    for section in &mut fe.sections {
                        if let SectionKind::Beam(beam) = &mut section.kind
                            && beam.cross_section == old
                        {
                            beam.cross_section = cross_section.name.clone();
                        }
                    }
                }
                put(&mut fe.cross_sections, index, cross_section);
            }
            Draft::Step(mut step, region) => {
                if let Some(design) = step.kind.design_region_mut() {
                    *design = region.region();
                }
                match index.and_then(|i| fe.steps.get_mut(i)) {
                    // The dialog edits the settings; the step's items stay.
                    Some(existing) => {
                        existing.name = step.name;
                        existing.kind = step.kind;
                        existing.contact_changes = step.contact_changes;
                    }
                    None => {
                        copy_items_of_last_step(fe, &mut step);
                        fe.steps.push(step);
                    }
                }
            }
            // Switching an item on or off while its dialog is open is kept.
            Draft::BoundaryCondition(s, mut bc, region) => {
                bc.region = region.region();
                if !bc.kind.takes_amplitude() {
                    bc.amplitude = None;
                }
                let (index, moved) = move_out(fe, from_step, s, index, |step| {
                    &mut step.boundary_conditions
                });
                if let Some(moved) = moved {
                    bc.active = moved.active;
                }
                let list = &mut fe.steps[s].boundary_conditions;
                let mut renamed = None;
                if let Some(existing) = index.and_then(|i| list.get(i)) {
                    bc.active = existing.active;
                    renamed = Some(existing.name.clone()).filter(|old| *old != bc.name);
                }
                let remote = matches!(bc.kind, BoundaryKind::Displacement(_));
                let new = bc.name.clone();
                put(list, index, bc);
                // History outputs of a remote displacement's point follow a renamed one.
                if let Some(old) = renamed.filter(|_| remote) {
                    fe.rename_remote_point(&old, &new);
                }
            }
            Draft::Load(s, mut load, region) => {
                load.region = region.region();
                if load.kind.factor_amplitude_label().is_none() {
                    load.factor_amplitude = None;
                }
                let (index, moved) = move_out(fe, from_step, s, index, |step| &mut step.loads);
                if let Some(moved) = moved {
                    load.active = moved.active;
                }
                let list = &mut fe.steps[s].loads;
                if let Some(existing) = index.and_then(|i| list.get(i)) {
                    load.active = existing.active;
                }
                put(list, index, load);
            }
            Draft::InitialCondition(mut condition, region) => {
                condition.region = region.region();
                if let Some(existing) = index.and_then(|i| fe.initial_conditions.get(i)) {
                    condition.active = existing.active;
                }
                put(&mut fe.initial_conditions, index, condition);
            }
            Draft::FieldOutput(s, mut output) => {
                output.normalize_variables();
                let list = &mut fe.steps[s].field_outputs;
                if let Some(existing) = index.and_then(|i| list.get(i)) {
                    output.active = existing.active;
                }
                put(list, index, output);
            }
            Draft::DefinedField(s, mut field, region) => {
                if field.kind.takes_region() {
                    field.region = region.region();
                } else {
                    field.region = Region::Nodes(Vec::new());
                }
                if !field.kind.takes_amplitude() {
                    field.amplitude = None;
                }
                let list = &mut fe.steps[s].defined_fields;
                if let Some(existing) = index.and_then(|i| list.get(i)) {
                    field.active = existing.active;
                }
                put(list, index, field);
            }
            Draft::ContactRegion(mut region, sides, _) => {
                (region.side_a, region.side_b) = sides.regions();
                if let Some(existing) = index.and_then(|i| fe.contact_regions.get(i)) {
                    region.active = existing.active;
                    // A detected region the user changed is kept when detecting again.
                    region.edited |= region.detected_by.is_some() && region != *existing;
                    let old = existing.name.clone();
                    fe.rename_contact_pair(&old, &region.name);
                }
                put(&mut fe.contact_regions, index, region);
            }
            Draft::ContactDetection(detection) => {
                if let Some(existing) = index.and_then(|i| fe.contact_detections.get(i)) {
                    let old = existing.name.clone();
                    for region in &mut fe.contact_regions {
                        if region.detected_by.as_deref() == Some(old.as_str()) {
                            region.detected_by = Some(detection.name.clone());
                        }
                    }
                }
                put(&mut fe.contact_detections, index, detection);
            }
            Draft::Bolt(mut bolt, sides, actions, _) => {
                (bolt.side_a, bolt.side_b) = sides.regions();
                if let Some(existing) = index.and_then(|i| fe.bolts.get(i)) {
                    bolt.active = existing.active;
                    let old = existing.name.clone();
                    fe.remove_bolt_changes(&old);
                }
                crate::bolts::set_step_actions(fe, &bolt.name, &actions);
                put(&mut fe.bolts, index, bolt);
            }
            Draft::Connections(defaults) => fe.contact_defaults = defaults,
            Draft::Tie(mut tie, regions) => {
                (tie.master, tie.slave) = regions.regions();
                if let Some(existing) = index.and_then(|i| fe.ties.get(i)) {
                    tie.active = existing.active;
                }
                put(&mut fe.ties, index, tie);
            }
            Draft::NodeTie(mut tie, region) => {
                tie.region = region.region();
                if let Some(existing) = index.and_then(|i| fe.node_ties.get(i)) {
                    tie.active = existing.active;
                }
                put(&mut fe.node_ties, index, tie);
            }
            Draft::HistoryOutput(s, mut output, region) => {
                if let Some(target) = output.kind.region_mut() {
                    *target = region.region();
                }
                output.normalize_variables();
                let list = &mut fe.steps[s].history_outputs;
                if let Some(existing) = index.and_then(|i| list.get(i)) {
                    output.active = existing.active;
                }
                put(list, index, output);
            }
            Draft::Constraint(draft) => {
                let mut constraint = draft.finish();
                if let Some(existing) = index.and_then(|i| fe.constraints.get(i)) {
                    *constraint.active_mut() = existing.active();
                }
                put(&mut fe.constraints, index, constraint);
            }
            Draft::SurfaceInteraction(interaction, _) => {
                // Contact pairs follow a renamed interaction.
                if let Some(old) = index.and_then(|i| fe.surface_interactions.get(i)) {
                    let old = old.name.clone();
                    for pair in fe.contact_pairs.iter_mut().filter(|c| c.interaction == old) {
                        pair.interaction = interaction.name.clone();
                    }
                }
                put(&mut fe.surface_interactions, index, interaction);
            }
            Draft::ContactPair(mut pair, regions) => {
                (pair.master, pair.slave) = regions.regions();
                if let Some(existing) = index.and_then(|i| fe.contact_pairs.get(i)) {
                    pair.active = existing.active;
                    // Contact history outputs follow a renamed pair.
                    let old = existing.name.clone();
                    fe.rename_contact_pair(&old, &pair.name);
                }
                put(&mut fe.contact_pairs, index, pair);
            }
            Draft::Amplitude(amplitude, _) => {
                // Boundary conditions and loads follow a renamed amplitude.
                if let Some(old) = index.and_then(|i| fe.amplitudes.get(i)) {
                    let old = old.name.clone();
                    fe.rename_amplitude(&old, &amplitude.name);
                }
                put(&mut fe.amplitudes, index, amplitude);
            }
            Draft::Distribution(distribution, _) => {
                // Pressures and temperatures follow a renamed distribution.
                if let Some(old) = index.and_then(|i| fe.distributions.get(i)) {
                    let old = old.name.clone();
                    fe.rename_distribution(&old, &distribution.name);
                }
                put(&mut fe.distributions, index, distribution);
            }
            Draft::MeshSet(mut set, region, replaces) => {
                set.region = region.region();
                // Items on the set follow a new name.
                let old = (index.and_then(|i| fe.mesh_sets.get(i)))
                    .map(|o| o.name.clone())
                    .or(replaces);
                if let Some(old) = old {
                    fe.rename_set(set.kind, &old, &set.name);
                }
                put(&mut fe.mesh_sets, index, set);
            }
        }
    }
}

/// Names of the mesh's sets of a kind together with the user-defined ones, which a
/// selection that finds nothing leaves out of the mesh.
fn set_names<'a>(kind: SetKind, fe: &'a FeModel, mesh: &'a FeMesh) -> Vec<&'a str> {
    let mut names = kind.names(mesh);
    for set in fe.mesh_sets.iter().filter(|s| s.kind == kind) {
        if !names.contains(&set.name.as_str()) {
            names.push(&set.name);
        }
    }
    names
}

fn set_sources(kind: SetKind) -> &'static [Source] {
    match kind {
        SetKind::Nodes | SetKind::Elements => SET_SOURCES,
        SetKind::Surface => SURFACE_SOURCES,
    }
}

/// Node sets pick nodes; element sets and surfaces element faces, or edges in 2D models.
fn set_target(kind: SetKind, fe: &FeModel) -> Target {
    match kind {
        SetKind::Nodes => Target::Nodes,
        SetKind::Elements | SetKind::Surface => face_target(fe),
    }
}

/// Removes an item; returns false for items that cannot be deleted.
pub fn delete(fe: &mut FeModel, item: &TreeItem) -> bool {
    fn remove<T>(items: &mut Vec<T>, index: usize) -> bool {
        (index < items.len()).then(|| items.remove(index)).is_some()
    }
    match *item {
        TreeItem::Material(i) => remove(&mut fe.materials, i),
        TreeItem::Section(i) => remove(&mut fe.sections, i),
        TreeItem::CrossSection(i) => remove(&mut fe.cross_sections, i),
        TreeItem::Step(i) => remove(&mut fe.steps, i),
        TreeItem::BoundaryCondition(s, i) => fe
            .steps
            .get_mut(s)
            .is_some_and(|st| remove(&mut st.boundary_conditions, i)),
        TreeItem::Load(s, i) => fe
            .steps
            .get_mut(s)
            .is_some_and(|st| remove(&mut st.loads, i)),
        TreeItem::FieldOutput(s, i) => fe
            .steps
            .get_mut(s)
            .is_some_and(|st| remove(&mut st.field_outputs, i)),
        TreeItem::HistoryOutput(s, i) => fe
            .steps
            .get_mut(s)
            .is_some_and(|st| remove(&mut st.history_outputs, i)),
        TreeItem::DefinedField(s, i) => fe
            .steps
            .get_mut(s)
            .is_some_and(|st| remove(&mut st.defined_fields, i)),
        TreeItem::InitialCondition(i) => remove(&mut fe.initial_conditions, i),
        TreeItem::Constraint(i) => remove(&mut fe.constraints, i),
        TreeItem::SurfaceInteraction(i) => remove(&mut fe.surface_interactions, i),
        TreeItem::ContactPair(i) | TreeItem::ContactRegion(i) => {
            let name = match item {
                TreeItem::ContactPair(_) => fe.contact_pairs.get(i).map(|p| p.name.clone()),
                _ => fe.contact_regions.get(i).map(|r| r.name.clone()),
            };
            let Some(name) = name else {
                return false;
            };
            fe.remove_contact_changes(&name);
            match item {
                TreeItem::ContactPair(_) => remove(&mut fe.contact_pairs, i),
                _ => remove(&mut fe.contact_regions, i),
            }
        }
        TreeItem::Bolt(i) => {
            let Some(name) = fe.bolts.get(i).map(|b| b.name.clone()) else {
                return false;
            };
            fe.remove_bolt_changes(&name);
            remove(&mut fe.bolts, i)
        }
        TreeItem::ContactDetection(i) => {
            let Some(name) = fe.contact_detections.get(i).map(|d| d.name.clone()) else {
                return false;
            };
            fe.contact_detections.remove(i);
            // Its regions go with it, except those the user changed, which stay as regions
            // of their own.
            fe.contact_regions.retain_mut(|r| {
                if r.detected_by.as_deref() != Some(name.as_str()) {
                    return true;
                }
                r.detected_by = None;
                std::mem::take(&mut r.edited)
            });
            true
        }
        TreeItem::Tie(i) => remove(&mut fe.ties, i),
        TreeItem::NodeTie(i) => remove(&mut fe.node_ties, i),
        TreeItem::Amplitude(i) => remove(&mut fe.amplitudes, i),
        TreeItem::Distribution(i) => remove(&mut fe.distributions, i),
        _ => false,
    }
}

/// Activates a deactivated step, boundary condition or load, or deactivates an active one,
/// with Activate and Deactivate; returns false for other items.
pub fn toggle_active(fe: &mut FeModel, item: &TreeItem) -> bool {
    active_mut(fe, item).map(|a| *a = !*a).is_some()
}

/// Activates or deactivates an item; returns false for items that cannot be.
pub fn set_active(fe: &mut FeModel, item: &TreeItem, active: bool) -> bool {
    active_mut(fe, item).map(|a| *a = active).is_some()
}

/// Whether a step, boundary condition, load and the like is active; `None` for items that
/// cannot be deactivated.
pub fn is_active(fe: &mut FeModel, item: &TreeItem) -> Option<bool> {
    active_mut(fe, item).map(|a| *a)
}

fn active_mut<'a>(fe: &'a mut FeModel, item: &TreeItem) -> Option<&'a mut bool> {
    match *item {
        TreeItem::Step(s) => fe.steps.get_mut(s).map(|st| &mut st.active),
        TreeItem::BoundaryCondition(s, i) => (fe.steps.get_mut(s))
            .and_then(|st| st.boundary_conditions.get_mut(i))
            .map(|b| &mut b.active),
        TreeItem::Load(s, i) => (fe.steps.get_mut(s))
            .and_then(|st| st.loads.get_mut(i))
            .map(|l| &mut l.active),
        TreeItem::FieldOutput(s, i) => (fe.steps.get_mut(s))
            .and_then(|st| st.field_outputs.get_mut(i))
            .map(|f| &mut f.active),
        TreeItem::HistoryOutput(s, i) => (fe.steps.get_mut(s))
            .and_then(|st| st.history_outputs.get_mut(i))
            .map(|h| &mut h.active),
        TreeItem::DefinedField(s, i) => (fe.steps.get_mut(s))
            .and_then(|st| st.defined_fields.get_mut(i))
            .map(|f| &mut f.active),
        TreeItem::Constraint(i) => fe.constraints.get_mut(i).map(Constraint::active_mut),
        TreeItem::ContactPair(i) => fe.contact_pairs.get_mut(i).map(|c| &mut c.active),
        TreeItem::ContactRegion(i) => fe.contact_regions.get_mut(i).map(|r| &mut r.active),
        TreeItem::Bolt(i) => fe.bolts.get_mut(i).map(|b| &mut b.active),
        TreeItem::Tie(i) => fe.ties.get_mut(i).map(|t| &mut t.active),
        TreeItem::NodeTie(i) => fe.node_ties.get_mut(i).map(|t| &mut t.active),
        TreeItem::InitialCondition(i) => fe.initial_conditions.get_mut(i).map(|c| &mut c.active),
        _ => None,
    }
}

/// Shows or hides the symbol of a constraint, boundary condition or load in the 3D view;
/// returns false for other items.
pub fn set_visible(fe: &mut FeModel, item: &TreeItem, visible: bool) -> bool {
    let shown = match *item {
        TreeItem::BoundaryCondition(s, i) => (fe.steps.get_mut(s))
            .and_then(|st| st.boundary_conditions.get_mut(i))
            .map(|b| &mut b.visible),
        TreeItem::Load(s, i) => (fe.steps.get_mut(s))
            .and_then(|st| st.loads.get_mut(i))
            .map(|l| &mut l.visible),
        TreeItem::Constraint(i) => fe.constraints.get_mut(i).and_then(Constraint::visible_mut),
        _ => None,
    };
    shown.map(|v| *v = visible).is_some()
}

/// Moves step `s` one place up or down, like reordering in the step list; returns the
/// new index. Loads, boundary conditions and outputs belong to their step and move with it.
pub fn move_step(fe: &mut FeModel, s: usize, up: bool) -> Option<usize> {
    let to = if up { s.checked_sub(1)? } else { s + 1 };
    if s >= fe.steps.len() || to >= fe.steps.len() {
        return None;
    }
    fe.steps.swap(s, to);
    Some(to)
}

/// Swaps master and slave of a tie, spring connection or contact pair; a swapped name
/// `<slave>_to_<master>` that is taken gets the next free number.
pub fn swap_master_slave(fe: &mut FeModel, item: &TreeItem) -> bool {
    match *item {
        TreeItem::Constraint(i) => {
            let Some(constraint) = fe.constraints.get_mut(i) else {
                return false;
            };
            let old = constraint.name().to_owned();
            if !constraint.swap_master_slave() {
                return false;
            }
            let name = constraint.name().to_owned();
            let others = (fe.constraints.iter().enumerate())
                .filter(|&(j, _)| j != i)
                .map(|(_, c)| c.name());
            if name != old && others.clone().any(|n| n.eq_ignore_ascii_case(&name)) {
                let free = next_name(&name, others);
                *fe.constraints[i].name_mut() = free;
            }
            true
        }
        TreeItem::ContactPair(i) => {
            let Some(pair) = fe.contact_pairs.get_mut(i) else {
                return false;
            };
            let old = pair.name.clone();
            pair.swap_master_slave();
            let name = pair.name.clone();
            let others = (fe.contact_pairs.iter().enumerate())
                .filter(|&(j, _)| j != i)
                .map(|(_, c)| c.name.as_str());
            if name != old && others.clone().any(|n| n.eq_ignore_ascii_case(&name)) {
                let free = next_name(&name, others);
                fe.contact_pairs[i].name = free;
            }
            let new = fe.contact_pairs[i].name.clone();
            fe.rename_contact_pair(&old, &new);
            true
        }
        TreeItem::ContactRegion(i) => fe.contact_regions.get_mut(i).map(|r| r.flip()).is_some(),
        TreeItem::Tie(i) => {
            let Some(tie) = fe.ties.get_mut(i) else {
                return false;
            };
            let old = tie.name.clone();
            tie.swap_master_slave();
            let name = tie.name.clone();
            let others = (fe.ties.iter().enumerate())
                .filter(|&(j, _)| j != i)
                .map(|(_, t)| t.name.as_str());
            if name != old && others.clone().any(|n| n.eq_ignore_ascii_case(&name)) {
                let free = next_name(&name, others);
                fe.ties[i].name = free;
            }
            true
        }
        _ => false,
    }
}

/// How an item selected in the tree shows in the 3D view: its region, or master and slave.
pub fn item_highlight(model: &Model, item: &TreeItem) -> Highlight {
    let fe = &model.fe;
    let master_slave = match *item {
        TreeItem::Constraint(i) => fe.constraints.get(i).and_then(Constraint::master_slave),
        TreeItem::ContactPair(i) => fe.contact_pairs.get(i).map(|c| [&c.master, &c.slave]),
        TreeItem::Tie(i) => fe.ties.get(i).map(|t| [&t.master, &t.slave]),
        TreeItem::ContactRegion(i) => fe.contact_regions.get(i).map(|r| [&r.side_a, &r.side_b]),
        TreeItem::Bolt(i) => fe.bolts.get(i).map(|b| [&b.side_a, &b.side_b]),
        TreeItem::HistoryOutput(s, i) => {
            let output = fe.steps.get(s).and_then(|st| st.history_outputs.get(i));
            match output.map(|h| &h.kind) {
                Some(HistoryKind::Contact { pair }) => (fe.contact_pairs.iter())
                    .find(|c| c.name == *pair)
                    .map(|c| [&c.master, &c.slave])
                    .or_else(|| {
                        (fe.contact_regions.iter())
                            .find(|r| r.name == *pair)
                            .map(|r| [&r.side_a, &r.side_b])
                    }),
                _ => None,
            }
        }
        _ => None,
    };
    if let Some([master, slave]) = master_slave {
        return contacts::master_slave_highlight(model, master, slave);
    }
    if let (Some(kind), Some(name)) = (crate::tree::set_kind(item), crate::tree::set_name(item)) {
        return region_highlight(model, &kind.reference(name));
    }
    item_region(fe, item)
        .map(|region| region_highlight(model, region))
        .unwrap_or_default()
}

/// The contact pairs, ties, constraints, boundary conditions and loads that act on any of
/// `parts`: those with a region on them. A click on a part in the 3D view marks them in the
/// tree.
pub fn acting_on_parts(model: &Model, parts: &BTreeSet<usize>) -> BTreeSet<TreeItem> {
    let mesh = &model.mesh;
    let names: HashSet<&str> = (parts.iter())
        .filter_map(|&p| mesh.parts.get(p))
        .map(|p| p.name.as_str())
        .collect();
    let elements: HashSet<ElementId> = (parts.iter())
        .filter_map(|&p| mesh.parts.get(p))
        .flat_map(|p| p.elements.iter().copied())
        .collect();
    let fe = &model.fe;
    let on_parts = |region: &Region| match region {
        Region::Parts(regions) => regions.iter().any(|n| names.contains(n.as_str())),
        Region::RemotePoint(_) => false,
        _ => {
            let mut ids = region.elements(mesh);
            ids.extend(region.faces(mesh).into_iter().map(|(e, _)| e));
            if ids.is_empty() {
                (region.nodes(mesh).into_iter())
                    .any(|n| model.node_part(n).is_some_and(|p| parts.contains(&p)))
            } else {
                ids.iter().any(|e| elements.contains(e))
            }
        }
    };
    // A load on a reference point acts on the rigid bodies the point drives.
    let driven = |region: &Region| match region.reference_point() {
        Some(point) => fe.constraints.iter().any(|c| match c {
            Constraint::RigidBody(body) => body.reference_point == point && on_parts(&body.region),
            _ => false,
        }),
        None => on_parts(region),
    };
    let mut acting = BTreeSet::new();
    for (i, c) in fe.constraints.iter().enumerate() {
        if c.regions().into_iter().any(on_parts) {
            acting.insert(TreeItem::Constraint(i));
        }
    }
    for (i, c) in fe.contact_pairs.iter().enumerate() {
        if on_parts(&c.master) || on_parts(&c.slave) {
            acting.insert(TreeItem::ContactPair(i));
        }
    }
    for (i, t) in fe.ties.iter().enumerate() {
        if on_parts(&t.master) || on_parts(&t.slave) {
            acting.insert(TreeItem::Tie(i));
        }
    }
    for (i, r) in fe.contact_regions.iter().enumerate() {
        if on_parts(&r.side_a) || on_parts(&r.side_b) {
            acting.insert(TreeItem::ContactRegion(i));
        }
    }
    for (i, t) in fe.node_ties.iter().enumerate() {
        if on_parts(&t.region) {
            acting.insert(TreeItem::NodeTie(i));
        }
    }
    for (i, b) in fe.bolts.iter().enumerate() {
        if on_parts(&b.side_a) || on_parts(&b.side_b) {
            acting.insert(TreeItem::Bolt(i));
        }
    }
    for (s, step) in fe.steps.iter().enumerate() {
        for (i, b) in step.boundary_conditions.iter().enumerate() {
            if driven(&b.region) {
                acting.insert(TreeItem::BoundaryCondition(s, i));
            }
        }
        for (i, l) in step.loads.iter().enumerate() {
            if driven(&l.region) {
                acting.insert(TreeItem::Load(s, i));
            }
        }
    }
    acting
}

/// Region of an item, for highlighting it when it is selected in the tree.
pub fn item_region<'a>(fe: &'a FeModel, item: &TreeItem) -> Option<&'a Region> {
    match *item {
        TreeItem::Section(i) => fe.sections.get(i).map(|s| &s.region),
        TreeItem::BoundaryCondition(s, i) => fe
            .steps
            .get(s)?
            .boundary_conditions
            .get(i)
            .map(|b| &b.region),
        TreeItem::Load(s, i) => fe.steps.get(s)?.loads.get(i).map(|l| &l.region),
        TreeItem::HistoryOutput(s, i) => fe.steps.get(s)?.history_outputs.get(i)?.kind.region(),
        TreeItem::DefinedField(s, i) => (fe.steps.get(s)?.defined_fields.get(i))
            .filter(|f| f.kind.takes_region())
            .map(|f| &f.region),
        TreeItem::InitialCondition(i) => fe.initial_conditions.get(i).map(|c| &c.region),
        TreeItem::Constraint(i) => fe.constraints.get(i)?.regions().first().copied(),
        TreeItem::NodeTie(i) => fe.node_ties.get(i).map(|t| &t.region),
        _ => None,
    }
}

/// What a history output picks its region with.
fn history_picks(kind: &HistoryKind, fe: &FeModel) -> (&'static [Source], Target) {
    match kind {
        HistoryKind::Element { .. } => (SOLID_SOURCES, face_target(fe)),
        _ => (HISTORY_NODE_SOURCES, Target::Nodes),
    }
}

/// The region draft of a history output: nodes for nodal values, elements for element
/// values.
fn history_region(kind: &HistoryKind, fe: &FeModel, mesh: &FeMesh) -> RegionDraft {
    let (sources, target) = history_picks(kind, fe);
    match kind.region() {
        Some(region) => RegionDraft::from_region(region, sources, target, mesh),
        None => RegionDraft::new(sources, target),
    }
}

/// The history output dialog: the kind, the variables, the totals and the region or
/// contact pair.
fn history_output_form(
    ui: &mut Ui,
    model: &Model,
    output: &mut HistoryOutput,
    region: &mut RegionDraft,
    taken: &[&str],
) {
    name_row(ui, &mut output.name);
    ui.label("Type");
    ui.horizontal(|ui| {
        let current = output.kind.prefix();
        let pair = (model.fe.contact_pairs.first()).map_or(String::new(), |c| c.name.clone());
        let kinds = [
            HistoryKind::Node {
                region: Region::Nodes(Vec::new()),
            },
            HistoryKind::Element {
                region: Region::Parts(Vec::new()),
            },
            HistoryKind::Contact { pair },
        ];
        for kind in kinds {
            let selected = kind.prefix() == current;
            if ui.radio(selected, kind.label()).clicked() && !selected {
                let (sources, target) = history_picks(&kind, &model.fe);
                let (kept, dropped) = region.converted(sources, target, &model.mesh);
                *region = kept;
                dropped_note(ui, dropped);
                let defaults = match &kind {
                    HistoryKind::Node { region } => HistoryOutput::node("", region.clone()),
                    HistoryKind::Element { region } => HistoryOutput::element("", region.clone()),
                    HistoryKind::Contact { pair } => HistoryOutput::contact("", pair.clone()),
                };
                output.variables = defaults.variables;
                rename_default(&mut output.name, current, kind.prefix(), taken);
                output.kind = kind;
            }
        }
    });
    ui.end_row();
    show_dropped_note(ui);
    ui.label("Variables");
    let choices = output.kind.choices();
    // Fixed rows of five in a nested grid; a wrapped layout inside the property grid
    // overlapped the row below it.
    let mut changed = false;
    egui::Grid::new("history variables")
        .spacing([10.0, 2.0])
        .show(ui, |ui| {
            for row in choices.chunks(5) {
                for &variable in row {
                    let mut on = output.variables.iter().any(|v| v == variable);
                    let checkbox = ui
                        .checkbox(&mut on, variable)
                        .on_hover_text(plx_model::output_variable_description(variable));
                    if checkbox.changed() {
                        changed = true;
                        output.variables.retain(|v| v != variable);
                        if on {
                            output.variables.push(variable.to_string());
                        }
                    }
                }
                ui.end_row();
            }
        });
    if changed {
        output.normalize_variables();
    }
    ui.end_row();
    ui.label("Totals");
    egui::ComboBox::from_id_salt("history totals")
        .selected_text(output.totals.label())
        .width(200.0)
        .show_ui(ui, |ui| {
            for totals in plx_model::Totals::ALL {
                ui.selectable_value(&mut output.totals, totals, totals.label());
            }
        });
    ui.end_row();
    match &mut output.kind {
        HistoryKind::Contact { pair } => {
            ui.label("Contact Pair");
            egui::ComboBox::from_id_salt("history contact pair")
                .selected_text(pair.as_str())
                .width(200.0)
                .show_ui(ui, |ui| {
                    for name in model.fe.contact_pair_names() {
                        ui.selectable_value(pair, name.to_owned(), name);
                    }
                });
            ui.end_row();
            ui.label("");
            ui.weak(
                "The solver writes the values of all contact elements;
                 the pair defines the surfaces of the contact forces CF.",
            );
            ui.end_row();
        }
        _ => {
            region.ui(ui, model);
            if let Some(hint) = point_hint(&region.region(), &model.fe, &model.mesh) {
                ui.label("");
                ui.weak(hint);
                ui.end_row();
            }
        }
    }
}

/// What a nodal history output of a point prints besides forces and displacements.
fn point_hint(region: &Region, fe: &FeModel, mesh: &FeMesh) -> Option<&'static str> {
    let behavior = match region {
        Region::ReferencePoint(_) => RemoteBehavior::Rigid,
        Region::RemotePoint(name) => match fe.remote_displacement(mesh, name) {
            Some((s, bc, _)) => {
                let i =
                    (fe.steps[s].boundary_conditions.iter()).position(|b| std::ptr::eq(b, bc))?;
                match fe.formulation(mesh, s, i)? {
                    Formulation::Point(behavior) => behavior,
                    Formulation::Nodes => return None,
                }
            }
            None => return None,
        },
        _ => return None,
    };
    Some(match behavior {
        RemoteBehavior::Rigid => {
            "RF and U of the point include its moments (RM) and rotations (UR), \
             V its angular velocities."
        }
        RemoteBehavior::Deformable => {
            "U includes the rotations (UR) of the point. The solver has no reaction forces or \
             moments at a deformable point; use the Rigid behavior for them."
        }
    })
}

/// The field output dialog: the kind, chosen only for a new output, the variables
/// and the options of `*NODE FILE`, `*EL FILE` and `*CONTACT FILE`.
fn field_output_form(ui: &mut Ui, output: &mut FieldOutput, taken: &[&str], new: bool) {
    name_row(ui, &mut output.name);
    ui.label("Type");
    ui.horizontal(|ui| {
        for kind in OutputKind::ALL {
            let selected = kind == output.kind;
            let radio = ui.add_enabled(
                new || selected,
                egui::RadioButton::new(selected, kind.label()),
            );
            if radio.clicked() && !selected {
                rename_default(&mut output.name, output.kind.prefix(), kind.prefix(), taken);
                output.variables = (kind.default_variables().iter())
                    .map(|v| v.to_string())
                    .collect();
                output.kind = kind;
            }
        }
    });
    ui.end_row();
    ui.label("Variables");
    // Fixed rows: wrapping inside the grid overlaps the next row.
    ui.vertical(|ui| {
        for row in output.kind.choices().chunks(7) {
            ui.horizontal(|ui| {
                for &variable in row {
                    let mut on = output.variables.iter().any(|v| v == variable);
                    let checkbox = ui
                        .checkbox(&mut on, variable)
                        .on_hover_text(plx_model::output_variable_description(variable));
                    if checkbox.changed() {
                        output.variables.retain(|v| v != variable);
                        if on {
                            output.variables.push(variable.to_string());
                        }
                        output.normalize_variables();
                    }
                }
            });
        }
    });
    ui.end_row();
    let on_off = |ui: &mut Ui, value: &mut bool, id: &str, labels: [&str; 2], hint: &str| {
        egui::ComboBox::from_id_salt(id)
            .selected_text(if *value { labels[0] } else { labels[1] })
            .width(200.0)
            .show_ui(ui, |ui| {
                ui.selectable_value(value, true, labels[0]);
                ui.selectable_value(value, false, labels[1]);
            })
            .response
            .on_hover_text(hint);
        ui.end_row();
    };
    ui.label("Last iterations");
    on_off(
        ui,
        &mut output.last_iterations,
        "field last iterations",
        ["On", "Off"],
        "Writes every iteration of the last increment; useful for finding out why a \
         solution diverges.",
    );
    ui.label("Contact elements");
    on_off(
        ui,
        &mut output.contact_elements,
        "field contact elements",
        ["On", "Off"],
        "Writes the contact elements into a file with the .cel extension.",
    );
    if output.kind == OutputKind::Element {
        ui.label("Output (2D/3D)");
        egui::ComboBox::from_id_salt("field output dimension")
            .selected_text(output.dimension.label())
            .width(200.0)
            .show_ui(ui, |ui| {
                for dimension in OutputDimension::ALL {
                    ui.selectable_value(&mut output.dimension, dimension, dimension.label());
                }
            })
            .response
            .on_hover_text(
                "With 3D, 1D and 2D elements are written in their expanded 3D form. Default \
                 writes shell and plane models in 2D.",
            );
        ui.end_row();
    }
    if output.kind != OutputKind::Contact {
        ui.label("Global");
        on_off(
            ui,
            &mut output.global,
            "field global",
            ["Yes", "No"],
            "Whether the results are written in the global or in the local coordinate systems.",
        );
    }
}

fn validate_history_output(
    output: &HistoryOutput,
    region: &RegionDraft,
    fe: &FeModel,
) -> Result<(), String> {
    if output.variables.is_empty() {
        return Err("Please select at least one variable.".into());
    }
    match &output.kind {
        HistoryKind::Contact { pair } => {
            if !fe.contact_pair_names().contains(&pair.as_str()) {
                return Err(
                    "Please select a contact pair; create one under Contact Pairs first.".into(),
                );
            }
        }
        _ => {
            if region.is_empty() {
                return Err("The region is empty.".into());
            }
        }
    }
    Ok(())
}

/// Keeps a default name in step with the item kind, e.g. Fixed-1 becomes the next free
/// Displacement_Rotation-n, but leaves names the user chose.
fn rename_default(name: &mut String, from: &str, to: &str, taken: &[&str]) {
    if let Some(number) = name.strip_prefix(from).and_then(|n| n.strip_prefix('-'))
        && number.parse::<u32>().is_ok()
    {
        *name = next_name(to, taken.iter().copied());
    }
}

/// The step a boundary condition or load belongs to; picking another moves it there. Only
/// the steps `takes` accepts are offered.
fn step_row(ui: &mut Ui, step: &mut usize, fe: &FeModel, takes: impl Fn(&Step) -> bool) {
    ui.label("Step");
    let current = fe.steps.get(*step).map_or("", |s| s.name.as_str());
    egui::ComboBox::from_id_salt("item step")
        .selected_text(current)
        .width(200.0)
        .show_ui(ui, |ui| {
            for (i, s) in fe.steps.iter().enumerate() {
                if takes(s) || i == *step {
                    ui.selectable_value(step, i, &s.name);
                }
            }
        });
    ui.end_row();
}

pub(crate) fn name_row(ui: &mut Ui, name: &mut String) {
    ui.label("Name");
    ui.add(egui::TextEdit::singleline(name).desired_width(200.0));
    ui.end_row();
}

/// Profile, normal and offsets of a beam section, with CalculiX's directions: the
/// 1-direction is the normal, the 2-direction the beam axis crossed with it.
fn beam_form(ui: &mut Ui, beam: &mut BeamSection, fe: &FeModel, units: UnitSystem) {
    if fe.cross_section(&beam.cross_section).is_none()
        && let Some(first) = fe.cross_sections.first()
        && beam.cross_section.is_empty()
    {
        beam.cross_section = first.name.clone();
    }
    ui.label("Cross-section");
    let text = match beam.cross_section.as_str() {
        "" => "Select …",
        name => name,
    };
    egui::ComboBox::from_id_salt("beam cross-section")
        .selected_text(text)
        .width(200.0)
        .show_ui(ui, |ui| {
            for cross_section in &fe.cross_sections {
                let label = format!("{} ({})", cross_section.name, cross_section.profile.label());
                let name = cross_section.name.clone();
                ui.selectable_value(&mut beam.cross_section, name, label);
            }
        });
    ui.end_row();
    match fe.cross_section(&beam.cross_section) {
        Some(cross_section) => {
            ui.label("");
            crate::cross_section::preview(ui, &cross_section.profile, units);
            ui.end_row();
            if cross_section.profile.needs_reduced_integration() {
                ui.label("");
                ui.label(
                    egui::RichText::new("Pipe and box need lines with 3 nodes (B32R).")
                        .small()
                        .weak(),
                );
                ui.end_row();
            }
        }
        None => {
            ui.label("");
            ui.weak("Create cross-sections under Sections > Cross-Sections.");
            ui.end_row();
        }
    }
    ui.label("Normal (1-direction)");
    ui.horizontal(|ui| {
        let automatic = matches!(beam.orientation, BeamOrientation::Automatic);
        if ui.radio(automatic, "Automatic").clicked() && !automatic {
            beam.orientation = BeamOrientation::Automatic;
        }
        if ui.radio(!automatic, "Vector").clicked() && automatic {
            beam.orientation = BeamOrientation::Direction([0.0, 0.0, 1.0]);
        }
    });
    ui.end_row();
    match &mut beam.orientation {
        BeamOrientation::Automatic => {
            ui.label("");
            ui.label(
                egui::RichText::new("Global z-axis, for beams along z the x-axis.")
                    .small()
                    .weak(),
            );
            ui.end_row();
        }
        BeamOrientation::Direction(normal) => {
            ui.label("");
            ui.horizontal(|ui| {
                for (axis, value) in ["x", "y", "z"].iter().zip(normal.iter_mut()) {
                    ui.label(*axis);
                    ui.add(numeric::drag_value(value));
                }
            });
            ui.end_row();
        }
    }
    let [offset_1, offset_2] = &mut beam.offset;
    ui.label("Offset in 1-direction");
    ui.add(numeric::drag_value(offset_1));
    ui.end_row();
    ui.label("Offset in 2-direction");
    ui.add(numeric::drag_value(offset_2));
    ui.end_row();
}

fn material_form(ui: &mut Ui, material: &mut Material, units: UnitSystem) {
    name_row(ui, &mut material.name);
    let mut has_density = material.density.is_some();
    ui.checkbox(&mut has_density, "Density");
    let mut density = material.density.unwrap_or(0.0);
    expose::field(ui, Field::Density, |ui| {
        ui.add_enabled(
            has_density,
            numeric::physical(&mut density, units, Quantity::Density),
        )
    });
    material.density = has_density.then_some(density);
    ui.end_row();
    let mut elastic = material.elastic.is_some();
    ui.checkbox(&mut elastic, "Elasticity");
    ui.end_row();
    let mut values = material.elastic.unwrap_or(Elastic {
        young: 0.0,
        poisson: 0.0,
    });
    ui.label("    Young's modulus");
    expose::field(ui, Field::Young, |ui| {
        ui.add_enabled(
            elastic,
            numeric::physical(&mut values.young, units, Quantity::Pressure),
        )
    });
    ui.end_row();
    ui.label("    Poisson's ratio");
    expose::field(ui, Field::Poisson, |ui| {
        ui.add_enabled(
            elastic,
            numeric::drag_value(&mut values.poisson)
                .range(0.0..=0.5)
                .speed(0.01)
                .max_decimals(4),
        )
    });
    ui.end_row();
    material.elastic = elastic.then_some(values);
    plastic_rows(ui, material, units);
    creep_rows(ui, material, units);
    for (label, value, quantity, field) in [
        (
            "Thermal conductivity",
            &mut material.conductivity,
            Quantity::ThermalConductivity,
            Field::Conductivity,
        ),
        (
            "Specific heat",
            &mut material.specific_heat,
            Quantity::SpecificHeat,
            Field::SpecificHeat,
        ),
    ] {
        let mut set = value.is_some();
        ui.checkbox(&mut set, label);
        let mut number = value.unwrap_or(0.0);
        expose::field(ui, field, |ui| {
            ui.add_enabled(set, numeric::physical(&mut number, units, quantity))
        });
        *value = set.then_some(number);
        ui.end_row();
    }
    let mut expands = material.expansion.is_some();
    ui.checkbox(&mut expands, "Thermal expansion");
    ui.end_row();
    let mut expansion = material.expansion.unwrap_or_default();
    ui.label("    Expansion coefficient");
    expose::field(ui, Field::Expansion, |ui| {
        ui.add_enabled(
            expands,
            numeric::physical(
                &mut expansion.coefficient,
                units,
                Quantity::ThermalExpansion,
            ),
        )
    });
    ui.end_row();
    ui.label("    Reference temperature");
    ui.add_enabled(
        expands,
        numeric::quantity(
            &mut expansion.zero_temperature,
            units,
            Quantity::Temperature,
        ),
    );
    ui.end_row();
    material.expansion = expands.then_some(expansion);
}

/// The plasticity of the material form: the hardening rule and the hardening curve as an
/// editable table, rows of yield stress, plastic strain
/// and temperature.
fn plastic_rows(ui: &mut Ui, material: &mut Material, units: UnitSystem) {
    let mut plastic = material.plastic.is_some();
    ui.checkbox(&mut plastic, "Plasticity");
    ui.end_row();
    let mut values = material.plastic.clone().unwrap_or_default();
    ui.label("    Hardening");
    ui.add_enabled_ui(plastic, |ui| {
        egui::ComboBox::from_id_salt("plastic hardening")
            .selected_text(hardening_label(values.hardening))
            .width(200.0)
            .show_ui(ui, |ui| {
                for hardening in Hardening::ALL {
                    ui.selectable_value(
                        &mut values.hardening,
                        hardening,
                        hardening_label(hardening),
                    );
                }
            });
    });
    ui.end_row();
    ui.label("    Hardening curve");
    ui.add_enabled_ui(plastic, |ui| {
        ui.vertical(|ui| {
            let header = |quantity| {
                let unit = units.unit(quantity);
                if unit.is_empty() {
                    String::new()
                } else {
                    format!(" [{unit}]")
                }
            };
            let mut remove = None;
            egui::Grid::new("plastic points")
                .num_columns(4)
                .striped(true)
                .spacing([8.0, 4.0])
                .show(ui, |ui| {
                    ui.strong(format!("Yield stress{}", header(Quantity::Pressure)));
                    ui.strong("Plastic strain");
                    ui.strong(format!("Temperature{}", header(Quantity::Temperature)));
                    ui.label("");
                    ui.end_row();
                    let removable = values.points.len() > 1;
                    for (i, point) in values.points.iter_mut().enumerate() {
                        ui.add(numeric::without_unit(
                            &mut point.stress,
                            units,
                            Quantity::Pressure,
                        ));
                        ui.add(
                            numeric::drag_value(&mut point.plastic_strain)
                                .speed(0.001)
                                .range(0.0..=f64::MAX),
                        );
                        ui.add(numeric::without_unit(
                            &mut point.temperature,
                            units,
                            Quantity::Temperature,
                        ));
                        if ui
                            .add_enabled(removable, egui::Button::new("Remove").small())
                            .clicked()
                        {
                            remove = Some(i);
                        }
                        ui.end_row();
                    }
                });
            if let Some(i) = remove {
                values.points.remove(i);
            }
            if ui.button("Add row").clicked() {
                // Continues the curve: the same temperature, a larger plastic strain.
                let next = match values.points.as_slice() {
                    [.., a, b] => PlasticPoint {
                        stress: b.stress,
                        plastic_strain: b.plastic_strain
                            + (b.plastic_strain - a.plastic_strain).max(0.0),
                        temperature: b.temperature,
                    },
                    [b] => PlasticPoint {
                        stress: b.stress,
                        plastic_strain: b.plastic_strain + 0.1,
                        temperature: b.temperature,
                    },
                    [] => PlasticPoint {
                        stress: 0.0,
                        plastic_strain: 0.0,
                        temperature: 0.0,
                    },
                };
                values.points.push(next);
            }
            ui.weak(
                "First row at plastic strain 0 with the yield stress; the strain grows from \
                 row to row. Rows at other temperatures start at 0 again.",
            );
        });
    });
    ui.end_row();
    material.plastic = plastic.then_some(values);
}

/// Norton's creep law of the material form, which acts in Visco steps.
fn creep_rows(ui: &mut Ui, material: &mut Material, units: UnitSystem) {
    let mut creeps = material.creep.is_some();
    ui.checkbox(&mut creeps, "Creep (Norton)")
        .on_hover_text("The material creeps in Visco steps; Static steps ignore the creep law.");
    ui.end_row();
    let mut creep = material.creep.unwrap_or_default();
    ui.label("    A");
    ui.add_enabled(creeps, creep_coefficient(&mut creep, units));
    ui.end_row();
    for (label, value) in [
        ("    Stress exponent n", &mut creep.n),
        ("    Time exponent m", &mut creep.m),
    ] {
        ui.label(label);
        ui.add_enabled(creeps, numeric::drag_value(value).speed(0.0));
        ui.end_row();
    }
    ui.label("");
    let (stress, time) = (units.unit(Quantity::Pressure), units.unit(Quantity::Time));
    ui.weak(format!(
        "Creep strain rate = A * stress^n * time^m, stress in {stress}, time in {time}."
    ));
    ui.end_row();
    material.creep = creeps.then_some(creep);
}

/// The field of Norton's coefficient A, stored for stresses in Pa and shown for stresses
/// in the units of `units`: the strain rate A * stress^n stays the same, so A scales with
/// the n-th power of the unit of stress. All unit systems measure time in seconds.
pub fn creep_coefficient(creep: &mut plx_model::Creep, units: UnitSystem) -> egui::DragValue<'_> {
    let pascal = units.si_factor(Quantity::Pressure).unwrap_or(1.0);
    let factor = pascal.powf(creep.n);
    let a = &mut creep.a;
    egui::DragValue::from_get_set(move |shown| {
        if let Some(shown) = shown {
            *a = shown / factor;
        }
        *a * factor
    })
    .custom_parser(numeric::evaluate)
    .custom_formatter(|v, _| numeric::format_physical(v))
    .speed(0.0)
}

fn hardening_label(hardening: Hardening) -> &'static str {
    match hardening {
        Hardening::Isotropic => "Isotropic",
        Hardening::Kinematic => "Kinematic",
        Hardening::Combined => "Combined",
    }
}

/// Solution settings for a new static step: those of the last static step
/// are carried over.
fn previous_static(fe: &FeModel) -> StaticStep {
    (fe.steps.iter().rev())
        .find_map(|s| match &s.kind {
            StepKind::Static(settings) => Some(settings.clone()),
            _ => None,
        })
        .unwrap_or_default()
}

/// A new step starts with the boundary conditions and loads of the last step:
/// only those the new step takes, so a heat transfer
/// step leaves the supports behind and a static step the temperatures.
fn copy_items_of_last_step(fe: &FeModel, step: &mut Step) {
    let Some(last) = fe.steps.last() else {
        return;
    };
    step.boundary_conditions = (last.boundary_conditions.iter())
        .filter(|bc| step.kind.supports_boundary(&bc.kind))
        .cloned()
        .collect();
    step.loads = (last.loads.iter())
        .filter(|load| step.kind.supports_load(&load.kind))
        .cloned()
        .collect();
    if step.kind.supports_defined_fields() {
        step.defined_fields = last.defined_fields.clone();
    }
}

/// The step kinds of the dialog: label, the kind with its settings and its default field
/// outputs, in order.
fn step_kinds(fe: &FeModel) -> [(&'static str, StepKind, Vec<FieldOutput>); 12] {
    let heat = HeatTransferStep::default();
    [
        (
            STATIC_LABEL,
            StepKind::Static(previous_static(fe)),
            FieldOutput::defaults(),
        ),
        (
            FREQUENCY_LABEL,
            StepKind::Frequency(FrequencyStep::default()),
            FieldOutput::frequency_defaults(),
        ),
        (
            DYNAMIC_LABEL,
            StepKind::Dynamic(DynamicStep::default()),
            FieldOutput::dynamic_defaults(),
        ),
        (
            MODAL_DYNAMICS_LABEL,
            StepKind::ModalDynamics(ModalDynamicsStep::default()),
            FieldOutput::dynamic_defaults(),
        ),
        (
            STEADY_STATE_LABEL,
            StepKind::SteadyStateDynamics(SteadyStateDynamicsStep::default()),
            FieldOutput::defaults(),
        ),
        (
            COMPLEX_FREQUENCY_LABEL,
            StepKind::ComplexFrequency(ComplexFrequencyStep::default()),
            FieldOutput::complex_frequency_defaults(),
        ),
        (
            BUCKLE_LABEL,
            StepKind::Buckle(BuckleStep::default()),
            FieldOutput::defaults(),
        ),
        (
            HEAT_TRANSFER_LABEL,
            StepKind::HeatTransfer(heat.clone()),
            FieldOutput::heat_transfer_defaults(),
        ),
        (
            COUPLED_LABEL,
            StepKind::CoupledTempDisp(heat.clone()),
            FieldOutput::coupled_defaults(),
        ),
        (
            UNCOUPLED_LABEL,
            StepKind::UncoupledTempDisp(heat),
            FieldOutput::coupled_defaults(),
        ),
        (
            VISCO_LABEL,
            StepKind::Visco(ViscoStep::default()),
            FieldOutput::visco_defaults(),
        ),
        (
            SENSITIVITY_LABEL,
            StepKind::Sensitivity(SensitivityStep::default()),
            FieldOutput::sensitivity_defaults(),
        ),
    ]
}

const STATIC_LABEL: &str = "Static";
const FREQUENCY_LABEL: &str = "Frequency";
const HEAT_TRANSFER_LABEL: &str = "Heat Transfer";
const COUPLED_LABEL: &str = "Coupled Temperature-Displacement";
const UNCOUPLED_LABEL: &str = "Uncoupled Temperature-Displacement";
const DYNAMIC_LABEL: &str = "Dynamic (time integration)";
const MODAL_DYNAMICS_LABEL: &str = "Modal Dynamics (from stored modes)";
const STEADY_STATE_LABEL: &str = "Steady State Dynamics (frequency response)";
const COMPLEX_FREQUENCY_LABEL: &str = "Complex Frequency (rotating, Coriolis)";
const BUCKLE_LABEL: &str = "Buckle";
const SENSITIVITY_LABEL: &str = "Sensitivity (shape)";
const VISCO_LABEL: &str = "Visco (creep)";

fn step_kind_label(kind: &StepKind) -> &'static str {
    match kind {
        StepKind::Static(_) => STATIC_LABEL,
        StepKind::Frequency(_) => FREQUENCY_LABEL,
        StepKind::Dynamic(_) => DYNAMIC_LABEL,
        StepKind::ModalDynamics(_) => MODAL_DYNAMICS_LABEL,
        StepKind::SteadyStateDynamics(_) => STEADY_STATE_LABEL,
        StepKind::ComplexFrequency(_) => COMPLEX_FREQUENCY_LABEL,
        StepKind::Buckle(_) => BUCKLE_LABEL,
        StepKind::HeatTransfer(_) => HEAT_TRANSFER_LABEL,
        StepKind::CoupledTempDisp(_) => COUPLED_LABEL,
        StepKind::UncoupledTempDisp(_) => UNCOUPLED_LABEL,
        StepKind::Sensitivity(_) => SENSITIVITY_LABEL,
        StepKind::Visco(_) => VISCO_LABEL,
    }
}

/// The step dialog. The kind is chosen when the step is created, from the list of
/// step types; switching it starts with that kind's default field outputs.
/// `cyclic_sectors` is the number of sectors of a cyclic symmetric model.
fn step_form(
    ui: &mut Ui,
    step: &mut Step,
    creating: bool,
    fe: &FeModel,
    cyclic_sectors: Option<u32>,
) {
    name_row(ui, &mut step.name);
    ui.label("Type");
    if creating {
        let current = step_kind_label(&step.kind);
        egui::ComboBox::from_id_salt("step kind")
            .selected_text(current)
            .width(280.0)
            .show_ui(ui, |ui| {
                for (label, kind, outputs) in step_kinds(fe) {
                    if ui.selectable_label(current == label, label).clicked() && current != label {
                        step.kind = kind;
                        step.field_outputs = outputs;
                    }
                }
            });
    } else {
        ui.label(step_kind_label(&step.kind));
    }
    ui.end_row();
    let units = fe.properties.units;
    match &mut step.kind {
        StepKind::Static(settings) => static_form(ui, settings, units),
        StepKind::Frequency(settings) => frequency_form(ui, settings, units, cyclic_sectors),
        StepKind::Dynamic(settings) => dynamic_form(ui, settings, units),
        StepKind::ModalDynamics(settings) => modal_dynamics_form(ui, settings, units),
        StepKind::SteadyStateDynamics(settings) => steady_state_form(ui, settings, units),
        StepKind::ComplexFrequency(settings) => complex_frequency_form(ui, settings),
        StepKind::Buckle(settings) => buckle_form(ui, settings),
        StepKind::HeatTransfer(settings) => heat_transfer_form(ui, settings, units, false),
        StepKind::CoupledTempDisp(settings) => heat_transfer_form(ui, settings, units, true),
        // Nodalis offers no NLGEOM for the uncoupled step.
        StepKind::UncoupledTempDisp(settings) => heat_transfer_form(ui, settings, units, false),
        StepKind::Sensitivity(settings) => sensitivity_form(ui, settings, units),
        StepKind::Visco(settings) => visco_form(ui, settings, units),
    }
}

/// Settings of a visco step: those of a static step over the creep time and the creep
/// strain tolerance of the automatic increments.
fn visco_form(ui: &mut Ui, settings: &mut ViscoStep, units: UnitSystem) {
    increments_form(ui, &mut settings.increments, units, true);
    solver_row(ui, &mut settings.increments.solver);
    ui.label("Creep strain tolerance");
    ui.add(
        numeric::drag_value(&mut settings.creep_tolerance)
            .range(0.0..=1.0)
            .speed(0.0),
    )
    .on_hover_text(
        "Largest error of the creep strain in an increment (CETOL); smaller values give \
         shorter automatic increments.",
    );
    ui.end_row();
    ui.label("");
    ui.weak("Materials creep only with a creep law (Material > Creep); the others stay elastic.");
    ui.end_row();
}

/// Settings of a sensitivity step: the design responses and the filter. The design region
/// follows below the form.
fn sensitivity_form(ui: &mut Ui, settings: &mut SensitivityStep, units: UnitSystem) {
    ui.label("");
    ui.weak("Derivatives of the responses below with respect to the positions of the");
    ui.end_row();
    ui.label("");
    ui.weak("design nodes, from the solution of the previous static or frequency step.");
    ui.end_row();
    ui.label("Design responses");
    ui.vertical(|ui| {
        ui.checkbox(&mut settings.mass, "Mass")
            .on_hover_text("Mass of all elements (MASS); needs a density.");
        ui.checkbox(&mut settings.strain_energy, "Strain energy")
            .on_hover_text(
                "Strain energy of all elements under the loads of the previous static step \
                 (STRAINENERGY): the lower, the stiffer.",
            );
        ui.checkbox(&mut settings.eigenfrequency, "Eigenfrequencies")
            .on_hover_text(
                "Every eigenfrequency of the previous frequency step (EIGENFREQUENCY). \
                 No other response can be taken together with it.",
            );
    });
    ui.end_row();
    let mut filter = settings.filter_radius.is_some();
    if ui.checkbox(&mut filter, "Filter radius").changed() {
        settings.filter_radius =
            filter.then_some(UnitSystem::MmTonSC.stored(1.0, Quantity::Length));
    }
    let mut radius =
        (settings.filter_radius).unwrap_or(UnitSystem::MmTonSC.stored(1.0, Quantity::Length));
    let field = ui.add_enabled(
        filter,
        numeric::quantity(&mut radius, units, Quantity::Length).speed(0.1),
    );
    if filter && field.changed() {
        settings.filter_radius = Some(radius);
    }
    ui.end_row();
    ui.label("");
    ui.weak("The filter smooths the sensitivities over the given distance (*FILTER).");
    ui.end_row();
}

/// Settings of a heat transfer or coupled step: steady state or transient, then the
/// increments as in a static step.
fn heat_transfer_form(
    ui: &mut Ui,
    settings: &mut HeatTransferStep,
    units: UnitSystem,
    coupled: bool,
) {
    ui.label("");
    ui.checkbox(&mut settings.steady_state, "Steady state");
    ui.end_row();
    let mut limited = settings.deltmx.is_some();
    ui.add_enabled_ui(!settings.steady_state, |ui| {
        ui.checkbox(&mut limited, "Max. temperature change");
    });
    let mut deltmx = settings.deltmx.unwrap_or(10.0);
    ui.add_enabled(
        limited && !settings.steady_state,
        numeric::quantity(&mut deltmx, units, Quantity::TemperatureDifference)
            .range(0.0..=f64::MAX)
            .speed(1.0),
    );
    settings.deltmx = limited.then_some(deltmx);
    ui.end_row();
    increments_form(ui, &mut settings.increments, units, coupled);
    solver_row(ui, &mut settings.increments.solver);
    if !settings.steady_state {
        ui.label("");
        ui.weak("Transient: materials need density and specific heat.");
        ui.end_row();
    }
}

/// Settings of a dynamic step: the time integration, then the increments as in a static
/// step, then the Rayleigh damping.
fn dynamic_form(ui: &mut Ui, settings: &mut DynamicStep, units: UnitSystem) {
    ui.label("Procedure");
    egui::ComboBox::from_id_salt("dynamic procedure")
        .selected_text(settings.procedure.label())
        .show_ui(ui, |ui| {
            for choice in DynamicProcedure::ALL {
                ui.selectable_value(&mut settings.procedure, choice, choice.label());
            }
        });
    ui.end_row();
    ui.label("Alpha (HHT)");
    ui.add(
        numeric::drag_value(&mut settings.alpha)
            .range(-1.0 / 3.0..=0.0)
            .speed(0.01)
            .max_decimals(4),
    );
    ui.end_row();
    ui.label("");
    ui.weak("Numerical damping of the time integration, -1/3 to 0; -0.05 is the default.");
    ui.end_row();
    increments_form(ui, &mut settings.increments, units, true);
    solver_row(ui, &mut settings.increments.solver);
    let mut damped = settings.damping.is_some();
    ui.label("");
    ui.checkbox(&mut damped, "Rayleigh damping");
    ui.end_row();
    let mut damping = settings.damping.unwrap_or_default();
    ui.label("    Alpha (mass)");
    ui.add_enabled(
        damped,
        numeric::physical(&mut damping.alpha, units, Quantity::Frequency),
    );
    ui.end_row();
    ui.label("    Beta (stiffness)");
    ui.add_enabled(
        damped,
        numeric::physical(&mut damping.beta, units, Quantity::Time),
    );
    ui.end_row();
    ui.label("");
    ui.weak("Damping ratio zeta at circular frequency omega: alpha = 2 zeta omega,");
    ui.end_row();
    ui.label("");
    ui.weak("beta = 2 zeta / omega. Loads need a stepped amplitude to act at once.");
    ui.end_row();
    settings.damping = damped.then_some(damping);
}

/// Settings of a modal dynamics step: the time increments, then the modal damping.
fn modal_dynamics_form(ui: &mut Ui, settings: &mut ModalDynamicsStep, units: UnitSystem) {
    ui.label("");
    ui.checkbox(
        &mut settings.steady_state,
        "Steady state (until the response repeats)",
    );
    ui.end_row();
    ui.label("Time increment");
    ui.add(numeric::physical(
        &mut settings.increment,
        units,
        Quantity::Time,
    ));
    ui.end_row();
    if settings.steady_state {
        ui.label("Relative error");
        ui.add(
            numeric::drag_value(&mut settings.relative_error)
                .range(0.0..=1.0)
                .speed(0.001)
                .max_decimals(6),
        );
        ui.end_row();
    } else {
        ui.label("Time period");
        ui.add(numeric::physical(
            &mut settings.time_period,
            units,
            Quantity::Time,
        ));
        ui.end_row();
    }
    ui.label("Max. increments");
    ui.add(numeric::drag_value(&mut settings.max_increments).range(1..=1_000_000));
    ui.end_row();
    ui.label("");
    ui.weak("Needs a Frequency step with stored modes before it; loads need an amplitude.");
    ui.end_row();
    modal_damping_rows(ui, &mut settings.damping, units);
}

/// Settings of a steady state dynamics step: the frequency sweep, then the modal damping.
fn steady_state_form(ui: &mut Ui, settings: &mut SteadyStateDynamicsStep, units: UnitSystem) {
    ui.label("");
    ui.checkbox(&mut settings.harmonic, "Harmonic excitation");
    ui.end_row();
    for (label, value) in [
        ("Lower frequency", &mut settings.lower_frequency),
        ("Upper frequency", &mut settings.upper_frequency),
    ] {
        ui.label(label);
        ui.add(numeric::physical(value, units, Quantity::Frequency));
        ui.end_row();
    }
    ui.label("Data points");
    ui.add(numeric::drag_value(&mut settings.data_points).range(2..=100_000));
    ui.end_row();
    ui.label("Bias");
    ui.add(
        numeric::drag_value(&mut settings.bias)
            .range(1.0..=1000.0)
            .speed(0.1),
    );
    ui.end_row();
    ui.label("");
    ui.weak("Points between two eigenfrequencies; a bias above 1 crowds them near the modes.");
    ui.end_row();
    if !settings.harmonic {
        ui.label("Fourier terms");
        ui.add(numeric::drag_value(&mut settings.fourier_terms).range(1..=10_000));
        ui.end_row();
        for (label, value) in [
            ("Period start", &mut settings.time_lower),
            ("Period end", &mut settings.time_upper),
        ] {
            ui.label(label);
            ui.add(numeric::physical(value, units, Quantity::Time));
            ui.end_row();
        }
    }
    ui.label("");
    ui.weak("Needs a Frequency step with stored modes before it.");
    ui.end_row();
    modal_damping_rows(ui, &mut settings.damping, units);
}

/// The modal damping of a modal step: off, one ratio, a ratio per mode range, or Rayleigh.
fn modal_damping_rows(ui: &mut Ui, damping: &mut Option<ModalDamping>, units: UnitSystem) {
    let current = match damping {
        None => 0,
        Some(ModalDamping::Constant(_)) => 1,
        Some(ModalDamping::Direct(_)) => 2,
        Some(ModalDamping::Rayleigh(_)) => 3,
    };
    const CHOICES: [&str; 4] = ["Off", "Constant ratio", "Ratio per mode range", "Rayleigh"];
    ui.label("Modal damping");
    let mut chosen = current;
    egui::ComboBox::from_id_salt("modal damping")
        .selected_text(CHOICES[current])
        .show_ui(ui, |ui| {
            for (i, label) in CHOICES.iter().enumerate() {
                ui.selectable_value(&mut chosen, i, *label);
            }
        });
    ui.end_row();
    if chosen != current {
        *damping = match chosen {
            1 => Some(ModalDamping::Constant(0.02)),
            2 => Some(ModalDamping::Direct(vec![ModeDamping {
                lowest: 1,
                highest: 10,
                ratio: 0.02,
            }])),
            3 => Some(ModalDamping::Rayleigh(Default::default())),
            _ => None,
        };
    }
    match damping {
        None => {}
        Some(ModalDamping::Constant(ratio)) => {
            ui.label("    Damping ratio");
            ui.add(
                numeric::drag_value(ratio)
                    .range(0.0..=1.0)
                    .speed(0.001)
                    .max_decimals(6),
            );
            ui.end_row();
        }
        Some(ModalDamping::Direct(ranges)) => {
            let mut remove = None;
            for (i, range) in ranges.iter_mut().enumerate() {
                ui.label(if i == 0 { "    Modes, ratio" } else { "" });
                ui.horizontal(|ui| {
                    ui.add(numeric::drag_value(&mut range.lowest).range(1..=1_000_000));
                    ui.label("to");
                    ui.add(numeric::drag_value(&mut range.highest).range(1..=1_000_000));
                    ui.add(
                        numeric::drag_value(&mut range.ratio)
                            .range(0.0..=1.0)
                            .speed(0.001)
                            .max_decimals(6),
                    );
                    if ui.small_button("x").clicked() {
                        remove = Some(i);
                    }
                });
                ui.end_row();
            }
            if let Some(i) = remove {
                ranges.remove(i);
            }
            ui.label("");
            if ui.button("Add range").clicked() {
                let next = ranges.last().map_or(1, |r| r.highest + 1);
                ranges.push(ModeDamping {
                    lowest: next,
                    highest: next,
                    ratio: 0.02,
                });
            }
            ui.end_row();
        }
        Some(ModalDamping::Rayleigh(r)) => {
            ui.label("    Alpha (mass)");
            ui.add(numeric::physical(&mut r.alpha, units, Quantity::Frequency));
            ui.end_row();
            ui.label("    Beta (stiffness)");
            ui.add(numeric::physical(&mut r.beta, units, Quantity::Time));
            ui.end_row();
        }
    }
}

fn validate_heat_transfer_step(settings: &HeatTransferStep) -> Result<(), String> {
    if settings.deltmx.is_some_and(|d| d <= 0.0) && !settings.steady_state {
        return Err("The maximum temperature change must be greater than zero.".into());
    }
    Ok(())
}

/// "Equation solver: Automatic | Direct | Iterative" of the steps whose systems the
/// iterative solver can solve: static (also nonlinear and with contact), visco, dynamic,
/// heat transfer, frequency and buckle.
fn solver_row(ui: &mut Ui, solver: &mut LinearSolverChoice) {
    ui.label("Equation solver");
    ui.horizontal(|ui| {
        for choice in LinearSolverChoice::ALL {
            ui.radio_value(solver, choice, choice.name())
                .on_hover_text(solver_hint(choice));
        }
    });
    ui.end_row();
}

fn solver_hint(choice: LinearSolverChoice) -> &'static str {
    match choice {
        LinearSolverChoice::Automatic => {
            "The solver chooses: direct while the factorization fits into the free memory, \
             iterative for models too large for it. If the chosen solver fails, the other \
             one takes over."
        }
        LinearSolverChoice::Direct => {
            "Sparse direct factorization (LDL^T). Solves every system; needs memory for the \
             factor, which grows quickly with the model."
        }
        LinearSolverChoice::Iterative => {
            "Conjugate gradients with an incomplete Cholesky preconditioner. Needs a fraction \
             of the memory and is often faster on large solid models, but only solves \
             positive definite systems; fails near buckling or with parts held only by \
             contact."
        }
    }
}

fn frequency_form(
    ui: &mut Ui,
    settings: &mut FrequencyStep,
    units: UnitSystem,
    cyclic_sectors: Option<u32>,
) {
    ui.label("");
    ui.checkbox(
        &mut settings.perturbation,
        "Perturbation (prestress from previous step)",
    );
    ui.end_row();
    ui.label("Number of eigenfrequencies");
    expose::field(ui, Field::Modes, |ui| {
        ui.add(numeric::drag_value(&mut settings.num_frequencies).range(1..=10_000))
    });
    ui.end_row();
    for (label, bound) in [
        ("Lower frequency bound", &mut settings.lower_frequency),
        ("Upper frequency bound", &mut settings.upper_frequency),
    ] {
        let mut set = bound.is_some();
        ui.checkbox(&mut set, label);
        let mut value = bound.unwrap_or(0.0);
        ui.add_enabled(
            set,
            numeric::quantity(&mut value, units, Quantity::Frequency)
                .range(0.0..=f64::MAX)
                .speed(1.0),
        );
        *bound = set.then_some(value);
        ui.end_row();
    }
    ui.label("");
    ui.checkbox(
        &mut settings.storage,
        "Store matrices and eigenmodes (Storage, .eig)",
    );
    ui.end_row();
    if let Some(sectors) = cyclic_sectors {
        nodal_diameter_rows(ui, &mut settings.nodal_diameters, sectors);
    }
    solver_row(ui, &mut settings.solver);
    ui.label("");
    ui.weak("Loads have no effect in a frequency step; only the boundary conditions count.");
    ui.end_row();
}

/// The nodal diameters a cyclic symmetric model is solved for, the minimum and
/// maximum cyclic symmetry mode; unchecked, the solver solves all of them.
fn nodal_diameter_rows(ui: &mut Ui, range: &mut Option<[u32; 2]>, sectors: u32) {
    let highest = CyclicSymmetry::highest_nodal_diameter(sectors);
    let mut set = range.is_some();
    ui.checkbox(&mut set, "Nodal diameter range")
        .on_hover_text(format!(
            "Cyclic symmetry: the eigenfrequencies are computed for each nodal diameter \
             from 0 to {highest}, or only for this range. The number of eigenfrequencies \
             counts per nodal diameter."
        ));
    let mut values = range.unwrap_or([0, highest]);
    ui.add_enabled_ui(set, |ui| {
        ui.horizontal(|ui| {
            ui.label("from");
            ui.add(crate::numeric::count(&mut values[0]).range(0..=highest));
            ui.label("to");
            let low = values[0];
            ui.add(crate::numeric::count(&mut values[1]).range(low..=highest));
        });
    });
    *range = set.then_some(values);
    ui.end_row();
}

/// Settings of a complex frequency step: the number of modes; the Coriolis
/// option is CalculiX's, with a hint at what the step builds on. The perturbation flag is the
/// one of the Frequency step, since the solver requires the same on both steps.
fn complex_frequency_form(ui: &mut Ui, settings: &mut ComplexFrequencyStep) {
    ui.label("");
    ui.checkbox(
        &mut settings.coriolis,
        "Coriolis forces of the rotation (CORIOLIS)",
    );
    ui.end_row();
    ui.label("Number of complex frequencies");
    ui.add(numeric::drag_value(&mut settings.num_frequencies).range(1..=10_000));
    ui.end_row();
    ui.label("");
    ui.weak(
        "Solved on the modes of the last Frequency step with Storage before this step.\n\
         The rotation comes from a Centrifugal load in the Static step before that one.\n\
         Perturbation is taken from that Frequency step: tick \"Perturbation\" there\n\
         to include the stiffening by the centrifugal preload.\n\
         Loads have no effect in this step; only the boundary conditions count.",
    );
    ui.end_row();
}

fn validate_frequency_step(settings: &FrequencyStep) -> Result<(), String> {
    if let (Some(lower), Some(upper)) = (settings.lower_frequency, settings.upper_frequency)
        && lower >= upper
    {
        return Err("The lower frequency bound must be less than the upper one.".into());
    }
    Ok(())
}

/// The buckle step view: perturbation, solver, number of buckling factors and
/// accuracy.
fn buckle_form(ui: &mut Ui, settings: &mut BuckleStep) {
    ui.label("");
    ui.checkbox(
        &mut settings.perturbation,
        "Preload from previous step (Perturbation)",
    );
    ui.end_row();
    ui.label("Number of buckling factors");
    ui.add(numeric::drag_value(&mut settings.num_factors).range(1..=10_000));
    ui.end_row();
    ui.label("Accuracy");
    ui.add(numeric::drag_value(&mut settings.accuracy).speed(0.0));
    ui.end_row();
    solver_row(ui, &mut settings.solver);
    ui.label("");
    ui.weak("The critical load is the buckling factor times the loads of this step.");
    ui.end_row();
}

fn validate_buckle_step(settings: &BuckleStep) -> Result<(), String> {
    if settings.accuracy <= 0.0 {
        return Err("The accuracy must be greater than zero.".into());
    }
    Ok(())
}

fn static_form(ui: &mut Ui, settings: &mut StaticStep, units: UnitSystem) {
    increments_form(ui, settings, units, true);
    solver_row(ui, &mut settings.solver);
}

/// Nonlinear geometry and increments of a step with a time period; `mechanical`
/// shows the geometric nonlinearity, which only steps with displacements have.
fn increments_form(ui: &mut Ui, settings: &mut StaticStep, units: UnitSystem, mechanical: bool) {
    if mechanical {
        ui.label("");
        ui.checkbox(&mut settings.nlgeom, "Nonlinear geometry (Nlgeom)");
        ui.end_row();
    }
    ui.label("Incrementation");
    egui::ComboBox::from_id_salt("incrementation")
        .selected_text(incrementation_label(settings.incrementation))
        .show_ui(ui, |ui| {
            for choice in [
                Incrementation::Default,
                Incrementation::Automatic,
                Incrementation::Direct,
            ] {
                let label = incrementation_label(choice);
                ui.selectable_value(&mut settings.incrementation, choice, label);
            }
        });
    ui.end_row();
    let custom = settings.incrementation != Incrementation::Default;
    let automatic = settings.incrementation == Incrementation::Automatic;
    ui.label("Max. increments");
    ui.add_enabled(
        custom,
        numeric::drag_value(&mut settings.max_increments).range(1..=1_000_000),
    );
    ui.end_row();
    for (label, value, enabled, field) in [
        (
            "Time period",
            &mut settings.time_period,
            custom,
            Some(Field::TimePeriod),
        ),
        (
            "Initial increment",
            &mut settings.initial_increment,
            custom,
            Some(Field::InitialIncrement),
        ),
        (
            "Min. increment",
            &mut settings.min_increment,
            automatic,
            None,
        ),
        (
            "Max. increment",
            &mut settings.max_increment,
            automatic,
            None,
        ),
    ] {
        ui.label(label);
        let mut add =
            |ui: &mut Ui| ui.add_enabled(enabled, numeric::physical(value, units, Quantity::Time));
        match field {
            Some(field) => expose::field(ui, field, add),
            None => add(ui),
        };
        ui.end_row();
    }
}

fn incrementation_label(incrementation: Incrementation) -> &'static str {
    match incrementation {
        Incrementation::Default => "Default",
        Incrementation::Automatic => "Automatic",
        Incrementation::Direct => "Direct",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contact_detections_rename_mark_and_release_their_regions() {
        let mut fe = FeModel::default();
        let editor = Editor::create(NewItem::ContactDetection, &fe).unwrap();
        assert_eq!(editor.title(), "Create Contact Detection: Contacts");
        assert_eq!(editor.detection(&fe), Some(0));
        editor.apply(&mut fe);
        let found = |name: &str, edited| plx_model::ContactRegion {
            side_a: Region::Faces(vec![(1, 2)]),
            side_b: Region::Faces(vec![(2, 1)]),
            detected_by: Some("Contacts".into()),
            edited,
            ..plx_model::ContactRegion::new(name, plx_model::ContactBehavior::Bonded)
        };
        fe.contact_regions = vec![found("A", false), found("B", false)];
        // Changing a region in its dialog marks it; applying it unchanged does not.
        let mesh = FeMesh::default();
        let unchanged = Editor::edit(&TreeItem::ContactRegion(1), &fe, &mesh).unwrap();
        unchanged.apply(&mut fe);
        assert!(!fe.contact_regions[1].edited);
        let mut editor = Editor::edit(&TreeItem::ContactRegion(1), &fe, &mesh).unwrap();
        if let Draft::ContactRegion(region, ..) = &mut editor.draft {
            region.behavior = plx_model::ContactBehavior::Rough;
        }
        editor.apply(&mut fe);
        assert!(fe.contact_regions[1].edited);
        // Renaming the detection renames it in its regions.
        let mut editor = Editor::edit(&TreeItem::ContactDetection(0), &fe, &mesh).unwrap();
        if let Draft::ContactDetection(detection) = &mut editor.draft {
            detection.name = "Bolts".into();
        }
        assert_eq!(editor.detection(&fe), Some(0));
        editor.apply(&mut fe);
        assert_eq!(fe.contact_regions[0].detected_by.as_deref(), Some("Bolts"));
        // Deleting it deletes what it found; a region the user changed stays, as one of
        // their own.
        assert!(delete(&mut fe, &TreeItem::ContactDetection(0)));
        assert!(fe.contact_detections.is_empty());
        assert_eq!(fe.contact_regions.len(), 1);
        assert_eq!(fe.contact_regions[0].name, "B");
        assert_eq!(fe.contact_regions[0].detected_by, None);
        assert!(!fe.contact_regions[0].edited);
    }

    #[test]
    fn contact_regions_are_created_edited_flipped_and_deleted() {
        let mut fe = FeModel::default();
        let editor = Editor::create(NewItem::ContactRegion, &fe).unwrap();
        assert_eq!(editor.title(), "Create Contact Region: Contact_Region-1");
        // Both sides must be picked.
        assert_eq!(
            editor.validate(&fe),
            Err("The Side A region is empty.".into())
        );
        let Draft::ContactRegion(mut region, mut sides, view) = editor.draft else {
            unreachable!()
        };
        sides.master = RegionDraft::from_region(
            &Region::Faces(vec![(1, 2)]),
            FACE_SOURCES,
            Target::Surfaces,
            &FeMesh::default(),
        );
        sides.slave = RegionDraft::from_region(
            &Region::Faces(vec![(2, 1)]),
            FACE_SOURCES,
            Target::Surfaces,
            &FeMesh::default(),
        );
        region.behavior = plx_model::ContactBehavior::Frictional { mu: 0.2 };
        let editor = Editor {
            draft: Draft::ContactRegion(region, sides, view),
            ..Editor::create(NewItem::ContactRegion, &fe).unwrap()
        };
        assert_eq!(editor.validate(&fe), Ok(()));
        assert_eq!(
            editor.new_interaction_item(&fe),
            Some(TreeItem::ContactRegion(0))
        );
        editor.apply(&mut fe);
        let region = &fe.contact_regions[0];
        assert_eq!(region.side_a, Region::Faces(vec![(1, 2)]));
        assert_eq!(region.side_b, Region::Faces(vec![(2, 1)]));
        let item = TreeItem::ContactRegion(0);
        assert!(swap_master_slave(&mut fe, &item));
        assert_eq!(fe.contact_regions[0].side_a, Region::Faces(vec![(2, 1)]));
        assert!(toggle_active(&mut fe, &item));
        assert!(!fe.contact_regions[0].active);
        // The connections edit the settings all regions share.
        let mut editor =
            Editor::edit(&TreeItem::Group("Contacts"), &fe, &FeMesh::default()).unwrap();
        assert_eq!(editor.title(), "Connections");
        if let Draft::Connections(defaults) = &mut editor.draft {
            defaults.formulation = Some(plx_model::ContactMethod::Mortar);
        }
        editor.apply(&mut fe);
        assert_eq!(
            fe.contact_defaults.formulation,
            Some(plx_model::ContactMethod::Mortar)
        );
        assert!(delete(&mut fe, &item));
        assert!(fe.contact_regions.is_empty());
    }

    #[test]
    fn a_new_load_is_a_force() {
        let mut fe = FeModel::default();
        Editor::create(NewItem::Step, &fe).unwrap().apply(&mut fe);
        let Draft::Load(_, load, region) = Editor::create(NewItem::Load(0), &fe).unwrap().draft
        else {
            unreachable!()
        };
        assert_eq!(load.name, "Force-1");
        assert_eq!(load.kind, LoadKind::force([0.0; 3]));
        assert_eq!(region.target, Target::Nodes);
        assert!(region.sources.contains(&Source::ReferencePoint));
        assert_eq!(load_kinds()[0].0, "Force");
    }

    #[test]
    fn a_node_history_output_takes_the_point_of_a_remote_displacement() {
        let mut fe = FeModel::default();
        Editor::create(NewItem::Step, &fe).unwrap().apply(&mut fe);
        let editor = Editor::create(NewItem::HistoryOutput(0), &fe).unwrap();
        let Draft::HistoryOutput(_, _, region) = &editor.draft else {
            unreachable!()
        };
        assert!(region.sources.contains(&Source::RemotePoint));
        let point = Region::RemotePoint("Remote_Displacement-1".into());
        let draft = RegionDraft::from_region(
            &point,
            HISTORY_NODE_SOURCES,
            Target::Nodes,
            &FeMesh::default(),
        );
        assert_eq!(draft.region(), point);
        // History outputs follow a renamed remote displacement.
        let mut output = HistoryOutput::node("NH_Output-1", point);
        output.normalize_variables();
        fe.steps[0].history_outputs.push(output);
        fe.rename_remote_point("Remote_Displacement-1", "Shaft");
        assert_eq!(
            fe.steps[0].history_outputs[0].kind.region(),
            Some(&Region::RemotePoint("Shaft".into()))
        );
    }

    #[test]
    fn converting_a_region_keeps_what_the_new_kind_takes() {
        let mesh = FeMesh::default();
        let mut faces = RegionDraft::new(FACE_SOURCES, Target::Faces);
        faces.take(&mesh, Items::Faces([(1, 2)].into()), Operation::Replace);
        // A pressure's faces stay for a moving load's faces, but not for nodes.
        let (kept, dropped) = faces.converted(FACE_SOURCES, Target::Faces, &mesh);
        assert_eq!(
            (kept.region(), dropped),
            (Region::Faces(vec![(1, 2)]), false)
        );
        let (cleared, dropped) = faces.converted(NODE_SOURCES, Target::Nodes, &mesh);
        assert!(cleared.is_empty() && dropped);
        // A named surface needs a kind that lists surfaces.
        let surface = RegionDraft::from_region(
            &Region::Surface("BORE".into()),
            FACE_SOURCES,
            Target::Faces,
            &mesh,
        );
        let (kept, _) = surface.converted(SUPPORT_SOURCES, Target::Nodes, &mesh);
        assert_eq!(kept.region(), Region::Surface("BORE".into()));
        let (cleared, dropped) = surface.converted(SOLID_SOURCES, Target::Faces, &mesh);
        assert!(cleared.is_empty() && dropped);
        // Nothing picked, nothing dropped.
        let (_, dropped) = RegionDraft::new(FACE_SOURCES, Target::Faces).converted(
            NODE_SOURCES,
            Target::Nodes,
            &mesh,
        );
        assert!(!dropped);
    }

    #[test]
    fn default_names_follow_the_kind() {
        let mut name = "Displacement_Rotation-2".to_string();
        rename_default(
            &mut name,
            DISPLACEMENT,
            SYMMETRY,
            &["Displacement_Rotation-1"],
        );
        assert_eq!(name, "Symmetry-1");
        let mut name = "Clamp".to_string();
        rename_default(&mut name, DISPLACEMENT, SYMMETRY, &[]);
        assert_eq!(name, "Clamp");
    }

    #[test]
    fn switching_the_kind_proposes_a_free_name() {
        let mut fe = FeModel::default();
        let mut step = Step::new_static("Step-1");
        step.boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Displacement_Rotation-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(Vec::new()),
            kind: BoundaryKind::displacement([Some(0.0), None, None, None, None, None]),
            amplitude: None,
        });
        fe.steps.push(step);
        let mut editor = Editor::create(NewItem::BoundaryCondition(0), &fe).unwrap();
        let taken = editor.taken(&fe);
        let Draft::BoundaryCondition(_, bc, _) = &mut editor.draft else {
            unreachable!()
        };
        assert_eq!(bc.name, "Displacement_Rotation-2");
        rename_default(&mut bc.name, DISPLACEMENT, SYMMETRY, &taken);
        assert_eq!(bc.name, "Symmetry-1");
        // An edited item does not block its own name.
        let mut editor = Editor::create(NewItem::BoundaryCondition(0), &fe).unwrap();
        editor.index = Some(0);
        assert!(editor.taken(&fe).is_empty());
    }

    #[test]
    fn regions_survive_the_dialog() {
        let mesh = FeMesh::default();
        for (region, target) in [
            (Region::Nodes(vec![1, 5]), Target::Nodes),
            (Region::Faces(vec![(3, 2), (4, 6)]), Target::Faces),
            (Region::NodeSet("FIX".into()), Target::Nodes),
            (Region::Surface("TIP".into()), Target::Faces),
        ] {
            let draft = RegionDraft::from_region(&region, NODE_SOURCES, target, &mesh);
            assert_eq!(draft.region(), region);
        }
        let parts = Region::Parts(vec!["A".into(), "B".into()]);
        let draft = RegionDraft::from_region(&parts, ELEMENT_SOURCES, Target::Faces, &mesh);
        assert_eq!(draft.region(), parts);
        // Element regions list whole parts first; picked faces still reopen as a selection.
        let faces = Region::Faces(vec![(3, 2)]);
        let draft = RegionDraft::from_region(&faces, SOLID_SOURCES, Target::Faces, &mesh);
        assert_eq!(draft.source, Source::Selection);
        assert_eq!(draft.region(), faces);
    }

    #[test]
    fn new_items_get_default_names_and_land_in_their_step() {
        let mut fe = FeModel::default();
        for kind in [NewItem::Material, NewItem::Step] {
            Editor::create(kind, &fe).unwrap().apply(&mut fe);
        }
        assert_eq!(fe.materials[0].name, "Material-1");
        assert_eq!(fe.steps[0].name, "Step-1");
        let mut editor = Editor::create(NewItem::BoundaryCondition(0), &fe).unwrap();
        assert!(editor.validate(&fe).is_err(), "empty region");
        if let Draft::BoundaryCondition(_, _, region) = &mut editor.draft {
            region
                .nodes
                .push(Operation::Replace, BTreeSet::from([1, 4]));
        }
        assert_eq!(editor.validate(&fe), Ok(()));
        editor.apply(&mut fe);
        let bc = &fe.steps[0].boundary_conditions[0];
        assert_eq!(
            (bc.name.as_str(), &bc.region),
            ("Displacement_Rotation-1", &Region::Nodes(vec![1, 4]))
        );
        assert!(Editor::create(NewItem::Load(3), &fe).is_none());
    }

    #[test]
    fn the_step_row_moves_a_load_to_another_step() {
        let mut fe = FeModel::default();
        for name in ["Step-1", "Step-2"] {
            fe.steps.push(Step::new_static(name));
        }
        let mut editor = Editor::create(NewItem::Load(0), &fe).unwrap();
        if let Draft::Load(_, _, region) = &mut editor.draft {
            region.nodes.push(Operation::Replace, BTreeSet::from([1]));
        }
        editor.apply(&mut fe);
        fe.steps[0].loads[0].active = false;
        let item = TreeItem::Load(0, 0);
        let mut editor = Editor::edit(&item, &fe, &FeMesh::default()).unwrap();
        if let Draft::Load(step, ..) = &mut editor.draft {
            *step = 1;
        }
        editor.apply(&mut fe);
        assert!(fe.steps[0].loads.is_empty());
        let moved = &fe.steps[1].loads[0];
        assert_eq!(moved.name, "Force-1");
        assert!(!moved.active, "it keeps being switched off");
    }

    #[test]
    fn beam_sections_follow_a_renamed_cross_section() {
        let mut fe = FeModel::default();
        Editor::create(NewItem::CrossSection, &fe)
            .unwrap()
            .apply(&mut fe);
        assert_eq!(fe.cross_sections[0].name, "Cross_Section-1");
        fe.sections.push(Section {
            name: "Beam_Section-1".into(),
            material: String::new(),
            region: Region::Parts(Vec::new()),
            thickness: 1.0,
            kind: SectionKind::Beam(BeamSection {
                cross_section: "Cross_Section-1".into(),
                ..BeamSection::DEFAULT
            }),
        });
        let mesh = FeMesh::default();
        let mut editor = Editor::edit(&TreeItem::CrossSection(0), &fe, &mesh).unwrap();
        let Draft::CrossSection(cross_section) = &mut editor.draft else {
            panic!("not a cross-section");
        };
        cross_section.name = "Tube".into();
        cross_section.profile = BeamProfile::Pipe {
            radius: 0.01,
            thickness: 0.02,
        };
        assert!(
            editor.validate(&fe).is_err(),
            "wall thicker than the radius"
        );
        if let Draft::CrossSection(c) = &mut editor.draft {
            c.profile = BeamProfile::ALL[2].clone();
        }
        assert_eq!(editor.validate(&fe), Ok(()));
        editor.apply(&mut fe);
        let SectionKind::Beam(beam) = &fe.sections[0].kind else {
            panic!("not a beam");
        };
        assert_eq!(beam.cross_section, "Tube");
        assert!(delete(&mut fe, &TreeItem::CrossSection(0)));
        assert!(fe.cross_sections.is_empty());
    }

    #[test]
    fn field_outputs_are_created_deactivated_and_deleted() {
        let mut fe = FeModel::default();
        Editor::create(NewItem::Step, &fe).unwrap().apply(&mut fe);
        let mut editor = Editor::create(NewItem::FieldOutput(0), &fe).unwrap();
        if let Draft::FieldOutput(_, output) = &mut editor.draft {
            assert_eq!(output.name, "NF_Output-1");
            output.kind = OutputKind::Element;
            output.variables = vec!["ZZS".into(), "S".into(), "ERR".into()];
        }
        assert!(
            editor.validate(&fe).is_err(),
            "ERR and ZZS exclude each other"
        );
        if let Draft::FieldOutput(_, output) = &mut editor.draft {
            output.variables.pop();
        }
        assert_eq!(editor.validate(&fe), Ok(()));
        editor.apply(&mut fe);
        let added = TreeItem::FieldOutput(0, 2);
        assert_eq!(fe.steps[0].field_outputs[2].variables, ["S", "ZZS"]);
        assert!(toggle_active(&mut fe, &added));
        assert!(!fe.steps[0].field_outputs[2].active);
        // Editing keeps the output deactivated.
        let editor = Editor::edit(&added, &fe, &FeMesh::default()).unwrap();
        editor.apply(&mut fe);
        assert!(!fe.steps[0].field_outputs[2].active);
        assert!(delete(&mut fe, &added));
        assert_eq!(fe.steps[0].field_outputs.len(), 2);
    }

    #[test]
    fn a_frequency_step_keeps_the_bcs_but_takes_no_loads() {
        let mut fe = FeModel::default();
        Editor::create(NewItem::Step, &fe).unwrap().apply(&mut fe);
        fe.steps[0].boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Fixed-1".into(),
            active: true,
            visible: true,
            region: Region::NodeSet("FIX".into()),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        fe.steps[0].loads.push(Load {
            distribution: None,
            name: "Pressure-1".into(),
            active: true,
            visible: true,
            region: Region::Surface("TOP".into()),
            kind: LoadKind::Pressure(1.0),
            amplitude: None,
            factor_amplitude: None,
        });
        let mut editor = Editor::create(NewItem::Step, &fe).unwrap();
        if let Draft::Step(step, _) = &mut editor.draft {
            step.kind = StepKind::Frequency(FrequencyStep {
                lower_frequency: Some(100.0),
                upper_frequency: Some(50.0),
                ..FrequencyStep::default()
            });
        }
        assert!(editor.validate(&fe).is_err(), "bounds the wrong way round");
        if let Draft::Step(
            Step {
                kind: StepKind::Frequency(settings),
                ..
            },
            _,
        ) = &mut editor.draft
        {
            settings.upper_frequency = None;
        }
        assert_eq!(editor.validate(&fe), Ok(()));
        editor.apply(&mut fe);
        let frequency = &fe.steps[1];
        assert_eq!(frequency.name, "Step-2");
        assert_eq!(
            frequency.boundary_conditions,
            fe.steps[0].boundary_conditions
        );
        assert!(frequency.loads.is_empty());
        assert!(Editor::create(NewItem::Load(1), &fe).is_none());
        // A static step after it starts from the last static step's settings.
        let StepKind::Static(settings) = &mut fe.steps[0].kind else {
            unreachable!()
        };
        settings.nlgeom = true;
        Editor::create(NewItem::Step, &fe).unwrap().apply(&mut fe);
        assert!(matches!(&fe.steps[2].kind, StepKind::Static(s) if s.nlgeom));
    }

    #[test]
    fn a_heat_transfer_step_takes_only_thermal_items() {
        let mut fe = FeModel::default();
        Editor::create(NewItem::Step, &fe).unwrap().apply(&mut fe);
        let mut editor = Editor::create(NewItem::Step, &fe).unwrap();
        if let Draft::Step(step, _) = &mut editor.draft {
            step.kind = StepKind::CoupledTempDisp(HeatTransferStep::default());
        }
        editor.apply(&mut fe);
        let step = &mut fe.steps[1];
        step.boundary_conditions = vec![
            BoundaryCondition {
                distribution: None,
                name: "Fixed-1".into(),
                active: true,
                visible: true,
                region: Region::NodeSet("FIX".into()),
                kind: BoundaryKind::displacement([Some(0.0); 6]),
                amplitude: None,
            },
            BoundaryCondition {
                distribution: None,
                name: "Temperature-1".into(),
                active: true,
                visible: true,
                region: Region::NodeSet("HOT".into()),
                kind: BoundaryKind::Temperature(100.0),
                amplitude: None,
            },
        ];
        step.loads = vec![
            Load {
                distribution: None,
                name: "Pressure-1".into(),
                active: true,
                visible: true,
                region: Region::Surface("TOP".into()),
                kind: LoadKind::Pressure(1.0),
                amplitude: None,
                factor_amplitude: None,
            },
            Load {
                distribution: None,
                name: "Convective_Film-1".into(),
                active: true,
                visible: true,
                region: Region::Surface("TOP".into()),
                kind: LoadKind::Film {
                    sink: 20.0,
                    coefficient: 0.01,
                },
                amplitude: None,
                factor_amplitude: None,
            },
        ];
        // A heat transfer step after it keeps the temperature and the film only.
        let mut editor = Editor::create(NewItem::Step, &fe).unwrap();
        if let Draft::Step(step, _) = &mut editor.draft {
            step.kind = StepKind::HeatTransfer(HeatTransferStep::default());
        }
        editor.apply(&mut fe);
        let heat = &fe.steps[2];
        let names = |s: &Step| -> Vec<String> {
            (s.boundary_conditions.iter().map(|b| b.name.clone()))
                .chain(s.loads.iter().map(|l| l.name.clone()))
                .collect()
        };
        assert_eq!(names(heat), ["Temperature-1", "Convective_Film-1"]);
        // New items there start as the first thermal kind.
        let Draft::BoundaryCondition(_, bc, _) = Editor::create(NewItem::BoundaryCondition(2), &fe)
            .unwrap()
            .draft
        else {
            unreachable!()
        };
        assert_eq!(
            (bc.name.as_str(), bc.kind),
            ("Temperature-2", BoundaryKind::Temperature(0.0))
        );
        let Draft::Load(_, load, region) = Editor::create(NewItem::Load(2), &fe).unwrap().draft
        else {
            unreachable!()
        };
        assert_eq!(load.kind, LoadKind::ConcentratedFlux(0.0));
        assert_eq!(region.target, Target::Nodes);
        // A static step after it leaves the thermal items behind.
        Editor::create(NewItem::Step, &fe).unwrap().apply(&mut fe);
        assert!(names(&fe.steps[3]).is_empty());
    }

    #[test]
    fn initial_temperatures_are_created_and_switched() {
        let mut fe = FeModel::default();
        let mut editor = Editor::create(NewItem::InitialCondition, &fe).unwrap();
        assert!(editor.validate(&fe).is_err(), "empty region");
        if let Draft::InitialCondition(_, region) = &mut editor.draft {
            region.source = Source::Parts;
            region.parts = PartPicks::from_names(["A".to_string()]);
        }
        assert_eq!(editor.validate(&fe), Ok(()));
        editor.apply(&mut fe);
        let condition = &fe.initial_conditions[0];
        assert_eq!(condition.name, "Temperature-1");
        assert_eq!(condition.region, Region::Parts(vec!["A".into()]));
        assert_eq!(condition.kind, InitialConditionKind::Temperature(20.0));
        let item = TreeItem::InitialCondition(0);
        assert!(toggle_active(&mut fe, &item));
        assert!(!fe.initial_conditions[0].active);
        assert!(delete(&mut fe, &item));
        assert!(fe.initial_conditions.is_empty());
    }

    #[test]
    fn symbols_are_hidden_and_shown() {
        let mut fe = FeModel::default();
        Editor::create(NewItem::Step, &fe).unwrap().apply(&mut fe);
        fe.steps[0].loads.push(Load {
            distribution: None,
            name: "Force-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![1]),
            kind: LoadKind::force([1.0, 0.0, 0.0]),
            amplitude: None,
            factor_amplitude: None,
        });
        assert!(set_visible(&mut fe, &TreeItem::Load(0, 0), false));
        assert!(!fe.steps[0].loads[0].visible);
        assert!(!set_visible(
            &mut fe,
            &TreeItem::BoundaryCondition(0, 0),
            false
        ));
        assert!(!set_visible(&mut fe, &TreeItem::Step(0), false));
    }

    #[test]
    fn steps_bcs_and_loads_are_switched_off_and_on() {
        let mut fe = FeModel::default();
        Editor::create(NewItem::Step, &fe).unwrap().apply(&mut fe);
        fe.steps[0].boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Fixed-1".into(),
            active: true,
            visible: true,
            region: Region::NodeSet("FIX".into()),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        let bc = TreeItem::BoundaryCondition(0, 0);
        let editor = Editor::edit(&bc, &fe, &FeMesh::default()).unwrap();
        assert!(toggle_active(&mut fe, &bc));
        assert!(!fe.steps[0].boundary_conditions[0].active);
        // A dialog opened before keeps the switch as it is now.
        editor.apply(&mut fe);
        assert!(!fe.steps[0].boundary_conditions[0].active);
        assert!(toggle_active(&mut fe, &TreeItem::Step(0)));
        let editor = Editor::edit(&TreeItem::Step(0), &fe, &FeMesh::default()).unwrap();
        editor.apply(&mut fe);
        assert!(!fe.steps[0].active);
        // A new step takes the items of the last one as they are, as copies.
        Editor::create(NewItem::Step, &fe).unwrap().apply(&mut fe);
        assert!(fe.steps[1].active);
        assert!(!fe.steps[1].boundary_conditions[0].active);
        assert!(toggle_active(&mut fe, &bc));
        assert!(fe.steps[0].boundary_conditions[0].active);
        assert!(!toggle_active(&mut fe, &TreeItem::Load(0, 0)));
        assert!(!toggle_active(&mut fe, &TreeItem::Material(0)));
    }

    #[test]
    fn steps_are_moved_with_their_items() {
        let mut fe = FeModel::default();
        for _ in 0..3 {
            Editor::create(NewItem::Step, &fe).unwrap().apply(&mut fe);
        }
        let names: Vec<_> = fe.steps.iter().map(|s| s.name.clone()).collect();
        assert_eq!(move_step(&mut fe, 0, true), None);
        assert_eq!(move_step(&mut fe, 2, false), None);
        assert_eq!(move_step(&mut fe, 2, true), Some(1));
        assert_eq!(
            fe.steps.iter().map(|s| s.name.as_str()).collect::<Vec<_>>(),
            [&names[0], &names[2], &names[1]]
        );
        assert_eq!(move_step(&mut fe, 0, false), Some(1));
        assert_eq!(fe.steps[0].name, names[2]);
    }

    #[test]
    fn a_swapped_name_that_is_taken_gets_a_number() {
        let mut fe = FeModel::default();
        fe.contact_pairs.push(ContactPair::new("A_to_B", ""));
        fe.contact_pairs.push(ContactPair::new("B_to_A", ""));
        fe.contact_pairs[0].master = Region::Surface("A".into());
        assert!(swap_master_slave(&mut fe, &TreeItem::ContactPair(0)));
        assert_eq!(fe.contact_pairs[0].name, "B_to_A-1");
        assert_eq!(fe.contact_pairs[0].slave, Region::Surface("A".into()));
        fe.constraints
            .push(Constraint::Tie(plx_model::Tie::new("Tie-1")));
        assert!(swap_master_slave(&mut fe, &TreeItem::Constraint(0)));
        assert!(!swap_master_slave(&mut fe, &TreeItem::Material(0)));
    }

    #[test]
    fn loads_follow_a_renamed_amplitude_and_drop_it_when_they_cannot_use_it() {
        let mut fe = FeModel::default();
        for kind in [NewItem::Amplitude, NewItem::Step] {
            Editor::create(kind, &fe).unwrap().apply(&mut fe);
        }
        assert_eq!(fe.amplitudes[0].name, "Amplitude-1");
        let mut film = Load {
            distribution: None,
            name: "Film-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![1]),
            kind: LoadKind::Film {
                sink: 20.0,
                coefficient: 1.0,
            },
            amplitude: Some("Amplitude-1".into()),
            factor_amplitude: Some("Amplitude-1".into()),
        };
        fe.steps[0].loads.push(film.clone());
        let mut editor = Editor::edit(&TreeItem::Amplitude(0), &fe, &FeMesh::default()).unwrap();
        if let Draft::Amplitude(amplitude, _) = &mut editor.draft {
            amplitude.name = "Ramp".into();
        }
        editor.apply(&mut fe);
        let load = &fe.steps[0].loads[0];
        assert_eq!(load.amplitude.as_deref(), Some("Ramp"));
        assert_eq!(load.factor_amplitude.as_deref(), Some("Ramp"));
        // A pressure has no film coefficient to scale.
        film.kind = LoadKind::Pressure(1.0);
        film.factor_amplitude = Some("Ramp".into());
        Editor {
            draft: Draft::Load(0, film, RegionDraft::new(FACE_SOURCES, Target::Faces)),
            index: Some(0),
            error: None,
            picker: Picker::default(),
            motion: None,
            cyclic: Default::default(),
            expose: None,
            linked: Vec::new(),
            detection_scope: None,
            from_step: None,
        }
        .apply(&mut fe);
        assert_eq!(fe.steps[0].loads[0].factor_amplitude, None);
        assert!(delete(&mut fe, &TreeItem::Amplitude(0)));
        assert!(fe.amplitudes.is_empty());
    }

    #[test]
    fn a_part_shows_what_acts_on_it() {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/platte_mit_stuetzen.inp");
        let mesh = plx_io::inp::read_inp(&path).unwrap().mesh;
        let mut model = Model::new(&path, mesh);
        let part = |name: &str| {
            (model.mesh.parts.iter())
                .position(|p| p.name == name)
                .unwrap()
        };
        let (plate, columns) = (part("PLATTE"), part("STUETZEN"));
        let mut step = Step::new_static("Step-1");
        step.boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Feet".into(),
            active: true,
            visible: true,
            region: Region::NodeSet("FUESSE".into()),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        step.loads.push(Load {
            distribution: None,
            name: "Pressure".into(),
            active: true,
            visible: true,
            region: Region::Faces(vec![(1, 1), (2, 1)]),
            kind: LoadKind::Pressure(1.0),
            amplitude: None,
            factor_amplitude: None,
        });
        model.fe.steps.push(step);
        let mut pair = ContactPair::new("Contact_Pair-1", "Interaction-1");
        pair.master = Region::Parts(vec!["PLATTE".into()]);
        pair.slave = Region::Parts(vec!["STUETZEN".into()]);
        model.fe.contact_pairs.push(pair);

        let acting = |parts: &[usize]| acting_on_parts(&model, &parts.iter().copied().collect());
        assert_eq!(
            acting(&[plate]),
            [TreeItem::ContactPair(0), TreeItem::Load(0, 0)].into()
        );
        assert_eq!(
            acting(&[columns]),
            [TreeItem::ContactPair(0), TreeItem::BoundaryCondition(0, 0)].into()
        );
        assert_eq!(acting(&[plate, columns]).len(), 3);
    }

    #[test]
    fn sets_are_created_renamed_and_take_over_sets_of_the_input_file() {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/kragbalken_c3d8.inp");
        let mesh = plx_io::inp::read_inp(&path).unwrap().mesh;
        let mut model = Model::new(&path, mesh);
        let part = model.mesh.parts[0].name.clone();

        // A new element set of the whole part.
        let mut editor = Editor::create_set(SetKind::Elements, &model.fe, &model.mesh);
        let Draft::MeshSet(set, region, _) = &mut editor.draft else {
            panic!("set dialog expected");
        };
        assert_eq!(set.name, "Element_Set-1");
        region.source = Source::Parts;
        region.parts = PartPicks::from_names([part.clone()]);
        assert_eq!(editor.validate(&model.fe), Ok(()));
        assert_eq!(editor.validate_set(&model), Ok(()));
        // An element set must not take a part's name.
        if let Draft::MeshSet(set, ..) = &mut editor.draft {
            set.name = part.clone();
        }
        assert!(editor.validate_set(&model).is_err());
        if let Draft::MeshSet(set, ..) = &mut editor.draft {
            set.name = "Beam".into();
        }
        let sets = model.fe.mesh_sets.clone();
        editor.apply(&mut model.fe);
        model.fe.sync_mesh_sets(&sets, &mut model.mesh);
        let elements = model.mesh.element_sets["Beam"].len();
        assert_eq!(elements, model.mesh.element_count());

        // A section on the set follows its new name.
        model.fe.sections.push(Section {
            name: "Section-1".into(),
            material: "Steel".into(),
            region: Region::ElementSet("Beam".into()),
            thickness: 1.0,
            kind: SectionKind::Solid,
        });
        let item = TreeItem::ElementSet("Beam".into());
        let mut editor = Editor::edit(&item, &model.fe, &model.mesh).unwrap();
        if let Draft::MeshSet(set, ..) = &mut editor.draft {
            set.name = "Cantilever".into();
        }
        assert_eq!(editor.validate_set(&model), Ok(()));
        let sets = model.fe.mesh_sets.clone();
        editor.apply(&mut model.fe);
        model.fe.sync_mesh_sets(&sets, &mut model.mesh);
        assert!(!model.mesh.element_sets.contains_key("Beam"));
        assert_eq!(model.mesh.element_sets["Cantilever"].len(), elements);
        assert_eq!(
            model.fe.sections[0].region,
            Region::ElementSet("Cantilever".into())
        );

        // A node set of the input file keeps its nodes and becomes the user's.
        let (name, nodes) = (model.mesh.node_sets.iter().next())
            .map(|(n, ids)| (n.clone(), ids.clone()))
            .unwrap();
        let item = TreeItem::NodeSet(name.clone());
        let editor = Editor::edit(&item, &model.fe, &model.mesh).unwrap();
        assert_eq!(editor.replaced_set(), Some((SetKind::Nodes, name.as_str())));
        assert_eq!(editor.validate_set(&model), Ok(()));
        let sets = model.fe.mesh_sets.clone();
        editor.apply(&mut model.fe);
        model.fe.sync_mesh_sets(&sets, &mut model.mesh);
        assert_eq!(
            model.fe.mesh_set(SetKind::Nodes, &name).unwrap().region,
            Region::Nodes(nodes.clone())
        );
        assert_eq!(model.mesh.node_sets[&name], nodes);

        // A new mesh gets the sets again.
        let mut mesh = model.mesh.clone();
        mesh.element_sets.remove("Cantilever");
        model.set_mesh(mesh);
        assert_eq!(model.mesh.element_sets["Cantilever"].len(), elements);
    }
}
