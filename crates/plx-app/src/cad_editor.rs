//! The editor of a CAD Operation block of the project schematic: the geometry linked to the
//! block in its own 3D view, where the edges, faces or corners the operation acts on are
//! picked, with the
//! kind of operation, its length and a preview of the result.

use std::collections::BTreeSet;
use std::path::Path;

use plx_mesher::CadEntity;
use plx_model::{
    CadOperation, CadOperationKind, DefeatureFaces, Geometry, Quantity, SplitKeep, SplitPlane,
    UnitSystem,
};

use crate::model::Model;
use crate::viewport::{Click, ViewCommand, Viewport};

pub enum CadEditorResult {
    Open,
    /// Closed with OK: the operation, and its result when the preview made it for exactly
    /// this operation.
    Ok(CadOperation, Option<Box<Geometry>>),
    Cancel,
}

/// Which geometry the view shows.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Shown {
    Input,
    Result,
}

pub struct CadOperationEditor {
    /// The block edited.
    pub id: u64,
    name: String,
    input: Geometry,
    /// The input as shown, where edges or faces are picked; `None` when it cannot be shown.
    view: Option<Model>,
    /// The input has faces but no solids: fillets and chamfers act on corners.
    flat: bool,
    /// The result of the last preview and the operation it was made for.
    result: Option<(Model, Geometry, CadOperation)>,
    shown: Shown,
    kind: CadOperationKind,
    value: f64,
    /// The plane, the side kept and whether split solids stay one part, for a split.
    plane: SplitPlane,
    keep: SplitKeep,
    compound: bool,
    /// The parts selected when the split was opened, and whether it cuts only them.
    parts: Vec<String>,
    only_parts: bool,
    /// The global axis a split plane is tilted about and the angle in degrees.
    tilt_axis: usize,
    tilt_angle: f64,
    /// Which faces a defeaturing removes.
    faces: DefeatureFaces,
    /// The fillets and chamfers found in the input for a defeaturing, with the size they
    /// were found for, or why they could not be.
    found: Option<(f64, Result<BTreeSet<CadEntity>, String>)>,
    /// What the last preview of a defeaturing removed.
    note: Option<String>,
    units: UnitSystem,
    /// A parameter study gives the length.
    length_linked: bool,
    /// The picked edges or faces of the input.
    picks: BTreeSet<CadEntity>,
    /// Why the input cannot be shown or the preview failed.
    message: Option<String>,
    /// The view has to take the shown geometry anew.
    scene_due: bool,
    /// The camera has to frame the geometry, once it is shown first.
    fit_due: bool,
}

impl CadOperationEditor {
    pub fn new(
        id: u64,
        name: &str,
        input: Geometry,
        operation: &CadOperation,
        units: UnitSystem,
        length_linked: bool,
    ) -> Self {
        // Picks only stay with the signatures of the entities they are.
        let input = if input.signatures.is_empty() {
            plx_mesher::with_signatures(input.clone()).unwrap_or(input)
        } else {
            input
        };
        let (view, flat, message) = match plx_mesher::tessellate(&input) {
            Ok(display) => {
                let flat = display.solids == 0 && display.faces > 0;
                // Suppressed parts are drawn as their edges and cannot be picked.
                let mut view = Model::geometry_view(Path::new(name), display);
                view.mark_suppressed(&input);
                (Some(view), flat, None)
            }
            Err(error) => (
                None,
                false,
                Some(format!("The geometry cannot be shown: {error}")),
            ),
        };
        let picks = (operation.resolve(&input).unwrap_or_default().into_iter())
            .filter(|&e| operation.kind.picks(e))
            .collect();
        let mut editor = Self {
            id,
            name: name.to_owned(),
            input,
            view,
            flat,
            result: None,
            shown: Shown::Input,
            kind: operation.kind,
            value: operation.value,
            plane: operation.plane,
            keep: operation.keep,
            compound: operation.compound,
            parts: operation.parts.clone(),
            only_parts: !operation.parts.is_empty(),
            tilt_axis: operation.tilt_axis.min(2),
            tilt_angle: operation.tilt_angle,
            faces: operation.faces,
            found: None,
            note: None,
            units,
            length_linked,
            picks,
            message,
            scene_due: true,
            fit_due: true,
        };
        // Turning a plane about its own normal leaves it as it is: an axis lying in it.
        let axes = editor.plane.tilt_axes();
        if !axes.contains(&editor.tilt_axis) {
            editor.tilt_axis = axes[0];
        }
        editor.update_highlight();
        editor
    }

    /// The geometry it acts on, with the signatures of its entities.
    pub fn input(&self) -> &Geometry {
        &self.input
    }

    pub fn rename(&mut self, name: &str) {
        name.clone_into(&mut self.name);
    }

    /// The operation as set in the dialog.
    fn operation(&self) -> CadOperation {
        let entities = (self.picks.iter())
            .filter(|&&e| self.kind.picks(e))
            .filter_map(|&e| {
                self.input
                    .signatures
                    .iter()
                    .find(|s| s.entity == e)
                    .cloned()
            })
            .collect();
        CadOperation {
            kind: self.kind,
            entities,
            value: self.value,
            plane: self.plane,
            keep: self.keep,
            compound: self.compound,
            faces: self.faces,
            parts: self.split_parts().to_vec(),
            tilt_axis: self.tilt_axis,
            tilt_angle: self.tilt_angle,
        }
    }

    /// The parts a split cuts; all when empty.
    fn split_parts(&self) -> &[String] {
        if self.kind == CadOperationKind::SplitPlane && self.only_parts {
            &self.parts
        } else {
            &[]
        }
    }

    /// The indices of the parts of the view a split cuts, none when it cuts all.
    fn split_part_indices(&self) -> Vec<usize> {
        let (parts, Some(view)) = (self.split_parts(), self.view.as_ref()) else {
            return Vec::new();
        };
        (view.parts.iter().enumerate())
            .filter(|(_, p)| parts.iter().any(|n| n.eq_ignore_ascii_case(&p.name)))
            .map(|(i, _)| i)
            .collect()
    }

    /// Whether a click picks something: not for a split along a global plane, nor for a
    /// defeaturing of the faces found by size.
    fn picking(&self) -> bool {
        match self.kind {
            CadOperationKind::SplitPlane => self.plane == SplitPlane::Face,
            CadOperationKind::Defeature => self.faces == DefeatureFaces::Picked,
            _ => true,
        }
    }

    /// Whether the faces of a defeaturing are found by size.
    fn finding(&self) -> bool {
        self.kind == CadOperationKind::Defeature && self.faces == DefeatureFaces::Found
    }

    /// The fillets and chamfers found for the size, found anew when it changed.
    fn found_faces(&mut self) -> Result<BTreeSet<CadEntity>, String> {
        let size = self.value;
        if self.found.as_ref().is_none_or(|(at, _)| *at != size) {
            let found = plx_mesher::find_blends(&self.input, size)
                .map(|faces| faces.into_iter().collect())
                .map_err(|e| e.to_string());
            self.found = Some((size, found));
        }
        self.found
            .as_ref()
            .map_or(Ok(BTreeSet::new()), |(_, found)| found.clone())
    }

    fn update_highlight(&mut self) {
        let shown = if self.finding() {
            self.found_faces().unwrap_or_default()
        } else {
            self.picks.clone()
        };
        // The parts a split cuts are outlined, as when selected.
        let outlined = self.split_part_indices();
        if let Some(view) = &mut self.view {
            view.highlight = crate::meshing::entities_highlight(view, &shown);
            view.highlight.outlines.extend(outlined);
        }
        self.scene_due = true;
    }

    /// What is picked for the operation, one and many: corners of flat faces for a fillet
    /// or a chamfer of flat geometry.
    fn nouns(&self) -> (&'static str, &'static str) {
        if self.flat && self.corners() {
            ("corner", "corners")
        } else {
            (self.kind.picked_one(), self.kind.picked_noun())
        }
    }

    /// Whether it picks corners: a fillet or chamfer of flat geometry.
    fn corners(&self) -> bool {
        self.flat
            && matches!(
                self.kind,
                CadOperationKind::Fillet | CadOperationKind::Chamfer
            )
    }

    /// Where a picked vertex is drawn.
    fn vertex_position(&self, vertex: CadEntity) -> Option<glam::Vec3> {
        let view = self.view.as_ref()?;
        let signature = self.input.signatures.iter().find(|s| s.entity == vertex)?;
        Some(view.to_render(signature.centroid))
    }

    /// The corner a click points at: the vertex nearest to the click ray, within a few times
    /// the pick tolerance.
    fn corner_at(&self, click: &Click) -> Option<CadEntity> {
        let direction = click.direction.normalize_or_zero();
        (self.input.signatures.iter())
            .filter(|s| matches!(s.entity, CadEntity::Vertex(_)))
            .filter_map(|s| {
                let point = self.vertex_position(s.entity)?;
                let offset = point - click.origin;
                let foot = click.origin + direction * offset.dot(direction);
                let off_ray = (point - foot).length();
                (off_ray <= 2.0 * click.precision_at(foot)).then_some((s.entity, off_ray))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(vertex, _)| vertex)
    }

    /// The view as shown now.
    fn shown_model(&self) -> Option<&Model> {
        match self.shown {
            Shown::Input => self.view.as_ref(),
            Shown::Result => self.result.as_ref().map(|(model, ..)| model),
        }
    }

    /// What a click points at for the operation: the CAD edge near it, else the edge of
    /// the face under it nearest to it; or the planar face under it.
    fn entity_at(&self, click: &Click) -> Option<CadEntity> {
        if !self.picking() {
            return None;
        }
        if self.corners() {
            return self.corner_at(click);
        }
        let view = self.view.as_ref()?;
        let hit = view.pick_click(click)?;
        if self.kind.picks(CadEntity::Face(0)) {
            let face = crate::geometry_selection::face_at(view, &hit).map(CadEntity::Face)?;
            // Fillets are curved.
            let planar =
                (self.input.signatures.iter()).any(|s| s.entity == face && s.kind == "Plane");
            return (planar || self.kind == CadOperationKind::Defeature).then_some(face);
        }
        let edge = match crate::meshing::cad_pick(view, &hit, click.precision_at(hit.point)) {
            Some(edge @ CadEntity::Edge(_)) => Some(edge),
            _ => crate::geometry_selection::edge_at(view, &hit).map(CadEntity::Edge),
        }?;
        // The edges of holes are curved.
        let straight = (self.input.signatures.iter()).any(|s| s.entity == edge && s.kind == "Line");
        (self.kind != CadOperationKind::HoleRing || !straight).then_some(edge)
    }

    /// Applies the operation to the input and shows the result, or says why it failed.
    fn preview(&mut self) {
        let operation = self.operation();
        self.note = None;
        if operation.kind == CadOperationKind::Defeature {
            let made = operation.resolve(&self.input).and_then(|entities| {
                plx_mesher::defeature(&self.input, &operation, &entities).map_err(|e| e.to_string())
            });
            if let Ok(made) = &made {
                self.note = Some(match (made.removed, made.kept) {
                    (0, _) => "No fillets or chamfers found up to the max size.".to_owned(),
                    (1, 0) => "1 face removed.".to_owned(),
                    (removed, 0) => format!("{removed} faces removed."),
                    (removed, kept) => format!(
                        "{removed} faces removed; {kept} found could not be removed and stay."
                    ),
                });
            }
            self.show_result(made.map(|d| d.geometry), operation);
            return;
        }
        let made = operation
            .problem()
            .map_or_else(|| Ok(()), Err)
            .and_then(|()| operation.resolve(&self.input))
            .and_then(|entities| {
                plx_mesher::apply_operation(&self.input, &operation, &entities)
                    .map_err(|e| e.to_string())
            });
        self.show_result(made, operation);
    }

    /// Shows the result of a preview, or why it failed.
    fn show_result(&mut self, made: Result<Geometry, String>, operation: CadOperation) {
        let made = made.and_then(|geometry| {
            let display = plx_mesher::tessellate(&geometry).map_err(|e| e.to_string())?;
            let mut view = Model::geometry_view(Path::new(&self.name), display);
            view.mark_suppressed(&geometry);
            Ok((view, geometry))
        });
        match made {
            Ok((model, geometry)) => {
                self.result = Some((model, geometry, operation));
                self.shown = Shown::Result;
                self.message = None;
            }
            Err(why) => {
                self.result = None;
                self.shown = Shown::Input;
                self.message = Some(format!("The operation failed: {why}"));
            }
        }
        self.scene_due = true;
    }

    pub fn show(&mut self, ctx: &egui::Context, viewport: &mut Viewport) -> CadEditorResult {
        let mut result = CadEditorResult::Open;
        let mut open = true;
        egui::Window::new(format!("CAD Operation - {}", self.name))
            .id(crate::ids::window(ctx, ("cad operation", self.id)))
            .open(&mut open)
            .collapsible(false)
            .resizable(true)
            .default_size([760.0, 600.0])
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                self.form(ui);
                ui.separator();
                let height = (ui.available_height() - 40.0).max(320.0);
                let size = egui::vec2(ui.available_width().max(480.0), height);
                ui.allocate_ui(size, |ui| self.view_ui(ui, viewport));
                ui.separator();
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if ui.button("Cancel").clicked() {
                        result = CadEditorResult::Cancel;
                    }
                    let operation = self.operation();
                    let problem = operation.problem();
                    let ok = ui
                        .add_enabled(problem.is_none(), egui::Button::new("OK"))
                        .on_hover_text(
                            "Keeps the operation; Update Project makes the geometry again \
                             unless the preview made it.",
                        );
                    if ok.clicked() {
                        let made = (self.result.as_ref())
                            .filter(|(_, _, previewed)| *previewed == operation)
                            .map(|(_, geometry, _)| Box::new(geometry.clone()));
                        result = CadEditorResult::Ok(operation, made);
                    }
                    if let Some(problem) = problem {
                        ui.colored_label(crate::style::error(), problem);
                    }
                });
            });
        if !open {
            result = CadEditorResult::Cancel;
        }
        result
    }

    fn form(&mut self, ui: &mut egui::Ui) {
        egui::Grid::new("cad operation form")
            .num_columns(2)
            .spacing([12.0, 6.0])
            .show(ui, |ui| {
                ui.label("Operation");
                ui.horizontal(|ui| {
                    for kind in CadOperationKind::ALL {
                        // Moving a face out of its plane would make a solid of it; defeaturing
                        // removes faces of solids.
                        if self.flat
                            && matches!(
                                kind,
                                CadOperationKind::PushPull | CadOperationKind::Defeature
                            )
                        {
                            continue;
                        }
                        if ui
                            .selectable_value(&mut self.kind, kind, kind.label())
                            .changed()
                        {
                            // Edges stay picked from fillet to chamfer, not for faces.
                            self.picks.retain(|&e| kind.picks(e));
                            if kind == CadOperationKind::SplitPlane {
                                self.picks.clear();
                            }
                            self.update_highlight();
                        }
                    }
                });
                ui.end_row();
                ui.label(self.kind.value_label());
                ui.add_enabled_ui(!self.length_linked, |ui| {
                    let response = ui.add(crate::numeric::quantity(
                        &mut self.value,
                        self.units,
                        Quantity::Length,
                    ));
                    // The outline of the plane moves with its offset, the faces found with
                    // the size.
                    if response.changed() {
                        self.scene_due = true;
                        if self.finding() {
                            self.update_highlight();
                        }
                    }
                    if self.length_linked {
                        response.on_disabled_hover_text(
                            "A parameter study gives it through the link of its row.",
                        );
                    }
                });
                ui.end_row();
                if self.kind == CadOperationKind::SplitPlane {
                    self.split_rows(ui);
                }
                if self.kind == CadOperationKind::Defeature {
                    self.defeature_rows(ui);
                }
                if !self.picking() {
                    return;
                }
                let (one, many) = self.nouns();
                ui.label(capitalised(many));
                ui.horizontal(|ui| {
                    ui.label(match self.picks.len() {
                        0 => "none picked".to_owned(),
                        1 => format!("1 {one}"),
                        n => format!("{n} {many}"),
                    });
                    if ui
                        .add_enabled(!self.picks.is_empty(), egui::Button::new("Clear"))
                        .clicked()
                    {
                        self.picks.clear();
                        self.update_highlight();
                    }
                });
                ui.end_row();
            });
        ui.horizontal(|ui| {
            let previewed = self.result.is_some();
            ui.label("Show");
            for (shown, label) in [(Shown::Input, "Input"), (Shown::Result, "Result")] {
                let enabled = shown == Shown::Input || previewed;
                if ui
                    .add_enabled(
                        enabled,
                        egui::Button::selectable(self.shown == shown, label),
                    )
                    .clicked()
                {
                    self.shown = shown;
                    self.scene_due = true;
                }
            }
            ui.separator();
            if ui
                .add_enabled(
                    self.view.is_some() && (!self.picks.is_empty() || !self.picking()),
                    egui::Button::new("Preview"),
                )
                .on_hover_text("Applies the operation to the geometry and shows the result.")
                .clicked()
            {
                self.preview();
            }
            if ui.button("Fit").clicked() {
                self.fit_due = true;
            }
        });
        match &self.message {
            Some(message) => {
                ui.colored_label(crate::style::error(), message);
            }
            None if self.shown == Shown::Input => {
                let hint = match self.kind {
                    CadOperationKind::PushPull => "Click the planar faces to move along their \
                        normal, outwards for a positive offset; a click on a picked face removes \
                        it. Neighbours at right angles follow, sloped ones get a step."
                        .to_owned(),
                    CadOperationKind::SplitPlane if self.plane == SplitPlane::Face => format!(
                        "Click the planar face whose plane to cut along; the offset moves the \
                         cut into the part. {}",
                        self.split_scope()
                    ),
                    CadOperationKind::SplitPlane => format!(
                        "{} The outline shows where; the offset moves the plane along its axis, \
                         the tilt turns it about an axis through that point.",
                        self.split_scope()
                    ),
                    CadOperationKind::HoleRing => "Click the circular edges of holes on planar \
                        faces; each face gets a ring of the width around the hole, a face of \
                        its own to pick. A click on a picked edge removes it."
                        .to_owned(),
                    CadOperationKind::Defeature if self.finding() => match &self.found {
                        Some((_, Err(why))) => format!("The faces could not be found: {why}"),
                        Some((_, Ok(found))) => format!(
                            "{} found, drawn red: fillets of a radius up to the max size and \
                             chamfers up to that width. Holes and the round ends of slots \
                             stay. Preview removes them; the faces next to them grow until \
                             they meet.",
                            match found.len() {
                                0 => "No fillets or chamfers".to_owned(),
                                1 => "1 face".to_owned(),
                                n => format!("{n} faces"),
                            }
                        ),
                        None => String::new(),
                    },
                    CadOperationKind::Defeature => "Click the fillet and chamfer faces to \
                        remove; a click on a picked face removes it from the list. A fillet \
                        that runs over several faces needs all of them."
                        .to_owned(),
                    kind if self.corners() => format!(
                        "Click the corners of the faces to {}; a click on a picked corner \
                         removes it.",
                        kind.label().to_lowercase()
                    ),
                    kind => format!(
                        "Click the edges to {}; a click on a picked edge removes it.",
                        kind.label().to_lowercase()
                    ),
                };
                if self.input.suppressed.is_empty() {
                    ui.weak(hint);
                } else {
                    ui.weak(format!(
                        "{hint} Suppressed parts, drawn as their edges, stay as they are."
                    ));
                }
            }
            None if self.note.is_some() => {
                ui.weak(self.note.clone().unwrap_or_default());
            }
            None if !self.picking() => {
                ui.weak("The result of the operation.");
            }
            None => {
                ui.weak(format!(
                    "The result of the operation; show the input to pick {}.",
                    self.nouns().1
                ));
            }
        }
    }

    fn view_ui(&mut self, ui: &mut egui::Ui, viewport: &mut Viewport) {
        if self.scene_due {
            self.scene_due = false;
            match self.shown_model() {
                Some(model) => viewport.set_parts(&model.scene_meshes()),
                None => viewport.set_parts(&[]),
            }
            // The picked edges, drawn over the input in the highlight colour.
            let edges = match (self.shown, &self.view) {
                (Shown::Input, Some(view)) => (view.highlight.lines.iter())
                    .filter_map(|ends| {
                        let [a, b] = ends.map(|id| view.node_position(view.mesh.node_index(id)?));
                        Some([a?, b?])
                    })
                    .collect(),
                _ => Vec::new(),
            };
            let mut edges = edges;
            if self.shown == Shown::Input {
                edges.extend(self.plane_outline());
            }
            // The picked corners, drawn as points.
            let nodes = match self.shown {
                Shown::Input => (self.picks.iter())
                    .filter(|e| matches!(e, CadEntity::Vertex(_)))
                    .filter_map(|&e| self.vertex_position(e))
                    .collect(),
                Shown::Result => Vec::new(),
            };
            viewport.overlay = crate::overlay::Overlay {
                edges,
                nodes,
                show_view_triad: true,
                ..Default::default()
            };
        }
        if self.fit_due
            && let Some(bounds) = self.shown_model().and_then(Model::visible_bounds)
        {
            self.fit_due = false;
            // Flat geometry is seen straight onto its plane, which is normal to its thinnest
            // extent.
            if self.flat {
                let size = bounds.1 - bounds.0;
                let axis = match size.min_element() {
                    thinnest if thinnest == size.x => crate::viewport::Axis::X,
                    thinnest if thinnest == size.y => crate::viewport::Axis::Y,
                    _ => crate::viewport::Axis::Z,
                };
                viewport.apply(ViewCommand::AxisView(axis), Some(bounds));
            }
            viewport.apply(ViewCommand::Fit, Some(bounds));
        }
        let picking = self.shown == Shown::Input;
        viewport.selecting = picking;
        let response = viewport.ui(ui);
        if let Some(command) = response.command {
            let bounds = self.shown_model().and_then(Model::visible_bounds);
            viewport.apply(command, bounds);
        }
        if let Some(click) = response.pivot {
            let pivot = self.shown_model().and_then(|m| m.pick_click(&click));
            viewport.set_pivot(pivot.map(|hit| hit.point));
        }
        if !picking {
            viewport.preview = Default::default();
            return;
        }
        if let Some(click) = response.click
            && let Some(entity) = self.entity_at(&click)
        {
            if !self.picks.remove(&entity) {
                // A plane is taken from one face.
                if self.kind == CadOperationKind::SplitPlane {
                    self.picks.clear();
                }
                self.picks.insert(entity);
            }
            self.update_highlight();
        }
        if let Some(hover) = response.hover {
            viewport.preview = (hover.and_then(|click| self.entity_at(&click)))
                .zip(self.view.as_ref())
                .map(|(entity, view)| match entity {
                    CadEntity::Vertex(_) => crate::viewport::Preview {
                        points: self.vertex_position(entity).into_iter().collect(),
                        ..Default::default()
                    },
                    _ => crate::meshing::entities_preview(view, &BTreeSet::from([entity])),
                })
                .unwrap_or_default();
            ui.ctx().request_repaint();
        }
    }
}

impl CadOperationEditor {
    /// Which parts a split cuts, for the hint below the form.
    fn split_scope(&self) -> &'static str {
        if self.split_parts().is_empty() {
            "The parts the plane runs through are split."
        } else {
            "The selected parts the plane runs through are split, drawn outlined; the \
             others stay as they are."
        }
    }

    /// The rows of a split by plane: the plane, the side kept, whether solids cut in two
    /// stay one part, and whether it cuts only the parts selected.
    fn split_rows(&mut self, ui: &mut egui::Ui) {
        ui.label("Plane");
        let before = self.plane;
        egui::ComboBox::from_id_salt("split plane")
            .selected_text(self.plane.label())
            .show_ui(ui, |ui| {
                for plane in SplitPlane::ALL {
                    ui.selectable_value(&mut self.plane, plane, plane.label());
                }
            });
        if self.plane != before {
            self.picks.clear();
            self.update_highlight();
            // Turning a plane about its own normal leaves it as it is.
            let axes = self.plane.tilt_axes();
            if !axes.contains(&self.tilt_axis) {
                self.tilt_axis = axes[0];
            }
        }
        ui.end_row();
        ui.label("Tilt");
        ui.horizontal(|ui| {
            let mut changed = false;
            changed |= ui
                .add(
                    crate::numeric::drag_value(&mut self.tilt_angle)
                        .speed(1.0)
                        .suffix(" deg")
                        .max_decimals(6),
                )
                .on_hover_text(
                    "The angle the plane is turned by about the axis, right-handed; 0 leaves \
                     it as chosen.",
                )
                .changed();
            ui.label("about");
            egui::ComboBox::from_id_salt("split tilt axis")
                .width(40.0)
                .selected_text(AXIS_NAMES[self.tilt_axis])
                .show_ui(ui, |ui| {
                    for k in self.plane.tilt_axes() {
                        changed |= ui
                            .selectable_value(&mut self.tilt_axis, k, AXIS_NAMES[k])
                            .changed();
                    }
                })
                .response
                .on_hover_text(
                    "The global axis the plane is turned about, through the point of the \
                     plane the offset gives.",
                );
            if changed {
                self.scene_due = true;
            }
        });
        ui.end_row();
        ui.label("Keep");
        egui::ComboBox::from_id_salt("split keep")
            .selected_text(self.keep.label(self.plane))
            .show_ui(ui, |ui| {
                for keep in SplitKeep::ALL {
                    ui.selectable_value(&mut self.keep, keep, keep.label(self.plane));
                }
            });
        ui.end_row();
        ui.label("Parts");
        ui.add_enabled_ui(self.keep == SplitKeep::Both, |ui| {
            ui.checkbox(&mut self.compound, "Keep the pieces as one part")
                .on_hover_text(
                    "On: a solid or a shell cut in two stays one part of the FE model, a \
                     compound part of its pieces, which are tied where they were cut. Off: \
                     each piece is a part of its own.",
                );
        });
        ui.end_row();
        if !self.parts.is_empty() {
            ui.label("Cut");
            let label = match self.parts.len() {
                1 => format!("Only the selected part ({})", self.parts[0]),
                n => format!("Only the {n} selected parts"),
            };
            if ui
                .checkbox(&mut self.only_parts, label)
                .on_hover_text(
                    "On: the parts selected in the tree when the tool was opened are split, \
                     the others stay as they are. Off: every part the plane runs through.",
                )
                .changed()
            {
                self.update_highlight();
                self.scene_due = true;
            }
            ui.end_row();
        }
        if self.plane != before {
            self.scene_due = true;
        }
    }

    /// The row of a defeaturing: which faces it removes. Going to picked faces starts from
    /// those found.
    fn defeature_rows(&mut self, ui: &mut egui::Ui) {
        ui.label("Remove");
        ui.horizontal(|ui| {
            for faces in DefeatureFaces::ALL {
                if ui
                    .selectable_value(&mut self.faces, faces, faces.label())
                    .on_hover_text(match faces {
                        DefeatureFaces::Found => {
                            "The fillets and chamfers up to the max size, found again whenever \
                             the geometry or the size changes."
                        }
                        DefeatureFaces::Picked => {
                            "The faces clicked, starting from those found by size."
                        }
                    })
                    .changed()
                {
                    if self.faces == DefeatureFaces::Picked && self.picks.is_empty() {
                        self.picks = self.found_faces().unwrap_or_default();
                    }
                    self.update_highlight();
                }
            }
        });
        ui.end_row();
    }

    /// The outline of a global plane over the input, as edges in the view: a rectangle in
    /// the plane a little larger than the geometry, or the parts it cuts.
    fn plane_outline(&self) -> Vec<[glam::Vec3; 2]> {
        if self.kind != CadOperationKind::SplitPlane {
            return Vec::new();
        }
        let (Some(axis), Some(view)) = (self.plane.axis(), self.view.as_ref()) else {
            return Vec::new();
        };
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        let chosen = self.split_part_indices();
        if chosen.is_empty() {
            for s in &self.input.signatures {
                for j in 0..3 {
                    lo[j] = lo[j].min(s.min[j]);
                    hi[j] = hi[j].max(s.max[j]);
                }
            }
        } else {
            // Over the parts it cuts.
            for (min, max) in chosen.iter().filter_map(|&i| view.parts[i].bounds) {
                let (min, max) = (view.model_point(min), view.model_point(max));
                for j in 0..3 {
                    lo[j] = lo[j].min(min[j]);
                    hi[j] = hi[j].max(max[j]);
                }
            }
        }
        if !lo.iter().chain(&hi).all(|x| x.is_finite()) {
            return Vec::new();
        }
        let rect = tilted_rectangle(
            axis,
            self.value,
            self.tilt_axis,
            if self.tilt_angle.is_finite() {
                self.tilt_angle
            } else {
                0.0
            },
            lo,
            hi,
        );
        let c = rect.map(|p| view.to_render(p));
        vec![[c[0], c[1]], [c[1], c[2]], [c[2], c[3]], [c[3], c[0]]]
    }
}

const AXIS_NAMES: [&str; 3] = ["X", "Y", "Z"];

/// The corners of a rectangle in the global plane normal to `axis` at `offset`, tilted by
/// `degrees` about the global axis `tilt_axis` through the point `axis * offset`, which covers
/// the box `lo`..`hi` with a margin of a tenth of its size: the outline of a split plane.
fn tilted_rectangle(
    axis: [f64; 3],
    offset: f64,
    tilt_axis: usize,
    degrees: f64,
    lo: [f64; 3],
    hi: [f64; 3],
) -> [[f64; 3]; 4] {
    let dot = |a: [f64; 3], b: [f64; 3]| (0..3).map(|k| a[k] * b[k]).sum::<f64>();
    let turn = |v: [f64; 3]| plx_model::tilted_normal(v, tilt_axis, degrees);
    let k = axis.iter().position(|&a| a == 1.0).unwrap_or(0);
    let unit = |j: usize| {
        let mut e = [0.0; 3];
        e[j] = 1.0;
        e
    };
    // The in-plane directions of the untilted plane, turned with it.
    let (u, v) = (turn(unit((k + 1) % 3)), turn(unit((k + 2) % 3)));
    let origin = axis.map(|a| a * offset);
    // The extent of the box along u and v.
    let corners = (0..8).map(|i| [0, 1, 2].map(|j| if i >> j & 1 == 0 { lo[j] } else { hi[j] }));
    let (mut u0, mut u1, mut v0, mut v1) = (
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    );
    for p in corners {
        let d = [0, 1, 2].map(|j| p[j] - origin[j]);
        let (a, b) = (dot(d, u), dot(d, v));
        u0 = u0.min(a);
        u1 = u1.max(a);
        v0 = v0.min(b);
        v1 = v1.max(b);
    }
    let (mu, mv) = (0.1 * (u1 - u0).max(1e-9), 0.1 * (v1 - v0).max(1e-9));
    let (u0, u1, v0, v1) = (u0 - mu, u1 + mu, v0 - mv, v1 + mv);
    let at = |a: f64, b: f64| [0, 1, 2].map(|j| origin[j] + a * u[j] + b * v[j]);
    [at(u0, v0), at(u1, v0), at(u1, v1), at(u0, v1)]
}

fn capitalised(text: &str) -> String {
    let mut chars = text.chars();
    chars
        .next()
        .map_or_else(String::new, |c| c.to_uppercase().chain(chars).collect())
}

#[cfg(test)]
mod tests {
    use super::tilted_rectangle;

    #[test]
    fn the_outline_of_a_tilted_plane_turns_with_it() {
        let (lo, hi) = ([0.0; 3], [10.0; 3]);
        // The YZ plane at x = 5 as it is: x = 5 everywhere, a tenth larger than the box.
        let flat = tilted_rectangle([1.0, 0.0, 0.0], 5.0, 2, 0.0, lo, hi);
        assert!(flat.iter().all(|p| (p[0] - 5.0).abs() < 1e-12));
        assert!((flat[0][1] + 1.0).abs() < 1e-12 && (flat[2][1] - 11.0).abs() < 1e-12);
        // Turned 90 degrees about z it becomes the XZ plane through (5, 0, 0).
        let turned = tilted_rectangle([1.0, 0.0, 0.0], 5.0, 2, 90.0, lo, hi);
        assert!(turned.iter().all(|p| p[1].abs() < 1e-9));
        // 45 degrees: every corner on x + y = 5.
        let slope = tilted_rectangle([1.0, 0.0, 0.0], 5.0, 2, 45.0, lo, hi);
        assert!(slope.iter().all(|p| (p[0] + p[1] - 5.0).abs() < 1e-9));
    }
}
