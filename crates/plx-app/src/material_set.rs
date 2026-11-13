//! The editor of a materials or cross-sections block of the project schematic: a collection
//! of materials or cross-sections kept in the project, which analyses link item by item or as
//! a whole. It edits them with the dialogs of the FE Model tree and copies them from an
//! analysis, materials also from the material library.

use crate::clicks::TolerantDoubleClick as _;
use std::path::Path;

use plx_mesh::FeMesh;
use plx_model::{FeModel, UnitSystem};

use crate::material_library::{LibraryResult, MaterialLibraryEditor};
use crate::model::Model;
use crate::setup::{Editor, EditorResult, NewItem};
use crate::tree::TreeItem;

/// What a collection block holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Collection {
    Materials,
    CrossSections,
}

impl Collection {
    /// The name of the items, as in the FE Model tree.
    pub fn title(self) -> &'static str {
        match self {
            Self::Materials => "Materials",
            Self::CrossSections => crate::tree::CROSS_SECTIONS,
        }
    }

    fn noun(self) -> &'static str {
        match self {
            Self::Materials => "materials",
            Self::CrossSections => "cross-sections",
        }
    }

    /// The names of its items in a model.
    pub fn names(self, fe: &FeModel) -> Vec<&str> {
        match self {
            Self::Materials => fe.materials.iter().map(|m| m.name.as_str()).collect(),
            Self::CrossSections => fe.cross_sections.iter().map(|c| c.name.as_str()).collect(),
        }
    }

    fn new_item(self) -> NewItem {
        match self {
            Self::Materials => NewItem::Material,
            Self::CrossSections => NewItem::CrossSection,
        }
    }

    fn tree_item(self, index: usize) -> TreeItem {
        match self {
            Self::Materials => TreeItem::Material(index),
            Self::CrossSections => TreeItem::CrossSection(index),
        }
    }

    fn remove(self, fe: &mut FeModel, index: usize) {
        match self {
            Self::Materials => drop(fe.materials.remove(index)),
            Self::CrossSections => drop(fe.cross_sections.remove(index)),
        }
    }

    /// A model with only its items of `fe`.
    pub fn copy(self, fe: &FeModel) -> FeModel {
        let mut copy = FeModel::default();
        self.take(&mut copy, fe);
        copy
    }

    /// Replaces the items of `into` with those of `from`.
    pub fn take(self, into: &mut FeModel, from: &FeModel) {
        match self {
            Self::Materials => into.materials.clone_from(&from.materials),
            Self::CrossSections => into.cross_sections.clone_from(&from.cross_sections),
        }
    }

    /// Copies the items of `from` into `into`; one of the same name is replaced.
    fn import(self, into: &mut FeModel, from: &FeModel) {
        fn merge<T: Clone>(own: &mut Vec<T>, other: &[T], name: impl Fn(&T) -> &str) {
            for item in other {
                match own.iter_mut().find(|o| name(o) == name(item)) {
                    Some(slot) => *slot = item.clone(),
                    None => own.push(item.clone()),
                }
            }
        }
        match self {
            Self::Materials => merge(&mut into.materials, &from.materials, |m| &m.name),
            Self::CrossSections => {
                merge(&mut into.cross_sections, &from.cross_sections, |c| &c.name);
            }
        }
    }
}

pub enum MaterialSetResult {
    Open,
    /// Closed with OK: a model with only the items, stored in SI units, their exposed
    /// parameters, and the unit system they are shown in.
    Ok(Box<FeModel>),
    Cancel,
}

pub struct MaterialSetEditor {
    /// The block edited.
    pub id: u64,
    name: String,
    collection: Collection,
    /// What the dialogs of the FE Model tree edit: a model without a mesh whose setup has
    /// only the items.
    model: Model,
    selected: Option<usize>,
    item: Option<Editor>,
    library: Option<MaterialLibraryEditor>,
    /// The items of the project's analyses, by analysis, to import.
    analyses: Vec<(String, FeModel)>,
    /// The blocks value links give exposed parameters from, by key.
    linked: Vec<(u32, String)>,
}

impl MaterialSetEditor {
    pub fn new(
        id: u64,
        name: &str,
        collection: Collection,
        fe: &FeModel,
        units: UnitSystem,
        analyses: Vec<(String, FeModel)>,
        linked: Vec<(u32, String)>,
    ) -> Self {
        let mut model = Model::new(Path::new(""), FeMesh::default());
        model.fe = collection.copy(fe);
        model.fe.exposed.clone_from(&fe.exposed);
        model.fe.properties.units = units;
        Self {
            id,
            name: name.to_owned(),
            collection,
            selected: (!collection.names(&model.fe).is_empty()).then_some(0),
            model,
            item: None,
            library: None,
            analyses,
            linked,
        }
    }

    fn len(&self) -> usize {
        self.collection.names(&self.model.fe).len()
    }

    pub fn rename(&mut self, name: &str) {
        name.clone_into(&mut self.name);
    }

    fn edit(&mut self, index: usize) {
        self.selected = Some(index);
        let item = self.collection.tree_item(index);
        self.item = Editor::edit(&item, &self.model.fe, &self.model.mesh);
    }

    pub fn show(&mut self, ctx: &egui::Context) -> MaterialSetResult {
        // The dialogs of an item and the library come over this window and take its
        // input while they are open.
        let busy = self.item.is_some() || self.library.is_some();
        let mut result = MaterialSetResult::Open;
        let mut open = true;
        egui::Window::new(format!("{} - {}", self.collection.title(), self.name))
            .id(crate::ids::window(ctx, ("material set", self.id)))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                ui.add_enabled_ui(!busy, |ui| result = self.body(ui));
            });
        if !open {
            result = MaterialSetResult::Cancel;
        }
        if let Some(editor) = &mut self.item {
            editor.set_linked(self.linked.clone());
            match editor.show(ctx, &self.model) {
                EditorResult::Open => {}
                EditorResult::Cancel => self.item = None,
                EditorResult::Ok => {
                    let editor = self.item.take().expect("shown above");
                    let count = self.len();
                    editor.apply(&mut self.model.fe);
                    if self.len() > count {
                        self.selected = Some(count);
                    }
                }
            }
        }
        if let Some(library) = &mut self.library {
            match library.show(ctx) {
                LibraryResult::Open => {}
                LibraryResult::Cancel => self.library = None,
                LibraryResult::Ok(materials) => {
                    self.library = None;
                    if let Some(materials) = materials {
                        self.model.fe.materials = materials;
                        self.selected =
                            self.selected.filter(|&i| i < self.model.fe.materials.len());
                    }
                }
            }
        }
        result
    }

    fn body(&mut self, ui: &mut egui::Ui) -> MaterialSetResult {
        let mut result = MaterialSetResult::Open;
        ui.horizontal(|ui| {
            ui.label("Shown in");
            let units = &mut self.model.fe.properties.units;
            egui::ComboBox::from_id_salt("material set units")
                .selected_text(units.label())
                .width(220.0)
                .show_ui(ui, |ui| {
                    for choice in UnitSystem::ALL {
                        ui.selectable_value(units, choice, choice.label());
                    }
                });
        })
        .response
        .on_hover_text(
            "The unit system the values are shown and typed in; they are stored in SI units, \
             and every analysis that links them shows them in its own.",
        );
        ui.add_space(4.0);
        ui.horizontal_top(|ui| {
            crate::tree_rows::frame().show(ui, |ui| {
                ui.set_min_size(egui::vec2(220.0, 240.0));
                ui.set_max_width(220.0);
                egui::ScrollArea::vertical()
                    .id_salt("material set list")
                    .max_height(240.0)
                    .auto_shrink([false, false])
                    .show(ui, |ui| ui.vertical(|ui| self.list(ui)));
            });
            ui.vertical(|ui| {
                ui.set_width(150.0);
                let wide = |text: &str| egui::Button::new(text).min_size(egui::vec2(150.0, 0.0));
                if ui.add(wide("New …")).clicked() {
                    self.item = Editor::create(self.collection.new_item(), &self.model.fe);
                }
                let selected = self.selected;
                if ui.add_enabled(selected.is_some(), wide("Edit …")).clicked()
                    && let Some(index) = selected
                {
                    self.edit(index);
                }
                if ui.add_enabled(selected.is_some(), wide("Delete")).clicked()
                    && let Some(index) = selected
                {
                    self.collection.remove(&mut self.model.fe, index);
                    let left = self.len();
                    self.selected = (left > 0).then(|| index.min(left - 1));
                }
                ui.separator();
                let tip = "Opens the material library: copy materials from it into this block, \
                           or from this block into the library.";
                if self.collection == Collection::Materials
                    && ui
                        .add(wide("Add from Library …"))
                        .on_hover_text(tip)
                        .clicked()
                {
                    self.library = Some(MaterialLibraryEditor::new(
                        &self.model.fe.materials,
                        self.model.fe.properties.units,
                    ));
                }
                let analyses = std::mem::take(&mut self.analyses);
                let collection = self.collection;
                let count = |fe: &FeModel| collection.names(fe).len();
                let with_items = analyses.iter().any(|(_, fe)| count(fe) > 0);
                ui.add_enabled_ui(with_items, |ui| {
                    ui.menu_button("Import from Analysis", |ui| {
                        for (name, fe) in &analyses {
                            let text = format!("{name} ({})", count(fe));
                            let button = ui.add_enabled(count(fe) > 0, egui::Button::new(text));
                            if button.clicked() {
                                collection.import(&mut self.model.fe, fe);
                                ui.close();
                            }
                        }
                    })
                    .response
                    .on_hover_text(format!(
                        "Copies the {} of an analysis into this block, so that analyses can \
                         link them from here; one of the same name is replaced.",
                        collection.noun()
                    ));
                });
                self.analyses = analyses;
            });
        });
        ui.separator();
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
            if ui.button("Cancel").clicked() {
                result = MaterialSetResult::Cancel;
            }
            if ui.button("OK").clicked() {
                result = MaterialSetResult::Ok(Box::new(self.model.fe.clone()));
            }
        });
        result
    }

    fn list(&mut self, ui: &mut egui::Ui) {
        let names = self.collection.names(&self.model.fe);
        if names.is_empty() {
            ui.weak(match self.collection {
                Collection::Materials => "No materials yet: New … or Add from Library …",
                Collection::CrossSections => "No cross-sections yet: New …",
            });
            return;
        }
        let mut edit = None;
        ui.spacing_mut().item_spacing.y = 1.0;
        for (index, name) in names.into_iter().enumerate() {
            let selected = self.selected == Some(index);
            let text = egui::RichText::new(name);
            let row = crate::tree_rows::tree_row(ui, 0, None, selected, text);
            if row.clicked() {
                self.selected = Some(index);
            }
            if row.double_clicked_tolerant() {
                edit = Some(index);
            }
        }
        if let Some(index) = edit {
            self.edit(index);
        }
    }
}
