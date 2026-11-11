//! The Material Library Editor: copies materials between the user's material library
//! and the FE model, and keeps the library in the user's data directory.

use std::path::PathBuf;

use egui::{RichText, Ui, vec2};
use plx_model::convert::Conversion;
use plx_model::library::{LibraryNode, LibraryPath, ROOT_NAME, name_for_model};
use plx_model::{Material, MaterialLibrary, Quantity, UnitSystem};

use crate::icons::{self, Icon};
use crate::tree_rows::{frame, tree_row};

/// Name of the library file in nodalis's data directory.
const FILE_NAME: &str = "materials.ron";

/// The library file: in the directory where the settings are kept (see
/// [`crate::portable::data_dir`]).
pub fn library_file() -> Option<PathBuf> {
    crate::portable::data_dir().map(|dir| dir.join(FILE_NAME))
}

/// Which list was used last; the move buttons and the preview follow it.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Side {
    Library,
    Model,
}

pub enum LibraryResult {
    Open,
    /// Closed with OK: the model's new materials, if they changed.
    Ok(Option<Vec<Material>>),
    Cancel,
}

pub struct MaterialLibraryEditor {
    library: MaterialLibrary,
    /// Where the library is saved; `None` when it could not be read and must not be
    /// overwritten.
    file: Option<PathBuf>,
    /// Why the library could not be read.
    load_error: Option<String>,
    library_changed: bool,
    materials: Vec<Material>,
    /// Unit system of the model's materials.
    units: UnitSystem,
    materials_changed: bool,
    /// Selected node of the library tree; the empty path is the root.
    library_selected: Option<LibraryPath>,
    material_selected: Option<usize>,
    side: Side,
    /// Text of the rename field.
    name: String,
    preview: bool,
    /// Material model selected in the preview, by its label.
    preview_model: &'static str,
    error: Option<String>,
    /// Ask whether to save library changes before closing.
    confirm_close: bool,
}

impl MaterialLibraryEditor {
    pub fn new(materials: &[Material], units: UnitSystem) -> Self {
        let file = library_file();
        let (library, file, load_error) = match file.as_deref().map(plx_io::library::read_library) {
            Some(Ok(library)) => (library, file, None),
            Some(Err(error)) => (MaterialLibrary::default(), None, Some(error.to_string())),
            None => (
                MaterialLibrary::default(),
                None,
                Some("No user data directory found.".into()),
            ),
        };
        let mut editor = Self {
            library,
            file,
            load_error,
            library_changed: false,
            materials: materials.to_vec(),
            units,
            materials_changed: false,
            library_selected: None,
            material_selected: None,
            side: Side::Library,
            name: String::new(),
            preview: false,
            preview_model: DENSITY,
            error: None,
            confirm_close: false,
        };
        // Nodalis selects the first model material and the first library material.
        let first = editor.library.first_material().unwrap_or_default();
        editor.select_library(first);
        if !editor.materials.is_empty() {
            editor.material_selected = Some(0);
            editor.side = Side::Model;
        }
        editor
    }

    fn select_library(&mut self, path: LibraryPath) {
        self.name = match self.library.node(&path) {
            Some(node) => node.name().to_string(),
            None => ROOT_NAME.to_string(),
        };
        self.library_selected = Some(path);
        self.side = Side::Library;
    }

    fn selected_library_material(&self) -> Option<&Material> {
        self.library.material(self.library_selected.as_deref()?)
    }

    /// The material shown in the preview, the one selected in the list used last, with its
    /// values in the units they are shown in: those of the library, or of the model, whose
    /// materials are stored in SI units.
    fn previewed(&self) -> Option<(Material, UnitSystem)> {
        match self.side {
            Side::Library => Some((
                self.selected_library_material()?.clone(),
                self.library.units,
            )),
            Side::Model => {
                let mut material = self.materials.get(self.material_selected?)?.clone();
                material.convert_units(&Conversion::new(plx_model::STORED, self.units));
                Some((material, self.units))
            }
        }
    }

    fn copy_to_model(&mut self) {
        let Some(material) = self.selected_library_material() else {
            self.error = Some("Please select a library material to copy to the model.".into());
            return;
        };
        let mut material = material.clone();
        // A material of its own, also when the same one is copied again.
        material.id = plx_model::Id::new();
        material.convert_units(&Conversion::new(self.library.units, plx_model::STORED));
        material.name = name_for_model(
            &material.name,
            self.materials.iter().map(|m| m.name.as_str()),
        );
        self.materials.push(material);
        self.material_selected = Some(self.materials.len() - 1);
        self.side = Side::Model;
        self.materials_changed = true;
    }

    fn copy_to_library(&mut self) {
        let (Some(index), Some(selected)) = (self.material_selected, &self.library_selected) else {
            self.error = Some("Please select a library category to copy the material to.".into());
            return;
        };
        let mut material = self.materials[index].clone();
        material.convert_units(&Conversion::new(plx_model::STORED, self.library.units));
        if let Some(path) = self.library.add_material(selected, &material) {
            self.library_changed = true;
            self.select_library(path);
        }
    }

    fn add_category(&mut self) {
        let selected = self.library_selected.clone().unwrap_or_default();
        if let Some(path) = self.library.add_category(&selected) {
            self.library_changed = true;
            self.select_library(path);
        }
    }

    fn delete_from_library(&mut self) {
        if let Some(path) = &self.library_selected
            && let Some(next) = self.library.delete(path)
        {
            self.library_changed = true;
            self.select_library(next);
        }
    }

    fn rename(&mut self) {
        let Some(path) = self.library_selected.clone() else {
            return;
        };
        if self
            .library
            .node(&path)
            .is_some_and(|n| n.name() == self.name)
        {
            return;
        }
        match self.library.rename(&path, &self.name) {
            Ok(()) => self.library_changed = true,
            Err(error) => self.error = Some(error),
        }
    }

    fn delete_from_model(&mut self) {
        let Some(index) = self.material_selected else {
            return;
        };
        self.materials.remove(index);
        self.materials_changed = true;
        self.material_selected = if self.materials.is_empty() {
            None
        } else {
            Some(index.min(self.materials.len() - 1))
        };
    }

    fn can_move(&self, up: bool) -> bool {
        match self.side {
            Side::Library => (self.library_selected.as_ref())
                .is_some_and(|path| self.library.move_target(path, up).is_some()),
            Side::Model => self.material_selected.is_some_and(|i| {
                if up {
                    i > 0
                } else {
                    i + 1 < self.materials.len()
                }
            }),
        }
    }

    fn move_selected(&mut self, up: bool) {
        match self.side {
            Side::Library => {
                if let Some(path) = &self.library_selected
                    && let Some(moved) = self.library.move_material(path, up)
                {
                    self.library_changed = true;
                    self.library_selected = Some(moved);
                }
            }
            Side::Model => {
                if let Some(i) = self.material_selected
                    && self.can_move(up)
                {
                    let j = if up { i - 1 } else { i + 1 };
                    self.materials.swap(i, j);
                    self.material_selected = Some(j);
                    self.materials_changed = true;
                }
            }
        }
    }

    fn save_library(&mut self) -> bool {
        let Some(file) = &self.file else {
            self.error = Some("The library cannot be saved.".into());
            return false;
        };
        match plx_io::library::save_library(file, &self.library) {
            Ok(()) => {
                self.library_changed = false;
                true
            }
            Err(error) => {
                self.error = Some(format!("Saving failed: {error}"));
                false
            }
        }
    }

    /// OK: saves the library and hands the model's materials back.
    fn accept(&mut self) -> LibraryResult {
        if self.library_changed && !self.save_library() {
            return LibraryResult::Open;
        }
        let materials = self.materials_changed.then(|| self.materials.clone());
        LibraryResult::Ok(materials)
    }

    /// Cancel or the window's close button: asks first when the library changed.
    fn cancel(&mut self) -> LibraryResult {
        if self.library_changed && self.file.is_some() {
            self.confirm_close = true;
            LibraryResult::Open
        } else {
            LibraryResult::Cancel
        }
    }

    pub fn show(&mut self, ctx: &egui::Context) -> LibraryResult {
        let mut open = true;
        let mut result = LibraryResult::Open;
        let size = vec2(600.0, 560.0);
        let window = egui::Window::new("Material Library")
            .id(crate::ids::window(ctx, "material library"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .fixed_size(size)
            // Centred together with the preview on its right.
            .default_pos(ctx.content_rect().center() - vec2(505.0, 300.0))
            .show(ctx, |ui| {
                self.libraries_section(ui);
                ui.add_space(4.0);
                let body = size.y - 70.0;
                // Height inside a group box, below its title.
                let inner = body - 34.0;
                ui.horizontal_top(|ui| {
                    // Two group boxes with their frames and the button column between them.
                    let column = (size.x - 80.0) / 2.0;
                    group(ui, "Library Materials", vec2(column, body), |ui| {
                        self.library_side(ui, inner)
                    });
                    ui.vertical(|ui| {
                        ui.add_space(52.0);
                        self.copy_buttons(ui);
                    });
                    group(ui, "FE Model Materials", vec2(column, body), |ui| {
                        self.model_side(ui, inner)
                    });
                });
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.preview, "Preview material properties");
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("OK").clicked() {
                            result = self.accept();
                        }
                        if ui.button("Cancel").clicked() {
                            result = self.cancel();
                        }
                    });
                });
            });
        if !open {
            result = self.cancel();
        }
        if self.preview
            && let Some(window) = &window
        {
            let at = window.response.rect.right_top() + vec2(10.0, 0.0);
            self.preview_window(ctx, at);
        }
        if let Some(error) = self.error.clone() {
            egui::Modal::new(crate::ids::window(ctx, "material library error")).show(ctx, |ui| {
                ui.set_max_width(360.0);
                ui.label(error);
                ui.add_space(6.0);
                if ui.button("OK").clicked() {
                    self.error = None;
                }
            });
        }
        if self.confirm_close {
            let question = "Discard the changes to the material library?";
            let consequence = "OK in the library window saves them.";
            match crate::dialog::confirm(
                ctx,
                "material library close",
                question,
                consequence,
                "Discard",
            ) {
                Some(true) => result = LibraryResult::Cancel,
                Some(false) => self.confirm_close = false,
                None => {}
            }
        }
        result
    }

    /// The collapsed "Libraries" box: the one library file of the user.
    fn libraries_section(&mut self, ui: &mut Ui) {
        egui::CollapsingHeader::new("Libraries")
            .default_open(false)
            .show(ui, |ui| {
                match &self.file {
                    Some(file) => {
                        ui.label("User library:");
                        ui.add(
                            egui::Label::new(RichText::new(file.display().to_string()).monospace())
                                .truncate(),
                        );
                    }
                    None => {
                        ui.label("The library is not saved.");
                    }
                }
                if let Some(error) = &self.load_error {
                    ui.colored_label(crate::style::error(), error);
                }
            });
    }

    fn library_side(&mut self, ui: &mut Ui, height: f32) {
        let selected = self.library_selected.clone();
        let is_category = selected
            .as_deref()
            .is_some_and(|p| self.library.category(p).is_some());
        let is_node = selected.as_deref().is_some_and(|p| !p.is_empty());
        button_row(ui, |ui| {
            if ui
                .add_enabled(is_category, egui::Button::new("Add Category"))
                .clicked()
            {
                self.add_category();
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui
                    .add_enabled(is_node, egui::Button::new("Delete"))
                    .clicked()
                {
                    self.delete_from_library();
                }
            });
        });
        let tree_height = height - 2.0 * ROW - 10.0;
        let mut clicked = None;
        let mut double_clicked = false;
        frame().show(ui, |ui| {
            ui.set_min_size(vec2(ui.available_width(), tree_height));
            egui::ScrollArea::both()
                .id_salt("library tree")
                .max_height(tree_height)
                .auto_shrink(false)
                .show(ui, |ui| {
                    ui.spacing_mut().item_spacing.y = 1.0;
                    let root = &mut self.library.root;
                    let response = tree_row(
                        ui,
                        0,
                        Some(&mut root.expanded),
                        selected.as_deref() == Some(&[]),
                        category_label(ROOT_NAME, selected.as_deref() == Some(&[])),
                    );
                    if response.clicked() {
                        clicked = Some(Vec::new());
                    }
                    if root.expanded {
                        library_tree(
                            ui,
                            &mut root.items,
                            &mut Vec::new(),
                            selected.as_deref(),
                            &mut clicked,
                            &mut double_clicked,
                        );
                    }
                });
        });
        if let Some(path) = clicked {
            self.select_library(path);
            if double_clicked && self.selected_library_material().is_some() {
                self.copy_to_model();
            }
        }
        button_row(ui, |ui| {
            let field = ui.add_enabled(
                is_node,
                egui::TextEdit::singleline(&mut self.name)
                    .desired_width(ui.available_width() - 90.0),
            );
            let enter = field.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if ui
                .add_enabled(is_node, egui::Button::new("Rename"))
                .clicked()
                || enter
            {
                self.rename();
            }
        });
    }

    fn copy_buttons(&mut self, ui: &mut Ui) {
        let to_model = self.selected_library_material().is_some();
        if icons::dialog_button(ui, Icon::Arrow(vec2(1.0, 0.0)), "Copy to Model", to_model)
            .clicked()
        {
            self.copy_to_model();
        }
        let to_library = self.material_selected.is_some() && self.library_selected.is_some();
        if icons::dialog_button(
            ui,
            Icon::Arrow(vec2(-1.0, 0.0)),
            "Copy to Library",
            to_library,
        )
        .clicked()
        {
            self.copy_to_library();
        }
        if icons::dialog_button(
            ui,
            Icon::Arrow(vec2(0.0, -1.0)),
            "Move Up",
            self.can_move(true),
        )
        .clicked()
        {
            self.move_selected(true);
        }
        if icons::dialog_button(
            ui,
            Icon::Arrow(vec2(0.0, 1.0)),
            "Move Down",
            self.can_move(false),
        )
        .clicked()
        {
            self.move_selected(false);
        }
    }

    fn model_side(&mut self, ui: &mut Ui, height: f32) {
        button_row(ui, |ui| {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui
                    .add_enabled(
                        self.material_selected.is_some(),
                        egui::Button::new("Delete"),
                    )
                    .clicked()
                {
                    self.delete_from_model();
                }
            });
        });
        let list_height = height - ROW - 6.0;
        let mut clicked = None;
        let mut double_clicked = false;
        frame().show(ui, |ui| {
            ui.set_min_size(vec2(ui.available_width(), list_height));
            egui::ScrollArea::both()
                .id_salt("model materials")
                .max_height(list_height)
                .auto_shrink(false)
                .show(ui, |ui| {
                    ui.spacing_mut().item_spacing.y = 1.0;
                    for (i, material) in self.materials.iter().enumerate() {
                        let selected = self.material_selected == Some(i);
                        let response =
                            tree_row(ui, 0, None, selected, RichText::new(&material.name));
                        if response.clicked() || response.double_clicked() {
                            clicked = Some(i);
                            double_clicked |= response.double_clicked();
                        }
                    }
                });
        });
        if let Some(i) = clicked {
            self.material_selected = Some(i);
            self.side = Side::Model;
            if double_clicked {
                self.copy_to_library();
            }
        }
    }

    /// The "Preview Material Properties": the material models of the selected
    /// material and their values, read only.
    fn preview_window(&mut self, ctx: &egui::Context, at: egui::Pos2) {
        let mut open = true;
        let material = self.previewed();
        egui::Window::new("Material Properties")
            .id(crate::ids::window(ctx, "material properties"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .fixed_size(vec2(400.0, 470.0))
            // Follows the editor, so that it never covers it.
            .current_pos(at)
            .show(ctx, |ui| {
                let Some((material, units)) = material else {
                    ui.weak("No material selected.");
                    return;
                };
                ui.strong("Data");
                egui::Grid::new("preview data")
                    .num_columns(2)
                    .show(ui, |ui| {
                        ui.label("Material name");
                        let mut name = material.name.clone();
                        ui.add(
                            egui::TextEdit::singleline(&mut name)
                                .interactive(false)
                                .desired_width(280.0),
                        );
                        ui.end_row();
                    });
                ui.add_space(6.0);
                ui.strong("Material Models");
                let models = material_models(&material, units);
                if !models.iter().any(|(label, _)| *label == self.preview_model)
                    && let Some((first, _)) = models.first()
                {
                    self.preview_model = first;
                }
                frame().show(ui, |ui| {
                    ui.set_min_size(vec2(ui.available_width(), 120.0));
                    for (label, _) in &models {
                        let selected = self.preview_model == *label;
                        if tree_row(ui, 0, None, selected, RichText::new(*label)).clicked() {
                            self.preview_model = label;
                        }
                    }
                });
                ui.add_space(6.0);
                ui.strong("Properties");
                frame().show(ui, |ui| {
                    ui.set_min_size(vec2(ui.available_width(), 150.0));
                    let rows = models
                        .into_iter()
                        .find(|(label, _)| *label == self.preview_model)
                        .map(|(_, rows)| rows)
                        .unwrap_or_default();
                    egui::Grid::new("preview properties")
                        .num_columns(2)
                        .striped(true)
                        .min_col_width(150.0)
                        .show(ui, |ui| {
                            for (name, value) in rows {
                                ui.label(name);
                                ui.label(value);
                                ui.end_row();
                            }
                        });
                });
                ui.add_space(4.0);
                ui.weak(format!("Unit system: {}", units.label()));
            });
        if !open {
            self.preview = false;
        }
    }
}

const DENSITY: &str = "Density";
const ELASTIC: &str = "Elastic";

/// Material models with their property rows (name, value with unit).
fn material_models(
    material: &Material,
    units: UnitSystem,
) -> Vec<(&'static str, Vec<(&'static str, String)>)> {
    let with_unit = |value, quantity| {
        let unit = units.unit(quantity);
        let value = format_value(value);
        if unit.is_empty() {
            value
        } else {
            format!("{value} {unit}")
        }
    };
    let mut models = Vec::new();
    if let Some(density) = material.density {
        models.push((
            DENSITY,
            vec![("Density", with_unit(density, Quantity::Density))],
        ));
    }
    if let Some(elastic) = material.elastic {
        models.push((
            ELASTIC,
            vec![
                (
                    "Young's modulus",
                    with_unit(elastic.young, Quantity::Pressure),
                ),
                ("Poisson's ratio", format_value(elastic.poisson)),
            ],
        ));
    }
    if let Some(hyper) = &material.hyperelastic {
        let mut rows = vec![("Model", hyper.model.label().to_string())];
        for (parameter, value) in hyper.model.parameters().iter().zip(&hyper.constants) {
            rows.push((
                parameter.name,
                match parameter.kind {
                    plx_model::hyperelastic::ParameterKind::Stress => {
                        with_unit(*value, Quantity::Pressure)
                    }
                    plx_model::hyperelastic::ParameterKind::Number => format_value(*value),
                },
            ));
        }
        rows.push(("Poisson's ratio", format_value(hyper.poisson)));
        models.push(("Hyperelastic", rows));
    }
    if let Some(plastic) = &material.plastic {
        let mut rows = vec![("Hardening", plastic.hardening.keyword().to_string())];
        let unit = units.unit(Quantity::Pressure);
        for point in &plastic.points {
            rows.push((
                "Stress, strain, temperature",
                format!(
                    "{} {unit}, {}, {}",
                    format_value(point.stress),
                    format_value(point.plastic_strain),
                    format_value(point.temperature)
                ),
            ));
        }
        models.push(("Plastic", rows));
    }
    if let Some(creep) = material.creep {
        models.push((
            "Creep",
            vec![
                ("Law", "Norton".to_string()),
                ("A", format_value(creep.a)),
                ("Stress exponent n", format_value(creep.n)),
                ("Time exponent m", format_value(creep.m)),
            ],
        ));
    }
    if let Some(expansion) = material.expansion {
        models.push((
            "Thermal Expansion",
            vec![
                (
                    "Thermal expansion coefficient",
                    with_unit(expansion.coefficient, Quantity::ThermalExpansion),
                ),
                (
                    "Zero temperature",
                    with_unit(expansion.zero_temperature, Quantity::Temperature),
                ),
            ],
        ));
    }
    if let Some(conductivity) = material.conductivity {
        models.push((
            "Thermal Conductivity",
            vec![(
                "Thermal conductivity",
                with_unit(conductivity, Quantity::ThermalConductivity),
            )],
        ));
    }
    if let Some(specific_heat) = material.specific_heat {
        models.push((
            "Specific Heat",
            vec![(
                "Specific heat",
                with_unit(specific_heat, Quantity::SpecificHeat),
            )],
        ));
    }
    models
}

/// Number as property grids show it: very small or large values in E notation.
fn format_value(value: f64) -> String {
    if value != 0.0 && !(1e-3..1e7).contains(&value.abs()) {
        format!("{value:.2E}")
    } else {
        format!("{value}")
    }
}

/// Categories are blue in the library tree.
fn category_label(name: &str, selected: bool) -> RichText {
    let text = RichText::new(name);
    if selected {
        text
    } else {
        text.color(crate::style::highlight())
    }
}

fn library_tree(
    ui: &mut Ui,
    items: &mut [LibraryNode],
    path: &mut LibraryPath,
    selected: Option<&[usize]>,
    clicked: &mut Option<LibraryPath>,
    double_clicked: &mut bool,
) {
    for (index, node) in items.iter_mut().enumerate() {
        path.push(index);
        let is_selected = selected == Some(path.as_slice());
        let response = match node {
            LibraryNode::Category(category) => {
                let label = category_label(&category.name, is_selected);
                let expandable = !category.items.is_empty();
                let expanded = expandable.then_some(&mut category.expanded);
                tree_row(ui, path.len(), expanded, is_selected, label)
            }
            LibraryNode::Material(material) => tree_row(
                ui,
                path.len(),
                None,
                is_selected,
                RichText::new(&material.name),
            ),
        };
        if response.clicked() || response.double_clicked() {
            *clicked = Some(path.clone());
            *double_clicked |= response.double_clicked();
        }
        if let LibraryNode::Category(category) = node
            && category.expanded
        {
            library_tree(
                ui,
                &mut category.items,
                path,
                selected,
                clicked,
                double_clicked,
            );
        }
        path.pop();
    }
}

/// Group box with a title.
fn group(ui: &mut Ui, title: &str, size: egui::Vec2, body: impl FnOnce(&mut Ui)) {
    ui.allocate_ui_with_layout(size, egui::Layout::top_down(egui::Align::Min), |ui| {
        ui.set_width(size.x);
        ui.label(title);
        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.set_width(ui.available_width());
            body(ui);
        });
    });
}

/// Height of a row of buttons above or below a list.
const ROW: f32 = 26.0;

/// Row of fixed height, laid out left to right.
fn button_row(ui: &mut Ui, body: impl FnOnce(&mut Ui)) {
    let layout = egui::Layout::left_to_right(egui::Align::Center);
    ui.allocate_ui_with_layout(vec2(ui.available_width(), ROW), layout, |ui| {
        ui.set_height(ROW);
        body(ui);
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn editor() -> MaterialLibraryEditor {
        editor_in(UnitSystem::MmTonSC)
    }

    fn editor_in(units: UnitSystem) -> MaterialLibraryEditor {
        let mut editor = MaterialLibraryEditor::new(&[], units);
        // Tests never touch the user's own library file.
        editor.library = MaterialLibrary::default();
        editor.file = None;
        editor.select_library(editor.library.first_material().unwrap());
        editor
    }

    #[test]
    fn library_materials_are_copied_into_the_model_with_free_names() {
        let mut editor = editor();
        editor.copy_to_model();
        editor.copy_to_model();
        let names: Vec<&str> = editor.materials.iter().map(|m| m.name.as_str()).collect();
        assert_eq!(names, ["Generic Steel", "Generic Steel_Library-1"]);
        assert_eq!(editor.material_selected, Some(1));
        let LibraryResult::Ok(Some(materials)) = editor.accept() else {
            panic!("the model materials changed");
        };
        // Stored in SI units.
        assert!((materials[0].elastic.unwrap().young - 2.1e11).abs() < 1.0);
    }

    #[test]
    fn model_materials_are_copied_into_the_selected_category() {
        let mut editor = editor();
        editor.materials.push(Material {
            name: "Alu".into(),
            density: Some(2.7e-9),
            elastic: None,
            ..Default::default()
        });
        editor.material_selected = Some(0);
        editor.select_library(Vec::new());
        editor.copy_to_library();
        assert_eq!(editor.library_selected, Some(vec![2]));
        assert_eq!(editor.name, "Alu");
        assert!(editor.library_changed);
        // Without a file the changes cannot be saved, so OK keeps the dialog open.
        assert!(matches!(editor.accept(), LibraryResult::Open));
        assert!(editor.error.is_some());
    }

    #[test]
    fn preview_lists_the_material_models() {
        let editor = editor();
        let (material, units) = editor.previewed().unwrap();
        let models = material_models(&material, units);
        let labels: Vec<&str> = models.iter().map(|(label, _)| *label).collect();
        assert_eq!(
            labels,
            [
                DENSITY,
                ELASTIC,
                "Thermal Expansion",
                "Thermal Conductivity",
                "Specific Heat"
            ]
        );
        assert_eq!(models[0].1[0].1, "7.85E-9 t/mm³");
        assert_eq!(models[1].1[0].1, "210000 MPa");
    }

    #[test]
    fn materials_are_converted_between_library_and_model_units() {
        let mut editor = editor_in(UnitSystem::MKgSC);
        editor.copy_to_model();
        let steel = &editor.materials[0];
        assert!((steel.density.unwrap() - 7850.0).abs() < 1e-9);
        assert!((steel.elastic.unwrap().young - 2.1e11).abs() < 1.0);
        let (_, units) = editor.previewed().unwrap();
        assert_eq!(units, UnitSystem::MKgSC);
        // Back in the library, the values are in its units again.
        editor.select_library(vec![0, 0, 0]);
        editor.side = Side::Model;
        editor.copy_to_library();
        let copy = editor.selected_library_material().unwrap();
        assert!((copy.elastic.unwrap().young - 210_000.0).abs() < 1e-6);
    }

    #[test]
    fn model_materials_are_stored_in_si_and_shown_in_the_model_units() {
        let mut editor = editor_in(UnitSystem::MmTonSC);
        editor.copy_to_model();
        assert!((editor.materials[0].elastic.unwrap().young - 2.1e11).abs() < 1.0);
        let (material, units) = editor.previewed().unwrap();
        assert_eq!(material_models(&material, units)[1].1[0].1, "210000 MPa");
    }
}
