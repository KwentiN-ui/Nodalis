//! The expose buttons of the item dialogs, Ansys Workbench's "P" box: a small toggle next to a
//! number that makes it a parameter of the analysis' block in the project schematic. While a
//! value link gives the parameter, the number is read-only in the dialog.
//!
//! A dialog sets up a [`Scope`] around its form; the fields of the form call [`field`], which
//! finds the scope in the egui context, so the forms keep their signatures.

use std::collections::BTreeSet;

use egui::{Response, Ui};
use plx_model::{FeModel, Field, Geometry, ItemRef, ValuePath};

/// What the expose buttons of one dialog know: the item, which of its fields are exposed and
/// which of those a link gives, and the fields toggled while the dialog is open.
#[derive(Clone, Debug, Default)]
pub struct Scope {
    /// The item as it is in the model, for its exposed fields; `None` for a new item.
    pub item: Option<ItemRef>,
    /// Exposed fields of the item, each with the block a value link gives it from.
    exposed: Vec<(Field, Option<String>)>,
    /// Fields whose button was clicked, to be exposed or no longer exposed on OK.
    toggled: BTreeSet<Field>,
    /// The fields the item has as the dialog shows it now; only these get a button.
    pub available: Vec<Field>,
}

fn id() -> egui::Id {
    egui::Id::new("expose scope")
}

impl Scope {
    /// The scope of `item` in `fe`; `linked` names the blocks value links give exposed
    /// parameters from, by key.
    pub fn new(fe: &FeModel, item: Option<ItemRef>, linked: &[(u32, String)]) -> Self {
        let exposed = (fe.exposed.iter())
            .filter(|e| item.as_ref() == Some(&e.path.item))
            .map(|e| {
                let source = (linked.iter())
                    .find(|(key, _)| *key == e.key)
                    .map(|(_, source)| source.clone());
                (e.path.field, source)
            })
            .collect();
        Self {
            item,
            exposed,
            toggled: BTreeSet::new(),
            available: Vec::new(),
        }
    }

    fn is_exposed(&self, field: Field) -> bool {
        self.exposed.iter().any(|(f, _)| *f == field) != self.toggled.contains(&field)
    }

    fn linked(&self, field: Field) -> Option<&str> {
        (self.exposed.iter())
            .find(|(f, _)| *f == field)
            .and_then(|(_, source)| source.as_deref())
    }

    /// Draws `form` with the expose buttons of this scope.
    pub fn show<R>(&mut self, ui: &mut Ui, form: impl FnOnce(&mut Ui) -> R) -> R {
        ui.ctx().data_mut(|d| d.insert_temp(id(), self.clone()));
        let result = form(ui);
        if let Some(scope) = ui.ctx().data_mut(|d| d.remove_temp::<Scope>(id())) {
            *self = scope;
        }
        result
    }

    /// Applies the toggles to the item `item` of `fe` after the dialog's OK: fields clicked
    /// on are exposed, those clicked off are no longer, unless a link gives them. `geometry`
    /// holds the numbers of an item of the mesh setup.
    pub fn apply(mut self, fe: &mut FeModel, geometry: Option<&Geometry>, item: &ItemRef) {
        for field in std::mem::take(&mut self.toggled) {
            let path = ValuePath::new(*item, field);
            let at = fe.exposed.iter().position(|e| e.path == path);
            match at {
                Some(at) if self.linked(field).is_none() => {
                    fe.exposed.remove(at);
                }
                Some(_) => {}
                None if fe.value_in(geometry, &path).is_some() => {
                    fe.expose_in(geometry, path);
                }
                None => {}
            }
        }
    }
}

/// A number of a dialog that can be exposed: `add` draws its field, and the expose button
/// follows it. Outside a [`Scope`], or without an item, it is the field alone.
pub fn field(ui: &mut Ui, field: Field, add: impl FnOnce(&mut Ui) -> Response) -> Response {
    let Some(scope) = ui.ctx().data(|d| d.get_temp::<Scope>(id())) else {
        return add(ui);
    };
    if !scope.available.contains(&field) && !scope.is_exposed(field) {
        return add(ui);
    }
    ui.horizontal(|ui| {
        let linked = scope.linked(field).map(str::to_owned);
        let response = match &linked {
            Some(source) => ui
                .add_enabled_ui(false, add)
                .inner
                .on_disabled_hover_text(format!(
                    "An exposed parameter: its value comes from {source} in the project \
                     window."
                )),
            None => add(ui),
        };
        if button(ui, &scope, field, linked.as_deref()) {
            ui.ctx().data_mut(|d| {
                if let Some(mut scope) = d.get_temp::<Scope>(id()) {
                    scope.toggle(field);
                    d.insert_temp(id(), scope);
                }
            });
        }
        response
    })
    .inner
}

impl Scope {
    fn toggle(&mut self, field: Field) {
        if !self.toggled.remove(&field) {
            self.toggled.insert(field);
        }
    }
}

/// The "P" toggle; returns whether it was clicked.
fn button(ui: &mut Ui, scope: &Scope, field: Field, linked: Option<&str>) -> bool {
    let exposed = scope.is_exposed(field);
    let text = egui::RichText::new("P").small().strong();
    let button = egui::Button::new(text)
        .selected(exposed)
        .min_size(egui::vec2(16.0, 16.0));
    let tip = match (exposed, linked) {
        (_, Some(source)) => format!(
            "Exposed parameter, given by {source}: remove the link in the project window to \
             stop exposing it."
        ),
        (true, None) => "Exposed parameter: it is a row of the analysis' block in the \
                         project window, under Exposed Parameters, where a value link can \
                         set it. Click to stop exposing it."
            .to_owned(),
        (false, None) => "Expose as a parameter: it becomes a row of the analysis' block in \
                          the project window, under Exposed Parameters, where a Parameters \
                          block or a parameter study can set it through a value link."
            .to_owned(),
    };
    ui.add_enabled(linked.is_none(), button)
        .on_hover_text(&tip)
        .on_disabled_hover_text(&tip)
        .clicked()
}
