//! The distribution dialog: a factor over the model by an equation in the coordinates of a
//! coordinate system and named variables, or by points read from a text file; and the
//! distribution choice of the pressure and temperature dialogs.

use egui::Ui;
use plx_model::{
    Distribution, DistributionKind, FeModel, Field, Interpolation, Quantity, parse_points,
};

use crate::model::Model;

/// State of the dialog besides the distribution itself.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DistributionView {
    /// Why the last file could not be read.
    import_error: Option<String>,
    /// The factors at the mesh nodes, for the distribution they were found for.
    range: Option<(Distribution, FactorRange)>,
    /// The factors are not shown in colours over the mesh.
    hide_colors: bool,
    /// Where the text cursor of the equation was last, in characters: what the help
    /// inserts goes there.
    cursor: Option<usize>,
}

/// What a click in the help puts into the equation.
enum Insert {
    /// Text at the cursor.
    Text(String),
    /// A whole new equation.
    Whole(String),
}

/// Moves the cursor of the text field `id` to the character `at` and gives it the keyboard.
fn place_cursor(ui: &Ui, id: egui::Id, at: usize) {
    if let Some(mut state) = egui::TextEdit::load_state(ui.ctx(), id) {
        let cursor = egui::text::CCursorRange::one(egui::text::CCursor::new(at));
        state.cursor.set_char_range(Some(cursor));
        state.store(ui.ctx(), id);
    }
    ui.ctx().memory_mut(|m| m.request_focus(id));
}

/// The factor over the mesh, or why there is none, in red; and whether the mesh shows it in
/// colours.
fn range_label(ui: &mut Ui, view: &mut DistributionView) {
    ui.vertical(|ui| {
        range_text(ui, view);
        if matches!(view.range, Some((_, Ok(_)))) {
            let mut show = !view.hide_colors;
            ui.checkbox(&mut show, "Show in colors on the mesh")
                .on_hover_text(
                    "Colours the mesh by the factor at the nodes while the dialog is open, \
                     with a legend in the 3D view.",
                );
            view.hide_colors = !show;
        }
    });
}

fn range_text(ui: &mut Ui, view: &DistributionView) {
    match view.range.as_ref().map(|(_, range)| range) {
        Some(Ok(Factors { min, max, .. })) if min == max => {
            ui.label(format!("Factor over the mesh: {}", short(*min)))
        }
        Some(Ok(Factors { min, max, .. })) => ui.label(format!(
            "Factor over the mesh: {} to {}",
            short(*min),
            short(*max)
        )),
        Some(Err(problem)) => ui.colored_label(crate::style::error(), problem),
        None => ui.label(""),
    };
}

/// The help on writing the equation: the names it can use, its operators and functions,
/// and examples. Returns what a click on one of them puts into the equation.
fn equation_help(
    ui: &mut Ui,
    fe: &FeModel,
    system: Option<plx_model::Id>,
    variables: &[plx_model::DistributionVariable],
    length: &str,
) -> Option<Insert> {
    let mut insert = None;
    let frame = fe.coordinate_system_or_global(system);
    let cylindrical =
        (frame.as_ref()).is_some_and(|f| f.kind == plx_model::CoordinateSystemKind::Cylindrical);
    let system_name = match system {
        None => "the global system".to_owned(),
        Some(_) => fe.system_name(system),
    };
    let code = |ui: &mut Ui, text: &str, tip: &str| {
        let button = egui::Button::new(egui::RichText::new(text).monospace()).small();
        ui.add(button).on_hover_text(tip).clicked()
    };
    ui.vertical(|ui| {
        weak(
            ui,
            "Click a name, function or example to put it into the equation.",
        );
        egui::CollapsingHeader::new("Names")
            .id_salt("distribution help names")
            .default_open(true)
            .show(ui, |ui| {
                let kind = if cylindrical {
                    "cylindrical"
                } else {
                    "rectangular"
                };
                weak(ui, &format!("Coordinates in {system_name} ({kind}):"));
                egui::Grid::new("distribution names")
                    .num_columns(2)
                    .spacing([10.0, 3.0])
                    .show(ui, |ui| {
                        let mut names: Vec<(String, String)> = vec![
                            ("x".into(), format!("Along its x axis, in {length}")),
                            ("y".into(), format!("Along its y axis, in {length}")),
                            ("z".into(), format!("Along its z axis, in {length}")),
                            ("r".into(), format!("Distance from its z axis, in {length}")),
                            (
                                "theta".into(),
                                "Angle about its z axis from its x axis, in radians, -pi to pi"
                                    .into(),
                            ),
                            ("pi".into(), "3.14159...".into()),
                        ];
                        if cylindrical {
                            // A cylindrical system's own coordinates r, theta, z first.
                            names[..5].rotate_left(3);
                            names[2..5].rotate_right(1);
                        }
                        names.extend(variables.iter().map(|v| {
                            (v.name.clone(), format!("Variable, now {}", short(v.value)))
                        }));
                        for (name, meaning) in names {
                            if code(ui, &name, "Puts it into the equation") {
                                insert = Some(Insert::Text(name.clone()));
                            }
                            ui.label(meaning);
                            ui.end_row();
                        }
                    });
            });
        egui::CollapsingHeader::new("Operators")
            .id_salt("distribution help operators")
            .show(ui, |ui| {
                egui::Grid::new("distribution operators")
                    .num_columns(2)
                    .spacing([10.0, 3.0])
                    .show(ui, |ui| {
                        for (operator, meaning) in [
                            ("+  -  *  /", "Arithmetic"),
                            ("^", "Power: x^2, also x**2"),
                            ("%", "Remainder: 7 % 3 is 1"),
                            ("<  <=  >  >=", "Comparisons, 1 where true, 0 where not"),
                            ("==  !=", "Equal, not equal"),
                            ("and  or  not", "Combine conditions, also && || !"),
                            ("c ? a : b", "a where c holds, else b, like If(c, a, b)"),
                        ] {
                            ui.monospace(operator);
                            ui.label(meaning);
                            ui.end_row();
                        }
                    });
            });
        egui::CollapsingHeader::new("Functions")
            .id_salt("distribution help functions")
            .show(ui, |ui| {
                egui::Grid::new("distribution functions")
                    .num_columns(2)
                    .spacing([10.0, 3.0])
                    .striped(true)
                    .show(ui, |ui| {
                        for (signature, meaning) in plx_model::equation::FUNCTIONS {
                            if code(ui, signature, "Puts it into the equation") {
                                let name = &signature[..signature.find('(').unwrap_or(0)];
                                insert = Some(Insert::Text(format!("{name}(")));
                            }
                            ui.label(meaning);
                            ui.end_row();
                        }
                    });
            });
        egui::CollapsingHeader::new("Examples")
            .id_salt("distribution help examples")
            .show(ui, |ui| {
                egui::Grid::new("distribution examples")
                    .num_columns(2)
                    .spacing([10.0, 3.0])
                    .show(ui, |ui| {
                        for (example, meaning) in EXAMPLES {
                            if code(ui, example, "Replaces the equation with it") {
                                insert = Some(Insert::Whole(example.to_owned()));
                            }
                            ui.label(meaning.replace("{length}", length));
                            ui.end_row();
                        }
                    });
            });
    });
    insert
}

/// Equations to start from, with what they give.
const EXAMPLES: [(&str, &str); 7] = [
    (
        "1 - z/200",
        "Falls linearly from 1 at z = 0 to 0 at z = 200 {length}",
    ),
    ("Max(0, 1 - z/200)", "The same, but no lower than 0"),
    ("If(x > 0, 1, 0)", "1 where x is positive, 0 elsewhere"),
    (
        "If(r < 20, 1, 0.5)",
        "1 within 20 {length} of the z axis, 0.5 outside",
    ),
    (
        "1 - (r/50)^2",
        "Parabola over a circle of radius 50 {length}, 1 at its center",
    ),
    (
        "Max(0, Cos(theta))",
        "Bearing load: largest at theta = 0, none on the far side",
    ),
    ("Exp(-(r/10)^2)", "Bell about the z axis, 10 {length} wide"),
];

/// The factors at the mesh nodes, or why there are none.
type FactorRange = Result<Factors, String>;

/// The factors of a distribution at the mesh nodes.
#[derive(Clone, Debug, PartialEq)]
struct Factors {
    min: f64,
    max: f64,
    /// The factor at each node, by node index.
    at_nodes: std::sync::Arc<Vec<f32>>,
}

impl DistributionView {
    /// The factors at the nodes, to colour the mesh by, while the dialog shows them.
    pub fn preview(
        &self,
        distribution: &Distribution,
        model: &Model,
    ) -> Option<crate::model::DistributionPreview> {
        if self.hide_colors {
            return None;
        }
        let (found, Ok(factors)) = self.range.as_ref()? else {
            return None;
        };
        // Factors of a mesh that changed since do not fit it.
        (same_factors(found, distribution) && factors.at_nodes.len() == model.mesh.coords().len())
            .then(|| crate::model::DistributionPreview {
                name: distribution.name.clone(),
                factors: factors.at_nodes.clone(),
            })
    }
}

/// The rows of the dialog, in the item form's grid.
pub fn distribution_rows(
    ui: &mut Ui,
    distribution: &mut Distribution,
    view: &mut DistributionView,
    model: &Model,
) {
    let units = model.fe.properties.units;
    let length = units.unit(Quantity::Length);
    ui.label("Type");
    let by_equation = matches!(distribution.kind, DistributionKind::Equation(_));
    crate::layout::wrapped_row(ui, |ui| {
        if ui.radio(by_equation, "Equation").clicked() && !by_equation {
            distribution.kind = DistributionKind::Equation("1".into());
        }
        if ui.radio(!by_equation, "Points from file").clicked() && by_equation {
            distribution.kind = DistributionKind::Points {
                source: String::new(),
                points: Vec::new(),
                interpolation: Interpolation::default(),
            };
        }
    });
    ui.end_row();
    // The factor over the mesh, found again when the distribution changes.
    if (view.range.as_ref()).is_none_or(|(found, _)| !same_factors(found, distribution)) {
        let range = factor_range(distribution, model);
        view.range = Some((distribution.clone(), range));
    }
    let fe = &model.fe;
    ui.label("Coordinate system");
    crate::pick_item::system_combo(
        ui,
        "distribution system",
        &mut distribution.system,
        fe,
        200.0,
    );
    ui.end_row();
    if fe.system_attachment(distribution.system).is_some() {
        ui.label("");
        ui.colored_label(
            crate::style::lift(crate::features::MOVING_COLOR),
            "Moves with a selection: the distribution takes the system where it is in the \
             undeformed model.",
        );
        ui.end_row();
    }
    match &mut distribution.kind {
        DistributionKind::Equation(text) => {
            ui.label("Factor =");
            let edit = egui::TextEdit::multiline(text)
                .id_salt("distribution equation")
                .desired_rows(2)
                .desired_width(ui.available_width().min(340.0))
                .hint_text("1 - z/200")
                .font(egui::TextStyle::Monospace)
                .show(ui);
            if let Some(range) = edit.cursor_range {
                view.cursor = Some(range.primary.index.0);
            }
            ui.end_row();
            ui.label("");
            range_label(ui, view);
            ui.end_row();
            variable_rows(ui, &mut distribution.variables);
            ui.label("Help");
            let insert =
                equation_help(ui, fe, distribution.system, &distribution.variables, length);
            ui.end_row();
            if let DistributionKind::Equation(text) = &mut distribution.kind {
                match insert {
                    Some(Insert::Text(add)) => {
                        let at = view
                            .cursor
                            .unwrap_or(text.chars().count())
                            .min(text.chars().count());
                        let byte = text.char_indices().nth(at).map_or(text.len(), |(b, _)| b);
                        text.insert_str(byte, &add);
                        place_cursor(ui, edit.response.id, at + add.chars().count());
                    }
                    Some(Insert::Whole(new)) => {
                        *text = new;
                        place_cursor(ui, edit.response.id, text.chars().count());
                    }
                    None => {}
                }
            }
        }
        DistributionKind::Points {
            source,
            points,
            interpolation,
        } => {
            ui.label("File");
            ui.horizontal(|ui| {
                let name = if source.is_empty() {
                    "(none)".to_string()
                } else {
                    format!("{source}, {} points", points.len())
                };
                ui.label(name);
                if ui.button("Import …").clicked()
                    && let Some(path) = rfd::FileDialog::new()
                        .set_title("Points of the distribution")
                        .add_filter("Text files (*.txt, *.csv, *.dat)", &["txt", "csv", "dat"])
                        .add_filter("All files", &["*"])
                        .pick_file()
                {
                    let read = std::fs::read_to_string(&path).map_err(|e| e.to_string());
                    match read.and_then(|text| parse_points(&text, units)) {
                        Ok(read) => {
                            *points = read;
                            *source = (path.file_name())
                                .map_or_else(String::new, |n| n.to_string_lossy().into_owned());
                            view.import_error = None;
                        }
                        Err(e) => view.import_error = Some(e),
                    }
                }
            });
            ui.end_row();
            if let Some(error) = &view.import_error {
                ui.label("");
                ui.colored_label(crate::style::error(), error);
                ui.end_row();
            }
            ui.label("");
            weak(
                ui,
                &format!(
                    "One point per line: X Y Z factor, coordinates in {length} along the axes \
                     of the coordinate system (also of a cylindrical one). Separated by \
                     blanks, tabs, commas or semicolons; # starts a comment."
                ),
            );
            ui.end_row();
            ui.label("Interpolation");
            egui::ComboBox::from_id_salt("distribution interpolation")
                .selected_text(interpolation.label())
                .width(crate::dock::fit_width(ui, 200.0))
                .show_ui(ui, |ui| {
                    for option in Interpolation::ALL {
                        ui.selectable_value(interpolation, option, option.label());
                    }
                });
            ui.end_row();
        }
    }
    // The factor over the mesh, found again when the distribution changes.
    if !matches!(distribution.kind, DistributionKind::Equation(_)) {
        ui.label("");
        range_label(ui, view);
        ui.end_row();
    }
    ui.label("");
    weak(
        ui,
        "A pressure or temperature that uses the distribution takes its value times the \
         factor at every face or node. With a value of 1 the factors are the values \
         themselves.",
    );
    ui.end_row();
}

/// A hint that wraps at the width of its column.
fn weak(ui: &mut Ui, text: &str) {
    ui.add(egui::Label::new(egui::RichText::new(text).weak()).wrap());
}

/// Whether two distributions give the same factors: they differ in their name at most.
fn same_factors(a: &Distribution, b: &Distribution) -> bool {
    a.kind == b.kind && a.system == b.system && a.variables == b.variables
}

/// The named variables of an equation: name, value with its expose button, and a button to
/// remove it; and one to add a variable.
fn variable_rows(ui: &mut Ui, variables: &mut Vec<plx_model::DistributionVariable>) {
    ui.label("Variables");
    ui.vertical(|ui| {
        let mut remove = None;
        for (i, variable) in variables.iter_mut().enumerate() {
            ui.push_id(variable.key, |ui| {
                ui.horizontal(|ui| {
                    ui.add(
                        egui::TextEdit::singleline(&mut variable.name)
                            .desired_width(70.0)
                            .font(egui::TextStyle::Monospace),
                    );
                    ui.label("=");
                    crate::expose::field(ui, Field::Variable(variable.key), |ui| {
                        ui.add_sized(
                            [80.0, 20.0],
                            crate::numeric::drag_value(&mut variable.value).speed(0.01),
                        )
                    });
                    let icon = crate::icons::Icon::Remove;
                    if crate::icons::dialog_button(ui, icon, "Remove the variable", true).clicked()
                    {
                        remove = Some(i);
                    }
                });
            });
        }
        if let Some(i) = remove {
            variables.remove(i);
        }
        if ui.small_button("Add variable").clicked() {
            variables.push(plx_model::DistributionVariable::next(variables));
        }
    });
    ui.end_row();
    ui.label("");
    weak(
        ui,
        "Numbers the equation uses by name, e.g. p1 * (1 - z/h). The P button exposes one \
         as a parameter of the analysis' block in the project window, where a Parameters \
         block or a parameter study can set it.",
    );
    ui.end_row();
}

/// The factors at the nodes of the mesh, with the smallest and largest.
fn factor_range(distribution: &Distribution, model: &Model) -> FactorRange {
    let sampler = distribution.sampler(model.fe.properties.units, &model.fe.coordinate_systems)?;
    let factors: Vec<f64> = (model.mesh.coords().iter())
        .map(|&p| sampler.factor(p))
        .collect();
    if factors.is_empty() {
        return Err("The model has no mesh yet.".into());
    }
    if factors.iter().any(|f| !f.is_finite()) {
        return Err(
            "The factor is not a number at some nodes, such as where it divides \
                    by zero."
                .into(),
        );
    }
    let (min, max) = (factors.iter()).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &f| {
        (lo.min(f), hi.max(f))
    });
    Ok(Factors {
        min,
        max,
        at_nodes: std::sync::Arc::new(factors.iter().map(|&f| f as f32).collect()),
    })
}

fn short(value: f64) -> String {
    let rounded = format!("{value:.4}");
    let trimmed = rounded.trim_end_matches('0').trim_end_matches('.');
    match trimmed {
        "-0" => "0".into(),
        t => t.into(),
    }
}

pub fn validate(distribution: &Distribution, fe: &FeModel) -> Result<(), String> {
    match distribution.problem(&fe.coordinate_systems) {
        Some(problem) => Err(problem),
        None => Ok(()),
    }
}

/// The distribution choice of a pressure or temperature: "Uniform" or one of the model's
/// distributions.
pub fn distribution_row(
    ui: &mut Ui,
    id: &str,
    reference: &mut Option<plx_model::Id>,
    fe: &FeModel,
) {
    ui.label("Distribution");
    let text = match *reference {
        None => UNIFORM.to_string(),
        Some(id) => fe.name_or_missing(Some(id)),
    };
    egui::ComboBox::from_id_salt(id)
        .selected_text(text)
        .width(crate::dock::fit_width(ui, 200.0))
        .show_ui(ui, |ui| {
            ui.selectable_value(reference, None, UNIFORM);
            for distribution in &fe.distributions {
                ui.selectable_value(reference, Some(distribution.id), &distribution.name);
            }
        });
    ui.end_row();
    if let Some(id) = *reference
        && fe.distribution(id).is_none()
    {
        ui.label("");
        ui.colored_label(crate::style::error(), "The distribution does not exist");
        ui.end_row();
    }
}

/// The same value over the whole region.
const UNIFORM: &str = "Uniform";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rounds_the_range_for_reading() {
        assert_eq!(short(0.5), "0.5");
        assert_eq!(short(1.0), "1");
        assert_eq!(short(-0.00001), "0");
        assert_eq!(short(0.123456), "0.1235");
    }
}
