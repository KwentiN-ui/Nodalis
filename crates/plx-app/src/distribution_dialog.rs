//! The distribution dialog: a factor over the model by an equation in x, y and z or by
//! points read from a text file; and the distribution choice of the pressure and
//! temperature dialogs.

use egui::Ui;
use plx_model::{Distribution, DistributionKind, FeModel, Interpolation, Quantity, parse_points};

use crate::model::Model;

/// State of the dialog besides the distribution itself.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DistributionView {
    /// Why the last file could not be read.
    import_error: Option<String>,
    /// The range of the factor over the mesh nodes, for the distribution it was found for.
    range: Option<(DistributionKind, FactorRange)>,
}

/// The smallest and largest factor at the mesh nodes, or why there are none.
type FactorRange = Result<(f64, f64), String>;

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
    ui.horizontal_wrapped(|ui| {
        if ui.radio(by_equation, "Equation").clicked() && !by_equation {
            distribution.kind = DistributionKind::Equation("=1".into());
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
    match &mut distribution.kind {
        DistributionKind::Equation(text) => {
            ui.label("Factor");
            ui.add(
                egui::TextEdit::singleline(text)
                    .desired_width(260.0)
                    .font(egui::TextStyle::Monospace),
            );
            ui.end_row();
            ui.label("");
            ui.weak(format!(
                "Coordinates x, y, z in {length}; e.g. =1 - z/200 or =Max(0, x/50).\n\
                 Operators + - * / % and functions such as Sqrt, Pow, Sin, Exp, If."
            ));
            ui.end_row();
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
            ui.weak(format!(
                "One point per line: X Y Z factor, coordinates in {length}.\n\
                 Separated by blanks, tabs, commas or semicolons; # starts a comment."
            ));
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
    if view
        .range
        .as_ref()
        .is_none_or(|(kind, _)| *kind != distribution.kind)
    {
        let range = factor_range(distribution, model);
        view.range = Some((distribution.kind.clone(), range));
    }
    if let Some((_, range)) = &view.range {
        ui.label("Over the mesh");
        match range {
            Ok((min, max)) => ui.label(format!("{} to {}", short(*min), short(*max))),
            Err(problem) => ui.colored_label(crate::style::error(), problem),
        };
        ui.end_row();
    }
    ui.label("");
    ui.weak(
        "A pressure or temperature that uses the distribution takes its\n\
         value times the factor at every face or node. With a value of 1\n\
         the factors are the values themselves.",
    );
    ui.end_row();
}

/// The smallest and largest factor at the nodes of the mesh.
fn factor_range(distribution: &Distribution, model: &Model) -> FactorRange {
    let sampler = distribution.sampler(model.fe.properties.units)?;
    let (min, max) = (model.mesh.coords().iter())
        .map(|&p| sampler.factor(p))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), f| {
            (lo.min(f), hi.max(f))
        });
    if min.is_nan() || max.is_nan() {
        return Err("The factor is not a number at some nodes.".into());
    }
    if min > max {
        return Err("The model has no mesh yet.".into());
    }
    Ok((min, max))
}

fn short(value: f64) -> String {
    let rounded = format!("{value:.4}");
    let trimmed = rounded.trim_end_matches('0').trim_end_matches('.');
    match trimmed {
        "-0" => "0".into(),
        t => t.into(),
    }
}

pub fn validate(distribution: &Distribution) -> Result<(), String> {
    match distribution.problem() {
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
