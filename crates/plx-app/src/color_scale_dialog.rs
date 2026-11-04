//! The Color Scale dialog of a results file: colour spectrum,
//! number of colours and automatic or manual limits with colours for the values beyond them.

use egui::{Color32, Rect, Sense, Stroke, StrokeKind, vec2};
use plx_model::color_scale::{MAX_LEVELS, valid_limits};
use plx_model::{ColorScale, ColorSpectrum};
use plx_render::contour::{band_colors, spectrum_color};

use crate::numeric;
use crate::results::format_value;

/// The open dialog. Changes show at once; Cancel restores the scale it was opened with.
pub struct ColorScaleDialog {
    /// The results file whose scale is edited.
    pub result: usize,
    original: ColorScale,
    pub draft: ColorScale,
    /// Limits entered for a manual scale, kept while the scale is automatic.
    range: [f64; 2],
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum DialogResult {
    Open,
    Ok,
    Cancel,
}

impl ColorScaleDialog {
    /// `extremes` are the legend's current limits, the start of a manual range.
    pub fn new(result: usize, scale: &ColorScale, extremes: Option<(f32, f32)>) -> Self {
        // Rounded as the legend labels them, a tidy start for the user's own range.
        let tidy = |v: f32| format_value(v).parse().unwrap_or(v as f64);
        let range = scale
            .manual
            .unwrap_or_else(|| extremes.map_or([0.0, 1.0], |(min, max)| [tidy(min), tidy(max)]));
        Self {
            result,
            original: scale.clone(),
            draft: scale.clone(),
            range,
        }
    }

    pub fn original(&self) -> &ColorScale {
        &self.original
    }

    pub fn show(&mut self, ctx: &egui::Context) -> DialogResult {
        let mut result = DialogResult::Open;
        let mut open = true;
        let draft = &mut self.draft;
        let range = &mut self.range;
        egui::Window::new("Color Scale")
            .id(crate::ids::window(ctx, "color scale dialog"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            // On the right, clear of the legend and the model, which show the changes.
            .pivot(egui::Align2::RIGHT_TOP)
            .default_pos(ctx.content_rect().right_top() + vec2(-60.0, 200.0))
            .show(ctx, |ui| {
                ui.strong("Color spectrum");
                egui::Grid::new("color scale spectrum")
                    .num_columns(2)
                    .spacing([12.0, 4.0])
                    .show(ui, |ui| {
                        ui.label("Spectrum");
                        egui::ComboBox::from_id_salt("color spectrum")
                            .selected_text(draft.spectrum.label())
                            .width(180.0)
                            .height(400.0)
                            .show_ui(ui, |ui| {
                                for spectrum in ColorSpectrum::ALL {
                                    ui.horizontal(|ui| {
                                        let preview = ColorScale {
                                            spectrum,
                                            ..draft.clone()
                                        };
                                        gradient(ui, &preview, vec2(60.0, 14.0));
                                        ui.selectable_value(
                                            &mut draft.spectrum,
                                            spectrum,
                                            spectrum.label(),
                                        );
                                    });
                                }
                            });
                        ui.end_row();
                        ui.label("Brightness");
                        ui.add(
                            numeric::drag_value(&mut draft.brightness)
                                .range(0.0..=1.0)
                                .speed(0.01),
                        )
                        .on_hover_text("Lightens the colours towards white (0 ... 1).");
                        ui.end_row();
                        ui.label("Reverse colors");
                        ui.checkbox(&mut draft.reverse, "");
                        ui.end_row();
                        ui.label("Number of colors");
                        ui.add(numeric::drag_value(&mut draft.levels).range(2..=MAX_LEVELS));
                        ui.end_row();
                    });
                ui.add_space(4.0);
                bands(ui, draft);
                ui.separator();
                ui.strong("Limits");
                let mut manual = draft.manual.is_some();
                egui::Grid::new("color scale limits")
                    .num_columns(2)
                    .spacing([12.0, 4.0])
                    .show(ui, |ui| {
                        ui.label("Min/max limit type");
                        ui.horizontal_wrapped(|ui| {
                            let automatic = ui.radio_value(&mut manual, false, "Automatic");
                            let user = ui.radio_value(&mut manual, true, "Manual");
                            if automatic.changed() || user.changed() {
                                draft.manual = manual.then_some(*range);
                            }
                        });
                        ui.end_row();
                        for (label, k) in [("Maximum", 1), ("Minimum", 0)] {
                            ui.label(label);
                            let changed = ui
                                .add_enabled(manual, numeric::drag_value(&mut range[k]))
                                .changed();
                            if changed && manual {
                                draft.manual = Some(*range);
                            }
                            ui.end_row();
                        }
                        ui.label("Above maximum");
                        ui.add_enabled_ui(manual, |ui| {
                            ui.horizontal(|ui| {
                                ui.color_edit_button_srgb(&mut draft.max_color);
                                ui.checkbox(&mut draft.show_max_color, "Show in legend");
                            });
                        });
                        ui.end_row();
                        ui.label("Below minimum");
                        ui.add_enabled_ui(manual, |ui| {
                            ui.horizontal(|ui| {
                                ui.color_edit_button_srgb(&mut draft.min_color);
                                ui.checkbox(&mut draft.show_min_color, "Show in legend");
                            });
                        });
                        ui.end_row();
                    });
                let valid = !manual || valid_limits(range[0], range[1]);
                if !valid {
                    ui.colored_label(
                        ui.visuals().error_fg_color,
                        "The minimum must be smaller than the maximum.",
                    );
                }
                // A tool window: the results show the scale as it is edited.
                let mut defaults = false;
                let footer = crate::dialog::tool_footer_with(ui, None, |ui| {
                    defaults = ui
                        .button("Defaults")
                        .on_hover_text("Rainbow with automatic limits")
                        .clicked();
                });
                if defaults {
                    *draft = ColorScale::default();
                }
                if footer == crate::dialog::Tool::Close {
                    result = DialogResult::Ok;
                }
            });
        if !open || ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            result = DialogResult::Ok;
        }
        // Limits that cannot be shown are dropped on closing.
        if result == DialogResult::Ok && self.draft.manual.is_some_and(|[a, b]| !valid_limits(a, b))
        {
            result = DialogResult::Cancel;
        }
        result
    }
}

fn color32([r, g, b]: [f32; 3]) -> Color32 {
    let byte = |c: f32| (c * 255.0).round() as u8;
    Color32::from_rgb(byte(r), byte(g), byte(b))
}

/// The continuous spectrum of a scale, minimum on the left.
fn gradient(ui: &mut egui::Ui, scale: &ColorScale, size: egui::Vec2) {
    let (rect, _) = ui.allocate_exact_size(size, Sense::hover());
    let painter = ui.painter_at(rect);
    let steps = 32;
    let width = rect.width() / steps as f32;
    for i in 0..steps {
        let t = (i as f64 + 0.5) / steps as f64;
        let cell = Rect::from_min_size(
            rect.left_top() + vec2(i as f32 * width, 0.0),
            vec2(width + 0.5, rect.height()),
        );
        painter.rect_filled(cell, 0.0, color32(spectrum_color(scale, t)));
    }
    painter.rect_stroke(
        rect,
        0.0,
        Stroke::new(1.0, Color32::GRAY),
        StrokeKind::Inside,
    );
}

/// The bands the scale colours the results with, minimum on the left.
fn bands(ui: &mut egui::Ui, scale: &ColorScale) {
    let colors = band_colors(scale);
    let (rect, _) = ui.allocate_exact_size(vec2(300.0, 16.0), Sense::hover());
    let painter = ui.painter_at(rect);
    let band = rect.width() / colors.len() as f32;
    for (i, color) in colors.into_iter().enumerate() {
        let cell = Rect::from_min_size(
            rect.left_top() + vec2(i as f32 * band, 0.0),
            vec2(band, rect.height()),
        );
        painter.rect(
            cell,
            0.0,
            color32(color),
            Stroke::new(1.0, Color32::BLACK),
            StrokeKind::Inside,
        );
    }
}
