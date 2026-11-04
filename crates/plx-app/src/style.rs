//! Look of the application: a light theme after the classic Windows controls and a
//! dark one after Windows' dark mode, picked in the settings.
//!
//! Segoe UI, the classic Windows face, may not be redistributed. Noto Sans (SIL Open Font License,
//! `assets/fonts/OFL.txt`) is a free face of the same humanist style; the bundled file is
//! subset to Latin, Greek and common symbols.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use egui::{Color32, CornerRadius, Stroke, Theme, Visuals};
use serde::{Deserialize, Serialize};

const NOTO_SANS: &[u8] = include_bytes!("../assets/fonts/NotoSans-Regular.ttf");

/// The colour scheme the user picked in the settings.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ThemeChoice {
    /// Light or dark as the operating system is set.
    #[default]
    System,
    Light,
    Dark,
}

impl ThemeChoice {
    pub const ALL: [ThemeChoice; 3] = [ThemeChoice::System, ThemeChoice::Light, ThemeChoice::Dark];

    pub fn label(self) -> &'static str {
        match self {
            ThemeChoice::System => "System",
            ThemeChoice::Light => "Light",
            ThemeChoice::Dark => "Dark",
        }
    }

    fn preference(self) -> egui::ThemePreference {
        match self {
            ThemeChoice::System => egui::ThemePreference::System,
            ThemeChoice::Light => egui::ThemePreference::Light,
            ThemeChoice::Dark => egui::ThemePreference::Dark,
        }
    }
}

/// The colours of one theme. The light one follows the classic Windows controls, the
/// dark one Windows' dark mode.
struct Palette {
    /// Windows "Control" colour: panels, menu and tool bars.
    control: Color32,
    /// Windows "Window" colour: tree, lists and text fields.
    window: Color32,
    /// Windows highlight for selected items.
    highlight: Color32,
    border: Color32,
    text: Color32,
    hover_fill: Color32,
    pressed_fill: Color32,
    pressed_border: Color32,
    /// Alternate rows of tables.
    faint: Color32,
    /// Buttons at rest.
    button: Color32,
    /// Frames of group boxes and separators.
    separator: Color32,
    error: Color32,
    warning: Color32,
    ok: Color32,
    /// Background gradient of the 3D view, top and bottom.
    viewport: [Color32; 2],
    /// Lines and labels drawn over the 3D view.
    viewport_text: Color32,
    /// Axes and labels of the plots.
    plot_text: Color32,
    plot_grid: Color32,
}

const LIGHT: Palette = Palette {
    control: Color32::from_rgb(240, 240, 240),
    window: Color32::WHITE,
    highlight: Color32::from_rgb(0, 120, 215),
    border: Color32::from_rgb(173, 173, 173),
    text: Color32::from_rgb(0, 0, 0),
    hover_fill: Color32::from_rgb(229, 241, 251),
    pressed_fill: Color32::from_rgb(204, 228, 247),
    pressed_border: Color32::from_rgb(0, 84, 153),
    faint: Color32::from_rgb(246, 246, 246),
    button: Color32::from_rgb(225, 225, 225),
    separator: Color32::from_rgb(210, 210, 210),
    error: Color32::from_rgb(200, 0, 0),
    warning: Color32::from_rgb(170, 110, 0),
    ok: Color32::from_rgb(0, 128, 0),
    // The background gradient: Gainsboro at the top, WhiteSmoke at the bottom.
    viewport: [
        Color32::from_rgb(220, 220, 220),
        Color32::from_rgb(245, 245, 245),
    ],
    viewport_text: Color32::BLACK,
    plot_text: Color32::from_gray(40),
    plot_grid: Color32::from_gray(225),
};

const DARK: Palette = Palette {
    control: Color32::from_rgb(43, 43, 46),
    window: Color32::from_rgb(30, 30, 32),
    highlight: Color32::from_rgb(0, 120, 215),
    border: Color32::from_rgb(90, 90, 94),
    text: Color32::from_rgb(225, 225, 225),
    hover_fill: Color32::from_rgb(45, 70, 95),
    pressed_fill: Color32::from_rgb(30, 85, 135),
    pressed_border: Color32::from_rgb(90, 165, 235),
    faint: Color32::from_rgb(38, 38, 41),
    button: Color32::from_rgb(62, 62, 66),
    separator: Color32::from_rgb(70, 70, 74),
    error: Color32::from_rgb(255, 110, 100),
    warning: Color32::from_rgb(235, 175, 60),
    ok: Color32::from_rgb(110, 205, 110),
    viewport: [Color32::from_rgb(38, 41, 48), Color32::from_rgb(78, 83, 94)],
    viewport_text: Color32::from_rgb(235, 235, 235),
    plot_text: Color32::from_gray(215),
    plot_grid: Color32::from_gray(65),
};

/// Whether the dark theme is shown, set each frame by [`sync`]. A global, because many
/// painters draw without access to the context.
static DARK_MODE: AtomicBool = AtomicBool::new(false);

pub fn dark() -> bool {
    DARK_MODE.load(Ordering::Relaxed)
}

fn palette() -> &'static Palette {
    if dark() { &DARK } else { &LIGHT }
}

pub fn control() -> Color32 {
    palette().control
}
pub fn window() -> Color32 {
    palette().window
}
pub fn highlight() -> Color32 {
    palette().highlight
}
pub fn border() -> Color32 {
    palette().border
}
pub fn text() -> Color32 {
    palette().text
}
pub fn hover_fill() -> Color32 {
    palette().hover_fill
}
pub fn pressed_fill() -> Color32 {
    palette().pressed_fill
}
/// Alternate rows and header bands of tables.
pub fn faint() -> Color32 {
    palette().faint
}
/// Header band of a block in the project schematic: a muted blue-grey.
pub fn node_header() -> Color32 {
    if dark() {
        Color32::from_rgb(52, 66, 84)
    } else {
        Color32::from_rgb(214, 223, 234)
    }
}
/// Header band of a materials block in the project schematic: a muted orange-grey.
pub fn materials_header() -> Color32 {
    if dark() {
        Color32::from_rgb(84, 66, 48)
    } else {
        Color32::from_rgb(236, 222, 204)
    }
}
/// Header band of a parameter study and a Campbell block in the project schematic: a muted
/// violet.
pub fn study_header() -> Color32 {
    if dark() {
        Color32::from_rgb(58, 50, 96)
    } else {
        Color32::from_rgb(212, 206, 244)
    }
}

pub fn measure_header() -> Color32 {
    if dark() {
        Color32::from_rgb(88, 64, 40)
    } else {
        Color32::from_rgb(244, 224, 200)
    }
}

pub fn parameters_header() -> Color32 {
    if dark() {
        Color32::from_rgb(70, 54, 88)
    } else {
        Color32::from_rgb(226, 214, 240)
    }
}

pub fn mesh_header() -> Color32 {
    if dark() {
        Color32::from_rgb(54, 74, 58)
    } else {
        Color32::from_rgb(212, 230, 214)
    }
}
/// Header band of an export block in the project schematic: a muted blue-grey.
pub fn export_header() -> Color32 {
    if dark() {
        Color32::from_rgb(52, 62, 84)
    } else {
        Color32::from_rgb(214, 222, 238)
    }
}
/// Header band of a note in the project schematic: a muted yellow.
pub fn note_header() -> Color32 {
    if dark() {
        Color32::from_rgb(80, 74, 44)
    } else {
        Color32::from_rgb(240, 232, 196)
    }
}
/// Outline of the pins of the project schematic.
pub fn pin_edge() -> Color32 {
    if dark() {
        Color32::from_rgb(20, 20, 22)
    } else {
        Color32::from_rgb(70, 70, 74)
    }
}
/// Dots of the schematic's background.
pub fn schematic_dot() -> Color32 {
    if dark() {
        Color32::from_rgb(70, 70, 74)
    } else {
        Color32::from_rgb(196, 196, 198)
    }
}
/// Messages about errors and invalid input.
pub fn error() -> Color32 {
    palette().error
}
pub fn warning() -> Color32 {
    palette().warning
}
/// Messages about success.
pub fn ok() -> Color32 {
    palette().ok
}
/// Background gradient of the 3D view as RGBA fractions, top and bottom.
pub fn viewport_background() -> [[f32; 4]; 2] {
    palette().viewport.map(|c| {
        let [r, g, b, a] = c.to_array();
        [r, g, b, a].map(|v| v as f32 / 255.0)
    })
}
/// Colour of the wireframe of the undeformed shape as RGB fractions.
pub fn viewport_wireframe() -> [f32; 3] {
    let [r, g, b, _] = viewport_text().to_array();
    [r, g, b].map(|v| v as f32 / 255.0)
}
/// Text, frames and leader lines of the legend, labels and other overlays of the 3D view.
pub fn viewport_text() -> Color32 {
    palette().viewport_text
}
/// Background of labels in the 3D view and of plots.
pub fn label_fill() -> Color32 {
    palette().window
}
/// Axes, ticks and labels of plots.
pub fn plot_text() -> Color32 {
    palette().plot_text
}
pub fn plot_grid() -> Color32 {
    palette().plot_grid
}
/// Translucent background behind text inside a plot.
pub fn plot_label_fill(alpha: u8) -> Color32 {
    let [r, g, b, _] = label_fill().to_array();
    Color32::from_rgba_unmultiplied(r, g, b, alpha)
}

/// A colour drawn for the light theme, as shown in the current theme: in the dark theme,
/// greys are inverted (dark ink becomes light, light faces dark) while coloured parts keep
/// their colour. For icons and other drawings with hard-coded colours.
pub fn adapt(color: Color32) -> Color32 {
    if dark() { invert_grey(color) } else { color }
}

/// A dark colour drawn over the 3D view, lightened in the dark theme so that it stands out
/// from the dark background.
pub fn lift(color: Color32) -> Color32 {
    if !dark() {
        return color;
    }
    let [r, g, b, a] = color.to_srgba_unmultiplied();
    let mix = |v: u8| (v as f32 + (255.0 - v as f32) * 0.4).round() as u8;
    Color32::from_rgba_unmultiplied(mix(r), mix(g), mix(b), a)
}

fn invert_grey(color: Color32) -> Color32 {
    let [r, g, b, a] = color.to_srgba_unmultiplied();
    let (max, min) = (r.max(g).max(b), r.min(g).min(b));
    if max - min > 40 {
        return color;
    }
    // Lightness mapped onto 70 (white) to 236 (black), so that faces stay apart from the
    // dark panels and ink stays short of glaring white.
    let shift = |v: u8| (70.0 + (255.0 - v as f32) * 0.65).round() as u8;
    Color32::from_rgba_unmultiplied(shift(r), shift(g), shift(b), a)
}

/// Shows the theme the user picked.
pub fn set_theme(ctx: &egui::Context, choice: ThemeChoice) {
    ctx.set_theme(choice.preference());
    sync(ctx);
}

/// Takes over the theme egui shows, which follows the system for [`ThemeChoice::System`];
/// called at the start of each frame.
pub fn sync(ctx: &egui::Context) {
    DARK_MODE.store(ctx.theme() == Theme::Dark, Ordering::Relaxed);
}

/// Minimum height of buttons, selectable labels and combo entries: text row plus padding and
/// frame stroke, rounded up (checked by a test).
const INTERACT_HEIGHT: f32 = 22.0;

pub fn apply(ctx: &egui::Context) {
    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "Noto Sans".into(),
        Arc::new(egui::FontData::from_static(NOTO_SANS)),
    );
    fonts
        .families
        .entry(egui::FontFamily::Proportional)
        .or_default()
        .insert(0, "Noto Sans".into());
    ctx.set_fonts(fonts);

    // egui falls back to dark when the system does not tell; the default look is light.
    ctx.options_mut(|o| o.fallback_theme = Theme::Light);
    ctx.set_visuals_of(Theme::Light, visuals(Visuals::light(), &LIGHT));
    ctx.set_visuals_of(Theme::Dark, visuals(Visuals::dark(), &DARK));
    for theme in [Theme::Light, Theme::Dark] {
        ctx.style_mut_of(theme, |style| {
            style.spacing.item_spacing = egui::vec2(6.0, 3.0);
            style.spacing.button_padding = egui::vec2(6.0, 2.0);
            // egui lays out a selectable label without frame (unselected, not hovered) with
            // its padding reduced by the frame stroke, so hovering or selecting it grows the
            // row by two pixels and moves everything below. A minimum height that covers the
            // framed size keeps all states of buttons, selectables and combo entries equally
            // tall.
            style.spacing.interact_size.y = INTERACT_HEIGHT;
            style.spacing.indent = 16.0;
            for (text_style, size) in [
                (egui::TextStyle::Body, 13.0),
                (egui::TextStyle::Button, 13.0),
                (egui::TextStyle::Small, 11.0),
                (egui::TextStyle::Heading, 16.0),
                (egui::TextStyle::Monospace, 12.0),
            ] {
                if let Some(font) = style.text_styles.get_mut(&text_style) {
                    font.size = size;
                }
            }
        });
    }
    set_theme(ctx, ThemeChoice::Light);
}

fn visuals(mut v: Visuals, p: &Palette) -> Visuals {
    v.override_text_color = Some(p.text);
    v.panel_fill = p.control;
    v.window_fill = p.control;
    v.window_stroke = Stroke::new(1.0, p.border);
    v.window_corner_radius = CornerRadius::ZERO;
    v.menu_corner_radius = CornerRadius::ZERO;
    v.extreme_bg_color = p.window;
    v.text_edit_bg_color = Some(p.window);
    v.faint_bg_color = p.faint;
    v.code_bg_color = p.faint;
    v.selection.bg_fill = p.highlight;
    v.selection.stroke = Stroke::new(1.0, Color32::WHITE);
    v.hyperlink_color = if std::ptr::eq(p, &DARK) {
        Color32::from_rgb(110, 175, 255)
    } else {
        Color32::from_rgb(0, 102, 204)
    };
    v.error_fg_color = p.error;
    v.warn_fg_color = p.warning;
    v.indent_has_left_vline = true;

    let w = &mut v.widgets;
    w.noninteractive.bg_fill = p.control;
    w.noninteractive.weak_bg_fill = p.control;
    w.noninteractive.bg_stroke = Stroke::new(1.0, p.separator);
    w.noninteractive.fg_stroke = Stroke::new(1.0, p.text);
    w.inactive.bg_fill = p.button;
    w.inactive.weak_bg_fill = p.button;
    w.inactive.bg_stroke = Stroke::new(1.0, p.border);
    w.inactive.fg_stroke = Stroke::new(1.0, p.text);
    w.hovered.bg_fill = p.hover_fill;
    w.hovered.weak_bg_fill = p.hover_fill;
    w.hovered.bg_stroke = Stroke::new(1.0, p.highlight);
    w.hovered.fg_stroke = Stroke::new(1.0, p.text);
    w.hovered.expansion = 0.0;
    w.active.bg_fill = p.pressed_fill;
    w.active.weak_bg_fill = p.pressed_fill;
    w.active.bg_stroke = Stroke::new(1.0, p.pressed_border);
    w.active.fg_stroke = Stroke::new(1.0, p.text);
    w.active.expansion = 0.0;
    w.open.bg_fill = p.pressed_fill;
    w.open.weak_bg_fill = p.pressed_fill;
    w.open.bg_stroke = Stroke::new(1.0, p.highlight);
    w.open.fg_stroke = Stroke::new(1.0, p.text);
    for state in [
        &mut w.noninteractive,
        &mut w.inactive,
        &mut w.hovered,
        &mut w.active,
        &mut w.open,
    ] {
        state.corner_radius = CornerRadius::ZERO;
    }
    v
}

/// A row of tabs like a Windows tab control, drawn on the panel above a pane. Returns the
/// index of the tab clicked and the rect of the selected one, for [`open_tab`].
pub fn tabs(
    ui: &mut egui::Ui,
    titles: &[&str],
    selected: usize,
) -> (Option<usize>, Option<egui::Rect>) {
    let mut clicked = None;
    let mut selected_rect = None;
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 0.0;
        ui.add_space(2.0);
        for (index, title) in titles.iter().enumerate() {
            let is_selected = index == selected;
            let galley = ui.painter().layout_no_wrap(
                title.to_string(),
                egui::TextStyle::Body.resolve(ui.style()),
                text(),
            );
            let size = galley.size() + egui::vec2(16.0, 8.0);
            let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click());
            let fill = if is_selected {
                window()
            } else if response.hovered() {
                hover_fill()
            } else {
                control()
            };
            let rect = if is_selected {
                selected_rect = Some(rect);
                rect
            } else {
                rect.shrink2(egui::vec2(0.0, 1.0))
                    .translate(egui::vec2(0.0, 1.0))
            };
            ui.painter().rect(
                rect,
                0.0,
                fill,
                Stroke::new(1.0, border()),
                egui::StrokeKind::Inside,
            );
            ui.painter()
                .galley(rect.center() - galley.size() * 0.5, galley, text());
            if response.clicked() {
                clicked = Some(index);
            }
        }
    });
    // Tabs sit flush on the pane: their bottom border is the pane's top border.
    ui.add_space(-ui.spacing().item_spacing.y - 1.0);
    (clicked, selected_rect)
}

/// Opens the selected tab of [`tabs`] into the pane below it, once the pane is drawn.
pub fn open_tab(ui: &egui::Ui, tab: Option<egui::Rect>) {
    if let Some(tab) = tab {
        ui.painter().hline(
            tab.x_range().shrink(1.0),
            tab.bottom() - 0.5,
            Stroke::new(1.0, window()),
        );
    }
}

#[cfg(test)]
mod tests {
    /// Heights of an unselected and a selected label; the selected one is framed like a hovered
    /// one.
    fn selectable_heights(ctx: &egui::Context) -> (f32, f32, f32) {
        let mut heights = (0.0, 0.0, 0.0);
        let mut output = ctx.run_ui(egui::RawInput::default(), |ui| {
            heights.0 = ui.selectable_label(false, "1, 4").rect.height();
            heights.1 = ui.selectable_label(true, "1, 4").rect.height();
            heights.2 = ui.button("1, 4").rect.height();
        });
        output.textures_delta.clear();
        heights
    }

    #[test]
    fn dark_theme_inverts_greys_only() {
        use egui::Color32;
        let ink = super::invert_grey(Color32::from_rgb(70, 70, 70));
        let face = super::invert_grey(Color32::WHITE);
        assert!(ink.r() > 150, "dark ink turns light: {ink:?}");
        assert!(face.r() < 100, "white faces turn dark: {face:?}");
        let accent = Color32::from_rgb(70, 140, 215);
        assert_eq!(super::invert_grey(accent), accent, "colours stay");
        let translucent = Color32::from_rgba_unmultiplied(0, 0, 0, 128);
        assert_eq!(super::invert_grey(translucent).a(), 128);
    }

    #[test]
    fn hover_frame_keeps_row_height() {
        let ctx = egui::Context::default();
        super::apply(&ctx);
        // Fonts are loaded during the first pass.
        selectable_heights(&ctx);
        let (plain, framed, button) = selectable_heights(&ctx);
        assert_eq!(plain, framed, "selectable label grows when framed");
        assert_eq!(plain, button);
        assert_eq!(
            plain,
            super::INTERACT_HEIGHT,
            "INTERACT_HEIGHT no longer covers the text"
        );
    }
}
