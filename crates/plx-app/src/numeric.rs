//! Number input shared by all numeric fields of the GUI.

use egui::emath::Numeric;
use plx_model::{Quantity, UnitSystem};

pub use plx_model::units::{evaluate, parse_number};

/// A [`egui::DragValue`] that reads typed input with [`evaluate`], so "1/30" may be typed for
/// its result, and shows the value with all its digits ([`format_physical`]). egui would show
/// only as many decimals as its drag speed resolves, so 0.005 would read "0.01" and be stored
/// so on the next edit.
pub fn drag_value<Num: Numeric>(value: &mut Num) -> egui::DragValue<'_> {
    egui::DragValue::new(value)
        .custom_parser(evaluate)
        .custom_formatter(|v, _| format_physical(v))
}

/// A [`egui::DragValue`] for a whole number such as a count, which takes a calculation
/// ([`evaluate`]) as well, but only one with a whole result: "12/4" gives 3, "10/4" is
/// rejected rather than cut to 2.
pub fn count<Num: Numeric>(value: &mut Num) -> egui::DragValue<'_> {
    egui::DragValue::new(value).custom_parser(parse_count)
}

/// A whole number typed, possibly as a calculation ([`count`]).
pub fn parse_count(text: &str) -> Option<f64> {
    evaluate(text).filter(|v| v.fract() == 0.0)
}

/// A field for a value of a physical quantity, stored in SI units ([`plx_model::STORED`]) and
/// shown in the model's units: the unit stands after the number,
/// and a value typed with another unit is converted, so "3 cm" becomes "30 mm" in a
/// millimetre model.
pub fn quantity(value: &mut f64, units: UnitSystem, quantity: Quantity) -> egui::DragValue<'_> {
    let unit = units.unit(quantity);
    let field = without_unit(value, units, quantity);
    if unit.is_empty() {
        field
    } else {
        field.suffix(format!(" {unit}"))
    }
}

/// A [`quantity`] field that does not show the unit, e.g. because a column title does.
pub fn without_unit(value: &mut f64, units: UnitSystem, quantity: Quantity) -> egui::DragValue<'_> {
    converted(value, plx_model::STORED, units, quantity)
}

/// A field without the unit for a value stored in `stored` units and shown in `shown` ones,
/// e.g. a coordinate of results, which are stored in the units they show.
pub fn converted(
    value: &mut f64,
    stored: UnitSystem,
    shown: UnitSystem,
    quantity: Quantity,
) -> egui::DragValue<'_> {
    egui::DragValue::from_get_set(move |new| {
        if let Some(new) = new {
            *value = shown.convert(new, quantity, stored);
        }
        stored.convert(*value, quantity, shown)
    })
    .custom_parser(move |text| shown.parse_value(text, quantity).ok())
    .custom_formatter(|v, _| format_physical(v))
}

/// A [`quantity`] field for values that may be very small or large, such as a density of
/// 7.85e-9: shown in e-notation then, and not changed by dragging.
pub fn physical(value: &mut f64, units: UnitSystem, of: Quantity) -> egui::DragValue<'_> {
    quantity(value, units, of).speed(0.0)
}

/// A number field of a fixed size, such as in a block of the project schematic, where
/// [`format_physical`] may not fit with the unit: shown with as many significant digits as
/// fit, the unit always whole, and the full value in the tooltip, before `tip`. Only the
/// display is shortened: typing starts from the full value, so a value that is not edited
/// keeps all its digits. A disabled field, e.g. one a value link gives, shows the tooltip as
/// well.
pub fn fitted(
    ui: &mut egui::Ui,
    size: egui::Vec2,
    field: egui::DragValue<'_>,
    tip: &str,
) -> egui::Response {
    let layout = egui::Layout::centered_and_justified(ui.layout().main_dir());
    ui.allocate_ui_with_layout(size, layout, |ui| {
        // The id the DragValue takes: while it has the focus it is typed in, in full.
        let editing = ui.memory(|m| m.has_focus(ui.next_auto_id()));
        let affix = field
            .atoms()
            .text()
            .map(|t| t.into_owned())
            .unwrap_or_default();
        let font = egui::TextStyle::Button.resolve(ui.style());
        let ctx = ui.ctx().clone();
        let width = move |text: &str| {
            let galley = ctx.fonts_mut(|f| {
                f.layout_no_wrap(text.to_owned(), font.clone(), egui::Color32::WHITE)
            });
            galley.size().x
        };
        let room = size.x - 2.0 * ui.spacing().button_padding.x - width(&affix) - 2.0;
        let shown = std::rc::Rc::new(std::cell::Cell::new(None));
        let field = if editing {
            field
        } else {
            let shown = shown.clone();
            field.custom_formatter(move |value, _| {
                let text = fit(value, |text| width(text) <= room);
                shown.set(Some((value, text.clone())));
                text
            })
        };
        let response = ui.add(field);
        let tip = match shown.take() {
            Some((value, text)) if text != format_physical(value) => {
                format!("{}{affix} (shown rounded)\n{tip}", format_physical(value))
            }
            _ => tip.to_owned(),
        };
        let tip = tip.trim_end();
        if response.enabled() {
            response.on_hover_text(tip)
        } else {
            read_only_tooltip(&response, tip);
            response
        }
    })
    .inner
}

/// The tooltip of a disabled field. egui's own one looks for the pointer in the layer of the
/// field, which misses fields in a sublayer, such as in the zoomed project schematic.
fn read_only_tooltip(response: &egui::Response, tip: &str) {
    let ctx = &response.ctx;
    let rect = (ctx.layer_transform_to_global(response.layer_id))
        .map_or(response.rect, |to_global| to_global * response.rect);
    let (pointer, still) =
        ctx.input(|i| (i.pointer.hover_pos(), i.pointer.time_since_last_movement()));
    if tip.is_empty() || !pointer.is_some_and(|p| rect.contains(p)) {
        return;
    }
    let delay = ctx.global_style().interaction.tooltip_delay;
    if still < delay {
        ctx.request_repaint_after_secs(delay - still);
    } else {
        response.show_tooltip_text(tip);
    }
}

/// `value` with all its digits ([`format_physical`]) if `fits` them, else with as many
/// significant digits as fit, at least two.
pub fn fit(value: f64, fits: impl Fn(&str) -> bool) -> String {
    let full = format_physical(value);
    if fits(&full) {
        return full;
    }
    for digits in (2..15).rev() {
        let rounded: f64 = format!("{value:.*e}", digits - 1).parse().unwrap_or(value);
        let text = format_physical(rounded);
        if text.len() < full.len() && fits(&text) {
            return text;
        }
    }
    format!("{value:.1e}")
}

/// A value with all its digits, never rounded to fewer: very small or large numbers in
/// e-notation, others as they are. Only the noise of a unit conversion beyond 15
/// significant digits, such as 30.000000000000004, is dropped.
pub fn format_physical(value: f64) -> String {
    let value = format!("{value:.14e}").parse().unwrap_or(value);
    if value != 0.0 && !(1e-3..1e7).contains(&value.abs()) {
        format!("{value:e}")
    } else {
        format!("{value}")
    }
}

#[cfg(test)]
mod tests {
    use super::{fit, format_physical, parse_count, parse_number};
    use plx_model::{Quantity, UnitSystem};

    #[test]
    fn small_values_keep_their_digits() {
        // 5 mm in a metre model is 0.005 m, not the 0.01 m egui's decimals would show.
        let value = UnitSystem::MKgSC
            .parse_value("5mm", Quantity::Length)
            .unwrap();
        assert_eq!(value, 0.005);
        assert_eq!(format_physical(value), "0.005");
        assert_eq!(parse_number(&format_physical(value)), Some(value));
        assert_eq!(format_physical(0.000_25), "2.5e-4");
        assert_eq!(format_physical(1.234_567_891_234), "1.234567891234");
        assert_eq!(format_physical(2.1e11), "2.1e11");
        assert_eq!(format_physical(0.0), "0");
        assert_eq!(format_physical(-12.5), "-12.5");
        // Conversion noise does not show, the typed value does.
        let value = UnitSystem::MmTonSC
            .parse_value("3cm", Quantity::Length)
            .unwrap();
        assert_eq!(format_physical(value), "30");
    }

    #[test]
    fn accepts_point_and_comma() {
        assert_eq!(parse_number("1.5"), Some(1.5));
        assert_eq!(parse_number("1,5"), Some(1.5));
        assert_eq!(parse_number(" -0,25 "), Some(-0.25));
        assert_eq!(parse_number(",5"), Some(0.5));
        assert_eq!(parse_number("42"), Some(42.0));
        assert_eq!(parse_number("\u{2212}3"), Some(-3.0));
    }

    #[test]
    fn accepts_e_notation() {
        assert_eq!(parse_number("2,1e5"), Some(2.1e5));
        assert_eq!(parse_number("2.1E-3"), Some(2.1e-3));
        assert_eq!(parse_number("7,85e-9"), Some(7.85e-9));
        assert_eq!(parse_number("1e+3"), Some(1e3));
    }

    #[test]
    fn rejects_thousands_separators_and_junk() {
        assert_eq!(parse_number("1.000,5"), None);
        assert_eq!(parse_number("1,000.5"), None);
        assert_eq!(parse_number("1 000"), None);
        assert_eq!(parse_number(""), None);
        assert_eq!(parse_number("abc"), None);
        assert_eq!(parse_number("inf"), None);
        assert_eq!(parse_number("NaN"), None);
        assert_eq!(parse_number("1e999"), None);
    }

    #[test]
    fn counts_take_whole_results_only() {
        assert_eq!(parse_count("12/4"), Some(3.0));
        assert_eq!(parse_count("2*(3+4)"), Some(14.0));
        assert_eq!(parse_count("10/4"), None);
        assert_eq!(parse_count("x"), None);
    }

    #[test]
    fn fitted_numbers_keep_as_many_digits_as_fit() {
        let value = 0.091_000_273_437_5;
        assert_eq!(fit(value, |_| true), "0.0910002734375");
        assert_eq!(fit(value, |t| t.len() <= 9), "0.0910003");
        assert_eq!(fit(value, |t| t.len() <= 5), "0.091");
        assert_eq!(fit(2.0 / 3.0, |t| t.len() <= 6), "0.6667");
        assert_eq!(fit(123_456.789, |t| t.len() <= 6), "123457");
        assert_eq!(fit(1.234_567e-9, |t| t.len() <= 8), "1.235e-9");
        // Nothing fits: two significant digits.
        assert_eq!(fit(2.0 / 3.0, |_| false), "6.7e-1");
    }
}
