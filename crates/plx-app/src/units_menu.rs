//! The unit system of the project as a menu button: the text "Units: mm, ton, s, °C" at the
//! right end of the status bar of the project window and of the analysis workspaces, which
//! opens the list of unit systems when clicked.

use plx_model::{Quantity, UnitSystem};

/// The unit systems a project offers: all but Unitless, which would show the SI values
/// without their units.
pub const CHOICES: [UnitSystem; 5] = [
    UnitSystem::MKgSC,
    UnitSystem::MmTonSC,
    UnitSystem::MmKgSC,
    UnitSystem::MTonSC,
    UnitSystem::InLbSF,
];

/// The unit system a new project starts in: the one chosen last, if a project offers it.
pub fn first(last: UnitSystem) -> UnitSystem {
    if CHOICES.contains(&last) {
        last
    } else {
        UnitSystem::default()
    }
}

/// A few units of the system, to tell it in a tooltip.
pub fn summary(units: UnitSystem) -> String {
    let unit = |q| units.unit(q);
    format!(
        "Length {}, mass {}, time {}, temperature {}\nForce {}, moment {}, stress {}, density {}",
        unit(Quantity::Length),
        unit(Quantity::Mass),
        unit(Quantity::Time),
        unit(Quantity::Temperature),
        unit(Quantity::Force),
        unit(Quantity::Moment),
        unit(Quantity::Pressure),
        unit(Quantity::Density),
    )
}

/// The menu button of the unit system `current`; returns the one picked. It reads like the
/// other texts of the status bar: its frame shows only while the pointer is on it or the
/// menu is open, and has the same size then, so nothing beside it moves. `busy` says why
/// the units cannot change now, and greys it out.
pub fn button(ui: &mut egui::Ui, current: UnitSystem, busy: Option<&str>) -> Option<UnitSystem> {
    let mut chosen = None;
    let response = ui
        .scope(|ui| {
            if busy.is_some() {
                ui.disable();
            }
            // Not framed while inactive, but with a frame stroke that does not show: egui
            // leaves the stroke out of the size of an unframed button, which would make it
            // grow by its width on hover.
            let inactive = &mut ui.visuals_mut().widgets.inactive;
            inactive.weak_bg_fill = egui::Color32::TRANSPARENT;
            inactive.bg_fill = egui::Color32::TRANSPARENT;
            inactive.bg_stroke.color = egui::Color32::TRANSPARENT;
            let text = format!("Units: {}", current.label());
            let button = egui::Button::new(text);
            egui::containers::menu::MenuButton::from_button(button)
                .ui(ui, |ui| {
                    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                    for choice in CHOICES {
                        let response = ui
                            .selectable_label(choice == current, choice.label())
                            .on_hover_text(summary(choice));
                        if response.clicked() {
                            chosen = Some(choice);
                            ui.close();
                        }
                    }
                })
                .0
        })
        .inner;
    match busy {
        Some(why) => {
            response.on_disabled_hover_text(why);
        }
        None => {
            response.on_hover_text(format!(
                "The unit system the blocks and the analyses of the project show values in \
                 and take them typed. Values are stored in SI units: another unit system \
                 changes only what is shown.\n{}",
                summary(current)
            ));
        }
    }
    chosen.filter(|&units| units != current)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_new_project_starts_in_the_units_chosen_last_unless_unitless() {
        assert_eq!(first(UnitSystem::InLbSF), UnitSystem::InLbSF);
        assert_eq!(first(UnitSystem::Unitless), UnitSystem::default());
        assert!(CHOICES.iter().all(|u| u.has_units()));
    }

    #[test]
    fn the_summary_names_the_main_units() {
        let text = summary(UnitSystem::MmTonSC);
        assert!(text.contains("Length mm"), "{text}");
        assert!(text.contains("stress MPa"), "{text}");
    }
}
