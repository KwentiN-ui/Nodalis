//! The forms of contact regions and of the connections they share: the user picks two sides
//! and a type, and everything CalculiX needs besides is derived ("Auto"), shown and can be
//! overridden one setting at a time. Side A shows in the primary, side B in the secondary
//! highlight colour.

use egui::{RichText, Ui};
use plx_model::{
    BondedMethod, ContactBehavior, ContactCheck, ContactDefaults, ContactDetection, ContactMethod,
    ContactRegion, ContactStatus, DetectionGrouping, FeModel, InterfaceTreatment, Quantity,
    Resolved, ResolvedContacts, ResolvedKind, STORED, SideRoles, Stiffness, UnitSystem,
    auto_detection_tolerance, auto_geometry_detection_tolerance, conducts_across_contacts,
};

use crate::contacts::MasterSlave;
use crate::model::Model;
use crate::numeric;
use crate::tree::{TreeItem, TreeState, TreeView};

/// What the region dialog keeps besides the region: whether the advanced settings show and
/// the derivation of the region as last edited, computed again only when it changes.
#[derive(Clone, Debug, Default)]
pub struct RegionView {
    advanced: bool,
    /// Boxed, as the dialog holds it by value.
    resolved: Option<Box<(ContactRegion, ContactDefaults, Resolved)>>,
    /// The contact check of the region as last edited.
    check: Option<Box<(ContactRegion, ContactDefaults, Option<ContactCheck>)>>,
}

impl RegionView {
    fn resolved(&mut self, model: &Model, region: &ContactRegion) -> &Resolved {
        let defaults = &model.fe.contact_defaults;
        let stale =
            !matches!(self.resolved.as_deref(), Some((r, d, _)) if r == region && d == defaults);
        if stale {
            let resolved = model.fe.resolve_contact(region, &model.mesh);
            self.resolved = Some(Box::new((region.clone(), defaults.clone(), resolved)));
        }
        &self.resolved.as_ref().expect("resolved above").2
    }

    fn check(&mut self, model: &Model, region: &ContactRegion) -> Option<&ContactCheck> {
        let defaults = &model.fe.contact_defaults;
        let stale =
            !matches!(self.check.as_deref(), Some((r, d, _)) if r == region && d == defaults);
        if stale {
            let check = model.fe.check_contact(region, &model.mesh);
            self.check = Some(Box::new((region.clone(), defaults.clone(), check)));
        }
        self.check.as_ref().expect("checked above").2.as_ref()
    }
}

/// A value of a quantity in the model's units with the unit, e.g. "2.1e5 N/mm³".
pub fn with_unit(value: f64, units: UnitSystem, quantity: Quantity) -> String {
    let shown = STORED.convert(value, quantity, units);
    let unit = units.unit(quantity);
    let number = if shown != 0.0 && !(1e-2..1e5).contains(&shown.abs()) {
        format!("{shown:.3e}")
    } else {
        format!("{}", (shown * 1e4).round() / 1e4)
    };
    if unit.is_empty() {
        number
    } else {
        format!("{number} {unit}")
    }
}

/// The contact region dialog below the name: type, sides, advanced settings and what is
/// exported.
pub fn region_form(
    ui: &mut Ui,
    model: &Model,
    region: &mut ContactRegion,
    sides: &mut MasterSlave,
    view: &mut RegionView,
) {
    (region.side_a, region.side_b) = sides.regions();
    behavior_rows(ui, "contact region type", "Type", &mut region.behavior);
    sides.ui_named(ui, model, ["Side A", "Side B"]);
    (region.side_a, region.side_b) = sides.regions();
    ui.label("Side A color");
    ui.color_edit_button_srgb(&mut region.color_a);
    ui.end_row();
    ui.label("Side B color");
    ui.color_edit_button_srgb(&mut region.color_b);
    ui.end_row();
    let resolved = view.resolved(model, region).clone();
    ui.label("");
    ui.toggle_value(&mut view.advanced, "Advanced settings")
        .on_hover_text("Settings that are derived (Auto) unless you choose them.");
    ui.end_row();
    if view.advanced {
        advanced_rows(ui, model, region, &resolved);
    }
    ui.label("Exported as");
    ui.add(egui::Label::new(exported_as(model, &resolved)).wrap());
    ui.end_row();
    if !resolved.why.is_empty() {
        ui.label("");
        let text = resolved.why.join("\n");
        ui.add(egui::Label::new(RichText::new(text).weak()).wrap());
        ui.end_row();
    }
    if let Some(check) = view.check(model, region) {
        ui.label("Contact check").on_hover_text(
            "How the sides lie to each other in the mesh before the analysis: closed where \
             the slave nodes are within the tolerance of the master side.",
        );
        ui.vertical(|ui| {
            ui.label(status_text(check));
            let text = check_summary(check, model.fe.properties.units);
            ui.add(egui::Label::new(RichText::new(text).weak()).wrap());
        });
        ui.end_row();
    }
}

/// The type of a region with its friction coefficient and what it means.
fn behavior_rows(ui: &mut Ui, id: &str, label: &str, behavior: &mut ContactBehavior) {
    ui.label(label);
    egui::ComboBox::from_id_salt(id)
        .selected_text(behavior.name())
        .width(200.0)
        .show_ui(ui, |ui| {
            for choice in ContactBehavior::ALL {
                let selected = choice.name() == behavior.name();
                let response = ui.selectable_label(selected, choice.name());
                if response.on_hover_text(choice.summary()).clicked() && !selected {
                    *behavior = choice;
                }
            }
        });
    ui.end_row();
    if let ContactBehavior::Frictional { mu } = behavior {
        ui.label("Friction coefficient");
        ui.add(numeric::drag_value(mu).speed(0.01).range(0.0..=5.0));
        ui.end_row();
    }
    ui.label("");
    ui.add(egui::Label::new(RichText::new(behavior.summary()).weak()).wrap());
    ui.end_row();
}

/// The CAD geometry a contact detection searches, for its dialog.
#[derive(Clone, Debug, PartialEq)]
pub struct DetectionScope {
    /// The parts of the geometry and whether each is shown.
    pub parts: Vec<(String, bool)>,
    /// The parts searched, as the detection names them, and the derived tolerance for them.
    pub auto: (Vec<String>, f64),
}

impl DetectionScope {
    /// The scope of a detection of `parts` on the geometry display `geometry`.
    pub fn new(model: &Model, geometry: &Model, parts: &[String]) -> Self {
        let visible = detection_visibility(model, geometry);
        let searched: Vec<bool> = (geometry.mesh.parts.iter().zip(&visible))
            .map(|(part, &shown)| {
                if parts.is_empty() {
                    shown
                } else {
                    parts.contains(&part.name)
                }
            })
            .collect();
        let tolerance =
            auto_geometry_detection_tolerance(&geometry.mesh, geometry.skins(), &searched);
        Self {
            parts: (geometry.mesh.parts.iter().zip(visible))
                .map(|(part, shown)| (part.name.clone(), shown))
                .collect(),
            auto: (parts.to_vec(), tolerance),
        }
    }
}

/// The geometry display a contact detection searches instead of the mesh: the model's
/// geometry, if every meshed part comes from it. Detecting on the geometry finds the
/// regions before the parts are meshed and keeps them to whole CAD faces.
pub fn detection_display<'a>(model: &Model, geometry: Option<&'a Model>) -> Option<&'a Model> {
    let geometry = geometry.filter(|g| g.is_geometry() && !g.mesh.parts.is_empty())?;
    let from_geometry =
        |name: &str| (geometry.mesh.parts.iter()).any(|p| p.name.eq_ignore_ascii_case(name));
    (model.mesh.parts.iter())
        .all(|p| from_geometry(&p.name))
        .then_some(geometry)
}

/// Runs the contact detection `index` of the model: on the CAD geometry when every part
/// comes from it, see [`detection_display`], else on the mesh.
pub fn detect(
    model: &mut Model,
    geometry: Option<&Model>,
    index: usize,
) -> Option<plx_model::DetectionReport> {
    match detection_display(model, geometry) {
        Some(display) => {
            let visible = detection_visibility(model, display);
            let dimensions = display.part_dimensions();
            let geometry = plx_model::DetectionGeometry {
                mesh: &display.mesh,
                skins: display.skins(),
                entities: display.cad_entities(),
                face_edges: display.face_edges(),
                part_dimensions: &dimensions,
            };
            model
                .fe
                .detect_contacts_on_geometry(index, &geometry, &visible)
        }
        None => {
            let visible: Vec<bool> = model.parts.iter().map(|p| p.visible).collect();
            let mut fe = std::mem::take(&mut model.fe);
            let report = fe.detect_contacts(index, &model.mesh, model.skins(), &visible);
            model.fe = fe;
            report
        }
    }
}

/// Whether each part of the geometry display is shown: as in the FE model once it is
/// meshed, else as in the Geometry tab.
pub fn detection_visibility(model: &Model, geometry: &Model) -> Vec<bool> {
    (geometry.mesh.parts.iter().zip(&geometry.parts))
        .map(|(part, shown)| {
            (model.mesh.parts.iter().zip(&model.parts))
                .find(|(p, _)| p.name.eq_ignore_ascii_case(&part.name))
                .map_or(shown.visible, |(_, s)| s.visible)
        })
        .collect()
}

/// The contact detection dialog below the name: which parts it searches, how close and
/// how parallel surfaces must be to touch, and the type of the regions it finds. With a
/// `scope` it searches the CAD geometry.
pub fn detection_form(
    ui: &mut Ui,
    model: &Model,
    detection: &mut ContactDetection,
    scope: Option<&DetectionScope>,
) {
    let units = model.fe.properties.units;
    let parts: Vec<(String, bool)> = match scope {
        Some(scope) => scope.parts.clone(),
        None => (model.mesh.parts.iter().zip(&model.parts))
            .map(|(part, shown)| (part.name.clone(), shown.visible))
            .collect(),
    };
    ui.label("Parts");
    ui.vertical(|ui| {
        let mut all = detection.parts.is_empty();
        if ui
            .checkbox(&mut all, "All visible parts")
            .on_hover_text("Untick to choose the parts to search.")
            .changed()
        {
            detection.parts = if all {
                Vec::new()
            } else {
                (parts.iter())
                    .filter(|(_, shown)| *shown)
                    .map(|(name, _)| name.clone())
                    .collect()
            };
        }
        if !all {
            egui::ScrollArea::vertical()
                .max_height(160.0)
                .id_salt("detection parts")
                .show(ui, |ui| {
                    for (name, _) in &parts {
                        let mut on = detection.parts.contains(name);
                        if ui.checkbox(&mut on, name).changed() {
                            if on {
                                detection.parts.push(name.clone());
                            } else {
                                detection.parts.retain(|p| p != name);
                            }
                        }
                    }
                });
        }
    });
    ui.end_row();
    ui.label("Tolerance").on_hover_text(if scope.is_some() {
        "Largest gap between surfaces that touch. Auto: 0.5 % of the size of the parts \
         searched."
    } else {
        "Largest gap between surfaces that touch. Auto: 0.5 % of the size of the parts \
         searched, at least half their element size."
    });
    ui.horizontal(|ui| {
        let auto = match scope {
            Some(scope) => scope.auto.1,
            None => auto_tolerance(ui, model, &detection.parts),
        };
        let mut set = detection.tolerance.is_some();
        ui.checkbox(&mut set, "").on_hover_text(format!(
            "Untick for Auto ({})",
            with_unit(auto, units, Quantity::Length)
        ));
        if set {
            let mut value = detection.tolerance.unwrap_or(auto);
            ui.add(numeric::physical(&mut value, units, Quantity::Length));
            detection.tolerance = Some(value);
        } else {
            detection.tolerance = None;
            ui.weak(format!(
                "Auto ({})",
                with_unit(auto, units, Quantity::Length)
            ));
        }
    });
    ui.end_row();
    ui.label("Angle").on_hover_text(
        "Largest angle between the normals of surfaces facing each other, in degrees.",
    );
    ui.add(
        numeric::drag_value(&mut detection.angle_deg)
            .speed(1.0)
            .range(0.0..=90.0)
            .suffix(" °"),
    );
    ui.end_row();
    if !model.fe.properties.space.is_2d() {
        ui.label("Surfaces");
        ui.horizontal(|ui| {
            let filter = &mut detection.filter;
            ui.checkbox(&mut filter.solid, "Solid")
                .on_hover_text("Outer faces of solids.");
            ui.checkbox(&mut filter.shell, "Shell")
                .on_hover_text("Top and bottom of shells and membranes.");
            ui.checkbox(&mut filter.shell_edge, "Shell edge")
                .on_hover_text("Free edges of shells and membranes.");
        });
        ui.end_row();
    }
    ui.label("Group by");
    egui::ComboBox::from_id_salt("detection grouping")
        .selected_text(detection.grouping.name())
        .show_ui(ui, |ui| {
            for (grouping, hint) in [
                (
                    DetectionGrouping::Parts,
                    "One region per pair of touching parts.",
                ),
                (
                    DetectionGrouping::Faces,
                    "One region per pair of touching faces.",
                ),
                (
                    DetectionGrouping::Graph,
                    "Touching faces sharing nodes are merged, so that every node is on the \
                     slave side of one region only.",
                ),
            ] {
                ui.selectable_value(&mut detection.grouping, grouping, grouping.name())
                    .on_hover_text(hint);
            }
        });
    ui.end_row();
    behavior_rows(ui, "detection type", "Type", &mut detection.behavior);
    ui.label("");
    let searches = if scope.is_some() {
        "OK searches the CAD geometry of the parts, meshed or not: a region takes the whole \
         faces that touch and keeps them when the parts are meshed again. It replaces the \
         regions found before, except those you changed since."
    } else {
        "OK searches the parts and replaces the regions found before, except those you \
         changed since."
    };
    ui.add(egui::Label::new(RichText::new(searches).weak()).wrap());
    ui.end_row();
}

/// The derived tolerance of a detection of `parts`, kept while they and the mesh stay the
/// same: it walks the skins of all parts searched.
fn auto_tolerance(ui: &Ui, model: &Model, parts: &[String]) -> f64 {
    let searched: Vec<bool> = (model.mesh.parts.iter().zip(&model.parts))
        .map(|(part, shown)| {
            if parts.is_empty() {
                shown.visible
            } else {
                parts.contains(&part.name)
            }
        })
        .collect();
    let key = (
        searched.clone(),
        model.mesh.coords().len(),
        model.mesh.elements().len(),
    );
    let id = egui::Id::new("detection auto tolerance");
    let cached: Option<(Vec<bool>, usize, usize, f64)> = ui.data(|d| d.get_temp(id));
    if let Some((s, n, e, value)) = cached
        && (s, n, e) == key
    {
        return value;
    }
    let value = auto_detection_tolerance(&model.mesh, model.skins(), &searched);
    ui.data_mut(|d| d.insert_temp(id, (key.0, key.1, key.2, value)));
    value
}

/// Why the detection cannot be applied as it is.
pub fn validate_detection(detection: &ContactDetection) -> Result<(), String> {
    if detection
        .tolerance
        .is_some_and(|t| !(t.is_finite() && t > 0.0))
    {
        return Err("The tolerance must be positive.".into());
    }
    let filter = detection.filter;
    if !(filter.solid || filter.shell || filter.shell_edge) {
        return Err("Choose at least one kind of surface to search.".into());
    }
    if let ContactBehavior::Frictional { mu } = detection.behavior
        && !(mu.is_finite() && mu >= 0.0)
    {
        return Err("The friction coefficient must not be negative.".into());
    }
    Ok(())
}

/// One line on what the region becomes, e.g. "tie constraint (linear), slave: side B,
/// tolerance 1.25 mm".
fn exported_as(model: &Model, resolved: &Resolved) -> String {
    let units = model.fe.properties.units;
    let mut text = format!(
        "{}, master: side {}",
        resolved.kind.describe(),
        if resolved.a_is_master { "A" } else { "B" }
    );
    if let Some(k) = resolved.normal_stiffness() {
        text += &format!(
            ", stiffness {}",
            with_unit(k, units, Quantity::ForcePerVolume)
        );
    }
    if let ResolvedKind::Tie {
        tolerance: Some(t), ..
    } = resolved.kind
    {
        text += &format!(", tolerance {}", with_unit(t, units, Quantity::Length));
    }
    if resolved.adjusts() {
        text += ", adjusted to touch";
    }
    if let ResolvedKind::Pair {
        clearance,
        conductance,
        ..
    } = resolved.kind
    {
        if let Some(offset) = clearance {
            text += &format!(", offset {}", with_unit(offset, units, Quantity::Length));
        }
        if let Some(c) = conductance {
            text += &format!(
                ", heat conductance {}",
                with_unit(c, units, Quantity::HeatTransferCoefficient)
            );
        }
    }
    text
}

/// A choice that is derived unless the user picks one of `options`.
fn auto_combo<T: Copy + PartialEq>(
    ui: &mut Ui,
    id: &str,
    label: &str,
    value: &mut Option<T>,
    auto: String,
    options: &[(T, &str)],
) {
    ui.label(label);
    let text = match *value {
        None => format!("Auto ({auto})"),
        Some(v) => (options.iter())
            .find(|(o, _)| *o == v)
            .map_or(String::new(), |(_, name)| (*name).to_owned()),
    };
    egui::ComboBox::from_id_salt(id)
        .selected_text(text)
        .width(200.0)
        .show_ui(ui, |ui| {
            ui.selectable_value(value, None, format!("Auto ({auto})"));
            for &(option, name) in options {
                ui.selectable_value(value, Some(option), name);
            }
        });
    ui.end_row();
}

/// A stiffness that is derived, a factor on E / h or a value.
fn stiffness_row(
    ui: &mut Ui,
    id: &str,
    label: &str,
    value: &mut Option<Stiffness>,
    auto: Option<f64>,
    units: UnitSystem,
) {
    ui.label(label);
    ui.horizontal(|ui| {
        let auto_text = auto.map_or("CalculiX".into(), |k| {
            with_unit(k, units, Quantity::ForcePerVolume)
        });
        let kind = match value {
            None => 0,
            Some(Stiffness::Factor(_)) => 1,
            Some(Stiffness::Value(_)) => 2,
        };
        let names = [
            format!("Auto ({auto_text})"),
            "Factor of E/h".into(),
            "Value".into(),
        ];
        let mut chosen = kind;
        egui::ComboBox::from_id_salt(id)
            .selected_text(names[kind].clone())
            .width(170.0)
            .show_ui(ui, |ui| {
                for (k, name) in names.iter().enumerate() {
                    ui.selectable_value(&mut chosen, k, name);
                }
            });
        if chosen != kind {
            *value = match chosen {
                1 => Some(Stiffness::Factor(plx_model::STIFFNESS_FACTOR)),
                2 => Some(Stiffness::Value(auto.unwrap_or(1e12))),
                _ => None,
            };
        }
        match value {
            Some(Stiffness::Factor(f)) => {
                ui.add(numeric::drag_value(f).speed(0.5).range(1e-6..=1e6));
            }
            Some(Stiffness::Value(k)) => {
                ui.add(numeric::physical(k, units, Quantity::ForcePerVolume));
            }
            None => {}
        }
    });
    ui.end_row();
}

fn advanced_rows(ui: &mut Ui, model: &Model, region: &mut ContactRegion, resolved: &Resolved) {
    let units = model.fe.properties.units;
    let settings = &mut region.settings;
    let master = if resolved.a_is_master {
        "side A"
    } else {
        "side B"
    };
    auto_combo(
        ui,
        "region master",
        "Master",
        &mut settings.sides,
        master.into(),
        &[
            (SideRoles::AIsMaster, "Side A"),
            (SideRoles::BIsMaster, "Side B"),
        ],
    );
    let tie = matches!(resolved.kind, ResolvedKind::Tie { .. });
    let pair = matches!(resolved.kind, ResolvedKind::Pair { .. });
    if region.behavior == ContactBehavior::Bonded {
        let auto = if tie { "constraint" } else { "contact" };
        auto_combo(
            ui,
            "region bonded",
            "Bonded as",
            &mut settings.bonded_method,
            auto.into(),
            &BondedMethod::ALL.map(|m| (m, m.name())),
        );
    }
    if pair {
        stiffness_row(
            ui,
            "region normal",
            "Normal stiffness",
            &mut settings.normal_stiffness,
            resolved.normal_stiffness(),
            units,
        );
        let friction = matches!(
            &resolved.kind,
            ResolvedKind::Pair {
                friction: Some(_),
                ..
            }
        );
        if friction {
            stiffness_row(
                ui,
                "region tangential",
                "Tangential stiffness",
                &mut settings.tangential_stiffness,
                resolved.tangential_stiffness(),
                units,
            );
        }
    }
    if region.behavior != ContactBehavior::NoSeparation {
        interface_row(ui, &mut settings.interface, resolved, units);
    }
    ui.label("Tolerance").on_hover_text(
        "Distance within which the sides count as touching: tied by a tie, closed by \
         the adjustment.",
    );
    ui.horizontal(|ui| {
        let mut set = settings.tolerance.is_some();
        let auto = resolved
            .tolerance()
            .or(resolved.mesh_size.map(|h| 0.25 * h));
        let auto_text = auto.map_or("CalculiX".into(), |t| with_unit(t, units, Quantity::Length));
        ui.checkbox(&mut set, "")
            .on_hover_text(format!("Untick for Auto ({auto_text})"));
        if set {
            let mut value = settings.tolerance.or(auto).unwrap_or(0.0);
            ui.add(numeric::physical(&mut value, units, Quantity::Length));
            settings.tolerance = Some(value);
        } else {
            settings.tolerance = None;
            ui.weak(format!("Auto ({auto_text})"));
        }
    });
    ui.end_row();
    let thermal = (model.fe.steps.iter()).any(|s| s.active && conducts_across_contacts(&s.kind));
    if pair && (thermal || settings.conductance.is_some()) {
        conductance_row(ui, &mut settings.conductance, resolved, units);
    }
    if model.fe.contact_formulation().0 == ContactMethod::NodeToSurface && pair {
        let auto = match &resolved.kind {
            ResolvedKind::Pair { small_sliding, .. } if *small_sliding => "on",
            _ => "off",
        };
        auto_combo(
            ui,
            "region small sliding",
            "Small sliding",
            &mut settings.small_sliding,
            auto.into(),
            &[(true, "On"), (false, "Off")],
        );
    }
}

/// What happens to an initial gap: derived, as is, adjusted to touch or an offset with its
/// value.
fn interface_row(
    ui: &mut Ui,
    value: &mut Option<InterfaceTreatment>,
    resolved: &Resolved,
    units: UnitSystem,
) {
    ui.label("Initial gap").on_hover_text(
        "Adjust to touch closes gaps within the tolerance at the start. An offset sets the \
         distance of the sides wherever they face each other, whatever the geometry: \
         negative for an interference fit (surface to surface contact only).",
    );
    ui.horizontal(|ui| {
        let auto = format!(
            "Auto ({})",
            if resolved.adjusts() {
                "adjust to touch"
            } else {
                "as is"
            }
        );
        let text = value.map_or(auto.clone(), |t| t.name().to_owned());
        egui::ComboBox::from_id_salt("region interface")
            .selected_text(text)
            .width(170.0)
            .show_ui(ui, |ui| {
                ui.selectable_value(value, None, auto);
                for option in InterfaceTreatment::ALL {
                    let selected = value.is_some_and(|v| v.same_kind(option));
                    if ui.selectable_label(selected, option.name()).clicked() && !selected {
                        *value = Some(option);
                    }
                }
            });
        if let Some(InterfaceTreatment::Offset(offset)) = value {
            ui.add(numeric::physical(offset, units, Quantity::Length));
        }
    });
    ui.end_row();
}

/// Heat conductance across the contact, derived or a value.
fn conductance_row(ui: &mut Ui, value: &mut Option<f64>, resolved: &Resolved, units: UnitSystem) {
    let auto = match resolved.kind {
        ResolvedKind::Pair {
            conductance: Some(c),
            ..
        } if value.is_none() => Some(c),
        _ => (resolved.conductivity.zip(resolved.mesh_size))
            .map(|(k, h)| plx_model::CONDUCTANCE_FACTOR * k / h),
    };
    ui.label("Heat conductance").on_hover_text(format!(
        "Heat flow across the closed contact per area and temperature difference, in steps \
         with temperatures and displacements. Auto is {} k / h of the less conducting \
         side: nearly ideal.",
        plx_model::CONDUCTANCE_FACTOR
    ));
    ui.horizontal(|ui| {
        let mut set = value.is_some();
        let auto_text = auto.map_or("none, no conductivity".into(), |c| {
            with_unit(c, units, Quantity::HeatTransferCoefficient)
        });
        ui.checkbox(&mut set, "")
            .on_hover_text(format!("Untick for Auto ({auto_text})"));
        if set {
            let mut c = value.or(auto).unwrap_or(0.0);
            ui.add(numeric::physical(
                &mut c,
                units,
                Quantity::HeatTransferCoefficient,
            ));
            *value = Some(c);
        } else {
            *value = None;
            ui.weak(format!("Auto ({auto_text})"));
        }
    });
    ui.end_row();
}

/// The settings all contact regions share, at the Connections of the tree.
pub fn connections_form(ui: &mut Ui, fe: &FeModel, defaults: &mut ContactDefaults) {
    let mut derived = fe.clone();
    derived.contact_defaults.formulation = None;
    let (auto, reason) = derived.contact_formulation();
    ui.label("Formulation");
    egui::ComboBox::from_id_salt("connections formulation")
        .selected_text(match defaults.formulation {
            None => format!("Auto ({})", auto.name()),
            Some(m) => m.name().to_owned(),
        })
        .width(220.0)
        .show_ui(ui, |ui| {
            ui.selectable_value(
                &mut defaults.formulation,
                None,
                format!("Auto ({})", auto.name()),
            )
            .on_hover_text(reason);
            for method in [
                ContactMethod::SurfaceToSurface,
                ContactMethod::NodeToSurface,
                ContactMethod::Mortar,
            ] {
                ui.selectable_value(&mut defaults.formulation, Some(method), method.name())
                    .on_hover_text(method.summary());
            }
        });
    ui.end_row();
    ui.label("");
    let description = defaults.formulation.unwrap_or(auto).description();
    ui.add(
        egui::Label::new(
            RichText::new(format!(
                "{description}\n\nCalculiX takes one formulation for all contacts of a model. \
             Bonded regions written as ties are not affected."
            ))
            .weak(),
        )
        .wrap(),
    );
    ui.end_row();
    ui.label("Stiffness factor").on_hover_text(
        "Normal stiffness of the contact regions as a factor on E / h, E the Young's \
         modulus of the softer side and h its mesh size. Larger is stiffer: less \
         penetration, but harder to converge.",
    );
    ui.add(
        numeric::drag_value(&mut defaults.stiffness_factor)
            .speed(0.5)
            .range(1e-3..=1e4),
    );
    ui.end_row();
    ui.label("Weak springs").on_hover_text(format!(
        "Parts held only by contacts get very soft springs to ground in the input file, \
         {:e} E L per direction spread over their nodes (E the Young's modulus, L the size \
         of the part). They keep the parts from drifting before the contacts close, \
         which otherwise often ends in \"too many cutbacks\" with frictionless contact, \
         an initial gap or Nlgeom, and carry about a millionth of the load.",
        plx_model::WEAK_SPRING_FACTOR
    ));
    ui.checkbox(&mut defaults.weak_springs, "For parts held only by contact");
    ui.end_row();
}

/// Why the region cannot be applied as it is.
pub fn validate_region(region: &ContactRegion, sides: &MasterSlave) -> Result<(), String> {
    sides.validate_named(["Side A", "Side B"])?;
    if let ContactBehavior::Frictional { mu } = region.behavior
        && !(mu.is_finite() && mu >= 0.0)
    {
        return Err("The friction coefficient must not be negative.".into());
    }
    for stiffness in [
        region.settings.normal_stiffness,
        region.settings.tangential_stiffness,
    ]
    .into_iter()
    .flatten()
    {
        let (Stiffness::Factor(v) | Stiffness::Value(v)) = stiffness;
        if !(v.is_finite() && v > 0.0) {
            return Err("Stiffnesses must be greater than 0.".into());
        }
    }
    if region
        .settings
        .tolerance
        .is_some_and(|t| !(t.is_finite() && t >= 0.0))
    {
        return Err("The tolerance must not be negative.".into());
    }
    if let Some(InterfaceTreatment::Offset(offset)) = region.settings.interface
        && !offset.is_finite()
    {
        return Err("The offset must be a number.".into());
    }
    if (region.settings.conductance).is_some_and(|c| !(c.is_finite() && c >= 0.0)) {
        return Err("The heat conductance must not be negative.".into());
    }
    Ok(())
}

/// The derivation of all contact regions of the model, kept until the model changes.
fn resolved_contacts(ctx: &egui::Context, model: &Model) -> std::sync::Arc<ResolvedContacts> {
    type Cache = std::sync::Arc<(FeModel, std::sync::Arc<ResolvedContacts>)>;
    let id = egui::Id::new("contact worksheet resolved");
    let cached: Option<Cache> = ctx.data(|d| d.get_temp(id));
    if let Some(cache) = cached
        && cache.0 == model.fe
    {
        return cache.1.clone();
    }
    let resolved = std::sync::Arc::new(model.fe.resolve_contacts(&model.mesh));
    let cache: Cache = std::sync::Arc::new((model.fe.clone(), resolved.clone()));
    ctx.data_mut(|d| d.insert_temp(id, cache));
    resolved
}

/// The contact check of all regions, computed again when the model or the mesh changes.
fn contact_checks(ctx: &egui::Context, model: &Model) -> std::sync::Arc<Vec<Option<ContactCheck>>> {
    type Key = (FeModel, usize, usize);
    type Cache = std::sync::Arc<(Key, std::sync::Arc<Vec<Option<ContactCheck>>>)>;
    let id = egui::Id::new("contact worksheet checks");
    let key = (
        model.fe.clone(),
        model.mesh.coords().len(),
        model.mesh.elements().len(),
    );
    let cached: Option<Cache> = ctx.data(|d| d.get_temp(id));
    if let Some(cache) = cached
        && cache.0 == key
    {
        return cache.1.clone();
    }
    let checks = std::sync::Arc::new(model.fe.check_contacts(&model.mesh));
    let cache: Cache = std::sync::Arc::new((key, checks.clone()));
    ctx.data_mut(|d| d.insert_temp(id, cache));
    checks
}

/// The status of a contact check in its colour: green closed, yellow partly closed or near
/// open, red far open.
pub fn status_text(check: &ContactCheck) -> RichText {
    let color = match check.status {
        ContactStatus::Closed => egui::Color32::from_rgb(0, 150, 60),
        ContactStatus::PartlyClosed | ContactStatus::NearOpen => {
            egui::Color32::from_rgb(200, 140, 0)
        }
        ContactStatus::FarOpen => crate::style::error(),
    };
    RichText::new(check.status.name()).color(color)
}

/// Gap and overlap of a contact check in a few words, e.g. "gap 0.01 mm".
pub fn gap_text(check: &ContactCheck, units: UnitSystem) -> String {
    let length = |v: f64| with_unit(v, units, Quantity::Length);
    // Below a thousandth of the tolerance is rounding of the coordinates.
    let noise = 1e-3 * check.tolerance;
    let mut parts = Vec::new();
    if let Some(p) = check.max_penetration.filter(|&p| p > noise) {
        parts.push(format!("overlap {}", length(p)));
    }
    match (check.max_gap, check.min_gap) {
        (Some(g), _) if g > noise => parts.push(format!("gap up to {}", length(g))),
        (_, Some(g)) if check.touching == 0 => parts.push(format!("gap {}", length(g))),
        _ => {}
    }
    parts.join(", ")
}

/// The contact check in a sentence for the region dialog and the worksheet's tooltip.
pub fn check_summary(check: &ContactCheck, units: UnitSystem) -> String {
    let mut text = format!(
        "{} of {} slave nodes within {} of the master side",
        check.touching,
        check.slave_nodes,
        with_unit(check.tolerance, units, Quantity::Length)
    );
    let gap = gap_text(check, units);
    if !gap.is_empty() {
        text += &format!("; {gap}");
    }
    if check.status == ContactStatus::FarOpen {
        text += "; nothing within ten times that";
    }
    text
}

/// What CalculiX found in a contact region, from the contact results (`CONTACT`) of the
/// last increment that has them, at the slave nodes.
#[derive(Clone, Debug, PartialEq)]
pub struct SolvedContact {
    pub slave_nodes: usize,
    /// Slave nodes with contact pressure, and of those the ones that slide: whose shear
    /// reaches the friction coefficient times the pressure.
    pub in_contact: usize,
    pub sliding: usize,
    /// Largest contact pressure and largest opening, in the units of the results.
    pub max_pressure: f32,
    pub max_open: f32,
}

impl SolvedContact {
    /// "Sticking", "Sliding", "Sticking and sliding" or "Open".
    pub fn status(&self) -> &'static str {
        match (self.in_contact, self.sliding) {
            (0, _) => "Open",
            (_, 0) => "Sticking",
            (n, s) if s == n => "Sliding",
            _ => "Sticking and sliding",
        }
    }

    pub fn summary(&self, units: UnitSystem) -> String {
        let unit = crate::results::field_unit("CONTACT", "CPRESS", units).unwrap_or_default();
        let length = crate::results::field_unit("CONTACT", "COPEN", units).unwrap_or_default();
        let mut text = format!(
            "{}: {} of {} slave nodes in contact",
            self.status(),
            self.in_contact,
            self.slave_nodes
        );
        if self.sliding > 0 {
            text += &format!(", {} sliding", self.sliding);
        }
        if self.in_contact > 0 {
            text += &format!(
                ", max pressure {} {unit}",
                crate::results::format_value(self.max_pressure)
            );
        }
        if self.max_open > 0.0 {
            text += &format!(
                ", max opening {} {length}",
                crate::results::format_value(self.max_open)
            );
        }
        text
    }
}

/// The contact results of every region of `fe` in `results`; `None` for a region written
/// as a tie, which has none, or where the results hold no contact field.
pub fn solved_contacts(
    fe: &FeModel,
    mesh: &plx_mesh::FeMesh,
    resolved: &ResolvedContacts,
    results: &Model,
) -> Vec<Option<SolvedContact>> {
    let field = (results.results.as_ref()).and_then(|view| {
        view.increments
            .iter()
            .rev()
            .find_map(|i| i.field("CONTACT"))
    });
    let component = |name: &str| field.and_then(|f| f.component(name)).map(|c| &c.values);
    let (pressure, open) = (component("CPRESS"), component("COPEN"));
    let shear = [component("CSHEAR1"), component("CSHEAR2")];
    (fe.contact_regions.iter().zip(&resolved.items))
        .map(|(region, resolved)| {
            let ResolvedKind::Pair { friction, .. } = &resolved.kind else {
                return None;
            };
            let pressure = pressure?;
            let slave = if resolved.a_is_master {
                &region.side_b
            } else {
                &region.side_a
            };
            let mu = friction.as_ref().map(|f| f.coefficient as f32);
            let mut solved = SolvedContact {
                slave_nodes: 0,
                in_contact: 0,
                sliding: 0,
                max_pressure: 0.0,
                max_open: 0.0,
            };
            for id in slave.nodes(mesh) {
                let Some(index) = results.mesh.node_index(id) else {
                    continue;
                };
                solved.slave_nodes += 1;
                let value = |values: Option<&Vec<f32>>| {
                    values
                        .and_then(|v| v.get(index).copied())
                        .filter(|v| v.is_finite())
                        .unwrap_or(0.0)
                };
                let p = value(Some(pressure));
                if p > 0.0 {
                    solved.in_contact += 1;
                    solved.max_pressure = solved.max_pressure.max(p);
                    let tau = value(shear[0]).hypot(value(shear[1]));
                    if mu.is_some_and(|mu| tau >= 0.98 * mu * p) {
                        solved.sliding += 1;
                    }
                } else {
                    solved.max_open = solved.max_open.max(value(open));
                }
            }
            Some(solved)
        })
        .collect()
}

/// What the worksheet does to the selected regions.
enum Change {
    Behavior(ContactBehavior),
    Friction(f64),
    Flip,
    Active(bool),
    Rename,
    Delete,
}

/// Ansys' contact worksheet: a row per contact region with its type, sides and what it is
/// exported as. Clicking rows selects the regions in the tree (Ctrl and Shift for several),
/// which shows them in the 3D view; the buttons change all selected regions at once.
pub fn worksheet(
    ctx: &egui::Context,
    open: &mut bool,
    model: &mut Model,
    results: Option<&Model>,
    tree: &mut TreeState,
) {
    let view = TreeView::FeModel;
    let rows: Vec<TreeItem> = (0..model.fe.contact_regions.len())
        .map(TreeItem::ContactRegion)
        .collect();
    let selected: Vec<usize> = (tree.selected_items(view).into_iter())
        .filter_map(|item| match item {
            TreeItem::ContactRegion(i) if !tree.linked.contains_key(&item) => Some(i),
            _ => None,
        })
        .collect();
    let resolved = resolved_contacts(ctx, model);
    let checks = contact_checks(ctx, model);
    let solved = results.map(|r| solved_contacts(&model.fe, &model.mesh, &resolved, r));
    let units = model.fe.properties.units;
    let mut change = None;
    let mu_id = egui::Id::new("contact worksheet mu");
    let mut mu: f64 = ctx.data(|d| d.get_temp(mu_id)).unwrap_or(0.15);
    let mut close = false;
    egui::Window::new("Contact Worksheet")
        .id(crate::ids::window(ctx, "contact worksheet"))
        .open(open)
        .collapsible(false)
        .default_width(900.0)
        .default_height(360.0)
        .resizable(true)
        .show(ctx, |ui| {
            ui.label(format!(
                "Formulation of all contacts: {}",
                resolved.formulation_reason
            ));
            ui.horizontal_wrapped(|ui| {
                ui.add_enabled_ui(!selected.is_empty(), |ui| {
                    ui.label(format!("{} selected:", selected.len()));
                    ui.menu_button("Set Type", |ui| {
                        for behavior in ContactBehavior::ALL {
                            if ui.button(behavior.name()).clicked() {
                                change = Some(Change::Behavior(behavior));
                                ui.close();
                            }
                        }
                    });
                    ui.add(numeric::drag_value(&mut mu).speed(0.01).range(0.0..=5.0));
                    if ui
                        .button("Set Friction")
                        .on_hover_text(
                            "Makes the selected regions frictional with this coefficient.",
                        )
                        .clicked()
                    {
                        change = Some(Change::Friction(mu));
                    }
                    if ui.button("Flip Sides").clicked() {
                        change = Some(Change::Flip);
                    }
                    if ui.button("Activate").clicked() {
                        change = Some(Change::Active(true));
                    }
                    if ui.button("Deactivate").clicked() {
                        change = Some(Change::Active(false));
                    }
                    if ui
                        .button("Name from Definition")
                        .on_hover_text(
                            "Names the regions by their type and the parts of their sides.",
                        )
                        .clicked()
                    {
                        change = Some(Change::Rename);
                    }
                    if ui.button("Delete").clicked() {
                        change = Some(Change::Delete);
                    }
                });
            });
            ui.separator();
            // A work window: Close at the bottom right, below the table.
            close = crate::dialog::close_bar(ui, "contact worksheet");
            if rows.is_empty() {
                ui.weak(
                    "No contact regions. Create one with Contact Region on the Connections tab.",
                );
                return;
            }
            egui::ScrollArea::both()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    egui::Grid::new("contact worksheet grid")
                        .striped(true)
                        .num_columns(if solved.is_some() { 12 } else { 11 })
                        .spacing([14.0, 4.0])
                        .show(ui, |ui| {
                            let after = solved.as_ref().map(|_| "After solving");
                            for title in [
                                "Name",
                                "Detection",
                                "Type",
                                "Side A",
                                "Side B",
                                "Master",
                                "Initial status",
                                "Touching",
                                "Gap",
                                "Exported as",
                                "Active",
                            ]
                            .into_iter()
                            .chain(after)
                            {
                                ui.strong(title);
                            }
                            ui.end_row();
                            for (i, region) in model.fe.contact_regions.iter().enumerate() {
                                let item = TreeItem::ContactRegion(i);
                                let is_selected = tree.selected_items(view).contains(&item);
                                let response = ui.selectable_label(is_selected, &region.name);
                                if response.clicked() {
                                    let modifiers = ui.input(|i| i.modifiers);
                                    if modifiers.command || modifiers.shift {
                                        tree.select_sibling(
                                            view,
                                            item.clone(),
                                            &rows,
                                            modifiers.command,
                                            modifiers.shift,
                                        );
                                    } else {
                                        tree.select_item(view, item.clone());
                                    }
                                }
                                if response.double_clicked() {
                                    tree.reveal = true;
                                }
                                match &region.detected_by {
                                    Some(name) if region.edited => {
                                        ui.label(format!("{name}, changed")).on_hover_text(
                                            "Changed by you: detecting again keeps it.",
                                        );
                                    }
                                    Some(name) => {
                                        ui.label(name);
                                    }
                                    None => {
                                        ui.weak("by hand");
                                    }
                                }
                                let behavior = match region.behavior {
                                    ContactBehavior::Frictional { mu } => {
                                        format!("Frictional ({mu})")
                                    }
                                    other => other.name().to_owned(),
                                };
                                ui.label(behavior);
                                ui.label(region.side_a.describe());
                                ui.label(region.side_b.describe());
                                let master = resolved.items.get(i).map(|r| r.a_is_master);
                                ui.label(match master {
                                    Some(true) => "A",
                                    Some(false) => "B",
                                    None => "",
                                });
                                match checks.get(i).and_then(Option::as_ref) {
                                    Some(check) => {
                                        let summary = check_summary(check, units);
                                        ui.label(status_text(check)).on_hover_text(&summary);
                                        ui.label(format!(
                                            "{} %",
                                            (100.0 * check.touching_share()).round()
                                        ))
                                        .on_hover_text(&summary);
                                        ui.label(gap_text(check, units));
                                    }
                                    None => {
                                        ui.weak("-")
                                            .on_hover_text("The sides have no element faces.");
                                        ui.label("");
                                        ui.label("");
                                    }
                                }
                                match resolved.items.get(i) {
                                    Some(r) => {
                                        ui.label(r.kind.describe())
                                            .on_hover_text(exported_hover(r, units));
                                    }
                                    None => {
                                        ui.label("");
                                    }
                                }
                                ui.label(if region.active { "Yes" } else { "No" });
                                if let Some(solved) = &solved {
                                    match solved.get(i).and_then(Option::as_ref) {
                                        Some(s) => {
                                            ui.label(s.status()).on_hover_text(s.summary(units));
                                        }
                                        None => {
                                            ui.weak("-").on_hover_text(
                                                "A tie, or no contact results in the results \
                                                 file.",
                                            );
                                        }
                                    }
                                }
                                ui.end_row();
                            }
                        });
                });
        });
    if close {
        *open = false;
    }
    ctx.data_mut(|d| d.insert_temp(mu_id, mu));
    let Some(change) = change else {
        return;
    };
    let fe = &mut model.fe;
    if let Change::Delete = change {
        let mut indices = selected;
        indices.sort_unstable();
        for i in indices.into_iter().rev() {
            if i < fe.contact_regions.len() {
                let region = fe.contact_regions.remove(i);
                fe.remove_contact_changes(&region.name);
            }
        }
        tree.selected = None;
        return;
    }
    let names: Vec<String> = fe.contact_regions.iter().map(|r| r.name.clone()).collect();
    for &i in &selected {
        let Some(region) = fe.contact_regions.get_mut(i) else {
            continue;
        };
        match change {
            Change::Behavior(behavior) => {
                // Frictional keeps its coefficient.
                if behavior.name() != region.behavior.name() {
                    region.set_behavior(behavior);
                }
            }
            Change::Friction(mu) => region.set_behavior(ContactBehavior::Frictional { mu }),
            Change::Flip => region.flip(),
            Change::Active(active) => region.active = active,
            Change::Rename => region.name = region.name_from_definition(&model.mesh),
            Change::Delete => {}
        }
        // A detected region the user changed is kept when detecting again.
        region.edited |= region.detected_by.is_some();
    }
    if (fe.contact_regions.iter().zip(&names)).any(|(r, old)| r.name != *old) {
        // Names stay unique: a repeated one gets a number.
        for i in 0..fe.contact_regions.len() {
            let name = fe.contact_regions[i].name.clone();
            let taken =
                (fe.contact_regions[..i].iter()).any(|r| r.name.eq_ignore_ascii_case(&name));
            if taken {
                let others = fe.contact_regions.iter().map(|r| r.name.as_str());
                fe.contact_regions[i].name = plx_model::next_name(&name, others);
            }
        }
        // History outputs and step changes follow the new names, by way of names no region
        // has, so that a name passed on from one region to another is not followed twice.
        let renamed: Vec<(usize, &String)> = (names.iter().enumerate())
            .filter(|(i, old)| fe.contact_regions[*i].name != **old)
            .collect();
        for &(i, old) in &renamed {
            fe.rename_contact_pair(old, &format!("\u{0}{i}"));
        }
        for &(i, _) in &renamed {
            let new = fe.contact_regions[i].name.clone();
            fe.rename_contact_pair(&format!("\u{0}{i}"), &new);
        }
    }
}

/// The derived values of a region with units and the reasons, for the worksheet's tooltip.
fn exported_hover(resolved: &Resolved, units: UnitSystem) -> String {
    let mut lines = Vec::new();
    if let Some(k) = resolved.normal_stiffness() {
        lines.push(format!(
            "Normal stiffness {}",
            with_unit(k, units, Quantity::ForcePerVolume)
        ));
    }
    if let Some(k) = resolved.tangential_stiffness() {
        lines.push(format!(
            "Tangential stiffness {}",
            with_unit(k, units, Quantity::ForcePerVolume)
        ));
    }
    if let Some(t) = resolved.tolerance() {
        lines.push(format!(
            "Tolerance {}",
            with_unit(t, units, Quantity::Length)
        ));
    }
    lines.extend(resolved.why.iter().cloned());
    lines.join("\n")
}

/// Whether contacts take part in a step of this kind, so that it can switch them.
fn step_takes_contact(kind: &plx_model::StepKind) -> bool {
    use plx_model::StepKind;
    matches!(
        kind,
        StepKind::Static(_)
            | StepKind::Visco(_)
            | StepKind::Dynamic(_)
            | StepKind::CoupledTempDisp(_)
            | StepKind::UncoupledTempDisp(_)
    )
}

/// The contact regions in a step, as rows of the step dialog: switched on or off and the
/// friction coefficient of frictional ones, from this step on. `index` is the step's place,
/// `None` for a new step after the others.
pub fn step_contacts(ui: &mut Ui, fe: &FeModel, step: &mut plx_model::Step, index: Option<usize>) {
    let regions: Vec<&ContactRegion> = fe.contact_regions.iter().filter(|r| r.active).collect();
    if regions.is_empty() || !step_takes_contact(&step.kind) {
        return;
    }
    let earlier = &fe.steps[..index.unwrap_or(fe.steps.len()).min(fe.steps.len())];
    ui.label("Contacts in this step").on_hover_text(
        "Contact regions switched off or on, or with another friction coefficient, from \
         this step on. A change holds in the following steps until another one.",
    );
    ui.vertical(|ui| {
        egui::Grid::new("step contacts")
            .num_columns(4)
            .striped(true)
            .spacing([12.0, 4.0])
            .show(ui, |ui| {
                ui.strong("Region");
                ui.strong("Active");
                ui.strong("Friction");
                ui.label("");
                ui.end_row();
                for region in regions {
                    let before = region.state_after(earlier);
                    let mut wanted = region.state_in(step, before);
                    ui.label(&region.name);
                    ui.checkbox(&mut wanted.active, "");
                    match &mut wanted.friction {
                        Some(mu) => {
                            ui.add(numeric::drag_value(mu).speed(0.01).range(0.0..=5.0));
                        }
                        None => {
                            ui.weak("-");
                        }
                    }
                    if wanted != region.state_in(step, before) {
                        step.set_contact_state(&region.name, before, wanted);
                    }
                    if wanted != before {
                        ui.weak("changed here");
                    } else {
                        ui.label("");
                    }
                    ui.end_row();
                }
            });
    });
    ui.end_row();
}

#[cfg(test)]
mod tests {
    use plx_io::frd::read_frd;
    use plx_mesh::CadEntity;
    use plx_model::{
        BoundaryCondition, BoundaryKind, Elastic, Load, LoadKind, Material, Region, Section,
        SectionKind, Step, UnitSystem,
    };

    use super::*;

    fn gmsh_available() -> bool {
        match plx_mesher::self_test() {
            Ok(_) => true,
            Err(error) if std::env::var_os("NODALIS_REQUIRE_GMSH").is_none() => {
                eprintln!("Gmsh not available, test skipped: {error}");
                false
            }
            Err(error) => panic!("Gmsh is required: {error}"),
        }
    }

    /// The CAD faces of a part of the geometry display whose corners all satisfy `on`.
    fn faces_where(view: &Model, part: usize, on: impl Fn([f64; 3]) -> bool) -> Vec<CadEntity> {
        let mut faces: Vec<CadEntity> = Vec::new();
        let mut off = std::collections::BTreeSet::new();
        for &id in &view.mesh.parts[part].elements {
            let Some(CadEntity::Face(tag)) = view.cad_entity(id) else {
                continue;
            };
            let element = view.mesh.element(id).unwrap();
            if element
                .nodes
                .iter()
                .all(|&n| on(view.mesh.node(n).unwrap()))
            {
                faces.push(CadEntity::Face(tag));
            } else {
                off.insert(tag);
            }
        }
        faces.retain(|f| !matches!(f, CadEntity::Face(t) if off.contains(t)));
        faces.sort_unstable();
        faces.dedup();
        faces
    }

    /// The largest displacement against `component` CalculiX computes, if ccx is installed.
    fn settlement(model: &Model, component: &str) -> Option<f32> {
        let inp = plx_io::inp::write_inp(&model.mesh, &model.fe, "Block").unwrap();
        let dir = std::env::temp_dir().join(format!(
            "nodalis-geometry-detection-{}-{}",
            std::process::id(),
            model.mesh.element_count()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("block.inp"), &inp).unwrap();
        plxs_session::run_input_file(&dir, "block").unwrap_or_else(|e| panic!("{e}"));
        let results = read_frd(&dir.join("block.frd")).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        let disp = results.increments.last().unwrap().field("DISP").unwrap();
        let values = &disp.component(component).unwrap().values;
        Some(-values.iter().copied().fold(f32::MAX, f32::min))
    }

    /// A block 10 x 10 x 10 standing on a plate 40 x 40 x 5, two solids that only touch.
    /// The detection on the geometry finds them bonded before anything is meshed: one
    /// region of the block's bottom and the plate's top, whole CAD faces, although the block
    /// covers only part of the plate. Meshed, and meshed again finer, the region finds its
    /// faces on each mesh, and CalculiX holds the pressed block on the fixed plate.
    #[test]
    fn contacts_detected_on_the_geometry_hold_on_every_mesh() {
        if !gmsh_available() {
            return;
        }
        let dir = std::env::temp_dir().join(format!("nodalis-block-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("block.brep");
        plx_mesher::gmsh::with_gmsh(|gmsh| {
            gmsh.add_box([0.0; 3], [40.0, 40.0, 5.0])?;
            gmsh.add_box([10.0, 10.0, 5.0], [10.0; 3])?;
            gmsh.write(&file)
        })
        .unwrap();
        let loaded = crate::model::load_geometry(&[file], UnitSystem::Unitless).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        let (mut model, view) = (loaded.model, loaded.geometry_view.unwrap());
        let names: Vec<String> = view.mesh.parts.iter().map(|p| p.name.clone()).collect();
        assert_eq!(names.len(), 2, "{names:?}");
        let plate_top = faces_where(&view, 0, |p| (p[2] - 5.0).abs() < 1e-6);
        let block_bottom = faces_where(&view, 1, |p| (p[2] - 5.0).abs() < 1e-6);
        assert_eq!((plate_top.len(), block_bottom.len()), (1, 1));

        model.fe.materials.push(Material {
            name: "Steel".into(),
            density: None,
            elastic: Some(Elastic {
                young: 210000.0,
                poisson: 0.3,
            }),
            conductivity: None,
            specific_heat: None,
            expansion: None,
            plastic: None,
            creep: None,
        });
        model.fe.sections.push(Section {
            name: "Section-1".into(),
            material: "Steel".into(),
            region: Region::Parts(names.clone()),
            thickness: 1.0,
            kind: SectionKind::Solid,
        });
        let mut step = Step::new_static("Step-1");
        step.boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Fix".into(),
            active: true,
            visible: true,
            region: Region::Geometry(faces_where(&view, 0, |p| p[2].abs() < 1e-6)),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        step.loads.push(Load {
            distribution: None,
            name: "Press".into(),
            active: true,
            visible: true,
            region: Region::Geometry(faces_where(&view, 1, |p| (p[2] - 15.0).abs() < 1e-6)),
            kind: LoadKind::Pressure(100.0),
            amplitude: None,
            factor_amplitude: None,
        });
        model.fe.steps.push(step);
        model
            .fe
            .contact_detections
            .push(ContactDetection::new("Detection-1"));

        // Nothing is meshed yet: the detection runs on the geometry.
        assert!(detection_display(&model, Some(&view)).is_some());
        let report = detect(&mut model, Some(&view), 0).unwrap();
        assert_eq!(report.added, 1, "{report:?}");
        let region = &model.fe.contact_regions[0];
        let mut sides = [region.side_a.clone(), region.side_b.clone()];
        sides.sort_by_key(|r| format!("{r:?}"));
        let mut expected = [
            Region::Geometry(plate_top.clone()),
            Region::Geometry(block_bottom.clone()),
        ];
        expected.sort_by_key(|r| format!("{r:?}"));
        assert_eq!(sides, expected);
        assert_eq!(region.settings.tolerance, None, "left to the mesh");
        // 0.5 % of the diagonal of the parts, not of their triangles.
        let diagonal = (40.0f64.powi(2) * 2.0 + 15.0f64.powi(2)).sqrt();
        assert!(
            (report.tolerance - 0.005 * diagonal).abs() < 1e-9,
            "{report:?}"
        );
        let found = model.fe.contact_regions.clone();

        let mut geometry = model.geometry.clone().unwrap();
        geometry.meshing.max_size = 5.0;
        model.set_mesh(plx_mesher::generate_mesh(&geometry).unwrap().mesh);
        // Meshed from the geometry, detecting again finds the same region.
        assert!(detection_display(&model, Some(&view)).is_some());
        detect(&mut model, Some(&view), 0).unwrap();
        assert_eq!(model.fe.contact_regions, found);
        let region = &model.fe.contact_regions[0];
        let coarse =
            region.side_a.faces(&model.mesh).len() + region.side_b.faces(&model.mesh).len();
        assert!(coarse > 0);
        let coarse_settlement = settlement(&model, "U3");

        geometry.meshing.max_size = 2.5;
        model.set_mesh(plx_mesher::generate_mesh(&geometry).unwrap().mesh);
        let region = &model.fe.contact_regions[0];
        let fine = region.side_a.faces(&model.mesh).len() + region.side_b.faces(&model.mesh).len();
        assert!(fine > 2 * coarse, "{coarse} -> {fine}");
        let fine_settlement = settlement(&model, "U3");

        if let (Some(coarse), Some(fine)) = (coarse_settlement, fine_settlement) {
            // 100 MPa on the block compress it by 0.0048 mm; the plate gives a little more.
            let block = 100.0 / 210000.0 * 10.0;
            for value in [coarse, fine] {
                assert!(value > block && value < 3.0 * block, "{value} vs. {block}");
            }
            assert!((fine - coarse).abs() < 0.05 * fine, "{coarse} -> {fine}");
        }
    }

    /// The CAD edges of a part of the geometry display whose points all satisfy `on`.
    fn edges_where(view: &Model, part: usize, on: impl Fn([f64; 3]) -> bool) -> Vec<CadEntity> {
        let mut edges: Vec<CadEntity> = Vec::new();
        let mut off = std::collections::BTreeSet::new();
        for &id in &view.mesh.parts[part].elements {
            let Some(CadEntity::Edge(tag)) = view.cad_entity(id) else {
                continue;
            };
            let element = view.mesh.element(id).unwrap();
            if element
                .nodes
                .iter()
                .all(|&n| on(view.mesh.node(n).unwrap()))
            {
                edges.push(CadEntity::Edge(tag));
            } else {
                off.insert(tag);
            }
        }
        edges.retain(|e| !matches!(e, CadEntity::Edge(t) if off.contains(t)));
        edges.sort_unstable();
        edges.dedup();
        edges
    }

    /// A beam 40 x 5 lying on a block 10 x 10 in a plane stress model, two faces that only
    /// touch along y = 0: the detection on the geometry ties the block's top edge to the
    /// beam's bottom edge, whole CAD edges, before anything is meshed, and CalculiX holds
    /// the pressed beam on the fixed block.
    #[test]
    fn edges_detected_on_the_geometry_of_a_2d_model_hold() {
        if !gmsh_available() {
            return;
        }
        let dir = std::env::temp_dir().join(format!("nodalis-beam-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("beam.brep");
        plx_mesher::gmsh::with_gmsh(|gmsh| {
            gmsh.add_rectangle([0.0; 3], [40.0, 5.0], 0.0)?;
            gmsh.add_rectangle([15.0, -10.0, 0.0], [10.0, 10.0], 0.0)?;
            gmsh.write(&file)
        })
        .unwrap();
        let loaded = crate::model::load_geometry(&[file], UnitSystem::Unitless).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        let (mut model, view) = (loaded.model, loaded.geometry_view.unwrap());
        model.fe.properties.space = plx_model::ModelSpace::PlaneStress;
        let names: Vec<String> = view.mesh.parts.iter().map(|p| p.name.clone()).collect();
        assert_eq!(names.len(), 2, "{names:?}");
        let beam_bottom = edges_where(&view, 0, |p| p[1].abs() < 1e-6);
        let block_top = edges_where(&view, 1, |p| p[1].abs() < 1e-6);
        assert_eq!((beam_bottom.len(), block_top.len()), (1, 1));

        model.fe.materials.push(Material {
            name: "Steel".into(),
            density: None,
            elastic: Some(Elastic {
                young: 210000.0,
                poisson: 0.3,
            }),
            conductivity: None,
            specific_heat: None,
            expansion: None,
            plastic: None,
            creep: None,
        });
        model.fe.sections.push(Section {
            name: "Section-1".into(),
            material: "Steel".into(),
            region: Region::Parts(names.clone()),
            thickness: 1.0,
            kind: SectionKind::Solid,
        });
        let mut step = Step::new_static("Step-1");
        step.boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Fix".into(),
            active: true,
            visible: true,
            region: Region::Geometry(edges_where(&view, 1, |p| (p[1] + 10.0).abs() < 1e-6)),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        step.loads.push(Load {
            distribution: None,
            name: "Press".into(),
            active: true,
            visible: true,
            region: Region::Geometry(edges_where(&view, 0, |p| (p[1] - 5.0).abs() < 1e-6)),
            kind: LoadKind::Pressure(10.0),
            amplitude: None,
            factor_amplitude: None,
        });
        model.fe.steps.push(step);
        model
            .fe
            .contact_detections
            .push(ContactDetection::new("Detection-1"));

        let report = detect(&mut model, Some(&view), 0).unwrap();
        assert_eq!(report.added, 1, "{report:?}");
        let region = &model.fe.contact_regions[0];
        let mut sides = [region.side_a.clone(), region.side_b.clone()];
        sides.sort_by_key(|r| format!("{r:?}"));
        let mut expected = [
            Region::Geometry(beam_bottom.clone()),
            Region::Geometry(block_top.clone()),
        ];
        expected.sort_by_key(|r| format!("{r:?}"));
        assert_eq!(sides, expected);

        let mut geometry = model.geometry.clone().unwrap();
        geometry.meshing.max_size = 2.5;
        let mut mesh = plx_mesher::generate_mesh(&geometry).unwrap().mesh;
        (model.fe.properties.space)
            .prepare_generated_mesh(&mut mesh)
            .unwrap();
        model.set_mesh(mesh);
        let region = &model.fe.contact_regions[0];
        for side in [&region.side_a, &region.side_b] {
            let faces = side.faces(&model.mesh);
            assert!(!faces.is_empty(), "{side:?}");
            let nodes = Region::Faces(faces).nodes(&model.mesh);
            assert!(
                nodes
                    .iter()
                    .all(|&n| model.mesh.node(n).unwrap()[1].abs() < 1e-6)
            );
        }
        if let Some(value) = settlement(&model, "U2") {
            // The ends of the beam, 15 mm beyond the block, bend down by q L^4 / 8 E I =
            // 0.029 mm as cantilevers; shear and the block add some. Untied, the beam would
            // float and CalculiX fail.
            assert!(value > 0.029 && value < 0.1, "{value}");
        }
    }
}
