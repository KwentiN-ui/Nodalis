//! The dialog of a cross-section and the sketch of a profile shown in it and in the beam
//! section dialog: the outline with the 1- and 2-direction, the centroid and the principal
//! axes through it.

use egui::{Color32, FontId, Pos2, Rect, Shape, Stroke, Ui, pos2, vec2};
use plx_model::{BeamProfile, CrossSection, Quantity, Sketch, UnitSystem};

use crate::{expose, numeric, setup, style};

/// Size of the sketch of a profile.
const SKETCH: egui::Vec2 = egui::vec2(240.0, 180.0);

/// The rows of the cross-section dialog: name, profile type, dimensions and the sketch with
/// the area and second moments.
pub fn form(ui: &mut Ui, cross_section: &mut CrossSection, units: UnitSystem) {
    setup::name_row(ui, &mut cross_section.name);
    let profile = &mut cross_section.profile;
    ui.label("Profile");
    egui::ComboBox::from_id_salt("cross-section profile")
        .selected_text(profile.label())
        .width(crate::dock::fit_width(ui, 200.0))
        // All kinds without scrolling.
        .height(400.0)
        .show_ui(ui, |ui| {
            for other in &BeamProfile::ALL {
                let selected = std::mem::discriminant(other) == std::mem::discriminant(profile);
                if ui.selectable_label(selected, other.label()).clicked() && !selected {
                    *profile = scaled_like(other, profile);
                }
            }
        });
    ui.end_row();
    dimensions(ui, profile, units);
    if profile.needs_reduced_integration() {
        ui.label("");
        ui.weak("Pipe and box need beams on lines with 3 nodes (B32R).");
        ui.end_row();
    }
    if !profile.is_native() {
        ui.label("");
        ui.scope(|ui| {
            ui.set_max_width(SKETCH.x);
            let note = "A general beam of CalculiX (U1): lines with 2 nodes, the axis through \
                        the centroid, linear Static and Frequency steps only.";
            ui.add(egui::Label::new(egui::RichText::new(note).weak()).wrap())
        })
        .inner
        .on_hover_text(
            "CalculiX has beam sections of its own for rectangles, circles, pipes and boxes \
             only. Other profiles are written by their area and second moments of area about \
             the principal axes, for the user element U1. It takes neither large \
             deformations (Nlgeom) nor buckling, its torsion stiffness is k G (I11 + I22), \
             and the results show no stresses on it.",
        );
        ui.end_row();
    }
    match profile {
        BeamProfile::Sketch(sketch) => crate::sketch::rows(ui, sketch, units),
        _ => {
            ui.label("");
            preview(ui, profile, units);
            ui.end_row();
        }
    }
}

/// A profile of the kind of `kind` about as large as `like`, so that switching the kind
/// keeps the size of the profile rather than falling back to a millimetre.
fn scaled_like(kind: &BeamProfile, like: &BeamProfile) -> BeamProfile {
    let [a, b] = like.size();
    let size = a.max(b);
    let wall = a.min(b) / 10.0;
    match kind {
        BeamProfile::Rect { .. } => BeamProfile::Rect { a, b },
        BeamProfile::Circ { .. } => BeamProfile::Circ { radius: size / 2.0 },
        BeamProfile::Pipe { .. } => BeamProfile::Pipe {
            radius: size / 2.0,
            thickness: wall,
        },
        BeamProfile::Box { .. } => BeamProfile::Box { a, b, t: [wall; 4] },
        BeamProfile::I { .. } => BeamProfile::I {
            h: b,
            b: a,
            tw: wall * 0.6,
            tf: wall,
        },
        BeamProfile::T { .. } => BeamProfile::T {
            h: b,
            b: a,
            tw: wall * 0.6,
            tf: wall,
        },
        BeamProfile::U { .. } => BeamProfile::U {
            h: b,
            b: a,
            tw: wall * 0.6,
            tf: wall,
        },
        BeamProfile::L { .. } => BeamProfile::L {
            h: b,
            b: a,
            t: wall,
        },
        // The values of the profile it was, so that a known section can be adjusted.
        BeamProfile::General { .. } => {
            let p = like.properties();
            BeamProfile::General {
                area: p.area,
                i11: p.i11,
                i22: p.i22,
                k: like.shear_factor(),
            }
        }
        // The outline of the profile it was, to draw on from there.
        BeamProfile::Sketch(_) => BeamProfile::Sketch(Sketch::of(like)),
    }
}

/// The dimension fields of a profile, each one an exposable parameter.
fn dimensions(ui: &mut Ui, profile: &mut BeamProfile, units: UnitSystem) {
    for dimension in profile.dimensions_mut() {
        ui.label(dimension.label);
        let value = dimension.value;
        expose::field(ui, dimension.field, |ui| {
            let field = match dimension.quantity {
                Some(Quantity::Length) => numeric::quantity(value, units, Quantity::Length),
                // Areas and second moments span many orders of magnitude.
                Some(quantity) => numeric::physical(value, units, quantity),
                None => numeric::drag_value(value).speed(0.01),
            };
            ui.add(field.range(0.0..=f64::MAX))
        });
        ui.end_row();
    }
}

/// The sketch of a profile with its area and second moments of area below, in the model's
/// units. The 1-direction points right and the 2-direction up, from the beam's nodes at the
/// centre of the profile.
pub fn preview(ui: &mut Ui, profile: &BeamProfile, units: UnitSystem) {
    ui.vertical(|ui| {
        let (rect, _) = ui.allocate_exact_size(SKETCH, egui::Sense::hover());
        if ui.is_rect_visible(rect) {
            paint(ui, rect, profile);
        }
        if profile.is_valid() {
            properties(ui, profile, units);
        } else {
            ui.colored_label(style::error(), "Invalid dimensions");
        }
    });
}

fn paint(ui: &Ui, rect: Rect, profile: &BeamProfile) {
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 2.0, style::window());
    painter.rect_stroke(
        rect,
        2.0,
        Stroke::new(1.0, style::border()),
        egui::StrokeKind::Inside,
    );
    if !profile.is_valid() {
        return;
    }
    let outlines = profile.outlines();
    let (mut min, mut max) = ([f64::MAX; 2], [f64::MIN; 2]);
    for &[u, v] in outlines.iter().flat_map(|o| &o.points) {
        min = [min[0].min(u), min[1].min(v)];
        max = [max[0].max(u), max[1].max(v)];
    }
    // Room for the axis labels around the profile.
    let inner = rect.shrink(22.0);
    let span = (max[0] - min[0]).max(max[1] - min[1]);
    if span.is_nan() || span <= 0.0 {
        return;
    }
    let scale = f64::from(inner.width().min(inner.height())) / span;
    let mid = [(min[0] + max[0]) / 2.0, (min[1] + max[1]) / 2.0];
    let centre = inner.center();
    let at = |[u, v]: [f64; 2]| -> Pos2 {
        pos2(
            centre.x + ((u - mid[0]) * scale) as f32,
            centre.y - ((v - mid[1]) * scale) as f32,
        )
    };
    let fill = style::adapt(Color32::from_rgb(170, 190, 215));
    let edge = Stroke::new(1.5, style::adapt(Color32::from_rgb(40, 70, 120)));
    let rings: Vec<Vec<[f64; 2]>> = outlines.iter().map(|o| o.points.clone()).collect();
    fill_rings(&painter, &rings, at, fill, edge);
    let properties = profile.properties();
    let length = 0.3 * inner.width().min(inner.height());
    let (_, angle) = properties.principal();
    paint_axes(
        &painter,
        rect,
        at([0.0, 0.0]),
        at(properties.centroid),
        angle,
        length,
    );
}

/// The beam's 1- and 2-direction from the nodes at `origin`, and the principal axes through
/// the centroid, dashed across the whole of `rect`, the first one at `angle` in radians.
pub(crate) fn paint_axes(
    painter: &egui::Painter,
    rect: Rect,
    origin: Pos2,
    centroid: Pos2,
    angle: f64,
    length: f32,
) {
    let axis = Stroke::new(1.0, style::text());
    let font = FontId::proportional(11.0);
    // The labels sit beside the arrow heads, clear of the principal axes along them.
    for (direction, label, beside, align) in [
        (
            vec2(1.0, 0.0),
            "1",
            vec2(-2.0, 4.0),
            egui::Align2::RIGHT_TOP,
        ),
        (vec2(0.0, -1.0), "2", vec2(4.0, 2.0), egui::Align2::LEFT_TOP),
    ] {
        let tip = origin + direction * length;
        painter.arrow(origin, tip - origin, axis);
        painter.text(tip + beside, align, label, font.clone(), style::text());
    }
    // The principal axes through the centroid, dashed across the whole sketch.
    let principal = Stroke::new(1.0, style::adapt(Color32::from_rgb(200, 60, 40)));
    let reach = rect.width().max(rect.height());
    for (k, turn) in [0.0, std::f64::consts::FRAC_PI_2].into_iter().enumerate() {
        let (sin, cos) = (angle + turn).sin_cos();
        // Up on the screen is the positive 2-direction.
        let direction = vec2(cos as f32, -sin as f32);
        let ends = [centroid - direction * reach, centroid + direction * reach];
        painter.extend(Shape::dashed_line(&ends, principal, 5.0, 3.0));
        let label = if k == 0 { "I" } else { "II" };
        let at_edge = clip_to(rect.shrink(9.0), centroid, direction);
        painter.text(
            at_edge,
            egui::Align2::CENTER_CENTER,
            label,
            font.clone(),
            principal.color,
        );
    }
    painter.circle(centroid, 3.5, style::window(), principal);
    painter.circle_filled(centroid, 1.5, principal.color);
}

/// The region inside the first ring and outside the others filled by its triangles, as I, T,
/// U and L profiles and sketches are concave, with the rings drawn around it.
pub(crate) fn fill_rings(
    painter: &egui::Painter,
    rings: &[Vec<[f64; 2]>],
    at: impl Fn([f64; 2]) -> Pos2,
    fill: Color32,
    edge: Stroke,
) {
    let mut mesh = egui::Mesh::default();
    for &point in rings.iter().flatten() {
        mesh.colored_vertex(at(point), fill);
    }
    for [a, b, c] in plx_model::triangulate_with_holes(rings) {
        mesh.add_triangle(a as u32, b as u32, c as u32);
    }
    painter.add(Shape::mesh(mesh));
    for ring in rings {
        painter.add(Shape::closed_line(
            ring.iter().map(|&p| at(p)).collect(),
            edge,
        ));
    }
}

/// Where the ray from `start` along `direction` leaves `rect`.
fn clip_to(rect: Rect, start: Pos2, direction: egui::Vec2) -> Pos2 {
    let mut t = f32::MAX;
    for (d, low, high, s) in [
        (direction.x, rect.left(), rect.right(), start.x),
        (direction.y, rect.top(), rect.bottom(), start.y),
    ] {
        if d > 1e-6 {
            t = t.min((high - s) / d);
        } else if d < -1e-6 {
            t = t.min((low - s) / d);
        }
    }
    start + direction * t.max(0.0)
}

/// Area, centroid and second moments of area of a profile as text, in the model's units.
fn properties(ui: &mut Ui, profile: &BeamProfile, units: UnitSystem) {
    property_lines(ui, profile.properties(), profile.size(), units);
}

/// The lines of [`properties`] for the values `p` of a profile of `size`.
pub(crate) fn property_lines(
    ui: &mut Ui,
    mut p: plx_model::SectionProperties,
    size: [f64; 2],
    units: UnitSystem,
) {
    // A centroid at the centre up to rounding is at the centre.
    let size = size.into_iter().fold(0.0, f64::max);
    for c in &mut p.centroid {
        if c.abs() < 1e-9 * size {
            *c = 0.0;
        }
    }
    let length = plx_model::STORED.factor_to(Quantity::Length, units);
    let unit = units.unit(Quantity::Length);
    let value = |v: f64, power: i32| {
        let shown = numeric::fit(v * length.powi(power), |t| t.len() <= 9);
        match power {
            1 => format!("{shown} {unit}"),
            power => format!("{shown} {unit}^{power}"),
        }
    };
    let ([big, small], angle) = p.principal();
    let lines = [
        format!("A = {}", value(p.area, 2)),
        format!(
            "Centroid at 1 = {}, 2 = {}",
            value(p.centroid[0], 1),
            value(p.centroid[1], 1)
        ),
        format!("I11 = {}, I22 = {}", value(p.i11, 4), value(p.i22, 4)),
        format!("Principal: I = {}, II = {}", value(big, 4), value(small, 4)),
        format!(
            "Axis I at {:.1} deg from the 1-direction",
            // Without a sign on 0.
            if angle.to_degrees().abs() < 0.05 {
                0.0
            } else {
                angle.to_degrees()
            }
        ),
    ];
    for line in lines {
        ui.label(egui::RichText::new(line).small());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn switching_the_kind_keeps_the_size() {
        let rect = BeamProfile::Rect { a: 0.02, b: 0.04 };
        let pipe = scaled_like(&BeamProfile::ALL[2], &rect);
        assert_eq!(pipe.size(), [0.04, 0.04]);
        assert!(pipe.is_valid());
        let boxed = scaled_like(&BeamProfile::ALL[3], &rect);
        assert_eq!(boxed.size(), [0.02, 0.04]);
        assert!(boxed.is_valid());
        // Every kind made from it is valid, the general one with its values.
        for kind in &BeamProfile::ALL {
            assert!(scaled_like(kind, &rect).is_valid(), "{kind:?}");
        }
        let general = scaled_like(&BeamProfile::ALL[8], &rect);
        assert_eq!(general.properties().i11, rect.properties().i11);
    }

    #[test]
    fn rays_leave_the_rect_at_its_border() {
        let rect = Rect::from_min_max(pos2(0.0, 0.0), pos2(10.0, 20.0));
        assert_eq!(
            clip_to(rect, pos2(5.0, 10.0), vec2(1.0, 0.0)),
            pos2(10.0, 10.0)
        );
        assert_eq!(
            clip_to(rect, pos2(5.0, 10.0), vec2(0.0, -1.0)),
            pos2(5.0, 0.0)
        );
    }
}
