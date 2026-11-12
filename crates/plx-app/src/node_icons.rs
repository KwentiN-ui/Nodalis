//! Flat, single-colour symbols of the rows of a block in the project schematic, drawn with
//! strokes in the style of a node editor like Blender's: simple outlines in the text colour
//! that read at a glance without competing with the coloured pins and wires.

use crate::tree_icons::{self, StepGlyph, TreeIcon};
use egui::{Color32, Painter, Pos2, Shape, Stroke, pos2, vec2};

/// What a row of a block stands for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeIcon {
    /// Where the analysis starts: an arrow into a circle.
    State,
    /// The CAD geometry: a cube.
    Geometry,
    /// The mesh: a grid.
    Mesh,
    /// The model setup: sliders.
    Setup,
    /// Features such as reference points: a crosshair.
    Features,
    /// Materials: a shaded sphere.
    Materials,
    /// Sections: an I profile.
    Sections,
    /// Cross-sections: a box profile with the axes through its centroid.
    CrossSections,
    /// Constraints: a support.
    Constraints,
    /// Contacts: two bodies touching.
    Contacts,
    /// Amplitudes: a ramp over time.
    Amplitudes,
    /// Initial conditions: a clock at the start.
    InitialConditions,
    /// The list of steps.
    Steps,
    /// One step: a play sign.
    Step,
    /// The CalculiX run: a gear.
    Solution,
    /// The results: bars.
    Results,
    /// Taken over from another analysis: two chain links.
    Link,
    /// A file written by an export block: an arrow out of a tray.
    Export,
    /// A note: a sheet with lines of text and a folded corner.
    Note,
    /// A CAD operation: a cube with a rounded edge.
    CadOperation,
    /// A Transform block: a square and an arrow to where it goes.
    Transform,
    /// A Math block: a plus over a minus.
    Math,
    /// A Switch block: two inputs and a lever to one of them.
    Switch,
    /// A Plot block: two axes with a curve.
    Plot,
    /// A Preview block: a framed picture of a triangle mesh.
    Picture,
    /// A node group: two small blocks linked inside a frame.
    Group,
    /// The Group Input block: an arrow into a frame; the Group Output block: one out of it.
    GroupInput,
    GroupOutput,
    /// A step of this type, drawn like in the tree of the FE model.
    StepType(StepGlyph),
}

/// Paints `icon` into the 14 x 14 square at `min` in `color`.
pub fn paint(painter: &Painter, min: Pos2, icon: NodeIcon, color: Color32) {
    paint_scaled(painter, min, icon, color, 1.0);
}

/// Paints `icon` into the square of 14 times `scale` at `min` in `color`, for the larger
/// tiles of the ribbon.
pub fn paint_scaled(painter: &Painter, min: Pos2, icon: NodeIcon, color: Color32, scale: f32) {
    if let NodeIcon::StepType(glyph) = icon {
        tree_icons::paint(painter, min, TreeIcon::StepType(glyph));
        return;
    }
    let s = scale;
    let stroke = Stroke::new(1.2 * s.sqrt(), color);
    let p = |x: f32, y: f32| pos2(min.x + x * s, min.y + y * s);
    let line = |points: &[Pos2]| {
        painter.add(Shape::line(points.to_vec(), stroke));
    };
    let closed = |points: &[Pos2]| {
        painter.add(Shape::closed_line(points.to_vec(), stroke));
    };
    let circle = |center: Pos2, radius: f32| {
        painter.circle_stroke(center, radius * s, stroke);
    };
    let dot = |center: Pos2, radius: f32| {
        painter.circle_filled(center, radius * s, color);
    };
    match icon {
        NodeIcon::StepType(_) => {}
        NodeIcon::Link => {
            let link = |x0: f32, y0: f32| {
                let points: Vec<Pos2> = [
                    (0.0, 0.0),
                    (4.5, 0.0),
                    (6.0, 1.5),
                    (6.0, 3.0),
                    (4.5, 4.5),
                    (0.0, 4.5),
                    (-1.5, 3.0),
                    (-1.5, 1.5),
                ]
                .iter()
                .map(|(x, y)| p(x0 + x, y0 + y))
                .collect();
                closed(&points);
            };
            link(1.5, 8.0);
            link(6.0, 1.5);
        }
        NodeIcon::Math => {
            line(&[p(1.5, 4.5), p(7.5, 4.5)]);
            line(&[p(4.5, 1.5), p(4.5, 7.5)]);
            line(&[p(6.5, 10.5), p(12.5, 10.5)]);
            line(&[p(11.0, 2.0), p(3.0, 12.0)]);
        }
        NodeIcon::Plot => {
            line(&[p(1.5, 1.5), p(1.5, 12.5), p(12.5, 12.5)]);
            line(&[
                p(3.0, 10.5),
                p(5.0, 6.0),
                p(7.0, 8.0),
                p(9.5, 3.5),
                p(12.0, 4.5),
            ]);
        }
        NodeIcon::Picture => {
            closed(&[p(1.5, 2.5), p(12.5, 2.5), p(12.5, 11.5), p(1.5, 11.5)]);
            line(&[p(3.5, 9.5), p(7.0, 4.5), p(10.5, 9.5), p(3.5, 9.5)]);
            line(&[p(5.25, 7.0), p(8.75, 7.0), p(7.0, 9.5), p(5.25, 7.0)]);
        }
        NodeIcon::Group => {
            closed(&[p(1.0, 1.5), p(13.0, 1.5), p(13.0, 12.5), p(1.0, 12.5)]);
            closed(&[p(3.0, 4.0), p(6.0, 4.0), p(6.0, 7.5), p(3.0, 7.5)]);
            closed(&[p(8.0, 6.5), p(11.0, 6.5), p(11.0, 10.0), p(8.0, 10.0)]);
            line(&[p(6.0, 5.75), p(7.0, 5.75), p(7.0, 8.25), p(8.0, 8.25)]);
        }
        NodeIcon::GroupInput => {
            // An arrow into a frame open to the left.
            line(&[p(6.0, 1.5), p(12.5, 1.5), p(12.5, 12.5), p(6.0, 12.5)]);
            line(&[p(1.0, 7.0), p(9.5, 7.0)]);
            line(&[p(7.0, 4.5), p(9.5, 7.0), p(7.0, 9.5)]);
        }
        NodeIcon::GroupOutput => {
            // An arrow out of a frame open to the right.
            line(&[p(8.0, 1.5), p(1.5, 1.5), p(1.5, 12.5), p(8.0, 12.5)]);
            line(&[p(4.5, 7.0), p(13.0, 7.0)]);
            line(&[p(10.5, 4.5), p(13.0, 7.0), p(10.5, 9.5)]);
        }
        NodeIcon::Switch => {
            dot(p(2.5, 3.5), 1.6);
            dot(p(2.5, 10.5), 1.6);
            dot(p(11.5, 7.0), 1.6);
            line(&[p(2.5, 3.5), p(11.5, 7.0)]);
            line(&[p(4.0, 10.5), p(6.0, 10.5)]);
        }
        NodeIcon::Note => {
            closed(&[
                p(2.5, 1.5),
                p(9.0, 1.5),
                p(11.5, 4.0),
                p(11.5, 12.5),
                p(2.5, 12.5),
            ]);
            line(&[p(9.0, 1.5), p(9.0, 4.0), p(11.5, 4.0)]);
            for y in [6.5, 8.7, 10.9] {
                line(&[p(4.5, y), p(9.5, y)]);
            }
        }
        NodeIcon::Export => {
            line(&[p(1.5, 8.0), p(1.5, 12.5), p(12.5, 12.5), p(12.5, 8.0)]);
            line(&[p(7.0, 9.5), p(7.0, 1.5)]);
            line(&[p(4.2, 4.3), p(7.0, 1.5), p(9.8, 4.3)]);
        }
        NodeIcon::State => {
            line(&[p(0.5, 7.0), p(5.0, 7.0)]);
            line(&[p(3.0, 4.8), p(5.2, 7.0), p(3.0, 9.2)]);
            circle(p(9.5, 7.0), 3.3);
        }
        NodeIcon::CadOperation => {
            // The front face with its upper right corner rounded, the cube behind it.
            let mut front = vec![p(1.5, 12.0), p(1.5, 4.5), p(5.5, 4.5)];
            for k in 0..=6 {
                let angle = std::f32::consts::FRAC_PI_2 * k as f32 / 6.0;
                front.push(p(5.5 + 3.5 * angle.sin(), 8.0 - 3.5 * angle.cos()));
            }
            front.push(p(9.0, 12.0));
            closed(&front);
            line(&[
                p(1.5, 4.5),
                p(5.0, 1.5),
                p(12.5, 1.5),
                p(12.5, 9.0),
                p(9.0, 12.0),
            ]);
        }
        NodeIcon::Transform => {
            closed(&[p(1.5, 7.5), p(6.5, 7.5), p(6.5, 12.5), p(1.5, 12.5)]);
            line(&[p(5.0, 9.0), p(12.5, 1.5)]);
            line(&[p(8.0, 1.5), p(12.5, 1.5), p(12.5, 6.0)]);
        }
        NodeIcon::Geometry => {
            closed(&[p(1.5, 4.5), p(9.0, 4.5), p(9.0, 12.0), p(1.5, 12.0)]);
            line(&[p(1.5, 4.5), p(5.0, 1.5), p(12.5, 1.5), p(9.0, 4.5)]);
            line(&[p(12.5, 1.5), p(12.5, 9.0), p(9.0, 12.0)]);
        }
        NodeIcon::Mesh => {
            closed(&[p(1.5, 1.5), p(12.5, 1.5), p(12.5, 12.5), p(1.5, 12.5)]);
            for v in [5.2, 8.8] {
                line(&[p(v, 1.5), p(v, 12.5)]);
                line(&[p(1.5, v), p(12.5, v)]);
            }
        }
        NodeIcon::Setup => {
            for (y, knob) in [(3.5, 9.0), (7.0, 4.5), (10.5, 8.0)] {
                line(&[p(1.5, y), p(12.5, y)]);
                dot(p(knob, y), 1.8);
            }
        }
        NodeIcon::Features => {
            circle(p(7.0, 7.0), 4.0);
            line(&[p(7.0, 0.8), p(7.0, 3.5)]);
            line(&[p(7.0, 10.5), p(7.0, 13.2)]);
            line(&[p(0.8, 7.0), p(3.5, 7.0)]);
            line(&[p(10.5, 7.0), p(13.2, 7.0)]);
            dot(p(7.0, 7.0), 1.1);
        }
        NodeIcon::Materials => {
            circle(p(7.0, 7.0), 5.5);
            let shade: Vec<Pos2> = (0..=8)
                .map(|i| {
                    let angle = (-10.0 + 110.0 * i as f32 / 8.0_f32).to_radians();
                    p(7.0 + 3.3 * angle.cos(), 7.0 + 3.3 * angle.sin())
                })
                .collect();
            line(&shade);
        }
        NodeIcon::Sections => closed(&[
            p(2.0, 2.0),
            p(12.0, 2.0),
            p(12.0, 4.5),
            p(8.5, 4.5),
            p(8.5, 9.5),
            p(12.0, 9.5),
            p(12.0, 12.0),
            p(2.0, 12.0),
            p(2.0, 9.5),
            p(5.5, 9.5),
            p(5.5, 4.5),
            p(2.0, 4.5),
        ]),
        NodeIcon::CrossSections => {
            closed(&[p(2.5, 2.5), p(11.5, 2.5), p(11.5, 11.5), p(2.5, 11.5)]);
            closed(&[p(5.0, 5.0), p(9.0, 5.0), p(9.0, 9.0), p(5.0, 9.0)]);
            line(&[p(7.0, 0.5), p(7.0, 2.5)]);
            line(&[p(7.0, 11.5), p(7.0, 13.5)]);
            line(&[p(0.5, 7.0), p(2.5, 7.0)]);
            line(&[p(11.5, 7.0), p(13.5, 7.0)]);
        }
        NodeIcon::Constraints => {
            closed(&[p(7.0, 1.5), p(2.5, 9.0), p(11.5, 9.0)]);
            line(&[p(1.0, 11.0), p(13.0, 11.0)]);
            for x in [3.0, 6.0, 9.0] {
                line(&[p(x + 2.0, 11.0), p(x, 13.5)]);
            }
        }
        NodeIcon::Contacts => {
            closed(&[p(2.0, 1.5), p(12.0, 1.5), p(12.0, 6.0), p(2.0, 6.0)]);
            closed(&[p(2.0, 8.0), p(12.0, 8.0), p(12.0, 12.5), p(2.0, 12.5)]);
        }
        NodeIcon::Amplitudes => {
            line(&[p(2.0, 1.5), p(2.0, 12.5), p(13.0, 12.5)]);
            line(&[p(2.0, 10.5), p(5.5, 10.5), p(9.0, 3.5), p(13.0, 3.5)]);
        }
        NodeIcon::InitialConditions => {
            circle(p(7.0, 7.0), 5.5);
            line(&[p(7.0, 3.5), p(7.0, 7.0), p(10.0, 7.0)]);
        }
        NodeIcon::Steps => {
            for y in [3.5, 7.0, 10.5] {
                dot(p(3.0, y), 1.2);
                line(&[p(6.0, y), p(12.5, y)]);
            }
        }
        NodeIcon::Step => closed(&[p(4.0, 2.5), p(11.5, 7.0), p(4.0, 11.5)]),
        NodeIcon::Solution => {
            let gear: Vec<Pos2> = (0..16)
                .map(|i| {
                    let angle = std::f32::consts::TAU * i as f32 / 16.0;
                    let radius = if i % 2 == 0 { 6.0 } else { 4.3 };
                    p(7.0, 7.0) + radius * s * vec2(angle.cos(), angle.sin())
                })
                .collect();
            closed(&gear);
            circle(p(7.0, 7.0), 1.8);
        }
        NodeIcon::Results => {
            line(&[p(1.5, 12.5), p(12.5, 12.5)]);
            for (x, height) in [(3.0, 5.0), (6.5, 9.0), (10.0, 7.0)] {
                painter.rect_filled(
                    egui::Rect::from_min_size(p(x, 12.5 - height), vec2(2.5, height) * s),
                    0.5,
                    color,
                );
            }
        }
    }
}
