//! Model tree icons, 16 x 16 images drawn as pixel rectangles with the
//! egui painter. The pixel patterns are traced from PrePoMax's `Icons` folder (GPL-3.0, by
//! Matej Borovinsek), gradients reduced to a few shades.

use egui::epaint::Mesh;
use egui::{Color32, Painter, Pos2, Rect, Shape, Stroke, pos2, vec2};
use plx_model::StepKind;

/// Edge length of a tree icon in points.
pub const SIZE: f32 = 16.0;

/// The images shown in front of tree nodes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TreeIcon {
    Mesh,
    Part,
    NodeSet,
    ElementSet,
    Surface,
    Features,
    ReferencePoint,
    CoordinateSystem,
    Solid,
    Shell,
    Wire,
    Hidden,
    /// A deactivated item: the red no-entry sign.
    Inactive,
    Material,
    Section,
    Constraints,
    Contacts,
    SurfaceInteraction,
    ContactPair,
    Distribution,
    Amplitude,
    InitialConditions,
    Steps,
    Step,
    /// A step of a known type, drawn as a pictogram of that type.
    StepType(StepGlyph),
    FieldOutput,
    HistoryOutput,
    BoundaryCondition,
    Load,
    DefinedField,
    Running,
    Warning,
    Geometry,
    MeshSetup,
    HotSpot,
    Plane,
    Path,
    PlaneResult,
    Dots,
    DotsOpen,
}

/// A 16 x 16 image: one character per pixel, `.` is transparent, `A`.. `Z`, `a`.. `z` index
/// `colors` (RGBA, unmultiplied).
struct Pixmap {
    colors: &'static [[u8; 4]],
    rows: [&'static str; 16],
}

fn pixmap(icon: TreeIcon) -> &'static Pixmap {
    match icon {
        TreeIcon::Mesh => &MESH,
        TreeIcon::Part => &PART,
        TreeIcon::NodeSet => &NODE_SET,
        TreeIcon::ElementSet => &ELEMENT_SET,
        TreeIcon::Surface => &SURFACE,
        TreeIcon::Features => &FEATURES,
        TreeIcon::ReferencePoint => &REFERENCE_POINT,
        TreeIcon::CoordinateSystem => &COORDINATE_SYSTEM,
        TreeIcon::Solid => &SOLID,
        TreeIcon::Shell => &SHELL,
        TreeIcon::Wire => &WIRE,
        TreeIcon::Hidden => &HIDDEN,
        TreeIcon::Inactive => &INACTIVE,
        TreeIcon::Material => &MATERIAL,
        TreeIcon::Section => &SECTION,
        TreeIcon::Constraints => &CONSTRAINTS,
        TreeIcon::Contacts => &CONTACTS,
        TreeIcon::SurfaceInteraction => &SURFACE_INTERACTION,
        TreeIcon::ContactPair => &CONTACT_PAIR,
        TreeIcon::Distribution => &DISTRIBUTION,
        TreeIcon::Amplitude => &AMPLITUDE,
        TreeIcon::InitialConditions => &INITIAL_CONDITIONS,
        TreeIcon::Steps => &STEPS,
        TreeIcon::Step => &STEP,
        TreeIcon::StepType(_) => unreachable!("step types are drawn with strokes"),
        TreeIcon::FieldOutput => &FIELD_OUTPUT,
        TreeIcon::HistoryOutput => &HISTORY_OUTPUT,
        TreeIcon::HotSpot => &HOT_SPOT,
        TreeIcon::BoundaryCondition => &BOUNDARY_CONDITION,
        TreeIcon::Load => &LOAD,
        TreeIcon::DefinedField => &DEFINED_FIELD,
        TreeIcon::Running => &RUNNING,
        TreeIcon::Warning => &WARNING,
        TreeIcon::Geometry => &GEOMETRY,
        TreeIcon::MeshSetup => &MESH_SETUP,
        TreeIcon::Plane => &PLANE,
        TreeIcon::Path => &PATH,
        TreeIcon::PlaneResult => &PLANE_RESULT,
        TreeIcon::Dots => &DOTS,
        TreeIcon::DotsOpen => &DOTS_OPEN,
    }
}

/// Paints `icon` with its top left corner at `min`. Pixels are snapped to the screen raster
/// and runs of equal pixels merged, so that the icon stays sharp at any zoom factor. A plain
/// mesh is used because egui would blur the edges of single pixel rectangles.
pub fn paint(painter: &Painter, min: Pos2, icon: TreeIcon) {
    if let TreeIcon::StepType(glyph) = icon {
        paint_step_type(painter, min, glyph);
        return;
    }
    let ppp = painter.pixels_per_point();
    let snap = |v: f32| (v * ppp).round() / ppp;
    let pixmap = pixmap(icon);
    let mut mesh = Mesh::default();
    for (y, row) in pixmap.rows.iter().enumerate() {
        let row = row.as_bytes();
        let mut x = 0;
        while x < row.len() {
            let c = row[x];
            let start = x;
            while x < row.len() && row[x] == c {
                x += 1;
            }
            let index = match c {
                b'A'..=b'Z' => c - b'A',
                b'a'..=b'z' => c - b'a' + 26,
                _ => continue,
            };
            let [r, g, b, a] = pixmap.colors[index as usize];
            let rect = Rect::from_min_max(
                pos2(snap(min.x + start as f32), snap(min.y + y as f32)),
                pos2(snap(min.x + x as f32), snap(min.y + y as f32 + 1.0)),
            );
            mesh.add_colored_rect(
                rect,
                crate::style::adapt(Color32::from_rgba_unmultiplied(r, g, b, a)),
            );
        }
    }
    painter.add(Shape::mesh(mesh));
}

/// The pictograms of the step types, so that a step shows its type independent of its name.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepGlyph {
    Static,
    Frequency,
    ComplexFrequency,
    Buckle,
    HeatTransfer,
    CoupledTempDisp,
    UncoupledTempDisp,
    Dynamic,
    ModalDynamics,
    SteadyStateDynamics,
    Sensitivity,
    Visco,
}

impl StepGlyph {
    /// Name of the step type, shown in the tooltip.
    pub fn name(self) -> &'static str {
        match self {
            StepGlyph::Static => "Static",
            StepGlyph::Frequency => "Frequency",
            StepGlyph::ComplexFrequency => "Complex Frequency",
            StepGlyph::Buckle => "Buckle",
            StepGlyph::HeatTransfer => "Heat Transfer",
            StepGlyph::CoupledTempDisp => "Coupled Temperature-Displacement",
            StepGlyph::UncoupledTempDisp => "Uncoupled Temperature-Displacement",
            StepGlyph::Dynamic => "Dynamic",
            StepGlyph::ModalDynamics => "Modal Dynamics",
            StepGlyph::SteadyStateDynamics => "Steady State Dynamics",
            StepGlyph::Sensitivity => "Sensitivity",
            StepGlyph::Visco => "Visco",
        }
    }
}

impl From<&StepKind> for StepGlyph {
    fn from(kind: &StepKind) -> Self {
        match kind {
            StepKind::Static(_) => StepGlyph::Static,
            StepKind::Frequency(_) => StepGlyph::Frequency,
            StepKind::ComplexFrequency(_) => StepGlyph::ComplexFrequency,
            StepKind::Buckle(_) => StepGlyph::Buckle,
            StepKind::HeatTransfer(_) => StepGlyph::HeatTransfer,
            StepKind::CoupledTempDisp(_) => StepGlyph::CoupledTempDisp,
            StepKind::UncoupledTempDisp(_) => StepGlyph::UncoupledTempDisp,
            StepKind::Dynamic(_) => StepGlyph::Dynamic,
            StepKind::ModalDynamics(_) => StepGlyph::ModalDynamics,
            StepKind::SteadyStateDynamics(_) => StepGlyph::SteadyStateDynamics,
            StepKind::Sensitivity(_) => StepGlyph::Sensitivity,
            StepKind::Visco(_) => StepGlyph::Visco,
        }
    }
}

const INK: Color32 = Color32::from_rgb(70, 70, 70);
const BLUE: Color32 = Color32::from_rgb(52, 101, 190);
const RED: Color32 = Color32::from_rgb(214, 70, 40);
const GREEN: Color32 = Color32::from_rgb(40, 145, 80);
const ORANGE: Color32 = Color32::from_rgb(225, 140, 20);
const PURPLE: Color32 = Color32::from_rgb(130, 70, 170);

/// Draws the pictogram of a step type into the 16 x 16 square at `min` with strokes, so
/// that it scales with the display.
fn paint_step_type(painter: &Painter, min: Pos2, glyph: StepGlyph) {
    let at = |x: f32, y: f32| min + vec2(x, y);
    let line = |points: &[(f32, f32)], color: Color32| {
        let points = points.iter().map(|&(x, y)| at(x, y)).collect();
        painter.add(Shape::line(
            points,
            Stroke::new(1.5, crate::style::adapt(color)),
        ));
    };
    let curve = |from: f32, to: f32, color: Color32, y: &dyn Fn(f32) -> f32| {
        let steps = 28;
        let points = (0..=steps)
            .map(|i| {
                let x = from + (to - from) * i as f32 / steps as f32;
                (x, y(x))
            })
            .collect::<Vec<_>>();
        line(&points, color);
    };
    let axes = || line(&[(1.5, 1.0), (1.5, 14.5), (15.0, 14.5)], INK);
    let arrow = |from: (f32, f32), to: (f32, f32), color: Color32| {
        let (dx, dy) = (to.0 - from.0, to.1 - from.1);
        let length = dx.hypot(dy);
        let (ux, uy) = (dx / length, dy / length);
        line(&[from, to], color);
        let base = (to.0 - 3.0 * ux, to.1 - 3.0 * uy);
        let wing = |sign: f32| (base.0 - 2.0 * uy * sign, base.1 + 2.0 * ux * sign);
        painter.add(Shape::convex_polygon(
            vec![
                at(to.0, to.1),
                at(wing(1.0).0, wing(1.0).1),
                at(wing(-1.0).0, wing(-1.0).1),
            ],
            color,
            Stroke::NONE,
        ));
    };
    let tau = std::f32::consts::TAU;
    match glyph {
        // A load on a supported bar.
        StepGlyph::Static => {
            arrow((8.0, 0.5), (8.0, 7.5), RED);
            painter.rect_filled(
                Rect::from_min_size(at(1.5, 8.5), vec2(13.0, 3.0)),
                0.0,
                BLUE,
            );
            line(&[(3.0, 12.0), (3.0, 15.0)], INK);
            line(&[(13.0, 12.0), (13.0, 15.0)], INK);
        }
        // A standing wave.
        StepGlyph::Frequency => {
            line(&[(1.0, 8.0), (15.0, 8.0)], Color32::from_gray(190));
            curve(1.0, 15.0, BLUE, &|x| {
                8.0 - 5.5 * ((x - 1.0) / 14.0 * tau * 1.5).sin()
            });
        }
        // A decaying wave.
        StepGlyph::ComplexFrequency => {
            line(&[(1.0, 8.0), (15.0, 8.0)], Color32::from_gray(190));
            curve(1.0, 15.0, PURPLE, &|x| {
                let t = (x - 1.0) / 14.0;
                8.0 - 6.5 * (-1.6 * t).exp() * (t * tau * 2.0).sin()
            });
        }
        // A bent column under a load.
        StepGlyph::Buckle => {
            arrow((8.0, 0.5), (8.0, 4.5), RED);
            let points = (0..=20)
                .map(|i| {
                    let t = i as f32 / 20.0;
                    (8.0 + 3.5 * (std::f32::consts::PI * t).sin(), 4.5 + 9.0 * t)
                })
                .collect::<Vec<_>>();
            line(&points, BLUE);
            line(&[(3.5, 14.8), (12.5, 14.8)], INK);
        }
        // Rising heat.
        StepGlyph::HeatTransfer => {
            for x in [3.5, 8.0, 12.5] {
                let points = (0..=16)
                    .map(|i| {
                        let t = i as f32 / 16.0;
                        (x + 1.3 * (t * tau * 1.5).sin(), 14.5 - 13.0 * t)
                    })
                    .collect::<Vec<_>>();
                line(&points, RED);
            }
        }
        // Heat on a bar.
        StepGlyph::CoupledTempDisp => {
            for x in [4.5, 8.0, 11.5] {
                let points = (0..=10)
                    .map(|i| {
                        let t = i as f32 / 10.0;
                        (x + 1.1 * (t * tau * 1.5).sin(), 8.0 - 7.0 * t)
                    })
                    .collect::<Vec<_>>();
                line(&points, RED);
            }
            painter.rect_filled(
                Rect::from_min_size(at(1.5, 10.0), vec2(13.0, 3.0)),
                0.0,
                BLUE,
            );
            line(&[(3.0, 13.5), (3.0, 15.5)], INK);
            line(&[(13.0, 13.5), (13.0, 15.5)], INK);
        }
        // Heat flowing one way into a bar.
        StepGlyph::UncoupledTempDisp => {
            for x in [4.0, 8.0] {
                let points = (0..=10)
                    .map(|i| {
                        let t = i as f32 / 10.0;
                        (x + 1.1 * (t * tau * 1.5).sin(), 8.0 - 7.0 * t)
                    })
                    .collect::<Vec<_>>();
                line(&points, RED);
            }
            arrow((12.5, 1.0), (12.5, 8.0), ORANGE);
            painter.rect_filled(
                Rect::from_min_size(at(1.5, 10.0), vec2(13.0, 3.0)),
                0.0,
                BLUE,
            );
            line(&[(3.0, 13.5), (3.0, 15.5)], INK);
            line(&[(13.0, 13.5), (13.0, 15.5)], INK);
        }
        // A clock.
        StepGlyph::Dynamic => {
            painter.circle_stroke(at(8.0, 8.0), 6.5, Stroke::new(1.5, BLUE));
            line(&[(8.0, 3.5), (8.0, 8.0), (11.0, 10.0)], INK);
        }
        // Two superposed modes over time.
        StepGlyph::ModalDynamics => {
            axes();
            curve(2.0, 15.0, GREEN, &|x| {
                let t = (x - 2.0) / 13.0 * tau;
                8.0 - 3.5 * t.sin() - 2.5 * (3.0 * t + 0.6).sin()
            });
        }
        // A resonance peak over the frequency.
        StepGlyph::SteadyStateDynamics => {
            axes();
            curve(2.0, 15.0, ORANGE, &|x| {
                let d = (x - 8.0) / 1.8;
                14.0 - 12.0 / (1.0 + d * d)
            });
        }
        // A shape moved by a design step.
        StepGlyph::Sensitivity => {
            let arc = |x: f32| (std::f32::consts::PI * (x - 1.5) / 13.0).sin();
            curve(1.5, 14.5, INK, &|x| 12.5 - 4.0 * arc(x));
            curve(1.5, 14.5, BLUE, &|x| 7.0 - 4.0 * arc(x));
            arrow((8.0, 12.0), (8.0, 6.5), ORANGE);
        }
        // A creep curve: the strain grows on over time.
        StepGlyph::Visco => {
            axes();
            curve(2.0, 15.0, RED, &|x| {
                let t = (x - 2.0) / 13.0;
                12.5 - 4.0 * (1.0 - (-8.0 * t).exp()) - 6.0 * t
            });
        }
    }
}

/// Mesh.ico
const MESH: Pixmap = Pixmap {
    colors: &[
        [210, 210, 210, 255],
        [80, 80, 80, 255],
        [223, 223, 223, 255],
        [125, 125, 125, 255],
        [170, 170, 170, 255],
        [115, 115, 115, 255],
        [86, 86, 86, 255],
    ],
    rows: [
        "..........ABBBBB",
        ".........ABCCCBB",
        ".........BBBBBDB",
        ".ABBBBBB.BEEEBDB",
        "ABCCCCBB.BEEEBDB",
        "BBBBBBDB.BEEEBBA",
        "BEEEEBDB.BBBBBA.",
        "BEEEEBDB........",
        "BEEEEBFGGGGGG...",
        "BEEEEBBCCCCGG...",
        "BBBBBBBBBBBFG...",
        "BEEEEBEEEEBDB...",
        "BEEEEBEEEEBDB...",
        "BEEEEBEEEEBDB...",
        "BEEEEBEEEEBBA...",
        "BBBBBBBBBBBA....",
    ],
};

/// Part.ico
const PART: Pixmap = Pixmap {
    colors: &[
        [210, 210, 210, 255],
        [80, 80, 80, 255],
        [223, 223, 223, 255],
        [125, 125, 125, 255],
        [170, 170, 170, 255],
        [115, 115, 115, 255],
        [86, 86, 86, 255],
    ],
    rows: [
        "................",
        ".ABBBBBBB.......",
        "ABCCCCCBB.......",
        "BBBBBBBDB.......",
        "BEEEEEBDB.......",
        "BEEEEEBDB.......",
        "BEEEEEBDB.......",
        "BEEEEEBFGGGGGGG.",
        "BEEEEEBBCCCCCGG.",
        "BBBBBBBBBBBBBFG.",
        "BEEEEEBEEEEEBDB.",
        "BEEEEEBEEEEEBDB.",
        "BEEEEEBEEEEEBDB.",
        "BEEEEEBEEEEEBDB.",
        "BEEEEEBEEEEEBBA.",
        "BBBBBBBBBBBBBA..",
    ],
};

/// Node_set.ico
const NODE_SET: Pixmap = Pixmap {
    colors: &[
        [255, 0, 0, 94],
        [255, 0, 0, 255],
        [80, 80, 80, 255],
        [201, 107, 107, 255],
        [170, 170, 170, 255],
    ],
    rows: [
        "................",
        "ABBA............",
        "BBBB............",
        "BBBBCCCCCCCC....",
        "ABBDEEEEEEEC....",
        "..CEEEEEEEEC....",
        "..CEEEEEEEEC....",
        "..CEEEEEEEEC....",
        "..CEEEEEEEEC....",
        "..CEEEEEEEEC....",
        "..CEEEEEEEEC....",
        "ABBDEEEEEEDBBA..",
        "BBBBCCCCCCBBBB..",
        "BBBB......BBBB..",
        "ABBA......ABBA..",
        "................",
    ],
};

/// Element_set.ico
const ELEMENT_SET: Pixmap = Pixmap {
    colors: &[
        [245, 186, 186, 255],
        [255, 0, 0, 255],
        [251, 182, 182, 255],
        [255, 198, 198, 255],
        [255, 147, 147, 255],
        [86, 86, 86, 255],
        [223, 223, 223, 255],
        [80, 80, 80, 255],
        [115, 115, 115, 255],
        [170, 170, 170, 255],
        [125, 125, 125, 255],
        [210, 210, 210, 255],
    ],
    rows: [
        "................",
        ".ABBBBBBB.......",
        "CBDDDDDBB.......",
        "BBBBBBBCB.......",
        "BEEEEEBCB.......",
        "BEEEEEBCB.......",
        "BEEEEEBCB.......",
        "BEEEEEBCBFFFFFF.",
        "BEEEEEBBGGGGGFF.",
        "BBBBBBBHHHHHHIF.",
        "HJJJJJHJJJJJHKH.",
        "HJJJJJHJJJJJHKH.",
        "HJJJJJHJJJJJHKH.",
        "HJJJJJHJJJJJHKH.",
        "HJJJJJHJJJJJHHL.",
        "HHHHHHHHHHHHHL..",
    ],
};

/// Surface.ico
const SURFACE: Pixmap = Pixmap {
    colors: &[
        [210, 210, 210, 255],
        [80, 80, 80, 255],
        [223, 223, 223, 255],
        [125, 125, 125, 255],
        [255, 0, 0, 255],
        [115, 115, 115, 255],
        [86, 86, 86, 255],
        [170, 170, 170, 255],
    ],
    rows: [
        "................",
        ".ABBBBBBBB......",
        "ABCCCCCCBB......",
        "DBBBBBBBDB......",
        "DBEEEEEBDB......",
        "DBEEEEEBDB......",
        "DBEEEEEBDB......",
        "DBEEEEEBFGGGGGGG",
        "DBEEEEEBBCCCCCGG",
        "DBBBBBBBBBBBBBFG",
        "DBEEEEEBHHHHHBDB",
        "DBEEEEEBHHHHHBDB",
        "DBEEEEEBHHHHHBDB",
        "DBEEEEEBHHHHHBDB",
        "DBEEEEEBHHHHHBBA",
        "DBBBBBBBBBBBBBA.",
    ],
};

/// Feature.ico
const FEATURES: Pixmap = Pixmap {
    colors: &[[0, 0, 0, 255], [150, 150, 150, 255], [255, 0, 0, 255]],
    rows: [
        "................",
        "................",
        "..AAAAA.AAAAA...",
        "..ABBBA.ABBBA...",
        "..ABBBA.ABBBA...",
        "..ABBBA.ABBBA...",
        "..AAAAA.AAAAA...",
        "................",
        "..AAAAA.AAAAAAA.",
        "..ABBBA.ACCCCCA.",
        "..ABBBA.ACCCCCA.",
        "..ABBBA.ACCCCCA.",
        "..AAAAA.ACCCCCA.",
        "........ACCCCCA.",
        "........AAAAAAA.",
        "................",
    ],
};

/// Reference_point.ico
const REFERENCE_POINT: Pixmap = Pixmap {
    colors: &[
        [140, 0, 0, 255],
        [173, 0, 0, 255],
        [255, 0, 0, 255],
        [205, 0, 0, 255],
    ],
    rows: [
        "................",
        "................",
        "................",
        "................",
        "................",
        "......ABBA......",
        ".....ACCCCA.....",
        ".....DCCCCD.....",
        ".....DCCCCD.....",
        ".....ACCCCA.....",
        "......ABBA......",
        "................",
        "................",
        "................",
        "................",
        "................",
    ],
};

/// CoordinateSystem.ico
const COORDINATE_SYSTEM: Pixmap = Pixmap {
    colors: &[
        [30, 175, 0, 255],
        [85, 255, 51, 255],
        [40, 237, 0, 255],
        [239, 37, 37, 255],
        [248, 151, 151, 255],
        [0, 65, 174, 99],
        [0, 0, 0, 255],
        [208, 17, 15, 255],
        [208, 15, 15, 255],
        [0, 65, 174, 255],
        [2, 90, 242, 255],
    ],
    rows: [
        "......A.........",
        ".....BAB........",
        ".....CAC........",
        "......A.........",
        "......A.........",
        "......A.........",
        "......A.........",
        "......A.........",
        "......A......DE.",
        ".....FGHIIIIIIII",
        "....FJF......DE.",
        "...FJF..........",
        ".KFJF...........",
        ".KJF............",
        ".KKK............",
        "................",
    ],
};

/// Solid.ico
const SOLID: Pixmap = Pixmap {
    colors: &[
        [144, 176, 220, 150],
        [100, 136, 199, 255],
        [119, 155, 209, 255],
        [147, 180, 221, 255],
        [116, 153, 208, 255],
        [186, 210, 237, 255],
        [157, 187, 225, 255],
        [178, 204, 233, 129],
        [172, 199, 230, 255],
        [204, 223, 243, 255],
        [135, 168, 216, 255],
        [80, 110, 161, 255],
        [60, 84, 121, 213],
        [60, 84, 121, 255],
        [150, 182, 222, 106],
        [110, 147, 206, 146],
        [173, 200, 231, 91],
        [183, 207, 235, 42],
        [60, 84, 121, 143],
    ],
    rows: [
        ".......ABCA.....",
        ".....DEDFDBGA...",
        "..HEEIJJIDKELC..",
        ".LBIJJFIDKKKEBM.",
        ".NKKDFJIDDKCBBN.",
        ".NFJDEKFIDELKGN.",
        ".NIJFIKEKBBGIKN.",
        ".NIIGIIDNKIGKEN.",
        ".NIIDKDDNIGKCEN.",
        ".NIIKKKKNIDKEEN.",
        ".NEDKKKCNIDKEBM.",
        ".OLLEKCENIKCBLP.",
        "...CLBEENGKBLO..",
        "....ALLBNKBLQ...",
        ".....RPLNLL.....",
        ".......QSA......",
    ],
};

/// Shell.ico
const SHELL: Pixmap = Pixmap {
    colors: &[
        [183, 207, 235, 114],
        [74, 102, 148, 229],
        [99, 135, 198, 255],
        [60, 84, 121, 255],
        [135, 168, 216, 255],
        [145, 178, 220, 255],
        [188, 212, 237, 255],
        [201, 220, 241, 255],
        [118, 153, 208, 255],
        [183, 207, 235, 135],
        [174, 201, 231, 255],
        [137, 170, 217, 169],
        [156, 187, 225, 123],
        [86, 117, 171, 255],
        [125, 160, 212, 255],
        [104, 141, 203, 167],
        [144, 177, 220, 162],
        [183, 207, 235, 50],
        [105, 142, 204, 103],
        [60, 84, 121, 127],
    ],
    rows: [
        "....A...........",
        "....BCA.........",
        "....DEEFA.......",
        "....DGHFIJ......",
        "....DKHGKIEJ....",
        "....DKKKKKEIL...",
        "....DKKFEKKFD...",
        "....DKKFEEFFD...",
        "....DKKEEEFFD...",
        "....DIFEEEEED...",
        "....MNNIEEEOD...",
        "......PNCEOID...",
        ".......QNCIID...",
        "........ANNCD...",
        ".........RSBD...",
        "...........RT...",
    ],
};

/// Wire.ico
const WIRE: Pixmap = Pixmap {
    colors: &[
        [61, 85, 123, 206],
        [57, 80, 117, 148],
        [108, 134, 173, 255],
        [64, 88, 124, 204],
        [52, 76, 114, 102],
        [58, 82, 117, 80],
        [76, 101, 138, 255],
        [117, 151, 206, 255],
        [149, 178, 218, 255],
        [139, 168, 211, 255],
        [77, 104, 147, 255],
        [59, 83, 119, 45],
        [104, 138, 196, 255],
        [58, 82, 119, 176],
        [54, 78, 116, 49],
        [87, 120, 174, 255],
        [128, 154, 188, 255],
        [121, 146, 181, 255],
        [53, 78, 115, 148],
    ],
    rows: [
        "................",
        "................",
        "..AB............",
        ".BCCDE..........",
        "FGHIJKE.........",
        "LAKMJICNO.......",
        "..BKPHIQKE......",
        "...FAPHJJGN.....",
        ".....NKCJIRGE...",
        "......EGPHIJGS..",
        ".......LNKMJICAL",
        ".........EGPHJKF",
        "..........FAKKS.",
        "............BA..",
        "................",
        "................",
    ],
};

/// Hide.ico
const INACTIVE: Pixmap = Pixmap {
    colors: &[
        [248, 0, 0, 80],
        [248, 0, 0, 144],
        [248, 0, 0, 176],
        [248, 0, 0, 160],
        [248, 0, 0, 112],
        [248, 0, 0, 32],
        [248, 0, 0, 48],
        [248, 0, 0, 208],
        [248, 0, 0, 255],
        [248, 0, 0, 128],
        [248, 0, 0, 64],
        [248, 0, 0, 192],
        [248, 0, 0, 224],
        [248, 0, 0, 96],
        [248, 0, 0, 16],
    ],
    rows: [
        "................",
        ".....ABCDEF.....",
        "...GHIIIIIIB....",
        "..GIIJF..KHIL...",
        "..MIK....AIIIJ..",
        ".NIE....KIIEHI..",
        ".CIO...KIIE.EIA.",
        ".HH...KIIE..KIE.",
        ".HH..KIIE...KIE.",
        ".CIOKIIE....EIA.",
        ".NIDIIE.....MI..",
        "..HIIE.....DIE..",
        "..FIIBG.OAHIC...",
        "...FLIIIIIIJ....",
        ".....KJDDEO.....",
        "................",
    ],
};

const HIDDEN: Pixmap = Pixmap {
    colors: &[
        [90, 90, 90, 107],
        [90, 90, 90, 210],
        [90, 90, 90, 224],
        [90, 90, 90, 176],
        [90, 90, 90, 59],
        [90, 90, 90, 68],
        [90, 90, 90, 140],
    ],
    rows: [
        "................",
        "................",
        "................",
        ".....ABCCBA.....",
        "...ABDAEEADBA...",
        "..DDF......EDD..",
        ".DD..........GD.",
        "DB............DD",
        "DB............DD",
        ".DD..........GD.",
        "EGDDF......EDDGE",
        "GE.GBDAEEADBG.EG",
        "..EG.ABCCBA.GE..",
        "..GE..G..G..EG..",
        "......G..G......",
        "................",
    ],
};

/// Material.ico
const MATERIAL: Pixmap = Pixmap {
    colors: &[
        [87, 87, 87, 255],
        [190, 190, 190, 255],
        [118, 118, 118, 255],
        [255, 43, 43, 65],
        [239, 0, 0, 69],
        [255, 73, 73, 207],
        [253, 63, 63, 255],
        [234, 0, 0, 255],
        [178, 4, 4, 255],
        [235, 114, 114, 209],
        [199, 42, 42, 255],
        [239, 5, 5, 159],
        [255, 178, 178, 255],
        [255, 68, 68, 255],
        [255, 93, 93, 255],
        [248, 0, 0, 255],
        [233, 120, 120, 255],
        [154, 0, 0, 255],
        [208, 0, 0, 255],
    ],
    rows: [
        "................",
        ".A..............",
        "BAB.............",
        "CAC....DDDEEEE..",
        ".A...DFGHHHHHH..",
        ".A..DIJD........",
        ".A.DKL..........",
        ".A.MH...........",
        ".A.NO...........",
        ".ADP............",
        ".AQR............",
        ".APS............",
        ".AR.............",
        ".AS.........CB..",
        ".AAAAAAAAAAAAAA.",
        "............CB..",
    ],
};

/// Section.ico
const SECTION: Pixmap = Pixmap {
    colors: &[
        [231, 231, 231, 255],
        [87, 87, 87, 255],
        [189, 189, 189, 255],
        [223, 223, 223, 255],
        [155, 155, 155, 255],
        [113, 113, 113, 255],
        [215, 215, 215, 255],
        [90, 90, 90, 255],
    ],
    rows: [
        "................",
        "...........AB...",
        "BBBBBBB...ABB.B.",
        "BCDCEFB..ABGB.B.",
        "BCDCEFB.ABGGB.B.",
        "BCDCEFB.BGGGB.B.",
        "BCDCEFB.BGGGB.B.",
        "BCDCEFB.BGGGB.B.",
        "BCDCEFB.BGGGB.B.",
        "BCDCEFB.BGGGB.B.",
        "BCDCEFB.BGGGB.B.",
        "BCDCEFB.BGGGB.B.",
        "BCDCEFB.BGGBA.B.",
        "BBBBBHB.BGBA..B.",
        "........BBA.....",
        "........BA......",
    ],
};

/// Constraints.ico
const CONSTRAINTS: Pixmap = Pixmap {
    colors: &[
        [80, 80, 80, 255],
        [170, 170, 170, 255],
        [255, 0, 0, 94],
        [255, 0, 0, 255],
        [80, 80, 80, 204],
    ],
    rows: [
        ".........AAAAAA.",
        "........AABBBBA.",
        ".......A.ABBBBA.",
        "......A..ABBBBA.",
        ".....A...ABBBBA.",
        "....A...AAAAAAA.",
        "CDDC..AA.ABBBBA.",
        "DDDDEE...ABBBBA.",
        "DDDDEE...ABBBBA.",
        "CDDC..AA.ABBBBA.",
        "....A...AAAAAAA.",
        ".....A...ABBBBA.",
        "......A..ABBBBA.",
        ".......A.ABBBBA.",
        "........AABBBBA.",
        ".........AAAAAA.",
    ],
};

/// Contact.ico
const CONTACTS: Pixmap = Pixmap {
    colors: &[
        [0, 0, 0, 255],
        [149, 149, 150, 255],
        [134, 134, 134, 255],
        [155, 154, 159, 255],
        [148, 148, 150, 255],
        [150, 149, 155, 255],
        [164, 164, 167, 255],
        [140, 140, 140, 255],
        [157, 156, 161, 255],
        [255, 0, 0, 255],
        [255, 205, 205, 255],
        [255, 133, 133, 255],
        [255, 166, 166, 255],
        [104, 104, 104, 255],
        [161, 160, 164, 255],
    ],
    rows: [
        "AAAAAAAAA.......",
        "ABBCBBBBA.......",
        "ADDEDDDBA.......",
        "AFFBBFFGH.......",
        "AIFDFBBJK.......",
        "AEIFJLBJKMJ.....",
        "AIFILJLJMJM.....",
        "ABBBBLJJJMKKNAAA",
        "AAAJJJJJJJJJOBBA",
        "...KKMJJJLBIIFIA",
        "....MJMJLJLFFIEA",
        "....JMKJBLJDDFIA",
        "......KJBFFBBFFA",
        ".......ABDDDEDDA",
        ".......ABBBBCBBA",
        ".......AAAAAAAAA",
    ],
};

/// SurfaceInteraction.ico
const SURFACE_INTERACTION: Pixmap = Pixmap {
    colors: &[
        [0, 0, 0, 255],
        [148, 148, 150, 255],
        [143, 143, 144, 255],
        [147, 147, 149, 255],
        [138, 138, 139, 255],
        [255, 0, 0, 255],
        [145, 145, 147, 255],
        [144, 144, 146, 255],
        [0, 0, 0, 83],
        [146, 146, 148, 255],
        [143, 143, 145, 255],
        [172, 0, 0, 255],
        [144, 144, 145, 255],
        [139, 139, 140, 255],
        [141, 141, 143, 255],
        [140, 140, 141, 255],
    ],
    rows: [
        "AAAAAAAAA.......",
        "ABBCCCBBA.......",
        "ADECCCCCAF......",
        "AGHCCCCAIF......",
        "ABBJKKBA.FFF....",
        "AGDCJJA....F....",
        "AGDBBAL....FFF..",
        "ABBAA.FFF...IAAA",
        "AAAL....F..AABBA",
        "...F....FLABBDJA",
        "...FFF...ABMJJMA",
        ".....F..ABJNCOHA",
        ".....FFLACCCCNHA",
        ".......ACCCCCPBA",
        ".......ABJCCCMBA",
        ".......AAAAAAAAA",
    ],
};

/// ContactPair.ico
const CONTACT_PAIR: Pixmap = Pixmap {
    colors: &[
        [0, 0, 0, 255],
        [255, 0, 0, 255],
        [0, 0, 0, 83],
        [0, 0, 0, 77],
        [0, 0, 0, 230],
        [187, 29, 255, 255],
    ],
    rows: [
        "AAAAAAAAA.......",
        "ABBBBBBBA.......",
        "ABBBBBBBA.......",
        "ABBBBBBAC.......",
        "ABBBBBBA........",
        "ABBBBBA.........",
        "ABBBBA..........",
        "ABBAA.......DAAE",
        "AAAC.......AAFFA",
        "..........AFFFFA",
        ".........AFFFFFA",
        "........AFFFFFFA",
        ".......CAFFFFFFA",
        ".......AFFFFFFFA",
        ".......AFFFFFFFA",
        ".......AAAAAAAAA",
    ],
};

/// Distribution.ico
const DISTRIBUTION: Pixmap = Pixmap {
    colors: &[
        [80, 80, 80, 255],
        [207, 216, 231, 255],
        [190, 208, 243, 255],
        [176, 200, 247, 255],
        [137, 169, 248, 255],
        [119, 151, 241, 255],
        [81, 107, 213, 255],
        [61, 78, 194, 255],
        [220, 219, 220, 255],
        [163, 190, 247, 255],
        [228, 213, 207, 255],
        [153, 184, 251, 255],
        [105, 136, 233, 255],
        [236, 205, 192, 255],
        [242, 188, 167, 255],
        [242, 167, 141, 255],
        [239, 198, 182, 255],
        [237, 142, 115, 255],
        [243, 179, 155, 255],
        [227, 112, 89, 255],
    ],
    rows: [
        "................",
        "................",
        "..AAAAAAAAAAAA..",
        "..ABBCDEFGGHHA..",
        "..AIBBDJEFGGHA..",
        "..AKIBBDLEMGGA..",
        "..ANKIBBDLEMGA..",
        "..AOKKIBBDLFMA..",
        "..APQKKIBCDEFA..",
        "..ARSQKIBBDJEA..",
        "..ARPOQKIBBDLA..",
        "..ATRPONKIBBJA..",
        "..ATTRPOKKIBBA..",
        "..AAAAAAAAAAAA..",
        "................",
        "................",
    ],
};

/// Amplitude.ico
const AMPLITUDE: Pixmap = Pixmap {
    colors: &[
        [87, 87, 87, 255],
        [190, 190, 190, 255],
        [118, 118, 118, 255],
        [255, 0, 0, 122],
        [255, 0, 0, 255],
        [255, 0, 0, 186],
    ],
    rows: [
        "................",
        ".A..............",
        "BAB.............",
        "CAC......DEEEEE.",
        ".A......FE......",
        ".A.....DE.......",
        ".A.....ED.......",
        ".A.....E........",
        ".A.....E........",
        ".A.....E........",
        ".A....DE........",
        ".A....ED........",
        ".A...EF.........",
        ".AEEED......CB..",
        ".AAAAAAAAAAAAAA.",
        "............CB..",
    ],
};

/// Initial_conditions.ico
const INITIAL_CONDITIONS: Pixmap = Pixmap {
    colors: &[
        [80, 80, 80, 255],
        [214, 214, 214, 255],
        [255, 0, 0, 255],
        [226, 187, 187, 255],
    ],
    rows: [
        "................",
        "................",
        "AAAAAAAAAAAAAAAA",
        "ABBBBBBBBBBBBBBA",
        "ABBBBCBBBBBBBBBA",
        "ABBBBCBBBBBBBBBA",
        "ABBBCCCCBBBBBBBA",
        "ABBBBCBBBDCCDBBA",
        "ABBBBCBBBCDDCBBA",
        "ABBBBCBBBCBBCBBA",
        "ABBBBCDCBCDDCBBA",
        "ABBBBDCDBDCCDBBA",
        "ABBBBBBBBBBBBBBA",
        "AAAAAAAAAAAAAAAA",
        "................",
        "................",
    ],
};

/// Steps.ico
const STEPS: Pixmap = Pixmap {
    colors: &[
        [40, 40, 40, 255],
        [23, 23, 24, 255],
        [30, 32, 34, 255],
        [4, 4, 5, 255],
        [103, 103, 105, 255],
        [113, 113, 113, 255],
        [126, 153, 189, 255],
        [49, 49, 49, 255],
        [26, 26, 26, 82],
        [140, 167, 204, 255],
        [67, 68, 68, 255],
        [91, 91, 91, 255],
        [157, 183, 222, 255],
        [42, 42, 42, 53],
        [167, 191, 226, 255],
    ],
    rows: [
        "................",
        "................",
        "................",
        "ABCBBA.BBBDDE...",
        "FEGGGHIFEGGGD...",
        ".DJJJEK.DJJJLE..",
        ".FEMMMBNEEMMMD..",
        "..DOOOEE.DOOOLE.",
        "..HEOOOD.HEOOOD.",
        "..DOOOEE.DOOOLE.",
        ".FEMMMBNEEMMMD..",
        ".DJJJEK.DJJJLE..",
        "FEGGGHIFEGGGD...",
        "ABBBBA.BBBDDE...",
        "................",
        "................",
    ],
};

/// Step.ico
const STEP: Pixmap = Pixmap {
    colors: &[
        [40, 40, 40, 255],
        [23, 24, 25, 255],
        [42, 43, 44, 255],
        [115, 115, 115, 255],
        [100, 100, 101, 255],
        [126, 153, 189, 255],
        [28, 28, 30, 82],
        [2, 4, 6, 255],
        [67, 67, 67, 255],
        [140, 167, 204, 255],
        [150, 177, 214, 255],
        [167, 191, 226, 255],
        [126, 127, 128, 255],
    ],
    rows: [
        "................",
        ".ABBBBBBCBBA....",
        ".DEFFFFFFFFCG...",
        "..HFFFFFFFFDI...",
        "..DEJJJJJJJJB...",
        "...HKKKKKKKKEE..",
        "...CELLLLLLLLH..",
        "....HLLLLLLLLEE.",
        "....CELLLLLLLLH.",
        "....HLLLLLLLLEE.",
        "...DELLLLLLLLB..",
        "...HKKKKKKKKEE..",
        "..DEKKKKKKKKB...",
        "..HFFFFFFFFMI...",
        ".DEFFFFFFFFCG...",
        ".ABCBBBBBBBA....",
    ],
};

/// Field_output.ico
const FIELD_OUTPUT: Pixmap = Pixmap {
    colors: &[
        [66, 66, 66, 190],
        [80, 80, 80, 255],
        [180, 4, 38, 255],
        [222, 97, 77, 255],
        [244, 154, 123, 255],
        [245, 196, 173, 255],
        [221, 221, 221, 255],
        [204, 204, 255, 255],
        [86, 86, 86, 255],
        [184, 208, 249, 255],
        [141, 176, 254, 255],
        [98, 130, 234, 255],
        [59, 76, 192, 255],
        [0, 0, 0, 84],
    ],
    rows: [
        "................",
        "..ABBBBBBB......",
        ".ABCDEFGBB......",
        ".BBBBBBBHB......",
        ".BCDEFGBHB......",
        ".BCDEFGBHB......",
        ".BCDEFGBHB......",
        ".BCDEFGBHIIIIIII",
        ".BCDEFGBBJKLMMII",
        ".BCDEFGBBBBBBBMI",
        ".BCDEFGGJKLMMBMB",
        ".BCDEFGGJKLMMBMB",
        ".BCDEFGGJKLMMBMB",
        ".BCDEFGGJKLMMBMB",
        ".BCDEFGGJKLMMBBN",
        ".BBBBBBBBBBBBBN.",
    ],
};

/// History_output.ico
/// Stress rising towards a weld toe with two read-out points.
const HOT_SPOT: Pixmap = Pixmap {
    colors: &[
        [60, 60, 60, 255],
        [220, 0, 0, 255],
        [40, 40, 40, 255],
        [150, 150, 150, 255],
        [0, 90, 200, 255],
    ],
    rows: [
        "....A...........",
        "....AB..........",
        "....AB..........",
        "....A.B.........",
        "....A.EE........",
        "....A.EEB.......",
        "....A....BB.....",
        "....A......BEE..",
        "....A.......EEBB",
        "....A...........",
        "....A...........",
        "...CA...........",
        "..CCA...........",
        ".CCCA...........",
        "DDDDDDDDDDDDDDDD",
        "DDDDDDDDDDDDDDDD",
    ],
};

const HISTORY_OUTPUT: Pixmap = Pixmap {
    colors: &[
        [254, 68, 68, 255],
        [87, 87, 87, 255],
        [174, 0, 0, 255],
        [190, 190, 190, 255],
        [254, 201, 201, 255],
        [244, 1, 1, 255],
        [118, 118, 118, 255],
        [254, 91, 91, 255],
        [255, 29, 29, 255],
        [244, 140, 140, 255],
        [230, 148, 148, 255],
        [153, 0, 0, 255],
        [255, 166, 166, 255],
        [255, 178, 178, 255],
        [255, 36, 36, 255],
        [239, 239, 239, 73],
        [204, 0, 0, 255],
        [245, 116, 116, 255],
    ],
    rows: [
        "...............A",
        ".B.............C",
        "DBD...........EF",
        "GBG...........HA",
        ".B............IJ",
        ".B..KCCH......L.",
        ".B..C..FI....MF.",
        ".B.NF...FF...OJ.",
        ".B.AH....FF.PC..",
        ".B.FN.....ACQR..",
        ".B.L............",
        ".B.Q............",
        ".B..............",
        ".B..........GD..",
        ".BBBBBBBBBBBBBB.",
        "............GD..",
    ],
};

/// Bc.ico
const BOUNDARY_CONDITION: Pixmap = Pixmap {
    colors: &[
        [210, 210, 210, 255],
        [80, 80, 80, 255],
        [223, 223, 223, 255],
        [125, 125, 125, 255],
        [170, 170, 170, 255],
        [115, 115, 115, 255],
        [86, 86, 86, 255],
        [255, 0, 0, 255],
    ],
    rows: [
        "..ABBBBBB.......",
        ".ABCCCCBB.......",
        ".BBBBBBDB.......",
        ".BEEEEBDB.......",
        ".BEEEEBDB.......",
        ".BEEEEBFGGGGGG..",
        ".BEEEEBBCCCCGG..",
        ".BBBBBBBBBBBFG..",
        ".BEEEEBEEEEBDB..",
        ".BEEEEBEEEEBDB..",
        ".BEEEEBEEEEBDB..",
        ".BEEEEBEEEEBBA..",
        ".BBBBBBBBBBBA...",
        "..HH..HH..HH....",
        "...HH..HH..HH...",
        "....HH..HH..HH..",
    ],
};

/// Load.ico
const LOAD: Pixmap = Pixmap {
    colors: &[
        [255, 0, 0, 255],
        [210, 210, 210, 255],
        [80, 80, 80, 255],
        [223, 223, 223, 255],
        [125, 125, 125, 255],
        [170, 170, 170, 255],
        [115, 115, 115, 255],
        [86, 86, 86, 255],
    ],
    rows: [
        "................",
        "...........AA...",
        "...........AA...",
        ".BCCCCCC...AA...",
        "BCDDDDCC...AA...",
        "CCCCCCEC.AAAAAA.",
        "CFFFFCEC..AAAA..",
        "CFFFFCEC...AA...",
        "CFFFFCGHHHHAA...",
        "CFFFFCCDDDDHH...",
        "CCCCCCCCCCCGH...",
        "CFFFFCFFFFCEC...",
        "CFFFFCFFFFCEC...",
        "CFFFFCFFFFCEC...",
        "CFFFFCFFFFCCB...",
        "CCCCCCCCCCCB....",
    ],
};

/// Defined_field.ico
const DEFINED_FIELD: Pixmap = Pixmap {
    colors: &[
        [66, 66, 66, 190],
        [80, 80, 80, 255],
        [180, 4, 38, 255],
        [222, 97, 77, 255],
        [244, 154, 123, 255],
        [245, 196, 173, 255],
        [221, 221, 221, 255],
        [204, 204, 255, 255],
        [86, 86, 86, 255],
        [184, 208, 249, 255],
        [141, 176, 254, 255],
        [98, 130, 234, 255],
        [59, 76, 192, 255],
        [0, 0, 0, 84],
    ],
    rows: [
        "................",
        "..ABBBBBBB......",
        ".ABCDEFGBB......",
        ".BBBBBBBHB......",
        ".BCDEFGBHB......",
        ".BCDEFGBHB......",
        ".BCDEFGBHB......",
        ".BCDEFGBHIIIIIII",
        ".BCDEFGBBJKLMMII",
        ".BCDEFGBBBBBBBMI",
        ".BCDEFGGJKLMMBMB",
        ".BCDEFGGJKLMMBMB",
        ".BCDEFGGJKLMMBMB",
        ".BCDEFGGJKLMMBMB",
        ".BCDEFGGJKLMMBBN",
        ".BBBBBBBBBBBBBN.",
    ],
};

/// Running.ico
const RUNNING: Pixmap = Pixmap {
    colors: &[
        [0, 0, 0, 44],
        [0, 0, 0, 255],
        [0, 0, 0, 189],
        [0, 0, 0, 125],
    ],
    rows: [
        "................",
        "........A.BB.A..",
        ".......ABBBBBBA.",
        "........BC..CB..",
        ".......BB.CC.BB.",
        ".......BB.CC.BB.",
        "........BC..CB..",
        "..A.BB.DBBBBBBA.",
        ".ABBBBBBD.BB.A..",
        "..BC..CB........",
        ".BB.CC.BB.......",
        ".BB.CC.BB.......",
        "..BC..CB........",
        ".ABBBBBBA.......",
        "..A.BB.A........",
        "................",
    ],
};

/// Warning.ico
const WARNING: Pixmap = Pixmap {
    colors: &[
        [254, 169, 0, 255],
        [255, 208, 114, 255],
        [250, 166, 0, 255],
        [255, 251, 244, 255],
    ],
    rows: [
        "................",
        ".......AA.......",
        "......BCCB......",
        "......ACCA......",
        ".....BCCCCB.....",
        ".....ACDDCA.....",
        "....BCCDDCCB....",
        "....ACCDDCCA....",
        "...BCCCDDCCCB...",
        "...ACCCDDCCCA...",
        "..BCCCCCCCCCCB..",
        "..ACCCCDDCCCCA..",
        ".BCCCCCDDCCCCCB.",
        ".ACCCCCCCCCCCCA.",
        ".BCCCCCCCCCCCCB.",
        "................",
    ],
};

/// Geometry.ico
const GEOMETRY: Pixmap = Pixmap {
    colors: &[
        [210, 210, 210, 255],
        [80, 80, 80, 255],
        [223, 223, 223, 255],
        [125, 125, 125, 255],
        [170, 170, 170, 255],
        [115, 115, 115, 255],
        [86, 86, 86, 255],
    ],
    rows: [
        "..........ABBBBB",
        ".........ABCCCBB",
        ".........BBBBBDB",
        ".ABBBBBB.BEEEBDB",
        "ABCCCCBB.BEEEBDB",
        "BBBBBBDB.BEEEBBA",
        "BEEEEBDB.BBBBBA.",
        "BEEEEBDB........",
        "BEEEEBFGGGGGG...",
        "BEEEEBBCCCCGG...",
        "BEEEEBBBBBBFG...",
        "BEEEEEEEEEBDB...",
        "BEEEEEEEEEBDB...",
        "BEEEEEEEEEBDB...",
        "BEEEEEEEEEBBA...",
        "BBBBBBBBBBBA....",
    ],
};

/// Mesh_refinement.ico
const MESH_SETUP: Pixmap = Pixmap {
    colors: &[
        [255, 0, 0, 94],
        [255, 0, 0, 255],
        [80, 80, 80, 255],
        [170, 170, 170, 255],
        [201, 107, 107, 255],
    ],
    rows: [
        "................",
        ".ABBA......ABBA.",
        ".BBBBCCCCCCBBBB.",
        ".BBBBDDDDDDBBBB.",
        ".ABBEDDDDDDEBBA.",
        "..CDDDDDDDDDDC..",
        ".ABBEDDDDDDDDC..",
        ".BBBBDDDDDDDDC..",
        ".BBBBDDDDDDDDC..",
        ".ABBEDDDDDDDDC..",
        "..CDDDDDDDDDDC..",
        ".ABBEDEBBEDEBBA.",
        ".BBBBDBBBBDBBBB.",
        ".BBBBCBBBBCBBBB.",
        ".ABBA.ABBA.ABBA.",
        "................",
    ],
};

/// A plane with its normal.
const PLANE: Pixmap = Pixmap {
    colors: &[
        [0, 110, 160, 255],
        [175, 215, 240, 255],
        [0, 80, 130, 255],
        [0, 50, 90, 255],
    ],
    rows: [
        "........A.......",
        ".......AAA......",
        "......A.A.A.....",
        "........A.......",
        "........A.......",
        "........A.......",
        ".......CACCCCCCC",
        "......CBABBBBBC.",
        ".....CBBABBBBC..",
        "....CBBBDBBBC...",
        "...CBBBBBBBC....",
        "..CBBBBBBBC.....",
        ".CCCCCCCCC......",
        "................",
        "................",
        "................",
    ],
};

/// A straight path between two points.
const PATH: Pixmap = Pixmap {
    colors: &[[200, 0, 160, 255], [255, 255, 255, 255]],
    rows: [
        "................",
        "............AAA.",
        "............ABA.",
        "............AAA.",
        "...........A....",
        "..........A.....",
        ".........A......",
        "........A.......",
        ".......A........",
        "......A.........",
        ".....A..........",
        "....A...........",
        ".AAA............",
        ".ABA............",
        ".AAA............",
        "................",
    ],
};

/// A plane coloured by results.
const PLANE_RESULT: Pixmap = Pixmap {
    colors: &[
        [70, 70, 70, 255],
        [40, 40, 255, 255],
        [0, 200, 255, 255],
        [0, 230, 120, 255],
        [120, 240, 0, 255],
        [255, 235, 0, 255],
        [255, 140, 0, 255],
        [255, 30, 30, 255],
    ],
    rows: [
        "................",
        "................",
        "................",
        "................",
        "................",
        "................",
        ".......AAAAAAAAA",
        "......AEEFGGHHA.",
        ".....ADEEFGGHA..",
        "....ACDEEFGGA...",
        "...ACCDEEFGA....",
        "..ABCCDEEFA.....",
        ".AAAAAAAAA......",
        "................",
        "................",
        "................",
    ],
};

/// Dots.ico
const DOTS: Pixmap = Pixmap {
    colors: &[[109, 109, 109, 255]],
    rows: [
        "................",
        "................",
        "................",
        "................",
        "................",
        "................",
        "................",
        "................",
        "................",
        ".A.A.A.A.A.A.A.A",
        "................",
        "................",
        "................",
        "................",
        "................",
        "................",
    ],
};

/// Dots_t.ico
const DOTS_OPEN: Pixmap = Pixmap {
    colors: &[[109, 109, 109, 255]],
    rows: [
        "................",
        "................",
        "................",
        "................",
        "................",
        "................",
        "................",
        "................",
        "................",
        ".A.A.A.A.A.A.A.A",
        "................",
        "........A.......",
        "................",
        "........A.......",
        "................",
        "........A.......",
    ],
};

#[cfg(test)]
mod tests {
    use super::*;

    /// Every pixel names a colour of its icon.
    #[test]
    fn pixmaps_are_complete() {
        for icon in [
            TreeIcon::Mesh,
            TreeIcon::Part,
            TreeIcon::NodeSet,
            TreeIcon::ElementSet,
            TreeIcon::Surface,
            TreeIcon::Features,
            TreeIcon::ReferencePoint,
            TreeIcon::CoordinateSystem,
            TreeIcon::Solid,
            TreeIcon::Shell,
            TreeIcon::Wire,
            TreeIcon::Hidden,
            TreeIcon::Inactive,
            TreeIcon::Material,
            TreeIcon::Section,
            TreeIcon::Constraints,
            TreeIcon::Contacts,
            TreeIcon::SurfaceInteraction,
            TreeIcon::ContactPair,
            TreeIcon::Distribution,
            TreeIcon::Amplitude,
            TreeIcon::InitialConditions,
            TreeIcon::Steps,
            TreeIcon::Step,
            TreeIcon::FieldOutput,
            TreeIcon::HistoryOutput,
            TreeIcon::BoundaryCondition,
            TreeIcon::Load,
            TreeIcon::DefinedField,
            TreeIcon::Running,
            TreeIcon::Warning,
            TreeIcon::Geometry,
            TreeIcon::MeshSetup,
            TreeIcon::HotSpot,
            TreeIcon::Plane,
            TreeIcon::Path,
            TreeIcon::PlaneResult,
            TreeIcon::Dots,
            TreeIcon::DotsOpen,
        ] {
            let pixmap = pixmap(icon);
            for row in pixmap.rows {
                assert_eq!(row.len(), 16, "{icon:?}");
                for c in row.bytes() {
                    let index = match c {
                        b'.' => continue,
                        b'A'..=b'Z' => c - b'A',
                        b'a'..=b'z' => c - b'a' + 26,
                        _ => panic!("{icon:?}: {c}"),
                    };
                    assert!((index as usize) < pixmap.colors.len(), "{icon:?}");
                }
            }
        }
    }
}
