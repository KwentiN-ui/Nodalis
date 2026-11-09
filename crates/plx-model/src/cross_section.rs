//! Cross-sections of beams, kept apart from the sections that use them: a named profile a
//! beam section picks by name, so that several beam sections, and analyses linked in the
//! project, share one definition.
//!
//! Coordinates in the plane of a profile are those of CalculiX's `*BEAM SECTION`: `u` along
//! the 1-direction (the section's normal) and `v` along the 2-direction (the beam axis crossed
//! with the 1-direction), about the centre of the profile.

use serde::{Deserialize, Serialize};

use crate::{Field, Quantity};

/// Points on the outline of a round profile, in [`BeamProfile::outlines`].
pub const ROUND_POINTS: usize = 24;

/// A named beam profile.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CrossSection {
    /// The internal identity other items refer to; the user sees only the name.
    pub id: crate::Id,
    pub name: String,
    pub profile: BeamProfile,
}

impl CrossSection {
    /// Prefix of new cross-sections' names, like `Cross_Section-1`.
    pub const PREFIX: &'static str = "Cross_Section";

    pub fn new(name: impl Into<String>, profile: BeamProfile) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            profile,
        }
    }
}

/// Cross-section of a beam, with CalculiX's dimensions: the 1-direction is the normal of
/// the section, the 2-direction the beam axis crossed with it.
///
/// Rectangles, circles, pipes and boxes are CalculiX's own beam sections, which it expands
/// into solids. The others are *general* profiles: the solver knows them only as the user
/// element `U1` with `SECTION=GENERAL`, a linear beam given by its area and second moments of
/// area, whose axis runs through the centroid. They lie with the centroid on the nodes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum BeamProfile {
    /// Rectangle with thickness `a` in the 1-direction and `b` in the 2-direction.
    Rect { a: f64, b: f64 },
    /// Circle; the solver takes the diameter, see [`BeamProfile::data_line`].
    Circ { radius: f64 },
    /// Hollow circle with outer radius and wall thickness (`B32R` only).
    Pipe { radius: f64, thickness: f64 },
    /// Hollow rectangle `a` by `b` with four wall thicknesses, in the order the solver reads
    /// them: at +1, +2, -1 and -2 (`B32R` only).
    Box { a: f64, b: f64, t: [f64; 4] },
    /// I-beam of height `h` in the 2-direction with flanges `b` wide, web `tw` and flanges
    /// `tf` thick.
    I { h: f64, b: f64, tw: f64, tf: f64 },
    /// T-beam like the I-beam with its flange at +2 only.
    T { h: f64, b: f64, tw: f64, tf: f64 },
    /// Channel with its web at -1 and the flanges reaching towards +1.
    U { h: f64, b: f64, tw: f64, tf: f64 },
    /// Angle with legs `h` along +2 and `b` along +1 from the corner, `t` thick.
    L { h: f64, b: f64, t: f64 },
    /// A profile given by its values alone: the area, the second moments of area about the
    /// 1- and 2-axis through the centroid, and CalculiX's factor `k` of the shear and torsion
    /// stiffness, see [`BeamProfile::data_line`].
    General {
        area: f64,
        i11: f64,
        i22: f64,
        k: f64,
    },
    /// A profile sketched as polygons, see [`Sketch`].
    Sketch(Sketch),
}

/// A profile drawn as closed polygons in the 1-2 plane: the outline first, then its holes.
/// It lies with its centroid on the nodes, wherever it was drawn.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Sketch {
    pub loops: Vec<Vec<[f64; 2]>>,
}

impl Sketch {
    /// The outlines of a profile as a sketch to edit: a round one as a polygon.
    pub fn of(profile: &BeamProfile) -> Self {
        Self {
            loops: profile.outlines().into_iter().map(|o| o.points).collect(),
        }
    }

    /// Why the polygons make no profile, if they do not: too few points, an outline crossing
    /// itself or another, a hole outside the outline or inside another hole.
    pub fn problem(&self) -> Option<String> {
        let Some(outer) = self.loops.first() else {
            return Some("The sketch has no outline".into());
        };
        for (k, points) in self.loops.iter().enumerate() {
            if points.len() < 3 {
                return Some(format!("{} has fewer than 3 points", loop_name(k)));
            }
            if points.iter().flatten().any(|c| !c.is_finite()) {
                return Some(format!("{} has an invalid point", loop_name(k)));
            }
            if polygon_area(points).abs() <= f64::EPSILON * extent(points).powi(2) {
                return Some(format!("{} encloses no area", loop_name(k)));
            }
        }
        // No two edges may cross or touch but at the corner they share.
        let edges: Vec<(usize, usize, [f64; 2], [f64; 2])> = (self.loops.iter().enumerate())
            .flat_map(|(k, points)| {
                (0..points.len()).map(move |i| (k, i, points[i], points[(i + 1) % points.len()]))
            })
            .collect();
        for (a, &(ka, ia, p, q)) in edges.iter().enumerate() {
            for &(kb, ib, r, t) in &edges[a + 1..] {
                let n = self.loops[ka].len();
                let neighbours = ka == kb && (ib == (ia + 1) % n || ia == (ib + 1) % n);
                if !neighbours && segments_meet(p, q, r, t) {
                    return Some(if ka == kb {
                        format!("{} crosses itself", loop_name(ka))
                    } else {
                        format!("{} and {} cross", loop_name(ka), loop_name(kb))
                    });
                }
            }
        }
        for (k, hole) in self.loops.iter().enumerate().skip(1) {
            if !inside(hole[0], outer) {
                return Some(format!("{} lies outside the outline", loop_name(k)));
            }
            if (self.loops[1..].iter().enumerate())
                .any(|(j, other)| j + 1 != k && inside(hole[0], other))
            {
                return Some(format!("{} lies inside another hole", loop_name(k)));
            }
        }
        None
    }
}

/// "The outline" or "Hole 2", for messages.
fn loop_name(k: usize) -> String {
    if k == 0 {
        "The outline".into()
    } else {
        format!("Hole {k}")
    }
}

/// The signed area of a polygon, positive counter-clockwise.
fn polygon_area(points: &[[f64; 2]]) -> f64 {
    let n = points.len();
    (0..n)
        .map(|i| {
            let ([x0, y0], [x1, y1]) = (points[i], points[(i + 1) % n]);
            x0 * y1 - x1 * y0
        })
        .sum::<f64>()
        / 2.0
}

/// The largest coordinate difference of the points.
fn extent(points: &[[f64; 2]]) -> f64 {
    let (mut min, mut max) = ([f64::MAX; 2], [f64::MIN; 2]);
    for &[u, v] in points {
        min = [min[0].min(u), min[1].min(v)];
        max = [max[0].max(u), max[1].max(v)];
    }
    (max[0] - min[0]).max(max[1] - min[1])
}

/// Whether the segments p-q and r-t have a point in common.
fn segments_meet(p: [f64; 2], q: [f64; 2], r: [f64; 2], t: [f64; 2]) -> bool {
    let cross = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let within = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        c[0] >= a[0].min(b[0])
            && c[0] <= a[0].max(b[0])
            && c[1] >= a[1].min(b[1])
            && c[1] <= a[1].max(b[1])
    };
    let (d1, d2) = (cross(r, t, p), cross(r, t, q));
    let (d3, d4) = (cross(p, q, r), cross(p, q, t));
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }
    (d1 == 0.0 && within(r, t, p))
        || (d2 == 0.0 && within(r, t, q))
        || (d3 == 0.0 && within(p, q, r))
        || (d4 == 0.0 && within(p, q, t))
}

/// Whether a point lies inside a polygon, by the crossings of a ray to +u.
fn inside(point: [f64; 2], polygon: &[[f64; 2]]) -> bool {
    let n = polygon.len();
    let mut odd = false;
    for i in 0..n {
        let ([x0, y0], [x1, y1]) = (polygon[i], polygon[(i + 1) % n]);
        if (y0 > point[1]) != (y1 > point[1]) {
            let x = x0 + (point[1] - y0) / (y1 - y0) * (x1 - x0);
            if x > point[0] {
                odd = !odd;
            }
        }
    }
    odd
}

/// A number of a profile: the exposed field it is, its label in the dialog and its quantity;
/// `None` for a factor.
pub struct Dimension<'a> {
    pub field: Field,
    pub label: &'static str,
    pub quantity: Option<Quantity>,
    pub value: &'a mut f64,
}

impl BeamProfile {
    /// The profiles with dimensions of a millimetre, in SI units.
    pub const ALL: [BeamProfile; 10] = [
        BeamProfile::Rect { a: 1e-3, b: 1e-3 },
        BeamProfile::Circ { radius: 1e-3 },
        BeamProfile::Pipe {
            radius: 1e-3,
            thickness: 1e-4,
        },
        BeamProfile::Box {
            a: 1e-3,
            b: 1e-3,
            t: [1e-4; 4],
        },
        BeamProfile::I {
            h: 1e-3,
            b: 1e-3,
            tw: 1e-4,
            tf: 1e-4,
        },
        BeamProfile::T {
            h: 1e-3,
            b: 1e-3,
            tw: 1e-4,
            tf: 1e-4,
        },
        BeamProfile::U {
            h: 1e-3,
            b: 1e-3,
            tw: 1e-4,
            tf: 1e-4,
        },
        BeamProfile::L {
            h: 1e-3,
            b: 1e-3,
            t: 1e-4,
        },
        BeamProfile::General {
            area: 1e-6,
            i11: 1e-13,
            i22: 1e-13,
            k: 5.0 / 6.0,
        },
        BeamProfile::Sketch(Sketch { loops: Vec::new() }),
    ];

    /// Whether the solver has the profile as a beam section of its own; the others are written
    /// as general sections of `U1` elements.
    pub fn is_native(&self) -> bool {
        matches!(
            self,
            BeamProfile::Rect { .. }
                | BeamProfile::Circ { .. }
                | BeamProfile::Pipe { .. }
                | BeamProfile::Box { .. }
        )
    }

    /// Value of `SECTION=` in the input file.
    pub fn keyword(&self) -> &'static str {
        match self {
            BeamProfile::Rect { .. } => "RECT",
            BeamProfile::Circ { .. } => "CIRC",
            BeamProfile::Pipe { .. } => "PIPE",
            BeamProfile::Box { .. } => "BOX",
            _ => "GENERAL",
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            BeamProfile::Rect { .. } => "Rectangular",
            BeamProfile::Circ { .. } => "Circular",
            BeamProfile::Pipe { .. } => "Pipe",
            BeamProfile::Box { .. } => "Box",
            BeamProfile::I { .. } => "I-Beam",
            BeamProfile::T { .. } => "T-Beam",
            BeamProfile::U { .. } => "Channel (U)",
            BeamProfile::L { .. } => "Angle (L)",
            BeamProfile::General { .. } => "General (A, I)",
            BeamProfile::Sketch(_) => "Sketch",
        }
    }

    /// Pipes and boxes exist for `B32R` only in CalculiX.
    pub fn needs_reduced_integration(&self) -> bool {
        matches!(self, BeamProfile::Pipe { .. } | BeamProfile::Box { .. })
    }

    /// The dimensions as CalculiX 2.21 reads them on the first data line of `*BEAM
    /// SECTION`. A circle is given by its thickness in both directions, the diameter: a
    /// value of 4 puts the expanded nodes at radius 2. A pipe takes the radius itself.
    ///
    /// A general profile gives `A, I11, I12, I22, k` about its principal axes, the
    /// 1-direction turned by [`BeamProfile::principal_turn`]. CalculiX's `I11` is the integral
    /// of the coordinate in the 1-direction squared, so it stiffens bending in the
    /// 1-direction (checked with CalculiX 2.21 and 2.23 against a rectangle). `k` scales both
    /// the shear stiffness `k G A` and the torsion stiffness `k G (I11 + I22)`.
    pub fn data_line(&self) -> Vec<f64> {
        match *self {
            BeamProfile::Rect { a, b } => vec![a, b],
            BeamProfile::Circ { radius } => vec![2.0 * radius, 2.0 * radius],
            BeamProfile::Pipe { radius, thickness } => vec![radius, thickness],
            BeamProfile::Box { a, b, t } => vec![a, b, t[0], t[1], t[2], t[3]],
            _ => {
                let p = self.properties();
                let (sin, cos) = self.principal_turn().sin_cos();
                // Moments of u' = u cos + v sin and v' = v cos - u sin; i22 is that of u.
                let uu = p.i22 * cos * cos + p.i11 * sin * sin + 2.0 * p.i12 * sin * cos;
                let vv = p.i22 * sin * sin + p.i11 * cos * cos - 2.0 * p.i12 * sin * cos;
                vec![p.area, uu, 0.0, vv, self.shear_factor()]
            }
        }
    }

    /// The angle in radians the 1-direction of a general profile is turned by, towards the
    /// 2-direction, so that its product moment of area vanishes: the solver takes no `I12`.
    /// 0 for symmetric profiles.
    pub fn principal_turn(&self) -> f64 {
        let p = self.properties();
        // The product moment about axes turned by t: (i11 - i22) sin(2t) / 2 + i12 cos(2t).
        if p.i12.abs() <= 1e-12 * (p.i11 + p.i22) {
            return 0.0;
        }
        let mut turn = 0.5 * (-2.0 * p.i12).atan2(p.i11 - p.i22);
        // Either principal axis does; the nearer one keeps the profile's directions.
        let quarter = std::f64::consts::FRAC_PI_2;
        if turn > quarter / 2.0 {
            turn -= quarter;
        } else if turn < -quarter / 2.0 {
            turn += quarter;
        }
        turn
    }

    /// CalculiX's factor of the shear stiffness `k G A` of a general profile: the share of
    /// the area that carries shear along the 2-direction, the web of an I, T or U, a leg of
    /// an angle.
    pub fn shear_factor(&self) -> f64 {
        let area = self.properties().area;
        let share = |carrying: f64| (carrying / area).clamp(0.0, 1.0);
        match *self {
            BeamProfile::I { h, tw, tf, .. } | BeamProfile::U { h, tw, tf, .. } => {
                share((h - 2.0 * tf) * tw)
            }
            BeamProfile::T { h, tw, tf, .. } => share((h - tf) * tw),
            BeamProfile::L { h, t, .. } => share(h * t),
            BeamProfile::General { k, .. } => k,
            // A sketch may be anything; that of a solid section.
            BeamProfile::Rect { .. } | BeamProfile::Circ { .. } | BeamProfile::Sketch(_) => {
                5.0 / 6.0
            }
            BeamProfile::Pipe { .. } | BeamProfile::Box { .. } => 0.5,
        }
    }

    /// Whether the dimensions make a section the solver accepts.
    pub fn is_valid(&self) -> bool {
        let positive = |v: f64| v.is_finite() && v > 0.0;
        match *self {
            BeamProfile::Rect { a, b } => positive(a) && positive(b),
            BeamProfile::Circ { radius } => positive(radius),
            BeamProfile::Pipe { radius, thickness } => {
                positive(radius) && positive(thickness) && thickness < radius
            }
            BeamProfile::Box { a, b, t } => {
                positive(a)
                    && positive(b)
                    && t.iter().all(|&t| positive(t))
                    && t[0] + t[2] < a
                    && t[1] + t[3] < b
            }
            BeamProfile::I { h, b, tw, tf } | BeamProfile::U { h, b, tw, tf } => {
                [h, b, tw, tf].into_iter().all(positive) && 2.0 * tf < h && tw < b
            }
            BeamProfile::T { h, b, tw, tf } => {
                [h, b, tw, tf].into_iter().all(positive) && tf < h && tw <= b
            }
            BeamProfile::L { h, b, t } => [h, b, t].into_iter().all(positive) && t < h && t < b,
            BeamProfile::General { area, i11, i22, k } => {
                [area, i11, i22, k].into_iter().all(positive)
            }
            BeamProfile::Sketch(ref sketch) => sketch.problem().is_none(),
        }
    }

    /// Every number of the profile, in the order the dialog shows them.
    pub fn dimensions_mut(&mut self) -> Vec<Dimension<'_>> {
        let length = |field, label, value| Dimension {
            field,
            label,
            quantity: Some(Quantity::Length),
            value,
        };
        match self {
            BeamProfile::Rect { a, b } => vec![
                length(Field::BeamA, "Thickness in 1-direction (a)", a),
                length(Field::BeamB, "Thickness in 2-direction (b)", b),
            ],
            BeamProfile::Circ { radius } => vec![length(Field::Radius, "Radius", radius)],
            BeamProfile::Pipe { radius, thickness } => vec![
                length(Field::Radius, "Outer radius", radius),
                length(Field::Thickness, "Wall thickness", thickness),
            ],
            BeamProfile::Box { a, b, t } => {
                let [t1, t2, t3, t4] = t;
                vec![
                    length(Field::BeamA, "Width in 1-direction (a)", a),
                    length(Field::BeamB, "Width in 2-direction (b)", b),
                    length(Field::Wall1, "Wall thickness at +1 (t1)", t1),
                    length(Field::Wall2, "Wall thickness at +2 (t2)", t2),
                    length(Field::Wall3, "Wall thickness at -1 (t3)", t3),
                    length(Field::Wall4, "Wall thickness at -2 (t4)", t4),
                ]
            }
            BeamProfile::I { h, b, tw, tf }
            | BeamProfile::T { h, b, tw, tf }
            | BeamProfile::U { h, b, tw, tf } => vec![
                length(Field::Height, "Height in 2-direction (h)", h),
                length(Field::Width, "Flange width in 1-direction (b)", b),
                length(Field::WebThickness, "Web thickness (tw)", tw),
                length(Field::FlangeThickness, "Flange thickness (tf)", tf),
            ],
            BeamProfile::L { h, b, t } => vec![
                length(Field::Height, "Leg in 2-direction (h)", h),
                length(Field::Width, "Leg in 1-direction (b)", b),
                length(Field::Thickness, "Thickness (t)", t),
            ],
            BeamProfile::General { area, i11, i22, k } => vec![
                Dimension {
                    field: Field::Area,
                    label: "Area (A)",
                    quantity: Some(Quantity::Area),
                    value: area,
                },
                Dimension {
                    field: Field::I11,
                    label: "I11 (about the 1-axis)",
                    quantity: Some(Quantity::SecondMomentOfArea),
                    value: i11,
                },
                Dimension {
                    field: Field::I22,
                    label: "I22 (about the 2-axis)",
                    quantity: Some(Quantity::SecondMomentOfArea),
                    value: i22,
                },
                Dimension {
                    field: Field::ShearFactor,
                    label: "Shear and torsion factor (k)",
                    quantity: None,
                    value: k,
                },
            ],
            // Its points are edited in the sketch, not exposed one by one.
            BeamProfile::Sketch(_) => Vec::new(),
        }
    }

    pub fn convert_units(&mut self, c: &crate::convert::Conversion) {
        if let BeamProfile::Sketch(sketch) = self {
            for value in sketch.loops.iter_mut().flatten().flatten() {
                c.value(value, Quantity::Length);
            }
        }
        for dimension in self.dimensions_mut() {
            if let Some(quantity) = dimension.quantity {
                c.value(dimension.value, quantity);
            }
        }
    }

    /// Scales the profile by `length`: lengths by it, areas by its square and second moments
    /// of area by its fourth power.
    pub fn scale(&mut self, length: f64) {
        if let BeamProfile::Sketch(sketch) = self {
            for value in sketch.loops.iter_mut().flatten().flatten() {
                *value *= length;
            }
        }
        for dimension in self.dimensions_mut() {
            *dimension.value *= match dimension.quantity {
                Some(Quantity::Area) => length * length,
                Some(Quantity::SecondMomentOfArea) => length.powi(4),
                Some(_) => length,
                None => 1.0,
            };
        }
    }

    /// The thickness of the profile in the 1- and 2-direction, which the offsets of a beam
    /// section refer to.
    pub fn size(&self) -> [f64; 2] {
        match *self {
            BeamProfile::Rect { a, b } | BeamProfile::Box { a, b, .. } => [a, b],
            BeamProfile::Circ { radius } | BeamProfile::Pipe { radius, .. } => {
                [2.0 * radius, 2.0 * radius]
            }
            BeamProfile::I { h, b, .. }
            | BeamProfile::T { h, b, .. }
            | BeamProfile::U { h, b, .. }
            | BeamProfile::L { h, b, .. } => [b, h],
            BeamProfile::General { area, i11, i22, .. } => equivalent_rectangle(area, i11, i22),
            BeamProfile::Sketch(ref sketch) => {
                let points: Vec<[f64; 2]> = sketch.loops.first().cloned().unwrap_or_default();
                let (mut min, mut max) = ([f64::MAX; 2], [f64::MIN; 2]);
                for [u, v] in points {
                    min = [min[0].min(u), min[1].min(v)];
                    max = [max[0].max(u), max[1].max(v)];
                }
                if min[0] > max[0] {
                    return [0.0; 2];
                }
                [max[0] - min[0], max[1] - min[1]]
            }
        }
    }

    /// The outlines of the profile about its centre in the 1-2 plane, counter-clockwise: the
    /// outer one first, then the hole of a hollow profile. A general profile lies with its
    /// centroid at the centre; one given by its values alone is drawn as the rectangle of the
    /// same second moments.
    pub fn outlines(&self) -> Vec<Outline> {
        if let BeamProfile::Sketch(sketch) = self {
            let [cu, cv] = SectionProperties::of_loops(&sketch.loops).centroid;
            return (sketch.loops.iter())
                .map(|points| {
                    let mut points: Vec<[f64; 2]> =
                        points.iter().map(|&[u, v]| [u - cu, v - cv]).collect();
                    if signed_area(&points) < 0.0 {
                        points.reverse();
                    }
                    Outline {
                        points,
                        round: false,
                    }
                })
                .collect();
        }
        let circle = |r: f64| Outline {
            points: (0..ROUND_POINTS)
                .map(|k| {
                    let angle = std::f64::consts::TAU * k as f64 / ROUND_POINTS as f64;
                    [r * angle.cos(), r * angle.sin()]
                })
                .collect(),
            round: true,
        };
        let rect = |[u0, v0]: [f64; 2], [u1, v1]: [f64; 2]| Outline {
            points: vec![[u0, v0], [u1, v0], [u1, v1], [u0, v1]],
            round: false,
        };
        // A polygon moved so that its centroid lies at the centre.
        let centred = |points: Vec<[f64; 2]>| {
            let [cu, cv] = SectionProperties::of_loops(std::slice::from_ref(&points)).centroid;
            Outline {
                points: points.into_iter().map(|[u, v]| [u - cu, v - cv]).collect(),
                round: false,
            }
        };
        match *self {
            BeamProfile::Rect { a, b } => vec![rect([-a / 2.0, -b / 2.0], [a / 2.0, b / 2.0])],
            BeamProfile::Circ { radius } => vec![circle(radius)],
            BeamProfile::Pipe { radius, thickness } => {
                vec![circle(radius), circle(radius - thickness)]
            }
            // Walls at +1, +2, -1 and -2.
            BeamProfile::Box { a, b, t } => vec![
                rect([-a / 2.0, -b / 2.0], [a / 2.0, b / 2.0]),
                rect(
                    [-a / 2.0 + t[2], -b / 2.0 + t[3]],
                    [a / 2.0 - t[0], b / 2.0 - t[1]],
                ),
            ],
            BeamProfile::I { h, b, tw, tf } => {
                let (u, v, w) = (b / 2.0, h / 2.0, tw / 2.0);
                vec![centred(vec![
                    [-u, -v],
                    [u, -v],
                    [u, -v + tf],
                    [w, -v + tf],
                    [w, v - tf],
                    [u, v - tf],
                    [u, v],
                    [-u, v],
                    [-u, v - tf],
                    [-w, v - tf],
                    [-w, -v + tf],
                    [-u, -v + tf],
                ])]
            }
            BeamProfile::T { h, b, tw, tf } => {
                let (u, w) = (b / 2.0, tw / 2.0);
                vec![centred(vec![
                    [-w, 0.0],
                    [w, 0.0],
                    [w, h - tf],
                    [u, h - tf],
                    [u, h],
                    [-u, h],
                    [-u, h - tf],
                    [-w, h - tf],
                ])]
            }
            BeamProfile::U { h, b, tw, tf } => vec![centred(vec![
                [0.0, 0.0],
                [b, 0.0],
                [b, tf],
                [tw, tf],
                [tw, h - tf],
                [b, h - tf],
                [b, h],
                [0.0, h],
            ])],
            BeamProfile::L { h, b, t } => vec![centred(vec![
                [0.0, 0.0],
                [b, 0.0],
                [b, t],
                [t, t],
                [t, h],
                [0.0, h],
            ])],
            BeamProfile::General { area, i11, i22, .. } => {
                let [a, b] = equivalent_rectangle(area, i11, i22);
                vec![rect([-a / 2.0, -b / 2.0], [a / 2.0, b / 2.0])]
            }
            BeamProfile::Sketch(_) => unreachable!("returned above"),
        }
    }

    /// Area, centroid and second moments of area of the profile, exact for round ones too.
    pub fn properties(&self) -> SectionProperties {
        match *self {
            BeamProfile::Circ { radius } => SectionProperties::disc(radius, 0.0),
            BeamProfile::Pipe { radius, thickness } => {
                SectionProperties::disc(radius, radius - thickness)
            }
            BeamProfile::General { area, i11, i22, .. } => SectionProperties {
                area,
                centroid: [0.0; 2],
                i11,
                i22,
                i12: 0.0,
            },
            _ => {
                let loops: Vec<Vec<[f64; 2]>> =
                    self.outlines().into_iter().map(|o| o.points).collect();
                SectionProperties::of_loops(&loops)
            }
        }
    }
}

/// The sides in the 1- and 2-direction of the rectangle with the second moments `i22` and
/// `i11` for the area `area`: what a profile given by its values alone is drawn as.
fn equivalent_rectangle(area: f64, i11: f64, i22: f64) -> [f64; 2] {
    if area.is_nan() || area <= 0.0 {
        return [0.0; 2];
    }
    [(12.0 * i22 / area).sqrt(), (12.0 * i11 / area).sqrt()]
}

/// Triangles filling a simple polygon, by ear clipping: indices into `points`, counter-
/// clockwise. Concave outlines such as an I-beam's need it; a fan only fills convex ones.
pub fn triangulate(points: &[[f64; 2]]) -> Vec<[usize; 3]> {
    let mut left: Vec<usize> = (0..points.len()).collect();
    if signed_area(points) < 0.0 {
        left.reverse();
    }
    ear_clip(points, left)
}

/// Triangles filling the region inside the first loop and outside the others, indices into
/// the points of all loops one after the other, counter-clockwise. Each hole is joined to
/// the outline by a bridge to a vertex it sees, which turns the region into one polygon.
pub fn triangulate_with_holes(loops: &[Vec<[f64; 2]>]) -> Vec<[usize; 3]> {
    let Some(outer) = loops.first() else {
        return Vec::new();
    };
    let points: Vec<[f64; 2]> = loops.iter().flatten().copied().collect();
    let mut starts = vec![0];
    for points in loops {
        starts.push(starts.last().unwrap() + points.len());
    }
    // The outline counter-clockwise, the holes clockwise.
    let ring = |k: usize| -> Vec<usize> {
        let mut ring: Vec<usize> = (starts[k]..starts[k + 1]).collect();
        let area = signed_area(&loops[k]);
        if (k == 0) == (area < 0.0) {
            ring.reverse();
        }
        ring
    };
    let mut polygon = ring(0);
    if outer.len() < 3 {
        return Vec::new();
    }
    // The holes from right to left, each from its rightmost point, as the bridges of the
    // holes further right then cannot cross it.
    let mut holes: Vec<usize> = (1..loops.len()).filter(|&k| loops[k].len() >= 3).collect();
    let rightmost = |k: usize| {
        (starts[k]..starts[k + 1])
            .max_by(|&a, &b| points[a][0].total_cmp(&points[b][0]))
            .unwrap()
    };
    holes.sort_by(|&a, &b| points[rightmost(b)][0].total_cmp(&points[rightmost(a)][0]));
    for (done, &k) in holes.iter().enumerate() {
        let hole = ring(k);
        let m = rightmost(k);
        let from = points[m];
        // The edges a bridge must not cross: the polygon so far and the holes still to come.
        let mut edges: Vec<(usize, usize)> = (0..polygon.len())
            .map(|i| (polygon[i], polygon[(i + 1) % polygon.len()]))
            .collect();
        for &later in &holes[done..] {
            let ring = ring(later);
            edges.extend((0..ring.len()).map(|i| (ring[i], ring[(i + 1) % ring.len()])));
        }
        let distance = |i: usize| {
            let p = points[polygon[i]];
            (p[0] - from[0]).hypot(p[1] - from[1])
        };
        let visible = |i: usize| {
            let to = points[polygon[i]];
            // A point met twice, at both ends of an earlier bridge, takes the bridge on the
            // side of its copy that it leaves into.
            let n = polygon.len();
            let (next, prev) = (
                points[polygon[(i + 1) % n]],
                points[polygon[(i + n - 1) % n]],
            );
            if !inside_corner(to, next, prev, from) {
                return false;
            }
            !edges.iter().any(|&(a, b)| {
                let shared = [a, b].iter().any(|&e| points[e] == to || points[e] == from);
                !shared && segments_meet(from, to, points[a], points[b])
            })
        };
        let mut order: Vec<usize> = (0..polygon.len()).collect();
        order.sort_by(|&a, &b| distance(a).total_cmp(&distance(b)));
        let Some(i) = order.into_iter().find(|&i| visible(i)) else {
            continue;
        };
        // Into the hole at its rightmost point, around it and back along the bridge.
        let at = hole.iter().position(|&h| h == m).unwrap();
        let mut spliced: Vec<usize> = polygon[..=i].to_vec();
        spliced.extend(hole[at..].iter().chain(&hole[..=at]));
        spliced.push(polygon[i]);
        spliced.extend(&polygon[i + 1..]);
        polygon = spliced;
    }
    ear_clip(&points, polygon)
}

/// Whether `q` lies in the corner of a counter-clockwise polygon at `p`, which runs from the
/// edge to `next` counter-clockwise round to the edge to `prev`.
fn inside_corner(p: [f64; 2], next: [f64; 2], prev: [f64; 2], q: [f64; 2]) -> bool {
    let cross =
        |a: [f64; 2], b: [f64; 2]| (a[0] - p[0]) * (b[1] - p[1]) - (a[1] - p[1]) * (b[0] - p[0]);
    if cross(next, prev) >= 0.0 {
        cross(next, q) > 0.0 && cross(q, prev) > 0.0
    } else {
        !(cross(prev, q) >= 0.0 && cross(q, next) >= 0.0)
    }
}

/// Twice the area of a polygon, positive counter-clockwise.
fn signed_area(points: &[[f64; 2]]) -> f64 {
    let n = points.len();
    (0..n)
        .map(|i| {
            let ([x0, y0], [x1, y1]) = (points[i], points[(i + 1) % n]);
            x0 * y1 - x1 * y0
        })
        .sum()
}

/// Ear clipping of the counter-clockwise polygon through `left`, indices into `points`. A
/// point may appear twice, at both ends of a bridge to a hole.
fn ear_clip(points: &[[f64; 2]], mut left: Vec<usize>) -> Vec<[usize; 3]> {
    if left.len() < 3 {
        return Vec::new();
    }
    let cross = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let mut triangles = Vec::with_capacity(left.len() - 2);
    while left.len() > 3 {
        let m = left.len();
        let ear = (0..m).find(|&k| {
            let (a, b, c) = (left[(k + m - 1) % m], left[k], left[(k + 1) % m]);
            let (pa, pb, pc) = (points[a], points[b], points[c]);
            cross(pa, pb, pc) > 0.0
                && !(left.iter()).any(|&o| {
                    let po = points[o];
                    po != pa
                        && po != pb
                        && po != pc
                        && cross(pa, pb, po) >= 0.0
                        && cross(pb, pc, po) >= 0.0
                        && cross(pc, pa, po) >= 0.0
                })
        });
        // A degenerate polygon has no ear left: fill the rest as a fan.
        let k = ear.unwrap_or(0);
        let (a, b, c) = (left[(k + m - 1) % m], left[k], left[(k + 1) % m]);
        triangles.push([a, b, c]);
        left.remove(k);
    }
    triangles.push([left[0], left[1], left[2]]);
    triangles
}

/// An outline of a profile, closed; a round one is shaded smooth around.
#[derive(Clone, Debug, PartialEq)]
pub struct Outline {
    pub points: Vec<[f64; 2]>,
    pub round: bool,
}

/// Area and second moments of area of a profile, in the 1-2 plane: `centroid` about the
/// centre of the profile, the moments about axes through the centroid parallel to the 1- and
/// 2-direction. `i11` is the moment about the 1-axis (the integral of `v^2`), `i22` about the
/// 2-axis (`u^2`), `i12` the product moment (`u v`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SectionProperties {
    pub area: f64,
    pub centroid: [f64; 2],
    pub i11: f64,
    pub i22: f64,
    pub i12: f64,
}

impl SectionProperties {
    /// A full or hollow circle about its centre; `inner` 0 for a full one.
    fn disc(outer: f64, inner: f64) -> Self {
        let pi = std::f64::consts::PI;
        let i = pi / 4.0 * (outer.powi(4) - inner.powi(4));
        Self {
            area: pi * (outer * outer - inner * inner),
            centroid: [0.0; 2],
            i11: i,
            i22: i,
            i12: 0.0,
        }
    }

    /// A region bounded by closed polygons: the first counts as the outer boundary and the
    /// others as holes, whichever way they run.
    pub fn of_loops(loops: &[Vec<[f64; 2]>]) -> Self {
        // Integrals over the region about the origin, by Green's theorem per polygon.
        let mut sums = [0.0; 6];
        for (k, points) in loops.iter().enumerate() {
            let mut own = [0.0; 6];
            for (i, &[x0, y0]) in points.iter().enumerate() {
                let [x1, y1] = points[(i + 1) % points.len()];
                let cross = x0 * y1 - x1 * y0;
                own[0] += cross / 2.0;
                own[1] += cross * (x0 + x1) / 6.0;
                own[2] += cross * (y0 + y1) / 6.0;
                own[3] += cross * (y0 * y0 + y0 * y1 + y1 * y1) / 12.0;
                own[4] += cross * (x0 * x0 + x0 * x1 + x1 * x1) / 12.0;
                own[5] += cross * (x0 * y1 + 2.0 * x0 * y0 + 2.0 * x1 * y1 + x1 * y0) / 24.0;
            }
            // Counter-clockwise for the outer boundary, clockwise for holes.
            let sign = own[0].signum() * if k == 0 { 1.0 } else { -1.0 };
            for (sum, value) in sums.iter_mut().zip(own) {
                *sum += sign * value;
            }
        }
        let [area, su, sv, iuu_v, iuu_u, iuv] = sums;
        if area.abs() < f64::MIN_POSITIVE {
            return Self::default();
        }
        let centroid = [su / area, sv / area];
        Self {
            area,
            centroid,
            i11: iuu_v - area * centroid[1] * centroid[1],
            i22: iuu_u - area * centroid[0] * centroid[0],
            i12: iuv - area * centroid[0] * centroid[1],
        }
    }

    /// The principal moments of area, largest first, and the angle in radians from the
    /// 1-axis to the principal axis of the largest one, counter-clockwise.
    pub fn principal(&self) -> ([f64; 2], f64) {
        let mean = (self.i11 + self.i22) / 2.0;
        let radius = (((self.i11 - self.i22) / 2.0).powi(2) + self.i12 * self.i12).sqrt();
        let mut angle = if radius > 1e-12 * mean.abs().max(f64::MIN_POSITIVE) {
            0.5 * (-2.0 * self.i12).atan2(self.i11 - self.i22)
        } else {
            0.0
        };
        // An axis, not a direction: from above -90 up to 90 degrees.
        if angle <= -std::f64::consts::FRAC_PI_2 + 1e-12 {
            angle += std::f64::consts::PI;
        }
        ([mean + radius, mean - radius], angle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0)
    }

    #[test]
    fn rectangles_have_the_textbook_moments() {
        let p = BeamProfile::Rect { a: 20.0, b: 40.0 }.properties();
        assert!(close(p.area, 800.0));
        assert!(close(p.i11, 20.0 * 40.0_f64.powi(3) / 12.0));
        assert!(close(p.i22, 40.0 * 20.0_f64.powi(3) / 12.0));
        assert!(close(p.i12, 0.0));
        assert_eq!(p.centroid, [0.0, 0.0]);
        let ([big, small], angle) = p.principal();
        assert!(close(big, p.i11) && close(small, p.i22));
        assert!(close(angle, 0.0));
        // Turned a quarter, axis I runs along the 2-direction.
        let (_, angle) = BeamProfile::Rect { a: 40.0, b: 20.0 }
            .properties()
            .principal();
        assert!(close(angle, std::f64::consts::FRAC_PI_2));
    }

    #[test]
    fn an_unequal_box_moves_its_centroid_towards_the_thicker_wall() {
        // A 10 by 10 box, wall 3 at +1 and 1 elsewhere: the hole lies towards -1.
        let p = BeamProfile::Box {
            a: 10.0,
            b: 10.0,
            t: [3.0, 1.0, 1.0, 1.0],
        }
        .properties();
        // Outer 100 minus a hole of 6 by 8 centred at u = -1.
        assert!(close(p.area, 100.0 - 48.0));
        assert!(close(p.centroid[0], 48.0 / 52.0));
        assert!(close(p.centroid[1], 0.0));
        let i22_origin = 10.0 * 1000.0 / 12.0 - (8.0 * 216.0 / 12.0 + 48.0);
        assert!(close(p.i22, i22_origin - 52.0 * p.centroid[0].powi(2)));
        assert!(close(p.i11, 10.0 * 1000.0 / 12.0 - 6.0 * 512.0 / 12.0));
    }

    #[test]
    fn pipes_subtract_the_hole() {
        let p = BeamProfile::Pipe {
            radius: 2.0,
            thickness: 1.0,
        }
        .properties();
        let pi = std::f64::consts::PI;
        assert!(close(p.area, 3.0 * pi));
        assert!(close(p.i11, pi / 4.0 * 15.0));
    }

    #[test]
    fn i_beams_have_the_textbook_moments() {
        let (h, b, tw, tf) = (200.0, 100.0, 5.6, 8.5);
        let profile = BeamProfile::I { h, b, tw, tf };
        let p = profile.properties();
        let web = h - 2.0 * tf;
        assert!(close(p.area, 2.0 * b * tf + web * tw));
        assert!(close(
            p.i11,
            (b * h.powi(3) - (b - tw) * web.powi(3)) / 12.0
        ));
        assert!(close(
            p.i22,
            (2.0 * tf * b.powi(3) + web * tw.powi(3)) / 12.0
        ));
        assert!(p.centroid[0].abs() < 1e-9 && p.centroid[1].abs() < 1e-9);
        assert!(close(profile.principal_turn(), 0.0));
        // CalculiX's I11 stiffens bending in the 1-direction: the moment of u squared.
        let line = profile.data_line();
        assert!(close(line[1], p.i22) && close(line[3], p.i11) && line[2] == 0.0);
        assert!(close(line[4], web * tw / p.area));
        assert_eq!(profile.size(), [b, h]);
        assert_eq!(profile.keyword(), "GENERAL");
        assert!(!profile.is_native() && profile.is_valid());
    }

    #[test]
    fn general_profiles_lie_with_their_centroid_on_the_nodes() {
        for profile in [
            BeamProfile::T {
                h: 50.0,
                b: 40.0,
                tw: 5.0,
                tf: 6.0,
            },
            BeamProfile::U {
                h: 100.0,
                b: 50.0,
                tw: 6.0,
                tf: 8.5,
            },
            BeamProfile::L {
                h: 80.0,
                b: 40.0,
                t: 6.0,
            },
        ] {
            let p = profile.properties();
            assert!(p.centroid[0].abs() < 1e-9, "{profile:?}");
            assert!(p.centroid[1].abs() < 1e-9, "{profile:?}");
            // Filled by triangles of the same area: the outlines are concave.
            let points = &profile.outlines()[0].points;
            let filled: f64 = (triangulate(points).iter())
                .map(|&[a, b, c]| {
                    let ([xa, ya], [xb, yb], [xc, yc]) = (points[a], points[b], points[c]);
                    ((xb - xa) * (yc - ya) - (yb - ya) * (xc - xa)) / 2.0
                })
                .sum();
            assert!(close(filled, p.area), "{profile:?}: {filled}");
        }
    }

    #[test]
    fn an_unequal_angle_is_written_about_its_principal_axes() {
        let profile = BeamProfile::L {
            h: 80.0,
            b: 40.0,
            t: 6.0,
        };
        let p = profile.properties();
        assert!(p.i12.abs() > 1.0);
        let turn = profile.principal_turn();
        assert!(turn.abs() > 0.1 && turn.abs() <= std::f64::consts::FRAC_PI_4);
        let ([big, small], _) = p.principal();
        let line = profile.data_line();
        // The moments about the turned axes are the principal ones.
        let mut written = [line[1], line[3]];
        written.sort_by(f64::total_cmp);
        assert!(
            close(written[0], small) && close(written[1], big),
            "{line:?}"
        );
    }

    #[test]
    fn general_values_scale_with_their_powers_of_length() {
        let mut profile = BeamProfile::General {
            area: 2.0,
            i11: 3.0,
            i22: 4.0,
            k: 0.5,
        };
        profile.scale(10.0);
        assert_eq!(
            profile,
            BeamProfile::General {
                area: 200.0,
                i11: 30000.0,
                i22: 40000.0,
                k: 0.5,
            }
        );
        // Drawn as the rectangle of the same moments.
        let [a, b] = profile.size();
        assert!(close(a * a * a * b / 12.0, 40000.0 * a * b / 200.0));
        assert_eq!(profile.data_line(), vec![200.0, 40000.0, 0.0, 30000.0, 0.5]);
    }

    #[test]
    fn a_tilted_angle_has_its_principal_axis_on_the_diagonal() {
        // An equal-legged angle of 10 by 10 with legs 1 thick: principal axes at 45 degrees.
        let outline = vec![
            [0.0, 0.0],
            [10.0, 0.0],
            [10.0, 1.0],
            [1.0, 1.0],
            [1.0, 10.0],
            [0.0, 10.0],
        ];
        let p = SectionProperties::of_loops(&[outline]);
        assert!(close(p.area, 19.0));
        assert!(p.i12 != 0.0);
        let ([big, small], angle) = p.principal();
        assert!(big > small);
        assert!(close(angle.abs(), std::f64::consts::FRAC_PI_4));
        // The sum of the moments does not depend on the axes.
        assert!(close(big + small, p.i11 + p.i22));
    }

    /// A square of side 10 with two square holes of side 2, the second one clockwise.
    fn holed_square() -> Sketch {
        Sketch {
            loops: vec![
                vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]],
                vec![[2.0, 2.0], [4.0, 2.0], [4.0, 4.0], [2.0, 4.0]],
                vec![[6.0, 8.0], [8.0, 8.0], [8.0, 6.0], [6.0, 6.0]],
            ],
        }
    }

    #[test]
    fn sketches_subtract_their_holes() {
        let sketch = holed_square();
        assert_eq!(sketch.problem(), None);
        let profile = BeamProfile::Sketch(sketch.clone());
        let p = profile.properties();
        assert!(close(p.area, 92.0));
        // The holes lie symmetric about the diagonal u = v.
        assert!(close(p.i11, p.i22));
        assert_eq!(profile.size(), [10.0, 10.0]);
        // The outlines lie about the centroid, all counter-clockwise.
        let outlines = profile.outlines();
        assert_eq!(outlines.len(), 3);
        for outline in &outlines {
            assert!(signed_area(&outline.points) > 0.0);
        }
        let moved = SectionProperties::of_loops(
            &outlines
                .iter()
                .map(|o| o.points.clone())
                .collect::<Vec<_>>(),
        );
        assert!(moved.centroid.iter().all(|c| c.abs() < 1e-12));
        assert!(close(moved.i11, p.i11));
    }

    #[test]
    fn holes_are_left_out_of_the_triangles() {
        let sketch = holed_square();
        let points: Vec<[f64; 2]> = sketch.loops.iter().flatten().copied().collect();
        let triangles = triangulate_with_holes(&sketch.loops);
        let mut area = 0.0;
        for t in &triangles {
            let [a, b, c] = t.map(|k| points[k]);
            let twice = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
            assert!(twice > -1e-12, "clockwise triangle {t:?}");
            area += twice / 2.0;
        }
        assert!(close(area, 92.0), "{area}");
        // No triangle covers the middle of a hole.
        let covers = |[x, y]: [f64; 2]| {
            triangles.iter().any(|t| {
                let [a, b, c] = t.map(|k| points[k]);
                let side = |p: [f64; 2], q: [f64; 2]| {
                    (q[0] - p[0]) * (y - p[1]) - (q[1] - p[1]) * (x - p[0]) > 0.0
                };
                side(a, b) && side(b, c) && side(c, a)
            })
        };
        assert!(!covers([3.0, 3.0]) && !covers([7.0, 7.0]));
        assert!(covers([5.3, 1.1]) && covers([1.2, 8.7]));
    }

    #[test]
    fn sketches_tell_what_is_wrong() {
        let mut sketch = holed_square();
        sketch.loops[1] = vec![[-2.0, 2.0], [-1.0, 2.0], [-1.0, 3.0]];
        assert!(sketch.problem().unwrap().contains("Hole 1"), "{sketch:?}");
        let crossing = Sketch {
            loops: vec![vec![[0.0, 0.0], [1.0, 1.0], [1.0, 0.0], [0.0, 1.0]]],
        };
        assert!(crossing.problem().is_some());
        assert!(!BeamProfile::Sketch(crossing).is_valid());
        // A sketch of a profile is that profile.
        let i = BeamProfile::I {
            h: 0.2,
            b: 0.1,
            tw: 0.01,
            tf: 0.02,
        };
        let sketched = BeamProfile::Sketch(Sketch::of(&i)).properties();
        assert!(close(sketched.area, i.properties().area));
        assert!(close(sketched.i11, i.properties().i11));
    }

    #[test]
    fn holes_beside_each_other_are_both_left_out() {
        // A trapezoid tube with a second hole in its wall, both holes reaching as far right.
        let loops = vec![
            vec![[-60.0, 0.0], [60.0, 0.0], [40.0, 160.0], [-40.0, 160.0]],
            vec![[-40.0, 15.0], [40.0, 15.0], [25.0, 135.0], [-25.0, 135.0]],
            vec![[30.0, 130.0], [30.0, 140.0], [40.0, 140.0], [40.0, 130.0]],
        ];
        let points: Vec<[f64; 2]> = loops.iter().flatten().copied().collect();
        let mut area = 0.0;
        for t in triangulate_with_holes(&loops) {
            let [a, b, c] = t.map(|k| points[k]);
            let twice = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
            assert!(twice > -1e-9, "clockwise triangle {t:?}");
            area += twice / 2.0;
        }
        let expected = SectionProperties::of_loops(&loops).area;
        assert!(close(area, expected), "{area} != {expected}");
    }
}
