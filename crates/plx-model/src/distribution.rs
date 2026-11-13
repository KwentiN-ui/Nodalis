//! Distributions: factors that vary over the model. A pressure, a
//! prescribed temperature, an initial temperature or a defined temperature refers to one by
//! name; the input file then holds the item's value times the factor at every element face
//! or node instead of one value for the whole region.

use serde::{Deserialize, Serialize};

use crate::coordinate_system::{CoordinateSystem, Frame};
use crate::equation::Equation;
use crate::units::{LengthUnit, Quantity, UnitSystem};

/// A factor over the model, given by an equation or by points with a value each.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Distribution {
    /// The internal identity other items refer to; the user sees only the name.
    pub id: crate::Id,
    pub name: String,
    pub kind: DistributionKind,
    /// The coordinate system the equation takes its coordinates in and the points are given
    /// in; `None` for the global one. A moving system counts where it is in the undeformed
    /// model.
    #[serde(default)]
    pub system: Option<crate::Id>,
    /// Named numbers the equation can use, each of which can be exposed as a parameter.
    #[serde(default)]
    pub variables: Vec<DistributionVariable>,
    /// The unit the equation takes its coordinates in and the points file gives them in. It
    /// belongs to the distribution, so that changing the project's units does not change the
    /// factors.
    #[serde(default)]
    pub length: LengthUnit,
}

/// A named number of a distribution's equation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DistributionVariable {
    /// Stays the same when the variable is renamed, for the exposed parameter of it.
    pub key: u32,
    pub name: String,
    pub value: f64,
}

impl DistributionVariable {
    /// A variable of the value 1 to add to `variables`, with a name and key they do not
    /// have: p1, p2, ...
    pub fn next(variables: &[Self]) -> Self {
        let key = variables.iter().map(|v| v.key + 1).max().unwrap_or(1);
        let name = (variables.len() + 1..)
            .map(|n| format!("p{n}"))
            .find(|name| !variables.iter().any(|v| &v.name == name))
            .expect("a free name");
        Self {
            key,
            name,
            value: 1.0,
        }
    }
}

/// The coordinates an equation can use besides the variables: along the axes of the
/// coordinate system, and the radius and angle (radians) about its z axis.
pub const COORDINATES: [&str; 5] = ["x", "y", "z", "r", "theta"];

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum DistributionKind {
    /// An equation in the [`COORDINATES`] of the distribution's coordinate system, which it
    /// takes in the length unit of the project, and its variables, e.g. `1 - z/h`; a leading
    /// `=` is allowed.
    Equation(String),
    /// Values at points, as read from a text file of `X Y Z value` rows; the factor between
    /// the points is interpolated.
    Points {
        /// Name of the file the points came from, shown to the user; the points are kept in
        /// the model, so the file is no longer needed.
        source: String,
        /// Position (stored in meters) and value of every point.
        points: Vec<[f64; 4]>,
        interpolation: Interpolation,
    },
}

/// How the factor between the points of a [`DistributionKind::Points`] is found.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Interpolation {
    /// The value of the closest point.
    #[default]
    ClosestPoint,
    /// Shepard's inverse distance weighting of the closest points: smooth between the
    /// points and exact at them.
    Shepard,
}

impl Interpolation {
    pub const ALL: [Interpolation; 2] = [Interpolation::ClosestPoint, Interpolation::Shepard];

    pub fn label(self) -> &'static str {
        match self {
            Interpolation::ClosestPoint => "Closest point",
            Interpolation::Shepard => "Shepard (inverse distance)",
        }
    }
}

/// Points Shepard's interpolation weighs.
const SHEPARD_POINTS: usize = 8;

impl Distribution {
    /// A distribution with the factor 1 everywhere.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            kind: DistributionKind::Equation("1".into()),
            system: None,
            variables: Vec::new(),
            length: LengthUnit::default(),
        }
    }

    /// Adds a variable of the value 1 with a free name and key.
    pub fn add_variable(&mut self) -> &mut DistributionVariable {
        let variable = DistributionVariable::next(&self.variables);
        self.variables.push(variable);
        self.variables.last_mut().expect("just pushed")
    }

    pub fn variable(&self, key: u32) -> Option<&DistributionVariable> {
        self.variables.iter().find(|v| v.key == key)
    }

    /// Why the names of the variables cannot be used, if they cannot.
    pub fn variables_problem(&self) -> Option<String> {
        for (i, variable) in self.variables.iter().enumerate() {
            let name = &variable.name;
            let mut chars = name.chars();
            let valid = chars
                .next()
                .is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
                && chars.all(|c| c.is_ascii_alphanumeric() || c == '_');
            if !valid {
                return Some(format!(
                    "The variable name '{name}' is not valid: use letters, digits and _, \
                     starting with a letter."
                ));
            }
            let lower = name.to_ascii_lowercase();
            if COORDINATES.contains(&lower.as_str())
                || ["pi", "true", "false"].contains(&lower.as_str())
            {
                return Some(format!(
                    "The name '{name}' is taken by the equation itself."
                ));
            }
            if self.variables[..i].iter().any(|v| &v.name == name) {
                return Some(format!("There are two variables named '{name}'."));
            }
            if !variable.value.is_finite() {
                return Some(format!("The variable '{name}' is not a number."));
            }
        }
        None
    }

    /// The factor at positions given in meters; `systems` are the coordinate systems of the
    /// model. The equation takes its coordinates in [`Self::length`]. Fails when the
    /// equation is invalid, there are no points or the coordinate system is missing.
    pub fn sampler(&self, systems: &[CoordinateSystem]) -> Result<Sampler, String> {
        let frame = match self.system {
            None => Frame::global_system(),
            Some(id) => {
                let system = (systems.iter().find(|s| s.id == id))
                    .ok_or("The coordinate system of the distribution does not exist.")?;
                let frame = system.frame(systems);
                frame
                    .axes()
                    .map_err(|e| format!("Coordinate system {}: {e}", system.name))?;
                frame
            }
        };
        match &self.kind {
            DistributionKind::Equation(text) => {
                if let Some(problem) = self.variables_problem() {
                    return Err(problem);
                }
                let equation = Equation::parse(text)?;
                let inputs = (equation.variables().iter())
                    .map(|v| {
                        if let Some(variable) = self.variables.iter().find(|w| &w.name == v) {
                            return Ok(Input::Constant(variable.value));
                        }
                        let lower = v.to_ascii_lowercase();
                        match COORDINATES.iter().position(|c| *c == lower) {
                            Some(i) => Ok(Input::Coordinate(i)),
                            None => {
                                let mut names: Vec<&str> = COORDINATES.to_vec();
                                names.extend(self.variables.iter().map(|v| v.name.as_str()));
                                Err(format!(
                                    "Unknown variable '{v}'; the equation can use {}.",
                                    names.join(", ")
                                ))
                            }
                        }
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                Ok(Sampler::Equation {
                    equation,
                    inputs,
                    length: self.length,
                    frame,
                })
            }
            DistributionKind::Points {
                points,
                interpolation,
                ..
            } => {
                if points.is_empty() {
                    return Err("The distribution has no points.".into());
                }
                if points.iter().flatten().any(|v| !v.is_finite()) {
                    return Err("The points contain invalid numbers.".into());
                }
                Ok(Sampler::Points {
                    tree: KdTree::new(points),
                    interpolation: *interpolation,
                    frame,
                })
            }
        }
    }

    /// Why the distribution cannot be evaluated with the coordinate systems `systems`, if
    /// it cannot.
    pub fn problem(&self, systems: &[CoordinateSystem]) -> Option<String> {
        self.sampler(systems).err()
    }
}

/// What a name of an equation stands for.
#[derive(Clone, Copy, Debug)]
pub enum Input {
    /// One of [`COORDINATES`] by its place.
    Coordinate(usize),
    /// The value of a variable.
    Constant(f64),
}

/// A distribution ready to give its factor at positions.
#[derive(Clone, Debug)]
pub enum Sampler {
    Equation {
        equation: Equation,
        /// What each variable of the equation stands for.
        inputs: Vec<Input>,
        /// The unit of the coordinates.
        length: LengthUnit,
        /// The coordinate system the coordinates are taken in.
        frame: Frame,
    },
    Points {
        /// The points with their positions along the axes of `frame`.
        tree: KdTree,
        interpolation: Interpolation,
        frame: Frame,
    },
}

impl Sampler {
    /// The factor at a position in meters.
    pub fn factor(&self, position: [f64; 3]) -> f64 {
        match self {
            Sampler::Equation {
                equation,
                inputs,
                length: unit,
                frame,
            } => {
                let local = frame.cartesian_local(position).unwrap_or([f64::NAN; 3]);
                let length = |v: f64| unit.of_meters(v);
                let values: Vec<f64> = (inputs.iter())
                    .map(|input| match *input {
                        Input::Constant(value) => value,
                        Input::Coordinate(i @ 0..=2) => length(local[i]),
                        Input::Coordinate(3) => length(local[0].hypot(local[1])),
                        Input::Coordinate(_) => local[1].atan2(local[0]),
                    })
                    .collect();
                equation.evaluate(&values)
            }
            Sampler::Points {
                tree,
                interpolation,
                frame,
            } => {
                let position = frame.cartesian_local(position).unwrap_or([f64::NAN; 3]);
                match interpolation {
                    Interpolation::ClosestPoint => tree
                        .nearest(position, 1)
                        .first()
                        .map_or(0.0, |&(_, value)| value),
                    Interpolation::Shepard => {
                        let nearest = tree.nearest(position, SHEPARD_POINTS);
                        if let Some(&(_, value)) = nearest.iter().find(|(d2, _)| *d2 == 0.0) {
                            return value;
                        }
                        let (sum, weights) = (nearest.iter())
                            .fold((0.0, 0.0), |(s, w), &(d2, v)| (s + v / d2, w + 1.0 / d2));
                        if weights > 0.0 { sum / weights } else { 0.0 }
                    }
                }
            }
        }
    }

    /// `value` of a quantity scaled by the factor at `position`. Temperatures are scaled
    /// in the units the user sees them in, so that 100 °F times 0.5 stays 50 °F.
    pub fn scale(
        &self,
        value: f64,
        quantity: Quantity,
        units: UnitSystem,
        position: [f64; 3],
    ) -> f64 {
        let factor = self.factor(position);
        if quantity == Quantity::Temperature {
            units.stored(units.shown(value, quantity) * factor, quantity)
        } else {
            value * factor
        }
    }
}

/// Points sorted into a k-d tree for finding the closest ones fast.
#[derive(Clone, Debug)]
pub struct KdTree {
    /// Points in tree order: the median of each range splits it along the range's axis.
    points: Vec<[f64; 4]>,
}

impl KdTree {
    fn new(points: &[[f64; 4]]) -> Self {
        let mut points = points.to_vec();
        Self::build(&mut points, 0);
        Self { points }
    }

    fn build(points: &mut [[f64; 4]], depth: usize) {
        if points.len() <= 1 {
            return;
        }
        let axis = depth % 3;
        let mid = points.len() / 2;
        points.select_nth_unstable_by(mid, |a, b| a[axis].total_cmp(&b[axis]));
        let (left, right) = points.split_at_mut(mid);
        Self::build(left, depth + 1);
        Self::build(&mut right[1..], depth + 1);
    }

    /// Squared distance and value of the `count` closest points, closest first.
    fn nearest(&self, position: [f64; 3], count: usize) -> Vec<(f64, f64)> {
        let mut best = Vec::with_capacity(count + 1);
        self.search(&self.points, 0, position, count, &mut best);
        best
    }

    fn search(
        &self,
        points: &[[f64; 4]],
        depth: usize,
        position: [f64; 3],
        count: usize,
        best: &mut Vec<(f64, f64)>,
    ) {
        if points.is_empty() {
            return;
        }
        let mid = points.len() / 2;
        let point = points[mid];
        let d2: f64 = (0..3).map(|k| (point[k] - position[k]).powi(2)).sum();
        if best.len() < count || d2 < best[best.len() - 1].0 {
            let at = best.partition_point(|(d, _)| *d <= d2);
            best.insert(at, (d2, point[3]));
            best.truncate(count);
        }
        let axis = depth % 3;
        let offset = position[axis] - point[axis];
        let (near, far) = if offset < 0.0 {
            (&points[..mid], &points[mid + 1..])
        } else {
            (&points[mid + 1..], &points[..mid])
        };
        self.search(near, depth + 1, position, count, best);
        if best.len() < count || offset * offset < best[best.len() - 1].0 {
            self.search(far, depth + 1, position, count, best);
        }
    }
}

/// Reads points from text with an `X Y Z value` row per point, the coordinates in
/// `length`. Columns may be separated by blanks, tabs, commas or semicolons;
/// lines starting with `#` and lines that hold no four numbers, like a header, are skipped.
/// Returns the points with their positions in meters, or why there are none.
pub fn parse_points(text: &str, unit: LengthUnit) -> Result<Vec<[f64; 4]>, String> {
    let mut points = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let numbers: Option<Vec<f64>> = (line
            .split(|c: char| c.is_whitespace() || c == ',' || c == ';'))
        .filter(|s| !s.is_empty())
        .map(|s| s.parse::<f64>().ok().filter(|v| v.is_finite()))
        .collect();
        let Some(numbers) = numbers.filter(|n| n.len() >= 4) else {
            continue;
        };
        let length = |v: f64| unit.to_meters(v);
        points.push([
            length(numbers[0]),
            length(numbers[1]),
            length(numbers[2]),
            numbers[3],
        ]);
    }
    if points.is_empty() {
        return Err("The file holds no rows of four numbers X Y Z value.".into());
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn points(points: Vec<[f64; 4]>, interpolation: Interpolation) -> Distribution {
        Distribution {
            id: crate::Id::new(),
            name: "D".into(),
            kind: DistributionKind::Points {
                source: "points.txt".into(),
                points,
                interpolation,
            },
            system: None,
            variables: Vec::new(),
            length: LengthUnit::Millimeter,
        }
    }

    #[test]
    fn equation_takes_coordinates_in_the_length_unit_of_the_distribution() {
        let mut distribution = Distribution::new("Depth");
        distribution.kind = DistributionKind::Equation("=1 - z/200 + 0*X".into());
        let sampler = distribution.sampler(&[]).unwrap();
        // 0.1 m = 100 mm
        assert!((sampler.factor([0.0, 0.0, 0.1]) - 0.5).abs() < 1e-12);
        distribution.length = LengthUnit::Meter;
        let sampler = distribution.sampler(&[]).unwrap();
        assert!((sampler.factor([0.0, 0.0, 100.0]) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn equation_with_other_variables_is_refused() {
        let mut distribution = Distribution::new("D");
        distribution.kind = DistributionKind::Equation("=x + STRESS.MISES".into());
        assert!(distribution.problem(&[]).unwrap().contains("STRESS.MISES"));
        distribution.kind = DistributionKind::Equation("=x +".into());
        assert!(distribution.problem(&[]).is_some());
        assert!(Distribution::new("D").problem(&[]).is_none());
    }

    #[test]
    fn closest_point_matches_a_brute_force_search() {
        let cloud: Vec<[f64; 4]> = (0..500)
            .map(|i| {
                let t = i as f64;
                [(t * 0.37).sin(), (t * 0.73).cos(), (t * 1.31).sin(), t]
            })
            .collect();
        let distribution = points(cloud.clone(), Interpolation::ClosestPoint);
        let sampler = distribution.sampler(&[]).unwrap();
        for i in 0..200 {
            let t = i as f64 * 0.11;
            let p = [t.cos() * 0.9, (t * 1.7).sin(), (t * 0.3).cos()];
            let d2 = |q: &[f64; 4]| (0..3).map(|k| (q[k] - p[k]).powi(2)).sum::<f64>();
            let expected = cloud.iter().min_by(|a, b| d2(a).total_cmp(&d2(b))).unwrap()[3];
            assert_eq!(sampler.factor(p), expected);
        }
    }

    #[test]
    fn shepard_is_exact_at_points_and_between_them_in_range() {
        let distribution = points(
            vec![[0.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 3.0]],
            Interpolation::Shepard,
        );
        let sampler = distribution.sampler(&[]).unwrap();
        assert_eq!(sampler.factor([1.0, 0.0, 0.0]), 3.0);
        assert!((sampler.factor([0.5, 0.0, 0.0]) - 2.0).abs() < 1e-12);
        let quarter = sampler.factor([0.25, 0.0, 0.0]);
        assert!(quarter > 1.0 && quarter < 2.0);
    }

    #[test]
    fn temperatures_scale_in_the_units_shown() {
        let sampler = Distribution::new("Half");
        let mut sampler = sampler;
        sampler.kind = DistributionKind::Equation("=0.5".into());
        let sampler = sampler.sampler(&[]).unwrap();
        let units = UnitSystem::InLbSF;
        let stored = units.stored(100.0, Quantity::Temperature);
        let scaled = sampler.scale(stored, Quantity::Temperature, units, [0.0; 3]);
        assert!((units.shown(scaled, Quantity::Temperature) - 50.0).abs() < 1e-9);
        assert_eq!(sampler.scale(4.0, Quantity::Pressure, units, [0.0; 3]), 2.0);
    }

    /// A cylindrical system at (0.1, 0, 0) m whose z axis is the global x axis.
    fn cylinder() -> CoordinateSystem {
        CoordinateSystem::from_points("Axis", [0.1, 0.0, 0.0], [0.1, 1.0, 0.0], [0.1, 0.0, 1.0])
    }

    #[test]
    fn equation_takes_coordinates_in_its_coordinate_system() {
        let mut system = cylinder();
        system.kind = crate::CoordinateSystemKind::Cylindrical;
        let mut distribution = Distribution::new("Ring");
        distribution.system = Some(system.id);
        distribution.kind = DistributionKind::Equation("=r + 1000*z + 0*x".into());
        let systems = [system];
        let sampler = distribution.sampler(&systems).unwrap();
        // 30 mm from the axis, 20 mm along it
        let f = sampler.factor([0.12, 0.0, 0.03]);
        assert!((f - (30.0 + 1000.0 * 20.0)).abs() < 1e-9, "{f}");
        // Local y is global z: theta is 90 degrees there.
        distribution.kind = DistributionKind::Equation("=theta".into());
        let sampler = distribution.sampler(&systems).unwrap();
        let theta = sampler.factor([0.5, 0.0, 0.03]);
        assert!(
            (theta - std::f64::consts::FRAC_PI_2).abs() < 1e-12,
            "{theta}"
        );
        // Points are along the axes of the system.
        distribution.kind = DistributionKind::Points {
            source: String::new(),
            points: vec![[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.05, 2.0]],
            interpolation: Interpolation::ClosestPoint,
        };
        let sampler = distribution.sampler(&systems).unwrap();
        assert_eq!(sampler.factor([0.15, 0.0, 0.0]), 2.0);
        assert_eq!(sampler.factor([0.1, 0.01, 0.0]), 1.0);
        // A missing system cannot be evaluated.
        assert!(
            distribution
                .problem(&[])
                .unwrap()
                .contains("coordinate system")
        );
    }

    #[test]
    fn equation_uses_named_variables() {
        let mut distribution = Distribution::new("Wedge");
        distribution.add_variable().value = 4.0;
        distribution.add_variable().name = "h".into();
        distribution.variables[1].value = 200.0;
        assert_eq!(distribution.variables[0].name, "p1");
        distribution.kind = DistributionKind::Equation("=p1 * (1 - z/h)".into());
        let sampler = distribution.sampler(&[]).unwrap();
        assert!((sampler.factor([0.0, 0.0, 0.1]) - 2.0).abs() < 1e-12);
        for bad in ["theta", "2h", "p1", "", "Pi"] {
            distribution.variables[1].name = bad.into();
            assert!(distribution.problem(&[]).is_some(), "{bad}");
        }
        distribution.variables[1].name = "h".into();
        distribution.kind = DistributionKind::Equation("=p1 * q".into());
        let problem = distribution.problem(&[]).unwrap();
        assert!(
            problem.contains("'q'") && problem.contains("p1, h"),
            "{problem}"
        );
    }

    #[test]
    fn variables_can_be_exposed_and_set() {
        use crate::{Field, ItemRef, ValuePath};
        let mut fe = crate::FeModel::default();
        let mut distribution = Distribution::new("Wedge");
        distribution.add_variable();
        let key = distribution.add_variable().key;
        distribution.variables[1].name = "h".into();
        fe.distributions.push(distribution);
        let item = ItemRef::Distribution("Wedge".into());
        assert_eq!(
            fe.exposable(&item),
            [Field::Variable(1), Field::Variable(key)]
        );
        let path = ValuePath::new(item.clone(), Field::Variable(key));
        let exposed = fe.expose(path.clone());
        assert_eq!(fe.exposed(exposed).unwrap().name, "Wedge h");
        assert!(fe.set_value(&path, 250.0));
        assert_eq!(fe.distributions[0].variables[1].value, 250.0);
        assert_eq!(fe.value_at(&path), Some(250.0));
        // Renaming the variable keeps the parameter; renaming the distribution moves it.
        fe.distributions[0].variables[1].name = "height".into();
        assert_eq!(fe.value_at(&path), Some(250.0));
        fe.distributions[0].name = "Ramp".into();
        fe.rename_exposed(&item, "Ramp");
        assert_eq!(fe.value_at(&fe.exposed(exposed).unwrap().path), Some(250.0));
    }

    #[test]
    fn reads_points_in_the_length_unit_of_the_analysis() {
        let text = "# pressure map\nX Y Z P\n0 0 0 1\n1000,0,0,2\n\n10;20;30;3.5\n";
        let points = parse_points(text, LengthUnit::Millimeter).unwrap();
        assert_eq!(
            points,
            [
                [0.0, 0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0, 2.0],
                [0.01, 0.02, 0.03, 3.5]
            ]
        );
        assert!(parse_points("Hello\n1 2 3", LengthUnit::Millimeter).is_err());
    }
}
