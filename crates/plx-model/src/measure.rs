//! A key figure taken from the results of a step, as a Measure block of the project schematic
//! computes it: the von Mises stresses of a region, the displacement of a node, a total
//! force, an eigenfrequency. A parameter study reads it.

use serde::{Deserialize, Serialize};

/// The nodes a field measure looks at.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum MeasureRegion {
    /// Every node of the results.
    #[default]
    All,
    /// The nodes of a node set of the analysis' mesh.
    NodeSet(String),
}

/// What a Measure block computes from the results of a step: of its last increment, or of the
/// mode for an eigenfrequency. Field and component names are those of the results workspace
/// (`STRESS` / `MISES`, `DISP` / `ALL`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum MeasureKind {
    /// The values of a field component at the nodes of a region, a series: a Math block
    /// takes its largest, smallest or mean value, a Plot block its histogram.
    FieldValues {
        field: String,
        component: String,
        region: MeasureRegion,
    },
    /// A field component at one node.
    NodeValue {
        field: String,
        component: String,
        node: u32,
    },
    /// A field component at a point, interpolated with the shape functions of the element
    /// that contains it. The coordinates are in SI units, each a row of the block that a
    /// value link can set.
    PointValue {
        field: String,
        component: String,
        point: [f64; 3],
    },
    /// A value of a history output (`.dat`), such as the total force of a set
    /// (`TOTAL_FORCE` / `RF3`): of the last increment of the step, or with `whole` the
    /// values of all its increments, a series that goes to Math and Plot blocks.
    History {
        set: String,
        field: String,
        component: String,
        #[serde(default)]
        whole: bool,
    },
    /// The eigenfrequency of mode `mode`, counted from 1, of a frequency step.
    Eigenfrequency { mode: u32 },
}

/// The keys of the coordinate rows of a measure at a point, X, Y and Z: a value link sets
/// them at `["Parameters", key]`.
pub const POINT_AXES: [&str; 3] = ["x", "y", "z"];

impl Default for MeasureKind {
    fn default() -> Self {
        Self::FieldValues {
            field: "STRESS".into(),
            component: "MISES".into(),
            region: MeasureRegion::All,
        }
    }
}

impl MeasureKind {
    /// The kinds a measure editor offers, each with defaults.
    pub fn choices() -> [Self; 5] {
        [
            Self::default(),
            Self::NodeValue {
                field: "DISP".into(),
                component: "ALL".into(),
                node: 1,
            },
            Self::PointValue {
                field: "DISP".into(),
                component: "ALL".into(),
                point: [0.0; 3],
            },
            Self::History {
                set: String::new(),
                field: "TOTAL_FORCE".into(),
                component: "RF3".into(),
                whole: false,
            },
            Self::Eigenfrequency { mode: 1 },
        ]
    }

    pub fn label(&self) -> &'static str {
        match self {
            Self::FieldValues { .. } => "Field values",
            Self::NodeValue { .. } => "Value at a node",
            Self::PointValue { .. } => "Value at a point",
            Self::History { .. } => "History output",
            Self::Eigenfrequency { .. } => "Eigenfrequency",
        }
    }

    /// The field and component its value has, for its unit; an eigenfrequency is `FREQ`.
    pub fn field(&self) -> (&str, &str) {
        match self {
            Self::FieldValues {
                field, component, ..
            }
            | Self::NodeValue {
                field, component, ..
            }
            | Self::PointValue {
                field, component, ..
            }
            | Self::History {
                field, component, ..
            } => (field, component),
            Self::Eigenfrequency { .. } => ("FREQ", ""),
        }
    }

    /// Whether it gives a series of values, one per increment, and not a single number.
    pub fn is_series(&self) -> bool {
        matches!(
            self,
            Self::FieldValues { .. } | Self::History { whole: true, .. }
        )
    }

    /// The point of a measure at a point.
    pub fn point(&self) -> Option<[f64; 3]> {
        match self {
            Self::PointValue { point, .. } => Some(*point),
            _ => None,
        }
    }

    /// Sets the coordinate of a measure at a point by its key in [`POINT_AXES`]; whether it
    /// changed.
    pub fn set_coordinate(&mut self, key: &str, value: f64) -> bool {
        let Self::PointValue { point, .. } = self else {
            return false;
        };
        let Some(axis) = POINT_AXES.iter().position(|a| *a == key) else {
            return false;
        };
        if !value.is_finite() || point[axis] == value {
            return false;
        }
        point[axis] = value;
        true
    }

    /// A short text of what it measures: "Maximum of STRESS MISES in TIP".
    pub fn describe(&self) -> String {
        match self {
            Self::FieldValues {
                field,
                component,
                region,
            } => {
                let region = match region {
                    MeasureRegion::All => String::new(),
                    MeasureRegion::NodeSet(set) => format!(" in {set}"),
                };
                format!("{field} {component}{region}")
            }
            Self::NodeValue {
                field,
                component,
                node,
            } => format!("{field} {component} at node {node}"),
            Self::PointValue {
                field, component, ..
            } => format!("{field} {component} at a point"),
            Self::History {
                set,
                field,
                component,
                whole,
            } => {
                let take = if *whole { "History of " } else { "" };
                format!("{take}{field} {component} of {set}")
            }
            Self::Eigenfrequency { mode } => format!("Eigenfrequency of mode {mode}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn field_values_and_whole_histories_are_series() {
        assert!(MeasureKind::default().is_series());
        let mut history = MeasureKind::choices()[3].clone();
        assert!(!history.is_series());
        if let MeasureKind::History { whole, .. } = &mut history {
            *whole = true;
        }
        assert!(history.is_series());
        assert!(!MeasureKind::Eigenfrequency { mode: 1 }.is_series());
    }
}
