//! The arithmetic of the number blocks of the project schematic: a Math block computes a
//! number from two or three others, a Switch block picks one of several by its index.

use serde::{Deserialize, Serialize};

use crate::ValueKind;
use crate::units::Dimension;

/// What a Math block computes, like the operations of Blender's Math node.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MathOp {
    #[default]
    Add,
    Subtract,
    Multiply,
    Divide,
    Minimum,
    Maximum,
    /// The value kept between a lower and an upper limit.
    Clamp,
}

impl MathOp {
    pub const ALL: [Self; 7] = [
        Self::Add,
        Self::Subtract,
        Self::Multiply,
        Self::Divide,
        Self::Minimum,
        Self::Maximum,
        Self::Clamp,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Add => "Add",
            Self::Subtract => "Subtract",
            Self::Multiply => "Multiply",
            Self::Divide => "Divide",
            Self::Minimum => "Minimum",
            Self::Maximum => "Maximum",
            Self::Clamp => "Clamp",
        }
    }

    /// What it computes, for its tooltip.
    pub fn formula(self) -> &'static str {
        match self {
            Self::Add => "A + B",
            Self::Subtract => "A - B",
            Self::Multiply => "A x B",
            Self::Divide => "A / B",
            Self::Minimum => "the smaller of A and B",
            Self::Maximum => "the larger of A and B",
            Self::Clamp => "Value, but at least Min and at most Max",
        }
    }

    /// The names of its inputs, in order.
    pub fn inputs(self) -> &'static [&'static str] {
        match self {
            Self::Clamp => &["Value", "Min", "Max"],
            _ => &["A", "B"],
        }
    }

    /// Whether its inputs have kinds of their own, which the kind of its result follows
    /// from: a force over a length is a force per length. The other operations take and give
    /// numbers of one kind.
    pub fn combines(self) -> bool {
        matches!(self, Self::Multiply | Self::Divide)
    }

    /// The kinds of A, B and the result of a product or quotient, with a missing one found
    /// from the other two: a force over a force per length is a length. An error says why
    /// the kinds do not fit together, or that no kind of number has the missing one's unit.
    pub fn settle_kinds(
        self,
        kinds: [Option<ValueKind>; 3],
    ) -> Result<[Option<ValueKind>; 3], String> {
        if !self.combines() {
            return Ok(kinds);
        }
        let dim = |k: ValueKind| k.dimension();
        let divide = self == Self::Divide;
        let (missing, dimension) = match kinds {
            [Some(a), Some(b), result] => {
                let dimension = if divide {
                    dim(a).over(dim(b))
                } else {
                    dim(a).times(dim(b))
                };
                if let Some(result) = result {
                    return if dim(result) == dimension {
                        Ok(kinds)
                    } else {
                        Err(format!(
                            "{} is {}, not a {}",
                            self.formula(),
                            describe(dimension),
                            result.label().to_lowercase()
                        ))
                    };
                }
                (2, dimension)
            }
            [Some(a), None, Some(result)] => (
                1,
                if divide {
                    dim(a).over(dim(result))
                } else {
                    dim(result).over(dim(a))
                },
            ),
            [None, Some(b), Some(result)] => (
                0,
                if divide {
                    dim(result).times(dim(b))
                } else {
                    dim(result).over(dim(b))
                },
            ),
            _ => return Ok(kinds),
        };
        let Some(kind) = ValueKind::of_dimension(dimension) else {
            let name = ["A", "B", "the result"][missing];
            return Err(format!(
                "{name} would be in {}, and no kind of number is",
                dimension.si()
            ));
        };
        let mut kinds = kinds;
        kinds[missing] = Some(kind);
        Ok(kinds)
    }

    /// The dimension a product or quotient of `a` and `b` has, else that of `a`.
    pub fn dimension(self, a: Dimension, b: Dimension) -> Dimension {
        match self {
            Self::Multiply => a.times(b),
            Self::Divide => a.over(b),
            _ => a,
        }
    }

    /// The result of `values`, its inputs in order; `None` for a division by zero or too
    /// few values.
    pub fn apply(self, values: &[f64]) -> Option<f64> {
        let (a, b) = (*values.first()?, *values.get(1)?);
        let result = match self {
            Self::Add => a + b,
            Self::Subtract => a - b,
            Self::Multiply => a * b,
            Self::Divide if b == 0.0 => return None,
            Self::Divide => a / b,
            Self::Minimum => a.min(b),
            Self::Maximum => a.max(b),
            // Like Blender: the upper limit wins when the limits cross.
            Self::Clamp => a.max(b).min(*values.get(2)?),
        };
        result.is_finite().then_some(result)
    }
}

/// What numbers of a dimension are, for messages: "a force", or "in kg m^3" for a dimension
/// no kind of number has.
fn describe(dimension: Dimension) -> String {
    match ValueKind::of_dimension(dimension) {
        Some(ValueKind::Factor) => "a factor".to_owned(),
        Some(kind) => format!("a {}", kind.label().to_lowercase()),
        None => format!("in {}", dimension.si()),
    }
}

/// The value a Switch block gives: the choice of number `index`, counted from 1; `None` when
/// there is no such choice.
pub fn switch(index: f64, choices: &[f64]) -> Option<f64> {
    let index = index.round();
    if !(1.0..=choices.len() as f64).contains(&index) {
        return None;
    }
    choices.get(index as usize - 1).copied()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn math_computes_its_operation() {
        assert_eq!(MathOp::Add.apply(&[2.0, 3.0]), Some(5.0));
        assert_eq!(MathOp::Subtract.apply(&[2.0, 3.0]), Some(-1.0));
        assert_eq!(MathOp::Multiply.apply(&[0.1, 2.0]), Some(0.2));
        assert_eq!(MathOp::Divide.apply(&[1.0, 4.0]), Some(0.25));
        assert_eq!(MathOp::Divide.apply(&[1.0, 0.0]), None);
        assert_eq!(MathOp::Minimum.apply(&[2.0, 3.0]), Some(2.0));
        assert_eq!(MathOp::Maximum.apply(&[2.0, 3.0]), Some(3.0));
        assert_eq!(MathOp::Clamp.apply(&[5.0, 0.0, 3.0]), Some(3.0));
        assert_eq!(MathOp::Clamp.apply(&[-1.0, 0.0, 3.0]), Some(0.0));
        assert_eq!(MathOp::Clamp.apply(&[1.0, 0.0]), None);
        assert_eq!(MathOp::Clamp.inputs().len(), 3);
        assert!(MathOp::Multiply.combines() && !MathOp::Add.combines());
    }

    #[test]
    fn products_and_quotients_combine_the_kinds_of_their_numbers() {
        use crate::Quantity::*;
        let q = |q| Some(ValueKind::Quantity(q));
        let divide = |k| MathOp::Divide.settle_kinds(k);
        let multiply = |k| MathOp::Multiply.settle_kinds(k);
        // The stiffness of a spring: a force over a length.
        assert_eq!(
            divide([q(Force), q(Length), None]),
            Ok([q(Force), q(Length), q(ForcePerLength)])
        );
        // Like over like is a factor; a length times a factor stays a length.
        assert_eq!(
            divide([q(Length), q(Length), None]).unwrap()[2],
            Some(ValueKind::Factor)
        );
        assert_eq!(
            multiply([q(Length), Some(ValueKind::Factor), None]).unwrap()[2],
            q(Length)
        );
        // A missing input follows from the other one and the result.
        assert_eq!(divide([q(Force), None, q(Pressure)]).unwrap()[1], q(Area));
        assert_eq!(divide([None, q(Time), q(Velocity)]).unwrap()[0], q(Length));
        assert_eq!(
            multiply([q(Force), None, q(Force)]).unwrap()[1],
            Some(ValueKind::Factor)
        );
        assert_eq!(multiply([None, q(Length), q(Moment)]).unwrap()[0], q(Force));
        assert_eq!(
            divide([q(Angle), q(Time), None]).unwrap()[2],
            q(RotationalSpeed)
        );
        // A product of temperatures counts with changes of temperature.
        assert_eq!(
            multiply([q(ThermalExpansion), q(TemperatureDifference), None]).unwrap()[2],
            Some(ValueKind::Factor)
        );
        assert_eq!(
            multiply([q(Area), q(Area), q(Length)]).unwrap_err(),
            "A x B is a second moment of area, not a length"
        );
        // Kinds that do not fit, and units no kind has.
        let wrong = divide([q(Force), q(Length), q(Pressure)]).unwrap_err();
        assert_eq!(wrong, "A / B is a force per length, not a pressure");
        let none = multiply([q(SecondMomentOfArea), q(Pressure), None]).unwrap_err();
        assert_eq!(
            none,
            "the result would be in kg m^3 s^-2, and no kind of number is"
        );
        // Only two of three known: nothing to find yet.
        assert_eq!(divide([q(Force), None, None]), Ok([q(Force), None, None]));
        assert_eq!(
            MathOp::Add.settle_kinds([q(Force), None, None]),
            Ok([q(Force), None, None])
        );
    }

    #[test]
    fn switch_picks_a_choice_by_its_index_from_one() {
        let choices = [10.0, 20.0, 30.0];
        assert_eq!(switch(1.0, &choices), Some(10.0));
        assert_eq!(switch(2.6, &choices), Some(30.0));
        assert_eq!(switch(0.0, &choices), None);
        assert_eq!(switch(4.0, &choices), None);
        assert_eq!(switch(f64::NAN, &choices), None);
    }
}
