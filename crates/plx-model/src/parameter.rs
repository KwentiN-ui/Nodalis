//! Numbers of the project schematic that flow along value links: the parameters of a
//! Parameters block, and the inputs of other blocks they feed, such as the size and division
//! of a mesh block.

use serde::{Deserialize, Serialize};

use crate::Quantity;
use crate::units::Dimension;

/// What a number is, so that a value link joins like with like: a force goes to a force, a
/// count of elements to a count.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ValueKind {
    /// A physical quantity, stored in SI units.
    Quantity(Quantity),
    /// A whole number, such as the elements along an edge.
    Count,
    /// A factor without unit, such as the refinement of a mesh.
    Factor,
    /// A number of a unit no quantity has, such as an intermediate result of a Math block:
    /// a second moment of area times a pressure. Shown in SI base units, it goes only to
    /// other Math blocks.
    Derived(Dimension),
}

impl Default for ValueKind {
    fn default() -> Self {
        Self::LENGTH
    }
}

impl ValueKind {
    pub const LENGTH: Self = Self::Quantity(Quantity::Length);

    /// Every kind a parameter can have: count and factor, then the quantities.
    pub fn all() -> Vec<Self> {
        let mut all = vec![Self::Count, Self::Factor];
        all.extend(Quantity::ALL.into_iter().map(Self::Quantity));
        all
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Quantity(quantity) => quantity.label(),
            Self::Count => "Count",
            Self::Factor => "Factor",
            Self::Derived(_) => "Derived quantity",
        }
    }

    /// What its numbers are, for messages: its label in lower case, with the SI base units
    /// of a derived quantity.
    pub fn describe(self) -> String {
        match self {
            Self::Derived(dimension) => format!("quantity in {}", dimension.si()),
            _ => self.label().to_lowercase(),
        }
    }

    /// The quantity its values are shown in, for a physical quantity.
    pub fn quantity(self) -> Option<Quantity> {
        match self {
            Self::Quantity(quantity) => Some(quantity),
            _ => None,
        }
    }

    /// The powers of the base units its numbers are made of; none for a count or a factor.
    pub fn dimension(self) -> Dimension {
        match self {
            Self::Quantity(quantity) => quantity.dimension(),
            Self::Derived(dimension) => dimension,
            Self::Count | Self::Factor => Dimension::default(),
        }
    }

    /// The kind of numbers of a dimension: a factor without unit, else its quantity (see
    /// [`Dimension::quantity`]), else a derived quantity.
    pub fn of_dimension(dimension: Dimension) -> Self {
        if dimension.is_none() {
            return Self::Factor;
        }
        dimension
            .quantity()
            .map_or(Self::Derived(dimension), Self::Quantity)
    }

    /// Whether a value of this kind may go to a number of kind `to`: only like to like, a
    /// force to a force and a count to a count.
    pub fn fits(self, to: Self) -> bool {
        self == to
    }

    /// A value of this kind as it is kept: a count is a whole number of at least one.
    pub fn clean(self, value: f64) -> f64 {
        match self {
            Self::Count if value.is_finite() => value.round().max(1.0),
            _ => value,
        }
    }
}

/// A named number of a Parameters block, which value links give to other blocks.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Parameter {
    pub name: String,
    pub kind: ValueKind,
    /// In SI units.
    pub value: f64,
}

impl Parameter {
    pub fn new(name: impl Into<String>, kind: ValueKind, value: f64) -> Self {
        Self {
            name: name.into(),
            kind,
            value: kind.clean(value),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counts_are_whole_numbers_of_at_least_one() {
        assert_eq!(Parameter::new("n", ValueKind::Count, 3.6).value, 4.0);
        assert_eq!(ValueKind::Count.clean(0.2), 1.0);
        assert_eq!(ValueKind::LENGTH.clean(0.0125), 0.0125);
        assert_eq!(ValueKind::Factor.clean(1.5), 1.5);
    }
}
