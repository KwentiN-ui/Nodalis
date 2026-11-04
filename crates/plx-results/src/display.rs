//! How the shown field is drawn: as colour contours alone, or with the directions of a vector
//! source on top, whose colours still come from the shown component and its legend.

use serde::{Deserialize, Serialize};

use crate::{Field, FieldRank, Principal, VectorSource};

/// The display of the shown field, independent of the field itself.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Representation {
    /// Colour contours of the shown component.
    #[default]
    Contour,
    /// A glyph per node on the surface, along the direction of the vector source.
    Arrows,
    /// Line integral convolution: streaks along the direction of the vector source over the
    /// contours.
    Lic,
}

impl Representation {
    pub const ALL: [Representation; 3] = [
        Representation::Contour,
        Representation::Arrows,
        Representation::Lic,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Representation::Contour => "Contour",
            Representation::Arrows => "Arrows",
            Representation::Lic => "LIC",
        }
    }
}

/// How arrows are drawn.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ArrowOptions {
    /// About this many arrows spread evenly over the surface.
    pub count: u32,
    /// Arrow length relative to their spacing; the longest when scaled by magnitude.
    pub length: f64,
    /// Lengths follow the magnitude of the source, else all are equally long.
    pub by_magnitude: bool,
    /// Arrows take the contour colours and the surfaces turn grey; else the arrows are black
    /// on the contours.
    pub colored: bool,
}

impl Default for ArrowOptions {
    fn default() -> Self {
        Self {
            count: 2000,
            length: 0.8,
            by_magnitude: false,
            colored: true,
        }
    }
}

impl ArrowOptions {
    pub const MIN_COUNT: u32 = 50;
    pub const MAX_COUNT: u32 = 50_000;
}

/// How the streaks of [`Representation::Lic`] look.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LicOptions {
    /// Length of a streak to either side of a pixel, in pixels.
    pub length: u32,
    /// Size of the grains of the noise the streaks are smeared from, in percent of the model
    /// size; never finer than a pixel.
    pub grain: f64,
    /// How strongly the streaks stand out from each other.
    pub contrast: f64,
    /// Share of the brightness the streaks set, from 0 (contours alone) to 1.
    pub mix: f64,
}

impl Default for LicOptions {
    fn default() -> Self {
        Self {
            length: 15,
            grain: 0.1,
            contrast: 4.0,
            mix: 0.7,
        }
    }
}

impl LicOptions {
    pub const LENGTH: std::ops::RangeInclusive<u32> = 3..=60;
    pub const GRAIN: std::ops::RangeInclusive<f64> = 0.01..=2.0;
    pub const CONTRAST: std::ops::RangeInclusive<f64> = 1.0..=10.0;
}

/// What the results view draws besides the contours, saved with the view.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct VectorDisplay {
    /// The chosen representation; shown only while the field gives a vector source.
    pub representation: Representation,
    /// Which principal direction a tensor field gives.
    pub principal: Principal,
    /// The field whose directions are drawn; `None` follows the field shown as contours.
    pub field: Option<String>,
    pub arrows: ArrowOptions,
    pub lic: LicOptions,
}

impl VectorDisplay {
    /// The directions the shown field gives: a vector field its own, a tensor field the chosen
    /// principal direction; `None` for a scalar field.
    pub fn source(&self, field: &Field) -> Option<VectorSource> {
        let field_name = field.name.clone();
        match field.rank {
            FieldRank::Vector => Some(VectorSource::Vector { field: field_name }),
            FieldRank::Tensor(_) => Some(VectorSource::Principal {
                field: field_name,
                which: self.principal,
            }),
            FieldRank::Scalar => None,
        }
    }

    /// The field whose directions are drawn over `shown`: the chosen one where `increment`
    /// has it, else `shown` itself.
    pub fn directions<'a>(&self, shown: &'a Field, increment: &'a crate::Increment) -> &'a Field {
        (self.field.as_deref())
            .and_then(|name| increment.field(name))
            .unwrap_or(shown)
    }

    /// The representation drawn for the field: contours alone where it has no directions,
    /// while the choice stays for the next vector or tensor field.
    pub fn shown(&self, field: &Field) -> Representation {
        match self.source(field) {
            Some(_) => self.representation,
            None => Representation::Contour,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Component;

    fn field(name: &str, raw: usize) -> Field {
        let component = |k: usize| Component {
            name: format!("C{k}"),
            values: vec![0.0],
            derived: false,
        };
        Field::new(name, (0..raw).map(component).collect())
    }

    #[test]
    fn scalar_fields_fall_back_to_contours() {
        let display = VectorDisplay {
            representation: Representation::Arrows,
            principal: Principal::Min,
            ..Default::default()
        };
        let stress = field("STRESS", 6);
        assert_eq!(display.shown(&stress), Representation::Arrows);
        assert_eq!(
            display.source(&stress).unwrap().label(),
            "Principal Min of STRESS"
        );
        assert_eq!(display.source(&field("DISP", 3)).unwrap().label(), "DISP");
        assert_eq!(display.shown(&field("ENER", 1)), Representation::Contour);
    }
}
