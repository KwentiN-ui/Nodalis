//! Settings of the colour scale (legend) of a results file:
//! colour spectrum, number of colours and the value range they span.

use serde::{Deserialize, Serialize};

/// Default number of colour bands.
pub const DEFAULT_LEVELS: u32 = 9;
/// Most colour bands of a scale.
pub const MAX_LEVELS: u32 = 24;

/// Colour spectra of the legend settings.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ColorSpectrum {
    CoolWarm,
    #[default]
    Rainbow,
    RainbowDesaturated,
    Turbo,
    Jet,
    Warm,
    Cool,
    Cividis,
    Viridis,
    Plasma,
    BlackBody,
    Inferno,
    Kindlmann,
    Grayscale,
}

impl ColorSpectrum {
    pub const ALL: [ColorSpectrum; 14] = [
        ColorSpectrum::CoolWarm,
        ColorSpectrum::Rainbow,
        ColorSpectrum::RainbowDesaturated,
        ColorSpectrum::Turbo,
        ColorSpectrum::Jet,
        ColorSpectrum::Warm,
        ColorSpectrum::Cool,
        ColorSpectrum::Cividis,
        ColorSpectrum::Viridis,
        ColorSpectrum::Plasma,
        ColorSpectrum::BlackBody,
        ColorSpectrum::Inferno,
        ColorSpectrum::Kindlmann,
        ColorSpectrum::Grayscale,
    ];

    pub fn label(self) -> &'static str {
        match self {
            ColorSpectrum::CoolWarm => "Cool-warm",
            ColorSpectrum::Rainbow => "Rainbow",
            ColorSpectrum::RainbowDesaturated => "Rainbow desaturated",
            ColorSpectrum::Turbo => "Turbo",
            ColorSpectrum::Jet => "Jet",
            ColorSpectrum::Warm => "Warm",
            ColorSpectrum::Cool => "Cool",
            ColorSpectrum::Cividis => "Cividis",
            ColorSpectrum::Viridis => "Viridis",
            ColorSpectrum::Plasma => "Plasma",
            ColorSpectrum::BlackBody => "Black body",
            ColorSpectrum::Inferno => "Inferno",
            ColorSpectrum::Kindlmann => "Kindlmann",
            ColorSpectrum::Grayscale => "Grayscale",
        }
    }
}

/// The colour scale of one results file.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ColorScale {
    pub spectrum: ColorSpectrum,
    /// Lightens the spectrum towards white, 0 ... 1.
    pub brightness: f64,
    /// Maximum colour at the minimum and vice versa.
    pub reverse: bool,
    /// Number of discrete colours, 2 ... [`MAX_LEVELS`].
    pub levels: u32,
    /// Minimum and maximum the user set; `None` for the automatic limits, the
    /// smallest and largest shown value. Values beyond them get the out-of-range colours.
    pub manual: Option<[f64; 2]>,
    /// sRGB colour of the values below the manual minimum.
    pub min_color: [u8; 3],
    /// sRGB colour of the values above the manual maximum.
    pub max_color: [u8; 3],
    /// Show the box of the values below the minimum in the legend.
    pub show_min_color: bool,
    /// Show the box of the values above the maximum in the legend.
    pub show_max_color: bool,
}

impl Default for ColorScale {
    /// The defaults: a discrete rainbow of nine colours lightened by 20 %, light and
    /// dark grey beyond the limits.
    fn default() -> Self {
        Self {
            spectrum: ColorSpectrum::Rainbow,
            brightness: 0.2,
            reverse: false,
            levels: DEFAULT_LEVELS,
            manual: None,
            min_color: [211, 211, 211],
            max_color: [169, 169, 169],
            show_min_color: true,
            show_max_color: true,
        }
    }
}

impl ColorScale {
    /// The manual limits, if the scale has valid ones.
    pub fn manual_limits(&self) -> Option<(f64, f64)> {
        let [min, max] = self.manual?;
        valid_limits(min, max).then_some((min, max))
    }
}

/// Limits a manual scale can use: finite with the minimum below the maximum.
pub fn valid_limits(min: f64, max: f64) -> bool {
    min.is_finite() && max.is_finite() && min < max
}
