//! What a Campbell block of the project schematic keeps: the settings of its sweep over the
//! rotational speed and the complex modes the solver found at each speed. The sweep itself and
//! the diagram are in the application.

use serde::{Deserialize, Serialize};

/// Whirl of a mode as CalculiX reports it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Whirl {
    Forward,
    Backward,
    Unknown,
}

impl Whirl {
    /// The letter the diagram and the table mark the whirl with.
    pub fn letter(self) -> &'static str {
        match self {
            Whirl::Forward => "F",
            Whirl::Backward => "B",
            Whirl::Unknown => "",
        }
    }
}

/// One complex mode at one speed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Mode {
    /// Real part of the frequency in cycles per time.
    pub frequency: f64,
    pub whirl: Whirl,
    /// The eigenvalue has an imaginary part of weight: the mode grows instead of
    /// oscillating, as beyond a stability limit.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub unstable: bool,
    /// Number of nodal diameters of the mode about the rotation axis, as the solver finds
    /// it from the mode shape; `None` where it has no clear one (a rotor that is not
    /// axisymmetric) or the sweep is older than the column. It tells the frequencies in
    /// the stationary frame: `f ± n Ω` for `n` nodal diameters.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nodal_diameter: Option<u32>,
}

/// The frame of reference the diagram shows the frequencies in.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Frame {
    /// The solver's: the frame rotating with the rotor.
    #[default]
    Rotating,
    /// That of an observer at rest, such as a microphone or a sensor on the housing: a mode
    /// of `n` nodal diameters at `f` in the rotating frame is seen as the forward travelling
    /// wave `f + n Ω` and the backward travelling wave `|f − n Ω|`.
    Stationary,
}

impl Frame {
    pub const ALL: [Frame; 2] = [Frame::Rotating, Frame::Stationary];

    pub fn label(self) -> &'static str {
        match self {
            Frame::Rotating => "Rotating (with the rotor)",
            Frame::Stationary => "Stationary (observer at rest)",
        }
    }
}

/// The modes the solver found at one speed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpeedPoint {
    /// Rotational speed in radians per time.
    pub speed: f64,
    pub modes: Vec<Mode>,
}

/// Settings of the sweep.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Setup {
    /// Lowest speed of the sweep in radians per time; standstill itself is left out.
    pub min_speed: f64,
    /// Highest speed of the sweep in radians per time.
    pub max_speed: f64,
    /// The solver runs before the sweep stops by itself; 0 runs until it is stopped.
    pub max_runs: u32,
    /// Engine orders drawn as lines, e.g. "1, 2".
    pub orders: String,
    /// Upper end of the frequency axis in Hz; 0 fits the axis to the modes.
    pub max_frequency: f64,
    /// The frame of reference of the diagram.
    #[serde(default)]
    pub frame: Frame,
}

impl Default for Setup {
    fn default() -> Self {
        Self {
            min_speed: 0.0,
            max_speed: 0.0,
            max_runs: 40,
            orders: "1".into(),
            max_frequency: 0.0,
            frame: Frame::Rotating,
        }
    }
}
