//! The loads of the material on the walls of a Particle Flow step, taken by a step of
//! another analysis: the Wall loads input of the step, linked from the Particle Flow step,
//! makes a [`LoadKind::WallLoads`](crate::LoadKind::WallLoads) of the step. When the input
//! file is written, the points of the walls with their mean forces are spread onto the faces
//! of the load's region near them as nodal forces (roadmap stage 5, the Loads link).

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// The loads of a Particle Flow run on its walls, as a load of a step.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct WallLoads {
    /// The file of the loads the run wrote (`.walls`), a copy for the analysis taking them;
    /// set by the link.
    pub file: PathBuf,
    /// The walls taken, by the ids of their rigid bodies or boundary conditions; empty: all.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub walls: Vec<crate::Id>,
    /// Which time of the run.
    pub time: WallLoadTime,
}

/// Which time of the Particle Flow run a step takes the wall loads of. The loads of a frame
/// are the mean since the frame before.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum WallLoadTime {
    /// The last frame.
    #[default]
    Last,
    /// The frame nearest the time.
    At(f64),
    /// The mean from a time to another, to the end without one: for a steady flow, the
    /// loads without the noise of single frames.
    Mean { from: f64, to: Option<f64> },
    /// Every frame at its own time of the step, linear in between: the loads over the time,
    /// for a dynamic step.
    OverTime,
}

impl WallLoadTime {
    pub const LABELS: [&'static str; 4] =
        ["Last frame", "Frame at time", "Mean over time", "Over time"];

    pub fn index(&self) -> usize {
        match self {
            WallLoadTime::Last => 0,
            WallLoadTime::At(_) => 1,
            WallLoadTime::Mean { .. } => 2,
            WallLoadTime::OverTime => 3,
        }
    }
}

impl WallLoads {
    /// The file, if one is set.
    pub fn file(&self) -> Option<&Path> {
        (!self.file.as_os_str().is_empty()).then_some(self.file.as_path())
    }

    /// Whether the wall `id` is taken.
    pub fn takes(&self, id: crate::Id) -> bool {
        self.walls.is_empty() || self.walls.contains(&id)
    }
}
