//! The Particle Flow step: the material of the model flows as particles through a fixed
//! background grid (material point method), for pastes and fluids that deform far beyond
//! what a mesh follows. The step takes the model as it is: meshed solid parts with a section
//! are filled with particles of their material, rigid bodies are the walls the material
//! meets, contact regions between them give the wall law, a gravity load the gravity. The
//! step itself holds what has no counterpart in the model: the resolution, the time and the
//! box the material moves in.
//!
//! Like everything in the model this is what the user defined; the solver's particle model
//! is made of it when the analysis is started (`plx-io::particles`).

use serde::{Deserialize, Serialize};

/// How a material flows beyond its elasticity in a Particle Flow step. The elastic part
/// comes from the material's elastic constants (Young's modulus and Poisson's ratio).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Flow {
    /// Herschel-Bulkley: elastic below the yield stress in shear, flowing above it with the
    /// shear stress `yield + consistency * rate^exponent` (Bingham for `exponent = 1`).
    /// `consistency` is in Pa s^exponent.
    HerschelBulkley {
        yield_stress: f64,
        consistency: f64,
        exponent: f64,
    },
    /// A Newtonian fluid of this dynamic viscosity (Pa s), weakly compressible by the bulk
    /// modulus of the elastic constants, which carries no tension.
    Newtonian { viscosity: f64 },
}

impl Flow {
    /// The kinds with typical values: a stiff paste (such as minced meat) and water.
    pub const KINDS: [Flow; 2] = [
        Flow::HerschelBulkley {
            yield_stress: 1000.0,
            consistency: 500.0,
            exponent: 0.5,
        },
        Flow::Newtonian { viscosity: 1e-3 },
    ];

    pub fn name(&self) -> &'static str {
        match self {
            Flow::HerschelBulkley { .. } => "Herschel-Bulkley",
            Flow::Newtonian { .. } => "Newtonian",
        }
    }
}

/// What a side of the box of a Particle Flow step does to the material.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum BoxSide {
    /// A resting wall the material slides along without friction.
    #[default]
    Frictionless,
    /// A resting wall the material sticks to.
    Rough,
    /// A resting wall with Coulomb friction of this coefficient.
    Frictional { mu: f64 },
    /// Open: material that reaches it leaves the model and counts as outflow.
    Open,
}

impl BoxSide {
    pub const KINDS: [BoxSide; 4] = [
        BoxSide::Frictionless,
        BoxSide::Rough,
        BoxSide::Frictional { mu: 0.3 },
        BoxSide::Open,
    ];

    pub fn name(&self) -> &'static str {
        match self {
            BoxSide::Frictionless => "Frictionless",
            BoxSide::Rough => "Rough",
            BoxSide::Frictional { .. } => "Frictional",
            BoxSide::Open => "Open",
        }
    }
}

/// The names of the sides of the box, in the order of [`ParticleFlowStep::sides`].
pub const SIDE_NAMES: [&str; 6] = ["X min", "X max", "Y min", "Y max", "Z min", "Z max"];

/// The box the material of a Particle Flow step moves in: material outside of it is gone.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum ParticleBox {
    /// The bounding box of all parts of the model, enlarged by `margin` on every side.
    Around { margin: f64 },
    /// A box of its own, in global coordinates.
    Fixed { min: [f64; 3], max: [f64; 3] },
}

impl Default for ParticleBox {
    fn default() -> Self {
        ParticleBox::Around { margin: 0.01 }
    }
}

/// The settings of a Particle Flow step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ParticleFlowStep {
    /// Length of the step in time.
    pub duration: f64,
    /// Result frames written over the duration, evenly spaced, the last one at the end.
    pub frames: u32,
    /// The size of a cell of the background grid; the particles start two to a cell in
    /// each direction. Like the element size of a mesh it decides the accuracy and the
    /// computing time: a gap needs eight cells or more across.
    pub cell_size: f64,
    /// The speed the elastic waves are slowed to, by a smaller bulk modulus, so that the
    /// time step grows (like mass scaling); `None` keeps the material's own. Allowed while
    /// the material moves much slower than it.
    pub wave_speed: Option<f64>,
    pub domain: ParticleBox,
    /// The sides of the box: x min, x max, y min, y max, z min, z max.
    pub sides: [BoxSide; 6],
    /// The volume change is averaged over the cells (F-bar), against volumetric locking
    /// and noise of the pressure.
    pub smooth_volume: bool,
    /// The temperature of the material: heated by its own flow and by friction at the
    /// walls, conducted through it, exchanged with walls of a given temperature. It needs
    /// the conductivity and specific heat of the materials and their temperature at the
    /// start; a temperature table of the flow makes the material softer or stiffer.
    pub heat: bool,
    /// The heat transfer coefficient between the material and the walls of a given
    /// temperature, in W/(m² K).
    pub wall_transfer: f64,
}

impl Default for ParticleFlowStep {
    fn default() -> Self {
        Self {
            duration: 1.0,
            frames: 50,
            cell_size: 0.005,
            wave_speed: Some(20.0),
            domain: ParticleBox::default(),
            sides: [BoxSide::default(); 6],
            smooth_volume: true,
            heat: false,
            wall_transfer: 1000.0,
        }
    }
}

impl ParticleFlowStep {
    /// The box the background grid covers: the fixed one, or `parts` (the bounds of the
    /// parts that take part) enlarged by the margin.
    pub fn grid_box(&self, parts: Option<([f64; 3], [f64; 3])>) -> Option<([f64; 3], [f64; 3])> {
        match self.domain {
            ParticleBox::Fixed { min, max } => Some((min, max)),
            ParticleBox::Around { margin } => {
                parts.map(|(lo, hi)| (lo.map(|v| v - margin), hi.map(|v| v + margin)))
            }
        }
    }

    /// What keeps the settings from being computed, in words.
    pub fn problem(&self) -> Option<String> {
        let positive = |v: f64| v.is_finite() && v > 0.0;
        if !positive(self.duration) {
            return Some("The duration must be positive.".into());
        }
        if self.frames == 0 {
            return Some("At least one frame is needed.".into());
        }
        if !positive(self.cell_size) {
            return Some("The cell size must be positive.".into());
        }
        if self.wave_speed.is_some_and(|c| !positive(c)) {
            return Some("The wave speed must be positive.".into());
        }
        match self.domain {
            ParticleBox::Around { margin } if !(margin.is_finite() && margin >= 0.0) => {
                return Some("The margin of the box must not be negative.".into());
            }
            ParticleBox::Fixed { min, max } if (0..3).any(|a| max[a] <= min[a]) => {
                return Some("The box must be larger than its smallest corner.".into());
            }
            _ => {}
        }
        if self.heat && !(self.wall_transfer.is_finite() && self.wall_transfer >= 0.0) {
            return Some("The heat transfer coefficient must not be negative.".into());
        }
        for (side, name) in self.sides.iter().zip(SIDE_NAMES) {
            if let BoxSide::Frictional { mu } = side
                && !(mu.is_finite() && *mu >= 0.0)
            {
                return Some(format!("Side {name}: the friction must not be negative."));
            }
        }
        None
    }
}

impl Flow {
    /// What keeps the flow from being computed, in words.
    pub fn problem(&self) -> Option<String> {
        let ok = |v: f64| v.is_finite() && v >= 0.0;
        match *self {
            Flow::HerschelBulkley {
                yield_stress,
                consistency,
                exponent,
            } => {
                if !ok(yield_stress) || !ok(consistency) {
                    Some("Yield stress and consistency must not be negative.".into())
                } else if !(exponent.is_finite() && exponent > 0.0) {
                    Some("The flow index must be positive.".into())
                } else if yield_stress == 0.0 && consistency == 0.0 {
                    Some("Give a yield stress or a consistency.".into())
                } else {
                    None
                }
            }
            Flow::Newtonian { viscosity } => (!(viscosity.is_finite() && viscosity > 0.0))
                .then(|| "The viscosity must be positive.".into()),
        }
    }
}
