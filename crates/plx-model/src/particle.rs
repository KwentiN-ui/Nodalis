//! What a Particle Analysis block of the project schematic keeps: which parts of its
//! geometry are filled with material and which are rigid walls, how the material flows,
//! the box it moves in and how long it is computed. It is the abstract description of the
//! analysis (concept `STUFENKONZEPT_1_2_PARTIKEL_MPM.md`); the solver's input, with
//! triangulated surfaces, moduli and the background grid, is made from it only when it is
//! solved.
//!
//! The elastic constants and the density come from the material of the body, a material
//! of the materials linked to the block; how it flows beyond its elastic part is set here,
//! per body.
//!
//! Values in SI units ([`crate::STORED`]).

use serde::{Deserialize, Serialize};

/// The setup of a particle analysis.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ParticleSetup {
    /// Parts filled with material at the start.
    #[serde(default)]
    pub bodies: Vec<ParticleBody>,
    /// Parts that are rigid walls.
    #[serde(default)]
    pub walls: Vec<ParticleWall>,
    /// The size of a cell of the background grid: the resolution, like the element size
    /// of a mesh. Particles start two to a cell and direction.
    pub cell: f64,
    /// The box the material moves in; `None`: the box around all bodies and walls.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bounds: Option<[[f64; 3]; 2]>,
    /// The sides of the box: `[x min, x max, y min, y max, z min, z max]`.
    pub sides: [BoxSide; 6],
    pub gravity: [f64; 3],
    pub end_time: f64,
    /// Frames of the results over the time, besides the start.
    pub frames: usize,
    /// The speed the elastic waves are slowed to, or `None` for the material's own; see
    /// [`ParticleSetup::suggested_sound_speed`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sound_speed: Option<f64>,
}

impl Default for ParticleSetup {
    fn default() -> Self {
        Self {
            bodies: Vec::new(),
            walls: Vec::new(),
            cell: 0.005,
            bounds: None,
            sides: [BoxSide::Slip; 6],
            gravity: [0.0, 0.0, -9.81],
            end_time: 1.0,
            frames: 50,
            sound_speed: None,
        }
    }
}

impl ParticleSetup {
    /// The role of a part: a body, a wall, or none.
    pub fn role(&self, part: &str) -> PartRole {
        if let Some(body) = self.bodies.iter().find(|b| b.part == part) {
            return PartRole::Body(body.clone());
        }
        if let Some(wall) = self.walls.iter().find(|w| w.part == part) {
            return PartRole::Wall(wall.law);
        }
        PartRole::None
    }

    /// Gives a part a role, in place of the one it had.
    pub fn set_role(&mut self, part: &str, role: PartRole) {
        self.bodies.retain(|b| b.part != part);
        self.walls.retain(|w| w.part != part);
        match role {
            PartRole::Body(mut body) => {
                body.part = part.to_owned();
                self.bodies.push(body);
            }
            PartRole::Wall(law) => self.walls.push(ParticleWall {
                part: part.to_owned(),
                law,
            }),
            PartRole::None => {}
        }
    }

    /// Renames a part in the roles.
    pub fn rename_part(&mut self, old: &str, new: &str) {
        for b in &mut self.bodies {
            if b.part == old {
                b.part = new.to_owned();
            }
        }
        for w in &mut self.walls {
            if w.part == old {
                w.part = new.to_owned();
            }
        }
    }

    /// A wave speed that keeps the time step large and the result unchanged: ten times the
    /// speed of a free fall over `height`, the largest speed gravity gives the material.
    pub fn suggested_sound_speed(&self, height: f64) -> f64 {
        let g = (self.gravity.iter().map(|g| g * g).sum::<f64>()).sqrt();
        10.0 * (2.0 * g * height.max(0.0)).sqrt().max(1.0)
    }
}

/// What a part of the geometry is in a particle analysis.
#[derive(Clone, Debug, PartialEq)]
pub enum PartRole {
    Body(ParticleBody),
    Wall(WallLaw),
    None,
}

/// A part filled with material.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ParticleBody {
    pub part: String,
    /// The name of the material, one of the materials linked to the block.
    pub material: String,
    pub flow: Flow,
    #[serde(default)]
    pub velocity: [f64; 3],
}

/// How the material of a body flows beyond its elastic part.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Flow {
    /// It does not: a hyperelastic solid at large strains.
    Elastic,
    /// A paste with a yield stress in shear after Herschel-Bulkley: in flow the shear
    /// stress is `yield_stress + consistency · rate^exponent` (Bingham for exponent 1).
    Paste {
        yield_stress: f64,
        /// In Pa·s^exponent.
        consistency: f64,
        exponent: f64,
    },
    /// A Newtonian fluid: the bulk modulus of the material and this viscosity, no shear
    /// stiffness and no tension.
    Fluid { viscosity: f64 },
}

impl Flow {
    pub const NAMES: [&'static str; 3] = ["Elastic", "Paste (Herschel-Bulkley)", "Fluid"];

    pub fn index(&self) -> usize {
        match self {
            Flow::Elastic => 0,
            Flow::Paste { .. } => 1,
            Flow::Fluid { .. } => 2,
        }
    }

    /// The flow of the kind of `NAMES[index]`, with typical values.
    pub fn of_index(index: usize) -> Self {
        match index {
            1 => Flow::Paste {
                yield_stress: 500.0,
                consistency: 50.0,
                exponent: 0.5,
            },
            2 => Flow::Fluid { viscosity: 1e-3 },
            _ => Flow::Elastic,
        }
    }
}

/// A part that is a rigid wall.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ParticleWall {
    pub part: String,
    pub law: WallLaw,
}

/// How material slides along a wall.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum WallLaw {
    /// No slip.
    Stick,
    /// Free slip.
    Slip,
    /// Coulomb friction.
    Friction(f64),
}

impl WallLaw {
    pub const NAMES: [&'static str; 3] = ["Stick", "Slip", "Friction"];

    pub fn index(&self) -> usize {
        match self {
            WallLaw::Stick => 0,
            WallLaw::Slip => 1,
            WallLaw::Friction(_) => 2,
        }
    }

    pub fn of_index(index: usize) -> Self {
        match index {
            0 => WallLaw::Stick,
            2 => WallLaw::Friction(0.3),
            _ => WallLaw::Slip,
        }
    }
}

/// A side of the box the material moves in.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum BoxSide {
    /// A wall with free slip.
    Slip,
    /// A wall without slip.
    Stick,
    /// Open: material that passes it leaves the analysis.
    Open,
}

impl BoxSide {
    pub const ALL: [BoxSide; 3] = [BoxSide::Slip, BoxSide::Stick, BoxSide::Open];

    pub fn label(self) -> &'static str {
        match self {
            BoxSide::Slip => "Wall, slip",
            BoxSide::Stick => "Wall, stick",
            BoxSide::Open => "Open",
        }
    }
}

/// The names of the sides of the box, in the order of [`ParticleSetup::sides`].
pub const SIDE_NAMES: [&str; 6] = ["X min", "X max", "Y min", "Y max", "Z min", "Z max"];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roles_replace_each_other() {
        let mut setup = ParticleSetup::default();
        setup.set_role("A", PartRole::Wall(WallLaw::Stick));
        assert_eq!(setup.role("A"), PartRole::Wall(WallLaw::Stick));
        let body = ParticleBody {
            part: String::new(),
            material: "Paste".into(),
            flow: Flow::of_index(1),
            velocity: [0.0; 3],
        };
        setup.set_role("A", PartRole::Body(body));
        assert!(setup.walls.is_empty());
        assert!(matches!(setup.role("A"), PartRole::Body(b) if b.part == "A"));
        setup.rename_part("A", "B");
        assert!(matches!(setup.role("B"), PartRole::Body(_)));
        setup.set_role("B", PartRole::None);
        assert_eq!(setup.role("B"), PartRole::None);
    }
}
