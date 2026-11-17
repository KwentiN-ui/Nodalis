//! The input and the results of a particle analysis (material point method), beside the FE
//! [`crate::Model`]: what is filled with material, the rigid bodies it meets, the box it moves
//! in, and what the solver writes back. Concept: `STUFENKONZEPT_1_2_PARTIKEL_MPM.md`.
//!
//! Like the FE model this holds data only. Cells, shape functions and the time step are the
//! solver's (`plxs-particles`); the model gives the cell size as the resolution, the way a
//! mesh gives its element size.
//!
//! Units: consistent, like the FE model (Nodalis hands in SI).

use serde::{Deserialize, Serialize};

/// A particle analysis: everything the solver needs.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ParticleModel {
    pub heading: String,
    pub materials: Vec<ParticleMaterial>,
    /// What is filled with material at the start.
    pub bodies: Vec<MaterialBody>,
    /// Rigid bodies the material meets.
    pub walls: Vec<RigidWall>,
    pub domain: Domain,
    pub gravity: [f64; 3],
    pub control: ParticleControl,
    /// Whether and how the temperature is computed; `None` keeps every particle at the
    /// temperature of its body.
    #[serde(default)]
    pub heat: Option<Heat>,
}

/// How heat moves in a particle analysis: it is made by the flow and at the walls,
/// conducted through the material and exchanged with walls that have a temperature.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Heat {
    /// The heat transfer coefficient between the material and a wall with a temperature
    /// (W/(m² K)).
    pub wall_transfer: f64,
}

/// The thermal properties of a material.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ParticleThermal {
    /// W/(m K).
    pub conductivity: f64,
    /// J/(kg K).
    pub specific_heat: f64,
}

/// The value of a table of (x, value) points at `x`: linear in between, held beyond the
/// ends; 1 for an empty table.
pub fn table_value(points: &[[f64; 2]], x: f64) -> f64 {
    let (Some(first), Some(last)) = (points.first(), points.last()) else {
        return 1.0;
    };
    if x <= first[0] {
        return first[1];
    }
    if x >= last[0] {
        return last[1];
    }
    let k = points.partition_point(|p| p[0] <= x).max(1);
    let ([x0, y0], [x1, y1]) = (points[k - 1], points[k]);
    if x1 > x0 {
        y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    } else {
        y1
    }
}

/// A material of a particle analysis.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ParticleMaterial {
    pub name: String,
    pub density: f64,
    pub law: ParticleLaw,
    /// For the temperature: without, the material keeps its temperature.
    #[serde(default)]
    pub thermal: Option<ParticleThermal>,
    /// The factor on the yield stress, the consistency and the viscosity over the
    /// temperature, as (temperature, factor) points ([`table_value`]); empty is 1.
    #[serde(default)]
    pub flow_factor: Vec<[f64; 2]>,
}

/// How a material responds.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum ParticleLaw {
    /// Hyperelastic at large strains (Hencky: linear in the logarithmic strain).
    Elastic { young: f64, poisson: f64 },
    /// Elastic below a yield stress, flowing above it after Herschel-Bulkley: the shear
    /// stress in flow is `yield + consistency * rate^exponent` (Bingham for `exponent = 1`).
    /// `yield` is the yield stress in shear; `consistency` has the unit `Pa s^exponent`.
    ViscoPlastic {
        shear: f64,
        bulk: f64,
        yield_stress: f64,
        consistency: f64,
        exponent: f64,
    },
    /// A weakly compressible Newtonian fluid that carries no tension.
    Fluid { bulk: f64, viscosity: f64 },
    /// A dry granular material after the μ(I) rheology (Jop, Forterre & Pouliquen 2006):
    /// in flow the equivalent shear stress is `μ(I) p` with
    /// `μ(I) = friction + (friction_limit − friction) I / (inertial + I)` and the inertial
    /// number `I = γ̇ grain √(ρ/p)` (ρ the density of the material). It carries no tension.
    /// `friction` and `friction_limit` are friction coefficients (tangents of the angles),
    /// `grain` is the grain diameter.
    ///
    /// With `fine_grain` above zero the material is a mixture of two grain sizes, `grain`
    /// the coarse and `fine_grain` the fine one, `fine_fraction` the volume fraction of the
    /// fine grains at the start. Where it flows the fine grains sink through the coarse ones
    /// and the coarse rise (segregation), and both mix by diffusion; the mean grain diameter
    /// of the mixture enters `I`.
    Granular {
        shear: f64,
        bulk: f64,
        friction: f64,
        friction_limit: f64,
        grain: f64,
        inertial: f64,
        fine_grain: f64,
        fine_fraction: f64,
    },
}

impl ParticleLaw {
    /// The bulk and the shear modulus of its elastic part (zero shear for a fluid).
    pub fn moduli(&self) -> (f64, f64) {
        match *self {
            Self::Elastic { young, poisson } => (
                young / (3.0 * (1.0 - 2.0 * poisson)),
                young / (2.0 * (1.0 + poisson)),
            ),
            Self::ViscoPlastic { shear, bulk, .. } => (bulk, shear),
            Self::Fluid { bulk, .. } => (bulk, 0.0),
            Self::Granular { shear, bulk, .. } => (bulk, shear),
        }
    }
}

/// A closed surface of triangles: corner positions and the corners of each triangle,
/// counterclockwise seen from outside.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct TriangleSurface {
    pub points: Vec<[f64; 3]>,
    pub triangles: Vec<[u32; 3]>,
}

impl TriangleSurface {
    /// The surface of an axis-parallel box.
    pub fn cuboid(min: [f64; 3], max: [f64; 3]) -> Self {
        let p = |i: usize| {
            [
                if i & 1 == 0 { min[0] } else { max[0] },
                if i & 2 == 0 { min[1] } else { max[1] },
                if i & 4 == 0 { min[2] } else { max[2] },
            ]
        };
        let points = (0..8).map(p).collect();
        // Faces as quads of corner numbers (bit 0: x, bit 1: y, bit 2: z), outward.
        let quads = [
            [0, 2, 3, 1], // z min
            [4, 5, 7, 6], // z max
            [0, 1, 5, 4], // y min
            [2, 6, 7, 3], // y max
            [0, 4, 6, 2], // x min
            [1, 3, 7, 5], // x max
        ];
        let triangles = quads
            .iter()
            .flat_map(|q| [[q[0], q[1], q[2]], [q[0], q[2], q[3]]])
            .collect();
        Self { points, triangles }
    }

    /// The surface of a cylinder about the axis through `base` along `axis` (its length the
    /// cylinder's height), with `segments` facets around.
    pub fn cylinder(base: [f64; 3], axis: [f64; 3], radius: f64, segments: usize) -> Self {
        let length = norm(axis);
        let a = scale(axis, 1.0 / length);
        let helper = if a[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let u = normalize(cross(a, helper));
        let v = cross(a, u);
        let n = segments.max(3);
        let mut points = Vec::with_capacity(2 * n + 2);
        for end in [0.0, length] {
            for k in 0..n {
                let phi = std::f64::consts::TAU * k as f64 / n as f64;
                let (s, c) = phi.sin_cos();
                points.push(std::array::from_fn(|i| {
                    base[i] + end * a[i] + radius * (c * u[i] + s * v[i])
                }));
            }
        }
        let bottom = points.len() as u32;
        points.push(base);
        points.push(std::array::from_fn(|i| base[i] + axis[i]));
        let top = bottom + 1;
        let n = n as u32;
        let mut triangles = Vec::new();
        for k in 0..n {
            let k1 = (k + 1) % n;
            triangles.push([k, k1, n + k1]);
            triangles.push([k, n + k1, n + k]);
            triangles.push([bottom, k1, k]);
            triangles.push([top, n + k, n + k1]);
        }
        Self { points, triangles }
    }

    /// The smallest and largest coordinates of its points.
    pub fn bounds(&self) -> Option<([f64; 3], [f64; 3])> {
        let first = *self.points.first()?;
        Some(self.points.iter().fold((first, first), |(lo, hi), p| {
            (
                std::array::from_fn(|i| lo[i].min(p[i])),
                std::array::from_fn(|i| hi[i].max(p[i])),
            )
        }))
    }

    /// The surface with its orientation turned around (normals inward).
    pub fn flipped(mut self) -> Self {
        for t in &mut self.triangles {
            t.swap(1, 2);
        }
        self
    }
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn normalize(a: [f64; 3]) -> [f64; 3] {
    scale(a, 1.0 / norm(a))
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// A region filled with material at the start.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MaterialBody {
    pub name: String,
    /// Index into [`ParticleModel::materials`].
    pub material: usize,
    pub shape: TriangleSurface,
    pub velocity: [f64; 3],
    /// The temperature at the start.
    #[serde(default)]
    pub temperature: f64,
    /// The acceleration of gravity on this body, on top of [`ParticleModel::gravity`]: the
    /// gravity loads that select it.
    #[serde(default)]
    pub gravity: [f64; 3],
}

/// How material slides along a rigid surface.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum WallLaw {
    /// No slip: the material moves with the wall.
    #[default]
    Stick,
    /// Free slip: only the motion into the wall is stopped.
    Slip,
    /// Coulomb friction with this coefficient.
    Coulomb { friction: f64 },
}

/// The motion of a rigid body: a rotation about an axis through `center` plus a
/// translation, `angular s(t)` and `velocity s(t)` at time `t`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RigidMotion {
    /// The translation per unit of `s`.
    pub velocity: [f64; 3],
    /// A point on the axis of rotation at the start; it moves with the translation.
    pub center: [f64; 3],
    /// The rotation vector per unit of `s` (rad): its direction is the axis.
    pub angular: [f64; 3],
    /// `s` over time as (time, s) points, linear in between and held beyond the ends,
    /// like the amplitude of a displacement; empty is `s = t`, a constant velocity.
    #[serde(default)]
    pub amplitude: Vec<[f64; 2]>,
}

impl RigidMotion {
    pub fn is_resting(&self) -> bool {
        (self.velocity == [0.0; 3] && self.angular == [0.0; 3])
            || (!self.amplitude.is_empty() && self.amplitude.iter().all(|p| p[1] == 0.0))
    }

    /// `ds/dt` at time `t`: the factor on `velocity` and `angular` that gives the
    /// velocities (of the segment that starts at `t` at a kink).
    pub fn factor(&self, t: f64) -> f64 {
        let points = &self.amplitude;
        if points.is_empty() {
            return 1.0;
        }
        let k = points.partition_point(|p| p[0] <= t);
        if k == 0 || k == points.len() {
            return 0.0;
        }
        let ([t0, s0], [t1, s1]) = (points[k - 1], points[k]);
        if t1 > t0 { (s1 - s0) / (t1 - t0) } else { 0.0 }
    }

    /// `s` at time `t`.
    pub fn moved_time(&self, t: f64) -> f64 {
        let points = &self.amplitude;
        let (Some(first), Some(last)) = (points.first(), points.last()) else {
            return t;
        };
        if t <= first[0] {
            return first[1];
        }
        if t >= last[0] {
            return last[1];
        }
        let k = points.partition_point(|p| p[0] <= t).max(1);
        let ([t0, s0], [t1, s1]) = (points[k - 1], points[k]);
        if t1 > t0 {
            s0 + (s1 - s0) * (t - t0) / (t1 - t0)
        } else {
            s1
        }
    }

    /// Where the point `p` of the body (where it is at the start) is at time `t`.
    pub fn place(&self, p: [f64; 3], t: f64) -> [f64; 3] {
        let s = self.moved_time(t);
        let turn: [f64; 3] = std::array::from_fn(|a| self.angular[a] * s);
        let arm: [f64; 3] = std::array::from_fn(|a| p[a] - self.center[a]);
        let turned = rotate(turn, arm);
        std::array::from_fn(|a| self.center[a] + self.velocity[a] * s + turned[a])
    }
}

/// `v` turned by the rotation vector `turn` (Rodrigues).
fn rotate(turn: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    let angle = (turn[0] * turn[0] + turn[1] * turn[1] + turn[2] * turn[2]).sqrt();
    if angle < 1e-300 {
        return v;
    }
    let k = turn.map(|c| c / angle);
    let (sin, cos) = angle.sin_cos();
    let kv = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
    let cross = [
        k[1] * v[2] - k[2] * v[1],
        k[2] * v[0] - k[0] * v[2],
        k[0] * v[1] - k[1] * v[0],
    ];
    std::array::from_fn(|a| v[a] * cos + cross[a] * sin + k[a] * kv * (1.0 - cos))
}

/// A rigid body: its closed surface (the solid is inside), how material slides on it, and
/// how it moves.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigidWall {
    pub name: String,
    pub shape: TriangleSurface,
    pub law: WallLaw,
    pub motion: RigidMotion,
    /// The temperature of the wall, which it exchanges heat with the material at
    /// ([`Heat::wall_transfer`]); `None` exchanges none.
    #[serde(default)]
    pub temperature: Option<f64>,
}

/// What happens at a side of the box the material moves in.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Side {
    /// A resting wall with this law.
    Wall(WallLaw),
    /// Material leaves the model through it and counts as outflow.
    Open,
}

impl Default for Side {
    fn default() -> Self {
        Self::Wall(WallLaw::Slip)
    }
}

/// The box the material moves in, and the resolution.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Domain {
    pub min: [f64; 3],
    pub max: [f64; 3],
    /// The size of a cell of the background grid; particles start two to a cell in each
    /// direction.
    pub cell: f64,
    /// The sides: `[x min, x max, y min, y max, z min, z max]`.
    pub sides: [Side; 6],
    /// Axes along which the box repeats (the sides of those axes are ignored). Along a
    /// periodic axis the box must span a whole number of cells, at least three (one along
    /// z in a 2D space).
    pub periodic: [bool; 3],
    /// A 3D box, or the slab of a 2D model: one cell thick along z, periodic there.
    #[serde(default)]
    pub space: Space,
}

/// What the box stands for.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Space {
    #[default]
    Solid,
    /// A long body across which the material flows in the x-y plane: the slab one cell
    /// thick along z repeats. Forces on walls are per unit length along z.
    PlaneStrain,
    /// A body of revolution about the y axis, x the radius: the slab stands for the whole
    /// ring at its radius. The box starts at the axis, x = 0. Forces on walls are of the
    /// whole ring, along the axis only.
    Axisymmetric,
}

impl Default for Domain {
    fn default() -> Self {
        Self {
            min: [0.0; 3],
            max: [1.0; 3],
            cell: 0.1,
            sides: [Side::default(); 6],
            periodic: [false; 3],
            space: Space::Solid,
        }
    }
}

/// How long and how the solver runs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ParticleControl {
    pub end_time: f64,
    /// How many frames are written, evenly over the time, besides the start.
    pub frames: usize,
    /// The speed the elastic waves are slowed to (by a smaller bulk modulus), or `None` for
    /// the material's own. Allowed while the material moves much slower than it (the
    /// report gives the largest Mach number); like mass scaling in an explicit step.
    pub sound_speed: Option<f64>,
    /// Whether the volume ratio is averaged over the cells (F-bar), against volumetric
    /// locking and pressure noise.
    pub smooth_volume: bool,
    /// The Courant number of the time step.
    pub courant: f64,
}

impl Default for ParticleControl {
    fn default() -> Self {
        Self {
            end_time: 1.0,
            frames: 50,
            sound_speed: None,
            smooth_volume: true,
            courant: 0.3,
        }
    }
}

/// The results of a particle analysis.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ParticleResults {
    pub frames: Vec<ParticleFrame>,
    pub history: ParticleHistory,
    pub report: ParticleReport,
}

/// The names of the fields of a frame, in the order of [`ParticleFrame::fields`].
pub const PARTICLE_FIELDS: [&str; 8] = [
    "Pressure",
    "Shear stress",
    "Plastic strain",
    "Shear rate",
    "Volume ratio",
    "Temperature",
    "Fine fraction",
    "Body",
];

/// The particles at one time.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ParticleFrame {
    pub time: f64,
    /// The number of each particle, counted from 0 in the order of the first frame:
    /// particles that leave the model are missing from the frames after.
    pub ids: Vec<u32>,
    pub positions: Vec<[f32; 3]>,
    pub velocities: Vec<[f32; 3]>,
    /// One value per particle for each of [`PARTICLE_FIELDS`].
    pub fields: Vec<Vec<f32>>,
}

/// Quantities of the whole model over the time.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ParticleHistory {
    pub time: Vec<f64>,
    pub time_step: Vec<f64>,
    pub kinetic: Vec<f64>,
    /// Potential energy of gravity, relative to the start.
    pub potential: Vec<f64>,
    /// Elastic energy stored in the material.
    pub elastic: Vec<f64>,
    /// Energy dissipated by plastic flow and viscosity, summed from the start.
    pub dissipated: Vec<f64>,
    /// Energy dissipated where walls stop or rub the material, summed from the start.
    pub wall_loss: Vec<f64>,
    /// Work the moving walls did on the material, summed from the start.
    #[serde(default)]
    pub wall_work: Vec<f64>,
    /// Energy that left with material through open sides, summed from the start.
    pub outflow_energy: Vec<f64>,
    /// What the balance misses, relative to the largest energy so far: the numerical
    /// dissipation of the transfers.
    pub balance_error: Vec<f64>,
    /// Mass in the model and mass that left it.
    pub mass: Vec<f64>,
    pub outflow: Vec<f64>,
    /// The largest particle speed over the wave speed.
    pub mach: Vec<f64>,
    /// Per body of material, in the order of [`ParticleModel::bodies`]: its mass in the
    /// model and its temperature.
    #[serde(default)]
    pub bodies: Vec<BodyHistory>,
    /// Per wall (the walls of the model, then the six sides of the box): the force the
    /// material exerts on it and the moment of that force about the wall's center of
    /// motion where it is (about the origin for the sides).
    pub walls: Vec<WallHistory>,
}

/// One body of material over the time.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BodyHistory {
    pub name: String,
    /// The mass of its particles still in the model.
    pub mass: Vec<f64>,
    /// The mean (by mass) and the highest temperature of its particles.
    pub temperature_mean: Vec<f64>,
    pub temperature_max: Vec<f64>,
}

/// The load of one rigid surface over the time.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct WallHistory {
    pub name: String,
    /// The heat the wall gave the material, by transfer and by friction, summed from the
    /// start.
    #[serde(default)]
    pub heat: Vec<f64>,
    pub force: Vec<[f64; 3]>,
    pub moment: Vec<[f64; 3]>,
    /// The work the wall did on the material, summed from the start.
    #[serde(default)]
    pub work: Vec<f64>,
}

/// What the solver tells about the run.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ParticleReport {
    pub particles: usize,
    pub cells: [usize; 3],
    pub steps: usize,
    /// The largest and smallest time step.
    pub time_step: (f64, f64),
    /// The factor the wave speed was slowed by (1 for none).
    pub sound_speed_factor: f64,
    pub max_mach: f64,
    pub seconds: f64,
    /// Whether the run was stopped before its end time.
    pub stopped: bool,
    /// Remarks for the user, such as the slowed wave speed.
    pub notes: Vec<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_motion_follows_its_amplitude() {
        let motion = RigidMotion {
            velocity: [1.0, 0.0, 0.0],
            center: [0.0; 3],
            angular: [0.0, 0.0, std::f64::consts::PI],
            amplitude: vec![[0.0, 0.0], [1.0, 0.5], [2.0, 1.5]],
        };
        // Half speed for a second, then full speed, then held.
        assert_eq!(motion.factor(0.5), 0.5);
        assert_eq!(motion.factor(1.5), 1.0);
        assert_eq!(motion.factor(3.0), 0.0);
        assert!((motion.moved_time(1.5) - 1.0).abs() < 1e-12);
        assert_eq!(motion.moved_time(3.0), 1.5);
        // At t = 2 the body turned by 1.5 pi and moved by 1.5.
        let p = motion.place([1.0, 0.0, 0.0], 2.0);
        assert!(
            (p[0] - 1.5).abs() < 1e-12 && (p[1] + 1.0).abs() < 1e-12,
            "{p:?}"
        );
        assert!(!motion.is_resting());
        assert!(RigidMotion::default().is_resting());
    }
}
