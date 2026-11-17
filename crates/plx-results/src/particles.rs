//! The results of a particle analysis as results of the built-in solver ([`SolverResults`]),
//! so that the Results workspace shows them like any other: every particle is a cube of
//! the particle spacing at its start (an eight-node hexahedron), the frames are increments
//! whose displacements move the cubes to the particles, and the fields of the particles
//! are nodal fields of their cubes. A particle that left the model shrinks to a point where
//! it left, without values. The halves of a split particle are smaller cubes; a half made
//! during the run has its element after the walls, at its size where it was made, shrunk
//! to a point there in the frames before. The walls are triangles after the particles, a part each, that
//! move as the walls do, without values. The energies and the loads of the walls are history output.
//!
//! In a 2D model (plane strain or axisymmetric) a particle is a square in the x-y plane (a
//! four-node plane element) and a wall the triangles of its face in the plane.

use plxs_core::particles::{PARTICLE_FIELDS, ParticleFrame, ParticleHistory, ParticleModel, Space};
use plxs_core::results::{HistoryBlock, IncrementKind, ResultField, ResultsIncrement};
use plxs_core::{Model, element::ElementType};

/// The field of the quantities of the particles, beside `DISP` and `VELO`.
pub const FIELD: &str = "PFLOW";

/// The components of [`FIELD`], in the order of [`PARTICLE_FIELDS`] (without the
/// temperature, a field of its own, and the body, which the parts show).
pub const COMPONENTS: [&str; 5] = ["PRESS", "SHEAR", "PEEQ", "RATE", "J"];

/// The name of the set the energies of the model are printed for.
pub const MODEL_SET: &str = "PARTICLES";

/// Corner offsets of a cube in the node order of an eight-node hexahedron, in half edges.
const CORNERS: [[f64; 3]; 8] = [
    [-1.0, -1.0, -1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, -1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [1.0, -1.0, 1.0],
    [1.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0],
];

/// Corner offsets of a square in the x-y plane, in half edges.
const SQUARE: [[f64; 3]; 4] = [
    [-1.0, -1.0, 0.0],
    [1.0, -1.0, 0.0],
    [1.0, 1.0, 0.0],
    [-1.0, 1.0, 0.0],
];

/// The corners of the element of a particle and its type: a cube in 3D, a square in 2D.
fn shape(model: &ParticleModel) -> (&'static [[f64; 3]], ElementType) {
    match model.domain.space {
        Space::Solid => (&CORNERS, ElementType::C3D8),
        Space::PlaneStrain => (&SQUARE, ElementType::CPE4),
        Space::Axisymmetric => (&SQUARE, ElementType::CAX4),
    }
}

/// Where a point of the model is shown: in 2D in the plane z = 0.
fn shown(model: &ParticleModel, p: [f64; 3]) -> [f64; 3] {
    match model.domain.space {
        Space::Solid => p,
        _ => [p[0], p[1], 0.0],
    }
}

/// The index of the temperature in [`ParticleFrame::fields`].
const TEMPERATURE: usize = COMPONENTS.len();

/// The index of the fine fraction of grain mixtures in [`ParticleFrame::fields`].
const FINE: usize = TEMPERATURE + 1;

/// The field of the fine fraction of grain mixtures, written when a material is one.
pub const FINE_FIELD: &str = "FINE";

/// The index of the body field in [`ParticleFrame::fields`].
fn body_field() -> usize {
    PARTICLE_FIELDS.len() - 1
}

/// The mesh of the particles at the start (`first`, the frame at time 0): a cube per
/// particle, its edge the particle spacing (half a cell), element `k + 1` with nodes
/// `8 k + 1` to `8 k + 8` (in 2D a square, `4 k + 1` to `4 k + 4`); one section per body,
/// so that the results have a part per filled part of the model, named like it.
pub fn particle_mesh(model: &ParticleModel, first: &ParticleFrame) -> Model {
    let half = 0.25 * model.domain.cell;
    let (corners, kind) = shape(model);
    let per = corners.len() as u32;
    let mut mesh = Model::default();
    mesh.heading = model.heading.clone();
    // The results make a part of each material: here one named like each body.
    for body in &model.bodies {
        mesh.materials.push(plxs_core::material::Material {
            name: body.name.clone(),
            ..Default::default()
        });
        mesh.sections.push(plxs_core::section::Section {
            elset: body.name.clone(),
            kind: plxs_core::section::SectionKind::Solid {
                material: body.name.clone(),
                thickness: None,
                orientation: None,
            },
        });
    }
    let bodies = first.fields.get(body_field());
    for (k, x) in first.positions.iter().enumerate() {
        let id = first.ids.get(k).copied().unwrap_or(k as u32);
        let nodes: Vec<u32> = (0..per).map(|c| per * id + c + 1).collect();
        for (c, &node) in nodes.iter().enumerate() {
            let p = std::array::from_fn(|a| f64::from(x[a]) + half * corners[c][a]);
            mesh.nodes.insert(node, shown(model, p));
        }
        let body = bodies.and_then(|b| b.get(k)).map_or(0, |&b| b as usize);
        mesh.elements.push(plxs_core::element::Element {
            id: id + 1,
            kind,
            nodes,
            section: Some(body as u32),
        });
    }
    let mut node = wall_base(model, first);
    let mut element = node;
    for (w, wall) in model.walls.iter().enumerate() {
        mesh.materials.push(plxs_core::material::Material {
            name: wall.name.clone(),
            ..Default::default()
        });
        mesh.sections.push(plxs_core::section::Section {
            elset: wall.name.clone(),
            kind: plxs_core::section::SectionKind::Membrane {
                material: wall.name.clone(),
                thickness: 0.0,
                offset: 0.0,
            },
        });
        let first_node = node + 1;
        for p in &wall.shape.points {
            node += 1;
            mesh.nodes.insert(node, shown(model, *p));
        }
        // In 2D the wall is extruded along z: its face in the plane is the end at the
        // lowest z.
        let bottom = (wall.shape.points.iter()).fold(f64::INFINITY, |z, p| z.min(p[2]));
        let in_plane = |t: &[u32; 3]| {
            model.domain.space == Space::Solid
                || t.iter()
                    .all(|&i| wall.shape.points[i as usize][2] == bottom)
        };
        for t in wall.shape.triangles.iter().filter(|t| in_plane(t)) {
            element += 1;
            mesh.elements.push(plxs_core::element::Element {
                id: element,
                kind: ElementType::M3D3,
                nodes: t.iter().map(|&i| first_node + i).collect(),
                section: Some((model.bodies.len() + w) as u32),
            });
        }
    }
    mesh.rebuild_indices();
    mesh
}

/// The first node and element number of the walls, less one: after every particle.
fn wall_base(model: &ParticleModel, first: &ParticleFrame) -> u32 {
    let ids = (0..first.positions.len()).map(|k| first.ids.get(k).copied().unwrap_or(k as u32));
    let per = shape(model).0.len() as u32;
    ids.max().map_or(0, |id| per * (id + 1))
}

/// The number of the nodes of the walls of [`particle_mesh`], in order.
fn wall_nodes(model: &ParticleModel, first: &ParticleFrame) -> Vec<u32> {
    let count: usize = model.walls.iter().map(|w| w.shape.points.len()).sum();
    let base = wall_base(model, first);
    (1..=count as u32).map(|k| base + k).collect()
}

/// The first node and element number of the particles made during the run, less one: after
/// the walls.
fn born_base(model: &ParticleModel, first: &ParticleFrame) -> u32 {
    let points: usize = model.walls.iter().map(|w| w.shape.points.len()).sum();
    let triangles: usize = model.walls.iter().map(|w| w.shape.triangles.len()).sum();
    wall_base(model, first) + points.max(triangles) as u32
}

/// What the frames of a run need of the frames before them: the start of every particle
/// and where it was last seen.
#[derive(Clone, Debug)]
pub struct FrameTrack {
    half: f64,
    /// The corners of the element of a particle.
    corners: &'static [[f64; 3]],
    ids: Vec<u32>,
    /// The node before the first of each particle's element.
    firsts: Vec<u32>,
    /// Half the edge of each particle's element in the mesh.
    halves: Vec<f64>,
    start: Vec<[f32; 3]>,
    last: Vec<[f32; 3]>,
    /// The particles made during the run (halves of split ones): their index in `ids` and
    /// their body.
    born: Vec<(usize, u32)>,
    /// The node before the first of the particles made during the run.
    born_base: u32,
    /// The index of each particle id in `start`.
    index: std::collections::HashMap<u32, usize>,
    /// The nodes of the walls, each with where it starts and the motion of its wall.
    walls: Vec<u32>,
    wall_points: Vec<([f64; 3], usize)>,
    motions: Vec<plxs_core::particles::RigidMotion>,
    /// The temperature of each wall, when the run computes the temperature (`None`: it
    /// does not, and the results have no temperature).
    temperatures: Option<Vec<Option<f64>>>,
    /// Whether a material is a mixture of two grain sizes (the results then have the fine
    /// fraction).
    mixtures: bool,
}

impl FrameTrack {
    /// The track from the frame at time 0.
    pub fn new(model: &ParticleModel, first: &ParticleFrame) -> Self {
        let ids: Vec<u32> = (0..first.positions.len())
            .map(|k| first.ids.get(k).copied().unwrap_or(k as u32))
            .collect();
        let per = shape(model).0.len() as u32;
        Self {
            half: 0.25 * model.domain.cell,
            corners: shape(model).0,
            index: ids.iter().enumerate().map(|(k, &id)| (id, k)).collect(),
            firsts: ids.iter().map(|&id| per * id).collect(),
            halves: vec![0.25 * model.domain.cell; ids.len()],
            born: Vec::new(),
            born_base: born_base(model, first),
            ids,
            start: first.positions.clone(),
            last: first.positions.clone(),
            walls: wall_nodes(model, first),
            wall_points: (model.walls.iter().enumerate())
                .flat_map(|(w, wall)| wall.shape.points.iter().map(move |&p| (p, w)))
                .collect(),
            motions: model.walls.iter().map(|w| w.motion.clone()).collect(),
            temperatures: (model.heat.is_some())
                .then(|| model.walls.iter().map(|w| w.temperature).collect()),
            mixtures: model.materials.iter().any(|m| {
                matches!(
                    m.law,
                    plxs_core::particles::ParticleLaw::Granular { fine_grain, .. } if fine_grain > 0.0
                )
            }),
        }
    }

    /// Half the edge of the element of particle `j` of `frame`, by its size: in 2D its area
    /// in the plane (which grows near the axis of a body of revolution and halves when it
    /// splits), in 3D the volume of the particle it was made as that it keeps.
    fn size(&self, frame: &ParticleFrame, j: usize) -> f64 {
        let share = frame.shares.get(j).map_or(1.0, |&s| f64::from(s));
        let root = if self.corners.len() == SQUARE.len() {
            share.sqrt()
        } else {
            share.cbrt()
        };
        self.half * root
    }

    /// Whether particles were made during the run, which then need [`FrameTrack::born_mesh`]
    /// and [`FrameTrack::complete`].
    pub fn has_born(&self) -> bool {
        !self.born.is_empty()
    }

    /// Adds the elements of the particles made during the run to the mesh of
    /// [`particle_mesh`]: at their size and place where they were made.
    pub fn born_mesh(&self, model: &ParticleModel, mesh: &mut Model) {
        let (corners, kind) = shape(model);
        let per = corners.len() as u32;
        for (b, &(k, body)) in self.born.iter().enumerate() {
            let first = self.firsts[k];
            let nodes: Vec<u32> = (1..=per).map(|c| first + c).collect();
            for (c, &node) in nodes.iter().enumerate() {
                let p = std::array::from_fn(|a| {
                    f64::from(self.start[k][a]) + self.halves[k] * corners[c][a]
                });
                mesh.nodes.insert(node, shown(model, p));
            }
            mesh.elements.push(plxs_core::element::Element {
                id: self.born_base + b as u32 + 1,
                kind,
                nodes,
                section: Some(body),
            });
        }
        mesh.rebuild_indices();
    }

    /// Gives an increment written before particles were made the nodes of those: shrunk to
    /// a point where they were made, without values.
    pub fn complete(&self, increment: &mut ResultsIncrement) {
        for field in &mut increment.fields {
            let known: std::collections::HashSet<u32> = (field.nodes.iter())
                .copied()
                .filter(|&n| n > self.born_base)
                .collect();
            for &(k, _) in &self.born {
                let first = self.firsts[k];
                if known.contains(&(first + 1)) {
                    continue;
                }
                for (c, corner) in self.corners.iter().enumerate() {
                    field.nodes.push(first + c as u32 + 1);
                    for (a, column) in field.values.iter_mut().enumerate() {
                        column.push(if field.name == "DISP" {
                            -self.halves[k] * corner[a]
                        } else {
                            f64::NAN
                        });
                    }
                }
            }
        }
    }

    /// The increment of a frame, numbered `increment` in step `step`: the displacements,
    /// velocities and particle quantities of every node of the mesh of [`particle_mesh`].
    pub fn increment(
        &mut self,
        frame: &ParticleFrame,
        step: u32,
        increment: u32,
    ) -> ResultsIncrement {
        // Halves of split particles seen for the first time are new particles.
        let body = frame.fields.get(body_field());
        for (j, x) in frame.positions.iter().enumerate() {
            let id = frame.ids.get(j).copied().unwrap_or(j as u32);
            if !self.index.contains_key(&id) {
                let per = self.corners.len() as u32;
                let k = self.ids.len();
                self.index.insert(id, k);
                self.ids.push(id);
                self.firsts
                    .push(self.born_base + per * self.born.len() as u32);
                self.halves.push(self.size(frame, j));
                self.start.push(*x);
                self.last.push(*x);
                let body = body.and_then(|b| b.get(j)).map_or(0, |&b| b as u32);
                self.born.push((k, body));
            }
        }
        let n = self.start.len();
        let mut at = vec![None; n];
        for (j, x) in frame.positions.iter().enumerate() {
            let id = frame.ids.get(j).copied().unwrap_or(j as u32);
            if let Some(&k) = self.index.get(&id) {
                at[k] = Some(j);
                self.last[k] = *x;
            }
        }
        let per = self.corners.len() as u32;
        let nodes: Vec<u32> = (self.firsts.iter())
            .flat_map(|&first| (1..=per).map(move |c| first + c))
            .chain(self.walls.iter().copied())
            .collect();
        let size = nodes.len();
        let mut disp: Vec<Vec<f64>> = (0..3).map(|_| Vec::with_capacity(size)).collect();
        let mut velo: Vec<Vec<f64>> = (0..3).map(|_| Vec::with_capacity(size)).collect();
        let mut quantities: Vec<Vec<f64>> = (0..COMPONENTS.len())
            .map(|_| Vec::with_capacity(size))
            .collect();
        let mut temperature = Vec::with_capacity(size);
        let mut fine = Vec::with_capacity(if self.mixtures { size } else { 0 });
        for (k, seen) in at.iter().enumerate() {
            let moved: [f64; 3] =
                std::array::from_fn(|a| f64::from(self.last[k][a]) - f64::from(self.start[k][a]));
            // A particle that left shrinks to the point it was last seen at.
            let half = seen.map_or(0.0, |j| self.size(frame, j));
            for corner in self.corners {
                for a in 0..3 {
                    disp[a].push(moved[a] + (half - self.halves[k]) * corner[a]);
                    velo[a].push(seen.map_or(f64::NAN, |j| f64::from(frame.velocities[j][a])));
                }
                for (c, values) in quantities.iter_mut().enumerate() {
                    let value = seen.and_then(|j| frame.fields.get(c)?.get(j).copied());
                    values.push(value.map_or(f64::NAN, f64::from));
                }
                let value = seen.and_then(|j| frame.fields.get(TEMPERATURE)?.get(j).copied());
                temperature.push(value.map_or(f64::NAN, f64::from));
                if self.mixtures {
                    let value = seen.and_then(|j| frame.fields.get(FINE)?.get(j).copied());
                    fine.push(value.map_or(f64::NAN, f64::from));
                }
            }
        }
        // The walls move rigidly and carry no values.
        let t = frame.time;
        for &(start, index) in &self.wall_points {
            let motion = &self.motions[index];
            let at = motion.place(start, t);
            let center = motion.place(motion.center, t);
            let factor = motion.factor(t);
            let arm: [f64; 3] = std::array::from_fn(|a| at[a] - center[a]);
            let (v, w) = (motion.velocity, motion.angular);
            let turn = [
                w[1] * arm[2] - w[2] * arm[1],
                w[2] * arm[0] - w[0] * arm[2],
                w[0] * arm[1] - w[1] * arm[0],
            ];
            for a in 0..3 {
                disp[a].push(at[a] - start[a]);
                velo[a].push(factor * (v[a] + turn[a]));
            }
            for values in &mut quantities {
                values.push(f64::NAN);
            }
            let wall = self.temperatures.as_ref().and_then(|t| t[index]);
            temperature.push(wall.unwrap_or(f64::NAN));
            if self.mixtures {
                fine.push(f64::NAN);
            }
        }
        let field = |name: &str, components: &[&str], values| ResultField {
            name: name.into(),
            components: components.iter().map(|c| (*c).to_string()).collect(),
            nodes: nodes.clone(),
            values,
        };
        ResultsIncrement {
            step,
            increment,
            kind: IncrementKind::Dynamic,
            value: frame.time,
            total_time: frame.time,
            nodal_diameter: None,
            fields: [
                field("DISP", &["D1", "D2", "D3"], disp),
                field("VELO", &["V1", "V2", "V3"], velo),
                field(FIELD, &COMPONENTS, quantities),
            ]
            .into_iter()
            .chain(
                (self.temperatures.is_some()).then(|| field("NDTEMP", &["T"], vec![temperature])),
            )
            .chain(
                self.mixtures
                    .then(|| field(FINE_FIELD, &["FINE"], vec![fine])),
            )
            .collect(),
        }
    }
}

/// What the history outputs of a Particle Flow step ask for; nothing else is written.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct HistoryRequest {
    /// The energies of the whole model, by CalculiX's names: `ALLIE` (elastic), `ALLKE`,
    /// `ALLWK` (the work of gravity and of the moving walls) and `ALLVD` (dissipated by
    /// flow and at the walls).
    pub energies: Vec<String>,
    pub walls: Vec<WallRequest>,
    pub temperatures: Vec<TemperatureRequest>,
}

/// The values of one wall, by its index in [`ParticleHistory::walls`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct WallRequest {
    pub wall: usize,
    /// The force of the material on it, the moment about its reference point and the work
    /// it did (`RF`).
    pub force: bool,
    /// The heat it gave the material (`RFL`).
    pub heat: bool,
}

/// The mean and highest temperature of some bodies of material, written under `set`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TemperatureRequest {
    pub set: String,
    /// Indices into [`ParticleHistory::bodies`].
    pub bodies: Vec<usize>,
}

impl HistoryRequest {
    /// Everything the run records, for tests and tools that want it all.
    pub fn all(history: &ParticleHistory) -> Self {
        Self {
            energies: ["ALLIE", "ALLKE", "ALLWK", "ALLVD"]
                .map(String::from)
                .to_vec(),
            walls: (0..history.walls.len())
                .map(|wall| WallRequest {
                    wall,
                    force: true,
                    heat: true,
                })
                .collect(),
            temperatures: vec![TemperatureRequest {
                set: MODEL_SET.into(),
                bodies: (0..history.bodies.len()).collect(),
            }],
        }
    }
}

/// The name of a wall as the history output names its set: without spaces.
pub fn set_name(wall: &str) -> String {
    wall.split_whitespace().collect::<Vec<_>>().join("_")
}

/// The history of a run as the history output of step `step`, as far as `request` asks for
/// it: the energies of the model (kinetic, elastic as internal, dissipated by flow and at the
/// walls as damping dissipation, the work of gravity and of the moving walls as external
/// work); per wall the force of the material on it, the moment of that force about the
/// wall's reference point (about the origin for the sides of the box), the work the wall did
/// on the material and the heat it gave; the temperature of bodies of material.
pub fn history_blocks(
    history: &ParticleHistory,
    request: &HistoryRequest,
    step: u32,
) -> Vec<HistoryBlock> {
    let mut blocks = Vec::new();
    let block =
        |increment: usize, quantity: &str, set: String, time: f64, values: Vec<f64>| HistoryBlock {
            step,
            increment: increment as u32 + 1,
            quantity: quantity.into(),
            set,
            time,
            rows: vec![(0, values)],
        };
    let at = |v: &[f64], i: usize| v.get(i).copied().unwrap_or(0.0);
    let asked = |name: &str| request.energies.iter().any(|e| e == name);
    for (i, &time) in history.time.iter().enumerate() {
        let energies = [
            (
                "ALLKE",
                "total model kinetic energy",
                at(&history.kinetic, i),
            ),
            (
                "ALLIE",
                "total model internal energy",
                at(&history.elastic, i),
            ),
            (
                "ALLVD",
                "total model damping dissipation",
                at(&history.dissipated, i) + at(&history.wall_loss, i),
            ),
            (
                "ALLWK",
                "total model external work",
                at(&history.wall_work, i) - at(&history.potential, i),
            ),
        ];
        for (name, quantity, value) in energies {
            if asked(name) {
                blocks.push(block(i, quantity, MODEL_SET.into(), time, vec![value]));
            }
        }
        for asked in &request.temperatures {
            let bodies = || asked.bodies.iter().filter_map(|&b| history.bodies.get(b));
            let mass: f64 = bodies().map(|b| at(&b.mass, i)).sum();
            let heat: f64 = bodies()
                .map(|b| at(&b.mass, i) * at(&b.temperature_mean, i))
                .sum();
            let hottest = bodies()
                .filter_map(|b| b.temperature_max.get(i).copied())
                .fold(f64::NEG_INFINITY, f64::max);
            if hottest.is_finite() {
                let mean = if mass > 0.0 { heat / mass } else { hottest };
                blocks.push(block(
                    i,
                    "particle temperature",
                    asked.set.clone(),
                    time,
                    vec![mean, hottest],
                ));
            }
        }
        for asked in &request.walls {
            let w = asked.wall;
            let Some(wall) = history.walls.get(w) else {
                continue;
            };
            let name = set_name(&wall.name);
            if asked.force {
                if let Some(force) = wall.force.get(i) {
                    blocks.push(block(
                        i,
                        "total force (fx,fy,fz)",
                        name.clone(),
                        time,
                        force.to_vec(),
                    ));
                }
                if let Some(&work) = wall.work.get(i) {
                    blocks.push(block(
                        i,
                        "total model external work",
                        name.clone(),
                        time,
                        vec![work],
                    ));
                }
                if let Some(moment) = wall.moment.get(i) {
                    // The moments as the rotation node of a reference point prints them.
                    let set = format!("INTERNAL_ROTATION-{}_{name}", w + 1);
                    blocks.push(block(
                        i,
                        "total force (fx,fy,fz)",
                        set,
                        time,
                        moment.to_vec(),
                    ));
                }
            }
            if asked.heat
                && let Some(&heat) = wall.heat.get(i)
            {
                blocks.push(block(i, "total wall heat", name.clone(), time, vec![heat]));
            }
        }
    }
    blocks
}

#[cfg(test)]
mod tests {
    use super::*;
    use plxs_core::particles::BodyHistory;

    fn model() -> ParticleModel {
        let mut model = ParticleModel::default();
        model.domain.cell = 0.4;
        model
            .materials
            .push(plxs_core::particles::ParticleMaterial {
                name: "Paste".into(),
                density: 1000.0,
                law: plxs_core::particles::ParticleLaw::Fluid {
                    bulk: 1e6,
                    viscosity: 1.0,
                },
                thermal: None,
                flow_tables: Default::default(),
            });
        model.bodies.push(plxs_core::particles::MaterialBody {
            name: "Body".into(),
            material: 0,
            shape: Default::default(),
            velocity: [0.0; 3],
            temperature: 0.0,
            gravity: [0.0; 3],
        });
        model
    }

    fn frame(time: f64, ids: Vec<u32>, positions: Vec<[f32; 3]>) -> ParticleFrame {
        let n = ids.len();
        ParticleFrame {
            time,
            ids,
            velocities: vec![[1.0, 0.0, 0.0]; n],
            positions,
            fields: vec![vec![2.0; n]; PARTICLE_FIELDS.len()],
            shares: Vec::new(),
        }
    }

    #[test]
    fn particles_are_cubes_that_move_and_shrink_when_gone() {
        let model = model();
        let first = frame(0.0, vec![0, 1], vec![[0.0; 3], [1.0, 0.0, 0.0]]);
        let mesh = particle_mesh(&model, &first);
        assert_eq!(mesh.elements.len(), 2);
        assert_eq!(mesh.nodes.ids.len(), 16);
        // Edge of a cube: half a cell.
        let c = &mesh.nodes.coordinates;
        assert!((c[1][0] - c[0][0] - 0.2).abs() < 1e-9);
        let mut track = FrameTrack::new(&model, &first);
        // Particle 0 moved by 0.5 in x, particle 1 left the model.
        let later = frame(1.0, vec![0], vec![[0.5, 0.0, 0.0]]);
        let increment = track.increment(&later, 1, 2);
        let disp = &increment.fields[0].values;
        assert!((0..8).all(|n| (disp[0][n] - 0.5).abs() < 1e-6));
        // The corners of particle 1 meet at its centre: its first corner moves by +h/2.
        assert!((disp[0][8] - 0.1).abs() < 1e-6 && (disp[0][9] + 0.1).abs() < 1e-6);
        let pflow = &increment.fields[2].values;
        assert_eq!(pflow[0][0], 2.0);
        assert!(pflow[0][8].is_nan());
    }

    /// A grain mixture writes its fine fraction as a field of its own; other materials
    /// do not.
    /// A particle that splits: the half that stays shrinks, the new half gets an element
    /// after the walls, at its size where it was made, and the frames before it have it
    /// shrunk to a point there, without values.
    #[test]
    fn the_halves_of_a_split_particle_are_smaller_cubes_from_their_split_on() {
        let model = model();
        let first = frame(0.0, vec![0], vec![[0.0; 3]]);
        let mut mesh = particle_mesh(&model, &first);
        let mut track = FrameTrack::new(&model, &first);
        let mut before = track.increment(&first, 1, 1);
        let mut split = frame(1.0, vec![0, 1], vec![[-0.1, 0.0, 0.0], [0.1, 0.0, 0.0]]);
        split.shares = vec![0.5, 0.5];
        let after = track.increment(&split, 1, 2);
        assert!(track.has_born());
        track.born_mesh(&model, &mut mesh);
        assert_eq!(mesh.elements.len(), 2);
        // The new element at its size: a cube of half the volume where it was made.
        let half = 0.1 * 0.5f64.cbrt();
        let born = &mesh.elements[1];
        let corner = mesh.nodes.coordinates[mesh.nodes.index(born.nodes[0]).unwrap() as usize];
        assert!((corner[0] - (0.1 - half)).abs() < 1e-6, "{corner:?}");
        // The half that stayed shrinks in place of its cube's half edge 0.1.
        let disp = &after.fields[0].values;
        assert!(
            (disp[0][0] - (-0.1 - (half - 0.1))).abs() < 1e-6,
            "{}",
            disp[0][0]
        );
        assert_eq!(after.fields[0].nodes.len(), 16);
        // The frame before gets the new half's nodes: shrunk to a point, without values.
        assert_eq!(before.fields[0].nodes.len(), 8);
        track.complete(&mut before);
        assert_eq!(before.fields[0].nodes, after.fields[0].nodes);
        assert!((before.fields[0].values[0][8] - half).abs() < 1e-6);
        assert!(before.fields[1].values[0][8].is_nan());
    }

    #[test]
    fn a_grain_mixture_has_the_fine_fraction() {
        let mut model = model();
        let first = frame(0.0, vec![0], vec![[0.0; 3]]);
        let plain = FrameTrack::new(&model, &first).increment(&first, 1, 1);
        assert!(plain.fields.iter().all(|f| f.name != FINE_FIELD));
        model.materials[0].law = plxs_core::particles::ParticleLaw::Granular {
            shear: 1e5,
            bulk: 1e6,
            friction: 0.4,
            friction_limit: 0.6,
            grain: 2e-3,
            inertial: 0.3,
            fine_grain: 1e-3,
            fine_fraction: 0.5,
        };
        let mut later = frame(1.0, vec![0], vec![[0.0; 3]]);
        later.fields[FINE] = vec![0.75];
        let increment = FrameTrack::new(&model, &first).increment(&later, 1, 2);
        let fine = increment
            .fields
            .iter()
            .find(|f| f.name == FINE_FIELD)
            .unwrap();
        assert_eq!(fine.components, vec!["FINE".to_string()]);
        assert!(fine.values[0][..8].iter().all(|&v| v == 0.75));
    }

    /// In 2D a particle is a square in the plane, a wall the end of its extrusion there.
    #[test]
    fn a_plane_model_has_squares_and_the_walls_in_the_plane() {
        let mut model = model();
        model.domain.space = Space::Axisymmetric;
        model.walls.push(plxs_core::particles::RigidWall {
            name: "Piston".into(),
            shape: plxs_core::particles::TriangleSurface {
                points: vec![
                    [0.0, 1.0, -1.0],
                    [1.0, 1.0, -1.0],
                    [0.0, 2.0, -1.0],
                    [0.0, 1.0, 2.0],
                    [1.0, 1.0, 2.0],
                    [0.0, 2.0, 2.0],
                ],
                triangles: vec![[0, 2, 1], [3, 4, 5]],
            },
            law: Default::default(),
            motion: plxs_core::particles::RigidMotion {
                velocity: [0.0, -0.5, 0.0],
                ..Default::default()
            },
            temperature: None,
        });
        let first = frame(0.0, vec![0, 1], vec![[0.1, 0.1, 0.2], [0.3, 0.1, 0.2]]);
        let mesh = particle_mesh(&model, &first);
        assert_eq!(mesh.elements.len(), 3);
        assert_eq!(mesh.elements[0].kind, ElementType::CAX4);
        assert_eq!(mesh.elements[1].nodes, vec![5, 6, 7, 8]);
        // The wall's triangle at its lowest z, its nodes after the squares' 8, in the plane.
        assert_eq!(mesh.elements[2].nodes, vec![9, 11, 10]);
        assert!(mesh.nodes.coordinates.iter().all(|c| c[2] == 0.0));
        let mut track = FrameTrack::new(&model, &first);
        let increment = track.increment(&frame(1.0, vec![0, 1], first.positions.clone()), 1, 2);
        let disp = &increment.fields[0].values;
        assert_eq!(disp[1].len(), 8 + 6);
        assert!(
            disp[1][8..].iter().all(|d| (d + 0.5).abs() < 1e-9),
            "{disp:?}"
        );
    }

    #[test]
    fn walls_are_resting_triangles_after_the_particles() {
        let mut model = model();
        model.walls.push(plxs_core::particles::RigidWall {
            name: "Die".into(),
            shape: plxs_core::particles::TriangleSurface {
                points: vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                triangles: vec![[0, 1, 2]],
            },
            law: Default::default(),
            motion: Default::default(),
            temperature: None,
        });
        let first = frame(0.0, vec![0, 1], vec![[0.0; 3], [1.0, 0.0, 0.0]]);
        let mesh = particle_mesh(&model, &first);
        assert_eq!(mesh.elements.len(), 3);
        assert_eq!(mesh.elements[2].nodes, vec![17, 18, 19]);
        assert_eq!(mesh.elements[2].section, Some(1));
        let mut track = FrameTrack::new(&model, &first);
        let increment = track.increment(&first, 1, 1);
        assert_eq!(increment.fields[0].nodes.len(), 19);
        assert_eq!(increment.fields[0].values[0].len(), 19);
        assert!(increment.fields[2].values[0][18].is_nan());
        // Turned by a quarter about the z axis through the first point, over a second.
        model.walls[0].motion.angular = [0.0, 0.0, std::f64::consts::FRAC_PI_2];
        let mut track = FrameTrack::new(&model, &first);
        let increment = track.increment(&frame(1.0, vec![0, 1], first.positions.clone()), 1, 2);
        let (disp, velo) = (&increment.fields[0].values, &increment.fields[1].values);
        // Node 18 starts at (1, 0, 0) and ends at (0, 1, 0), moving at pi/2 in -x.
        assert!((disp[0][17] + 1.0).abs() < 1e-9 && (disp[1][17] - 1.0).abs() < 1e-9);
        assert!((velo[0][17] + std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn the_history_writes_what_is_asked_and_names_walls_as_sets() {
        let wall = |name: &str| plxs_core::particles::WallHistory {
            name: name.into(),
            force: vec![[1.0, 2.0, 3.0]],
            moment: vec![[0.0, 0.0, 4.0]],
            work: vec![5.0],
            heat: vec![6.0],
        };
        let body = |name: &str, mass: f64, mean: f64, max: f64| BodyHistory {
            name: name.into(),
            mass: vec![mass],
            temperature_mean: vec![mean],
            temperature_max: vec![max],
        };
        let history = ParticleHistory {
            time: vec![0.5],
            kinetic: vec![1.0],
            walls: vec![wall("Blade"), wall("Box X min")],
            bodies: vec![
                body("Paste", 1.0, 20.0, 30.0),
                body("Dough", 3.0, 40.0, 50.0),
            ],
            ..Default::default()
        };
        // Nothing asked: nothing written.
        assert!(history_blocks(&history, &HistoryRequest::default(), 1).is_empty());
        let request = HistoryRequest {
            energies: vec!["ALLKE".into()],
            walls: vec![WallRequest {
                wall: 1,
                force: true,
                heat: false,
            }],
            temperatures: vec![TemperatureRequest {
                set: "Mix".into(),
                bodies: vec![0, 1],
            }],
        };
        let blocks = history_blocks(&history, &request, 1);
        let quantities: Vec<(&str, &str)> = (blocks.iter())
            .map(|b| (b.quantity.as_str(), b.set.as_str()))
            .collect();
        assert_eq!(
            quantities,
            [
                ("total model kinetic energy", MODEL_SET),
                ("particle temperature", "Mix"),
                ("total force (fx,fy,fz)", "Box_X_min"),
                ("total model external work", "Box_X_min"),
                ("total force (fx,fy,fz)", "INTERNAL_ROTATION-2_Box_X_min"),
            ]
        );
        // The mean by mass over both bodies, the highest of both.
        assert_eq!(blocks[1].rows[0].1, vec![35.0, 50.0]);
        assert_eq!(blocks[3].rows[0].1, vec![5.0]);
    }
}
