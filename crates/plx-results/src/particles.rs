//! The results of a particle analysis as results of the built-in solver ([`SolverResults`]),
//! so that the Results workspace shows them like any other: every particle is a cube of
//! the particle spacing at its start (an eight-node hexahedron), the frames are increments
//! whose displacements move the cubes to the particles, and the fields of the particles
//! are nodal fields of their cubes. A particle that left the model shrinks to a point where
//! it left, without values. The walls are triangles after the particles, a part each, that
//! move as the walls do, without values. The energies and the loads of the walls are history output.

use plxs_core::particles::{PARTICLE_FIELDS, ParticleFrame, ParticleHistory, ParticleModel};
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

/// The index of the temperature in [`ParticleFrame::fields`].
const TEMPERATURE: usize = COMPONENTS.len();

/// The index of the body field in [`ParticleFrame::fields`].
fn body_field() -> usize {
    PARTICLE_FIELDS.len() - 1
}

/// The mesh of the particles at the start (`first`, the frame at time 0): a cube per
/// particle, its edge the particle spacing (half a cell), element `k + 1` with nodes
/// `8 k + 1` to `8 k + 8`; one section per material, so that the results have a part per
/// material.
pub fn particle_mesh(model: &ParticleModel, first: &ParticleFrame) -> Model {
    let half = 0.25 * model.domain.cell;
    let mut mesh = Model::default();
    mesh.heading = model.heading.clone();
    for material in &model.materials {
        mesh.materials.push(plxs_core::material::Material {
            name: material.name.clone(),
            ..Default::default()
        });
        mesh.sections.push(plxs_core::section::Section {
            elset: material.name.clone(),
            kind: plxs_core::section::SectionKind::Solid {
                material: material.name.clone(),
                thickness: None,
                orientation: None,
            },
        });
    }
    let bodies = first.fields.get(body_field());
    for (k, x) in first.positions.iter().enumerate() {
        let id = first.ids.get(k).copied().unwrap_or(k as u32);
        let nodes: Vec<u32> = (0..8).map(|c| 8 * id + c + 1).collect();
        for (c, &node) in nodes.iter().enumerate() {
            let p = std::array::from_fn(|a| f64::from(x[a]) + half * CORNERS[c][a]);
            mesh.nodes.insert(node, p);
        }
        let body = bodies.and_then(|b| b.get(k)).map_or(0, |&b| b as usize);
        let material = (model.bodies.get(body)).map_or(0, |b| b.material);
        mesh.elements.push(plxs_core::element::Element {
            id: id + 1,
            kind: ElementType::C3D8,
            nodes,
            section: Some(material as u32),
        });
    }
    let mut node = wall_base(first);
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
            mesh.nodes.insert(node, *p);
        }
        for t in &wall.shape.triangles {
            element += 1;
            mesh.elements.push(plxs_core::element::Element {
                id: element,
                kind: ElementType::M3D3,
                nodes: t.iter().map(|&i| first_node + i).collect(),
                section: Some((model.materials.len() + w) as u32),
            });
        }
    }
    mesh.rebuild_indices();
    mesh
}

/// The first node and element number of the walls, less one: after every particle.
fn wall_base(first: &ParticleFrame) -> u32 {
    let ids = (0..first.positions.len()).map(|k| first.ids.get(k).copied().unwrap_or(k as u32));
    ids.max().map_or(0, |id| 8 * (id + 1))
}

/// The number of the nodes of the walls of [`particle_mesh`], in order.
fn wall_nodes(model: &ParticleModel, first: &ParticleFrame) -> Vec<u32> {
    let count: usize = model.walls.iter().map(|w| w.shape.points.len()).sum();
    let base = wall_base(first);
    (1..=count as u32).map(|k| base + k).collect()
}

/// What the frames of a run need of the frames before them: the start of every particle
/// and where it was last seen.
#[derive(Clone, Debug)]
pub struct FrameTrack {
    half: f64,
    ids: Vec<u32>,
    start: Vec<[f32; 3]>,
    last: Vec<[f32; 3]>,
    /// The index of each particle id in `start`.
    index: std::collections::HashMap<u32, usize>,
    /// The nodes of the walls, each with where it starts and the motion of its wall.
    walls: Vec<u32>,
    wall_points: Vec<([f64; 3], usize)>,
    motions: Vec<plxs_core::particles::RigidMotion>,
    /// The temperature of each wall, when the run computes the temperature (`None`: it
    /// does not, and the results have no temperature).
    temperatures: Option<Vec<Option<f64>>>,
}

impl FrameTrack {
    /// The track from the frame at time 0.
    pub fn new(model: &ParticleModel, first: &ParticleFrame) -> Self {
        let ids: Vec<u32> = (0..first.positions.len())
            .map(|k| first.ids.get(k).copied().unwrap_or(k as u32))
            .collect();
        Self {
            half: 0.25 * model.domain.cell,
            index: ids.iter().enumerate().map(|(k, &id)| (id, k)).collect(),
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
        let n = self.start.len();
        let mut at = vec![None; n];
        for (j, x) in frame.positions.iter().enumerate() {
            let id = frame.ids.get(j).copied().unwrap_or(j as u32);
            if let Some(&k) = self.index.get(&id) {
                at[k] = Some(j);
                self.last[k] = *x;
            }
        }
        let nodes: Vec<u32> = (self.ids.iter())
            .flat_map(|&id| (0..8).map(move |c| 8 * id + c + 1))
            .chain(self.walls.iter().copied())
            .collect();
        let size = nodes.len();
        let mut disp: Vec<Vec<f64>> = (0..3).map(|_| Vec::with_capacity(size)).collect();
        let mut velo: Vec<Vec<f64>> = (0..3).map(|_| Vec::with_capacity(size)).collect();
        let mut quantities: Vec<Vec<f64>> = (0..COMPONENTS.len())
            .map(|_| Vec::with_capacity(size))
            .collect();
        let mut temperature = Vec::with_capacity(size);
        for (k, seen) in at.iter().enumerate() {
            let moved: [f64; 3] =
                std::array::from_fn(|a| f64::from(self.last[k][a]) - f64::from(self.start[k][a]));
            for corner in CORNERS {
                for a in 0..3 {
                    // A particle that left shrinks to the point it was last seen at.
                    let shrink = if seen.is_some() {
                        0.0
                    } else {
                        self.half * corner[a]
                    };
                    disp[a].push(moved[a] - shrink);
                    velo[a].push(seen.map_or(f64::NAN, |j| f64::from(frame.velocities[j][a])));
                }
                for (c, values) in quantities.iter_mut().enumerate() {
                    let value = seen.and_then(|j| frame.fields.get(c)?.get(j).copied());
                    values.push(value.map_or(f64::NAN, f64::from));
                }
                let value = seen.and_then(|j| frame.fields.get(TEMPERATURE)?.get(j).copied());
                temperature.push(value.map_or(f64::NAN, f64::from));
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
            energies: ["ALLIE", "ALLKE", "ALLWK", "ALLVD"].map(String::from).to_vec(),
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
            ("ALLKE", "total model kinetic energy", at(&history.kinetic, i)),
            ("ALLIE", "total model internal energy", at(&history.elastic, i)),
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
                flow_factor: Vec::new(),
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
            bodies: vec![body("Paste", 1.0, 20.0, 30.0), body("Dough", 3.0, 40.0, 50.0)],
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
