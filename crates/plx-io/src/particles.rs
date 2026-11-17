//! The particle model of a Particle Flow step, made of the FE model and its mesh when the
//! analysis starts: the meshed solid parts with a section are filled with particles of their
//! material, the parts of rigid bodies are the walls, the contact regions between them give
//! the wall laws, the gravity loads of the step the gravity (concept:
//! `STUFENKONZEPT_PARTICLE_FLOW_STEP.md`).
//!
//! The solver fills a body with particles inside its closed surface, which is the skin of
//! the part's mesh; a wall is the skin of the parts of its rigid body.

use std::collections::{BTreeSet, HashMap};

use plx_mesh::{ElementFamily, FeMesh};
use plx_model::particle_flow::{BoxSide, Flow, ParticleBox, ParticleFlowStep};
use plx_model::{Constraint, ContactBehavior, FeModel, Material, ModelItem, StepKind};
use plxs_core::particles::{
    self as ir, MaterialBody, ParticleControl, ParticleMaterial, ParticleModel, RigidWall, Side,
    TriangleSurface, WallLaw,
};

/// Why a Particle Flow step cannot be computed, with the item of the model it concerns.
#[derive(Clone, Debug, PartialEq)]
pub struct ParticleError {
    pub message: String,
    pub item: Option<ModelItem>,
}

impl std::fmt::Display for ParticleError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ParticleError {}

fn error(message: impl Into<String>, item: Option<ModelItem>) -> ParticleError {
    ParticleError {
        message: message.into(),
        item,
    }
}

/// The particle model of a Particle Flow step.
#[derive(Clone, Debug, PartialEq)]
pub struct LoweredParticles {
    pub model: ParticleModel,
    /// The step of the FE model, by index.
    pub step: usize,
    /// The name of the FE part each body is filled from, in the order of
    /// [`ParticleModel::bodies`].
    pub bodies: Vec<String>,
    /// What the lowering noticed but did not stop for.
    pub warnings: Vec<String>,
}

/// The active Particle Flow step of the model, by index, if it has one.
pub fn particle_step(fe: &FeModel) -> Option<usize> {
    (fe.steps.iter()).position(|s| s.active && matches!(s.kind, StepKind::ParticleFlow(_)))
}

/// The particle model of the active Particle Flow step of `fe` on `mesh`.
pub fn lower(
    mesh: &FeMesh,
    fe: &FeModel,
    heading: &str,
) -> Result<LoweredParticles, ParticleError> {
    let index = particle_step(fe)
        .ok_or_else(|| error("The model has no active Particle Flow step.", None))?;
    let step = &fe.steps[index];
    let StepKind::ParticleFlow(settings) = &step.kind else {
        unreachable!("particle_step finds Particle Flow steps")
    };
    let item = Some(ModelItem::Step(index));
    if let Some(other) = (fe.steps.iter()).position(|s| s.active && !std::ptr::eq(s, step)) {
        return Err(error(
            format!(
                "{}: a Particle Flow step is computed alone. Deactivate step {} or put the \
                 Particle Flow step into an analysis of its own.",
                step.name, fe.steps[other].name
            ),
            item,
        ));
    }
    if let Some(problem) = settings.problem() {
        return Err(error(format!("{}: {problem}", step.name), item));
    }
    if fe.properties.space.is_2d() {
        return Err(error(
            format!("{}: a Particle Flow step needs a 3D model.", step.name),
            item,
        ));
    }
    let mut warnings = Vec::new();

    // The parts of the rigid bodies are the walls.
    let mut walls: Vec<(String, BTreeSet<usize>, usize)> = Vec::new();
    for (c, constraint) in fe.constraints.iter().enumerate() {
        let Constraint::RigidBody(body) = constraint else {
            continue;
        };
        if !body.active {
            continue;
        }
        let parts = parts_of(&body.region, mesh);
        if parts.is_empty() {
            return Err(error(
                format!("{}: the rigid body holds no part of the mesh.", body.name),
                Some(ModelItem::Constraint(c)),
            ));
        }
        walls.push((body.name.clone(), parts, c));
    }
    let wall_part: BTreeSet<usize> = walls.iter().flat_map(|w| w.1.iter().copied()).collect();

    // Every other solid part is filled with the material of its section.
    let element_part = element_parts(mesh);
    let mut section_of = vec![None; mesh.parts.len()];
    for (s, section) in fe.sections.iter().enumerate() {
        for element in section.region.elements(mesh) {
            if let Some(&part) = element_part.get(&element)
                && section_of[part].is_none()
            {
                section_of[part] = Some(s);
            }
        }
    }
    let mut model = ParticleModel {
        heading: heading.to_owned(),
        ..Default::default()
    };
    let mut bodies = Vec::new();
    let mut body_part = Vec::new();
    let mut material_index: HashMap<plx_model::Id, usize> = HashMap::new();
    for (p, part) in mesh.parts.iter().enumerate() {
        if wall_part.contains(&p) {
            continue;
        }
        if !is_solid(mesh, p) {
            warnings.push(format!(
                "Part {} is no solid and takes no part in the particle flow.",
                part.name
            ));
            continue;
        }
        let Some(s) = section_of[p] else {
            return Err(error(
                format!(
                    "Part {} has no section: give it a section with the material it is filled \
                     with, or make it a rigid body.",
                    part.name
                ),
                Some(ModelItem::Part(p)),
            ));
        };
        let section = &fe.sections[s];
        let (m, material) = (section.material)
            .and_then(|id| fe.materials.iter().enumerate().find(|(_, m)| m.id == id))
            .ok_or_else(|| {
                error(
                    format!("{}: the section has no material.", section.name),
                    Some(ModelItem::Section(s)),
                )
            })?;
        let solver_material = match material_index.get(&material.id) {
            Some(&i) => i,
            None => {
                let law = particle_material(material)
                    .map_err(|e| error(e, Some(ModelItem::Material(m))))?;
                model.materials.push(law);
                material_index.insert(material.id, model.materials.len() - 1);
                model.materials.len() - 1
            }
        };
        model.bodies.push(MaterialBody {
            name: part.name.clone(),
            material: solver_material,
            shape: part_surface(mesh, &[p]),
            velocity: [0.0; 3],
        });
        bodies.push(part.name.clone());
        body_part.push(p);
    }
    if model.bodies.is_empty() {
        return Err(error(
            format!(
                "{}: no part is filled with material. Every solid part with a section that is \
                 not a rigid body is.",
                step.name
            ),
            item,
        ));
    }

    // The walls with the laws of the contact regions between them and the bodies.
    for (name, parts, c) in &walls {
        let parts: Vec<usize> = parts.iter().copied().collect();
        if parts.iter().any(|&p| !is_solid(mesh, p)) {
            return Err(error(
                format!(
                    "{name}: the walls of a Particle Flow step are solid parts; a rigid body of \
                     shells or beams has no inside."
                ),
                Some(ModelItem::Constraint(*c)),
            ));
        }
        let law = wall_law(fe, mesh, &parts, &body_part, name, &mut warnings);
        let motion = wall_motion(fe, index, *c)?;
        model.walls.push(RigidWall {
            name: name.clone(),
            shape: part_surface(mesh, &parts),
            law,
            motion,
        });
    }

    // The loads of the step: gravity, and nothing else yet.
    let mut gravity = [0.0; 3];
    for load in step.loads.iter().filter(|l| l.active) {
        match load.kind.gravity_vector() {
            Some(g) => (0..3).for_each(|a| gravity[a] += g[a]),
            None => warnings.push(format!(
                "{}: a Particle Flow step takes gravity loads only; the load is left out.",
                load.name
            )),
        }
    }
    model.gravity = gravity;
    // Boundary conditions move the rigid bodies at their reference points, and nothing else.
    let drivers: BTreeSet<plx_model::Id> = (walls.iter())
        .filter_map(|(_, _, c)| match &fe.constraints[*c] {
            Constraint::RigidBody(body) => Some(body.reference_point),
            _ => None,
        })
        .collect();
    for bc in step.boundary_conditions.iter().filter(|bc| bc.active) {
        let drives =
            matches!(bc.region, plx_model::Region::ReferencePoint(id) if drivers.contains(&id));
        if !drives {
            warnings.push(format!(
                "{}: in a Particle Flow step boundary conditions move the reference points of \
                 rigid bodies only; it is left out.",
                bc.name
            ));
        }
    }

    model.domain = domain(settings, &model)?;
    model.control = ParticleControl {
        end_time: settings.duration,
        frames: settings.frames as usize,
        sound_speed: settings.wave_speed,
        smooth_volume: settings.smooth_volume,
        ..ParticleControl::default()
    };
    Ok(LoweredParticles {
        model,
        step: index,
        bodies,
        warnings,
    })
}

/// The motion of the rigid body `constraint` in step `step`: the displacement boundary
/// condition on its reference point, if any, as a translation and a rotation about that
/// point that follow the amplitude (a ramp over the step without one).
fn wall_motion(
    fe: &FeModel,
    step: usize,
    constraint: usize,
) -> Result<ir::RigidMotion, ParticleError> {
    let Constraint::RigidBody(body) = &fe.constraints[constraint] else {
        return Ok(ir::RigidMotion::default());
    };
    let Some(point) = (fe.reference_points.iter()).find(|p| p.id == body.reference_point) else {
        return Err(error(
            format!("{}: the reference point does not exist.", body.name),
            Some(ModelItem::Constraint(constraint)),
        ));
    };
    let mut motion = ir::RigidMotion {
        center: point.position,
        ..Default::default()
    };
    let the_step = &fe.steps[step];
    let mut found = None;
    for (b, bc) in the_step.boundary_conditions.iter().enumerate() {
        if !bc.active || bc.region != plx_model::Region::ReferencePoint(body.reference_point) {
            continue;
        }
        let item = Some(ModelItem::BoundaryCondition(step, b));
        if found.is_some() {
            return Err(error(
                format!(
                    "{}: the reference point of {} has a boundary condition already; give it \
                     one.",
                    bc.name, body.name
                ),
                item,
            ));
        }
        found = Some(b);
        let plx_model::BoundaryKind::Displacement(displacement) = &bc.kind else {
            return Err(error(
                format!(
                    "{}: a rigid body of a Particle Flow step moves by a displacement and \
                     rotation of its reference point.",
                    bc.name
                ),
                item,
            ));
        };
        if displacement.tables.iter().any(Option::is_some) {
            return Err(error(
                format!(
                    "{}: a Particle Flow step takes no time tables; give the motion an \
                     amplitude instead.",
                    bc.name
                ),
                item,
            ));
        }
        // The directions of its coordinate system, in global coordinates.
        let axes = match displacement.system {
            None => [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            Some(id) => {
                let system = (fe.coordinate_systems.iter())
                    .find(|s| s.id == id)
                    .ok_or_else(|| {
                        error(
                            format!("{}: the coordinate system does not exist.", bc.name),
                            item,
                        )
                    })?;
                let frame = system.frame(&fe.coordinate_systems);
                frame
                    .directions_at(point.position)
                    .map_err(|e| error(format!("{}: {e}", bc.name), item))?
            }
        };
        let value = |i: usize| displacement.values[i].unwrap_or(0.0);
        for (a, axis) in axes.iter().enumerate() {
            for (g, &d) in axis.iter().enumerate() {
                motion.velocity[g] += value(a) * d;
                motion.angular[g] += value(3 + a) * d;
            }
        }
        let duration = the_step.kind.period();
        motion.amplitude = match bc.amplitude {
            None => vec![[0.0, 0.0], [duration, 1.0]],
            Some(id) => {
                let amplitude = (fe.amplitudes.iter()).find(|a| a.id == id).ok_or_else(|| {
                    error(format!("{}: the amplitude does not exist.", bc.name), item)
                })?;
                if let Some(problem) = amplitude.points_problem() {
                    return Err(error(format!("{}: {problem}.", amplitude.name), item));
                }
                (amplitude.points.iter())
                    .map(|p| p[0] + amplitude.shift_time)
                    .map(|t| [t, amplitude.value_at(t)])
                    .collect()
            }
        };
    }
    Ok(motion)
}

/// The solver's material of a material of the model: its density and elastic constants,
/// flowing after its [`Flow`] or elastic without.
pub fn particle_material(material: &Material) -> Result<ParticleMaterial, String> {
    let density = (material.density)
        .filter(|d| d.is_finite() && *d > 0.0)
        .ok_or_else(|| format!("Material {} has no density.", material.name))?;
    let elastic = material.small_strain_elastic().ok_or_else(|| {
        format!(
            "Material {} has no elastic constants (Young's modulus and Poisson's ratio).",
            material.name
        )
    })?;
    if let Some(problem) = material.flow.as_ref().and_then(Flow::problem) {
        return Err(format!("Material {}: {problem}", material.name));
    }
    let bulk = elastic.young / (3.0 * (1.0 - 2.0 * elastic.poisson));
    let shear = elastic.young / (2.0 * (1.0 + elastic.poisson));
    let law = match material.flow {
        None => ir::ParticleLaw::Elastic {
            young: elastic.young,
            poisson: elastic.poisson,
        },
        Some(Flow::HerschelBulkley {
            yield_stress,
            consistency,
            exponent,
        }) => ir::ParticleLaw::ViscoPlastic {
            shear,
            bulk,
            yield_stress,
            consistency,
            exponent,
        },
        Some(Flow::Newtonian { viscosity }) => ir::ParticleLaw::Fluid { bulk, viscosity },
    };
    Ok(ParticleMaterial {
        name: material.name.clone(),
        density,
        law,
    })
}

/// The law between a wall and the material: that of the first active contact region
/// between its parts and those of a body; the material sticks without one.
fn wall_law(
    fe: &FeModel,
    mesh: &FeMesh,
    wall: &[usize],
    bodies: &[usize],
    name: &str,
    warnings: &mut Vec<String>,
) -> WallLaw {
    let mut laws = Vec::new();
    for region in fe.contact_regions.iter().filter(|r| r.active) {
        let (a, b) = region.parts(mesh);
        let touches = |side: &BTreeSet<usize>, of: &[usize]| of.iter().any(|p| side.contains(p));
        if (touches(&a, wall) && touches(&b, bodies)) || (touches(&b, wall) && touches(&a, bodies))
        {
            laws.push((region.name.clone(), law_of(region.behavior)));
        }
    }
    match laws.as_slice() {
        [] => WallLaw::Stick,
        [(_, law)] => *law,
        [(first, law), rest @ ..] => {
            if rest.iter().any(|(_, l)| l != law) {
                warnings.push(format!(
                    "{name}: contact regions with different behaviours meet the wall; {first} \
                     is taken for all of it."
                ));
            }
            *law
        }
    }
}

/// The wall law of a contact behaviour.
pub fn law_of(behavior: ContactBehavior) -> WallLaw {
    match behavior {
        ContactBehavior::Bonded | ContactBehavior::Rough => WallLaw::Stick,
        ContactBehavior::NoSeparation | ContactBehavior::Frictionless => WallLaw::Slip,
        ContactBehavior::Frictional { mu } => WallLaw::Coulomb { friction: mu },
    }
}

/// The box of the step: around all bodies and walls with the margin, or its own.
fn domain(settings: &ParticleFlowStep, model: &ParticleModel) -> Result<ir::Domain, ParticleError> {
    let (min, max) = match settings.domain {
        ParticleBox::Fixed { min, max } => (min, max),
        ParticleBox::Around { margin } => {
            let shapes =
                (model.bodies.iter().map(|b| &b.shape)).chain(model.walls.iter().map(|w| &w.shape));
            let mut bounds = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
            for (lo, hi) in shapes.filter_map(TriangleSurface::bounds) {
                for a in 0..3 {
                    bounds.0[a] = bounds.0[a].min(lo[a]);
                    bounds.1[a] = bounds.1[a].max(hi[a]);
                }
            }
            (bounds.0.map(|v| v - margin), bounds.1.map(|v| v + margin))
        }
    };
    let side = |side: BoxSide| match side {
        BoxSide::Frictionless => Side::Wall(WallLaw::Slip),
        BoxSide::Rough => Side::Wall(WallLaw::Stick),
        BoxSide::Frictional { mu } => Side::Wall(WallLaw::Coulomb { friction: mu }),
        BoxSide::Open => Side::Open,
    };
    Ok(ir::Domain {
        min,
        max,
        cell: settings.cell_size,
        sides: settings.sides.map(side),
        periodic: [false; 3],
    })
}

/// The parts the elements and faces of a region belong to.
fn parts_of(region: &plx_model::Region, mesh: &FeMesh) -> BTreeSet<usize> {
    if let plx_model::Region::Parts(names) = region {
        return (mesh.parts.iter().enumerate())
            .filter(|(_, p)| names.contains(&p.name))
            .map(|(i, _)| i)
            .collect();
    }
    let element_part = element_parts(mesh);
    (region.elements(mesh).into_iter())
        .chain(region.faces(mesh).into_iter().map(|(e, _)| e))
        .filter_map(|e| element_part.get(&e).copied())
        .collect()
}

fn element_parts(mesh: &FeMesh) -> HashMap<plx_mesh::ElementId, usize> {
    (mesh.parts.iter().enumerate())
        .flat_map(|(p, part)| part.elements.iter().map(move |&e| (e, p)))
        .collect()
}

/// Whether a part is meshed with solid elements.
fn is_solid(mesh: &FeMesh, part: usize) -> bool {
    let elements = &mesh.parts[part].elements;
    !elements.is_empty()
        && elements.iter().all(|&e| {
            mesh.element(e)
                .is_some_and(|e| e.shape.family() == ElementFamily::Solid)
        })
}

/// The closed surface of solid parts: the corner triangles of their skins, facing out.
pub fn part_surface(mesh: &FeMesh, parts: &[usize]) -> TriangleSurface {
    let coords = mesh.coords();
    let mut surface = TriangleSurface::default();
    let mut index: HashMap<usize, u32> = HashMap::new();
    for &p in parts {
        let skin = plx_mesh::extract_part_skin(mesh, &mesh.parts[p], 30.0);
        let mut triangles = Vec::new();
        for face in &skin.faces {
            let corners: Vec<u32> = (face.corners.iter())
                .map(|&node| {
                    *index.entry(node).or_insert_with(|| {
                        surface.points.push(coords[node]);
                        surface.points.len() as u32 - 1
                    })
                })
                .collect();
            for k in 1..corners.len().saturating_sub(1) {
                triangles.push([corners[0], corners[k], corners[k + 1]]);
            }
        }
        // Each body of the part faces out: a negative enclosed volume turns it.
        if signed_volume(&surface.points, &triangles) < 0.0 {
            triangles.iter_mut().for_each(|t| t.swap(1, 2));
        }
        surface.triangles.extend(triangles);
    }
    surface
}

/// The volume a closed surface of triangles encloses, negative where they face in.
fn signed_volume(points: &[[f64; 3]], triangles: &[[u32; 3]]) -> f64 {
    let mut volume = 0.0;
    for t in triangles {
        let [a, b, c] = t.map(|i| points[i as usize]);
        volume += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0]);
    }
    volume / 6.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_mesh::{Element, ElementShape, Part};
    use plx_model::{
        BoundaryCondition, BoundaryKind, Elastic, Id, ReferencePoint, Region, RigidBody, Section,
        Step,
    };

    /// Two unit cubes side by side: a paste and a tool.
    fn mesh() -> FeMesh {
        let mut mesh = FeMesh::default();
        for (c, name) in ["Paste", "Tool"].into_iter().enumerate() {
            let x0 = 1.5 * c as f64;
            let first = 8 * c as u32 + 1;
            for (k, [x, y, z]) in [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [1.0, 1.0, 1.0],
                [0.0, 1.0, 1.0],
            ]
            .into_iter()
            .enumerate()
            {
                mesh.set_node(first + k as u32, [x0 + x, y, z]);
            }
            mesh.add_element(Element {
                id: c as u32 + 1,
                type_name: "C3D8".into(),
                shape: ElementShape::Hex8,
                nodes: (first..first + 8).collect(),
            })
            .unwrap();
            mesh.parts.push(Part {
                bodies: Vec::new(),
                name: name.into(),
                elements: vec![c as u32 + 1],
            });
        }
        mesh
    }

    /// A Particle Flow step that turns the tool by `turn` about z at its reference point.
    fn model(turn: Option<f64>) -> FeModel {
        let material = Material {
            name: "Paste".into(),
            density: Some(1000.0),
            elastic: Some(Elastic {
                young: 1e5,
                poisson: 0.3,
            }),
            flow: Some(Flow::KINDS[0]),
            ..Material::default()
        };
        let point = ReferencePoint {
            id: Id::new(),
            name: "RP".into(),
            position: [2.0, 0.5, 0.5],
            definition: Default::default(),
            visible: true,
        };
        let mut body = RigidBody::new("Tool", point.id);
        body.region = Region::Parts(vec!["Tool".into()]);
        let mut step = Step::new_static("Flow");
        step.kind = StepKind::ParticleFlow(ParticleFlowStep {
            duration: 2.0,
            cell_size: 0.25,
            ..Default::default()
        });
        if let Some(turn) = turn {
            step.boundary_conditions.push(BoundaryCondition::new(
                "Turn",
                Region::ReferencePoint(point.id),
                BoundaryKind::displacement([
                    Some(0.0),
                    Some(0.0),
                    Some(0.0),
                    Some(0.0),
                    Some(0.0),
                    Some(turn),
                ]),
            ));
        }
        FeModel {
            sections: vec![Section {
                orientation: None,
                name: "Paste".into(),
                material: Some(material.id),
                region: Region::Parts(vec!["Paste".into()]),
                thickness: 1.0,
                kind: Default::default(),
            }],
            materials: vec![material],
            reference_points: vec![point],
            constraints: vec![Constraint::RigidBody(body)],
            steps: vec![step],
            ..FeModel::default()
        }
    }

    #[test]
    fn a_rigid_body_turns_by_its_reference_point_over_the_step() {
        let mesh = mesh();
        let lowered = lower(&mesh, &model(Some(6.0)), "Test").expect("lowers");
        assert_eq!(lowered.bodies, vec!["Paste".to_string()]);
        assert_eq!(lowered.model.walls.len(), 1);
        let motion = &lowered.model.walls[0].motion;
        assert_eq!(motion.angular, [0.0, 0.0, 6.0]);
        assert_eq!(motion.center, [2.0, 0.5, 0.5]);
        // The default ramp: the whole turn over the duration, at 3 rad/s.
        assert_eq!(motion.amplitude, vec![[0.0, 0.0], [2.0, 1.0]]);
        assert_eq!(motion.factor(1.0) * motion.angular[2], 3.0);
        assert!(lowered.warnings.is_empty(), "{:?}", lowered.warnings);
        // Without a boundary condition the tool rests.
        let resting = lower(&mesh, &model(None), "Test").expect("lowers");
        assert!(resting.model.walls[0].motion.is_resting());
    }
}
