//! The particle model of a Particle Flow step, made of the FE model and its mesh when the
//! analysis starts: the meshed solid parts with a section are filled with particles of their
//! material, the parts of rigid bodies are the walls, the contact regions between them give
//! the wall laws, the gravity loads of the step the gravity (concept:
//! `STUFENKONZEPT_PARTICLE_FLOW_STEP.md`).
//!
//! The solver fills a body with particles inside its closed surface, which is the skin of
//! the part's mesh; a wall is the skin of the parts of its rigid body. A 2D model (plane
//! strain or axisymmetric) is a slab one cell thick along z: its parts are extruded along z
//! well beyond it, so that the slab cuts them in their middle.

use std::collections::{BTreeSet, HashMap};

use plx_mesh::{ElementFamily, FeMesh};
use plx_model::particle_flow::{BoxSide, Flow, FlowProperty, ParticleFlowStep};
use plx_model::{
    Constraint, ContactBehavior, FeModel, HistoryKind, Material, ModelItem, ModelSpace, StepKind,
};
use plx_results::particles::{HistoryRequest, TemperatureRequest, WallRequest, set_name};
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
    /// What the history outputs of the step ask for.
    pub history: HistoryRequest,
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
    let space = match fe.properties.space {
        ModelSpace::ThreeD => ir::Space::Solid,
        ModelSpace::PlaneStrain => ir::Space::PlaneStrain,
        ModelSpace::Axisymmetric => ir::Space::Axisymmetric,
        ModelSpace::PlaneStress => {
            return Err(error(
                format!(
                    "{}: a Particle Flow step takes a 3D, plane strain or axisymmetric model; \
                     a thin plate in plane stress has no flow of its own.",
                    step.name
                ),
                item,
            ));
        }
    };
    let shapes = Shapes {
        space,
        cell: settings.cell_size,
    };
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
        if !shapes.is_solid(mesh, p) {
            warnings.push(format!(
                "Part {} is no {} and takes no part in the particle flow.",
                part.name,
                shapes.solid_name()
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
            shape: shapes.surface(mesh, &[p]),
            velocity: [0.0; 3],
            temperature: 0.0,
            gravity: [0.0; 3],
        });
        bodies.push(part.name.clone());
        body_part.push(p);
    }
    if settings.heat {
        model.heat = Some(ir::Heat {
            wall_transfer: settings.wall_transfer,
        });
        for (body, &part) in model.bodies.iter_mut().zip(&body_part) {
            let material = &model.materials[body.material];
            if material.thermal.is_none() {
                return Err(error(
                    format!(
                        "Material {}: a Particle Flow step that follows the temperature needs \
                         its conductivity and specific heat.",
                        material.name
                    ),
                    fe.materials
                        .iter()
                        .position(|m| m.name == material.name)
                        .map(ModelItem::Material),
                ));
            }
            body.temperature = start_temperature(fe, mesh, part).ok_or_else(|| {
                error(
                    format!(
                        "Part {}: a Particle Flow step that follows the temperature needs its \
                         temperature at the start, a temperature initial condition.",
                        body.name
                    ),
                    Some(ModelItem::Part(part)),
                )
            })?;
        }
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
        if parts.iter().any(|&p| !shapes.is_solid(mesh, p)) {
            return Err(error(
                format!(
                    "{name}: the walls of a Particle Flow step are {} parts; a rigid body of \
                     {} has no inside.",
                    shapes.solid_name(),
                    if space == ir::Space::Solid {
                        "shells or beams"
                    } else {
                        "beams"
                    }
                ),
                Some(ModelItem::Constraint(*c)),
            ));
        }
        let law = wall_law(fe, mesh, &parts, &body_part, name, &mut warnings);
        let motion = wall_motion(fe, index, *c)?;
        let temperature = wall_temperature(step, &fe.constraints[*c]);
        if temperature.is_some() && !settings.heat {
            warnings.push(format!(
                "{name}: the temperature of the wall acts only when the step follows the \
                 temperature."
            ));
        }
        model.walls.push(RigidWall {
            name: name.clone(),
            shape: shapes.surface(mesh, &parts),
            law,
            motion,
            temperature: temperature.filter(|_| settings.heat),
        });
    }

    // The loads of the step: gravity on the bodies it selects, and nothing else yet.
    for load in step.loads.iter().filter(|l| l.active) {
        let Some(g) = load.kind.gravity_vector() else {
            warnings.push(format!(
                "{}: a Particle Flow step takes gravity loads only; the load is left out.",
                load.name
            ));
            continue;
        };
        // In 2D the gravity acts in the plane, in a body of revolution along its axis.
        let mut g = g;
        let across: &[usize] = match space {
            ir::Space::Solid => &[],
            ir::Space::PlaneStrain => &[2],
            ir::Space::Axisymmetric => &[0, 2],
        };
        if across.iter().any(|&a| g[a] != 0.0) {
            warnings.push(format!(
                "{}: in a {} model gravity acts {}; the rest of it is left out.",
                load.name,
                shapes.space_name(),
                if space == ir::Space::Axisymmetric {
                    "along the axis (y) only"
                } else {
                    "in the plane"
                }
            ));
        }
        across.iter().for_each(|&a| g[a] = 0.0);
        let parts = parts_of(&load.region, mesh);
        let mut selected = false;
        for (body, part) in model.bodies.iter_mut().zip(&body_part) {
            if parts.contains(part) {
                (0..3).for_each(|a| body.gravity[a] += g[a]);
                selected = true;
            }
        }
        if !selected {
            warnings.push(format!(
                "{}: the gravity load selects no part filled with material and acts on none.",
                load.name
            ));
        }
    }
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

    model.domain = domain(settings, &model, space, &mut warnings)?;
    model.control = ParticleControl {
        end_time: settings.duration,
        frames: settings.frames as usize,
        sound_speed: settings.wave_speed,
        smooth_volume: settings.smooth_volume,
        ..ParticleControl::default()
    };
    let history = history_request(fe, mesh, index, &walls, &body_part, &model, &mut warnings);
    Ok(LoweredParticles {
        model,
        step: index,
        bodies,
        history,
        warnings,
    })
}

/// What the history outputs of step `step` ask for: the energies, and of a node output
/// the force, moment and work (`RF`) and the heat (`RFL`) of the walls of the rigid bodies
/// whose reference point or parts it names, and the temperature (`NT`) of the bodies of
/// material in its parts. The rest is left out with a warning.
fn history_request(
    fe: &FeModel,
    mesh: &FeMesh,
    step: usize,
    walls: &[(String, BTreeSet<usize>, usize)],
    body_part: &[usize],
    model: &ParticleModel,
    warnings: &mut Vec<String>,
) -> HistoryRequest {
    let mut request = HistoryRequest::default();
    for output in fe
        .step_history_outputs(step)
        .into_iter()
        .filter(|o| o.active)
    {
        let name = &output.name;
        let region = match &output.kind {
            HistoryKind::Energies => {
                for variable in &output.variables {
                    if !request.energies.contains(variable) {
                        request.energies.push(variable.clone());
                    }
                }
                continue;
            }
            HistoryKind::Node { region } => region,
            HistoryKind::Element { .. } | HistoryKind::Contact { .. } => {
                warnings.push(format!(
                    "{name}: a Particle Flow step writes node outputs and energies only; the \
                     output is left out."
                ));
                continue;
            }
        };
        let parts = parts_of(region, mesh);
        let hit: Vec<usize> = (walls.iter().enumerate())
            .filter(|(_, (_, wall_parts, c))| {
                let at_point = matches!(
                    (&fe.constraints[*c], region),
                    (Constraint::RigidBody(body), plx_model::Region::ReferencePoint(id))
                        if body.reference_point == *id
                );
                at_point || !wall_parts.is_disjoint(&parts)
            })
            .map(|(w, _)| w)
            .collect();
        let bodies: Vec<usize> = (body_part.iter().enumerate())
            .filter(|(_, p)| parts.contains(p))
            .map(|(b, _)| b)
            .collect();
        let asks = |v: &str| output.variables.iter().any(|x| x == v);
        let (force, heat) = (asks("RF"), asks("RFL"));
        if (force || heat) && hit.is_empty() {
            warnings.push(format!(
                "{name}: RF and RFL are written for rigid bodies, by their reference point or \
                 their parts; the output names none."
            ));
        }
        if heat && model.heat.is_none() {
            warnings.push(format!(
                "{name}: RFL needs the heat of the step; it is left out."
            ));
        }
        for &wall in &hit {
            match request.walls.iter_mut().find(|r| r.wall == wall) {
                Some(r) => {
                    r.force |= force;
                    r.heat |= heat;
                }
                None => request.walls.push(WallRequest { wall, force, heat }),
            }
        }
        if asks("NT") {
            if model.heat.is_none() {
                warnings.push(format!(
                    "{name}: NT needs the heat of the step; it is left out."
                ));
            } else if bodies.is_empty() {
                warnings.push(format!(
                    "{name}: NT is written for parts filled with material; the output names \
                     none."
                ));
            } else {
                request.temperatures.push(TemperatureRequest {
                    set: set_name(name),
                    bodies,
                });
            }
        }
        let other: Vec<&str> = (output.variables.iter())
            .map(String::as_str)
            .filter(|v| !["RF", "RFL", "NT"].contains(v))
            .collect();
        if !other.is_empty() {
            warnings.push(format!(
                "{name}: a Particle Flow step writes RF, RFL and NT; {} is left out.",
                other.join(", ")
            ));
        }
    }
    request.walls.retain(|r| r.force || r.heat);
    request
}

/// The temperature a temperature boundary condition of the step gives the reference point
/// of a rigid body: the temperature of its walls.
fn wall_temperature(step: &plx_model::Step, constraint: &Constraint) -> Option<f64> {
    let Constraint::RigidBody(body) = constraint else {
        return None;
    };
    (step.boundary_conditions.iter())
        .filter(|bc| {
            bc.active && bc.region == plx_model::Region::ReferencePoint(body.reference_point)
        })
        .find_map(|bc| match bc.kind {
            plx_model::BoundaryKind::Temperature(t) => Some(t),
            _ => None,
        })
}

/// The temperature at the start of the part with index `part`: that of the first active
/// temperature initial condition whose region holds elements or nodes of it.
fn start_temperature(fe: &FeModel, mesh: &FeMesh, part: usize) -> Option<f64> {
    (fe.initial_conditions.iter().filter(|c| c.active)).find_map(|condition| {
        let plx_model::InitialConditionKind::Temperature(t) = condition.kind else {
            return None;
        };
        let holds = parts_of(&condition.region, mesh).contains(&part) || {
            let nodes: BTreeSet<_> = condition.region.nodes(mesh).into_iter().collect();
            (mesh.parts[part].elements.iter())
                .filter_map(|&e| mesh.element(e))
                .any(|e| e.nodes.iter().any(|n| nodes.contains(n)))
        };
        holds.then_some(t)
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
        if !bc.active
            || bc.region != plx_model::Region::ReferencePoint(body.reference_point)
            || matches!(bc.kind, plx_model::BoundaryKind::Temperature(_))
        {
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
    if !(elastic.young > 0.0 && elastic.poisson > -1.0 && elastic.poisson < 0.5) {
        return Err(format!(
            "Material {}: Young's modulus must be above zero and Poisson's ratio below 0.5 \
             (E = {}, nu = {}); take 0.49 for a nearly incompressible material.",
            material.name, elastic.young, elastic.poisson
        ));
    }
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
        Some(Flow::Granular {
            friction_angle,
            limit_angle,
            grain,
            inertial,
            fine_grain,
            fine_fraction,
        }) => ir::ParticleLaw::Granular {
            shear,
            bulk,
            friction: friction_angle.to_radians().tan(),
            friction_limit: limit_angle.to_radians().tan(),
            grain,
            inertial,
            fine_grain,
            fine_fraction,
        },
    };
    let thermal = match (material.conductivity, material.specific_heat) {
        (Some(conductivity), Some(specific_heat)) => Some(ir::ParticleThermal {
            conductivity,
            specific_heat,
        }),
        _ => None,
    };
    if let Some(flow) = &material.flow
        && let Some(problem) = material.flow_tables.problem(flow)
    {
        return Err(format!("Material {}: {problem}", material.name));
    }
    // Only the tables of the material's law act; a fixed value has none.
    let table = |property: FlowProperty| -> Vec<[f64; 2]> {
        let acts =
            (material.flow.as_ref()).is_some_and(|f| FlowProperty::of(f).contains(&property));
        match material.flow_tables.get(property) {
            Some(table) if acts => table.points.clone(),
            _ => Vec::new(),
        }
    };
    let flow_tables = ir::FlowTables {
        yield_stress: table(FlowProperty::YieldStress),
        consistency: table(FlowProperty::Consistency),
        viscosity: table(FlowProperty::Viscosity),
    };
    Ok(ParticleMaterial {
        name: material.name.clone(),
        density,
        law,
        thermal,
        flow_tables,
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

/// The box of the step: around all bodies and walls with the margin, or its own. The box of
/// a 2D model is the slab one cell thick along z; that of a body of revolution starts at its
/// axis, where its side is a frictionless wall.
fn domain(
    settings: &ParticleFlowStep,
    model: &ParticleModel,
    space: ir::Space,
    warnings: &mut Vec<String>,
) -> Result<ir::Domain, ParticleError> {
    let shapes =
        (model.bodies.iter().map(|b| &b.shape)).chain(model.walls.iter().map(|w| &w.shape));
    let mut bounds = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for (lo, hi) in shapes.filter_map(TriangleSurface::bounds) {
        for a in 0..3 {
            bounds.0[a] = bounds.0[a].min(lo[a]);
            bounds.1[a] = bounds.1[a].max(hi[a]);
        }
    }
    let (mut min, mut max) = settings
        .grid_box(Some(bounds))
        .expect("a box around given bounds");
    let mut sides = settings.sides;
    let mut periodic = [false; 3];
    if space != ir::Space::Solid {
        (min[2], max[2]) = (0.0, settings.cell_size);
        periodic[2] = true;
    }
    if space == ir::Space::Axisymmetric {
        if bounds.0[0] < -1e-9 * settings.cell_size {
            return Err(error(
                "In an axisymmetric model the x coordinate is the radius: the parts of a \
                 Particle Flow step lie at x of zero or above.",
                None,
            ));
        }
        min[0] = min[0].max(0.0);
        if min[0] == 0.0 {
            if sides[0] != BoxSide::Frictionless {
                warnings.push(
                    "The side X min of the box lies on the axis of revolution and is \
                     frictionless there."
                        .into(),
                );
            }
            sides[0] = BoxSide::Frictionless;
        }
    }
    if max[0] <= min[0] {
        return Err(error("The box of the Particle Flow step is empty.", None));
    }
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
        sides: sides.map(side),
        periodic,
        space,
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

/// How the parts of a model become closed surfaces: their skins in 3D, extruded along z
/// in 2D.
struct Shapes {
    space: ir::Space,
    cell: f64,
}

impl Shapes {
    /// Whether a part is meshed with solid elements, or in 2D with plane elements.
    fn is_solid(&self, mesh: &FeMesh, part: usize) -> bool {
        let family = if self.space == ir::Space::Solid {
            ElementFamily::Solid
        } else {
            ElementFamily::Surface
        };
        let elements = &mesh.parts[part].elements;
        !elements.is_empty()
            && elements
                .iter()
                .all(|&e| mesh.element(e).is_some_and(|e| e.shape.family() == family))
    }

    fn solid_name(&self) -> &'static str {
        if self.space == ir::Space::Solid {
            "solid"
        } else {
            "plane face"
        }
    }

    fn space_name(&self) -> &'static str {
        match self.space {
            ir::Space::Solid => "3D",
            ir::Space::PlaneStrain => "plane strain",
            ir::Space::Axisymmetric => "axisymmetric",
        }
    }

    fn surface(&self, mesh: &FeMesh, parts: &[usize]) -> TriangleSurface {
        if self.space == ir::Space::Solid {
            part_surface(mesh, parts)
        } else {
            // Far enough beyond the slab that the ends do not turn the distance near it.
            let reach = 8.0 * self.cell;
            plane_surface(mesh, parts, -reach, self.cell + reach)
        }
    }
}

/// The closed surface of plane parts extruded along z from `z0` to `z1`: the elements as
/// the two ends, the edges on the outline of each part as its sides, facing out.
pub fn plane_surface(mesh: &FeMesh, parts: &[usize], z0: f64, z1: f64) -> TriangleSurface {
    let coords = mesh.coords();
    let mut surface = TriangleSurface::default();
    for &p in parts {
        let mut index: HashMap<usize, u32> = HashMap::new();
        let mut point = |surface: &mut TriangleSurface, node: usize| {
            *index.entry(node).or_insert_with(|| {
                let [x, y, _] = coords[node];
                surface.points.push([x, y, z0]);
                surface.points.push([x, y, z1]);
                surface.points.len() as u32 - 2
            })
        };
        // The corners of each element, turned counterclockwise about z.
        let mut polygons = Vec::new();
        for &e in &mesh.parts[p].elements {
            let Some(element) = mesh.element(e) else {
                continue;
            };
            let Some(face) = element.shape.faces().first() else {
                continue;
            };
            let mut corners: Vec<usize> = (face.corners.iter())
                .filter_map(|&c| mesh.node_index(element.nodes[c]))
                .collect();
            let area: f64 = (0..corners.len())
                .map(|i| {
                    let (a, b) = (coords[corners[i]], coords[corners[(i + 1) % corners.len()]]);
                    a[0] * b[1] - a[1] * b[0]
                })
                .sum();
            if area < 0.0 {
                corners.reverse();
            }
            polygons.push(corners);
        }
        // An edge on the outline belongs to one element only.
        let mut edges: HashMap<(usize, usize), (usize, usize, usize)> = HashMap::new();
        for corners in &polygons {
            for i in 0..corners.len() {
                let (a, b) = (corners[i], corners[(i + 1) % corners.len()]);
                edges.entry((a.min(b), a.max(b))).or_insert((a, b, 0)).2 += 1;
            }
        }
        for corners in &polygons {
            let ids: Vec<u32> = corners.iter().map(|&n| point(&mut surface, n)).collect();
            for k in 1..ids.len().saturating_sub(1) {
                // The bottom end faces down, the top end up.
                surface.triangles.push([ids[0], ids[k + 1], ids[k]]);
                surface
                    .triangles
                    .push([ids[0] + 1, ids[k] + 1, ids[k + 1] + 1]);
            }
        }
        let mut outline: Vec<_> = edges.into_values().filter(|e| e.2 == 1).collect();
        outline.sort_unstable();
        for (a, b, _) in outline {
            let (a, b) = (point(&mut surface, a), point(&mut surface, b));
            surface.triangles.push([a, b, b + 1]);
            surface.triangles.push([a, b + 1, a + 1]);
        }
    }
    surface
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

    /// The two parts of [`mesh`] in the plane: the paste of two squares, the tool of one.
    fn plane_mesh() -> FeMesh {
        let mut mesh = FeMesh::default();
        let mut node = 0u32;
        let mut element = 0u32;
        for (name, x0, squares) in [("Paste", 0.0, 2), ("Tool", 1.5, 1)] {
            let w = 1.0 / squares as f64;
            // Two rows of nodes, the squares between them share their edges.
            let first = node + 1;
            for row in [0.0, 1.0] {
                for c in 0..=squares {
                    node += 1;
                    mesh.set_node(node, [x0 + w * c as f64, row, 0.0]);
                }
            }
            let up = squares as u32 + 1;
            let mut elements = Vec::new();
            for s in 0..squares as u32 {
                let a = first + s;
                element += 1;
                // Clockwise, which the extrusion turns.
                mesh.add_element(Element {
                    id: element,
                    type_name: "CAX4".into(),
                    shape: ElementShape::Quad4,
                    nodes: vec![a, a + up, a + up + 1, a + 1],
                })
                .unwrap();
                elements.push(element);
            }
            mesh.parts.push(Part {
                bodies: Vec::new(),
                name: name.into(),
                elements,
            });
        }
        mesh
    }

    #[test]
    fn a_plane_model_is_a_slab_of_its_parts_extruded() {
        let mesh = plane_mesh();
        let mut fe = model(None);
        fe.properties.space = ModelSpace::Axisymmetric;
        let lowered = lower(&mesh, &fe, "Test").expect("lowers");
        let domain = &lowered.model.domain;
        assert_eq!(domain.space, ir::Space::Axisymmetric);
        assert_eq!(domain.periodic, [false, false, true]);
        assert_eq!(
            (domain.min[0], domain.min[2], domain.max[2]),
            (0.0, 0.0, 0.25)
        );
        assert_eq!(domain.sides[0], Side::Wall(WallLaw::Slip));
        // The paste, two squares that share an edge, encloses its area times the length.
        let body = &lowered.model.bodies[0].shape;
        let (lo, hi) = body.bounds().expect("points");
        let volume = signed_volume(&body.points, &body.triangles);
        assert!((volume - (hi[2] - lo[2])).abs() < 1e-9, "{volume}");
        assert!(lo[2] < -1.0 && hi[2] > 1.25);
        assert_eq!(body.triangles.len(), 2 * 2 * 2 + 6 * 2);
        let wall = &lowered.model.walls[0].shape;
        assert!(signed_volume(&wall.points, &wall.triangles) > 0.0);

        fe.properties.space = ModelSpace::PlaneStress;
        let refused = lower(&mesh, &fe, "Test").expect_err("plane stress is refused");
        assert!(
            refused.message.contains("plane strain"),
            "{}",
            refused.message
        );
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

    #[test]
    fn the_step_writes_the_history_it_asks_for_and_gravity_acts_on_what_it_selects() {
        let mesh = mesh();
        let mut fe = model(Some(6.0));
        let point = fe.reference_points[0].id;
        let mut torque = plx_model::HistoryOutput::node("Torque", Region::ReferencePoint(point));
        torque.variables = vec!["RF".into(), "U".into()];
        fe.put_history_output(0, torque);
        let mut energies = plx_model::HistoryOutput::energies("Energies");
        energies.variables = vec!["ALLKE".into()];
        fe.put_history_output(0, energies);
        fe.steps[0].loads.push(plx_model::Load::new(
            "Gravity",
            Region::Parts(vec!["Paste".into()]),
            plx_model::LoadKind::Gravity([0.0, 0.0, -9.81]),
        ));
        fe.steps[0].loads.push(plx_model::Load::new(
            "Nowhere",
            Region::Parts(Vec::new()),
            plx_model::LoadKind::Gravity([1.0, 0.0, 0.0]),
        ));
        let lowered = lower(&mesh, &fe, "Test").expect("lowers");
        let history = &lowered.history;
        assert_eq!(history.energies, vec!["ALLKE".to_string()]);
        assert_eq!(
            history.walls,
            vec![WallRequest {
                wall: 0,
                force: true,
                heat: false,
            }]
        );
        assert!(history.temperatures.is_empty());
        // Only the selected paste falls; the load without parts is reported.
        assert_eq!(lowered.model.gravity, [0.0; 3]);
        assert_eq!(lowered.model.bodies[0].gravity, [0.0, 0.0, -9.81]);
        let warned = |text: &str| lowered.warnings.iter().any(|w| w.contains(text));
        assert!(warned("Nowhere: the gravity load selects no part"));
        assert!(warned(
            "Torque: a Particle Flow step writes RF, RFL and NT; U is left out"
        ));
        assert_eq!(lowered.warnings.len(), 2, "{:?}", lowered.warnings);

        // Without outputs nothing is asked for.
        let bare = lower(&mesh, &model(None), "Test").expect("lowers");
        assert_eq!(bare.history, HistoryRequest::default());
    }

    #[test]
    fn an_incompressible_poisson_ratio_is_refused() {
        let mesh = mesh();
        let mut fe = model(None);
        fe.materials[0].elastic.as_mut().unwrap().poisson = 0.5;
        let error = lower(&mesh, &fe, "Test").expect_err("refused");
        assert!(error.message.contains("below 0.5"), "{error}");
    }

    #[test]
    fn a_granular_flow_gives_its_friction_as_coefficients() {
        let mesh = mesh();
        let mut fe = model(None);
        fe.materials[0].flow = Some(Flow::Granular {
            friction_angle: 30.0,
            limit_angle: 45.0,
            grain: 2e-3,
            inertial: 0.25,
            fine_grain: 0.0,
            fine_fraction: 0.0,
        });
        let lowered = lower(&mesh, &fe, "Test").expect("lowers");
        let ir::ParticleLaw::Granular {
            friction,
            friction_limit,
            grain,
            inertial,
            ..
        } = lowered.model.materials[0].law
        else {
            panic!("{:?}", lowered.model.materials[0].law)
        };
        assert!((friction - 30f64.to_radians().tan()).abs() < 1e-12);
        assert!((friction_limit - 1.0).abs() < 1e-12);
        assert_eq!((grain, inertial), (2e-3, 0.25));
        // A limit below the friction at rest is refused.
        fe.materials[0].flow = Some(Flow::Granular {
            friction_angle: 30.0,
            limit_angle: 20.0,
            grain: 2e-3,
            inertial: 0.25,
            fine_grain: 0.0,
            fine_fraction: 0.0,
        });
        let refused = lower(&mesh, &fe, "Test").unwrap_err();
        assert!(
            refused.message.contains("limit angle"),
            "{}",
            refused.message
        );
        // A mixture of two grain sizes, the fine ones smaller.
        fe.materials[0].flow = Some(Flow::Granular {
            friction_angle: 30.0,
            limit_angle: 45.0,
            grain: 2e-3,
            inertial: 0.25,
            fine_grain: 1e-3,
            fine_fraction: 0.3,
        });
        let lowered = lower(&mesh, &fe, "Test").expect("lowers");
        let ir::ParticleLaw::Granular {
            fine_grain,
            fine_fraction,
            ..
        } = lowered.model.materials[0].law
        else {
            unreachable!()
        };
        assert_eq!((fine_grain, fine_fraction), (1e-3, 0.3));
        if let Some(Flow::Granular { fine_grain, .. }) = &mut fe.materials[0].flow {
            *fine_grain = 3e-3;
        }
        let refused = lower(&mesh, &fe, "Test").unwrap_err();
        assert!(refused.message.contains("smaller"), "{}", refused.message);
    }

    #[test]
    fn a_step_with_heat_takes_the_temperatures_of_the_model() {
        let mesh = mesh();
        let mut fe = model(None);
        let StepKind::ParticleFlow(settings) = &mut fe.steps[0].kind else {
            unreachable!()
        };
        settings.heat = true;
        settings.wall_transfer = 500.0;
        fe.materials[0].conductivity = Some(0.5);
        fe.materials[0].specific_heat = Some(3500.0);
        // A table of the yield stress, one of the viscosity, which a paste does not have.
        let table = |points: Vec<[f64; 2]>| Some(plx_model::ValueTable { points });
        fe.materials[0].flow_tables = plx_model::particle_flow::FlowTables {
            yield_stress: table(vec![[20.0, 1000.0], [60.0, 500.0]]),
            consistency: None,
            viscosity: table(vec![[20.0, 1.0]]),
        };
        // Without a temperature at the start the part cannot be filled.
        let missing = lower(&mesh, &fe, "Test").unwrap_err();
        assert!(
            missing.message.contains("Part Paste"),
            "{}",
            missing.message
        );
        fe.initial_conditions.push(plx_model::InitialCondition {
            name: "Cold".into(),
            active: true,
            region: Region::Parts(vec!["Paste".into()]),
            kind: plx_model::InitialConditionKind::Temperature(4.0),
            distribution: None,
        });
        let point = fe.reference_points[0].id;
        fe.steps[0].boundary_conditions.push(BoundaryCondition::new(
            "Warm tool",
            Region::ReferencePoint(point),
            BoundaryKind::Temperature(40.0),
        ));
        let lowered = lower(&mesh, &fe, "Test").expect("lowers");
        let model = &lowered.model;
        assert_eq!(
            model.heat,
            Some(ir::Heat {
                wall_transfer: 500.0
            })
        );
        assert_eq!(model.bodies[0].temperature, 4.0);
        assert_eq!(model.walls[0].temperature, Some(40.0));
        // The temperature does not move the tool.
        assert!(model.walls[0].motion.is_resting());
        let material = &model.materials[0];
        assert_eq!(
            material.thermal,
            Some(ir::ParticleThermal {
                conductivity: 0.5,
                specific_heat: 3500.0
            })
        );
        assert_eq!(
            material.flow_tables,
            ir::FlowTables {
                yield_stress: vec![[20.0, 1000.0], [60.0, 500.0]],
                ..Default::default()
            }
        );
        assert!(lowered.warnings.is_empty(), "{:?}", lowered.warnings);
    }
}
