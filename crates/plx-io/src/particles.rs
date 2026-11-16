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
        model.walls.push(RigidWall {
            name: name.clone(),
            shape: part_surface(mesh, &parts),
            law,
            motion: ir::RigidMotion::default(),
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
    if step.boundary_conditions.iter().any(|bc| bc.active) {
        warnings.push(format!(
            "{}: a Particle Flow step does not move the rigid bodies by boundary conditions \
             yet; they rest.",
            step.name
        ));
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
