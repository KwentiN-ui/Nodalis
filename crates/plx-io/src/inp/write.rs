//! Lowering of a [`FeModel`] with its mesh into the solver's input, a
//! [`plxs_core::Analysis`], in the layout of a CalculiX input file.
//!
//! Regions the user picked in the GUI become node sets, element sets and surfaces named
//! as internal selections (`Internal_Selection-1_Fixed-1`), so the user never
//! defines sets by hand. Every step holds what is in force during it; a step takes over
//! from the steps before only what [`TakesOver`] says. An input file is the analysis
//! rendered as text ([`plxs_input::render`]), for the export only.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use plx_mesh::{ElementFamily, ElementId, FeMesh, NodeId, SurfaceDefinition};
use plx_model::{
    Active, AmplitudeTime, BeamProfile, BeamSection, BoundaryKind, BuckleStep,
    ComplexFrequencyStep, Constraint, ContactMethod, ContactPair, ContactScale, Damping,
    DefinedFieldKind, DynamicMethod, DynamicStep, FeModel, FieldResult, Formulation, FrequencyStep,
    GapConductance, HeatTransferStep, HistoryKind, HistoryOutput, HistoryVariable, Id,
    IncrementPlan, InitialConditionKind, InteractionProperty, LoadKind, ModalDamping, ModelItem,
    ModelSpace, MovingFrame, NodeTie, Origins, OutputDimension, Quantity, RayleighFit, Region,
    RemoteBehavior, RemoteDof, Sampler, Section, SectionKind, SectorDisplay, SensitivityStep,
    StaticStep, SteadyStateDynamicsStep, Step, StepKind, SurfaceBehavior, SurfaceInteraction,
    TableOwner, Tie, Totals, UnitSystem, ViscoStep, line_tangent, symmetry_dofs, symmetry_plane,
};
use plxs_core as ir;
use plxs_core::mesh::{ElementRef, NodeRef};
use plxs_core::normalized_name;
use plxs_core::step::OutputVariable;

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum WriteError {
    #[error("{item}: Region contains no {what}")]
    EmptyRegion { item: String, what: &'static str },
    #[error("{item}: Material {material} does not exist")]
    UnknownMaterial { item: String, material: String },
    #[error("{item}: Surface Interaction {interaction} does not exist")]
    UnknownInteraction { item: String, interaction: String },
    #[error("{item}: Amplitude {amplitude} does not exist")]
    UnknownAmplitude { item: String, amplitude: String },
    #[error("{item}: {reason}")]
    InvalidAmplitude { item: String, reason: String },
    #[error("{item}: Distribution {distribution} does not exist")]
    UnknownDistribution { item: String, distribution: String },
    /// A distribution whose equation is invalid or that has no points.
    #[error("{item}: Distribution {distribution}: {reason}")]
    InvalidDistribution {
        item: String,
        distribution: String,
        reason: String,
    },
    #[error("{item}: Surface {surface} does not exist")]
    UnknownSurface { item: String, surface: String },
    /// Contact forces need the surfaces of an active contact pair.
    #[error("{item}: Contact Pair {pair} does not exist or is deactivated")]
    UnknownContactPair { item: String, pair: String },
    /// A section that does not fit its elements, see [`Section::kind_problem`].
    #[error("{item}: {reason}")]
    InvalidSection { item: String, reason: String },
    #[error("{item}: {reason}")]
    InvalidCyclicSymmetry { item: String, reason: String },
    /// A defined field reads a result file whose name the solver cannot take.
    #[error("{item}: the result file {file} has no valid name")]
    InvalidResultFile { item: String, file: String },
    /// A boundary condition or load on a reference point no active rigid body is driven by,
    /// or a rigid body whose reference point does not exist.
    #[error("{item}: Reference Point {point} has no active rigid body")]
    UnknownReferencePoint { item: String, point: String },
    /// A history output of the point of a remote displacement that no active step applies.
    #[error("{item}: Remote Displacement {point} does not exist or is deactivated")]
    UnknownRemotePoint { item: String, point: String },
    /// A load without a direction, such as a gravity of zero.
    #[error("{item}: {reason}")]
    InvalidLoad { item: String, reason: String },
    /// A constraint that cannot be written, such as a spring with ground on both ends.
    #[error("{item}: {reason}")]
    InvalidConstraint { item: String, reason: String },
    /// A remote displacement whose point cannot be placed, e.g. in a missing coordinate
    /// system, or without nodes in its pinball region.
    #[error("{item}: {reason}")]
    InvalidBoundary { item: String, reason: String },
    /// A submodel boundary condition in a model that names no global results file.
    #[error(
        "{item}: the submodel has no global results file (Model > Model Properties: model \
         type Submodel)"
    )]
    NoGlobalResults { item: String },
    /// A symmetry boundary condition whose region fixes no plane.
    #[error("{item}: {reason}")]
    NoSymmetryPlane { item: String, reason: String },
    /// Something of the model the solver does not compute, such as massless contact.
    #[error("{item}: {reason}")]
    Unsupported { item: String, reason: String },
}

impl WriteError {
    /// The name of the item the error is about.
    pub fn item_name(&self) -> &str {
        use WriteError as E;
        match self {
            E::EmptyRegion { item, .. }
            | E::UnknownMaterial { item, .. }
            | E::UnknownInteraction { item, .. }
            | E::UnknownAmplitude { item, .. }
            | E::InvalidAmplitude { item, .. }
            | E::UnknownDistribution { item, .. }
            | E::InvalidDistribution { item, .. }
            | E::UnknownSurface { item, .. }
            | E::UnknownContactPair { item, .. }
            | E::InvalidSection { item, .. }
            | E::InvalidCyclicSymmetry { item, .. }
            | E::InvalidResultFile { item, .. }
            | E::UnknownReferencePoint { item, .. }
            | E::UnknownRemotePoint { item, .. }
            | E::InvalidLoad { item, .. }
            | E::InvalidConstraint { item, .. }
            | E::InvalidBoundary { item, .. }
            | E::NoGlobalResults { item }
            | E::NoSymmetryPlane { item, .. }
            | E::Unsupported { item, .. } => item,
        }
    }
}

impl WriteError {
    /// The item another refers to, by its id as the lowering found it.
    fn reference_mut(&mut self) -> Option<&mut String> {
        use WriteError as E;
        match self {
            E::UnknownMaterial { material: r, .. }
            | E::UnknownInteraction { interaction: r, .. }
            | E::UnknownAmplitude { amplitude: r, .. }
            | E::UnknownDistribution {
                distribution: r, ..
            }
            | E::UnknownContactPair { pair: r, .. }
            | E::UnknownReferencePoint { point: r, .. }
            | E::UnknownRemotePoint { point: r, .. } => Some(r),
            _ => None,
        }
    }

    /// Names the item the error refers to by its name in `model` instead of its id, which
    /// the user never sees; one that is not there is named as missing.
    fn name_reference(&mut self, model: &FeModel) {
        let Some(reference) = self.reference_mut() else {
            return;
        };
        if !reference.starts_with('#') {
            return;
        }
        let items = model.item_ids();
        let item = items.iter().find(|(_, _, id)| id.to_string() == *reference);
        *reference = item.map_or_else(|| "(missing)".to_string(), |(_, name, _)| name.clone());
    }
}

/// An error of the model with the item of the tree it is about, so that the GUI can point
/// at it. `item` is `None` when the error is about something the lowering derived (a tie of
/// a contact region's detection) that has no item of its own.
#[derive(Debug, thiserror::Error, PartialEq)]
#[error("{error}")]
pub struct ModelError {
    pub item: Option<ModelItem>,
    pub error: WriteError,
}

impl ModelError {
    /// The error with the item it names, looked up in `model` (in the step that was being
    /// lowered first).
    fn new(model: &FeModel, mut error: WriteError, step: Option<usize>) -> Self {
        error.name_reference(model);
        Self {
            item: model.item_named(error.item_name(), step),
            error,
        }
    }
}

/// Why a model cannot be lowered: the model itself (an empty region, a missing material),
/// or the analysis made of it, which the solver's validation refuses.
#[derive(Debug, thiserror::Error)]
pub enum LowerError {
    #[error(transparent)]
    Model(#[from] ModelError),
    #[error("{}", .0.iter().map(ToString::to_string).collect::<Vec<_>>().join("\n"))]
    Solver(Vec<ir::Error>),
}

/// The analysis of a model as the solver takes it.
#[derive(Clone, Debug, Default)]
pub struct Lowered {
    pub analysis: ir::Analysis,
    /// What the assembly of the steps and the validation noticed but did not stop for.
    pub warnings: Vec<String>,
    /// The constraints whose overlaps with others were resolved, each with what was
    /// changed, in words (see [`resolved_overlaps`]).
    pub resolved_overlaps: Vec<String>,
    /// The names of the parts of the mesh, which are element sets.
    pub part_sets: Vec<String>,
    /// The profile of each general beam section, by its element set, as the comment the
    /// input file names it with (see [`super::PROFILE_COMMENT`]): the solver knows the
    /// section by its values only, the results draw it from the comment.
    pub profile_comments: Vec<(String, String)>,
    /// The item of the model each section, tie, rigid body, contact pair, material,
    /// amplitude and step of the analysis came from, for the solver's diagnostics.
    pub origins: Origins,
    /// The steps of the analysis (by index) that only prepare the step of the model they
    /// come from, e.g. the preload and the eigenmodes of a complex frequency step; their
    /// results are not kept.
    pub preparation_steps: Vec<usize>,
}

impl Lowered {
    /// The input file of the analysis: the comments naming the resolved overlaps, then the
    /// analysis rendered as text.
    pub fn text(&self) -> String {
        let mut out = String::new();
        for overlap in &self.resolved_overlaps {
            out.push_str(&format!("{}{overlap}\n", dofs::PREFIX));
        }
        let text = plxs_input::render_with_parts(&self.analysis, &self.part_sets);
        for line in text.split_inclusive('\n') {
            if let Some(rest) = line.strip_prefix("*Beam section, Elset=") {
                let set = rest.split(',').next().unwrap_or_default();
                for (_, comment) in (self.profile_comments.iter()).filter(|(s, _)| s == set) {
                    out.push_str(comment);
                    out.push('\n');
                }
            }
            out.push_str(line);
        }
        out
    }
}

/// Lowers the mesh and the analysis of `model` into the solver's input, validated. The
/// model is an [`Active`] one: what the analysis solves, without its suppressed objects.
pub fn lower(model: &Active, heading: &str) -> Result<Lowered, LowerError> {
    validated(lower_unvalidated(model, heading)?)
}

/// [`lower`] with the sets of `extra` steps too, see [`write_inp_with_sets_of`].
pub fn lower_with_sets_of(
    model: &Active,
    heading: &str,
    extra: &[(usize, Vec<Step>)],
) -> Result<Lowered, LowerError> {
    validated(lower_with_sets_unvalidated(model, heading, extra)?)
}

fn validated(mut lowered: Lowered) -> Result<Lowered, LowerError> {
    let warnings = ir::validate(&mut lowered.analysis).map_err(LowerError::Solver)?;
    lowered.warnings.extend(warnings);
    Ok(lowered)
}

/// The analysis before the solver's validation, for a run that validates it itself and
/// reports what it finds as its diagnostics.
pub fn lower_unvalidated(model: &Active, heading: &str) -> Result<Lowered, ModelError> {
    lower_whole(model.mesh(), model.fe(), heading)
}

/// [`lower_with_sets_of`] before the solver's validation, see [`lower_unvalidated`].
pub fn lower_with_sets_unvalidated(
    model: &Active,
    heading: &str,
    extra: &[(usize, Vec<Step>)],
) -> Result<Lowered, ModelError> {
    lower_with_sets_whole(model.mesh(), model.fe(), heading, extra)
}

/// Writes the mesh and the analysis as the text of an input file.
pub fn write_inp(model: &Active, heading: &str) -> Result<String, WriteError> {
    write_inp_whole(model.mesh(), model.fe(), heading)
}

/// Writes the input file of `model` with the sets of `extra` steps too, which are not
/// written themselves: the steps of analyses that continue from the state after a step of
/// this one. The solver takes no new sets after reading a state, so the run that writes it has
/// to define them. Each entry holds the index of the step of `model` the steps follow; they
/// take the place in the order of the sets they have in the input file of the analysis that
/// continues, so that the sets are named as there. When the extra steps cannot be written,
/// the file is written without their sets.
pub fn write_inp_with_sets_of(
    model: &Active,
    heading: &str,
    extra: &[(usize, Vec<Step>)],
) -> Result<String, WriteError> {
    Ok(lower_with_sets_unvalidated(model, heading, extra)
        .map_err(|e| e.error)?
        .text())
}

/// [`lower_unvalidated`] of all of `mesh` and `model`, for the tests and the mesh export,
/// which have nothing suppressed.
pub(crate) fn lower_whole(
    mesh: &FeMesh,
    model: &FeModel,
    heading: &str,
) -> Result<Lowered, ModelError> {
    Ok(lower_model(mesh, model, heading)?.assemble(|_| true))
}

/// [`lower_with_sets_unvalidated`] of all of `mesh` and `model`.
pub(crate) fn lower_with_sets_whole(
    mesh: &FeMesh,
    model: &FeModel,
    heading: &str,
    extra: &[(usize, Vec<Step>)],
) -> Result<Lowered, ModelError> {
    if extra.iter().all(|(_, steps)| steps.is_empty()) {
        return lower_whole(mesh, model, heading);
    }
    let mut combined = model.clone();
    combined.steps.clear();
    let mut written = Vec::new();
    for (index, step) in model.steps.iter().enumerate() {
        combined.steps.push(step.clone());
        written.push(true);
        for (_, steps) in extra.iter().filter(|(after, _)| *after == index) {
            combined.steps.extend(steps.iter().cloned());
            written.extend(steps.iter().map(|_| false));
        }
    }
    let Ok(built) = lower_model(mesh, &combined, heading) else {
        return lower_whole(mesh, model, heading);
    };
    Ok(built.assemble(|s| written.get(s).copied().unwrap_or(true)))
}

/// [`write_inp`] of all of `mesh` and `model`.
pub(crate) fn write_inp_whole(
    mesh: &FeMesh,
    model: &FeModel,
    heading: &str,
) -> Result<String, WriteError> {
    Ok(lower_whole(mesh, model, heading)
        .map_err(|e| e.error)?
        .text())
}

/// An input file with only the mesh, its nodes moved by `scale` times `displacements` (one per
/// node, in the order of [`FeMesh::coords`]), for "Export deformed mesh": the
/// deformed shape, e.g. a buckling mode as imperfection, can be imported into another model.
pub fn write_deformed_mesh_inp(
    mesh: &FeMesh,
    displacements: &[[f32; 3]],
    scale: f64,
    heading: &str,
) -> Result<String, WriteError> {
    let mut deformed = mesh.clone();
    for ((&id, coords), u) in mesh.node_ids().iter().zip(mesh.coords()).zip(displacements) {
        let moved = std::array::from_fn(|k| coords[k] + scale * f64::from(u[k]));
        deformed.set_node(id, moved);
    }
    write_inp_whole(&deformed, &FeModel::default(), heading)
}

/// Boundary conditions, loads, defined fields and outputs of a step, as its items give
/// them.
#[derive(Clone, Debug, Default)]
struct Content {
    boundaries: Vec<ir::step::Boundary>,
    loads: ir::step::Loads,
    fields: Vec<ir::step::DefinedField>,
    outputs: ir::step::Outputs,
}

impl Content {
    fn boundaries(items: Vec<ir::step::Boundary>) -> Self {
        Self {
            boundaries: items,
            ..Self::default()
        }
    }

    fn cloads(items: Vec<ir::step::ConcentratedLoad>) -> Self {
        let mut content = Self::default();
        content.loads.concentrated = items;
        content
    }

    fn dloads(items: Vec<ir::step::DistributedLoad>) -> Self {
        let mut content = Self::default();
        content.loads.distributed = items;
        content
    }

    fn cfluxes(items: Vec<ir::step::ConcentratedFlux>) -> Self {
        let mut content = Self::default();
        content.loads.concentrated_flux = items;
        content
    }

    fn films(items: Vec<ir::step::Film>) -> Self {
        let mut content = Self::default();
        content.loads.films = items;
        content
    }

    fn radiations(items: Vec<ir::step::Radiate>) -> Self {
        let mut content = Self::default();
        content.loads.radiations = items;
        content
    }

    /// Adds `other` after what the step has.
    fn add(&mut self, other: Content) {
        self.boundaries.extend(other.boundaries);
        self.loads.append(other.loads);
        self.fields.extend(other.fields);
        self.outputs.append(other.outputs);
    }

    /// The step `step` with this content.
    fn into_step(self, mut step: ir::Step) -> ir::Step {
        step.boundaries = self.boundaries;
        step.loads = self.loads;
        step.fields = self.fields;
        step.outputs = self.outputs;
        step
    }
}

/// What a step takes over from the steps before it, its own items added: the boundary
/// conditions of the step before, the loads and defined fields of the last step before
/// that was no perturbation (see [`ir::Step::handed_on`]). Everything else a step holds is
/// its own.
#[derive(Clone, Copy, Debug, Default)]
struct TakesOver {
    /// A step on stored eigenmodes keeps the supports they were computed with, and a
    /// sensitivity step those of the state it differentiates.
    boundaries: bool,
    /// A step without loads of its own (frequency, sensitivity) acts about the loads of
    /// the state before it: the centrifugal loads soften the stiffness, the sensitivity
    /// differentiates them.
    loads: bool,
    /// A step without defined temperatures of its own keeps those before it.
    fields: bool,
}

/// One step of the analysis: its settings with everything of its own, and what it takes
/// over from the steps before.
#[derive(Clone, Debug)]
struct StepBlock {
    step: ir::Step,
    /// Whether the step keeps its name; the blocks of a step with moving loads are named
    /// by their number, `Step-n`, as a step without a name.
    named: bool,
    takes_over: TakesOver,
    /// A step that only prepares the next one, such as the preload and the eigenmodes
    /// of a complex frequency step; its results are not kept.
    preparation: bool,
}

/// Everything of the analysis but the assembly of the steps.
struct Built {
    model: ir::Model,
    /// The blocks of each step of the model; none for a deactivated step.
    steps: Vec<Vec<StepBlock>>,
    resolved_overlaps: Vec<String>,
    part_sets: Vec<String>,
    profile_comments: Vec<(String, String)>,
    /// The origins of everything but the steps, which [`Built::assemble`] adds.
    origins: Origins,
}

impl Built {
    /// The analysis with the steps `written` takes, by their index in the model.
    fn assemble(self, written: impl Fn(usize) -> bool) -> Lowered {
        let model = self.model;
        let mut steps: Vec<ir::Step> = Vec::new();
        let mut origins = self.origins;
        let mut preparation_steps = Vec::new();
        // What the steps so far hand on to the next one.
        let mut handed = ir::step::HandedOn::default();
        // The steps written are those of the model, in order; the others are of the
        // analyses that continue from it.
        let mut model_step = 0;
        for (index, blocks) in self.steps.into_iter().enumerate() {
            if !written(index) {
                continue;
            }
            let origin = model_step;
            model_step += 1;
            for block in blocks {
                let mut step = block.step;
                if !block.named {
                    step.name = format!("Step-{}", steps.len() + 1);
                }
                let takes_over = block.takes_over;
                if takes_over.boundaries {
                    let own = std::mem::take(&mut step.boundaries);
                    step.boundaries = handed.boundaries.clone();
                    step.boundaries.extend(own);
                }
                if takes_over.loads {
                    let own = std::mem::take(&mut step.loads);
                    step.loads = handed.loads.clone();
                    // The rotation of the step replaces the one before.
                    if own.distributed.iter().any(|l| l.kind.is_centrifugal()) {
                        (step.loads.distributed).retain(|l| !l.kind.is_centrifugal());
                    }
                    step.loads.append(own);
                }
                if takes_over.fields {
                    let own = std::mem::take(&mut step.fields);
                    step.fields = handed.fields.clone();
                    step.fields.extend(own);
                }
                handed.follow(&model, &step);
                origins.insert(ir::ItemRef::Step(steps.len()), ModelItem::Step(origin));
                if block.preparation {
                    preparation_steps.push(steps.len());
                }
                steps.push(step);
            }
        }
        Lowered {
            warnings: Vec::new(),
            analysis: ir::Analysis { model, steps },
            resolved_overlaps: self.resolved_overlaps,
            part_sets: self.part_sets,
            profile_comments: self.profile_comments,
            origins,
            preparation_steps,
        }
    }
}

/// The name of a set, surface, material or amplitude as the solver compares it.
fn ir_name(name: &str) -> String {
    normalized_name(name)
}

/// The name of an item as the solver prints it, e.g. the set of a history output.
pub fn solver_name(item: &str) -> String {
    ir_name(&name(item))
}

/// The handles of the sets and surfaces by their names in the lowering: the names are
/// unique there (see [`Sets::free_name`]), the solver model refers to the sets by these
/// handles and keeps the names as labels. The members follow when the model is built.
#[derive(Default)]
struct Handles {
    node_sets: Vec<String>,
    element_sets: Vec<String>,
    surfaces: Vec<String>,
    index: HashMap<(u8, String), u32>,
}

impl Handles {
    fn slot(&mut self, kind: u8, name: &str) -> u32 {
        if let Some(&i) = self.index.get(&(kind, name.to_string())) {
            return i;
        }
        let names = match kind {
            0 => &mut self.node_sets,
            1 => &mut self.element_sets,
            _ => &mut self.surfaces,
        };
        names.push(name.to_string());
        let i = names.len() as u32 - 1;
        self.index.insert((kind, name.to_string()), i);
        i
    }

    fn node_set(&mut self, name: &str) -> ir::NodeSetId {
        ir::NodeSetId(self.slot(0, name))
    }

    fn element_set(&mut self, name: &str) -> ir::ElementSetId {
        ir::ElementSetId(self.slot(1, name))
    }

    fn surface(&mut self, name: &str) -> ir::SurfaceId {
        ir::SurfaceId(self.slot(2, name))
    }

    /// Empty sets and surfaces in `out` up to the handles given so far, labelled by their
    /// names.
    fn fill(&self, out: &mut ir::mesh::Sets) {
        for name in &self.node_sets[out.node_sets.len()..] {
            out.add_node_set(ir_name(name), []);
        }
        for name in &self.element_sets[out.element_sets.len()..] {
            out.add_element_set(ir_name(name), []);
        }
        for name in &self.surfaces[out.surfaces.len()..] {
            out.add_surface(ir_name(name), ir::mesh::Surface::Node(Vec::new()));
        }
    }
}

/// The name a step gets: its own, as the title comment before `*STEP` gives it, or the
/// number when the title has no room for its name.
fn step_title(name: &str) -> Option<String> {
    let title = format!("{:+<60}", format!("** {name} "));
    let text = title.strip_prefix("**")?.trim();
    text.ends_with('+')
        .then(|| text.trim_end_matches('+').trim().to_string())
        .filter(|t| !t.is_empty())
}

/// The analysis lowered from mesh and model, without the assembly of its steps; an error
/// names the item it is about.
fn lower_model(mesh: &FeMesh, model: &FeModel, heading: &str) -> Result<Built, ModelError> {
    let mut step = None;
    lower_model_named(mesh, model, heading, &mut step)
        .map_err(|error| ModelError::new(model, error, step))
}

/// [`lower_model`], keeping the index of the step being lowered in `lowering`.
fn lower_model_named(
    mesh: &FeMesh,
    model: &FeModel,
    heading: &str,
    lowering: &mut Option<usize>,
) -> Result<Built, WriteError> {
    let original = model;
    // The faces of 2D elements are their edges; a mesh whose surface elements are still
    // typed as shells is lowered as the model space types them. Parts get the element
    // types the user chose for them, and linear shells of membrane sections become
    // membranes.
    let space = model.properties.space;
    let retyped;
    let mesh = if !model.element_types.is_empty()
        || (model.sections.iter()).any(|s| matches!(s.kind, SectionKind::Membrane { .. }))
        || (mesh.elements().iter())
            .any(|e| space.element_type(&e.type_name, e.shape) != e.type_name)
    {
        let mut copy = mesh.clone();
        model.apply_element_types(&mut copy);
        model.apply_membrane_sections(&mut copy);
        space.convert_mesh(&mut copy);
        retyped = copy;
        &retyped
    } else {
        mesh
    };
    // General profiles, ropes among them, are sections of the 2-node user element U1:
    // quadratic lines under them are split in two at their middle node.
    let general_lines: BTreeSet<ElementId> = (model.sections.iter())
        .filter(|s| (model.section_profile(&s.kind)).is_some_and(|p| !p.is_native()))
        .flat_map(|s| s.region.elements(mesh))
        .filter(|&e| {
            mesh.element(e)
                .is_some_and(|e| e.shape == plx_mesh::ElementShape::Line3)
        })
        .collect();
    let halved;
    let mesh = if general_lines.is_empty() {
        mesh
    } else {
        let mut copy = mesh.clone();
        copy.split_quadratic_lines(&general_lines, "B31");
        halved = copy;
        &halved
    };
    // Bolts become pre-tension loads and ties, and switch off the contact regions on
    // their cuts.
    let (bolted, bolt_items);
    let model = if model.bolts.is_empty() {
        bolt_items = None;
        model
    } else {
        let (copy, items) = model.lower_bolts(mesh);
        (bolted, bolt_items) = (copy, Some(items));
        &bolted
    };
    // Contact regions become the ties and contact pairs they stand for.
    let (lowered, contact_items);
    let model = if model.contact_regions.is_empty() && !plx_model::has_compound_ties(mesh) {
        contact_items = None;
        model
    } else {
        let (copy, items) = model.lower_contacts(mesh);
        (lowered, contact_items) = (copy, Some(items));
        &lowered
    };
    // The item of the original model an item of the lowered one stands for: a tie or
    // contact pair of a contact region the region, one of a bolt the bolt, an item the
    // lowering added itself (a weak spring) none.
    let origin = |item: ModelItem| -> Option<ModelItem> {
        if let Some(items) = &contact_items {
            if let Some(region) = items.region_of(item) {
                return Some(ModelItem::ContactRegion(region));
            }
            if let Some(part) = items.part_of(item) {
                return Some(ModelItem::Part(part));
            }
            match item {
                ModelItem::Tie(i) if i >= items.first_tie => return None,
                ModelItem::ContactPair(i) if i >= items.first_pair => return None,
                ModelItem::ContactRegion(_) => return None,
                _ => {}
            }
        }
        if let Some(items) = &bolt_items {
            if let Some(bolt) = items.bolt_of(item) {
                return Some(ModelItem::Bolt(bolt));
            }
            if matches!(item, ModelItem::Tie(i) if i >= items.first_tie) {
                return None;
            }
        }
        match item {
            ModelItem::Constraint(i) if i >= original.constraints.len() => None,
            item => Some(item),
        }
    };
    // Parts held only by contact get weak springs to ground.
    let springs = model.weak_springs(mesh);
    let sprung;
    let model = if springs.is_empty() {
        model
    } else {
        let mut copy = model.clone();
        copy.constraints
            .extend(springs.into_iter().map(plx_model::Constraint::Spring));
        sprung = copy;
        &sprung
    };
    let lines = LineElements::new(mesh, model)?;
    // Node ties of beams, and of anything without rotations, become one shared node.
    let merged_nodes = node_merges(mesh, model, &lines);
    let (merged_mesh, merged_model);
    let (mesh, model) = if merged_nodes.is_empty() {
        (mesh, model)
    } else {
        let mut copy = mesh.clone();
        copy.merge_nodes(&merged_nodes);
        merged_mesh = copy;
        let mut copy = model.clone();
        copy.merge_nodes(&merged_nodes);
        merged_model = copy;
        (&merged_mesh, &merged_model)
    };
    // Bolt pre-tension cuts the mesh apart at its faces.
    let split_mesh;
    let (mesh, cuts) = match pre_tension::split(mesh, model)? {
        Some((split, cuts)) => {
            split_mesh = split;
            (&split_mesh, cuts)
        }
        None => (mesh, BTreeMap::new()),
    };
    let mut sets = Sets::new(mesh);
    sets.cut_originals = (cuts.values())
        .flat_map(|cut| cut.pairs.iter().map(|&(original, copy)| (copy, original)))
        .collect();
    let lines = LineElements::new(mesh, model)?;
    // The boundary conditions leave out the slave nodes of cyclic symmetries, which its MPCs
    // depend on; every other constraint keeps off what these two hold.
    let cyclic_slaves = cyclic_slave_nodes(mesh, model);
    let mut held = model.held_translations(mesh);
    held.retain(|n, _| !cyclic_slaves.contains(n));
    sets.dofs = dofs::Dofs::new(held);
    sets.dofs.mark_nodes(cyclic_slaves.iter().copied());
    sets.node_frames = node_frames::node_frames(model, mesh, |n| cyclic_slaves.contains(&n))?;
    // Regions are resolved first, in the order of the input file that the names of their
    // sets follow. The constraints come in their order of precedence for the dependent
    // degrees of freedom (see [`dofs`]): rigid bodies and remote points, pre-tension cuts,
    // node ties, last springs and contact regions.
    let (mut materials, material_ids) = materials(model);
    let mut orientations = Vec::new();
    let (mut sections, profile_comments, section_owners) = sections(
        &mut sets,
        model,
        &material_ids,
        &mut materials,
        &mut orientations,
    )?;
    let mut generated = constraints::Generated::default();
    let (mut constraints, mut constraint_owners) =
        constraints(&mut sets, model, &mut generated, lowering)?;
    let pre_tension_equations = pre_tension_sections(&mut sets, model, &cuts, &mut generated);
    let node_tie_equations = node_ties(&mut sets, model, &lines)?;
    constraints::springs(&mut sets, model, &mut generated)?;
    constraints.extend(
        (generated.equations.iter()).map(|terms| ConstraintItem::Equation(equation(terms))),
    );
    sections.extend((generated.sections.iter()).map(|g| generated_section(&mut sets, g)));
    materials.extend(generated.material.as_deref().map(gap_material));
    // The boundary condition holding the ground nodes of the gaps, in every mechanical step.
    let extra_boundary = (generated.boundary.as_ref()).map(|(set, dofs)| {
        Content::boundaries(
            (1..=*dofs as u8)
                .map(|dof| boundary(sets.node_ref(set), dof, dof, Some(0.0), None))
                .collect(),
        )
    });
    let young = if model.surface_interactions.is_empty() {
        Default::default()
    } else {
        ContactScale::element_young(model, mesh)
    };
    let interactions = (model.surface_interactions.iter())
        .map(|i| interaction(i, &InteractionDefaults::of(i, mesh, model, &young)))
        .collect();
    let (contact_pairs, clearances, mut pair_surfaces) = contact_pairs(&mut sets, model)?;
    // A compression only support prints its contact forces as the slave of a pair whose
    // master is the ground (no surface).
    for (id, surface) in &generated.support_surfaces {
        pair_surfaces.insert(*id, (String::new(), surface.clone()));
    }
    let initial_conditions = initial_conditions(&mut sets, model)?;
    let design_variables = design_variables(&mut sets, model)?;
    let (amplitudes, amplitude_ids) = amplitudes(model)?;
    let steps = (model.steps.iter().enumerate())
        .map(|(s, step)| {
            let context = StepContext {
                cyclic: (model.ties.iter()).any(|t| t.active && t.cyclic.is_some()),
                cyclic_slaves: &cyclic_slaves,
                extra_boundary: extra_boundary.as_ref(),
                space,
                lines: &lines,
                amplitudes: &amplitude_ids,
                pair_surfaces: &pair_surfaces,
                model,
            };
            *lowering = Some(s);
            lower_step(&mut sets, step, &context)
        })
        .collect::<Result<Vec<_>, _>>()?;
    *lowering = None;

    // The model in the order of the input file, which the solver reads it in: submodel,
    // nodes, elements, sets, surfaces, coordinate systems, materials, sections,
    // constraints, contact, amplitudes, initial conditions, design variables.
    let mut ir_model = ir::Model::default();
    ir_model.heading = heading
        .lines()
        .next()
        .unwrap_or_default()
        .trim()
        .to_string();
    ir_model.constraints.submodel = submodel(model, &mut sets.handles, &sets.submodel_sets)?;
    for (id, &coords) in mesh.node_ids().iter().zip(mesh.coords()) {
        ir_model.nodes.insert(*id, coords);
    }
    for &(id, coords) in &generated.nodes {
        ir_model.nodes.insert(id, coords);
    }
    let mut handles = std::mem::take(&mut sets.handles);
    elements(&mut ir_model, &mut handles, mesh, space, &lines)?;
    for block in &generated.elements {
        generated_elements(&mut ir_model, &mut handles, block);
    }
    // Every set and surface of the lowering, also the ones nothing refers to (the export
    // writes them).
    for (name, _) in &sets.node_sets {
        handles.node_set(name);
    }
    for (name, _) in &sets.element_sets {
        handles.element_set(name);
    }
    for (name, _) in &sets.surfaces {
        handles.surface(name);
    }
    handles.fill(&mut ir_model.sets);
    for (name, ids) in &sets.node_sets {
        let set = handles.node_set(name);
        ir_model.sets.add_nodes(set, ids.iter().copied());
    }
    for (name, members) in &sets.element_sets {
        let ids = match members {
            Members::Ids(ids) => ids.clone(),
            Members::Parts(parts) => (parts.iter())
                .flat_map(|p| ir_model.sets.elements(handles.element_set(p)).to_vec())
                .collect(),
        };
        let set = handles.element_set(name);
        ir_model.sets.add_elements(set, ids);
    }
    handles.fill(&mut ir_model.sets);
    for (name, surface) in &sets.surfaces {
        let surface = match surface {
            Surface::Faces(faces) => ir::mesh::Surface::Element(
                (faces.iter())
                    .flat_map(|(set, face)| {
                        let members = ir_model.sets.elements(handles.element_set(set));
                        (members.iter()).map(|&element| ir::mesh::SurfaceFace {
                            element,
                            face: *face,
                        })
                    })
                    .collect(),
            ),
            Surface::Nodes(set) => {
                ir::mesh::Surface::Node(ir_model.sets.nodes(handles.node_set(set)).to_vec())
            }
        };
        handles.fill(&mut ir_model.sets);
        let id = handles.surface(name);
        ir_model.sets.surfaces[id.index()].surface = surface;
    }
    sets.handles = handles;
    ir_model.physical = physical_constants(model);
    // Nodes held in different systems take a system of their own, defined last so that
    // it wins over those of the boundary conditions' sets.
    let mut node_frames: Vec<(node_frames::Axes, Vec<NodeId>)> = Vec::new();
    for (&node, axes) in &sets.node_frames {
        match node_frames.iter_mut().find(|(a, _)| a == axes) {
            Some((_, nodes)) => nodes.push(node),
            None => node_frames.push((*axes, vec![node])),
        }
    }
    for (axes, nodes) in node_frames {
        let set = sets.free_name("Internal_Selection", "Node_System");
        let id = sets.node_id(&set);
        sets.handles.fill(&mut ir_model.sets);
        ir_model.sets.add_nodes(id, nodes.iter().copied());
        sets.transforms
            .push((set, Transform::Rectangular(axes[0], axes[1])));
    }
    ir_model.constraints.transforms = transforms(&mut sets.handles, &sets.transforms);
    ir_model.materials = materials;
    ir_model.sections = sections;
    ir_model.orientations = orientations;
    let mut origins = Origins::default();
    for (i, &section) in section_owners.iter().enumerate() {
        if let Some(item) = origin(ModelItem::Section(section)) {
            origins.insert(ir::ItemRef::Section(i), item);
        }
    }
    let mut point_masses = Vec::new();
    let constraint_list = &mut ir_model.constraints;
    constraint_list.equations.extend(pre_tension_equations);
    // The equations generated after them have no item of their own.
    constraint_owners.resize(constraints.len(), None);
    for (item, owner) in constraints.into_iter().zip(constraint_owners) {
        match item {
            ConstraintItem::Equation(equation) => constraint_list.equations.push(equation),
            ConstraintItem::RigidBody(body) => {
                if let Some(item) = owner.and_then(origin) {
                    let index = constraint_list.rigid_bodies.len();
                    origins.insert(ir::ItemRef::RigidBody(index), item);
                }
                constraint_list.rigid_bodies.push(body);
            }
            ConstraintItem::Joint(joint) => {
                if let Some(item) = owner.and_then(origin) {
                    let id = ir::JointId::at(constraint_list.joints.len());
                    origins.insert(ir::ItemRef::Joint(id), item);
                }
                constraint_list.joints.push(joint);
            }
            ConstraintItem::PointMass(mass) => point_masses.push(mass),
            ConstraintItem::Tie(tie, cyclic) => {
                // The cyclic symmetry refers to its tie.
                let id = ir::TieId::at(constraint_list.ties.len());
                if let Some(item) = owner.and_then(origin) {
                    origins.insert(ir::ItemRef::Tie(id), item);
                }
                constraint_list.ties.push(tie);
                if let Some(mut cyclic) = cyclic {
                    cyclic.tie = id;
                    constraint_list.cyclic_symmetry = Some(cyclic);
                }
            }
        }
    }
    ir_model.point_masses = point_masses;
    let active_pairs = (model.contact_pairs.iter().enumerate()).filter(|(_, p)| p.active);
    for (index, (i, _)) in active_pairs.enumerate().take(contact_pairs.len()) {
        if let Some(item) = origin(ModelItem::ContactPair(i)) {
            origins.insert(ir::ItemRef::ContactPair(index), item);
        }
    }
    for (i, material) in original.materials.iter().enumerate() {
        if let Some(&id) = material_ids.get(&material.id) {
            origins.insert(ir::ItemRef::Material(id), ModelItem::Material(i));
        }
    }
    for (i, amplitude) in original.amplitudes.iter().enumerate() {
        if let Some(&id) = amplitude_ids.ids.get(&AmplitudeKey::User(amplitude.id)) {
            origins.insert(ir::ItemRef::Amplitude(id), ModelItem::Amplitude(i));
        }
    }
    for (s, step) in original.steps.iter().enumerate().filter(|(_, s)| s.active) {
        for table in step.tables() {
            let (_, item, label) = table_owner(step, &table);
            let Some(amplitude) = amplitude_ids.table(&step.name, item, label) else {
                continue;
            };
            origins.insert(
                ir::ItemRef::Amplitude(amplitude),
                match table.owner {
                    TableOwner::BoundaryCondition(i) => ModelItem::BoundaryCondition(s, i),
                    TableOwner::Load(i) => ModelItem::Load(s, i),
                },
            );
        }
    }
    ir_model.contact = ir::contact::Contact {
        interactions,
        pairs: contact_pairs,
        clearances,
        compression_only: std::mem::take(&mut generated.compression_only),
    };
    ir_model.constraints.equations.extend(node_tie_equations);
    ir_model.amplitudes = amplitudes;
    ir_model.initial = initial_conditions;
    ir_model.design_variables = design_variables;
    let resolved_overlaps = (sets.dofs.comments())
        .map(|comments| resolved_overlaps(&comments))
        .unwrap_or_default();
    Ok(Built {
        model: ir_model,
        steps,
        resolved_overlaps,
        part_sets: mesh.parts.iter().map(|p| p.name.clone()).collect(),
        profile_comments,
        origins,
    })
}

/// A constraint in the order of the input file.
enum ConstraintItem {
    Equation(ir::constraints::Equation),
    RigidBody(ir::constraints::RigidBody),
    Joint(ir::constraints::Joint),
    PointMass(ir::PointMass),
    Tie(
        ir::constraints::Tie,
        Option<ir::constraints::CyclicSymmetryModel>,
    ),
}

/// A local coordinate system of a node set.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Transform {
    /// Directions x and y.
    Rectangular([f64; 3], [f64; 3]),
    /// Two points on the axis, z from the first to the second: radial, tangential, axial.
    Cylindrical([f64; 3], [f64; 3]),
}

/// The local coordinate system of each node set that a skewed symmetry plane or a
/// displacement in a coordinate system of its own holds.
fn transforms(
    handles: &mut Handles,
    transforms: &[(String, Transform)],
) -> Vec<ir::constraints::Transform> {
    (transforms.iter())
        .map(|(set, transform)| {
            let (kind, a, b) = match *transform {
                Transform::Rectangular(a, b) => (ir::constraints::TransformKind::Rectangular, a, b),
                Transform::Cylindrical(a, b) => (ir::constraints::TransformKind::Cylindrical, a, b),
            };
            ir::constraints::Transform {
                nset: handles.node_set(set),
                kind,
                a,
                b,
            }
        })
        .collect()
}

/// The submodel with the node sets of the submodel boundary conditions. The global results
/// file is named without its directory; it has to be next to the input file, where the
/// analysis and the export put it.
fn submodel(
    model: &FeModel,
    handles: &mut Handles,
    node_sets: &[(String, String)],
) -> Result<Option<ir::constraints::Submodel>, WriteError> {
    let Some((_, first)) = node_sets.first() else {
        return Ok(None);
    };
    let input = (model.properties.submodel_input())
        .and_then(|path| path.file_name())
        .ok_or_else(|| WriteError::NoGlobalResults {
            item: first.clone(),
        })?;
    Ok(Some(ir::constraints::Submodel {
        global: ir::handover::Source::Path(input.into()),
        node_sets: (node_sets.iter())
            .map(|(set, _)| handles.node_set(set))
            .collect(),
        global_elset: None,
    }))
}

/// The line elements of beam and truss sections: the CalculiX type each one is written
/// with, and the nodes that have no rotations because they belong to trusses only.
struct LineElements {
    types: BTreeMap<ElementId, &'static str>,
    truss_nodes: BTreeSet<NodeId>,
    /// Nodes of beams, which have rotations.
    beam_nodes: BTreeSet<NodeId>,
}

impl LineElements {
    fn new(mesh: &FeMesh, model: &FeModel) -> Result<Self, WriteError> {
        let mut types = BTreeMap::new();
        for section in &model.sections {
            if let Some(reason) = section.kind_problem(model, mesh) {
                return Err(WriteError::InvalidSection {
                    item: section.name.clone(),
                    reason,
                });
            }
            if !section.kind.is_line() {
                continue;
            }
            for element in
                (section.region.elements(mesh).into_iter()).filter_map(|id| mesh.element(id))
            {
                if let Some(type_name) =
                    (section.kind).element_type(element.shape, model.section_profile(&section.kind))
                {
                    types.insert(element.id, type_name);
                }
            }
        }
        // A node of a truss has translations only; a beam or solid at the same node adds
        // its rotations back. Without line sections there are neither.
        if types.is_empty() {
            return Ok(Self {
                types,
                truss_nodes: BTreeSet::new(),
                beam_nodes: BTreeSet::new(),
            });
        }
        let mut truss_nodes = BTreeSet::new();
        let mut other_nodes = BTreeSet::new();
        let mut beam_nodes = BTreeSet::new();
        for element in mesh.elements() {
            let type_name = types.get(&element.id).copied();
            let nodes = if type_name == Some("T3D2") {
                &mut truss_nodes
            } else {
                &mut other_nodes
            };
            nodes.extend(element.nodes.iter().copied());
            if type_name.is_some_and(|t| t.starts_with('B') || t == "U1") {
                beam_nodes.extend(element.nodes.iter().copied());
            }
        }
        truss_nodes.retain(|n| !other_nodes.contains(n));
        Ok(Self {
            types,
            truss_nodes,
            beam_nodes,
        })
    }

    /// Type and nodes an element is written with. A truss is always `T3D2`, so a 3-node line
    /// keeps its end nodes only; see [`SectionKind::element_type`].
    fn written(&self, element: &plx_mesh::Element, space: ModelSpace) -> (String, Vec<NodeId>) {
        match self.types.get(&element.id) {
            Some(&"T3D2") if element.nodes.len() == 3 => {
                ("T3D2".into(), vec![element.nodes[0], element.nodes[2]])
            }
            Some(&type_name) => (type_name.into(), element.nodes.clone()),
            None => (
                space.element_type(&element.type_name, element.shape),
                element.nodes.clone(),
            ),
        }
    }

    /// Whether every node of the list belongs to trusses only.
    fn all_truss(&self, nodes: &[NodeId]) -> bool {
        !nodes.is_empty() && nodes.iter().all(|n| self.truss_nodes.contains(n))
    }
}

/// The elements, per part and element type; the part name is their element set. Surface
/// elements get the type of the model space, e.g. `CAX6` in an axisymmetric model, line
/// elements the type of their beam or truss section.
fn elements(
    out: &mut ir::Model,
    handles: &mut Handles,
    mesh: &FeMesh,
    space: ModelSpace,
    lines: &LineElements,
) -> Result<(), WriteError> {
    let mut written = vec![false; mesh.element_count()];
    let mut groups: Vec<(Option<&str>, String, Vec<ElementId>)> = Vec::new();
    let type_name = |element: &plx_mesh::Element| lines.written(element, space).0;
    for part in &mesh.parts {
        for &id in &part.elements {
            let Some(index) = mesh.element_index(id) else {
                continue;
            };
            if !std::mem::replace(&mut written[index], true) {
                let type_name = type_name(&mesh.elements()[index]);
                group(&mut groups, Some(&part.name), type_name).push(id);
            }
        }
    }
    for (element, done) in mesh.elements().iter().zip(&written) {
        if !done {
            group(&mut groups, None, type_name(element)).push(element.id);
        }
    }
    for (set, type_name, mut ids) in groups {
        let kind = element_type(set.unwrap_or("Mesh"), &type_name)?;
        ids.sort_unstable();
        for element in ids.iter().filter_map(|&id| mesh.element(id)) {
            let (_, nodes) = lines.written(element, space);
            out.insert_element(ir::element::Element {
                id: element.id,
                kind,
                nodes,
                section: None,
            });
        }
        if let Some(set) = set {
            let set = handles.element_set(set);
            handles.fill(&mut out.sets);
            out.sets.add_elements(set, ids);
        }
    }
    Ok(())
}

/// The solver's element type of a type name.
fn element_type(item: &str, type_name: &str) -> Result<ir::element::ElementType, WriteError> {
    ir::element::ElementType::parse(type_name).ok_or_else(|| WriteError::Unsupported {
        item: item.to_owned(),
        reason: format!("the solver has no element type {type_name}"),
    })
}

/// The spring and gap elements of constraints.
fn generated_elements(
    out: &mut ir::Model,
    handles: &mut Handles,
    block: &constraints::ElementBlock,
) {
    let kind = element_type(block.kind, block.kind).expect("a spring or gap type");
    for (id, nodes) in &block.elements {
        out.insert_element(ir::element::Element {
            id: *id,
            kind,
            nodes: nodes.clone(),
            section: None,
        });
    }
    if let Some(set) = &block.elset {
        let ids = block.elements.iter().map(|(id, _)| *id);
        let set = handles.element_set(set);
        handles.fill(&mut out.sets);
        out.sets.add_elements(set, ids);
    }
}

fn group<'a, 'b>(
    groups: &'b mut Vec<(Option<&'a str>, String, Vec<ElementId>)>,
    set: Option<&'a str>,
    type_name: String,
) -> &'b mut Vec<ElementId> {
    let index = match groups.iter().position(|g| g.0 == set && g.1 == type_name) {
        Some(index) => index,
        None => {
            groups.push((set, type_name, Vec::new()));
            groups.len() - 1
        }
    };
    &mut groups[index].2
}

enum Members {
    Ids(Vec<ElementId>),
    /// Element sets of whole parts, written by part name.
    Parts(Vec<String>),
}

enum Surface {
    /// Element set and face number per face side.
    Faces(Vec<(String, u8)>),
    /// A node set.
    Nodes(String),
}

/// All sets of the file: those read with the mesh and those derived from regions.
struct Sets<'a> {
    mesh: &'a FeMesh,
    node_sets: Vec<(String, Vec<NodeId>)>,
    element_sets: Vec<(String, Members)>,
    surfaces: Vec<(String, Surface)>,
    /// Upper-case names in use; CalculiX names are case-insensitive.
    used: BTreeSet<String>,
    /// Face element sets and node set of each element surface.
    surface_sets: BTreeMap<String, (Vec<(String, u8)>, String)>,
    /// The pre-tension node of each pre-tension load and the node set by which its force
    /// and shortening are printed, by the load's name.
    pre_tension_nodes: BTreeMap<String, NodeId>,
    pre_tension_sets: BTreeMap<String, String>,
    /// The original node of each copy a pre-tension cut made.
    cut_originals: BTreeMap<NodeId, NodeId>,
    /// Node sets of submodel boundary conditions, with the first boundary condition naming
    /// each; `*SUBMODEL` lists them.
    submodel_sets: Vec<(String, String)>,
    /// Reference and rotation node of each reference point an active rigid body is driven
    /// by, and the node set holding the reference node once something refers to it.
    reference_nodes: BTreeMap<Id<plx_model::ReferencePoint>, ReferenceNodes>,
    /// The name (`*POINT MASS, NAME=`) of the point masses on each reference point, by which
    /// a gravity or centrifugal load on the point pulls them alone.
    point_mass_names: BTreeMap<Id<plx_model::ReferencePoint>, String>,
    /// Reference and rotation node of the point of each displacement written with a point,
    /// with its region; boundary conditions with the same region and point share it.
    remote_points: Vec<(Region, plx_model::Displacement, [NodeId; 2])>,
    /// Reference and rotation node of the point of each displacement written with a point by
    /// the boundary condition's id, and of each remote force by the load's id, with its
    /// behavior.
    remote_names: BTreeMap<Id, ([NodeId; 2], RemoteBehavior)>,
    /// Local coordinate systems (`*Transform`) of node sets whose symmetry plane is no
    /// coordinate plane or whose displacement is in a system of its own: the set with the
    /// local x and y axes (type R), or with two points on the axis (type C).
    transforms: Vec<(String, Transform)>,
    /// Nodes that boundary conditions hold in different coordinate systems, with the
    /// system each gets, see [`node_frames`].
    node_frames: BTreeMap<NodeId, node_frames::Axes>,
    /// The dependent degrees of freedom of all MPCs so far.
    dofs: dofs::Dofs,
    /// Reference points fixed to the world ([`FeModel::point_grounded`]): their nodes in
    /// `reference_nodes` are held in every mechanical step where no displacement moves them.
    grounded: Vec<Id<plx_model::ReferencePoint>>,
    /// The handles of the sets and surfaces by name.
    handles: Handles,
}

/// The nodes CalculiX's `*Rigid body` adds for a reference point: the reference node takes
/// the translations, the rotation node the rotations as its displacements.
#[derive(Clone, Debug)]
struct ReferenceNodes {
    /// The name of the reference point, for the name of its set.
    name: String,
    reference: NodeId,
    rotation: NodeId,
    set: Option<String>,
    /// How the selection follows the point.
    behavior: RemoteBehavior,
}

impl<'a> Sets<'a> {
    fn new(mesh: &'a FeMesh) -> Self {
        let mut sets = Self {
            mesh,
            node_sets: Vec::new(),
            element_sets: Vec::new(),
            surfaces: Vec::new(),
            used: mesh
                .parts
                .iter()
                .map(|p| p.name.to_ascii_uppercase())
                .chain(mesh.node_sets.keys().map(|n| n.to_ascii_uppercase()))
                .chain(mesh.element_sets.keys().map(|n| n.to_ascii_uppercase()))
                .chain(mesh.surfaces.keys().map(|n| n.to_ascii_uppercase()))
                .collect(),
            surface_sets: BTreeMap::new(),
            reference_nodes: BTreeMap::new(),
            point_mass_names: BTreeMap::new(),
            remote_points: Vec::new(),
            remote_names: BTreeMap::new(),
            pre_tension_nodes: BTreeMap::new(),
            pre_tension_sets: BTreeMap::new(),
            cut_originals: BTreeMap::new(),
            submodel_sets: Vec::new(),
            transforms: Vec::new(),
            node_frames: BTreeMap::new(),
            dofs: dofs::Dofs::default(),
            grounded: Vec::new(),
            handles: Handles::default(),
        };
        for (name, ids) in &mesh.node_sets {
            sets.node_sets.push((name.clone(), ids.clone()));
        }
        for (name, ids) in &mesh.element_sets {
            // Part sets come from the element blocks; only extra members are written.
            let part = mesh
                .parts
                .iter()
                .find(|p| p.name.eq_ignore_ascii_case(name));
            // A lookup, as a part's set holds all its elements: searching the part's list for
            // each one takes seconds on a large mesh.
            let in_part: std::collections::HashSet<ElementId> = part
                .map(|p| p.elements.iter().copied().collect())
                .unwrap_or_default();
            let extra: Vec<ElementId> = ids
                .iter()
                .copied()
                .filter(|id| !in_part.contains(id))
                .collect();
            if part.is_none() || !extra.is_empty() {
                sets.element_sets.push((name.clone(), Members::Ids(extra)));
            }
        }
        for (name, surface) in &mesh.surfaces {
            match surface {
                SurfaceDefinition::ElementFaces(faces) => sets.add_face_surface(name, faces),
                SurfaceDefinition::Nodes(nodes) => {
                    let set = sets.free_name("Internal", name);
                    sets.node_sets.push((set.clone(), nodes.clone()));
                    sets.surfaces.push((name.clone(), Surface::Nodes(set)));
                }
            }
        }
        sets
    }

    /// The node set `name` as the solver model refers to it.
    fn node_id(&mut self, name: &str) -> ir::NodeSetId {
        self.handles.node_set(name)
    }

    fn element_id(&mut self, name: &str) -> ir::ElementSetId {
        self.handles.element_set(name)
    }

    fn surface_id(&mut self, name: &str) -> ir::SurfaceId {
        self.handles.surface(name)
    }

    /// The node set of a region as the solver model refers to it.
    fn region_ref(&mut self, item: &str, region: &Region) -> Result<NodeRef, WriteError> {
        let set = self.node_set(item, region)?;
        Ok(self.node_ref(&set))
    }

    fn node_ref(&mut self, name: &str) -> NodeRef {
        NodeRef::Set(self.node_id(name))
    }

    fn element_ref(&mut self, name: &str) -> ElementRef {
        ElementRef::Set(self.element_id(name))
    }

    /// `name` itself if it is free, otherwise the next free `<name>-<n>`.
    fn unique(&mut self, name: &str) -> String {
        if self.used.insert(name.to_ascii_uppercase()) {
            return name.to_owned();
        }
        (2..)
            .map(|n| format!("{name}-{n}"))
            .find(|name| self.used.insert(name.to_ascii_uppercase()))
            .expect("unbounded range")
    }

    /// Next free name `<prefix>-<n>`.
    fn unique_numbered(&mut self, prefix: &str) -> String {
        (1..)
            .map(|n| format!("{prefix}-{n}"))
            .find(|name| self.used.insert(name.to_ascii_uppercase()))
            .expect("unbounded range")
    }

    /// Next free name `<prefix>-<n>_<postfix>`.
    fn free_name(&mut self, prefix: &str, postfix: &str) -> String {
        (1..)
            .map(|n| format!("{prefix}-{n}_{postfix}"))
            .find(|name| self.used.insert(name.to_ascii_uppercase()))
            .expect("unbounded range")
    }

    /// An element surface: one element set per face number and a node
    /// set with all face nodes for boundary conditions.
    fn add_face_surface(&mut self, name: &str, faces: &[(ElementId, u8)]) {
        let mut by_face: BTreeMap<u8, BTreeSet<ElementId>> = BTreeMap::new();
        for &(element, face) in faces {
            by_face.entry(face).or_default().insert(element);
        }
        let base = self.free_name("Internal", name);
        let mut sides = Vec::new();
        for (face, ids) in by_face {
            let set = format!("{base}_S{face}");
            self.used.insert(set.to_ascii_uppercase());
            self.element_sets
                .push((set.clone(), Members::Ids(ids.into_iter().collect())));
            sides.push((set, face));
        }
        let nodes = Region::Faces(faces.to_vec()).nodes(self.mesh);
        self.node_sets.push((base.clone(), nodes));
        self.surfaces
            .push((name.to_owned(), Surface::Faces(sides.clone())));
        self.surface_sets.insert(name.to_owned(), (sides, base));
    }

    /// Element set holding the elements of a region.
    fn element_set(&mut self, item: &str, region: &Region) -> Result<String, WriteError> {
        let named = region.mesh_set(self.mesh);
        let region = named.as_ref().unwrap_or(region);
        let members = match region {
            Region::ElementSet(set) => return Ok(set.clone()),
            Region::Parts(parts) if !parts.is_empty() => Members::Parts(
                (parts.iter())
                    .filter_map(|&id| self.mesh.part(id))
                    .map(|p| p.name.clone())
                    .collect(),
            ),
            _ => {
                let ids = region.elements(self.mesh);
                if ids.is_empty() {
                    return Err(empty(item, "elements"));
                }
                Members::Ids(ids)
            }
        };
        let set = self.free_name("Internal_Selection", &name(item));
        self.element_sets.push((set.clone(), members));
        Ok(set)
    }

    /// The target of a gravity or centrifugal load: the element set of its region, or the
    /// name of the point masses on its reference point.
    fn body_load_set(&mut self, item: &str, region: &Region) -> Result<String, WriteError> {
        match region {
            Region::ReferencePoint(point) => (self.point_mass_names.get(point).cloned())
                .ok_or_else(|| WriteError::InvalidLoad {
                    item: item.to_string(),
                    reason: "no point mass sits on the reference point".into(),
                }),
            _ => self.element_set(item, region),
        }
    }

    /// The nodes of the rigid body a reference point drives.
    fn reference_nodes(
        &mut self,
        item: &str,
        point: Id<plx_model::ReferencePoint>,
    ) -> Result<ReferenceNodes, WriteError> {
        let Some(nodes) = self.reference_nodes.get(&point).cloned() else {
            return Err(WriteError::UnknownReferencePoint {
                item: item.to_string(),
                point: point.to_string(),
            });
        };
        if nodes.set.is_none() {
            let set = self.free_name("Internal_Selection", &name(&nodes.name));
            self.node_sets.push((set.clone(), vec![nodes.reference]));
            self.reference_nodes.get_mut(&point).unwrap().set = Some(set);
        }
        Ok(self.reference_nodes[&point].clone())
    }

    /// Node set holding the nodes of a region.
    fn node_set(&mut self, item: &str, region: &Region) -> Result<String, WriteError> {
        let named = region.mesh_set(self.mesh);
        let region = named.as_ref().unwrap_or(region);
        match region {
            Region::NodeSet(set) => return Ok(set.clone()),
            Region::ReferencePoint(point) => {
                return Ok(self.reference_nodes(item, *point)?.set.unwrap());
            }
            Region::Surface(surface) => {
                if let Some((_, nodes)) = self.surface_sets.get(surface) {
                    return Ok(nodes.clone());
                }
            }
            _ => {}
        }
        let ids = region.nodes(self.mesh);
        if ids.is_empty() {
            return Err(empty(item, "nodes"));
        }
        let set = self.free_name("Internal_Selection", &name(item));
        self.node_sets.push((set.clone(), ids));
        Ok(set)
    }

    /// Surface of a contact or tie side: a surface of the input file, or one built from
    /// picked faces and named like `Internal_Selection-1_Tie-1_Master`.
    fn surface(&mut self, item: &str, side: &str, region: &Region) -> Result<String, WriteError> {
        let named = region.mesh_set(self.mesh);
        let region = named.as_ref().unwrap_or(region);
        match region {
            Region::Surface(surface) if self.mesh.surfaces.contains_key(surface) => {
                Ok(surface.clone())
            }
            Region::Surface(surface) => Err(WriteError::UnknownSurface {
                item: item.to_owned(),
                surface: surface.clone(),
            }),
            Region::Faces(_) | Region::Geometry(_) => {
                let faces = region.faces(self.mesh);
                if faces.is_empty() {
                    return Err(empty(item, "element faces"));
                }
                let postfix = format!("{}_{side}", name(item));
                let surface = self.free_name("Internal_Selection", &postfix);
                self.add_face_surface(&surface, &faces);
                Ok(surface)
            }
            _ => Err(empty(item, "element faces")),
        }
    }

    /// A node surface of the nodes of a region, named like [`Self::surface`].
    fn node_surface(
        &mut self,
        item: &str,
        side: &str,
        region: &Region,
    ) -> Result<String, WriteError> {
        let ids = region.nodes(self.mesh);
        if ids.is_empty() {
            return Err(empty(item, "nodes"));
        }
        let postfix = format!("{}_{side}", name(item));
        let surface = self.free_name("Internal_Selection", &postfix);
        let set = self.free_name("Internal", &surface);
        self.node_sets.push((set.clone(), ids));
        self.surfaces.push((surface.clone(), Surface::Nodes(set)));
        Ok(surface)
    }

    /// Element sets and face numbers of a face region, for pressure loads.
    fn face_sets(&mut self, item: &str, region: &Region) -> Result<Vec<(String, u8)>, WriteError> {
        let named = region.mesh_set(self.mesh);
        let region = named.as_ref().unwrap_or(region);
        match region {
            Region::Surface(surface) => {
                if let Some((sides, _)) = self.surface_sets.get(surface) {
                    return Ok(sides.clone());
                }
            }
            Region::Faces(_) | Region::Geometry(_) => {
                let faces = region.faces(self.mesh);
                if !faces.is_empty() {
                    let surface = self.free_name("Internal_Selection", &name(item));
                    self.add_face_surface(&surface, &faces);
                    return Ok(self.surface_sets[&surface].0.clone());
                }
            }
            _ => {}
        }
        Err(empty(item, "element faces"))
    }
}

/// Whether a face region holds edges (S3 and up) of membrane elements.
fn membrane_edges(mesh: &FeMesh, region: &Region) -> bool {
    (region.faces(mesh).iter()).any(|&(element, face)| {
        face >= 3
            && mesh.element(element).is_some_and(|e| {
                e.is_shell() && e.type_name.to_ascii_uppercase().starts_with("M3D")
            })
    })
}

/// A value that does not depend on the temperature.
fn constant(value: f64) -> Vec<ir::material::TemperatureValue> {
    vec![ir::material::TemperatureValue::constant(value)]
}

/// The materials of the model, each with its handle: two materials stay two, whatever
/// their names.
fn materials(
    model: &FeModel,
) -> (
    Vec<ir::material::Material>,
    std::collections::HashMap<Id<plx_model::Material>, ir::MaterialId>,
) {
    use ir::material as m;
    let mut materials: Vec<m::Material> = Vec::new();
    let mut ids = std::collections::HashMap::new();
    for material in &model.materials {
        let mut out = m::Material {
            name: material.name.clone(),
            ..Default::default()
        };
        out.density = (material.density).map(|rho| m::Density {
            rows: constant(rho),
        });
        // A hyperelastic or anisotropic material replaces the linear elasticity.
        out.anisotropic = (material.anisotropic.as_ref())
            .filter(|_| material.hyperelastic.is_none())
            .map(plx_model::Anisotropic::solver);
        out.elastic = (material.elastic)
            .filter(|_| material.hyperelastic.is_none() && material.anisotropic.is_none())
            .map(|elastic| m::Elastic {
                rows: vec![m::ElasticRow {
                    young: elastic.young,
                    poisson: elastic.poisson,
                    temperature: 0.0,
                }],
            });
        out.hyperelastic = material.hyperelastic.as_ref().map(|hyper| m::Hyperelastic {
            model: match hyper.model {
                plx_model::HyperelasticModel::NeoHooke => m::HyperelasticModel::NeoHooke,
                plx_model::HyperelasticModel::MooneyRivlin => m::HyperelasticModel::MooneyRivlin,
                plx_model::HyperelasticModel::Yeoh => m::HyperelasticModel::Yeoh,
                plx_model::HyperelasticModel::Polynomial2 => m::HyperelasticModel::Polynomial(2),
                plx_model::HyperelasticModel::Ogden(n) => {
                    m::HyperelasticModel::Ogden(n.clamp(1, 3))
                }
                plx_model::HyperelasticModel::ArrudaBoyce => m::HyperelasticModel::ArrudaBoyce,
            },
            constants: hyper.solver_constants(),
        });
        out.plastic = material.plastic.as_ref().map(|plastic| m::Plastic {
            hardening: match plastic.hardening {
                plx_model::Hardening::Isotropic => m::Hardening::Isotropic,
                plx_model::Hardening::Kinematic => m::Hardening::Kinematic,
                plx_model::Hardening::Combined => m::Hardening::Combined,
            },
            rows: (plastic.points.iter())
                .map(|point| m::PlasticRow {
                    stress: point.stress,
                    plastic_strain: point.plastic_strain,
                    temperature: point.temperature,
                })
                .collect(),
        });
        out.creep = material.creep.map(|creep| m::Creep {
            rows: vec![m::CreepRow {
                a: creep.a,
                n: creep.n,
                m: creep.m,
                temperature: 0.0,
            }],
        });
        out.expansion = material.expansion.map(|expansion| m::Expansion {
            zero: expansion.zero_temperature,
            rows: constant(expansion.coefficient),
        });
        out.conductivity = (material.conductivity).map(|k| m::Conductivity { rows: constant(k) });
        out.specific_heat = (material.specific_heat).map(|c| m::SpecificHeat { rows: constant(c) });
        ids.insert(material.id, ir::MaterialId::at(materials.len()));
        materials.push(out);
    }
    (materials, ids)
}

/// The dummy plastic material that makes the solver solve gaps nonlinearly.
fn gap_material(name: &str) -> ir::material::Material {
    use ir::material as m;
    m::Material {
        name: name.to_owned(),
        density: Some(m::Density {
            rows: constant(1.0),
        }),
        elastic: Some(m::Elastic {
            rows: vec![m::ElasticRow {
                young: 1.0,
                poisson: 0.0,
                temperature: 0.0,
            }],
        }),
        plastic: Some(m::Plastic {
            hardening: m::Hardening::Isotropic,
            rows: vec![m::PlasticRow {
                stress: 0.0,
                plastic_strain: 0.0,
                temperature: 0.0,
            }],
        }),
        ..Default::default()
    }
}

/// The physical constants.
fn physical_constants(model: &FeModel) -> ir::PhysicalConstants {
    ir::PhysicalConstants {
        absolute_zero: model.properties.absolute_zero,
        stefan_boltzmann: model.properties.stefan_boltzmann,
        newton_gravity: None,
    }
}

/// The amplitudes.
fn amplitudes(
    model: &FeModel,
) -> Result<(Vec<ir::amplitude::Amplitude>, AmplitudeIds), WriteError> {
    let mut amplitudes = Vec::new();
    let mut ids = AmplitudeIds::default();
    for amplitude in &model.amplitudes {
        if let Some(reason) = amplitude.points_problem() {
            return Err(WriteError::InvalidAmplitude {
                item: amplitude.name.clone(),
                reason,
            });
        }
        let ir_amplitude = ir::amplitude::Amplitude {
            name: amplitude.name.clone(),
            total_time: amplitude.time_span == AmplitudeTime::Total,
            shift_x: amplitude.shift_time,
            shift_y: amplitude.shift_amplitude,
            points: amplitude.points.iter().map(|&[t, a]| (t, a)).collect(),
            signal: None,
        };
        ids.add(
            &mut amplitudes,
            AmplitudeKey::User(amplitude.id),
            ir_amplitude,
        );
    }
    // Each time table of a component is an amplitude of its own, in step time, that a value
    // of 1 follows.
    for step in model.steps.iter().filter(|s| s.active) {
        for table in step.tables() {
            let (item, item_id, label) = table_owner(step, &table);
            if let Some(reason) = table.table.points_problem() {
                return Err(WriteError::InvalidAmplitude {
                    item: item.to_owned(),
                    reason,
                });
            }
            let amplitude = ir::amplitude::Amplitude {
                name: format!("{}_{item}_{label}", step.name),
                total_time: false,
                shift_x: 0.0,
                shift_y: 0.0,
                points: table.table.points.iter().map(|&[t, v]| (t, v)).collect(),
                signal: None,
            };
            let key = AmplitudeKey::Table(step.name.clone(), item_id, label);
            ids.add(&mut amplitudes, key, amplitude);
        }
        // Wall loads over time: an amplitude per frame of the run, 1 at its time and 0 at
        // the frames before and after, so that the loads go linearly from frame to frame.
        for load in step.loads.iter().filter(|l| l.active) {
            let LoadKind::WallLoads(walls) = &load.kind else {
                continue;
            };
            if walls.time != plx_model::WallLoadTime::OverTime {
                continue;
            }
            let Some(file) = walls.file() else {
                continue;
            };
            let loads =
                crate::wall_loads::read(file).map_err(|reason| WriteError::InvalidLoad {
                    item: load.name.clone(),
                    reason,
                })?;
            let times = &loads.times;
            for k in 0..times.len() {
                let before = if k == 0 { 0.0 } else { times[k - 1] };
                let mut points = vec![(before, 0.0), (times[k], 1.0)];
                if k == 0 && before == times[k] {
                    points.remove(0);
                }
                if let Some(&next) = times.get(k + 1) {
                    points.push((next, 0.0));
                }
                let amplitude = ir::amplitude::Amplitude {
                    name: format!("{}_{}_{k}", step.name, load.name),
                    total_time: false,
                    shift_x: 0.0,
                    shift_y: 0.0,
                    points,
                    signal: None,
                };
                let key = AmplitudeKey::Walls(step.name.clone(), load.id.any(), k);
                ids.add(&mut amplitudes, key, amplitude);
            }
        }
        // The signal of each excitation is an amplitude of its own, in step time, that the
        // solver evaluates at every time step.
        let StepKind::Dynamic(dynamic) = &step.kind else {
            continue;
        };
        for load in step.loads.iter().filter(|l| l.active) {
            let LoadKind::Excitation(excitation) = &load.kind else {
                continue;
            };
            if let Some(reason) = excitation.signal.problem(dynamic.duration) {
                return Err(WriteError::InvalidAmplitude {
                    item: load.name.clone(),
                    reason,
                });
            }
            let amplitude = ir::amplitude::Amplitude {
                name: format!("{}_{}", step.name, load.name),
                total_time: false,
                shift_x: 0.0,
                shift_y: 0.0,
                points: Vec::new(),
                signal: Some(excitation.signal.to_ir(dynamic.duration)),
            };
            let key = AmplitudeKey::Signal(step.name.clone(), load.id.any());
            ids.add(&mut amplitudes, key, amplitude);
        }
    }
    Ok((amplitudes, ids))
}

/// The labels of the degrees of freedom of a displacement and the components of a load,
/// which name the amplitudes of their tables.
const DOF_TABLE_LABELS: [&str; 6] = ["U1", "U2", "U3", "UR1", "UR2", "UR3"];
const FORCE_TABLE_LABELS: [&str; 3] = ["F1", "F2", "F3"];
const MOMENT_TABLE_LABELS: [&str; 3] = ["M1", "M2", "M3"];

/// The name and id of the item a table belongs to and the label of its component.
fn table_owner<'a>(step: &'a Step, table: &plx_model::StepTable) -> (&'a str, Id, &'static str) {
    match table.owner {
        TableOwner::BoundaryCondition(i) => (
            &step.boundary_conditions[i].name,
            step.boundary_conditions[i].id.any(),
            DOF_TABLE_LABELS[table.component],
        ),
        TableOwner::Load(i) => {
            let load = &step.loads[i];
            let label = match load.kind {
                LoadKind::Moment(_) => MOMENT_TABLE_LABELS[table.component],
                LoadKind::Pressure(_) => "P",
                _ => FORCE_TABLE_LABELS[table.component],
            };
            (&load.name, load.id.any(), label)
        }
    }
}

/// What an amplitude of the analysis stands for: one of the model, or one the lowering
/// makes for an item of a step (the step by its name, unique in the model; the item by its
/// id, which a copied step shares with its source).
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
enum AmplitudeKey {
    User(Id<plx_model::Amplitude>),
    /// The time table of component `label` of a boundary condition or load.
    Table(String, Id, &'static str),
    /// Frame `frame` of wall loads.
    Walls(String, Id, usize),
    /// The signal of an excitation.
    Signal(String, Id),
}

/// The handles of the amplitudes of the analysis.
#[derive(Default)]
struct AmplitudeIds {
    ids: std::collections::HashMap<AmplitudeKey, ir::AmplitudeId>,
}

impl AmplitudeIds {
    /// Adds `amplitude` for `key` to `list`; a key given before keeps its amplitude.
    fn add(
        &mut self,
        list: &mut Vec<ir::amplitude::Amplitude>,
        key: AmplitudeKey,
        amplitude: ir::amplitude::Amplitude,
    ) -> ir::AmplitudeId {
        *self.ids.entry(key).or_insert_with(|| {
            list.push(amplitude);
            ir::AmplitudeId::at(list.len() - 1)
        })
    }

    /// The amplitude of the table of component `label` of item `item` in step `step`.
    fn table(&self, step: &str, item: Id, label: &'static str) -> Option<ir::AmplitudeId> {
        let key = AmplitudeKey::Table(step.to_owned(), item, label);
        self.ids.get(&key).copied()
    }

    /// The amplitude of frame `frame` of the wall loads `item` in step `step`.
    fn walls(&self, step: &str, item: Id, frame: usize) -> Option<ir::AmplitudeId> {
        let key = AmplitudeKey::Walls(step.to_owned(), item, frame);
        self.ids.get(&key).copied()
    }

    /// The amplitude of the signal of excitation `item` in step `step`.
    fn signal(&self, step: &str, item: Id) -> Option<ir::AmplitudeId> {
        let key = AmplitudeKey::Signal(step.to_owned(), item);
        self.ids.get(&key).copied()
    }
}

/// The values a displacement holds and the amplitude each follows: a degree of freedom with
/// a table holds 1 and follows the amplitude of its table, the others their value and the
/// amplitude of the boundary condition.
fn displacement_values(
    step: &Step,
    bc: &plx_model::BoundaryCondition,
    displacement: &plx_model::Displacement,
    amplitude: &Option<ir::AmplitudeId>,
    amplitudes: &AmplitudeIds,
) -> ([Option<f64>; 6], [Option<ir::AmplitudeId>; 6]) {
    let values = std::array::from_fn(|d| match displacement.table(d) {
        Some(_) => Some(1.0),
        None => displacement.values[d],
    });
    let amplitudes = std::array::from_fn(|d| match displacement.table(d) {
        Some(_) => amplitudes.table(&step.name, bc.id.any(), DOF_TABLE_LABELS[d]),
        None => *amplitude,
    });
    (values, amplitudes)
}

/// The components of a load's value and the amplitude each follows: a component with a
/// table is 1 and follows the amplitude of its table.
fn load_components<const N: usize>(
    step: &Step,
    load: &plx_model::Load,
    values: [f64; N],
    labels: [&'static str; N],
    amplitude: &Option<ir::AmplitudeId>,
    amplitudes: &AmplitudeIds,
) -> ([f64; N], [Option<ir::AmplitudeId>; N]) {
    let values = std::array::from_fn(|c| match load.table(c) {
        Some(_) => 1.0,
        None => values[c],
    });
    let amplitudes = std::array::from_fn(|c| match load.table(c) {
        Some(_) => amplitudes.table(&step.name, load.id.any(), labels[c]),
        None => *amplitude,
    });
    (values, amplitudes)
}

/// The amplitude a boundary condition or load refers to; `None` without one.
fn amplitude_parameter(
    amplitudes: &AmplitudeIds,
    item: &str,
    reference: &Option<Id<plx_model::Amplitude>>,
) -> Result<Option<ir::AmplitudeId>, WriteError> {
    let Some(reference) = reference else {
        return Ok(None);
    };
    let Some(&amplitude) = amplitudes.ids.get(&AmplitudeKey::User(*reference)) else {
        return Err(WriteError::UnknownAmplitude {
            item: item.to_owned(),
            amplitude: reference.to_string(),
        });
    };
    Ok(Some(amplitude))
}

/// The distribution an item scales its value by, ready to sample; `None` without one.
fn distribution(
    model: &FeModel,
    item: &str,
    reference: &Option<Id<plx_model::Distribution>>,
) -> Result<Option<Sampler>, WriteError> {
    let Some(id) = *reference else {
        return Ok(None);
    };
    let Some(distribution) = model.distribution(id) else {
        return Err(WriteError::UnknownDistribution {
            item: item.to_owned(),
            distribution: id.to_string(),
        });
    };
    (distribution.sampler(&model.coordinate_systems))
        .map(Some)
        .map_err(|reason| WriteError::InvalidDistribution {
            item: item.to_owned(),
            distribution: distribution.name.clone(),
            reason,
        })
}

/// The value at each node of `nodes`, scaled by the distribution at the node.
fn distributed_nodes(
    mesh: &FeMesh,
    nodes: &[NodeId],
    value: f64,
    sampler: &Sampler,
    units: UnitSystem,
) -> Vec<(NodeRef, f64)> {
    (nodes.iter())
        .filter_map(|&node| {
            let position = mesh.node(node)?;
            let scaled = sampler.scale(value, Quantity::Temperature, units, position);
            Some((NodeRef::Node(node), scaled))
        })
        .collect()
}

/// Center of an element face, the mean of its corners.
fn face_center(mesh: &FeMesh, element: ElementId, face: u8) -> Option<[f64; 3]> {
    let element = mesh.element(element)?;
    let topology = element.faces().get(usize::from(face).checked_sub(1)?)?;
    let mut center = [0.0; 3];
    for &local in topology.corners {
        let point = mesh.node(*element.nodes.get(local)?)?;
        for k in 0..3 {
            center[k] += point[k] / topology.corners.len() as f64;
        }
    }
    Some(center)
}

/// Initial temperatures and velocities: one entry
/// per set or node and non-zero component, 2D models without the third.
fn initial_conditions(
    sets: &mut Sets,
    model: &FeModel,
) -> Result<ir::InitialConditions, WriteError> {
    let components = if model.properties.space.is_2d() { 2 } else { 3 };
    let mut out = ir::InitialConditions::default();
    let velocities = |out: &mut ir::InitialConditions, target: NodeRef, v: [f64; 3]| {
        for (dof, value) in (1..).zip(v).take(components) {
            if value != 0.0 {
                out.velocities.push((target.clone(), dof, value));
            }
        }
    };
    for condition in model.initial_conditions.iter().filter(|c| c.active) {
        match condition.kind {
            InitialConditionKind::Temperature(t) => {
                match distribution(model, &condition.name, &condition.distribution)? {
                    Some(sampler) => {
                        let nodes = condition.region.nodes(sets.mesh);
                        if nodes.is_empty() {
                            return Err(empty(&condition.name, "nodes"));
                        }
                        let units = model.properties.units;
                        (out.temperatures)
                            .extend(distributed_nodes(sets.mesh, &nodes, t, &sampler, units));
                    }
                    None => {
                        let set = sets.node_set(&condition.name, &condition.region)?;
                        out.temperatures.push((sets.node_ref(&set), t));
                    }
                }
            }
            InitialConditionKind::Velocity(v) => {
                let set = sets.node_set(&condition.name, &condition.region)?;
                velocities(&mut out, sets.node_ref(&set), v);
            }
            InitialConditionKind::AngularVelocity { .. } => {
                let nodes = condition.region.nodes(sets.mesh);
                if nodes.is_empty() {
                    return Err(empty(&condition.name, "nodes"));
                }
                for id in nodes {
                    let Some(position) = sets.mesh.node(id) else {
                        continue;
                    };
                    let v = condition.kind.velocity_at(position).unwrap_or_default();
                    velocities(&mut out, NodeRef::Node(id), v);
                }
            }
        }
    }
    Ok(out)
}

/// The sections; infill sections add the materials of their elements to `materials`,
/// oriented solid sections their orientations to `orientations`.
/// The nodes on symmetry planes of the active steps, inside an infill part rather than on its
/// printed surface.
fn symmetry_nodes(mesh: &FeMesh, model: &FeModel) -> std::collections::HashSet<NodeId> {
    (model.steps.iter())
        .filter(|step| step.active)
        .flat_map(|step| &step.boundary_conditions)
        .filter(|bc| bc.active && matches!(bc.kind, BoundaryKind::Symmetry { .. }))
        .flat_map(|bc| bc.region.nodes(mesh))
        .collect()
}

fn sections(
    sets: &mut Sets,
    model: &FeModel,
    material_ids: &std::collections::HashMap<Id<plx_model::Material>, ir::MaterialId>,
    materials: &mut Vec<ir::material::Material>,
    orientations: &mut Vec<ir::section::Orientation>,
) -> Result<SectionsAndProfiles, WriteError> {
    use ir::section::{Section as IrSection, SectionKind as Kind};
    let mut out = Vec::new();
    let mut comments = Vec::new();
    let mut owners = Vec::new();
    let halved_beams = model.halved_beams(sets.mesh);
    for (index, section) in model.sections.iter().enumerate() {
        let Some(material) = section.material.and_then(|m| model.material(m)) else {
            return Err(WriteError::UnknownMaterial {
                item: section.name.clone(),
                material: model.name_or_missing(section.material),
            });
        };
        let base = material;
        let material = material_ids[&material.id];
        let axes = (model.section_axes(section)).map_err(|reason| WriteError::InvalidSection {
            item: section.name.clone(),
            reason,
        })?;
        match &section.kind {
            SectionKind::Solid => {
                let set = sets.element_set(&section.name, &section.region)?;
                // Plane stress and plane strain sections have a thickness.
                let thickness =
                    (model.properties.space.has_thickness()).then_some(section.thickness);
                // The axes of an anisotropic material.
                let orientation = axes.filter(|_| base.anisotropic.is_some()).map(|axes| {
                    orientations.push(ir::section::Orientation {
                        name: format!("ORIENTATION_{set}"),
                        a: axes[0],
                        b: axes[1],
                    });
                    ir::OrientationId::at(orientations.len() - 1)
                });
                out.push(IrSection {
                    elset: sets.element_id(&set),
                    kind: Kind::Solid {
                        material,
                        thickness,
                        orientation,
                    },
                });
            }
            SectionKind::Infill(infill) => {
                let failed = |reason: String| WriteError::InvalidSection {
                    item: section.name.clone(),
                    reason,
                };
                let elastic = (base.elastic)
                    .filter(|_| base.anisotropic.is_none() && base.hyperelastic.is_none())
                    .ok_or_else(|| {
                        failed(format!(
                            "the printed material {} needs isotropic linear elasticity",
                            base.name
                        ))
                    })?;
                let core_material = (infill.core_material)
                    .and_then(|m| model.material(m))
                    .filter(|_| infill.pattern == plx_model::InfillPattern::Custom);
                let core = match core_material {
                    Some(core) => Some(match (&core.anisotropic, core.small_strain_elastic()) {
                        (Some(a), _) => a.matrix().map_err(failed)?,
                        (None, Some(e)) => plx_model::Anisotropic::Orthotropic(
                            plx_model::EngineeringConstants::isotropic(e),
                        )
                        .matrix()
                        .map_err(failed)?,
                        (None, None) => {
                            return Err(failed(format!(
                                "the core material {} has no elasticity",
                                core.name
                            )));
                        }
                    }),
                    None => None,
                };
                let axes = axes.unwrap_or([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
                let (wall, core_matrix) =
                    (infill.matrices_in(elastic, core, &axes)).map_err(failed)?;
                let elements = section.region.elements(sets.mesh);
                if elements.is_empty() {
                    return Err(empty(&section.name, "elements"));
                }
                let inside = symmetry_nodes(sets.mesh, model);
                let levels = infill::wall_levels(sets.mesh, &elements, infill, axes[2], &inside);
                let core_density = match core_material {
                    Some(core) => core.density,
                    None => base.density.map(|rho| rho * infill.density),
                };
                let template = materials[material.index()].clone();
                for (level, ids) in levels {
                    let share = level as f64 / infill::LEVELS as f64;
                    let mut d = [[0.0; 6]; 6];
                    for i in 0..6 {
                        for j in 0..6 {
                            d[i][j] = share * wall[i][j] + (1.0 - share) * core_matrix[i][j];
                        }
                    }
                    let set = sets.free_name(
                        "Internal_Selection",
                        &format!("{}_Wall{level}", name(&section.name)),
                    );
                    sets.element_sets.push((set.clone(), Members::Ids(ids)));
                    let mut printed = template.clone();
                    printed.name = format!("Infill_{set}");
                    printed.elastic = None;
                    printed.anisotropic = Some(
                        plx_model::Anisotropic::General(Box::new(plx_model::anisotropic_upper(&d)))
                            .solver(),
                    );
                    printed.density = match (base.density, core_density) {
                        (Some(wall), Some(core)) => Some(ir::material::Density {
                            rows: constant(share * wall + (1.0 - share) * core),
                        }),
                        _ => None,
                    };
                    materials.push(printed);
                    out.push(IrSection {
                        elset: sets.element_id(&set),
                        kind: Kind::Solid {
                            material: ir::MaterialId::at(materials.len() - 1),
                            thickness: None,
                            orientation: None,
                        },
                    });
                }
            }
            SectionKind::Truss { area } => {
                let set = sets.element_set(&section.name, &section.region)?;
                out.push(IrSection {
                    elset: sets.element_id(&set),
                    kind: Kind::Solid {
                        material,
                        orientation: None,
                        thickness: Some(*area),
                    },
                });
            }
            &SectionKind::Shell { thickness, offset } => {
                let set = sets.element_set(&section.name, &section.region)?;
                out.push(IrSection {
                    elset: sets.element_id(&set),
                    kind: Kind::Shell {
                        material,
                        thickness,
                        offset,
                        nodal_thickness: false,
                    },
                });
            }
            &SectionKind::Membrane { thickness, offset } => {
                let set = sets.element_set(&section.name, &section.region)?;
                out.push(IrSection {
                    elset: sets.element_id(&set),
                    kind: Kind::Membrane {
                        material,
                        thickness,
                        offset,
                    },
                });
            }
            SectionKind::Beam(beam) => {
                let profile = profile_of(model, &section.name, beam)?;
                let ir_profile = beam_profile(&profile);
                // One section per normal: the solver takes one normal per section, and an
                // automatic orientation may differ between the beams of the region. Beams on
                // a symmetry plane get half the section, in sets of their own.
                for (normal, drawn, set, half) in
                    beam_groups(sets, section, &profile, &halved_beams)?
                {
                    // A halved section keeps the comment of its whole profile: the solver
                    // reads the values below it, the results draw the beam whole, as the
                    // model shows it.
                    if !profile.is_native() {
                        comments.push((ir_name(&set), profile_comment(&profile, drawn)));
                    }
                    out.push(IrSection {
                        elset: sets.element_id(&set),
                        kind: Kind::Beam {
                            material,
                            profile: if half {
                                halved_profile(&ir_profile)
                            } else {
                                ir_profile.clone()
                            },
                            normal,
                            offset1: beam.offset[0],
                            offset2: beam.offset[1],
                        },
                    });
                }
            }
        }
        owners.resize(out.len(), index);
    }
    Ok((out, comments, owners))
}

/// The sections, the profile comments of the general beam sections (see
/// [`Lowered::profile_comments`]) and the section of the model each section came from.
type SectionsAndProfiles = (Vec<ir::section::Section>, Vec<(String, String)>, Vec<usize>);

/// The comment before the `*Beam section` of a general profile: the profile, in RON, and
/// the normal it is drawn with, which [`read_inp`](super::read_inp) takes back.
fn profile_comment(profile: &BeamProfile, normal: [f64; 3]) -> String {
    let ron = ron::to_string(profile).unwrap_or_default();
    let normal = normal.map(number).join(", ");
    format!("{}{ron}; Normal: {normal}", super::PROFILE_COMMENT)
}

/// The profile of a beam section as the solver takes it: a general profile by its values
/// about its principal axes (see [`BeamProfile::data_line`]).
fn beam_profile(profile: &BeamProfile) -> ir::section::BeamProfile {
    use ir::section::BeamProfile as P;
    let v = profile.data_line();
    let at = |i: usize| v.get(i).copied().unwrap_or(0.0);
    match profile.keyword() {
        "RECT" => P::Rect { a: at(0), b: at(1) },
        "CIRC" => P::Circ { a: at(0), b: at(1) },
        "PIPE" => P::Pipe {
            radius: at(0),
            thickness: at(1),
        },
        "BOX" => P::Box {
            a: at(0),
            b: at(1),
            t1: at(2),
            t2: at(3),
            t3: at(4),
            t4: at(5),
        },
        _ => P::General {
            area: at(0),
            i11: at(1),
            i12: at(2),
            i22: at(3),
            shear_factor: at(4),
            torsion: Some(at(5)),
            contact_radius: profile.contact_radius(),
        },
    }
}

/// Half a general profile: the beam on a symmetry plane, of which the model holds the half
/// on one side. Area, second moments and torsion constant (and so the mass) halve; the
/// contact radius stays the rope's.
fn halved_profile(profile: &ir::section::BeamProfile) -> ir::section::BeamProfile {
    use ir::section::BeamProfile as P;
    match profile.clone() {
        P::General {
            area,
            i11,
            i12,
            i22,
            shear_factor,
            torsion,
            contact_radius,
        } => P::General {
            area: 0.5 * area,
            i11: 0.5 * i11,
            i12: 0.5 * i12,
            i22: 0.5 * i22,
            shear_factor,
            torsion: torsion.map(|j| 0.5 * j),
            contact_radius,
        },
        other => other,
    }
}

/// The section of generated spring and gap elements.
fn generated_section(
    sets: &mut Sets,
    section: &constraints::GeneratedSection,
) -> ir::section::Section {
    use ir::section::{Gap, SectionKind as Kind, Spring, SpringStiffness};
    match section {
        constraints::GeneratedSection::Spring {
            elset,
            dofs,
            stiffness,
            stabilize,
        } => ir::section::Section {
            elset: sets.element_id(elset),
            kind: Kind::Spring(Spring {
                dof1: dofs[0],
                dof2: *dofs.get(1).unwrap_or(&dofs[0]),
                stiffness: SpringStiffness::Linear(constant(*stiffness)),
                orientation: None,
                stabilize: *stabilize,
            }),
        },
        constraints::GeneratedSection::Gap {
            elset,
            clearance,
            direction,
            properties,
            ..
        } => ir::section::Section {
            elset: sets.element_id(elset),
            kind: Kind::Gap(Gap {
                clearance: *clearance,
                direction: *direction,
                stiffness: properties.map(|(k, _)| k).filter(|&k| k > 0.0),
                friction: None,
            }),
        },
    }
}

/// The profile of a beam section's cross-section.
fn profile_of(
    model: &FeModel,
    section: &str,
    beam: &BeamSection,
) -> Result<BeamProfile, WriteError> {
    let cross_section = beam.cross_section.and_then(|c| model.cross_section(c));
    (cross_section.map(|c| c.profile.clone())).ok_or_else(|| WriteError::InvalidSection {
        item: section.to_owned(),
        reason: match beam.cross_section {
            None => "The beam section has no cross-section".into(),
            Some(_) => "The cross-section does not exist".into(),
        },
    })
}

/// The normal a group of beams is written with, the one its profile is drawn with, its
/// element set and whether the beams lie on a symmetry plane, which halves their section.
type BeamGroup = ([f64; 3], [f64; 3], String, bool);

/// The beams of a section grouped by the normal they are written with, each group as an
/// element set with that normal and the one the profile is drawn with. A region whose beams
/// all share one normal keeps its own set. A general profile is written about its principal
/// axes: its normal is turned about the beam by [`BeamProfile::principal_turn`].
fn beam_groups(
    sets: &mut Sets,
    section: &Section,
    profile: &BeamProfile,
    halved: &BTreeSet<ElementId>,
) -> Result<Vec<BeamGroup>, WriteError> {
    let SectionKind::Beam(beam) = &section.kind else {
        return Ok(Vec::new());
    };
    let mesh = sets.mesh;
    let turn = profile.principal_turn();
    // (normal, drawn normal, halved, elements)
    type Group = ([f64; 3], [f64; 3], bool, Vec<ElementId>);
    let mut groups: Vec<Group> = Vec::new();
    for element in (section.region.elements(mesh).into_iter()).filter_map(|id| mesh.element(id)) {
        if element.shape.family() != ElementFamily::Line {
            continue;
        }
        let tangent = line_tangent(mesh, element);
        let drawn =
            (beam.orientation.normal_for(tangent)).ok_or_else(|| WriteError::InvalidSection {
                item: section.name.clone(),
                reason: format!(
                    "The normal is parallel to the axis of element {}",
                    element.id
                ),
            })?;
        let normal = if turn == 0.0 {
            drawn
        } else {
            // The 1-direction perpendicular to the beam, turned towards the 2-direction,
            // which is the beam axis crossed with it.
            let n1 = glam::DVec3::from(drawn);
            let n1 = (n1 - tangent * n1.dot(tangent)).normalize_or_zero();
            let (sin, cos) = turn.sin_cos();
            let turned = n1 * cos + tangent.cross(n1) * sin;
            // Rounded so that beams along the same line share a group.
            turned.to_array().map(|c| (c * 1e12).round() / 1e12)
        };
        let half = halved.contains(&element.id);
        match groups
            .iter_mut()
            .find(|(n, d, h, _)| *n == normal && *d == drawn && *h == half)
        {
            Some((_, _, _, ids)) => ids.push(element.id),
            None => groups.push((normal, drawn, half, vec![element.id])),
        }
    }
    if groups.is_empty() {
        return Err(empty(&section.name, "line elements"));
    }
    // CalculiX 2.21 reads the element set of a pipe, box or general section 20 characters
    // wide and fails on longer names, so those get short sets of their own.
    let short = profile.needs_reduced_integration() || !profile.is_native();
    if groups.len() == 1 && !short && !groups[0].2 {
        let set = sets.element_set(&section.name, &section.region)?;
        return Ok(vec![(groups[0].0, groups[0].1, set, false)]);
    }
    let mut named = Vec::new();
    for (normal, drawn, half, ids) in groups {
        let set = if short {
            sets.unique_numbered("Beam")
        } else {
            sets.free_name("Internal_Selection", &name(&section.name))
        };
        sets.element_sets.push((set.clone(), Members::Ids(ids)));
        named.push((normal, drawn, set, half));
    }
    Ok(named)
}

/// Rigid bodies, remote points and ties; ties take the slave surface
/// first. Springs and supports become elements, see [`constraints::springs`].
fn constraints(
    sets: &mut Sets,
    model: &FeModel,
    generated: &mut constraints::Generated,
    lowering: &mut Option<usize>,
) -> Result<(Vec<ConstraintItem>, Vec<Option<ModelItem>>), WriteError> {
    let mut items = Vec::new();
    // The item each of `items` came from, where the solver can name it (a rigid body).
    let mut owners: Vec<Option<ModelItem>> = Vec::new();
    let mut next_node = (sets.mesh.node_ids().iter().copied())
        .chain(generated.nodes.iter().map(|(id, _)| *id))
        .max()
        .map_or(1, |n| n + 1);
    for (index, constraint) in model.constraints.iter().enumerate() {
        if !constraint.active() {
            continue;
        }
        match constraint {
            Constraint::Spring(_) | Constraint::CompressionOnly(_) | Constraint::Joint(_) => {}
            Constraint::RigidBody(_) => {
                let coupling = constraint.coupling().expect("a rigid body couples");
                couple_point(
                    index,
                    coupling,
                    sets,
                    model,
                    generated,
                    &mut next_node,
                    (&mut items, &mut owners),
                )?;
            }
            // After the rigid bodies, whose points they may sit on.
            Constraint::PointMass(_) => {}
        }
    }
    // A point mass sits on the nodes of its reference point: those of the rigid body that
    // couples the point, or of the coupling of its own selection.
    for (index, constraint) in model.constraints.iter().enumerate() {
        let Constraint::PointMass(mass) = constraint else {
            continue;
        };
        if !mass.active {
            continue;
        }
        if let Some(problem) = model.point_mass_problem(mass) {
            return Err(WriteError::InvalidConstraint {
                item: mass.name.clone(),
                reason: problem,
            });
        }
        let nodes = match constraint.coupling() {
            Some(coupling) => couple_point(
                index,
                coupling,
                sets,
                model,
                generated,
                &mut next_node,
                (&mut items, &mut owners),
            )?,
            None => sets.reference_nodes(&mass.name, mass.reference_point)?,
        };
        let inertia = match &mass.inertia {
            Some(inertia) => {
                let tensor = if model.properties.space.is_2d() {
                    // About the axis normal to the plane, which no system turns.
                    [0.0, 0.0, inertia.tensor[2], 0.0, 0.0, 0.0]
                } else {
                    let axes = (model.coordinate_system_or_global(inertia.system))
                        .and_then(|frame| frame.axes().ok())
                        .ok_or_else(|| WriteError::InvalidConstraint {
                            item: mass.name.clone(),
                            reason: "the coordinate system of the rotary inertia has no axes"
                                .into(),
                        })?;
                    inertia.global(axes)
                };
                Some((nodes.rotation, tensor))
            }
            None => None,
        };
        // The elements that carry the mass, for the gravity and centrifugal loads on them.
        let elements = model.point_mass_carriers(mass, sets.mesh);
        let elset = (!elements.is_empty()).then(|| {
            let set = sets.free_name("Internal_Selection", &name(&mass.name));
            sets.element_sets
                .push((set.clone(), Members::Ids(elements)));
            sets.element_id(&set)
        });
        let label = name(&nodes.name);
        let named = match sets.point_mass_names.get(&mass.reference_point) {
            Some(named) => named.clone(),
            None => {
                let named = sets.free_name("Internal_Mass", &label);
                (sets.point_mass_names).insert(mass.reference_point, named.clone());
                named
            }
        };
        owners.resize(items.len(), None);
        items.push(ConstraintItem::PointMass(ir::PointMass {
            node: nodes.reference,
            mass: mass.mass,
            inertia,
            elset,
            name: Some(sets.element_id(&named)),
        }));
        owners.push(Some(ModelItem::Constraint(index)));
    }
    // A displacement that needs a point drives its region from it, by default at the centre
    // of the region; the point exists only in the analysis. Its reference node takes the
    // translations, its rotation node the rotations as displacements, both in the directions
    // of the displacement's coordinate system. Nodes that boundary conditions hold directly
    // in a step the point acts in stay out of the coupling; supports of other steps do not
    // count, the solver takes the equations around them there.
    for (s, step) in model.steps.iter().enumerate().filter(|(_, s)| s.active) {
        for (i, bc) in step.boundary_conditions.iter().enumerate() {
            let BoundaryKind::Displacement(remote) = &bc.kind else {
                continue;
            };
            if !bc.active || !step.kind.supports_boundary(&bc.kind) {
                continue;
            }
            let Some(Formulation::Point(behavior)) = model.formulation(sets.mesh, s, i) else {
                continue;
            };
            if let Some(&(_, _, nodes)) = (sets.remote_points.iter())
                .find(|(r, p, _)| *r == bc.region && p.same_point(remote))
            {
                (sets.remote_names.entry(bc.id.any())).or_insert((nodes, behavior));
                continue;
            }
            // An error is about the boundary condition of this step, whatever the others of
            // the same name.
            *lowering = Some(s);
            let steps = model.point_steps(sets.mesh, &bc.region, remote);
            let excluded = model.directly_held_nodes_in(sets.mesh, |s| steps.contains(&s));
            let held = model.held_translations_in(sets.mesh, |s| steps.contains(&s));
            let point =
                (remote.resolve(model, sets.mesh, &bc.region, &excluded)).map_err(|reason| {
                    WriteError::InvalidBoundary {
                        item: bc.name.clone(),
                        reason,
                    }
                })?;
            let nodes = remote_point(
                RemoteSource {
                    item: &bc.name,
                    region: &bc.region,
                    spec: remote,
                    behavior,
                    owner: ModelItem::BoundaryCondition(s, i),
                    free_point: false,
                },
                point,
                held,
                &mut next_node,
                (&mut items, &mut owners),
                sets,
                model,
                generated,
            )?;
            (sets.remote_names.entry(bc.id.any())).or_insert((nodes, behavior));
        }
    }
    // A remote force drives its region from a point of its own the same way, or from the
    // point of a remote displacement of the same region and point. A force on a reference
    // point acts on its nodes and needs none.
    for (s, i, region, force) in model.remote_forces() {
        if region.reference_point().is_some() {
            continue;
        }
        let spec = force.point();
        let load = &model.steps[s].loads[i];
        // Node history outputs find the point by the load's id.
        if let Some(&(_, _, nodes)) =
            (sets.remote_points.iter()).find(|(r, p, _)| r == region && p.same_point(&spec))
        {
            (sets.remote_names.entry(load.id.any())).or_insert((nodes, force.behavior));
            continue;
        }
        *lowering = Some(s);
        let steps = model.remote_force_steps(region, force);
        let excluded = model.directly_held_nodes_in(sets.mesh, |s| steps.contains(&s));
        let held = model.held_translations_in(sets.mesh, |s| steps.contains(&s));
        let point = (spec.resolve(model, sets.mesh, region, &excluded)).map_err(|reason| {
            WriteError::InvalidLoad {
                item: load.name.clone(),
                reason,
            }
        })?;
        let nodes = remote_point(
            RemoteSource {
                item: &load.name,
                region,
                spec: &spec,
                behavior: force.behavior,
                owner: ModelItem::Load(s, i),
                free_point: true,
            },
            point,
            held,
            &mut next_node,
            (&mut items, &mut owners),
            sets,
            model,
            generated,
        )?;
        (sets.remote_names.entry(load.id.any())).or_insert((nodes, force.behavior));
    }
    *lowering = None;
    // Joints, after the points of boundary conditions and remote forces they may connect.
    for (index, constraint) in model.constraints.iter().enumerate() {
        let Constraint::Joint(joint) = constraint else {
            continue;
        };
        if !joint.active {
            continue;
        }
        let item = joint_item(
            index,
            joint,
            &mut next_node,
            (&mut items, &mut owners),
            sets,
            model,
            generated,
        )?;
        owners.resize(items.len(), None);
        items.push(item);
        owners.push(Some(ModelItem::Constraint(index)));
    }
    for (index, t) in model.ties.iter().enumerate().filter(|(_, t)| t.active) {
        owners.resize(items.len(), None);
        items.push(tie(sets, t)?);
        owners.push(Some(ModelItem::Tie(index)));
    }
    // A spring ending on a reference point no rigid body drives ends on ground there: a
    // node of its own at the point, which a displacement on the point moves.
    for id in model.grounded_points() {
        let Some(point) = model.reference_point(id) else {
            continue;
        };
        let (reference, rotation) = (next_node, next_node + 1);
        next_node += 2;
        generated.nodes.push((reference, point.position));
        generated.nodes.push((rotation, point.position));
        let nodes = ReferenceNodes {
            name: point.name.clone(),
            reference,
            rotation,
            set: None,
            behavior: RemoteBehavior::Rigid,
        };
        sets.reference_nodes.insert(id, nodes);
        sets.grounded.push(id);
    }
    owners.resize(items.len(), None);
    Ok((items, owners))
}

/// A joint between two ends. A selection end follows a point of its own at the joint, a
/// point end is the point's reference and rotation node, a ground end has none.
fn joint_item(
    index: usize,
    joint: &plx_model::Joint,
    next_node: &mut NodeId,
    (items, owners): (&mut Vec<ConstraintItem>, &mut Vec<Option<ModelItem>>),
    sets: &mut Sets,
    model: &FeModel,
    generated: &mut constraints::Generated,
) -> Result<ConstraintItem, WriteError> {
    let invalid = |reason: String| WriteError::InvalidConstraint {
        item: joint.name.clone(),
        reason,
    };
    let (origin, axes) = model.joint_frame(sets.mesh, joint).map_err(invalid)?;
    let mut ends = [None; 2];
    for (slot, end) in ends.iter_mut().zip(&joint.ends) {
        let Some(region) = end.region() else {
            continue;
        };
        if matches!(region, Region::ReferencePoint(_) | Region::RemotePoint(_)) {
            *slot = Some(point_nodes(sets, &joint.name, region)?.0);
            continue;
        }
        // The selection follows a point of its own at the joint.
        let spec = plx_model::Displacement {
            values: [None; 6],
            behavior: match joint.behavior {
                RemoteBehavior::Rigid => plx_model::Behavior::Rigid,
                RemoteBehavior::Deformable => plx_model::Behavior::Deformable,
            },
            system: None,
            location: Some(origin),
            pinball: None,
            tables: Default::default(),
        };
        let excluded = model.directly_held_nodes_in(sets.mesh, |_| true);
        let held = model.held_translations(sets.mesh);
        let point = (spec.resolve(model, sets.mesh, region, &excluded)).map_err(invalid)?;
        let nodes = remote_point(
            RemoteSource {
                item: &joint.name,
                region,
                spec: &spec,
                behavior: joint.behavior,
                owner: ModelItem::Constraint(index),
                free_point: false,
            },
            point,
            held,
            next_node,
            (items, owners),
            sets,
            model,
            generated,
        )?;
        *slot = Some(nodes);
    }
    let free = joint.free_dofs();
    let mut fixed = [false; 6];
    for &d in model.properties.space.remote_dofs() {
        fixed[d] = !free[d];
    }
    Ok(ConstraintItem::Joint(ir::constraints::Joint {
        name: ir_name(&name(&joint.name)),
        ends,
        origin,
        axes,
        fixed,
    }))
}

/// What a point of a remote displacement or remote force comes from: the item, its region,
/// the point as a displacement and how the region follows it.
struct RemoteSource<'r> {
    item: &'r str,
    region: &'r Region,
    spec: &'r plx_model::Displacement,
    behavior: RemoteBehavior,
    owner: ModelItem,
    /// Nothing holds the point or binds its nodes otherwise (a force on it, or a reference
    /// point without boundary conditions or joints): a deformable point may then depend on
    /// the region where the region's nodes are taken, e.g. by a rigid body of another step.
    free_point: bool,
}

/// Adds the reference and rotation node of a resolved point and couples the nodes of the
/// region to it, rigidly or as averaging equations. `held` are the translations boundary
/// conditions hold in the steps the point acts in, which the coupling leaves out. The
/// point's nodes get the local directions of the point, if it has some.
#[allow(clippy::too_many_arguments)]
fn remote_point(
    source: RemoteSource,
    point: plx_model::RemotePoint,
    held: BTreeMap<NodeId, [bool; 3]>,
    next_node: &mut NodeId,
    (items, owners): (&mut Vec<ConstraintItem>, &mut Vec<Option<ModelItem>>),
    sets: &mut Sets,
    model: &FeModel,
    generated: &mut constraints::Generated,
) -> Result<[NodeId; 2], WriteError> {
    let RemoteSource {
        item,
        region,
        spec,
        behavior,
        owner,
        free_point,
    } = source;
    let (reference, rotation) = (*next_node, *next_node + 1);
    *next_node += 2;
    generated.nodes.push((reference, point.position));
    generated.nodes.push((rotation, point.position));
    sets.remote_points
        .push((region.clone(), spec.clone(), [reference, rotation]));
    if let Some([x, y, _]) = point.axes {
        let set = sets.free_name("Internal_Selection", &format!("{}_Point", name(item)));
        sets.node_sets
            .push((set.clone(), vec![reference, rotation]));
        sets.transforms.push((set, Transform::Rectangular(x, y)));
    }
    let all_steps = sets.dofs.replace_held(held);
    match behavior {
        RemoteBehavior::Rigid => {
            let whole =
                spec.pinball.is_none() && point.nodes.len() == region.nodes(sets.mesh).len();
            let region = if whole {
                region.clone()
            } else {
                Region::Nodes(point.nodes.clone())
            };
            owners.resize(items.len(), None);
            let written = rigid_body(items, sets, item, &region, [reference, rotation]);
            sets.dofs.replace_held(all_steps);
            written?;
            owners.resize(items.len(), Some(owner));
        }
        RemoteBehavior::Deformable => {
            let space = model.properties.space;
            for equation_terms in point.equations(sets.mesh, space) {
                let mut terms = remote_terms(&equation_terms, point.axes, [reference, rotation]);
                let mesh = sets.mesh;
                // The region's nodes depend on the point; a free point depends on the
                // region where they are all taken. Either way a force on the point reaches
                // the region.
                if (sets.dofs).claim_equation(item, &mut terms, |n| {
                    if n == reference || n == rotation {
                        free_point
                    } else {
                        mesh.node(n).is_some()
                    }
                }) {
                    items.push(ConstraintItem::Equation(equation(&terms)));
                }
            }
            sets.dofs.replace_held(all_steps);
        }
    }
    Ok([reference, rotation])
}

/// A tie of a slave surface glued to its master, with its cyclic symmetry model if it is
/// one.
fn tie(sets: &mut Sets, tie: &Tie) -> Result<ConstraintItem, WriteError> {
    // A bolt split into two parts is tied at its pre-tension cut. The faces on the cut side
    // belong to the copies of the cut nodes then, and a tie to them would bypass the preload:
    // the original nodes, which the cut's equations hold apart from the copies, become the
    // slave nodes instead, tied to the faces of the other part.
    // The nodes of a tie side, also of its faces: a side picked on the CAD geometry lists the
    // original nodes, its element faces the copies.
    let side_nodes = |region: &Region| -> BTreeSet<NodeId> {
        let faces = region.faces(sets.mesh);
        (region.nodes(sets.mesh).into_iter())
            .chain(
                faces
                    .iter()
                    .flat_map(|&(e, f)| constraints::face_node_ids(sets.mesh, e, f)),
            )
            .collect()
    };
    let at_cut = |nodes: &BTreeSet<NodeId>| {
        (nodes.iter()).any(|n| {
            sets.cut_originals.contains_key(n) || sets.cut_originals.values().any(|o| o == n)
        })
    };
    let (master_nodes, slave_nodes) = (side_nodes(&tie.master), side_nodes(&tie.slave));
    let (master_at_cut, slave_at_cut) = (at_cut(&master_nodes), at_cut(&slave_nodes));
    let (slave_region, master_region, slave_nodes) = if master_at_cut && !slave_at_cut {
        (&tie.master, &tie.slave, master_nodes)
    } else {
        (&tie.slave, &tie.master, slave_nodes)
    };
    let master = sets.surface(&tie.name, "Master", master_region)?;
    // CalculiX (2.21 to 2.23) ties the edge faces of membranes to nothing: their expanded
    // nodes have no active degrees of freedom there, and the model has no stiffness. Their
    // nodes tie correctly, and a membrane has no rotations to lose.
    let slave = if master_at_cut || slave_at_cut {
        let originals: BTreeSet<NodeId> = (slave_nodes.iter())
            .map(|n| sets.cut_originals.get(n).copied().unwrap_or(*n))
            .collect();
        let originals = originals.into_iter().collect();
        sets.node_surface(&tie.name, "Slave", &Region::Nodes(originals))?
    } else if membrane_edges(sets.mesh, slave_region) || matches!(slave_region, Region::Nodes(_)) {
        // Picked nodes: of the bodies of a compound part those that lie on the master.
        sets.node_surface(&tie.name, "Slave", slave_region)?
    } else {
        sets.surface(&tie.name, "Slave", slave_region)?
    };
    let tie_name = ir_name(&name(&tie.name));
    let cyclic = match &tie.cyclic {
        Some(cyclic) => {
            let sectors = (tie.cyclic_sectors(sets.mesh))
                .unwrap_or_else(|| Err("not a cyclic symmetry".into()))
                .map_err(|reason| WriteError::InvalidCyclicSymmetry {
                    item: tie.name.clone(),
                    reason,
                })?;
            // The axis by a point and a second point on it.
            let [x, y, z] = cyclic.axis_point;
            let [dx, dy, dz] = cyclic.axis_direction;
            Some(ir::constraints::CyclicSymmetryModel {
                sectors,
                ngraph: if cyclic.display == SectorDisplay::All {
                    sectors
                } else {
                    1
                },
                // Set when the tie takes its place.
                tie: ir::TieId(u32::MAX),
                axis: [[x, y, z], [x + dx, y + dy, z + dz]],
                elset: None,
                check: true,
            })
        }
        None => None,
    };
    Ok(ConstraintItem::Tie(
        ir::constraints::Tie {
            name: tie_name,
            slave: sets.surface_id(&slave),
            master: sets.surface_id(&master),
            position_tolerance: tie.position_tolerance,
            adjust: tie.adjust,
            cyclic_symmetry: tie.cyclic.is_some(),
            multistage: false,
        },
        cyclic,
    ))
}

/// Nodes on the slave side of an active cyclic symmetry and not on its master side (the
/// axis). The solver computes them from the master side and stops when a boundary condition
/// prescribes them too ("dependent side of a MPC and a SPC"), so boundary conditions leave
/// them out; they follow the prescribed master nodes anyway.
fn cyclic_slave_nodes(mesh: &FeMesh, model: &FeModel) -> BTreeSet<NodeId> {
    let mut nodes = BTreeSet::new();
    for tie in model.ties.iter().filter(|t| t.active && t.cyclic.is_some()) {
        let master: BTreeSet<NodeId> = tie.master.nodes(mesh).into_iter().collect();
        nodes.extend(
            tie.slave
                .nodes(mesh)
                .into_iter()
                .filter(|n| !master.contains(n)),
        );
    }
    nodes
}

/// One cut per pre-tension load, as the equations that tie its two sides to a new node
/// carrying the preload (see [`pre_tension`]): loads of the same name in several steps
/// share the cut, as the bolt is one.
fn pre_tension_sections(
    sets: &mut Sets,
    model: &FeModel,
    cuts: &BTreeMap<String, pre_tension::Cut>,
    generated: &mut constraints::Generated,
) -> Vec<ir::constraints::Equation> {
    let mut equations = Vec::new();
    let mut next_node = (sets.mesh.node_ids().iter().copied())
        .chain(generated.nodes.iter().map(|(id, _)| *id))
        .max()
        .map_or(1, |n| n + 1);
    for step in model
        .steps
        .iter()
        .filter(|s| s.active && s.kind.supports_loads())
    {
        for load in step.loads.iter().filter(|l| l.active) {
            let Some(cut) = cuts.get(&load.name) else {
                continue;
            };
            if sets.pre_tension_nodes.contains_key(&load.name) {
                continue;
            }
            // The node sits at the centre of the cut; the solver does not use its position.
            let mut centre = [0.0; 3];
            for coords in cut.pairs.iter().filter_map(|&(n, _)| sets.mesh.node(n)) {
                for (c, x) in centre.iter_mut().zip(coords) {
                    *c += x / cut.pairs.len() as f64;
                }
            }
            let node = next_node;
            next_node += 1;
            generated.nodes.push((node, centre));
            sets.pre_tension_nodes.insert(load.name.clone(), node);
            let set = sets.free_name("Internal_Selection", &name(&load.name));
            sets.node_sets.push((set.clone(), vec![node]));
            sets.pre_tension_sets.insert(load.name.clone(), set);
            let mesh = sets.mesh;
            for mut terms in pre_tension::equations(cut, node) {
                if (sets.dofs).claim_equation(&load.name, &mut terms, |n| {
                    n != node && mesh.node(n).is_some()
                }) {
                    equations.push(equation(&terms));
                }
            }
        }
    }
    equations
}

/// The equations of the node ties, which follow the contact pairs.
fn node_ties(
    sets: &mut Sets,
    model: &FeModel,
    lines: &LineElements,
) -> Result<Vec<ir::constraints::Equation>, WriteError> {
    let mut equations = Vec::new();
    for tie in model.node_ties.iter().filter(|t| t.active) {
        node_tie(&mut equations, sets, tie, lines)?;
    }
    Ok(equations)
}

/// Whether a node tie is a hinge between beams, written as equations of the translations.
/// Every other tie is written as one shared node (see [`node_merges`]): CalculiX 2.21 does
/// not couple the rotations of beam nodes through equations, and nodes without rotations
/// (trusses, solids) are joined completely by their translations anyway.
fn hinge_between_beams(tie: &NodeTie, nodes: &[NodeId], lines: &LineElements) -> bool {
    !tie.rotations && nodes.iter().all(|n| lines.beam_nodes.contains(n))
}

/// The nodes every active node tie merges into its first node, as nodes to replace by the
/// node that stays.
fn node_merges(mesh: &FeMesh, model: &FeModel, lines: &LineElements) -> BTreeMap<NodeId, NodeId> {
    let mut replaced = BTreeMap::new();
    for tie in model.node_ties.iter().filter(|t| t.active) {
        let nodes = tie.region.nodes(mesh);
        if hinge_between_beams(tie, &nodes, lines) {
            continue;
        }
        if let Some((&first, rest)) = nodes.split_first() {
            let first = replaced.get(&first).copied().unwrap_or(first);
            for &node in rest {
                if node != first {
                    replaced.insert(node, first);
                }
            }
        }
    }
    replaced
}

/// A node tie: a hinge between beams as equations of the translations, every other tie as
/// one shared node, which [`node_merges`] put into the mesh already.
fn node_tie(
    equations: &mut Vec<ir::constraints::Equation>,
    sets: &mut Sets,
    tie: &NodeTie,
    lines: &LineElements,
) -> Result<(), WriteError> {
    let nodes = tie.region.nodes(sets.mesh);
    let Some((&first, rest)) = nodes.split_first() else {
        return Err(WriteError::EmptyRegion {
            item: tie.name.clone(),
            what: "nodes",
        });
    };
    if !hinge_between_beams(tie, &nodes, lines) {
        return Ok(());
    }
    for &node in rest {
        for dof in 1..=3 {
            let mut terms = [(node, dof, 1.0), (first, dof, -1.0)];
            if sets.dofs.claim_equation(&tie.name, &mut terms, |_| true) {
                equations.push(equation(&terms));
            }
        }
    }
    Ok(())
}

/// The values the writer puts in where a surface interaction leaves them to CalculiX: its
/// defaults for the contact stiffness are multiples of Young's modulus, not stiffnesses per
/// length, and so about 1000 times too soft in SI units.
struct InteractionDefaults {
    /// Hard behavior of penalty contact pairs as a linear one: normal stiffness K and the
    /// tension at large clearances, and how K came about.
    hard_as_linear: Option<(f64, f64, String)>,
    /// Hard behavior used by mortar contact pairs only: true hard contact, which mortar
    /// contact gives without a surface behavior.
    true_hard: bool,
    /// Stick slope of a friction without one.
    stick_slope: Option<f64>,
}

impl InteractionDefaults {
    /// For the active contact pairs that use the interaction. Pairs of different stiffness
    /// share the softest, which converges for all of them.
    fn of(
        interaction: &SurfaceInteraction,
        mesh: &FeMesh,
        model: &FeModel,
        young: &plx_model::ElementYoung,
    ) -> Self {
        let pairs: Vec<&ContactPair> = (model.contact_pairs.iter())
            .filter(|p| p.active && p.interaction == Some(interaction.id))
            .collect();
        let softest = |penalty_only: bool| {
            (pairs.iter())
                .filter(|p| {
                    !penalty_only
                        || matches!(
                            p.method,
                            ContactMethod::NodeToSurface | ContactMethod::SurfaceToSurface
                        )
                })
                .filter_map(|p| ContactScale::with(young, mesh, &p.master, &p.slave))
                .min_by(|a, b| a.normal_stiffness().total_cmp(&b.normal_stiffness()))
        };
        let hard = (interaction.properties.iter()).any(|p| {
            matches!(
                p,
                InteractionProperty::SurfaceBehavior(SurfaceBehavior::Hard)
            )
        });
        let true_hard =
            hard && !pairs.is_empty() && (pairs.iter()).all(|p| p.method == ContactMethod::Mortar);
        let hard_as_linear = (softest(true)).filter(|_| hard).map(|scale| {
            let note = format!(
                "** Hard contact as linear, K = {} E/h, E = {}, h = {}",
                number(plx_model::NORMAL_STIFFNESS_FACTOR),
                number(scale.young),
                number(scale.edge_length)
            );
            // CalculiX's default tension, E / 70000: a stress, so free of the units.
            (scale.normal_stiffness(), scale.young / 70_000.0, note)
        });
        let normal = (interaction.own_normal_stiffness())
            .or_else(|| softest(false).map(ContactScale::normal_stiffness));
        // Without contact pairs to scale with, CalculiX's default: half the first elastic
        // constant of the first material. CalculiX 2.23 no longer fills it in itself, and a
        // friction without one sticks with no stiffness.
        let stick_slope = match normal {
            Some(k) => Some(plx_model::default_stick_slope(k, interaction.is_tied())),
            None => (model.materials.first())
                .and_then(|m| m.small_strain_elastic())
                .map(|e| e.young / 2.0),
        };
        Self {
            hard_as_linear,
            true_hard,
            stick_slope,
        }
    }
}

/// The handle of a surface interaction of `model`: the analysis has the model's
/// interactions in their order.
fn interaction_id(model: &FeModel, interaction: &SurfaceInteraction) -> ir::InteractionId {
    (model.surface_interactions.iter())
        .position(|i| i.id == interaction.id)
        .map_or(ir::InteractionId(u32::MAX), ir::InteractionId::at)
}

/// A surface interaction, with the defaults filled
/// in.
fn interaction(
    interaction: &SurfaceInteraction,
    defaults: &InteractionDefaults,
) -> ir::contact::SurfaceInteraction {
    use ir::contact as c;
    let mut out = c::SurfaceInteraction {
        name: interaction.name.clone(),
        behavior: None,
        friction: None,
        gap_conductance: None,
    };
    for property in &interaction.properties {
        match property {
            InteractionProperty::SurfaceBehavior(SurfaceBehavior::Hard) if defaults.true_hard => {}
            InteractionProperty::SurfaceBehavior(SurfaceBehavior::Hard)
                if defaults.hard_as_linear.is_some() =>
            {
                let (k, sigma_inf, _) = defaults.hard_as_linear.as_ref().unwrap();
                out.behavior = Some(c::SurfaceBehavior::Linear {
                    k: *k,
                    sigma_inf: Some(*sigma_inf),
                    c0: None,
                });
            }
            InteractionProperty::SurfaceBehavior(behavior) => {
                out.behavior = Some(match behavior {
                    SurfaceBehavior::Hard => c::SurfaceBehavior::Hard,
                    &SurfaceBehavior::Linear { k, sigma_inf, c0 } => c::SurfaceBehavior::Linear {
                        k,
                        sigma_inf: Some(sigma_inf),
                        c0,
                    },
                    &SurfaceBehavior::Exponential { c0, p0 } => {
                        c::SurfaceBehavior::Exponential { c0, p0 }
                    }
                    SurfaceBehavior::Tabular(rows) => {
                        c::SurfaceBehavior::Tabular(rows.iter().map(|&[p, u]| (p, u)).collect())
                    }
                    &SurfaceBehavior::Tied { k } => c::SurfaceBehavior::Tied { k },
                });
            }
            InteractionProperty::Friction(friction) => {
                out.friction = Some(c::Friction {
                    coefficient: friction.coefficient,
                    stick_slope: friction.stick_slope.or(defaults.stick_slope),
                });
            }
            InteractionProperty::GapConductance(conductance) => {
                out.gap_conductance = Some(match conductance {
                    GapConductance::Constant(value) => c::GapConductance::Constant(*value),
                    GapConductance::Tabular(rows) => {
                        let rows: Vec<(f64, f64, f64)> = (rows.iter())
                            .map(|row| {
                                let at = |i: usize| row.get(i).copied().unwrap_or(0.0);
                                (at(0), at(1), at(2))
                            })
                            .collect();
                        match rows.as_slice() {
                            [(c, p, t)] if *p == 0.0 && *t == 0.0 => {
                                c::GapConductance::Constant(*c)
                            }
                            _ => c::GapConductance::Table(rows),
                        }
                    }
                });
            }
        }
    }
    out
}

/// Master and slave surface of each active contact pair, by the pair's name.
type PairSurfaces = BTreeMap<Id, (String, String)>;

/// Contact pairs: slave surface first; with
/// their clearances.
fn contact_pairs(
    sets: &mut Sets,
    model: &FeModel,
) -> Result<
    (
        Vec<ir::contact::ContactPair>,
        Vec<ir::contact::Clearance>,
        PairSurfaces,
    ),
    WriteError,
> {
    use ir::contact as c;
    let (mut pairs, mut clearances) = (Vec::new(), Vec::new());
    let mut surfaces = PairSurfaces::new();
    for pair in model.contact_pairs.iter().filter(|p| p.active) {
        let Some(interaction) = pair.interaction.and_then(|i| model.surface_interaction(i)) else {
            return Err(WriteError::UnknownInteraction {
                item: pair.name.clone(),
                interaction: model.name_or_missing(pair.interaction),
            });
        };
        // Faces on a symmetry plane are the cut of the half model, no surface to touch.
        let master_region = model.without_symmetry_faces(sets.mesh, &pair.master);
        let master = sets.surface(
            &pair.name,
            "Master",
            master_region.as_ref().unwrap_or(&pair.master),
        )?;
        // Beams and ropes as the slave side touch with their nodes, at the radius of a rope
        // profile from the axis: a node surface of node-to-surface contact.
        let lines = pair.slave.is_lines(sets.mesh);
        let slave = if lines {
            if pair.method != ContactMethod::NodeToSurface {
                return Err(WriteError::Unsupported {
                    item: pair.name.clone(),
                    reason: "a slave side of beams or ropes needs Node to Surface contact".into(),
                });
            }
            sets.node_surface(&pair.name, "Slave", &pair.slave)?
        } else {
            let slave_region = model.without_symmetry_faces(sets.mesh, &pair.slave);
            sets.surface(
                &pair.name,
                "Slave",
                slave_region.as_ref().unwrap_or(&pair.slave),
            )?
        };
        let kind = match pair.method {
            ContactMethod::NodeToSurface => c::ContactKind::NodeToSurface,
            ContactMethod::SurfaceToSurface => c::ContactKind::SurfaceToSurface,
            ContactMethod::Mortar => c::ContactKind::Mortar,
            ContactMethod::Massless => {
                return Err(WriteError::Unsupported {
                    item: pair.name.clone(),
                    reason: "the solver has no massless contact".into(),
                });
            }
        };
        pairs.push(c::ContactPair {
            interaction: interaction_id(model, interaction),
            kind,
            small_sliding: pair.method == ContactMethod::NodeToSurface && pair.small_sliding,
            adjust: (pair.adjust).then(|| c::Adjust::Distance(pair.adjustment_size.unwrap_or(0.0))),
            slave: sets.surface_id(&slave),
            master: sets.surface_id(&master),
        });
        if let Some(value) = pair.clearance {
            clearances.push(c::Clearance {
                master: sets.surface_id(&master),
                slave: sets.surface_id(&slave),
                value,
            });
        }
        surfaces.insert(pair.id.any(), (master, slave));
    }
    Ok((pairs, clearances, surfaces))
}

/// What the steps share: the model's settings and the items lowered before them.
struct StepContext<'a> {
    extra_boundary: Option<&'a Content>,
    space: ModelSpace,
    lines: &'a LineElements,
    amplitudes: &'a AmplitudeIds,
    pair_surfaces: &'a PairSurfaces,
    /// The model, for the coordinate systems of moving loads.
    model: &'a FeModel,
    /// The model is a cyclic symmetric sector: frequency steps select the nodal diameters.
    cyclic: bool,
    /// See [`cyclic_slave_nodes`].
    cyclic_slaves: &'a BTreeSet<NodeId>,
}

/// A step as the solver takes it: the procedure, then the
/// boundary conditions, loads, defined fields, history and field outputs. A deactivated
/// step, or item, is left out, and so are its sets.
///
/// A step with moving loads becomes one `*STEP` per position of the loads, all alike but
/// for the forces of the moving loads; a moving load with fewer positions than another
/// stays at its last one.
fn lower_step(
    sets: &mut Sets,
    step: &Step,
    context: &StepContext,
) -> Result<Vec<StepBlock>, WriteError> {
    if !step.active {
        return Ok(Vec::new());
    }
    let two_d = context.space.is_2d();
    let mut moving = Vec::new();
    for (index, load) in step.loads.iter().enumerate() {
        if let LoadKind::Moving(load_kind) = &load.kind
            && load.active
            && step.kind.supports_load(&load.kind)
        {
            if context.space == ModelSpace::Axisymmetric {
                return Err(WriteError::InvalidLoad {
                    item: load.name.clone(),
                    reason: "a moving load needs a 3D or plane model".into(),
                });
            }
            let faces = load.region.faces(sets.mesh);
            if faces.is_empty() {
                return Err(empty(&load.name, "element faces"));
            }
            let frames =
                (load_kind.frames(context.model, sets.mesh, &faces, two_d)).map_err(|reason| {
                    WriteError::InvalidLoad {
                        item: load.name.clone(),
                        reason,
                    }
                })?;
            moving.push((index, load_kind.stations(), frames));
        }
    }
    let first = step_block(sets, step, context, |index| {
        let (_, _, frames) = moving.iter().find(|(i, ..)| *i == index)?;
        frames.first()
    })?;
    let title = step_title(&step.name).filter(|_| moving.is_empty());
    let count = moving.iter().map(|(_, s, _)| s.len()).max().unwrap_or(1);
    let mut blocks = Vec::with_capacity(count);
    for position in 0..count {
        let mut parts = first.clone();
        if position > 0 {
            for (index, _, frames) in &moving {
                let load = &step.loads[*index];
                let frame = &frames[position.min(frames.len() - 1)];
                let forces = moving_load(load, frame, context)?;
                if let Some((_, slot)) = (parts.loads.iter_mut()).find(|(i, _)| i == index) {
                    *slot = forces;
                }
            }
        }
        let mut ir_step = parts.step;
        ir_step.name = title.clone().unwrap_or_default();
        let mut content = parts.boundaries;
        for (_, load) in parts.loads {
            content.add(load);
        }
        content.add(parts.rest);
        blocks.push(StepBlock {
            step: content.into_step(ir_step),
            named: title.is_some(),
            takes_over: parts.takes_over,
            preparation: false,
        });
    }
    if let StepKind::ComplexFrequency(settings) = &step.kind
        && let Some(complex) = blocks.pop()
    {
        return complex_frequency_blocks(sets, step, settings, context, complex);
    }
    // A Dynamic step by modal superposition superposes the eigenmodes a frequency step
    // right before it stores; the solver computes them with the step's supports, preloaded
    // as a perturbation of the previous step's state when asked.
    if let StepKind::Dynamic(settings) = &step.kind
        && let DynamicMethod::Modal { modes, base } = settings.method
        && let Some(dynamic) = blocks.pop()
    {
        if let Some(problem) = settings.problem() {
            return Err(WriteError::InvalidLoad {
                item: step.name.clone(),
                reason: problem,
            });
        }
        let modes = FrequencyStep {
            num_frequencies: modes,
            storage: true,
            base,
            solver: settings.solver,
            ..FrequencyStep::default()
        };
        let mut eigenmodes = frequency_step(&modes, context.cyclic);
        eigenmodes.name = format!("{} Eigenmodes", dynamic.step.name);
        eigenmodes.boundaries = dynamic.step.boundaries.clone();
        return Ok(vec![
            StepBlock {
                step: eigenmodes,
                named: dynamic.named,
                takes_over: TakesOver {
                    boundaries: false,
                    loads: true,
                    fields: true,
                },
                preparation: true,
            },
            dynamic,
        ]);
    }
    // A steady state dynamics step superposes eigenmodes it computes for itself in a
    // frequency step right before it, unless an Eigenmodes link of another analysis gives
    // them. Neither changes the state: the pair is an isolated group.
    if let StepKind::SteadyStateDynamics(settings) = &step.kind
        && context.model.properties.stored_modes.is_none()
        && let Some(mut response) = blocks.pop()
    {
        let modes = FrequencyStep {
            num_frequencies: settings.modes,
            storage: true,
            base: settings.base,
            ..FrequencyStep::default()
        };
        let mut eigenmodes = frequency_step(&modes, context.cyclic);
        eigenmodes.name = format!("{} Eigenmodes", response.step.name);
        eigenmodes.isolation.opens = true;
        response.step.perturbation = settings.base.is_preloaded();
        response.step.isolation.closes = true;
        // The step keeps the supports of its eigenmodes (new ones are refused there), so
        // the eigenmode step starts them anew.
        eigenmodes.boundaries = response.step.boundaries.clone();
        return Ok(vec![
            StepBlock {
                step: eigenmodes,
                named: response.named,
                takes_over: TakesOver {
                    boundaries: false,
                    loads: true,
                    fields: true,
                },
                preparation: true,
            },
            response,
        ]);
    }
    Ok(blocks)
}

/// The steps the solver solves a complex frequency step as: with stress stiffening a
/// static step that preloads the model by the centrifugal forces of the rotation, then a
/// frequency step that stores the eigenmodes (a perturbation of the preload with stress
/// stiffening, which also brings the spin softening), then the complex frequency step
/// `complex` on those modes. The rotation acts on all parts; the first step adds it to the
/// loads the steps before left, the others take it over.
fn complex_frequency_blocks(
    sets: &mut Sets,
    step: &Step,
    settings: &ComplexFrequencyStep,
    context: &StepContext,
    complex: StepBlock,
) -> Result<Vec<StepBlock>, WriteError> {
    let invalid = |reason: String| WriteError::InvalidLoad {
        item: step.name.clone(),
        reason,
    };
    if let Some(problem) = settings.problem() {
        return Err(invalid(problem));
    }
    let (point, axis) = settings.axis_in(context.model).map_err(invalid)?;
    // The rotating elements; no parts picked: all of them.
    let region = if settings.rotates_whole_model() {
        Region::Parts(sets.mesh.part_ids())
    } else {
        settings.region.clone()
    };
    let set = sets.element_set(&format!("{}_Rotation", step.name), &region)?;
    let rotation = vec![dload(
        sets.element_ref(&set),
        ir::step::DistributedLoadKind::Centrifugal {
            omega_squared: settings.speed * settings.speed,
            point,
            axis,
        },
        &None,
    )];
    let unloaded = !settings.base.is_preloaded();
    // Named after the step, so that they do not take the name of another one. With the
    // rotation the step loads the model: about the unloaded model the rotation is the only
    // load, otherwise it replaces the rotation of the loads the steps before left. Without,
    // the step takes the loads over.
    let helper = |mut ir_step: ir::Step, purpose: &str, rotation: Option<Vec<_>>| {
        ir_step.name = format!("{} {purpose}", complex.step.name);
        ir_step.boundaries = complex.step.boundaries.clone();
        let loads = rotation.is_none() || !unloaded;
        ir_step.loads.distributed = rotation.unwrap_or_default();
        StepBlock {
            step: ir_step,
            named: complex.named,
            takes_over: TakesOver {
                boundaries: false,
                loads,
                fields: true,
            },
            preparation: true,
        }
    };
    let mut blocks = Vec::with_capacity(3);
    let mut rotation = Some(rotation);
    if settings.stiffening {
        // The frequency step after it factorizes the same stiffness anyway, so the
        // automatic choice solves the preload directly as well: the multigrid of the
        // iterative solver converges badly on thin shells, which are solids of one
        // element through the thickness.
        let solver = match settings.solver {
            plx_model::LinearSolverChoice::Automatic => plx_model::LinearSolverChoice::Direct,
            other => other,
        };
        let preload = StaticStep {
            solver,
            ..StaticStep::default()
        };
        blocks.push(helper(
            static_step(&preload, false),
            "Preload",
            rotation.take(),
        ));
    }
    let preloaded = settings.stiffening || settings.base.is_preloaded();
    let modes = FrequencyStep {
        num_frequencies: settings.num_modes,
        storage: true,
        base: if preloaded {
            plx_model::Base::PreviousState
        } else {
            plx_model::Base::Unloaded
        },
        solver: settings.solver,
        ..FrequencyStep::default()
    };
    blocks.push(helper(
        frequency_step(&modes, context.cyclic),
        "Eigenmodes",
        rotation,
    ));
    let mut complex = complex;
    complex.step.perturbation = preloaded;
    // The group leaves the state as it found it: the step after it starts from where the
    // steps before the complex frequency step ended.
    complex.step.isolation.closes = true;
    blocks.push(complex);
    let first = &mut blocks[0].step.isolation;
    first.opens = true;
    first.unloaded = unloaded;
    Ok(blocks)
}

/// Couples the selection of a rigid body or point mass to its reference point, rigidly
/// (`*RIGID BODY` on a reference and a rotation node at the point) or deformably (the point
/// follows the mean motion, as the point of a deformable remote displacement does). The
/// nodes are those of the point from then on.
fn couple_point(
    index: usize,
    coupling: plx_model::Coupling<'_>,
    sets: &mut Sets,
    model: &FeModel,
    generated: &mut constraints::Generated,
    next_node: &mut NodeId,
    (items, owners): (&mut Vec<ConstraintItem>, &mut Vec<Option<ModelItem>>),
) -> Result<ReferenceNodes, WriteError> {
    let Some(point) = model.reference_point(coupling.reference_point) else {
        return Err(WriteError::UnknownReferencePoint {
            item: coupling.name.to_string(),
            point: coupling.reference_point.to_string(),
        });
    };
    if coupling.behavior == RemoteBehavior::Deformable {
        let spec = plx_model::Displacement {
            values: [None; 6],
            behavior: plx_model::Behavior::Deformable,
            system: None,
            location: Some(point.position),
            pinball: None,
            tables: Default::default(),
        };
        let invalid = |reason: String| WriteError::InvalidConstraint {
            item: coupling.name.to_string(),
            reason,
        };
        let excluded = model.directly_held_nodes_in(sets.mesh, |_| true);
        let held = model.held_translations(sets.mesh);
        let resolved =
            (spec.resolve(model, sets.mesh, coupling.region, &excluded)).map_err(invalid)?;
        let [reference, rotation] = remote_point(
            RemoteSource {
                item: coupling.name,
                region: coupling.region,
                spec: &spec,
                behavior: RemoteBehavior::Deformable,
                owner: ModelItem::Constraint(index),
                free_point: !model.point_bound(coupling.reference_point),
            },
            resolved,
            held,
            next_node,
            (items, owners),
            sets,
            model,
            generated,
        )?;
        let nodes = ReferenceNodes {
            name: point.name.clone(),
            reference,
            rotation,
            set: None,
            behavior: RemoteBehavior::Deformable,
        };
        sets.reference_nodes
            .insert(coupling.reference_point, nodes.clone());
        return Ok(nodes);
    }
    // Both nodes sit on the reference point.
    let (reference, rotation) = (*next_node, *next_node + 1);
    *next_node += 2;
    generated.nodes.push((reference, point.position));
    generated.nodes.push((rotation, point.position));
    let nodes = ReferenceNodes {
        name: point.name.clone(),
        reference,
        rotation,
        set: None,
        behavior: RemoteBehavior::Rigid,
    };
    sets.reference_nodes
        .insert(coupling.reference_point, nodes.clone());
    owners.resize(items.len(), None);
    rigid_body(
        items,
        sets,
        coupling.name,
        coupling.region,
        [reference, rotation],
    )?;
    owners.resize(items.len(), Some(ModelItem::Constraint(index)));
    Ok(nodes)
}

/// A rigid body of the nodes of `region` driven by a reference and a rotation node at
/// `position`. The solver couples the nodes of beams and shells by their expanded nodes, so
/// the body holds their rotations too.
fn rigid_body(
    items: &mut Vec<ConstraintItem>,
    sets: &mut Sets,
    item: &str,
    region: &Region,
    nodes: [NodeId; 2],
) -> Result<(), WriteError> {
    let [reference, rotation] = nodes;
    // Nodes that a boundary condition or another constraint binds already stay out.
    let all = region.nodes(sets.mesh);
    let count = all.len();
    let kept = sets.dofs.claim_nodes(item, all);
    let set = if kept.len() == count {
        sets.node_set(item, region)?
    } else {
        sets.node_set(item, &Region::Nodes(kept))?
    };
    items.push(ConstraintItem::RigidBody(ir::constraints::RigidBody {
        nodes: ir::constraints::RigidBodyNodes::NodeSet(sets.node_id(&set)),
        reference_node: reference,
        rotation_node: rotation,
    }));
    Ok(())
}

/// How the displacement boundary condition of the id in the step of the id is written.
fn formulation(
    model: &FeModel,
    mesh: &FeMesh,
    step: Id<Step>,
    bc: Id<plx_model::BoundaryCondition>,
) -> Option<Formulation> {
    let s = model.step_index(step)?;
    let i = (model.steps[s].boundary_conditions.iter()).position(|b| b.id == bc)?;
    model.formulation(mesh, s, i)
}

/// The terms of an `*Equation` of a remote point: the point's translations and rotations on
/// the reference and rotation node, turned into the point's directions, which the
/// `*Transform` of the two nodes makes theirs.
fn remote_terms(
    equation: &[(RemoteDof, f64)],
    axes: Option<[[f64; 3]; 3]>,
    [reference, rotation]: [NodeId; 2],
) -> Vec<dofs::Term> {
    let mut terms: Vec<dofs::Term> = Vec::new();
    let mut add = |node: NodeId, dof: u8, value: f64| match terms
        .iter_mut()
        .find(|(n, d, _)| *n == node && *d == dof)
    {
        Some(term) => term.2 += value,
        None => terms.push((node, dof, value)),
    };
    for &(dof, value) in equation {
        let (node, global) = match dof {
            RemoteDof::Node(node, k) => {
                add(node, k as u8 + 1, value);
                continue;
            }
            RemoteDof::Translation(k) => (reference, k),
            RemoteDof::Rotation(k) => (rotation, k),
        };
        match axes {
            // The global direction k is sum_j axes[j][k] times the local direction j.
            Some(axes) => {
                for (j, axis) in axes.iter().enumerate() {
                    if axis[global] != 0.0 {
                        add(node, j as u8 + 1, value * axis[global]);
                    }
                }
            }
            None => add(node, global as u8 + 1, value),
        }
    }
    terms.retain(|(_, _, v)| *v != 0.0);
    terms
}

/// An equation of its terms, the dependent one first.
fn equation(terms: &[dofs::Term]) -> ir::constraints::Equation {
    ir::constraints::Equation {
        terms: (terms.iter())
            .map(|&(node, dof, coefficient)| ir::constraints::EquationTerm {
                node: NodeRef::Node(node),
                dof,
                coefficient,
            })
            .collect(),
    }
}

/// A boundary condition on the degrees of freedom `first` to `last` of `region`.
fn boundary(
    region: NodeRef,
    first: u8,
    last: u8,
    value: Option<f64>,
    amplitude: Option<ir::AmplitudeId>,
) -> ir::step::Boundary {
    ir::step::Boundary {
        region,
        first_dof: first,
        last_dof: last,
        value,
        amplitude,
        fixed: false,
        submodel_step: None,
        load_case: None,
    }
}

/// A concentrated load on a degree of freedom of `region`.
/// The components of a force or moment in global directions at `position`, from the
/// coordinate system of the load.
fn global_components(
    model: &FeModel,
    load: &plx_model::Load,
    position: [f64; 3],
    local: [f64; 3],
) -> Result<[f64; 3], WriteError> {
    let invalid = |reason: String| WriteError::InvalidLoad {
        item: load.name.clone(),
        reason,
    };
    let system = load.component_system(model).map_err(invalid)?;
    plx_model::Load::to_global(system.as_ref(), position, local).map_err(invalid)
}

/// Where a concentrated force or moment on `target` acts and the position its directions
/// are taken at: the whole target in a rectangular system (the same directions everywhere),
/// every node of it at its own position in a cylindrical one.
fn component_positions(
    model: &FeModel,
    sets: &mut Sets,
    load: &plx_model::Load,
    target: &NodeRef,
) -> Result<Vec<(NodeRef, [f64; 3])>, WriteError> {
    let invalid = |reason: String| WriteError::InvalidLoad {
        item: load.name.clone(),
        reason,
    };
    let system = load.component_system(model).map_err(invalid)?;
    if system.is_none_or(|s| s.kind != plx_model::CoordinateSystemKind::Cylindrical) {
        return Ok(vec![(target.clone(), [0.0; 3])]);
    }
    if let Some(point) = load.region.reference_point() {
        let position = (model.reference_point(point)).map_or([0.0; 3], |p| p.position);
        return Ok(vec![(target.clone(), position)]);
    }
    let nodes = load.region.nodes(sets.mesh);
    Ok((nodes.into_iter())
        .filter_map(|n| Some((NodeRef::Node(n), sets.mesh.node(n)?)))
        .collect())
}

/// What a follower load turns with: the selection its coordinate system moves with, the
/// rotation of its reference point, or the nodes of its region (a single beam or shell node
/// turns with its rotations in the solver). `set` is the node set of the region if it was
/// written already.
fn follower(
    model: &FeModel,
    sets: &mut Sets,
    load: &plx_model::Load,
    set: Option<&NodeRef>,
) -> Result<Option<ir::step::Follower>, WriteError> {
    if !load.kind.takes_system() {
        return Ok(None);
    }
    if let Some(follower) = moving_system_follower(model, sets, load.system)? {
        return Ok(Some(follower));
    }
    if !load.follower {
        return Ok(None);
    }
    Ok(Some(match (load.region.reference_point(), set) {
        (Some(point), _) => {
            ir::step::Follower::Rotation(sets.reference_nodes(&load.name, point)?.rotation)
        }
        (None, Some(set)) => ir::step::Follower::Nodes(set.clone()),
        (None, None) => ir::step::Follower::Nodes(sets.region_ref(&load.name, &load.region)?),
    }))
}

/// A load in a coordinate system that moves with a selection turns with the selection's
/// nodes; `None` for systems fixed in space.
fn moving_system_follower(
    model: &FeModel,
    sets: &mut Sets,
    system: Option<plx_model::Id<plx_model::CoordinateSystem>>,
) -> Result<Option<ir::step::Follower>, WriteError> {
    let Some(region) = model.system_attachment(system) else {
        return Ok(None);
    };
    let name = model.system_name(system);
    let set = sets.node_set(&name, region)?;
    Ok(Some(ir::step::Follower::Nodes(sets.node_ref(&set))))
}

fn set_follower(items: &mut [ir::step::ConcentratedLoad], follower: Option<ir::step::Follower>) {
    for item in items {
        item.follower.clone_from(&follower);
    }
}

fn cload(
    region: NodeRef,
    dof: u8,
    value: f64,
    amplitude: &Option<ir::AmplitudeId>,
) -> ir::step::ConcentratedLoad {
    ir::step::ConcentratedLoad {
        region,
        dof,
        value,
        amplitude: *amplitude,
        load_case: None,
        sector: None,
        follower: None,
    }
}

/// A distributed load on `region`.
fn dload(
    region: ElementRef,
    kind: ir::step::DistributedLoadKind,
    amplitude: &Option<ir::AmplitudeId>,
) -> ir::step::DistributedLoad {
    ir::step::DistributedLoad {
        region,
        kind,
        amplitude: *amplitude,
        load_case: None,
        sector: None,
    }
}

/// The boundary conditions of prescribed displacements and rotations of the first `dofs`
/// degrees of freedom, a range for each run of equal values. The rotations go to the
/// translations of `rotation`, the rotation node of a rigid body, if there is one.
fn displacement_boundaries(
    nodes: &NodeRef,
    rotation: Option<&NodeRef>,
    values: &[Option<f64>; 6],
    dofs: usize,
    amplitudes: &[Option<ir::AmplitudeId>; 6],
) -> Vec<ir::step::Boundary> {
    // Each held degree of freedom as (target, its number there, value, amplitude).
    let held = (1..)
        .zip(values.iter().zip(amplitudes))
        .take(dofs)
        .filter_map(|(dof, (value, amplitude))| {
            let value = (*value)?;
            Some(match rotation {
                Some(rotation) if dof > 3 => (rotation, dof - 3, value, amplitude),
                _ => (nodes, dof, value, amplitude),
            })
        });
    let mut out = Vec::new();
    let mut run: Option<(&NodeRef, u8, u8, f64, &Option<ir::AmplitudeId>)> = None;
    for (target, dof, value, amplitude) in held {
        match &mut run {
            Some((t, _, last, v, a))
                if *t == target && *last + 1 == dof && *v == value && *a == amplitude =>
            {
                *last = dof;
            }
            _ => {
                if let Some((t, first, last, v, a)) = run {
                    out.push(boundary(t.clone(), first, last, Some(v), *a));
                }
                run = Some((target, dof, dof, value, amplitude));
            }
        }
    }
    if let Some((t, first, last, v, a)) = run {
        out.push(boundary(t.clone(), first, last, Some(v), *a));
    }
    out
}

/// A moving load at one position: its nodal forces as concentrated loads; none while all
/// wheels are off the region.
fn moving_load(
    load: &plx_model::Load,
    frame: &MovingFrame,
    context: &StepContext,
) -> Result<Content, WriteError> {
    if frame.forces.is_empty() {
        return Ok(Content::default());
    }
    let dofs = if context.space.is_2d() { 2 } else { 3 };
    let amplitude = amplitude_parameter(context.amplitudes, &load.name, &load.amplitude)?;
    let mut items = Vec::new();
    for (node, values) in &frame.forces {
        for (dof, value) in (1..).zip(values).take(dofs) {
            if *value != 0.0 {
                items.push(cload(NodeRef::Node(*node), dof, *value, &amplitude));
            }
        }
    }
    Ok(Content::cloads(items))
}

/// A step before its moving loads are placed: its settings, what it takes over from the
/// steps before, its boundary conditions, each load (by the load's index) and what comes
/// after the loads.
#[derive(Clone)]
struct StepParts {
    step: ir::Step,
    takes_over: TakesOver,
    boundaries: Content,
    loads: Vec<(usize, Content)>,
    rest: Content,
}

/// A step and everything in it, with the moving loads at the positions `moving` gives by
/// load index.
fn step_block<'f>(
    sets: &mut Sets,
    step: &Step,
    context: &StepContext,
    moving: impl Fn(usize) -> Option<&'f MovingFrame>,
) -> Result<StepParts, WriteError> {
    let &StepContext {
        extra_boundary,
        space,
        lines,
        amplitudes,
        pair_surfaces,
        cyclic,
        cyclic_slaves,
        model,
    } = context;
    // Nodes of 2D models move in the x-y plane only; the solver fails on rotations there.
    let dofs = if space.is_2d() { 2 } else { 6 };
    // Every how many increments the field outputs are written.
    let mut frame_every = 1;
    // Large deformations, also when the materials switch them on: they start the automatic
    // increments shorter.
    let nlgeom = model.step_nlgeom(&step.kind);
    let mut ir_step = match &step.kind {
        StepKind::Static(settings) => static_step(settings, nlgeom),
        StepKind::Frequency(settings) => frequency_step(settings, cyclic),
        StepKind::ComplexFrequency(settings) => complex_frequency_step(settings),
        StepKind::Buckle(settings) => buckle_step(settings),
        StepKind::HeatTransfer(settings) => heat_transfer_step(settings, HeatKind::Heat, false),
        StepKind::CoupledTempDisp(settings) => {
            heat_transfer_step(settings, HeatKind::Coupled, nlgeom)
        }
        StepKind::UncoupledTempDisp(settings) => {
            heat_transfer_step(settings, HeatKind::Uncoupled, false)
        }
        StepKind::Dynamic(settings) => {
            let (ir_step, per_frame) = dynamic_step(settings, model, step);
            frame_every = per_frame;
            ir_step
        }
        StepKind::Visco(settings) => visco_step(settings, nlgeom),
        StepKind::SteadyStateDynamics(settings) => steady_state_dynamics_step(settings),
        StepKind::Sensitivity(settings) => sensitivity_step(sets, &step.name, settings)?,
        // The particle solver takes the model as it is (`crate::particles`), not as input.
        StepKind::ParticleFlow(_) => {
            return Err(WriteError::Unsupported {
                item: step.name.clone(),
                reason: "a Particle Flow step is solved by the particle solver, not as an \
                         input file"
                    .into(),
            });
        }
    };
    // Hyperelastic materials deform with large strains: steps that deform them take NLGEOM.
    if model.step_nlgeom(&step.kind)
        && matches!(
            ir_step.procedure,
            ir::step::Procedure::Static(_)
                | ir::step::Procedure::Visco(_)
                | ir::step::Procedure::Dynamic(_)
        )
    {
        ir_step.nlgeom = true;
    }
    ir_step.contact_changes = contact_changes(sets, step, model, pair_surfaces);
    // A modal step keeps the supports of the frequency step that stored the modes: the
    // solver refuses new ones ("in a modal dynamic step new SPCs are not allowed"), and
    // giving them again is accepted.
    let mut takes_over = TakesOver {
        boundaries: step.kind.uses_stored_modes() || step.kind.keeps_previous_boundaries(),
        ..TakesOver::default()
    };
    let mut boundaries = Content::default();
    for bc in &step.boundary_conditions {
        // A step leaves out what it cannot take, like a deactivated item: displacements in
        // a heat transfer step, temperatures in a static one.
        if !bc.active || !step.kind.supports_boundary(&bc.kind) {
            continue;
        }
        let on_point = sets.remote_names.contains_key(&bc.id.any())
            && matches!(
                formulation(model, sets.mesh, step.id, bc.id),
                Some(Formulation::Point(_))
            );
        if let BoundaryKind::Displacement(remote) = &bc.kind
            && on_point
        {
            let Some(&(_, _, [reference, rotation])) = (sets.remote_points.iter())
                .find(|(r, p, _)| *r == bc.region && p.same_point(remote))
            else {
                return Err(empty(&bc.name, "nodes"));
            };
            let amplitude = amplitude_parameter(amplitudes, &bc.name, &bc.amplitude)?;
            // The translations are those of the reference node, the rotations the
            // displacements of the rotation node.
            let (values, dof_amplitudes) =
                displacement_values(step, bc, remote, &amplitude, amplitudes);
            let held = remote.held(space).map(|v| v.map(|_| ()));
            let held = std::array::from_fn(|d| held[d].and(values[d]));
            let (reference, rotation) = (NodeRef::Node(reference), NodeRef::Node(rotation));
            boundaries.add(Content::boundaries(displacement_boundaries(
                &reference,
                Some(&rotation),
                &held,
                6,
                &dof_amplitudes,
            )));
            continue;
        }
        // Truss nodes have no rotations either; the solver fails when they are fixed.
        let dofs = if dofs == 6 && lines.all_truss(&bc.region.nodes(sets.mesh)) {
            3
        } else {
            dofs
        };
        // A distributed temperature takes a value per node, which needs no set.
        let distributed = match bc.kind {
            BoundaryKind::Temperature(_) => distribution(model, &bc.name, &bc.distribution)?,
            _ => None,
        };
        // Nodes held in different systems take the boundary condition in their own system,
        // node by node; the set keeps the others.
        let mut region = std::borrow::Cow::Borrowed(&bc.region);
        if !sets.node_frames.is_empty() && distributed.is_none() {
            let invalid = |reason: String| WriteError::InvalidBoundary {
                item: bc.name.clone(),
                reason,
            };
            let form = formulation(model, sets.mesh, step.id, bc.id);
            let hold = node_frames::hold(model, sets.mesh, bc, form, dofs).map_err(invalid)?;
            if let Some(hold) = hold.filter(|h| !h.submodel) {
                let (own, rest): (Vec<NodeId>, Vec<NodeId>) = (bc.region.nodes(sets.mesh))
                    .into_iter()
                    .filter(|n| !cyclic_slaves.contains(n))
                    .partition(|n| sets.node_frames.contains_key(n));
                if !own.is_empty() {
                    let reference = bc.amplitude.filter(|_| bc.kind.takes_amplitude());
                    let amplitude = amplitude_parameter(amplitudes, &bc.name, &reference)?;
                    let dof_amplitudes = match &bc.kind {
                        BoundaryKind::Displacement(displacement) => {
                            displacement_values(step, bc, displacement, &amplitude, amplitudes).1
                        }
                        _ => std::array::from_fn(|_| amplitude),
                    };
                    let mut items = Vec::new();
                    for node in own {
                        let point = sets.mesh.node(node).unwrap_or_default();
                        let at = hold
                            .at_node(point, &sets.node_frames[&node])
                            .map_err(invalid)?;
                        let values = at.map(|v| v.map(|(value, _)| value));
                        let node_amplitudes =
                            std::array::from_fn(|j| at[j].and_then(|(_, d)| dof_amplitudes[d]));
                        items.extend(displacement_boundaries(
                            &NodeRef::Node(node),
                            None,
                            &values,
                            dofs,
                            &node_amplitudes,
                        ));
                    }
                    boundaries.add(Content::boundaries(items));
                    if rest.is_empty() {
                        continue;
                    }
                    region = std::borrow::Cow::Owned(Region::Nodes(rest));
                }
            }
        }
        let set = if let Some(sampler) = &distributed {
            let nodes: Vec<NodeId> = (bc.region.nodes(sets.mesh).into_iter())
                .filter(|n| !cyclic_slaves.contains(n))
                .collect();
            if nodes.is_empty() {
                return Err(empty(&bc.name, "nodes"));
            }
            let amplitude = amplitude_parameter(amplitudes, &bc.name, &bc.amplitude)?;
            let mut items = Vec::new();
            if let BoundaryKind::Temperature(t) = bc.kind {
                let units = model.properties.units;
                for (node, value) in distributed_nodes(sets.mesh, &nodes, t, sampler, units) {
                    items.push(boundary(node, 11, 11, Some(value), amplitude));
                }
            }
            boundaries.add(Content::boundaries(items));
            continue;
        } else if cyclic_slaves.is_empty() {
            sets.node_set(&bc.name, &region)?
        } else {
            let nodes = region.nodes(sets.mesh);
            if nodes.iter().any(|n| cyclic_slaves.contains(n)) {
                let free = nodes.into_iter().filter(|n| !cyclic_slaves.contains(n));
                sets.node_set(&bc.name, &Region::Nodes(free.collect()))?
            } else {
                sets.node_set(&bc.name, &region)?
            }
        };
        let target = sets.node_ref(&set);
        // The rotations of a rigid body are the displacements of its rotation node.
        let rotation_node = match bc.region.reference_point() {
            Some(point) => Some(sets.reference_nodes(&bc.name, point)?.rotation),
            None => None,
        };
        // Symmetry supports stay zero; an amplitude would not change them.
        let reference = bc.amplitude.filter(|_| bc.kind.takes_amplitude());
        let amplitude = amplitude_parameter(amplitudes, &bc.name, &reference)?;
        let mut items = Vec::new();
        match bc.kind {
            BoundaryKind::Displacement(ref displacement) => {
                let (values, dof_amplitudes) =
                    displacement_values(step, bc, displacement, &amplitude, amplitudes);
                // A coordinate system of its own turns the directions of the nodes.
                if displacement.system.is_some() && rotation_node.is_none() {
                    let system = (model.coordinate_system_or_global(displacement.system))
                        .ok_or_else(|| WriteError::InvalidBoundary {
                            item: bc.name.clone(),
                            reason: "The coordinate system does not exist".into(),
                        })?;
                    let invalid = |reason: String| WriteError::InvalidBoundary {
                        item: bc.name.clone(),
                        reason,
                    };
                    let [x, y, z] = system.axes().map_err(invalid)?;
                    let transform = match system.kind {
                        plx_model::CoordinateSystemKind::Rectangular => {
                            Transform::Rectangular(x, y)
                        }
                        plx_model::CoordinateSystemKind::Cylindrical => Transform::Cylindrical(
                            system.origin,
                            std::array::from_fn(|k| system.origin[k] + z[k]),
                        ),
                    };
                    if !sets.transforms.iter().any(|(s, ..)| *s == set) {
                        sets.transforms.push((set.clone(), transform));
                    }
                }
                let rotation = rotation_node.map(NodeRef::Node);
                // The point of a rigid body turns in a plane model too: about z, the
                // rotation node's third degree of freedom. Its other directions do not exist.
                if rotation.is_some() && space.is_2d() {
                    let held = std::array::from_fn(|i| {
                        space
                            .remote_dofs()
                            .contains(&i)
                            .then(|| values[i])
                            .flatten()
                    });
                    items = displacement_boundaries(
                        &target,
                        rotation.as_ref(),
                        &held,
                        6,
                        &dof_amplitudes,
                    );
                } else {
                    items = displacement_boundaries(
                        &target,
                        rotation.as_ref(),
                        &values,
                        dofs,
                        &dof_amplitudes,
                    );
                }
            }
            BoundaryKind::Temperature(t) => {
                items.push(boundary(target, 11, 11, Some(t), amplitude));
            }
            // Only the particle solver knows the flow.
            BoundaryKind::Flow(_) => {}
            // The global results hold displacements, no rotations.
            BoundaryKind::Submodel {
                dofs: held, step, ..
            } => {
                for dof in (1..=dofs.min(3)).filter(|&d| held[d - 1]) {
                    let mut item = boundary(target.clone(), dof as u8, dof as u8, None, None);
                    item.submodel_step = Some(step.max(1));
                    items.push(item);
                }
                if !sets.submodel_sets.iter().any(|(s, _)| *s == set) {
                    sets.submodel_sets.push((set, bc.name.clone()));
                }
            }
            BoundaryKind::Symmetry { normal, .. } => {
                let plane = symmetry_plane(sets.mesh, &bc.region, space, normal).map_err(|e| {
                    WriteError::NoSymmetryPlane {
                        item: bc.name.clone(),
                        reason: e.to_string(),
                    }
                })?;
                // A coordinate plane holds global degrees of freedom; any other plane gets a
                // local system with x along its normal, which turns rotations as well.
                let (axis, transformed) = match plane.axis() {
                    Some(axis) => (axis, false),
                    None => (0, true),
                };
                if transformed {
                    let (t1, _) = plane.in_plane_axes();
                    if !sets.transforms.iter().any(|(s, ..)| *s == set) {
                        (sets.transforms)
                            .push((set.clone(), Transform::Rectangular(plane.normal, t1)));
                    }
                }
                for dof in symmetry_dofs(axis, dofs) {
                    let dof = dof as u8 + 1;
                    items.push(boundary(target.clone(), dof, dof, Some(0.0), amplitude));
                }
            }
        }
        boundaries.add(Content::boundaries(items));
    }
    // Boundary conditions that constraints need.
    if step.kind.is_mechanical() {
        boundaries.add(extra_boundary.cloned().unwrap_or_default());
        boundaries.add(grounded_boundaries(sets, step, space).unwrap_or_default());
    }
    let mut loads: Vec<(usize, Content)> = Vec::new();
    // A step that takes no loads keeps those of the state it starts from.
    let kind = &step.kind;
    takes_over.loads =
        !kind.supports_load(&LoadKind::Pressure(0.0)) && !kind.supports_load(&LoadKind::flux(0.0));
    let step_loads: &[_] = if kind.supports_loads() {
        &step.loads
    } else {
        &[]
    };
    for (index, load) in step_loads.iter().enumerate() {
        if !load.active || !kind.supports_load(&load.kind) {
            continue;
        }
        // An excitation follows its signal instead of an amplitude.
        let amplitude = match load.kind {
            LoadKind::Excitation(_) => amplitudes.signal(&step.name, load.id.any()),
            _ => amplitude_parameter(amplitudes, &load.name, &load.amplitude)?,
        };
        // The second amplitude of a film scales its coefficient, of radiation the emissivity.
        let factor_amplitude = match load.kind {
            LoadKind::Film { .. } | LoadKind::Radiation { .. } => {
                amplitude_parameter(amplitudes, &load.name, &load.factor_amplitude)?
            }
            _ => None,
        };
        let content = match load.kind {
            LoadKind::WallLoads(ref walls) => {
                let invalid = |reason: String| WriteError::InvalidLoad {
                    item: load.name.clone(),
                    reason,
                };
                let file = (walls.file())
                    .ok_or_else(|| invalid("no Particle Flow step is linked to it".into()))?;
                let loads = crate::wall_loads::read(file).map_err(invalid)?;
                let sets_of_forces =
                    crate::wall_loads::nodal_forces(sets.mesh, &load.region, walls, &loads)
                        .map_err(invalid)?;
                let mut items = Vec::new();
                for (k, (time, mapped)) in sets_of_forces.into_iter().enumerate() {
                    let amplitude = match time {
                        Some(_) => amplitudes.walls(&step.name, load.id.any(), k),
                        None => amplitude,
                    };
                    for (node, force) in mapped.forces {
                        for (dof, value) in (1..).zip(force).take(dofs) {
                            if value != 0.0 {
                                items.push(cload(NodeRef::Node(node), dof, value, &amplitude));
                            }
                        }
                    }
                }
                if items.is_empty() {
                    return Err(invalid(
                        "no face of the region lies near the walls of the Particle Flow run".into(),
                    ));
                }
                Content::cloads(items)
            }
            LoadKind::Moving(_) => {
                let frame = moving(index).cloned().unwrap_or_default();
                loads.push((index, moving_load(load, &frame, context)?));
                continue;
            }
            LoadKind::Moment(moment) => {
                // Moments at a reference point act on the rotation node of its rigid body,
                // at nodes of beams and shells on their rotations.
                let (target, first_dof) = match load.region.reference_point() {
                    Some(point) => {
                        let nodes = sets.reference_nodes(&load.name, point)?;
                        (NodeRef::Node(nodes.rotation), 1)
                    }
                    None => (sets.region_ref(&load.name, &load.region)?, 4),
                };
                let (moment, component_amplitudes) = load_components(
                    step,
                    load,
                    moment,
                    MOMENT_TABLE_LABELS,
                    &amplitude,
                    amplitudes,
                );
                // In a system of its own, at every node in the directions there.
                let positions = component_positions(model, sets, load, &target)?;
                let mut items = Vec::new();
                for (target, position) in positions {
                    let moment = global_components(model, load, position, moment)?;
                    for (dof, value) in (0..).zip(moment) {
                        if value != 0.0 && dof as usize + 4 <= dofs {
                            let amplitude = &component_amplitudes[dof as usize];
                            items.push(cload(target.clone(), first_dof + dof, value, amplitude));
                        }
                    }
                }
                let set = load.region.reference_point().is_none().then_some(&target);
                set_follower(&mut items, follower(model, sets, load, set)?);
                Content::cloads(items)
            }
            LoadKind::RemoteForce(ref remote) => {
                // The force acts on the reference node of the point, the moment on its
                // rotation node, in the point's directions; a reference point takes them in
                // global directions.
                let (nodes, force, moment) = match load.region.reference_point() {
                    Some(point) => {
                        let nodes = sets.reference_nodes(&load.name, point)?;
                        let position =
                            (model.reference_point(point)).map_or([0.0; 3], |p| p.position);
                        let (force, moment) =
                            (remote.global(model, position)).map_err(|reason| {
                                WriteError::InvalidLoad {
                                    item: load.name.clone(),
                                    reason,
                                }
                            })?;
                        ([nodes.reference, nodes.rotation], force, moment)
                    }
                    None => {
                        let spec = remote.point();
                        let Some(&(_, _, nodes)) = (sets.remote_points.iter())
                            .find(|(r, p, _)| *r == load.region && p.same_point(&spec))
                        else {
                            return Err(empty(&load.name, "nodes"));
                        };
                        let allowed = space.remote_dofs();
                        let keep = |v: [f64; 3], offset: usize| {
                            std::array::from_fn(|k| {
                                if allowed.contains(&(k + offset)) {
                                    v[k]
                                } else {
                                    0.0
                                }
                            })
                        };
                        (nodes, keep(remote.force, 0), keep(remote.moment, 3))
                    }
                };
                let mut items = Vec::new();
                for (node, values) in nodes.into_iter().zip([force, moment]) {
                    for (dof, value) in (1..).zip(values) {
                        if value != 0.0 {
                            items.push(cload(NodeRef::Node(node), dof, value, &amplitude));
                        }
                    }
                }
                set_follower(
                    &mut items,
                    moving_system_follower(model, sets, remote.system)?,
                );
                Content::cloads(items)
            }
            LoadKind::Pressure(pressure) => {
                use ir::step::DistributedLoadKind::Pressure;
                let ([pressure], [amplitude]) =
                    load_components(step, load, [pressure], ["P"], &amplitude, amplitudes);
                let mut items = Vec::new();
                match distribution(model, &load.name, &load.distribution)? {
                    // Every face takes the pressure scaled at its center.
                    Some(sampler) => {
                        let faces = load.region.faces(sets.mesh);
                        if faces.is_empty() {
                            return Err(empty(&load.name, "element faces"));
                        }
                        for (element, face) in faces {
                            if let Some(center) = face_center(sets.mesh, element, face) {
                                let value = pressure * sampler.factor(center);
                                let region = ElementRef::Element(element);
                                items.push(dload(region, Pressure { face, value }, &amplitude));
                            }
                        }
                    }
                    None => {
                        for (set, face) in sets.face_sets(&load.name, &load.region)? {
                            let kind = Pressure {
                                face,
                                value: pressure,
                            };
                            items.push(dload(sets.element_ref(&set), kind, &amplitude));
                        }
                    }
                }
                Content::dloads(items)
            }
            LoadKind::PreTension {
                value,
                by_displacement,
                lock,
                ..
            } => {
                // The preload is the first degree of freedom of the pre-tension node: a
                // force, or the shortening of the bolt since the start as a prescribed
                // displacement. Locked, the node keeps the shortening of the previous step, or
                // by displacement adds the value to it.
                let node = NodeRef::Node(sets.pre_tension_nodes[&load.name]);
                if lock {
                    let mut item = if by_displacement {
                        boundary(node, 1, 1, Some(value), amplitude)
                    } else {
                        boundary(node, 1, 1, None, None)
                    };
                    item.fixed = true;
                    Content::boundaries(vec![item])
                } else if by_displacement {
                    Content::boundaries(vec![boundary(node, 1, 1, Some(value), amplitude)])
                } else {
                    Content::cloads(vec![cload(node, 1, value, &amplitude)])
                }
            }
            // An excitation is a force times its signal.
            LoadKind::Force {
                value: force,
                each_node,
            }
            | LoadKind::Excitation(plx_model::Excitation {
                value: force,
                each_node,
                ..
            }) => {
                let faces = load.region.faces(sets.mesh);
                let edges = load.region.edges(sets.mesh);
                let axisymmetric = space == ModelSpace::Axisymmetric;
                let (force, component_amplitudes) = load_components(
                    step,
                    load,
                    force,
                    FORCE_TABLE_LABELS,
                    &amplitude,
                    amplitudes,
                );
                let mut items = Vec::new();
                let mut region_set = None;
                // On points the nodes share the force equally, a reference point takes it
                // whole: the share at every node of their set. With `each_node` every node
                // of the region takes the whole force.
                if each_node || (faces.is_empty() && edges.is_empty()) {
                    let count = match load.region.reference_point() {
                        _ if each_node => 1,
                        Some(_) => 1,
                        None => load.region.nodes(sets.mesh).len(),
                    };
                    let target = sets.region_ref(&load.name, &load.region)?;
                    region_set = Some(target.clone());
                    for (target, position) in component_positions(model, sets, load, &target)? {
                        let force = global_components(model, load, position, force)?;
                        for (dof, value) in (1..).zip(force).take(dofs) {
                            if value != 0.0 {
                                let share = value / count as f64;
                                let amplitude = &component_amplitudes[dof as usize - 1];
                                items.push(cload(target.clone(), dof, share, amplitude));
                            }
                        }
                    }
                } else {
                    // On faces by area, on picked edges by length.
                    let nodal = if faces.is_empty() {
                        edge_traction_forces(sets.mesh, &edges, force, axisymmetric)
                    } else {
                        traction_forces(sets.mesh, &faces, force, axisymmetric)
                    };
                    if nodal.is_empty() {
                        return Err(empty(&load.name, "element faces or edges"));
                    }
                    for (node, values) in nodal {
                        let position = sets.mesh.node(node).unwrap_or([0.0; 3]);
                        let values = global_components(model, load, position, values)?;
                        for (dof, value) in (1..).zip(values).take(dofs) {
                            if value != 0.0 {
                                let amplitude = &component_amplitudes[dof as usize - 1];
                                items.push(cload(NodeRef::Node(node), dof, value, amplitude));
                            }
                        }
                    }
                }
                set_follower(
                    &mut items,
                    follower(model, sets, load, region_set.as_ref())?,
                );
                Content::cloads(items)
            }
            LoadKind::Flux {
                value: flux,
                each_node,
            } => {
                let faces = load.region.faces(sets.mesh);
                let edges = load.region.edges(sets.mesh);
                let region = (load.region.mesh_set(sets.mesh)).unwrap_or(load.region.clone());
                let bodies = matches!(region, Region::Parts(_) | Region::ElementSet(_));
                let axisymmetric = space == ModelSpace::Axisymmetric;
                // On points the nodes share the heat flow equally; with `each_node` every
                // node of the region takes the whole of it.
                let items = if each_node || (faces.is_empty() && edges.is_empty() && !bodies) {
                    let count = match each_node {
                        true => 1,
                        false => load.region.nodes(sets.mesh).len().max(1),
                    };
                    let set = sets.node_set(&load.name, &load.region)?;
                    vec![ir::step::ConcentratedFlux {
                        region: sets.node_ref(&set),
                        value: flux / count as f64,
                        amplitude,
                    }]
                } else {
                    // On faces by area, on picked edges by length, in parts by volume.
                    let nodal = if !faces.is_empty() {
                        traction_forces(sets.mesh, &faces, [flux, 0.0, 0.0], axisymmetric)
                    } else if !edges.is_empty() {
                        edge_traction_forces(sets.mesh, &edges, [flux, 0.0, 0.0], axisymmetric)
                    } else {
                        let elements = load.region.elements(sets.mesh);
                        volume_fluxes(sets.mesh, &elements, flux, axisymmetric)
                    };
                    if nodal.is_empty() {
                        return Err(empty(&load.name, "element faces, edges or volumes"));
                    }
                    (nodal.into_iter())
                        .filter(|(_, value)| value[0] != 0.0)
                        .map(|(node, value)| ir::step::ConcentratedFlux {
                            region: NodeRef::Node(node),
                            value: value[0],
                            amplitude,
                        })
                        .collect()
                };
                Content::cfluxes(items)
            }
            LoadKind::Film { sink, coefficient } => {
                let items = (sets.face_sets(&load.name, &load.region)?.into_iter())
                    .map(|(set, face)| ir::step::Film {
                        region: sets.element_ref(&set),
                        face,
                        sink_temperature: sink,
                        coefficient,
                        amplitude,
                        film_amplitude: factor_amplitude,
                    })
                    .collect();
                Content::films(items)
            }
            LoadKind::Radiation { sink, emissivity } => {
                let items = (sets.face_sets(&load.name, &load.region)?.into_iter())
                    .map(|(set, face)| ir::step::Radiate {
                        region: sets.element_ref(&set),
                        face,
                        cavity: false,
                        sink_temperature: sink,
                        emissivity,
                        amplitude,
                        radiation_amplitude: factor_amplitude,
                    })
                    .collect();
                Content::radiations(items)
            }
            // Gravity: the size of the acceleration and its direction.
            LoadKind::Gravity(_) | LoadKind::StandardGravity(_) => {
                let acceleration = load.kind.gravity_vector().unwrap_or_default();
                let set = sets.body_load_set(&load.name, &load.region)?;
                let Some((size, direction)) = unit_vector(acceleration) else {
                    return Err(WriteError::InvalidLoad {
                        item: load.name.clone(),
                        reason: "the gravitational acceleration is zero".into(),
                    });
                };
                let kind = ir::step::DistributedLoadKind::Gravity {
                    magnitude: size,
                    direction,
                };
                Content::dloads(vec![dload(sets.element_ref(&set), kind, &amplitude)])
            }
            // Centrifugal load: the square of the rotational speed, a point on
            // the axis and the axis direction.
            LoadKind::Centrifugal { point, axis, speed } => {
                let set = sets.body_load_set(&load.name, &load.region)?;
                let Some((_, direction)) = unit_vector(axis) else {
                    return Err(WriteError::InvalidLoad {
                        item: load.name.clone(),
                        reason: "the rotation axis has no direction".into(),
                    });
                };
                let kind = ir::step::DistributedLoadKind::Centrifugal {
                    omega_squared: speed * speed,
                    point,
                    axis: direction,
                };
                Content::dloads(vec![dload(sets.element_ref(&set), kind, &amplitude)])
            }
        };
        loads.push((index, content));
    }
    let mut rest = Content::default();
    match defined_fields(sets, step, amplitudes, model)? {
        Some(fields) => rest.fields = fields,
        None => takes_over.fields = true,
    }
    // Only the time integration computes the energies.
    let dynamic = matches!(step.kind, StepKind::Dynamic(_));
    // The outputs of the model the step lists are written into it as its own.
    for output in (step.history_outputs.iter())
        .filter_map(|&id| model.history_output(id))
        .filter(|o| o.active && (dynamic || o.kind != HistoryKind::Energies))
    {
        (rest.outputs).append(history_output(sets, output, pair_surfaces)?);
    }
    // The force of a bolt is the reaction of its pre-tension node, also while it is locked,
    // and its shortening the node's displacement: shown under the name of the load.
    for load in step.loads.iter().filter(|l| l.active) {
        if let Some(set) = sets.pre_tension_sets.get(&load.name).cloned() {
            let set = Some(ir::step::HistorySet::Nodes(sets.node_id(&set)));
            (rest.outputs.node_print).push(node_print(set, &["RF", "U"], Totals::No));
        }
    }
    rest.outputs.append(field_outputs(model, step, frame_every));
    Ok(StepParts {
        step: ir_step,
        takes_over,
        boundaries,
        loads,
        rest,
    })
}

/// The contact pairs a step switches off or on and whose friction it changes, with the
/// stick slope of the interaction. Changes of pairs not lowered, deactivated or unknown,
/// are left out.
fn contact_changes(
    sets: &mut Sets,
    step: &Step,
    model: &FeModel,
    pair_surfaces: &PairSurfaces,
) -> Vec<ir::step::ContactChange> {
    let mut changes = Vec::new();
    for change in &step.contact_changes {
        let Some((master, slave)) = pair_surfaces.get(&change.region) else {
            continue;
        };
        if let Some(add) = change.active {
            changes.push(ir::step::ContactChange::ContactPair {
                add,
                slave: sets.surface_id(slave),
                master: sets.surface_id(master),
            });
        }
        let pair = model.contact_pair(change.region.cast());
        let friction = pair
            .and_then(|p| model.surface_interaction(p.interaction?))
            .and_then(|i| {
                i.properties.iter().find_map(|p| match p {
                    InteractionProperty::Friction(f) => Some((i, f)),
                    _ => None,
                })
            });
        if let (Some(mu), Some((interaction, friction))) = (change.friction, friction) {
            let default = (model.materials.first())
                .and_then(|m| m.small_strain_elastic())
                .map(|e| e.young / 2.0);
            changes.push(ir::step::ContactChange::Friction {
                interaction: interaction_id(model, interaction),
                friction: ir::contact::Friction {
                    coefficient: mu,
                    stick_slope: friction.stick_slope.or(default),
                },
            });
        }
    }
    changes
}

/// The defined temperatures of a step: on
/// the node set with the value, or from the result file of a previous analysis and the step
/// to read; `None` for a step that keeps those before it. CalculiX 2.21 takes the file
/// name with its extension and looks for it next to the input file; `BStep` is the only
/// step parameter it knows.
fn defined_fields(
    sets: &mut Sets,
    step: &Step,
    amplitudes: &AmplitudeIds,
    model: &FeModel,
) -> Result<Option<Vec<ir::step::DefinedField>>, WriteError> {
    // A thermal step solves for the temperatures; it leaves the fields out like
    // deactivated ones.
    if !step.kind.supports_defined_fields() {
        return Ok(None);
    }
    // What a step shows is what acts: its own temperatures replace those of the steps
    // before, and a step that passes its state on without temperatures of its own sets
    // those a step before was given back to the initial ones (ramped like removed loads).
    let index = (model.steps.iter().position(|s| std::ptr::eq(s, step)))
        .or_else(|| model.steps.iter().position(|s| s.name == step.name));
    if let Some(index) = index
        && model.temperature_source(index, step).is_some()
    {
        return Ok(Some(vec![ir::step::DefinedField {
            kind: ir::step::DefinedFieldKind::Temperature(Vec::new()),
            amplitude: None,
        }]));
    }
    let mut fields = Vec::new();
    for field in step.defined_fields.iter().filter(|f| f.active) {
        let field = match &field.kind {
            DefinedFieldKind::Temperature(t) => {
                let amplitude = amplitude_parameter(amplitudes, &field.name, &field.amplitude)?;
                let values = match distribution(model, &field.name, &field.distribution)? {
                    Some(sampler) => {
                        let nodes = field.region.nodes(sets.mesh);
                        if nodes.is_empty() {
                            return Err(empty(&field.name, "nodes"));
                        }
                        let units = model.properties.units;
                        distributed_nodes(sets.mesh, &nodes, *t, &sampler, units)
                    }
                    None => {
                        let set = sets.node_set(&field.name, &field.region)?;
                        vec![(sets.node_ref(&set), *t)]
                    }
                };
                ir::step::DefinedField {
                    kind: ir::step::DefinedFieldKind::Temperature(values),
                    amplitude,
                }
            }
            DefinedFieldKind::TemperatureFromFile { file, step } => {
                let name = (file.path().and_then(|p| p.file_name())).map(|n| n.to_string_lossy());
                let Some(name) = name.filter(|n| !n.is_empty() && !n.contains(',')) else {
                    return Err(WriteError::InvalidResultFile {
                        item: field.name.clone(),
                        file: file.label(),
                    });
                };
                ir::step::DefinedField {
                    kind: ir::step::DefinedFieldKind::TemperatureFrom {
                        source: ir::handover::Source::Path(name.into_owned().into()),
                        bstep: Some(*step),
                        estep: None,
                        binc: None,
                        einc: None,
                    },
                    amplitude: None,
                }
            }
        };
        fields.push(field);
    }
    Ok((!fields.is_empty()).then_some(fields))
}

/// Equivalent nodal forces of a total force spread evenly over element faces, the way
/// a surface traction is turned into concentrated loads. Each face takes its share by
/// area; within a face the share follows the shape functions: equal parts on linear faces,
/// only midside nodes on quadratic triangles, and -1/12 per corner and 1/3 per midside
/// node on quadratic quadrilaterals.
///
/// The faces of 2D elements are their edges, which share the force by length. In an
/// axisymmetric model the force acts on the whole revolution, as CalculiX's concentrated
/// loads there do, so an edge's share grows with its distance from the axis.
fn traction_forces(
    mesh: &FeMesh,
    faces: &[(ElementId, u8)],
    force: [f64; 3],
    axisymmetric: bool,
) -> BTreeMap<NodeId, [f64; 3]> {
    let (weights, total_area) = constraints::node_areas(mesh, faces, axisymmetric, false);
    if total_area <= 0.0 {
        return BTreeMap::new();
    }
    (weights.into_iter())
        .map(|(node, weight)| (node, force.map(|f| f * weight / total_area)))
        .collect()
}

/// A total force spread over element edges by length, the edges given as their end nodes
/// and midside node: the consistent nodal forces of a constant line load.
fn edge_traction_forces(
    mesh: &FeMesh,
    edges: &[Vec<NodeId>],
    force: [f64; 3],
    axisymmetric: bool,
) -> BTreeMap<NodeId, [f64; 3]> {
    let mut weights: BTreeMap<NodeId, f64> = BTreeMap::new();
    let mut total = 0.0;
    for edge in edges {
        let Some(points) = edge
            .iter()
            .map(|&n| mesh.node(n))
            .collect::<Option<Vec<_>>>()
        else {
            continue;
        };
        for (&node, weight) in edge.iter().zip(edge_weights(&points, axisymmetric)) {
            *weights.entry(node).or_default() += weight;
            total += weight;
        }
    }
    if total <= 0.0 {
        return BTreeMap::new();
    }
    (weights.into_iter())
        .map(|(node, weight)| (node, force.map(|f| f * weight / total)))
        .collect()
}

/// A total heat flow spread over the volume of elements: each node takes its share of the
/// volume ([`plx_mesh::Element::node_volumes`]), so the flow per volume is the same
/// everywhere, a body flux. Kept in the first component, like the nodal forces of
/// [`traction_forces`].
fn volume_fluxes(
    mesh: &FeMesh,
    elements: &[ElementId],
    flux: f64,
    axisymmetric: bool,
) -> BTreeMap<NodeId, [f64; 3]> {
    let mut weights: BTreeMap<NodeId, f64> = BTreeMap::new();
    let mut total = 0.0;
    for element in elements.iter().filter_map(|&e| mesh.element(e)) {
        let Some(shares) = element.node_volumes(mesh, axisymmetric) else {
            continue;
        };
        for (&node, share) in element.nodes.iter().zip(shares) {
            *weights.entry(node).or_default() += share;
            total += share;
        }
    }
    if total <= 0.0 {
        return BTreeMap::new();
    }
    (weights.into_iter())
        .map(|(node, weight)| (node, [flux * weight / total, 0.0, 0.0]))
        .collect()
}

/// Integrals of the shape functions along an edge of two end points, or of two end points
/// and a midside point: the nodes' shares of the edge's length, or with `axisymmetric` of
/// the length times the radius x (the area the edge sweeps, divided by 2 pi).
fn edge_weights(points: &[[f64; 3]], axisymmetric: bool) -> Vec<f64> {
    // Three-point Gauss rule, exact for the quadratic edge with a linear radius.
    let gauss = [
        (-(0.6f64.sqrt()), 5.0 / 9.0),
        (0.0, 8.0 / 9.0),
        (0.6f64.sqrt(), 5.0 / 9.0),
    ];
    let mut weights = vec![0.0; points.len()];
    for (t, w) in gauss {
        let (shape, slope): (Vec<f64>, Vec<f64>) = if points.len() == 3 {
            (
                vec![t * (t - 1.0) / 2.0, t * (t + 1.0) / 2.0, 1.0 - t * t],
                vec![t - 0.5, t + 0.5, -2.0 * t],
            )
        } else {
            (vec![(1.0 - t) / 2.0, (1.0 + t) / 2.0], vec![-0.5, 0.5])
        };
        let combine =
            |f: &[f64], k: usize| -> f64 { f.iter().zip(points).map(|(n, p)| n * p[k]).sum() };
        let jacobian = (0..3)
            .map(|k| combine(&slope, k).powi(2))
            .sum::<f64>()
            .sqrt();
        let radius = if axisymmetric {
            combine(&shape, 0)
        } else {
            1.0
        };
        for (weight, n) in weights.iter_mut().zip(&shape) {
            *weight += w * n * jacobian * radius;
        }
    }
    weights
}

/// Area of a (possibly slightly warped) polygon from its vector area.
fn polygon_area(corners: &[[f64; 3]]) -> f64 {
    let mut sum = [0.0; 3];
    for (i, a) in corners.iter().enumerate() {
        let b = corners[(i + 1) % corners.len()];
        sum[0] += a[1] * b[2] - a[2] * b[1];
        sum[1] += a[2] * b[0] - a[0] * b[2];
        sum[2] += a[0] * b[1] - a[1] * b[0];
    }
    0.5 * (sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]).sqrt()
}

/// The equation solver a step asks for.
fn equation_solver(choice: plx_model::LinearSolverChoice) -> ir::step::SolverMode {
    use ir::step::SolverMode as S;
    match choice {
        plx_model::LinearSolverChoice::Automatic => S::Automatic,
        plx_model::LinearSolverChoice::Direct => S::Direct,
        plx_model::LinearSolverChoice::Iterative => S::Iterative,
    }
}

/// A step of `procedure` with the solver's defaults, named later.
fn new_step(procedure: ir::step::Procedure) -> ir::Step {
    ir::Step::new(String::new(), procedure)
}

/// A static step.
fn static_step(settings: &StaticStep, nlgeom: bool) -> ir::Step {
    let plan = settings.plan(nlgeom);
    incremented_step(
        &plan,
        ir::step::Procedure::Static(ir::step::Static {
            solver: equation_solver(settings.solver),
            increments: incrementation(&plan),
            time_reset: false,
            total_time_at_start: None,
        }),
        settings.nlgeom,
    )
}

/// A visco step: a static step whose automatic increments the creep strain tolerance
/// limits.
fn visco_step(settings: &ViscoStep, nlgeom: bool) -> ir::Step {
    let increments = &settings.increments;
    let plan = increments.plan(nlgeom);
    incremented_step(
        &plan,
        ir::step::Procedure::Visco(ir::step::Visco {
            solver: equation_solver(increments.solver),
            increments: incrementation(&plan),
            cetol: settings.creep_tolerance,
        }),
        increments.nlgeom,
    )
}

/// A sensitivity step: one design response per response, named like it, on all elements of
/// the model, then the optional filter.
fn sensitivity_step(
    sets: &mut Sets,
    step: &str,
    settings: &SensitivityStep,
) -> Result<ir::Step, WriteError> {
    let responses = settings.responses();
    let elements = if (responses.iter()).any(|r| *r != ir::step::DesignResponseKind::Eigenfrequency)
    {
        let parts = sets.mesh.part_ids();
        let set = sets.element_set(step, &Region::Parts(parts))?;
        Some(sets.element_id(&set))
    } else {
        None
    };
    let responses = (responses.iter())
        .map(|&response| ir::step::DesignResponse {
            kind: response,
            elset: elements,
        })
        .collect();
    Ok(new_step(ir::step::Procedure::Sensitivity(
        ir::step::Sensitivity {
            nlgeom: false,
            responses,
            filter: (settings.filter_radius).map(|radius| ir::step::Filter { radius }),
        },
    )))
}

/// The design variables of the model: the design region of the first active sensitivity
/// step. The solver reads them before the steps.
fn design_variables(
    sets: &mut Sets,
    model: &FeModel,
) -> Result<Option<ir::model::DesignVariables>, WriteError> {
    let Some((name, region)) = (model.steps.iter())
        .filter(|s| s.active)
        .find_map(|s| Some((&s.name, s.kind.design_region()?)))
    else {
        return Ok(None);
    };
    let set = sets.node_set(&format!("{name}_Design"), region)?;
    Ok(Some(ir::model::DesignVariables {
        nset: sets.node_id(&set),
    }))
}

#[derive(Clone, Copy)]
enum HeatKind {
    Heat,
    Coupled,
    Uncoupled,
}

/// A heat transfer or coupled step.
fn heat_transfer_step(settings: &HeatTransferStep, kind: HeatKind, nlgeom: bool) -> ir::Step {
    let increments = &settings.increments;
    let plan = increments.plan(nlgeom);
    let heat = ir::step::HeatTransfer {
        solver: equation_solver(increments.solver),
        steady_state: settings.steady_state,
        increments: incrementation(&plan),
        deltmx: settings.deltmx.filter(|_| !settings.steady_state),
        frequency_count: None,
        direct: plan.fixed,
    };
    let (procedure, nlgeom) = match kind {
        HeatKind::Heat => (ir::step::Procedure::HeatTransfer(heat), false),
        HeatKind::Coupled => (
            ir::step::Procedure::CoupledTemperatureDisplacement(heat),
            increments.nlgeom,
        ),
        HeatKind::Uncoupled => (
            ir::step::Procedure::UncoupledTemperatureDisplacement(heat),
            false,
        ),
    };
    incremented_step(&plan, procedure, nlgeom)
}

/// A Dynamic step: fixed time steps that put the result frames exactly on time steps
/// (every `per_frame`-th increment is written, see [`DynamicStep::integration`]), the
/// numerical damping as the HHT alpha and the damping as Rayleigh coefficients. Its loads
/// act at once; an amplitude shapes them over time. Returns the time steps per frame.
fn dynamic_step(settings: &DynamicStep, model: &FeModel, step: &Step) -> (ir::Step, u32) {
    let amplitude_interval = model.amplitude_interval(step, settings.duration);
    let integration = settings.integration(amplitude_interval);
    let (rayleigh, damping_ratio) = match settings.rayleigh(integration.increment) {
        None => (None, None),
        Some(RayleighFit::Coefficients(r)) => (
            Some(ir::step::Rayleigh {
                alpha: r.alpha,
                beta: r.beta,
            }),
            None,
        ),
        // The solver fits it from the lowest eigenfrequency, which it computes first.
        Some(RayleighFit::FromRatio { zeta, upper }) => (
            None,
            Some(ir::step::DampingRatio {
                ratio: zeta,
                lower: None,
                upper,
            }),
        ),
    };
    if let DynamicMethod::Modal { .. } = settings.method {
        // Every mode is integrated exactly; the time step only decides where the loads are
        // sampled and the results written, so the direct step's time step serves.
        let damping = match settings.damping {
            Damping::None => None,
            Damping::Ratio { zeta, .. } => {
                Some(ir::step::ModalDamping::Direct(vec![(1, 1_000_000, zeta)]))
            }
            Damping::Rayleigh(r) => Some(ir::step::ModalDamping::Rayleigh {
                alpha: r.alpha,
                beta: r.beta,
            }),
        };
        let mut ir_step = new_step(ir::step::Procedure::ModalDynamics(
            ir::step::ModalDynamics {
                solver: ir::step::SolverMode::Automatic,
                steady_state: false,
                increment: integration.increment,
                period: settings.duration,
                damping,
            },
        ));
        ir_step.max_increments = (integration.increments(settings) + 1).min(u32::MAX as u64) as u32;
        ir_step.default_amplitude = ir::step::DefaultAmplitude::Step;
        return (ir_step, integration.per_frame);
    }
    if let DynamicMethod::Explicit = settings.method {
        // The solver finds the time step from the mesh; the increments are the frames.
        let mut ir_step = new_step(ir::step::Procedure::Dynamic(ir::step::Dynamic {
            solver: ir::step::SolverMode::Automatic,
            alpha: None,
            integration: ir::step::Integration::Explicit(ir::step::ExplicitControls {
                mass_scaling: settings.mass_scaling,
                time_step: match settings.time_step {
                    plx_model::TimeStep::Fixed(dt) if dt > 0.0 => Some(dt),
                    _ => None,
                },
                full_shell_inertia: settings.full_shell_inertia,
            }),
            increments: ir::step::Incrementation {
                initial: Some(settings.frame_interval()),
                period: Some(settings.duration),
                direct: true,
                ..ir::step::Incrementation::default()
            },
            rayleigh,
            damping_ratio,
            event_frames: settings.event_frames,
        }));
        ir_step.max_increments = settings.frames.max(1) + 1;
        ir_step.default_amplitude = ir::step::DefaultAmplitude::Step;
        ir_step.nlgeom = settings.large_deformations;
        return (ir_step, 1);
    }
    let alpha = settings.numerical_damping.alpha();
    let mut ir_step = new_step(ir::step::Procedure::Dynamic(ir::step::Dynamic {
        solver: equation_solver(settings.solver),
        alpha: (alpha != -0.05).then_some(alpha),
        integration: match settings.method {
            // Implicit on the time steps of an implicit step, explicit where the solver
            // switches.
            DynamicMethod::Automatic => ir::step::Integration::Automatic {
                explicit: ir::step::ExplicitControls {
                    mass_scaling: settings.mass_scaling,
                    time_step: None,
                    full_shell_inertia: settings.full_shell_inertia,
                },
                switch_factor: settings
                    .switch_factor
                    .unwrap_or(plx_model::DynamicStep::DEFAULT_SWITCH_FACTOR),
            },
            _ => ir::step::Integration::Implicit,
        },
        increments: ir::step::Incrementation {
            initial: Some(integration.increment),
            period: Some(settings.duration),
            direct: true,
            ..ir::step::Incrementation::default()
        },
        rayleigh,
        damping_ratio,
        event_frames: settings.event_frames,
    }));
    ir_step.max_increments = (integration.increments(settings) + 1).min(u32::MAX as u64) as u32;
    ir_step.default_amplitude = ir::step::DefaultAmplitude::Step;
    ir_step.nlgeom = settings.large_deformations;
    (ir_step, integration.per_frame)
}

/// A steady state dynamics step.
fn steady_state_dynamics_step(settings: &SteadyStateDynamicsStep) -> ir::Step {
    new_step(ir::step::Procedure::SteadyStateDynamics(
        ir::step::SteadyStateDynamics {
            solver: ir::step::SolverMode::Automatic,
            harmonic: settings.harmonic,
            lower: settings.lower_frequency,
            upper: settings.upper_frequency,
            points: settings.data_points,
            bias: settings.bias,
            fourier_terms: (!settings.harmonic).then_some(settings.fourier_terms),
            damping: modal_damping(settings.damping.as_ref()),
        },
    ))
}

/// The modal damping: a constant ratio covers
/// modes 1 to 1000000.
fn modal_damping(damping: Option<&ModalDamping>) -> Option<ir::step::ModalDamping> {
    Some(match damping? {
        ModalDamping::Constant(ratio) => {
            ir::step::ModalDamping::Direct(vec![(1, 1_000_000, *ratio)])
        }
        ModalDamping::Direct(ranges) => ir::step::ModalDamping::Direct(
            (ranges.iter())
                .map(|r| (r.lowest, r.highest, r.ratio))
                .collect(),
        ),
        ModalDamping::Rayleigh(r) => ir::step::ModalDamping::Rayleigh {
            alpha: r.alpha,
            beta: r.beta,
        },
    })
}

/// The increments of a step with a duration: every value resolved, the result frames as
/// `FRAMES=`.
fn incrementation(plan: &IncrementPlan) -> ir::step::Incrementation {
    ir::step::Incrementation {
        initial: Some(plan.initial),
        period: Some(plan.duration),
        minimum: (!plan.fixed).then_some(plan.minimum),
        maximum: (!plan.fixed).then_some(plan.maximum),
        direct: plan.fixed,
        frames: Some(plan.frames),
    }
}

/// A step of `procedure` with the increments of `plan`: without a limit on their number
/// the step takes as many as it needs.
fn incremented_step(
    plan: &IncrementPlan,
    procedure: ir::step::Procedure,
    nlgeom: bool,
) -> ir::Step {
    let mut step = new_step(procedure);
    step.nlgeom = nlgeom;
    step.max_increments = plan.limit.unwrap_or(u32::MAX);
    step
}

/// A frequency step: the lower bound is 0 when only an upper bound
/// is given. In a cyclic symmetric model the solver finds every mode twice, as a cosine and a
/// sine part, so it is asked for twice the number of eigenfrequencies per nodal diameter,
/// in the range of nodal diameters if one is set.
fn frequency_step(settings: &FrequencyStep, cyclic: bool) -> ir::Step {
    let count = settings.num_frequencies * if cyclic { 2 } else { 1 };
    let (lower, upper) = match (settings.lower_frequency, settings.upper_frequency) {
        (None, None) => (None, None),
        (lower, upper) => (Some(lower.unwrap_or(0.0)), upper),
    };
    let mut step = new_step(ir::step::Procedure::Frequency(ir::step::Frequency {
        solver: equation_solver(settings.solver),
        storage: settings.storage,
        global: true,
        cycmpc_active: true,
        count,
        lower,
        upper,
        cyclic_modes: cyclic.then(|| ir::step::CyclicModes {
            nmin: settings.nodal_diameters.map(|[min, _]| min),
            nmax: settings.nodal_diameters.map(|[_, max]| max),
        }),
    }));
    step.perturbation = settings.base.is_preloaded();
    step
}

/// A complex frequency step.
/// `perturbation` is the flag of the frequency step that stored the modes: CalculiX
/// requires the same on both steps.
fn complex_frequency_step(settings: &ComplexFrequencyStep) -> ir::Step {
    let mut step = new_step(ir::step::Procedure::ComplexFrequency(
        ir::step::ComplexFrequency {
            coriolis: settings.coriolis,
            fluid: false,
            count: settings.num_frequencies,
        },
    ));
    step.perturbation = settings.stiffening || settings.base.is_preloaded();
    step
}

/// A buckle step.
fn buckle_step(settings: &BuckleStep) -> ir::Step {
    let mut step = new_step(ir::step::Procedure::Buckle(ir::step::Buckle {
        solver: equation_solver(settings.solver),
        count: settings.num_factors,
        accuracy: Some(settings.accuracy),
    }));
    step.perturbation = settings.base.is_preloaded();
    step
}

/// The variables of an output request the solver knows; it would print nothing for others.
fn variables<'a>(variables: impl IntoIterator<Item = &'a str>) -> Vec<OutputVariable> {
    (variables.into_iter())
        .filter_map(OutputVariable::from_code)
        .collect()
}

/// The solver's variables of a result of a step: of the nodes, of the elements and of the
/// contacts.
fn field_variables(result: FieldResult) -> [&'static [OutputVariable]; 3] {
    use OutputVariable as V;
    match result {
        FieldResult::Displacements => [&[V::Displacements], &[], &[]],
        FieldResult::ReactionForces => [&[V::ReactionForces], &[], &[]],
        FieldResult::Velocities => [&[V::Velocities], &[], &[]],
        FieldResult::Accelerations => [&[V::Accelerations], &[], &[]],
        FieldResult::Temperatures => [&[V::Temperatures], &[], &[]],
        FieldResult::ReactionHeatFlows => [&[V::ReactionHeatFlows], &[], &[]],
        FieldResult::Sensitivities => [&[V::Sensitivities], &[], &[]],
        FieldResult::Stresses => [&[], &[V::Stresses], &[]],
        FieldResult::Strains => [&[], &[V::Strains], &[]],
        FieldResult::PlasticStrain => [&[], &[V::EquivalentPlasticStrain], &[]],
        FieldResult::StrainEnergy => [&[], &[V::StrainEnergyDensity], &[]],
        FieldResult::HeatFlux => [&[], &[V::HeatFlux], &[]],
        FieldResult::StressError => [&[], &[V::StressError], &[]],
        FieldResult::Contact => [&[], &[], &[V::ContactDisplacements, V::ContactStresses]],
        FieldResult::ContactEnergy => [&[], &[], &[V::ContactEnergy]],
        FieldResult::AmplitudePhase => [&[V::DisplacementPhase], &[V::StressPhase], &[]],
    }
}

/// The field output requests of a step: what the step writes by its kind and by the model,
/// and the additional results of the model's field output, every `every`-th increment.
/// The solver takes the variables of the nodes, the elements and the contacts apart.
fn field_outputs(model: &FeModel, step: &Step, every: u32) -> ir::step::Outputs {
    use OutputVariable as V;
    // The order the solver's variables are listed in.
    const ORDER: [&[OutputVariable]; 3] = [
        &[
            V::ReactionForces,
            V::Displacements,
            V::DisplacementPhase,
            V::Velocities,
            V::Accelerations,
            V::Temperatures,
            V::ReactionHeatFlows,
            V::Sensitivities,
        ],
        &[
            V::Stresses,
            V::StressPhase,
            V::Strains,
            V::EquivalentPlasticStrain,
            V::StrainEnergyDensity,
            V::HeatFlux,
            V::StressError,
        ],
        &[
            V::ContactDisplacements,
            V::ContactStresses,
            V::ContactEnergy,
        ],
    ];
    let results = model.field_results(step);
    let mut names: [Vec<OutputVariable>; 3] = Default::default();
    for (kind, order) in ORDER.iter().enumerate() {
        names[kind] = (order.iter().copied())
            .filter(|v| (results.iter()).any(|&r| field_variables(r)[kind].contains(v)))
            .collect();
    }
    // The error estimate of the stresses is written with them unless it is switched off.
    if results.contains(&FieldResult::Stresses) && !results.contains(&FieldResult::StressError) {
        names[1].push(V::NoStressError);
    }
    let [node, element, contact] = names;
    let request = |variables: Vec<OutputVariable>, contact: bool| ir::step::FieldOutput {
        variables,
        frequency: every,
        nset: None,
        last_iterations: false,
        contact_elements: false,
        // By default shells and beams are written as the solids they are expanded to; at
        // the shell nodes (2D) the stresses are those of the mid-surface, without bending.
        output: match model.field_output.dimension {
            _ if contact => ir::step::OutputDimension::Expanded,
            OutputDimension::Default => ir::step::OutputDimension::Expanded,
            OutputDimension::TwoD => ir::step::OutputDimension::TwoD,
            OutputDimension::ThreeD => ir::step::OutputDimension::ThreeD,
        },
        // The field output's coordinate system is applied to the results when they are
        // shown, so that it may move; the solver writes global results.
        global: true,
        section_forces: false,
        time_points: None,
    };
    let mut outputs = ir::step::Outputs::default();
    if !node.is_empty() {
        outputs.node_file.push(request(node, false));
    }
    if !element.is_empty() {
        outputs.el_file.push(request(element, false));
    }
    if !contact.is_empty() {
        outputs.contact_file.push(request(contact, true));
    }
    outputs
}

/// The totals of a history output.
fn totals(totals: Totals) -> ir::step::Totals {
    match totals {
        Totals::No => ir::step::Totals::No,
        Totals::Yes => ir::step::Totals::Yes,
        Totals::Only => ir::step::Totals::Only,
    }
}

/// A nodal history output of a node set.
fn node_print(
    set: Option<ir::step::HistorySet>,
    names: &[&str],
    totals_of: Totals,
) -> ir::step::HistoryOutput {
    ir::step::HistoryOutput {
        origin: None,
        set,
        variables: variables(names.iter().copied()),
        frequency: 1,
        totals: totals(totals_of),
        global: true,
        time_points: None,
        surfaces: None,
        rotations: false,
    }
}

/// The solver's name of a variable of a history output.
fn history_variable(variable: HistoryVariable) -> &'static str {
    use HistoryVariable as V;
    match variable {
        V::ReactionForces => "RF",
        V::Displacements => "U",
        V::Velocities => "V",
        V::Temperatures => "NT",
        V::ReactionHeatFlows => "RFL",
        V::Stresses => "S",
        V::Strains => "E",
        V::MechanicalStrains => "ME",
        V::PlasticStrain => "PEEQ",
        V::HeatFlux => "HFL",
        V::EnergyDensity => "ENER",
        V::ElementInternalEnergy => "ELSE",
        V::ElementKineticEnergy => "ELKE",
        V::ElementVolume => "EVOL",
        V::ElementHeatGeneration => "EBHE",
        V::ContactDisplacements => "CDIS",
        V::ContactStresses => "CSTR",
        V::ContactEnergy => "CELS",
        V::ContactElements => "CNUM",
        V::ContactForces => "CF",
        V::InternalEnergy => "ALLIE",
        V::KineticEnergy => "ALLKE",
        V::ExternalWork => "ALLWK",
        V::DampingDissipation => "ALLVD",
    }
}

/// The history output requests of a history output, which carry its id to the history they
/// print; none without variables.
fn history_output(
    sets: &mut Sets,
    output: &HistoryOutput,
    pair_surfaces: &PairSurfaces,
) -> Result<ir::step::Outputs, WriteError> {
    let mut outputs = history_requests(sets, output, pair_surfaces)?;
    for request in (outputs.node_print.iter_mut())
        .chain(&mut outputs.el_print)
        .chain(&mut outputs.contact_print)
    {
        request.origin = Some(output.id.raw());
    }
    Ok(outputs)
}

fn history_requests(
    sets: &mut Sets,
    output: &HistoryOutput,
    pair_surfaces: &PairSurfaces,
) -> Result<ir::step::Outputs, WriteError> {
    if output.variables.is_empty() {
        return Ok(ir::step::Outputs::default());
    }
    let names: Vec<&str> = output
        .variables
        .iter()
        .map(|&v| history_variable(v))
        .collect();
    let mut outputs = ir::step::Outputs::default();
    match &output.kind {
        HistoryKind::Node {
            region: region @ (Region::ReferencePoint(_) | Region::RemotePoint(_)),
        } => {
            return point_output(sets, output, region);
        }
        HistoryKind::Node { region } => {
            let set = sets.node_set(&output.name, region)?;
            let set = Some(ir::step::HistorySet::Nodes(sets.node_id(&set)));
            (outputs.node_print).push(node_print(set, &names, output.totals));
        }
        HistoryKind::Energies => {
            // Sums over the whole model, printed under a set of all elements.
            let parts = sets.mesh.part_ids();
            let set = sets.element_set(&output.name, &Region::Parts(parts))?;
            let set = Some(ir::step::HistorySet::Elements(sets.element_id(&set)));
            (outputs.el_print).push(node_print(set, &names, Totals::Only));
        }
        HistoryKind::Element { region } => {
            let set = sets.element_set(&output.name, region)?;
            let set = Some(ir::step::HistorySet::Elements(sets.element_id(&set)));
            (outputs.el_print).push(node_print(set, &names, output.totals));
        }
        HistoryKind::Contact { pair } => {
            // Contact forces are summed over the slave surface of the pair.
            let mut request = node_print(None, &names, output.totals);
            if output.variables.contains(&HistoryVariable::ContactForces) {
                let (master, slave) =
                    (pair.and_then(|p| pair_surfaces.get(&p))).ok_or_else(|| {
                        WriteError::UnknownContactPair {
                            item: output.name.clone(),
                            pair: pair.map_or_else(|| "(none)".into(), |p| p.to_string()),
                        }
                    })?;
                let master = (!master.is_empty()).then(|| sets.surface_id(master));
                request.surfaces = Some((sets.surface_id(slave), master));
            }
            outputs.contact_print.push(request);
        }
    }
    Ok(outputs)
}

/// Holds the reference points fixed to the world in a mechanical step: every degree of
/// freedom at zero that no displacement of the step on the point moves, the rotations as
/// the displacements of the rotation node.
fn grounded_boundaries(sets: &Sets, step: &Step, space: ModelSpace) -> Option<Content> {
    let mut items = Vec::new();
    for id in &sets.grounded {
        let nodes = &sets.reference_nodes[id];
        let mut moved = [false; 6];
        for bc in &step.boundary_conditions {
            if let BoundaryKind::Displacement(displacement) = &bc.kind
                && bc.active
                && step.kind.supports_boundary(&bc.kind)
                && bc.region.reference_point() == Some(*id)
            {
                for (moved, value) in moved.iter_mut().zip(displacement.values) {
                    *moved |= value.is_some();
                }
            }
        }
        let held: [Option<f64>; 6] =
            std::array::from_fn(|d| (space.remote_dofs().contains(&d) && !moved[d]).then_some(0.0));
        items.extend(displacement_boundaries(
            &NodeRef::Node(nodes.reference),
            Some(&NodeRef::Node(nodes.rotation)),
            &held,
            6,
            &[const { None }; 6],
        ));
    }
    (!items.is_empty()).then_some(Content::boundaries(items))
}

/// Reference and rotation node of a reference point or the point of a remote displacement,
/// and how the region follows the point.
fn point_nodes(
    sets: &Sets,
    item: &str,
    region: &Region,
) -> Result<([NodeId; 2], RemoteBehavior), WriteError> {
    match region {
        Region::ReferencePoint(point) => match sets.reference_nodes.get(point) {
            Some(nodes) => Ok(([nodes.reference, nodes.rotation], nodes.behavior)),
            None => Err(WriteError::UnknownReferencePoint {
                item: item.to_string(),
                point: point.to_string(),
            }),
        },
        Region::RemotePoint(point) => match sets.remote_names.get(point) {
            Some(&nodes) => Ok(nodes),
            None => Err(WriteError::UnknownRemotePoint {
                item: item.to_string(),
                point: point.to_string(),
            }),
        },
        _ => unreachable!("not a point"),
    }
}

/// The nodal history output of a point: the values of its reference node, and the
/// rotations, moments and angular velocities as the displacements, forces and velocities
/// of its rotation node. The rotation node's set is named `Internal_Rotation-n_<output>`,
/// by which the history reader shows its values under the output's name.
///
/// The solver has reaction forces at the point of a rigid body only: the point of equations,
/// a deformable or coupled remote displacement, prints zeros, so its `RF` is left out. A
/// coupled point does not turn.
fn point_output(
    sets: &mut Sets,
    output: &HistoryOutput,
    region: &Region,
) -> Result<ir::step::Outputs, WriteError> {
    let ([reference, rotation], behavior) = point_nodes(sets, &output.name, region)?;
    let reactions = behavior == RemoteBehavior::Rigid;
    let printed: Vec<&str> = (output.variables.iter())
        .filter(|&&v| reactions || v != HistoryVariable::ReactionForces)
        .map(|&v| history_variable(v))
        .collect();
    if printed.is_empty() {
        return Ok(ir::step::Outputs::default());
    }
    let set = sets.free_name("Internal_Selection", &name(&output.name));
    sets.node_sets.push((set.clone(), vec![reference]));
    let printed_set = Some(ir::step::HistorySet::Nodes(sets.node_id(&set)));
    let mut outputs = ir::step::Outputs::default();
    (outputs.node_print).push(node_print(printed_set, &printed, output.totals));
    let turning: Vec<&str> = (printed.iter().copied())
        .filter(|v| ["RF", "U", "V"].contains(v))
        .collect();
    if !turning.is_empty() {
        let set = sets.free_name("Internal_Rotation", &name(&output.name));
        sets.node_sets.push((set.clone(), vec![rotation]));
        let set = Some(ir::step::HistorySet::Nodes(sets.node_id(&set)));
        let mut rotations = node_print(set, &turning, output.totals);
        rotations.rotations = true;
        outputs.node_print.push(rotations);
    }
    Ok(outputs)
}

/// Length and direction of a vector; `None` for the zero vector.
fn unit_vector(v: [f64; 3]) -> Option<(f64, [f64; 3])> {
    let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    (length > 0.0).then(|| (length, v.map(|c| c / length)))
}

/// A real number in at most 16 characters, CalculiX's field width.
fn number(value: f64) -> String {
    let short = value.to_string();
    if short.len() <= 16 {
        short
    } else {
        format!("{value:.8E}")
    }
}

/// Item names as CalculiX accepts them: no spaces, commas or other special characters.
fn name(item: &str) -> String {
    item.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

fn empty(item: &str, what: &'static str) -> WriteError {
    WriteError::EmptyRegion {
        item: item.to_owned(),
        what,
    }
}

mod constraints;
mod dofs;
mod infill;
mod node_frames;
mod pre_tension;

pub use dofs::resolved_overlaps;
#[cfg(test)]
mod tests;
