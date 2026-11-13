//! Lowering of a [`FeModel`] with its mesh into the solver's input, a
//! [`plxs_core::Analysis`], in the layout of a CalculiX input file.
//!
//! Regions the user picked in the GUI become node sets, element sets and surfaces named
//! as internal selections (`Internal_Selection-1_Fixed-1`), so the user never
//! defines sets by hand. The steps are assembled with CalculiX's carry-over between them
//! resolved ([`plxs_input::steps`]), as the solver reads an input file. An input file is
//! the analysis rendered as text ([`plxs_input::render`]), for the export only.

use std::collections::{BTreeMap, BTreeSet};

use plx_mesh::{ElementFamily, ElementId, FeMesh, NodeId, SurfaceDefinition};
use plx_model::{
    Amplitude, AmplitudeTime, BeamProfile, BeamSection, BoundaryKind, BuckleStep,
    ComplexFrequencyStep, Constraint, ContactMethod, ContactPair, ContactScale, Damping,
    DefinedFieldKind, DynamicMethod, DynamicStep, FeModel, FieldOutput, Formulation, FrequencyStep,
    GapConductance, HeatTransferStep, HistoryKind, HistoryOutput, Id, IncrementPlan,
    InitialConditionKind, InteractionProperty, LoadKind, ModalDamping, ModelItem, ModelSpace,
    MovingFrame, NodeTie, Origins, OutputDimension, OutputKind, Quantity, RayleighFit, Region,
    RemoteBehavior, RemoteDof, Sampler, Section, SectionKind, SectorDisplay, SensitivityStep,
    StaticStep, SteadyStateDynamicsStep, Step, StepKind, SurfaceBehavior, SurfaceInteraction,
    TableOwner, Tie, Totals, UnitSystem, ViscoStep, line_tangent, symmetry_dofs, symmetry_plane,
};
use plxs_core as ir;
use plxs_core::mesh::{ElementRef, NodeRef};
use plxs_core::normalized_name;
use plxs_input::steps::{Assembler, Card};

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
    /// Set names that were shortened, each with the full name it stands for.
    pub shortened_names: Vec<(String, String)>,
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
    /// The input file of the analysis: the comments naming the shortened names and the
    /// resolved overlaps, then the analysis rendered as text.
    pub fn text(&self) -> String {
        let mut out = String::new();
        for (short, full) in &self.shortened_names {
            out.push_str(&format!("{SHORTENED_NAME}{short} = {full}\n"));
        }
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

/// Lowers the mesh and the analysis of `model` into the solver's input, validated.
pub fn lower(mesh: &FeMesh, model: &FeModel, heading: &str) -> Result<Lowered, LowerError> {
    validated(lower_unvalidated(mesh, model, heading)?)
}

/// [`lower`] with the sets of `extra` steps too, see [`write_inp_with_sets_of`].
pub fn lower_with_sets_of(
    mesh: &FeMesh,
    model: &FeModel,
    heading: &str,
    extra: &[(usize, Vec<Step>)],
) -> Result<Lowered, LowerError> {
    validated(lower_with_sets_unvalidated(mesh, model, heading, extra)?)
}

fn validated(mut lowered: Lowered) -> Result<Lowered, LowerError> {
    let warnings = ir::validate(&mut lowered.analysis).map_err(LowerError::Solver)?;
    lowered.warnings.extend(warnings);
    Ok(lowered)
}

/// The analysis before the solver's validation, for a run that validates it itself and
/// reports what it finds as its diagnostics.
pub fn lower_unvalidated(
    mesh: &FeMesh,
    model: &FeModel,
    heading: &str,
) -> Result<Lowered, ModelError> {
    Ok(lower_model(mesh, model, heading)?.assemble(|_| true))
}

/// [`lower_with_sets_of`] before the solver's validation, see [`lower_unvalidated`].
pub fn lower_with_sets_unvalidated(
    mesh: &FeMesh,
    model: &FeModel,
    heading: &str,
    extra: &[(usize, Vec<Step>)],
) -> Result<Lowered, ModelError> {
    if extra.iter().all(|(_, steps)| steps.is_empty()) {
        return lower_unvalidated(mesh, model, heading);
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
        return lower_unvalidated(mesh, model, heading);
    };
    Ok(built.assemble(|s| written.get(s).copied().unwrap_or(true)))
}

/// Writes the mesh and the analysis as the text of an input file.
pub fn write_inp(mesh: &FeMesh, model: &FeModel, heading: &str) -> Result<String, WriteError> {
    Ok(lower_unvalidated(mesh, model, heading)
        .map_err(|e| e.error)?
        .text())
}

/// Writes the input file of `model` with the sets of `extra` steps too, which are not
/// written themselves: the steps of analyses that continue from the state after a step of
/// this one. The solver takes no new sets after reading a state, so the run that writes it has
/// to define them. Each entry holds the index of the step of `model` the steps follow; they
/// take the place in the order of the sets they have in the input file of the analysis that
/// continues, so that the sets are named as there. When the extra steps cannot be written,
/// the file is written without their sets.
pub fn write_inp_with_sets_of(
    mesh: &FeMesh,
    model: &FeModel,
    heading: &str,
    extra: &[(usize, Vec<Step>)],
) -> Result<String, WriteError> {
    Ok(lower_with_sets_unvalidated(mesh, model, heading, extra)
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
    write_inp(&deformed, &FeModel::default(), heading)
}

/// One `*STEP` of the analysis before the carry-over between the steps: its settings and
/// its cards in the order of the input file.
#[derive(Clone, Debug)]
struct StepBlock {
    /// The settings; boundaries, loads, fields and outputs come from the cards.
    step: ir::Step,
    /// Whether the step keeps its name; the blocks of a step with moving loads are named
    /// by their number, `Step-n`, as a step without a name.
    named: bool,
    /// The cards of its boundary conditions, then those of everything after them.
    boundaries: Vec<Card>,
    cards: Vec<Card>,
    /// A step that only prepares the next one, such as the preload and the eigenmodes
    /// of a complex frequency step; its results are not kept.
    preparation: bool,
}

/// Everything of the analysis but the assembly of the steps.
struct Built {
    model: ir::Model,
    /// The cards before the first step: the boundary conditions of a first step that works
    /// on stored modes (see [`boundaries_before_stored_modes`]).
    model_cards: Vec<Card>,
    /// The blocks of each step of the model; none for a deactivated step.
    steps: Vec<Vec<StepBlock>>,
    resolved_overlaps: Vec<String>,
    shortened_names: Vec<(String, String)>,
    part_sets: Vec<String>,
    profile_comments: Vec<(String, String)>,
    /// The origins of everything but the steps, which [`Built::assemble`] adds.
    origins: Origins,
}

impl Built {
    /// The analysis with the steps `written` takes, by their index in the model.
    fn assemble(self, written: impl Fn(usize) -> bool) -> Lowered {
        let model = self.model;
        let mut assembler = Assembler::new();
        for card in self.model_cards {
            // Boundaries outside of a step belong to the first one.
            let _ = assembler.apply(&model, card);
        }
        let mut steps = Vec::new();
        let mut origins = self.origins;
        let mut preparation_steps = Vec::new();
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
                assembler.begin(step);
                // A step prints the history outputs it lists, none carried over from the
                // steps before: the model keeps one output that each step lists or not.
                let prints = |kind: fn(&Card) -> bool| block.cards.iter().any(kind);
                let node = prints(|c| matches!(c, Card::NodePrint(_)));
                let element = prints(|c| matches!(c, Card::ElPrint(_)));
                let contact = prints(|c| matches!(c, Card::ContactPrint(_)));
                for card in block.boundaries.into_iter().chain(block.cards) {
                    let _ = assembler.apply(&model, card);
                }
                let first = steps.len();
                steps.extend(assembler.end(&model).map(|mut step| {
                    let outputs = &mut step.outputs;
                    for (own, list) in [
                        (node, &mut outputs.node_print),
                        (element, &mut outputs.el_print),
                        (contact, &mut outputs.contact_print),
                    ] {
                        if !own {
                            list.clear();
                        }
                    }
                    step
                }));
                for step in first..steps.len() {
                    origins.insert(ir::ItemRef::Step(step), ModelItem::Step(origin));
                    if block.preparation {
                        preparation_steps.push(step);
                    }
                }
            }
        }
        Lowered {
            warnings: assembler.take_warnings(),
            analysis: ir::Analysis { model, steps },
            resolved_overlaps: self.resolved_overlaps,
            shortened_names: self.shortened_names,
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

fn node_set_ref(set: &str) -> NodeRef {
    NodeRef::Set(ir_name(set))
}

fn element_set_ref(set: &str) -> ElementRef {
    ElementRef::Set(ir_name(set))
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
    // Regions are resolved first, in the order of the input file that the names of their
    // sets follow. The constraints come in their order of precedence for the dependent
    // degrees of freedom (see [`dofs`]): rigid bodies and remote points, pre-tension cuts,
    // node ties, last springs and contact regions.
    let mut materials = materials(model);
    let mut orientations = Vec::new();
    let (mut sections, profile_comments, section_owners) =
        sections(&mut sets, model, &mut materials, &mut orientations)?;
    let mut generated = constraints::Generated::default();
    let (mut constraints, mut constraint_owners) =
        constraints(&mut sets, model, &mut generated, lowering)?;
    let pre_tension_equations = pre_tension_sections(&mut sets, model, &cuts, &mut generated);
    let node_tie_equations = node_ties(&mut sets, model, &lines)?;
    constraints::springs(&mut sets, model, &mut generated)?;
    constraints.extend(
        (generated.equations.iter()).map(|terms| ConstraintItem::Equation(equation(terms))),
    );
    sections.extend(generated.sections.iter().map(generated_section));
    materials.extend(generated.material.as_deref().map(gap_material));
    // The boundary condition holding the ground nodes of the gaps, in every mechanical step.
    let extra_boundary = (generated.boundary.as_ref()).map(|(set, dofs)| Card::Boundary {
        op_new: false,
        items: (1..=*dofs as u8)
            .map(|dof| boundary(node_set_ref(set), dof, dof, Some(0.0), None))
            .collect(),
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
    let amplitudes = amplitudes(model)?;
    let flux_kinds = FluxKinds::of(model);
    let mut steps = (model.steps.iter().enumerate())
        .map(|(s, step)| {
            let context = StepContext {
                cyclic: (model.ties.iter()).any(|t| t.active && t.cyclic.is_some()),
                cyclic_slaves: &cyclic_slaves,
                extra_boundary: extra_boundary.as_ref(),
                space,
                lines: &lines,
                flux_kinds,
                amplitudes: &model.amplitudes,
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
    ir_model.constraints.submodel = submodel(model, &sets.submodel_sets)?;
    for (id, &coords) in mesh.node_ids().iter().zip(mesh.coords()) {
        ir_model.nodes.insert(*id, coords);
    }
    for &(id, coords) in &generated.nodes {
        ir_model.nodes.insert(id, coords);
    }
    elements(&mut ir_model, mesh, space, &lines)?;
    for block in &generated.elements {
        generated_elements(&mut ir_model, block);
    }
    for (name, ids) in &sets.node_sets {
        ir_model.sets.add_nodes(&ir_name(name), ids.iter().copied());
    }
    for (name, members) in &sets.element_sets {
        let ids = match members {
            Members::Ids(ids) => ids.clone(),
            Members::Parts(parts) => (parts.iter())
                .flat_map(|p| ir_model.sets.element_sets.get(&ir_name(p)).cloned())
                .flatten()
                .collect(),
        };
        ir_model.sets.add_elements(&ir_name(name), ids);
    }
    for (name, surface) in &sets.surfaces {
        let surface = match surface {
            Surface::Faces(faces) => ir::mesh::Surface::Element(
                (faces.iter())
                    .flat_map(|(set, face)| {
                        let members = ir_model.sets.element_sets.get(&ir_name(set));
                        (members.into_iter().flatten()).map(|&element| ir::mesh::SurfaceFace {
                            element,
                            face: *face,
                        })
                    })
                    .collect(),
            ),
            Surface::Nodes(set) => ir::mesh::Surface::Node(
                (ir_model.sets.node_sets.get(&ir_name(set)).cloned()).unwrap_or_default(),
            ),
        };
        ir_model.sets.surfaces.insert(ir_name(name), surface);
    }
    ir_model.physical = physical_constants(model);
    ir_model.constraints.transforms = transforms(&sets.transforms);
    ir_model.materials = materials;
    ir_model.sections = sections;
    ir_model.orientations = orientations;
    let mut origins = Origins::default();
    for (i, &section) in section_owners.iter().enumerate() {
        if let Some(item) = origin(ModelItem::Section(section)) {
            origins.insert(ir::ItemRef::Section(i), item);
        }
    }
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
                    origins.insert(ir::ItemRef::Joint(joint.name.clone()), item);
                }
                constraint_list.joints.push(joint);
            }
            ConstraintItem::Tie(tie, cyclic) => {
                constraint_list.ties.push(tie);
                if cyclic.is_some() {
                    constraint_list.cyclic_symmetry = cyclic;
                }
            }
        }
    }
    for (i, tie) in model.ties.iter().enumerate().filter(|(_, t)| t.active) {
        if let Some(item) = origin(ModelItem::Tie(i)) {
            origins.insert(ir::ItemRef::Tie(ir_name(&name(&tie.name))), item);
        }
    }
    let active_pairs = (model.contact_pairs.iter().enumerate()).filter(|(_, p)| p.active);
    for (index, (i, _)) in active_pairs.enumerate().take(contact_pairs.len()) {
        if let Some(item) = origin(ModelItem::ContactPair(i)) {
            origins.insert(ir::ItemRef::ContactPair(index), item);
        }
    }
    for (i, material) in original.materials.iter().enumerate() {
        let item = ir::ItemRef::Material(ir_name(&name(&material.name)));
        origins.insert(item, ModelItem::Material(i));
    }
    for (i, amplitude) in original.amplitudes.iter().enumerate() {
        let item = ir::ItemRef::Amplitude(ir_name(&name(&amplitude.name)));
        origins.insert(item, ModelItem::Amplitude(i));
    }
    for (s, step) in original.steps.iter().enumerate().filter(|(_, s)| s.active) {
        for table in step.tables() {
            let (item, label) = table_owner(step, &table);
            let amplitude = ir::ItemRef::Amplitude(table_amplitude(&step.name, item, label));
            origins.insert(
                amplitude,
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
    let model_cards = boundaries_before_stored_modes(model, &mut steps).unwrap_or_default();
    let resolved_overlaps = (sets.dofs.comments())
        .map(|comments| resolved_overlaps(&comments))
        .unwrap_or_default();
    Ok(Built {
        model: ir_model,
        model_cards,
        steps,
        resolved_overlaps,
        shortened_names: sets.shortened.clone(),
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
    Tie(
        ir::constraints::Tie,
        Option<ir::constraints::CyclicSymmetryModel>,
    ),
}

/// The boundary conditions of a first step that works on the eigenmodes of another
/// analysis, taken out of the step: the solver allows no new boundary conditions in a modal
/// dynamics or steady state dynamics step, so they go into the model definition, where the
/// run that stored the eigenmodes had them before its frequency step.
fn boundaries_before_stored_modes(
    model: &FeModel,
    steps: &mut [Vec<StepBlock>],
) -> Option<Vec<Card>> {
    model.properties.stored_modes.as_ref()?;
    let first = model.steps.iter().position(|s| s.active)?;
    if !model.steps[first].kind.uses_stored_modes() {
        return None;
    }
    let block = steps.get_mut(first)?.first_mut()?;
    Some(std::mem::take(&mut block.boundaries))
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
fn transforms(transforms: &[(String, Transform)]) -> Vec<ir::constraints::Transform> {
    (transforms.iter())
        .map(|(set, transform)| {
            let (kind, a, b) = match *transform {
                Transform::Rectangular(a, b) => (ir::constraints::TransformKind::Rectangular, a, b),
                Transform::Cylindrical(a, b) => (ir::constraints::TransformKind::Cylindrical, a, b),
            };
            ir::constraints::Transform {
                nset: ir_name(set),
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
        input: input.to_string_lossy().into_owned(),
        node_sets: node_sets.iter().map(|(set, _)| ir_name(set)).collect(),
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
            out.sets.add_elements(&ir_name(set), ids);
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
fn generated_elements(out: &mut ir::Model, block: &constraints::ElementBlock) {
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
        out.sets.add_elements(&ir_name(set), ids);
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
    /// Names too long for CalculiX as written, each with the name it stands for.
    shortened: Vec<(String, String)>,
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
    reference_nodes: BTreeMap<Id, ReferenceNodes>,
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
    /// Nodes of line, shell and membrane elements, see [`rigid_body`].
    expanded: BTreeSet<NodeId>,
    /// The dependent degrees of freedom of all MPCs so far.
    dofs: dofs::Dofs,
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
            shortened: Vec::new(),
            surface_sets: BTreeMap::new(),
            reference_nodes: BTreeMap::new(),
            remote_points: Vec::new(),
            remote_names: BTreeMap::new(),
            pre_tension_nodes: BTreeMap::new(),
            pre_tension_sets: BTreeMap::new(),
            cut_originals: BTreeMap::new(),
            submodel_sets: Vec::new(),
            transforms: Vec::new(),
            expanded: plx_model::expanded_nodes(mesh),
            dofs: dofs::Dofs::default(),
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
        let (name, full) = (1..)
            .map(|n| {
                let head = format!("{prefix}-{n}_");
                (fitted_name(&head, postfix), format!("{head}{postfix}"))
            })
            .find(|(name, _)| self.used.insert(name.to_ascii_uppercase()))
            .expect("unbounded range");
        if full != name {
            self.shortened.push((name.clone(), full));
        }
        name
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
        let members = match region {
            Region::ElementSet(set) => return Ok(set.clone()),
            Region::Parts(parts) if !parts.is_empty() => Members::Parts(parts.clone()),
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

    /// The nodes of the rigid body a reference point drives.
    fn reference_nodes(&mut self, item: &str, point: Id) -> Result<ReferenceNodes, WriteError> {
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

fn materials(model: &FeModel) -> Vec<ir::material::Material> {
    use ir::material as m;
    let mut materials: Vec<m::Material> = Vec::new();
    for material in &model.materials {
        let mut out = m::Material {
            name: ir_name(&name(&material.name)),
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
        add_material(&mut materials, out);
    }
    materials
}

/// Adds a material; one of the same name, which the solver does not tell apart from it,
/// takes its properties.
fn add_material(materials: &mut Vec<ir::material::Material>, material: ir::material::Material) {
    let Some(known) = materials.iter_mut().find(|m| m.name == material.name) else {
        materials.push(material);
        return;
    };
    macro_rules! take {
        ($($field:ident),*) => {
            $(if material.$field.is_some() {
                known.$field = material.$field;
            })*
        };
    }
    take!(
        elastic,
        anisotropic,
        plastic,
        creep,
        expansion,
        conductivity,
        specific_heat,
        density
    );
}

/// The dummy plastic material that makes the solver solve gaps nonlinearly.
fn gap_material(name: &str) -> ir::material::Material {
    use ir::material as m;
    m::Material {
        name: ir_name(name),
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
fn amplitudes(model: &FeModel) -> Result<Vec<ir::amplitude::Amplitude>, WriteError> {
    let mut amplitudes = Vec::new();
    for amplitude in &model.amplitudes {
        if let Some(reason) = amplitude.points_problem() {
            return Err(WriteError::InvalidAmplitude {
                item: amplitude.name.clone(),
                reason,
            });
        }
        amplitudes.push(ir::amplitude::Amplitude {
            name: ir_name(&name(&amplitude.name)),
            total_time: amplitude.time_span == AmplitudeTime::Total,
            shift_x: amplitude.shift_time,
            shift_y: amplitude.shift_amplitude,
            points: amplitude.points.iter().map(|&[t, a]| (t, a)).collect(),
            signal: None,
        });
    }
    // Each time table of a component is an amplitude of its own, in step time, that a value
    // of 1 follows.
    for step in model.steps.iter().filter(|s| s.active) {
        for table in step.tables() {
            let (item, label) = table_owner(step, &table);
            if let Some(reason) = table.table.points_problem() {
                return Err(WriteError::InvalidAmplitude {
                    item: item.to_owned(),
                    reason,
                });
            }
            let name = table_amplitude(&step.name, item, label);
            if amplitudes.iter().any(|a| a.name == name) {
                continue;
            }
            amplitudes.push(ir::amplitude::Amplitude {
                name,
                total_time: false,
                shift_x: 0.0,
                shift_y: 0.0,
                points: table.table.points.iter().map(|&[t, v]| (t, v)).collect(),
                signal: None,
            });
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
            amplitudes.push(ir::amplitude::Amplitude {
                name: signal_amplitude(&step.name, &load.name),
                total_time: false,
                shift_x: 0.0,
                shift_y: 0.0,
                points: Vec::new(),
                signal: Some(excitation.signal.to_ir(dynamic.duration)),
            });
        }
    }
    Ok(amplitudes)
}

/// The labels of the degrees of freedom of a displacement and the components of a load,
/// which name the amplitudes of their tables.
const DOF_TABLE_LABELS: [&str; 6] = ["U1", "U2", "U3", "UR1", "UR2", "UR3"];
const FORCE_TABLE_LABELS: [&str; 3] = ["F1", "F2", "F3"];
const MOMENT_TABLE_LABELS: [&str; 3] = ["M1", "M2", "M3"];

/// The name of the item a table belongs to and the label of its component.
fn table_owner<'a>(step: &'a Step, table: &plx_model::StepTable) -> (&'a str, &'static str) {
    match table.owner {
        TableOwner::BoundaryCondition(i) => (
            &step.boundary_conditions[i].name,
            DOF_TABLE_LABELS[table.component],
        ),
        TableOwner::Load(i) => {
            let load = &step.loads[i];
            let label = match load.kind {
                LoadKind::Moment(_) => MOMENT_TABLE_LABELS[table.component],
                LoadKind::Pressure(_) => "P",
                _ => FORCE_TABLE_LABELS[table.component],
            };
            (&load.name, label)
        }
    }
}

/// The name of the amplitude of the table of component `label` of item `item` in a step,
/// as the solver compares it.
fn table_amplitude(step: &str, item: &str, label: &str) -> String {
    ir_name(&name(&format!("Table~{step}~{item}~{label}")))
}

/// The name of the amplitude of the signal of excitation `item` in a step.
fn signal_amplitude(step: &str, item: &str) -> String {
    ir_name(&name(&format!("Signal~{step}~{item}")))
}

/// The values a displacement holds and the amplitude each follows: a degree of freedom with
/// a table holds 1 and follows the amplitude of its table, the others their value and the
/// amplitude of the boundary condition.
fn displacement_values(
    step: &Step,
    bc: &plx_model::BoundaryCondition,
    displacement: &plx_model::Displacement,
    amplitude: &Option<String>,
) -> ([Option<f64>; 6], [Option<String>; 6]) {
    let values = std::array::from_fn(|d| match displacement.table(d) {
        Some(_) => Some(1.0),
        None => displacement.values[d],
    });
    let amplitudes = std::array::from_fn(|d| match displacement.table(d) {
        Some(_) => Some(table_amplitude(&step.name, &bc.name, DOF_TABLE_LABELS[d])),
        None => amplitude.clone(),
    });
    (values, amplitudes)
}

/// The components of a load's value and the amplitude each follows: a component with a
/// table is 1 and follows the amplitude of its table.
fn load_components<const N: usize>(
    step: &Step,
    load: &plx_model::Load,
    values: [f64; N],
    labels: [&str; N],
    amplitude: &Option<String>,
) -> ([f64; N], [Option<String>; N]) {
    let values = std::array::from_fn(|c| match load.table(c) {
        Some(_) => 1.0,
        None => values[c],
    });
    let amplitudes = std::array::from_fn(|c| match load.table(c) {
        Some(_) => Some(table_amplitude(&step.name, &load.name, labels[c])),
        None => amplitude.clone(),
    });
    (values, amplitudes)
}

/// The amplitude a boundary condition or load refers to, by its name in the analysis;
/// `None` without one.
fn amplitude_parameter(
    amplitudes: &[Amplitude],
    item: &str,
    reference: &Option<Id>,
) -> Result<Option<String>, WriteError> {
    let Some(reference) = reference else {
        return Ok(None);
    };
    let Some(amplitude) = amplitudes.iter().find(|a| a.id == *reference) else {
        return Err(WriteError::UnknownAmplitude {
            item: item.to_owned(),
            amplitude: reference.to_string(),
        });
    };
    Ok(Some(ir_name(&name(&amplitude.name))))
}

/// The distribution an item scales its value by, ready to sample; `None` without one.
fn distribution(
    model: &FeModel,
    item: &str,
    reference: &Option<Id>,
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
    (distribution.sampler(model.properties.units))
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
                        out.temperatures.push((node_set_ref(&set), t));
                    }
                }
            }
            InitialConditionKind::Velocity(v) => {
                let set = sets.node_set(&condition.name, &condition.region)?;
                velocities(&mut out, node_set_ref(&set), v);
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
        let material = ir_name(&name(&material.name));
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
                    let name = ir_name(&format!("ORIENTATION_{set}"));
                    orientations.push(ir::section::Orientation {
                        name: name.clone(),
                        a: axes[0],
                        b: axes[1],
                    });
                    name
                });
                out.push(IrSection {
                    elset: ir_name(&set),
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
                let template = (materials.iter())
                    .find(|m| m.name == material)
                    .cloned()
                    .unwrap_or_default();
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
                    let material_name = ir_name(&format!("Infill_{set}"));
                    let mut printed = template.clone();
                    printed.name = material_name.clone();
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
                        elset: ir_name(&set),
                        kind: Kind::Solid {
                            material: material_name,
                            thickness: None,
                            orientation: None,
                        },
                    });
                }
            }
            SectionKind::Truss { area } => {
                let set = sets.element_set(&section.name, &section.region)?;
                out.push(IrSection {
                    elset: ir_name(&set),
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
                    elset: ir_name(&set),
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
                    elset: ir_name(&set),
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
                    // A halved section is no profile of the model: read back, it is a
                    // general one of its values.
                    if !profile.is_native() && !half {
                        comments.push((ir_name(&set), profile_comment(&profile, drawn)));
                    }
                    out.push(IrSection {
                        elset: ir_name(&set),
                        kind: Kind::Beam {
                            material: material.clone(),
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
            torsion: at(4),
            shear_area_1: None,
            shear_area_2: None,
            contact_radius: profile.contact_radius(),
        },
    }
}

/// Half a general profile: the beam on a symmetry plane, of which the model holds the half
/// on one side. Area and second moments (and so the torsion stiffness and the mass) halve;
/// the contact radius stays the rope's.
fn halved_profile(profile: &ir::section::BeamProfile) -> ir::section::BeamProfile {
    use ir::section::BeamProfile as P;
    match profile.clone() {
        P::General {
            area,
            i11,
            i12,
            i22,
            torsion,
            shear_area_1,
            shear_area_2,
            contact_radius,
        } => P::General {
            area: 0.5 * area,
            i11: 0.5 * i11,
            i12: 0.5 * i12,
            i22: 0.5 * i22,
            torsion,
            shear_area_1: shear_area_1.map(|a| 0.5 * a),
            shear_area_2: shear_area_2.map(|a| 0.5 * a),
            contact_radius,
        },
        other => other,
    }
}

/// The section of generated spring and gap elements.
fn generated_section(section: &constraints::GeneratedSection) -> ir::section::Section {
    use ir::section::{Gap, SectionKind as Kind, Spring, SpringStiffness};
    match section {
        constraints::GeneratedSection::Spring {
            elset,
            dofs,
            stiffness,
            stabilize,
        } => ir::section::Section {
            elset: ir_name(elset),
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
            elset: ir_name(elset),
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
            // Moved to the model's node ties when the project was read.
            Constraint::NodeTie(_) => {}
            Constraint::RigidBody(body) => {
                let Some(point) = model.reference_point(body.reference_point) else {
                    return Err(WriteError::UnknownReferencePoint {
                        item: body.name.clone(),
                        point: body.reference_point.to_string(),
                    });
                };
                if body.behavior == RemoteBehavior::Deformable {
                    // The point follows the mean motion of the selection, as the point of a
                    // deformable remote displacement does.
                    let spec = plx_model::Displacement {
                        values: [None; 6],
                        behavior: plx_model::Behavior::Deformable,
                        system: None,
                        location: Some(point.position),
                        pinball: None,
                        tables: Default::default(),
                    };
                    let invalid = |reason: String| WriteError::InvalidConstraint {
                        item: body.name.clone(),
                        reason,
                    };
                    let excluded = model.directly_held_nodes_in(sets.mesh, |_| true);
                    let held = model.held_translations(sets.mesh);
                    let resolved = (spec.resolve(model, sets.mesh, &body.region, &excluded))
                        .map_err(invalid)?;
                    let [reference, rotation] = remote_point(
                        RemoteSource {
                            item: &body.name,
                            region: &body.region,
                            spec: &spec,
                            behavior: RemoteBehavior::Deformable,
                            owner: ModelItem::Constraint(index),
                            free_point: !model.point_bound(body.reference_point),
                        },
                        resolved,
                        held,
                        &mut next_node,
                        (&mut items, &mut owners),
                        sets,
                        model,
                        generated,
                    )?;
                    sets.reference_nodes.insert(
                        body.reference_point,
                        ReferenceNodes {
                            name: point.name.clone(),
                            reference,
                            rotation,
                            set: None,
                            behavior: RemoteBehavior::Deformable,
                        },
                    );
                    continue;
                }
                // Both nodes sit on the reference point.
                let (reference, rotation) = (next_node, next_node + 1);
                next_node += 2;
                generated.nodes.push((reference, point.position));
                generated.nodes.push((rotation, point.position));
                sets.reference_nodes.insert(
                    body.reference_point,
                    ReferenceNodes {
                        name: point.name.clone(),
                        reference,
                        rotation,
                        set: None,
                        behavior: RemoteBehavior::Rigid,
                    },
                );
                owners.resize(items.len(), None);
                rigid_body(
                    &mut items,
                    sets,
                    &body.name,
                    &body.region,
                    point.position,
                    None,
                    [reference, rotation],
                )?;
                owners.resize(items.len(), Some(ModelItem::Constraint(index)));
            }
            Constraint::Tie(t) => items.push(tie(sets, t)?),
        }
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
                (sets.remote_names.entry(bc.id)).or_insert((nodes, behavior));
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
            (sets.remote_names.entry(bc.id)).or_insert((nodes, behavior));
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
            (sets.remote_names.entry(load.id)).or_insert((nodes, force.behavior));
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
        (sets.remote_names.entry(load.id)).or_insert((nodes, force.behavior));
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
    for t in model.ties.iter().filter(|t| t.active) {
        items.push(tie(sets, t)?);
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
            let written = rigid_body(
                items,
                sets,
                item,
                &region,
                point.position,
                point.axes,
                [reference, rotation],
            );
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
    } else if membrane_edges(sets.mesh, slave_region) {
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
                tie: tie_name.clone(),
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
            slave: ir_name(&slave),
            master: ir_name(&master),
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

/// A surface interaction, with the defaults filled
/// in.
fn interaction(
    interaction: &SurfaceInteraction,
    defaults: &InteractionDefaults,
) -> ir::contact::SurfaceInteraction {
    use ir::contact as c;
    let mut out = c::SurfaceInteraction {
        name: ir_name(&name(&interaction.name)),
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
        let master = sets.surface(&pair.name, "Master", &pair.master)?;
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
            sets.surface(&pair.name, "Slave", &pair.slave)?
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
            interaction: ir_name(&name(&interaction.name)),
            kind,
            small_sliding: pair.method == ContactMethod::NodeToSurface && pair.small_sliding,
            adjust: (pair.adjust).then(|| c::Adjust::Distance(pair.adjustment_size.unwrap_or(0.0))),
            slave: ir_name(&slave),
            master: ir_name(&master),
        });
        if let Some(value) = pair.clearance {
            clearances.push(c::Clearance {
                master: ir_name(&master),
                slave: ir_name(&slave),
                value,
            });
        }
        surfaces.insert(pair.id, (master, slave));
    }
    Ok((pairs, clearances, surfaces))
}

/// What the steps share: the model's settings and the items lowered before them.
struct StepContext<'a> {
    extra_boundary: Option<&'a Card>,
    space: ModelSpace,
    lines: &'a LineElements,
    flux_kinds: FluxKinds,
    amplitudes: &'a [Amplitude],
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
                let cards = moving_load_cards(load, frame, context)?;
                if let Some((_, slot)) = (parts.loads.iter_mut()).find(|(i, _)| *i == Some(*index))
                {
                    *slot = cards;
                }
            }
        }
        let mut ir_step = parts.step;
        ir_step.name = title.clone().unwrap_or_default();
        let mut cards: Vec<Card> = parts.loads.into_iter().flat_map(|(_, c)| c).collect();
        cards.extend(parts.rest);
        blocks.push(StepBlock {
            step: ir_step,
            named: title.is_some(),
            boundaries: parts.boundaries,
            cards,
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
        return Ok(vec![
            StepBlock {
                step: eigenmodes,
                named: dynamic.named,
                boundaries: dynamic.boundaries.clone(),
                cards: Vec::new(),
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
        let mut boundaries = vec![Card::Boundary {
            op_new: true,
            items: Vec::new(),
        }];
        boundaries.extend(response.boundaries.iter().cloned());
        return Ok(vec![
            StepBlock {
                step: eigenmodes,
                named: response.named,
                boundaries,
                cards: Vec::new(),
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
        Region::Parts(sets.mesh.parts.iter().map(|p| p.name.clone()).collect())
    } else {
        settings.region.clone()
    };
    let set = sets.element_set(&format!("{}_Rotation", step.name), &region)?;
    let rotation = Card::Dload {
        op_new: false,
        items: vec![dload(
            element_set_ref(&set),
            ir::step::DistributedLoadKind::Centrifugal {
                omega_squared: settings.speed * settings.speed,
                point,
                axis,
            },
            &None,
        )],
    };
    // Named after the step, so that they do not take the name of another one.
    let helper = |mut ir_step: ir::Step, purpose: &str, cards: Vec<Card>| {
        ir_step.name = format!("{} {purpose}", complex.step.name);
        StepBlock {
            step: ir_step,
            named: complex.named,
            boundaries: complex.boundaries.clone(),
            cards,
            preparation: true,
        }
    };
    let mut blocks = Vec::with_capacity(3);
    let unloaded = !settings.base.is_preloaded();
    // About the unloaded model the rotation is the only load; otherwise it adds to the
    // loads the steps before left.
    let mut first_cards = if unloaded {
        vec![
            Card::Cload {
                op_new: true,
                items: Vec::new(),
            },
            Card::Dload {
                op_new: true,
                items: Vec::new(),
            },
        ]
    } else {
        Vec::new()
    };
    first_cards.push(rotation);
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
            std::mem::take(&mut first_cards),
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
        first_cards,
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

/// A rigid body of the nodes of `region` driven by a reference and a rotation node at
/// `position`: a rigid body, but with the nodes of beams, trusses, shells and membranes
/// rigidly coupled by equations. CalculiX 2.23 stops on a rigid body containing such a
/// node ("nodes belonging to beam elements must not be subject to a RIGID MPC"); the
/// equations hold them for small rotations as the rigid body does in a linear analysis.
fn rigid_body(
    items: &mut Vec<ConstraintItem>,
    sets: &mut Sets,
    item: &str,
    region: &Region,
    position: [f64; 3],
    axes: Option<[[f64; 3]; 3]>,
    nodes: [NodeId; 2],
) -> Result<(), WriteError> {
    let [reference, rotation] = nodes;
    // Nodes that a boundary condition or another constraint binds already stay out.
    let all = region.nodes(sets.mesh);
    let count = all.len();
    let kept = sets.dofs.claim_nodes(item, all);
    let whole = kept.len() == count;
    let (expanded, others): (Vec<NodeId>, Vec<NodeId>) =
        (kept.into_iter()).partition(|n| sets.expanded.contains(n));
    let rigid = |set: String| {
        ConstraintItem::RigidBody(ir::constraints::RigidBody {
            nodes: ir::constraints::RigidBodyNodes::NodeSet(ir_name(&set)),
            reference_node: reference,
            rotation_node: rotation,
        })
    };
    if expanded.is_empty() {
        let set = if whole {
            sets.node_set(item, region)?
        } else {
            sets.node_set(item, &Region::Nodes(others))?
        };
        items.push(rigid(set));
        return Ok(());
    }
    if !others.is_empty() {
        items.push(rigid(sets.node_set(item, &Region::Nodes(others))?));
    }
    // Nodes of beams and shells, which the solver refuses in a rigid body.
    for row in plx_model::rigid_rows(sets.mesh, &expanded, position) {
        items.push(ConstraintItem::Equation(equation(&remote_terms(
            &row, axes, nodes,
        ))));
    }
    Ok(())
}

/// How the displacement boundary condition of the name in the step of the name is written.
fn formulation(model: &FeModel, mesh: &FeMesh, step: &str, bc: &str) -> Option<Formulation> {
    let s = model.steps.iter().position(|s| s.name == step)?;
    let i = (model.steps[s].boundary_conditions.iter()).position(|b| b.name == bc)?;
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
    amplitude: Option<String>,
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
        (None, None) => {
            ir::step::Follower::Nodes(node_set_ref(&sets.node_set(&load.name, &load.region)?))
        }
    }))
}

/// A load in a coordinate system that moves with a selection turns with the selection's
/// nodes; `None` for systems fixed in space.
fn moving_system_follower(
    model: &FeModel,
    sets: &mut Sets,
    system: Option<plx_model::Id>,
) -> Result<Option<ir::step::Follower>, WriteError> {
    let Some(region) = model.system_attachment(system) else {
        return Ok(None);
    };
    let name = model.system_name(system);
    let set = sets.node_set(&name, region)?;
    Ok(Some(ir::step::Follower::Nodes(node_set_ref(&set))))
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
    amplitude: &Option<String>,
) -> ir::step::ConcentratedLoad {
    ir::step::ConcentratedLoad {
        region,
        dof,
        value,
        amplitude: amplitude.clone(),
        load_case: None,
        sector: None,
        follower: None,
    }
}

/// A distributed load on `region`.
fn dload(
    region: ElementRef,
    kind: ir::step::DistributedLoadKind,
    amplitude: &Option<String>,
) -> ir::step::DistributedLoad {
    ir::step::DistributedLoad {
        region,
        kind,
        amplitude: amplitude.clone(),
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
    amplitudes: &[Option<String>; 6],
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
    let mut run: Option<(&NodeRef, u8, u8, f64, &Option<String>)> = None;
    for (target, dof, value, amplitude) in held {
        match &mut run {
            Some((t, _, last, v, a))
                if *t == target && *last + 1 == dof && *v == value && *a == amplitude =>
            {
                *last = dof;
            }
            _ => {
                if let Some((t, first, last, v, a)) = run {
                    out.push(boundary(t.clone(), first, last, Some(v), a.clone()));
                }
                run = Some((target, dof, dof, value, amplitude));
            }
        }
    }
    if let Some((t, first, last, v, a)) = run {
        out.push(boundary(t.clone(), first, last, Some(v), a.clone()));
    }
    out
}

/// The cards of a moving load at one position: its nodal forces as concentrated loads;
/// none while all wheels are off the region.
fn moving_load_cards(
    load: &plx_model::Load,
    frame: &MovingFrame,
    context: &StepContext,
) -> Result<Vec<Card>, WriteError> {
    if frame.forces.is_empty() {
        return Ok(Vec::new());
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
    Ok(vec![Card::Cload {
        op_new: false,
        items,
    }])
}

/// A step before its moving loads are placed: its settings, the cards of its boundary
/// conditions, those of each load (by the load's index) and those after the loads.
#[derive(Clone)]
struct StepParts {
    step: ir::Step,
    boundaries: Vec<Card>,
    loads: Vec<(Option<usize>, Vec<Card>)>,
    rest: Vec<Card>,
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
        flux_kinds,
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
    ir_step.contact_changes = contact_changes(step, model, pair_surfaces);
    // A modal step keeps the supports of the frequency step that stored the modes;
    // The solver refuses new ones ("in a modal dynamic step new SPCs are not allowed"), and
    // the reset would make the same supports new. Giving them again is accepted.
    let mut boundaries = if step.kind.uses_stored_modes() || step.kind.keeps_previous_boundaries() {
        Vec::new()
    } else {
        vec![Card::Boundary {
            op_new: true,
            items: Vec::new(),
        }]
    };
    for bc in &step.boundary_conditions {
        // A step leaves out what it cannot take, like a deactivated item: displacements in
        // a heat transfer step, temperatures in a static one.
        if !bc.active || !step.kind.supports_boundary(&bc.kind) {
            continue;
        }
        let on_point = sets.remote_names.contains_key(&bc.id)
            && matches!(
                formulation(model, sets.mesh, &step.name, &bc.name),
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
            let (values, dof_amplitudes) = displacement_values(step, bc, remote, &amplitude);
            let held = remote.held(space).map(|v| v.map(|_| ()));
            let held = std::array::from_fn(|d| held[d].and(values[d]));
            let (reference, rotation) = (NodeRef::Node(reference), NodeRef::Node(rotation));
            boundaries.push(Card::Boundary {
                op_new: false,
                items: displacement_boundaries(
                    &reference,
                    Some(&rotation),
                    &held,
                    6,
                    &dof_amplitudes,
                ),
            });
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
                    items.push(boundary(node, 11, 11, Some(value), amplitude.clone()));
                }
            }
            boundaries.push(Card::Boundary {
                op_new: false,
                items,
            });
            continue;
        } else if cyclic_slaves.is_empty() {
            sets.node_set(&bc.name, &bc.region)?
        } else {
            let nodes = bc.region.nodes(sets.mesh);
            if nodes.iter().any(|n| cyclic_slaves.contains(n)) {
                let free = nodes.into_iter().filter(|n| !cyclic_slaves.contains(n));
                sets.node_set(&bc.name, &Region::Nodes(free.collect()))?
            } else {
                sets.node_set(&bc.name, &bc.region)?
            }
        };
        let target = node_set_ref(&set);
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
                    displacement_values(step, bc, displacement, &amplitude);
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
                    items.push(boundary(
                        target.clone(),
                        dof,
                        dof,
                        Some(0.0),
                        amplitude.clone(),
                    ));
                }
            }
        }
        boundaries.push(Card::Boundary {
            op_new: false,
            items,
        });
    }
    // Boundary conditions that constraints need.
    if step.kind.is_mechanical() {
        boundaries.extend(extra_boundary.cloned());
    }
    let mut loads: Vec<(Option<usize>, Vec<Card>)> = Vec::new();
    // A step resets the loads it takes; one that takes no loads gets none,
    // not even the reset. Distributed fluxes, films and radiation are reset only when the
    // model has them.
    let kind = &step.kind;
    let mechanical_loads = kind.supports_load(&LoadKind::Pressure(0.0));
    let mut resets = Vec::new();
    if mechanical_loads {
        resets.push(Card::Cload {
            op_new: true,
            items: Vec::new(),
        });
        resets.push(Card::Dload {
            op_new: true,
            items: Vec::new(),
        });
    }
    if kind.supports_load(&LoadKind::flux(0.0)) {
        resets.push(Card::Cflux {
            op_new: true,
            items: Vec::new(),
        });
    }
    if kind.is_thermal() && flux_kinds.film {
        resets.push(Card::Film {
            op_new: true,
            items: Vec::new(),
        });
    }
    if kind.is_thermal() && flux_kinds.radiation {
        resets.push(Card::Radiate {
            op_new: true,
            items: Vec::new(),
        });
    }
    loads.push((None, resets));
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
            LoadKind::Excitation(_) => Some(signal_amplitude(&step.name, &load.name)),
            _ => amplitude_parameter(amplitudes, &load.name, &load.amplitude)?,
        };
        // The second amplitude of a film scales its coefficient, of radiation the emissivity.
        let factor_amplitude = match load.kind {
            LoadKind::Film { .. } | LoadKind::Radiation { .. } => {
                amplitude_parameter(amplitudes, &load.name, &load.factor_amplitude)?
            }
            _ => None,
        };
        let cards = match load.kind {
            LoadKind::Moving(_) => {
                let frame = moving(index).cloned().unwrap_or_default();
                loads.push((Some(index), moving_load_cards(load, &frame, context)?));
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
                    None => (node_set_ref(&sets.node_set(&load.name, &load.region)?), 4),
                };
                let (moment, component_amplitudes) =
                    load_components(step, load, moment, MOMENT_TABLE_LABELS, &amplitude);
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
                vec![Card::Cload {
                    op_new: false,
                    items,
                }]
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
                vec![Card::Cload {
                    op_new: false,
                    items,
                }]
            }
            LoadKind::Pressure(pressure) => {
                use ir::step::DistributedLoadKind::Pressure;
                let ([pressure], [amplitude]) =
                    load_components(step, load, [pressure], ["P"], &amplitude);
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
                            items.push(dload(element_set_ref(&set), kind, &amplitude));
                        }
                    }
                }
                vec![Card::Dload {
                    op_new: false,
                    items,
                }]
            }
            LoadKind::PreTension {
                value,
                by_displacement,
                lock,
                ..
            } => {
                // The preload is the first degree of freedom of the pre-tension node: a
                // force, or the shortening of the bolt since the start as a prescribed
                // displacement. Locked, the node keeps the shortening of the previous step.
                let node = NodeRef::Node(sets.pre_tension_nodes[&load.name]);
                if lock {
                    let mut item = boundary(node, 1, 1, None, None);
                    item.fixed = true;
                    vec![Card::Boundary {
                        op_new: false,
                        items: vec![item],
                    }]
                } else if by_displacement {
                    vec![Card::Boundary {
                        op_new: false,
                        items: vec![boundary(node, 1, 1, Some(value), amplitude)],
                    }]
                } else {
                    vec![Card::Cload {
                        op_new: false,
                        items: vec![cload(node, 1, value, &amplitude)],
                    }]
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
                let (force, component_amplitudes) =
                    load_components(step, load, force, FORCE_TABLE_LABELS, &amplitude);
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
                    let target = node_set_ref(&sets.node_set(&load.name, &load.region)?);
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
                vec![Card::Cload {
                    op_new: false,
                    items,
                }]
            }
            LoadKind::Flux {
                value: flux,
                each_node,
            } => {
                let faces = load.region.faces(sets.mesh);
                let edges = load.region.edges(sets.mesh);
                let bodies = matches!(load.region, Region::Parts(_) | Region::ElementSet(_));
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
                        region: node_set_ref(&set),
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
                            amplitude: amplitude.clone(),
                        })
                        .collect()
                };
                vec![Card::Cflux {
                    op_new: false,
                    items,
                }]
            }
            LoadKind::Film { sink, coefficient } => {
                let items = (sets.face_sets(&load.name, &load.region)?.into_iter())
                    .map(|(set, face)| ir::step::Film {
                        region: element_set_ref(&set),
                        face,
                        sink_temperature: sink,
                        coefficient,
                        amplitude: amplitude.clone(),
                        film_amplitude: factor_amplitude.clone(),
                    })
                    .collect();
                vec![Card::Film {
                    op_new: false,
                    items,
                }]
            }
            LoadKind::Radiation { sink, emissivity } => {
                let items = (sets.face_sets(&load.name, &load.region)?.into_iter())
                    .map(|(set, face)| ir::step::Radiate {
                        region: element_set_ref(&set),
                        face,
                        cavity: false,
                        sink_temperature: sink,
                        emissivity,
                        amplitude: amplitude.clone(),
                        radiation_amplitude: factor_amplitude.clone(),
                    })
                    .collect();
                vec![Card::Radiate {
                    op_new: false,
                    items,
                }]
            }
            // Gravity: the size of the acceleration and its direction.
            LoadKind::Gravity(_) | LoadKind::StandardGravity(_) => {
                let acceleration = load.kind.gravity_vector().unwrap_or_default();
                let set = sets.element_set(&load.name, &load.region)?;
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
                vec![Card::Dload {
                    op_new: false,
                    items: vec![dload(element_set_ref(&set), kind, &amplitude)],
                }]
            }
            // Centrifugal load: the square of the rotational speed, a point on
            // the axis and the axis direction.
            LoadKind::Centrifugal { point, axis, speed } => {
                let set = sets.element_set(&load.name, &load.region)?;
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
                vec![Card::Dload {
                    op_new: false,
                    items: vec![dload(element_set_ref(&set), kind, &amplitude)],
                }]
            }
        };
        loads.push((Some(index), cards));
    }
    let mut rest = defined_fields(sets, step, amplitudes, model)?;
    // Only the time integration computes the energies.
    let dynamic = matches!(step.kind, StepKind::Dynamic(_));
    // The outputs of the model the step lists are written into it as its own.
    for output in (step.history_outputs.iter())
        .filter_map(|&id| model.history_output(id))
        .filter(|o| o.active && (dynamic || o.kind != HistoryKind::Energies))
    {
        rest.extend(history_output(sets, output, pair_surfaces)?);
    }
    // The force of a bolt is the reaction of its pre-tension node, also while it is locked,
    // and its shortening the node's displacement: shown under the name of the load.
    for load in step.loads.iter().filter(|l| l.active) {
        if let Some(set) = sets.pre_tension_sets.get(&load.name) {
            rest.push(Card::NodePrint(node_print(set, &["RF", "U"], Totals::No)));
        }
    }
    // The solver writes all results of the step in one form: the node outputs follow the
    // element outputs.
    let dimension = (step.field_outputs.iter())
        .filter(|o| o.active && o.kind == OutputKind::Element)
        .map(|o| o.dimension)
        .find(|&d| d != OutputDimension::Default)
        .unwrap_or_default();
    rest.extend(
        (step.field_outputs.iter())
            .filter_map(|output| field_output(output, dimension, frame_every)),
    );
    Ok(StepParts {
        step: ir_step,
        boundaries,
        loads,
        rest,
    })
}

/// The contact pairs a step switches off or on and whose friction it changes, with the
/// stick slope of the interaction. Changes of pairs not lowered, deactivated or unknown,
/// are left out.
fn contact_changes(
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
                slave: ir_name(slave),
                master: ir_name(master),
            });
        }
        let pair = model.contact_pair(change.region);
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
                interaction: ir_name(&name(&interaction.name)),
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
/// to read. CalculiX 2.21 takes the file name with its extension and looks for it next to
/// the input file; `BStep` is the only step parameter it knows.
fn defined_fields(
    sets: &mut Sets,
    step: &Step,
    amplitudes: &[Amplitude],
    model: &FeModel,
) -> Result<Vec<Card>, WriteError> {
    let mut cards = Vec::new();
    // A thermal step solves for the temperatures; it leaves the fields out like
    // deactivated ones.
    if !step.kind.supports_defined_fields() {
        return Ok(cards);
    }
    // What a step shows is what acts: its own temperatures replace those of the steps
    // before, and a step that passes its state on without temperatures of its own sets
    // those a step before was given back to the initial ones (ramped like removed loads).
    let index = (model.steps.iter().position(|s| std::ptr::eq(s, step)))
        .or_else(|| model.steps.iter().position(|s| s.name == step.name));
    if let Some(index) = index
        && model.temperature_source(index, step).is_some()
    {
        cards.push(Card::Temperature {
            op_new: true,
            field: ir::step::DefinedField {
                kind: ir::step::DefinedFieldKind::Temperature(Vec::new()),
                amplitude: None,
            },
        });
        return Ok(cards);
    }
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
                        vec![(node_set_ref(&set), *t)]
                    }
                };
                ir::step::DefinedField {
                    kind: ir::step::DefinedFieldKind::Temperature(values),
                    amplitude,
                }
            }
            DefinedFieldKind::TemperatureFromFile { file, step } => {
                let name = file.file_name().map(|n| n.to_string_lossy());
                let Some(name) = name.filter(|n| !n.is_empty() && !n.contains(',')) else {
                    return Err(WriteError::InvalidResultFile {
                        item: field.name.clone(),
                        file: file.display().to_string(),
                    });
                };
                ir::step::DefinedField {
                    kind: ir::step::DefinedFieldKind::TemperatureFile {
                        file: name.into_owned(),
                        bstep: Some(*step),
                        estep: None,
                        binc: None,
                        einc: None,
                    },
                    amplitude: None,
                }
            }
        };
        cards.push(Card::Temperature {
            op_new: cards.is_empty(),
            field,
        });
    }
    Ok(cards)
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

/// Which kinds of heat loads the model has in any step: films and radiation are reset only
/// then.
#[derive(Clone, Copy, Debug, Default)]
struct FluxKinds {
    film: bool,
    radiation: bool,
}

impl FluxKinds {
    fn of(model: &FeModel) -> Self {
        let mut kinds = Self::default();
        for load in model.steps.iter().flat_map(|s| &s.loads) {
            match load.kind {
                LoadKind::Film { .. } => kinds.film = true,
                LoadKind::Radiation { .. } => kinds.radiation = true,
                _ => {}
            }
        }
        kinds
    }
}

/// The equation solver a step asks for.
fn equation_solver(choice: plx_model::LinearSolverChoice) -> ir::step::EquationSolver {
    use ir::step::EquationSolver as S;
    match choice {
        plx_model::LinearSolverChoice::Automatic => S::Default,
        plx_model::LinearSolverChoice::Direct => S::Pastix,
        plx_model::LinearSolverChoice::Iterative => S::IterativeCholesky,
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
    let elements = if responses.iter().any(|r| *r != "EIGENFREQUENCY") {
        let parts = sets.mesh.parts.iter().map(|p| p.name.clone()).collect();
        Some(ir_name(&sets.element_set(step, &Region::Parts(parts))?))
    } else {
        None
    };
    let responses = (responses.iter())
        .map(|response| ir::step::DesignResponse {
            name: ir_name(response),
            kind: ir_name(response),
            elset: elements.clone(),
            data: Vec::new(),
        })
        .collect();
    Ok(new_step(ir::step::Procedure::Sensitivity(
        ir::step::Sensitivity {
            nlgeom: false,
            responses,
            filter: (settings.filter_radius).map(|radius| ir::step::Filter {
                kind: "LINEAR".into(),
                radius,
            }),
            objectives: Vec::new(),
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
        nset: ir_name(&set),
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
                solver: ir::step::EquationSolver::Default,
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
            solver: ir::step::EquationSolver::Default,
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
            solver: ir::step::EquationSolver::Default,
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

/// The variables of an output request as the solver compares them.
fn variables<'a>(variables: impl IntoIterator<Item = &'a str>) -> Vec<String> {
    (variables.into_iter())
        .map(|v| v.trim().to_ascii_uppercase())
        .collect()
}

/// The field output request of a field output; none without variables. Node outputs take the form
/// `dimension` of the step's element outputs.
fn field_output(output: &FieldOutput, dimension: OutputDimension, every: u32) -> Option<Card> {
    if !output.active || output.variables.is_empty() {
        return None;
    }
    let mut names = variables(output.variables.iter().map(String::as_str));
    if output.kind == OutputKind::Element {
        // Nodalis adds nodal extrapolation of stresses unless error estimators are
        // requested.
        let has = |v: &str| output.variables.iter().any(|x| x == v);
        if has("S") && !has("ERR") && !has("HER") && !has("ZZS") {
            names.push("NOE".into());
        }
    }
    let contact = output.kind == OutputKind::Contact;
    let dimension = match output.kind {
        OutputKind::Element => output.dimension,
        _ => dimension,
    };
    let request = ir::step::FieldOutput {
        variables: names,
        frequency: every,
        nset: None,
        last_iterations: output.last_iterations,
        contact_elements: output.contact_elements,
        // CalculiX's default writes shells and beams as the solids it expands them to; at
        // the shell nodes (2D) the stresses are those of the mid-surface, without bending.
        output: match dimension {
            _ if contact => ir::step::OutputDimension::Expanded,
            OutputDimension::Default => ir::step::OutputDimension::Expanded,
            OutputDimension::TwoD => ir::step::OutputDimension::TwoD,
            OutputDimension::ThreeD => ir::step::OutputDimension::ThreeD,
        },
        // A field output's coordinate system is applied to the results when they are
        // shown, so that it may move; the solver writes global results.
        global: true,
        section_forces: false,
        time_points: None,
    };
    Some(match output.kind {
        OutputKind::Node => Card::NodeFile(request),
        OutputKind::Element => Card::ElFile(request),
        OutputKind::Contact => Card::ContactFile(request),
    })
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
fn node_print(set: &str, names: &[&str], totals_of: Totals) -> ir::step::HistoryOutput {
    ir::step::HistoryOutput {
        set: Some(ir_name(set)),
        variables: variables(names.iter().copied()),
        frequency: 1,
        totals: totals(totals_of),
        global: true,
        time_points: None,
        surfaces: None,
    }
}

/// The history output requests of a history output; none without variables.
fn history_output(
    sets: &mut Sets,
    output: &HistoryOutput,
    pair_surfaces: &PairSurfaces,
) -> Result<Vec<Card>, WriteError> {
    if output.variables.is_empty() {
        return Ok(Vec::new());
    }
    let names: Vec<&str> = output.variables.iter().map(String::as_str).collect();
    let card = match &output.kind {
        HistoryKind::Node {
            region: region @ (Region::ReferencePoint(_) | Region::RemotePoint(_)),
        } => {
            return point_output(sets, output, region);
        }
        HistoryKind::Node { region } => {
            let set = sets.node_set(&output.name, region)?;
            Card::NodePrint(node_print(&set, &names, output.totals))
        }
        HistoryKind::Energies => {
            // Sums over the whole model, printed under a set of all elements.
            let parts = sets.mesh.parts.iter().map(|p| p.name.clone()).collect();
            let set = sets.element_set(&output.name, &Region::Parts(parts))?;
            let mut request = node_print(&set, &names, Totals::Only);
            request.set = Some(ir_name(&set));
            Card::ElPrint(request)
        }
        HistoryKind::Element { region } => {
            let set = sets.element_set(&output.name, region)?;
            let mut request = node_print(&set, &names, output.totals);
            request.set = Some(ir_name(&set));
            Card::ElPrint(request)
        }
        HistoryKind::Contact { pair } => {
            // Contact forces are summed over the slave surface of the pair.
            let mut request = node_print("", &names, output.totals);
            request.set = None;
            if output.variables.iter().any(|v| v == "CF") {
                let (master, slave) =
                    (pair.and_then(|p| pair_surfaces.get(&p))).ok_or_else(|| {
                        WriteError::UnknownContactPair {
                            item: output.name.clone(),
                            pair: pair.map_or_else(|| "(none)".into(), |p| p.to_string()),
                        }
                    })?;
                let master = if master.is_empty() {
                    String::new()
                } else {
                    ir_name(master)
                };
                request.surfaces = Some((ir_name(slave), master));
            }
            Card::ContactPrint(request)
        }
    };
    Ok(vec![card])
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
) -> Result<Vec<Card>, WriteError> {
    let ([reference, rotation], behavior) = point_nodes(sets, &output.name, region)?;
    let reactions = behavior == RemoteBehavior::Rigid;
    let printed: Vec<&str> = (output.variables.iter())
        .map(String::as_str)
        .filter(|&v| reactions || v != "RF")
        .collect();
    if printed.is_empty() {
        return Ok(Vec::new());
    }
    let set = sets.free_name("Internal_Selection", &name(&output.name));
    sets.node_sets.push((set.clone(), vec![reference]));
    let mut cards = vec![Card::NodePrint(node_print(&set, &printed, output.totals))];
    let turning: Vec<&str> = (printed.iter().copied())
        .filter(|v| ["RF", "U", "V"].contains(v))
        .collect();
    if !turning.is_empty() {
        let set = sets.free_name("Internal_Rotation", &name(&output.name));
        sets.node_sets.push((set.clone(), vec![rotation]));
        cards.push(Card::NodePrint(node_print(&set, &turning, output.totals)));
    }
    Ok(cards)
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
/// Longest set or surface name the solver reads.
const NAME_LIMIT: usize = 80;

/// Longest name of a set made for a region: an element surface appends the face (`_S6`) to
/// its sets.
const SET_NAME_LIMIT: usize = NAME_LIMIT - 3;

/// Start of the comment that names what a shortened name stands for, which the reader of
/// the `.dat` file puts back.
pub const SHORTENED_NAME: &str = "** Shortened name: ";

/// `head` and `postfix` as one name, which the solver reads: a name too long keeps its start
/// and ends in a hash of the whole name, so that names alike in their start differ.
fn fitted_name(head: &str, postfix: &str) -> String {
    let full = format!("{head}{postfix}");
    if full.len() <= SET_NAME_LIMIT {
        return full;
    }
    // FNV-1a, case-insensitive like the names.
    let hash = (full.bytes()).fold(0x811c_9dc5_u32, |hash, byte| {
        (hash ^ u32::from(byte.to_ascii_uppercase())).wrapping_mul(0x0100_0193)
    });
    let keep = SET_NAME_LIMIT.saturating_sub(head.len() + 9);
    let start: String = postfix.chars().take(keep).collect();
    format!("{head}{start}_{hash:08X}")
}

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
mod pre_tension;

pub use dofs::resolved_overlaps;
#[cfg(test)]
mod tests;
