//! The text writer as it was before the direct lowering ([`super::write::lower`]), kept
//! for the tests only: the writer tests check that the lowered analysis is what the solver
//! reads of this text. Removed with the switch of the callers to the lowering.
#![allow(dead_code)]

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

use plx_mesh::{ElementFamily, ElementId, FeMesh, NodeId, SurfaceDefinition};
use plx_model::{
    Amplitude, AmplitudeTime, BeamProfile, BeamSection, BoundaryKind, BuckleStep,
    ComplexFrequencyStep, Constraint, ContactMethod, ContactPair, ContactScale, DefinedFieldKind,
    DynamicStep, FeModel, FieldOutput, Formulation, FrequencyStep, GapConductance,
    HeatTransferStep, HistoryKind, HistoryOutput, Incrementation, InitialConditionKind,
    InteractionProperty, LoadKind, ModalDamping, ModalDynamicsStep, ModelSpace, MovingFrame,
    NodeTie, OutputDimension, OutputKind, Quantity, Region, RemoteBehavior, RemoteDof, Sampler,
    Section, SectionKind, SectorDisplay, SensitivityStep, StaticStep, SteadyStateDynamicsStep,
    Step, StepKind, SurfaceBehavior, SurfaceInteraction, Tie, Totals, UnitSystem, ViscoStep,
    line_tangent, symmetry_dofs, symmetry_plane,
};

pub use super::write::WriteError;

/// One entry of the keyword tree of an input file: section titles hold the keywords written under them, a step holds its loads and
/// outputs. The text of the file is the tree written depth first.
#[derive(Clone, Debug, PartialEq)]
pub struct Keyword {
    pub kind: KeywordKind,
    /// The lines of a generated keyword, each ending in a newline.
    pub text: String,
    pub children: Vec<Keyword>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum KeywordKind {
    /// Section banner such as `** Materials ++++`; the text is empty.
    Title(String),
    /// Written from the mesh and the FE model.
    Generated,
}

impl Keyword {
    fn title(name: &str, children: Vec<Keyword>) -> Self {
        Self {
            kind: KeywordKind::Title(name.to_owned()),
            text: String::new(),
            children,
        }
    }

    fn generated(text: String) -> Self {
        Self::parent(text, Vec::new())
    }

    fn parent(text: String, children: Vec<Keyword>) -> Self {
        Self {
            kind: KeywordKind::Generated,
            text,
            children,
        }
    }

    /// What this keyword adds to the file, without its children.
    pub fn output(&self) -> String {
        match &self.kind {
            KeywordKind::Title(name) => format!("**\n{:+<60}\n**\n", format!("** {name} ")),
            KeywordKind::Generated => self.text.clone(),
        }
    }
}

/// Writes the mesh and the analysis as the text of an input file.
pub fn write_inp(mesh: &FeMesh, model: &FeModel, heading: &str) -> Result<String, WriteError> {
    Ok(write_keywords(&model_keywords(mesh, model, heading)?))
}

/// Writes the input file of `model` with the sets of `extra` steps too, which are not
/// written themselves: the steps of analyses that continue from the state after a step of
/// this one. CalculiX takes no new sets after reading a state, so the run that writes it has
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
    if extra.iter().all(|(_, steps)| steps.is_empty()) {
        return write_inp(mesh, model, heading);
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
    let Ok(mut tree) = model_keywords(mesh, &combined, heading) else {
        return write_inp(mesh, model, heading);
    };
    if let Some(steps) = tree
        .iter_mut()
        .find(|k| matches!(&k.kind, KeywordKind::Title(name) if name == "Steps"))
    {
        let mut written = written.into_iter();
        steps.children.retain(|_| written.next().unwrap_or(true));
    }
    Ok(write_keywords(&tree))
}

/// An input file with only the mesh, its nodes moved by `scale` times `displacements` (one per
/// node, in the order of [`FeMesh::coords`]), as PrePoMax's "Export deformed mesh": the
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

/// The text of a keyword tree.
pub fn write_keywords(tree: &[Keyword]) -> String {
    fn write(out: &mut String, keyword: &Keyword) {
        out.push_str(&keyword.output());
        for child in &keyword.children {
            write(out, child);
        }
    }
    let mut out = String::new();
    for keyword in tree {
        write(&mut out, keyword);
    }
    out
}

/// The keyword tree written from mesh and model. Like PrePoMax it always has every section
/// title.
pub fn model_keywords(
    mesh: &FeMesh,
    model: &FeModel,
    heading: &str,
) -> Result<Vec<Keyword>, WriteError> {
    // The faces of 2D elements are their edges; a mesh whose surface elements are still
    // typed as shells is written as the model space types them. Parts get the element
    // types the user chose for them, and linear shells of membrane sections are written as
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
    // Bolts become pre-tension loads and ties, and switch off the contact regions on
    // their cuts.
    let bolted;
    let model = if model.bolts.is_empty() {
        model
    } else {
        bolted = model.lower_bolts(mesh).0;
        &bolted
    };
    // Contact regions become the ties and contact pairs they stand for.
    let lowered;
    let model = if model.contact_regions.is_empty() {
        model
    } else {
        lowered = model.lower_contacts(mesh).0;
        &lowered
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
    // Node ties of beams, and of anything without rotations, are written as one shared node.
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
    // Regions are resolved first: their sets must precede the materials and steps. The
    // constraints come in their order of precedence for the dependent degrees of freedom
    // (see [`dofs`]): rigid bodies and remote points, pre-tension cuts, node ties, last
    // springs and contact regions.
    let materials = materials(model);
    let mut sections = sections(&mut sets, model)?;
    let mut generated = constraints::Generated::default();
    let mut constraints = constraints(&mut sets, model, &mut generated)?;
    let pre_tension_sections = pre_tension_sections(&mut sets, model, &cuts, &mut generated);
    let node_ties = node_ties(&mut sets, model, &lines)?;
    constraints::springs(&mut sets, model, &mut generated)?;
    constraints.extend(generated.equations);
    if let Some(comments) = sets.dofs.comments() {
        constraints.insert(0, Keyword::generated(comments));
    }
    sections.extend(generated.sections);
    let mut materials = materials;
    materials.extend(generated.material);
    let young = if model.surface_interactions.is_empty() {
        Default::default()
    } else {
        ContactScale::element_young(model, mesh)
    };
    let interactions = (model.surface_interactions.iter())
        .map(|i| interaction(i, &InteractionDefaults::of(i, mesh, model, &young)))
        .collect();
    let (mut contact_pairs, pair_surfaces) = contact_pairs(&mut sets, model)?;
    contact_pairs.extend(node_ties);
    let initial_conditions = initial_conditions(&mut sets, model)?;
    let design_variables = design_variables(&mut sets, model)?;
    let amplitudes = amplitudes(model)?;
    let flux_kinds = FluxKinds::of(model);
    let steps = model
        .steps
        .iter()
        .enumerate()
        .map(|(s, step)| {
            let context = StepContext {
                stored_perturbation: stored_perturbation(&model.steps[..s]),
                cyclic: (model.ties.iter()).any(|t| t.active && t.cyclic.is_some()),
                cyclic_slaves: &cyclic_slaves,
                extra_boundary: generated.boundary.as_ref(),
                space,
                lines: &lines,
                flux_kinds,
                amplitudes: &model.amplitudes,
                pair_surfaces: &pair_surfaces,
                model,
            };
            write_step(&mut sets, step, &context)
        })
        .collect::<Result<Vec<_>, _>>()?;

    let mut nodes = String::from("*Node\n");
    for (id, &[x, y, z]) in mesh.node_ids().iter().zip(mesh.coords()) {
        let _ = writeln!(nodes, "{id}, {:.8E}, {:.8E}, {:.8E}", x, y, z);
    }
    for (id, [x, y, z]) in &generated.nodes {
        let _ = writeln!(nodes, "{id}, {:.8E}, {:.8E}, {:.8E}", x, y, z);
    }
    let mut element_blocks = elements(mesh, space, &lines);
    element_blocks.extend(generated.elements);
    let node_sets =
        (shortened_names(&sets.shortened).into_iter())
            .chain(sets.node_sets.iter().map(|(name, ids)| {
                Keyword::generated(id_list(&format!("*Nset, Nset={name}"), ids))
            }))
            .collect();
    let element_sets = sets
        .element_sets
        .iter()
        .map(|(name, members)| {
            let header = format!("*Elset, Elset={name}");
            Keyword::generated(match members {
                Members::Ids(ids) => id_list(&header, ids),
                Members::Parts(parts) => format!("{header}\n{}\n", parts.join(",\n")),
            })
        })
        .collect();
    let surfaces = sets
        .surfaces
        .iter()
        .map(|(name, surface)| {
            Keyword::generated(match surface {
                Surface::Faces(faces) => {
                    let mut out = format!("*Surface, Name={name}, Type=Element\n");
                    for (set, face) in faces {
                        let _ = writeln!(out, "{set}, S{face}");
                    }
                    out
                }
                Surface::Nodes(set) => format!("*Surface, Name={name}, Type=Node\n{set}\n"),
            })
        })
        .collect();
    let heading = format!("*Heading\n{}\n", heading.lines().next().unwrap_or_default());
    let mut tree = vec![Keyword::title("Heading", vec![Keyword::generated(heading)])];
    if let Some(submodel) = submodel(model, &sets.submodel_sets)? {
        tree.push(Keyword::title("Submodel", vec![submodel]));
    }
    tree.extend([
        Keyword::title("Nodes", vec![Keyword::generated(nodes)]),
        Keyword::title("Elements", element_blocks),
        Keyword::title("Node sets", node_sets),
        Keyword::title("Element sets", element_sets),
        Keyword::title("Surfaces", surfaces),
        Keyword::title("Physical constants", physical_constants(model)),
        Keyword::title("Coordinate systems", transforms(&sets.transforms)),
        Keyword::title("Materials", materials),
        Keyword::title("Sections", sections),
        Keyword::title("Pre-tension sections", pre_tension_sections),
        Keyword::title("Constraints", constraints),
        Keyword::title("Surface interactions", interactions),
        Keyword::title("Contact pairs", contact_pairs),
        Keyword::title("Amplitudes", amplitudes),
        Keyword::title("Initial conditions", initial_conditions),
    ]);
    if !design_variables.is_empty() {
        tree.push(Keyword::title("Design variables", design_variables));
    }
    let mut steps = Keyword::title("Steps", steps);
    if let Some(boundaries) = boundaries_before_stored_modes(model, &mut steps) {
        tree.push(Keyword::title("Boundary conditions", boundaries));
    }
    tree.push(steps);
    Ok(tree)
}

/// The boundary conditions of a first step that works on the eigenmodes of another
/// analysis, taken out of the step: CalculiX allows no new boundary conditions in a modal
/// dynamics or steady state dynamics step, so they go into the model definition, where the
/// run that stored the eigenmodes had them before its frequency step.
fn boundaries_before_stored_modes(model: &FeModel, steps: &mut Keyword) -> Option<Vec<Keyword>> {
    model.properties.stored_modes.as_ref()?;
    let first = model.steps.iter().position(|s| s.active)?;
    if !model.steps[first].kind.uses_stored_modes() {
        return None;
    }
    let block = steps.children.get_mut(first)?.children.first_mut()?;
    let boundaries = (block.children.iter_mut())
        .find(|k| matches!(&k.kind, KeywordKind::Title(t) if t == "Boundary conditions"))?;
    Some(std::mem::take(&mut boundaries.children))
}

/// A local coordinate system of a node set.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Transform {
    /// Directions x and y.
    Rectangular([f64; 3], [f64; 3]),
    /// Two points on the axis, z from the first to the second: radial, tangential, axial.
    Cylindrical([f64; 3], [f64; 3]),
}

/// `*Transform` of each node set that a skewed symmetry plane or a displacement in a
/// coordinate system of its own holds.
fn transforms(transforms: &[(String, Transform)]) -> Vec<Keyword> {
    (transforms.iter())
        .map(|(set, transform)| {
            let (kind, a, b) = match transform {
                Transform::Rectangular(a, b) => ("R", a, b),
                Transform::Cylindrical(a, b) => ("C", a, b),
            };
            let values: Vec<String> = a.iter().chain(b).map(|&v| number(v)).collect();
            Keyword::generated(format!(
                "*Transform, Nset={set}, Type={kind}\n{}\n",
                values.join(", ")
            ))
        })
        .collect()
}

/// `*SUBMODEL` with the node sets of the submodel boundary conditions, as PrePoMax writes it
/// after the heading. The global results file is named without its directory; it has to be
/// next to the input file, where the analysis and the export put it.
fn submodel(
    model: &FeModel,
    node_sets: &[(String, String)],
) -> Result<Option<Keyword>, WriteError> {
    let Some((_, first)) = node_sets.first() else {
        return Ok(None);
    };
    let input = (model.properties.submodel_input())
        .and_then(|path| path.file_name())
        .ok_or_else(|| WriteError::NoGlobalResults {
            item: first.clone(),
        })?;
    let mut out = format!(
        "*Submodel, Type=Node, Input=\"{}\"\n",
        input.to_string_lossy()
    );
    for (set, _) in node_sets {
        let _ = writeln!(out, "{set}");
    }
    Ok(Some(Keyword::generated(out)))
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
        // its rotations back.
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

/// One `*Element` block per part and element type; the part name is the element set.
/// Surface elements get the type of the model space, e.g. `CAX6` in an axisymmetric model,
/// line elements the type of their beam or truss section.
fn elements(mesh: &FeMesh, space: ModelSpace, lines: &LineElements) -> Vec<Keyword> {
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
    let mut blocks = Vec::new();
    let mut user_element = false;
    for (set, type_name, mut ids) in groups {
        let mut out = String::new();
        // General beam sections exist for CalculiX's user element U1 only, which the input
        // declares before its first element.
        if type_name == "U1" && !std::mem::replace(&mut user_element, true) {
            out.push_str("*User element, Type=U1, Nodes=2, Integration points=2, Maxdof=6\n");
        }
        out += &match set {
            Some(set) => format!("*Element, Type={type_name}, Elset={set}\n"),
            None => format!("*Element, Type={type_name}\n"),
        };
        ids.sort_unstable();
        for element in ids.iter().filter_map(|&id| mesh.element(id)) {
            let _ = write!(out, "{}", element.id);
            let (_, nodes) = lines.written(element, space);
            // A line holds 16 entries; the rest continues on the next line.
            for (entry, node) in (2..).zip(&nodes) {
                let separator = if entry == 17 { ",\n" } else { ", " };
                let _ = write!(out, "{separator}{node}");
            }
            out.push('\n');
        }
        blocks.push(Keyword::generated(out));
    }
    blocks
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
    reference_nodes: BTreeMap<String, ReferenceNodes>,
    /// Reference and rotation node of the point of each displacement written with a point,
    /// with its region; boundary conditions with the same region and point share it.
    remote_points: Vec<(Region, plx_model::Displacement, [NodeId; 2])>,
    /// Reference and rotation node of the point of each displacement written with a point by
    /// the boundary condition's name, with its behavior.
    remote_names: BTreeMap<String, ([NodeId; 2], RemoteBehavior)>,
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
    reference: NodeId,
    rotation: NodeId,
    set: Option<String>,
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
            let extra: Vec<ElementId> = ids
                .iter()
                .copied()
                .filter(|id| part.is_none_or(|p| !p.elements.contains(id)))
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

    /// Next free name `<prefix>-<n>_<postfix>`, like PrePoMax's `GetNextNumberedKey`.
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

    /// An element surface as PrePoMax builds it: one element set per face number and a node
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
    fn reference_nodes(&mut self, item: &str, point: &str) -> Result<ReferenceNodes, WriteError> {
        let Some(nodes) = self.reference_nodes.get(point).cloned() else {
            return Err(WriteError::UnknownReferencePoint {
                item: item.to_string(),
                point: point.to_string(),
            });
        };
        if nodes.set.is_none() {
            let set = self.free_name("Internal_Selection", &name(point));
            self.node_sets.push((set.clone(), vec![nodes.reference]));
            self.reference_nodes.get_mut(point).unwrap().set = Some(set);
        }
        Ok(self.reference_nodes[point].clone())
    }

    /// Node set holding the nodes of a region.
    fn node_set(&mut self, item: &str, region: &Region) -> Result<String, WriteError> {
        match region {
            Region::NodeSet(set) => return Ok(set.clone()),
            Region::ReferencePoint(point) => {
                return Ok(self.reference_nodes(item, point)?.set.unwrap());
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
    /// picked faces and named like PrePoMax's `Internal_Selection-1_Tie-1_Master`.
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

fn materials(model: &FeModel) -> Vec<Keyword> {
    model
        .materials
        .iter()
        .map(|material| {
            let mut properties = Vec::new();
            if let Some(density) = material.density {
                properties.push(Keyword::generated(format!(
                    "*Density\n{}\n",
                    number(density)
                )));
            }
            if let Some(elastic) = material.elastic {
                let (young, poisson) = (number(elastic.young), number(elastic.poisson));
                properties.push(Keyword::generated(format!(
                    "*Elastic\n{young}, {poisson}\n"
                )));
            }
            if let Some(plastic) = &material.plastic {
                let mut out = format!("*Plastic, Hardening={}\n", plastic.hardening.keyword());
                for point in &plastic.points {
                    let _ = writeln!(
                        out,
                        "{}, {}, {}",
                        number(point.stress),
                        number(point.plastic_strain),
                        number(point.temperature)
                    );
                }
                properties.push(Keyword::generated(out));
            }
            // CalculiX reads the creep law after the elasticity and the plasticity.
            if let Some(creep) = material.creep {
                properties.push(Keyword::generated(format!(
                    "*Creep, Law=Norton\n{}, {}, {}\n",
                    number(creep.a),
                    number(creep.n),
                    number(creep.m)
                )));
            }
            // Thermal properties in PrePoMax's order.
            if let Some(expansion) = material.expansion {
                let mut out = String::from("*Expansion");
                if expansion.zero_temperature != 0.0 {
                    let _ = write!(out, ", Zero={}", number(expansion.zero_temperature));
                }
                let _ = writeln!(out, "\n{}", number(expansion.coefficient));
                properties.push(Keyword::generated(out));
            }
            if let Some(conductivity) = material.conductivity {
                properties.push(Keyword::generated(format!(
                    "*Conductivity\n{}\n",
                    number(conductivity)
                )));
            }
            if let Some(specific_heat) = material.specific_heat {
                properties.push(Keyword::generated(format!(
                    "*Specific heat\n{}\n",
                    number(specific_heat)
                )));
            }
            let header = format!("*Material, Name={}\n", name(&material.name));
            Keyword::parent(header, properties)
        })
        .collect()
}

/// `*Physical constants` as PrePoMax's `CalPhysicalConstants` writes it, if any is defined.
fn physical_constants(model: &FeModel) -> Vec<Keyword> {
    let properties = &model.properties;
    let mut out = String::from("*Physical constants");
    if let Some(zero) = properties.absolute_zero {
        let _ = write!(out, ", Absolute zero={}", number(zero));
    }
    if let Some(sigma) = properties.stefan_boltzmann {
        let _ = write!(out, ", Stefan Boltzmann={}", number(sigma));
    }
    if properties.absolute_zero.is_none() && properties.stefan_boltzmann.is_none() {
        return Vec::new();
    }
    out.push('\n');
    vec![Keyword::generated(out)]
}

/// `*Amplitude` as PrePoMax's `CalAmplitude` writes it: four points per line, the most
/// CalculiX reads on one.
fn amplitudes(model: &FeModel) -> Result<Vec<Keyword>, WriteError> {
    let mut keywords = Vec::new();
    for amplitude in &model.amplitudes {
        if let Some(reason) = amplitude.points_problem() {
            return Err(WriteError::InvalidAmplitude {
                item: amplitude.name.clone(),
                reason,
            });
        }
        let mut out = format!("*Amplitude, Name={}", name(&amplitude.name));
        if amplitude.time_span == AmplitudeTime::Total {
            out.push_str(", Time=Total time");
        }
        if amplitude.shift_time != 0.0 {
            let _ = write!(out, ", Shiftx={}", number(amplitude.shift_time));
        }
        if amplitude.shift_amplitude != 0.0 {
            let _ = write!(out, ", Shifty={}", number(amplitude.shift_amplitude));
        }
        out.push('\n');
        for line in amplitude.points.chunks(4) {
            let pairs: Vec<String> = line
                .iter()
                .map(|[t, a]| format!("{}, {}", number(*t), number(*a)))
                .collect();
            let _ = writeln!(out, "{}", pairs.join(", "));
        }
        keywords.push(Keyword::generated(out));
    }
    Ok(keywords)
}

/// The amplitude parameter of a boundary condition or load, such as `, Amplitude=Ramp`;
/// empty without one. `parameter` is the name CalculiX gives it on the keyword.
fn amplitude_parameter(
    amplitudes: &[Amplitude],
    item: &str,
    parameter: &str,
    reference: &Option<String>,
) -> Result<String, WriteError> {
    let Some(reference) = reference else {
        return Ok(String::new());
    };
    if !amplitudes.iter().any(|a| &a.name == reference) {
        return Err(WriteError::UnknownAmplitude {
            item: item.to_owned(),
            amplitude: reference.clone(),
        });
    }
    Ok(format!(", {parameter}={}", name(reference)))
}

/// The distribution an item scales its value by, ready to sample; `None` without one.
fn distribution(
    model: &FeModel,
    item: &str,
    reference: &Option<String>,
) -> Result<Option<Sampler>, WriteError> {
    let Some(name) = reference else {
        return Ok(None);
    };
    let Some(distribution) = model.distribution(name) else {
        return Err(WriteError::UnknownDistribution {
            item: item.to_owned(),
            distribution: name.clone(),
        });
    };
    (distribution.sampler(model.properties.units))
        .map(Some)
        .map_err(|reason| WriteError::InvalidDistribution {
            item: item.to_owned(),
            distribution: name.clone(),
            reason,
        })
}

/// One line per node of `nodes` with the value scaled by the distribution at the node, after
/// the node and `columns` (such as `11, 11, ` for a temperature boundary condition).
fn distributed_nodes(
    out: &mut String,
    mesh: &FeMesh,
    nodes: &[NodeId],
    columns: &str,
    value: f64,
    sampler: &Sampler,
    units: UnitSystem,
) {
    for &node in nodes {
        if let Some(position) = mesh.node(node) {
            let scaled = sampler.scale(value, Quantity::Temperature, units, position);
            let _ = writeln!(out, "{node}, {columns}{}", number(scaled));
        }
    }
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

/// Initial temperatures and velocities as PrePoMax's `CalInitialTemperature`,
/// `CalInitialTranslationalVelocity` and `CalInitialAngularVelocity` write them: one line
/// per set or node and non-zero component, 2D models without the third.
fn initial_conditions(sets: &mut Sets, model: &FeModel) -> Result<Vec<Keyword>, WriteError> {
    let components = if model.properties.space.is_2d() { 2 } else { 3 };
    let mut keywords = Vec::new();
    for condition in &model.initial_conditions {
        if !condition.active {
            keywords.push(deactivated(&condition.name));
            continue;
        }
        let mut out = format!("** Name: {}\n", condition.name);
        let velocity_lines = |out: &mut String, target: &str, v: [f64; 3]| {
            for (dof, value) in v.iter().enumerate().take(components) {
                if *value != 0.0 {
                    out.push_str(&format!("{target}, {}, {}\n", dof + 1, number(*value)));
                }
            }
        };
        match condition.kind {
            InitialConditionKind::Temperature(t) => {
                match distribution(model, &condition.name, &condition.distribution)? {
                    Some(sampler) => {
                        let nodes = condition.region.nodes(sets.mesh);
                        if nodes.is_empty() {
                            return Err(empty(&condition.name, "nodes"));
                        }
                        out.push_str("*Initial conditions, Type=Temperature\n");
                        let units = model.properties.units;
                        distributed_nodes(&mut out, sets.mesh, &nodes, "", t, &sampler, units);
                    }
                    None => {
                        let set = sets.node_set(&condition.name, &condition.region)?;
                        out.push_str(&format!(
                            "*Initial conditions, Type=Temperature\n{set}, {}\n",
                            number(t)
                        ));
                    }
                }
            }
            InitialConditionKind::Velocity(v) => {
                let set = sets.node_set(&condition.name, &condition.region)?;
                out.push_str("*Initial conditions, Type=Velocity\n");
                velocity_lines(&mut out, &set, v);
            }
            InitialConditionKind::AngularVelocity { .. } => {
                let nodes = condition.region.nodes(sets.mesh);
                if nodes.is_empty() {
                    return Err(empty(&condition.name, "nodes"));
                }
                out.push_str("*Initial conditions, Type=Velocity\n");
                for id in nodes {
                    let Some(position) = sets.mesh.node(id) else {
                        continue;
                    };
                    let v = condition.kind.velocity_at(position).unwrap_or_default();
                    velocity_lines(&mut out, &id.to_string(), v);
                }
            }
        }
        keywords.push(Keyword::generated(out));
    }
    Ok(keywords)
}

fn sections(sets: &mut Sets, model: &FeModel) -> Result<Vec<Keyword>, WriteError> {
    let mut keywords = Vec::new();
    for section in &model.sections {
        if !model.materials.iter().any(|m| m.name == section.material) {
            return Err(WriteError::UnknownMaterial {
                item: section.name.clone(),
                material: section.material.clone(),
            });
        }
        let material = name(&section.material);
        let mut out = format!("** Name: {}\n", section.name);
        match &section.kind {
            SectionKind::Solid => {
                let set = sets.element_set(&section.name, &section.region)?;
                let _ = writeln!(out, "*Solid section, Elset={set}, Material={material}");
                // Plane stress and plane strain sections have a thickness, as in PrePoMax.
                if model.properties.space.has_thickness() {
                    let _ = writeln!(out, "{}", number(section.thickness));
                }
            }
            SectionKind::Truss { area } => {
                let set = sets.element_set(&section.name, &section.region)?;
                let _ = writeln!(out, "*Solid section, Elset={set}, Material={material}");
                let _ = writeln!(out, "{}", number(*area));
            }
            SectionKind::Shell { thickness, offset }
            | SectionKind::Membrane { thickness, offset } => {
                let set = sets.element_set(&section.name, &section.region)?;
                let keyword = match section.kind {
                    SectionKind::Membrane { .. } => "Membrane",
                    _ => "Shell",
                };
                let _ = write!(out, "*{keyword} section, Elset={set}, Material={material}");
                if *offset != 0.0 {
                    let _ = write!(out, ", Offset={}", number(*offset));
                }
                let _ = writeln!(out, "\n{}", number(*thickness));
            }
            SectionKind::Beam(beam) => {
                let profile = profile_of(model, &section.name, beam)?;
                let _ = writeln!(out, "** Cross-Section: {}", beam.cross_section);
                let mut options = format!("Section={}", profile.keyword());
                for (k, offset) in (1..).zip(beam.offset) {
                    if offset != 0.0 {
                        let _ = write!(options, ", Offset{k}={}", number(offset));
                    }
                }
                let dimensions: Vec<String> =
                    profile.data_line().iter().map(|&v| number(v)).collect();
                // One keyword per normal: CalculiX takes one normal per section, and an
                // automatic orientation may differ between the beams of the region.
                for (normal, drawn, set) in beam_groups(sets, section, &profile)? {
                    if !profile.is_native() {
                        // CalculiX knows a general section by its values only: the profile
                        // and the normal it is drawn with, for the results.
                        let _ = writeln!(out, "{}", profile_comment(&profile, drawn));
                    }
                    let _ = writeln!(
                        out,
                        "*Beam section, Elset={set}, Material={material}, {options}"
                    );
                    let _ = writeln!(out, "{}", dimensions.join(", "));
                    let _ = writeln!(
                        out,
                        "{}, {}, {}",
                        number(normal[0]),
                        number(normal[1]),
                        number(normal[2])
                    );
                }
            }
        }
        keywords.push(Keyword::generated(out));
    }
    Ok(keywords)
}

/// The profile of a beam section's cross-section.
fn profile_of(
    model: &FeModel,
    section: &str,
    beam: &BeamSection,
) -> Result<BeamProfile, WriteError> {
    let cross_section = model.cross_section(&beam.cross_section);
    (cross_section.map(|c| c.profile.clone())).ok_or_else(|| WriteError::InvalidSection {
        item: section.to_owned(),
        reason: format!("Cross-Section {} does not exist", beam.cross_section),
    })
}

/// The comment before the `*Beam section` of a general profile: the profile, in RON, and
/// the normal it is drawn with, which [`read_inp`](super::read_inp) takes back.
fn profile_comment(profile: &BeamProfile, normal: [f64; 3]) -> String {
    let ron = ron::to_string(profile).unwrap_or_default();
    let normal = normal.map(number).join(", ");
    format!("{}{ron}; Normal: {normal}", super::PROFILE_COMMENT)
}

/// The normal a group of beams is written with, the one its profile is drawn with, and its
/// element set.
type BeamGroup = ([f64; 3], [f64; 3], String);

/// The beams of a section grouped by the normal they are written with, each group as an
/// element set with that normal and the one the profile is drawn with. A region whose beams
/// all share one normal keeps its own set. A general profile is written about its principal
/// axes: its normal is turned about the beam by [`BeamProfile::principal_turn`].
fn beam_groups(
    sets: &mut Sets,
    section: &Section,
    profile: &BeamProfile,
) -> Result<Vec<BeamGroup>, WriteError> {
    let SectionKind::Beam(beam) = &section.kind else {
        return Ok(Vec::new());
    };
    let mesh = sets.mesh;
    let turn = profile.principal_turn();
    let mut groups: Vec<([f64; 3], [f64; 3], Vec<ElementId>)> = Vec::new();
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
        match groups
            .iter_mut()
            .find(|(n, d, _)| *n == normal && *d == drawn)
        {
            Some((_, _, ids)) => ids.push(element.id),
            None => groups.push((normal, drawn, vec![element.id])),
        }
    }
    if groups.is_empty() {
        return Err(empty(&section.name, "line elements"));
    }
    // CalculiX 2.21 reads the element set of a pipe, box or general section 20 characters
    // wide and fails on longer names, so those get short sets of their own.
    let short = profile.needs_reduced_integration() || !profile.is_native();
    if groups.len() == 1 && !short {
        let set = sets.element_set(&section.name, &section.region)?;
        return Ok(vec![(groups[0].0, groups[0].1, set)]);
    }
    let mut named = Vec::new();
    for (normal, drawn, ids) in groups {
        let set = if short {
            sets.unique_numbered("Beam")
        } else {
            sets.free_name("Internal_Selection", &name(&section.name))
        };
        sets.element_sets.push((set.clone(), Members::Ids(ids)));
        named.push((normal, drawn, set));
    }
    Ok(named)
}

/// Tie constraints as PrePoMax's `CalTie` writes them: slave surface first. Springs and
/// supports are written as elements, see [`constraints::springs`].
fn constraints(
    sets: &mut Sets,
    model: &FeModel,
    generated: &mut constraints::Generated,
) -> Result<Vec<Keyword>, WriteError> {
    let mut keywords = Vec::new();
    let mut next_node = (sets.mesh.node_ids().iter().copied())
        .chain(generated.nodes.iter().map(|(id, _)| *id))
        .max()
        .map_or(1, |n| n + 1);
    for constraint in &model.constraints {
        if !constraint.active() {
            keywords.push(deactivated(constraint.name()));
            continue;
        }
        match constraint {
            Constraint::Spring(_) | Constraint::CompressionOnly(_) => {}
            // Moved to the model's node ties when the project was read.
            Constraint::NodeTie(_) => {}
            Constraint::RigidBody(body) => {
                let Some(point) = model.reference_point(&body.reference_point) else {
                    return Err(WriteError::UnknownReferencePoint {
                        item: body.name.clone(),
                        point: body.reference_point.clone(),
                    });
                };
                // Both nodes sit on the reference point, as PrePoMax writes them.
                let (reference, rotation) = (next_node, next_node + 1);
                next_node += 2;
                generated.nodes.push((reference, point.position));
                generated.nodes.push((rotation, point.position));
                sets.reference_nodes.insert(
                    body.reference_point.clone(),
                    ReferenceNodes {
                        reference,
                        rotation,
                        set: None,
                    },
                );
                let mut text = format!("** Name: {}\n", body.name);
                rigid_body(
                    &mut text,
                    sets,
                    &body.name,
                    &body.region,
                    point.position,
                    None,
                    [reference, rotation],
                )?;
                keywords.push(Keyword::generated(text));
            }
            Constraint::Tie(t) => keywords.push(tie(sets, t)?),
        }
    }
    // A displacement that needs a point drives its region from it, by default at the centre
    // of the region; the point exists only in the input file. Its reference node takes the
    // translations, its rotation node the rotations as displacements, both in the directions
    // of the displacement's coordinate system. Nodes that boundary conditions hold directly
    // stay out of the coupling: CalculiX refuses them there.
    let mut excluded = None;
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
                (sets.remote_names.entry(bc.name.clone())).or_insert((nodes, behavior));
                continue;
            }
            let excluded = excluded.get_or_insert_with(|| model.directly_held_nodes(sets.mesh));
            let point =
                (remote.resolve(model, sets.mesh, &bc.region, excluded)).map_err(|reason| {
                    WriteError::InvalidBoundary {
                        item: bc.name.clone(),
                        reason,
                    }
                })?;
            let (reference, rotation) = (next_node, next_node + 1);
            next_node += 2;
            generated.nodes.push((reference, point.position));
            generated.nodes.push((rotation, point.position));
            sets.remote_points
                .push((bc.region.clone(), remote.clone(), [reference, rotation]));
            (sets.remote_names.entry(bc.name.clone())).or_insert(([reference, rotation], behavior));
            if let Some([x, y, _]) = point.axes {
                let set =
                    sets.free_name("Internal_Selection", &format!("{}_Point", name(&bc.name)));
                sets.node_sets
                    .push((set.clone(), vec![reference, rotation]));
                sets.transforms.push((set, Transform::Rectangular(x, y)));
            }
            let mut text = format!("** Name: {}: Remote point\n", bc.name);
            match behavior {
                RemoteBehavior::Rigid => {
                    let whole = remote.pinball.is_none()
                        && point.nodes.len() == bc.region.nodes(sets.mesh).len();
                    let region = if whole {
                        bc.region.clone()
                    } else {
                        Region::Nodes(point.nodes.clone())
                    };
                    rigid_body(
                        &mut text,
                        sets,
                        &bc.name,
                        &region,
                        point.position,
                        point.axes,
                        [reference, rotation],
                    )?;
                }
                RemoteBehavior::Deformable => {
                    let space = model.properties.space;
                    for equation in point.equations(sets.mesh, space) {
                        let mut terms = remote_terms(&equation, point.axes, [reference, rotation]);
                        let mesh = sets.mesh;
                        if (sets.dofs).claim_equation(&bc.name, &mut terms, |n| {
                            n != reference && n != rotation && mesh.node(n).is_some()
                        }) {
                            equation_text(&mut text, &terms);
                        }
                    }
                }
            }
            keywords.push(Keyword::generated(text));
        }
    }
    for t in &model.ties {
        if t.active {
            keywords.push(tie(sets, t)?);
        } else {
            keywords.push(deactivated(&t.name));
        }
    }
    Ok(keywords)
}

/// `*Tie` of a slave surface glued to its master.
fn tie(sets: &mut Sets, tie: &Tie) -> Result<Keyword, WriteError> {
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
    let mut out = format!("*Tie, Name={}", name(&tie.name));
    if let Some(tolerance) = tie.position_tolerance {
        let _ = write!(out, ", Position tolerance={}", number(tolerance));
    }
    if !tie.adjust {
        out.push_str(", Adjust=No");
    }
    if tie.cyclic.is_some() {
        out.push_str(", Cyclic symmetry");
    }
    let _ = writeln!(out, "\n{slave}, {master}");
    if let Some(cyclic) = &tie.cyclic {
        let sectors = (tie.cyclic_sectors(sets.mesh))
            .unwrap_or_else(|| Err("not a cyclic symmetry".into()))
            .map_err(|reason| WriteError::InvalidCyclicSymmetry {
                item: tie.name.clone(),
                reason,
            })?;
        // As PrePoMax's CalCyclicSymmetry: the axis by a point and a second point on it.
        let _ = write!(
            out,
            "*Cyclic symmetry model, N={sectors}, Tie={}",
            name(&tie.name)
        );
        if cyclic.display == SectorDisplay::All {
            let _ = write!(out, ", Ngraph={sectors}");
        }
        let [x, y, z] = cyclic.axis_point;
        let [dx, dy, dz] = cyclic.axis_direction;
        let points = [x, y, z, x + dx, y + dy, z + dz].map(number);
        let _ = writeln!(out, "\n{}", points.join(", "));
    }
    Ok(Keyword::generated(out))
}

/// Nodes on the slave side of an active cyclic symmetry and not on its master side (the
/// axis). CalculiX computes them from the master side and stops when a boundary condition
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

/// One cut per pre-tension load, written as the `*Equation`s that tie its two sides to a new
/// node carrying the preload (see [`pre_tension`]): loads of the same name in several steps
/// share the cut, as the bolt is one.
fn pre_tension_sections(
    sets: &mut Sets,
    model: &FeModel,
    cuts: &BTreeMap<String, pre_tension::Cut>,
    generated: &mut constraints::Generated,
) -> Vec<Keyword> {
    let mut keywords = Vec::new();
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
            // The node sits at the centre of the cut; CalculiX does not use its position.
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
            let mut out = format!("** Name: {}\n", load.name);
            let mesh = sets.mesh;
            for mut equation in pre_tension::equations(cut, node) {
                if (sets.dofs).claim_equation(&load.name, &mut equation, |n| {
                    n != node && mesh.node(n).is_some()
                }) {
                    equation_text(&mut out, &equation);
                }
            }
            keywords.push(Keyword::generated(out));
        }
    }
    keywords
}

/// The node ties, listed with the contact pairs.
fn node_ties(
    sets: &mut Sets,
    model: &FeModel,
    lines: &LineElements,
) -> Result<Vec<Keyword>, WriteError> {
    model
        .node_ties
        .iter()
        .map(|tie| {
            if tie.active {
                node_tie(sets, tie, lines)
            } else {
                Ok(deactivated(&tie.name))
            }
        })
        .collect()
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
/// one shared node, which [`node_merges`] put into the mesh already; the comment names it.
fn node_tie(sets: &mut Sets, tie: &NodeTie, lines: &LineElements) -> Result<Keyword, WriteError> {
    let nodes = tie.region.nodes(sets.mesh);
    let Some((&first, rest)) = nodes.split_first() else {
        return Err(WriteError::EmptyRegion {
            item: tie.name.clone(),
            what: "nodes",
        });
    };
    let mut out = format!("** Name: {}\n", tie.name);
    if !hinge_between_beams(tie, &nodes, lines) {
        let _ = writeln!(out, "** Node {first} (merged)");
        return Ok(Keyword::generated(out));
    }
    for &node in rest {
        for dof in 1..=3 {
            let mut terms = [(node, dof, 1.0), (first, dof, -1.0)];
            if sets.dofs.claim_equation(&tie.name, &mut terms, |_| true) {
                equation_text(&mut out, &terms);
            }
        }
    }
    Ok(Keyword::generated(out))
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
            .filter(|p| p.active && p.interaction == interaction.name)
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
                .and_then(|m| m.elastic)
                .map(|e| e.young / 2.0),
        };
        Self {
            hard_as_linear,
            true_hard,
            stick_slope,
        }
    }
}

/// A surface interaction with its models as children, like PrePoMax's
/// `CalSurfaceInteraction`, with the defaults filled in.
fn interaction(interaction: &SurfaceInteraction, defaults: &InteractionDefaults) -> Keyword {
    let properties = (interaction.properties.iter())
        .filter(|property| {
            !(defaults.true_hard
                && matches!(
                    property,
                    InteractionProperty::SurfaceBehavior(SurfaceBehavior::Hard)
                ))
        })
        .map(|property| {
            let mut out = String::new();
            match property {
                InteractionProperty::SurfaceBehavior(SurfaceBehavior::Hard)
                    if defaults.hard_as_linear.is_some() =>
                {
                    let (k, sigma_inf, note) = defaults.hard_as_linear.as_ref().unwrap();
                    let _ = writeln!(
                        out,
                        "{note}\n*Surface behavior, Pressure-overclosure=Linear\n{}, {}",
                        number(*k),
                        number(*sigma_inf)
                    );
                }
                InteractionProperty::SurfaceBehavior(behavior) => {
                    let kind = behavior.keyword();
                    let _ = writeln!(out, "*Surface behavior, Pressure-overclosure={kind}");
                    match behavior {
                        SurfaceBehavior::Hard => {}
                        SurfaceBehavior::Linear { k, sigma_inf, c0 } => {
                            let _ = write!(out, "{}, {}", number(*k), number(*sigma_inf));
                            if let Some(c0) = c0 {
                                let _ = write!(out, ", {}", number(*c0));
                            }
                            out.push('\n');
                        }
                        SurfaceBehavior::Exponential { c0, p0 } => {
                            let _ = writeln!(out, "{}, {}", number(*c0), number(*p0));
                        }
                        SurfaceBehavior::Tabular(rows) => {
                            for [pressure, overclosure] in rows {
                                let _ = writeln!(
                                    out,
                                    "{}, {}",
                                    number(*pressure),
                                    number(*overclosure)
                                );
                            }
                        }
                        SurfaceBehavior::Tied { k } => {
                            let _ = writeln!(out, "{}", number(*k));
                        }
                    }
                }
                InteractionProperty::Friction(friction) => {
                    let _ = write!(out, "*Friction\n{}", number(friction.coefficient));
                    if let Some(slope) = friction.stick_slope.or(defaults.stick_slope) {
                        let _ = write!(out, ", {}", number(slope));
                    }
                    out.push('\n');
                }
                InteractionProperty::GapConductance(conductance) => {
                    out.push_str("*Gap conductance\n");
                    match conductance {
                        GapConductance::Constant(value) => {
                            let _ = writeln!(out, "{}", number(*value));
                        }
                        GapConductance::Tabular(rows) => {
                            for row in rows {
                                let row: Vec<String> = row.iter().map(|&v| number(v)).collect();
                                let _ = writeln!(out, "{}", row.join(", "));
                            }
                        }
                    }
                }
            }
            Keyword::generated(out)
        })
        .collect();
    let header = format!("*Surface interaction, Name={}\n", name(&interaction.name));
    Keyword::parent(header, properties)
}

/// Contact pairs as PrePoMax's `CalContactPair` writes them: slave surface first.
/// Master and slave surface of each active contact pair, by the pair's name.
type PairSurfaces = BTreeMap<String, (String, String)>;

fn contact_pairs(
    sets: &mut Sets,
    model: &FeModel,
) -> Result<(Vec<Keyword>, PairSurfaces), WriteError> {
    let mut keywords = Vec::new();
    let mut surfaces = PairSurfaces::new();
    for pair in &model.contact_pairs {
        if !pair.active {
            keywords.push(deactivated(&pair.name));
            continue;
        }
        if !(model.surface_interactions.iter()).any(|s| s.name == pair.interaction) {
            return Err(WriteError::UnknownInteraction {
                item: pair.name.clone(),
                interaction: pair.interaction.clone(),
            });
        }
        let master = sets.surface(&pair.name, "Master", &pair.master)?;
        let slave = sets.surface(&pair.name, "Slave", &pair.slave)?;
        keywords.push(Keyword::generated(contact_pair(pair, &master, &slave)));
        surfaces.insert(pair.name.clone(), (master, slave));
    }
    Ok((keywords, surfaces))
}

fn contact_pair(pair: &ContactPair, master: &str, slave: &str) -> String {
    let mut out = format!(
        "** Name: {}\n*Contact pair, Interaction={}, Type={}",
        pair.name,
        name(&pair.interaction),
        pair.method.name()
    );
    if pair.method == ContactMethod::NodeToSurface && pair.small_sliding {
        out.push_str(", Small sliding");
    }
    if pair.adjust {
        let size = pair.adjustment_size.unwrap_or(0.0);
        let _ = write!(out, ", Adjust={}", number(size));
    }
    let _ = writeln!(out, "\n{slave}, {master}");
    if let Some(clearance) = pair.clearance {
        let _ = writeln!(
            out,
            "*Clearance, Master={master}, Slave={slave}, Value={}",
            number(clearance)
        );
    }
    out
}

/// What the steps share: the model's settings and the items written before them.
struct StepContext<'a> {
    extra_boundary: Option<&'a Keyword>,
    space: ModelSpace,
    lines: &'a LineElements,
    flux_kinds: FluxKinds,
    amplitudes: &'a [Amplitude],
    pair_surfaces: &'a PairSurfaces,
    /// `*STEP, PERTURBATION` of the frequency step whose stored modes a complex frequency
    /// step uses; CalculiX stops when the two differ.
    stored_perturbation: bool,
    /// The model, for the coordinate systems of moving loads.
    model: &'a FeModel,
    /// The model is a cyclic symmetric sector: frequency steps select the nodal diameters.
    cyclic: bool,
    /// See [`cyclic_slave_nodes`].
    cyclic_slaves: &'a BTreeSet<NodeId>,
}

/// The perturbation flag of the last active frequency step with storage among `before`.
fn stored_perturbation(before: &[Step]) -> bool {
    (before.iter().rev().filter(|s| s.active))
        .find_map(|s| match &s.kind {
            StepKind::Frequency(f) if f.storage => Some(f.perturbation),
            _ => None,
        })
        .unwrap_or(false)
}

/// A step as PrePoMax structures it: the step title holds `*Step`, which holds the procedure
/// and a title for each kind of item, down to the one holding `*End step`.
///
/// Like PrePoMax, a deactivated step or item keeps its place in the file as a comment
/// (`** Name: Fixed-1: Deactivated`), and nothing of it is written, not even its sets.
///
/// A step with moving loads holds one `*Step` per position of the loads, all alike but for
/// the forces of the moving loads; a moving load with fewer positions than another stays at
/// its last one.
fn write_step(sets: &mut Sets, step: &Step, context: &StepContext) -> Result<Keyword, WriteError> {
    if !step.active {
        return Ok(deactivated_step(step));
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
    let count = moving.iter().map(|(_, s, _)| s.len()).max().unwrap_or(1);
    let mut blocks = Vec::with_capacity(count);
    for position in 0..count {
        let mut block = first.clone();
        if !moving.is_empty() {
            let mut comment = format!("** Moving load position {} of {count}", position + 1);
            for (index, stations, _) in &moving {
                let station = stations[position.min(stations.len() - 1)];
                let _ = write!(
                    comment,
                    ", {} at {}",
                    step.loads[*index].name,
                    number(station)
                );
            }
            block.text = format!("{comment}\n{}", block.text);
        }
        if position > 0
            && let Some(loads) = (block.children.iter_mut())
                .find(|k| matches!(&k.kind, KeywordKind::Title(t) if t == "Loads"))
        {
            for (index, _, frames) in &moving {
                let load = &step.loads[*index];
                let frame = &frames[position.min(frames.len() - 1)];
                let text = moving_load_text(load, frame, context)?;
                let marker = format!("** Name: {}", load.name);
                if let Some(keyword) = (loads.children.iter_mut()).find(|k| {
                    k.text
                        .strip_prefix(&marker)
                        .is_some_and(|rest| rest.starts_with(['\n', ':']))
                }) {
                    keyword.text = text;
                }
            }
        }
        blocks.push(block);
    }
    Ok(Keyword::title(&step.name, blocks))
}

/// A rigid body of the nodes of `region` driven by a reference and a rotation node at
/// `position`: a `*Rigid body`, but with the nodes of beams, trusses, shells and membranes
/// rigidly coupled by `*Equation`s. CalculiX 2.23 stops on a rigid body containing such a
/// node ("nodes belonging to beam elements must not be subject to a RIGID MPC"); the
/// equations hold them for small rotations as the rigid body does in a linear analysis.
fn rigid_body(
    text: &mut String,
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
    let rigid = |text: &mut String, set: String| {
        let _ = writeln!(
            text,
            "*Rigid body, Nset={set}, Ref node={reference}, Rot node={rotation}"
        );
    };
    if expanded.is_empty() {
        let set = if whole {
            sets.node_set(item, region)?
        } else {
            sets.node_set(item, &Region::Nodes(others))?
        };
        rigid(text, set);
        return Ok(());
    }
    if !others.is_empty() {
        rigid(text, sets.node_set(item, &Region::Nodes(others))?);
    }
    text.push_str("** Nodes of beams and shells, which CalculiX refuses in a rigid body\n");
    for row in plx_model::rigid_rows(sets.mesh, &expanded, position) {
        equation_text(text, &remote_terms(&row, axes, nodes));
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

/// An `*Equation` of its terms, the dependent one first.
fn equation_text(out: &mut String, terms: &[dofs::Term]) {
    let _ = writeln!(out, "*Equation\n{}", terms.len());
    let terms: Vec<String> = (terms.iter())
        .map(|(node, dof, value)| format!("{node}, {dof}, {}", number(*value)))
        .collect();
    // Four terms (twelve entries) per line, as PrePoMax writes them.
    for chunk in terms.chunks(4) {
        out.push_str(&chunk.join(", "));
        out.push('\n');
    }
}

/// The `*Boundary` lines of prescribed displacements and rotations of the first `dofs`
/// degrees of freedom, a range for each run of equal values. The rotations go to the
/// translations of `rotation`, the rotation node of a rigid body, if there is one.
fn displacement_lines(
    out: &mut String,
    nodes: &str,
    rotation: Option<&str>,
    values: &[Option<f64>; 6],
    dofs: usize,
) {
    // Each held degree of freedom as (target, its number there, value).
    let held = (1..).zip(values).take(dofs).filter_map(|(dof, value)| {
        let value = (*value)?;
        Some(match rotation {
            Some(rotation) if dof > 3 => (rotation, dof - 3, value),
            _ => (nodes, dof, value),
        })
    });
    let mut run: Option<(&str, usize, usize, f64)> = None;
    for (target, dof, value) in held {
        match &mut run {
            Some((t, _, last, v)) if *t == target && *last + 1 == dof && *v == value => {
                *last = dof;
            }
            _ => {
                if let Some((t, first, last, v)) = run {
                    let _ = writeln!(out, "{t}, {first}, {last}, {}", number(v));
                }
                run = Some((target, dof, dof, value));
            }
        }
    }
    if let Some((t, first, last, v)) = run {
        let _ = writeln!(out, "{t}, {first}, {last}, {}", number(v));
    }
}

/// The keyword of a moving load at one position: its nodal forces as `*Cload`.
fn moving_load_text(
    load: &plx_model::Load,
    frame: &MovingFrame,
    context: &StepContext,
) -> Result<String, WriteError> {
    // All wheels off the region: nothing to load, and an empty `*Cload` is no keyword.
    if frame.forces.is_empty() {
        return Ok(format!("** Name: {}: No wheel on the region\n", load.name));
    }
    let dofs = if context.space.is_2d() { 2 } else { 3 };
    let amplitude =
        amplitude_parameter(context.amplitudes, &load.name, "Amplitude", &load.amplitude)?;
    let mut out = format!("** Name: {}\n*Cload{amplitude}\n", load.name);
    for (node, values) in &frame.forces {
        for (dof, value) in (1..).zip(values).take(dofs) {
            if *value != 0.0 {
                let _ = writeln!(out, "{node}, {dof}, {}", number(*value));
            }
        }
    }
    Ok(out)
}

/// The `*Step` of a step and everything in it, with the moving loads at the positions
/// `moving` gives by load index.
fn step_block<'f>(
    sets: &mut Sets,
    step: &Step,
    context: &StepContext,
    moving: impl Fn(usize) -> Option<&'f MovingFrame>,
) -> Result<Keyword, WriteError> {
    let &StepContext {
        extra_boundary,
        space,
        lines,
        flux_kinds,
        amplitudes,
        pair_surfaces,
        stored_perturbation,
        cyclic,
        cyclic_slaves,
        model,
        ..
    } = context;
    // Nodes of 2D models move in the x-y plane only; CalculiX fails on rotations there.
    let dofs = if space.is_2d() { 2 } else { 6 };
    let (header, procedure) = match &step.kind {
        StepKind::Static(settings) => static_step(settings),
        StepKind::Frequency(settings) => {
            let (header, mut procedure) = frequency_step(settings, cyclic);
            if cyclic {
                procedure.push_str(&select_cyclic_symmetry_modes(settings));
            }
            (header, procedure)
        }
        StepKind::ComplexFrequency(settings) => {
            complex_frequency_step(settings, stored_perturbation)
        }
        StepKind::Buckle(settings) => buckle_step(settings),
        StepKind::HeatTransfer(settings) => heat_transfer_step(settings, "*Heat transfer", false),
        StepKind::CoupledTempDisp(settings) => {
            heat_transfer_step(settings, "*Coupled temperature-displacement", true)
        }
        StepKind::UncoupledTempDisp(settings) => {
            heat_transfer_step(settings, "*Uncoupled temperature-displacement", false)
        }
        StepKind::Dynamic(settings) => dynamic_step(settings),
        StepKind::Visco(settings) => visco_step(settings),
        StepKind::ModalDynamics(settings) => modal_dynamics_step(settings),
        StepKind::SteadyStateDynamics(settings) => steady_state_dynamics_step(settings),
        StepKind::Sensitivity(settings) => sensitivity_step(sets, &step.name, settings)?,
    };
    // A modal step keeps the supports of the frequency step that stored the modes;
    // CalculiX refuses new ones ("in a modal dynamic step new SPCs are not allowed"), and
    // the reset would make the same supports new. Writing them again is accepted.
    let mut boundaries = if step.kind.uses_stored_modes() || step.kind.keeps_previous_boundaries() {
        Vec::new()
    } else {
        vec![Keyword::generated("*Boundary, op=New\n".into())]
    };
    for bc in &step.boundary_conditions {
        // A step leaves out what it cannot take, like a deactivated item: displacements in
        // a heat transfer step, temperatures in a static one.
        if !bc.active || !step.kind.supports_boundary(&bc.kind) {
            boundaries.push(deactivated(&bc.name));
            continue;
        }
        let on_point = sets.remote_names.contains_key(&bc.name)
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
            let amplitude = amplitude_parameter(amplitudes, &bc.name, "Amplitude", &bc.amplitude)?;
            // The translations are those of the reference node, the rotations the
            // displacements of the rotation node.
            let mut out = format!("** Name: {}\n*Boundary{amplitude}\n", bc.name);
            let (reference, rotation) = (reference.to_string(), rotation.to_string());
            let held = remote.held(space);
            displacement_lines(&mut out, &reference, Some(&rotation), &held, 6);
            boundaries.push(Keyword::generated(out));
            continue;
        }
        // Truss nodes have no rotations either; CalculiX fails when they are fixed.
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
            let amplitude = amplitude_parameter(amplitudes, &bc.name, "Amplitude", &bc.amplitude)?;
            let mut out = format!("** Name: {}\n*Boundary{amplitude}\n", bc.name);
            if let BoundaryKind::Temperature(t) = bc.kind {
                let units = model.properties.units;
                distributed_nodes(&mut out, sets.mesh, &nodes, "11, 11, ", t, sampler, units);
            }
            boundaries.push(Keyword::generated(out));
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
        // The rotations of a rigid body are the displacements of its rotation node.
        let rotation_node = match bc.region.reference_point() {
            Some(point) => Some(sets.reference_nodes(&bc.name, point)?.rotation),
            None => None,
        };
        // Symmetry supports stay zero; an amplitude would not change them.
        let reference = bc.amplitude.as_ref().filter(|_| bc.kind.takes_amplitude());
        let amplitude =
            amplitude_parameter(amplitudes, &bc.name, "Amplitude", &reference.cloned())?;
        let options = match bc.kind {
            BoundaryKind::Submodel { step, .. } => format!(", Submodel, Step={}", step.max(1)),
            _ => amplitude,
        };
        let mut out = format!("** Name: {}\n*Boundary{options}\n", bc.name);
        match bc.kind {
            BoundaryKind::Displacement(ref displacement) => {
                let values = displacement.values;
                // A coordinate system of its own turns the directions of the nodes.
                if displacement.system != plx_model::GLOBAL && rotation_node.is_none() {
                    let system = (model.coordinate_system_or_global(&displacement.system))
                        .ok_or_else(|| WriteError::InvalidBoundary {
                            item: bc.name.clone(),
                            reason: format!(
                                "Coordinate System {} does not exist",
                                displacement.system
                            ),
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
                let rotation = rotation_node.map(|n| n.to_string());
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
                    displacement_lines(&mut out, &set, rotation.as_deref(), &held, 6);
                } else {
                    displacement_lines(&mut out, &set, rotation.as_deref(), &values, dofs);
                }
            }
            BoundaryKind::Temperature(t) => {
                let _ = writeln!(out, "{set}, 11, 11, {}", number(t));
            }
            // The global results hold displacements, no rotations.
            BoundaryKind::Submodel { dofs: held, .. } => {
                for dof in (1..=dofs.min(3)).filter(|&d| held[d - 1]) {
                    let _ = writeln!(out, "{set}, {dof}, {dof}");
                }
                if !sets.submodel_sets.iter().any(|(s, _)| *s == set) {
                    sets.submodel_sets.push((set, bc.name.clone()));
                }
            }
            BoundaryKind::Symmetry { normal } => {
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
                    let dof = dof + 1;
                    let _ = writeln!(out, "{set}, {dof}, {dof}, 0");
                }
            }
        }
        boundaries.push(Keyword::generated(out));
    }
    // Boundary conditions that constraints need, PrePoMax's additional boundary conditions.
    if step.kind.is_mechanical() {
        boundaries.extend(extra_boundary.cloned());
    }
    let mut loads = Vec::new();
    // Like PrePoMax, a step resets the loads it takes; one that takes no loads gets none
    // written, not even the reset. Distributed fluxes, films and radiation are reset only
    // when the model has them.
    let kind = &step.kind;
    let resets = [
        ("Cload", kind.supports_load(&LoadKind::Pressure(0.0))),
        ("Dload", kind.supports_load(&LoadKind::Pressure(0.0))),
        (
            "Cflux",
            kind.supports_load(&LoadKind::ConcentratedFlux(0.0)),
        ),
        ("Dflux", kind.is_thermal() && flux_kinds.distributed),
        ("Film", kind.is_thermal() && flux_kinds.film),
        ("Radiate", kind.is_thermal() && flux_kinds.radiation),
    ];
    for (keyword, _) in resets.into_iter().filter(|(_, reset)| *reset) {
        loads.push(Keyword::generated(format!("*{keyword}, op=New\n")));
    }
    let step_loads: &[_] = if kind.supports_loads() {
        &step.loads
    } else {
        &[]
    };
    for (index, load) in step_loads.iter().enumerate() {
        if !load.active || !kind.supports_load(&load.kind) {
            loads.push(deactivated(&load.name));
            continue;
        }
        let mut out = format!("** Name: {}\n", load.name);
        let amplitude = amplitude_parameter(amplitudes, &load.name, "Amplitude", &load.amplitude)?;
        // The second amplitude of a film scales its coefficient, of radiation the emissivity.
        let factor = match load.kind {
            LoadKind::Film { .. } => Some("Film amplitude"),
            LoadKind::Radiation { .. } => Some("Radiation amplitude"),
            _ => None,
        };
        let factor_amplitude = match factor {
            Some(parameter) => {
                amplitude_parameter(amplitudes, &load.name, parameter, &load.factor_amplitude)?
            }
            None => String::new(),
        };
        match load.kind {
            LoadKind::Moving(_) => {
                let frame = moving(index).cloned().unwrap_or_default();
                out = moving_load_text(load, &frame, context)?;
            }
            LoadKind::Moment(moment) => {
                // Moments at a reference point act on the rotation node of its rigid body,
                // at nodes of beams and shells on their rotations.
                let (target, first_dof) = match load.region.reference_point() {
                    Some(point) => {
                        let nodes = sets.reference_nodes(&load.name, point)?;
                        (nodes.rotation.to_string(), 1)
                    }
                    None => (sets.node_set(&load.name, &load.region)?, 4),
                };
                let _ = writeln!(out, "*Cload{amplitude}");
                for (dof, value) in (0..).zip(moment) {
                    if value != 0.0 && dof + 4 <= dofs {
                        let _ = writeln!(out, "{target}, {}, {}", first_dof + dof, number(value));
                    }
                }
            }
            LoadKind::Pressure(pressure) => {
                let _ = writeln!(out, "*Dload{amplitude}");
                match distribution(model, &load.name, &load.distribution)? {
                    // Every face takes the pressure scaled at its center.
                    Some(sampler) => {
                        let faces = load.region.faces(sets.mesh);
                        if faces.is_empty() {
                            return Err(empty(&load.name, "element faces"));
                        }
                        for (element, face) in faces {
                            if let Some(center) = face_center(sets.mesh, element, face) {
                                let p = pressure * sampler.factor(center);
                                let _ = writeln!(out, "{element}, P{face}, {}", number(p));
                            }
                        }
                    }
                    None => {
                        for (set, face) in sets.face_sets(&load.name, &load.region)? {
                            let _ = writeln!(out, "{set}, P{face}, {}", number(pressure));
                        }
                    }
                }
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
                let node = sets.pre_tension_nodes[&load.name];
                if lock {
                    let _ = writeln!(out, "*Boundary, Fixed\n{node}, 1, 1");
                } else if by_displacement {
                    let _ = writeln!(out, "*Boundary{amplitude}\n{node}, 1, 1, {}", number(value));
                } else {
                    let _ = writeln!(out, "*Cload{amplitude}\n{node}, 1, {}", number(value));
                }
            }
            LoadKind::Force {
                value: force,
                each_node,
            } => {
                let faces = load.region.faces(sets.mesh);
                let edges = load.region.edges(sets.mesh);
                let axisymmetric = space == ModelSpace::Axisymmetric;
                let _ = writeln!(out, "*Cload{amplitude}");
                // On points the nodes share the force equally, a reference point takes it
                // whole: the share at every node of their set. With `each_node` every node
                // of the region takes the whole force.
                if each_node || (faces.is_empty() && edges.is_empty()) {
                    let count = match load.region.reference_point() {
                        _ if each_node => 1,
                        Some(_) => 1,
                        None => load.region.nodes(sets.mesh).len(),
                    };
                    let set = sets.node_set(&load.name, &load.region)?;
                    for (dof, value) in (1..).zip(force).take(dofs) {
                        if value != 0.0 {
                            let share = value / count as f64;
                            let _ = writeln!(out, "{set}, {dof}, {}", number(share));
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
                        for (dof, value) in (1..).zip(values).take(dofs) {
                            if value != 0.0 {
                                let _ = writeln!(out, "{node}, {dof}, {}", number(value));
                            }
                        }
                    }
                }
            }
            LoadKind::ConcentratedFlux(flux) => {
                let set = sets.node_set(&load.name, &load.region)?;
                let _ = writeln!(out, "*Cflux{amplitude}\n{set}, 11, {}", number(flux));
            }
            LoadKind::SurfaceFlux(flux) => {
                let _ = writeln!(out, "*Dflux{amplitude}");
                for (set, face) in sets.face_sets(&load.name, &load.region)? {
                    let _ = writeln!(out, "{set}, S{face}, {}", number(flux));
                }
            }
            LoadKind::BodyFlux(flux) => {
                let set = sets.element_set(&load.name, &load.region)?;
                let _ = writeln!(out, "*Dflux{amplitude}\n{set}, BF, {}", number(flux));
            }
            LoadKind::Film { sink, coefficient } => {
                let _ = writeln!(out, "*Film{amplitude}{factor_amplitude}");
                for (set, face) in sets.face_sets(&load.name, &load.region)? {
                    let (sink, h) = (number(sink), number(coefficient));
                    let _ = writeln!(out, "{set}, F{face}, {sink}, {h}");
                }
            }
            LoadKind::Radiation { sink, emissivity } => {
                let _ = writeln!(out, "*Radiate{amplitude}{factor_amplitude}");
                for (set, face) in sets.face_sets(&load.name, &load.region)? {
                    let (sink, e) = (number(sink), number(emissivity));
                    let _ = writeln!(out, "{set}, R{face}, {sink}, {e}");
                }
            }
            // As PrePoMax's CalGravityLoad: the size of the acceleration and its direction.
            LoadKind::Gravity(acceleration) => {
                let set = sets.element_set(&load.name, &load.region)?;
                let Some((size, direction)) = unit_vector(acceleration) else {
                    return Err(WriteError::InvalidLoad {
                        item: load.name.clone(),
                        reason: "the gravitational acceleration is zero".into(),
                    });
                };
                let _ = writeln!(out, "*Dload{amplitude}");
                let [x, y, z] = direction.map(number);
                let _ = writeln!(out, "{set}, Grav, {}, {x}, {y}, {z}", number(size));
            }
            // As PrePoMax's CalCentrifLoad: the square of the rotational speed, a point on
            // the axis and the axis direction.
            LoadKind::Centrifugal { point, axis, speed } => {
                let set = sets.element_set(&load.name, &load.region)?;
                let Some((_, direction)) = unit_vector(axis) else {
                    return Err(WriteError::InvalidLoad {
                        item: load.name.clone(),
                        reason: "the rotation axis has no direction".into(),
                    });
                };
                let _ = writeln!(out, "*Dload{amplitude}");
                let [px, py, pz] = point.map(number);
                let [x, y, z] = direction.map(number);
                let _ = writeln!(
                    out,
                    "{set}, Centrif, {}, {px}, {py}, {pz}, {x}, {y}, {z}",
                    number(speed * speed)
                );
            }
        }
        loads.push(Keyword::generated(out));
    }
    let defined_fields = defined_fields(sets, step, amplitudes, model)?;
    let mut history_outputs = Vec::new();
    for output in &step.history_outputs {
        if !output.active {
            history_outputs.push(deactivated(&output.name));
        } else if let Some(keyword) = history_output(sets, output, pair_surfaces)? {
            history_outputs.push(keyword);
        }
    }
    // The force of a bolt is the reaction of its pre-tension node, also while it is locked,
    // and its shortening the node's displacement: shown under the name of the load.
    for load in step.loads.iter().filter(|l| l.active) {
        if let Some(set) = sets.pre_tension_sets.get(&load.name) {
            history_outputs.push(Keyword::generated(format!(
                "** Name: {}\n*Node print, Nset={set}\nRF, U\n",
                load.name
            )));
        }
    }
    // CalculiX writes all results of the step in one form: the node outputs follow the
    // element outputs.
    let dimension = (step.field_outputs.iter())
        .filter(|o| o.active && o.kind == OutputKind::Element)
        .map(|o| o.dimension)
        .find(|&d| d != OutputDimension::Default)
        .unwrap_or_default();
    let field_outputs = (step.field_outputs.iter())
        .filter_map(|output| field_output(output, dimension))
        .collect();
    let end = Keyword::generated("*End step\n".into());
    let mut contents = vec![
        Keyword::generated(procedure),
        Keyword::title("Controls", Vec::new()),
        Keyword::title("Output frequency", Vec::new()),
        Keyword::title("Boundary conditions", boundaries),
        Keyword::title("Loads", loads),
        Keyword::title("Defined fields", defined_fields),
        Keyword::title("History outputs", history_outputs),
        Keyword::title("Field outputs", field_outputs),
        Keyword::title("End step", vec![end]),
    ];
    let changes = contact_changes(step, model, pair_surfaces);
    if !changes.is_empty() {
        contents.insert(3, Keyword::title("Contacts", changes));
    }
    Ok(Keyword::parent(header, contents))
}

/// The contact pairs a step switches off or on (`*Model change, Type=Contact pair`) and
/// whose friction it changes (`*Change friction`, with the stick slope of the interaction).
/// Changes of pairs not written, deactivated or unknown, are left out.
fn contact_changes(step: &Step, model: &FeModel, pair_surfaces: &PairSurfaces) -> Vec<Keyword> {
    let mut keywords = Vec::new();
    for change in &step.contact_changes {
        let Some((master, slave)) = pair_surfaces.get(&change.region) else {
            continue;
        };
        let mut out = format!("** Name: {}\n", change.region);
        if let Some(active) = change.active {
            let action = if active { "Add" } else { "Remove" };
            let _ = writeln!(
                out,
                "*Model change, Type=Contact pair, {action}\n{slave}, {master}"
            );
        }
        let pair = (model.contact_pairs.iter()).find(|p| p.name == change.region);
        let friction = pair
            .and_then(|p| (model.surface_interactions.iter()).find(|i| i.name == p.interaction))
            .and_then(|i| {
                i.properties.iter().find_map(|p| match p {
                    InteractionProperty::Friction(f) => Some((i, f)),
                    _ => None,
                })
            });
        if let (Some(mu), Some((interaction, friction))) = (change.friction, friction) {
            let _ = write!(
                out,
                "*Change friction, Interaction={}\n*Friction\n{}",
                name(&interaction.name),
                number(mu)
            );
            let default = (model.materials.first())
                .and_then(|m| m.elastic)
                .map(|e| e.young / 2.0);
            if let Some(slope) = friction.stick_slope.or(default) {
                let _ = write!(out, ", {}", number(slope));
            }
            out.push('\n');
        }
        keywords.push(Keyword::generated(out));
    }
    keywords
}

/// The defined temperatures of a step as PrePoMax's `CalDefinedTemperature` writes them:
/// `*Temperature` with the node set and the value, or with the result file of a previous
/// analysis and the step to read. CalculiX 2.21 takes the file name with its extension and
/// looks for it next to the input file; `BStep` is the only step parameter it knows.
fn defined_fields(
    sets: &mut Sets,
    step: &Step,
    amplitudes: &[Amplitude],
    model: &FeModel,
) -> Result<Vec<Keyword>, WriteError> {
    let mut keywords = Vec::new();
    for field in &step.defined_fields {
        // A thermal step solves for the temperatures; it leaves the field out like a
        // deactivated one.
        if !field.active || !step.kind.supports_defined_fields() {
            keywords.push(deactivated(&field.name));
            continue;
        }
        let out = match &field.kind {
            DefinedFieldKind::Temperature(t) => {
                let amplitude =
                    amplitude_parameter(amplitudes, &field.name, "Amplitude", &field.amplitude)?;
                let mut out = format!("** Name: {}\n*Temperature{amplitude}\n", field.name);
                match distribution(model, &field.name, &field.distribution)? {
                    Some(sampler) => {
                        let nodes = field.region.nodes(sets.mesh);
                        if nodes.is_empty() {
                            return Err(empty(&field.name, "nodes"));
                        }
                        let units = model.properties.units;
                        distributed_nodes(&mut out, sets.mesh, &nodes, "", *t, &sampler, units);
                    }
                    None => {
                        let set = sets.node_set(&field.name, &field.region)?;
                        let _ = writeln!(out, "{set}, {}", number(*t));
                    }
                }
                out
            }
            DefinedFieldKind::TemperatureFromFile { file, step } => {
                let name = file.file_name().map(|n| n.to_string_lossy());
                let Some(name) = name.filter(|n| !n.is_empty() && !n.contains(',')) else {
                    return Err(WriteError::InvalidResultFile {
                        item: field.name.clone(),
                        file: file.display().to_string(),
                    });
                };
                format!(
                    "** Name: {}\n*Temperature, File={name}, BStep={step}\n",
                    field.name
                )
            }
        };
        keywords.push(Keyword::generated(out));
    }
    Ok(keywords)
}

/// Comment that stands for a deactivated item, PrePoMax's `CalDeactivated`.
fn deactivated(name: &str) -> Keyword {
    Keyword::generated(format!("** Name: {name}: Deactivated\n"))
}

/// A deactivated step as PrePoMax writes it: the titles of the step with a comment for the
/// step, its procedure and each of its items, but no keyword CalculiX would read.
fn deactivated_step(step: &Step) -> Keyword {
    let procedure = match step.kind {
        StepKind::Static(_) => "StaticStep",
        StepKind::Frequency(_) => "FrequencyStep",
        StepKind::ComplexFrequency(_) => "ComplexFrequencyStep",
        StepKind::Buckle(_) => "BuckleStep",
        StepKind::HeatTransfer(_) => "HeatTransferStep",
        StepKind::CoupledTempDisp(_) => "CoupledTempDispStep",
        StepKind::UncoupledTempDisp(_) => "UncoupledTempDispStep",
        StepKind::Dynamic(_) => "DynamicStep",
        StepKind::Visco(_) => "ViscoStep",
        StepKind::ModalDynamics(_) => "ModalDynamicsStep",
        StepKind::SteadyStateDynamics(_) => "SteadyStateDynamicsStep",
        StepKind::Sensitivity(_) => "SensitivityStep",
    };
    fn all<'a>(names: impl Iterator<Item = &'a str>) -> Vec<Keyword> {
        names.map(deactivated).collect()
    }
    let loads = if step.kind.supports_loads() {
        all(step.loads.iter().map(|l| l.name.as_str()))
    } else {
        Vec::new()
    };
    let contents = vec![
        deactivated(procedure),
        Keyword::title(
            "Boundary conditions",
            all(step.boundary_conditions.iter().map(|b| b.name.as_str())),
        ),
        Keyword::title("Loads", loads),
        Keyword::title(
            "Defined fields",
            all(step.defined_fields.iter().map(|f| f.name.as_str())),
        ),
        Keyword::title(
            "History outputs",
            all(step.history_outputs.iter().map(|h| h.name.as_str())),
        ),
        Keyword::title(
            "Field outputs",
            all(step.field_outputs.iter().map(|f| f.name.as_str())),
        ),
        Keyword::title("End step", vec![deactivated(&step.name)]),
    ];
    Keyword::title(
        &step.name,
        vec![Keyword::parent(
            format!("** Name: {}: Deactivated\n", step.name),
            contents,
        )],
    )
}

/// Equivalent nodal forces of a total force spread evenly over element faces, the way
/// PrePoMax turns a surface traction into concentrated loads. Each face takes its share by
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

/// Which kinds of heat loads the model has in any step: PrePoMax resets distributed fluxes,
/// films and radiation only then.
#[derive(Clone, Copy, Debug, Default)]
struct FluxKinds {
    distributed: bool,
    film: bool,
    radiation: bool,
}

impl FluxKinds {
    fn of(model: &FeModel) -> Self {
        let mut kinds = Self::default();
        for load in model.steps.iter().flat_map(|s| &s.loads) {
            match load.kind {
                LoadKind::SurfaceFlux(_) | LoadKind::BodyFlux(_) => kinds.distributed = true,
                LoadKind::Film { .. } => kinds.film = true,
                LoadKind::Radiation { .. } => kinds.radiation = true,
                _ => {}
            }
        }
        kinds
    }
}

/// The `*Step` line and the procedure keyword of a static step.
fn static_step(settings: &StaticStep) -> (String, String) {
    let mut keyword = String::from("*Static");
    if let Some(solver) = settings.solver.keyword() {
        let _ = write!(keyword, ", Solver={solver}");
    }
    incremented_step(settings, keyword, settings.nlgeom)
}

/// The `*Step` line and the procedure of a visco step: a static step whose automatic
/// increments the creep strain tolerance limits.
fn visco_step(settings: &ViscoStep) -> (String, String) {
    let increments = &settings.increments;
    let mut keyword = String::from("*Visco");
    if let Some(solver) = increments.solver.keyword() {
        let _ = write!(keyword, ", Solver={solver}");
    }
    let _ = write!(keyword, ", Cetol={}", number(settings.creep_tolerance));
    incremented_step(increments, keyword, increments.nlgeom)
}

/// The `*Step` line and the procedure of a sensitivity step: one `*Design response` per
/// response, named like it, on all elements of the model, then the optional filter.
fn sensitivity_step(
    sets: &mut Sets,
    step: &str,
    settings: &SensitivityStep,
) -> Result<(String, String), WriteError> {
    let mut procedure = String::from("*Sensitivity\n");
    let responses = settings.responses();
    if responses.iter().any(|r| *r != "EIGENFREQUENCY") {
        let parts = sets.mesh.parts.iter().map(|p| p.name.clone()).collect();
        let elements = sets.element_set(step, &Region::Parts(parts))?;
        for response in &responses {
            let _ = writeln!(
                procedure,
                "*Design response, Name={response}\n{response}, {elements}"
            );
        }
    } else {
        for response in &responses {
            let _ = writeln!(procedure, "*Design response, Name={response}\n{response}");
        }
    }
    if let Some(radius) = settings.filter_radius {
        let _ = writeln!(procedure, "*Filter, Type=Linear\n{}", number(radius));
    }
    Ok(("*Step\n".into(), procedure))
}

/// The design variables of the model (`*Design variables`): the design region of the first
/// active sensitivity step. CalculiX reads them before the steps.
fn design_variables(sets: &mut Sets, model: &FeModel) -> Result<Vec<Keyword>, WriteError> {
    let Some((name, region)) = (model.steps.iter())
        .filter(|s| s.active)
        .find_map(|s| Some((&s.name, s.kind.design_region()?)))
    else {
        return Ok(Vec::new());
    };
    let set = sets.node_set(&format!("{name}_Design"), region)?;
    Ok(vec![Keyword::generated(format!(
        "*Design variables, Type=Coordinate\n{set}\n"
    ))])
}

/// The `*Step` line and the procedure keyword of a heat transfer or coupled step, as
/// PrePoMax's `CalHeatTransferStep` and `CalCoupledTempDispStep` write them.
fn heat_transfer_step(
    settings: &HeatTransferStep,
    keyword: &str,
    coupled: bool,
) -> (String, String) {
    let increments = &settings.increments;
    let mut keyword = keyword.to_string();
    if let Some(solver) = increments.solver.keyword() {
        let _ = write!(keyword, ", Solver={solver}");
    }
    if settings.steady_state {
        keyword.push_str(", Steady state");
    }
    if let Some(deltmx) = settings.deltmx.filter(|_| !settings.steady_state) {
        let _ = write!(keyword, ", Deltmx={}", number(deltmx));
    }
    incremented_step(increments, keyword, coupled && increments.nlgeom)
}

/// The `*Step` line and the procedure keyword of a dynamic step, as PrePoMax's
/// `CalDynamicStep` and `CalDamping` write them: the Rayleigh damping follows the
/// procedure in the step, where CalculiX applies it to the whole model.
fn dynamic_step(settings: &DynamicStep) -> (String, String) {
    let mut keyword = String::from("*Dynamic");
    if let Some(solver) = settings.increments.solver.keyword() {
        let _ = write!(keyword, ", Solver={solver}");
    }
    if settings.alpha != -0.05 {
        let _ = write!(keyword, ", Alpha={}", number(settings.alpha));
    }
    if let Some(explicit) = settings.procedure.keyword() {
        let _ = write!(keyword, ", Explicit={explicit}");
    }
    let (header, mut procedure) =
        incremented_step(&settings.increments, keyword, settings.increments.nlgeom);
    if let Some(damping) = &settings.damping {
        let _ = writeln!(
            procedure,
            "*Damping, Alpha={}, Beta={}",
            number(damping.alpha),
            number(damping.beta)
        );
    }
    (header, procedure)
}

/// The `*Step` line and the procedure keyword of a modal dynamics step, as PrePoMax's
/// `CalModalDynamicsStep` writes them, with the modal damping after the procedure.
fn modal_dynamics_step(settings: &ModalDynamicsStep) -> (String, String) {
    let header = format!("*Step, Inc={}\n", settings.increments());
    let mut procedure = String::from("*Modal dynamics");
    if settings.steady_state {
        procedure.push_str(", Steady state");
    }
    let second = if settings.steady_state {
        settings.relative_error
    } else {
        settings.time_period
    };
    let _ = writeln!(
        procedure,
        "\n{}, {}",
        number(settings.increment),
        number(second)
    );
    modal_damping(&mut procedure, settings.damping.as_ref());
    (header, procedure)
}

/// The `*Step` line and the procedure keyword of a steady state dynamics step, as
/// PrePoMax's `CalSteadyStateDynamicsStep` writes them.
fn steady_state_dynamics_step(settings: &SteadyStateDynamicsStep) -> (String, String) {
    let mut procedure = String::from("*Steady state dynamics");
    if !settings.harmonic {
        procedure.push_str(", Harmonic=No");
    }
    let _ = write!(
        procedure,
        "\n{}, {}, {}, {}",
        number(settings.lower_frequency),
        number(settings.upper_frequency),
        settings.data_points,
        number(settings.bias)
    );
    if !settings.harmonic {
        let _ = write!(
            procedure,
            ", {}, {}, {}",
            settings.fourier_terms,
            number(settings.time_lower),
            number(settings.time_upper)
        );
    }
    procedure.push('\n');
    modal_damping(&mut procedure, settings.damping.as_ref());
    ("*Step\n".into(), procedure)
}

/// `*Modal damping` as PrePoMax's `CalModalDamping` writes it: a constant ratio covers
/// modes 1 to 1000000, Rayleigh damping leaves the mode range empty.
fn modal_damping(out: &mut String, damping: Option<&ModalDamping>) {
    match damping {
        None => {}
        Some(ModalDamping::Constant(ratio)) => {
            let _ = writeln!(out, "*Modal damping\n1, 1000000, {}", number(*ratio));
        }
        Some(ModalDamping::Direct(ranges)) => {
            out.push_str("*Modal damping\n");
            for range in ranges {
                let _ = writeln!(
                    out,
                    "{}, {}, {}",
                    range.lowest,
                    range.highest,
                    number(range.ratio)
                );
            }
        }
        Some(ModalDamping::Rayleigh(r)) => {
            let _ = writeln!(
                out,
                "*Modal damping, Rayleigh\n , , {}, {}",
                number(r.alpha),
                number(r.beta)
            );
        }
    }
}

/// The `*Step` line and the procedure with its increments, for steps with a time period.
fn incremented_step(settings: &StaticStep, keyword: String, nlgeom: bool) -> (String, String) {
    let default = settings.incrementation == Incrementation::Default;
    let mut header = String::from("*Step");
    if nlgeom {
        header.push_str(", Nlgeom");
    }
    if !default {
        let _ = write!(header, ", Inc={}", settings.max_increments);
    }
    header.push('\n');
    let procedure = match settings.incrementation {
        Incrementation::Default => format!("{keyword}\n"),
        Incrementation::Automatic => format!(
            "{keyword}\n{}, {}, {}, {}\n",
            number(settings.initial_increment),
            number(settings.time_period),
            number(settings.min_increment),
            number(settings.max_increment)
        ),
        Incrementation::Direct => format!(
            "{keyword}, Direct\n{}, {}\n",
            number(settings.initial_increment),
            number(settings.time_period)
        ),
    };
    (header, procedure)
}

/// The `*Step` line and the procedure keyword of a frequency step, as PrePoMax writes them:
/// the lower bound is written as 0 when only an upper bound is given. In a cyclic symmetric
/// model CalculiX finds every mode twice, as a cosine and a sine part, so it is asked for
/// twice the number of eigenfrequencies per nodal diameter.
fn frequency_step(settings: &FrequencyStep, cyclic: bool) -> (String, String) {
    let mut header = String::from("*Step");
    if settings.perturbation {
        header.push_str(", Perturbation");
    }
    header.push('\n');
    let mut procedure = String::from("*Frequency");
    if let Some(solver) = settings.solver.keyword() {
        let _ = write!(procedure, ", Solver={solver}");
    }
    if settings.storage {
        procedure.push_str(", Storage=Yes");
    }
    let count = settings.num_frequencies * if cyclic { 2 } else { 1 };
    let _ = write!(procedure, "\n{count}");
    match (settings.lower_frequency, settings.upper_frequency) {
        (None, None) => {}
        (lower, None) => {
            let _ = write!(procedure, ", {}", number(lower.unwrap_or(0.0)));
        }
        (lower, Some(upper)) => {
            let lower = number(lower.unwrap_or(0.0));
            let _ = write!(procedure, ", {lower}, {}", number(upper));
        }
    }
    procedure.push('\n');
    (header, procedure)
}

/// `*Select cyclic symmetry modes` of a frequency step in a cyclic symmetric model, with the
/// range of nodal diameters if one is set.
fn select_cyclic_symmetry_modes(settings: &FrequencyStep) -> String {
    match settings.nodal_diameters {
        Some([min, max]) => format!("*Select cyclic symmetry modes, Nmin={min}, Nmax={max}\n"),
        None => "*Select cyclic symmetry modes\n".into(),
    }
}

/// The `*Step` line and the procedure keyword of a complex frequency step, as PrePoMax's
/// `CalComplexFrequency` writes them. `perturbation` is the flag of the frequency step that
/// stored the modes: CalculiX requires the same on both `*STEP` cards.
fn complex_frequency_step(settings: &ComplexFrequencyStep, perturbation: bool) -> (String, String) {
    let mut header = String::from("*Step");
    if perturbation {
        header.push_str(", Perturbation");
    }
    header.push('\n');
    let mut procedure = String::from("*Complex frequency");
    if settings.coriolis {
        procedure.push_str(", Coriolis");
    }
    let _ = writeln!(procedure, "\n{}", settings.num_frequencies);
    (header, procedure)
}

/// The `*Step` line and the procedure keyword of a buckle step, as PrePoMax's
/// `CalBuckleStep` writes them.
fn buckle_step(settings: &BuckleStep) -> (String, String) {
    let mut header = String::from("*Step");
    if settings.perturbation {
        header.push_str(", Perturbation");
    }
    header.push('\n');
    let mut procedure = String::from("*Buckle");
    if let Some(solver) = settings.solver.keyword() {
        let _ = write!(procedure, ", Solver={solver}");
    }
    let _ = writeln!(
        procedure,
        "\n{}, {}",
        settings.num_factors,
        number(settings.accuracy)
    );
    (header, procedure)
}

/// `*Node file`, `*El file` or `*Contact file` of a field output, as PrePoMax's
/// `CalNodeFile`, `CalElFile` and `CalContactFile` write them; none without variables.
/// Node outputs are written in the form `dimension` of the step's element outputs.
fn field_output(output: &FieldOutput, dimension: OutputDimension) -> Option<Keyword> {
    if !output.active {
        return Some(deactivated(&output.name));
    }
    if output.variables.is_empty() {
        return None;
    }
    let mut variables = output.variables.join(", ");
    let mut header = match output.kind {
        OutputKind::Node => "*Node file",
        OutputKind::Element => {
            // PrePoMax adds nodal extrapolation of stresses unless error estimators are requested.
            let has = |v: &str| output.variables.iter().any(|x| x == v);
            if has("S") && !has("ERR") && !has("HER") && !has("ZZS") {
                variables.push_str(", NOE");
            }
            "*El file"
        }
        OutputKind::Contact => "*Contact file",
    }
    .to_string();
    if output.last_iterations {
        header.push_str(", Last iterations");
    }
    if output.contact_elements {
        header.push_str(", Contact elements");
    }
    if output.kind != OutputKind::Contact {
        let dimension = match output.kind {
            OutputKind::Element => output.dimension,
            _ => dimension,
        };
        // CalculiX's default writes shells and beams as the solids it expands them to; at
        // the shell nodes (2D) the stresses are those of the mid-surface, without bending.
        match dimension {
            OutputDimension::Default => {}
            OutputDimension::TwoD => header.push_str(", Output=2D"),
            OutputDimension::ThreeD => header.push_str(", Output=3D"),
        }
        if !output.global {
            header.push_str(", Global=No");
        }
    }
    Some(Keyword::generated(format!(
        "** Name: {}\n{header}\n{variables}\n",
        output.name
    )))
}

/// `*Node print`, `*El print` or `*Contact print` of a history output, as PrePoMax's
/// `CalNodePrint`, `CalElPrint` and `CalContactPrint` write them; none without variables.
fn history_output(
    sets: &mut Sets,
    output: &HistoryOutput,
    pair_surfaces: &PairSurfaces,
) -> Result<Option<Keyword>, WriteError> {
    if output.variables.is_empty() {
        return Ok(None);
    }
    let totals = match output.totals {
        Totals::No => "",
        Totals::Yes => ", Totals=Yes",
        Totals::Only => ", Totals=Only",
    };
    let header = match &output.kind {
        HistoryKind::Node {
            region: region @ (Region::ReferencePoint(_) | Region::RemotePoint(_)),
        } => {
            return point_output(sets, output, region, totals);
        }
        HistoryKind::Node { region } => {
            let set = sets.node_set(&output.name, region)?;
            format!("*Node print, Nset={set}{totals}")
        }
        HistoryKind::Element { region } => {
            let set = sets.element_set(&output.name, region)?;
            format!("*El print, Elset={set}{totals}")
        }
        HistoryKind::Contact { pair } => {
            // Contact forces are summed over the slave surface of the pair.
            let mut header = format!("*Contact print{totals}");
            if output.variables.iter().any(|v| v == "CF") {
                let (master, slave) =
                    pair_surfaces
                        .get(pair)
                        .ok_or_else(|| WriteError::UnknownContactPair {
                            item: output.name.clone(),
                            pair: pair.clone(),
                        })?;
                let _ = write!(header, ", Master={master}, Slave={slave}");
            }
            header
        }
    };
    Ok(Some(Keyword::generated(format!(
        "** Name: {}\n{header}\n{}\n",
        output.name,
        output.variables.join(", ")
    ))))
}

/// Reference and rotation node of a reference point or the point of a remote displacement,
/// and how the region follows the point: a reference point drives a rigid body.
fn point_nodes(
    sets: &Sets,
    item: &str,
    region: &Region,
) -> Result<([NodeId; 2], RemoteBehavior), WriteError> {
    match region {
        Region::ReferencePoint(point) => match sets.reference_nodes.get(point) {
            Some(nodes) => Ok(([nodes.reference, nodes.rotation], RemoteBehavior::Rigid)),
            None => Err(WriteError::UnknownReferencePoint {
                item: item.to_string(),
                point: point.clone(),
            }),
        },
        Region::RemotePoint(point) => match sets.remote_names.get(point) {
            Some(&nodes) => Ok(nodes),
            None => Err(WriteError::UnknownRemotePoint {
                item: item.to_string(),
                point: point.clone(),
            }),
        },
        _ => unreachable!("not a point"),
    }
}

/// The nodal history output of a point: the values of its reference node, and the
/// rotations, moments and angular velocities as the displacements, forces and velocities
/// of its rotation node. The rotation node's set is named `Internal_Rotation-n_<output>`,
/// by which the `.dat` reader shows its values under the output's name.
///
/// CalculiX has reaction forces at the point of a rigid body only: the point of equations,
/// a deformable or coupled remote displacement, prints zeros, so its `RF` is left out. A
/// coupled point does not turn.
fn point_output(
    sets: &mut Sets,
    output: &HistoryOutput,
    region: &Region,
    totals: &str,
) -> Result<Option<Keyword>, WriteError> {
    let ([reference, rotation], behavior) = point_nodes(sets, &output.name, region)?;
    let reactions = behavior == RemoteBehavior::Rigid;
    let printed: Vec<&str> = (output.variables.iter())
        .map(String::as_str)
        .filter(|&v| reactions || v != "RF")
        .collect();
    if printed.is_empty() {
        return Ok(None);
    }
    let set = sets.free_name("Internal_Selection", &name(&output.name));
    sets.node_sets.push((set.clone(), vec![reference]));
    let mut text = format!(
        "** Name: {}\n*Node print, Nset={set}{totals}\n{}\n",
        output.name,
        printed.join(", ")
    );
    let turning: Vec<&str> = (printed.iter().copied())
        .filter(|v| ["RF", "U", "V"].contains(v))
        .collect();
    if !turning.is_empty() {
        let set = sets.free_name("Internal_Rotation", &name(&output.name));
        sets.node_sets.push((set.clone(), vec![rotation]));
        let _ = write!(
            text,
            "*Node print, Nset={set}{totals}\n{}\n",
            turning.join(", ")
        );
    }
    Ok(Some(Keyword::generated(text)))
}

/// A keyword line followed by ids, 16 per line, the most CalculiX reads on one line.
fn id_list(header: &str, ids: &[u32]) -> String {
    let lines: Vec<String> = ids
        .chunks(16)
        .map(|chunk| {
            let ids: Vec<String> = chunk.iter().map(u32::to_string).collect();
            ids.join(", ")
        })
        .collect();
    format!("{header}\n{}\n", lines.join(",\n"))
}

/// A real number in at most 16 characters, CalculiX's field width.
/// Length and direction of a vector; `None` for the zero vector.
fn unit_vector(v: [f64; 3]) -> Option<(f64, [f64; 3])> {
    let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    (length > 0.0).then(|| (length, v.map(|c| c / length)))
}

fn number(value: f64) -> String {
    let short = value.to_string();
    if short.len() <= 16 {
        short
    } else {
        format!("{value:.8E}")
    }
}

/// Item names as CalculiX accepts them: no spaces, commas or other special characters.
/// Longest set or surface name CalculiX reads.
const NAME_LIMIT: usize = 80;

/// Longest name of a set made for a region: an element surface appends the face (`_S6`) to
/// its sets.
const SET_NAME_LIMIT: usize = NAME_LIMIT - 3;

/// Start of the comment that names what a shortened name stands for, which the reader of
/// the `.dat` file puts back.
pub const SHORTENED_NAME: &str = "** Shortened name: ";

/// `head` and `postfix` as one name, which CalculiX reads: a name too long keeps its start
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

/// The comments naming what each shortened name stands for.
fn shortened_names(shortened: &[(String, String)]) -> Vec<Keyword> {
    if shortened.is_empty() {
        return Vec::new();
    }
    let mut text = String::new();
    for (short, full) in shortened {
        let _ = writeln!(text, "{SHORTENED_NAME}{short} = {full}");
    }
    vec![Keyword::generated(text)]
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
mod pre_tension;
