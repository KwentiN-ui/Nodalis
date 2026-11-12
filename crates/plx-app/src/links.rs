//! Links between the analyses of a project, like the links between the cells of Ansys
//! Workbench's project schematic, but down to single items of the model tree: an analysis
//! takes the geometry, the mesh, its setup, a category of the setup or one item of it from
//! another analysis, which owns them. The results of a step of one analysis can drive a step
//! of another, the submodel.
//!
//! A path names what is linked, as the block of the analysis lists it: `["Geometry"]`,
//! one part of the geometry of a file `["Geometry", "BOLT"]`, `["Mesh"]`, `["Setup"]`, `["Setup", "Materials"]`, `["Setup", "Materials", "Steel"]` or
//! `["Results", "Step-1"]`.
//!
//! Links are of two kinds. A reference link makes a row of the target the same as the row
//! of the same path in the source: it goes from like to like, and the target cannot change
//! what it takes. An operation link gives what a step of the source produced, its results,
//! to a step of the target that uses it: it starts at a Results row, which only gives, and
//! ends at an input of a step under `["Setup", "Steps", step]`: its `"Temperatures"`, its
//! `"Eigenmodes"` or its `"Global results"` (see [`STEP_INPUTS`]), at the temperatures of
//! the initial conditions, or at the State row.
//!
//! A value link gives a number, a row `["Value", name]` of a Parameters block, to a
//! parameter of another block, `["Parameters", key]`, such as the length of a mesh block.
//! Like a reference, the parameter is then the same as the number and read-only.
//!
//! A state link, `["State", "Step-1"]`, makes the target continue the source: it takes the
//! source's model and its steps up to that step, then solves its own steps from the state
//! CalculiX wrote after it.

use std::collections::BTreeMap;
use std::path::PathBuf;

use plx_io::project::Link;
use plx_mesh::{FeMesh, Fnv};
use plx_model::{
    BoundaryKind, DefinedField, DefinedFieldKind, FeModel, Geometry, ModelKind, Step, StepKind,
    TagMap,
};

use crate::tree::{CROSS_SECTIONS, TreeItem};

pub const GEOMETRY: &str = "Geometry";
pub const MESH: &str = "Mesh";
pub const SETUP: &str = "Setup";
pub const RESULTS: &str = "Results";
pub const STATE: &str = "State";
/// The rows of numbers a block gives, `["Value", name]`.
pub const VALUE: &str = "Value";
/// The rows of numbers a block takes, `["Parameters", key]`.
pub const PARAMETERS: &str = "Parameters";
/// The row of a measure block that takes the results of a step.
pub const MEASURE: &str = "Measure";
/// The row of a deformed mesh block that takes the results of a step, whose displacements
/// move the nodes of the mesh of the analysis they belong to.
pub const STEP_RESULT: &str = "Step Result";
/// The inputs of a parameter study, `["Loop", column]`: value links that lead back to the
/// blocks before the study, the feedback of its loop.
pub const LOOP: &str = "Loop";
/// The outputs of a parameter study, `["Outputs", column]`, which take numbers.
pub const OUTPUTS: &str = "Outputs";
/// The row of a Campbell block that takes the setup of an analysis, whose model it sweeps.
pub const ANALYSIS: &str = "Analysis";

/// The categories of the setup in the order of the FE Model tree; each can be linked as a
/// whole or item by item.
pub const CATEGORIES: [&str; 9] = [
    "Features",
    "Materials",
    CROSS_SECTIONS,
    "Sections",
    "Constraints",
    "Contacts",
    "Amplitudes",
    "Initial Conditions",
    "Steps",
];

/// What an analysis has that links read: the data of a closed analysis or of an open window.
#[derive(Clone, Copy)]
pub struct Data<'a> {
    pub geometry: Option<&'a Geometry>,
    pub mesh: &'a FeMesh,
    pub fe: &'a FeModel,
    pub mesh_inputs: &'a BTreeMap<String, u64>,
}

/// A mesh with what belongs to it, taken over by a link of the mesh.
pub struct MeshCopy {
    /// The geometry it was meshed from.
    pub geometry: Option<Geometry>,
    pub mesh: FeMesh,
    pub mesh_inputs: BTreeMap<String, u64>,
    pub mesh_sets: Vec<plx_model::MeshSet>,
    pub element_types: Vec<plx_model::PartElementType>,
    /// The operations the mesh has applied, so that the copy meshes alike.
    pub mesh_operations: Vec<plx_model::MeshOperation>,
}

impl MeshCopy {
    pub fn of(data: Data) -> Self {
        Self {
            geometry: data.geometry.cloned(),
            mesh: data.mesh.clone(),
            mesh_inputs: data.mesh_inputs.clone(),
            mesh_sets: data.fe.mesh_sets.clone(),
            element_types: data.fe.element_types.clone(),
            mesh_operations: data.fe.mesh_operations.clone(),
        }
    }
}

/// Meshes of several sources put together into one, in their order, for the Mesh row of an
/// analysis that takes several mesh links. Node and element numbers of each mesh are
/// shifted past those before it; parts, sets and surfaces whose names an earlier mesh has
/// get "-2", "-3" …; `merged` is the geometry of all, put together from theirs, with the new
/// tags of each one's CAD entities: without it, where one has no geometry, the meshes have
/// none either. The parts stay as meshed, see [`MeshCopy::mesh_inputs`]. Returns the mesh with the renamings, by mesh, for the output.
pub fn merge_meshes(
    copies: Vec<MeshCopy>,
    merged: Option<plx_mesher::MergedGeometry>,
) -> (MeshCopy, Vec<BTreeMap<String, String>>) {
    use plx_mesh::{Part, SurfaceDefinition};
    use plx_model::Region;
    use std::collections::BTreeSet;
    let unique = |name: &str, used: &BTreeSet<String>| {
        (1..)
            .map(|n| {
                if n == 1 {
                    name.to_owned()
                } else {
                    format!("{name}-{n}")
                }
            })
            .find(|c| !used.contains(c))
            .unwrap_or_default()
    };
    let (geometry, tags, geometry_renamed) = match merged {
        Some(m) => (Some(m.geometry), m.tags, m.renamed),
        None => (None, Vec::new(), Vec::new()),
    };
    let mut mesh = FeMesh::default();
    let mut out = MeshCopy {
        geometry,
        mesh: FeMesh::default(),
        mesh_inputs: BTreeMap::new(),
        mesh_sets: Vec::new(),
        element_types: Vec::new(),
        mesh_operations: Vec::new(),
    };
    let (mut parts_used, mut sets_used, mut ops_used) =
        (BTreeSet::new(), BTreeSet::new(), BTreeSet::new());
    let mut renamings = Vec::with_capacity(copies.len());
    let (mut node_offset, mut element_offset) = (0, 0);
    for (index, copy) in copies.into_iter().enumerate() {
        let mut parts: BTreeMap<String, String> = BTreeMap::new();
        for part in &copy.mesh.parts {
            let wanted = (geometry_renamed.get(index))
                .and_then(|r| r.get(&part.name))
                .cloned()
                .unwrap_or_else(|| part.name.clone());
            let name = unique(&wanted, &parts_used);
            parts_used.insert(name.clone());
            parts.insert(part.name.clone(), name);
        }
        let part = |name: &String| parts.get(name).cloned().unwrap_or_else(|| name.clone());
        let mut sets: BTreeMap<String, String> = BTreeMap::new();
        let names = (copy.mesh.node_sets.keys())
            .chain(copy.mesh.element_sets.keys())
            .chain(copy.mesh.surfaces.keys())
            .chain(copy.mesh_sets.iter().map(|s| &s.name));
        for name in names.collect::<BTreeSet<_>>() {
            let new = unique(name, &sets_used);
            sets.insert(name.clone(), new);
        }
        sets_used.extend(sets.values().cloned());
        let set = |name: &String| sets.get(name).cloned().unwrap_or_else(|| name.clone());
        let (nodes, elements) = (node_offset, element_offset);
        for (&id, &coords) in copy.mesh.node_ids().iter().zip(copy.mesh.coords()) {
            mesh.set_node(id + nodes, coords);
        }
        for element in copy.mesh.elements() {
            let _ = mesh.add_element(plx_mesh::Element {
                id: element.id + elements,
                nodes: element.nodes.iter().map(|n| n + nodes).collect(),
                ..element.clone()
            });
        }
        node_offset = mesh.node_ids().iter().copied().max().unwrap_or(0);
        element_offset = mesh.elements().iter().map(|e| e.id).max().unwrap_or(0);
        for p in &copy.mesh.parts {
            mesh.parts.push(Part {
                name: part(&p.name),
                elements: p.elements.iter().map(|e| e + elements).collect(),
                bodies: (p.bodies.iter())
                    .map(|b| plx_mesh::Body {
                        name: (geometry_renamed.get(index))
                            .and_then(|r| r.get(&b.name))
                            .cloned()
                            .unwrap_or_else(|| b.name.clone()),
                        elements: b.elements.iter().map(|e| e + elements).collect(),
                    })
                    .collect(),
            });
        }
        for (name, ids) in &copy.mesh.node_sets {
            (mesh.node_sets).insert(set(name), ids.iter().map(|n| n + nodes).collect());
        }
        for (name, ids) in &copy.mesh.element_sets {
            (mesh.element_sets).insert(set(name), ids.iter().map(|e| e + elements).collect());
        }
        for (name, surface) in &copy.mesh.surfaces {
            let surface = match surface {
                SurfaceDefinition::ElementFaces(faces) => SurfaceDefinition::ElementFaces(
                    faces.iter().map(|&(e, f)| (e + elements, f)).collect(),
                ),
                SurfaceDefinition::Nodes(ids) => {
                    SurfaceDefinition::Nodes(ids.iter().map(|n| n + nodes).collect())
                }
            };
            mesh.surfaces.insert(set(name), surface);
        }
        let entity_tags = tags.get(index);
        if let Some(entity_tags) = entity_tags {
            mesh.cad.extend(
                copy.mesh
                    .cad
                    .offset(nodes, elements)
                    .renumbered(entity_tags),
            );
        }
        // No mesh inputs: the parts stay as they are, like parts read from an input file,
        // and are out of date only with the meshes they come from.
        for mut set_item in copy.mesh_sets {
            set_item.name = set(&set_item.name);
            set_item.region = match set_item.region {
                Region::Parts(names) => Region::Parts(names.iter().map(part).collect()),
                Region::Nodes(ids) => Region::Nodes(ids.iter().map(|n| n + nodes).collect()),
                Region::Faces(faces) => {
                    Region::Faces(faces.iter().map(|&(e, f)| (e + elements, f)).collect())
                }
                Region::Edges(edges) => {
                    Region::Edges(edges.iter().map(|e| e.map(|n| n + nodes)).collect())
                }
                Region::NodeSet(name) => Region::NodeSet(set(&name)),
                Region::ElementSet(name) => Region::ElementSet(set(&name)),
                Region::Surface(name) => Region::Surface(set(&name)),
                Region::Geometry(entities) => Region::Geometry(
                    (entities.iter())
                        .filter_map(|e| entity_tags.and_then(|t| t.get(e)).copied())
                        .collect(),
                ),
                other => other,
            };
            out.mesh_sets.push(set_item);
        }
        for mut element_type in copy.element_types {
            element_type.part = part(&element_type.part);
            out.element_types.push(element_type);
        }
        for mut operation in copy.mesh_operations {
            let name = unique(&operation.name, &ops_used);
            ops_used.insert(name.clone());
            operation.name = name;
            operation.parts = operation.parts.iter().map(part).collect();
            out.mesh_operations.push(operation);
        }
        let mut renamed: BTreeMap<String, String> =
            parts.into_iter().filter(|(a, b)| a != b).collect();
        renamed.extend(sets.into_iter().filter(|(a, b)| a != b));
        renamings.push(renamed);
    }
    out.mesh = mesh;
    (out, renamings)
}

/// The path as text, as the output pane names it: "Setup > Materials > Steel".
pub fn label(path: &[String]) -> String {
    path.join(" > ")
}

/// Whether `path` is `prefix` or lies inside it.
pub fn covers(prefix: &[String], path: &[String]) -> bool {
    path.len() >= prefix.len() && path[..prefix.len()] == *prefix
}

/// Where a link puts what it takes in the target.
pub fn slot(link: &Link) -> &[String] {
    if link.slot.is_empty() {
        &link.item
    } else {
        &link.slot
    }
}

/// Whether `link` ends at the row of `path` of its target: in it, or, for the global
/// results of a submodel, at the Global results row of the block.
pub fn ends_at(path: &[String], link: &Link) -> bool {
    covers(path, slot(link))
        || (matches!(path, [cell] if cell == GLOBAL_RESULTS)
            && ResultsUse::of(link) == Some(ResultsUse::Submodel))
}

/// Whether the link carries the results of a step, for a submodel.
pub fn is_results(link: &Link) -> bool {
    link.item.first().is_some_and(|c| c == RESULTS)
}

/// Whether the link gives a number to a parameter, or to a study.
pub fn is_value(link: &Link) -> bool {
    link.item.first().is_some_and(|c| c == VALUE || c == LOOP)
}

/// Whether the link is the feedback of a study: it gives an input of the study to the blocks
/// before it, and is left out of the order of the blocks.
pub fn is_feedback(link: &Link) -> bool {
    link.item.first().is_some_and(|c| c == LOOP)
}

/// The exposed parameters of a block that value links give, by key, with the block each
/// comes from, of the paths links fill in it (see [`Incoming::paths`]).
pub fn linked_parameters(paths: &[(Vec<String>, String)]) -> Vec<(u32, String)> {
    (paths.iter())
        .filter_map(|(path, source)| match path.as_slice() {
            [cell, key] if cell == PARAMETERS => Some((key.parse().ok()?, source.clone())),
            _ => None,
        })
        .collect()
}

/// Whether the link makes its target continue from the state after a step of the source.
pub fn is_state(link: &Link) -> bool {
    link.item.first().is_some_and(|c| c == STATE)
}

/// The row of the source a link starts at: the results of the step for a state link.
pub fn output_path(link: &Link) -> Vec<String> {
    if is_state(link) {
        let mut path = link.item.clone();
        path[0] = RESULTS.to_owned();
        path
    } else {
        link.item.clone()
    }
}

/// Whether the link takes one part of the geometry of its source, `["Geometry", part]`, into
/// the Geometry row of its target. The parts a source gives to the same target go together:
/// the target takes the geometry with only those parts, see [`linked_parts`].
pub fn is_geometry_part(link: &Link) -> bool {
    matches!(link.item.as_slice(), [cell, _] if cell == GEOMETRY)
}

/// The parts of the geometry of `source` that `target` takes through part links, in the
/// order of the links.
pub fn linked_parts(links: &[Link], source: u64, target: u64) -> Vec<String> {
    let mut parts: Vec<String> = Vec::new();
    for link in links
        .iter()
        .filter(|l| l.source == source && l.target == target)
    {
        if is_geometry_part(link) && !parts.contains(&link.item[1]) {
            parts.push(link.item[1].clone());
        }
    }
    parts
}

/// Whether the link takes a whole category of the setup. It goes to a collection, whose pin
/// takes any number of wires, and is a bundle of links of the category's items: the target
/// keeps its own items of other names, and items can be left out, see [`Link::excluded`].
pub fn is_bundle(link: &Link) -> bool {
    link.slot.is_empty() && matches!(link.item.as_slice(), [cell, _] if cell == SETUP)
}

/// Whether the row of `path` is a collection that takes any number of wires: a category of
/// the setup. Every other row takes one, and a new wire replaces the old.
pub fn is_collection(path: &[String]) -> bool {
    matches!(path, [cell, category] if cell == SETUP && CATEGORIES.contains(&category.as_str()))
}

/// Whether the row of `path` puts the geometries or meshes of several links together when it
/// is a row of an analysis: its Geometry and its Mesh row.
pub fn assembles(path: &[String]) -> bool {
    matches!(path, [cell] if cell == GEOMETRY || cell == MESH)
}

/// The category of the setup a link of a category or of one of its items fills.
pub fn setup_category(link: &Link) -> Option<&str> {
    if !link.slot.is_empty() {
        return None;
    }
    match link.item.as_slice() {
        [cell, category, ..] if cell == SETUP => Some(category),
        _ => None,
    }
}

/// The items a link of a category or of an item offers its target: those of the source's
/// category but the excluded ones, or the item.
pub fn offered(source: &FeModel, link: &Link) -> Vec<String> {
    match link.item.as_slice() {
        [_, category] => (entries(source, category).into_iter())
            .filter(|name| !link.excluded.contains(name))
            .collect(),
        [_, _, name] => vec![name.clone()],
        _ => Vec::new(),
    }
}

/// Whether the target of a category link has the items `names` of the source as they are
/// there, and took no others last time.
pub fn bundle_up_to_date(
    source: &FeModel,
    target: &FeModel,
    link: &Link,
    names: &[String],
) -> bool {
    let Some(category) = setup_category(link) else {
        return true;
    };
    let print = |fe: &FeModel| {
        let mut hash = Fnv::new();
        for name in names {
            category_fingerprint(&mut hash, fe, category, Some(name));
        }
        hash.finish()
    };
    link.inherited == names && print(source) == print(target)
}

/// Copies the items `names` of a category of `source` into `target`, see [`is_bundle`]: an
/// item replaces the target's item of the same name or comes after the linked items before
/// it, those named in `ahead` and those of this link; own items of other names stay. Items
/// the link took last time that the source no longer has are removed. Returns the names
/// taken, for [`Link::inherited`].
pub fn apply_bundle(
    target: &mut FeModel,
    source: &FeModel,
    link: &Link,
    names: &[String],
    ahead: &[String],
) -> Vec<String> {
    let Some(category) = setup_category(link) else {
        return Vec::new();
    };
    let present = entries(source, category);
    let stale: Vec<String> = (link.inherited.iter())
        .filter(|name| !present.contains(name))
        .cloned()
        .collect();
    let mode = Mode::Bundle {
        names,
        stale: &stale,
        ahead,
    };
    copy_category(target, source, category, mode);
    names.to_vec()
}

/// What a link fills in its target, with the name of the source: for a state link the
/// geometry, the mesh, the setup but the steps, and the steps it took; for a category
/// the items it took, so that the target's own items stay its own.
pub fn filled_paths(link: &Link, source: &str) -> Vec<(Vec<String>, String)> {
    let path = |parts: &[&str]| parts.iter().map(|p| p.to_string()).collect::<Vec<_>>();
    if is_bundle(link) {
        return (link.inherited.iter())
            .map(|name| {
                let mut path = link.item.clone();
                path.push(name.clone());
                (path, source.to_owned())
            })
            .collect();
    }
    if !is_state(link) {
        return vec![(slot(link).to_vec(), source.to_owned())];
    }
    let mut paths = vec![path(&[STATE]), path(&[GEOMETRY]), path(&[MESH])];
    for category in CATEGORIES.iter().filter(|c| **c != "Steps") {
        paths.push(path(&[SETUP, category]));
    }
    for step in &link.inherited {
        paths.push(path(&[SETUP, "Steps", step]));
    }
    paths.into_iter().map(|p| (p, source.to_owned())).collect()
}

/// The index of the step of `source` a state link continues from.
pub fn state_step(source: &FeModel, link: &Link) -> Option<usize> {
    let name = link.item.get(1)?;
    source.steps.iter().position(|s| &s.name == name)
}

/// The steps of the target of a state link that are its own, not taken from the source.
pub fn own_steps(target: &FeModel, link: &Link) -> Vec<Step> {
    (target.steps.iter())
        .filter(|s| !link.inherited.contains(&s.name))
        .cloned()
        .collect()
}

/// Fingerprint of what a state link takes of a model besides the geometry and mesh: the
/// setup but the steps, the model kind and these steps.
fn chain_fingerprint(fe: &FeModel, steps: &[Step]) -> u64 {
    let mut hash = Fnv::new();
    for category in CATEGORIES.iter().filter(|c| **c != "Steps") {
        category_fingerprint(&mut hash, fe, category, None);
    }
    hash.add_debug(&fe.properties.kind);
    hash.add_debug(&fe.properties.global_results);
    hash.add_debug(&steps);
    hash.add_debug(&fe.printed_history_outputs(steps));
    hash.finish()
}

/// Whether the target of a state link has what it takes from `source`, whose step `step` it
/// continues from.
pub fn chain_up_to_date(source: &FeModel, step: usize, target: &FeModel, link: &Link) -> bool {
    let chain = &source.steps[..=step];
    let names: Vec<&String> = chain.iter().map(|s| &s.name).collect();
    let n = chain.len();
    names == link.inherited.iter().collect::<Vec<_>>()
        && target.steps.len() >= n
        && chain_fingerprint(source, chain) == chain_fingerprint(target, &target.steps[..n])
}

/// Makes `target` continue `source` after its step `step`: the setup but the steps is the
/// source's, the steps are the source's up to `step` followed by the target's own. Returns
/// the names of the steps taken, for [`Link::inherited`].
pub fn apply_chain(
    target: &mut FeModel,
    source: &FeModel,
    step: usize,
    link: &Link,
) -> Vec<String> {
    let own = own_steps(target, link);
    for category in CATEGORIES.iter().filter(|c| **c != "Steps") {
        copy_category(target, source, category, Mode::All);
    }
    target.properties.kind = source.properties.kind;
    target.properties.global_results = source.properties.global_results.clone();
    target.steps = source.steps[..=step].to_vec();
    let inherited = names(&target.steps);
    target.steps.extend(own);
    target.adopt_history_outputs(source);
    inherited
}

fn names(steps: &[Step]) -> Vec<String> {
    steps.iter().map(|s| s.name.clone()).collect()
}

/// Splits `fe` after its step `index`: returns it with the steps up to that one, and gives
/// the steps after it to `new`, the analysis that continues it.
pub fn split_after(fe: &FeModel, index: usize, new: &mut FeModel) -> FeModel {
    let mut kept = fe.clone();
    kept.steps.truncate(index + 1);
    kept.drop_unused_history_outputs();
    new.steps = fe.steps[index + 1..].to_vec();
    new.adopt_history_outputs(fe);
    kept
}

/// Gives the own steps of `target`, which continues `source` after its last step, to
/// `source`; returns their names.
pub fn merge_into(source: &mut FeModel, target: &FeModel, link: &Link) -> Vec<String> {
    let own = own_steps(target, link);
    let moved = names(&own);
    source.steps.extend(own);
    source.adopt_history_outputs(target);
    moved
}

/// What an analysis needs to continue from the states of others, and to let others
/// continue from its own.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Continuation {
    /// Work directory and job name of each analysis it continues, nearest first: their
    /// states and results are where a restart starts from.
    pub upstream: Vec<(PathBuf, String)>,
    /// The own steps of the analyses that continue it, after the index of the step of this
    /// one they follow: their sets go into its input file.
    pub dependent_steps: Vec<(usize, Vec<Step>)>,
    /// The steps it takes over from the analysis it continues, by name, with the name of
    /// that analysis.
    pub inherited_steps: Vec<(String, String)>,
}

/// What the results of a step of another analysis are used for, by the slot they go to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResultsUse {
    /// The displacements on the cut boundary of a submodel:
    /// `["Setup", "Steps", step, "Global results"]`.
    Submodel,
    /// The temperatures as a defined field of a step:
    /// `["Setup", "Steps", step, "Temperatures"]`.
    Temperature,
    /// The eigenmodes of a frequency step, for steady state dynamics:
    /// `["Setup", "Steps", step, "Eigenmodes"]`.
    Modes,
    /// The temperatures the model starts at: `["Setup", "Initial Conditions", "Temperature"]`.
    InitialTemperatures,
}

pub const TEMPERATURE: &str = "Temperature";
pub const STEPS: &str = "Steps";
pub const TEMPERATURES: &str = "Temperatures";
pub const EIGENMODES: &str = "Eigenmodes";
pub const GLOBAL_RESULTS: &str = "Global results";
const INITIAL_CONDITIONS: &str = "Initial Conditions";

/// The inputs a step has for the results of steps of other analyses, the rows under it.
pub const STEP_INPUTS: [&str; 3] = [TEMPERATURES, EIGENMODES, GLOBAL_RESULTS];

/// Name of the defined field a temperature link sets in its step.
pub const LINKED_TEMPERATURES: &str = "Linked Temperatures";

impl ResultsUse {
    /// What the results go to in the target, by the slot of a results link.
    pub fn of(link: &Link) -> Option<Self> {
        if !is_results(link) {
            return None;
        }
        Self::of_slot(slot(link))
    }

    /// What results that go to `slot` are used for.
    pub fn of_slot(slot: &[String]) -> Option<Self> {
        match slot {
            [cell, steps, _, input] if cell == SETUP && steps == STEPS => match input.as_str() {
                TEMPERATURES => Some(Self::Temperature),
                EIGENMODES => Some(Self::Modes),
                GLOBAL_RESULTS => Some(Self::Submodel),
                _ => None,
            },
            [cell, category, input] if cell == SETUP && category == INITIAL_CONDITIONS => {
                (input == TEMPERATURE).then_some(Self::InitialTemperatures)
            }
            _ => None,
        }
    }
}

/// The step of the target a results link goes to.
pub fn target_step(link: &Link) -> Option<&str> {
    match slot(link) {
        [cell, steps, step, _] if cell == SETUP && steps == STEPS => Some(step),
        _ => None,
    }
}

/// The inputs `step` has for results of other analyses, as rows under it: temperatures
/// when it takes defined fields, eigenmodes for modal and steady state dynamics, and the
/// global results that make the analysis a submodel.
pub fn step_inputs(step: &Step) -> Vec<&'static str> {
    let mut inputs = Vec::new();
    if step.kind.supports_defined_fields() {
        inputs.push(TEMPERATURES);
    }
    if step.kind.uses_stored_modes() {
        inputs.push(EIGENMODES);
    }
    inputs.push(GLOBAL_RESULTS);
    inputs
}

/// Where results of `source_step` dropped on `slot` of `target` go: a step of the target
/// takes temperatures of a thermal step as a defined field, the eigenmodes of a frequency
/// step for modal and steady state dynamics, other results for a submodel; the initial
/// conditions take temperatures. Dropped on a step, they go to the input of it that fits;
/// dropped on an input, they go there if they fit. `Err` tells why they cannot go there.
pub fn results_slot(
    source_step: &Step,
    target: &FeModel,
    slot: &[String],
) -> Result<Vec<String>, String> {
    let path = |parts: &[&str]| parts.iter().map(|p| p.to_string()).collect::<Vec<_>>();
    let thermal = source_step.kind.is_thermal();
    let source = &source_step.name;
    match slot {
        // The Global results row of a submodel's block: the step whose submodel boundary
        // conditions read them, else the first step.
        [cell] if cell == GLOBAL_RESULTS => {
            let submodel = |s: &&Step| {
                (s.boundary_conditions.iter())
                    .any(|bc| matches!(bc.kind, BoundaryKind::Submodel { .. }))
            };
            let step = (target.steps.iter().find(submodel))
                .or_else(|| target.steps.first())
                .ok_or_else(|| "the submodel has no step yet".to_string())?;
            results_slot(
                source_step,
                target,
                &path(&[SETUP, STEPS, &step.name, GLOBAL_RESULTS]),
            )
        }
        [cell, category, rest @ ..]
            if cell == SETUP
                && category == INITIAL_CONDITIONS
                && (rest.is_empty() || rest == [TEMPERATURE]) =>
        {
            if thermal {
                Ok(path(&[SETUP, INITIAL_CONDITIONS, TEMPERATURE]))
            } else {
                Err(format!(
                    "only the temperatures of a thermal step can be initial conditions, and \
                     {source} is none"
                ))
            }
        }
        [cell, steps, step, input @ ..] if cell == SETUP && steps == STEPS && input.len() <= 1 => {
            let Some(target_step) = target.steps.iter().find(|s| &s.name == step) else {
                return Err(format!("there is no step {step}"));
            };
            let inputs = step_inputs(target_step);
            let modes = match &source_step.kind {
                StepKind::Frequency(frequency) => Some(frequency.storage),
                _ => None,
            };
            let input = match input.first().map(String::as_str) {
                Some(input) if !inputs.contains(&input) => {
                    return Err(format!("{step} has no input {input}"));
                }
                Some(input) => input,
                // Dropped on the step: the input that fits.
                None if modes.is_some() && inputs.contains(&EIGENMODES) => EIGENMODES,
                None if thermal && inputs.contains(&TEMPERATURES) => TEMPERATURES,
                None => GLOBAL_RESULTS,
            };
            match input {
                EIGENMODES if modes.is_none() => {
                    return Err(format!("{source} is no frequency step"));
                }
                EIGENMODES if modes == Some(false) => {
                    return Err(format!(
                        "{source} does not store its eigenmodes; turn on Storage in it"
                    ));
                }
                TEMPERATURES if !thermal => {
                    return Err(format!("{source} is no thermal step"));
                }
                _ => {}
            }
            Ok(path(&[SETUP, STEPS, step, input]))
        }
        _ => Err("they cannot go there".into()),
    }
}

/// Lets `fe` use the results of another analysis: `file` is the results file (`.frd`) or,
/// for eigenmodes, the `.eig` file, `global` the step of it. Returns whether anything
/// changed.
pub fn apply_results(fe: &mut FeModel, link: &Link, global: u32, file: PathBuf) -> bool {
    let Some(kind) = ResultsUse::of(link) else {
        return false;
    };
    let step = target_step(link).unwrap_or_default().to_owned();
    match kind {
        ResultsUse::Submodel => apply_submodel(fe, &step, global, file),
        ResultsUse::Modes => {
            let changed = fe.properties.stored_modes.as_ref() != Some(&file);
            fe.properties.stored_modes = Some(file);
            changed
        }
        ResultsUse::InitialTemperatures => {
            let value = Some((file, global));
            let changed = fe.properties.initial_temperatures != value;
            fe.properties.initial_temperatures = value;
            changed
        }
        ResultsUse::Temperature => {
            let Some(step) = fe.steps.iter_mut().find(|s| s.name == step) else {
                return false;
            };
            let field = DefinedField {
                distribution: None,
                name: LINKED_TEMPERATURES.into(),
                active: true,
                region: plx_model::Region::Parts(Vec::new()),
                kind: DefinedFieldKind::TemperatureFromFile { file, step: global },
                amplitude: None,
            };
            match (step.defined_fields.iter_mut()).find(|f| f.name == LINKED_TEMPERATURES) {
                Some(own) if own.kind == field.kind => false,
                Some(own) => {
                    own.kind = field.kind;
                    true
                }
                None => {
                    step.defined_fields.push(field);
                    true
                }
            }
        }
    }
}

/// Takes back what a results link set in `fe`, once it is removed; `submodel` tells
/// whether other links still make it a submodel.
pub fn clear_results(fe: &mut FeModel, link: &Link, submodel: bool) {
    match ResultsUse::of(link) {
        Some(ResultsUse::Submodel) if !submodel => clear_submodel(fe),
        Some(ResultsUse::Modes) => fe.properties.stored_modes = None,
        Some(ResultsUse::InitialTemperatures) => fe.properties.initial_temperatures = None,
        Some(ResultsUse::Temperature) => {
            let step = target_step(link);
            for s in fe
                .steps
                .iter_mut()
                .filter(|s| Some(s.name.as_str()) == step)
            {
                s.defined_fields.retain(|f| f.name != LINKED_TEMPERATURES);
            }
        }
        _ => {}
    }
}

/// The names of the items of a category of the setup, as the tree lists them.
pub fn entries(fe: &FeModel, category: &str) -> Vec<String> {
    let names = |names: &mut dyn Iterator<Item = &str>| names.map(str::to_owned).collect();
    match category {
        "Features" => names(
            &mut (fe.reference_points.iter().map(|p| p.name.as_str()))
                .chain(fe.coordinate_systems.iter().map(|c| c.name.as_str()))
                .chain(fe.planes.iter().map(|p| p.name.as_str())),
        ),
        "Materials" => names(&mut fe.materials.iter().map(|m| m.name.as_str())),
        CROSS_SECTIONS => names(&mut fe.cross_sections.iter().map(|c| c.name.as_str())),
        "Sections" => names(&mut fe.sections.iter().map(|s| s.name.as_str())),
        "Constraints" => names(&mut fe.constraints.iter().map(|c| c.name())),
        "Contacts" => names(
            &mut (fe.contact_regions.iter().map(|r| r.name.as_str()))
                .chain(fe.surface_interactions.iter().map(|s| s.name.as_str()))
                .chain(fe.contact_pairs.iter().map(|c| c.name.as_str()))
                .chain(fe.ties.iter().map(|t| t.name.as_str()))
                .chain(fe.node_ties.iter().map(|t| t.name.as_str()))
                .chain(fe.bolts.iter().map(|b| b.name.as_str())),
        ),
        "Amplitudes" => names(&mut fe.amplitudes.iter().map(|a| a.name.as_str())),
        "Initial Conditions" => names(&mut fe.initial_conditions.iter().map(|c| c.name.as_str())),
        "Steps" => names(&mut fe.steps.iter().map(|s| s.name.as_str())),
        _ => Vec::new(),
    }
}

/// Whether the analysis has what the path names, so that a link can take it.
pub fn exists(data: Data, path: &[String]) -> bool {
    match path {
        [cell] if cell == GEOMETRY => data.geometry.is_some(),
        [cell] if cell == MESH => data.mesh.element_count() > 0,
        [cell] if cell == SETUP => true,
        [cell, category] if cell == SETUP => CATEGORIES.contains(&category.as_str()),
        [cell, category, name] if cell == SETUP => entries(data.fe, category).contains(name),
        [cell, step] if cell == RESULTS => data.fe.steps.iter().any(|s| &s.name == step),
        _ => false,
    }
}

/// Takes over the shapes of a geometry link's source: the mesh setup, the default meshing
/// parameters and the refinement stay those the target has, so that each analysis meshes
/// the same geometry its own way. A target without geometry starts from the source's setup.
///
/// When the shapes changed, Gmsh's tags did with them: the local mesh sizes are renumbered
/// here, and the returned tags are for the selections on the old shapes, see
/// [`follow_geometry`].
pub fn take_geometry(target: &mut Option<Geometry>, source: Option<Geometry>) -> Option<TagMap> {
    let mut tags = None;
    *target = match (target.take(), source) {
        (Some(own), Some(source)) => {
            tags = own.tags_in(&source);
            let mut taken = Geometry {
                meshing: own.meshing,
                mesh_items: own.mesh_items,
                refinement: own.refinement,
                ..source
            };
            if let Some(tags) = &tags {
                taken.renumber_mesh_items(tags);
            }
            Some(taken)
        }
        (_, source) => source,
    };
    tags
}

/// Where the shapes of `old` are in `new`, which takes their place with its own mesh, e.g.
/// through a mesh link; `None` when nothing needs renumbering.
pub fn geometry_tags(old: Option<&Geometry>, new: Option<&Geometry>) -> Option<TagMap> {
    old?.tags_in(new?)
}

/// Follows the selections on CAD entities, and the mesh made from the old shapes, into the
/// new shapes after [`take_geometry`]; a message says how many were not found, if any.
pub fn follow_geometry(
    fe: &mut FeModel,
    mesh: Option<&mut FeMesh>,
    tags: &TagMap,
) -> Option<String> {
    if let Some(mesh) = mesh {
        mesh.cad = mesh.cad.renumbered(tags);
    }
    match fe.follow_cad(tags) {
        0 => None,
        1 => Some(
            "1 picked geometry entity was not found in the changed geometry; pick it again".into(),
        ),
        lost => Some(format!(
            "{lost} picked geometry entities were not found in the changed geometry; pick \
             them again"
        )),
    }
}

/// Fingerprint of what the path takes from the analysis: a link copies it anew when the
/// source's differs from what the target has, or had last time for the mesh and geometry.
pub fn fingerprint(data: Data, path: &[String]) -> u64 {
    let mut hash = Fnv::new();
    match path {
        [cell] if cell == GEOMETRY => {
            // The shapes only: the mesh setup stays the target's own, see [`take_geometry`].
            if let Some(geometry) = data.geometry {
                hash.add_str(&geometry.source);
                hash.add_str(&geometry.brep);
                hash.add_debug(&geometry.part_names);
            }
        }
        [cell] if cell == MESH => {
            hash.add(&data.mesh.input_fingerprint().to_le_bytes());
            // A refine mesh block gives geometry to mesh with other sizes.
            let refinement = data.geometry.map(|g| g.refinement.to_bits());
            hash.add_debug(&refinement);
            hash.add_debug(data.mesh_inputs);
            hash.add_debug(&data.fe.mesh_sets);
            hash.add_debug(&data.fe.element_types);
            hash.add_debug(&data.fe.mesh_operations);
        }
        [cell, rest @ ..] if cell == SETUP => setup_fingerprint(&mut hash, data.fe, rest),
        _ => {}
    }
    hash.finish()
}

fn setup_fingerprint(hash: &mut Fnv, fe: &FeModel, path: &[String]) {
    match path {
        [] => {
            for category in CATEGORIES {
                category_fingerprint(hash, fe, category, None);
            }
        }
        [category] => category_fingerprint(hash, fe, category, None),
        [category, name] => category_fingerprint(hash, fe, category, Some(name)),
        _ => {}
    }
}

/// Hashes the items of a category, or only the one named. The global step of a submodel's
/// boundary conditions is left out: a results link sets it in the target.
fn category_fingerprint(hash: &mut Fnv, fe: &FeModel, category: &str, name: Option<&str>) {
    fn add<T: std::fmt::Debug>(
        hash: &mut Fnv,
        items: &[T],
        key: fn(&T) -> &str,
        name: Option<&str>,
    ) {
        for item in items.iter().filter(|i| name.is_none_or(|n| key(i) == n)) {
            hash.add_debug(item);
        }
    }
    hash.add_str(category);
    match category {
        "Features" => {
            add(hash, &fe.reference_points, |p| &p.name, name);
            add(hash, &fe.coordinate_systems, |c| &c.name, name);
            add(hash, &fe.planes, |p| &p.name, name);
        }
        "Materials" => add(hash, &fe.materials, |m| &m.name, name),
        CROSS_SECTIONS => add(hash, &fe.cross_sections, |c| &c.name, name),
        "Sections" => add(hash, &fe.sections, |s| &s.name, name),
        "Constraints" => add(hash, &fe.constraints, |c| c.name(), name),
        "Contacts" => {
            add(hash, &fe.contact_regions, |r| &r.name, name);
            if name.is_none() {
                hash.add_debug(&fe.contact_defaults);
                hash.add_debug(&fe.contact_detections);
            }
            add(hash, &fe.surface_interactions, |s| &s.name, name);
            add(hash, &fe.contact_pairs, |c| &c.name, name);
            add(hash, &fe.ties, |t| &t.name, name);
            add(hash, &fe.node_ties, |t| &t.name, name);
            add(hash, &fe.bolts, |b| &b.name, name);
        }
        "Amplitudes" => add(hash, &fe.amplitudes, |a| &a.name, name),
        "Initial Conditions" => add(hash, &fe.initial_conditions, |c| &c.name, name),
        "Steps" => {
            let mut steps: Vec<_> = (fe.steps.iter())
                .filter(|s| name.is_none_or(|n| s.name == n))
                .cloned()
                .collect();
            for bc in steps.iter_mut().flat_map(|s| &mut s.boundary_conditions) {
                if let BoundaryKind::Submodel { step, .. } = &mut bc.kind {
                    *step = 0;
                }
            }
            hash.add_debug(&steps);
            hash.add_debug(&fe.printed_history_outputs(&steps));
        }
        _ => {}
    }
}

/// Copies what the path names from `source` into `target`: the setup or a category
/// replaces the target's, an item replaces the target's item of the same name or is added.
/// Links of a category take its items with [`apply_bundle`] instead.
pub fn apply_setup(target: &mut FeModel, source: &FeModel, path: &[String]) {
    match path {
        [cell] if cell == SETUP => {
            for category in CATEGORIES {
                copy_category(target, source, category, Mode::All);
            }
        }
        [cell, category] if cell == SETUP => copy_category(target, source, category, Mode::All),
        [cell, category, name] if cell == SETUP => {
            copy_category(target, source, category, Mode::One(name))
        }
        _ => {}
    }
}

/// What [`copy_category`] copies.
#[derive(Clone, Copy)]
enum Mode<'a> {
    /// All items, in place of the target's.
    All,
    /// The item of this name.
    One(&'a str),
    /// The items of these names, see [`apply_bundle`].
    Bundle {
        names: &'a [String],
        stale: &'a [String],
        ahead: &'a [String],
    },
}

fn copy_category(target: &mut FeModel, source: &FeModel, category: &str, name: Mode) {
    fn copy<T: Clone>(target: &mut Vec<T>, source: &[T], key: fn(&T) -> &str, mode: Mode) {
        let named = |names: &[String], item: &T| names.iter().any(|n| n == key(item));
        match mode {
            Mode::All => *target = source.to_vec(),
            Mode::One(name) => {
                let Some(item) = source.iter().find(|i| key(i) == name) else {
                    return;
                };
                match target.iter_mut().find(|i| key(i) == name) {
                    Some(own) => *own = item.clone(),
                    None => target.push(item.clone()),
                }
            }
            Mode::Bundle {
                names,
                stale,
                ahead,
            } => {
                target.retain(|item| !named(stale, item));
                for item in source.iter().filter(|item| named(names, item)) {
                    match target.iter().position(|own| key(own) == key(item)) {
                        Some(at) => target[at] = item.clone(),
                        None => {
                            // Linked items come first, in the order of the links.
                            let at = (target.iter())
                                .rposition(|own| named(ahead, own) || named(names, own))
                                .map_or(0, |at| at + 1);
                            target.insert(at, item.clone());
                        }
                    }
                }
            }
        }
    }
    // The global steps of the target's submodel boundary conditions stay as a results link
    // set them.
    let global_steps = submodel_steps(target);
    // Items refer to each other by id: an item copied over the target's own of the same
    // name takes over the references to it.
    let before = target.item_ids();
    match category {
        "Features" => {
            copy(
                &mut target.reference_points,
                &source.reference_points,
                |p| &p.name,
                name,
            );
            copy(
                &mut target.coordinate_systems,
                &source.coordinate_systems,
                |c| &c.name,
                name,
            );
            copy(&mut target.planes, &source.planes, |p| &p.name, name);
        }
        "Materials" => copy(&mut target.materials, &source.materials, |m| &m.name, name),
        CROSS_SECTIONS => copy(
            &mut target.cross_sections,
            &source.cross_sections,
            |c| &c.name,
            name,
        ),
        "Sections" => copy(&mut target.sections, &source.sections, |s| &s.name, name),
        "Constraints" => copy(
            &mut target.constraints,
            &source.constraints,
            |c| c.name(),
            name,
        ),
        "Contacts" => {
            copy(
                &mut target.contact_regions,
                &source.contact_regions,
                |r| &r.name,
                name,
            );
            // The settings shared by all regions come with the whole category.
            if matches!(name, Mode::All | Mode::Bundle { .. }) {
                target.contact_defaults = source.contact_defaults.clone();
                target.contact_detections = source.contact_detections.clone();
            }
            copy(
                &mut target.surface_interactions,
                &source.surface_interactions,
                |s| &s.name,
                name,
            );
            copy(
                &mut target.contact_pairs,
                &source.contact_pairs,
                |c| &c.name,
                name,
            );
            copy(&mut target.ties, &source.ties, |t| &t.name, name);
            copy(&mut target.node_ties, &source.node_ties, |t| &t.name, name);
            copy(&mut target.bolts, &source.bolts, |b| &b.name, name);
        }
        "Amplitudes" => copy(
            &mut target.amplitudes,
            &source.amplitudes,
            |a| &a.name,
            name,
        ),
        "Initial Conditions" => copy(
            &mut target.initial_conditions,
            &source.initial_conditions,
            |c| &c.name,
            name,
        ),
        "Steps" => {
            copy(&mut target.steps, &source.steps, |s| &s.name, name);
            target.adopt_history_outputs(source);
            for (step, global) in global_steps {
                set_global_step(target, &step, global);
            }
        }
        _ => {}
    }
    target.follow_replaced(&before);
}

/// The global step each step's submodel boundary conditions read, by step name.
fn submodel_steps(fe: &FeModel) -> Vec<(String, u32)> {
    (fe.steps.iter())
        .filter_map(|step| {
            (step.boundary_conditions.iter()).find_map(|bc| match bc.kind {
                BoundaryKind::Submodel { step: global, .. } => Some((step.name.clone(), global)),
                _ => None,
            })
        })
        .collect()
}

fn set_global_step(fe: &mut FeModel, step: &str, global: u32) -> bool {
    let mut changed = false;
    let Some(step) = fe.steps.iter_mut().find(|s| s.name == step) else {
        return false;
    };
    for bc in &mut step.boundary_conditions {
        if let BoundaryKind::Submodel { step, .. } = &mut bc.kind
            && *step != global
        {
            *step = global;
            changed = true;
        }
    }
    changed
}

/// The number the solver gives a step of the global model in its results: the active steps
/// counted from 1.
pub fn global_step_number(fe: &FeModel, step: &str) -> Option<u32> {
    let index = (fe.steps.iter().filter(|s| s.active)).position(|s| s.name == step)?;
    u32::try_from(index + 1).ok()
}

/// Makes `fe` a submodel whose step `step` reads the results of global step `global` from
/// `results`; returns whether anything changed.
pub fn apply_submodel(fe: &mut FeModel, step: &str, global: u32, results: PathBuf) -> bool {
    let mut changed = false;
    if fe.properties.kind != ModelKind::Submodel {
        fe.properties.kind = ModelKind::Submodel;
        changed = true;
    }
    if fe.properties.global_results.as_ref() != Some(&results) {
        fe.properties.global_results = Some(results);
        changed = true;
    }
    set_global_step(fe, step, global) || changed
}

/// Takes the global results from `fe` once its last results link is gone. It stays a
/// submodel, whose Global results row takes a new link; Model Properties makes it a general
/// model again.
pub fn clear_submodel(fe: &mut FeModel) {
    fe.properties.global_results = None;
}

/// What flows through the pin of a row: the type a link checks before anything is copied.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DataType {
    Geometry,
    Mesh,
    /// The setup of a model or a part of it: materials, steps and the like.
    Setup,
    /// The results of a step, also where a row takes them: the State row, a measure.
    Results,
    /// A number: a parameter, the value of a measure, an output of a study.
    Number,
    /// An input a study gives.
    Loop,
    /// A whole analysis, to sweep.
    Analysis,
}

impl DataType {
    /// The type of the row of `path`; `None` for rows without a pin of their own.
    pub fn of(path: &[String]) -> Option<Self> {
        Some(match path.first()?.as_str() {
            GEOMETRY => Self::Geometry,
            MESH => Self::Mesh,
            SETUP if is_step_input(path) => Self::Results,
            SETUP => Self::Setup,
            RESULTS | STATE | MEASURE | STEP_RESULT | GLOBAL_RESULTS => Self::Results,
            VALUE | PARAMETERS | OUTPUTS => Self::Number,
            LOOP => Self::Loop,
            ANALYSIS => Self::Analysis,
            _ => return None,
        })
    }

    /// Whether it is data of a model, which only goes between blocks of the same model
    /// space: numbers and loop values have none.
    pub fn has_space(self) -> bool {
        !matches!(self, Self::Number | Self::Loop)
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Geometry => "geometry",
            Self::Mesh => "mesh",
            Self::Setup => "setup",
            Self::Results => "results",
            Self::Number => "number",
            Self::Loop => "loop value",
            Self::Analysis => "analysis",
        }
    }
}

/// Whether a link from `item` of one analysis may go to `slot` of another, by one of two
/// rules. A reference goes from like to like: a row to the row of the same path, an item of
/// a category to the category or its item of another name, a part of a geometry to the
/// Geometry row. An operation link goes from the
/// results of a step to a row that uses results: the State row, the initial conditions, a
/// step of the setup or one of its inputs (see [`results_slot`]).
pub fn compatible(item: &[String], slot: &[String]) -> bool {
    match item.first().map(String::as_str) {
        Some(RESULTS) => item.len() == 2 && takes_results(slot),
        // A number goes to a parameter; whether its kind fits, the blocks tell.
        Some(VALUE) => {
            item.len() == 2
                && (matches!(slot, [cell, _] if cell == PARAMETERS || cell == VALUE)
                    || matches!(slot, [cell, ..] if cell == OUTPUTS && slot.len() <= 2))
        }
        // The Inputs row of a study adds a column, the row of a column gives it.
        Some(LOOP) => {
            item.len() <= 2 && matches!(slot, [cell, _] if cell == PARAMETERS || cell == VALUE)
        }
        Some(STATE) | None => false,
        Some(_) => match (item, slot) {
            ([a], [b]) if b == ANALYSIS => a == SETUP,
            // A part of a geometry goes to the Geometry row, with the other parts linked.
            ([a, _], [b]) if a == GEOMETRY => b == GEOMETRY,
            ([a], [b]) => a == b && b != RESULTS && b != STATE,
            ([a, ..], [b, ..]) if a == SETUP && b == SETUP && slot.len() <= 3 => {
                // A category or an item goes to the target's category, or into its setup.
                match (item.get(1), slot.get(1)) {
                    (None, None) => true,
                    (Some(c), Some(d)) => c == d,
                    _ => false,
                }
            }
            _ => false,
        },
    }
}

/// Whether the row of `path` takes the results of a step of another analysis.
pub fn takes_results(path: &[String]) -> bool {
    match path {
        [cell] => cell == STATE || cell == MEASURE || cell == STEP_RESULT || cell == GLOBAL_RESULTS,
        [cell, category] => cell == SETUP && category == INITIAL_CONDITIONS,
        [cell, category, _] if cell == SETUP && category == STEPS => true,
        _ => ResultsUse::of_slot(path).is_some(),
    }
}

/// Whether the row of `path` takes nothing but results: an input of a step.
pub fn is_step_input(path: &[String]) -> bool {
    matches!(path, [cell, steps, _, input]
        if cell == SETUP && steps == STEPS && STEP_INPUTS.contains(&input.as_str()))
}

/// The analyses that come before `target` through links, nearest first; the feedback of a
/// loop is left out.
pub fn upstream(links: &[Link], target: u64) -> Vec<u64> {
    let mut found = Vec::new();
    let mut queue = vec![target];
    while let Some(id) = queue.pop() {
        for link in links.iter().filter(|l| l.target == id && !is_feedback(l)) {
            if !found.contains(&link.source) && link.source != target {
                found.push(link.source);
                queue.push(link.source);
            }
        }
    }
    found
}

/// Whether a link from `source` to `target` would make a loop.
pub fn makes_cycle(links: &[Link], source: u64, target: u64) -> bool {
    source == target || upstream(links, source).contains(&target)
}

/// The analyses in an order in which every source comes before its targets.
pub fn order(ids: &[u64], links: &[Link]) -> Vec<u64> {
    let mut done: Vec<u64> = Vec::with_capacity(ids.len());
    let mut left: Vec<u64> = ids.to_vec();
    while !left.is_empty() {
        let before = left.len();
        left.retain(|&id| {
            let ready = (links.iter())
                .filter(|l| l.target == id && ids.contains(&l.source) && !is_feedback(l))
                .all(|l| done.contains(&l.source));
            if ready {
                done.push(id);
            }
            !ready
        });
        if left.len() == before {
            // A loop, which links never make; take the rest as they are.
            done.append(&mut left);
        }
    }
    done
}

/// What of a model the tree shows for a path: the category and its items, or the item.
pub fn tree_items(fe: &FeModel, path: &[String]) -> Vec<TreeItem> {
    let all = |category: &str| -> Vec<TreeItem> {
        let mut items = category_items(fe, category, None);
        items.extend(category_groups(category));
        items
    };
    match path {
        [cell] if cell == MESH => vec![TreeItem::Mesh],
        [cell] if cell == SETUP => CATEGORIES.iter().flat_map(|c| all(c)).collect(),
        [cell, category] if cell == SETUP => all(category),
        [cell, category, name] if cell == SETUP => category_items(fe, category, Some(name)),
        _ => Vec::new(),
    }
}

fn category_groups(category: &str) -> Vec<TreeItem> {
    match category {
        "Contacts" => vec![
            TreeItem::Group("Contacts"),
            TreeItem::Group(crate::tree::CONTACT_REGIONS),
            TreeItem::Group(crate::tree::ADVANCED_CONTACTS),
            TreeItem::Group("Surface Interactions"),
            TreeItem::Group("Contact Pairs"),
        ],
        "Features" => vec![TreeItem::Group("Features")],
        other => (CATEGORIES.iter())
            .find(|c| **c == other)
            .map(|c| vec![TreeItem::Group(c)])
            .unwrap_or_default(),
    }
}

fn category_items(fe: &FeModel, category: &str, name: Option<&str>) -> Vec<TreeItem> {
    let wanted = |n: &str| name.is_none_or(|m| m == n);
    let mut items = Vec::new();
    match category {
        "Features" => {
            for (i, p) in fe.reference_points.iter().enumerate() {
                if wanted(&p.name) {
                    items.push(TreeItem::ReferencePoint(i));
                }
            }
            for (i, c) in fe.coordinate_systems.iter().enumerate() {
                if wanted(&c.name) {
                    items.push(TreeItem::CoordinateSystem(i));
                }
            }
            for (i, p) in fe.planes.iter().enumerate() {
                if wanted(&p.name) {
                    items.push(TreeItem::Plane(i));
                }
            }
        }
        "Materials" => items.extend(
            (fe.materials.iter().enumerate())
                .filter(|(_, m)| wanted(&m.name))
                .map(|(i, _)| TreeItem::Material(i)),
        ),
        CROSS_SECTIONS => items.extend(
            (fe.cross_sections.iter().enumerate())
                .filter(|(_, c)| wanted(&c.name))
                .map(|(i, _)| TreeItem::CrossSection(i)),
        ),
        "Sections" => items.extend(
            (fe.sections.iter().enumerate())
                .filter(|(_, s)| wanted(&s.name))
                .map(|(i, _)| TreeItem::Section(i)),
        ),
        "Constraints" => items.extend(
            (fe.constraints.iter().enumerate())
                .filter(|(_, c)| wanted(c.name()))
                .map(|(i, _)| TreeItem::Constraint(i)),
        ),
        "Contacts" => {
            items.extend(
                (fe.contact_regions.iter().enumerate())
                    .filter(|(_, r)| wanted(&r.name))
                    .map(|(i, _)| TreeItem::ContactRegion(i)),
            );
            items.extend(
                (fe.surface_interactions.iter().enumerate())
                    .filter(|(_, s)| wanted(&s.name))
                    .map(|(i, _)| TreeItem::SurfaceInteraction(i)),
            );
            items.extend(
                (fe.contact_pairs.iter().enumerate())
                    .filter(|(_, c)| wanted(&c.name))
                    .map(|(i, _)| TreeItem::ContactPair(i)),
            );
            items.extend(
                (fe.ties.iter().enumerate())
                    .filter(|(_, t)| wanted(&t.name))
                    .map(|(i, _)| TreeItem::Tie(i)),
            );
            items.extend(
                (fe.node_ties.iter().enumerate())
                    .filter(|(_, t)| wanted(&t.name))
                    .map(|(i, _)| TreeItem::NodeTie(i)),
            );
            items.extend(
                (fe.bolts.iter().enumerate())
                    .filter(|(_, b)| wanted(&b.name))
                    .map(|(i, _)| TreeItem::Bolt(i)),
            );
        }
        "Amplitudes" => items.extend(
            (fe.amplitudes.iter().enumerate())
                .filter(|(_, a)| wanted(&a.name))
                .map(|(i, _)| TreeItem::Amplitude(i)),
        ),
        "Initial Conditions" => items.extend(
            (fe.initial_conditions.iter().enumerate())
                .filter(|(_, c)| wanted(&c.name))
                .map(|(i, _)| TreeItem::InitialCondition(i)),
        ),
        "Steps" => {
            for (s, step) in fe.steps.iter().enumerate().filter(|(_, s)| wanted(&s.name)) {
                items.push(TreeItem::Step(s));
                for group in [
                    "BCs",
                    "Loads",
                    "Field Outputs",
                    "History Outputs",
                    "Defined Fields",
                ] {
                    items.push(TreeItem::StepGroup(s, group));
                }
                let n = step.boundary_conditions.len();
                items.extend((0..n).map(|i| TreeItem::BoundaryCondition(s, i)));
                items.extend((0..step.loads.len()).map(|i| TreeItem::Load(s, i)));
                items.extend((0..step.field_outputs.len()).map(|i| TreeItem::FieldOutput(s, i)));
                let n = step.history_outputs.len();
                items.extend((0..n).map(|i| TreeItem::HistoryOutput(s, i)));
                let n = step.defined_fields.len();
                items.extend((0..n).map(|i| TreeItem::DefinedField(s, i)));
            }
        }
        _ => {}
    }
    items
}

/// The path a tree item belongs to, to find whether a link covers it.
pub fn path_of(fe: &FeModel, item: &TreeItem) -> Option<Vec<String>> {
    let entry = |category: &str, name: &str| {
        Some(vec![SETUP.to_owned(), category.to_owned(), name.to_owned()])
    };
    let category = |category: &str| Some(vec![SETUP.to_owned(), category.to_owned()]);
    match item {
        TreeItem::Mesh
        | TreeItem::Part(_)
        | TreeItem::NodeSet(_)
        | TreeItem::ElementSet(_)
        | TreeItem::Surface(_) => Some(vec![MESH.to_owned()]),
        TreeItem::MeshItem(_) | TreeItem::MeshOperation(_) => Some(vec![MESH.to_owned()]),
        TreeItem::Material(i) => entry("Materials", &fe.materials.get(*i)?.name),
        TreeItem::CrossSection(i) => entry(CROSS_SECTIONS, &fe.cross_sections.get(*i)?.name),
        TreeItem::Section(i) => entry("Sections", &fe.sections.get(*i)?.name),
        TreeItem::Constraint(i) => entry("Constraints", fe.constraints.get(*i)?.name()),
        TreeItem::SurfaceInteraction(i) => {
            entry("Contacts", &fe.surface_interactions.get(*i)?.name)
        }
        TreeItem::ContactPair(i) => entry("Contacts", &fe.contact_pairs.get(*i)?.name),
        TreeItem::ContactRegion(i) => entry("Contacts", &fe.contact_regions.get(*i)?.name),
        TreeItem::Tie(i) => entry("Contacts", &fe.ties.get(*i)?.name),
        TreeItem::NodeTie(i) => entry("Contacts", &fe.node_ties.get(*i)?.name),
        TreeItem::Bolt(i) => entry("Contacts", &fe.bolts.get(*i)?.name),
        TreeItem::Amplitude(i) => entry("Amplitudes", &fe.amplitudes.get(*i)?.name),
        TreeItem::InitialCondition(i) => {
            entry("Initial Conditions", &fe.initial_conditions.get(*i)?.name)
        }
        TreeItem::ReferencePoint(i) => entry("Features", &fe.reference_points.get(*i)?.name),
        TreeItem::CoordinateSystem(i) => entry("Features", &fe.coordinate_systems.get(*i)?.name),
        TreeItem::Plane(i) => entry("Features", &fe.planes.get(*i)?.name),
        TreeItem::Step(s)
        | TreeItem::StepGroup(s, _)
        | TreeItem::BoundaryCondition(s, _)
        | TreeItem::Load(s, _)
        | TreeItem::FieldOutput(s, _)
        | TreeItem::HistoryOutput(s, _)
        | TreeItem::DefinedField(s, _) => entry("Steps", &fe.steps.get(*s)?.name),
        TreeItem::Group(
            "Surface Interactions"
            | "Contact Pairs"
            | "Contacts"
            | crate::tree::CONTACT_REGIONS
            | crate::tree::ADVANCED_CONTACTS,
        ) => category("Contacts"),
        TreeItem::Group(name) if CATEGORIES.contains(name) => category(name),
        _ => None,
    }
}

/// The paths of a model's tree that links of other analyses fill, with the name of the
/// analysis each comes from.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Incoming {
    pub paths: Vec<(Vec<String>, String)>,
    pub continuation: Continuation,
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_model::{BoundaryCondition, Material, Region, Step};

    #[test]
    fn meshes_put_together_keep_their_own_nodes_and_names() {
        use plx_mesh::{Element, ElementShape, Part};
        let tet = |offset: f64, set: &str| {
            let mut mesh = FeMesh::default();
            for (i, p) in [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
            .iter()
            .enumerate()
            {
                mesh.set_node(i as u32 + 1, [p[0] + offset, p[1], p[2]]);
            }
            mesh.add_element(Element {
                id: 1,
                type_name: "C3D4".into(),
                shape: ElementShape::Tet4,
                nodes: vec![1, 2, 3, 4],
            })
            .unwrap();
            mesh.parts.push(Part::new("BLOCK", vec![1]));
            mesh.node_sets.insert(set.into(), vec![1, 2]);
            MeshCopy {
                geometry: None,
                mesh,
                mesh_inputs: BTreeMap::from([("BLOCK".to_owned(), 7)]),
                mesh_sets: vec![plx_model::MeshSet::new(
                    set,
                    plx_model::SetKind::Nodes,
                    Region::Nodes(vec![1, 2]),
                )],
                element_types: vec![plx_model::PartElementType {
                    part: "BLOCK".into(),
                    type_name: "C3D4".into(),
                }],
                mesh_operations: Vec::new(),
            }
        };
        let (merged, renamed) = merge_meshes(vec![tet(0.0, "FIX"), tet(5.0, "FIX")], None);
        let mesh = &merged.mesh;
        assert_eq!(mesh.node_ids(), [1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(mesh.elements()[1].nodes, [5, 6, 7, 8]);
        let names: Vec<&str> = mesh.parts.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(names, ["BLOCK", "BLOCK-2"]);
        assert_eq!(mesh.parts[1].elements, [2]);
        assert_eq!(mesh.node_sets["FIX-2"], [5, 6]);
        assert_eq!(merged.mesh_sets[1].name, "FIX-2");
        assert_eq!(merged.mesh_sets[1].region, Region::Nodes(vec![5, 6]));
        assert_eq!(merged.element_types[1].part, "BLOCK-2");
        // Parts as meshed: not out of date with a geometry of their own.
        assert!(merged.mesh_inputs.is_empty());
        assert!(renamed[0].is_empty());
        assert_eq!(renamed[1]["BLOCK"], "BLOCK-2");
    }

    fn path(parts: &[&str]) -> Vec<String> {
        parts.iter().map(|p| p.to_string()).collect()
    }

    fn fp(fe: &FeModel, path: &[String]) -> u64 {
        let (mesh, inputs) = (FeMesh::default(), BTreeMap::new());
        let data = Data {
            geometry: None,
            mesh: &mesh,
            fe,
            mesh_inputs: &inputs,
        };
        fingerprint(data, path)
    }

    #[test]
    fn a_geometry_link_takes_the_shapes_and_leaves_the_mesh_setup_to_the_target() {
        let shapes = |brep: &str| Geometry {
            compounds: Vec::new(),
            source: "part.step".into(),
            brep: brep.into(),
            meshing: Default::default(),
            mesh_items: Vec::new(),
            part_names: Vec::new(),
            refinement: 1.0,
            signatures: Vec::new(),
        };
        let mut own = shapes("old");
        own.meshing.max_size = 2.0;
        own.mesh_items.push(plx_model::MeshSetupItem {
            name: "Local Size-1".into(),
            kind: plx_model::MeshSetupKind::LocalMeshSize {
                faces: vec![1],
                edges: Vec::new(),
                size: 0.5,
            },
        });
        own.refinement = 2.0;
        // The setup of the target counts as nothing the link takes.
        let print = |geometry: &Geometry| {
            let (mesh, fe, inputs) = (FeMesh::default(), FeModel::default(), BTreeMap::new());
            let data = Data {
                geometry: Some(geometry),
                mesh: &mesh,
                fe: &fe,
                mesh_inputs: &inputs,
            };
            fingerprint(data, &path(&[GEOMETRY]))
        };
        assert_eq!(print(&own), print(&shapes("old")));
        assert_ne!(print(&own), print(&shapes("new")));
        let mut target = Some(own.clone());
        take_geometry(&mut target, Some(shapes("new")));
        let taken = target.unwrap();
        assert_eq!(taken.brep, "new");
        assert_eq!(taken.meshing, own.meshing);
        assert_eq!(taken.mesh_items, own.mesh_items);
        assert_eq!(taken.refinement, 2.0);
        // Without geometry yet, the target starts from the source's; unlinked, it has none.
        let mut target = None;
        take_geometry(&mut target, Some(own.clone()));
        assert_eq!(target.as_ref(), Some(&own));
        take_geometry(&mut target, None);
        assert_eq!(target, None);
    }

    #[test]
    fn changed_shapes_take_the_selections_and_local_sizes_with_them() {
        use plx_mesh::CadEntity::{Edge, Face};
        let face = |entity, z: f64| plx_model::EntitySignature {
            entity,
            kind: "Plane".into(),
            centroid: [0.5, 0.5, z],
            min: [0.0, 0.0, z],
            max: [1.0, 1.0, z],
            size: 1.0,
        };
        let shapes = |brep: &str, signatures| Geometry {
            compounds: Vec::new(),
            source: "part.step".into(),
            brep: brep.into(),
            meshing: Default::default(),
            mesh_items: vec![plx_model::MeshSetupItem {
                name: "Local Size-1".into(),
                kind: plx_model::MeshSetupKind::LocalMeshSize {
                    faces: vec![1, 2],
                    edges: Vec::new(),
                    size: 0.5,
                },
            }],
            part_names: Vec::new(),
            refinement: 1.0,
            signatures,
        };
        // The bottom face 1 is now 7; the top face 2 went away.
        let own = shapes("old", vec![face(Face(1), 0.0), face(Face(2), 1.0)]);
        let source = shapes("new", vec![face(Face(7), 0.0), face(Face(8), 5.0)]);
        let mut target = Some(own);
        let tags = take_geometry(&mut target, Some(source)).unwrap();
        let plx_model::MeshSetupKind::LocalMeshSize { faces, .. } =
            &target.unwrap().mesh_items[0].kind
        else {
            unreachable!()
        };
        assert_eq!(faces, &[7]);
        let mut fe = FeModel::default();
        let region = plx_model::Region::Geometry(vec![Face(1), Face(2), Edge(3)]);
        let set = plx_model::MeshSet::new("Set-1", plx_model::SetKind::Nodes, region);
        fe.mesh_sets.push(set);
        let lost = follow_geometry(&mut fe, None, &tags);
        assert_eq!(
            lost.as_deref(),
            Some(
                "2 picked geometry entities were not found in the changed geometry; pick them \
                 again"
            )
        );
        let plx_model::Region::Geometry(entities) = &fe.mesh_sets[0].region else {
            unreachable!()
        };
        assert_eq!(entities, &[Face(7), Face(-2), Edge(-3)]);
        assert_eq!(
            plx_model::describe_entities(entities),
            "2 faces, 1 edge (2 not found)"
        );
        // Lost ones stay lost on the next change, and the same shapes renumber nothing.
        let next = BTreeMap::from([(Face(7), Face(9))]);
        assert_eq!(follow_geometry(&mut fe, None, &next), None);
        let plx_model::Region::Geometry(entities) = &fe.mesh_sets[0].region else {
            unreachable!()
        };
        assert_eq!(entities, &[Face(9), Face(-2), Edge(-3)]);
        let same = shapes("new", Vec::new());
        assert!(same.tags_in(&same).is_none());
    }

    fn material(name: &str, young: f64) -> Material {
        Material {
            name: name.into(),
            elastic: Some(plx_model::Elastic {
                young,
                poisson: 0.3,
            }),
            ..Default::default()
        }
    }

    #[test]
    fn an_item_replaces_the_one_of_the_same_name_and_leaves_the_others() {
        let source = FeModel {
            materials: vec![
                material("Steel", 210_000.0),
                material("Aluminium", 70_000.0),
            ],
            ..Default::default()
        };
        let mut target = FeModel {
            materials: vec![material("Steel", 1.0), material("Rubber", 5.0)],
            ..Default::default()
        };
        let steel = path(&["Setup", "Materials", "Steel"]);
        assert_ne!(fp(&source, &steel), fp(&target, &steel));
        apply_setup(&mut target, &source, &steel);
        let names: Vec<&str> = target.materials.iter().map(|m| m.name.as_str()).collect();
        assert_eq!(names, ["Steel", "Rubber"]);
        assert_eq!(fp(&source, &steel), fp(&target, &steel));
        apply_setup(&mut target, &source, &path(&["Setup", "Materials"]));
        assert_eq!(target.materials, source.materials);
    }

    #[test]
    fn a_category_link_is_a_bundle_of_item_links() {
        let mut source = FeModel {
            materials: vec![
                material("Steel", 210_000.0),
                material("Aluminium", 70_000.0),
            ],
            ..Default::default()
        };
        let mut target = FeModel {
            materials: vec![material("Steel", 1.0), material("Rubber", 5.0)],
            ..Default::default()
        };
        let mut link = Link {
            source: 1,
            target: 2,
            item: path(&["Setup", "Materials"]),
            ..Default::default()
        };
        assert!(is_bundle(&link));
        assert_eq!(setup_category(&link), Some("Materials"));
        let names =
            |fe: &FeModel| -> Vec<String> { fe.materials.iter().map(|m| m.name.clone()).collect() };
        let sync = |source: &FeModel, target: &mut FeModel, link: &mut Link| {
            let offered = offered(source, link);
            if bundle_up_to_date(source, target, link, &offered) {
                return false;
            }
            link.inherited = apply_bundle(target, source, link, &offered, &[]);
            true
        };
        // The linked items come first, the own one stays.
        assert!(sync(&source, &mut target, &mut link));
        assert_eq!(names(&target), ["Steel", "Aluminium", "Rubber"]);
        assert_eq!(target.materials[0], source.materials[0]);
        assert!(!sync(&source, &mut target, &mut link));
        // The tree marks the taken items, not the category, so own items stay editable.
        let filled: Vec<Vec<String>> = (filled_paths(&link, "Source").into_iter())
            .map(|(p, _)| p)
            .collect();
        assert_eq!(
            filled,
            [
                path(&["Setup", "Materials", "Steel"]),
                path(&["Setup", "Materials", "Aluminium"])
            ]
        );
        // New items of the source follow, removed ones go.
        source.materials.push(material("Copper", 110_000.0));
        source.materials.remove(1);
        assert!(sync(&source, &mut target, &mut link));
        assert_eq!(names(&target), ["Steel", "Copper", "Rubber"]);
        // A left out item stays as a copy and is no longer taken.
        link.excluded.push("Steel".into());
        link.inherited.retain(|n| n != "Steel");
        source.materials[0].elastic = None;
        assert!(!sync(&source, &mut target, &mut link));
        assert_eq!(link.inherited, ["Copper"]);
        assert!(target.materials[0].elastic.is_some());
        assert_eq!(names(&target), ["Steel", "Copper", "Rubber"]);
    }

    #[test]
    fn linked_steps_come_before_the_own_ones_in_the_order_of_the_links() {
        let model = |names: &[&str]| {
            let mut fe = FeModel::default();
            fe.steps.extend(names.iter().map(|n| Step::new_static(*n)));
            fe
        };
        let (first, second) = (model(&["Preload"]), model(&["Service"]));
        let mut target = model(&["Overload"]);
        let link = |source| Link {
            source,
            target: 3,
            item: path(&["Setup", "Steps"]),
            ..Default::default()
        };
        let preload = path(&["Preload"]);
        apply_bundle(&mut target, &first, &link(1), &preload, &[]);
        apply_bundle(
            &mut target,
            &second,
            &link(2),
            &path(&["Service"]),
            &preload,
        );
        let names: Vec<&str> = target.steps.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, ["Preload", "Service", "Overload"]);
    }

    #[test]
    fn steps_keep_the_global_step_a_results_link_set() {
        let mut step = Step::new_static("Step-1");
        step.boundary_conditions.push(BoundaryCondition {
            id: plx_model::Id::new(),
            distribution: None,
            name: "Submodel-1".into(),
            active: true,
            visible: true,
            region: Region::NodeSet("CUT".into()),
            kind: BoundaryKind::Submodel {
                step: 1,
                dofs: [true; 6],
            },
            amplitude: None,
        });
        let mut source = FeModel::default();
        source.steps.push(step);
        let mut target = source.clone();
        assert!(apply_submodel(
            &mut target,
            "Step-1",
            2,
            "global.frd".into()
        ));
        assert_eq!(target.properties.kind, ModelKind::Submodel);
        // The steps still count as the same, and taking them anew keeps global step 2.
        let steps = path(&["Setup", "Steps"]);
        assert_eq!(fp(&source, &steps), fp(&target, &steps));
        apply_setup(&mut target, &source, &steps);
        assert_eq!(submodel_steps(&target), [("Step-1".to_string(), 2)]);
        assert!(!apply_submodel(
            &mut target,
            "Step-1",
            2,
            "global.frd".into()
        ));
    }

    #[test]
    fn global_steps_count_the_active_steps() {
        let mut fe = FeModel::default();
        for name in ["Preload", "Off", "Service"] {
            fe.steps.push(Step::new_static(name));
        }
        fe.steps[1].active = false;
        assert_eq!(global_step_number(&fe, "Preload"), Some(1));
        assert_eq!(global_step_number(&fe, "Service"), Some(2));
        assert_eq!(global_step_number(&fe, "Off"), None);
    }

    #[test]
    fn links_never_make_a_loop_and_sources_come_first() {
        let link = |source, target| Link {
            source,
            target,
            item: path(&["Mesh"]),
            slot: Vec::new(),
            ..Default::default()
        };
        let links = vec![link(1, 2), link(2, 3)];
        assert!(makes_cycle(&links, 3, 1));
        assert!(makes_cycle(&links, 2, 2));
        assert!(!makes_cycle(&links, 1, 3));
        assert_eq!(order(&[3, 2, 1], &links), [1, 2, 3]);
        assert_eq!(upstream(&links, 3), [2, 1]);
    }

    #[test]
    fn the_feedback_of_a_study_is_left_out_of_the_order() {
        // Study 4 varies mesh block 1, analysis 2 solves on it, measure 3 reads it and the
        // study reads the measure.
        let link = |source, target, item: &[&str], slot: &[&str]| Link {
            source,
            target,
            item: path(item),
            slot: path(slot),
            ..Default::default()
        };
        let links = vec![
            link(4, 1, &["Loop", "Lx"], &["Parameters", "refinement"]),
            link(1, 2, &["Mesh"], &[]),
            link(2, 3, &["Results", "Step-1"], &["Measure"]),
            link(3, 4, &["Value", "Result"], &["Outputs", "Stress"]),
        ];
        assert!(is_feedback(&links[0]) && is_value(&links[0]));
        assert!(!is_feedback(&links[3]) && is_value(&links[3]));
        assert_eq!(order(&[4, 3, 2, 1], &links), [1, 2, 3, 4]);
        assert_eq!(upstream(&links, 1), Vec::<u64>::new());
        assert!(!makes_cycle(&links, 3, 4));
        // The inputs of a study go to parameters, numbers to its outputs; the Inputs and
        // Outputs rows add columns.
        assert!(compatible(
            &path(&["Loop", "Lx"]),
            &path(&["Parameters", "nx"])
        ));
        assert!(compatible(&path(&["Loop"]), &path(&["Parameters", "nx"])));
        assert!(!compatible(&path(&["Loop", "Lx"]), &path(&["Outputs"])));
        assert!(compatible(&path(&["Value", "Result"]), &path(&["Outputs"])));
        assert!(compatible(
            &path(&["Value", "Result"]),
            &path(&["Outputs", "Stress"])
        ));
    }

    #[test]
    fn a_state_link_takes_the_model_and_the_steps_up_to_its_step() {
        let mut source = FeModel {
            materials: vec![material("Steel", 210_000.0)],
            ..Default::default()
        };
        for name in ["Preload", "Service", "Overload"] {
            source.steps.push(Step::new_static(name));
        }
        let mut target = FeModel::default();
        target.steps.push(Step::new_static("Unload"));
        let mut link = Link {
            source: 1,
            target: 2,
            item: path(&["State", "Service"]),
            slot: path(&["State"]),
            ..Default::default()
        };
        let step = state_step(&source, &link).unwrap();
        assert!(!chain_up_to_date(&source, step, &target, &link));
        link.inherited = apply_chain(&mut target, &source, step, &link);
        assert_eq!(link.inherited, ["Preload", "Service"]);
        let names: Vec<&str> = target.steps.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, ["Preload", "Service", "Unload"]);
        assert_eq!(target.materials, source.materials);
        assert!(chain_up_to_date(&source, step, &target, &link));
        // A changed material or step of the source is taken again; the own step stays.
        source.materials[0].name = "S355".into();
        source.steps[0].name = "Bolts".into();
        assert!(!chain_up_to_date(&source, step, &target, &link));
        link.inherited = apply_chain(&mut target, &source, step, &link);
        let names: Vec<&str> = target.steps.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, ["Bolts", "Service", "Unload"]);
        assert_eq!(own_steps(&target, &link).len(), 1);
        // The tree shows the taken steps as linked, the own one not.
        let filled = filled_paths(&link, "Source");
        assert!(
            filled
                .iter()
                .any(|(p, _)| *p == path(&["Setup", "Steps", "Bolts"]))
        );
        assert!(filled.iter().any(|(p, _)| *p == path(&["Mesh"])));
        assert!(
            !filled
                .iter()
                .any(|(p, _)| covers(p, &path(&["Setup", "Steps", "Unload"])))
        );
        assert_eq!(output_path(&link), path(&["Results", "Service"]));
    }

    #[test]
    fn results_go_where_the_steps_at_both_ends_take_them() {
        let mut frequency = Step::new_frequency("Modes");
        let heat = Step::new_heat_transfer("Heat");
        let mut target = FeModel::default();
        target.steps.push(Step::new_static("Load"));
        target.steps.push(Step::new_steady_state_dynamics("Shake"));
        let slot = |parts: &[&str]| results_slot(&heat, &target, &path(parts));
        // Dropped on a step, they go to the input of it that fits.
        assert_eq!(
            slot(&["Setup", "Steps", "Load"]).unwrap(),
            path(&["Setup", "Steps", "Load", "Temperatures"])
        );
        assert_eq!(
            slot(&["Setup", "Steps", "Load", "Temperatures"]).unwrap(),
            path(&["Setup", "Steps", "Load", "Temperatures"])
        );
        assert!(slot(&["Setup", "Steps", "Load", "Eigenmodes"]).is_err());
        assert!(slot(&["Setup", "Steps", "Nowhere"]).is_err());
        assert_eq!(
            slot(&["Setup", "Initial Conditions"]).unwrap(),
            path(&["Setup", "Initial Conditions", "Temperature"])
        );
        let static_step = Step::new_static("Preload");
        assert!(
            results_slot(
                &static_step,
                &target,
                &path(&["Setup", "Initial Conditions"])
            )
            .is_err()
        );
        // Displacements of a static step drive a submodel.
        let load = path(&["Setup", "Steps", "Load"]);
        assert_eq!(
            results_slot(&static_step, &target, &load).unwrap(),
            path(&["Setup", "Steps", "Load", "Global results"])
        );
        let temperatures = path(&["Setup", "Steps", "Load", "Temperatures"]);
        assert!(results_slot(&static_step, &target, &temperatures).is_err());
        // Eigenmodes only when the frequency step stores them.
        let shake = path(&["Setup", "Steps", "Shake"]);
        assert!(results_slot(&frequency, &target, &shake).is_err());
        if let StepKind::Frequency(f) = &mut frequency.kind {
            f.storage = true;
        }
        let modes = results_slot(&frequency, &target, &shake).unwrap();
        assert_eq!(modes, path(&["Setup", "Steps", "Shake", "Eigenmodes"]));
        assert_eq!(
            step_inputs(&target.steps[0]),
            ["Temperatures", "Global results"]
        );
        assert!(step_inputs(&target.steps[1]).contains(&"Eigenmodes"));
        // The Global results row of a submodel's block: the step with submodel boundary
        // conditions takes them, else the first step.
        let global = path(&["Global results"]);
        assert_eq!(DataType::of(&global), Some(DataType::Results));
        assert!(compatible(&path(&["Results", "Preload"]), &global));
        assert!(!compatible(&path(&["Mesh"]), &global));
        assert_eq!(
            results_slot(&static_step, &target, &global).unwrap(),
            path(&["Setup", "Steps", "Load", "Global results"])
        );
        target.steps[1].boundary_conditions.push(BoundaryCondition {
            id: plx_model::Id::new(),
            distribution: None,
            name: "Cut".into(),
            active: true,
            visible: true,
            region: Region::Geometry(Vec::new()),
            kind: BoundaryKind::Submodel {
                step: 1,
                dofs: Default::default(),
            },
            amplitude: None,
        });
        assert_eq!(
            results_slot(&static_step, &target, &global).unwrap(),
            path(&["Setup", "Steps", "Shake", "Global results"])
        );
        let submodel = Link {
            item: path(&["Results", "Preload"]),
            slot: path(&["Setup", "Steps", "Shake", "Global results"]),
            ..Default::default()
        };
        assert!(ends_at(&global, &submodel) && ends_at(&path(&["Setup", "Steps"]), &submodel));
        assert!(!ends_at(&path(&["State"]), &submodel));

        let link = |item: &[&str], slot: Vec<String>| Link {
            source: 1,
            target: 2,
            item: path(item),
            slot,
            ..Default::default()
        };
        let temperature = link(&["Results", "Heat"], temperatures);
        assert_eq!(ResultsUse::of(&temperature), Some(ResultsUse::Temperature));
        assert_eq!(target_step(&temperature), Some("Load"));
        assert!(apply_results(
            &mut target,
            &temperature,
            1,
            "heat.frd".into()
        ));
        assert!(!apply_results(
            &mut target,
            &temperature,
            1,
            "heat.frd".into()
        ));
        assert_eq!(target.result_files(), [std::path::Path::new("heat.frd")]);
        let modes = link(&["Results", "Modes"], modes);
        assert_eq!(ResultsUse::of(&modes), Some(ResultsUse::Modes));
        assert!(apply_results(&mut target, &modes, 1, "modes.eig".into()));
        assert_eq!(target.properties.stored_modes, Some("modes.eig".into()));
        clear_results(&mut target, &temperature, false);
        clear_results(&mut target, &modes, false);
        assert!(target.result_files().is_empty());
        assert_eq!(target.properties.stored_modes, None);
    }

    #[test]
    fn references_go_like_to_like_and_results_to_where_they_are_used() {
        assert!(compatible(&path(&["Mesh"]), &path(&["Mesh"])));
        assert!(!compatible(&path(&["Mesh"]), &path(&["Geometry"])));
        assert!(compatible(&path(&["Setup"]), &path(&["Analysis"])));
        assert!(!compatible(&path(&["Mesh"]), &path(&["Analysis"])));
        let steel = path(&["Setup", "Materials", "Steel"]);
        assert!(compatible(&steel, &path(&["Setup", "Materials"])));
        assert!(compatible(&steel, &path(&["Setup", "Materials", "Rubber"])));
        assert!(!compatible(&steel, &path(&["Setup", "Sections"])));
        assert!(!compatible(&steel, &path(&["Setup"])));
        assert!(compatible(&path(&["Setup"]), &path(&["Setup"])));
        let step = path(&["Setup", "Steps", "Step-1"]);
        assert!(compatible(&step, &path(&["Setup", "Steps", "Step-2"])));
        // A step's definition never goes to an input of a step.
        let input = path(&["Setup", "Steps", "Step-2", "Temperatures"]);
        assert!(!compatible(&step, &input));
        // Results only give: nothing goes to them, not even other results.
        let results = path(&["Results", "Step-1"]);
        assert!(!compatible(&results, &path(&["Results", "Step-2"])));
        assert!(!compatible(&path(&["Results"]), &path(&["Results"])));
        assert!(compatible(&results, &step));
        assert!(compatible(&results, &input));
        assert!(compatible(
            &results,
            &path(&["Setup", "Steps", "Step-2", "Global results"])
        ));
        assert!(!compatible(&results, &path(&["Setup", "Steps"])));
        assert!(compatible(&results, &path(&["State"])));
        assert!(!compatible(&path(&["Mesh"]), &path(&["State"])));
        assert!(compatible(
            &results,
            &path(&["Setup", "Initial Conditions"])
        ));
        assert!(!compatible(&results, &path(&["Setup", "Materials"])));
        assert!(is_step_input(&input));
        assert!(!is_step_input(&step));
        let value = path(&["Value", "length"]);
        assert!(compatible(&value, &path(&["Parameters", "lx"])));
        assert!(!compatible(&value, &path(&["Mesh"])));
        assert!(!compatible(&path(&["Mesh"]), &path(&["Parameters", "lx"])));
        assert!(is_collection(&path(&["Setup", "Materials"])));
        assert!(!is_collection(&path(&["Setup", "Materials", "Steel"])));
        assert!(!is_collection(&path(&["Mesh"])));
    }

    #[test]
    fn split_and_merge_move_the_steps() {
        let mut fe = FeModel::default();
        for name in ["Preload", "Service", "Unload"] {
            fe.steps.push(Step::new_static(name));
        }
        let mut new = FeModel::default();
        let mut kept = split_after(&fe, 0, &mut new);
        assert_eq!(names(&kept.steps), ["Preload"]);
        assert_eq!(names(&new.steps), ["Service", "Unload"]);
        // The new analysis continues the kept one, as the sync after the split makes it.
        let mut link = Link {
            source: 1,
            target: 2,
            item: path(&["State", "Preload"]),
            slot: path(&["State"]),
            ..Default::default()
        };
        link.inherited = apply_chain(&mut new, &kept, 0, &link);
        assert_eq!(new.steps, fe.steps);
        assert!(chain_up_to_date(&kept, 0, &new, &link));
        // Merging gives back what was split.
        let moved = merge_into(&mut kept, &new, &link);
        assert_eq!(moved, ["Service", "Unload"]);
        assert_eq!(kept.steps, fe.steps);
    }
}
