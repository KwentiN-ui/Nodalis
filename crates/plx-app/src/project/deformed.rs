//! The Deformed Mesh block of the project schematic: the mesh of the analysis whose step
//! results are linked to it, each node moved by its displacement in one increment or mode of
//! that step times a factor. Its mesh goes on to the Mesh row of another analysis, e.g. a
//! buckling mode as the imperfection of a nonlinear static analysis, which PrePoMax does by
//! exporting the deformed mesh and importing it into another model.
//!
//! The mesh keeps the parts, sets, surfaces and the map of the CAD entities of the analysis
//! it comes from, so that what an analysis defines on the original mesh finds its regions on
//! the deformed one too. Reading the results takes a while: the mesh is made on a worker
//! thread, whenever the results, the mesh, the increment or the factor changed.

use plx_io::project::BoundResults;
use plx_mesh::FeMesh;

use super::*;

/// The keys of its factor and increment among its parameters, `["Parameters", key]`.
pub(super) const DEFORM_FACTOR: &str = "factor";
pub(super) const DEFORM_INCREMENT: &str = "increment";

/// A deformed mesh block: its numbers, the mesh it made and from what, and the mesh being
/// made.
pub(super) struct DeformedBlock {
    /// The mesh as last made, given on through links.
    pub(super) model: LinkedModel,
    pub(super) factor: f64,
    /// The increment of the step, or its mode for a buckling or frequency step, from 1.
    pub(super) increment: u32,
    /// The model space of the analysis it deforms the mesh of, once known.
    pub(super) space: Option<ModelSpace>,
    /// Fingerprint of what the mesh was made from, see [`wanted`].
    pub(super) made_from: Option<u64>,
    /// The largest displacement of a node as made, in SI units.
    pub(super) largest: Option<f64>,
    /// Why making it failed, for that fingerprint.
    pub(super) failed: Option<(u64, String)>,
    /// The mesh a worker is making, for that fingerprint.
    pub(super) making: Option<(u64, Receiver<Result<Made, String>>)>,
    /// What it is to be made from, as last found by [`refresh_wants`]: hashing the mesh of
    /// the analysis in every frame would cost too much for a large one.
    pub(super) want: Option<Want>,
}

/// The fingerprint of what a deformed mesh block is to make its mesh from now, see
/// [`wanted`], and whether the results it takes are up to date.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Want {
    pub(super) key: u64,
    pub(super) fresh: bool,
}

/// What a worker made.
pub(super) struct Made {
    model: LinkedModel,
    largest: f64,
    /// The units of the analysis, which the output gives the displacement in.
    units: UnitSystem,
}

impl DeformedBlock {
    pub(super) fn new(factor: f64, increment: u32) -> Self {
        Self {
            model: LinkedModel::default(),
            factor,
            increment,
            space: None,
            made_from: None,
            largest: None,
            failed: None,
            making: None,
            want: None,
        }
    }

    /// A block read from the project file, with the mesh it made.
    pub(super) fn read(block: file::DeformedMesh) -> Self {
        let mut model = LinkedModel {
            mesh: block.mesh,
            ..Default::default()
        };
        model.fe.mesh_sets = block.mesh_sets;
        model.fe.element_types = block.element_types;
        model.fe.properties.space = block.space;
        let made = model.mesh.element_count() > 0;
        Self {
            model,
            factor: clean_factor(block.factor),
            increment: block.increment.max(1),
            space: made.then_some(block.space),
            made_from: block.made_from.filter(|_| made),
            largest: block.largest,
            failed: None,
            making: None,
            want: None,
        }
    }

    /// The block as the project file keeps it.
    pub(super) fn written(&self, id: u64, name: String, position: [f32; 2]) -> file::DeformedMesh {
        let made = self.made_from.is_some();
        file::DeformedMesh {
            id,
            name,
            position,
            factor: self.factor,
            increment: self.increment,
            mesh: if made {
                self.model.mesh.clone()
            } else {
                Default::default()
            },
            mesh_sets: if made {
                self.model.fe.mesh_sets.clone()
            } else {
                Vec::new()
            },
            element_types: if made {
                self.model.fe.element_types.clone()
            } else {
                Vec::new()
            },
            space: self.space.unwrap_or_default(),
            made_from: self.made_from,
            largest: self.largest,
        }
    }

    /// What links read of it: the mesh it made, which stays given while a new one is made.
    pub(super) fn link_data(&self) -> Option<links::Data<'_>> {
        (self.model.mesh.element_count() > 0).then(|| self.model.data())
    }

    pub(super) fn set_factor(&mut self, value: f64) {
        if value.is_finite() {
            self.factor = value;
        }
    }

    pub(super) fn set_increment(&mut self, value: f64) {
        if value.is_finite() {
            self.increment = ValueKind::Count.clean(value) as u32;
        }
    }

    /// The state of its mesh: missing without results to take the displacements from,
    /// running while it is made, out of date while the results are or once something it is
    /// made from changed.
    pub(super) fn state(&self) -> CellState {
        if self.making.is_some() {
            return CellState::Running;
        }
        let Some(want) = self.want else {
            return CellState::Missing;
        };
        if !want.fresh {
            CellState::OutOfDate
        } else if (self.failed.as_ref()).is_some_and(|(key, _)| *key == want.key) {
            CellState::Failed
        } else if self.made_from == Some(want.key) && self.link_data().is_some() {
            CellState::UpToDate
        } else {
            CellState::OutOfDate
        }
    }

    /// Whether it is making its mesh, or is to start making it: its results are up to date
    /// and its mesh is not made from them, nor failed for them.
    pub(super) fn pending(&self) -> bool {
        self.making.is_some()
            || self.want.is_some_and(|w| {
                w.fresh
                    && self.made_from != Some(w.key)
                    && self.failed.as_ref().is_none_or(|(key, _)| *key != w.key)
            })
    }
}

/// A factor as kept: any finite number, 1 otherwise.
fn clean_factor(factor: f64) -> f64 {
    if factor.is_finite() { factor } else { 1.0 }
}

/// The link of the step results a deformed mesh block deforms the mesh with.
pub(super) fn deformed_link(links: &[Link], id: u64) -> Option<&Link> {
    (links.iter())
        .find(|l| l.target == id && matches!(links::slot(l), [cell] if cell == links::STEP_RESULT))
}

/// What a deformed mesh block is to make its mesh from now: the analysis, its results and the
/// number of the step in them, with the fingerprint of all that and of the block's numbers.
pub(super) struct Wanted<'a> {
    pub(super) source: &'a Node,
    pub(super) link: &'a Link,
    pub(super) bound: &'a BoundResults,
    pub(super) step: u32,
    pub(super) key: u64,
}

/// What the block `id` is to make its mesh from; `None` without a link, or while the analysis
/// linked has no results.
pub(super) fn wanted<'a>(nodes: &'a [Node], links: &'a [Link], id: u64) -> Option<Wanted<'a>> {
    let block = nodes.iter().find(|n| n.id == id)?.as_deformed()?;
    let link = deformed_link(links, id)?;
    let source = nodes.iter().find(|n| n.id == link.source)?;
    let bound = source.bound_results()?;
    let data = source.link_data()?;
    let name = link.item.get(1)?;
    let step = links::global_step_number(data.fe, name)?;
    let mut print = plx_mesh::Fnv::new();
    print.add(&source.step_key(name).to_le_bytes());
    print.add(&bound.input.unwrap_or_default().to_le_bytes());
    print.add(&step.to_le_bytes());
    print.add(&block.increment.to_le_bytes());
    print.add(&block.factor.to_bits().to_le_bytes());
    print.add(&links::fingerprint(data, &[links::MESH.to_owned()]).to_le_bytes());
    print.add_debug(&data.fe.properties.space);
    Some(Wanted {
        source,
        link,
        bound,
        step,
        key: print.finish(),
    })
}

/// Finds what each deformed mesh block among `nodes` is to make its mesh from now, see
/// [`DeformedBlock::want`].
pub(super) fn refresh_wants(nodes: &mut [Node], links: &[Link]) {
    let wants: Vec<(u64, Option<Want>)> = (nodes.iter())
        .filter(|n| n.kind() == Kind::DeformedMesh)
        .map(|n| {
            let want = wanted(nodes, links, n.id).map(|w| Want {
                key: w.key,
                fresh: !w.source.out_of_date_for(w.link),
            });
            (n.id, want)
        })
        .collect();
    for (id, want) in wants {
        if let Some(block) = (nodes.iter_mut().find(|n| n.id == id)).and_then(Node::as_deformed_mut)
        {
            block.want = want;
        }
    }
}

/// The mesh moved by the displacements of increment `increment` of step `step` of the
/// results, times `factor`, with the largest displacement of a node. Nodes of shells and
/// beams that the solver expanded to solids move by the mean of the nodes they became, as
/// the input it ran tells.
pub(super) fn deform(
    mesh: &FeMesh,
    read: &plx_io::project::ReadResults,
    step: u32,
    increment: u32,
    factor: f64,
) -> Result<(FeMesh, f64), String> {
    let results = &read.results;
    let increments: Vec<&plx_results::Increment> = (results.increments.iter())
        .filter(|i| i.step == step)
        .collect();
    let numbers: Vec<u32> = (increments.iter())
        .map(|i| i.increment)
        .filter(|&n| n > 0)
        .collect();
    let Some(chosen) = increments.iter().find(|i| i.increment == increment) else {
        return Err(match (numbers.first(), numbers.last()) {
            (Some(first), Some(last)) if first == last => {
                format!("the step has only increment {first}, not {increment}")
            }
            (Some(first), Some(last)) => {
                format!("the step has increments {first} to {last}, not {increment}")
            }
            _ => "the results have no increments of the step".into(),
        });
    };
    let displacements = chosen
        .displacements()
        .ok_or("the results have no displacements in that increment")?;
    let expanded = (read.input.as_deref())
        .and_then(|text| plx_io::inp::read_inp_str(text, None).ok())
        .map(|input| plx_io::frd::expanded_nodes(&input.mesh, &results.mesh))
        .unwrap_or_default();
    let mut deformed = mesh.clone();
    let mut largest = 0.0_f64;
    let mut missing = 0;
    for (&node, point) in mesh.node_ids().iter().zip(mesh.coords()) {
        let indices = plx_io::frd::node_indices(&results.mesh, &expanded, node);
        let found: Vec<[f32; 3]> = (indices.iter())
            .filter_map(|&i| displacements.get(i).copied())
            .collect();
        if found.is_empty() {
            missing += 1;
            continue;
        }
        let mean = [0, 1, 2].map(|k| {
            found.iter().map(|u| f64::from(u[k])).sum::<f64>() / found.len() as f64 * factor
        });
        largest = largest.max(mean.iter().map(|u| u * u).sum::<f64>().sqrt());
        deformed.set_node(node, [0, 1, 2].map(|k| point[k] + mean[k]));
    }
    if missing > 0 {
        return Err(format!(
            "{missing} of {} nodes have no displacement in the results; solve the analysis \
             again",
            mesh.node_ids().len()
        ));
    }
    Ok((deformed, largest))
}

impl ProjectApp {
    /// Adds a deformed mesh block with the factor 1 for the first increment; returns its id.
    pub(super) fn add_deformed_mesh(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = super::block_name("Deformed Mesh");
        let block = DeformedBlock::new(1.0, 1);
        self.nodes.push(Node::new(
            id,
            name.clone(),
            Block::DeformedMesh(Box::new(block)),
        ));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: drag the results of a step to its Step Result row, e.g. of a \
             buckle step, and its mesh to the Mesh row of the analysis that starts from it."
        ));
        id
    }

    /// Makes the meshes of deformed mesh blocks whose results, mesh or numbers changed, on
    /// worker threads; [`ProjectApp::poll_deformed`] takes them.
    pub(super) fn evaluate_deformed(&mut self) {
        refresh_wants(&mut self.nodes, &self.links);
        let ids: Vec<u64> = (self.nodes.iter())
            .filter(|n| n.kind() == Kind::DeformedMesh)
            .map(|n| n.id)
            .collect();
        for id in ids {
            // One that is not to be made, or being made from what it is to be made from now.
            let due = (self.nodes.iter().find(|n| n.id == id))
                .and_then(Node::as_deformed)
                .is_some_and(|d| {
                    d.pending() && d.making.as_ref().map(|(key, _)| *key) != d.want.map(|w| w.key)
                });
            if !due {
                continue;
            }
            let Some(wanted) = wanted(&self.nodes, &self.links, id) else {
                continue;
            };
            let key = wanted.key;
            let Some(data) = wanted.source.link_data() else {
                continue;
            };
            let mut model = LinkedModel {
                mesh: data.mesh.clone(),
                ..Default::default()
            };
            model.fe.mesh_sets = data.fe.mesh_sets.clone();
            model.fe.element_types = data.fe.element_types.clone();
            let space = data.fe.properties.space;
            let units = data.fe.properties.units;
            model.fe.properties.space = space;
            let bound = wanted.bound.clone();
            let step = wanted.step;
            let dir = self.measure_dir(id, wanted.source, wanted.bound, "");
            let Some(block) = self.node(id).and_then(Node::as_deformed_mut) else {
                continue;
            };
            let (increment, factor) = (block.increment, block.factor);
            let (sender, receiver) = std::sync::mpsc::channel();
            std::thread::spawn(move || {
                let made = (|| {
                    let read = bound.read(&dir)?;
                    let (mesh, largest) = deform(&model.mesh, &read, step, increment, factor)?;
                    model.mesh = mesh;
                    Ok(Made {
                        model,
                        largest,
                        units,
                    })
                })();
                let _ = sender.send(made);
            });
            block.space = Some(space);
            block.making = Some((key, receiver));
        }
    }

    /// Takes the meshes the workers of deformed mesh blocks made; returns whether one came.
    pub(super) fn poll_deformed(&mut self) -> bool {
        let mut lines = Vec::new();
        for node in &mut self.nodes {
            let name = node.name.clone();
            let Some(block) = node.as_deformed_mut() else {
                continue;
            };
            let Some((key, receiver)) = &block.making else {
                continue;
            };
            let key = *key;
            let made = match receiver.try_recv() {
                Ok(made) => made,
                Err(std::sync::mpsc::TryRecvError::Empty) => continue,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    Err("making it stopped unexpectedly".into())
                }
            };
            block.making = None;
            match made {
                Ok(made) => {
                    let units = made.units;
                    lines.push(format!(
                        "{name}: mesh deformed, {} elements, largest displacement {} {}",
                        made.model.mesh.element_count(),
                        crate::results::format_value(
                            units.shown(made.largest, Quantity::Length) as f32
                        ),
                        units.unit(Quantity::Length)
                    ));
                    block.model = made.model;
                    block.largest = Some(made.largest);
                    block.made_from = Some(key);
                    block.failed = None;
                }
                Err(why) => {
                    lines.push(format!("{name}: no mesh made: {why}."));
                    block.failed = Some((key, why));
                }
            }
        }
        let came = !lines.is_empty();
        self.output.extend(lines);
        came
    }

    /// Whether a deformed mesh block is making its mesh.
    pub(super) fn deforming(&self) -> bool {
        (self.nodes.iter())
            .filter_map(Node::as_deformed)
            .any(|d| d.making.is_some())
    }
}

/// The rows of a deformed mesh block: the results it takes, its factor and increment, which
/// value links may give, and the mesh it gives with its state.
pub(super) fn deformed_rows(
    node: &Node,
    incoming: &[(Vec<String>, String)],
    state: CellState,
    units: UnitSystem,
) -> Vec<Row> {
    let Some(block) = node.as_deformed() else {
        return Vec::new();
    };
    let linked = |path: &[String]| {
        (incoming.iter())
            .find(|(slot, _)| slot.as_slice() == path)
            .map(|(_, source)| source.clone())
    };
    let row = |path: Vec<String>, text: String, icon, tip: String| Row {
        path,
        text,
        depth: 0,
        icon,
        state: None,
        expander: None,
        input: false,
        output: false,
        linked_from: None,
        collection: false,
        partial: false,
        widget: None,
        tip,
    };
    let results = vec![links::STEP_RESULT.to_owned()];
    let factor = vec![links::PARAMETERS.to_owned(), DEFORM_FACTOR.to_owned()];
    let increment = vec![links::PARAMETERS.to_owned(), DEFORM_INCREMENT.to_owned()];
    let mesh = vec![links::MESH.to_owned()];
    let count = block.model.mesh.element_count();
    let mut tip = "The deformed mesh: drag its pin to the Mesh row of an analysis, which \
                   starts from it, e.g. a nonlinear static analysis with a buckling mode as \
                   its imperfection. It keeps the parts and sets of the analysis it comes \
                   from."
        .to_owned();
    tip.push_str(&format!("\nMesh: {}", state.text()));
    match state {
        CellState::Missing => {
            tip.push_str(": drag the results of a step here, and solve the analysis they belong to")
        }
        CellState::OutOfDate => {
            tip.push_str(": Update Project solves the analysis and makes the mesh again")
        }
        CellState::Failed => {
            if let Some((_, why)) = &block.failed {
                tip.push_str(&format!(": {why}"));
            }
        }
        _ => {}
    }
    let text = match (state, block.largest) {
        (CellState::Running, _) => "Mesh: making …".to_owned(),
        (_, Some(largest)) if count > 0 => format!(
            "Mesh, max. {} {}",
            crate::results::format_value(units.shown(largest, Quantity::Length) as f32),
            units.unit(Quantity::Length)
        ),
        _ => links::MESH.to_owned(),
    };
    if count > 0 {
        tip.push_str(&format!(
            "\n{count} elements; the largest displacement of a node, times the factor, is \
             shown in the row."
        ));
    }
    vec![
        Row {
            input: true,
            linked_from: linked(&results),
            ..row(
                results,
                "Step Result".to_owned(),
                NodeIcon::Results,
                "The results of a step whose displacements move the nodes: drag the pin of \
                 a step under Results of an analysis here, e.g. a buckle step."
                    .to_owned(),
            )
        },
        Row {
            input: true,
            linked_from: linked(&increment),
            widget: Some(RowWidget::DeformIncrement),
            ..row(
                increment,
                "Mode".to_owned(),
                NodeIcon::Results,
                "Mode: the mode of a buckle or frequency step, or the increment of another \
                 step, counted from 1, whose displacements move the nodes."
                    .to_owned(),
            )
        },
        Row {
            input: true,
            linked_from: linked(&factor),
            widget: Some(RowWidget::DeformFactor),
            ..row(
                factor,
                "Factor".to_owned(),
                NodeIcon::Mesh,
                "Factor: the displacements are multiplied by it. Mode shapes are normalized, \
                 so the factor sets the size of an imperfection; the row of the mesh shows \
                 the largest displacement it gives. Drop an input of a parameter study here \
                 to vary it."
                    .to_owned(),
            )
        },
        // The pin is there before the mesh, so that the analyses after it can be linked
        // before anything is solved.
        Row {
            output: true,
            state: Some(state),
            ..row(mesh, text, NodeIcon::Mesh, tip)
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn testdata(name: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name)
    }

    #[test]
    fn the_nodes_move_by_the_displacements_times_the_factor() {
        let frd = plx_io::frd::read_frd(&testdata("kragbalken_c3d8.frd")).unwrap();
        let chosen = frd.increments.iter().rfind(|i| i.displacements().is_some());
        let chosen = chosen.unwrap();
        let (step, increment) = (chosen.step, chosen.increment);
        let u = chosen.displacements().unwrap();
        let mesh = frd.mesh.clone();
        let read = plx_io::project::ReadResults {
            results: frd,
            history: None,
            input: None,
        };
        let (deformed, largest) = deform(&mesh, &read, step, increment, -2.5).unwrap();
        assert_eq!(deformed.element_count(), mesh.element_count());
        assert_eq!(deformed.node_ids(), mesh.node_ids());
        let mut most = 0.0_f64;
        for (i, (a, b)) in mesh.coords().iter().zip(deformed.coords()).enumerate() {
            for k in 0..3 {
                let moved = a[k] - 2.5 * f64::from(u[i][k]);
                assert!((b[k] - moved).abs() < 1e-9, "node {i}: {b:?} {a:?}");
            }
            let length = u[i].map(|c| f64::from(c) * 2.5);
            most = most.max(length.iter().map(|c| c * c).sum::<f64>().sqrt());
        }
        assert!(largest > 0.0 && (largest - most).abs() < 1e-9 * most.max(1.0));
        // An increment the step does not have says which it has.
        let why = deform(&mesh, &read, step, 99, 1.0).unwrap_err();
        assert!(why.contains("not 99"), "{why}");
        // Nodes the results do not know make no mesh.
        let mut other = mesh.clone();
        other.set_node(9_999_999, [0.0; 3]);
        assert!(deform(&other, &read, step, increment, 1.0).is_err());
    }

    fn solved(mesh: &FeMesh) -> ProjectAnalysis {
        let mut model = plx_model::FeModel::default();
        model.steps.push(plx_model::Step::new_static("Step-1"));
        let results = plx_io::project::BoundResults {
            input: Some(crate::solve_fingerprint::solve_input(mesh, &model)),
            steps: crate::solve_fingerprint::step_inputs(mesh, &model),
            ..Default::default()
        };
        ProjectAnalysis {
            mesh: mesh.clone(),
            model,
            results: Some(results),
            ..Default::default()
        }
    }

    /// The state of block 2 once its wants are found.
    fn state(nodes: &mut [Node], links: &[Link]) -> CellState {
        refresh_wants(nodes, links);
        nodes[1].as_deformed().unwrap().state()
    }

    fn pending(nodes: &mut [Node], links: &[Link]) -> bool {
        refresh_wants(nodes, links);
        nodes[1].as_deformed().unwrap().pending()
    }

    #[test]
    fn a_deformed_mesh_block_is_up_to_date_with_the_results_it_was_made_from() {
        let primitive = MeshPrimitive::default_box();
        let mesh = primitive.generate(ModelSpace::ThreeD, "BOX");
        let mut buckle = Node::analysis(1, "Buckle".into(), solved(&mesh));
        buckle.refresh_closed();
        let block = DeformedBlock::new(1.0, 1);
        let deformed = Node::new(
            2,
            "Deformed Mesh".into(),
            Block::DeformedMesh(Box::new(block)),
        );
        let mut target = Node::analysis(3, "Static".into(), ProjectAnalysis::default());
        target.refresh_closed();
        let mut nodes = vec![buckle, deformed, target];
        let mut links = vec![Link {
            source: 2,
            target: 3,
            item: vec![links::MESH.into()],
            ..Default::default()
        }];
        // Nothing to deform yet.
        assert_eq!(state(&mut nodes, &links), CellState::Missing);
        assert!(!pending(&mut nodes, &links));
        assert_eq!(stale_meshes(&nodes, &links, 3), ["Deformed Mesh"]);
        // The results of a step go to the Step Result row, from an analysis only.
        let results = vec![links::RESULTS.to_owned(), "Step-1".to_owned()];
        let slot = vec![links::STEP_RESULT.to_owned()];
        assert!(links::compatible(&results, &slot));
        assert!(!links::compatible(&[links::MESH.to_owned()], &slot));
        assert!(refusal(&nodes, &links, 1, &results, 2, &slot).is_none());
        links.push(Link {
            source: 1,
            target: 2,
            item: results,
            slot,
            ..Default::default()
        });
        // Results to make it from: it waits to be made.
        assert_eq!(state(&mut nodes, &links), CellState::OutOfDate);
        assert!(pending(&mut nodes, &links));
        let key = wanted(&nodes, &links, 2).unwrap().key;
        let block = nodes[1].as_deformed_mut().unwrap();
        block.model.mesh = mesh.clone();
        block.made_from = Some(key);
        assert_eq!(state(&mut nodes, &links), CellState::UpToDate);
        assert!(!pending(&mut nodes, &links));
        assert!(stale_meshes(&nodes, &links, 3).is_empty());
        // Another factor makes it anew.
        nodes[1].as_deformed_mut().unwrap().set_factor(2.0);
        assert_eq!(state(&mut nodes, &links), CellState::OutOfDate);
        assert!(pending(&mut nodes, &links));
        // A failure is kept for what failed, and nothing is tried again.
        let key = wanted(&nodes, &links, 2).unwrap().key;
        nodes[1].as_deformed_mut().unwrap().failed = Some((key, "no".into()));
        assert_eq!(state(&mut nodes, &links), CellState::Failed);
        assert!(!pending(&mut nodes, &links));
        // Its numbers and its mesh read back.
        let block = nodes[1].as_deformed_mut().unwrap();
        block.set_increment(3.4);
        assert_eq!(block.increment, 3);
        let node = &mut nodes[1];
        let Some(ProjectNode::DeformedMesh(written)) = node.written([1.0, 2.0]) else {
            panic!("not written");
        };
        assert_eq!((written.factor, written.increment), (2.0, 3));
        assert_eq!(written.mesh.element_count(), mesh.element_count());
        let read = Node::read(ProjectNode::DeformedMesh(written));
        assert_eq!(read.kind(), Kind::DeformedMesh);
        let read = read.as_deformed().unwrap();
        assert_eq!(read.model.mesh.element_count(), mesh.element_count());
        assert_eq!(read.space, Some(ModelSpace::default()));
    }

    #[test]
    fn a_deformed_mesh_block_shows_its_rows() {
        let mut block = DeformedBlock::new(0.5, 2);
        block.model.mesh = MeshPrimitive::default_box().generate(ModelSpace::ThreeD, "BOX");
        block.largest = Some(0.002);
        let node = Node::new(
            2,
            "Deformed Mesh".into(),
            Block::DeformedMesh(Box::new(block)),
        );
        let incoming = vec![(vec![links::STEP_RESULT.to_owned()], "Buckle".to_owned())];
        let rows = deformed_rows(&node, &incoming, CellState::UpToDate, UnitSystem::MmTonSC);
        let paths: Vec<&[String]> = rows.iter().map(|r| r.path.as_slice()).collect();
        assert_eq!(paths[0], [links::STEP_RESULT]);
        assert_eq!(paths[1], [links::PARAMETERS, DEFORM_INCREMENT]);
        assert_eq!(paths[2], [links::PARAMETERS, DEFORM_FACTOR]);
        assert_eq!(paths[3], [links::MESH]);
        assert!(rows[0].input && rows[0].linked_from.as_deref() == Some("Buckle"));
        assert!(rows[3].output && !rows[3].input);
        assert_eq!(rows[3].text, "Mesh, max. 2 mm");
        let nodes = vec![node];
        assert_eq!(value_kind(&nodes, 2, &rows[1].path), Some(ValueKind::Count));
        assert_eq!(
            value_kind(&nodes, 2, &rows[2].path),
            Some(ValueKind::Factor)
        );
    }
}
