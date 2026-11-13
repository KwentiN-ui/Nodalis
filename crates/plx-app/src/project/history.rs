//! Undo and redo of what the user changed in the project window: blocks added, deleted,
//! renamed and moved, links made and removed, what the blocks that are no analysis hold, and
//! the steps Split and Merge move between analyses.
//!
//! What is changed inside an analysis is undone in its window, by its own history. The
//! project keeps the state before each change as a [`Snapshot`]: the blocks with their
//! names and positions, the data of the blocks that are no analysis as the project file
//! keeps them, and the links. The data of an analysis is large; a snapshot keeps it only
//! for the analyses a change rewrites, and a deleted block is kept whole with the change.

use super::*;

/// Changes that can be undone at most.
const DEPTH: usize = 100;

/// A block as a change can be undone to.
struct Kept {
    id: u64,
    name: String,
    position: [f32; 2],
    /// What a block that is no analysis holds, as the project file keeps it.
    data: Option<ProjectNode>,
    /// The data of an analysis the change rewrote.
    analysis: Option<ProjectAnalysis>,
}

/// The project as a change can be undone to.
struct Snapshot {
    blocks: Vec<Kept>,
    links: Vec<Link>,
    frames: Vec<file::Frame>,
    compact: BTreeSet<u64>,
}

impl Snapshot {
    /// Whether the project is still as `self` keeps it; positions apart, which are compared
    /// on their own.
    fn same(&self, other: &Self) -> bool {
        self.links == other.links
            && self.frames == other.frames
            && self.compact == other.compact
            && self.blocks.len() == other.blocks.len()
            && (self.blocks.iter().zip(&other.blocks)).all(|(a, b)| {
                a.id == b.id
                    && a.name == b.name
                    && a.analysis.is_none()
                    && same_data(a.data.as_ref(), b.data.as_ref())
            })
    }
}

/// Whether two blocks hold the same, leaving out what they computed.
fn same_data(a: Option<&ProjectNode>, b: Option<&ProjectNode>) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(ProjectNode::Materials(a)), Some(ProjectNode::Materials(b))) => a == b,
        (Some(ProjectNode::CrossSections(a)), Some(ProjectNode::CrossSections(b))) => a == b,
        // The mesh is made from the primitive.
        (Some(ProjectNode::Mesh(a)), Some(ProjectNode::Mesh(b))) => {
            a.space == b.space && a.units == b.units && a.primitive == b.primitive
        }
        (Some(ProjectNode::RefineMesh(a)), Some(ProjectNode::RefineMesh(b))) => {
            a.factor == b.factor
        }
        (Some(ProjectNode::DeformedMesh(a)), Some(ProjectNode::DeformedMesh(b))) => {
            a.factor == b.factor && a.increment == b.increment
        }
        (Some(ProjectNode::Parameters(a)), Some(ProjectNode::Parameters(b))) => a == b,
        (Some(ProjectNode::Measure(a)), Some(ProjectNode::Measure(b))) => {
            a.units == b.units && a.kind == b.kind
        }
        (Some(ProjectNode::Study(a)), Some(ProjectNode::Study(b))) => a == b,
        (Some(ProjectNode::MeshExport(a)), Some(ProjectNode::MeshExport(b))) => a.path == b.path,
        (Some(ProjectNode::File(a)), Some(ProjectNode::File(b))) => {
            a.path == b.path && a.units == b.units && a.embedded == b.embedded
        }
        (Some(ProjectNode::Campbell(a)), Some(ProjectNode::Campbell(b))) => a.setup == b.setup,
        (Some(ProjectNode::Note(a)), Some(ProjectNode::Note(b))) => a.text == b.text,
        (Some(ProjectNode::Math(a)), Some(ProjectNode::Math(b))) => a == b,
        (Some(ProjectNode::Switch(a)), Some(ProjectNode::Switch(b))) => a == b,
        (Some(ProjectNode::Formula(a)), Some(ProjectNode::Formula(b))) => a == b,
        (Some(ProjectNode::Plot(a)), Some(ProjectNode::Plot(b))) => a == b,
        (Some(ProjectNode::Preview(_)), Some(ProjectNode::Preview(_))) => true,
        (Some(ProjectNode::Group(a)), Some(ProjectNode::Group(b))) => {
            a.nodes == b.nodes && a.inputs == b.inputs && a.outputs == b.outputs
        }
        (Some(ProjectNode::GroupInput(_)), Some(ProjectNode::GroupInput(_)))
        | (Some(ProjectNode::GroupOutput(_)), Some(ProjectNode::GroupOutput(_))) => true,
        _ => false,
    }
}

/// A change that can be undone: the project before it, with the blocks it deleted.
struct Change {
    before: Snapshot,
    removed: Vec<Node>,
}

/// The changes of the project that can be undone and redone.
#[derive(Default)]
pub(super) struct History {
    undo: Vec<Change>,
    redo: Vec<Change>,
    /// The project before what the user does in this frame, until the frame ends.
    pending: Option<Snapshot>,
    /// Blocks deleted in this frame.
    removed: Vec<Node>,
    /// The positions of the blocks as last seen, to notice that they were moved.
    positions: Vec<(u64, [f32; 2])>,
    /// The highest id a block had: ids of deleted blocks are not given again, so that undo
    /// can bring them back.
    pub(super) highest_id: u64,
    /// Changes made, undone and redone so far, for autosave; another project does not
    /// start it again.
    pub(super) edits: u64,
}

impl History {
    pub(super) fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }

    pub(super) fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }

    /// Forgets every change, for another project.
    pub(super) fn clear(&mut self) {
        *self = Self {
            highest_id: self.highest_id,
            edits: self.edits,
            ..Self::default()
        };
    }

    fn push(&mut self, change: Change) {
        self.edits += 1;
        self.undo.push(change);
        self.redo.clear();
        if self.undo.len() > DEPTH {
            self.undo.remove(0);
        }
    }
}

impl ProjectApp {
    /// The positions of the blocks in the schematic.
    pub(super) fn positions(&self) -> Vec<(u64, [f32; 2])> {
        let mut positions: Vec<(u64, [f32; 2])> = (self.graph.nodes_pos_ids())
            .map(|(_, pos, id)| (*id, [pos.x, pos.y]))
            .collect();
        positions.sort_by_key(|(id, _)| *id);
        positions
    }

    /// The project as it is now, with the data of the analyses `rewritten`.
    fn snapshot(&mut self, rewritten: &[u64]) -> Snapshot {
        let positions = self.positions();
        let blocks = (self.nodes.iter_mut())
            .map(|node| {
                let position = (positions.iter())
                    .find(|(id, _)| *id == node.id)
                    .map_or([0.0; 2], |(_, pos)| *pos);
                let (data, analysis) = match node.kind() {
                    Kind::Analysis if rewritten.contains(&node.id) => {
                        match node.written(position) {
                            Some(ProjectNode::Analysis(data)) => (None, Some(*data)),
                            _ => (None, None),
                        }
                    }
                    Kind::Analysis => (None, None),
                    _ => (node.written(position), None),
                };
                Kept {
                    id: node.id,
                    name: node.name.clone(),
                    position,
                    data,
                    analysis,
                }
            })
            .collect();
        Snapshot {
            blocks,
            links: self.links.clone(),
            frames: self.frames.clone(),
            compact: self.compact.clone(),
        }
    }

    /// Notes the project before a change the user makes in this frame, with the data of the
    /// analyses the change rewrites; the change can be undone once the frame ends.
    pub(super) fn checkpoint(&mut self, rewritten: &[u64]) {
        match self.history.pending.take() {
            None => {
                let mut snapshot = self.snapshot(rewritten);
                // Blocks moved but not noted yet, as they are dragged, go back with the change.
                for kept in &mut snapshot.blocks {
                    if let Some((_, pos)) =
                        (self.history.positions.iter()).find(|(id, _)| *id == kept.id)
                    {
                        kept.position = *pos;
                    }
                }
                self.history.pending = Some(snapshot);
            }
            Some(mut pending) => {
                // A second change in the same frame: the analyses it rewrites are kept as
                // they were before it, unless the first one kept them already.
                let missing: Vec<u64> = (rewritten.iter().copied())
                    .filter(|id| {
                        (pending.blocks.iter()).any(|b| b.id == *id && b.analysis.is_none())
                    })
                    .collect();
                if !missing.is_empty() {
                    let now = self.snapshot(&missing);
                    for kept in now.blocks.into_iter().filter(|b| b.analysis.is_some()) {
                        if let Some(old) = pending.blocks.iter_mut().find(|b| b.id == kept.id) {
                            old.analysis = kept.analysis;
                        }
                    }
                }
                self.history.pending = Some(pending);
            }
        }
    }

    /// Keeps a deleted block with the change that deleted it, to bring it back.
    pub(super) fn keep_deleted(&mut self, node: Node) {
        self.history.highest_id = self.history.highest_id.max(node.id);
        self.history.removed.push(node);
    }

    /// At the end of every frame: what the user changed in it becomes a change that can be
    /// undone; so do blocks moved, once the mouse button is up.
    pub(super) fn record_change(&mut self, ctx: &egui::Context) {
        let highest = self.nodes.iter().map(|n| n.id).max().unwrap_or(0);
        self.history.highest_id = self.history.highest_id.max(highest);
        if let Some(before) = self.history.pending.take() {
            let removed = std::mem::take(&mut self.history.removed);
            let now = self.snapshot(&[]);
            if !removed.is_empty() || !before.same(&now) {
                self.history.push(Change { before, removed });
                self.history.positions = self.positions();
                return;
            }
        }
        if ctx.input(|i| i.pointer.any_down()) {
            return;
        }
        let positions = self.positions();
        if positions == self.history.positions {
            return;
        }
        let ids = |p: &[(u64, [f32; 2])]| p.iter().map(|(id, _)| *id).collect::<Vec<_>>();
        let moved = ids(&positions) == ids(&self.history.positions);
        let old = std::mem::replace(&mut self.history.positions, positions);
        if moved {
            let mut before = self.snapshot(&[]);
            for kept in &mut before.blocks {
                if let Some((_, pos)) = old.iter().find(|(id, _)| *id == kept.id) {
                    kept.position = *pos;
                }
            }
            self.history.push(Change {
                before,
                removed: Vec::new(),
            });
        }
    }

    /// Edit > Undo in the project window.
    pub(super) fn undo(&mut self, ctx: &egui::Context) {
        let Some(change) = self.history.undo.pop() else {
            return;
        };
        match self.restore(ctx, change) {
            Ok(change) => {
                self.history.redo.push(change);
                self.history.edits += 1;
                self.output.push("Undone".into());
            }
            Err(change) => self.history.undo.push(change),
        }
    }

    /// Edit > Redo in the project window.
    pub(super) fn redo(&mut self, ctx: &egui::Context) {
        let Some(change) = self.history.redo.pop() else {
            return;
        };
        match self.restore(ctx, change) {
            Ok(change) => {
                self.history.undo.push(change);
                self.history.edits += 1;
                self.output.push("Redone".into());
            }
            Err(change) => self.history.redo.push(change),
        }
    }

    /// Puts the project back to the state `change` keeps; returns the change that goes back
    /// to the state it had, or `change` if it cannot be done now.
    fn restore(&mut self, ctx: &egui::Context, change: Change) -> Result<Change, Change> {
        let Change {
            before,
            removed: mut kept_nodes,
        } = change;
        // Analyses whose window works on something cannot be deleted or rewritten now.
        let rewritten: Vec<u64> = (before.blocks.iter())
            .filter(|b| b.analysis.is_some())
            .map(|b| b.id)
            .collect();
        let gone: Vec<u64> = (self.nodes.iter())
            .filter(|n| !before.blocks.iter().any(|b| b.id == n.id))
            .map(|n| n.id)
            .collect();
        for id in rewritten.iter().chain(&gone) {
            if let Some(node) = self.nodes.iter().find(|n| n.id == *id)
                && let Some(reason) = node.window().and_then(AnalysisWindow::busy)
            {
                self.output
                    .push(format!("Not undone: {} is open and {reason}.", node.name));
                return Err(Change {
                    before,
                    removed: kept_nodes,
                });
            }
        }
        let after = self.snapshot(&rewritten);
        // Editors of blocks open on the state before are closed.
        self.material_set = None;
        self.mesh_block = None;
        self.parameter_set = None;
        self.measure_editor = None;
        self.study_editor = None;
        self.rename = None;
        self.confirm_delete.clear();
        self.frame_label = None;
        let mut removed = Vec::new();
        for id in gone {
            if let Some(node) = self.detach(id) {
                removed.push(node);
            }
        }
        let path = self.model_path();
        let mut nodes = Vec::with_capacity(before.blocks.len());
        for kept in before.blocks {
            let current = match self.nodes.iter().position(|n| n.id == kept.id) {
                Some(index) => Some(self.nodes.remove(index)),
                None => (kept_nodes.iter().position(|n| n.id == kept.id))
                    .map(|index| kept_nodes.remove(index)),
            };
            let mut node = match (current, kept.data) {
                // A block that is no analysis takes what it held, unless it holds that still.
                (Some(node), Some(data)) => {
                    let position = kept.position;
                    let mut node = node;
                    let now = node.written(position);
                    if same_data(now.as_ref(), Some(&data)) {
                        node
                    } else {
                        Node::read(data)
                    }
                }
                (Some(node), None) => node,
                (None, Some(data)) => Node::read(data),
                (None, None) => continue,
            };
            node.name = kept.name.clone();
            if let Some(analysis) = node.as_analysis_mut() {
                if let Some(window) = &mut analysis.window {
                    window.rename(&kept.name);
                }
                if let Some(mut data) = kept.analysis {
                    data.id = kept.id;
                    data.name = kept.name.clone();
                    match &mut analysis.window {
                        Some(window) => window.load(ctx, path.clone(), data),
                        None => analysis.data = Some(data),
                    }
                    analysis.seen = None;
                }
            }
            node.refresh_closed();
            let pos = egui::pos2(kept.position[0], kept.position[1]);
            match self.graph_node(kept.id) {
                Some(graph_id) => {
                    if let Some(info) = self.graph.get_node_info_mut(graph_id) {
                        info.pos = pos;
                    }
                }
                None => {
                    self.graph.insert_node(pos, kept.id);
                }
            }
            nodes.push(node);
        }
        // Blocks the change added after the snapshot was taken, and kept ones not back.
        removed.extend(std::mem::take(&mut self.nodes));
        removed.retain(|n| !nodes.iter().any(|kept| kept.id == n.id));
        self.nodes = nodes;
        self.links = before.links;
        self.frames = before.frames;
        self.compact = before.compact;
        self.applied.clear();
        self.sync_due = true;
        self.history.positions = self.positions();
        // Blocks brought back stay in the groups the change kept them in.
        self.known = self.nodes.iter().map(|n| n.id).collect();
        Ok(Change {
            before: after,
            removed,
        })
    }
}
