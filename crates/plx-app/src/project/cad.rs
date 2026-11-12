//! The CAD Operation block of the project schematic: a fillet or chamfer of picked edges,
//! planar faces moved, or parts and faces split, of the geometry linked to it, which gives the changed geometry on to analyses or further
//! operations, Blender-modifier-like. Its length is a row with a pin, so that a parameter
//! study varies a radius like any other number; every change makes the geometry again in
//! Update Project, and the analyses after it mesh it anew.

use plx_model::{CadOperation, Geometry};

use super::*;

/// The key of the length of a CAD operation among its parameters, `["Parameters", key]`.
pub(super) const CAD_LENGTH: &str = "length";

/// A CAD operation block: the operation, the geometry it takes and the one it made.
pub(super) struct CadBlock {
    /// Its geometry is the result as last made, given on through links.
    pub(super) model: LinkedModel,
    /// The geometry the link of its Geometry row brings; set by [`CadBlock::set_input`].
    input: Option<Geometry>,
    pub(super) operation: CadOperation,
    pub(super) units: UnitSystem,
    /// Fingerprint of the input and operation the result was made from, see
    /// [`CadOperation::fingerprint`].
    pub(super) made_from: Option<u64>,
    /// Why the operation failed, for that fingerprint.
    pub(super) failed: Option<(u64, String)>,
    /// [`CadBlock::wanted`] as last found, by a quick key of the operation: its fingerprint
    /// hashes the whole input, too slow for every frame. Forgotten with a new input.
    wanted_memo: std::cell::Cell<Option<(u64, Option<u64>)>>,
}

impl CadBlock {
    pub(super) fn new(
        operation: CadOperation,
        units: UnitSystem,
        result: Option<Geometry>,
        made_from: Option<u64>,
    ) -> Self {
        Self {
            model: LinkedModel {
                geometry: result,
                ..Default::default()
            },
            input: None,
            operation,
            units,
            made_from,
            failed: None,
            wanted_memo: std::cell::Cell::new(None),
        }
    }

    /// The geometry the link of its Geometry row brings.
    pub(super) fn input(&self) -> Option<&Geometry> {
        self.input.as_ref()
    }

    /// Takes a new input geometry.
    pub(super) fn set_input(&mut self, input: Option<Geometry>) {
        self.input = input;
        self.wanted_memo.set(None);
    }

    /// Fingerprint of what the result is to be made from now; `None` without input or
    /// before anything is picked.
    fn wanted(&self) -> Option<u64> {
        let mut key = plx_mesh::Fnv::new();
        key.add_debug(&self.operation);
        let key = key.finish();
        if let Some((seen, wanted)) = self.wanted_memo.get()
            && seen == key
        {
            return wanted;
        }
        let wanted = self.find_wanted();
        self.wanted_memo.set(Some((key, wanted)));
        wanted
    }

    fn find_wanted(&self) -> Option<u64> {
        if self.operation.needs_picks() && self.operation.entities.is_empty() {
            return None;
        }
        Some(self.operation.fingerprint(self.input.as_ref()?))
    }

    /// The state of its geometry: missing without input or picks, failed while the operation fails
    /// as it is, out of date once input or operation changed.
    pub(super) fn state(&self) -> CellState {
        let Some(wanted) = self.wanted() else {
            return CellState::Missing;
        };
        if self
            .failed
            .as_ref()
            .is_some_and(|(print, _)| *print == wanted)
        {
            CellState::Failed
        } else if self.made_from == Some(wanted) && self.model.geometry.is_some() {
            CellState::UpToDate
        } else {
            CellState::OutOfDate
        }
    }

    /// Why it failed as it is now.
    pub(super) fn error(&self) -> Option<&str> {
        let wanted = self.wanted()?;
        (self.failed.as_ref())
            .filter(|(print, _)| *print == wanted)
            .map(|(_, why)| why.as_str())
    }

    /// The operation applied to the input, without keeping it: for Generate and Preview.
    pub(super) fn apply(input: &Geometry, operation: &CadOperation) -> Result<Geometry, String> {
        if let Some(problem) = operation.problem() {
            return Err(problem);
        }
        let entities = operation.resolve(input)?;
        plx_mesher::apply_operation(input, operation, &entities).map_err(|e| e.to_string())
    }

    /// Makes its geometry anew; returns a line on what it made, or why it failed, which
    /// the block shows until something changes.
    pub(super) fn generate(&mut self) -> Result<String, String> {
        let input = self.input.as_ref().ok_or("no geometry is linked to it")?;
        let wanted = self.operation.fingerprint(input);
        match Self::apply(input, &self.operation) {
            Ok(geometry) => {
                self.take_result(geometry, wanted);
                Ok(format!(
                    "{} of {} made",
                    self.operation.kind.label().to_lowercase(),
                    self.operation.describe_picks()
                ))
            }
            Err(why) => {
                self.failed = Some((wanted, why.clone()));
                Err(why)
            }
        }
    }

    /// Keeps a result made for the input and operation of `print`.
    pub(super) fn take_result(&mut self, geometry: Geometry, print: u64) {
        self.model.geometry = Some(geometry);
        self.made_from = Some(print);
        self.failed = None;
    }

    /// What links read of it: its result, unless the operation failed as it is or has
    /// nothing to act on, so that what comes after it is skipped rather than given a
    /// geometry that is not meant.
    pub(super) fn link_data(&self) -> Option<links::Data<'_>> {
        matches!(self.state(), CellState::UpToDate | CellState::OutOfDate)
            .then(|| self.model.data())
    }

    /// Fingerprint of what the result depends on apart from the length, for the parameter
    /// study: the input and the picks.
    pub(super) fn body_print(&self, length_driven: bool, print: &mut plx_mesh::Fnv) {
        let mut operation = self.operation.clone();
        if length_driven {
            operation.value = 0.0;
        }
        print.add(format!("{operation:?}").as_bytes());
    }
}

/// Brings the geometry the CAD operation block `id` takes up to date with the block linked
/// to its Geometry row; the picks follow the input when it changes. Without a result yet,
/// it is made at once, so that it can be linked on.
pub(super) fn take_cad_input(nodes: &mut [Node], links: &[Link], id: u64) {
    let link = links
        .iter()
        .find(|l| l.target == id && l.item.first().is_some_and(|c| c == links::GEOMETRY));
    let input = match link {
        // Some parts of a file: the geometry with only them, once it is made.
        Some(link) if links::is_geometry_part(link) => {
            let parts = links::linked_parts(links, link.source, id);
            let file = (nodes.iter_mut().find(|n| n.id == link.source)).and_then(Node::as_file_mut);
            match file.map(|f| f.subset(&parts)) {
                Some(Ok(None)) => return,
                Some(Ok(Some(geometry))) => Some(geometry),
                _ => None,
            }
        }
        _ => link
            .and_then(|l| nodes.iter().find(|n| n.id == l.source))
            .and_then(Node::link_data)
            .and_then(|d| d.geometry.cloned()),
    };
    // An analysis that is loading in its window gives its geometry once it has loaded.
    if link.is_some() && input.is_none() {
        let loading = (link.and_then(|l| nodes.iter().find(|n| n.id == l.source)))
            .is_some_and(|n| n.link_data().is_none() && n.kind() == Kind::Analysis);
        if loading {
            return;
        }
    }
    let Some(block) = nodes
        .iter_mut()
        .find(|n| n.id == id)
        .and_then(Node::as_cad_mut)
    else {
        return;
    };
    let same = |a: &Option<Geometry>, b: &Option<Geometry>| match (a, b) {
        (Some(a), Some(b)) => a.brep == b.brep && a.part_names == b.part_names,
        (None, None) => true,
        _ => false,
    };
    if same(&block.input, &input) {
        return;
    }
    if let Some(input) = &input {
        block.operation.follow(input);
    }
    block.set_input(input);
    if block.model.geometry.is_none() && block.state() == CellState::OutOfDate {
        let _ = block.generate();
    }
}

/// The rows of a CAD operation block: the geometry it takes and gives with its state, what
/// it does, and its length, which a value link may give.
pub(super) fn cad_rows(node: &Node, incoming: &[(Vec<String>, String)]) -> Vec<Row> {
    let Some(block) = node.as_cad() else {
        return Vec::new();
    };
    let linked = |path: &[String]| {
        (incoming.iter())
            .find(|(slot, _)| slot.as_slice() == path)
            .map(|(_, source)| source.clone())
    };
    let state = block.state();
    let mut tip = "Geometry: drag the geometry of a file block, an analysis or another CAD \
                   operation here; drag the pin on the right to the Geometry row of an \
                   analysis for the changed geometry."
        .to_owned();
    match state {
        CellState::Missing if block.input.is_some() => {
            tip.push_str("\nNothing picked yet: double-click the block to pick what it acts on.")
        }
        CellState::Missing => {}
        CellState::OutOfDate => tip.push_str(
            "\nOut of date: Update Project makes it again, or Update Geometry in the context menu.",
        ),
        CellState::Failed => {
            if let Some(why) = block.error() {
                tip.push_str(&format!(
                    "\nThe operation failed: {why}.\nWhat comes after it is skipped."
                ));
            }
        }
        _ => tip.push_str(&format!("\n{}", state.text())),
    }
    let operation = &block.operation;
    let path = vec![links::GEOMETRY.to_owned()];
    let length = vec![links::PARAMETERS.to_owned(), CAD_LENGTH.to_owned()];
    vec![
        Row {
            text: links::GEOMETRY.to_owned(),
            depth: 0,
            icon: NodeIcon::Geometry,
            state: block.input.is_some().then_some(state),
            expander: None,
            input: true,
            output: block.model.geometry.is_some(),
            linked_from: linked(&path),
            collection: false,
            partial: false,
            widget: None,
            tip,
            path,
        },
        Row {
            text: format!("{}: {}", operation.kind.label(), operation.describe_picks()),
            depth: 0,
            icon: NodeIcon::CadOperation,
            state: None,
            expander: None,
            input: false,
            output: false,
            linked_from: None,
            collection: false,
            partial: false,
            widget: None,
            tip: if operation.needs_picks() {
                format!(
                    "What the operation does and to what: double-click the block to pick the \
                     {} on the geometry and to see the result.",
                    operation.kind.picked_noun()
                )
            } else if operation.kind == plx_model::CadOperationKind::Defeature {
                "What the operation does: it removes the fillets and chamfers up to the max \
                 size. Double-click the block to see them and the result."
                    .to_owned()
            } else {
                "What the operation does: double-click the block to choose the plane and to \
                 see the result."
                    .to_owned()
            },
            path: vec![CAD_OPERATION_ROW.to_owned()],
        },
        Row {
            text: operation.kind.value_label().to_owned(),
            depth: 0,
            icon: NodeIcon::CadOperation,
            state: None,
            expander: None,
            input: true,
            output: false,
            linked_from: linked(&length),
            collection: false,
            partial: false,
            widget: Some(RowWidget::CadLength),
            tip: format!(
                "{}: drop an input of a parameter study here to vary it in its design \
                 points; each makes the geometry again and meshes it anew.",
                operation.kind.value_label()
            ),
            path: length,
        },
    ]
}

/// The path of the row that says what a CAD operation does.
pub(super) const CAD_OPERATION_ROW: &str = "Operation";
