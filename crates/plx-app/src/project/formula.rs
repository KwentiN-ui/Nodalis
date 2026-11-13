//! The Formula block of the project schematic: a number or a series computed by a formula
//! the user types, of any number of inputs. A wire dropped on its empty pin adds an input,
//! named by the next letter: a, b, c, ... Numbers and series go together like the arrays of
//! NumPy, element by element, and the units go through the formula as in the Math block: a
//! force over a length is a force per length. See [`plx_model::formula`].

use super::*;
use plx_model::formula::{Expr, Function, MAX_INPUTS, input_index, input_letter};

impl NumberBlock {
    /// A Formula block of `text` with the values of its inputs.
    pub(super) fn formula(units: UnitSystem, text: String, values: Vec<f64>) -> Self {
        let mut block = Self {
            units,
            kind: None,
            operands: [None; 2],
            operation: MathOp::default(),
            series: Vec::new(),
            formula: Some(Box::new(FormulaState {
                text,
                kinds: vec![Some(ValueKind::Factor); values.len()],
                problem: None,
            })),
            values,
        };
        block.settle_formula();
        block
    }

    pub(super) fn formula_text(&self) -> &str {
        self.formula.as_ref().map_or("", |f| f.text.as_str())
    }

    /// Why a Formula block gives nothing, see [`FormulaState::problem`].
    pub(super) fn formula_problem(&self) -> Option<&str> {
        self.formula.as_ref()?.problem.as_deref()
    }

    /// Its formula read, or why it cannot be.
    fn parsed(&self) -> Result<Expr, String> {
        Expr::parse(self.formula_text())
    }

    /// The kind of its result follows from the kinds of the inputs the formula uses, once
    /// all are known; it keeps its name for a unit of the same powers, so that a moment
    /// linked to an energy stays an energy. A formula that cannot be read, or whose units
    /// do not go together, has no kind and says why.
    pub(super) fn settle_formula(&mut self) {
        let Some(formula) = &mut self.formula else {
            return;
        };
        formula
            .kinds
            .resize(self.values.len(), Some(ValueKind::Factor));
        formula.problem = None;
        let expr = match Expr::parse(&formula.text) {
            Ok(expr) => expr,
            Err(why) => {
                formula.problem = Some(why);
                self.kind = None;
                return;
            }
        };
        let open = (expr.inputs().into_iter())
            .any(|i| i < formula.kinds.len() && formula.kinds[i].is_none());
        if open {
            self.kind = None;
            return;
        }
        let dimensions: Vec<_> = (formula.kinds.iter())
            .map(|k| k.map(ValueKind::dimension).unwrap_or_default())
            .collect();
        match expr.dimension(&dimensions) {
            Ok(dimension) => {
                if self.kind.is_none_or(|k| k.dimension() != dimension) {
                    self.kind = Some(ValueKind::of_dimension(dimension));
                }
            }
            Err(why) => {
                formula.problem = Some(why);
                self.kind = None;
            }
        }
    }

    /// Takes the kind a link gives the input of `path`, or, for its result, the name of a
    /// kind of the same unit as the formula gives.
    pub(super) fn take_formula_kind(&mut self, path: &[String], kind: ValueKind) {
        let Some(formula) = &mut self.formula else {
            return;
        };
        match path {
            [cell, key] if cell == links::PARAMETERS => {
                if let Some(index) = input_index(key) {
                    if formula.kinds.len() <= index {
                        formula.kinds.resize(index + 1, Some(ValueKind::Factor));
                    }
                    formula.kinds[index] = Some(kind);
                }
                self.settle_formula();
            }
            _ => {
                self.settle_formula();
                if self.kind.is_some_and(|k| k.dimension() == kind.dimension()) {
                    self.kind = Some(kind);
                }
            }
        }
    }

    /// Each input with its values and whether it is a series.
    fn formula_inputs(&self) -> Vec<(Vec<f64>, bool)> {
        (0..self.values.len())
            .map(|i| match self.series.get(i).cloned().flatten() {
                Some(series) => (series, true),
                None => (vec![self.values[i]], false),
            })
            .collect()
    }

    fn formula_evaluated(&self) -> Option<(bool, Result<Vec<f64>, String>)> {
        let expr = self.parsed().ok()?;
        let inputs = self.formula_inputs();
        let flags: Vec<bool> = inputs.iter().map(|(_, series)| *series).collect();
        let series = expr.gives_series(&flags);
        if let Some(problem) = self.formula_problem() {
            return Some((series, Err(problem.to_owned())));
        }
        let refs: Vec<(&[f64], bool)> = (inputs.iter())
            .map(|(values, series)| (values.as_slice(), *series))
            .collect();
        Some((series, expr.evaluate(&refs)))
    }

    /// The one number a Formula block gives; `None` when it gives a series or nothing.
    pub(super) fn formula_number(&self) -> Option<f64> {
        let (false, Ok(values)) = self.formula_evaluated()? else {
            return None;
        };
        values.first().copied().filter(|v| v.is_finite())
    }

    /// The series a Formula block gives, or why it gives none; `None` when it gives one
    /// number.
    pub(super) fn formula_series(&self) -> Option<Result<Vec<f64>, String>> {
        match self.formula_evaluated() {
            Some((true, result)) => Some(result),
            Some((false, _)) => None,
            // A formula that cannot be read says so where a series would be.
            None if self.series.iter().any(Option::is_some) => Some(Err(self
                .formula_problem()
                .unwrap_or("no formula")
                .to_owned())),
            None => None,
        }
    }
}

/// Whether the result of the Formula block `number` of block `id` is a series, with the
/// inputs links give series of by `series`.
pub(super) fn formula_gives_series(
    number: &NumberBlock,
    links: &[Link],
    id: u64,
    series: &dyn Fn(&Link) -> bool,
) -> bool {
    let Ok(expr) = number.parsed() else {
        return false;
    };
    let flags: Vec<bool> = (0..number.values.len())
        .map(|i| {
            let key = input_letter(i).to_string();
            (links.iter())
                .filter(|l| l.target == id && links::is_value(l))
                .filter(|l| matches!(links::slot(l), [c, k] if c == links::PARAMETERS && *k == key))
                .any(series)
        })
        .collect();
    expr.gives_series(&flags)
}

/// Removes the inputs of Formula blocks that no link and no wire through a group reaches
/// any more: the inputs after one that goes move up a letter, in the formula, the links and
/// the wires of groups too, so that the formula keeps its meaning. Whether any went.
pub(super) fn drop_unlinked_inputs(nodes: &mut [Node], links: &mut [Link]) -> bool {
    let formulas: Vec<(u64, usize)> = (nodes.iter())
        .filter_map(|n| match &n.block {
            Block::Formula(number) => Some((n.id, number.values.len())),
            _ => None,
        })
        .collect();
    let input_of = |path: &[String]| match path {
        [c, k] if c == links::PARAMETERS => input_index(k),
        _ => None,
    };
    let mut dropped = false;
    for (id, count) in formulas {
        let mut used = vec![false; count];
        let mut mark = |path: &[String]| {
            if let Some(i) = input_of(path).filter(|i| *i < count) {
                used[i] = true;
            }
        };
        for l in links.iter().filter(|l| l.target == id) {
            mark(links::slot(l));
        }
        for node in nodes.iter() {
            if let Block::Group(group) = &node.block {
                for socket in group.inputs.iter().chain(&group.outputs) {
                    let pins = socket.targets.iter().chain(&socket.source);
                    for pin in pins.filter(|p| p.node == id) {
                        mark(&pin.path);
                    }
                }
            }
        }
        if used.iter().all(|u| *u) {
            continue;
        }
        dropped = true;
        // The new index of every input that stays.
        let mut moved = vec![0; count];
        let mut next = 0;
        for i in 0..count {
            moved[i] = next;
            if used[i] {
                next += 1;
            }
        }
        let rename = |path: &mut Vec<String>| {
            if let Some(i) = input_of(path).filter(|i| *i < count) {
                path[1] = input_letter(moved[i]).to_string();
            }
        };
        for l in links.iter_mut().filter(|l| l.target == id) {
            if l.slot.is_empty() {
                rename(&mut l.item);
            } else {
                rename(&mut l.slot);
            }
        }
        for node in nodes.iter_mut() {
            if let Block::Group(group) = &mut node.block {
                for socket in group.inputs.iter_mut().chain(group.outputs.iter_mut()) {
                    let pins = socket.targets.iter_mut().chain(socket.source.iter_mut());
                    for pin in pins.filter(|p| p.node == id) {
                        rename(&mut pin.path);
                    }
                }
            }
        }
        let Some(Block::Formula(number)) =
            (nodes.iter_mut().find(|n| n.id == id)).map(|n| &mut n.block)
        else {
            continue;
        };
        let keep = |v: &mut Vec<f64>| {
            let mut i = 0;
            v.retain(|_| {
                i += 1;
                used.get(i - 1).copied().unwrap_or(false)
            });
        };
        keep(&mut number.values);
        let mut i = 0;
        number.series.retain(|_| {
            i += 1;
            used.get(i - 1).copied().unwrap_or(false)
        });
        if let Some(formula) = &mut number.formula {
            formula.text = plx_model::formula::rename_inputs(&formula.text, &|i| {
                match used.get(i) {
                    Some(true) => Some(moved[i]),
                    Some(false) => None,
                    // A letter beyond the inputs stays as typed, and is reported.
                    None => Some(i),
                }
            });
        }
        number.settle_formula();
    }
    dropped
}

/// The unlinked inputs of the Formula blocks of `numbers` are factors, the linked ones open
/// until their link gives a kind, see [`numbers::settle_numbers`].
pub(super) fn open_linked_inputs(number: &mut NumberBlock, id: u64, links: &[Link]) {
    let Some(formula) = &mut number.formula else {
        return;
    };
    formula.kinds = (0..number.values.len())
        .map(|i| {
            let key = input_letter(i).to_string();
            let linked = links.iter().any(|l| {
                l.target == id
                    && matches!(links::slot(l), [c, k] if c == links::PARAMETERS && *k == key)
            });
            (!linked).then_some(ValueKind::Factor)
        })
        .collect();
    number.settle_formula();
}

/// The text of the result of a Formula block, for messages: its kind and unit.
fn result_text(number: &NumberBlock) -> String {
    if let Some(problem) = number.formula_problem() {
        return format!("No result: {problem}.");
    }
    let value = match (number.formula_series(), number.formula_number()) {
        (Some(Ok(series)), _) => super::series::shown_series(&series, number.kind, number.units),
        (Some(Err(why)), _) => return format!("No result: {why}."),
        (None, Some(value)) => numbers::shown_number(value, number.kind, number.units),
        (None, None) if number.kind.is_none() => {
            return "No result yet: an input has no kind of number yet.".into();
        }
        (None, None) => return "No result: the value is not a finite number.".into(),
    };
    match number.kind {
        Some(kind) => format!("Result: {value} ({})", describe_kind(kind)),
        None => format!("Result: {value}"),
    }
}

fn describe_kind(kind: ValueKind) -> String {
    match kind {
        ValueKind::Factor => "without unit".into(),
        kind => kind.describe(),
    }
}

/// The editor of the formula of a Formula block, its properties: the formula, what it
/// gives, its inputs and the functions it can call.
pub(super) struct FormulaEditor {
    pub(super) id: u64,
    name: String,
    draft: String,
    /// The block as it is, to try the draft on.
    block: NumberBlock,
    /// Where each input comes from, by index.
    sources: Vec<Option<String>>,
    focus: bool,
}

pub(super) enum FormulaEditorResult {
    Open,
    Cancel,
    Ok(String),
}

impl FormulaEditor {
    fn show(&mut self, ctx: &egui::Context) -> FormulaEditorResult {
        let mut result = FormulaEditorResult::Open;
        let mut open = true;
        let mut tried = self.block.clone();
        if let Some(formula) = &mut tried.formula {
            formula.text = self.draft.clone();
        }
        // Kinds of the inputs stay; only what follows from the formula is found again.
        tried.settle_formula();
        egui::Window::new(format!("Formula: {}", self.name))
            .id(crate::ids::window(ctx, "formula block"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                ui.set_width(540.0);
                ui.label("Formula");
                let edit = ui.add(
                    egui::TextEdit::multiline(&mut self.draft)
                        .font(egui::TextStyle::Monospace)
                        .desired_rows(2)
                        .desired_width(540.0)
                        .hint_text("sqrt(a^2 + b^2)"),
                );
                if self.focus {
                    edit.request_focus();
                    self.focus = false;
                }
                let text = result_text(&tried);
                if text.starts_with("No result") {
                    ui.colored_label(crate::style::error(), text);
                } else {
                    ui.label(text);
                }
                ui.add_space(6.0);
                ui.strong("Inputs");
                if self.block.values.is_empty() {
                    ui.weak(
                        "None yet: drag the pin of a number or a series to the empty pin of \
                         the block, and it becomes a, then b, ...",
                    );
                }
                egui::Grid::new("formula inputs")
                    .num_columns(3)
                    .spacing([12.0, 2.0])
                    .show(ui, |ui| {
                        for (i, (key, _, kind, value)) in
                            self.block.inputs(false).into_iter().enumerate()
                        {
                            ui.monospace(key);
                            let series = self.block.series.get(i).cloned().flatten();
                            ui.label(match (&series, kind) {
                                (Some(s), Some(kind)) => {
                                    format!("Series of {} values, {}", s.len(), describe_kind(kind))
                                }
                                (Some(s), None) => format!("Series of {} values", s.len()),
                                (None, Some(kind)) => format!(
                                    "{}, {}",
                                    numbers::shown_number(value, Some(kind), self.block.units),
                                    describe_kind(kind)
                                ),
                                (None, None) => "No kind of number yet".into(),
                            });
                            match self.sources.get(i).cloned().flatten() {
                                Some(source) => ui.weak(format!("from {source}")),
                                None => ui.weak("not linked"),
                            };
                            ui.end_row();
                        }
                    });
                ui.add_space(6.0);
                ui.strong("Writing formulas");
                ui.weak(
                    "+ - * / and ^ (or **) for powers, parentheses, numbers such as 2.5 or \
                     1e-3, numbers with units such as 10mm, 210GPa, 10N/mm^2 or 5[mm], and pi. Values are \
                     in SI units. Series go element by element, like NumPy arrays; a number \
                     goes with every value of a series. Sums need numbers of one unit; \
                     products and powers combine units.",
                );
                ui.add_space(6.0);
                ui.strong("Functions");
                ui.weak("Click a function to add it to the formula.");
                egui::ScrollArea::vertical()
                    .max_height(220.0)
                    .auto_shrink([false, true])
                    .show(ui, |ui| {
                        egui::Grid::new("formula functions")
                            .num_columns(2)
                            .spacing([12.0, 2.0])
                            .striped(true)
                            .show(ui, |ui| {
                                for function in Function::ALL {
                                    let button = egui::Button::new(
                                        egui::RichText::new(function.signature()).monospace(),
                                    )
                                    .frame(false);
                                    if ui
                                        .add(button)
                                        .on_hover_text("Adds it to the formula")
                                        .clicked()
                                    {
                                        if !self.draft.is_empty() && !self.draft.ends_with(' ') {
                                            self.draft.push(' ');
                                        }
                                        self.draft.push_str(function.name());
                                        self.draft.push('(');
                                        self.focus = true;
                                    }
                                    ui.label(function.description());
                                    ui.end_row();
                                }
                            });
                    });
                ui.add_space(8.0);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    if ui.button("Cancel").clicked() {
                        result = FormulaEditorResult::Cancel;
                    }
                    if ui.button("OK").clicked() {
                        result = FormulaEditorResult::Ok(self.draft.trim().to_owned());
                    }
                });
            });
        if !open || ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            result = FormulaEditorResult::Cancel;
        }
        result
    }
}

impl ProjectApp {
    /// Adds a Formula block without inputs; returns its id.
    pub(super) fn add_formula(&mut self, position: egui::Pos2) -> u64 {
        let id = self.next_id();
        let name = block_name("Formula");
        let block = NumberBlock::formula(self.settings.new_model.units, String::new(), Vec::new());
        self.nodes
            .push(Node::new(id, name.clone(), Block::Formula(block)));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: drag numbers or series to its empty pin, they become a, b, c, \
             ...; then click its formula row to write the formula."
        ));
        id
    }

    /// The formula of a Formula block; its kinds follow from its links again.
    pub(super) fn set_formula(&mut self, id: u64, text: String) {
        let Some(Block::Formula(number)) = self.node(id).map(|n| &mut n.block) else {
            return;
        };
        if let Some(formula) = &mut number.formula {
            formula.text = text;
        }
        number.settle_formula();
        self.refresh_number_kinds();
        // A formula of a series may now give one, or one number of it.
        self.drop_series_conflicts();
        self.sync_due = true;
    }

    /// The input of a Formula block a wire dropped on its empty pin takes: a new one, by
    /// the next letter, which is added. It goes again at the next sync unless the link is
    /// made, see [`drop_unlinked_inputs`].
    pub(super) fn new_formula_input(&mut self, target: u64) -> Option<Vec<String>> {
        let Some(Block::Formula(number)) = self.node(target).map(|n| &mut n.block) else {
            return None;
        };
        let count = number.values.len();
        if count >= MAX_INPUTS {
            return None;
        }
        number.values.push(0.0);
        number.settle_formula();
        self.sync_due = true;
        Some(vec![
            links::PARAMETERS.to_owned(),
            input_letter(count).to_string(),
        ])
    }

    /// The input a wire dropped on the empty pin of a Formula block is checked against:
    /// the one it would make.
    pub(super) fn next_formula_input(&self, target: u64) -> Option<Vec<String>> {
        let node = self.nodes.iter().find(|n| n.id == target)?;
        let Block::Formula(number) = &node.block else {
            return None;
        };
        Some(vec![
            links::PARAMETERS.to_owned(),
            input_letter(number.values.len()).to_string(),
        ])
    }

    pub(super) fn edit_formula(&mut self, id: u64) {
        let Some(node) = self.nodes.iter().find(|n| n.id == id) else {
            return;
        };
        let Block::Formula(number) = &node.block else {
            return;
        };
        let sources = (0..number.values.len())
            .map(|i| {
                let key = input_letter(i).to_string();
                (self.links.iter())
                    .find(|l| {
                        l.target == id
                            && matches!(links::slot(l), [c, k] if c == links::PARAMETERS && *k == key)
                    })
                    .map(|l| {
                        let item = l.item.get(1).map_or("", String::as_str);
                        format!("{} of {}", item, self.name_of(l.source))
                    })
            })
            .collect();
        self.formula_editor = Some(FormulaEditor {
            id,
            name: node.name.clone(),
            draft: number.formula_text().to_owned(),
            block: number.clone(),
            sources,
            focus: true,
        });
    }

    pub(super) fn formula_editor_window(&mut self, ctx: &egui::Context) {
        let Some(editor) = &mut self.formula_editor else {
            return;
        };
        let id = editor.id;
        match editor.show(ctx) {
            FormulaEditorResult::Open => {}
            FormulaEditorResult::Cancel => self.formula_editor = None,
            FormulaEditorResult::Ok(text) => {
                self.formula_editor = None;
                let unchanged = (self.nodes.iter().find(|n| n.id == id))
                    .and_then(Node::as_number)
                    .is_some_and(|(n, _)| n.formula_text() == text);
                if !unchanged {
                    self.checkpoint(&[]);
                    self.set_formula(id, text);
                }
            }
        }
    }
}

impl Viewer {
    /// The row of a Formula block that shows its formula, read-only, in red while it gives
    /// no result; a click opens its editor.
    pub(super) fn edit_formula_row(
        &mut self,
        ui: &mut egui::Ui,
        id: u64,
        field: egui::Rect,
        tip: &str,
    ) {
        let Some((block, _)) = self.block(id).and_then(|b| b.number.clone()) else {
            return;
        };
        let formula = block.formula_text();
        let (text, color) = if formula.is_empty() {
            ("Click to write a formula", ui.visuals().weak_text_color())
        } else if block.formula_problem().is_some() {
            (formula, crate::style::error())
        } else {
            (formula, ui.visuals().text_color())
        };
        let mut job = egui::text::LayoutJob::simple_singleline(
            text.to_owned(),
            egui::TextStyle::Monospace.resolve(ui.style()),
            color,
        );
        job.wrap = egui::text::TextWrapping {
            max_width: field.width() - 8.0,
            max_rows: 1,
            break_anywhere: true,
            overflow_character: Some('…'),
        };
        let galley = ui.painter().layout_job(job);
        let response = ui.allocate_rect(field, egui::Sense::click());
        let frame = if response.hovered() {
            ui.visuals().widgets.hovered.bg_stroke
        } else {
            ui.visuals().widgets.noninteractive.bg_stroke
        };
        ui.painter()
            .rect_stroke(field, 2.0, frame, egui::StrokeKind::Inside);
        ui.painter().galley(
            egui::pos2(field.left() + 4.0, field.center().y - galley.size().y / 2.0),
            galley,
            color,
        );
        let tip = if formula.is_empty() {
            tip.to_owned()
        } else {
            format!("{formula}\n{tip}")
        };
        if response
            .on_hover_cursor(egui::CursorIcon::PointingHand)
            .on_hover_text(tip)
            .clicked()
        {
            self.actions.push(Action::Open(id, None));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn path(parts: &[&str]) -> Vec<String> {
        parts.iter().map(|p| p.to_string()).collect()
    }

    fn link(source: u64, item: &[&str], target: u64, slot: &[&str]) -> Link {
        Link {
            source,
            target,
            item: path(item),
            slot: path(slot),
            ..Default::default()
        }
    }

    fn parameters(id: u64, parameters: Vec<Parameter>) -> Node {
        Node::read(ProjectNode::Parameters(file::ParameterSet {
            id,
            name: format!("Parameters-{id}"),
            parameters,
            ..Default::default()
        }))
    }

    fn formula(id: u64, text: &str, inputs: usize) -> Node {
        Node::read(ProjectNode::Formula(file::FormulaBlock {
            id,
            name: format!("Formula-{id}"),
            formula: text.into(),
            values: vec![0.0; inputs],
            ..Default::default()
        }))
    }

    fn number(nodes: &[Node], id: u64) -> NumberBlock {
        let node = nodes.iter().find(|n| n.id == id).unwrap();
        node.as_number().unwrap().0.clone()
    }

    #[test]
    fn a_formula_combines_the_units_of_its_inputs() {
        let force = Parameter::new("F", ValueKind::Quantity(Quantity::Force), 100.0);
        let length = Parameter::new("L", ValueKind::LENGTH, 0.5);
        let mut nodes = vec![
            parameters(1, vec![force, length]),
            formula(2, "a / b + 3[N/mm]", 2),
        ];
        let mut links = vec![
            link(1, &["Value", "F"], 2, &["Parameters", "a"]),
            link(1, &["Value", "L"], 2, &["Parameters", "b"]),
        ];
        numbers::refresh_kinds(&mut nodes, &mut links);
        let (block, _) = nodes[1].as_number_mut().unwrap();
        block.set(false, "a", 100.0);
        block.set(false, "b", 0.5);
        let block = number(&nodes, 2);
        assert_eq!(
            block.kind,
            Some(ValueKind::Quantity(Quantity::ForcePerLength))
        );
        assert_eq!(block.formula_number(), Some(200.0 + 3000.0));
        // Units that do not go together give no result, and say why.
        nodes[1] = formula(2, "a + b", 2);
        numbers::refresh_kinds(&mut nodes, &mut links);
        let block = number(&nodes, 2);
        assert_eq!(block.kind, None);
        assert_eq!(
            block.formula_problem(),
            Some("a + b adds a force and a length")
        );
        assert_eq!(block.formula_number(), None);
        // A square root halves the powers; an unlinked input is a factor.
        nodes[1] = formula(2, "sqrt(b * b * 4)", 2);
        numbers::refresh_kinds(&mut nodes, &mut links);
        assert_eq!(number(&nodes, 2).kind, Some(ValueKind::LENGTH));
    }

    #[test]
    fn a_formula_result_linked_to_another_unit_says_so_and_keeps_the_link() {
        let force = Parameter::new("F", ValueKind::Quantity(Quantity::Force), 1.0);
        let length = Parameter::new("L", ValueKind::LENGTH, 1.0);
        let mut nodes = vec![
            parameters(1, vec![force]),
            formula(2, "a", 1),
            parameters(3, vec![length]),
        ];
        let mut links = vec![
            link(1, &["Value", "F"], 2, &["Parameters", "a"]),
            link(2, &["Value", "Result"], 3, &["Value", "L"]),
        ];
        numbers::refresh_kinds(&mut nodes, &mut links);
        assert_eq!(
            links.len(),
            2,
            "the link stays while the formula is being written"
        );
        let block = number(&nodes, 2);
        assert!(
            block
                .formula_problem()
                .is_some_and(|p| p.contains("length"))
        );
        assert_eq!(block.result(false), None);
        // A new link of the wrong kind is refused.
        let why = numbers::number_refusal(
            &nodes,
            &links[..1],
            2,
            &path(&["Value", "Result"]),
            3,
            &path(&["Value", "L"]),
        );
        assert!(matches!(why, Some(Some(_))), "{why:?}");
    }

    #[test]
    fn formulas_compute_with_series_and_reduce_them() {
        let mut nodes = vec![formula(1, "max(a) * b", 2)];
        // Whether it gives a series follows from the links: max of one series is a number.
        let links = vec![link(5, &["Value", "Result"], 1, &["Parameters", "a"])];
        let series = |_: &Link| true;
        assert!(!formula_gives_series(
            &number(&nodes, 1),
            &links,
            1,
            &series
        ));
        let (block, _) = nodes[0].as_number_mut().unwrap();
        block.series = vec![Some(vec![1.0, 5.0, 2.0]), None];
        block.values = vec![0.0, 2.0];
        assert_eq!(block.formula_number(), Some(10.0));
        assert_eq!(block.formula_series(), None);
        if let Some(f) = &mut block.formula {
            f.text = "a * b".into();
        }
        block.settle_formula();
        assert_eq!(block.formula_series(), Some(Ok(vec![2.0, 10.0, 4.0])));
        assert_eq!(block.result(false), None);
        assert!(formula_gives_series(&number(&nodes, 1), &links, 1, &series));
    }

    #[test]
    fn an_input_whose_link_goes_goes_too_and_later_ones_move_up() {
        let length = Parameter::new("L", ValueKind::LENGTH, 2.0);
        let mut nodes = vec![
            parameters(1, vec![length]),
            formula(2, "a + max(b, c) + 2[mm]", 3),
        ];
        // b lost its link: c becomes b, in the formula and in its link.
        let mut links = vec![
            link(1, &["Value", "L"], 2, &["Parameters", "a"]),
            link(1, &["Value", "L"], 2, &["Parameters", "c"]),
        ];
        numbers::refresh_kinds(&mut nodes, &mut links);
        let block = number(&nodes, 2);
        assert_eq!(block.formula_text(), "a + max(?, b) + 2[mm]");
        assert!(
            block
                .formula_problem()
                .is_some_and(|p| p.contains("link was removed"))
        );
        assert_eq!(block.values.len(), 2);
        assert_eq!(links[1].slot, path(&["Parameters", "b"]));
        assert_eq!(block.kind, None);
    }

    #[test]
    fn a_wire_on_the_empty_pin_adds_the_next_letter() {
        let node = formula(1, "", 0);
        let rows = numbers::number_rows(&node, &[]);
        assert!(rows.iter().any(|r| r.path == path(&["Parameters", "+"])));
        let mut node = formula(1, "a + b", 2);
        let rows = numbers::number_rows(&node, &[]);
        let inputs: Vec<&str> = (rows.iter())
            .filter(|r| r.input && r.path.len() == 2 && r.path[1] != "+")
            .map(|r| r.text.as_str())
            .collect();
        assert_eq!(inputs, ["a", "b"]);
        // Written and read again.
        let written = node.written([0.0; 2]).unwrap();
        match &written {
            ProjectNode::Formula(f) => {
                assert_eq!(f.formula, "a + b");
                assert_eq!(f.values.len(), 2);
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(Node::read(written).kind(), Kind::Formula);
    }
}
