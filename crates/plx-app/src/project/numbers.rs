//! The Math and Switch blocks of the project schematic, like Blender's Math node: numbers
//! computed from the numbers value links give them, for parameters derived from others
//! (Lz = 2 Lx) and variants picked by an index, also as inputs and outputs of a parameter
//! study.

use super::*;

/// The row of a Math block that chooses its operation, and the row of a Switch block that
/// adds and removes choices.
const OPERATION: &str = "Operation";
const CHOICES: &str = "Choices";
/// The row of a Formula block with its formula.
const FORMULA: &str = "Formula";

/// The most choices a Switch block holds.
const MAX_CHOICES: usize = 20;

/// A number as a block shows it: in the unit system, with its unit for a quantity.
pub(super) fn shown_number(value: f64, kind: Option<ValueKind>, units: UnitSystem) -> String {
    match kind.and_then(ValueKind::quantity) {
        Some(quantity) => format!(
            "{} {}",
            crate::results::format_value(units.shown(value, quantity) as f32),
            units.unit(quantity)
        )
        .trim()
        .to_owned(),
        None => match kind {
            Some(ValueKind::Derived(dimension)) => format!(
                "{} {}",
                crate::results::format_value(value as f32),
                dimension.si()
            ),
            _ => crate::results::format_value(value as f32),
        },
    }
}

/// The rows of a Math or Switch block: what it computes, its inputs and its result.
pub(super) fn number_rows(node: &Node, incoming: &[(Vec<String>, String)]) -> Vec<Row> {
    let Some((number, switch)) = node.as_number() else {
        return Vec::new();
    };
    let row = |path: Vec<String>, text: String, tip: String| Row {
        path,
        text,
        depth: 0,
        icon: NodeIcon::Setup,
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
    let mut rows = Vec::new();
    let formula = number.formula.is_some();
    if formula {
        let mut tip = "The formula of the inputs a, b, c, ...: + - * / ^, numbers with units in \
                       brackets such as 5[mm], and functions such as sqrt, max, min and mean. \
                       Double-click the block for the list of functions."
            .to_owned();
        if let Some(problem) = number.formula_problem() {
            tip.push_str(&format!("\nNo result: {problem}."));
        }
        rows.push(Row {
            widget: Some(RowWidget::Formula),
            ..row(vec![FORMULA.to_owned()], "Formula".into(), tip)
        });
    } else if !switch {
        rows.push(Row {
            widget: Some(RowWidget::Operation),
            ..row(
                vec![OPERATION.to_owned()],
                "Operation".into(),
                format!(
                    "What the block computes: {}.",
                    number.operation.formula().to_lowercase()
                ),
            )
        });
    }
    for (index, (key, label, kind, _)) in number.inputs(switch).into_iter().enumerate() {
        let path = vec![links::PARAMETERS.to_owned(), key.clone()];
        let linked_from = (incoming.iter())
            .find(|(slot, _)| *slot == path)
            .map(|(_, source)| source.clone());
        let what = if key == INDEX && !formula {
            "Which choice the block gives, counted from 1".to_owned()
        } else if formula {
            match kind {
                Some(ValueKind::Factor) => {
                    format!("Input {label} of the formula, a number without unit")
                }
                Some(kind) => format!("Input {label} of the formula, a {}", kind.describe()),
                None => format!(
                    "Input {label} of the formula: it takes the kind of the number linked to it"
                ),
            }
        } else {
            match kind {
                Some(ValueKind::Factor) => format!("{label}, a factor without unit"),
                Some(kind) => format!("{label}, a {}", kind.describe()),
                None if number.operation.combines() && !switch => format!(
                    "{label}, a number of any kind: the kind of the result follows from those \
                     of A and B, so a force over a length is a force per length. Unlinked and \
                     with no kind from the others, it is a factor without unit"
                ),
                None => format!(
                    "{label}, a number of any kind: the block takes the kind of the numbers \
                     linked to it. Without one, it is a factor without unit"
                ),
            }
        };
        rows.push(Row {
            input: true,
            linked_from,
            widget: Some(RowWidget::Input(index)),
            ..row(
                path,
                label,
                format!(
                    "{what}. Type it, or drag the pin of a number here: a parameter, another \
                     Math, Switch or Formula block or an input of a parameter study."
                ),
            )
        });
    }
    if formula && number.values.len() < plx_model::formula::MAX_INPUTS {
        rows.push(Row {
            input: true,
            ..row(
                vec![links::PARAMETERS.to_owned(), super::groups::OPEN.to_owned()],
                String::new(),
                "Drag the pin of a number or a series here to add it as the next input of the \
                 formula: a, b, c, ..."
                    .into(),
            )
        });
    }
    if formula {
        rows.push(Row {
            widget: Some(RowWidget::Choices),
            ..row(
                vec![CHOICES.to_owned()],
                match number.values.len() {
                    1 => "1 input".to_owned(),
                    n => format!("{n} inputs"),
                },
                "Adds an input, or removes the last one with its link.".into(),
            )
        });
    }
    if switch && number.values.len().saturating_sub(1) < MAX_CHOICES {
        // An empty pin below the choices: a wire dropped on it adds a choice, as in Blender.
        rows.push(Row {
            input: true,
            ..row(
                vec![links::PARAMETERS.to_owned(), super::groups::OPEN.to_owned()],
                String::new(),
                "Drag the pin of a number here to add it as a new choice.".into(),
            )
        });
    }
    if switch {
        rows.push(Row {
            widget: Some(RowWidget::Choices),
            ..row(
                vec![CHOICES.to_owned()],
                format!("{} choices", number.values.len().saturating_sub(1)),
                "Adds a choice, or removes the last one with its link.".into(),
            )
        });
    }
    let mut tip = "The number the block gives: drag its pin to a parameter of the same kind, to \
                   another Math, Switch or Formula block, or to the Outputs of a parameter study."
        .to_owned();
    let text = match (number.series_result(switch), number.result(switch)) {
        (Some(Ok(series)), _) => {
            tip = "The series the block gives, computed value by value: drag its pin to \
                   another Math block or to a Plot block."
                .into();
            format!(
                "Result: {}",
                super::series::shown_series(&series, number.kind, number.units)
            )
        }
        (Some(Err(why)), _) => {
            tip.push_str(&format!("\nNo result: {why}."));
            "Result: none".into()
        }
        (None, None) if number.formula_problem().is_some() => {
            let problem = number.formula_problem().unwrap_or_default();
            tip.push_str(&format!("\nNo result: {problem}."));
            "Result: none".into()
        }
        (None, Some(value)) => {
            format!("Result: {}", shown_number(value, number.kind, number.units))
        }
        (None, None) if switch => "Result: no choice".into(),
        (None, None) => "Result: none".into(),
    };
    rows.push(Row {
        output: true,
        icon: NodeIcon::Results,
        ..row(vec![links::VALUE.to_owned(), RESULT.to_owned()], text, tip)
    });
    rows
}

impl ProjectApp {
    /// Adds a Math block, which adds two numbers until another operation is chosen.
    pub(super) fn add_math(&mut self, position: egui::Pos2) -> u64 {
        self.add_number(position, false)
    }

    /// Adds a Switch block with two choices, the first one picked.
    pub(super) fn add_switch(&mut self, position: egui::Pos2) -> u64 {
        self.add_number(position, true)
    }

    fn add_number(&mut self, position: egui::Pos2, switch: bool) -> u64 {
        let id = self.next_id();
        let number = NumberBlock {
            units: self.settings.new_model.units,
            kind: None,
            operands: [None; 2],
            operation: MathOp::default(),
            values: if switch {
                vec![1.0, 0.0, 0.0]
            } else {
                vec![0.0, 0.0]
            },
            series: Vec::new(),
            formula: None,
        };
        let (name, block) = if switch {
            (super::block_name("Switch"), Block::Switch(number))
        } else {
            (super::block_name("Math"), Block::Math(number))
        };
        self.nodes.push(Node::new(id, name.clone(), block));
        self.graph.insert_node(position, id);
        self.output.push(format!(
            "{name} created: drag numbers to its inputs and its Result to the number it gives."
        ));
        id
    }

    /// The operation of a Math block, with as many inputs as it takes. Its kinds follow
    /// from its links again: a sum takes numbers of one kind, a product has kinds of A and B
    /// of their own.
    pub(super) fn set_operation(&mut self, id: u64, operation: MathOp) {
        let Some((number, false)) = self.node(id).and_then(Node::as_number_mut) else {
            return;
        };
        number.operation = operation;
        number.values.resize(operation.inputs().len(), 0.0);
        // A link to an input the operation no longer has goes.
        let keys: Vec<String> = (number.inputs(false).into_iter())
            .map(|(key, ..)| key)
            .collect();
        self.links.retain(|l| {
            l.target != id
                || !matches!(links::slot(l), [c, k] if c == links::PARAMETERS && !keys.contains(k))
        });
        self.refresh_number_kinds();
        // Max, Min and Mean give one number; the others a series of one at an input.
        self.drop_series_conflicts();
        self.sync_due = true;
    }

    /// The number of choices of a Switch block; links to choices it no longer has go.
    pub(super) fn set_choices(&mut self, id: u64, count: usize) {
        let Some((number, true)) = self.node(id).and_then(Node::as_number_mut) else {
            return;
        };
        let count = count.clamp(1, MAX_CHOICES);
        number.values.resize(count + 1, 0.0);
        self.links.retain(|l| {
            l.target != id
                || !matches!(links::slot(l), [c, k]
                    if c == links::PARAMETERS && k.parse::<usize>().is_ok_and(|i| i > count))
        });
        self.sync_due = true;
    }

    /// The kinds of the numbers of every Math and Switch block, found from their links,
    /// see [`refresh_kinds`].
    pub(super) fn refresh_number_kinds(&mut self) {
        let (changed, removed) = refresh_kinds(&mut self.nodes, &mut self.links);
        if removed {
            self.output.push(
                "Links between numbers of different kinds removed: the operation changed what \
                 the block takes."
                    .into(),
            );
        }
        if changed {
            self.sync_due = true;
        }
    }
}

/// Whether a number of kind `given` may go to one of kind `taken` at a Math or Switch block:
/// one of the same unit, so a moment to an energy and a quantity in SI base units to the
/// quantity it is.
fn convertible(given: ValueKind, taken: ValueKind) -> bool {
    given.dimension() == taken.dimension()
}

/// A Math or Switch block apart from its block: its id, its numbers and whether it is a
/// switch.
type Number = (u64, NumberBlock, bool);

/// Whether the link goes from the result of a Formula block.
fn from_formula(numbers: &[Number], link: &Link) -> bool {
    link.item == [links::VALUE, RESULT]
        && (numbers.iter()).any(|(id, n, _)| *id == link.source && n.formula.is_some())
}

/// The value links at Math and Switch blocks.
fn number_links(numbers: &[Number], links: &[Link]) -> Vec<Link> {
    let is_number = |id: u64| numbers.iter().any(|n| n.0 == id);
    (links.iter())
        .filter(|l| l.item.first().is_some_and(|c| c == links::VALUE))
        .filter(|l| is_number(l.source) || is_number(l.target))
        .cloned()
        .collect()
}

/// The kind of the number at `path` of block `id`: of a Math or Switch block from `numbers`,
/// else from `outside`.
fn kind_at(
    numbers: &[Number],
    outside: &dyn Fn(u64, &[String]) -> Option<ValueKind>,
    id: u64,
    path: &[String],
) -> Option<ValueKind> {
    match numbers.iter().find(|n| n.0 == id) {
        Some((_, number, switch)) => number.kind_at(*switch, path).flatten(),
        None => outside(id, path),
    }
}

/// Finds the kinds of the Math and Switch blocks `numbers` from the value links at them
/// alone, other blocks giving theirs by `outside`: a block takes the kind of what is linked
/// to it, and a product or quotient the kinds that follow from those of the others, such as
/// a force over a length a force per length. An input of a product or quotient that no link
/// gives, and no other number gives a kind, is a factor: a length over 2 is a length.
fn settle_numbers(
    numbers: &mut [Number],
    links: &[Link],
    outside: &dyn Fn(u64, &[String]) -> Option<ValueKind>,
) {
    for (id, number, _) in numbers.iter_mut() {
        number.kind = None;
        number.operands = [None; 2];
        super::formula::open_linked_inputs(number, *id, links);
    }
    let ids: Vec<u64> = numbers.iter().map(|n| n.0).collect();
    let is_number = |id: u64| ids.contains(&id);
    let kinds = |numbers: &[Number]| -> Vec<(Option<ValueKind>, [Option<ValueKind>; 2])> {
        (numbers.iter())
            .map(|(_, n, _)| (n.kind, n.operands))
            .collect()
    };
    // A chain of blocks passes its kinds on by one link a round at worst; each input made a
    // factor starts the rounds again.
    for _ in 0..=3 * (links.len() + numbers.len()) {
        let round = kinds(numbers);
        for l in links {
            let (item, slot) = (l.item.as_slice(), links::slot(l));
            let given = kind_at(numbers, outside, l.source, item);
            let taken = kind_at(numbers, outside, l.target, slot);
            for (id, path, kind) in [(l.target, slot, given), (l.source, item, taken)] {
                let Some(kind) = kind else {
                    continue;
                };
                let Some((_, number, switch)) = numbers.iter_mut().find(|n| n.0 == id) else {
                    continue;
                };
                // A number found from others takes the name of a block outside with the same
                // unit: a moment linked to an energy is an energy.
                let other = if id == l.target { l.source } else { l.target };
                let renamed = !*switch
                    && !is_number(other)
                    && number
                        .kind_at(*switch, path)
                        .flatten()
                        .is_some_and(|own| own != kind && own.dimension() == kind.dimension());
                if (number.kind_at(*switch, path) == Some(None) || renamed)
                    && let Ok(adopted) = number.with_kind(*switch, path, kind)
                {
                    *number = adopted;
                }
            }
        }
        if kinds(numbers) != round {
            continue;
        }
        let linked = |id: u64, key: &str| {
            links.iter().any(|l| {
                l.target == id
                    && matches!(links::slot(l), [c, k] if c == links::PARAMETERS && k == key)
            })
        };
        let factor = numbers
            .iter()
            .enumerate()
            .find_map(|(index, (id, number, switch))| {
                if *switch || !number.operation.combines() {
                    return None;
                }
                (0..2).find_map(|i| {
                    if number.operands[i].is_some() || linked(*id, ["a", "b"][i]) {
                        return None;
                    }
                    let mut factor = number.clone();
                    factor.operands[i] = Some(ValueKind::Factor);
                    factor.settle().ok().map(|()| (index, factor))
                })
            });
        let Some((index, factor)) = factor else {
            break;
        };
        numbers[index].1 = factor;
    }
}

/// The kinds of the numbers of every Math and Switch block, found from their value links,
/// see [`settle_numbers`]. A link whose ends no longer fit together, after an operation
/// changed, goes. Whether any kind changed, and whether a link went.
pub(super) fn refresh_kinds(nodes: &mut [Node], links: &mut Vec<Link>) -> (bool, bool) {
    let mut numbers: Vec<Number> = (nodes.iter())
        .filter_map(|n| n.as_number().map(|(b, switch)| (n.id, b.clone(), switch)))
        .collect();
    if numbers.is_empty() {
        return (false, false);
    }
    let mut removed = false;
    loop {
        let at_numbers = number_links(&numbers, links);
        settle_numbers(&mut numbers, &at_numbers, &|id, path| {
            value_kind(nodes, id, path)
        });
        let fits = |l: &Link| {
            if !at_numbers.contains(l) || from_formula(&numbers, l) {
                return true;
            }
            let outside = |id, path: &[String]| value_kind(nodes, id, path);
            let given = kind_at(&numbers, &outside, l.source, &l.item);
            let taken = kind_at(&numbers, &outside, l.target, links::slot(l));
            given
                .zip(taken)
                .is_none_or(|(given, taken)| convertible(given, taken))
        };
        let count = links.len();
        links.retain(fits);
        if links.len() == count {
            break;
        }
        removed = true;
    }
    // The result of a formula linked to a number of another unit keeps its link, as the
    // formula is being written, and gives nothing until the units fit.
    let outside = |id, path: &[String]| value_kind(nodes, id, path);
    let mut conflicts: Vec<(u64, String)> = Vec::new();
    for l in number_links(&numbers, links) {
        if !from_formula(&numbers, &l) {
            continue;
        }
        let given = kind_at(&numbers, &outside, l.source, &l.item);
        let taken = kind_at(&numbers, &outside, l.target, links::slot(&l));
        if let (Some(given), Some(taken)) = (given, taken)
            && !convertible(given, taken)
            && !conflicts.iter().any(|(id, _)| *id == l.source)
        {
            let name = (nodes.iter().find(|n| n.id == l.target)).map_or("", |n| n.name.as_str());
            conflicts.push((
                l.source,
                format!(
                    "the formula gives a {}, and its Result goes to {} of {name}, a {}",
                    given.describe(),
                    parameter_label(nodes, l.target, links::slot(&l)),
                    taken.describe()
                ),
            ));
        }
    }
    for (id, why) in conflicts {
        if let Some((_, number, _)) = numbers.iter_mut().find(|n| n.0 == id)
            && let Some(formula) = &mut number.formula
            && formula.problem.is_none()
        {
            formula.problem = Some(why);
        }
    }
    let mut changed = false;
    for (id, new, _) in numbers {
        if let Some((old, _)) =
            (nodes.iter_mut().find(|n| n.id == id)).and_then(Node::as_number_mut)
            && (old.kind, old.operands, &old.formula) != (new.kind, new.operands, &new.formula)
        {
            old.kind = new.kind;
            old.operands = new.operands;
            old.formula = new.formula;
            changed = true;
        }
    }
    (changed, removed)
}

/// Why a new value link from `item` of `source` to `slot` of `target` cannot be, when a Math
/// or Switch block is at an end: the kinds of its ends would differ once the kinds of the
/// numbers are found with it. `None` when it can, or when no such block is at an end.
pub(super) fn number_refusal(
    nodes: &[Node],
    existing: &[Link],
    source: u64,
    item: &[String],
    target: u64,
    slot: &[String],
) -> Option<Option<String>> {
    let mut numbers: Vec<Number> = (nodes.iter())
        .filter_map(|n| n.as_number().map(|(b, switch)| (n.id, b.clone(), switch)))
        .collect();
    let is_number = |id: u64| numbers.iter().any(|n| n.0 == id);
    if !is_number(source) && !is_number(target) {
        return None;
    }
    let link = Link {
        source,
        target,
        item: item.to_vec(),
        slot: slot.to_vec(),
        ..Default::default()
    };
    // The new link takes the place of one that fills the same input.
    let mut links: Vec<Link> = (existing.iter())
        .filter(|l| !(l.target == target && links::slot(l) == slot))
        .cloned()
        .collect();
    links.push(link);
    let at_numbers = number_links(&numbers, &links);
    let outside = |id, path: &[String]| value_kind(nodes, id, path);
    settle_numbers(&mut numbers, &at_numbers, &outside);
    let given = kind_at(&numbers, &outside, source, item);
    let taken = kind_at(&numbers, &outside, target, slot);
    let (Some(given), Some(taken)) = (given, taken) else {
        return Some(None);
    };
    if convertible(given, taken) {
        return Some(None);
    }
    // Why a product or quotient at an end cannot take the kind of the other end.
    let why = [(target, slot, given), (source, item, taken)]
        .into_iter()
        .find_map(|(id, path, kind)| {
            let (_, number, switch) = numbers.iter().find(|n| n.0 == id)?;
            let mut open = number.clone();
            if !switch && open.operation.combines() {
                // Its kinds as the other links alone give them.
                let others: Vec<Link> = (at_numbers.iter())
                    .filter(|l| !(l.source == source && l.target == target && l.item == item))
                    .cloned()
                    .collect();
                let mut alone = numbers.clone();
                settle_numbers(&mut alone, &others, &outside);
                open = alone.iter().find(|n| n.0 == id)?.1.clone();
                for (i, key) in ["a", "b"].into_iter().enumerate() {
                    let linked = others.iter().any(|l| {
                        l.target == id
                            && matches!(links::slot(l), [c, k] if c == links::PARAMETERS && k == key)
                    });
                    if !linked {
                        open.operands[i] = None;
                    }
                }
                if !others.iter().any(|l| l.source == id) {
                    open.kind = None;
                }
                open.with_kind(false, path, kind).err().map(|why| (id, path, kind, why))
            } else {
                None
            }
        });
    let label = |kind: ValueKind| kind.describe();
    let name = |id: u64| (nodes.iter().find(|n| n.id == id)).map_or("", |n| n.name.as_str());
    Some(Some(match why {
        Some((id, path, kind, why)) => format!(
            "{} of {} cannot be a {}: {why}.",
            parameter_label(nodes, id, path),
            name(id),
            label(kind)
        ),
        None => format!(
            "{} of {} is a {} and {} of {} a {}.",
            item.get(1).map_or("", String::as_str),
            name(source),
            label(given),
            parameter_label(nodes, target, slot),
            name(target),
            label(taken)
        ),
    }))
}

/// Whether the row of `path` of a block is a number of a Math or Switch block that still
/// takes any kind: its result, or an input of the kind of the block, while it has none; an
/// input of a product or quotient while it has none of its own.
pub(super) fn open_number(node: &Node, path: &[String]) -> bool {
    node.number_kind(path) == Some(None)
}

impl Viewer {
    /// The rows of a Math or Switch block that edit it in place.
    pub(super) fn edit_number_row(
        &mut self,
        ui: &mut egui::Ui,
        id: u64,
        row: &Row,
        widget: RowWidget,
        field: egui::Rect,
        tip: &str,
    ) {
        let Some(number) = self.block(id).and_then(|b| b.number.clone()) else {
            return;
        };
        let (block, switch) = number;
        match widget {
            RowWidget::Operation => {
                let mut operation = block.operation;
                egui::ComboBox::from_id_salt(("math operation", id))
                    .selected_text(operation.label())
                    .width(field.width())
                    .show_ui(ui, |ui| {
                        for choice in MathOp::ALL {
                            if choice == MathOp::SeriesMax {
                                ui.separator();
                                ui.weak("Of a series");
                            }
                            ui.selectable_value(&mut operation, choice, choice.label())
                                .on_hover_text(choice.formula());
                        }
                    })
                    .response
                    .on_hover_text(tip);
                if operation != block.operation {
                    self.actions.push(Action::SetOperation(id, operation));
                }
            }
            RowWidget::Input(index) => {
                let Some((key, _, kind, mut value)) = block.inputs(switch).into_iter().nth(index)
                else {
                    return;
                };
                // A series a link gives shows how many values it has.
                if let Some(series) = block.series.get(index).cloned().flatten() {
                    ui.add_enabled_ui(false, |ui| {
                        ui.add_sized(
                            field.size(),
                            egui::Button::new(format!("{} values", series.len())),
                        )
                    })
                    .response
                    .on_disabled_hover_text(tip);
                    return;
                }
                let before = value;
                let kind = kind.unwrap_or(ValueKind::Factor);
                let drag = crate::parameter_set::field(&mut value, kind, block.units);
                number_field(ui, field, row, drag, tip);
                if value != before {
                    self.actions.push(Action::SetNumberInput(id, key, value));
                }
            }
            RowWidget::Choices if block.formula.is_some() => {
                let count = block.values.len();
                let most = plx_model::formula::MAX_INPUTS;
                ui.horizontal(|ui| {
                    let add = ui.add_enabled(count < most, egui::Button::new("Add").small());
                    if add.on_hover_text("Adds an input").clicked() {
                        self.actions.push(Action::SetChoices(id, count + 1));
                    }
                    let remove = ui.add_enabled(count > 0, egui::Button::new("Remove").small());
                    if remove
                        .on_hover_text("Removes the last input, with its link")
                        .clicked()
                    {
                        self.actions.push(Action::SetChoices(id, count - 1));
                    }
                });
            }
            RowWidget::Choices => {
                let count = block.values.len().saturating_sub(1);
                ui.horizontal(|ui| {
                    let add = ui.add_enabled(count < MAX_CHOICES, egui::Button::new("Add").small());
                    if add.on_hover_text("Adds a choice").clicked() {
                        self.actions.push(Action::SetChoices(id, count + 1));
                    }
                    let remove = ui.add_enabled(count > 1, egui::Button::new("Remove").small());
                    if remove
                        .on_hover_text("Removes the last choice, with its link")
                        .clicked()
                    {
                        self.actions.push(Action::SetChoices(id, count - 1));
                    }
                });
            }
            _ => {}
        }
    }
}
