//! Series of numbers along value links, like the arrays of NumPy: the values of a field over
//! a region or the whole history of a step that a Measure block gives. Math blocks compute
//! with them element by element or reduce them to one number (Max, Min, Mean); Plot blocks
//! draw them. Every other block takes one number, so a series cannot go there.

use super::*;

/// The output of a Measure block with the times of a whole history.
pub(super) const TIME: &str = "Time";

/// How deep a chain of Math blocks is followed back to find a series.
const MAX_DEPTH: usize = 64;

/// Whether `item` of block `id` gives a series of numbers, with the links `links`: a Measure
/// block of field values or a whole history, and a Math block that computes element by
/// element with a series at an input.
pub(super) fn gives_series(nodes: &[Node], links: &[Link], id: u64, item: &[String]) -> bool {
    gives_series_at(nodes, links, id, item, 0)
}

fn gives_series_at(nodes: &[Node], links: &[Link], id: u64, item: &[String], depth: usize) -> bool {
    let Some(node) = nodes.iter().find(|n| n.id == id) else {
        return false;
    };
    if let Some(measure) = node.as_measure() {
        return match item {
            [cell, name] if cell == links::VALUE && name == RESULT => measure.kind.is_series(),
            [cell, name] if cell == links::VALUE && name == TIME => measure.kind.is_series(),
            _ => false,
        };
    }
    let Some((number, false)) = node.as_number() else {
        return false;
    };
    if number.operation.reduces() || depth > MAX_DEPTH || item != [links::VALUE, RESULT] {
        return false;
    }
    (links.iter())
        .filter(|l| l.target == id && links::is_value(l))
        .filter(|l| matches!(links::slot(l), [cell, _] if cell == links::PARAMETERS))
        .any(|l| gives_series_at(nodes, links, l.source, &l.item, depth + 1))
}

/// Whether the row `slot` of block `id` takes a series: an input of a Math block, or a
/// series input of a Plot block.
pub(super) fn takes_series(nodes: &[Node], id: u64, slot: &[String]) -> bool {
    let Some(node) = nodes.iter().find(|n| n.id == id) else {
        return false;
    };
    let [cell, key] = slot else {
        return false;
    };
    if cell != links::PARAMETERS {
        return false;
    }
    if let Some((number, false)) = node.as_number() {
        return number.inputs(false).iter().any(|(k, ..)| k == key);
    }
    if node.as_preview().is_some() {
        return key == visualize::PREVIEW_VALUE;
    }
    node.as_plot().is_some_and(|plot| plot.takes_series(key))
}

/// The first value link of `links` that brings a series to a row that takes one number.
pub(super) fn series_conflict<'a>(nodes: &[Node], links: &'a [Link]) -> Option<&'a Link> {
    (links.iter())
        .filter(|l| l.item.first().is_some_and(|c| c == links::VALUE))
        .find(|l| {
            gives_series(nodes, links, l.source, &l.item)
                && !takes_series(nodes, l.target, links::slot(l))
        })
}

/// Why a new value link from `item` of `source` to `slot` of `target` cannot be because of a
/// series: it would bring one, or make a Math block bring one, to a row that takes one number.
pub(super) fn series_refusal(
    nodes: &[Node],
    existing: &[Link],
    source: u64,
    item: &[String],
    target: u64,
    slot: &[String],
) -> Option<String> {
    let link = Link {
        source,
        target,
        item: item.to_vec(),
        slot: slot.to_vec(),
        ..Default::default()
    };
    let mut links: Vec<Link> = (existing.iter())
        .filter(|l| !(l.target == target && links::slot(l) == slot))
        .cloned()
        .collect();
    links.push(link.clone());
    let conflict = series_conflict(nodes, &links)?;
    let name = |id: u64| (nodes.iter().find(|n| n.id == id)).map_or("", |n| n.name.as_str());
    let row = |l: &Link| parameter_label(nodes, l.target, links::slot(l));
    Some(if *conflict == link {
        format!(
            "{} of {} is a series of values, and {} of {} takes one number: a Math block \
             makes one of it (Max, Min, Mean).",
            item.get(1).map_or("", String::as_str),
            name(source),
            row(conflict),
            name(target),
        )
    } else {
        format!(
            "With this link {} gives a series of values to {} of {}, which takes one number.",
            name(conflict.source),
            row(conflict),
            name(conflict.target),
        )
    })
}

impl ProjectApp {
    /// The series a block gives at `item`: of a Measure block once it is up to date, of a
    /// Math block computing element by element with one; `None` for one number.
    pub(super) fn series_of(&self, id: u64, item: &[String]) -> Option<Vec<f64>> {
        let node = self.nodes.iter().find(|n| n.id == id)?;
        if let Some(measure) = node.as_measure() {
            if !measure.kind.is_series() || self.measure_state(id) != CellState::UpToDate {
                return None;
            }
            return match item {
                [cell, name] if cell == links::VALUE && name == RESULT => {
                    Some(measure.series.clone())
                }
                [cell, name] if cell == links::VALUE && name == TIME => Some(measure.times.clone()),
                _ => None,
            };
        }
        let (number, switch) = node.as_number()?;
        (item == [links::VALUE, RESULT])
            .then(|| number.series_result(switch))
            .flatten()?
            .ok()
    }

    /// Gives the series of value links to the Math and Plot blocks they go to, and the
    /// numbers of value links to Plot blocks; whether any input changed.
    pub(super) fn apply_series(&mut self) -> bool {
        let mut taken: HashMap<u64, BTreeMap<String, Vec<f64>>> = HashMap::new();
        let mut given_series = std::collections::HashSet::new();
        for link in self.links.iter().filter(|l| links::is_value(l)) {
            let [cell, key] = links::slot(link) else {
                continue;
            };
            if cell != links::PARAMETERS {
                continue;
            }
            let target = self.nodes.iter().find(|n| n.id == link.target);
            let plot = target.is_some_and(|n| n.as_plot().is_some() || n.as_preview().is_some());
            let number = target.is_some_and(|n| n.as_number().is_some_and(|(_, s)| !s));
            if !plot && !number {
                continue;
            }
            let series = self.series_of(link.source, &link.item);
            if series.is_some() {
                given_series.insert(link.target);
            }
            let values = match series {
                Some(series) => series,
                // A Math block takes a single number as it always did.
                None if number => continue,
                None => match self.value_of(link.source, &link.item) {
                    Some(value) => vec![value],
                    None => continue,
                },
            };
            taken
                .entry(link.target)
                .or_default()
                .insert(key.clone(), values);
        }
        let mut changed = false;
        for node in &mut self.nodes {
            let mut given = taken.remove(&node.id).unwrap_or_default();
            if let Some((number, false)) = node.as_number_mut() {
                changed |= number.set_series(&mut given);
            } else if let Some(plot) = node.as_plot_mut() {
                changed |= plot.take_inputs(given);
            } else if let Block::Preview(preview) = &mut node.block {
                let series = given_series.contains(&node.id);
                let values = given.remove(visualize::PREVIEW_VALUE).map(|v| (v, series));
                changed |= preview.take_values(values);
            }
        }
        changed
    }

    /// Removes the value links that bring a series to a row that takes one number, after a
    /// Measure block or a Math block started to give one.
    pub(super) fn drop_series_conflicts(&mut self) {
        let mut removed = Vec::new();
        while let Some(conflict) = series_conflict(&self.nodes, &self.links).cloned() {
            removed.push(format!(
                "{} to {}",
                self.name_of(conflict.source),
                self.name_of(conflict.target)
            ));
            self.links.retain(|l| *l != conflict);
        }
        if !removed.is_empty() {
            self.output.push(format!(
                "Links removed, as they would bring a series of values to a number: {}.",
                removed.join(", ")
            ));
            self.sync_due = true;
        }
    }
}

impl NumberBlock {
    /// Each input as a series: the series a link gives it, else its one number.
    fn input_series(&self) -> Vec<Vec<f64>> {
        (0..self.operation.inputs().len())
            .map(|i| match self.series.get(i).cloned().flatten() {
                Some(series) => series,
                None => vec![self.values.get(i).copied().unwrap_or(0.0)],
            })
            .collect()
    }

    /// The result of a Math block that computes element by element with a series at an
    /// input: the series, or why the inputs do not go together. `None` for one number.
    pub(super) fn series_result(&self, switch: bool) -> Option<Result<Vec<f64>, String>> {
        if switch || self.operation.reduces() || !self.series.iter().any(Option::is_some) {
            return None;
        }
        let inputs = self.input_series();
        let slices: Vec<&[f64]> = inputs.iter().map(Vec::as_slice).collect();
        Some(plx_model::apply_series(self.operation, &slices))
    }

    /// The one number a Max, Min, Max Abs or Mean block makes of the series at its input, or
    /// of its one number.
    pub(super) fn reduced(&self) -> Option<f64> {
        let inputs = self.input_series();
        self.operation.reduce(inputs.first()?)
    }

    /// Takes the series links give its inputs, by key; whether any changed.
    fn set_series(&mut self, given: &mut BTreeMap<String, Vec<f64>>) -> bool {
        let series: Vec<Option<Vec<f64>>> = (self.inputs(false).into_iter())
            .map(|(key, ..)| given.remove(&key))
            .collect();
        let changed = series != self.series;
        self.series = series;
        changed
    }
}

/// A series as a block shows it: the range of its values with the unit of `kind`, and how
/// many there are.
pub(super) fn shown_series(series: &[f64], kind: Option<ValueKind>, units: UnitSystem) -> String {
    let finite = || series.iter().copied().filter(|v| v.is_finite());
    match (finite().reduce(f64::min), finite().reduce(f64::max)) {
        (Some(low), Some(high)) => crate::measure::range_text(
            &numbers::shown_number(low, kind, units),
            &numbers::shown_number(high, kind, units),
            series.len(),
        ),
        _ => format!("{} values", series.len()),
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

    /// A Measure block of the values of a field (1), a parameters block (2), a sum (3), a
    /// maximum (4) and a plot (5).
    fn blocks() -> Vec<Node> {
        let measure = Node::read(ProjectNode::Measure(file::MeasureBlock {
            id: 1,
            name: "Stresses".into(),
            kind: MeasureKind::default(),
            ..Default::default()
        }));
        let parameters = Node::read(ProjectNode::Parameters(file::ParameterSet {
            id: 2,
            name: "Parameters".into(),
            parameters: vec![Parameter::new(
                "Limit",
                ValueKind::Quantity(Quantity::Pressure),
                1.0,
            )],
            ..Default::default()
        }));
        let math = |id, operation| {
            Node::read(ProjectNode::Math(file::MathBlock {
                id,
                name: format!("Math-{id}"),
                operation,
                values: vec![0.0, 2.0],
                ..Default::default()
            }))
        };
        let plot = Node::read(ProjectNode::Plot(file::PlotBlock {
            id: 5,
            name: "Plot".into(),
            kind: plx_model::PlotKind::Histogram,
            bins: 10,
            ..Default::default()
        }));
        vec![
            measure,
            parameters,
            math(3, MathOp::Multiply),
            math(4, MathOp::SeriesMax),
            plot,
        ]
    }

    #[test]
    fn a_series_goes_to_math_and_plot_blocks_but_not_to_a_number() {
        let nodes = blocks();
        let result = ["Value", "Result"];
        let limit = ["Value", "Limit"];
        assert!(gives_series(&nodes, &[], 1, &path(&result)));
        // Straight to a parameter: refused, with the way round it.
        let why = series_refusal(&nodes, &[], 1, &path(&result), 2, &path(&limit)).unwrap();
        assert!(why.contains("Max, Min, Mean"), "{why}");
        // A product with a series is one, a maximum of it one number.
        let product = [link(1, &result, 3, &["Parameters", "a"])];
        assert!(gives_series(&nodes, &product, 3, &path(&result)));
        assert!(series_refusal(&nodes, &product, 3, &path(&result), 2, &path(&limit)).is_some());
        let max = [link(1, &result, 4, &["Parameters", "a"])];
        assert!(!gives_series(&nodes, &max, 4, &path(&result)));
        assert_eq!(
            series_refusal(&nodes, &max, 4, &path(&result), 2, &path(&limit)),
            None
        );
        // A link that makes a Math block give a series to a number is refused too.
        let onward = [link(3, &result, 2, &["Value", "Limit"])];
        let why = series_refusal(
            &nodes,
            &onward,
            1,
            &path(&result),
            3,
            &path(&["Parameters", "a"]),
        )
        .unwrap();
        assert!(why.starts_with("With this link Math-3"), "{why}");
        // A plot takes a series, its bins only a number.
        assert_eq!(
            series_refusal(
                &nodes,
                &[],
                1,
                &path(&result),
                5,
                &path(&["Parameters", "values"])
            ),
            None
        );
        assert!(
            series_refusal(
                &nodes,
                &[],
                1,
                &path(&result),
                5,
                &path(&["Parameters", "bins"])
            )
            .is_some()
        );
        assert_eq!(
            refusal(
                &nodes,
                &[],
                1,
                &path(&result),
                5,
                &path(&["Parameters", "values"])
            ),
            None
        );
    }

    #[test]
    fn a_math_block_computes_with_series_value_by_value() {
        let mut nodes = blocks();
        let (number, _) = nodes[2].as_number_mut().unwrap();
        let mut given = BTreeMap::from([("a".to_owned(), vec![1.0, 2.0, 3.0])]);
        assert!(number.set_series(&mut given));
        assert_eq!(number.result(false), None);
        assert_eq!(number.series_result(false), Some(Ok(vec![2.0, 4.0, 6.0])));
        let (max, _) = nodes[3].as_number_mut().unwrap();
        let mut given = BTreeMap::from([("a".to_owned(), vec![1.0, 5.0, 3.0])]);
        max.set_series(&mut given);
        assert_eq!(max.series_result(false), None);
        assert_eq!(max.result(false), Some(5.0));
    }
}
