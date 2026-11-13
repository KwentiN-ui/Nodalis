//! A parameter study whose blocks are only numbers (Parameters, Math, Switch and Formula
//! blocks) runs on a thread of its own: it needs no solve and no redraw of the window between
//! its points, so a thousand points take a blink instead of a frame each, and it goes on when
//! the window is not drawn.

use super::*;

/// A block of a study that only computes numbers, copied for the thread.
#[derive(Clone)]
enum Pure {
    Number(NumberBlock, bool),
    Parameters(Vec<Parameter>),
}

/// Where an output column takes its number from.
#[derive(Clone)]
enum Source {
    /// A row of a block of the study, `["Value", name]`.
    Block(u64, Vec<String>),
    /// A number that does not follow the inputs, read when the study started.
    Fixed(Option<f64>),
}

/// The blocks of a study and the links between them, as the study started: what the thread
/// evaluates for each point.
pub(super) struct NumberStudy {
    study: u64,
    blocks: Vec<(u64, Pure)>,
    /// The value links to the blocks, `(source, item, target, slot)`, sources first.
    links: Vec<(u64, Vec<String>, u64, Vec<String>)>,
    /// The numbers of sources outside the study, read when it started.
    fixed: HashMap<(u64, Vec<String>), f64>,
    /// The names of the input columns, by index.
    inputs: Vec<String>,
    outputs: Vec<Source>,
    /// The points to evaluate and their inputs.
    points: Vec<(usize, Vec<f64>)>,
}

/// What the thread found at a point.
pub(super) struct PointDone {
    pub(super) point: usize,
    pub(super) outputs: Vec<Option<f64>>,
    pub(super) seconds: f64,
}

/// A study under way on its thread: the points it sends back as it goes, and when it started.
pub(super) struct NumberRun {
    pub(super) receiver: Receiver<Vec<PointDone>>,
    pub(super) started: Instant,
}

/// How often the thread sends what it has: often enough for the block and the table to show
/// the progress, seldom enough that the window is not drawn for every point.
const BATCH: Duration = Duration::from_millis(50);

impl NumberStudy {
    /// The number `item` of the block `source` gives at a point with `inputs`.
    fn value(&self, source: u64, item: &[String], inputs: &[f64]) -> Option<f64> {
        if source == self.study {
            let [cell, name] = item else {
                return None;
            };
            if cell != links::LOOP {
                return None;
            }
            let column = self.inputs.iter().position(|n| n == name)?;
            return inputs.get(column).copied();
        }
        match self.blocks.iter().find(|(id, _)| *id == source) {
            Some((_, Pure::Number(number, switch))) => {
                matches!(item, [cell, name] if cell == links::VALUE && name == RESULT)
                    .then(|| number.result(*switch))
                    .flatten()
            }
            Some((_, Pure::Parameters(parameters))) => {
                let [_, name] = item else {
                    return None;
                };
                (parameters.iter().find(|p| &p.name == name)).map(|p| p.value)
            }
            None => self.fixed.get(&(source, item.to_vec())).copied(),
        }
    }

    /// Gives the blocks the inputs of a point, passes the numbers on until none changes, as
    /// [`ProjectApp::apply_values`] does, and reads the outputs.
    fn evaluate(&mut self, inputs: &[f64]) -> Vec<Option<f64>> {
        for _ in 0..=self.blocks.len() {
            let mut changed = false;
            for at in 0..self.links.len() {
                let (source, item, target, slot) = &self.links[at];
                let Some(value) = self.value(*source, item, inputs) else {
                    continue;
                };
                let Some((_, block)) = self.blocks.iter_mut().find(|(id, _)| id == target) else {
                    continue;
                };
                match (block, slot.as_slice()) {
                    (Pure::Number(number, switch), [_, key]) => {
                        changed |= number.set(*switch, key, value);
                    }
                    (Pure::Parameters(parameters), [cell, name]) if cell == links::VALUE => {
                        if let Some(parameter) = parameters.iter_mut().find(|p| &p.name == name) {
                            let value = parameter.kind.clean(value);
                            changed |= parameter.value != value;
                            parameter.value = value;
                        }
                    }
                    _ => {}
                }
            }
            if !changed {
                break;
            }
        }
        (self.outputs.iter())
            .map(|source| match source {
                Source::Block(id, item) => self.value(*id, item, inputs),
                Source::Fixed(value) => *value,
            })
            .collect()
    }

    /// Evaluates the points on a thread of its own, which sends them back in batches and has
    /// the window drawn for each; it stops once the receiver is gone.
    pub(super) fn spawn(mut self, ctx: &egui::Context) -> NumberRun {
        let (sender, receiver) = std::sync::mpsc::channel();
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let points = std::mem::take(&mut self.points);
            let mut batch = Vec::new();
            let mut sent = Instant::now();
            for (point, inputs) in points {
                let started = Instant::now();
                let outputs = self.evaluate(&inputs);
                batch.push(PointDone {
                    point,
                    outputs,
                    seconds: started.elapsed().as_secs_f64(),
                });
                if sent.elapsed() >= BATCH {
                    if sender.send(std::mem::take(&mut batch)).is_err() {
                        return;
                    }
                    ctx.request_repaint();
                    sent = Instant::now();
                }
            }
            let _ = sender.send(batch);
            ctx.request_repaint();
        });
        NumberRun {
            receiver,
            started: Instant::now(),
        }
    }
}

impl ProjectApp {
    /// The study `id` as its thread evaluates it, for the points `points`; `None` unless all
    /// the blocks it runs only compute numbers, of which none takes a series.
    pub(super) fn number_study(&self, id: u64, points: &[usize]) -> Option<NumberStudy> {
        let body = self.study_body(id);
        if body.is_empty() {
            return None;
        }
        let mut blocks = Vec::new();
        for &at in &body {
            let node = self.nodes.iter().find(|n| n.id == at)?;
            let pure = match &node.block {
                Block::Parameters(parameters) => Pure::Parameters(parameters.parameters.clone()),
                Block::Math(number) | Block::Switch(number) | Block::Formula(number) => {
                    if number.series.iter().any(Option::is_some) {
                        return None;
                    }
                    Pure::Number(number.clone(), node.kind() == Kind::Switch)
                }
                _ => return None,
            };
            blocks.push((at, pure));
        }
        let ids: Vec<u64> = self.nodes.iter().map(|n| n.id).collect();
        let order = links::order(&ids, &self.links);
        let rank = |id: u64| order.iter().position(|&o| o == id).unwrap_or(usize::MAX);
        let mut value_links: Vec<&Link> = (self.links.iter())
            .filter(|l| links::is_value(l) && body.contains(&l.target))
            .collect();
        value_links.sort_by_key(|l| rank(l.target));
        let mut fixed = HashMap::new();
        for link in &value_links {
            if link.source != id
                && !body.contains(&link.source)
                && let Some(value) = self.value_of(link.source, &link.item)
            {
                fixed.insert((link.source, link.item.clone()), value);
            }
        }
        let study = self.nodes.iter().find(|n| n.id == id)?.as_study()?;
        let table = &study.table;
        let outputs = (table.outputs.iter())
            .map(|column| match self.output_link(id, &column.name) {
                Some(link) if body.contains(&link.source) => {
                    Source::Block(link.source, link.item.clone())
                }
                _ => Source::Fixed(self.output_value(id, &column.name)),
            })
            .collect();
        Some(NumberStudy {
            study: id,
            blocks,
            links: (value_links.into_iter())
                .map(|l| (l.source, l.item.clone(), l.target, links::slot(l).to_vec()))
                .collect(),
            fixed,
            inputs: table.inputs.iter().map(|c| c.name.clone()).collect(),
            outputs,
            points: (points.iter())
                .filter_map(|&p| Some((p, table.points.get(p)?.inputs.clone())))
                .collect(),
        })
    }

    /// Takes in what the thread of a study sent; ends the study once the thread is done.
    pub(super) fn poll_number_study(&mut self) {
        let Some(run) = &self.looping else {
            return;
        };
        let id = run.id;
        let body = run.body;
        let Some(worker) = &run.number else {
            return;
        };
        let mut done = Vec::new();
        let finished = loop {
            match worker.receiver.try_recv() {
                Ok(batch) => done.extend(batch),
                Err(std::sync::mpsc::TryRecvError::Empty) => break false,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => break true,
            }
        };
        let started = worker.started;
        let (mut solved, mut failed) = (0, 0);
        if let Some(study) = self.node(id).and_then(Node::as_study_mut) {
            for point in done {
                let Some(row) = study.table.points.get_mut(point.point) else {
                    continue;
                };
                let missing = point.outputs.iter().any(Option::is_none);
                row.outputs = point.outputs;
                row.run = if missing {
                    PointRun::Failed
                } else {
                    PointRun::Solved
                };
                row.seconds = point.seconds;
                row.elements = None;
                row.fingerprint = body;
                study.active = Some(point.point);
                solved += 1;
                failed += usize::from(missing);
            }
        }
        if let Some(run) = &mut self.looping {
            run.solved += solved;
            run.failed += failed;
        }
        if finished {
            let count = self.looping.as_ref().map_or(0, |run| run.solved);
            self.output.push(format!(
                "{}: {count} design points evaluated in {:.0} ms, without solving.",
                self.name_of(id),
                started.elapsed().as_secs_f64() * 1000.0
            ));
            self.finish_study(id);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn link(
        source: u64,
        item: &[&str],
        target: u64,
        slot: &[&str],
    ) -> (u64, Vec<String>, u64, Vec<String>) {
        let strings = |s: &[&str]| s.iter().map(|s| (*s).to_owned()).collect();
        (source, strings(item), target, strings(slot))
    }

    #[test]
    fn a_number_study_passes_its_inputs_through_the_blocks() {
        // Study (1) -> Parameters (2) Length -> Math (3) A, B = 2 -> Result.
        let math = Node::read(ProjectNode::Math(file::MathBlock {
            id: 3,
            name: "Math".into(),
            operation: MathOp::Multiply,
            values: vec![0.0, 2.0],
            ..Default::default()
        }));
        let math = math.as_number().unwrap().0.clone();
        let mut study = NumberStudy {
            study: 1,
            blocks: vec![
                (
                    2,
                    Pure::Parameters(vec![Parameter::new("Length", ValueKind::Factor, 0.0)]),
                ),
                (3, Pure::Number(math, false)),
            ],
            links: vec![
                link(1, &[links::LOOP, "Length"], 2, &[links::VALUE, "Length"]),
                link(2, &[links::VALUE, "Length"], 3, &[links::PARAMETERS, "a"]),
            ],
            fixed: HashMap::new(),
            inputs: vec!["Length".into()],
            outputs: vec![
                Source::Block(3, vec![links::VALUE.into(), RESULT.into()]),
                Source::Fixed(Some(7.0)),
            ],
            points: Vec::new(),
        };
        assert_eq!(study.evaluate(&[5.0]), vec![Some(10.0), Some(7.0)]);
        assert_eq!(study.evaluate(&[0.25]), vec![Some(0.5), Some(7.0)]);
    }
}
