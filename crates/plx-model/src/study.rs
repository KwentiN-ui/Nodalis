//! What a Parameter Study block of the project schematic holds, like the parameter set of
//! Ansys Workbench: the numbers it gives to other blocks (its inputs), the numbers it reads
//! back (its outputs) and a table of design points, each a row of input values with the
//! outputs it gave. One row is the current point: the values the blocks stand at.

use serde::{Deserialize, Serialize};

use crate::{Id, ValueKind};

/// A column of the table: an input the study gives through a value link, or an output it
/// reads. Links name the column by its id, so that renaming it keeps them.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StudyColumn {
    pub id: Id<Self>,
    pub name: String,
    /// The kind of number of an input, taken from the first parameter it was linked to;
    /// `None` for outputs, which show the units of their source.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub kind: Option<ValueKind>,
}

/// How a design point was solved last.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum PointRun {
    #[default]
    NotSolved,
    /// It gave its outputs.
    Solved,
    /// An output was missing: a solve or a measure failed.
    Failed,
}

/// A row of the table.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DesignPoint {
    /// A value per input column, stored in SI units.
    pub inputs: Vec<f64>,
    /// A value per output column, in the units its source stores it in; `None` while not
    /// solved.
    pub outputs: Vec<Option<f64>>,
    pub run: PointRun,
    /// How long it took to solve.
    pub seconds: f64,
    /// The elements of the largest mesh it was solved on.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub elements: Option<u64>,
    /// The fingerprint of the blocks the study runs as they were when it was solved,
    /// without the values the study gives them; see [`DesignPoint::state`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fingerprint: Option<u64>,
}

/// The state of a design point as the table shows it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointState {
    NotSolved,
    UpToDate,
    /// Solved, but the blocks changed since, an input was edited or an output added.
    OutOfDate,
    Failed,
}

impl PointState {
    pub fn label(self) -> &'static str {
        match self {
            Self::NotSolved => "not solved",
            Self::UpToDate => "up to date",
            Self::OutOfDate => "out of date",
            Self::Failed => "failed",
        }
    }
}

impl DesignPoint {
    /// Its state against the fingerprint of the blocks as they are now, `body`, with
    /// `outputs` output columns.
    pub fn state(&self, body: Option<u64>, outputs: usize) -> PointState {
        match self.run {
            PointRun::NotSolved => PointState::NotSolved,
            PointRun::Failed => PointState::Failed,
            PointRun::Solved
                if self.fingerprint.is_some()
                    && self.fingerprint == body
                    && self.outputs.len() == outputs
                    && self.outputs.iter().all(Option::is_some) =>
            {
                PointState::UpToDate
            }
            PointRun::Solved => PointState::OutOfDate,
        }
    }

    /// Forgets what it gave: it is solved anew.
    pub fn reset(&mut self) {
        self.outputs.iter_mut().for_each(|o| *o = None);
        self.run = PointRun::NotSolved;
        self.seconds = 0.0;
        self.elements = None;
        self.fingerprint = None;
    }
}

/// How the fill helper makes the values of an input.
#[derive(Clone, Debug, PartialEq)]
pub enum FillRule {
    /// The value of the current point.
    Current,
    /// These values, one or more.
    List(Vec<f64>),
    /// `count` values from `from` to `to`, evenly spaced.
    Range { from: f64, to: f64, count: usize },
}

impl FillRule {
    /// Its values, `current` the value of the current point.
    pub fn values(&self, current: f64) -> Vec<f64> {
        match self {
            Self::Current => vec![current],
            Self::List(values) => values.clone(),
            Self::Range { from, to, count } => match count {
                0 => Vec::new(),
                1 => vec![*from],
                n => (0..*n)
                    .map(|i| from + (to - from) * i as f64 / (n - 1) as f64)
                    .collect(),
            },
        }
    }

    /// Whether it varies the input.
    pub fn varies(&self) -> bool {
        !matches!(self, Self::Current)
    }
}

/// How the fill helper combines the values of the inputs into points.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FillMode {
    /// Every combination of the values.
    #[default]
    Grid,
    /// One input varied at a time, the others at the current point.
    OneAtATime,
}

/// The columns and points of a study, with the operations of its table.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct StudyTable {
    pub inputs: Vec<StudyColumn>,
    pub outputs: Vec<StudyColumn>,
    pub points: Vec<DesignPoint>,
    /// The index of the current point.
    pub current: usize,
}

impl StudyTable {
    /// The current point, the values the blocks stand at.
    pub fn current(&self) -> Option<&DesignPoint> {
        self.points.get(self.current)
    }

    /// The index of the input column whose id is written as `key`, as links name it.
    pub fn input(&self, key: &str) -> Option<usize> {
        self.inputs.iter().position(|c| c.id.to_string() == key)
    }

    /// The index of the output column whose id is written as `key`.
    pub fn output(&self, key: &str) -> Option<usize> {
        self.outputs.iter().position(|c| c.id.to_string() == key)
    }

    /// `base`, or `base (2)` and up when a column has the name already.
    pub fn unique_name(&self, base: &str) -> String {
        let taken = |name: &str| (self.inputs.iter().chain(&self.outputs)).any(|c| c.name == name);
        let base = if base.trim().is_empty() {
            "P"
        } else {
            base.trim()
        };
        if !taken(base) {
            return base.to_owned();
        }
        (2..)
            .map(|n| format!("{base} ({n})"))
            .find(|name| !taken(name))
            .unwrap_or_default()
    }

    /// Adds an input column of `kind`; every point gives it `value`, the value of what it
    /// was linked to, and a table without points gets its current point. Returns its id.
    pub fn add_input(
        &mut self,
        name: &str,
        kind: Option<ValueKind>,
        value: f64,
    ) -> Id<StudyColumn> {
        let id = Id::new();
        self.inputs.push(StudyColumn {
            id,
            name: self.unique_name(name),
            kind,
        });
        if self.points.is_empty() {
            self.points.push(DesignPoint::default());
            self.current = 0;
        }
        for point in &mut self.points {
            point.inputs.push(value);
            point.reset();
        }
        id
    }

    /// Adds an output column; the points solved before have no value for it yet. Returns
    /// its id.
    pub fn add_output(&mut self, name: &str) -> Id<StudyColumn> {
        let id = Id::new();
        self.outputs.push(StudyColumn {
            id,
            name: self.unique_name(name),
            kind: None,
        });
        if self.points.is_empty() {
            self.points.push(DesignPoint {
                inputs: vec![0.0; self.inputs.len()],
                ..Default::default()
            });
            self.current = 0;
        }
        for point in &mut self.points {
            point.outputs.resize(self.outputs.len(), None);
        }
        id
    }

    pub fn remove_input(&mut self, column: usize) {
        if column >= self.inputs.len() {
            return;
        }
        self.inputs.remove(column);
        for point in &mut self.points {
            if column < point.inputs.len() {
                point.inputs.remove(column);
            }
            point.reset();
        }
    }

    pub fn remove_output(&mut self, column: usize) {
        if column >= self.outputs.len() {
            return;
        }
        self.outputs.remove(column);
        for point in &mut self.points {
            if column < point.outputs.len() {
                point.outputs.remove(column);
            }
        }
    }

    /// Sets an input of a point, cleaned to its kind; the point is out of date once it
    /// changed.
    pub fn set_input(&mut self, point: usize, column: usize, value: f64) {
        let kind = self.inputs.get(column).and_then(|c| c.kind);
        let value = kind.map_or(value, |k| k.clean(value));
        let Some(point) = self.points.get_mut(point) else {
            return;
        };
        if let Some(input) = point.inputs.get_mut(column)
            && *input != value
        {
            *input = value;
            point.fingerprint = None;
        }
    }

    /// Adds a point with the inputs of the current one, not solved.
    pub fn add_point(&mut self) {
        let inputs = (self.current().map(|p| p.inputs.clone()))
            .unwrap_or_else(|| vec![0.0; self.inputs.len()]);
        self.points.push(DesignPoint {
            inputs,
            outputs: vec![None; self.outputs.len()],
            ..Default::default()
        });
    }

    /// Adds a copy of a point after the last one, not solved.
    pub fn duplicate(&mut self, point: usize) {
        if let Some(mut copy) = self.points.get(point).cloned() {
            copy.reset();
            self.points.push(copy);
        }
    }

    /// Removes a point; the current one stays.
    pub fn remove_point(&mut self, point: usize) {
        if point >= self.points.len() || point == self.current {
            return;
        }
        self.points.remove(point);
        if point < self.current {
            self.current -= 1;
        }
    }

    /// Forgets what every point gave; the inputs stay.
    pub fn reset(&mut self) {
        self.points.iter_mut().for_each(DesignPoint::reset);
    }

    /// The points to solve, in order: those not up to date, the current one last whenever
    /// another is solved, so that the blocks end at the current point.
    pub fn to_solve(&self, body: Option<u64>) -> Vec<usize> {
        let mut rows: Vec<usize> = (0..self.points.len())
            .filter(|&i| {
                self.points[i].state(body, self.outputs.len()) != PointState::UpToDate
                    && i != self.current
            })
            .collect();
        let current_due = self
            .current()
            .is_some_and(|p| p.state(body, self.outputs.len()) != PointState::UpToDate);
        if (!rows.is_empty() || current_due) && self.current < self.points.len() {
            rows.push(self.current);
        }
        rows
    }

    /// The inputs of the points the fill helper makes, each value cleaned to the kind of its
    /// input and without repeats.
    pub fn fill_rows(&self, rules: &[FillRule], mode: FillMode) -> Vec<Vec<f64>> {
        let current = (self.current().map(|p| p.inputs.clone()))
            .unwrap_or_else(|| vec![0.0; self.inputs.len()]);
        let values: Vec<Vec<f64>> = (self.inputs.iter().enumerate())
            .map(|(i, column)| {
                let rule = rules.get(i).unwrap_or(&FillRule::Current);
                let at = current.get(i).copied().unwrap_or(0.0);
                let values = rule.values(at);
                match column.kind {
                    Some(kind) => values.into_iter().map(|v| kind.clean(v)).collect(),
                    None => values,
                }
            })
            .collect();
        let mut rows: Vec<Vec<f64>> = Vec::new();
        match mode {
            FillMode::Grid => {
                if values.iter().any(Vec::is_empty) {
                    return Vec::new();
                }
                rows.push(Vec::new());
                for column in &values {
                    rows = (rows.iter())
                        .flat_map(|row| {
                            column.iter().map(move |&v| {
                                let mut row = row.clone();
                                row.push(v);
                                row
                            })
                        })
                        .collect();
                }
            }
            FillMode::OneAtATime => {
                for (i, column) in values.iter().enumerate() {
                    if !rules.get(i).is_some_and(FillRule::varies) {
                        continue;
                    }
                    for &v in column {
                        let mut row = current.clone();
                        row[i] = v;
                        rows.push(row);
                    }
                }
            }
        }
        let mut unique: Vec<Vec<f64>> = Vec::new();
        for row in rows {
            if !unique.contains(&row) {
                unique.push(row);
            }
        }
        unique
    }

    /// Adds points with these inputs, not solved, but none with the inputs of a point there
    /// already; `replace` first removes all points but the current one, which becomes DP 0.
    /// Returns how many were added.
    pub fn add_rows(&mut self, rows: Vec<Vec<f64>>, replace: bool) -> usize {
        if replace && self.current < self.points.len() {
            let current = self.points.swap_remove(self.current);
            self.points = vec![current];
            self.current = 0;
        }
        let mut added = 0;
        for mut inputs in rows {
            inputs.resize(self.inputs.len(), 0.0);
            for (value, column) in inputs.iter_mut().zip(&self.inputs) {
                if let Some(kind) = column.kind {
                    *value = kind.clean(*value);
                }
            }
            if self.points.iter().any(|p| p.inputs == inputs) {
                continue;
            }
            self.points.push(DesignPoint {
                inputs,
                outputs: vec![None; self.outputs.len()],
                ..Default::default()
            });
            added += 1;
        }
        added
    }

    /// How many points are in each state: up to date, failed, and all.
    pub fn summary(&self, body: Option<u64>) -> (usize, usize, usize) {
        let states = self
            .points
            .iter()
            .map(|p| p.state(body, self.outputs.len()));
        let (mut solved, mut failed) = (0, 0);
        for state in states {
            match state {
                PointState::UpToDate => solved += 1,
                PointState::Failed => failed += 1,
                _ => {}
            }
        }
        (solved, failed, self.points.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn columns_take_values_and_points_go_out_of_date() {
        let mut table = StudyTable::default();
        let force = table.add_input("Force-1 Y", Some(ValueKind::Factor), 2.0);
        assert_eq!(table.input(&force.to_string()), Some(0));
        assert_eq!(table.points.len(), 1);
        table.add_input("Force-1 Y", None, 1.0);
        assert_eq!(table.inputs[1].name, "Force-1 Y (2)");
        table.add_output("Stress");
        table.add_point();
        assert_eq!(table.points[1].inputs, [2.0, 1.0]);
        // Both are due, the current one last.
        assert_eq!(table.to_solve(Some(7)), [1, 0]);
        for point in &mut table.points {
            point.run = PointRun::Solved;
            point.outputs = vec![Some(1.0)];
            point.fingerprint = Some(7);
        }
        assert!(table.to_solve(Some(7)).is_empty());
        assert_eq!(table.summary(Some(7)), (2, 0, 2));
        // A change of the blocks makes all out of date.
        assert_eq!(table.points[1].state(Some(8), 1), PointState::OutOfDate);
        // An edited input only its point; the current one is solved after it.
        table.set_input(1, 0, 3.0);
        assert_eq!(table.to_solve(Some(7)), [1, 0]);
        table.add_output("Mass");
        assert_eq!(table.points[0].state(Some(7), 2), PointState::OutOfDate);
        table.remove_point(0);
        assert_eq!(table.points.len(), 2);
        table.remove_point(1);
        assert_eq!(table.points.len(), 1);
    }

    #[test]
    fn the_fill_helper_makes_grids_and_one_at_a_time() {
        let mut table = StudyTable::default();
        table.add_input("Force", None, 10.0);
        table.add_input("Count", Some(ValueKind::Count), 2.0);
        let range = FillRule::Range {
            from: 1.0,
            to: 3.0,
            count: 3,
        };
        assert_eq!(range.values(0.0), [1.0, 2.0, 3.0]);
        let count = FillRule::List(vec![1.4, 2.0, 4.0]);
        let grid = table.fill_rows(&[range.clone(), count.clone()], FillMode::Grid);
        assert_eq!(grid.len(), 9);
        assert_eq!(grid[1], [1.0, 2.0]);
        let one = table.fill_rows(&[range, count], FillMode::OneAtATime);
        assert_eq!(
            one,
            [
                vec![1.0, 2.0],
                vec![2.0, 2.0],
                vec![3.0, 2.0],
                vec![10.0, 1.0],
                vec![10.0, 2.0],
                vec![10.0, 4.0]
            ]
        );
        // The current point is not added again.
        assert_eq!(table.add_rows(one.clone(), false), 5);
        assert_eq!(table.add_rows(one, false), 0);
        table.current = 2;
        assert_eq!(table.add_rows(vec![vec![5.0, 1.0]], true), 1);
        assert_eq!(table.current, 0);
        assert_eq!(table.points[0].inputs, [2.0, 2.0]);
        assert_eq!(table.points.len(), 2);
    }
}
