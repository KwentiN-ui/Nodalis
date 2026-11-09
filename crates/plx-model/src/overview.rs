//! The step overview: a table of every boundary condition, load, defined temperature and
//! model-wide item against the steps, Abaqus' load manager. Each cell says what the item is
//! in that step compared with the step before it: created, the same, changed, switched off,
//! removed, or acting in every step because it belongs to the model. Nothing here is stored;
//! the overview is derived from the steps, which refer to the same item by its stable id.

use crate::{BoltAction, FeModel, Id, Step};

/// The kind of item a row of the overview is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OverviewGroup {
    BoundaryCondition,
    Load,
    DefinedField,
    /// Constraints, ties, contact and bolts: they belong to the model, not to a step.
    ModelWide,
}

impl OverviewGroup {
    pub fn label(self) -> &'static str {
        match self {
            Self::BoundaryCondition => "Boundary conditions",
            Self::Load => "Loads",
            Self::DefinedField => "Defined fields",
            Self::ModelWide => "Model-wide",
        }
    }
}

/// The step item a row stands for, the same in every step: boundary conditions and loads
/// by their id, defined fields, which have none, by their name.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum StepItemKey {
    BoundaryCondition(Id),
    Load(Id),
    DefinedField(String),
}

/// What an item is in one step.
#[derive(Clone, Debug, PartialEq)]
pub enum Cell {
    /// Not in the step, nor in the step before it.
    Absent,
    /// New in the step: the step before it does not have it.
    Created,
    /// As in the step before it.
    Same,
    /// In the step before it too, with other values or another region.
    Changed,
    /// In the step, but switched off.
    Off,
    /// In the step before it, not in this one.
    Removed,
    /// A load of the step before it that this static step moves down to zero over its time.
    RampsToZero,
    /// A model-wide item that acts in the step.
    ModelWide,
    /// A model-wide item that does not act in the step (switched off from a step on, or
    /// as a whole).
    ModelWideOff,
    /// What a model-wide item does in the step in words: "Load", "Lock", "Friction 0.2".
    Action(String),
}

impl Cell {
    pub fn text(&self) -> &str {
        match self {
            Cell::Absent => "",
            Cell::Created => "Created",
            Cell::Same => "Same",
            Cell::Changed => "Changed",
            Cell::Off => "Off",
            Cell::Removed => "Removed",
            Cell::RampsToZero => "Ramps to 0",
            Cell::ModelWide => "Model-wide",
            Cell::ModelWideOff => "Off",
            Cell::Action(text) => text,
        }
    }

    /// Whether the item acts in the step.
    pub fn acts(&self) -> bool {
        matches!(
            self,
            Cell::Created
                | Cell::Same
                | Cell::Changed
                | Cell::RampsToZero
                | Cell::ModelWide
                | Cell::Action(_)
        )
    }

    /// Whether the step has the item itself, so that it can be opened there.
    pub fn defined(&self) -> bool {
        matches!(self, Cell::Created | Cell::Same | Cell::Changed | Cell::Off)
    }
}

/// A row of the overview.
#[derive(Clone, Debug, PartialEq)]
pub struct OverviewRow {
    pub group: OverviewGroup,
    /// The step item, for boundary conditions, loads and defined fields.
    pub key: Option<StepItemKey>,
    pub name: String,
    /// One cell per step.
    pub cells: Vec<Cell>,
}

/// A column of the overview: a step.
#[derive(Clone, Debug, PartialEq)]
pub struct OverviewColumn {
    pub name: String,
    pub active: bool,
    /// Whether the step leaves the state as it found it.
    pub probing: bool,
    /// Where the step starts: "Starts from: End of Preload", "Base: Unloaded model".
    pub start: String,
}

/// The table of items against steps.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct StepOverview {
    pub columns: Vec<OverviewColumn>,
    pub rows: Vec<OverviewRow>,
}

impl FeModel {
    /// Every boundary condition, load, defined field and model-wide item against the
    /// steps, each cell compared with the step the step starts from.
    pub fn step_overview(&self) -> StepOverview {
        let columns = (self.steps.iter().enumerate())
            .map(|(s, step)| OverviewColumn {
                name: step.name.clone(),
                active: step.active,
                probing: !step.kind.continues_state(),
                start: self.step_handover(s).summary(),
            })
            .collect();
        let mut rows = Vec::new();
        // The rows in the order the items first appear.
        let mut keys: Vec<(StepItemKey, String)> = Vec::new();
        for step in &self.steps {
            for (key, name) in step_items(step) {
                if !keys.iter().any(|(k, _)| *k == key) {
                    keys.push((key, name));
                }
            }
        }
        for group in [
            OverviewGroup::BoundaryCondition,
            OverviewGroup::Load,
            OverviewGroup::DefinedField,
        ] {
            for (key, name) in keys.iter().filter(|(k, _)| key_group(k) == group) {
                let cells = (0..self.steps.len())
                    .map(|s| self.step_item_cell(s, key))
                    .collect();
                rows.push(OverviewRow {
                    group,
                    key: Some(key.clone()),
                    name: name.clone(),
                    cells,
                });
            }
        }
        self.model_wide_rows(&mut rows);
        StepOverview { columns, rows }
    }

    /// What the step item `key` is in step `s`.
    fn step_item_cell(&self, s: usize, key: &StepItemKey) -> Cell {
        let step = &self.steps[s];
        // A step that cannot take such items has none, which says nothing.
        let takes = match key {
            StepItemKey::BoundaryCondition(_) => true,
            StepItemKey::Load(_) => step.kind.supports_loads(),
            StepItemKey::DefinedField(_) => step.kind.supports_defined_fields(),
        };
        let item = find_item(step, key);
        if item.is_none() && !takes {
            return Cell::Absent;
        }
        // The step it starts from, or a probing step's base: the last active step before
        // it that passes its state on.
        let previous = (0..s)
            .rev()
            .find(|&p| self.steps[p].active && self.steps[p].kind.continues_state());
        let before = previous.and_then(|p| find_item(&self.steps[p], key));
        match (item, before) {
            (Some(item), _) if !item.active() => Cell::Off,
            (Some(_), None) => Cell::Created,
            (Some(_), Some(before)) if !before.active() => Cell::Created,
            (Some(item), Some(before)) if item.same_as(&before) => Cell::Same,
            (Some(_), Some(_)) => Cell::Changed,
            (None, Some(before)) if before.active() => {
                let ramps = matches!(key, StepItemKey::Load(_))
                    && (self.handover(s, step).removed_loads).contains(&before.name());
                if ramps {
                    Cell::RampsToZero
                } else {
                    Cell::Removed
                }
            }
            (None, _) => Cell::Absent,
        }
    }

    fn model_wide_rows(&self, rows: &mut Vec<OverviewRow>) {
        let steps = self.steps.len();
        let mut row = |name: &str, cells: Vec<Cell>| {
            rows.push(OverviewRow {
                group: OverviewGroup::ModelWide,
                key: None,
                name: name.to_owned(),
                cells,
            });
        };
        let every = |active: bool| {
            vec![
                if active {
                    Cell::ModelWide
                } else {
                    Cell::ModelWideOff
                };
                steps
            ]
        };
        for constraint in &self.constraints {
            row(constraint.name(), every(constraint.active()));
        }
        for tie in &self.ties {
            row(&tie.name, every(tie.active));
        }
        for tie in &self.node_ties {
            row(&tie.name, every(tie.active));
        }
        // Contact switched off or given another friction from a step on.
        for region in &self.contact_regions {
            let mut state = region.state_after(&[]);
            let cells = (self.steps.iter())
                .map(|step| {
                    if !step.active {
                        return contact_cell(region.active, state, None);
                    }
                    let before = state;
                    state = region.state_in(step, state);
                    contact_cell(region.active, state, Some(before))
                })
                .collect();
            row(&region.name, cells);
        }
        for pair in &self.contact_pairs {
            row(&pair.name, every(pair.active));
        }
        for bolt in &self.bolts {
            let cells = (0..steps)
                .map(|s| match self.bolt_action(bolt, s) {
                    _ if !bolt.active => Cell::ModelWideOff,
                    Some(action) => Cell::Action(bolt_text(action)),
                    None => Cell::Absent,
                })
                .collect();
            row(&bolt.name, cells);
        }
        // The rigid point of a remote force keeps its selection rigid in every step.
        for (s, i, _, remote) in self.remote_forces() {
            if remote.behavior == crate::RemoteBehavior::Rigid {
                let load = &self.steps[s].loads[i];
                let name = format!("Rigid coupling of {} ({})", load.name, self.steps[s].name);
                row(&name, every(load.active));
            }
        }
    }

    /// Copies the step item `key` of step `s` into every following step that can take it,
    /// in place of the one there: the explicit "propagate" of other tools. Returns how many
    /// steps got it.
    pub fn copy_to_following_steps(&mut self, s: usize, key: &StepItemKey) -> usize {
        let Some(item) = self.steps.get(s).and_then(|step| find_item(step, key)) else {
            return 0;
        };
        let item = item.owned();
        let mut copied = 0;
        for step in self.steps.iter_mut().skip(s + 1) {
            if item.put_into(step) {
                copied += 1;
            }
        }
        copied
    }

    /// Removes the step item `key` from every step after step `s`. Returns from how many.
    pub fn remove_from_following_steps(&mut self, s: usize, key: &StepItemKey) -> usize {
        let mut removed = 0;
        for step in self.steps.iter_mut().skip(s + 1) {
            let before = step_items(step).len();
            match key {
                StepItemKey::BoundaryCondition(id) => {
                    step.boundary_conditions.retain(|b| b.id != *id)
                }
                StepItemKey::Load(id) => step.loads.retain(|l| l.id != *id),
                StepItemKey::DefinedField(name) => step.defined_fields.retain(|f| f.name != *name),
            }
            if step_items(step).len() < before {
                removed += 1;
            }
        }
        removed
    }
}

fn key_group(key: &StepItemKey) -> OverviewGroup {
    match key {
        StepItemKey::BoundaryCondition(_) => OverviewGroup::BoundaryCondition,
        StepItemKey::Load(_) => OverviewGroup::Load,
        StepItemKey::DefinedField(_) => OverviewGroup::DefinedField,
    }
}

/// The step items of a step with their names.
fn step_items(step: &Step) -> Vec<(StepItemKey, String)> {
    let boundaries = (step.boundary_conditions.iter())
        .map(|b| (StepItemKey::BoundaryCondition(b.id), b.name.clone()));
    let loads = (step.loads.iter()).map(|l| (StepItemKey::Load(l.id), l.name.clone()));
    let fields = (step.defined_fields.iter())
        .map(|f| (StepItemKey::DefinedField(f.name.clone()), f.name.clone()));
    boundaries.chain(loads).chain(fields).collect()
}

/// A step item, borrowed from a step or owned to be put into others.
#[derive(Clone)]
enum Item<B, L, F> {
    Boundary(B),
    Load(L),
    Field(F),
}

type ItemRef<'a> = Item<&'a crate::BoundaryCondition, &'a crate::Load, &'a crate::DefinedField>;
type ItemOwned = Item<crate::BoundaryCondition, crate::Load, crate::DefinedField>;

fn find_item<'a>(step: &'a Step, key: &StepItemKey) -> Option<ItemRef<'a>> {
    match key {
        StepItemKey::BoundaryCondition(id) => (step.boundary_conditions.iter())
            .find(|b| b.id == *id)
            .map(Item::Boundary),
        StepItemKey::Load(id) => (step.loads.iter()).find(|l| l.id == *id).map(Item::Load),
        StepItemKey::DefinedField(name) => (step.defined_fields.iter())
            .find(|f| f.name == *name)
            .map(Item::Field),
    }
}

impl ItemRef<'_> {
    fn active(&self) -> bool {
        match self {
            Item::Boundary(b) => b.active,
            Item::Load(l) => l.active,
            Item::Field(f) => f.active,
        }
    }

    fn name(&self) -> String {
        match self {
            Item::Boundary(b) => b.name.clone(),
            Item::Load(l) => l.name.clone(),
            Item::Field(f) => f.name.clone(),
        }
    }

    /// The same definition, whether shown or switched on aside.
    fn same_as(&self, other: &Self) -> bool {
        match (self, other) {
            (Item::Boundary(a), Item::Boundary(b)) => {
                let shown = |bc: &crate::BoundaryCondition| crate::BoundaryCondition {
                    active: true,
                    visible: true,
                    ..bc.clone()
                };
                shown(a) == shown(b)
            }
            (Item::Load(a), Item::Load(b)) => {
                let shown = |load: &crate::Load| crate::Load {
                    active: true,
                    visible: true,
                    ..load.clone()
                };
                shown(a) == shown(b)
            }
            (Item::Field(a), Item::Field(b)) => {
                let on = |field: &crate::DefinedField| crate::DefinedField {
                    active: true,
                    ..field.clone()
                };
                on(a) == on(b)
            }
            _ => false,
        }
    }

    fn owned(&self) -> ItemOwned {
        match self {
            Item::Boundary(b) => Item::Boundary((*b).clone()),
            Item::Load(l) => Item::Load((*l).clone()),
            Item::Field(f) => Item::Field((*f).clone()),
        }
    }
}

impl ItemOwned {
    /// Puts the item into `step` in place of the one with its id or name, or at the end;
    /// `false` when the step cannot take it.
    fn put_into(&self, step: &mut Step) -> bool {
        fn put<T: Clone>(list: &mut Vec<T>, item: &T, same: impl Fn(&T) -> bool) {
            match list.iter_mut().find(|i| same(i)) {
                Some(existing) => *existing = item.clone(),
                None => list.push(item.clone()),
            }
        }
        match self {
            Item::Boundary(b) if step.kind.supports_boundary(&b.kind) => {
                put(&mut step.boundary_conditions, b, |o| o.id == b.id)
            }
            Item::Load(l) if step.kind.supports_loads() && step.kind.supports_load(&l.kind) => {
                put(&mut step.loads, l, |o| o.id == l.id)
            }
            Item::Field(f) if step.kind.supports_defined_fields() => {
                put(&mut step.defined_fields, f, |o| o.name == f.name)
            }
            _ => return false,
        }
        true
    }
}

fn contact_cell(
    defined: bool,
    state: crate::ContactState,
    before: Option<crate::ContactState>,
) -> Cell {
    if !defined {
        return Cell::ModelWideOff;
    }
    let changed = before.is_some_and(|b| b != state);
    match (state.active, state.friction) {
        (false, _) if changed => Cell::Action("Off from here".into()),
        (false, _) => Cell::ModelWideOff,
        (true, _) if before.is_some_and(|b| !b.active) => Cell::Action("On from here".into()),
        (true, Some(mu)) if changed => Cell::Action(format!("Friction {mu}")),
        _ => Cell::ModelWide,
    }
}

fn bolt_text(action: BoltAction) -> String {
    match action {
        BoltAction::Load(_) => "Preload".into(),
        BoltAction::Adjust(_) => "Adjusted".into(),
        BoltAction::Lock => "Locked".into(),
        BoltAction::Open => "Open".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Base, BoundaryCondition, BoundaryKind, ContactBehavior, ContactRegion, Load, LoadKind,
        Region, StepKind,
    };

    fn fixed() -> BoundaryCondition {
        BoundaryCondition {
            id: Id::new(),
            name: "Fixed".into(),
            active: true,
            visible: true,
            region: Region::Parts(Vec::new()),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
            distribution: None,
        }
    }

    /// Preload with a support and a pressure, a frequency step about it, Unload without the
    /// pressure.
    fn chain() -> FeModel {
        let mut fe = FeModel::default();
        let mut preload = Step::new_static("Preload");
        preload.boundary_conditions.push(fixed());
        preload.loads.push(Load::new(
            "Pressure",
            Region::Parts(Vec::new()),
            LoadKind::Pressure(1.0),
        ));
        let mut modes = Step::new_frequency("Modes");
        if let StepKind::Frequency(settings) = &mut modes.kind {
            settings.base = Base::PreviousState;
        }
        modes.boundary_conditions = preload.boundary_conditions.clone();
        let mut unload = Step::new_static("Unload");
        unload.boundary_conditions = preload.boundary_conditions.clone();
        fe.steps = vec![preload, modes, unload];
        fe
    }

    fn row<'a>(overview: &'a StepOverview, name: &str) -> &'a OverviewRow {
        (overview.rows.iter()).find(|r| r.name == name).unwrap()
    }

    #[test]
    fn items_are_compared_with_the_step_before() {
        let mut fe = chain();
        let overview = fe.step_overview();
        assert_eq!(overview.columns[1].start, "Base: End of Preload");
        assert!(overview.columns[1].probing);
        assert_eq!(
            row(&overview, "Fixed").cells,
            [Cell::Created, Cell::Same, Cell::Same]
        );
        // The frequency step takes no loads; Unload, which starts from Preload, ramps the
        // pressure down.
        assert_eq!(
            row(&overview, "Pressure").cells,
            [Cell::Created, Cell::Absent, Cell::RampsToZero]
        );

        // Changed values and a switched off item; Unload is compared with Preload.
        if let BoundaryKind::Displacement(d) = &mut fe.steps[2].boundary_conditions[0].kind {
            d.values[0] = Some(1.0);
        }
        fe.steps[1].boundary_conditions[0].active = false;
        let overview = fe.step_overview();
        assert_eq!(
            row(&overview, "Fixed").cells,
            [Cell::Created, Cell::Off, Cell::Changed]
        );

        // A removed support of a static step after a static one.
        fe.steps[2].boundary_conditions.clear();
        let overview = fe.step_overview();
        assert_eq!(row(&overview, "Fixed").cells[2], Cell::Removed);
    }

    #[test]
    fn model_wide_items_show_in_every_step() {
        let mut fe = chain();
        let mut region = ContactRegion::new("Flange", ContactBehavior::Frictional { mu: 0.1 });
        let id = region.id;
        region.active = true;
        fe.contact_regions.push(region);
        let before = fe.contact_regions[0].state_after(&fe.steps[..2]);
        let off = crate::ContactState {
            active: false,
            ..before
        };
        fe.steps[2].set_contact_state(id, before, off);
        let overview = fe.step_overview();
        let flange = row(&overview, "Flange");
        assert_eq!(flange.group, OverviewGroup::ModelWide);
        assert_eq!(
            flange.cells,
            [
                Cell::ModelWide,
                Cell::ModelWide,
                Cell::Action("Off from here".into())
            ]
        );
    }

    #[test]
    fn items_are_copied_to_and_removed_from_following_steps() {
        let mut fe = chain();
        let pressure = StepItemKey::Load(fe.steps[0].loads[0].id);
        // The frequency step takes no loads; Unload does.
        assert_eq!(fe.copy_to_following_steps(0, &pressure), 1);
        assert!(fe.steps[1].loads.is_empty());
        assert_eq!(fe.steps[2].loads, fe.steps[0].loads);
        // Copying again replaces it instead of adding a second one.
        assert_eq!(fe.copy_to_following_steps(0, &pressure), 1);
        assert_eq!(fe.steps[2].loads.len(), 1);

        let support = StepItemKey::BoundaryCondition(fe.steps[0].boundary_conditions[0].id);
        assert_eq!(fe.remove_from_following_steps(0, &support), 2);
        assert!(fe.steps[1].boundary_conditions.is_empty());
        assert!(fe.steps[2].boundary_conditions.is_empty());
        assert_eq!(fe.steps[0].boundary_conditions.len(), 1);
    }
}
