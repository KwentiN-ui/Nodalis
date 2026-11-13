//! The time tables of a step: components of boundary conditions and loads that follow a
//! table of values over the step time instead of a fixed value and the amplitude of their
//! item (see [`crate::ValueTable`]).

use crate::{BoundaryKind, Step, ValueTable};

/// What a table belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TableOwner {
    /// Boundary condition `index` of the step; the component is the degree of freedom 0..6.
    BoundaryCondition(usize),
    /// Load `index` of the step; the component is that of the value 0..3.
    Load(usize),
}

/// A table of a component of an item of a step.
#[derive(Clone, Copy, Debug)]
pub struct StepTable<'a> {
    pub owner: TableOwner,
    pub component: usize,
    pub table: &'a ValueTable,
}

impl Step {
    /// The tables of the active boundary conditions and loads, in their order.
    pub fn tables(&self) -> Vec<StepTable<'_>> {
        let mut out = Vec::new();
        for (i, bc) in self.boundary_conditions.iter().enumerate() {
            if let (true, BoundaryKind::Displacement(d)) = (bc.active, &bc.kind) {
                for component in 0..6 {
                    if let Some(table) = d.table(component) {
                        out.push(StepTable {
                            owner: TableOwner::BoundaryCondition(i),
                            component,
                            table,
                        });
                    }
                }
            }
        }
        for (i, load) in self.loads.iter().enumerate().filter(|(_, l)| l.active) {
            let components = load.kind.table_components();
            for (component, table) in load.tables.iter().enumerate().take(components) {
                if let Some(table) = table {
                    out.push(StepTable {
                        owner: TableOwner::Load(i),
                        component,
                        table,
                    });
                }
            }
        }
        out
    }
}

impl crate::Load {
    /// The table of component `c` of the value, if the load takes one there and has it.
    pub fn table(&self, c: usize) -> Option<&ValueTable> {
        (c < self.kind.table_components())
            .then(|| self.tables[c].as_ref())
            .flatten()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BoundaryCondition, Displacement, Load, LoadKind, Region};

    fn table(value: f64) -> Option<ValueTable> {
        Some(ValueTable::ramp(value, 1.0))
    }

    #[test]
    fn a_step_lists_the_tables_of_held_components_and_active_items() {
        let mut displacement = Displacement::new([Some(0.0), None, Some(1.0), None, None, None]);
        displacement.tables[0] = table(1.0);
        // A free degree of freedom ignores its table.
        displacement.tables[1] = table(2.0);
        let bc = BoundaryCondition {
            id: crate::Id::new(),
            name: "Move".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![1]),
            kind: BoundaryKind::Displacement(displacement),
            amplitude: None,
            distribution: None,
        };
        let mut load = Load {
            id: crate::Id::new(),
            name: "Pressure".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![1]),
            kind: LoadKind::Pressure(1.0),
            amplitude: None,
            factor_amplitude: None,
            tables: [table(3.0), table(4.0), None],
            distribution: None,
            system: None,
            follower: false,
        };
        let mut step = Step::new_static("Step-1");
        step.boundary_conditions.push(bc.clone());
        step.loads.push(load.clone());
        let found: Vec<_> = (step.tables().iter())
            .map(|t| (t.owner, t.component))
            .collect();
        // A pressure has one component.
        assert_eq!(
            found,
            [
                (TableOwner::BoundaryCondition(0), 0),
                (TableOwner::Load(0), 0)
            ]
        );
        load.active = false;
        step.loads[0] = load;
        assert_eq!(step.tables().len(), 1);
        let BoundaryKind::Displacement(d) = &bc.kind else {
            unreachable!()
        };
        assert!(d.moves(0) && !d.still(0));
        assert!(d.moves(2) && !d.moves(1));
    }

    #[test]
    fn tables_are_left_out_of_the_project_file_until_there_is_one() {
        let mut displacement = Displacement::new([Some(0.0); 6]);
        let text = ron::to_string(&displacement).unwrap();
        assert!(!text.contains("tables"), "{text}");
        displacement.tables[1] = table(1.0);
        let text = ron::to_string(&displacement).unwrap();
        let back: Displacement = ron::from_str(&text).unwrap();
        assert_eq!(back, displacement);
    }
}
