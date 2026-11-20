//! History sets straight from the solver's typed history blocks, without the `.dat` text:
//! the same sets, fields, components and entries [`super::parse_dat`] reads from the file
//! [`plxs_output::dat::write_dat`] writes, at full precision.

use std::collections::HashMap;

use plxs_core::mesh::Sets;
use plxs_core::results::{HistoryBlock, HistoryQuantity, HistorySource, Results};

use super::{
    ALL_CONTACT_ELEMENTS, Collector, DatImport, Ids, KINDS, Kind, ROTATION_KINDS, SetKey,
    contact_set_name, cyclic_mode, name_rotations_after_point, repair_set_name,
};

/// The history sets of `results`; `sets` names the sets the blocks are of.
pub fn import_blocks(results: &Results, sets: &Sets) -> DatImport {
    let mut import = Import {
        sets,
        data: Collector::default(),
        step: None,
        frequencies: HashMap::new(),
        offset: 0,
        cyclic_modes: 0,
        rotation_sets: Vec::new(),
        unknown: Vec::new(),
    };
    for block in &results.history {
        import.block(block);
    }
    let mut warnings = Vec::new();
    if !import.unknown.is_empty() {
        warnings.push(format!(
            "Unsupported history outputs skipped: {}",
            import.unknown.join(", ")
        ));
    }
    let rotation_sets = import.rotation_sets;
    let mut sets = import.data.finish();
    for set in sets.iter_mut().filter(|s| rotation_sets.contains(&s.name)) {
        name_rotations_after_point(set);
    }
    DatImport { sets, warnings }
}

struct Import<'a> {
    sets: &'a Sets,
    data: Collector,
    /// The step of the blocks so far; the modes below are those of this step.
    step: Option<u32>,
    /// The frequency of each mode of the step, from its eigenvalue table.
    frequencies: HashMap<u32, f64>,
    /// A cyclic symmetric frequency step numbers the modes of its nodal diameters on
    /// through the step: the modes before the current table, and all so far.
    offset: u32,
    cyclic_modes: u32,
    /// Sets with the values of the rotation node of a point.
    rotation_sets: Vec<String>,
    unknown: Vec<String>,
}

impl Import<'_> {
    fn block(&mut self, block: &HistoryBlock) {
        use HistoryQuantity as Q;
        if self.step != Some(block.step) {
            self.step = Some(block.step);
            self.frequencies.clear();
            (self.offset, self.cyclic_modes) = (0, 0);
        }
        match block.quantity {
            Q::Eigenvalues | Q::ComplexEigenvalues | Q::CyclicEigenvalues => {
                self.eigenvalues(block)
            }
            Q::TurningDirection => self.turning_direction(block),
            Q::CyclicTurningDirection => self.cyclic_turning_direction(block),
            Q::BucklingFactors => {
                let set = step_set(block);
                for (m, values) in block.rows.iter().filter(|r| r.0 != 0) {
                    let Some(&factor) = values.first() else {
                        continue;
                    };
                    let row = self.data.row(block.step, *m, factor);
                    let name = "BUCKLING_FACTOR";
                    (self.data).add(&set, "BUCKLING_FACTOR_OUTPUT", name, name, row, factor);
                }
            }
            Q::ParticipationFactors => self.rigid_table(block, "PARTICIPATION_FACTORS"),
            Q::EffectiveModalMass => self.rigid_table(block, "EFFECTIVE_MODAL_MASS"),
            // The totals of the effective modal mass are not shown.
            Q::TotalEffectiveMass => {}
            Q::DesignResponses => self.design_responses(block),
            Q::ContactStatistics => self.contact_statistics(block),
            _ => self.values(block),
        }
    }

    /// The name a set is shown under: upper case and without the prefix of the sets
    /// written for picked regions.
    fn shown(&self, name: &str) -> String {
        repair_set_name(name)
    }

    /// The eigenvalue table of a frequency step: rows `(mode, [eigenvalue, omega,
    /// frequency, imaginary part])`, `(mode, [omega, frequency, imaginary part of omega])`
    /// of a complex frequency step, `(mode, [nodal diameter, eigenvalue, omega, frequency,
    /// imaginary part])` of a cyclic symmetric one.
    fn eigenvalues(&mut self, block: &HistoryBlock) {
        let set = step_set(block);
        self.offset = self.cyclic_modes;
        for (mode, values) in block.rows.iter().filter(|r| r.0 != 0) {
            if block.quantity == HistoryQuantity::CyclicEigenvalues {
                let [diameter, rest @ ..] = &values[..] else {
                    continue;
                };
                if rest.len() != 4 {
                    continue;
                }
                self.cyclic_modes += 1;
                let Some(m) = cyclic_mode(self.offset + mode) else {
                    continue;
                };
                self.frequencies.insert(m, rest[2]);
                let row = self.data.row(block.step, m, rest[2]);
                let names = [
                    "NODAL_DIAMETER",
                    "EIGENVALUE",
                    "OMEGA",
                    "FREQUENCY",
                    "FREQUENCY_IM",
                ];
                for (name, value) in names.iter().zip(std::iter::once(diameter).chain(rest)) {
                    self.data
                        .add(&set, "EIGENVALUE_OUTPUT", name, name, row, *value);
                }
                continue;
            }
            let (frequency, names): (f64, &[&str]) = match values.len() {
                4 => (
                    values[2],
                    &["EIGENVALUE", "OMEGA", "FREQUENCY", "FREQUENCY_IM"],
                ),
                3 => (values[1], &["OMEGA", "FREQUENCY", "OMEGA_IM"]),
                _ => continue,
            };
            self.frequencies.insert(*mode, frequency);
            let row = self.data.row(block.step, *mode, frequency);
            for (name, value) in names.iter().zip(values) {
                self.data
                    .add(&set, "EIGENVALUE_OUTPUT", name, name, row, *value);
            }
        }
    }

    /// The row of a mode of the step, at its frequency.
    fn mode_row(&mut self, step: u32, m: u32) -> usize {
        let frequency = self.frequencies.get(&m).copied().unwrap_or(m as f64);
        self.data.row(step, m, frequency)
    }

    /// The whirl of each complex mode, `(mode, [+1 forward or −1 backward, nodal diameter
    /// or −1])`, and the axis `(0, [x, y, z])`, which is not shown.
    fn turning_direction(&mut self, block: &HistoryBlock) {
        let set = step_set(block);
        let diameters = block.rows.iter().any(|r| r.0 != 0 && r.1.len() > 1);
        for (m, values) in block.rows.iter().filter(|r| r.0 != 0) {
            let row = self.mode_row(block.step, *m);
            let direction = if values.first().copied().unwrap_or(1.0) >= 0.0 {
                1.0
            } else {
                -1.0
            };
            let name = "TURNING_DIRECTION";
            (self.data).add(&set, "EIGENVALUE_OUTPUT", name, name, row, direction);
            if diameters {
                // Every mode gets a value, −1 without a diameter, so the values line up
                // with the modes.
                let diameter = match values.get(1) {
                    Some(d) if *d >= 0.0 => f64::from(*d as u32),
                    _ => -1.0,
                };
                let name = "NODAL_DIAMETER";
                (self.data).add(&set, "EIGENVALUE_OUTPUT", name, name, row, diameter);
            }
        }
    }

    /// The turning direction of the modes of a nodal diameter of a cyclic symmetric model:
    /// the axis first, then `(mode within the diameter, [nodal diameter, +1 forward /
    /// −1 backward])`.
    fn cyclic_turning_direction(&mut self, block: &HistoryBlock) {
        let set = step_set(block);
        for (local, values) in block.rows.iter().skip(1) {
            let Some(m) = cyclic_mode(self.offset + local) else {
                continue;
            };
            let row = self.mode_row(block.step, m);
            let direction = if values.get(1).is_some_and(|d| *d > 0.0) {
                1.0
            } else {
                -1.0
            };
            let name = "TURNING_DIRECTION";
            (self.data).add(&set, "EIGENVALUE_OUTPUT", name, name, row, direction);
        }
    }

    /// The participation factors or effective modal masses by mode, `(mode, [x, y, z,
    /// rx, ry, rz])`; the row of totals is not shown.
    fn rigid_table(&mut self, block: &HistoryBlock, field: &str) {
        let set = step_set(block);
        for (mode, values) in block.rows.iter().filter(|r| r.0 != 0) {
            if values.len() != 6 {
                continue;
            }
            let m = if self.cyclic_modes > 0 {
                let Some(m) = cyclic_mode(self.offset + mode) else {
                    continue;
                };
                m
            } else {
                *mode
            };
            let row = self.mode_row(block.step, m);
            let names = [
                "X_COMPONENT",
                "Y_COMPONENT",
                "Z_COMPONENT",
                "X_ROTATION",
                "Y_ROTATION",
                "Z_ROTATION",
            ];
            for (name, value) in names.iter().zip(values) {
                self.data.add(&set, field, name, name, row, *value);
            }
        }
    }

    /// The design responses of a sensitivity step, `(index, [value, length of the
    /// gradient])`, the source naming their kinds joined by commas.
    fn design_responses(&mut self, block: &HistoryBlock) {
        let set = step_set(block);
        let row = self.data.row(block.step, 1, 0.0);
        let names: Vec<&str> = match &block.source {
            HistorySource::Named(names) => names.split(',').collect(),
            _ => Vec::new(),
        };
        for (id, values) in &block.rows {
            let Some(name) = names.get(*id as usize).filter(|n| !n.is_empty()) else {
                break;
            };
            let value = values.first().copied().unwrap_or(0.0);
            let gradient = values.get(1).copied().unwrap_or(0.0);
            self.data
                .add(&set, "DESIGN_RESPONSE", name, name, row, value);
            (self.data).add(&set, "GRADIENT_LENGTH", name, name, row, gradient);
        }
    }

    /// The force, moment, centre, normal and area of a contact pair or a compression only
    /// support, in four rows.
    fn contact_statistics(&mut self, block: &HistoryBlock) {
        let HistorySource::Contact { slave, master } = block.source else {
            self.skip(block);
            return;
        };
        let slave = self.shown(self.sets.surface_name(slave));
        // A compression only support rests on the ground.
        let master = master.map_or("GROUND".to_string(), |m| {
            self.shown(self.sets.surface_name(m))
        });
        let set = key(contact_set_name(&slave, &master), block);
        let rows: Vec<&Vec<f64>> = block.rows.iter().map(|r| &r.1).take(4).collect();
        if rows.len() < 4 || rows[0].len() < 6 || rows[1].len() < 6 || rows[2].len() < 3 {
            return;
        }
        let row = self.data.row(block.step, block.increment, block.time);
        let data = &mut self.data;
        let mut put = |field: &str, names: &[&str], values: &[f64]| {
            for (name, value) in names.iter().zip(values) {
                data.add(&set, field, name, name, row, *value);
            }
        };
        put("TOTAL_SURFACE_FORCE", &["FX", "FY", "FZ"], &rows[0][..3]);
        put("MOMENT_ABOUT_ORIGIN", &["MX", "MY", "MZ"], &rows[0][3..6]);
        put("CENTER_OF_GRAVITY_CG", &["X", "Y", "Z"], &rows[1][..3]);
        put("MEAN_SURFACE_NORMAL", &["NX", "NY", "NZ"], &rows[1][3..6]);
        put("MOMENT_ABOUT_CG", &["MX", "MY", "MZ"], &rows[2][..3]);
        put("SURFACE_AREA", &["A"], &rows[3][..1.min(rows[3].len())]);
        if rows[3].len() >= 3 {
            put(
                "SURFACE_LOADS",
                &["NORMAL_FORCE", "SHEAR_FORCE"],
                &rows[3][1..3],
            );
        }
    }

    /// The values of an increment: by node, element and integration point, contact element
    /// and face, or a sum.
    fn values(&mut self, block: &HistoryBlock) {
        let Some(kind) = kind_of(block.quantity) else {
            self.skip(block);
            return;
        };
        let ids = match block.quantity {
            // Face-to-face contact by slave element and face, node-to-face by node.
            HistoryQuantity::RelativeContactDisplacement { faces }
            | HistoryQuantity::ContactStress { faces }
            | HistoryQuantity::ContactSpringEnergy { faces } => {
                if faces {
                    Ids::IdPoint
                } else {
                    Ids::Id
                }
            }
            _ => kind.ids,
        };
        let set = match &block.source {
            // The contact values of all pairs, printed without a set.
            HistorySource::Step if is_contact(block.quantity) => ALL_CONTACT_ELEMENTS.to_string(),
            HistorySource::Step => {
                // Nothing names the set of the values.
                self.skip(block);
                return;
            }
            source => self.shown(plxs_output::dat::set_name(source, self.sets)),
        };
        if ROTATION_KINDS.iter().any(|k| std::ptr::eq(k, kind))
            && !self.rotation_sets.contains(&set)
        {
            self.rotation_sets.push(set.clone());
        }
        let set = key(set, block);
        let row = self.data.row(block.step, block.increment, block.time);
        // The built-in solver leaves the trailing out-of-plane values of a 2D model out (the
        // z value, the shear stresses xz and yz).
        let least = match kind.components.len() {
            3 => 2,
            6 => 4,
            n => n,
        };
        for (id, values) in &block.rows {
            let (entry, values) = match ids {
                Ids::None => (None, &values[..]),
                Ids::Id => (Some(id.to_string()), &values[..]),
                Ids::IdPoint => {
                    let Some((point, values)) = values.split_first() else {
                        continue;
                    };
                    (Some(format!("{id}_{}", *point as u32)), values)
                }
            };
            if values.len() < least {
                continue;
            }
            for (component, value) in kind.components.iter().zip(values) {
                let value = if block.quantity == HistoryQuantity::ContactElements {
                    value.trunc()
                } else {
                    *value
                };
                let entry = entry.as_deref().unwrap_or(component);
                self.data
                    .add(&set, kind.field, component, entry, row, value);
            }
        }
    }

    fn skip(&mut self, block: &HistoryBlock) {
        let name = format!("{:?}", block.quantity);
        if !self.unknown.contains(&name) {
            self.unknown.push(name);
        }
    }
}

/// The set of the tables of a step.
fn step_set(block: &HistoryBlock) -> SetKey {
    SetKey::Step(block.step)
}

/// The set of the values of `block`, printed for the set named `name`: that of the history
/// output the block was printed for, so that two outputs of the same set or name stay
/// apart.
fn key(name: String, block: &HistoryBlock) -> SetKey {
    match block.origin {
        Some(origin) => SetKey::Output(origin, name),
        None => SetKey::Named(name),
    }
}

fn is_contact(quantity: HistoryQuantity) -> bool {
    use HistoryQuantity as Q;
    matches!(
        quantity,
        Q::ContactElements
            | Q::RelativeContactDisplacement { .. }
            | Q::ContactStress { .. }
            | Q::ContactSpringEnergy { .. }
    )
}

/// The field and components of the values of a quantity.
fn kind_of(quantity: HistoryQuantity) -> Option<&'static Kind> {
    use HistoryQuantity as Q;
    let field = match quantity {
        Q::Displacements => "DISPLACEMENTS",
        Q::Velocities => "VELOCITIES",
        Q::Forces => "FORCES",
        Q::TotalForce => "TOTAL_FORCE",
        Q::Rotations => "ROTATIONS",
        Q::AngularVelocities => "ANGULAR_VELOCITIES",
        Q::Moments => "MOMENTS",
        Q::TotalMoment => "TOTAL_MOMENT",
        Q::Temperatures => "TEMPERATURES",
        Q::HeatGeneration => "HEAT_GENERATION",
        Q::TotalHeatGeneration => "TOTAL_HEAT_GENERATION",
        Q::HeatFlux => "HEAT_FLUX",
        Q::Stresses => "STRESSES",
        Q::Strains => "STRAINS",
        Q::EquivalentPlasticStrain => "EQUIVALENT_PLASTIC_STRAIN",
        Q::InternalEnergyDensity => "INTERNAL_ENERGY_DENSITY",
        Q::Volume => "VOLUME",
        Q::TotalVolume => "TOTAL_VOLUME",
        Q::ModelInternalEnergy => "MODEL_INTERNAL_ENERGY",
        Q::ModelKineticEnergy => "MODEL_KINETIC_ENERGY",
        Q::ModelExternalWork => "MODEL_EXTERNAL_WORK",
        Q::ModelDampingDissipation => "MODEL_DAMPING_DISSIPATION",
        Q::ParticleTemperature => "PARTICLE_TEMPERATURE",
        Q::WallHeat => "WALL_HEAT",
        Q::ContactElements => "TOTAL_NUMBER_OF_CONTACT_ELEMENTS",
        Q::RelativeContactDisplacement { .. } => "RELATIVE_CONTACT_DISPLACEMENT",
        Q::ContactStress { .. } => "CONTACT_STRESS",
        Q::ContactSpringEnergy { .. } => "CONTACT_SPRING_ENERGY",
        _ => return None,
    };
    (KINDS.iter().chain(ROTATION_KINDS)).find(|k| k.field == field)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn block(
        quantity: HistoryQuantity,
        source: HistorySource,
        rows: Vec<(u32, Vec<f64>)>,
    ) -> HistoryBlock {
        HistoryBlock {
            origin: None,
            step: 1,
            increment: 1,
            quantity,
            source,
            time: 1.0,
            rows,
        }
    }

    fn value(import: &DatImport, set: &str, field: &str, component: &str, entry: &str) -> f64 {
        let set = import.sets.iter().find(|s| s.name == set).unwrap();
        let field = set.fields.iter().find(|f| f.name == field).unwrap();
        let component = field.component(component).unwrap();
        let entry = component.entries.iter().find(|e| e.name == entry).unwrap();
        entry.values[0]
    }

    #[test]
    fn node_to_face_contact_is_listed_by_node() {
        let results = Results {
            history: vec![block(
                HistoryQuantity::ContactStress { faces: false },
                HistorySource::Step,
                vec![(101, vec![2.5, 0.1, 0.2])],
            )],
            ..Results::default()
        };
        let import = import_blocks(&results, &Sets::default());
        let press = value(
            &import,
            ALL_CONTACT_ELEMENTS,
            "CONTACT_STRESS",
            "PRESS",
            "101",
        );
        assert_eq!(press, 2.5);
    }

    #[test]
    fn a_set_names_the_history_output_it_was_printed_for() {
        let output = plx_model::Id::new();
        let mut tip = block(
            HistoryQuantity::Displacements,
            HistorySource::Named("TIP".into()),
            vec![(7, vec![1.0, 2.0, 3.0])],
        );
        tip.origin = Some(output.raw());
        let table = block(
            HistoryQuantity::BucklingFactors,
            HistorySource::Step,
            vec![(1, vec![4.5])],
        );
        let results = Results {
            history: vec![tip, table],
            ..Results::default()
        };
        let import = import_blocks(&results, &Sets::default());
        let output_of = |name: &str| import.sets.iter().find(|s| s.name == name).unwrap().output;
        assert_eq!(output_of("TIP"), Some(output));
        assert_eq!(output_of("STEP_1"), None);
    }

    #[test]
    fn two_history_outputs_of_one_set_stay_apart() {
        let (first, second) = (plx_model::Id::new(), plx_model::Id::new());
        let printed = |origin: plx_model::Id, value: f64| {
            let mut b = block(
                HistoryQuantity::Displacements,
                HistorySource::Named("TIP".into()),
                vec![(7, vec![value, 0.0, 0.0])],
            );
            b.origin = Some(origin.raw());
            b
        };
        let results = Results {
            history: vec![printed(first, 1.0), printed(second, 2.0)],
            ..Results::default()
        };
        let import = import_blocks(&results, &Sets::default());
        let of = |id: plx_model::Id| import.sets.iter().find(|s| s.output == Some(id)).unwrap();
        let ux = |set: &plx_results::history_output::HistorySet| {
            set.fields[0].components[0].entries[0].values[0]
        };
        assert_eq!(import.sets.len(), 2);
        assert_eq!((ux(of(first)), ux(of(second))), (1.0, 2.0));
    }

    #[test]
    fn the_tables_of_a_step_are_not_a_set_of_that_name() {
        let mut user = block(
            HistoryQuantity::Displacements,
            HistorySource::Named("STEP_1".into()),
            vec![(7, vec![1.0, 2.0, 3.0])],
        );
        user.origin = Some(plx_model::Id::<plx_model::HistoryOutput>::new().raw());
        let table = block(
            HistoryQuantity::BucklingFactors,
            HistorySource::Step,
            vec![(1, vec![4.5])],
        );
        let results = Results {
            history: vec![user, table],
            ..Results::default()
        };
        let import = import_blocks(&results, &Sets::default());
        assert_eq!(import.sets.len(), 2);
        let tables = import.sets.iter().find(|s| s.step == Some(1)).unwrap();
        assert_eq!(tables.fields[0].name, "BUCKLING_FACTOR_OUTPUT");
    }

    #[test]
    fn particle_temperature_has_no_id_column() {
        let results = Results {
            history: vec![block(
                HistoryQuantity::ParticleTemperature,
                HistorySource::Named("Mix".into()),
                vec![(0, vec![300.0, 350.0])],
            )],
            ..Results::default()
        };
        let import = import_blocks(&results, &Sets::default());
        let field = "PARTICLE_TEMPERATURE";
        assert_eq!(value(&import, "MIX", field, "TMEAN", "TMEAN"), 300.0);
        assert_eq!(value(&import, "MIX", field, "TMAX", "TMAX"), 350.0);
    }
}
