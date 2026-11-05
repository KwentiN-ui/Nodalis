//! Results of a run in memory: what the `.frd` and `.dat` writers serialize and what
//! Nodalis will read directly in phase 2.

use serde::{Deserialize, Serialize};

use crate::mesh::NodeId;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Results {
    /// One entry per written increment (or mode).
    pub increments: Vec<ResultsIncrement>,
    /// History rows (`.dat`), in the order they are written.
    pub history: Vec<HistoryBlock>,
    /// The eigenmodes of the eigenvalue steps (frequency, buckling, complex frequency): one
    /// set per step, shapes only. The `.frd` fields of these steps (displacements,
    /// stresses... of every mode as CalculiX writes them) are in `increments` only where
    /// the step's output requests ask for them.
    pub modes: Vec<ModeSet>,
}

impl Results {
    /// Every field of the increments and every mode shape as a `DISP` field (with `DISPI`
    /// for its imaginary part), for what changes all nodal fields alike: the fields of the
    /// modes are written back by [`Results::set_mode_fields`].
    pub fn mode_fields(&self) -> Vec<ResultField> {
        (self.modes.iter())
            .flat_map(|set| set.modes.iter().flat_map(|m| m.fields(&set.nodes)))
            .collect()
    }

    /// Takes the mode shapes back from fields in the order [`Results::mode_fields`] gave
    /// them (on other nodes, perhaps), normalized again.
    pub fn set_mode_fields(&mut self, fields: Vec<ResultField>) {
        let mut fields = fields.into_iter();
        for set in &mut self.modes {
            for mode in &mut set.modes {
                let Some(real) = fields.next() else { return };
                set.nodes = real.nodes.clone();
                mode.shape = ModeShape::rows(&real);
                if mode.imaginary.is_some() {
                    mode.imaginary = fields.next().map(|f| ModeShape::rows(&f));
                }
                mode.normalize();
            }
        }
    }
}

/// The kind of an eigenvalue step.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ModeKind {
    /// Natural frequencies (`*FREQUENCY`), also of a cyclic symmetric sector.
    Frequency,
    /// Buckling load factors (`*BUCKLE`).
    Buckling,
    /// Complex eigenmodes of a rotating structure (`*COMPLEX FREQUENCY`).
    ComplexFrequency,
}

/// The eigenmodes of one eigenvalue step: their shapes on the written nodes, each stored
/// once and normalized, with the eigenvalue. Mode shapes are scaled arbitrarily, so their
/// stresses, strains and forces tell nothing and are not kept.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ModeSet {
    pub step: u32,
    pub kind: ModeKind,
    /// The nodes of the shapes, in the order of their rows.
    pub nodes: Vec<NodeId>,
    pub modes: Vec<ModeShape>,
}

/// One eigenmode.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ModeShape {
    /// The number of the mode in its step, from 1, as the `.dat` tables count it.
    pub number: u32,
    /// The eigenvalue: ω² of a frequency mode, the load factor of a buckling mode, the
    /// angular frequency μ of a complex mode.
    pub eigenvalue: f64,
    /// The frequency in Hz (frequency and complex modes) or the load factor (buckling).
    pub value: f64,
    /// Nodal diameter of a mode of a cyclic symmetric sector.
    pub nodal_diameter: Option<u32>,
    /// Displacements x, y, z per node, scaled so that the largest displacement (over a
    /// turn, for a mode with an imaginary part) is 1.
    pub shape: Vec<[f32; 3]>,
    /// The imaginary part: of a complex mode, which whirls, and of a cyclic symmetric
    /// mode, the shape a quarter period later that turns it into the other sectors.
    pub imaginary: Option<Vec<[f32; 3]>>,
}

impl ModeShape {
    /// A mode from its displacements in rows of `x, y, z`, normalized.
    pub fn new(
        number: u32,
        eigenvalue: f64,
        value: f64,
        shape: Vec<[f32; 3]>,
        imaginary: Option<Vec<[f32; 3]>>,
    ) -> Self {
        let mut mode = Self {
            number,
            eigenvalue,
            value,
            nodal_diameter: None,
            shape,
            imaginary,
        };
        mode.normalize();
        mode
    }

    /// The rows of the first three components of a field.
    pub fn rows(field: &ResultField) -> Vec<[f32; 3]> {
        let at = |k: usize, n: usize| field.values.get(k).map_or(0.0, |c| c[n] as f32);
        (0..field.nodes.len())
            .map(|n| [at(0, n), at(1, n), at(2, n)])
            .collect()
    }

    /// Scales the mode so that its largest displacement is 1; with an imaginary part, the
    /// largest of `|Re|² + |Im|²`, which bounds the displacement over a turn.
    pub fn normalize(&mut self) {
        let size = |u: &[f32; 3]| u.iter().map(|v| v * v).sum::<f32>();
        let largest = match &self.imaginary {
            Some(im) => (self.shape.iter().zip(im))
                .map(|(a, b)| size(a) + size(b))
                .fold(0.0f32, f32::max),
            None => self.shape.iter().map(size).fold(0.0f32, f32::max),
        }
        .sqrt();
        if largest > 0.0 && largest.is_finite() {
            for row in self
                .shape
                .iter_mut()
                .chain(self.imaginary.iter_mut().flatten())
            {
                row.iter_mut().for_each(|v| *v /= largest);
            }
        }
    }

    /// The shape as a `DISP` field on `nodes`, and the imaginary part as `DISPI`.
    pub fn fields(&self, nodes: &[NodeId]) -> Vec<ResultField> {
        let field = |name: &str, components: [&str; 3], rows: &[[f32; 3]]| ResultField {
            name: name.into(),
            components: components.map(String::from).to_vec(),
            nodes: nodes.to_vec(),
            values: (0..3)
                .map(|k| rows.iter().map(|r| f64::from(r[k])).collect())
                .collect(),
        };
        let mut fields = vec![field("DISP", ["D1", "D2", "D3"], &self.shape)];
        if let Some(im) = &self.imaginary {
            fields.push(field("DISPI", ["D1", "D2", "D3"], im));
        }
        fields
    }
}

/// The kind of increment, as the `.frd` header tells it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum IncrementKind {
    /// A time increment: `value` is the total time (what CalculiX writes into the `.frd`).
    Time,
    /// A time increment of a transient step (transient heat transfer, dynamics): `value` is
    /// the step time; the `.frd` marks it as analysis type 1.
    Dynamic,
    /// A mode: `value` is the eigenfrequency (Hz) or, for `*BUCKLE`, the load factor.
    Frequency,
    Buckling,
    /// A complex mode: `value` is the real part of the frequency.
    ComplexFrequency,
    /// A frequency point of a steady state dynamics step: `value` is the excitation
    /// frequency (Hz); the `.frd` marks it as analysis type 1 with increment 0.
    SteadyState,
    /// A data set of a `*SENSITIVITY` step (the `.frd` marks it as analysis type 3 without
    /// a mode): the surface normals (`value` 0) or the sensitivities of one design response
    /// (`value` the eigenfrequency for `EIGENFREQUENCY`, 0 otherwise).
    Sensitivity,
}

/// Fields written for one increment of one step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResultsIncrement {
    pub step: u32,
    pub increment: u32,
    pub kind: IncrementKind,
    pub value: f64,
    /// Total time at the increment (time increments only).
    pub total_time: f64,
    /// Nodal diameter of a cyclic symmetric mode.
    pub nodal_diameter: Option<u32>,
    pub fields: Vec<ResultField>,
}

/// A nodal result field as written to the `.frd`: a CalculiX field name (`DISP`, `STRESS`,
/// `FORC`...), component names (`D1`, `SXX`...) and the values per component, in the order
/// of `nodes` (the output mesh: expanded nodes of shells and beams included, hence an own
/// id list rather than the model's dense order).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResultField {
    pub name: String,
    pub components: Vec<String>,
    pub nodes: Vec<NodeId>,
    /// `values[component][node]`.
    pub values: Vec<Vec<f64>>,
}

/// A block of the `.dat` file: a quantity for a set at a time, one row per node, element or
/// integration point.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HistoryBlock {
    pub step: u32,
    pub increment: u32,
    /// The heading as CalculiX writes it (`displacements (vx,vy,vz) for set TIP and time ...`
    /// without the time), which `plx-io::dat` recognises.
    pub quantity: String,
    pub set: String,
    pub time: f64,
    /// Rows: the id (node, element, or 0 for totals) and the values.
    pub rows: Vec<(u32, Vec<f64>)>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mode_is_normalized_to_its_largest_displacement() {
        let mode = ModeShape::new(1, 4.0, 2.0, vec![[3.0, 4.0, 0.0], [0.0, -10.0, 0.0]], None);
        assert_eq!(mode.shape, [[0.3, 0.4, 0.0], [0.0, -1.0, 0.0]]);
        // A complex mode by the largest |Re|² + |Im|², its size over a turn.
        let mode = ModeShape::new(
            1,
            1.0,
            1.0,
            vec![[3.0, 0.0, 0.0]],
            Some(vec![[0.0, 4.0, 0.0]]),
        );
        assert_eq!(mode.shape, [[0.6, 0.0, 0.0]]);
        assert_eq!(mode.imaginary, Some(vec![[0.0, 0.8, 0.0]]));
    }

    #[test]
    fn mode_fields_go_back_onto_other_nodes() {
        let mut results = Results {
            modes: vec![ModeSet {
                step: 2,
                kind: ModeKind::Frequency,
                nodes: vec![1, 2, 3],
                modes: vec![ModeShape::new(1, 1.0, 0.2, vec![[0.0, 0.0, 1.0]; 3], None)],
            }],
            ..Default::default()
        };
        let mut fields = results.mode_fields();
        assert_eq!(fields.len(), 1);
        assert_eq!(fields[0].name, "DISP");
        // The written mesh drops node 2 and scales the shape.
        fields[0].nodes = vec![1, 3];
        fields[0].values = vec![vec![0.0, 0.0], vec![0.0, 0.0], vec![2.0, 4.0]];
        results.set_mode_fields(fields);
        let set = &results.modes[0];
        assert_eq!(set.nodes, [1, 3]);
        assert_eq!(set.modes[0].shape, [[0.0, 0.0, 0.5], [0.0, 0.0, 1.0]]);
    }
}
