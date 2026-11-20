//! The out-of-plane components of the results of a 2D model (plane stress, plane strain,
//! axisymmetric), which are zero by the model: the third displacement, velocity,
//! acceleration and force, the transverse shear stresses and strains, the thickness stress
//! of plane stress and the thickness strain of plane strain, the second slip of 2D contact.
//! They are left out of the results Nodalis keeps ([`Results::drop_out_of_plane`]) and put
//! back as zeros where a reader needs all components ([`full_components`],
//! [`Results::restore_out_of_plane`]).
//!
//! What is left out is decided by the values, not only by the element types: a component
//! goes only where it is zero in every increment, so the hoop stress and strain of an
//! axisymmetric model and the thickness stress of plane strain stay, and nothing that
//! carries a value (a rotation about the axis normal to the plane) is ever lost.

use super::{FieldQuantity, HistoryBlock, HistoryQuantity, HistorySource, ResultField, Results};
use crate::{ElementFamily, Model};

/// The components of a field in full, in their order, of the fields that have out-of-plane
/// components.
const GROUPS: &[&[&str]] = &[
    &["D1", "D2", "D3"],
    &["V1", "V2", "V3"],
    &["A1", "A2", "A3"],
    &["F1", "F2", "F3"],
    &["SXX", "SYY", "SZZ", "SXY", "SYZ", "SZX"],
    &["EXX", "EYY", "EZZ", "EXY", "EYZ", "EZX"],
    &["MEXX", "MEYY", "MEZZ", "MEXY", "MEYZ", "MEZX"],
    &["MAG1", "MAG2", "MAG3", "PHA1", "PHA2", "PHA3"],
    &[
        "MAGXX", "MAGYY", "MAGZZ", "MAGXY", "MAGYZ", "MAGZX", "PHAXX", "PHAYY", "PHAZZ", "PHAXY",
        "PHAYZ", "PHAZX",
    ],
    &["COPEN", "CSLIP1", "CSLIP2", "CPRESS", "CSHEAR1", "CSHEAR2"],
];

/// Whether a component may be out of the plane of a 2D model (in the x-y plane, with y the
/// axis of an axisymmetric one): the z components, and the second slip direction of 2D
/// contact. Whether it is left out depends on its values.
fn out_of_plane(component: &str) -> bool {
    matches!(
        component,
        "D3" | "V3"
            | "A3"
            | "F3"
            | "SZZ"
            | "SYZ"
            | "SZX"
            | "EZZ"
            | "EYZ"
            | "EZX"
            | "MEZZ"
            | "MEYZ"
            | "MEZX"
            | "MAG3"
            | "PHA3"
            | "MAGZZ"
            | "MAGYZ"
            | "MAGZX"
            | "PHAZZ"
            | "PHAYZ"
            | "PHAZX"
            | "CSLIP2"
            | "CSHEAR2"
    )
}

/// The components of a field in full, when some of them were left out as out-of-plane
/// components of a 2D model: the field's components are a part of them, in the same
/// order. `None` when the field has all of its components, or is none of these fields.
pub fn full_components(components: &[String]) -> Option<&'static [&'static str]> {
    let group = GROUPS
        .iter()
        .find(|g| !components.is_empty() && components.iter().all(|c| g.contains(&c.as_str())))?;
    // In the order of the group, every left-out component one that may be.
    let mut at = 0;
    for c in components {
        let k = group.iter().position(|g| g == c)?;
        if k < at || group[at..k].iter().any(|g| !out_of_plane(g)) {
            return None;
        }
        at = k + 1;
    }
    if group[at..].iter().any(|g| !out_of_plane(g)) {
        return None;
    }
    (components.len() < group.len()).then_some(*group)
}

/// Whether every element of the model is a plane stress, plane strain or axisymmetric
/// element, or one that goes with them (springs, gaps, trusses), and there is one of them.
pub fn is_plane(model: &Model) -> bool {
    let mut plane = false;
    for element in &model.elements {
        match element.kind.family() {
            ElementFamily::PlaneStress
            | ElementFamily::PlaneStrain
            | ElementFamily::Axisymmetric => plane = true,
            ElementFamily::Spring | ElementFamily::Gap | ElementFamily::Truss => {}
            _ => return false,
        }
    }
    plane
}

/// Whether the values are zero, against the largest value of their field: what the solver
/// computes as zero may come out as round-off.
fn zero(values: &[f64], scale: f64) -> bool {
    let tiny = 1e-12 * scale;
    values.iter().all(|v| v.abs() <= tiny)
}

fn largest(values: &[f64]) -> f64 {
    values
        .iter()
        .fold(0.0, |m, v| if v.is_finite() { m.max(v.abs()) } else { m })
}

/// History blocks of vector and tensor quantities and how many of their last values are
/// out-of-plane components (the z value; the shear stresses or strains xz, yz). The
/// rotations and moments of a reference point have none: about z is the one a 2D model has.
fn trailing_out_of_plane(block: &HistoryBlock) -> Option<usize> {
    use HistoryQuantity as Q;
    match block.quantity {
        Q::Displacements | Q::Velocities | Q::Forces | Q::TotalForce | Q::HeatFlux => Some(1),
        Q::Stresses | Q::Strains => Some(2),
        _ => None,
    }
}

/// The number of values of a full row of a history block of [`trailing_out_of_plane`]:
/// the integration point first for element values.
fn full_row(block: &HistoryBlock) -> Option<usize> {
    use HistoryQuantity as Q;
    let (point, values) = match block.quantity {
        Q::Stresses | Q::Strains => (1, 6),
        Q::HeatFlux => (1, 3),
        _ => (0, 3),
    };
    trailing_out_of_plane(block).map(|_| point + values)
}

/// Which out-of-plane components of the fields of a 2D model are zero in every increment,
/// gathered a field at a time ([`OutOfPlane::add`]) for results that are not all in memory
/// at once, and left out of each field ([`OutOfPlane::drop`]) once all are seen.
#[derive(Clone, Debug, Default)]
pub struct OutOfPlane {
    /// By field name: the largest finite value of any component, and of the out-of-plane
    /// components every field of the name has, the largest magnitude (infinite where one
    /// is not finite).
    fields: Vec<((FieldQuantity, bool), f64, Vec<Magnitude>)>,
}

/// A component and its largest magnitude.
type Magnitude = (String, f64);

impl OutOfPlane {
    /// Takes the values of `field` into account.
    pub fn add(&mut self, field: &ResultField) {
        let scale = field.values.iter().map(|v| largest(v)).fold(0.0, f64::max);
        let magnitudes = (field.components.iter().zip(&field.values))
            .filter(|(c, _)| out_of_plane(c))
            .map(|(c, v)| (c.clone(), magnitude(v)));
        match self.fields.iter_mut().find(|(n, _, _)| *n == field.key()) {
            Some((_, s, kept)) => {
                *s = s.max(scale);
                let magnitudes: Vec<(String, f64)> = magnitudes.collect();
                kept.retain_mut(|(c, m)| match magnitudes.iter().find(|(n, _)| n == c) {
                    Some((_, v)) => {
                        *m = m.max(*v);
                        true
                    }
                    None => false,
                });
            }
            None => self.fields.push((field.key(), scale, magnitudes.collect())),
        }
    }

    /// The components of fields of `key` that are zero in every one seen.
    fn zeros(&self, key: (FieldQuantity, bool)) -> Vec<String> {
        let Some((_, scale, kept)) = self.fields.iter().find(|(n, _, _)| *n == key) else {
            return Vec::new();
        };
        let tiny = 1e-12 * scale;
        (kept.iter())
            .filter(|(_, m)| *m <= tiny)
            .map(|(c, _)| c.clone())
            .collect()
    }

    /// The components `field` keeps: without those zero in every field of its name, where
    /// a reader can put them back ([`full_components`]); `None` if it keeps all.
    pub fn kept(&self, field: &ResultField) -> Option<Vec<bool>> {
        let zeros = self.zeros(field.key());
        if zeros.is_empty() {
            return None;
        }
        let keep: Vec<bool> = field
            .components
            .iter()
            .map(|c| !zeros.contains(c))
            .collect();
        let kept: Vec<String> = (field.components.iter().zip(&keep))
            .filter(|(_, k)| **k)
            .map(|(c, _)| c.clone())
            .collect();
        // Only what a reader can put back in full is left out; a field of another layout
        // keeps its zeros.
        (keep.contains(&false) && full_components(&kept).is_some()).then_some(keep)
    }

    /// Leaves out of `field` the components [`OutOfPlane::kept`] does not keep.
    pub fn drop(&self, field: &mut ResultField) {
        let Some(keep) = self.kept(field) else {
            return;
        };
        let mut keep_iter = keep.iter();
        field
            .components
            .retain(|_| *keep_iter.next().unwrap_or(&true));
        let mut keep_iter = keep.iter();
        field.values.retain(|_| *keep_iter.next().unwrap_or(&true));
    }
}

/// The largest magnitude of `values`, infinite where one is not finite: zero only where
/// all values are.
fn magnitude(values: &[f64]) -> f64 {
    values.iter().fold(0.0, |m, v| {
        if v.is_nan() {
            f64::INFINITY
        } else {
            m.max(v.abs())
        }
    })
}

impl Results {
    /// Leaves out the out-of-plane components of the results of a 2D model: of the fields
    /// those zero in every increment (see the module), of the history output the trailing
    /// out-of-plane values of the vector and tensor quantities, where zero in every block
    /// of the quantity and set. The mode shapes keep their three components. Results of
    /// any other model stay as they are.
    pub fn drop_out_of_plane(&mut self, model: &Model) {
        if !is_plane(model) {
            return;
        }
        // Fields by name: the components zero in every increment.
        let mut zeros = OutOfPlane::default();
        for field in self.increments.iter().flat_map(|i| &i.fields) {
            zeros.add(field);
        }
        for field in self.increments.iter_mut().flat_map(|i| &mut i.fields) {
            zeros.drop(field);
        }

        // History: per quantity and set, how many trailing values are zero everywhere.
        let key = |b: &HistoryBlock| (b.quantity, b.source.clone());
        let mut trailing: Vec<((HistoryQuantity, HistorySource), usize)> = Vec::new();
        for block in &self.history {
            let Some(count) = trailing_out_of_plane(block) else {
                continue;
            };
            let scale = (block.rows.iter())
                .map(|(_, v)| largest(v))
                .fold(0.0, f64::max);
            let zero_tail = block
                .rows
                .iter()
                .all(|(_, v)| v.len() >= count && zero(&v[v.len() - count..], scale));
            let count = if zero_tail { count } else { 0 };
            match trailing.iter_mut().find(|(k, _)| *k == key(block)) {
                Some((_, c)) => *c = (*c).min(count),
                None => trailing.push((key(block), count)),
            }
        }
        for block in &mut self.history {
            let Some(&(_, count)) = trailing.iter().find(|(k, _)| *k == key(block)) else {
                continue;
            };
            if count > 0 {
                for (_, values) in &mut block.rows {
                    values.truncate(values.len() - count);
                }
            }
        }
    }

    /// Whether [`Results::drop_out_of_plane`] left components out.
    pub fn has_out_of_plane_dropped(&self) -> bool {
        let fields = self.increments.iter().flat_map(|i| &i.fields);
        let history = self.history.iter();
        fields
            .into_iter()
            .any(|f| full_components(&f.components).is_some())
            || history
                .into_iter()
                .any(|b| full_row(b).is_some_and(|n| b.rows.iter().any(|(_, v)| v.len() < n)))
    }

    /// Puts the components [`Results::drop_out_of_plane`] left out back as zeros, as the
    /// files of CalculiX have them.
    pub fn restore_out_of_plane(&mut self) {
        for field in self.increments.iter_mut().flat_map(|i| &mut i.fields) {
            if let Some(full) = full_components(&field.components) {
                let missing: Vec<String> = (full.iter())
                    .filter(|c| !field.components.iter().any(|f| f == *c))
                    .map(|c| c.to_string())
                    .collect();
                *field = restore(std::mem::replace(field, empty()), &missing);
            }
        }
        for block in &mut self.history {
            if let Some(full) = full_row(block) {
                for (_, values) in &mut block.rows {
                    if values.len() < full {
                        values.resize(full, 0.0);
                    }
                }
            }
        }
    }
}

fn empty() -> ResultField {
    ResultField {
        quantity: FieldQuantity::Displacements,
        imaginary: false,
        components: Vec::new(),
        nodes: Vec::new(),
        values: Vec::new(),
    }
}

/// The field with the components `missing` as zeros, in the order of its group.
fn restore(mut field: ResultField, missing: &[String]) -> ResultField {
    if missing.is_empty() {
        return field;
    }
    let mut names = field.components.clone();
    names.extend(missing.iter().cloned());
    let Some(group) = GROUPS
        .iter()
        .find(|g| names.iter().all(|c| g.contains(&c.as_str())))
    else {
        return field;
    };
    let n = field.nodes.len();
    let mut components = Vec::new();
    let mut values = Vec::new();
    for &c in group.iter() {
        if let Some(k) = field.components.iter().position(|f| f == c) {
            components.push(c.to_string());
            values.push(std::mem::take(&mut field.values[k]));
        } else if missing.iter().any(|m| m == c) {
            components.push(c.to_string());
            values.push(vec![0.0; n]);
        }
    }
    field.components = components;
    field.values = values;
    field
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::results::{IncrementKind, ResultsIncrement};

    fn names(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn full_components_of_fields_without_their_out_of_plane_ones() {
        assert_eq!(
            full_components(&names(&["D1", "D2"])),
            Some(&["D1", "D2", "D3"][..])
        );
        let stress = names(&["SXX", "SYY", "SXY"]);
        assert_eq!(full_components(&stress).map(|g| g.len()), Some(6));
        // Complete fields, in-plane components missing, other fields.
        assert_eq!(full_components(&names(&["D1", "D2", "D3"])), None);
        assert_eq!(full_components(&names(&["D1", "D3"])), None);
        assert_eq!(full_components(&names(&["SXX", "SZZ", "SXY"])), None);
        assert_eq!(full_components(&names(&["T"])), None);
        assert_eq!(full_components(&names(&["UR1", "UR2"])), None);
    }

    fn plane_model(kind: crate::ElementType) -> Model {
        let mut model = Model::default();
        model.elements.push(crate::Element {
            id: 1,
            kind,
            nodes: vec![1, 2, 3],
            section: None,
        });
        model
    }

    fn field(quantity: FieldQuantity, components: &[&str], values: &[f64]) -> ResultField {
        ResultField {
            quantity,
            imaginary: false,
            components: names(components),
            nodes: vec![1, 2],
            values: values.iter().map(|&v| vec![v, -v]).collect(),
        }
    }

    fn results(fields: Vec<Vec<ResultField>>) -> Results {
        Results {
            increments: (fields.into_iter().enumerate())
                .map(|(i, fields)| ResultsIncrement {
                    step: 1,
                    increment: i as u32 + 1,
                    kind: IncrementKind::Time,
                    value: 1.0,
                    total_time: 1.0,
                    nodal_diameter: None,
                    fields,
                })
                .collect(),
            ..Results::default()
        }
    }

    const STRESS: [&str; 6] = ["SXX", "SYY", "SZZ", "SXY", "SYZ", "SZX"];

    #[test]
    fn a_plane_model_drops_the_components_zero_in_every_increment() {
        let disp = |z| {
            field(
                FieldQuantity::Displacements,
                &["D1", "D2", "D3"],
                &[1.0, 2.0, z],
            )
        };
        // Hoop stress zero in the first increment only: it stays.
        let stress = |zz| {
            field(
                FieldQuantity::Stresses,
                &STRESS,
                &[1.0, 2.0, zz, 3.0, 1e-15, 0.0],
            )
        };
        let mut r = results(vec![
            vec![disp(0.0), stress(0.0)],
            vec![disp(0.0), stress(5.0)],
        ]);
        let original = r.clone();
        r.drop_out_of_plane(&plane_model(crate::ElementType::CAX3));
        for increment in &r.increments {
            assert_eq!(increment.fields[0].components, names(&["D1", "D2"]));
            assert_eq!(increment.fields[0].values.len(), 2);
            assert_eq!(
                increment.fields[1].components,
                names(&["SXX", "SYY", "SZZ", "SXY"])
            );
        }
        let mut back = r.clone();
        back.restore_out_of_plane();
        let mut expected = original.clone();
        // The round-off shear comes back as zero.
        for increment in &mut expected.increments {
            increment.fields[1].values[4] = vec![0.0; 2];
        }
        assert_eq!(back, expected);

        // A third displacement that is not zero, and models that are not 2D, keep it all.
        let mut r = results(vec![vec![disp(0.5)]]);
        r.drop_out_of_plane(&plane_model(crate::ElementType::CPS3));
        assert_eq!(r.increments[0].fields[0].components.len(), 3);
        let mut r = results(vec![vec![disp(0.0)]]);
        r.drop_out_of_plane(&plane_model(crate::ElementType::C3D4));
        assert_eq!(r.increments[0].fields[0].components.len(), 3);
    }

    #[test]
    fn a_plane_model_drops_trailing_zeros_of_the_history() {
        let block = |quantity, rows: Vec<(u32, Vec<f64>)>| HistoryBlock {
            origin: None,
            step: 1,
            increment: 1,
            quantity,
            source: HistorySource::Step,
            time: 1.0,
            rows,
        };
        let mut r = Results {
            history: vec![
                block(
                    HistoryQuantity::Displacements,
                    vec![(4, vec![1.0, 2.0, 0.0])],
                ),
                block(
                    HistoryQuantity::Stresses,
                    vec![(1, vec![1.0, 5.0, 6.0, 0.0, 7.0, 0.0, 0.0])],
                ),
                // Rotations of a reference point: about z is the one there is.
                block(HistoryQuantity::Rotations, vec![(9, vec![0.0, 0.0, 0.3])]),
            ],
            ..Results::default()
        };
        let original = r.clone();
        r.drop_out_of_plane(&plane_model(crate::ElementType::CPE3));
        assert_eq!(r.history[0].rows[0].1, [1.0, 2.0]);
        assert_eq!(r.history[1].rows[0].1, [1.0, 5.0, 6.0, 0.0, 7.0]);
        assert_eq!(r.history[2].rows[0].1, [0.0, 0.0, 0.3]);
        r.restore_out_of_plane();
        assert_eq!(r, original);
    }
}
