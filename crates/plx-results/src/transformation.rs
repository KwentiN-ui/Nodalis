//! The result transformations: mirrored and patterned copies of the results, e.g. to
//! show the full model of a half model. They only change what is drawn, not the results.
//!
//! The copies are chained: every transformation copies everything drawn so far,
//! including the copies of the transformations before it. Directional values are rotated or
//! mirrored with the copy, so that a mirrored displacement component changes its sign.

use glam::{DAffine3, DMat3, DVec3};
use serde::{Deserialize, Serialize};

use crate::{Field, FieldRank};

/// Plane of a symmetry, named by the axis normal to it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SymmetryPlane {
    X,
    Y,
    Z,
}

impl SymmetryPlane {
    pub const ALL: [SymmetryPlane; 3] = [SymmetryPlane::X, SymmetryPlane::Y, SymmetryPlane::Z];

    pub fn axis(self) -> usize {
        self as usize
    }

    pub fn label(self) -> &'static str {
        ["X", "Y", "Z"][self.axis()]
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum TransformationKind {
    /// Mirror at the plane normal to the axis through the point: one copy.
    Symmetry {
        plane: SymmetryPlane,
        point: [f64; 3],
    },
    /// `count` items in all, each shifted from the one before by `end - start`.
    LinearPattern {
        start: [f64; 3],
        end: [f64; 3],
        count: u32,
    },
    /// `count` items in all, each turned from the one before by `angle` degrees about the axis
    /// from `axis_start` to `axis_end` (right-hand rule).
    CircularPattern {
        axis_start: [f64; 3],
        axis_end: [f64; 3],
        angle: f64,
        count: u32,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Transformation {
    pub name: String,
    pub kind: TransformationKind,
}

impl Transformation {
    /// The defaults: planes through the origin, a pattern along x and one about y,
    /// the axis of CalculiX's axisymmetric models.
    pub fn symmetry(plane: SymmetryPlane) -> Self {
        Self {
            name: format!("Symmetry-{}", plane.label()),
            kind: TransformationKind::Symmetry {
                plane,
                point: [0.0; 3],
            },
        }
    }

    pub fn linear_pattern() -> Self {
        Self {
            name: "Linear".into(),
            kind: TransformationKind::LinearPattern {
                start: [0.0; 3],
                end: [1.0, 0.0, 0.0],
                count: 2,
            },
        }
    }

    pub fn circular_pattern() -> Self {
        Self {
            name: "Circular".into(),
            kind: TransformationKind::CircularPattern {
                axis_start: [0.0; 3],
                axis_end: [0.0, 1.0, 0.0],
                angle: 45.0,
                count: 2,
            },
        }
    }

    /// Number of items including the original, at least two.
    pub fn items(&self) -> u32 {
        match self.kind {
            TransformationKind::Symmetry { .. } => 2,
            TransformationKind::LinearPattern { count, .. }
            | TransformationKind::CircularPattern { count, .. } => count.max(2),
        }
    }

    /// The transformation from one item to the next.
    pub fn step(&self) -> DAffine3 {
        match self.kind {
            TransformationKind::Symmetry { plane, point } => {
                let mut scale = DVec3::ONE;
                scale[plane.axis()] = -1.0;
                let point = DVec3::from(point);
                DAffine3::from_translation(point)
                    * DAffine3::from_scale(scale)
                    * DAffine3::from_translation(-point)
            }
            TransformationKind::LinearPattern { start, end, .. } => {
                DAffine3::from_translation(DVec3::from(end) - DVec3::from(start))
            }
            TransformationKind::CircularPattern {
                axis_start,
                axis_end,
                angle,
                ..
            } => {
                let point = DVec3::from(axis_start);
                let Some(axis) = (DVec3::from(axis_end) - point).try_normalize() else {
                    return DAffine3::IDENTITY;
                };
                DAffine3::from_translation(point)
                    * DAffine3::from_axis_angle(axis, angle.to_radians())
                    * DAffine3::from_translation(-point)
            }
        }
    }

    /// The checks when the transformations are applied.
    pub fn check(&self) -> Result<(), String> {
        let error = |text: &str| Err(format!("{}: {text}", self.name));
        match self.kind {
            TransformationKind::Symmetry { .. } => Ok(()),
            TransformationKind::LinearPattern { start, end, .. } => {
                if DVec3::from(end) == DVec3::from(start) {
                    return error("The offset of the pattern must be greater than 0.");
                }
                Ok(())
            }
            TransformationKind::CircularPattern {
                axis_start,
                axis_end,
                angle,
                ..
            } => {
                if angle == 0.0 || !angle.is_finite() {
                    return error("The angle of the pattern must not be 0.");
                }
                if DVec3::from(axis_end) == DVec3::from(axis_start) {
                    return error("The points of the axis coincide.");
                }
                Ok(())
            }
        }
    }
}

/// Every item drawn: the original (identity) first, then the copies, chained.
///
pub fn instances(transformations: &[Transformation]) -> Vec<DAffine3> {
    let mut instances = vec![DAffine3::IDENTITY];
    for transformation in transformations {
        let step = transformation.step();
        for index in 0..instances.len() {
            let mut current = instances[index];
            for _ in 1..transformation.items() {
                current = step * current;
                instances.push(current);
            }
        }
    }
    instances
}

/// How a component changes with the copy it is drawn on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ComponentKind {
    /// Magnitudes, equivalent and principal values and everything else: unchanged.
    Scalar,
    /// The given row of a vector.
    Vector(usize),
    /// The given (row, column) of a symmetric tensor.
    Tensor(usize, usize),
}

/// How the component at `index` of the field transforms, judged by the field's rank and the
/// component's place among the raw (not derived) components.
pub fn component_kind(field: &Field, index: usize) -> ComponentKind {
    let Some(component) = field.components.get(index) else {
        return ComponentKind::Scalar;
    };
    if component.derived {
        return ComponentKind::Scalar;
    }
    let raw: Vec<usize> = (0..field.components.len())
        .filter(|&i| !field.components[i].derived)
        .collect();
    let Some(position) = raw.iter().position(|&i| i == index) else {
        return ComponentKind::Scalar;
    };
    match field.rank {
        FieldRank::Vector if raw.len() == 3 => ComponentKind::Vector(position),
        FieldRank::Tensor(_) if raw.len() == 6 => {
            let (row, column) = [(0, 0), (1, 1), (2, 2), (0, 1), (1, 2), (2, 0)][position];
            ComponentKind::Tensor(row, column)
        }
        _ => ComponentKind::Scalar,
    }
}

/// Values of a component as drawn on a copy rotated or mirrored by `q`; `None` when they are
/// those of the original.
pub fn transformed_values(field: &Field, index: usize, q: DMat3) -> Option<Vec<f32>> {
    if q == DMat3::IDENTITY {
        return None;
    }
    let raw: Vec<&[f32]> = (field.components.iter())
        .filter(|c| !c.derived)
        .map(|c| c.values.as_slice())
        .collect();
    // Row-major access to q: q.col(j)[i] is the element in row i, column j.
    let at = |row: usize, column: usize| q.col(column)[row];
    match component_kind(field, index) {
        ComponentKind::Scalar => None,
        ComponentKind::Vector(row) => {
            let weights = [at(row, 0), at(row, 1), at(row, 2)];
            Some(
                (0..raw[0].len())
                    .map(|n| (0..3).map(|k| weights[k] * raw[k][n] as f64).sum::<f64>() as f32)
                    .collect(),
            )
        }
        ComponentKind::Tensor(row, column) => {
            // T' = Q T Q^T, components ordered xx, yy, zz, xy, yz, zx.
            const SLOT: [[usize; 3]; 3] = [[0, 3, 5], [3, 1, 4], [5, 4, 2]];
            Some(
                (0..raw[0].len())
                    .map(|n| {
                        let mut value = 0.0;
                        for i in 0..3 {
                            for j in 0..3 {
                                value += at(row, i) * raw[SLOT[i][j]][n] as f64 * at(column, j);
                            }
                        }
                        value as f32
                    })
                    .collect(),
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Component;

    fn field(name: &str, components: &[(&str, f32)]) -> Field {
        let mut field = Field::new(
            name,
            components
                .iter()
                .map(|&(name, value)| Component {
                    name: name.into(),
                    values: vec![value],
                    derived: false,
                })
                .collect(),
        );
        crate::add_derived_components(&mut field);
        field
    }

    fn index(field: &Field, name: &str) -> usize {
        field
            .components
            .iter()
            .position(|c| c.name == name)
            .unwrap()
    }

    #[test]
    fn symmetry_mirrors_at_the_plane_through_the_point() {
        let mirror = Transformation {
            name: "S".into(),
            kind: TransformationKind::Symmetry {
                plane: SymmetryPlane::X,
                point: [2.0, 0.0, 0.0],
            },
        };
        let all = instances(&[mirror]);
        assert_eq!(all.len(), 2);
        let p = all[1].transform_point3(DVec3::new(3.0, 1.0, 1.0));
        assert!((p - DVec3::new(1.0, 1.0, 1.0)).length() < 1e-12);
    }

    #[test]
    fn copies_are_chained() {
        // A quarter mirrored at x and then at y gives four items; a circular pattern of
        // three after it twelve.
        let mut list = vec![
            Transformation::symmetry(SymmetryPlane::X),
            Transformation::symmetry(SymmetryPlane::Y),
        ];
        let all = instances(&list);
        assert_eq!(all.len(), 4);
        let p = DVec3::new(1.0, 2.0, 3.0);
        let images: Vec<DVec3> = all.iter().map(|a| a.transform_point3(p)).collect();
        assert!(images.contains(&DVec3::new(-1.0, -2.0, 3.0)));
        list.push(Transformation {
            name: "C".into(),
            kind: TransformationKind::CircularPattern {
                axis_start: [0.0; 3],
                axis_end: [0.0, 0.0, 1.0],
                angle: 90.0,
                count: 3,
            },
        });
        assert_eq!(instances(&list).len(), 12);
    }

    #[test]
    fn circular_pattern_turns_about_the_axis() {
        let pattern = Transformation {
            name: "C".into(),
            kind: TransformationKind::CircularPattern {
                axis_start: [1.0, 0.0, 0.0],
                axis_end: [1.0, 5.0, 0.0],
                angle: 90.0,
                count: 4,
            },
        };
        let all = instances(&[pattern]);
        assert_eq!(all.len(), 4);
        // About +y, x turns into -z.
        let p = all[1].transform_point3(DVec3::new(2.0, 3.0, 0.0));
        assert!((p - DVec3::new(1.0, 3.0, -1.0)).length() < 1e-12, "{p}");
    }

    #[test]
    fn mirrored_vectors_flip_the_normal_component_only() {
        let disp = field("DISP", &[("U1", 1.0), ("U2", 2.0), ("U3", 3.0)]);
        let q = Transformation::symmetry(SymmetryPlane::X).step().matrix3;
        let u1 = transformed_values(&disp, index(&disp, "U1"), q);
        assert_eq!(u1, Some(vec![-1.0]));
        assert_eq!(
            transformed_values(&disp, index(&disp, "U2"), q),
            Some(vec![2.0])
        );
        // The magnitude stays.
        assert_eq!(transformed_values(&disp, index(&disp, "ALL"), q), None);
    }

    #[test]
    fn tensors_rotate_with_the_copy() {
        let stress = field(
            "STRESS",
            &[
                ("SXX", 10.0),
                ("SYY", 0.0),
                ("SZZ", 0.0),
                ("SXY", 3.0),
                ("SYZ", 4.0),
                ("SZX", 5.0),
            ],
        );
        let mirror = Transformation::symmetry(SymmetryPlane::X).step().matrix3;
        let values = |name: &str, q| transformed_values(&stress, index(&stress, name), q);
        // Mirroring at x flips the shear components with one x index.
        assert_eq!(values("SXX", mirror), Some(vec![10.0]));
        assert_eq!(values("SXY", mirror), Some(vec![-3.0]));
        assert_eq!(values("SYZ", mirror), Some(vec![4.0]));
        assert_eq!(values("SZX", mirror), Some(vec![-5.0]));
        assert_eq!(values("MISES", mirror), None);
        // A quarter turn about z swaps xx and yy.
        let turn = DMat3::from_rotation_z(std::f64::consts::FRAC_PI_2);
        let syy = values("SYY", turn).unwrap()[0];
        assert!((syy - 10.0).abs() < 1e-4, "{syy}");
    }

    #[test]
    fn other_fields_and_bad_patterns() {
        let temperature = field("NDTEMP", &[("T", 20.0)]);
        let q = Transformation::symmetry(SymmetryPlane::Z).step().matrix3;
        assert_eq!(transformed_values(&temperature, 0, q), None);
        let mut pattern = Transformation::circular_pattern();
        assert!(pattern.check().is_ok());
        if let TransformationKind::CircularPattern { angle, .. } = &mut pattern.kind {
            *angle = 0.0;
        }
        assert!(pattern.check().is_err());
        let mut linear = Transformation::linear_pattern();
        if let TransformationKind::LinearPattern { end, .. } = &mut linear.kind {
            *end = [0.0; 3];
        }
        assert!(linear.check().is_err());
    }
}
