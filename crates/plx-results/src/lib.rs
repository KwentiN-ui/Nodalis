//! Ergebnis-Datenmodell und abgeleitete Größen.
//!
//! Results are stored per increment as nodal fields. Every component holds one value per node
//! of the result mesh, in the order of [`plx_mesh::FeMesh::coords`]; nodes without a value are
//! `NaN`.

mod derived;
pub mod display;
pub use plx_model::equation;
pub mod field_output;
pub mod harmonic;
pub mod history_output;
pub mod hot_spot;
pub mod names;
pub mod path;
pub mod principal;
pub mod saved_view;
pub mod solver;
pub mod steps;
pub mod transformation;

pub use derived::{add_derived_components, principal_values, split_rotations};
pub use equation::Equation;
pub use field_output::{FieldOutput, FieldOutputKind, LimitBasis};
pub use principal::{Principal, VectorSource};
pub use solver::{SolverResults, from_solver};
pub use transformation::{Transformation, TransformationKind};

use serde::{Deserialize, Serialize};

/// What the user defined on a results file, saved with results bound to a project so that
/// they open as they were left: derived outputs, transformations, hot spots, the features
/// and paths and planes of the results, and what was shown.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ResultsSettings {
    pub field_outputs: Vec<FieldOutput>,
    pub history_outputs: Vec<history_output::HistoryOutput>,
    pub transformations: Vec<Transformation>,
    pub hot_spots: Vec<hot_spot::HotSpot>,
    pub reference_points: Vec<plx_model::ReferencePoint>,
    pub coordinate_systems: Vec<plx_model::CoordinateSystem>,
    pub planes: Vec<plx_model::Plane>,
    pub result_paths: Vec<plx_model::ResultPath>,
    pub result_planes: Vec<plx_model::ResultPlane>,
    /// Parts shown without colour contours, by name: results of another run may have other
    /// parts, or the same ones in another order.
    pub plain_part_names: Vec<String>,
    /// The same by index, as projects saved before the names were kept have it; only
    /// read, and only applied to results of the same mesh.
    #[serde(skip_serializing)]
    pub plain_parts: Vec<usize>,
    /// The increment shown, by index.
    pub increment: usize,
    /// The field and component shown, by name.
    pub shown: Option<(String, String)>,
    /// Camera, annotations and display settings of the 3D view.
    pub view: saved_view::SavedView,
    /// [`plx_mesh::FeMesh::fingerprint`] of the results' mesh, to tell whether what refers
    /// to its nodes and elements still fits results of another run.
    pub mesh: Option<u64>,
}

impl ResultsSettings {
    /// Drops what refers to nodes, elements or parts of the mesh, for results of a changed
    /// mesh: query annotations except bounding boxes and hot spots on picked nodes or faces.
    /// Reference points at picked nodes or faces stay where they were. Returns what was
    /// dropped or changed.
    pub fn without_mesh_references(&mut self) -> Vec<String> {
        use plx_model::{PointDefinition, Region};
        let picked = |region: &Region| matches!(region, Region::Nodes(_) | Region::Faces(_));
        let mut changes = Vec::new();
        let annotations = self.view.annotations.len();
        (self.view.annotations).retain(|a| matches!(a, saved_view::SavedAnnotation::BoundingBox));
        let dropped = annotations - self.view.annotations.len();
        if dropped > 0 {
            changes.push(format!(
                "{dropped} annotation(s) on nodes or elements dropped"
            ));
        }
        self.hot_spots.retain(|h| {
            let keep = !picked(&h.toe);
            if !keep {
                changes.push(format!("Hot spot {} on picked nodes dropped", h.name));
            }
            keep
        });
        for point in &mut self.reference_points {
            if let PointDefinition::CenterOfGravity(region)
            | PointDefinition::BoundingBoxCenter(region) = &point.definition
                && picked(region)
            {
                point.definition = PointDefinition::Global;
                changes.push(format!(
                    "Reference Point {} kept at its last position",
                    point.name
                ));
            }
        }
        changes
    }
}

/// Kind of analysis an increment belongs to, as CalculiX reports it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnalysisKind {
    Static,
    Frequency,
    /// Complex eigenmodes of a rotating structure (`*COMPLEX FREQUENCY`); the value is the
    /// real part of the frequency, the mode shape whirls with the magnitudes and phases of
    /// the `PDISP` field.
    ComplexFrequency,
    Buckling,
    Dynamic,
    /// Derivatives of design responses with respect to the design node positions
    /// (`*SENSITIVITY`): fields `NORM` and `SEN...`. For eigenfrequencies there is one
    /// increment per mode with its frequency as the value.
    Sensitivity,
    Other(i32),
}

impl AnalysisKind {
    /// Label of the increment value (time, eigenfrequency, buckling factor).
    pub fn value_label(self) -> &'static str {
        match self {
            AnalysisKind::Frequency | AnalysisKind::ComplexFrequency => "Frequency",
            AnalysisKind::Buckling => "Load factor",
            AnalysisKind::Sensitivity => "Value",
            _ => "Time",
        }
    }

    /// Whether the increments are eigenmodes, whose shapes are scaled arbitrarily and swing
    /// about the undeformed shape.
    pub fn is_modal(self) -> bool {
        matches!(
            self,
            AnalysisKind::Frequency | AnalysisKind::ComplexFrequency | AnalysisKind::Buckling
        )
    }

    /// Whether the increments are eigenmodes with a frequency.
    pub fn is_frequency(self) -> bool {
        matches!(
            self,
            AnalysisKind::Frequency | AnalysisKind::ComplexFrequency
        )
    }
}

/// One component of a field, e.g. `D1` of `DISP` or `MISES` of `STRESS`.
#[derive(Clone, Debug, PartialEq)]
pub struct Component {
    pub name: String,
    pub values: Vec<f32>,
    /// Computed by nodalis rather than read from the file.
    pub derived: bool,
    /// An out-of-plane component of a 2D model, zero everywhere: the built-in solver leaves
    /// it out of the results, and it is filled in with zeros for what computes with all
    /// components of the field (magnitudes, invariants, transformations), but not listed.
    pub out_of_plane: bool,
}

impl Component {
    /// A magnitude or equivalent value that never turns negative (`ALL`, `MISES`, `TRESCA`,
    /// `EQUIVALENT`): it keeps its sign when the result is scaled by a negative factor.
    pub fn is_invariant(&self) -> bool {
        self.derived
            && matches!(
                self.name.as_str(),
                "ALL" | "MISES" | "TRESCA" | "EQUIVALENT"
            )
    }

    /// Smallest and largest finite value, if any.
    pub fn range(&self) -> Option<(f32, f32)> {
        self.values
            .iter()
            .copied()
            .filter(|v| v.is_finite())
            .fold(None, |range, v| match range {
                None => Some((v, v)),
                Some((min, max)) => Some((min.min(v), max.max(v))),
            })
    }
}

/// What the raw (not derived) components of a field form.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FieldRank {
    /// Every component stands for itself.
    Scalar,
    /// The first three raw components are x, y and z of a vector; its magnitude is `ALL`.
    Vector,
    /// Six raw components of a symmetric tensor, ordered xx, yy, zz, xy, yz, zx.
    Tensor(TensorKind),
}

/// Which equivalent values a tensor field gets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TensorKind {
    /// Von Mises and Tresca equivalents.
    Stress,
    /// The equivalent strain, 2/3 of the von Mises expression.
    Strain,
}

impl FieldRank {
    /// The rank of a field CalculiX wrote: by the field name and the
    /// number of raw components.
    pub fn of_calculix(name: &str, raw_components: usize) -> Self {
        match (name, raw_components) {
            (
                "DISP" | "MODE" | "VELO" | "ACCE" | "FORC" | "FLUX" | "NORM" | "ROTATION"
                | "MOMENT",
                3,
            ) => FieldRank::Vector,
            ("STRESS" | "ZZSTR", 6) => FieldRank::Tensor(TensorKind::Stress),
            ("TOSTRAIN" | "MESTRAIN", 6) => FieldRank::Tensor(TensorKind::Strain),
            _ => FieldRank::Scalar,
        }
    }

    pub fn is_tensor(self) -> bool {
        matches!(self, FieldRank::Tensor(_))
    }
}

/// A nodal result field such as `DISP` or `STRESS`.
#[derive(Clone, Debug, PartialEq)]
pub struct Field {
    pub name: String,
    pub components: Vec<Component>,
    /// Whether the raw components form a vector or a tensor; fields the user derives keep
    /// the rank of what they compute, whatever their name.
    pub rank: FieldRank,
}

impl Field {
    /// A field CalculiX wrote, ranked by its name and raw components.
    pub fn new(name: impl Into<String>, components: Vec<Component>) -> Self {
        let name = name.into();
        let raw = components.iter().filter(|c| !c.derived).count();
        let rank = FieldRank::of_calculix(&name, raw);
        Self {
            name,
            components,
            rank,
        }
    }

    /// The values of the raw components, in their order.
    pub fn raw_values(&self) -> Vec<&[f32]> {
        (self.components.iter())
            .filter(|c| !c.derived)
            .map(|c| c.values.as_slice())
            .collect()
    }

    pub fn component(&self, name: &str) -> Option<&Component> {
        self.components.iter().find(|c| c.name == name)
    }
}

/// All fields written for one increment of one step.
#[derive(Clone, Debug, PartialEq)]
pub struct Increment {
    pub step: u32,
    pub increment: u32,
    pub kind: AnalysisKind,
    /// Time, eigenfrequency or buckling factor, depending on [`Increment::kind`].
    pub value: f64,
    pub fields: Vec<Field>,
    /// Nodal diameter of a mode of a cyclic symmetric model.
    pub nodal_diameter: Option<u32>,
}

impl Increment {
    pub fn field(&self, name: &str) -> Option<&Field> {
        self.fields.iter().find(|f| f.name == name)
    }

    /// The field that moves the nodes: the displacements (`DISP`), or the shape of a mode
    /// of the built-in solver ([`solver::MODE`]).
    pub fn deformation_field(&self) -> Option<&Field> {
        self.field("DISP").or_else(|| self.field(solver::MODE))
    }

    /// Whether the increment is a mode as the built-in solver keeps it: its shape only,
    /// normalized to a largest displacement of 1.
    pub fn is_mode_shape(&self) -> bool {
        self.field(solver::MODE).is_some()
    }

    /// Nodal displacements, if the increment has a `DISP` field (or a mode shape) with three
    /// components.
    pub fn displacements(&self) -> Option<Vec<[f32; 3]>> {
        let field = self.deformation_field()?;
        let [x, y, z] = ["U1", "U2", "U3"].map(|n| field.component(n));
        let (x, y, z) = (x?, y?, z?);
        Some(
            x.values
                .iter()
                .zip(&y.values)
                .zip(&z.values)
                .map(|((&x, &y), &z)| [x, y, z].map(|v| if v.is_finite() { v } else { 0.0 }))
                .collect(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::saved_view::SavedAnnotation;
    use plx_model::{PointDefinition, ReferencePoint, Region};

    #[test]
    fn a_changed_mesh_drops_what_refers_to_its_nodes() {
        let mut on_faces = ReferencePoint::new("RP-1", [1.0, 2.0, 3.0]);
        on_faces.definition = PointDefinition::CenterOfGravity(Region::Faces(vec![(7, 2)]));
        let mut by_set = super::hot_spot::HotSpot::new("Hot_Spot-2");
        by_set.toe = Region::NodeSet("TOE".into());
        let mut settings = super::ResultsSettings {
            hot_spots: vec![super::hot_spot::HotSpot::new("Hot_Spot-1"), by_set],
            reference_points: vec![on_faces],
            ..Default::default()
        };
        settings.view.annotations = vec![
            SavedAnnotation::Nodes(vec![3]),
            SavedAnnotation::BoundingBox,
        ];
        assert_eq!(settings.without_mesh_references().len(), 3);
        assert_eq!(settings.view.annotations, [SavedAnnotation::BoundingBox]);
        assert_eq!(settings.hot_spots.len(), 1);
        assert_eq!(settings.hot_spots[0].name, "Hot_Spot-2");
        let point = &settings.reference_points[0];
        assert_eq!(point.definition, PointDefinition::Global);
        assert_eq!(point.position, [1.0, 2.0, 3.0]);
    }
}
