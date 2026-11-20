//! Field outputs of an analysis: the results its steps write for every node, which the
//! Results show as contours.
//!
//! A step always writes the results that belong to its kind ([`StepKind::standard_results`]
//! and what the model adds, see [`FeModel::field_results`]): they are no items of the model
//! and nothing the user has to ask for. The model keeps one [`FieldOutput`] with what goes
//! beyond them. What the solver is asked for is made from both when the analysis is
//! exported.

use serde::{Deserialize, Serialize};

use crate::{FeModel, Step, StepKind};

/// A result a step writes for the nodes of the model.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum FieldResult {
    Displacements,
    ReactionForces,
    Velocities,
    Accelerations,
    Stresses,
    Strains,
    /// The equivalent plastic strain, which is the equivalent creep strain of a creep law.
    PlasticStrain,
    Temperatures,
    ReactionHeatFlows,
    HeatFlux,
    /// Gap and slip, pressure and shear of the contacts.
    Contact,
    Sensitivities,
    StrainEnergy,
    StressError,
    ContactEnergy,
    /// Displacements and stresses of a harmonic response as amplitude and phase, next to
    /// their real and imaginary parts.
    AmplitudePhase,
}

impl FieldResult {
    /// The results the user can add to those the steps always write, in the order of the
    /// dialog.
    pub const ADDITIONAL: [FieldResult; 4] = [
        Self::StrainEnergy,
        Self::StressError,
        Self::ContactEnergy,
        Self::AmplitudePhase,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Displacements => "Displacements",
            Self::ReactionForces => "Reaction forces",
            Self::Velocities => "Velocities",
            Self::Accelerations => "Accelerations",
            Self::Stresses => "Stresses",
            Self::Strains => "Strains",
            Self::PlasticStrain => "Equivalent plastic strain",
            Self::Temperatures => "Temperatures",
            Self::ReactionHeatFlows => "Reaction heat flows",
            Self::HeatFlux => "Heat flux",
            Self::Contact => "Contact gap and pressure",
            Self::Sensitivities => "Design sensitivities",
            Self::StrainEnergy => "Strain energy density",
            Self::StressError => "Stress error estimate",
            Self::ContactEnergy => "Contact energy",
            Self::AmplitudePhase => "Amplitude and phase",
        }
    }

    /// What the result is, for the hover text of the dialog.
    pub fn description(self) -> &'static str {
        match self {
            Self::Displacements => "Displacements of the nodes.",
            Self::ReactionForces => "Forces of the supports on the nodes they hold.",
            Self::Velocities => "Velocities of the nodes.",
            Self::Accelerations => "Accelerations of the nodes.",
            Self::Stresses => "Stresses, averaged at the nodes.",
            Self::Strains => "Total strains, averaged at the nodes.",
            Self::PlasticStrain => {
                "Equivalent plastic strain; the equivalent creep strain of a creep law."
            }
            Self::Temperatures => "Temperatures of the nodes.",
            Self::ReactionHeatFlows => "Heat flows into the nodes of prescribed temperature.",
            Self::HeatFlux => "Heat flux, averaged at the nodes.",
            Self::Contact => "Gap and slip, pressure and shear of the contacts.",
            Self::Sensitivities => "Sensitivities of the design responses.",
            Self::StrainEnergy => "Strain energy per volume, averaged at the nodes.",
            Self::StressError => {
                "How much the stresses of the elements at a node differ, in percent: large \
                 where the mesh is too coarse for the stress gradient."
            }
            Self::ContactEnergy => "Energy stored in the contacts.",
            Self::AmplitudePhase => {
                "Displacements and stresses of a harmonic response as amplitude and phase, \
                 next to their real and imaginary parts."
            }
        }
    }

    /// The names of the results as the Results list them.
    pub fn fields(self) -> &'static [&'static str] {
        match self {
            Self::Displacements => &["DISP"],
            Self::ReactionForces => &["FORC"],
            Self::Velocities => &["VELO"],
            Self::Accelerations => &["ACCE"],
            Self::Stresses => &["STRESS"],
            Self::Strains => &["TOSTRAIN"],
            Self::PlasticStrain => &["PE"],
            Self::Temperatures => &["NDTEMP"],
            Self::ReactionHeatFlows => &["RFL"],
            Self::HeatFlux => &["FLUX"],
            Self::Contact => &["CONTACT"],
            Self::Sensitivities => &["SEN"],
            Self::StrainEnergy => &["ENER"],
            Self::StressError => &["ERROR"],
            Self::ContactEnergy => &["CELS"],
            Self::AmplitudePhase => &["PDISP", "PSTRESS"],
        }
    }
}

impl StepKind {
    /// The results a step of the kind always writes. An eigenvalue step keeps its mode
    /// shapes instead, a particle flow step its particles.
    pub fn standard_results(&self) -> &'static [FieldResult] {
        use FieldResult::*;
        match self {
            StepKind::Static(_) | StepKind::Visco(_) | StepKind::SteadyStateDynamics(_) => {
                &[Displacements, ReactionForces, Stresses, Strains]
            }
            StepKind::Dynamic(_) => &[
                Displacements,
                ReactionForces,
                Velocities,
                Accelerations,
                Stresses,
                Strains,
            ],
            StepKind::HeatTransfer(_) => &[Temperatures, ReactionHeatFlows, HeatFlux],
            StepKind::CoupledTempDisp(_) | StepKind::UncoupledTempDisp(_) => &[
                Displacements,
                ReactionForces,
                Temperatures,
                ReactionHeatFlows,
                Stresses,
                Strains,
                HeatFlux,
            ],
            StepKind::Sensitivity(_) => &[Sensitivities],
            StepKind::Frequency(_)
            | StepKind::ComplexFrequency(_)
            | StepKind::Buckle(_)
            | StepKind::ParticleFlow(_) => &[],
        }
    }

    /// Whether a step of the kind deforms the model under loads, over one or more
    /// increments: such a step closes contacts and yields materials.
    fn deforms_under_loads(&self) -> bool {
        matches!(
            self,
            StepKind::Static(_)
                | StepKind::Visco(_)
                | StepKind::Dynamic(_)
                | StepKind::CoupledTempDisp(_)
                | StepKind::UncoupledTempDisp(_)
        )
    }

    /// Whether a step of the kind can write the additional result.
    pub fn writes_additional(&self, result: FieldResult) -> bool {
        match result {
            FieldResult::StrainEnergy | FieldResult::StressError => {
                self.standard_results().contains(&FieldResult::Stresses)
            }
            FieldResult::ContactEnergy => self.takes_contact_results(),
            FieldResult::AmplitudePhase => matches!(self, StepKind::SteadyStateDynamics(_)),
            _ => false,
        }
    }

    /// Whether a step of the kind writes the results of contacts.
    fn takes_contact_results(&self) -> bool {
        matches!(
            self,
            StepKind::Static(_) | StepKind::Visco(_) | StepKind::Dynamic(_)
        )
    }
}

/// What the steps of an analysis write beyond their standard results, and how the results
/// are written and shown. One for the whole analysis: each step takes from it what it can
/// compute.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FieldOutput {
    /// Results of [`FieldResult::ADDITIONAL`], in their order.
    pub additional: Vec<FieldResult>,
    /// The coordinate system the vector and tensor results are also shown in, next to the
    /// global ones; `None` for the global system only. The solver writes global results;
    /// the transformation is made when the results are shown, so that it follows a moving
    /// system.
    pub system: Option<crate::Id<crate::CoordinateSystem>>,
    /// How shells and beams are written.
    pub dimension: OutputDimension,
}

/// The form shells and beams have in the results.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum OutputDimension {
    /// Shells and beams as the solids the solver expands them to; plane models in 2D.
    #[default]
    Default,
    /// On the mid-surface and the axis.
    TwoD,
    /// All 1D and 2D elements in their expanded 3D form.
    ThreeD,
}

impl OutputDimension {
    pub const ALL: [OutputDimension; 3] = [Self::Default, Self::TwoD, Self::ThreeD];

    pub fn label(self) -> &'static str {
        match self {
            Self::Default => "Default",
            Self::TwoD => "On the mid-surface (2D)",
            Self::ThreeD => "On the expanded solid (3D)",
        }
    }
}

impl FieldOutput {
    /// Whether nothing goes beyond the standard results: the model then shows no field
    /// output at all.
    pub fn is_standard(&self) -> bool {
        *self == Self::default()
    }

    /// Sorts the additional results into their fixed order and drops what is none.
    pub fn normalize(&mut self) {
        let order = |r: &FieldResult| FieldResult::ADDITIONAL.iter().position(|a| a == r);
        self.additional.retain(|r| order(r).is_some());
        self.additional.sort_by_key(order);
        self.additional.dedup();
    }
}

impl FeModel {
    /// Whether the model has contacts or compression only supports that are active.
    fn has_contact(&self) -> bool {
        self.contact_pairs.iter().any(|p| p.active)
            || self.contact_regions.iter().any(|r| r.active)
            || (self.constraints.iter())
                .any(|c| matches!(c, crate::Constraint::CompressionOnly(s) if s.active))
    }

    /// Whether a material of the model yields or creeps.
    fn has_inelastic_material(&self) -> bool {
        (self.materials.iter()).any(|m| m.plastic.is_some() || m.creep.is_some())
    }

    /// Whether a step of the model prescribes temperatures.
    fn has_prescribed_temperatures(&self) -> bool {
        (self.steps.iter())
            .filter(|s| s.active)
            .any(|s| s.defined_fields.iter().any(|f| f.active))
    }

    /// The results the model adds to the standard ones of every step that deforms under
    /// loads: the plastic strain where a material yields or creeps, the temperatures where
    /// steps prescribe them, those of contacts where there are any.
    pub fn model_results(&self) -> Vec<FieldResult> {
        let mut results = Vec::new();
        if self.has_inelastic_material() {
            results.push(FieldResult::PlasticStrain);
        }
        if self.has_prescribed_temperatures() {
            results.push(FieldResult::Temperatures);
        }
        if self.has_contact() {
            results.push(FieldResult::Contact);
        }
        results
    }

    /// All results `step` writes: the standard ones of its kind, those the model adds and
    /// the additional ones of the field output it can compute.
    pub fn field_results(&self, step: &Step) -> Vec<FieldResult> {
        let kind = &step.kind;
        let mut results = kind.standard_results().to_vec();
        for result in self.model_results() {
            let takes = match result {
                FieldResult::Contact => kind.takes_contact_results(),
                _ => kind.deforms_under_loads(),
            };
            if takes && !results.contains(&result) {
                results.push(result);
            }
        }
        results.extend(
            (self.field_output.additional.iter())
                .copied()
                .filter(|&r| kind.writes_additional(r)),
        );
        results
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steps_write_the_results_of_their_kind_without_being_asked() {
        let mut model = FeModel::default();
        let step = Step::new_static("Step-1");
        assert!(model.field_output.is_standard());
        assert_eq!(
            model.field_results(&step),
            [
                FieldResult::Displacements,
                FieldResult::ReactionForces,
                FieldResult::Stresses,
                FieldResult::Strains
            ]
        );
        let dynamic = model.field_results(&Step::new_dynamic("Step-2"));
        assert!(dynamic.contains(&FieldResult::Velocities));
        // An eigenvalue step keeps its mode shapes only.
        assert!(
            model
                .field_results(&Step::new_frequency("Step-3"))
                .is_empty()
        );
        let heat = model.field_results(&Step::new_heat_transfer("Step-4"));
        assert_eq!(heat[0], FieldResult::Temperatures);
        assert!(!heat.contains(&FieldResult::Stresses));
        // The additional results reach the steps that can compute them only.
        model.field_output.additional = vec![
            FieldResult::AmplitudePhase,
            FieldResult::Displacements,
            FieldResult::StrainEnergy,
        ];
        model.field_output.normalize();
        assert_eq!(
            model.field_output.additional,
            [FieldResult::StrainEnergy, FieldResult::AmplitudePhase]
        );
        let results = model.field_results(&step);
        assert!(results.contains(&FieldResult::StrainEnergy));
        assert!(!results.contains(&FieldResult::AmplitudePhase));
        let heat = model.field_results(&Step::new_heat_transfer("Step-4"));
        assert!(!heat.contains(&FieldResult::StrainEnergy));
    }

    #[test]
    fn yielding_materials_add_the_plastic_strain() {
        let mut model = FeModel::default();
        let step = Step::new_static("Step-1");
        assert!(model.model_results().is_empty());
        model.materials.push(crate::Material {
            plastic: Some(crate::Plastic::default()),
            ..Default::default()
        });
        assert_eq!(model.model_results(), [FieldResult::PlasticStrain]);
        assert!(
            model
                .field_results(&step)
                .contains(&FieldResult::PlasticStrain)
        );
        let heat = model.field_results(&Step::new_heat_transfer("Step-2"));
        assert!(!heat.contains(&FieldResult::PlasticStrain));
    }
}
