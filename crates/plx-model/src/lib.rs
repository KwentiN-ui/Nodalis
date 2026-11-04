//! FE-Modell: Materialien, Sections, Steps, Randbedingungen, Lasten.
//!
//! The model describes an analysis on top of a mesh. Regions
//! are what the user picked in the GUI (parts, nodes, element faces) or named sets of the
//! input file; node and element sets that the solver needs for them are derived when the input
//! file is written, so the user never has to define sets by hand.

mod amplitude;
mod bolt;
mod cad_operation;
pub mod campbell;
mod checks;
pub mod color_scale;
mod connection;
mod constraint;
mod contact;
mod contact_check;
mod contact_steps;
pub mod convert;
mod cross_section;
mod detection;
mod distribution;
mod element_type;
pub mod equation;
mod exposed;
mod features;
mod field;
pub mod fingerprint;
mod geometry;
mod history;
pub mod library;
mod measure;
mod mesh_set;
mod moving_load;
mod number;
mod parameter;
mod primitive;
mod properties;
mod region;
mod remote;
mod section;
mod signature;
mod solver_diagnostics;
mod study;
mod symmetry;
pub mod units;
mod validity;

pub use amplitude::{Amplitude, AmplitudeTime};
pub use bolt::{Bolt, BoltAction, BoltChange, LoweredBolts, Preload, ResolvedBolt};
pub use cad_operation::{CadOperation, CadOperationKind};
pub use checks::{
    Finding, MeshCheck, Problem, Severity, diagnose_solver_output, diagnose_solver_warnings,
};
pub use color_scale::{ColorScale, ColorSpectrum};
pub use connection::{
    BondedMethod, CONDUCTANCE_FACTOR, ContactBehavior, ContactDefaults, ContactRegion,
    ContactSettings, InterfaceTreatment, Lowered, ROUGH_FRICTION, Resolved, ResolvedContacts,
    ResolvedKind, STIFFNESS_FACTOR, SideRoles, Stiffness, TIED_STIFFNESS_FACTOR, TOLERANCE_FACTOR,
    WEAK_SPRING_FACTOR, conducts_across_contacts, mortar_supports,
};
pub use constraint::{CompressionOnly, NodeTie, RigidBody, Spring};
pub use contact::{
    Constraint, ContactMethod, ContactPair, ContactScale, CyclicSymmetry, DEFAULT_SURFACE_COLOR,
    ElementYoung, Friction, GapConductance, InteractionProperty, NORMAL_STIFFNESS_FACTOR,
    STICK_SLOPE_RATIO, SectorDisplay, SurfaceBehavior, SurfaceInteraction, Tie,
    default_stick_slope,
};
pub use contact_check::{ContactCheck, ContactStatus};
pub use contact_steps::{ContactChange, ContactState};
pub use cross_section::{
    BeamProfile, CrossSection, Dimension, Outline, ROUND_POINTS, SectionProperties, Sketch,
    triangulate, triangulate_with_holes,
};
pub use detection::{
    ContactDetection, DetectionFilter, DetectionGeometry, DetectionGrouping, DetectionReport,
    auto_detection_tolerance, auto_geometry_detection_tolerance,
};
pub use distribution::{Distribution, DistributionKind, Interpolation, Sampler, parse_points};
pub use element_type::{PartElementType, element_type_options};
pub use exposed::{Exposed, Field, ItemRef, ValuePath};
pub use features::{
    CoordinatePlane, CoordinateSystem, CoordinateSystemKind, GLOBAL, Plane, PlaneSource,
    PointDefinition, PointRef, ReferencePoint, ResultPath, ResultPlane,
};
pub use field::{FieldOutput, OutputDimension, OutputKind, output_variable_description};
pub use geometry::{
    Algorithm2d, Algorithm3d, ExtrudeLayers, ExtrudeMesh, Geometry, MeshSetupItem, MeshSetupKind,
    MeshingParameters, Recombination,
};
pub use history::{HistoryKind, HistoryOutput, Totals};
pub use library::MaterialLibrary;
pub use measure::{Extreme, MeasureKind, MeasureRegion, POINT_AXES};
pub use mesh_set::{MeshSet, SetKind};
pub use moving_load::{AXIS_LABELS, LoadStations, MovingFrame, MovingLoad, Wheel, load_stations};
pub use number::{MathOp, switch};
pub use parameter::{Parameter, ValueKind};
pub use primitive::{MAX_PRIMITIVE_ELEMENTS, MeshPrimitive, Param, ParamSlot, ParamValue};
pub use properties::{ModelKind, ModelProperties, ModelSpace};
pub use region::{Region, describe_entities};
pub use remote::{
    Behavior, Displacement, Formulation, RemoteBehavior, RemoteDof, RemotePoint, expanded_nodes,
    rigid_rows,
};
pub use section::{
    BeamOrientation, BeamSection, Section, SectionKind, line_tangent, membrane_type, unit_thickness,
};
pub use signature::{EntitySignature, TagMap, match_entities};
pub use solver_diagnostics::{Origins, diagnose_solver_check};
pub use study::{DesignPoint, FillMode, FillRule, PointRun, PointState, StudyColumn, StudyTable};
pub use symmetry::{SymmetryError, SymmetryPlane, symmetry_dofs, symmetry_plane};
pub use units::{BASE_QUANTITIES, DERIVED_QUANTITIES, Quantity, STORED, UnitSystem};
pub use validity::{Invalid, ModelItem};

use std::path::{Path, PathBuf};

use plx_mesh::{CadEntity, NodeId};
use serde::{Deserialize, Serialize};

/// Version of the project file format written by this build.
/// Format 2 holds several analyses; projects of format 1 had one model and are not read.
/// Format 3 stores all values in SI units ([`STORED`]) and gives each analysis its own model
/// space and unit system. Format 4 adds state links: an analysis that continues the steps of
/// another one. Format 5 holds blocks of several kinds, the analyses among them, in place of
/// the analyses alone. Format 6 lets the results of a step go to an input of a step of the
/// setup (`["Setup", "Steps", step, "Temperatures"]`) in place of a step of the results.
/// Format 7 drops the fixed boundary condition, a displacement holding all degrees of freedom
/// at zero takes its place. Format 8 drops the concentrated force and the surface traction for
/// the force, which acts on faces, edges or points and optionally at each node. Format 9 gives
/// numbers a physical quantity, format 10 replaces the optimizer by the parameter study.
/// Format 11 adds notes, frames and compact blocks to the schematic, format 12 the Math and
/// Switch blocks; formats 10 and 11 read as they are. Format 13 records the signatures of
/// the geometry's entities, to find them again in changed shapes; format 12 reads as it is,
/// without them. Format 14 drops the user keywords of the keyword editor; older formats read
/// without them. Format 15 merges the remote displacement into the displacement, which
/// chooses how it is written from its selection; older formats are refused. Format 16 gives
/// the equation solver of a step only the choice Direct or Iterative; older formats are
/// refused.
pub const PROJECT_FORMAT: u32 = 16;

/// Everything besides the mesh that makes up an analysis.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FeModel {
    /// Model space and unit system; projects saved before they existed are 3D.
    #[serde(default)]
    pub properties: ModelProperties,
    pub materials: Vec<Material>,
    pub sections: Vec<Section>,
    /// Profiles of beams, which beam sections pick by name.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub cross_sections: Vec<CrossSection>,
    /// Springs, supports and ties.
    #[serde(default)]
    pub constraints: Vec<Constraint>,
    #[serde(default)]
    pub surface_interactions: Vec<SurfaceInteraction>,
    #[serde(default)]
    pub contact_pairs: Vec<ContactPair>,
    /// Surfaces glued to each other (`*TIE`), shown among the contact pairs.
    #[serde(default)]
    pub ties: Vec<Tie>,
    /// Ends of beams and trusses tied node to node, shown among the contact pairs.
    #[serde(default)]
    pub node_ties: Vec<NodeTie>,
    /// Surfaces in contact by the behavior the user wants, written as the ties and contact
    /// pairs they stand for ([`FeModel::lower_contacts`]).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub contact_regions: Vec<ContactRegion>,
    /// Searches that found contact regions, kept to run them again.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub contact_detections: Vec<ContactDetection>,
    /// Settings shared by the contact regions.
    #[serde(default)]
    pub contact_defaults: ContactDefaults,
    /// Bolts preloaded across a cut through their shank, written as the pre-tension loads
    /// and ties they stand for ([`FeModel::lower_bolts`]).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bolts: Vec<Bolt>,
    /// Time curves boundary conditions and loads refer to by name.
    #[serde(default)]
    pub amplitudes: Vec<Amplitude>,
    /// Factors over the model that pressures and temperatures refer to by name.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub distributions: Vec<Distribution>,
    /// State of the model before the first step, such as its initial temperature.
    #[serde(default)]
    pub initial_conditions: Vec<InitialCondition>,
    pub steps: Vec<Step>,
    /// The features: points and coordinate systems other items refer to by name.
    #[serde(default)]
    pub reference_points: Vec<ReferencePoint>,
    #[serde(default)]
    pub coordinate_systems: Vec<CoordinateSystem>,
    #[serde(default)]
    pub planes: Vec<Plane>,
    /// Results on planes; not part of the input file.
    #[serde(default)]
    pub result_planes: Vec<ResultPlane>,
    /// Straight paths results are read on; not part of the input file.
    #[serde(default)]
    pub result_paths: Vec<ResultPath>,
    /// Node sets, element sets and surfaces the user defined by a selection; their members
    /// are written into the mesh's sets.
    #[serde(default)]
    pub mesh_sets: Vec<MeshSet>,
    /// Element types the user chose for parts instead of those the mesh came with.
    #[serde(default)]
    pub element_types: Vec<PartElementType>,
    /// Fields exposed as parameters, which value links of the project schematic set.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub exposed: Vec<Exposed>,
}

impl FeModel {
    /// The first active tie with a cyclic symmetry; Frequency steps then compute the modes
    /// by nodal diameter.
    pub fn cyclic_tie(&self) -> Option<&Tie> {
        (self.ties.iter()).find(|t| t.active && t.cyclic.is_some())
    }

    /// Every region of the model: of sections, constraints, contact pairs, boundary
    /// conditions and loads.
    pub fn regions(&self) -> impl Iterator<Item = &Region> {
        (self.sections.iter().map(|s| &s.region))
            .chain(self.constraints.iter().flat_map(Constraint::regions))
            .chain((self.contact_pairs.iter()).flat_map(|c| [&c.master, &c.slave]))
            .chain((self.ties.iter()).flat_map(|t| [&t.master, &t.slave]))
            .chain(self.node_ties.iter().map(|t| &t.region))
            .chain(self.contact_regions.iter().flat_map(ContactRegion::regions))
            .chain(self.bolts.iter().flat_map(Bolt::regions))
            .chain(self.mesh_sets.iter().map(|s| &s.region))
            .chain(self.steps.iter().flat_map(|step| {
                (step.boundary_conditions.iter().map(|b| &b.region))
                    .chain(step.loads.iter().map(|l| &l.region))
                    .chain(step.history_outputs.iter().filter_map(|h| h.kind.region()))
                    .chain(step.kind.design_region())
            }))
    }

    /// Follows Gmsh's new numbers of the CAD entities after a geometry part was deleted:
    /// `tags` gives the new entity by the old one. Entities without a new one are dropped
    /// from the regions picked on the geometry.
    pub fn renumber_cad(&mut self, tags: &std::collections::BTreeMap<CadEntity, CadEntity>) {
        for region in self.regions_mut() {
            if let Region::Geometry(entities) = region {
                *entities = entities
                    .iter()
                    .filter_map(|e| tags.get(e).copied())
                    .collect();
            }
        }
    }

    /// Follows the CAD entities into a geometry that took the place of the one they were
    /// picked on, `tags` from [`Geometry::tags_in`]: each entity names the one it became, or
    /// is marked lost ([`CadEntity::lost`]) when it was not found, so that the item shows it
    /// must be picked again. Returns how many were lost.
    pub fn follow_cad(&mut self, tags: &TagMap) -> usize {
        let mut lost = 0;
        let mut follow = |region: &mut Region| {
            if let Region::Geometry(entities) = region {
                for entity in entities.iter_mut().filter(|e| !e.is_lost()) {
                    *entity = tags.get(entity).copied().unwrap_or_else(|| {
                        lost += 1;
                        entity.lost()
                    });
                }
            }
        };
        self.regions_mut().for_each(&mut follow);
        (self.initial_conditions.iter_mut()).for_each(|i| follow(&mut i.region));
        lost
    }

    /// Follows nodes merged into others, as [`plx_mesh::FeMesh::merge_nodes`] does: regions
    /// of nodes name the nodes that stay.
    pub fn merge_nodes(&mut self, replaced: &std::collections::BTreeMap<NodeId, NodeId>) {
        let follow = |region: &mut Region| match region {
            Region::Nodes(nodes) => {
                for node in nodes.iter_mut() {
                    if let Some(&kept) = replaced.get(node) {
                        *node = kept;
                    }
                }
                nodes.sort_unstable();
                nodes.dedup();
            }
            Region::Edges(edges) => {
                for edge in edges.iter_mut() {
                    *edge = edge.map(|id| replaced.get(&id).copied().unwrap_or(id));
                }
            }
            _ => {}
        };
        self.regions_mut().for_each(follow);
        (self.initial_conditions.iter_mut()).for_each(|i| follow(&mut i.region));
    }

    /// Follows renumbered nodes and elements, as [`plx_mesh::FeMesh::renumber`] does:
    /// regions of node and face ids name the new ids.
    pub fn renumber(
        &mut self,
        nodes: &std::collections::BTreeMap<NodeId, NodeId>,
        elements: &std::collections::BTreeMap<plx_mesh::ElementId, plx_mesh::ElementId>,
    ) {
        let follow = |region: &mut Region| match region {
            Region::Nodes(ids) => {
                for id in ids.iter_mut() {
                    *id = nodes.get(id).copied().unwrap_or(*id);
                }
                ids.sort_unstable();
            }
            Region::Edges(edges) => {
                for edge in edges.iter_mut() {
                    *edge = edge.map(|id| nodes.get(&id).copied().unwrap_or(id));
                }
            }
            Region::Faces(faces) => {
                for (element, _) in faces.iter_mut() {
                    *element = elements.get(element).copied().unwrap_or(*element);
                }
            }
            _ => {}
        };
        self.regions_mut().for_each(follow);
        (self.initial_conditions.iter_mut()).for_each(|i| follow(&mut i.region));
    }

    /// Follows the face numbers of inverted elements, as
    /// [`plx_mesh::FeMesh::transform_parts`] returns them: regions of faces name the new
    /// numbers.
    pub fn renumber_faces(&mut self, renumbering: &plx_mesh::FaceRenumbering) {
        let follow = |region: &mut Region| {
            if let Region::Faces(faces) = region {
                for (element, face) in faces.iter_mut() {
                    if let Some(new) = renumbering.get(element)
                        && let Some(&number) = new.get(usize::from(*face).wrapping_sub(1))
                        && number > 0
                    {
                        *face = number;
                    }
                }
            }
        };
        self.regions_mut().for_each(follow);
        (self.initial_conditions.iter_mut()).for_each(|i| follow(&mut i.region));
    }

    pub(crate) fn regions_mut(&mut self) -> impl Iterator<Item = &mut Region> {
        (self.sections.iter_mut().map(|s| &mut s.region))
            .chain(
                self.constraints
                    .iter_mut()
                    .flat_map(Constraint::regions_mut),
            )
            .chain((self.contact_pairs.iter_mut()).flat_map(|c| [&mut c.master, &mut c.slave]))
            .chain((self.ties.iter_mut()).flat_map(|t| [&mut t.master, &mut t.slave]))
            .chain(self.node_ties.iter_mut().map(|t| &mut t.region))
            .chain((self.contact_regions.iter_mut()).flat_map(ContactRegion::regions_mut))
            .chain(self.bolts.iter_mut().flat_map(Bolt::regions_mut))
            .chain(self.mesh_sets.iter_mut().map(|s| &mut s.region))
            .chain(self.steps.iter_mut().flat_map(|step| {
                (step.boundary_conditions.iter_mut().map(|b| &mut b.region))
                    .chain(step.loads.iter_mut().map(|l| &mut l.region))
                    .chain(step.defined_fields.iter_mut().map(|f| &mut f.region))
                    .chain((step.history_outputs.iter_mut()).filter_map(|h| h.kind.region_mut()))
                    .chain(step.kind.design_region_mut())
            }))
    }

    /// Brings a model read from an older project up to date: ties and node ties saved among
    /// the constraints move to [`FeModel::ties`] and [`FeModel::node_ties`], in their order.
    pub fn migrate(&mut self) {
        let mut constraints = Vec::with_capacity(self.constraints.len());
        for constraint in std::mem::take(&mut self.constraints) {
            match constraint {
                Constraint::Tie(tie) => self.ties.push(tie),
                Constraint::NodeTie(tie) => self.node_ties.push(tie),
                other => constraints.push(other),
            }
        }
        self.constraints = constraints;
    }

    /// Follows a renamed amplitude: boundary conditions and loads keep referring to it.
    pub fn rename_amplitude(&mut self, old: &str, new: &str) {
        for step in &mut self.steps {
            let references = (step.boundary_conditions.iter_mut())
                .map(|b| &mut b.amplitude)
                .chain(
                    (step.loads.iter_mut())
                        .flat_map(|l| [&mut l.amplitude, &mut l.factor_amplitude]),
                )
                .chain(step.defined_fields.iter_mut().map(|f| &mut f.amplitude));
            for reference in references {
                if reference.as_deref() == Some(old) {
                    *reference = Some(new.to_string());
                }
            }
        }
    }

    /// Follows a renamed contact pair: contact history outputs keep referring to it.
    pub fn rename_contact_pair(&mut self, old: &str, new: &str) {
        for output in self.steps.iter_mut().flat_map(|s| &mut s.history_outputs) {
            if let HistoryKind::Contact { pair } = &mut output.kind
                && pair == old
            {
                *pair = new.to_string();
            }
        }
        for change in self.steps.iter_mut().flat_map(|s| &mut s.contact_changes) {
            if change.region == old {
                change.region = new.to_string();
            }
        }
    }

    /// Whether an active step that takes it has an active submodel boundary condition, so
    /// that the input file reads the results of the global model.
    pub fn uses_global_results(&self) -> bool {
        (self.steps.iter().filter(|s| s.active)).any(|step| {
            (step.boundary_conditions.iter()).any(|b| {
                b.active
                    && matches!(b.kind, BoundaryKind::Submodel { .. })
                    && step.kind.supports_boundary(&b.kind)
            })
        })
    }

    /// The amplitude of the name, if the model has it.
    pub fn amplitude(&self, name: &str) -> Option<&Amplitude> {
        self.amplitudes.iter().find(|a| a.name == name)
    }

    pub fn cross_section(&self, name: &str) -> Option<&CrossSection> {
        self.cross_sections.iter().find(|c| c.name == name)
    }

    /// The profile of a beam section's cross-section; `None` for other sections or a
    /// cross-section the model lacks.
    pub fn section_profile(&self, kind: &SectionKind) -> Option<&BeamProfile> {
        match kind {
            SectionKind::Beam(beam) => Some(&self.cross_section(&beam.cross_section)?.profile),
            _ => None,
        }
    }

    pub fn distribution(&self, name: &str) -> Option<&Distribution> {
        self.distributions.iter().find(|d| d.name == name)
    }

    /// Every reference to a distribution: of boundary conditions, loads, initial conditions
    /// and defined fields.
    fn distribution_references(&mut self) -> impl Iterator<Item = &mut Option<String>> {
        let initial = self
            .initial_conditions
            .iter_mut()
            .map(|c| &mut c.distribution);
        let steps = self.steps.iter_mut().flat_map(|step| {
            (step
                .boundary_conditions
                .iter_mut()
                .map(|b| &mut b.distribution))
            .chain(step.loads.iter_mut().map(|l| &mut l.distribution))
            .chain(step.defined_fields.iter_mut().map(|f| &mut f.distribution))
        });
        initial.chain(steps)
    }

    /// Follows a renamed distribution: the items that use it keep using it.
    pub fn rename_distribution(&mut self, old: &str, new: &str) {
        for reference in self.distribution_references() {
            if reference.as_deref() == Some(old) {
                *reference = Some(new.to_string());
            }
        }
    }

    /// The names of the distributions some item uses.
    pub fn used_distributions(&self) -> std::collections::BTreeSet<&str> {
        let initial = self.initial_conditions.iter().map(|c| &c.distribution);
        let steps = self.steps.iter().flat_map(|step| {
            (step.boundary_conditions.iter().map(|b| &b.distribution))
                .chain(step.loads.iter().map(|l| &l.distribution))
                .chain(step.defined_fields.iter().map(|f| &f.distribution))
        });
        initial.chain(steps).filter_map(|d| d.as_deref()).collect()
    }

    /// Follows a renamed part: regions on the part, or on the element set an input file
    /// defines for it, keep pointing at it.
    pub fn rename_part(&mut self, old: &str, new: &str) {
        self.rename_element_types(old, new);
        let regions = (self.sections.iter_mut().map(|s| &mut s.region))
            .chain(
                self.constraints
                    .iter_mut()
                    .flat_map(Constraint::regions_mut),
            )
            .chain((self.contact_pairs.iter_mut()).flat_map(|c| [&mut c.master, &mut c.slave]))
            .chain((self.contact_regions.iter_mut()).flat_map(ContactRegion::regions_mut))
            .chain(self.bolts.iter_mut().flat_map(Bolt::regions_mut))
            .chain(self.initial_conditions.iter_mut().map(|i| &mut i.region))
            .chain(self.mesh_sets.iter_mut().map(|s| &mut s.region))
            .chain(self.steps.iter_mut().flat_map(|step| {
                (step.boundary_conditions.iter_mut().map(|b| &mut b.region))
                    .chain(step.loads.iter_mut().map(|l| &mut l.region))
                    .chain(step.defined_fields.iter_mut().map(|f| &mut f.region))
                    .chain((step.history_outputs.iter_mut()).filter_map(|h| h.kind.region_mut()))
                    .chain(step.kind.design_region_mut())
            }));
        for region in regions {
            match region {
                Region::Parts(names) => {
                    for name in names.iter_mut().filter(|n| n.as_str() == old) {
                        *name = new.to_string();
                    }
                    // Merged parts leave the same name twice.
                    let mut seen = std::collections::BTreeSet::new();
                    names.retain(|name| seen.insert(name.clone()));
                }
                Region::ElementSet(name) if name == old => *name = new.to_string(),
                _ => {}
            }
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Material {
    pub name: String,
    pub density: Option<f64>,
    pub elastic: Option<Elastic>,
    /// Thermal conductivity (`*CONDUCTIVITY`), for heat transfer.
    #[serde(default)]
    pub conductivity: Option<f64>,
    /// Specific heat (`*SPECIFIC HEAT`), for transient heat transfer.
    #[serde(default)]
    pub specific_heat: Option<f64>,
    /// Thermal expansion (`*EXPANSION`), for thermal strains.
    #[serde(default)]
    pub expansion: Option<Expansion>,
    /// Rate-independent plasticity (`*PLASTIC`) with a tabular hardening curve.
    #[serde(default)]
    pub plastic: Option<Plastic>,
    /// Creep by Norton's law (`*CREEP`), which acts in Visco steps.
    #[serde(default)]
    pub creep: Option<Creep>,
}

/// Norton's creep law (`*CREEP, LAW=NORTON`): the equivalent creep strain rate is
/// `a * stress^n * time^m`, stored in SI units (stress in Pa, time in s).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Creep {
    pub a: f64,
    pub n: f64,
    pub m: f64,
}

impl Default for Creep {
    /// A steel creeping about 1 % in 1000 s at 100 MPa: `a` = 1e-15 for MPa.
    fn default() -> Self {
        Self {
            a: 1e-45,
            n: 5.0,
            m: 0.0,
        }
    }
}

impl Creep {
    /// Why the solver would reject the constants.
    pub fn problem(&self) -> Option<&'static str> {
        if self.a <= 0.0 {
            Some("The creep constant A must be positive.")
        } else if self.n <= 0.0 {
            Some("The stress exponent n must be positive.")
        } else {
            None
        }
    }
}

/// How the yield surface grows with plastic strain (`*PLASTIC, HARDENING=`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Hardening {
    #[default]
    Isotropic,
    Kinematic,
    Combined,
}

impl Hardening {
    pub const ALL: [Hardening; 3] = [
        Hardening::Isotropic,
        Hardening::Kinematic,
        Hardening::Combined,
    ];

    /// The value of CalculiX's `HARDENING` parameter.
    pub fn keyword(self) -> &'static str {
        match self {
            Hardening::Isotropic => "Isotropic",
            Hardening::Kinematic => "Kinematic",
            Hardening::Combined => "Combined",
        }
    }
}

/// One point of the hardening curve: the yield stress at a plastic strain, valid at a
/// temperature. The solver interpolates between the points and keeps the last stress beyond
/// them; rows at different temperatures are interpolated in temperature.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlasticPoint {
    pub stress: f64,
    pub plastic_strain: f64,
    pub temperature: f64,
}

/// Von Mises plasticity with a tabular hardening curve.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Plastic {
    pub hardening: Hardening,
    /// Rows in the order the solver wants them: by temperature, then by plastic strain,
    /// the first row of every temperature at plastic strain 0.
    pub points: Vec<PlasticPoint>,
}

impl Default for Plastic {
    /// An ideally plastic material has one row; the yield stress is still to be entered.
    fn default() -> Self {
        Self {
            hardening: Hardening::Isotropic,
            points: vec![PlasticPoint {
                stress: 0.0,
                plastic_strain: 0.0,
                temperature: 0.0,
            }],
        }
    }
}

impl Plastic {
    /// The first row of each temperature must start at plastic strain 0 and the plastic
    /// strain must grow within a temperature; otherwise the solver stops with an error or
    /// interpolates nonsense. Returns the number of the first offending row.
    pub fn invalid_row(&self) -> Option<usize> {
        if self.points.is_empty() {
            return Some(0);
        }
        let mut previous: Option<&PlasticPoint> = None;
        for (i, point) in self.points.iter().enumerate() {
            let new_temperature = previous.is_none_or(|p| p.temperature != point.temperature);
            let ok = if new_temperature {
                point.plastic_strain == 0.0 && point.stress > 0.0
            } else {
                previous.is_some_and(|p| point.plastic_strain > p.plastic_strain)
                    && point.stress > 0.0
            };
            if !ok {
                return Some(i);
            }
            previous = Some(point);
        }
        None
    }
}

/// Linear isotropic thermal expansion.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Expansion {
    pub coefficient: f64,
    /// Temperature at which the expansion is zero (`ZERO=`); the default is 20.
    pub zero_temperature: f64,
}

impl Default for Expansion {
    fn default() -> Self {
        Self {
            coefficient: 0.0,
            zero_temperature: 20.0,
        }
    }
}

/// Linear isotropic elasticity.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Elastic {
    pub young: f64,
    pub poisson: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Step {
    pub name: String,
    /// A deactivated step is written to the input file as a comment only.
    #[serde(default = "active")]
    pub active: bool,
    pub kind: StepKind,
    pub boundary_conditions: Vec<BoundaryCondition>,
    pub loads: Vec<Load>,
    pub field_outputs: Vec<FieldOutput>,
    /// Values printed into the `.dat` file; steps saved before they existed have none.
    #[serde(default)]
    pub history_outputs: Vec<HistoryOutput>,
    /// Temperatures prescribed for the step; steps saved before
    /// they existed have none.
    #[serde(default)]
    pub defined_fields: Vec<DefinedField>,
    /// Contact regions and pairs switched on or off, or given another friction, from this
    /// step on.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub contact_changes: Vec<ContactChange>,
    /// What bolts do in this step instead of their automatic action.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bolt_changes: Vec<BoltChange>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum StepKind {
    Static(StaticStep),
    /// Eigenfrequencies and mode shapes (`*FREQUENCY`).
    Frequency(FrequencyStep),
    /// Complex eigenfrequencies of a rotating structure with Coriolis forces
    /// (`*COMPLEX FREQUENCY`), from the eigenmodes a frequency step stored before.
    ComplexFrequency(ComplexFrequencyStep),
    /// Buckling factors and buckling modes (`*BUCKLE`).
    Buckle(BuckleStep),
    /// Temperatures only (`*HEAT TRANSFER`).
    HeatTransfer(HeatTransferStep),
    /// Temperatures and displacements solved together
    /// (`*COUPLED TEMPERATURE-DISPLACEMENT`).
    CoupledTempDisp(HeatTransferStep),
    /// Temperatures first, then displacements from them, in each increment
    /// (`*UNCOUPLED TEMPERATURE-DISPLACEMENT`): cheaper than the coupled step when the
    /// deformation does not change the heat flow.
    UncoupledTempDisp(HeatTransferStep),
    /// Displacements over time with inertia and damping (`*DYNAMIC`).
    Dynamic(DynamicStep),
    /// Creep over time of materials with a creep law (`*VISCO`), otherwise a static step.
    Visco(ViscoStep),
    /// Response over time as a superposition of the stored eigenmodes (`*MODAL DYNAMICS`).
    ModalDynamics(ModalDynamicsStep),
    /// Harmonic response over a frequency range from the stored eigenmodes
    /// (`*STEADY STATE DYNAMICS`).
    SteadyStateDynamics(SteadyStateDynamicsStep),
    /// How strongly mass, stiffness or an eigenfrequency react to moving the surface nodes
    /// of a design region (`*SENSITIVITY`), from the state of the step before it.
    Sensitivity(SensitivityStep),
}

impl StepKind {
    /// Whether the step is computed with large deformations (`NLGEOM`).
    pub fn nlgeom(&self) -> bool {
        match self {
            StepKind::Static(settings) => settings.nlgeom,
            StepKind::CoupledTempDisp(settings) => settings.increments.nlgeom,
            StepKind::Dynamic(settings) => settings.increments.nlgeom,
            StepKind::Visco(settings) => settings.increments.nlgeom,
            _ => false,
        }
    }

    /// Whether the step superposes the eigenmodes a previous frequency step stored.
    pub fn uses_stored_modes(&self) -> bool {
        matches!(
            self,
            StepKind::ModalDynamics(_) | StepKind::SteadyStateDynamics(_)
        )
    }

    /// Whether the step takes loads. A frequency step has none; preloads
    /// come from the previous step with [`FrequencyStep::perturbation`].
    pub fn supports_loads(&self) -> bool {
        !matches!(
            self,
            StepKind::Frequency(_) | StepKind::ComplexFrequency(_) | StepKind::Sensitivity(_)
        )
    }

    /// The design region of a sensitivity step.
    pub fn design_region(&self) -> Option<&Region> {
        match self {
            StepKind::Sensitivity(settings) => Some(&settings.design_region),
            _ => None,
        }
    }

    pub fn design_region_mut(&mut self) -> Option<&mut Region> {
        match self {
            StepKind::Sensitivity(settings) => Some(&mut settings.design_region),
            _ => None,
        }
    }

    /// Whether the step keeps the supports of the step before it. A sensitivity step
    /// evaluates that step's solution, so it takes no boundary conditions of its own.
    pub fn keeps_previous_boundaries(&self) -> bool {
        matches!(self, StepKind::Sensitivity(_))
    }

    /// Whether the step computes eigenmodes, whose shapes are scaled arbitrarily.
    pub fn is_modal(&self) -> bool {
        matches!(self, StepKind::Frequency(_) | StepKind::ComplexFrequency(_))
    }

    /// Whether the step takes defined fields. A thermal step solves for the temperatures
    /// instead of taking them, and a step that superposes stored eigenmodes
    /// only takes the loads the solver allows there (forces, pressures, base motion).
    pub fn supports_defined_fields(&self) -> bool {
        !self.is_thermal() && !self.uses_stored_modes() && !self.keeps_previous_boundaries()
    }

    /// Whether the step solves for displacements.
    pub fn is_mechanical(&self) -> bool {
        !matches!(self, StepKind::HeatTransfer(_))
    }

    /// Whether the step solves for temperatures.
    pub fn is_thermal(&self) -> bool {
        matches!(
            self,
            StepKind::HeatTransfer(_)
                | StepKind::CoupledTempDisp(_)
                | StepKind::UncoupledTempDisp(_)
        )
    }

    /// Whether a boundary condition of this kind acts in the step:
    /// temperatures in thermal steps, displacements in
    /// mechanical ones.
    pub fn supports_boundary(&self, kind: &BoundaryKind) -> bool {
        // Nodalis drives submodels in static steps only.
        if self.keeps_previous_boundaries() {
            false
        } else if let BoundaryKind::Submodel { .. } = kind {
            matches!(self, StepKind::Static(_))
        } else if kind.is_thermal() {
            self.is_thermal()
        } else {
            self.is_mechanical()
        }
    }

    /// Whether a load of this kind acts in the step.
    pub fn supports_load(&self, kind: &LoadKind) -> bool {
        // Each position of a moving load is a static step of its own.
        if matches!(kind, LoadKind::Moving(_)) {
            return matches!(self, StepKind::Static(_));
        }
        self.supports_loads()
            && if kind.is_thermal() {
                self.is_thermal()
            } else {
                self.is_mechanical()
            }
    }
}

/// How the increments of a step are chosen.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Incrementation {
    /// CalculiX's own defaults; no increment data is written.
    #[default]
    Default,
    /// Automatic increments within the given limits.
    Automatic,
    /// Fixed increments of `initial_increment` (`DIRECT`).
    Direct,
}

/// How a step solves its systems of equations.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum LinearSolverChoice {
    /// The solver chooses: direct while the factor fits into the memory, iterative beyond
    /// where the step allows it, and the other one when the first fails.
    #[default]
    Automatic,
    /// A sparse direct factorization (LDLᵀ): any system, memory for the factor.
    Direct,
    /// Conjugate gradients with an incomplete Cholesky preconditioner: symmetric positive
    /// definite systems only (static, also nonlinear and with contact, dynamic and heat
    /// transfer steps), far less memory.
    Iterative,
}

impl LinearSolverChoice {
    pub const ALL: [LinearSolverChoice; 3] = [
        LinearSolverChoice::Automatic,
        LinearSolverChoice::Direct,
        LinearSolverChoice::Iterative,
    ];

    /// The name shown in the interface.
    pub fn name(self) -> &'static str {
        match self {
            LinearSolverChoice::Automatic => "Automatic",
            LinearSolverChoice::Direct => "Direct",
            LinearSolverChoice::Iterative => "Iterative",
        }
    }

    /// The value of `SOLVER=` in the input file. Automatic leaves the choice to the solver
    /// (the solver takes its default); Direct names PaStiX, which the Nodalis solver reads as
    /// its direct solver; Iterative asks for conjugate gradients with an incomplete
    /// Cholesky preconditioner, which CalculiX and the Nodalis solver both have.
    pub fn keyword(self) -> Option<&'static str> {
        match self {
            LinearSolverChoice::Automatic => None,
            LinearSolverChoice::Direct => Some("PaStiX"),
            LinearSolverChoice::Iterative => Some("Iterative Cholesky"),
        }
    }
}

/// Settings of a `*STATIC` step, with the default values.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StaticStep {
    /// Geometrically nonlinear (`NLGEOM`).
    pub nlgeom: bool,
    pub incrementation: Incrementation,
    /// Maximum number of increments (`INC`), written unless incrementation is default.
    pub max_increments: u32,
    pub initial_increment: f64,
    pub time_period: f64,
    pub min_increment: f64,
    pub max_increment: f64,
    /// Missing in projects saved before the solver could be chosen.
    #[serde(default)]
    pub solver: LinearSolverChoice,
}

impl Default for StaticStep {
    fn default() -> Self {
        Self {
            nlgeom: false,
            incrementation: Incrementation::Default,
            max_increments: 100,
            initial_increment: 1.0,
            time_period: 1.0,
            min_increment: 1e-5,
            max_increment: 1e30,
            solver: LinearSolverChoice::Automatic,
        }
    }
}

/// Settings of a `*HEAT TRANSFER` or `*COUPLED TEMPERATURE-DISPLACEMENT` step, with
/// the default values; it extends the static step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HeatTransferStep {
    /// Increments, time period and solver as in a static step. `nlgeom` only applies to a
    /// coupled step.
    pub increments: StaticStep,
    /// Steady state (`STEADY STATE`): the temperatures the loads settle to, without heat
    /// capacity; otherwise transient over the time period.
    pub steady_state: bool,
    /// Largest temperature change allowed in an increment of a transient analysis
    /// (`DELTMX`); `None` leaves it open.
    pub deltmx: Option<f64>,
}

impl Default for HeatTransferStep {
    fn default() -> Self {
        Self {
            increments: StaticStep::default(),
            steady_state: true,
            deltmx: None,
        }
    }
}

/// Settings of a `*FREQUENCY` step, with the default values.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FrequencyStep {
    /// Number of eigenfrequencies to compute.
    pub num_frequencies: u32,
    /// Lower bound of the frequency range in cycles per time unit; `None` is CalculiX's 0.
    pub lower_frequency: Option<f64>,
    /// Upper bound of the frequency range; `None` leaves it open.
    pub upper_frequency: Option<f64>,
    /// Writes eigenvalues, mode shapes and the mass and stiffness matrices to the `.eig`
    /// file (`STORAGE=YES`).
    pub storage: bool,
    /// Takes the stiffness of the deformed state of the previous step into account, e.g. the
    /// stiffening by a preload (`*STEP, PERTURBATION`).
    pub perturbation: bool,
    /// Lowest and highest nodal diameter of a cyclic symmetric model
    /// (`*SELECT CYCLIC SYMMETRY MODES, NMIN=, NMAX=`); `None` is CalculiX's 0 and the
    /// highest one the sectors allow.
    #[serde(default)]
    pub nodal_diameters: Option<[u32; 2]>,
    /// How the shifted systems of the eigenvalue iteration are solved. Iterative needs no
    /// lower frequency bound (the shift must stay below the spectrum).
    #[serde(default)]
    pub solver: LinearSolverChoice,
}

/// Settings of a `*DYNAMIC` step, with the default values; it
/// extends the static step by the time integration and the damping.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DynamicStep {
    /// Increments, time period, geometric nonlinearity and solver as in a static step.
    pub increments: StaticStep,
    /// Numerical damping of the Hilber-Hughes-Taylor integration (`ALPHA=`), between -1/3
    /// and 0; CalculiX's default is -0.05.
    pub alpha: f64,
    /// Implicit or explicit integration of the structure and the fluid (`EXPLICIT=`).
    pub procedure: DynamicProcedure,
    /// Rayleigh damping of the whole model, written as `*DAMPING` in the step
    /// does; `None` leaves the model undamped.
    pub damping: Option<RayleighDamping>,
}

impl Default for DynamicStep {
    fn default() -> Self {
        Self {
            increments: StaticStep {
                incrementation: Incrementation::Automatic,
                initial_increment: 0.01,
                ..StaticStep::default()
            },
            alpha: -0.05,
            procedure: DynamicProcedure::Implicit,
            damping: None,
        }
    }
}

impl DynamicStep {
    /// What is wrong with the settings, if anything.
    pub fn problem(&self) -> Option<String> {
        if !(-1.0 / 3.0..=0.0).contains(&self.alpha) {
            return Some("Alpha must lie between -1/3 and 0.".into());
        }
        if self.increments.incrementation == Incrementation::Default {
            return Some("A dynamic step needs its time period and increments.".into());
        }
        if self.increments.time_period <= 0.0 || self.increments.initial_increment <= 0.0 {
            return Some(
                "The time period and the initial increment must be greater than 0.".into(),
            );
        }
        if let Some(damping) = &self.damping
            && (damping.alpha < 0.0 || damping.beta < 0.0)
        {
            return Some("The damping coefficients cannot be negative.".into());
        }
        None
    }
}

/// Settings of a `*MODAL DYNAMICS` step, with the default values: the response over time
/// as a superposition of the eigenmodes a previous frequency step stored.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ModalDynamicsStep {
    /// Fixed time increment of the response.
    pub increment: f64,
    pub time_period: f64,
    /// Maximum number of increments (`INC`); the writer raises it to fit the time period.
    pub max_increments: u32,
    /// Steady state (`STEADY STATE`): integrated until the response repeats within the
    /// relative error, instead of over the time period.
    pub steady_state: bool,
    pub relative_error: f64,
    pub damping: Option<ModalDamping>,
}

impl Default for ModalDynamicsStep {
    fn default() -> Self {
        Self {
            increment: 0.1,
            time_period: 1.0,
            max_increments: 100,
            steady_state: false,
            relative_error: 0.01,
            damping: None,
        }
    }
}

impl ModalDynamicsStep {
    /// What is wrong with the settings, if anything.
    pub fn problem(&self) -> Option<String> {
        if self.increment <= 0.0 {
            return Some("The time increment must be greater than 0.".into());
        }
        if !self.steady_state && self.time_period <= 0.0 {
            return Some("The time period must be greater than 0.".into());
        }
        if self.steady_state && !(0.0..=1.0).contains(&self.relative_error) {
            return Some("The relative error must lie between 0 and 1.".into());
        }
        self.damping.as_ref().and_then(ModalDamping::problem)
    }

    /// The `INC` of the step: the limit, or the number of increments the time period
    /// takes when that is more.
    pub fn increments(&self) -> u32 {
        if self.steady_state || self.increment <= 0.0 {
            return self.max_increments;
        }
        let needed = (self.time_period / self.increment).ceil() as u32 + 1;
        self.max_increments.max(needed)
    }
}

/// Settings of a `*STEADY STATE DYNAMICS` step, with the default values: the harmonic
/// response over a frequency range from the stored eigenmodes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SteadyStateDynamicsStep {
    /// Harmonic excitation; otherwise the loads are periodic over the time range and
    /// expanded into Fourier terms (`HARMONIC=NO`).
    pub harmonic: bool,
    pub lower_frequency: f64,
    pub upper_frequency: f64,
    /// Frequencies evaluated between two eigenfrequencies, at least 2.
    pub data_points: u32,
    /// Crowds the frequencies towards the eigenfrequencies; 1 spreads them evenly.
    pub bias: f64,
    /// Fourier terms of a non-harmonic excitation.
    pub fourier_terms: u32,
    /// Time range of one period of a non-harmonic excitation.
    pub time_lower: f64,
    pub time_upper: f64,
    pub damping: Option<ModalDamping>,
}

impl Default for SteadyStateDynamicsStep {
    fn default() -> Self {
        Self {
            harmonic: true,
            lower_frequency: 0.0,
            upper_frequency: 10.0,
            data_points: 20,
            bias: 3.0,
            fourier_terms: 20,
            time_lower: 0.0,
            time_upper: 1.0,
            damping: None,
        }
    }
}

impl SteadyStateDynamicsStep {
    /// What is wrong with the settings, if anything.
    pub fn problem(&self) -> Option<String> {
        if self.lower_frequency < 0.0 || self.upper_frequency <= self.lower_frequency {
            return Some("The upper frequency must be greater than the lower one.".into());
        }
        if self.data_points < 2 {
            return Some("At least 2 data points are needed.".into());
        }
        if self.bias < 1.0 {
            return Some("The bias must be 1 or more.".into());
        }
        if !self.harmonic && (self.fourier_terms < 1 || self.time_upper <= self.time_lower) {
            return Some("A periodic excitation needs Fourier terms and a time range.".into());
        }
        self.damping.as_ref().and_then(ModalDamping::problem)
    }
}

/// Damping of the modes of a modal dynamics or steady state dynamics step
/// (`*MODAL DAMPING`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ModalDamping {
    /// One viscous damping ratio (damping over critical damping) for all modes.
    Constant(f64),
    /// A damping ratio per range of modes.
    Direct(Vec<ModeDamping>),
    /// Rayleigh damping from the mass and stiffness matrices.
    Rayleigh(RayleighDamping),
}

impl ModalDamping {
    /// What is wrong with the damping, if anything.
    pub fn problem(&self) -> Option<String> {
        let bad = |ratio: f64| !(0.0..=1.0).contains(&ratio);
        match self {
            ModalDamping::Constant(ratio) if bad(*ratio) => {
                Some("The damping ratio must lie between 0 and 1.".into())
            }
            ModalDamping::Direct(ranges) if ranges.is_empty() => {
                Some("Give at least one range of modes with its damping ratio.".into())
            }
            ModalDamping::Direct(ranges)
                if ranges
                    .iter()
                    .any(|r| r.lowest < 1 || r.highest < r.lowest || bad(r.ratio)) =>
            {
                Some("Each range needs modes from 1 up and a ratio between 0 and 1.".into())
            }
            ModalDamping::Rayleigh(r) if r.alpha < 0.0 || r.beta < 0.0 => {
                Some("The damping coefficients cannot be negative.".into())
            }
            _ => None,
        }
    }
}

/// The viscous damping ratio of a range of modes.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ModeDamping {
    pub lowest: u32,
    pub highest: u32,
    pub ratio: f64,
}

/// How a dynamic step integrates over time, CalculiX's `EXPLICIT` parameter of `*DYNAMIC`:
/// the structure and, with fluids, the fluid.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum DynamicProcedure {
    /// Implicit for both, the default; unconditionally stable.
    #[default]
    Implicit,
    /// Implicit structure, explicit fluid (`EXPLICIT=1`).
    ImplicitExplicit,
    /// Explicit structure, implicit fluid (`EXPLICIT=2`).
    ExplicitImplicit,
    /// Explicit for both (`EXPLICIT=3`); needs increments below the stability limit.
    Explicit,
}

impl DynamicProcedure {
    pub const ALL: [DynamicProcedure; 4] = [
        DynamicProcedure::Implicit,
        DynamicProcedure::ImplicitExplicit,
        DynamicProcedure::ExplicitImplicit,
        DynamicProcedure::Explicit,
    ];

    /// The value of `EXPLICIT=`; `None` for the implicit default.
    pub fn keyword(self) -> Option<u8> {
        match self {
            DynamicProcedure::Implicit => None,
            DynamicProcedure::ImplicitExplicit => Some(1),
            DynamicProcedure::ExplicitImplicit => Some(2),
            DynamicProcedure::Explicit => Some(3),
        }
    }

    /// Name of the procedure in the GUI.
    pub fn label(self) -> &'static str {
        match self {
            DynamicProcedure::Implicit => "Implicit / Implicit",
            DynamicProcedure::ImplicitExplicit => "Implicit / Explicit",
            DynamicProcedure::ExplicitImplicit => "Explicit / Implicit",
            DynamicProcedure::Explicit => "Explicit / Explicit",
        }
    }
}

/// Rayleigh damping: the damping matrix is `alpha` times the mass plus `beta` times the
/// stiffness matrix. For a damping ratio zeta at the circular frequency omega,
/// `alpha = 2 zeta omega` (mass) or `beta = 2 zeta / omega` (stiffness).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RayleighDamping {
    /// Mass-proportional coefficient, in 1 / time.
    pub alpha: f64,
    /// Stiffness-proportional coefficient, in time.
    pub beta: f64,
}

impl Default for FrequencyStep {
    fn default() -> Self {
        Self {
            num_frequencies: 10,
            lower_frequency: None,
            upper_frequency: None,
            storage: false,
            perturbation: false,
            nodal_diameters: None,
            solver: LinearSolverChoice::Automatic,
        }
    }
}

/// Settings of a `*COMPLEX FREQUENCY` step, with the default values. The solver solves it on
/// the eigenmodes of the last frequency step with [`FrequencyStep::storage`]; the Coriolis
/// forces come from the centrifugal load of the static step before that frequency step.
///
/// The step has no perturbation flag of its own: the solver stops unless `*STEP, PERTURBATION`
/// of this step matches the one of the frequency step that stored the modes, so the writer
/// takes [`FrequencyStep::perturbation`] of that step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ComplexFrequencyStep {
    /// Number of complex eigenfrequencies to compute.
    pub num_frequencies: u32,
    /// Coriolis forces of the rotation (`CORIOLIS`); the usual reason for the step.
    pub coriolis: bool,
}

impl Default for ComplexFrequencyStep {
    fn default() -> Self {
        Self {
            num_frequencies: 10,
            coriolis: true,
        }
    }
}

/// Settings of a `*BUCKLE` step, with the default values. The loads of the step are the
/// reference loads; the buckling factors scale them to the critical loads.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BuckleStep {
    /// Number of buckling factors to compute.
    pub num_factors: u32,
    /// Accuracy of the eigenvalue solver.
    pub accuracy: f64,
    /// Adds the stiffness of the deformed state of the previous step, e.g. of a preload that
    /// is not scaled by the buckling factor (`*STEP, PERTURBATION`).
    pub perturbation: bool,
    /// How the reference solution and the systems of the eigenvalue iteration are solved.
    #[serde(default)]
    pub solver: LinearSolverChoice,
}

impl Default for BuckleStep {
    fn default() -> Self {
        Self {
            num_factors: 1,
            accuracy: 1e-4,
            perturbation: false,
            solver: LinearSolverChoice::Automatic,
        }
    }
}

/// Settings of a `*VISCO` step: the increments of a static step over the creep time, and
/// the creep strain tolerance (`CETOL`) that limits the automatic increments.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ViscoStep {
    pub increments: StaticStep,
    /// Largest difference of the creep strain increment computed from the rates at the
    /// start and at the end of an increment (`CETOL`).
    pub creep_tolerance: f64,
}

impl Default for ViscoStep {
    fn default() -> Self {
        Self {
            increments: StaticStep {
                incrementation: Incrementation::Automatic,
                max_increments: 1000,
                initial_increment: 1.0,
                time_period: 1000.0,
                ..StaticStep::default()
            },
            creep_tolerance: 1e-4,
        }
    }
}

/// Settings of a `*SENSITIVITY` step: the design region whose surface nodes may move
/// (`*DESIGN VARIABLES, TYPE=COORDINATE`) and the design responses whose derivatives with
/// respect to these node positions the solver computes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SensitivityStep {
    /// Nodes that may move, usually picked faces. The solver keeps only surface nodes.
    pub design_region: Region,
    /// Mass of the model (`MASS`).
    pub mass: bool,
    /// Strain energy (`STRAINENERGY`), the compliance under the loads of the previous
    /// static step: less strain energy means a stiffer part.
    pub strain_energy: bool,
    /// The eigenfrequencies of the previous frequency step (`EIGENFREQUENCY`). CalculiX
    /// takes no other response together with it.
    pub eigenfrequency: bool,
    /// Radius of the linear filter that smooths the sensitivities (`*FILTER`).
    pub filter_radius: Option<f64>,
}

impl Default for SensitivityStep {
    fn default() -> Self {
        Self {
            design_region: Region::Nodes(Vec::new()),
            mass: true,
            strain_energy: true,
            eigenfrequency: false,
            filter_radius: None,
        }
    }
}

impl SensitivityStep {
    /// The design responses as CalculiX names them (`*DESIGN RESPONSE`).
    pub fn responses(&self) -> Vec<&'static str> {
        let mut responses = Vec::new();
        if self.eigenfrequency {
            responses.push("EIGENFREQUENCY");
        }
        if self.mass {
            responses.push("MASS");
        }
        if self.strain_energy {
            responses.push("STRAINENERGY");
        }
        responses
    }

    /// Why the solver would reject the settings.
    pub fn problem(&self) -> Option<String> {
        if self.responses().is_empty() {
            Some("Select at least one design response.".into())
        } else if self.eigenfrequency && (self.mass || self.strain_energy) {
            Some("The eigenfrequency can only be the single design response.".into())
        } else if self.filter_radius.is_some_and(|r| r <= 0.0) {
            Some("The filter radius must be positive.".into())
        } else {
            None
        }
    }
}

impl Step {
    /// A sensitivity step: its node output holds the sensitivities (`SEN`).
    pub fn new_sensitivity(name: impl Into<String>) -> Self {
        Self {
            kind: StepKind::Sensitivity(SensitivityStep::default()),
            field_outputs: FieldOutput::sensitivity_defaults(),
            ..Self::new_static(name)
        }
    }

    /// A complex frequency step with the default field outputs.
    pub fn new_complex_frequency(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            kind: StepKind::ComplexFrequency(ComplexFrequencyStep::default()),
            boundary_conditions: Vec::new(),
            loads: Vec::new(),
            history_outputs: Vec::new(),
            field_outputs: FieldOutput::complex_frequency_defaults(),
            defined_fields: Vec::new(),
            contact_changes: Vec::new(),
            bolt_changes: Vec::new(),
        }
    }

    /// A visco step with the static step's field outputs and the creep strain (`PEEQ`).
    pub fn new_visco(name: impl Into<String>) -> Self {
        let mut step = Self::new_static(name);
        step.kind = StepKind::Visco(ViscoStep::default());
        step.field_outputs = FieldOutput::visco_defaults();
        step
    }

    /// A static step with the default field outputs.
    pub fn new_static(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            kind: StepKind::Static(StaticStep::default()),
            boundary_conditions: Vec::new(),
            loads: Vec::new(),
            history_outputs: Vec::new(),
            defined_fields: Vec::new(),
            contact_changes: Vec::new(),
            bolt_changes: Vec::new(),
            field_outputs: FieldOutput::defaults(),
        }
    }

    /// A frequency step with the default field outputs.
    pub fn new_frequency(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            kind: StepKind::Frequency(FrequencyStep::default()),
            boundary_conditions: Vec::new(),
            loads: Vec::new(),
            history_outputs: Vec::new(),
            defined_fields: Vec::new(),
            contact_changes: Vec::new(),
            bolt_changes: Vec::new(),
            field_outputs: FieldOutput::frequency_defaults(),
        }
    }

    /// A buckle step with the default field outputs.
    pub fn new_buckle(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            kind: StepKind::Buckle(BuckleStep::default()),
            boundary_conditions: Vec::new(),
            loads: Vec::new(),
            history_outputs: Vec::new(),
            defined_fields: Vec::new(),
            contact_changes: Vec::new(),
            bolt_changes: Vec::new(),
            field_outputs: FieldOutput::defaults(),
        }
    }

    /// A heat transfer step with the default field outputs.
    pub fn new_heat_transfer(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            kind: StepKind::HeatTransfer(HeatTransferStep::default()),
            boundary_conditions: Vec::new(),
            loads: Vec::new(),
            history_outputs: Vec::new(),
            defined_fields: Vec::new(),
            contact_changes: Vec::new(),
            bolt_changes: Vec::new(),
            field_outputs: FieldOutput::heat_transfer_defaults(),
        }
    }

    /// A dynamic step with the default field outputs.
    pub fn new_dynamic(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            kind: StepKind::Dynamic(DynamicStep::default()),
            boundary_conditions: Vec::new(),
            loads: Vec::new(),
            history_outputs: Vec::new(),
            field_outputs: FieldOutput::dynamic_defaults(),
            defined_fields: Vec::new(),
            contact_changes: Vec::new(),
            bolt_changes: Vec::new(),
        }
    }

    /// A modal dynamics step with the default field outputs.
    pub fn new_modal_dynamics(name: impl Into<String>) -> Self {
        Self {
            kind: StepKind::ModalDynamics(ModalDynamicsStep::default()),
            ..Self::new_dynamic(name)
        }
    }

    /// A steady state dynamics step with the default field outputs.
    pub fn new_steady_state_dynamics(name: impl Into<String>) -> Self {
        Self {
            kind: StepKind::SteadyStateDynamics(SteadyStateDynamicsStep::default()),
            field_outputs: FieldOutput::defaults(),
            ..Self::new_dynamic(name)
        }
    }

    /// A coupled temperature-displacement step with the default field outputs.
    pub fn new_coupled(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            kind: StepKind::CoupledTempDisp(HeatTransferStep::default()),
            boundary_conditions: Vec::new(),
            loads: Vec::new(),
            history_outputs: Vec::new(),
            defined_fields: Vec::new(),
            contact_changes: Vec::new(),
            bolt_changes: Vec::new(),
            field_outputs: FieldOutput::coupled_defaults(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoundaryCondition {
    pub name: String,
    /// A deactivated boundary condition is left out of the input file and the 3D view.
    #[serde(default = "active")]
    pub active: bool,
    /// Hidden ones have no symbol in the 3D view unless selected.
    #[serde(default = "active")]
    pub visible: bool,
    pub region: Region,
    pub kind: BoundaryKind,
    /// Amplitude the values follow over time; `None` is CalculiX's default ramp or step.
    #[serde(default)]
    pub amplitude: Option<String>,
    /// Distribution the value is scaled by over the region; `None` is the same value
    /// everywhere.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distribution: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum BoundaryKind {
    /// Prescribed displacements (U1..U3) and rotations (UR1..UR3) of the region, held at
    /// every node or, where the region should turn, at a point it follows (Ansys' remote
    /// displacement); see [`Displacement::formulation`].
    Displacement(Displacement),
    /// Prescribed temperature (degree of freedom 11).
    Temperature(f64),
    /// Displacements (U1..U3) taken from the results of the global model of a submodel
    /// (`*BOUNDARY, SUBMODEL`). The results file of the global
    /// model holds no rotations, so the rotations (the last three) never follow it.
    Submodel {
        /// Step of the global model whose results are read, counted from 1; the results
        /// link of the step sets it.
        step: u32,
        /// The degrees of freedom that follow the global model.
        dofs: [bool; 6],
    },
    /// The region lies on a symmetry plane: the displacement normal to the plane and, for
    /// beams and shells, the rotations about the two in-plane axes are zero. The plane is
    /// fitted to the nodes of the region when it is needed, see [`symmetry_plane`].
    Symmetry {
        /// Normal of the plane as the user entered it; `None` takes it from the region.
        normal: Option<[f64; 3]>,
    },
}

impl BoundaryKind {
    /// Whether an amplitude can scale the boundary condition; symmetry supports stay zero.
    pub fn takes_amplitude(&self) -> bool {
        !matches!(
            self,
            BoundaryKind::Submodel { .. } | BoundaryKind::Symmetry { .. }
        )
    }

    pub fn is_thermal(&self) -> bool {
        matches!(self, BoundaryKind::Temperature(_))
    }

    /// A displacement with the values, written as the selection needs.
    pub fn displacement(values: [Option<f64>; 6]) -> Self {
        BoundaryKind::Displacement(Displacement::new(values))
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Load {
    pub name: String,
    /// A deactivated load is left out of the input file and the 3D view.
    #[serde(default = "active")]
    pub active: bool,
    /// Hidden ones have no symbol in the 3D view unless selected.
    #[serde(default = "active")]
    pub visible: bool,
    pub region: Region,
    pub kind: LoadKind,
    /// Amplitude the value follows over time, of a film or radiation its sink temperature;
    /// `None` is CalculiX's default ramp or step.
    #[serde(default)]
    pub amplitude: Option<String>,
    /// Amplitude of the film coefficient or the emissivity (`FILM AMPLITUDE`,
    /// `RADIATION AMPLITUDE`); other loads have none.
    #[serde(default)]
    pub factor_amplitude: Option<String>,
    /// Distribution the value is scaled by over the region; `None` is the same value
    /// everywhere.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distribution: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum LoadKind {
    /// Moment applied at every node of the region (`*CLOAD` on degrees of freedom 4 to 6):
    /// at a reference point of a rigid body, or at nodes of beams and shells, which have
    /// rotations.
    Moment([f64; 3]),
    /// Pressure on a surface region; positive pushes into the material.
    Pressure(f64),
    /// Total force on the region, spread over its nodes when the input file is written: on
    /// faces by area (the surface traction), along edges by length and on nodes in
    /// equal shares; at the reference point of a rigid body it acts as a whole. With
    /// `each_node` the whole value acts at every node of the region instead
    /// (a concentrated force).
    Force { value: [f64; 3], each_node: bool },
    /// Bolt preload across a cut through the shank (`*PRE-TENSION SECTION`), the
    /// pre-tension load: the region is the element faces on one side of the cut, the value
    /// the force pulling the two sides together, or the shortening when `by_displacement`.
    /// The shortening counts from the start of the analysis, so 0 loosens the bolt; `lock`
    /// keeps the length the bolt had at the end of the previous step instead
    /// (`*BOUNDARY, FIXED`), and the value is not used.
    PreTension {
        value: f64,
        by_displacement: bool,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        lock: bool,
        /// Direction of the preload; `None` lets the solver take the surface normal.
        direction: Option<[f64; 3]>,
    },
    /// Heat flow into every node of the region (`*CFLUX`).
    ConcentratedFlux(f64),
    /// Heat flow per area into a surface (`*DFLUX`, `S`).
    SurfaceFlux(f64),
    /// Heat generated per volume in the elements of the region (`*DFLUX`, `BF`).
    BodyFlux(f64),
    /// Convection to the surroundings at the sink temperature (`*FILM`).
    Film { sink: f64, coefficient: f64 },
    /// Radiation to the surroundings at the sink temperature (`*RADIATE`); needs the
    /// physical constants of the model.
    Radiation { sink: f64, emissivity: f64 },
    /// Gravity, as the acceleration vector acting on the elements of the region
    /// (`*DLOAD`, `GRAV`). Needs the density of the materials.
    Gravity([f64; 3]),
    /// Rotation of the elements of the region about an axis through `point` along `axis`
    /// at `speed` radians per time (`*DLOAD`, `CENTRIF`).
    /// The solver takes the square of the speed; the model keeps the speed the user entered.
    Centrifugal {
        point: [f64; 3],
        axis: [f64; 3],
        speed: f64,
    },
    /// Wheel loads that travel over a surface region, one static step per position; see
    /// [`MovingLoad`].
    Moving(MovingLoad),
}

impl LoadKind {
    /// A total force, spread over the nodes of the region.
    pub fn force(value: [f64; 3]) -> Self {
        LoadKind::Force {
            value,
            each_node: false,
        }
    }

    /// What the second amplitude of a film or radiation scales, if the load has one.
    pub fn factor_amplitude_label(&self) -> Option<&'static str> {
        match self {
            LoadKind::Film { .. } => Some("Film coefficient"),
            LoadKind::Radiation { .. } => Some("Emissivity"),
            _ => None,
        }
    }

    /// Whether the load is a heat flow rather than a force.
    pub fn is_thermal(&self) -> bool {
        !matches!(
            self,
            LoadKind::Moment(_)
                | LoadKind::Pressure(_)
                | LoadKind::Force { .. }
                | LoadKind::PreTension { .. }
                | LoadKind::Gravity(_)
                | LoadKind::Centrifugal { .. }
                | LoadKind::Moving(_)
        )
    }

    /// Whether the load acts on the mass of the elements (gravity, centrifugal), so the
    /// materials need a density.
    pub fn is_body_force(&self) -> bool {
        matches!(self, LoadKind::Gravity(_) | LoadKind::Centrifugal { .. })
    }
}

/// A state of the model before the first step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct InitialCondition {
    pub name: String,
    #[serde(default = "active")]
    pub active: bool,
    pub region: Region,
    pub kind: InitialConditionKind,
    /// Distribution the value is scaled by over the region; `None` is the same value
    /// everywhere.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distribution: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum InitialConditionKind {
    /// `*INITIAL CONDITIONS, TYPE=TEMPERATURE`
    Temperature(f64),
    /// `*INITIAL CONDITIONS, TYPE=VELOCITY`: the same velocity at every node of the region,
    /// the start of a Dynamic step.
    Velocity([f64; 3]),
    /// A rigid rotation at the start of a Dynamic step: `speed` in rad/s about the axis
    /// through `point`, written as the velocity of every node of the region.
    AngularVelocity {
        point: [f64; 3],
        axis: [f64; 3],
        speed: f64,
    },
}

impl InitialConditionKind {
    /// A velocity that only a Dynamic step takes up.
    pub fn is_velocity(&self) -> bool {
        !matches!(self, Self::Temperature(_))
    }

    /// Velocity of a node at `position` under this condition; `None` for a temperature.
    pub fn velocity_at(&self, position: [f64; 3]) -> Option<[f64; 3]> {
        match self {
            Self::Temperature(_) => None,
            Self::Velocity(v) => Some(*v),
            Self::AngularVelocity { point, axis, speed } => {
                let length = axis.iter().map(|a| a * a).sum::<f64>().sqrt();
                if length == 0.0 {
                    return Some([0.0; 3]);
                }
                let n = axis.map(|a| a / length);
                let r: [f64; 3] = std::array::from_fn(|k| position[k] - point[k]);
                Some([
                    speed * (n[1] * r[2] - n[2] * r[1]),
                    speed * (n[2] * r[0] - n[0] * r[2]),
                    speed * (n[0] * r[1] - n[1] * r[0]),
                ])
            }
        }
    }

    /// Why the values cannot be written.
    pub fn problem(&self) -> Option<String> {
        match self {
            Self::AngularVelocity { axis, .. } if axis.iter().all(|a| *a == 0.0) => {
                Some("The axis must not be the zero vector.".into())
            }
            _ => None,
        }
    }
}

/// Temperatures prescribed in a step for the thermal strains of a mechanical analysis
/// (`*TEMPERATURE`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DefinedField {
    pub name: String,
    /// A deactivated field is left out of the input file.
    #[serde(default = "active")]
    pub active: bool,
    /// The nodes that take the value; a field read from a file covers all nodes of the
    /// file and ignores the region.
    pub region: Region,
    pub kind: DefinedFieldKind,
    /// Amplitude a value follows over time; `None` is CalculiX's default ramp. A field
    /// read from a file has none.
    #[serde(default)]
    pub amplitude: Option<String>,
    /// Distribution the value is scaled by over the region; `None` is the same value
    /// everywhere.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distribution: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum DefinedFieldKind {
    /// One temperature on the nodes of the region.
    Temperature(f64),
    /// The temperatures of a step of a result file, as a heat transfer analysis on the
    /// same mesh wrote it (`*TEMPERATURE, FILE=`); the step counts from 1. The solver reads
    /// the file from the working directory of the analysis, so the file is copied there
    /// when the analysis starts.
    TemperatureFromFile { file: PathBuf, step: u32 },
}

impl DefinedFieldKind {
    /// Whether the field takes its nodes from its region.
    pub fn takes_region(&self) -> bool {
        matches!(self, DefinedFieldKind::Temperature(_))
    }

    /// Whether the value can follow an amplitude.
    pub fn takes_amplitude(&self) -> bool {
        matches!(self, DefinedFieldKind::Temperature(_))
    }
}

impl FeModel {
    /// The result files the active defined fields of the active steps read, each once;
    /// The solver needs them next to the input file.
    pub fn result_files(&self) -> Vec<&Path> {
        let mut files: Vec<&Path> = (self.steps.iter().filter(|s| s.active))
            .filter(|s| s.kind.supports_defined_fields())
            .flat_map(|s| s.defined_fields.iter().filter(|f| f.active))
            .filter_map(|f| match &f.kind {
                DefinedFieldKind::TemperatureFromFile { file, .. } => Some(file.as_path()),
                DefinedFieldKind::Temperature(_) => None,
            })
            .collect();
        files.sort_unstable();
        files.dedup();
        files
    }
}

/// Items of projects saved before they could be deactivated are active.
pub(crate) fn active() -> bool {
    true
}

/// Next free default name such as "Material-2": new items are numbered per kind.
pub fn next_name<'a>(prefix: &str, existing: impl IntoIterator<Item = &'a str>) -> String {
    let existing: Vec<&str> = existing.into_iter().collect();
    (1..)
        .map(|n| format!("{prefix}-{n}"))
        .find(|name| !existing.iter().any(|e| e.eq_ignore_ascii_case(name)))
        .expect("unbounded range")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_steps_with_prescribed_temperatures_take_defined_fields() {
        let kinds = [
            (StepKind::Static(StaticStep::default()), true),
            (StepKind::Frequency(FrequencyStep::default()), true),
            (StepKind::Dynamic(DynamicStep::default()), true),
            (StepKind::Buckle(BuckleStep::default()), true),
            (StepKind::ModalDynamics(ModalDynamicsStep::default()), false),
            (
                StepKind::SteadyStateDynamics(SteadyStateDynamicsStep::default()),
                false,
            ),
            (StepKind::HeatTransfer(HeatTransferStep::default()), false),
            (
                StepKind::CoupledTempDisp(HeatTransferStep::default()),
                false,
            ),
            (
                StepKind::UncoupledTempDisp(HeatTransferStep::default()),
                false,
            ),
        ];
        for (kind, expected) in kinds {
            assert_eq!(kind.supports_defined_fields(), expected, "{kind:?}");
        }
    }

    #[test]
    fn regions_follow_renamed_parts() {
        let mut model = FeModel::default();
        model.sections.push(Section {
            name: "Section-1".into(),
            material: "Steel".into(),
            region: Region::Parts(vec!["A".into(), "B".into()]),
            thickness: 1.0,
            kind: SectionKind::Solid,
        });
        let mut step = Step::new_static("Step-1");
        step.boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Fixed-1".into(),
            active: true,
            visible: true,
            region: Region::ElementSet("A".into()),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        step.loads.push(Load {
            distribution: None,
            name: "Pressure-1".into(),
            active: true,
            visible: true,
            region: Region::Surface("A".into()),
            kind: LoadKind::Pressure(1.0),
            amplitude: None,
            factor_amplitude: None,
        });
        model.steps.push(step);
        model.rename_part("A", "C");
        assert_eq!(
            model.sections[0].region,
            Region::Parts(vec!["C".into(), "B".into()])
        );
        let step = &model.steps[0];
        assert_eq!(
            step.boundary_conditions[0].region,
            Region::ElementSet("C".into())
        );
        assert_eq!(step.loads[0].region, Region::Surface("A".into()));
        // A part merged into another leaves no duplicate.
        model.rename_part("B", "C");
        assert_eq!(model.sections[0].region, Region::Parts(vec!["C".into()]));
    }

    #[test]
    fn default_names_count_up_per_kind() {
        assert_eq!(next_name("Material", []), "Material-1");
        assert_eq!(
            next_name("Material", ["Material-1", "MATERIAL-2", "Steel"]),
            "Material-3"
        );
    }
}
