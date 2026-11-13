//! FE-Modell: Materialien, Sections, Steps, Randbedingungen, Lasten.
//!
//! The model describes an analysis on top of a mesh. Regions
//! are what the user picked in the GUI (parts, nodes, element faces) or named sets of the
//! input file; node and element sets that the solver needs for them are derived when the input
//! file is written, so the user never has to define sets by hand.

mod amplitude;
mod anisotropic;
mod bolt;
mod cad_operation;
pub mod campbell;
mod checks;
pub mod color_scale;
mod compound;
mod connection;
mod constraint;
mod contact;
mod contact_check;
mod contact_steps;
pub mod convert;
mod coordinate_system;
mod cross_section;
mod detection;
mod distribution;
mod dynamic;
mod element_type;
pub mod equation;
mod excitation;
mod exposed;
mod features;
mod field;
pub mod fingerprint;
pub mod formula;
mod geometry;
pub mod handover;
mod history;
pub mod hyperelastic;
pub mod hyperelastic_fit;
mod id;
pub mod infill;
pub mod library;
mod measure;
mod mesh_operation;
mod mesh_set;
mod moving_load;
mod number;
pub mod overview;
mod parameter;
mod part_filter;
mod plot;
mod primitive;
mod properties;
mod refs;
mod region;
mod remote;
mod remote_force;
mod section;
mod signature;
mod solver_diagnostics;
mod study;
mod symmetry;
mod tables;
pub mod units;
pub use tables::{StepTable, TableOwner};
mod validity;

pub use amplitude::{Amplitude, AmplitudeTime, ValueTable, no_tables};
pub use bolt::{Bolt, BoltAction, BoltChange, LoweredBolts, ResolvedBolt, ResolvedThread};
pub use cad_operation::{
    CadOperation, CadOperationKind, DefeatureFaces, SplitKeep, SplitPlane, tilted_normal,
};
pub use checks::{
    Finding, MeshCheck, Problem, Severity, diagnose_solver_output, diagnose_solver_warnings,
};
pub use color_scale::{ColorScale, ColorSpectrum};
pub use compound::{CompoundTie, has_compound_ties};
pub use connection::{
    BondedMethod, CONDUCTANCE_FACTOR, ContactBehavior, ContactDefaults, ContactRegion,
    ContactSettings, InterfaceTreatment, Lowered, ROUGH_FRICTION, Resolved, ResolvedContacts,
    ResolvedKind, STIFFNESS_FACTOR, SideRoles, Stiffness, TIED_STIFFNESS_FACTOR, TOLERANCE_FACTOR,
    WEAK_SPRING_FACTOR, WEAK_SPRING_STABILIZATION, conducts_across_contacts, mortar_supports,
};
pub use constraint::{
    CompressionOnly, Joint, JointAxis, JointKind, NodeTie, PointMass, RigidBody, RotaryInertia,
    Spring, SpringEnd, SpringLayout,
};
pub use contact::{
    Constraint, ContactMethod, ContactPair, ContactScale, Coupling, CyclicSymmetry, ElementYoung,
    Friction, GapConductance, InteractionProperty, NORMAL_STIFFNESS_FACTOR, STICK_SLOPE_RATIO,
    SectorDisplay, SurfaceBehavior, SurfaceInteraction, Tie, default_stick_slope,
};
pub use contact_check::{ContactCheck, ContactStatus};
pub use contact_steps::{ContactChange, ContactState};
pub use coordinate_system::{
    CoordinatePlane, CoordinateSystem, CoordinateSystemKind, Frame, RigidMotion, SystemDefinition,
    region_positions,
};
pub use cross_section::{
    BeamProfile, CrossSection, Dimension, Outline, ROUND_POINTS, RopeBending, SectionProperties,
    Sketch, triangulate, triangulate_with_holes,
};
pub use detection::{
    ContactDetection, ContactRule, DetectionFilter, DetectionGeometry, DetectionGrouping,
    DetectionReport, RuleAction, auto_detection_tolerance, auto_geometry_detection_tolerance,
};
pub use distribution::{Distribution, DistributionKind, Interpolation, Sampler, parse_points};
pub use dynamic::{Integration, RayleighFit, StableStepEstimate, StepLimit};
pub use element_type::{PartElementType, element_type_options};
pub use excitation::{Excitation, Signal, SignalKind, Sweep};
pub use exposed::{Exposed, Field, ItemRef, ValuePath};
pub use features::{
    GLOBAL, Plane, PlaneSource, PointDefinition, PointRef, ReferencePoint, ResultPath, ResultPlane,
};
pub use field::{
    FieldOutput, OutputDimension, OutputKind, output_variable_description, output_variable_field,
    output_variable_label,
};
pub use geometry::{
    Algorithm2d, Algorithm3d, CompoundPart, ExtrudeLayers, ExtrudeMesh, Geometry, MeshSetupItem,
    MeshSetupKind, MeshingParameters, Recombination, ThickenShellMesh,
};
pub use handover::Handover;
pub use history::{HistoryKind, HistoryOutput, Totals};
pub use hyperelastic::{Hyperelastic, HyperelasticModel, TestMode};
pub use id::Id;
pub use infill::{InfillPattern, InfillSection};
pub use library::MaterialLibrary;
pub use measure::{MeasureKind, MeasureRegion, POINT_AXES};
pub use mesh_operation::{MeshOperation, MeshOperationKind};
pub use mesh_set::{MeshSet, SetKind};
pub use moving_load::{AXIS_LABELS, LoadStations, MovingFrame, MovingLoad, Wheel, load_stations};
pub use number::{MathOp, apply_series, switch};
pub use overview::{Cell, OverviewGroup, OverviewRow, StepItemKey, StepOverview};
pub use parameter::{Parameter, ValueKind};
pub use part_filter::{PartContext, PartFilter, PartGroup, PartTerm, name_matches};
pub use plot::{PlotKind, histogram};
pub use primitive::{MAX_PRIMITIVE_ELEMENTS, MeshPrimitive, Param, ParamSlot, ParamValue};
pub use properties::{ModelKind, ModelProperties, ModelSpace};
pub use region::{Region, describe_entities};
pub use remote::{
    Behavior, Displacement, Formulation, RemoteBehavior, RemoteDof, RemotePoint, expanded_nodes,
    rigid_rows,
};
pub use remote_force::RemoteForce;
pub use section::{
    BeamOrientation, BeamSection, Section, SectionKind, line_tangent, membrane_type, unit_thickness,
};
pub use signature::{EntitySignature, TagMap, match_entities};
pub use solver_diagnostics::{Origins, diagnose_solver_check};
pub use study::{DesignPoint, FillMode, FillRule, PointRun, PointState, StudyColumn, StudyTable};
pub use symmetry::{BeamsOnPlane, SymmetryError, SymmetryPlane, symmetry_dofs, symmetry_plane};
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
/// refused. Format 17 adds the remote force; format 16 reads as it is. Format 18 gives the
/// items of the model an internal id ([`Id`]) and lets items refer to each other by it in
/// place of their names, so that renaming keeps the references; older formats are refused.
/// Format 19 replaces the perturbation switches of the frequency, buckle and modal dynamic
/// steps by the [`Base`] they are computed about; older formats are refused. Format 20 keeps
/// the history outputs once in the model ([`FeModel::history_outputs`]) and lets the steps
/// list those they print by id; older formats are refused. Format 21 replaces the field
/// extreme of a Measure block by the values of the field over a region, a series that a
/// Math block reduces, and adds the Plot and Mesh Preview blocks; older formats are refused.
/// Format 22 adds node groups; format 21 reads as it is. Format 23 adds the Thicken Shell
/// Mesh item of the mesh setup; formats 21 and 22 read as they are. Format 24 adds the
/// compound parts of the geometry ([`CompoundPart`]) and the bodies of mesh parts; formats 21
/// to 23 read as they are. Format 25 adds the splits of the CAD Operation block
/// ([`CadOperationKind::SplitPlane`], [`CadOperationKind::HoleRing`]); formats 21 to 24 read as
/// they are. Format 26 adds the thread of a bolt ([`Bolt::thread`]) and the amplitude of its
/// steps ([`BoltChange::amplitude`]); formats 21 to 25 read as they are. Format 27 lets contact
/// detections choose their parts by filters and give the pairs they find their type by rules
/// ([`ContactRule`]), and adds part groups ([`PartGroup`]); formats 21 to 26 read as they are.
/// Format 28 merges the concentrated, surface and body flux into one total heat flow
/// ([`LoadKind::Flux`]); formats 21 to 27 read as they are unless a step has one of those
/// loads. Format 29 adds the defeaturing of the CAD Operation block
/// ([`CadOperationKind::Defeature`]); formats 21 to 28 read as they are. Format 30 adds the
/// Transform block of the project schematic; formats 21 to 29 read as they are. Format 31
/// adds anisotropic elasticity ([`Material::anisotropic`]), the orientation of sections
/// ([`Section::orientation`]) and infill sections ([`SectionKind::Infill`]); formats 21 to
/// 30 read as they are. Format 32 gives coordinate systems a base system, a definition by
/// three points or by offset and rotation ([`SystemDefinition`]) and a selection they move
/// with ([`CoordinateSystem::attached`]); formats 21 to 31 read as they are unless they have
/// coordinate systems. Format 33 adds the Formula block of the project schematic; formats 21
/// to 32 read as they are. Format 34 adds suppressed parts of the geometry
/// ([`Geometry::suppressed`]); formats 21 to 33 read as they are. Format 35 gives a bolt its
/// surface instead of the two sides of a cut ([`Bolt::surface`]); formats 21 to 34 read as
/// they are unless they have bolts. Format 36 lets a File block keep its file in the project
/// (embedded); formats 21 to 35 read as they are. Format 37 drops the preload of a bolt: its
/// steps set it ([`BoltChange`]), an adjustment adds to the length at the end of the previous
/// step; formats 21 to 36 read as they are unless they have bolts. Format 38 adds point
/// masses ([`PointMass`]); formats 21 to 37 read as they are. Format 39 gives the project one
/// unit system its schematic shows values in, in place of one for each block; formats 21 to
/// 38 read as they are, in the default units.
pub const PROJECT_FORMAT: u32 = 39;

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
    /// Named sets of parts, chosen by filters, that contact detections refer to.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub part_groups: Vec<PartGroup>,
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
    /// Values printed into the `.dat` file, each once for the model; the steps list those
    /// they print ([`Step::history_outputs`]).
    #[serde(default)]
    pub history_outputs: Vec<HistoryOutput>,
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
    /// Changes of the mesh parts applied after meshing, such as transforms, in order; the
    /// mesh has them applied.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub mesh_operations: Vec<MeshOperation>,
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
            .chain(self.history_outputs.iter().filter_map(|h| h.kind.region()))
            .chain(self.steps.iter().flat_map(|step| {
                (step.boundary_conditions.iter().map(|b| &b.region))
                    .chain(step.loads.iter().map(|l| &l.region))
                    .chain(step.kind.region())
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
            .chain((self.history_outputs.iter_mut()).filter_map(|h| h.kind.region_mut()))
            .chain(self.steps.iter_mut().flat_map(|step| {
                (step.boundary_conditions.iter_mut().map(|b| &mut b.region))
                    .chain(step.loads.iter_mut().map(|l| &mut l.region))
                    .chain(step.defined_fields.iter_mut().map(|f| &mut f.region))
                    .chain(step.kind.region_mut())
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

    /// The profile of a beam section's cross-section; `None` for other sections or a
    /// cross-section the model lacks.
    pub fn section_profile(&self, kind: &SectionKind) -> Option<&BeamProfile> {
        match kind {
            SectionKind::Beam(beam) => Some(&self.cross_section(beam.cross_section?)?.profile),
            _ => None,
        }
    }

    /// The axes (rows, global coordinates) of a section's orientation: the material axes of
    /// a solid section, the print axes of an infill section. `None` without an orientation
    /// (the global axes); `Err` when its coordinate system is missing or not rectangular.
    pub fn section_axes(&self, section: &Section) -> Result<Option<[[f64; 3]; 3]>, String> {
        let Some(id) = section.orientation else {
            return Ok(None);
        };
        let Some(system) = self.coordinate_system_or_global(Some(id)) else {
            return Err("The coordinate system of the orientation does not exist".into());
        };
        if system.kind != CoordinateSystemKind::Rectangular {
            return Err(format!(
                "The orientation {} is cylindrical; material axes take a rectangular \
                 coordinate system",
                system.name
            ));
        }
        system.axes().map(Some)
    }

    /// The distributions some item uses.
    pub fn used_distributions(&self) -> std::collections::BTreeSet<Id> {
        let initial = self.initial_conditions.iter().map(|c| &c.distribution);
        let steps = self.steps.iter().flat_map(|step| {
            (step.boundary_conditions.iter().map(|b| &b.distribution))
                .chain(step.loads.iter().map(|l| &l.distribution))
                .chain(step.defined_fields.iter().map(|f| &f.distribution))
        });
        initial.chain(steps).filter_map(|d| *d).collect()
    }

    /// Follows a renamed part: regions on the part, or on the element set an input file
    /// defines for it, keep pointing at it.
    pub fn rename_part(&mut self, old: &str, new: &str) {
        self.rename_element_types(old, new);
        self.rename_mesh_operation_parts(old, new);
        self.replace_part_in_regions(old, &[new.to_owned()]);
        self.rename_part_in_filters(old, new);
    }

    /// Part filters naming the part `old` name the part `new`.
    fn rename_part_in_filters(&mut self, old: &str, new: &str) {
        for detection in &mut self.contact_detections {
            detection.rename_part(old, new);
        }
        for group in &mut self.part_groups {
            group.filter.rename_part(old, new);
        }
    }

    /// Follows a part split into several, as a dissolved compound part into the parts it
    /// joined: what was on the part is on each of them.
    pub fn split_part(&mut self, old: &str, new: &[String]) {
        let Some((first, rest)) = new.split_first() else {
            return;
        };
        let choices: Vec<PartElementType> = (self.element_types.iter())
            .filter(|c| c.part == old)
            .cloned()
            .collect();
        for name in rest {
            for choice in &choices {
                if !self.element_types.iter().any(|c| c.part == *name) {
                    self.element_types.push(PartElementType {
                        part: name.clone(),
                        ..choice.clone()
                    });
                }
            }
        }
        self.rename_element_types(old, first);
        self.split_part_in_filters(old, new);
        for operation in &mut self.mesh_operations {
            if let Some(at) = operation.parts.iter().position(|n| n == old) {
                operation.parts.splice(at..=at, new.iter().cloned());
                let mut seen = std::collections::BTreeSet::new();
                operation.parts.retain(|name| seen.insert(name.clone()));
            }
        }
        self.replace_part_in_regions(old, new);
    }

    /// Parts deleted from the mesh leave the regions that picked them: the region keeps
    /// the other parts, and nothing is reported for the deleted ones.
    pub fn remove_parts_from_regions(&mut self, deleted: &[String]) {
        let drop = |region: &mut Region| {
            if let Region::Parts(names) = region {
                names.retain(|name| !deleted.contains(name));
            }
        };
        self.regions_mut().for_each(drop);
        (self.initial_conditions.iter_mut()).for_each(|i| drop(&mut i.region));
    }

    /// Regions on the part `old`, or on the element set of its name, are on the parts `new`.
    fn replace_part_in_regions(&mut self, old: &str, new: &[String]) {
        let replace = |region: &mut Region| match region {
            Region::Parts(names) => {
                if let Some(at) = names.iter().position(|n| n == old) {
                    names.splice(at..=at, new.iter().cloned());
                }
                // Merged parts leave the same name twice.
                let mut seen = std::collections::BTreeSet::new();
                names.retain(|name| seen.insert(name.clone()));
            }
            Region::ElementSet(name) if name == old && new.len() == 1 => {
                *name = new[0].clone();
            }
            _ => {}
        };
        self.regions_mut().for_each(replace);
        (self.initial_conditions.iter_mut()).for_each(|i| replace(&mut i.region));
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Material {
    /// The internal identity other items refer to; the user sees only the name. The user's
    /// material library, whose materials nothing refers to, may hold materials without one:
    /// they get a new one when read, and one copied into a model a new one again.
    #[serde(default)]
    pub id: crate::Id,
    pub name: String,
    pub density: Option<f64>,
    pub elastic: Option<Elastic>,
    /// Hyperelasticity: a strain energy model for large strains in place of `elastic`.
    #[serde(default)]
    pub hyperelastic: Option<Hyperelastic>,
    /// Anisotropic linear elasticity in place of `elastic`, in the material axes 1, 2, 3
    /// that a section's orientation puts into the model (the global x, y, z without one).
    #[serde(default)]
    pub anisotropic: Option<Anisotropic>,
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

impl Material {
    /// The elastic constants at small strains: the linear elasticity, or the initial moduli
    /// of a hyperelastic material.
    pub fn small_strain_elastic(&self) -> Option<Elastic> {
        match (&self.hyperelastic, &self.anisotropic) {
            (Some(hyper), _) => Some(hyper.small_strain()),
            (None, Some(anisotropic)) => anisotropic.isotropic_equivalent(),
            (None, None) => self.elastic,
        }
    }

    /// Whether the material has any elasticity: linear, anisotropic or hyperelastic.
    pub fn has_elasticity(&self) -> bool {
        self.elastic.is_some() || self.hyperelastic.is_some() || self.anisotropic.is_some()
    }
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

pub use anisotropic::{
    Anisotropic, EngineeringConstants, GENERAL_COMPONENTS, from_matrix as anisotropic_upper,
    upper_index,
};

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
    /// The history outputs of the model the step prints, by id, in their order.
    #[serde(default)]
    pub history_outputs: Vec<Id>,
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
    /// (`*COMPLEX FREQUENCY`), from eigenmodes it computes for itself.
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
    /// Harmonic response over a frequency range from eigenmodes it computes for itself
    /// (`*STEADY STATE DYNAMICS`).
    SteadyStateDynamics(SteadyStateDynamicsStep),
    /// How strongly mass, stiffness or an eigenfrequency react to moving the surface nodes
    /// of a design region (`*SENSITIVITY`), from the state of the step before it.
    Sensitivity(SensitivityStep),
}

impl StepKind {
    /// How long the step lasts in step time: the time period of static, heat transfer,
    /// visco steps, the duration of a dynamic one; 1 for the others.
    pub fn period(&self) -> f64 {
        match self {
            StepKind::Static(settings) => settings.period(),
            StepKind::HeatTransfer(settings)
            | StepKind::CoupledTempDisp(settings)
            | StepKind::UncoupledTempDisp(settings) => settings.increments.period(),
            StepKind::Visco(settings) => settings.increments.period(),
            StepKind::Dynamic(settings) => settings.duration,
            _ => 1.0,
        }
    }

    /// Whether the step is computed with large deformations (`NLGEOM`).
    pub fn nlgeom(&self) -> bool {
        match self {
            StepKind::Static(settings) => settings.nlgeom,
            StepKind::CoupledTempDisp(settings) => settings.increments.nlgeom,
            StepKind::Visco(settings) => settings.increments.nlgeom,
            StepKind::Dynamic(settings) => settings.large_deformations,
            _ => false,
        }
    }

    /// Whether the step superposes eigenmodes kept before it (by its own eigenmode step or
    /// an Eigenmodes link): it keeps their supports.
    pub fn uses_stored_modes(&self) -> bool {
        matches!(self, StepKind::SteadyStateDynamics(_))
    }

    /// Whether the step takes loads. A frequency step has none; preloads
    /// come from the steps before with [`FrequencyStep::base`].
    pub fn supports_loads(&self) -> bool {
        !matches!(
            self,
            StepKind::Frequency(_) | StepKind::ComplexFrequency(_) | StepKind::Sensitivity(_)
        )
    }

    /// The region the step itself acts on: the design region of a sensitivity step, the
    /// rotating elements of a complex frequency step.
    pub fn region(&self) -> Option<&Region> {
        match self {
            StepKind::Sensitivity(settings) => Some(&settings.design_region),
            StepKind::ComplexFrequency(settings) => Some(&settings.region),
            _ => None,
        }
    }

    pub fn region_mut(&mut self) -> Option<&mut Region> {
        match self {
            StepKind::Sensitivity(settings) => Some(&mut settings.design_region),
            StepKind::ComplexFrequency(settings) => Some(&mut settings.region),
            _ => None,
        }
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

    /// Whether the step is an eigenvalue problem (frequency, complex frequency or
    /// buckling): its results are the mode shapes with their eigenvalues, kept once and
    /// normalized, and it takes no field outputs.
    pub fn computes_eigenmodes(&self) -> bool {
        matches!(
            self,
            StepKind::Frequency(_) | StepKind::ComplexFrequency(_) | StepKind::Buckle(_)
        )
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

    /// Whether the step prints history outputs: an eigenvalue step prints one row per
    /// mode, a sensitivity step evaluates the step before it and prints none.
    pub fn prints_history(&self) -> bool {
        !matches!(self, StepKind::Sensitivity(_))
    }

    /// Whether a history output of this kind prints anything in the step: only the time
    /// integration of a Dynamic step computes the energies.
    pub fn supports_history_output(&self, kind: &crate::HistoryKind) -> bool {
        if !self.prints_history() {
            false
        } else if *kind == crate::HistoryKind::Energies {
            matches!(self, StepKind::Dynamic(_))
        } else {
            true
        }
    }

    /// Whether a load of this kind acts in the step.
    pub fn supports_load(&self, kind: &LoadKind) -> bool {
        // Each position of a moving load is a static step of its own.
        if matches!(kind, LoadKind::Moving(_)) {
            return matches!(self, StepKind::Static(_));
        }
        // An excitation shakes the structure over time, which only a dynamic step follows.
        if matches!(kind, LoadKind::Excitation(_)) {
            return matches!(self, StepKind::Dynamic(_));
        }
        self.supports_loads()
            && if kind.is_thermal() {
                self.is_thermal()
            } else {
                self.is_mechanical()
            }
    }
}

/// How a step solves its systems of equations.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum LinearSolverChoice {
    /// The solver chooses: iterative for large models of solids (translations only) in
    /// static and heat transfer steps, otherwise direct while the factor fits into the
    /// memory and iterative beyond; the other one takes over when the first fails.
    #[default]
    Automatic,
    /// A sparse direct factorization (LDLᵀ): any system, memory for the factor.
    Direct,
    /// Conjugate gradients with an algebraic multigrid preconditioner: symmetric positive
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

/// Settings of a static step (also the increments of heat transfer and visco steps): how
/// long it lasts, how many result frames it keeps and whether it follows large
/// deformations. How the solver divides the duration into increments is derived from these
/// (see [`StaticStep::plan`]); [`IncrementControl`] overrides it where needed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct StaticStep {
    /// Length of the step in (pseudo) time; loads ramp up over it.
    pub duration: f64,
    /// Result frames kept over the duration, evenly spaced, the last one at the end. The
    /// increments are shortened where needed so that they end on the frames.
    pub frames: u32,
    /// Large displacements and rotations (`NLGEOM`).
    pub nlgeom: bool,
    /// How the increments are chosen; automatic by default.
    pub control: IncrementControl,
    pub solver: LinearSolverChoice,
}

impl Default for StaticStep {
    fn default() -> Self {
        Self {
            duration: 1.0,
            frames: 1,
            nlgeom: false,
            control: IncrementControl::default(),
            solver: LinearSolverChoice::Automatic,
        }
    }
}

/// The advanced increment settings of a [`StaticStep`]: everything `None` is derived from
/// the duration and the frames.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct IncrementControl {
    pub size: IncrementSize,
    /// Size of the first increment (automatic increments).
    pub initial: Option<f64>,
    /// Smallest increment a cutback may reach before the step fails (automatic increments).
    pub minimum: Option<f64>,
    /// Largest increment (automatic increments).
    pub maximum: Option<f64>,
    /// Most increments the step may take; `None` sets no limit.
    pub limit: Option<u32>,
}

/// Whether the solver chooses the increment sizes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum IncrementSize {
    /// Grown after easy increments, cut back after failed ones.
    #[default]
    Automatic,
    /// Always this size (shortened to end on the frames); a failed increment fails the step.
    Fixed(f64),
}

/// The increments a [`StaticStep`] asks of the solver, with every automatic value resolved.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IncrementPlan {
    pub duration: f64,
    pub frames: u32,
    /// The first increment; with fixed increments, every one.
    pub initial: f64,
    pub minimum: f64,
    pub maximum: f64,
    pub fixed: bool,
    /// `None`: no limit.
    pub limit: Option<u32>,
}

impl StaticStep {
    /// The most frames a step may keep.
    pub const MAX_FRAMES: u32 = 100_000;
    /// A step with large deformations starts with at most this share of its duration.
    pub const NONLINEAR_START: f64 = 0.1;

    /// The length of the step in time (its duration).
    pub fn period(&self) -> f64 {
        self.duration
    }

    /// The time between two result frames.
    pub fn frame_interval(&self) -> f64 {
        self.duration / f64::from(self.frames.max(1))
    }

    /// The increments of the step, `nlgeom` saying whether it is solved with large
    /// deformations (it may be switched on by the materials). Automatically, the first
    /// increment is a frame interval, with large deformations at most a tenth of the
    /// duration; the smallest one 1e-5 of the duration, the largest one the duration. The
    /// first increment never exceeds the largest one.
    pub fn plan(&self, nlgeom: bool) -> IncrementPlan {
        let duration = self.duration;
        let c = &self.control;
        let maximum = c.maximum.unwrap_or(duration).min(duration);
        let (initial, fixed) = match c.size {
            IncrementSize::Fixed(dt) => (dt.min(duration), true),
            IncrementSize::Automatic => {
                let automatic = if nlgeom {
                    self.frame_interval().min(Self::NONLINEAR_START * duration)
                } else {
                    self.frame_interval()
                };
                (c.initial.unwrap_or(automatic).min(maximum), false)
            }
        };
        let minimum = c.minimum.unwrap_or(1e-5 * duration).min(initial);
        IncrementPlan {
            duration,
            frames: self.frames.max(1),
            initial,
            minimum,
            maximum,
            fixed,
            limit: c.limit,
        }
    }

    /// What is wrong with the settings, if anything.
    pub fn problem(&self) -> Option<String> {
        let positive = |v: f64| v.is_finite() && v > 0.0;
        if !positive(self.duration) {
            return Some("The duration must be greater than 0.".into());
        }
        if !(1..=Self::MAX_FRAMES).contains(&self.frames) {
            return Some(format!(
                "The number of frames must lie between 1 and {}.",
                Self::MAX_FRAMES
            ));
        }
        let c = &self.control;
        if let IncrementSize::Fixed(dt) = c.size
            && !(positive(dt) && dt <= self.duration)
        {
            return Some(
                "A fixed increment must be greater than 0 and no longer than the duration.".into(),
            );
        }
        for (value, name) in [
            (c.initial, "initial"),
            (c.minimum, "smallest"),
            (c.maximum, "largest"),
        ] {
            if value.is_some_and(|v| !positive(v)) {
                return Some(format!("The {name} increment must be greater than 0."));
            }
        }
        if let (Some(min), Some(max)) = (c.minimum, c.maximum)
            && min > max
        {
            return Some("The smallest increment must not exceed the largest one.".into());
        }
        if c.limit == Some(0) {
            return Some("The increment limit must be at least 1.".into());
        }
        None
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

/// What a step that probes the model (eigenmodes, buckling factors) is computed about.
/// Such a step leaves the state as it found it: the step after it starts from where the
/// steps before it ended.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Base {
    /// The unloaded model: its linear stiffness, without stresses or deformation.
    #[default]
    Unloaded,
    /// The state the steps before leave: their deformation, the stiffening by their
    /// stresses and their contact status.
    PreviousState,
}

impl Base {
    pub const ALL: [Base; 2] = [Base::Unloaded, Base::PreviousState];

    /// Whether the step is computed about a preloaded state (`*STEP, PERTURBATION`).
    pub fn is_preloaded(self) -> bool {
        self == Base::PreviousState
    }
}

/// Settings of a `*FREQUENCY` step, with the default values.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FrequencyStep {
    /// Number of eigenfrequencies to compute, counted from the lower bound (modes below it,
    /// rigid body modes say, are not among them). Unused with an upper bound.
    pub num_frequencies: u32,
    /// Lower bound of the frequency range in cycles per time unit; `None` is 0.
    pub lower_frequency: Option<f64>,
    /// Upper bound of the frequency range: every mode between the bounds is computed,
    /// whatever [`Self::num_frequencies`] says; `None` leaves it open.
    pub upper_frequency: Option<f64>,
    /// Writes eigenvalues, mode shapes and the mass and stiffness matrices to the `.eig`
    /// file (`STORAGE=YES`).
    pub storage: bool,
    /// What the eigenmodes are computed about: the unloaded model, or the state the steps
    /// before leave with the stiffening by its stresses (`*STEP, PERTURBATION`).
    pub base: Base,
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

/// Settings of a Dynamic step (direct time integration): what the user decides, how long
/// the step lasts, how many result frames it keeps and how the structure is damped. The
/// time step, the numerical damping of the integration and the Rayleigh coefficients are
/// derived from it when the model is lowered for the solver.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DynamicStep {
    /// How the response is computed: direct integration of all equations, or the
    /// superposition of a number of eigenmodes (linear models).
    #[serde(default)]
    pub method: DynamicMethod,
    /// Length of the step in time.
    pub duration: f64,
    /// Result frames written over the duration, evenly spaced, the last one at the end.
    pub frames: u32,
    /// Also a frame whenever a contact closes or opens and at the sub-steps of an impact,
    /// between the evenly spaced frames (direct integration only).
    #[serde(default = "default_true")]
    pub event_frames: bool,
    pub damping: Damping,
    pub time_step: TimeStep,
    /// Highest frequency the integration must resolve: the time step is at most
    /// `1 / (20 f)`. `None` leaves it to the frames and the amplitudes.
    pub frequency_limit: Option<f64>,
    pub numerical_damping: NumericalDamping,
    pub solver: LinearSolverChoice,
    /// Large displacements and rotations (`NLGEOM`): Newton iterations in every time step,
    /// rigid bodies turn by finite angles. Off, the motion is linear (small deformations).
    #[serde(default)]
    pub large_deformations: bool,
    /// Explicit integration: the stable time step mass scaling raises the elements to
    /// (they get the mass that needs); `None` scales nothing.
    #[serde(default)]
    pub mass_scaling: Option<f64>,
    /// Automatic method: implicit time steps shorter than this multiple of the stable
    /// explicit time step go on explicitly; `None` takes the default
    /// ([`DynamicStep::DEFAULT_SWITCH_FACTOR`]), 0 never switches.
    #[serde(default)]
    pub switch_factor: Option<f64>,
    /// Explicit integration: expanded shells keep the whole inertia of their thickness
    /// modes. Off (the default), the solver raises it so that the thickness does not set
    /// the time step (selective mass scaling; translations unchanged).
    #[serde(default)]
    pub full_shell_inertia: bool,
}

fn default_true() -> bool {
    true
}

impl Default for DynamicStep {
    fn default() -> Self {
        Self {
            method: DynamicMethod::Direct,
            duration: 1.0,
            frames: 200,
            event_frames: true,
            damping: Damping::None,
            time_step: TimeStep::Automatic,
            frequency_limit: None,
            numerical_damping: NumericalDamping::Normal,
            solver: LinearSolverChoice::Automatic,
            large_deformations: false,
            mass_scaling: None,
            switch_factor: None,
            full_shell_inertia: false,
        }
    }
}

impl DynamicStep {
    /// The most frames a step may keep.
    pub const MAX_FRAMES: u32 = 100_000;

    /// The switch factor of the Automatic method when none is given: an implicit time step
    /// costs about as much as 30 to 100 explicit ones.
    pub const DEFAULT_SWITCH_FACTOR: f64 = 30.0;

    /// What is wrong with the settings, if anything.
    pub fn problem(&self) -> Option<String> {
        if self.duration.is_nan() || self.duration <= 0.0 {
            return Some("The duration must be greater than 0.".into());
        }
        if !(1..=Self::MAX_FRAMES).contains(&self.frames) {
            return Some(format!(
                "The number of frames must lie between 1 and {}.",
                Self::MAX_FRAMES
            ));
        }
        if let TimeStep::Fixed(dt) = self.time_step
            && !(dt > 0.0 && dt <= self.duration)
        {
            return Some(
                "A fixed time step must be greater than 0 and no longer than the duration.".into(),
            );
        }
        if let Some(t) = self.mass_scaling
            && !(t > 0.0 && t <= self.duration)
        {
            return Some(
                "The mass scaling time step must be greater than 0 and no longer than the \
                 duration."
                    .into(),
            );
        }
        if let Some(k) = self.switch_factor
            && !(k >= 0.0 && k.is_finite())
        {
            return Some("The switch factor must be 0 or greater.".into());
        }
        if let Some(f) = self.frequency_limit
            && (f.is_nan() || f <= 0.0)
        {
            return Some("The frequency limit must be greater than 0.".into());
        }
        if let DynamicMethod::Modal { modes, .. } = self.method {
            if modes == 0 {
                return Some("Modal superposition needs at least one mode.".into());
            }
            if self.large_deformations {
                return Some(
                    "Modal superposition is linear: switch off Large deformations or use \
                     Direct integration."
                        .into(),
                );
            }
        }
        match self.damping {
            Damping::None => None,
            Damping::Ratio { zeta, range } => {
                if !(zeta > 0.0 && zeta <= 1.0) {
                    return Some("The damping ratio must lie between 0 and 1 (100 %).".into());
                }
                match range {
                    DampingRange::Between(f1, f2) if !(f1 > 0.0 && f2 > f1) => Some(
                        "The damping range needs 0 < lower frequency < upper frequency.".into(),
                    ),
                    _ => None,
                }
            }
            Damping::Rayleigh(r) if r.alpha < 0.0 || r.beta < 0.0 => {
                Some("The damping coefficients cannot be negative.".into())
            }
            Damping::Rayleigh(_) => None,
        }
    }

    /// Time between two result frames.
    pub fn frame_interval(&self) -> f64 {
        self.duration / self.frames.max(1) as f64
    }
}

/// How a Dynamic step computes the response over time.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum DynamicMethod {
    /// Implicit direct integration of the equation of motion (HHT-alpha), unconditionally
    /// stable; takes contact, large deformations and plasticity.
    #[default]
    Direct,
    /// Superposition of the lowest `modes` eigenmodes, which the solver computes first:
    /// every time step then costs almost nothing, so fine steps and long durations of
    /// linear models (no contact, no large deformations) come cheap. The response is
    /// exact in the modes it has; the modes must reach above the frequencies the loads
    /// excite.
    Modal {
        modes: u32,
        /// What the modes are computed about: the unloaded model, or the state the steps
        /// before leave with its stress stiffening (a perturbation of a preload).
        #[serde(default)]
        base: Base,
    },
    /// Explicit integration (central differences, lumped mass): many small time steps
    /// below the stability limit of the mesh, no equation system. For impacts, crashes and
    /// drop tests with buckling and plasticity.
    Explicit,
    /// Implicit while the motion is smooth, explicit through impacts and wherever the
    /// implicit time steps would have to become tiny; the solver switches within the step.
    Automatic,
}

impl DynamicMethod {
    /// The number of modes a new modal step superposes.
    pub const DEFAULT_MODES: u32 = 30;

    pub fn label(&self) -> &'static str {
        match self {
            DynamicMethod::Direct => "Implicit (direct integration)",
            DynamicMethod::Modal { .. } => "Modal superposition",
            DynamicMethod::Explicit => "Explicit",
            DynamicMethod::Automatic => "Automatic (implicit/explicit)",
        }
    }

    /// What the method is for, as the dialog explains it.
    pub fn hint(&self) -> &'static str {
        match self {
            DynamicMethod::Direct => {
                "Large implicit time steps with an equation system each: smooth motions, \
                 vibrations, slow drop tests. Takes contact, large deformations and plasticity."
            }
            DynamicMethod::Modal { .. } => {
                "Superposes the lowest eigenmodes: fine steps and long durations of linear \
                 models come cheap."
            }
            DynamicMethod::Explicit => {
                "Thousands of tiny time steps without equation solving: impacts, crashes, drop \
                 tests with buckling and plasticity. Each step must be shorter than a wave \
                 needs to cross the smallest element; the estimate is shown below."
            }
            DynamicMethod::Automatic => {
                "Implicit while the motion is smooth, explicit through impacts and wherever the \
                 implicit time steps would have to become tiny: a drop, a hit, then the slow \
                 motion afterwards. The output and the solver log show where it switched."
            }
        }
    }

    /// Whether the method integrates explicitly.
    pub fn is_explicit(&self) -> bool {
        matches!(self, DynamicMethod::Explicit)
    }

    /// Whether the method integrates explicitly at least in parts of the step.
    pub fn uses_explicit(&self) -> bool {
        matches!(self, DynamicMethod::Explicit | DynamicMethod::Automatic)
    }
}

/// The time step of a Dynamic step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum TimeStep {
    /// Fine enough for the frames, the amplitudes of the loads and the frequency limit.
    #[default]
    Automatic,
    /// At most this long; shortened so the frames fall on time steps.
    Fixed(f64),
}

/// The structural damping of a Dynamic step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum Damping {
    /// Undamped; only the numerical damping of the integration removes energy.
    #[default]
    None,
    /// Viscous damping ratio (1 = critical), applied as Rayleigh damping that has exactly
    /// this ratio at both ends of `range` and less between them.
    Ratio { zeta: f64, range: DampingRange },
    /// Mass and stiffness proportional coefficients given directly.
    Rayleigh(RayleighDamping),
}

/// The frequencies at which a damping ratio is met exactly.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum DampingRange {
    /// From the lowest eigenfrequency of the model to the highest frequency the time step
    /// resolves.
    #[default]
    Automatic,
    /// Between two frequencies, in cycles per time unit.
    Between(f64, f64),
}

/// The numerical damping of the time integration: it removes the high frequencies the time
/// step cannot resolve.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum NumericalDamping {
    /// Slight (HHT alpha -0.05).
    #[default]
    Normal,
    /// None (alpha 0, the trapezoidal rule): energy is conserved.
    None,
    /// The most the scheme allows (alpha -1/3).
    Strong,
}

impl NumericalDamping {
    pub const ALL: [NumericalDamping; 3] = [
        NumericalDamping::Normal,
        NumericalDamping::None,
        NumericalDamping::Strong,
    ];

    /// The alpha of the Hilber-Hughes-Taylor integration.
    pub fn alpha(self) -> f64 {
        match self {
            NumericalDamping::Normal => -0.05,
            NumericalDamping::None => 0.0,
            NumericalDamping::Strong => -1.0 / 3.0,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            NumericalDamping::Normal => "Normal",
            NumericalDamping::None => "None",
            NumericalDamping::Strong => "Strong",
        }
    }
}

/// Settings of a `*STEADY STATE DYNAMICS` step, with the default values: the harmonic
/// response over a frequency range, superposed from eigenmodes the step computes for itself
/// (an eigenmode step before it in the solver's input, about its [`Base`]), or from those an
/// Eigenmodes link of another analysis gives it.
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
    /// Number of eigenmodes the response is superposed from.
    #[serde(default = "default_modes")]
    pub modes: u32,
    /// What the eigenmodes are computed about.
    #[serde(default)]
    pub base: Base,
}

fn default_modes() -> u32 {
    SteadyStateDynamicsStep::DEFAULT_MODES
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
            modes: Self::DEFAULT_MODES,
            base: Base::Unloaded,
        }
    }
}

impl SteadyStateDynamicsStep {
    /// The number of modes a new step superposes.
    pub const DEFAULT_MODES: u32 = 20;

    /// What is wrong with the settings, if anything.
    pub fn problem(&self) -> Option<String> {
        if self.modes == 0 {
            return Some("At least one eigenmode is needed.".into());
        }
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

/// Damping of the modes of a steady state dynamics step
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
            base: Base::Unloaded,
            nodal_diameters: None,
            solver: LinearSolverChoice::Automatic,
        }
    }
}

/// Settings of a complex frequency step: the eigenmodes of a rotating structure with the
/// Coriolis forces of the rotation. The step is complete on its own: it holds the rotation
/// and the solver gets it as up to three steps (see `plx_io::inp`), a static preload by the
/// centrifugal forces (with [`ComplexFrequencyStep::stiffening`]), a frequency step that
/// stores [`ComplexFrequencyStep::num_modes`] eigenmodes (a perturbation of the preload
/// with stiffening) and the complex frequency step on those modes. Only the complex modes
/// are kept as results. The rotation acts on [`ComplexFrequencyStep::region`]; the rest of
/// the model stands still.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ComplexFrequencyStep {
    /// Number of complex eigenfrequencies to compute.
    pub num_frequencies: u32,
    /// Coriolis forces of the rotation (`CORIOLIS`); the usual reason for the step.
    pub coriolis: bool,
    /// Rotational speed in radians per time.
    pub speed: f64,
    /// A point on the rotation axis.
    pub point: [f64; 3],
    /// Direction of the rotation axis.
    pub axis: [f64; 3],
    /// A coordinate system whose z axis is the rotation axis, through its origin; `None`
    /// takes [`ComplexFrequencyStep::point`] and [`ComplexFrequencyStep::axis`].
    #[serde(default)]
    pub system: Option<Id>,
    /// The elements that rotate (parts or an element set); no parts picked: the whole
    /// model. Centrifugal forces, stress stiffening, spin softening and Coriolis forces act
    /// on these elements only.
    #[serde(default = "whole_model")]
    pub region: Region,
    /// Stress stiffening and spin softening by the centrifugal forces: the eigenmodes are
    /// those of the structure preloaded by the rotation. Off, they are those at standstill
    /// (as some other programs compute them).
    pub stiffening: bool,
    /// What the step is computed about: the unloaded model with only its rotation, or the
    /// state the steps before leave with the rotation added (a bolt preload stiffening the
    /// rotor as well).
    #[serde(default)]
    pub base: Base,
    /// Number of eigenmodes the complex modes are combined from; at least
    /// [`ComplexFrequencyStep::num_frequencies`].
    pub num_modes: u32,
    pub solver: LinearSolverChoice,
}

impl Default for ComplexFrequencyStep {
    fn default() -> Self {
        Self {
            num_frequencies: 10,
            coriolis: true,
            speed: 0.0,
            point: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            system: None,
            region: whole_model(),
            stiffening: true,
            base: Base::Unloaded,
            num_modes: 20,
            solver: LinearSolverChoice::Automatic,
        }
    }
}

/// No parts picked: the region of a step that acts on the whole model.
fn whole_model() -> Region {
    Region::Parts(Vec::new())
}

impl ComplexFrequencyStep {
    /// Whether the whole model rotates.
    pub fn rotates_whole_model(&self) -> bool {
        matches!(&self.region, Region::Parts(parts) if parts.is_empty())
    }

    /// The rotation axis in `fe`: a point on it and its unit direction, from the coordinate
    /// system when the step names one.
    pub fn axis_in(&self, fe: &FeModel) -> Result<([f64; 3], [f64; 3]), String> {
        let (point, axis) = match self.system {
            Some(id) => {
                let system = fe
                    .coordinate_system_or_global(Some(id))
                    .ok_or("The coordinate system does not exist")?;
                (system.origin, system.axes()?[2])
            }
            None => (self.point, self.axis),
        };
        let length = axis.iter().map(|c| c * c).sum::<f64>().sqrt();
        if length == 0.0 || !length.is_finite() {
            return Err("The rotation axis has no direction.".into());
        }
        Ok((point, axis.map(|c| c / length)))
    }

    /// What keeps the settings from being solved, in words.
    pub fn problem(&self) -> Option<String> {
        if self.system.is_none() {
            let length = self.axis.iter().map(|c| c * c).sum::<f64>().sqrt();
            if length == 0.0 || !length.is_finite() {
                return Some("The rotation axis has no direction.".into());
            }
        }
        if self.num_frequencies == 0 {
            return Some("Compute at least one complex frequency.".into());
        }
        if self.num_modes < self.num_frequencies {
            return Some(
                "The number of eigenmodes must be at least the number of complex frequencies."
                    .into(),
            );
        }
        None
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
    /// What the buckling loads add to: the unloaded model, or the state the steps before
    /// leave, e.g. a preload that is not scaled by the buckling factor
    /// (`*STEP, PERTURBATION`).
    pub base: Base,
    /// How the reference solution and the systems of the eigenvalue iteration are solved.
    #[serde(default)]
    pub solver: LinearSolverChoice,
}

impl Default for BuckleStep {
    fn default() -> Self {
        Self {
            num_factors: 1,
            accuracy: 1e-4,
            base: Base::Unloaded,
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
            // Creep starts fast: small first increments that grow while the creep strain
            // tolerance allows.
            increments: StaticStep {
                duration: 1000.0,
                frames: 10,
                control: IncrementControl {
                    initial: Some(1.0),
                    ..IncrementControl::default()
                },
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

    /// A complex frequency step; its results are the mode shapes, without field outputs.
    pub fn new_complex_frequency(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            kind: StepKind::ComplexFrequency(ComplexFrequencyStep::default()),
            boundary_conditions: Vec::new(),
            loads: Vec::new(),
            history_outputs: Vec::new(),
            field_outputs: Vec::new(),
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

    /// A frequency step; its results are the mode shapes, without field outputs.
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
            field_outputs: Vec::new(),
        }
    }

    /// A buckle step; its results are the mode shapes, without field outputs.
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
            field_outputs: Vec::new(),
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

    /// A steady state dynamics step with the default field outputs.
    pub fn new_steady_state_dynamics(name: impl Into<String>) -> Self {
        Self {
            kind: StepKind::SteadyStateDynamics(SteadyStateDynamicsStep::default()),
            field_outputs: FieldOutput::defaults(),
            history_outputs: Vec::new(),
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
    /// The internal identity other items refer to; the user sees only the name.
    pub id: crate::Id,
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
    pub amplitude: Option<Id>,
    /// Distribution the value is scaled by over the region; `None` is the same value
    /// everywhere.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distribution: Option<Id>,
}

// The tables of a displacement make it larger than the other kinds; there are few
// boundary conditions, so boxing would only add indirection.
#[allow(clippy::large_enum_variant)]
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
        /// Beams lying on the plane are cut in half by it: the solver gets half their
        /// section (area, second moments, torsion and so mass), see
        /// [`FeModel::halved_beams`]. Loads on them the user halves.
        #[serde(default = "crate::active")]
        halve_beams: bool,
    },
}

impl BoundaryCondition {
    /// An active, visible boundary condition without amplitude or distribution, with an
    /// id of its own.
    pub fn new(name: impl Into<String>, region: Region, kind: BoundaryKind) -> Self {
        Self {
            id: Id::new(),
            name: name.into(),
            active: true,
            visible: true,
            region,
            kind,
            amplitude: None,
            distribution: None,
        }
    }
}

impl Load {
    /// An active, visible load without amplitudes or distribution, with an id of its own.
    pub fn new(name: impl Into<String>, region: Region, kind: LoadKind) -> Self {
        Self {
            id: Id::new(),
            name: name.into(),
            active: true,
            visible: true,
            region,
            kind,
            amplitude: None,
            factor_amplitude: None,
            tables: Default::default(),
            distribution: None,
            system: None,
            follower: false,
        }
    }
}

impl Load {
    /// The coordinate system of the components ([`Load::system`]); `None` for the global
    /// one or for a kind without one.
    pub fn component_system(&self, fe: &FeModel) -> Result<Option<Frame>, String> {
        match self.system.filter(|_| self.kind.takes_system()) {
            None => Ok(None),
            Some(id) => (fe.coordinate_system_or_global(Some(id)))
                .map(Some)
                .ok_or_else(|| format!("{}: The coordinate system does not exist.", self.name)),
        }
    }

    /// Components given in `system` (none: global) at `position` in global directions.
    pub fn to_global(
        system: Option<&Frame>,
        position: [f64; 3],
        local: [f64; 3],
    ) -> Result<[f64; 3], String> {
        let Some(system) = system else {
            return Ok(local);
        };
        let axes = system.directions_at(position)?;
        Ok(std::array::from_fn(|i| {
            (0..3).map(|k| local[k] * axes[k][i]).sum()
        }))
    }
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
    /// The internal identity other items refer to; the user sees only the name.
    pub id: crate::Id,
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
    pub amplitude: Option<Id>,
    /// Amplitude of the film coefficient or the emissivity (`FILM AMPLITUDE`,
    /// `RADIATION AMPLITUDE`); other loads have none.
    #[serde(default)]
    pub factor_amplitude: Option<Id>,
    /// A time table per component of the value (force and moment x, y, z; the pressure)
    /// that replaces the component: it follows the table over the step time instead of
    /// the amplitude. See [`LoadKind::table_components`].
    #[serde(default, skip_serializing_if = "crate::no_tables")]
    pub tables: [Option<ValueTable>; 3],
    /// Distribution the value is scaled by over the region; `None` is the same value
    /// everywhere.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distribution: Option<Id>,
    /// Coordinate system the components of a force, moment or excitation are given in;
    /// `None` is the global one. A cylindrical system turns them with the position of every
    /// node. See [`LoadKind::takes_system`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub system: Option<Id>,
    /// A follower load: under large deformations (`NLGEOM`, explicit dynamics) its
    /// direction turns with the region, by the rigid rotation that fits the motion of the
    /// region's nodes best, or with the rotation of a reference point or of a single beam or
    /// shell node. Only for the kinds of [`LoadKind::takes_system`].
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub follower: bool,
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
    /// Force and moment at a point the region follows, rigidly or on average, Ansys'
    /// remote force; see [`RemoteForce`].
    RemoteForce(RemoteForce),
    /// Bolt preload across a cut through the shank (`*PRE-TENSION SECTION`), the
    /// pre-tension load: the region is the element faces on one side of the cut, the value
    /// the force pulling the two sides together, or the shortening when `by_displacement`.
    /// The shortening counts from the start of the analysis, so 0 loosens the bolt; `lock`
    /// keeps the length the bolt had at the end of the previous step instead
    /// (`*BOUNDARY, FIXED`); with `by_displacement` as well the bolt is shortened by the value
    /// beyond that length.
    PreTension {
        value: f64,
        by_displacement: bool,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        lock: bool,
        /// Direction of the preload; `None` lets the solver take the surface normal.
        direction: Option<[f64; 3]>,
    },
    /// Total heat flow into the region, spread over its nodes when the input file is
    /// written, like a [`LoadKind::Force`]: on faces by area (a surface flux), along edges by
    /// length, in whole parts or element sets by volume (a body flux) and on nodes in equal
    /// shares. With `each_node` the whole value flows into every node of the region instead
    /// (a concentrated flux).
    Flux { value: f64, each_node: bool },
    /// Convection to the surroundings at the sink temperature (`*FILM`).
    Film { sink: f64, coefficient: f64 },
    /// Radiation to the surroundings at the sink temperature (`*RADIATE`); needs the
    /// physical constants of the model.
    Radiation { sink: f64, emissivity: f64 },
    /// Gravity, as the acceleration vector acting on the elements of the region
    /// (`*DLOAD`, `GRAV`). Needs the density of the materials.
    Gravity([f64; 3]),
    /// Standard gravity along one axis of the global system (`*DLOAD`, `GRAV`): the user
    /// picks the direction, the size is [`STANDARD_GRAVITY`]. Lowered to the same
    /// acceleration vector as [`LoadKind::Gravity`] when the model is exported.
    StandardGravity(GravityDirection),
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
    /// A force that follows a signal (an impulse, a sine, a frequency sweep) to excite the
    /// structure in a dynamic step; see [`Excitation`].
    Excitation(Excitation),
}

impl LoadKind {
    /// A total force, spread over the nodes of the region.
    pub fn force(value: [f64; 3]) -> Self {
        LoadKind::Force {
            value,
            each_node: false,
        }
    }

    /// A total heat flow, spread over the nodes of the region.
    pub fn flux(value: f64) -> Self {
        LoadKind::Flux {
            value,
            each_node: false,
        }
    }

    /// Whether the components can be given in a coordinate system of their own and the
    /// load can follow the deformation ([`Load::system`], [`Load::follower`]): forces,
    /// moments and excitations.
    pub fn takes_system(&self) -> bool {
        matches!(
            self,
            LoadKind::Force { .. } | LoadKind::Moment(_) | LoadKind::Excitation(_)
        )
    }

    /// How many components of the value can follow a time table of their own: the three
    /// of a force or moment, the pressure; none for the other loads.
    pub fn table_components(&self) -> usize {
        match self {
            LoadKind::Force { .. } | LoadKind::Moment(_) => 3,
            LoadKind::Pressure(_) => 1,
            _ => 0,
        }
    }

    /// The quantity of the values in the time tables of the components, if it takes any.
    pub fn table_quantity(&self) -> Option<units::Quantity> {
        match self {
            LoadKind::Force { .. } => Some(units::Quantity::Force),
            LoadKind::Moment(_) => Some(units::Quantity::Moment),
            LoadKind::Pressure(_) => Some(units::Quantity::Pressure),
            _ => None,
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
                | LoadKind::RemoteForce(_)
                | LoadKind::PreTension { .. }
                | LoadKind::Gravity(_)
                | LoadKind::StandardGravity(_)
                | LoadKind::Centrifugal { .. }
                | LoadKind::Moving(_)
                | LoadKind::Excitation(_)
        )
    }

    /// Whether the load acts on the mass of the elements (gravity, centrifugal), so the
    /// materials need a density.
    pub fn is_body_force(&self) -> bool {
        matches!(
            self,
            LoadKind::Gravity(_) | LoadKind::StandardGravity(_) | LoadKind::Centrifugal { .. }
        )
    }

    /// The acceleration vector of a gravity load, in the stored units (m/s²): what the user
    /// typed for [`LoadKind::Gravity`], the standard acceleration along the picked axis for
    /// [`LoadKind::StandardGravity`].
    pub fn gravity_vector(&self) -> Option<[f64; 3]> {
        match self {
            LoadKind::Gravity(g) => Some(*g),
            LoadKind::StandardGravity(direction) => Some(direction.vector()),
            _ => None,
        }
    }
}

/// The standard acceleration of gravity in m/s², the unit the model stores accelerations in.
pub const STANDARD_GRAVITY: f64 = 9.80665;

/// The axis of the global system a standard gravity acts along, with its sign.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum GravityDirection {
    PlusX,
    MinusX,
    PlusY,
    MinusY,
    PlusZ,
    #[default]
    MinusZ,
}

impl GravityDirection {
    pub const ALL: [GravityDirection; 6] = [
        GravityDirection::PlusX,
        GravityDirection::MinusX,
        GravityDirection::PlusY,
        GravityDirection::MinusY,
        GravityDirection::PlusZ,
        GravityDirection::MinusZ,
    ];

    /// The directions of a 2D model, which lies in the X-Y plane.
    pub const PLANAR: [GravityDirection; 4] = [
        GravityDirection::PlusX,
        GravityDirection::MinusX,
        GravityDirection::PlusY,
        GravityDirection::MinusY,
    ];

    pub fn label(self) -> &'static str {
        match self {
            GravityDirection::PlusX => "+X",
            GravityDirection::MinusX => "-X",
            GravityDirection::PlusY => "+Y",
            GravityDirection::MinusY => "-Y",
            GravityDirection::PlusZ => "+Z",
            GravityDirection::MinusZ => "-Z",
        }
    }

    /// The unit vector of the direction.
    pub fn unit(self) -> [f64; 3] {
        match self {
            GravityDirection::PlusX => [1.0, 0.0, 0.0],
            GravityDirection::MinusX => [-1.0, 0.0, 0.0],
            GravityDirection::PlusY => [0.0, 1.0, 0.0],
            GravityDirection::MinusY => [0.0, -1.0, 0.0],
            GravityDirection::PlusZ => [0.0, 0.0, 1.0],
            GravityDirection::MinusZ => [0.0, 0.0, -1.0],
        }
    }

    /// The standard acceleration along the direction.
    pub fn vector(self) -> [f64; 3] {
        self.unit().map(|c| c * STANDARD_GRAVITY)
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
    pub distribution: Option<Id>,
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
    pub amplitude: Option<Id>,
    /// Distribution the value is scaled by over the region; `None` is the same value
    /// everywhere.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distribution: Option<Id>,
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
    fn a_static_step_derives_its_increments_from_duration_and_frames() {
        let mut step = StaticStep {
            duration: 2.0,
            frames: 4,
            ..StaticStep::default()
        };
        let plan = step.plan(false);
        assert_eq!(plan.initial, 0.5);
        assert_eq!(plan.maximum, 2.0);
        assert!((plan.minimum - 2e-5).abs() < 1e-15);
        assert_eq!(plan.limit, None, "no limit on the number of increments");
        // Large deformations start with at most a tenth of the duration.
        assert_eq!(step.plan(true).initial, 0.2);
        step.frames = 20;
        assert_eq!(step.plan(true).initial, 0.1);
    }

    #[test]
    fn the_first_increment_never_exceeds_the_largest_one() {
        let step = StaticStep {
            control: IncrementControl {
                initial: Some(0.5),
                maximum: Some(0.1),
                ..IncrementControl::default()
            },
            ..StaticStep::default()
        };
        let plan = step.plan(true);
        assert_eq!(plan.initial, 0.1);
        assert_eq!(plan.maximum, 0.1);
        // Automatic as well: one frame over the whole step, the largest increment smaller.
        let step = StaticStep {
            control: IncrementControl {
                maximum: Some(0.25),
                ..IncrementControl::default()
            },
            ..StaticStep::default()
        };
        assert_eq!(step.plan(false).initial, 0.25);
    }

    #[test]
    fn fixed_increments_and_wrong_settings() {
        let mut step = StaticStep {
            control: IncrementControl {
                size: IncrementSize::Fixed(0.05),
                limit: Some(30),
                ..IncrementControl::default()
            },
            ..StaticStep::default()
        };
        let plan = step.plan(false);
        assert!(plan.fixed);
        assert_eq!(plan.initial, 0.05);
        assert_eq!(plan.limit, Some(30));
        assert_eq!(step.problem(), None);
        step.control.size = IncrementSize::Fixed(2.0);
        assert!(step.problem().is_some(), "longer than the duration");
        step.control.size = IncrementSize::Automatic;
        step.frames = 0;
        assert!(step.problem().is_some());
        step.frames = 1;
        step.control.minimum = Some(0.5);
        step.control.maximum = Some(0.1);
        assert!(step.problem().is_some(), "smallest above largest");
    }

    #[test]
    fn settings_of_older_static_steps_fall_back_to_the_defaults() {
        // An old project: its fields are ignored, the automatic settings take over and no
        // increment limit is carried over.
        let old = "(nlgeom: true, incrementation: Automatic, max_increments: 100, \
            initial_increment: 0.1, time_period: 1.0, min_increment: 1e-5, \
            max_increment: 0.1, solver: Automatic)";
        let step: StaticStep = ron::from_str(old).unwrap();
        assert!(step.nlgeom);
        assert_eq!(step.duration, 1.0);
        assert_eq!(step.control, IncrementControl::default());
    }

    #[test]
    fn eigenvalue_steps_start_without_field_outputs() {
        for step in [
            Step::new_frequency("F"),
            Step::new_buckle("B"),
            Step::new_complex_frequency("C"),
        ] {
            assert!(step.kind.computes_eigenmodes());
            assert!(step.field_outputs.is_empty(), "{}", step.name);
        }
        let static_step = Step::new_static("S");
        assert!(!static_step.kind.computes_eigenmodes());
        assert_eq!(static_step.field_outputs.len(), 2);
    }

    #[test]
    fn only_steps_with_prescribed_temperatures_take_defined_fields() {
        let kinds = [
            (StepKind::Static(StaticStep::default()), true),
            (StepKind::Frequency(FrequencyStep::default()), true),
            (StepKind::Dynamic(DynamicStep::default()), true),
            (StepKind::Buckle(BuckleStep::default()), true),
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
            orientation: None,
            name: "Section-1".into(),
            material: None,
            region: Region::Parts(vec!["A".into(), "B".into()]),
            thickness: 1.0,
            kind: SectionKind::Solid,
        });
        let mut step = Step::new_static("Step-1");
        step.boundary_conditions.push(BoundaryCondition {
            id: crate::Id::new(),
            distribution: None,
            name: "Fixed-1".into(),
            active: true,
            visible: true,
            region: Region::ElementSet("A".into()),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        step.loads.push(Load {
            id: crate::Id::new(),
            distribution: None,
            name: "Pressure-1".into(),
            active: true,
            visible: true,
            region: Region::Surface("A".into()),
            kind: LoadKind::Pressure(1.0),
            amplitude: None,
            factor_amplitude: None,
            tables: Default::default(),
            system: None,
            follower: false,
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
    fn deleted_parts_leave_regions() {
        let mut model = FeModel::default();
        model.sections.push(Section {
            orientation: None,
            name: "Section-1".into(),
            material: None,
            region: Region::Parts(vec!["A".into(), "B".into()]),
            thickness: 1.0,
            kind: SectionKind::Solid,
        });
        let mut step = Step::new_static("Step-1");
        step.boundary_conditions.push(BoundaryCondition {
            id: crate::Id::new(),
            distribution: None,
            name: "Fixed-1".into(),
            active: true,
            visible: true,
            region: Region::Parts(vec!["A".into()]),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        model.steps.push(step);
        model.remove_parts_from_regions(&["A".to_string()]);
        assert_eq!(model.sections[0].region, Region::Parts(vec!["B".into()]));
        assert_eq!(
            model.steps[0].boundary_conditions[0].region,
            Region::Parts(Vec::new())
        );
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

#[cfg(test)]
mod standard_gravity_tests {
    use super::*;

    #[test]
    fn standard_gravity_is_the_standard_acceleration_along_its_axis() {
        for direction in GravityDirection::ALL {
            let vector = direction.vector();
            let size = vector.iter().map(|c| c * c).sum::<f64>().sqrt();
            assert!((size - STANDARD_GRAVITY).abs() < 1e-12);
            assert_eq!(
                LoadKind::StandardGravity(direction).gravity_vector(),
                Some(vector)
            );
        }
        assert_eq!(
            LoadKind::StandardGravity(GravityDirection::MinusY).gravity_vector(),
            Some([0.0, -STANDARD_GRAVITY, 0.0])
        );
        assert_eq!(LoadKind::Pressure(1.0).gravity_vector(), None);
        assert!(LoadKind::StandardGravity(GravityDirection::MinusZ).is_body_force());
    }
}
