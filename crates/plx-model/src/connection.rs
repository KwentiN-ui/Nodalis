//! Contact regions: two sides and how they behave towards each other (bonded, frictionless,
//! frictional, ...), the way Ansys' contact regions describe contact. The user states the
//! intent; how it becomes CalculiX keywords (a tie or a contact pair, the contact
//! formulation, stiffnesses, master and slave) is derived when the input file is written, by
//! [`FeModel::resolve_contacts`]. Every derived choice can be overridden in
//! [`ContactSettings`], and the derivation says why it chose what it chose.
//!
//! [`FeModel::lower_contacts`] turns the regions into the explicit ties, surface interactions
//! and contact pairs the writer and the model checks already know.

use std::collections::{BTreeSet, HashMap};

use plx_mesh::{ElementId, FeMesh, NodeId};
use serde::{Deserialize, Serialize};

use crate::{
    ContactMethod, ContactPair, FeModel, Friction, InteractionProperty, Region, STICK_SLOPE_RATIO,
    StepKind, SurfaceBehavior, SurfaceInteraction, Tie,
};

/// Two surfaces of the model and how they behave towards each other. Which side becomes the
/// master is derived, see [`ContactSettings::sides`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ContactRegion {
    pub name: String,
    #[serde(default = "crate::active")]
    pub active: bool,
    pub side_a: Region,
    pub side_b: Region,
    pub behavior: ContactBehavior,
    #[serde(default)]
    pub settings: ContactSettings,
    pub color_a: [u8; 3],
    pub color_b: [u8; 3],
    /// The contact detection that found the region; `None` for a region made by hand.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub detected_by: Option<String>,
    /// Changed by the user after its detection: detecting again keeps it as it is.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub edited: bool,
}

impl ContactRegion {
    pub fn new(name: impl Into<String>, behavior: ContactBehavior) -> Self {
        Self {
            name: name.into(),
            active: true,
            side_a: Region::Faces(Vec::new()),
            side_b: Region::Faces(Vec::new()),
            behavior,
            settings: ContactSettings::default(),
            color_a: [255, 0, 0],
            color_b: [0, 0, 255],
            detected_by: None,
            edited: false,
        }
    }

    pub fn regions(&self) -> [&Region; 2] {
        [&self.side_a, &self.side_b]
    }

    pub fn regions_mut(&mut self) -> [&mut Region; 2] {
        [&mut self.side_a, &mut self.side_b]
    }

    /// The name from the definition, Ansys' "Rename Based on Definition": the type and the
    /// parts of both sides, e.g. "Bonded - Housing to Cover".
    pub fn name_from_definition(&self, mesh: &FeMesh) -> String {
        let parts = |region: &Region| -> String {
            let names: Vec<&str> = (parts_of(region, mesh).into_iter())
                .map(|p| mesh.parts[p].name.as_str())
                .collect();
            match names.len() {
                0 => "?".into(),
                1..=2 => names.join(", "),
                n => format!("{} and {} more", names[0], n - 1),
            }
        };
        format!(
            "{} - {} to {}",
            self.behavior.name(),
            parts(&self.side_a),
            parts(&self.side_b)
        )
    }

    /// Changes the type of the region; a name that starts with the type, as names made from
    /// the definition do (`Bonded - A to B`), follows it.
    pub fn set_behavior(&mut self, behavior: ContactBehavior) {
        if behavior.name() != self.behavior.name()
            && let Some(rest) = (self.name).strip_prefix(&format!("{} - ", self.behavior.name()))
        {
            self.name = format!("{} - {rest}", behavior.name());
        }
        self.behavior = behavior;
    }

    /// Swaps side A and side B, together with a master side chosen by hand.
    pub fn flip(&mut self) {
        std::mem::swap(&mut self.side_a, &mut self.side_b);
        std::mem::swap(&mut self.color_a, &mut self.color_b);
        self.settings.sides = self.settings.sides.map(SideRoles::flipped);
    }
}

/// The indices of the mesh parts a region lies on.
pub(crate) fn parts_of(region: &Region, mesh: &FeMesh) -> BTreeSet<usize> {
    let elements: BTreeSet<ElementId> = (region.elements(mesh).into_iter())
        .chain(region.faces(mesh).into_iter().map(|(e, _)| e))
        .collect();
    (mesh.parts.iter().enumerate())
        .filter(|(_, p)| p.elements.iter().any(|e| elements.contains(e)))
        .map(|(i, _)| i)
        .collect()
}

/// How the two sides of a contact region behave towards each other, Ansys' contact types.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum ContactBehavior {
    /// Glued: no separation, no sliding.
    Bonded,
    /// Held together normal to the surfaces, free to slide without friction.
    NoSeparation,
    /// Free to separate and to slide without friction.
    Frictionless,
    /// Free to separate, sliding with Coulomb friction of this coefficient.
    Frictional { mu: f64 },
    /// Free to separate, no sliding while in contact.
    Rough,
}

impl ContactBehavior {
    /// The behaviors in the order the dialog offers them, frictional with a typical
    /// coefficient for steel on steel.
    pub const ALL: [ContactBehavior; 5] = [
        ContactBehavior::Bonded,
        ContactBehavior::NoSeparation,
        ContactBehavior::Frictionless,
        ContactBehavior::Frictional { mu: 0.15 },
        ContactBehavior::Rough,
    ];

    pub fn name(self) -> &'static str {
        match self {
            ContactBehavior::Bonded => "Bonded",
            ContactBehavior::NoSeparation => "No Separation",
            ContactBehavior::Frictionless => "Frictionless",
            ContactBehavior::Frictional { .. } => "Frictional",
            ContactBehavior::Rough => "Rough",
        }
    }

    /// One line for the tooltip of the type list.
    pub fn summary(self) -> &'static str {
        match self {
            ContactBehavior::Bonded => {
                "Glued together: no gap opens, no sliding. Linear, no contact iterations."
            }
            ContactBehavior::NoSeparation => {
                "No gap opens, but the sides slide freely along each other. Linear, for small \
                 displacements."
            }
            ContactBehavior::Frictionless => {
                "The sides may separate and slide freely. Nonlinear, symmetric matrix."
            }
            ContactBehavior::Frictional { .. } => {
                "The sides may separate; they slide once the shear exceeds mu times the \
                 pressure. Nonlinear, unsymmetric matrix (slower on large models)."
            }
            ContactBehavior::Rough => {
                "The sides may separate but do not slide while in contact (friction \
                 coefficient 10). Nonlinear, unsymmetric matrix."
            }
        }
    }

    /// Whether the behavior is a contact (nonlinear, may separate).
    pub fn separates(self) -> bool {
        !matches!(
            self,
            ContactBehavior::Bonded | ContactBehavior::NoSeparation
        )
    }

    /// The friction coefficient written for the behavior; rough contact sticks by a
    /// coefficient so large that it never slides.
    pub fn friction(self) -> Option<f64> {
        match self {
            ContactBehavior::Frictional { mu } => Some(mu),
            ContactBehavior::Rough => Some(ROUGH_FRICTION),
            ContactBehavior::Bonded
            | ContactBehavior::NoSeparation
            | ContactBehavior::Frictionless => None,
        }
    }
}

/// Friction coefficient of rough contact.
pub const ROUGH_FRICTION: f64 = 10.0;

/// The settings of a contact region the derivation chooses; `None` is "Auto" (Ansys'
/// "Program Controlled"), a value overrides it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ContactSettings {
    /// How a bonded region is modeled.
    pub bonded_method: Option<BondedMethod>,
    /// Stiffness of the contact normal to the surfaces.
    pub normal_stiffness: Option<Stiffness>,
    /// Stiffness along the surfaces while sticking.
    pub tangential_stiffness: Option<Stiffness>,
    /// What happens to an initial gap or penetration.
    pub interface: Option<InterfaceTreatment>,
    /// Which side is the master.
    pub sides: Option<SideRoles>,
    /// Distance within which the sides count as touching: the position tolerance of a tie
    /// and the distance an adjustment closes.
    pub tolerance: Option<f64>,
    /// Pairing once per increment, node-to-surface contact only.
    pub small_sliding: Option<bool>,
    /// Heat conductance across the contact [power / (area temperature)], for steps with
    /// temperatures and displacements.
    pub conductance: Option<f64>,
}

/// How a bonded region is written.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BondedMethod {
    /// Constraint equations between the sides (`*TIE`): linear and cheap.
    Constraint,
    /// A penalty contact that neither opens nor slides (tied contact).
    Contact,
}

impl BondedMethod {
    pub const ALL: [BondedMethod; 2] = [BondedMethod::Constraint, BondedMethod::Contact];

    pub fn name(self) -> &'static str {
        match self {
            BondedMethod::Constraint => "Constraint (tie)",
            BondedMethod::Contact => "Contact (tied contact)",
        }
    }
}

/// A contact stiffness [force / length^3].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Stiffness {
    /// Factor on Young's modulus over the mesh size, E / h, of the softer side.
    Factor(f64),
    Value(f64),
}

/// What happens to an initial gap or penetration between the sides.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum InterfaceTreatment {
    /// The geometry as it is.
    AsIs,
    /// Slave nodes within the tolerance are moved onto the master at the start.
    AdjustToTouch,
    /// The sides count as this far apart wherever they face each other, whatever the
    /// geometry; negative for an interference fit.
    Offset(f64),
}

impl InterfaceTreatment {
    pub const ALL: [InterfaceTreatment; 3] = [
        InterfaceTreatment::AsIs,
        InterfaceTreatment::AdjustToTouch,
        InterfaceTreatment::Offset(0.0),
    ];

    pub fn name(self) -> &'static str {
        match self {
            InterfaceTreatment::AsIs => "As is",
            InterfaceTreatment::AdjustToTouch => "Adjust to touch",
            InterfaceTreatment::Offset(_) => "Offset",
        }
    }

    /// Whether two treatments are of the same kind, whatever their offset.
    pub fn same_kind(self, other: InterfaceTreatment) -> bool {
        std::mem::discriminant(&self) == std::mem::discriminant(&other)
    }
}

/// Which side of a contact region is the master.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SideRoles {
    AIsMaster,
    BIsMaster,
}

impl SideRoles {
    pub fn flipped(self) -> Self {
        match self {
            SideRoles::AIsMaster => SideRoles::BIsMaster,
            SideRoles::BIsMaster => SideRoles::AIsMaster,
        }
    }
}

/// The settings of all contact regions of a model.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ContactDefaults {
    /// Formulation of all contact pairs; the solver takes one per input file. `None` derives
    /// it.
    pub formulation: Option<ContactMethod>,
    /// Normal stiffness of contacts as a factor on E / h.
    pub stiffness_factor: f64,
    /// Parts held only by contact get weak springs to ground in the input file, see
    /// [`FeModel::weak_springs`].
    pub weak_springs: bool,
}

impl Default for ContactDefaults {
    fn default() -> Self {
        Self {
            formulation: None,
            stiffness_factor: STIFFNESS_FACTOR,
            weak_springs: true,
        }
    }
}

/// Normal contact stiffness in units of E / h, E the softer side's Young's modulus and h its
/// mesh size. CalculiX's hard contact takes 50 E with E in the model's units, which in
/// millimetres is about 50 E / h for elements of a millimetre and 1000 times softer in
/// metres; a factor on E / h is the same in every unit system.
pub const STIFFNESS_FACTOR: f64 = 10.0;
/// Stiffness of the weak springs of a part held only by contact in each direction, in units
/// of E L, E the part's Young's modulus and L its size: about the stiffness of the part
/// itself times this factor. Tried with CalculiX 2.21 and 2.23 on a block resting on another
/// one: 1e-6 solves every case (frictionless, initial gap, Nlgeom, all three formulations)
/// and the springs take about 1e-6 of the load, plus a share of the order 1e-6 E g / (p L)
/// for a gap g closed under the pressure p; 1e-8 fails node to surface contact again.
pub const WEAK_SPRING_FACTOR: f64 = 1e-6;
/// The factor of a bonded region modeled as a tied contact, which should not give way.
pub const TIED_STIFFNESS_FACTOR: f64 = 100.0;
/// Tie tolerance in units of the slave side's mesh size.
pub const TOLERANCE_FACTOR: f64 = 0.25;
/// Heat conductance across a closed contact in units of k / h, k the thermal conductivity
/// of the less conductive side and h its mesh size: far more than the layer of elements
/// next to it conducts, so that the contact hardly shows in the temperatures.
pub const CONDUCTANCE_FACTOR: f64 = 100.0;

/// Everything the derivation decided for the contact regions of a model.
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedContacts {
    /// The formulation of all contact pairs.
    pub formulation: ContactMethod,
    /// Why this formulation.
    pub formulation_reason: String,
    /// One entry per contact region, in their order; inactive regions too.
    pub items: Vec<Resolved>,
}

/// What a contact region becomes.
#[derive(Clone, Debug, PartialEq)]
pub struct Resolved {
    /// Whether side A is the master.
    pub a_is_master: bool,
    pub kind: ResolvedKind,
    /// The softer side's Young's modulus and the mesh size the stiffness derives from.
    pub young: Option<f64>,
    pub mesh_size: Option<f64>,
    /// The less conducting side's thermal conductivity the heat conductance derives from.
    pub conductivity: Option<f64>,
    /// Plain-language reasons, one per decision, for the dialog: "Bonded as a tie
    /// constraint: linear, no contact iterations".
    pub why: Vec<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ResolvedKind {
    /// A tie constraint; `tolerance` `None` leaves it to CalculiX.
    Tie {
        tolerance: Option<f64>,
        adjust: bool,
    },
    /// A contact pair.
    Pair {
        behavior: SurfaceBehavior,
        friction: Option<Friction>,
        /// Adjustment distance of `ADJUST=`.
        adjust: Option<f64>,
        small_sliding: bool,
        /// Distance between the sides that replaces the geometric one (`*CLEARANCE`).
        clearance: Option<f64>,
        /// Heat conductance across the contact (`*GAP CONDUCTANCE`).
        conductance: Option<f64>,
    },
    /// No separation: one equation per slave node within the tolerance of the master that
    /// holds it to the master normal to the surface.
    NoSeparation { tolerance: Option<f64> },
}

impl Resolved {
    /// The normal stiffness of a contact pair; `None` for a tie and hard contact.
    pub fn normal_stiffness(&self) -> Option<f64> {
        match &self.kind {
            ResolvedKind::Pair {
                behavior: SurfaceBehavior::Linear { k, .. } | SurfaceBehavior::Tied { k },
                ..
            } => Some(*k),
            _ => None,
        }
    }

    pub fn tangential_stiffness(&self) -> Option<f64> {
        match &self.kind {
            ResolvedKind::Pair { friction, .. } => friction.as_ref()?.stick_slope,
            ResolvedKind::Tie { .. } | ResolvedKind::NoSeparation { .. } => None,
        }
    }

    /// The tie tolerance or the adjustment distance.
    pub fn tolerance(&self) -> Option<f64> {
        match &self.kind {
            ResolvedKind::Tie { tolerance, .. } | ResolvedKind::NoSeparation { tolerance } => {
                *tolerance
            }
            ResolvedKind::Pair { adjust, .. } => *adjust,
        }
    }

    /// Whether slave nodes are moved onto the master at the start.
    pub fn adjusts(&self) -> bool {
        match &self.kind {
            ResolvedKind::Tie { adjust, .. } => *adjust,
            ResolvedKind::Pair { adjust, .. } => adjust.is_some(),
            ResolvedKind::NoSeparation { .. } => false,
        }
    }
}

impl ResolvedKind {
    /// One line for the dialog on what is written, without keywords.
    pub fn describe(&self) -> String {
        match self {
            ResolvedKind::Tie { .. } => "tie constraint (linear)".into(),
            ResolvedKind::NoSeparation { .. } => "normal constraint equations (linear)".into(),
            ResolvedKind::Pair {
                behavior, friction, ..
            } => match (behavior, friction) {
                (SurfaceBehavior::Tied { .. }, _) => "tied contact (penalty)".into(),
                (_, Some(f)) => format!("contact pair with friction {}", f.coefficient),
                (_, None) => "contact pair without friction".into(),
            },
        }
    }
}

/// The ties, contact pairs and surface interactions [`FeModel::lower_contacts`] added, by
/// the contact region each one stands for.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Lowered {
    /// Index of the first added tie and the region of each added tie.
    pub first_tie: usize,
    pub tie_regions: Vec<Vec<usize>>,
    pub first_pair: usize,
    pub pair_regions: Vec<usize>,
    /// The region of each contact region left in the lowered model: those of no separation,
    /// which the writer turns into equations.
    pub equation_regions: Vec<usize>,
}

impl Lowered {
    /// The contact region a tie or contact pair of the lowered model stands for, `None` for
    /// one the user defined.
    pub fn region_of(&self, item: crate::ModelItem) -> Option<usize> {
        match item {
            crate::ModelItem::Tie(i) => i
                .checked_sub(self.first_tie)
                .and_then(|i| self.tie_regions.get(i)?.first().copied()),
            crate::ModelItem::ContactPair(i) => i
                .checked_sub(self.first_pair)
                .and_then(|i| self.pair_regions.get(i).copied()),
            crate::ModelItem::ContactRegion(i) => self.equation_regions.get(i).copied(),
            _ => None,
        }
    }
}

/// Material properties by element.
struct Properties {
    young: HashMap<ElementId, f64>,
    conductivity: HashMap<ElementId, f64>,
    /// The distance each detection that found regions with an automatic tolerance searched
    /// within, by its name.
    detections: HashMap<String, f64>,
}

/// What the derivation needs to know about the model.
struct Derivation<'a> {
    formulation: ContactMethod,
    defaults: &'a ContactDefaults,
    /// Some active step has large deformations.
    nlgeom: bool,
    /// Some active step conducts heat across contacts.
    thermal: bool,
    /// The steps, which may switch regions off and on.
    steps: &'a [crate::Step],
    /// The distance the detection that found the region searched within.
    detected_within: Option<f64>,
}

/// Whether CalculiX conducts heat across the contact pairs in a step: those with
/// temperatures and displacements.
pub fn conducts_across_contacts(kind: &StepKind) -> bool {
    matches!(
        kind,
        StepKind::CoupledTempDisp(_) | StepKind::UncoupledTempDisp(_)
    )
}

/// What the derivation needs to know about one side of a region.
#[derive(Clone, Debug, Default)]
struct Side {
    /// The largest Young's modulus of the elements of the side.
    young: Option<f64>,
    /// The largest thermal conductivity of the elements of the side.
    conductivity: Option<f64>,
    /// Mean edge length of the faces, weighted by area.
    mesh_size: f64,
    area: f64,
}

impl Side {
    fn new(region: &Region, mesh: &FeMesh, properties: &Properties) -> Self {
        let faces = region.faces(mesh);
        let (mut area, mut weighted) = (0.0, 0.0);
        let mut young: Option<f64> = None;
        let mut conductivity: Option<f64> = None;
        let largest = |value: &mut Option<f64>, of: &HashMap<ElementId, f64>, e: ElementId| {
            if let Some(&v) = of.get(&e) {
                *value = Some(value.map_or(v, |x| x.max(v)));
            }
        };
        for &(element_id, face) in &faces {
            let Some(element) = mesh.element(element_id) else {
                continue;
            };
            largest(&mut young, &properties.young, element_id);
            largest(&mut conductivity, &properties.conductivity, element_id);
            let Some(topology) = element.faces().get(usize::from(face).saturating_sub(1)) else {
                continue;
            };
            let corners: Vec<[f64; 3]> = (topology.corners.iter())
                .filter_map(|&local| mesh.node(*element.nodes.get(local)?))
                .collect();
            let (face_area, size) = face_size(&corners);
            area += face_area;
            weighted += face_area * size;
        }
        if faces.is_empty() {
            for element in region.elements(mesh) {
                largest(&mut young, &properties.young, element);
                largest(&mut conductivity, &properties.conductivity, element);
            }
        }
        Self {
            young,
            conductivity,
            mesh_size: if area > 0.0 { weighted / area } else { 0.0 },
            area,
        }
    }

    /// The mesh size in steps of 25 %, so that about equal meshes count as equal.
    fn mesh_step(&self) -> i64 {
        if self.mesh_size > 0.0 {
            (self.mesh_size.ln() / 1.25f64.ln()).round() as i64
        } else {
            i64::MIN
        }
    }
}

/// Area of a face and its mean edge length; of an edge (2D faces, shell edges) its length
/// for both.
fn face_size(corners: &[[f64; 3]]) -> (f64, f64) {
    let dist =
        |a: [f64; 3], b: [f64; 3]| ((0..3).map(|k| (b[k] - a[k]).powi(2)).sum::<f64>()).sqrt();
    match corners {
        [a, b] => {
            let length = dist(*a, *b);
            (length, length)
        }
        [a, rest @ ..] if rest.len() >= 2 => {
            let mut area = 0.0;
            for pair in rest.windows(2) {
                let (u, v) = (
                    [0, 1, 2].map(|k| pair[0][k] - a[k]),
                    [0, 1, 2].map(|k| pair[1][k] - a[k]),
                );
                let c = [
                    u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0],
                ];
                area += 0.5 * (c.iter().map(|x| x * x).sum::<f64>()).sqrt();
            }
            let n = corners.len();
            let perimeter: f64 = (0..n).map(|i| dist(corners[i], corners[(i + 1) % n])).sum();
            (area, perimeter / n as f64)
        }
        _ => (0.0, 0.0),
    }
}

impl FeModel {
    /// The names contact history outputs can refer to: of the contact pairs and of the
    /// contact regions written as contact pairs.
    pub fn contact_pair_names(&self) -> Vec<&str> {
        let pairs = self.contact_pairs.iter().map(|p| p.name.as_str());
        let regions = (self.contact_regions.iter())
            .filter(|r| match r.behavior {
                // A bonded region left to Auto becomes a tied contact when its contact values
                // are printed.
                ContactBehavior::Bonded => {
                    r.settings.bonded_method != Some(BondedMethod::Constraint)
                }
                behavior => behavior.separates(),
            })
            .map(|r| r.name.as_str());
        pairs.chain(regions).collect()
    }

    /// Young's modulus of each element with a section whose material has one.
    pub(crate) fn young_of_elements(&self, mesh: &FeMesh) -> HashMap<ElementId, f64> {
        self.material_of_elements(mesh, |m| m.elastic.map(|e| e.young))
    }

    /// A property of the material of each element with a section whose material has it.
    fn material_of_elements(
        &self,
        mesh: &FeMesh,
        property: impl Fn(&crate::Material) -> Option<f64>,
    ) -> HashMap<ElementId, f64> {
        let mut values = HashMap::new();
        for section in &self.sections {
            let Some(value) = (self.materials.iter())
                .find(|m| m.name == section.material)
                .and_then(&property)
            else {
                continue;
            };
            for element in section.region.elements(mesh) {
                values.insert(element, value);
            }
        }
        values
    }

    fn properties_of_elements(&self, mesh: &FeMesh) -> Properties {
        let detections = (self.contact_detections.iter())
            .filter(|d| {
                (self.contact_regions.iter()).any(|r| {
                    r.settings.tolerance.is_none() && r.detected_by.as_ref() == Some(&d.name)
                })
            })
            .map(|d| {
                let distance = d
                    .tolerance
                    .unwrap_or_else(|| d.searched_diagonal(mesh) * 0.005);
                (d.name.clone(), distance)
            })
            .collect();
        Properties {
            young: self.young_of_elements(mesh),
            conductivity: self.material_of_elements(mesh, |m| m.conductivity),
            detections,
        }
    }

    /// The formulation of all contact pairs and why: the one chosen at the connections, else
    /// that of the contact pairs defined by hand, else surface to surface.
    pub fn contact_formulation(&self) -> (ContactMethod, String) {
        if let Some(method) = self.contact_defaults.formulation {
            return (method, format!("{} as chosen", method.name()));
        }
        if let Some(pair) = self.contact_pairs.iter().find(|p| p.active) {
            return (
                pair.method,
                format!(
                    "{}, as the contact pair {} defined by hand",
                    pair.method.name(),
                    pair.name
                ),
            );
        }
        (
            ContactMethod::SurfaceToSurface,
            "Surface to surface: smooth pressures, works with all elements".into(),
        )
    }

    /// What each contact region becomes in the input file and why.
    pub fn resolve_contacts(&self, mesh: &FeMesh) -> ResolvedContacts {
        let (formulation, formulation_reason) = self.contact_formulation();
        let properties = self.properties_of_elements(mesh);
        let items = (self.contact_regions.iter())
            .map(|region| self.resolve_with(region, mesh, &properties, formulation))
            .collect();
        ResolvedContacts {
            formulation,
            formulation_reason,
            items,
        }
    }

    /// What a contact region, of the model or not yet, becomes in the input file and why.
    pub fn resolve_contact(&self, region: &ContactRegion, mesh: &FeMesh) -> Resolved {
        let properties = self.properties_of_elements(mesh);
        self.resolve_with(region, mesh, &properties, self.contact_formulation().0)
    }

    fn resolve_with(
        &self,
        region: &ContactRegion,
        mesh: &FeMesh,
        properties: &Properties,
        formulation: ContactMethod,
    ) -> Resolved {
        let active = self.steps.iter().filter(|s| s.active);
        let nlgeom = active.clone().any(|s| s.kind.nlgeom());
        let thermal = active.clone().any(|s| conducts_across_contacts(&s.kind));
        let a = Side::new(&region.side_a, mesh, properties);
        let b = Side::new(&region.side_b, mesh, properties);
        let detected_within = (region.detected_by.as_ref())
            .and_then(|name| properties.detections.get(name))
            .copied();
        let model = Derivation {
            formulation,
            defaults: &self.contact_defaults,
            nlgeom,
            thermal,
            steps: &self.steps,
            detected_within,
        };
        resolve(region, &a, &b, &model)
    }

    /// The model with its contact regions replaced by the ties, surface interactions and
    /// contact pairs they stand for, added after those defined by hand, and which region each
    /// added item comes from. Bonded regions whose slave sides share nodes become one tie, as
    /// the solver ties a node to one master only. Active regions of no separation stay, with
    /// their master and tolerance settled, for the writer to turn into equations.
    pub fn lower_contacts(&self, mesh: &FeMesh) -> (FeModel, Lowered) {
        let mut lowered = self.clone();
        let mut origin = Lowered {
            first_tie: self.ties.len(),
            first_pair: self.contact_pairs.len(),
            ..Lowered::default()
        };
        if self.contact_regions.is_empty() {
            return (lowered, origin);
        }
        let resolved = self.resolve_contacts(mesh);
        lowered.contact_regions.clear();
        let mut tie_names: Vec<String> = self.ties.iter().map(|t| t.name.clone()).collect();
        // Active ties by their slave nodes, for merging those that share some.
        let mut ties: Vec<(Tie, BTreeSet<NodeId>, Vec<usize>)> = Vec::new();
        // Interactions of one region each, whose friction a step changes.
        let mut private: Vec<String> = Vec::new();
        for (r, (region, item)) in self.contact_regions.iter().zip(&resolved.items).enumerate() {
            let (master, slave, master_color, slave_color) = if item.a_is_master {
                (
                    &region.side_a,
                    &region.side_b,
                    region.color_a,
                    region.color_b,
                )
            } else {
                (
                    &region.side_b,
                    &region.side_a,
                    region.color_b,
                    region.color_a,
                )
            };
            match &item.kind {
                ResolvedKind::Tie { tolerance, adjust } => {
                    let name = unique_name(&region.name, &tie_names);
                    tie_names.push(name.clone());
                    let mut tie = Tie::new(name);
                    tie.active = region.active;
                    tie.position_tolerance = *tolerance;
                    tie.adjust = *adjust;
                    tie.master = master.clone();
                    tie.slave = slave.clone();
                    tie.master_color = master_color;
                    tie.slave_color = slave_color;
                    let nodes: BTreeSet<NodeId> = if region.active {
                        slave.nodes(mesh).into_iter().collect()
                    } else {
                        BTreeSet::new()
                    };
                    match (ties.iter_mut()).find(|(t, n, _)| t.active && !n.is_disjoint(&nodes)) {
                        Some((merged, merged_nodes, regions)) => {
                            merged.master = union(&merged.master, &tie.master, mesh);
                            merged.slave = union(&merged.slave, &tie.slave, mesh);
                            merged.position_tolerance =
                                match (merged.position_tolerance, tie.position_tolerance) {
                                    (Some(x), Some(y)) => Some(x.max(y)),
                                    _ => None,
                                };
                            merged.adjust |= tie.adjust;
                            merged_nodes.extend(nodes);
                            regions.push(r);
                        }
                        None => ties.push((tie, nodes, vec![r])),
                    }
                }
                ResolvedKind::NoSeparation { tolerance } => {
                    if region.active {
                        let mut kept = region.clone();
                        kept.settings.sides = Some(if item.a_is_master {
                            SideRoles::AIsMaster
                        } else {
                            SideRoles::BIsMaster
                        });
                        kept.settings.tolerance = *tolerance;
                        lowered.contact_regions.push(kept);
                        origin.equation_regions.push(r);
                    }
                }
                ResolvedKind::Pair {
                    behavior,
                    friction,
                    adjust,
                    small_sliding,
                    clearance,
                    conductance,
                } => {
                    let mut properties =
                        vec![InteractionProperty::SurfaceBehavior(behavior.clone())];
                    properties.extend(friction.clone().map(InteractionProperty::Friction));
                    properties.extend(conductance.map(|c| {
                        InteractionProperty::GapConductance(crate::GapConductance::Constant(c))
                    }));
                    // A step changing the friction changes the whole interaction.
                    let own = region.friction_changed(&self.steps);
                    let interaction = match (lowered.surface_interactions.iter())
                        .skip(self.surface_interactions.len())
                        .find(|i| i.properties == properties && !own && !private.contains(&i.name))
                    {
                        Some(found) => found.name.clone(),
                        None => {
                            let taken: Vec<String> = (lowered.surface_interactions.iter())
                                .map(|i| i.name.clone())
                                .collect();
                            let name = unique_name(&interaction_name(region.behavior), &taken);
                            lowered.surface_interactions.push(SurfaceInteraction {
                                name: name.clone(),
                                properties,
                            });
                            if own {
                                private.push(name.clone());
                            }
                            name
                        }
                    };
                    let mut pair = ContactPair::new(region.name.clone(), interaction);
                    pair.active = region.active;
                    pair.method = resolved.formulation;
                    pair.small_sliding = *small_sliding;
                    pair.adjust = adjust.is_some();
                    pair.adjustment_size = *adjust;
                    pair.clearance = *clearance;
                    pair.master = master.clone();
                    pair.slave = slave.clone();
                    pair.master_color = master_color;
                    pair.slave_color = slave_color;
                    lowered.contact_pairs.push(pair);
                    origin.pair_regions.push(r);
                }
            }
        }
        for (tie, _, regions) in ties {
            lowered.ties.push(tie);
            origin.tie_regions.push(regions);
        }
        // Changes in the steps are kept for the contact pairs; ties and equations stay.
        let pairs: BTreeSet<String> = (lowered.contact_pairs.iter())
            .map(|p| p.name.clone())
            .collect();
        // Contact values are printed for contact pairs only; of a region written as a tie
        // or as equations there are none (see the model check).
        let regions: BTreeSet<&str> = self
            .contact_regions
            .iter()
            .map(|r| r.name.as_str())
            .collect();
        for step in &mut lowered.steps {
            step.contact_changes.retain(|c| pairs.contains(&c.region));
            step.history_outputs.retain(|o| match &o.kind {
                crate::HistoryKind::Contact { pair } => {
                    pairs.contains(pair) || !regions.contains(pair.as_str())
                }
                _ => true,
            });
        }
        // Contact results for the contact check after solving, where none are asked for.
        if (lowered.contact_pairs.iter().skip(origin.first_pair)).any(|p| p.active) {
            for step in &mut lowered.steps {
                let asked = (step.field_outputs.iter())
                    .any(|f| f.active && f.kind == crate::OutputKind::Contact);
                let structural = matches!(
                    step.kind,
                    StepKind::Static(_) | StepKind::Visco(_) | StepKind::Dynamic(_)
                );
                if structural && !asked {
                    step.field_outputs.push(crate::FieldOutput::new(
                        "PLX_Contact",
                        crate::OutputKind::Contact,
                        crate::OutputKind::Contact.default_variables(),
                    ));
                }
            }
        }
        (lowered, origin)
    }
}

/// The name of the surface interaction of contact regions of a behavior.
fn interaction_name(behavior: ContactBehavior) -> String {
    format!("PLX_{}", behavior.name())
}

/// `name`, or `name-2`, `name-3`, ... if it is taken.
fn unique_name(name: &str, taken: &[String]) -> String {
    let free = |candidate: &str| !taken.iter().any(|t| t.eq_ignore_ascii_case(candidate));
    if free(name) {
        return name.to_owned();
    }
    (2..)
        .map(|n| format!("{name}-{n}"))
        .find(|candidate| free(candidate))
        .expect("unbounded range")
}

/// The element faces of both regions, or the first region if either has none.
fn union(a: &Region, b: &Region, mesh: &FeMesh) -> Region {
    let (fa, fb) = (a.faces(mesh), b.faces(mesh));
    if fa.is_empty() || fb.is_empty() {
        return a.clone();
    }
    let faces: BTreeSet<(ElementId, u8)> = fa.into_iter().chain(fb).collect();
    Region::Faces(faces.into_iter().collect())
}

/// The derivation for one region.
fn resolve(region: &ContactRegion, a: &Side, b: &Side, model: &Derivation) -> Resolved {
    let Derivation {
        formulation,
        defaults,
        nlgeom,
        thermal,
        steps,
        detected_within,
    } = *model;
    let settings = &region.settings;
    let mut why = Vec::new();
    // Master: the coarser mesh, so that the finer mesh's nodes cannot pass through it; then
    // the stiffer material; then the larger area.
    let a_is_master = match settings.sides {
        Some(roles) => {
            why.push("Master side as chosen".into());
            roles == SideRoles::AIsMaster
        }
        None => {
            let ordering = (a.mesh_step().cmp(&b.mesh_step()))
                .then(a.young.unwrap_or(0.0).total_cmp(&b.young.unwrap_or(0.0)))
                .then(a.area.total_cmp(&b.area));
            let a_is_master = ordering != std::cmp::Ordering::Less;
            let reason = if a.mesh_step() != b.mesh_step() {
                "coarser mesh"
            } else if a.young != b.young {
                "stiffer material"
            } else {
                "larger area"
            };
            let side = if a_is_master { "A" } else { "B" };
            why.push(format!("Master: side {side} ({reason})"));
            a_is_master
        }
    };
    let slave = if a_is_master { b } else { a };
    let master = if a_is_master { a } else { b };
    let smaller = |x: Option<f64>, y: Option<f64>| match (x, y) {
        (Some(x), Some(y)) => Some(x.min(y)),
        (x, y) => x.or(y),
    };
    let young = smaller(a.young, b.young);
    let conductivity = smaller(a.conductivity, b.conductivity);
    let mesh_size = [slave.mesh_size, master.mesh_size]
        .into_iter()
        .find(|&h| h > 0.0);
    // Automatically a quarter of the mesh size, but at least the distance the detection that
    // found the region searched within, so that a tie catches every node it saw touching:
    // a detection on the geometry finds faces apart by up to 0.5 % of the model size.
    let by_mesh = mesh_size.map(|h| TOLERANCE_FACTOR * h);
    let by_detection = detected_within.filter(|&d| by_mesh.is_none_or(|m| d > m));
    let tolerance = settings.tolerance.or(by_detection).or(by_mesh);
    let tolerance_reason = match by_detection {
        Some(_) => format!(
            "Tolerance: the distance the detection searched within, more than \
             {TOLERANCE_FACTOR} x mesh size"
        ),
        None => format!("Tolerance: {TOLERANCE_FACTOR} x mesh size"),
    };
    let bonded_method = match (region.behavior, settings.bonded_method) {
        (ContactBehavior::Bonded, Some(BondedMethod::Contact))
            if formulation != ContactMethod::SurfaceToSurface =>
        {
            why.push(format!(
                "Bonded as a tie constraint: a tied contact needs Surface to surface, the \
                 contacts use {}",
                formulation.name()
            ));
            Some(BondedMethod::Constraint)
        }
        (ContactBehavior::Bonded, Some(method)) => {
            why.push(format!("Bonded as {}, as chosen", method.name()));
            Some(method)
        }
        (ContactBehavior::Bonded, None)
            if region.switched(steps) && formulation == ContactMethod::SurfaceToSurface =>
        {
            why.push(
                "Bonded as a tied contact: a step switches it off or on, which a tie cannot do"
                    .into(),
            );
            Some(BondedMethod::Contact)
        }
        (ContactBehavior::Bonded, None)
            if region.prints_contact(steps) && formulation == ContactMethod::SurfaceToSurface =>
        {
            why.push(
                "Bonded as a tied contact: a history output prints its contact values, which \
                 a tie has none of"
                    .into(),
            );
            Some(BondedMethod::Contact)
        }
        (ContactBehavior::Bonded, None) => {
            why.push("Bonded as a tie constraint: linear, no contact iterations".into());
            Some(BondedMethod::Constraint)
        }
        _ => None,
    };
    let stiffness = |setting: Option<Stiffness>, factor: f64| -> Option<f64> {
        match setting {
            Some(Stiffness::Value(k)) => Some(k),
            Some(Stiffness::Factor(f)) => Some(f * young? / mesh_size?),
            None => Some(factor * young? / mesh_size?),
        }
    };
    let adjust_to_touch = match settings.interface {
        Some(InterfaceTreatment::AdjustToTouch) => true,
        Some(InterfaceTreatment::AsIs | InterfaceTreatment::Offset(_)) => false,
        None => region.behavior == ContactBehavior::Bonded,
    };
    let offset = match settings.interface {
        Some(InterfaceTreatment::Offset(offset)) => Some(offset),
        _ => None,
    };
    let kind = if region.behavior == ContactBehavior::NoSeparation {
        why.push(
            "No separation as equations normal to the master side: linear, the sides slide \
             freely"
                .into(),
        );
        if settings.tolerance.is_none() && tolerance.is_some() {
            why.push(format!(
                "{tolerance_reason}; slave nodes further from the master stay free"
            ));
        }
        if nlgeom {
            why.push(
                "The equations hold for small displacements only; with large deformations the \
                 sides drift apart as they slide or turn"
                    .into(),
            );
        }
        if settings.interface.is_some() {
            why.push("The initial gap stays as it is: the equations keep it".into());
        }
        ResolvedKind::NoSeparation { tolerance }
    } else if bonded_method == Some(BondedMethod::Constraint) {
        if offset.is_some() {
            why.push("No offset: a tie keeps the sides as they are".into());
        }
        if settings.tolerance.is_none() && tolerance.is_some() {
            why.push(tolerance_reason);
        }
        ResolvedKind::Tie {
            tolerance,
            adjust: adjust_to_touch,
        }
    } else {
        let tied = bonded_method == Some(BondedMethod::Contact);
        // Mortar contact is hard without a spring; the solver takes no surface behavior there.
        let factor = if tied {
            TIED_STIFFNESS_FACTOR
        } else {
            defaults.stiffness_factor
        };
        let k = stiffness(settings.normal_stiffness, factor);
        match (k, settings.normal_stiffness) {
            (Some(_), None) => why.push(format!("Normal stiffness: {factor} x E/h")),
            (None, _) => why.push("Normal stiffness: hard (no Young's modulus known)".into()),
            _ => {}
        }
        let behavior = match (tied, k) {
            (true, k) => SurfaceBehavior::Tied {
                k: k.unwrap_or(0.0),
            },
            (false, _) if formulation == ContactMethod::Mortar => SurfaceBehavior::Hard,
            (false, Some(k)) => SurfaceBehavior::Linear {
                k,
                // The tension at large clearances, E / 70000.
                sigma_inf: young.unwrap_or(0.0) / 70_000.0,
                c0: None,
            },
            (false, None) => SurfaceBehavior::Hard,
        };
        let coefficient = if tied {
            Some(0.1)
        } else {
            region.behavior.friction()
        };
        let friction = coefficient.map(|coefficient| {
            // Rough and tied contact should not slide, frictional contact converges better
            // with the softer stick slope CalculiX recommends.
            let ratio = if tied || region.behavior == ContactBehavior::Rough {
                1.0
            } else {
                STICK_SLOPE_RATIO
            };
            let stick_slope = match settings.tangential_stiffness {
                Some(Stiffness::Value(v)) => Some(v),
                Some(Stiffness::Factor(f)) => young.zip(mesh_size).map(|(e, h)| f * e / h),
                None => k.map(|k| ratio * k),
            };
            if settings.tangential_stiffness.is_none() && stick_slope.is_some() {
                why.push(if ratio == 1.0 {
                    "Tangential stiffness: equal to the normal stiffness, no sliding".into()
                } else {
                    "Tangential stiffness: normal stiffness / 100".into()
                });
            }
            Friction {
                coefficient,
                stick_slope,
            }
        });
        let adjust = adjust_to_touch.then(|| tolerance.unwrap_or(0.0));
        if adjust_to_touch {
            why.push("Adjusted to touch at the start".into());
        }
        // The solver takes a clearance for face-to-face penalty contact only.
        let clearance = match offset {
            Some(offset) if formulation == ContactMethod::SurfaceToSurface && !tied => Some(offset),
            Some(_) => {
                why.push(format!(
                    "No offset: it needs a contact of {} that is not tied",
                    ContactMethod::SurfaceToSurface.name()
                ));
                None
            }
            None => None,
        };
        if let Some(offset) = clearance {
            why.push(if offset < 0.0 {
                "Offset: the sides overlap by the offset (interference fit)".into()
            } else {
                "Offset: the sides are the offset apart wherever they face each other".into()
            });
        }
        let conductance = match settings.conductance {
            Some(c) => Some(c),
            None if thermal => {
                let c = conductivity
                    .zip(mesh_size)
                    .map(|(k, h)| CONDUCTANCE_FACTOR * k / h);
                why.push(match c {
                    Some(_) => format!("Heat conductance: {CONDUCTANCE_FACTOR} x k/h"),
                    None => "No heat conductance: no thermal conductivity known".into(),
                });
                c
            }
            None => None,
        };
        let small_sliding = formulation == ContactMethod::NodeToSurface
            && settings.small_sliding.unwrap_or(!nlgeom);
        ResolvedKind::Pair {
            behavior,
            friction,
            adjust,
            small_sliding,
            clearance,
            conductance,
        }
    };
    Resolved {
        a_is_master,
        kind,
        young,
        mesh_size,
        conductivity,
        why,
    }
}

/// Whether a step type takes contact; the mortar formulation takes static and dynamic
/// steps only.
pub fn mortar_supports(kind: &StepKind) -> bool {
    matches!(
        kind,
        StepKind::Static(_) | StepKind::Visco(_) | StepKind::Dynamic(_)
    )
}
