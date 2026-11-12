//! Contact and tie definitions: the surface interactions, contact pairs and tie
//! constraints. Master and slave are regions the user picked; their surfaces are written to
//! the input file only when it is exported.

use serde::{Deserialize, Serialize};

use crate::{CompressionOnly, NodeTie, Region, RigidBody, Spring, SpringEnd};

/// Mechanical and thermal behaviour of touching surfaces (`*SURFACE INTERACTION`), referred to
/// by contact pairs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SurfaceInteraction {
    /// The internal identity other items refer to; the user sees only the name.
    pub id: crate::Id,
    pub name: String,
    /// The interaction models in the order the user added them; each kind at most once.
    pub properties: Vec<InteractionProperty>,
}

impl SurfaceInteraction {
    pub fn new(name: impl Into<String>, properties: Vec<InteractionProperty>) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            properties,
        }
    }

    /// Whether the surface behavior is tied.
    pub fn is_tied(&self) -> bool {
        (self.properties.iter()).any(|p| {
            matches!(
                p,
                InteractionProperty::SurfaceBehavior(SurfaceBehavior::Tied { .. })
            )
        })
    }

    pub fn has_friction(&self) -> bool {
        (self.properties.iter()).any(|p| matches!(p, InteractionProperty::Friction(_)))
    }

    /// Adds the friction the solver requires with a tied surface behavior: its stick slope is
    /// the stiffness of the tie along the surfaces. Returns whether it was added.
    pub fn add_friction_if_tied(&mut self) -> bool {
        let add = self.is_tied() && !self.has_friction();
        if add {
            (self.properties).push(InteractionProperty::Friction(Friction::default()));
        }
        add
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum InteractionProperty {
    SurfaceBehavior(SurfaceBehavior),
    Friction(Friction),
    GapConductance(GapConductance),
}

impl InteractionProperty {
    /// The names of the interaction models, which are also the default models.
    pub fn all() -> [InteractionProperty; 3] {
        [
            InteractionProperty::SurfaceBehavior(SurfaceBehavior::default()),
            InteractionProperty::Friction(Friction::default()),
            InteractionProperty::GapConductance(GapConductance::default()),
        ]
    }

    pub fn name(&self) -> &'static str {
        match self {
            InteractionProperty::SurfaceBehavior(_) => "Surface Behavior",
            InteractionProperty::Friction(_) => "Friction",
            InteractionProperty::GapConductance(_) => "Gap Conductance",
        }
    }
}

/// Pressure-overclosure relation of the contact (`*SURFACE BEHAVIOR`), with the default
/// values for mm, N and MPa.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum SurfaceBehavior {
    /// No penetration at all.
    #[default]
    Hard,
    /// Linear spring: slope `k`, tension `sigma_inf` at large clearances and the clearance
    /// `c0` up to which the spring acts (`None` lets the solver choose it).
    Linear {
        k: f64,
        sigma_inf: f64,
        c0: Option<f64>,
    },
    /// Pressure falling off exponentially with the clearance: `c0` where it is 1 % of the
    /// pressure `p0` at zero clearance.
    Exponential { c0: f64, p0: f64 },
    /// Pressure over overclosure as a table of (pressure, overclosure).
    Tabular(Vec<[f64; 2]>),
    /// Tied contact with the stiffness `k`.
    Tied { k: f64 },
}

impl SurfaceBehavior {
    /// Kinds of the relation with the default values, as offered in the dialog.
    pub fn kinds() -> [SurfaceBehavior; 5] {
        // The values for mm, t, s, stored in SI units. E = 200 GPa: K = 50 E,
        // sigma_inf = E / 70000.
        use crate::{Quantity, UnitSystem::MmTonSC};
        let k = MmTonSC.stored(1e7, Quantity::ForcePerVolume);
        let pressure = |p| MmTonSC.stored(p, Quantity::Pressure);
        let length = |l| MmTonSC.stored(l, Quantity::Length);
        [
            SurfaceBehavior::Hard,
            SurfaceBehavior::Linear {
                k,
                sigma_inf: pressure(2.86),
                c0: None,
            },
            SurfaceBehavior::Exponential {
                c0: length(1.0),
                p0: pressure(0.1),
            },
            SurfaceBehavior::Tabular(vec![[0.0, 0.0], [pressure(1e5), length(1.0)]]),
            SurfaceBehavior::Tied { k },
        ]
    }

    /// The value of `PRESSURE-OVERCLOSURE=`.
    pub fn keyword(&self) -> &'static str {
        match self {
            SurfaceBehavior::Hard => "Hard",
            SurfaceBehavior::Linear { .. } => "Linear",
            SurfaceBehavior::Exponential { .. } => "Exponential",
            SurfaceBehavior::Tabular(_) => "Tabular",
            SurfaceBehavior::Tied { .. } => "Tied",
        }
    }
}

/// Coulomb friction (`*FRICTION`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Friction {
    pub coefficient: f64,
    /// Shear stress over tangential displacement while sticking; `None` lets the solver choose.
    pub stick_slope: Option<f64>,
}

impl Default for Friction {
    fn default() -> Self {
        Self {
            coefficient: 0.1,
            stick_slope: None,
        }
    }
}

/// Heat conductance across the contact (`*GAP CONDUCTANCE`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum GapConductance {
    Constant(f64),
    /// Rows of (conductance, contact pressure, temperature).
    Tabular(Vec<[f64; 3]>),
}

impl Default for GapConductance {
    fn default() -> Self {
        GapConductance::Constant(0.0)
    }
}

/// How the solver treats a contact pair (`TYPE=`). All pairs of a model must use the same.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContactMethod {
    NodeToSurface,
    #[default]
    SurfaceToSurface,
    Mortar,
    Massless,
}

impl ContactMethod {
    pub const ALL: [ContactMethod; 4] = [
        ContactMethod::NodeToSurface,
        ContactMethod::SurfaceToSurface,
        ContactMethod::Mortar,
        ContactMethod::Massless,
    ];

    /// The name, which is also the value of `TYPE=`.
    pub fn name(self) -> &'static str {
        match self {
            ContactMethod::NodeToSurface => "Node to surface",
            ContactMethod::SurfaceToSurface => "Surface to surface",
            ContactMethod::Mortar => "Mortar",
            ContactMethod::Massless => "Massless",
        }
    }
}

impl ContactMethod {
    /// One line for the tooltip of the method list and the search dialog.
    pub fn summary(self) -> &'static str {
        match self {
            ContactMethod::NodeToSurface => {
                "Slave nodes against master faces. Fast and robust; the default of many solvers."
            }
            ContactMethod::SurfaceToSurface => {
                "Penalty contact integrated over the slave faces. Smoother pressures than node to surface."
            }
            ContactMethod::Mortar => {
                "Mortar contact. Most accurate pressures, but for static steps only."
            }
            ContactMethod::Massless => "Massless node-to-face contact for explicit dynamics only.",
        }
    }

    /// When to use the method, its strengths, weaknesses and requirements.
    pub fn description(self) -> &'static str {
        match self {
            ContactMethod::NodeToSurface => {
                "Each slave node is kept outside the master surface. Works with nodes or faces as \
                 slave and with all step types, including dynamics. Master nodes may poke \
                 through the slave surface, and the pressure is noisy, so choose the finer mesh \
                 as slave. Recommended with linear elements and for explicit dynamics."
            }
            ContactMethod::SurfaceToSurface => {
                "Face-to-face penalty contact: the contact condition is integrated over the \
                 faces of the slave surface, which must therefore consist of element faces. \
                 Pressures are smoother and the solution converges better than with node to \
                 surface, especially with quadratic elements (C3D10, C3D20). Slower per \
                 iteration. A good default for static contact."
            }
            ContactMethod::Mortar => {
                "Face-to-face mortar contact. The most accurate contact pressure and little \
                 dependence on which side is master. It can only be used in static steps and \
                 needs element-face surfaces; try it when Surface to surface gives \
                 oscillating pressures. Slowest of the methods."
            }
            ContactMethod::Massless => {
                "Node-to-face contact without contact stiffness mass, designed for explicit \
                 dynamics (Dynamic step with explicit integration). Use it only there."
            }
        }
    }
}

/// Two surfaces that may touch (`*CONTACT PAIR`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ContactPair {
    /// The internal identity other items refer to; the user sees only the name.
    pub id: crate::Id,
    pub name: String,
    /// A deactivated contact pair is written to the input file as a comment only.
    #[serde(default = "crate::active")]
    pub active: bool,
    /// Name of the surface interaction.
    pub interaction: Option<crate::Id>,
    pub method: ContactMethod,
    /// Pairing only at the start of each increment; node to surface contact only.
    pub small_sliding: bool,
    /// Moves slave nodes onto the master surface at the start (`ADJUST=`).
    pub adjust: bool,
    /// Distance within which slave nodes are moved; `None` moves only penetrating nodes.
    pub adjustment_size: Option<f64>,
    /// Distance between the sides that replaces the geometric one (`*CLEARANCE`), surface to
    /// surface contact only; negative for an interference fit.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub clearance: Option<f64>,
    pub master: Region,
    pub slave: Region,
}

/// The name of a swapped pair: `<slave>_to_<master>` for `<master>_to_<slave>`, other
/// names as they are.
pub fn swapped_name(name: &str) -> String {
    match name.split("_to_").collect::<Vec<_>>()[..] {
        [master, slave] => format!("{slave}_to_{master}"),
        _ => name.to_owned(),
    }
}

impl ContactPair {
    /// Swaps master and slave, also in a name `<master>_to_<slave>`.
    pub fn swap_master_slave(&mut self) {
        std::mem::swap(&mut self.master, &mut self.slave);
        self.name = swapped_name(&self.name);
    }

    pub fn new(name: impl Into<String>, interaction: Option<crate::Id>) -> Self {
        Self {
            id: crate::Id::new(),
            name: name.into(),
            active: true,
            interaction,
            method: ContactMethod::default(),
            small_sliding: false,
            adjust: false,
            adjustment_size: None,
            clearance: None,
            master: Region::Faces(Vec::new()),
            slave: Region::Faces(Vec::new()),
        }
    }
}

/// A constraint between parts of the model or to ground.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Constraint {
    Spring(Spring),
    CompressionOnly(CompressionOnly),
    /// Only in projects saved while ties were constraints; [`crate::FeModel::migrate`] moves
    /// them to the model's ties, listed with the contact pairs.
    Tie(Tie),
    RigidBody(RigidBody),
    Joint(crate::Joint),
    /// Only in projects saved while node ties were constraints; [`crate::FeModel::migrate`]
    /// moves them to the model's node ties, where they belong with the contact pairs.
    NodeTie(NodeTie),
}

impl Constraint {
    pub fn name(&self) -> &str {
        match self {
            Constraint::Spring(c) => &c.name,
            Constraint::CompressionOnly(c) => &c.name,
            Constraint::Tie(tie) => &tie.name,
            Constraint::RigidBody(c) => &c.name,
            Constraint::Joint(c) => &c.name,
            Constraint::NodeTie(c) => &c.name,
        }
    }

    pub fn active(&self) -> bool {
        match self {
            Constraint::Spring(c) => c.active,
            Constraint::CompressionOnly(c) => c.active,
            Constraint::Tie(tie) => tie.active,
            Constraint::RigidBody(c) => c.active,
            Constraint::Joint(c) => c.active,
            Constraint::NodeTie(c) => c.active,
        }
    }

    pub fn active_mut(&mut self) -> &mut bool {
        match self {
            Constraint::Spring(c) => &mut c.active,
            Constraint::CompressionOnly(c) => &mut c.active,
            Constraint::Tie(tie) => &mut tie.active,
            Constraint::RigidBody(c) => &mut c.active,
            Constraint::Joint(c) => &mut c.active,
            Constraint::NodeTie(c) => &mut c.active,
        }
    }

    /// Whether the constraint's symbol is shown; ties have none.
    pub fn visible(&self) -> bool {
        match self {
            Constraint::Spring(c) => c.visible,
            Constraint::CompressionOnly(c) => c.visible,
            Constraint::RigidBody(c) => c.visible,
            Constraint::Joint(c) => c.visible,
            Constraint::Tie(_) | Constraint::NodeTie(_) => true,
        }
    }

    /// The visibility of the constraint's symbol, for constraints that have one.
    pub fn visible_mut(&mut self) -> Option<&mut bool> {
        match self {
            Constraint::Spring(c) => Some(&mut c.visible),
            Constraint::CompressionOnly(c) => Some(&mut c.visible),
            Constraint::RigidBody(c) => Some(&mut c.visible),
            Constraint::Joint(c) => Some(&mut c.visible),
            Constraint::Tie(_) | Constraint::NodeTie(_) => None,
        }
    }

    pub fn name_mut(&mut self) -> &mut String {
        match self {
            Constraint::Spring(c) => &mut c.name,
            Constraint::CompressionOnly(c) => &mut c.name,
            Constraint::Tie(tie) => &mut tie.name,
            Constraint::RigidBody(c) => &mut c.name,
            Constraint::Joint(c) => &mut c.name,
            Constraint::NodeTie(c) => &mut c.name,
        }
    }

    /// Swaps master and slave of a tie or spring connection, also in a name
    /// `<master>_to_<slave>`; returns false for constraints without the two.
    pub fn swap_master_slave(&mut self) -> bool {
        let (name, master, slave) = match self {
            Constraint::Tie(tie) => (&mut tie.name, &mut tie.master, &mut tie.slave),
            Constraint::Spring(spring) if spring.master_slave().is_some() => {
                spring.ends.swap(0, 1);
                spring.name = swapped_name(&spring.name);
                return true;
            }
            _ => return false,
        };
        std::mem::swap(master, slave);
        *name = swapped_name(name);
        true
    }

    /// Master and slave region of a tie or spring connection.
    pub fn master_slave(&self) -> Option<[&Region; 2]> {
        match self {
            Constraint::Tie(tie) => Some([&tie.master, &tie.slave]),
            Constraint::Spring(c) => c.master_slave(),
            _ => None,
        }
    }

    /// The regions the constraint is defined on, master before slave.
    pub fn regions(&self) -> Vec<&Region> {
        match self {
            Constraint::Spring(c) => c.ends.iter().rev().filter_map(SpringEnd::region).collect(),
            Constraint::CompressionOnly(c) => vec![&c.region],
            Constraint::Tie(tie) => vec![&tie.master, &tie.slave],
            Constraint::RigidBody(c) => vec![&c.region],
            Constraint::Joint(c) => c.ends.iter().rev().filter_map(SpringEnd::region).collect(),
            Constraint::NodeTie(c) => vec![&c.region],
        }
    }

    pub fn regions_mut(&mut self) -> Vec<&mut Region> {
        match self {
            Constraint::Spring(c) => (c.ends.iter_mut().rev())
                .filter_map(SpringEnd::region_mut)
                .collect(),
            Constraint::CompressionOnly(c) => vec![&mut c.region],
            Constraint::Tie(tie) => vec![&mut tie.master, &mut tie.slave],
            Constraint::RigidBody(c) => vec![&mut c.region],
            Constraint::Joint(c) => (c.ends.iter_mut().rev())
                .filter_map(SpringEnd::region_mut)
                .collect(),
            Constraint::NodeTie(c) => vec![&mut c.region],
        }
    }
}

/// Slave surface glued to a master surface (`*TIE`), e.g. parts meshed separately.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Tie {
    pub name: String,
    #[serde(default = "crate::active")]
    pub active: bool,
    /// Distance within which slave nodes are tied; `None` lets the solver choose.
    pub position_tolerance: Option<f64>,
    /// Moves the tied slave nodes onto the master surface.
    pub adjust: bool,
    pub master: Region,
    pub slave: Region,
    /// Makes the tie the cyclic symmetry of a sector: the slave side is the master side
    /// rotated by one sector about the axis (`*TIE, CYCLIC SYMMETRY`).
    #[serde(default)]
    pub cyclic: Option<CyclicSymmetry>,
}

/// A sector of a part that repeats about an axis, such as a blade of an impeller
/// (`*CYCLIC SYMMETRY MODEL`). Frequency steps then compute the modes by nodal diameter.
/// The number of sectors is not stored: it follows from the angle between the two cut faces
/// ([`CyclicSymmetry::sectors`]), as CalculiX solves only sectors that close the full circle.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CyclicSymmetry {
    pub axis_point: [f64; 3],
    pub axis_direction: [f64; 3],
    /// What the results show (`NGRAPH=`).
    pub display: SectorDisplay,
}

/// The sectors the results of a cyclic symmetric model are expanded to.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum SectorDisplay {
    /// Only the modelled sector (`NGRAPH=1`).
    Single,
    /// The whole part (`NGRAPH=N`).
    #[default]
    All,
}

impl Default for CyclicSymmetry {
    fn default() -> Self {
        Self {
            axis_point: [0.0; 3],
            axis_direction: [0.0, 0.0, 1.0],
            display: SectorDisplay::All,
        }
    }
}

/// How far the angle between the cut faces may be off 360 degrees over a whole number of
/// sectors, relative to that angle.
const SECTOR_ANGLE_TOLERANCE: f64 = 2e-3;

impl CyclicSymmetry {
    /// The highest nodal diameter `sectors` sectors can carry.
    pub fn highest_nodal_diameter(sectors: u32) -> u32 {
        sectors / 2
    }

    /// Why the solver would reject the settings, whatever the sides.
    pub fn problem(&self) -> Option<String> {
        if self.axis_direction.iter().all(|v| *v == 0.0) {
            Some("The axis direction must not be the zero vector.".into())
        } else {
            None
        }
    }

    /// The angle in radians between the master and the slave side about the axis, from
    /// the mean direction of their points seen from the axis. On cut faces through the
    /// axis all points of a side lie in the same direction, so the mesh does not matter.
    pub fn sector_angle(&self, master: &[[f64; 3]], slave: &[[f64; 3]]) -> Result<f64, String> {
        if let Some(problem) = self.problem() {
            return Err(problem);
        }
        if master.is_empty() || slave.is_empty() {
            return Err("Pick both cut faces of the sector to find the number of sectors.".into());
        }
        let axis = glam::DVec3::from(self.axis_direction).normalize();
        let point = glam::DVec3::from(self.axis_point);
        let size = (master.iter().chain(slave))
            .map(|p| (glam::DVec3::from(*p) - point).length())
            .fold(0.0, f64::max);
        // Points on the axis have no direction.
        let direction = |points: &[[f64; 3]]| -> glam::DVec3 {
            let radial = |p: &[f64; 3]| {
                let r = glam::DVec3::from(*p) - point;
                r - r.dot(axis) * axis
            };
            (points.iter().map(radial))
                .filter(|r| r.length() > 1e-9 * size)
                .map(glam::DVec3::normalize)
                .sum()
        };
        let (m, s) = (direction(master), direction(slave));
        if m.length() < 1e-6 || s.length() < 1e-6 {
            return Err("A cut face lies on the axis. Check the axis point and direction.".into());
        }
        Ok(m.cross(s).dot(axis).atan2(m.dot(s)).abs())
    }

    /// The number of sectors in 360 degrees for the cut faces `master` and `slave` (`N=`).
    pub fn sectors(&self, master: &[[f64; 3]], slave: &[[f64; 3]]) -> Result<u32, String> {
        let angle = self.sector_angle(master, slave)?;
        let degrees = angle.to_degrees();
        if degrees < 1e-6 {
            return Err(
                "The cut faces lie in the same direction from the axis. Check the \
                        axis point and direction."
                    .into(),
            );
        }
        let sectors = 360.0 / degrees;
        let whole = sectors.round();
        if whole < 2.0 || (360.0 / whole - degrees).abs() > SECTOR_ANGLE_TOLERANCE * degrees {
            return Err(format!(
                "The cut faces are {} degrees apart, which does not divide 360 degrees \
                 (360 / {} = {}). The solver needs sectors that close the full circle; check \
                 the cut faces and the axis.",
                round_to(degrees, 4),
                round_to(degrees, 4),
                round_to(sectors, 3)
            ));
        }
        Ok(whole as u32)
    }
}

/// `value` with at most `digits` decimals, for messages.
fn round_to(value: f64, digits: i32) -> f64 {
    let factor = 10f64.powi(digits);
    (value * factor).round() / factor
}

impl Tie {
    /// The number of sectors of a cyclic symmetry tie, from the angle between its sides;
    /// `None` for an ordinary tie.
    pub fn cyclic_sectors(&self, mesh: &plx_mesh::FeMesh) -> Option<Result<u32, String>> {
        let cyclic = self.cyclic.as_ref()?;
        let points = |region: &Region| -> Vec<[f64; 3]> {
            (region.nodes(mesh).into_iter())
                .filter_map(|n| mesh.node(n))
                .collect()
        };
        Some(cyclic.sectors(&points(&self.master), &points(&self.slave)))
    }

    /// Turns the tie around, as [`ContactPair::swap_master_slave`].
    pub fn swap_master_slave(&mut self) {
        std::mem::swap(&mut self.master, &mut self.slave);
        self.name = swapped_name(&self.name);
    }

    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            active: true,
            position_tolerance: None,
            adjust: true,
            master: Region::Faces(Vec::new()),
            slave: Region::Faces(Vec::new()),
            cyclic: None,
        }
    }
}

/// Factor f of the normal stiffness K = f E / h of a penalty contact whose surface behavior
/// leaves it open (Hard): E is the smaller Young's modulus of the two sides, h the mean edge
/// length of the slave faces. CalculiX's own default, 50 E, is not a stiffness per length and
/// so depends on the unit system: in SI (as Nodalis writes) it is about 1000 times softer
/// than in mm. f E / h is the same in every unit system.
pub const NORMAL_STIFFNESS_FACTOR: f64 = 10.0;

/// Stick slope of a friction without one, as a fraction of the normal stiffness; the
/// CalculiX manual recommends about 1/100.
pub const STICK_SLOPE_RATIO: f64 = 0.01;

/// What the stiffness of a contact between two surfaces scales with.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactScale {
    /// The smaller Young's modulus of the materials on both sides.
    pub young: f64,
    /// Mean edge length of the slave faces.
    pub edge_length: f64,
}

/// Young's modulus of each element with an elastic material, for [`ContactScale::with`].
pub type ElementYoung = std::collections::HashMap<plx_mesh::ElementId, f64>;

impl ContactScale {
    /// Of a master and a slave region, see [`Self::with`]. For many regions of a model, look
    /// up [`Self::element_young`] once and use [`Self::with`].
    pub fn of(
        model: &crate::FeModel,
        mesh: &plx_mesh::FeMesh,
        master: &Region,
        slave: &Region,
    ) -> Option<Self> {
        Self::with(&Self::element_young(model, mesh), mesh, master, slave)
    }

    /// Young's modulus of each element of a section with an elastic material; the smaller
    /// one for an element in several sections.
    pub fn element_young(model: &crate::FeModel, mesh: &plx_mesh::FeMesh) -> ElementYoung {
        let mut young = ElementYoung::new();
        for section in &model.sections {
            let Some(e) = (section.material.and_then(|m| model.material(m)))
                .and_then(|m| m.small_strain_elastic())
                .map(|elastic| elastic.young)
                .filter(|&e| e > 0.0)
            else {
                continue;
            };
            for id in section.region.elements(mesh) {
                let value = young.entry(id).or_insert(e);
                *value = value.min(e);
            }
        }
        young
    }

    /// Of a master and a slave region: the smaller Young's modulus of the elements the faces
    /// of both regions belong to, and the mean edge length of the slave faces (of the master
    /// faces for a slave of nodes). `None` without element faces or without an elastic
    /// material on them.
    pub fn with(
        young: &ElementYoung,
        mesh: &plx_mesh::FeMesh,
        master: &Region,
        slave: &Region,
    ) -> Option<Self> {
        let master_faces = master.faces(mesh);
        let slave_faces = slave.faces(mesh);
        let edges = if slave_faces.is_empty() {
            &master_faces
        } else {
            &slave_faces
        };
        let lengths: Vec<f64> = (edges.iter())
            .filter_map(|&(id, face)| face_edge_length(mesh, id, face))
            .collect();
        if lengths.is_empty() {
            return None;
        }
        let edge_length = lengths.iter().sum::<f64>() / lengths.len() as f64;
        let young = (master_faces.iter().chain(&slave_faces))
            .filter_map(|(id, _)| young.get(id).copied())
            .reduce(f64::min)?;
        (edge_length > 0.0).then_some(Self { young, edge_length })
    }

    /// The normal stiffness K = f E / h, see [`NORMAL_STIFFNESS_FACTOR`].
    pub fn normal_stiffness(self) -> f64 {
        NORMAL_STIFFNESS_FACTOR * self.young / self.edge_length
    }
}

/// Stick slope of a friction without one, from the normal stiffness of the contact: all of it
/// for a tied contact, which should not slide at all, otherwise [`STICK_SLOPE_RATIO`] of it.
pub fn default_stick_slope(normal_stiffness: f64, tied: bool) -> f64 {
    if tied {
        normal_stiffness
    } else {
        STICK_SLOPE_RATIO * normal_stiffness
    }
}

impl SurfaceInteraction {
    /// The normal stiffness the surface behavior gives itself (Linear and Tied K); `None` for
    /// the others, which leave it to [`ContactScale::normal_stiffness`].
    pub fn own_normal_stiffness(&self) -> Option<f64> {
        (self.properties.iter()).find_map(|p| match p {
            InteractionProperty::SurfaceBehavior(SurfaceBehavior::Linear { k, .. })
            | InteractionProperty::SurfaceBehavior(SurfaceBehavior::Tied { k }) => Some(*k),
            _ => None,
        })
    }
}

/// Mean length of the edges around a face of an element, of the corner nodes; the length of
/// the face of a 2D element, which is an edge.
fn face_edge_length(mesh: &plx_mesh::FeMesh, id: plx_mesh::ElementId, face: u8) -> Option<f64> {
    let element = mesh.element(id)?;
    let topology = element.faces().get(usize::from(face).checked_sub(1)?)?;
    let points: Vec<[f64; 3]> = (topology.corners.iter())
        .map(|&i| mesh.node(*element.nodes.get(i)?))
        .collect::<Option<_>>()?;
    let distance =
        |a: [f64; 3], b: [f64; 3]| ((0..3).map(|i| (b[i] - a[i]).powi(2)).sum::<f64>()).sqrt();
    match points[..] {
        [] | [_] => None,
        [a, b] => Some(distance(a, b)),
        _ => {
            let n = points.len();
            Some(
                (0..n)
                    .map(|i| distance(points[i], points[(i + 1) % n]))
                    .sum::<f64>()
                    / n as f64,
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two cubes of 2 x 2 x 2 elements of size `size`, one on top of the other, steel below
    /// and aluminium above; the top faces of the lower and the bottom faces of the upper one.
    fn stacked_cubes(size: f64) -> (crate::FeModel, plx_mesh::FeMesh, Region, Region) {
        use plx_mesh::{Element, ElementShape, FeMesh, Part};
        let mut mesh = FeMesh::default();
        let mut faces = [Vec::new(), Vec::new()];
        for block in 0..2u32 {
            let node = |i: u32, j: u32, k: u32| 100 * block + 1 + i + 3 * j + 9 * k;
            for k in 0..3 {
                for j in 0..3 {
                    for i in 0..3 {
                        let z = f64::from(2 * block + k) * size;
                        mesh.set_node(node(i, j, k), [f64::from(i) * size, f64::from(j) * size, z]);
                    }
                }
            }
            let mut elements = Vec::new();
            for k in 0..2 {
                for j in 0..2 {
                    for i in 0..2 {
                        let id = 100 * block + 1 + i + 2 * j + 4 * k;
                        let mut nodes = Vec::new();
                        for k in [k, k + 1] {
                            nodes.extend([node(i, j, k), node(i + 1, j, k)]);
                            nodes.extend([node(i + 1, j + 1, k), node(i, j + 1, k)]);
                        }
                        let element = Element {
                            id,
                            type_name: "C3D8".into(),
                            shape: ElementShape::Hex8,
                            nodes,
                        };
                        mesh.add_element(element).unwrap();
                        elements.push(id);
                        if (block, k) == (0, 1) {
                            faces[0].push((id, 2));
                        } else if (block, k) == (1, 0) {
                            faces[1].push((id, 1));
                        }
                    }
                }
            }
            let name = ["LOWER", "UPPER"][block as usize];
            mesh.parts.push(Part {
                bodies: Vec::new(),
                name: name.into(),
                elements,
            });
        }
        let mut model = crate::FeModel::default();
        for (name, young, part) in [("Steel", 210e9, "LOWER"), ("Aluminium", 70e9, "UPPER")] {
            model.materials.push(crate::Material {
                name: name.into(),
                elastic: Some(crate::Elastic {
                    young,
                    poisson: 0.3,
                }),
                ..Default::default()
            });
            model.sections.push(crate::Section {
                orientation: None,
                name: part.into(),
                material: model.materials.last().map(|m| m.id),
                region: Region::Parts(vec![part.into()]),
                thickness: 1.0,
                kind: crate::SectionKind::Solid,
            });
        }
        let [lower, upper] = faces;
        (model, mesh, Region::Faces(lower), Region::Faces(upper))
    }

    #[test]
    fn contact_stiffness_grows_with_the_softer_modulus_over_the_element_size() {
        let (model, mesh, master, slave) = stacked_cubes(0.005);
        let scale = ContactScale::of(&model, &mesh, &master, &slave).unwrap();
        assert_eq!(scale.young, 70e9);
        assert!((scale.edge_length - 0.005).abs() < 1e-12);
        assert!((scale.normal_stiffness() / (10.0 * 70e9 / 0.005) - 1.0).abs() < 1e-12);
        // The same model in mm and MPa has the same stiffness, in N/mm^3 = 1e9 N/m^3.
        let (mut model_mm, mesh_mm, ..) = stacked_cubes(5.0);
        for material in &mut model_mm.materials {
            material.elastic.as_mut().unwrap().young /= 1e6;
        }
        let k_mm = ContactScale::of(&model_mm, &mesh_mm, &master, &slave)
            .unwrap()
            .normal_stiffness();
        assert!((k_mm * 1e9 / scale.normal_stiffness() - 1.0).abs() < 1e-9);
        // Without faces or materials there is nothing to scale with.
        let nodes = Region::Nodes(vec![1]);
        assert!(ContactScale::of(&model, &mesh, &nodes, &nodes).is_none());
        assert!(ContactScale::of(&crate::FeModel::default(), &mesh, &master, &slave).is_none());
        // A slave of nodes takes the edge length of the master faces.
        assert!(ContactScale::of(&model, &mesh, &master, &nodes).is_some());
    }

    #[test]
    fn a_tied_contact_sticks_with_the_whole_normal_stiffness() {
        assert_eq!(default_stick_slope(1e12, true), 1e12);
        assert_eq!(default_stick_slope(1e12, false), 1e10);
        let tied = SurfaceInteraction {
            id: crate::Id::new(),
            name: "Tied".into(),
            properties: vec![
                InteractionProperty::SurfaceBehavior(SurfaceBehavior::Tied { k: 5e15 }),
                InteractionProperty::Friction(Friction::default()),
            ],
        };
        assert_eq!(tied.own_normal_stiffness(), Some(5e15));
        let hard = SurfaceInteraction {
            id: crate::Id::new(),
            name: "Hard".into(),
            properties: vec![InteractionProperty::SurfaceBehavior(SurfaceBehavior::Hard)],
        };
        assert_eq!(hard.own_normal_stiffness(), None);
    }

    #[test]
    fn constraints_saved_before_hiding_are_visible() {
        let text = r#"RigidBody((name: "Rigid_Body-1", region: Parts([]), reference_point: 7))"#;
        let mut constraint: Constraint = ron::from_str(text).unwrap();
        assert!(constraint.visible());
        *constraint.visible_mut().unwrap() = false;
        assert!(!constraint.visible());
        assert!(Constraint::Tie(Tie::new("Tie-1")).visible_mut().is_none());
    }

    #[test]
    fn swapping_turns_regions_and_a_pair_name_around() {
        let mut pair = ContactPair::new("BEAM_to_DISC", None);
        pair.master = Region::Surface("TOP".into());
        pair.slave = Region::Surface("BOTTOM".into());
        pair.swap_master_slave();
        assert_eq!(pair.name, "DISC_to_BEAM");
        assert_eq!(pair.master, Region::Surface("BOTTOM".into()));
        assert_eq!(pair.slave, Region::Surface("TOP".into()));
        let mut tie = Constraint::Tie(Tie::new("Bearing"));
        assert!(tie.swap_master_slave());
        assert_eq!(tie.name(), "Bearing");
        assert_eq!(swapped_name("A_to_B_to_C"), "A_to_B_to_C");
    }
}
