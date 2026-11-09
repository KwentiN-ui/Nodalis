//! Contact (concept section 1.4): surface interactions and the pairs that use them.

use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Contact {
    pub interactions: Vec<SurfaceInteraction>,
    pub pairs: Vec<ContactPair>,
    pub clearances: Vec<Clearance>,
    /// `*COMPRESSION ONLY`: surfaces held against fixed ground in compression only.
    #[serde(default)]
    pub compression_only: Vec<CompressionOnly>,
}

/// `*SURFACE INTERACTION` with its `*SURFACE BEHAVIOR`, `*FRICTION` and `*GAP CONDUCTANCE`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SurfaceInteraction {
    /// Upper case name.
    pub name: String,
    pub behavior: Option<SurfaceBehavior>,
    pub friction: Option<Friction>,
    pub gap_conductance: Option<GapConductance>,
}

/// `*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=`: the pressure as a function of the
/// overclosure (negative gap).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SurfaceBehavior {
    /// `HARD`: no penetration (Lagrange in mortar, a stiff penalty otherwise).
    Hard,
    /// `LINEAR`: stiffness `k`; `sigma_inf` the tension at infinite distance and `c0` the
    /// distance from which the surfaces are pulled together (node-to-surface only).
    Linear {
        k: f64,
        sigma_inf: Option<f64>,
        c0: Option<f64>,
    },
    /// `EXPONENTIAL`: pressure p0 at zero gap, decaying to zero at distance c0.
    Exponential { c0: f64, p0: f64 },
    /// `TABULAR`: (pressure, overclosure) points.
    Tabular(Vec<(f64, f64)>),
    /// `TIED`: no separation and no sliding, stiffness `k`.
    Tied { k: f64 },
}

/// `*FRICTION`: Coulomb friction coefficient and the stick slope (tangential stiffness
/// before sliding).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Friction {
    pub coefficient: f64,
    pub stick_slope: Option<f64>,
}

/// `*GAP CONDUCTANCE`: heat transfer across the contact, constant or (conductance, pressure,
/// temperature) rows; `USER` is not supported.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum GapConductance {
    Constant(f64),
    Table(Vec<(f64, f64, f64)>),
}

/// `*CONTACT PAIR`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ContactPair {
    pub interaction: String,
    pub kind: ContactKind,
    pub small_sliding: bool,
    /// `ADJUST=<value>` moves slave nodes within the distance on to the master surface,
    /// `ADJUST=<nset>` those of the set.
    pub adjust: Option<Adjust>,
    pub slave: String,
    pub master: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContactKind {
    NodeToSurface,
    SurfaceToSurface,
    Mortar,
    /// `TYPE=LINMORTAR`, `PGLINMORTAR`: not written by Nodalis, kept for completeness.
    LinearMortar,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Adjust {
    Distance(f64),
    NodeSet(String),
}

/// `*CLEARANCE`: an initial clearance between the surfaces of a pair, overriding their
/// distance in the mesh.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Clearance {
    pub master: String,
    pub slave: String,
    pub value: f64,
}

/// `*COMPRESSION ONLY, SURFACE=`: a support that pushes on an element surface and never pulls,
/// like a part resting on a rigid floor. Unlike `GAPUNI` elements at the nodes it acts at
/// the integration points of the faces, so that the pressure is spread over the faces as a
/// pressure load is: rigid nodal gaps on quadratic faces leave the corner nodes without
/// load (their consistent share of a pressure is negative) and the stresses checkered.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CompressionOnly {
    /// Upper case name of the element surface.
    pub surface: String,
    /// Distance the surface can move towards the ground before the support takes load.
    pub clearance: f64,
    /// Stiffness per area (pressure per closure); `None`: practically rigid, see the
    /// assembly.
    pub stiffness: Option<f64>,
    /// The ground lies inside the material and pushes the surface outward (`FLIP`),
    /// instead of lying outside and pushing onto it.
    pub flipped: bool,
}
