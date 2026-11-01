//! Checks of the model before CalculiX runs, for the mistakes that most often make a run abort
//! or give useless results: parts that are not held, elements without a material, materials
//! without the constants a step needs, distorted elements, contradicting boundary conditions.
//! Each finding names the tree item it belongs to and explains the problem and its fix.
//!
//! The checks are derived from the model like [`crate::validity`]; the expensive part that
//! only depends on the mesh (connectivity and element distortion) is computed once in
//! [`MeshCheck`]. CalculiX's own error messages after a failed run are mapped to the same
//! explanations by [`diagnose_solver_output`].

use std::collections::{BTreeMap, HashMap, HashSet};

use plx_mesh::{ElementFamily, ElementId, FeMesh, NodeId};

use crate::{Formulation, GLOBAL, Region, SymmetryPlane, symmetry_dofs, symmetry_plane};

use crate::{
    BoundaryKind, Constraint, ContactMethod, DefinedFieldKind, FeModel, LoadKind, Material,
    ModelItem, ModelSpace, SectionKind, StepKind,
};

/// Whether CalculiX aborts or the results are likely wrong.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// CalculiX aborts, or the results are meaningless.
    Error,
    /// The run may work, but the model probably does not do what was intended.
    Warning,
}

/// A kind of problem, with the explanation shown when its warning sign is clicked.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Problem {
    MissingReference,
    /// A section whose type does not fit its elements, such as a beam section on solids or
    /// a membrane section on quadratic elements, or whose values CalculiX rejects.
    InvalidSection,
    NoSection,
    NoElastic,
    NoDensity,
    /// A boundary condition, load or history output on a reference point that no active
    /// rigid body is driven by: the point has no node in the input file.
    NoRigidBody,
    /// Nodes in two rigid bodies, of rigid body constraints or displacements on a point:
    /// CalculiX keeps them in the first and only warns.
    SharedRigidBody,
    /// Nodes of a rigid body constraint that a boundary condition holds directly: CalculiX
    /// refuses them ("dependent side of a MPC and a SPC"), so the input file leaves them
    /// out of the rigid body.
    HeldRigidBodyNodes,
    /// Slave nodes of a tie CalculiX did not tie: the master side lies opposite them but
    /// further away than the position tolerance. Found in the output of a run.
    TieNodesNotTied,
    /// Beam, truss, shell or membrane nodes in a rigid body or rigid displacement point:
    /// CalculiX 2.23 refuses them in a `*RIGID BODY`, so they are coupled by linear
    /// equations, which hold for small rotations only. Warned of in steps with large
    /// deformations, and found in the output of a CalculiX run of another input file.
    RigidBeamNodes,
    /// A plastic hardening curve CalculiX rejects: no elasticity, a row without a yield
    /// stress, or plastic strains that do not start at 0 and grow.
    InvalidPlastic,
    /// A creep law with nonpositive constants, or without elasticity.
    InvalidCreep,
    NoConductivity,
    NoSpecificHeat,
    NoInitialTemperature,
    InvalidElastic,
    DistortedElements,
    RigidBodyMotion,
    /// Truss nodes that no bar or support holds in some direction.
    Mechanism,
    /// A pre-tension load locked in the first step it acts in: there is no length of a
    /// previous step to keep, so CalculiX holds the bolt at its unstretched length.
    LockWithoutPreload,
    /// A bolt whose cut cannot be made: no faces of solid elements, or a side missing.
    InvalidBolt,
    /// A bolt without a preload: zero, or no step that takes one.
    BoltWithoutPreload,
    /// The faces of a bolt's cut also in a tie or contact pair defined by hand: CalculiX
    /// then loops forever or does not converge.
    BoltFacesUsedTwice,
    /// A bonding contact region at a bolt's cut, such as the shank bonded into its hole:
    /// it holds the shank, and the bolt cannot stretch.
    BoltHeldAtCut,
    /// Beams held by pins only, free to turn about the line through them.
    SpinAboutPins,
    HeldOnlyByContact,
    ConflictingBoundaries,
    LoadOnFixedNodes,
    RotationsIgnored,
    /// Beam nodes with some but not all of their rotations given: CalculiX holds them all.
    PartialBeamRotations,
    /// A submodel boundary condition in a model without a global results file.
    NoGlobalResults,
    /// A symmetry boundary condition whose nodes do not fix a plane.
    NoSymmetryPlane,
    /// A symmetry boundary condition whose nodes are not on one plane.
    SymmetryNotPlanar,
    /// A symmetry plane of an axisymmetric model that is not perpendicular to the axis.
    SymmetryOffAxis,
    /// Nodes of a symmetry plane that is no coordinate plane, which get a local coordinate
    /// system, also used by another boundary condition or a point load.
    SkewedSymmetryShared,
    IncrementExceedsStep,
    NoLoad,
    /// An amplitude that no boundary condition, load or defined field uses.
    UnusedAmplitude,
    /// A distribution that no pressure or temperature uses.
    UnusedDistribution,
    /// A radiation load in a model without absolute zero or Stefan-Boltzmann constant.
    NoPhysicalConstants,
    /// A defined temperature in a step whose materials have no thermal expansion.
    NoExpansion,
    /// A step that builds on stored eigenmodes (modal dynamics, steady state dynamics,
    /// complex frequency) without a frequency step before it that stores them.
    NoStoredModes,
    /// A sensitivity step without the static or frequency step before it whose solution
    /// its design responses need.
    NoSensitivityBase,
    /// A later sensitivity step with another design region: CalculiX reads one, the first.
    DesignRegionIgnored,
    /// A Visco step in a model whose materials have no creep law: it is a static step.
    NoCreep,
    /// A step CalculiX's general beam (user element U1) cannot take: large deformations,
    /// buckling, direct dynamics, heat transfer.
    GeneralBeamStep,
    /// A cyclic symmetry tie CalculiX rejects, or nodal diameters beyond what its sectors
    /// carry.
    InvalidCyclicSymmetry,
    /// An initial velocity in a model without a Dynamic step.
    VelocityIgnored,
    /// A Coriolis complex frequency step without a centrifugal load before it.
    NoRotation,
    /// Contact pairs in a frequency step without perturbation: CalculiX ignores them.
    ContactIgnored,
    /// Contact pairs in a complex frequency step: the Coriolis coupling goes wrong.
    ContactWithCoriolis,
    /// A contact pair with tied surface behavior that CalculiX rejects: without friction in
    /// its surface interaction or with another method than surface to surface.
    InvalidTiedContact,
    /// Contact pairs of different formulations: CalculiX takes one for all of them.
    ContactFormulationsDiffer,
    /// Mortar contact in a step that is not static: CalculiX takes mortar in static steps
    /// only.
    MortarNotSupported,
    /// A contact region whose sides do not touch in the mesh: a bonded one ties nothing,
    /// another one lets its parts move freely until they meet.
    ContactRegionOpen,
    /// A region of no separation in a step with large deformations: its equations hold for
    /// small displacements only.
    NoSeparationLargeDeformation,
    /// A step switches a contact region off or on that CalculiX cannot switch: a tie, the
    /// equations of no separation, or a region deactivated as a whole.
    ContactChangeIgnored,
    /// A history output prints the contact values of a region that has none: a tie, the
    /// equations of no separation, or a region deactivated as a whole.
    ContactOutputIgnored,
    /// A wheel of a moving load that is off the selected surface at some positions.
    WheelOffSurface,
    /// Found in the solver output only.
    NoConvergence,
    /// Found in the solver output only.
    MpcAndSpc,
    /// Found in the solver output only.
    RotationIn2d,
    /// Any other `*ERROR` of the solver output.
    SolverError,
}

impl Problem {
    pub fn severity(self) -> Severity {
        match self {
            Problem::HeldOnlyByContact
            | Problem::ConflictingBoundaries
            | Problem::LoadOnFixedNodes
            | Problem::RotationsIgnored
            | Problem::PartialBeamRotations
            | Problem::SpinAboutPins
            | Problem::SymmetryNotPlanar
            | Problem::SymmetryOffAxis
            | Problem::SkewedSymmetryShared
            | Problem::VelocityIgnored
            | Problem::NoLoad
            | Problem::UnusedAmplitude
            | Problem::UnusedDistribution
            | Problem::NoExpansion
            | Problem::NoRotation
            | Problem::ContactIgnored
            | Problem::ContactWithCoriolis
            | Problem::WheelOffSurface
            | Problem::ContactRegionOpen
            | Problem::NoSeparationLargeDeformation
            | Problem::ContactChangeIgnored
            | Problem::ContactOutputIgnored
            | Problem::DesignRegionIgnored
            | Problem::RigidBeamNodes
            | Problem::HeldRigidBodyNodes
            | Problem::TieNodesNotTied
            | Problem::LockWithoutPreload
            | Problem::BoltWithoutPreload
            | Problem::BoltHeldAtCut
            | Problem::NoCreep => Severity::Warning,
            _ => Severity::Error,
        }
    }

    pub fn title(self) -> &'static str {
        match self {
            Problem::MissingReference => "Missing reference",
            Problem::InvalidSection => "Section does not fit its elements",
            Problem::NoSection => "Elements without material",
            Problem::NoElastic => "Material without elasticity",
            Problem::NoDensity => "Material without density",
            Problem::NoConductivity => "Material without conductivity",
            Problem::NoSpecificHeat => "Material without specific heat",
            Problem::NoInitialTemperature => "No initial temperature",
            Problem::InvalidElastic => "Invalid material constants",
            Problem::DistortedElements => "Distorted elements",
            Problem::RigidBodyMotion => "Rigid body motion possible",
            Problem::Mechanism => "Truss structure is a mechanism",
            Problem::SpinAboutPins => "Beams free to turn about their pins",
            Problem::HeldOnlyByContact => "Held only by contact",
            Problem::ConflictingBoundaries => "Conflicting boundary conditions",
            Problem::LoadOnFixedNodes => "Load on fixed nodes",
            Problem::RotationsIgnored => "Rotations without effect",
            Problem::PartialBeamRotations => "Beam rotations partly held",
            Problem::NoGlobalResults => "No global results",
            Problem::NoSymmetryPlane => "No symmetry plane",
            Problem::SymmetryNotPlanar => "Symmetry region not planar",
            Problem::SymmetryOffAxis => "Symmetry plane not perpendicular to the axis",
            Problem::SkewedSymmetryShared => "Nodes of a skewed symmetry plane shared",
            Problem::IncrementExceedsStep => "Increment larger than the step",
            Problem::NoLoad => "No load",
            Problem::UnusedAmplitude => "Amplitude not used",
            Problem::UnusedDistribution => "Distribution not used",
            Problem::NoPhysicalConstants => "Physical constants missing",
            Problem::NoRigidBody => "Reference point without rigid body",
            Problem::SharedRigidBody => "Nodes in several rigid bodies or remote points",
            Problem::HeldRigidBodyNodes => "Rigid body on nodes held by a boundary condition",
            Problem::TieNodesNotTied => "Nodes of a bonded contact not tied",
            Problem::RigidBeamNodes => "Beam or shell nodes in a rigid body",
            Problem::InvalidPlastic => "Invalid plasticity",
            Problem::InvalidCreep => "Invalid creep law",
            Problem::NoExpansion => "Temperature without thermal expansion",
            Problem::NoStoredModes => "No stored eigenmodes",
            Problem::NoSensitivityBase => "No step for the sensitivities",
            Problem::DesignRegionIgnored => "Design region ignored",
            Problem::NoCreep => "Visco step without creep",
            Problem::GeneralBeamStep => "Step not possible with general beam profiles",
            Problem::LockWithoutPreload => "Bolt locked before it is preloaded",
            Problem::InvalidBolt => "Bolt cannot be cut",
            Problem::BoltWithoutPreload => "Bolt without preload",
            Problem::BoltFacesUsedTwice => "Faces of a bolt's cut used twice",
            Problem::BoltHeldAtCut => "Bolt held at its cut",
            Problem::InvalidCyclicSymmetry => "Invalid cyclic symmetry",
            Problem::VelocityIgnored => "Initial velocity without a Dynamic step",
            Problem::NoRotation => "No rotation",
            Problem::ContactIgnored => "Contact ignored",
            Problem::ContactWithCoriolis => "Contact with Coriolis",
            Problem::InvalidTiedContact => "Tied contact not possible like this",
            Problem::ContactFormulationsDiffer => "Contact formulations differ",
            Problem::MortarNotSupported => "Mortar contact not possible in this step",
            Problem::ContactRegionOpen => "Contact region open",
            Problem::NoSeparationLargeDeformation => "No separation with large deformations",
            Problem::ContactChangeIgnored => "Contact cannot change in steps",
            Problem::ContactOutputIgnored => "No contact values to print",
            Problem::WheelOffSurface => "Wheel off the surface",
            Problem::NoConvergence => "No convergence",
            Problem::MpcAndSpc => "Degree of freedom bound twice",
            Problem::RotationIn2d => "Rotation in a 2D model",
            Problem::SolverError => "Solver error",
        }
    }

    /// What goes wrong and why CalculiX cannot cope with it.
    pub fn explanation(self) -> &'static str {
        match self {
            Problem::MissingReference => {
                "The item refers to something that no longer exists, for example a \
                 deleted material or a set the mesh no longer has. The input file \
                 cannot be written like this."
            }
            Problem::InvalidSection => {
                "The type of the section and the elements of its region do not go together: \
                 beams and trusses need line elements, solid sections solid or plane \
                 elements, shell and membrane sections surface elements of a 3D model. \
                 Membranes must be linear, since CalculiX computes quadratic membranes \
                 (M3D6, M3D8) wrongly. Dimensions such as a thickness of 0 are rejected too. \
                 The input file cannot be written like this."
            }
            Problem::NoSection => {
                "CalculiX needs a material for every element. Elements that no section \
                 covers abort the run with \"no material was assigned to element\"."
            }
            Problem::NoElastic => {
                "A section uses this material, but it has no Young's modulus and no \
                 Poisson's ratio. Without them there is no stiffness; CalculiX aborts \
                 with \"no elastic constants were assigned\"."
            }
            Problem::NoDensity => {
                "A Frequency step, a Dynamic step, a transient heat transfer as well as \
                 gravity and centrifugal loads need the mass of the model. Without a \
                 density CalculiX aborts with \"no density was assigned\"."
            }
            Problem::RigidBeamNodes => {
                "CalculiX refuses nodes of beams, trusses, shells and membranes in a rigid \
                 body (\"nodes belonging to beam elements must not be subject to a RIGID \
                 MPC\"), because it expands these elements into solids. Nodalis therefore \
                 couples such nodes of a Rigid Body constraint or a rigid displacement point \
                 with linear equations to the point. They are exact for small rotations, as \
                 in a linear analysis; with large deformations (NLGEOM) and large rotations \
                 of the point these nodes lag behind the rigid motion. As with a rigid body, \
                 the rotations of beam and shell nodes are not coupled."
            }
            Problem::TieNodesNotTied => {
                "CalculiX ties a slave node of a bonded contact region or tie only if the \
                 master side is closer than the position tolerance. These nodes are further \
                 away: they stay unconnected, the parts can open there and the stresses next \
                 to them are wrong. CalculiX lists the nodes in the file \
                 <job>_WarnNodeMissTiedContact.nam of the working directory."
            }
            Problem::HeldRigidBodyNodes => {
                "A node of a rigid body follows its reference point; a boundary condition on \
                 it binds it a second time, which CalculiX refuses (\"dependent side of a MPC \
                 and a SPC\"). Nodalis leaves these nodes out of the rigid body: they keep \
                 their boundary condition, and the rigid body is not held by it."
            }
            Problem::SharedRigidBody => {
                "A node can follow one rigid body or remote point only. Rigid body constraints \
                 and displacements on a point each tie their selection to it; where two share \
                 nodes, Nodalis leaves these nodes in the first rigid body and the second \
                 moves without them."
            }
            Problem::NoRigidBody => {
                "A boundary condition or load on a reference point acts on the rigid body the \
                 point drives, a history output prints its values. Without an active rigid body constraint with this reference \
                 point there is no node to apply it to; the input file would be incomplete."
            }
            Problem::InvalidCreep => {
                "CalculiX reads Norton's creep law only after an isotropic elasticity and \
                 refuses a constant A or an exponent n that is not positive."
            }
            Problem::InvalidPlastic => {
                "The hardening curve of a plastic material lists the yield stress over the \
                 plastic strain. Every row needs a stress above 0, the first row of each \
                 temperature starts at plastic strain 0 and the strain grows from row to \
                 row; plasticity also needs the elastic constants. Otherwise CalculiX stops \
                 with \"*PLASTIC: the plastic strain must be increasing\" or gives a wrong \
                 stiffness."
            }
            Problem::NoConductivity => {
                "A heat transfer needs the thermal conductivity of every material. \
                 CalculiX only warns (\"no conductivity constants were assigned\") and \
                 then crashes while solving."
            }
            Problem::NoSpecificHeat => {
                "A transient heat transfer needs the specific heat. CalculiX aborts \
                 with \"no specific heat was assigned\"."
            }
            Problem::NoInitialTemperature => {
                "A transient heat transfer starts from an initial temperature, and a \
                 prescribed temperature (Defined Field) expands the model relative to it. \
                 Without it CalculiX aborts with \"please define initial conditions for \
                 the temperature\" or \"no thermal *INITIAL CONDITIONS are given\"."
            }
            Problem::InvalidElastic => {
                "Young's modulus must be greater than 0 and Poisson's ratio must lie \
                 between -1 and 0.5 (0.5 would be an incompressible material, which a \
                 linear elastic model cannot represent). CalculiX rejects the material \
                 when reading the input."
            }
            Problem::DistortedElements => {
                "These elements are inverted or so badly distorted that their Jacobian \
                 determinant is not positive at an integration point. CalculiX aborts \
                 with \"nonpositive jacobian determinant in element\". Typical causes \
                 are quadratic tetrahedra whose midside nodes were moved onto a strongly \
                 curved face, very thin regions, or 2D elements whose nodes are numbered \
                 clockwise."
            }
            Problem::SpinAboutPins => {
                "These beams are held by pins, nodes with all displacements and no rotation \
                 given, which leave them free to turn about the line through the pins. \
                 Nothing loads this rotation, and CalculiX solves such models in linear and \
                 nonlinear steps alike; the results are valid. Only a moment about that line \
                 would turn the beams without limit."
            }
            Problem::RigidBodyMotion => {
                "These parts can move as a rigid body: the boundary conditions and \
                 springs do not fix all displacements and rotations, and no Tie or \
                 tied contact connects them to supported parts. A Compression Only \
                 support holds only perpendicular to its surface: on a flat face the \
                 part can still slide and turn in the plane. The stiffness matrix is \
                 then singular. Depending on the matrix solver CalculiX aborts with \"zero \
                 pivot\" or returns huge, useless displacements without an error."
            }
            Problem::Mechanism => {
                "Trusses (Truss Section) carry axial force only. A node at which all \
                 trusses lie on one line or in one plane is not held across it: a truss \
                 of several elements buckles at every inner node without resistance, and \
                 a plane truss can move perpendicular to its plane. The stiffness matrix \
                 is singular; CalculiX does not report this but returns huge, useless \
                 displacements."
            }
            Problem::HeldOnlyByContact => {
                "These parts are held only by contact pairs without tied surface \
                 behavior. At the start of the \
                 analysis a contact carries load only once it is closed; until then the \
                 parts can drift away freely. This often leads to \"zero pivot\", to \
                 \"too many cutbacks\" or to very long analyses."
            }
            Problem::ConflictingBoundaries => {
                "In this step the same nodes are prescribed by several boundary \
                 conditions with different values, e.g. a face fixed and an adjacent \
                 edge with a displacement. CalculiX does not report this but uses the \
                 value written last; the result then depends on the order in the tree."
            }
            Problem::LoadOnFixedNodes => {
                "All nodes of this load are fixed in the load direction. The load goes \
                 directly into the reaction force and does not deform the model."
            }
            Problem::RotationsIgnored => {
                "Only beam and shell nodes have rotations (UR1 to UR3). Solid and 2D \
                 elements have displacements only; prescribed rotations are ignored or \
                 abort the run in 2D models (\"mpc of type is unknown\")."
            }
            Problem::PartialBeamRotations => {
                "CalculiX expands beams into solid elements. A beam node with a given \
                 rotation becomes a rigid knot of its cross section, and the knot is held \
                 in all three rotations even if only one of them is given: a pin that \
                 should only stop the twist acts as a clamp, and its reaction forces are \
                 wrong. In geometrically nonlinear steps (Nlgeom) the knot turns, and the \
                 increments mostly shrink until the step aborts without convergence."
            }
            Problem::NoGlobalResults => {
                "A submodel boundary condition takes its displacements from the results \
                 of a global model (*SUBMODEL). Without the global results file the input \
                 file cannot be written."
            }
            Problem::NoSymmetryPlane => {
                "A symmetry boundary condition holds its nodes in a plane, which it takes \
                 from the selected faces. A single node, nodes on a line (other than the \
                 edge of shells or a line of a 2D model) or a reference point do not fix \
                 a plane, so the input file cannot be written."
            }
            Problem::SymmetryNotPlanar => {
                "A symmetry plane cuts the model in a plane: the selected faces must be \
                 flat and lie in one plane. These nodes are not; the boundary condition \
                 holds them normal to the best-fitting plane, which forces curved or \
                 offset faces into a shape the real structure does not have."
            }
            Problem::SymmetryOffAxis => {
                "An axisymmetric model already stands for the full revolution about the y \
                 axis. A further symmetry can only be a mirror plane perpendicular to the \
                 axis, i.e. a line of constant y in the model. Holding other lines normal \
                 to themselves is a support, not a symmetry."
            }
            Problem::SkewedSymmetryShared => {
                "A symmetry plane that is not a coordinate plane is written with a local \
                 coordinate system for its nodes (*TRANSFORM). CalculiX then reads every \
                 boundary condition and point load at these nodes in that local system \
                 (x along the plane normal), and a node can have only one such system."
            }
            Problem::IncrementExceedsStep => {
                "The initial increment is larger than the time period of the step. \
                 CalculiX rejects the step with \"initial increment size exceeds step \
                 size\"."
            }
            Problem::WheelOffSurface => {
                "At some positions of the moving load a wheel is partly or fully off the \
                 selected surface. Its force is left out there, as when a crane drives off \
                 the end of its runway beam. Wanted for a vehicle that drives over the whole \
                 structure; a mistake if the wheels should stay on it."
            }
            Problem::NoLoad => {
                "The step has neither an active load nor a prescribed displacement. The \
                 analysis runs, but all results are zero."
            }
            Problem::UnusedDistribution => {
                "No pressure, temperature boundary condition, initial temperature or defined \
                 temperature refers to this distribution, so it has no effect."
            }
            Problem::UnusedAmplitude => {
                "No boundary condition, load or defined field refers to this amplitude, so it \
                 has no effect. Loads and boundary conditions without an amplitude follow \
                 CalculiX's default ramp or step."
            }
            Problem::NoExpansion => {
                "A defined temperature (Defined Field) deforms the model only through the \
                 thermal expansion of the materials. Without an expansion coefficient \
                 CalculiX runs, but the temperature has no effect."
            }
            Problem::NoStoredModes => {
                "Modal Dynamics, Steady State Dynamics and Complex Frequency steps work on \
                 the eigenmodes a previous Frequency step with Storage wrote to the .eig \
                 file. Without it CalculiX stops with \"error opening the eigenvalue file\" \
                 or \"the eigenvalue file does not exist\"."
            }
            Problem::NoSensitivityBase => {
                "A Sensitivity step differentiates the solution of the step right before it: \
                 strain energy needs a Static step, eigenfrequencies a Frequency step that \
                 stores its eigenmodes, mass either. Otherwise CalculiX stops (\"cannot open \
                 eigenvalue file\") or computes the wrong responses."
            }
            Problem::DesignRegionIgnored => {
                "CalculiX defines the design variables once for the model. Nodalis writes \
                 the design region of the first Sensitivity step; the regions of later ones \
                 are not used."
            }
            Problem::NoCreep => {
                "A Visco step lets the materials with a creep law (Norton) creep over its time \
                 period. Without such a material it computes the same as a Static step."
            }
            Problem::GeneralBeamStep => {
                "I, T, channel, angle and general profiles are not beam sections of CalculiX \
                 itself: they are written as general sections of its user element U1, a \
                 linear beam. It computes Static steps without large deformations (Nlgeom), \
                 Frequency steps (CalculiX 2.23, not 2.21) and the modal and steady state \
                 dynamics built on them; other steps stop CalculiX with an error."
            }
            Problem::InvalidCyclicSymmetry => {
                "A cyclic symmetry tie models a whole part by one sector of a 3D mesh. The \
                 number of sectors N is 360 degrees over the angle between the two cut faces \
                 about the axis, which must be a whole number: CalculiX solves only sectors \
                 that close the full circle. A model of N sectors carries the nodal diameters \
                 0 to N/2 only."
            }
            Problem::InvalidBolt => {
                "A bolt is cut between the faces of solid elements: the two faces of a bolt \
                 split into two bodies, or the face inside a bolt meshed in one piece. The \
                 bolt has no such faces, so it is left out of the input file."
            }
            Problem::BoltWithoutPreload => {
                "The bolt is preloaded in the first step that takes forces. Its preload is \
                 zero, or no step takes one (frequency and heat transfer steps do not), so the \
                 bolt only holds its two halves together."
            }
            Problem::BoltFacesUsedTwice => {
                "The bolt joins the two sides of its cut itself. A tie or contact pair on the \
                 same faces binds the nodes a second time: CalculiX then loops forever while \
                 resolving the constraints, or the contact does not converge."
            }
            Problem::BoltHeldAtCut => {
                "A bonded or rough contact region touches the cut of the \
                 bolt, for example the shank bonded into its hole. It holds the shank, so the \
                 preload compresses the region instead of stretching the bolt."
            }
            Problem::LockWithoutPreload => {
                "Lock keeps the length the bolt had at the end of the previous step. In the \
                 first step the pre-tension acts in there is none, and CalculiX holds the bolt \
                 at its unstretched length: the bolt carries no preload."
            }
            Problem::VelocityIgnored => {
                "Only a Dynamic step (time integration) starts from an initial velocity; \
                 static, frequency and thermal steps ignore it without a message."
            }
            Problem::NoRotation => {
                "The Coriolis forces of a Complex Frequency step come from the centrifugal \
                 load of a static step before the Frequency step. Without a rotation the \
                 complex eigenfrequencies are those of the Frequency step."
            }
            Problem::ContactIgnored => {
                "CalculiX takes contact into account in a Frequency step only as a \
                 perturbation of the static step before it. Without Perturbation the contact \
                 pairs, tied ones included, are left out without a message: parts held only \
                 by them float and give eigenfrequencies near zero or on weak springs."
            }
            Problem::ContactFormulationsDiffer => {
                "CalculiX takes one formulation (node to surface, surface to surface, mortar \
                 or massless) for all contact pairs of a model. A contact pair defined by hand \
                 has another one than the contact regions, which use the formulation of the \
                 Connections."
            }
            Problem::MortarNotSupported => {
                "CalculiX takes mortar contact in static steps only. In frequency, \
                 buckling, dynamic and thermal steps the run aborts or the contact is left \
                 out."
            }
            Problem::ContactRegionOpen => {
                "No node of the slave side lies within the tolerance of the master side in \
                 the mesh. A bonded region then ties nothing, and the parts are not \
                 connected. A contact that starts open holds nothing until the parts meet, \
                 so a part held only by it can move freely and the solution fails."
            }
            Problem::NoSeparationLargeDeformation => {
                "No separation is written as linear equations between each slave node and \
                 the master side, along the normal of the undeformed surfaces. With large \
                 deformations the sides slide and turn, and the equations no longer keep \
                 them together: gaps open or the sides press into each other."
            }
            Problem::ContactOutputIgnored => {
                "CalculiX prints contact values (*CONTACT PRINT) of contact pairs only: a \
                 bonded region written as a tie and a region of no separation have no \
                 contact elements, and a deactivated region is not written. The history \
                 output is left out."
            }
            Problem::ContactChangeIgnored => {
                "CalculiX switches contact pairs off and on within the steps, but not ties \
                 and constraint equations: a bonded region written as a tie and a region of \
                 no separation hold in every step. A region deactivated as a whole is not \
                 written at all, so no step can switch it on."
            }
            Problem::InvalidTiedContact => {
                "CalculiX offers the Tied surface behavior only for Surface to surface \
                 contact pairs, and it needs a Friction model in the same surface \
                 interaction: the stick slope of the friction is the stiffness of the tie \
                 along the surfaces (the friction coefficient is irrelevant). Without \
                 friction the run aborts with \"a stick slope using the *FRICTION card is \
                 mandatory\"; with Node to surface it aborts too, and Mortar fails."
            }
            Problem::ContactWithCoriolis => {
                "CalculiX computes the Coriolis coupling of a Complex Frequency step without \
                 the contact springs of the Frequency step. With parts joined by contact \
                 pairs whirl pairs do not split, and modes go missing from the results."
            }
            Problem::NoConvergence => {
                "The Newton iteration did not converge; CalculiX kept reducing the \
                 increment and gave up (\"too many cutbacks\" or \"increment size \
                 smaller than minimum\"). The cause is usually contacts, poorly supported \
                 parts or too large loads in a nonlinear analysis. An even smaller \
                 minimum increment rarely helps."
            }
            Problem::MpcAndSpc => {
                "A degree of freedom is both fixed by a boundary condition and bound as \
                 the dependent degree of freedom of an equation (MPC). CalculiX then aborts. \
                 Nodalis keeps the equations and rigid bodies it writes itself off the \
                 degrees of freedom that boundary conditions or other constraints hold (the \
                 solver output lists what it changed), so the overlap comes from a \
                 combination it does not resolve yet."
            }
            Problem::RotationIn2d => {
                "Rotations (degrees of freedom 4 to 6) were fixed in a 2D model. \
                 CalculiX expands 2D elements internally into solid elements and cannot \
                 prescribe rotations for them; the run aborts with \"mpc of type is \
                 unknown\" or a floating point error."
            }
            Problem::NoPhysicalConstants => {
                "Radiation (*RADIATE) depends on the fourth power of the absolute \
                 temperature, so the solver needs the absolute zero of the model's temperature \
                 scale and the Stefan-Boltzmann constant in its units (*PHYSICAL \
                 CONSTANTS). Without them it aborts."
            }
            Problem::SolverError => {
                "The solver aborted with an error message that Nodalis does not know. \
                 The exact text is shown in the Monitor."
            }
        }
    }

    /// How to fix it.
    pub fn fix(self) -> &'static str {
        match self {
            Problem::MissingReference => {
                "Edit the item and select an existing selection or an existing \
                 material, or delete it."
            }
            Problem::InvalidSection => {
                "Edit the section and choose the type that fits its elements, or change the \
                 element type of the part (double-click it under Mesh). For a membrane \
                 section mesh the part again with Second order off."
            }
            Problem::NoSection => {
                "Under Sections create a section for the part or extend an existing one \
                 to it."
            }
            Problem::NoElastic => {
                "Edit the material and enter Young's modulus and Poisson's ratio, or \
                 take a material from the material library."
            }
            Problem::NoDensity => "Edit the material and enter a density.",
            Problem::NoConductivity => "Edit the material and enter a thermal conductivity.",
            Problem::NoSpecificHeat => {
                "Edit the material and enter a specific heat, or compute the step as \
                 steady state."
            }
            Problem::NoInitialTemperature => {
                "Under Initial Conditions create an initial temperature for the model \
                 (for thermal expansion the temperature of the stress-free state), or \
                 compute the step as steady state."
            }
            Problem::InvalidElastic => {
                "Enter a Young's modulus greater than 0 and a Poisson's ratio less than \
                 0.5 (steel: 210000 MPa, 0.3)."
            }
            Problem::RigidBeamNodes => {
                "If the point turns by more than a few degrees, select solid faces or nodes \
                 for the rigid body, or join the beams with a Node Tie (rigid) at a shared \
                 node instead of a rigid body. For small rotations nothing needs to change. \
                 A run that stopped with this message used an input file from an older \
                 Nodalis: run the analysis again."
            }
            Problem::TieNodesNotTied => {
                "Set the Tolerance of the contact region (or tie) a little above the largest \
                 distance, or close the gap in the geometry. A gap that should stay open is no \
                 bonded contact: use another behavior or Interface treatment there."
            }
            Problem::HeldRigidBodyNodes => {
                "To hold the rigid body, put the boundary condition on its reference point, or \
                 leave the nodes of the rigid body out of the boundary condition's selection."
            }
            Problem::SharedRigidBody => {
                "Make the selections of the rigid bodies and displacements on a point separate, \
                 or use one displacement or rigid body for the joint selection."
            }
            Problem::NoRigidBody => {
                "Create a Rigid Body constraint on the surface or nodes the reference point \
                 shall drive, or put the boundary condition or load on nodes of the mesh."
            }
            Problem::InvalidCreep => {
                "Edit the material: enter the elasticity and positive values of A and n."
            }
            Problem::InvalidPlastic => {
                "Edit the material: enter the elastic constants and a hardening curve that \
                 starts at plastic strain 0 with the yield stress, e.g. 235 MPa at 0 and \
                 400 MPa at 0.2."
            }
            Problem::DistortedElements => {
                "Remesh the part: a smaller element size at tight radii or thin walls, \
                 or linear instead of quadratic elements. For imported meshes check the \
                 node order."
            }
            Problem::SpinAboutPins => {
                "Nothing to do for a beam between two ball joints. To hold the rotation, \
                 give all three rotations at one of the pins; only one of them is not \
                 enough, see \"Beam rotations partly held\"."
            }
            Problem::RigidBodyMotion => {
                "Add a boundary condition that blocks the listed motions, tie the parts \
                 to supported parts with a Tie, or add weak springs (Constraints \
                 > Spring) to ground."
            }
            Problem::Mechanism => {
                "Mesh every truss with exactly one element (Mesh Setup > Meshing \
                 Parameters: max element size larger than the longest truss). Also fix \
                 a plane truss at all nodes perpendicular to its plane. If the trusses \
                 should carry bending, use a Beam Section instead of the Truss \
                 Section."
            }
            Problem::HeldOnlyByContact => {
                "Turn Weak springs on again (Connections > Connection Settings): Nodalis \
                 then holds these parts with very soft springs. Otherwise support them \
                 weakly yourself (a Spring with a small stiffness), let the contact \
                 surfaces touch at the start (Adjust in the contact pair) or bring the \
                 parts into contact with a small displacement in a first step."
            }
            Problem::ConflictingBoundaries => {
                "Change the selection of the boundary conditions so that no nodes \
                 overlap, or give both boundary conditions the same value."
            }
            Problem::LoadOnFixedNodes => {
                "Apply the load to a surface or nodes that can move, or remove the \
                 boundary condition there."
            }
            Problem::RotationsIgnored => {
                "Leave UR1 to UR3 free in the boundary condition. Prescribe rotations of \
                 solids by displacements of several nodes."
            }
            Problem::PartialBeamRotations => {
                "Give all three rotations UR1 to UR3 of the node (a clamp) or none of them \
                 (a pin). A pin at each end of a beam may leave it free to spin about the \
                 line through both; CalculiX copes with that."
            }
            Problem::NoSymmetryPlane => {
                "Select the planar faces of the cut (or the edge of shells), or enter the \
                 normal of the plane in the boundary condition."
            }
            Problem::SymmetryNotPlanar => {
                "Select only the flat faces that lie in the cut plane. Use a \
                 Displacement/Rotation boundary condition for other supports."
            }
            Problem::SymmetryOffAxis => {
                "Select edges of constant y, or use a Displacement/Rotation boundary \
                 condition for other supports."
            }
            Problem::SkewedSymmetryShared => {
                "Keep other boundary conditions and point loads off the nodes of the \
                 symmetry plane, or align the model so that the plane is a coordinate \
                 plane."
            }
            Problem::NoGlobalResults => {
                "Open Model > Model Properties, set the model type to Submodel and pick the results \
                 file (.frd) of the global model."
            }
            Problem::IncrementExceedsStep => {
                "In the step choose an initial increment no larger than the time period."
            }
            Problem::NoLoad => "Under Loads create a load or activate a deactivated one.",
            Problem::UnusedDistribution => {
                "Choose the distribution in the Distribution field of the pressure or \
                 temperature it belongs to, or delete it."
            }
            Problem::UnusedAmplitude => {
                "Choose the amplitude in the Amplitude field of the boundary condition, load or \
                 defined field it belongs to, or delete it."
            }
            Problem::WheelOffSurface => {
                "If the wheels should stay on the surface, check the start and end of the \
                 path, the wheel offsets and the coordinate system, or select all faces the \
                 wheels run on."
            }
            Problem::NoExpansion => {
                "Edit the material and enter a thermal expansion coefficient \
                 (steel: 1.2e-5 1/K)."
            }
            Problem::NoStoredModes => {
                "Add a Frequency step with \"Store eigenmodes (Storage)\" before it, with \
                 the same supports (for Complex Frequency as a perturbation step after a \
                 static step with the centrifugal load)."
            }
            Problem::NoSensitivityBase => {
                "Put a Static step (strain energy, mass) or a Frequency step with \"Store \
                 eigenmodes (Storage)\" (eigenfrequencies, mass) directly before the \
                 Sensitivity step, or change its design responses."
            }
            Problem::DesignRegionIgnored => {
                "Give all Sensitivity steps the same design region, or analyse the other \
                 region in a copy of the model."
            }
            Problem::NoCreep => {
                "Edit a material of the model and tick Creep (Norton), or use a Static step."
            }
            Problem::GeneralBeamStep => {
                "Untick Nlgeom, use a Static or Frequency step, or give the beams a \
                 rectangular, circular, pipe or box cross-section."
            }
            Problem::InvalidCyclicSymmetry => {
                "Edit the tie: pick the two cut faces of the sector as master and slave and \
                 check the axis, or untick Cyclic symmetry in 2D models. In the Frequency step \
                 keep the nodal diameter range within 0 to half the number of sectors."
            }
            Problem::VelocityIgnored => "Add a Dynamic step or deactivate the initial condition.",
            Problem::InvalidBolt => {
                "Pick the two faces of the cut on the geometry, or element faces of solid \
                 elements."
            }
            Problem::BoltWithoutPreload => {
                "Enter the preload of the bolt, and add a static step if the analysis has none."
            }
            Problem::BoltFacesUsedTwice => {
                "Deactivate or delete the tie or contact pair on the faces of the cut; the bolt \
                 replaces it."
            }
            Problem::BoltHeldAtCut => {
                "Make the region frictionless or frictional, so that the shank can slide in \
                 its hole."
            }
            Problem::LockWithoutPreload => {
                "Preload the bolt by force or displacement in this step and lock it in the \
                 following steps."
            }
            Problem::NoRotation => {
                "Create a Centrifugal load in the static step before the Frequency step."
            }
            Problem::ContactIgnored => {
                "Tick Perturbation in the Frequency step (after a static step), or join \
                 bonded parts with a Tie constraint instead of a tied contact pair (Interaction > Create Tie)."
            }
            Problem::ContactFormulationsDiffer => {
                "Set the contact pair to the formulation of the contact regions, or choose \
                 the pair's formulation for all contacts at the Connections."
            }
            Problem::MortarNotSupported => {
                "Set the formulation at the Connections to Auto or Surface to surface, or \
                 deactivate the step."
            }
            Problem::ContactRegionOpen => {
                "Check the sides of the region in the 3D view and the gap in the Contact \
                 Worksheet. Pick faces that touch, raise the Tolerance in the region's \
                 advanced settings, or close the gap in the geometry."
            }
            Problem::NoSeparationLargeDeformation => {
                "Switch off large deformations in the step if the displacements are small, \
                 or make the region Frictionless with Adjust to touch: a contact that may \
                 open, but follows the sides."
            }
            Problem::ContactOutputIgnored => {
                "Set Bonded as to Auto or Contact (tied contact) with the Surface to surface \
                 formulation, so that the bonded region becomes a contact pair, or activate \
                 the region."
            }
            Problem::ContactChangeIgnored => {
                "Set Bonded as to Auto or Contact (tied contact) with the Surface to surface \
                 formulation, so that the region becomes a contact pair, or activate the \
                 region and switch it off in the steps where it should not hold."
            }
            Problem::InvalidTiedContact => {
                "Edit the surface interaction and add Friction, and set the method of the \
                 contact pairs to Surface to surface. To bond parts, a Tie (Interaction > \
                 Create Tie) works with every method."
            }
            Problem::ContactWithCoriolis => {
                "Join bonded parts with a Tie constraint (Interaction > Create Tie) instead \
                 of a contact pair with tied surface behavior."
            }
            Problem::NoConvergence => {
                "Check the contacts (stiffness of the Surface Interaction, Adjust, \
                 position of the surfaces), support the parts sufficiently, spread the \
                 load over several increments (incrementation Automatic with a smaller \
                 initial increment) and refine the mesh at the contact surfaces."
            }
            Problem::MpcAndSpc => {
                "Look up the node of the message in the input file and leave it out of \
                 the boundary condition or of the constraint, contact region or remote \
                 displacement that binds it."
            }
            Problem::RotationIn2d => "In 2D models prescribe only U1 and U2.",
            Problem::NoPhysicalConstants => {
                "Open Model > Model Properties and enter Absolute zero and Stefan-Boltzmann \
                 under Physical Constants; Defaults fills them in for the unit system."
            }
            Problem::SolverError => {
                "Read the message in the Monitor; \"Check Model\" in the context menu of \
                 the analysis often helps."
            }
        }
    }
}

/// A problem found at an item of the model.
#[derive(Clone, Debug, PartialEq)]
pub struct Finding {
    pub item: ModelItem,
    pub problem: Problem,
    /// What exactly is affected, e.g. the parts or the number of nodes.
    pub detail: String,
}

impl Finding {
    pub(crate) fn new(item: ModelItem, problem: Problem, detail: impl Into<String>) -> Self {
        Self {
            item,
            problem,
            detail: detail.into(),
        }
    }

    pub fn severity(&self) -> Severity {
        self.problem.severity()
    }
}

/// What the checks need of the mesh, computed once per mesh: which nodes are connected and
/// which elements are distorted.
#[derive(Clone, Debug)]
pub struct MeshCheck {
    /// Node count, element count, part count and model space it was made for.
    stamp: (usize, usize, usize, ModelSpace),
    /// Connected piece of each node by node index; `u32::MAX` for nodes of no element.
    piece: Vec<u32>,
    pieces: usize,
    /// Parts by index that each piece contains.
    piece_parts: Vec<Vec<usize>>,
    /// Elements with a nonpositive Jacobian determinant of each part, by part index.
    distorted: Vec<Vec<ElementId>>,
}

impl MeshCheck {
    pub fn new(mesh: &FeMesh, space: ModelSpace) -> Self {
        let mut union = UnionFind::new(mesh.node_count());
        let mut used = vec![false; mesh.node_count()];
        for element in mesh.elements() {
            let mut first = None;
            for index in element.nodes.iter().filter_map(|&n| mesh.node_index(n)) {
                used[index] = true;
                match first {
                    None => first = Some(index),
                    Some(first) => union.join(first, index),
                }
            }
        }
        let mut label = vec![u32::MAX; mesh.node_count()];
        let mut pieces = 0;
        let piece: Vec<u32> = (0..mesh.node_count())
            .map(|i| {
                if !used[i] {
                    return u32::MAX;
                }
                let root = union.root(i);
                if label[root] == u32::MAX {
                    label[root] = pieces as u32;
                    pieces += 1;
                }
                label[root]
            })
            .collect();
        let mut piece_parts = vec![Vec::new(); pieces];
        let mut distorted = Vec::with_capacity(mesh.parts.len());
        for (index, part) in mesh.parts.iter().enumerate() {
            let mut bad = Vec::new();
            let mut seen = HashSet::new();
            for element in part.elements.iter().filter_map(|&e| mesh.element(e)) {
                if let Some(p) = (element.nodes.first())
                    .and_then(|&n| mesh.node_index(n))
                    .map(|i| piece[i])
                    && seen.insert(p)
                {
                    piece_parts[p as usize].push(index);
                }
                if element.min_jacobian(mesh).is_some_and(|j| j <= 0.0) {
                    bad.push(element.id);
                }
            }
            distorted.push(bad);
        }
        Self {
            stamp: Self::stamp(mesh, space),
            piece,
            pieces,
            piece_parts,
            distorted,
        }
    }

    fn stamp(mesh: &FeMesh, space: ModelSpace) -> (usize, usize, usize, ModelSpace) {
        (
            mesh.node_count(),
            mesh.element_count(),
            mesh.parts.len(),
            space,
        )
    }

    /// Whether it was made for this mesh; a mesh with other counts needs a new check.
    pub fn is_current(&self, mesh: &FeMesh, space: ModelSpace) -> bool {
        self.stamp == Self::stamp(mesh, space)
    }

    fn piece_of(&self, mesh: &FeMesh, node: NodeId) -> Option<u32> {
        let piece = *self.piece.get(mesh.node_index(node)?)?;
        (piece != u32::MAX).then_some(piece)
    }
}

impl FeModel {
    /// Cyclic symmetry ties CalculiX rejects, such as cut faces that do not divide the
    /// full circle, and nodal diameters the sectors cannot carry.
    fn check_cyclic_symmetry(&self, mesh: &FeMesh, findings: &mut Vec<Finding>) {
        for (i, tie) in self.ties.iter().enumerate().filter(|(_, t)| t.active) {
            let Some(sectors) = tie.cyclic_sectors(mesh) else {
                continue;
            };
            let problem = if self.properties.space.is_2d() {
                Some("Cyclic symmetry needs a 3D model.".to_string())
            } else {
                sectors.err()
            };
            if let Some(problem) = problem {
                let detail = format!("{}: {problem}", tie.name);
                findings.push(Finding::new(
                    ModelItem::Tie(i),
                    Problem::InvalidCyclicSymmetry,
                    detail,
                ));
            }
        }
        let Some(Some(Ok(sectors))) = self.cyclic_tie().map(|t| t.cyclic_sectors(mesh)) else {
            return;
        };
        let highest = crate::CyclicSymmetry::highest_nodal_diameter(sectors);
        for (s, step) in self.steps.iter().enumerate().filter(|(_, s)| s.active) {
            if let StepKind::Frequency(f) = &step.kind
                && let Some([min, max]) = f.nodal_diameters
                && (min > max || max > highest)
            {
                findings.push(Finding::new(
                    ModelItem::Step(s),
                    Problem::InvalidCyclicSymmetry,
                    format!(
                        "{} asks for nodal diameters {min} to {max}, the sectors carry 0 to \
                         {highest}",
                        step.name
                    ),
                ));
            }
        }
    }

    /// All problems of the model, invalid references included, errors first.
    pub fn check(&self, mesh: &FeMesh, mesh_check: &MeshCheck) -> Vec<Finding> {
        if self.bolts.is_empty() {
            return self.check_connections(mesh, mesh_check);
        }
        // Bolts are checked as the pre-tension loads and ties they become, with the contact
        // regions they replace switched off; what is found on those belongs to the bolt.
        let (lowered, origin) = self.lower_bolts(mesh);
        let mut findings = lowered.check_connections(mesh, mesh_check);
        for finding in &mut findings {
            if let Some(bolt) = origin.bolt_of(finding.item) {
                finding.item = ModelItem::Bolt(bolt);
            }
        }
        self.check_bolts(mesh, &origin, &mut findings);
        findings.sort_by_key(|f| f.severity());
        findings
    }

    /// [`FeModel::check`] of a model without bolts.
    fn check_connections(&self, mesh: &FeMesh, mesh_check: &MeshCheck) -> Vec<Finding> {
        let mut findings = if self.contact_regions.is_empty() {
            self.check_explicit(mesh, mesh_check)
        } else {
            // Contact regions are checked as the ties and contact pairs they become; what is
            // found on those belongs to the region.
            let (lowered, origin) = self.lower_contacts(mesh);
            let mut findings = lowered.check_explicit(mesh, mesh_check);
            for finding in &mut findings {
                if let Some(region) = origin.region_of(finding.item) {
                    finding.item = ModelItem::ContactRegion(region);
                }
            }
            self.check_contact_regions(mesh, &lowered, &mut findings);
            findings
        };
        // The input file holds these parts with weak springs.
        if self.contact_defaults.weak_springs {
            findings.retain(|f| f.problem != Problem::HeldOnlyByContact);
        }
        findings
    }

    /// Weak springs to ground for the parts that only contacts hold in a static step, so
    /// that CalculiX does not stall on the free motions before the contacts close: with
    /// frictionless contact, an initial gap or Nlgeom it otherwise gives up with "too many
    /// cutbacks", and node to surface contact drifts sideways by arbitrary amounts. Each
    /// part gets a stiffness of [`WEAK_SPRING_FACTOR`](crate::WEAK_SPRING_FACTOR) E L per
    /// direction, spread over all its nodes: E its Young's modulus, L the geometric mean of
    /// its extents. None if the connection settings turn them off. For the model the input
    /// file is written from, with the contact regions lowered.
    pub fn weak_springs(&self, mesh: &FeMesh) -> Vec<crate::Spring> {
        if !self.contact_defaults.weak_springs
            || (self.contact_pairs.iter()).all(|p| !p.active)
            || !self.steps.iter().any(|s| s.active)
        {
            return Vec::new();
        }
        let space = self.properties.space;
        let findings = self.check_explicit(mesh, &MeshCheck::new(mesh, space));
        let mut parts: Vec<usize> = (findings.iter())
            .filter(|f| f.problem == Problem::HeldOnlyByContact)
            .filter_map(|f| match f.item {
                ModelItem::Part(p) => Some(p),
                _ => None,
            })
            .collect();
        parts.sort_unstable();
        parts.dedup();
        if parts.is_empty() {
            return Vec::new();
        }
        let young = self.young_of_elements(mesh);
        let mut springs = Vec::new();
        for p in parts {
            let Some(part) = mesh.parts.get(p) else {
                continue;
            };
            let mut nodes: Vec<NodeId> = (part.elements.iter())
                .filter_map(|&e| mesh.element(e))
                .flat_map(|e| e.nodes.iter().copied())
                .collect();
            nodes.sort_unstable();
            nodes.dedup();
            let e = (part.elements.iter())
                .filter_map(|e| young.get(e).copied())
                .fold(0.0, f64::max);
            let (mut min, mut max) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
            for x in nodes.iter().filter_map(|&n| mesh.node(n)) {
                for i in 0..3 {
                    min[i] = min[i].min(x[i]);
                    max[i] = max[i].max(x[i]);
                }
            }
            let extents: Vec<f64> = (0..3).map(|i| (max[i] - min[i]).max(0.0)).collect();
            let largest = extents.iter().copied().fold(0.0, f64::max);
            // A flat part counts with a hundredth of its size across.
            let size = (extents.iter())
                .map(|&x| x.max(0.01 * largest))
                .product::<f64>()
                .cbrt();
            let k = crate::WEAK_SPRING_FACTOR * e * size;
            if !(k.is_finite() && k > 0.0) || nodes.is_empty() {
                continue;
            }
            let mut spring = crate::Spring::new(format!("PLX_Weak_Springs_{}", part.name));
            spring.visible = false;
            spring.region = Region::Nodes(nodes);
            spring.stiffness = [k; 3];
            springs.push(spring);
        }
        springs
    }

    /// Contact regions whose sides do not touch, contact pairs whose formulation differs
    /// from that of the others, and the mortar formulation where CalculiX does not take it.
    fn check_contact_regions(&self, mesh: &FeMesh, lowered: &FeModel, findings: &mut Vec<Finding>) {
        for (i, check) in self.check_contacts(mesh).into_iter().enumerate() {
            let region = &self.contact_regions[i];
            let Some(check) = check.filter(|_| region.active) else {
                continue;
            };
            if matches!(
                check.status,
                crate::ContactStatus::NearOpen | crate::ContactStatus::FarOpen
            ) {
                findings.push(Finding::new(
                    ModelItem::ContactRegion(i),
                    Problem::ContactRegionOpen,
                    format!(
                        "{}: no slave node within the tolerance of the master side ({})",
                        region.name,
                        check.status.name().to_lowercase()
                    ),
                ));
            }
        }
        let resolved = self.resolve_contacts(mesh);
        for (i, (region, item)) in self.contact_regions.iter().zip(&resolved.items).enumerate() {
            if !region.switched(&self.steps) {
                continue;
            }
            let reason = match item.kind {
                _ if !region.active => "it is deactivated as a whole",
                crate::ResolvedKind::Tie { .. } => "it is written as a tie",
                crate::ResolvedKind::NoSeparation { .. } => "no separation is written as equations",
                crate::ResolvedKind::Pair { .. } => continue,
            };
            findings.push(Finding::new(
                ModelItem::ContactRegion(i),
                Problem::ContactChangeIgnored,
                format!("{}: a step switches it, but {reason}", region.name),
            ));
        }
        for (i, (region, item)) in self.contact_regions.iter().zip(&resolved.items).enumerate() {
            if !region.prints_contact(&self.steps) {
                continue;
            }
            let reason = match item.kind {
                _ if !region.active => "it is deactivated as a whole",
                crate::ResolvedKind::Tie { .. } => "it is written as a tie",
                crate::ResolvedKind::NoSeparation { .. } => "no separation is written as equations",
                crate::ResolvedKind::Pair { .. } => continue,
            };
            findings.push(Finding::new(
                ModelItem::ContactRegion(i),
                Problem::ContactOutputIgnored,
                format!(
                    "{}: a history output prints its contact values, but {reason}",
                    region.name
                ),
            ));
        }
        let nlgeom = (self.steps.iter()).find(|s| s.active && s.kind.nlgeom());
        for (i, region) in self.contact_regions.iter().enumerate() {
            if let Some(step) = nlgeom.filter(|_| {
                region.active && region.behavior == crate::ContactBehavior::NoSeparation
            }) {
                findings.push(Finding::new(
                    ModelItem::ContactRegion(i),
                    Problem::NoSeparationLargeDeformation,
                    format!("{}: {} has large deformations", region.name, step.name),
                ));
            }
        }
        let (formulation, _) = self.contact_formulation();
        for (i, pair) in self.contact_pairs.iter().enumerate() {
            if pair.active && pair.method != formulation {
                findings.push(Finding::new(
                    ModelItem::ContactPair(i),
                    Problem::ContactFormulationsDiffer,
                    format!(
                        "{}: {}, the contact regions use {}",
                        pair.name,
                        pair.method.name(),
                        formulation.name()
                    ),
                ));
            }
        }
        if formulation != ContactMethod::Mortar || !lowered.contact_pairs.iter().any(|p| p.active) {
            return;
        }
        for (s, step) in self.steps.iter().enumerate().filter(|(_, s)| s.active) {
            if !crate::mortar_supports(&step.kind) {
                findings.push(Finding::new(
                    ModelItem::Step(s),
                    Problem::MortarNotSupported,
                    format!("{}: mortar contact in a step that is not static", step.name),
                ));
            }
        }
    }

    fn check_explicit(&self, mesh: &FeMesh, mesh_check: &MeshCheck) -> Vec<Finding> {
        let constants =
            self.properties.absolute_zero.is_some() && self.properties.stefan_boltzmann.is_some();
        let mut findings: Vec<Finding> = (self.invalid_items(mesh).into_iter())
            .map(|i| {
                let radiation = match i.item {
                    ModelItem::Load(s, l) => (self.steps.get(s)).and_then(|s| s.loads.get(l)),
                    _ => None,
                }
                .is_some_and(|load| matches!(load.kind, LoadKind::Radiation { .. }));
                let problem = if radiation && !constants {
                    Problem::NoPhysicalConstants
                } else {
                    Problem::MissingReference
                };
                Finding::new(i.item, problem, i.reason)
            })
            .collect();
        for (i, section) in self.sections.iter().enumerate() {
            let item = ModelItem::Section(i);
            if !findings.iter().any(|f| f.item == item)
                && let Some(reason) = section.kind_problem(self, mesh)
            {
                findings.push(Finding::new(item, Problem::InvalidSection, reason));
            }
        }
        self.check_materials(&mut findings);
        self.check_sections(mesh, &mut findings);
        for (part, bad) in mesh_check.distorted.iter().enumerate() {
            if !bad.is_empty() {
                let detail = format!(
                    "{} element(s) with nonpositive Jacobian determinant, e.g. {}",
                    bad.len(),
                    list(bad.iter().take(5))
                );
                findings.push(Finding::new(
                    ModelItem::Part(part),
                    Problem::DistortedElements,
                    detail,
                ));
            }
        }
        let initial_temperature = self.properties.initial_temperatures.is_some()
            || (self.initial_conditions.iter()).any(|c| c.active && !c.kind.is_velocity());
        let dynamic =
            (self.steps.iter()).any(|s| s.active && matches!(s.kind, StepKind::Dynamic(_)));
        for (i, condition) in self.initial_conditions.iter().enumerate() {
            if condition.active && condition.kind.is_velocity() && !dynamic {
                findings.push(Finding::new(
                    ModelItem::InitialCondition(i),
                    Problem::VelocityIgnored,
                    format!("{} needs a Dynamic step", condition.name),
                ));
            }
        }
        self.check_pre_tension_locks(&mut findings);
        self.check_cyclic_symmetry(mesh, &mut findings);
        self.check_tied_contacts(&mut findings);
        self.check_amplitudes(&mut findings);
        let used = self.used_distributions();
        for (i, distribution) in self.distributions.iter().enumerate() {
            if !used.contains(distribution.name.as_str()) {
                findings.push(Finding::new(
                    ModelItem::Distribution(i),
                    Problem::UnusedDistribution,
                    format!(
                        "{} is not used by any load or temperature",
                        distribution.name
                    ),
                ));
            }
        }
        self.check_shared_rigid_bodies(mesh, &mut findings);
        let trusses = Trusses::new(self, mesh);
        let general_beams = self.general_beam_sections(mesh);
        for (s, step) in self.steps.iter().enumerate().filter(|(_, s)| s.active) {
            self.check_step(s, mesh, mesh_check, &trusses, &mut findings);
            let general_fits = !step.kind.nlgeom()
                && matches!(
                    step.kind,
                    StepKind::Static(_)
                        | StepKind::Frequency(_)
                        | StepKind::ModalDynamics(_)
                        | StepKind::SteadyStateDynamics(_)
                );
            if !general_beams.is_empty() && !general_fits {
                findings.push(Finding::new(
                    ModelItem::Step(s),
                    Problem::GeneralBeamStep,
                    format!(
                        "{}{}: {} use general beam profiles",
                        step.name,
                        if step.kind.nlgeom() { " (Nlgeom)" } else { "" },
                        general_beams.join(", ")
                    ),
                ));
            }
            let stored = self.properties.stored_modes.is_some()
                || (self.steps[..s].iter().filter(|p| p.active))
                    .any(|p| matches!(&p.kind, StepKind::Frequency(f) if f.storage));
            if step.kind.uses_stored_modes() && !stored {
                findings.push(Finding::new(
                    ModelItem::Step(s),
                    Problem::NoStoredModes,
                    format!("{} needs the eigenmodes of a Frequency step", step.name),
                ));
            }
            let used = |m: &&crate::Material| self.sections.iter().any(|s| s.material == m.name);
            if matches!(step.kind, StepKind::Visco(_))
                && !(self.materials.iter().filter(used)).any(|m| m.creep.is_some())
            {
                findings.push(Finding::new(
                    ModelItem::Step(s),
                    Problem::NoCreep,
                    format!("{}: no material of a section has a creep law", step.name),
                ));
            }
            if let StepKind::Sensitivity(settings) = &step.kind {
                let previous = (self.steps[..s].iter()).rev().find(|p| p.active);
                let fits = match previous.map(|p| &p.kind) {
                    Some(StepKind::Static(_)) => !settings.eigenfrequency,
                    // CalculiX reads the eigenmodes from the .eig file.
                    Some(StepKind::Frequency(f)) => {
                        !settings.strain_energy && (f.storage || !settings.eigenfrequency)
                    }
                    _ => false,
                };
                if !fits {
                    findings.push(Finding::new(
                        ModelItem::Step(s),
                        Problem::NoSensitivityBase,
                        format!("{} has no fitting step before it", step.name),
                    ));
                }
                let first = (self.steps[..s].iter().filter(|p| p.active))
                    .find_map(|p| p.kind.design_region());
                if first.is_some_and(|r| *r != settings.design_region) {
                    findings.push(Finding::new(
                        ModelItem::Step(s),
                        Problem::DesignRegionIgnored,
                        format!(
                            "{} uses the design region of the first sensitivity step",
                            step.name
                        ),
                    ));
                }
            }
            if let StepKind::HeatTransfer(h)
            | StepKind::CoupledTempDisp(h)
            | StepKind::UncoupledTempDisp(h) = &step.kind
                && !h.steady_state
                && !initial_temperature
            {
                findings.push(Finding::new(
                    ModelItem::Step(s),
                    Problem::NoInitialTemperature,
                    format!("{} is transient", step.name),
                ));
            }
            // CalculiX refuses a *Temperature without an initial temperature.
            let defined =
                step.kind.supports_defined_fields() && step.defined_fields.iter().any(|f| f.active);
            if defined && !initial_temperature {
                findings.push(Finding::new(
                    ModelItem::Step(s),
                    Problem::NoInitialTemperature,
                    format!("{} prescribes temperatures (Defined Field)", step.name),
                ));
            }
            if let StepKind::Static(settings)
            | StepKind::Visco(crate::ViscoStep {
                increments: settings,
                ..
            }) = &step.kind
                && settings.incrementation != crate::Incrementation::Default
                && settings.initial_increment > settings.time_period
            {
                findings.push(Finding::new(
                    ModelItem::Step(s),
                    Problem::IncrementExceedsStep,
                    format!(
                        "Initial increment {} > time period {}",
                        settings.initial_increment, settings.time_period
                    ),
                ));
            }
        }
        findings.sort_by_key(|f| f.severity());
        findings
    }

    /// Contact pairs with tied surface behavior CalculiX rejects.
    /// Nodes in more than one rigid body: of rigid body constraints, and of remote
    /// displacements, one per selection as in the input file.
    fn check_shared_rigid_bodies(&self, mesh: &FeMesh, findings: &mut Vec<Finding>) {
        let mut bodies: Vec<(ModelItem, &str, HashSet<NodeId>)> = Vec::new();
        let mut held = None;
        for (i, constraint) in self.constraints.iter().enumerate() {
            if let Constraint::RigidBody(body) = constraint
                && body.active
            {
                let nodes: HashSet<NodeId> = body.region.nodes(mesh).into_iter().collect();
                let held = held.get_or_insert_with(|| self.held_translations(mesh));
                let count = nodes.iter().filter(|n| held.contains_key(n)).count();
                if count > 0 {
                    findings.push(Finding::new(
                        ModelItem::Constraint(i),
                        Problem::HeldRigidBodyNodes,
                        format!(
                            "{}: {count} nodes held by boundary conditions, left out of the \
                             rigid body",
                            body.name
                        ),
                    ));
                }
                bodies.push((ModelItem::Constraint(i), &body.name, nodes));
            }
        }
        let mut points: Vec<(&Region, &crate::Displacement)> = Vec::new();
        let mut excluded = None;
        for (s, step) in self.steps.iter().enumerate().filter(|(_, s)| s.active) {
            for (i, bc) in step.boundary_conditions.iter().enumerate() {
                let BoundaryKind::Displacement(remote) = &bc.kind else {
                    continue;
                };
                if !bc.active
                    || !step.kind.supports_boundary(&bc.kind)
                    || !matches!(self.formulation(mesh, s, i), Some(Formulation::Point(_)))
                    || (points.iter()).any(|(r, p)| **r == bc.region && p.same_point(remote))
                {
                    continue;
                }
                points.push((&bc.region, remote));
                let excluded = excluded.get_or_insert_with(|| self.directly_held_nodes(mesh));
                if let Ok(point) = remote.resolve(self, mesh, &bc.region, excluded) {
                    let nodes = point.nodes.into_iter().collect();
                    bodies.push((ModelItem::BoundaryCondition(s, i), &bc.name, nodes));
                }
            }
        }
        if let Some(step) = (self.steps.iter()).find(|s| s.active && s.kind.nlgeom()) {
            let expanded = crate::expanded_nodes(mesh);
            for (item, name, nodes) in &bodies {
                let count = nodes.iter().filter(|n| expanded.contains(n)).count();
                if count > 0 {
                    findings.push(Finding::new(
                        *item,
                        Problem::RigidBeamNodes,
                        format!(
                            "{name}: {count} beam or shell nodes coupled by equations, {} \
                             with large deformations",
                            step.name
                        ),
                    ));
                }
            }
        }
        for (b, (item, name, nodes)) in bodies.iter().enumerate() {
            for (_, other, others) in &bodies[..b] {
                let shared = nodes.intersection(others).count();
                if shared > 0 {
                    findings.push(Finding::new(
                        *item,
                        Problem::SharedRigidBody,
                        format!("{name}: {shared} nodes also in {other}"),
                    ));
                }
            }
        }
    }

    /// What only the bolts themselves tell: references, cuts that cannot be made, missing
    /// preloads, and other connections on their cut.
    fn check_bolts(
        &self,
        mesh: &FeMesh,
        lowered: &crate::LoweredBolts,
        findings: &mut Vec<Finding>,
    ) {
        let invalid = self.invalid_items(mesh);
        for item in invalid
            .iter()
            .filter(|i| matches!(i.item, ModelItem::Bolt(_)))
        {
            findings.push(Finding::new(
                item.item,
                Problem::MissingReference,
                item.reason.clone(),
            ));
        }
        for (b, bolt) in self.bolts.iter().enumerate().filter(|(_, b)| b.active) {
            if invalid.iter().any(|i| i.item == ModelItem::Bolt(b)) {
                continue;
            }
            if let Err(reason) = self.resolve_bolt(bolt, mesh) {
                findings.push(Finding::new(
                    ModelItem::Bolt(b),
                    Problem::InvalidBolt,
                    format!("{}: {reason}", bolt.name),
                ));
                continue;
            }
            let preloaded = (0..self.steps.len()).any(|s| match self.bolt_action(bolt, s) {
                Some(crate::BoltAction::Load(v) | crate::BoltAction::Adjust(v)) => v != 0.0,
                _ => false,
            });
            if !preloaded {
                findings.push(Finding::new(
                    ModelItem::Bolt(b),
                    Problem::BoltWithoutPreload,
                    format!("{} is not preloaded in any step", bolt.name),
                ));
            }
            let cut = self.bolt_cut_nodes(bolt, mesh);
            let on_cut = |region: &Region| -> bool {
                let nodes = crate::bolt::side_nodes(region, mesh);
                let on = nodes.iter().filter(|n| cut.contains(n)).count();
                !nodes.is_empty() && 2 * on >= nodes.len()
            };
            let twice = (self
                .ties
                .iter()
                .filter(|t| t.active)
                .map(|t| (&t.name, [&t.master, &t.slave])))
            .chain(
                (self.contact_pairs.iter().filter(|p| p.active))
                    .map(|p| (&p.name, [&p.master, &p.slave])),
            );
            for (name, sides) in twice {
                if sides.iter().all(|side| on_cut(side)) {
                    findings.push(Finding::new(
                        ModelItem::Bolt(b),
                        Problem::BoltFacesUsedTwice,
                        format!("{name} lies on the cut of {}", bolt.name),
                    ));
                }
            }
            for (r, region) in self.contact_regions.iter().enumerate() {
                let holds = matches!(
                    region.behavior,
                    crate::ContactBehavior::Bonded | crate::ContactBehavior::Rough
                );
                if !region.active || !holds || lowered.replaced.contains_key(&r) {
                    continue;
                }
                let touches = region.regions().into_iter().any(|side| {
                    crate::bolt::side_nodes(side, mesh)
                        .iter()
                        .any(|n| cut.contains(n))
                });
                if touches {
                    // Shown at the bolt: detected regions have long names, the warning
                    // sign after them is often out of sight.
                    findings.push(Finding::new(
                        ModelItem::Bolt(b),
                        Problem::BoltHeldAtCut,
                        format!("{} holds {} at its cut", region.name, bolt.name),
                    ));
                }
            }
        }
    }

    /// Pre-tension loads locked in the first active step they act in.
    fn check_pre_tension_locks(&self, findings: &mut Vec<Finding>) {
        let mut preloaded = std::collections::BTreeSet::new();
        for (s, step) in self.steps.iter().enumerate().filter(|(_, s)| s.active) {
            for (i, load) in step.loads.iter().enumerate().filter(|(_, l)| l.active) {
                let LoadKind::PreTension { lock, .. } = load.kind else {
                    continue;
                };
                if lock && !preloaded.contains(&load.name) {
                    findings.push(Finding::new(
                        ModelItem::Load(s, i),
                        Problem::LockWithoutPreload,
                        format!(
                            "{} is locked in {} without a preload before",
                            load.name, step.name
                        ),
                    ));
                }
                preloaded.insert(load.name.clone());
            }
        }
    }

    /// Amplitudes that nothing refers to: a forgotten choice in a load or boundary condition
    /// leaves the model without the motion the amplitude was made for.
    fn check_amplitudes(&self, findings: &mut Vec<Finding>) {
        let used: HashSet<&str> = (self.steps.iter())
            .flat_map(|step| {
                let bcs = step.boundary_conditions.iter().map(|b| &b.amplitude);
                let loads = (step.loads.iter()).flat_map(|l| [&l.amplitude, &l.factor_amplitude]);
                let fields = step.defined_fields.iter().map(|f| &f.amplitude);
                bcs.chain(loads).chain(fields)
            })
            .filter_map(|name| name.as_deref())
            .collect();
        for (i, amplitude) in self.amplitudes.iter().enumerate() {
            if !used.contains(amplitude.name.as_str()) {
                findings.push(Finding::new(
                    ModelItem::Amplitude(i),
                    Problem::UnusedAmplitude,
                    format!(
                        "{} is not used by any boundary condition, load or defined field",
                        amplitude.name
                    ),
                ));
            }
        }
    }

    fn check_tied_contacts(&self, findings: &mut Vec<Finding>) {
        for (i, pair) in self.contact_pairs.iter().enumerate() {
            let Some(interaction) = (self.surface_interactions.iter())
                .find(|s| s.name == pair.interaction)
                .filter(|_| pair.active)
            else {
                continue;
            };
            if !interaction.is_tied() {
                continue;
            }
            let mut reasons = Vec::new();
            if !interaction.has_friction() {
                reasons.push(format!("{} has no Friction", interaction.name));
            }
            if pair.method != ContactMethod::SurfaceToSurface {
                reasons.push(format!("method {}", pair.method.name()));
            }
            if reasons.is_empty() {
                continue;
            }
            let detail = format!("{}: tied, but {}", pair.name, reasons.join(" and "));
            findings.push(Finding::new(
                ModelItem::ContactPair(i),
                Problem::InvalidTiedContact,
                detail,
            ));
        }
    }

    fn check_materials(&self, findings: &mut Vec<Finding>) {
        let active = || self.steps.iter().filter(|s| s.active);
        let mechanical = active().any(|s| s.kind.is_mechanical());
        let thermal = active().any(|s| s.kind.is_thermal());
        let transient = active().any(|s| match &s.kind {
            StepKind::HeatTransfer(h)
            | StepKind::CoupledTempDisp(h)
            | StepKind::UncoupledTempDisp(h) => !h.steady_state,
            _ => false,
        });
        // Eigenfrequencies and inertia need the mass.
        let frequency = active().any(|s| {
            matches!(s.kind, StepKind::Frequency(_) | StepKind::Dynamic(_))
                || s.kind.uses_stored_modes()
        });
        // Gravity and centrifugal loads act on the mass of the elements.
        let body_force = active().any(|s| {
            s.kind.supports_loads() && (s.loads.iter()).any(|l| l.active && l.kind.is_body_force())
        });
        for (i, material) in self.materials.iter().enumerate() {
            if !self.sections.iter().any(|s| s.material == material.name) {
                continue;
            }
            let item = ModelItem::Material(i);
            match material.elastic {
                None if mechanical => findings.push(Finding::new(
                    item,
                    Problem::NoElastic,
                    format!("{} has no elasticity", material.name),
                )),
                Some(e) if e.young <= 0.0 || e.poisson >= 0.5 || e.poisson <= -1.0 => {
                    findings.push(Finding::new(
                        item,
                        Problem::InvalidElastic,
                        format!("E = {}, nu = {}", e.young, e.poisson),
                    ));
                }
                _ => {}
            }
            if let Some(plastic) = &material.plastic
                && mechanical
            {
                let detail = match plastic.invalid_row() {
                    _ if material.elastic.is_none() => {
                        Some(format!("{} is plastic but not elastic", material.name))
                    }
                    Some(row) => Some(format!(
                        "{}: row {} of the hardening curve",
                        material.name,
                        row + 1
                    )),
                    None => None,
                };
                if let Some(detail) = detail {
                    findings.push(Finding::new(item, Problem::InvalidPlastic, detail));
                }
            }
            if let Some(creep) = &material.creep
                && mechanical
            {
                let detail = match creep.problem() {
                    _ if material.elastic.is_none() => {
                        Some(format!("{} creeps but is not elastic", material.name))
                    }
                    Some(problem) => Some(format!("{}: {problem}", material.name)),
                    None => None,
                };
                if let Some(detail) = detail {
                    findings.push(Finding::new(item, Problem::InvalidCreep, detail));
                }
            }
            if thermal && material.conductivity.is_none() {
                findings.push(Finding::new(
                    item,
                    Problem::NoConductivity,
                    format!("{} has no thermal conductivity", material.name),
                ));
            }
            if transient && material.specific_heat.is_none() {
                findings.push(Finding::new(
                    item,
                    Problem::NoSpecificHeat,
                    format!("{} has no specific heat", material.name),
                ));
            }
            if (frequency || transient || body_force) && material.density.is_none_or(|d| d <= 0.0) {
                findings.push(Finding::new(
                    item,
                    Problem::NoDensity,
                    format!("{} has no density", material.name),
                ));
            }
        }
    }

    /// Parts with elements no section covers.
    /// The beam sections on elements whose cross-section is a general profile, which
    /// CalculiX knows only as the user element U1.
    fn general_beam_sections(&self, mesh: &FeMesh) -> Vec<&str> {
        (self.sections.iter())
            .filter(|s| {
                self.section_profile(&s.kind)
                    .is_some_and(|p| !p.is_native())
            })
            .filter(|s| !s.region.elements(mesh).is_empty())
            .map(|s| s.name.as_str())
            .collect()
    }

    fn check_sections(&self, mesh: &FeMesh, findings: &mut Vec<Finding>) {
        let whole: HashSet<&str> = (self.sections.iter())
            .filter_map(|s| match &s.region {
                crate::Region::Parts(names) => Some(names.iter().map(String::as_str)),
                _ => None,
            })
            .flatten()
            .collect();
        let mut covered: Option<HashSet<ElementId>> = None;
        for (i, part) in mesh.parts.iter().enumerate() {
            if whole.contains(part.name.as_str()) || part.elements.is_empty() {
                continue;
            }
            // Element sets and faces are resolved only when a part is not covered as a whole.
            let covered = covered.get_or_insert_with(|| {
                (self.sections.iter())
                    .filter(|s| !matches!(s.region, crate::Region::Parts(_)))
                    .flat_map(|s| s.region.elements(mesh))
                    .collect()
            });
            let missing = (part.elements.iter())
                .filter(|e| !covered.contains(e))
                .count();
            if missing > 0 {
                findings.push(Finding::new(
                    ModelItem::Part(i),
                    Problem::NoSection,
                    format!(
                        "{missing} of {} elements of {} without section",
                        part.elements.len(),
                        part.name
                    ),
                ));
            }
        }
    }

    fn check_step(
        &self,
        s: usize,
        mesh: &FeMesh,
        check: &MeshCheck,
        trusses: &Trusses,
        findings: &mut Vec<Finding>,
    ) {
        let step = &self.steps[s];
        let space = self.properties.space;
        let dofs = if space.is_2d() { 2 } else { 6 };
        let driven = |region: &Region| {
            region.reference_point().is_none_or(|point| {
                self.constraints.iter().any(|c| {
                    matches!(c, Constraint::RigidBody(body) if body.active && body.reference_point == point)
                })
            })
        };
        for (i, bc) in step.boundary_conditions.iter().enumerate() {
            if bc.active && step.kind.supports_boundary(&bc.kind) && !driven(&bc.region) {
                findings.push(Finding::new(
                    ModelItem::BoundaryCondition(s, i),
                    Problem::NoRigidBody,
                    format!("{}: {}", bc.name, bc.region.describe()),
                ));
            }
        }
        for (i, load) in step.loads.iter().enumerate() {
            if let LoadKind::Moving(moving) = &load.kind
                && load.active
                && step.kind.supports_load(&load.kind)
                && let Ok(frames) =
                    moving.frames(self, mesh, &load.region.faces(mesh), space.is_2d())
            {
                for wheel in 0..moving.wheels.len() {
                    let off = (frames.iter())
                        .filter(|f| f.on_region.get(wheel).is_some_and(|&s| s < 1.0 - 1e-6))
                        .count();
                    if off > 0 {
                        findings.push(Finding::new(
                            ModelItem::Load(s, i),
                            Problem::WheelOffSurface,
                            format!(
                                "{}: wheel {} at {off} of {} positions",
                                load.name,
                                wheel + 1,
                                frames.len()
                            ),
                        ));
                    }
                }
            }
            if load.active && step.kind.supports_load(&load.kind) && !driven(&load.region) {
                findings.push(Finding::new(
                    ModelItem::Load(s, i),
                    Problem::NoRigidBody,
                    format!("{}: {}", load.name, load.region.describe()),
                ));
            }
        }
        for (i, output) in step.history_outputs.iter().enumerate() {
            if let Some(region) = output.kind.region()
                && output.active
                && !driven(region)
            {
                findings.push(Finding::new(
                    ModelItem::HistoryOutput(s, i),
                    Problem::NoRigidBody,
                    format!("{}: {}", output.name, region.describe()),
                ));
            }
        }
        // Value of each constrained node and degree of freedom, with the boundary condition.
        let mut fixed: HashMap<(NodeId, usize), (usize, f64)> = HashMap::new();
        // Symmetry planes that are no coordinate planes, by boundary condition.
        let mut skewed: Vec<(usize, SymmetryPlane)> = Vec::new();
        // Whether a boundary condition moves something: remote displacements and those of
        // reference points hold no nodes of the mesh, so `fixed` does not see them.
        let mut moved = false;
        // Supports of displacements in a coordinate system of their own.
        let mut local_supports: Vec<(NodeId, [f64; 3], bool)> = Vec::new();
        for (i, bc) in step.boundary_conditions.iter().enumerate() {
            // A step leaves out what it cannot take, as the input file does.
            if !bc.active
                || !step.kind.supports_boundary(&bc.kind)
                || bc.region.missing_reference(mesh).is_some()
            {
                continue;
            }
            let formulation = self.formulation(mesh, s, i);
            if let BoundaryKind::Displacement(displacement) = &bc.kind {
                moved |= (displacement.values.iter()).any(|v| v.is_some_and(|v| v != 0.0));
            }
            let values: Vec<(usize, f64)> = match &bc.kind {
                // The nodes follow the rigid body of a point, see `check_rigid_body`.
                BoundaryKind::Displacement(_) if formulation != Some(Formulation::Nodes) => {
                    Vec::new()
                }
                // Directions of a coordinate system of its own hold each node along them.
                BoundaryKind::Displacement(displacement) if displacement.system != GLOBAL => {
                    if let Some(system) = self.coordinate_system_or_global(&displacement.system) {
                        for node in bc.region.nodes(mesh) {
                            let Some(x) = mesh.node(node) else {
                                continue;
                            };
                            let Ok(axes) = system.directions_at(x) else {
                                continue;
                            };
                            for d in (0..dofs).filter(|&d| displacement.values[d].is_some()) {
                                local_supports.push((node, axes[d % 3], d >= 3));
                            }
                        }
                    }
                    Vec::new()
                }
                BoundaryKind::Displacement(displacement) => (displacement.values.iter())
                    .enumerate()
                    .filter_map(|(d, v)| v.map(|v| (d, v)))
                    .collect(),
                // Degree of freedom 11.
                BoundaryKind::Temperature(t) => vec![(10, *t)],
                // The values come from the global model; NaN stands for them.
                BoundaryKind::Submodel { dofs: held, .. } => (0..dofs.min(3))
                    .filter(|&d| held[d])
                    .map(|d| (d, f64::NAN))
                    .collect(),
                BoundaryKind::Symmetry { normal } => {
                    match symmetry_plane(mesh, &bc.region, space, *normal) {
                        Err(error) => {
                            findings.push(Finding::new(
                                ModelItem::BoundaryCondition(s, i),
                                Problem::NoSymmetryPlane,
                                format!("{}: {error}", bc.name),
                            ));
                            Vec::new()
                        }
                        Ok(plane) => {
                            if let Some(warning) = plane.warning(space) {
                                let problem = if plane.is_planar() {
                                    Problem::SymmetryOffAxis
                                } else {
                                    Problem::SymmetryNotPlanar
                                };
                                findings.push(Finding::new(
                                    ModelItem::BoundaryCondition(s, i),
                                    problem,
                                    format!("{}: {warning}", bc.name),
                                ));
                            }
                            match plane.axis() {
                                Some(k) => symmetry_dofs(k, dofs)
                                    .into_iter()
                                    .map(|d| (d, 0.0))
                                    .collect(),
                                None => {
                                    skewed.push((i, plane));
                                    Vec::new()
                                }
                            }
                        }
                    }
                }
            };
            if let BoundaryKind::Submodel { .. } = bc.kind
                && self.properties.submodel_input().is_none()
            {
                findings.push(Finding::new(
                    ModelItem::BoundaryCondition(s, i),
                    Problem::NoGlobalResults,
                    format!(
                        "{} needs the results of a global model: in the project window, link the \
                         results of a step of the global analysis to Global results",
                        bc.name
                    ),
                ));
            }
            // The rotations of a reference point turn its rigid body.
            // Rotations held at zero do no harm; nothing turns solid nodes anyway.
            if let BoundaryKind::Displacement(displacement) = &bc.kind
                && formulation == Some(Formulation::Nodes)
                && displacement.values[3..]
                    .iter()
                    .any(|v| v.is_some_and(|v| v != 0.0))
                && bc.region.reference_point().is_none()
                && (space.is_2d() || !trusses.rotational.iter().any(|&r| r))
            {
                findings.push(Finding::new(
                    ModelItem::BoundaryCondition(s, i),
                    Problem::RotationsIgnored,
                    format!("{} prescribes rotations", bc.name),
                ));
            }
            let mut conflicts: HashMap<usize, usize> = HashMap::new();
            for node in bc.region.nodes(mesh) {
                for &(dof, value) in &values {
                    match fixed.insert((node, dof), (i, value)) {
                        Some((other, old)) if other != i && !same(old, value) => {
                            *conflicts.entry(other).or_default() += 1;
                        }
                        _ => {}
                    }
                }
            }
            for (other, count) in conflicts {
                findings.push(Finding::new(
                    ModelItem::BoundaryCondition(s, i),
                    Problem::ConflictingBoundaries,
                    format!(
                        "{count} degrees of freedom also in {} with a different value; {} applies",
                        step.boundary_conditions[other].name, bc.name
                    ),
                ));
            }
        }
        // The rotations given at each beam node, with the first boundary condition giving one.
        let mut beam_rotations: BTreeMap<NodeId, (usize, [bool; 3])> = BTreeMap::new();
        for (&(node, dof), &(i, _)) in &fixed {
            if (3..6).contains(&dof) && mesh.node_index(node).is_some_and(|k| trusses.beam[k]) {
                let entry = beam_rotations.entry(node).or_insert((i, [false; 3]));
                entry.0 = entry.0.min(i);
                entry.1[dof - 3] = true;
            }
        }
        let mut partial: BTreeMap<usize, Vec<NodeId>> = BTreeMap::new();
        for (&node, &(i, given)) in &beam_rotations {
            if given.iter().any(|&g| !g) {
                partial.entry(i).or_default().push(node);
            }
        }
        for (i, nodes) in partial {
            let shown: Vec<String> = nodes.iter().take(5).map(|n| n.to_string()).collect();
            findings.push(Finding::new(
                ModelItem::BoundaryCondition(s, i),
                Problem::PartialBeamRotations,
                format!(
                    "{}: not all rotations given at beam node{} {}{}",
                    step.boundary_conditions[i].name,
                    if nodes.len() == 1 { "" } else { "s" },
                    shown.join(", "),
                    if nodes.len() > 5 { ", ..." } else { "" }
                ),
            ));
        }
        // Supports of the skewed symmetry planes: the normal displacement and the rotations
        // about the in-plane axes.
        let mut skewed_supports = local_supports;
        for (index, &(i, plane)) in skewed.iter().enumerate() {
            let nodes = step.boundary_conditions[i].region.nodes(mesh);
            let (t1, t2) = plane.in_plane_axes();
            for &node in &nodes {
                skewed_supports.push((node, plane.normal, false));
                if dofs == 6 {
                    skewed_supports.push((node, t1, true));
                    skewed_supports.push((node, t2, true));
                }
            }
            // The local system of the nodes turns whatever else acts on them.
            let nodes: HashSet<NodeId> = nodes.into_iter().collect();
            let mut shared: Vec<&str> = Vec::new();
            // Zero displacements in all directions, or zero rotations about all axes, are
            // the same in any system.
            let mut held: HashMap<NodeId, Vec<(usize, f64)>> = HashMap::new();
            for (&(node, dof), &(_, value)) in &fixed {
                if nodes.contains(&node) {
                    held.entry(node).or_default().push((dof, value));
                }
            }
            let turned = held.values().any(|values| {
                [0..dofs.min(3), 3..dofs].into_iter().any(|block| {
                    let inside: Vec<_> = values.iter().filter(|(d, _)| block.contains(d)).collect();
                    !inside.is_empty()
                        && (inside.len() < block.len() || inside.iter().any(|(_, v)| *v != 0.0))
                })
            });
            if turned {
                shared.push("boundary conditions");
            }
            let other_planes = skewed.iter().enumerate().any(|(other, &(j, p))| {
                other != index
                    && p.normal != plane.normal
                    && (step.boundary_conditions[j].region.nodes(mesh).iter())
                        .any(|n| nodes.contains(n))
            });
            if other_planes {
                shared.push("other skewed symmetry planes");
            }
            let point_loads = step.kind.supports_loads()
                && step.loads.iter().any(|load| {
                    load.active
                        && matches!(load.kind, LoadKind::Force { .. } | LoadKind::Moment(_))
                        && load.region.nodes(mesh).iter().any(|n| nodes.contains(n))
                });
            if point_loads {
                shared.push("point loads");
            }
            if !shared.is_empty() {
                findings.push(Finding::new(
                    ModelItem::BoundaryCondition(s, i),
                    Problem::SkewedSymmetryShared,
                    format!(
                        "{}: its nodes also carry {}",
                        step.boundary_conditions[i].name,
                        shared.join(" and ")
                    ),
                ));
            }
        }
        // A defined temperature strains the model like a load, if the materials expand.
        let mut temperatures = false;
        if step.kind.supports_defined_fields() {
            for (i, field) in step.defined_fields.iter().enumerate() {
                let missing = match &field.kind {
                    DefinedFieldKind::Temperature(_) => field.region.missing_reference(mesh),
                    DefinedFieldKind::TemperatureFromFile { .. } => None,
                };
                if !field.active || missing.is_some() {
                    continue;
                }
                temperatures = true;
                let used = |m: &&Material| self.sections.iter().any(|s| s.material == m.name);
                let expands = (self.materials.iter().filter(used)).any(|m| m.expansion.is_some());
                if !expands {
                    findings.push(Finding::new(
                        ModelItem::DefinedField(s, i),
                        Problem::NoExpansion,
                        format!("{}: no material with thermal expansion", field.name),
                    ));
                }
            }
        }
        if step.kind.supports_loads() {
            let mut loaded = temperatures;
            for (i, load) in step.loads.iter().enumerate() {
                if !load.active
                    || !step.kind.supports_load(&load.kind)
                    || load.region.missing_reference(mesh).is_some()
                {
                    continue;
                }
                loaded = true;
                let directions: Vec<usize> = match load.kind {
                    LoadKind::Force { value: f, .. } => {
                        (0..3).filter(|&d| f[d] != 0.0 && d < dofs).collect()
                    }
                    LoadKind::Moment(m) => (0..3)
                        .filter(|&d| m[d] != 0.0 && d + 3 < dofs)
                        .map(|d| d + 3)
                        .collect(),
                    // Acts on its own node, which nothing holds.
                    LoadKind::PreTension { .. } => Vec::new(),
                    LoadKind::Pressure(_) | LoadKind::Centrifugal { .. } => {
                        (0..dofs.min(3)).collect()
                    }
                    LoadKind::Gravity(g) => (0..3).filter(|&d| g[d] != 0.0 && d < dofs).collect(),
                    // Heat flows go into temperatures, which nothing but a temperature holds.
                    _ => Vec::new(),
                };
                let nodes = load.region.nodes(mesh);
                let held = !nodes.is_empty()
                    && !directions.is_empty()
                    && nodes
                        .iter()
                        .all(|&n| directions.iter().all(|&d| fixed.contains_key(&(n, d))));
                if held {
                    findings.push(Finding::new(
                        ModelItem::Load(s, i),
                        Problem::LoadOnFixedNodes,
                        format!("{}: all {} nodes fixed", load.name, nodes.len()),
                    ));
                }
            }
            let displaced = moved || fixed.values().any(|&(_, v)| v != 0.0);
            // A Dynamic step may start from an initial velocity instead of a load.
            let started = matches!(step.kind, StepKind::Dynamic(_))
                && (self.initial_conditions.iter()).any(|c| c.active && c.kind.is_velocity());
            if !loaded && !displaced && !started {
                findings.push(Finding::new(
                    ModelItem::Step(s),
                    Problem::NoLoad,
                    format!("{} has no active load", step.name),
                ));
            }
        }
        // Free-free eigenfrequencies are fine; a static or buckle step needs every part held.
        let contact_pairs = (self.contact_pairs.iter()).filter(|c| c.active).count();
        if let StepKind::Frequency(settings) = &step.kind
            && !settings.perturbation
            && contact_pairs > 0
        {
            findings.push(Finding::new(
                ModelItem::Step(s),
                Problem::ContactIgnored,
                format!(
                    "{}: {contact_pairs} contact pair(s) ignored without Perturbation",
                    step.name
                ),
            ));
        }
        if let StepKind::ComplexFrequency(settings) = &step.kind {
            if contact_pairs > 0 {
                findings.push(Finding::new(
                    ModelItem::Step(s),
                    Problem::ContactWithCoriolis,
                    format!(
                        "{}: {contact_pairs} contact pair(s) in the model",
                        step.name
                    ),
                ));
            }
            let before = || self.steps[..s].iter().filter(|s| s.active);
            let stored = before().rev().find_map(|s| match &s.kind {
                StepKind::Frequency(f) => Some(f.storage),
                _ => None,
            });
            if stored != Some(true) {
                findings.push(Finding::new(
                    ModelItem::Step(s),
                    Problem::NoStoredModes,
                    format!("{}: no Frequency step with Storage before it", step.name),
                ));
            }
            let rotating = before().any(|s| {
                s.kind.supports_loads()
                    && (s.loads.iter())
                        .any(|l| l.active && matches!(l.kind, LoadKind::Centrifugal { .. }))
            });
            if settings.coriolis && !rotating {
                findings.push(Finding::new(
                    ModelItem::Step(s),
                    Problem::NoRotation,
                    format!("{}: no centrifugal load in a step before it", step.name),
                ));
            }
        }
        // A buckle step solves the static state of its loads first.
        let static_mechanical = matches!(
            step.kind,
            StepKind::Static(_)
                | StepKind::Visco(_)
                | StepKind::Buckle(_)
                | StepKind::CoupledTempDisp(_)
                | StepKind::UncoupledTempDisp(_)
        );
        if static_mechanical {
            self.check_rigid_body(s, mesh, check, trusses, &fixed, &skewed_supports, findings);
            self.check_trusses(s, mesh, trusses, &fixed, findings);
        }
    }

    /// Degrees of freedom (0..6) the active boundary conditions of the step prescribe at a
    /// reference point.
    fn point_dofs(&self, s: usize, point: &str) -> Vec<usize> {
        let step = &self.steps[s];
        let dofs = if self.properties.space.is_2d() { 2 } else { 6 };
        let mut out = Vec::new();
        for bc in &step.boundary_conditions {
            if !bc.active
                || !step.kind.supports_boundary(&bc.kind)
                || bc.region.reference_point() != Some(point)
            {
                continue;
            }
            match bc.kind {
                BoundaryKind::Displacement(ref displacement) => {
                    out.extend((0..dofs).filter(|&d| displacement.values[d].is_some()));
                }
                BoundaryKind::Submodel { dofs: driven, .. } => {
                    out.extend((0..dofs.min(3)).filter(|&d| driven[d]));
                }
                // A reference point fixes no plane.
                BoundaryKind::Temperature(_) | BoundaryKind::Symmetry { .. } => {}
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }

    /// Finds pieces of the model that can move as a rigid body in a static step: the rigid
    /// body motions a support at point x in direction n prevents are those with
    /// n . (t + w x (x - c)) != 0, so a piece is held when the rows [n, (x - c) x n] of its
    /// supports have full rank. Ties join pieces; contact pairs join them only loosely,
    /// except tied ones: CalculiX activates tied contact from the first iteration,
    /// whatever the gap, so it joins pieces like a tie.
    #[allow(clippy::too_many_arguments)]
    fn check_rigid_body(
        &self,
        s: usize,
        mesh: &FeMesh,
        check: &MeshCheck,
        trusses: &Trusses,
        fixed: &HashMap<(NodeId, usize), (usize, f64)>,
        skewed: &[(NodeId, [f64; 3], bool)],
        findings: &mut Vec<Finding>,
    ) {
        let Some((min, max)) = mesh.bounds() else {
            return;
        };
        let center: [f64; 3] = std::array::from_fn(|i| 0.5 * (min[i] + max[i]));
        let size = (0..3)
            .map(|i| max[i] - min[i])
            .fold(0.0, f64::max)
            .max(1e-30);
        let space = self.properties.space;
        let mut grams = vec![[[0.0; 6]; 6]; check.pieces];
        // A support holds its node in a direction, or a beam or shell node about an axis.
        let mut support = |node: NodeId, direction: [f64; 3], rotation: bool| {
            let (Some(piece), Some(x)) = (check.piece_of(mesh, node), mesh.node(node)) else {
                return;
            };
            let row = if rotation {
                match space {
                    ModelSpace::ThreeD => [0.0, 0.0, 0.0, direction[0], direction[1], direction[2]],
                    // A plane model turns about z only.
                    ModelSpace::PlaneStress | ModelSpace::PlaneStrain => {
                        [0.0, 0.0, direction[2], 0.0, 0.0, 0.0]
                    }
                    ModelSpace::Axisymmetric => [0.0; 6],
                }
            } else {
                let r: [f64; 3] = std::array::from_fn(|i| (x[i] - center[i]) / size);
                rigid_row(space, r, direction)
            };
            let gram = &mut grams[piece as usize];
            for a in 0..6 {
                for b in 0..6 {
                    gram[a][b] += row[a] * row[b];
                }
            }
        };
        let axis = |d: usize| std::array::from_fn(|i| if i == d { 1.0 } else { 0.0 });
        for &(node, dof) in fixed.keys() {
            if dof < 3 {
                support(node, axis(dof), false);
            } else if dof < 6
                && space == ModelSpace::ThreeD
                && (mesh.node_index(node)).is_some_and(|i| trusses.rotational[i])
            {
                support(node, axis(dof - 3), true);
            }
        }
        for &(node, direction, rotation) in skewed {
            let rotates = space == ModelSpace::ThreeD
                && (mesh.node_index(node)).is_some_and(|i| trusses.rotational[i]);
            if !rotation || rotates {
                support(node, direction, rotation);
            }
        }
        let mut ties = UnionFind::new(check.pieces);
        let mut contacts = UnionFind::new(check.pieces);
        let pieces = |region: &crate::Region| -> Vec<usize> {
            let mut pieces: Vec<usize> = (region.nodes(mesh).into_iter())
                .filter_map(|n| check.piece_of(mesh, n))
                .map(|p| p as usize)
                .collect();
            pieces.sort_unstable();
            pieces.dedup();
            pieces
        };
        let join = |union: &mut UnionFind, a: &[usize], b: &[usize]| {
            for &x in a.iter().chain(b) {
                union.join(a.first().copied().unwrap_or(x), x);
            }
        };
        for constraint in self.constraints.iter().filter(|c| c.active()) {
            match constraint {
                Constraint::Spring(c) if c.master.is_none() => {
                    for node in c.region.nodes(mesh) {
                        for d in (0..3).filter(|&d| c.stiffness[d] > 0.0) {
                            support(node, axis(d), false);
                        }
                    }
                }
                Constraint::CompressionOnly(c) => {
                    for (node, normal) in face_normals(mesh, &c.region) {
                        support(node, normal, false);
                    }
                }
                Constraint::Tie(_) | Constraint::Spring(_) => {
                    if let Some([master, slave]) = constraint.master_slave() {
                        let (a, b) = (pieces(master), pieces(slave));
                        join(&mut ties, &a, &b);
                        join(&mut contacts, &a, &b);
                    }
                }
                Constraint::NodeTie(c) => {
                    let tied = pieces(&c.region);
                    join(&mut ties, &tied, &[]);
                    join(&mut contacts, &tied, &[]);
                }
                Constraint::RigidBody(body) => {
                    let tied = pieces(&body.region);
                    join(&mut ties, &tied, &[]);
                    join(&mut contacts, &tied, &[]);
                    // A boundary condition on the reference point holds the body: its
                    // translations like a support at the node nearest the point, its
                    // rotations like those of a beam node.
                    let Some(point) = self.reference_point(&body.reference_point) else {
                        continue;
                    };
                    let Some(node) = nearest_node(mesh, &body.region, point.position) else {
                        continue;
                    };
                    for dof in self.point_dofs(s, &body.reference_point) {
                        if dof < 3 {
                            support(node, axis(dof), false);
                        } else if dof < 6 && space == ModelSpace::ThreeD {
                            support(node, axis(dof - 3), true);
                        }
                    }
                }
            }
        }
        // A displacement on a point holds its region like a rigid body with a boundary
        // condition on its point.
        let step = &self.steps[s];
        let mut excluded = None;
        for (i, bc) in step.boundary_conditions.iter().enumerate() {
            let BoundaryKind::Displacement(remote) = &bc.kind else {
                continue;
            };
            if !bc.active
                || !step.kind.supports_boundary(&bc.kind)
                || !matches!(self.formulation(mesh, s, i), Some(Formulation::Point(_)))
            {
                continue;
            }
            let excluded = excluded.get_or_insert_with(|| self.directly_held_nodes(mesh));
            let Ok(point) = remote.resolve(self, mesh, &bc.region, excluded) else {
                continue;
            };
            let axes = (point.axes).unwrap_or([axis(0), axis(1), axis(2)]);
            let held = remote.held(space);
            let region = Region::Nodes(point.nodes.clone());
            let tied = pieces(&region);
            join(&mut ties, &tied, &[]);
            join(&mut contacts, &tied, &[]);
            let Some(node) = nearest_node(mesh, &region, point.position) else {
                continue;
            };
            for dof in (0..6).filter(|&d| held[d].is_some()) {
                support(node, axes[dof % 3], dof >= 3);
            }
        }
        for tie in self.node_ties.iter().filter(|t| t.active) {
            let tied = pieces(&tie.region);
            join(&mut ties, &tied, &[]);
            join(&mut contacts, &tied, &[]);
        }
        for tie in self.ties.iter().filter(|t| t.active) {
            let (a, b) = (pieces(&tie.master), pieces(&tie.slave));
            join(&mut ties, &a, &b);
            join(&mut contacts, &a, &b);
        }
        for pair in self.contact_pairs.iter().filter(|c| c.active) {
            let (a, b) = (pieces(&pair.master), pieces(&pair.slave));
            let tied = (self.surface_interactions.iter())
                .any(|s| s.name == pair.interaction && s.is_tied());
            if tied {
                join(&mut ties, &a, &b);
            }
            join(&mut contacts, &a, &b);
        }
        // Regions of no separation, left in the lowered model, hold normal to the surfaces
        // only: like a contact.
        for region in self.contact_regions.iter().filter(|r| r.active) {
            join(
                &mut contacts,
                &pieces(&region.side_a),
                &pieces(&region.side_b),
            );
        }
        let sum = |union: &mut UnionFind| {
            let mut sums: HashMap<usize, [[f64; 6]; 6]> = HashMap::new();
            for (piece, gram) in grams.iter().enumerate() {
                let total = sums.entry(union.root(piece)).or_insert([[0.0; 6]; 6]);
                for a in 0..6 {
                    for b in 0..6 {
                        total[a][b] += gram[a][b];
                    }
                }
            }
            sums
        };
        let held = sum(&mut ties);
        let held_by_contact = sum(&mut contacts);
        let unknowns = match space {
            ModelSpace::ThreeD => 6,
            ModelSpace::Axisymmetric => 1,
            _ => 3,
        };
        let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();
        for piece in 0..check.pieces {
            groups.entry(ties.root(piece)).or_default().push(piece);
        }
        let mut groups: Vec<(usize, Vec<usize>)> = groups.into_iter().collect();
        groups.sort_by_key(|(_, pieces)| pieces[0]);
        for (root, group) in groups {
            let free = free_motions(&held[&root], unknowns);
            if free.is_empty() {
                continue;
            }
            let mut parts: Vec<usize> = (group.iter())
                .flat_map(|&p| check.piece_parts[p].iter().copied())
                .collect();
            parts.sort_unstable();
            parts.dedup();
            let names: Vec<&str> = (parts.iter())
                .filter_map(|&p| mesh.parts.get(p).map(|p| p.name.as_str()))
                .collect();
            let contact_root = contacts.root(group[0]);
            // Beam nodes held in all displacements: a single free motion that keeps two of
            // them in place is the turn about the line through them.
            let members: HashSet<usize> = group.iter().copied().collect();
            let pins = (fixed.keys())
                .filter(|&&(node, dof)| dof == 0 && fixed.contains_key(&(node, 1)))
                .filter(|&&(node, _)| fixed.contains_key(&(node, 2)))
                .filter(|&&(node, _)| mesh.node_index(node).is_some_and(|i| trusses.beam[i]))
                .filter(|&&(node, _)| {
                    check
                        .piece_of(mesh, node)
                        .is_some_and(|p| members.contains(&(p as usize)))
                })
                .count();
            let problem = if free_motions(&held_by_contact[&contact_root], unknowns).is_empty() {
                Problem::HeldOnlyByContact
            } else if space == ModelSpace::ThreeD && free.len() == 1 && pins >= 2 {
                Problem::SpinAboutPins
            } else {
                Problem::RigidBodyMotion
            };
            let what = if names.is_empty() {
                "Elements without part".to_string()
            } else if names.len() > 5 {
                format!("{} and {} more", names[..5].join(", "), names.len() - 5)
            } else {
                names.join(", ")
            };
            let detail = format!(
                "{}: {what}: {}",
                self.steps[s].name,
                describe_motions(space, &free)
            );
            findings.push(Finding::new(
                ModelItem::BoundaryConditions(s),
                problem,
                detail.clone(),
            ));
            for part in parts {
                findings.push(Finding::new(ModelItem::Part(part), problem, detail.clone()));
            }
        }
    }

    /// Finds truss nodes that can move without resistance in a static step. A bar holds its
    /// end nodes along its axis only, so a node is held when its bars, supports and springs
    /// span all three directions; tied nodes are written as one node and count together.
    /// Inner nodes of a bar meshed with several elements and the nodes of a plane truss
    /// fail this, and CalculiX solves the singular system without a word.
    fn check_trusses(
        &self,
        s: usize,
        mesh: &FeMesh,
        trusses: &Trusses,
        fixed: &HashMap<(NodeId, usize), (usize, f64)>,
        findings: &mut Vec<Finding>,
    ) {
        if trusses.bars.is_empty() || self.properties.space != ModelSpace::ThreeD {
            return;
        }
        let mut joined = UnionFind::new(mesh.node_count());
        // Nodes held by something this check does not follow, such as a tie to a solid.
        let mut held_otherwise = vec![false; mesh.node_count()];
        let mark = |region: &crate::Region, held: &mut Vec<bool>| {
            for index in region
                .nodes(mesh)
                .iter()
                .filter_map(|&n| mesh.node_index(n))
            {
                held[index] = true;
            }
        };
        let mut gram = vec![[[0.0; 6]; 6]; mesh.node_count()];
        let add = |gram: &mut Vec<[[f64; 6]; 6]>, node: NodeId, d: [f64; 3]| {
            if let Some(index) = mesh.node_index(node) {
                for a in 0..3 {
                    for b in 0..3 {
                        gram[index][a][b] += d[a] * d[b];
                    }
                }
            }
        };
        let axis =
            |d: usize| -> [f64; 3] { std::array::from_fn(|i| if i == d { 1.0 } else { 0.0 }) };
        for tie in self.node_ties.iter().filter(|t| t.active) {
            let nodes: Vec<usize> = (tie.region.nodes(mesh).into_iter())
                .filter_map(|n| mesh.node_index(n))
                .collect();
            for pair in nodes.windows(2) {
                joined.join(pair[0], pair[1]);
            }
        }
        for constraint in self.constraints.iter().filter(|c| c.active()) {
            match constraint {
                // Moved to the node ties when the project was read.
                Constraint::NodeTie(_) => {}
                Constraint::Spring(c) if c.master.is_none() => {
                    for node in c.region.nodes(mesh) {
                        for d in (0..3).filter(|&d| c.stiffness[d] > 0.0) {
                            add(&mut gram, node, axis(d));
                        }
                    }
                }
                Constraint::CompressionOnly(c) => mark(&c.region, &mut held_otherwise),
                Constraint::Tie(_) | Constraint::Spring(_) => {
                    for region in constraint.master_slave().into_iter().flatten() {
                        mark(region, &mut held_otherwise);
                    }
                }
                Constraint::RigidBody(body) => {
                    if !self.point_dofs(s, &body.reference_point).is_empty() {
                        mark(&body.region, &mut held_otherwise);
                    }
                }
            }
        }
        for tie in self.ties.iter().filter(|t| t.active) {
            mark(&tie.master, &mut held_otherwise);
            mark(&tie.slave, &mut held_otherwise);
        }
        let step = &self.steps[s];
        for (i, bc) in step.boundary_conditions.iter().enumerate() {
            if let BoundaryKind::Displacement(remote) = &bc.kind
                && bc.active
                && step.kind.supports_boundary(&bc.kind)
                && matches!(self.formulation(mesh, s, i), Some(Formulation::Point(_)))
                && remote.values.iter().any(Option::is_some)
            {
                mark(&bc.region, &mut held_otherwise);
            }
        }
        for pair in self.contact_pairs.iter().filter(|c| c.active) {
            mark(&pair.master, &mut held_otherwise);
            mark(&pair.slave, &mut held_otherwise);
        }
        for &(node, dof) in fixed.keys() {
            if dof < 3 {
                add(&mut gram, node, axis(dof));
            }
        }
        for &(_, [a, b]) in &trusses.bars {
            let (Some(x), Some(y)) = (mesh.node(a), mesh.node(b)) else {
                continue;
            };
            let d: [f64; 3] = std::array::from_fn(|i| y[i] - x[i]);
            let length = d.iter().map(|v| v * v).sum::<f64>().sqrt();
            if length > 0.0 {
                let d = d.map(|v| v / length);
                add(&mut gram, a, d);
                add(&mut gram, b, d);
            }
        }
        // Sum over the tied nodes; a group with a node of another element is held by it.
        let mut groups: HashMap<usize, ([[f64; 6]; 6], bool, NodeId)> = HashMap::new();
        for (index, &id) in mesh.node_ids().iter().enumerate() {
            let (truss, other) = (trusses.truss[index], trusses.other[index]);
            if !truss && !other {
                continue;
            }
            let entry = (groups.entry(joined.root(index))).or_insert(([[0.0; 6]; 6], false, id));
            for (sum, row) in entry.0.iter_mut().zip(&gram[index]) {
                for (sum, value) in sum.iter_mut().zip(row) {
                    *sum += value;
                }
            }
            entry.1 |= other || held_otherwise[index];
            entry.2 = entry.2.min(id);
        }
        let mut free: Vec<(NodeId, Vec<[f64; 6]>)> = (groups.into_values())
            .filter(|(_, held, _)| !held)
            .filter_map(|(gram, _, node)| {
                let (values, vectors) = eigen(&gram, 3);
                let free: Vec<[f64; 6]> = (0..3)
                    .filter(|&i| values[i] <= 1e-9)
                    .map(|i| vectors[i])
                    .collect();
                (!free.is_empty()).then_some((node, free))
            })
            .collect();
        if free.is_empty() {
            return;
        }
        free.sort_by_key(|(node, _)| *node);
        let examples: Vec<String> = (free.iter().take(3))
            .map(|(node, directions)| format!("{node} {}", describe_directions(directions)))
            .collect();
        let detail = format!(
            "{}: {} truss nodes not held, e.g. node {}",
            self.steps[s].name,
            free.len(),
            examples.join(", ")
        );
        let free_nodes: HashSet<NodeId> = free.iter().map(|(node, _)| *node).collect();
        findings.push(Finding::new(
            ModelItem::BoundaryConditions(s),
            Problem::Mechanism,
            detail.clone(),
        ));
        for (part, p) in mesh.parts.iter().enumerate() {
            let touched = (p.elements.iter().filter_map(|&e| mesh.element(e)))
                .any(|e| e.nodes.iter().any(|n| free_nodes.contains(n)));
            if touched {
                findings.push(Finding::new(
                    ModelItem::Part(part),
                    Problem::Mechanism,
                    detail.clone(),
                ));
            }
        }
    }
}

/// Whether the nodes of an element have rotations: beams (B31, B32) and shells (S3 to S8R)
/// have them; trusses, plane and solid elements only displacements. A line element of a
/// truss section is a truss whatever its type name says; see [`Trusses`].
fn has_rotations(element: &plx_mesh::Element) -> bool {
    match element.shape.family() {
        ElementFamily::Line => element.type_name.to_ascii_uppercase().starts_with('B'),
        ElementFamily::Surface => {
            !element.is_plane() && element.type_name.to_ascii_uppercase().starts_with('S')
        }
        ElementFamily::Solid => false,
    }
}

/// The trusses of the model. The mesh stores beams and trusses alike as line elements
/// (meshed lines are `B32`); only a truss section makes one a truss, which has no rotations
/// and is written as `T3D2` between its end nodes.
struct Trusses {
    /// Each truss element with its two end nodes.
    bars: Vec<(ElementId, [NodeId; 2])>,
    /// Whether each node, by node index, is an end node of a truss.
    truss: Vec<bool>,
    /// Whether each node, by node index, belongs to an element other than a truss.
    other: Vec<bool>,
    /// Whether each node, by node index, has rotations.
    rotational: Vec<bool>,
    /// Whether each node, by node index, belongs to a beam.
    beam: Vec<bool>,
}

impl Trusses {
    fn new(model: &FeModel, mesh: &FeMesh) -> Self {
        let ids: HashSet<ElementId> = (model.sections.iter())
            .filter(|s| matches!(s.kind, SectionKind::Truss { .. }))
            .flat_map(|s| s.region.elements(mesh))
            .collect();
        let mut bars = Vec::new();
        let mut truss = vec![false; mesh.node_count()];
        let mut other = vec![false; mesh.node_count()];
        let mut rotational = vec![false; mesh.node_count()];
        let mut beam = vec![false; mesh.node_count()];
        for element in mesh.elements() {
            if element.shape.family() == ElementFamily::Line && ids.contains(&element.id) {
                // The midside node of a 3-node line is left out of the written truss.
                if let (Some(&a), Some(&b)) = (element.nodes.first(), element.nodes.last()) {
                    bars.push((element.id, [a, b]));
                    for index in [a, b].into_iter().filter_map(|n| mesh.node_index(n)) {
                        truss[index] = true;
                    }
                }
                continue;
            }
            let rotations = has_rotations(element);
            let line = element.shape.family() == ElementFamily::Line;
            for index in element.nodes.iter().filter_map(|&n| mesh.node_index(n)) {
                other[index] = true;
                rotational[index] |= rotations;
                beam[index] |= rotations && line;
            }
        }
        Self {
            bars,
            truss,
            other,
            rotational,
            beam,
        }
    }
}

/// Names the directions a truss node is free in: two free directions are across a single
/// bar, one is out of the plane of its bars.
fn describe_directions(free: &[[f64; 6]]) -> String {
    match free {
        [f] => match (0..3).find(|&i| f[i].abs() > 0.99) {
            Some(i) => format!("free in {}", ["X", "Y", "Z"][i]),
            None => "free perpendicular to the plane of the trusses".into(),
        },
        [_, _] => "free across the truss".into(),
        _ => "completely free".into(),
    }
}

/// Whether two prescribed values are the same up to rounding.
fn same(a: f64, b: f64) -> bool {
    // Values read from the global model of a submodel.
    if a.is_nan() && b.is_nan() {
        return true;
    }
    (a - b).abs() <= 1e-12 * a.abs().max(b.abs())
}

fn list<'a>(ids: impl Iterator<Item = &'a ElementId>) -> String {
    ids.map(u32::to_string).collect::<Vec<_>>().join(", ")
}

/// The rigid body motions a support in `direction` at `r` (relative to the centre) prevents,
/// as a row over the model's rigid body unknowns: translations, then rotations.
/// The node of the region nearest to `point`.
fn nearest_node(mesh: &FeMesh, region: &Region, point: [f64; 3]) -> Option<NodeId> {
    let distance = |x: &[f64; 3]| (0..3).map(|i| (x[i] - point[i]).powi(2)).sum::<f64>();
    (region.nodes(mesh).into_iter())
        .filter_map(|n| mesh.node(n).map(|x| (n, distance(&x))))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(n, _)| n)
}

fn rigid_row(space: ModelSpace, r: [f64; 3], n: [f64; 3]) -> [f64; 6] {
    match space {
        ModelSpace::ThreeD => [
            n[0],
            n[1],
            n[2],
            r[1] * n[2] - r[2] * n[1],
            r[2] * n[0] - r[0] * n[2],
            r[0] * n[1] - r[1] * n[0],
        ],
        // A body of revolution can only move along its axis.
        ModelSpace::Axisymmetric => [n[1], 0.0, 0.0, 0.0, 0.0, 0.0],
        _ => [n[0], n[1], r[0] * n[1] - r[1] * n[0], 0.0, 0.0, 0.0],
    }
}

/// Nodes of the faces of a region with the face normal, the direction a compression only
/// support holds them in.
fn face_normals(mesh: &FeMesh, region: &crate::Region) -> Vec<(NodeId, [f64; 3])> {
    let mut out = Vec::new();
    for (element, face) in region.faces(mesh) {
        let Some(element) = mesh.element(element) else {
            continue;
        };
        let Some(topology) = element.faces().get(usize::from(face).wrapping_sub(1)) else {
            continue;
        };
        let corners: Vec<(NodeId, [f64; 3])> = (topology.corners.iter())
            .filter_map(|&l| element.nodes.get(l))
            .filter_map(|&n| Some((n, mesh.node(n)?)))
            .collect();
        let normal = match corners.as_slice() {
            [(_, a), (_, b)] => [-(b[1] - a[1]), b[0] - a[0], 0.0],
            [(_, a), (_, b), (_, c), ..] => {
                let u: [f64; 3] = std::array::from_fn(|i| b[i] - a[i]);
                let v: [f64; 3] = std::array::from_fn(|i| c[i] - a[i]);
                [
                    u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0],
                ]
            }
            _ => continue,
        };
        let length = normal.iter().map(|x| x * x).sum::<f64>().sqrt();
        if length > 0.0 {
            let normal = normal.map(|x| x / length);
            out.extend(corners.iter().map(|&(n, _)| (n, normal)));
        }
    }
    out
}

/// The rigid body motions the supports summed up in `gram` leave free, as an orthonormal
/// basis of the first `unknowns` rigid body unknowns.
fn free_motions(gram: &[[f64; 6]; 6], unknowns: usize) -> Vec<[f64; 6]> {
    let (values, vectors) = eigen(gram, unknowns);
    let largest = values.iter().copied().fold(0.0, f64::max);
    (0..unknowns)
        .filter(|&i| values[i] <= 1e-9 * largest.max(1e-300) || largest == 0.0)
        .map(|i| vectors[i])
        .collect()
}

/// Names the free motions, e.g. "translation in X, rotation about Z".
pub(crate) fn describe_motions(space: ModelSpace, free: &[[f64; 6]]) -> String {
    let project = |v: &[f64; 6]| -> f64 {
        free.iter()
            .map(|f| (0..6).map(|i| f[i] * v[i]).sum::<f64>().powi(2))
            .sum()
    };
    let unit = |i: usize| -> [f64; 6] { std::array::from_fn(|j| if i == j { 1.0 } else { 0.0 }) };
    type Axes<'a> = &'a [(usize, &'a str)];
    let (translations, rotations): (Axes, Axes) = match space {
        ModelSpace::ThreeD => (
            &[(0, "X"), (1, "Y"), (2, "Z")],
            &[(3, "X"), (4, "Y"), (5, "Z")],
        ),
        ModelSpace::Axisymmetric => (&[(0, "Y")], &[]),
        _ => (&[(0, "X"), (1, "Y")], &[(2, "Z")]),
    };
    if free.len() == translations.len() + rotations.len() {
        return "completely free, nothing holds them".into();
    }
    let mut parts = Vec::new();
    let mut found = 0;
    for &(i, name) in translations {
        if project(&unit(i)) > 0.99 {
            parts.push(format!("displacement in {name}"));
            found += 1;
        }
    }
    // The remaining free motions are rotations about axes through some point; each axis is
    // named by the largest component of its rotation.
    let mut axes: Vec<&str> = Vec::new();
    for f in free {
        let translation: f64 = (translations.iter())
            .filter(|(i, _)| project(&unit(*i)) > 0.99)
            .map(|&(i, _)| f[i] * f[i])
            .sum();
        if translation > 0.99 {
            continue;
        }
        if let Some(&(_, name)) =
            (rotations.iter()).max_by(|a, b| f[a.0].abs().total_cmp(&f[b.0].abs()))
            && !axes.contains(&name)
        {
            axes.push(name);
        }
    }
    let rotations_free = free.len().saturating_sub(found);
    for axis in axes.iter().take(rotations_free) {
        parts.push(format!("rotation about {axis}"));
    }
    if parts.is_empty() {
        parts.push(format!("{} motion(s)", free.len()));
    }
    format!("{} free", parts.join(", "))
}

/// Eigenvalues and eigenvectors of the upper left `n` x `n` block of a symmetric matrix by
/// Jacobi rotations; small enough to be exact for the six rigid body unknowns.
fn eigen(matrix: &[[f64; 6]; 6], n: usize) -> ([f64; 6], [[f64; 6]; 6]) {
    let mut a = *matrix;
    let mut v = [[0.0; 6]; 6];
    for (i, row) in v.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for _ in 0..100 {
        let off: f64 = (0..n)
            .flat_map(|i| (0..n).filter(move |&j| j != i).map(move |j| (i, j)))
            .map(|(i, j)| a[i][j] * a[i][j])
            .sum();
        if off < 1e-30 {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                if a[p][q].abs() < 1e-300 {
                    continue;
                }
                let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let t = if theta == 0.0 { 1.0 } else { t };
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for row in a.iter_mut().take(n) {
                    let (akp, akq) = (row[p], row[q]);
                    row[p] = c * akp - s * akq;
                    row[q] = s * akp + c * akq;
                }
                let (row_p, row_q) = (a[p], a[q]);
                for k in 0..n {
                    a[p][k] = c * row_p[k] - s * row_q[k];
                    a[q][k] = s * row_p[k] + c * row_q[k];
                }
                for row in v.iter_mut().take(n) {
                    let (vp, vq) = (row[p], row[q]);
                    row[p] = c * vp - s * vq;
                    row[q] = s * vp + c * vq;
                }
            }
        }
    }
    let values = std::array::from_fn(|i| a[i][i]);
    // Columns of v are the eigenvectors.
    let vectors = std::array::from_fn(|i| std::array::from_fn(|k| v[k][i]));
    (values, vectors)
}

/// Warnings CalculiX gives in its output after any run that point to wrong results: slave
/// nodes of ties it did not tie because the master side is too far away ("no tied MPC
/// generated for node ... too far away; distance: ... tolerance: ...").
pub fn diagnose_solver_warnings(lines: &[String]) -> Vec<Finding> {
    let text = lines.join(" ").to_ascii_lowercase();
    let marker = "too far away; distance:";
    let (mut count, mut distance, mut tolerance) = (0usize, 0.0f64, 0.0f64);
    for (at, _) in text.match_indices(marker) {
        let mut words = text[at + marker.len()..].split_whitespace();
        let Some(d) = words.next().and_then(|w| w.parse::<f64>().ok()) else {
            continue;
        };
        count += 1;
        distance = distance.max(d);
        if words.next() == Some("tolerance:")
            && let Some(t) = words.next().and_then(|w| w.parse::<f64>().ok())
        {
            tolerance = tolerance.max(t);
        }
    }
    // The Nodalis solver: "*WARNING in *TIE <name>: <n> slave node(s) are too far away
    // from the master surface and are not tied: <ids>".
    let plxs_marker = "slave node(s) are too far away from the master surface";
    let untied: usize = (lines.iter())
        .filter_map(|line| {
            let at = line.find(plxs_marker)?;
            line[..at].split_whitespace().last()?.parse::<usize>().ok()
        })
        .sum();
    if count == 0 && untied == 0 {
        return Vec::new();
    }
    if count == 0 {
        return vec![Finding::new(
            ModelItem::Analysis,
            Problem::TieNodesNotTied,
            format!(
                "{untied} slave nodes not tied: farther than the tolerance from the master side"
            ),
        )];
    }
    vec![Finding::new(
        ModelItem::Analysis,
        Problem::TieNodesNotTied,
        format!(
            "{count} slave nodes not tied: up to {distance:.4e} from the master side, \
             tolerance {tolerance:.4e} (lengths in the units of the input file)"
        ),
    )]
}

/// Problems the solver reports in its output after a failed run, mapped to the explanations of
/// the checks. Distorted elements and elements without material are also attached to their
/// parts.
pub fn diagnose_solver_output(lines: &[String], mesh: &FeMesh) -> Vec<Finding> {
    let text = lines.join(" ").to_ascii_lowercase();
    let text: String = text.split_whitespace().collect::<Vec<_>>().join(" ");
    let mut findings = Vec::new();
    let add = |findings: &mut Vec<Finding>, problem: Problem, detail: String| {
        if !findings.iter().any(|f| f.problem == problem) {
            findings.push(Finding::new(ModelItem::Analysis, problem, detail));
        }
    };
    let elements_after = |marker: &str| -> Vec<ElementId> {
        let mut ids: Vec<ElementId> = (text.match_indices(marker))
            .filter_map(|(at, _)| {
                let rest = &text[at + marker.len()..];
                rest.split_whitespace().next()?.parse().ok()
            })
            .collect();
        ids.sort_unstable();
        ids.dedup();
        ids
    };
    let mut part_findings = Vec::new();
    for (problem, marker) in [
        (
            Problem::DistortedElements,
            "nonpositive jacobian determinant in element",
        ),
        (Problem::NoSection, "no material was assigned to element"),
    ] {
        let ids = elements_after(marker);
        if ids.is_empty() {
            continue;
        }
        add(
            &mut findings,
            problem,
            format!(
                "{} element(s), e.g. {}",
                ids.len(),
                list(ids.iter().take(5))
            ),
        );
        for (p, part) in mesh.parts.iter().enumerate() {
            let in_part: Vec<ElementId> = (ids.iter())
                .filter(|id| part.elements.contains(id))
                .copied()
                .collect();
            if !in_part.is_empty() {
                part_findings.push(Finding::new(
                    ModelItem::Part(p),
                    problem,
                    format!(
                        "According to CalculiX: element(s) {}",
                        list(in_part.iter().take(5))
                    ),
                ));
            }
        }
    }
    let known: [(Problem, &[&str]); 14] = [
        (Problem::NoElastic, &["no elastic constants"]),
        (
            Problem::InvalidPlastic,
            &["reading *plastic", "plastic strain must be"],
        ),
        (Problem::NoDensity, &["no density was assigned"]),
        (Problem::NoConductivity, &["no conductivity constants"]),
        (Problem::NoSpecificHeat, &["no specific heat was assigned"]),
        (
            Problem::NoInitialTemperature,
            &["define initial conditions for the temperature"],
        ),
        (
            Problem::InvalidElastic,
            &["poisson coefficient should be less than 0.5"],
        ),
        (
            Problem::RigidBodyMotion,
            &["zero pivot", "matrix is singular", "singular matrix"],
        ),
        (
            Problem::NoConvergence,
            &[
                "too many cutbacks",
                "increment size smaller than minimum",
                "increment size is smaller than minimum",
                "too far apart",
            ],
        ),
        (
            Problem::IncrementExceedsStep,
            &["initial increment size exceeds step size"],
        ),
        (
            Problem::NoPhysicalConstants,
            &[
                "*radiate card was selected",
                "no *physical constants card",
                "radiation needs *physical constants",
            ],
        ),
        (Problem::MpcAndSpc, &["dependent side of a mpc and a spc"]),
        (Problem::RotationIn2d, &["usermpc: mpc of type"]),
        (Problem::RigidBeamNodes, &["must not be subject to a rigid"]),
    ];
    for (problem, markers) in known {
        if let Some(marker) = markers.iter().find(|m| text.contains(*m)) {
            // The line as CalculiX wrote it, if the marker does not span two lines.
            let line = (lines.iter().map(|l| l.trim()))
                .find(|l| l.to_ascii_lowercase().contains(marker))
                .unwrap_or(marker);
            add(
                &mut findings,
                problem,
                format!("The solver reports \"{line}\""),
            );
        }
    }
    if findings.is_empty()
        && let Some(line) = lines.iter().find(|l| l.contains("*ERROR"))
    {
        add(&mut findings, Problem::SolverError, line.trim().to_string());
    }
    findings.extend(part_findings);
    findings
}

/// Disjoint sets with path halving.
struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
        }
    }

    fn root(&mut self, mut i: usize) -> usize {
        while self.parent[i] != i {
            self.parent[i] = self.parent[self.parent[i]];
            i = self.parent[i];
        }
        i
    }

    fn join(&mut self, a: usize, b: usize) {
        let (a, b) = (self.root(a), self.root(b));
        if a != b {
            self.parent[b] = a;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::diagnose_solver_warnings as warnings_of;

    #[test]
    fn untied_nodes_of_the_nodalis_solver_are_found() {
        let lines = vec![
            "*WARNING: *WARNING in *TIE GLUE: 9 slave node(s) are too far away from the master surface and are not tied: 1001, 1002".to_string(),
        ];
        let found = warnings_of(&lines);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].problem, Problem::TieNodesNotTied);
        assert!(
            found[0].detail.starts_with("9 slave nodes"),
            "{}",
            found[0].detail
        );
    }

    use plx_mesh::{Element, ElementShape, Part};

    use super::*;
    use crate::{
        BoundaryCondition, ContactPair, Elastic, Hardening, Load, Material, Plastic, PlasticPoint,
        ReferencePoint, Region, RigidBody, Section, Step, SurfaceInteraction, Tie,
    };
    use crate::{InteractionProperty, SurfaceBehavior};

    /// Makes the first section a beam section of a rectangular cross-section.
    fn beam_section(model: &mut FeModel) {
        model.cross_sections.push(crate::CrossSection {
            name: "Rect".into(),
            profile: crate::BeamProfile::ALL[0].clone(),
        });
        model.sections[0].kind = crate::SectionKind::Beam(crate::BeamSection {
            cross_section: "Rect".into(),
            ..crate::BeamSection::DEFAULT
        });
    }

    /// Unit cubes side by side along x, one C3D8 and one part each; neighbours share nodes
    /// when `joined`.
    fn cubes(count: u32, joined: bool) -> FeMesh {
        let mut mesh = FeMesh::default();
        let mut next = 1;
        for c in 0..count {
            let x0 = if joined { c as f64 } else { 1.5 * c as f64 };
            let first = next;
            for [x, y, z] in [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [1.0, 1.0, 1.0],
                [0.0, 1.0, 1.0],
            ] {
                mesh.set_node(next, [x0 + x, y, z]);
                next += 1;
            }
            let mut nodes: Vec<NodeId> = (first..first + 8).collect();
            if joined && c > 0 {
                // The left face is the right face of the previous cube.
                let previous = first - 8;
                for (left, right) in [(0, 1), (3, 2), (4, 5), (7, 6)] {
                    nodes[left] = previous + right;
                }
            }
            mesh.add_element(Element {
                id: c + 1,
                type_name: "C3D8".into(),
                shape: ElementShape::Hex8,
                nodes,
            })
            .unwrap();
            mesh.parts.push(Part {
                name: format!("PART-{}", c + 1),
                elements: vec![c + 1],
            });
        }
        mesh
    }

    fn steel() -> Material {
        Material {
            name: "Steel".into(),
            density: Some(7.85e-9),
            elastic: Some(Elastic {
                young: 210000.0,
                poisson: 0.3,
            }),
            ..Material::default()
        }
    }

    /// Steel on all parts, a static step with the left face of the first cube fixed and a
    /// force on node 7.
    fn model(mesh: &FeMesh) -> FeModel {
        let mut step = Step::new_static("Step-1");
        step.boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Fixed-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![1, 4, 5, 8]),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        step.loads.push(Load {
            distribution: None,
            name: "Force-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![7]),
            kind: LoadKind::force([0.0, 0.0, -1.0]),
            amplitude: None,
            factor_amplitude: None,
        });
        FeModel {
            materials: vec![steel()],
            sections: vec![Section {
                name: "Section-1".into(),
                material: "Steel".into(),
                region: Region::Parts(mesh.parts.iter().map(|p| p.name.clone()).collect()),
                thickness: 1.0,
                kind: Default::default(),
            }],
            steps: vec![step],
            ..FeModel::default()
        }
    }

    fn check(model: &FeModel, mesh: &FeMesh) -> Vec<Finding> {
        model.check(mesh, &MeshCheck::new(mesh, model.properties.space))
    }

    fn problems(findings: &[Finding]) -> Vec<(ModelItem, Problem)> {
        findings.iter().map(|f| (f.item, f.problem)).collect()
    }

    /// A section that does not fit its elements is its own problem, not a missing reference.
    #[test]
    fn a_section_on_the_wrong_elements_is_an_invalid_section() {
        let mesh = cubes(2, true);
        let mut model = model(&mesh);
        model.sections[0].kind = crate::SectionKind::Membrane {
            thickness: 1.0,
            offset: 0.0,
        };
        let findings = check(&model, &mesh);
        assert_eq!(
            problems(&findings),
            [(ModelItem::Section(0), Problem::InvalidSection)]
        );
        assert!(
            findings[0].detail.contains("triangles or quads"),
            "{findings:?}"
        );
        // A missing material is reported once, as a missing reference.
        model.sections[0].material = "Gone".into();
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Section(0), Problem::MissingReference)]
        );
    }

    #[test]
    fn an_amplitude_nobody_uses_is_reported() {
        let mesh = cubes(2, true);
        let mut model = model(&mesh);
        model.amplitudes.push(crate::Amplitude::new("Ramp"));
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Amplitude(0), Problem::UnusedAmplitude)]
        );
        model.steps[0].loads[0].amplitude = Some("Ramp".into());
        assert_eq!(check(&model, &mesh), []);
        model.steps[0].loads[0].amplitude = None;
        model.steps[0].loads[0].factor_amplitude = Some("Ramp".into());
        assert_eq!(check(&model, &mesh), []);
    }

    #[test]
    fn a_bolt_locked_before_its_preload_is_reported() {
        let mesh = cubes(2, true);
        let mut model = model(&mesh);
        let force = model.steps[0].loads[0].clone();
        let mut bolt = force.clone();
        bolt.name = "Bolt".into();
        bolt.kind = LoadKind::PreTension {
            value: 1000.0,
            by_displacement: false,
            lock: false,
            direction: None,
        };
        model.steps[0].loads.push(bolt);
        let mut second = model.steps[0].clone();
        second.name = "Step-2".into();
        let LoadKind::PreTension { lock, .. } = &mut second.loads[1].kind else {
            unreachable!()
        };
        *lock = true;
        model.steps.push(second.clone());
        assert_eq!(check(&model, &mesh), []);
        model.steps[0].loads.pop();
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Load(1, 1), Problem::LockWithoutPreload)]
        );
    }

    #[test]
    fn a_visco_step_needs_a_creeping_material() {
        let mesh = cubes(2, true);
        let mut model = model(&mesh);
        let mut visco = Step::new_visco("Step-2");
        visco.boundary_conditions = model.steps[0].boundary_conditions.clone();
        visco.loads = model.steps[0].loads.clone();
        model.steps.push(visco);
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Step(1), Problem::NoCreep)]
        );
        model.materials[0].creep = Some(crate::Creep::default());
        assert_eq!(check(&model, &mesh), []);
        model.materials[0].creep.as_mut().unwrap().n = 0.0;
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Material(0), Problem::InvalidCreep)]
        );
    }

    #[test]
    fn cyclic_symmetry_needs_valid_sectors_and_nodal_diameters() {
        let mesh = cubes(2, true);
        let mut model = model(&mesh);
        // About the z axis through the origin: node 2 lies at (1, 0, 0), node 1 on the axis.
        let mut tie = crate::Tie::new("Tie-1");
        tie.master = Region::Nodes(vec![2]);
        tie.slave = Region::Nodes(vec![1]);
        tie.cyclic = Some(crate::CyclicSymmetry::default());
        model.ties.push(tie);
        let mut frequency = Step::new_frequency("Step-2");
        frequency.boundary_conditions = model.steps[0].boundary_conditions.clone();
        if let StepKind::Frequency(f) = &mut frequency.kind {
            f.nodal_diameters = Some([0, 7]);
        }
        model.steps.push(frequency);
        let cyclic = |model: &FeModel| -> Vec<ModelItem> {
            (check(model, &mesh).iter())
                .filter(|f| f.problem == Problem::InvalidCyclicSymmetry)
                .map(|f| f.item)
                .collect()
        };
        // A side on the axis has no angle.
        assert_eq!(cyclic(&model), [ModelItem::Tie(0)]);
        // (2, 1, 0) is 26.6 degrees from the master side: no whole number of sectors.
        model.ties[0].slave = Region::Nodes(vec![11]);
        assert_eq!(cyclic(&model), [ModelItem::Tie(0)]);
        // 90 degrees are four sectors, carrying the nodal diameters up to 2.
        model.ties[0].slave = Region::Nodes(vec![4]);
        assert_eq!(cyclic(&model), [ModelItem::Step(1)]);
        // 45 degrees are eight sectors, up to 4.
        model.ties[0].slave = Region::Nodes(vec![3]);
        assert_eq!(cyclic(&model), [ModelItem::Step(1)]);
        if let StepKind::Frequency(f) = &mut model.steps[1].kind {
            f.nodal_diameters = Some([0, 4]);
        }
        assert_eq!(cyclic(&model), []);
    }

    #[test]
    fn a_held_and_loaded_model_has_no_findings() {
        let mesh = cubes(2, true);
        assert_eq!(check(&model(&mesh), &mesh), []);
    }

    #[test]
    fn an_initial_velocity_needs_a_dynamic_step() {
        let mesh = cubes(2, true);
        let mut model = model(&mesh);
        model.initial_conditions.push(crate::InitialCondition {
            distribution: None,
            name: "Initial_Velocity-1".into(),
            active: true,
            region: Region::Nodes(vec![1]),
            kind: crate::InitialConditionKind::Velocity([1.0, 0.0, 0.0]),
        });
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::InitialCondition(0), Problem::VelocityIgnored)]
        );
        // A Dynamic step takes it up, even without a load.
        model.steps[0].kind = StepKind::Dynamic(crate::DynamicStep::default());
        model.steps[0].loads.clear();
        assert_eq!(check(&model, &mesh), []);
        model.initial_conditions[0].active = false;
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Step(0), Problem::NoLoad)]
        );
    }

    #[test]
    fn a_sensitivity_step_needs_the_fitting_step_before_it() {
        let mesh = cubes(2, true);
        let mut model = model(&mesh);
        let design = crate::SensitivityStep {
            design_region: Region::Nodes(vec![2, 3]),
            ..Default::default()
        };
        model.steps.push(Step {
            kind: StepKind::Sensitivity(design.clone()),
            ..Step::new_sensitivity("Step-2")
        });
        // Mass and strain energy after the static step.
        assert_eq!(check(&model, &mesh), []);
        // Strain energy has nothing to differentiate after a frequency step.
        model.steps[0].kind = StepKind::Frequency(Default::default());
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Step(1), Problem::NoSensitivityBase)]
        );
        // Eigenfrequencies need the stored modes.
        model.steps[1].kind = StepKind::Sensitivity(crate::SensitivityStep {
            mass: false,
            strain_energy: false,
            eigenfrequency: true,
            ..design.clone()
        });
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Step(1), Problem::NoSensitivityBase)]
        );
        let StepKind::Frequency(frequency) = &mut model.steps[0].kind else {
            unreachable!()
        };
        frequency.storage = true;
        assert_eq!(check(&model, &mesh), []);
        // A second sensitivity step cannot have a design region of its own.
        model.steps.push(Step {
            kind: StepKind::Sensitivity(crate::SensitivityStep {
                design_region: Region::Nodes(vec![2]),
                eigenfrequency: false,
                ..design
            }),
            ..Step::new_sensitivity("Step-3")
        });
        assert_eq!(
            problems(&check(&model, &mesh)),
            [
                (ModelItem::Step(2), Problem::NoSensitivityBase),
                (ModelItem::Step(2), Problem::DesignRegionIgnored),
            ]
        );
    }

    #[test]
    fn submodel_boundaries_load_the_step_and_need_the_global_results() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        let step = &mut model.steps[0];
        step.loads.clear();
        step.boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Submodel-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![2, 3, 6, 7]),
            kind: BoundaryKind::Submodel {
                step: 1,
                dofs: [true, true, true, false, false, false],
            },
            amplitude: None,
        });
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::BoundaryCondition(0, 1), Problem::NoGlobalResults)]
        );
        model.properties.kind = crate::ModelKind::Submodel;
        model.properties.global_results = Some("global.frd".into());
        assert_eq!(check(&model, &mesh), []);
        assert!(model.uses_global_results());
        model.steps[0].kind = StepKind::Frequency(Default::default());
        assert!(!model.uses_global_results());
    }

    #[test]
    fn symmetry_planes_need_planar_nodes_and_a_system_of_their_own() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        let symmetry = |nodes: Vec<NodeId>| BoundaryCondition {
            distribution: None,
            name: "Symmetry-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(nodes),
            kind: BoundaryKind::Symmetry { normal: None },
            amplitude: None,
        };
        // The face y = 0.
        model.steps[0]
            .boundary_conditions
            .push(symmetry(vec![1, 2, 5, 6]));
        assert_eq!(check(&model, &mesh), []);
        let item = ModelItem::BoundaryCondition(0, 1);
        model.steps[0].boundary_conditions[1] = symmetry(vec![1, 2, 3, 6]);
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(item, Problem::SymmetryNotPlanar)]
        );
        model.steps[0].boundary_conditions[1] = symmetry(vec![2]);
        assert!(problems(&check(&model, &mesh)).contains(&(item, Problem::NoSymmetryPlane)));
        // The diagonal plane x = y shares the fixed nodes 1 and 5, which stay fixed in any
        // system, and node 7 with the force, which would turn.
        model.steps[0].boundary_conditions[1] = symmetry(vec![1, 3, 5, 7]);
        let findings = check(&model, &mesh);
        assert_eq!(
            problems(&findings),
            [(item, Problem::SkewedSymmetryShared)],
            "{findings:?}"
        );
        assert!(findings[0].detail.ends_with("point loads"), "{findings:?}");
        model.steps[0].loads[0].region = Region::Nodes(vec![6]);
        assert_eq!(check(&model, &mesh), []);
        // Only U1 fixed at node 1 means something else in the turned system.
        model.steps[0].boundary_conditions[0].kind =
            each_node([Some(0.0), None, None, None, None, None]);
        assert!(problems(&check(&model, &mesh)).contains(&(item, Problem::SkewedSymmetryShared)));
    }

    #[test]
    fn missing_supports_name_the_free_motions() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        model.steps[0].boundary_conditions[0].region = Region::Nodes(vec![1]);
        let findings = check(&model, &mesh);
        assert_eq!(
            problems(&findings),
            [
                (ModelItem::BoundaryConditions(0), Problem::RigidBodyMotion),
                (ModelItem::Part(0), Problem::RigidBodyMotion),
            ]
        );
        assert!(
            findings[0].detail.contains("rotation about"),
            "{findings:?}"
        );
        assert!(!findings[0].detail.contains("displacement"), "{findings:?}");
        model.steps[0].boundary_conditions.clear();
        let detail = &check(&model, &mesh)[0].detail;
        assert!(
            detail.ends_with(": completely free, nothing holds them"),
            "{detail}"
        );
        // Two corners fixed leave the rotation about the line through them.
        model.steps[0].boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Fixed-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![1, 2]),
            kind: BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        });
        let detail = &check(&model, &mesh)[0].detail;
        assert!(detail.ends_with(": rotation about X free"), "{detail}");
        // A spring on a third corner holds it.
        model.constraints.push(Constraint::Spring(crate::Spring {
            region: Region::Nodes(vec![7]),
            stiffness: [0.0, 0.0, 1.0],
            ..crate::Spring::new("Spring-1")
        }));
        assert_eq!(check(&model, &mesh), []);
    }

    #[test]
    fn unconnected_parts_need_a_tie_and_contact_holds_them_only_loosely() {
        let mut mesh = cubes(2, false);
        mesh.surfaces.insert(
            "RIGHT".into(),
            plx_mesh::SurfaceDefinition::ElementFaces(vec![(1, 4)]),
        );
        mesh.surfaces.insert(
            "LEFT".into(),
            plx_mesh::SurfaceDefinition::ElementFaces(vec![(2, 6)]),
        );
        let mut model = model(&mesh);
        assert_eq!(
            problems(&check(&model, &mesh)),
            [
                (ModelItem::BoundaryConditions(0), Problem::RigidBodyMotion),
                (ModelItem::Part(1), Problem::RigidBodyMotion),
            ]
        );
        let mut pair = ContactPair::new("Contact_Pair-1", "Surface_Interaction-1");
        pair.master = Region::Surface("RIGHT".into());
        pair.slave = Region::Surface("LEFT".into());
        model.contact_pairs.push(pair);
        model.surface_interactions.push(SurfaceInteraction {
            name: "Surface_Interaction-1".into(),
            properties: Vec::new(),
        });
        // Weak springs hold the part in the input file, so nothing is reported.
        assert_eq!(check(&model, &mesh), []);
        let springs = model.weak_springs(&mesh);
        assert_eq!(springs.len(), 1, "{springs:?}");
        assert_eq!(springs[0].name, "PLX_Weak_Springs_PART-2");
        assert_eq!(springs[0].region, Region::Nodes((9..=16).collect()));
        // A unit cube of steel: 1e-6 E L.
        assert!(
            (springs[0].stiffness[2] - 0.21).abs() < 1e-12,
            "{springs:?}"
        );
        model.contact_defaults.weak_springs = false;
        assert_eq!(
            problems(&check(&model, &mesh))[1],
            (ModelItem::Part(1), Problem::HeldOnlyByContact)
        );
        assert_eq!(model.weak_springs(&mesh), []);
        let mut tie = Tie::new("Tie-1");
        tie.master = Region::Surface("RIGHT".into());
        tie.slave = Region::Surface("LEFT".into());
        model.ties.push(tie);
        assert_eq!(check(&model, &mesh), []);
        // Tied contact holds from the first iteration, like a tie.
        model.ties.clear();
        model.surface_interactions[0].properties = vec![
            InteractionProperty::SurfaceBehavior(SurfaceBehavior::Tied { k: 1e7 }),
            InteractionProperty::Friction(crate::Friction::default()),
        ];
        assert_eq!(check(&model, &mesh), []);
    }

    #[test]
    fn ties_and_tied_contact_hold_a_chain_of_parts_from_one_support() {
        let count = 6;
        let mesh = cubes(count, false);
        let mut model = model(&mesh);
        model.surface_interactions.push(SurfaceInteraction {
            name: "Tied".into(),
            properties: vec![
                InteractionProperty::SurfaceBehavior(SurfaceBehavior::Tied { k: 1e7 }),
                InteractionProperty::Friction(crate::Friction::default()),
            ],
        });
        // Only the first cube is fixed; each cube is tied to the next, alternately by a
        // tie and by tied contact.
        for c in 0..count - 1 {
            let right = Region::Nodes([2, 3, 6, 7].map(|n| 8 * c + n).to_vec());
            let left = Region::Nodes([1, 4, 5, 8].map(|n| 8 * (c + 1) + n).to_vec());
            if c % 2 == 0 {
                let mut tie = Tie::new(format!("Tie-{c}"));
                (tie.master, tie.slave) = (right, left);
                model.ties.push(tie);
            } else {
                let mut pair = ContactPair::new(format!("Contact_Pair-{c}"), "Tied");
                (pair.master, pair.slave) = (right, left);
                model.contact_pairs.push(pair);
            }
        }
        assert_eq!(check(&model, &mesh), []);
        // Without the link in the middle the far half moves freely.
        model.ties.remove(1);
        assert!(
            problems(&check(&model, &mesh))
                .contains(&(ModelItem::Part(5), Problem::RigidBodyMotion))
        );
    }

    #[test]
    fn contact_pairs_need_perturbation_in_frequency_steps_and_spoil_coriolis() {
        let mesh = cubes(2, true);
        let mut model = model(&mesh);
        model
            .contact_pairs
            .push(ContactPair::new("Contact_Pair-1", "Tied"));
        let mut frequency = Step::new_frequency("Step-2");
        let StepKind::Frequency(settings) = &mut frequency.kind else {
            unreachable!()
        };
        settings.storage = true;
        model.steps.push(frequency);
        let modal = |model: &FeModel| {
            (check(model, &mesh).into_iter())
                .filter(|f| {
                    matches!(
                        f.problem,
                        Problem::ContactIgnored | Problem::ContactWithCoriolis
                    )
                })
                .map(|f| (f.item, f.problem))
                .collect::<Vec<_>>()
        };
        assert_eq!(
            modal(&model),
            [(ModelItem::Step(1), Problem::ContactIgnored)]
        );
        let StepKind::Frequency(settings) = &mut model.steps[1].kind else {
            unreachable!()
        };
        settings.perturbation = true;
        assert_eq!(modal(&model), []);
        model.steps.push(Step::new_complex_frequency("Step-3"));
        assert_eq!(
            modal(&model),
            [(ModelItem::Step(2), Problem::ContactWithCoriolis)]
        );
        model.contact_pairs[0].active = false;
        assert_eq!(modal(&model), []);
    }

    #[test]
    fn tied_contact_needs_friction_and_surface_to_surface() {
        let mesh = cubes(2, true);
        let mut model = model(&mesh);
        model
            .contact_pairs
            .push(ContactPair::new("Contact_Pair-1", "Tied"));
        model.surface_interactions.push(SurfaceInteraction {
            name: "Tied".into(),
            properties: vec![InteractionProperty::SurfaceBehavior(
                SurfaceBehavior::Tied { k: 1e7 },
            )],
        });
        let tied = |model: &FeModel| {
            (check(model, &mesh).into_iter())
                .filter(|f| f.problem == Problem::InvalidTiedContact)
                .map(|f| f.detail)
                .collect::<Vec<_>>()
        };
        assert_eq!(
            tied(&model),
            ["Contact_Pair-1: tied, but Tied has no Friction"]
        );
        (model.surface_interactions[0].properties)
            .push(InteractionProperty::Friction(crate::Friction::default()));
        assert_eq!(tied(&model), Vec::<String>::new());
        model.contact_pairs[0].method = ContactMethod::NodeToSurface;
        assert_eq!(
            tied(&model),
            ["Contact_Pair-1: tied, but method Node to surface"]
        );
        model.contact_pairs[0].active = false;
        assert_eq!(tied(&model), Vec::<String>::new());
    }

    #[test]
    fn frequency_steps_may_be_free_but_need_a_density() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        model.steps = vec![Step::new_frequency("Step-1")];
        assert_eq!(check(&model, &mesh), []);
        model.materials[0].density = None;
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Material(0), Problem::NoDensity)]
        );
    }

    #[test]
    fn body_loads_need_a_density() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        model.materials[0].density = None;
        assert_eq!(check(&model, &mesh), []);
        model.steps[0].loads.push(Load {
            distribution: None,
            name: "Gravity-1".into(),
            active: true,
            visible: true,
            region: Region::Parts(mesh.parts.iter().map(|p| p.name.clone()).collect()),
            kind: LoadKind::Gravity([0.0, 0.0, -9.81]),
            amplitude: None,
            factor_amplitude: None,
        });
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Material(0), Problem::NoDensity)]
        );
    }

    #[test]
    fn materials_and_sections_are_checked() {
        let mesh = cubes(2, true);
        let mut model = model(&mesh);
        model.materials[0].elastic = None;
        model.sections[0].region = Region::Parts(vec!["PART-1".into()]);
        assert_eq!(
            problems(&check(&model, &mesh)),
            [
                (ModelItem::Material(0), Problem::NoElastic),
                (ModelItem::Part(1), Problem::NoSection),
            ]
        );
        model.materials[0].elastic = Some(Elastic {
            young: 1.0,
            poisson: 0.5,
        });
        model.sections[0].region = Region::ElementSet("ALL".into());
        let mut mesh = mesh;
        mesh.element_sets.insert("ALL".into(), vec![1, 2]);
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Material(0), Problem::InvalidElastic)]
        );
    }

    #[test]
    fn a_reference_point_needs_an_active_rigid_body_and_holds_it() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        model
            .reference_points
            .push(ReferencePoint::new("RP-1", [0.0, 0.5, 0.5]));
        // The support moves from the nodes to the reference point: without a rigid body
        // the point has no node, the cube is loose.
        model.steps[0].boundary_conditions[0].region = Region::ReferencePoint("RP-1".into());
        let found = problems(&check(&model, &mesh));
        assert!(
            found.contains(&(ModelItem::BoundaryCondition(0, 0), Problem::NoRigidBody)),
            "{found:?}"
        );
        assert!(
            found.contains(&(ModelItem::Part(0), Problem::RigidBodyMotion)),
            "{found:?}"
        );
        let mut body = RigidBody::new("Rigid_Body-1", "RP-1");
        body.region = Region::Nodes(vec![1, 4, 5, 8]);
        model.constraints.push(Constraint::RigidBody(body));
        assert_eq!(problems(&check(&model, &mesh)), []);
        // An inactive body does not count.
        *model.constraints[0].active_mut() = false;
        let found = problems(&check(&model, &mesh));
        assert!(
            found.contains(&(ModelItem::BoundaryCondition(0, 0), Problem::NoRigidBody)),
            "{found:?}"
        );
        *model.constraints[0].active_mut() = true;
        // Only the translations held: the cube can still turn about the point.
        model.steps[0].boundary_conditions[0].kind =
            BoundaryKind::displacement([Some(0.0), Some(0.0), Some(0.0), None, None, None]);
        let found = problems(&check(&model, &mesh));
        assert!(
            found.contains(&(ModelItem::Part(0), Problem::RigidBodyMotion)),
            "{found:?}"
        );
        // A moment on the point without the body is flagged too.
        model.steps[0].boundary_conditions[0].kind = BoundaryKind::displacement([Some(0.0); 6]);
        model.steps[0].loads[0].region = Region::ReferencePoint("RP-1".into());
        model.steps[0].loads[0].kind = LoadKind::Moment([1.0, 0.0, 0.0]);
        assert_eq!(problems(&check(&model, &mesh)), []);
        model.constraints.clear();
        let found = problems(&check(&model, &mesh));
        assert!(
            found.contains(&(ModelItem::Load(0, 0), Problem::NoRigidBody)),
            "{found:?}"
        );
    }

    #[test]
    fn untied_slave_nodes_are_found_in_the_solver_output() {
        let lines: Vec<String> = [
            " *WARNING in gentiedmpc: no tied MPC",
            "          generated for node       126382",
            "          opposite master face found but",
            "          too far away; distance:    1.6141397112631843E-003",
            "          tolerance:    9.5492346099999999E-004",
            " *WARNING in gentiedmpc: no tied MPC",
            "          generated for node       126385",
            "          opposite master face found but",
            "          too far away; distance:    1.7738618684124165E-003",
            "          tolerance:    9.5492346099999999E-004",
        ]
        .map(String::from)
        .into();
        let found = diagnose_solver_warnings(&lines);
        assert_eq!(
            problems(&found),
            [(ModelItem::Analysis, Problem::TieNodesNotTied)]
        );
        assert_eq!(
            found[0].detail,
            "2 slave nodes not tied: up to 1.7739e-3 from the master side, tolerance \
             9.5492e-4 (lengths in the units of the input file)"
        );
        assert!(diagnose_solver_warnings(&["Job finished".into()]).is_empty());
    }

    #[test]
    fn a_rigid_body_on_held_nodes_is_warned_of() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        model
            .reference_points
            .push(ReferencePoint::new("RP-1", [0.0; 3]));
        let held = model.steps[0].boundary_conditions[0].region.nodes(&mesh);
        let mut body = RigidBody::new("Rigid_Body-1", "RP-1");
        body.region = Region::Nodes(held[..2].to_vec());
        model.constraints.push(Constraint::RigidBody(body));
        let found = check(&model, &mesh);
        let finding = (found.iter())
            .find(|f| f.problem == Problem::HeldRigidBodyNodes)
            .unwrap();
        assert_eq!(finding.item, ModelItem::Constraint(0));
        assert_eq!(finding.severity(), Severity::Warning);
        assert!(
            finding.detail.contains("2 nodes held"),
            "{}",
            finding.detail
        );
    }

    #[test]
    fn a_remote_displacement_holds_its_region_through_its_point() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        let bc = &mut model.steps[0].boundary_conditions[0];
        bc.region = Region::Nodes(vec![1, 4, 5, 8]);
        bc.kind = BoundaryKind::Displacement(crate::Displacement::rigid([Some(0.0); 6]));
        assert_eq!(problems(&check(&model, &mesh)), []);
        // A rigid body constraint on some of the same nodes takes them away.
        model
            .reference_points
            .push(ReferencePoint::new("RP-1", [0.0; 3]));
        let mut body = RigidBody::new("Rigid_Body-1", "RP-1");
        body.region = Region::Nodes(vec![1, 2]);
        model.constraints.push(Constraint::RigidBody(body));
        let found = check(&model, &mesh);
        assert_eq!(
            problems(&found),
            [(ModelItem::BoundaryCondition(0, 0), Problem::SharedRigidBody)]
        );
        assert!(found[0].detail.ends_with("1 nodes also in Rigid_Body-1"));
        model.constraints.clear();
        // Translations alone let the cube turn about the point.
        model.steps[0].boundary_conditions[0].kind =
            BoundaryKind::Displacement(crate::Displacement::rigid([
                Some(0.0),
                Some(0.0),
                Some(0.0),
                None,
                None,
                None,
            ]));
        let found = problems(&check(&model, &mesh));
        assert!(
            found.contains(&(ModelItem::Part(0), Problem::RigidBodyMotion)),
            "{found:?}"
        );
    }

    #[test]
    fn a_moving_remote_or_reference_point_displacement_is_a_load() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        model.steps[0].loads.clear();
        let no_load = |model: &FeModel| {
            problems(&check(model, &mesh)).contains(&(ModelItem::Step(0), Problem::NoLoad))
        };
        assert!(no_load(&model));
        // A remote displacement holds no nodes of the mesh itself.
        let mut values = [Some(0.0); 6];
        model.steps[0].boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Remote_Displacement-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![2, 3, 6, 7]),
            kind: BoundaryKind::Displacement(crate::Displacement::rigid(values)),
            amplitude: None,
        });
        assert!(no_load(&model));
        values[1] = Some(-0.001);
        model.steps[0].boundary_conditions[1].kind =
            BoundaryKind::Displacement(crate::Displacement::rigid(values));
        assert!(!no_load(&model));
        // Neither does a displacement of a reference point.
        model
            .reference_points
            .push(ReferencePoint::new("RP-1", [1.0, 0.5, 0.5]));
        let mut body = RigidBody::new("Rigid_Body-1", "RP-1");
        body.region = Region::Nodes(vec![2, 3, 6, 7]);
        model.constraints.push(Constraint::RigidBody(body));
        let bc = &mut model.steps[0].boundary_conditions[1];
        bc.region = Region::ReferencePoint("RP-1".into());
        bc.kind = BoundaryKind::displacement([None, None, None, None, None, Some(0.0)]);
        assert!(no_load(&model));
        model.steps[0].boundary_conditions[1].kind =
            BoundaryKind::displacement([None, None, None, None, None, Some(0.01)]);
        assert!(!no_load(&model));
    }

    #[test]
    fn a_plastic_material_needs_elasticity_and_a_growing_hardening_curve() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        let plastic = |rows: &[(f64, f64, f64)]| {
            Some(Plastic {
                hardening: Hardening::Isotropic,
                points: rows
                    .iter()
                    .map(|&(stress, plastic_strain, temperature)| PlasticPoint {
                        stress,
                        plastic_strain,
                        temperature,
                    })
                    .collect(),
            })
        };
        model.materials[0].plastic = plastic(&[(235.0, 0.0, 0.0), (400.0, 0.2, 0.0)]);
        assert_eq!(problems(&check(&model, &mesh)), []);
        // Rows at a second temperature start at plastic strain 0 again.
        model.materials[0].plastic = plastic(&[
            (235.0, 0.0, 20.0),
            (400.0, 0.2, 20.0),
            (200.0, 0.0, 300.0),
            (300.0, 0.2, 300.0),
        ]);
        assert_eq!(problems(&check(&model, &mesh)), []);
        for rows in [
            &[(235.0, 0.1, 0.0)][..],
            &[(235.0, 0.0, 0.0), (400.0, 0.0, 0.0)],
            &[(0.0, 0.0, 0.0)],
            &[],
        ] {
            model.materials[0].plastic = plastic(rows);
            assert_eq!(
                problems(&check(&model, &mesh)),
                [(ModelItem::Material(0), Problem::InvalidPlastic)],
                "{rows:?}"
            );
        }
        model.materials[0].plastic = plastic(&[(235.0, 0.0, 0.0)]);
        model.materials[0].elastic = None;
        assert_eq!(
            problems(&check(&model, &mesh)),
            [
                (ModelItem::Material(0), Problem::NoElastic),
                (ModelItem::Material(0), Problem::InvalidPlastic),
            ]
        );
    }

    #[test]
    fn overlapping_boundary_conditions_and_held_loads_are_found() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        let step = &mut model.steps[0];
        step.boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Displacement-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![1, 2]),
            kind: each_node([Some(0.1), None, None, None, None, None]),
            amplitude: None,
        });
        step.loads[0].region = Region::Nodes(vec![4]);
        let findings = check(&model, &mesh);
        assert_eq!(
            problems(&findings),
            [
                (
                    ModelItem::BoundaryCondition(0, 1),
                    Problem::ConflictingBoundaries
                ),
                (ModelItem::Load(0, 0), Problem::LoadOnFixedNodes),
            ]
        );
        assert!(
            findings[0]
                .detail
                .starts_with("1 degrees of freedom also in Fixed-1")
        );
        // The same value is no conflict.
        model.steps[0].boundary_conditions[1].kind =
            BoundaryKind::displacement([Some(0.0), None, None, None, None, None]);
        model.steps[0].loads.clear();
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Step(0), Problem::NoLoad)]
        );
    }

    #[test]
    fn a_defined_temperature_is_a_load_and_needs_thermal_expansion() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        let parts: Vec<String> = mesh.parts.iter().map(|p| p.name.clone()).collect();
        model.steps[0].loads.clear();
        model.initial_conditions.push(crate::InitialCondition {
            distribution: None,
            name: "Initial_Temperature-1".into(),
            active: true,
            region: Region::Parts(parts.clone()),
            kind: crate::InitialConditionKind::Temperature(20.0),
        });
        model.steps[0].defined_fields.push(crate::DefinedField {
            distribution: None,
            name: "Defined_Temperature-1".into(),
            active: true,
            region: Region::Parts(parts),
            kind: DefinedFieldKind::Temperature(100.0),
            amplitude: None,
        });
        let findings = check(&model, &mesh);
        assert_eq!(
            problems(&findings),
            [(ModelItem::DefinedField(0, 0), Problem::NoExpansion)]
        );
        assert_eq!(
            findings[0].detail,
            "Defined_Temperature-1: no material with thermal expansion"
        );
        model.materials[0].expansion = Some(crate::Expansion::default());
        assert_eq!(check(&model, &mesh), []);
        model.initial_conditions[0].active = false;
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Step(0), Problem::NoInitialTemperature)]
        );
        model.initial_conditions[0].active = true;
        // A thermal step solves for the temperatures and takes no defined field.
        model.steps[0].kind = StepKind::CoupledTempDisp(Default::default());
        model.materials[0].conductivity = Some(50.0);
        model.steps[0].defined_fields[0].region = Region::Parts(vec!["Nowhere".into()]);
        assert!(
            check(&model, &mesh)
                .iter()
                .all(|f| f.problem != Problem::NoExpansion)
        );
    }

    /// A displacement that holds every node.
    fn each_node(values: [Option<f64>; 6]) -> BoundaryKind {
        BoundaryKind::Displacement(crate::Displacement {
            behavior: crate::Behavior::EachNode,
            ..crate::Displacement::new(values)
        })
    }

    #[test]
    fn a_face_of_solids_with_free_rotations_turns_about_a_point() {
        let mesh = cubes(2, true);
        let mut model = model(&mesh);
        // A clamp holds each node.
        assert_eq!(model.formulation(&mesh, 0, 0), Some(Formulation::Nodes));
        // Free rotations need a point: the face turns about its centre.
        let bc = &mut model.steps[0].boundary_conditions[0];
        bc.kind = BoundaryKind::displacement([Some(0.0), Some(0.0), Some(0.0), None, None, None]);
        let rigid = Some(Formulation::Point(crate::RemoteBehavior::Rigid));
        assert_eq!(model.formulation(&mesh, 0, 0), rigid);
        // So the cubes turn about the line through the point: a mechanism, no ignored
        // rotations.
        assert!(
            problems(&check(&model, &mesh))
                .iter()
                .all(|(_, p)| *p != Problem::RotationsIgnored)
        );
        // A turned face as well, but not a single node, nor one held directly.
        let bc = &mut model.steps[0].boundary_conditions[0];
        bc.kind = BoundaryKind::displacement([
            Some(0.0),
            Some(0.0),
            Some(0.0),
            Some(0.1),
            Some(0.0),
            Some(0.0),
        ]);
        assert_eq!(model.formulation(&mesh, 0, 0), rigid);
        let bc = &mut model.steps[0].boundary_conditions[0];
        bc.region = Region::Nodes(vec![1]);
        assert_eq!(model.formulation(&mesh, 0, 0), Some(Formulation::Nodes));
        let bc = &mut model.steps[0].boundary_conditions[0];
        bc.region = Region::Nodes(vec![1, 4, 5, 8]);
        bc.kind = each_node([Some(0.0), Some(0.0), Some(0.0), None, None, None]);
        assert_eq!(model.formulation(&mesh, 0, 0), Some(Formulation::Nodes));
        // An exposed rotation counts as turning, whatever its value.
        model.steps[0].boundary_conditions[0].kind = BoundaryKind::displacement([Some(0.0); 6]);
        model.exposed.push(crate::Exposed {
            key: 1,
            name: "Twist".into(),
            path: crate::ValuePath::new(
                crate::ItemRef::BoundaryCondition {
                    step: "Step-1".into(),
                    name: "Fixed-1".into(),
                },
                crate::Field::RotationX,
            ),
        });
        assert_eq!(model.formulation(&mesh, 0, 0), rigid);
        // A point in one step is the point of the boundary condition in all of them.
        model.exposed.clear();
        let mut second = model.steps[0].clone();
        second.name = "Step-2".into();
        second.boundary_conditions[0].kind =
            BoundaryKind::displacement([Some(0.0), Some(0.0), Some(0.0), None, None, None]);
        model.steps.push(second);
        assert_eq!(model.formulation(&mesh, 0, 0), rigid);
    }

    #[test]
    fn rotations_on_solids_are_ignored() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        model.steps[0].boundary_conditions[0].kind =
            each_node([Some(0.0), Some(0.0), Some(0.0), Some(0.01), None, None]);
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(
                ModelItem::BoundaryCondition(0, 0),
                Problem::RotationsIgnored
            )]
        );
        // Rotations held at zero, a clamp, change nothing.
        model.steps[0].boundary_conditions[0].kind = BoundaryKind::displacement([Some(0.0); 6]);
        assert_eq!(problems(&check(&model, &mesh)), []);
    }

    #[test]
    fn distorted_elements_are_attached_to_their_part() {
        let mut mesh = cubes(2, true);
        // Node 7 pushed through the cube, like a node moved by hand.
        mesh.set_node(7, [-1.0, -1.0, -1.0]);
        let model = model(&mesh);
        let findings = check(&model, &mesh);
        assert_eq!(
            problems(&findings),
            [
                (ModelItem::Part(0), Problem::DistortedElements),
                (ModelItem::Part(1), Problem::DistortedElements),
            ]
        );
        assert!(findings[0].detail.ends_with("e.g. 1"));
    }

    #[test]
    fn plane_models_rotate_about_z_only() {
        let mut mesh = FeMesh::default();
        for (id, c) in [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
            .into_iter()
            .enumerate()
        {
            mesh.set_node(id as u32 + 1, [c[0], c[1], 0.0]);
        }
        mesh.add_element(Element {
            id: 1,
            type_name: "CPS4".into(),
            shape: ElementShape::Quad4,
            nodes: vec![1, 2, 3, 4],
        })
        .unwrap();
        mesh.parts.push(Part {
            name: "PART-1".into(),
            elements: vec![1],
        });
        let mut model = model(&mesh);
        model.properties.space = ModelSpace::PlaneStress;
        model.steps[0].boundary_conditions[0].region = Region::Nodes(vec![1]);
        model.steps[0].loads[0].region = Region::Nodes(vec![3]);
        model.steps[0].loads[0].kind = LoadKind::force([1.0, 0.0, 0.0]);
        let findings = check(&model, &mesh);
        assert_eq!(findings[0].problem, Problem::RigidBodyMotion);
        assert!(
            findings[0].detail.ends_with(": rotation about Z free"),
            "{findings:?}"
        );
        // Axisymmetric: only the axial motion is a rigid body motion.
        model.properties.space = ModelSpace::Axisymmetric;
        model.steps[0].boundary_conditions[0].kind =
            BoundaryKind::displacement([None, Some(0.0), None, None, None, None]);
        assert_eq!(check(&model, &mesh), []);
    }

    #[test]
    fn a_beam_fixed_at_one_end_is_held_by_its_rotations() {
        let line = |type_name: &str| {
            let mut mesh = FeMesh::default();
            for (id, x) in [0.0, 1.0, 2.0].into_iter().enumerate() {
                mesh.set_node(id as u32 + 1, [x, 0.0, 0.0]);
            }
            for (id, nodes) in [(1, vec![1, 2]), (2, vec![2, 3])] {
                mesh.add_element(Element {
                    id,
                    type_name: type_name.into(),
                    shape: ElementShape::Line2,
                    nodes,
                })
                .unwrap();
            }
            mesh.parts.push(Part {
                name: "BEAM".into(),
                elements: vec![1, 2],
            });
            mesh
        };
        let mesh = line("B31");
        let mut model = model(&mesh);
        beam_section(&mut model);
        model.steps[0].boundary_conditions[0].region = Region::Nodes(vec![1]);
        model.steps[0].loads[0].region = Region::Nodes(vec![3]);
        assert_eq!(check(&model, &mesh), []);
        // Trusses have no rotations: the same support leaves them free to turn.
        let trusses = line("T3D2");
        model.sections[0].kind = crate::SectionKind::Truss { area: 1.0 };
        assert_eq!(check(&model, &trusses)[0].problem, Problem::RigidBodyMotion);
        // Meshed lines are B31 or B32 whatever their section; a truss section makes them
        // trusses all the same.
        assert_eq!(check(&model, &mesh)[0].problem, Problem::RigidBodyMotion);
    }

    #[test]
    fn general_beam_profiles_refuse_steps_that_user_element_u1_cannot_take() {
        let mut mesh = FeMesh::default();
        for (id, x) in [0.0, 1.0, 2.0].into_iter().enumerate() {
            mesh.set_node(id as u32 + 1, [x, 0.0, 0.0]);
        }
        for (id, nodes) in [(1, vec![1, 2]), (2, vec![2, 3])] {
            mesh.add_element(Element {
                id,
                type_name: "B31".into(),
                shape: ElementShape::Line2,
                nodes,
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "BEAM".into(),
            elements: vec![1, 2],
        });
        let mut model = model(&mesh);
        beam_section(&mut model);
        model.cross_sections[0].profile = crate::BeamProfile::I {
            h: 0.2,
            b: 0.1,
            tw: 0.006,
            tf: 0.008,
        };
        model.steps[0].boundary_conditions[0].region = Region::Nodes(vec![1]);
        model.steps[0].loads[0].region = Region::Nodes(vec![3]);
        let refused = |model: &FeModel| {
            (check(model, &mesh).into_iter())
                .filter(|f| f.problem == Problem::GeneralBeamStep)
                .map(|f| f.detail)
                .collect::<Vec<_>>()
        };
        // A linear static step is what U1 computes.
        assert_eq!(refused(&model), Vec::<String>::new());
        let StepKind::Static(settings) = &mut model.steps[0].kind else {
            unreachable!()
        };
        settings.nlgeom = true;
        assert_eq!(
            refused(&model),
            ["Step-1 (Nlgeom): Section-1 use general beam profiles"]
        );
        model.steps[0].kind = StepKind::Buckle(Default::default());
        assert_eq!(refused(&model).len(), 1);
        model.steps[0].kind = StepKind::Frequency(Default::default());
        assert!(refused(&model).is_empty());
        // Native profiles are CalculiX's own beams, which take every step.
        model.cross_sections[0].profile = crate::BeamProfile::ALL[0].clone();
        model.steps[0].kind = StepKind::Buckle(Default::default());
        assert!(refused(&model).is_empty());
    }

    #[test]
    fn beam_nodes_in_a_rigid_remote_displacement_are_warned_of_with_nlgeom() {
        let mut mesh = FeMesh::default();
        for (id, x) in [0.0, 1.0, 2.0].into_iter().enumerate() {
            mesh.set_node(id as u32 + 1, [x, 0.0, 0.0]);
        }
        for (id, nodes) in [(1, vec![1, 2]), (2, vec![2, 3])] {
            mesh.add_element(Element {
                id,
                type_name: "B31".into(),
                shape: ElementShape::Line2,
                nodes,
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "BEAM".into(),
            elements: vec![1, 2],
        });
        let mut model = model(&mesh);
        beam_section(&mut model);
        let bc = &mut model.steps[0].boundary_conditions[0];
        bc.region = Region::Nodes(vec![1]);
        bc.kind = BoundaryKind::Displacement(crate::Displacement::rigid([Some(0.0); 6]));
        model.steps[0].loads[0].region = Region::Nodes(vec![3]);
        // Linear: the equations replacing the rigid body are exact.
        let rigid = |found: &[Finding]| {
            (found.iter())
                .filter(|f| f.problem == Problem::RigidBeamNodes)
                .map(|f| (f.item, f.detail.clone()))
                .collect::<Vec<_>>()
        };
        assert_eq!(rigid(&check(&model, &mesh)), []);
        let StepKind::Static(settings) = &mut model.steps[0].kind else {
            unreachable!()
        };
        settings.nlgeom = true;
        assert_eq!(
            rigid(&check(&model, &mesh)),
            [(
                ModelItem::BoundaryCondition(0, 0),
                "Fixed-1: 1 beam or shell nodes coupled by equations, Step-1 with large \
                 deformations"
                    .to_string()
            )]
        );
    }

    /// CalculiX holds all rotations of a beam node as soon as one is given, so a beam node
    /// with some rotations left free is not what it seems.
    #[test]
    fn beam_nodes_take_all_rotations_or_none() {
        let mut mesh = FeMesh::default();
        for (id, x) in [0.0, 1.0, 2.0].into_iter().enumerate() {
            mesh.set_node(id as u32 + 1, [x, 0.0, 0.0]);
        }
        for (id, nodes) in [(1, vec![1, 2]), (2, vec![2, 3])] {
            mesh.add_element(Element {
                id,
                type_name: "B31".into(),
                shape: ElementShape::Line2,
                nodes,
            })
            .unwrap();
        }
        mesh.parts.push(Part {
            name: "BEAM".into(),
            elements: vec![1, 2],
        });
        let mut model = model(&mesh);
        beam_section(&mut model);
        model.steps[0].loads[0].region = Region::Nodes(vec![2]);
        let bcs = &mut model.steps[0].boundary_conditions;
        bcs[0].region = Region::Nodes(vec![1]);
        bcs.push(BoundaryCondition {
            distribution: None,
            name: "Pin".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![3]),
            kind: BoundaryKind::displacement([Some(0.0), Some(0.0), Some(0.0), None, None, None]),
            amplitude: None,
        });
        // A clamp and a pin.
        assert_eq!(check(&model, &mesh), []);
        // The pin also stops the twist: CalculiX makes a clamp of it.
        model.steps[0].boundary_conditions[1].kind =
            BoundaryKind::displacement([Some(0.0), Some(0.0), Some(0.0), Some(0.0), None, None]);
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(
                ModelItem::BoundaryCondition(0, 1),
                Problem::PartialBeamRotations
            )]
        );
        // The missing rotations given by another boundary condition make a clamp.
        model.steps[0].boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Rotations".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![3]),
            kind: BoundaryKind::displacement([None, None, None, None, Some(0.0), Some(0.0)]),
            amplitude: None,
        });
        assert_eq!(check(&model, &mesh), []);
        // Two pins leave the beam free to turn about the line through them, which CalculiX
        // copes with.
        model.steps[0].boundary_conditions.truncate(2);
        model.steps[0].boundary_conditions[0].kind =
            BoundaryKind::displacement([Some(0.0), Some(0.0), Some(0.0), None, None, None]);
        model.steps[0].boundary_conditions[1].kind =
            BoundaryKind::displacement([Some(0.0), Some(0.0), Some(0.0), None, None, None]);
        let findings = check(&model, &mesh);
        assert_eq!(
            problems(&findings),
            [
                (ModelItem::BoundaryConditions(0), Problem::SpinAboutPins),
                (ModelItem::Part(0), Problem::SpinAboutPins),
            ]
        );
        assert_eq!(findings[0].severity(), Severity::Warning);
        // A single pin leaves more free: that stays an error.
        model.steps[0].boundary_conditions.truncate(1);
        assert_eq!(check(&model, &mesh)[0].problem, Problem::RigidBodyMotion);
    }

    /// A plane truss triangle in the XZ plane, as meshed lines (`B31`) with a truss section:
    /// nodes 1 (0, 0, 0), 2 (2, 0, 0) and 3 (1, 0, 1), the bottom bar split into
    /// `bottom` elements, held statically determinate in the plane and at every node in Y.
    /// With `tied` the left bar ends in node 9 at node 3 instead, joined by a node tie.
    fn truss_triangle(bottom: u32, tied: bool) -> (FeMesh, FeModel) {
        let mut mesh = FeMesh::default();
        mesh.set_node(1, [0.0, 0.0, 0.0]);
        mesh.set_node(2, [2.0, 0.0, 0.0]);
        mesh.set_node(3, [1.0, 0.0, 1.0]);
        let mut bottom_nodes = vec![1];
        for i in 1..bottom {
            mesh.set_node(3 + i, [2.0 * f64::from(i) / f64::from(bottom), 0.0, 0.0]);
            bottom_nodes.push(3 + i);
        }
        bottom_nodes.push(2);
        let mut bars: Vec<Vec<Vec<NodeId>>> =
            vec![(bottom_nodes.windows(2)).map(<[NodeId]>::to_vec).collect()];
        bars.push(vec![vec![2, 3]]);
        let top = if tied { 9 } else { 3 };
        if tied {
            mesh.set_node(9, [1.0, 0.0, 1.0]);
        }
        bars.push(vec![vec![top, 1]]);
        let mut id = 1;
        for (b, elements) in bars.into_iter().enumerate() {
            let mut part = Vec::new();
            for nodes in elements {
                mesh.add_element(Element {
                    id,
                    type_name: "B31".into(),
                    shape: ElementShape::Line2,
                    nodes,
                })
                .unwrap();
                part.push(id);
                id += 1;
            }
            mesh.parts.push(Part {
                name: format!("LINE-{}", b + 1),
                elements: part,
            });
        }
        let mut model = model(&mesh);
        model.sections[0].kind = crate::SectionKind::Truss { area: 1.0 };
        let bcs = &mut model.steps[0].boundary_conditions;
        bcs[0].region = Region::Nodes(vec![1]);
        bcs[0].kind = BoundaryKind::displacement([Some(0.0), None, Some(0.0), None, None, None]);
        bcs.push(BoundaryCondition {
            distribution: None,
            name: "Roller".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![2]),
            kind: BoundaryKind::displacement([None, None, Some(0.0), None, None, None]),
            amplitude: None,
        });
        bcs.push(BoundaryCondition {
            distribution: None,
            name: "Plane".into(),
            active: true,
            visible: true,
            region: Region::Nodes(mesh.node_ids().to_vec()),
            kind: BoundaryKind::displacement([None, Some(0.0), None, None, None, None]),
            amplitude: None,
        });
        model.steps[0].loads[0].region = Region::Nodes(vec![3]);
        if tied {
            model.node_ties.push(crate::NodeTie {
                name: "Node_Tie-1".into(),
                active: true,
                region: Region::Nodes(vec![3, 9]),
                rotations: true,
            });
        }
        (mesh, model)
    }

    #[test]
    fn truss_nodes_need_bars_or_supports_in_every_direction() {
        let (mesh, model) = truss_triangle(1, false);
        assert_eq!(check(&model, &mesh), []);
        // A bar of two elements buckles at its inner node without resistance.
        let (mesh, model) = truss_triangle(2, false);
        let findings = check(&model, &mesh);
        assert_eq!(
            problems(&findings),
            [
                (ModelItem::BoundaryConditions(0), Problem::Mechanism),
                (ModelItem::Part(0), Problem::Mechanism),
            ]
        );
        assert_eq!(
            findings[0].detail,
            "Step-1: 1 truss nodes not held, e.g. node 4 free in Z"
        );
        // Without the supports in Y the plane truss moves out of its plane.
        let (mesh, mut model) = truss_triangle(1, false);
        model.steps[0].boundary_conditions[2].region = Region::Nodes(vec![1, 2]);
        let findings = check(&model, &mesh);
        assert!(findings.iter().any(|f| f.problem == Problem::Mechanism
            && f.detail == "Step-1: 1 truss nodes not held, e.g. node 3 free in Y"));
        // Tied nodes count as one node, as they are written.
        let (mesh, model) = truss_triangle(1, true);
        assert_eq!(check(&model, &mesh), []);
    }

    #[test]
    fn heat_transfer_needs_thermal_constants() {
        let mesh = cubes(1, false);
        let mut model = model(&mesh);
        let mut step = Step::new_heat_transfer("Step-1");
        step.boundary_conditions.push(BoundaryCondition {
            distribution: None,
            name: "Temperature-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![1]),
            kind: BoundaryKind::Temperature(20.0),
            amplitude: None,
        });
        step.loads.push(Load {
            distribution: None,
            name: "Flux-1".into(),
            active: true,
            visible: true,
            region: Region::Nodes(vec![7]),
            kind: LoadKind::ConcentratedFlux(10.0),
            amplitude: None,
            factor_amplitude: None,
        });
        model.steps = vec![step];
        // Displacements are free in a heat transfer, and no elasticity is needed.
        model.materials[0].elastic = None;
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Material(0), Problem::NoConductivity)]
        );
        model.materials[0].conductivity = Some(50.0);
        assert_eq!(check(&model, &mesh), []);
        // Radiation needs the physical constants.
        model.steps[0].loads[0].kind = LoadKind::Radiation {
            sink: 20.0,
            emissivity: 0.8,
        };
        assert_eq!(
            problems(&check(&model, &mesh)),
            [(ModelItem::Load(0, 0), Problem::NoPhysicalConstants)]
        );
        model.properties.absolute_zero = Some(-273.15);
        model.properties.stefan_boltzmann = Some(5.67e-11);
        assert_eq!(check(&model, &mesh), []);
        model.steps[0].loads[0].kind = LoadKind::ConcentratedFlux(10.0);
        if let StepKind::HeatTransfer(settings) = &mut model.steps[0].kind {
            settings.steady_state = false;
        }
        assert_eq!(
            problems(&check(&model, &mesh)),
            [
                (ModelItem::Material(0), Problem::NoSpecificHeat),
                (ModelItem::Step(0), Problem::NoInitialTemperature),
            ]
        );
    }

    #[test]
    fn solver_errors_are_explained() {
        let mesh = cubes(2, true);
        let lines: Vec<String> = [
            " *ERROR in e_c3d: nonpositive jacobian",
            "        determinant in element           2",
            "",
            " *ERROR: too many cutbacks",
        ]
        .map(String::from)
        .to_vec();
        let findings = diagnose_solver_output(&lines, &mesh);
        assert_eq!(
            problems(&findings),
            [
                (ModelItem::Analysis, Problem::DistortedElements),
                (ModelItem::Analysis, Problem::NoConvergence),
                (ModelItem::Part(1), Problem::DistortedElements),
            ]
        );
        let unknown = [" *ERROR in something: new message"].map(String::from);
        let findings = diagnose_solver_output(&unknown, &mesh);
        assert_eq!(findings.len(), 1);
        assert_eq!(findings[0].problem, Problem::SolverError);
        assert_eq!(findings[0].detail, "*ERROR in something: new message");
        let radiate = [
            " *ERROR reading *RADIATE: *RADIATE card was selected",
            "        but no *PHYSICAL CONSTANTS card encountered",
            "        or incomplete (Stefan Boltzmann constant",
        ]
        .map(String::from);
        let findings = diagnose_solver_output(&radiate, &mesh);
        assert_eq!(
            problems(&findings),
            [(ModelItem::Analysis, Problem::NoPhysicalConstants)]
        );
        assert_eq!(
            findings[0].detail,
            "The solver reports \"*ERROR reading *RADIATE: *RADIATE card was selected\""
        );
        let rigid = [
            " *ERROR in gen3dmpc: nodes belonging to",
            "        beam elements must not be",
            "        subject to a RIGID                MPC.",
        ]
        .map(String::from);
        assert_eq!(
            problems(&diagnose_solver_output(&rigid, &mesh)),
            [(ModelItem::Analysis, Problem::RigidBeamNodes)]
        );
        let unknown = [" *ERROR in something: new message".to_string()];
        assert_eq!(
            diagnose_solver_output(&unknown, &mesh)[0].problem,
            Problem::SolverError
        );
    }
}
