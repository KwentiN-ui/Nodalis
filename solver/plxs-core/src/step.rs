//! Steps (concept section 1.5 and 1.6): the procedure, what holds and acts during the step,
//! and what is written out. A step is self-contained: the reader resolves CalculiX's
//! carry-over rules (`OP=NEW`, loads kept from earlier steps) so that a procedure only looks
//! at its own step.

use serde::{Deserialize, Serialize};

use crate::mesh::{ElementRef, NodeRef};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Step {
    /// Name from the `** Name:` comment Nodalis writes before the step, or `Step-<n>`.
    pub name: String,
    pub procedure: Procedure,
    /// `*STEP, NLGEOM`: geometrically nonlinear.
    pub nlgeom: bool,
    /// `*STEP, PERTURBATION`: the step is a linear perturbation about the state of the
    /// previous step (frequency/buckle/complex frequency), reset afterwards.
    pub perturbation: bool,
    /// `*STEP, INC=`: maximum number of increments (CalculiX default 100).
    pub max_increments: u32,
    /// `*STEP, AMPLITUDE=RAMP|STEP`: how loads without an amplitude vary over the step.
    pub default_amplitude: DefaultAmplitude,
    /// Single-point constraints in force during the step (already including those of
    /// earlier steps unless the step started with `OP=NEW`).
    pub boundaries: Vec<Boundary>,
    pub loads: Loads,
    /// Temperature fields (`*TEMPERATURE`), in force during the step.
    pub fields: Vec<DefinedField>,
    /// `*MODEL CHANGE` and `*CHANGE FRICTION` at the start of the step.
    pub contact_changes: Vec<ContactChange>,
    pub outputs: Outputs,
    /// `*RESTART, WRITE` in this step.
    pub restart_write: Option<RestartWrite>,
    /// `*CONTROLS`, not written by Nodalis; `None` means CalculiX's defaults.
    pub controls: Option<Controls>,
}

impl Step {
    pub fn new(name: impl Into<String>, procedure: Procedure) -> Self {
        Self {
            name: name.into(),
            procedure,
            nlgeom: false,
            perturbation: false,
            max_increments: 100,
            default_amplitude: DefaultAmplitude::Ramp,
            boundaries: Vec::new(),
            loads: Loads::default(),
            fields: Vec::new(),
            contact_changes: Vec::new(),
            outputs: Outputs::default(),
            restart_write: None,
            controls: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DefaultAmplitude {
    Ramp,
    Step,
}

/// The procedure of a step with its parameters (concept section 1.6).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Procedure {
    /// `*NO ANALYSIS`: read the input, compute nothing (the model check).
    NoAnalysis,
    Static(Static),
    /// `*VISCO`: static with creep, time integration controlled by `cetol`.
    Visco(Visco),
    Frequency(Frequency),
    ComplexFrequency(ComplexFrequency),
    Buckle(Buckle),
    HeatTransfer(HeatTransfer),
    CoupledTemperatureDisplacement(HeatTransfer),
    UncoupledTemperatureDisplacement(HeatTransfer),
    Dynamic(Dynamic),
    ModalDynamics(ModalDynamics),
    SteadyStateDynamics(SteadyStateDynamics),
    /// `*SENSITIVITY`: derivatives of design responses with respect to the design node
    /// positions (`plxs-procedures::sensitivity`, T17).
    Sensitivity(Sensitivity),
}

impl Procedure {
    /// Whether the step solves an eigenvalue problem: its results are the modes
    /// ([`crate::results::ModeSet`]).
    pub fn computes_eigenmodes(&self) -> bool {
        matches!(
            self,
            Procedure::Frequency(_) | Procedure::ComplexFrequency(_) | Procedure::Buckle(_)
        )
    }

    /// The keyword of the procedure as written in the input.
    pub fn keyword(&self) -> &'static str {
        match self {
            Procedure::NoAnalysis => "*NO ANALYSIS",
            Procedure::Static(_) => "*STATIC",
            Procedure::Visco(_) => "*VISCO",
            Procedure::Frequency(_) => "*FREQUENCY",
            Procedure::ComplexFrequency(_) => "*COMPLEX FREQUENCY",
            Procedure::Buckle(_) => "*BUCKLE",
            Procedure::HeatTransfer(_) => "*HEAT TRANSFER",
            Procedure::CoupledTemperatureDisplacement(_) => "*COUPLED TEMPERATURE-DISPLACEMENT",
            Procedure::UncoupledTemperatureDisplacement(_) => "*UNCOUPLED TEMPERATURE-DISPLACEMENT",
            Procedure::Dynamic(_) => "*DYNAMIC",
            Procedure::ModalDynamics(_) => "*MODAL DYNAMICS",
            Procedure::SteadyStateDynamics(_) => "*STEADY STATE DYNAMICS",
            Procedure::Sensitivity(_) => "*SENSITIVITY",
        }
    }
}

/// The equation solver CalculiX is asked for (`SOLVER=`); the own solver maps the names to
/// its automatic choice, its direct and its iterative solver, see [`Self::mode`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum EquationSolver {
    /// No `SOLVER=`: CalculiX takes its default; the own solver chooses automatically.
    #[default]
    Default,
    Spooles,
    Pastix,
    Pardiso,
    IterativeScaling,
    IterativeCholesky,
}

impl EquationSolver {
    /// Whether the step asks for an iterative solver.
    pub fn is_iterative(self) -> bool {
        self.mode() == SolverMode::Iterative
    }

    /// How the own solver solves the step's systems: no `SOLVER=` chooses automatically,
    /// the iterative names mean conjugate gradients (`plxs_linalg::PcgSolver`), every
    /// other name the direct solver.
    pub fn mode(self) -> SolverMode {
        match self {
            EquationSolver::Default => SolverMode::Automatic,
            EquationSolver::IterativeScaling | EquationSolver::IterativeCholesky => {
                SolverMode::Iterative
            }
            EquationSolver::Spooles | EquationSolver::Pastix | EquationSolver::Pardiso => {
                SolverMode::Direct
            }
        }
    }
}

/// The own solver's reading of [`EquationSolver`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolverMode {
    /// Direct while the factor fits into the memory, iterative beyond (and back to direct
    /// when the iteration fails), where the procedure has an iterative solver.
    Automatic,
    Direct,
    Iterative,
}

/// Time incrementation of a step: initial increment, step period, minimum and maximum
/// increment (`*STATIC` data line and the like). `None` means CalculiX's defaults: initial =
/// period = 1, minimum = initial * 1e-5, maximum = 1e30 (or the period).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Incrementation {
    pub initial: Option<f64>,
    pub period: Option<f64>,
    pub minimum: Option<f64>,
    pub maximum: Option<f64>,
    /// `DIRECT`: fixed increments, no automatic control.
    pub direct: bool,
}

impl Incrementation {
    pub fn initial(&self) -> f64 {
        self.initial.unwrap_or(1.0)
    }

    pub fn period(&self) -> f64 {
        self.period.unwrap_or(1.0)
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Static {
    pub solver: EquationSolver,
    pub increments: Incrementation,
    /// `EXPLICIT`, `TIME RESET`, `TOTAL TIME AT START` are not written by Nodalis; kept
    /// for completeness.
    pub time_reset: bool,
    pub total_time_at_start: Option<f64>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Visco {
    pub solver: EquationSolver,
    pub increments: Incrementation,
    /// `CETOL`: creep strain tolerance that controls the time step.
    pub cetol: f64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Frequency {
    pub solver: EquationSolver,
    /// `STORAGE=YES`: write the modes to `<job>.eig` for later modal steps.
    pub storage: bool,
    /// `GLOBAL=NO`: modes in local systems; not written by Nodalis.
    pub global: bool,
    /// `CYCMPC=INACTIVE`, not written by Nodalis.
    pub cycmpc_active: bool,
    pub count: u32,
    pub lower: Option<f64>,
    pub upper: Option<f64>,
    /// `*SELECT CYCLIC SYMMETRY MODES` in the step: the nodal diameters to compute.
    pub cyclic_modes: Option<CyclicModes>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CyclicModes {
    pub nmin: Option<u32>,
    pub nmax: Option<u32>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ComplexFrequency {
    pub coriolis: bool,
    pub fluid: bool,
    pub count: u32,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Buckle {
    pub solver: EquationSolver,
    pub count: u32,
    pub accuracy: Option<f64>,
}

/// `*HEAT TRANSFER` and the coupled procedures.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct HeatTransfer {
    pub solver: EquationSolver,
    pub steady_state: bool,
    pub increments: Incrementation,
    /// `DELTMX`: maximum temperature change per increment.
    pub deltmx: Option<f64>,
    /// `FREQUENCY=`: heat transfer modes (eigenvalues), not written by Nodalis.
    pub frequency_count: Option<u32>,
    /// `COMPRESSIBLE`, `SHOCK SMOOTHING`, `TURBULENCE MODEL`: fluid parameters Nodalis does
    /// not use; rejected by the reader.
    pub direct: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Dynamic {
    pub solver: EquationSolver,
    /// `ALPHA=`: numerical damping of the HHT-alpha integration (`-1/3..0`).
    pub alpha: Option<f64>,
    pub explicit: bool,
    pub increments: Incrementation,
    /// `*DAMPING, ALPHA=, BETA=` in the step: Rayleigh damping.
    pub rayleigh: Option<Rayleigh>,
    /// `*DAMPING, RATIO=, LOWER=, UPPER=` (Nodalis): a viscous damping ratio, applied as
    /// the Rayleigh damping that meets it at the lower and upper frequency. Without a lower
    /// frequency the solver takes the lowest eigenfrequency of the model.
    #[serde(default)]
    pub damping_ratio: Option<DampingRatio>,
    /// `EVENTS` (Nodalis): besides the increments, a result frame at every sub-increment
    /// at which a contact closes or opens and at the sub-increments an impact is split
    /// into, so the results show the contact events between the frames.
    #[serde(default)]
    pub event_frames: bool,
}

/// A damping ratio fitted as Rayleigh damping between two frequencies (cycles per time).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DampingRatio {
    pub ratio: f64,
    pub lower: Option<f64>,
    pub upper: f64,
}

impl Rayleigh {
    /// The coefficients that meet the damping ratio `zeta` exactly at `f1` and `f2`:
    /// `alpha = 2 zeta w1 w2 / (w1 + w2)`, `beta = 2 zeta / (w1 + w2)`.
    pub fn from_ratio(zeta: f64, f1: f64, f2: f64) -> Self {
        let (w1, w2) = (std::f64::consts::TAU * f1, std::f64::consts::TAU * f2);
        Self {
            alpha: 2.0 * zeta * w1 * w2 / (w1 + w2),
            beta: 2.0 * zeta / (w1 + w2),
        }
    }

    /// The damping ratio at the frequency `f`.
    pub fn ratio_at(&self, f: f64) -> f64 {
        let w = std::f64::consts::TAU * f;
        self.alpha / (2.0 * w) + self.beta * w / 2.0
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Rayleigh {
    pub alpha: f64,
    pub beta: f64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ModalDynamics {
    pub solver: EquationSolver,
    pub steady_state: bool,
    /// Time increment and period (steady state: relative error and maximum time instead).
    pub increment: f64,
    pub period: f64,
    pub damping: Option<ModalDamping>,
}

/// `*MODAL DAMPING`: constant viscous damping ratio (all modes), per mode ranges, or
/// Rayleigh.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ModalDamping {
    /// Rows of (first mode, last mode, ratio).
    Direct(Vec<(u32, u32, f64)>),
    Rayleigh {
        alpha: f64,
        beta: f64,
    },
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SteadyStateDynamics {
    pub solver: EquationSolver,
    pub harmonic: bool,
    pub lower: f64,
    pub upper: f64,
    pub points: u32,
    pub bias: f64,
    /// Fourier terms for non-harmonic loading.
    pub fourier_terms: Option<u32>,
    pub damping: Option<ModalDamping>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Sensitivity {
    pub nlgeom: bool,
    pub responses: Vec<DesignResponse>,
    pub filter: Option<Filter>,
    /// `*OBJECTIVE`, written by older Nodalis versions; names of the responses.
    pub objectives: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DesignResponse {
    pub name: String,
    /// `EIGENFREQUENCY`, `MASS`, `STRAINENERGY`, `STRESS`, `MISESSTRESS`, ...
    pub kind: String,
    pub elset: Option<String>,
    /// Further data (reference stress for `STRESS` and the like).
    pub data: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Filter {
    pub kind: String,
    pub radius: f64,
}

/// `*BOUNDARY`: a single-point constraint on a range of degrees of freedom (1-3
/// translations, 4-6 rotations, 11 temperature, 8 pressure in CalculiX) of a node or set.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Boundary {
    pub region: NodeRef,
    pub first_dof: u8,
    pub last_dof: u8,
    /// Prescribed value; `None` means zero (homogeneous).
    pub value: Option<f64>,
    pub amplitude: Option<String>,
    /// `FIXED`: the node keeps the value it had at the end of the previous step.
    pub fixed: bool,
    /// `SUBMODEL, STEP=<n>`: the value comes from the global model's results of step n.
    pub submodel_step: Option<u32>,
    /// `LOAD CASE`, steady state dynamics: 1 real, 2 imaginary part.
    pub load_case: Option<u8>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Loads {
    pub concentrated: Vec<ConcentratedLoad>,
    pub distributed: Vec<DistributedLoad>,
    pub concentrated_flux: Vec<ConcentratedFlux>,
    pub distributed_flux: Vec<DistributedFlux>,
    pub films: Vec<Film>,
    pub radiations: Vec<Radiate>,
}

/// `*CLOAD`: a force (DOF 1-3) or moment (4-6) on a node or set.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConcentratedLoad {
    pub region: NodeRef,
    pub dof: u8,
    pub value: f64,
    pub amplitude: Option<String>,
    pub load_case: Option<u8>,
    /// `SECTOR=` of cyclic symmetry models.
    pub sector: Option<u32>,
}

/// `*DLOAD`: a distributed load on elements or faces.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DistributedLoad {
    pub region: ElementRef,
    pub kind: DistributedLoadKind,
    pub amplitude: Option<String>,
    pub load_case: Option<u8>,
    pub sector: Option<u32>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum DistributedLoadKind {
    /// `P<n>`: pressure on face n (positive into the element).
    Pressure { face: u8, value: f64 },
    /// `P<n>NU`, non-uniform pressure through a user routine, not supported.
    /// `GRAV`: acceleration magnitude and direction.
    Gravity { magnitude: f64, direction: [f64; 3] },
    /// `CENTRIF`: angular velocity squared, a point on the axis and the axis direction.
    Centrifugal {
        omega_squared: f64,
        point: [f64; 3],
        axis: [f64; 3],
    },
    /// `NEWTON`, `BX`/`BY`/`BZ` and other labels Nodalis does not write.
    Other { label: String, values: Vec<f64> },
}

/// `*CFLUX`: concentrated heat flux into a node (DOF 11).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConcentratedFlux {
    pub region: NodeRef,
    pub value: f64,
    pub amplitude: Option<String>,
}

/// `*DFLUX`: heat flux on a face (`S<n>`) or per volume (`BF`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DistributedFlux {
    pub region: ElementRef,
    pub kind: FluxKind,
    pub value: f64,
    pub amplitude: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FluxKind {
    Face(u8),
    Body,
}

/// `*FILM`: convection on a face, flux = coefficient * (sink - surface temperature).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Film {
    pub region: ElementRef,
    pub face: u8,
    pub sink_temperature: f64,
    pub coefficient: f64,
    /// Amplitude of the sink temperature.
    pub amplitude: Option<String>,
    /// `FILM AMPLITUDE`: amplitude of the coefficient.
    pub film_amplitude: Option<String>,
}

/// `*RADIATE`: radiation from a face to the environment (`R<n>`) or a cavity (`R<n>CR`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Radiate {
    pub region: ElementRef,
    pub face: u8,
    pub cavity: bool,
    pub sink_temperature: f64,
    pub emissivity: f64,
    pub amplitude: Option<String>,
    /// `RADIATION AMPLITUDE`: amplitude of the emissivity.
    pub radiation_amplitude: Option<String>,
}

/// `*TEMPERATURE`: temperatures of nodes as a load (thermal strain) in mechanical steps.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DefinedField {
    pub kind: DefinedFieldKind,
    pub amplitude: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum DefinedFieldKind {
    /// Values per node or set.
    Temperature(Vec<(NodeRef, f64)>),
    /// `FILE=<x>.frd, BSTEP=, ESTEP=`: temperatures from a result file.
    TemperatureFile {
        file: String,
        bstep: Option<u32>,
        estep: Option<u32>,
        binc: Option<u32>,
        einc: Option<u32>,
    },
}

/// `*MODEL CHANGE` and `*CHANGE FRICTION` at the start of a step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ContactChange {
    /// `*MODEL CHANGE, TYPE=CONTACT PAIR, ADD|REMOVE` with the slave and master surface.
    ContactPair {
        add: bool,
        slave: String,
        master: String,
    },
    /// `*MODEL CHANGE, TYPE=ELEMENT, ADD|REMOVE`, not written by Nodalis.
    Elements { add: bool, elset: String },
    /// `*CHANGE FRICTION, INTERACTION=` with the new `*FRICTION` values.
    Friction {
        interaction: String,
        friction: crate::contact::Friction,
    },
}

/// Field (`.frd`) and history (`.dat`) outputs of a step.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Outputs {
    pub node_file: Vec<FieldOutput>,
    pub el_file: Vec<FieldOutput>,
    pub contact_file: Vec<FieldOutput>,
    pub node_print: Vec<HistoryOutput>,
    pub el_print: Vec<HistoryOutput>,
    pub contact_print: Vec<HistoryOutput>,
}

/// `*NODE FILE`, `*EL FILE`, `*CONTACT FILE`: variables (`U`, `S`, `NOE`...) with their
/// options.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FieldOutput {
    pub variables: Vec<String>,
    /// `FREQUENCY=`: every n-th increment (0 or 1: every one).
    pub frequency: u32,
    /// `FREQUENCYF=` of fluids, ignored.
    pub nset: Option<String>,
    pub last_iterations: bool,
    pub contact_elements: bool,
    /// `OUTPUT=2D|3D`: whether 2D, shell and beam results are written on the original
    /// elements or on the expanded solids (CalculiX's default).
    pub output: OutputDimension,
    /// `GLOBAL=NO`: results in local systems.
    pub global: bool,
    /// `SECTION FORCES`: shell results as section forces rather than stresses.
    pub section_forces: bool,
    /// `TIME POINTS=` of `*TIME POINTS`, not written by Nodalis.
    pub time_points: Option<String>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum OutputDimension {
    #[default]
    Expanded,
    TwoD,
    ThreeD,
}

/// `*NODE PRINT`, `*EL PRINT`, `*CONTACT PRINT`: variables on a set with `TOTALS=`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HistoryOutput {
    /// Node set, element set, or `None` for contact print.
    pub set: Option<String>,
    pub variables: Vec<String>,
    pub frequency: u32,
    pub totals: Totals,
    pub global: bool,
    pub time_points: Option<String>,
    /// `*CONTACT PRINT, SLAVE=, MASTER=`: the slave and master surface of the pair the
    /// statistics (`CF`) are printed for (upper case); `None` for all pairs.
    #[serde(default)]
    pub surfaces: Option<(String, String)>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Totals {
    #[default]
    No,
    Yes,
    Only,
}

/// `*RESTART, WRITE`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RestartWrite {
    pub frequency: u32,
    pub overlay: bool,
}

/// `*CONTROLS`: convergence parameters. Not written by Nodalis; kept so that an input with
/// them is not rejected later.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Controls {
    pub parameters: String,
    pub values: Vec<f64>,
}
