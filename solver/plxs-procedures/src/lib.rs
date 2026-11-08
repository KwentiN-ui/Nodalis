//! Procedures of the Nodalis solver: what a step computes. M1: linear static analysis
//! (`*STATIC` without `NLGEOM`). M2: heat transfer and temperature-displacement steps
//! ([`heat`], [`thermal`]), nonlinear static (`NLGEOM`, plastic and creeping materials,
//! [`nonlinear`]) and `*VISCO`, `*FREQUENCY` and `*BUCKLE` ([`eigen`]), `*DYNAMIC`
//! ([`dynamic`]; with `NLGEOM`, nonlinear materials or contact [`nonlinear_dynamic`]), modal dynamics, steady state dynamics and complex frequency ([`modal`],
//! [`complex`]), `*SENSITIVITY` ([`sensitivity`], T17). The remaining procedures follow in
//! M3 (concept section 4.5).
//!
//! A procedure works on a [`ProcedureContext`]: the model, its element table, the assembled
//! stiffness matrix and solver, the state it starts from, and the results it appends to.
//! Progress goes to a [`Progress`] sink as events; the command line prints them in
//! CalculiX's wording.
//!
//! No CalculiX code was consulted.

pub mod complex;
pub mod contact;
pub mod cyclic;
pub mod dynamic;
pub mod eigen;
pub mod equations;
pub mod explicit_dynamic;
pub mod fields;
pub mod frames;
pub mod heat;
pub mod history;
pub mod hybrid_dynamic;
pub mod modal;
pub mod nonlinear;
pub mod nonlinear_dynamic;
pub mod profile;
pub mod sensitivity;
pub mod static_linear;
pub mod submodel;
pub mod thermal;

use std::sync::Arc;

use plxs_assembly::ElementTable;
use plxs_core::step::{Integration, Procedure as ProcedureKind};
use plxs_core::{Error, Model, Progress, Result, Results, State, Step};
use plxs_linalg::{LdltSolver, Pattern, SparseMatrix};
use plxs_output::{CvgRow, StaRow};

/// How a step ended.
#[derive(Clone, Debug, PartialEq)]
pub enum StepOutcome {
    Converged,
    Diverged {
        increment: u32,
        /// What failed, for Nodalis to explain it.
        failure: plxs_core::Failure,
        message: String,
    },
    Partial {
        increments: u32,
    },
    /// The caller cancelled the run; the state is that of the last converged increment.
    Cancelled,
}

/// Everything a procedure works on.
pub struct ProcedureContext<'a> {
    pub model: &'a Model,
    pub table: &'a ElementTable,
    /// The model's multi-point constraints and local systems, prepared once.
    pub constraints: &'a plxs_constraints::ModelConstraints,
    pub pattern: Arc<Pattern>,
    /// The global stiffness matrix of the last assembly, `None` before the first.
    pub stiffness: Option<SparseMatrix>,
    pub solver: LdltSolver,
    pub state: &'a mut State,
    pub results: &'a mut Results,
    pub threads: usize,
    /// 1-based number of the step being computed.
    pub step_number: u32,
    /// Rows of the `.sta` file written so far.
    pub sta: Vec<StaRow>,
    /// Rows of the `.cvg` file written so far (one per Newton iteration).
    pub cvg: Vec<CvgRow>,
    /// CalculiX's time-average force of the previous steps (the scale of the residual
    /// check), `None` before the first nonlinear step.
    pub average_force: Option<f64>,
    /// The job's directory: files the input refers to (`*TEMPERATURE, FILE=`) are read
    /// from here.
    pub work_dir: std::path::PathBuf,
    /// Set by the caller to stop the run between increments.
    pub cancel: plxs_core::CancelToken,
    /// The contact pairs of the model, prepared once (empty without contact).
    pub contact: plxs_contact::ContactModel,
    /// The matrix pattern with the contact springs of the last increment, reused while it
    /// holds the springs of the next (see [`contact::cached_pattern`]); kept over the steps
    /// of a session.
    pub contact_pattern: Option<Arc<Pattern>>,
    /// Time per phase, summed over the steps of the session.
    pub profile: profile::Profile,
    /// A part of a dynamic step that switches between implicit and explicit integration
    /// (`INTEGRATION=AUTOMATIC`); `None` computes the whole step.
    pub segment: Option<Segment>,
}

/// A part of a dynamic step, computed by one of its integrators on the state the part
/// before it ended with ([`hybrid_dynamic`]).
#[derive(Clone, Debug, Default)]
pub struct Segment {
    /// Step time the part starts at; the state holds the motion at this time.
    pub start: f64,
    /// Step time the part ends at.
    pub end: f64,
    /// The first part of the step: the changes the step makes to the contact
    /// (`*MODEL CHANGE`) and the initial conditions apply.
    pub first: bool,
    /// The displacements at the start of the step (prescribed displacements count from
    /// them).
    pub previous: Vec<f64>,
    /// Implicit parts: a sub-increment shorter than this stops the part (0 never stops).
    pub shortest: f64,
    /// The result frames of the step written before the part; the part's own are added.
    pub record: u32,
    /// The work of the external forces and the dissipated energy before the part; the
    /// part's own are added.
    pub external_work: f64,
    pub dissipation: f64,
    /// Out: the step time the part reached (`end` unless it stopped).
    pub reached: f64,
    /// Out: why an implicit part stopped before its end.
    pub stopped: Option<String>,
    /// Out: the lowest eigenfrequency a damping ratio was fitted to, which the next parts
    /// take instead of computing it again.
    pub lowest: Option<f64>,
    /// Out: the damping an implicit part fitted to a model with rigid body or mechanism
    /// modes (stiffness-proportional only), which the next parts take as it is.
    pub rigid_damping: Option<plxs_core::step::Rayleigh>,
}

/// A procedure computes a step on the context.
pub trait Procedure {
    fn run(
        &self,
        ctx: &mut ProcedureContext<'_>,
        step: &Step,
        progress: &mut dyn Progress,
    ) -> Result<StepOutcome>;
}

/// Runs the procedure of a step.
pub fn run_step(
    ctx: &mut ProcedureContext<'_>,
    step: &Step,
    progress: &mut dyn Progress,
) -> Result<StepOutcome> {
    // Hyperelastic materials deform with large strains: steps that deform the model take
    // NLGEOM, as in CalculiX.
    let hyperelastic_step;
    let step = if ctx.table.hyperelastic
        && !step.nlgeom
        && matches!(
            step.procedure,
            ProcedureKind::Static(_) | ProcedureKind::Visco(_) | ProcedureKind::Dynamic(_)
        ) {
        progress.event(plxs_core::progress::ProgressEvent::Info(format!(
            "step {}: NLGEOM switched on for the hyperelastic materials",
            step.name
        )));
        let mut nonlinear = step.clone();
        nonlinear.nlgeom = true;
        hyperelastic_step = nonlinear;
        &hyperelastic_step
    } else {
        step
    };
    match &step.procedure {
        ProcedureKind::NoAnalysis => Ok(StepOutcome::Converged),
        ProcedureKind::Static(_)
            if !step.nlgeom && !ctx.table.has_nonlinear_material() && ctx.contact.is_empty() =>
        {
            static_linear::StaticLinear.run(ctx, step, progress)
        }
        ProcedureKind::Static(_) | ProcedureKind::Visco(_) => {
            nonlinear::NonlinearStatic.run(ctx, step, progress)
        }
        ProcedureKind::Frequency(_)
        | ProcedureKind::Buckle(_)
        | ProcedureKind::Sensitivity(_)
        | ProcedureKind::ComplexFrequency(_)
        | ProcedureKind::ModalDynamics(_)
        | ProcedureKind::SteadyStateDynamics(_)
            if !ctx.contact.is_empty() =>
        {
            Err(Error::Unsupported(format!(
                "contact in {} steps is not implemented yet (step {})",
                step.procedure.keyword(),
                step.name
            )))
        }
        ProcedureKind::Frequency(_) => eigen::FrequencyProcedure.run(ctx, step, progress),
        ProcedureKind::Buckle(_) => eigen::BuckleProcedure.run(ctx, step, progress),
        ProcedureKind::Dynamic(d) if matches!(d.integration, Integration::Explicit(_)) => {
            explicit_dynamic::ExplicitDynamic.run(ctx, step, progress)
        }
        ProcedureKind::Dynamic(d) if matches!(d.integration, Integration::Automatic { .. }) => {
            hybrid_dynamic::HybridDynamic.run(ctx, step, progress)
        }
        // Contact makes a dynamic step nonlinear as well: Newton iterations on the contact
        // status in every increment.
        ProcedureKind::Dynamic(_)
            if step.nlgeom || ctx.table.has_nonlinear_material() || !ctx.contact.is_empty() =>
        {
            nonlinear_dynamic::NonlinearDynamic.run(ctx, step, progress)
        }
        ProcedureKind::Dynamic(_) => dynamic::DynamicProcedure.run(ctx, step, progress),
        ProcedureKind::ComplexFrequency(_) => {
            complex::ComplexFrequencyProcedure.run(ctx, step, progress)
        }
        ProcedureKind::ModalDynamics(_) => modal::ModalDynamicsProcedure.run(ctx, step, progress),
        ProcedureKind::SteadyStateDynamics(_) => {
            modal::SteadyStateProcedure.run(ctx, step, progress)
        }
        ProcedureKind::Sensitivity(_) => sensitivity::SensitivityProcedure.run(ctx, step, progress),
        ProcedureKind::HeatTransfer(_)
        | ProcedureKind::CoupledTemperatureDisplacement(_)
        | ProcedureKind::UncoupledTemperatureDisplacement(_)
            if !step.nlgeom =>
        {
            heat::HeatTransferProcedure.run(ctx, step, progress)
        }
        other => Err(Error::Unsupported(format!(
            "procedure not implemented yet: {}{} (step {})",
            other.keyword(),
            if step.nlgeom { ", NLGEOM" } else { "" },
            step.name
        ))),
    }
}
