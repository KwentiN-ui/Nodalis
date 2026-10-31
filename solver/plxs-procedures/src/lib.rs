//! Procedures of the PrePoLix solver: what a step computes. M1: linear static analysis
//! (`*STATIC` without `NLGEOM`). Nonlinear, eigenvalue, dynamic and thermal procedures
//! follow in M2/M3 (concept section 4.5).
//!
//! A procedure works on a [`ProcedureContext`]: the model, its element table, the assembled
//! stiffness matrix and solver, the state it starts from, and the results it appends to.
//! Progress goes to a [`Progress`] sink as events; the command line prints them in
//! CalculiX's wording.
//!
//! No CalculiX code was consulted.

pub mod fields;
pub mod history;
pub mod static_linear;

use std::sync::Arc;

use plxs_assembly::ElementTable;
use plxs_core::step::Procedure as ProcedureKind;
use plxs_core::{Error, Model, Progress, Result, Results, State, Step};
use plxs_linalg::{LdltSolver, Pattern, SparseMatrix};
use plxs_output::StaRow;

/// How a step ended.
#[derive(Clone, Debug, PartialEq)]
pub enum StepOutcome {
    Converged,
    Diverged { increment: u32, message: String },
    Partial { increments: u32 },
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
    match &step.procedure {
        ProcedureKind::NoAnalysis => Ok(StepOutcome::Converged),
        ProcedureKind::Static(_) if !step.nlgeom => {
            static_linear::StaticLinear.run(ctx, step, progress)
        }
        other => Err(Error::Unsupported(format!(
            "procedure not implemented yet: {}{} (step {})",
            other.keyword(),
            if step.nlgeom { ", NLGEOM" } else { "" },
            step.name
        ))),
    }
}
