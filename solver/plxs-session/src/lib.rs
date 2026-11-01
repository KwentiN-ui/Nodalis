//! The session API of the PrePoLix solver: load a model, run its steps one after another
//! on a persistent state, keep the results, snapshot the state for a restart. The command
//! line is its only user in phase 1; the PrePoLix node graph will call it directly in phase
//! 2 (a state link is a [`State`] value, not a `.rout` file).
//!
//! Every session has its own rayon thread pool with `threads` threads; no global state, so
//! several sessions can run in one process.

pub mod check;
pub mod job;
pub mod restart;

use std::path::PathBuf;
use std::sync::Arc;

use plxs_assembly::{ElementTable, build_pattern};
use plxs_constraints::ModelConstraints;
use plxs_core::state::Snapshot;
use plxs_core::{CancelToken, Error, Model, Progress, ProgressEvent, Result, Results, State, Step};
use plxs_linalg::{LdltSolver, Pattern};
use plxs_output::{CvgRow, StaRow};
use plxs_procedures::{ProcedureContext, run_step};

pub use check::{Check, check_analysis, check_input, format_diagnostic};
pub use job::{JobOptions, JobOutcome, format_event, run_job};
pub use plxs_procedures::StepOutcome;
pub use plxs_procedures::fields::unsupported_outputs;

#[derive(Clone, Debug)]
pub struct SessionOptions {
    /// Threads of the session's pool (0: all cores).
    pub threads: usize,
    /// Where the job's files go (restart, eigenmodes).
    pub work_dir: PathBuf,
    /// Stops the run between increments when cancelled.
    pub cancel: CancelToken,
}

impl Default for SessionOptions {
    fn default() -> Self {
        Self {
            threads: 0,
            work_dir: PathBuf::from("."),
            cancel: CancelToken::new(),
        }
    }
}

pub struct Session {
    model: Model,
    table: ElementTable,
    constraints: ModelConstraints,
    /// Warnings of the model preparation, reported with the first step.
    pending_warnings: Vec<String>,
    pattern: Arc<Pattern>,
    stiffness: Option<plxs_linalg::SparseMatrix>,
    solver: LdltSolver,
    state: State,
    results: Results,
    sta: Vec<StaRow>,
    cvg: Vec<CvgRow>,
    average_force: Option<f64>,
    profile: plxs_procedures::profile::Profile,
    pool: rayon::ThreadPool,
    threads: usize,
    options: SessionOptions,
    /// [`restart::model_fingerprint`] of the model as given, before ties moved nodes.
    fingerprint: u64,
}

impl Session {
    /// Prepares a model: element table, DOF numbering, sparsity pattern.
    pub fn new(model: Model, options: SessionOptions) -> Result<Session> {
        let threads = if options.threads == 0 {
            std::thread::available_parallelism().map_or(1, |n| n.get())
        } else {
            options.threads
        };
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .map_err(|e| Error::Unsupported(format!("thread pool: {e}")))?;
        let fingerprint = restart::model_fingerprint(&model);
        // Ties move their slave nodes before anything uses the coordinates.
        let mut model = model;
        let constraints = ModelConstraints::new(&mut model)?;
        let pending_warnings = constraints.warnings.clone();
        let (table, pattern) = pool.install(|| -> Result<_> {
            let mut table = ElementTable::new(&model, 0.0)?;
            // Nodes outside every element that constraints tie in (rigid body reference
            // and rotation nodes) keep their DOFs; the out-of-plane DOF of plane nodes
            // stays held.
            let referenced = constraints.referenced_nodes();
            let unused = table.dofs.unused.clone();
            table.dofs.unused.retain(|&dof| {
                let node = dof / 3;
                let unconnected = unused.binary_search(&(node * 3)).is_ok()
                    && unused.binary_search(&(node * 3 + 1)).is_ok();
                !(unconnected && referenced.binary_search(&node).is_ok())
            });
            let pattern = build_pattern(&table);
            Ok((table, pattern))
        })?;
        let state = State {
            displacements: plxs_core::state::NodalField::zeros(model.nodes.len(), 3),
            ..State::default()
        };
        Ok(Session {
            model,
            table,
            constraints,
            pending_warnings,
            pattern,
            stiffness: None,
            solver: LdltSolver::new(threads),
            state,
            results: Results::default(),
            sta: Vec::new(),
            cvg: Vec::new(),
            average_force: None,
            profile: Default::default(),
            pool,
            threads,
            options,
            fingerprint,
        })
    }

    /// Computes a step on the current state; events go to `progress`.
    pub fn run_step(&mut self, step: &Step, progress: &mut dyn Progress) -> Result<StepOutcome> {
        let step_number = self.state.steps_done + 1;
        progress.event(ProgressEvent::Step { step: step_number });
        for warning in std::mem::take(&mut self.pending_warnings) {
            progress.event(ProgressEvent::Warning(warning));
        }
        let ctx = ProcedureContext {
            model: &self.model,
            table: &self.table,
            constraints: &self.constraints,
            pattern: self.pattern.clone(),
            stiffness: self.stiffness.take(),
            solver: std::mem::replace(&mut self.solver, LdltSolver::new(1)),
            state: &mut self.state,
            results: &mut self.results,
            threads: self.threads,
            step_number,
            sta: std::mem::take(&mut self.sta),
            cvg: std::mem::take(&mut self.cvg),
            average_force: self.average_force,
            work_dir: self.options.work_dir.clone(),
            cancel: self.options.cancel.clone(),
            profile: std::mem::take(&mut self.profile),
        };
        // The procedure runs on the session's pool in a thread of its own; events come
        // through a channel so that the caller's (not necessarily `Send`) progress sink
        // sees them as they happen.
        let (tx, rx) = std::sync::mpsc::channel::<ProgressEvent>();
        let pool = &self.pool;
        let (outcome, back) = std::thread::scope(|scope| {
            let handle = scope.spawn(move || {
                let mut ctx = ctx;
                let mut sink = ChannelProgress(tx);
                let outcome = pool.install(|| run_step(&mut ctx, step, &mut sink));
                let back = Carried {
                    stiffness: ctx.stiffness.take(),
                    solver: ctx.solver,
                    sta: std::mem::take(&mut ctx.sta),
                    cvg: std::mem::take(&mut ctx.cvg),
                    average_force: ctx.average_force,
                    profile: std::mem::take(&mut ctx.profile),
                };
                (outcome, back)
            });
            for event in rx {
                progress.event(event);
            }
            handle.join().unwrap_or_else(|_| {
                (
                    Err(Error::Unsupported("the solver thread panicked".into())),
                    Carried {
                        stiffness: None,
                        solver: LdltSolver::new(1),
                        sta: Vec::new(),
                        cvg: Vec::new(),
                        average_force: None,
                        profile: Default::default(),
                    },
                )
            })
        });
        self.stiffness = back.stiffness;
        self.solver = back.solver;
        self.sta = back.sta;
        self.cvg = back.cvg;
        self.average_force = back.average_force;
        self.profile = back.profile;
        let outcome = outcome?;
        progress.event(ProgressEvent::StepDone {
            step: step_number,
            converged: matches!(outcome, StepOutcome::Converged),
        });
        Ok(outcome)
    }

    pub fn model(&self) -> &Model {
        &self.model
    }

    pub fn state(&self) -> &State {
        &self.state
    }

    pub fn results(&self) -> &Results {
        &self.results
    }

    /// Time per phase of the steps so far, with the symbolic analyses the direct solver
    /// reused.
    pub fn profile(&self) -> plxs_procedures::profile::Profile {
        let mut profile = self.profile.clone();
        profile.analyses_reused = self.solver.analyses_reused();
        profile
    }

    /// Rows of the `.sta` file so far.
    pub fn sta_rows(&self) -> &[StaRow] {
        &self.sta
    }

    /// Rows of the `.cvg` file so far.
    pub fn cvg_rows(&self) -> &[CvgRow] {
        &self.cvg
    }

    /// Number of steps computed so far, including those of a resumed snapshot.
    pub fn steps_done(&self) -> u32 {
        self.state.steps_done
    }

    /// The state after the last step with the model's fingerprint and what the session
    /// carries to the next step: what `*RESTART, WRITE` stores and a state link hands on.
    /// Taken after every step it costs a copy of the state.
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            model_fingerprint: self.fingerprint,
            state: self.state.clone(),
            average_force: self.average_force,
        }
    }

    /// Continues from a snapshot: the model must be the one the snapshot was taken of (sets,
    /// surfaces and amplitudes may be new), the next step is the one after the snapshot's.
    /// The results of the earlier steps are not part of the session; they stay with the
    /// caller.
    pub fn resume(model: Model, snapshot: Snapshot, options: SessionOptions) -> Result<Session> {
        if restart::model_fingerprint(&model) != snapshot.model_fingerprint {
            return Err(Error::Unsupported(
                "the restart state belongs to another model: nodes, elements, materials, \
                 sections, constraints or contact changed"
                    .into(),
            ));
        }
        let mut session = Session::new(model, options)?;
        check_layout(&session.model, &snapshot.state)?;
        session.state = snapshot.state;
        session.average_force = snapshot.average_force;
        Ok(session)
    }
}

/// A state that does not fit the model would index out of bounds in the procedures.
fn check_layout(model: &Model, state: &State) -> Result<()> {
    let nodes = model.nodes.len();
    let elements = model.elements.len();
    let nodal = |name: &str, field: Option<&plxs_core::state::NodalField>| -> Result<()> {
        match field {
            Some(f) if f.per_node == 0 || f.values.len() != nodes * f.per_node => {
                Err(Error::Unsupported(format!(
                    "the restart state does not fit the model: {name} for {} nodes, the model has {nodes}",
                    f.values.len() / f.per_node.max(1)
                )))
            }
            _ => Ok(()),
        }
    };
    nodal("displacements", Some(&state.displacements))?;
    nodal("velocities", state.velocities.as_ref())?;
    nodal("accelerations", state.accelerations.as_ref())?;
    nodal("temperatures", state.temperatures.as_ref())?;
    for (name, field) in [
        ("stresses", &state.stresses),
        ("strains", &state.strains),
        ("material state", &state.material),
    ] {
        if !field.offsets.is_empty() && field.offsets.len() != elements + 1 {
            return Err(Error::Unsupported(format!(
                "the restart state does not fit the model: {name} of {} elements, the model has {elements}",
                field.offsets.len() - 1
            )));
        }
    }
    Ok(())
}

/// What a step hands back to the session besides its outcome.
struct Carried {
    stiffness: Option<plxs_linalg::SparseMatrix>,
    solver: LdltSolver,
    sta: Vec<StaRow>,
    cvg: Vec<CvgRow>,
    average_force: Option<f64>,
    profile: plxs_procedures::profile::Profile,
}

/// Forwards events into a channel.
struct ChannelProgress(std::sync::mpsc::Sender<ProgressEvent>);

impl Progress for ChannelProgress {
    fn event(&mut self, event: ProgressEvent) {
        let _ = self.0.send(event);
    }
}
