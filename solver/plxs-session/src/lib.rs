//! The session API of the PrePoLix solver: load a model, run its steps one after another
//! on a persistent state, keep the results, snapshot the state for a restart. The command
//! line is its only user in phase 1; the PrePoLix node graph will call it directly in phase
//! 2 (a state link is a [`State`] value, not a `.rout` file).
//!
//! Every session has its own rayon thread pool with `threads` threads; no global state, so
//! several sessions can run in one process.

use std::path::PathBuf;
use std::sync::Arc;

use plxs_assembly::{ElementTable, build_pattern};
use plxs_constraints::ModelConstraints;
use plxs_core::state::Snapshot;
use plxs_core::{Error, Model, Progress, ProgressEvent, Result, Results, State, Step};
use plxs_linalg::{LdltSolver, Pattern};
use plxs_output::StaRow;
use plxs_procedures::{ProcedureContext, run_step};

pub use plxs_procedures::StepOutcome;
pub use plxs_procedures::fields::unsupported_outputs;

#[derive(Clone, Debug)]
pub struct SessionOptions {
    /// Threads of the session's pool (0: all cores).
    pub threads: usize,
    /// Where the job's files go (restart, eigenmodes).
    pub work_dir: PathBuf,
}

impl Default for SessionOptions {
    fn default() -> Self {
        Self {
            threads: 0,
            work_dir: PathBuf::from("."),
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
    pool: rayon::ThreadPool,
    threads: usize,
    #[allow(dead_code)]
    options: SessionOptions,
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
        // Ties move their slave nodes before anything uses the coordinates.
        let mut model = model;
        let constraints = ModelConstraints::new(&mut model)?;
        let pending_warnings = constraints.warnings.clone();
        let (table, pattern) = pool.install(|| -> Result<_> {
            let table = ElementTable::new(&model, 0.0)?;
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
            pool,
            threads,
            options,
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
        };
        // The procedure runs on the session's pool in a thread of its own; events come
        // through a channel so that the caller's (not necessarily `Send`) progress sink
        // sees them as they happen.
        let (tx, rx) = std::sync::mpsc::channel::<ProgressEvent>();
        let pool = &self.pool;
        let (outcome, stiffness, solver, sta) = std::thread::scope(|scope| {
            let handle = scope.spawn(move || {
                let mut ctx = ctx;
                let mut sink = ChannelProgress(tx);
                let outcome = pool.install(|| run_step(&mut ctx, step, &mut sink));
                (
                    outcome,
                    ctx.stiffness.take(),
                    ctx.solver,
                    std::mem::take(&mut ctx.sta),
                )
            });
            for event in rx {
                progress.event(event);
            }
            handle.join().unwrap_or_else(|_| {
                (
                    Err(Error::Unsupported("the solver thread panicked".into())),
                    None,
                    LdltSolver::new(1),
                    Vec::new(),
                )
            })
        });
        self.stiffness = stiffness;
        self.solver = solver;
        self.sta = sta;
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

    /// Rows of the `.sta` file so far.
    pub fn sta_rows(&self) -> &[StaRow] {
        &self.sta
    }

    /// The state with a fingerprint of the model, for `*RESTART, WRITE`.
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            model_fingerprint: fingerprint(&self.model),
            state: self.state.clone(),
        }
    }

    /// Continues from a snapshot of the same model.
    pub fn resume(model: Model, snapshot: Snapshot, options: SessionOptions) -> Result<Session> {
        if fingerprint(&model) != snapshot.model_fingerprint {
            return Err(Error::Unsupported(
                "the restart file belongs to another model".into(),
            ));
        }
        let mut session = Session::new(model, options)?;
        session.state = snapshot.state;
        Ok(session)
    }
}

/// Forwards events into a channel.
struct ChannelProgress(std::sync::mpsc::Sender<ProgressEvent>);

impl Progress for ChannelProgress {
    fn event(&mut self, event: ProgressEvent) {
        let _ = self.0.send(event);
    }
}

/// A hash of the model's nodes, elements and sections.
fn fingerprint(model: &Model) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    model.nodes.ids.hash(&mut h);
    for c in &model.nodes.coordinates {
        for v in c {
            v.to_bits().hash(&mut h);
        }
    }
    for e in &model.elements {
        e.id.hash(&mut h);
        e.kind.name().hash(&mut h);
        e.nodes.hash(&mut h);
    }
    model.sections.len().hash(&mut h);
    h.finish()
}
