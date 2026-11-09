//! The session API of the Nodalis solver: load a model, run its steps one after another
//! on a persistent state, keep the results, snapshot the state for a restart. The command
//! line is its only user in phase 1; the Nodalis node graph will call it directly in phase
//! 2 (a state link is a [`State`] value, not a `.rout` file).
//!
//! A session computes on a rayon thread pool: the caller's ([`SessionOptions::pool`]), which
//! several sessions may share, so that one large solve gets every thread and several small
//! ones divide them as they need them (Nodalis keeps one such pool), or one of its own with
//! `threads` threads (the command line). No global state, so several sessions can run in
//! one process.

pub mod cache;
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

pub use cache::StepCache;
pub use check::{Check, check_analysis, check_input, format_diagnostic};
pub use job::{
    JobOptions, JobOutcome, JobProfile, KeptResults, ProfileSink, format_event,
    run_analysis_keeping, run_input_file, run_job, run_job_keeping,
};
pub use plxs_procedures::StepOutcome;
pub use plxs_procedures::fields::unsupported_outputs;

#[derive(Clone, Debug)]
pub struct SessionOptions {
    /// Threads of the session's own pool (0: all cores), when no `pool` is given.
    pub threads: usize,
    /// A pool shared with other sessions; the session computes on it instead of a pool of
    /// its own.
    pub pool: Option<Arc<rayon::ThreadPool>>,
    /// Where the job's files go (restart, eigenmodes).
    pub work_dir: PathBuf,
    /// Stops the run between increments when cancelled.
    pub cancel: CancelToken,
}

impl Default for SessionOptions {
    fn default() -> Self {
        Self {
            threads: 0,
            pool: None,
            work_dir: PathBuf::from("."),
            cancel: CancelToken::new(),
        }
    }
}

pub struct Session {
    model: Model,
    table: ElementTable,
    constraints: ModelConstraints,
    /// Contact pairs, prepared once; moved into the procedure context for a step.
    contact: plxs_contact::ContactModel,
    /// The matrix pattern with the contact springs of the last increment, kept over the
    /// steps so that the next one can reuse it.
    contact_pattern: Option<Arc<Pattern>>,
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
    pool: Arc<rayon::ThreadPool>,
    threads: usize,
    options: SessionOptions,
    /// [`restart::model_fingerprint`] of the model as given, before ties moved nodes.
    fingerprint: u64,
    /// The state before an isolated group of steps ([`plxs_core::step::Isolation`]), which
    /// returns after its last step.
    kept: Option<(State, Option<f64>)>,
}

/// A pool of `threads` threads (0: all cores).
pub fn own_pool(threads: usize) -> Result<rayon::ThreadPool> {
    let threads = if threads == 0 {
        std::thread::available_parallelism().map_or(1, |n| n.get())
    } else {
        threads
    };
    rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .thread_name(|i| format!("plxs-{i}"))
        .build()
        .map_err(|e| Error::Unsupported(format!("thread pool: {e}")))
}

impl Session {
    /// Prepares a model: element table, DOF numbering, sparsity pattern.
    pub fn new(model: Model, options: SessionOptions) -> Result<Session> {
        let pool = match &options.pool {
            Some(pool) => Arc::clone(pool),
            None => Arc::new(own_pool(options.threads)?),
        };
        let threads = pool.current_num_threads();
        let fingerprint = restart::model_fingerprint(&model);
        // Ties move their slave nodes before anything uses the coordinates.
        let mut model = model;
        let constraints = ModelConstraints::new(&mut model)?;
        // Contact pairs with ADJUST move their slave nodes too.
        let contact = plxs_contact::ContactModel::new(&mut model)?;
        let mut pending_warnings = constraints.warnings.clone();
        pending_warnings.extend(contact.warnings.iter().cloned());
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
        let mut state = State {
            displacements: plxs_core::state::NodalField::zeros(model.nodes.len(), 3),
            ..State::default()
        };
        // Modes stored by an earlier run (`<job>.eig`) for modal steps without a frequency
        // step of their own.
        if let Some(path) = &model.stored_modes_file {
            match read_stored_modes(path, &model) {
                Ok(modes) => state.stored_modes = Some(modes),
                Err(e) => pending_warnings.push(format!("{e}; the file is not used")),
            }
        }
        Ok(Session {
            model,
            table,
            constraints,
            contact,
            contact_pattern: None,
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
            kept: None,
        })
    }

    /// Computes a step on the current state; events go to `progress`.
    pub fn run_step(&mut self, step: &Step, progress: &mut dyn Progress) -> Result<StepOutcome> {
        if step.isolation.opens {
            self.open_group(step.isolation.unloaded);
        }
        let outcome = self.run_one_step(step, progress);
        if step.isolation.closes {
            self.close_group();
        }
        outcome
    }

    /// Keeps the state for the end of an isolated group; with `unloaded` the group starts
    /// from the unloaded model, at the time the steps before it reached.
    fn open_group(&mut self, unloaded: bool) {
        self.kept = Some((self.state.clone(), self.average_force));
        if unloaded {
            self.state = State {
                steps_done: self.state.steps_done,
                total_time: self.state.total_time,
                displacements: plxs_core::state::NodalField::zeros(self.model.nodes.len(), 3),
                stored_modes: self.state.stored_modes.clone(),
                ..State::default()
            };
            self.average_force = None;
            self.stiffness = None;
            self.contact_pattern = None;
        }
    }

    /// Returns to the state before the isolated group that ends here; the steps of the
    /// group still count.
    fn close_group(&mut self) {
        if let Some((state, average_force)) = self.kept.take() {
            let steps_done = self.state.steps_done;
            self.state = state;
            self.state.steps_done = steps_done;
            self.average_force = average_force;
            self.stiffness = None;
            self.contact_pattern = None;
        }
    }

    fn run_one_step(&mut self, step: &Step, progress: &mut dyn Progress) -> Result<StepOutcome> {
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
            contact: std::mem::take(&mut self.contact),
            contact_pattern: self.contact_pattern.take(),
            profile: std::mem::take(&mut self.profile),
            segment: None,
        };
        // The procedure runs on the session's pool in a thread of its own; events come
        // through a channel so that the caller's (not necessarily `Send`) progress sink
        // sees them as they happen.
        let (tx, rx) = std::sync::mpsc::channel::<ProgressEvent>();
        let pool = &self.pool;
        let cancel = self.options.cancel.clone();
        let (outcome, back) = std::thread::scope(|scope| {
            let handle = scope.spawn(move || {
                let mut ctx = ctx;
                let mut sink = ChannelProgress(tx);
                // The linear algebra looks at the token itself: a stop ends a long
                // factorization or eigensolution at once, not after it.
                let outcome = pool.install(|| cancel.scope(|| run_step(&mut ctx, step, &mut sink)));
                let outcome = match outcome {
                    Err(Error::Cancelled) => Ok(StepOutcome::Cancelled),
                    outcome => outcome,
                };
                let back = Carried {
                    stiffness: ctx.stiffness.take(),
                    solver: ctx.solver,
                    sta: std::mem::take(&mut ctx.sta),
                    cvg: std::mem::take(&mut ctx.cvg),
                    average_force: ctx.average_force,
                    profile: std::mem::take(&mut ctx.profile),
                    contact: std::mem::take(&mut ctx.contact),
                    contact_pattern: ctx.contact_pattern.take(),
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
                        contact: plxs_contact::ContactModel::default(),
                        contact_pattern: None,
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
        self.contact = back.contact;
        self.contact_pattern = back.contact_pattern;
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

    pub fn results_mut(&mut self) -> &mut Results {
        &mut self.results
    }

    /// Time per phase of the steps so far, with the symbolic analyses the direct solver
    /// reused.
    pub fn profile(&self) -> plxs_procedures::profile::Profile {
        let mut profile = self.profile.clone();
        profile.analyses_reused = self.solver.analyses_reused();
        profile
    }

    /// Threads the session computes on.
    pub fn threads(&self) -> usize {
        self.threads
    }

    /// Degrees of freedom of the model, before constraints are eliminated.
    pub fn degrees_of_freedom(&self) -> usize {
        self.pattern.dimension()
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
/// The modes of a `.eig` file that `plxs` wrote (`*FREQUENCY, STORAGE=YES`), checked against
/// the nodes of the model.
fn read_stored_modes(
    path: &std::path::Path,
    model: &Model,
) -> Result<plxs_core::state::StoredModes> {
    let io = |message: String| Error::Io {
        path: path.display().to_string(),
        message,
    };
    let bytes = std::fs::read(path).map_err(|e| io(e.to_string()))?;
    let modes: plxs_core::state::StoredModes = postcard::from_bytes(&bytes)
        .map_err(|e| io(format!("not a file of stored modes of this solver: {e}")))?;
    let nodes = model.nodes.len();
    if modes
        .modes
        .iter()
        .any(|m| m.per_node == 0 || m.values.len() != nodes * m.per_node)
    {
        return Err(Error::Unsupported(format!(
            "the stored modes of {} do not fit the model ({nodes} nodes)",
            path.display()
        )));
    }
    Ok(modes)
}

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
    contact: plxs_contact::ContactModel,
    contact_pattern: Option<Arc<Pattern>>,
}

/// Forwards events into a channel.
struct ChannelProgress(std::sync::mpsc::Sender<ProgressEvent>);

impl Progress for ChannelProgress {
    fn event(&mut self, event: ProgressEvent) {
        let _ = self.0.send(event);
    }
}
