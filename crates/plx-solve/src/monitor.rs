//! How a run converges, collected from the solver's events as they come: the residual and
//! correction of every Newton iteration with the limits they had to reach, the increments
//! with their sizes and times, the cutbacks, and the iterations of the iterative linear
//! solver. The monitor plots it while the run goes on ([`LiveLog`]).

use std::sync::{Arc, Mutex, MutexGuard};

use plxs_core::{LinearIteration, ProgressEvent};

/// Points kept per series; older ones are dropped from the front in a long run.
pub const MAX_POINTS: usize = 200_000;

/// Iterations kept of a solve of the iterative linear solver.
const MAX_LINEAR_POINTS: usize = 20_000;

/// A Newton iteration whose convergence the procedure checked.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IterationPoint {
    /// Number of the checked iteration in the whole run, from 1: the x of the plot.
    pub x: u64,
    pub step: u32,
    pub increment: u32,
    pub attempt: u32,
    pub iteration: u32,
    /// Largest residual force and largest correction of the solution in the iteration.
    pub residual: f64,
    pub correction: f64,
    /// What they had to reach; `None` when the procedure did not check it.
    pub residual_limit: Option<f64>,
    pub correction_limit: Option<f64>,
    pub converged: bool,
}

impl IterationPoint {
    /// The residual over its limit: converged at 1 or below.
    pub fn residual_ratio(&self) -> Option<f64> {
        ratio(self.residual, self.residual_limit?)
    }

    /// The correction over its limit, from the second iteration on: the first correction
    /// is the whole change of the increment so far, its ratio is the inverse of the tolerance
    /// whatever the model does.
    pub fn correction_ratio(&self) -> Option<f64> {
        if self.iteration < 2 {
            return None;
        }
        ratio(self.correction, self.correction_limit?)
    }
}

fn ratio(value: f64, limit: f64) -> Option<f64> {
    let ratio = value / limit;
    (ratio.is_finite() && ratio > 0.0).then_some(ratio)
}

/// An increment that converged.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IncrementPoint {
    /// Number of the converged increment in the whole run, from 1.
    pub x: u64,
    pub step: u32,
    pub increment: u32,
    pub attempts: u32,
    pub iterations: u32,
    pub size: f64,
    pub step_time: f64,
    pub total_time: f64,
    /// The checked iterations so far, see [`IterationPoint::x`]: where the increment ends
    /// in the plot of the iterations.
    pub iterations_x: u64,
}

/// An attempt of an increment that did not converge and was cut back.
#[derive(Clone, Debug, PartialEq)]
pub struct Cutback {
    /// The increment it tried, numbered as [`IncrementPoint::x`].
    pub x: u64,
    /// The checked iterations so far, see [`IterationPoint::x`].
    pub iterations_x: u64,
    pub step: u32,
    pub increment: u32,
    pub attempt: u32,
    /// Size and step time of the attempt.
    pub size: f64,
    pub step_time: f64,
    pub total_time: f64,
    pub reason: String,
}

/// A step of the run as far as it got.
#[derive(Clone, Debug, PartialEq)]
pub struct StepRecord {
    pub number: u32,
    pub name: String,
    /// The step time it runs to, for the steps that go through time.
    pub period: Option<f64>,
    /// The checked iterations and converged increments before it.
    pub iterations_x: u64,
    pub increments_x: u64,
    /// The step finished; `Some(false)` when it did not converge.
    pub converged: Option<bool>,
}

/// A solve of the iterative linear solver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearSolve {
    pub iterations: u32,
    pub relative_residual: f64,
    pub seconds: f64,
}

/// The iterations of one solve of the iterative linear solver: (iteration, relative
/// residual).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LinearCurve {
    pub tolerance: f64,
    pub points: Vec<[f64; 2]>,
}

/// The increment the solver is at.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Current {
    pub step: u32,
    pub increment: u32,
    pub attempt: u32,
    pub iteration: u32,
    pub size: f64,
    pub step_time: f64,
    pub total_time: f64,
}

/// What a run told about its convergence so far.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ConvergenceLog {
    pub iterations: Vec<IterationPoint>,
    pub increments: Vec<IncrementPoint>,
    pub cutbacks: Vec<Cutback>,
    pub steps: Vec<StepRecord>,
    pub solves: Vec<LinearSolve>,
    /// The solve of the iterative linear solver going on or done last, and the one
    /// before, to compare with.
    pub linear: Option<LinearCurve>,
    pub linear_before: Option<LinearCurve>,
    pub current: Option<Current>,
    /// Counts the changes, for a view that redraws only when something came.
    pub revision: u64,
    /// The checked iterations and converged increments of the whole run.
    iterations_x: u64,
    increments_x: u64,
    /// Names and periods of the steps of the analysis, by number from 1.
    plan: Vec<(String, Option<f64>)>,
}

impl ConvergenceLog {
    /// A log for a run of the steps `plan`: their names and the step time they run to.
    pub fn new(plan: Vec<(String, Option<f64>)>) -> Self {
        Self {
            plan,
            ..Self::default()
        }
    }

    /// Takes an event of the run into account.
    pub fn event(&mut self, event: &ProgressEvent) {
        match *event {
            ProgressEvent::Step { step } => {
                let (name, period) = (self.plan.get(step as usize - 1).cloned())
                    .unwrap_or_else(|| (format!("Step-{step}"), None));
                self.steps.push(StepRecord {
                    number: step,
                    name,
                    period,
                    iterations_x: self.iterations_x,
                    increments_x: self.increments_x,
                    converged: None,
                });
                self.current = Some(Current {
                    step,
                    ..Current::default()
                });
            }
            ProgressEvent::Increment {
                step,
                increment,
                attempt,
                size,
                step_time,
                total_time,
            } => {
                self.current = Some(Current {
                    step,
                    increment,
                    attempt,
                    iteration: 0,
                    size,
                    step_time,
                    total_time,
                });
            }
            ProgressEvent::Iteration {
                step,
                increment,
                iteration,
                residual,
                correction,
                residual_limit,
                correction_limit,
                converged,
            } => {
                let attempt = match &mut self.current {
                    Some(current) => {
                        current.iteration = iteration;
                        current.attempt
                    }
                    None => 1,
                };
                // A linear solution checks nothing: no point of the plot.
                if residual_limit.is_some() || correction_limit.is_some() {
                    self.iterations_x += 1;
                    self.iterations.push(IterationPoint {
                        x: self.iterations_x,
                        step,
                        increment,
                        attempt: attempt.max(1),
                        iteration,
                        residual,
                        correction,
                        residual_limit,
                        correction_limit,
                        converged,
                    });
                    keep_last(&mut self.iterations);
                }
            }
            ProgressEvent::IncrementDone {
                step,
                increment,
                attempt,
                iterations,
                total_time,
                step_time,
                size,
            } => {
                self.increments_x += 1;
                self.increments.push(IncrementPoint {
                    x: self.increments_x,
                    step,
                    increment,
                    attempts: attempt.max(1),
                    iterations,
                    size,
                    step_time,
                    total_time,
                    iterations_x: self.iterations_x,
                });
                keep_last(&mut self.increments);
            }
            ProgressEvent::Cutback {
                step,
                increment,
                attempt,
                ref reason,
            } => {
                let current = self.current.unwrap_or_default();
                self.cutbacks.push(Cutback {
                    x: self.increments_x + 1,
                    iterations_x: self.iterations_x,
                    step,
                    increment,
                    attempt,
                    size: current.size,
                    step_time: current.step_time,
                    total_time: current.total_time,
                    reason: reason.clone(),
                });
                keep_last(&mut self.cutbacks);
            }
            ProgressEvent::StepDone { step, converged } => {
                if let Some(record) = self.steps.iter_mut().rev().find(|s| s.number == step) {
                    record.converged = Some(converged);
                }
            }
            ProgressEvent::LinearSolve {
                iterations,
                relative_residual,
                seconds,
                ..
            } => {
                self.solves.push(LinearSolve {
                    iterations,
                    relative_residual,
                    seconds,
                });
                keep_last(&mut self.solves);
            }
            ProgressEvent::JobDone => self.current = None,
            _ => return,
        }
        self.revision += 1;
    }

    /// Takes an iteration of the iterative linear solver into account.
    pub fn linear(&mut self, iteration: LinearIteration) {
        match iteration {
            LinearIteration::Start { tolerance } => {
                // A solve that stopped at once (a rebuilt preconditioner, the next try)
                // is no solve to compare with.
                let previous = self.linear.take().filter(|c| c.points.len() > 1);
                if previous.is_some() {
                    self.linear_before = previous;
                }
                self.linear = Some(LinearCurve {
                    tolerance,
                    points: Vec::new(),
                });
            }
            LinearIteration::Residual {
                iteration,
                residual,
            } => {
                let curve = self.linear.get_or_insert_with(LinearCurve::default);
                if curve.points.len() < MAX_LINEAR_POINTS {
                    curve.points.push([f64::from(iteration), residual]);
                } else if let Some(last) = curve.points.last_mut() {
                    // The end of a very long solve: its last point moves on.
                    *last = [f64::from(iteration), residual];
                }
            }
        }
        self.revision += 1;
    }

    /// The step the run is in, or did last.
    pub fn step(&self) -> Option<&StepRecord> {
        self.steps.last()
    }

    /// How far the step the run is in got through its period, 0 to 1: the step time
    /// of the last converged increment of the step over the period.
    pub fn step_fraction(&self) -> Option<f64> {
        let step = self.step()?;
        let period = step.period.filter(|p| *p > 0.0)?;
        let done = (self.increments.last())
            .filter(|i| i.step == step.number)
            .map_or(0.0, |i| i.step_time);
        Some((done / period).clamp(0.0, 1.0))
    }

    /// Nothing told yet that the plot could show.
    pub fn is_empty(&self) -> bool {
        self.steps.is_empty() && self.increments.is_empty() && self.linear.is_none()
    }
}

/// Drops the oldest points over [`MAX_POINTS`], a quarter at a time.
fn keep_last<T>(points: &mut Vec<T>) {
    if points.len() > MAX_POINTS {
        points.drain(..MAX_POINTS / 4);
    }
}

/// The log of a running solve, shared by the solver thread that fills it and the monitor
/// that shows it.
#[derive(Clone, Debug, Default)]
pub struct LiveLog(Arc<Mutex<ConvergenceLog>>);

impl LiveLog {
    pub fn new(log: ConvergenceLog) -> Self {
        Self(Arc::new(Mutex::new(log)))
    }

    pub fn lock(&self) -> MutexGuard<'_, ConvergenceLog> {
        self.0.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// The watch that takes the iterations of the iterative linear solver into the log.
    pub fn linear_watch(&self) -> plxs_core::LinearWatch {
        let log = self.clone();
        plxs_core::LinearWatch::new(move |iteration| log.lock().linear(iteration))
    }

    /// Both handles are of the same log.
    pub fn same(&self, other: &LiveLog) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn iteration(iteration: u32, residual: f64, converged: bool) -> ProgressEvent {
        ProgressEvent::Iteration {
            step: 1,
            increment: 1,
            iteration,
            residual,
            correction: residual * 1e-3,
            residual_limit: Some(1.0),
            correction_limit: Some(1e-3),
            converged,
        }
    }

    fn increment(increment: u32, attempt: u32, size: f64, step_time: f64) -> ProgressEvent {
        ProgressEvent::Increment {
            step: 1,
            increment,
            attempt,
            size,
            step_time,
            total_time: step_time,
        }
    }

    #[test]
    fn collects_iterations_increments_and_cutbacks() {
        let mut log = ConvergenceLog::new(vec![("Load".into(), Some(2.0))]);
        log.event(&ProgressEvent::Step { step: 1 });
        log.event(&increment(1, 1, 1.0, 1.0));
        log.event(&iteration(1, 100.0, false));
        log.event(&iteration(2, 300.0, false));
        log.event(&ProgressEvent::Cutback {
            step: 1,
            increment: 1,
            attempt: 1,
            reason: "the residual grows".into(),
        });
        log.event(&increment(1, 2, 0.25, 0.25));
        log.event(&iteration(1, 10.0, false));
        log.event(&iteration(2, 0.5, true));
        log.event(&ProgressEvent::IncrementDone {
            step: 1,
            increment: 1,
            attempt: 2,
            iterations: 2,
            total_time: 0.25,
            step_time: 0.25,
            size: 0.25,
        });
        assert_eq!(log.iterations.len(), 4);
        assert_eq!(log.iterations[2].attempt, 2);
        assert_eq!(log.iterations[3].x, 4);
        assert_eq!(log.iterations[3].residual_ratio(), Some(0.5));
        assert_eq!(log.iterations[3].correction_ratio(), Some(0.5));
        assert_eq!(log.iterations[2].correction_ratio(), None);
        assert_eq!(log.cutbacks.len(), 1);
        let cutback = &log.cutbacks[0];
        assert_eq!((cutback.x, cutback.iterations_x, cutback.size), (1, 2, 1.0));
        assert_eq!(log.increments[0].iterations_x, 4);
        assert_eq!(log.step().unwrap().name, "Load");
        assert_eq!(log.step_fraction(), Some(0.125));
        assert_eq!(log.current.unwrap().iteration, 2);
    }

    #[test]
    fn linear_solutions_add_no_iteration_points() {
        let mut log = ConvergenceLog::default();
        log.event(&ProgressEvent::Step { step: 1 });
        log.event(&ProgressEvent::Iteration {
            step: 1,
            increment: 1,
            iteration: 1,
            residual: 0.0,
            correction: 1.0,
            residual_limit: None,
            correction_limit: None,
            converged: true,
        });
        assert!(log.iterations.is_empty());
        assert_eq!(log.step().unwrap().name, "Step-1");
        assert_eq!(log.step_fraction(), None);
    }

    #[test]
    fn keeps_the_solve_before_of_the_linear_solver() {
        let mut log = ConvergenceLog::default();
        let residual = |iteration, residual| LinearIteration::Residual {
            iteration,
            residual,
        };
        log.linear(LinearIteration::Start { tolerance: 1e-12 });
        log.linear(residual(1, 0.1));
        log.linear(residual(2, 1e-13));
        log.linear(LinearIteration::Start { tolerance: 1e-12 });
        log.linear(residual(1, 0.5));
        assert_eq!(log.linear_before.as_ref().unwrap().points.len(), 2);
        assert_eq!(log.linear.as_ref().unwrap().points, vec![[1.0, 0.5]]);
        // A try that stopped at once leaves the solve before as it was.
        log.linear(LinearIteration::Start { tolerance: 1e-12 });
        assert_eq!(log.linear_before.as_ref().unwrap().points.len(), 2);
    }

    #[test]
    fn a_watch_fills_the_shared_log() {
        let live = LiveLog::default();
        let watch = live.linear_watch();
        watch.scope(|| {
            plxs_core::LinearWatch::current()
                .unwrap()
                .report(LinearIteration::Residual {
                    iteration: 1,
                    residual: 0.25,
                })
        });
        assert_eq!(
            live.lock().linear.as_ref().unwrap().points,
            vec![[1.0, 0.25]]
        );
        assert!(plxs_core::LinearWatch::current().is_none());
    }
}
