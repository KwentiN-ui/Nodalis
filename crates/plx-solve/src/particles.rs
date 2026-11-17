//! Runs of the particle solver (`plxs-particles`) for a Particle Flow step: like an
//! analysis of the FE solver they run on a thread of their own, report their progress to
//! the monitor and write a `.ndlr` file, whose results the Results workspace shows like any
//! other ([`plx_results::particles`]). The frames go to the spool on disk as they are
//! computed, so a long run needs the memory of one frame.

use std::path::Path;
use std::sync::{Arc, Mutex};

use plx_results::SolverResults;
use plx_results::particles::{FrameTrack, HistoryRequest, history_blocks, particle_mesh};
use plxs_core::particles::ParticleModel;
use plxs_core::{CancelToken, Diagnostic, ProgressEvent};

use crate::{Convergence, Finished, JobStatus, LiveProgress, Run, VERSION, monitor, pool};

/// A particle analysis made of the model for the particle solver.
#[derive(Clone, Debug)]
pub struct PreparedParticles {
    pub model: ParticleModel,
    /// The model in words, written as `<name>.inp` beside the results for reference.
    pub text: String,
    /// What the lowering noticed but did not stop for, shown as the solver's warnings.
    pub warnings: Vec<String>,
    /// The step of the FE model the run computes, kept with the results.
    pub steps: Vec<plx_results::steps::StepRecord>,
    /// The history the step's outputs ask for.
    pub history: HistoryRequest,
}

/// The particle model in words: what the solver gets, without the triangles.
pub fn describe(model: &ParticleModel) -> String {
    let mut out = String::new();
    let mut line = |text: String| {
        out.push_str("** ");
        out.push_str(&text);
        out.push('\n');
    };
    line(format!("Particle Flow: {}", model.heading));
    line("Units: SI (m, kg, s)".into());
    for m in &model.materials {
        line(format!(
            "Material {}: density {}, {:?}",
            m.name, m.density, m.law
        ));
    }
    for b in &model.bodies {
        line(format!(
            "Body {}: material {}, {} triangles",
            b.name,
            model
                .materials
                .get(b.material)
                .map_or("?", |m| m.name.as_str()),
            b.shape.triangles.len()
        ));
    }
    for w in &model.walls {
        line(format!(
            "Wall {}: {:?}, {} triangles",
            w.name,
            w.law,
            w.shape.triangles.len()
        ));
        if !w.motion.is_resting() {
            let m = &w.motion;
            line(format!(
                "  moves about {:?}: translation {:?}, rotation {:?} over {:?}",
                m.center, m.velocity, m.angular, m.amplitude
            ));
        }
    }
    let d = &model.domain;
    line(format!(
        "Box {:?} to {:?}, cell {}, sides {:?}",
        d.min, d.max, d.cell, d.sides
    ));
    line(format!("Gravity {:?}", model.gravity));
    line(format!("Control {:?}", model.control));
    out
}

impl Run {
    /// The model check of a Particle Flow step: the particle model could be made, its
    /// warnings and the model in words go to the output, nothing is computed.
    pub fn check_particles(
        work_dir: &Path,
        name: &str,
        prepared: PreparedParticles,
    ) -> std::io::Result<Self> {
        std::fs::create_dir_all(work_dir)?;
        std::fs::write(work_dir.join(format!("{name}.inp")), &prepared.text)?;
        let mut lines: Vec<String> = (prepared.warnings.iter())
            .map(|w| format!(" *WARNING: {w}"))
            .collect();
        lines.extend(prepared.text.lines().map(str::to_owned));
        let model = &prepared.model;
        lines.push(format!(
            " *INFO: particle model with {} bodies, {} walls; model check finished",
            model.bodies.len(),
            model.walls.len()
        ));
        let output = Arc::new(Mutex::new(lines));
        let handle = std::thread::Builder::new()
            .name(format!("plxs particle check {name}"))
            .spawn(|| Finished::Done)?;
        Ok(Self {
            name: name.to_owned(),
            work_dir: work_dir.to_owned(),
            output,
            solved: Arc::new(Mutex::new(None)),
            keep: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            written: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            live: LiveProgress::default(),
            cancel: CancelToken::new(),
            handle: Some(handle),
            status: JobStatus::Running,
            check: true,
            diagnostics: Arc::new(Mutex::new(Some(Vec::new()))),
            profile: plxs_session::ProfileSink::default(),
            awake: None,
            convergence: monitor::LiveLog::new(monitor::ConvergenceLog::new(Vec::new())),
        })
    }

    /// Starts the particle solver on a Particle Flow step, on the shared [`pool`] of
    /// `threads` threads (0: all cores): the results are written to `<work_dir>/<name>.ndlr`.
    pub fn start_particles(
        threads: u32,
        work_dir: &Path,
        name: &str,
        prepared: PreparedParticles,
    ) -> std::io::Result<Self> {
        std::fs::create_dir_all(work_dir)?;
        for extension in crate::OLD_FILES {
            let path = work_dir.join(format!("{name}.{extension}"));
            if path.exists() {
                std::fs::remove_file(path)?;
            }
        }
        std::fs::write(work_dir.join(format!("{name}.inp")), &prepared.text)?;
        let pool = pool(threads)?;
        let output = Arc::new(Mutex::new(vec![
            "************************************************************".to_owned(),
            format!("Nodalis particle solver plxs {VERSION}"),
            "************************************************************".to_owned(),
        ]));
        let cancel = CancelToken::new();
        let plan = (prepared.steps.iter())
            .map(|s| (s.name.clone(), Some(prepared.model.control.end_time)))
            .collect();
        let convergence = monitor::LiveLog::new(monitor::ConvergenceLog::new(plan));
        let lines = Arc::clone(&output);
        let log = convergence.clone();
        let solved = Arc::new(Mutex::new(None));
        let keep_results = Arc::clone(&solved);
        let keep = Arc::new(std::sync::atomic::AtomicBool::new(true));
        let written = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let (keeping, writing) = (Arc::clone(&keep), Arc::clone(&written));
        let results_file = work_dir.join(format!("{name}.{}", plx_results::solver::EXTENSION));
        let spool_file = work_dir.join(format!("{name}.{}.spool", plx_results::solver::EXTENSION));
        let diagnostics = Arc::new(Mutex::new(None));
        let found = Arc::clone(&diagnostics);
        let token = cancel.clone();
        let handle = std::thread::Builder::new()
            .name(format!("plxs particles {name}"))
            .spawn(move || {
                let say = |line: String| lines.lock().unwrap_or_else(|e| e.into_inner()).push(line);
                let fail = |message: String| {
                    say(format!("*ERROR: {message}"));
                    *found.lock().unwrap_or_else(|e| e.into_inner()) = Some(vec![Diagnostic::new(
                        plxs_core::DiagnosticCode::Numerical(plxs_core::Failure::Internal),
                        plxs_core::Severity::Error,
                        message,
                    )]);
                    Finished::Error
                };
                let PreparedParticles {
                    model,
                    warnings,
                    steps: records,
                    history: request,
                    ..
                } = prepared;
                let mut track = Convergence::default();
                // Both callbacks of the solver report progress.
                let progress = Mutex::new(|event: ProgressEvent| {
                    track.event(&event);
                    log.lock().event(&event);
                    for line in plxs_session::format_event(&event) {
                        say(line);
                    }
                });
                let progress =
                    |event: ProgressEvent| (progress.lock().unwrap_or_else(|e| e.into_inner()))(event);
                for warning in warnings {
                    progress(ProgressEvent::Warning(warning));
                }
                progress(ProgressEvent::Step { step: 1 });
                let mut spool = match plxs_output::spool::Spool::create(&spool_file) {
                    Ok(spool) => spool,
                    Err(e) => return fail(format!("results not spooled: {e}")),
                };
                let mut frames = None;
                let mut mesh = None;
                let mut increment = 0u32;
                let mut spool_error = None;
                let mut last_time = 0.0;
                let mut last_note = std::time::Instant::now();
                let mut told_card = false;
                let end_time = model.control.end_time;
                let frame_dt = end_time / model.control.frames.max(1) as f64;
                // Each frame is an increment (the first, at time zero, is the start): the
                // monitor shows which one runs and up to which time.
                let start = |increment: u32| {
                    // Increment 1 is the start, at time zero.
                    let time = (f64::from(increment - 1) * frame_dt).min(end_time);
                    ProgressEvent::Increment {
                        step: 1,
                        increment,
                        attempt: 1,
                        size: frame_dt,
                        step_time: time,
                        total_time: time,
                    }
                };
                let solved_run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    // The solver asks the token of its own thread: the pool's, where
                    // `install` runs it.
                    pool.install(|| {
                        token.scope(|| {
                            plxs_particles::solve_streaming(
                                &model,
                                &mut |status| {
                                    if status.on_card && !told_card {
                                        told_card = true;
                                        progress(ProgressEvent::Info(
                                            "particles: time steps on the graphics card".into(),
                                        ));
                                    }
                                    if last_note.elapsed().as_secs_f64() > 5.0 {
                                        last_note = std::time::Instant::now();
                                        progress(ProgressEvent::Info(format!(
                                            "time {:.4e} of {:.4e}, {} time steps of {:.3e}, \
                                             {} particles",
                                            status.time,
                                            status.end_time,
                                            status.steps,
                                            status.time_step,
                                            status.particles
                                        )));
                                    }
                                },
                                &mut |frame| {
                                    let track = frames.get_or_insert_with(|| {
                                        mesh = Some(particle_mesh(&model, &frame));
                                        FrameTrack::new(&model, &frame)
                                    });
                                    increment += 1;
                                    let size = frame.time - last_time;
                                    last_time = frame.time;
                                    if spool_error.is_none()
                                        && let Err(e) =
                                            spool.push(track.increment(&frame, 1, increment))
                                    {
                                        spool_error = Some(e.to_string());
                                    }
                                    progress(ProgressEvent::IncrementDone {
                                        step: 1,
                                        increment,
                                        attempt: 1,
                                        iterations: 0,
                                        total_time: frame.time,
                                        step_time: frame.time,
                                        size: if increment == 1 { frame_dt } else { size },
                                    });
                                    if frame.time < end_time - 1e-9 * end_time {
                                        progress(start(increment + 1));
                                    }
                                },
                            )
                        })
                    })
                }));
                let results = match solved_run {
                    Ok(Ok(results)) => results,
                    Ok(Err(message)) => return fail(message),
                    Err(panic) => {
                        let message = panic
                            .downcast_ref::<String>()
                            .cloned()
                            .or_else(|| panic.downcast_ref::<&str>().map(|s| (*s).to_owned()))
                            .unwrap_or_else(|| "unknown cause".into());
                        return fail(format!("the particle solver failed: {message}"));
                    }
                };
                if let Some(e) = spool_error {
                    return fail(format!("results not spooled: {e}"));
                }
                let report = &results.report;
                progress(ProgressEvent::Info(format!(
                    "{} particles, {} x {} x {} cells, {} time steps of {:.3e} to {:.3e} in {:.1} s",
                    report.particles,
                    report.cells[0],
                    report.cells[1],
                    report.cells[2],
                    report.steps,
                    report.time_step.0,
                    report.time_step.1,
                    report.seconds
                )));
                for note in &report.notes {
                    progress(ProgressEvent::Info(note.clone()));
                }
                if let Some(balance) = results.history.balance_error.last() {
                    progress(ProgressEvent::Info(format!(
                        "energy balance error at the end: {:.2} %",
                        100.0 * balance
                    )));
                }
                if token.is_cancelled() {
                    say(" *INFO: the run was stopped".into());
                    return Finished::Cancelled;
                }
                progress(ProgressEvent::StepDone {
                    step: 1,
                    converged: true,
                });
                let Some(mesh) = mesh else {
                    return fail("the particle solver wrote no frame".into());
                };
                let solved = plxs_core::Results {
                    history: history_blocks(&results.history, &request, 1),
                    ..Default::default()
                };
                let mut out = SolverResults::new(&mesh, solved, VERSION);
                out.steps = records;
                out.convergence = track.increments;
                match out.write_spooled(spool, &[], &results_file) {
                    Ok(()) => {
                        writing.store(true, std::sync::atomic::Ordering::Release);
                        if keeping.load(std::sync::atomic::Ordering::Acquire) {
                            *keep_results.lock().unwrap_or_else(|e| e.into_inner()) =
                                SolverResults::read(&results_file).ok().map(Arc::new);
                        }
                        Finished::Done
                    }
                    Err(e) => fail(format!(
                        "results not written to {}: {e}",
                        results_file.display()
                    )),
                }
            })?;
        Ok(Self {
            name: name.to_owned(),
            work_dir: work_dir.to_owned(),
            output,
            solved,
            keep,
            written,
            live: LiveProgress::default(),
            cancel,
            handle: Some(handle),
            status: JobStatus::Running,
            check: false,
            diagnostics,
            profile: plxs_session::ProfileSink::default(),
            awake: Some(crate::awake::Guard::new()),
            convergence,
        })
    }
}

#[cfg(test)]
mod tests {
    use plxs_core::particles::*;

    use super::*;

    #[test]
    fn a_particle_run_writes_results_the_results_workspace_reads() {
        let dir = std::env::temp_dir().join(format!("plx-particles-{}", std::process::id()));
        let model = ParticleModel {
            heading: "drop".into(),
            materials: vec![ParticleMaterial {
                name: "Water".into(),
                density: 1000.0,
                law: ParticleLaw::Fluid {
                    bulk: 2e5,
                    viscosity: 1.0,
                },
                thermal: None,
                flow_tables: Default::default(),
            }],
            bodies: vec![MaterialBody {
                name: "Drop".into(),
                material: 0,
                shape: TriangleSurface::cuboid([0.2, 0.2, 0.2], [0.4, 0.4, 0.4]),
                velocity: [0.0; 3],
                temperature: 0.0,
                gravity: [0.0; 3],
            }],
            domain: Domain {
                min: [0.0; 3],
                max: [0.6; 3],
                cell: 0.05,
                ..Default::default()
            },
            gravity: [0.0, 0.0, -9.81],
            control: ParticleControl {
                end_time: 0.05,
                frames: 2,
                ..Default::default()
            },
            ..Default::default()
        };
        let prepared = PreparedParticles {
            text: describe(&model),
            model,
            warnings: Vec::new(),
            steps: Vec::new(),
            history: HistoryRequest {
                energies: vec!["ALLKE".into()],
                ..Default::default()
            },
        };
        let mut run = Run::start_particles(1, &dir, "drop", prepared).unwrap();
        assert_eq!(run.wait(), JobStatus::Completed, "{:?}", run.new_output());
        let results = run.solver_results().unwrap();
        // Four cells of two particles each way, three frames.
        assert_eq!(results.model.elements.len(), 512);
        let import = plx_results::from_solver(&results);
        assert_eq!(import.increments.len(), 3);
        let last = &import.increments[2];
        let disp = last.field("DISP").unwrap().component("U3").unwrap();
        // The drop falls: g t^2 / 2 = 12 mm after 0.05 s.
        let fall = disp.values.get()[0];
        assert!((fall + 0.0123).abs() < 2e-3, "{fall}");
        assert!(last.field(plx_results::particles::FIELD).is_some());
        assert!(!results.results.history.is_empty());
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn a_stopped_particle_run_stops_computing() {
        let dir = std::env::temp_dir().join(format!("plx-particles-stop-{}", std::process::id()));
        let model = ParticleModel {
            heading: "slosh".into(),
            materials: vec![ParticleMaterial {
                name: "Water".into(),
                density: 1000.0,
                law: ParticleLaw::Fluid {
                    bulk: 2e5,
                    viscosity: 1.0,
                },
                thermal: None,
                flow_tables: Default::default(),
            }],
            bodies: vec![MaterialBody {
                name: "Block".into(),
                material: 0,
                shape: TriangleSurface::cuboid([0.0; 3], [0.3, 0.6, 0.6]),
                velocity: [0.0; 3],
                temperature: 0.0,
                gravity: [0.0; 3],
            }],
            domain: Domain {
                min: [0.0; 3],
                max: [0.6; 3],
                cell: 0.02,
                ..Default::default()
            },
            gravity: [0.0, 0.0, -9.81],
            control: ParticleControl {
                // Minutes of computing.
                end_time: 100.0,
                frames: 2,
                ..Default::default()
            },
            ..Default::default()
        };
        let prepared = PreparedParticles {
            text: describe(&model),
            model,
            warnings: Vec::new(),
            steps: Vec::new(),
            history: HistoryRequest::default(),
        };
        let mut run = Run::start_particles(1, &dir, "slosh", prepared).unwrap();
        std::thread::sleep(std::time::Duration::from_millis(500));
        let output = std::sync::Arc::clone(&run.output);
        run.kill();
        let stopped = std::time::Instant::now();
        let said =
            || (output.lock().unwrap().iter()).any(|line| line.contains("the run was stopped"));
        while !said() && stopped.elapsed().as_secs_f64() < 20.0 {
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        assert!(said(), "the solver went on after the stop");
        let _ = std::fs::remove_dir_all(dir);
    }
}
