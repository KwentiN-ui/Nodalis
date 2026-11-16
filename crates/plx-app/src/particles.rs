//! The window of a Particle Analysis block: which parts of the linked geometry are filled
//! with material and which are walls, the settings of the run, and the particles of its
//! results over the time, coloured by a field.
//!
//! The block keeps the abstract setup ([`ParticleSetup`]); the solver's input
//! ([`ir::ParticleModel`]) is made from it, the triangulated geometry and the linked
//! materials only when the run starts ([`model_of`]), in a thread of its own like the
//! triangulation of the geometry.
//!
//! The particles are drawn by the window itself as small squares sorted by depth, the
//! walls as their feature edges: a first display, without a renderer of its own (concept,
//! section 3.4: points with field colours first).

use std::sync::mpsc::{Receiver, channel};
use std::time::{Duration, Instant};

use egui::{Color32, Ui};
use plx_io::blob::Blob;
use plx_io::project::BoundFile;
use plx_mesh::Fnv;
use plx_model::particle::{
    BoxSide, Flow, PartRole, ParticleBody, ParticleSetup, SIDE_NAMES, WallLaw,
};
use plx_model::{Geometry, Material, Quantity, UnitSystem};
use plx_results::particles::ParticleRun;
use plxs_core::particles::{self as ir, PARTICLE_FIELDS, TriangleSurface};

use crate::numeric;

/// The name of the results file a block keeps.
const RESULTS_NAME: &str = "particles.ndlr";

/// A part of the geometry as the window knows it: its name and its closed surface (solids
/// only; faces and edges cannot hold material or be walls).
#[derive(Clone, Debug)]
pub struct PartSurface {
    pub name: String,
    pub surface: TriangleSurface,
}

/// What the links of the block bring.
pub struct Inputs<'a> {
    pub geometry: Option<&'a Geometry>,
    pub materials: &'a [Material],
    pub units: UnitSystem,
    /// Their fingerprint, see [`input_key`].
    pub key: u64,
}

/// Fingerprint of the linked geometry and materials.
pub fn input_key(geometry: Option<&Geometry>, materials: &[Material]) -> u64 {
    let mut hash = Fnv::new();
    if let Some(g) = geometry {
        hash.add(&geometry_key(g).to_le_bytes());
    }
    for m in materials {
        hash.add_debug(&(&m.name, m.density, m.small_strain_elastic()));
    }
    hash.finish()
}

/// Fingerprint of what a run is computed from: the setup and the inputs ([`input_key`]).
pub fn fingerprint(setup: &ParticleSetup, key: u64) -> u64 {
    let mut hash = Fnv::new();
    hash.add_str(&ron::to_string(setup).unwrap_or_default());
    hash.add(&key.to_le_bytes());
    hash.finish()
}

/// Fingerprint of the shapes of a geometry.
fn geometry_key(geometry: &Geometry) -> u64 {
    let mut hash = Fnv::new();
    hash.add_str(&geometry.brep);
    hash.add_debug(&geometry.suppressed);
    hash.finish()
}

/// The closed surfaces of the solid parts of a geometry, by name, without the suppressed
/// ones: the triangulation the display uses.
pub fn surfaces_of(geometry: &Geometry) -> Result<Vec<PartSurface>, String> {
    let display = plx_mesher::tessellate(geometry).map_err(|e| e.to_string())?;
    let mesh = &display.mesh;
    let mut out = Vec::new();
    for (part, dim) in mesh.parts.iter().zip(&display.part_dimensions) {
        if *dim != 3 || geometry.suppressed.contains(&part.name) {
            continue;
        }
        let mut surface = TriangleSurface::default();
        let mut index = std::collections::HashMap::new();
        for id in &part.elements {
            let Some(element) = mesh.element(*id) else {
                continue;
            };
            if element.shape != plx_mesh::ElementShape::Tri3 {
                continue;
            }
            let mut corners = [0u32; 3];
            for (corner, node) in corners.iter_mut().zip(&element.nodes) {
                let next = surface.points.len() as u32;
                *corner = *index.entry(*node).or_insert_with(|| {
                    surface.points.push(mesh.node(*node).unwrap_or_default());
                    next
                });
            }
            surface.triangles.push(corners);
        }
        if !surface.triangles.is_empty() {
            out.push(PartSurface {
                name: part.name.clone(),
                surface,
            });
        }
    }
    Ok(out)
}

/// The solver's input with the walls by name and the names of the bodies.
pub type SolverInput = (
    ir::ParticleModel,
    Vec<(String, TriangleSurface)>,
    Vec<String>,
);

/// The solver's input made from the setup, the surfaces of the parts and the materials:
/// moduli from the elastic constants of each body's material, the box around the bodies
/// and walls unless the setup gives one. Also returns the names of the walls and of the
/// bodies in the solver's order.
pub fn model_of(
    setup: &ParticleSetup,
    parts: &[PartSurface],
    materials: &[Material],
) -> Result<SolverInput, String> {
    if setup.bodies.is_empty() {
        return Err("No part is filled with material: give a part the role Material.".into());
    }
    if setup.cell.is_nan() || setup.cell <= 0.0 {
        return Err("The cell size must be positive.".into());
    }
    let surface = |part: &str| {
        (parts.iter().find(|p| p.name == part))
            .map(|p| p.surface.clone())
            .ok_or_else(|| format!("Part {part} is not a solid of the linked geometry."))
    };
    let mut model = ir::ParticleModel {
        heading: String::new(),
        gravity: setup.gravity,
        ..Default::default()
    };
    let mut names = Vec::new();
    for body in &setup.bodies {
        let material = (materials.iter().find(|m| m.name == body.material)).ok_or_else(|| {
            format!(
                "Part {}: the material {} is not among the linked materials.",
                body.part, body.material
            )
        })?;
        let density = material
            .density
            .filter(|d| *d > 0.0)
            .ok_or_else(|| format!("Material {} has no density.", material.name))?;
        let elastic = material
            .small_strain_elastic()
            .ok_or_else(|| format!("Material {} has no elastic constants.", material.name))?;
        let bulk = elastic.young / (3.0 * (1.0 - 2.0 * elastic.poisson));
        let shear = elastic.young / (2.0 * (1.0 + elastic.poisson));
        let law = match body.flow {
            Flow::Elastic => ir::ParticleLaw::Elastic {
                young: elastic.young,
                poisson: elastic.poisson,
            },
            Flow::Paste {
                yield_stress,
                consistency,
                exponent,
            } => ir::ParticleLaw::ViscoPlastic {
                shear,
                bulk,
                yield_stress,
                consistency,
                exponent,
            },
            Flow::Fluid { viscosity } => ir::ParticleLaw::Fluid { bulk, viscosity },
        };
        let index = model.materials.len();
        model.materials.push(ir::ParticleMaterial {
            name: format!("{} ({})", material.name, body.part),
            density,
            law,
        });
        model.bodies.push(ir::MaterialBody {
            name: body.part.clone(),
            material: index,
            shape: surface(&body.part)?,
            velocity: body.velocity,
        });
        names.push(body.part.clone());
    }
    let mut walls = Vec::new();
    for wall in &setup.walls {
        let shape = surface(&wall.part)?;
        walls.push((wall.part.clone(), shape.clone()));
        model.walls.push(ir::RigidWall {
            name: wall.part.clone(),
            shape,
            law: wall_law(wall.law),
            motion: ir::RigidMotion::default(),
        });
    }
    let (min, max) = match setup.bounds {
        Some([lo, hi]) => (lo, hi),
        None => {
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            let shapes =
                (model.bodies.iter().map(|b| &b.shape)).chain(model.walls.iter().map(|w| &w.shape));
            for shape in shapes {
                if let Some((a, b)) = shape.bounds() {
                    for k in 0..3 {
                        lo[k] = lo[k].min(a[k]);
                        hi[k] = hi[k].max(b[k]);
                    }
                }
            }
            (lo, hi)
        }
    };
    let sides = setup.sides.map(|s| match s {
        BoxSide::Slip => ir::Side::Wall(ir::WallLaw::Slip),
        BoxSide::Stick => ir::Side::Wall(ir::WallLaw::Stick),
        BoxSide::Open => ir::Side::Open,
    });
    model.domain = ir::Domain {
        min,
        max,
        cell: setup.cell,
        sides,
        periodic: [false; 3],
    };
    let cells: f64 = (0..3)
        .map(|k| ((max[k] - min[k]) / setup.cell).ceil() + 5.0)
        .product();
    if cells > 5e7 {
        return Err(format!(
            "The box would take {cells:.2e} cells; choose a larger cell size or a smaller box."
        ));
    }
    model.control = ir::ParticleControl {
        end_time: setup.end_time,
        frames: setup.frames.max(1),
        sound_speed: setup.sound_speed,
        ..Default::default()
    };
    Ok((model, walls, names))
}

fn wall_law(law: WallLaw) -> ir::WallLaw {
    match law {
        WallLaw::Stick => ir::WallLaw::Stick,
        WallLaw::Slip => ir::WallLaw::Slip,
        WallLaw::Friction(friction) => ir::WallLaw::Coulomb { friction },
    }
}

/// What the thread of a run tells.
enum Message {
    Status(String, f32),
    Done(Box<Result<ParticleRun, String>>),
}

/// A run in its thread.
struct Job {
    rx: Receiver<Message>,
    cancel: plxs_core::CancelToken,
    status: (String, f32),
    started: Instant,
    fingerprint: u64,
    /// Only the particles at the start, not a run over the time.
    fill_only: bool,
}

/// The triangulation of the linked geometry, made in a thread of its own.
enum Surfaces {
    None,
    Making(u64, Receiver<Result<Vec<PartSurface>, String>>),
    Ready(u64, Result<Vec<PartSurface>, String>),
}

/// The 3D view: an orbit about a point, orthographic.
#[derive(Clone, Copy, Debug)]
struct View {
    yaw: f32,
    pitch: f32,
    center: [f32; 3],
    /// Pixels per unit length.
    zoom: f32,
    /// The box the view was fitted to; a new one fits the view again.
    fitted: Option<[[f32; 3]; 2]>,
}

impl Default for View {
    fn default() -> Self {
        Self {
            yaw: 0.6,
            pitch: 0.5,
            center: [0.0; 3],
            zoom: 1.0,
            fitted: None,
        }
    }
}

impl View {
    /// The axes of the screen in model coordinates: right, up, toward the viewer. The model
    /// is shown with z up.
    fn axes(&self) -> [[f32; 3]; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        let right = [cy, sy, 0.0];
        let toward = [sy * cp, -cy * cp, sp];
        let up = [-sy * sp, cy * sp, cp];
        [right, up, toward]
    }

    fn fit(&mut self, lo: [f32; 3], hi: [f32; 3], rect: egui::Rect) {
        self.center = std::array::from_fn(|k| 0.5 * (lo[k] + hi[k]));
        let size = (0..3)
            .map(|k| hi[k] - lo[k])
            .fold(0.0f32, f32::max)
            .max(1e-9);
        self.zoom = 0.7 * rect.width().min(rect.height()) / size;
        self.fitted = Some([lo, hi]);
    }

    /// Screen position and depth (larger is nearer) of a point.
    fn project(&self, axes: &[[f32; 3]; 3], rect: egui::Rect, p: [f32; 3]) -> (egui::Pos2, f32) {
        let d = [
            p[0] - self.center[0],
            p[1] - self.center[1],
            p[2] - self.center[2],
        ];
        let dot = |a: [f32; 3]| a[0] * d[0] + a[1] * d[1] + a[2] * d[2];
        let c = rect.center();
        (
            egui::pos2(
                c.x + self.zoom * dot(axes[0]),
                c.y - self.zoom * dot(axes[1]),
            ),
            dot(axes[2]),
        )
    }
}

/// The field the particles are coloured by: the speed, or one of [`PARTICLE_FIELDS`].
const SPEED: usize = usize::MAX;

/// The window of a Particle Analysis block.
pub struct ParticleWindow {
    pub visible: bool,
    pub setup: ParticleSetup,
    run: Option<ParticleRun>,
    /// The results as the project keeps them.
    stored: Option<BoundFile>,
    /// Fingerprint of what the results were computed from.
    fingerprint: Option<u64>,
    /// The stored results were not read yet.
    unread: bool,
    job: Option<Job>,
    surfaces: Surfaces,
    log: Vec<String>,
    view: View,
    frame: usize,
    field: usize,
    playing: Option<Instant>,
    /// Feature edges of the walls of the run, per wall.
    wall_edges: Vec<Vec<[[f32; 3]; 2]>>,
}

impl ParticleWindow {
    pub fn new() -> Self {
        Self::saved(ParticleSetup::default(), None, None)
    }

    /// The window of a block as the project keeps it.
    pub fn saved(
        setup: ParticleSetup,
        results: Option<BoundFile>,
        fingerprint: Option<u64>,
    ) -> Self {
        Self {
            visible: false,
            setup,
            run: None,
            unread: results.is_some(),
            stored: results,
            fingerprint,
            job: None,
            surfaces: Surfaces::None,
            log: Vec::new(),
            view: View::default(),
            frame: 0,
            field: SPEED,
            playing: None,
            wall_edges: Vec::new(),
        }
    }

    pub fn results(&self) -> Option<&BoundFile> {
        self.stored.as_ref()
    }

    pub fn fingerprint(&self) -> Option<u64> {
        self.fingerprint
    }

    pub fn is_running(&self) -> bool {
        self.job.as_ref().is_some_and(|j| !j.fill_only)
    }

    /// Whether a run or the triangulation of the geometry is going on.
    pub fn is_busy(&self) -> bool {
        self.job.is_some() || matches!(self.surfaces, Surfaces::Making(..))
    }

    pub fn has_results(&self) -> bool {
        self.stored.is_some()
    }

    /// Lets the window read its stored results and take a finished run.
    pub fn poll(&mut self) {
        if self.unread {
            self.unread = false;
            if let Some(file) = &self.stored {
                match file
                    .data
                    .bytes()
                    .map_err(|e| e.to_string())
                    .and_then(|b| ParticleRun::from_bytes(&b))
                {
                    Ok(run) => self.take_run(run),
                    Err(e) => self.log.push(format!("The results could not be read: {e}")),
                }
            }
        }
        let Some(job) = &mut self.job else {
            return;
        };
        let mut done = None;
        while let Ok(message) = job.rx.try_recv() {
            match message {
                Message::Status(text, fraction) => job.status = (text, fraction),
                Message::Done(result) => done = Some(*result),
            }
        }
        let Some(result) = done else {
            return;
        };
        let job = self.job.take().expect("a job");
        match result {
            Ok(mut run) => {
                let report = &run.results.report;
                self.log.push(format!(
                    "{}: {} particles, {} cells ({} x {} x {}), {} steps of {:.3e} to {:.3e} s, \
                     {:.1} s",
                    if job.fill_only { "Filled" } else { "Solved" },
                    report.particles,
                    report.cells.iter().product::<usize>(),
                    report.cells[0],
                    report.cells[1],
                    report.cells[2],
                    report.steps,
                    report.time_step.1,
                    report.time_step.0,
                    job.started.elapsed().as_secs_f64()
                ));
                if report.sound_speed_factor > 1.0 {
                    self.log.push(format!(
                        "Waves slowed by a factor {:.1}; largest Mach number {:.3}.",
                        report.sound_speed_factor, report.max_mach
                    ));
                }
                if let Some(e) = run.results.history.balance_error.last() {
                    self.log.push(format!(
                        "Energy balance error at the end: {:.2} %",
                        100.0 * e
                    ));
                }
                self.log.extend(report.notes.iter().cloned());
                if report.stopped {
                    self.log.push("Stopped before the end time.".into());
                }
                if !job.fill_only {
                    let bytes = run.to_bytes();
                    self.stored = Some(BoundFile {
                        name: RESULTS_NAME.into(),
                        size: bytes.len() as u64,
                        data: Blob::from_bytes(bytes),
                    });
                    self.fingerprint = Some(job.fingerprint);
                }
                self.take_run(run);
                self.frame = if job.fill_only {
                    0
                } else {
                    self.frame_count().saturating_sub(1)
                };
            }
            Err(e) => self.log.push(e),
        }
    }

    fn take_run(&mut self, run: ParticleRun) {
        self.wall_edges = run.walls.iter().map(|(_, s)| feature_edges(s)).collect();
        self.view.fitted = None;
        self.run = Some(run);
        self.frame = self.frame.min(self.frame_count().saturating_sub(1));
    }

    fn frame_count(&self) -> usize {
        self.run.as_ref().map_or(0, |r| r.results.frames.len())
    }

    /// Starts a run: the whole run, or only filling the bodies with particles.
    pub fn start(&mut self, inputs: &Inputs, fill_only: bool) {
        if self.job.is_some() {
            return;
        }
        let Some(geometry) = inputs.geometry.cloned() else {
            self.log.push("No geometry is linked to the block.".into());
            return;
        };
        let mut setup = self.setup.clone();
        if fill_only {
            setup.end_time = 0.0;
        }
        let materials = inputs.materials.to_vec();
        let fingerprint = fingerprint(&self.setup, inputs.key);
        let (tx, rx) = channel();
        let cancel = plxs_core::CancelToken::new();
        let token = cancel.clone();
        let spawned = std::thread::Builder::new()
            .name("particles".into())
            .spawn(move || {
                let _ = tx.send(Message::Status("Triangulating the geometry".into(), 0.0));
                let result = surfaces_of(&geometry)
                    .and_then(|parts| model_of(&setup, &parts, &materials))
                    .and_then(|(model, walls, bodies)| {
                        let progress_tx = tx.clone();
                        let mut progress = move |s: plxs_particles::Status| {
                            let fraction = if s.end_time > 0.0 {
                                s.time / s.end_time
                            } else {
                                1.0
                            };
                            let _ = progress_tx.send(Message::Status(
                                format!(
                                    "{:.3} of {} s, {} particles",
                                    s.time, s.end_time, s.particles
                                ),
                                fraction as f32,
                            ));
                        };
                        let results =
                            token.scope(|| plxs_particles::solve(&model, &mut progress))?;
                        Ok(ParticleRun {
                            results,
                            walls,
                            bodies,
                        })
                    });
                let _ = tx.send(Message::Done(Box::new(result)));
            });
        match spawned {
            Ok(_) => {
                self.job = Some(Job {
                    rx,
                    cancel,
                    status: ("Starting".into(), 0.0),
                    started: Instant::now(),
                    fingerprint,
                    fill_only,
                });
            }
            Err(e) => self.log.push(format!("The run could not start: {e}")),
        }
    }

    pub fn cancel(&mut self) {
        if let Some(job) = &self.job {
            job.cancel.cancel();
        }
    }

    /// The parts of the linked geometry, triangulated in the background when it changed.
    fn parts(&mut self, geometry: Option<&Geometry>) -> Option<&Result<Vec<PartSurface>, String>> {
        let Some(geometry) = geometry else {
            self.surfaces = Surfaces::None;
            return None;
        };
        let key = geometry_key(geometry);
        let current = match &self.surfaces {
            Surfaces::Making(k, _) | Surfaces::Ready(k, _) => Some(*k),
            Surfaces::None => None,
        };
        if current != Some(key) {
            let (tx, rx) = channel();
            let geometry = geometry.clone();
            let _ = std::thread::Builder::new()
                .name("particle parts".into())
                .spawn(move || {
                    let _ = tx.send(surfaces_of(&geometry));
                });
            self.surfaces = Surfaces::Making(key, rx);
        }
        if let Surfaces::Making(k, rx) = &self.surfaces
            && let Ok(result) = rx.try_recv()
        {
            self.surfaces = Surfaces::Ready(*k, result);
        }
        match &self.surfaces {
            Surfaces::Ready(_, result) => Some(result),
            _ => None,
        }
    }

    /// Shows the window; returns false when it was closed.
    pub fn show(&mut self, ctx: &egui::Context, (title, id): (&str, u64), inputs: &Inputs) -> bool {
        self.poll();
        let mut open = true;
        let mut close = false;
        egui::Window::new(format!("Particle analysis: {title}"))
            .id(crate::ids::window(ctx, ("particles", id)))
            .open(&mut open)
            .collapsible(false)
            .resizable(true)
            .default_size([1180.0, 760.0])
            .min_size([700.0, 460.0])
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                close = crate::dialog::close_bar(ui, "particles");
                egui::Panel::left(egui::Id::new(("particle setup", id)))
                    .resizable(true)
                    .default_size(400.0)
                    .size_range(300.0..=700.0)
                    .show(ui, |ui| {
                        egui::ScrollArea::vertical().show(ui, |ui| self.side(ui, inputs));
                    });
                egui::Panel::bottom(egui::Id::new(("particle time", id)))
                    .frame(egui::Frame::new().inner_margin(egui::Margin::symmetric(8, 4)))
                    .show(ui, |ui| self.timeline(ui));
                self.viewport(ui, inputs);
            });
        if self.job.is_some()
            || self.playing.is_some()
            || matches!(self.surfaces, Surfaces::Making(..))
        {
            ctx.request_repaint_after(Duration::from_millis(100));
        }
        open && !close
    }

    /// The setup, the buttons of the run and its output.
    fn side(&mut self, ui: &mut Ui, inputs: &Inputs) {
        let units = inputs.units;
        let running = self.job.is_some();
        ui.horizontal(|ui| {
            let ready = inputs.geometry.is_some() && !self.setup.bodies.is_empty();
            if ui
                .add_enabled(ready && !running, egui::Button::new("Solve"))
                .on_hover_text("Fills the bodies with particles and computes them over the time.")
                .clicked()
            {
                self.start(inputs, false);
            }
            if ui
                .add_enabled(ready && !running, egui::Button::new("Fill"))
                .on_hover_text(
                    "Fills the bodies with particles and shows them, without computing: to \
                     check the resolution and the parts.",
                )
                .clicked()
            {
                self.start(inputs, true);
            }
            if ui.add_enabled(running, egui::Button::new("Stop")).clicked() {
                self.cancel();
            }
        });
        if let Some(job) = &self.job {
            ui.add(egui::ProgressBar::new(job.status.1).text(job.status.0.clone()));
        } else {
            let state = match (self.fingerprint, self.stored.is_some()) {
                (_, false) => "Not solved".to_owned(),
                (Some(f), true) if f == fingerprint(&self.setup, inputs.key) => {
                    "Results up to date".to_owned()
                }
                _ => "Results out of date: the setup or its inputs changed".to_owned(),
            };
            ui.weak(state);
        }
        ui.separator();

        ui.strong("Parts");
        let materials: Vec<String> = inputs.materials.iter().map(|m| m.name.clone()).collect();
        let parts = self.parts(inputs.geometry).map(|r| {
            r.as_ref()
                .map(|p| p.iter().map(|s| s.name.clone()).collect::<Vec<_>>())
                .map_err(Clone::clone)
        });
        match parts {
            None if inputs.geometry.is_none() => {
                ui.weak("Link geometry to the Geometry row of the block.");
            }
            None => {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.weak("Reading the parts of the geometry");
                });
            }
            Some(Err(e)) => {
                ui.colored_label(ui.visuals().error_fg_color, e);
            }
            Some(Ok(names)) => {
                if materials.is_empty() {
                    ui.weak("Link materials to the Materials row: bodies take their density and elastic constants.");
                }
                for name in &names {
                    self.part_row(ui, name, &materials, units);
                }
            }
        }
        ui.separator();

        ui.strong("Resolution and time");
        egui::Grid::new("particle settings")
            .num_columns(2)
            .show(ui, |ui| {
                ui.label("Cell size").on_hover_text(
                    "The size of a cell of the background grid. Particles start two to a cell in \
                 each direction; walls and openings should span several cells.",
                );
                ui.add(numeric::scaled(
                    &mut self.setup.cell,
                    units,
                    Quantity::Length,
                    1e-3,
                ));
                ui.end_row();
                ui.label("End time");
                ui.add(numeric::scaled(
                    &mut self.setup.end_time,
                    units,
                    Quantity::Time,
                    0.1,
                ));
                ui.end_row();
                ui.label("Frames");
                ui.add(numeric::count(&mut self.setup.frames).range(1..=10000));
                ui.end_row();
                ui.label("Gravity");
                ui.horizontal(|ui| {
                    for g in &mut self.setup.gravity {
                        ui.add(numeric::without_unit(g, units, Quantity::Acceleration).speed(0.01));
                    }
                    ui.label(units.unit(Quantity::Acceleration));
                });
                ui.end_row();
                let mut slowed = self.setup.sound_speed.is_some();
                ui.checkbox(&mut slowed, "Slow waves to").on_hover_text(
                    "Lowers the bulk modulus so that the elastic waves run at this speed, which \
                 makes the time step larger by the same factor. The result stays the same \
                 while the material moves much slower than the waves: the output gives the \
                 largest ratio (Mach number), which should stay below 0.1. Like mass scaling.",
                );
                if slowed && self.setup.sound_speed.is_none() {
                    self.setup.sound_speed = Some(20.0);
                } else if !slowed {
                    self.setup.sound_speed = None;
                }
                if let Some(speed) = &mut self.setup.sound_speed {
                    ui.add(numeric::scaled(speed, units, Quantity::Velocity, 1.0));
                } else {
                    ui.weak("the material's own");
                }
                ui.end_row();
            });
        ui.separator();

        ui.strong("Box");
        let mut automatic = self.setup.bounds.is_none();
        if ui
            .checkbox(&mut automatic, "Around the bodies and walls")
            .on_hover_text("The material moves inside this box; its sides are walls or open.")
            .changed()
        {
            self.setup.bounds = if automatic {
                None
            } else {
                self.box_of_parts().or(Some([[0.0; 3], [1.0; 3]]))
            };
        }
        if let Some(bounds) = &mut self.setup.bounds {
            egui::Grid::new("particle box")
                .num_columns(4)
                .show(ui, |ui| {
                    for (row, label) in ["Min", "Max"].iter().enumerate() {
                        ui.label(*label);
                        for value in &mut bounds[row] {
                            ui.add(
                                numeric::without_unit(value, units, Quantity::Length).speed(0.001),
                            );
                        }
                        ui.end_row();
                    }
                });
        }
        egui::Grid::new("particle sides")
            .num_columns(2)
            .show(ui, |ui| {
                for (side, name) in self.setup.sides.iter_mut().zip(SIDE_NAMES) {
                    ui.label(name);
                    egui::ComboBox::from_id_salt(("particle side", name))
                        .selected_text(side.label())
                        .show_ui(ui, |ui| {
                            for choice in BoxSide::ALL {
                                ui.selectable_value(side, choice, choice.label());
                            }
                        });
                    ui.end_row();
                }
            });
        ui.separator();

        if let Some(run) = &self.run
            && !run.results.history.time.is_empty()
        {
            egui::CollapsingHeader::new("History")
                .default_open(true)
                .show(ui, |ui| history_plots(ui, run));
            ui.separator();
        }
        ui.strong("Output");
        for line in &self.log {
            ui.label(line);
        }
    }

    /// The bounding box of the parts as last triangulated.
    fn box_of_parts(&self) -> Option<[[f64; 3]; 2]> {
        let Surfaces::Ready(_, Ok(parts)) = &self.surfaces else {
            return None;
        };
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in parts {
            let used = self.setup.role(&p.name) != PartRole::None;
            if let (true, Some((a, b))) = (used, p.surface.bounds()) {
                for k in 0..3 {
                    lo[k] = lo[k].min(a[k]);
                    hi[k] = hi[k].max(b[k]);
                }
            }
        }
        lo[0].is_finite().then_some([lo, hi])
    }

    /// The row of a part: its role and what goes with it.
    fn part_row(&mut self, ui: &mut Ui, name: &str, materials: &[String], units: UnitSystem) {
        let role = self.setup.role(name);
        let mut changed = None;
        ui.horizontal(|ui| {
            ui.label(name);
            let current = match role {
                PartRole::Body(_) => "Material",
                PartRole::Wall(_) => "Wall",
                PartRole::None => "Not used",
            };
            egui::ComboBox::from_id_salt(("particle role", name))
                .selected_text(current)
                .show_ui(ui, |ui| {
                    if ui
                        .selectable_label(current == "Not used", "Not used")
                        .clicked()
                    {
                        changed = Some(PartRole::None);
                    }
                    if ui
                        .selectable_label(current == "Material", "Material")
                        .on_hover_text("Filled with particles of a material at the start")
                        .clicked()
                        && current != "Material"
                    {
                        changed = Some(PartRole::Body(ParticleBody {
                            part: name.to_owned(),
                            material: materials.first().cloned().unwrap_or_default(),
                            flow: Flow::of_index(1),
                            velocity: [0.0; 3],
                        }));
                    }
                    if ui
                        .selectable_label(current == "Wall", "Wall")
                        .on_hover_text("A rigid body the material meets")
                        .clicked()
                        && current != "Wall"
                    {
                        changed = Some(PartRole::Wall(WallLaw::Stick));
                    }
                });
        });
        if let Some(role) = changed {
            self.setup.set_role(name, role);
        }
        if let Some(body) = self.setup.bodies.iter_mut().find(|b| b.part == name) {
            ui.indent(("particle body", name), |ui| {
                body_fields(ui, body, materials, units)
            });
        }
        if let Some(wall) = self.setup.walls.iter_mut().find(|w| w.part == name) {
            ui.indent(("particle wall", name), |ui| {
                ui.horizontal(|ui| {
                    ui.label("Wall law");
                    let mut index = wall.law.index();
                    egui::ComboBox::from_id_salt(("particle wall law", name))
                        .selected_text(WallLaw::NAMES[index])
                        .show_ui(ui, |ui| {
                            for (i, label) in WallLaw::NAMES.iter().enumerate() {
                                ui.selectable_value(&mut index, i, *label);
                            }
                        });
                    if index != wall.law.index() {
                        wall.law = WallLaw::of_index(index);
                    }
                    if let WallLaw::Friction(mu) = &mut wall.law {
                        ui.label("coefficient");
                        ui.add(numeric::drag_value(mu).speed(0.01).range(0.0..=10.0));
                    }
                });
            });
        }
    }

    /// The frame slider, the field and the play button.
    fn timeline(&mut self, ui: &mut Ui) {
        let count = self.frame_count();
        ui.horizontal(|ui| {
            ui.label("Colour by");
            let label = |f: usize| {
                if f == SPEED {
                    "Speed"
                } else {
                    PARTICLE_FIELDS[f]
                }
            };
            egui::ComboBox::from_id_salt("particle field")
                .selected_text(label(self.field))
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.field, SPEED, "Speed");
                    for (i, name) in PARTICLE_FIELDS.iter().enumerate() {
                        ui.selectable_value(&mut self.field, i, *name);
                    }
                });
            ui.separator();
            let playing = self.playing.is_some();
            if ui
                .add_enabled(
                    count > 1,
                    egui::Button::new(if playing { "Pause" } else { "Play" }),
                )
                .clicked()
            {
                self.playing = if playing { None } else { Some(Instant::now()) };
                if !playing && self.frame + 1 >= count {
                    self.frame = 0;
                }
            }
            if let Some(since) = self.playing
                && since.elapsed() > Duration::from_millis(80)
            {
                self.playing = Some(Instant::now());
                if self.frame + 1 < count {
                    self.frame += 1;
                } else {
                    self.playing = None;
                }
            }
            if count > 0 {
                let mut frame = self.frame;
                ui.add(egui::Slider::new(&mut frame, 0..=count - 1).text("frame"));
                self.frame = frame;
                if let Some(f) = self
                    .run
                    .as_ref()
                    .and_then(|r| r.results.frames.get(self.frame))
                {
                    ui.label(format!(
                        "t = {} s, {} particles",
                        numeric::format_physical(f.time),
                        f.positions.len()
                    ));
                }
            } else {
                ui.weak("No results: Fill shows the particles, Solve computes them.");
            }
        });
    }

    /// The particles of the frame and the walls.
    fn viewport(&mut self, ui: &mut Ui, inputs: &Inputs) {
        let (rect, response) =
            ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, {
            let [r, g, b, _] = crate::style::viewport_background()[0].map(|v| (v * 255.0) as u8);
            egui::Color32::from_rgb(r, g, b)
        });
        let frame = self
            .run
            .as_ref()
            .and_then(|r| r.results.frames.get(self.frame));
        // Fit the view to what there is to see.
        let bounds = frame
            .and_then(|f| bounds_of(f.positions.iter().copied()))
            .map(|(lo, hi)| {
                let walls = self.wall_edges.iter().flatten().flatten().copied();
                bounds_of(walls).map_or((lo, hi), |(a, b)| {
                    (
                        std::array::from_fn(|k| lo[k].min(a[k])),
                        std::array::from_fn(|k| hi[k].max(b[k])),
                    )
                })
            });
        let preview = if frame.is_none() {
            match &self.surfaces {
                Surfaces::Ready(_, Ok(parts)) => parts
                    .iter()
                    .filter(|p| {
                        self.setup.role(&p.name) != PartRole::None || self.setup.bodies.is_empty()
                    })
                    .map(|p| feature_edges(&p.surface))
                    .collect(),
                _ => Vec::new(),
            }
        } else {
            Vec::new()
        };
        let bounds = bounds.or_else(|| bounds_of(preview.iter().flatten().flatten().copied()));
        if let Some((lo, hi)) = bounds
            && self.view.fitted.is_none()
        {
            self.view.fit(lo, hi, rect);
        }
        if response.dragged_by(egui::PointerButton::Primary) {
            let d = response.drag_delta();
            self.view.yaw -= 0.01 * d.x;
            self.view.pitch = (self.view.pitch + 0.01 * d.y).clamp(-1.55, 1.55);
        }
        if response.dragged_by(egui::PointerButton::Secondary)
            || response.dragged_by(egui::PointerButton::Middle)
        {
            let d = response.drag_delta();
            let axes = self.view.axes();
            for (k, center) in self.view.center.iter_mut().enumerate() {
                *center -= (d.x * axes[0][k] - d.y * axes[1][k]) / self.view.zoom;
            }
        }
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.view.zoom *= (scroll * 0.002).exp();
            }
        }
        if response.double_clicked()
            && let Some((lo, hi)) = bounds
        {
            self.view.fit(lo, hi, rect);
        }
        let axes = self.view.axes();
        let ink = ui.visuals().weak_text_color();
        let edges = if frame.is_some() {
            &self.wall_edges
        } else {
            &preview
        };
        for edge in edges.iter().flatten() {
            let a = self.view.project(&axes, rect, edge[0]).0;
            let b = self.view.project(&axes, rect, edge[1]).0;
            painter.line_segment([a, b], egui::Stroke::new(1.0, ink));
        }
        if let Some(frame) = frame {
            let values: Vec<f32> = if self.field == SPEED {
                frame
                    .velocities
                    .iter()
                    .map(|v| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt())
                    .collect()
            } else {
                frame.fields.get(self.field).cloned().unwrap_or_default()
            };
            let (lo, hi) = values
                .iter()
                .fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), v| {
                    (a.min(*v), b.max(*v))
                });
            let scale = plx_model::ColorScale::default();
            let mut order: Vec<(f32, usize)> = (frame.positions.iter().enumerate())
                .map(|(i, p)| (self.view.project(&axes, rect, *p).1, i))
                .collect();
            order.sort_by(|a, b| a.0.total_cmp(&b.0));
            let cell = self.setup.cell as f32;
            let half = (0.5 * cell * self.view.zoom).clamp(1.0, 12.0) * 0.5;
            let mut mesh = egui::Mesh::default();
            for (_, i) in order {
                let (at, _) = self.view.project(&axes, rect, frame.positions[i]);
                if !rect.contains(at) {
                    continue;
                }
                let t = if hi > lo {
                    (values.get(i).copied().unwrap_or(lo) - lo) / (hi - lo)
                } else {
                    0.5
                };
                let c = plx_render::contour::spectrum_color(&scale, f64::from(t));
                let color = Color32::from_rgb(
                    (c[0] * 255.0) as u8,
                    (c[1] * 255.0) as u8,
                    (c[2] * 255.0) as u8,
                );
                mesh.add_colored_rect(
                    egui::Rect::from_center_size(at, egui::vec2(2.0 * half, 2.0 * half)),
                    color,
                );
            }
            painter.add(egui::Shape::mesh(mesh));
            let quantity = match self.field {
                SPEED => Some(Quantity::Velocity),
                0 | 1 => Some(Quantity::Pressure),
                3 => Some(Quantity::Frequency),
                _ => None,
            };
            let shown =
                |v: f32| quantity.map_or(f64::from(v), |q| inputs.units.shown(f64::from(v), q));
            let unit = quantity.map_or("", |q| inputs.units.unit(q));
            legend(
                &painter,
                rect,
                &scale,
                (shown(lo), shown(hi), unit),
                ui.visuals().text_color(),
            );
        } else if inputs.geometry.is_some() && preview.is_empty() {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "Give parts a role",
                egui::FontId::proportional(14.0),
                ink,
            );
        }
        response.on_hover_text_at_pointer(
            "Drag to turn, right-drag to move, scroll to zoom, double-click to fit",
        );
    }
}

impl Default for ParticleWindow {
    fn default() -> Self {
        Self::new()
    }
}

/// The fields of a material body: its material and how it flows.
fn body_fields(ui: &mut Ui, body: &mut ParticleBody, materials: &[String], units: UnitSystem) {
    egui::Grid::new(("particle body grid", body.part.clone()))
        .num_columns(2)
        .show(ui, |ui| {
            ui.label("Material");
            egui::ComboBox::from_id_salt(("particle material", body.part.clone()))
                .selected_text(if body.material.is_empty() {
                    "(none)"
                } else {
                    &body.material
                })
                .show_ui(ui, |ui| {
                    for m in materials {
                        ui.selectable_value(&mut body.material, m.clone(), m);
                    }
                });
            ui.end_row();
            ui.label("Flow").on_hover_text(
                "How the material flows beyond its elastic part; density and elastic constants \
             come from the material.",
            );
            let mut index = body.flow.index();
            egui::ComboBox::from_id_salt(("particle flow", body.part.clone()))
                .selected_text(Flow::NAMES[index])
                .show_ui(ui, |ui| {
                    for (i, label) in Flow::NAMES.iter().enumerate() {
                        ui.selectable_value(&mut index, i, *label);
                    }
                });
            if index != body.flow.index() {
                body.flow = Flow::of_index(index);
            }
            ui.end_row();
            match &mut body.flow {
                Flow::Elastic => {}
                Flow::Paste {
                    yield_stress,
                    consistency,
                    exponent,
                } => {
                    ui.label("Yield stress")
                        .on_hover_text("In shear: the paste flows above it.");
                    ui.add(numeric::physical(yield_stress, units, Quantity::Pressure));
                    ui.end_row();
                    ui.label("Consistency").on_hover_text(
                        "k of the flow stress τ_y + k γ̇ⁿ, in Pa·sⁿ (the viscosity for n = 1).",
                    );
                    ui.add(numeric::drag_value(consistency).speed(0.0).suffix(" Pa·sⁿ"));
                    ui.end_row();
                    ui.label("Exponent n").on_hover_text(
                        "Below 1 the paste thins with the shear rate; 1 is Bingham.",
                    );
                    ui.add(numeric::drag_value(exponent).speed(0.01).range(0.05..=2.0));
                    ui.end_row();
                }
                Flow::Fluid { viscosity } => {
                    ui.label("Viscosity");
                    ui.add(numeric::drag_value(viscosity).speed(0.0).suffix(" Pa·s"));
                    ui.end_row();
                }
            }
            ui.label("Initial velocity");
            ui.horizontal(|ui| {
                for v in &mut body.velocity {
                    ui.add(numeric::without_unit(v, units, Quantity::Velocity).speed(0.01));
                }
                ui.label(units.unit(Quantity::Velocity));
            });
            ui.end_row();
        });
}

/// The edges of a surface where its faces meet at an angle (or that border one face only).
fn feature_edges(surface: &TriangleSurface) -> Vec<[[f32; 3]; 2]> {
    use std::collections::HashMap;
    let normal = |t: &[u32; 3]| {
        let p = t.map(|i| surface.points[i as usize]);
        let a = [p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]];
        let b = [p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]];
        let n = [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ];
        let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-300);
        n.map(|x| x / l)
    };
    // Corners by position: the triangulation of the faces has corners of its own per face.
    let key = |i: u32| surface.points[i as usize].map(|x| (x * 1e9).round() as i64);
    type Edge = ([i64; 3], [i64; 3]);
    let mut sides: HashMap<Edge, Vec<[f64; 3]>> = HashMap::new();
    for t in &surface.triangles {
        let n = normal(t);
        for k in 0..3 {
            let (a, b) = (key(t[k]), key(t[(k + 1) % 3]));
            sides
                .entry(if a < b { (a, b) } else { (b, a) })
                .or_default()
                .push(n);
        }
    }
    let point = |k: [i64; 3]| k.map(|x| (x as f64 * 1e-9) as f32);
    (sides.into_iter())
        .filter(|(_, normals)| {
            normals.len() != 2 || {
                let (a, b) = (normals[0], normals[1]);
                a[0] * b[0] + a[1] * b[1] + a[2] * b[2] < 0.94
            }
        })
        .map(|((a, b), _)| [point(a), point(b)])
        .collect()
}

fn bounds_of(points: impl Iterator<Item = [f32; 3]>) -> Option<([f32; 3], [f32; 3])> {
    let mut lo = [f32::INFINITY; 3];
    let mut hi = [f32::NEG_INFINITY; 3];
    for p in points {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    lo[0].is_finite().then_some((lo, hi))
}

/// The colour bar at the right edge with the smallest and largest value.
/// The colour bar of the field, with its range `(lo, hi, unit)` as shown.
fn legend(
    painter: &egui::Painter,
    rect: egui::Rect,
    scale: &plx_model::ColorScale,
    (lo, hi, unit): (f64, f64, &str),
    ink: Color32,
) {
    let bar = egui::Rect::from_min_size(
        rect.right_top() + egui::vec2(-60.0, 20.0),
        egui::vec2(16.0, 200.0),
    );
    let steps = 32;
    for s in 0..steps {
        let t0 = s as f32 / steps as f32;
        let c = plx_render::contour::spectrum_color(scale, f64::from(t0 + 0.5 / steps as f32));
        let color = Color32::from_rgb(
            (c[0] * 255.0) as u8,
            (c[1] * 255.0) as u8,
            (c[2] * 255.0) as u8,
        );
        let y1 = bar.bottom() - t0 * bar.height();
        let y0 = y1 - bar.height() / steps as f32;
        painter.rect_filled(
            egui::Rect::from_x_y_ranges(bar.x_range(), y0..=y1),
            0.0,
            color,
        );
    }
    let font = egui::FontId::proportional(12.0);
    let text = |v: f64| {
        let v = if v == 0.0 {
            0.0
        } else {
            format!("{v:.3e}").parse().unwrap_or(v)
        };
        format!("{} {unit}", numeric::format_physical(v))
            .trim_end()
            .to_owned()
    };
    painter.text(
        bar.right_top() + egui::vec2(-20.0, -4.0),
        egui::Align2::RIGHT_BOTTOM,
        text(hi),
        font.clone(),
        ink,
    );
    painter.text(
        bar.right_bottom() + egui::vec2(-20.0, 4.0),
        egui::Align2::RIGHT_TOP,
        text(lo),
        font,
        ink,
    );
}

/// The energies and the forces on the walls over the time.
fn history_plots(ui: &mut Ui, run: &ParticleRun) {
    use egui_plot::{Legend, Line, Plot, PlotPoints};
    let h = &run.results.history;
    let series = |values: &[f64]| -> PlotPoints {
        h.time.iter().zip(values).map(|(t, v)| [*t, *v]).collect()
    };
    ui.label("Energy [J]");
    Plot::new("particle energies")
        .height(150.0)
        .legend(Legend::default())
        .allow_scroll(false)
        .show(ui, |plot| {
            plot.line(Line::new("Kinetic", series(&h.kinetic)));
            plot.line(Line::new("Potential", series(&h.potential)));
            plot.line(Line::new("Elastic", series(&h.elastic)));
            plot.line(Line::new("Dissipated", series(&h.dissipated)));
            plot.line(Line::new("Wall loss", series(&h.wall_loss)));
        });
    ui.label("Force on the walls [N]");
    Plot::new("particle wall forces")
        .height(150.0)
        .legend(Legend::default())
        .allow_scroll(false)
        .show(ui, |plot| {
            for wall in &h.walls {
                if wall.force.iter().all(|f| *f == [0.0; 3]) {
                    continue;
                }
                let magnitude: Vec<f64> = wall
                    .force
                    .iter()
                    .map(|f| (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt())
                    .collect();
                plot.line(Line::new(wall.name.clone(), series(&magnitude)));
            }
        });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn material(name: &str) -> Material {
        Material {
            name: name.into(),
            density: Some(1000.0),
            elastic: Some(plx_model::Elastic {
                young: 3e4,
                poisson: 0.45,
            }),
            ..Default::default()
        }
    }

    #[test]
    fn model_takes_moduli_and_box() {
        let mut setup = ParticleSetup::default();
        setup.set_role(
            "Block",
            PartRole::Body(ParticleBody {
                part: String::new(),
                material: "Paste".into(),
                flow: Flow::Paste {
                    yield_stress: 100.0,
                    consistency: 5.0,
                    exponent: 1.0,
                },
                velocity: [0.0; 3],
            }),
        );
        setup.set_role("Floor", PartRole::Wall(WallLaw::Friction(0.2)));
        let parts = vec![
            PartSurface {
                name: "Block".into(),
                surface: TriangleSurface::cuboid([0.0, 0.0, 0.1], [0.1, 0.1, 0.2]),
            },
            PartSurface {
                name: "Floor".into(),
                surface: TriangleSurface::cuboid([-0.1, -0.1, 0.0], [0.3, 0.2, 0.1]),
            },
        ];
        let (model, walls, bodies) = model_of(&setup, &parts, &[material("Paste")]).unwrap();
        assert_eq!(bodies, ["Block"]);
        assert_eq!(walls.len(), 1);
        assert_eq!(model.domain.min, [-0.1, -0.1, 0.0]);
        assert_eq!(model.domain.max, [0.3, 0.2, 0.2]);
        let ir::ParticleLaw::ViscoPlastic { shear, bulk, .. } = model.materials[0].law else {
            panic!("a paste");
        };
        assert!((shear - 3e4 / 2.9).abs() < 1e-6);
        assert!((bulk - 3e4 / 0.3).abs() < 1e-6);
        assert_eq!(model.walls[0].law, ir::WallLaw::Coulomb { friction: 0.2 });
        // Missing material.
        assert!(model_of(&setup, &parts, &[material("Steel")]).is_err());
    }
}
