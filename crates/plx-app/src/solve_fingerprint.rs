//! The fingerprints of what a model's solution is computed from, taken from the analysis
//! the model is lowered to ([`plx_io::fingerprint`]): renaming or hiding an item, or adding
//! one that reaches no step, leaves the results up to date. Lowering takes a while on a big
//! model, so the fingerprints are kept by the quick fingerprint of the model itself
//! ([`plx_model::fingerprint::solve_input`]) and made again only when the model changed.
//! What is shown while the user edits takes them with [`fingerprints_soon`], which lowers
//! in the background, so that a change of a large model does not freeze the window.

use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};

use plx_io::fingerprint::Fingerprints;
use plx_mesh::FeMesh;
use plx_model::FeModel;

/// Fingerprints of the last models asked for, by their quick fingerprint.
static KEPT: Mutex<Vec<(u64, Fingerprints)>> = Mutex::new(Vec::new());
const KEPT_MODELS: usize = 16;

/// The fingerprints of the solution of `fe` on `mesh`. A model that cannot be lowered (its
/// analysis cannot be set up) counts by its quick fingerprints.
pub fn fingerprints(mesh: &FeMesh, fe: &FeModel) -> Fingerprints {
    let key = plx_model::fingerprint::solve_input(mesh, fe);
    kept(key).unwrap_or_else(|| made(key, mesh, fe))
}

/// The fingerprints kept for the model with the quick fingerprint `key`.
fn kept(key: u64) -> Option<Fingerprints> {
    (KEPT.lock().ok()).and_then(|kept| kept.iter().find(|(k, _)| *k == key).map(|(_, p)| p.clone()))
}

/// Lowers the model for its fingerprints and keeps them.
fn made(key: u64, mesh: &FeMesh, fe: &FeModel) -> Fingerprints {
    let prints = match plx_io::inp::lower_unvalidated(mesh, fe, "") {
        Ok(lowered) => plx_io::fingerprint::fingerprints(&lowered, fe),
        Err(_) => Fingerprints {
            input: key,
            steps: plx_model::fingerprint::step_inputs(mesh, fe),
        },
    };
    if let Ok(mut kept) = KEPT.lock() {
        if kept.len() >= KEPT_MODELS {
            kept.remove(0);
        }
        kept.push((key, prints.clone()));
    }
    prints
}

/// A model waiting to be lowered in the background, by its quick fingerprint.
struct Job {
    key: u64,
    mesh: FeMesh,
    fe: FeModel,
}

/// The background lowering: the model it lowers now and the one it lowers next. Only the
/// latest model asked for waits, as the models before it are gone by then.
struct Background {
    running: Option<u64>,
    next: Option<Job>,
}

static BACKGROUND: Mutex<Background> = Mutex::new(Background {
    running: None,
    next: None,
});
/// Counts the fingerprints made in the background, see [`generation`].
static GENERATION: AtomicU64 = AtomicU64::new(0);
/// The window to repaint once fingerprints are made in the background.
static REPAINT: Mutex<Option<egui::Context>> = Mutex::new(None);

/// Sets the window that is repainted once fingerprints made in the background are ready.
pub fn repaint_with(ctx: &egui::Context) {
    if let Ok(mut repaint) = REPAINT.lock() {
        *repaint = Some(ctx.clone());
    }
}

/// Changes each time fingerprints made in the background are ready: what waits for them
/// asks [`fingerprints_soon`] again.
pub fn generation() -> u64 {
    GENERATION.load(Ordering::Acquire)
}

/// [`fingerprints`] if they are at hand; otherwise `None`, and the model is lowered in the
/// background: lowering a large model takes seconds, too long for a frame.
pub fn fingerprints_soon(mesh: &FeMesh, fe: &FeModel) -> Option<Fingerprints> {
    let key = plx_model::fingerprint::solve_input(mesh, fe);
    if let Some(prints) = kept(key) {
        return Some(prints);
    }
    let Ok(mut background) = BACKGROUND.lock() else {
        return Some(made(key, mesh, fe));
    };
    let waiting = background.running == Some(key)
        || background.next.as_ref().is_some_and(|job| job.key == key);
    if waiting {
        return None;
    }
    background.next = Some(Job {
        key,
        mesh: mesh.clone(),
        fe: fe.clone(),
    });
    if background.running.is_none() {
        background.running = Some(key);
        let started = std::thread::Builder::new()
            .name("solve fingerprints".into())
            .spawn(lower_in_background);
        if started.is_err() {
            background.running = None;
            background.next = None;
            drop(background);
            return Some(made(key, mesh, fe));
        }
    }
    None
}

/// Lowers the models asked for until none waits.
fn lower_in_background() {
    loop {
        let job = {
            let Ok(mut background) = BACKGROUND.lock() else {
                return;
            };
            match background.next.take() {
                Some(job) => {
                    background.running = Some(job.key);
                    job
                }
                None => {
                    background.running = None;
                    return;
                }
            }
        };
        if kept(job.key).is_none() {
            made(job.key, &job.mesh, &job.fe);
        }
        drop(job);
        GENERATION.fetch_add(1, Ordering::AcqRel);
        if let Some(ctx) = REPAINT.lock().ok().and_then(|r| r.clone()) {
            ctx.request_repaint();
        }
    }
}

/// What the whole solution is computed from.
#[cfg(test)]
pub fn solve_input(mesh: &FeMesh, fe: &FeModel) -> u64 {
    fingerprints(mesh, fe).input
}

/// What the results of each step are computed from: the model and the steps up to it.
#[cfg(test)]
pub fn step_inputs(mesh: &FeMesh, fe: &FeModel) -> Vec<u64> {
    fingerprints(mesh, fe).steps
}
