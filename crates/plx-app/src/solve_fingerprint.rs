//! The fingerprints of what a model's solution is computed from, taken from the analysis
//! the model is lowered to ([`plx_io::fingerprint`]): renaming or hiding an item, or adding
//! one that reaches no step, leaves the results up to date. Lowering takes a while on a big
//! model, so the fingerprints are kept by the quick fingerprint of the model itself
//! ([`plx_model::fingerprint::solve_input`]) and made again only when the model changed.

use std::sync::Mutex;

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
    if let Some((_, kept)) =
        (KEPT.lock().ok()).and_then(|kept| kept.iter().find(|(k, _)| *k == key).cloned())
    {
        return kept;
    }
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

/// What the whole solution is computed from.
pub fn solve_input(mesh: &FeMesh, fe: &FeModel) -> u64 {
    fingerprints(mesh, fe).input
}

/// What the results of each step are computed from: the model and the steps up to it.
pub fn step_inputs(mesh: &FeMesh, fe: &FeModel) -> Vec<u64> {
    fingerprints(mesh, fe).steps
}
