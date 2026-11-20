//! Internal identities of the model's items. Items refer to each other by these ids, never
//! by name, so that renaming an item keeps every reference to it: a history output stays on
//! its reference point, a load keeps its amplitude. The user sees and edits only names; the
//! ids are made when an item is created and never change.

use std::hash::{BuildHasher, Hasher};
use std::sync::atomic::{AtomicU64, Ordering};

use serde::{Deserialize, Serialize};

/// The identity of an item of the model. Ids are unique across models, also across runs of
/// the program, so that items copied from one analysis into another keep theirs without
/// clashing with the items there.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Id(u64);

/// The next id: a random start for each run of the program, counted up from there.
static NEXT: AtomicU64 = AtomicU64::new(0);

impl Id {
    /// A new id, of no item yet.
    pub fn new() -> Self {
        loop {
            let current = NEXT.load(Ordering::Relaxed);
            let id = match current {
                // The first id of the run picks the random start, in the upper half so that
                // it never reaches zero.
                0 => {
                    let random = std::collections::hash_map::RandomState::new()
                        .build_hasher()
                        .finish();
                    random | 1 << 63
                }
                id => id,
            };
            let next = id.checked_add(1).unwrap_or(1 << 63);
            if (NEXT.compare_exchange(current, next, Ordering::Relaxed, Ordering::Relaxed)).is_ok()
            {
                return Self(id);
            }
        }
    }

    /// The number behind the id, e.g. for a stable key in a map of the GUI.
    pub fn raw(self) -> u64 {
        self.0
    }
}

impl Default for Id {
    /// A new id: an item made by its default gets its own.
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Display for Id {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "#{:x}", self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_are_unique() {
        let ids: std::collections::BTreeSet<Id> = (0..1000).map(|_| Id::new()).collect();
        assert_eq!(ids.len(), 1000);
        assert!(ids.iter().all(|id| id.raw() >= 1 << 63));
    }

    #[test]
    fn ids_survive_a_round_trip() {
        let id = Id::new();
        let text = ron::to_string(&id).unwrap();
        assert_eq!(ron::from_str::<Id>(&text).unwrap(), id);
    }
}
