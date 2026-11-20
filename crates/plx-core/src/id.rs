//! Internal identities of the model's items. Items refer to each other by these ids, never
//! by name, so that renaming an item keeps every reference to it: a history output stays on
//! its reference point, a load keeps its amplitude. The user sees and edits only names; the
//! ids are made when an item is created and never change.

use std::hash::{BuildHasher, Hasher};
use std::marker::PhantomData;
use std::sync::atomic::{AtomicU64, Ordering};

use serde::{Deserialize, Serialize};

/// The identity of an item of the model. Ids are unique across models, also across runs of
/// the program, so that items copied from one analysis into another keep theirs without
/// clashing with the items there.
///
/// `T` is the kind of item, e.g. `Id<Step>`: the compiler refuses the id of a mesh part
/// where a boundary condition is asked for. The kind is only in the type; an id is the same
/// number in the project file whatever its kind. [`Id::cast`] turns the id of one kind into
/// that of another where both items are one, such as a part of the geometry and its mesh.
/// `Id` alone, of [`Any`] kind, is an id of any item.
///
/// ```compile_fail
/// # use plx_core::Id;
/// struct Step;
/// struct Material;
/// let step: Id<Step> = Id::new();
/// let material: Id<Material> = step; // the id of a step is no id of a material
/// ```
pub struct Id<T: ?Sized = Any>(u64, PhantomData<fn() -> T>);

/// The kind of an [`Id`] that may be of any item.
pub enum Any {}

/// The next id: a random start for each run of the program, counted up from there.
static NEXT: AtomicU64 = AtomicU64::new(0);

impl<T: ?Sized> Id<T> {
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
                return Self::from_raw(id);
            }
        }
    }

    /// The id an item made for `seed` always has, e.g. the item a link of the project makes
    /// in the analysis it points to: the link finds it again by it, whatever it is renamed
    /// to. In the upper half like the ids [`Id::new`] makes.
    pub fn derived(seed: &str) -> Self {
        // FNV-1a, the same in every version of the program.
        let hash = (seed.bytes()).fold(0xcbf2_9ce4_8422_2325_u64, |hash, byte| {
            (hash ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3)
        });
        Self::from_raw(hash | 1 << 63)
    }

    /// The id of the number [`Self::raw`] gave, such as one the solver carried along.
    pub const fn from_raw(raw: u64) -> Self {
        Self(raw, PhantomData)
    }

    /// The number behind the id, e.g. for a stable key in a map of the GUI.
    pub const fn raw(self) -> u64 {
        self.0
    }

    /// The number behind the id, to change, e.g. for a reference that goes to another item.
    pub fn raw_mut(&mut self) -> &mut u64 {
        &mut self.0
    }

    /// The same id as that of an item of kind `U`, where both are one item: a part of the
    /// geometry and its mesh, a body and the part it is the only body of.
    pub const fn cast<U: ?Sized>(self) -> Id<U> {
        Id(self.0, PhantomData)
    }

    /// The same id, of any kind, e.g. as a key of things of several kinds.
    pub const fn any(self) -> Id {
        self.cast()
    }
}

impl<T: ?Sized> Clone for Id<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T: ?Sized> Copy for Id<T> {}

impl<T: ?Sized> PartialEq for Id<T> {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T: ?Sized> Eq for Id<T> {}

impl<T: ?Sized> PartialOrd for Id<T> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T: ?Sized> Ord for Id<T> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.cmp(&other.0)
    }
}

impl<T: ?Sized> std::hash::Hash for Id<T> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.0.hash(state);
    }
}

impl<T: ?Sized> std::fmt::Debug for Id<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Id({})", self.0)
    }
}

impl<T: ?Sized> Default for Id<T> {
    /// A new id: an item made by its default gets its own.
    fn default() -> Self {
        Self::new()
    }
}

impl<T: ?Sized> Serialize for Id<T> {
    /// The number alone, whatever the kind.
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.0.serialize(serializer)
    }
}

impl<'de, T: ?Sized> Deserialize<'de> for Id<T> {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        u64::deserialize(deserializer).map(Self::from_raw)
    }
}

impl<T: ?Sized> std::fmt::Display for Id<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "#{:x}", self.0)
    }
}

impl<T: ?Sized> std::str::FromStr for Id<T> {
    type Err = ();

    /// Reads an id as [`Id`]'s `Display` writes it, `#` and the number in hex, e.g. from a
    /// path of the project schematic that names an item.
    fn from_str(text: &str) -> Result<Self, ()> {
        let hex = text.strip_prefix('#').ok_or(())?;
        u64::from_str_radix(hex, 16)
            .map(Self::from_raw)
            .map_err(|_| ())
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
    fn the_kind_is_not_in_the_file() {
        struct Step;
        let id: Id<Step> = Id::new();
        assert_eq!(
            ron::to_string(&id).unwrap(),
            ron::to_string(&id.any()).unwrap()
        );
        assert_eq!(
            ron::from_str::<Id<Step>>(&ron::to_string(&id).unwrap()).unwrap(),
            id
        );
    }

    #[test]
    fn ids_survive_a_round_trip() {
        let id = Id::new();
        let text = ron::to_string(&id).unwrap();
        assert_eq!(ron::from_str::<Id>(&text).unwrap(), id);
        assert_eq!(id.to_string().parse(), Ok(id));
        assert!("Step-1".parse::<Id>().is_err());
    }
}
