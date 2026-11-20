//! Typed handles: how one part of the model refers to another. A handle is the index of
//! the item in its list of the [`crate::Model`]; names are labels only and never used to
//! find an item, so that two items with similar or equal names stay two items.

use serde::{Deserialize, Serialize};

macro_rules! handle {
    ($(#[$doc:meta])* $name:ident) => {
        $(#[$doc])*
        #[derive(
            Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
        )]
        pub struct $name(pub u32);

        impl $name {
            /// The index of the item in its list.
            pub fn index(self) -> usize {
                self.0 as usize
            }

            /// The handle of the item at `index`.
            pub fn at(index: usize) -> Self {
                Self(index as u32)
            }
        }
    };
}

handle!(
    /// A material: index in [`crate::Model::materials`].
    MaterialId
);
handle!(
    /// An orientation: index in [`crate::Model::orientations`].
    OrientationId
);
handle!(
    /// An amplitude: index in [`crate::Model::amplitudes`].
    AmplitudeId
);
handle!(
    /// A surface interaction: index in [`crate::Contact::interactions`].
    InteractionId
);
