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
handle!(
    /// A node set: index in [`crate::mesh::Sets::node_sets`].
    NodeSetId
);
handle!(
    /// An element set: index in [`crate::mesh::Sets::element_sets`].
    ElementSetId
);
handle!(
    /// A surface: index in [`crate::mesh::Sets::surfaces`].
    SurfaceId
);
handle!(
    /// A tie: index in [`crate::Constraints::ties`].
    TieId
);

/// The handles a part of the model holds, by the name of their type
/// (`"NodeSetId"`, `"AmplitudeId"`, ...) and index: what it refers to. Found by
/// serializing the part, so that every handle of every field counts without a list of
/// them to keep up.
pub fn referenced(value: &impl Serialize) -> std::collections::BTreeSet<(&'static str, u32)> {
    let mut found = Collector::default();
    // The collector never fails.
    let _ = value.serialize(&mut found);
    found.handles
}

#[derive(Default)]
struct Collector {
    handles: std::collections::BTreeSet<(&'static str, u32)>,
    /// The type of the newtype being serialized, when it is a handle.
    pending: Option<&'static str>,
}

#[derive(Debug)]
struct Never;

impl std::fmt::Display for Never {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("never")
    }
}

impl std::error::Error for Never {}

impl serde::ser::Error for Never {
    fn custom<T: std::fmt::Display>(_: T) -> Self {
        Never
    }
}

const HANDLES: [&str; 8] = [
    "MaterialId",
    "OrientationId",
    "AmplitudeId",
    "InteractionId",
    "NodeSetId",
    "ElementSetId",
    "SurfaceId",
    "TieId",
];

macro_rules! ignore {
    ($($method:ident: $ty:ty),* $(,)?) => {
        $(fn $method(self, _: $ty) -> Result<(), Never> {
            self.pending = None;
            Ok(())
        })*
    };
}

impl serde::Serializer for &mut Collector {
    type Ok = ();
    type Error = Never;
    type SerializeSeq = Self;
    type SerializeTuple = Self;
    type SerializeTupleStruct = Self;
    type SerializeTupleVariant = Self;
    type SerializeMap = Self;
    type SerializeStruct = Self;
    type SerializeStructVariant = Self;

    ignore!(
        serialize_bool: bool, serialize_i8: i8, serialize_i16: i16, serialize_i32: i32,
        serialize_i64: i64, serialize_u8: u8, serialize_u16: u16, serialize_u64: u64,
        serialize_f32: f32, serialize_f64: f64, serialize_char: char, serialize_str: &str,
        serialize_bytes: &[u8],
    );

    fn serialize_u32(self, v: u32) -> Result<(), Never> {
        if let Some(kind) = self.pending.take() {
            self.handles.insert((kind, v));
        }
        Ok(())
    }

    fn serialize_none(self) -> Result<(), Never> {
        Ok(())
    }

    fn serialize_some<T: ?Sized + Serialize>(self, value: &T) -> Result<(), Never> {
        value.serialize(self)
    }

    fn serialize_unit(self) -> Result<(), Never> {
        Ok(())
    }

    fn serialize_unit_struct(self, _: &'static str) -> Result<(), Never> {
        Ok(())
    }

    fn serialize_unit_variant(self, _: &'static str, _: u32, _: &'static str) -> Result<(), Never> {
        Ok(())
    }

    fn serialize_newtype_struct<T: ?Sized + Serialize>(
        self,
        name: &'static str,
        value: &T,
    ) -> Result<(), Never> {
        self.pending = HANDLES.iter().find(|&&h| h == name).copied();
        value.serialize(&mut *self)?;
        self.pending = None;
        Ok(())
    }

    fn serialize_newtype_variant<T: ?Sized + Serialize>(
        self,
        _: &'static str,
        _: u32,
        _: &'static str,
        value: &T,
    ) -> Result<(), Never> {
        value.serialize(self)
    }

    fn serialize_seq(self, _: Option<usize>) -> Result<Self, Never> {
        Ok(self)
    }

    fn serialize_tuple(self, _: usize) -> Result<Self, Never> {
        Ok(self)
    }

    fn serialize_tuple_struct(self, _: &'static str, _: usize) -> Result<Self, Never> {
        Ok(self)
    }

    fn serialize_tuple_variant(
        self,
        _: &'static str,
        _: u32,
        _: &'static str,
        _: usize,
    ) -> Result<Self, Never> {
        Ok(self)
    }

    fn serialize_map(self, _: Option<usize>) -> Result<Self, Never> {
        Ok(self)
    }

    fn serialize_struct(self, _: &'static str, _: usize) -> Result<Self, Never> {
        Ok(self)
    }

    fn serialize_struct_variant(
        self,
        _: &'static str,
        _: u32,
        _: &'static str,
        _: usize,
    ) -> Result<Self, Never> {
        Ok(self)
    }
}

macro_rules! compound {
    ($($trait:ident { $($method:ident),* }),* $(,)?) => {
        $(impl serde::ser::$trait for &mut Collector {
            type Ok = ();
            type Error = Never;
            $(fn $method<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), Never> {
                value.serialize(&mut **self)
            })*
            fn end(self) -> Result<(), Never> {
                Ok(())
            }
        })*
    };
}

compound!(
    SerializeSeq { serialize_element },
    SerializeTuple { serialize_element },
    SerializeTupleStruct { serialize_field },
    SerializeTupleVariant { serialize_field },
);

impl serde::ser::SerializeMap for &mut Collector {
    type Ok = ();
    type Error = Never;

    fn serialize_key<T: ?Sized + Serialize>(&mut self, key: &T) -> Result<(), Never> {
        key.serialize(&mut **self)
    }

    fn serialize_value<T: ?Sized + Serialize>(&mut self, value: &T) -> Result<(), Never> {
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<(), Never> {
        Ok(())
    }
}

impl serde::ser::SerializeStruct for &mut Collector {
    type Ok = ();
    type Error = Never;

    fn serialize_field<T: ?Sized + Serialize>(
        &mut self,
        _: &'static str,
        value: &T,
    ) -> Result<(), Never> {
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<(), Never> {
        Ok(())
    }
}

impl serde::ser::SerializeStructVariant for &mut Collector {
    type Ok = ();
    type Error = Never;

    fn serialize_field<T: ?Sized + Serialize>(
        &mut self,
        _: &'static str,
        value: &T,
    ) -> Result<(), Never> {
        value.serialize(&mut **self)
    }

    fn end(self) -> Result<(), Never> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn referenced_finds_every_handle_of_a_step() {
        let mut step =
            crate::Step::new("Step-1", crate::step::Procedure::Static(Default::default()));
        step.boundaries.push(crate::step::Boundary {
            region: crate::mesh::NodeRef::Set(NodeSetId(4)),
            first_dof: 1,
            last_dof: 3,
            value: Some(3.0),
            amplitude: Some(AmplitudeId(2)),
            fixed: false,
            submodel_step: Some(7),
            load_case: None,
        });
        let found = referenced(&step);
        assert!(found.contains(&("NodeSetId", 4)), "{found:?}");
        assert!(found.contains(&("AmplitudeId", 2)), "{found:?}");
        assert_eq!(found.len(), 2, "{found:?}");
    }
}
