//! CAD operations on geometry: a fillet or a chamfer of picked edges or a planar face moved
//! along its normal, which a block of the
//! project schematic applies to the geometry linked to it, so that a parameter study can
//! vary a radius like any other number.

use plx_mesh::{CadEntity, Fnv};
use serde::{Deserialize, Serialize};

use crate::{EntitySignature, Geometry, match_entities};

/// What a CAD operation does.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum CadOperationKind {
    /// Rounds edges with a radius.
    #[default]
    Fillet,
    /// Bevels edges by a distance on both faces.
    Chamfer,
    /// Moves planar faces along their normal by an offset, outwards when it is positive:
    /// a prism on the face is added to the solid or cut from it, so that perpendicular
    /// neighbours grow or shrink with it and sloped ones get a step.
    PushPull,
}

impl CadOperationKind {
    pub const ALL: [Self; 3] = [Self::Fillet, Self::Chamfer, Self::PushPull];

    pub fn label(self) -> &'static str {
        match self {
            Self::Fillet => "Fillet",
            Self::Chamfer => "Chamfer",
            Self::PushPull => "Push/Pull",
        }
    }

    /// The name of its length, a row of the block.
    pub fn value_label(self) -> &'static str {
        match self {
            Self::Fillet => "Radius",
            Self::Chamfer => "Distance",
            Self::PushPull => "Offset",
        }
    }

    /// The key of its length among the parameters of the block, `["Parameters", key]`.
    pub fn value_key(self) -> &'static str {
        match self {
            Self::Fillet => "radius",
            Self::Chamfer => "distance",
            Self::PushPull => "offset",
        }
    }

    /// Whether it acts on entities of this kind.
    pub fn picks(self, entity: CadEntity) -> bool {
        match self {
            // Edges of solids, or corners of flat faces.
            Self::Fillet | Self::Chamfer => {
                matches!(entity, CadEntity::Edge(_) | CadEntity::Vertex(_))
            }
            Self::PushPull => matches!(entity, CadEntity::Face(_)),
        }
    }

    /// What it acts on, for texts.
    pub fn picked_noun(self) -> &'static str {
        match self {
            Self::Fillet | Self::Chamfer => "edges",
            Self::PushPull => "faces",
        }
    }

    /// One of what it acts on, for texts.
    pub fn picked_one(self) -> &'static str {
        match self {
            Self::Fillet | Self::Chamfer => "edge",
            Self::PushPull => "face",
        }
    }

    /// Whether its length may be negative: an offset inwards.
    pub fn signed(self) -> bool {
        self == Self::PushPull
    }
}

/// A CAD operation with what it acts on and its length.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadOperation {
    pub kind: CadOperationKind,
    /// The picked entities of the input geometry with what they looked like, so that they
    /// are found again when the input changes.
    pub entities: Vec<EntitySignature>,
    /// The radius or distance in metres.
    pub value: f64,
}

impl Default for CadOperation {
    fn default() -> Self {
        Self {
            kind: CadOperationKind::default(),
            entities: Vec::new(),
            // One millimetre.
            value: 1e-3,
        }
    }
}

impl CadOperation {
    /// What it acts on, for texts, e.g. "2 edges" or "1 corner".
    pub fn describe_picks(&self) -> String {
        let count = self.entities.len();
        if count == 0 {
            return "nothing picked".to_owned();
        }
        let corners = (self.entities.iter()).any(|s| matches!(s.entity, CadEntity::Vertex(_)));
        let (one, many) = if corners {
            ("corner", "corners")
        } else {
            (self.kind.picked_one(), self.kind.picked_noun())
        };
        format!("{count} {}", if count == 1 { one } else { many })
    }

    /// The picked entities in the geometry it is applied to, or why they cannot all be found.
    pub fn resolve(&self, input: &Geometry) -> Result<Vec<CadEntity>, String> {
        if self.entities.is_empty() {
            return Err(format!("no {} picked", self.kind.picked_noun()));
        }
        if input.signatures.is_empty() {
            return Ok(self.entities.iter().map(|s| s.entity).collect());
        }
        let map = match_entities(&self.entities, &input.signatures);
        let found: Vec<CadEntity> = (self.entities.iter())
            .filter_map(|s| map.get(&s.entity).copied())
            .collect();
        match self.entities.len() - found.len() {
            0 => Ok(found),
            1 => Err("1 picked entity is no longer found in the geometry; pick it again".into()),
            lost => Err(format!(
                "{lost} picked entities are no longer found in the geometry; pick them again"
            )),
        }
    }

    /// Follows the picks into a changed input: each takes the signature of the entity it
    /// was found as, so that the next change starts from there. Picks not found stay.
    pub fn follow(&mut self, input: &Geometry) {
        if input.signatures.is_empty() {
            return;
        }
        let map = match_entities(&self.entities, &input.signatures);
        for pick in &mut self.entities {
            let found = (map.get(&pick.entity))
                .and_then(|new| input.signatures.iter().find(|s| s.entity == *new));
            if let Some(found) = found {
                *pick = found.clone();
            }
        }
    }

    /// Fingerprint of the result for this input: the shapes, the operation, the entities
    /// it acts on and its length.
    pub fn fingerprint(&self, input: &Geometry) -> u64 {
        let mut hash = Fnv::new();
        hash.add_str(&input.brep);
        hash.add_debug(&input.part_names);
        hash.add_debug(&self.kind);
        hash.add_debug(&self.resolve(input));
        hash.add(&self.value.to_le_bytes());
        hash.finish()
    }

    /// Why the operation cannot be applied as it is, before trying.
    pub fn problem(&self) -> Option<String> {
        if !self.value.is_finite() {
            return Some(format!(
                "the {} is not a number",
                self.kind.value_label().to_lowercase()
            ));
        }
        if !self.kind.signed() && self.value < 0.0 {
            return Some(format!(
                "the {} must not be negative",
                self.kind.value_label().to_lowercase()
            ));
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MeshingParameters;

    fn edge(tag: i32, x: f64) -> EntitySignature {
        EntitySignature {
            entity: CadEntity::Edge(tag),
            kind: "Line".into(),
            centroid: [x, 0.5, 0.0],
            min: [x, 0.0, 0.0],
            max: [x, 1.0, 0.0],
            size: 1.0,
        }
    }

    fn geometry(signatures: Vec<EntitySignature>) -> Geometry {
        Geometry {
            compounds: Vec::new(),
            source: "block.step".into(),
            brep: format!("{signatures:?}"),
            meshing: MeshingParameters::default(),
            mesh_items: Vec::new(),
            part_names: Vec::new(),
            refinement: 1.0,
            signatures,
        }
    }

    #[test]
    fn picks_are_found_and_followed_in_a_changed_input() {
        let operation = CadOperation {
            entities: vec![edge(3, 1.0)],
            ..CadOperation::default()
        };
        let input = geometry(vec![edge(1, 0.0), edge(3, 1.0)]);
        assert_eq!(operation.resolve(&input), Ok(vec![CadEntity::Edge(3)]));
        // Numbered anew: the edge is 5 now.
        let changed = geometry(vec![edge(5, 1.0), edge(6, 0.0)]);
        assert_eq!(operation.resolve(&changed), Ok(vec![CadEntity::Edge(5)]));
        assert_ne!(
            operation.fingerprint(&input),
            operation.fingerprint(&changed)
        );
        let mut followed = operation.clone();
        followed.follow(&changed);
        assert_eq!(followed.entities[0].entity, CadEntity::Edge(5));
        // Gone: the operation says so rather than taking another edge.
        let gone = geometry(vec![edge(1, 0.0)]);
        assert!(
            operation
                .resolve(&gone)
                .unwrap_err()
                .contains("pick it again")
        );
        assert!(CadOperation::default().resolve(&input).is_err());
        let flat = CadOperation {
            value: 0.0,
            ..operation
        };
        // Zero leaves the geometry as it is, so that a study may switch it off.
        assert_eq!(flat.problem(), None);
        let negative = CadOperation {
            value: -1e-3,
            ..flat.clone()
        };
        assert!(negative.problem().unwrap().contains("radius"));
        // An offset may point inwards.
        let inwards = CadOperation {
            kind: CadOperationKind::PushPull,
            value: -2e-3,
            ..flat
        };
        assert_eq!(inwards.problem(), None);
        assert_eq!(
            CadOperation {
                value: 0.0,
                ..inwards
            }
            .problem(),
            None
        );
        assert!(CadOperationKind::PushPull.picks(CadEntity::Face(1)));
        assert!(!CadOperationKind::PushPull.picks(CadEntity::Edge(1)));
    }
}
