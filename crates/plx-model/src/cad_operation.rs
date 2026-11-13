//! CAD operations on geometry: a fillet or a chamfer of picked edges, a planar face moved
//! along its normal, parts and faces split along a plane or faces split around holes, which
//! a block of the
//! project schematic applies to the geometry linked to it, so that a parameter study can
//! vary a radius like any other number.

use plx_mesh::{CadEntity, Fnv};
use serde::{Deserialize, Serialize};

use crate::{CoordinatePlane, EntitySignature, Geometry, match_entities};

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
    /// Splits the parts a plane runs through: solids into one solid on each side, faces into
    /// faces that share the cut edge; one side may be dropped. The plane is a plane of the
    /// global coordinate system or one of a picked planar face, shifted by an offset.
    SplitPlane,
    /// Splits the planar face around a hole into a ring of a width and the rest, so that the
    /// ring can be picked on its own: for loads and supports next to a hole.
    HoleRing,
    /// Removes fillets and chamfers: the faces next to them grow until they meet again, so
    /// that a fillet of a STEP file can be replaced by one of a CAD operation that a study
    /// varies, or cosmetic ones do not make the mesh fine. The faces are found by their
    /// shape up to a size, or picked.
    Defeature,
}

impl CadOperationKind {
    pub const ALL: [Self; 6] = [
        Self::Fillet,
        Self::Chamfer,
        Self::PushPull,
        Self::SplitPlane,
        Self::HoleRing,
        Self::Defeature,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Fillet => "Fillet",
            Self::Chamfer => "Chamfer",
            Self::PushPull => "Push/Pull",
            Self::SplitPlane => "Split by Plane",
            Self::HoleRing => "Split around Hole",
            Self::Defeature => "Defeature",
        }
    }

    /// Whether it splits the geometry rather than changing its shape.
    pub fn splits(self) -> bool {
        matches!(self, Self::SplitPlane | Self::HoleRing)
    }

    /// The name of its length, a row of the block.
    pub fn value_label(self) -> &'static str {
        match self {
            Self::Fillet => "Radius",
            Self::Chamfer => "Distance",
            Self::PushPull | Self::SplitPlane => "Offset",
            Self::HoleRing => "Width",
            Self::Defeature => "Max Size",
        }
    }

    /// The key of its length among the parameters of the block, `["Parameters", key]`.
    pub fn value_key(self) -> &'static str {
        match self {
            Self::Fillet => "radius",
            Self::Chamfer => "distance",
            Self::PushPull | Self::SplitPlane => "offset",
            Self::HoleRing => "width",
            Self::Defeature => "max_size",
        }
    }

    /// Whether it acts on entities of this kind.
    pub fn picks(self, entity: CadEntity) -> bool {
        match self {
            // Edges of solids, or corners of flat faces.
            Self::Fillet | Self::Chamfer => {
                matches!(entity, CadEntity::Edge(_) | CadEntity::Vertex(_))
            }
            Self::PushPull | Self::SplitPlane | Self::Defeature => {
                matches!(entity, CadEntity::Face(_))
            }
            Self::HoleRing => matches!(entity, CadEntity::Edge(_)),
        }
    }

    /// What it acts on, for texts.
    pub fn picked_noun(self) -> &'static str {
        match self {
            Self::Fillet | Self::Chamfer => "edges",
            Self::PushPull | Self::SplitPlane | Self::Defeature => "faces",
            Self::HoleRing => "hole edges",
        }
    }

    /// One of what it acts on, for texts.
    pub fn picked_one(self) -> &'static str {
        match self {
            Self::Fillet | Self::Chamfer => "edge",
            Self::PushPull | Self::SplitPlane | Self::Defeature => "face",
            Self::HoleRing => "hole edge",
        }
    }

    /// Whether its length may be negative: an offset inwards.
    pub fn signed(self) -> bool {
        matches!(self, Self::PushPull | Self::SplitPlane)
    }
}

/// The plane a [`CadOperationKind::SplitPlane`] cuts along, before its offset.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SplitPlane {
    /// A plane of the global coordinate system through the origin, shifted along its axis.
    Global(CoordinatePlane),
    /// The plane of the picked planar face, shifted into the part it bounds.
    Face,
}

impl Default for SplitPlane {
    fn default() -> Self {
        Self::Global(CoordinatePlane::Yz)
    }
}

impl SplitPlane {
    pub const ALL: [Self; 4] = [
        Self::Global(CoordinatePlane::Yz),
        Self::Global(CoordinatePlane::Xz),
        Self::Global(CoordinatePlane::Xy),
        Self::Face,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Global(CoordinatePlane::Yz) => "YZ plane",
            Self::Global(CoordinatePlane::Xz) => "XZ plane",
            Self::Global(CoordinatePlane::Xy) => "XY plane",
            Self::Face => "Plane of a face",
        }
    }

    /// The axes a global plane can be tilted about: the two lying in it, as indices 0 to 2;
    /// a plane of a face about any.
    pub fn tilt_axes(self) -> Vec<usize> {
        match self {
            Self::Global(plane) => (0..3).filter(|&k| k != plane.normal_axis()).collect(),
            Self::Face => vec![0, 1, 2],
        }
    }

    /// The unit normal of a global plane, the direction its offset goes.
    pub fn axis(self) -> Option<[f64; 3]> {
        match self {
            Self::Global(plane) => {
                let mut normal = [0.0; 3];
                normal[plane.normal_axis()] = 1.0;
                Some(normal)
            }
            Self::Face => None,
        }
    }
}

/// `normal` turned by `degrees` about the global axis `axis` (0 to 2), right-handed.
pub fn tilted_normal(normal: [f64; 3], axis: usize, degrees: f64) -> [f64; 3] {
    if degrees == 0.0 {
        return normal;
    }
    let (s, c) = degrees.to_radians().sin_cos();
    let (a, b) = ((axis + 1) % 3, (axis + 2) % 3);
    let mut turned = normal;
    turned[a] = c * normal[a] - s * normal[b];
    turned[b] = s * normal[a] + c * normal[b];
    turned
}

/// Which faces a [`CadOperationKind::Defeature`] removes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum DefeatureFaces {
    /// The fillets and chamfers found by their shape: fillets of a radius up to the size,
    /// chamfers up to that width.
    #[default]
    Found,
    /// The picked faces.
    Picked,
}

impl DefeatureFaces {
    pub const ALL: [Self; 2] = [Self::Found, Self::Picked];

    pub fn label(self) -> &'static str {
        match self {
            Self::Found => "Found by size",
            Self::Picked => "Picked faces",
        }
    }
}

/// Which pieces a [`CadOperationKind::SplitPlane`] keeps.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum SplitKeep {
    /// Both sides.
    #[default]
    Both,
    /// The side the normal of the plane points to: the positive axis of a global plane, or
    /// for the plane of a face the side away from the face.
    Positive,
    /// The other side: the negative axis, or the slice between the face and the cut.
    Negative,
}

impl SplitKeep {
    pub const ALL: [Self; 3] = [Self::Both, Self::Positive, Self::Negative];

    /// Its label for the plane cut along.
    pub fn label(self, plane: SplitPlane) -> String {
        let axis = |sign: &str| match plane {
            SplitPlane::Global(plane) => {
                format!("{sign}{} side", ["X", "Y", "Z"][plane.normal_axis()])
            }
            SplitPlane::Face => String::new(),
        };
        match (self, plane) {
            (Self::Both, _) => "Both sides".to_owned(),
            (Self::Positive, SplitPlane::Face) => "Beyond the cut".to_owned(),
            (Self::Negative, SplitPlane::Face) => "Between face and cut".to_owned(),
            (Self::Positive, _) => axis("+"),
            (Self::Negative, _) => axis("-"),
        }
    }
}

fn yes() -> bool {
    true
}

fn is_default<T: Default + PartialEq>(value: &T) -> bool {
    *value == T::default()
}

/// A CAD operation with what it acts on and its length.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadOperation {
    pub kind: CadOperationKind,
    /// The picked entities of the input geometry with what they looked like, so that they
    /// are found again when the input changes.
    pub entities: Vec<EntitySignature>,
    /// The radius, distance, offset or width in metres.
    pub value: f64,
    /// The plane a split cuts along.
    #[serde(default, skip_serializing_if = "is_default")]
    pub plane: SplitPlane,
    /// The pieces a split keeps.
    #[serde(default, skip_serializing_if = "is_default")]
    pub keep: SplitKeep,
    /// Solids split in two stay one part of the FE model, a compound part of their pieces,
    /// which are tied where they were cut; otherwise each piece is a part of its own.
    #[serde(default = "yes")]
    pub compound: bool,
    /// Which faces a defeaturing removes.
    #[serde(default, skip_serializing_if = "is_default")]
    pub faces: DefeatureFaces,
    /// The names of the parts a split by plane cuts; the others stay as they are. All parts
    /// when empty.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub parts: Vec<String>,
    /// The global axis, 0 to 2 for x to z, a split plane is tilted about, through the point
    /// of the plane its offset gives.
    #[serde(default, skip_serializing_if = "is_default")]
    pub tilt_axis: usize,
    /// The angle in degrees the split plane is tilted by about its tilt axis, right-handed;
    /// zero leaves it as it is.
    #[serde(default, skip_serializing_if = "is_default")]
    pub tilt_angle: f64,
}

impl Default for CadOperation {
    fn default() -> Self {
        Self {
            kind: CadOperationKind::default(),
            entities: Vec::new(),
            // One millimetre.
            value: 1e-3,
            plane: SplitPlane::default(),
            keep: SplitKeep::default(),
            compound: true,
            faces: DefeatureFaces::default(),
            parts: Vec::new(),
            tilt_axis: 0,
            tilt_angle: 0.0,
        }
    }
}

impl CadOperation {
    /// What it acts on, for texts, e.g. "2 edges" or "1 corner".
    pub fn describe_picks(&self) -> String {
        if self.kind == CadOperationKind::SplitPlane {
            let mut plane = match self.plane {
                SplitPlane::Face if self.entities.is_empty() => "no face picked".to_owned(),
                plane => plane.label().to_owned(),
            };
            if self.tilt_angle != 0.0 {
                plane = format!(
                    "{plane} tilted {} deg about {}",
                    self.tilt_angle,
                    ["X", "Y", "Z"][self.tilt_axis.min(2)]
                );
            }
            return match self.parts.len() {
                0 => plane,
                1 => format!("{plane}, 1 part"),
                n => format!("{plane}, {n} parts"),
            };
        }
        if self.kind == CadOperationKind::Defeature && self.faces == DefeatureFaces::Found {
            return "fillets and chamfers by size".to_owned();
        }
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

    /// Whether it needs picked entities: all but a split along a global plane and a
    /// defeaturing of the faces found by size do.
    pub fn needs_picks(&self) -> bool {
        match self.kind {
            CadOperationKind::SplitPlane => self.plane == SplitPlane::Face,
            CadOperationKind::Defeature => self.faces == DefeatureFaces::Picked,
            _ => true,
        }
    }

    /// The picked entities in the geometry it is applied to, or why they cannot all be found.
    pub fn resolve(&self, input: &Geometry) -> Result<Vec<CadEntity>, String> {
        if !self.needs_picks() {
            return Ok(Vec::new());
        }
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
        if self.kind == CadOperationKind::SplitPlane {
            hash.add_debug(&(self.plane, self.keep, self.compound));
            hash.add_debug(&(self.tilt_axis, self.tilt_angle.to_bits()));
        }
        if self.kind == CadOperationKind::Defeature {
            hash.add_debug(&self.faces);
        }
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
        if self.kind == CadOperationKind::SplitPlane && !self.tilt_angle.is_finite() {
            return Some("the tilt angle is not a number".to_owned());
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
            suppressed: Vec::new(),
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
