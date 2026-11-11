//! FE-Netz: Knoten, Elemente, Sets, Surfaces, Geometrie-Topologie.

mod cad;
mod contact_search;
mod element;
mod fast_map;
mod jacobian;
mod line_joints;
mod locate;
mod mesh;
mod orient;
mod skin;
pub mod structured;
mod transform;

pub use cad::{CadEntity, CadMap, display_cad_map};
pub use contact_search::{
    GeometryFilter, GroupBy, MasterSlaveItem, SearchParameters, SurfaceId, SurfaceKind,
    find_contact_pairs, surface_faces,
};
pub use element::{ElementFamily, ElementShape, FaceTopology};
pub use line_joints::{LineJoint, find_line_joints};
pub use locate::{PointLocation, SNAP};
pub use mesh::{Body, Element, ElementId, FeMesh, Fnv, MeshError, NodeId, Part, SurfaceDefinition};
pub use orient::orient_outward;
pub use skin::{
    PartSkin, SkinEdge, SkinFace, extract_part_skin, extract_part_skin_welded, face_normal,
};
pub use transform::{FaceRenumbering, MeshTransform};
