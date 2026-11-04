//! How CalculiX names and numbers what results hold, mapped to Nodalis's
//! names: shared by the `.frd` reader and the results of the built-in solver, so that both
//! give the same fields, components and elements.

use plx_mesh::ElementShape;

/// The component name Nodalis shows for a component name of the `.frd` file: `D1` becomes
/// `U1`, `SXX` becomes `S11` and so on.
pub fn rename_component(name: &str) -> &str {
    match name {
        "D1" => "U1",
        "D2" => "U2",
        "D3" => "U3",
        "SXX" => "S11",
        "SYY" => "S22",
        "SZZ" => "S33",
        "SXY" => "S12",
        "SYZ" => "S23",
        "SZX" => "S13",
        "EXX" => "E11",
        "EYY" => "E22",
        "EZZ" => "E33",
        "EXY" => "E12",
        "EYZ" => "E23",
        "EZX" => "E13",
        "MEXX" => "ME11",
        "MEYY" => "ME22",
        "MEZZ" => "ME33",
        "MEXY" => "ME12",
        "MEYZ" => "ME23",
        "MEZX" => "ME13",
        "TEM(%)" => "TEM",
        "STR(%)" => "STR",
        other => other,
    }
}

/// Element shape and an Abaqus-style type name for an frd element type number.
pub fn element_kind(kind: i32) -> Option<(ElementShape, &'static str)> {
    Some(match kind {
        1 => (ElementShape::Hex8, "C3D8"),
        2 => (ElementShape::Wedge6, "C3D6"),
        3 => (ElementShape::Tet4, "C3D4"),
        4 => (ElementShape::Hex20, "C3D20"),
        5 => (ElementShape::Wedge15, "C3D15"),
        6 => (ElementShape::Tet10, "C3D10"),
        7 => (ElementShape::Tri3, "S3"),
        8 => (ElementShape::Tri6, "S6"),
        9 => (ElementShape::Quad4, "S4"),
        10 => (ElementShape::Quad8, "S8"),
        11 => (ElementShape::Line2, "B31"),
        12 => (ElementShape::Line3, "B32"),
        _ => return None,
    })
}

/// The nodes of an element in Nodalis's order from those in the `.frd` file's order, which
/// lists the vertical edge mid nodes of quadratic hexahedra and wedges before the top ones.
pub fn reorder_nodes<T: Copy>(shape: ElementShape, frd: &[T]) -> Vec<T> {
    let mut nodes = frd.to_vec();
    match shape {
        ElementShape::Hex20 => {
            nodes[12..16].copy_from_slice(&frd[16..20]);
            nodes[16..20].copy_from_slice(&frd[12..16]);
        }
        ElementShape::Wedge15 => {
            nodes[9..12].copy_from_slice(&frd[12..15]);
            nodes[12..15].copy_from_slice(&frd[9..12]);
        }
        _ => {}
    }
    nodes
}
