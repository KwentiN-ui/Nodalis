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

/// The directions the components of a vector or tensor field are given in: the axes of a
/// rectangular system, or radial, tangential and axial in a cylindrical one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axes {
    Rectangular,
    Cylindrical,
}

/// The fields whose numbered components are directions: `U1` is along x, `S12` the shear
/// in x-y. Field outputs and history outputs alike.
const DIRECTIONAL_FIELDS: [&str; 31] = [
    "DISP",
    "MODE",
    "DISPR",
    "DISPI",
    "DISPLACEMENTS",
    "ROTATION",
    "ROTATIONI",
    "ROTATIONS",
    "FORC",
    "FORCR",
    "FORCI",
    "FORCES",
    "TOTAL_FORCE",
    "MOMENT",
    "MOMENTS",
    "TOTAL_MOMENT",
    "VELO",
    "VELOCITIES",
    "ANGULAR_VELOCITIES",
    "ACCE",
    "STRESS",
    "STRESSR",
    "STRESSI",
    "STRESSES",
    "ZZSTR",
    "ZZSTRR",
    "ZZSTRI",
    "TOSTRAIN",
    "TOSTRAII",
    "MESTRAIN",
    "STRAINS",
];

/// Whether the numbered components of the field `field` are directions.
pub fn is_directional(field: &str) -> bool {
    let field = field.to_ascii_uppercase();
    DIRECTIONAL_FIELDS.contains(&field.as_str()) || matches!(field.as_str(), "FLUX" | "HEAT_FLUX")
}

/// The letters and the directions (0 to 2, one for a vector, two for a tensor) of a
/// numbered component such as `U1` or `S12`; `None` for other names.
fn split_component(component: &str) -> Option<(&str, Vec<usize>)> {
    let digits = component.len()
        - component
            .trim_end_matches(|c: char| c.is_ascii_digit())
            .len();
    let (prefix, number) = component.split_at(component.len() - digits);
    if prefix.is_empty() || !prefix.chars().all(|c| c.is_ascii_alphabetic()) {
        return None;
    }
    let directions: Vec<usize> = number.bytes().map(|b| usize::from(b - b'1')).collect();
    let valid = matches!(directions.len(), 1 | 2) && directions.iter().all(|&d| d < 3);
    valid.then_some((prefix, directions))
}

/// The name the results show for the component `component` of a directional field (see
/// [`is_directional`]) in `axes`: `U1` as `UX`, `S12` as `SXY`; in a cylindrical system
/// `U1` as `U radial` and `S12` as `SRT` (radial, tangential, axial). Other names stay.
/// Only for display: fields keep the numbered names, which equations use.
pub fn component_label(component: &str, axes: Axes) -> String {
    let Some((prefix, directions)) = split_component(component) else {
        return component.to_string();
    };
    match (axes, directions.as_slice()) {
        (Axes::Rectangular, _) => {
            let axes: String = directions.iter().map(|&d| ['X', 'Y', 'Z'][d]).collect();
            format!("{prefix}{axes}")
        }
        (Axes::Cylindrical, [d]) => {
            format!("{prefix} {}", ["radial", "tangential", "axial"][*d])
        }
        (Axes::Cylindrical, _) => {
            let axes: String = directions.iter().map(|&d| ['R', 'T', 'A'][d]).collect();
            format!("{prefix}{axes}")
        }
    }
}

/// The name shown for the component `component` of the solver's field `field` in global
/// directions: `SXY` for `S12` of `STRESS`, unchanged for fields that hold no directions.
pub fn global_component_label(field: &str, component: &str) -> String {
    if is_directional(field) {
        component_label(component, Axes::Rectangular)
    } else {
        component.to_string()
    }
}

/// What the component `component` of a directional field means in `axes`, for a tooltip,
/// with the name equations use; `None` for components that are no direction.
pub fn component_description(field: &str, component: &str, axes: Axes) -> Option<String> {
    let (_, directions) = split_component(component)?;
    let names = match axes {
        Axes::Rectangular => ["x", "y", "z"],
        Axes::Cylindrical => ["radial", "tangential", "axial"],
    };
    let meaning = match directions.as_slice() {
        [d] => match axes {
            Axes::Rectangular => format!("Along {}", names[*d]),
            Axes::Cylindrical => format!("In the {} direction", names[*d]),
        },
        [a, b] if a == b => format!("Normal component, {}", names[*a]),
        [a, b] => format!("Shear component, {}-{}", names[*a], names[*b]),
        _ => return None,
    };
    Some(format!("{meaning}. In equations: {field}.{component}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn numbered_components_are_named_by_their_directions() {
        let rect = |c| component_label(c, Axes::Rectangular);
        let cyl = |c| component_label(c, Axes::Cylindrical);
        assert_eq!(rect("U1"), "UX");
        assert_eq!(rect("UR3"), "URZ");
        assert_eq!(rect("S12"), "SXY");
        assert_eq!(rect("S13"), "SXZ");
        assert_eq!(rect("ME23"), "MEYZ");
        assert_eq!(rect("RF2"), "RFY");
        assert_eq!(cyl("U2"), "U tangential");
        assert_eq!(cyl("S11"), "SRR");
        assert_eq!(cyl("S23"), "STA");
        // Other components keep their names.
        for name in ["ALL", "MISES", "S4", "D5", "S123", "1", "VALUE"] {
            assert_eq!(rect(name), name);
        }
        assert!(is_directional("disp") && is_directional("STRESSES"));
        assert!(!is_directional("CONTACT") && !is_directional("PDISP"));
        assert_eq!(
            component_description("STRESS", "S12", Axes::Cylindrical).unwrap(),
            "Shear component, radial-tangential. In equations: STRESS.S12"
        );
    }
}
