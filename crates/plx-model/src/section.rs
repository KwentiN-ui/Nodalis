//! Sections: what the elements of a region are made of. A solid section gives a material to
//! solid or plane elements, a beam or truss section turns line elements into CalculiX beams
//! (`*BEAM SECTION`) or trusses (`*SOLID SECTION` with an area).
//!
//! The mesh only knows the shape of a line element, two or three nodes; which CalculiX type
//! it becomes (B31, B32, B32R, T3D2) follows from its section when the input file is written.
//! Beam and truss sections follow CalculiX's own keywords.

use glam::DVec3;
use plx_mesh::{Element, ElementFamily, ElementShape};
use serde::{Deserialize, Serialize};

use crate::{BeamProfile, Region};

/// Assigns a material to the elements of a region, a solid section, or a beam or
/// truss section of CalculiX.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Section {
    pub name: String,
    /// The material of the section; `None` until one is chosen.
    pub material: Option<crate::Id>,
    pub region: Region,
    /// Thickness of plane stress and plane strain elements; other models ignore it.
    #[serde(default = "unit_thickness")]
    pub thickness: f64,
    /// Projects saved before beams existed hold solid sections.
    #[serde(default)]
    pub kind: SectionKind,
}

/// The default thickness of 2D sections, one millimetre in SI units, as values are
/// stored.
pub fn unit_thickness() -> f64 {
    1e-3
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum SectionKind {
    /// Solid, shell or plane elements with a material.
    #[default]
    Solid,
    /// Line elements carrying axial force only (`T3D2`); `area` is the cross-section.
    Truss { area: f64 },
    /// Line elements with bending stiffness (`B31`, `B32`, `B32R`).
    Beam(BeamSection),
    /// Surface elements of a 3D model as shells (`*SHELL SECTION`), a shell
    /// section: the thickness and the offset of the reference surface from the mid-surface
    /// as a fraction of the thickness.
    Shell { thickness: f64, offset: f64 },
    /// Surface elements of a 3D model as membranes (`*MEMBRANE SECTION`): like a shell, but
    /// carrying in-plane forces only, without bending stiffness or rotations. The elements
    /// must be linear: membranes `M3D3`, `M3D4` or `M3D4R`, or linear shells, which are
    /// written as these (see [`membrane_type`]); see [`crate::element_type_options`] for
    /// why quadratic ones are refused.
    Membrane { thickness: f64, offset: f64 },
}

impl SectionKind {
    pub const ALL: [SectionKind; 5] = [
        SectionKind::Solid,
        // One square millimetre and one millimetre, in SI units.
        SectionKind::Truss { area: 1e-6 },
        SectionKind::Beam(BeamSection::DEFAULT),
        SectionKind::Shell {
            thickness: 1e-3,
            offset: 0.0,
        },
        SectionKind::Membrane {
            thickness: 1e-3,
            offset: 0.0,
        },
    ];

    /// Name of the kind, also the prefix of new sections' names, as in
    /// `Solid_Section-1`.
    pub fn prefix(&self) -> &'static str {
        match self {
            SectionKind::Solid => "Solid_Section",
            SectionKind::Truss { .. } => "Truss_Section",
            SectionKind::Beam(_) => "Beam_Section",
            SectionKind::Shell { .. } => "Shell_Section",
            SectionKind::Membrane { .. } => "Membrane_Section",
        }
    }

    /// Name in the dialogs.
    pub fn label(&self) -> &'static str {
        match self {
            SectionKind::Solid => "Solid",
            SectionKind::Truss { .. } => "Truss",
            SectionKind::Beam(_) => "Beam",
            SectionKind::Shell { .. } => "Shell",
            SectionKind::Membrane { .. } => "Membrane",
        }
    }

    pub fn is_line(&self) -> bool {
        matches!(self, SectionKind::Truss { .. } | SectionKind::Beam(_))
    }

    /// The CalculiX type the section gives a line element; `None` when the section does
    /// not change the element's type. Trusses are always `T3D2`: CalculiX 2.21 gets the
    /// stiffness of `T3D3` wrong (3.6 times too soft in a tension test), so a quadratic
    /// line is written with its end nodes only. Pipe and box profiles need `B32R`; `profile`
    /// is the one of a beam section's cross-section.
    pub fn element_type(
        &self,
        shape: ElementShape,
        profile: Option<&BeamProfile>,
    ) -> Option<&'static str> {
        match (self, shape) {
            (SectionKind::Solid | SectionKind::Shell { .. } | SectionKind::Membrane { .. }, _) => {
                None
            }
            (SectionKind::Truss { .. }, ElementShape::Line2 | ElementShape::Line3) => Some("T3D2"),
            // The solver has the general profiles for its user element U1 only, of 2 nodes: a
            // 3-node line is written as two of them.
            (SectionKind::Beam(_), ElementShape::Line2 | ElementShape::Line3)
                if profile.is_some_and(|p| !p.is_native()) =>
            {
                Some("U1")
            }
            (SectionKind::Beam(_), ElementShape::Line2) => Some("B31"),
            (SectionKind::Beam(_), ElementShape::Line3) => Some(
                if profile.is_some_and(BeamProfile::needs_reduced_integration) {
                    "B32R"
                } else {
                    "B32"
                },
            ),
            _ => None,
        }
    }

    /// Why the section cannot be written for an element of the region, if it cannot.
    /// `profile` is the one of a beam section's cross-section.
    pub fn rejects(&self, element: &Element, profile: Option<&BeamProfile>) -> Option<String> {
        let line = element.shape.family() == ElementFamily::Line;
        let surface = element.shape.family() == ElementFamily::Surface;
        let upper = element.type_name.to_ascii_uppercase();
        // Plane elements of a 2D model take a solid section; surface elements of a 3D model
        // are shells (S3 to S8R) or membranes (M3D3 to M3D8R) and need their thickness.
        let membrane = surface && upper.starts_with("M3D");
        let shell = surface && upper.starts_with('S');
        match self {
            SectionKind::Solid if line => Some(format!(
                "Element {} is a line element; lines need a Beam or Truss \
                 Section",
                element.id
            )),
            SectionKind::Solid if shell => Some(format!(
                "Element {} is a shell element; shells need a Shell Section",
                element.id
            )),
            SectionKind::Solid if membrane => Some(format!(
                "Element {} is a membrane element; membranes need a Membrane Section",
                element.id
            )),
            SectionKind::Solid => None,
            SectionKind::Shell { .. } | SectionKind::Membrane { .. } if !surface => Some(format!(
                "Element {} is not a surface element; a {} section needs triangles or \
                     quads",
                element.id,
                self.label().to_ascii_lowercase()
            )),
            SectionKind::Shell { .. } if membrane => Some(format!(
                "Element {} is a membrane element ({}); a Shell Section needs shell \
                 elements, choose an S element type for its part",
                element.id, element.type_name
            )),
            SectionKind::Shell { .. } => None,
            SectionKind::Membrane { .. }
                if matches!(element.shape, ElementShape::Tri6 | ElementShape::Quad8) =>
            {
                Some(format!(
                    "Element {} is quadratic; quadratic membranes are not \
                     supported, mesh the part with linear elements",
                    element.id
                ))
            }
            SectionKind::Membrane { .. } if !membrane && membrane_type(&upper).is_none() => {
                Some(format!(
                    "Element {} is a {} element; a Membrane Section needs triangles or \
                     quads of a 3D model",
                    element.id, element.type_name
                ))
            }
            SectionKind::Membrane { .. } => None,
            _ if !line => Some(format!("Element {} is not a line element", element.id)),
            SectionKind::Beam(_)
                if profile.is_some_and(BeamProfile::needs_reduced_integration)
                    && element.shape != ElementShape::Line3 =>
            {
                Some(format!(
                    "Element {} has 2 nodes; pipe and box profiles need lines with 3 \
                     nodes (B32R)",
                    element.id
                ))
            }
            _ => None,
        }
    }
}

/// The membrane a linear shell is written as under a membrane section: `S3` becomes `M3D3`,
/// `S4` `M3D4` and `S4R` `M3D4R`. Quadratic shells have none; the solver gets quadratic
/// membranes wrong.
pub fn membrane_type(shell_type: &str) -> Option<&'static str> {
    match shell_type.to_ascii_uppercase().as_str() {
        "S3" => Some("M3D3"),
        "S4" => Some("M3D4"),
        "S4R" => Some("M3D4R"),
        _ => None,
    }
}

/// A CalculiX beam section: the cross-section, its orientation and the offset of the beam
/// axis.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BeamSection {
    /// The [`crate::CrossSection`] of the model the beam has; `None` until one is chosen.
    #[serde(default)]
    pub cross_section: Option<crate::Id>,
    #[serde(default)]
    pub orientation: BeamOrientation,
    /// Offset of the beam axis from the nodes in the 1- and 2-direction, in multiples of
    /// the profile's thickness in that direction (`OFFSET1`, `OFFSET2`).
    #[serde(default)]
    pub offset: [f64; 2],
}

impl BeamSection {
    pub const DEFAULT: BeamSection = BeamSection {
        cross_section: None,
        orientation: BeamOrientation::Automatic,
        offset: [0.0, 0.0],
    };
}

impl Default for BeamSection {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Direction 1 of a beam's section, the normal `n1` of `*BEAM SECTION`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum BeamOrientation {
    /// The global z-axis, or the x-axis for beams along z, chosen per element when the
    /// input file is written; the solver projects it onto the plane normal to the beam.
    #[default]
    Automatic,
    /// A direction given by the user; must not be parallel to any beam of the section.
    Direction([f64; 3]),
}

/// Angle in degrees below which a normal counts as parallel to the beam axis.
const PARALLEL_ANGLE_DEG: f64 = 1.0;

impl BeamOrientation {
    /// The normal written for an element with the given unit tangent, or `None` when the
    /// user's direction is parallel to it. Automatic orientation takes +z, or +x for beams
    /// within 1 degree of z.
    pub fn normal_for(&self, tangent: DVec3) -> Option<[f64; 3]> {
        let parallel = |n: DVec3| {
            let n = n.normalize_or_zero();
            n.cross(tangent).length() < PARALLEL_ANGLE_DEG.to_radians().sin()
        };
        match *self {
            BeamOrientation::Automatic => Some(if parallel(DVec3::Z) {
                [1.0, 0.0, 0.0]
            } else {
                [0.0, 0.0, 1.0]
            }),
            BeamOrientation::Direction(n) => {
                let n = DVec3::from(n);
                (n.length() > 0.0 && !parallel(n)).then_some(n.into())
            }
        }
    }
}

/// Unit tangent of a line element, from its first to its last corner node; zero for a
/// degenerate element or missing nodes.
pub fn line_tangent(mesh: &plx_mesh::FeMesh, element: &Element) -> DVec3 {
    let corners = match element.shape {
        ElementShape::Line2 => (0, 1),
        ElementShape::Line3 => (0, 2),
        _ => return DVec3::ZERO,
    };
    let point = |k: usize| element.nodes.get(k).and_then(|&id| mesh.node(id));
    match (point(corners.0), point(corners.1)) {
        (Some(a), Some(b)) => (DVec3::from(b) - DVec3::from(a)).normalize_or_zero(),
        _ => DVec3::ZERO,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_types_follow_the_section() {
        let beam = SectionKind::Beam(BeamSection::DEFAULT);
        let rect = Some(&BeamProfile::ALL[0]);
        assert_eq!(beam.element_type(ElementShape::Line2, rect), Some("B31"));
        assert_eq!(beam.element_type(ElementShape::Line3, rect), Some("B32"));
        assert_eq!(beam.element_type(ElementShape::Tet4, rect), None);
        let pipe = Some(&BeamProfile::ALL[2]);
        assert_eq!(beam.element_type(ElementShape::Line3, pipe), Some("B32R"));
        // General profiles are sections of the user element U1, which has 2 nodes; a
        // 3-node line is written as two.
        let angle = Some(&BeamProfile::ALL[7]);
        assert_eq!(beam.element_type(ElementShape::Line2, angle), Some("U1"));
        assert_eq!(beam.element_type(ElementShape::Line3, angle), Some("U1"));
        let line3 = plx_mesh::Element {
            id: 4,
            type_name: "B32".into(),
            shape: ElementShape::Line3,
            nodes: vec![1, 2, 3],
        };
        assert_eq!(beam.rejects(&line3, angle), None);
        let truss = SectionKind::Truss { area: 2.0 };
        assert_eq!(truss.element_type(ElementShape::Line3, None), Some("T3D2"));
        assert_eq!(
            SectionKind::Solid.element_type(ElementShape::Line2, None),
            None
        );
    }

    #[test]
    fn automatic_normals_avoid_the_beam_axis() {
        let auto = BeamOrientation::Automatic;
        assert_eq!(auto.normal_for(DVec3::X), Some([0.0, 0.0, 1.0]));
        assert_eq!(auto.normal_for(DVec3::Z), Some([1.0, 0.0, 0.0]));
        assert_eq!(auto.normal_for(-DVec3::Z), Some([1.0, 0.0, 0.0]));
        let given = BeamOrientation::Direction([0.0, 1.0, 0.0]);
        assert_eq!(given.normal_for(DVec3::X), Some([0.0, 1.0, 0.0]));
        assert_eq!(given.normal_for(DVec3::Y), None);
        assert_eq!(
            BeamOrientation::Direction([0.0; 3]).normal_for(DVec3::X),
            None
        );
    }

    #[test]
    fn profiles_write_calculix_dimensions() {
        assert_eq!(
            BeamProfile::Circ { radius: 2.0 }.data_line(),
            vec![4.0, 4.0]
        );
        assert_eq!(
            BeamProfile::Box {
                a: 10.0,
                b: 5.0,
                t: [1.0, 2.0, 3.0, 4.0]
            }
            .data_line(),
            vec![10.0, 5.0, 1.0, 2.0, 3.0, 4.0]
        );
        assert!(
            !BeamProfile::Pipe {
                radius: 1.0,
                thickness: 1.0
            }
            .is_valid()
        );
    }

    #[test]
    fn membranes_and_shells_need_their_own_section() {
        let element = |type_name: &str, shape| Element {
            id: 1,
            type_name: type_name.into(),
            shape,
            nodes: Vec::new(),
        };
        let membrane = SectionKind::Membrane {
            thickness: 1.0,
            offset: 0.0,
        };
        let shell = SectionKind::Shell {
            thickness: 1.0,
            offset: 0.0,
        };
        for type_name in ["M3D3", "M3D4", "M3D4R"] {
            let shape = ElementShape::from_type_name(type_name).unwrap();
            assert_eq!(membrane.rejects(&element(type_name, shape), None), None);
            assert!(shell.rejects(&element(type_name, shape), None).is_some());
            assert!(
                SectionKind::Solid
                    .rejects(&element(type_name, shape), None)
                    .is_some()
            );
        }
        // Linear shells are written as membranes.
        assert_eq!(
            membrane.rejects(&element("S4R", ElementShape::Quad4), None),
            None
        );
        assert_eq!(membrane_type("S4R"), Some("M3D4R"));
        assert!(
            membrane
                .rejects(&element("S8", ElementShape::Quad8), None)
                .is_some()
        );
        assert!(
            membrane
                .rejects(&element("C3D8", ElementShape::Hex8), None)
                .is_some()
        );
        // The solver gets quadratic membranes wrong.
        assert!(
            membrane
                .rejects(&element("M3D8", ElementShape::Quad8), None)
                .is_some()
        );
        assert_eq!(
            shell.rejects(&element("S8R", ElementShape::Quad8), None),
            None
        );
    }

    #[test]
    fn beam_sections_name_their_cross_section() {
        let text = r#"(cross_section: Some(5), orientation: Automatic, offset: (0.0, 0.5))"#;
        let beam: BeamSection = ron::from_str(text).unwrap();
        assert_eq!(beam.cross_section.map(|id| id.raw()), Some(5));
        // A beam section saved with its own profile reads without a cross-section, which
        // the model checks then ask for.
        let text = r#"(profile: Rect(a: 1.0, b: 2.0), orientation: Automatic)"#;
        let beam: BeamSection = ron::from_str(text).unwrap();
        assert_eq!(beam.cross_section, None);
    }

    #[test]
    fn sections_of_old_projects_are_solid() {
        let text = r#"(name: "S", material: Some(3), region: Parts(["P"]))"#;
        let section: Section = ron::from_str(text).unwrap();
        assert_eq!(section.kind, SectionKind::Solid);
        // One millimetre, in SI units.
        assert_eq!(section.thickness, 1e-3);
    }
}
