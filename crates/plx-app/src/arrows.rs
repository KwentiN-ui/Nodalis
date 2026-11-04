//! Arrows of the shown vector source on the parts' surfaces: which nodes carry one, how long
//! and in which colour, for the results and each transformed copy of them.

use std::collections::{HashMap, HashSet};

use glam::{DAffine3, DVec3};
use plx_mesh::PartSkin;
use plx_render::Vertex;
use plx_render::glyphs::{Arrow, arrow_lines, spread_nodes, surface_area, surface_nodes};

use crate::results::ResultsView;

/// Colour of nodes without a value, like the contours' (NO_VALUE_COLOR of viewport.wgsl).
const NO_VALUE_COLOR: [f32; 3] = [0.75; 3];

/// The arrows of every part, picked once and drawn on the results and their copies.
pub struct PartArrows {
    /// Per part: its arrow nodes with their outward normals on the undeformed surface.
    nodes: Vec<Vec<(usize, [f64; 3])>>,
    /// Unit direction and magnitude at every node.
    samples: Vec<([f32; 3], f32)>,
    largest: f32,
    length: f64,
    by_magnitude: bool,
    line_field: bool,
    /// Arrows take the contour colours; else they are black.
    pub colored: bool,
    palette: Vec<[f32; 3]>,
}

impl PartArrows {
    /// The arrows of the shown field when the view draws them. `reference` are the node
    /// positions the arrows are spread over, `with_arrows` tells which parts get any.
    pub fn new(
        view: &ResultsView,
        reference: &[[f64; 3]],
        skins: &[PartSkin],
        with_arrows: impl Fn(usize) -> bool,
    ) -> Option<Self> {
        if view.representation() != plx_results::display::Representation::Arrows {
            return None;
        }
        let source = view.vector_source()?;
        let samples = source.sample(view.current_increment()?)?;
        let options = &view.vector_display.arrows;
        let area: f64 = (skins.iter().enumerate())
            .filter(|(i, _)| with_arrows(*i))
            .map(|(_, skin)| surface_area(reference, skin))
            .sum();
        let count = options.count.max(1) as f64;
        let spacing = (area / count).sqrt();
        let mut taken = HashSet::new();
        let nodes: Vec<Vec<(usize, [f64; 3])>> = (skins.iter().enumerate())
            .map(|(i, skin)| {
                if !with_arrows(i) {
                    return Vec::new();
                }
                let surface = surface_nodes(reference, skin);
                let normals: HashMap<usize, [f64; 3]> = surface.iter().copied().collect();
                let usable = (surface.iter().copied())
                    .filter(|&(n, _)| samples.get(n).is_some_and(|s| s.0[0].is_finite()));
                spread_nodes(reference, usable, spacing, &mut taken)
                    .into_iter()
                    .map(|n| (n, normals[&n]))
                    .collect()
            })
            .collect();
        let largest = (nodes.iter().flatten())
            .map(|&(n, _)| samples[n].1.abs())
            .filter(|m| m.is_finite())
            .fold(0.0f32, f32::max);
        let palette = view.legend().map(|l| l.palette()).unwrap_or_default();
        Some(Self {
            nodes,
            samples,
            largest,
            length: options.length * spacing,
            by_magnitude: options.by_magnitude,
            line_field: source.is_line_field(),
            colored: options.colored,
            palette,
        })
    }

    fn color(&self, normalized: Option<f32>) -> [f32; 3] {
        if !self.colored {
            return [0.0; 3];
        }
        let n = self.palette.len();
        match normalized {
            Some(t) if t >= 0.0 && n > 0 => {
                self.palette[((t * n as f32).floor() as usize).min(n - 1)]
            }
            _ => NO_VALUE_COLOR,
        }
    }

    /// Line vertices of the arrows of part `index` at the drawn node positions `coords` of
    /// the item transformed by `instance`, coloured by the normalized contour values.
    pub fn lines(
        &self,
        index: usize,
        coords: &[[f64; 3]],
        scalars: Option<&[f32]>,
        instance: &DAffine3,
        origin: DVec3,
    ) -> Vec<Vertex> {
        let Some(nodes) = self.nodes.get(index) else {
            return Vec::new();
        };
        let turn = instance.matrix3;
        let mut normals = HashMap::with_capacity(nodes.len());
        let arrows: Vec<Arrow> = (nodes.iter())
            .map(|&(node, normal)| {
                normals.insert(node, (turn * DVec3::from(normal)).to_array());
                let (direction, magnitude) = self.samples[node];
                let direction = turn * DVec3::from(direction.map(f64::from));
                let scale = if self.by_magnitude && self.largest > 0.0 {
                    (magnitude.abs() / self.largest) as f64
                } else {
                    1.0
                };
                Arrow {
                    node,
                    direction: direction.to_array(),
                    scale,
                    color: self.color(scalars.and_then(|s| s.get(node).copied())),
                }
            })
            .collect();
        arrow_lines(
            coords,
            &normals,
            &arrows,
            self.length,
            self.line_field,
            origin,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plx_results::display::Representation;

    fn cantilever() -> (ResultsView, plx_mesh::FeMesh, Vec<PartSkin>) {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/kragbalken_c3d8.frd");
        let import = plx_io::frd::read_frd(&path).unwrap();
        let skins = (import.mesh.parts.iter())
            .map(|part| plx_mesh::extract_part_skin(&import.mesh, part, 30.0))
            .collect();
        let view = ResultsView::new(import.increments, import.mesh.bounds());
        (view, import.mesh, skins)
    }

    fn show(view: &mut ResultsView, field: &str, component: &str) {
        let fields = &view.current_increment().unwrap().fields;
        let index = fields.iter().position(|f| f.name == field).unwrap();
        let components = &fields[index].components;
        let c = components.iter().position(|c| c.name == component).unwrap();
        (view.field, view.component) = (index, c);
    }

    #[test]
    fn arrows_follow_the_shown_field_and_spread_over_the_surface() {
        let (mut view, mesh, skins) = cantilever();
        let coords = mesh.coords();
        show(&mut view, "DISP", "ALL");
        assert!(
            PartArrows::new(&view, coords, &skins, |_| true).is_none(),
            "contours"
        );
        view.vector_display.representation = Representation::Arrows;
        view.vector_display.arrows.count = 40;
        let arrows = PartArrows::new(&view, coords, &skins, |_| true).unwrap();
        let picked = arrows.nodes[0].len();
        assert!((20..=80).contains(&picked), "{picked} arrows for about 40");
        let lines = arrows.lines(0, coords, None, &DAffine3::IDENTITY, DVec3::ZERO);
        // Six vertices per arrow on either side; nodes without displacement get none.
        assert!(!lines.is_empty() && lines.len() <= picked * 12);
        assert!(view.legend().unwrap().title.ends_with("Arrows: DISP"));

        // A tensor gives strokes along its principal direction; a scalar field none.
        show(&mut view, "STRESS", "PRINCIPAL_MAX");
        let strokes = PartArrows::new(&view, coords, &skins, |_| true).unwrap();
        assert!(strokes.line_field);
        assert!(
            view.legend()
                .unwrap()
                .title
                .contains("Principal Max of STRESS")
        );
        assert!(
            PartArrows::new(&view, coords, &skins, |_| false).is_some_and(|a| {
                a.lines(0, coords, None, &DAffine3::IDENTITY, DVec3::ZERO)
                    .is_empty()
            })
        );
    }

    #[test]
    fn arrows_can_follow_another_field_than_the_contours() {
        let (mut view, mesh, skins) = cantilever();
        let coords = mesh.coords();
        show(&mut view, "DISP", "ALL");
        view.vector_display.representation = Representation::Arrows;
        // The principal stresses over the displacement contours.
        view.vector_display.field = Some("STRESS".into());
        view.vector_display.principal = plx_results::Principal::Max;
        let arrows = PartArrows::new(&view, coords, &skins, |_| true).unwrap();
        assert!(arrows.line_field);
        let title = view.legend().unwrap().title;
        assert!(title.contains("Principal Max of STRESS"), "{title}");
        // A field this increment lacks: the shown field again.
        view.vector_display.field = Some("NOPE".into());
        let arrows = PartArrows::new(&view, coords, &skins, |_| true).unwrap();
        assert!(!arrows.line_field);
    }

    #[test]
    fn arrow_settings_are_saved_with_the_view() {
        let (mut view, mesh, _) = cantilever();
        view.vector_display.representation = Representation::Arrows;
        view.vector_display.principal = plx_results::Principal::Min;
        let settings = view.settings();
        let (mut restored, _, _) = cantilever();
        restored.restore(&settings, &mesh);
        assert_eq!(restored.vector_display, view.vector_display);
    }
}
