//! Hot spot stresses in the Results workspace: the definitions of a results file, their
//! paths in the 3D view, the evaluation and output file, and the plot of the stress along
//! the path with its extrapolation that the dialog and the 3D view show.

use std::path::{Path, PathBuf};

use glam::Vec3;
use plx_mesh::{NodeId, SkinFace};
use plx_model::UnitSystem;
use plx_results::hot_spot::{
    self, HotSpot, HotSpotComponent, HotSpotPath, HotSpotReport, ReadoutPoint,
};

use crate::features::Marks;
use crate::model::Model;

/// The hot spots of one results file: defined on it, like its derived outputs, and
/// evaluated whenever they change.
#[derive(Default)]
pub struct HotSpots {
    pub definitions: Vec<HotSpot>,
    /// Values of the definitions, in their order.
    pub reports: Vec<HotSpotReport>,
    /// The output file, once written.
    pub file: Option<PathBuf>,
}

fn surface_faces(model: &Model) -> Vec<&SkinFace> {
    (0..model.parts.len())
        .flat_map(|part| &model.skin(part).faces)
        .collect()
}

/// The paths of a definition on the results, for showing them while it is edited or
/// selected.
pub fn paths(model: &Model, hot_spot: &HotSpot) -> Vec<HotSpotPath> {
    let hot_spot = hot_spot.in_units(units(model));
    hot_spot::hot_spot_paths(&model.mesh, surface_faces(model), &hot_spot).0
}

/// The unit system of the results, which IIW's millimetre distances are converted to.
pub fn units(model: &Model) -> UnitSystem {
    (model.results.as_ref()).map_or(model.fe.properties.units, |view| view.units)
}

/// Paths in render coordinates, deformed like the shown mesh, each starting at its toe.
pub fn render_paths<'a>(
    model: &Model,
    paths: impl IntoIterator<Item = &'a HotSpotPath>,
) -> Vec<Vec<Vec3>> {
    let position = |weights: &[(usize, f64)]| {
        weights.iter().try_fold(Vec3::ZERO, |sum, &(node, weight)| {
            Some(sum + model.node_position(node)? * weight as f32)
        })
    };
    paths
        .into_iter()
        .filter_map(|path| {
            std::iter::once(model.node_position(path.index))
                .chain(path.points.iter().map(|p| position(&p.weights)))
                .collect()
        })
        .collect()
}

/// Evaluates the hot spot definitions of the results file over all its increments.
pub fn evaluate(model: &Model) -> Vec<HotSpotReport> {
    let Some(view) = &model.results else {
        return Vec::new();
    };
    let definitions: Vec<HotSpot> = (model.hot_spots.definitions.iter())
        .map(|h| h.in_units(units(model)))
        .collect();
    hot_spot::evaluate(
        &model.mesh,
        &surface_faces(model),
        &definitions,
        &view.increments,
    )
}

/// Where the values of a results file are written: next to it, e.g. `Analysis-1.frd` gives
/// `Analysis-1_hot_spots.csv`.
pub fn output_path(results: &Path) -> PathBuf {
    let stem = results
        .file_stem()
        .map_or_else(|| "results".into(), |s| s.to_string_lossy());
    results.with_file_name(format!("{stem}_hot_spots.csv"))
}

pub fn write(results: &Path, reports: &[HotSpotReport]) -> Result<PathBuf, String> {
    let path = output_path(results);
    std::fs::write(&path, hot_spot::to_csv(reports))
        .map_err(|e| format!("{} not written: {e}", path.display()))?;
    Ok(path)
}

/// One line per definition with its largest value over all increments.
pub fn summary(reports: &[HotSpotReport]) -> Vec<String> {
    reports
        .iter()
        .map(|report| {
            let maximum = (report.increments.iter())
                .filter_map(|i| Some((i, i.maximum()?)))
                .max_by(|a, b| a.1.hot_spot.total_cmp(&b.1.hot_spot));
            match maximum {
                Some((increment, value)) => format!(
                    "{}: max. {} = {:.2} at node {} (Step {}, Increment {})",
                    report.name,
                    report.component.short(),
                    value.hot_spot,
                    value.node,
                    increment.step,
                    increment.increment
                ),
                None => format!("{}: no values", report.name),
            }
        })
        .collect()
}

/// Points on the plotted stress curve of a hot spot path.
const CURVE_POINTS: usize = 81;
/// The curve runs this far beyond the furthest read-out point.
const CURVE_REACH: f64 = 1.25;

/// What the dialog and the plot in the 3D view show of a hot spot in the shown increment:
/// the stress along the path of its toe node with the highest hot spot stress, the read-out
/// points and the extrapolation back to the toe.
#[derive(Clone, Debug, PartialEq)]
pub struct HotSpotPlot {
    pub node: NodeId,
    /// Toe nodes with a path; the plotted one has the highest hot spot stress of them.
    pub paths: usize,
    pub component: HotSpotComponent,
    /// Stress along the path, by distance from the toe.
    pub distances: Vec<f64>,
    pub values: Vec<f64>,
    /// Read-out points, extrapolation line and hot spot stress, drawn over the curve.
    pub marks: Marks,
    /// Start and end of the plotted stretch of the path, in model coordinates.
    pub ends: [[f64; 3]; 2],
    pub warnings: Vec<String>,
}

impl HotSpotPlot {
    /// Values the plot shows when its range is automatic: all but a sharp peak right at the
    /// toe, the notch stress, which would flatten the extrapolation to a line at the bottom.
    pub fn range(&self) -> [f64; 2] {
        let first = self.marks.support.first().map_or(0.0, |p| p[0]);
        let marked = (self.marks.support.iter().chain(&self.marks.line)).map(|p| p[1]);
        let away = (self.distances.iter().zip(&self.values))
            .filter(|(d, _)| **d >= 0.5 * first)
            .map(|(_, v)| *v);
        let finite = || marked.clone().chain(away.clone()).filter(|v| v.is_finite());
        let low = finite().fold(f64::INFINITY, f64::min);
        let high = finite().fold(f64::NEG_INFINITY, f64::max);
        if !(low.is_finite() && high.is_finite()) {
            return [0.0, 1.0];
        }
        let span = (high - low).max(1e-12 * high.abs().max(1.0));
        let all = self.values.iter().copied().filter(|v| v.is_finite());
        let peak_low = all.clone().fold(low, f64::min);
        let peak_high = all.fold(high, f64::max);
        // A peak that stays within twice the range is shown whole.
        let low = if low - peak_low <= span {
            peak_low
        } else {
            low - 0.1 * span
        };
        let high = if peak_high - high <= span {
            peak_high
        } else {
            high + 0.1 * span
        };
        [low, high]
    }

    /// The curve, the read-out points and the hot spot stress as tables, for spreadsheets.
    pub fn to_csv(&self, separator: char) -> String {
        let name = self.component.short();
        let mut text = format!("Distance{separator}{name}\n");
        for (d, v) in self.distances.iter().zip(&self.values) {
            text += &format!("{d}{separator}{v}\n");
        }
        text += &format!("\nRead-out distance{separator}{name}\n");
        for [d, s] in &self.marks.support {
            text += &format!("{d}{separator}{s}\n");
        }
        text += &format!(
            "\nNode{separator}S_hs\n{}{separator}{}\n",
            self.node, self.marks.hot_spot
        );
        text
    }
}

/// The paths of a definition, located again only when it changes, and the stress curve of
/// the path last plotted.
#[derive(Default)]
pub struct Located {
    /// The definition in the units of the results the paths were located for.
    key: Option<HotSpot>,
    /// The results' mesh the paths lie on, to locate them again on new results.
    mesh: Option<(usize, usize)>,
    paths: Vec<HotSpotPath>,
    warnings: Vec<String>,
    /// Read-out points along the path of the plotted toe node, by the index of the path.
    curve: Option<(usize, Vec<ReadoutPoint>)>,
}

impl Located {
    /// The plot of `hot_spot` on the shown increment of `model`, or why there is none.
    pub fn plot(&mut self, model: &Model, hot_spot: &HotSpot) -> Result<HotSpotPlot, String> {
        let view = model.results.as_ref().ok_or("No results loaded.")?;
        let hot_spot = hot_spot.in_units(units(model));
        let coords = model.mesh.coords();
        let mesh = Some((coords.as_ptr() as usize, coords.len()));
        if self.key.as_ref() != Some(&hot_spot) || self.mesh != mesh {
            (self.paths, self.warnings) =
                hot_spot::hot_spot_paths(&model.mesh, surface_faces(model), &hot_spot);
            self.key = Some(hot_spot.clone());
            self.mesh = mesh;
            self.curve = None;
        }
        if self.paths.is_empty() {
            return Err(match self.warnings.first() {
                Some(warning) => warning.clone(),
                None if hot_spot.toe.nodes(&model.mesh).is_empty() => {
                    "Select a node at the weld toe.".into()
                }
                None => "No path could be placed.".into(),
            });
        }
        let increment = view.current_increment().ok_or("No increment selected.")?;
        let distances = hot_spot.distances();
        let weights = hot_spot::extrapolation_weights(&distances);
        let values =
            hot_spot::evaluate_increment(&self.paths, hot_spot.component, &weights, increment)
                .ok_or("The shown increment has no stresses (S).")?;
        let (k, value) = (values.iter().enumerate())
            .filter(|(_, v)| v.hot_spot.is_finite())
            .max_by(|a, b| a.1.hot_spot.total_cmp(&b.1.hot_spot))
            .unwrap_or((0, &values[0]));
        let path = &self.paths[k];
        let reach = CURVE_REACH * distances.iter().copied().fold(0.0, f64::max);
        if self.curve.as_ref().is_none_or(|(index, _)| *index != k) {
            let points =
                hot_spot::path_curve(&model.mesh, surface_faces(model), path, reach, CURVE_POINTS);
            self.curve = Some((k, points));
        }
        let curve = self.curve.as_ref().map_or(&[][..], |(_, c)| c.as_slice());
        let values_along =
            hot_spot::readings(curve, hot_spot.component, increment).unwrap_or_default();
        let last = distances.iter().copied().fold(0.0, f64::max);
        let line = (0..=40)
            .map(|i| {
                let x = last * i as f64 / 40.0;
                [x, hot_spot::extrapolated(&distances, &value.readings, x)]
            })
            .collect();
        let end = curve.last().map_or(reach, |p| p.distance);
        let ends = [
            path.position,
            std::array::from_fn(|i| path.position[i] + path.direction[i] * end),
        ];
        Ok(HotSpotPlot {
            node: value.node,
            paths: self.paths.len(),
            component: hot_spot.component,
            distances: curve.iter().map(|p| p.distance).collect(),
            values: values_along,
            marks: Marks {
                support: distances
                    .iter()
                    .zip(&value.readings)
                    .map(|(&d, &s)| [d, s])
                    .collect(),
                line,
                hot_spot: value.hot_spot,
            },
            ends,
            warnings: self.warnings.clone(),
        })
    }
}

/// A distance without float noise, e.g. 4.8 instead of 4.800000000000001.
pub fn short(value: f64) -> String {
    let text = format!("{value:.4}");
    text.trim_end_matches('0').trim_end_matches('.').to_string()
}

#[cfg(test)]
mod tests {
    use plx_model::Region;
    use plx_results::hot_spot::{Extrapolation, HotSpotComponent};

    use super::*;
    use crate::model::load;

    fn testdata(name: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata")
            .join(name)
    }

    /// Node of the beam mesh at a point.
    fn node_at(model: &Model, point: [f64; 3]) -> u32 {
        let index = (model.mesh.coords().iter())
            .position(|p| (0..3).all(|k| (p[k] - point[k]).abs() < 1e-9))
            .unwrap();
        model.mesh.node_ids()[index]
    }

    #[test]
    fn cantilever_hot_spot_matches_beam_theory() {
        // Cantilever 100 x 10 x 10, 100 N across at its end, solved by CalculiX 2.21 with
        // C3D20R. On top the bending stress rises linearly towards the support:
        // S11 = F (100 - x) (h / 2) / I = 0.6 (100 - x), so 30 at x = 50.
        let mut results = load(&testdata("kragbalken_c3d20r.frd"), UnitSystem::Unitless)
            .unwrap()
            .model;
        let toe = [40.0, 50.0, 60.0].map(|x| node_at(&results, [x, 5.0, 10.0]));
        for (i, extrapolation) in Extrapolation::IIW[..3].iter().enumerate() {
            results.hot_spots.definitions.push(HotSpot {
                toe: Region::Nodes(toe.to_vec()),
                direction: [1.0, 0.0, 0.0],
                thickness: 10.0,
                extrapolation: extrapolation.clone(),
                component: HotSpotComponent::Perpendicular,
                ..HotSpot::new(format!("Hot_Spot-{}", i + 1))
            });
        }
        let reports = evaluate(&results);
        assert_eq!(reports.len(), 3);
        for report in &reports {
            assert!(report.warnings.is_empty(), "{:?}", report.warnings);
            let values = &report.increments[0].values;
            assert_eq!(values.len(), 3);
            for (value, x) in values.iter().zip([40.0, 50.0, 60.0]) {
                let beam = 0.6 * (100.0 - x);
                let error = (value.hot_spot - beam).abs() / beam;
                assert!(error < 0.01, "{}: {value:?} vs {beam}", report.name);
            }
        }
        let csv = hot_spot::to_csv(&reports);
        assert_eq!(csv.lines().count(), 1 + 3 * 3);
        // The paths start at the toe and run along the top in +x.
        let paths = paths(&results, &results.hot_spots.definitions[0]);
        assert_eq!(paths.len(), 3);
        assert_eq!(render_paths(&results, &paths)[0].len(), 3);
    }

    #[test]
    fn thickness_is_in_the_units_of_the_results() {
        // Results of a millimetre model: t = 500 mm gives read-out points at 200 and 500
        // (mesh units), not at 0.2 and 0.5 as if t were taken as SI metres. The dialog's
        // thickness field must therefore convert from the results' units, not from SI.
        let results = load(&testdata("kragbalken_c3d20r.frd"), UnitSystem::MmTonSC)
            .unwrap()
            .model;
        assert_eq!(units(&results), UnitSystem::MmTonSC);
        assert_eq!(results.stored_units(), units(&results));
        let hot_spot = HotSpot {
            thickness: 500.0,
            ..HotSpot::new("t")
        };
        assert_eq!(
            hot_spot.in_units(units(&results)).distances(),
            [200.0, 500.0]
        );
    }

    #[test]
    fn plot_shows_the_edge_node_with_the_highest_stress() {
        // The cantilever of the test above: an edge across the top at x = 40 to 60 picks the
        // node nearest to the support, x = 40, where the bending stress is highest.
        let results = load(&testdata("kragbalken_c3d20r.frd"), UnitSystem::Unitless)
            .unwrap()
            .model;
        let toe = [40.0, 50.0, 60.0].map(|x| node_at(&results, [x, 5.0, 10.0]));
        let hot_spot = HotSpot {
            toe: Region::Nodes(toe.to_vec()),
            direction: [1.0, 0.0, 0.0],
            thickness: 10.0,
            ..HotSpot::new("Hot_Spot-1")
        };
        let mut located = Located::default();
        let plot = located.plot(&results, &hot_spot).unwrap();
        assert_eq!(plot.node, toe[0]);
        assert_eq!(plot.paths, 3);
        assert!((plot.marks.hot_spot - 36.0).abs() / 36.0 < 0.01, "{plot:?}");
        // Read-out points at 0.4 t and 1.0 t; the dashed line starts at the hot spot stress
        // and passes through them.
        assert_eq!(plot.marks.support.len(), 2);
        let line = &plot.marks.line;
        assert_eq!(line[0], [0.0, plot.marks.hot_spot]);
        assert!((line.last().unwrap()[1] - plot.marks.support[1][1]).abs() < 1e-9);
        // The curve runs from the toe a quarter beyond the last read-out point and follows
        // the linear bending stress there.
        assert_eq!(plot.distances.len(), CURVE_POINTS);
        assert!((plot.distances.last().unwrap() - 12.5).abs() < 1e-9);
        let (d, s) = (plot.distances[40], plot.values[40]);
        let beam = 0.6 * (100.0 - 40.0 - d);
        assert!((s - beam).abs() / beam < 0.02, "{d}: {s} vs {beam}");
        assert_eq!(plot.ends[0], [40.0, 5.0, 10.0]);
        assert!((plot.ends[1][0] - 52.5).abs() < 1e-9);
        let [low, high] = plot.range();
        assert!(low <= 0.6 * 47.5 && high >= plot.marks.hot_spot);
        let csv = plot.to_csv(';');
        assert!(csv.starts_with("Distance;S_PERP\n0;"));
        assert!(csv.contains("\nRead-out distance;S_PERP\n4;"));
        // Without toe nodes there is nothing to plot.
        let empty = HotSpot::new("Hot_Spot-2");
        assert!(located.plot(&results, &empty).is_err());
    }

    #[test]
    fn output_file_sits_next_to_the_results() {
        let path = output_path(Path::new("/tmp/work/Analysis-1.frd"));
        assert_eq!(path, Path::new("/tmp/work/Analysis-1_hot_spots.csv"));
    }
}
