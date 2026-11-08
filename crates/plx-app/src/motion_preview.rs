//! Preview of the motion a displacement boundary condition prescribes over time: the nodes
//! of its region move through the time range of its amplitude or value tables in the 3D view,
//! exaggerated so that the motion is visible, while the rest of the mesh stays put.

use plx_model::{Amplitude, AmplitudeTime, Displacement, Region, ValueTable};

use crate::model::Model;
use crate::results::{model_size, round_significant};

/// Real seconds one pass through the time range takes.
const PASS_SECONDS: f64 = 3.0;
/// Rest at the end of a pass before the next one starts.
const REST_SECONDS: f64 = 0.7;
/// The largest motion is drawn this fraction of the model size, like the automatic
/// deformation scale of results.
const VISIBLE_FRACTION: f64 = 0.25;

/// A playing preview, started by the play button of the boundary condition dialog.
pub struct MotionPreview {
    started: f64,
    /// Characteristic size of the model for the scale.
    model_size: f64,
    /// The region the node indices were found for.
    nodes: Option<(Region, Vec<usize>)>,
    /// The moment drawn last, for the dialog.
    pub last: Option<Moment>,
}

/// One moment of the preview.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Moment {
    /// Time on the time axis of the amplitude or the tables.
    pub time: f64,
    /// Exaggeration of the drawn motion.
    pub scale: f64,
}

/// The translations a displacement prescribes over time: per direction the displacement as
/// a table of (time, value) points, `None` where it is free or held still.
#[derive(Clone, Debug, PartialEq)]
pub struct Motion {
    pub components: [Option<ValueTable>; 3],
    /// Where a pass starts: 0, or the first point of an amplitude in total time.
    pub start: f64,
}

impl Motion {
    /// The motion of a displacement: a translation with a table follows it, the others their
    /// value times the amplitude. `None` when nothing has a time curve to show.
    pub fn of(displacement: &Displacement, amplitude: Option<&Amplitude>) -> Option<Self> {
        let mut start = 0.0;
        let mut any = false;
        let components = std::array::from_fn(|k| {
            if let Some(table) = displacement.table(k) {
                any = true;
                return Some(table.clone());
            }
            let (value, amplitude) = (displacement.values[k]?, amplitude?);
            any = true;
            Some(ValueTable {
                points: (amplitude.points.iter())
                    .map(|&[t, a]| {
                        [
                            t + amplitude.shift_time,
                            value * (a + amplitude.shift_amplitude),
                        ]
                    })
                    .collect(),
            })
        });
        // Total time amplitudes start at their first point, step time at the step's start.
        if let Some(a) = amplitude
            && a.time_span == AmplitudeTime::Total
            && (0..3).all(|k| displacement.table(k).is_none())
        {
            start = a.points.first().map_or(0.0, |p| p[0] + a.shift_time);
        }
        any.then_some(Self { components, start })
    }

    /// The time range a preview passes through: from the start to the last point; a single
    /// point is shown over a unit of time.
    pub fn time_range(&self) -> (f64, f64) {
        let tables = self.components.iter().flatten();
        let first = (tables.clone())
            .filter_map(|t| t.points.first())
            .map(|p| p[0])
            .fold(f64::INFINITY, f64::min);
        let last = (tables.filter_map(|t| t.points.last()))
            .map(|p| p[0])
            .fold(f64::NEG_INFINITY, f64::max);
        let start = self.start.min(first);
        if last > start {
            (start, last)
        } else {
            (start, start + 1.0)
        }
    }

    /// The displacement at a time.
    pub fn at(&self, time: f64) -> [f64; 3] {
        std::array::from_fn(|k| {
            self.components[k]
                .as_ref()
                .map_or(0.0, |t| t.value_at(time))
        })
    }

    /// Largest length of the displacement; linear in between, it is reached at a point.
    fn peak(&self) -> f64 {
        let times = self
            .components
            .iter()
            .flatten()
            .flat_map(|t| t.points.iter().map(|p| p[0]));
        times
            .map(|t| self.at(t).iter().map(|v| v * v).sum::<f64>().sqrt())
            .fold(0.0, f64::max)
    }

    /// The time `elapsed` real seconds after the preview started; passes repeat.
    fn time_at(&self, elapsed: f64) -> f64 {
        let (start, end) = self.time_range();
        let phase = elapsed.max(0.0) % (PASS_SECONDS + REST_SECONDS);
        start + (end - start) * (phase / PASS_SECONDS).min(1.0)
    }
}

impl MotionPreview {
    pub fn new(now: f64, model: &Model) -> Self {
        Self {
            started: now,
            model_size: model.mesh.bounds().map_or(1.0, model_size),
            nodes: None,
            last: None,
        }
    }

    /// The drawn offsets of the region's nodes by node index at `now` (seconds of the GUI
    /// clock) for the motion.
    pub fn frame(
        &mut self,
        now: f64,
        model: &Model,
        region: Region,
        motion: &Motion,
    ) -> Vec<(usize, [f64; 3])> {
        if self.nodes.as_ref().is_none_or(|(r, _)| *r != region) {
            let mesh = &model.mesh;
            let indices = (region.nodes(mesh).into_iter())
                .filter_map(|id| mesh.node_index(id))
                .collect();
            self.nodes = Some((region, indices));
        }
        let scale = automatic_scale(self.model_size, motion.peak());
        let time = motion.time_at(now - self.started);
        let offset = motion.at(time).map(|v| v * scale);
        let nodes = self.nodes.as_ref().map_or(&[][..], |(_, n)| n);
        self.last = Some(Moment { time, scale });
        nodes.iter().map(|&n| (n, offset)).collect()
    }
}

/// Scale that draws the largest motion a quarter of the model size, rounded to two digits;
/// 1 when nothing moves.
fn automatic_scale(model_size: f64, peak: f64) -> f64 {
    if peak > 0.0 && peak.is_finite() {
        round_significant(VISIBLE_FRACTION * model_size / peak, 2)
    } else {
        1.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ramp() -> Amplitude {
        let mut amplitude = Amplitude::new("A");
        amplitude.points = vec![[0.0, 0.0], [2.0, -1.0], [4.0, 0.5]];
        amplitude
    }

    fn along_x(amplitude: &Amplitude) -> Motion {
        let displacement = Displacement::new([Some(2.0), None, None, None, None, None]);
        Motion::of(&displacement, Some(amplitude)).unwrap()
    }

    #[test]
    fn passes_through_the_time_range_and_rests_at_its_end() {
        let motion = along_x(&ramp());
        assert_eq!(motion.time_range(), (0.0, 4.0));
        assert_eq!(motion.time_at(0.0), 0.0);
        assert_eq!(motion.time_at(PASS_SECONDS / 2.0), 2.0);
        assert_eq!(motion.time_at(PASS_SECONDS + 0.1), 4.0);
        assert_eq!(motion.time_at(PASS_SECONDS + REST_SECONDS), 0.0);
    }

    #[test]
    fn step_time_starts_at_zero_and_a_single_point_spans_a_unit() {
        let mut amplitude = ramp();
        amplitude.points = vec![[1.0, 1.0], [3.0, 2.0]];
        assert_eq!(along_x(&amplitude).time_range(), (0.0, 3.0));
        amplitude.time_span = AmplitudeTime::Total;
        assert_eq!(along_x(&amplitude).time_range(), (1.0, 3.0));
        amplitude.points = vec![[0.0, 1.0]];
        assert_eq!(along_x(&amplitude).time_range(), (0.0, 1.0));
    }

    #[test]
    fn scale_draws_the_largest_motion_a_quarter_of_the_model() {
        let motion = along_x(&ramp());
        assert_eq!(motion.peak(), 2.0);
        // Model size 100, 2 mm at the peak: 25 / 2.
        assert_eq!(automatic_scale(100.0, motion.peak()), 13.0);
        assert_eq!(automatic_scale(100.0, 0.0), 1.0);
    }

    #[test]
    fn tables_move_each_direction_by_themselves() {
        // The motion of the issue: down 0.8 mm, then 4 mm to the side and up to 0.8 mm.
        let mut displacement = Displacement::new([Some(0.0), Some(0.0), None, None, None, None]);
        displacement.tables[0] = Some(ValueTable {
            points: vec![[0.0, 0.0], [1.0, 0.0], [2.0, 4.0]],
        });
        displacement.tables[1] = Some(ValueTable {
            points: vec![[0.0, 0.0], [1.0, -0.8], [2.0, 0.8]],
        });
        let motion = Motion::of(&displacement, None).unwrap();
        assert_eq!(motion.time_range(), (0.0, 2.0));
        assert_eq!(motion.at(1.0), [0.0, -0.8, 0.0]);
        let middle = motion.at(1.5);
        assert!((middle[0] - 2.0).abs() < 1e-12 && middle[1].abs() < 1e-12);
        // Without an amplitude or a table there is nothing to show.
        let still = Displacement::new([Some(1.0); 6]);
        assert!(Motion::of(&still, None).is_none());
    }
}
