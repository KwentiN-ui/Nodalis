//! The loads of the material on the walls of a Particle Flow run, as the run writes them
//! beside its results (`<job>.walls`): per wall and frame the points on its surface with the
//! mean force on each since the frame before, in the wall's own frame
//! ([`plxs_core::particles::WallFrame`]). FE steps take them as nodal forces on the faces of
//! the tool parts (the Wall loads input of a step).

use std::path::Path;

use plxs_core::particles::{Space, WallFrame};
use serde::{Deserialize, Serialize};

/// The extension of the file.
pub const EXTENSION: &str = "walls";

/// The first bytes of the file.
const MAGIC: &[u8; 8] = b"NDLWALL1";

/// The loads on the walls of a run.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct WallLoads {
    /// The cell of the grid: points lie up to two cells inside the wall's surface, and
    /// neighbouring points one cell apart.
    pub cell: f64,
    pub space: Space,
    /// The time of each frame after the start; frame `k` holds the mean since frame `k - 1`
    /// (since the start for the first).
    pub times: Vec<f64>,
    pub walls: Vec<WallSeries>,
}

/// One wall over the frames.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct WallSeries {
    /// The name of the rigid body (or boundary condition) the wall is.
    pub name: String,
    /// The names of the FE parts it is made of.
    pub parts: Vec<String>,
    /// One per time of [`WallLoads::times`].
    pub frames: Vec<WallFrame>,
}

/// Which time of the run the loads are taken at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Pick {
    /// The frame nearest the time.
    At(f64),
    /// The mean over the frames between the times.
    Mean(f64, f64),
}

/// Point loads, each a point, the outward normal of the wall there and a force.
pub type PointLoads = Vec<([f64; 3], [f64; 3], [f64; 3])>;

impl WallLoads {
    pub fn new(cell: f64, space: Space, walls: Vec<(String, Vec<String>)>) -> Self {
        Self {
            cell,
            space,
            times: Vec::new(),
            walls: (walls.into_iter())
                .map(|(name, parts)| WallSeries {
                    name,
                    parts,
                    frames: Vec::new(),
                })
                .collect(),
        }
    }

    /// Adds a frame at `time`: its loads, one per wall (the first frame, at the start, has
    /// none and is left out).
    pub fn push(&mut self, time: f64, walls: &[WallFrame]) {
        if walls.len() != self.walls.len() || self.walls.is_empty() {
            return;
        }
        self.times.push(time);
        for (series, frame) in self.walls.iter_mut().zip(walls) {
            series.frames.push(frame.clone());
        }
    }

    /// The time of the last frame, 0 without any.
    pub fn end_time(&self) -> f64 {
        self.times.last().copied().unwrap_or(0.0)
    }

    /// The frames `pick` takes and the weight of each (the share of the time it stands for).
    pub fn frames(&self, pick: Pick) -> Vec<(usize, f64)> {
        let n = self.times.len();
        if n == 0 {
            return Vec::new();
        }
        let start = |k: usize| if k == 0 { 0.0 } else { self.times[k - 1] };
        match pick {
            Pick::At(t) => {
                let k = (0..n)
                    .min_by(|&a, &b| {
                        (self.times[a] - t)
                            .abs()
                            .total_cmp(&(self.times[b] - t).abs())
                    })
                    .unwrap_or(n - 1);
                vec![(k, 1.0)]
            }
            Pick::Mean(from, to) => {
                let (from, to) = (from.min(to), from.max(to));
                let mut out: Vec<(usize, f64)> = (0..n)
                    .filter_map(|k| {
                        let overlap = self.times[k].min(to) - start(k).max(from);
                        (overlap > 0.0).then_some((k, overlap))
                    })
                    .collect();
                let total: f64 = out.iter().map(|(_, w)| w).sum();
                if total <= 0.0 {
                    // A span within one frame, or none: the frame nearest.
                    return self.frames(Pick::At(0.5 * (from + to)));
                }
                out.iter_mut().for_each(|(_, w)| *w /= total);
                out
            }
        }
    }

    /// The point loads on wall `w` of the frames picked, weighted: the points of the
    /// frames one after the other.
    pub fn point_loads(&self, w: usize, frames: &[(usize, f64)]) -> PointLoads {
        let Some(series) = self.walls.get(w) else {
            return Vec::new();
        };
        let wide = |v: &[f32; 3]| v.map(f64::from);
        let mut out = Vec::new();
        for &(k, weight) in frames {
            let Some(frame) = series.frames.get(k) else {
                continue;
            };
            for ((p, n), f) in frame.points.iter().zip(&frame.normals).zip(&frame.forces) {
                out.push((wide(p), wide(n), wide(f).map(|v| v * weight)));
            }
        }
        out
    }

    /// The total force on wall `w` in the frames picked.
    pub fn total_force(&self, w: usize, frames: &[(usize, f64)]) -> [f64; 3] {
        let mut sum = [0.0; 3];
        for (_, _, f) in self.point_loads(w, frames) {
            (0..3).for_each(|a| sum[a] += f[a]);
        }
        sum
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = MAGIC.to_vec();
        out.extend(postcard::to_stdvec(self).unwrap_or_default());
        out
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, String> {
        let body = bytes
            .strip_prefix(MAGIC.as_slice())
            .ok_or_else(|| "not a file of wall loads".to_string())?;
        postcard::from_bytes(body).map_err(|e| e.to_string())
    }

    pub fn write(&self, path: &Path) -> std::io::Result<()> {
        std::fs::write(path, self.to_bytes())
    }

    pub fn read(path: &Path) -> Result<Self, String> {
        let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Self::from_bytes(&bytes).map_err(|e| format!("{}: {e}", path.display()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn loads() -> WallLoads {
        let mut loads = WallLoads::new(0.01, Space::Solid, vec![("Screw".into(), vec![])]);
        for (t, f) in [(0.1, 1.0f32), (0.2, 3.0), (0.4, 5.0)] {
            loads.push(
                t,
                &[WallFrame {
                    points: vec![[0.0; 3]],
                    normals: vec![[0.0, 0.0, 1.0]],
                    forces: vec![[0.0, 0.0, f]],
                }],
            );
        }
        loads
    }

    #[test]
    fn a_mean_weighs_the_frames_by_their_time() {
        let loads = loads();
        assert_eq!(loads.frames(Pick::At(0.18)), vec![(1, 1.0)]);
        // From 0.15 to 0.4: half of frame 1 (0.05 s) and frame 2 (0.2 s).
        let frames = loads.frames(Pick::Mean(0.15, 0.4));
        assert_eq!(frames.len(), 2);
        let force = loads.total_force(0, &frames)[2];
        assert!(
            (force - (0.05 * 3.0 + 0.2 * 5.0) / 0.25).abs() < 1e-9,
            "{force}"
        );
        let back = WallLoads::from_bytes(&loads.to_bytes()).unwrap();
        assert_eq!(back, loads);
    }
}
