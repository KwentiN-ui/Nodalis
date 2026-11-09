//! Campbell diagram of a rotor: the complex eigenfrequencies over the rotational speed.
//!
//! A single Complex Frequency step computes the whirling modes for one speed only.
//! The sweep runs the analysis once per speed, scaling the centrifugal loads of the model,
//! reads the complex frequencies and the turning direction of every mode from the `.dat`
//! file and plots them over the speed, with the engine order lines (1x, 2x, ...) whose
//! crossings with the modes are the critical speeds.
//!
//! The sweep is adaptive: after a coarse grid over the speed range it keeps adding speeds
//! where the curves bend most, until it is stopped or reaches its number of runs. The modes
//! are followed from speed to speed by their course, not by their rank, so a spurious root
//! the solver finds at one speed does not shift every curve above it.

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::Duration;

use egui::{Color32, Ui};
pub use plx_model::campbell::{Frame, Mode, Setup, SpeedPoint, Whirl};
use plx_model::{FeModel, StepKind};
use plx_solve::JobStatus;
use plx_solve::Run;

use crate::numeric;
use crate::table_export;
use crate::xy_plot;

/// Speeds of the coarse grid the sweep starts with.
const GRID: usize = 8;

/// Smallest gap between two speeds, as a fraction of the speed range. It also bounds the runs
/// spent on a jump of a curve, which no refinement smooths.
const MIN_GAP: f64 = 2e-3;

/// Weight of the gap width in the refinement score: once the curves bend less than about a
/// hundredth of the frequency axis over a gap, the speeds spread over the whole range.
const WIDTH_WEIGHT: f64 = 0.01;

/// Fewest speeds a mode must be found at to count as a curve; shorter runs are drawn as
/// unmatched points.
const MIN_TRACK: usize = 3;

/// Speeds in a row a curve may be missing at, as when a spurious root takes its place among
/// the modes the solver computes, before it counts as ended.
const MAX_MISSED: usize = 2;

/// Which line of a mode crosses an engine order: the mode as the solver gives it in the
/// rotating frame, or one of its travelling waves in the stationary frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Branch {
    Rotating,
    Forward,
    Backward,
}

/// A crossing of a mode with an engine order line.
#[derive(Clone, Debug, PartialEq)]
pub struct CriticalSpeed {
    /// The engine order; 0 in the stationary frame where a backward wave comes to rest, a
    /// standing wave every load at rest excites.
    pub order: f64,
    /// Mode number, from 1.
    pub mode: usize,
    /// Rotational speed in radians per time.
    pub speed: f64,
    pub whirl: Whirl,
    pub branch: Branch,
    pub nodal_diameter: Option<u32>,
}

impl CriticalSpeed {
    /// The frequency of the crossing in Hz.
    pub fn frequency(&self) -> f64 {
        self.order * self.speed / std::f64::consts::TAU
    }

    /// The order as the diagram labels it: "2x", or "standing" for a wave at rest.
    pub fn order_label(&self) -> String {
        if self.order == 0.0 {
            "standing".into()
        } else {
            format!("{}x", numeric::format_physical(self.order))
        }
    }

    /// The branch as the table names it.
    pub fn branch_label(&self) -> String {
        let diameter = match self.nodal_diameter {
            Some(n) => format!(", ND {n}"),
            None => String::new(),
        };
        match self.branch {
            Branch::Rotating => match self.whirl {
                Whirl::Forward => format!("forward{diameter}"),
                Whirl::Backward => format!("backward{diameter}"),
                Whirl::Unknown => format!("-{diameter}"),
            },
            Branch::Forward => format!("forward wave{diameter}"),
            Branch::Backward => format!("backward wave{diameter}"),
        }
    }
}

/// The engine orders of `orders`, e.g. "1, 2", as numbers, in order and without repeats.
pub fn order_values(orders: &str) -> Vec<f64> {
    let mut orders: Vec<f64> = orders
        .split([',', ';', ' '])
        .filter_map(|t| numeric::parse_number(t.trim()))
        .filter(|o| *o > 0.0 && o.is_finite())
        .collect();
    orders.sort_by(f64::total_cmp);
    orders.dedup();
    orders
}

/// The rotational speed of the model: that of its fastest active complex frequency step,
/// `None` without one.
pub fn rotor_speed(fe: &FeModel) -> Option<f64> {
    (fe.steps.iter().filter(|s| s.active))
        .filter_map(|s| match &s.kind {
            StepKind::ComplexFrequency(c) => Some(c.speed.abs()),
            _ => None,
        })
        .reduce(f64::max)
}

/// Index in the model of the last active complex frequency step.
pub fn complex_step(fe: &FeModel) -> Option<usize> {
    (fe.steps.iter().enumerate())
        .filter(|(_, s)| s.active && matches!(s.kind, StepKind::ComplexFrequency(_)))
        .map(|(i, _)| i)
        .next_back()
}

/// Number (from 1) of the solver step the model's step `step` became: the last one made
/// of it, after those that prepare it.
fn solver_step(lowered: &plx_io::inp::Lowered, step: usize) -> Option<u32> {
    (0..lowered.analysis.steps.len())
        .filter(|i| {
            lowered.origins.get(&plxs_core::ItemRef::Step(*i))
                == Some(plx_model::ModelItem::Step(step))
        })
        .max()
        .map(|i| i as u32 + 1)
}

/// Whether the model can have a Campbell diagram: it has a complex frequency step.
pub fn applies(fe: &FeModel) -> bool {
    complex_step(fe).is_some()
}

/// The model with the speed of every complex frequency step scaled from the model's speed
/// to `speed`; at standstill every one turns at `speed`.
pub fn with_speed(fe: &FeModel, speed: f64) -> FeModel {
    let mut fe = fe.clone();
    set_speed(&mut fe, speed);
    fe
}

/// Sets the model to turn at `speed`, as [`with_speed`] does: the speed of every complex
/// frequency step scaled from the model's speed, or at standstill turning at `speed` itself.
pub fn set_speed(fe: &mut FeModel, speed: f64) {
    // A model at standstill has no speed to scale from: its steps take the speed itself.
    // The ratio comes first, so that the fastest step turns at exactly `speed` and the
    // fingerprint of a sweep does not change with the speed the model is set to.
    let reference = rotor_speed(fe).filter(|r| *r > 0.0);
    for step in &mut fe.steps {
        if let StepKind::ComplexFrequency(c) = &mut step.kind {
            c.speed = match reference {
                Some(reference) => speed * (c.speed / reference),
                None => speed,
            };
        }
    }
}

/// Fingerprint of what a sweep computes with: the mesh and the model but not the speed of
/// its centrifugal loads, which the sweep sets itself.
pub fn sweep_fingerprint(mesh: &plx_mesh::FeMesh, fe: &FeModel) -> u64 {
    plx_model::fingerprint::solve_input(mesh, &with_speed(fe, 1.0))
}

/// Whether an eigenvalue with real part `real` and imaginary part `imaginary` (both in
/// radians per time) belongs to a growing mode. Stable modes have an imaginary part of
/// rounding noise only.
fn is_unstable(real: f64, imaginary: f64) -> bool {
    imaginary.abs() > 1e-4 * real.abs().max(1.0)
}

/// The complex frequencies and whirls of step `step` in the text of a `.dat` file.
#[cfg(test)]
fn modes_of(dat: &str, step: u32) -> Vec<Mode> {
    modes_of_history(&plx_io::dat::parse_dat(dat), step)
}

/// The complex frequencies and whirls of step `step` in history output.
pub fn modes_of_history(import: &plx_io::dat::DatImport, step: u32) -> Vec<Mode> {
    let name = format!("STEP_{step}");
    let Some(set) = import.sets.iter().find(|s| s.name == name) else {
        return Vec::new();
    };
    let Some(output) = set.field("EIGENVALUE_OUTPUT") else {
        return Vec::new();
    };
    let values = |component: &str| -> Vec<f64> {
        output
            .component(component)
            .and_then(|c| c.entries.first())
            .map(|e| e.values.clone())
            .unwrap_or_default()
    };
    let frequencies = values("FREQUENCY");
    let whirls = values("TURNING_DIRECTION");
    let omegas = values("OMEGA");
    let imaginary = values("OMEGA_IM");
    let diameters = values("NODAL_DIAMETER");
    (frequencies.iter().enumerate())
        .map(|(i, &frequency)| Mode {
            frequency,
            whirl: match whirls.get(i) {
                Some(w) if *w > 0.0 => Whirl::Forward,
                Some(w) if *w < 0.0 => Whirl::Backward,
                _ => Whirl::Unknown,
            },
            unstable: match (omegas.get(i), imaginary.get(i)) {
                (Some(&real), Some(&imaginary)) => is_unstable(real, imaginary),
                _ => false,
            },
            nodal_diameter: diameters.get(i).filter(|d| **d >= 0.0).map(|d| *d as u32),
        })
        .collect()
}

/// The complex frequencies of step `step` from the `.frd` file, for when the `.dat` file has
/// none CalculiX versions print it differently. The `.frd` file has no whirl directions.
pub fn modes_of_frd(frd: &plx_io::frd::FrdImport, step: u32) -> Vec<Mode> {
    let mut modes: Vec<(u32, f64)> = (frd.increments.iter())
        .filter(|i| i.step == step && i.kind == plx_results::AnalysisKind::ComplexFrequency)
        .map(|i| (i.increment, i.value))
        .collect();
    modes.sort_by_key(|&(mode, _)| mode);
    modes.dedup_by_key(|&mut (mode, _)| mode);
    (modes.into_iter())
        .map(|(_, frequency)| Mode {
            frequency,
            whirl: Whirl::Unknown,
            unstable: false,
            nodal_diameter: None,
        })
        .collect()
}

/// The coarse grid a sweep starts with: `count` speeds spread evenly over the range, both
/// ends included. A range from standstill starts at a fiftieth of the highest speed instead:
/// without rotation CalculiX merges the coinciding modes of a pair into one, which would
/// shift every curve of the diagram at its start.
pub fn grid(min_speed: f64, max_speed: f64, count: usize) -> Vec<f64> {
    let low = if min_speed > 0.0 {
        min_speed
    } else {
        max_speed / 50.0
    };
    if count <= 1 {
        return vec![max_speed];
    }
    (0..count)
        .map(|i| low + (max_speed - low) * i as f64 / (count - 1) as f64)
        .collect()
}

/// The speeds that extend a sweep with `points` to a changed range `min_speed..=max_speed`:
/// a coarse grid over each part of the range below or above the speeds it has, without the
/// speed it joins. Empty when the range lies within the speeds, and the whole grid of the
/// range when there are none. Up to `count` speeds go into each part, fewer into a narrow
/// one.
pub fn extension(points: &[SpeedPoint], min_speed: f64, max_speed: f64, count: usize) -> Vec<f64> {
    let range = max_speed - min_speed;
    if !(range.is_finite() && range > 0.0) {
        return Vec::new();
    }
    let (low, high) = (points.iter().map(|p| p.speed))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), s| {
            (a.min(s), b.max(s))
        });
    if low > high {
        return grid(min_speed, max_speed, count);
    }
    // Where the grid of the range would start: standstill itself is left out.
    let first = grid(min_speed, max_speed, 2)[0];
    let tolerance = 2.0 * MIN_GAP * range;
    let speeds_in = |width: f64| {
        let share = (count as f64 * width / range).ceil() as usize;
        share.clamp(1, count.max(1)) + 1
    };
    let mut speeds = Vec::new();
    if first < low - tolerance {
        let part = grid(first, low, speeds_in(low - first));
        speeds.extend(&part[..part.len() - 1]);
    }
    if max_speed > high + tolerance {
        let part = grid(high, max_speed, speeds_in(max_speed - high));
        speeds.extend(&part[1..]);
    }
    speeds
}

/// Radians per time to revolutions per minute, for a time unit of seconds.
pub fn rpm(speed: f64) -> f64 {
    speed * 60.0 / std::f64::consts::TAU
}

/// Revolutions per minute to radians per time.
pub fn from_rpm(rpm: f64) -> f64 {
    rpm * std::f64::consts::TAU / 60.0
}

/// One mode followed over the speed: its speeds in increasing order with the mode there.
#[derive(Clone, Debug, PartialEq)]
pub struct Track {
    pub name: String,
    pub points: Vec<(f64, Mode)>,
}

impl Track {
    /// The whirl if it is the same at every speed and known.
    fn whirl(&self) -> Whirl {
        let mut whirls = self.points.iter().map(|(_, m)| m.whirl);
        match whirls.next() {
            Some(first) if whirls.all(|w| w == first) => first,
            _ => Whirl::Unknown,
        }
    }

    /// The number of nodal diameters most of the speeds agree on, `None` when fewer than
    /// half of them have one or they disagree.
    pub fn nodal_diameter(&self) -> Option<u32> {
        let mut counts: Vec<(u32, usize)> = Vec::new();
        for (_, mode) in &self.points {
            if let Some(n) = mode.nodal_diameter {
                match counts.iter_mut().find(|(d, _)| *d == n) {
                    Some((_, c)) => *c += 1,
                    None => counts.push((n, 1)),
                }
            }
        }
        let (n, count) = counts.into_iter().max_by_key(|(_, c)| *c)?;
        (2 * count >= self.points.len()).then_some(n)
    }

    /// The number of the mode in its name, "Mode 3 (F, ND 2)" being 3.
    fn number(&self) -> &str {
        self.name.split(' ').nth(1).unwrap_or_default()
    }

    /// The frequency of the mode at `speed` in the stationary frame: the forward wave
    /// `f + n Ω` and the backward wave `f − n Ω` (negative once it has come to rest and
    /// turned around), `None` without a nodal diameter.
    fn waves(&self, speed: f64, mode: &Mode) -> Option<(f64, f64)> {
        let n = f64::from(self.nodal_diameter()?);
        let shift = n * speed / std::f64::consts::TAU;
        Some((mode.frequency + shift, mode.frequency - shift))
    }
}

/// The modes of a sweep sorted into curves.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Tracks {
    /// The curves, named "Mode 1", "Mode 2", ... by their frequency where they start.
    pub modes: Vec<Track>,
    /// Roots found at too few neighbouring speeds to form a curve: (speed, frequency).
    pub unmatched: Vec<(f64, f64)>,
    /// Growing modes: (speed, frequency).
    pub unstable: Vec<(f64, f64)>,
}

/// Where a curve is heading at `speed`: the line through its last two points, or its last
/// value when it has one point.
fn predict(points: &[(f64, Mode)], speed: f64) -> f64 {
    match points {
        [.., (s0, m0), (s1, m1)] if s1 > s0 => {
            m1.frequency + (m1.frequency - m0.frequency) / (s1 - s0) * (speed - s1)
        }
        [.., (_, m)] => m.frequency,
        [] => f64::NAN,
    }
}

/// Sorts the modes of the speeds into curves. Going up in speed, each mode joins the curve
/// whose course it continues best, the closest pairs first; a mode that continues none starts
/// a new curve. Curves found at fewer than three speeds and growing modes are kept apart, so
/// a spurious root does not tear the curves.
pub fn track(points: &[SpeedPoint]) -> Tracks {
    let mut order: Vec<&SpeedPoint> = points.iter().collect();
    order.sort_by(|a, b| a.speed.total_cmp(&b.speed));
    let mut tracks: Vec<Vec<(f64, Mode)>> = Vec::new();
    // Index into `order` of the last speed each curve was found at.
    let mut last_seen: Vec<usize> = Vec::new();
    let mut unstable = Vec::new();
    for (index, point) in order.into_iter().enumerate() {
        let (growing, modes): (Vec<&Mode>, Vec<&Mode>) =
            point.modes.iter().partition(|m| m.unstable);
        unstable.extend(growing.iter().map(|m| (point.speed, m.frequency)));
        let scale = modes.iter().map(|m| m.frequency.abs()).fold(0.0, f64::max);
        let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
        for (t, track) in tracks.iter().enumerate() {
            // A curve missing at more than two speeds in a row has ended.
            if index - last_seen[t] > MAX_MISSED + 1 {
                continue;
            }
            let expected = predict(track, point.speed);
            // A curve takes a mode within a fifth of its value or a few percent of the
            // spectrum, so curves near zero still find their continuation.
            let tolerance = (0.2 * expected.abs()).max(0.05 * scale);
            for (m, mode) in modes.iter().enumerate() {
                // The solver gives the size of a frequency: a curve heading below zero
                // comes back up mirrored.
                let distance = (mode.frequency.abs() - expected.abs()).abs();
                if distance <= tolerance {
                    pairs.push((distance, t, m));
                }
            }
        }
        pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut track_taken = vec![false; tracks.len()];
        let mut mode_taken = vec![false; modes.len()];
        for (_, t, m) in pairs {
            if track_taken[t] || mode_taken[m] {
                continue;
            }
            track_taken[t] = true;
            mode_taken[m] = true;
            tracks[t].push((point.speed, modes[m].clone()));
            last_seen[t] = index;
        }
        for (m, mode) in modes.iter().enumerate() {
            if !mode_taken[m] {
                tracks.push(vec![(point.speed, (*mode).clone())]);
                last_seen.push(index);
            }
        }
    }
    let needed = MIN_TRACK.min(points.len());
    let (long, short): (Vec<_>, Vec<_>) = tracks.into_iter().partition(|t| t.len() >= needed);
    // Named by where they start: the lowest speed first, then the lowest frequency.
    let mut long: Vec<Vec<(f64, Mode)>> = long;
    long.sort_by(|a, b| {
        (a[0].0.total_cmp(&b[0].0)).then(a[0].1.frequency.total_cmp(&b[0].1.frequency))
    });
    let modes = (long.into_iter().enumerate())
        .map(|(i, points)| {
            let mut track = Track {
                name: String::new(),
                points,
            };
            let mut marks: Vec<String> = Vec::new();
            if let whirl @ (Whirl::Forward | Whirl::Backward) = track.whirl() {
                marks.push(whirl.letter().into());
            }
            if let Some(n) = track.nodal_diameter() {
                marks.push(format!("ND {n}"));
            }
            track.name = if marks.is_empty() {
                format!("Mode {}", i + 1)
            } else {
                format!("Mode {} ({})", i + 1, marks.join(", "))
            };
            track
        })
        .collect();
    let unmatched = (short.iter().flatten())
        .map(|(s, m)| (*s, m.frequency))
        .collect();
    Tracks {
        modes,
        unmatched,
        unstable,
    }
}

/// The crossings of the curves with the engine order lines, by linear interpolation between
/// neighbouring speeds of a curve. In the stationary frame the travelling waves of every
/// mode with a nodal diameter cross the lines, and a backward wave coming to rest (the
/// line of order 0) is a critical speed of its own: a standing wave.
pub fn critical_speeds(tracks: &Tracks, orders: &[f64], frame: Frame) -> Vec<CriticalSpeed> {
    let mut found = Vec::new();
    let mut add = |order: f64,
                   index: usize,
                   branch: Branch,
                   track: &Track,
                   value: &dyn Fn(f64, &Mode) -> f64| {
        for pair in track.points.windows(2) {
            let ((sa, a), (sb, b)) = (&pair[0], &pair[1]);
            let line = |speed: f64| order * speed / std::f64::consts::TAU;
            let (da, db) = (value(*sa, a) - line(*sa), value(*sb, b) - line(*sb));
            // A crossing has the mode above the line at one end and below at the other;
            // standstill itself (speed 0, line 0) is no critical speed, and neither is a
            // curve running along the line within rounding, such as the rigid tilt of a
            // free rotor at exactly 1x.
            let along = 1e-6 * (value(*sa, a).abs() + value(*sb, b).abs() + 1.0);
            if da == db
                || da * db > 0.0
                || (*sa == 0.0 && da == 0.0)
                || (da.abs() <= along && db.abs() <= along)
            {
                continue;
            }
            let t = da / (da - db);
            let speed = sa + t * (sb - sa);
            if speed <= 0.0 {
                continue;
            }
            found.push(CriticalSpeed {
                order,
                mode: index + 1,
                speed,
                whirl: if t < 0.5 { a.whirl } else { b.whirl },
                branch,
                nodal_diameter: track.nodal_diameter(),
            });
        }
    };
    for (index, track) in tracks.modes.iter().enumerate() {
        match frame {
            Frame::Rotating => {
                for &order in orders {
                    add(order, index, Branch::Rotating, track, &|_, m| m.frequency);
                }
            }
            Frame::Stationary => {
                let Some(n) = track.nodal_diameter() else {
                    continue;
                };
                let forward = |s: f64, m: &Mode| track.waves(s, m).map_or(f64::NAN, |w| w.0);
                let backward = |s: f64, m: &Mode| track.waves(s, m).map_or(f64::NAN, |w| w.1);
                for &order in orders {
                    add(order, index, Branch::Forward, track, &forward);
                    if n > 0 {
                        add(order, index, Branch::Backward, track, &backward);
                    }
                }
                if n > 0 {
                    add(0.0, index, Branch::Backward, track, &backward);
                }
            }
        }
    }
    found.sort_by(|a, b| a.speed.total_cmp(&b.speed));
    found
}

/// How far each point of the curves lies off the straight line between its neighbours on
/// the curve, as (speed, deviation in Hz): the larger, the more the curve bends there.
fn bends(tracks: &Tracks) -> Vec<(f64, f64)> {
    let mut bends = Vec::new();
    for track in &tracks.modes {
        for window in track.points.windows(3) {
            let [(sa, a), (sb, b), (sc, c)] = window else {
                continue;
            };
            if sc <= sa {
                continue;
            }
            let chord = a.frequency + (c.frequency - a.frequency) * (sb - sa) / (sc - sa);
            bends.push((*sb, (b.frequency - chord).abs()));
        }
    }
    bends
}

/// The next speed to run within `min_speed..=max_speed`, the whole sweep or a range the user
/// marked: the middle of the gap between neighbouring speeds where the curves bend most,
/// weighted with the width of the gap. The resolution is relative to the range, so a marked
/// range is sampled finer. Gaps next to a failed or running speed and
/// gaps narrower than twice the resolution are left alone, so the sweep does not chase
/// a speed the solver cannot solve. `None` when no gap is left to split.
pub fn next_speed(
    points: &[SpeedPoint],
    failed: &[f64],
    running: &[f64],
    min_speed: f64,
    max_speed: f64,
) -> Option<f64> {
    let range = max_speed - min_speed;
    if range.is_nan() || range <= 0.0 {
        return None;
    }
    let tracks = track(points);
    let bends = bends(&tracks);
    let scale = (tracks.modes.iter().flat_map(|t| &t.points))
        .map(|(_, m)| m.frequency.abs())
        .fold(0.0, f64::max)
        .max(1e-30);
    // Every speed taken, with whether it gave modes.
    let mut taken: Vec<(f64, bool)> = (points.iter().map(|p| (p.speed, true)))
        .chain(failed.iter().chain(running).map(|&s| (s, false)))
        .collect();
    taken.sort_by(|a, b| a.0.total_cmp(&b.0));
    let bend_at = |speed: f64| {
        (bends.iter())
            .filter(|(s, _)| *s == speed)
            .map(|(_, d)| *d)
            .fold(0.0, f64::max)
    };
    let mut best: Option<(f64, f64)> = None;
    for pair in taken.windows(2) {
        let ((a, a_ok), (b, b_ok)) = (pair[0], pair[1]);
        // Only the part of the gap within the range counts, so that a narrow range between
        // two speeds is refined on its own.
        let (low, high) = (a.max(min_speed), b.min(max_speed));
        let width = (high - low) / range;
        if !(a_ok && b_ok) || width < 2.0 * MIN_GAP {
            continue;
        }
        let bend = bend_at(a).max(bend_at(b)) / scale;
        let score = bend + WIDTH_WEIGHT * width;
        if best.is_none_or(|(s, _)| score > s) {
            best = Some((score, 0.5 * (low + high)));
        }
    }
    best.map(|(_, speed)| speed)
}

/// A range marked with the mouse, rounded to a power of ten of at most a hundredth of its
/// width.
pub fn round_range(low: f64, high: f64) -> [f64; 2] {
    let width = high - low;
    if !(width.is_finite() && width > 0.0) {
        return [low, high];
    }
    let step = 10f64.powf((width / 1000.0).log10().ceil());
    [(low / step).round() * step, (high / step).round() * step]
}

/// A curve of the diagram, in rpm over Hz.
#[derive(Clone, Debug, PartialEq)]
pub struct Curve {
    pub name: String,
    pub points: Vec<[f64; 2]>,
    /// Index of the mode the curve belongs to, which gives its colour.
    pub mode: usize,
    /// A backward travelling wave, drawn dashed.
    pub dashed: bool,
}

/// What the diagram draws, in rpm over Hz.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Diagram {
    pub curves: Vec<Curve>,
    pub unmatched: Vec<[f64; 2]>,
    pub unstable: Vec<[f64; 2]>,
    /// Engine order lines from standstill to the highest speed.
    pub guides: Vec<(String, Vec<[f64; 2]>)>,
    /// Modes left out of the stationary frame for want of a nodal diameter.
    pub left_out: Vec<String>,
}

/// The diagram of the curves, with the engine order lines from standstill to `max_speed`.
/// In the stationary frame every mode with a nodal diameter `n` becomes its forward wave
/// `f + n Ω` and, for `n > 0`, its backward wave `|f − n Ω|`; the others are left out and
/// named.
pub fn diagram(tracks: &Tracks, orders: &[f64], max_speed: f64, frame: Frame) -> Diagram {
    let to_plot = |&(s, f): &(f64, f64)| [rpm(s), f];
    let mut curves = Vec::new();
    let mut left_out = Vec::new();
    for (index, t) in tracks.modes.iter().enumerate() {
        match frame {
            Frame::Rotating => curves.push(Curve {
                name: t.name.clone(),
                points: t
                    .points
                    .iter()
                    .map(|(s, m)| [rpm(*s), m.frequency])
                    .collect(),
                mode: index,
                dashed: false,
            }),
            Frame::Stationary => {
                let Some(n) = t.nodal_diameter() else {
                    left_out.push(t.name.clone());
                    continue;
                };
                let waves: Vec<(f64, (f64, f64))> = (t.points.iter())
                    .filter_map(|(s, m)| Some((rpm(*s), t.waves(*s, m)?)))
                    .collect();
                if n == 0 {
                    curves.push(Curve {
                        name: t.name.clone(),
                        points: waves.iter().map(|(s, w)| [*s, w.0]).collect(),
                        mode: index,
                        dashed: false,
                    });
                    continue;
                }
                let number = t.number();
                curves.push(Curve {
                    name: format!("Mode {number} forward (ND {n})"),
                    points: waves.iter().map(|(s, w)| [*s, w.0]).collect(),
                    mode: index,
                    dashed: false,
                });
                curves.push(Curve {
                    name: format!("Mode {number} backward (ND {n})"),
                    points: waves.iter().map(|(s, w)| [*s, w.1.abs()]).collect(),
                    mode: index,
                    dashed: true,
                });
            }
        }
    }
    Diagram {
        curves,
        left_out,
        unmatched: tracks.unmatched.iter().map(to_plot).collect(),
        unstable: tracks.unstable.iter().map(to_plot).collect(),
        guides: (orders.iter())
            .map(|&order| {
                let end = [rpm(max_speed), order * max_speed / std::f64::consts::TAU];
                (
                    format!("{}x", numeric::format_physical(order)),
                    vec![[0.0, 0.0], end],
                )
            })
            .collect(),
    }
}

/// The sweep as a table for a spreadsheet, its cells separated by `separator`: the speed,
/// the frequency and whirl of each curve, the other roots the solver found at that speed and
/// the frequency of each engine order line.
pub fn table_text(points: &[SpeedPoint], orders: &[f64], separator: char) -> String {
    let tracks = track(points);
    let mut speeds: Vec<f64> = points.iter().map(|p| p.speed).collect();
    speeds.sort_by(f64::total_cmp);
    speeds.dedup();
    let mut cells = vec!["Speed [rad/s]".to_string(), "Speed [rpm]".to_string()];
    for track in &tracks.modes {
        let number = track.number();
        cells.push(format!("Mode {number} [Hz]"));
        cells.push(format!("Whirl {number}"));
        cells.push(format!("ND {number}"));
    }
    cells.push("Other roots [Hz]".into());
    for &order in orders {
        cells.push(format!("{}x [Hz]", numeric::format_physical(order)));
    }
    let mut text = String::new();
    let mut row = |cells: &mut Vec<String>| {
        text.push_str(&cells.join(&separator.to_string()));
        text.push('\n');
        cells.clear();
    };
    row(&mut cells);
    for speed in speeds {
        cells.push(numeric::format_physical(speed));
        cells.push(numeric::format_physical(rpm(speed)));
        for track in &tracks.modes {
            match track.points.iter().find(|(s, _)| *s == speed) {
                Some((_, m)) => {
                    cells.push(numeric::format_physical(m.frequency));
                    cells.push(m.whirl.letter().to_string());
                    cells.push(m.nodal_diameter.map_or(String::new(), |n| n.to_string()));
                }
                None => cells.extend([String::new(), String::new(), String::new()]),
            }
        }
        let others: Vec<String> = (tracks.unmatched.iter().chain(&tracks.unstable))
            .filter(|(s, _)| *s == speed)
            .map(|(_, f)| numeric::format_physical(*f))
            .collect();
        cells.push(others.join(" "));
        for &order in orders {
            cells.push(numeric::format_physical(
                order * speed / std::f64::consts::TAU,
            ));
        }
        row(&mut cells);
    }
    text
}

/// A speed is done, with its modes or the reason it failed.
struct Done {
    speed: f64,
    result: Result<SpeedPoint, String>,
}

/// The running sweep: the solver runs in threads, as many speeds at a time as the solver
/// settings allow.
struct Sweep {
    input: Arc<SweepInput>,
    sender: Sender<Done>,
    receiver: Receiver<Done>,
    cancel: Arc<AtomicBool>,
    /// Speeds of the coarse grid not started yet.
    grid: VecDeque<f64>,
    running: Vec<f64>,
    failed: Vec<f64>,
    /// Runs started so far.
    started: u32,
    parallel: usize,
    max_runs: u32,
    min_speed: f64,
    max_speed: f64,
    /// Stop was pressed: running speeds are cancelled, no new ones start.
    stopping: bool,
    /// Pause was pressed: running speeds finish, no new ones start until Continue.
    paused: bool,
}

impl Drop for Sweep {
    /// A sweep dropped with its block or window leaves no solver running.
    fn drop(&mut self) {
        self.cancel.store(true, Ordering::Relaxed);
    }
}

/// What the sweep threads work on.
struct SweepInput {
    /// Threads of the pool the speeds share, see [`plx_solve::pool`].
    threads: u32,
    work_dir: std::path::PathBuf,
    mesh: Arc<plx_mesh::FeMesh>,
    /// The model at its own speed, with the default solver resolved.
    fe: FeModel,
    /// Index of the complex frequency step in the model.
    step: usize,
}

/// Runs the solver at one speed in a thread of its own and sends the modes back. The files of a
/// successful run are removed, those of a failed one stay for a look.
fn spawn_speed(
    input: Arc<SweepInput>,
    speed: f64,
    index: u32,
    sender: Sender<Done>,
    cancel: Arc<AtomicBool>,
) -> std::io::Result<()> {
    std::thread::Builder::new()
        .name(format!("campbell {index}"))
        .spawn(move || {
            let result = run_speed(&input, speed, index, &cancel);
            if result.is_ok() {
                remove_job_files(&input.work_dir, &job_name(index));
            }
            let _ = sender.send(Done { speed, result });
        })
        .map(|_| ())
}

fn job_name(index: u32) -> String {
    format!("Campbell-{index}")
}

/// Removes the files `name.*` the solver wrote in `dir`.
fn remove_job_files(dir: &std::path::Path, name: &str) {
    let prefix = format!("{name}.");
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        if entry.file_name().to_string_lossy().starts_with(&prefix) {
            let _ = std::fs::remove_file(entry.path());
        }
    }
}

/// One run of the solver at `speed`, as job `Campbell-<index>` in the work directory.
fn run_speed(
    input: &SweepInput,
    speed: f64,
    index: u32,
    cancel: &AtomicBool,
) -> Result<SpeedPoint, String> {
    let label = format!("{} rpm", numeric::format_physical(rpm(speed)));
    let fe = with_speed(&input.fe, speed);
    let heading = format!("Nodalis Campbell sweep: {label}");
    let lowered = plx_io::inp::lower_unvalidated(&input.mesh, &fe, &heading)
        .map_err(|e| format!("{label}: analysis not set up: {e}"))?;
    let name = job_name(index);
    let step = solver_step(&lowered, input.step)
        .ok_or_else(|| format!("{label}: the Complex Frequency step is not solved"))?;
    let prepared = crate::analysis::prepare(lowered, &fe, &[]);
    let mut job = Run::start_analysis(input.threads, &input.work_dir, &name, prepared)
        .map_err(|e| format!("{label}: solver not started: {e}"))?;
    let status = loop {
        let status = job.poll();
        if status != JobStatus::Running {
            break status;
        }
        if cancel.load(Ordering::Relaxed) {
            job.kill();
            return Err(format!("{label}: cancelled"));
        }
        std::thread::sleep(Duration::from_millis(50));
    };
    // The results are taken in memory: the history output, else the result fields.
    let modes = job.solver_results().map_or_else(Vec::new, |solved| {
        let history = plx_io::solver::history(&solved, None);
        let modes = history.map_or_else(Vec::new, |h| modes_of_history(&h, step));
        if modes.is_empty() {
            modes_of_frd(&plx_io::solver::import(&solved), step)
        } else {
            modes
        }
    });
    if modes.is_empty() {
        let output = job.new_output();
        let error = output
            .iter()
            .find(|l| l.contains("*ERROR"))
            .cloned()
            .unwrap_or_else(|| {
                format!(
                    "Solver {status:?}, no complex frequencies of step {} in the results \
                     of {name}",
                    step
                )
            });
        return Err(format!("{label}: {error}"));
    }
    Ok(SpeedPoint { speed, modes })
}

/// The Campbell diagram window: the setup of the sweep, its progress and the diagram.
pub struct CampbellWindow {
    pub setup: Setup,
    /// False while the window is closed; its last sweep stays for when it is opened again.
    pub visible: bool,
    sweep: Option<Sweep>,
    points: Vec<SpeedPoint>,
    log: Vec<String>,
    /// Orders the shown diagram was made with.
    orders: Vec<f64>,
    /// Highest speed of the shown sweep, where the order lines end.
    swept_speed: f64,
    /// A sweep was started, so an empty diagram is a failure rather than nothing yet.
    swept: bool,
    /// Counts sweeps, so each starts with a fresh view of the plot.
    plot_generation: u32,
    /// The frequency axis limit the plot was last shown with.
    shown_max_frequency: f64,
    /// Speed range in radians per time the user marked in the diagram for refinement.
    focus: Option<[f64; 2]>,
    /// The range being marked by dragging, in rpm: where the drag started and is now.
    marking: Option<[f64; 2]>,
    /// Fingerprint of the model the points were computed with, so that a sweep continues
    /// only on the same model.
    fingerprint: Option<u64>,
    /// Number of the next solver job, counted over continued sweeps.
    next_index: u32,
    /// Speed in radians per time selected in the diagram or the table of critical speeds.
    selected: Option<f64>,
    /// Speed the user asked to set the analysis to, see [`Self::take_analysis_speed`].
    apply: Option<f64>,
    /// The critical speeds as last found, with the plot generation, the number of speeds and
    /// the orders they were found for.
    critical: std::cell::RefCell<Option<CriticalCache>>,
}

/// See [`CampbellWindow::critical`].
type CriticalCache = ((u32, usize, Vec<f64>, Frame), Vec<CriticalSpeed>);

impl CampbellWindow {
    /// A window for the model: the sweep runs up to the speed of its centrifugal load, or
    /// up to 10000 rpm when the model stands still.
    pub fn new(fe: &FeModel) -> Self {
        let speed = rotor_speed(fe)
            .filter(|s| *s > 0.0)
            .unwrap_or(from_rpm(10_000.0));
        Self::saved(
            Setup {
                max_speed: speed,
                ..Setup::default()
            },
            Vec::new(),
            None,
            None,
        )
    }

    /// A window with what a Campbell block keeps: its settings, the speeds of its last sweep,
    /// the range marked for refinement and the fingerprint of the model it swept.
    pub fn saved(
        setup: Setup,
        points: Vec<SpeedPoint>,
        focus: Option<[f64; 2]>,
        fingerprint: Option<u64>,
    ) -> Self {
        let speed = setup.max_speed;
        Self {
            orders: order_values(&setup.orders),
            swept: !points.is_empty(),
            setup,
            visible: false,
            sweep: None,
            points,
            log: Vec::new(),
            swept_speed: speed,
            plot_generation: 0,
            shown_max_frequency: 0.0,
            focus,
            marking: None,
            fingerprint,
            next_index: 0,
            selected: None,
            apply: None,
            critical: Default::default(),
        }
    }

    /// The speed in radians per time the user asked to set the analysis to with Set Analysis
    /// Speed, once.
    pub fn take_analysis_speed(&mut self) -> Option<f64> {
        self.apply.take()
    }

    /// Notes in the output of the window.
    pub fn note(&mut self, line: String) {
        self.log.push(line);
    }

    pub fn setup(&self) -> &Setup {
        &self.setup
    }

    /// The speeds of the last sweep, in the order they were computed.
    pub fn points(&self) -> &[SpeedPoint] {
        &self.points
    }

    pub fn focus(&self) -> Option<[f64; 2]> {
        self.focus
    }

    /// Fingerprint of the model the speeds were computed with, see [`sweep_fingerprint`].
    pub fn fingerprint(&self) -> Option<u64> {
        self.fingerprint
    }

    /// The critical speeds of the diagram as shown, by order and speed.
    pub fn critical_speeds(&self) -> Vec<CriticalSpeed> {
        let key = (
            self.plot_generation,
            self.points.len(),
            self.shown_orders(),
            self.setup.frame,
        );
        let mut cache = self.critical.borrow_mut();
        if let Some((cached, speeds)) = &*cache
            && *cached == key
        {
            return speeds.clone();
        }
        let speeds = critical_speeds(&track(&self.points), &key.2, key.3);
        *cache = Some((key, speeds.clone()));
        speeds
    }

    /// How far the sweep is, in a few words.
    pub fn progress(&self) -> String {
        let count = self.points.len();
        let speeds = if count == 1 { "speed" } else { "speeds" };
        match &self.sweep {
            Some(sweep) if sweep.stopping => format!("Stopping at {count} {speeds}"),
            Some(sweep) if sweep.paused => format!("Paused at {count} {speeds}"),
            Some(sweep) => format!(
                "Sweeping: {count} {speeds}, {} running",
                sweep.running.len()
            ),
            None if count == 0 => "No sweep yet".into(),
            None => {
                let (low, high) = (self.points.iter().map(|p| p.speed))
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), s| {
                        (a.min(s), b.max(s))
                    });
                format!(
                    "{count} {speeds}, {} to {} rpm",
                    numeric::format_physical(rpm(low).round()),
                    numeric::format_physical(rpm(high).round())
                )
            }
        }
    }

    /// Whether the sweep is paused: no new speeds start until it continues.
    pub fn is_paused(&self) -> bool {
        self.sweep.as_ref().is_some_and(|s| s.paused)
    }

    /// Pauses the sweep: the running speeds finish, no new ones start.
    pub fn pause(&mut self) {
        if let Some(sweep) = &mut self.sweep {
            sweep.paused = true;
            self.log
                .push("Sweep paused; the running speeds finish.".into());
        }
    }

    /// Whether a finished or stopped sweep can go on with more runs.
    fn can_continue(&self) -> bool {
        self.is_paused() || (self.sweep.is_none() && self.points.len() >= 2)
    }

    pub fn is_running(&self) -> bool {
        self.sweep.is_some()
    }

    /// Starts the sweep on the model of `mesh` and `fe` in `work_dir`. With `keep` it continues the last sweep instead: a paused
    /// one goes on, a finished one starts another `max_runs` runs that refine between the
    /// speeds it has, as long as the model is the same.
    pub fn start(
        &mut self,
        solver: &crate::settings::Solver,
        (mesh, fe): (&plx_mesh::FeMesh, &FeModel),
        work_dir: std::path::PathBuf,
        keep: bool,
    ) -> Result<(), String> {
        if keep && let Some(sweep) = &mut self.sweep {
            sweep.paused = false;
            self.log.push("Sweep continued.".into());
            self.poll();
            return Ok(());
        }
        let fingerprint = sweep_fingerprint(mesh, fe);
        if keep && self.fingerprint != Some(fingerprint) {
            return Err(
                "The model changed since the sweep; start a new sweep for the changed model."
                    .into(),
            );
        }
        if mesh.element_count() == 0 {
            return Err("The analysis has no mesh.".into());
        }
        let Some(step) = complex_step(fe) else {
            return Err("The analysis has no active Complex Frequency step.".into());
        };
        let Setup {
            min_speed,
            max_speed,
            max_runs,
            ..
        } = self.setup;
        if max_speed <= 0.0 {
            return Err("The highest speed must be greater than zero.".into());
        }
        if min_speed >= max_speed {
            return Err("The lowest speed must be below the highest.".into());
        }
        // The speeds solved at the same time share the solver's one pool.
        let fe = fe.clone();
        let grid_count = if max_runs == 0 {
            GRID
        } else {
            GRID.min(max_runs as usize).max(2)
        };
        let (sender, receiver) = channel();
        let runs = match max_runs {
            0 => "until stopped".to_string(),
            n => format!("up to {n} runs"),
        };
        // A continued sweep keeps its speeds: a grid covers the part of the speed range beyond
        // them, if any, and refinement goes between them.
        let grid = if keep {
            let more = extension(&self.points, min_speed, max_speed, grid_count);
            self.swept_speed = self.swept_speed.max(max_speed);
            self.log.push(if more.is_empty() {
                format!("Sweep continued: {runs}.")
            } else {
                format!(
                    "Sweep continued over {} to {} rpm: {runs}, the speeds done stay.",
                    shown(rpm(min_speed)),
                    shown(rpm(max_speed))
                )
            });
            more.into()
        } else {
            self.orders = order_values(&self.setup.orders);
            self.points.clear();
            self.swept = true;
            self.swept_speed = max_speed;
            self.plot_generation += 1;
            self.focus = None;
            self.selected = None;
            self.fingerprint = Some(fingerprint);
            self.log = vec![format!(
                "Sweep started: {} to {} rpm, {runs}, in {}",
                numeric::format_physical(rpm(min_speed)),
                numeric::format_physical(rpm(max_speed)),
                work_dir.display()
            )];
            grid(min_speed, max_speed, grid_count).into()
        };
        self.sweep = Some(Sweep {
            input: Arc::new(SweepInput {
                threads: solver.threads,
                work_dir,
                mesh: Arc::new(mesh.clone()),
                fe,
                step,
            }),
            sender,
            receiver,
            cancel: Arc::new(AtomicBool::new(false)),
            grid,
            running: Vec::new(),
            failed: Vec::new(),
            started: 0,
            parallel: solver.parallel_runs.max(1) as usize,
            max_runs,
            min_speed,
            max_speed,
            stopping: false,
            paused: false,
        });
        self.poll();
        Ok(())
    }

    /// Stops the sweep: the running speeds are cancelled, the finished ones stay.
    pub fn cancel(&mut self) {
        if let Some(sweep) = &mut self.sweep {
            sweep.stopping = true;
            sweep.cancel.store(true, Ordering::Relaxed);
        }
    }

    /// Takes the finished speeds and starts new ones while there is room.
    pub fn poll(&mut self) {
        let Some(sweep) = &mut self.sweep else {
            return;
        };
        while let Ok(Done { speed, result }) = sweep.receiver.try_recv() {
            sweep.running.retain(|&s| s != speed);
            match result {
                Ok(point) => {
                    self.log.push(format!(
                        "{} rpm: {} modes, first {} Hz",
                        shown(rpm(point.speed)),
                        point.modes.len(),
                        point
                            .modes
                            .first()
                            .map_or_else(|| "-".to_string(), |m| shown(m.frequency))
                    ));
                    self.points.push(point);
                }
                Err(error) => {
                    if !sweep.stopping {
                        self.log.push(error);
                    }
                    sweep.failed.push(speed);
                }
            }
        }
        // A marked range takes the refinement, within the speed range of the sweep.
        let (low, high) = match self.focus {
            Some([a, b]) => (a.max(sweep.min_speed), b.min(sweep.max_speed)),
            None => (sweep.min_speed, sweep.max_speed),
        };
        let mut exhausted = false;
        while !sweep.stopping
            && !sweep.paused
            && sweep.running.len() < sweep.parallel
            && (sweep.max_runs == 0 || sweep.started < sweep.max_runs)
        {
            let next = sweep.grid.pop_front().or_else(|| {
                // Refinement needs the coarse grid's curves.
                (self.points.len() + sweep.failed.len() >= 3)
                    .then(|| next_speed(&self.points, &sweep.failed, &sweep.running, low, high))
                    .flatten()
            });
            let Some(speed) = next else {
                exhausted = true;
                break;
            };
            let index = self.next_index;
            self.next_index += 1;
            sweep.started += 1;
            let spawned = spawn_speed(
                Arc::clone(&sweep.input),
                speed,
                index,
                sweep.sender.clone(),
                Arc::clone(&sweep.cancel),
            );
            match spawned {
                Ok(()) => sweep.running.push(speed),
                Err(e) => {
                    self.log.push(format!("Speed not started: {e}"));
                    sweep.stopping = true;
                }
            }
        }
        let limit = sweep.max_runs != 0 && sweep.started >= sweep.max_runs;
        if sweep.running.is_empty() && (sweep.stopping || (!sweep.paused && (limit || exhausted))) {
            self.log.push(if sweep.stopping {
                format!("Sweep stopped after {} speeds.", self.points.len())
            } else if limit {
                format!("Sweep finished: all {} runs done.", sweep.max_runs)
            } else if self.focus.is_some() {
                "Sweep finished: the marked range is as fine as the sweep resolves.".into()
            } else {
                "Sweep finished: the speeds are as close as the sweep resolves.".into()
            });
            self.sweep = None;
        }
    }

    /// Shows the window; returns false when it was closed. `can_start` tells whether the
    /// model is ready; `start` is called with the setup when the user starts the sweep.
    ///
    /// The diagram fills the left, the setup, the selected speed, the critical speeds and the
    /// output of the sweep stand in a panel on the right whose width the separator sets.
    pub fn show(
        &mut self,
        ctx: &egui::Context,
        (title, id): (&str, u64),
        can_start: bool,
        mut start: impl FnMut(&mut Self, bool) -> Result<(), String>,
    ) -> bool {
        self.poll();
        let mut open = true;
        let mut close = false;
        let running = self.is_running();
        egui::Window::new(format!("Campbell diagram: {title}"))
            .id(crate::ids::window(ctx, ("campbell", id)))
            .open(&mut open)
            .collapsible(false)
            .resizable(true)
            .default_size([1060.0, 680.0])
            .min_size([640.0, 400.0])
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                self.toolbar(ui, can_start, running, &mut start);
                ui.separator();
                close = crate::dialog::close_bar(ui, "campbell");
                let tracks = track(&self.points);
                let orders = self.shown_orders();
                let critical = critical_speeds(&tracks, &orders, self.setup.frame);
                egui::Panel::right(egui::Id::new(("campbell side", id)))
                    .resizable(true)
                    .default_size(340.0)
                    .size_range(250.0..=700.0)
                    .frame(egui::Frame::new().inner_margin(egui::Margin {
                        left: 10,
                        ..Default::default()
                    }))
                    .show(ui, |ui| self.side(ui, running, &tracks, &critical));
                if self.points.len() >= 2 {
                    let end =
                        (self.points.iter().map(|p| p.speed)).fold(self.swept_speed, f64::max);
                    let data = diagram(&tracks, &orders, end, self.setup.frame);
                    self.plot(ui, &data);
                } else if self.swept && !running {
                    ui.colored_label(
                        ui.visuals().warn_fg_color,
                        format!(
                            "No diagram: {} of the speeds gave complex frequencies, at least \
                             2 are needed. The output says why the others failed.",
                            self.points.len()
                        ),
                    );
                } else {
                    ui.add_space(8.0);
                    ui.weak(
                        "The sweep runs the analysis once per speed, with the centrifugal \
                         load set to it, and plots the complex frequencies of the last \
                         Complex Frequency step over the speed. It starts with a coarse grid \
                         over the speed range and then adds speeds where the curves bend \
                         most, until it is stopped or has done its runs. The crossings with \
                         the engine order lines are the critical speeds. The solver computes \
                         the Coriolis modes in the rotating frame of reference; the \
                         frequencies and the whirl directions (F forward, B backward) are \
                         the solver's. Frame of reference switches the diagram to what an \
                         observer at rest sees: every mode of n nodal diameters as its \
                         forward wave f + n x speed and its backward wave f - n x speed.",
                    );
                }
            });
        if running {
            ctx.request_repaint_after(Duration::from_millis(200));
        }
        open && !close
    }

    /// The toolbar of the window: the sweep on the left, its progress, the export on the right.
    fn toolbar(
        &mut self,
        ui: &mut Ui,
        can_start: bool,
        running: bool,
        start: &mut impl FnMut(&mut Self, bool) -> Result<(), String>,
    ) {
        ui.horizontal(|ui| {
            if ui
                .add_enabled(can_start && !running, egui::Button::new("Start Sweep"))
                .on_hover_text("Starts a new sweep; the speeds of the last one go.")
                .clicked()
                && let Err(error) = start(self, false)
            {
                self.log.push(error);
            }
            if ui
                .add_enabled(running && !self.is_paused(), egui::Button::new("Pause"))
                .on_hover_text(
                    "The running speeds finish, no new ones start. Mark a range in the diagram \
                     or change the speed range meanwhile and continue.",
                )
                .clicked()
            {
                self.pause();
            }
            if ui
                .add_enabled(
                    can_start && self.can_continue(),
                    egui::Button::new("Continue"),
                )
                .on_hover_text(
                    "Goes on with the sweep, keeping its speeds: a paused one where it \
                     stopped, a finished one with another Maximum runs. New speeds first fill \
                     a part of the speed range not swept yet, then go between the speeds it \
                     has, within the marked range if there is one.",
                )
                .clicked()
                && let Err(error) = start(self, true)
            {
                self.log.push(error);
            }
            if ui
                .add_enabled(running, egui::Button::new("Stop"))
                .on_hover_text("Stops the sweep; the speeds done so far stay.")
                .clicked()
            {
                self.cancel();
            }
            if let Some(sweep) = &self.sweep {
                ui.separator();
                if sweep.paused && sweep.running.is_empty() {
                    ui.label("Paused:");
                } else {
                    ui.spinner();
                }
                let total = match sweep.max_runs {
                    0 => String::new(),
                    n => format!(" of {n}"),
                };
                ui.label(format!(
                    "{} runs done{total}, {} running",
                    sweep.started as usize - sweep.running.len(),
                    sweep.running.len()
                ));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let any = !self.points.is_empty();
                for (label, format) in [
                    ("Export JSON …", table_export::Format::Json),
                    ("Export CSV …", table_export::Format::Csv),
                ] {
                    if ui.add_enabled(any, egui::Button::new(label)).clicked() {
                        self.export(format);
                    }
                }
                if ui
                    .add_enabled(any, egui::Button::new("Copy Table"))
                    .on_hover_text("Copies the sweep with tabs, for a spreadsheet.")
                    .clicked()
                {
                    ui.ctx()
                        .copy_text(table_text(&self.points, &self.shown_orders(), '\t'));
                }
            });
        });
    }

    /// The panel on the right: setup, selected speed, critical speeds and the output, which
    /// takes the height left.
    fn side(&mut self, ui: &mut Ui, running: bool, tracks: &Tracks, critical: &[CriticalSpeed]) {
        // The output takes the foot of the panel, with a separator of its own to drag.
        egui::Panel::bottom(ui.id().with("campbell output"))
            .resizable(true)
            .default_size(120.0)
            .size_range(60.0..=600.0)
            .frame(egui::Frame::new())
            .show(ui, |ui| {
                heading(ui, "Output");
                egui::ScrollArea::vertical()
                    .id_salt("campbell log")
                    .auto_shrink(false)
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        for line in &self.log {
                            ui.add(
                                egui::Label::new(egui::RichText::new(line).monospace().small())
                                    .wrap(),
                            );
                        }
                    });
            });
        egui::ScrollArea::vertical()
            .id_salt("campbell details")
            .auto_shrink(false)
            .show(ui, |ui| {
                heading(ui, "Sweep");
                self.setup_grid(ui, running);
                if !running && self.extends_beyond_points() {
                    ui.weak("Continue sweeps the new part of the range and keeps the speeds done.");
                }
                heading(ui, "Critical speeds");
                self.critical_table(ui, critical);
                heading(ui, "Selected speed");
                self.selection(ui, tracks);
            });
    }

    /// Whether the speed range of the setup reaches beyond the speeds of the sweep, so that
    /// Continue sweeps the new part.
    fn extends_beyond_points(&self) -> bool {
        self.points.len() >= 2
            && !extension(&self.points, self.setup.min_speed, self.setup.max_speed, 2).is_empty()
    }

    fn setup_grid(&mut self, ui: &mut Ui, running: bool) {
        egui::Grid::new("campbell setup")
            .num_columns(2)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                ui.label("Speed range [rpm]").on_hover_text(format!(
                    "{} to {} rad/s. It can change after a sweep: Continue then sweeps the \
                     new part and keeps the speeds done.",
                    numeric::format_physical(self.setup.min_speed),
                    numeric::format_physical(self.setup.max_speed)
                ));
                ui.horizontal(|ui| {
                    let mut low = rpm(self.setup.min_speed);
                    let mut high = rpm(self.setup.max_speed);
                    if ui
                        .add_enabled(
                            !running,
                            numeric::drag_value(&mut low)
                                .range(0.0..=f64::MAX)
                                .speed(10.0),
                        )
                        .changed()
                    {
                        self.setup.min_speed = from_rpm(low);
                    }
                    ui.label("to");
                    if ui
                        .add_enabled(
                            !running,
                            numeric::drag_value(&mut high)
                                .range(0.0..=f64::MAX)
                                .speed(10.0),
                        )
                        .changed()
                    {
                        self.setup.max_speed = from_rpm(high);
                    }
                });
                ui.end_row();
                ui.label("Maximum runs").on_hover_text(
                    "The solver runs once per speed; 0 runs until Stop. Continue does as many \
                     runs again.",
                );
                ui.add_enabled(
                    !running,
                    numeric::drag_value(&mut self.setup.max_runs).range(0..=10_000),
                );
                ui.end_row();
                ui.label("Engine orders")
                    .on_hover_text("Lines f = order x speed, e.g. 1, 2, 3");
                ui.add_enabled(
                    !running,
                    egui::TextEdit::singleline(&mut self.setup.orders).desired_width(120.0),
                );
                ui.end_row();
                // The axis can change any time, also while the sweep runs.
                ui.label("Frequency axis [Hz]")
                    .on_hover_text("Upper end of the frequency axis; 0 fits it to the modes.");
                ui.add(
                    numeric::drag_value(&mut self.setup.max_frequency)
                        .range(0.0..=f64::MAX)
                        .speed(10.0),
                );
                ui.end_row();
                // The frame is a view of the same sweep, so it can change any time too.
                ui.label("Frame of reference").on_hover_text(
                    "Rotating: the frequencies as the solver computes them, in the frame \
                     turning with the rotor. Stationary: as a sensor or microphone at rest \
                     sees them. A mode of n nodal diameters (ND n in the legend, from its \
                     shape) is then a forward travelling wave at f + n x speed and a backward \
                     one at |f - n x speed|; where the backward wave comes to rest, the rotor \
                     carries a standing wave that any load at rest excites.",
                );
                egui::ComboBox::from_id_salt("campbell frame")
                    .selected_text(self.setup.frame.label())
                    .width(190.0)
                    .show_ui(ui, |ui| {
                        for frame in Frame::ALL {
                            ui.selectable_value(&mut self.setup.frame, frame, frame.label());
                        }
                    });
                ui.end_row();
            });
    }

    /// The speed selected in the diagram or the table, its modes, and the button that sets the
    /// analysis to it.
    fn selection(&mut self, ui: &mut Ui, tracks: &Tracks) {
        let Some(speed) = self.selected else {
            ui.weak("Click a point in the diagram or a critical speed to select its speed.");
            return;
        };
        ui.horizontal(|ui| {
            ui.strong(format!("{} rpm", shown(rpm(speed))));
            ui.weak(format!("= {} rad/s", shown(speed)));
        });
        let modes: Vec<(&Track, &Mode)> = (tracks.modes.iter())
            .filter_map(|t| {
                let (_, mode) = t.points.iter().find(|(s, _)| *s == speed)?;
                Some((t, mode))
            })
            .collect();
        if modes.is_empty() {
            ui.weak("Between the computed speeds; the diagram has no modes here.");
        } else {
            let stationary = self.setup.frame == Frame::Stationary;
            egui::Grid::new("campbell selected modes")
                .num_columns(if stationary { 3 } else { 2 })
                .spacing([16.0, 1.0])
                .show(ui, |ui| {
                    if stationary {
                        ui.strong("Mode");
                        ui.strong("Forward");
                        ui.strong("Backward");
                        ui.end_row();
                    }
                    for (track, mode) in modes {
                        ui.label(&track.name);
                        match (stationary, track.waves(speed, mode)) {
                            (true, Some((forward, backward))) => {
                                ui.label(format!("{} Hz", shown(forward)));
                                ui.label(format!("{} Hz", shown(backward.abs())));
                            }
                            (true, None) => {
                                ui.label(format!("{} Hz", shown(mode.frequency)));
                                ui.weak("rotating, no ND");
                            }
                            (false, _) => {
                                ui.label(format!("{} Hz", shown(mode.frequency)));
                            }
                        }
                        ui.end_row();
                    }
                });
        }
        ui.horizontal(|ui| {
            if ui
                .button("Set Analysis Speed")
                .on_hover_text(
                    "Sets the speed of the Complex Frequency step of the analysis to this \
                     speed. Solve the \
                     analysis then to see all results at this speed; the diagram stays as it \
                     is.",
                )
                .clicked()
            {
                self.apply = Some(speed);
            }
            if ui.button("Clear").clicked() {
                self.selected = None;
            }
        });
    }

    /// The crossings of the modes with the engine order lines; a click on a row selects its
    /// speed.
    fn critical_table(&mut self, ui: &mut Ui, critical: &[CriticalSpeed]) {
        if critical.is_empty() {
            ui.weak(if self.points.len() < 2 {
                "The crossings of the modes with the engine order lines, after a sweep."
            } else if self.setup.frame == Frame::Stationary {
                "No crossing of a travelling wave with an engine order line, and no wave \
                 coming to rest, in the sweep. Modes without a nodal diameter are left out."
            } else {
                "No crossing of a mode with an engine order line in the sweep."
            });
            return;
        }
        egui::ScrollArea::vertical()
            .id_salt("critical speeds")
            .max_height(160.0)
            .show(ui, |ui| {
                egui::Grid::new("critical speeds grid")
                    .num_columns(5)
                    .striped(true)
                    .spacing([14.0, 2.0])
                    .show(ui, |ui| {
                        for title in ["Order", "Mode", "Branch", "Speed [rpm]", "Freq. [Hz]"] {
                            ui.strong(title);
                        }
                        ui.end_row();
                        for c in critical {
                            ui.label(c.order_label()).on_hover_text(if c.order == 0.0 {
                                "The backward travelling wave of the mode comes to rest: the \
                                 rotor carries a standing wave that any load at rest excites."
                            } else {
                                "The line crosses this engine order."
                            });
                            ui.label(c.mode.to_string());
                            ui.label(c.branch_label());
                            let selected = self.selected == Some(c.speed);
                            if ui
                                .selectable_label(selected, shown(rpm(c.speed)))
                                .on_hover_text("Selects this speed.")
                                .clicked()
                            {
                                self.selected = Some(c.speed);
                            }
                            ui.label(shown(c.frequency()));
                            ui.end_row();
                        }
                    });
            });
    }

    /// The orders of the shown diagram: those of the running sweep, else the text field's,
    /// which can change after the sweep.
    fn shown_orders(&self) -> Vec<f64> {
        if self.is_running() {
            self.orders.clone()
        } else {
            order_values(&self.setup.orders)
        }
    }

    /// Saves the sweep as a CSV or JSON file, to a path from a Save As dialog.
    fn export(&mut self, format: table_export::Format) {
        let Some(path) = table_export::save_dialog("Export Campbell Diagram", "campbell", format)
        else {
            return;
        };
        let text = table_text(&self.points, &self.shown_orders(), ',');
        self.log
            .push(match table_export::write(&path, &text, ',', format) {
                Ok(()) => format!("Exported: {}", path.display()),
                Err(err) => format!("Export failed ({}): {err}", path.display()),
            });
    }

    /// The diagram as an interactive plot over the height left: drag to pan, scroll or drag
    /// along an axis to zoom, double-click to fit, click a legend entry to hide or show it,
    /// click a point to select its speed, Shift and drag to mark a range to refine.
    fn plot(&mut self, ui: &mut Ui, data: &Diagram) {
        use egui_plot::{Legend, Line, LineStyle, MarkerShape, Plot, Points, Span, VLine};
        let max_frequency = self.setup.max_frequency;
        let line = ui.text_style_height(&egui::TextStyle::Body) + ui.spacing().item_spacing.y;
        let height = (ui.available_height() - line - 6.0).max(160.0);
        let mut plot = Plot::new(("campbell plot", self.plot_generation))
            .height(height)
            .legend(Legend::default().follow_insertion_order(true))
            .x_axis_label("Speed [rpm]")
            .y_axis_label(match self.setup.frame {
                Frame::Rotating => "Frequency [Hz], rotating frame",
                Frame::Stationary => "Frequency [Hz], stationary frame",
            })
            .allow_axis_zoom_drag(true)
            .allow_double_click_reset(true)
            .grid_color(crate::style::plot_grid())
            .include_x(0.0)
            .include_y(0.0)
            .label_formatter(|hover| match hover {
                egui_plot::HoverPosition::NearDataPoint {
                    plot_name,
                    position,
                    ..
                } => Some(format!(
                    "{plot_name}\n{} rpm\n{} Hz",
                    shown(position.x),
                    shown(position.y)
                )),
                _ => None,
            });
        if max_frequency > 0.0 {
            plot = plot.default_y_bounds(0.0, max_frequency);
        }
        if max_frequency != self.shown_max_frequency {
            // A new axis limit applies at once, as a reset of the view.
            plot = plot.reset();
            self.shown_max_frequency = max_frequency;
        }
        // Shift and drag marks a speed range instead of moving the view.
        let shift = ui.input(|i| i.modifiers.shift);
        if shift || self.marking.is_some() {
            plot = plot.allow_drag(false);
        }
        let guide = crate::style::adapt(Color32::from_gray(110));
        // Every speed is a marker, so the adaptive sampling shows; smaller when many.
        let many = data
            .curves
            .iter()
            .map(|c| c.points.len())
            .max()
            .unwrap_or(0)
            > 80;
        let radius = if many { 1.8 } else { 2.5 };
        let marked = match (self.marking, self.focus) {
            (Some([a, b]), _) => Some([a.min(b), a.max(b)]),
            (None, Some([a, b])) => Some([rpm(a), rpm(b)]),
            (None, None) => None,
        };
        let selected = self.selected.map(rpm);
        let response = plot.show(ui, |plot_ui| {
            if let Some([a, b]) = marked {
                let color = crate::style::adapt(Color32::from_rgb(40, 110, 200));
                plot_ui.span(
                    Span::new("Refine range", a..=b)
                        .fill(color.gamma_multiply(0.12))
                        .border_color(color),
                );
            }
            if let Some(x) = selected {
                plot_ui.vline(
                    VLine::new("", x)
                        .color(crate::style::plot_text().gamma_multiply(0.7))
                        .width(1.0)
                        .allow_hover(false),
                );
            }
            for (name, points) in &data.guides {
                plot_ui.line(
                    Line::new(name.as_str(), points.clone())
                        .color(guide)
                        .style(LineStyle::dashed_loose())
                        .allow_hover(false),
                );
            }
            for curve in &data.curves {
                let Curve {
                    name,
                    points,
                    mode,
                    dashed,
                } = curve;
                let color = xy_plot::color(*mode);
                let mut line = Line::new(name.as_str(), points.clone())
                    .color(color)
                    .width(2.0);
                if *dashed {
                    line = line.style(LineStyle::dashed_dense());
                }
                plot_ui.line(line);
                plot_ui.points(
                    Points::new(name.as_str(), points.clone())
                        .color(color)
                        .radius(radius),
                );
                // The modes at the selected speed stand out.
                if let Some(x) = selected {
                    let at: Vec<[f64; 2]> = points.iter().filter(|p| p[0] == x).copied().collect();
                    if !at.is_empty() {
                        plot_ui.points(
                            Points::new(name.as_str(), at)
                                .color(color)
                                .shape(MarkerShape::Circle)
                                .filled(false)
                                .radius(6.0)
                                .allow_hover(false),
                        );
                    }
                }
            }
            if !data.unmatched.is_empty() {
                plot_ui.points(
                    Points::new("Unmatched roots", data.unmatched.clone())
                        .color(crate::style::adapt(Color32::from_gray(120)))
                        .shape(MarkerShape::Cross)
                        .radius(4.0),
                );
            }
            if !data.unstable.is_empty() {
                plot_ui.points(
                    Points::new("Unstable", data.unstable.clone())
                        .color(Color32::from_rgb(200, 30, 30))
                        .shape(MarkerShape::Diamond)
                        .radius(4.0),
                );
            }
            let pixels_per_rpm = plot_ui.transform().dpos_dvalue_x().abs();
            (plot_ui.pointer_coordinate().map(|p| p.x), pixels_per_rpm)
        });
        let ((pointer, pixels_per_rpm), response) = (response.inner, response.response);
        if let Some(x) = pointer {
            if shift && response.drag_started_by(egui::PointerButton::Primary) {
                self.marking = Some([x, x]);
            } else if let Some(marking) = &mut self.marking {
                marking[1] = x;
            }
        }
        if response.drag_stopped()
            && let Some([a, b]) = self.marking.take()
        {
            let [low, high] = round_range(a.min(b).max(0.0), a.max(b));
            if high > low {
                self.focus = Some([from_rpm(low), from_rpm(high)]);
            }
        }
        // A click selects the computed speed nearest to it, within a few pixels.
        if response.clicked()
            && !shift
            && let Some(x) = pointer
            && let Some(nearest) = (self.points.iter().map(|p| p.speed))
                .min_by(|a, b| (rpm(*a) - x).abs().total_cmp(&(rpm(*b) - x).abs()))
            && (rpm(nearest) - x).abs() * pixels_per_rpm <= 8.0
        {
            self.selected = Some(nearest);
        }
        if !data.left_out.is_empty() {
            ui.weak(format!(
                "Not in the stationary frame, no nodal diameter: {}.",
                data.left_out.join(", ")
            ));
        }
        ui.horizontal(|ui| match self.focus {
            Some([a, b]) => {
                ui.label(format!(
                    "Refine range: {} to {} rpm",
                    numeric::format_physical(rpm(a)),
                    numeric::format_physical(rpm(b))
                ));
                if ui
                    .button("Clear Range")
                    .on_hover_text("New speeds go anywhere in the speed range again.")
                    .clicked()
                {
                    self.focus = None;
                }
            }
            None => {
                ui.weak(
                    "Click a point to select its speed; Shift and drag marks a range to refine.",
                );
            }
        });
    }
}

/// A heading of the side panel of the Campbell window.
fn heading(ui: &mut Ui, title: &str) {
    ui.add_space(8.0);
    ui.label(egui::RichText::new(title).strong());
    ui.separator();
}

/// A number for reading, to six significant digits.
pub fn shown(value: f64) -> String {
    if value == 0.0 || !value.is_finite() {
        return numeric::format_physical(value);
    }
    let scale = 10f64.powi(5 - value.abs().log10().floor() as i32);
    numeric::format_physical((value * scale).round() / scale)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(speed: f64, frequencies: &[(f64, Whirl)]) -> SpeedPoint {
        SpeedPoint {
            speed,
            modes: frequencies
                .iter()
                .map(|&(frequency, whirl)| Mode {
                    frequency,
                    whirl,
                    unstable: false,
                    nodal_diameter: None,
                })
                .collect(),
        }
    }

    #[test]
    fn critical_speeds_are_the_crossings_with_the_order_lines() {
        // A backward mode falling from 100 Hz and a forward mode rising; the 1x line
        // f = speed / 2 pi crosses the backward mode where 100 - 10 t = 60 t / 2pi.
        let points: Vec<SpeedPoint> = (0..=10)
            .map(|i| {
                let t = f64::from(i);
                point(
                    60.0 * t,
                    &[
                        (100.0 - 10.0 * t, Whirl::Backward),
                        (100.0 + 10.0 * t, Whirl::Forward),
                    ],
                )
            })
            .collect();
        let critical = critical_speeds(&track(&points), &[1.0], Frame::Rotating);
        assert_eq!(critical.len(), 1, "{critical:?}");
        let c = &critical[0];
        assert_eq!((c.order, c.mode, c.whirl), (1.0, 1, Whirl::Backward));
        let t = 100.0 / (10.0 + 60.0 / std::f64::consts::TAU);
        assert!((c.speed - 60.0 * t).abs() < 1e-9, "{}", c.speed);
        // With the 20x line both modes are crossed.
        assert_eq!(
            critical_speeds(&track(&points), &[20.0], Frame::Rotating).len(),
            2
        );
        // Standstill, where every order line starts at zero, is no critical speed.
        let still = vec![
            point(0.0, &[(0.0, Whirl::Unknown)]),
            point(60.0, &[(5.0, Whirl::Forward)]),
        ];
        assert!(critical_speeds(&track(&still), &[1.0], Frame::Rotating).is_empty());
    }

    /// A disc mode of two nodal diameters rising with the speed: in the stationary frame
    /// it is a forward wave f + 2 F and a backward wave |f − 2 F|, and the backward wave
    /// comes to rest where the mode crosses the 2x line of the rotating frame.
    #[test]
    fn the_stationary_frame_splits_a_disc_mode_into_travelling_waves() {
        let tau = std::f64::consts::TAU;
        let points: Vec<SpeedPoint> = (1..=16)
            .map(|i| {
                let speed = 25.0 * tau * f64::from(i);
                SpeedPoint {
                    speed,
                    modes: vec![Mode {
                        frequency: (120f64.powi(2) + 2.3 * (25.0 * f64::from(i)).powi(2)).sqrt(),
                        whirl: Whirl::Forward,
                        unstable: false,
                        nodal_diameter: Some(2),
                    }],
                }
            })
            .collect();
        let tracks = track(&points);
        assert_eq!(tracks.modes[0].name, "Mode 1 (F, ND 2)");
        let data = diagram(&tracks, &[1.0], 400.0 * tau, Frame::Stationary);
        let names: Vec<&str> = data.curves.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["Mode 1 forward (ND 2)", "Mode 1 backward (ND 2)"]);
        assert!(!data.curves[0].dashed && data.curves[1].dashed);
        let f = points[0].modes[0].frequency;
        assert!((data.curves[0].points[0][1] - (f + 50.0)).abs() < 1e-9);
        assert!((data.curves[1].points[0][1] - (f - 50.0).abs()).abs() < 1e-9);
        // The standing wave: f = 2 F, at F = 120 / sqrt(4 − 2.3) Hz.
        let critical = critical_speeds(&tracks, &[], Frame::Stationary);
        assert_eq!(critical.len(), 1, "{critical:?}");
        let c = &critical[0];
        assert_eq!(
            (c.order, c.branch, c.nodal_diameter),
            (0.0, Branch::Backward, Some(2))
        );
        assert_eq!(
            (c.order_label(), c.branch_label()),
            ("standing".into(), "backward wave, ND 2".into())
        );
        let expected = 120.0 / (4.0f64 - 2.3).sqrt();
        assert!(
            (c.speed / tau - expected).abs() < 2.0,
            "{} vs {expected}",
            c.speed / tau
        );
        // The same speed is the crossing with the 2x line in the rotating frame.
        let rotating = critical_speeds(&tracks, &[2.0], Frame::Rotating);
        assert_eq!(rotating.len(), 1);
        assert!((rotating[0].speed - c.speed).abs() < 1e-9);
        // Without a nodal diameter a mode stays out of the stationary frame.
        let mut unknown = points.clone();
        for p in &mut unknown {
            p.modes[0].nodal_diameter = None;
        }
        let data = diagram(&track(&unknown), &[], 400.0 * tau, Frame::Stationary);
        assert!(data.curves.is_empty());
        assert_eq!(data.left_out, ["Mode 1 (F)"]);
        assert!(critical_speeds(&track(&unknown), &[2.0], Frame::Stationary).is_empty());
    }

    #[test]
    fn the_diagram_names_modes_by_their_whirl_and_adds_order_lines() {
        let tau = std::f64::consts::TAU;
        let points = vec![
            point(tau, &[(90.0, Whirl::Backward), (110.0, Whirl::Forward)]),
            point(
                2.0 * tau,
                &[(80.0, Whirl::Backward), (120.0, Whirl::Forward)],
            ),
            point(
                3.0 * tau,
                &[(70.0, Whirl::Backward), (130.0, Whirl::Unknown)],
            ),
        ];
        let data = diagram(&track(&points), &[1.0, 2.0], 3.0 * tau, Frame::Rotating);
        let names: Vec<&str> = data.curves.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["Mode 1 (B)", "Mode 2"]);
        let [x, y] = data.curves[0].points[0];
        assert!((x - 60.0).abs() < 1e-9 && y == 90.0, "{x} {y}");
        let guides: Vec<&str> = data.guides.iter().map(|(n, _)| n.as_str()).collect();
        assert_eq!(guides, ["1x", "2x"]);
        // The order lines start at standstill, though the sweep leaves it out.
        assert_eq!(data.guides[0].1[0], [0.0, 0.0]);
        assert!((data.guides[1].1[1][0] - 180.0).abs() < 1e-9);
        assert!((data.guides[1].1[1][1] - 6.0).abs() < 1e-9);
    }

    /// The curves of a pair of modes with a lower one falling to zero and rising again, as
    /// in the rotating frame, with spurious roots at some speeds.
    fn sweep_with_spurious_roots() -> Vec<SpeedPoint> {
        (1..=20)
            .map(|i| {
                let s = f64::from(i) * 100.0;
                let mut modes = vec![
                    ((150.0 - 0.1 * s).abs(), Whirl::Forward),
                    (350.0 + 0.02 * s, Whirl::Backward),
                    (1000.0 + 0.05 * s, Whirl::Forward),
                    (2000.0, Whirl::Backward),
                ];
                // The solver finds a root near zero at two speeds, which pushes the top mode
                // out of the requested number of modes.
                if i == 9 || i == 14 {
                    modes.insert(0, (-3.0, Whirl::Unknown));
                    modes.pop();
                }
                modes.sort_by(|a, b| a.0.total_cmp(&b.0));
                point(s, &modes)
            })
            .collect()
    }

    #[test]
    fn modes_are_followed_by_their_course_not_their_rank() {
        let points = sweep_with_spurious_roots();
        let tracks = track(&points);
        assert_eq!(tracks.modes.len(), 4, "{:?}", tracks.modes);
        assert_eq!(tracks.unmatched, [(900.0, -3.0), (1400.0, -3.0)]);
        // Mode 2 stays on its line at the spurious speeds instead of dropping to mode 1.
        let second = &tracks.modes[1];
        assert_eq!(second.points.len(), 20);
        for (s, m) in &second.points {
            assert!(
                (m.frequency - (350.0 + 0.02 * s)).abs() < 1e-9,
                "{s}: {m:?}"
            );
        }
        // The lowest mode passes through zero and rises again as one curve.
        assert_eq!(tracks.modes[0].points.len(), 20);
        // The top mode just misses the two speeds.
        assert_eq!(tracks.modes[3].points.len(), 18);
        // Growing modes are kept apart.
        let mut points = points;
        points[4].modes.push(Mode {
            frequency: 0.0,
            whirl: Whirl::Unknown,
            unstable: true,
            nodal_diameter: None,
        });
        let tracks = track(&points);
        assert_eq!(tracks.unstable, [(500.0, 0.0)]);
        assert_eq!(tracks.modes.len(), 4);
    }

    #[test]
    fn crossing_modes_keep_their_curves() {
        let points: Vec<SpeedPoint> = (1..=10)
            .map(|i| {
                let s = f64::from(i);
                let mut modes = [
                    (100.0 + 10.0 * s, Whirl::Forward),
                    (200.0 - 10.0 * s, Whirl::Backward),
                ];
                modes.sort_by(|a, b| a.0.total_cmp(&b.0));
                point(s, &modes)
            })
            .collect();
        let tracks = track(&points);
        assert_eq!(tracks.modes.len(), 2);
        assert!(
            tracks.modes.iter().all(|t| t.whirl() != Whirl::Unknown),
            "{tracks:?}"
        );
    }

    #[test]
    fn refinement_goes_where_the_curves_bend() {
        // A mode with a kink at speed 50 of the range 0..100.
        let curve = |s: f64| {
            if s < 50.0 {
                100.0
            } else {
                100.0 + 4.0 * (s - 50.0)
            }
        };
        let mut points: Vec<SpeedPoint> = grid(0.0, 100.0, 8)
            .into_iter()
            .map(|s| point(s, &[(curve(s), Whirl::Forward), (1000.0, Whirl::Forward)]))
            .collect();
        for _ in 0..6 {
            let next = next_speed(&points, &[], &[], 0.0, 100.0).unwrap();
            assert!((30.0..=72.0).contains(&next), "{next}");
            points.push(point(
                next,
                &[(curve(next), Whirl::Forward), (1000.0, Whirl::Forward)],
            ));
        }
        // A running speed takes its gap out of the refinement.
        let running = next_speed(&points, &[], &[], 0.0, 100.0).unwrap();
        let mut taken: Vec<f64> = points.iter().map(|p| p.speed).collect();
        taken.sort_by(f64::total_cmp);
        let after = taken.iter().position(|&s| s > running).unwrap();
        let (before, after) = (taken[after - 1], taken[after]);
        let next = next_speed(&points, &[], &[running], 0.0, 100.0).unwrap();
        assert!(
            next <= before || next >= after,
            "{next} in {before}..{after}"
        );
        // Straight curves spread the speeds evenly: the widest gap first.
        let straight: Vec<SpeedPoint> = [10.0, 20.0, 60.0, 70.0]
            .into_iter()
            .map(|s| point(s, &[(s, Whirl::Forward)]))
            .collect();
        assert_eq!(next_speed(&straight, &[], &[], 0.0, 100.0), Some(40.0));
        assert_eq!(round_range(19815.49, 34924.93), [19800.0, 34900.0]);
        // A marked range is refined on its own, also inside a single gap, and finer.
        let next = next_speed(&straight, &[], &[], 25.0, 30.0).unwrap();
        assert_eq!(next, 27.5);
        let mut fine = straight.clone();
        for _ in 0..20 {
            let Some(next) = next_speed(&fine, &[], &[], 25.0, 30.0) else {
                break;
            };
            assert!((25.0..=30.0).contains(&next), "{next}");
            fine.push(point(next, &[(next, Whirl::Forward)]));
        }
        assert_eq!(fine.len(), 24);
        // No gap wider than the resolution is left.
        let dense: Vec<SpeedPoint> = [10.0, 10.1, 10.2]
            .into_iter()
            .map(|s| point(s, &[(s, Whirl::Forward)]))
            .collect();
        assert_eq!(next_speed(&dense, &[], &[], 0.0, 100.0), None);
    }

    #[test]
    fn refinement_does_not_chase_spurious_roots() {
        let points = sweep_with_spurious_roots();
        // The spurious roots are no curve, so the straight curves only spread the speeds;
        // the bend of the lowest mode at its zero (speed 1500) draws the first new speed.
        let next = next_speed(&points, &[], &[], 0.0, 2000.0).unwrap();
        assert!((1400.0..=1600.0).contains(&next), "{next}");
    }

    #[test]
    fn the_table_lists_speeds_modes_and_order_lines() {
        let tau = std::f64::consts::TAU;
        let points = vec![
            point(tau, &[(90.0, Whirl::Backward), (110.0, Whirl::Forward)]),
            point(
                2.0 * tau,
                &[(80.0, Whirl::Backward), (120.0, Whirl::Forward)],
            ),
            point(3.0 * tau, &[(70.0, Whirl::Backward), (5.0, Whirl::Unknown)]),
        ];
        let csv = table_text(&points, &[1.0, 2.0], ',');
        let rows: Vec<Vec<&str>> = csv.lines().map(|l| l.split(',').collect()).collect();
        assert_eq!(
            rows[0],
            [
                "Speed [rad/s]",
                "Speed [rpm]",
                "Mode 1 [Hz]",
                "Whirl 1",
                "ND 1",
                "Other roots [Hz]",
                "1x [Hz]",
                "2x [Hz]"
            ]
        );
        assert_eq!(rows[1][1..], ["60", "90", "B", "", "110", "1", "2"]);
        assert_eq!(rows[3][1..], ["180", "70", "B", "", "5", "3", "6"]);
        assert_eq!(rows.len(), 4);
    }

    #[test]
    fn a_wider_range_adds_a_grid_beyond_the_swept_speeds() {
        let swept: Vec<SpeedPoint> = [100.0, 200.0, 300.0]
            .iter()
            .map(|&s| point(s, &[(10.0, Whirl::Forward)]))
            .collect();
        // Within the swept speeds nothing is added: refinement goes between them.
        assert!(extension(&swept, 100.0, 300.0, 8).is_empty());
        assert!(extension(&swept, 150.0, 250.0, 8).is_empty());
        // Above: a grid from the highest speed on, without it.
        let above = extension(&swept, 100.0, 500.0, 8);
        // Half the range gets half the grid.
        assert_eq!(above, vec![350.0, 400.0, 450.0, 500.0]);
        // Below: down to the lowest speed of the range, without the lowest swept one.
        let below = extension(&swept, 20.0, 300.0, 8);
        assert_eq!(below.first(), Some(&20.0));
        assert!(below.iter().all(|&s| s < 100.0), "{below:?}");
        // From standstill the grid starts at a fiftieth of the highest speed, as a new sweep
        // would: a sweep from standstill made wider adds nothing below its speeds.
        let wider = extension(&swept, 0.0, 5000.0, 8);
        assert!(wider.iter().all(|&s| s > 300.0), "{wider:?}");
        // No speeds yet: the whole grid.
        assert_eq!(extension(&[], 100.0, 300.0, 3), vec![100.0, 200.0, 300.0]);
    }

    #[test]
    fn setting_the_speed_scales_the_complex_frequency_steps_exactly() {
        let spin = |name: &str, speed: f64| {
            let mut step = plx_model::Step::new_complex_frequency(name);
            if let StepKind::ComplexFrequency(c) = &mut step.kind {
                c.speed = speed;
            }
            step
        };
        let mut fe = FeModel {
            steps: vec![spin("Step-1", 3.0), spin("Step-2", 1.5)],
            ..FeModel::default()
        };
        let speeds = |fe: &FeModel| -> Vec<f64> {
            (fe.steps.iter())
                .filter_map(|s| match &s.kind {
                    StepKind::ComplexFrequency(c) => Some(c.speed),
                    _ => None,
                })
                .collect()
        };
        set_speed(&mut fe, 2042.035);
        // The fastest step turns at exactly the speed, the others keep their ratio.
        assert_eq!(speeds(&fe), vec![2042.035, 2042.035 * 0.5]);
        assert_eq!(rotor_speed(&fe), Some(2042.035));
    }

    #[test]
    fn orders_are_parsed_and_the_grid_spreads_evenly() {
        let setup = Setup {
            min_speed: 0.0,
            max_speed: 100.0,
            max_runs: 0,
            orders: "2, 1; 1 x".into(),
            max_frequency: 0.0,
            frame: Frame::Rotating,
        };
        assert_eq!(order_values(&setup.orders), [1.0, 2.0]);
        assert_eq!(grid(0.0, 100.0, 3), [2.0, 51.0, 100.0]);
        assert_eq!(grid(40.0, 100.0, 4), [40.0, 60.0, 80.0, 100.0]);
        assert!((rpm(from_rpm(3000.0)) - 3000.0).abs() < 1e-9);
    }

    /// The spinning test cantilever, swept over three speeds: the bending modes split more
    /// the faster it turns, and the sweep tells forward from backward.
    #[test]
    fn the_sweep_splits_the_modes_of_the_spinning_cantilever() {
        let testdata = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/kragbalken_c3d8.inp");
        let mesh = plx_io::inp::read_inp(&testdata).unwrap().mesh;
        let mut fe = FeModel {
            materials: vec![plx_model::Material {
                id: crate::test_ids::named("Steel"),
                name: "Steel".into(),
                density: Some(7.85e-9),
                elastic: Some(plx_model::Elastic {
                    young: 210_000.0,
                    poisson: 0.3,
                }),
                ..Default::default()
            }],
            sections: vec![plx_model::Section {
                name: "Section-1".into(),
                material: Some(crate::test_ids::named("Steel")),
                region: plx_model::Region::Parts(vec!["EALL".into()]),
                thickness: 1.0,
                kind: plx_model::SectionKind::Solid,
            }],
            ..FeModel::default()
        };
        let fixed = plx_model::BoundaryCondition {
            id: crate::test_ids::named("Fixed-1"),
            distribution: None,
            name: "Fixed-1".into(),
            active: true,
            visible: true,
            region: plx_model::Region::NodeSet("FIX".into()),
            kind: plx_model::BoundaryKind::displacement([Some(0.0); 6]),
            amplitude: None,
        };
        let mut complex = plx_model::Step::new_complex_frequency("Step-1");
        complex.boundary_conditions.push(fixed);
        if let StepKind::ComplexFrequency(c) = &mut complex.kind {
            c.num_frequencies = 4;
            c.num_modes = 4;
            c.point = [0.0, 5.0, 5.0];
            c.axis = [1.0, 0.0, 0.0];
            c.speed = 200.0;
        }
        fe.steps = vec![complex];
        assert!(applies(&fe));
        assert_eq!(
            (rotor_speed(&fe), complex_step(&fe)),
            (Some(200.0), Some(0))
        );
        // A rotor at standstill sweeps all the same: its steps take the speed, and the
        // fingerprint of the sweep leaves the speed out.
        let mut still = fe.clone();
        let speed_of = |fe: &FeModel| rotor_speed(fe).unwrap();
        if let StepKind::ComplexFrequency(c) = &mut still.steps[0].kind {
            c.speed = 0.0;
        }
        assert!(applies(&still));
        assert_eq!(speed_of(&with_speed(&still, 150.0)), 150.0);
        assert_eq!(speed_of(&with_speed(&fe, 50.0)), 50.0);
        assert_eq!(
            sweep_fingerprint(&mesh, &still),
            sweep_fingerprint(&mesh, &fe)
        );

        let work_dir = std::env::temp_dir().join(format!("plx-campbell-{}", std::process::id()));
        let input = SweepInput {
            threads: 1,
            work_dir: work_dir.clone(),
            mesh: Arc::new(mesh),
            fe,
            step: 0,
        };
        let cancel = AtomicBool::new(false);
        let points: Vec<SpeedPoint> = (grid(50.0, 200.0, 3).into_iter().enumerate())
            .map(|(i, speed)| run_speed(&input, speed, i as u32, &cancel).unwrap())
            .collect();
        // The result fields have the same frequencies as the history output.
        let solved = plx_results::SolverResults::read(&work_dir.join("Campbell-2.ndlr")).unwrap();
        let frd = plx_io::solver::import(&solved);
        let from_frd: Vec<f64> = modes_of_frd(&frd, 3).iter().map(|m| m.frequency).collect();
        let _ = std::fs::remove_dir_all(&work_dir);
        assert_eq!(points.len(), 3);
        let from_dat: Vec<f64> = points[2].modes.iter().map(|m| m.frequency).collect();
        assert_eq!(
            from_frd.len(),
            from_dat.len(),
            "{from_frd:?} vs {from_dat:?}"
        );
        for (a, b) in from_frd.iter().zip(&from_dat) {
            assert!(
                (a - b).abs() < 1e-3 * b.abs().max(1.0),
                "{from_frd:?} vs {from_dat:?}"
            );
        }
        // In the rotating frame the pair splits by twice the speed: 2 x 200 rad/s = 63.7 Hz
        // at the top speed, a quarter of it at the first.
        let split = |p: &SpeedPoint| p.modes[1].frequency - p.modes[0].frequency;
        assert!((13.0..19.0).contains(&split(&points[0])), "{:?}", points[0]);
        assert!((55.0..72.0).contains(&split(&points[2])), "{points:?}");
        // The solver computes in the rotating frame of reference, as CalculiX does, where
        // the forward whirl appears below the backward one; both directions occur in the
        // pair.
        let whirls = [points[2].modes[0].whirl, points[2].modes[1].whirl];
        assert!(whirls.contains(&Whirl::Forward) && whirls.contains(&Whirl::Backward));
        let tracks = track(&points);
        let critical = critical_speeds(&tracks, &[1.0], Frame::Rotating);
        assert!(critical.is_empty(), "{critical:?}");
        assert_eq!(
            diagram(&tracks, &[1.0], 200.0, Frame::Rotating)
                .curves
                .len(),
            4
        );
        // The bending pair of a shaft has one nodal diameter: in the stationary frame its
        // forward and backward waves are the pair's frequencies shifted by the speed.
        for t in &tracks.modes {
            assert_eq!(t.nodal_diameter(), Some(1), "{t:?}");
        }
        let stationary = diagram(&tracks, &[1.0], 200.0, Frame::Stationary);
        assert_eq!(stationary.curves.len(), 8);
        assert!(stationary.left_out.is_empty());
        assert!(table_text(&points, &[1.0], ',').lines().count() == 4);
    }

    /// Writes the demo project of the user guide's Campbell chapter: a spinning disc (a CD
    /// of polycarbonate as S4 shells, clamped at the hub) swept to 16000 rpm, its diagram
    /// set to the stationary frame. Run by hand: `NODALIS_DEMO=/tmp/disc.ndl cargo test -p plx-app --release
    /// -- --ignored demo_project`.
    #[test]
    #[ignore]
    fn demo_project_of_a_spinning_disc() {
        use std::fmt::Write as _;
        let Ok(path) = std::env::var("NODALIS_DEMO") else {
            return;
        };
        let (inner, outer, rings, around) = (7.5e-3, 60e-3, 8, 48);
        let mut inp = String::from("*HEADING\nDisc\n*NODE, NSET=NALL\n");
        for i in 0..=rings {
            let r = inner + (outer - inner) * f64::from(i) / f64::from(rings);
            for j in 0..around {
                let t = std::f64::consts::TAU * f64::from(j) / f64::from(around);
                let id = i * around + j + 1;
                writeln!(inp, "{id}, {:.8e}, {:.8e}, 0", r * t.cos(), r * t.sin()).unwrap();
            }
        }
        inp.push_str("*NSET, NSET=HUB, GENERATE\n");
        writeln!(inp, "1, {around}").unwrap();
        inp.push_str("*ELEMENT, TYPE=S4, ELSET=DISC\n");
        for i in 0..rings {
            for j in 0..around {
                let j2 = (j + 1) % around;
                let n = |i: u32, j: u32| i * around + j + 1;
                let id = i * around + j + 1;
                writeln!(
                    inp,
                    "{id}, {}, {}, {}, {}",
                    n(i, j),
                    n(i + 1, j),
                    n(i + 1, j2),
                    n(i, j2)
                )
                .unwrap();
            }
        }
        let mesh = plx_io::inp::read_inp_str(&inp, None).unwrap().mesh;
        let polycarbonate = plx_model::Id::new();
        let mut fe = FeModel {
            materials: vec![plx_model::Material {
                id: polycarbonate,
                name: "Polycarbonate".into(),
                density: Some(1200.0),
                elastic: Some(plx_model::Elastic {
                    young: 2.3e9,
                    poisson: 0.37,
                }),
                ..Default::default()
            }],
            sections: vec![plx_model::Section {
                name: "Shell-1".into(),
                material: Some(polycarbonate),
                region: plx_model::Region::Parts(vec!["DISC".into()]),
                thickness: 1.0,
                kind: plx_model::SectionKind::Shell {
                    thickness: 1.2e-3,
                    offset: 0.0,
                },
            }],
            properties: plx_model::ModelProperties {
                units: plx_model::UnitSystem::MKgSC,
                ..Default::default()
            },
            ..FeModel::default()
        };
        let mut complex = plx_model::Step::new_complex_frequency("Step-1");
        complex
            .boundary_conditions
            .push(plx_model::BoundaryCondition {
                id: plx_model::Id::new(),
                distribution: None,
                name: "Hub".into(),
                active: true,
                visible: true,
                region: plx_model::Region::NodeSet("HUB".into()),
                kind: plx_model::BoundaryKind::displacement([Some(0.0); 6]),
                amplitude: None,
            });
        if let StepKind::ComplexFrequency(c) = &mut complex.kind {
            c.num_frequencies = 8;
            c.num_modes = 24;
            c.axis = [0.0, 0.0, 1.0];
            c.speed = from_rpm(16_000.0);
        }
        fe.steps = vec![complex];
        let work_dir = std::env::temp_dir().join(format!("plx-disc-{}", std::process::id()));
        let input = SweepInput {
            threads: 2,
            work_dir: work_dir.clone(),
            mesh: Arc::new(mesh.clone()),
            fe: fe.clone(),
            step: 0,
        };
        let cancel = AtomicBool::new(false);
        let points: Vec<SpeedPoint> = (grid(from_rpm(1000.0), from_rpm(16_000.0), 16)
            .into_iter()
            .enumerate())
        .map(|(i, speed)| run_speed(&input, speed, i as u32, &cancel).unwrap())
        .collect();
        let _ = std::fs::remove_dir_all(&work_dir);
        let setup = Setup {
            min_speed: 0.0,
            max_speed: from_rpm(16_000.0),
            max_runs: 16,
            orders: "1, 2, 3".into(),
            max_frequency: 1500.0,
            frame: Frame::Stationary,
        };
        let fingerprint = sweep_fingerprint(&mesh, &fe);
        let project = plx_io::project::Project {
            nodes: vec![
                plx_io::project::ProjectNode::Analysis(Box::new(
                    plx_io::project::ProjectAnalysis {
                        id: 1,
                        name: "Disc".into(),
                        position: [-200.0, 0.0],
                        mesh,
                        model: fe,
                        ..Default::default()
                    },
                )),
                plx_io::project::ProjectNode::Campbell(Box::new(plx_io::project::CampbellBlock {
                    id: 2,
                    name: "Campbell".into(),
                    position: [140.0, 40.0],
                    setup,
                    points,
                    focus: None,
                    fingerprint: Some(fingerprint),
                })),
            ],
            links: vec![plx_io::project::Link {
                source: 1,
                target: 2,
                item: vec!["Setup".into()],
                slot: vec!["Analysis".into()],
                ..Default::default()
            }],
            ..Default::default()
        };
        plx_io::project::save_project(std::path::Path::new(&path), &project).unwrap();
    }

    #[test]
    fn complex_frequencies_are_read_from_the_dat_text() {
        let dat = "
                        S T E P       3

     E I G E N V A L U E   O U T P U T

 MODE NO                     FREQUENCY
                      REAL PART         IMAGINARY PART
             (RAD/TIME)   (CYCLES/TIME)   (RAD/TIME)

      1   0.4602752E+04   0.7325508E+03   0.4018004E-11
      2   0.6595139E+04   0.1049649E+04   0.5608028E-12

     E I G E N M O D E   T U R N I N G   D I R E C T I O N

    Axis reference direction:  0.1000E+01  0.0000E+00  0.0000E+00

 MODE NO     TURNING DIRECTION (F=FORWARD,B=BACKWARD)   NODAL DIAMETER

      1          F                            1
      2          B                            -
";
        let modes = modes_of(dat, 3);
        assert_eq!(
            modes,
            [
                Mode {
                    frequency: 732.5508,
                    whirl: Whirl::Forward,
                    unstable: false,
                    nodal_diameter: Some(1),
                },
                Mode {
                    frequency: 1049.649,
                    whirl: Whirl::Backward,
                    unstable: false,
                    nodal_diameter: None,
                }
            ]
        );
        assert!(modes_of(dat, 2).is_empty());
        // CalculiX's table without the column.
        let without = dat
            .replace("   NODAL DIAMETER", "")
            .replace("                            1", "")
            .replace("                            -", "");
        let modes = modes_of(&without, 3);
        assert_eq!(modes.len(), 2);
        assert!(modes.iter().all(|m| m.nodal_diameter.is_none()));
    }
}
