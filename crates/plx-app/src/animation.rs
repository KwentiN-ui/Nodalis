//! Result animation: either the deformation of one increment grows from zero to
//! its full scale, or the increments of a step, of all steps or the positions of a moving load
//! play one after another. A mode shape of a
//! frequency or buckling step swings from -1 to 1 instead, so both halves of the oscillation show.
//!
//! The increments of steps that run over time can play by their time instead of one per frame:
//! the animation then takes a set duration, each frame lies at an even simulation time and is
//! interpolated between the two increments around it, so the extra frames the solver writes at
//! an impact do not slow the playback down there.

/// What the frames of an animation show.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnimationKind {
    /// The shown result scaled from 0 to 1 over the frames, from -1 to 1 for a mode shape
    /// (label "Scale factor").
    ScaleFactor,
    /// Each frame is one increment of the shown step (label "Step").
    Step,
    /// Each frame is one increment of all the steps, as the timeline shows them, of the kind
    /// of the shown step: timed steps one after another, or the modes of the modal steps.
    AllSteps,
    /// Each frame is one position of the moving load of the shown step, its last increment
    /// there.
    Positions,
}

impl AnimationKind {
    /// The name in the animation window and the menu of Animate.
    pub fn label(self) -> &'static str {
        match self {
            Self::ScaleFactor => "Scale factor",
            Self::Step => "Step",
            Self::AllSteps => "All steps",
            Self::Positions => "Positions",
        }
    }
}

impl AnimationKind {
    /// Whether the frames are increments of the results rather than a scaled one.
    pub fn plays_increments(self) -> bool {
        matches!(
            self,
            AnimationKind::Step | AnimationKind::AllSteps | AnimationKind::Positions
        )
    }

    /// Whether the increments can play by their time: positions of a moving load have none.
    pub fn plays_by_time(self) -> bool {
        matches!(self, AnimationKind::Step | AnimationKind::AllSteps)
    }
}

/// How the increments of an animation are spaced over its frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pace {
    /// One increment per frame, however far apart in time they lie.
    Frames,
    /// The frames lie at even simulation times over [`Animation::duration`] seconds of
    /// playback, interpolated between the increments around them.
    Time,
}

/// What happens at the last frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Playback {
    Once,
    Loop,
    /// Back and forth.
    Swing,
}

/// Range the colour legend covers while animating.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ColorLimits {
    /// Minimum and maximum of the frame on screen; the legend changes with each frame.
    CurrentFrame,
    /// Minimum and maximum over all frames; colours stay comparable between frames.
    AllFrames,
}

pub const DEFAULT_FRAMES: u32 = 60;
pub const DEFAULT_FPS: f32 = 30.0;
/// Seconds an animation by time plays in.
pub const DEFAULT_DURATION: f32 = 4.0;

pub struct Animation {
    pub kind: AnimationKind,
    /// Number of frames of a scale factor animation.
    pub frames: u32,
    pub fps: f32,
    pub playback: Playback,
    pub limits: ColorLimits,
    pub frame: usize,
    pub playing: bool,
    /// Increment indices of an increment animation, in order.
    pub increments: Vec<usize>,
    /// Total time of each of [`Animation::increments`], when every one has a time and they
    /// span more than one moment; empty otherwise, and the increments play frame by frame.
    pub times: Vec<f64>,
    /// How timed increments are spaced over the frames.
    pub pace: Pace,
    /// Seconds a playback by time takes from the first to the last increment.
    pub duration: f32,
    /// Increment shown before the animation started, shown again when it ends.
    pub start_increment: usize,
    /// The shown result is a mode shape (frequency or buckling step): scaling runs from -1 to 1.
    pub modal: bool,
    forward: bool,
    elapsed: f32,
}

impl Animation {
    pub fn new(
        kind: AnimationKind,
        increments: Vec<usize>,
        times: Vec<f64>,
        start_increment: usize,
        modal: bool,
    ) -> Self {
        let mut animation = Self {
            kind,
            frames: DEFAULT_FRAMES,
            fps: DEFAULT_FPS,
            playback: Playback::Swing,
            limits: ColorLimits::CurrentFrame,
            frame: DEFAULT_FRAMES as usize - 1,
            playing: false,
            increments: Vec::new(),
            times: Vec::new(),
            pace: Pace::Time,
            duration: DEFAULT_DURATION,
            start_increment,
            modal,
            forward: true,
            elapsed: 0.0,
        };
        animation.retarget(increments, times, start_increment, modal);
        animation
    }

    /// Moves the animation to another increment, possibly of another step, keeping the
    /// settings, the playing state and, for a scale factor animation, the frame.
    pub fn retarget(
        &mut self,
        increments: Vec<usize>,
        times: Vec<f64>,
        start_increment: usize,
        modal: bool,
    ) {
        self.modal = modal;
        self.start_increment = start_increment;
        let timed = times.len() == increments.len()
            && times.windows(2).all(|w| w[0] <= w[1])
            && times
                .first()
                .is_some_and(|first| times.last() > Some(first));
        self.times = if timed { times } else { Vec::new() };
        self.increments = increments;
        if self.kind.plays_increments() {
            // An increment animation starts where the user was looking.
            let position = (self.increments.iter())
                .position(|&i| i == start_increment)
                .unwrap_or(0);
            // By time, the first frame at (or just after) the increment, so that it is the
            // increment shown.
            self.frame = match self.times.get(position) {
                Some(&time) if self.by_time() => self.frame_at(time, f64::ceil),
                _ => position,
            };
        }
    }

    /// Whether the frames lie at even simulation times rather than one per increment.
    pub fn by_time(&self) -> bool {
        self.pace == Pace::Time && self.can_play_by_time()
    }

    /// Whether the increments have times to play by.
    pub fn can_play_by_time(&self) -> bool {
        self.kind.plays_by_time() && !self.times.is_empty()
    }

    pub fn frame_count(&self) -> usize {
        match self.kind {
            AnimationKind::ScaleFactor => self.frames.max(2) as usize,
            _ if self.by_time() => ((self.duration * self.fps).round() as usize).max(2),
            AnimationKind::Step | AnimationKind::AllSteps | AnimationKind::Positions => {
                self.increments.len().max(1)
            }
        }
    }

    /// Position of the shown frame of a scale factor animation, from 0 at the first frame to
    /// 1 at the last.
    pub fn progress(&self) -> f32 {
        self.frame.min(self.frame_count() - 1) as f32 / (self.frame_count() - 1) as f32
    }

    /// First and last time of the increments, for a playback by time.
    fn span(&self) -> Option<(f64, f64)> {
        Some((*self.times.first()?, *self.times.last()?))
    }

    /// Simulation time of the shown frame of a playback by time.
    pub fn time(&self) -> Option<f64> {
        if !self.by_time() {
            return None;
        }
        let (first, last) = self.span()?;
        Some(first + (last - first) * f64::from(self.progress()))
    }

    /// The frame of a playback by time at `time`, rounded by `round` (nearest, or the first
    /// frame at or after the time).
    fn frame_at(&self, time: f64, round: fn(f64) -> f64) -> usize {
        let Some((first, last)) = self.span() else {
            return 0;
        };
        let last_frame = self.frame_count() - 1;
        let fraction = ((time - first) / (last - first)).clamp(0.0, 1.0);
        // A hair below a frame counts as that frame.
        (round(fraction * last_frame as f64 - 1e-9) as usize).min(last_frame)
    }

    /// Factor applied to deformation and values of the shown frame.
    pub fn amplitude(&self) -> f32 {
        match self.kind {
            AnimationKind::ScaleFactor => {
                let t = self.progress();
                if self.modal {
                    // The modal ratios: a sine from -1 to 1, slow at the turning points
                    // like the oscillation itself.
                    ((2.0 * t - 1.0) * std::f32::consts::FRAC_PI_2).sin()
                } else {
                    t
                }
            }
            AnimationKind::Step | AnimationKind::AllSteps | AnimationKind::Positions => 1.0,
        }
    }

    /// Increment of the shown frame, for increment animations: by time, the last increment at
    /// or before the frame's time.
    pub fn increment(&self) -> Option<usize> {
        match self.kind {
            AnimationKind::ScaleFactor => None,
            _ if self.by_time() => self.increments.get(self.blend()?.0).copied(),
            AnimationKind::Step | AnimationKind::AllSteps | AnimationKind::Positions => {
                self.increments.get(self.frame).copied()
            }
        }
    }

    /// Where the shown frame of a playback by time lies between the increments: the position
    /// of the increment at or before its time in [`Animation::increments`] and the fraction of
    /// the way to the next one, 0 at an increment itself.
    pub fn blend(&self) -> Option<(usize, f64)> {
        let time = self.time()?;
        let after = self.times.partition_point(|&t| t <= time);
        let position = after.checked_sub(1)?;
        let fraction = match self.times.get(after) {
            Some(&next) if next > self.times[position] => {
                (time - self.times[position]) / (next - self.times[position])
            }
            _ => 0.0,
        };
        Some((position, fraction))
    }

    /// The increment the shown frame is interpolated towards, with the fraction of the way to
    /// it, when it lies between two increments.
    pub fn interpolation(&self) -> Option<(usize, f32)> {
        let (position, fraction) = self.blend()?;
        if fraction <= 0.0 {
            return None;
        }
        let next = *self.increments.get(position + 1)?;
        Some((next, fraction as f32))
    }

    /// Jumps to a frame; returns true when the shown frame changed.
    pub fn go_to(&mut self, frame: usize) -> bool {
        let frame = frame.min(self.frame_count() - 1);
        let changed = frame != self.frame;
        self.frame = frame;
        changed
    }

    /// Changes the pace, the duration or the frame rate of a playback by time, keeping the
    /// shown moment where it can.
    pub fn retime(&mut self, change: impl FnOnce(&mut Self)) {
        let time = self.time();
        let position = self.blend().map(|(position, _)| position);
        change(self);
        self.frame = match (time, self.by_time()) {
            (Some(time), true) => self.frame_at(time, f64::round),
            (Some(_), false) => position.unwrap_or(0).min(self.frame_count() - 1),
            (None, true) => match self.times.get(self.frame) {
                Some(&time) => self.frame_at(time, f64::ceil),
                None => 0,
            },
            (None, false) => self.frame.min(self.frame_count() - 1),
        };
    }

    /// Advances playback by `dt` seconds; returns true when the shown frame changed.
    pub fn tick(&mut self, dt: f32) -> bool {
        if !self.playing {
            return false;
        }
        self.elapsed += dt;
        let period = 1.0 / self.fps.max(0.1);
        let mut changed = false;
        while self.elapsed >= period {
            self.elapsed -= period;
            changed |= self.advance();
            if !self.playing {
                self.elapsed = 0.0;
                break;
            }
        }
        changed
    }

    fn advance(&mut self) -> bool {
        let last = self.frame_count() - 1;
        if last == 0 {
            self.playing = false;
            return false;
        }
        let before = self.frame;
        match self.playback {
            Playback::Once => {
                if self.frame < last {
                    self.frame += 1;
                }
                if self.frame == last {
                    self.playing = false;
                }
            }
            Playback::Loop => {
                self.frame = if self.frame >= last {
                    0
                } else {
                    self.frame + 1
                }
            }
            Playback::Swing => {
                if self.forward && self.frame >= last {
                    self.forward = false;
                } else if !self.forward && self.frame == 0 {
                    self.forward = true;
                }
                self.frame = if self.forward {
                    self.frame + 1
                } else {
                    self.frame - 1
                };
            }
        }
        self.frame != before
    }

    /// Starts playing; a finished one-shot animation starts over.
    pub fn play(&mut self) {
        if self.playback == Playback::Once && self.frame + 1 >= self.frame_count() {
            self.frame = 0;
        }
        self.playing = true;
        self.elapsed = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scale_animation(frames: u32, playback: Playback) -> Animation {
        let mut animation =
            Animation::new(AnimationKind::ScaleFactor, Vec::new(), Vec::new(), 0, false);
        animation.frames = frames;
        animation.playback = playback;
        animation.frame = 0;
        animation.play();
        animation
    }

    fn sequence(animation: &mut Animation, steps: usize) -> Vec<usize> {
        let period = 1.0 / animation.fps;
        (0..steps)
            .map(|_| {
                animation.tick(period);
                animation.frame
            })
            .collect()
    }

    #[test]
    fn scale_factor_runs_from_zero_to_full() {
        let mut animation = scale_animation(5, Playback::Once);
        assert_eq!(animation.amplitude(), 0.0);
        assert_eq!(sequence(&mut animation, 6), [1, 2, 3, 4, 4, 4]);
        assert_eq!(animation.amplitude(), 1.0);
        assert!(!animation.playing);
    }

    #[test]
    fn mode_shape_swings_from_minus_one_to_one() {
        let mut animation = scale_animation(5, Playback::Once);
        animation.modal = true;
        let amplitudes: Vec<f32> = (0..5)
            .map(|frame| {
                animation.go_to(frame);
                animation.amplitude()
            })
            .collect();
        let expected = [-1.0, -std::f32::consts::FRAC_1_SQRT_2, 0.0, 0.70710677, 1.0];
        for (a, e) in amplitudes.iter().zip(expected) {
            assert!((a - e).abs() < 1e-6, "{amplitudes:?}");
        }
    }

    #[test]
    fn swing_turns_at_both_ends_and_loop_wraps() {
        let mut swing = scale_animation(3, Playback::Swing);
        assert_eq!(sequence(&mut swing, 6), [1, 2, 1, 0, 1, 2]);
        let mut looped = scale_animation(3, Playback::Loop);
        assert_eq!(sequence(&mut looped, 4), [1, 2, 0, 1]);
    }

    #[test]
    fn increment_animation_starts_at_the_shown_increment() {
        let animation = Animation::new(AnimationKind::Step, vec![3, 4, 5], Vec::new(), 4, false);
        assert_eq!(animation.frame, 1);
        assert_eq!(animation.increment(), Some(4));
        assert_eq!(animation.amplitude(), 1.0);
        assert!(!animation.can_play_by_time());
        assert_eq!(animation.frame_count(), 3);
    }

    /// Increments at 0, 1, 1.1 and 2 s (an event frame close after the one at 1 s): played
    /// by time over 4 s at 1 frame per second, the frames lie at 0, 0.5, 1, 1.5 and 2 s.
    fn timed_animation() -> Animation {
        let mut animation = Animation::new(
            AnimationKind::Step,
            vec![10, 11, 12, 13],
            vec![0.0, 1.0, 1.1, 2.0],
            12,
            false,
        );
        animation.retime(|a| a.fps = 1.0);
        animation
    }

    #[test]
    fn a_playback_by_time_spaces_the_frames_evenly_and_interpolates() {
        let mut animation = timed_animation();
        assert!(animation.by_time());
        assert_eq!(animation.frame_count(), 4);
        // Started at the increment at 1.1 s: the first frame after it is the one at 1.33 s.
        assert_eq!(animation.frame, 2);
        assert_eq!(animation.increment(), Some(12));
        // A longer duration keeps the moment: 1.33 s is nearest to the frame at 1.5 s, which
        // lies between the increments at 1.1 and 2 s.
        animation.retime(|a| a.duration = 5.0);
        assert_eq!(animation.frame_count(), 5);
        assert_eq!(animation.frame, 3);
        assert_eq!(animation.time(), Some(1.5));
        assert_eq!(animation.increment(), Some(12));
        let (next, fraction) = animation.interpolation().unwrap();
        assert_eq!(next, 13);
        assert!((fraction - 0.4 / 0.9).abs() < 1e-6, "{fraction}");
        animation.go_to(2);
        assert_eq!(animation.time(), Some(1.0));
        assert_eq!(animation.increment(), Some(11));
        assert_eq!(animation.interpolation(), None);
        animation.go_to(1);
        assert_eq!(animation.increment(), Some(10));
        let (next, fraction) = animation.interpolation().unwrap();
        assert_eq!(next, 11);
        assert!((fraction - 0.5).abs() < 1e-6);
        animation.go_to(4);
        assert_eq!(animation.increment(), Some(13));
        assert_eq!(animation.interpolation(), None);
    }

    #[test]
    fn switching_the_pace_keeps_the_shown_moment() {
        let mut animation = timed_animation();
        animation.retime(|a| a.duration = 5.0);
        animation.go_to(3);
        animation.retime(|a| a.pace = Pace::Frames);
        assert!(!animation.by_time());
        assert_eq!(animation.frame_count(), 4);
        assert_eq!(animation.increment(), Some(12));
        assert_eq!(animation.interpolation(), None);
        animation.go_to(1);
        animation.retime(|a| a.pace = Pace::Time);
        assert_eq!(animation.time(), Some(1.0));
    }

    #[test]
    fn increments_without_times_play_frame_by_frame() {
        let animation = Animation::new(
            AnimationKind::Step,
            vec![1, 2, 3],
            vec![0.0, 0.0, 0.0],
            1,
            false,
        );
        assert!(!animation.can_play_by_time());
        assert_eq!(animation.frame, 0);
        let positions = Animation::new(
            AnimationKind::Positions,
            vec![1, 2],
            vec![0.0, 1.0],
            1,
            false,
        );
        assert!(!positions.can_play_by_time());
    }
}
