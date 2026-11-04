//! Result animation: either the deformation of one increment grows from zero to
//! its full scale, or the increments of a step play one after another, or the steps (their last
//! increments) do. A mode shape of a
//! frequency or buckling step swings from -1 to 1 instead, so both halves of the oscillation show.

/// What the frames of an animation show.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnimationKind {
    /// The shown result scaled from 0 to 1 over the frames, from -1 to 1 for a mode shape
    /// (label "Scale factor").
    ScaleFactor,
    /// Each frame is one increment of the shown step (label "Time increments").
    Increments,
    /// Each frame is the last increment of one step, e.g. one position of a moving load.
    Steps,
}

impl AnimationKind {
    /// Whether the frames are increments of the results rather than a scaled one.
    pub fn plays_increments(self) -> bool {
        matches!(self, AnimationKind::Increments | AnimationKind::Steps)
    }
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
        start_increment: usize,
        modal: bool,
    ) -> Self {
        // An increment animation starts where the user was looking.
        let frame = increments
            .iter()
            .position(|&i| i == start_increment)
            .unwrap_or(0);
        Self {
            kind,
            frames: DEFAULT_FRAMES,
            fps: DEFAULT_FPS,
            playback: Playback::Swing,
            limits: ColorLimits::CurrentFrame,
            frame: if kind.plays_increments() {
                frame
            } else {
                DEFAULT_FRAMES as usize - 1
            },
            playing: false,
            increments,
            start_increment,
            modal,
            forward: true,
            elapsed: 0.0,
        }
    }

    /// Moves the animation to another increment, possibly of another step, keeping the
    /// settings, the playing state and, for a scale factor animation, the frame.
    pub fn retarget(&mut self, increments: Vec<usize>, start_increment: usize, modal: bool) {
        self.modal = modal;
        self.start_increment = start_increment;
        if self.kind.plays_increments() {
            self.frame = increments
                .iter()
                .position(|&i| i == start_increment)
                .unwrap_or(0);
        }
        self.increments = increments;
    }

    pub fn frame_count(&self) -> usize {
        match self.kind {
            AnimationKind::ScaleFactor => self.frames.max(2) as usize,
            AnimationKind::Increments | AnimationKind::Steps => self.increments.len().max(1),
        }
    }

    /// Position of the shown frame of a scale factor animation, from 0 at the first frame to
    /// 1 at the last.
    pub fn progress(&self) -> f32 {
        self.frame.min(self.frame_count() - 1) as f32 / (self.frame_count() - 1) as f32
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
            AnimationKind::Increments | AnimationKind::Steps => 1.0,
        }
    }

    /// Increment of the shown frame, for increment animations.
    pub fn increment(&self) -> Option<usize> {
        match self.kind {
            AnimationKind::ScaleFactor => None,
            AnimationKind::Increments | AnimationKind::Steps => {
                self.increments.get(self.frame).copied()
            }
        }
    }

    /// Jumps to a frame; returns true when the shown frame changed.
    pub fn go_to(&mut self, frame: usize) -> bool {
        let frame = frame.min(self.frame_count() - 1);
        let changed = frame != self.frame;
        self.frame = frame;
        changed
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
        let mut animation = Animation::new(AnimationKind::ScaleFactor, Vec::new(), 0, false);
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
        let animation = Animation::new(AnimationKind::Increments, vec![3, 4, 5], 4, false);
        assert_eq!(animation.frame, 1);
        assert_eq!(animation.increment(), Some(4));
        assert_eq!(animation.amplitude(), 1.0);
    }
}
