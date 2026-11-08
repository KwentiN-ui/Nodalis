//! Result frames evenly spaced over a step (`FRAMES=` on `*STATIC`, `*VISCO` and the heat
//! transfer procedures): automatic or fixed increments are shortened so that they end on
//! the frames, and only the increments that end on one are written.

use plxs_core::step::Incrementation;

/// The frames of a step and the next one still to reach.
#[derive(Clone, Debug)]
pub struct Frames {
    period: f64,
    count: u32,
    /// The next frame (1-based).
    next: u32,
}

impl Frames {
    /// The frames of `increments` over `period`; `None` without `FRAMES=`.
    pub fn of(increments: &Incrementation, period: f64) -> Option<Self> {
        let count = increments.frames?.max(1);
        Some(Self {
            period,
            count,
            next: 1,
        })
    }

    fn tolerance(&self) -> f64 {
        1e-9 * self.period
    }

    /// The step time of the next frame.
    pub fn next_time(&self) -> f64 {
        Incrementation::frame_time(self.period, self.count, self.next)
    }

    /// The size of an increment of planned size `dt` from `time`: no further than the next
    /// frame. Within a millionth of the step of it, the increment ends on it rather than
    /// leaving a sliver.
    pub fn limit(&self, time: f64, dt: f64) -> f64 {
        let left = self.next_time() - time;
        if left <= 0.0 {
            return dt;
        }
        if dt >= left - 1e-6 * self.period {
            left
        } else {
            dt
        }
    }

    /// Whether an increment ending at `time` reached the next frame; moves on to the frame
    /// after it.
    pub fn reached(&mut self, time: f64) -> bool {
        let mut reached = false;
        while self.next <= self.count && time >= self.next_time() - self.tolerance() {
            self.next += 1;
            reached = true;
        }
        reached
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frames(count: u32) -> Frames {
        let increments = Incrementation {
            frames: Some(count),
            ..Incrementation::default()
        };
        Frames::of(&increments, 2.0).unwrap()
    }

    #[test]
    fn increments_are_shortened_to_end_on_the_frames() {
        let mut f = frames(4);
        // Frames at 0.5, 1, 1.5, 2.
        assert_eq!(f.limit(0.0, 0.2), 0.2);
        assert_eq!(f.limit(0.4, 0.2), 0.5 - 0.4);
        assert!(!f.reached(0.4));
        assert!(f.reached(0.5));
        assert_eq!(f.next_time(), 1.0);
        // An increment over several frames stops at the first one.
        assert_eq!(f.limit(0.5, 5.0), 0.5);
        // Almost on the frame: ends on it.
        assert_eq!(f.limit(0.5, 0.5 - 1e-8), 0.5);
        assert!(f.reached(1.0));
        assert!(f.reached(2.0));
        assert!(!f.reached(2.0));
    }

    #[test]
    fn without_frames_nothing_is_limited() {
        assert!(Frames::of(&Incrementation::default(), 1.0).is_none());
    }
}
