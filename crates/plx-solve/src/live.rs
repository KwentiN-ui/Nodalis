//! Where a run is, read from CalculiX's output as it comes in. The `.sta` file only gets a
//! line once an increment has converged, which can take minutes in a hard nonlinear step;
//! the output tells about every attempt and iteration.

/// The latest step, increment, attempt and iteration in the output so far.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LiveProgress {
    /// From the `STEP n` header, written by CalculiX's Fortran part.
    pub step: Option<u32>,
    /// From `increment n attempt m`; cleared by a new step.
    pub increment: Option<u32>,
    pub attempt: Option<u32>,
    /// From `iteration n`; cleared by a new increment or attempt.
    pub iteration: Option<u32>,
    /// From `increment size=`, shown as "size".
    pub increment_size: Option<f64>,
    pub step_time: Option<f64>,
}

impl LiveProgress {
    /// Takes one line of CalculiX's output into account.
    pub fn read(&mut self, line: &str) {
        let line = line.trim();
        let number = |key: &str| -> Option<f64> { line.strip_prefix(key)?.trim().parse().ok() };
        if let Some(step) = line
            .strip_prefix("STEP")
            .and_then(|s| s.trim().parse().ok())
        {
            *self = Self {
                step: Some(step),
                ..Self::default()
            };
        } else if let Some((increment, attempt)) = increment_attempt(line) {
            self.increment = Some(increment);
            self.attempt = Some(attempt);
            (self.iteration, self.increment_size, self.step_time) = (None, None, None);
        } else if let Some(size) = number("increment size=") {
            self.increment_size = Some(size);
        } else if let Some(time) = number("actual step time=") {
            self.step_time = Some(time);
        } else if let Some(iteration) = line.strip_prefix("iteration ") {
            // Not the solver's "- iteration 1 :" lines, which do not start with the word.
            if let Ok(iteration) = iteration.trim().parse() {
                self.iteration = Some(iteration);
            }
        }
    }

    /// One line for the monitor, or `None` before the first step.
    pub fn text(&self) -> Option<String> {
        let mut parts = Vec::new();
        if let Some(step) = self.step {
            parts.push(format!("Step {step}"));
        }
        if let Some(increment) = self.increment {
            parts.push(format!("increment {increment}"));
        }
        if let Some(attempt) = self.attempt.filter(|&a| a > 1) {
            parts.push(format!("attempt {attempt}"));
        }
        if let Some(iteration) = self.iteration {
            parts.push(format!("iteration {iteration}"));
        }
        if let Some(size) = self.increment_size {
            parts.push(format!("size {}", short(size)));
        }
        if let Some(time) = self.step_time {
            parts.push(format!("step time {}", short(time)));
        }
        let mut text = parts.join(", ");
        if let Some(first) = text.get(..1) {
            let upper = first.to_uppercase();
            text.replace_range(..1, &upper);
        }
        (!text.is_empty()).then_some(text)
    }
}

/// `increment 3 attempt 2`, as CalculiX starts each try of an increment.
fn increment_attempt(line: &str) -> Option<(u32, u32)> {
    let rest = line.strip_prefix("increment ")?;
    let (increment, attempt) = rest.split_once(" attempt ")?;
    Some((increment.trim().parse().ok()?, attempt.trim().parse().ok()?))
}

/// A time or size as CalculiX prints it, without its trailing zeros: 0.0625, 1.5e-7.
fn short(value: f64) -> String {
    if value != 0.0 && !(1e-4..1e5).contains(&value.abs()) {
        format!("{value:.3e}")
    } else {
        format!("{:.6}", value)
            .trim_end_matches('0')
            .trim_end_matches('.')
            .to_owned()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read(text: &str) -> LiveProgress {
        let mut live = LiveProgress::default();
        text.lines().for_each(|line| live.read(line));
        live
    }

    #[test]
    fn follows_steps_increments_attempts_and_iterations() {
        assert_eq!(read("CalculiX Version 2.23\n").text(), None);
        assert_eq!(read(" STEP            2\n").text(), Some("Step 2".into()));
        let output = " STEP            1
 Static analysis was selected
 increment 1 attempt 1 
 increment size= 1.000000e+00
 sum of previous increments=0.000000e+00
 actual step time=1.000000e+00
 actual total time=1.000000e+00

 iteration 1
    - iteration 1 :
 largest residual force= 37.045449 in node 155636 and dof 3
 iteration 2
 divergence; the increment size is decreased to 6.250000e-02
 the increment is reattempted

 increment 1 attempt 3
 increment size= 6.250000e-02
 actual step time=6.250000e-02
";
        let live = read(output);
        assert_eq!(
            (live.increment, live.attempt, live.iteration),
            (Some(1), Some(3), None)
        );
        assert_eq!(
            live.text().unwrap(),
            "Step 1, increment 1, attempt 3, size 0.0625, step time 0.0625"
        );
        let live = read(&format!("{output} iteration 1\n"));
        assert_eq!(
            live.text().unwrap(),
            "Step 1, increment 1, attempt 3, iteration 1, size 0.0625, step time 0.0625"
        );
        // A new step starts over.
        let live = read(&format!("{output} STEP 2\n"));
        assert_eq!(live.text().unwrap(), "Step 2");
        // Without the Fortran header the increments still tell.
        assert_eq!(
            read(" increment 4 attempt 1\n iteration 2\n")
                .text()
                .unwrap(),
            "Increment 4, iteration 2"
        );
    }

    #[test]
    fn short_numbers() {
        assert_eq!(short(1.0), "1");
        assert_eq!(short(0.0625), "0.0625");
        assert_eq!(short(0.0), "0");
        assert_eq!(short(1e-7), "1.000e-7");
    }
}
