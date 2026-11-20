//! A running solve in the status bar, for when the Analysis tab with its monitor is out of
//! view: a spinner, the analysis, how long it runs and where the solver is.

/// What the status bar tells about a running solve.
pub struct RunStatus {
    pub name: String,
    /// What it does now, as the monitor says: "running", "meshing", ...
    pub state: String,
    pub seconds: f32,
    /// The monitor's line about the solver, "Step 1, increment 5, iteration 1, ...".
    pub progress: Option<String>,
}

impl RunStatus {
    /// The short line: "Analysis-1: running 8:32 min, Step 1, increment 5".
    pub fn line(&self) -> String {
        let mut line = format!("{}: {} {}", self.name, self.state, duration(self.seconds));
        if let Some(rough) = self.progress.as_deref().and_then(rough) {
            line.push_str(" | ");
            line.push_str(&rough);
        }
        line
    }

    /// The whole line, as the monitor shows it.
    pub fn full(&self) -> String {
        let mut line = format!("{}: {} ({:.1} s)", self.name, self.state, self.seconds);
        if let Some(progress) = &self.progress {
            line.push_str(", ");
            line.push_str(progress);
        }
        line
    }
}

/// Shows the spinner and the short line; returns whether it was clicked.
pub fn show(ui: &mut egui::Ui, status: &RunStatus) -> bool {
    let response = ui
        .horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            ui.spinner();
            ui.add(egui::Label::new(status.line()).sense(egui::Sense::click()))
        })
        .inner;
    // The seconds go on while nothing else moves.
    ui.ctx()
        .request_repaint_after(std::time::Duration::from_millis(500));
    response
        .on_hover_text(format!("{}\nClick to show the Analysis tab", status.full()))
        .clicked()
}

/// "12 s" under a minute, then "8:32 min" and "1:02:05 h".
pub fn duration(seconds: f32) -> String {
    let total = seconds.max(0.0) as u64;
    let (h, m, s) = (total / 3600, total / 60 % 60, total % 60);
    if h > 0 {
        format!("{h}:{m:02}:{s:02} h")
    } else if m > 0 {
        format!("{m}:{s:02} min")
    } else {
        format!("{s} s")
    }
}

/// The step and increment of the solver's line, without the attempt, iteration, size and
/// time it goes on with; `None` if it tells neither.
pub fn rough(progress: &str) -> Option<String> {
    let parts: Vec<&str> = (progress.split(", "))
        .map(str::trim)
        .filter(|part| {
            let lower = part.to_lowercase();
            lower.starts_with("step ") && !lower.starts_with("step time")
                || lower.starts_with("increment ")
        })
        .collect();
    (!parts.is_empty()).then(|| parts.join(", "))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn durations_read_as_seconds_minutes_and_hours() {
        assert_eq!(duration(12.7), "12 s");
        assert_eq!(duration(512.1), "8:32 min");
        assert_eq!(duration(3725.0), "1:02:05 h");
    }

    #[test]
    fn the_short_line_keeps_step_and_increment() {
        let status = RunStatus {
            name: "Analysis-1".into(),
            state: "running".into(),
            seconds: 512.1,
            progress: Some(
                "Step 1, increment 5, iteration 1, size 0.021094, step time 0.149219".into(),
            ),
        };
        assert_eq!(
            status.line(),
            "Analysis-1: running 8:32 min | Step 1, increment 5"
        );
        assert_eq!(
            rough("Step 2, increment 3, iterations 4, time 1.5"),
            Some("Step 2, increment 3".into())
        );
        assert_eq!(rough("Solving DP 3 …"), None);
    }
}
