//! What the status bar tells of the output while its tab is not shown: the number of new
//! lines on the Output tab, and for a few seconds the newest line next to the tabs.

/// How long the newest line stays next to the tabs.
pub const SECONDS: f64 = 8.0;

#[derive(Default)]
pub struct OutputNews {
    /// The lines read: the length of the output when its tab was shown last.
    read: usize,
    /// The length of the output seen last, and when it grew.
    seen: usize,
    since: Option<f64>,
}

impl OutputNews {
    /// Follows `lines`; `shown` tells whether the Output tab is in view, so that its lines
    /// are read. Returns the number of new lines and the newest one while it is new.
    pub fn update<'a>(
        &mut self,
        ctx: &egui::Context,
        lines: &'a [String],
        shown: bool,
    ) -> (usize, Option<&'a str>) {
        let now = ctx.input(|i| i.time);
        self.step(now, lines.len(), shown);
        let fresh = self.since.map(|since| SECONDS - (now - since));
        match fresh {
            Some(left) if left > 0.0 => {
                ctx.request_repaint_after(std::time::Duration::from_secs_f64(left));
                (self.unread(), lines.last().map(String::as_str))
            }
            _ => {
                self.since = None;
                (self.unread(), None)
            }
        }
    }

    fn step(&mut self, now: f64, len: usize, shown: bool) {
        // A cleared output starts again.
        if len < self.seen || len < self.read {
            self.read = self.read.min(len);
            self.since = None;
        }
        if len > self.seen && !shown {
            self.since = Some(now);
        }
        self.seen = len;
        if shown {
            self.read = len;
            self.since = None;
        }
    }

    fn unread(&self) -> usize {
        self.seen - self.read
    }

    /// The title of the Output tab.
    pub fn title(unread: usize) -> String {
        if unread == 0 {
            "Output".to_owned()
        } else {
            format!("Output ({unread} new)")
        }
    }

    /// Shows the newest line, cut to what fits a status bar; returns whether it was clicked.
    pub fn line(ui: &mut egui::Ui, line: &str) -> bool {
        let first = line.lines().next().unwrap_or_default();
        let text = match first.char_indices().nth(90) {
            Some((end, _)) => format!("{} ...", &first[..end]),
            None => first.to_owned(),
        };
        ui.add(egui::Label::new(egui::RichText::new(text).weak()).sense(egui::Sense::click()))
            .on_hover_text("Shows the output")
            .clicked()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lines_that_come_while_the_output_is_hidden_are_new_until_it_is_shown() {
        let mut news = OutputNews::default();
        news.step(0.0, 2, false);
        assert_eq!(news.unread(), 2);
        assert_eq!(news.since, Some(0.0));
        news.step(1.0, 3, false);
        assert_eq!((news.unread(), news.since), (3, Some(1.0)));
        news.step(2.0, 3, true);
        assert_eq!((news.unread(), news.since), (0, None));
        // Lines that come while the output is in view are read at once.
        news.step(3.0, 5, true);
        assert_eq!((news.unread(), news.since), (0, None));
        news.step(4.0, 5, false);
        assert_eq!((news.unread(), news.since), (0, None));
        // A cleared output starts again.
        news.step(5.0, 1, false);
        assert_eq!(news.unread(), 0);
        news.step(6.0, 2, false);
        assert_eq!((news.unread(), news.since), (1, Some(6.0)));
        assert_eq!(OutputNews::title(1), "Output (1 new)");
        assert_eq!(OutputNews::title(0), "Output");
    }
}
