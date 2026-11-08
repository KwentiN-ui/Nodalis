//! Where the time of a solve went: the solver measures its phases on every run, and the
//! monitor shows them as a table beside the output once the run is done. "Copy Report"
//! puts the table with the version, the system and the size of the model on the clipboard
//! as text, ready to paste into an issue about a slow model.

use plx_solve::{JobProfile, Phase};

/// A row of the table: a phase with its time, share of the total and how often it ran.
#[derive(Clone, Debug, PartialEq)]
pub struct Row {
    pub name: &'static str,
    pub seconds: f64,
    /// Share of the solver's total time, 0 to 1.
    pub share: f64,
    /// How often the phase ran; `None` for the rows that are no phase of their own.
    pub count: Option<u32>,
}

/// The phases that took time, the longest first, then writing the results and the time no
/// phase measured.
pub fn rows(profile: &JobProfile) -> Vec<Row> {
    let total = profile.total.as_secs_f64();
    let share = |seconds: f64| if total > 0.0 { seconds / total } else { 0.0 };
    let phases = &profile.phases;
    let mut rows: Vec<Row> = (Phase::ALL.iter())
        .filter(|&&phase| phases.count(phase) > 0)
        .map(|&phase| {
            let seconds = phases.time(phase).as_secs_f64();
            Row {
                name: phase.name(),
                seconds,
                share: share(seconds),
                count: Some(phases.count(phase)),
            }
        })
        .collect();
    rows.sort_by(|a, b| b.seconds.total_cmp(&a.seconds));
    let measured: f64 = rows.iter().map(|r| r.seconds).sum();
    let writing = profile.writing.as_secs_f64();
    rows.push(Row {
        name: "writing results",
        seconds: writing,
        share: share(writing),
        count: None,
    });
    let other = (total - measured - writing).max(0.0);
    rows.push(Row {
        name: "other",
        seconds: other,
        share: share(other),
        count: None,
    });
    rows
}

/// Whether the monitors show the profile beside the output; one switch for all of them,
/// kept with the window layout.
pub fn shown(ctx: &egui::Context) -> bool {
    ctx.data_mut(|d| *d.get_persisted_mut_or(shown_id(), true))
}

fn shown_id() -> egui::Id {
    egui::Id::new("solver profile shown")
}

/// What the report tells about the run besides its profile.
pub struct ReportInfo<'a> {
    /// Name of the analysis.
    pub name: &'a str,
    /// How the run ended.
    pub state: &'a str,
    /// Wall-clock time of the whole solve in Nodalis (meshing and preparing included).
    pub seconds: f32,
    /// The steps solved, one line each.
    pub steps: &'a [String],
}

/// The buttons of a monitor's row: Profile switches the table on and off, Copy Report
/// copies the report of the run. Both need a finished run with a profile.
pub fn buttons(ui: &mut egui::Ui, profile: Option<&JobProfile>, info: &ReportInfo<'_>) {
    let ctx = ui.ctx().clone();
    let mut on = shown(&ctx);
    if ui
        .add_enabled(profile.is_some(), egui::Button::selectable(on, "Profile"))
        .on_hover_text("Shows where the solver spent its time, per phase")
        .on_disabled_hover_text("The profile shows once the run has finished")
        .clicked()
    {
        on = !on;
        ctx.data_mut(|d| d.insert_persisted(shown_id(), on));
    }
    if let Some(profile) = profile {
        // The graphics cards are detected off the user interface; started here, they are
        // known by the time the button is pressed.
        let gpu = crate::settings::gpu_detection(&ctx);
        // The button tells for a moment that it copied.
        let copied_id = egui::Id::new("solver report copied");
        let now = ctx.input(|i| i.time);
        let copied_at = ctx.data(|d| d.get_temp::<f64>(copied_id));
        let recent = copied_at.is_some_and(|t| now - t < 1.5);
        if recent {
            ctx.request_repaint_after(std::time::Duration::from_millis(300));
        }
        let label = if recent { "Copied" } else { "Copy Report" };
        if ui
            .button(label)
            .on_hover_text(
                "Copies the version, the system, the size of the model and the profile of \
                 the run as text, e.g. for an issue on GitHub",
            )
            .clicked()
        {
            ctx.copy_text(report(profile, info, gpu));
            ctx.data_mut(|d| d.insert_temp(copied_id, now));
        }
    } else {
        ui.add_enabled(false, egui::Button::new("Copy Report"));
    }
}

/// The output of a monitor, with the profile table to its left when there is one and it is
/// switched on.
pub fn with_output(
    ui: &mut egui::Ui,
    profile: Option<&JobProfile>,
    output: impl FnOnce(&mut egui::Ui),
) {
    let Some(profile) = profile.filter(|_| shown(ui.ctx())) else {
        output(ui);
        return;
    };
    ui.horizontal_top(|ui| {
        let height = ui.available_height();
        ui.allocate_ui_with_layout(
            egui::vec2(340.0, height),
            egui::Layout::top_down(egui::Align::Min),
            |ui| {
                egui::ScrollArea::vertical()
                    .id_salt("solver profile")
                    .auto_shrink([false, false])
                    .show(ui, |ui| table(ui, profile));
            },
        );
        ui.separator();
        ui.vertical(output);
    });
}

/// The table of the phases with a bar per share.
pub fn table(ui: &mut egui::Ui, profile: &JobProfile) {
    ui.strong(format!(
        "Solver time {}",
        seconds_text(profile.total.as_secs_f64())
    ));
    ui.weak(format!(
        "{} nodes, {} elements, {} DOFs, {} threads",
        thousands(profile.nodes),
        thousands(profile.elements),
        thousands(profile.degrees_of_freedom),
        profile.threads
    ));
    let bar_color = ui.visuals().selection.bg_fill;
    egui::Grid::new("solver profile rows")
        .num_columns(4)
        .striped(true)
        .spacing([10.0, 3.0])
        .show(ui, |ui| {
            for title in ["Phase", "Time", "Share", "Calls"] {
                ui.strong(title);
            }
            ui.end_row();
            for row in rows(profile) {
                ui.label(capitalized(row.name));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(seconds_text(row.seconds));
                });
                share_bar(ui, row.share, bar_color);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(row.count.map_or_else(String::new, thousands_u32));
                });
                ui.end_row();
            }
        });
    if profile.phases.analyses_reused > 0 {
        ui.weak(format!(
            "Symbolic analysis reused {} times",
            profile.phases.analyses_reused
        ));
    }
}

/// A bar as long as the share, with the percentage on it.
fn share_bar(ui: &mut egui::Ui, share: f64, color: egui::Color32) {
    let size = egui::vec2(90.0, ui.spacing().interact_size.y * 0.8);
    let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
    let painter = ui.painter();
    painter.rect_filled(rect, 2.0, ui.visuals().extreme_bg_color);
    let mut filled = rect;
    filled.set_width(rect.width() * share.clamp(0.0, 1.0) as f32);
    painter.rect_filled(filled, 2.0, color);
    painter.text(
        rect.right_center() - egui::vec2(4.0, 0.0),
        egui::Align2::RIGHT_CENTER,
        format!("{:.1} %", share * 100.0),
        egui::TextStyle::Small.resolve(ui.style()),
        ui.visuals().strong_text_color(),
    );
}

/// The report Copy Report puts on the clipboard: plain text that reads well in an issue as
/// it is (a code block keeps the columns).
pub fn report(
    profile: &JobProfile,
    info: &ReportInfo<'_>,
    gpu: Option<&plxs_linalg::gpu::Detection>,
) -> String {
    let mut lines = vec![
        "```".to_string(),
        format!(
            "Nodalis {} (solver plxs {})",
            env!("CARGO_PKG_VERSION"),
            plx_solve::VERSION
        ),
        format!(
            "System: {} {}, {} logical cores, solver threads {}",
            std::env::consts::OS,
            std::env::consts::ARCH,
            std::thread::available_parallelism().map_or(0, |n| n.get()),
            profile.threads
        ),
        format!("Graphics card for the solver: {}", gpu_text(gpu)),
        format!(
            "Analysis: {}, {} after {:.1} s (solver {})",
            info.name,
            info.state,
            info.seconds,
            seconds_text(profile.total.as_secs_f64())
        ),
        format!(
            "Model as solved: {} nodes, {} elements, {} degrees of freedom",
            profile.nodes, profile.elements, profile.degrees_of_freedom
        ),
    ];
    if !info.steps.is_empty() {
        lines.push("Steps:".into());
        lines.extend(info.steps.iter().map(|s| format!("  {s}")));
    }
    lines.push("Profile (wall clock):".into());
    lines.push(format!(
        "  {:<24}{:>12}{:>9}{:>10}",
        "phase", "time", "share", "calls"
    ));
    for row in rows(profile) {
        lines.push(format!(
            "  {:<24}{:>12}{:>8.1}%{:>10}",
            row.name,
            format!("{:.3} s", row.seconds),
            row.share * 100.0,
            row.count.map_or_else(String::new, |c| c.to_string())
        ));
    }
    lines.push(format!(
        "  {:<24}{:>12}",
        "total",
        format!("{:.3} s", profile.total.as_secs_f64())
    ));
    if profile.phases.analyses_reused > 0 {
        lines.push(format!(
            "  (symbolic analysis reused {} times)",
            profile.phases.analyses_reused
        ));
    }
    lines.push("```".into());
    lines.join("\n") + "\n"
}

/// The graphics card the solver would use and whether it may.
fn gpu_text(gpu: Option<&plxs_linalg::gpu::Detection>) -> String {
    let mode = match plxs_linalg::amg::GpuMode::from_environment() {
        plxs_linalg::amg::GpuMode::Off => "off",
        plxs_linalg::amg::GpuMode::Automatic => "automatic",
        plxs_linalg::amg::GpuMode::On => "on",
    };
    let card = match gpu {
        None => "not detected yet".to_string(),
        Some(found) => match &found.chosen {
            Some(card) => {
                let precision = if found.double_precision {
                    "double precision"
                } else {
                    "no double precision"
                };
                format!(
                    "{card}, {}, {precision}",
                    found.backend.as_deref().unwrap_or("unknown API")
                )
            }
            None if found.adapters.is_empty() => "none found".into(),
            None => format!("no discrete card (found {})", found.adapters.join(", ")),
        },
    };
    format!("{card}; use: {mode}")
}

/// A step for the report: its name, kind and the settings that change the cost most.
pub fn step_line(step: &plx_model::Step) -> String {
    use plx_model::StepKind as K;
    let kind = match &step.kind {
        K::Static(_) => "static".to_string(),
        K::Frequency(_) => "frequency".into(),
        K::ComplexFrequency(_) => "complex frequency".into(),
        K::Buckle(_) => "buckle".into(),
        K::HeatTransfer(_) => "heat transfer".into(),
        K::CoupledTempDisp(_) => "coupled temperature-displacement".into(),
        K::UncoupledTempDisp(_) => "uncoupled temperature-displacement".into(),
        K::Dynamic(settings) => format!("dynamic, {}", settings.method.label()),
        K::Visco(_) => "visco".into(),
        K::ModalDynamics(_) => "modal dynamics".into(),
        K::SteadyStateDynamics(_) => "steady state dynamics".into(),
        K::Sensitivity(_) => "sensitivity".into(),
    };
    let nlgeom = if step.kind.nlgeom() { ", NLGEOM" } else { "" };
    format!("{} ({kind}{nlgeom})", step.name)
}

fn seconds_text(seconds: f64) -> String {
    if seconds >= 100.0 {
        format!("{seconds:.0} s")
    } else if seconds >= 1.0 {
        format!("{seconds:.2} s")
    } else {
        format!("{:.1} ms", seconds * 1000.0)
    }
}

fn capitalized(name: &str) -> String {
    let mut chars = name.chars();
    chars.next().map_or_else(String::new, |first| {
        first.to_uppercase().chain(chars).collect()
    })
}

fn thousands_u32(n: u32) -> String {
    thousands(n as usize)
}

/// `1234567` as `1 234 567`.
fn thousands(n: usize) -> String {
    let digits = n.to_string();
    let mut out = String::new();
    for (i, c) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i).is_multiple_of(3) {
            out.push(' ');
        }
        out.push(c);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    fn profile() -> JobProfile {
        let mut phases = plxs_procedures_profile();
        phases.analyses_reused = 3;
        JobProfile {
            phases,
            total: Duration::from_secs(10),
            writing: Duration::from_secs(1),
            nodes: 1200,
            elements: 800,
            degrees_of_freedom: 3600,
            threads: 4,
        }
    }

    /// A profile with time in assembly and factorization; the clock is real, so the times
    /// are small and only their order is checked.
    fn plxs_procedures_profile() -> plx_solve::Profile {
        let mut phases = plx_solve::Profile::default();
        let started = Instant::now();
        std::thread::sleep(Duration::from_millis(4));
        phases.add(Phase::Factorization, started);
        let started = Instant::now();
        std::thread::sleep(Duration::from_millis(1));
        phases.add(Phase::Assembly, started);
        phases
    }

    #[test]
    fn the_rows_hold_the_phases_that_ran_longest_first() {
        let rows = rows(&profile());
        let names: Vec<&str> = rows.iter().map(|r| r.name).collect();
        assert_eq!(
            names,
            ["factorization", "assembly", "writing results", "other"]
        );
        let total: f64 = rows.iter().map(|r| r.share).sum();
        assert!((total - 1.0).abs() < 1e-9, "shares add up to {total}");
        assert_eq!(rows[2].seconds, 1.0);
    }

    #[test]
    fn the_report_tells_version_system_model_and_phases() {
        let steps = ["Step-1 (static, NLGEOM)".to_string()];
        let info = ReportInfo {
            name: "Bracket",
            state: "solved",
            seconds: 12.5,
            steps: &steps,
        };
        let text = report(&profile(), &info, None);
        for part in [
            env!("CARGO_PKG_VERSION"),
            std::env::consts::OS,
            "Analysis: Bracket, solved after 12.5 s",
            "1200 nodes, 800 elements, 3600 degrees of freedom",
            "Step-1 (static, NLGEOM)",
            "factorization",
            "writing results",
            "10.000 s",
            "reused 3 times",
        ] {
            assert!(text.contains(part), "{part} missing in\n{text}");
        }
        assert!(text.is_ascii(), "the report stays plain text");
    }

    #[test]
    fn large_numbers_get_spaces() {
        assert_eq!(thousands(1_234_567), "1 234 567");
        assert_eq!(thousands(999), "999");
    }
}
