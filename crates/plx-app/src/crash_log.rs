//! The log folder: `nodalis.log` with the messages of the running session (the one before it
//! is kept as `nodalis.previous.log`) and one `crash-<time>.log` per panic, with the
//! message, where it happened, the backtrace and the version. The Windows build has no
//! console, so without these files a crash would leave no trace. Help > Open Log Folder
//! shows the folder.
//!
//! The folder is `logs` in the data directory: Linux `~/.local/share/nodalis/logs`,
//! Windows `%APPDATA%\nodalis\data\logs`, the portable ZIP `Settings\logs` next to the
//! program.

use egui_wgpu::wgpu;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// The log of the running session.
const SESSION_LOG: &str = "nodalis.log";
/// The log of the session before, kept so a crash of the last run can still be looked at
/// after a restart.
const PREVIOUS_LOG: &str = "nodalis.previous.log";
/// How many crash reports are kept; older ones are deleted at the start.
const KEPT_REPORTS: usize = 20;
/// How many graphics errors of one session go into the log, so a broken frame repeated at
/// 60 frames per second does not fill the disk.
const LOGGED_GPU_ERRORS: usize = 20;

/// Set by the graphics driver thread when the GPU device was lost; the window reads and
/// clears it to tell the user.
static DEVICE_LOST: AtomicBool = AtomicBool::new(false);
static GPU_ERRORS: AtomicUsize = AtomicUsize::new(0);

/// The folder of the log files.
pub fn log_dir() -> Option<PathBuf> {
    crate::portable::data_dir().map(|dir| dir.join("logs"))
}

/// Opens the log folder in the file manager; returns the line for the output.
pub fn open_log_dir() -> String {
    let Some(dir) = log_dir() else {
        return "No log folder: the data directory of this system is unknown".into();
    };
    if let Err(err) = std::fs::create_dir_all(&dir) {
        return format!("Could not create the log folder {}: {err}", dir.display());
    }
    match open::that_detached(&dir) {
        Ok(()) => format!("Opened the log folder {}", dir.display()),
        Err(err) => format!("Could not open the log folder {}: {err}", dir.display()),
    }
}

/// Sets up the session log and the panic hook. Called first thing in `main`; without a
/// data directory the messages only go to stderr, as before.
pub fn install() {
    let dir = log_dir();
    let file = dir.as_deref().and_then(|dir| open_session_log(dir).ok());
    let mut builder =
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"));
    if let Some(file) = file {
        builder.target(env_logger::Target::Pipe(Box::new(Tee(Mutex::new(file)))));
    }
    builder.init();
    log::info!("{}", system_line());
    if let Some(dir) = &dir {
        prune_reports(dir, KEPT_REPORTS);
    }
    let previous = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let thread = std::thread::current();
        let thread = thread.name().unwrap_or("unnamed").to_owned();
        let message = panic_message(info.payload());
        let location = info
            .location()
            .map_or_else(|| "unknown".to_owned(), ToString::to_string);
        let backtrace = std::backtrace::Backtrace::force_capture().to_string();
        let report = report(&thread, &message, &location, &backtrace);
        let written = dir
            .as_deref()
            .and_then(|dir| write_report(dir, &report).ok());
        log::error!("Panic in thread '{thread}' at {location}: {message}");
        previous(info);
        // A panic of the main thread ends the program; the Windows build has no console,
        // so a message box says where the report went.
        #[cfg(windows)]
        if thread == "main" {
            crash_dialog(&message, written.as_deref());
        }
        #[cfg(not(windows))]
        let _ = written;
    }));
}

/// Makes wgpu log its errors instead of ending the program: the default handler panics on
/// every validation error, and a broken frame is better than a lost project. A lost device
/// (driver reset, GPU switched off while the window was in the background) is logged and
/// reported through [`take_device_lost`].
pub fn watch_gpu(device: &wgpu::Device) {
    device.on_uncaptured_error(std::sync::Arc::new(|err: wgpu::Error| {
        let count = GPU_ERRORS.fetch_add(1, Ordering::Relaxed);
        if count < LOGGED_GPU_ERRORS {
            let backtrace = std::backtrace::Backtrace::force_capture();
            log::error!("Graphics error: {err}\n{backtrace}");
        } else if count == LOGGED_GPU_ERRORS {
            log::error!("Further graphics errors of this session are not logged");
        }
    }));
    device.set_device_lost_callback(|reason, message| {
        log::error!("Graphics device lost ({reason:?}): {message}");
        if reason != wgpu::DeviceLostReason::Destroyed {
            DEVICE_LOST.store(true, Ordering::Relaxed);
        }
    });
}

/// Whether the GPU device was lost since the last call.
pub fn take_device_lost() -> bool {
    DEVICE_LOST.swap(false, Ordering::Relaxed)
}

/// Starts a new session log; the one before becomes [`PREVIOUS_LOG`].
fn open_session_log(dir: &Path) -> std::io::Result<std::fs::File> {
    std::fs::create_dir_all(dir)?;
    let log = dir.join(SESSION_LOG);
    if log.is_file() {
        std::fs::rename(&log, dir.join(PREVIOUS_LOG))?;
    }
    std::fs::File::create(log)
}

/// Writes to stderr and the session log, so a terminal still shows the messages.
struct Tee(Mutex<std::fs::File>);

impl Write for Tee {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let _ = std::io::stderr().write_all(buf);
        let mut file = self
            .0
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        file.write_all(buf)?;
        file.flush()?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        let mut file = self
            .0
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        file.flush()
    }
}

fn system_line() -> String {
    format!(
        "Nodalis {} ({} {}, {} build)",
        env!("CARGO_PKG_VERSION"),
        std::env::consts::OS,
        std::env::consts::ARCH,
        if cfg!(debug_assertions) {
            "debug"
        } else {
            "release"
        }
    )
}

fn panic_message(payload: &(dyn std::any::Any + Send)) -> String {
    if let Some(text) = payload.downcast_ref::<&str>() {
        (*text).to_owned()
    } else if let Some(text) = payload.downcast_ref::<String>() {
        text.clone()
    } else {
        "(no message)".to_owned()
    }
}

fn report(thread: &str, message: &str, location: &str, backtrace: &str) -> String {
    format!(
        "Nodalis crash report\n\
         Time: {}\n\
         Version: {}\n\
         Thread: {thread}\n\
         Location: {location}\n\
         Message: {message}\n\
         \n\
         Backtrace:\n{backtrace}\n",
        chrono::Local::now().format("%Y-%m-%d %H:%M:%S %z"),
        system_line(),
    )
}

/// Writes a crash report; returns its path.
fn write_report(dir: &Path, report: &str) -> std::io::Result<PathBuf> {
    std::fs::create_dir_all(dir)?;
    let stamp = chrono::Local::now().format("%Y-%m-%d_%H-%M-%S%.3f");
    let path = dir.join(format!("crash-{stamp}.log"));
    std::fs::write(&path, report)?;
    Ok(path)
}

/// Deletes all but the newest `keep` crash reports. The names sort by time.
fn prune_reports(dir: &Path, keep: usize) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    let mut reports: Vec<PathBuf> = entries
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with("crash-") && n.ends_with(".log"))
        })
        .collect();
    reports.sort();
    let old = reports.len().saturating_sub(keep);
    for path in &reports[..old] {
        let _ = std::fs::remove_file(path);
    }
}

#[cfg(windows)]
fn crash_dialog(message: &str, report: Option<&Path>) {
    let place = report.map_or_else(
        || "No crash report could be written.".to_owned(),
        |path| format!("A crash report was written to\n{}", path.display()),
    );
    let _ = rfd::MessageDialog::new()
        .set_level(rfd::MessageLevel::Error)
        .set_title("Nodalis crashed")
        .set_description(format!("Nodalis has to close:\n{message}\n\n{place}"))
        .set_buttons(rfd::MessageButtons::Ok)
        .show();
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("nodalis-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn report_names_message_place_version_and_backtrace() {
        let text = report("main", "index out of bounds", "src/app.rs:12:5", "0: main");
        assert!(text.contains("Thread: main"));
        assert!(text.contains("Location: src/app.rs:12:5"));
        assert!(text.contains("Message: index out of bounds"));
        assert!(text.contains(env!("CARGO_PKG_VERSION")));
        assert!(text.contains("Backtrace:\n0: main"));
    }

    #[test]
    fn panic_message_reads_both_payload_kinds() {
        let text: Box<dyn std::any::Any + Send> = Box::new("static");
        assert_eq!(panic_message(text.as_ref()), "static");
        let owned: Box<dyn std::any::Any + Send> = Box::new(String::from("owned"));
        assert_eq!(panic_message(owned.as_ref()), "owned");
        let other: Box<dyn std::any::Any + Send> = Box::new(3);
        assert_eq!(panic_message(other.as_ref()), "(no message)");
    }

    #[test]
    fn only_the_newest_reports_are_kept() {
        let dir = temp_dir("prune");
        for i in 0..5 {
            std::fs::write(dir.join(format!("crash-2026-11-0{i}.log")), "x").unwrap();
        }
        std::fs::write(dir.join(SESSION_LOG), "x").unwrap();
        prune_reports(&dir, 2);
        let mut left: Vec<String> = std::fs::read_dir(&dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .collect();
        left.sort();
        assert_eq!(
            left,
            ["crash-2026-11-03.log", "crash-2026-11-04.log", SESSION_LOG]
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_new_session_keeps_the_log_before() {
        let dir = temp_dir("session");
        std::fs::write(dir.join(SESSION_LOG), "last run").unwrap();
        let mut file = open_session_log(&dir).unwrap();
        writeln!(file, "this run").unwrap();
        assert_eq!(
            std::fs::read_to_string(dir.join(PREVIOUS_LOG)).unwrap(),
            "last run"
        );
        assert_eq!(
            std::fs::read_to_string(dir.join(SESSION_LOG)).unwrap(),
            "this run\n"
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_report_lands_in_the_folder() {
        let dir = temp_dir("report");
        let path = write_report(&dir, "text").unwrap();
        assert!(path.starts_with(&dir));
        assert_eq!(std::fs::read_to_string(path).unwrap(), "text");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
