//! Help > Check for Updates and the check at startup: asks GitHub for the latest release. A
//! newer one shows a symbol next to the version in the menu bar; it opens a window with the
//! release notes. The Windows installer is downloaded there and run with `/UPDATE`, which skips
//! its pages, waits for Nodalis to close and starts the new version. An AppImage is left to the
//! system's AppImage manager; the portable ZIP gets a link to the new ZIP, other builds (from
//! source) only point to the release page.

use std::io::{Read, Write};
use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, channel};
use std::time::Duration;

use serde::Deserialize;
use sha2::Digest;

/// The latest release that is neither a draft nor a pre-release.
const LATEST_RELEASE: &str = "https://api.github.com/repos/KwentiN-ui/Nodalis/releases/latest";
pub const RELEASES_PAGE: &str = "https://github.com/KwentiN-ui/Nodalis/releases/latest";

const CANCELLED: &str = "download cancelled";

/// A version number `major.minor.patch`, compared numerically.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Version(u64, u64, u64);

impl Version {
    /// `0.2.5` or a tag like `v0.2.5`; a suffix such as `-rc1` is ignored.
    pub fn parse(text: &str) -> Option<Self> {
        let text = text.trim();
        let text = text.strip_prefix(['v', 'V']).unwrap_or(text);
        let core = text.split(['-', '+']).next()?;
        let parts: Option<Vec<u64>> = core.split('.').map(|p| p.parse().ok()).collect();
        match parts?[..] {
            [major] => Some(Self(major, 0, 0)),
            [major, minor] => Some(Self(major, minor, 0)),
            [major, minor, patch] => Some(Self(major, minor, patch)),
            _ => None,
        }
    }

    /// The version of this program.
    pub fn current() -> Self {
        Self::parse(env!("CARGO_PKG_VERSION")).unwrap_or(Self(0, 0, 0))
    }
}

impl std::fmt::Display for Version {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}.{}", self.0, self.1, self.2)
    }
}

/// How this copy of Nodalis was installed, which decides how it is updated.
#[derive(Clone, Debug, PartialEq)]
pub enum Installation {
    /// Installed by the Windows installer (`uninstall.exe` next to the program).
    WindowsInstaller,
    /// Unpacked from the portable Windows ZIP (`portable.txt` next to the program); the new
    /// ZIP is downloaded in the browser and unpacked by hand.
    Portable,
    /// Started from an AppImage, which an AppImage manager such as Gear Lever updates.
    AppImage,
    /// Built from source or installed otherwise; updated by hand.
    Other,
}

impl Installation {
    pub fn detect() -> Self {
        let exe_dir = std::env::current_exe()
            .ok()
            .and_then(|exe| exe.parent().map(Path::to_path_buf));
        if crate::portable::is_portable() {
            Self::Portable
        } else if cfg!(windows) && exe_dir.is_some_and(|dir| dir.join("uninstall.exe").is_file()) {
            Self::WindowsInstaller
        } else if cfg!(target_os = "linux") && std::env::var_os("APPIMAGE").is_some() {
            // The AppImage runtime names the file it was started from.
            Self::AppImage
        } else {
            Self::Other
        }
    }

    /// Which file of a release updates this installation.
    fn wants(&self, asset_name: &str) -> bool {
        match self {
            Self::WindowsInstaller => asset_name.ends_with("-setup.exe"),
            Self::Portable => asset_name.ends_with("-windows-portable.zip"),
            Self::AppImage | Self::Other => false,
        }
    }

    /// Whether Nodalis installs the release's file itself ("Update Now").
    fn installs_itself(&self) -> bool {
        *self == Self::WindowsInstaller
    }

    /// Whether Nodalis asks for updates at startup: not in an AppImage, whose manager does.
    pub fn checks_at_startup(&self) -> bool {
        *self != Self::AppImage
    }

    /// One line for the settings window.
    pub fn description(&self) -> &'static str {
        match self {
            Self::WindowsInstaller => {
                "Installed with the Windows installer: updates are downloaded and installed from \
                 within Nodalis."
            }
            Self::Portable => {
                "Portable version: new versions are announced with a link to the new ZIP file, \
                 which you unpack yourself."
            }
            Self::AppImage => {
                "Running as AppImage: an AppImage manager such as Gear Lever or AppImageLauncher \
                 updates it, so Nodalis does not check at startup."
            }
            Self::Other => {
                "Not installed with the Windows installer: new versions are announced, but must \
                 be downloaded from the release page."
            }
        }
    }
}

/// The latest release, as far as this installation is concerned.
#[derive(Clone, Debug, PartialEq)]
pub struct Release {
    pub version: Version,
    /// The release notes, Markdown as written on GitHub.
    pub notes: String,
    pub page: String,
    /// The file that updates this installation, if the release has one.
    pub asset: Option<Asset>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Asset {
    pub name: String,
    pub url: String,
    pub size: u64,
    /// SHA-256 of the file in hexadecimal, as GitHub reports it.
    pub sha256: Option<String>,
}

/// What GitHub's REST API returns for a release, as far as needed.
#[derive(Deserialize)]
struct ApiRelease {
    tag_name: String,
    html_url: String,
    #[serde(default)]
    body: Option<String>,
    #[serde(default)]
    assets: Vec<ApiAsset>,
}

#[derive(Deserialize)]
struct ApiAsset {
    name: String,
    browser_download_url: String,
    size: u64,
    /// `sha256:<hex>`, given for files uploaded since mid 2025.
    #[serde(default)]
    digest: Option<String>,
}

fn parse_release(json: &str, installation: &Installation) -> Result<Release, String> {
    let release: ApiRelease =
        serde_json::from_str(json).map_err(|e| format!("unexpected answer from GitHub: {e}"))?;
    let version = Version::parse(&release.tag_name).ok_or_else(|| {
        format!(
            "the latest release has no version number: {}",
            release.tag_name
        )
    })?;
    let asset = (release.assets.into_iter())
        .find(|a| installation.wants(&a.name))
        .map(|a| Asset {
            sha256: (a.digest.as_deref())
                .and_then(|d| d.strip_prefix("sha256:"))
                .map(str::to_ascii_lowercase),
            name: a.name,
            url: a.browser_download_url,
            size: a.size,
        });
    Ok(Release {
        version,
        notes: release.body.unwrap_or_default().replace("\r\n", "\n"),
        page: release.html_url,
        asset,
    })
}

fn agent(timeout: Option<Duration>) -> ureq::Agent {
    use ureq::tls::{RootCerts, TlsConfig};
    ureq::Agent::config_builder()
        .timeout_global(timeout)
        .timeout_connect(Some(Duration::from_secs(20)))
        .user_agent(concat!("Nodalis/", env!("CARGO_PKG_VERSION")))
        // See `get`.
        .max_redirects(0)
        // The certificates of the system, so that a company proxy that inspects TLS works.
        .tls_config(
            TlsConfig::builder()
                .root_certs(RootCerts::PlatformVerifier)
                .build(),
        )
        .build()
        .into()
}

/// A GET request that follows redirects itself: ureq reads the large redirect of a GitHub
/// download (a few kB of headers) only partly and then fails on its body.
fn get(
    agent: &ureq::Agent,
    url: &str,
    accept: &str,
) -> Result<ureq::http::Response<ureq::Body>, ureq::Error> {
    let mut url = url.to_owned();
    for _ in 0..5 {
        let response = agent.get(&url).header("Accept", accept).call()?;
        let location = (response.status().is_redirection())
            .then(|| response.headers().get("location")?.to_str().ok())
            .flatten();
        match location {
            Some(location) => url = location.to_owned(),
            None => return Ok(response),
        }
    }
    Err(ureq::Error::TooManyRedirects)
}

/// Asks GitHub for the latest release.
fn fetch_latest(installation: &Installation) -> Result<Release, String> {
    let agent = agent(Some(Duration::from_secs(30)));
    let mut response =
        get(&agent, LATEST_RELEASE, "application/vnd.github+json").map_err(|e| match e {
            // The repository has no release yet.
            ureq::Error::StatusCode(404) => "No release published yet".to_owned(),
            e => format!("GitHub not reachable: {e}"),
        })?;
    let json = response
        .body_mut()
        .read_to_string()
        .map_err(|e| format!("GitHub not reachable: {e}"))?;
    parse_release(&json, installation)
}

/// Downloads the asset to `dest`, through a `.part` file next to it, and checks its size and
/// checksum. `progress` counts the bytes received; `cancel` stops the download.
fn download(
    asset: &Asset,
    dest: &Path,
    progress: &AtomicU64,
    cancel: &AtomicBool,
) -> Result<(), String> {
    let part = dest.with_file_name(format!(
        ".{}.part",
        dest.file_name().unwrap_or_default().to_string_lossy()
    ));
    let result = download_to(asset, &part, progress, cancel)
        .and_then(|()| std::fs::rename(&part, dest).map_err(|e| e.to_string()));
    if result.is_err() {
        let _ = std::fs::remove_file(&part);
    }
    result
}

fn download_to(
    asset: &Asset,
    part: &Path,
    progress: &AtomicU64,
    cancel: &AtomicBool,
) -> Result<(), String> {
    let response = get(&agent(None), &asset.url, "application/octet-stream")
        .map_err(|e| format!("download failed: {e}"))?;
    let mut reader = response.into_body().into_reader();
    let mut file =
        std::fs::File::create(part).map_err(|e| format!("cannot write {}: {e}", part.display()))?;
    let mut hasher = sha2::Sha256::new();
    let mut buffer = vec![0; 1 << 16];
    let mut received = 0u64;
    loop {
        if cancel.load(Ordering::Relaxed) {
            return Err(CANCELLED.into());
        }
        let n = reader
            .read(&mut buffer)
            .map_err(|e| format!("download failed: {e}"))?;
        if n == 0 {
            break;
        }
        hasher.update(&buffer[..n]);
        file.write_all(&buffer[..n])
            .map_err(|e| format!("cannot write {}: {e}", part.display()))?;
        received += n as u64;
        progress.store(received, Ordering::Relaxed);
    }
    file.sync_all().map_err(|e| e.to_string())?;
    check_download(asset, received, &hex(&hasher.finalize()))
}

fn check_download(asset: &Asset, size: u64, sha256: &str) -> Result<(), String> {
    if size != asset.size {
        return Err(format!(
            "download incomplete: {size} of {} bytes received",
            asset.size
        ));
    }
    match &asset.sha256 {
        Some(expected) if expected != sha256 => {
            Err("the downloaded file does not match its checksum on GitHub".into())
        }
        _ => Ok(()),
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

/// Starts the downloaded Windows installer with `/UPDATE`. It needs administrator rights, which
/// only ShellExecute asks for (CreateProcess fails), so PowerShell's `Start-Process -Verb RunAs`
/// starts it. Returns once the user allowed it in the UAC prompt, or with the error.
#[cfg(windows)]
fn start_installer(installer: &Path) -> Result<(), String> {
    use std::os::windows::process::CommandExt;
    const CREATE_NO_WINDOW: u32 = 0x0800_0000;
    let path = installer.display().to_string().replace('\'', "''");
    let script = format!(
        "$ErrorActionPreference = 'Stop'; \
         Start-Process -FilePath '{path}' -ArgumentList '/UPDATE' -Verb RunAs"
    );
    let output = std::process::Command::new("powershell.exe")
        .args(["-NoProfile", "-NonInteractive", "-Command", &script])
        .creation_flags(CREATE_NO_WINDOW)
        .output()
        .map_err(|e| format!("cannot start PowerShell: {e}"))?;
    if output.status.success() {
        Ok(())
    } else {
        let error = String::from_utf8_lossy(&output.stderr);
        // Declining the UAC prompt is the usual reason.
        if error.contains("canceled") || error.contains("cancelled") {
            Err("the installation was not allowed".into())
        } else {
            Err(format!("cannot start the installer: {}", error.trim()))
        }
    }
}

#[cfg(not(windows))]
fn start_installer(_installer: &Path) -> Result<(), String> {
    Err("the Windows installer only runs on Windows".into())
}

/// Downloads the installer to the temporary directory and starts it. On success the program
/// must close.
fn download_and_install(
    asset: &Asset,
    progress: &AtomicU64,
    cancel: &AtomicBool,
) -> Result<(), String> {
    let dir = std::env::temp_dir().join("nodalis-update");
    std::fs::create_dir_all(&dir).map_err(|e| format!("cannot create {}: {e}", dir.display()))?;
    let dest = dir.join(&asset.name);
    download(asset, &dest, progress, cancel)?;
    start_installer(&dest)
}

enum State {
    Idle,
    Checking {
        manual: bool,
        receiver: Receiver<Result<Release, String>>,
    },
    Installing {
        progress: Arc<AtomicU64>,
        cancel: Arc<AtomicBool>,
        receiver: Receiver<Result<(), String>>,
    },
}

/// The update check, the symbol of a newer release and its window.
pub struct Updater {
    installation: Installation,
    state: State,
    /// The newer release found by the last check.
    available: Option<Release>,
    /// The window is open: with the newer release, or with this message of a check by hand
    /// (up to date, or failed) or of a failed installation.
    window: bool,
    message: Option<String>,
}

impl Default for Updater {
    fn default() -> Self {
        Self {
            installation: Installation::detect(),
            state: State::Idle,
            available: None,
            window: false,
            message: None,
        }
    }
}

impl Updater {
    pub fn installation(&self) -> &Installation {
        &self.installation
    }

    /// Whether a check or an installation is under way.
    pub fn busy(&self) -> bool {
        !matches!(self.state, State::Idle)
    }

    /// Asks GitHub for the latest release in the background. `manual` for Help > Check for
    /// Updates, which opens the window with the answer, whatever it is.
    pub fn check(&mut self, ctx: &egui::Context, manual: bool) {
        if self.busy() {
            return;
        }
        let (sender, receiver) = channel();
        let (installation, ctx) = (self.installation.clone(), ctx.clone());
        std::thread::spawn(move || {
            let _ = sender.send(fetch_latest(&installation));
            ctx.request_repaint();
        });
        self.state = State::Checking { manual, receiver };
    }

    /// The symbol next to the version in the menu bar while a newer release is known; a click
    /// opens the update window. Call it in a right-to-left layout.
    pub fn indicator(&mut self, ui: &mut egui::Ui) {
        let Some(release) = &self.available else {
            return;
        };
        let (rect, response) = ui.allocate_exact_size(egui::vec2(18.0, 18.0), egui::Sense::click());
        let response = response
            .on_hover_text(format!(
                "Nodalis {} is available. Click for what's new and to update.",
                release.version
            ))
            .on_hover_cursor(egui::CursorIcon::PointingHand);
        let painter = ui.painter();
        let fill = if response.hovered() {
            egui::Color32::from_rgb(0, 150, 60)
        } else {
            egui::Color32::from_rgb(20, 170, 80)
        };
        let c = rect.center();
        painter.circle_filled(c, 8.0, fill);
        // A white arrow pointing down: download.
        let stroke = egui::Stroke::new(2.0, egui::Color32::WHITE);
        painter.line_segment(
            [c + egui::vec2(0.0, -4.5), c + egui::vec2(0.0, 3.0)],
            stroke,
        );
        painter.line_segment(
            [c + egui::vec2(-3.5, -0.5), c + egui::vec2(0.0, 3.5)],
            stroke,
        );
        painter.line_segment(
            [c + egui::vec2(3.5, -0.5), c + egui::vec2(0.0, 3.5)],
            stroke,
        );
        if response.clicked() {
            self.message = None;
            self.window = true;
        }
    }

    /// Takes the answer of a running check or installation and shows the update window.
    /// Lines for the output pane are pushed to `output`.
    pub fn show(&mut self, ctx: &egui::Context, output: &mut Vec<String>) {
        self.poll(ctx, output);
        if !self.window {
            return;
        }
        let mut open = true;
        let mut close = false;
        let mut install = None;
        let title = if self.message.is_some() {
            "Check for Updates"
        } else {
            "Update Available"
        };
        egui::Window::new(title)
            .id(crate::ids::window(ctx, "update window"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .default_width(480.0)
            .pivot(egui::Align2::CENTER_CENTER)
            .default_pos(ctx.content_rect().center())
            .show(ctx, |ui| {
                if let Some(message) = &self.message {
                    ui.add(egui::Label::new(message.as_str()).wrap());
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        if ui.button("OK").clicked() {
                            close = true;
                        }
                        if ui.button("Open Release Page").clicked() {
                            open_page(RELEASES_PAGE, output);
                            close = true;
                        }
                    });
                } else if let Some(release) = &self.available {
                    match &self.state {
                        State::Installing {
                            progress, cancel, ..
                        } => installing_ui(ui, release, progress, cancel),
                        _ => {
                            let (closed, update) =
                                release_ui(ui, release, &self.installation, output);
                            close = closed;
                            if update {
                                install = release.asset.clone();
                            }
                        }
                    }
                }
            });
        if let Some(asset) = install {
            self.install(asset, ctx);
        }
        if close || !open {
            if let State::Installing { cancel, .. } = &self.state {
                cancel.store(true, Ordering::Relaxed);
            }
            self.window = false;
            self.message = None;
        }
    }

    /// Downloads and installs the release in the background.
    fn install(&mut self, asset: Asset, ctx: &egui::Context) {
        let (progress, cancel) = (Arc::<AtomicU64>::default(), Arc::<AtomicBool>::default());
        let (sender, receiver) = channel();
        let ctx = ctx.clone();
        let (thread_progress, thread_cancel) = (progress.clone(), cancel.clone());
        std::thread::spawn(move || {
            let _ = sender.send(download_and_install(
                &asset,
                &thread_progress,
                &thread_cancel,
            ));
            ctx.request_repaint();
        });
        self.state = State::Installing {
            progress,
            cancel,
            receiver,
        };
    }

    fn poll(&mut self, ctx: &egui::Context, output: &mut Vec<String>) {
        match &self.state {
            State::Idle => {}
            State::Checking { manual, receiver } => {
                let Ok(result) = receiver.try_recv() else {
                    return;
                };
                let manual = *manual;
                self.state = State::Idle;
                match result {
                    Ok(release) if release.version > Version::current() => {
                        output.push(format!("Nodalis {} is available", release.version));
                        self.available = Some(release);
                        self.message = None;
                        self.window |= manual;
                    }
                    Ok(_) => {
                        self.available = None;
                        if manual {
                            self.message = Some(format!(
                                "Nodalis {} is the latest version.",
                                Version::current()
                            ));
                            self.window = true;
                        }
                    }
                    Err(error) if manual => {
                        self.message = Some(format!("No update information: {error}."));
                        self.window = true;
                    }
                    Err(error) => log::info!("update check: {error}"),
                }
            }
            State::Installing { receiver, .. } => {
                let Ok(result) = receiver.try_recv() else {
                    return;
                };
                self.state = State::Idle;
                let version = self.available.as_ref().map(|r| r.version.to_string());
                match result {
                    Ok(()) => {
                        output.push(format!(
                            "Installing Nodalis {}",
                            version.unwrap_or_default()
                        ));
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                    Err(error) => {
                        output.push(format!("Update not installed: {error}"));
                        if error != CANCELLED {
                            self.message = Some(format!("The update was not installed: {error}."));
                            self.window = true;
                        }
                    }
                }
            }
        }
    }
}

/// The newer release with its notes and what can be done; returns whether the window closes
/// and whether to update now.
fn release_ui(
    ui: &mut egui::Ui,
    release: &Release,
    installation: &Installation,
    output: &mut Vec<String>,
) -> (bool, bool) {
    let (mut close, mut update) = (false, false);
    ui.label(egui::RichText::new(format!("Nodalis {} is available.", release.version)).strong());
    ui.label(format!("You are using version {}.", Version::current()));
    if !release.notes.trim().is_empty() {
        ui.add_space(4.0);
        egui::Frame::new()
            .stroke(egui::Stroke::new(1.0, crate::style::border()))
            .inner_margin(6)
            .show(ui, |ui| {
                egui::ScrollArea::vertical()
                    .max_height(260.0)
                    .auto_shrink([false, true])
                    .show(ui, |ui| notes_ui(ui, &release.notes));
            });
    }
    ui.add_space(4.0);
    let installable = release.asset.is_some() && installation.installs_itself();
    let note = match installation {
        Installation::WindowsInstaller if installable => {
            "Nodalis closes to install the update and starts again afterwards. Save your \
             project first. Windows asks for administrator rights."
        }
        Installation::WindowsInstaller => {
            "The release has no installer. Download the new version from the release page."
        }
        Installation::Portable if release.asset.is_some() => {
            "Download the new ZIP file and unpack it next to this folder. Copy the Settings \
             folder over to keep your settings."
        }
        Installation::Portable => {
            "The release has no portable ZIP file. Download the new version from the release \
             page."
        }
        Installation::AppImage => "Your AppImage manager installs the new version.",
        Installation::Other => {
            "This copy of Nodalis cannot update itself. Download the new version from the \
             release page."
        }
    };
    ui.add(egui::Label::new(note).wrap());
    ui.add_space(4.0);
    ui.horizontal(|ui| {
        if installable && ui.button("Update Now").clicked() {
            update = true;
        }
        if *installation == Installation::Portable
            && let Some(asset) = &release.asset
            && ui
                .button("Download ZIP")
                .on_hover_text(&asset.name)
                .clicked()
        {
            open_page(&asset.url, output);
            close = true;
        }
        if ui.button("Open Release Page").clicked() {
            open_page(&release.page, output);
            close = true;
        }
        if ui.button("Close").clicked() {
            close = true;
        }
    });
    (close, update)
}

fn installing_ui(ui: &mut egui::Ui, release: &Release, progress: &AtomicU64, cancel: &AtomicBool) {
    ui.label(format!("Downloading Nodalis {} …", release.version));
    let size = release.asset.as_ref().map_or(0, |a| a.size).max(1);
    let done = progress.load(Ordering::Relaxed);
    let fraction = (done as f32 / size as f32).clamp(0.0, 1.0);
    ui.add(
        egui::ProgressBar::new(fraction)
            .text(format!("{:.1} of {:.1} MB", mb(done), mb(size)))
            .desired_width(440.0),
    );
    if fraction >= 1.0 {
        ui.label("Starting the installation …");
    }
    ui.add_space(4.0);
    if ui.button("Cancel").clicked() {
        cancel.store(true, Ordering::Relaxed);
    }
    ui.ctx().request_repaint_after(Duration::from_millis(100));
}

/// One line of GitHub's release notes, as far as they are shown.
#[derive(Debug, PartialEq)]
enum NoteLine<'a> {
    Heading(&'a str),
    /// A list item, with the pull request its text ends with ("... by @user in <url>").
    Item(&'a str, Option<&'a str>),
    /// A link on a line of its own, e.g. "**Full Changelog**: <url>".
    Link(&'a str, &'a str),
    Text(&'a str),
}

fn note_line(line: &str) -> NoteLine<'_> {
    let line = line.trim();
    if let Some(heading) = line.strip_prefix('#') {
        return NoteLine::Heading(heading.trim_start_matches('#').trim());
    }
    if let Some(item) = line.strip_prefix("* ").or_else(|| line.strip_prefix("- ")) {
        // "Title by @author in https://github.com/.../pull/123"
        if let Some((text, url)) = item.rsplit_once(" in https://") {
            let text = text.rsplit_once(" by @").map_or(text, |(title, _)| title);
            let url = &item[item.len() - url.len() - "https://".len()..];
            return NoteLine::Item(text, Some(url));
        }
        return NoteLine::Item(item, None);
    }
    if let Some((label, url)) = line.split_once(": https://") {
        let url = &line[line.len() - url.len() - "https://".len()..];
        return NoteLine::Link(label.trim_matches('*'), url);
    }
    NoteLine::Text(line)
}

/// GitHub's generated release notes, "What's Changed" with one line per pull request.
fn notes_ui(ui: &mut egui::Ui, notes: &str) {
    for line in notes.lines().filter(|l| !l.trim().is_empty()) {
        match note_line(line) {
            NoteLine::Heading(text) => {
                ui.label(egui::RichText::new(text).strong());
            }
            NoteLine::Item(text, url) => {
                ui.horizontal_wrapped(|ui| {
                    ui.label("-");
                    ui.label(text);
                    if let Some(url) = url {
                        let number = url.rsplit('/').next().unwrap_or_default();
                        ui.hyperlink_to(format!("#{number}"), url);
                    }
                });
            }
            NoteLine::Link(label, url) => {
                ui.hyperlink_to(label, url);
            }
            NoteLine::Text(text) => {
                ui.add(egui::Label::new(text).wrap());
            }
        }
    }
}

fn mb(bytes: u64) -> f64 {
    bytes as f64 / 1e6
}

fn open_page(url: &str, output: &mut Vec<String>) {
    if let Err(error) = open::that_detached(url) {
        output.push(format!("Error opening {url} in the browser: {error}"));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn versions_compare_numerically() {
        assert_eq!(Version::parse("v0.2.5"), Some(Version(0, 2, 5)));
        assert_eq!(Version::parse("0.10.0"), Some(Version(0, 10, 0)));
        assert_eq!(Version::parse("v1.0"), Some(Version(1, 0, 0)));
        assert_eq!(Version::parse("v0.3.0-rc1"), Some(Version(0, 3, 0)));
        assert_eq!(Version::parse("nightly"), None);
        assert_eq!(Version::parse("1.2.3.4"), None);
        assert_eq!(Version::parse("1.x"), None);
        assert!(Version::parse("v0.10.0") > Version::parse("v0.9.9"));
        assert_eq!(Version::current().to_string(), env!("CARGO_PKG_VERSION"));
    }

    const RELEASE: &str = r###"{
        "tag_name": "v0.3.0",
        "html_url": "https://github.com/KwentiN-ui/Nodalis/releases/tag/v0.3.0",
        "body": "## What's Changed\r\n* Self update",
        "assets": [
            {"name": "nodalis-0.3.0-user-guide.pdf", "size": 10,
             "browser_download_url": "https://example.com/guide.pdf"},
            {"name": "nodalis-0.3.0-setup.exe", "size": 1234,
             "browser_download_url": "https://example.com/setup.exe",
             "digest": "sha256:ABCDEF"},
            {"name": "nodalis-0.3.0-x86_64.AppImage.zsync", "size": 5,
             "browser_download_url": "https://example.com/a.zsync", "digest": null},
            {"name": "nodalis-0.3.0-windows-portable.zip", "size": 77,
             "browser_download_url": "https://example.com/portable.zip"},
            {"name": "nodalis-0.3.0-x86_64.AppImage", "size": 99,
             "browser_download_url": "https://example.com/a.AppImage"}
        ]
    }"###;

    #[test]
    fn each_installation_takes_its_file() {
        let windows = parse_release(RELEASE, &Installation::WindowsInstaller).unwrap();
        assert_eq!(windows.version, Version(0, 3, 0));
        assert_eq!(windows.notes, "## What's Changed\n* Self update");
        let asset = windows.asset.unwrap();
        assert_eq!(asset.name, "nodalis-0.3.0-setup.exe");
        assert_eq!(asset.size, 1234);
        assert_eq!(asset.sha256.as_deref(), Some("abcdef"));
        // The portable version links to the new ZIP, but does not install it.
        let portable = parse_release(RELEASE, &Installation::Portable).unwrap();
        assert_eq!(
            portable.asset.unwrap().url,
            "https://example.com/portable.zip"
        );
        assert!(!Installation::Portable.installs_itself());
        // An AppImage manager updates the AppImage.
        let appimage = parse_release(RELEASE, &Installation::AppImage).unwrap();
        assert_eq!(appimage.asset, None);
        let other = parse_release(RELEASE, &Installation::Other).unwrap();
        assert_eq!(other.asset, None);
        assert!(parse_release("{}", &Installation::Other).is_err());
    }

    /// Asks GitHub; run by hand with `cargo test -p plx-app -- --ignored latest_release`.
    #[test]
    #[ignore = "needs the internet"]
    fn latest_release_is_found() {
        let release = fetch_latest(&Installation::WindowsInstaller).unwrap();
        println!("{release:#?}");
        assert!(release.version >= Version(0, 2, 0));
        // The installer's download, with its size and checksum checked.
        let asset = release.asset.unwrap();
        let dest = std::env::temp_dir().join(format!("plx-update-{}", asset.name));
        let progress = AtomicU64::new(0);
        download(&asset, &dest, &progress, &AtomicBool::new(false)).unwrap();
        assert_eq!(progress.into_inner(), asset.size);
        std::fs::remove_file(dest).unwrap();
    }

    #[test]
    fn downloads_are_checked() {
        let asset = Asset {
            name: "a".into(),
            url: String::new(),
            size: 3,
            sha256: Some(hex(&sha2::Sha256::digest(b"abc"))),
        };
        let sha = hex(&sha2::Sha256::digest(b"abc"));
        assert_eq!(
            sha,
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
        assert!(check_download(&asset, 3, &sha).is_ok());
        assert!(check_download(&asset, 2, &sha).is_err());
        assert!(check_download(&asset, 3, &hex(&sha2::Sha256::digest(b"abd"))).is_err());
        let unknown = Asset {
            sha256: None,
            ..asset
        };
        assert!(check_download(&unknown, 3, "anything").is_ok());
    }

    #[test]
    fn release_notes_are_read() {
        assert_eq!(
            note_line("## What's Changed"),
            NoteLine::Heading("What's Changed")
        );
        assert_eq!(
            note_line(
                "* Hexahedra for bodies of revolution by @KwentiN-ui in https://github.com/KwentiN-ui/Nodalis/pull/211"
            ),
            NoteLine::Item(
                "Hexahedra for bodies of revolution",
                Some("https://github.com/KwentiN-ui/Nodalis/pull/211")
            )
        );
        assert_eq!(
            note_line("- Plain item"),
            NoteLine::Item("Plain item", None)
        );
        assert_eq!(
            note_line(
                "**Full Changelog**: https://github.com/KwentiN-ui/Nodalis/compare/v0.2.4...v0.2.5"
            ),
            NoteLine::Link(
                "Full Changelog",
                "https://github.com/KwentiN-ui/Nodalis/compare/v0.2.4...v0.2.5"
            )
        );
        assert_eq!(note_line("Some text"), NoteLine::Text("Some text"));
    }
}
