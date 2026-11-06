//! Export of the result animation as GIF, MP4 or MKV.
//!
//! The frames are taken from the 3D view like a screenshot, with legend and labels, one after
//! another while the animation window is hidden. A worker thread encodes them: GIF with the
//! pure Rust `gif` encoder, MP4 and MKV by piping the raw frames into FFmpeg (H.264 if FFmpeg
//! has it), which has to be installed.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc::{Receiver, Sender};

use egui::{Color32, ColorImage, Context, Rect, UserData, ViewportCommand};

use crate::animation::Playback;

/// File format of an exported animation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    Gif,
    Mp4,
    Mkv,
}

impl Format {
    pub const ALL: [Format; 3] = [Format::Gif, Format::Mp4, Format::Mkv];

    pub fn extension(self) -> &'static str {
        match self {
            Format::Gif => "gif",
            Format::Mp4 => "mp4",
            Format::Mkv => "mkv",
        }
    }

    /// Entry of the export menu.
    pub fn menu_label(self) -> &'static str {
        match self {
            Format::Gif => "GIF …",
            Format::Mp4 => "MP4 …",
            Format::Mkv => "MKV …",
        }
    }

    /// Name in the file dialog's filter.
    pub fn filter_name(self) -> &'static str {
        match self {
            Format::Gif => "GIF animation",
            Format::Mp4 => "MP4 video",
            Format::Mkv => "Matroska video",
        }
    }

    /// Whether the format is written by FFmpeg.
    pub fn needs_ffmpeg(self) -> bool {
        self != Format::Gif
    }
}

/// Frames in the order the file shows them, so that it plays like the animation window: once
/// or a loop runs through all frames, a swing runs there and back again without repeating the
/// turning frames, so that a looping player swings smoothly.
pub fn frame_sequence(frame_count: usize, playback: Playback) -> Vec<usize> {
    let forward = 0..frame_count.max(1);
    match playback {
        Playback::Once | Playback::Loop => forward.collect(),
        Playback::Swing => forward
            .clone()
            .chain((1..frame_count.saturating_sub(1)).rev())
            .collect(),
    }
}

/// The FFmpeg executable: the one set in the settings, else `ffmpeg` on the `PATH`.
pub fn ffmpeg_executable(setting: &str) -> PathBuf {
    let setting = crate::settings::clean_path(setting);
    if setting.is_empty() {
        "ffmpeg".into()
    } else {
        setting.into()
    }
}

/// Version line of FFmpeg, or why it cannot be run.
pub fn ffmpeg_version(ffmpeg: &Path) -> Result<String, String> {
    let output = plx_solve::hide_console(&mut Command::new(ffmpeg))
        .arg("-version")
        .stdin(Stdio::null())
        .output()
        .map_err(|err| format!("FFmpeg could not be started ({}): {err}", ffmpeg.display()))?;
    if !output.status.success() {
        return Err(format!("FFmpeg failed ({})", ffmpeg.display()));
    }
    let text = String::from_utf8_lossy(&output.stdout);
    Ok(text.lines().next().unwrap_or("FFmpeg").trim().to_owned())
}

/// Frames painted after a frame is shown before it is captured, so that the scene and the
/// legend of the new frame are on screen.
const SETTLE_FRAMES: u8 = 2;
/// Frames to wait for a requested capture before asking again.
const CAPTURE_TIMEOUT: u32 = 60;

/// Tag of a capture of the export, with the position in the frame sequence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct FrameTag(usize);

/// The frames being taken from the 3D view, one per few painted frames.
pub struct Recording {
    sequence: Vec<usize>,
    /// Position in `sequence` of the frame on screen.
    position: usize,
    settle: u8,
    /// Painted frames since the capture of the frame on screen was requested.
    waiting: Option<u32>,
    encoder: Encoder,
    /// File name, for the status bar.
    pub file: String,
    /// Animation frame and playing state from before, restored at the end.
    pub restore: (usize, bool),
    /// The frame drawn now is captured: the progress window stays out of it.
    pub capturing: bool,
    cancelled: bool,
    /// A capture was requested in the last frame.
    requested: bool,
}

/// What the app does next for a recording.
pub enum Step {
    /// Show this animation frame.
    Show(usize),
    Wait,
    /// All frames taken; the encoder finishes the file.
    Finished(Encoder),
    /// The user pressed Escape.
    Cancelled,
}

impl Recording {
    /// A recording of the frames in `sequence`; the app shows the first one.
    pub fn new(
        sequence: Vec<usize>,
        encoder: Encoder,
        file: String,
        restore: (usize, bool),
    ) -> Self {
        Self {
            sequence,
            position: 0,
            settle: SETTLE_FRAMES,
            waiting: None,
            encoder,
            file,
            restore,
            capturing: false,
            cancelled: false,
            requested: false,
        }
    }

    /// Stops the recording at the next update.
    pub fn cancel(&mut self) {
        self.cancelled = true;
    }

    pub fn first_frame(&self) -> usize {
        self.sequence.first().copied().unwrap_or(0)
    }

    /// Frames taken and the number of all frames.
    pub fn progress(&self) -> (usize, usize) {
        (self.position, self.sequence.len())
    }

    /// Takes the capture of the frame on screen if it arrived, or requests it; `view` is the
    /// 3D view in points.
    pub fn update(mut self, ctx: &Context, view: Rect) -> (Option<Self>, Step) {
        if self.cancelled || ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            return (None, Step::Cancelled);
        }
        // The screenshot requested last frame is of the frame drawn now.
        self.capturing = std::mem::take(&mut self.requested);
        ctx.request_repaint();
        let position = self.position;
        let captured = ctx.input(|i| {
            i.raw.events.iter().find_map(|event| match event {
                egui::Event::Screenshot {
                    user_data, image, ..
                } => user_data
                    .data
                    .as_ref()
                    .and_then(|d| d.downcast_ref::<FrameTag>())
                    .filter(|tag| tag.0 == position)
                    .map(|_| image.clone()),
                _ => None,
            })
        });
        if let Some(image) = captured {
            let image = crate::screenshot::crop(&image, view, ctx.pixels_per_point());
            self.encoder.add(image);
            self.position += 1;
            self.waiting = None;
            self.settle = SETTLE_FRAMES;
            return match self.sequence.get(self.position) {
                Some(&frame) => (Some(self), Step::Show(frame)),
                None => {
                    self.encoder.finish();
                    (None, Step::Finished(self.encoder))
                }
            };
        }
        if self.settle > 0 {
            self.settle -= 1;
            return (Some(self), Step::Wait);
        }
        match &mut self.waiting {
            Some(frames) if *frames < CAPTURE_TIMEOUT => *frames += 1,
            _ => {
                ctx.send_viewport_cmd(ViewportCommand::Screenshot(UserData::new(FrameTag(
                    position,
                ))));
                self.waiting = Some(0);
                self.requested = true;
            }
        }
        (Some(self), Step::Wait)
    }
}

/// What the encoder is told to write.
pub struct Job {
    pub format: Format,
    pub path: PathBuf,
    pub fps: f32,
    /// The file plays endlessly (GIF); videos play once anyway.
    pub repeat: bool,
    pub ffmpeg: PathBuf,
}

/// Message to the encoder thread.
enum Message {
    Frame(ColorImage),
    /// All frames sent; finish the file.
    Finish,
}

/// An encoder running in a worker thread. Dropping it without [`Encoder::finish`] cancels the
/// export and removes the partial file.
pub struct Encoder {
    sender: Sender<Message>,
    result: Receiver<Result<String, String>>,
    /// Size of the first frame; later frames are fitted to it.
    size: Option<[usize; 2]>,
}

impl Encoder {
    pub fn start(job: Job) -> Self {
        let (sender, messages) = std::sync::mpsc::channel();
        let (results, result) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let outcome = encode(&job, messages);
            if outcome.is_err() {
                let _ = std::fs::remove_file(&job.path);
            }
            let _ = results.send(outcome);
        });
        Self {
            sender,
            result,
            size: None,
        }
    }

    /// Hands a frame to the encoder. Video frames need even sizes and all frames the same, so
    /// the first frame is cut to even size and later ones to that of the first.
    pub fn add(&mut self, image: ColorImage) {
        let size = *self
            .size
            .get_or_insert([image.size[0] & !1, image.size[1] & !1].map(|v| v.max(2)));
        let _ = self.sender.send(Message::Frame(fit(&image, size)));
    }

    /// Tells the encoder that all frames are sent; the result comes from [`Encoder::poll`].
    pub fn finish(&self) {
        let _ = self.sender.send(Message::Finish);
    }

    /// The outcome once the file is written: a message for the output, or the error.
    pub fn poll(&self) -> Option<Result<String, String>> {
        match self.result.try_recv() {
            Ok(result) => Some(result),
            Err(std::sync::mpsc::TryRecvError::Empty) => None,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                Some(Err("The encoder stopped unexpectedly".into()))
            }
        }
    }
}

/// `image` cut or padded (with white) to `size`.
fn fit(image: &ColorImage, size: [usize; 2]) -> ColorImage {
    if image.size == size {
        return image.clone();
    }
    let mut fitted = ColorImage::filled(size, Color32::WHITE);
    for y in 0..size[1].min(image.size[1]) {
        for x in 0..size[0].min(image.size[0]) {
            fitted.pixels[y * size[0] + x] = image.pixels[y * image.size[0] + x];
        }
    }
    fitted
}

fn rgba(image: &ColorImage) -> Vec<u8> {
    image
        .pixels
        .iter()
        .flat_map(|c| c.to_srgba_unmultiplied())
        .collect()
}

/// Writes the frames received until [`Message::Finish`]; an error when the sender went away
/// before (cancelled).
fn encode(job: &Job, messages: Receiver<Message>) -> Result<String, String> {
    let mut writer: Option<Writer> = None;
    let mut frames = 0usize;
    let mut size = [0, 0];
    loop {
        match messages.recv() {
            Ok(Message::Frame(image)) => {
                if writer.is_none() {
                    size = image.size;
                    writer = Some(Writer::new(job, size)?);
                }
                if let Some(writer) = &mut writer {
                    writer.frame(&image, frames, job.fps)?;
                }
                frames += 1;
            }
            Ok(Message::Finish) => break,
            Err(_) => {
                if let Some(writer) = writer {
                    writer.abort();
                }
                return Err("Export cancelled".into());
            }
        }
    }
    let writer = writer.ok_or("No frames to export")?;
    writer.finish()?;
    Ok(format!(
        "Animation saved ({frames} frames, {} x {}): {}",
        size[0],
        size[1],
        job.path.display()
    ))
}

enum Writer {
    Gif(gif::Encoder<std::io::BufWriter<std::fs::File>>),
    Ffmpeg {
        child: Child,
        stdin: Option<ChildStdin>,
        stderr: std::thread::JoinHandle<String>,
    },
}

impl Writer {
    fn new(job: &Job, [width, height]: [usize; 2]) -> Result<Self, String> {
        let fail = |err: &dyn std::fmt::Display| {
            format!("{} could not be written: {err}", job.path.display())
        };
        match job.format {
            Format::Gif => {
                let (Ok(w), Ok(h)) = (u16::try_from(width), u16::try_from(height)) else {
                    return Err(fail(&"the view is too large for a GIF"));
                };
                let file = std::fs::File::create(&job.path).map_err(|e| fail(&e))?;
                let mut encoder = gif::Encoder::new(std::io::BufWriter::new(file), w, h, &[])
                    .map_err(|e| fail(&e))?;
                let repeat = if job.repeat {
                    gif::Repeat::Infinite
                } else {
                    gif::Repeat::Finite(0)
                };
                encoder.set_repeat(repeat).map_err(|e| fail(&e))?;
                Ok(Writer::Gif(encoder))
            }
            Format::Mp4 | Format::Mkv => {
                let mut command = Command::new(&job.ffmpeg);
                plx_solve::hide_console(&mut command);
                command.args(ffmpeg_args(
                    [width, height],
                    job.fps,
                    has_h264(&job.ffmpeg),
                    &job.path,
                ));
                command
                    .stdin(Stdio::piped())
                    .stdout(Stdio::null())
                    .stderr(Stdio::piped());
                let mut child = command.spawn().map_err(|err| {
                    format!(
                        "FFmpeg could not be started ({}): {err}",
                        job.ffmpeg.display()
                    )
                })?;
                let stdin = child.stdin.take();
                let mut pipe = child.stderr.take();
                // Read the log while writing, so that a full pipe cannot block FFmpeg.
                let stderr = std::thread::spawn(move || {
                    let mut text = String::new();
                    if let Some(pipe) = &mut pipe {
                        let _ = std::io::Read::read_to_string(pipe, &mut text);
                    }
                    text
                });
                Ok(Writer::Ffmpeg {
                    child,
                    stdin,
                    stderr,
                })
            }
        }
    }

    fn frame(&mut self, image: &ColorImage, index: usize, fps: f32) -> Result<(), String> {
        let mut pixels = rgba(image);
        match self {
            Writer::Gif(encoder) => {
                let [w, h] = image.size.map(|v| v as u16);
                let mut frame = gif::Frame::from_rgba_speed(w, h, &mut pixels, 10);
                frame.delay = gif_delay(index, fps);
                encoder
                    .write_frame(&frame)
                    .map_err(|err| format!("GIF frame could not be written: {err}"))
            }
            Writer::Ffmpeg { stdin, .. } => {
                let written = stdin
                    .as_mut()
                    .is_some_and(|pipe| pipe.write_all(&pixels).is_ok());
                if written {
                    return Ok(());
                }
                // FFmpeg quit; its log says why.
                self.finish_ffmpeg()?;
                Err("FFmpeg stopped reading frames".into())
            }
        }
    }

    fn finish(mut self) -> Result<(), String> {
        match self {
            Writer::Gif(encoder) => encoder
                .into_inner()
                .and_then(|mut file| file.flush().map_err(Into::into))
                .map_err(|err| format!("GIF could not be written: {err}")),
            Writer::Ffmpeg { .. } => self.finish_ffmpeg(),
        }
    }

    /// Closes FFmpeg's input and waits for it; an error with its last log line when it
    /// failed.
    fn finish_ffmpeg(&mut self) -> Result<(), String> {
        let Writer::Ffmpeg {
            child,
            stdin,
            stderr,
        } = self
        else {
            return Ok(());
        };
        stdin.take();
        let status = child
            .wait()
            .map_err(|err| format!("FFmpeg failed: {err}"))?;
        if status.success() {
            return Ok(());
        }
        let log = std::mem::replace(stderr, std::thread::spawn(String::new))
            .join()
            .unwrap_or_default();
        let reason = log
            .lines()
            .rev()
            .find(|line| !line.trim().is_empty())
            .unwrap_or("no message");
        Err(format!("FFmpeg failed ({status}): {reason}"))
    }

    fn abort(self) {
        if let Writer::Ffmpeg {
            mut child, stdin, ..
        } = self
        {
            drop(stdin);
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

/// Whether FFmpeg has the H.264 encoder libx264; some distributions ship it without.
fn has_h264(ffmpeg: &Path) -> bool {
    plx_solve::hide_console(&mut Command::new(ffmpeg))
        .args(["-hide_banner", "-encoders"])
        .stdin(Stdio::null())
        .stderr(Stdio::null())
        .output()
        .is_ok_and(|out| String::from_utf8_lossy(&out.stdout).contains("libx264"))
}

/// FFmpeg's arguments to read raw RGBA frames from its input and write `path`: H.264 when
/// available, else the container's default codec, in yuv420p so that every player can show it.
fn ffmpeg_args(size: [usize; 2], fps: f32, h264: bool, path: &Path) -> Vec<std::ffi::OsString> {
    let mut args: Vec<std::ffi::OsString> = [
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgba",
        "-s",
        &format!("{}x{}", size[0], size[1]),
        "-framerate",
        &format!("{fps}"),
        "-i",
        "-",
    ]
    .iter()
    .map(Into::into)
    .collect();
    if h264 {
        args.extend(["-c:v", "libx264", "-preset", "medium", "-crf", "18"].map(Into::into));
    } else {
        args.extend(["-q:v", "2"].map(Into::into));
    }
    args.extend(["-pix_fmt", "yuv420p"].map(Into::into));
    args.push(path.into());
    args
}

/// Delay of GIF frame `index` in hundredths of a second. The delays add up to the exact
/// playing time; at least 2, since players show shorter delays far slower.
fn gif_delay(index: usize, fps: f32) -> u16 {
    let at = |i: usize| (i as f64 * 100.0 / fps.max(0.1) as f64).round() as i64;
    (at(index + 1) - at(index)).clamp(2, u16::MAX as i64) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn swing_goes_there_and_back_without_repeating_the_ends() {
        assert_eq!(frame_sequence(4, Playback::Swing), [0, 1, 2, 3, 2, 1]);
        assert_eq!(frame_sequence(2, Playback::Swing), [0, 1]);
        assert_eq!(frame_sequence(3, Playback::Once), [0, 1, 2]);
        assert_eq!(frame_sequence(3, Playback::Loop), [0, 1, 2]);
        assert_eq!(frame_sequence(1, Playback::Swing), [0]);
    }

    #[test]
    fn gif_delays_add_up_to_the_frame_rate() {
        let total: u32 = (0..30).map(|i| gif_delay(i, 30.0) as u32).sum();
        assert_eq!(total, 100);
        assert_eq!(gif_delay(0, 15.0), 7);
        assert_eq!(gif_delay(0, 60.0), 2);
    }

    #[test]
    fn frames_are_cut_to_the_first_size() {
        let mut image = ColorImage::filled([3, 2], Color32::RED);
        image.pixels[1] = Color32::BLUE;
        let fitted = fit(&image, [2, 3]);
        assert_eq!(fitted.size, [2, 3]);
        assert_eq!(fitted.pixels[1], Color32::BLUE);
        assert_eq!(fitted.pixels[2], Color32::RED);
        assert_eq!(fitted.pixels[5], Color32::WHITE);
    }

    fn colored_frames(encoder: &mut Encoder) {
        for color in [Color32::RED, Color32::GREEN, Color32::BLUE] {
            encoder.add(ColorImage::filled([33, 20], color));
        }
        encoder.finish();
    }

    fn wait(encoder: &Encoder) -> Result<String, String> {
        loop {
            if let Some(result) = encoder.poll() {
                return result;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }

    fn temp_path(extension: &str) -> PathBuf {
        std::env::temp_dir().join(format!("plx-animation-{}.{extension}", std::process::id()))
    }

    fn job(format: Format, path: &Path) -> Job {
        Job {
            format,
            path: path.to_owned(),
            fps: 15.0,
            repeat: true,
            ffmpeg: ffmpeg_executable(""),
        }
    }

    #[test]
    fn writes_a_looping_gif() {
        let path = temp_path("gif");
        let mut encoder = Encoder::start(job(Format::Gif, &path));
        colored_frames(&mut encoder);
        let message = wait(&encoder).unwrap();
        assert!(message.contains("3 frames, 32 x 20"), "{message}");
        let bytes = std::fs::read(&path).unwrap();
        std::fs::remove_file(&path).unwrap();
        assert_eq!(&bytes[..6], b"GIF89a");
        assert_eq!(u16::from_le_bytes([bytes[6], bytes[7]]), 32);
        assert!(bytes.windows(11).any(|w| w == b"NETSCAPE2.0"));
    }

    #[test]
    fn dropping_the_encoder_removes_the_file() {
        let path = temp_path("cancel.gif");
        let mut encoder = Encoder::start(job(Format::Gif, &path));
        encoder.add(ColorImage::filled([4, 4], Color32::RED));
        let Encoder { sender, result, .. } = encoder;
        drop(sender);
        assert!(result.recv().unwrap().is_err());
        assert!(!path.exists());
    }

    #[test]
    fn writes_videos_with_ffmpeg() {
        if ffmpeg_version(&ffmpeg_executable("")).is_err() {
            eprintln!("ffmpeg not found, test skipped");
            return;
        }
        for format in [Format::Mp4, Format::Mkv] {
            let path = temp_path(format.extension());
            let mut encoder = Encoder::start(job(format, &path));
            colored_frames(&mut encoder);
            wait(&encoder).unwrap();
            let size = std::fs::metadata(&path).unwrap().len();
            std::fs::remove_file(&path).unwrap();
            assert!(size > 100, "{format:?}: {size} bytes");
        }
    }

    #[test]
    fn missing_ffmpeg_is_reported() {
        let path = temp_path("missing.mp4");
        let mut job = job(Format::Mp4, &path);
        job.ffmpeg = "/nonexistent/ffmpeg".into();
        assert!(ffmpeg_version(&job.ffmpeg).is_err());
        let mut encoder = Encoder::start(job);
        colored_frames(&mut encoder);
        let err = wait(&encoder).unwrap_err();
        assert!(err.contains("FFmpeg could not be started"), "{err}");
    }
}
