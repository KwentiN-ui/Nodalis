//! Audio preview of eigenmodes: the eigenfrequencies of a frequency step played as sine tones,
//! alone or several together as one sound.
//!
//! Eigenfrequencies of real parts are often outside the audible range (a long beam hums at a
//! few hertz, a small bracket rings at tens of kilohertz). By default all modes are shifted by
//! the same number of octaves so the lowest chosen mode lands between 110 and 220 Hz; the
//! intervals between the modes, and with them the character of the sound, stay as they are.

use std::f64::consts::TAU;
use std::sync::{Arc, Mutex};

use plx_results::{AnalysisKind, Component, Field, FieldRank, Increment, add_derived_components};

use crate::results::Superposition;

/// Lowest frequency a person hears.
pub const AUDIBLE_MIN: f64 = 20.0;
/// Highest frequency a person hears.
pub const AUDIBLE_MAX: f64 = 20_000.0;
/// Without shifting, modes are played as they are while all lie in this comfortable range.
const COMFORT_MIN: f64 = 40.0;
const COMFORT_MAX: f64 = 16_000.0;
/// An automatic shift puts the lowest mode just below this frequency (A3).
const AUTO_TARGET: f64 = 220.0;
/// Sample rate of exported WAV files.
pub const WAV_RATE: u32 = 44_100;

/// How the eigenfrequencies are made audible.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Transpose {
    /// Played as computed; modes outside the audible range stay silent.
    Off,
    /// Shifted by whole octaves when a chosen mode is outside the comfortable range.
    Automatic,
    /// Shifted by the given number of octaves.
    Octaves,
}

/// How a tone develops over time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Envelope {
    /// Constant loudness until stopped.
    Sustained,
    /// Struck like a bell: every mode dies away, higher modes faster.
    Struck,
}

/// One mode of the frequency step in the sound window.
#[derive(Clone, Debug, PartialEq)]
pub struct Voice {
    /// Index into the increments of the results.
    pub increment: usize,
    pub mode: u32,
    /// Eigenfrequency in hertz.
    pub frequency: f64,
    pub enabled: bool,
    /// Relative loudness, 0 to 1.
    pub level: f32,
}

/// Settings of the sound window, kept with the results it belongs to.
pub struct ModeSound {
    pub step: u32,
    pub voices: Vec<Voice>,
    pub transpose: Transpose,
    /// Octaves of a manual shift.
    pub octaves: i32,
    pub envelope: Envelope,
    /// Time in seconds in which a struck lowest mode falls to about a third.
    pub decay: f32,
    pub volume: f32,
    /// Length of an exported sustained tone in seconds.
    pub duration: f32,
    /// Last problem with the audio device or the export, shown in the window.
    pub message: Option<String>,
    /// Overlay the chosen mode shapes in the 3D view while the sound plays.
    pub show_shape: bool,
    /// Time of the egui clock when the sound was started.
    pub started: Option<f64>,
    /// Swings of the lowest overlaid mode per second in the 3D view.
    pub shape_speed: f32,
    /// Frames per second of the overlay.
    pub shape_fps: f32,
    /// Swings of the lowest mode shown so far, and the clock time of the last frame.
    pub shape_clock: (f64, f64),
    /// Mode shapes prepared for the overlay; rebuilt when the settings or the shown
    /// component change.
    pub mix: Option<ShapeMix>,
    /// Increment of the mode clicked in the window; it swings in the 3D view while no sound
    /// plays.
    pub preview: Option<usize>,
}

/// A sine tone as the synthesizer plays it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tone {
    pub frequency: f64,
    pub amplitude: f32,
    /// Decay time constant in seconds; infinite for a sustained tone.
    pub decay: f32,
}

impl ModeSound {
    /// Sound window for the frequency step of the increment `shown`; the shown mode is the one
    /// enabled at the start. `None` when `shown` is not a mode.
    pub fn new(increments: &[Increment], shown: usize) -> Option<Self> {
        let current = increments.get(shown)?;
        if current.kind != AnalysisKind::Frequency {
            return None;
        }
        let voices = increments
            .iter()
            .enumerate()
            .filter(|(_, inc)| inc.step == current.step && inc.kind == AnalysisKind::Frequency)
            .map(|(index, inc)| Voice {
                increment: index,
                mode: inc.increment,
                frequency: inc.value,
                enabled: index == shown,
                level: 1.0,
            })
            .collect();
        Some(Self {
            step: current.step,
            voices,
            transpose: Transpose::Automatic,
            octaves: 0,
            envelope: Envelope::Sustained,
            decay: 1.5,
            volume: 0.5,
            duration: 3.0,
            message: None,
            show_shape: true,
            started: None,
            shape_speed: DEFAULT_SHAPE_SPEED,
            shape_fps: DEFAULT_SHAPE_FPS,
            shape_clock: (0.0, 0.0),
            preview: None,
            mix: None,
        })
    }

    /// Octaves every mode is shifted by.
    pub fn shift(&self) -> i32 {
        match self.transpose {
            Transpose::Off => 0,
            Transpose::Octaves => self.octaves,
            Transpose::Automatic => automatic_octaves(
                &self
                    .voices
                    .iter()
                    .filter(|v| v.enabled && v.level > 0.0)
                    .map(|v| v.frequency)
                    .collect::<Vec<_>>(),
            ),
        }
    }

    /// Frequency a voice is played at.
    pub fn played(&self, voice: &Voice) -> f64 {
        voice.frequency * 2f64.powi(self.shift())
    }

    /// The tones of the enabled, audible modes, scaled so their sum never clips.
    pub fn tones(&self) -> Vec<Tone> {
        let audible: Vec<(f64, f32)> = self
            .voices
            .iter()
            .filter(|v| v.enabled && v.level > 0.0)
            .map(|v| (self.played(v), v.level))
            .filter(|&(f, _)| is_audible(f))
            .collect();
        let total: f32 = audible.iter().map(|&(_, level)| level).sum();
        let lowest = audible
            .iter()
            .map(|&(f, _)| f)
            .fold(f64::INFINITY, f64::min);
        audible
            .iter()
            .map(|&(frequency, level)| Tone {
                frequency,
                amplitude: self.volume * level / total.max(1.0),
                decay: self.decay_of(frequency, lowest),
            })
            .collect()
    }

    /// Decay time constant of a mode: infinite for a sustained sound; higher modes ring
    /// shorter, as in a struck bell or plate.
    fn decay_of(&self, frequency: f64, lowest: f64) -> f32 {
        match self.envelope {
            Envelope::Sustained => f32::INFINITY,
            Envelope::Struck => self.decay * (lowest / frequency).sqrt() as f32,
        }
    }

    /// Length of an exported file in seconds.
    pub fn export_seconds(&self) -> f32 {
        match self.envelope {
            Envelope::Sustained => self.duration,
            Envelope::Struck => (5.0 * self.decay).min(30.0),
        }
    }
}

pub fn is_audible(frequency: f64) -> bool {
    (AUDIBLE_MIN..=AUDIBLE_MAX).contains(&frequency)
}

/// Octave shift that makes the given frequencies audible: none while all lie in the
/// comfortable range, otherwise the lowest one is put between 110 and 220 Hz.
pub fn automatic_octaves(frequencies: &[f64]) -> i32 {
    let positive = frequencies.iter().copied().filter(|f| *f > 0.0);
    let (low, high) = positive.fold((f64::INFINITY, 0.0f64), |(lo, hi), f| {
        (lo.min(f), hi.max(f))
    });
    if !low.is_finite() || (low >= COMFORT_MIN && high <= COMFORT_MAX) {
        return 0;
    }
    (AUTO_TARGET / low).log2().floor() as i32
}

/// Sine synthesizer shared between the window and the audio callback.
#[derive(Default)]
pub struct Synth {
    tones: Vec<Tone>,
    phases: Vec<f64>,
    /// Seconds since the sound was started.
    time: f64,
    /// Loudness ramp against clicks: 0 silent, 1 full.
    gain: f32,
    gate: bool,
}

/// Time of the fade in and out in seconds.
const RAMP: f32 = 0.02;

impl Synth {
    /// Starts the tones from the beginning, e.g. strikes again.
    pub fn start(&mut self, tones: Vec<Tone>) {
        self.phases = vec![0.0; tones.len()];
        self.tones = tones;
        self.time = 0.0;
        self.gain = 0.0;
        self.gate = true;
    }

    /// Changes the tones of a running sound without restarting it.
    pub fn update(&mut self, tones: Vec<Tone>) {
        self.phases.resize(tones.len(), 0.0);
        self.tones = tones;
    }

    /// Fades out.
    pub fn stop(&mut self) {
        self.gate = false;
    }

    /// Whether anything can still be heard.
    pub fn sounding(&self) -> bool {
        (self.gate || self.gain > 0.0)
            && self
                .tones
                .iter()
                .any(|t| (self.time as f32) < 8.0 * t.decay)
    }

    pub fn next_sample(&mut self, rate: f64) -> f32 {
        let step = (1.0 / (RAMP as f64 * rate)) as f32;
        self.gain = if self.gate {
            (self.gain + step).min(1.0)
        } else {
            (self.gain - step).max(0.0)
        };
        if self.gain == 0.0 && !self.gate {
            return 0.0;
        }
        let time = self.time as f32;
        let mut sum = 0.0;
        for (tone, phase) in self.tones.iter().zip(&mut self.phases) {
            // Tones above the Nyquist frequency of the device would alias.
            if tone.frequency < rate / 2.0 {
                sum += tone.amplitude * (-time / tone.decay).exp() * (*phase * TAU).sin() as f32;
            }
            *phase = (*phase + tone.frequency / rate).fract();
        }
        self.time += 1.0 / rate;
        sum * self.gain
    }
}

/// Renders the tones into a mono 16 bit WAV file.
pub fn wav(tones: Vec<Tone>, seconds: f32) -> Vec<u8> {
    let rate = WAV_RATE as f64;
    let count = (seconds.max(0.1) as f64 * rate) as usize;
    let mut synth = Synth::default();
    synth.start(tones);
    let mut samples: Vec<f32> = (0..count).map(|_| synth.next_sample(rate)).collect();
    // Fade out at the end so the file stops without a click.
    let ramp = ((RAMP as f64 * rate) as usize).min(count);
    for (i, sample) in samples[count - ramp..].iter_mut().enumerate() {
        *sample *= 1.0 - (i + 1) as f32 / ramp as f32;
    }
    let data_len = (count * 2) as u32;
    let mut out = Vec::with_capacity(44 + count * 2);
    out.extend_from_slice(b"RIFF");
    out.extend_from_slice(&(36 + data_len).to_le_bytes());
    out.extend_from_slice(b"WAVEfmt ");
    out.extend_from_slice(&16u32.to_le_bytes());
    out.extend_from_slice(&1u16.to_le_bytes()); // PCM
    out.extend_from_slice(&1u16.to_le_bytes()); // mono
    out.extend_from_slice(&WAV_RATE.to_le_bytes());
    out.extend_from_slice(&(WAV_RATE * 2).to_le_bytes());
    out.extend_from_slice(&2u16.to_le_bytes());
    out.extend_from_slice(&16u16.to_le_bytes());
    out.extend_from_slice(b"data");
    out.extend_from_slice(&data_len.to_le_bytes());
    for sample in samples {
        let value = (sample.clamp(-1.0, 1.0) * i16::MAX as f32) as i16;
        out.extend_from_slice(&value.to_le_bytes());
    }
    out
}

/// Output on the default audio device of the system.
pub struct Player {
    synth: Arc<Mutex<Synth>>,
    _stream: cpal::Stream,
}

impl Player {
    /// Opens the default output device; the error text is shown to the user.
    pub fn open() -> Result<Self, String> {
        use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
        use cpal::{FromSample, SampleFormat, SizedSample};

        fn build<T: SizedSample + FromSample<f32>>(
            device: &cpal::Device,
            config: cpal::StreamConfig,
            synth: Arc<Mutex<Synth>>,
        ) -> Result<cpal::Stream, cpal::Error> {
            let rate = config.sample_rate as f64;
            let channels = config.channels as usize;
            device.build_output_stream(
                config,
                move |data: &mut [T], _: &cpal::OutputCallbackInfo| {
                    let mut synth = synth.lock().unwrap_or_else(|e| e.into_inner());
                    for frame in data.chunks_mut(channels) {
                        let value = T::from_sample(synth.next_sample(rate));
                        frame.fill(value);
                    }
                },
                |error| log::warn!("Audio output: {error}"),
                None,
            )
        }

        let device = cpal::default_host()
            .default_output_device()
            .ok_or("No audio output device found")?;
        let config = device
            .default_output_config()
            .map_err(|e| format!("Audio device not usable: {e}"))?;
        let synth = Arc::new(Mutex::new(Synth::default()));
        let shared = Arc::clone(&synth);
        let stream = match config.sample_format() {
            SampleFormat::F32 => build::<f32>(&device, config.into(), shared),
            SampleFormat::F64 => build::<f64>(&device, config.into(), shared),
            SampleFormat::I16 => build::<i16>(&device, config.into(), shared),
            SampleFormat::I32 => build::<i32>(&device, config.into(), shared),
            SampleFormat::U16 => build::<u16>(&device, config.into(), shared),
            SampleFormat::U8 => build::<u8>(&device, config.into(), shared),
            SampleFormat::I8 => build::<i8>(&device, config.into(), shared),
            other => return Err(format!("Unsupported audio format {other}")),
        }
        .map_err(|e| format!("Audio output could not be opened: {e}"))?;
        stream
            .play()
            .map_err(|e| format!("Audio output could not be started: {e}"))?;
        Ok(Self {
            synth,
            _stream: stream,
        })
    }

    pub fn synth(&self) -> std::sync::MutexGuard<'_, Synth> {
        self.synth.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// Default slow-motion frequency of the lowest chosen mode in the 3D view, in hertz. The
/// other modes swing faster in the ratio of their eigenfrequencies, as they sound.
pub const DEFAULT_SHAPE_SPEED: f32 = 0.5;
/// Default frame rate of the overlay.
pub const DEFAULT_SHAPE_FPS: f32 = 30.0;

/// The chosen mode shapes, prepared to be overlaid frame by frame while the sound plays.
pub struct ShapeMix {
    field: String,
    component: String,
    /// Names of the raw components of the field, when the shown one is derived from them.
    base_names: Option<Vec<String>>,
    /// What the raw components form, which decides what is derived from them.
    rank: FieldRank,
    modes: Vec<MixMode>,
    range: Option<(f32, f32)>,
    peak: f32,
    numbers: Vec<u32>,
    /// The single previewed mode, or `None` for the chosen modes of the sound.
    only: Option<usize>,
}

struct MixMode {
    /// Eigenfrequency relative to the lowest overlaid mode.
    ratio: f64,
    /// Factor that gives every mode the same largest displacement, times its level.
    weight: f32,
    decay: f32,
    displacements: Vec<[f32; 3]>,
    /// Values of the shown component, or of the raw components it is derived from.
    values: Vec<Vec<f32>>,
}

impl ShapeMix {
    /// Prepares the enabled modes for the shown component `field`/`component`, or with `only`
    /// just that mode (the increment index) at full level and without dying away. Mode
    /// shapes have arbitrary amplitudes, so each is scaled to the same largest displacement
    /// first and then by its level. `None` without a mode with displacements.
    pub fn new(
        sound: &ModeSound,
        increments: &[Increment],
        field: &str,
        component: &str,
        only: Option<usize>,
    ) -> Option<Self> {
        let preview: Vec<Voice> = (sound.voices.iter())
            .filter(|v| Some(v.increment) == only)
            .map(|v| Voice {
                level: 1.0,
                ..v.clone()
            })
            .collect();
        let voices = match only {
            Some(_) => &preview,
            None => &sound.voices,
        };
        let chosen: Vec<(&Voice, &Increment, Vec<[f32; 3]>, f32)> = voices
            .iter()
            .filter(|v| (v.enabled || only.is_some()) && v.level > 0.0 && v.frequency > 0.0)
            .filter_map(|v| {
                let increment = increments.get(v.increment)?;
                let displacements = increment.displacements()?;
                let max = displacements
                    .iter()
                    .map(|d| (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt())
                    .fold(0.0f32, f32::max);
                (max > 0.0).then_some((v, increment, displacements, max))
            })
            .collect();
        let reference = chosen.iter().map(|c| c.3).fold(0.0f32, f32::max);
        let lowest = (chosen.iter())
            .map(|c| c.0.frequency)
            .fold(f64::INFINITY, f64::min);
        let derived = increments
            .iter()
            .find_map(|i| i.field(field)?.component(component))
            .is_some_and(|c| c.derived);
        let rank = (chosen.iter())
            .find_map(|c| c.1.field(field))
            .map_or(FieldRank::Scalar, |f| f.rank);
        let base_names: Option<Vec<String>> = derived.then(|| {
            let field = chosen.first().and_then(|c| c.1.field(field));
            field.map_or_else(Vec::new, |f| {
                (f.components.iter())
                    .filter(|c| !c.derived)
                    .map(|c| c.name.clone())
                    .collect()
            })
        });
        let mut bound = 0.0f32;
        let mut invariant = false;
        let modes: Vec<MixMode> = chosen
            .iter()
            .map(|(voice, increment, displacements, max)| {
                let weight = voice.level * reference / max;
                let shown = increment.field(field).and_then(|f| f.component(component));
                if let Some(shown) = shown {
                    invariant = shown.is_invariant();
                    let largest = (shown.range()).map_or(0.0, |(a, b)| a.abs().max(b.abs()));
                    bound += weight * largest;
                }
                let values = match &base_names {
                    Some(names) => names
                        .iter()
                        .map(|n| component_values(increment, field, n))
                        .collect(),
                    None => vec![component_values(increment, field, component)],
                };
                MixMode {
                    ratio: voice.frequency / lowest,
                    weight,
                    decay: match only {
                        Some(_) => f32::INFINITY,
                        None => sound.decay_of(voice.frequency, lowest),
                    },
                    displacements: displacements.clone(),
                    values,
                }
            })
            .collect();
        if modes.is_empty() {
            return None;
        }
        let levels: f32 = chosen.iter().map(|c| c.0.level.powi(2)).sum();
        Some(Self {
            field: field.to_string(),
            component: component.to_string(),
            base_names,
            rank,
            modes,
            range: (bound > 0.0).then_some(if invariant {
                (0.0, bound)
            } else {
                (-bound, bound)
            }),
            peak: reference * levels.sqrt(),
            numbers: chosen.iter().map(|c| c.0.mode).collect(),
            only,
        })
    }

    /// Whether the mix was prepared for this shown component and the same mode selection.
    pub fn shows(&self, field: &str, component: &str, only: Option<usize>) -> bool {
        self.field == field && self.component == component && self.only == only
    }

    /// The overlaid mode shapes after `swings` swings of the lowest mode, `time` seconds after
    /// the sound started (for a struck sound dying away).
    pub fn frame(&self, swings: f64, time: f64) -> Superposition {
        let factors: Vec<f32> = self
            .modes
            .iter()
            .map(|m| {
                let swing = (TAU * m.ratio * swings).sin() as f32;
                m.weight * (-(time as f32) / m.decay).exp() * swing
            })
            .collect();
        let nodes = self.modes[0].displacements.len();
        let mut displacements = vec![[0.0f32; 3]; nodes];
        for (mode, &factor) in self.modes.iter().zip(&factors) {
            for (sum, d) in displacements.iter_mut().zip(&mode.displacements) {
                for k in 0..3 {
                    sum[k] += factor * d[k];
                }
            }
        }
        let sums: Vec<Vec<f32>> = (0..self.modes[0].values.len())
            .map(|c| {
                let mut sum = vec![0.0f32; nodes];
                for (mode, &factor) in self.modes.iter().zip(&factors) {
                    for (s, v) in sum.iter_mut().zip(&mode.values[c]) {
                        *s += factor * v;
                    }
                }
                sum
            })
            .collect();
        let values = match &self.base_names {
            None => sums.into_iter().next(),
            Some(names) => {
                let mut field = Field {
                    name: self.field.clone(),
                    components: names
                        .iter()
                        .zip(sums)
                        .map(|(name, values)| Component {
                            name: name.clone(),
                            values,
                            derived: false,
                            out_of_plane: false,
                        })
                        .collect(),
                    rank: self.rank,
                };
                add_derived_components(&mut field);
                (field.components.into_iter())
                    .find(|c| c.name == self.component)
                    .map(|c| c.values)
            }
        };
        Superposition {
            displacements,
            values: values.filter(|v| v.len() == nodes),
            range: self.range,
            peak: self.peak,
            modes: self.numbers.clone(),
        }
    }
}

/// Values of a component, empty where the increment does not have it.
fn component_values(increment: &Increment, field: &str, component: &str) -> Vec<f32> {
    (increment.field(field))
        .and_then(|f| f.component(component))
        .map(|c| c.values.clone())
        .unwrap_or_default()
}

/// What the user did in the sound window.
#[derive(Default)]
pub struct WindowActions {
    pub play: bool,
    pub stop: bool,
    /// The tones changed; a running sound takes them over.
    pub changed: bool,
    pub export: bool,
    pub close: bool,
    /// Increment of a mode the user clicked, to show it in the 3D view.
    pub show: Option<usize>,
}

fn format_hz(frequency: f64) -> String {
    match frequency.abs() {
        f if f >= 1000.0 => format!("{frequency:.1} Hz"),
        f if f >= 1.0 => format!("{frequency:.2} Hz"),
        _ => format!("{frequency:.4} Hz"),
    }
}

/// The sound window: modes of the step with their frequencies, how they are made audible,
/// and the player.
pub fn window(
    ctx: &egui::Context,
    sound: &mut ModeSound,
    shown: usize,
    playing: bool,
) -> WindowActions {
    use crate::icons::{self, Icon};
    use crate::numeric;

    let mut actions = WindowActions::default();
    let mut open = true;
    egui::Window::new("Sound of the Mode Shapes")
        .id(crate::ids::window(ctx, "mode sound"))
        .open(&mut open)
        .collapsible(false)
        .resizable(false)
        .pivot(egui::Align2::RIGHT_BOTTOM)
        .default_pos(ctx.content_rect().right_bottom() + egui::vec2(-130.0, -60.0))
        .show(ctx, |ui| {
            ui.label(format!(
                "Step {}: check modes to hear them together",
                sound.step
            ));
            let shift = sound.shift();
            let factor = 2f64.powi(shift);
            egui::ScrollArea::vertical()
                .max_height(220.0)
                .show(ui, |ui| {
                    egui::Grid::new("sound modes")
                        .num_columns(5)
                        .spacing([12.0, 4.0])
                        .striped(true)
                        .show(ui, |ui| {
                            ui.label("");
                            ui.strong("Mode");
                            ui.strong("Frequency");
                            ui.strong("Played");
                            ui.strong("Level");
                            ui.end_row();
                            for voice in &mut sound.voices {
                                actions.changed |= ui.checkbox(&mut voice.enabled, "").changed();
                                // Fixed size like the tree rows, so hovering does not make the
                                // row taller.
                                let label = crate::tree::row_label(
                                    ui,
                                    voice.increment == shown,
                                    None,
                                    voice.mode.to_string(),
                                )
                                .on_hover_text("Show this mode shape");
                                if label.clicked() {
                                    actions.show = Some(voice.increment);
                                }
                                ui.label(format_hz(voice.frequency));
                                let played = voice.frequency * factor;
                                if is_audible(played) {
                                    ui.label(format_hz(played));
                                } else {
                                    ui.weak("inaudible");
                                }
                                actions.changed |= ui
                                    .add(
                                        egui::Slider::new(&mut voice.level, 0.0..=1.0)
                                            .show_value(false),
                                    )
                                    .changed();
                                ui.end_row();
                            }
                        });
                });
            ui.horizontal(|ui| {
                for (label, enabled) in [("All", true), ("None", false)] {
                    if ui.button(label).clicked() {
                        sound.voices.iter_mut().for_each(|v| v.enabled = enabled);
                        actions.changed = true;
                    }
                }
            });
            ui.separator();
            egui::Grid::new("sound settings")
                .num_columns(2)
                .spacing([12.0, 6.0])
                .show(ui, |ui| {
                    ui.label("Make audible");
                    ui.horizontal(|ui| {
                        for (transpose, label, tip) in [
                            (
                                Transpose::Automatic,
                                "Automatic",
                                "Shift by whole octaves when a mode lies outside \
                                 40 Hz to 16 kHz; the intervals are kept",
                            ),
                            (Transpose::Octaves, "Octaves", "Shift by fixed octaves"),
                            (Transpose::Off, "Original", "The computed frequencies"),
                        ] {
                            actions.changed |= ui
                                .radio_value(&mut sound.transpose, transpose, label)
                                .on_hover_text(tip)
                                .changed();
                        }
                    });
                    ui.end_row();
                    ui.label("Shift");
                    ui.horizontal(|ui| {
                        if sound.transpose == Transpose::Octaves {
                            actions.changed |= ui
                                .add(numeric::drag_value(&mut sound.octaves).range(-16..=16))
                                .changed();
                            ui.label("Octaves");
                        } else {
                            ui.label(format!("{shift:+} octaves"));
                        }
                        ui.weak(format!("(factor {})", format_factor(shift)));
                    });
                    ui.end_row();
                    ui.label("Sound");
                    ui.horizontal(|ui| {
                        for (envelope, label) in [
                            (Envelope::Sustained, "Sustained"),
                            (Envelope::Struck, "Struck"),
                        ] {
                            actions.changed |= ui
                                .radio_value(&mut sound.envelope, envelope, label)
                                .changed();
                        }
                    });
                    ui.end_row();
                    if sound.envelope == Envelope::Struck {
                        ui.label("Decay time");
                        ui.horizontal(|ui| {
                            actions.changed |= ui
                                .add(numeric::drag_value(&mut sound.decay).range(0.1..=10.0))
                                .changed();
                            ui.label("s");
                        });
                    } else {
                        ui.label("WAV file duration");
                        ui.horizontal(|ui| {
                            ui.add(numeric::drag_value(&mut sound.duration).range(0.5..=60.0));
                            ui.label("s");
                        });
                    }
                    ui.end_row();
                    ui.label("Mode shape");
                    actions.changed |= ui
                        .checkbox(&mut sound.show_shape, "Show superposed while playing")
                        .on_hover_text(
                            "Superposes the selected mode shapes scaled by level; they \
                             oscillate in the ratio of their frequencies, slowed down",
                        )
                        .changed();
                    ui.end_row();
                    if sound.show_shape {
                        ui.label("Speed");
                        ui.horizontal(|ui| {
                            ui.add(
                                numeric::drag_value(&mut sound.shape_speed)
                                    .range(0.02..=10.0)
                                    .speed(0.01),
                            )
                            .on_hover_text(
                                "Oscillations per second of the lowest mode in the view; the \
                                 others oscillate faster in the ratio of their frequencies",
                            );
                            ui.label("Hz (lowest mode)");
                        });
                        ui.end_row();
                        ui.label("Frames per second");
                        ui.add(numeric::drag_value(&mut sound.shape_fps).range(1.0..=60.0));
                        ui.end_row();
                    }
                    ui.label("Volume");
                    actions.changed |= ui
                        .add(egui::Slider::new(&mut sound.volume, 0.0..=1.0).show_value(false))
                        .changed();
                    ui.end_row();
                });
            ui.separator();
            ui.horizontal(|ui| {
                let audible = !sound.tones().is_empty();
                let (icon, tip) = if playing {
                    (Icon::Pause, "Stop")
                } else {
                    (Icon::Animate, "Play")
                };
                if icons::button(ui, icon, tip, playing || audible, false).clicked() {
                    if playing {
                        actions.stop = true;
                    } else {
                        actions.play = true;
                    }
                }
                if sound.envelope == Envelope::Struck
                    && icons::button(ui, Icon::Sound, "Strike Again", audible, false).clicked()
                {
                    actions.play = true;
                }
                ui.add_space(8.0);
                if ui
                    .add_enabled(audible, egui::Button::new("Save as WAV …"))
                    .clicked()
                {
                    actions.export = true;
                }
                if !audible {
                    ui.weak("No audible mode selected");
                }
            });
            if let Some(message) = &sound.message {
                ui.colored_label(ui.visuals().error_fg_color, message);
            }
            if crate::dialog::tool_footer(ui, None) == crate::dialog::Tool::Close {
                actions.close = true;
            }
        });
    actions.close |= !open;
    actions
}

fn format_factor(octaves: i32) -> String {
    if octaves >= 0 {
        format!("{}", 1u64 << octaves.min(62))
    } else {
        format!("1/{}", 1u64 << (-octaves).min(62))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mode(step: u32, number: u32, kind: AnalysisKind, frequency: f64) -> Increment {
        Increment {
            nodal_diameter: None,
            step,
            increment: number,
            kind,
            value: frequency,
            fields: Vec::new(),
        }
    }

    fn modes(frequencies: &[f64]) -> Vec<Increment> {
        let mut increments = vec![mode(1, 1, AnalysisKind::Static, 1.0)];
        for (i, &f) in frequencies.iter().enumerate() {
            increments.push(mode(2, i as u32 + 1, AnalysisKind::Frequency, f));
        }
        increments
    }

    #[test]
    fn only_modes_open_the_window_with_the_shown_mode_enabled() {
        let increments = modes(&[100.0, 250.0, 400.0]);
        assert!(ModeSound::new(&increments, 0).is_none());
        let sound = ModeSound::new(&increments, 2).unwrap();
        assert_eq!(sound.step, 2);
        assert_eq!(sound.voices.len(), 3);
        let enabled: Vec<u32> = (sound.voices.iter())
            .filter(|v| v.enabled)
            .map(|v| v.mode)
            .collect();
        assert_eq!(enabled, [2]);
    }

    #[test]
    fn automatic_shift_keeps_audible_modes_and_moves_the_others_by_octaves() {
        assert_eq!(automatic_octaves(&[100.0, 3000.0]), 0);
        // 3 Hz is moved up six octaves to 192 Hz.
        assert_eq!(automatic_octaves(&[3.0, 7.5]), 6);
        // 50 kHz is moved down eight octaves to about 195 Hz.
        assert_eq!(automatic_octaves(&[50_000.0, 80_000.0]), -8);
        assert_eq!(automatic_octaves(&[]), 0);
        let low = 50_000.0 * 2f64.powi(-8);
        assert!((110.0..220.0).contains(&low));
    }

    #[test]
    fn combined_tones_keep_their_ratio_and_never_clip() {
        let increments = modes(&[3.0, 7.5, 1.0e6]);
        let mut sound = ModeSound::new(&increments, 1).unwrap();
        for voice in &mut sound.voices {
            voice.enabled = true;
        }
        sound.volume = 1.0;
        let tones = sound.tones();
        // The third mode is shifted far above hearing and left out.
        assert_eq!(tones.len(), 2);
        assert!((tones[1].frequency / tones[0].frequency - 2.5).abs() < 1e-9);
        let peak: f32 = tones.iter().map(|t| t.amplitude).sum();
        assert!(peak <= 1.0);
        sound.transpose = Transpose::Off;
        assert!(sound.tones().is_empty());
    }

    #[test]
    fn struck_sound_dies_away_and_higher_modes_faster() {
        let mut sound = ModeSound::new(&modes(&[200.0, 800.0]), 1).unwrap();
        sound.voices[1].enabled = true;
        sound.envelope = Envelope::Struck;
        let tones = sound.tones();
        assert!(tones[1].decay < tones[0].decay);
        let mut synth = Synth::default();
        synth.start(tones);
        let rate = 8000.0;
        let early: f32 = (0..800).map(|_| synth.next_sample(rate).abs()).sum();
        for _ in 0..40_000 {
            synth.next_sample(rate);
        }
        let late: f32 = (0..800).map(|_| synth.next_sample(rate).abs()).sum();
        assert!(late < early * 0.05);
    }

    #[test]
    fn synth_plays_a_sine_and_fades_out_after_stop() {
        let mut synth = Synth::default();
        let tone = Tone {
            frequency: 1000.0,
            amplitude: 0.5,
            decay: f32::INFINITY,
        };
        synth.start(vec![tone]);
        let rate = 48_000.0;
        let samples: Vec<f32> = (0..4800).map(|_| synth.next_sample(rate)).collect();
        let peak = samples[2400..].iter().fold(0.0f32, |m, s| m.max(s.abs()));
        assert!((peak - 0.5).abs() < 0.01);
        assert!(synth.sounding());
        synth.stop();
        for _ in 0..2000 {
            synth.next_sample(rate);
        }
        assert!(!synth.sounding());
        assert_eq!(synth.next_sample(rate), 0.0);
    }

    fn shape(
        step: u32,
        number: u32,
        frequency: f64,
        disp: [f32; 3],
        stress: [f32; 6],
    ) -> Increment {
        let component = |name: &str, value: f32| Component {
            name: name.into(),
            values: vec![value, 0.0],
            derived: false,
            out_of_plane: false,
        };
        let mut disp_field = Field::new(
            "DISP",
            ["U1", "U2", "U3"]
                .iter()
                .zip(disp)
                .map(|(n, v)| component(n, v))
                .collect(),
        );
        let mut stress_field = Field::new(
            "STRESS",
            ["SXX", "SYY", "SZZ", "SXY", "SYZ", "SZX"]
                .iter()
                .zip(stress)
                .map(|(n, v)| component(n, v))
                .collect(),
        );
        add_derived_components(&mut disp_field);
        add_derived_components(&mut stress_field);
        Increment {
            nodal_diameter: None,
            step,
            increment: number,
            kind: AnalysisKind::Frequency,
            value: frequency,
            fields: vec![disp_field, stress_field],
        }
    }

    #[test]
    fn overlay_scales_modes_to_the_same_size_and_by_level() {
        // Mode 2 has a ten times larger raw amplitude and twice the frequency.
        let increments = vec![
            shape(
                1,
                1,
                100.0,
                [1.0, 0.0, 0.0],
                [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ),
            shape(
                1,
                2,
                200.0,
                [0.0, 10.0, 0.0],
                [0.0, 100.0, 0.0, 0.0, 0.0, 0.0],
            ),
        ];
        let mut sound = ModeSound::new(&increments, 0).unwrap();
        sound.voices[1].enabled = true;
        sound.voices[1].level = 0.5;
        let mix = ShapeMix::new(&sound, &increments, "DISP", "U2", None).unwrap();
        // A quarter swing of the lowest mode: mode 1 at its peak, mode 2 (twice as fast) back
        // at zero.
        let frame = mix.frame(0.25, 0.0);
        assert!((frame.displacements[0][0] - 10.0).abs() < 1e-4);
        assert!(frame.displacements[0][1].abs() < 1e-4);
        // An eighth period: mode 2 at its peak, scaled to the size of the largest mode and
        // by level 0.5.
        let frame = mix.frame(0.125, 0.0);
        assert!((frame.displacements[0][1] - 5.0).abs() < 1e-4);
        assert_eq!(frame.values.unwrap()[0], frame.displacements[0][1]);
        assert_eq!(frame.modes, [1, 2]);
        assert_eq!(frame.range, Some((-5.0, 5.0)));
    }

    #[test]
    fn preview_swings_one_mode_at_full_size_without_dying_away() {
        let increments = vec![
            shape(
                1,
                1,
                100.0,
                [1.0, 0.0, 0.0],
                [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ),
            shape(
                1,
                2,
                200.0,
                [0.0, 10.0, 0.0],
                [0.0, 100.0, 0.0, 0.0, 0.0, 0.0],
            ),
        ];
        let mut sound = ModeSound::new(&increments, 0).unwrap();
        // Mode 2 is neither ticked nor loud, and the sound would die away.
        sound.voices[1].level = 0.2;
        sound.envelope = Envelope::Struck;
        let mix = ShapeMix::new(&sound, &increments, "DISP", "U2", Some(1)).unwrap();
        assert!(mix.shows("DISP", "U2", Some(1)));
        assert!(!mix.shows("DISP", "U2", None));
        let frame = mix.frame(0.25, 100.0);
        assert_eq!(frame.modes, [2]);
        assert!((frame.displacements[0][1] - 10.0).abs() < 1e-4);
        assert_eq!(frame.displacements[0][0], 0.0);
    }

    #[test]
    fn overlay_derives_equivalent_values_from_the_superposed_components() {
        let increments = vec![
            shape(
                1,
                1,
                100.0,
                [1.0, 0.0, 0.0],
                [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ),
            shape(
                1,
                2,
                100.0,
                [1.0, 0.0, 0.0],
                [-10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ),
        ];
        let mut sound = ModeSound::new(&increments, 0).unwrap();
        sound.voices[1].enabled = true;
        let mix = ShapeMix::new(&sound, &increments, "STRESS", "MISES", None).unwrap();
        // Same frequency, opposite stresses: they cancel, so the von Mises stress is zero
        // although each mode alone has 10.
        let frame = mix.frame(0.25, 0.0);
        assert!(frame.values.unwrap()[0].abs() < 1e-4);
        assert_eq!(frame.range, Some((0.0, 20.0)));
    }

    #[test]
    fn wav_has_a_valid_header() {
        let tone = Tone {
            frequency: 440.0,
            amplitude: 0.5,
            decay: f32::INFINITY,
        };
        let data = wav(vec![tone], 0.5);
        let samples = (WAV_RATE / 2) as usize;
        assert_eq!(data.len(), 44 + samples * 2);
        assert_eq!(&data[0..4], b"RIFF");
        assert_eq!(&data[8..16], b"WAVEfmt ");
        assert_eq!(
            u32::from_le_bytes(data[40..44].try_into().unwrap()),
            samples as u32 * 2
        );
        // The last sample is faded to silence.
        assert_eq!(
            i16::from_le_bytes([data[data.len() - 2], data[data.len() - 1]]),
            0
        );
    }
}
