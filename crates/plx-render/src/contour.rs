//! Colour spectra for result contours, following the classic legend: the control points of
//! each spectrum, lightened by the brightness and interpolated in the colour space VTK uses
//! for it. The default is a discrete rainbow from blue (minimum) to red (maximum), lightened
//! by 20 %.

use std::f64::consts::PI;

pub use plx_model::color_scale::{DEFAULT_LEVELS, MAX_LEVELS};
use plx_model::{ColorScale, ColorSpectrum};

/// Colours the shader can hold: the bands plus one below and one above the range.
pub const PALETTE_SLOTS: u32 = MAX_LEVELS + 2;
/// Normalized value written for nodes without a result; drawn grey.
pub const NO_VALUE: f32 = -1.0;

/// How VTK interpolates between the control points of a spectrum.
#[derive(Clone, Copy)]
enum Space {
    Rgb,
    Hsv,
    Lab,
    /// Moreland's diverging colour map through white, in Msh space.
    Diverging,
}

/// Interpolation space and control points (position, r, g, b) of a spectrum.
fn control_points(spectrum: ColorSpectrum) -> (Space, &'static [[f64; 4]]) {
    match spectrum {
        ColorSpectrum::CoolWarm => (
            Space::Diverging,
            &[[0.0, 0.230, 0.299, 0.754], [1.0, 0.706, 0.016, 0.150]],
        ),
        ColorSpectrum::Rainbow => (
            Space::Hsv,
            &[
                [0.00, 0.0, 0.0, 1.0],
                [0.25, 0.0, 1.0, 1.0],
                [0.50, 0.0, 1.0, 0.0],
                [0.75, 1.0, 1.0, 0.0],
                [1.00, 1.0, 0.0, 0.0],
            ],
        ),
        ColorSpectrum::RainbowDesaturated => (
            Space::Rgb,
            &[
                [0.0, 0.278, 0.278, 0.788],
                [0.143, 0.02, 0.02, 0.396],
                [0.286, 0.0, 0.871, 0.918],
                [0.429, 0.0, 0.604, 0.2],
                [0.571, 1.0, 0.875, 0.0],
                [0.714, 1.0, 0.329, 0.0],
                [0.857, 0.451, 0.02, 0.02],
                [1.0, 0.878, 0.302, 0.302],
            ],
        ),
        ColorSpectrum::Jet => (
            Space::Rgb,
            &[
                [0.000, 0.0, 0.0, 0.5],
                [0.125, 0.0, 0.0, 1.0],
                [0.375, 0.0, 1.0, 1.0],
                [0.500, 0.0, 1.0, 0.0],
                [0.625, 1.0, 1.0, 0.0],
                [0.875, 1.0, 0.0, 0.0],
                [1.000, 0.5, 0.0, 0.0],
            ],
        ),
        ColorSpectrum::Turbo => (
            Space::Rgb,
            &[
                [0.000, 0.18995, 0.07176, 0.23217],
                [0.067, 0.25107, 0.25237, 0.63374],
                [0.133, 0.27698, 0.46153, 0.93309],
                [0.200, 0.24427, 0.60937, 0.99697],
                [0.267, 0.17223, 0.71680, 0.93981],
                [0.333, 0.10342, 0.89600, 0.71500],
                [0.400, 0.26180, 0.96765, 0.52981],
                [0.467, 0.47375, 0.99755, 0.34963],
                [0.533, 0.69525, 0.97610, 0.21294],
                [0.600, 0.88331, 0.86553, 0.21719],
                [0.667, 0.98000, 0.73000, 0.22161],
                [0.733, 0.99297, 0.55214, 0.15417],
                [0.800, 0.94084, 0.35566, 0.07031],
                [0.867, 0.84662, 0.21407, 0.02487],
                [0.933, 0.68602, 0.09536, 0.00481],
                [1.000, 0.47960, 0.01583, 0.01055],
            ],
        ),
        ColorSpectrum::Warm => (
            Space::Rgb,
            &[
                [0.0, 0.549, 0.176, 0.016],
                [0.143, 0.8, 0.298, 0.008],
                [0.286, 0.925, 0.439, 0.078],
                [0.429, 0.996, 0.6, 0.161],
                [0.571, 0.996, 0.769, 0.31],
                [0.714, 0.996, 0.89, 0.569],
                [0.857, 1.0, 0.969, 0.737],
                [1.0, 1.0, 1.0, 0.898],
            ],
        ),
        ColorSpectrum::Cool => (
            Space::Rgb,
            &[
                [0.0, 0.031, 0.271, 0.58],
                [0.143, 0.129, 0.443, 0.71],
                [0.286, 0.259, 0.573, 0.776],
                [0.429, 0.42, 0.682, 0.839],
                [0.571, 0.62, 0.792, 0.882],
                [0.714, 0.776, 0.859, 0.937],
                [0.857, 0.871, 0.922, 0.969],
                [1.0, 0.969, 0.984, 1.0],
            ],
        ),
        ColorSpectrum::Cividis => (
            Space::Rgb,
            &[
                [0.0, 0.0, 0.125, 0.297],
                [0.125, 0.023, 0.211, 0.43],
                [0.25, 0.254, 0.301, 0.418],
                [0.375, 0.375, 0.391, 0.43],
                [0.5, 0.484, 0.48, 0.469],
                [0.625, 0.609, 0.578, 0.465],
                [0.75, 0.738, 0.684, 0.43],
                [0.875, 0.875, 0.793, 0.363],
                [1.0, 0.996, 0.91, 0.27],
            ],
        ),
        ColorSpectrum::Viridis => (
            Space::Rgb,
            &[
                [0.0, 0.267, 0.005, 0.329],
                [0.143, 0.275, 0.197, 0.497],
                [0.286, 0.213, 0.359, 0.552],
                [0.429, 0.153, 0.498, 0.558],
                [0.571, 0.122, 0.632, 0.531],
                [0.714, 0.29, 0.759, 0.428],
                [0.857, 0.622, 0.854, 0.226],
                [1.0, 0.993, 0.906, 0.144],
            ],
        ),
        ColorSpectrum::Plasma => (
            Space::Rgb,
            &[
                [0.0, 0.05, 0.03, 0.528],
                [0.143, 0.328, 0.007, 0.64],
                [0.286, 0.545, 0.038, 0.647],
                [0.429, 0.725, 0.197, 0.538],
                [0.571, 0.859, 0.359, 0.408],
                [0.714, 0.956, 0.534, 0.285],
                [0.857, 0.995, 0.738, 0.167],
                [1.0, 0.94, 0.975, 0.131],
            ],
        ),
        ColorSpectrum::BlackBody => (
            Space::Lab,
            &[
                [0.0, 0.0, 0.0, 0.0],
                [0.143, 0.257, 0.089, 0.069],
                [0.286, 0.502, 0.123, 0.107],
                [0.429, 0.735, 0.198, 0.124],
                [0.571, 0.877, 0.395, 0.038],
                [0.714, 0.911, 0.632, 0.1],
                [0.857, 0.907, 0.855, 0.189],
                [1.0, 1.0, 1.0, 1.0],
            ],
        ),
        ColorSpectrum::Inferno => (
            Space::Lab,
            &[
                [0.0, 0.001, 0.0, 0.014],
                [0.143, 0.159, 0.044, 0.329],
                [0.286, 0.397, 0.083, 0.433],
                [0.429, 0.623, 0.165, 0.388],
                [0.571, 0.831, 0.283, 0.259],
                [0.714, 0.962, 0.49, 0.084],
                [0.857, 0.982, 0.756, 0.153],
                [1.0, 0.988, 0.998, 0.645],
            ],
        ),
        ColorSpectrum::Kindlmann => (
            Space::Lab,
            &[
                [0.0, 0.0, 0.0, 0.0],
                [0.143, 0.14, 0.022, 0.46],
                [0.286, 0.028, 0.243, 0.587],
                [0.429, 0.021, 0.449, 0.382],
                [0.571, 0.031, 0.625, 0.083],
                [0.714, 0.44, 0.767, 0.037],
                [0.857, 0.98, 0.815, 0.572],
                [1.0, 1.0, 1.0, 1.0],
            ],
        ),
        ColorSpectrum::Grayscale => (Space::Lab, &[[0.0, 0.2, 0.2, 0.2], [1.0, 1.0, 1.0, 1.0]]),
    }
}

/// sRGB colour of the scale's spectrum at `t` (0 at the minimum, 1 at the maximum).
pub fn spectrum_color(scale: &ColorScale, t: f64) -> [f32; 3] {
    let (space, points) = control_points(scale.spectrum);
    let x = scale.brightness.clamp(0.0, 1.0);
    let lighten = |p: &[f64; 4]| [1, 2, 3].map(|k| p[k] + x * (1.0 - p[k]));
    let t = t.clamp(0.0, 1.0);
    let t = if scale.reverse { 1.0 - t } else { t };
    let next = points
        .iter()
        .position(|p| p[0] >= t)
        .unwrap_or(points.len() - 1);
    let rgb = if next == 0 {
        lighten(&points[0])
    } else {
        let (a, b) = (&points[next - 1], &points[next]);
        let s = if b[0] > a[0] {
            (t - a[0]) / (b[0] - a[0])
        } else {
            1.0
        };
        interpolate(space, lighten(a), lighten(b), s)
    };
    rgb.map(|c| c.clamp(0.0, 1.0) as f32)
}

/// sRGB colours of the scale's bands, minimum first: the spectrum sampled evenly from end
/// to end.
pub fn band_colors(scale: &ColorScale) -> Vec<[f32; 3]> {
    let levels = scale.levels.clamp(2, MAX_LEVELS);
    (0..levels)
        .map(|i| spectrum_color(scale, i as f64 / (levels - 1) as f64))
        .collect()
}

/// sRGB colour of an 8 bit colour.
pub fn rgb(color: [u8; 3]) -> [f32; 3] {
    color.map(|c| c as f32 / 255.0)
}

/// Maps result values to the normalized range used for colouring: 0 at `min`, 1 at `max`,
/// [`NO_VALUE`] where there is no finite value.
pub fn normalize(values: &[f32], min: f32, max: f32) -> Vec<f32> {
    let span = max - min;
    values
        .iter()
        .map(|&v| {
            if !v.is_finite() {
                NO_VALUE
            } else if span > 0.0 {
                ((v - min) / span).clamp(0.0, 1.0)
            } else {
                0.5
            }
        })
        .collect()
}

fn interpolate(space: Space, a: [f64; 3], b: [f64; 3], s: f64) -> [f64; 3] {
    let lerp = |a: [f64; 3], b: [f64; 3], s: f64| [0, 1, 2].map(|k| a[k] + s * (b[k] - a[k]));
    match space {
        Space::Rgb => lerp(a, b, s),
        Space::Hsv => {
            let (mut ha, mut hb) = (rgb_to_hsv(a), rgb_to_hsv(b));
            // The shorter way round the hue circle, as VTK's HSVWrap.
            if hb[0] - ha[0] > 0.5 {
                ha[0] += 1.0;
            } else if ha[0] - hb[0] > 0.5 {
                hb[0] += 1.0;
            }
            let mut hsv = lerp(ha, hb, s);
            hsv[0] = hsv[0].rem_euclid(1.0);
            hsv_to_rgb(hsv)
        }
        Space::Lab => lab_to_rgb(lerp(rgb_to_lab(a), rgb_to_lab(b), s)),
        Space::Diverging => diverging(a, b, s),
    }
}

/// VTK's diverging interpolation (Moreland 2009): through white between two saturated
/// colours of different hue.
fn diverging(a: [f64; 3], b: [f64; 3], mut s: f64) -> [f64; 3] {
    let mut m1 = lab_to_msh(rgb_to_lab(a));
    let mut m2 = lab_to_msh(rgb_to_lab(b));
    if m1[1] > 0.05 && m2[1] > 0.05 && angle_difference(m1[2], m2[2]) > 0.33 * PI {
        let middle = m1[0].max(m2[0]).max(88.0);
        if s < 0.5 {
            m2 = [middle, 0.0, 0.0];
            s *= 2.0;
        } else {
            m1 = [middle, 0.0, 0.0];
            s = 2.0 * s - 1.0;
        }
    }
    if m1[1] < 0.05 && m2[1] > 0.05 {
        m1[2] = adjust_hue(m2, m1[0]);
    } else if m2[1] < 0.05 && m1[1] > 0.05 {
        m2[2] = adjust_hue(m1, m2[0]);
    }
    let msh = [0, 1, 2].map(|k| (1.0 - s) * m1[k] + s * m2[k]);
    lab_to_rgb(msh_to_lab(msh))
}

fn angle_difference(a: f64, b: f64) -> f64 {
    let d = (a - b).abs();
    if d > PI { 2.0 * PI - d } else { d }
}

/// Hue of an unsaturated colour of magnitude `m` next to the saturated `msh`.
fn adjust_hue(msh: [f64; 3], m: f64) -> f64 {
    if msh[0] >= m - 0.1 {
        return msh[2];
    }
    let spin = msh[1] * (m * m - msh[0] * msh[0]).sqrt() / (msh[0] * msh[1].sin());
    if msh[2] > -0.3 * PI {
        msh[2] + spin
    } else {
        msh[2] - spin
    }
}

fn lab_to_msh([l, a, b]: [f64; 3]) -> [f64; 3] {
    let m = (l * l + a * a + b * b).sqrt();
    let s = if m > 0.001 { (l / m).acos() } else { 0.0 };
    let h = if s > 0.001 { b.atan2(a) } else { 0.0 };
    [m, s, h]
}

fn msh_to_lab([m, s, h]: [f64; 3]) -> [f64; 3] {
    [m * s.cos(), m * s.sin() * h.cos(), m * s.sin() * h.sin()]
}

/// D65 white of VTK's Lab conversion.
const WHITE: [f64; 3] = [0.9505, 1.0, 1.089];

fn rgb_to_lab(rgb: [f64; 3]) -> [f64; 3] {
    let [r, g, b] = rgb.map(|c| {
        if c > 0.04045 {
            ((c + 0.055) / 1.055).powf(2.4)
        } else {
            c / 12.92
        }
    });
    let xyz = [
        r * 0.4124 + g * 0.3576 + b * 0.1805,
        r * 0.2126 + g * 0.7152 + b * 0.0722,
        r * 0.0193 + g * 0.1192 + b * 0.9505,
    ];
    let [fx, fy, fz] = [0, 1, 2].map(|k| {
        let t = xyz[k] / WHITE[k];
        if t > 0.008856 {
            t.cbrt()
        } else {
            7.787 * t + 16.0 / 116.0
        }
    });
    [116.0 * fy - 16.0, 500.0 * (fx - fy), 200.0 * (fy - fz)]
}

fn lab_to_rgb([l, a, b]: [f64; 3]) -> [f64; 3] {
    let fy = (l + 16.0) / 116.0;
    let f = [fy + a / 500.0, fy, fy - b / 200.0];
    let [x, y, z] = [0, 1, 2].map(|k| {
        let cube = f[k].powi(3);
        let t = if cube > 0.008856 {
            cube
        } else {
            (f[k] - 16.0 / 116.0) / 7.787
        };
        t * WHITE[k]
    });
    [
        x * 3.2406 + y * -1.5372 + z * -0.4986,
        x * -0.9689 + y * 1.8758 + z * 0.0415,
        x * 0.0557 + y * -0.2040 + z * 1.0570,
    ]
    .map(|c| {
        let c = if c > 0.0031308 {
            1.055 * c.powf(1.0 / 2.4) - 0.055
        } else {
            12.92 * c
        };
        c.clamp(0.0, 1.0)
    })
}

/// Hue (0 ... 1), saturation and value.
fn rgb_to_hsv([r, g, b]: [f64; 3]) -> [f64; 3] {
    let max = r.max(g).max(b);
    let min = r.min(g).min(b);
    let delta = max - min;
    let saturation = if max > 0.0 { delta / max } else { 0.0 };
    let hue = if delta <= 0.0 {
        0.0
    } else if max == r {
        ((g - b) / delta).rem_euclid(6.0) / 6.0
    } else if max == g {
        ((b - r) / delta + 2.0) / 6.0
    } else {
        ((r - g) / delta + 4.0) / 6.0
    };
    [hue, saturation, max]
}

fn hsv_to_rgb([hue, saturation, value]: [f64; 3]) -> [f64; 3] {
    let h = hue.rem_euclid(1.0) * 6.0;
    let c = value * saturation;
    let x = c * (1.0 - (h % 2.0 - 1.0).abs());
    let (r, g, b) = match h as u32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    let m = value - c;
    [r + m, g + m, b + m]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: [f32; 3], b: [f32; 3]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 2e-3)
    }

    #[test]
    fn rainbow_runs_from_light_blue_to_light_red() {
        let scale = ColorScale::default();
        let bands = band_colors(&scale);
        assert_eq!(bands.len(), 9);
        assert!(close(bands[0], [0.2, 0.2, 1.0]));
        assert!(close(bands[4], [0.2, 1.0, 0.2]));
        assert!(close(bands[8], [1.0, 0.2, 0.2]));
        let five = band_colors(&ColorScale {
            levels: 5,
            ..ColorScale::default()
        });
        assert!(close(five[2], [0.2, 1.0, 0.2]));
        let reversed = band_colors(&ColorScale {
            reverse: true,
            ..ColorScale::default()
        });
        assert!(close(reversed[0], [1.0, 0.2, 0.2]));
    }

    #[test]
    fn spectra_hit_their_control_points() {
        let scale = |spectrum| ColorScale {
            spectrum,
            brightness: 0.0,
            ..ColorScale::default()
        };
        let viridis = scale(ColorSpectrum::Viridis);
        assert!(close(spectrum_color(&viridis, 0.0), [0.267, 0.005, 0.329]));
        assert!(close(spectrum_color(&viridis, 1.0), [0.993, 0.906, 0.144]));
        let gray = scale(ColorSpectrum::Grayscale);
        assert!(close(spectrum_color(&gray, 1.0), [1.0, 1.0, 1.0]));
        assert!(close(spectrum_color(&gray, 0.0), [0.2, 0.2, 0.2]));
        // Cool-warm passes through a light grey in the middle.
        let [r, g, b] = spectrum_color(&scale(ColorSpectrum::CoolWarm), 0.5);
        assert!(
            r > 0.8 && g > 0.8 && b > 0.8 && (r - b).abs() < 0.1,
            "{r} {g} {b}"
        );
        assert!(close(
            spectrum_color(&scale(ColorSpectrum::CoolWarm), 0.0),
            [0.230, 0.299, 0.754]
        ));
    }

    #[test]
    fn lab_conversion_round_trips() {
        for rgb in [
            [0.2, 0.4, 0.6],
            [1.0, 1.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.9, 0.1, 0.3],
        ] {
            let back = lab_to_rgb(rgb_to_lab(rgb));
            assert!(
                (0..3).all(|k| (back[k] - rgb[k]).abs() < 1e-3),
                "{rgb:?} {back:?}"
            );
        }
    }

    #[test]
    fn normalization_marks_missing_values() {
        assert_eq!(
            normalize(&[0.0, 5.0, 10.0, f32::NAN], 0.0, 10.0),
            [0.0, 0.5, 1.0, NO_VALUE]
        );
        assert_eq!(normalize(&[3.0], 3.0, 3.0), [0.5]);
    }
}
