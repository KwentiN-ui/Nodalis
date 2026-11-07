//! What a Dynamic step derives from the user's settings: the time step, how often a result
//! frame is written, and the Rayleigh coefficients of a damping ratio.

use crate::{
    Amplitude, AmplitudeTime, Damping, DampingRange, DynamicStep, FeModel, RayleighDamping, Step,
    TimeStep,
};

/// How a Dynamic step is integrated: `increment` long time steps, of which every
/// `per_frame`-th is a result frame, so the frames fall exactly on the time steps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Integration {
    pub increment: f64,
    pub per_frame: u32,
    /// Which bound made the time step this short.
    pub limited_by: StepLimit,
}

/// Which bound sets the time step of a Dynamic step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepLimit {
    /// The interval between two result frames.
    Frames,
    /// A tenth of the shortest segment of an amplitude the loads follow.
    Amplitude,
    /// `1 / (20 f)` of the frequency limit.
    Frequency,
    /// The time step the user fixed.
    Fixed,
}

impl Integration {
    /// The total number of time steps of the step.
    pub fn increments(&self, settings: &DynamicStep) -> u64 {
        settings.frames.max(1) as u64 * self.per_frame as u64
    }

    /// The highest frequency the time step resolves well (ten steps per period), which is
    /// also the upper end of an automatic damping range.
    pub fn resolved_frequency(&self) -> f64 {
        1.0 / (10.0 * self.increment)
    }
}

impl DynamicStep {
    /// The time step and the time steps per frame. `amplitude_interval` is the shortest
    /// segment of the amplitudes the step's loads follow, see
    /// [`FeModel::amplitude_interval`].
    pub fn integration(&self, amplitude_interval: Option<f64>) -> Integration {
        let frame = self.frame_interval();
        let (wanted, limited_by) = match self.time_step {
            TimeStep::Fixed(dt) if dt > 0.0 => (dt, StepLimit::Fixed),
            _ => {
                let mut wanted = (frame, StepLimit::Frames);
                let amplitude = amplitude_interval.filter(|t| *t > 0.0).map(|t| t / 10.0);
                let frequency = (self.frequency_limit)
                    .filter(|f| *f > 0.0)
                    .map(|f| 1.0 / (20.0 * f));
                for (dt, limit) in [
                    (amplitude, StepLimit::Amplitude),
                    (frequency, StepLimit::Frequency),
                ] {
                    if let Some(dt) = dt
                        && dt < wanted.0
                    {
                        wanted = (dt, limit);
                    }
                }
                wanted
            }
        };
        // Whole time steps per frame; a hair of tolerance so that 1e-3 / 1e-4 stays 10.
        let per_frame = ((frame / wanted - 1e-9).ceil().max(1.0)).min(u32::MAX as f64) as u32;
        Integration {
            increment: frame / per_frame as f64,
            per_frame,
            limited_by: if per_frame == 1 && limited_by != StepLimit::Fixed {
                StepLimit::Frames
            } else {
                limited_by
            },
        }
    }

    /// The Rayleigh coefficients of the damping, for a step integrated with `increment`
    /// long time steps; `None` undamped. A ratio over the automatic range needs the
    /// lowest eigenfrequency, which only the solver knows: it is returned as
    /// [`RayleighFit::FromRatio`].
    pub fn rayleigh(&self, increment: f64) -> Option<RayleighFit> {
        match self.damping {
            Damping::None => None,
            Damping::Rayleigh(r) => Some(RayleighFit::Coefficients(r)),
            Damping::Ratio { zeta, range } => Some(match range {
                DampingRange::Between(f1, f2) => {
                    RayleighFit::Coefficients(RayleighDamping::from_ratio(zeta, f1, f2))
                }
                DampingRange::Automatic => RayleighFit::FromRatio {
                    zeta,
                    upper: 1.0 / (10.0 * increment),
                },
            }),
        }
    }
}

/// The damping of a Dynamic step as the solver takes it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RayleighFit {
    Coefficients(RayleighDamping),
    /// A damping ratio met exactly at the lowest eigenfrequency, which the solver computes,
    /// and at `upper`.
    FromRatio {
        zeta: f64,
        upper: f64,
    },
}

impl RayleighDamping {
    /// The coefficients with the damping ratio `zeta` exactly at the frequencies `f1` and
    /// `f2` (cycles per time unit) and less between them:
    /// `alpha = 2 zeta w1 w2 / (w1 + w2)`, `beta = 2 zeta / (w1 + w2)`.
    pub fn from_ratio(zeta: f64, f1: f64, f2: f64) -> Self {
        let (w1, w2) = (
            2.0 * std::f64::consts::PI * f1,
            2.0 * std::f64::consts::PI * f2,
        );
        Self {
            alpha: 2.0 * zeta * w1 * w2 / (w1 + w2),
            beta: 2.0 * zeta / (w1 + w2),
        }
    }
}

impl FeModel {
    /// The amplitudes the active loads, boundary conditions and defined fields of a step
    /// follow.
    pub fn step_amplitudes<'a>(&'a self, step: &'a Step) -> Vec<&'a Amplitude> {
        let bcs = (step.boundary_conditions.iter())
            .filter(|b| b.active && b.kind.takes_amplitude())
            .map(|b| &b.amplitude);
        let loads = (step.loads.iter().filter(|l| l.active))
            .flat_map(|l| [&l.amplitude, &l.factor_amplitude]);
        let fields = (step.defined_fields.iter().filter(|f| f.active)).map(|f| &f.amplitude);
        let mut used: Vec<&Amplitude> = Vec::new();
        for name in bcs.chain(loads).chain(fields).flatten() {
            if let Some(amplitude) = self.amplitudes.iter().find(|a| &a.name == name)
                && !used.iter().any(|a| a.name == amplitude.name)
            {
                used.push(amplitude);
            }
        }
        used
    }

    /// The shortest segment between two points of the amplitudes a step follows, within
    /// the step: what the time step must resolve so the loads are applied as given.
    /// Amplitudes in total time count with all their segments. `None` without amplitudes.
    pub fn amplitude_interval(&self, step: &Step, duration: f64) -> Option<f64> {
        let mut shortest: Option<f64> = None;
        for amplitude in self.step_amplitudes(step) {
            for pair in amplitude.points.windows(2) {
                let (t0, t1) = (
                    pair[0][0] + amplitude.shift_time,
                    pair[1][0] + amplitude.shift_time,
                );
                let gap = t1 - t0;
                let inside =
                    amplitude.time_span == AmplitudeTime::Total || (t1 > 0.0 && t0 < duration);
                if gap > 0.0 && inside {
                    shortest = Some(shortest.map_or(gap, |s| s.min(gap)));
                }
            }
        }
        shortest
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frames_fall_on_time_steps() {
        let settings = DynamicStep::default();
        let integration = settings.integration(None);
        assert_eq!(integration.per_frame, 1);
        assert!((integration.increment - 0.005).abs() < 1e-15);
        assert_eq!(integration.limited_by, StepLimit::Frames);
        // A 2 ms rise time: steps of at most 0.2 ms, 25 per 5 ms frame.
        let integration = settings.integration(Some(0.002));
        assert_eq!(integration.per_frame, 25);
        assert!((integration.increment - 0.0002).abs() < 1e-15);
        assert_eq!(integration.limited_by, StepLimit::Amplitude);
        assert_eq!(integration.increments(&settings), 5000);
    }

    #[test]
    fn the_frequency_limit_and_a_fixed_step_shorten_the_time_step() {
        let settings = DynamicStep {
            duration: 0.1,
            frames: 10,
            frequency_limit: Some(300.0),
            ..DynamicStep::default()
        };
        // 1 / (20 * 300) = 0.1667 ms: 60 per 10 ms frame.
        let integration = settings.integration(None);
        assert_eq!(integration.per_frame, 60);
        assert_eq!(integration.limited_by, StepLimit::Frequency);
        // A fixed step is shortened to hit the frames: 3 ms -> 4 steps of 2.5 ms.
        let settings = DynamicStep {
            time_step: TimeStep::Fixed(0.003),
            ..settings
        };
        let integration = settings.integration(Some(1e-6));
        assert_eq!(integration.per_frame, 4);
        assert!((integration.increment - 0.0025).abs() < 1e-15);
        assert_eq!(integration.limited_by, StepLimit::Fixed);
    }

    #[test]
    fn a_ratio_is_met_at_both_frequencies() {
        let r = RayleighDamping::from_ratio(0.02, 10.0, 1000.0);
        let ratio = |f: f64| {
            let w = 2.0 * std::f64::consts::PI * f;
            r.alpha / (2.0 * w) + r.beta * w / 2.0
        };
        assert!((ratio(10.0) - 0.02).abs() < 1e-12);
        assert!((ratio(1000.0) - 0.02).abs() < 1e-12);
        assert!(ratio(100.0) < 0.02);
    }

    #[test]
    fn the_shortest_amplitude_segment_inside_the_step_counts() {
        let mut model = FeModel::default();
        let mut amplitude = Amplitude::new("Pulse");
        amplitude.points = vec![
            [0.0, 0.0],
            [0.001, 1.0],
            [0.01, 0.0],
            [5.0, 0.0],
            [5.0001, 1.0],
        ];
        model.amplitudes.push(amplitude);
        let mut step = Step::new_dynamic("Dyn");
        assert_eq!(model.amplitude_interval(&step, 1.0), None);
        step.loads.push(crate::Load {
            name: "F".into(),
            active: true,
            visible: true,
            region: crate::Region::Nodes(Vec::new()),
            kind: crate::LoadKind::Pressure(1.0),
            amplitude: Some("Pulse".into()),
            factor_amplitude: None,
            distribution: None,
        });
        // The segment at 5 s lies behind the step.
        assert_eq!(model.amplitude_interval(&step, 1.0), Some(0.001));
    }
}

/// A rough stable time step of explicit integration, from the mesh and the materials
/// (the solver computes the exact one from the element eigenvalues): the characteristic
/// length of the elements (see [`characteristic_length`]), or the node spacing through a
/// shell's thickness (expanded shells have their nodes there), divided by the
/// dilatational wave speed, with the solver's safety factor.
#[derive(Clone, Debug, PartialEq)]
pub struct StableStepEstimate {
    pub time_step: f64,
    /// The section whose elements limit it.
    pub section: String,
    /// The shell thickness when the spacing through it limits the step.
    pub thickness: Option<f64>,
}

impl FeModel {
    /// See [`StableStepEstimate`]; `None` without sections that have an elastic material
    /// with a density.
    pub fn explicit_time_step_estimate(
        &self,
        mesh: &plx_mesh::FeMesh,
    ) -> Option<StableStepEstimate> {
        use crate::SectionKind;
        use plx_mesh::ElementShape;
        let mut best: Option<StableStepEstimate> = None;
        for section in &self.sections {
            let Some(material) = self.materials.iter().find(|m| m.name == section.material) else {
                continue;
            };
            let (Some(rho), Some(elastic)) = (material.density, material.elastic.as_ref()) else {
                continue;
            };
            let nu = elastic.poisson.clamp(-0.99, 0.499);
            let modulus = elastic.young * (1.0 - nu) / ((1.0 + nu) * (1.0 - 2.0 * nu));
            if !(rho > 0.0 && modulus > 0.0) {
                continue;
            }
            let speed = (modulus / rho).sqrt();
            let mut shortest = f64::INFINITY;
            let mut through_thickness = None;
            for id in section.region.elements(mesh) {
                let Some(element) = mesh.element(id) else {
                    continue;
                };
                let coords: Vec<[f64; 3]> =
                    element.nodes.iter().filter_map(|&n| mesh.node(n)).collect();
                if let Some(length) = characteristic_length(element.shape, &coords)
                    && length < shortest
                {
                    shortest = length;
                    through_thickness = None;
                }
                if let SectionKind::Shell { thickness, .. } = section.kind {
                    let quadratic =
                        matches!(element.shape, ElementShape::Tri6 | ElementShape::Quad8);
                    let spacing = if quadratic {
                        0.41 * thickness
                    } else {
                        thickness
                    };
                    if spacing > 0.0 && spacing < shortest {
                        shortest = spacing;
                        through_thickness = Some(thickness);
                    }
                }
            }
            if !shortest.is_finite() {
                continue;
            }
            let time_step = SAFETY * shortest / speed;
            if best.as_ref().is_none_or(|b| time_step < b.time_step) {
                best = Some(StableStepEstimate {
                    time_step,
                    section: section.name.clone(),
                    thickness: through_thickness,
                });
            }
        }
        best
    }
}

/// The fraction of the stable time step the solver takes (with its bulk viscosity).
const SAFETY: f64 = 0.85;

/// The length a wave crosses in an element's stable time step: the smallest altitude of a
/// tetrahedron or triangle, the shortest edge of other shapes (both from the corners), and
/// for quadratic elements 0.41 of it (the midside nodes carry two thirds of the lumped
/// mass of an edge: a quadratic bar element of length `L` has `Δt = 0.41 L/c`); 0.3 for
/// the ten-node tetrahedron, whose corners keep very little of the lumped mass (checked
/// against the solver's element eigenvalues on a C3D10 cantilever).
pub fn characteristic_length(shape: plx_mesh::ElementShape, coords: &[[f64; 3]]) -> Option<f64> {
    use plx_mesh::ElementShape::*;
    let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let norm = |a: [f64; 3]| (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    let (corners, quadratic) = match shape {
        Line2 => (2, false),
        Line3 => (2, true),
        Tri3 => (3, false),
        Tri6 => (3, true),
        Quad4 => (4, false),
        Quad8 => (4, true),
        Tet4 => (4, false),
        Tet10 => (4, true),
        Wedge6 => (6, false),
        Wedge15 => (6, true),
        Hex8 => (8, false),
        Hex20 => (8, true),
    };
    if coords.len() < corners {
        return None;
    }
    let c = &coords[..corners];
    let length = match shape {
        Tet4 | Tet10 => {
            let volume = (sub(c[1], c[0])
                .iter()
                .zip(cross(sub(c[2], c[0]), sub(c[3], c[0])))
                .map(|(a, b)| a * b)
                .sum::<f64>()
                / 6.0)
                .abs();
            let faces = [[1, 2, 3], [0, 2, 3], [0, 1, 3], [0, 1, 2]];
            let largest = faces
                .iter()
                .map(|f| 0.5 * norm(cross(sub(c[f[1]], c[f[0]]), sub(c[f[2]], c[f[0]]))))
                .fold(0.0_f64, f64::max);
            (largest > 0.0).then(|| 3.0 * volume / largest)?
        }
        Tri3 | Tri6 => {
            let area = 0.5 * norm(cross(sub(c[1], c[0]), sub(c[2], c[0])));
            let longest = (0..3)
                .map(|i| norm(sub(c[(i + 1) % 3], c[i])))
                .fold(0.0_f64, f64::max);
            (longest > 0.0).then(|| 2.0 * area / longest)?
        }
        _ => {
            let mut shortest = f64::INFINITY;
            for (i, a) in c.iter().enumerate() {
                for b in &c[i + 1..] {
                    let d = norm(sub(*a, *b));
                    if d > 0.0 {
                        shortest = shortest.min(d);
                    }
                }
            }
            shortest.is_finite().then_some(shortest)?
        }
    };
    Some(match shape {
        Tet10 => 0.3 * length,
        _ if quadratic => 0.41 * length,
        _ => length,
    })
}
