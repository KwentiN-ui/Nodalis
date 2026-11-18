//! Granular material after the μ(I) rheology: against the analytical flow down a rough
//! incline (Bagnold's profile of GDR MiDi 2004), rest below the angle of friction, and the
//! measured runout of collapsing columns (Lube et al. 2005). The finer versions run with
//! `--ignored`.
//!
//! Plane cases are computed as plane strain slabs one cell thick, periodic in z.

use plxs_core::particles::{
    Domain, MaterialBody, ParticleControl, ParticleFrame, ParticleLaw, ParticleMaterial,
    ParticleModel, ParticleResults, Side, Space, TriangleSurface, WallLaw,
};

const G: f64 = 9.81;
/// Glass beads (Jop, Forterre & Pouliquen 2006: 20.9° and 32.8°, I0 0.279 with the grain
/// density; here with the bulk density, so I0 is scaled by √0.6).
const FRICTION: f64 = 0.382;
const FRICTION_LIMIT: f64 = 0.644;
const INERTIAL: f64 = 0.216;
const DENSITY: f64 = 1500.0;

fn granular(name: &str, friction: f64, friction_limit: f64, grain: f64) -> ParticleMaterial {
    ParticleMaterial {
        name: name.into(),
        density: DENSITY,
        law: ParticleLaw::Granular {
            shear: 4e5,
            bulk: 1e6,
            friction,
            friction_limit,
            grain,
            inertial: INERTIAL,
            fine_grain: 0.0,
            fine_fraction: 0.0,
        },
        thermal: None,
        flow_tables: Default::default(),
    }
}

fn beads(grain: f64) -> ParticleMaterial {
    granular("Glass beads", FRICTION, FRICTION_LIMIT, grain)
}

/// Quartz sand as in the experiments of Lube et al. (2005): an angle of repose of 31°, the
/// limit of the friction taken at 42°.
fn sand() -> ParticleMaterial {
    granular("Sand", 0.6, 0.9, 1e-3)
}

fn solve(model: &ParticleModel) -> ParticleResults {
    // On a thread of the pool, as Nodalis runs it: from outside, every parallel loop of a
    // time step hands its work to the pool and waits, which costs more than the step of a
    // small model.
    let results = rayon::scope(|_| plxs_particles::solve(model, &mut |_| {})).expect("solves");
    eprintln!(
        "{}: {} particles, {} steps, {:.1} s, factor {:.1}",
        model.heading,
        results.report.particles,
        results.report.steps,
        results.report.seconds,
        results.report.sound_speed_factor,
    );
    for n in &results.report.notes {
        eprintln!("  {n}");
    }
    results
}

/// A plane model in x and y: a slab one cell thick, periodic in z.
fn plane(
    heading: &str,
    min: [f64; 2],
    max: [f64; 2],
    cell: f64,
    material: ParticleMaterial,
    body: ([f64; 2], [f64; 2]),
) -> ParticleModel {
    let depth = cell;
    let (lo, hi) = body;
    ParticleModel {
        heading: heading.into(),
        materials: vec![material],
        bodies: vec![MaterialBody {
            name: "Body".into(),
            material: 0,
            shape: TriangleSurface::cuboid([lo[0], lo[1], -cell], [hi[0], hi[1], depth + cell]),
            velocity: [0.0; 3],
            temperature: 0.0,
            gravity: [0.0; 3],
        }],
        walls: Vec::new(),
        domain: Domain {
            min: [min[0], min[1], 0.0],
            max: [max[0], max[1], depth],
            cell,
            sides: [Side::Wall(WallLaw::Slip); 6],
            periodic: [false, false, true],
            space: Space::PlaneStrain,
        },
        gravity: [0.0, -G, 0.0],
        control: ParticleControl::default(),
        heat: None,
        inlets: Vec::new(),
    }
}

/// A layer `h` deep on a rough floor, periodic along the slope, the gravity tilted by the
/// angle of the slope.
fn incline(angle_deg: f64, per_depth: f64, end_time: f64) -> (ParticleResults, f64, f64) {
    let grain = 0.5e-3;
    let (results, h) = incline_of(beads(grain), angle_deg, per_depth, end_time);
    (results, h, grain)
}

fn incline_of(
    material: ParticleMaterial,
    angle_deg: f64,
    per_depth: f64,
    end_time: f64,
) -> (ParticleResults, f64) {
    let h = 0.01;
    let cell = h / per_depth;
    let mut model = plane(
        "Incline",
        [0.0, 0.0],
        [4.0 * cell, 1.5 * h],
        cell,
        material,
        ([-cell, 0.0], [5.0 * cell, h]),
    );
    model.domain.periodic[0] = true;
    model.domain.sides[2] = Side::Wall(WallLaw::Stick);
    model.domain.sides[3] = Side::Open;
    let theta = angle_deg.to_radians();
    model.gravity = [G * theta.sin(), -G * theta.cos(), 0.0];
    model.control.end_time = end_time;
    model.control.frames = 20;
    model.control.sound_speed = Some(4.0);
    (solve(&model), h)
}

fn mean_velocity(frame: &ParticleFrame) -> f64 {
    frame
        .velocities
        .iter()
        .map(|v| f64::from(v[0]))
        .sum::<f64>()
        / frame.velocities.len() as f64
}

/// Steady flow down a rough incline steeper than the angle of friction: the shear stress
/// carries the weight along the slope, so `μ(I) = tan θ` over the whole depth and the
/// inertial number is `I = I0 (tan θ − μs)/(μ2 − tan θ)` everywhere. This gives the profile
/// of Bagnold, `u(y) = ⅔ I/d √(g cos θ) (h^{3/2} − (h − y)^{3/2})` (GDR MiDi 2004).
///
/// Compared are the ratio of shear stress and pressure and the shear across the depth above
/// the floor: within a cell of the rough floor, where the profile is steepest, the speed
/// lags behind the exact one by about as much as a Newtonian film's at the same resolution,
/// and the mean speed with it (30 % at 12 cells over the depth, 13 % at 24).
fn bagnold(per_depth: f64) {
    let angle: f64 = 25.0;
    let (results, h, grain) = incline(angle, per_depth, 2.0);
    let theta = angle.to_radians();
    let i = INERTIAL * (theta.tan() - FRICTION) / (FRICTION_LIMIT - theta.tan());
    let scale = i / grain * (G * theta.cos()).sqrt();
    let exact = |y: f64| 2.0 / 3.0 * scale * (h.powf(1.5) - (h - y).powf(1.5));
    let speeds: Vec<f64> = results.frames.iter().map(mean_velocity).collect();
    for (frame, u) in results.frames.iter().zip(&speeds) {
        eprintln!(
            "t {:.2}: mean speed {u:.4} ({:.4})",
            frame.time,
            0.4 * scale * h.powf(1.5)
        );
    }
    let n = speeds.len();
    assert!(
        (speeds[n - 1] - speeds[n - 2]).abs() < 0.01 * speeds[n - 1],
        "not steady"
    );
    // Bands across the depth: speed, pressure and shear stress.
    let frame = results.frames.last().unwrap();
    let bands = 5;
    let mut rows = Vec::new();
    for b in 0..bands {
        let (lo, hi) = (
            b as f64 * h / bands as f64,
            (b + 1) as f64 * h / bands as f64,
        );
        let inside: Vec<usize> = (0..frame.positions.len())
            .filter(|&k| (lo..hi).contains(&f64::from(frame.positions[k][1])))
            .collect();
        let mean = |values: &[f32]| {
            inside.iter().map(|&k| f64::from(values[k])).sum::<f64>() / inside.len() as f64
        };
        let x: Vec<f32> = frame.velocities.iter().map(|v| v[0]).collect();
        let (u, p, tau) = (mean(&x), mean(&frame.fields[0]), mean(&frame.fields[1]));
        let y = 0.5 * (lo + hi);
        eprintln!(
            "y {y:.4}: u {u:.4} ({:.4}) p {p:.1} ({:.1}) tau/p {:.3} ({:.3})",
            exact(y),
            DENSITY * G * theta.cos() * (h - y),
            tau / p,
            theta.tan()
        );
        rows.push((y, u, tau / p));
    }
    // Not at the floor and not at the surface, where the stencils reach past the material.
    for &(y, _, ratio) in &rows[1..bands - 1] {
        assert!(
            (ratio - theta.tan()).abs() < 0.05 * theta.tan(),
            "y {y}: shear stress / pressure {ratio} instead of {}",
            theta.tan()
        );
    }
    let (low, high) = (rows[1], rows[bands - 1]);
    let shear = high.1 - low.1;
    let expected = exact(high.0) - exact(low.0);
    eprintln!("shear across the depth {shear:.4} ({expected:.4})");
    assert!(
        (shear - expected).abs() < 0.05 * expected,
        "speed difference {shear} instead of {expected}"
    );
}

#[test]
fn incline_flow() {
    bagnold(12.0);
}

#[test]
#[ignore]
fn incline_flow_fine() {
    bagnold(24.0);
}

/// Below the angle of friction the layer stays at rest.
#[test]
fn incline_at_rest() {
    let angle = FRICTION.atan().to_degrees() - 3.0;
    let (results, _, _) = incline(angle, 10.0, 0.5);
    let u = mean_velocity(results.frames.last().unwrap());
    eprintln!("{angle:.1} deg: mean speed {u:.2e}");
    assert!(u.abs() < 2e-3, "flows at {u} m/s");
}

/// A column `a = H/L` of sand released against a back wall onto a rough floor. Lube et al.
/// (2005, Phys. Rev. E 72, 041301) measured the runout `(L∞ − L)/L = 1.2 a` below
/// `a = 2.3` and `1.9 a^{2/3}` above, the final height `H∞ = H` below `a = 1.15` and
/// `H∞/L = a^{2/5}` above. Returns the runout and the height at the end, with the
/// measured ones.
fn collapse(ratio: f64, per_length: f64) -> ((f64, f64), (f64, f64)) {
    let length = 0.1;
    let height = ratio * length;
    let cell = length / per_length;
    let runout = if ratio < 2.3 {
        1.2 * ratio
    } else {
        1.9 * ratio.powf(2.0 / 3.0)
    } * length
        + length;
    let final_height = if ratio < 1.15 {
        height
    } else {
        ratio.powf(0.4) * length
    };
    let mut model = plane(
        "Granular collapse",
        [0.0, 0.0],
        [runout * 1.6, height * 1.1],
        cell,
        sand(),
        ([0.0, 0.0], [length, height]),
    );
    // A rough floor: its friction that of the sand at rest.
    model.domain.sides[2] = Side::Wall(WallLaw::Coulomb { friction: 0.6 });
    model.control.end_time = 6.0 * (height.max(length) / G).sqrt();
    model.control.frames = 6;
    let results = solve(&model);
    let mut shapes = Vec::new();
    for frame in &results.frames {
        let reach = |axis: usize| {
            frame
                .positions
                .iter()
                .map(|p| f64::from(p[axis]))
                .fold(0.0, f64::max)
                + 0.25 * cell
        };
        let (front, top) = (reach(0), reach(1));
        eprintln!(
            "t {:.3}: runout {front:.4} ({runout:.4}) height {top:.4} ({final_height:.4})",
            frame.time
        );
        shapes.push((front, top));
    }
    let (a, b) = (shapes[shapes.len() - 2], shapes[shapes.len() - 1]);
    assert!((b.0 - a.0).abs() < 0.02 * (b.0 - length), "still flowing");
    (b, (runout, final_height))
}

/// The runout beyond the column, relative to the measured one, and the height. Tall
/// columns come to rest some 10 to 20 % lower than Lube's `a^{2/5}` (9 % at `a = 2`, 17 %
/// at `a = 4`), their runout within 5 %.
fn check(ratio: f64, per_length: f64, runout_tolerance: f64) {
    let length = 0.1;
    let ((front, top), (runout, height)) = collapse(ratio, per_length);
    let spread = (front - length) / (runout - length);
    assert!(
        (spread - 1.0).abs() < runout_tolerance,
        "runout {front} instead of {runout}"
    );
    assert!(
        (top - height).abs() < 0.2 * height,
        "height {top} instead of {height}"
    );
}

/// A tall column: runout and height as measured.
#[test]
fn granular_column_collapse() {
    check(2.0, 10.0, 0.2);
}

#[test]
#[ignore]
fn granular_column_collapse_fine() {
    check(2.0, 20.0, 0.2);
}

#[test]
#[ignore]
fn granular_tall_column_collapse() {
    check(4.0, 16.0, 0.2);
}

/// A flat column keeps its height, only its edge slides off. Its runout depends on the
/// friction more than Lube's measurements show (for this sand a third short of `1.2 a`;
/// glass beads at 21° reach it within 5 %): checked is only that it slides a part of it.
#[test]
fn granular_flat_column_collapse() {
    check(0.5, 16.0, 0.5);
}

/// The index of the fine fraction in the fields of a frame.
const FINE: usize = 6;

/// A mixture of fine and coarse glass beads, half and half, flows down the incline: the
/// fine grains sink to the floor and the coarse rise (Savage & Lun 1988), until the layer
/// is sorted with the fine half at the bottom. The amount of fine grains stays.
fn segregation(end_time: f64) {
    let mut beads = beads(1e-3);
    if let ParticleLaw::Granular {
        fine_grain,
        fine_fraction,
        ..
    } = &mut beads.law
    {
        *fine_grain = 0.5e-3;
        *fine_fraction = 0.5;
    }
    let (results, h) = incline_of(beads, 25.0, 12.0, end_time);
    let fine_in = |frame: &ParticleFrame, lo: f64, hi: f64| {
        let (sum, n) = (frame.positions.iter().zip(&frame.fields[FINE]))
            .filter(|(x, _)| (lo..hi).contains(&f64::from(x[1])))
            .fold((0.0, 0), |(s, n), (_, f)| (s + f64::from(*f), n + 1));
        sum / f64::from(n.max(1))
    };
    let mut total = Vec::new();
    for frame in &results.frames {
        let mean: f64 = frame.fields[FINE]
            .iter()
            .map(|f| f64::from(*f))
            .sum::<f64>()
            / frame.fields[FINE].len() as f64;
        eprintln!(
            "t {:.2}: fine fraction bottom quarter {:.3}, top quarter {:.3}, mean {mean:.4}",
            frame.time,
            fine_in(frame, 0.0, 0.25 * h),
            fine_in(frame, 0.75 * h, h + 0.01),
        );
        total.push(mean);
    }
    let last = results.frames.last().unwrap();
    assert!(
        (total.last().unwrap() - 0.5).abs() < 0.005,
        "fine grains not kept"
    );
    assert!(fine_in(last, 0.0, 0.25 * h) > 0.9, "the floor is not fine");
    assert!(
        fine_in(last, 0.75 * h, h + 0.01) < 0.1,
        "the top is not coarse"
    );
}

#[test]
fn segregation_on_an_incline() {
    segregation(4.0);
}
