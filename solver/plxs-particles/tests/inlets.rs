//! Inlets: a channel driven by the pressure of a reservoir against Poiseuille's flow rate,
//! and a velocity inlet that feeds as much as its speed says, in 2D.

use plxs_core::particles::{
    Domain, Inlet, InletKind, MaterialBody, ParticleControl, ParticleLaw, ParticleMaterial,
    ParticleModel, ParticleResults, Side, Space, TriangleSurface, WallLaw,
};

const DENSITY: f64 = 1000.0;

fn solve(model: &ParticleModel) -> ParticleResults {
    let results = plxs_particles::solve(model, &mut |_| {}).expect("solves");
    eprintln!(
        "{}: {} particles, {} steps, {:.1} s",
        model.heading, results.report.particles, results.report.steps, results.report.seconds,
    );
    results
}

fn fluid(viscosity: f64) -> ParticleMaterial {
    ParticleMaterial {
        name: "Fluid".into(),
        density: DENSITY,
        law: ParticleLaw::Fluid {
            bulk: 2e6,
            viscosity,
        },
        thermal: None,
        flow_tables: Default::default(),
    }
}

/// A line from `a` to `b` in the plane, extruded across the slab `0..depth`, wound so that
/// its normal is the line's direction turned a quarter to the right (direction × z).
fn line(a: [f64; 2], b: [f64; 2], depth: f64) -> TriangleSurface {
    let points = vec![
        [a[0], a[1], -depth],
        [b[0], b[1], -depth],
        [b[0], b[1], 2.0 * depth],
        [a[0], a[1], 2.0 * depth],
    ];
    TriangleSurface {
        points,
        triangles: vec![[0, 1, 2], [0, 2, 3]],
    }
}

/// A plane channel `length` long and `width` wide along x, full of the fluid, between
/// rough walls at y = 0 and y = width; the side x max is open, at x min the inlet.
fn channel(kind: InletKind, viscosity: f64, cell: f64, length: f64, width: f64) -> ParticleModel {
    let mut sides = [Side::Wall(WallLaw::Stick); 6];
    sides[0] = Side::Open;
    sides[1] = Side::Open;
    ParticleModel {
        heading: format!("Channel {kind:?}"),
        materials: vec![fluid(viscosity)],
        bodies: vec![MaterialBody {
            name: "Fluid".into(),
            material: 0,
            shape: TriangleSurface::cuboid([0.0, 0.0, -cell], [length, width, 2.0 * cell]),
            velocity: [0.0; 3],
            temperature: 0.0,
            gravity: [0.0; 3],
        }],
        walls: Vec::new(),
        domain: Domain {
            min: [0.0; 3],
            max: [length, width, cell],
            cell,
            sides,
            periodic: [false, false, true],
            space: Space::PlaneStrain,
        },
        gravity: [0.0; 3],
        control: ParticleControl {
            end_time: 0.2,
            frames: 4,
            sound_speed: Some(20.0),
            ..ParticleControl::default()
        },
        heat: None,
        inlets: vec![Inlet {
            name: "Inlet".into(),
            // Upwards along x = 0: the normal turned to the right points along +x.
            shape: line([0.0, -cell], [0.0, width + cell], cell),
            kind,
            amplitude: Vec::new(),
            body: 0,
        }],
    }
}

/// The mean speed over the channel from the mass fed between the middle and the end.
fn fed_speed(results: &ParticleResults, width: f64) -> f64 {
    let h = &results.history;
    let n = h.time.len() - 1;
    let i = n / 2;
    (h.inflow[n] - h.inflow[i]) / (DENSITY * width * (h.time[n] - h.time[i]))
}

/// The mass is kept: what is in the model is what was, plus what came in, less what left.
fn assert_mass_kept(results: &ParticleResults) {
    let h = &results.history;
    let start = h.mass[0] + h.outflow[0] - h.inflow[0];
    for k in 0..h.time.len() {
        let now = h.mass[k] + h.outflow[k] - h.inflow[k];
        assert!(
            (now - start).abs() < 1e-9 * start,
            "mass {now} against {start} at {}",
            h.time[k]
        );
    }
}

#[test]
fn a_reservoir_drives_a_channel_at_the_flow_rate_of_poiseuille() {
    let (cell, length, width) = (1e-3, 0.03, 0.012);
    let viscosity = 10.0;
    let pressure = 1250.0;
    let results = solve(&channel(
        InletKind::Pressure(pressure),
        viscosity,
        cell,
        length,
        width,
    ));
    assert_mass_kept(&results);
    // The plane Poiseuille flow: the mean speed Δp W² / (12 μ L), of the particles of the
    // second half, which lie evenly across the channel.
    let expected = pressure * width * width / (12.0 * viscosity * length);
    let last = results.frames.last().expect("frames");
    let (sum, count) = (last.positions.iter().zip(&last.velocities))
        .filter(|(x, _)| f64::from(x[0]) > 0.5 * length)
        .fold((0.0, 0), |(s, n), (_, v)| (s + f64::from(v[0]), n + 1));
    let speed = sum / f64::from(count);
    eprintln!("mean speed {speed:.5} m/s, Poiseuille {expected:.5} m/s");
    assert!(
        (speed / expected - 1.0).abs() < 0.1,
        "mean speed {speed} against {expected}"
    );
    // What enters leaves. The material fed comes in as a plug, which the walls then slow
    // at the sides: there its particles crowd a little, so that a little more mass
    // enters than leaves while it fills the channel.
    let fed = fed_speed(&results, width);
    eprintln!("fed at {fed:.5} m/s");
    assert!((fed / speed - 1.0).abs() < 0.2, "fed at {fed} for {speed}");
    let h = &results.history;
    let n = h.time.len() - 1;
    let i = n / 2;
    let left = (h.outflow[n] - h.outflow[i]) / (h.inflow[n] - h.inflow[i]);
    assert!((left - 1.0).abs() < 0.2, "outflow / inflow {left}");
}

#[test]
fn a_velocity_inlet_feeds_what_its_speed_says() {
    let (cell, length, width) = (1e-3, 0.02, 0.008);
    let speed = 0.05;
    let results = solve(&channel(
        InletKind::Velocity(speed),
        1.0,
        cell,
        length,
        width,
    ));
    assert_mass_kept(&results);
    let fed = fed_speed(&results, width);
    eprintln!("fed at {fed:.5} m/s for {speed} m/s");
    assert!((fed / speed - 1.0).abs() < 0.05, "fed at {fed} for {speed}");
}

#[test]
fn a_reservoir_drives_a_pipe_of_revolution_at_the_flow_rate_of_hagen_poiseuille() {
    let (cell, length, radius) = (1e-3, 0.03, 0.008);
    let viscosity = 10.0;
    let pressure = 2000.0;
    let mut model = channel(
        InletKind::Pressure(pressure),
        viscosity,
        cell,
        length,
        radius,
    );
    // The pipe along y about the axis x = 0, the reservoir on top.
    model.heading = "Pipe of revolution".into();
    model.domain.space = Space::Axisymmetric;
    model.domain.max = [radius, length, cell];
    model.domain.sides = [
        Side::Wall(WallLaw::Slip),
        Side::Wall(WallLaw::Stick),
        Side::Open,
        Side::Open,
        Side::Wall(WallLaw::Slip),
        Side::Wall(WallLaw::Slip),
    ];
    model.bodies[0].shape =
        TriangleSurface::cuboid([0.0, 0.0, -cell], [radius, length, 2.0 * cell]);
    model.inlets[0].shape = line([0.0, length], [radius + cell, length], cell);
    let results = solve(&model);
    assert_mass_kept(&results);
    // Hagen-Poiseuille: the mean speed Δp R² / (8 μ L), of the material of the lower half
    // weighted by its rings.
    let expected = pressure * radius * radius / (8.0 * viscosity * length);
    let last = results.frames.last().expect("frames");
    let (sum, weight) = (last.positions.iter().zip(&last.velocities))
        .filter(|(x, _)| f64::from(x[1]) < 0.5 * length)
        .fold((0.0, 0.0), |(s, w), (x, v)| {
            (s - f64::from(v[1]) * f64::from(x[0]), w + f64::from(x[0]))
        });
    let speed = sum / weight;
    eprintln!("mean speed {speed:.5} m/s, Hagen-Poiseuille {expected:.5} m/s");
    assert!(
        (speed / expected - 1.0).abs() < 0.15,
        "mean speed {speed} against {expected}"
    );
}
