//! 2D particle models, a slab one cell thick: the plane flow against the same model three
//! cells thick and periodic, viscous flow down a pipe and a channel against Poiseuille's
//! parabola, and collapsing columns of revolution, of sand and of water, against a quarter
//! of the cylinder computed in 3D.

use plxs_core::particles::{
    Domain, MaterialBody, ParticleControl, ParticleLaw, ParticleMaterial, ParticleModel,
    ParticleResults, Side, Space, TriangleSurface, WallLaw,
};

const G: f64 = 9.81;
const DENSITY: f64 = 1500.0;

fn solve(model: &ParticleModel) -> ParticleResults {
    let results = plxs_particles::solve(model, &mut |_| {}).expect("solves");
    eprintln!(
        "{}: {} particles, {} steps, {:.1} s",
        model.heading, results.report.particles, results.report.steps, results.report.seconds,
    );
    results
}

/// Quartz sand as in the experiments of Lube et al.: an angle of repose of 31°.
fn sand() -> ParticleMaterial {
    ParticleMaterial {
        name: "Sand".into(),
        density: DENSITY,
        law: ParticleLaw::Granular {
            shear: 4e5,
            bulk: 1e6,
            friction: 0.6,
            friction_limit: 0.9,
            grain: 1e-3,
            inertial: 0.216,
            fine_grain: 0.0,
            fine_fraction: 0.0,
        },
        thermal: None,
        flow_tables: Default::default(),
    }
}

fn fluid(viscosity: f64, bulk: f64) -> ParticleMaterial {
    ParticleMaterial {
        name: "Fluid".into(),
        density: 1000.0,
        law: ParticleLaw::Fluid { bulk, viscosity },
        thermal: None,
        flow_tables: Default::default(),
    }
}

/// A model in x and y with the body `lo..hi`: a slab in `space`, or for `Space::Solid`
/// three cells thick and periodic in z.
fn model(
    space: Space,
    max: [f64; 2],
    cell: f64,
    material: ParticleMaterial,
    body: ([f64; 2], [f64; 2]),
) -> ParticleModel {
    let depth = if space == Space::Solid {
        3.0 * cell
    } else {
        cell
    };
    let (lo, hi) = body;
    ParticleModel {
        heading: format!("{space:?}"),
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
            min: [0.0; 3],
            max: [max[0], max[1], depth],
            cell,
            sides: [Side::Wall(WallLaw::Slip); 6],
            periodic: [false, false, true],
            space,
        },
        gravity: [0.0, -G, 0.0],
        control: ParticleControl::default(),
        heat: None,
        inlets: Vec::new(),
    }
}

/// A quarter of the cylinder of `radius` and `height` about the y axis in 3D, between two
/// frictionless planes of symmetry, in a box as wide as `reach`.
fn quarter(
    material: ParticleMaterial,
    radius: f64,
    height: f64,
    reach: f64,
    cell: f64,
) -> ParticleModel {
    ParticleModel {
        heading: "Quarter in 3D".into(),
        materials: vec![material],
        bodies: vec![MaterialBody {
            name: "Body".into(),
            material: 0,
            shape: TriangleSurface::cylinder([0.0; 3], [0.0, height, 0.0], radius, 128),
            velocity: [0.0; 3],
            temperature: 0.0,
            gravity: [0.0; 3],
        }],
        walls: Vec::new(),
        domain: Domain {
            min: [0.0; 3],
            max: [reach, height * 1.1, reach],
            cell,
            sides: [Side::Wall(WallLaw::Slip); 6],
            ..Default::default()
        },
        gravity: [0.0, -G, 0.0],
        control: ParticleControl::default(),
        heat: None,
        inlets: Vec::new(),
    }
}

/// The distance of a particle from the y axis: in 3D the radius, else x.
fn radius_of(p: &[f32; 3], solid: bool) -> f64 {
    let x = f64::from(p[0]);
    if solid { x.hypot(f64::from(p[2])) } else { x }
}

/// Per frame: how far the material reaches from the axis, and how high it stands per
/// band of the radius `band` wide.
fn shape(results: &ParticleResults, solid: bool, band: f64) -> Vec<(f64, Vec<f64>)> {
    (results.frames.iter())
        .map(|frame| {
            let mut top = vec![0.0f64; 40];
            let mut front = 0.0f64;
            for p in &frame.positions {
                let r = radius_of(p, solid);
                front = front.max(r);
                let b = ((r / band) as usize).min(top.len() - 1);
                top[b] = top[b].max(f64::from(p[1]));
            }
            (front, top)
        })
        .collect()
}

/// The force the material puts on the floor (the box side y min) at the end, averaged
/// over the last tenth of the history.
fn floor_force(results: &ParticleResults) -> f64 {
    let floor = (results.history.walls.iter())
        .find(|w| w.name == "Box Y min")
        .expect("floor");
    let tail = &floor.force[floor.force.len() * 9 / 10..];
    -tail.iter().map(|f| f[1]).sum::<f64>() / tail.len() as f64
}

/// A column of sand `a = 2` collapses on a rough floor: in the slab one cell thick as in a
/// box three cells thick, and the slab carries its weight per unit length on the floor.
#[test]
fn a_plane_strain_slab_flows_like_a_periodic_box() {
    let (length, height, cell) = (0.1, 0.2, 0.01);
    let run = |space| {
        let mut m = model(
            space,
            [0.5, 0.22],
            cell,
            sand(),
            ([0.0, 0.0], [length, height]),
        );
        m.domain.sides[2] = Side::Wall(WallLaw::Coulomb { friction: 0.6 });
        m.control.end_time = 6.0 * (height / G).sqrt();
        m.control.frames = 4;
        solve(&m)
    };
    let slab = run(Space::PlaneStrain);
    let solid = run(Space::Solid);
    let (a, b) = (shape(&slab, false, cell), shape(&solid, false, cell));
    let (a, b) = (a.last().expect("frames"), b.last().expect("frames"));
    eprintln!("runout: slab {:.4}, box {:.4}", a.0, b.0);
    assert!((a.0 - b.0).abs() < 0.01 * b.0, "runout {} {}", a.0, b.0);
    for (x, y) in a.1.iter().zip(&b.1) {
        assert!((x - y).abs() < 0.1 * cell, "height {x} {y}");
    }
    let weight = DENSITY * length * height * G;
    let force = floor_force(&slab);
    eprintln!("floor {force:.1} N/m, weight {weight:.1} N/m");
    assert!((force / weight - 1.0).abs() < 0.02, "{force} {weight}");
}

/// A viscous fluid flows down a pipe under its weight, Poiseuille's parabola
/// `v = ρ g (R² − r²) / (4 μ)`, and down a channel, `ρ g (R² − x²) / (2 μ)` (the axis of
/// the pipe, the middle of the channel at x = 0). Returns the largest difference of the
/// mean velocity per band of a cell from it, relative to the speed on the axis.
fn poiseuille(space: Space) -> f64 {
    let (radius, cell, viscosity) = (0.01, 0.001, 10.0);
    let length = 4.0 * cell;
    let mut m = model(
        space,
        [radius, length],
        cell,
        fluid(viscosity, 2e6),
        ([0.0, -cell], [radius, length + cell]),
    );
    m.domain.periodic[1] = true;
    m.domain.sides[1] = Side::Wall(WallLaw::Stick);
    // Some ten times the time the flow takes to settle, R² ρ / μ.
    m.control.end_time = 0.15;
    m.control.frames = 3;
    let results = solve(&m);
    let frame = results.frames.last().expect("frames");
    let factor = if space == Space::Axisymmetric {
        4.0
    } else {
        2.0
    };
    let axis = 1000.0 * G * radius * radius / (factor * viscosity);
    let mut bins = [(0.0, 0usize); 10];
    for (p, v) in frame.positions.iter().zip(&frame.velocities) {
        let b = ((f64::from(p[0]) / cell) as usize).min(9);
        bins[b].0 -= f64::from(v[1]);
        bins[b].1 += 1;
    }
    let mut worst: f64 = 0.0;
    for (b, (sum, n)) in bins.iter().enumerate() {
        let r = (b as f64 + 0.5) * cell;
        let exact = axis * (1.0 - (r / radius).powi(2));
        let mean = sum / *n as f64;
        eprintln!("{space:?} r {r:.4}: {mean:.5} m/s, Poiseuille {exact:.5} m/s");
        worst = worst.max((mean - exact).abs() / axis);
    }
    worst
}

#[test]
fn a_viscous_fluid_flows_down_a_pipe_as_poiseuille() {
    let worst = poiseuille(Space::Axisymmetric);
    assert!(worst < 0.03, "{worst}");
}

#[test]
fn a_viscous_fluid_flows_down_a_channel_as_poiseuille() {
    let worst = poiseuille(Space::PlaneStrain);
    assert!(worst < 0.03, "{worst}");
}

/// A cylinder of sand `a = H/r = 1` released on a rough floor comes to rest as high over
/// the radius, and as far, as a quarter of it computed in 3D. (Both stop some 20 % short
/// of the runout Lube et al. 2004 measured, at ten cells over the radius.)
#[test]
fn a_column_of_sand_of_revolution_collapses_as_in_3d() {
    let (radius, cell) = (0.05, 0.005);
    let rough = Side::Wall(WallLaw::Coulomb { friction: 0.6 });
    let end = 6.0 * (radius / G).sqrt();
    let mut solid = quarter(sand(), radius, radius, 2.7 * radius, cell);
    solid.domain.sides[2] = rough;
    solid.control.end_time = end;
    solid.control.frames = 2;
    let mut ring = model(
        Space::Axisymmetric,
        [2.7 * radius, 1.1 * radius],
        cell,
        sand(),
        ([0.0, 0.0], [radius, radius]),
    );
    ring.domain.sides[2] = rough;
    ring.control.end_time = end;
    ring.control.frames = 2;
    let a = shape(&solve(&ring), false, cell);
    let b = shape(&solve(&solid), true, cell);
    let (a, b) = (a.last().expect("frames"), b.last().expect("frames"));
    eprintln!("runout: ring {:.4}, 3D {:.4}", a.0, b.0);
    assert!((a.0 - b.0).abs() < cell, "runout {} {}", a.0, b.0);
    for (band, (x, y)) in a.1.iter().zip(&b.1).enumerate() {
        eprintln!(
            "r {:.4}: ring {x:.4}, 3D {y:.4}",
            (band as f64 + 0.5) * cell
        );
        if *y > 2.0 * cell {
            assert!((x - y).abs() < 0.1 * y, "height {x} {y}");
        }
    }
}

/// A cylinder of water breaks like a dam: its front runs out over the floor as fast as
/// that of a quarter of it computed in 3D.
#[test]
fn a_column_of_water_of_revolution_spreads_as_in_3d() {
    let (radius, cell) = (0.05, 0.005);
    let water = || fluid(0.05, 2e5);
    let mut solid = quarter(water(), radius, radius, 0.25, cell);
    solid.control.end_time = 0.15;
    solid.control.frames = 3;
    let mut ring = model(
        Space::Axisymmetric,
        [0.25, 1.1 * radius],
        cell,
        water(),
        ([0.0, 0.0], [radius, radius]),
    );
    ring.control.end_time = 0.15;
    ring.control.frames = 3;
    let a = shape(&solve(&ring), false, 0.01);
    let b = shape(&solve(&solid), true, 0.01);
    for ((front, top), (front_3d, top_3d)) in a.iter().zip(&b).skip(1) {
        eprintln!("front: ring {front:.4}, 3D {front_3d:.4}");
        assert!(
            (front - front_3d).abs() < 0.04 * front_3d,
            "{front} {front_3d}"
        );
        for (x, y) in top.iter().zip(top_3d) {
            assert!((x - y).abs() < cell, "height {x} {y}");
        }
    }
}

/// Water rests in a round cup: the floor carries the weight of the whole cylinder.
#[test]
fn water_in_a_cup_of_revolution_weighs_the_whole_cylinder() {
    let (radius, depth, cell) = (0.05, 0.04, 0.004);
    let mut m = model(
        Space::Axisymmetric,
        [radius, 0.05],
        cell,
        fluid(0.5, 2e5),
        ([0.0, 0.0], [radius, depth]),
    );
    m.control.end_time = 0.3;
    m.control.frames = 1;
    let results = solve(&m);
    let weight = 1000.0 * std::f64::consts::PI * radius * radius * depth * G;
    let force = floor_force(&results);
    eprintln!("floor {force:.3} N, weight {weight:.3} N");
    assert!((force / weight - 1.0).abs() < 0.02, "{force} {weight}");
}

/// A ring of fluid that flows to the axis of revolution: its particles spread in the plane
/// (a ring nearer the axis has a larger section) and split, so that the material stays
/// whole, every particle at most a cell from the next; the mass stays.
#[test]
fn a_ring_flowing_to_the_axis_splits_its_particles_and_stays_whole() {
    let cell = 0.005;
    let mut m = model(
        Space::Axisymmetric,
        [0.15, 0.04],
        cell,
        fluid(2.0, 2e5),
        ([0.09, 0.0], [0.13, 0.03]),
    );
    m.gravity = [0.0; 3];
    m.bodies[0].velocity = [-0.6, 0.0, 0.0];
    m.control.end_time = 0.15;
    m.control.frames = 3;
    let results = solve(&m);
    let first = &results.frames[0];
    let last = results.frames.last().unwrap();
    let mass = &results.history.mass;
    eprintln!(
        "{} particles at the start, {} at the end; mass {:.6e} to {:.6e}",
        first.ids.len(),
        last.ids.len(),
        mass[0],
        mass[mass.len() - 1]
    );
    assert!(last.ids.len() > first.ids.len() * 3 / 2, "no split");
    assert!((mass[mass.len() - 1] / mass[0] - 1.0).abs() < 1e-9);
    // Unique numbers, and the share of each particle.
    let mut ids = last.ids.clone();
    ids.sort_unstable();
    ids.dedup();
    assert_eq!(ids.len(), last.ids.len());
    assert_eq!(last.shares.len(), last.ids.len());
    // The material reached the axis and holds together: near the axis, where it spread
    // most, every cell holds a particle (without splitting every other column is empty).
    let inner = last
        .positions
        .iter()
        .map(|p| p[0])
        .fold(f32::INFINITY, f32::min);
    eprintln!("inner edge {inner}");
    assert!(f64::from(inner) < 0.02, "inner edge {inner}");
    let at = |p: &[f32; 3]| {
        (
            (f64::from(p[0]) / cell) as i64,
            (f64::from(p[1]) / cell) as i64,
        )
    };
    let taken: std::collections::HashSet<(i64, i64)> = last.positions.iter().map(at).collect();
    let near: Vec<(i64, i64)> = (1..6).flat_map(|i| (0..6).map(move |j| (i, j))).collect();
    let filled = near.iter().filter(|c| taken.contains(c)).count();
    eprintln!(
        "{filled} of {} cells near the axis hold particles",
        near.len()
    );
    assert_eq!(filled, near.len());
}
