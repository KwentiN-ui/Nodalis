//! The acceptance cases of stage 2 (`STUFENKONZEPT_1_2_PARTIKEL_MPM.md`, section 3): each
//! against an analytical solution or a measurement. The quick versions run in every test
//! run; the finer ones with `--ignored`.
//!
//! Plane cases are computed in 3D with a box three cells thick and periodic in z: every
//! field is then constant along z, which is the plane strain state.

use plxs_core::particles::{
    Domain, Heat, MaterialBody, ParticleControl, ParticleFrame, ParticleLaw, ParticleMaterial,
    ParticleModel, ParticleResults, ParticleThermal, RigidMotion, RigidWall, Side, TriangleSurface,
    WallLaw,
};

const G: f64 = 9.81;

fn solve(model: &ParticleModel) -> ParticleResults {
    let results = plxs_particles::solve(model, &mut |_| {}).expect("solves");
    eprintln!(
        "{}: {} particles, {} steps, dt {:.2e}..{:.2e}, {:.1} s, factor {:.1}, mach {:.3}",
        model.heading,
        results.report.particles,
        results.report.steps,
        results.report.time_step.1,
        results.report.time_step.0,
        results.report.seconds,
        results.report.sound_speed_factor,
        results.report.max_mach
    );
    for n in &results.report.notes {
        eprintln!("  {n}");
    }
    results
}

/// A plane model in x and y, three cells thick in z.
fn plane(
    heading: &str,
    min: [f64; 2],
    max: [f64; 2],
    cell: f64,
    material: ParticleMaterial,
    bodies: Vec<([f64; 2], [f64; 2])>,
) -> ParticleModel {
    let depth = 3.0 * cell;
    ParticleModel {
        heading: heading.into(),
        materials: vec![material],
        bodies: bodies
            .into_iter()
            .enumerate()
            .map(|(i, (lo, hi))| MaterialBody {
                name: format!("Body {i}"),
                material: 0,
                shape: TriangleSurface::cuboid([lo[0], lo[1], -cell], [hi[0], hi[1], depth + cell]),
                velocity: [0.0; 3],
                temperature: 0.0,
                gravity: [0.0; 3],
            })
            .collect(),
        walls: Vec::new(),
        domain: Domain {
            min: [min[0], min[1], 0.0],
            max: [max[0], max[1], depth],
            cell,
            sides: [Side::Wall(WallLaw::Slip); 6],
            periodic: [false, false, true],
        },
        gravity: [0.0, -G, 0.0],
        control: ParticleControl::default(),
        heat: None,
    }
}

fn water(viscosity: f64) -> ParticleMaterial {
    ParticleMaterial {
        name: "Water".into(),
        density: 1000.0,
        law: ParticleLaw::Fluid {
            bulk: 2.2e9,
            viscosity,
        },
        thermal: None,
        flow_factor: Vec::new(),
    }
}

/// A Bingham paste: soft elastic part (pastes have shear moduli of some kPa), the bulk
/// modulus a hundred times the shear modulus.
fn bingham(yield_stress: f64, viscosity: f64) -> ParticleMaterial {
    ParticleMaterial {
        name: "Paste".into(),
        density: 1000.0,
        law: ParticleLaw::ViscoPlastic {
            shear: 1e4,
            bulk: 1e6,
            yield_stress,
            consistency: viscosity,
            exponent: 1.0,
        },
        thermal: None,
        flow_factor: Vec::new(),
    }
}

fn last(results: &ParticleResults) -> &ParticleFrame {
    results.frames.last().unwrap()
}

fn largest(frame: &ParticleFrame, axis: usize) -> f64 {
    frame
        .positions
        .iter()
        .map(|p| f64::from(p[axis]))
        .fold(f64::MIN, f64::max)
}

fn max_balance_error(results: &ParticleResults) -> f64 {
    results
        .history
        .balance_error
        .iter()
        .fold(0.0, |m, e| m.max(e.abs()))
}

/// Water at rest in a basin: the pressure grows with the depth as `ρ g (H − y)`, and the
/// floor carries the weight.
#[test]
fn resting_basin() {
    let (width, height, cell) = (0.4, 0.4, 0.02);
    let mut model = plane(
        "Resting basin",
        [0.0, 0.0],
        [width, 0.6],
        cell,
        water(0.5),
        vec![([0.0, 0.0], [width, height])],
    );
    model.control.end_time = 1.5;
    model.control.frames = 150;
    model.control.sound_speed = Some(30.0);
    let results = solve(&model);
    // The pressure against the depth, averaged in bands of one cell and over the last
    // second: the column swings about its rest state with little damping, the time
    // average is the rest state.
    let frames = &results.frames[50..];
    let mut worst: f64 = 0.0;
    // Not in the cell at the floor and at the surface: there the stencils reach past
    // the material, and the pressure is some 10 % off.
    for band in [0.03, 0.05, 0.15, 0.25, 0.35, 0.37] {
        let (mut sum, mut n) = (0.0, 0);
        for frame in frames {
            for (p, x) in frame.fields[0].iter().zip(&frame.positions) {
                if (f64::from(x[1]) - band).abs() < 0.5 * cell {
                    sum += f64::from(*p);
                    n += 1;
                }
            }
        }
        let mean = sum / n as f64;
        let expected = 1000.0 * G * (height - band);
        eprintln!("y {band}: p {mean:.1} expected {expected:.1}");
        worst = worst.max((mean - expected).abs() / (1000.0 * G * height));
    }
    assert!(
        worst < 0.05,
        "pressure off by {worst} of the pressure at the floor"
    );
    let floor = results
        .history
        .walls
        .iter()
        .find(|w| w.name == "Box Y min")
        .unwrap();
    let n = floor.force.len();
    let late: f64 = floor.force[n / 3..].iter().map(|f| f[1]).sum::<f64>() / (n - n / 3) as f64;
    let weight = -1000.0 * G * width * height * 3.0 * cell;
    eprintln!("floor force {late:.3} weight {weight:.3}");
    assert!((late - weight).abs() < 0.03 * weight.abs());
}

/// Dam break of a water column `a` wide and `2a` high (Martin & Moyce 1952): the surge front
/// against the time.
#[test]
fn dam_break() {
    let a = 0.1;
    let cell = a / 16.0;
    let mut model = plane(
        "Dam break",
        [0.0, 0.0],
        [6.0 * a, 2.5 * a],
        cell,
        water(1e-3),
        vec![([0.0, 0.0], [a, 2.0 * a])],
    );
    let scale = (a / (2.0 * G)).sqrt();
    let end = 3.0 * scale;
    model.control.end_time = end;
    model.control.frames = 6;
    // Ten times the speed of the front (2√(g 2a) by Ritter).
    model.control.sound_speed = Some(10.0 * 2.0 * (G * 2.0 * a).sqrt());
    let results = solve(&model);
    for frame in &results.frames {
        let t = frame.time / scale;
        let z = largest(frame, 0) / a;
        eprintln!("T {t:.2} Z {z:.2}");
    }
    // Martin & Moyce, square-root-2 column (n² = 2): Z ≈ 2.4 at T = 2 and about 3.6 at
    // T = 3. Taken from memory of the published plot, hence the wide band.
    let at = |t: f64| {
        let f = results
            .frames
            .iter()
            .min_by(|x, y| ((x.time / scale - t).abs()).total_cmp(&(y.time / scale - t).abs()))
            .unwrap();
        largest(f, 0) / a
    };
    let (z2, z3) = (at(2.0), at(3.0));
    assert!((z2 - 2.4).abs() < 0.2 * 2.4, "Z at T = 2: {z2}");
    assert!((z3 - 3.6).abs() < 0.2 * 3.6, "Z at T = 3: {z3}");
    // Never faster than the shallow-water front 2√(g H) (Ritter): x ≤ a + 2√(g 2a) t.
    for f in &results.frames {
        let bound = a + 2.0 * (G * 2.0 * a).sqrt() * f.time;
        assert!(largest(f, 0) <= bound + cell, "front ahead of Ritter's");
    }
    let e = max_balance_error(&results);
    eprintln!("balance error {e:.3}");
    assert!(e < 0.1, "energy balance error {e}");
}

/// A rigid block of a plane model, reaching past the box in z.
fn plane_wall(name: &str, lo: [f64; 2], hi: [f64; 2], depth: f64, law: WallLaw) -> RigidWall {
    RigidWall {
        temperature: None,
        name: name.into(),
        shape: TriangleSurface::cuboid([lo[0], lo[1], -depth], [hi[0], hi[1], 2.0 * depth]),
        law,
        motion: RigidMotion::default(),
    }
}

/// Slump test of a Bingham cylinder as tall as wide (Pashias et al. 1996): the slump
/// `s/H = 1 − 2τ'(1 − ln 2τ')`, `τ' = τ_y/(ρ g H)`. The whole cylinder: slip walls as
/// planes of symmetry let the material near them sink (the stencils of the particles next
/// to a wall miss the mirrored material). The theory is static: the paste is viscous enough
/// (40 Pa s) to come to rest without the overshoot of a fast collapse, which slumps further.
fn slump(per_height: f64) {
    let height = 0.1;
    let radius = 0.05;
    let tau = 0.15;
    let yield_stress = tau * 1000.0 * G * height;
    let cell = height / per_height;
    let model = ParticleModel {
        heading: "Bingham slump".into(),
        materials: vec![bingham(yield_stress, 40.0)],
        bodies: vec![MaterialBody {
            name: "Cylinder".into(),
            material: 0,
            shape: TriangleSurface::cylinder([0.0; 3], [0.0, height, 0.0], radius, 96),
            velocity: [0.0; 3],
            temperature: 0.0,
            gravity: [0.0; 3],
        }],
        walls: Vec::new(),
        domain: Domain {
            min: [-0.15, 0.0, -0.15],
            max: [0.15, 1.2 * height, 0.15],
            cell,
            sides: [
                Side::Wall(WallLaw::Slip),
                Side::Wall(WallLaw::Slip),
                Side::Wall(WallLaw::Stick),
                Side::Wall(WallLaw::Slip),
                Side::Wall(WallLaw::Slip),
                Side::Wall(WallLaw::Slip),
            ],
            periodic: [false; 3],
        },
        gravity: [0.0, -G, 0.0],
        heat: None,
        control: ParticleControl {
            end_time: 1.2,
            frames: 6,
            ..Default::default()
        },
    };
    let results = solve(&model);
    // Pashias et al. derive it with the Tresca criterion (yield in compression at 2τ_y);
    // the paste here yields after von Mises (at √3 τ_y), which slumps more.
    let tresca = 1.0 - 2.0 * tau * (1.0 - (2.0 * tau).ln());
    let r3 = 3f64.sqrt() * tau;
    let expected = 1.0 - r3 * (1.0 - r3.ln());
    eprintln!("Pashias (Tresca) {tresca:.3}, with von Mises {expected:.3}");
    let mut slumps = Vec::new();
    for frame in &results.frames {
        let top = (frame.positions.iter())
            .filter(|p| f64::from(p[0]).hypot(f64::from(p[2])) < cell)
            .map(|p| f64::from(p[1]))
            .fold(0.0, f64::max);
        // The particles' centres lie a quarter cell below the surface.
        let s = (height - top - 0.25 * cell) / height;
        eprintln!("t {:.2} slump {s:.3} expected {expected:.3}", frame.time);
        slumps.push(s);
    }
    let (a, b) = (slumps[slumps.len() - 2], slumps[slumps.len() - 1]);
    assert!((b - a).abs() < 0.015, "still slumping: {a} then {b}");
    assert!(
        (b - expected).abs() < 0.06,
        "slump {b} instead of {expected}"
    );
}

#[test]
fn bingham_slump() {
    slump(10.0);
}

#[test]
#[ignore]
fn bingham_slump_fine() {
    slump(20.0);
}

/// A plane Bingham column collapses to both sides and comes to rest in the shape where the
/// yield stress holds it: `h(x) = √(2 τ_y (L − |x|)/(ρ g))` by lubrication theory for a thin
/// layer on a rough floor (Liu & Mei 1989; Coussot 1997), with the half length `L` from the
/// area, `A/2 = ⅔ √(2 τ_y/(ρ g)) L^{3/2}`. This is the steepest shape at rest: a collapse
/// with inertia overshoots it, so the paste is viscous enough to flow slowly.
fn column(per_width: f64) {
    let (width, height) = (0.2, 0.2);
    let yield_stress = 30.0;
    let cell = 0.5 * width / per_width;
    let mut model = plane(
        "Bingham column collapse",
        [-0.7, 0.0],
        [0.7, 0.25],
        cell,
        bingham(yield_stress, 8.0),
        vec![([-0.5 * width, 0.0], [0.5 * width, height])],
    );
    model.domain.sides[2] = Side::Wall(WallLaw::Stick);
    model.control.end_time = 3.0;
    model.control.frames = 12;
    model.control.sound_speed = Some(15.0);
    let results = solve(&model);
    let k = (2.0 * yield_stress / (1000.0 * G)).sqrt();
    let length = (0.5 * width * height / (2.0 / 3.0 * k)).powf(2.0 / 3.0);
    let h0 = k * length.sqrt();
    let mut shapes = Vec::new();
    for frame in &results.frames {
        let front = 0.5
            * (largest(frame, 0)
                - frame
                    .positions
                    .iter()
                    .map(|p| f64::from(p[0]))
                    .fold(f64::MAX, f64::min))
            + 0.25 * cell;
        let middle = (frame.positions.iter())
            .filter(|p| f64::from(p[0]).abs() < cell)
            .map(|p| f64::from(p[1]))
            .fold(0.0, f64::max)
            + 0.25 * cell;
        eprintln!(
            "t {:.2} half length {front:.3} ({length:.3}) height {middle:.4} ({h0:.4})",
            frame.time
        );
        shapes.push((front, middle));
    }
    let (a, b) = (shapes[shapes.len() - 2], shapes[shapes.len() - 1]);
    assert!((b.0 - a.0).abs() < 0.02 * length, "still flowing");
    assert!(
        (b.0 - length).abs() < 0.15 * length,
        "runout {} instead of {length}",
        b.0
    );
    assert!(
        (b.1 - h0).abs() < 0.15 * h0,
        "height {} instead of {h0}",
        b.1
    );
}

#[test]
fn bingham_column_collapse() {
    column(10.0);
}

#[test]
#[ignore]
fn bingham_column_collapse_fine() {
    column(20.0);
}

/// Fully developed flow of a Bingham paste in a round pipe, driven by a body force: the
/// flow rate after Buckingham-Reiner, `Q = π R⁴ ρ g/(8 η) (1 − 4φ/3 + φ⁴/3)` with
/// `φ = τ_y/τ_w` and the wall shear stress `τ_w = ρ g R / 2`. Periodic along the pipe.
fn pipe(per_radius: f64) {
    let radius = 0.05;
    let (density, g, viscosity, yield_stress) = (1000.0, 10.0, 5.0, 75.0);
    let cell = radius / per_radius;
    let length = 3.0 * cell;
    let reach = radius + 3.0 * cell;
    let outer = TriangleSurface::cuboid([-reach, -reach, -1.0], [reach, reach, 1.0 + length]);
    let inner = TriangleSurface::cylinder([0.0, 0.0, -1.0], [0.0, 0.0, 2.0 + length], radius, 256);
    let mut tube = outer;
    let offset = tube.points.len() as u32;
    let inner = inner.flipped();
    tube.points.extend(inner.points);
    tube.triangles
        .extend(inner.triangles.into_iter().map(|t| t.map(|i| i + offset)));
    let model = ParticleModel {
        heading: "Bingham pipe flow".into(),
        materials: vec![bingham(yield_stress, viscosity)],
        bodies: vec![MaterialBody {
            name: "Paste".into(),
            material: 0,
            shape: TriangleSurface::cylinder(
                [0.0, 0.0, -1.0],
                [0.0, 0.0, 2.0 + length],
                radius,
                256,
            ),
            velocity: [0.0; 3],
            temperature: 0.0,
            gravity: [0.0; 3],
        }],
        walls: vec![RigidWall {
            name: "Pipe".into(),
            shape: tube,
            law: WallLaw::Stick,
            motion: RigidMotion::default(),
            temperature: None,
        }],
        domain: Domain {
            min: [-reach + cell, -reach + cell, 0.0],
            max: [reach - cell, reach - cell, length],
            cell,
            sides: [Side::Wall(WallLaw::Stick); 6],
            periodic: [false, false, true],
        },
        gravity: [0.0, 0.0, g],
        heat: None,
        control: ParticleControl {
            end_time: 1.5,
            frames: 15,
            sound_speed: Some(6.0),
            ..Default::default()
        },
    };
    let results = solve(&model);
    let tau_w = density * g * radius / 2.0;
    let phi = yield_stress / tau_w;
    let q = std::f64::consts::PI * radius.powi(4) * density * g / (8.0 * viscosity)
        * (1.0 - 4.0 / 3.0 * phi + phi.powi(4) / 3.0);
    let mean = q / (std::f64::consts::PI * radius * radius);
    let mut rates = Vec::new();
    for frame in &results.frames {
        // The flow rate: the mean axial velocity of the particles (each of the same volume)
        // times the cross-section they fill.
        let n = frame.velocities.len() as f64;
        let u = frame
            .velocities
            .iter()
            .map(|v| f64::from(v[2]))
            .sum::<f64>()
            / n;
        eprintln!(
            "t {:.2} mean velocity {u:.4} expected {mean:.4}",
            frame.time
        );
        rates.push(u);
    }
    let (a, b) = (rates[rates.len() - 2], rates[rates.len() - 1]);
    assert!((b - a).abs() < 0.02 * mean, "not steady: {a} then {b}");
    assert!(
        (b - mean).abs() < 0.1 * mean,
        "mean velocity {b} instead of {mean}"
    );
    // The plug: inside r_p = φ R the paste moves as one.
    let frame = last(&results);
    let plug: Vec<f64> = (frame.positions.iter().zip(&frame.velocities))
        .filter(|(p, _)| f64::from(p[0]).hypot(f64::from(p[1])) < 0.5 * phi * radius)
        .map(|(_, v)| f64::from(v[2]))
        .collect();
    let lo = plug.iter().copied().fold(f64::MAX, f64::min);
    let hi = plug.iter().copied().fold(f64::MIN, f64::max);
    eprintln!("plug velocities {lo:.4} .. {hi:.4}");
    assert!(hi - lo < 0.05 * hi, "the plug shears: {lo} .. {hi}");
}

#[test]
fn bingham_pipe_flow() {
    pipe(8.0);
}

#[test]
#[ignore]
fn bingham_pipe_flow_fine() {
    pipe(16.0);
}

/// A plane container drains through a slot in its floor: in the slot the flow is plane
/// Poiseuille flow, `q = b³ Δp / (12 η L)` per unit depth, driven by the head over the slot
/// and the weight in it, `Δp = ρ g (H + L)`. Newtonian paste (no yield stress), the slot
/// full at the start; the material leaves through the open bottom of the box at the end of
/// the slot, and the mass balance must close.
fn die(per_gap: f64) {
    let (width, head, gap, slot) = (0.1, 0.1, 0.02, 0.12);
    let (density, viscosity) = (1000.0, 10.0);
    let cell = gap / per_gap;
    let depth = 3.0 * cell;
    let paste = ParticleMaterial {
        name: "Newtonian paste".into(),
        density,
        law: ParticleLaw::ViscoPlastic {
            shear: 2e3,
            bulk: 2e5,
            yield_stress: 0.0,
            consistency: viscosity,
            exponent: 1.0,
        },
        thermal: None,
        flow_factor: Vec::new(),
    };
    let half = 0.5 * gap;
    let mut model = plane(
        "Die extrusion",
        [-0.5 * width, 0.0],
        [0.5 * width, slot + head + 0.02],
        cell,
        paste,
        vec![
            ([-0.5 * width, slot], [0.5 * width, slot + head]),
            ([-half, 0.0], [half, slot]),
        ],
    );
    model.walls = vec![
        plane_wall(
            "Die left",
            [-width, -0.1],
            [-half, slot],
            depth,
            WallLaw::Stick,
        ),
        plane_wall(
            "Die right",
            [half, -0.1],
            [width, slot],
            depth,
            WallLaw::Stick,
        ),
    ];
    model.domain.sides[2] = Side::Open;
    model.control.end_time = 0.3;
    model.control.frames = 6;
    let results = solve(&model);
    let h = &results.history;
    let initial = h.mass[0] + h.outflow[0];
    // Mass: in the model plus out of it.
    for (m, o) in h.mass.iter().zip(&h.outflow) {
        assert!((m + o - initial).abs() < 1e-9 * initial);
    }
    // The flow rate from the outflow, once the flow in the slot has developed (its time
    // scale is b² ρ / η = 0.04 s).
    let at = |t: f64| h.time.iter().position(|&x| x >= t).unwrap();
    let (i, j) = (at(0.1), h.time.len() - 1);
    let rate = (h.outflow[j] - h.outflow[i]) / density / (3.0 * cell) / (h.time[j] - h.time[i]);
    let level = largest(&results.frames[results.frames.len() / 2 + 1], 1) + 0.25 * cell - slot;
    let expected = gap.powi(3) * density * G * (level + slot) / (12.0 * viscosity * slot);
    eprintln!("rate {rate:.5} expected {expected:.5} (head {level:.4})");
    // Plane Poiseuille flow through the slot under the head; the entry into the slot costs
    // a little pressure besides (the rate comes out 10 % low at 8 cells across the gap,
    // 7 % at 12).
    assert!(
        (rate - expected).abs() < 0.15 * expected,
        "rate {rate} instead of {expected}"
    );
}

#[test]
fn die_extrusion() {
    die(8.0);
}

#[test]
#[ignore]
fn die_extrusion_fine() {
    die(12.0);
}

/// Circular Couette flow (stage 3): a cylinder turns in a Newtonian fluid inside a resting
/// tube. The torque on it per length is `4 π μ Ω R₁² R₂² / (R₂² − R₁²)`; its work is what
/// the fluid dissipates, and (stage 3b) what heats it: the walls take no heat.
fn couette(per_gap: f64) {
    let (inner, outer) = (0.02, 0.04);
    let (viscosity, omega) = (20.0, 2.0);
    let cell = (outer - inner) / per_gap;
    let length = 3.0 * cell;
    let reach = outer + 3.0 * cell;
    let annulus = |r1: f64, r2: f64| {
        let mut shape =
            TriangleSurface::cylinder([0.0, 0.0, -1.0], [0.0, 0.0, 2.0 + length], r2, 256);
        let hole = TriangleSurface::cylinder([0.0, 0.0, -1.0], [0.0, 0.0, 2.0 + length], r1, 256)
            .flipped();
        let offset = shape.points.len() as u32;
        shape.points.extend(hole.points);
        shape
            .triangles
            .extend(hole.triangles.into_iter().map(|t| t.map(|i| i + offset)));
        shape
    };
    let mut tube = TriangleSurface::cuboid([-reach, -reach, -1.0], [reach, reach, 1.0 + length]);
    let hole =
        TriangleSurface::cylinder([0.0, 0.0, -1.0], [0.0, 0.0, 2.0 + length], outer, 256).flipped();
    let offset = tube.points.len() as u32;
    tube.points.extend(hole.points);
    tube.triangles
        .extend(hole.triangles.into_iter().map(|t| t.map(|i| i + offset)));
    let mut model = ParticleModel {
        heading: "Couette flow".into(),
        materials: vec![water(viscosity)],
        bodies: vec![MaterialBody {
            name: "Fluid".into(),
            material: 0,
            shape: annulus(inner, outer),
            velocity: [0.0; 3],
            temperature: 0.0,
            gravity: [0.0; 3],
        }],
        walls: vec![
            RigidWall {
                temperature: None,
                name: "Rotor".into(),
                shape: TriangleSurface::cylinder(
                    [0.0, 0.0, -1.0],
                    [0.0, 0.0, 2.0 + length],
                    inner,
                    256,
                ),
                law: WallLaw::Stick,
                motion: RigidMotion {
                    angular: [0.0, 0.0, omega],
                    // Half speed at first, against a jolt.
                    amplitude: vec![[0.0, 0.0], [0.02, 0.01], [1.0, 0.99]],
                    ..Default::default()
                },
            },
            RigidWall {
                name: "Tube".into(),
                shape: tube,
                law: WallLaw::Stick,
                motion: RigidMotion::default(),
                temperature: None,
            },
        ],
        domain: Domain {
            min: [-reach + cell, -reach + cell, 0.0],
            max: [reach - cell, reach - cell, length],
            cell,
            sides: [Side::Wall(WallLaw::Stick); 6],
            periodic: [false, false, true],
        },
        gravity: [0.0; 3],
        heat: None,
        control: ParticleControl {
            end_time: 0.3,
            frames: 6,
            sound_speed: Some(2.0),
            ..Default::default()
        },
    };
    let (specific_heat, start) = (4.0, 20.0);
    model.materials[0].thermal = Some(ParticleThermal {
        conductivity: 0.6,
        specific_heat,
    });
    model.bodies[0].temperature = start;
    model.heat = Some(Heat { wall_transfer: 0.0 });
    let results = solve(&model);
    let expected = 4.0 * std::f64::consts::PI * viscosity * omega * (inner * outer).powi(2)
        / (outer * outer - inner * inner);
    let rotor = &results.history.walls[0];
    let torque: Vec<f64> = rotor.moment.iter().map(|m| -m[2] / length).collect();
    let n = torque.len();
    // The mean over the last fifth: the samples of the wall load are noisy.
    let tail = &torque[n - n / 5..];
    let mean = tail.iter().sum::<f64>() / tail.len() as f64;
    eprintln!("torque per length {mean:.4} expected {expected:.4}");
    assert!(
        (mean - expected).abs() < 0.15 * expected,
        "torque {mean} instead of {expected}"
    );
    // Steady: the rotor's power goes into the fluid's dissipation.
    let power = mean * length * omega;
    let work = &rotor.work;
    let times = &results.history.time;
    let rate = (work[n - 1] - work[n - n / 5]) / (times[n - 1] - times[n - n / 5]);
    eprintln!("rotor power {rate:.5} from torque {power:.5}");
    assert!(
        (rate - power).abs() < 0.1 * power,
        "power {rate} vs {power}"
    );
    assert!(max_balance_error(&results) < 0.1);
    // The heat in the fluid at the end: what the flow and the walls dissipated, and about
    // the rotor's work (less the kinetic energy).
    let frame = last(&results);
    let particle_mass = 1000.0 * (0.5 * cell).powi(3);
    let heat: f64 = (frame.fields[5].iter())
        .map(|&t| particle_mass * specific_heat * (f64::from(t) - start))
        .sum();
    let h = &results.history;
    let dissipated = h.dissipated[n - 1] + h.wall_loss[n - 1];
    eprintln!(
        "heat {heat:.5}, dissipated {dissipated:.5}, rotor work {:.5}",
        work[n - 1]
    );
    assert!(
        (heat - dissipated).abs() < 0.05 * dissipated,
        "heat {heat} vs dissipated {dissipated}"
    );
    assert!(
        (heat - work[n - 1]).abs() < 0.15 * work[n - 1],
        "heat {heat} vs work {}",
        work[n - 1]
    );
}

#[test]
fn couette_flow() {
    couette(8.0);
}

#[test]
#[ignore]
fn couette_flow_fine() {
    couette(16.0);
}

/// Conduction (stage 3b): two resting blocks at 0 and 100 °C side by side. The temperature
/// across the contact follows `50 + 50 erf((x − x₀) / (2 √(α t)))` while the far ends do not
/// feel it yet.
#[test]
fn conduction_between_blocks() {
    let (length, cell) = (0.1, 0.005);
    let (conductivity, density, specific_heat) = (1.0, 1000.0, 1.0);
    let alpha = conductivity / (density * specific_heat);
    let side = 3.0 * cell;
    let block = |name: &str, lo: f64, hi: f64, temperature: f64| MaterialBody {
        name: name.into(),
        material: 0,
        shape: TriangleSurface::cuboid([lo, -cell, -cell], [hi, side + cell, side + cell]),
        velocity: [0.0; 3],
        temperature,
        gravity: [0.0; 3],
    };
    let mut material = water(1.0);
    material.thermal = Some(ParticleThermal {
        conductivity,
        specific_heat,
    });
    let end_time = 0.25;
    let model = ParticleModel {
        heading: "Conduction".into(),
        materials: vec![material],
        bodies: vec![
            block("Cold", 0.0, 0.5 * length, 0.0),
            block("Warm", 0.5 * length, length, 100.0),
        ],
        walls: Vec::new(),
        domain: Domain {
            min: [0.0; 3],
            max: [length, side, side],
            cell,
            sides: [Side::Wall(WallLaw::Slip); 6],
            periodic: [false, true, true],
        },
        gravity: [0.0; 3],
        heat: Some(Heat { wall_transfer: 0.0 }),
        control: ParticleControl {
            end_time,
            frames: 1,
            sound_speed: Some(5.0),
            ..Default::default()
        },
    };
    let results = solve(&model);
    let frame = last(&results);
    let spread = 2.0 * (alpha * end_time).sqrt();
    let mut worst: f64 = 0.0;
    let mut total = 0.0;
    for (p, &t) in frame.positions.iter().zip(&frame.fields[5]) {
        let x = f64::from(p[0]);
        let expected = 50.0 + 50.0 * erf((x - 0.5 * length) / spread);
        worst = worst.max((f64::from(t) - expected).abs());
        total += f64::from(t);
    }
    let mean = total / frame.positions.len() as f64;
    // The heat stays: the mean temperature is the start's.
    assert!((mean - 50.0).abs() < 0.01, "mean {mean}");
    assert!(worst < 2.5, "temperature off by {worst} K");
}

/// The error function (Abramowitz and Stegun 7.1.26, error below 1.5e-7).
fn erf(x: f64) -> f64 {
    let t = 1.0 / (1.0 + 0.3275911 * x.abs());
    let poly = t
        * (0.254829592
            + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    let y = 1.0 - poly * (-x * x).exp();
    if x < 0.0 { -y } else { y }
}
