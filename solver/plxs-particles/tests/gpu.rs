//! The time steps on the graphics card against the processor's: a paste stirred by a warm
//! paddle that turns with Coulomb friction, under gravity, with a flow factor over the
//! temperature. The card computes in single precision and sums in another order, so the
//! two runs part like two runs of a flow with slightly different starts: compared are the
//! sums of the run, not single particles. Runs where a graphics card is found
//! (of any kind, software renderers such as lavapipe included); skipped where `PLXS_GPU` is
//! set, which would choose for both runs.
#![cfg(feature = "gpu")]

use plxs_core::particles::{
    Domain, Heat, MaterialBody, ParticleControl, ParticleLaw, ParticleMaterial, ParticleModel,
    ParticleResults, ParticleThermal, RigidMotion, RigidWall, Side, TriangleSurface, WallLaw,
};
use plxs_linalg::amg::GpuMode;

fn mixer() -> ParticleModel {
    let cell = 0.004;
    ParticleModel {
        heading: "Mixer".into(),
        materials: vec![ParticleMaterial {
            name: "Paste".into(),
            density: 1100.0,
            law: ParticleLaw::ViscoPlastic {
                shear: 2e4,
                bulk: 2e6,
                yield_stress: 150.0,
                consistency: 20.0,
                exponent: 0.5,
            },
            thermal: Some(ParticleThermal {
                conductivity: 0.5,
                specific_heat: 3500.0,
            }),
            flow_factor: vec![[0.0, 1.0], [40.0, 0.6]],
        }],
        bodies: vec![MaterialBody {
            name: "Paste".into(),
            material: 0,
            shape: TriangleSurface::cuboid([-0.03, -0.03, 0.0], [0.03, 0.03, 0.024]),
            velocity: [0.0; 3],
            temperature: 4.0,
            gravity: [0.0; 3],
        }],
        walls: vec![RigidWall {
            name: "Paddle".into(),
            shape: TriangleSurface::cuboid([-0.022, -0.004, 0.004], [0.022, 0.004, 0.03]),
            law: WallLaw::Coulomb { friction: 0.3 },
            motion: RigidMotion {
                angular: [0.0, 0.0, 1.0],
                amplitude: vec![[0.0, 0.0], [0.05, 0.25], [0.3, 3.0]],
                ..Default::default()
            },
            temperature: Some(40.0),
        }],
        domain: Domain {
            min: [-0.03, -0.03, 0.0],
            max: [0.03, 0.03, 0.04],
            cell,
            sides: [
                Side::Wall(WallLaw::Stick),
                Side::Wall(WallLaw::Stick),
                Side::Wall(WallLaw::Slip),
                Side::Wall(WallLaw::Slip),
                Side::Wall(WallLaw::Coulomb { friction: 0.2 }),
                Side::Open,
            ],
            periodic: [false; 3],
            ..Default::default()
        },
        gravity: [0.0, 0.0, -9.81],
        control: ParticleControl {
            end_time: 0.3,
            frames: 3,
            sound_speed: Some(4.0),
            ..Default::default()
        },
        heat: Some(Heat {
            wall_transfer: 2000.0,
        }),
    }
}

/// The choice of the card is global: one run at a time.
static CHOICE: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn solve(mode: GpuMode, model: &ParticleModel) -> Result<ParticleResults, String> {
    let _one = CHOICE.lock().unwrap_or_else(|e| e.into_inner());
    GpuMode::set_preferred(mode);
    plxs_particles::solve(model, &mut |_| {})
}

fn run(mode: GpuMode, model: &ParticleModel) -> ParticleResults {
    let results = solve(mode, model).expect("solves");
    eprintln!(
        "{mode:?}: {} particles, {} steps, {:.1} s",
        results.report.particles, results.report.steps, results.report.seconds
    );
    for n in &results.report.notes {
        eprintln!("  {n}");
    }
    results
}

fn mean(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len().max(1) as f64
}

#[test]
fn the_card_computes_what_the_processor_computes() {
    if std::env::var("PLXS_GPU").is_ok() {
        eprintln!("skipped: PLXS_GPU would choose for both runs");
        return;
    }
    let model = mixer();
    let card = run(GpuMode::On, &model);
    if !card
        .report
        .notes
        .iter()
        .any(|n| n.contains("computed on") && !n.contains("processor"))
    {
        eprintln!("skipped: no graphics card");
        return;
    }
    let cpu = run(GpuMode::Off, &model);
    GpuMode::set_preferred(GpuMode::Automatic);

    // The time steps follow the same speeds.
    let steps = card.report.steps as f64 / cpu.report.steps as f64;
    assert!((steps - 1.0).abs() < 0.005, "steps {steps}");
    assert_eq!(card.frames.len(), cpu.frames.len());
    let (a, b) = (card.frames.last().unwrap(), cpu.frames.last().unwrap());
    assert_eq!(a.ids, b.ids, "the same particles are left");
    // Where the paste is: the mean position and the mean of the distances to the axis.
    let spread = |f: &plxs_core::particles::ParticleFrame| {
        let n = f.positions.len() as f64;
        let mut m = [0.0f64; 4];
        for x in &f.positions {
            m[0] += f64::from(x[0]) / n;
            m[1] += f64::from(x[1]) / n;
            m[2] += f64::from(x[2]) / n;
            m[3] += f64::from(x[0]).hypot(f64::from(x[1])) / n;
        }
        m
    };
    let (ha, hb) = (&card.history, &cpu.history);
    for k in (0..ha.time.len()).step_by(ha.time.len() / 8) {
        eprintln!(
            "t {:.3}: kinetic {:.4e} / {:.4e}, torque {:.4e} / {:.4e}, warmest {:.3} / {:.3}",
            ha.time[k],
            ha.kinetic[k],
            hb.kinetic[k],
            ha.walls[0].moment[k][2],
            hb.walls[0].moment[k][2],
            ha.bodies[0].temperature_max[k],
            hb.bodies[0].temperature_max[k],
        );
    }
    let (sa, sb) = (spread(a), spread(b));
    eprintln!("spread card {sa:?} processor {sb:?}");
    for k in 0..4 {
        assert!(
            (sa[k] - sb[k]).abs() < 0.01 * model.domain.cell,
            "{k}: {sa:?} {sb:?}"
        );
    }
    // The energies and the loads on the paddle over the run.
    let last = |v: &[f64]| *v.last().unwrap();
    let mut failed = Vec::new();
    for (name, x, y, tolerance) in [
        // At one time: what the flow does just then.
        ("kinetic", last(&ha.kinetic), last(&hb.kinetic), 0.02),
        // Summed over the run.
        (
            "dissipated",
            last(&ha.dissipated),
            last(&hb.dissipated),
            0.005,
        ),
        ("wall work", last(&ha.wall_work), last(&hb.wall_work), 0.005),
        (
            "paddle work",
            last(&ha.walls[0].work),
            last(&hb.walls[0].work),
            0.005,
        ),
        (
            "paddle heat",
            last(&ha.walls[0].heat),
            last(&hb.walls[0].heat),
            0.005,
        ),
        (
            "torque",
            mean(&ha.walls[0].moment.iter().map(|m| m[2]).collect::<Vec<_>>()),
            mean(&hb.walls[0].moment.iter().map(|m| m[2]).collect::<Vec<_>>()),
            0.005,
        ),
        (
            "floor",
            mean(&ha.walls[5].force.iter().map(|f| f[2]).collect::<Vec<_>>()),
            mean(&hb.walls[5].force.iter().map(|f| f[2]).collect::<Vec<_>>()),
            0.005,
        ),
        // Relative to the warming of the paste.
        (
            "mean warming",
            last(&ha.bodies[0].temperature_mean) - 4.0,
            last(&hb.bodies[0].temperature_mean) - 4.0,
            0.005,
        ),
        (
            "highest warming",
            last(&ha.bodies[0].temperature_max) - 4.0,
            last(&hb.bodies[0].temperature_max) - 4.0,
            0.01,
        ),
    ] {
        let off = (x - y).abs() / y.abs().max(1e-12);
        eprintln!(
            "{name}: card {x:.6e} processor {y:.6e} ({:.2} %)",
            100.0 * off
        );
        if off > tolerance {
            failed.push(name);
        }
    }
    assert!(failed.is_empty(), "card and processor differ in {failed:?}");
    let error = |h: &plxs_core::particles::ParticleHistory| {
        h.balance_error.iter().fold(0.0f64, |m, e| m.max(e.abs()))
    };
    eprintln!(
        "balance error card {:.4} processor {:.4}",
        error(ha),
        error(hb)
    );
    assert!((error(ha) - error(hb)).abs() < 0.005);
    assert_eq!(ha.time.len(), hb.time.len());
}

/// A run that blows up (time steps four times too long) stops with an error on both, and
/// the card does not hang on particles whose position is no number.
#[test]
fn an_unstable_run_stops_on_the_card_as_on_the_processor() {
    if std::env::var_os("PLXS_GPU").is_some() {
        return;
    }
    let mut model = mixer();
    model.heat = None;
    model.control.courant = 4.0;
    model.control.end_time = 0.1;
    for mode in [GpuMode::Off, GpuMode::On] {
        let started = std::time::Instant::now();
        let error = solve(mode, &model).expect_err("blows up");
        eprintln!(
            "{mode:?}: {error} after {:.1} s",
            started.elapsed().as_secs_f64()
        );
        assert!(error.contains("unstable"), "{mode:?}: {error}");
    }
}
