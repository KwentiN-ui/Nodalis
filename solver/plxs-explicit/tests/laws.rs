//! The kernels' material laws in double precision ([`Material::stress`], the reference of
//! the card) against the processor's laws: the same stresses and state variables along a
//! strain path that yields, unloads and yields again.

use plxs_core::material::{Elastic, ElasticRow, Hardening, Material as Card, Plastic, PlasticRow};
use plxs_explicit::{Material, STATE_SIZE};
use plxs_materials::{KernelLaw, MaterialInput, law_of};

fn steel(hardening: Hardening) -> Card {
    Card {
        name: "STEEL".into(),
        elastic: Some(Elastic {
            rows: vec![ElasticRow {
                young: 210e9,
                poisson: 0.3,
                temperature: 0.0,
            }],
        }),
        plastic: Some(Plastic {
            hardening,
            rows: [(235e6, 0.0), (300e6, 0.002), (400e6, 0.2)]
                .iter()
                .map(|&(stress, plastic_strain)| PlasticRow {
                    stress,
                    plastic_strain,
                    temperature: 0.0,
                })
                .collect(),
        }),
        ..Default::default()
    }
}

/// A Green-Lagrange strain (engineering components) along a path through stretch, shear
/// and back, far into the plastic range.
fn path(t: f64) -> [f64; 6] {
    let a = 0.08 * (std::f64::consts::PI * t).sin();
    let b = 0.05 * (2.0 * std::f64::consts::PI * t).sin();
    [a, -0.3 * a + 0.2 * b, -0.35 * a, 0.6 * b, 0.1 * a, -0.4 * b]
}

fn compare(hardening: Hardening, finite: bool) {
    let law = law_of(&steel(hardening), 0.0, finite).unwrap();
    let Some(KernelLaw::J2 {
        shear,
        bulk,
        isotropic,
        kinematic,
        finite: kernel_finite,
    }) = law.kernel_law()
    else {
        panic!("no kernel law");
    };
    assert_eq!(kernel_finite, finite);
    let kernel = Material::J2 {
        shear,
        bulk,
        isotropic,
        kinematic,
        finite,
    };
    let size = law.state_size();
    assert_eq!(size, STATE_SIZE);
    let (mut processor, mut card) = (vec![0.0; size], vec![0.0; size]);
    let steps = 400;
    let mut largest = 0.0_f64;
    for i in 1..=steps {
        let strain = path(i as f64 / steps as f64);
        let input = MaterialInput {
            strain,
            temperature: 0.0,
            dt: 1e-6,
            time: 0.0,
            creep_active: false,
        };
        let (expected, _) = law.stress_without_tangent(&input, &mut processor);
        let got = kernel.stress(&strain, Some(&mut card));
        let scale = expected.iter().fold(1e6_f64, |m, v| m.max(v.abs()));
        for k in 0..6 {
            assert!(
                (expected[k] - got[k]).abs() <= 1e-8 * scale,
                "step {i}: stress {k}: {} against {}",
                got[k],
                expected[k]
            );
        }
        for k in 0..size {
            assert!(
                (processor[k] - card[k]).abs() <= 1e-8 * processor[k].abs().max(1e-4),
                "step {i}: state {k}: {} against {}",
                card[k],
                processor[k]
            );
        }
        largest = largest.max(processor[0]);
    }
    assert!(largest > 0.01, "{largest}");
}

#[test]
fn small_strain_j2_is_the_processors() {
    compare(Hardening::Isotropic, false);
    compare(Hardening::Kinematic, false);
    compare(Hardening::Combined, false);
}

#[test]
fn finite_strain_j2_is_the_processors() {
    compare(Hardening::Isotropic, true);
    compare(Hardening::Kinematic, true);
    compare(Hardening::Combined, true);
}
