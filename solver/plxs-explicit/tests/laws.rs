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

/// The hyperelastic models at small and at large strains (constants in MPa).
#[test]
fn hyperelastic_laws_are_the_processors() {
    use plxs_core::material::{Hyperelastic as Definition, HyperelasticModel as Model};
    let models: [(Model, &[f64]); 7] = [
        (Model::NeoHooke, &[0.5, 0.02]),
        (Model::MooneyRivlin, &[0.4, 0.1, 0.02]),
        (
            Model::Polynomial(2),
            &[0.4, 0.1, 0.01, -0.005, 0.002, 0.02, 0.0],
        ),
        (Model::Yeoh, &[0.5, -0.02, 0.003, 0.02, 0.0, 0.0]),
        (
            Model::Ogden(3),
            &[0.6, 1.3, 0.002, 5.0, -0.01, -2.0, 0.02, 0.0, 0.0],
        ),
        (Model::ArrudaBoyce, &[0.5, 2.5, 0.02]),
        // Nearly incompressible: the pressure from J − 1 of tiny strains.
        (Model::NeoHooke, &[0.5, 0.0001]),
    ];
    for (model, constants) in models {
        let card = Card {
            name: "RUBBER".into(),
            hyperelastic: Some(Definition {
                model,
                constants: constants.to_vec(),
            }),
            ..Default::default()
        };
        let law = law_of(&card, 0.0, true).unwrap();
        let Some(KernelLaw::Hyperelastic {
            isochoric,
            volumetric,
        }) = law.kernel_law()
        else {
            panic!("no kernel law");
        };
        let kernel = Material::Hyperelastic {
            isochoric: convert_isochoric(isochoric),
            volumetric: convert_volumetric(volumetric),
        };
        for scale in [1e-5, 1e-2, 0.3, 1.0] {
            for t in [0.1, 0.25, 0.4, 0.7, 0.9] {
                let strain = path(t).map(|e| e * scale / 0.08);
                let input = MaterialInput {
                    strain,
                    temperature: 0.0,
                    dt: 0.0,
                    time: 0.0,
                    creep_active: false,
                };
                let (expected, _) = law.stress_without_tangent(&input, &mut []);
                let got = kernel.stress(&strain, None);
                let size = expected.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
                for k in 0..6 {
                    assert!(
                        (expected[k] - got[k]).abs() <= 1e-7 * size,
                        "{model:?} at {strain:?}: stress {k}: {} against {}",
                        got[k],
                        expected[k]
                    );
                }
            }
        }
    }
}

fn convert_isochoric(i: plxs_materials::hyper::Isochoric) -> plxs_explicit::Isochoric {
    use plxs_materials::hyper::Isochoric as I;
    match i {
        I::Polynomial(t) => plxs_explicit::Isochoric::Polynomial(t),
        I::ArrudaBoyce { mu, locking } => plxs_explicit::Isochoric::ArrudaBoyce { mu, locking },
        I::Ogden(t) => plxs_explicit::Isochoric::Ogden(t),
    }
}

fn convert_volumetric(v: plxs_materials::hyper::Volumetric) -> plxs_explicit::Volumetric {
    use plxs_materials::hyper::Volumetric as V;
    match v {
        V::Polynomial(c) => plxs_explicit::Volumetric::Polynomial(c),
        V::ArrudaBoyce(a) => plxs_explicit::Volumetric::ArrudaBoyce(a),
    }
}
