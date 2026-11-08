//! A block of bricks on the graphics card with the element kernels against the same central
//! differences with the element forces of [`Elements::forces`] in double precision on the
//! processor. Runs where a card with double precision is found (`PLXS_GPU=on` takes
//! software renderers too); elsewhere it says so and passes.

use plxs_explicit::{
    Block, Element, Elements, Engine, Hourglass, Isochoric, MAX_CHUNK, Material, Points, Reduction,
    Setup, Start, Volumetric,
};

/// The trilinear brick's natural node coordinates.
const CORNERS: [[f64; 3]; 8] = [
    [-1.0, -1.0, -1.0],
    [1.0, -1.0, -1.0],
    [1.0, 1.0, -1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [1.0, -1.0, 1.0],
    [1.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0],
];

/// The shape function derivatives of an axis-parallel brick of sides `s` at the natural
/// point `p`.
fn gradients(p: [f64; 3], s: [f64; 3]) -> Vec<[f64; 3]> {
    CORNERS
        .iter()
        .map(|c| {
            let f = |i: usize| 1.0 + c[i] * p[i];
            [
                c[0] * f(1) * f(2) / 8.0 * 2.0 / s[0],
                c[1] * f(0) * f(2) / 8.0 * 2.0 / s[1],
                c[2] * f(0) * f(1) / 8.0 * 2.0 / s[2],
            ]
        })
        .collect()
}

/// The dense operator `L` (9 × 24) of nodal gradients `g`, plus `H_zz += Σ_a u_ax / (8 r)`
/// with `r` fifty element lengths `h`: a translation along x strains it.
fn operator(g: &[[f64; 3]], h: f64) -> Vec<f64> {
    let nd = 3 * g.len();
    let mut l = vec![0.0; 9 * nd];
    for (a, ga) in g.iter().enumerate() {
        for k in 0..3 {
            for c in 0..3 {
                l[(3 * k + c) * nd + 3 * a + k] = ga[c];
            }
        }
        l[8 * nd + 3 * a] += 1.0 / (8.0 * 50.0 * h);
    }
    l
}

/// A constant stiffness of a brick: the hourglass stiffness `ε Σ γ γᵀ ⊗ I` (CalculiX's
/// `γ`) and a weak foundation `k_0 / 8` on every DOF, which translations load.
fn constant_matrix(x: &[[f64; 3]], g: &[[f64; 3]], epsilon: f64, k0: f64) -> Vec<f64> {
    const BASE: [[f64; 8]; 4] = [
        [1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0],
        [1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0],
        [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0],
    ];
    let mut k = vec![0.0; 24 * 24];
    for h in &BASE {
        let hz: f64 = (0..8).map(|a| h[a] * x[a][2]).sum();
        let gamma: Vec<f64> = (0..8).map(|a| h[a] - hz * g[a][2]).collect();
        for a in 0..8 {
            for b in 0..8 {
                for d in 0..3 {
                    k[(3 * a + d) * 24 + 3 * b + d] += epsilon * gamma[a] * gamma[b];
                }
            }
        }
    }
    for i in 0..24 {
        k[i * 24 + i] += k0 / 8.0;
    }
    k
}

fn isotropic(young: f64, poisson: f64) -> [[f64; 6]; 6] {
    let lambda = young * poisson / ((1.0 + poisson) * (1.0 - 2.0 * poisson));
    let mu = young / (2.0 * (1.0 + poisson));
    let mut d = [[0.0; 6]; 6];
    for i in 0..3 {
        d[i][..3].fill(lambda);
        d[i][i] = lambda + 2.0 * mu;
        d[i + 3][i + 3] = mu;
    }
    d
}

#[test]
fn a_block_of_bricks_swings_as_on_the_processor() {
    swing(false, 0.3, false);
}

/// Fast enough to yield: J2 plasticity, isotropic and kinematic hardening.
#[test]
fn a_block_of_bricks_yields_as_on_the_processor() {
    swing(false, 20.0, true);
}

/// Under NLGEOM, swinging far enough that the rotations matter.
#[test]
fn a_block_of_bricks_swings_far_as_on_the_processor() {
    swing(true, 400.0, false);
}

/// Under NLGEOM and yielding: J2 at finite strains.
#[test]
fn a_block_of_bricks_yields_far_as_on_the_processor() {
    swing(true, 60.0, true);
}

/// Under NLGEOM, hyperelastic: Mooney-Rivlin, Ogden and Arruda-Boyce with the moduli of
/// steel, so that the time step stays.
#[test]
fn a_block_of_hyperelastic_bricks_swings_far_as_on_the_processor() {
    swing_with(true, 400.0, false, true, false);
}

/// A cantilever block of bricks starting with `speed` across, pulled at its free end;
/// `plastic`: of J2 materials.
fn swing(nlgeom: bool, speed: f64, plastic: bool) {
    swing_with(nlgeom, speed, plastic, false, false);
}

/// General operators: every element's gradients as a dense `L`, with a term that a
/// translation strains (as the hoop strain of axisymmetric elements), and a constant
/// stiffness on the reduced ones in place of the hourglass control (with a part that
/// holds translations).
#[test]
fn a_block_of_dense_bricks_swings_as_on_the_processor() {
    swing_with(false, 0.3, false, false, true);
}

/// Dense operators under NLGEOM, yielding.
#[test]
fn a_block_of_dense_bricks_yields_far_as_on_the_processor() {
    swing_with(true, 25.0, true, false, true);
}

/// `swing`, `hyper`: of hyperelastic materials, `dense`: by general operators.
fn swing_with(nlgeom: bool, speed: f64, plastic: bool, hyper: bool, dense: bool) {
    if std::env::var("PLXS_GPU").is_ok_and(|v| v.eq_ignore_ascii_case("off")) {
        return;
    }
    let (nx, ny, nz) = (8, 3, 3);
    let s = [0.01, 0.01, 0.01];
    let (young, rho): (f64, f64) = (2.1e11, 7850.0);
    let node = |i: usize, j: usize, k: usize| (k * (ny + 1) + j) * (nx + 1) + i;
    let nodes = (nx + 1) * (ny + 1) * (nz + 1);
    let n = 3 * nodes;
    let mut mass = vec![0.0; n];
    let mut states = 0;
    let mut elements = Elements {
        nlgeom,
        ..Default::default()
    };
    let dt_e = s[0] / (young / rho).sqrt();
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let ids: Vec<usize> = CORNERS
                    .iter()
                    .map(|c| {
                        let at = |x: f64| usize::from(x > 0.0);
                        node(i + at(c[0]), j + at(c[1]), k + at(c[2]))
                    })
                    .collect();
                let coordinates: Vec<[f64; 3]> = ids
                    .iter()
                    .map(|&id| {
                        let (a, rest) = (id % (nx + 1), id / (nx + 1));
                        let (b, c) = (rest % (ny + 1), rest / (ny + 1));
                        [a as f64 * s[0], b as f64 * s[1], c as f64 * s[2]]
                    })
                    .collect();
                let volume = s[0] * s[1] * s[2];
                for &id in &ids {
                    for d in 0..3 {
                        mass[3 * id + d] += rho * volume / 8.0;
                    }
                }
                let mean = gradients([0.0; 3], s);
                // Alternating: the uniform-strain brick with hourglass control and the
                // fully integrated one.
                let reduced = (i + j + k) % 2 == 0;
                let points = if reduced {
                    vec![(volume, mean.clone())]
                } else {
                    let g = 1.0 / 3.0_f64.sqrt();
                    CORNERS
                        .iter()
                        .map(|c| (volume / 8.0, gradients([g * c[0], g * c[1], g * c[2]], s)))
                        .collect()
                };
                let count = points.len();
                elements
                    .push(Element {
                        dofs: ids
                            .iter()
                            .flat_map(|&id| (0..3).map(move |d| (3 * id + d) as u32))
                            .collect(),
                        points: if dense {
                            Points::Dense(
                                points
                                    .iter()
                                    .map(|(w, g)| (*w, operator(g, s[0])))
                                    .collect(),
                            )
                        } else {
                            Points::Nodal(points)
                        },
                        matrix: (dense && reduced).then(|| {
                            let mean = gradients([0.0; 3], s);
                            constant_matrix(
                                &coordinates,
                                &mean,
                                0.004 * young * volume * 0.5,
                                1e-3 * young * s[0],
                            )
                        }),
                        material: if plastic {
                            let mu = young / 2.6;
                            let bulk = young / (3.0 * 0.4);
                            let curve = vec![(0.0, 235e6), (0.002, 300e6), (0.2, 400e6)];
                            // Every third element with kinematic hardening.
                            let (isotropic, kinematic) = if (i + j) % 3 == 0 {
                                (
                                    vec![(0.0, 235e6)],
                                    curve.iter().map(|&(e, s)| (e, s - 235e6)).collect(),
                                )
                            } else {
                                (curve, Vec::new())
                            };
                            Material::J2 {
                                shear: mu,
                                bulk,
                                isotropic,
                                kinematic,
                                finite: nlgeom,
                            }
                        } else if hyper {
                            let mu = young / 2.6;
                            let inverse_d = young / (3.0 * 0.4) / 2.0;
                            match (i + j + k) % 3 {
                                0 => Material::Hyperelastic {
                                    isochoric: Isochoric::Polynomial(vec![
                                        (1, 0, 0.4 * mu),
                                        (0, 1, 0.1 * mu),
                                    ]),
                                    volumetric: Volumetric::Polynomial(vec![inverse_d]),
                                },
                                1 => Material::Hyperelastic {
                                    isochoric: Isochoric::Ogden(vec![
                                        (0.9 * mu, 1.3),
                                        (0.1 * mu, -2.0),
                                    ]),
                                    volumetric: Volumetric::Polynomial(vec![inverse_d, 0.0]),
                                },
                                _ => Material::Hyperelastic {
                                    isochoric: Isochoric::ArrudaBoyce {
                                        mu: mu * 0.95,
                                        locking: 2.5,
                                    },
                                    volumetric: Volumetric::ArrudaBoyce(inverse_d),
                                },
                            }
                        } else {
                            Material::Elastic(isotropic(young, 0.3))
                        },
                        state: plastic.then(|| {
                            let first = states;
                            states += 14 * count;
                            first
                        }),
                        bulk: 0.06 * young * dt_e * volume,
                        mean: mean.iter().flatten().copied().collect(),
                        hourglass: (reduced && !dense).then(|| Hourglass {
                            stiffness: 0.004 * young * volume * 0.5,
                            viscosity: 0.1 * rho * (young / rho).sqrt() * s[0] * s[0] / 4.0,
                            coordinates,
                        }),
                    })
                    .unwrap();
            }
        }
    }
    assert_eq!(elements.families.len(), 2);
    // Under small strains constant blocks too: a spring on every tip node across and a
    // stiff link between two nodes along the bar.
    if !nlgeom {
        for k in 0..=nz {
            for j in 0..=ny {
                elements
                    .push_block(Block {
                        dofs: vec![(3 * node(nx, j, k) + 1) as u32],
                        stiffness: vec![1e6],
                    })
                    .unwrap();
            }
        }
        let (a, b) = (node(2, 0, 0), node(6, ny, nz));
        let k0 = 1e7;
        let mut stiffness = vec![0.0; 36];
        for d in 0..3 {
            for (r, c, v) in [
                (d, d, k0),
                (d + 3, d + 3, k0),
                (d, d + 3, -k0),
                (d + 3, d, -k0),
            ] {
                stiffness[r * 6 + c] = v;
            }
        }
        elements
            .push_block(Block {
                dofs: (0..3)
                    .map(|d| (3 * a + d) as u32)
                    .chain((0..3).map(|d| (3 * b + d) as u32))
                    .collect(),
                stiffness,
            })
            .unwrap();
    }
    let mut inverse: Vec<f64> = mass.iter().map(|m| 1.0 / m).collect();
    // The face at x = 0 is held.
    for k in 0..=nz {
        for j in 0..=ny {
            for d in 0..3 {
                inverse[3 * node(0, j, k) + d] = 0.0;
            }
        }
    }
    let alpha = 50.0;
    let reduction = Reduction::selection(&inverse);
    let free: Vec<usize> = (0..n).filter(|&i| inverse[i] > 0.0).collect();
    let pick = |x: &[f64]| -> Vec<f64> { free.iter().map(|&i| x[i]).collect() };
    let setup = Setup {
        elements: &elements,
        mass: &mass,
        reduction: &reduction,
        alpha,
        prescribed_motion: false,
        contact: None,
    };
    let mut engine = match Engine::new(&setup, true) {
        Ok(e) => e,
        // Only the lack of a card skips the test, not a failing kernel.
        Err(e) if e.contains("graphics card") || e.contains("double precision") => {
            eprintln!("no graphics card for the test: {e}");
            return;
        }
        Err(e) => panic!("{e}"),
    };
    // A load on the far end that grows linearly, and a start velocity across.
    let tip: Vec<usize> = (0..=nz)
        .flat_map(|k| (0..=ny).map(move |j| (j, k)))
        .map(|(j, k)| 3 * node(nx, j, k) + 1)
        .collect();
    let load = |t: f64| {
        let mut p = vec![0.0; n];
        for &d in &tip {
            p[d] = 2.0e3 * t / 1e-4;
        }
        p
    };
    let h = 0.3 * dt_e;
    let steps = 2 * MAX_CHUNK + 31;
    let mut u = vec![0.0; n];
    let mut v: Vec<f64> = (0..n)
        .map(|i| {
            if inverse[i] > 0.0 && i % 3 == 2 {
                speed
            } else {
                0.0
            }
        })
        .collect();
    let state = std::cell::RefCell::new(vec![0.0; states]);
    let forces = |u: &[f64], v: &[f64]| {
        let (fi, mut fv) = elements.forces(u, v, &mut state.borrow_mut());
        for i in 0..n {
            fv[i] += alpha * mass[i] * v[i];
        }
        (fi, fv)
    };
    let accel = |p: &[f64], fi: &[f64], fv: &[f64]| -> Vec<f64> {
        (0..n)
            .map(|i| (p[i] - fi[i] - fv[i]) * inverse[i])
            .collect()
    };
    let (fi0, fv0) = forces(&u, &v);
    let start_state = state.borrow().clone();
    let p0 = load(0.0);
    let a0 = accel(&p0, &fi0, &fv0);
    engine
        .load(&Start {
            q: &pick(&u),
            qdot: &pick(&v),
            a: &pick(&a0),
            prescribed: &vec![0.0; n],
            prescribed_rate: &vec![0.0; n],
            reactions: &[],
            u: &u,
            v: &v,
            internal: &fi0,
            viscous: &fv0,
            loads: &p0,
            material: &start_state,
            contact: None,
        })
        .unwrap();
    let mut a = a0;
    let (mut fi, mut fv, mut p) = (fi0, fv0, p0);
    let mut work = [0.0; 3];
    let mut t = 0.0;
    let mut done = 0;
    while done < steps {
        let chunk = (steps - done).min(MAX_CHUNK);
        let list: Vec<(f64, f64)> = (0..chunk)
            .map(|j| (h, if done + j == 0 { 0.5 * h } else { h }))
            .collect();
        let from = load(t);
        let to = load(t + chunk as f64 * h);
        let zero = vec![0.0; n];
        engine.run(&list, [&from, &to], [&zero, &zero]).unwrap();
        for (j, &(h, cf)) in list.iter().enumerate() {
            let mut du = vec![0.0; n];
            for i in 0..n {
                if inverse[i] > 0.0 {
                    v[i] += cf * a[i];
                    du[i] = h * v[i];
                    u[i] += du[i];
                }
            }
            let s = (j + 1) as f64 / chunk as f64;
            let p_new: Vec<f64> = (0..n).map(|i| from[i] + s * (to[i] - from[i])).collect();
            let (fi_new, fv_new) = forces(&u, &v);
            for i in 0..n {
                work[0] += du[i] * 0.5 * (fi[i] + fi_new[i]);
                work[1] += du[i] * 0.5 * (p[i] + p_new[i]);
                work[2] += du[i] * 0.5 * (fv[i] + fv_new[i]);
            }
            a = accel(&p_new, &fi_new, &fv_new);
            (fi, fv, p) = (fi_new, fv_new, p_new);
        }
        t += chunk as f64 * h;
        done += chunk;
    }
    let reached = engine.read().unwrap();
    if plastic {
        let mut on_card = vec![0.0; states];
        engine.read_state(&mut on_card).unwrap();
        let cpu = state.borrow();
        let peeq = cpu.iter().step_by(14).fold(0.0_f64, |m, x| m.max(*x));
        eprintln!("largest equivalent plastic strain {peeq}");
        assert!(peeq > 1e-4, "{peeq}");
        let off = cpu
            .iter()
            .zip(&on_card)
            .step_by(14)
            .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs()));
        assert!(
            off <= 1e-3 * peeq,
            "equivalent plastic strain off by {off} of {peeq}"
        );
    }
    let largest = u.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
    assert!(largest > 1e-6, "{largest}");
    eprintln!("largest displacement {largest} m");
    let off = u
        .iter()
        .zip(&reached.u)
        .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs()));
    assert!(
        off <= 1e-5 * largest,
        "displacements off by {off} of {largest}"
    );
    let vmax = v.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
    let voff = v
        .iter()
        .zip(&reached.v)
        .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs()));
    assert!(voff <= 1e-4 * vmax, "velocities off by {voff} of {vmax}");
    for (name, cpu, gpu) in [
        ("internal", work[0], reached.internal_work),
        ("external", work[1], reached.external_work),
        ("viscous", work[2], reached.dissipation),
    ] {
        let scale = work.iter().fold(0.0_f64, |m, w| m.max(w.abs()));
        assert!(
            (cpu - gpu).abs() <= 1e-4 * scale,
            "{name} work: {cpu} on the processor, {gpu} on the card"
        );
    }
}
