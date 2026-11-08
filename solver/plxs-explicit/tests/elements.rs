//! A block of bricks on the graphics card with the element kernels against the same central
//! differences with the element forces of [`Elements::forces`] in double precision on the
//! processor. Runs where a card with double precision is found (`PLXS_GPU=on` takes
//! software renderers too); elsewhere it says so and passes.

use plxs_explicit::{Element, Elements, Engine, ForceModel, Hourglass, MAX_CHUNK, Setup, Start};

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
    let mut elements = Elements::default();
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
                elements
                    .push(Element {
                        dofs: ids
                            .iter()
                            .flat_map(|&id| (0..3).map(move |d| (3 * id + d) as u32))
                            .collect(),
                        points,
                        elastic: isotropic(young, 0.3),
                        bulk: 0.06 * young * dt_e * volume,
                        mean: mean.iter().flatten().copied().collect(),
                        hourglass: reduced.then(|| Hourglass {
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
    let setup = Setup {
        forces: ForceModel::Elements(&elements),
        inverse_mass: &inverse,
        alpha,
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
                0.3
            } else {
                0.0
            }
        })
        .collect();
    let forces = |u: &[f64], v: &[f64]| {
        let (fi, mut fv) = elements.forces(u, v);
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
    let p0 = load(0.0);
    let a0 = accel(&p0, &fi0, &fv0);
    engine
        .load(&Start {
            u: &u,
            v: &v,
            a: &a0,
            internal: &fi0,
            viscous: &fv0,
            loads: &p0,
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
        engine.run(&list, &from, &to).unwrap();
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
    let largest = u.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
    assert!(largest > 1e-6, "{largest}");
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
