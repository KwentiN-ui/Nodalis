//! A chain of masses and springs on the graphics card against the same central differences
//! in double precision on the processor. Runs where a card with double precision is found
//! (`PLXS_GPU=on` takes software renderers too); elsewhere it says so and passes.

use plxs_explicit::{Csr, Engine, MAX_CHUNK, Setup, Start};

/// `n` masses `m` in a row, springs `k` between them and to a wall at the first; the last
/// DOF is held. A dashpot `c` parallel to every spring.
fn chain(n: usize, k: f64, c: f64) -> (Csr, Csr) {
    let mut ks = Vec::new();
    let mut cs = Vec::new();
    for i in 0..n {
        for (list, s) in [(&mut ks, k), (&mut cs, c)] {
            list.push((i as u32, i as u32, s));
            if i + 1 < n {
                list.push((i as u32, i as u32, s));
                list.push((i as u32, i as u32 + 1, -s));
                list.push((i as u32 + 1, i as u32, -s));
                list.push((i as u32 + 1, i as u32 + 1, s));
            }
        }
    }
    (Csr::from_entries(n, ks), Csr::from_entries(n, cs))
}

#[test]
fn a_chain_of_springs_swings_as_on_the_processor() {
    if std::env::var("PLXS_GPU").is_ok_and(|v| v.eq_ignore_ascii_case("off")) {
        return;
    }
    let n = 1000;
    let (k, c) = chain(n, 4.0e6, 20.0);
    let mass: Vec<f64> = (0..n).map(|i| 1.0 + 0.001 * i as f64).collect();
    let mut inverse: Vec<f64> = mass.iter().map(|m| 1.0 / m).collect();
    inverse[n - 1] = 0.0;
    let alpha = 3.0;
    let setup = Setup {
        stiffness: &k,
        viscous: &c,
        inverse_mass: &inverse,
        alpha,
    };
    let mut engine = match Engine::new(&setup, true) {
        Ok(e) => e,
        Err(e) => {
            eprintln!("no graphics card for the test: {e}");
            return;
        }
    };
    // A pull at the first mass that grows linearly; a start velocity on the others.
    let load = |t: f64| {
        let mut p = vec![0.0; n];
        p[0] = 1.0e3 * t / 1e-3;
        p
    };
    let h = 0.4 * 2.0 / (2.0 * (4.0e6_f64 * 2.0).sqrt());
    let steps = 3 * MAX_CHUNK + 17;
    // Reference: central differences in double precision.
    let mut u = vec![0.0; n];
    let mut v: Vec<f64> = (0..n).map(|i| if i + 1 < n { 0.01 } else { 0.0 }).collect();
    let forces = |u: &[f64], v: &[f64]| {
        let fi = k.multiply(u);
        let mut fv = c.multiply(v);
        for i in 0..n {
            fv[i] += alpha * mass[i] * v[i];
        }
        (fi, fv)
    };
    let (fi0, fv0) = forces(&u, &v);
    let p0 = load(0.0);
    let accel = |p: &[f64], fi: &[f64], fv: &[f64]| -> Vec<f64> {
        (0..n)
            .map(|i| (p[i] - fi[i] - fv[i]) * inverse[i])
            .collect()
    };
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
    let off = u
        .iter()
        .zip(&reached.u)
        .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs()));
    assert!(
        off < 1e-5 * largest,
        "displacements off by {off} of {largest} on {}",
        engine.device()
    );
    let largest_v = v.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
    let off_v = v
        .iter()
        .zip(&reached.v)
        .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs()));
    assert!(
        off_v < 1e-4 * largest_v,
        "velocities off by {off_v} of {largest_v}"
    );
    for (got, want) in [
        reached.internal_work,
        reached.external_work,
        reached.dissipation,
    ]
    .iter()
    .zip(work)
    {
        assert!(
            (got - want).abs() < 1e-4 * want.abs().max(1e-12),
            "work {got} instead of {want}"
        );
    }
    assert_eq!(reached.u[n - 1], 0.0);
}
