//! A chain of masses and springs on the graphics card against the same central differences
//! in double precision on the processor, with every mass free or with constraints and a
//! coupled mass. Runs where a card with double precision is found (`PLXS_GPU=on` takes
//! software renderers too); elsewhere it says so and passes.

use plxs_explicit::{Block, Elements, Engine, MAX_CHUNK, Reduction, Setup, Sparse, Start};

/// `n` masses in a row, springs `k` between them and to a wall at the first, as constant
/// blocks.
fn chain(n: usize, k: f64) -> Elements {
    let mut elements = Elements::default();
    elements
        .push_block(Block {
            dofs: vec![0],
            stiffness: vec![k],
        })
        .unwrap();
    for i in 0..n - 1 {
        elements
            .push_block(Block {
                dofs: vec![i as u32, i as u32 + 1],
                stiffness: vec![k, -k, -k, k],
            })
            .unwrap();
    }
    elements
}

#[test]
fn a_chain_of_springs_swings_as_on_the_processor() {
    swing(false);
}

#[test]
fn a_chain_with_constraints_and_coupled_masses_swings_as_on_the_processor() {
    swing(true);
}

/// The last DOF is held (at a prescribed displacement when `constrained`). With
/// `constrained` every tenth DOF follows the mean of its neighbours (an equation), and
/// pairs of DOFs share a coupled mass (a fibre's `α(diag(m) − m mᵀ/Σm)`).
fn swing(constrained: bool) {
    if std::env::var("PLXS_GPU").is_ok_and(|v| v.eq_ignore_ascii_case("off")) {
        return;
    }
    let n = 1000;
    let elements = chain(n, 4.0e6);
    let mass: Vec<f64> = (0..n).map(|i| 1.0 + 0.001 * i as f64).collect();
    // The transformation: a column per free DOF; dependent DOFs follow their neighbours.
    let dependent = |i: usize| constrained && i % 10 == 5;
    let held = n - 1;
    let mut column = vec![u32::MAX; n];
    let mut r = 0;
    for (i, c) in column.iter_mut().enumerate() {
        if i != held && !dependent(i) {
            *c = r;
            r += 1;
        }
    }
    let mut t = Vec::new();
    for i in 0..n {
        if i == held {
            continue;
        }
        if dependent(i) {
            t.push((i as u32, column[i - 1], 0.5));
            t.push((i as u32, column[i + 1], 0.5));
        } else {
            t.push((i as u32, column[i], 1.0));
        }
    }
    let transform = Sparse::from_entries(n, t);
    // The mass `Tᵀ M T` plus coupled pairs of reduced unknowns.
    let mut entries = Vec::new();
    for (i, m) in mass.iter().enumerate() {
        for (a, ta) in transform.row(i) {
            for (b, tb) in transform.row(i) {
                entries.push((a, b, ta * m * tb));
            }
        }
    }
    let mut extra: Vec<(u32, u32, f64)> = Vec::new();
    if constrained {
        for pair in (0..r - 1).step_by(7) {
            let (m1, m2, alpha) = (1.0, 1.2, 0.4);
            let s = m1 + m2;
            let block = [
                alpha * (m1 - m1 * m1 / s),
                -alpha * m1 * m2 / s,
                -alpha * m1 * m2 / s,
                alpha * (m2 - m2 * m2 / s),
            ];
            for (k, (a, b)) in [(0, 0), (0, 1), (1, 0), (1, 1)].iter().enumerate() {
                extra.push((pair + a, pair + b, block[k]));
            }
        }
    }
    entries.extend(extra.iter().copied());
    let reduced_mass = Sparse::from_entries(r as usize, entries);
    let (reduction, clusters) = Reduction::new(transform.clone(), &reduced_mass).unwrap();
    if constrained {
        assert!((2..=6).contains(&clusters.largest), "{clusters:?}");
    } else {
        assert_eq!(clusters.largest, 1);
    }
    let extra = Sparse::from_entries(r as usize, extra);
    let tt = transform.transpose(r as usize);
    let prescribed: Vec<f64> = (0..n)
        .map(|i| if i == held && constrained { 1e-4 } else { 0.0 })
        .collect();
    let alpha = 3.0;
    let setup = Setup {
        elements: &elements,
        mass: &mass,
        reduction: &reduction,
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
    let h = 0.25 * 2.0 / (2.0 * (4.0e6_f64 * 2.0).sqrt());
    let steps = 3 * MAX_CHUNK + 17;
    // Reference: central differences in double precision.
    let expand = |q: &[f64], g: &[f64]| -> Vec<f64> {
        let tq = transform.multiply(q);
        tq.iter().zip(g).map(|(a, b)| a + b).collect()
    };
    let zero = vec![0.0; n];
    let mut q: Vec<f64> = vec![0.0; r as usize];
    let mut qdot: Vec<f64> = (0..r as usize).map(|j| 0.01 + 1e-5 * j as f64).collect();
    let mut u = expand(&q, &prescribed);
    let mut v = expand(&qdot, &zero);
    let forces = |u: &[f64], v: &[f64]| {
        let (fi, mut fv) = elements.forces(u, v, &mut []);
        for i in 0..n {
            fv[i] += alpha * mass[i] * v[i];
        }
        (fi, fv)
    };
    let accel = |p: &[f64], fi: &[f64], fv: &[f64]| -> Vec<f64> {
        let residual: Vec<f64> = (0..n).map(|i| p[i] - fi[i] - fv[i]).collect();
        reduction.inverse_mass.multiply(&tt.multiply(&residual))
    };
    let (fi0, fv0) = forces(&u, &v);
    let p0 = load(0.0);
    let a0 = accel(&p0, &fi0, &fv0);
    engine
        .load(&Start {
            q: &q,
            qdot: &qdot,
            a: &a0,
            prescribed: &prescribed,
            u: &u,
            v: &v,
            internal: &fi0,
            viscous: &fv0,
            loads: &p0,
            material: &[],
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
            for (qd, a) in qdot.iter_mut().zip(&a) {
                *qd += cf * a;
            }
            for (qi, qd) in q.iter_mut().zip(&qdot) {
                *qi += h * qd;
            }
            let u_new = expand(&q, &prescribed);
            let du: Vec<f64> = u_new.iter().zip(&u).map(|(a, b)| a - b).collect();
            u = u_new;
            v = expand(&qdot, &zero);
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
    let off = |a: &[f64], b: &[f64]| {
        let largest = a.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
        let off = a
            .iter()
            .zip(b)
            .fold(0.0_f64, |m, (x, y)| m.max((x - y).abs()));
        (off, largest)
    };
    for (name, a, b, tolerance) in [
        ("displacements", &u, &reached.u, 1e-5),
        ("reduced displacements", &q, &reached.q, 1e-5),
        ("velocities", &v, &reached.v, 1e-4),
        ("reduced velocities", &qdot, &reached.qdot, 1e-4),
    ] {
        let (off, largest) = off(a, b);
        assert!(
            off < tolerance * largest,
            "{name} off by {off} of {largest} on {}",
            engine.device()
        );
    }
    assert_eq!(reached.a.len(), r as usize);
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
    assert_eq!(reached.u[held], prescribed[held]);
    // The coupled masses act: the kinetic energy of the reduced velocities differs.
    if constrained {
        assert!(extra.multiply(&reached.qdot).iter().any(|x| *x != 0.0));
    }
}
