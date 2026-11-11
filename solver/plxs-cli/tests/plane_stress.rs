//! Plane stress elements carry no stress across their thickness: `S33`, `S13` and `S23` are
//! zero at every integration point, for every plane stress type and for elastic, plastic and
//! hyperelastic (NLGEOM) materials, and the thickness strain `E33` is the one that makes
//! `S33` vanish. (The elements once let the layers of their 3D expansion move out of plane,
//! which made `S33` zero only on average: the integration points of a bent, distorted mesh
//! carried thickness stresses of up to 10 % of the in-plane ones.)

use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

/// Runs a job in a fresh directory and returns its `.dat` file.
fn run(name: &str, input: &str) -> String {
    let dir = std::env::temp_dir().join(format!("plxs-plane-stress-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(format!("{name}.inp")), input).unwrap();
    let output = Command::new(plxs())
        .arg(name)
        .arg("--threads")
        .arg("2")
        .current_dir(&dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned();
    assert!(output.status.success(), "{name} failed:\n{log}");
    let dat = std::fs::read_to_string(dir.join(format!("{name}.dat"))).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    dat
}

/// The rows (6 values after element and point) of the last `.dat` block whose header starts
/// with `quantity`.
fn last_block(dat: &str, quantity: &str) -> Vec<[f64; 6]> {
    let mut rows = Vec::new();
    let mut take = false;
    for line in dat.lines() {
        if line.trim_start().starts_with(quantity) {
            take = true;
            rows.clear();
            continue;
        }
        if !take {
            continue;
        }
        let values: Vec<&str> = line.split_whitespace().collect();
        if values.len() == 8 && values[0].parse::<u32>().is_ok() {
            rows.push(std::array::from_fn(|k| values[k + 2].parse().unwrap()));
        } else if !rows.is_empty() {
            take = false;
        }
    }
    rows
}

/// A cantilever of 10 × 4 elements of `kind` on a distorted mesh, clamped at x = 0, with a
/// tip load down and along: bending, shear and a varying in-plane trace.
fn cantilever(kind: &str) -> String {
    let (nx, ny) = (10, 4);
    let quadratic = matches!(kind, "CPS8" | "CPS8R" | "CPS6");
    let m = if quadratic { 2 } else { 1 };
    let (cols, rows) = (nx * m, ny * m);
    let id = |i: usize, j: usize| 1 + j * (cols + 1) + i;
    let mut s = String::from("*NODE\n");
    for j in 0..=rows {
        for i in 0..=cols {
            let (x, y) = (10.0 * i as f64 / cols as f64, 2.0 * j as f64 / rows as f64);
            let (x, y) = (
                x + 0.15 * (1.3 * y + 0.7 * x).sin(),
                y + 0.1 * (0.9 * x).sin(),
            );
            writeln!(s, "{}, {x}, {y}, 0", id(i, j)).unwrap();
        }
    }
    writeln!(s, "*ELEMENT, TYPE={kind}, ELSET=EALL").unwrap();
    let mut e = 1;
    for j in 0..ny {
        for i in 0..nx {
            let (a, b) = (i * m, j * m);
            let nodes: Vec<Vec<usize>> = match kind {
                "CPS4" | "CPS4R" => {
                    vec![vec![id(a, b), id(a + 1, b), id(a + 1, b + 1), id(a, b + 1)]]
                }
                "CPS8" | "CPS8R" => vec![vec![
                    id(a, b),
                    id(a + 2, b),
                    id(a + 2, b + 2),
                    id(a, b + 2),
                    id(a + 1, b),
                    id(a + 2, b + 1),
                    id(a + 1, b + 2),
                    id(a, b + 1),
                ]],
                "CPS3" => vec![
                    vec![id(a, b), id(a + 1, b), id(a + 1, b + 1)],
                    vec![id(a, b), id(a + 1, b + 1), id(a, b + 1)],
                ],
                "CPS6" => vec![
                    vec![
                        id(a, b),
                        id(a + 2, b),
                        id(a + 2, b + 2),
                        id(a + 1, b),
                        id(a + 2, b + 1),
                        id(a + 1, b + 1),
                    ],
                    vec![
                        id(a, b),
                        id(a + 2, b + 2),
                        id(a, b + 2),
                        id(a + 1, b + 1),
                        id(a + 1, b + 2),
                        id(a, b + 1),
                    ],
                ],
                _ => unreachable!(),
            };
            for n in nodes {
                let list: Vec<String> = n.iter().map(|v| v.to_string()).collect();
                writeln!(s, "{e}, {}", list.join(", ")).unwrap();
                e += 1;
            }
        }
    }
    let fixed: Vec<String> = (0..=rows).map(|j| id(0, j).to_string()).collect();
    let tip: Vec<String> = (0..=rows).map(|j| id(cols, j).to_string()).collect();
    write!(
        s,
        "*NSET, NSET=FIX\n{}\n*NSET, NSET=TIP\n{}\n\
*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n\
*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL\n1\n\
*STEP\n*STATIC\n*BOUNDARY\nFIX, 1, 2\n*CLOAD\nTIP, 2, -100\nTIP, 1, 300\n\
*EL PRINT, ELSET=EALL\nS\n*EL PRINT, ELSET=EALL\nE\n*END STEP\n",
        fixed.join(", "),
        tip.join(", ")
    )
    .unwrap();
    s
}

#[test]
fn every_plane_stress_type_has_no_stress_across_its_thickness() {
    let nu: f64 = 0.3;
    for kind in ["CPS3", "CPS4", "CPS4R", "CPS6", "CPS8", "CPS8R"] {
        let dat = run(&kind.to_lowercase(), &cantilever(kind));
        let stresses = last_block(&dat, "stresses");
        let strains = last_block(&dat, "strains");
        assert!(
            !stresses.is_empty() && stresses.len() == strains.len(),
            "{kind}"
        );
        let largest = stresses
            .iter()
            .flatten()
            .fold(0.0_f64, |m, v| m.max(v.abs()));
        assert!(largest > 100.0, "{kind}: {largest}");
        for (s, e) in stresses.iter().zip(&strains) {
            // Exactly zero, so that the results leave the components out.
            assert_eq!([s[2], s[4], s[5]], [0.0; 3], "{kind}: {s:?}");
            assert_eq!([e[4], e[5]], [0.0; 2], "{kind}: {e:?}");
            // Hooke under plane stress: ε33 = -ν / (1 - ν) (ε11 + ε22).
            let expected = -nu / (1.0 - nu) * (e[0] + e[1]);
            assert!(
                (e[2] - expected).abs() <= 1e-5 * (e[0].abs() + e[1].abs()) + 1e-12,
                "{kind}: {e:?}"
            );
        }
    }
}

/// One CPS4 element over the unit square stretched along x by a prescribed displacement,
/// on symmetry supports.
fn strip(material: &str, step: &str) -> String {
    format!(
        "*NODE\n1, 0, 0, 0\n2, 1, 0, 0\n3, 1, 1, 0\n4, 0, 1, 0\n\
*ELEMENT, TYPE=CPS4, ELSET=EALL\n1, 1, 2, 3, 4\n\
*NSET, NSET=X0\n1, 4\n*NSET, NSET=X1\n2, 3\n*NSET, NSET=Y0\n1, 2\n\
*MATERIAL, NAME=M\n{material}*SOLID SECTION, ELSET=EALL, MATERIAL=M\n1\n{step}"
    )
}

/// The total x reaction of the last `TOTALS=ONLY` block.
fn total_force(dat: &str) -> f64 {
    let mut take = false;
    let mut force = f64::NAN;
    for line in dat.lines() {
        if line.contains("total force") {
            take = true;
        } else if take && let Some(v) = line.split_whitespace().next().and_then(|v| v.parse().ok())
        {
            force = v;
            take = false;
        }
    }
    force
}

#[test]
fn a_plane_stress_rubber_strip_stretched_to_twice_its_length_follows_neo_hooke() {
    let (c10, d1, stretch): (f64, f64, f64) = (1.0, 0.1, 2.0);
    let step = "*STEP, NLGEOM, INC=200\n*STATIC\n0.1, 1\n*BOUNDARY\nX0, 1\nY0, 2\nX1, 1, 1, 1.0\n\
*NODE PRINT, NSET=X0, TOTALS=ONLY\nRF\n*EL PRINT, ELSET=EALL\nS\n*EL PRINT, ELSET=EALL\nE\n*END STEP\n";
    let dat = run(
        "rubber",
        &strip(&format!("*HYPERELASTIC, NEO HOOKE\n{c10}, {d1}\n"), step),
    );
    // Uniaxial stress: F = diag(λ, t, t) with σ22 = σ33 = 0, as for the 3D cube of
    // `hyperelastic.rs`; the plane stress strip has the same lateral stretch in its plane and
    // across its thickness.
    let deviator = |t: f64| {
        let j = stretch * t * t;
        2.0 * c10 * j.powf(-5.0 / 3.0) * (stretch * stretch - t * t)
    };
    let residual = |t: f64| deviator(t) / 3.0 - 2.0 * (stretch * t * t - 1.0) / d1;
    let (mut lo, mut hi) = (0.1, 3.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if residual(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let t = 0.5 * (lo + hi);
    let force = -total_force(&dat);
    let expected = deviator(t) * t * t;
    assert!(
        (force - expected).abs() < 1e-3 * expected,
        "force {force}, expected {expected}"
    );
    for s in last_block(&dat, "stresses") {
        assert_eq!([s[2], s[4], s[5]], [0.0; 3], "{s:?}");
        // The Cauchy stress on the deformed cross section t × t (thickness included).
        assert!(
            (s[0] - deviator(t)).abs() < 1e-3 * deviator(t),
            "{s:?} vs {}",
            deviator(t)
        );
        assert!(s[1].abs() < 1e-6 * s[0], "{s:?}");
    }
    // The thickness strain (Green-Lagrange) is that of the lateral stretch.
    let lateral = 0.5 * (t * t - 1.0);
    for e in last_block(&dat, "strains") {
        assert!(
            (e[2] - lateral).abs() < 1e-3 * lateral.abs(),
            "{e:?} vs {lateral}"
        );
        assert!((e[1] - e[2]).abs() < 1e-3 * lateral.abs(), "{e:?}");
    }
}

#[test]
fn a_plane_stress_steel_strip_yields_at_its_yield_stress() {
    let material = "*ELASTIC\n200000, 0.3\n*PLASTIC\n200, 0\n";
    let step = "*STEP, INC=100\n*STATIC\n0.1, 1\n*BOUNDARY\nX0, 1\nY0, 2\nX1, 1, 1, 0.005\n\
*NODE PRINT, NSET=X0, TOTALS=ONLY\nRF\n*EL PRINT, ELSET=EALL\nS\n*EL PRINT, ELSET=EALL\nE\n*END STEP\n";
    let dat = run("steel", &strip(material, step));
    let force = -total_force(&dat);
    assert!((force - 200.0).abs() < 1e-3 * 200.0, "force {force}");
    for s in last_block(&dat, "stresses") {
        assert_eq!([s[2], s[4], s[5]], [0.0; 3], "{s:?}");
        assert!((s[0] - 200.0).abs() < 1e-3 * 200.0, "{s:?}");
    }
    // Plastic flow keeps the volume: at 5 times the yield strain the lateral strains are
    // close to -ε11 / 2, equal in the plane and across the thickness.
    for e in last_block(&dat, "strains") {
        assert!(e[2] < -0.4 * e[0] && e[2] > -0.5 * e[0], "{e:?}");
        assert!((e[1] - e[2]).abs() < 1e-6 * e[0], "{e:?}");
    }
}
