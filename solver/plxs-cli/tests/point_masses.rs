//! Point masses (`*POINT MASS`): a mass on a node and a rotary inertia on the three DOFs of
//! another, on springs to ground. The eigenfrequencies are those of the springs on the mass
//! (on the principal moments of the inertia), and a sudden load swings the mass to twice its
//! static deflection after half a period, in implicit and in explicit time integration; a
//! gravity load pulls the mass with its weight.

use std::path::{Path, PathBuf};
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

const MASS: f64 = 2.0;
const K: f64 = 800.0;
/// The rotary inertia `[I11, I22, I33, I12, I13, I23]`, with products of inertia.
const INERTIA: [f64; 6] = [3.0, 4.0, 5.0, 0.5, -0.4, 0.3];
const KR: f64 = 600.0;

/// Node 1 with the mass on springs to ground in all three directions, node 2 with the
/// rotary inertia on rotational springs, then the step.
fn model(step: &str) -> String {
    let inertia: Vec<String> = INERTIA.iter().map(|v| v.to_string()).collect();
    format!(
        "*Node\n1, 0, 0, 0\n2, 0, 0, 0\n\
         *Element, Type=SPRING1, Elset=S1\n1, 1\n*Element, Type=SPRING1, Elset=S2\n2, 1\n\
         *Element, Type=SPRING1, Elset=S3\n3, 1\n\
         *Element, Type=SPRING1, Elset=R1\n4, 2\n*Element, Type=SPRING1, Elset=R2\n5, 2\n\
         *Element, Type=SPRING1, Elset=R3\n6, 2\n\
         *Spring, Elset=S1\n1\n{K}\n*Spring, Elset=S2\n2\n{K}\n*Spring, Elset=S3\n3\n{K}\n\
         *Spring, Elset=R1\n1\n{KR}\n*Spring, Elset=R2\n2\n{KR}\n*Spring, Elset=R3\n3\n{KR}\n\
         *Point mass, Node=1\n{MASS}\n\
         *Point mass, Node=2, Rotation node=2\n0, {}\n\
         *Step\n{step}\n*End step\n",
        inertia.join(", ")
    )
}

fn run(dir: &Path, job: &str, step: &str) -> String {
    std::fs::write(dir.join(format!("{job}.inp")), model(step)).unwrap();
    let output = Command::new(plxs())
        .arg(job)
        .arg("--threads")
        .arg("1")
        .current_dir(dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned()
        + &String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.success() && log.contains("Job finished"),
        "{log}"
    );
    std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap()
}

fn directory(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("plxs-point-mass-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn clean(dir: &Path) {
    if std::env::var_os("PLXS_KEEP").is_none() {
        let _ = std::fs::remove_dir_all(dir);
    }
}

/// The eigenvalues of a symmetric 3x3 matrix, by Jacobi rotations.
fn eigenvalues(mut a: [[f64; 3]; 3]) -> [f64; 3] {
    for _ in 0..50 {
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-15 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            let mut r = [[0.0; 3]; 3];
            for (i, row) in r.iter_mut().enumerate() {
                row[i] = 1.0;
            }
            r[p][p] = c;
            r[q][q] = c;
            r[p][q] = s;
            r[q][p] = -s;
            let mut b = [[0.0; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    b[i][j] = (0..3)
                        .flat_map(|k| (0..3).map(move |l| (k, l)))
                        .map(|(k, l)| r[k][i] * a[k][l] * r[l][j])
                        .sum();
                }
            }
            a = b;
        }
    }
    let mut e = [a[0][0], a[1][1], a[2][2]];
    e.sort_by(f64::total_cmp);
    e
}

#[test]
fn the_frequencies_are_those_of_the_springs_on_the_mass_and_the_inertia() {
    let dir = directory("frequency");
    let dat = run(&dir, "frequency", "*Frequency\n6");
    let mut found: Vec<f64> = dat
        .lines()
        .skip_while(|l| !l.contains("E I G E N V A L U E   O U T P U T"))
        .skip(1)
        .take_while(|l| !l.contains("P A R T I C I P A T I O N"))
        .filter_map(|l| {
            let v: Vec<f64> = (l.split_whitespace())
                .filter_map(|w| w.parse().ok())
                .collect();
            (v.len() == 5 && v[0].fract() == 0.0 && v[0] >= 1.0).then(|| v[1])
        })
        .collect();
    found.sort_by(f64::total_cmp);
    let [i11, i22, i33, i12, i13, i23] = INERTIA;
    let principal = eigenvalues([[i11, i12, i13], [i12, i22, i23], [i13, i23, i33]]);
    // Eigenvalues ω²: k / m three times, k_r / I for every principal moment.
    let mut expected: Vec<f64> = vec![K / MASS; 3];
    expected.extend(principal.iter().map(|p| KR / p));
    expected.sort_by(f64::total_cmp);
    assert_eq!(found.len(), 6, "{dat}");
    for (f, e) in found.iter().zip(&expected) {
        assert!((f - e).abs() < 1e-6 * e, "{found:?} vs {expected:?}");
    }
    clean(&dir);
}

/// The displacements of node `node` in the last print of the `.dat` file.
fn last_print(dat: &str, node: f64) -> [f64; 3] {
    let values: Vec<f64> = (dat.lines().rev())
        .find_map(|line| {
            let values: Vec<f64> = (line.split_whitespace())
                .filter_map(|v| v.parse().ok())
                .collect();
            (values.len() == 4 && values[0] == node).then_some(values)
        })
        .unwrap();
    [values[1], values[2], values[3]]
}

/// A sudden load and moment, held: after half the period of the mass the node has moved
/// twice the static deflection; the rotations follow the inertia with its products.
fn swing(integration: &str) -> ([f64; 3], [f64; 3]) {
    let dir = directory(integration);
    let half = std::f64::consts::PI * (MASS / K).sqrt();
    let step = format!(
        "*Dynamic, {integration}\n{}, {half}\n*Cload\n1, 1, 10\n2, 1, 30\n\
         *Node print, Nset=NALL\nU",
        half / 400.0
    );
    let text = model(&step)
        .replace(
            "*Point mass, Node=1",
            "*Nset, Nset=NALL\n1, 2\n*Point mass, Node=1",
        )
        .replace("*Step\n", "*Step, Amplitude=Step, Inc=1000\n");
    let job = format!(
        "swing-{}",
        integration.to_ascii_lowercase().replace('=', "-")
    );
    std::fs::write(dir.join(format!("{job}.inp")), text).unwrap();
    let output = Command::new(plxs())
        .arg(&job)
        .arg("--threads")
        .arg("1")
        .current_dir(&dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned()
        + &String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.success() && log.contains("Job finished"),
        "{log}"
    );
    let dat = std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap();
    let result = (last_print(&dat, 1.0), last_print(&dat, 2.0));
    clean(&dir);
    result
}

#[test]
fn a_sudden_load_swings_the_mass_to_twice_its_deflection() {
    let (implicit_u, implicit_r) = swing("Direct");
    let (explicit_u, explicit_r) = swing("Integration=Explicit");
    let twice = 2.0 * 10.0 / K;
    for u in [implicit_u, explicit_u] {
        assert!((u[0] - twice).abs() < 2e-3 * twice, "{u:?}");
        assert!(u[1].abs() < 1e-12 && u[2].abs() < 1e-12, "{u:?}");
    }
    // The products of inertia turn the node about all three axes, alike in both.
    assert!(implicit_r.iter().all(|r| r.abs() > 1e-4), "{implicit_r:?}");
    for (i, e) in implicit_r.iter().zip(explicit_r) {
        assert!(
            (i - e).abs() < 1e-2 * implicit_r[0].abs(),
            "{implicit_r:?} vs {explicit_r:?}"
        );
    }
}

#[test]
fn gravity_pulls_a_point_mass_with_its_weight() {
    let dir = directory("gravity");
    let step = "*Static\n*Dload\nS1, Grav, 9.81, 0, 0, -1\n*Node print, Nset=NALL\nU";
    let text = model(step).replace(
        "*Point mass, Node=1\n",
        "*Nset, Nset=NALL\n1, 2\n*Point mass, Node=1, Elset=S3\n",
    );
    std::fs::write(dir.join("gravity.inp"), text).unwrap();
    let output = Command::new(plxs())
        .arg("gravity")
        .current_dir(&dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned()
        + &String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "{log}");
    let dat = std::fs::read_to_string(dir.join("gravity.dat")).unwrap();
    // The load is on S1, which does not carry the mass (S3 does): nothing moves.
    assert_eq!(last_print(&dat, 1.0), [0.0; 3], "{dat}");
    let text = model(step).replace(
        "*Point mass, Node=1\n",
        "*Nset, Nset=NALL\n1, 2\n*Point mass, Node=1, Elset=S1\n",
    );
    std::fs::write(dir.join("gravity.inp"), text).unwrap();
    let output = Command::new(plxs())
        .arg("gravity")
        .current_dir(&dir)
        .output()
        .unwrap();
    assert!(output.status.success());
    let dat = std::fs::read_to_string(dir.join("gravity.dat")).unwrap();
    let u = last_print(&dat, 1.0);
    let expected = -MASS * 9.81 / K;
    assert!((u[2] - expected).abs() < 1e-9, "{u:?} vs {expected}");
    clean(&dir);
}
