//! `plxs -o bin`: the binary `.frd` holds the same mesh and results as the text one, read
//! with PrePoLix's own reader (`plx-io`), for solids, an expanded shell model and several
//! increments.

use std::path::{Path, PathBuf};
use std::process::Command;

use plxs_verify::fixtures_root;

fn run(dir: &Path, job: &str, binary: bool) {
    let mut command = Command::new(PathBuf::from(env!("CARGO_BIN_EXE_plxs")));
    if binary {
        command.args(["-o", "bin"]);
    }
    let output = command.arg(job).current_dir(dir).output().unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stdout)
    );
}

fn check(area: &str, name: &str) {
    let source = fixtures_root().join(area).join(format!("{name}.inp"));
    let dir = std::env::temp_dir().join(format!("plxs-binfrd-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::copy(&source, dir.join("asc.inp")).unwrap();
    std::fs::copy(&source, dir.join("bin.inp")).unwrap();
    run(&dir, "asc", false);
    run(&dir, "bin", true);
    let asc = plx_io::frd::read_frd(&dir.join("asc.frd")).unwrap();
    let bin = plx_io::frd::read_frd(&dir.join("bin.frd")).unwrap();
    let size = |p: &str| std::fs::metadata(dir.join(p)).unwrap().len();
    assert!(
        size("bin.frd") < size("asc.frd"),
        "{name}: binary is not smaller"
    );
    assert_eq!(asc.mesh.node_ids(), bin.mesh.node_ids(), "{name}");
    assert_eq!(asc.mesh.element_count(), bin.mesh.element_count(), "{name}");
    for (a, b) in asc.mesh.elements().iter().zip(bin.mesh.elements()) {
        assert_eq!((a.id, &a.nodes), (b.id, &b.nodes), "{name}");
    }
    for (a, b) in asc.mesh.coords().iter().zip(bin.mesh.coords()) {
        for k in 0..3 {
            // The text holds single precision with six digits, the binary double.
            assert!(
                (a[k] - b[k]).abs() <= 1e-5 * a[k].abs().max(1.0),
                "{name}: {a:?} {b:?}"
            );
        }
    }
    assert_eq!(asc.materials, bin.materials, "{name}");
    assert_eq!(asc.increments.len(), bin.increments.len(), "{name}");
    for (ia, ib) in asc.increments.iter().zip(&bin.increments) {
        assert_eq!(
            (ia.step, ia.increment, ia.kind),
            (ib.step, ib.increment, ib.kind)
        );
        assert_eq!(ia.fields.len(), ib.fields.len());
        for (fa, fb) in ia.fields.iter().zip(&ib.fields) {
            assert_eq!(fa.name, fb.name);
            assert_eq!(fa.components.len(), fb.components.len(), "{}", fa.name);
            for (ca, cb) in fa.components.iter().zip(&fb.components) {
                assert_eq!(ca.name, cb.name);
                // Derived components (principal values, Mises) are computed from the
                // rounded text in one file and from the full numbers in the other.
                if ca.derived {
                    continue;
                }
                let scale = (ca.values.iter())
                    .filter(|v| v.is_finite())
                    .fold(0.0f32, |m, v| m.max(v.abs()))
                    .max(1e-30);
                for (x, y) in ca.values.iter().zip(&cb.values) {
                    assert!(
                        x.is_nan() && y.is_nan() || (x - y).abs() <= 1e-5 * scale,
                        "{name} {} {}: {x} vs {y}",
                        fa.name,
                        ca.name
                    );
                }
            }
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn binary_and_text_frd_hold_the_same_results() {
    check("static", "kragbalken");
    check("nonlinear", "kriechen");
    check("loads", "moving");
    check("structural", "edge-tie-s4-knoten");
    check("structural", "balken_b32");
    check("eigen", "eigenfrequenz");
}
