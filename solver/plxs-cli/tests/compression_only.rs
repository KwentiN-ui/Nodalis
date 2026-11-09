//! `*COMPRESSION ONLY` on a quadratic face: a C3D20 unit cube resting on its bottom face,
//! pressed by 100 on its top face. The support acts at the integration points of the face,
//! so the pressure under the cube is uniform and so is the stress at every bottom node,
//! corner and midside alike. Rigid gaps at the nodes (`GAPUNI`) left the corners without
//! load, which checkered the stresses of a pin-loaded lug.

use std::path::PathBuf;
use std::process::Command;

const CUBE: &str = "\
*NODE
1, 0, 0, 0
2, 1, 0, 0
3, 1, 1, 0
4, 0, 1, 0
5, 0, 0, 1
6, 1, 0, 1
7, 1, 1, 1
8, 0, 1, 1
9, 0.5, 0, 0
10, 1, 0.5, 0
11, 0.5, 1, 0
12, 0, 0.5, 0
13, 0.5, 0, 1
14, 1, 0.5, 1
15, 0.5, 1, 1
16, 0, 0.5, 1
17, 0, 0, 0.5
18, 1, 0, 0.5
19, 1, 1, 0.5
20, 0, 1, 0.5
*ELEMENT, TYPE=C3D20, ELSET=EALL
1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20
*SURFACE, NAME=BOTTOM
1, S1
*MATERIAL, NAME=STEEL
*ELASTIC
210000, 0.3
*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL
*COMPRESSION ONLY, SURFACE=BOTTOM
*STEP
*STATIC
*BOUNDARY
1, 1, 2
2, 2
*DLOAD
1, P2, 100
*EL FILE
S
*END STEP
";

#[test]
fn a_pressed_quadratic_face_carries_the_pressure_uniformly() {
    let dir = std::env::temp_dir().join(format!("plxs-compression-only-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("cube.inp"), CUBE).unwrap();
    let status = Command::new(PathBuf::from(env!("CARGO_BIN_EXE_plxs")))
        .arg("cube")
        .current_dir(&dir)
        .output()
        .unwrap();
    assert!(status.status.success(), "{status:?}");
    let frd = std::fs::read_to_string(dir.join("cube.frd")).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    // S33 of the nodes in the stress block (` -1` lines: node id, then 12-wide values).
    let stress = frd.split("STRESS").nth(1).expect("stress block");
    let mut bottom = 0;
    for line in stress.lines().take_while(|l| !l.starts_with(" -3")) {
        if !line.starts_with(" -1") {
            continue;
        }
        let node: u32 = line[3..13].trim().parse().unwrap();
        let s33: f64 = line[37..49].trim().parse().unwrap();
        assert!((s33 + 100.0).abs() < 1.0, "node {node}: S33 {s33}");
        if [1, 2, 3, 4, 9, 10, 11, 12].contains(&node) {
            bottom += 1;
        }
    }
    assert_eq!(bottom, 8);
}
