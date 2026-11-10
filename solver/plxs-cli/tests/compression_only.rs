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

/// A ring of C3D20 elements with straight sides, a cylinder meshed with flat faces, resting
/// with its outer surface on a support and pulled sideways at its inner nodes, as a bolt
/// hole bears on a pin. The support pushes along the smoothed normal of the cylinder, so the
/// contact pressure falls off smoothly from the line of the load; along the kinked normals
/// of the faces the nodes on the kinks were held in two directions and the pressure
/// alternated between the corner and the midside nodes (31, 57, 41, 56 … at 0°, 7.5°, 15°,
/// 22.5°), striping it along the element edges.
#[test]
fn a_faceted_cylinder_bears_a_smooth_pressure() {
    const N: usize = 24;
    let (ri, ro, h) = (1.0_f64, 2.0_f64, 0.5_f64);
    let mut ids: std::collections::BTreeMap<[i64; 3], u32> = Default::default();
    let mut points: Vec<[f64; 3]> = Vec::new();
    let mut node = |p: [f64; 3]| -> u32 {
        let key = p.map(|c| (c * 1e6).round() as i64);
        *ids.entry(key).or_insert_with(|| {
            points.push(p);
            points.len() as u32
        })
    };
    let corner = |r: f64, k: usize, z: f64| {
        let t = std::f64::consts::TAU * k as f64 / N as f64;
        [r * t.cos(), r * t.sin(), z]
    };
    let mid = |a: [f64; 3], b: [f64; 3]| [0, 1, 2].map(|i| 0.5 * (a[i] + b[i]));
    let mut elements = String::new();
    for k in 0..N {
        let c = [
            corner(ri, k, 0.0),
            corner(ro, k, 0.0),
            corner(ro, k + 1, 0.0),
            corner(ri, k + 1, 0.0),
            corner(ri, k, h),
            corner(ro, k, h),
            corner(ro, k + 1, h),
            corner(ri, k + 1, h),
        ];
        let mut p: Vec<[f64; 3]> = c.to_vec();
        let edges = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
        ];
        p.extend(edges.iter().map(|&(a, b)| mid(c[a], c[b])));
        p.extend([(0, 4), (1, 5), (2, 6), (3, 7)].map(|(a, b)| mid(c[a], c[b])));
        let nodes: Vec<String> = p.into_iter().map(|q| node(q).to_string()).collect();
        elements += &format!("{}, {}\n", k + 1, nodes.join(", "));
    }
    let radius = |p: &[f64; 3]| (p[0] * p[0] + p[1] * p[1]).sqrt();
    let mut text = String::from("*NODE\n");
    for (i, p) in points.iter().enumerate() {
        text += &format!("{}, {}, {}, {}\n", i + 1, p[0], p[1], p[2]);
    }
    text += "*ELEMENT, TYPE=C3D20, ELSET=EALL\n";
    text += &elements;
    text += "*SURFACE, NAME=OUTER\n";
    for k in 1..=N {
        text += &format!("{k}, S4\n");
    }
    text += "*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n\
             *SOLID SECTION, ELSET=EALL, MATERIAL=STEEL\n\
             *COMPRESSION ONLY, SURFACE=OUTER\n*STEP\n*STATIC\n*BOUNDARY\n";
    for (i, p) in points.iter().enumerate() {
        if p[2].abs() < 1e-9 {
            text += &format!("{}, 3\n", i + 1);
        }
        if p[1].abs() < 1e-9 && p[0] < 0.0 {
            text += &format!("{}, 2\n", i + 1);
        }
    }
    let inner: Vec<usize> = (0..points.len())
        .filter(|&i| (radius(&points[i]) - ri).abs() < 1e-9)
        .collect();
    text += "*CLOAD\n";
    for i in &inner {
        text += &format!("{}, 1, {}\n", i + 1, 100.0 / inner.len() as f64);
    }
    text += "*CONTACT FILE\nCDIS, CSTR\n*END STEP\n";

    let dir = std::env::temp_dir().join(format!("plxs-faceted-ring-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("ring.inp"), text).unwrap();
    let status = Command::new(PathBuf::from(env!("CARGO_BIN_EXE_plxs")))
        .arg("ring")
        .current_dir(&dir)
        .output()
        .unwrap();
    assert!(status.status.success(), "{status:?}");
    let frd = std::fs::read_to_string(dir.join("ring.frd")).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    // CPRESS, the fourth value of the contact block, at the outer nodes of the top, corner
    // and midside, by their angle from the line of the load.
    let contact = frd.split("CONTACT").nth(1).expect("contact block");
    let mut profile = Vec::new();
    for line in contact.lines().take_while(|l| !l.starts_with(" -3")) {
        if !line.starts_with(" -1") {
            continue;
        }
        let node: usize = line[3..13].trim().parse().unwrap();
        let p = points[node - 1];
        if (p[2] - h).abs() > 1e-9 || (radius(&p) - ro).abs() > 0.05 || p[1] < -1e-9 {
            continue;
        }
        let angle = p[1].atan2(p[0]).to_degrees();
        if angle > 90.5 {
            continue;
        }
        let pressure: f64 = line[49..61].trim().parse().unwrap();
        profile.push((angle, pressure));
    }
    profile.sort_by(|a, b| a.0.total_cmp(&b.0));
    let shown = format!("{profile:?}");
    assert_eq!(profile.len(), N / 2 + 1, "{shown}");
    for pair in profile.windows(2).filter(|w| w[1].0 < 89.0) {
        assert!(pair[1].1 <= pair[0].1 + 0.1, "{shown}");
    }
    assert!(profile[0].1 > 40.0, "{shown}");
}
