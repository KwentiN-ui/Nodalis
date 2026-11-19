//! A bolt in a nut thread with play, pulled axially, plane strain, mm, as a screw in a
//! tapped block whose thread had 0.065 mm of play on each flank: not even the first
//! increment converged, its crests jumped by a millimetre in every iteration.
//!
//! - A nut point beside a corner of the bolt's profile (its foot point clamped to the
//!   corner) took a far flank of the next tooth it projects into from behind, a millimetre
//!   away, as overclosed by a millimetre. The closest face is the clamped one.
//! - Before the play closes, the bolt moves without strain: residuals of 1e-13 N, and the
//!   average force of such increments is no larger. Measured against it they never
//!   converged; they are round-off next to the forces the motion would cause.
//! - Nut points beyond the tip of the bolt, where its thread ends with no face across the
//!   tip, project into the last flanks from behind and counted as deeply overclosed.

use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::Command;

fn plxs() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_plxs"))
}

/// Runs `input` as job `job`; returns the `.sta` rows (step, increment, attempt,
/// iterations, total, step time, increment), the total reaction of the last `*NODE PRINT`
/// and the log.
fn run(job: &str, input: &str) -> (Vec<Vec<f64>>, Option<[f64; 2]>, String) {
    let dir = std::env::temp_dir().join(format!("plxs-thread-{job}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(format!("{job}.inp")), input).unwrap();
    let output = Command::new(plxs())
        .arg(job)
        .arg("--threads")
        .arg("2")
        .current_dir(&dir)
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&output.stdout).into_owned();
    let sta = std::fs::read_to_string(dir.join(format!("{job}.sta"))).unwrap_or_default();
    let rows: Vec<Vec<f64>> = (sta.lines())
        .map(|l| {
            l.split_whitespace()
                .filter_map(|t| t.parse::<f64>().ok())
                .collect::<Vec<_>>()
        })
        .filter(|r| r.len() == 7)
        .collect();
    let dat = std::fs::read_to_string(dir.join(format!("{job}.dat"))).unwrap_or_default();
    let total = (dat.lines().rev())
        .take_while(|l| !l.contains("total force"))
        .filter(|l| !l.trim().is_empty())
        .last()
        .map(|l| {
            let v: Vec<f64> = (l.split_whitespace())
                .filter_map(|t| t.parse().ok())
                .collect();
            [v[0], v[1]]
        });
    let _ = std::fs::remove_dir_all(&dir);
    (rows, total, log)
}

const PITCH: f64 = 1.75;
const DEPTH: f64 = 0.95;

/// The bolt profile (60° flanks, flat crest and root) at `x`, root at 0.
fn profile(x: f64) -> f64 {
    let flank = DEPTH * 30f64.to_radians().tan();
    let crest = PITCH / 8.0;
    let root = PITCH - 2.0 * flank - crest;
    let t = x.rem_euclid(PITCH);
    if t <= root / 2.0 {
        0.0
    } else if t <= root / 2.0 + flank {
        (t - root / 2.0) / flank * DEPTH
    } else if t <= root / 2.0 + flank + crest {
        DEPTH
    } else if t <= PITCH - root / 2.0 {
        DEPTH - (t - root / 2.0 - flank - crest) / flank * DEPTH
    } else {
        0.0
    }
}

/// The columns of a body along the thread: the corners of the profile, each span split
/// into `split`.
fn columns(teeth: usize, split: usize) -> Vec<f64> {
    let flank = DEPTH * 30f64.to_radians().tan();
    let crest = PITCH / 8.0;
    let root = PITCH - 2.0 * flank - crest;
    let mut corners = Vec::new();
    for i in 0..teeth {
        let x = i as f64 * PITCH;
        corners.extend([
            x,
            x + root / 2.0,
            x + root / 2.0 + flank,
            x + root / 2.0 + flank + crest,
            x + PITCH - root / 2.0,
        ]);
    }
    corners.push(teeth as f64 * PITCH);
    let mut xs = Vec::new();
    for w in corners.windows(2) {
        for k in 0..split {
            xs.push(w[0] + (w[1] - w[0]) * k as f64 / split as f64);
        }
    }
    xs.push(*corners.last().unwrap());
    xs
}

struct Mesh {
    nodes: Vec<[f64; 2]>,
    text: String,
    elements: usize,
}

impl Mesh {
    fn node(&mut self, x: [f64; 2]) -> usize {
        self.nodes.push(x);
        self.nodes.len()
    }

    /// A body over the columns `xs` from `lo(x)` to `hi(x)` in `rows` rows; CPE4, or CPE6
    /// (each cell split into two triangles) when `quadratic`. Returns the node ids by column
    /// and row and the faces on the bottom (`hi == false`) or top side.
    #[allow(clippy::too_many_arguments)]
    fn body(
        &mut self,
        name: &str,
        xs: &[f64],
        lo: impl Fn(f64) -> f64,
        hi: impl Fn(f64) -> f64,
        rows: usize,
        quadratic: bool,
        top: bool,
    ) -> (Vec<Vec<usize>>, Vec<String>) {
        let ids: Vec<Vec<usize>> = (xs.iter())
            .map(|&x| {
                (0..=rows)
                    .map(|j| self.node([x, lo(x) + (hi(x) - lo(x)) * j as f64 / rows as f64]))
                    .collect()
            })
            .collect();
        let kind = if quadratic { "CPE6" } else { "CPE4" };
        writeln!(self.text, "*ELEMENT, TYPE={kind}, ELSET={name}").unwrap();
        let mut faces = Vec::new();
        let mut mids = std::collections::HashMap::new();
        for i in 0..xs.len() - 1 {
            for j in 0..rows {
                let q = [ids[i][j], ids[i + 1][j], ids[i + 1][j + 1], ids[i][j + 1]];
                if quadratic {
                    for (t, tri) in [[q[0], q[1], q[2]], [q[0], q[2], q[3]]].iter().enumerate() {
                        let mut mid = |a: usize, b: usize| {
                            let key = (a.min(b), a.max(b));
                            *mids.entry(key).or_insert_with(|| {
                                let (p, r) = (self.nodes[a - 1], self.nodes[b - 1]);
                                self.nodes.push([(p[0] + r[0]) / 2.0, (p[1] + r[1]) / 2.0]);
                                self.nodes.len()
                            })
                        };
                        let m = [
                            mid(tri[0], tri[1]),
                            mid(tri[1], tri[2]),
                            mid(tri[2], tri[0]),
                        ];
                        self.elements += 1;
                        let e = self.elements;
                        writeln!(
                            self.text,
                            "{e}, {}, {}, {}, {}, {}, {}",
                            tri[0], tri[1], tri[2], m[0], m[1], m[2]
                        )
                        .unwrap();
                        // The bottom edge is S1 of the first triangle, the top edge S2 of
                        // the second.
                        if !top && j == 0 && t == 0 {
                            faces.push(format!("{e}, S1"));
                        }
                        if top && j == rows - 1 && t == 1 {
                            faces.push(format!("{e}, S2"));
                        }
                    }
                } else {
                    self.elements += 1;
                    let e = self.elements;
                    writeln!(self.text, "{e}, {}, {}, {}, {}", q[0], q[1], q[2], q[3]).unwrap();
                    if !top && j == 0 {
                        faces.push(format!("{e}, S1"));
                    }
                    if top && j == rows - 1 {
                        faces.push(format!("{e}, S3"));
                    }
                }
            }
        }
        (ids, faces)
    }

    fn input(&self, rest: &str) -> String {
        let mut s = String::from("*NODE\n");
        for (i, x) in self.nodes.iter().enumerate() {
            writeln!(s, "{}, {:.9}, {:.9}", i + 1, x[0], x[1]).unwrap();
        }
        s + &self.text + rest
    }
}

fn nset(name: &str, ids: &[usize]) -> String {
    let mut s = format!("*NSET, NSET={name}\n");
    for chunk in ids.chunks(12) {
        let line: Vec<String> = chunk.iter().map(|n| n.to_string()).collect();
        writeln!(s, "{}", line.join(", ")).unwrap();
    }
    s
}

/// Four bolt teeth in a nut with `play` on each flank (the nut's profile shifted out by
/// twice the play), the bolt's core pulled by `pull` along the thread in increments of
/// `increment`; CPE6 when `quadratic`.
fn thread(
    play: f64,
    pull: f64,
    increment: f64,
    split: usize,
    rows: usize,
    quadratic: bool,
) -> String {
    let mut m = Mesh {
        nodes: Vec::new(),
        text: String::new(),
        elements: 0,
    };
    let xs = columns(4, split);
    let shift = 2.0 * play;
    let top = DEPTH + shift + 3.0;
    let (bolt, master) = m.body("BOLT", &xs, |_| -3.0, profile, rows, quadratic, true);
    let (nut, slave) = m.body(
        "NUT",
        &xs,
        |x| profile(x) + shift,
        |_| top,
        rows,
        quadratic,
        false,
    );
    let pull_nodes: Vec<usize> = bolt.iter().map(|c| c[0]).collect();
    let fix_nodes: Vec<usize> = nut.iter().map(|c| c[rows]).collect();
    m.input(&format!(
        "*SURFACE, NAME=MASTER, TYPE=ELEMENT\n{}\n*SURFACE, NAME=SLAVE, TYPE=ELEMENT\n{}\n{}{}\
*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n\
*SOLID SECTION, ELSET=BOLT, MATERIAL=STEEL\n1.\n*SOLID SECTION, ELSET=NUT, MATERIAL=STEEL\n1.\n\
*SURFACE INTERACTION, NAME=I1\n*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=LINEAR\n2.1e6, 3\n\
*FRICTION\n0.1, 2.1e4\n\
*CONTACT PAIR, INTERACTION=I1, TYPE=SURFACE TO SURFACE\nSLAVE, MASTER\n\
*BOUNDARY\nFIX, 1, 2\nPULL, 2, 2\n\
*STEP, INC=100\n*STATIC\n{increment}, 1.0\n*BOUNDARY\nPULL, 1, 1, {}\n\
*NODE PRINT, NSET=PULL, TOTALS=ONLY\nRF\n*END STEP\n",
        master.join("\n"),
        slave.join("\n"),
        nset("PULL", &pull_nodes),
        nset("FIX", &fix_nodes),
        -pull
    ))
}

/// Axial play on 60° flanks for a normal play of 0.065 mm: 0.075 mm. Pulled by 0.1 mm
/// the bolt presses its flanks into the nut's by 0.025 mm.
#[test]
fn a_bolt_pulled_through_the_play_of_its_thread_converges() {
    let (rows, total, log) = run("pull", &thread(0.065, 0.1, 1.0, 1, 2, true));
    assert_eq!(rows.len(), 1, "one increment expected:\n{log}");
    assert_eq!(rows[0][2], 1.0, "the increment was cut back:\n{log}");
    assert!(rows[0][3] <= 10.0, "too many iterations: {rows:?}");
    // The flanks carry the pull: about 1.2 kN per mm of thickness.
    let fx = total.expect("no reaction")[0];
    assert!((-1500.0..-900.0).contains(&fx), "reaction {fx}");
}

/// Increments of 0.01 mm: the first seven move the bolt in its play without strain.
#[test]
fn increments_in_the_play_converge_at_once() {
    let (rows, total, log) = run("play", &thread(0.065, 0.1, 0.1, 2, 4, false));
    assert!(
        rows.last().is_some_and(|r| (r[5] - 1.0).abs() < 1e-9),
        "the step did not finish:\n{log}"
    );
    assert!(
        rows.iter().all(|r| r[2] == 1.0),
        "an increment was cut back: {rows:?}"
    );
    // Before the play closes: two iterations each.
    assert!(
        rows.iter().filter(|r| r[5] < 0.7).all(|r| r[3] <= 2.0),
        "increments in the play iterated: {rows:?}"
    );
    let fx = total.expect("no reaction")[0];
    assert!((-1500.0..-900.0).contains(&fx), "reaction {fx}");
}

/// A block whose master face is tilted by 30° and ends at a vertical cut that belongs to no
/// surface (the tip of a bolt), beside it a block whose slave face lies behind the tilted
/// face's plane: its points project into the face from behind, 0.2 mm deep, though nothing
/// touches. A small load on the first block: no contact force.
#[test]
fn a_slave_beyond_the_open_end_of_a_master_touches_nothing() {
    let mut m = Mesh {
        nodes: Vec::new(),
        text: String::new(),
        elements: 0,
    };
    let slope = 30f64.to_radians().tan();
    let bolt_x: Vec<f64> = (0..=4).map(|i| i as f64 / 4.0).collect();
    let nut_x: Vec<f64> = (0..=4).map(|i| 1.05 + 0.95 * i as f64 / 4.0).collect();
    let (bolt, master) = m.body("BOLT", &bolt_x, |_| -1.0, |x| x * slope, 4, false, true);
    let (nut, slave) = m.body("NUT", &nut_x, |_| 0.0, |_| 0.4, 2, false, true);
    let tip = bolt[4][4];
    let input = m.input(&format!(
        "*SURFACE, NAME=MASTER, TYPE=ELEMENT\n{}\n*SURFACE, NAME=SLAVE, TYPE=ELEMENT\n{}\n{}{}\
*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n\
*SOLID SECTION, ELSET=BOLT, MATERIAL=STEEL\n1.\n*SOLID SECTION, ELSET=NUT, MATERIAL=STEEL\n1.\n\
*SURFACE INTERACTION, NAME=I1\n*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=LINEAR\n2.1e6, 3\n\
*CONTACT PAIR, INTERACTION=I1, TYPE=SURFACE TO SURFACE\nSLAVE, MASTER\n\
*BOUNDARY\nBB, 1, 2\nNB, 1, 2\n*STEP\n*STATIC\n1, 1\n*CLOAD\n{tip}, 2, -1\n\
*NODE PRINT, NSET=NB, TOTALS=ONLY\nRF\n*END STEP\n",
        master.join("\n"),
        slave.join("\n"),
        nset("BB", &bolt.iter().map(|c| c[0]).collect::<Vec<_>>()),
        nset("NB", &nut.iter().map(|c| c[0]).collect::<Vec<_>>()),
    ));
    let (rows, total, log) = run("tip", &input);
    assert_eq!(rows.len(), 1, "one increment expected:\n{log}");
    let [fx, fy] = total.expect("no reaction");
    assert!(
        fx.abs() < 1e-6 && fy.abs() < 1e-6,
        "contact force {fx}, {fy}"
    );
}
