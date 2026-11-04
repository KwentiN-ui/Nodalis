//! Snapshots: running plxs on a fixture and freezing what it writes as the expected
//! results, which later runs must reproduce within the fixture's class tolerance.

use std::path::Path;
use std::process::Command;

use crate::Fixture;
use crate::corpus::gzip;

/// Runs `solver` (plxs) on the fixture in `work` (a fresh directory) and writes its `.frd`
/// and `.dat` beside the fixture as its snapshot. Returns the name of the solver that made
/// it, or the reason it failed.
pub fn make_snapshot(solver: &Path, fixture: &Fixture, work: &Path) -> Result<String, String> {
    std::fs::create_dir_all(work).map_err(|e| e.to_string())?;
    let name = &fixture.meta.name;
    let text = std::fs::read_to_string(fixture.inp()).map_err(|e| e.to_string())?;
    std::fs::write(work.join(format!("{name}.inp")), text).map_err(|e| e.to_string())?;
    crate::corpus::stage_files(fixture, work)?;
    let output = Command::new(solver)
        .args(["-i", name])
        .current_dir(work)
        .env("OMP_NUM_THREADS", "1")
        .output()
        .map_err(|e| format!("{}: {e}", solver.display()))?;
    let log = String::from_utf8_lossy(&output.stdout);
    if !output.status.success() || log.contains("*ERROR") {
        let error = (log.lines())
            .skip_while(|l| !l.contains("*ERROR"))
            .take(4)
            .collect::<Vec<_>>()
            .join(" ");
        return Err(if error.is_empty() {
            format!("exit {}", output.status)
        } else {
            error
        });
    }
    let frd =
        std::fs::read(work.join(format!("{name}.frd"))).map_err(|e| format!("no .frd: {e}"))?;
    std::fs::write(fixture.ref_frd(), gzip(&frd).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    let dat = std::fs::read_to_string(work.join(format!("{name}.dat"))).unwrap_or_default();
    if dat.trim().is_empty() {
        let _ = std::fs::remove_file(fixture.ref_dat());
    } else {
        std::fs::write(fixture.ref_dat(), dat).map_err(|e| e.to_string())?;
    }
    Ok(format!("plxs {}", env!("CARGO_PKG_VERSION")))
}
