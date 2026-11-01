//! Golden files: running CalculiX on a fixture and storing what it writes.

use std::path::Path;
use std::process::Command;

use crate::Fixture;
use crate::corpus::gzip;

/// The CalculiX version `ccx -v` reports, if it runs.
pub fn ccx_version(ccx: &str) -> Option<String> {
    let output = Command::new(ccx).arg("-v").output().ok()?;
    let text = String::from_utf8_lossy(&output.stdout);
    text.lines()
        .find_map(|l| l.trim().strip_prefix("This is Version "))
        .map(|v| format!("ccx {}", v.trim()))
}

/// Runs CalculiX on the fixture in `work` (a fresh directory) and writes the golden files
/// beside the fixture. Returns the version string, or the reason it failed.
pub fn make_reference(ccx: &str, fixture: &Fixture, work: &Path) -> Result<String, String> {
    let version = ccx_version(ccx).ok_or_else(|| format!("{ccx} does not run"))?;
    std::fs::create_dir_all(work).map_err(|e| e.to_string())?;
    let name = &fixture.meta.name;
    let text = std::fs::read_to_string(fixture.inp()).map_err(|e| e.to_string())?;
    std::fs::write(work.join(format!("{name}.inp")), text).map_err(|e| e.to_string())?;
    crate::corpus::stage_files(fixture, work)?;
    let output = Command::new(ccx)
        .args(["-i", name])
        .current_dir(work)
        .env("OMP_NUM_THREADS", "1")
        .env("CCX_NPROC_EQUATION_SOLVER", "1")
        .output()
        .map_err(|e| e.to_string())?;
    let log = String::from_utf8_lossy(&output.stdout);
    if !output.status.success() || log.contains("*ERROR") {
        let error = log
            .lines()
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
    Ok(version)
}
