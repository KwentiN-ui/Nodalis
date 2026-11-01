//! The `.dat` writer: history blocks with CalculiX's headings (`plx-io::dat` recognises the
//! headings in `crates/plx-io/src/dat.rs`).

use std::io::{BufWriter, Write};
use std::path::Path;

use plxs_core::results::{HistoryBlock, Results};
use plxs_core::{Error, Result};

use crate::{fortran_e13, fortran_time};

/// Writes the history blocks of all steps.
pub fn write_dat(path: &Path, results: &Results) -> Result<()> {
    let io = |e: std::io::Error| Error::Io {
        path: path.display().to_string(),
        message: e.to_string(),
    };
    let file = std::fs::File::create(path).map_err(io)?;
    let mut out = BufWriter::new(file);
    write_dat_to(&mut out, results).map_err(io)?;
    out.flush().map_err(io)
}

pub fn write_dat_to(out: &mut impl Write, results: &Results) -> std::io::Result<()> {
    let mut last: Option<(u32, u32)> = None;
    for block in &results.history {
        if last.map(|(s, _)| s) != Some(block.step) {
            writeln!(out)?;
            writeln!(out, "                        S T E P {:>7}", block.step)?;
            writeln!(out)?;
        }
        if let Some(table) = EigenTable::of(block) {
            // The eigenvalue tables stand under the step heading without an increment.
            if last.map(|(s, _)| s) != Some(block.step) {
                writeln!(out)?;
            }
            last = Some((block.step, block.increment));
            write_eigen_table(out, table, block)?;
            continue;
        }
        if last != Some((block.step, block.increment)) {
            writeln!(out)?;
            writeln!(
                out,
                "                                INCREMENT {:>5}",
                block.increment
            )?;
            writeln!(out)?;
            last = Some((block.step, block.increment));
        }
        write_block(out, block)?;
    }
    Ok(())
}

fn write_block(out: &mut impl Write, block: &HistoryBlock) -> std::io::Result<()> {
    writeln!(out)?;
    writeln!(
        out,
        " {} for set {} and time  {}",
        block.quantity,
        block.set,
        fortran_time(block.time)
    )?;
    writeln!(out)?;
    let per_point = block.quantity.contains("integ.pnt.");
    let totals = block.quantity.starts_with("total ");
    for (id, values) in &block.rows {
        let mut values = values.as_slice();
        if totals {
            write!(out, "{:>6}", "")?;
        } else {
            write!(out, "{:>10}", id)?;
        }
        if per_point {
            write!(out, "{:>4}", values.first().copied().unwrap_or(0.0) as u32)?;
            values = &values[1..];
        }
        for v in values {
            write!(out, " {}", fortran_e13(*v))?;
        }
        writeln!(out)?;
    }
    Ok(())
}

/// The tables an eigenvalue step prints: [`HistoryBlock::quantity`] names them in
/// CalculiX's spelling without the spacing, the set is empty, rows are keyed by mode (0
/// for a total row). Written by `plxs-procedures::history`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EigenTable {
    /// Rows `(mode, [eigenvalue, omega, frequency, imaginary part])`.
    Eigenvalues,
    /// Rows `(mode, [x, y, z, rx, ry, rz])`.
    ParticipationFactors,
    /// Rows `(mode, [..6])` and a row `(0, [..6])` with the totals.
    EffectiveModalMass,
    /// One row `(0, [..6])`.
    TotalEffectiveMass,
    /// Rows `(mode, [factor])`.
    BucklingFactors,
    /// A complex frequency step: rows `(mode, [omega, frequency, imaginary part])`.
    ComplexEigenvalues,
    /// Rows `(mode, [+1 forward or −1 backward])` and a row `(0, [axis x, y, z])`.
    TurningDirection,
}

impl EigenTable {
    pub const EIGENVALUES: &'static str = "EIGENVALUE OUTPUT";
    pub const PARTICIPATION: &'static str = "PARTICIPATION FACTORS";
    pub const EFFECTIVE_MODAL_MASS: &'static str = "EFFECTIVE MODAL MASS";
    pub const TOTAL_EFFECTIVE_MASS: &'static str = "TOTAL EFFECTIVE MASS";
    pub const BUCKLING: &'static str = "BUCKLING FACTOR OUTPUT";
    pub const COMPLEX_EIGENVALUES: &'static str = "COMPLEX EIGENVALUE OUTPUT";
    pub const TURNING_DIRECTION: &'static str = "EIGENMODE TURNING DIRECTION";

    pub fn of(block: &HistoryBlock) -> Option<EigenTable> {
        if !block.set.is_empty() {
            return None;
        }
        Some(match block.quantity.as_str() {
            Self::EIGENVALUES => EigenTable::Eigenvalues,
            Self::PARTICIPATION => EigenTable::ParticipationFactors,
            Self::EFFECTIVE_MODAL_MASS => EigenTable::EffectiveModalMass,
            Self::TOTAL_EFFECTIVE_MASS => EigenTable::TotalEffectiveMass,
            Self::BUCKLING => EigenTable::BucklingFactors,
            Self::COMPLEX_EIGENVALUES => EigenTable::ComplexEigenvalues,
            Self::TURNING_DIRECTION => EigenTable::TurningDirection,
            _ => return None,
        })
    }

    fn heading(self) -> &'static str {
        match self {
            EigenTable::Eigenvalues => "E I G E N V A L U E   O U T P U T",
            EigenTable::ParticipationFactors => "P A R T I C I P A T I O N   F A C T O R S",
            EigenTable::EffectiveModalMass => "E F F E C T I V E   M O D A L   M A S S",
            EigenTable::TotalEffectiveMass => "T O T A L   E F F E C T I V E   M A S S",
            EigenTable::BucklingFactors => "B U C K L I N G   F A C T O R   O U T P U T",
            EigenTable::ComplexEigenvalues => "E I G E N V A L U E   O U T P U T",
            EigenTable::TurningDirection => "E I G E N M O D E   T U R N I N G   D I R E C T I O N",
        }
    }
}

const RIGID_COLUMNS: &str = "MODE NO.   X-COMPONENT     Y-COMPONENT     Z-COMPONENT     X-ROTATION      Y-ROTATION      Z-ROTATION";

fn write_eigen_table(
    out: &mut impl Write,
    table: EigenTable,
    block: &HistoryBlock,
) -> std::io::Result<()> {
    writeln!(out)?;
    writeln!(out, "     {}", table.heading())?;
    writeln!(out)?;
    match table {
        EigenTable::Eigenvalues => {
            writeln!(
                out,
                " MODE NO    EIGENVALUE                       FREQUENCY   "
            )?;
            writeln!(
                out,
                "                                     REAL PART            IMAGINARY PART"
            )?;
            writeln!(
                out,
                "                           (RAD/TIME)      (CYCLES/TIME     (RAD/TIME)"
            )?;
        }
        EigenTable::BucklingFactors => {
            writeln!(out, " MODE NO       BUCKLING")?;
            writeln!(out, "                FACTOR")?;
        }
        EigenTable::ComplexEigenvalues => {
            writeln!(
                out,
                " MODE NO                     FREQUENCY                   "
            )?;
            writeln!(
                out,
                "                      REAL PART         IMAGINARY PART"
            )?;
            writeln!(out, "             (RAD/TIME)   (CYCLES/TIME)   (RAD/TIME)")?;
        }
        EigenTable::TurningDirection => {
            let axis = block
                .rows
                .iter()
                .find(|r| r.0 == 0)
                .map(|r| r.1.clone())
                .unwrap_or_default();
            write!(out, "    Axis reference direction:")?;
            for v in &axis {
                write!(out, "{:>12}", fortran_e10(*v))?;
            }
            writeln!(out)?;
            writeln!(out)?;
            writeln!(out, " MODE NO     TURNING DIRECTION (F=FORWARD,B=BACKWARD)")?;
            writeln!(out)?;
            for (id, values) in block.rows.iter().filter(|r| r.0 != 0) {
                let direction = if values.first().copied().unwrap_or(1.0) >= 0.0 {
                    'F'
                } else {
                    'B'
                };
                writeln!(out, "{id:>7}          {direction}")?;
            }
            return Ok(());
        }
        _ => writeln!(out, "{RIGID_COLUMNS}")?,
    }
    writeln!(out)?;
    for (id, values) in &block.rows {
        if *id == 0 {
            if table == EigenTable::EffectiveModalMass {
                write!(out, "{:<7}", "TOTAL")?;
            } else {
                write!(out, "{:7}", "")?;
            }
        } else {
            write!(out, "{id:>7}")?;
        }
        for v in values {
            write!(out, "{:>16}", fortran_time(*v))?;
        }
        writeln!(out)?;
    }
    Ok(())
}

/// `0.1000E+01`: the short layout of the axis in the turning direction table.
fn fortran_e10(value: f64) -> String {
    if value == 0.0 || !value.is_finite() {
        return "0.0000E+00".to_string();
    }
    let text = format!("{:.3E}", value.abs());
    let (mantissa, exponent) = text.split_once('E').unwrap();
    let exponent: i32 = exponent.parse::<i32>().unwrap() + 1;
    let digits: String = mantissa.chars().filter(|c| c.is_ascii_digit()).collect();
    let sign = if value < 0.0 { "-" } else { "" };
    let exp_sign = if exponent < 0 { "-" } else { "+" };
    format!("{sign}0.{digits}E{exp_sign}{:02}", exponent.abs())
}
