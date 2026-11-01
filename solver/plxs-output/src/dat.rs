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

/// The blocks of `*CONTACT PRINT` (empty set): headings without a set, elements with their
/// face, the statistics in four labelled rows.
fn write_contact_block(out: &mut impl Write, block: &HistoryBlock) -> std::io::Result<()> {
    let time = fortran_time(block.time);
    writeln!(out)?;
    let q = &block.quantity;
    if q.starts_with("total number of contact elements") {
        writeln!(out, " {q} for time  {time}")?;
        writeln!(out)?;
        for (_, values) in &block.rows {
            writeln!(out, "{:>17}", values.first().copied().unwrap_or(0.0) as u64)?;
        }
        writeln!(out)?;
        return Ok(());
    }
    if q.starts_with("statistics for slave set") {
        writeln!(out, " {q} and time  {time}")?;
        let labels = [
            "total surface force (fx,fy,fz) and moment about the origin (mx,my,mz)",
            "center of gravity and mean normal",
            "moment about the center of gravity(mx,my,mz)",
            "area,  normal force (+ = tension) and shear force (size)",
        ];
        for (label, (_, values)) in labels.iter().zip(&block.rows) {
            writeln!(out)?;
            writeln!(out, "   {label}")?;
            writeln!(out)?;
            write!(out, "  ")?;
            for v in values {
                write!(out, " {}", fortran_e13(*v))?;
            }
            writeln!(out)?;
        }
        return Ok(());
    }
    writeln!(out, " {q} for all contact elements and time {time}")?;
    writeln!(out)?;
    let faces = q.contains("element+face");
    for (id, values) in &block.rows {
        let mut values = values.as_slice();
        write!(out, "{id:>10}")?;
        if faces {
            write!(out, "{:>11}", values.first().copied().unwrap_or(0.0) as u32)?;
            values = &values[1..];
        }
        for v in values {
            write!(out, " {}", fortran_e13(*v))?;
        }
        writeln!(out)?;
    }
    Ok(())
}

fn write_block(out: &mut impl Write, block: &HistoryBlock) -> std::io::Result<()> {
    if block.set.is_empty() && is_contact_quantity(&block.quantity) {
        return write_contact_block(out, block);
    }
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

fn is_contact_quantity(quantity: &str) -> bool {
    quantity.starts_with("total number of contact elements")
        || quantity.starts_with("statistics for slave set")
        || quantity.starts_with("relative contact displacement")
        || quantity.starts_with("contact stress")
        || quantity.starts_with("contact spring energy")
}
