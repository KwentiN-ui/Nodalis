//! The `.dat` writer, for the export: history blocks with CalculiX's headings (`plx-io::dat`
//! reads such files).

use std::io::{BufWriter, Write};
use std::path::Path;

use plxs_core::mesh::Sets;
use plxs_core::results::{HistoryBlock, HistoryQuantity, HistorySource, Results};
use plxs_core::{Error, Result};

use crate::{fortran_e13, fortran_time};

/// Writes the history blocks of all steps; `sets` names the sets they are of.
pub fn write_dat(path: &Path, results: &Results, sets: &Sets) -> Result<()> {
    let io = |e: std::io::Error| Error::Io {
        path: path.display().to_string(),
        message: e.to_string(),
    };
    let file = std::fs::File::create(path).map_err(io)?;
    let mut out = BufWriter::new(file);
    write_dat_to(&mut out, results, sets).map_err(io)?;
    out.flush().map_err(io)
}

pub fn write_dat_to(out: &mut impl Write, results: &Results, sets: &Sets) -> std::io::Result<()> {
    let mut last: Option<(u32, u32)> = None;
    for block in &results.history {
        if last.map(|(s, _)| s) != Some(block.step) {
            writeln!(out)?;
            writeln!(out, "                        S T E P {:>7}", block.step)?;
            writeln!(out)?;
        }
        if block.quantity == HistoryQuantity::DesignResponses {
            if last.map(|(s, _)| s) != Some(block.step) {
                writeln!(out)?;
            }
            last = Some((block.step, block.increment));
            write_design_response_table(out, block)?;
            continue;
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
        write_block(out, block, sets)?;
    }
    Ok(())
}

/// The heading of a block of increment values as CalculiX writes it, before `for set`.
fn heading(quantity: HistoryQuantity) -> String {
    use HistoryQuantity as Q;
    let slave = |faces: bool| {
        if faces {
            "slave element+face"
        } else {
            "slave node"
        }
    };
    match quantity {
        Q::Displacements | Q::Rotations => "displacements (vx,vy,vz)".into(),
        Q::Velocities | Q::AngularVelocities => "velocities (vx,vy,vz)".into(),
        Q::Forces | Q::Moments => "forces (fx,fy,fz)".into(),
        Q::TotalForce | Q::TotalMoment => "total force (fx,fy,fz)".into(),
        Q::Temperatures => "temperatures".into(),
        Q::HeatGeneration => "heat generation".into(),
        Q::TotalHeatGeneration => "total heat generation".into(),
        Q::HeatFlux => "heat flux (elem, integ.pnt.,qx,qy,qz)".into(),
        Q::Stresses => "stresses (elem, integ.pnt.,sxx,syy,szz,sxy,sxz,syz)".into(),
        Q::Strains => "strains (elem, integ.pnt.,exx,eyy,ezz,exy,exz,eyz)".into(),
        Q::EquivalentPlasticStrain => "equivalent plastic strain (elem, integ.pnt.,pe)".into(),
        Q::InternalEnergyDensity => "internal energy density (elem, integ.pnt.,energy)".into(),
        Q::Volume => "volume (element, volume)".into(),
        Q::TotalVolume => "total volume".into(),
        Q::ModelInternalEnergy => "total model internal energy".into(),
        Q::ModelKineticEnergy => "total model kinetic energy".into(),
        Q::ModelExternalWork => "total model external work".into(),
        Q::ModelDampingDissipation => "total model damping dissipation".into(),
        Q::ParticleTemperature => "particle temperature".into(),
        Q::WallHeat => "total wall heat".into(),
        Q::ContactElements => "total number of contact elements".into(),
        Q::RelativeContactDisplacement { faces } => {
            format!(
                "relative contact displacement ({},normal,tang1,tang2)",
                slave(faces)
            )
        }
        Q::ContactStress { faces } => {
            format!("contact stress ({},press,tang1,tang2)", slave(faces))
        }
        Q::ContactSpringEnergy { faces } => {
            format!("contact spring energy ({},energy)", slave(faces))
        }
        // Tables and statistics have headings of their own.
        _ => String::new(),
    }
}

/// The name of the set of a block, as the file prints it.
pub fn set_name<'a>(source: &'a HistorySource, sets: &'a Sets) -> &'a str {
    match source {
        HistorySource::Nodes(set) => sets.node_set_name(*set),
        HistorySource::Elements(set) => sets.element_set_name(*set),
        HistorySource::Contact { slave, .. } => sets.surface_name(*slave),
        HistorySource::Named(name) => name,
        HistorySource::Step => "",
    }
}

/// The set name a block is printed for: the rotations and moments of a group without a
/// set (the walls of a particle flow) under the name of a rotation node set, as the
/// rotation nodes of reference points are named.
fn printed_set(block: &HistoryBlock, sets: &Sets) -> String {
    use HistoryQuantity as Q;
    let name = set_name(&block.source, sets);
    let rotational = matches!(
        block.quantity,
        Q::Rotations | Q::AngularVelocities | Q::Moments | Q::TotalMoment
    );
    match &block.source {
        HistorySource::Named(_) if rotational => format!("INTERNAL_ROTATION-1_{name}"),
        _ => name.to_string(),
    }
}

/// The blocks of `*CONTACT PRINT` (empty set): headings without a set, elements with their
/// face, the statistics in four labelled rows.
fn write_contact_block(
    out: &mut impl Write,
    block: &HistoryBlock,
    sets: &Sets,
) -> std::io::Result<()> {
    let time = fortran_time(block.time);
    writeln!(out)?;
    let q = &heading(block.quantity);
    if block.quantity == HistoryQuantity::ContactElements {
        writeln!(out, " {q} for time  {time}")?;
        writeln!(out)?;
        for (_, values) in &block.rows {
            writeln!(out, "{:>17}", values.first().copied().unwrap_or(0.0) as u64)?;
        }
        writeln!(out)?;
        return Ok(());
    }
    if let HistorySource::Contact { slave, master } = block.source {
        // A compression only support rests on the ground.
        let master = master.map_or("GROUND", |m| sets.surface_name(m));
        writeln!(
            out,
            " statistics for slave set {}, master set {master} and time  {time}",
            sets.surface_name(slave)
        )?;
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

fn write_block(out: &mut impl Write, block: &HistoryBlock, sets: &Sets) -> std::io::Result<()> {
    let quantity = heading(block.quantity);
    if is_contact_quantity(block.quantity) && !matches!(block.source, HistorySource::Named(_)) {
        return write_contact_block(out, block, sets);
    }
    writeln!(out)?;
    writeln!(
        out,
        " {} for set {} and time  {}",
        quantity,
        printed_set(block, sets),
        fortran_time(block.time)
    )?;
    writeln!(out)?;
    let per_point = quantity.contains("integ.pnt.");
    let totals = quantity.starts_with("total ");
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

fn is_contact_quantity(quantity: HistoryQuantity) -> bool {
    use HistoryQuantity as Q;
    matches!(
        quantity,
        Q::ContactElements
            | Q::ContactStatistics
            | Q::RelativeContactDisplacement { .. }
            | Q::ContactStress { .. }
            | Q::ContactSpringEnergy { .. }
    )
}

const DESIGN_RESPONSE_RULE: &str = "   ##############################################################################################################";

/// The table a `*SENSITIVITY` step prints per design response (or per mode for
/// `EIGENFREQUENCY`), [`HistoryQuantity::DesignResponses`]: the source names the response
/// kinds of the rows joined by commas (`MASS,STRAINENERGY`; the writer names every response
/// by its kind), each row is `(index, [value, length of the gradient])`.
fn write_design_response_table(out: &mut impl Write, block: &HistoryBlock) -> std::io::Result<()> {
    let names: Vec<&str> = match &block.source {
        HistorySource::Named(names) => names.split(',').collect(),
        _ => Vec::new(),
    };
    writeln!(out)?;
    writeln!(out, "{DESIGN_RESPONSE_RULE}")?;
    writeln!(
        out,
        "   D E S I G N   R E S P O N S E                    I N F O R M A T I O N"
    )?;
    writeln!(out)?;
    writeln!(
        out,
        "   FUNCTION        FUNCTION VALUE    LENGTH GRADIENT   NAME                                                        "
    )?;
    writeln!(out, "{DESIGN_RESPONSE_RULE}")?;
    writeln!(out)?;
    for (id, values) in &block.rows {
        let name = names.get(*id as usize).copied().unwrap_or("");
        writeln!(
            out,
            "   {:<16} {:>13}{:>19}   {:<80}",
            name,
            fortran_time(values.first().copied().unwrap_or(0.0)),
            fortran_time(values.get(1).copied().unwrap_or(0.0)),
            name
        )?;
    }
    Ok(())
}

/// The tables an eigenvalue step prints, rows keyed by mode (0 for a total row).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EigenTable {
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
    /// A frequency step of a cyclic symmetric model, per nodal diameter: rows
    /// `(mode, [nodal diameter, eigenvalue, omega, frequency, imaginary part])`.
    CyclicEigenvalues,
    /// The turning direction of the modes of a cyclic symmetric model: a first row
    /// `(0, [axis x, y, z])` with the reference direction, then rows
    /// `(mode, [nodal diameter, +1 forward / -1 backward])`.
    CyclicTurningDirection,
    /// Rows `(mode, [+1 forward or −1 backward, nodal diameter or −1])` and a row
    /// `(0, [axis x, y, z])`; the nodal diameter is Nodalis's own column.
    TurningDirection,
}

impl EigenTable {
    fn of(block: &HistoryBlock) -> Option<EigenTable> {
        use HistoryQuantity as Q;
        Some(match block.quantity {
            Q::Eigenvalues => EigenTable::Eigenvalues,
            Q::ParticipationFactors => EigenTable::ParticipationFactors,
            Q::EffectiveModalMass => EigenTable::EffectiveModalMass,
            Q::TotalEffectiveMass => EigenTable::TotalEffectiveMass,
            Q::BucklingFactors => EigenTable::BucklingFactors,
            Q::ComplexEigenvalues => EigenTable::ComplexEigenvalues,
            Q::CyclicEigenvalues => EigenTable::CyclicEigenvalues,
            Q::TurningDirection => EigenTable::TurningDirection,
            Q::CyclicTurningDirection => EigenTable::CyclicTurningDirection,
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
            EigenTable::CyclicEigenvalues => "E I G E N V A L U E   O U T P U T",
            EigenTable::TurningDirection | EigenTable::CyclicTurningDirection => {
                "E I G E N M O D E   T U R N I N G   D I R E C T I O N"
            }
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
        EigenTable::CyclicEigenvalues => {
            writeln!(
                out,
                "  NODAL   MODE NO    EIGENVALUE                      FREQUENCY"
            )?;
            writeln!(
                out,
                " DIAMETER                                    REAL PART            IMAGINARY PART"
            )?;
            writeln!(
                out,
                "                                    (RAD/TIME)      (CYCLES/TIME)   (RAD/TIME)"
            )?;
            writeln!(out)?;
            for (mode, values) in &block.rows {
                write!(out, "{:>5}{mode:>11}", values[0] as u32)?;
                for v in &values[1..] {
                    write!(out, "{:>16}", fortran_time(*v))?;
                }
                writeln!(out)?;
            }
            return Ok(());
        }
        EigenTable::CyclicTurningDirection => {
            let mut rows = block.rows.iter();
            if let Some((_, axis)) = rows.next() {
                write!(out, "    Axis reference direction: ")?;
                for v in axis {
                    write!(out, "{:>12}", fortran_e10(*v))?;
                }
                writeln!(out)?;
            }
            writeln!(out)?;
            writeln!(
                out,
                "  NODAL   MODE NO     TURNING DIRECTION (F=FORWARD,B=BACKWARD)"
            )?;
            writeln!(out, " DIAMETER")?;
            writeln!(out)?;
            for (mode, values) in rows {
                let direction = if values[1] > 0.0 { "F" } else { "B" };
                writeln!(out, "{:>5}{mode:>11}{direction:>11}", values[0] as u32)?;
            }
            return Ok(());
        }
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
            let diameters = block.rows.iter().any(|r| r.0 != 0 && r.1.len() > 1);
            if diameters {
                writeln!(
                    out,
                    " MODE NO     TURNING DIRECTION (F=FORWARD,B=BACKWARD)   NODAL DIAMETER"
                )?;
            } else {
                writeln!(out, " MODE NO     TURNING DIRECTION (F=FORWARD,B=BACKWARD)")?;
            }
            writeln!(out)?;
            for (id, values) in block.rows.iter().filter(|r| r.0 != 0) {
                let direction = if values.first().copied().unwrap_or(1.0) >= 0.0 {
                    'F'
                } else {
                    'B'
                };
                write!(out, "{id:>7}          {direction}")?;
                if diameters {
                    match values.get(1) {
                        Some(d) if *d >= 0.0 => write!(out, "{:>29}", *d as u32)?,
                        _ => write!(out, "{:>29}", "-")?,
                    }
                }
                writeln!(out)?;
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
