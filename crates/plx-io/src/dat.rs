//! Reader for the history output the solver prints into the `.dat` file (`*NODE PRINT`,
//! `*EL PRINT`, `*CONTACT PRINT` and the eigenvalue output of frequency steps), from
//! CalculiX's `.dat` file.
//!
//! Every block of the file names a quantity, a set and a time:
//!
//! ```text
//!  displacements (vx,vy,vz) for set TIP and time  0.1000000E+01
//!
//!         41 -1.422638E-02  3.592771E-05 -1.902165E-01
//! ```
//!
//! The blocks become history sets named after the set, with the field and component
//! names (`DISPLACEMENTS`, `U1`), one entry per node, element or integration point and one
//! row per increment, or per mode of a frequency step. Sets nodalis writes for picked
//! regions (`INTERNAL_SELECTION-1_NH_OUTPUT-1`) are shown under the name of the history
//! output (`NH_OUTPUT-1`).

use std::collections::HashMap;
use std::path::Path;

use plx_results::history_output::{HistoryComponent, HistoryEntry, HistoryField, HistorySet};
use plx_results::principal_values;

/// History sets of a `.dat` file, and what could not be read.
#[derive(Debug, Default)]
pub struct DatImport {
    pub sets: Vec<HistorySet>,
    pub warnings: Vec<String>,
}

/// Name of the set of values the solver prints for all contact elements.
pub const ALL_CONTACT_ELEMENTS: &str = "ALL_CONTACT_ELEMENTS";

pub fn read_dat(path: &Path) -> std::io::Result<DatImport> {
    let bytes = std::fs::read(path)?;
    let mut text = String::from_utf8_lossy(&bytes).into_owned();
    // Names too long for CalculiX were shortened; the input file beside names them in full.
    let input = ["inp", "INP"]
        .map(|e| path.with_extension(e))
        .into_iter()
        .find(|p| p.is_file());
    if let Some(input) = input.and_then(|p| std::fs::read(p).ok()) {
        text = restore_shortened_names(&text, &String::from_utf8_lossy(&input));
    }
    Ok(parse_dat(&text))
}

/// The `.dat` text with the full names of the names `input` shortened.
pub fn restore_shortened_names(dat: &str, input: &str) -> String {
    let mut text = dat.to_owned();
    for line in input.lines() {
        let Some((short, full)) =
            (line.strip_prefix(crate::inp::SHORTENED_NAME)).and_then(|rest| rest.split_once(" = "))
        else {
            continue;
        };
        text = text.replace(
            &short.to_ascii_uppercase(),
            &full.trim().to_ascii_uppercase(),
        );
    }
    text
}

/// How the columns of a block are read.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Ids {
    /// No id column: a sum, one entry per component.
    None,
    /// A node or element number.
    Id,
    /// Element number and integration point, or contact element and face.
    IdPoint,
}

struct Kind {
    /// Start of the block's first line, lower case.
    key: &'static str,
    field: &'static str,
    components: &'static [&'static str],
    ids: Ids,
}

const STRESS: &[&str] = &["S11", "S22", "S33", "S12", "S13", "S23"];
const STRAIN: &[&str] = &["E11", "E22", "E33", "E12", "E13", "E23"];

/// The blocks the solver writes, with their field and component names.
const KINDS: &[Kind] = &[
    kind(
        "displacements",
        "DISPLACEMENTS",
        &["U1", "U2", "U3"],
        Ids::Id,
    ),
    kind("velocities", "VELOCITIES", &["V1", "V2", "V3"], Ids::Id),
    kind("forces", "FORCES", &["RF1", "RF2", "RF3"], Ids::Id),
    kind(
        "total force",
        "TOTAL_FORCE",
        &["RF1", "RF2", "RF3"],
        Ids::None,
    ),
    kind("temperatures", "TEMPERATURES", &["T"], Ids::Id),
    kind("heat generation", "HEAT_GENERATION", &["RFL"], Ids::Id),
    kind(
        "total heat generation",
        "TOTAL_HEAT_GENERATION",
        &["RFL"],
        Ids::None,
    ),
    kind("stresses", "STRESSES", STRESS, Ids::IdPoint),
    kind("strains", "STRAINS", STRAIN, Ids::IdPoint),
    kind(
        "mechanical strains",
        "MECHANICAL_STRAINS",
        STRAIN,
        Ids::IdPoint,
    ),
    kind(
        "equivalent plastic strain",
        "EQUIVALENT_PLASTIC_STRAIN",
        &["PEEQ"],
        Ids::IdPoint,
    ),
    kind(
        "internal energy density",
        "INTERNAL_ENERGY_DENSITY",
        &["ENER"],
        Ids::IdPoint,
    ),
    kind("internal energy", "INTERNAL_ENERGY", &["ELSE"], Ids::Id),
    kind(
        "total internal energy",
        "TOTAL_INTERNAL_ENERGY",
        &["SE"],
        Ids::None,
    ),
    kind("kinetic energy", "KINETIC_ENERGY", &["ELKE"], Ids::Id),
    kind(
        "total kinetic energy",
        "TOTAL_KINETIC_ENERGY",
        &["KE"],
        Ids::None,
    ),
    kind(
        "total model internal energy",
        "MODEL_INTERNAL_ENERGY",
        &["ALLIE"],
        Ids::None,
    ),
    kind(
        "total model kinetic energy",
        "MODEL_KINETIC_ENERGY",
        &["ALLKE"],
        Ids::None,
    ),
    kind(
        "total model external work",
        "MODEL_EXTERNAL_WORK",
        &["ALLWK"],
        Ids::None,
    ),
    kind(
        "total model damping dissipation",
        "MODEL_DAMPING_DISSIPATION",
        &["ALLVD"],
        Ids::None,
    ),
    kind("volume", "VOLUME", &["EVOL"], Ids::Id),
    kind("total volume", "TOTAL_VOLUME", &["VOL"], Ids::None),
    kind("heat flux", "HEAT_FLUX", &["Q1", "Q2", "Q3"], Ids::IdPoint),
    kind("body heating", "BODY_HEATING", &["EBHE"], Ids::Id),
    kind(
        "total body heating",
        "TOTAL_BODY_HEATING",
        &["BHE"],
        Ids::None,
    ),
    kind(
        "relative contact displacement",
        "RELATIVE_CONTACT_DISPLACEMENT",
        &["NORMAL", "TANG1", "TANG2"],
        Ids::IdPoint,
    ),
    kind(
        "contact stress",
        "CONTACT_STRESS",
        &["PRESS", "TANG1", "TANG2"],
        Ids::IdPoint,
    ),
    kind(
        "contact print energy",
        "CONTACT_PRINT_ENERGY",
        &["ENERGY"],
        Ids::IdPoint,
    ),
    kind(
        "contact spring energy",
        "CONTACT_SPRING_ENERGY",
        &["ENERGY"],
        Ids::IdPoint,
    ),
    kind(
        "total contact spring energy",
        "TOTAL_CONTACT_SPRING_ENERGY",
        &["ENERGY"],
        Ids::None,
    ),
    kind(
        "total number of contact elements",
        "TOTAL_NUMBER_OF_CONTACT_ELEMENTS",
        &["NUM"],
        Ids::None,
    ),
];

const fn kind(
    key: &'static str,
    field: &'static str,
    components: &'static [&'static str],
    ids: Ids,
) -> Kind {
    Kind {
        key,
        field,
        components,
        ids,
    }
}

/// The kind of a block's first line: the longest key it starts with, so that "total force"
/// is not taken for "forces" nor "internal energy density" for "internal energy".
fn kind_of(line: &str) -> Option<&'static Kind> {
    let lower = line.trim_start().to_ascii_lowercase();
    KINDS
        .iter()
        .filter(|k| {
            lower.starts_with(k.key)
                && (lower[k.key.len()..].starts_with([' ', '(']) || lower.len() == k.key.len())
        })
        .max_by_key(|k| k.key.len())
}

/// Collects values by set, field, component and entry, with a row per step and increment.
#[derive(Default)]
struct Collector {
    rows: Vec<(u32, u32, f64)>,
    /// Rows by step, increment and the bits of their time or frequency.
    row_index: HashMap<(u32, u32, u64), usize>,
    sets: Vec<SetData>,
}

#[derive(Default)]
struct SetData {
    name: String,
    /// Rows of this set, in the order of the file.
    rows: Vec<usize>,
    fields: Vec<(String, Components)>,
}

/// Components of a field by name, with their entries.
type Components = Vec<(String, Vec<EntryData>)>;

struct EntryData {
    name: String,
    /// Sum and count of the values by row; contact elements print a line per integration
    /// point of a face, which are averaged.
    values: HashMap<usize, (f64, u32)>,
}

impl Collector {
    /// The row of a block. The time tells rows apart too: some CalculiX builds print no
    /// step and increment lines into the `.dat` file, and every time then gets its own
    /// row, numbered as increments of step 0.
    fn row(&mut self, step: u32, increment: u32, value: f64) -> usize {
        let key = (step, increment, value.to_bits());
        if let Some(&row) = self.row_index.get(&key) {
            return row;
        }
        let increment = if step == 0 && increment == 0 {
            self.rows.iter().filter(|r| r.0 == 0).count() as u32 + 1
        } else {
            increment
        };
        self.rows.push((step, increment, value));
        self.row_index.insert(key, self.rows.len() - 1);
        self.rows.len() - 1
    }

    fn add(&mut self, set: &str, field: &str, component: &str, entry: &str, row: usize, v: f64) {
        let set = match self.sets.iter().position(|s| s.name == set) {
            Some(i) => &mut self.sets[i],
            None => {
                self.sets.push(SetData {
                    name: set.to_string(),
                    ..SetData::default()
                });
                self.sets.last_mut().unwrap()
            }
        };
        if !set.rows.contains(&row) {
            set.rows.push(row);
        }
        let fields = &mut set.fields;
        let f = match fields.iter().position(|(n, _)| n == field) {
            Some(f) => f,
            None => {
                fields.push((field.to_string(), Vec::new()));
                fields.len() - 1
            }
        };
        let components = &mut fields[f].1;
        let c = match components.iter().position(|(n, _)| n == component) {
            Some(c) => c,
            None => {
                components.push((component.to_string(), Vec::new()));
                components.len() - 1
            }
        };
        let entries = &mut components[c].1;
        let e = match entries.iter().position(|e| e.name == entry) {
            Some(e) => e,
            None => {
                entries.push(EntryData {
                    name: entry.to_string(),
                    values: HashMap::new(),
                });
                entries.len() - 1
            }
        };
        let slot = entries[e].values.entry(row).or_insert((0.0, 0));
        slot.0 += v;
        slot.1 += 1;
    }

    fn finish(self) -> Vec<HistorySet> {
        let rows = self.rows;
        self.sets
            .into_iter()
            .map(|set| {
                let mut order = set.rows.clone();
                order.sort_unstable();
                let fields = set
                    .fields
                    .into_iter()
                    .map(|(name, components)| HistoryField {
                        name,
                        components: components
                            .into_iter()
                            .map(|(name, entries)| HistoryComponent {
                                name,
                                entries: entries
                                    .into_iter()
                                    .map(|e| HistoryEntry {
                                        name: e.name,
                                        values: (order.iter())
                                            .map(|r| {
                                                e.values
                                                    .get(r)
                                                    .map_or(f64::NAN, |(s, n)| s / *n as f64)
                                            })
                                            .collect(),
                                    })
                                    .collect(),
                            })
                            .collect(),
                    })
                    .collect();
                let mut set = HistorySet {
                    name: set.name,
                    rows: order.iter().map(|&r| rows[r]).collect(),
                    fields,
                };
                add_stress_components(&mut set);
                set
            })
            .collect()
    }
}

/// Parses the text of a `.dat` file.
/// The mode of a row of a cyclic symmetric frequency step, numbered on through the step:
/// rows 1 and 2 are mode 1, the second of each pair is skipped.
fn cyclic_mode(row: u32) -> Option<u32> {
    (row % 2 == 1).then(|| row.div_ceil(2))
}

pub fn parse_dat(text: &str) -> DatImport {
    let lines: Vec<&str> = text.lines().collect();
    let mut data = Collector::default();
    let mut warnings = Vec::new();
    let mut unknown: Vec<String> = Vec::new();
    let mut step = 0u32;
    let mut increment = 0u32;
    // Mode of the eigenvalue block a frequency step prints its values in, and the
    // frequencies of the step's modes.
    let mut mode: Option<u32> = None;
    let mut frequencies: HashMap<u32, f64> = HashMap::new();
    // A cyclic symmetric frequency step prints one eigenvalue table per nodal diameter,
    // each counting its modes from 1; the modes are numbered on through the step, as in
    // the .frd file. `offset` is the number of rows before the table. Every mode comes
    // twice, as a cosine and a sine part; only the first of each pair is kept, as the .frd
    // file does (see `cyclic_mode`).
    let (mut offset, mut cyclic_modes) = (0u32, 0u32);
    // Sets with the values of the rotation node of a point.
    let mut rotation_sets: Vec<String> = Vec::new();
    let mut i = 0;
    while i < lines.len() {
        let line = lines[i];
        let trimmed = line.trim();
        i += 1;
        if trimmed.is_empty() {
            continue;
        }
        if let Some(n) = spaced_number(trimmed, "S T E P") {
            step = n;
            increment = 0;
            mode = None;
            frequencies.clear();
            (offset, cyclic_modes) = (0, 0);
            continue;
        }
        if let Some(n) = trimmed
            .strip_prefix("INCREMENT")
            .and_then(|n| n.trim().parse().ok())
        {
            increment = n;
            continue;
        }
        if let Some(n) = spaced_number(trimmed, "E I G E N V A L U E    N U M B E R") {
            mode = Some(n);
            continue;
        }
        if trimmed == "E I G E N V A L U E   O U T P U T" {
            let table = table_after(&lines, &mut i);
            let set = format!("STEP_{step}");
            offset = cyclic_modes;
            for values in &table {
                // Nodal diameter, mode, eigenvalue, omega, frequency, imaginary part.
                if values.len() == 6 {
                    cyclic_modes += 1;
                    let Some(m) = cyclic_mode(offset + values[1] as u32) else {
                        continue;
                    };
                    frequencies.insert(m, values[4]);
                    let row = data.row(step, m, values[4]);
                    let names = [
                        "NODAL_DIAMETER",
                        "EIGENVALUE",
                        "OMEGA",
                        "FREQUENCY",
                        "FREQUENCY_IM",
                    ];
                    for (name, value) in names.iter().zip([values[0]].iter().chain(&values[2..])) {
                        data.add(&set, "EIGENVALUE_OUTPUT", name, name, row, *value);
                    }
                    continue;
                }
                // A frequency step prints eigenvalue, omega, frequency and imaginary part; a
                // complex frequency step the real part as omega and frequency, then the
                // imaginary part of omega, the damping of the whirling mode.
                let (frequency, names): (f64, &[&str]) = match values.len() {
                    5 => (
                        values[3],
                        &["EIGENVALUE", "OMEGA", "FREQUENCY", "FREQUENCY_IM"],
                    ),
                    4 => (values[2], &["OMEGA", "FREQUENCY", "OMEGA_IM"]),
                    _ => continue,
                };
                let m = values[0] as u32;
                frequencies.insert(m, frequency);
                let row = data.row(step, m, frequency);
                for (name, value) in names.iter().zip(&values[1..]) {
                    data.add(&set, "EIGENVALUE_OUTPUT", name, name, row, *value);
                }
            }
            continue;
        }
        // The whirl of each complex mode: F (forward, with the rotation) or B (backward).
        if trimmed == "E I G E N M O D E   T U R N I N G   D I R E C T I O N" {
            let set = format!("STEP_{step}");
            while i < lines.len() {
                let line = lines[i].trim();
                i += 1;
                let mut tokens = line.split_whitespace();
                let Some(mut m) = tokens.next().and_then(|t| t.parse::<u32>().ok()) else {
                    let header = ["Axis", "MODE", "NODAL", "DIAMETER"];
                    if line.is_empty() || header.iter().any(|h| line.starts_with(h)) {
                        continue;
                    }
                    // The next block, such as the eigenvalues of the next nodal diameter.
                    i -= 1;
                    break;
                };
                // Cyclic symmetry: nodal diameter, then the mode within it, then the
                // direction. Nodalis's complex modes: the direction, then the nodal diameter
                // of the mode (`-` without one).
                let mut next = tokens.next();
                if let Some(local) = next.and_then(|t| t.parse::<u32>().ok()) {
                    let Some(mode) = cyclic_mode(offset + local) else {
                        continue;
                    };
                    m = mode;
                    next = tokens.next();
                }
                let direction = match next {
                    Some("F") => 1.0,
                    Some("B") => -1.0,
                    _ => 0.0,
                };
                let frequency = frequencies.get(&m).copied().unwrap_or(m as f64);
                let row = data.row(step, m, frequency);
                let name = "TURNING_DIRECTION";
                data.add(&set, "EIGENVALUE_OUTPUT", name, name, row, direction);
                // Every mode gets a value where the column exists, −1 without a diameter,
                // so the values line up with the modes.
                if let Some(diameter) = tokens.next() {
                    let diameter = diameter.parse::<u32>().map_or(-1.0, f64::from);
                    let name = "NODAL_DIAMETER";
                    data.add(&set, "EIGENVALUE_OUTPUT", name, name, row, diameter);
                }
            }
            continue;
        }
        // A buckle step lists its buckling factors by mode.
        if trimmed == "B U C K L I N G   F A C T O R   O U T P U T" {
            let table = table_after(&lines, &mut i);
            let set = format!("STEP_{step}");
            for values in table.iter().filter(|v| v.len() == 2) {
                let m = values[0] as u32;
                let row = data.row(step, m, values[1]);
                let name = "BUCKLING_FACTOR";
                data.add(&set, "BUCKLING_FACTOR_OUTPUT", name, name, row, values[1]);
            }
            continue;
        }
        // A sensitivity step lists each design response with its value and the length of
        // its gradient: "STRAINENERGY  0.67E+01  0.17E-01  STRAINENERGY".
        if trimmed.starts_with("D E S I G N   R E S P O N S E")
            && trimmed.ends_with("I N F O R M A T I O N")
        {
            let set = format!("STEP_{step}");
            let row = data.row(step, 1, 0.0);
            while i < lines.len() {
                let line = lines[i].trim();
                let tokens: Vec<&str> = line.split_whitespace().collect();
                let numbers = (tokens.get(1).and_then(|t| t.parse::<f64>().ok()))
                    .zip(tokens.get(2).and_then(|t| t.parse::<f64>().ok()));
                if let Some((value, gradient)) = numbers {
                    let name = tokens.get(3).unwrap_or(&tokens[0]);
                    data.add(&set, "DESIGN_RESPONSE", name, name, row, value);
                    data.add(&set, "GRADIENT_LENGTH", name, name, row, gradient);
                } else if !(line.is_empty()
                    || line.starts_with('#')
                    || line.starts_with("FUNCTION"))
                {
                    break;
                }
                i += 1;
            }
            continue;
        }
        let modal = [
            (
                "P A R T I C I P A T I O N   F A C T O R S",
                "PARTICIPATION_FACTORS",
            ),
            (
                "E F F E C T I V E   M O D A L   M A S S",
                "EFFECTIVE_MODAL_MASS",
            ),
        ];
        if let Some((_, field)) = modal.iter().find(|(key, _)| trimmed == *key) {
            let table = table_after(&lines, &mut i);
            let set = format!("STEP_{step}");
            for values in table.iter().filter(|v| v.len() == 7) {
                let m = values[0] as u32;
                let m = if cyclic_modes > 0 {
                    let Some(m) = cyclic_mode(offset + m) else {
                        continue;
                    };
                    m
                } else {
                    m
                };
                let frequency = frequencies.get(&m).copied().unwrap_or(m as f64);
                let row = data.row(step, m, frequency);
                let names = [
                    "X_COMPONENT",
                    "Y_COMPONENT",
                    "Z_COMPONENT",
                    "X_ROTATION",
                    "Y_ROTATION",
                    "Z_ROTATION",
                ];
                for (name, value) in names.iter().zip(&values[1..]) {
                    data.add(&set, field, name, name, row, *value);
                }
            }
            continue;
        }
        let lower = trimmed.to_ascii_lowercase();
        if lower.starts_with("statistics for slave set") {
            let Some((names, time)) = header_parts(trimmed) else {
                continue;
            };
            let row = row_of(&mut data, step, increment, mode, &frequencies, time);
            read_statistics(&lines, &mut i, &names, row, &mut data);
            continue;
        }
        if !lower.contains("time") {
            continue;
        }
        let Some((set, time)) = header_parts(trimmed) else {
            continue;
        };
        let Some(kind) = kind_of(trimmed) else {
            let description = trimmed.split(['(']).next().unwrap_or(trimmed);
            let description = description.split(" for ").next().unwrap_or(description);
            if !unknown.iter().any(|u| u == description.trim()) {
                unknown.push(description.trim().to_string());
            }
            continue;
        };
        let kind = if trimmed.to_ascii_uppercase().contains(ROTATION_PREFIX) {
            if !rotation_sets.contains(&set) {
                rotation_sets.push(set.clone());
            }
            rotation_kind(kind)
        } else {
            kind
        };
        let row = row_of(&mut data, step, increment, mode, &frequencies, time);
        // The values follow after blank lines and end with a blank line.
        while i < lines.len() && lines[i].trim().is_empty() {
            i += 1;
        }
        while i < lines.len() && !lines[i].trim().is_empty() {
            let values = numbers(lines[i]);
            let line_number = i + 1;
            i += 1;
            let offset = match kind.ids {
                Ids::None => 0,
                Ids::Id => 1,
                Ids::IdPoint => 2,
            };
            if values.len() < offset + kind.components.len() {
                warnings.push(format!("Line {line_number}: too few values"));
                continue;
            }
            let entry = match kind.ids {
                Ids::None => None,
                Ids::Id => Some(format!("{}", values[0] as i64)),
                Ids::IdPoint => Some(format!("{}_{}", values[0] as i64, values[1] as i64)),
            };
            for (component, value) in kind.components.iter().zip(&values[offset..]) {
                let entry = entry.as_deref().unwrap_or(component);
                data.add(&set, kind.field, component, entry, row, *value);
            }
        }
    }
    if !unknown.is_empty() {
        warnings.push(format!(
            "Unsupported history outputs skipped: {}",
            unknown.join(", ")
        ));
    }
    let mut sets = data.finish();
    for set in sets.iter_mut().filter(|s| rotation_sets.contains(&s.name)) {
        name_rotations_after_point(set);
    }
    DatImport { sets, warnings }
}

/// Shows the rotations and moments of a point under its reference node, the one node of the
/// set's other values, rather than under the rotation node.
fn name_rotations_after_point(set: &mut HistorySet) {
    let rotational = |name: &str| ROTATION_KINDS.iter().any(|k| k.field == name);
    let mut nodes: Vec<&str> = (set.fields.iter())
        .filter(|f| !rotational(&f.name))
        .flat_map(|f| &f.components)
        .flat_map(|c| &c.entries)
        .map(|e| e.name.as_str())
        .filter(|name| name.parse::<u64>().is_ok())
        .collect();
    nodes.dedup();
    let [node] = nodes[..] else {
        return;
    };
    let node = node.to_string();
    for field in set.fields.iter_mut().filter(|f| rotational(&f.name)) {
        for entry in field.components.iter_mut().flat_map(|c| &mut c.entries) {
            if entry.name.parse::<u64>().is_ok() {
                entry.name = node.clone();
            }
        }
    }
}

/// The row of a block: by step and increment, or by mode inside the eigenvalue blocks of a
/// frequency step, where the row's value is the mode's frequency instead of the time.
fn row_of(
    data: &mut Collector,
    step: u32,
    increment: u32,
    mode: Option<u32>,
    frequencies: &HashMap<u32, f64>,
    time: f64,
) -> usize {
    match mode {
        Some(m) => {
            let frequency = frequencies.get(&m).copied().unwrap_or(time);
            data.row(step, m, frequency)
        }
        None => data.row(step, increment, time),
    }
}

/// The number after a spaced out title such as `S T E P       1`.
fn spaced_number(line: &str, title: &str) -> Option<u32> {
    line.strip_prefix(title)?.trim().parse().ok()
}

/// Set and time of a block's first line. Values printed for all contact elements, or for a
/// time only, belong to the set of all contact elements.
fn header_parts(line: &str) -> Option<(String, f64)> {
    let lower = line.to_ascii_lowercase();
    let at = lower.rfind("time")?;
    let time = line[at + 4..].trim().parse().ok()?;
    let head = &line[..at];
    let lower_head = &lower[..at];
    let set = if let Some(start) = lower_head.find("slave set") {
        // statistics for slave set SS, master set MS and time ...
        let rest = &head[start + "slave set".len()..];
        let (slave, master) = rest.split_once(", master set")?;
        let master = master.trim().trim_end_matches("and").trim();
        contact_set_name(slave.trim(), master)
    } else if let Some(start) = lower_head.find("set ") {
        // "for set", "forset" and "eneset" of older versions alike.
        let rest = head[start + 4..].trim();
        let name = rest.split_whitespace().next()?;
        repair_set_name(name)
    } else {
        ALL_CONTACT_ELEMENTS.to_string()
    };
    Some((set, time))
}

/// Start of the sets nodalis writes for the rotation node of a reference point
/// (`INTERNAL_ROTATION-2_NH_OUTPUT-1`): its displacements are the rotations of the point, its
/// forces the moments.
const ROTATION_PREFIX: &str = "INTERNAL_ROTATION-";

/// The rotational counterparts of the blocks of a rotation node, shown with the values of
/// the point's reference node.
const ROTATION_KINDS: &[Kind] = &[
    kind(
        "displacements",
        "ROTATIONS",
        &["UR1", "UR2", "UR3"],
        Ids::Id,
    ),
    kind(
        "velocities",
        "ANGULAR_VELOCITIES",
        &["VR1", "VR2", "VR3"],
        Ids::Id,
    ),
    kind("forces", "MOMENTS", &["RM1", "RM2", "RM3"], Ids::Id),
    kind(
        "total force",
        "TOTAL_MOMENT",
        &["RM1", "RM2", "RM3"],
        Ids::None,
    ),
];

/// The block of a rotation node a block kind stands for; other kinds stay.
fn rotation_kind(kind: &'static Kind) -> &'static Kind {
    (ROTATION_KINDS.iter())
        .find(|k| k.key == kind.key)
        .unwrap_or(kind)
}

/// The name shown for a set: without the prefix of the sets written for picked
/// regions.
fn repair_set_name(name: &str) -> String {
    let upper = name.to_ascii_uppercase();
    for prefix in ["INTERNAL_SELECTION-", ROTATION_PREFIX, "INTERNAL-"] {
        if let Some(rest) = upper.strip_prefix(prefix)
            && let Some((number, name)) = rest.split_once('_')
            && number.chars().all(|c| c.is_ascii_digit())
            && !name.is_empty()
        {
            return name.to_string();
        }
    }
    upper
}

/// Name of the contact statistics of a slave and a master surface: the contact pair's name
/// when both are surfaces written for its picked regions.
fn contact_set_name(slave: &str, master: &str) -> String {
    let slave = repair_set_name(slave);
    let master = repair_set_name(master);
    let slave = slave.strip_suffix("_SLAVE").unwrap_or(&slave);
    let master = master.strip_suffix("_MASTER").unwrap_or(&master);
    if slave == master {
        slave.to_string()
    } else {
        format!("{slave}_{master}")
    }
}

/// The contact statistics after their first line: four labelled lines of values.
fn read_statistics(lines: &[&str], i: &mut usize, set: &str, row: usize, data: &mut Collector) {
    let mut rows: Vec<Vec<f64>> = Vec::new();
    while *i < lines.len() && rows.len() < 4 {
        let line = lines[*i].trim();
        if line.is_empty() || line.starts_with(|c: char| c.is_ascii_alphabetic()) {
            if line.to_ascii_lowercase().contains(" set ") {
                break;
            }
            *i += 1;
            continue;
        }
        rows.push(numbers(line));
        *i += 1;
    }
    if rows.len() < 4 || rows[0].len() < 6 || rows[1].len() < 6 || rows[2].len() < 3 {
        return;
    }
    let mut put = |field: &str, names: &[&str], values: &[f64]| {
        for (name, value) in names.iter().zip(values) {
            data.add(set, field, name, name, row, *value);
        }
    };
    put("TOTAL_SURFACE_FORCE", &["FX", "FY", "FZ"], &rows[0][..3]);
    put("MOMENT_ABOUT_ORIGIN", &["MX", "MY", "MZ"], &rows[0][3..6]);
    put("CENTER_OF_GRAVITY_CG", &["X", "Y", "Z"], &rows[1][..3]);
    put("MEAN_SURFACE_NORMAL", &["NX", "NY", "NZ"], &rows[1][3..6]);
    put("MOMENT_ABOUT_CG", &["MX", "MY", "MZ"], &rows[2][..3]);
    put("SURFACE_AREA", &["A"], &rows[3][..1.min(rows[3].len())]);
    if rows[3].len() >= 3 {
        put(
            "SURFACE_LOADS",
            &["NORMAL_FORCE", "SHEAR_FORCE"],
            &rows[3][1..3],
        );
    }
}

/// The numeric rows of a table such as the eigenvalue output, up to the blank line after
/// them; header lines and the line of totals are skipped.
fn table_after(lines: &[&str], i: &mut usize) -> Vec<Vec<f64>> {
    let mut table = Vec::new();
    while *i < lines.len() {
        let line = lines[*i].trim();
        if line.is_empty() {
            if !table.is_empty() {
                break;
            }
            *i += 1;
            continue;
        }
        let first = line.split_whitespace().next().unwrap_or("");
        if first.parse::<u32>().is_ok() {
            table.push(numbers(line));
        } else if !table.is_empty() {
            break;
        }
        *i += 1;
    }
    table
}

/// The numbers of a line of values; the `L` of local values and the `_shell_` tags of
/// expanded shells are left out.
fn numbers(line: &str) -> Vec<f64> {
    line.split_whitespace()
        .filter(|t| !t.eq_ignore_ascii_case("l") && !t.starts_with('_'))
        .map(parse_number)
        .collect()
}

/// A number as CalculiX prints it, also the three digit exponents it writes without `E`
/// (`1.090361-282`).
fn parse_number(text: &str) -> f64 {
    if let Ok(value) = text.parse() {
        return value;
    }
    if let Some(at) = text[1..].rfind(['-', '+']).map(|a| a + 1)
        && let Ok(value) = format!("{}E{}", &text[..at], &text[at..]).parse()
    {
        return value;
    }
    f64::NAN
}

/// Adds von Mises, Tresca and the principal stresses to stress fields.
fn add_stress_components(set: &mut HistorySet) {
    for field in set.fields.iter_mut().filter(|f| f.name == "STRESSES") {
        let Some(base) = STRESS
            .iter()
            .map(|n| field.component(n))
            .collect::<Option<Vec<_>>>()
        else {
            continue;
        };
        let names = [
            "MISES",
            "TRESCA",
            "SGN_MAX_ABS_PRI",
            "PRINCIPAL_MAX",
            "PRINCIPAL_MID",
            "PRINCIPAL_MIN",
        ];
        let mut derived: Vec<HistoryComponent> = names
            .iter()
            .map(|n| HistoryComponent {
                name: n.to_string(),
                entries: Vec::new(),
            })
            .collect();
        for (e, entry) in base[0].entries.iter().enumerate() {
            let mut columns: Vec<Vec<f64>> = vec![Vec::new(); names.len()];
            for row in 0..entry.values.len() {
                let value = |c: usize| {
                    base[c]
                        .entries
                        .get(e)
                        .and_then(|x| x.values.get(row))
                        .copied()
                        .unwrap_or(f64::NAN)
                };
                // S11, S22, S33, S12, S13, S23 as xx, yy, zz, xy, yz, zx.
                let [xx, yy, zz, xy, xz, yz] = [0, 1, 2, 3, 4, 5].map(value);
                let mises = (0.5
                    * ((xx - yy).powi(2)
                        + (yy - zz).powi(2)
                        + (zz - xx).powi(2)
                        + 6.0 * (xy * xy + yz * yz + xz * xz)))
                    .sqrt();
                let [p1, p2, p3] = principal_values([xx, yy, zz, xy, yz, xz]);
                let signed = if p1.abs() > p3.abs() { p1 } else { p3 };
                for (column, v) in columns.iter_mut().zip([mises, p1 - p3, signed, p1, p2, p3]) {
                    column.push(v);
                }
            }
            for (component, values) in derived.iter_mut().zip(columns) {
                component.entries.push(HistoryEntry {
                    name: entry.name.clone(),
                    values,
                });
            }
        }
        // Nodalis lists the equivalent stresses first and the principal stresses last.
        let principal = derived.split_off(2);
        let raw = std::mem::take(&mut field.components);
        field.components = derived.into_iter().chain(raw).chain(principal).collect();
    }
}

#[cfg(test)]
mod tests;
