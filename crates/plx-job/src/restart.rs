//! Solving again from the last unchanged step: CalculiX writes the state after each step into
//! `<job>.rout` (`*RESTART, WRITE`), and a later run of the same job whose first steps did
//! not change reads it back (`*RESTART, READ, STEP=n`) and solves only the steps after.
//!
//! What a run solves is told apart by the text of its input file: the model definition as a
//! set of keyword blocks, and a fingerprint per step that includes the steps before it. A
//! restart is only taken when it gives the same results as the full run:
//!
//! * the model definition is the same; sets only the new run needs are not allowed, as
//!   CalculiX takes no model definition after `*RESTART, READ`;
//! * the steps up to the restart are the same, and CalculiX wrote their state;
//! * these steps are steps over time (static, dynamic, heat transfer, visco), whose state the
//!   following steps continue; perturbation steps such as a frequency step are solved again;
//! * the results of the steps up to the restart are still there, to be joined with the new
//!   ones into one `.frd` and `.dat` file, numbered as if CalculiX had solved all steps;
//! * the same CalculiX executable wrote the state, as the files depend on its version.
//!
//! Otherwise the job runs in full. The input file kept with the results is always the full
//! one, which runs without the state files.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// The procedures after which CalculiX continues from the state of the step.
const TIME_PROCEDURES: [&str; 6] = [
    "STATIC",
    "DYNAMIC",
    "HEAT TRANSFER",
    "COUPLED TEMPERATURE-DISPLACEMENT",
    "UNCOUPLED TEMPERATURE-DISPLACEMENT",
    "VISCO",
];

/// Procedures CalculiX solves from the state of the step before without continuing it.
const OTHER_PROCEDURES: [&str; 9] = [
    "FREQUENCY",
    "BUCKLE",
    "COMPLEX FREQUENCY",
    "MODAL DYNAMIC",
    "STEADY STATE DYNAMICS",
    "SENSITIVITY",
    "GREEN",
    "ELECTROMAGNETICS",
    "NO ANALYSIS",
];

/// Keyword blocks of the model definition that may be left over from an earlier run: sets
/// the steps of this run no longer use.
const SET_KEYWORDS: [&str; 3] = ["NSET", "ELSET", "SURFACE"];

/// Name of the file holding the [`StateIndex`] of a job.
pub fn index_file(work_dir: &Path, job: &str) -> PathBuf {
    work_dir.join(format!("{job}.restart"))
}

/// The input file of a job taken apart: what tells runs of it apart.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Chain {
    /// The full input file, with `*RESTART, WRITE` in the steps whose state can be used.
    pub input: String,
    /// Fingerprints of the keyword blocks of the model definition, sets separate.
    pub model: Vec<u64>,
    pub sets: Vec<u64>,
    /// Where each step starts in [`Self::input`], and where the last one ends.
    bounds: Vec<usize>,
    /// Fingerprint of each step together with the model and the steps before it.
    pub steps: Vec<u64>,
    /// Number of steps from the first whose state is written: steps over time.
    pub restartable: usize,
}

impl Chain {
    /// Takes the input file apart; with `write_states` the state of the steps over time at
    /// the start of the job is written for later runs. `external` is a fingerprint of files
    /// the job reads besides its input file, such as the results of a global model.
    pub fn new(input: &str, write_states: bool, external: u64) -> Self {
        let lines = line_starts(input);
        let keyword = |at: usize| keyword_name(line_at(input, at));
        let step_starts: Vec<usize> = (lines.iter().copied())
            .filter(|&at| keyword(at).as_deref() == Some("STEP"))
            .collect();
        // A file with its own restart keywords is run as it is.
        let own = (lines.iter()).any(|&at| keyword(at).as_deref() == Some("RESTART"));
        let model_end = step_starts.first().copied().unwrap_or(input.len());
        let mut chain = Chain::default();
        let mut text = String::with_capacity(input.len() + 32 * step_starts.len());
        text.push_str(&input[..model_end]);
        for (block, set) in keyword_blocks(&input[..model_end]) {
            if set {
                chain.sets.push(block);
            } else {
                chain.model.push(block);
            }
        }
        chain.model.push(external);
        chain.model.sort_unstable();
        chain.sets.sort_unstable();
        chain.sets.dedup();
        let mut previous = fingerprint_all(&chain.model);
        let mut restartable = !own && write_states;
        for (i, &start) in step_starts.iter().enumerate() {
            let end = step_starts.get(i + 1).copied().unwrap_or(input.len());
            let step = &input[start..end];
            restartable &= continues_state(step);
            chain.bounds.push(text.len());
            if restartable {
                text.push_str(&with_restart_write(step));
                chain.restartable += 1;
            } else {
                text.push_str(step);
            }
            let mut hash = Fnv::new();
            hash.add(&previous.to_le_bytes());
            hash.add_text(&text[chain.bounds[i]..]);
            previous = hash.finish();
            chain.steps.push(previous);
        }
        chain.bounds.push(text.len());
        chain.input = text;
        chain
    }

    /// The input file of a run that reads the state after step `after` (counted from 1) and
    /// solves the steps after it.
    pub fn restart_input(&self, after: usize) -> String {
        let steps = &self.input[self.bounds[after]..];
        format!(
            "** Restart: the state after step {after} is read from the earlier run.\n\
             *Restart, Read, Step={after}\n{steps}"
        )
    }
}

/// How a job is run: in full or from the state after a step of an earlier run.
#[derive(Clone, Debug, PartialEq)]
pub enum Plan {
    /// Why no restart is possible.
    Full(String),
    Restart {
        /// The last step whose state is read, counted from 1.
        after: usize,
        /// The state file.
        file: PathBuf,
        /// The results file of the run that wrote it, with the results of the steps up to
        /// `after`; its history output is next to it.
        results: PathBuf,
        /// Total time at the end of that step.
        time: f64,
    },
}

/// Whether the job can start from a state, and from which: of its own earlier runs, the
/// first of `jobs` (work directory and job name), or of the jobs of analyses it continues,
/// the others. The state after the latest step wins; on a tie the job's own.
pub fn plan(chain: &Chain, solver: &str, jobs: &[(PathBuf, String)]) -> Plan {
    let mut best: Option<Plan> = None;
    let mut reason = None;
    for (dir, job) in jobs {
        let plan = StateIndex::read(dir, job).plan(chain, solver, dir, job);
        match (&plan, &best) {
            (Plan::Restart { after, .. }, Some(Plan::Restart { after: best, .. }))
                if after <= best => {}
            (Plan::Restart { .. }, _) => best = Some(plan),
            (Plan::Full(_), _) if reason.is_none() => reason = Some(plan),
            _ => {}
        }
    }
    best.or(reason)
        .unwrap_or_else(|| Plan::Full("no state of an earlier run".into()))
}

/// The states a job's earlier runs left in its work directory, saved next to them.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct StateIndex {
    /// The CalculiX that wrote the states and results.
    pub solver: String,
    /// Fingerprints of the steps whose results are in `<job>.frd` and `<job>.dat`.
    pub results: Vec<u64>,
    pub files: Vec<StateFile>,
    /// Number of the next state file.
    pub next: u32,
}

/// A `.rout` file kept from a run, with the states it holds.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct StateFile {
    pub name: String,
    /// The model definition of the run, see [`Chain`].
    pub model: Vec<u64>,
    pub sets: Vec<u64>,
    pub states: Vec<State>,
}

/// The state after a step, as a `.rout` file holds it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct State {
    /// The step, counted from 1 over the whole job.
    pub step: usize,
    /// Fingerprint of the step, see [`Chain::steps`].
    pub fingerprint: u64,
    /// Total time at the end of the step.
    pub time: f64,
}

impl StateIndex {
    /// The index of the job in `work_dir`; empty if there is none or it cannot be read.
    pub fn read(work_dir: &Path, job: &str) -> Self {
        std::fs::read_to_string(index_file(work_dir, job))
            .ok()
            .and_then(|text| ron::from_str(&text).ok())
            .unwrap_or_default()
    }

    pub fn write(&self, work_dir: &Path, job: &str) -> std::io::Result<()> {
        let text = ron::to_string(self).map_err(std::io::Error::other)?;
        std::fs::write(index_file(work_dir, job), text)
    }

    /// Whether the job can start from a state this index lists, and from which.
    pub fn plan(&self, chain: &Chain, solver: &str, work_dir: &Path, job: &str) -> Plan {
        let full = |reason: &str| Plan::Full(reason.to_owned());
        let steps = chain.steps.len();
        if chain.restartable == 0 {
            return full(if steps == 0 {
                "the job has no step"
            } else {
                "its first step does not continue a state (only steps over time do)"
            });
        }
        if self.files.is_empty() {
            return full("no state of an earlier run");
        }
        if self.solver != solver {
            return full("the states were written by another CalculiX");
        }
        let results = work_dir.join(format!("{job}.frd"));
        if !results.is_file() {
            return full("the results of the earlier run are gone");
        }
        // Steps whose results are there and the same as they would be now.
        let same = (self.results.iter().zip(&chain.steps))
            .take_while(|(a, b)| a == b)
            .count();
        if same == 0 {
            return full("the first step or the model changed");
        }
        // The last step is always solved again.
        let last = same.min(chain.restartable).min(steps - 1);
        let mut model_changed = false;
        for after in (1..=last).rev() {
            for file in &self.files {
                let Some(state) = (file.states.iter())
                    .find(|s| s.step == after && s.fingerprint == chain.steps[after - 1])
                else {
                    continue;
                };
                if !work_dir.join(&file.name).is_file() {
                    continue;
                }
                if file.model != chain.model || !is_subset(&chain.sets, &file.sets) {
                    model_changed = true;
                    continue;
                }
                return Plan::Restart {
                    after,
                    file: work_dir.join(&file.name),
                    results: results.clone(),
                    time: state.time,
                };
            }
        }
        if model_changed {
            full("the model definition changed, such as a new set")
        } else if last == 0 {
            full("only the last step is unchanged")
        } else {
            full("no state of the unchanged steps")
        }
    }

    /// Records the run that just ended: keeps its `.rout` as a state file for the steps over
    /// time it finished, notes which results `<job>.frd` holds and drops the states that no
    /// longer fit the job.
    pub fn record(
        &mut self,
        chain: &Chain,
        solver: &str,
        work_dir: &Path,
        job: &str,
        run: &Run,
    ) -> std::io::Result<()> {
        if self.solver != solver {
            // States of another CalculiX are of no use.
            self.files.clear();
            self.solver = solver.to_owned();
        }
        self.results = if run.completed && run.results {
            chain.steps.clone()
        } else {
            Vec::new()
        };
        let rout = work_dir.join(format!("{job}.rout"));
        if rout.is_file() {
            let finished = finished_steps(&run.sta, run.after, run.completed);
            let states: Vec<State> = (finished.into_iter())
                .filter(|(step, _)| *step <= chain.restartable)
                .map(|(step, time)| State {
                    step,
                    fingerprint: chain.steps[step - 1],
                    time,
                })
                .collect();
            if states.is_empty() {
                std::fs::remove_file(&rout)?;
            } else {
                let name = format!("{job}.state-{}.rout", self.next);
                self.next += 1;
                std::fs::rename(&rout, work_dir.join(&name))?;
                self.files.push(StateFile {
                    name,
                    model: chain.model.clone(),
                    sets: chain.sets.clone(),
                    states,
                });
            }
        }
        // The newest state of each step that is still part of the job; files left without
        // states are removed.
        let mut seen = Vec::new();
        for file in self.files.iter_mut().rev() {
            file.states.retain(|s| {
                let fits = chain.steps.get(s.step - 1) == Some(&s.fingerprint)
                    && file.model == chain.model;
                let newest = fits && !seen.contains(&s.step);
                if newest {
                    seen.push(s.step);
                }
                newest
            });
        }
        for file in &self.files {
            if file.states.is_empty() {
                let _ = std::fs::remove_file(work_dir.join(&file.name));
            }
        }
        self.files
            .retain(|f| !f.states.is_empty() && work_dir.join(&f.name).is_file());
        let _ = std::fs::remove_file(work_dir.join(format!("{job}.rin")));
        self.write(work_dir, job)
    }

    /// Forgets all states of the job and removes their files.
    pub fn clear(work_dir: &Path, job: &str) {
        let index = Self::read(work_dir, job);
        for file in &index.files {
            let _ = std::fs::remove_file(work_dir.join(&file.name));
        }
        let _ = std::fs::remove_file(index_file(work_dir, job));
    }
}

/// How a run ended, for [`StateIndex::record`].
#[derive(Clone, Debug, Default)]
pub struct Run {
    /// The step the run started after, 0 for a full run.
    pub after: usize,
    /// CalculiX finished without errors.
    pub completed: bool,
    /// The run left results.
    pub results: bool,
    /// The `.sta` file of the run.
    pub sta: String,
}

/// The steps a run finished with the total time at the end of each, from its `.sta` file;
/// the last step it worked on only counts if the run completed.
fn finished_steps(sta: &str, after: usize, completed: bool) -> Vec<(usize, f64)> {
    let mut steps: Vec<(usize, f64)> = Vec::new();
    for line in sta.lines() {
        let fields: Vec<&str> = line.split_whitespace().collect();
        let (Some(step), Some(time)) = (
            fields.first().and_then(|s| s.parse::<usize>().ok()),
            fields.get(4).and_then(|s| s.parse::<f64>().ok()),
        ) else {
            continue;
        };
        if fields.len() != 7 || step <= after {
            continue;
        }
        match steps.last_mut() {
            Some((last, at)) if *last == step => *at = time,
            _ => steps.push((step, time)),
        }
    }
    if !completed {
        steps.pop();
    }
    steps
}

/// Prepares the work directory for a run from the state after a step: the state file is
/// copied to `<job>.rin`, where CalculiX reads it, and the earlier results the new ones are
/// joined with are put aside: the job's own are moved, as the run starts new ones, those of
/// another job copied.
pub fn prepare(
    work_dir: &Path,
    job: &str,
    state_file: &Path,
    results: &Path,
) -> std::io::Result<()> {
    std::fs::create_dir_all(work_dir)?;
    std::fs::copy(state_file, work_dir.join(format!("{job}.rin")))?;
    let own = work_dir.join(format!("{job}.frd"));
    for extension in ["frd", "dat"] {
        let file = results.with_extension(extension);
        let kept = earlier(work_dir, job, extension);
        if !file.is_file() {
            if kept.is_file() {
                std::fs::remove_file(kept)?;
            }
        } else if results == own {
            std::fs::rename(file, kept)?;
        } else {
            std::fs::copy(file, kept)?;
        }
    }
    Ok(())
}

/// Where [`prepare`] keeps a results file of the earlier run.
fn earlier(work_dir: &Path, job: &str, extension: &str) -> PathBuf {
    work_dir.join(format!("{job}.earlier.{extension}"))
}

/// Removes what [`prepare`] left, when the run did not get to its results.
pub fn discard(work_dir: &Path, job: &str) {
    for extension in ["frd", "dat"] {
        let _ = std::fs::remove_file(earlier(work_dir, job, extension));
    }
    let _ = std::fs::remove_file(work_dir.join(format!("{job}.rin")));
}

/// Joins the results of the steps up to `after` from the earlier run with those of the run
/// that started after it, into `<job>.frd` and `<job>.dat`.
pub fn join_results(work_dir: &Path, job: &str, after: usize, time: f64) -> std::io::Result<()> {
    let frd = work_dir.join(format!("{job}.frd"));
    let old = std::fs::read(earlier(work_dir, job, "frd"))?;
    let new = std::fs::read(&frd).unwrap_or_default();
    std::fs::write(&frd, join_frd(&old, &new, after))?;
    let dat = work_dir.join(format!("{job}.dat"));
    if let Ok(old) = std::fs::read(earlier(work_dir, job, "dat")) {
        let new = std::fs::read(&dat).unwrap_or_default();
        std::fs::write(&dat, join_dat(&old, &new, after, time))?;
    }
    discard(work_dir, job);
    Ok(())
}

/// The results file of a run, as one from the start: the header, nodes, elements and
/// results of the steps up to `after` of `old`, then the results of `new`, numbered on from
/// those before as CalculiX numbers the data sets of a run.
pub fn join_frd(old: &[u8], new: &[u8], after: usize) -> Vec<u8> {
    let step_blocks = |bytes: &[u8]| -> Vec<(usize, usize)> {
        line_starts_bytes(bytes)
            .into_iter()
            .filter_map(|at| {
                let line = std::str::from_utf8(line_bytes(bytes, at)).ok()?;
                // `    1PSTEP   <data set> <increment> <step>`
                let rest = line.strip_prefix("    1PSTEP")?;
                let step = rest.split_whitespace().nth(2)?.parse().ok()?;
                Some((at, step))
            })
            .collect()
    };
    let end = |bytes: &[u8]| {
        (line_starts_bytes(bytes).into_iter().rev())
            .find(|&at| line_bytes(bytes, at).starts_with(b" 9999"))
            .unwrap_or(bytes.len())
    };
    let cut = (step_blocks(old).into_iter())
        .find(|&(_, step)| step > after)
        .map_or_else(|| end(old), |(at, _)| at);
    let start = (step_blocks(new).first())
        .map(|&(at, _)| at)
        .unwrap_or_else(|| end(new));
    let kept = &old[..cut];
    // The last numbers of the data sets kept: of the `1PSTEP` lines and of the result
    // blocks (`  100CL  105 ...    5`, set name and number of the increment).
    let (mut data_sets, mut increments) = (0, 0);
    for at in line_starts_bytes(kept) {
        let line = String::from_utf8_lossy(line_bytes(kept, at));
        if let Some(rest) = line.strip_prefix("    1PSTEP") {
            data_sets = rest
                .split_whitespace()
                .next()
                .and_then(|n| n.parse().ok())
                .unwrap_or(data_sets);
        } else if line.starts_with("  100C") {
            increments = field(&line, 58..63).unwrap_or(increments);
        }
    }
    let mut joined = kept.to_vec();
    let rest = &new[start..];
    let starts = line_starts_bytes(rest);
    for (i, &at) in starts.iter().enumerate() {
        let next = starts.get(i + 1).copied().unwrap_or(rest.len());
        let raw = &rest[at..next];
        let line = std::str::from_utf8(line_bytes(rest, at)).ok();
        let renumbered = line.and_then(|line| {
            if line.starts_with("    1PSTEP") {
                let (from, to) = token_span(line, 1)?;
                let number: i64 = line[from..to].parse().ok()?;
                Some(set_field(line, 10..to, number + data_sets))
            } else if line.starts_with("  100C") {
                let set: i64 = field(line, 7..12)?;
                let increment: i64 = field(line, 58..63)?;
                let line = set_field(line, 7..12, set + increments);
                Some(set_field(&line, 58..63, increment + increments))
            } else {
                None
            }
        });
        match renumbered {
            Some(line) => {
                joined.extend_from_slice(line.as_bytes());
                joined.extend_from_slice(&raw[line_bytes(rest, at).len()..]);
            }
            None => joined.extend_from_slice(raw),
        }
    }
    if start == new.len() {
        joined.extend_from_slice(b" 9999\n");
    }
    joined
}

/// The number in the columns `columns` of a line.
fn field(line: &str, columns: std::ops::Range<usize>) -> Option<i64> {
    line.get(columns)?.trim().parse().ok()
}

/// The line with `value` right-aligned in the columns `columns`; unchanged if it does not
/// fit.
fn set_field(line: &str, columns: std::ops::Range<usize>, value: i64) -> String {
    let text = format!("{value:>width$}", width = columns.len());
    match line.get(columns.clone()) {
        Some(_) if text.len() == columns.len() => {
            format!("{}{text}{}", &line[..columns.start], &line[columns.end..])
        }
        _ => line.to_owned(),
    }
}

/// Where the whitespace separated token `index` of a line starts and ends.
fn token_span(line: &str, index: usize) -> Option<(usize, usize)> {
    let mut tokens = Vec::new();
    let mut start = None;
    for (i, c) in line
        .char_indices()
        .chain(std::iter::once((line.len(), ' ')))
    {
        match (c.is_whitespace(), start) {
            (true, Some(s)) => {
                tokens.push((s, i));
                start = None;
            }
            (false, None) => start = Some(i),
            _ => {}
        }
    }
    tokens.get(index).copied()
}

/// The history output of a run, as one from the start: the blocks of the steps up to
/// `after` of `old`, then `new`. CalculiX marks the steps (`S T E P  3`); without these
/// marks the blocks are told apart by their time, `time` being the end of step `after`.
pub fn join_dat(old: &[u8], new: &[u8], after: usize, time: f64) -> Vec<u8> {
    let starts = line_starts_bytes(old);
    let text = |at: usize| {
        String::from_utf8_lossy(line_bytes(old, at))
            .trim()
            .to_owned()
    };
    let marked = starts.iter().any(|&at| step_mark(&text(at)).is_some());
    let cut = starts.iter().position(|&at| {
        let line = text(at);
        if marked {
            return step_mark(&line).is_some_and(|step| step > after);
        }
        let words: Vec<&str> = line.split_whitespace().collect();
        // Headers in spaced capitals, such as the eigenvalue output, come from steps that
        // are not over time, which are never among the steps before a restart.
        if words.len() >= 4 && words.iter().all(|w| w.len() == 1) {
            return true;
        }
        let lower = line.to_ascii_lowercase();
        (lower.rfind("time"))
            .and_then(|at| line[at + 4..].trim().parse::<f64>().ok())
            .is_some_and(|t| t > time + 1e-6 * time.abs().max(1.0))
    });
    let cut = match cut {
        Some(mut index) => {
            // The empty and increment lines before a block belong to it.
            while index > 0 && {
                let line = text(starts[index - 1]);
                line.is_empty() || (!marked && line.starts_with("INCREMENT"))
            } {
                index -= 1;
            }
            starts[index]
        }
        None => old.len(),
    };
    let mut joined = old[..cut].to_vec();
    joined.extend_from_slice(new);
    joined
}

/// The step of a `S T E P       3` line of the `.dat` file.
fn step_mark(line: &str) -> Option<usize> {
    line.strip_prefix("S T E P")?.trim().parse().ok()
}

/// Removes the oldest state files of the jobs in the folders of `project_dir` until they
/// take no more than `limit` bytes together; returns a note per removed file. Only time
/// is lost: a job without its state runs in full.
pub fn limit_states(project_dir: &Path, limit: u64) -> Vec<String> {
    let mut files = Vec::new();
    let Ok(dirs) = std::fs::read_dir(project_dir) else {
        return Vec::new();
    };
    for dir in dirs.flatten() {
        let Ok(entries) = std::fs::read_dir(dir.path()) else {
            continue;
        };
        for entry in entries.flatten() {
            let name = entry.file_name().to_string_lossy().into_owned();
            if !(name.contains(".state-") && name.ends_with(".rout")) {
                continue;
            }
            let Ok(meta) = entry.metadata() else {
                continue;
            };
            let modified = meta.modified().unwrap_or(std::time::UNIX_EPOCH);
            files.push((modified, meta.len(), entry.path()));
        }
    }
    files.sort();
    let mut total: u64 = files.iter().map(|f| f.1).sum();
    let mut notes = Vec::new();
    for (_, size, path) in files {
        if total <= limit {
            break;
        }
        if std::fs::remove_file(&path).is_ok() {
            total -= size;
            notes.push(format!(
                "Restart file {} removed: the restart files of the project are over the limit \
                 in the settings",
                path.display()
            ));
        }
    }
    notes
}

/// A fingerprint of files a job reads besides its input file: their names, sizes and the
/// first and last megabyte of each, enough to tell results of another run apart.
pub fn files_fingerprint(files: &[PathBuf]) -> u64 {
    use std::io::{Read, Seek, SeekFrom};
    const PART: u64 = 1 << 20;
    let mut hash = Fnv::new();
    for path in files {
        hash.add_text(&path.file_name().unwrap_or_default().to_string_lossy());
        let Ok(mut file) = std::fs::File::open(path) else {
            continue;
        };
        let size = file.metadata().map_or(0, |m| m.len());
        hash.add(&size.to_le_bytes());
        let mut buffer = Vec::new();
        let _ = (&mut file).take(PART).read_to_end(&mut buffer);
        if size > 2 * PART && file.seek(SeekFrom::Start(size - PART)).is_ok() {
            let _ = file.take(PART).read_to_end(&mut buffer);
        }
        hash.add(&buffer);
    }
    hash.finish()
}

/// Whether CalculiX continues the state of this step in the next one: a step over time
/// that is no perturbation step.
fn continues_state(step: &str) -> bool {
    let header = step.lines().next().unwrap_or_default().to_ascii_uppercase();
    if header.contains("PERTURBATION") {
        return false;
    }
    let procedure = (step.lines().skip(1))
        .filter_map(keyword_name)
        .find(|k| TIME_PROCEDURES.contains(&k.as_str()) || OTHER_PROCEDURES.contains(&k.as_str()));
    procedure.is_some_and(|p| TIME_PROCEDURES.contains(&p.as_str()))
}

/// The step with `*Restart, Write` before its `*End step`.
fn with_restart_write(step: &str) -> String {
    let end = (line_starts(step).into_iter().rev())
        .find(|&at| keyword_name(line_at(step, at)).as_deref() == Some("END STEP"));
    match end {
        Some(at) => format!("{}*Restart, Write\n{}", &step[..at], &step[at..]),
        None => step.to_owned(),
    }
}

/// The fingerprints of the keyword blocks of a model definition, each with whether it
/// defines a set. The heading and comments do not count.
fn keyword_blocks(model: &str) -> Vec<(u64, bool)> {
    let mut blocks = Vec::new();
    let mut current: Option<(Fnv, String)> = None;
    let mut finish = |current: Option<(Fnv, String)>| {
        if let Some((hash, name)) = current
            && name != "HEADING"
        {
            blocks.push((hash.finish(), SET_KEYWORDS.contains(&name.as_str())));
        }
    };
    for line in model.lines() {
        if line.starts_with("**") {
            continue;
        }
        if let Some(name) = keyword_name(line) {
            finish(current.take());
            current = Some((Fnv::new(), name));
        }
        if let Some((hash, _)) = &mut current {
            hash.add(line.trim_end().as_bytes());
            hash.add(b"\n");
        }
    }
    finish(current);
    blocks
}

/// The name of the keyword a line starts, upper case without parameters; `None` for data
/// lines and comments.
fn keyword_name(line: &str) -> Option<String> {
    let line = line.trim_start();
    if !line.starts_with('*') || line.starts_with("**") {
        return None;
    }
    let name = line[1..].split(',').next().unwrap_or_default();
    Some(
        name.split_whitespace()
            .collect::<Vec<_>>()
            .join(" ")
            .to_ascii_uppercase(),
    )
}

fn is_subset(small: &[u64], large: &[u64]) -> bool {
    small.iter().all(|s| large.binary_search(s).is_ok())
}

fn fingerprint_all(values: &[u64]) -> u64 {
    let mut hash = Fnv::new();
    for value in values {
        hash.add(&value.to_le_bytes());
    }
    hash.finish()
}

fn line_starts(text: &str) -> Vec<usize> {
    line_starts_bytes(text.as_bytes())
}

fn line_at(text: &str, at: usize) -> &str {
    let rest = &text[at..];
    rest.split('\n').next().unwrap_or_default()
}

fn line_starts_bytes(bytes: &[u8]) -> Vec<usize> {
    let mut starts = vec![0];
    starts.extend((bytes.iter().enumerate()).filter_map(|(i, &b)| (b == b'\n').then_some(i + 1)));
    starts.retain(|&at| at < bytes.len());
    starts
}

fn line_bytes(bytes: &[u8], at: usize) -> &[u8] {
    let rest = &bytes[at..];
    let end = rest.iter().position(|&b| b == b'\n').unwrap_or(rest.len());
    rest[..end].strip_suffix(b"\r").unwrap_or(&rest[..end])
}

/// FNV-1a: fingerprints that stay the same from one version of nodalis to the next, as the
/// index of the states is kept on disk.
struct Fnv(u64);

impl Fnv {
    fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }

    fn add(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 ^= u64::from(byte);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }

    /// Adds the lines of a text that are not comments.
    fn add_text(&mut self, text: &str) {
        for line in text.lines().filter(|l| !l.starts_with("**")) {
            self.add(line.trim_end().as_bytes());
            self.add(b"\n");
        }
    }

    fn finish(&self) -> u64 {
        self.0
    }
}

#[cfg(test)]
#[path = "restart_tests.rs"]
mod tests;
