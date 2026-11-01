//! Where the time of a solve goes: wall-clock time per phase, summed over the steps of a
//! session. `plxs --profile` (or `PLXS_PROFILE=1`) prints the table at the end of the job;
//! PrePoLix can show it from [`crate::ProcedureContext::profile`] in phase 2.

use std::time::{Duration, Instant};

/// The phases a procedure measures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// Element matrices and internal forces into the global matrix and vectors.
    Assembly,
    /// `Tᵀ K T` and `Tᵀ r`: constraints eliminated from the system.
    Reduction,
    /// Symbolic analysis of the direct solver (ordering, elimination tree, supernodes).
    Analysis,
    /// Numeric factorization.
    Factorization,
    /// Forward and back substitution.
    Solve,
    /// Stresses, strains, reaction forces and the result fields of an increment.
    Results,
}

impl Phase {
    pub const ALL: [Phase; 6] = [
        Phase::Assembly,
        Phase::Reduction,
        Phase::Analysis,
        Phase::Factorization,
        Phase::Solve,
        Phase::Results,
    ];

    fn index(self) -> usize {
        Self::ALL.iter().position(|&p| p == self).unwrap_or(0)
    }

    pub fn name(self) -> &'static str {
        match self {
            Phase::Assembly => "assembly",
            Phase::Reduction => "constraint reduction",
            Phase::Analysis => "symbolic analysis",
            Phase::Factorization => "factorization",
            Phase::Solve => "solve",
            Phase::Results => "results",
        }
    }
}

/// Time and count per phase, plus how often a symbolic analysis was reused.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Profile {
    time: [Duration; 6],
    count: [u32; 6],
    /// Symbolic analyses skipped because the pattern was the one analyzed before.
    pub analyses_reused: u32,
}

impl Profile {
    /// Adds the time since `started` to a phase.
    pub fn add(&mut self, phase: Phase, started: Instant) {
        let i = phase.index();
        self.time[i] += started.elapsed();
        self.count[i] += 1;
    }

    pub fn time(&self, phase: Phase) -> Duration {
        self.time[phase.index()]
    }

    pub fn count(&self, phase: Phase) -> u32 {
        self.count[phase.index()]
    }

    /// Adds another profile (of a step) to this one.
    pub fn merge(&mut self, other: &Profile) {
        for i in 0..self.time.len() {
            self.time[i] += other.time[i];
            self.count[i] += other.count[i];
        }
        self.analyses_reused += other.analyses_reused;
    }

    /// The table `plxs --profile` prints.
    pub fn report(&self, total: Duration) -> Vec<String> {
        let mut lines = vec![" profile (wall clock):".to_string()];
        let mut measured = Duration::ZERO;
        for phase in Phase::ALL {
            let t = self.time(phase);
            measured += t;
            let mut line = format!(
                "   {:<22}{:>9.3} s {:>6}x",
                phase.name(),
                t.as_secs_f64(),
                self.count(phase)
            );
            if phase == Phase::Analysis && self.analyses_reused > 0 {
                line.push_str(&format!(" ({} reused)", self.analyses_reused));
            }
            lines.push(line);
        }
        lines.push(format!(
            "   {:<22}{:>9.3} s",
            "other",
            total.saturating_sub(measured).as_secs_f64()
        ));
        lines.push(format!("   {:<22}{:>9.3} s", "total", total.as_secs_f64()));
        lines
    }
}
