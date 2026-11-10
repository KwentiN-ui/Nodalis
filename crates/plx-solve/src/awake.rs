//! Keeping the computer from going to standby while an analysis runs.
//!
//! A [`Guard`] is held for as long as a run is active; the operating system is asked to
//! stay awake while at least one guard exists and is told again that it may sleep when
//! the last one is released. The display is not touched: the screen may still switch off
//! and lock. On Windows this is `SetThreadExecutionState(ES_SYSTEM_REQUIRED)`, on Linux
//! an `idle` inhibitor lock of systemd-logind (like `systemd-inhibit`), both through the
//! crate `keepawake`.
//!
//! The request lives on one dedicated thread: on Windows the execution state belongs to
//! the thread that sets it, and it must be reset by the same thread. Failing to get the
//! request (no D-Bus, no logind) is not an error for the run, the computer just may sleep.

use std::sync::mpsc::{Sender, channel};
use std::sync::{Mutex, PoisonError};
use std::thread::JoinHandle;

/// The holder thread and the number of guards it serves.
struct Holder {
    users: usize,
    stop: Sender<()>,
    thread: JoinHandle<()>,
}

static HOLDER: Mutex<Option<Holder>> = Mutex::new(None);

/// Keeps the system awake until dropped.
#[derive(Debug)]
pub struct Guard(());

impl Guard {
    /// Asks the system to stay awake, besides the guards that already do.
    pub fn new() -> Self {
        let mut holder = HOLDER.lock().unwrap_or_else(PoisonError::into_inner);
        match holder.as_mut() {
            Some(holder) => holder.users += 1,
            None => *holder = Some(start()),
        }
        Guard(())
    }
}

impl Default for Guard {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for Guard {
    fn drop(&mut self) {
        let mut slot = HOLDER.lock().unwrap_or_else(PoisonError::into_inner);
        let Some(holder) = slot.as_mut() else {
            return;
        };
        holder.users -= 1;
        if holder.users == 0
            && let Some(Holder { stop, thread, .. }) = slot.take()
        {
            let _ = stop.send(());
            // The request is released by the holder thread; do not block on it.
            drop(thread);
        }
    }
}

/// Number of guards alive, for tests.
#[cfg(test)]
fn users() -> usize {
    HOLDER
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .as_ref()
        .map_or(0, |holder| holder.users)
}

fn start() -> Holder {
    let (stop, stopped) = channel::<()>();
    let thread = std::thread::Builder::new()
        .name("keep-awake".into())
        .spawn(move || {
            let request = keepawake::Builder::default()
                .idle(true)
                .reason("Running an analysis")
                .app_name("Nodalis")
                .app_reverse_domain("io.github.kwentin-ui.nodalis")
                .create();
            // Held until the last guard is released (or the process ends).
            let _request = request.ok();
            let _ = stopped.recv();
        })
        .expect("the keep-awake thread starts");
    Holder {
        users: 1,
        stop,
        thread,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counts_guards() {
        // Other tests start runs, which hold guards too: only compare against what this
        // test holds itself.
        let first = Guard::new();
        let second = Guard::new();
        assert!(users() >= 2);
        drop(first);
        assert!(users() >= 1);
        drop(second);
    }
}
