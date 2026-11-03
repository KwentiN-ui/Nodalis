//! How much memory the machine has free, for the automatic choice between the direct and
//! the iterative equation solver: Linux reads `MemAvailable` from `/proc/meminfo`, Windows
//! asks `GlobalMemoryStatusEx` for the available physical memory.

/// Bytes of physical memory available to a new allocation without swapping, when the
/// platform tells. `PLXS_MEMORY_LIMIT=<bytes>` overrides it, for measurements and tests.
pub fn available_memory() -> Option<u64> {
    if let Some(limit) = std::env::var("PLXS_MEMORY_LIMIT")
        .ok()
        .and_then(|v| v.trim().parse::<u64>().ok())
    {
        return Some(limit);
    }
    platform_available()
}

#[cfg(target_os = "linux")]
fn platform_available() -> Option<u64> {
    let text = std::fs::read_to_string("/proc/meminfo").ok()?;
    meminfo_available(&text)
}

#[cfg(windows)]
#[allow(unsafe_code)]
fn platform_available() -> Option<u64> {
    use windows_sys::Win32::System::SystemInformation::{GlobalMemoryStatusEx, MEMORYSTATUSEX};
    // SAFETY: the structure is plain data, initialized with its size as the call requires.
    unsafe {
        let mut status: MEMORYSTATUSEX = std::mem::zeroed();
        status.dwLength = std::mem::size_of::<MEMORYSTATUSEX>() as u32;
        (GlobalMemoryStatusEx(&mut status) != 0).then_some(status.ullAvailPhys)
    }
}

#[cfg(not(any(target_os = "linux", windows)))]
fn platform_available() -> Option<u64> {
    None
}

/// `MemAvailable` of a `/proc/meminfo` text, in bytes.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
fn meminfo_available(text: &str) -> Option<u64> {
    let line = text.lines().find(|l| l.starts_with("MemAvailable:"))?;
    let kib: u64 = line.split_whitespace().nth(1)?.parse().ok()?;
    Some(kib * 1024)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_mem_available() {
        let text = "MemTotal:       16303500 kB\nMemFree:        15000000 kB\n\
                    MemAvailable:   15400000 kB\n";
        assert_eq!(meminfo_available(text), Some(15_400_000 * 1024));
        assert_eq!(meminfo_available("MemTotal: 1 kB\n"), None);
    }

    #[cfg(any(target_os = "linux", windows))]
    #[test]
    fn the_platform_tells() {
        assert!(platform_available().is_some_and(|b| b > 0));
    }
}
