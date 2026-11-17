//! How the C library's allocator hands memory back on Linux.
//!
//! glibc serves an allocation of 128 kB or more by a mapping of its own, unmapped when it is
//! freed, but raises that threshold up to 32 MB as such blocks are freed. Result fields
//! read from the results file (a component of 44,000 nodes is 176 kB) then come from the
//! heaps of the threads that read them, and the fields the cache drops leave holes there
//! that are not given back: with 512 MB of fields kept, the program held 340 MB more than
//! with the threshold fixed. Fixing it keeps the memory near what the program holds.

/// Keeps large blocks in mappings of their own; does nothing outside glibc.
pub fn give_back_large_blocks() {
    #[cfg(all(target_os = "linux", target_env = "gnu"))]
    #[allow(unsafe_code)]
    // SAFETY: mallopt only sets a parameter of the allocator; called before other threads run.
    unsafe {
        unsafe extern "C" {
            fn mallopt(param: std::ffi::c_int, value: std::ffi::c_int) -> std::ffi::c_int;
        }
        const M_MMAP_THRESHOLD: std::ffi::c_int = -3;
        mallopt(M_MMAP_THRESHOLD, 128 * 1024);
    }
}
