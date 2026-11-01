//! A helper process that must end with the program that started it, such as the meshing
//! worker: it watches the program and ends itself at once.
#![allow(unsafe_code)]

/// Ends this process once the process `program` has ended, even if that crashed. On Unix
/// the program keeps this process's input open, which closes when it ends. On Windows a
/// thread reading the input would hold up everything else the process does with it, so
/// the process waits for the program's process instead.
pub fn end_with(program: Option<u32>) {
    #[cfg(windows)]
    {
        use windows_sys::Win32::System::Diagnostics::Debug::{
            SEM_FAILCRITICALERRORS, SEM_NOOPENFILEERRORBOX, SetErrorMode,
        };
        use windows_sys::Win32::System::Threading::{
            INFINITE, OpenProcess, PROCESS_SYNCHRONIZE, WaitForSingleObject,
        };
        // A hidden process must not wait for a click on an error box nobody sees.
        unsafe { SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOOPENFILEERRORBOX) };
        let Some(program) = program else {
            return;
        };
        let handle = unsafe { OpenProcess(PROCESS_SYNCHRONIZE, 0, program) };
        if handle.is_null() {
            end(2);
        }
        // The handle is a pointer, which threads may not share; its value may.
        let handle = handle as usize;
        std::thread::spawn(move || {
            unsafe { WaitForSingleObject(handle as _, INFINITE) };
            end(2);
        });
    }
    #[cfg(not(windows))]
    {
        let _ = program;
        std::thread::spawn(|| {
            let _ = std::io::copy(&mut std::io::stdin(), &mut std::io::sink());
            end(2);
        });
    }
}

/// Ends this process at once. On Windows it is terminated: exiting would unload libraries
/// such as Gmsh's, which may wait for threads of their own that are gone by then.
pub fn end(code: i32) -> ! {
    #[cfg(windows)]
    unsafe {
        use windows_sys::Win32::System::Threading::{GetCurrentProcess, TerminateProcess};
        TerminateProcess(GetCurrentProcess(), code as u32);
    }
    std::process::exit(code)
}
