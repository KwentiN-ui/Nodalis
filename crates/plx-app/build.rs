//! Embeds the program icon into nodalis.exe for Windows (GNU toolchain), so Explorer, the
//! task bar and shortcuts show it. Uses MinGW's windres; other targets need nothing here.

use std::{env, path::PathBuf, process::Command};

fn main() {
    let icon =
        PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("assets/icon/nodalis.ico");
    println!("cargo:rerun-if-changed={}", icon.display());
    println!("cargo:rerun-if-env-changed=WINDRES");
    if env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("windows")
        || env::var("CARGO_CFG_TARGET_ENV").as_deref() != Ok("gnu")
    {
        return;
    }
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    let rc = out.join("nodalis.rc");
    let obj = out.join("nodalis-icon.o");
    // windres reads the path inside the .rc with C escapes, so forward slashes are safest.
    let icon_path = icon.display().to_string().replace('\\', "/");
    std::fs::write(&rc, format!("1 ICON \"{icon_path}\"\n")).unwrap();
    let windres = env::var("WINDRES").unwrap_or_else(|_| {
        let arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap();
        let cross = format!("{arch}-w64-mingw32-windres");
        if Command::new(&cross).arg("--version").output().is_ok() {
            cross
        } else {
            "windres".into()
        }
    });
    match Command::new(&windres)
        .args(["-O", "coff", "-i"])
        .arg(&rc)
        .arg("-o")
        .arg(&obj)
        .status()
    {
        Ok(status) if status.success() => println!("cargo:rustc-link-arg-bins={}", obj.display()),
        _ => println!("cargo:warning={windres} failed, nodalis.exe is built without an icon"),
    }
}
