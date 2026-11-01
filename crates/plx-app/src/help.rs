//! Help > User Guide: opens the offline HTML guide shipped with the program in the default
//! browser, or the online guide on GitHub Pages when no local copy exists (e.g. a binary
//! built with `cargo install`).

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::Duration;

/// File name of the HTML guide (scripts/build_user_guide.sh).
const FILE: &str = "user-guide.html";

/// Where the guide lies relative to the directory of the executable: Windows installer, Linux
/// AppImage (usr/bin next to usr/share), and target/user-guide for target/<profile>/nodalis.
const LOCATIONS: [&str; 3] = ["doc", "../share/doc/nodalis", "../user-guide"];

/// The installed HTML guide, if there is one.
pub fn local_guide() -> Option<PathBuf> {
    let exe = std::env::current_exe().ok()?;
    find_guide(exe.parent()?)
}

fn find_guide(exe_dir: &Path) -> Option<PathBuf> {
    LOCATIONS
        .iter()
        .map(|dir| exe_dir.join(dir).join(FILE))
        .find(|path| path.is_file())
}

/// The user guide on GitHub Pages (.github/workflows/pages.yml).
pub fn online_url() -> String {
    "https://kwentin-ui.github.io/Nodalis/".to_owned()
}

/// Opens the guide and returns a line for the output pane.
pub fn open_user_guide() -> String {
    let (target, what) = match local_guide() {
        Some(path) => match local_target(&path) {
            Ok(target) => (target, format!("Opening the user guide {}", path.display())),
            Err(error) => return format!("Error opening the user guide: {error}"),
        },
        None => (
            online_url(),
            "No offline user guide installed, opening the online user guide".to_owned(),
        ),
    };
    match open::that_detached(&target) {
        Ok(()) => what,
        Err(error) => format!("Error opening {target} in the browser: {error}"),
    }
}

/// On Linux the guide is served on the loopback interface: browsers installed as Snap or
/// Flatpak cannot read files below /tmp, where an AppImage is mounted. Elsewhere the file is
/// opened directly.
fn local_target(path: &Path) -> std::io::Result<String> {
    if cfg!(target_os = "linux") {
        Ok(format!("http://127.0.0.1:{}/", serve(path)?))
    } else {
        Ok(path.display().to_string())
    }
}

/// Port of the running guide server, started on first use.
static SERVER: Mutex<Option<u16>> = Mutex::new(None);

fn serve(path: &Path) -> std::io::Result<u16> {
    let mut server = SERVER.lock().unwrap_or_else(|poison| poison.into_inner());
    if let Some(port) = *server {
        return Ok(port);
    }
    let listener = TcpListener::bind(("127.0.0.1", 0))?;
    let port = listener.local_addr()?.port();
    let path = path.to_owned();
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            let path = path.clone();
            // Browsers open spare connections that never send a request; each gets its own
            // thread and a timeout, so they cannot block the next page load.
            std::thread::spawn(move || {
                if let Err(error) = answer(stream, &path) {
                    log::debug!("user guide server: {error}");
                }
            });
        }
    });
    *server = Some(port);
    Ok(port)
}

/// Answers one HTTP request: the guide for "/", 404 for anything else.
fn answer(stream: TcpStream, path: &Path) -> std::io::Result<()> {
    stream.set_read_timeout(Some(Duration::from_secs(10)))?;
    let mut reader = BufReader::new(stream);
    let mut request = String::new();
    reader.read_line(&mut request)?;
    // Skip the headers up to the blank line.
    let mut line = String::new();
    while reader.read_line(&mut line)? > 2 {
        line.clear();
    }
    let mut stream = reader.into_inner();
    let target = request.split_whitespace().nth(1).unwrap_or_default();
    let (status, kind, body) = if target == "/" || target == format!("/{FILE}") {
        ("200 OK", "text/html; charset=utf-8", std::fs::read(path)?)
    } else {
        ("404 Not Found", "text/plain", b"Not found".to_vec())
    };
    write!(
        stream,
        "HTTP/1.1 {status}\r\nContent-Type: {kind}\r\nContent-Length: {}\r\n\
         Cache-Control: no-cache\r\nConnection: close\r\n\r\n",
        body.len()
    )?;
    stream.write_all(&body)?;
    stream.flush()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read;

    #[test]
    fn finds_the_guide_of_each_install_layout() {
        let root = std::env::temp_dir().join(format!("nodalis-help-{}", std::process::id()));
        for (layout, exe_dir) in [
            ("doc", "."),
            ("usr/share/doc/nodalis", "usr/bin"),
            ("target/user-guide", "target/release"),
        ] {
            let _ = std::fs::remove_dir_all(&root);
            std::fs::create_dir_all(root.join(layout)).unwrap();
            std::fs::create_dir_all(root.join(exe_dir)).unwrap();
            assert_eq!(find_guide(&root.join(exe_dir)), None);
            std::fs::write(root.join(layout).join(FILE), "<html></html>").unwrap();
            let found = find_guide(&root.join(exe_dir)).expect(layout);
            assert!(found.is_file());
        }
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn online_url_points_to_github_pages() {
        assert_eq!(online_url(), "https://kwentin-ui.github.io/Nodalis/");
    }

    #[test]
    fn server_answers_with_the_guide() {
        let path = std::env::temp_dir().join(format!("nodalis-guide-{}.html", std::process::id()));
        std::fs::write(&path, "<h1>Guide</h1>").unwrap();
        let port = serve(&path).unwrap();
        let get = |target: &str| {
            let mut stream = TcpStream::connect(("127.0.0.1", port)).unwrap();
            write!(stream, "GET {target} HTTP/1.1\r\nHost: localhost\r\n\r\n").unwrap();
            let mut response = String::new();
            stream.read_to_string(&mut response).unwrap();
            response
        };
        let page = get("/");
        assert!(page.starts_with("HTTP/1.1 200 OK"), "{page}");
        assert!(page.ends_with("<h1>Guide</h1>"));
        assert!(get("/favicon.ico").starts_with("HTTP/1.1 404"));
        std::fs::remove_file(&path).unwrap();
    }
}
