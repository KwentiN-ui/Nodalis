#import "../template.typ": *

= Introduction

Nodalis is a finite element program with its own solver: it prepares the model, solves it and
evaluates the results. Its input and result formats are those of
#link("http://www.calculix.de/")[CalculiX], so models can be exchanged with it. Nodalis runs on
Linux and Windows.

With Nodalis you

- import CAD geometry (STEP, IGES, BREP) and mesh it with Gmsh, or open an existing CalculiX mesh
  (`.inp`),
- define the finite element model: materials, sections, constraints, contacts, steps, boundary
  conditions and loads,
- keep several analyses in one project, as in Ansys Workbench, and solve only what changed,
- solve the analyses with the built-in solver and watch its progress,
- view and evaluate the results (`.frd`, `.dat`): contour plots, deformed shapes, animations,
  derived fields, history plots, paths, planes and hot spot stresses.

This guide is a reference. Each chapter describes one part of the program: where to find a
function, what every field of its dialog means, and what Nodalis writes to the input file of
the solver (CalculiX format). You do not need to read it from front to back.

== How Nodalis thinks about a model

You do not need to define node sets, element sets or surfaces. Instead you pick faces, edges,
parts or nodes in the 3D view, and the model stores your _intent_: "a pressure of 2 MPa on these
faces", "these two parts are tied". Only when the input file is written does Nodalis turn this
intent into CalculiX keywords: it creates the required node sets, element sets and surfaces with
generated names and splits a distributed load into what CalculiX needs. Named sets of your own
are still possible (@mesh-sets).

This has two consequences you will notice:

- When you remesh a part, loads and boundary conditions that were picked on the CAD geometry
  stay where they are, because they refer to faces, edges and points of the geometry, not to
  element numbers. This includes beam and truss lines: a click on a line picks its CAD edge, a
  click near its end the CAD point there, so node ties and supports at line ends survive a
  remesh as well.
- The `.inp` file contains sets you never named. You can inspect it at any time by exporting it
  (#menu("File", "Export", "CalculiX Input File ...")).

The project, including geometry, mesh and model, is saved as a Nodalis project file (`.ndl`).

== Requirements

- *No external solver* is needed: the solver is part of Nodalis, so analyses run right after
  installation. #menu("Edit", "Settings ...", "Solver") sets its threads and work directory;
  #ui("Test solver") checks it (@settings).
- *Gmsh* is needed to import and mesh CAD geometry. The Windows installer and the Linux AppImage
  include it. Without
  Gmsh Nodalis still starts; only geometry import and meshing are unavailable. Its library path
  can be set under #menu("Edit", "Settings ...", "Gmsh").
- On Linux, Nodalis needs a Vulkan or OpenGL driver and `libxkbcommon`, which most desktop
  installations provide. A Gmsh library installed separately additionally needs `libGLU` and
  `libXft`; the AppImage brings them along.

== Installation

*Windows.* Run `nodalis-<version>-setup.exe` from the release page. It installs Nodalis with
Gmsh to `C:\Program Files\nodalis`, creates a start menu entry, optionally a desktop icon, and
can associate `.ndl` project files with Nodalis. Uninstall it under _Apps_ in the Windows
settings. Because programs may not write to `C:\Program Files`, the work directory of the analyses
then defaults to `%TEMP%\nodalis`.

*Linux.* Download `nodalis-<version>-x86_64.AppImage` from the release page, make it executable
(`chmod +x nodalis-*-x86_64.AppImage`) and start it. It includes Gmsh and runs on x86_64
distributions with glibc 2.35 or newer (Ubuntu 22.04, Debian 12, Fedora 36 and later). A tool
such as Gear Lever or AppImageLauncher adds a menu entry, associates `.ndl` files and updates
the AppImage when a new release is published. The work
directory of the analyses defaults to `/tmp/nodalis`. Alternatively build Nodalis from source with a
current stable Rust toolchain (`cargo run --release`), see the README of the repository.

=== Portable version for Windows <portable>

Where installing is not possible, for example on a company computer without administrator
rights, download `nodalis-<version>-windows-portable.zip` from the release page instead. Unpack
it to any folder you can write to, for example in your documents or on a USB stick, and start `nodalis.exe` in the folder `Nodalis-<version>`. The
folder contains the same program, Gmsh and this guide as the installer, but Nodalis
does not register itself with Windows: there is no start menu entry and no file association, and
nothing is written to the registry. Settings and the material library are kept in the folder
`Settings` next to `nodalis.exe`, the work directory of the analyses defaults to the folder `Temp`
there, and the settings window shows #ui("Portable mode") at its bottom. The file `portable.txt`
switches this on; without it the settings go to `%APPDATA%\nodalis\data` as with the
installer. To update, unpack the new version (the update window links to its ZIP file, see
@updates) and copy the `Settings` folder over; to remove Nodalis, delete the folder.

#screenshot("portable.png", [The portable version under Windows: the work directory in the
unpacked folder, the note at the bottom])

== Updates <updates>

Nodalis asks GitHub for the latest release when it starts. If a newer version exists, a green
symbol with a download arrow appears at the right of the app bar, next to the version number.
Nothing else happens: ignore it if you do not want to update. A click on the symbol, or
#menu("Help", "Check for Updates ..."), opens the update window with the release notes
(_What's Changed_, one line per change with a link to it on GitHub):

#screenshot("update-available.png", [Update symbol next to the version and the update window of a
  Nodalis built from source; with the Windows installer it also offers #ui("Update Now")])

- #ui("Update Now") (Windows installer only) downloads the installer of the new version, checks
  its size and checksum and starts it. Windows asks for administrator rights, Nodalis closes,
  the installer only shows its progress and then starts Nodalis again; the shortcuts and the
  `.ndl` association stay as you installed them. Save your project before: the project that was
  open is reopened when it was saved as a `.ndl` file.
- #ui("Download ZIP") (portable version only, @portable) downloads the new portable ZIP file in
  the web browser. Unpack it yourself and copy the `Settings` folder over.
- #ui("Open Release Page") opens the release on GitHub. A Nodalis built from source offers only
  this.
- #ui("Close") closes the window; the symbol stays until Nodalis is closed.

The Linux AppImage does not check at startup, since an AppImage manager such as Gear Lever or
AppImageLauncher updates it; #menu("Help", "Check for Updates ...") still shows what is new. The
check at startup runs in the background and stays silent when there is nothing new or no
internet connection. It can be turned off under #menu("Edit", "Settings ...", "Updates")
(@settings). A proxy is taken from the `HTTPS_PROXY` variable or, on Windows, from a proxy server
entered in the Internet Options (automatic configuration scripts are not read). The connection is
checked with the certificates of the system, so a company proxy that inspects HTTPS works too.

== Logs and crash reports <crash-logs>

Nodalis writes its warnings and errors to `nodalis.log` in a `logs` folder; the log of the run
before is kept as `nodalis.previous.log`. If Nodalis crashes, it also writes a
`crash-<date>_<time>.log` there with the error message, the place in the code, the backtrace and
the version; under Windows a message box names the file before the program closes. Attach this
file, together with `nodalis.previous.log`, when you report a crash. The newest 20 reports are
kept. #menu("Help", "Open Log Folder") opens the folder:

#table(
  columns: 2,
  [Windows], [`%APPDATA%\nodalis\data\logs`],
  [Linux], [`~/.local/share/nodalis/logs`],
  [Portable version], [`Settings\logs` in the unpacked folder (@portable)],
)

Errors of the graphics driver are logged as well and no longer end the program. If the graphics
device is lost, for example after a driver reset or when the GPU was switched off while the
window was in the background, a note asks to save the project and restart Nodalis. Set the
environment variable `RUST_LOG=info` (or `debug`) for more detailed logs.

== Starting Nodalis

Start Nodalis from the start menu, by double-clicking a `.ndl` file (Windows, if associated),
by double-clicking `nodalis.exe` in the portable folder (@portable), or from a terminal. Files
given on the command line are opened in order, so a model and its results can be opened
together:

```sh
nodalis bracket.ndl bracket.frd
```

Several CAD files given together are combined into one geometry. Without command line arguments,
Nodalis reopens the project that was open when it was last closed, provided it was saved as a
`.ndl` file and still exists.

== This guide offline <offline-guide>

The Windows installer, the portable ZIP and the Linux AppImage contain this guide as a web page.
#menu("Help", "User Guide") opens it in the default web browser; no internet connection is
needed. On Linux Nodalis hands the page to the browser through a small web server on your
computer (address `http://127.0.0.1:<port>/`), because browsers installed as Snap or Flatpak
cannot read the files of a mounted AppImage. A Nodalis built from source without the guide opens
the online guide at #link("https://kwentin-ui.github.io/Nodalis/") instead. The guide is also
available as PDF on the release page on GitHub.
