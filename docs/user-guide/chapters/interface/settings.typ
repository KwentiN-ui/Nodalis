#import "../../template.typ": *

== Settings <settings>

#menu("Edit", "Settings ...") opens the settings window. The pages are listed on the left.
#ui("Defaults") resets all pages, #ui("Apply") applies without closing. Settings are stored per
user (`~/.local/share/nodalis` on Linux, `%APPDATA%\nodalis\data` on Windows), by the
portable Windows version in the folder `Settings` next to `nodalis.exe` (@portable).

#screenshot("settings.png", [Settings, Solver page])

=== Graphics

#fields(
  [Color theme], [#ui("Light") is the light look, #ui("Dark") shows dark panels, a dark
    3D background and light text over the model. #ui("System") (default) follows the light or
    dark mode of the operating system and switches along with it. Icons, tree symbols, plots and
    annotations change with the theme; exported SVG plots and the images of the user guide stay
    light.],
  [Global axes at the origin], [Draws a coordinate triad at the global origin (default on).],
  [Coordinate triad in the corner], [The clickable triad in the lower right (default on).],
  [Scale bar], [Shows a scale bar (default on).],
  [Shell thickness], [Draws shell and membrane elements of the FE model and the results as
    solids of their section's thickness, placed by its offset (default on), see @shell-section.],
  [Beam profiles], [Draws beams with their profile and trusses as round bars of their area in
    the FE model and the results (default on), see @beam-profiles.],
)

#screenshot("dark-theme.png", [The dark theme with the results of a cantilever])

=== Post-processing

#fields(
  [Show label at maximum / minimum], [Labels at the node with the largest / smallest value
    (default: maximum on, minimum off).],
  [Show status block], [The info block with file, step, increment and scale factor.],
  [Show undeformed outline], [Draws the undeformed model as a wireframe for newly opened
    results.],
  [Color levels], [Number of colour bands of the legend for newly opened results (2 to 24,
    default 9).],
  [FFmpeg executable], [FFmpeg for exporting animations as MP4 or MKV (@animation-export);
    empty for `ffmpeg` on the `PATH`.],
)

=== Solver

Nodalis computes every analysis with its own solver. It runs inside Nodalis: nothing has to be
installed, its progress and results stay in the program, and it stops at once when a run is
killed. A step it cannot compute yet is reported when the analysis runs (@model-check).

#fields(
  [Threads], [Number of threads the solver uses. #ui("Automatic") (the default) uses all
    processor cores. The solver computes every analysis in one shared pool of these
    threads: a single large analysis gets all of them, and analyses or Campbell speeds solved
    at the same time take them as they need them.],
  [Work directory], [Where input and result files of a run are written. Empty means a `Temp`
    folder next to the Nodalis program, or `<system temp>/nodalis` if that folder is not
    writable.],
  [Parallel analyses], [How many independent analyses #ui("Update Project") solves at the same
    time (default 2, @update-project). It limits the memory they need together; the threads
    are shared as described above.],
  [Test solver], [Computes a cantilever under a tip load and under pressure, and the tip load
    once more with the iterative equation solver (@equation-solver), and compares the results
    with beam theory. Each test shows OK or an error; while the test runs, a window over the
    application shows the running step.],
)

#screenshot("solver-builtin-test.png", [Result of #ui("Test solver")])

=== Gmsh

#fields(
  [Library], [Path to the Gmsh library (`libgmsh.so` or `gmsh-4.15.dll`). Empty means: next to
    the program, then the system path. A changed library is loaded after a restart.],
  [Test Gmsh], [Loads Gmsh and meshes a cube.],
)

=== Updates

#fields(
  [Check for updates at startup], [Asks GitHub for the latest release each time Nodalis
    starts and shows a symbol next to the version number if it is newer (default on; not in the
    AppImage). Turn it off to work without any request to the internet;
    #menu("Help", "Check for Updates ...") still asks on demand.],
)

Below, the page says how this copy of Nodalis is updated (@updates).
