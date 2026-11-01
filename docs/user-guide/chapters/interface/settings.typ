#import "../../template.typ": *

== Settings <settings>

#menu("Tools", "Settings ...") opens the settings window. The pages are listed on the left.
#ui("Defaults") resets all pages, #ui("Apply") applies without closing. Settings are stored per
user (`~/.local/share/prepolix` on Linux, `%APPDATA%\prepolix\data` on Windows), by the
portable Windows version in the folder `Settings` next to `prepolix.exe` (@portable).

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

PrePoLix computes analyses either with its built-in solver or with CalculiX as a separate
program. The built-in solver runs inside PrePoLix: nothing has to be installed, its progress
and results stay in the program, and it stops at once when a run is killed. It is being built
up step by step; a step it cannot compute yet is reported when the analysis runs, with the
hint to switch to CalculiX for that model. CalculiX remains the default until the built-in
solver covers every analysis.

#fields(
  [Solver], [#ui("PrePoLix solver") (built in) or #ui("CalculiX"). The rows below that only
    concern CalculiX (executable, restart files, equation solvers) show for CalculiX.],
  [Executable], [Path to `ccx`. Empty means the CalculiX shipped with PrePoLix (shown greyed
    in the field), or `ccx` on the system path if PrePoLix was built without it. A bare name
    such as `ccx` is looked up on the system path.],
  [Threads], [Number of threads CalculiX uses (`OMP_NUM_THREADS`). #ui("Automatic") (the
    default) uses all processor cores; analyses or Campbell speeds solved at the same time
    share them equally. Clear it to set a fixed number per run.],
  [Work directory], [Where input and result files of a run are written. Empty means a `Temp`
    folder next to the PrePoLix program, or `<system temp>/prepolix` if that folder is not
    writable.],
  [Restart files], [#ui("Keep the state after each step") (default on): CalculiX writes the
    state after each step over time, so that solving again after a change starts after the
    last unchanged step (@restart). Needs disk space next to the results.],
  [Restart files per project], [Beyond this size (default 4096 MB) the oldest restart files
    of the project are removed; an analysis without them is solved from the beginning.],
  [Parallel analyses], [How many independent analyses #ui("Update Project") solves at the same
    time (default 2), each with the threads above (@update-project). With automatic threads
    they share the cores.],
  [Equation solvers], [The direct solvers found in the CalculiX executable, for example
    PARDISO. Steps with the solver #ui("Default") use Pardiso when it is available, otherwise
    PaStiX, otherwise CalculiX's own choice.],
  [Test solver], [Computes a cantilever under a tip load and under pressure with the chosen
    solver and compares the results with beam theory. For CalculiX it also starts the
    executable, checks which solvers are available and whether CalculiX knows every keyword
    PrePoLix writes. Each test shows OK or an error. While the test runs, a window over the
    application shows the running step and, for the keyword check, how far it has come; the
    CalculiX test can take a while, particularly on Windows.],
)

#screenshot("solver-builtin-test.png", [Result of #ui("Test solver") with the built-in solver])

#screenshot("calculix-test-running.png", [While #ui("Test solver") runs for CalculiX])

#screenshot("calculix-test.png", [Result of #ui("Test solver") for CalculiX])

*Keywords.* Older CalculiX versions lack keywords of newer features, for example
`*COMPLEX FREQUENCY` (complex frequency step) or `*CYCLIC SYMMETRY MODEL`. CalculiX does not stop
at an unknown keyword: it skips the card with a warning and computes without it. The test
therefore reads each keyword PrePoLix can write on its own and lists the unknown ones by the
feature that needs them, e.g. `Complex frequency step (*COMPLEX FREQUENCY)`. Every analysis
checks the keywords of its own input file in the same way (@model-check).

=== Gmsh

#fields(
  [Library], [Path to the Gmsh library (`libgmsh.so` or `gmsh-4.15.dll`). Empty means: next to
    the program, then the system path. A changed library is loaded after a restart.],
  [Test Gmsh], [Loads Gmsh and meshes a cube.],
)

=== Updates

#fields(
  [Check for updates at startup], [Asks GitHub for the latest release each time PrePoLix
    starts and shows a symbol next to the version number if it is newer (default on; not in the
    AppImage). Turn it off to work without any request to the internet;
    #menu("Help", "Check for Updates ...") still asks on demand.],
)

Below, the page says how this copy of PrePoLix is updated (@updates).
