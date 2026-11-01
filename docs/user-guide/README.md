# Nodalis User Guide

The user guide is a reference for people who *use* Nodalis: where to find a function, what each
field of its dialog means and what Nodalis writes to the CalculiX input file. It is written in
English with [Typst](https://typst.app) and does not describe the code.

## Building

```sh
typst compile --root . --ignore-system-fonts docs/user-guide/main.typ nodalis-user-guide.pdf
```

Run it from the repository root: `--root .` lets the document read the version from the
workspace `Cargo.toml`, so the title page always shows the current version.
`--ignore-system-fonts` uses only the fonts built into Typst, so the PDF looks the same
everywhere. The release workflow builds the PDF for every tag `v*` and attaches it to the GitHub
release. The workflow `pages.yml` publishes the HTML guide on GitHub Pages
(<https://kwentin-ui.github.io/Nodalis/>) whenever the guide changes on `master`.

The same source also compiles to a single HTML page, which the Windows installer and the AppImage
ship for Help > User Guide:

```sh
scripts/build_user_guide.sh html   # target/user-guide/user-guide.html
scripts/build_user_guide.sh pdf    # target/user-guide/nodalis-<version>-user-guide.pdf
```

The script downloads the pinned Typst version (0.14, needed for HTML export) to `target/typst`.
HTML export is still experimental in Typst, so `template.typ` checks `target()` and emits plain
HTML elements there; `guide.css` styles the page. Check a new helper in both formats. Layout
functions such as `grid`, `place` or `v` are dropped in HTML; `#screenshot` and plain `grid`s of
images already work.

## Layout

- `main.typ` only includes the chapters. Add a line here for a new chapter file.
- `template.typ` holds the page layout and the helpers:
  - `#menu("File", "Open ...")` for a menu path,
  - `#ui("OK")` for a button, field or dialog name as it appears in the program,
  - `#key("Ctrl+S")` for keys,
  - `#fields([Field], [Meaning], ...)` for a table of dialog fields,
  - `#note[...]` and `#limitation[...]` for remarks,
  - `#screenshot("name.png", [Caption])` for a picture from `images/`. All screenshots are shown
    at the same scale, so dialogs and full windows match.
- `chapters/<area>/<topic>.typ` holds one topic per file (interface, geometry-mesh, model,
  analysis, results). Keep a feature in its own file, or in the file of its topic, so that threads
  working on different features do not edit the same lines.
- `images/` holds the screenshots.

## Documenting a feature

A pull request that adds or changes a user-visible feature also updates the guide:

1. Describe the feature in the chapter file of its topic, or in a new file under `chapters/`
   included from `main.typ`. Use the English labels of the program exactly, and say where to find
   the function, what its fields do and their defaults, and limitations a user will notice.
2. Add a screenshot if the feature has a dialog or a visible result.
3. Build the PDF and look at the pages you changed.

## Screenshots

Screenshots are taken from the real program on a virtual display (works in a Linux container
without a screen):

```sh
cargo build --release
docs/user-guide/screenshot.sh start testdata/platte_mit_loch.step
DISPLAY=:5 xdotool mousemove 199 13 click 1        # open the Mesh menu, and so on
docs/user-guide/screenshot.sh shot mesh-menu         # whole window -> images/mesh-menu.png
docs/user-guide/screenshot.sh shot dialog 421x358+300+90   # only a dialog
docs/user-guide/screenshot.sh stop
```

Graphics run on Mesa's software renderer (`mesa-vulkan-drivers`). The window is 1400 x 900;
dialogs open at the same place each time, so crop them by their pixel box. Typing into egui
fields with xdotool does not work reliably; for a model with materials, steps and loads, build a
`.ndl` project in code (a temporary `#[ignore]` test that calls `plx_io::project::save_project`)
and open it, or use the example files in `testdata/`. Open files from their own folder so that
the output pane shows short paths.
