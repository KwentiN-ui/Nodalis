// Layout of the Nodalis user guide. Only fonts embedded in Typst are used, so the PDF
// looks the same on every machine (`typst compile --ignore-system-fonts`).
//
// The same source also compiles to the offline HTML guide shipped with the program
// (`--features html --format html`, Typst 0.14 or newer). Helpers check `target()` and emit
// plain HTML elements there; the style sheet below replaces the page layout.

#let accent = rgb("#2b5797")

// True while compiling to HTML. Needs context. `target` only exists with `--features html`.
#let is-html() = "target" in dictionary(std) and std.target() == "html"

#let html-css = read("guide.css")

// Name of a dialog, button or field as it appears in the program.
// A name split over two source lines stays on one line. `strong` also gives <strong> in HTML.
#let ui(name) = strong(if type(name) == str { name.replace(regex("\s+"), " ") } else { name })

// Menu path, e.g. #menu("File", "Open...").
#let menu(..items) = {
  let parts = items.pos().map(it => ui(it))
  parts.join([ #sym.arrow.r ])
}

// Keyboard key or combination, e.g. #key("Ctrl+O").
#let key(name) = context if is-html() {
  html.elem("kbd", name)
} else {
  box(
    inset: (x: 3pt, y: 0pt),
    outset: (y: 2pt),
    stroke: 0.5pt + luma(150),
    radius: 2pt,
    fill: luma(245),
    text(size: 0.85em, font: "DejaVu Sans Mono", name),
  )
}

// Short boxed remark: a hint, a limitation or a CalculiX detail.
#let note(title: "Note", body) = context if is-html() {
  html.elem("div", attrs: (class: "note"), [#strong(title) #body])
} else {
  block(
    width: 100%,
    inset: 8pt,
    radius: 3pt,
    fill: rgb("#eef3fa"),
    stroke: (left: 2pt + accent),
    [#text(weight: "bold", fill: accent, title) #h(0.4em) #body],
  )
}

#let limitation(body) = note(title: "Limitation", body)

// Table of dialog fields: pairs of (field, description).
#let fields(..rows) = table(
  columns: (30%, 1fr),
  stroke: (x, y) => if y == 0 { (bottom: 0.7pt) } else { (bottom: 0.3pt + luma(200)) },
  inset: (x: 5pt, y: 4pt),
  table.header([*Field*], [*Meaning*]),
  ..rows.pos().flatten(),
)


// HTML (offline guide): a title header, the contents as a side bar and the chapters next to it,
// styled by guide.css. Equations become inline SVG pictures and grids of pictures a row of
// images; HTML export would otherwise drop both.
#let html-guide(version, body) = {
  show math.equation: it => html.frame(it)
  show grid: it => html.elem("div", attrs: (class: "grid"), it.children.map(c => c.body).join())
  html.elem("style", html-css)
  html.elem("header", {
    image("../../assets/icon/nodalis.svg", alt: "")
    html.elem("div", {
      html.elem("h1", [Nodalis User Guide])
      html.elem("p", [A pre- and post-processor for CalculiX, Version
        #version.])
    })
  })
  html.elem("div", attrs: (class: "layout"), {
    html.elem("aside", outline(depth: 2))
    html.elem("main", body)
  })
}

// PDF: title page, contents and A4 pages with a running header.
#let paged-guide(version, body) = {
  show heading.where(level: 1): it => {
    pagebreak(weak: true)
    v(2em)
    text(size: 22pt, fill: accent, it)
    v(1em)
  }
  show heading.where(level: 2): set text(size: 14pt, fill: accent)
  show heading.where(level: 3): set text(size: 11.5pt)

  // Title page.
  page(margin: 3cm)[
    #v(12%)
    #image("../../assets/icon/nodalis.svg", width: 4.5cm)
    #v(1.5em)
    #text(size: 36pt, weight: "bold", fill: accent)[Nodalis]
    #v(0.2em)
    #text(size: 18pt)[User Guide]
    #v(1em)
    #text(size: 12pt)[A pre- and post-processor for CalculiX]
    #v(1fr)
    #text(size: 11pt)[Version #version]
  ]

  set page(
    paper: "a4",
    margin: (x: 2.2cm, y: 2.5cm),
    numbering: "1",
    header: context {
      if counter(page).get().first() > 2 {
        set text(size: 9pt, fill: luma(110))
        [Nodalis User Guide #h(1fr) Version #version]
      }
    },
  )
  counter(page).update(1)

  outline(depth: 2, indent: auto)
  body
}

#let guide(version: "dev", body) = {
  set document(title: "Nodalis User Guide", author: "Nodalis contributors")
  set text(font: "Libertinus Serif", size: 10.5pt, lang: "en")
  set par(justify: true, leading: 0.6em)
  set heading(numbering: "1.1")
  show raw: set text(font: "DejaVu Sans Mono", size: 0.9em)
  show link: set text(fill: accent)

  context if is-html() { html-guide(version, body) } else { paged-guide(version, body) }
}

// A screenshot from images/, shown at the same scale as all others: 0.55 pt per pixel, at most
// the text width. Full-window shots therefore fill the line and dialogs stay readable. In HTML
// the picture keeps its pixel size, shrunk to the column width by the style sheet.
#let screenshot(name, caption) = figure(
  context if is-html() {
    image("images/" + name, alt: "")
  } else {
    layout(size => {
      let natural = measure(image("images/" + name)).width
      image("images/" + name, width: calc.min(natural * 0.55, size.width))
    })
  },
  caption: caption,
)
