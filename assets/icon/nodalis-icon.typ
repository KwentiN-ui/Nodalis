// Nodalis program icon: the letter N as a finite element mesh, coloured with a contour
// result like an FE contour plot. Nodalis = lat. "knot-like": FE nodes and the
// node graph of the project window.
//
// The N is one conforming structured mesh of three patches (left bar, diagonal, right
// bar), each quad split into two linear triangles. The result is a smooth field given at
// the nodes; every triangle interpolates it linearly and is cut into contour bands.
//
// Inputs (typst --input key=value):
//   variant color | mono | light     color: on the flat dark tile; mono: dark grey faces
//                                    with light lines, light: light grey faces with dark
//                                    lines; both without tile, on a transparent background
//   detail  full | small             small: no mesh lines or nodes, for 16-32 px
//
// The page is 256 pt square. scripts/make_icon.py exports all files the program uses.

#let variant = sys.inputs.at("variant", default: "color")
#let small = sys.inputs.at("detail", default: "full") == "small"
#let light-mono = variant == "light"
#let mono = variant == "mono" or light-mono

#let S = 256.0
#set page(width: S * 1pt, height: S * 1pt, margin: 0pt, fill: none)

// ---------------------------------------------------------------- colours

#let tile-fill = rgb("#24272e")
#let tile-edge = rgb("#3a3e48")
#let ink = rgb("#14161a")
#let node-fill = rgb("#eef0f4")
#let mono-face = rgb("#3a3f48")
#let mono-line = rgb("#c9ced6")
#let light-face = rgb("#dfe3e8")
#let light-line = rgb("#3a4350")
// Contour colours like an FE result legend, blue (low) to red (high).
#let legend = (
  rgb("#2a3fe0"), rgb("#1e7ff0"), rgb("#18b8e8"), rgb("#1fd6b0"), rgb("#4ad94a"),
  rgb("#b4e02c"), rgb("#f5dc1e"), rgb("#ffa21c"), rgb("#f8601a"), rgb("#e0201e"),
)
#let bands = legend.len()

// ---------------------------------------------------------------- geometry

// On the tile the N keeps a margin; without tile it fills more of the page.
#let fit = if mono { 1.18 } else { 1.0 }
#let to-page(p) = ((S / 2 + (p.at(0) - 128) * fit) * 1pt, (S / 2 + (p.at(1) - 128) * fit) * 1pt)

#let lerp(a, b, t) = (a.at(0) + (b.at(0) - a.at(0)) * t, a.at(1) + (b.at(1) - a.at(1)) * t)
// Corner order: (u, v) = (0, 0), (1, 0), (1, 1), (0, 1).
#let bilinear(c, u, v) = lerp(lerp(c.at(0), c.at(1), u), lerp(c.at(3), c.at(2), u), v)

// Patches as bilinear quads with their cell counts (u, v); cells are 16 pt along the bars.
#let patches = (
  (((48, 208), (96, 208), (96, 48), (48, 48)), 3, 10),
  (((160, 208), (208, 208), (208, 48), (160, 48)), 3, 10),
  (((96, 48), (160, 128), (160, 208), (96, 128)), 4, 5),
)

// The "result": rising from the lower left to the upper right, with the stress
// concentrations a loaded N would have in its two re-entrant corners.
#let field(p) = {
  let (x, y) = p
  let base = 0.58 * (x - 48) / 160 + 0.36 * (208 - y) / 160
  let bump(cx, cy, a) = a * calc.exp(-((x - cx) * (x - cx) + (y - cy) * (y - cy)) / (2 * 22.0 * 22.0))
  calc.clamp(base + bump(96, 128, 0.16) + bump(160, 128, 0.10), 0.0, 0.999)
}

#let triangles = {
  let out = ()
  for (c, nu, nv) in patches {
    for i in range(nu) {
      for j in range(nv) {
        let q = ((i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)).map(((a, b)) => bilinear(c, a / nu, b / nv))
        let split = if calc.rem(i + j, 2) == 0 { ((0, 1, 2), (0, 2, 3)) } else { ((0, 1, 3), (1, 2, 3)) }
        for t in split { out.push(t.map(k => q.at(k))) }
      }
    }
  }
  out
}

// ---------------------------------------------------------------- contour bands

// Part of a polygon (points with values) where the value is >= lo (keep-above) or < hi.
#let clip(poly, level, keep-above) = {
  let out = ()
  let n = poly.len()
  for k in range(n) {
    let (p, a) = poly.at(k)
    let (q, b) = poly.at(calc.rem(k + 1, n))
    let in-p = if keep-above { a >= level } else { a < level }
    let in-q = if keep-above { b >= level } else { b < level }
    if in-p { out.push((p, a)) }
    if in-p != in-q {
      let t = (level - a) / (b - a)
      out.push((lerp(p, q, t), level))
    }
  }
  out
}

#let shape(pts, fill, stroke) = place(polygon(fill: fill, stroke: stroke, ..pts.map(to-page)))

#let draw-triangle(tri) = {
  if mono {
    shape(tri, if light-mono { light-face } else { mono-face }, 0.6pt + if light-mono { light-face } else { mono-face })
    return
  }
  let poly = tri.map(p => (p, field(p)))
  let vals = poly.map(e => e.at(1))
  let lo = int(calc.min(..vals) * bands)
  let hi = int(calc.max(..vals) * bands)
  for b in range(lo, hi + 1) {
    let part = clip(clip(poly, b / bands, true), (b + 1) / bands, false)
    if part.len() >= 3 {
      let col = legend.at(calc.min(b, bands - 1))
      // A hairline in the fill colour hides the anti-aliasing seams between pieces.
      shape(part.map(e => e.at(0)), col, 0.6pt + col)
    }
  }
}

// ---------------------------------------------------------------- draw

#if not mono {
  place(dx: 1.5pt, dy: 1.5pt, rect(width: (S - 3) * 1pt, height: (S - 3) * 1pt, radius: 56pt,
    fill: tile-fill, stroke: 3pt + tile-edge))
}

#for tri in triangles { draw-triangle(tri) }

#if not small {
  let line-col = if light-mono { light-line } else if mono { mono-line } else { ink.transparentize(25%) }
  for tri in triangles { shape(tri, none, (paint: line-col, thickness: 0.8pt, join: "round")) }
}

// Outline of the N, so it holds together on any background.
#{
  let outline = ((48, 208), (96, 208), (96, 128), (160, 208), (208, 208), (208, 48), (160, 48),
    (160, 128), (96, 48), (48, 48))
  let col = if light-mono { light-line } else if mono { mono-line } else { ink }
  shape(outline, none, (paint: col, thickness: if small { 4pt } else { 2.4pt }, join: "round"))
}

#if not small {
  let nodes = triangles.flatten().chunks(2).map(p => (calc.round(p.at(0), digits: 3), calc.round(p.at(1), digits: 3))).dedup()
  let dot-col = if light-mono { light-line } else if mono { mono-line } else { node-fill }
  for p in nodes {
    let (x, y) = to-page(p)
    place(dx: x - 1.8pt, dy: y - 1.8pt, circle(radius: 1.8pt, fill: dot-col))
  }
}
