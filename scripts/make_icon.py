#!/usr/bin/env python3
"""Exports the program icon from its Typst source, assets/icon/nodalis-icon.typ.

Writes to assets/icon/:
  nodalis.svg          colour icon on its tile (README, user guide)
  nodalis-light.svg    light grey with dark mesh lines, for light backgrounds
  nodalis-mono.svg     dark grey with light mesh lines

and the raster files that window systems need, to crates/plx-app/assets/icon/:
  nodalis.png          256 px window and task bar icon
  nodalis.ico          16 to 256 px for the Windows program, installer and file association

    python3 scripts/make_icon.py

Needs typst (0.13) on PATH and Pillow. Only for changing the icon, the outputs are checked in.
To change the part itself, see assets/icon/make_part.py.
"""

import io
import pathlib
import re
import subprocess
import tempfile

from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parent.parent
SOURCE = ROOT / "assets/icon/nodalis-icon.typ"
VECTOR = ROOT / "assets/icon"
RASTER = ROOT / "crates/plx-app/assets/icon"
ICO_SIZES = (16, 20, 24, 32, 40, 48, 64, 128, 256)
SMALL = 32  # up to this size the icon is drawn without mesh lines


def typst(out, fmt, ppi=None, **inputs):
    args = ["typst", "compile", "--format", fmt]
    if ppi:
        args += ["--ppi", str(ppi)]
    for key, value in inputs.items():
        args += ["--input", f"{key}={value}"]
    subprocess.run(args + [str(SOURCE), str(out)], check=True)


def minify_svg(text):
    """Typst writes every polygon in its own indented group with all default attributes;
    dropping those and rounding to 0.01 pt cuts the file to about a third."""
    text = re.sub(r">\s+<", "><", text.strip())
    for default in (
        ' class="typst-shape"',
        ' class="typst-group"',
        ' fill-rule="nonzero"',
        ' stroke-linecap="butt"',
        ' stroke-linejoin="miter"',
        ' stroke-miterlimit="4"',
    ):
        text = text.replace(default, "")
    text = re.sub(r"(\d+\.\d{2})\d+", r"\1", text)
    return text + "\n"


def render(size, tmp):
    """The icon at size px: drawn at four times the size and scaled down for clean edges."""
    out = tmp / f"icon-{size}.png"
    typst(out, "png", ppi=72 * 4 * size / 256, detail="small" if size <= SMALL else "full")
    return Image.open(out).convert("RGBA").resize((size, size), Image.LANCZOS)


def main():
    for name, variant in (("nodalis", "color"), ("nodalis-light", "light"), ("nodalis-mono", "mono")):
        path = VECTOR / f"{name}.svg"
        typst(path, "svg", variant=variant)
        path.write_text(minify_svg(path.read_text()))
        print(f"{path.relative_to(ROOT)}: {path.stat().st_size // 1024} KiB")

    RASTER.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        images = {size: render(size, pathlib.Path(tmp)) for size in ICO_SIZES}
    images[256].save(RASTER / "nodalis.png", optimize=True)
    # Each size of the .ico is its own rendering, so the small ones keep the simpler drawing.
    images[256].save(
        RASTER / "nodalis.ico",
        sizes=[(s, s) for s in ICO_SIZES],
        append_images=[images[s] for s in ICO_SIZES[:-1]],
    )
    print(f"{RASTER.relative_to(ROOT)}: nodalis.png, nodalis.ico ({len(ICO_SIZES)} sizes)")


if __name__ == "__main__":
    main()
