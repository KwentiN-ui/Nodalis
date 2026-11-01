#!/usr/bin/env bash
# Builds the user guide from docs/user-guide with a pinned Typst, downloaded to target/typst on
# first use:
#
#     scripts/build_user_guide.sh html   # target/user-guide/user-guide.html (shipped with the program)
#     scripts/build_user_guide.sh pdf    # target/user-guide/nodalis-<version>-user-guide.pdf
#
# The HTML guide is one self-contained file (pictures embedded) that Help > User Guide opens in
# the browser. HTML export needs Typst 0.14 or newer and is still marked experimental there.
# Needs: curl, tar with xz.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
format="${1:-html}"
typst_version=0.14.2
typst_dir=target/typst/$typst_version
out=target/user-guide

case "$(uname -s)-$(uname -m)" in
    Linux-x86_64) asset=typst-x86_64-unknown-linux-musl ;;
    Linux-aarch64) asset=typst-aarch64-unknown-linux-musl ;;
    *) echo "Kein Typst-Download für $(uname -s)-$(uname -m) hinterlegt" >&2; exit 1 ;;
esac
if [ ! -x "$typst_dir/typst" ]; then
    mkdir -p "$typst_dir"
    curl -sSfL "https://github.com/typst/typst/releases/download/v$typst_version/$asset.tar.xz" \
        | tar -xJ --strip-components=1 -C "$typst_dir" "$asset/typst"
fi
typst="$typst_dir/typst"

version="$(sed -n '/^\[workspace.package\]/,/^\[/s/^version = "\(.*\)"/\1/p' Cargo.toml)"
[ -n "$version" ] || { echo "Version in Cargo.toml nicht gefunden" >&2; exit 1; }

mkdir -p "$out"
case "$format" in
    html)
        file="$out/user-guide.html"
        "$typst" compile --root . --ignore-system-fonts --features html --format html \
            docs/user-guide/main.typ "$file"
        ;;
    pdf)
        file="$out/nodalis-$version-user-guide.pdf"
        "$typst" compile --root . --ignore-system-fonts docs/user-guide/main.typ "$file"
        ;;
    *) echo "Format $format unbekannt (html oder pdf)" >&2; exit 1 ;;
esac
echo "User guide: $file ($(du -h "$file" | cut -f1))"
