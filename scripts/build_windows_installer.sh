#!/usr/bin/env bash
# Builds the Windows installer and the portable ZIP for nodalis on Linux: cross-compiles
# nodalis.exe for x86_64-pc-windows-gnu with MinGW, fetches the Gmsh DLL, builds the CalculiX
# solver and packs everything with NSIS and, for the ZIP, with Python's zipfile.
#
#     scripts/build_windows_installer.sh
#
# Needs (Debian/Ubuntu): sudo apt install gcc-mingw-w64-x86-64 nsis python3 curl xz-utils
# and what scripts/build_ccx.sh needs for CalculiX on Windows,
# and the Rust target:   rustup target add x86_64-pc-windows-gnu
#
# The installer ends up in target/windows-installer/nodalis-<version>-setup.exe, the ZIP in
# target/windows-installer/nodalis-<version>-windows-portable.zip.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
target=x86_64-pc-windows-gnu
out=target/windows-installer
stage="$out/stage"
gmsh=target/gmsh-windows
ccx=target/ccx/windows

for tool in makensis x86_64-w64-mingw32-gcc python3 curl; do
    command -v "$tool" >/dev/null || {
        echo "$tool fehlt, siehe Kopf von $0" >&2
        exit 1
    }
done
rustup target list --installed 2>/dev/null | grep -qx "$target" || rustup target add "$target"

version="$(sed -n '/^\[workspace.package\]/,/^\[/s/^version = "\(.*\)"/\1/p' Cargo.toml)"
[ -n "$version" ] || { echo "Version in Cargo.toml nicht gefunden" >&2; exit 1; }

cargo build --release --locked -p plx-app --target "$target"
[ -f "$gmsh/gmsh-4.15.dll" ] || python3 scripts/fetch_gmsh.py --platform windows --dest "$gmsh"
[ -f "$ccx/bin/ccx.exe" ] || scripts/build_ccx.sh windows
scripts/build_user_guide.sh html

rm -rf "$stage"
mkdir -p "$stage/licenses" "$stage/doc"
cp "target/$target/release/nodalis.exe" "$stage/"
cp "$gmsh/gmsh-4.15.dll" crates/plx-app/assets/icon/nodalis.ico installer/windows/THIRD-PARTY.txt "$stage/"
cp LICENSE "$stage/LICENSE.txt"
cp "$gmsh/GMSH-LICENSE.txt" "$gmsh/GMSH-CREDITS.txt" installer/windows/licenses/*.txt "$stage/licenses/"
cp crates/plx-app/assets/fonts/OFL.txt "$stage/licenses/NotoSans-OFL.txt"
cp target/user-guide/user-guide.html "$stage/doc/"
# CalculiX with its DLLs; nodalis uses it when no other CalculiX is set.
cp -r "$ccx" "$stage/ccx"
commit="$(git rev-parse HEAD 2>/dev/null || echo unbekannt)"
dirty="$(git status --porcelain --untracked-files=no 2>/dev/null | head -1)"
{
    echo "Nodalis $version"
    echo "Quelltext: https://github.com/KwentiN-ui/Nodalis"
    echo "Commit:    $commit${dirty:+ (mit lokalen Änderungen)}"
} >"$stage/SOURCE.txt"
# Windows editors expect CRLF line ends in the text files.
for file in "$stage"/*.txt "$stage"/licenses/*.txt "$stage"/ccx/*.txt "$stage"/ccx/licenses/*.txt; do
    sed -i 's/\r$//; s/$/\r/' "$file"
done

installer="$out/nodalis-$version-setup.exe"
makensis -V2 -DVERSION="$version" -DSTAGE="$root/$stage" -DOUTFILE="$root/$installer" \
    installer/windows/nodalis.nsi
echo "Installer: $installer ($(du -h "$installer" | cut -f1))"

# The portable ZIP, like PrePoMax's: the same files in one folder, unpacked anywhere and started
# without administrator rights or installation. portable.txt makes nodalis keep its settings
# in a Settings folder beside it (crates/plx-app/src/portable.rs).
portable="$out/nodalis-$version-windows-portable.zip"
cat >"$stage/portable.txt" <<EOF
Nodalis $version, portable version

Start nodalis.exe in this folder; no installation or administrator rights are needed.
Settings and the material library are kept in the Settings folder next to it, CalculiX
runs in the Temp folder. To remove Nodalis, delete this folder.

While this file exists, Nodalis stays portable. Without it, settings go to
%APPDATA%\nodalis like with the installer.
EOF
sed -i 's/\r$//; s/$/\r/' "$stage/portable.txt"
rm -f "$portable"
python3 - "$stage" "$portable" "Nodalis-$version" <<'EOF'
import os, sys, zipfile
stage, archive, top = sys.argv[1:]
with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
    for dirpath, dirnames, filenames in os.walk(stage):
        dirnames.sort()
        for name in sorted(filenames):
            path = os.path.join(dirpath, name)
            zf.write(path, os.path.join(top, os.path.relpath(path, stage)))
EOF
echo "Portable ZIP: $portable ($(du -h "$portable" | cut -f1))"
