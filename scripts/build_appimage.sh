#!/usr/bin/env bash
# Builds the Linux AppImage of nodalis: release build, Gmsh library and the system libraries it
# needs (via linuxdeploy), the CalculiX solver, desktop entry, icon, .ndl file type and license
# texts, packed with appimagetool, and the offline user guide.
#
#     scripts/build_appimage.sh
#
# Needs (Debian/Ubuntu): sudo apt install libasound2-dev libglu1-mesa libxft2 libxcursor1 \
#                            libxinerama1 libgomp1 python3 curl file zsync xz-utils
# and what scripts/build_ccx.sh needs for CalculiX.
# The AppImage runs on systems with a glibc at least as new as the build system's, so releases
# are built on the oldest supported Ubuntu runner.
#
# The AppImage ends up in target/appimage/nodalis-<version>-x86_64.AppImage, next to its .zsync
# file for updates.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"
out=target/appimage
appdir="$out/AppDir"
tools="$out/tools"
gmsh=target/gmsh
ccx=target/ccx/linux
# Pinned so a release does not change with the tools' rolling builds.
linuxdeploy_url=https://github.com/linuxdeploy/linuxdeploy/releases/download/1-alpha-20251107-1/linuxdeploy-x86_64.AppImage
appimagetool_url=https://github.com/AppImage/appimagetool/releases/download/1.9.1/appimagetool-x86_64.AppImage
# Lets the tools run without FUSE, e.g. in containers and on CI runners.
export APPIMAGE_EXTRACT_AND_RUN=1

for tool in python3 curl file; do
    command -v "$tool" >/dev/null || {
        echo "$tool fehlt, siehe Kopf von $0" >&2
        exit 1
    }
done

version="$(sed -n '/^\[workspace.package\]/,/^\[/s/^version = "\(.*\)"/\1/p' Cargo.toml)"
[ -n "$version" ] || { echo "Version in Cargo.toml nicht gefunden" >&2; exit 1; }

cargo build --release --locked -p plx-app
[ -f "$gmsh/libgmsh.so.4.15" ] || python3 scripts/fetch_gmsh.py --platform linux --dest "$gmsh"
[ -x "$ccx/bin/ccx" ] || scripts/build_ccx.sh linux
scripts/build_user_guide.sh html

mkdir -p "$tools"
for entry in "linuxdeploy $linuxdeploy_url" "appimagetool $appimagetool_url"; do
    set -- $entry
    [ -x "$tools/$1" ] || { curl -sSfL -o "$tools/$1" "$2" && chmod +x "$tools/$1"; }
done

rm -rf "$appdir"
# Gmsh is loaded at run time, so linuxdeploy only finds it when named; it bundles the
# libraries Gmsh links against except those every desktop system has (libGL, X11, glibc ...)
# and points the executable's RUNPATH at usr/lib, where the loader then finds libgmsh. It also
# copies the Debian copyright files of the bundled libraries to usr/share/doc/<package>.
"$tools/linuxdeploy" --appdir "$appdir" \
    --executable target/release/nodalis \
    --library "$gmsh/libgmsh.so.4.15" \
    --desktop-file installer/linux/nodalis.desktop \
    --icon-file crates/plx-app/assets/icon/nodalis.png

# The program icon (the file name gives the icon's name, which must match Icon= in the desktop
# entry), as the PNG sizes of nodalis.ico: the small ones are drawn without mesh lines, which a
# scaled-down 256 px image or the SVG would blur into coloured smudges in menus. No SVG, since
# desktops prefer a scalable icon over fitting PNGs. linuxdeploy only links .DirIcon, which file
# managers, AppImageLauncher and Gear Lever show for the AppImage; the AppImage spec asks for a
# real PNG file there.
python3 - crates/plx-app/assets/icon/nodalis.ico "$appdir/usr/share/icons/hicolor" <<'PY'
import pathlib, struct, sys
data = pathlib.Path(sys.argv[1]).read_bytes()
for i in range(struct.unpack_from("<H", data, 4)[0]):
    width, _, _, _, _, _, size, offset = struct.unpack_from("<BBBBHHII", data, 6 + 16 * i)
    width = width or 256
    # The standard sizes of the hicolor theme; the .ico stores each size as a PNG.
    if width in (16, 24, 32, 48, 64, 128, 256):
        assert data[offset : offset + 4] == b"\x89PNG", f"{width} px is not stored as PNG"
        icon = pathlib.Path(sys.argv[2], f"{width}x{width}", "apps", "nodalis.png")
        icon.parent.mkdir(parents=True, exist_ok=True)
        icon.write_bytes(data[offset : offset + size])
PY
rm -f "$appdir/.DirIcon"
cp crates/plx-app/assets/icon/nodalis.png "$appdir/.DirIcon"

# CalculiX with its own libraries (ccx finds them via its RPATH $ORIGIN/../lib), outside of
# linuxdeploy so that it keeps the libraries it was built and tested with. nodalis uses it
# when no other CalculiX is set.
mkdir -p "$appdir/usr/lib/nodalis"
cp -r "$ccx" "$appdir/usr/lib/nodalis/ccx"

install -Dm644 installer/linux/nodalis-mime.xml "$appdir/usr/share/mime/packages/nodalis.xml"

doc="$appdir/usr/share/doc/nodalis"
mkdir -p "$doc/licenses"
cp LICENSE "$doc/LICENSE.txt"
cp installer/linux/THIRD-PARTY.txt "$doc/"
cp "$gmsh/GMSH-LICENSE.txt" "$gmsh/GMSH-CREDITS.txt" installer/windows/licenses/*.txt "$doc/licenses/"
cp crates/plx-app/assets/fonts/OFL.txt "$doc/licenses/NotoSans-OFL.txt"
# Help > User Guide finds it at ../share/doc/nodalis next to usr/bin/nodalis.
cp target/user-guide/user-guide.html "$doc/"
commit="$(git rev-parse HEAD 2>/dev/null || echo unbekannt)"
dirty="$(git status --porcelain --untracked-files=no 2>/dev/null | head -1)"
{
    echo "Nodalis $version"
    echo "Source: https://github.com/KwentiN-ui/Nodalis"
    echo "Commit: $commit${dirty:+ (with local changes)}"
} >"$doc/SOURCE.txt"

appimage="$out/nodalis-$version-x86_64.AppImage"
rm -f "$appimage" "$appimage.zsync"
# Update information for AppImageUpdate, Gear Lever and similar tools: they look for the newest
# .zsync file in the latest GitHub release and download only the changed blocks. appimagetool
# writes the .zsync file into the current directory if zsyncmake is installed; it is moved next
# to the AppImage, and both go to the release.
update_info="gh-releases-zsync|KwentiN-ui|Nodalis|latest|nodalis-*-x86_64.AppImage.zsync"
command -v zsyncmake >/dev/null || echo "Warnung: zsyncmake fehlt (apt install zsync), keine .zsync-Datei" >&2
ARCH=x86_64 VERSION="$version" "$tools/appimagetool" --no-appstream \
    --updateinformation "$update_info" "$appdir" "$appimage"
zsync="$(basename "$appimage").zsync"
[ ! -f "$zsync" ] || mv "$zsync" "$out/"
echo "AppImage: $appimage ($(du -h "$appimage" | cut -f1))"
