#!/bin/bash
# Starts nodalis on a virtual X display and takes screenshots for the user guide.
#
#   docs/user-guide/screenshot.sh start [files...]   start nodalis (1400x900) with these files
#   docs/user-guide/screenshot.sh shot NAME [WxH+X+Y] save the window (or a crop) as images/NAME.png
#   docs/user-guide/screenshot.sh stop               stop nodalis and the display
#
# Drive the program in between with xdotool, e.g. `DISPLAY=:5 xdotool mousemove 246 13 click 1`
# opens the Model menu. Needs Xvfb, xdotool and ImageMagick; pngquant shrinks the PNGs if present.
# NODALIS defaults to target/release/nodalis of this checkout.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
export DISPLAY="${DISPLAY_NUMBER:-:5}"
nodalis="${NODALIS:-$here/../../target/release/nodalis}"

case "${1:-}" in
start)
    shift
    pgrep -f "Xvfb $DISPLAY" >/dev/null || { Xvfb "$DISPLAY" -screen 0 1500x950x24 &>/dev/null & sleep 2; }
    pkill -x nodalis || true
    "$nodalis" "$@" &>/tmp/nodalis-screenshot.log &
    sleep 6
    ;;
shot)
    out="$here/images/$2.png"
    if [ -n "${3:-}" ]; then
        import -window root -crop "$3" +repage "$out"
    else
        import -window root -crop 1400x900+0+0 +repage "$out"
    fi
    if command -v pngquant >/dev/null; then
        pngquant --force --ext .png --quality 80-95 --skip-if-larger "$out" || true
    fi
    echo "$out"
    ;;
stop)
    pkill -x nodalis || true
    pkill -f "Xvfb $DISPLAY" || true
    ;;
*)
    sed -n '2,11p' "$0"
    exit 1
    ;;
esac
