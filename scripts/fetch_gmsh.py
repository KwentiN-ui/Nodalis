#!/usr/bin/env python3
"""Fetches the Gmsh library that nodalis loads at run time.

The official Gmsh builds on PyPI contain the shared library with OpenCASCADE, Netgen and
TetGen built in (libgmsh.so on Linux, gmsh-X.Y.dll on Windows). This script downloads the
wheel for the platform, checks its SHA-256 against PyPI and unpacks the library together with
Gmsh's licence into a folder, by default target/gmsh, where development builds find it.
For a release, put the files next to the nodalis executable.

    python3 scripts/fetch_gmsh.py [--platform linux|windows] [--dest DIR]

Only the Python standard library is needed.
"""

import argparse
import hashlib
import io
import json
import pathlib
import shutil
import ssl
import subprocess
import sys
import urllib.error
import urllib.request
import zipfile

# Keep in step with TESTED_VERSION in crates/plx-mesher/src/gmsh.rs.
VERSION = "4.15.2"

WHEEL_TAGS = {
    "linux": "manylinux_2_24_x86_64",
    "windows": "win_amd64",
}


def download(url):
    """Fetches a URL. Python builds without CA certificates (e.g. the one bundled with
    Inkscape on Windows) cannot check HTTPS certificates; then the system's curl, which uses
    the system's certificate store, fetches instead. The wheel's SHA-256 is checked anyway."""
    try:
        with urllib.request.urlopen(url) as response:
            return response.read()
    except urllib.error.URLError as error:
        if not isinstance(error.reason, ssl.SSLError):
            raise
        curl = shutil.which("curl")
        if curl is None:
            sys.exit(f"HTTPS-Zertifikate nicht prüfbar ({error.reason}) und kein curl gefunden")
        print("Python kann das Zertifikat nicht prüfen, lade mit curl")
        return subprocess.run(
            [curl, "--fail", "--silent", "--show-error", "--location", url],
            check=True,
            stdout=subprocess.PIPE,
        ).stdout


def wheel_url(platform):
    release = json.loads(download(f"https://pypi.org/pypi/gmsh/{VERSION}/json"))
    for file in release["urls"]:
        if file["filename"].endswith(f"{WHEEL_TAGS[platform]}.whl"):
            return file["url"], file["digests"]["sha256"]
    sys.exit(f"Kein Gmsh-{VERSION}-Paket für {platform} auf PyPI gefunden")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    default = "windows" if sys.platform.startswith("win") else "linux"
    parser.add_argument("--platform", choices=sorted(WHEEL_TAGS), default=default)
    root = pathlib.Path(__file__).resolve().parent.parent
    parser.add_argument("--dest", type=pathlib.Path, default=root / "target" / "gmsh")
    args = parser.parse_args()

    url, sha256 = wheel_url(args.platform)
    print(f"Lade {url}")
    data = download(url)
    if hashlib.sha256(data).hexdigest() != sha256:
        sys.exit("Prüfsumme stimmt nicht, Download verworfen")

    args.dest.mkdir(parents=True, exist_ok=True)
    wanted = {"LICENSE.txt": "GMSH-LICENSE.txt", "CREDITS.txt": "GMSH-CREDITS.txt"}
    with zipfile.ZipFile(io.BytesIO(data)) as wheel:
        for name in wheel.namelist():
            base = name.rsplit("/", 1)[-1]
            library = "/lib/" in name and (
                base.startswith("libgmsh.so") or (base.startswith("gmsh") and base.endswith(".dll"))
            )
            if library or (base in wanted and "/doc/gmsh/" in name):
                target = args.dest / (base if library else wanted[base])
                target.write_bytes(wheel.read(name))
                print(f"  {target}")
    print(f"Gmsh {VERSION} liegt in {args.dest}")


if __name__ == "__main__":
    main()
