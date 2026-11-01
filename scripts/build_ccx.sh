#!/usr/bin/env bash
# Builds CalculiX (ccx), the reference the solver fixtures were made with (scripts/solver_reference.sh,
# plxs-fixtures reference), with the SPOOLES, PaStiX
# and ARPACK solvers, for Linux or (cross-compiled with MinGW) for Windows.
#
#     scripts/build_ccx.sh [linux|windows]
#
# The sources are pinned: CalculiX 2.23 from Guido Dhondt's repository with the bug fixes of
# conda-forge's CalculiX package, SPOOLES 2.2 with conda-forge's fixes for its multithreaded
# solver, ARPACK-NG, OpenBLAS, Scotch, and PaStiX in the version modified for CalculiX. Everything
# is linked statically into one executable, which needs nothing but the C library (Linux) or
# Windows 10 or newer.
#
# Result in target/ccx/<platform>: bin/ccx (bin/ccx.exe), licenses/ and SOURCE.txt. Intermediate
# files go to target/ccx-build/<platform>; finished parts are not built again.
#
# Needs (Debian/Ubuntu): sudo apt install gcc g++ gfortran make cmake git curl patch perl
#                            python3 flex bison zlib1g-dev
# and for Windows:       sudo apt install gcc-mingw-w64-x86-64 g++-mingw-w64-x86-64 \
#                            gfortran-mingw-w64-x86-64 wine
set -euo pipefail

platform="${1:-linux}"
case "$platform" in
linux | windows) ;;
*)
    echo "Plattform linux oder windows, nicht $platform" >&2
    exit 1
    ;;
esac

root="$(cd "$(dirname "$0")/.." && pwd)"
work="$root/target/ccx-build/$platform"
out="$root/target/ccx/$platform"
src="$work/src"
prefix="$work/prefix"
jobs="$(nproc)"
mkdir -p "$src" "$prefix/lib" "$prefix/include"

# Pinned sources. Git repositories by commit, archives by SHA-256.
CCX_REPO=https://github.com/Dhondtguido/CalculiX
# The repository's state of 24 Oct 2025, which ccx 2.23 (posted 2 Nov 2025) was made from:
# conda-forge's patches for the 2.23 tarball apply to it unchanged.
CCX_COMMIT=07ab64902a2c152c12d989f71d576f886779cfba
CCX_VERSION=2.23
CCX_FEEDSTOCK_REPO=https://github.com/conda-forge/calculix-feedstock
CCX_FEEDSTOCK_COMMIT=9d06fb5716f31488e18f67d13956164a6857e05c
SPOOLES_URL=http://archive.ubuntu.com/ubuntu/pool/universe/s/spooles/spooles_2.2.orig.tar.gz
# The same file as http://www.netlib.org/linalg/spooles/spooles.2.2.tgz.
SPOOLES_SHA256=a84559a0e987a1e423055ef4fdf3035d55b65bbe4bf915efaa1a35bef7f8c5dd
SPOOLES_FEEDSTOCK_REPO=https://github.com/conda-forge/spooles-feedstock
SPOOLES_FEEDSTOCK_COMMIT=3377d504926b51b831a817749dba8397ecca37bd
ARPACK_REPO=https://github.com/opencollab/arpack-ng
ARPACK_COMMIT=40329031ae8deb7c1e26baf8353fa384fc37c251 # 3.9.1
OPENBLAS_REPO=https://github.com/OpenMathLib/OpenBLAS
OPENBLAS_COMMIT=993fad6aebbce34a97d3f8c34d6d79d35b64cc48 # v0.3.30
SCOTCH_URL=http://archive.ubuntu.com/ubuntu/pool/universe/s/scotch/scotch_7.0.11.orig.tar.gz
# Scotch's own release archive of v7.0.11 from gitlab.inria.fr.
SCOTCH_SHA256=d3578b15a8ff5c7924ab0fa4cd166e58d907da4fd95cdaaab37a9034669c64d2
PASTIX_REPO=https://github.com/Kabbone/PaStiX4CalculiX
PASTIX_COMMIT=1b15d792bdf5c6ac4e89e377766668f225b5a4d4 # branch cudaless, tag 2.17_cudaless

if [ "$platform" = windows ]; then
    host=x86_64-w64-mingw32
    # The POSIX thread model, which libgomp and Scotch need (Debian's default is win32).
    CC=$host-gcc-posix CXX=$host-g++-posix FC=$host-gfortran-posix AR=$host-ar RANLIB=$host-ranlib
    exe=ccx.exe
else
    host=
    CC=gcc CXX=g++ FC=gfortran AR=ar RANLIB=ranlib
    exe=ccx
fi
tools=("$CC" "$CXX" "$FC" make cmake git curl patch perl python3 flex bison)
[ "$platform" = windows ] && tools+=(wine)
for tool in "${tools[@]}"; do
    command -v "$tool" >/dev/null || {
        echo "$tool fehlt, siehe Kopf von $0" >&2
        exit 1
    }
done
export CC CXX FC AR RANLIB

# Announces a part. The compilers write to log files, so a line every minute shows on CI that
# the build is still running (OpenBLAS alone takes several minutes).
heartbeat=
step() {
    [ -n "$heartbeat" ] && kill "$heartbeat" 2>/dev/null
    echo "== $*"
    (
        minutes=0
        while sleep 60 >/dev/null 2>&1; do
            minutes=$((minutes + 1))
            echo "   $* läuft seit $minutes min"
        done
    ) &
    heartbeat=$!
}
trap '[ -n "$heartbeat" ] && kill "$heartbeat" 2>/dev/null' EXIT
done_marker() { [ -f "$work/.done-$1" ]; }
mark_done() { touch "$work/.done-$1"; }

# Fetches a commit of a Git repository into $src/<name>.
git_source() {
    local name=$1 repo=$2 commit=$3
    if [ "$(git -C "$src/$name" rev-parse -q --verify HEAD 2>/dev/null)" != "$commit" ]; then
        rm -rf "$src/$name"
        git init -q "$src/$name"
        git -C "$src/$name" fetch -q --depth 1 "$repo" "$commit"
        git -C "$src/$name" checkout -q FETCH_HEAD
    fi
    # Undo the patches of an earlier run.
    git -C "$src/$name" reset -q --hard
    git -C "$src/$name" clean -qfdx
}

# Fetches an archive, checks it and unpacks it into $src/<name>.
# archive_source NAME URL SHA256 [STRIP]: STRIP leading folders (default 1) are dropped.
archive_source() {
    local name=$1 url=$2 sha256=$3 strip=${4:-1} file="$src/$(basename "$2")"
    [ -f "$file" ] || curl -sSfL -o "$file" "$url"
    echo "$sha256  $file" | sha256sum -c --quiet - || {
        rm -f "$file"
        echo "Prüfsumme von $url stimmt nicht" >&2
        exit 1
    }
    rm -rf "$src/$name"
    mkdir -p "$src/$name"
    tar -xf "$file" -C "$src/$name" --strip-components="$strip"
}

cmake_cross=()
if [ "$platform" = windows ]; then
    cmake_cross=(-DCMAKE_SYSTEM_NAME=Windows -DCMAKE_SYSTEM_PROCESSOR=x86_64
        -DCMAKE_C_COMPILER="$CC" -DCMAKE_CXX_COMPILER="$CXX" -DCMAKE_Fortran_COMPILER="$FC"
        -DCMAKE_RC_COMPILER="$host-windres"
        -DCMAKE_FIND_ROOT_PATH="$prefix;/usr/$host" -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER
        -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY
        -DCMAKE_EXE_LINKER_FLAGS=-static -DCMAKE_CROSSCOMPILING_EMULATOR=wine)
    # Scotch runs a program it builds to write its headers, which needs Wine here.
    export WINEPREFIX="$work/wine" WINEDEBUG=-all WINEDLLOVERRIDES="mscoree,mshtml="
fi

# OpenBLAS: BLAS, LAPACK and LAPACKE. Single-threaded, because PaStiX and SPOOLES run their own
# threads; with locking, since those call it from several threads. DYNAMIC_ARCH picks the
# kernels for the processor at run time, from those in DYNAMIC_LIST (all would take hours to
# build); TARGET is the oldest processor the rest of the code runs on.
if ! done_marker openblas; then
    step OpenBLAS
    git_source openblas "$OPENBLAS_REPO" "$OPENBLAS_COMMIT"
    blas_args=(USE_THREAD=0 USE_LOCKING=1 USE_OPENMP=0 DYNAMIC_ARCH=1 NO_SHARED=1 NO_CBLAS=0
        NO_LAPACKE=0 BUILD_TESTING=0 NUM_THREADS=64 CC="$CC" FC="$FC" TARGET=PRESCOTT
        DYNAMIC_LIST="PRESCOTT NEHALEM SANDYBRIDGE HASWELL SKYLAKEX ZEN")
    [ "$platform" = windows ] && blas_args+=(HOSTCC=gcc BINARY=64 OS=WINNT CROSS=1 CROSS_SUFFIX=$host-)
    make -C "$src/openblas" -s -j"$jobs" "${blas_args[@]}" libs netlib >"$work/openblas.log" 2>&1 ||
        { tail -30 "$work/openblas.log"; exit 1; }
    make -C "$src/openblas" -s "${blas_args[@]}" PREFIX="$prefix" install >>"$work/openblas.log" 2>&1
    mark_done openblas
fi

# ARPACK-NG, the eigenvalue solver.
if ! done_marker arpack; then
    step ARPACK
    git_source arpack "$ARPACK_REPO" "$ARPACK_COMMIT"
    cmake -S "$src/arpack" -B "$work/arpack" -DCMAKE_BUILD_TYPE=Release "${cmake_cross[@]}" \
        -DBUILD_SHARED_LIBS=OFF -DMPI=OFF -DICB=OFF -DEXAMPLES=OFF -DTESTS=OFF \
        -DBLA_VENDOR=OpenBLAS -DBLAS_LIBRARIES="$prefix/lib/libopenblas.a" \
        -DLAPACK_LIBRARIES="$prefix/lib/libopenblas.a" -DCMAKE_INSTALL_PREFIX="$prefix" \
        -DCMAKE_INSTALL_LIBDIR=lib >"$work/arpack.log" 2>&1 || { tail -30 "$work/arpack.log"; exit 1; }
    cmake --build "$work/arpack" -j"$jobs" >>"$work/arpack.log" 2>&1 || { tail -30 "$work/arpack.log"; exit 1; }
    cmake --install "$work/arpack" >>"$work/arpack.log" 2>&1
    mark_done arpack
fi

# SPOOLES with conda-forge's fixes: without them the multithreaded solver silently returns
# wrong results now and then.
if ! done_marker spooles; then
    step SPOOLES
    archive_source spooles "$SPOOLES_URL" "$SPOOLES_SHA256" 0
    git_source spooles-feedstock "$SPOOLES_FEEDSTOCK_REPO" "$SPOOLES_FEEDSTOCK_COMMIT"
    patches=(gcc.patch osx.patch)
    [ "$platform" = windows ] && patches=(win.patch osx.patch)
    patches+=(0001-MT-add-acquire-release-primitives-and-order-the-work.patch
        0002-MT-order-the-frontIsDone-p_mtx-publication-in-the-so.patch
        0003-MT-make-the-solve-s-three-inter-phase-barriers-actua.patch
        0004-MT-stop-losing-updates-on-the-factor-s-entry-counter.patch
        0005-MT-give-each-solve-thread-its-own-cpus-vector.patch
        0006-I2Ohash-fix-int-overflow-in-the-bucket-index.patch)
    for p in "${patches[@]}"; do
        patch -s -p1 -d "$src/spooles" -i "$src/spooles-feedstock/recipe/patches/$p"
    done
    (
        cd "$src/spooles"
        make -s lib CC="$CC" AR="$AR" RANLIB="$RANLIB" >"$work/spooles.log" 2>&1
        make -s -C MT/src CC="$CC" AR="$AR" RANLIB="$RANLIB" >>"$work/spooles.log" 2>&1
    ) || { tail -30 "$work/spooles.log"; exit 1; }
    mkdir -p "$prefix/include/spooles"
    cp "$src/spooles/spooles.a" "$src/spooles/MT/src/spoolesMT.a" "$prefix/lib/"
    (cd "$src/spooles" && find . -name '*.h' -exec cp --parents {} "$prefix/include/spooles/" \;)
    mark_done spooles
fi

# Scotch, the graph partitioner PaStiX orders the matrix with.
if ! done_marker scotch; then
    step Scotch
    archive_source scotch "$SCOTCH_URL" "$SCOTCH_SHA256"
    if [ "$platform" = windows ]; then
        # Sets up the Wine prefix once, before parallel jobs would do it at the same time.
        wine cmd /c exit >/dev/null 2>&1 || true
        wineserver -w
    fi
    # As a target name, the program is run through the emulator (Wine) when cross-compiling.
    sed -i 's/COMMAND $<TARGET_FILE:dummysizes>/COMMAND dummysizes/' "$src/scotch/src/libscotch/CMakeLists.txt"
    cmake -S "$src/scotch" -B "$work/scotch" -DCMAKE_BUILD_TYPE=Release "${cmake_cross[@]}" \
        -DBUILD_SHARED_LIBS=OFF -DBUILD_PTSCOTCH=OFF -DBUILD_LIBESMUMPS=OFF -DBUILD_LIBSCOTCHMETIS=OFF \
        -DBUILD_FORTRAN=OFF -DINSTALL_METIS_HEADERS=OFF -DUSE_ZLIB=OFF -DUSE_LZMA=OFF -DUSE_BZ2=OFF \
        -DTHREADS=ON -DMPI_THREAD_MULTIPLE=OFF -DENABLE_TESTS=OFF -DINTSIZE=32 \
        -DCMAKE_INSTALL_PREFIX="$prefix" -DCMAKE_INSTALL_LIBDIR=lib \
        >"$work/scotch.log" 2>&1 || { tail -30 "$work/scotch.log"; exit 1; }
    cmake --build "$work/scotch" -j"$jobs" >>"$work/scotch.log" 2>&1 || { tail -30 "$work/scotch.log"; exit 1; }
    cmake --install "$work/scotch" >>"$work/scotch.log" 2>&1
    mark_done scotch
fi

# PaStiX as modified for CalculiX (matrix reuse, single precision factorisation), without GPU
# support and with 32-bit integers like the rest of ccx.
if ! done_marker pastix; then
    step PaStiX
    git_source pastix "$PASTIX_REPO" "$PASTIX_COMMIT"
    for p in "$root"/scripts/ccx/patches/pastix-*.patch; do
        patch -s -p1 -d "$src/pastix" -i "$p"
    done
    # The static OpenBLAS needs the Fortran runtime for its LAPACK part.
    # PaStiX' CMake modules want each library as a file.
    blas="$prefix/lib/libopenblas.a;$("$FC" -print-file-name=libgfortran.a)"
    blas+=";$("$FC" -print-file-name=libquadmath.a);m;pthread"
    rm -rf "$work/pastix"
    cmake -S "$src/pastix" -B "$work/pastix" -DCMAKE_BUILD_TYPE=Release "${cmake_cross[@]}" \
        -DPYTHON_EXECUTABLE="$(command -v python3)" -DBUILD_SHARED_LIBS=OFF \
        -DPASTIX_WITH_PARSEC=OFF -DPASTIX_WITH_STARPU=OFF -DPASTIX_WITH_CUDA=OFF \
        -DPASTIX_WITH_MPI=OFF -DPASTIX_WITH_FORTRAN=OFF -DPASTIX_ORDERING_SCOTCH=ON \
        -DPASTIX_INT64=OFF -DSCOTCH_DIR="$prefix" -DBLA_VENDOR=OpenBLAS \
        -DBLAS_LIBRARIES="$blas" -DLAPACK_LIBRARIES="$blas" \
        -DCBLAS_INCDIR="$prefix/include" -DLAPACKE_INCDIR="$prefix/include" \
        -DCMAKE_PREFIX_PATH="$prefix" -DCMAKE_INSTALL_PREFIX="$prefix" -DCMAKE_INSTALL_LIBDIR=lib \
        -DCMAKE_DISABLE_FIND_PACKAGE_HWLOC=ON \
        >"$work/pastix.log" 2>&1 || { tail -30 "$work/pastix.log"; exit 1; }
    cmake --build "$work/pastix" -j"$jobs" >>"$work/pastix.log" 2>&1 || { tail -30 "$work/pastix.log"; exit 1; }
    cmake --install "$work/pastix" >>"$work/pastix.log" 2>&1
    mark_done pastix
fi

# CalculiX itself: the repository's sources laid out like the 2.23 tarball, then conda-forge's
# patches in the order of their recipe.
step CalculiX
git_source calculix "$CCX_REPO" "$CCX_COMMIT"
git_source calculix-feedstock "$CCX_FEEDSTOCK_REPO" "$CCX_FEEDSTOCK_COMMIT"
ccx="$work/calculix"
rm -rf "$ccx"
mkdir -p "$ccx/ccx_$CCX_VERSION"
git -C "$src/calculix" archive HEAD src | tar -x -C "$ccx/ccx_$CCX_VERSION"
mv "$ccx/ccx_$CCX_VERSION/src/CalculiX.c" "$ccx/ccx_$CCX_VERSION/src/ccx_$CCX_VERSION.c"
mv "$ccx/ccx_$CCX_VERSION/src/CalculiXstep.c" "$ccx/ccx_$CCX_VERSION/src/ccx_${CCX_VERSION}step.c"
sed -i "s/CalculiXstep\.c/ccx_${CCX_VERSION}step.c/" "$ccx/ccx_$CCX_VERSION/src/Makefile.inc"
recipe="$src/calculix-feedstock/recipe"
python3 - "$recipe/recipe.yaml" "$platform" >"$work/ccx-patches.txt" <<'PY'
import re, sys
# The recipe's patch list; those under "if: win ..." only for Windows, none of "if: osx".
windows = sys.argv[2] == "windows"
condition = ""
for line in open(sys.argv[1]):
    if m := re.match(r"\s*- if: (.*)", line):
        condition = m.group(1)
    elif m := re.match(r"\s*then: (patches/\S+\.patch)", line):
        if windows and "win" in condition:
            print(m.group(1))
    elif m := re.match(r"\s*- (patches/\S+\.patch)", line):
        print(m.group(1))
PY
while read -r p; do
    patch -s -p1 -d "$ccx" -i "$recipe/$p"
done <"$work/ccx-patches.txt"
cd "$ccx/ccx_$CCX_VERSION/src"
# The repository says DEVELOPMENT where the tarball names its version.
sed -i "s/Version DEVELOPMENT/Version $CCX_VERSION/" "ccx_$CCX_VERSION.c" "ccx_${CCX_VERSION}step.c"
# ccx prints its progress (increments, iterations) with printf. Into a pipe that is block
# buffered, so Nodalis would see it only every 4 KB, minutes late on large models: stdout is
# line buffered from the start of main, unbuffered on Windows, whose C library has no line
# buffering. The anchor is the timer start, which conda-forge's mingw_time patch comments out on
# Windows.
perl -0pi -e 's/\n(\s*(?:\/\/\s*)?clock_gettime\(CLOCK_MONOTONIC, &totalCalculixTimeStart\);)/\n#ifdef _WIN32\n  setvbuf(stdout, NULL, _IONBF, 0);\n#else\n  setvbuf(stdout, NULL, _IOLBF, 0);\n#endif\n$1/ or die "main of ccx not found\n"' "ccx_$CCX_VERSION.c"
cp "$recipe/date.pl" date.pl
PKG_VERSION="$CCX_VERSION" perl date.pl
if [ "$platform" = windows ]; then
    ldomp=-fopenmp
    link=(-static)
else
    # libgfortran, libquadmath, libgomp and libstdc++ linked in: only the C library remains. The
    # archives are named directly because gfortran before 13 (Ubuntu 22.04, the AppImage job) has
    # no -static-libquadmath and otherwise links libquadmath.so. libstdc++ is for PaStiX's C++
    # sort, which before GCC 13 needs std::ios_base::Init from it.
    ldomp=
    link=(-static-libgfortran -static-libgcc "$($FC -print-file-name=libgfortran.a)"
        "$($FC -print-file-name=libquadmath.a)" "$($FC -print-file-name=libgomp.a)"
        "$($CXX -print-file-name=libstdc++.a)" -lpthread -ldl -lrt)
fi
make -s -j"$jobs" -f "$root/scripts/ccx/Makefile.nodalis" VERSION="$CCX_VERSION" \
    CC="$CC" FC="$FC" AR="$AR" PREFIX="$prefix" LDOMP="$ldomp" LINK="${link[*]}" >"$work/calculix.log" 2>&1 ||
    { tail -30 "$work/calculix.log"; exit 1; }

rm -rf "$out"
mkdir -p "$out/bin" "$out/licenses"
cp "ccx_${CCX_VERSION}_nodalis" "$out/bin/$exe" 2>/dev/null || cp "ccx_${CCX_VERSION}_nodalis.exe" "$out/bin/$exe"
"${host:+$host-}strip" "$out/bin/$exe"
if [ "$platform" = linux ]; then
    # Only the C library may remain a dependency, otherwise ccx fails on systems without gfortran.
    extra=$(readelf -d "$out/bin/$exe" | sed -n 's/.*NEEDED.*\[\(.*\)\]/\1/p' |
        grep -Ev '^(libc|libm|libpthread|libdl|librt|libmvec|ld-linux-x86-64)\.so' || true)
    [ -z "$extra" ] || { echo "ccx hängt von $extra ab" >&2; exit 1; }
fi
cp "$src/calculix/LICENSE" "$out/licenses/CalculiX-GPL-2.0.txt"
cp "$root/LICENSE" "$out/licenses/GPL-3.0.txt"
cp "$src/arpack/COPYING" "$out/licenses/ARPACK-LICENSE.txt"
cp "$src/openblas/LICENSE" "$out/licenses/OpenBLAS-LICENSE.txt"
cp "$src/openblas/lapack-netlib/LICENSE" "$out/licenses/LAPACK-LICENSE.txt"
cp "$src/scotch/LICENSE_en.txt" "$out/licenses/Scotch-CeCILL-C.txt"
cp "$src/pastix/LICENSE" "$out/licenses/PaStiX-CeCILL-C.txt"
cp "$root/scripts/ccx/GCC-RUNTIME-LIBRARY-EXCEPTION.txt" "$out/licenses/"
[ "$platform" = windows ] && cp "$root/scripts/ccx/WINPTHREADS-LICENSE.txt" "$out/licenses/"
{
    echo "CalculiX ccx $CCX_VERSION for $platform, built by scripts/build_ccx.sh of Nodalis"
    echo "Solvers: SPOOLES (multithreaded), PaStiX, ARPACK"
    echo "CalculiX:  $CCX_REPO/tree/$CCX_COMMIT"
    echo "  patches: $CCX_FEEDSTOCK_REPO/tree/$CCX_FEEDSTOCK_COMMIT/recipe"
    echo "SPOOLES:   $SPOOLES_URL"
    echo "  patches: $SPOOLES_FEEDSTOCK_REPO/tree/$SPOOLES_FEEDSTOCK_COMMIT/recipe/patches"
    echo "ARPACK:    $ARPACK_REPO/tree/$ARPACK_COMMIT"
    echo "OpenBLAS:  $OPENBLAS_REPO/tree/$OPENBLAS_COMMIT"
    echo "Scotch:    $SCOTCH_URL"
    echo "PaStiX:    $PASTIX_REPO/tree/$PASTIX_COMMIT"
    echo "  patches: scripts/ccx/patches in https://github.com/KwentiN-ui/Nodalis"
} >"$out/SOURCE.txt"
echo "CalculiX: $out/bin/$exe ($(du -h "$out/bin/$exe" | cut -f1))"
