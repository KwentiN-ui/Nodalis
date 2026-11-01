#!/usr/bin/env bash
# Builds the fixture corpus of the Nodalis solver (solver/fixtures) and its golden files:
#
#   1. runs the plx-io writer tests with PLXS_FIXTURE_DIR set, which makes every test that
#      hands an input file to CalculiX keep it,
#   2. imports those inputs and the examples of testdata/ into solver/fixtures/<area>/,
#      classified by their keywords (plxs-fixtures import),
#   3. computes the golden .frd/.dat with CalculiX for every fixture without one
#      (plxs-fixtures reference).
#
#     scripts/solver_reference.sh                 all of it
#     scripts/solver_reference.sh --all           recompute every golden file
#     scripts/solver_reference.sh --name modal    one fixture's golden file only (step 3)
#
# CalculiX is taken from CCX (environment), else target/ccx/linux/bin/ccx from
# scripts/build_ccx.sh, else `ccx` on the PATH (in a Debian/Ubuntu container
# `apt install calculix-ccx` gives 2.21; the writer tests and fixtures do not depend on the
# differences between 2.21 and 2.23). Golden files are made with one thread.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

ccx="${CCX:-}"
if [ -z "$ccx" ] && [ -x target/ccx/linux/bin/ccx ]; then
    ccx="$root/target/ccx/linux/bin/ccx"
fi
ccx="${ccx:-ccx}"
if ! "$ccx" -v >/dev/null 2>&1; then
    echo "CalculiX nicht gefunden ($ccx); CCX=<pfad> setzen oder scripts/build_ccx.sh" >&2
    exit 1
fi

if [ "${1:-}" = "--name" ]; then
    cargo run -q -p plxs-verify --bin plxs-fixtures -- reference --ccx "$ccx" --name "$2"
    exit 0
fi

dump="$(mktemp -d)"
trap 'rm -rf "$dump"' EXIT
echo "Writer-Tests von plx-io laufen und sammeln ihre Eingaben in $dump"
PLXS_FIXTURE_DIR="$dump" OMP_NUM_THREADS=1 cargo test -q -p plx-io --profile ci >/dev/null || true
cargo run -q -p plxs-verify --bin plxs-fixtures -- import "$dump"

examples="$(mktemp -d)"
trap 'rm -rf "$dump" "$examples"' EXIT
for f in testdata/*.inp; do
    # Files without a step (pure meshes, included files) are no solver fixtures.
    grep -qi '^\*step' "$f" && grep -qvi '^\*include' "$f" && cp "$f" "$examples/"
done
cargo run -q -p plxs-verify --bin plxs-fixtures -- import "$examples" --source testdata

cargo run -q -p plxs-verify --bin plxs-fixtures -- reference --ccx "$ccx" ${1:-}
echo "Feature-Matrix für docs/solver/FIXTURES.md:"
cargo run -q -p plxs-verify --bin plxs-fixtures -- matrix > "$dump/matrix.md"
echo "  $(wc -l < "$dump/matrix.md") Zeilen; die Tabelle in docs/solver/FIXTURES.md ersetzen (cargo test -p plxs-verify prüft sie)"
