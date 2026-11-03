#!/usr/bin/env bash
# Builds the fixture corpus of the Nodalis solver (solver/fixtures) and its plxs snapshots:
#
#   1. runs the plx-io writer tests with PLXS_FIXTURE_DIR set, which makes every test that
#      writes a solver input file keep it,
#   2. imports those inputs and the examples of testdata/ into solver/fixtures/<area>/,
#      classified by their keywords (plxs-fixtures import),
#   3. computes the snapshot .frd/.dat with the release build of plxs for every fixture
#      without one (plxs-fixtures reference).
#
#     scripts/solver_reference.sh                 all of it
#     scripts/solver_reference.sh --all           recompute every snapshot
#     scripts/solver_reference.sh --name modal    one fixture's snapshot only (step 3)
#
# A recomputed snapshot replaces what the corpus tests compare against: only do it when a
# change of the results is intended, and say why in the pull request.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

cargo build -q --release -p plxs-cli --bin plxs
plxs="$root/target/release/plxs"

if [ "${1:-}" = "--name" ]; then
    cargo run -q -p plxs-verify --bin plxs-fixtures -- reference --solver "$plxs" --name "$2"
    exit 0
fi

dump="$(mktemp -d)"
trap 'rm -rf "$dump"' EXIT
echo "Writer-Tests von plx-io laufen und sammeln ihre Eingaben in $dump"
PLXS_FIXTURE_DIR="$dump" cargo test -q -p plx-io --profile ci >/dev/null || true
cargo run -q -p plxs-verify --bin plxs-fixtures -- import "$dump"

examples="$(mktemp -d)"
trap 'rm -rf "$dump" "$examples"' EXIT
for f in testdata/*.inp; do
    # Files without a step (pure meshes, included files) are no solver fixtures.
    grep -qi '^\*step' "$f" && grep -qvi '^\*include' "$f" && cp "$f" "$examples/"
done
cargo run -q -p plxs-verify --bin plxs-fixtures -- import "$examples" --source testdata

cargo run -q -p plxs-verify --bin plxs-fixtures -- reference --solver "$plxs" ${1:-}
echo "Feature-Matrix für docs/solver/FIXTURES.md:"
cargo run -q -p plxs-verify --bin plxs-fixtures -- matrix > "$dump/matrix.md"
echo "  $(wc -l < "$dump/matrix.md") Zeilen; die Tabelle in docs/solver/FIXTURES.md ersetzen (cargo test -p plxs-verify prüft sie)"
