#!/bin/bash -p
#
# Refuses any PERSONAL-FOOTAGE DERIVATIVE inside the repository working tree:
# a source VIDEO of any kind, and (since 2026-08-14) a person-box SIDECAR.
#
# The two clips the 2026-08-14 20-marker fixtures are generated from are
# PERSONAL FOOTAGE that lives outside this repository. The generator reads them
# in place through an environment variable and records only their SHA-256 and
# byte size in the fixture header; nothing may ever copy, stage, or commit the
# bytes themselves.
#
# The SIDECARS are the same class of artefact one derivation later. The iOS
# Simulator has no Vision ML inference backend, so a macOS host tool computes the
# person box per sampled frame and writes one JSON per clip; that JSON contains
# per-frame boxes and per-frame source-pixel hashes of personal footage. It is
# derived FROM the footage, so it is governed BY the footage's rule. Matching by
# EXTENSION alone would have let it through, because a `.json` is not a video.
#
# This is a PREFLIGHT probe rather than an XCTest method for three reasons:
#   1. It is strictly stronger. `tools/run_tests.sh` runs the preflight on EVERY
#      lane -- including `subset`, the only lane the fixture workflow may run --
#      and fails before `xcodebuild` starts. A fast-lane XCTest would never
#      execute during that workflow at all.
#   2. It does not perturb the fast lane's EXACT reviewed test count.
#   3. It is additive-safe against the preflight ORDERING pin in
#      tools/tests/dependency_boundary_probe_tests.sh, which asserts
#      cd < probe < lock < boot by text index rather than by exact block text.
#
# `.gitignore` carries the same extensions as a second layer. That layer alone
# cannot stop `git add -f`, which is why this probe exists as well.
#
# Usage:  /bin/bash -p tools/tests/no_source_video_probe.sh

set -eu

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  case "$-" in
    *p*) ;;
    *)
      printf '%s\n' 'FAIL: execute this script directly or with /bin/bash -p' >&2
      exit 78
      ;;
  esac
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

# Roots that are gitignored build output or vendored dependencies. They are
# excluded from the WALK, not from the rule: nothing in them is committed.
PRUNED='./build
./nimblephysics
./osqp
./DerivedData
./.git'

FOUND=0
while IFS= read -r candidate; do
  [ -n "$candidate" ] || continue
  skip=0
  while IFS= read -r root; do
    case "$candidate" in
      "$root"/*) skip=1; break ;;
    esac
  done <<EOF
$PRUNED
EOF
  [ "$skip" -eq 0 ] || continue
  printf 'NO_SOURCE_VIDEO_FAIL: a personal-footage derivative is inside the repository: %s\n' \
    "$candidate" >&2
  FOUND=$((FOUND + 1))
done <<EOF
$(/usr/bin/find . \
    \( -iname '*.mov' -o -iname '*.mp4' -o -iname '*.m4v' \
       -o -iname '*.avi' -o -iname '*.mkv' -o -iname '*.hevc' \
       -o -iname 'person_box_sidecar_*.json' \) \
    -type f -print 2>/dev/null)
EOF

if [ "$FOUND" -ne 0 ]; then
  printf 'NO_SOURCE_VIDEO_FAIL: %d personal-footage derivative(s) in the tree.\n' \
    "$FOUND" >&2
  printf 'Source clips are read IN PLACE through BIOMOTION_FIXTURE_VIDEO_* and their\n' >&2
  printf 'person-box sidecars through BIOMOTION_FIXTURE_BOX_SIDECAR_*; neither is ever\n' >&2
  printf 'copied, staged, or committed.\n' >&2
  exit 1
fi

printf 'NO_SOURCE_VIDEO_PASS\n'
