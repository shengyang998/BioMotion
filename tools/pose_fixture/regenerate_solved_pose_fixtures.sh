#!/bin/bash -p
#
# Regenerates BioMotionTests/Fixtures/solved_pose_video_{012,015}.txt.
#
# The fixtures hold the RAW per-frame IK joint angles for the two scored gait
# clips. They exist because the 2026-08-13 length-mode registration (owner
# decision 7) traded a live IK re-solve inside the fast lane -- about 6.4
# minutes of Debug IK -- for a stored trajectory. The Savitzky-Golay stage stays
# INSIDE the gates; only IK moves out.
#
# NEVER hand-edit a fixture. The gates re-read the live model's DOF count, DOF
# name order and SHA-256 and fail closed when they disagree with the header, so
# a stale fixture is refused rather than silently scored.
#
# Usage:  /bin/bash -p tools/pose_fixture/regenerate_solved_pose_fixtures.sh

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

GENERATOR='BioMotionTests/SolvedPoseFixtureGeneratorTests'
COMMIT="$(/usr/bin/git rev-parse HEAD)"

printf 'Running the generator (expect several minutes of Debug IK)...\n'
/bin/bash -p tools/run_tests.sh subset "-only-testing:${GENERATOR}" \
  | /usr/bin/tee /tmp/solved_pose_generator.log

# The generator writes into the test host's own container tmp/. Its absolute
# path is printed by the test itself, which is the only place it is knowable —
# the container UUID changes per install.
for clip in video_012 video_015; do
  src="$(/usr/bin/grep -a "SOLVED-POSE-FIXTURE clip=${clip} " /tmp/solved_pose_generator.log \
    | /usr/bin/sed -n 's/.* path=//p' | /usr/bin/tail -n 1)"
  dst="BioMotionTests/Fixtures/solved_pose_${clip}.txt"
  if [ -z "$src" ] || [ ! -f "$src" ]; then
    printf 'FAIL: generator did not report a written file for %s\n' "$clip" >&2
    exit 1
  fi
  /usr/bin/sed "s|^commit .*$|commit ${COMMIT}|" "$src" > "$dst"
  printf 'wrote %s\n' "$dst"
done

printf 'Done. Re-run the length-mode gates.\n'
