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
# Two generators write the SAME two destination files.
#
#   (default)  the 5-marker lineage: GaitClipFixture (Python-cache-derived) -> IK.
#   --video    the 2026-08-14 20-marker lineage: the production offline path
#              with ONE named deviation -- video decode -> person box via
#              macOS-host sidecar (macOS Vision, INTERIM provenance) ->
#              SAM3DBodyPose Core ML -> 127 MHR joints -> MHRRetarget ->
#              per-frame IK.
#
# The two write DISTINCT temp filenames and print DISTINCT stdout prefixes on
# purpose. They previously would have shared `solved_pose_<clip>.txt` and the
# `SOLVED-POSE-FIXTURE clip=` marker this script parses with `tail -n 1`, so
# running both in one host would have let the 5-marker output silently overwrite
# the video-driven one.
#
# --video requires the two source clips. They are PERSONAL FOOTAGE that lives
# OUTSIDE this repository and is never copied into it: pass absolute paths and
# the generator reads them in place, recording only SHA-256 and byte size.
# `tools/run_tests.sh` wraps xcodebuild in `/usr/bin/env -i`, so the paths travel
# as TEST_RUNNER_-prefixed ENVIRONMENT VARIABLES OF THE XCODEBUILD PROCESS,
# re-added through that scrub by RUN_TESTS_FORWARDED_ENV_NAMES. The
# xcodebuild-ARGUMENT form this header used to teach was FALSIFIED on 2026-08-14
# (Xcode 26.4/17E192): such tokens become BUILD-SETTING overrides and never reach
# the test host (receipt /tmp/biomotion-tests.uaoLdr, rc 65). --video also needs
# the developer model attached:
#   /bin/bash -p tools/assetpack/dev_bundle_model.sh on && xcodegen generate
#
# --video ALSO builds and runs the macOS HOST tool
# `tools/pose_fixture/person_box_sidecar.swift` once per clip. The iOS Simulator
# has NO Vision ML inference backend (VNDetectHumanRectanglesRequest.perform
# THREW on 12/12 sampled frames, com.apple.Vision Code=9), so the person box is
# computed on macOS and written to ONE sidecar per clip which the generator
# CONSUMES. Sidecars are derivatives of personal footage: they are written to
# BIOMOTION_FIXTURE_SIDECAR_DIR (default: a mode-0700 scratch directory) and the
# host tool REFUSES an output directory that resolves inside this repository.
#
# Usage:
#   /bin/bash -p tools/pose_fixture/regenerate_solved_pose_fixtures.sh
#   BIOMOTION_FIXTURE_VIDEO_012=/abs/video_012.mov \
#   BIOMOTION_FIXTURE_VIDEO_015=/abs/video_015.mov \
#     /bin/bash -p tools/pose_fixture/regenerate_solved_pose_fixtures.sh --video

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

MODE=marker5
case "${1-}" in
  '') ;;
  --video) MODE=video ;;
  *)
    printf 'FAIL: unknown argument %s (expected --video or nothing)\n' "${1-}" >&2
    exit 2
    ;;
esac

CLASS='BioMotionTests/SolvedPoseFixtureGeneratorTests'
COMMIT="$(/usr/bin/git rev-parse HEAD)"
LOG=/tmp/solved_pose_generator.log

if [ "$MODE" = video ]; then
  GENERATOR="${CLASS}/testRegenerateVideoDrivenSolvedPoseFixtures"
  MARKER='SOLVED-POSE-FIXTURE-VIDEO clip='

  for var in BIOMOTION_FIXTURE_VIDEO_012 BIOMOTION_FIXTURE_VIDEO_015; do
    eval "value=\${$var-}"
    if [ -z "$value" ] || [ ! -f "$value" ]; then
      printf 'FAIL: %s must name an existing source video (personal footage,\n' "$var" >&2
      printf '      OUTSIDE this repository; it is read in place and never copied in).\n' >&2
      exit 1
    fi
  done

  # Host-side receipts the Simulator cannot compute for itself. Recorded in the
  # fixture header in lieu of a Vision revision pin (DEVIATION B) and as the
  # model-lock identity (BOTH halves).
  MACOS_PRODUCT="$(/usr/bin/sw_vers -productVersion)"
  MACOS_BUILD="$(/usr/bin/sw_vers -buildVersion)"
  # FIELD-CLASS SANITIZATION (2026-08-14, registered): version/receipt strings
  # sanitize newlines to SPACES and PERMIT `/`. This line previously collapsed
  # newlines to `/` and deleted spaces; the change affects only this header
  # line's separator, in fixtures this round regenerates wholesale.
  XCODE_VERSION="$(/usr/bin/xcodebuild -version | /usr/bin/tr '\n' ' ')"
  MODEL_LOCK_SHA="$(/usr/bin/shasum -a 256 build/assetpack/release/sam3d-body-pose.aar.receipt.json | /usr/bin/awk '{print $1}')"
  DEPS_LOCK_SHA="$(/usr/bin/shasum -a 256 tools/dependencies.lock.json | /usr/bin/awk '{print $1}')"

  # `tools/assetpack/dev_bundle_model.sh on` is the step that runs
  # `verify_model_lock.py receipt <aar> <receipt>` against
  # BioMotion/Resources/SAM3DBodyPose.lock.json and prints
  # MODEL_LOCK_VERIFY_PASS before publishing. Re-extracting the 1.02 GiB AAR
  # here would repeat that check without adding evidence, so this asserts the
  # PRECONDITION instead and records the verified receipt's SHA-256.
  if [ ! -d build/DevBundledModel/SAM3DBodyPose.mlmodelc ]; then
    printf 'FAIL: build/DevBundledModel/SAM3DBodyPose.mlmodelc is absent. Run\n' >&2
    printf '      /bin/bash -p tools/assetpack/dev_bundle_model.sh on && xcodegen generate\n' >&2
    printf '      (it verifies the model lock and publishes the compiled model).\n' >&2
    exit 1
  fi

  # ---- The macOS HOST tool: person boxes, one sidecar per clip. -------------
  #
  # The iOS Simulator has NO Vision ML inference backend, so the generator
  # cannot obtain a person box in the only host it may run in. The sidecar is a
  # derivative of personal footage: it lives OUTSIDE this repository, the tool
  # DERIVES its filename, and the tool REFUSES an output directory that resolves
  # (after symlink resolution) under --repo-root.
  SIDECAR_DIR="${BIOMOTION_FIXTURE_SIDECAR_DIR:-${TMPDIR:-/tmp}/biomotion-person-box-sidecars}"
  /bin/mkdir -p -m 0700 "$SIDECAR_DIR"
  HOST_TOOL_SRC="$REPO_ROOT/tools/pose_fixture/person_box_sidecar.swift"
  HOST_TOOL_BIN="$SIDECAR_DIR/person_box_sidecar"
  printf 'Building the macOS person-box host tool...\n'
  /usr/bin/xcrun swiftc -O \
    -sdk "$(/usr/bin/xcrun --show-sdk-path --sdk macosx)" \
    -o "$HOST_TOOL_BIN" "$HOST_TOOL_SRC"

  for clip in video_012 video_015; do
    suffix="${clip#video_}"
    eval "video=\${BIOMOTION_FIXTURE_VIDEO_${suffix}}"
    host_log="$SIDECAR_DIR/person_box_sidecar_${clip}.log"
    printf 'Computing person boxes for %s on macOS Vision...\n' "$clip"
    PERSON_BOX_SIDECAR_LOG="$host_log" \
      "$HOST_TOOL_BIN" --clip "$clip" --out "$SIDECAR_DIR" \
        --repo-root "$REPO_ROOT" "$video" > "$host_log" 2>&1
    sidecar="$SIDECAR_DIR/person_box_sidecar_${clip}.json"
    if [ ! -f "$sidecar" ]; then
      printf 'FAIL: the host tool wrote no sidecar for %s (see %s)\n' "$clip" "$host_log" >&2
      exit 1
    fi
    eval "BIOMOTION_FIXTURE_BOX_SIDECAR_${suffix}=\"\$sidecar\""
    printf '  %s\n' "$(/usr/bin/grep -a 'SUMMARY' "$host_log" | /usr/bin/tail -n 1)"
    printf '  %s\n' "$(/usr/bin/grep -a 'WROTE' "$host_log" | /usr/bin/tail -n 1)"

    # The two macOS identity receipts must agree, fail-closed: the sidecar
    # carries its own sw_vers and this script carries the one recorded in the
    # fixture header.
    for pair in "macos_product_version:$MACOS_PRODUCT" "macos_build_version:$MACOS_BUILD"; do
      key="${pair%%:*}"; expected="${pair#*:}"
      got="$(/usr/bin/sed -n "s/.*\"${key}\": \"\\([^\"]*\\)\".*/\\1/p" "$sidecar" \
        | /usr/bin/head -n 1)"
      if [ "$got" != "$expected" ]; then
        printf 'FAIL: %s sidecar %s is "%s", this host reports "%s"\n' \
          "$clip" "$key" "$got" "$expected" >&2
        exit 1
      fi
    done
  done

  # `TEST_RUNNER_<NAME>` must be an ENVIRONMENT VARIABLE of the xcodebuild
  # PROCESS -- passing it as an xcodebuild ARGUMENT only creates a build-setting
  # override, which xcodebuild does NOT forward to the test host (measured
  # 2026-08-14 on Xcode 26.4/17E192; receipt /tmp/biomotion-tests.uaoLdr,
  # rc 65). `tools/run_tests.sh` forwards exactly these nine names through its
  # `env -i` scrub; see RUN_TESTS_FORWARDED_ENV_NAMES there.
  printf 'Running the VIDEO-DRIVEN generator (Core ML + Debug IK; expect a long run)...\n'
  TEST_RUNNER_BIOMOTION_FIXTURE_VIDEO_012="${BIOMOTION_FIXTURE_VIDEO_012}" \
  TEST_RUNNER_BIOMOTION_FIXTURE_VIDEO_015="${BIOMOTION_FIXTURE_VIDEO_015}" \
  TEST_RUNNER_BIOMOTION_FIXTURE_BOX_SIDECAR_012="${BIOMOTION_FIXTURE_BOX_SIDECAR_012}" \
  TEST_RUNNER_BIOMOTION_FIXTURE_BOX_SIDECAR_015="${BIOMOTION_FIXTURE_BOX_SIDECAR_015}" \
  TEST_RUNNER_BIOMOTION_FIXTURE_MACOS_PRODUCT="${MACOS_PRODUCT}" \
  TEST_RUNNER_BIOMOTION_FIXTURE_MACOS_BUILD="${MACOS_BUILD}" \
  TEST_RUNNER_BIOMOTION_FIXTURE_XCODE_VERSION="${XCODE_VERSION}" \
  TEST_RUNNER_BIOMOTION_FIXTURE_MODEL_LOCK_SHA256="${MODEL_LOCK_SHA}" \
  TEST_RUNNER_BIOMOTION_FIXTURE_DEPS_LOCK_SHA256="${DEPS_LOCK_SHA}" \
  /bin/bash -p tools/run_tests.sh subset "-only-testing:${GENERATOR}" \
    | /usr/bin/tee "$LOG"
else
  GENERATOR="$CLASS/testRegenerateSolvedPoseFixtures"
  MARKER='SOLVED-POSE-FIXTURE clip='
  printf 'Running the 5-marker generator (expect several minutes of Debug IK)...\n'
  /bin/bash -p tools/run_tests.sh subset "-only-testing:${GENERATOR}" \
    | /usr/bin/tee "$LOG"
fi

# `run_tests_one_lane` redirects xcodebuild's whole stdout+stderr into the LANE
# LOG and prints only its own summary lines, so the generator's marker is NOT in
# this script's tee. Read the lane-log path out of the runner's `log:` line and
# grep THAT. (Getting this wrong is silent: the grep simply finds nothing and
# the script reports "generator did not report a written file".)
LANE_LOG="$(/usr/bin/sed -n 's/^log: *//p' "$LOG" | /usr/bin/tail -n 1)"
if [ -z "$LANE_LOG" ] || [ ! -f "$LANE_LOG" ]; then
  printf 'FAIL: could not locate the lane log from %s\n' "$LOG" >&2
  exit 1
fi
printf 'lane log: %s\n' "$LANE_LOG"

# The generator writes into the test host's own container tmp/. Its absolute
# path is printed by the test itself, which is the only place it is knowable —
# the container UUID changes per install.
for clip in video_012 video_015; do
  src="$(/usr/bin/grep -a "${MARKER}${clip} " "$LANE_LOG" \
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
