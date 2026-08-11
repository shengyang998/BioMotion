#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NIMBLE_ROOT="$REPO_ROOT/nimblephysics"
SOURCE="$SCRIPT_DIR/collision_world_leak_probe.cpp"
PROBE_TMP="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-world-leak.XXXXXX")"
trap 'rm -r "$PROBE_TMP" 2>/dev/null || true' EXIT

CONFIGURE_LOG="$PROBE_TMP/configure.log"
BUILD_LOG="$PROBE_TMP/build.log"
EXECUTABLE="$PROBE_TMP/world_leak_probe"

require_output_pattern() {
  pattern="$1"
  output="$2"
  diagnostic="$3"
  if ! printf '%s\n' "$output" | grep -Eq "$pattern"; then
    printf '%s\n' "$diagnostic" >&2
    return 1
  fi
}

for command_name in cmake leaks xcrun; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    printf 'required command is unavailable: %s\n' "$command_name" >&2
    exit 20
  fi
done

if ! cmake \
    -S "$NIMBLE_ROOT" \
    -B "$PROBE_TMP/build" \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    >"$CONFIGURE_LOG" 2>&1; then
  cat "$CONFIGURE_LOG" >&2
  printf '%s\n' 'host leak-probe configure failed' >&2
  exit 21
fi

if ! cmake --build "$PROBE_TMP/build" \
    --target nimble_ios \
    --parallel \
    >"$BUILD_LOG" 2>&1; then
  cat "$BUILD_LOG" >&2
  printf '%s\n' 'host leak-probe native build failed' >&2
  exit 22
fi

HOST_CXX="$(xcrun --sdk macosx --find clang++)"
HOST_SDK="$(xcrun --sdk macosx --show-sdk-path)"
"$HOST_CXX" \
  -isysroot "$HOST_SDK" \
  -std=c++17 \
  -O0 \
  -g \
  -DDART_IOS_BUILD=1 \
  -DDART_USE_IDENTITY_JACOBIAN=1 \
  -DEIGEN_DONT_PARALLELIZE \
  -Wno-deprecated-literal-operator \
  -I"$NIMBLE_ROOT" \
  -I"$NIMBLE_ROOT/third_party/eigen" \
  "$SOURCE" \
  "$PROBE_TMP/build/libnimble_ios.a" \
  -Wl,-dead_strip \
  -o "$EXECUTABLE"

if CONTROL_OUTPUT="$(
    MallocStackLogging=1 leaks --quiet --atExit -- \
      "$EXECUTABLE" --positive-control 2>&1
)"; then
  control_status=0
else
  control_status="$?"
fi
printf '%s\n' "$CONTROL_OUTPUT"
if test "$control_status" -eq 0; then
  printf '%s\n' \
    'positive control was not rejected despite its deliberate leak' >&2
  exit 23
fi
require_output_pattern \
  '^WORLD_LEAK_POSITIVE_CONTROL_REACHED_EXIT$' \
  "$CONTROL_OUTPUT" \
  'positive control did not reach process exit'
require_output_pattern \
  'Process [0-9]+: [1-9][0-9]* leaks? for [1-9][0-9]* total leaked bytes\.' \
  "$CONTROL_OUTPUT" \
  'system leaks tool did not detect the positive-control allocation'
require_output_pattern \
  'ROOT LEAK: <malloc in .*leakForPositiveControl' \
  "$CONTROL_OUTPUT" \
  'positive-control leak report did not identify the deliberate allocation'

if NORMAL_OUTPUT="$(
    MallocStackLogging=1 leaks --quiet --atExit -- "$EXECUTABLE" 2>&1
)"; then
  normal_status=0
else
  normal_status="$?"
fi
printf '%s\n' "$NORMAL_OUTPUT"
if test "$normal_status" -ne 0; then
  printf 'World rejection leak probe failed with status %s\n' \
    "$normal_status" >&2
  exit "$normal_status"
fi
require_output_pattern \
  '^WORLD_COLLISION_REJECTION_NO_LEAK_PATH_REACHED_EXIT$' \
  "$NORMAL_OUTPUT" \
  'World rejection probe did not reach process exit'
require_output_pattern \
  'Process [0-9]+: 0 leaks for 0 total leaked bytes\.' \
  "$NORMAL_OUTPUT" \
  'World rejection paths were not proven leak-free'
if printf '%s\n' "$NORMAL_OUTPUT" | grep -Eq 'ROOT LEAK|[1-9][0-9]* leaks? for'; then
  printf '%s\n' 'World rejection leak probe contains a leak report' >&2
  exit 24
fi

printf '%s\n' 'WORLD_COLLISION_REJECTION_LEAK_PROBE_PASS'
