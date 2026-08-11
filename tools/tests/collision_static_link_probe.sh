#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ARCHIVE="$REPO_ROOT/nimblephysics/build_sim/libnimble_ios.a"
SOURCE="$SCRIPT_DIR/collision_factory_archive_probe.cpp"
PROBE_TMP="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-collision-probe.XXXXXX")"
trap 'rm -r "$PROBE_TMP" 2>/dev/null || true' EXIT

SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"
CXX="$(xcrun --sdk iphonesimulator --find clang++)"
OBJECT="$PROBE_TMP/factory_only.o"
EXECUTABLE="$PROBE_TMP/factory_only"
MAP="$PROBE_TMP/factory.map"
WHY_LOAD="$PROBE_TMP/why_load.log"
OBJECT_SYMBOLS="$PROBE_TMP/object.symbols"
FINAL_SYMBOLS="$PROBE_TMP/final.symbols"
FINAL_UNDEFINED="$PROBE_TMP/final.undefined"

require_fixed_string() {
  needle="$1"
  haystack="$2"
  diagnostic="$3"
  if ! grep -Fq "$needle" "$haystack"; then
    printf '%s\n' "$diagnostic" >&2
    return 1
  fi
}

test -f "$ARCHIVE"
test "$(uname -m)" = arm64
test "$(lipo -archs "$ARCHIVE")" = arm64

COMMON=(
  -target arm64-apple-ios17.0-simulator
  -isysroot "$SDK"
  -std=c++17
  -DDART_IOS_BUILD=1
  -DDART_USE_IDENTITY_JACOBIAN=1
  -DEIGEN_DONT_PARALLELIZE
  -Wno-deprecated-literal-operator
  -I"$REPO_ROOT/nimblephysics/build_sim"
  -I"$REPO_ROOT/nimblephysics"
  -I"$REPO_ROOT/nimblephysics/third_party/eigen"
)

"$CXX" "${COMMON[@]}" -c "$SOURCE" -o "$OBJECT"

nm -gu "$OBJECT" | c++filt > "$OBJECT_SYMBOLS"
require_fixed_string \
  'dart::collision::CollisionDetector::getFactory()' \
  "$OBJECT_SYMBOLS" \
  'factory-only probe does not reference CollisionDetector::getFactory()'
if grep -Eq 'DARTCollisionDetector|ConstraintSolver|dart::simulation::World' \
    "$OBJECT_SYMBOLS"; then
  printf '%s\n' 'factory-only probe has a forbidden direct dependency' >&2
  exit 20
fi

if "$CXX" "${COMMON[@]}" \
    "$OBJECT" \
    "$ARCHIVE" \
    -lc++ \
    -Wl,-dead_strip \
    -Wl,-why_load \
    -Wl,-map,"$MAP" \
    -o "$EXECUTABLE" \
    2> "$WHY_LOAD"; then
  :
else
  link_status="$?"
  cat "$WHY_LOAD" >&2
  printf 'factory-only archive link failed with status %s\n' \
    "$link_status" >&2
  exit "$link_status"
fi

require_fixed_string \
  'CollisionDetector.cpp.o' \
  "$WHY_LOAD" \
  'ordinary archive link did not extract CollisionDetector.cpp.o'
require_fixed_string \
  'DARTCollisionDetector_ios.cpp.o' \
  "$WHY_LOAD" \
  'ordinary archive link did not extract DARTCollisionDetector_ios.cpp.o'
require_fixed_string \
  'CollisionDetector.cpp.o' \
  "$MAP" \
  'link map does not contain CollisionDetector.cpp.o'
require_fixed_string \
  'DARTCollisionDetector_ios.cpp.o' \
  "$MAP" \
  'link map does not contain DARTCollisionDetector_ios.cpp.o'

if grep -Eq \
    'libnimble_ios\.a\((ConstraintSolver|BoxedLcpConstraintSolver|World|DARTCollisionDetector)\.cpp\.o\)' \
    "$WHY_LOAD"; then
  printf '%s\n' 'factory-only link pulled a forbidden consumer or real backend' >&2
  exit 21
fi

nm -gU "$EXECUTABLE" | c++filt > "$FINAL_SYMBOLS"
nm -gu "$EXECUTABLE" | c++filt > "$FINAL_UNDEFINED"
require_fixed_string \
  'dart::collision::CollisionDetector::getFactory()' \
  "$FINAL_SYMBOLS" \
  'linked probe does not define CollisionDetector::getFactory()'
require_fixed_string \
  'dart::collision::DARTCollisionDetector::create()' \
  "$FINAL_SYMBOLS" \
  'linked probe does not define DARTCollisionDetector::create()'
require_fixed_string \
  'dart::collision::DARTCollisionDetector::getStaticType()' \
  "$FINAL_SYMBOLS" \
  'linked probe does not define DARTCollisionDetector::getStaticType()'
if grep -Eq 'dart::constraint::|dart::simulation::World' "$FINAL_SYMBOLS"; then
  printf '%s\n' 'factory-only executable contains a consumer chain' >&2
  exit 22
fi
if grep -Fq 'dart::' "$FINAL_UNDEFINED"; then
  printf '%s\n' 'factory-only executable has unresolved DART symbols' >&2
  exit 23
fi

if SIMULATOR_UDID="$(
    xcrun simctl list devices -j | python3 -c '
import json, sys
for devices in json.load(sys.stdin)["devices"].values():
    for device in devices:
        if device.get("isAvailable") and device.get("name") == "BioMotion-CI":
            print(device["udid"])
            raise SystemExit(0)
raise SystemExit(1)
'
)"; then
  :
else
  printf '%s\n' 'available BioMotion-CI simulator not found' >&2
  exit 24
fi

xcrun simctl bootstatus "$SIMULATOR_UDID" -b >/dev/null
if PROBE_OUTPUT="$(
    xcrun simctl spawn "$SIMULATOR_UDID" "$EXECUTABLE" 2>&1
)"; then
  probe_status=0
else
  probe_status="$?"
fi
printf '%s\n' "$PROBE_OUTPUT"
if test "$probe_status" -ne 0; then
  printf 'factory-only simulator probe failed with status %s\n' \
    "$probe_status" >&2
  exit "$probe_status"
fi
if ! printf '%s\n' "$PROBE_OUTPUT" \
    | grep -Fxq 'ARCHIVE_FACTORY_PROBE_PASS'; then
  printf '%s\n' 'factory-only simulator probe omitted its pass sentinel' >&2
  exit 25
fi
