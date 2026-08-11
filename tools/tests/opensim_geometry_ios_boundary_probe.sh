#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NIMBLE_ROOT="$REPO_ROOT/nimblephysics"
SOURCE="$SCRIPT_DIR/opensim_geometry_ios_boundary_probe.cpp"
SIM_ARCHIVE="$NIMBLE_ROOT/build_sim/libnimble_ios.a"
DEVICE_ARCHIVE="$NIMBLE_ROOT/build_ios/libnimble_ios.a"
OSQP_SIM_ARCHIVE="$REPO_ROOT/osqp/build_sim/out/libosqpstatic.a"
OSQP_DEVICE_ARCHIVE="$REPO_ROOT/osqp/build_ios/out/libosqpstatic.a"
PROBE_TMP="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-opensim-geometry.XXXXXX")"
trap 'rm -r "$PROBE_TMP" 2>/dev/null || true' EXIT

SIM_SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"
SIM_CXX="$(xcrun --sdk iphonesimulator --find clang++)"
DEVICE_SDK="$(xcrun --sdk iphoneos --show-sdk-path)"
DEVICE_CXX="$(xcrun --sdk iphoneos --find clang++)"

COMMON=(
  -std=c++17
  -DDART_IOS_BUILD=1
  -DDART_USE_IDENTITY_JACOBIAN=1
  -DEIGEN_DONT_PARALLELIZE
  -Wall
  -Wextra
  -Werror
  -Wno-deprecated-literal-operator
  -Wno-sign-compare
  -I"$NIMBLE_ROOT"
  -I"$NIMBLE_ROOT/third_party/eigen"
  -I"$NIMBLE_ROOT/third_party/tinyxml2"
)

for required_file in \
  "$SOURCE" \
  "$SIM_ARCHIVE" \
  "$DEVICE_ARCHIVE" \
  "$OSQP_SIM_ARCHIVE" \
  "$OSQP_DEVICE_ARCHIVE"; do
  if [ ! -f "$required_file" ]; then
    printf 'required file is missing: %s\n' "$required_file" >&2
    exit 2
  fi
done

SIM_OBJECT="$PROBE_TMP/OpenSimGeometryBoundary.sim.o"
SIM_EXECUTABLE="$PROBE_TMP/OpenSimGeometryBoundary.sim"
SIM_MAP="$PROBE_TMP/OpenSimGeometryBoundary.sim.map"
SIM_WHY_LOAD="$PROBE_TMP/OpenSimGeometryBoundary.sim.why_load"
DEVICE_OBJECT="$PROBE_TMP/OpenSimGeometryBoundary.device.o"
DEVICE_EXECUTABLE="$PROBE_TMP/OpenSimGeometryBoundary.device"
DEVICE_MAP="$PROBE_TMP/OpenSimGeometryBoundary.device.map"
DEVICE_WHY_LOAD="$PROBE_TMP/OpenSimGeometryBoundary.device.why_load"

"$SIM_CXX" \
  -target arm64-apple-ios17.0-simulator \
  -isysroot "$SIM_SDK" \
  "${COMMON[@]}" \
  -c "$SOURCE" \
  -o "$SIM_OBJECT"

"$DEVICE_CXX" \
  -target arm64-apple-ios17.0 \
  -isysroot "$DEVICE_SDK" \
  "${COMMON[@]}" \
  -c "$SOURCE" \
  -o "$DEVICE_OBJECT"

"$SIM_CXX" \
  -target arm64-apple-ios17.0-simulator \
  -isysroot "$SIM_SDK" \
  "$SIM_OBJECT" \
  "$SIM_ARCHIVE" \
  "$OSQP_SIM_ARCHIVE" \
  -Wl,-dead_strip \
  -Wl,-why_load \
  -Wl,-map,"$SIM_MAP" \
  -o "$SIM_EXECUTABLE" \
  2>"$SIM_WHY_LOAD"

"$DEVICE_CXX" \
  -target arm64-apple-ios17.0 \
  -isysroot "$DEVICE_SDK" \
  "$DEVICE_OBJECT" \
  "$DEVICE_ARCHIVE" \
  "$OSQP_DEVICE_ARCHIVE" \
  -Wl,-dead_strip \
  -Wl,-why_load \
  -Wl,-map,"$DEVICE_MAP" \
  -o "$DEVICE_EXECUTABLE" \
  2>"$DEVICE_WHY_LOAD"

for receipt in \
  "$SIM_WHY_LOAD" "$SIM_MAP" "$DEVICE_WHY_LOAD" "$DEVICE_MAP"; do
  if ! grep -Fq 'OpenSimParser.cpp.o' "$receipt"; then
    printf 'ordinary archive link omitted OpenSimParser.cpp.o: %s\n' \
      "$receipt" >&2
    exit 3
  fi
done

for executable in "$SIM_EXECUTABLE" "$DEVICE_EXECUTABLE"; do
  executable_label="$(basename "$executable")"
  undefined_symbols="$PROBE_TMP/${executable_label}.undefined"
  nm -gu "$executable" | c++filt > "$undefined_symbols"
  if grep -Fq 'dart::' "$undefined_symbols"; then
    printf 'linked probe retains an unresolved DART symbol: %s\n' \
      "$executable" >&2
    exit 4
  fi
done

if [ -z "${SIMULATOR_UDID:-}" ]; then
  if ! SIMULATOR_UDID="$(
    xcrun simctl list devices -j | python3 -c '
import json
import sys

for runtime, devices in json.load(sys.stdin)["devices"].items():
    if "iOS" not in runtime:
        continue
    for device in devices:
        if device.get("isAvailable") and device.get("state") == "Booted":
            print(device["udid"])
            raise SystemExit(0)
raise SystemExit(1)
'
  )"; then
    printf '%s\n' 'no booted, available iOS simulator was found' >&2
    exit 5
  fi
fi

if output="$(
  xcrun simctl spawn "$SIMULATOR_UDID" "$SIM_EXECUTABLE" 2>&1
)"; then
  status=0
else
  status=$?
fi
printf '%s\n' "$output"
if [ "$status" -ne 0 ]; then
  printf 'OpenSim geometry probe returned status %s\n' "$status" >&2
  exit "$status"
fi
if ! printf '%s\n' "$output" \
    | grep -Fxq 'OPENSIM_GEOMETRY_IOS_BOUNDARY_PASS'; then
  printf '%s\n' 'OpenSim geometry probe omitted its pass sentinel' >&2
  exit 6
fi

printf '%s\n' 'OPENSIM_GEOMETRY_IOS_ARCHIVES_PASS'
