#!/bin/bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NIMBLE_ROOT="$REPO_ROOT/nimblephysics"
SOURCE="$SCRIPT_DIR/anthropometrics_ios_boundary_probe.cpp"
ANTHRO_HEADER="$NIMBLE_ROOT/dart/biomechanics/Anthropometrics.hpp"
ANTHRO_IMPLEMENTATION="$NIMBLE_ROOT/dart/biomechanics/Anthropometrics.cpp"
IK_HEADER="$NIMBLE_ROOT/dart/biomechanics/IKErrorReport.hpp"
IK_IMPLEMENTATION="$NIMBLE_ROOT/dart/biomechanics/IKErrorReport.cpp"
SIM_ARCHIVE="$NIMBLE_ROOT/build_sim/libnimble_ios.a"
DEVICE_ARCHIVE="$NIMBLE_ROOT/build_ios/libnimble_ios.a"
OSQP_SIM_ARCHIVE="$REPO_ROOT/osqp/build_sim/out/libosqpstatic.a"
OSQP_DEVICE_ARCHIVE="$REPO_ROOT/osqp/build_ios/out/libosqpstatic.a"
PROBE_TMP="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-anthro-boundary.XXXXXX")"
trap 'rm -r "$PROBE_TMP" 2>/dev/null || true' EXIT

SIM_SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"
SIM_CXX="$(xcrun --sdk iphonesimulator --find clang++)"
DEVICE_SDK="$(xcrun --sdk iphoneos --show-sdk-path)"
DEVICE_CXX="$(xcrun --sdk iphoneos --find clang++)"

SIM_COMMON=(
  -target arm64-apple-ios17.0-simulator
  -isysroot "$SIM_SDK"
  -std=c++17
  -DDART_IOS_BUILD=1
  -DDART_USE_IDENTITY_JACOBIAN=1
  -DEIGEN_DONT_PARALLELIZE
  -Wno-deprecated-literal-operator
  -Wno-sign-compare
  -I"$NIMBLE_ROOT"
  -I"$NIMBLE_ROOT/third_party/eigen"
  -I"$NIMBLE_ROOT/third_party/tinyxml2"
)

DEVICE_COMMON=(
  -target arm64-apple-ios17.0
  -isysroot "$DEVICE_SDK"
  -std=c++17
  -DDART_IOS_BUILD=1
  -DDART_USE_IDENTITY_JACOBIAN=1
  -DEIGEN_DONT_PARALLELIZE
  -Wno-deprecated-literal-operator
  -Wno-sign-compare
  -I"$NIMBLE_ROOT"
  -I"$NIMBLE_ROOT/third_party/eigen"
  -I"$NIMBLE_ROOT/third_party/tinyxml2"
)

failures=0

record_failure() {
  printf 'FAIL %s\n' "$1" >&2
  failures=$((failures + 1))
}

compile_argv_for() {
  if [ "$1" = simulator ]; then
    COMPILE_ARGV=("$SIM_CXX" "${SIM_COMMON[@]}")
  else
    COMPILE_ARGV=("$DEVICE_CXX" "${DEVICE_COMMON[@]}")
  fi
}

archive_for() {
  if [ "$1" = simulator ]; then
    printf '%s\n' "$SIM_ARCHIVE"
  else
    printf '%s\n' "$DEVICE_ARCHIVE"
  fi
}

osqp_archive_for() {
  if [ "$1" = simulator ]; then
    printf '%s\n' "$OSQP_SIM_ARCHIVE"
  else
    printf '%s\n' "$OSQP_DEVICE_ARCHIVE"
  fi
}

for required in \
  "$SOURCE" \
  "$ANTHRO_HEADER" \
  "$ANTHRO_IMPLEMENTATION" \
  "$IK_HEADER" \
  "$IK_IMPLEMENTATION" \
  "$SIM_ARCHIVE" \
  "$DEVICE_ARCHIVE" \
  "$OSQP_SIM_ARCHIVE" \
  "$OSQP_DEVICE_ARCHIVE"; do
  if [ ! -f "$required" ]; then
    record_failure "required input is missing: $required"
  fi
done

FORBIDDEN_CASES=(
  'BIOMOTION_ANTHRO_DEBUG_GUI:debugToGUI'
  'BIOMOTION_ANTHRO_DEBUG_VALUES:debugValues'
  'BIOMOTION_ANTHRO_GET_MARKERS:getMarkers'
  'BIOMOTION_ANTHRO_MEASURE:measure'
  'BIOMOTION_ANTHRO_GET_PDF:getPDF'
  'BIOMOTION_ANTHRO_GET_LOG_PDF:getLogPDF'
  'BIOMOTION_ANTHRO_BODY_GRADIENT:getGradientOfLogPDFWrtBodyScales'
  'BIOMOTION_ANTHRO_BODY_FD_GRADIENT:finiteDifferenceGradientOfLogPDFWrtBodyScales'
  'BIOMOTION_ANTHRO_GROUP_GRADIENT:getGradientOfLogPDFWrtGroupScales'
  'BIOMOTION_ANTHRO_GROUP_FD_GRADIENT:finiteDifferenceGradientOfLogPDFWrtGroupScales'
)

for entry in "${FORBIDDEN_CASES[@]}"; do
  macro="${entry%%:*}"
  symbol="${entry#*:}"
  log="$PROBE_TMP/forbidden_${symbol}.log"
  if "$SIM_CXX" "${SIM_COMMON[@]}" -Wall -Wextra -Werror \
      -Wno-sign-compare "-D${macro}=1" -fsyntax-only "$SOURCE" \
      >"$log" 2>&1; then
    record_failure "iOS header still advertises Anthropometrics::$symbol"
  elif ! grep -F 'error:' "$log" \
      | grep -F 'no member named' \
      | grep -Fq "'$symbol'"; then
    cat "$log" >&2
    record_failure "Anthropometrics::$symbol failed for an unrelated reason"
  fi
done

include_trace="$PROBE_TMP/ios-header-includes.log"
if ! printf '%s\n' \
    '#include "dart/biomechanics/Anthropometrics.hpp"' \
    '#include "dart/biomechanics/IKErrorReport.hpp"' \
    | "$SIM_CXX" "${SIM_COMMON[@]}" -H -x c++ -fsyntax-only - \
      >"$PROBE_TMP/ios-header-includes.stdout" 2>"$include_trace"; then
  cat "$include_trace" >&2
  record_failure 'iOS Anthropometrics/IKErrorReport headers did not compile'
elif grep -Eq '/(LilypadSolver|GUIWebsocketServer|MeshShape)\.hpp$' \
    "$include_trace"; then
  grep -E '/(LilypadSolver|GUIWebsocketServer|MeshShape)\.hpp$' \
    "$include_trace" >&2
  record_failure 'iOS headers retain a mesh or GUI dependency'
fi

for platform in simulator device; do
  compile_argv_for "$platform"
  for implementation in "$ANTHRO_IMPLEMENTATION" "$IK_IMPLEMENTATION"; do
    label="$(basename "$implementation" .cpp)"
    object="$PROBE_TMP/${label}.current.${platform}.o"
    log="$PROBE_TMP/${label}.${platform}.strict.log"
    if ! "${COMPILE_ARGV[@]}" -Wall -Wextra -Werror -Wno-sign-compare \
        -c "$implementation" -o "$object" >"$log" 2>&1; then
      cat "$log" >&2
      record_failure "$label is not warning-clean for $platform"
    fi
  done
done

FORBIDDEN_SYMBOLS=(
  'Anthropometrics::debugToGUI('
  'Anthropometrics::debugValues('
  'Anthropometrics::getMarkers('
  'Anthropometrics::measure('
  'Anthropometrics::getPDF('
  'Anthropometrics::getLogPDF('
  'Anthropometrics::getGradientOfLogPDFWrtBodyScales('
  'Anthropometrics::finiteDifferenceGradientOfLogPDFWrtBodyScales('
  'Anthropometrics::getGradientOfLogPDFWrtGroupScales('
  'Anthropometrics::finiteDifferenceGradientOfLogPDFWrtGroupScales('
)
SUPPORTED_SYMBOLS=(
  'Anthropometrics::loadFromFile('
  'Anthropometrics::addMetric('
  'Anthropometrics::getMetricNames('
  'Anthropometrics::setDistribution('
  'Anthropometrics::getDistribution('
  'Anthropometrics::condition('
  'Anthropometrics::setSkelToMetricPose('
  'IKErrorReport::IKErrorReport('
)

for platform in simulator device; do
  current_anthro="$PROBE_TMP/Anthropometrics.current.${platform}.o"
  current_ik="$PROBE_TMP/IKErrorReport.current.${platform}.o"
  current_symbols="$PROBE_TMP/current.${platform}.symbols"
  if [ ! -f "$current_anthro" ] || [ ! -f "$current_ik" ]; then
    record_failure "$platform current-source objects are unavailable"
    continue
  fi
  if ! nm -gU "$current_anthro" "$current_ik" \
      | c++filt >"$current_symbols"; then
    record_failure "$platform current-source symbols could not be inspected"
    continue
  fi
  for symbol in "${FORBIDDEN_SYMBOLS[@]}"; do
    if grep -Fq "$symbol" "$current_symbols"; then
      record_failure "$platform current source retains unavailable $symbol"
    fi
  done
  for symbol in "${SUPPORTED_SYMBOLS[@]}"; do
    if ! grep -Fq "$symbol" "$current_symbols"; then
      record_failure "$platform current source lost supported $symbol"
    fi
  done
done

for archive in "$SIM_ARCHIVE" "$DEVICE_ARCHIVE"; do
  label="$(basename "$(dirname "$archive")")"
  members="$PROBE_TMP/${label}.members"
  symbols="$PROBE_TMP/${label}.symbols"
  ar -t "$archive" >"$members"
  nm -gU "$archive" | c++filt >"$symbols"
  for member in Anthropometrics.cpp.o IKErrorReport.cpp.o; do
    if [ "$(grep -Fxc "$member" "$members" || true)" -ne 1 ]; then
      record_failure "$label does not contain exactly one $member"
    fi
  done
  for symbol in "${FORBIDDEN_SYMBOLS[@]}"; do
    if grep -Fq "$symbol" "$symbols"; then
      record_failure "$label archive retains unavailable $symbol"
    fi
  done
  for symbol in "${SUPPORTED_SYMBOLS[@]}"; do
    if ! grep -Fq "$symbol" "$symbols"; then
      record_failure "$label archive lost supported $symbol"
    fi
  done
done

for platform in simulator device; do
  compile_argv_for "$platform"
  archive="$(archive_for "$platform")"
  osqp_archive="$(osqp_archive_for "$platform")"
  consumer_object="$PROBE_TMP/runtime.${platform}.o"
  current_anthro="$PROBE_TMP/Anthropometrics.current.${platform}.o"
  current_ik="$PROBE_TMP/IKErrorReport.current.${platform}.o"
  executable="$PROBE_TMP/runtime.${platform}"
  why_load="$PROBE_TMP/runtime.${platform}.why-load"
  map="$PROBE_TMP/runtime.${platform}.map"
  if ! "${COMPILE_ARGV[@]}" -Wall -Wextra -Werror -Wno-sign-compare \
      -c "$SOURCE" -o "$consumer_object" \
      >"$PROBE_TMP/runtime.${platform}.compile" \
      2>&1; then
    cat "$PROBE_TMP/runtime.${platform}.compile" >&2
    record_failure "$platform positive/runtime consumer did not compile"
    continue
  fi
  if [ ! -f "$current_anthro" ] || [ ! -f "$current_ik" ]; then
    record_failure "$platform current-source objects are unavailable for link"
    continue
  fi
  if ! "${COMPILE_ARGV[@]}" "$consumer_object" \
      "$current_anthro" "$current_ik" "$archive" "$osqp_archive" \
      -Wl,-dead_strip -Wl,-why_load -Wl,-map,"$map" \
      -o "$executable" 2>"$why_load"; then
    cat "$why_load" >&2
    record_failure "$platform ordinary archive link failed"
    continue
  fi
  for current_object in "$current_anthro" "$current_ik"; do
    if ! grep -Fq "$current_object" "$map"; then
      record_failure \
        "$platform link receipt omitted $(basename "$current_object")"
    fi
  done
  if grep -Eq \
      'libnimble_ios\.a\((Anthropometrics|IKErrorReport)\.cpp\.o\)' \
      "$why_load" "$map"; then
    record_failure \
      "$platform link extracted stale Anthropometrics/IKErrorReport members"
  fi
  nm -gu "$executable" | c++filt >"$PROBE_TMP/runtime.${platform}.undefined"
  if grep -Fq 'dart::' "$PROBE_TMP/runtime.${platform}.undefined"; then
    record_failure "$platform executable retains unresolved DART symbols"
  fi
done

if [ -z "${SIMULATOR_UDID:-}" ]; then
  SIMULATOR_UDID="$({
    xcrun simctl list devices -j | python3 -c '
import json, sys
for runtime, devices in json.load(sys.stdin)["devices"].items():
    if "iOS" not in runtime:
        continue
    for device in devices:
        if device.get("isAvailable") and device.get("state") == "Booted":
            print(device["udid"])
            raise SystemExit(0)
raise SystemExit(1)
'
  } 2>/dev/null || true)"
fi

if [ -z "${SIMULATOR_UDID:-}" ]; then
  record_failure 'no booted, available iOS simulator was found'
elif [ -f "$PROBE_TMP/runtime.simulator" ]; then
  if output="$(
    xcrun simctl spawn "$SIMULATOR_UDID" \
      "$PROBE_TMP/runtime.simulator" 2>&1
  )"; then
    status=0
  else
    status=$?
  fi
  printf '%s\n' "$output"
  if [ "$status" -ne 0 ]; then
    record_failure "simulator runtime probe returned status $status"
  elif ! printf '%s\n' "$output" \
      | grep -Fxq 'ANTHROPOMETRICS_IOS_BOUNDARY_PASS'; then
    record_failure 'simulator runtime probe omitted pass sentinel'
  fi
fi

if [ "$failures" -ne 0 ]; then
  printf 'Anthropometrics iOS boundary found %s failure(s)\n' "$failures" >&2
  exit 1
fi

printf '%s\n' 'ANTHROPOMETRICS_IOS_ARCHIVES_PASS'
