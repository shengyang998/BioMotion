#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NIMBLE_ROOT="$REPO_ROOT/nimblephysics"
SOURCE="$SCRIPT_DIR/c3d_ios_archive_probe.cpp"
SIM_ARCHIVE="$NIMBLE_ROOT/build_sim/libnimble_ios.a"
DEVICE_ARCHIVE="$NIMBLE_ROOT/build_ios/libnimble_ios.a"
OSQP_SIM_ARCHIVE="$REPO_ROOT/osqp/build_sim/out/libosqpstatic.a"
OSQP_DEVICE_ARCHIVE="$REPO_ROOT/osqp/build_ios/out/libosqpstatic.a"
PROBE_TMP="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-c3d-probe.XXXXXX")"
trap 'rm -r "$PROBE_TMP" 2>/dev/null || true' EXIT

SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"
CXX="$(xcrun --sdk iphonesimulator --find clang++)"
OBJECT="$PROBE_TMP/c3d_boundary.o"
EXECUTABLE="$PROBE_TMP/c3d_boundary"
MAP="$PROBE_TMP/c3d_boundary.map"
WHY_LOAD="$PROBE_TMP/why_load.log"
DEVICE_SDK="$(xcrun --sdk iphoneos --show-sdk-path)"
DEVICE_CXX="$(xcrun --sdk iphoneos --find clang++)"
DEVICE_OBJECT="$PROBE_TMP/c3d_boundary_device.o"
DEVICE_EXECUTABLE="$PROBE_TMP/c3d_boundary_device"
DEVICE_MAP="$PROBE_TMP/c3d_boundary_device.map"
DEVICE_WHY_LOAD="$PROBE_TMP/device_why_load.log"

COMMON=(
  -target arm64-apple-ios17.0-simulator
  -isysroot "$SDK"
  -std=c++17
  -DDART_IOS_BUILD=1
  -DDART_USE_IDENTITY_JACOBIAN=1
  -DEIGEN_DONT_PARALLELIZE
  -Wno-deprecated-literal-operator
  -I"$NIMBLE_ROOT/build_sim"
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
  -I"$NIMBLE_ROOT/build_ios"
  -I"$NIMBLE_ROOT"
  -I"$NIMBLE_ROOT/third_party/eigen"
  -I"$NIMBLE_ROOT/third_party/tinyxml2"
)

failures=0

record_failure() {
  printf 'FAIL %s\n' "$1" >&2
  failures=$((failures + 1))
}

compile_must_succeed() {
  label=$1
  shift
  log="$PROBE_TMP/${label}.log"
  if ! "$CXX" "${COMMON[@]}" "$@" -fsyntax-only "$SOURCE" \
      >"$log" 2>&1; then
    cat "$log" >&2
    record_failure "$label must compile for an iOS consumer"
  fi
}

production_source_must_compile() {
  label=$1
  source_file=$2
  log="$PROBE_TMP/${label}.log"
  if ! "$CXX" "${COMMON[@]}" -fsyntax-only "$source_file" \
      >"$log" 2>&1; then
    cat "$log" >&2
    record_failure "$label must remain dependency-free in an iOS build"
  fi
}

compile_must_fail() {
  label=$1
  macro=$2
  diagnostic_kind=$3
  diagnostic_token=$4
  log="$PROBE_TMP/${label}.log"
  if "$CXX" "${COMMON[@]}" "-D${macro}=1" -fsyntax-only "$SOURCE" \
      >"$log" 2>&1; then
    record_failure "$label unexpectedly remains in the iOS public API"
    return
  fi
  if ! grep -F 'error:' "$log" \
      | grep -F "$diagnostic_kind" \
      | grep -Fq "'$diagnostic_token'"; then
    cat "$log" >&2
    record_failure "$label failed for an unrelated reason"
  fi
}

count_fixed_lines() {
  needle=$1
  input_file=$2
  awk -v needle="$needle" '$0 == needle { count++ } END { print count + 0 }' \
    "$input_file"
}

count_containing_lines() {
  needle=$1
  input_file=$2
  awk -v needle="$needle" \
    'index($0, needle) != 0 { count++ } END { print count + 0 }' \
    "$input_file"
}

contains_forbidden_c3d_surface() {
  grep -Eq \
    'ezc3d|C3DLoader|C3D::getWeightedDistFromCoPToNearestMarker|FORCE_PLATFORM_NUM_CONVENTIONS|dart::biomechanics::ForcePlatforms?([^[:alnum:]_]|$)' \
    "$@"
}

verify_archive_platform() {
  archive_label=$1
  archive_file=$2
  expected_platform=$3
  members_file=$4
  extract_dir="$PROBE_TMP/${archive_label}.objects"
  objects_file="$PROBE_TMP/${archive_label}.objects.list"

  mkdir -p "$extract_dir"
  (
    cd "$extract_dir"
    ar -x "$archive_file"
  )
  find "$extract_dir" -type f -name '*.o' -print | sort > "$objects_file"

  expected_objects=$(awk '/\.o$/ { count++ } END { print count + 0 }' \
    "$members_file")
  actual_objects=$(awk 'END { print NR + 0 }' "$objects_file")
  if [ "$expected_objects" -eq 0 ] \
      || [ "$actual_objects" -ne "$expected_objects" ]; then
    record_failure \
      "$archive_label archive extraction did not preserve every object member"
    return
  fi

  while IFS= read -r object_file; do
    if ! platform_values="$(
        otool -l "$object_file" | awk '
$1 == "cmd" && $2 == "LC_BUILD_VERSION" { awaiting_platform = 1; next }
awaiting_platform && $1 == "platform" { print $2; awaiting_platform = 0 }
'
    )"; then
      record_failure "$archive_label could not inspect $(basename "$object_file")"
      continue
    fi
    if [ "$platform_values" != "$expected_platform" ]; then
      record_failure \
        "$archive_label $(basename "$object_file") has platform ${platform_values:-missing}, expected $expected_platform"
    fi
  done < "$objects_file"
}

verify_link_receipt() {
  link_label=$1
  why_load_file=$2
  map_file=$3
  executable_file=$4

  for required_member in OpenSimParser.cpp.o ForcePlate.cpp.o; do
    if ! grep -Fq "$required_member" "$why_load_file"; then
      record_failure \
        "$link_label ordinary archive link did not extract $required_member"
    fi
    if ! grep -Fq "$required_member" "$map_file"; then
      record_failure "$link_label link map does not contain $required_member"
    fi
  done
  if grep -Eq 'C3DLoader\.cpp\.o|C3DForcePlatforms\.cpp\.o|ezc3d' \
      "$why_load_file" "$map_file"; then
    record_failure \
      "$link_label ordinary archive link extracted an unavailable C3D backend"
  fi

  nm -gu "$executable_file" | c++filt \
    > "$PROBE_TMP/${link_label}.final.undefined"
  if grep -Eq 'dart::|ezc3d' \
      "$PROBE_TMP/${link_label}.final.undefined"; then
    record_failure \
      "$link_label linked probe retains an unresolved C3D/DART dependency"
  fi
}

for required_file in \
  "$SOURCE" \
  "$SIM_ARCHIVE" \
  "$DEVICE_ARCHIVE" \
  "$OSQP_SIM_ARCHIVE" \
  "$OSQP_DEVICE_ARCHIVE"; do
  if [ ! -f "$required_file" ]; then
    record_failure "required file is missing: $required_file"
  fi
done

if [ "$failures" -ne 0 ]; then
  exit 1
fi

compile_must_succeed c3d_value_surface
compile_must_succeed force_platform_header \
  -DBIOMOTION_C3D_FORCE_HEADER_ONLY=1
production_source_must_compile \
  c3d_loader_source "$NIMBLE_ROOT/dart/biomechanics/C3DLoader.cpp"
production_source_must_compile \
  c3d_force_platform_source \
  "$NIMBLE_ROOT/dart/biomechanics/C3DForcePlatforms.cpp"

compile_must_fail \
  weighted_method BIOMOTION_C3D_WEIGHTED_METHOD \
  'no member named' \
  getWeightedDistFromCoPToNearestMarker
compile_must_fail \
  c3d_loader BIOMOTION_C3D_LOADER 'no type named' C3DLoader
compile_must_fail \
  force_convention BIOMOTION_C3D_FORCE_CONVENTION \
  'no member named' \
  FORCE_PLATFORM_NUM_CONVENTIONS
compile_must_fail \
  force_platform BIOMOTION_C3D_FORCE_PLATFORM \
  'no type named' ForcePlatform
compile_must_fail \
  force_platforms BIOMOTION_C3D_FORCE_PLATFORMS \
  'no type named' ForcePlatforms

if [ "$failures" -ne 0 ]; then
  printf 'C3D iOS surface probe found %s contract failure(s)\n' \
    "$failures" >&2
  exit 1
fi

for archive in "$SIM_ARCHIVE" "$DEVICE_ARCHIVE"; do
  archive_label="$(basename "$(dirname "$archive")")"
  members="$PROBE_TMP/${archive_label}.members"
  defined="$PROBE_TMP/${archive_label}.defined"
  undefined="$PROBE_TMP/${archive_label}.undefined"
  ar -t "$archive" > "$members"
  nm -gU "$archive" | c++filt > "$defined"
  nm -gu "$archive" | c++filt > "$undefined"

  if grep -Eq '(^|/)(C3DLoader|C3DForcePlatforms)\.cpp\.o$' "$members"; then
    record_failure "$archive contains an unavailable C3D implementation"
  fi
  for required_member in OpenSimParser.cpp.o ForcePlate.cpp.o; do
    if [ "$(count_fixed_lines "$required_member" "$members")" -ne 1 ]; then
      record_failure "$archive does not contain exactly one $required_member"
    fi
  done
  if [ "$(count_containing_lines \
      'OpenSimParser::loadMotAtLowestMarkerRMSERotation' "$defined")" \
      -ne 1 ]; then
    record_failure "$archive does not define exactly one OpenSim C3D consumer"
  fi
  if [ "$(count_containing_lines \
      'ForcePlate::copyForcePlate' "$defined")" -ne 1 ]; then
    record_failure "$archive does not define exactly one ForcePlate copy API"
  fi
  if contains_forbidden_c3d_surface "$defined" "$undefined"; then
    record_failure "$archive contains an unavailable C3D surface or dependency"
  fi

  if [ "$archive" = "$SIM_ARCHIVE" ]; then
    verify_archive_platform simulator "$archive" 7 "$members"
  else
    verify_archive_platform device "$archive" 2 "$members"
  fi
done

if [ "$failures" -ne 0 ]; then
  exit 1
fi

"$CXX" "${COMMON[@]}" -c "$SOURCE" -o "$OBJECT"
nm -gu "$OBJECT" | c++filt > "$PROBE_TMP/object.undefined"
for required_symbol in \
  'OpenSimParser::loadMotAtLowestMarkerRMSERotation' \
  'ForcePlate::copyForcePlate'; do
  if ! grep -Fq "$required_symbol" "$PROBE_TMP/object.undefined"; then
    record_failure "probe object does not reference $required_symbol"
  fi
done
if contains_forbidden_c3d_surface "$PROBE_TMP/object.undefined"; then
  record_failure 'probe object directly references an unavailable C3D API'
fi

if [ "$failures" -ne 0 ]; then
  exit 1
fi

if ! "$CXX" "${COMMON[@]}" \
    "$OBJECT" \
    "$SIM_ARCHIVE" \
    "$OSQP_SIM_ARCHIVE" \
    -lc++ \
    -Wl,-dead_strip \
    -Wl,-why_load \
    -Wl,-map,"$MAP" \
    -o "$EXECUTABLE" \
    2> "$WHY_LOAD"; then
  cat "$WHY_LOAD" >&2
  exit 2
fi

verify_link_receipt simulator "$WHY_LOAD" "$MAP" "$EXECUTABLE"

"$DEVICE_CXX" "${DEVICE_COMMON[@]}" -c "$SOURCE" -o "$DEVICE_OBJECT"
if ! "$DEVICE_CXX" "${DEVICE_COMMON[@]}" \
    "$DEVICE_OBJECT" \
    "$DEVICE_ARCHIVE" \
    "$OSQP_DEVICE_ARCHIVE" \
    -lc++ \
    -Wl,-dead_strip \
    -Wl,-why_load \
    -Wl,-map,"$DEVICE_MAP" \
    -o "$DEVICE_EXECUTABLE" \
    2> "$DEVICE_WHY_LOAD"; then
  cat "$DEVICE_WHY_LOAD" >&2
  exit 2
fi
verify_link_receipt \
  device "$DEVICE_WHY_LOAD" "$DEVICE_MAP" "$DEVICE_EXECUTABLE"

if [ "$failures" -ne 0 ]; then
  exit 1
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
  exit 3
fi

xcrun simctl bootstatus "$SIMULATOR_UDID" -b >/dev/null
if PROBE_OUTPUT="$(
    xcrun simctl spawn "$SIMULATOR_UDID" "$EXECUTABLE" 2>&1
)"; then
  probe_status=0
else
  probe_status=$?
fi
printf '%s\n' "$PROBE_OUTPUT"
if [ "$probe_status" -ne 0 ]; then
  printf 'C3D archive probe failed with status %s\n' "$probe_status" >&2
  exit "$probe_status"
fi
if ! printf '%s\n' "$PROBE_OUTPUT" \
    | grep -Fxq 'C3D_IOS_ARCHIVE_PROBE_PASS'; then
  printf '%s\n' 'C3D archive probe omitted its pass sentinel' >&2
  exit 4
fi

printf '%s\n' 'C3D_IOS_BOUNDARY_PROBE_PASS'
