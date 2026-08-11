#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NIMBLE_ROOT="$REPO_ROOT/nimblephysics"
SOURCE="$SCRIPT_DIR/xml_helpers_characterization_probe.cpp"
SIM_ARCHIVE="$NIMBLE_ROOT/build_sim/libnimble_ios.a"
DEVICE_ARCHIVE="$NIMBLE_ROOT/build_ios/libnimble_ios.a"
PROBE_TMP="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-xml-probe.XXXXXX")"
trap 'rm -r "$PROBE_TMP" 2>/dev/null || true' EXIT

SIM_SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"
SIM_CXX="$(xcrun --sdk iphonesimulator --find clang++)"
DEVICE_SDK="$(xcrun --sdk iphoneos --show-sdk-path)"
DEVICE_CXX="$(xcrun --sdk iphoneos --find clang++)"

COMMON_INCLUDES=(
  -std=c++17
  -DDART_IOS_BUILD=1
  -DDART_USE_IDENTITY_JACOBIAN=1
  -DEIGEN_DONT_PARALLELIZE
  -Wno-deprecated-literal-operator
  -I"$NIMBLE_ROOT"
  -I"$NIMBLE_ROOT/third_party/eigen"
  -I"$NIMBLE_ROOT/third_party/tinyxml2"
)

SIM_OBJECT="$PROBE_TMP/xml_helpers_sim.o"
SIM_EXECUTABLE="$PROBE_TMP/xml_helpers_sim"
SIM_MAP="$PROBE_TMP/xml_helpers_sim.map"
SIM_WHY_LOAD="$PROBE_TMP/xml_helpers_sim.why_load"
DEVICE_OBJECT="$PROBE_TMP/xml_helpers_device.o"
DEVICE_EXECUTABLE="$PROBE_TMP/xml_helpers_device"
DEVICE_MAP="$PROBE_TMP/xml_helpers_device.map"
DEVICE_WHY_LOAD="$PROBE_TMP/xml_helpers_device.why_load"

for required_file in "$SOURCE" "$SIM_ARCHIVE" "$DEVICE_ARCHIVE"; do
  if [ ! -f "$required_file" ]; then
    printf 'required file is missing: %s\n' "$required_file" >&2
    exit 2
  fi
done

"$SIM_CXX" \
  -target arm64-apple-ios17.0-simulator \
  -isysroot "$SIM_SDK" \
  "${COMMON_INCLUDES[@]}" \
  -c "$SOURCE" \
  -o "$SIM_OBJECT"

nm -u "$SIM_OBJECT" | c++filt > "$PROBE_TMP/probe.undefined"
for required_symbol in \
  'dart::utils::toString(bool)' \
  'dart::utils::toBool(std::__1::basic_string' \
  'dart::utils::toInt(std::__1::basic_string' \
  'dart::utils::toVector3s(std::__1::basic_string' \
  'dart::utils::toIsometry3s(std::__1::basic_string' \
  'dart::utils::getValueString(' \
  'dart::utils::getAttributeVector3s('; do
  if ! grep -Fq "$required_symbol" "$PROBE_TMP/probe.undefined"; then
    printf 'probe object does not directly exercise %s\n' "$required_symbol" >&2
    exit 3
  fi
done

if "$SIM_CXX" \
    -target arm64-apple-ios17.0-simulator \
    -isysroot "$SIM_SDK" \
    "$SIM_OBJECT" \
    "$SIM_ARCHIVE" \
    -Wl,-dead_strip \
    -Wl,-why_load \
    -Wl,-map,"$SIM_MAP" \
    -o "$SIM_EXECUTABLE" \
    2> "$SIM_WHY_LOAD"; then
  :
else
  link_status=$?
  cat "$SIM_WHY_LOAD" >&2
  exit "$link_status"
fi

"$DEVICE_CXX" \
  -target arm64-apple-ios17.0 \
  -isysroot "$DEVICE_SDK" \
  "${COMMON_INCLUDES[@]}" \
  -c "$SOURCE" \
  -o "$DEVICE_OBJECT"

if "$DEVICE_CXX" \
    -target arm64-apple-ios17.0 \
    -isysroot "$DEVICE_SDK" \
    "$DEVICE_OBJECT" \
    "$DEVICE_ARCHIVE" \
    -Wl,-dead_strip \
    -Wl,-why_load \
    -Wl,-map,"$DEVICE_MAP" \
    -o "$DEVICE_EXECUTABLE" \
    2> "$DEVICE_WHY_LOAD"; then
  :
else
  link_status=$?
  cat "$DEVICE_WHY_LOAD" >&2
  exit "$link_status"
fi

for receipt in \
  "$SIM_WHY_LOAD" "$SIM_MAP" "$DEVICE_WHY_LOAD" "$DEVICE_MAP"; do
  if ! grep -Fq 'XmlHelpers.cpp.o' "$receipt"; then
    printf 'ordinary archive link did not record XmlHelpers.cpp.o in %s\n' \
      "$receipt" >&2
    exit 4
  fi
done

for executable in "$SIM_EXECUTABLE" "$DEVICE_EXECUTABLE"; do
  nm -gu "$executable" | c++filt > "$PROBE_TMP/final.undefined"
  if grep -Fq 'dart::' "$PROBE_TMP/final.undefined"; then
    printf 'linked probe retains unresolved DART symbols: %s\n' \
      "$executable" >&2
    exit 5
  fi
done

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
  exit 6
fi

xcrun simctl bootstatus "$SIMULATOR_UDID" -b >/dev/null
if PROBE_OUTPUT="$(
    xcrun simctl spawn "$SIMULATOR_UDID" "$SIM_EXECUTABLE" 2>&1
)"; then
  probe_status=0
else
  probe_status=$?
fi
printf '%s\n' "$PROBE_OUTPUT"
if [ "$probe_status" -ne 0 ]; then
  printf 'XML helper probe failed with status %s\n' "$probe_status" >&2
  exit "$probe_status"
fi
if ! printf '%s\n' "$PROBE_OUTPUT" \
    | grep -Fxq 'XML_HELPERS_CHARACTERIZATION_PASS'; then
  printf '%s\n' 'XML helper probe omitted its pass sentinel' >&2
  exit 7
fi

printf '%s\n' 'XML_HELPERS_ARCHIVE_PROBE_PASS'
