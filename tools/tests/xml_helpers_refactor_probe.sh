#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NIMBLE_ROOT="$REPO_ROOT/nimblephysics"
XML_SOURCE="$NIMBLE_ROOT/dart/utils/XmlHelpers.cpp"
LOCALE_SOURCE="$SCRIPT_DIR/xml_helpers_locale_probe.cpp"
SIM_ARCHIVE="$NIMBLE_ROOT/build_sim/libnimble_ios.a"
DEVICE_ARCHIVE="$NIMBLE_ROOT/build_ios/libnimble_ios.a"
PROBE_TMP="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-xml-refactor.XXXXXX")"
trap 'rm -r "$PROBE_TMP" 2>/dev/null || true' EXIT

SIM_SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"
SIM_CXX="$(xcrun --sdk iphonesimulator --find clang++)"
DEVICE_SDK="$(xcrun --sdk iphoneos --show-sdk-path)"
DEVICE_CXX="$(xcrun --sdk iphoneos --find clang++)"

COMMON_FLAGS=(
  -std=c++17
  -DDART_IOS_BUILD=1
  -DDART_USE_IDENTITY_JACOBIAN=1
  -DEIGEN_DONT_PARALLELIZE
  -Wall
  -Wextra
  -Werror
  -Wno-deprecated-literal-operator
  -I"$NIMBLE_ROOT"
  -I"$NIMBLE_ROOT/third_party/eigen"
  -I"$NIMBLE_ROOT/third_party/tinyxml2"
)

for required_file in \
  "$XML_SOURCE" "$LOCALE_SOURCE" "$SIM_ARCHIVE" "$DEVICE_ARCHIVE"; do
  if [ ! -f "$required_file" ]; then
    printf 'required file is missing: %s\n' "$required_file" >&2
    exit 2
  fi
done

if rg -n '#[[:space:]]*include[[:space:]]*[<"]boost/|boost::' \
    "$XML_SOURCE" > "$PROBE_TMP/source-boost.txt"; then
  printf '%s\n' 'XmlHelpers.cpp still has a Boost source dependency:' >&2
  cat "$PROBE_TMP/source-boost.txt" >&2
  exit 3
fi

"$SIM_CXX" \
  -target arm64-apple-ios17.0-simulator \
  -isysroot "$SIM_SDK" \
  "${COMMON_FLAGS[@]}" \
  -c "$XML_SOURCE" \
  -o "$PROBE_TMP/XmlHelpers.sim.o"

"$DEVICE_CXX" \
  -target arm64-apple-ios17.0 \
  -isysroot "$DEVICE_SDK" \
  "${COMMON_FLAGS[@]}" \
  -c "$XML_SOURCE" \
  -o "$PROBE_TMP/XmlHelpers.device.o"

for source_object in \
  "$PROBE_TMP/XmlHelpers.sim.o" "$PROBE_TMP/XmlHelpers.device.o"; do
  nm "$source_object" | c++filt > "$PROBE_TMP/source-object.symbols"
  if grep -Fq 'boost::' "$PROBE_TMP/source-object.symbols"; then
    printf 'fresh no-Boost source compile still emitted Boost symbols: %s\n' \
      "$source_object" >&2
    exit 4
  fi
done

for archive in "$SIM_ARCHIVE" "$DEVICE_ARCHIVE"; do
  member_count="$(ar -t "$archive" | grep -Fxc 'XmlHelpers.cpp.o' || true)"
  if [ "$member_count" -ne 1 ]; then
    printf '%s contains %s XmlHelpers.cpp.o members; expected exactly 1\n' \
      "$archive" "$member_count" >&2
    exit 5
  fi

  symbols="$PROBE_TMP/$(basename "$(dirname "$archive")").symbols"
  nm -arch arm64 -A "$archive" | c++filt \
    | grep -F ':XmlHelpers.cpp.o:' > "$symbols"
  if [ ! -s "$symbols" ]; then
    printf 'could not inspect XmlHelpers.cpp.o symbols in %s\n' "$archive" >&2
    exit 6
  fi
  if grep -Fq 'boost::' "$symbols"; then
    printf 'rebuilt XmlHelpers.cpp.o still exports Boost symbols in %s\n' \
      "$archive" >&2
    grep -Fm 20 'boost::' "$symbols" >&2
    exit 7
  fi
done

SIM_OBJECT="$PROBE_TMP/xml_locale.sim.o"
SIM_EXECUTABLE="$PROBE_TMP/xml_locale.sim"
SIM_MAP="$PROBE_TMP/xml_locale.sim.map"
SIM_WHY_LOAD="$PROBE_TMP/xml_locale.sim.why_load"
DEVICE_OBJECT="$PROBE_TMP/xml_locale.device.o"
DEVICE_EXECUTABLE="$PROBE_TMP/xml_locale.device"
DEVICE_MAP="$PROBE_TMP/xml_locale.device.map"
DEVICE_WHY_LOAD="$PROBE_TMP/xml_locale.device.why_load"

"$SIM_CXX" \
  -target arm64-apple-ios17.0-simulator \
  -isysroot "$SIM_SDK" \
  "${COMMON_FLAGS[@]}" \
  -c "$LOCALE_SOURCE" \
  -o "$SIM_OBJECT"

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
  "${COMMON_FLAGS[@]}" \
  -c "$LOCALE_SOURCE" \
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
    exit 8
  fi
done

for executable in "$SIM_EXECUTABLE" "$DEVICE_EXECUTABLE"; do
  nm -gu "$executable" | c++filt > "$PROBE_TMP/final.undefined"
  if grep -Fq 'dart::' "$PROBE_TMP/final.undefined"; then
    printf 'linked locale probe retains unresolved DART symbols: %s\n' \
      "$executable" >&2
    exit 9
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
  exit 10
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
  printf 'XML locale probe failed with status %s\n' "$probe_status" >&2
  exit "$probe_status"
fi
if ! printf '%s\n' "$PROBE_OUTPUT" \
    | grep -Fxq 'XML_HELPERS_CLASSIC_LOCALE_PASS'; then
  printf '%s\n' 'XML locale probe omitted its pass sentinel' >&2
  exit 11
fi

"$SCRIPT_DIR/xml_helpers_characterization_probe.sh"
printf '%s\n' 'XML_HELPERS_NO_BOOST_ARCHIVES_PASS'
