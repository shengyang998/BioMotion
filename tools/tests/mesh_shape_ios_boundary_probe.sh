#!/bin/bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NIMBLE_ROOT="$REPO_ROOT/nimblephysics"
HEADER_PROBE="$SCRIPT_DIR/mesh_shape_ios_header_probe.cpp"
MESH_PROBE="$SCRIPT_DIR/mesh_shape_ios_fail_closed_probe.cpp"
SOFT_PROBE="$SCRIPT_DIR/soft_mesh_shape_ios_fail_closed_probe.cpp"
SOURCE_CONTRACT="$SCRIPT_DIR/mesh_shape_ios_source_contract.py"
IMPLEMENTATION="$NIMBLE_ROOT/dart/dynamics/MeshShape_ios.cpp"
MESH_HEADER="$NIMBLE_ROOT/dart/dynamics/MeshShape.hpp"
SOFT_HEADER="$NIMBLE_ROOT/dart/dynamics/SoftMeshShape.hpp"
SIM_ARCHIVE="$NIMBLE_ROOT/build_sim/libnimble_ios.a"
DEVICE_ARCHIVE="$NIMBLE_ROOT/build_ios/libnimble_ios.a"
PROBE_TMP="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-mesh-boundary.XXXXXX")"
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

MESH_MESSAGE='MeshShape is unavailable in this iOS build because Assimp mesh support is not linked.'
SOFT_MESSAGE='SoftMeshShape is unavailable in this iOS build because Assimp mesh support is not linked.'
failures=0

record_failure() {
  printf 'FAIL %s\n' "$1" >&2
  failures=$((failures + 1))
}

compile_sim() {
  "$SIM_CXX" "${SIM_COMMON[@]}" "$@"
}

compile_device() {
  "$DEVICE_CXX" "${DEVICE_COMMON[@]}" "$@"
}

compile_or_record() {
  label=$1
  platform=$2
  shift 2
  log="$PROBE_TMP/${label}.compile.log"
  if [ "$platform" = simulator ]; then
    if ! compile_sim "$@" >"$log" 2>&1; then
      cat "$log" >&2
      record_failure "$label did not compile for the iOS simulator"
      return 1
    fi
  elif ! compile_device "$@" >"$log" 2>&1; then
    cat "$log" >&2
    record_failure "$label did not compile for an iOS device"
    return 1
  fi
  return 0
}

link_or_record() {
  label=$1
  platform=$2
  probe_object=$3
  implementation_object=$4
  archive=$5
  executable=$6
  why_load=$7
  map=$8
  objects=("$probe_object")
  if [ "$implementation_object" != none ]; then
    objects+=("$implementation_object")
  fi

  if [ "$platform" = simulator ]; then
    linker=("$SIM_CXX" "${SIM_COMMON[@]}")
  else
    linker=("$DEVICE_CXX" "${DEVICE_COMMON[@]}")
  fi

  if ! "${linker[@]}" \
      "${objects[@]}" \
      "$archive" \
      -Wl,-dead_strip \
      -Wl,-why_load \
      -Wl,-map,"$map" \
      -o "$executable" \
      2>"$why_load"; then
    cat "$why_load" >&2
    record_failure "$label did not link"
    return 1
  fi

  nm -gu "$executable" | c++filt >"$PROBE_TMP/${label}.undefined"
  if grep -Fq 'dart::' "$PROBE_TMP/${label}.undefined"; then
    record_failure "$label retains an unresolved DART symbol"
  fi
  return 0
}

run_or_record() {
  label=$1
  executable=$2
  sentinel=$3
  if output="$(xcrun simctl spawn "$SIMULATOR_UDID" "$executable" 2>&1)"; then
    status=0
  else
    status=$?
  fi
  printf '%s\n' "$output"
  if [ "$status" -ne 0 ]; then
    record_failure "$label returned status $status"
    return
  fi
  if ! printf '%s\n' "$output" | grep -Fxq "$sentinel"; then
    record_failure "$label omitted $sentinel"
  fi
}

for required_file in \
  "$HEADER_PROBE" \
  "$MESH_PROBE" \
  "$SOFT_PROBE" \
  "$SOURCE_CONTRACT" \
  "$IMPLEMENTATION" \
  "$MESH_HEADER" \
  "$SOFT_HEADER" \
  "$SIM_ARCHIVE" \
  "$DEVICE_ARCHIVE"; do
  if [ ! -f "$required_file" ]; then
    record_failure "required file is missing: $required_file"
  fi
done

if grep -En \
    '^[[:space:]]*(struct|class)[[:space:]]+ai(Scene|Node|Mesh)[[:space:]]*\{' \
    "$MESH_HEADER" "$SOFT_HEADER" \
    >"$PROBE_TMP/fake-assimp-types.log"; then
  cat "$PROBE_TMP/fake-assimp-types.log" >&2
  record_failure \
    'no-Assimp iOS headers define fake Assimp types instead of forward declarations'
fi

if ! python3 "$SOURCE_CONTRACT" "$IMPLEMENTATION" \
    >"$PROBE_TMP/source-contract.stdout" \
    2>"$PROBE_TMP/source-contract.stderr"; then
  cat "$PROBE_TMP/source-contract.stderr" >&2
  record_failure \
    'unreachable MeshShape/SoftMeshShape methods bypass the shared rejection path'
fi

compile_or_record \
  header_surface_simulator simulator -Wall -Wextra -Werror \
  -fsyntax-only "$HEADER_PROBE" || true
compile_or_record \
  header_surface_device device -Wall -Wextra -Werror \
  -fsyntax-only "$HEADER_PROBE" || true

for platform in simulator device; do
  if [ "$platform" = simulator ]; then
    compile_command=compile_sim
  else
    compile_command=compile_device
  fi
  log="$PROBE_TMP/implementation_${platform}_strict.log"
  if ! "$compile_command" \
      -Wall -Wextra -Werror -Wno-sign-compare \
      -fsyntax-only "$IMPLEMENTATION" >"$log" 2>&1; then
    cat "$log" >&2
    record_failure \
      "MeshShape_ios.cpp is not warning-clean for the iOS $platform target"
  fi
done

compile_or_record implementation_simulator simulator \
  -c "$IMPLEMENTATION" -o "$PROBE_TMP/MeshShape_ios.sim.o" || true
compile_or_record implementation_device device \
  -c "$IMPLEMENTATION" -o "$PROBE_TMP/MeshShape_ios.device.o" || true
compile_or_record mesh_probe_simulator simulator -Wall -Wextra -Werror \
  -c "$MESH_PROBE" -o "$PROBE_TMP/mesh.sim.o" || true
compile_or_record mesh_probe_device device -Wall -Wextra -Werror \
  -c "$MESH_PROBE" -o "$PROBE_TMP/mesh.device.o" || true
compile_or_record soft_probe_simulator simulator -Wall -Wextra -Werror \
  -c "$SOFT_PROBE" -o "$PROBE_TMP/soft.sim.o" || true
compile_or_record soft_probe_device device -Wall -Wextra -Werror \
  -c "$SOFT_PROBE" -o "$PROBE_TMP/soft.device.o" || true

for archive in "$SIM_ARCHIVE" "$DEVICE_ARCHIVE"; do
  archive_label="$(basename "$(dirname "$archive")")"
  members="$PROBE_TMP/${archive_label}.members"
  symbols="$PROBE_TMP/${archive_label}.symbols"
  undefined="$PROBE_TMP/${archive_label}.undefined"
  archive_strings="$PROBE_TMP/${archive_label}.strings"
  ar -t "$archive" >"$members"
  nm -gU "$archive" | c++filt >"$symbols"
  nm -gu "$archive" | c++filt >"$undefined"
  strings -a "$archive" >"$archive_strings"

  if [ "$(grep -Fxc 'MeshShape_ios.cpp.o' "$members" || true)" -ne 1 ]; then
    record_failure "$archive does not contain exactly one MeshShape_ios.cpp.o"
  fi
  if [ "$(grep -Fxc 'SoftBodyNode.cpp.o' "$members" || true)" -ne 1 ]; then
    record_failure "$archive does not contain exactly one SoftBodyNode.cpp.o"
  fi
  if grep -Fxq 'MeshShape.cpp.o' "$members" \
      || grep -Fxq 'SoftMeshShape.cpp.o' "$members"; then
    record_failure "$archive contains an Assimp-backed desktop mesh object"
  fi
  for required_symbol in \
    'SharedMeshWrapper::SharedMeshWrapper(aiScene const*)' \
    'SharedMeshWrapper::~SharedMeshWrapper()' \
    'MeshShape::MeshShape(Eigen::Matrix<double, 3, 1, 0, 3, 1> const&, std::__1::shared_ptr<dart::dynamics::SharedMeshWrapper>' \
    'MeshShape::MeshShape(Eigen::Matrix<double, 3, 1, 0, 3, 1> const&, std::__1::basic_string<char' \
    'MeshShape::~MeshShape()' \
    'MeshShape::getType() const' \
    'MeshShape::getStaticType()' \
    'MeshShape::getMesh() const' \
    'MeshShape::getVertices() const' \
    'MeshShape::getMeshUri() const' \
    'MeshShape::getMeshUri2() const' \
    'MeshShape::update()' \
    'MeshShape::getMeshPath() const' \
    'MeshShape::getResourceRetriever()' \
    'MeshShape::setMesh(std::__1::shared_ptr<dart::dynamics::SharedMeshWrapper>, std::__1::basic_string<char' \
    'MeshShape::setMesh(std::__1::shared_ptr<dart::dynamics::SharedMeshWrapper>, dart::common::Uri const&' \
    'MeshShape::setScale(' \
    'MeshShape::getScale() const' \
    'MeshShape::setColorMode(' \
    'MeshShape::getColorMode() const' \
    'MeshShape::setAlphaMode(' \
    'MeshShape::getAlphaMode() const' \
    'MeshShape::setColorIndex(' \
    'MeshShape::getColorIndex() const' \
    'MeshShape::getDisplayList() const' \
    'MeshShape::setDisplayList(' \
    'MeshShape::loadMesh(std::__1::basic_string<char, std::__1::char_traits<char>, std::__1::allocator<char>> const&)' \
    'MeshShape::loadMesh(std::__1::basic_string<char, std::__1::char_traits<char>, std::__1::allocator<char>> const&, std::__1::shared_ptr<dart::common::ResourceRetriever> const&)' \
    'MeshShape::loadMesh(dart::common::Uri const&, std::__1::shared_ptr<dart::common::ResourceRetriever> const&)' \
    'MeshShape::computeInertia(double) const' \
    'MeshShape::clone() const' \
    'MeshShape::updateBoundingBox() const' \
    'MeshShape::updateVolume() const' \
    'SoftMeshShape::SoftMeshShape(dart::dynamics::SoftBodyNode*)' \
    'SoftMeshShape::~SoftMeshShape()' \
    'SoftMeshShape::getType() const' \
    'SoftMeshShape::getStaticType()' \
    'SoftMeshShape::getAssimpMesh() const' \
    'SoftMeshShape::getSoftBodyNode() const' \
    'SoftMeshShape::update()' \
    'SoftMeshShape::computeInertia(double) const' \
    'SoftMeshShape::clone() const' \
    'SoftMeshShape::updateBoundingBox() const' \
    'SoftMeshShape::updateVolume() const' \
    'SoftMeshShape::_buildMesh()' \
    'typeinfo for dart::dynamics::MeshShape' \
    'vtable for dart::dynamics::MeshShape' \
    'typeinfo for dart::dynamics::SoftMeshShape' \
    'vtable for dart::dynamics::SoftMeshShape'; do
    if ! grep -Fq "$required_symbol" "$symbols"; then
      record_failure "$archive does not define $required_symbol"
    fi
  done

  if ! grep -Fxq "$MESH_MESSAGE" "$archive_strings"; then
    record_failure "$archive does not carry the exact MeshShape rejection"
  fi
  if ! grep -Fxq "$SOFT_MESSAGE" "$archive_strings"; then
    record_failure "$archive does not carry the exact SoftMeshShape rejection"
  fi
  if grep -Eq 'Assimp::|ai(ImportFile|ReleaseImport|GetMaterial)' \
      "$undefined"; then
    record_failure "$archive retains an unavailable Assimp dependency"
  fi
done

for implementation_kind in fresh archive; do
  if [ "$implementation_kind" = fresh ]; then
    sim_implementation="$PROBE_TMP/MeshShape_ios.sim.o"
    device_implementation="$PROBE_TMP/MeshShape_ios.device.o"
  else
    sim_implementation=none
    device_implementation=none
  fi

  for probe_kind in mesh soft; do
    case "$probe_kind" in
      mesh)
        sim_probe="$PROBE_TMP/mesh.sim.o"
        device_probe="$PROBE_TMP/mesh.device.o"
        ;;
      soft)
        sim_probe="$PROBE_TMP/soft.sim.o"
        device_probe="$PROBE_TMP/soft.device.o"
        ;;
    esac

    sim_label="${implementation_kind}_${probe_kind}_simulator"
    device_label="${implementation_kind}_${probe_kind}_device"
    sim_executable="$PROBE_TMP/$sim_label"
    device_executable="$PROBE_TMP/$device_label"
    sim_why_load="$PROBE_TMP/$sim_label.why_load"
    device_why_load="$PROBE_TMP/$device_label.why_load"

    if [ -f "$sim_probe" ] \
        && { [ "$sim_implementation" = none ] \
          || [ -f "$sim_implementation" ]; }; then
      link_or_record \
        "$sim_label" simulator "$sim_probe" "$sim_implementation" \
        "$SIM_ARCHIVE" "$sim_executable" "$sim_why_load" \
        "$PROBE_TMP/$sim_label.map" || true
      if [ "$implementation_kind" = archive ] \
          && [ -f "$sim_executable" ] \
          && ! grep -Fq 'MeshShape_ios.cpp.o' "$sim_why_load"; then
        record_failure \
          "$sim_label ordinary archive link did not extract MeshShape_ios.cpp.o"
      fi
    fi

    if [ -f "$device_probe" ] \
        && { [ "$device_implementation" = none ] \
          || [ -f "$device_implementation" ]; }; then
      link_or_record \
        "$device_label" device "$device_probe" "$device_implementation" \
        "$DEVICE_ARCHIVE" "$device_executable" "$device_why_load" \
        "$PROBE_TMP/$device_label.map" || true
      if [ "$implementation_kind" = archive ] \
          && [ -f "$device_executable" ] \
          && ! grep -Fq 'MeshShape_ios.cpp.o' "$device_why_load"; then
        record_failure \
          "$device_label ordinary archive link did not extract MeshShape_ios.cpp.o"
      fi
    fi
  done
done

if [ -z "${SIMULATOR_UDID:-}" ]; then
  if ! SIMULATOR_UDID="$(
      xcrun simctl list devices -j | python3 -c '
import json, sys
for runtime, devices in json.load(sys.stdin)["devices"].items():
    if "iOS" not in runtime:
        continue
    for device in devices:
        if device.get("isAvailable") and device.get("name") == "BioMotion-CI":
            print(device["udid"])
            raise SystemExit(0)
raise SystemExit(1)
'
  )"; then
    SIMULATOR_UDID=''
    record_failure 'available BioMotion-CI simulator not found'
  fi
fi

if [ -n "$SIMULATOR_UDID" ]; then
  simulator_state="$(
    xcrun simctl list devices -j | python3 -c '
import json, sys
target = sys.argv[1]
for devices in json.load(sys.stdin)["devices"].values():
    for device in devices:
        if device.get("udid") == target and device.get("isAvailable"):
            print(device.get("state", ""))
            raise SystemExit(0)
raise SystemExit(1)
' "$SIMULATOR_UDID" 2>/dev/null || true
  )"
  if [ "$simulator_state" != Booted ] \
      && ! xcrun simctl bootstatus "$SIMULATOR_UDID" -b >/dev/null; then
    record_failure "could not boot simulator $SIMULATOR_UDID"
  fi
  for implementation_kind in fresh archive; do
    for probe_kind in mesh soft; do
      executable="$PROBE_TMP/${implementation_kind}_${probe_kind}_simulator"
      if [ ! -f "$executable" ]; then
        continue
      fi
      if [ "$probe_kind" = mesh ]; then
        sentinel=MESH_SHAPE_IOS_FAIL_CLOSED_PASS
      else
        sentinel=SOFT_MESH_SHAPE_IOS_FAIL_CLOSED_PASS
      fi
      run_or_record \
        "${implementation_kind}_${probe_kind}_simulator" \
        "$executable" "$sentinel"
    done
  done
fi

if [ "$failures" -ne 0 ]; then
  printf 'MeshShape iOS boundary probe found %s contract failure(s)\n' \
    "$failures" >&2
  exit 1
fi

printf '%s\n' 'MESH_SHAPE_IOS_BOUNDARY_PROBE_PASS'
