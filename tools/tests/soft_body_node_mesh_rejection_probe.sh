#!/bin/bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NIMBLE_ROOT="$REPO_ROOT/nimblephysics"
SOURCE="$SCRIPT_DIR/soft_body_node_mesh_rejection_probe.cpp"
SOFT_BODY_SOURCE="$NIMBLE_ROOT/dart/dynamics/SoftBodyNode.cpp"
PROBE_TMP="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-soft-transaction.XXXXXX")"

CONFIGURE_LOG="$PROBE_TMP/configure.log"
BUILD_LOG="$PROBE_TMP/build.log"
HOST_ARCHIVE="$PROBE_TMP/build/libnimble_ios.a"
INJECTED_ARCHIVE="$PROBE_TMP/libnimble_without_mesh.a"
EXECUTABLE="$PROBE_TMP/soft_body_transaction"
WHY_LOAD="$PROBE_TMP/soft_body_transaction.why_load"
MAP="$PROBE_TMP/soft_body_transaction.map"
failures=0

cleanup() {
  rm -r "$PROBE_TMP" 2>/dev/null || true
}
trap cleanup EXIT

record_failure() {
  printf 'FAIL %s\n' "$1" >&2
  failures=$((failures + 1))
}

require_output_pattern() {
  pattern=$1
  output=$2
  diagnostic=$3
  if ! printf '%s\n' "$output" | grep -Eq "$pattern"; then
    record_failure "$diagnostic"
  fi
}

run_transaction() {
  label=$1
  argument=$2
  exit_sentinel=$3
  process_log="$PROBE_TMP/${label}.process.log"
  if "$EXECUTABLE" "$argument" >"$process_log" 2>&1; then
    transaction_status=0
  else
    transaction_status=$?
  fi
  cat "$process_log"
  if [ "$transaction_status" -ne 0 ]; then
    record_failure "$label transaction returned status $transaction_status"
  elif ! grep -Fxq "$exit_sentinel" "$process_log"; then
    record_failure "$label transaction did not reach exit"
  fi
}

for command_name in ar cmake nm xcrun; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    record_failure "required command is unavailable: $command_name"
  fi
done
for required_file in "$SOURCE" "$SOFT_BODY_SOURCE"; do
  if [ ! -f "$required_file" ]; then
    record_failure "required file is missing: $required_file"
  fi
done
if [ "$failures" -ne 0 ]; then
  exit 1
fi

notifier_count="$(
  grep -Fc 'mNotifier = new PointMassNotifier' "$SOFT_BODY_SOURCE" || true
)"
mesh_count="$(
  grep -Fc 'std::make_shared<SoftMeshShape>' "$SOFT_BODY_SOURCE" || true
)"
notifier_line="$(
  grep -Fn 'mNotifier = new PointMassNotifier' "$SOFT_BODY_SOURCE" \
    | cut -d: -f1
)"
mesh_line="$(
  grep -Fn 'std::make_shared<SoftMeshShape>' "$SOFT_BODY_SOURCE" \
    | cut -d: -f1
)"
if [ "$notifier_count" -ne 1 ] || [ "$mesh_count" -ne 1 ]; then
  record_failure \
    'SoftBodyNode constructor must have one notifier allocation and one SoftMesh construction'
elif [ "$notifier_line" -ge "$mesh_line" ]; then
  record_failure \
    'SoftBodyNode no longer preserves the upstream notifier-before-mesh success ordering'
fi
if [ "$(grep -Fc 'mNotifier(nullptr)' "$SOFT_BODY_SOURCE" || true)" -ne 1 ] \
    || [ "$(grep -Fc 'delete mNotifier;' "$SOFT_BODY_SOURCE" || true)" -ne 2 ]; then
  record_failure \
    'SoftBodyNode must initialize the notifier and clean it in rejection plus destruction paths'
fi

SIM_SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"
SIM_CXX="$(xcrun --sdk iphonesimulator --find clang++)"
DEVICE_SDK="$(xcrun --sdk iphoneos --show-sdk-path)"
DEVICE_CXX="$(xcrun --sdk iphoneos --find clang++)"
COMMON=(
  -std=c++17
  -DDART_IOS_BUILD=1
  -DDART_USE_IDENTITY_JACOBIAN=1
  -DEIGEN_DONT_PARALLELIZE
  -Wno-deprecated-literal-operator
  -I"$NIMBLE_ROOT"
  -I"$NIMBLE_ROOT/third_party/eigen"
  -I"$NIMBLE_ROOT/third_party/tinyxml2"
  -Wall
  -Wextra
  -Werror
  -Wno-sign-compare
)
if ! "$SIM_CXX" \
    -target arm64-apple-ios17.0-simulator \
    -isysroot "$SIM_SDK" \
    "${COMMON[@]}" \
    -fsyntax-only "$SOURCE" \
    >"$PROBE_TMP/simulator-compile.log" 2>&1; then
  cat "$PROBE_TMP/simulator-compile.log" >&2
  record_failure 'SoftBodyNode transaction probe did not compile for simulator'
fi
if ! "$DEVICE_CXX" \
    -target arm64-apple-ios17.0 \
    -isysroot "$DEVICE_SDK" \
    "${COMMON[@]}" \
    -fsyntax-only "$SOURCE" \
    >"$PROBE_TMP/device-compile.log" 2>&1; then
  cat "$PROBE_TMP/device-compile.log" >&2
  record_failure 'SoftBodyNode transaction probe did not compile for device'
fi

if ! cmake \
    -S "$NIMBLE_ROOT" \
    -B "$PROBE_TMP/build" \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    >"$CONFIGURE_LOG" 2>&1; then
  cat "$CONFIGURE_LOG" >&2
  record_failure 'host transaction-probe configure failed'
elif ! cmake --build "$PROBE_TMP/build" \
    --target nimble_ios \
    --parallel \
    >"$BUILD_LOG" 2>&1; then
  cat "$BUILD_LOG" >&2
  record_failure 'host transaction-probe native build failed'
fi

if [ ! -f "$HOST_ARCHIVE" ]; then
  record_failure 'host transaction-probe archive is missing'
else
  cp "$HOST_ARCHIVE" "$INJECTED_ARCHIVE"
  if [ "$(ar -t "$INJECTED_ARCHIVE" \
      | grep -Fxc 'MeshShape_ios.cpp.o' || true)" -ne 1 ]; then
    record_failure 'host archive does not contain exactly one MeshShape stub'
  elif ! ar -d "$INJECTED_ARCHIVE" MeshShape_ios.cpp.o; then
    record_failure 'could not remove MeshShape stub for fault injection'
  elif ar -t "$INJECTED_ARCHIVE" | grep -Fxq 'MeshShape_ios.cpp.o'; then
    record_failure 'fault-injection archive still contains MeshShape stub'
  fi
fi

HOST_CXX="$(xcrun --sdk macosx --find clang++)"
HOST_SDK="$(xcrun --sdk macosx --show-sdk-path)"
if [ -f "$INJECTED_ARCHIVE" ]; then
  if ! "$HOST_CXX" \
      -isysroot "$HOST_SDK" \
      -O0 \
      -g \
      "${COMMON[@]}" \
      "$SOURCE" \
      "$INJECTED_ARCHIVE" \
      -fsanitize=address \
      -fno-omit-frame-pointer \
      -Wl,-dead_strip \
      -Wl,-why_load \
      -Wl,-map,"$MAP" \
      -o "$EXECUTABLE" \
      2>"$WHY_LOAD"; then
    cat "$WHY_LOAD" >&2
    record_failure 'fault-injected host transaction probe did not link'
  fi
fi

if [ -f "$EXECUTABLE" ]; then
  if ! grep -Fq 'SoftBodyNode.cpp.o' "$WHY_LOAD" \
      || ! grep -Fq 'SoftBodyNode.cpp.o' "$MAP"; then
    record_failure 'transaction link did not extract SoftBodyNode.cpp.o'
  fi
  if grep -Fq 'MeshShape_ios.cpp.o' "$WHY_LOAD" "$MAP"; then
    record_failure 'transaction link used the production MeshShape stub'
  fi
  nm -gu "$EXECUTABLE" | c++filt >"$PROBE_TMP/final.undefined"
  if grep -Fq 'dart::' "$PROBE_TMP/final.undefined"; then
    record_failure 'transaction executable retains an unresolved DART symbol'
  fi

  run_transaction \
    root --root-only SOFT_BODY_ROOT_REJECTION_TRANSACTION_REACHED_EXIT
  run_transaction \
    child --child-only SOFT_BODY_CHILD_REJECTION_TRANSACTION_REACHED_EXIT

  if CONTROL_OUTPUT="$(
      ASAN_OPTIONS='detect_leaks=0:halt_on_error=1:exitcode=87' \
        "$EXECUTABLE" --positive-control 2>&1
  )"; then
    control_status=0
  else
    control_status=$?
  fi
  printf '%s\n' "$CONTROL_OUTPUT"
  if [ "$control_status" -ne 86 ]; then
    record_failure \
      "AddressSanitizer positive control returned $control_status instead of 86"
  fi
  require_output_pattern \
    '^SOFT_BODY_ALLOCATION_POSITIVE_CONTROL allocated=1 freed=0 live=1$' \
    "$CONTROL_OUTPUT" \
    'allocation tracker did not detect its deliberate live allocation'

  if NORMAL_OUTPUT="$(
      ASAN_OPTIONS='detect_leaks=0:halt_on_error=1:exitcode=87' \
        "$EXECUTABLE" 2>&1
  )"; then
    normal_status=0
  else
    normal_status=$?
  fi
  printf '%s\n' "$NORMAL_OUTPUT"
  if [ "$normal_status" -ne 0 ]; then
    record_failure "SoftBodyNode rejection sanitizer probe returned $normal_status"
  fi
  require_output_pattern \
    '^SOFT_BODY_MESH_REJECTION_TRANSACTION_REACHED_EXIT$' \
    "$NORMAL_OUTPUT" \
    'SoftBodyNode rejection sanitizer probe did not reach process exit'
  require_output_pattern \
    '^SOFT_BODY_ROOT_ALLOCATION_TRANSACTIONS 32$' \
    "$NORMAL_OUTPUT" \
    'root notifier allocation transactions were not proven balanced'
  require_output_pattern \
    '^SOFT_BODY_CHILD_ALLOCATION_TRANSACTIONS 32$' \
    "$NORMAL_OUTPUT" \
    'child notifier allocation transactions were not proven balanced'
  if printf '%s\n' "$NORMAL_OUTPUT" \
      | grep -Eq 'AddressSanitizer|LeakSanitizer|runtime error:'; then
    record_failure 'SoftBodyNode rejection sanitizer probe reported an error'
  fi
fi

if [ "$failures" -ne 0 ]; then
  printf 'SoftBodyNode rejection transaction probe found %s failure(s)\n' \
    "$failures" >&2
  exit 1
fi

printf '%s\n' 'SOFT_BODY_MESH_REJECTION_TRANSACTION_PASS'
