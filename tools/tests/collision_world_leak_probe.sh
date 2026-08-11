#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NIMBLE_ROOT="$REPO_ROOT/nimblephysics"
SOURCE="$SCRIPT_DIR/collision_world_leak_probe.cpp"
READY_TIMEOUT_SECONDS=15
LEAKS_TIMEOUT_SECONDS=30
TERMINATE_TIMEOUT_SECONDS=5
DEVTOOLS_SECURITY=/usr/sbin/DevToolsSecurity

if ! test -x "$DEVTOOLS_SECURITY"; then
  printf 'required command is unavailable: %s\n' "$DEVTOOLS_SECURITY" >&2
  exit 20
fi
if DEVTOOLS_STATUS="$($DEVTOOLS_SECURITY -status 2>&1)"; then
  :
else
  devtools_status_code="$?"
  printf '%s\n' "$DEVTOOLS_STATUS" >&2
  printf 'could not determine Developer Mode status (status %s)\n' \
    "$devtools_status_code" >&2
  exit 25
fi
if printf '%s\n' "$DEVTOOLS_STATUS" | grep -Eqi 'disabled'; then
  printf '%s\n' "$DEVTOOLS_STATUS" >&2
  printf '%s\n' \
    'Developer Mode is disabled; non-interactive leaks attachment to the resident probe is not authorized or reliable.' \
    'Ask an administrator to run sudo /usr/sbin/DevToolsSecurity -enable, then rerun this probe.' >&2
  exit 25
fi
if ! printf '%s\n' "$DEVTOOLS_STATUS" | grep -Eqi 'enabled'; then
  printf '%s\n' "$DEVTOOLS_STATUS" >&2
  printf '%s\n' 'Developer Mode status was not recognized; refusing an unbounded leaks attachment.' >&2
  exit 25
fi

for command_name in cmake leaks xcrun; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    printf 'required command is unavailable: %s\n' "$command_name" >&2
    exit 20
  fi
done

PROBE_TMP="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-world-leak.XXXXXX")"
PROBE_PID=""
LEAKS_PID=""

terminate_and_reap() {
  process_id="$1"
  if kill -0 "$process_id" 2>/dev/null; then
    kill -TERM "$process_id" 2>/dev/null || true
  fi
  termination_deadline=$((SECONDS + TERMINATE_TIMEOUT_SECONDS))
  while kill -0 "$process_id" 2>/dev/null; do
    if test "$SECONDS" -ge "$termination_deadline"; then
      kill -KILL "$process_id" 2>/dev/null || true
      break
    fi
    sleep 0.05
  done
  wait "$process_id" 2>/dev/null || true
}

cleanup() {
  if test -n "$LEAKS_PID"; then
    terminate_and_reap "$LEAKS_PID"
    LEAKS_PID=""
  fi
  if test -n "$PROBE_PID"; then
    terminate_and_reap "$PROBE_PID"
    PROBE_PID=""
  fi
  rm -r "$PROBE_TMP" 2>/dev/null || true
}

trap cleanup EXIT
trap 'exit 130' INT TERM HUP

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

start_probe() {
  process_log="$1"
  shift
  MallocStackLogging=1 "$EXECUTABLE" "$@" >"$process_log" 2>&1 &
  PROBE_PID="$!"
}

wait_for_ready() {
  sentinel="$1"
  process_log="$2"
  deadline=$((SECONDS + READY_TIMEOUT_SECONDS))

  while test "$SECONDS" -lt "$deadline"; do
    if grep -Fqx "$sentinel" "$process_log" 2>/dev/null; then
      return 0
    fi
    if ! kill -0 "$PROBE_PID" 2>/dev/null; then
      if wait "$PROBE_PID"; then
        process_status=0
      else
        process_status="$?"
      fi
      PROBE_PID=""
      cat "$process_log" >&2
      printf 'probe exited with status %s before sentinel: %s\n' \
        "$process_status" "$sentinel" >&2
      return 1
    fi
    sleep 0.05
  done

  cat "$process_log" >&2
  printf 'timed out after %s seconds waiting for sentinel: %s\n' \
    "$READY_TIMEOUT_SECONDS" "$sentinel" >&2
  return 1
}

snapshot_leaks() {
  snapshot_log="$1"
  leaks --quiet "$PROBE_PID" >"$snapshot_log" 2>&1 &
  LEAKS_PID="$!"
  deadline=$((SECONDS + LEAKS_TIMEOUT_SECONDS))

  while kill -0 "$LEAKS_PID" 2>/dev/null; do
    if test "$SECONDS" -ge "$deadline"; then
      terminate_and_reap "$LEAKS_PID"
      LEAKS_PID=""
      cat "$snapshot_log" >&2
      printf 'leaks snapshot timed out after %s seconds for PID %s\n' \
        "$LEAKS_TIMEOUT_SECONDS" "$PROBE_PID" >&2
      return 124
    fi
    sleep 0.05
  done

  if wait "$LEAKS_PID"; then
    LEAKS_STATUS=0
  else
    LEAKS_STATUS="$?"
  fi
  LEAKS_PID=""
}

stop_probe() {
  if ! kill -0 "$PROBE_PID" 2>/dev/null; then
    if wait "$PROBE_PID"; then
      process_status=0
    else
      process_status="$?"
    fi
    PROBE_PID=""
    printf 'resident probe exited before cleanup with status %s\n' \
      "$process_status" >&2
    return 1
  fi

  kill -TERM "$PROBE_PID"
  termination_deadline=$((SECONDS + TERMINATE_TIMEOUT_SECONDS))
  while kill -0 "$PROBE_PID" 2>/dev/null; do
    if test "$SECONDS" -ge "$termination_deadline"; then
      kill -KILL "$PROBE_PID" 2>/dev/null || true
      wait "$PROBE_PID" 2>/dev/null || true
      PROBE_PID=""
      printf 'resident probe did not terminate within %s seconds\n' \
        "$TERMINATE_TIMEOUT_SECONDS" >&2
      return 1
    fi
    sleep 0.05
  done
  if wait "$PROBE_PID"; then
    process_status=0
  else
    process_status="$?"
  fi
  PROBE_PID=""
  if test "$process_status" -ne 0; then
    printf 'resident probe cleanup returned status %s\n' \
      "$process_status" >&2
    return 1
  fi
}

if ! cmake \
    -S "$NIMBLE_ROOT/ios" \
    -B "$PROBE_TMP/build" \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DNIMBLE_IOS_HOST_PROBE=ON \
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
  -I"$PROBE_TMP/build" \
  -I"$NIMBLE_ROOT" \
  -I"$NIMBLE_ROOT/third_party/eigen" \
  "$SOURCE" \
  "$PROBE_TMP/build/libnimble_ios.a" \
  -Wl,-dead_strip \
  -o "$EXECUTABLE"

CONTROL_PROCESS_LOG="$PROBE_TMP/control-process.log"
CONTROL_LEAKS_LOG="$PROBE_TMP/control-leaks.log"
start_probe "$CONTROL_PROCESS_LOG" --positive-control
wait_for_ready 'WORLD_LEAK_POSITIVE_CONTROL_READY' "$CONTROL_PROCESS_LOG"
if snapshot_leaks "$CONTROL_LEAKS_LOG"; then
  :
else
  snapshot_status="$?"
  exit "$snapshot_status"
fi
CONTROL_OUTPUT="$(cat "$CONTROL_PROCESS_LOG" "$CONTROL_LEAKS_LOG")"
printf '%s\n' "$CONTROL_OUTPUT"
if test "$LEAKS_STATUS" -ne 1; then
  printf '%s\n' \
    "positive-control leaks snapshot returned unexpected status $LEAKS_STATUS (expected 1)" >&2
  exit 23
fi
require_output_pattern \
  '^WORLD_LEAK_POSITIVE_CONTROL_READY$' \
  "$CONTROL_OUTPUT" \
  'positive control did not enter its resident ready state'
require_output_pattern \
  'Process [0-9]+: [1-9][0-9]* leaks? for [1-9][0-9]* total leaked bytes\.' \
  "$CONTROL_OUTPUT" \
  'system leaks tool did not detect the positive-control allocation'
require_output_pattern \
  'ROOT LEAK: <malloc in .*leakForPositiveControl' \
  "$CONTROL_OUTPUT" \
  'positive-control leak report did not identify the deliberate allocation'
stop_probe

NORMAL_PROCESS_LOG="$PROBE_TMP/normal-process.log"
NORMAL_LEAKS_LOG="$PROBE_TMP/normal-leaks.log"
start_probe "$NORMAL_PROCESS_LOG"
wait_for_ready 'WORLD_COLLISION_REJECTION_NO_LEAK_READY' "$NORMAL_PROCESS_LOG"
if snapshot_leaks "$NORMAL_LEAKS_LOG"; then
  :
else
  snapshot_status="$?"
  exit "$snapshot_status"
fi
NORMAL_OUTPUT="$(cat "$NORMAL_PROCESS_LOG" "$NORMAL_LEAKS_LOG")"
printf '%s\n' "$NORMAL_OUTPUT"
if test "$LEAKS_STATUS" -ne 0; then
  printf 'World rejection leak probe failed with status %s\n' \
    "$LEAKS_STATUS" >&2
  exit "$LEAKS_STATUS"
fi
require_output_pattern \
  '^WORLD_COLLISION_REJECTION_NO_LEAK_READY$' \
  "$NORMAL_OUTPUT" \
  'World rejection probe did not enter its resident ready state'
require_output_pattern \
  'Process [0-9]+: 0 leaks for 0 total leaked bytes\.' \
  "$NORMAL_OUTPUT" \
  'World rejection paths were not proven leak-free'
if printf '%s\n' "$NORMAL_OUTPUT" | grep -Eq 'ROOT LEAK|[1-9][0-9]* leaks? for'; then
  printf '%s\n' 'World rejection leak probe contains a leak report' >&2
  exit 24
fi
stop_probe

printf '%s\n' 'WORLD_COLLISION_REJECTION_LEAK_PROBE_PASS'
