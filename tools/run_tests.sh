#!/bin/bash
#
# Fail-closed XCTest runner. Every lane owns its selection and produces a
# structured xcresult receipt; a green xcodebuild line alone is never a pass.
#
# Usage:
#   tools/run_tests.sh fast
#       All reviewed tests except the >1-hour E1 experiment. Exact count: 643.
#
#   tools/run_tests.sh slow
#       E1MarkerSetComparisonTests/testE1RunAll only. Exact count: 1.
#
#   tools/run_tests.sh subset -only-testing:TEST-ID [diagnostic xcodebuild args...]
#       Debugging only. Requires at least one selected test and prints SUBSET
#       PASS rather than a commit-gate receipt.
#
#   tools/run_tests.sh all
#       Runs fast, then slow, and succeeds only if both receipts pass.
#
# The fast/slow/all lanes accept no caller arguments: their invocation is part
# of the reviewed receipt. The diagnostic subset lane rejects skip-testing,
# alternate test plans/configurations, retry/repetition controls, and
# runner-owned project/scheme/destination/result paths. Do not add a skip or
# retry to make a lane green.

# shellcheck source-path=SCRIPTDIR

set -u

RUN_TESTS_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=test_gate.sh
source "$RUN_TESTS_SCRIPT_DIR/test_gate.sh"

DEVICE_NAME='BioMotion-CI'
DEVICE_TYPE='iPhone 17'
LOCK_DIR="${TMPDIR:-/tmp}/biomotion-run-tests.lock"
REPO_ROOT="$(cd "$RUN_TESTS_SCRIPT_DIR/.." && pwd)"
TEST_DEVICE_UDID=''
RUN_OUTPUT_DIR=''

run_tests_usage() {
  cat >&2 <<'EOF'
Usage:
  tools/run_tests.sh fast
  tools/run_tests.sh slow
  tools/run_tests.sh subset -only-testing:TEST-ID [diagnostic xcodebuild args...]
  tools/run_tests.sh all

fast, slow, and all accept no caller arguments. subset accepts -only-testing
selectors plus diagnostic build arguments, but no skip, alternate test
plan/configuration, retry/repetition control, or override for the project,
scheme, destination, or result bundle.
EOF
}

run_tests_release_lock() {
  rmdir "$LOCK_DIR" 2>/dev/null || true
}

run_tests_acquire_lock() {
  if ! mkdir "$LOCK_DIR" 2>/dev/null; then
    printf 'REFUSING: %s exists, so another run is using %s.\n' "$LOCK_DIR" "$DEVICE_NAME"
    printf '%s\n' 'Two xcodebuild test processes on one simulator can evict each other'
    printf '%s\n' 'and report an incomplete suite. Wait, or remove the lock only after'
    printf 'confirming no run is live: rmdir %s\n' "$LOCK_DIR"
    return 2
  fi
  trap run_tests_release_lock EXIT
  return 0
}

run_tests_resolve_device() {
  local devices_json
  if ! devices_json=$(xcrun simctl list devices -j); then
    printf 'GATE FAIL: could not enumerate simulators\n' >&2
    return 1
  fi

  TEST_DEVICE_UDID=$(printf '%s' "$devices_json" | python3 -c "
import json,sys
d = json.load(sys.stdin)['devices']
for runtime, devices in d.items():
    if 'iOS' not in runtime: continue
    for dev in devices:
        if dev['name'] == '$DEVICE_NAME' and dev['isAvailable']:
            print(dev['udid']); sys.exit(0)
") || {
    printf 'GATE FAIL: could not parse simulator inventory\n' >&2
    return 1
  }

  if [ -z "$TEST_DEVICE_UDID" ]; then
    printf "Creating dedicated simulator '%s' (%s)...\n" "$DEVICE_NAME" "$DEVICE_TYPE"
    TEST_DEVICE_UDID=$(xcrun simctl create "$DEVICE_NAME" "$DEVICE_TYPE") || return 1
  fi

  if ! xcrun simctl bootstatus "$TEST_DEVICE_UDID" -b >/dev/null 2>&1; then
    printf 'GATE FAIL: could not boot dedicated simulator %s (%s)\n' \
      "$DEVICE_NAME" "$TEST_DEVICE_UDID" >&2
    return 1
  fi
  printf 'Device: %s (%s)\n' "$DEVICE_NAME" "$TEST_DEVICE_UDID"
  return 0
}

run_tests_print_failures() {
  local log_path=$1
  printf '\nFailures and skips (first 40):\n'
  grep -E 'error:|XCTAssert.* failed|Test Case .* skipped' "$log_path" | head -40
}

# Build the invocation in an array that is non-empty from its first expansion.
# macOS ships Bash 3.2, where expanding an EMPTY array under `set -u` raises
# "unbound variable". The subset lane has no runner-owned selector, so the old
# `selection_args=(); "${selection_args[@]}"` form died before xcodebuild.
run_tests_invoke_xcodebuild() {
  if [ "$#" -lt 2 ]; then
    printf 'run_tests_invoke_xcodebuild requires SELECTOR RESULT_PATH [ARGS...]\n' >&2
    return 2
  fi
  local selector=$1
  local result_path=$2
  shift 2

  local xcodebuild_args=(
    -project BioMotion.xcodeproj
    -scheme BioMotion
    -destination "platform=iOS Simulator,id=${TEST_DEVICE_UDID}"
    -resultBundlePath "$result_path"
  )
  if [ -n "$selector" ]; then
    xcodebuild_args+=("$selector")
  fi
  xcodebuild_args+=("$@" test)
  xcodebuild "${xcodebuild_args[@]}"
}

run_tests_one_lane() {
  local lane=$1
  shift

  local expected
  local selector
  expected=$(test_gate_expected_count "$lane") || return 2
  selector=$(test_gate_lane_selector "$lane") || return 2

  local lane_dir="$RUN_OUTPUT_DIR/$lane"
  local log_path="$lane_dir/xcodebuild.log"
  local result_path="$lane_dir/result.xcresult"
  local summary_path="$lane_dir/summary.json"
  local summary_error_path="$lane_dir/summary.stderr"
  mkdir -p "$lane_dir" || return 1

  printf '\nRunning %s lane (expected tests: %s)...\n' "$lane" "$expected"
  printf 'log:      %s\n' "$log_path"
  printf 'xcresult: %s\n' "$result_path"

  local start
  local wall
  local xcodebuild_rc
  start=$(date +%s)
  run_tests_invoke_xcodebuild "$selector" "$result_path" "$@" \
    >"$log_path" 2>&1
  xcodebuild_rc=$?
  wall=$(( $(date +%s) - start ))

  local summary_rc=1
  if [ -d "$result_path" ]; then
    xcrun xcresulttool get test-results summary \
      --path "$result_path" --compact \
      >"$summary_path" 2>"$summary_error_path"
    summary_rc=$?
  else
    printf 'result bundle was not created: %s\n' "$result_path" >"$summary_error_path"
  fi

  printf 'wall:     %ss\n' "$wall"
  printf 'summary:  %s (xcresulttool rc=%s)\n' "$summary_path" "$summary_rc"

  local gate_rc
  test_gate_evaluate "$lane" "$expected" "$xcodebuild_rc" "$summary_rc" \
    "$log_path" "$summary_path"
  gate_rc=$?

  if [ "$summary_rc" -ne 0 ]; then
    printf '\nxcresult summary error:\n'
    head -30 "$summary_error_path"
  fi
  if [ "$gate_rc" -ne 0 ]; then
    run_tests_print_failures "$log_path"
  fi
  return "$gate_rc"
}

run_tests_main() {
  if [ "$#" -eq 0 ]; then
    run_tests_usage
    return 2
  fi
  case "$1" in
    -h|--help)
      run_tests_usage
      return 0
      ;;
  esac

  local lane=$1
  shift
  case "$lane" in
    fast|slow|subset|all) ;;
    *)
      printf 'unknown test lane: %s\n' "$lane" >&2
      run_tests_usage
      return 2
      ;;
  esac

  if ! test_gate_validate_lane_args "$lane" "$@"; then
    run_tests_usage
    return 2
  fi

  local required_command
  for required_command in xcrun xcodebuild python3; do
    if ! command -v "$required_command" >/dev/null 2>&1; then
      printf 'GATE FAIL: required command is unavailable: %s\n' "$required_command" >&2
      return 1
    fi
  done

  cd "$REPO_ROOT" || return 1
  test_gate_assert_no_xctskip "$REPO_ROOT/BioMotionTests" || return 1
  run_tests_acquire_lock || return $?
  run_tests_resolve_device || return 1

  RUN_OUTPUT_DIR=$(mktemp -d "${TMPDIR:-/tmp}/biomotion-tests.XXXXXX") || return 1
  printf 'artifacts: %s\n' "$RUN_OUTPUT_DIR"

  if [ "$lane" = all ]; then
    if ! run_tests_one_lane fast "$@"; then
      printf 'ALL GATE FAIL: fast lane did not pass; slow lane was not started.\n'
      return 1
    fi
    if ! run_tests_one_lane slow "$@"; then
      printf 'ALL GATE FAIL: slow lane did not pass.\n'
      return 1
    fi
    printf 'ALL GATE PASS: fast and slow receipts both passed.\n'
    return 0
  fi

  run_tests_one_lane "$lane" "$@"
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  run_tests_main "$@"
  exit $?
fi
