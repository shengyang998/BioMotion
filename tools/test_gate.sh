#!/bin/bash

# Pure policy and receipt evaluation for tools/run_tests.sh.
#
# This file deliberately does not create a simulator or run xcodebuild. It can
# be sourced by the runner and by tools/tests/run_tests_gate_tests.sh, or called
# directly as:
#
#   tools/test_gate.sh LANE EXPECTED XCODEBUILD_RC SUMMARY_TOOL_RC LOG SUMMARY_JSON
#
# SUMMARY_JSON is the output of:
#   xcrun xcresulttool get test-results summary --compact --path RESULT.xcresult

TEST_GATE_E1_CLASS='BioMotionTests/E1MarkerSetComparisonTests'
TEST_GATE_E1_TEST="${TEST_GATE_E1_CLASS}/testE1RunAll"

test_gate_assert_no_xctskip() {
  if [ "$#" -ne 1 ] || [ ! -d "$1" ]; then
    printf 'XCTSkip policy requires a BioMotionTests directory\n' >&2
    return 2
  fi

  python3 - "$1" <<'PY'
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
pattern = re.compile(r"\bXCTSkip(?:If|Unless)?\s*\(")
hits = []
try:
    for path in sorted(root.rglob("*")):
        if path.suffix not in {".swift", ".m", ".mm"} or not path.is_file():
            continue
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if pattern.search(line):
                hits.append(f"{path}:{line_number}:{line.strip()}")
except OSError as error:
    print(f"GATE FAIL: could not inspect XCTest sources: {error}", file=sys.stderr)
    sys.exit(2)

if hits:
    print("GATE FAIL: BioMotionTests may not skip required tests:", file=sys.stderr)
    print("\n".join(hits), file=sys.stderr)
    sys.exit(1)
PY
}

test_gate_lane_selector() {
  case "${1-}" in
    fast)
      printf '%s\n' "-skip-testing:${TEST_GATE_E1_CLASS}"
      ;;
    slow)
      printf '%s\n' "-only-testing:${TEST_GATE_E1_TEST}"
      ;;
    subset|all)
      ;;
    *)
      printf 'unknown test lane: %s\n' "${1-<missing>}" >&2
      return 2
      ;;
  esac
}

test_gate_expected_count() {
  case "${1-}" in
    fast) printf '%s\n' 524 ;;
    slow) printf '%s\n' 1 ;;
    subset) printf '%s\n' 1 ;;
    *)
      printf 'lane %s has no single expected count\n' "${1-<missing>}" >&2
      return 2
      ;;
  esac
}

test_gate_validate_lane_args() {
  local lane=${1-}
  if [ "$#" -gt 0 ]; then shift; fi

  case "$lane" in
    fast|slow|subset|all) ;;
    *)
      printf 'unknown test lane: %s\n' "${lane:-<missing>}" >&2
      return 2
      ;;
  esac

  # The receipt is reviewed for one fixed invocation, not for an open-ended
  # xcodebuild command line. A denylist cannot anticipate every future option
  # that changes selection, repetitions, build configuration, or result
  # semantics, so gating lanes accept no caller arguments at all. `subset` is
  # explicitly non-gating and owns the diagnostic escape hatch below.
  if [ "$lane" != subset ] && [ "$#" -gt 0 ]; then
    printf '%s lane accepts no caller arguments; use subset for diagnostics\n' "$lane" >&2
    return 2
  fi

  local saw_only=0
  local needs_only_value=0
  local arg
  for arg in "$@"; do
    if [ "$needs_only_value" -eq 1 ]; then
      case "$arg" in
        ''|-*)
          printf 'subset -only-testing requires a test identifier value\n' >&2
          return 2
          ;;
        *)
          saw_only=1
          needs_only_value=0
          continue
          ;;
      esac
    fi

    case "$arg" in
      -only-testing)
        if [ "$lane" != subset ]; then
          printf '%s owns its test selection; remove caller-provided %s\n' "$lane" "$arg" >&2
          return 2
        fi
        needs_only_value=1
        ;;
      -only-testing:*|-only-testing=*)
        if [ "$lane" != subset ]; then
          printf '%s owns its test selection; remove caller-provided %s\n' "$lane" "$arg" >&2
          return 2
        fi
        if [ "$arg" = '-only-testing:' ] || [ "$arg" = '-only-testing=' ]; then
          printf 'subset -only-testing selector must not be empty\n' >&2
          return 2
        fi
        saw_only=1
        ;;
      -skip-testing|-skip-testing:*|-skip-testing=*|\
      -testPlan|-testPlan:*|-testPlan=*|\
      -only-test-configuration|-only-test-configuration:*|-only-test-configuration=*|\
      -skip-test-configuration|-skip-test-configuration:*|-skip-test-configuration=*|\
      --only-test-configuration|--only-test-configuration:*|--only-test-configuration=*|\
      --skip-test-configuration|--skip-test-configuration:*|--skip-test-configuration=*)
        printf '%s lane refuses caller-provided selection argument %s\n' "$lane" "$arg" >&2
        return 2
        ;;
      -test-iterations|-test-iterations:*|-test-iterations=*|\
      --test-iterations|--test-iterations:*|--test-iterations=*|\
      -test-repetitions|-test-repetitions:*|-test-repetitions=*|\
      --test-repetitions|--test-repetitions:*|--test-repetitions=*|\
      -retry-tests-on-failure|-retry-tests-on-failure:*|-retry-tests-on-failure=*|\
      --retry-tests-on-failure|--retry-tests-on-failure:*|--retry-tests-on-failure=*|\
      -run-tests-until-failure|-run-tests-until-failure:*|-run-tests-until-failure=*|\
      --run-tests-until-failure|--run-tests-until-failure:*|--run-tests-until-failure=*|\
      -test-repetition-relaunch-enabled|-test-repetition-relaunch-enabled:*|\
      -test-repetition-relaunch-enabled=*|--test-repetition-relaunch-enabled|\
      --test-repetition-relaunch-enabled:*|--test-repetition-relaunch-enabled=*)
        printf '%s lane refuses retry/repetition argument %s\n' "$lane" "$arg" >&2
        return 2
        ;;
      -project|-project=*|-project:*|\
      -workspace|-workspace=*|-workspace:*|\
      -scheme|-scheme=*|-scheme:*|\
      -destination|-destination=*|-destination:*|\
      -resultBundlePath|-resultBundlePath=*|-resultBundlePath:*|\
      -xctestrun|-xctestrun=*|-xctestrun:*|\
      -testProductsPath|-testProductsPath=*|-testProductsPath:*|\
      -enumerate-tests)
        printf '%s lane refuses runner-owned argument %s\n' "$lane" "$arg" >&2
        return 2
        ;;
    esac
  done

  if [ "$needs_only_value" -eq 1 ]; then
    printf 'subset -only-testing requires a test identifier value\n' >&2
    return 2
  fi
  if [ "$lane" = subset ] && [ "$saw_only" -eq 0 ]; then
    printf 'subset lane requires at least one -only-testing selector\n' >&2
    return 2
  fi
  return 0
}

test_gate_evaluate() {
  if [ "$#" -ne 6 ]; then
    printf '%s\n' \
      'usage: test_gate_evaluate LANE EXPECTED XCODEBUILD_RC SUMMARY_TOOL_RC LOG SUMMARY_JSON' >&2
    return 2
  fi

  local lane=$1
  local expected=$2
  local xcodebuild_rc=$3
  local summary_tool_rc=$4
  local log_path=$5
  local summary_path=$6

  case "$lane" in
    fast|slow|subset) ;;
    *)
      printf 'unknown test lane: %s\n' "$lane" >&2
      return 2
      ;;
  esac
  case "$expected" in
    ''|*[!0-9]*)
      printf 'expected test count is not a non-negative integer: %s\n' "$expected" >&2
      return 2
      ;;
  esac
  if [ "$expected" -eq 0 ]; then
    printf 'expected test count must be greater than zero\n' >&2
    return 2
  fi
  case "$xcodebuild_rc" in
    ''|*[!0-9]*)
      printf 'xcodebuild rc is not a non-negative integer: %s\n' "$xcodebuild_rc" >&2
      return 2
      ;;
  esac
  case "$summary_tool_rc" in
    ''|*[!0-9]*)
      printf 'xcresulttool rc is not a non-negative integer: %s\n' "$summary_tool_rc" >&2
      return 2
      ;;
  esac

  if [ ! -f "$log_path" ]; then
    printf 'GATE FAIL: xcodebuild log is missing: %s\n' "$log_path"
    return 1
  fi
  if [ ! -f "$summary_path" ]; then
    printf 'GATE FAIL: xcresult summary is missing: %s\n' "$summary_path"
    return 1
  fi

  local summary_fields
  summary_fields=$(python3 - "$summary_path" <<'PY'
import json
import sys

path = sys.argv[1]
required_ints = (
    "totalTestCount",
    "passedTests",
    "failedTests",
    "skippedTests",
    "expectedFailures",
)
try:
    with open(path, "r", encoding="utf-8") as handle:
        summary = json.load(handle)
    for key in required_ints:
        if type(summary.get(key)) is not int or summary[key] < 0:
            raise ValueError(f"{key} is missing or is not a non-negative integer")
    result = summary.get("result")
    if not isinstance(result, str):
        raise ValueError("result is missing or is not a string")
except (OSError, json.JSONDecodeError, ValueError) as error:
    print(f"xcresult summary parse error: {error}", file=sys.stderr)
    sys.exit(1)

print("|".join(str(summary[key]) for key in required_ints) + "|" + result)
PY
  )
  local summary_rc=$?
  if [ "$summary_rc" -ne 0 ]; then
    printf 'GATE FAIL: xcresult summary is malformed or unreadable\n'
    return 1
  fi

  local total
  local passed
  local failed
  local skipped
  local expected_failures
  local result
  local old_ifs=$IFS
  IFS='|'
  read -r total passed failed skipped expected_failures result <<EOF
$summary_fields
EOF
  IFS=$old_ifs

  local restarts
  local verdict
  restarts=$(grep -c 'Restarting after unexpected exit' "$log_path")
  verdict=$(grep -E '^\*\* TEST (SUCCEEDED|FAILED) \*\*$' "$log_path" | tail -1)

  printf 'lane:              %s\n' "$lane"
  printf 'xcodebuild rc:     %s\n' "$xcodebuild_rc"
  printf 'xcresulttool rc:   %s\n' "$summary_tool_rc"
  printf 'xcresult result:   %s\n' "$result"
  printf 'total/passed:      %s/%s\n' "$total" "$passed"
  printf 'failed/skipped:    %s/%s\n' "$failed" "$skipped"
  printf 'expected failures: %s\n' "$expected_failures"
  printf 'restarts:          %s\n' "$restarts"
  printf 'verdict:           %s\n' "${verdict:-<none>}"

  local gate_failed=0
  if [ "$xcodebuild_rc" -ne 0 ]; then
    printf 'GATE FAIL: xcodebuild exited %s\n' "$xcodebuild_rc"
    gate_failed=1
  fi
  if [ "$summary_tool_rc" -ne 0 ]; then
    printf 'GATE FAIL: xcresulttool exited %s\n' "$summary_tool_rc"
    gate_failed=1
  fi
  if [ "$verdict" != '** TEST SUCCEEDED **' ]; then
    printf 'GATE FAIL: textual verdict is not TEST SUCCEEDED\n'
    gate_failed=1
  fi
  if [ "$restarts" -ne 0 ]; then
    printf 'GATE FAIL: %s test host(s) restarted; the run is incomplete\n' "$restarts"
    gate_failed=1
  fi
  if [ "$result" != Passed ]; then
    printf 'GATE FAIL: xcresult result is %s, expected case-sensitive Passed\n' "$result"
    gate_failed=1
  fi
  if [ "$failed" -ne 0 ]; then
    printf 'GATE FAIL: %s test(s) failed\n' "$failed"
    gate_failed=1
  fi
  if [ "$skipped" -ne 0 ]; then
    printf 'GATE FAIL: %s test(s) skipped; required tests may not disappear\n' "$skipped"
    gate_failed=1
  fi
  if [ "$expected_failures" -ne 0 ]; then
    printf 'GATE FAIL: %s expected failure(s) are not passes\n' "$expected_failures"
    gate_failed=1
  fi
  if [ "$passed" -ne "$total" ]; then
    printf 'GATE FAIL: passed %s of %s total tests\n' "$passed" "$total"
    gate_failed=1
  fi

  if [ "$lane" = subset ]; then
    if [ "$total" -lt "$expected" ]; then
      printf 'GATE FAIL: subset ran %s tests; at least %s must run\n' "$total" "$expected"
      gate_failed=1
    fi
  elif [ "$total" -ne "$expected" ]; then
    printf 'GATE FAIL: %s ran %s tests; exact reviewed count is %s\n' \
      "$lane" "$total" "$expected"
    gate_failed=1
  fi

  if [ "$gate_failed" -ne 0 ]; then
    printf 'GATE FAIL -- do not treat this run as evidence\n'
    return 1
  fi

  if [ "$lane" = subset ]; then
    printf 'SUBSET PASS -- debugging evidence only, not a commit gate\n'
  else
    printf '%s GATE PASS\n' "$(printf '%s' "$lane" | tr '[:lower:]' '[:upper:]')"
  fi
  return 0
}

test_gate_usage() {
  printf 'usage: %s LANE EXPECTED XCODEBUILD_RC SUMMARY_TOOL_RC LOG SUMMARY_JSON\n' \
    "${0##*/}" >&2
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  if [ "$#" -ne 6 ]; then
    test_gate_usage
    exit 2
  fi
  test_gate_evaluate "$@"
  exit $?
fi
