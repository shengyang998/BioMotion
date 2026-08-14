#!/bin/bash -p

# Fast, simulator-free regression tests for the commit-gate policy. These use
# synthetic xcresult summaries and xcodebuild logs so every fail-closed branch
# is exercised without compiling or launching XCTest.

# shellcheck source-path=SCRIPTDIR

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# shellcheck source=../test_gate.sh
source "$REPO_ROOT/tools/test_gate.sh"
# shellcheck source=../run_tests.sh
source "$REPO_ROOT/tools/run_tests.sh"

TEST_TMP=$(mktemp -d "${TMPDIR:-/tmp}/biomotion-gate-tests.XXXXXX") || exit 1
trap 'rm -r "$TEST_TMP" 2>/dev/null || true' EXIT

PASS_COUNT=0
FAIL_COUNT=0

record_pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  printf 'PASS %s\n' "$1"
}

record_fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  printf 'FAIL %s\n%s\n' "$1" "$2"
}

expect_status() {
  local name=$1
  local expected=$2
  shift 2

  local output
  local actual
  output=$("$@" 2>&1)
  actual=$?
  if [ "$actual" -eq "$expected" ]; then
    record_pass "$name"
  else
    record_fail "$name" "expected rc=$expected, got rc=$actual\n$output"
  fi
}

expect_output() {
  local name=$1
  local expected=$2
  shift 2

  local output
  local actual
  output=$("$@" 2>&1)
  actual=$?
  if [ "$actual" -eq 0 ] && [ "$output" = "$expected" ]; then
    record_pass "$name"
  else
    record_fail "$name" "expected rc=0/output='$expected', got rc=$actual/output='$output'"
  fi
}

write_log() {
  local path=$1
  local verdict=$2
  local restarts=$3
  local i=0

  : > "$path"
  while [ "$i" -lt "$restarts" ]; do
    printf '%s\n' 'Restarting after unexpected exit, crash, or test timeout' >> "$path"
    i=$((i + 1))
  done
  if [ -n "$verdict" ]; then
    printf '%s\n' "$verdict" >> "$path"
  fi
}

write_summary() {
  local path=$1
  local total=$2
  local passed=$3
  local failed=$4
  local skipped=$5
  local expected_failures=$6
  local result=$7

  printf '%s\n' \
    "{\"totalTestCount\":$total,\"passedTests\":$passed,\"failedTests\":$failed,\"skippedTests\":$skipped,\"expectedFailures\":$expected_failures,\"result\":\"$result\"}" \
    > "$path"
}

# Most cases exercise a successful xcresulttool invocation. Keep that default
# visible here; the dedicated tool-failure case calls the evaluator directly.
evaluate_gate() {
  test_gate_evaluate "$1" "$2" "$3" 0 "$4" "$5"
}

GOOD_LOG="$TEST_TMP/good.log"
GOOD_FAST="$TEST_TMP/good-fast.json"
GOOD_SLOW="$TEST_TMP/good-slow.json"
write_log "$GOOD_LOG" '** TEST SUCCEEDED **' 0
write_summary "$GOOD_FAST" 698 698 0 0 0 Passed
write_summary "$GOOD_SLOW" 1 1 0 0 0 Passed

expect_status 'fast accepts its exact clean receipt' 0 \
  evaluate_gate fast 698 0 "$GOOD_LOG" "$GOOD_FAST"
expect_status 'slow accepts exactly one clean E1 test' 0 \
  evaluate_gate slow 1 0 "$GOOD_LOG" "$GOOD_SLOW"

ZERO="$TEST_TMP/zero.json"
write_summary "$ZERO" 0 0 0 0 0 Passed
expect_status 'fast rejects zero tests' 1 \
  evaluate_gate fast 698 0 "$GOOD_LOG" "$ZERO"
expect_status 'subset rejects zero tests' 1 \
  evaluate_gate subset 1 0 "$GOOD_LOG" "$ZERO"
expect_status 'slow rejects zero tests' 1 \
  evaluate_gate slow 1 0 "$GOOD_LOG" "$ZERO"

expect_status 'missing xcresult summary fails closed' 1 \
  evaluate_gate fast 698 0 "$GOOD_LOG" "$TEST_TMP/missing.json"

EMPTY_SUMMARY="$TEST_TMP/summary-tool-failed.json"
: > "$EMPTY_SUMMARY"
expect_status 'an empty summary from xcresulttool failure fails closed' 1 \
  test_gate_evaluate fast 698 0 64 "$GOOD_LOG" "$EMPTY_SUMMARY"
expect_status 'nonzero xcresulttool rc fails even with valid-looking JSON' 1 \
  test_gate_evaluate fast 698 0 64 "$GOOD_LOG" "$GOOD_FAST"

SKIPPED="$TEST_TMP/skipped.json"
write_summary "$SKIPPED" 698 697 0 1 0 Passed
expect_status 'one XCTSkip fails the gate' 1 \
  evaluate_gate fast 698 0 "$GOOD_LOG" "$SKIPPED"

EXPECTED_FAILURE="$TEST_TMP/expected-failure.json"
write_summary "$EXPECTED_FAILURE" 698 697 0 0 1 Passed
expect_status 'an expected failure is not a pass' 1 \
  evaluate_gate fast 698 0 "$GOOD_LOG" "$EXPECTED_FAILURE"

FAILED_SUMMARY="$TEST_TMP/failed.json"
write_summary "$FAILED_SUMMARY" 698 697 1 0 0 Failed
expect_status 'a failed test fails the gate' 1 \
  evaluate_gate fast 698 0 "$GOOD_LOG" "$FAILED_SUMMARY"

UNKNOWN_SUMMARY="$TEST_TMP/unknown.json"
write_summary "$UNKNOWN_SUMMARY" 698 698 0 0 0 unknown
expect_status 'xcresult result is case-sensitive Passed' 1 \
  evaluate_gate fast 698 0 "$GOOD_LOG" "$UNKNOWN_SUMMARY"

expect_status 'nonzero xcodebuild rc cannot be hidden by a success verdict' 1 \
  evaluate_gate fast 698 65 "$GOOD_LOG" "$GOOD_FAST"

RESTART_LOG="$TEST_TMP/restart.log"
write_log "$RESTART_LOG" '** TEST SUCCEEDED **' 1
expect_status 'a restarted test host fails the gate' 1 \
  evaluate_gate fast 698 0 "$RESTART_LOG" "$GOOD_FAST"

NO_VERDICT_LOG="$TEST_TMP/no-verdict.log"
write_log "$NO_VERDICT_LOG" '' 0
expect_status 'a missing textual verdict fails closed' 1 \
  evaluate_gate fast 698 0 "$NO_VERDICT_LOG" "$GOOD_FAST"

TOO_FEW="$TEST_TMP/too-few.json"
TOO_MANY="$TEST_TMP/too-many.json"
write_summary "$TOO_FEW" 697 697 0 0 0 Passed
write_summary "$TOO_MANY" 699 699 0 0 0 Passed
expect_status 'fast rejects fewer than its exact count' 1 \
  evaluate_gate fast 698 0 "$GOOD_LOG" "$TOO_FEW"
expect_status 'fast rejects more than its exact count until reviewed' 1 \
  evaluate_gate fast 698 0 "$GOOD_LOG" "$TOO_MANY"

SUBSET_TWO="$TEST_TMP/subset-two.json"
write_summary "$SUBSET_TWO" 2 2 0 0 0 Passed
expect_status 'subset accepts one or more clean selected tests' 0 \
  evaluate_gate subset 1 0 "$GOOD_LOG" "$SUBSET_TWO"

MALFORMED="$TEST_TMP/malformed.json"
printf '%s\n' '{"totalTestCount":698}' > "$MALFORMED"
expect_status 'missing structured fields fail closed' 1 \
  evaluate_gate fast 698 0 "$GOOD_LOG" "$MALFORMED"

expect_output 'fast owns the E1 and generator exclusion selectors' \
  '-skip-testing:BioMotionTests/E1MarkerSetComparisonTests
-skip-testing:BioMotionTests/SolvedPoseFixtureGeneratorTests' \
  test_gate_lane_selector fast
expect_output 'slow owns the exact E1 selector' \
  '-only-testing:BioMotionTests/E1MarkerSetComparisonTests/testE1RunAll' \
  test_gate_lane_selector slow
expect_output 'fast exact count is reviewed independently of slow' 724 \
  test_gate_expected_count fast
expect_output 'slow exact count is one E1 method' 1 \
  test_gate_expected_count slow

expect_status 'fast rejects caller-owned only-testing' 2 \
  test_gate_validate_lane_args fast -only-testing:BioMotionTests/SomeTests
expect_status 'fast rejects equals-form only-testing' 2 \
  test_gate_validate_lane_args fast -only-testing=BioMotionTests/SomeTests
expect_status 'slow rejects caller-owned skip-testing' 2 \
  test_gate_validate_lane_args slow -skip-testing:BioMotionTests/SomeTests
expect_status 'all rejects a conflicting test plan' 2 \
  test_gate_validate_lane_args all -testPlan OtherPlan
expect_status 'all rejects equals-form test plan' 2 \
  test_gate_validate_lane_args all -testPlan=OtherPlan
expect_status 'all rejects retry-on-failure' 2 \
  test_gate_validate_lane_args all -retry-tests-on-failure
expect_status 'fast rejects repeated iterations' 2 \
  test_gate_validate_lane_args fast -test-iterations 3
expect_status 'slow rejects run-until-failure' 2 \
  test_gate_validate_lane_args slow -run-tests-until-failure
expect_status 'all rejects repetition relaunch controls' 2 \
  test_gate_validate_lane_args all -test-repetition-relaunch-enabled=YES
expect_status 'gating lanes reject even otherwise safe caller overrides' 2 \
  test_gate_validate_lane_args fast -parallel-testing-enabled NO
expect_status 'subset rejects the documented test-repetitions alias' 2 \
  test_gate_validate_lane_args subset \
    -only-testing:BioMotionTests/SomeTests -test-repetitions=3
expect_status 'all rejects double-dash only-test-configuration' 2 \
  test_gate_validate_lane_args all --only-test-configuration Debug
expect_status 'subset rejects double-dash skip-test-configuration' 2 \
  test_gate_validate_lane_args subset \
    -only-testing:BioMotionTests/SomeTests --skip-test-configuration=Debug
expect_status 'subset requires an explicit only-testing selector' 2 \
  test_gate_validate_lane_args subset -parallel-testing-enabled NO
expect_status 'subset rejects skip-testing even with an inclusion' 2 \
  test_gate_validate_lane_args subset \
    -only-testing:BioMotionTests/SomeTests -skip-testing:BioMotionTests/OtherTests
expect_status 'subset accepts a colon-form selector and safe build args' 0 \
  test_gate_validate_lane_args subset \
    -only-testing:BioMotionTests/SomeTests -parallel-testing-enabled NO
expect_status 'subset accepts the two-argument selector form' 0 \
  test_gate_validate_lane_args subset \
    -only-testing BioMotionTests/SomeTests -parallel-testing-enabled NO
expect_status 'runner-owned destination cannot be overridden' 2 \
  test_gate_validate_lane_args fast -destination 'platform=iOS Simulator,name=iPhone 17'
expect_status 'runner-owned project cannot be overridden with colon form' 2 \
  test_gate_validate_lane_args slow -project:Other.xcodeproj
expect_status 'runner-owned result path cannot be overridden' 2 \
  test_gate_validate_lane_args all -resultBundlePath:Other.xcresult
expect_status 'runner-owned DerivedData cannot be overridden' 2 \
  test_gate_validate_lane_args subset \
    -only-testing:BioMotionTests/SomeTests -derivedDataPath /tmp/shared-cache
expect_status 'runner requires an explicit lane before touching the simulator' 2 \
  run_tests_main
expect_status 'runner rejects selector conflict before touching the simulator' 2 \
  run_tests_main fast -only-testing:BioMotionTests/SomeTests

ENTRY_ATTACK_ROOT="$TEST_TMP/entry-attack"
mkdir -p "$ENTRY_ATTACK_ROOT/fake-path"
cat > "$ENTRY_ATTACK_ROOT/bash-env" <<EOF
printf '%s\n' sourced > "$ENTRY_ATTACK_ROOT/bash-env-ran"
exit 0
EOF
for fake_tool in xcrun xcodebuild python3; do
  cat > "$ENTRY_ATTACK_ROOT/fake-path/$fake_tool" <<EOF
#!/bin/sh
printf '%s\n' "$fake_tool" >> "$ENTRY_ATTACK_ROOT/fake-tool-ran"
exit 0
EOF
  chmod 0755 "$ENTRY_ATTACK_ROOT/fake-path/$fake_tool"
done
protected_entry_rejects_environment_injection() {
  /usr/bin/env \
    PATH="$ENTRY_ATTACK_ROOT/fake-path" \
    BASH_ENV="$ENTRY_ATTACK_ROOT/bash-env" \
    SHELLOPTS=errexit:pipefail \
    'BASH_FUNC_xcodebuild%%=() { return 0; }' \
    "$REPO_ROOT/tools/run_tests.sh" invalid
  local status=$?
  if [ -e "$ENTRY_ATTACK_ROOT/bash-env-ran" ] || \
    [ -e "$ENTRY_ATTACK_ROOT/fake-tool-ran" ]; then
    return 99
  fi
  return "$status"
}
expect_status 'runner entry ignores BASH_ENV, SHELLOPTS, functions, and PATH tools' 2 \
  protected_entry_rejects_environment_injection

protected_test_gate_entry_rejects_environment_injection() {
  /usr/bin/env \
    BASH_ENV="$ENTRY_ATTACK_ROOT/bash-env" \
    'BASH_FUNC_test_gate_evaluate%%=() { return 0; }' \
    "$REPO_ROOT/tools/test_gate.sh"
  local status=$?
  if [ -e "$ENTRY_ATTACK_ROOT/bash-env-ran" ]; then
    return 99
  fi
  return "$status"
}
expect_status 'test gate direct entry ignores BASH_ENV and functions' 2 \
  protected_test_gate_entry_rejects_environment_injection
expect_status 'test gate rejects an explicit unprotected Bash entry' 78 \
  /bin/bash "$REPO_ROOT/tools/test_gate.sh"

# Real macOS `/bin/bash` is 3.2. Under `set -u`, expanding an empty array is an
# error; subset is the lane whose runner-owned selector is empty. Capture the
# helper's argv through a fake xcodebuild so this regression stays simulator-free.
capture_subset_xcodebuild_invocation() {
  TEST_DEVICE_UDID='TEST-UDID'
  local RUN_OUTPUT_DIR="$TEST_TMP/runner-output"
  local fake_xcodebuild="$TEST_TMP/fake-xcodebuild"
  cat > "$fake_xcodebuild" <<'EOF'
#!/bin/bash -p
if [[ "$PATH" != /usr/bin:/bin:/usr/sbin:/sbin || \
  -n "${BASH_ENV+x}" || -n "${ENV+x}" || \
  -n "${DEVELOPER_DIR+x}" || -n "${XCODE_XCCONFIG_FILE+x}" || \
  -n "${PYTHONPATH+x}" || -n "${DYLD_INSERT_LIBRARIES+x}" ]]; then
  exit 97
fi
printf '%s\n' "$@"
EOF
  chmod 0755 "$fake_xcodebuild"
  local RUN_TESTS_XCODEBUILD="$fake_xcodebuild"
  run_tests_invoke_xcodebuild '' "$TEST_TMP/subset-result.xcresult" \
    -only-testing:BioMotionTests/SomeTests
}
SUBSET_INVOCATION=$(printf '%s\n' \
  -project BioMotion.xcodeproj \
  -scheme BioMotion \
  -destination 'platform=iOS Simulator,id=TEST-UDID' \
  -derivedDataPath "$TEST_TMP/runner-output/DerivedData" \
  -resultBundlePath "$TEST_TMP/subset-result.xcresult" \
  -only-testing:BioMotionTests/SomeTests \
  test)
expect_output 'subset builds argv without expanding an empty Bash 3.2 array' \
  "$SUBSET_INVOCATION" capture_subset_xcodebuild_invocation

expect_status 'current XCTest source contains no runtime skips' 0 \
  test_gate_assert_no_xctskip "$REPO_ROOT/BioMotionTests"
BAD_TEST_ROOT="$TEST_TMP/TestsWithSkip"
mkdir -p "$BAD_TEST_ROOT"
printf '%s\n' 'func testRequiredFixture() throws { throw XCTSkip("missing") }' \
  > "$BAD_TEST_ROOT/RequiredFixtureTests.swift"
expect_status 'source policy rejects reintroduced XCTSkip' 1 \
  test_gate_assert_no_xctskip "$BAD_TEST_ROOT"
BAD_CONDITIONAL_ROOT="$TEST_TMP/TestsWithConditionalSkip"
mkdir -p "$BAD_CONDITIONAL_ROOT"
printf '%s\n' 'func testRequiredFixture() throws { try XCTSkipIf(true, "missing") }' \
  > "$BAD_CONDITIONAL_ROOT/RequiredFixtureTests.swift"
expect_status 'source policy also rejects XCTSkipIf and XCTSkipUnless family' 1 \
  test_gate_assert_no_xctskip "$BAD_CONDITIONAL_ROOT"

printf '\n%d passed, %d failed\n' "$PASS_COUNT" "$FAIL_COUNT"
[ "$FAIL_COUNT" -eq 0 ]
