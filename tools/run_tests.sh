#!/bin/bash
#
# The commit gate. Run this, not a hand-typed xcodebuild line.
#
# # Why this file exists
#
# Three reviewers ran "the test suite" on 2026-08-07 and got three different
# answers. Two causes, both mechanical, both removed here:
#
#   1. THE SHARED DEVICE. Every documented invocation named a simulator by
#      NAME -- `name=iPhone 17` in STATUS.md, `name=iPhone 17 Pro` in
#      README.md -- so two `xcodebuild test` processes resolved to the same
#      UDID and evicted each other's test host. The evicted tests are reported
#      neither passed nor failed. This script uses a device it owns, named
#      $DEVICE_NAME, and takes a lock so a second invocation refuses rather
#      than sharing it.
#
#   2. THE GREEN LINE THAT ISN'T ONE. A killed test host still prints
#      `Executed N tests, with 0 failures (0 unexpected)` for every suite that
#      completed before the kill. That line is not a verdict. Only the trailing
#      `** TEST SUCCEEDED **`, a zero count of `Restarting after unexpected
#      exit`, and a full test count together say the suite ran. This script
#      checks all three and exits non-zero if any fails.
#
# # What it does NOT do
#
# It does not skip anything except `E1MarkerSetComparisonTests`, which costs
# over an hour (STATUS.md, next-step 14). Do not add skips here to make a run
# green -- a test that cannot pass stays failing, with its number reported.
#
# Usage:  tools/run_tests.sh [extra xcodebuild args...]
#   e.g.  tools/run_tests.sh -only-testing:BioMotionTests/IKConvergenceTests
#
set -u

DEVICE_NAME="BioMotion-CI"
DEVICE_TYPE="iPhone 17"
LOCK_DIR="${TMPDIR:-/tmp}/biomotion-run-tests.lock"

# The floor the run must clear. It is not decoration: a test host killed
# mid-run, or a test file that `xcodegen generate` was never run for, both
# show up ONLY as a smaller count -- every other line still reads green.
# Raise it when you add tests; never lower it to make a run pass.
# 474 at 64c3959. +3 MuscleSolverExactnessTests (2026-08-09 QP fix, and
# WrappedMomentArmLeakTests replaced one test with one test) = 477, measured.
# +6 MultiWrapReferenceTests (2026-08-09 multi-wrap reference) = 483.
# +1 WrappedMomentArmLeakTests.testTheReferenceDisagreesWithItselfByMoreThanThe
#    GateAllows (2026-08-09 leak re-run) = 484, measured.
MIN_TESTS=484

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT" || exit 1

# ---- one run at a time, on a device nothing else is using ------------------
if ! mkdir "$LOCK_DIR" 2>/dev/null; then
  echo "REFUSING: $LOCK_DIR exists, so another run is using $DEVICE_NAME."
  echo "Two xcodebuild test processes on one simulator evict each other's test"
  echo "host and both report a partial suite as green. Wait, or remove the lock"
  echo "if you are sure no run is live: rmdir $LOCK_DIR"
  exit 2
fi
trap 'rmdir "$LOCK_DIR" 2>/dev/null' EXIT

UDID=$(xcrun simctl list devices -j \
  | python3 -c "
import json,sys
d = json.load(sys.stdin)['devices']
for runtime, devices in d.items():
    if 'iOS' not in runtime: continue
    for dev in devices:
        if dev['name'] == '$DEVICE_NAME' and dev['isAvailable']:
            print(dev['udid']); sys.exit(0)
")
if [ -z "$UDID" ]; then
  echo "Creating dedicated simulator '$DEVICE_NAME' ($DEVICE_TYPE)..."
  UDID=$(xcrun simctl create "$DEVICE_NAME" "$DEVICE_TYPE") || exit 1
fi
xcrun simctl bootstatus "$UDID" -b > /dev/null 2>&1
echo "Device: $DEVICE_NAME ($UDID)"

STAMP=$(date +%Y%m%d-%H%M%S)
LOG="${TMPDIR:-/tmp}/biomotion-tests-${STAMP}.log"
START=$(date +%s)

xcodebuild -project BioMotion.xcodeproj -scheme BioMotion \
  -destination "platform=iOS Simulator,id=${UDID}" \
  -skip-testing:BioMotionTests/E1MarkerSetComparisonTests \
  "$@" test > "$LOG" 2>&1
XCODEBUILD_RC=$?
WALL=$(( $(date +%s) - START ))

# ---- the three numbers that decide whether this run means anything --------
EXECUTED_LINE=$(grep -E "^[[:space:]]*Executed [0-9]+ test" "$LOG" | tail -1)
EXECUTED=$(echo "$EXECUTED_LINE" | sed -E 's/.*Executed ([0-9]+) test.*/\1/')
RESTARTS=$(grep -c "Restarting after unexpected exit" "$LOG")
VERDICT=$(grep -E "^\*\* TEST (SUCCEEDED|FAILED) \*\*" "$LOG" | tail -1)

echo
echo "log:      $LOG"
echo "wall:     ${WALL}s"
echo "executed: ${EXECUTED_LINE:-<none>}"
echo "restarts: $RESTARTS   (each one is tests silently not run)"
echo "verdict:  ${VERDICT:-<none, xcodebuild rc=$XCODEBUILD_RC>}"

if [ "$RESTARTS" -gt 0 ]; then
  echo
  echo "Killed hosts -- the tests running at these points did NOT report:"
  grep -n "Restarting after unexpected exit" "$LOG" | head -20
fi
if [ "$VERDICT" != "** TEST SUCCEEDED **" ]; then
  echo
  echo "Failures:"
  grep -E "error:|XCTAssert.* failed" "$LOG" | head -30
fi

FAIL=0
[ "$VERDICT" = "** TEST SUCCEEDED **" ] || { echo "GATE FAIL: verdict is not TEST SUCCEEDED"; FAIL=1; }
[ "$RESTARTS" -eq 0 ] || { echo "GATE FAIL: $RESTARTS test host(s) were killed; the suite is incomplete"; FAIL=1; }
# The count floor applies to a whole-suite run only. `-only-testing` is a
# debugging aid, and a subset that passes is NOT the commit gate -- say so
# rather than silently accepting a smaller number.
case " $* " in
  *" -only-testing"*)
    echo "NOTE: -only-testing given, so the $MIN_TESTS-test floor is not checked."
    echo "      A subset run is not the commit gate."
    ;;
  *)
    if [ -z "$EXECUTED" ] || [ "$EXECUTED" -lt "$MIN_TESTS" ]; then
      echo "GATE FAIL: ran ${EXECUTED:-0} tests, floor is $MIN_TESTS"
      FAIL=1
    fi
    ;;
esac

if [ "$FAIL" -eq 0 ]; then
  echo "GATE PASS"
else
  echo "GATE FAIL -- do not commit"
fi
exit "$FAIL"
