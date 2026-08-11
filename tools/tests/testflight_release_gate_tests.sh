#!/bin/bash
# Hermetic causal tests for the archive-to-TestFlight release wrapper.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
TEST_ROOT="$(/usr/bin/mktemp -d \
  "${TMPDIR:-/tmp}/biomotion-testflight-tests.XXXXXX")"
TEST_ROOT="$(cd "$TEST_ROOT" && pwd -P)"
trap '/bin/rm -rf "$TEST_ROOT"' EXIT

PASS_COUNT=0
TEST_KEY_ID="TESTKEY123"
TEST_ISSUER="12345678-1234-1234-1234-123456789abc"

fail() {
  printf 'FAIL: %s\n' "$*" >&2
  exit 1
}

pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  printf 'PASS: %s\n' "$*"
}

assert_status() {
  local expected="$1"
  local actual="$2"
  local label="$3"
  [[ "$actual" -eq "$expected" ]] || \
    fail "$label: expected exit $expected, found $actual"
}

assert_no_file() {
  local path="$1"
  local label="$2"
  [[ ! -e "$path" ]] || fail "$label: unexpected file $path"
}

assert_file_text() {
  local path="$1"
  local expected="$2"
  local label="$3"
  [[ -f "$path" ]] || fail "$label: missing file $path"
  local actual
  actual="$(/bin/cat "$path")"
  if [[ "$actual" != "$expected" ]]; then
    printf 'expected:\n%s\nactual:\n%s\n' "$expected" "$actual" >&2
    fail "$label: content mismatch"
  fi
}

assert_snapshot_removed() {
  local snapshot_log="$1"
  local label="$2"
  [[ -f "$snapshot_log" ]] || fail "$label: missing snapshot log"
  local snapshot
  snapshot="$(/usr/bin/tail -n 1 "$snapshot_log")"
  [[ -n "$snapshot" ]] || fail "$label: empty snapshot path"
  [[ ! -e "${snapshot%/*}" ]] || \
    fail "$label: private snapshot directory still exists: ${snapshot%/*}"
}

make_fixture() {
  local name="$1"
  local root="$TEST_ROOT/$name"
  /bin/mkdir -p \
    "$root/tools/release" \
    "$root/tools/tests" \
    "$root/fake-tools" \
    "$root/home/.appstoreconnect/private_keys" \
    "$root/BioMotion.xcarchive/Products/Applications/BioMotion.app"
  /bin/cp "$REPO_ROOT/tools/release/testflight_release.sh" \
    "$root/tools/release/testflight_release.sh"
  /bin/cp "$REPO_ROOT/tools/release/ExportOptions-TestFlight.plist" \
    "$root/tools/release/ExportOptions-TestFlight.plist"

  /usr/bin/python3 - "$root" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])

resource_gate = r'''#!/bin/bash
set -euo pipefail
if [[ -n "${ASC_API_KEY_ID+x}" || -n "${ASC_API_ISSUER+x}" ]]; then
  printf '%s\n' 'credentials:leaked' >> "$FAKE_EVENT_LOG"
  exit 97
fi
if [[ -n "${DEVELOPER_DIR+x}" || -n "${TOOLCHAINS+x}" || \
  -n "${SDKROOT+x}" || -n "${PYTHONPATH+x}" ]]; then
  printf '%s\n' 'environment:leaked' >> "$FAKE_EVENT_LOG"
  exit 96
fi
case "$#:${1:-}" in
  0:)
    printf '%s\n' 'resource:source' >> "$FAKE_EVENT_LOG"
    [[ "${FAKE_FAIL_GATE:-}" != source ]] || exit 41
    ;;
  2:--release-archive)
    [[ "$2" == "$FAKE_EXPECTED_ARCHIVE" ]] || exit 95
    printf '%s\n' 'resource:archive' >> "$FAKE_EVENT_LOG"
    [[ "${FAKE_FAIL_GATE:-}" != archive ]] || exit 42
    ;;
  3:--release-ipa)
    [[ "$2" == "$FAKE_EXPECTED_EXPORT/BioMotion.ipa" ]] || exit 94
    [[ "$3" == "$FAKE_EXPECTED_ARCHIVE" ]] || exit 93
    printf '%s\n' 'resource:ipa' >> "$FAKE_EVENT_LOG"
    [[ "${FAKE_FAIL_GATE:-}" != ipa ]] || exit 43
    if [[ "${FAKE_CREATE_KEY_AT_IPA_GATE:-0}" == 1 ]]; then
      /bin/mkdir -p "${FAKE_KEY_PATH%/*}"
      : > "$FAKE_KEY_PATH"
      /bin/chmod 0600 "$FAKE_KEY_PATH"
    fi
    ;;
  *) exit 92 ;;
esac
'''

privacy_gate = r'''#!/bin/bash
set -euo pipefail
if [[ -n "${ASC_API_KEY_ID+x}" || -n "${ASC_API_ISSUER+x}" ]]; then
  printf '%s\n' 'credentials:leaked' >> "$FAKE_EVENT_LOG"
  exit 97
fi
[[ "$#" -eq 1 ]]
[[ "$1" == "$FAKE_EXPECTED_ARCHIVE/Products/Applications/BioMotion.app" ]]
printf '%s\n' 'privacy:archive' >> "$FAKE_EVENT_LOG"
[[ "${FAKE_FAIL_GATE:-}" != privacy ]] || exit 44
'''

xcodebuild = r'''#!/bin/bash
set -euo pipefail
if [[ -n "${ASC_API_KEY_ID+x}" || -n "${ASC_API_ISSUER+x}" ]]; then
  printf '%s\n' 'credentials:leaked' >> "$FAKE_EVENT_LOG"
  exit 97
fi
if [[ -n "${DEVELOPER_DIR+x}" || -n "${TOOLCHAINS+x}" || \
  -n "${SDKROOT+x}" || -n "${PYTHONPATH+x}" ]]; then
  printf '%s\n' 'environment:leaked' >> "$FAKE_EVENT_LOG"
  exit 96
fi
[[ "$#" -eq 7 ]]
[[ "$1" == -exportArchive ]]
[[ "$2" == -archivePath && "$3" == "$FAKE_EXPECTED_ARCHIVE" ]]
[[ "$4" == -exportPath && "$5" == "$FAKE_EXPECTED_EXPORT" ]]
[[ "$6" == -exportOptionsPlist ]]
[[ "$7" == "$FAKE_EXPECTED_EXPORT_OPTIONS" ]]
for argument in "$@"; do
  [[ "$argument" != -allowProvisioningUpdates ]] || exit 95
done
printf '%s\n' 'xcodebuild:export' >> "$FAKE_EVENT_LOG"
[[ "${FAKE_FAIL_GATE:-}" != export ]] || exit 45
printf '%s\n' 'reviewed ipa bytes' > "$FAKE_EXPECTED_EXPORT/BioMotion.ipa"
if [[ "${FAKE_SECOND_IPA:-0}" == 1 ]]; then
  printf '%s\n' 'unexpected ipa' > "$FAKE_EXPECTED_EXPORT/Unexpected.ipa"
fi
'''

xcrun = r'''#!/bin/bash
set -euo pipefail
[[ "$#" -eq 10 ]]
[[ "$1" == altool ]]
case "$2" in
  --validate-app) event='xcrun:validate' ;;
  --upload-app) event='xcrun:upload' ;;
  *) exit 91 ;;
esac
[[ "$3" == -f ]]
ipa="$4"
[[ "$5" == -t && "$6" == ios ]]
[[ "$7" == --apiKey && "$8" == TESTKEY123 ]]
[[ "$9" == --apiIssuer ]]
[[ "$10" == 12345678-1234-1234-1234-123456789abc ]]
[[ -f "$HOME/.appstoreconnect/private_keys/AuthKey_TESTKEY123.p8" ]]
[[ ! -L "$ipa" && -f "$ipa" ]]
snapshot_directory="${ipa%/*}"
case "$snapshot_directory" in
  /tmp/biomotion-testflight.*|/private/tmp/biomotion-testflight.*) ;;
  *) exit 90 ;;
esac
[[ "$(/usr/bin/stat -f '%Lp' "$snapshot_directory")" == 700 ]]
[[ "$(/usr/bin/stat -f '%Lp' "$ipa")" == 600 ]]
[[ "$ipa" != "$FAKE_EXPECTED_EXPORT/BioMotion.ipa" ]]
printf '%s\n' "$event" >> "$FAKE_EVENT_LOG"
printf '%s\n' "$ipa" >> "$FAKE_SNAPSHOT_LOG"
if [[ "$event" == xcrun:validate && \
  "${FAKE_MUTATE_SNAPSHOT_AFTER_VALIDATE:-0}" == 1 ]]; then
  printf '%s\n' 'mutated after validation' >> "$ipa"
fi
'''

(root / "tools/tests/app_resource_boundary_probe.sh").write_text(
    resource_gate, encoding="utf-8"
)
(root / "tools/tests/privacy_manifest_probe.sh").write_text(
    privacy_gate, encoding="utf-8"
)
(root / "fake-tools/xcodebuild").write_text(xcodebuild, encoding="utf-8")
(root / "fake-tools/xcrun").write_text(xcrun, encoding="utf-8")

wrapper_path = root / "tools/release/testflight_release.sh"
wrapper = wrapper_path.read_text(encoding="utf-8")
replacements = {
    'XCODEBUILD="/usr/bin/xcodebuild"':
        f'XCODEBUILD="{root / "fake-tools/xcodebuild"}"',
    'XCRUN="/usr/bin/xcrun"':
        f'XCRUN="{root / "fake-tools/xcrun"}"',
}
for old, new in replacements.items():
    if wrapper.count(old) != 1:
        raise SystemExit(f"fixture replacement count changed for {old!r}")
    wrapper = wrapper.replace(old, new)
wrapper_path.write_text(wrapper, encoding="utf-8")
PY

  /bin/chmod 0755 \
    "$root/tools/release/testflight_release.sh" \
    "$root/tools/tests/app_resource_boundary_probe.sh" \
    "$root/tools/tests/privacy_manifest_probe.sh" \
    "$root/fake-tools/xcodebuild" \
    "$root/fake-tools/xcrun"
  : > "$root/home/.appstoreconnect/private_keys/AuthKey_${TEST_KEY_ID}.p8"
  /bin/chmod 0600 \
    "$root/home/.appstoreconnect/private_keys/AuthKey_${TEST_KEY_ID}.p8"
  printf '%s\n' "$root"
}

run_release() {
  local root="$1"
  shift
  local environment=(BIOMOTION_TEST_FIXTURE=1)
  while [[ "$#" -gt 0 && "$1" != -- ]]; do
    environment+=("$1")
    shift
  done
  [[ "$#" -gt 0 ]] || fail "run_release requires an argument separator"
  shift
  set +e
  (
    cd "$root"
    /usr/bin/env \
      PATH="$root/untrusted-path" \
      HOME="$root/home" \
      ASC_API_KEY_ID="$TEST_KEY_ID" \
      ASC_API_ISSUER="$TEST_ISSUER" \
      DEVELOPER_DIR="$root/untrusted-developer" \
      TOOLCHAINS=untrusted-toolchain \
      SDKROOT="$root/untrusted-sdk" \
      PYTHONPATH="$root/untrusted-python" \
      FAKE_EVENT_LOG="$root/events.log" \
      FAKE_SNAPSHOT_LOG="$root/snapshot.log" \
      FAKE_EXPECTED_ARCHIVE="$root/BioMotion.xcarchive" \
      FAKE_EXPECTED_EXPORT="$root/export" \
      FAKE_EXPECTED_EXPORT_OPTIONS="$root/tools/release/ExportOptions-TestFlight.plist" \
      FAKE_KEY_PATH="$root/home/.appstoreconnect/private_keys/AuthKey_${TEST_KEY_ID}.p8" \
      "${environment[@]}" \
      /bin/bash tools/release/testflight_release.sh "$@"
  ) >"$root/output.txt" 2>&1
  RUN_STATUS=$?
  set -e
}

standard_arguments() {
  STANDARD_ARGUMENTS=(
    --archive "$1/BioMotion.xcarchive"
    --export-dir "$1/export"
  )
}

# Supplying credentials cannot turn the default local-only mode into a request.
root="$(make_fixture default_no_network)"
standard_arguments "$root"
run_release "$root" -- "${STANDARD_ARGUMENTS[@]}"
assert_status 0 "$RUN_STATUS" "default local export"
assert_file_text "$root/events.log" \
  $'resource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa' \
  "default local gate order"
assert_no_file "$root/snapshot.log" "default mode must not invoke xcrun"
/usr/bin/grep -Fq 'TESTFLIGHT_RELEASE_PASS export-only' "$root/output.txt" || \
  fail "default mode did not report export-only success"
pass "default mode is a credential-blind local export with zero network calls"

# A failed archive gate must prevent both export and credential diagnostics.
root="$(make_fixture archive_gate_failure)"
standard_arguments "$root"
run_release "$root" \
  FAKE_FAIL_GATE=archive \
  ASC_API_KEY_ID= \
  ASC_API_ISSUER= \
  -- --upload "${STANDARD_ARGUMENTS[@]}"
assert_status 42 "$RUN_STATUS" "archive gate failure"
assert_file_text "$root/events.log" \
  $'resource:source\nresource:archive' \
  "archive failure causal order"
assert_no_file "$root/export/BioMotion.ipa" \
  "archive failure must block export"
assert_no_file "$root/snapshot.log" "archive failure must block xcrun"
if /usr/bin/grep -Eq 'ASC_API|AuthKey_' "$root/output.txt"; then
  fail "archive failure inspected credentials before returning"
fi
pass "source/archive gate failures dominate export and credential handling"

# The exact export command must precede, and be followed by, the IPA gate.
root="$(make_fixture export_then_ipa)"
standard_arguments "$root"
run_release "$root" -- "${STANDARD_ARGUMENTS[@]}"
assert_status 0 "$RUN_STATUS" "export and IPA gate"
assert_file_text "$root/events.log" \
  $'resource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa' \
  "export and IPA gate order"
[[ -f "$root/export/BioMotion.ipa" ]] || fail "exported IPA is missing"
[[ -f "$root/export/BioMotion.ipa.sha256" ]] || \
  fail "exported IPA SHA-256 receipt is missing"
/usr/bin/grep -Eq '^[0-9a-f]{64}  BioMotion\.ipa$' \
  "$root/export/BioMotion.ipa.sha256" || \
  fail "exported IPA SHA-256 receipt is malformed"
/usr/bin/grep -Eq '^IPA_SHA256 [0-9a-f]{64}  ' "$root/output.txt" || \
  fail "exported IPA SHA-256 was not recorded"
pass "export uses destination=export without provisioning updates, then gates one IPA"

# Validation performs exactly one external operation and cleans its snapshot.
root="$(make_fixture validate_only)"
standard_arguments "$root"
run_release "$root" -- --validate "${STANDARD_ARGUMENTS[@]}"
assert_status 0 "$RUN_STATUS" "validate mode"
assert_file_text "$root/events.log" \
  $'resource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa\nxcrun:validate' \
  "validate-only order"
assert_snapshot_removed "$root/snapshot.log" "validate-only cleanup"
pass "--validate validates the private snapshot and cannot upload"

# Upload is authorized only as validation followed by upload of the same bytes.
root="$(make_fixture upload_success)"
standard_arguments "$root"
run_release "$root" -- --upload "${STANDARD_ARGUMENTS[@]}"
assert_status 0 "$RUN_STATUS" "upload mode"
assert_file_text "$root/events.log" \
  $'resource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa\nxcrun:validate\nxcrun:upload' \
  "upload order"
/usr/bin/python3 - "$root/snapshot.log" <<'PY'
from pathlib import Path
import sys

paths = Path(sys.argv[1]).read_text(encoding="utf-8").splitlines()
if len(paths) != 2 or paths[0] != paths[1]:
    raise SystemExit(f"validation/upload did not use one snapshot: {paths!r}")
PY
assert_snapshot_removed "$root/snapshot.log" "upload cleanup"
pass "--upload validates before uploading the identical private snapshot"

# A validator-side mutation must be detected before the upload call.
root="$(make_fixture digest_change)"
standard_arguments "$root"
run_release "$root" \
  FAKE_MUTATE_SNAPSHOT_AFTER_VALIDATE=1 \
  -- --upload "${STANDARD_ARGUMENTS[@]}"
assert_status 1 "$RUN_STATUS" "digest mutation"
assert_file_text "$root/events.log" \
  $'resource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa\nxcrun:validate' \
  "digest mutation order"
/usr/bin/grep -Fq 'IPA bytes changed' "$root/output.txt" || \
  fail "digest mutation did not report the byte mismatch"
assert_snapshot_removed "$root/snapshot.log" "digest mutation cleanup"
pass "a changed post-validation snapshot blocks upload"

# The final local gate creates the key; success proves lookup happened later.
root="$(make_fixture credentials_deferred)"
/bin/rm "$root/home/.appstoreconnect/private_keys/AuthKey_${TEST_KEY_ID}.p8"
standard_arguments "$root"
run_release "$root" \
  FAKE_CREATE_KEY_AT_IPA_GATE=1 \
  -- --validate "${STANDARD_ARGUMENTS[@]}"
assert_status 0 "$RUN_STATUS" "deferred credentials"
assert_file_text "$root/events.log" \
  $'resource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa\nxcrun:validate' \
  "deferred credential order"
pass "ASC credentials and key path are inaccessible until every local gate passes"

# Two IPA files are never ambiguously selected for validation or upload.
root="$(make_fixture multiple_ipas)"
standard_arguments "$root"
run_release "$root" FAKE_SECOND_IPA=1 -- --validate "${STANDARD_ARGUMENTS[@]}"
assert_status 1 "$RUN_STATUS" "multiple IPA rejection"
assert_file_text "$root/events.log" \
  $'resource:source\nresource:archive\nprivacy:archive\nxcodebuild:export' \
  "multiple IPA rejection order"
assert_no_file "$root/snapshot.log" "ambiguous IPA set must block xcrun"
pass "the export directory must contain exactly one root IPA"

# Unknown, positional, incomplete, duplicate, and conflicting arguments fail 64.
for argument_case in \
  unknown positional missing_archive missing_export duplicate_archive conflicting_mode
do
  root="$(make_fixture "arguments_$argument_case")"
  case "$argument_case" in
    unknown)
      arguments=(--unknown)
      ;;
    positional)
      arguments=(unexpected)
      ;;
    missing_archive)
      arguments=(--export-dir "$root/export")
      ;;
    missing_export)
      arguments=(--archive "$root/BioMotion.xcarchive")
      ;;
    duplicate_archive)
      arguments=(
        --archive "$root/BioMotion.xcarchive"
        --archive "$root/BioMotion.xcarchive"
        --export-dir "$root/export"
      )
      ;;
    conflicting_mode)
      arguments=(
        --validate --upload
        --archive "$root/BioMotion.xcarchive"
        --export-dir "$root/export"
      )
      ;;
  esac
  run_release "$root" -- "${arguments[@]}"
  assert_status 64 "$RUN_STATUS" "argument rejection: $argument_case"
  assert_no_file "$root/events.log" \
    "argument rejection must precede all gates: $argument_case"
done
pass "malformed, incomplete, duplicate, positional, and conflicting arguments are rejected"

printf 'TESTFLIGHT_RELEASE_GATE_TESTS_PASS %s/9\n' "$PASS_COUNT"
