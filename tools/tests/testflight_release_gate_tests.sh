#!/bin/bash -p
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
    "$root/controls" \
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

shell_environment_guard = r'''
EVENT_LOG="$FIXTURE_ROOT/events.log"
CONTROL_DIRECTORY="$FIXTURE_ROOT/controls"

assert_clean_shell_environment() {
  local expected_home="$1"
  if [[ "$PATH" != "/usr/bin:/bin:/usr/sbin:/sbin" || \
    "$HOME" != "$expected_home" || -z "$USER" || \
    "$LOGNAME" != "$USER" || "$LANG" != C || "$LC_ALL" != C || \
    -n "${ASC_API_KEY_ID+x}" || -n "${ASC_API_ISSUER+x}" || \
    -n "${XCODE_XCCONFIG_FILE+x}" || -n "${BASH_ENV+x}" || \
    -n "${ENV+x}" || -n "${DEVELOPER_DIR+x}" || \
    -n "${TOOLCHAINS+x}" || -n "${SDKROOT+x}" || \
    -n "${PERL5OPT+x}" || -n "${PERL5LIB+x}" || \
    -n "${PYTHONPATH+x}" || -n "${PYTHONHOME+x}" || \
    -n "${PYTHONSTARTUP+x}" || -n "${PYTHONINSPECT+x}" || \
    -n "${PYTHONWARNINGS+x}" || -n "${DYLD_LIBRARY_PATH+x}" || \
    -n "${DYLD_INSERT_LIBRARIES+x}" || -n "${CPATH+x}" || \
    -n "${LIBRARY_PATH+x}" || -n "${RUBYOPT+x}" || \
    "$(type -t biomotion_injected || true)" == function ]]; then
    printf '%s\n' 'environment:leaked' >> "$EVENT_LOG"
    exit 96
  fi
  if /usr/bin/env | /usr/bin/grep -Eq '^BASH_FUNC_'; then
    printf '%s\n' 'environment:function-leaked' >> "$EVENT_LOG"
    exit 96
  fi
}

FAIL_GATE=""
if [[ -f "$CONTROL_DIRECTORY/fail-gate" ]]; then
  FAIL_GATE="$(/bin/cat "$CONTROL_DIRECTORY/fail-gate")"
fi
'''

dependency_gate = r'''#!/bin/bash
set -euo pipefail
FIXTURE_ROOT="$(cd -P -- "${BASH_SOURCE[0]%/*}/../.." && pwd -P)"
''' + shell_environment_guard + r'''
assert_clean_shell_environment /var/empty
printf '%s\n' 'dependency:current' >> "$EVENT_LOG"
[[ "$FAIL_GATE" != dependency ]] || exit 40
'''

dependency_receipt = r'''#!/usr/bin/env python3
import os
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[2]
event_log = root / "events.log"
blocked = {
    "ASC_API_KEY_ID",
    "ASC_API_ISSUER",
    "XCODE_XCCONFIG_FILE",
    "BASH_ENV",
    "ENV",
    "DEVELOPER_DIR",
    "TOOLCHAINS",
    "PERL5OPT",
    "PERL5LIB",
    "PYTHONPATH",
    "PYTHONHOME",
    "PYTHONSTARTUP",
    "PYTHONINSPECT",
    "PYTHONWARNINGS",
    "DYLD_LIBRARY_PATH",
    "DYLD_INSERT_LIBRARIES",
    "RUBYOPT",
}
caller_values = {
    str(root / "untrusted-sdk"),
    str(root / "untrusted-cpath"),
    str(root / "untrusted-library"),
}
if (
    blocked.intersection(os.environ)
    or any(name.startswith("BASH_FUNC_") for name in os.environ)
    or os.environ.get("HOME") != "/var/empty"
    or os.environ.get("PATH") != "/usr/bin:/bin:/usr/sbin:/sbin"
    or not os.environ.get("USER")
    or os.environ.get("LOGNAME") != os.environ.get("USER")
    or os.environ.get("LANG") != "C"
    or os.environ.get("LC_ALL") != "C"
    or caller_values.intersection(os.environ.values())
):
    event_log.open("a").write("environment:leaked\n")
    raise SystemExit(96)
expected = [
    sys.argv[0],
    "verify",
    str(root),
    str(root / "BioMotion.xcarchive"),
    str(root / "BioMotion.xcarchive.dependency-receipt.json"),
]
if sys.argv != expected:
    raise SystemExit(95)
receipt = Path(sys.argv[4])
if receipt.is_symlink() or not receipt.is_file():
    raise SystemExit(94)
count_path = root / "receipt-count"
count = int(count_path.read_text(encoding="utf-8")) if count_path.exists() else 0
count += 1
count_path.write_text(f"{count}\n", encoding="utf-8")
stage = {1: "pre", 2: "post"}.get(count)
if stage is None:
    raise SystemExit(93)
with event_log.open("a") as stream:
    stream.write(f"dependency:receipt:{stage}\n")
fail_path = root / "controls" / "fail-gate"
fail_gate = fail_path.read_text(encoding="utf-8").strip() if fail_path.exists() else ""
if fail_gate == f"receipt-{stage}":
    raise SystemExit(41)
'''

resource_gate = r'''#!/bin/bash
set -euo pipefail
FIXTURE_ROOT="$(cd -P -- "${BASH_SOURCE[0]%/*}/../.." && pwd -P)"
''' + shell_environment_guard + r'''
assert_clean_shell_environment /var/empty
EXPECTED_ARCHIVE="$FIXTURE_ROOT/BioMotion.xcarchive"
EXPECTED_EXPORT="$FIXTURE_ROOT/export"
case "$#:${1:-}" in
  0:)
    printf '%s\n' 'resource:source' >> "$EVENT_LOG"
    [[ "$FAIL_GATE" != source ]] || exit 41
    ;;
  2:--release-archive)
    [[ "$2" == "$EXPECTED_ARCHIVE" ]] || exit 95
    printf '%s\n' 'resource:archive' >> "$EVENT_LOG"
    [[ "$FAIL_GATE" != archive ]] || exit 42
    ;;
  3:--release-ipa)
    [[ "$2" == "$EXPECTED_EXPORT/BioMotion.ipa" ]] || exit 94
    [[ "$3" == "$EXPECTED_ARCHIVE" ]] || exit 93
    printf '%s\n' 'resource:ipa' >> "$EVENT_LOG"
    [[ "$FAIL_GATE" != ipa ]] || exit 43
    if [[ -f "$CONTROL_DIRECTORY/create-key-at-ipa-gate" ]]; then
      key_path="$FIXTURE_ROOT/home/.appstoreconnect/private_keys/AuthKey_TESTKEY123.p8"
      /bin/mkdir -p "${key_path%/*}"
      : > "$key_path"
      /bin/chmod 0600 "$key_path"
    fi
    ;;
  *) exit 92 ;;
esac
'''

privacy_gate = r'''#!/bin/bash
set -euo pipefail
FIXTURE_ROOT="$(cd -P -- "${BASH_SOURCE[0]%/*}/../.." && pwd -P)"
''' + shell_environment_guard + r'''
assert_clean_shell_environment /var/empty
[[ "$#" -eq 1 ]]
[[ "$1" == "$FIXTURE_ROOT/BioMotion.xcarchive/Products/Applications/BioMotion.app" ]]
printf '%s\n' 'privacy:archive' >> "$EVENT_LOG"
[[ "$FAIL_GATE" != privacy ]] || exit 44
'''

xcodebuild = r'''#!/bin/bash
set -euo pipefail
FIXTURE_ROOT="$(cd -P -- "${BASH_SOURCE[0]%/*}/.." && pwd -P)"
''' + shell_environment_guard + r'''
assert_clean_shell_environment "$FIXTURE_ROOT/home"
EXPECTED_ARCHIVE="$FIXTURE_ROOT/BioMotion.xcarchive"
EXPECTED_EXPORT="$FIXTURE_ROOT/export"
[[ "$#" -eq 7 ]]
[[ "$1" == -exportArchive ]]
[[ "$2" == -archivePath && "$3" == "$EXPECTED_ARCHIVE" ]]
[[ "$4" == -exportPath && "$5" == "$EXPECTED_EXPORT" ]]
[[ "$6" == -exportOptionsPlist ]]
[[ "$7" == "$FIXTURE_ROOT/tools/release/ExportOptions-TestFlight.plist" ]]
for argument in "$@"; do
  [[ "$argument" != -allowProvisioningUpdates ]] || exit 95
done
printf '%s\n' 'xcodebuild:export' >> "$EVENT_LOG"
[[ "$FAIL_GATE" != export ]] || exit 45
printf '%s\n' 'reviewed ipa bytes' > "$EXPECTED_EXPORT/BioMotion.ipa"
if [[ -f "$CONTROL_DIRECTORY/second-ipa" ]]; then
  printf '%s\n' 'unexpected ipa' > "$EXPECTED_EXPORT/Unexpected.ipa"
fi
if [[ -f "$CONTROL_DIRECTORY/ipa-receipt-symlink" ]]; then
  /bin/ln -s "$FIXTURE_ROOT/receipt-sentinel" \
    "$EXPECTED_EXPORT/BioMotion.ipa.sha256"
fi
'''

xcrun = r'''#!/bin/bash
set -euo pipefail
FIXTURE_ROOT="$(cd -P -- "${BASH_SOURCE[0]%/*}/.." && pwd -P)"
''' + shell_environment_guard + r'''
assert_clean_shell_environment "$FIXTURE_ROOT/home"
[[ "$#" -eq 12 ]]
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
[[ "$11" == --p8-file-path ]]
[[ "$12" == "$HOME/.appstoreconnect/private_keys/AuthKey_TESTKEY123.p8" ]]
[[ ! -L "$12" && -f "$12" ]]
[[ ! -L "$ipa" && -f "$ipa" ]]
snapshot_directory="${ipa%/*}"
case "$snapshot_directory" in
  /tmp/biomotion-testflight.*|/private/tmp/biomotion-testflight.*) ;;
  *) exit 90 ;;
esac
[[ "$(/usr/bin/stat -f '%Lp' "$snapshot_directory")" == 700 ]]
[[ "$(/usr/bin/stat -f '%Lp' "$ipa")" == 600 ]]
[[ "$ipa" != "$FIXTURE_ROOT/export/BioMotion.ipa" ]]
printf '%s\n' "$event" >> "$EVENT_LOG"
printf '%s\n' "$ipa" >> "$FIXTURE_ROOT/snapshot.log"
if [[ "$event" == xcrun:validate && \
  -f "$CONTROL_DIRECTORY/mutate-snapshot-after-validate" ]]; then
  printf '%s\n' 'mutated after validation' >> "$ipa"
fi
'''

security = r'''#!/bin/bash
set -euo pipefail
FIXTURE_ROOT="$(cd -P -- "${BASH_SOURCE[0]%/*}/.." && pwd -P)"
''' + shell_environment_guard + r'''
assert_clean_shell_environment "$FIXTURE_ROOT/home"
[[ "$#" -eq 6 ]]
[[ "$1" == find-generic-password ]]
[[ "$2" == -a && "$3" == biomotion ]]
[[ "$4" == -s && "$6" == -w ]]
case "$5" in
  com.soleilyu.biomotion.appstoreconnect.key-id)
    printf '%s\n' 'keychain:key-id' >> "$EVENT_LOG"
    printf '%s\n' 'TESTKEY123'
    ;;
  com.soleilyu.biomotion.appstoreconnect.issuer)
    printf '%s\n' 'keychain:issuer' >> "$EVENT_LOG"
    printf '%s\n' '12345678-1234-1234-1234-123456789abc'
    ;;
  *) exit 91 ;;
esac
'''

(root / "tools/tests/dependency_boundary_probe.sh").write_text(
    dependency_gate, encoding="utf-8"
)
(root / "tools/release/dependency_archive_receipt.py").write_text(
    dependency_receipt, encoding="utf-8"
)
(root / "tools/tests/app_resource_boundary_probe.sh").write_text(
    resource_gate, encoding="utf-8"
)
(root / "tools/tests/privacy_manifest_probe.sh").write_text(
    privacy_gate, encoding="utf-8"
)
(root / "fake-tools/xcodebuild").write_text(xcodebuild, encoding="utf-8")
(root / "fake-tools/xcrun").write_text(xcrun, encoding="utf-8")
(root / "fake-tools/security").write_text(security, encoding="utf-8")

wrapper_path = root / "tools/release/testflight_release.sh"
wrapper = wrapper_path.read_text(encoding="utf-8")
replacements = {
    'XCODEBUILD="/usr/bin/xcodebuild"':
        f'XCODEBUILD="{root / "fake-tools/xcodebuild"}"',
    'XCRUN="/usr/bin/xcrun"':
        f'XCRUN="{root / "fake-tools/xcrun"}"',
    'SECURITY="/usr/bin/security"':
        f'SECURITY="{root / "fake-tools/security"}"',
}
for old, new in replacements.items():
    if wrapper.count(old) != 1:
        raise SystemExit(f"fixture replacement count changed for {old!r}")
    wrapper = wrapper.replace(old, new)
identity_validation = 'if [[ ! "$TRUSTED_UID" =~ ^[0-9]+$'
if wrapper.count(identity_validation) != 1:
    raise SystemExit("fixture trusted-HOME insertion point changed")
wrapper = wrapper.replace(
    identity_validation,
    f'TRUSTED_HOME="{root / "home"}"\n' + identity_validation,
)
wrapper_path.write_text(wrapper, encoding="utf-8")

(root / "injected-bash-env.sh").write_text(
    f"printf '%s\\n' 'unexpected:bash-env' >> '{root / 'injection.log'}'\n"
    "exit 0\n",
    encoding="utf-8",
)
(root / "injected-env.sh").write_text(
    f"printf '%s\\n' 'unexpected:env-loaded' >> '{root / 'injection.log'}'\n",
    encoding="utf-8",
)
PY

  /bin/chmod 0755 \
    "$root/tools/release/testflight_release.sh" \
    "$root/tools/release/dependency_archive_receipt.py" \
    "$root/tools/tests/dependency_boundary_probe.sh" \
    "$root/tools/tests/app_resource_boundary_probe.sh" \
    "$root/tools/tests/privacy_manifest_probe.sh" \
    "$root/fake-tools/xcodebuild" \
    "$root/fake-tools/xcrun" \
    "$root/fake-tools/security" \
    "$root/injected-bash-env.sh" \
    "$root/injected-env.sh"
  : > "$root/home/.appstoreconnect/private_keys/AuthKey_${TEST_KEY_ID}.p8"
  printf '%s\n' 'reviewed dependency receipt' > \
    "$root/BioMotion.xcarchive.dependency-receipt.json"
  /bin/chmod 0600 \
    "$root/BioMotion.xcarchive.dependency-receipt.json"
  /bin/chmod 0600 \
    "$root/home/.appstoreconnect/private_keys/AuthKey_${TEST_KEY_ID}.p8"
  /bin/chmod 0700 \
    "$root/home/.appstoreconnect" \
    "$root/home/.appstoreconnect/private_keys"
  printf '%s\n' "$root"
}

run_release() {
  local root="$1"
  shift
  local key_id="$TEST_KEY_ID"
  local issuer="$TEST_ISSUER"
  while [[ "$#" -gt 0 && "$1" != -- ]]; do
    case "$1" in
      FAKE_FAIL_GATE=*)
        printf '%s\n' "${1#*=}" > "$root/controls/fail-gate"
        ;;
      FAKE_SECOND_IPA=1)
        : > "$root/controls/second-ipa"
        ;;
      FAKE_MUTATE_SNAPSHOT_AFTER_VALIDATE=1)
        : > "$root/controls/mutate-snapshot-after-validate"
        ;;
      FAKE_CREATE_KEY_AT_IPA_GATE=1)
        : > "$root/controls/create-key-at-ipa-gate"
        ;;
      FAKE_IPA_RECEIPT_SYMLINK=1)
        printf '%s\n' 'protected receipt sentinel' > "$root/receipt-sentinel"
        : > "$root/controls/ipa-receipt-symlink"
        ;;
      ASC_API_KEY_ID=*) key_id="${1#*=}" ;;
      ASC_API_ISSUER=*) issuer="${1#*=}" ;;
      *) fail "unsupported fixture control: $1" ;;
    esac
    shift
  done
  [[ "$#" -gt 0 ]] || fail "run_release requires an argument separator"
  shift
  set +e
  (
    cd "$root"
    /usr/bin/env \
      PATH="$root/untrusted-path" \
      HOME="$root/untrusted-caller-home" \
      ASC_API_KEY_ID="$key_id" \
      ASC_API_ISSUER="$issuer" \
      DEVELOPER_DIR="$root/untrusted-developer" \
      TOOLCHAINS=untrusted-toolchain \
      SDKROOT="$root/untrusted-sdk" \
      PYTHONPATH="$root/untrusted-python" \
      PYTHONHOME="$root/untrusted-python-home" \
      PYTHONSTARTUP="$root/untrusted-python-startup" \
      PYTHONINSPECT=1 \
      PYTHONWARNINGS=error \
      PERL5OPT=-Mstrict \
      PERL5LIB="$root/untrusted-perl" \
      DYLD_LIBRARY_PATH="$root/untrusted-dyld" \
      DYLD_INSERT_LIBRARIES="$root/untrusted-dyld.dylib" \
      CPATH="$root/untrusted-cpath" \
      LIBRARY_PATH="$root/untrusted-library" \
      RUBYOPT=-w \
      XCODE_XCCONFIG_FILE="$root/untrusted.xcconfig" \
      BASH_ENV="$root/injected-bash-env.sh" \
      ENV="$root/injected-env.sh" \
      'BASH_FUNC_biomotion_injected%%=() { printf injected; }' \
      tools/release/testflight_release.sh "$@"
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
  $'dependency:current\ndependency:receipt:pre\nresource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa\ndependency:receipt:post' \
  "default local gate order"
assert_no_file "$root/injection.log" \
  "protected TestFlight entry point must ignore inherited BASH_ENV"
assert_no_file "$root/snapshot.log" "default mode must not invoke xcrun"
/usr/bin/grep -Fq 'TESTFLIGHT_RELEASE_PASS export-only' "$root/output.txt" || \
  fail "default mode did not report export-only success"
pass "default mode is credential-blind and makes no explicit App Store Connect call"

# A release archive without its exact dependency receipt is never exportable.
root="$(make_fixture missing_dependency_receipt)"
/bin/rm "$root/BioMotion.xcarchive.dependency-receipt.json"
standard_arguments "$root"
run_release "$root" -- "${STANDARD_ARGUMENTS[@]}"
assert_status 1 "$RUN_STATUS" "missing dependency receipt"
assert_no_file "$root/events.log" \
  "missing dependency receipt must precede all executable gates"
assert_no_file "$root/export/BioMotion.ipa" \
  "missing dependency receipt must block export"
pass "an archive without its dependency receipt is rejected before export"

# Current native inputs must still match the reviewed dependency lock.
root="$(make_fixture dependency_gate_failure)"
standard_arguments "$root"
run_release "$root" FAKE_FAIL_GATE=dependency -- "${STANDARD_ARGUMENTS[@]}"
assert_status 40 "$RUN_STATUS" "dependency gate failure"
assert_file_text "$root/events.log" $'dependency:current' \
  "dependency failure causal order"
assert_no_file "$root/export/BioMotion.ipa" \
  "dependency failure must block export"
pass "current dependency drift blocks receipt review and export"

# A stale or byte-mismatched archive receipt dominates all source/export work.
root="$(make_fixture dependency_receipt_failure)"
standard_arguments "$root"
run_release "$root" FAKE_FAIL_GATE=receipt-pre -- "${STANDARD_ARGUMENTS[@]}"
assert_status 41 "$RUN_STATUS" "dependency receipt failure"
assert_file_text "$root/events.log" \
  $'dependency:current\ndependency:receipt:pre' \
  "dependency receipt failure causal order"
assert_no_file "$root/export/BioMotion.ipa" \
  "dependency receipt failure must block export"
pass "stale dependency receipts and changed archive bytes block export"

# Export may not mutate the reviewed archive before any external operation.
root="$(make_fixture post_export_receipt_failure)"
standard_arguments "$root"
run_release "$root" FAKE_FAIL_GATE=receipt-post -- "${STANDARD_ARGUMENTS[@]}"
assert_status 41 "$RUN_STATUS" "post-export dependency receipt failure"
assert_file_text "$root/events.log" \
  $'dependency:current\ndependency:receipt:pre\nresource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa\ndependency:receipt:post' \
  "post-export dependency receipt failure causal order"
assert_no_file "$root/snapshot.log" \
  "post-export receipt failure must block App Store Connect"
assert_no_file "$root/export/BioMotion.ipa.sha256" \
  "post-export receipt failure must block IPA receipt publication"
pass "archive bytes are reverified after local export and IPA comparison"

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
  $'dependency:current\ndependency:receipt:pre\nresource:source\nresource:archive' \
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
  $'dependency:current\ndependency:receipt:pre\nresource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa\ndependency:receipt:post' \
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

# Xcode output cannot redirect the adjacent SHA receipt through a symlink.
root="$(make_fixture ipa_receipt_symlink)"
standard_arguments "$root"
run_release "$root" FAKE_IPA_RECEIPT_SYMLINK=1 -- "${STANDARD_ARGUMENTS[@]}"
[[ "$RUN_STATUS" -ne 0 ]] || fail "symlink IPA receipt unexpectedly succeeded"
assert_file_text "$root/receipt-sentinel" 'protected receipt sentinel' \
  "exclusive IPA receipt writer must not follow a symlink"
[[ -L "$root/export/BioMotion.ipa.sha256" ]] || \
  fail "fixture IPA receipt symlink disappeared unexpectedly"
assert_no_file "$root/snapshot.log" \
  "IPA receipt collision must precede App Store Connect"
pass "IPA SHA receipts are exclusive private files and never follow symlinks"

# Validation performs exactly one external operation and cleans its snapshot.
root="$(make_fixture validate_only)"
standard_arguments "$root"
run_release "$root" -- --validate "${STANDARD_ARGUMENTS[@]}"
assert_status 0 "$RUN_STATUS" "validate mode"
assert_file_text "$root/events.log" \
  $'dependency:current\ndependency:receipt:pre\nresource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa\ndependency:receipt:post\nxcrun:validate' \
  "validate-only order"
assert_snapshot_removed "$root/snapshot.log" "validate-only cleanup"
pass "--validate validates the private snapshot and cannot upload"

# Upload is authorized only as validation followed by upload of the same bytes.
root="$(make_fixture upload_success)"
standard_arguments "$root"
run_release "$root" -- --upload "${STANDARD_ARGUMENTS[@]}"
assert_status 0 "$RUN_STATUS" "upload mode"
assert_file_text "$root/events.log" \
  $'dependency:current\ndependency:receipt:pre\nresource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa\ndependency:receipt:post\nxcrun:validate\nxcrun:upload' \
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

# With no per-run environment values, the upload path retrieves the stable
# workstation references from Keychain only after every local gate has passed.
root="$(make_fixture keychain_credentials)"
standard_arguments "$root"
run_release "$root" \
  ASC_API_KEY_ID= \
  ASC_API_ISSUER= \
  -- --upload "${STANDARD_ARGUMENTS[@]}"
assert_status 0 "$RUN_STATUS" "Keychain credential fallback"
assert_file_text "$root/events.log" \
  $'dependency:current\ndependency:receipt:pre\nresource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa\ndependency:receipt:post\nkeychain:key-id\nkeychain:issuer\nxcrun:validate\nxcrun:upload' \
  "Keychain credential fallback order"
assert_snapshot_removed "$root/snapshot.log" "Keychain credential cleanup"
pass "--upload falls back to owner Keychain references after local gates"

# A validator-side mutation must be detected before the upload call.
root="$(make_fixture digest_change)"
standard_arguments "$root"
run_release "$root" \
  FAKE_MUTATE_SNAPSHOT_AFTER_VALIDATE=1 \
  -- --upload "${STANDARD_ARGUMENTS[@]}"
assert_status 1 "$RUN_STATUS" "digest mutation"
assert_file_text "$root/events.log" \
  $'dependency:current\ndependency:receipt:pre\nresource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa\ndependency:receipt:post\nxcrun:validate' \
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
  $'dependency:current\ndependency:receipt:pre\nresource:source\nresource:archive\nprivacy:archive\nxcodebuild:export\nresource:ipa\ndependency:receipt:post\nxcrun:validate' \
  "deferred credential order"
pass "ASC credentials and key path are inaccessible until every local gate passes"

# Credentials are selected by one explicit private path, never altool's search
# order, and unsafe key storage blocks the first network operation.
for key_case in readable_key symlink_key_directory; do
  root="$(make_fixture "unsafe_key_$key_case")"
  case "$key_case" in
    readable_key)
      /bin/chmod 0644 \
        "$root/home/.appstoreconnect/private_keys/AuthKey_${TEST_KEY_ID}.p8"
      ;;
    symlink_key_directory)
      /bin/mv "$root/home/.appstoreconnect/private_keys" \
        "$root/private-keys-target"
      /bin/ln -s "$root/private-keys-target" \
        "$root/home/.appstoreconnect/private_keys"
      ;;
  esac
  standard_arguments "$root"
  run_release "$root" -- --validate "${STANDARD_ARGUMENTS[@]}"
  [[ "$RUN_STATUS" -ne 0 ]] || fail "$key_case unexpectedly reached validation"
  assert_no_file "$root/snapshot.log" \
    "$key_case must block xcrun before a network operation"
done
pass "App Store Connect keys require an explicit 0600 path in private physical directories"

# Two IPA files are never ambiguously selected for validation or upload.
root="$(make_fixture multiple_ipas)"
standard_arguments "$root"
run_release "$root" FAKE_SECOND_IPA=1 -- --validate "${STANDARD_ARGUMENTS[@]}"
assert_status 1 "$RUN_STATUS" "multiple IPA rejection"
assert_file_text "$root/events.log" \
  $'dependency:current\ndependency:receipt:pre\nresource:source\nresource:archive\nprivacy:archive\nxcodebuild:export' \
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

printf 'TESTFLIGHT_RELEASE_GATE_TESTS_PASS %s/16\n' "$PASS_COUNT"
