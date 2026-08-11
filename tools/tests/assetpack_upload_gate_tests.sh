#!/bin/bash
# Hermetic causal tests for the receipt-first asset-pack upload gate.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-upload-tests.XXXXXX")"
TEST_ROOT="$(cd "$TEST_ROOT" && pwd -P)"
trap 'rm -rf "$TEST_ROOT"' EXIT

PASS_COUNT=0

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  echo "PASS: $*"
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

assert_file_lines() {
  local path="$1"
  local expected="$2"
  local label="$3"
  [[ -f "$path" ]] || fail "$label: missing file $path"
  local actual
  actual="$(/bin/cat "$path")"
  [[ "$actual" == "$expected" ]] || {
    echo "expected:" >&2
    echo "$expected" >&2
    echo "actual:" >&2
    echo "$actual" >&2
    fail "$label: content mismatch"
  }
}

assert_verified_snapshot_removed() {
  local verifier_log="$1"
  local label="$2"
  /usr/bin/python3 - "$verifier_log" "$label" <<'PY'
import json
from pathlib import Path
import sys

arguments = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
snapshot_directory = Path(arguments[1]).parent
if snapshot_directory.exists():
    raise SystemExit(f"{sys.argv[2]}: snapshot still exists: {snapshot_directory}")
PY
}

make_fixture() {
  local name="$1"
  local root="$TEST_ROOT/$name"
  /bin/mkdir -p \
    "$root/tools/assetpack" \
    "$root/build/assetpack/release" \
    "$root/fake-tools" \
    "$root/fake-path" \
    "$root/home"
  /bin/cp "$REPO_ROOT/tools/assetpack/upload.sh" "$root/tools/assetpack/upload.sh"
  /bin/cp \
    "$REPO_ROOT/tools/tests/assetpack_upload_fake_verifier.py" \
    "$root/tools/assetpack/verify_model_lock.py"
  /bin/cp \
    "$REPO_ROOT/tools/tests/assetpack_upload_fake_xcrun.py" \
    "$root/fake-tools/xcrun"
  /bin/cp "$root/fake-tools/xcrun" "$root/fake-path/xcrun"
  /bin/chmod 0755 \
    "$root/tools/assetpack/upload.sh" \
    "$root/tools/assetpack/verify_model_lock.py" \
    "$root/fake-tools/xcrun" \
    "$root/fake-path/xcrun"
  /usr/bin/python3 - "$root" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
upload_path = root / "tools/assetpack/upload.sh"
upload_text = upload_path.read_text(encoding="utf-8")
needle = 'XCRUN="/usr/bin/xcrun"'
if upload_text.count(needle) != 1:
    raise SystemExit("cannot install the exact fixture-only xcrun path")
upload_path.write_text(
    upload_text.replace(needle, f'XCRUN="{root / "fake-tools/xcrun"}"'),
    encoding="utf-8",
)
poison_directory = root / "python-poison"
poison_directory.mkdir()
(poison_directory / "sitecustomize.py").write_text(
    "import os\n"
    "from pathlib import Path\n"
    "Path(os.environ['FAKE_PYTHON_POISON_LOG']).write_text('loaded\\n')\n",
    encoding="utf-8",
)
PY
  : > "$root/build/assetpack/release/sam3d-body-pose.aar"
  : > "$root/build/assetpack/release/sam3d-body-pose.aar.receipt.json"
  echo "$root"
}

run_gate() {
  local root="$1"
  shift
  local environment=()
  while [[ "$#" -gt 0 && "$1" != "--" ]]; do
    environment+=("$1")
    shift
  done
  [[ "$#" -gt 0 ]] || fail "run_gate requires an environment/argument separator"
  shift
  local output="$root/output.txt"
  local aar="$root/build/assetpack/release/sam3d-body-pose.aar"
  local receipt="$aar.receipt.json"
  set +e
  (
    cd "$root"
    /usr/bin/env \
      PATH="$root/fake-path:/usr/bin:/bin" \
      HOME="$root/home" \
      FAKE_EVENT_LOG="$root/events.log" \
      FAKE_VERIFIER_LOG="$root/verifier.log" \
      FAKE_VERIFIER_MARKER="$root/verified.marker" \
      FAKE_XCRUN_LOG="$root/xcrun.log" \
      FAKE_EXPECTED_AAR="$aar" \
      FAKE_EXPECTED_RECEIPT="$receipt" \
      "${environment[@]}" \
      /bin/bash tools/assetpack/upload.sh "$@"
  ) >"$output" 2>&1
  RUN_STATUS=$?
  set -e
}

# The default is a credential-free receipt check of the atomic release pair.
root="$(make_fixture default_verify)"
run_gate "$root" \
  ASC_API_KEY_ID= \
  ASC_API_ISSUER= \
  BIOMOTION_ASC_APP_ID= \
  PYTHONPATH="$root/python-poison" \
  FAKE_PYTHON_POISON_LOG="$root/python-poison.log" \
  DEVELOPER_DIR="$root/untrusted-xcode" \
  TOOLCHAINS=untrusted-toolchain \
  SDKROOT="$root/untrusted-sdk" \
  FAKE_UNTRUSTED_DEVELOPER_DIR="$root/untrusted-xcode" \
  FAKE_UNTRUSTED_TOOLCHAINS=untrusted-toolchain \
  FAKE_UNTRUSTED_SDKROOT="$root/untrusted-sdk" \
  --
assert_status 0 "$RUN_STATUS" "default verification"
assert_file_lines "$root/events.log" $'verifier:start\nverifier:pass' \
  "default verification order"
assert_no_file "$root/xcrun.log" "default verification must not invoke altool"
assert_no_file "$root/python-poison.log" "isolated verifier must ignore PYTHONPATH"
pass "default mode verifies the release AAR/receipt pair without credentials"

# An explicit read-only mode remains read-only even when credentials are present.
root="$(make_fixture explicit_verify)"
/bin/mkdir -p "$root/home/.appstoreconnect/private_keys"
: > "$root/home/.appstoreconnect/private_keys/AuthKey_TESTKEY123.p8"
run_gate "$root" \
  ASC_API_KEY_ID=TESTKEY123 \
  ASC_API_ISSUER=12345678-1234-1234-1234-123456789abc \
  FAKE_CREATE_KEY=0 \
  -- --verify-only
assert_status 0 "$RUN_STATUS" "explicit verification"
assert_file_lines "$root/events.log" $'verifier:start\nverifier:pass' \
  "explicit verification order"
assert_no_file "$root/xcrun.log" "--verify-only must not invoke altool"
pass "--verify-only cannot upload or list versions"

# A failed receipt gate must win over missing credentials and prevent altool.
root="$(make_fixture verifier_failure)"
run_gate "$root" \
  FAKE_VERIFIER_EXIT=42 \
  FAKE_EXPECT_SNAPSHOT=1 \
  ASC_API_KEY_ID=TESTKEY123 \
  ASC_API_ISSUER=12345678-1234-1234-1234-123456789abc \
  -- --upload
assert_status 42 "$RUN_STATUS" "receipt failure propagation"
assert_file_lines "$root/events.log" $'verifier:start\nverifier:fail' \
  "receipt failure order"
assert_no_file "$root/xcrun.log" "receipt failure must not invoke altool"
assert_verified_snapshot_removed "$root/verifier.log" "receipt failure cleanup"
if /usr/bin/grep -Eq 'AuthKey_|API key|ASC_API' "$root/output.txt"; then
  fail "receipt failure inspected or diagnosed credentials before returning"
fi
pass "receipt failure occurs before credential-path access and causes zero altool calls"

# The fake verifier creates the key: upload can only succeed if verification ran first.
root="$(make_fixture upload_success)"
run_gate "$root" \
  FAKE_CREATE_KEY=1 \
  FAKE_EXPECT_SNAPSHOT=1 \
  FAKE_MUTATE_SOURCE_AFTER_VERIFY=1 \
  ASC_API_KEY_ID=TESTKEY123 \
  ASC_API_ISSUER=12345678-1234-1234-1234-123456789abc \
  DEVELOPER_DIR="$root/untrusted-xcode" \
  TOOLCHAINS=untrusted-toolchain \
  SDKROOT="$root/untrusted-sdk" \
  FAKE_UNTRUSTED_DEVELOPER_DIR="$root/untrusted-xcode" \
  FAKE_UNTRUSTED_TOOLCHAINS=untrusted-toolchain \
  FAKE_UNTRUSTED_SDKROOT="$root/untrusted-sdk" \
  -- --upload
assert_status 0 "$RUN_STATUS" "authorized upload"
assert_file_lines "$root/events.log" \
  $'verifier:start\nverifier:pass\nxcrun:upload\nxcrun:list' \
  "authorized upload order"
/usr/bin/python3 - "$root/xcrun.log" "$root/verifier.log" \
  "$root/build/assetpack/release/sam3d-body-pose.aar" <<'PY'
import json
from pathlib import Path
import sys

rows = [json.loads(line) for line in Path(sys.argv[1]).read_text().splitlines()]
verifier_arguments = json.loads(Path(sys.argv[2]).read_text())
source_aar = sys.argv[3]
aar = verifier_arguments[1]
if verifier_arguments != ["receipt", aar, f"{aar}.receipt.json"]:
    raise SystemExit(f"unexpected verifier arguments: {verifier_arguments!r}")
if aar == source_aar:
    raise SystemExit("upload did not verify a private snapshot")
if Path(aar).parent.exists():
    raise SystemExit("private upload snapshot was not removed")
expected = [
    [
        "altool", "--upload-asset-pack", aar,
        "--apple-id", "6761994383", "--platform", "ios",
        "--apiKey", "TESTKEY123", "--apiIssuer",
        "12345678-1234-1234-1234-123456789abc",
    ],
    [
        "altool", "--list-asset-pack-versions",
        "--apple-id", "6761994383", "--asset-pack-identifier",
        "sam3d-body-pose", "--apiKey", "TESTKEY123", "--apiIssuer",
        "12345678-1234-1234-1234-123456789abc",
    ],
]
if rows != expected:
    raise SystemExit(f"unexpected xcrun calls: {rows!r}")
PY
pass "--upload pins verified snapshot bytes across source replacement, then lists"

# Custom locations are allowed only as an inseparable, canonically named pair.
root="$(make_fixture custom_pair)"
/bin/mkdir -p "$root/private-candidate"
: > "$root/private-candidate/sam3d-body-pose.aar"
: > "$root/private-candidate/sam3d-body-pose.aar.receipt.json"
custom_aar="$root/private-candidate/sam3d-body-pose.aar"
custom_receipt="$custom_aar.receipt.json"
set +e
(
  cd "$root"
  /usr/bin/env \
    PATH="$root/fake-path:/usr/bin:/bin" \
    HOME="$root/absent-home" \
    FAKE_EVENT_LOG="$root/events.log" \
    FAKE_VERIFIER_LOG="$root/verifier.log" \
    FAKE_VERIFIER_MARKER="$root/verified.marker" \
    FAKE_XCRUN_LOG="$root/xcrun.log" \
    FAKE_EXPECTED_AAR="$custom_aar" \
    FAKE_EXPECTED_RECEIPT="$custom_receipt" \
    /bin/bash tools/assetpack/upload.sh --verify-only \
      --aar "$custom_aar" --receipt "$custom_receipt"
) >"$root/output.txt" 2>&1
status=$?
set -e
assert_status 0 "$status" "custom atomic pair"
assert_file_lines "$root/events.log" $'verifier:start\nverifier:pass' \
  "custom pair verification"
pass "custom AAR and receipt flags preserve the same-directory exact-name contract"

for case_name in aar_only receipt_only wrong_name split_pair unknown positional conflict help_conflict; do
  root="$(make_fixture "args_$case_name")"
  /bin/mkdir -p "$root/one" "$root/two"
  : > "$root/one/sam3d-body-pose.aar"
  : > "$root/one/sam3d-body-pose.aar.receipt.json"
  : > "$root/two/sam3d-body-pose.aar.receipt.json"
  case "$case_name" in
    aar_only)
      arguments=(--aar "$root/one/sam3d-body-pose.aar")
      ;;
    receipt_only)
      arguments=(--receipt "$root/one/sam3d-body-pose.aar.receipt.json")
      ;;
    wrong_name)
      arguments=(--aar "$root/one/other.aar" --receipt "$root/one/other.receipt.json")
      ;;
    split_pair)
      arguments=(--aar "$root/one/sam3d-body-pose.aar" --receipt "$root/two/sam3d-body-pose.aar.receipt.json")
      ;;
    unknown)
      arguments=(--surprise)
      ;;
    positional)
      arguments=("$root/one/sam3d-body-pose.aar")
      ;;
    conflict)
      arguments=(--verify-only --upload)
      ;;
    help_conflict)
      arguments=(--upload --help)
      ;;
  esac
  set +e
  (
    cd "$root"
    /usr/bin/env \
      PATH="$root/fake-path:/usr/bin:/bin" \
      HOME="$root/home" \
      FAKE_EVENT_LOG="$root/events.log" \
      FAKE_VERIFIER_LOG="$root/verifier.log" \
      FAKE_VERIFIER_MARKER="$root/verified.marker" \
      FAKE_XCRUN_LOG="$root/xcrun.log" \
      FAKE_EXPECTED_AAR="$root/build/assetpack/release/sam3d-body-pose.aar" \
      FAKE_EXPECTED_RECEIPT="$root/build/assetpack/release/sam3d-body-pose.aar.receipt.json" \
      /bin/bash tools/assetpack/upload.sh "${arguments[@]}"
  ) >"$root/output.txt" 2>&1
  status=$?
  set -e
  assert_status 64 "$status" "$case_name argument error"
  assert_no_file "$root/verifier.log" "$case_name must fail before verification"
  assert_no_file "$root/xcrun.log" "$case_name must not invoke altool"
done
pass "malformed, partial, conflicting, and bypass-shaped arguments fail closed"

# Every credential/key failure happens only after the verified marker and before xcrun.
for case_name in missing_id missing_issuer missing_home missing_key symlink_key bad_id bad_issuer bad_app_id directory_key; do
  root="$(make_fixture "credential_$case_name")"
  /bin/mkdir -p "$root/home/.appstoreconnect/private_keys"
  case "$case_name" in
    missing_id)
      credential_env=(ASC_API_KEY_ID= ASC_API_ISSUER=12345678-1234-1234-1234-123456789abc)
      ;;
    missing_issuer)
      credential_env=(ASC_API_KEY_ID=TESTKEY123 ASC_API_ISSUER=)
      ;;
    missing_home)
      : > "$root/home/.appstoreconnect/private_keys/AuthKey_TESTKEY123.p8"
      credential_env=(HOME= ASC_API_KEY_ID=TESTKEY123 ASC_API_ISSUER=12345678-1234-1234-1234-123456789abc)
      ;;
    missing_key)
      credential_env=(ASC_API_KEY_ID=TESTKEY123 ASC_API_ISSUER=12345678-1234-1234-1234-123456789abc)
      ;;
    symlink_key)
      : > "$root/key-target.p8"
      /bin/ln -s "$root/key-target.p8" \
        "$root/home/.appstoreconnect/private_keys/AuthKey_TESTKEY123.p8"
      credential_env=(ASC_API_KEY_ID=TESTKEY123 ASC_API_ISSUER=12345678-1234-1234-1234-123456789abc)
      ;;
    bad_id)
      credential_env=(ASC_API_KEY_ID=../../key ASC_API_ISSUER=12345678-1234-1234-1234-123456789abc)
      ;;
    bad_issuer)
      credential_env=(ASC_API_KEY_ID=TESTKEY123 ASC_API_ISSUER=not-a-uuid)
      ;;
    bad_app_id)
      credential_env=(ASC_API_KEY_ID=TESTKEY123 ASC_API_ISSUER=12345678-1234-1234-1234-123456789abc BIOMOTION_ASC_APP_ID=not-numeric)
      ;;
    directory_key)
      /bin/mkdir \
        "$root/home/.appstoreconnect/private_keys/AuthKey_TESTKEY123.p8"
      credential_env=(ASC_API_KEY_ID=TESTKEY123 ASC_API_ISSUER=12345678-1234-1234-1234-123456789abc)
      ;;
  esac
  set +e
  (
    cd "$root"
    /usr/bin/env \
      PATH="$root/fake-path:/usr/bin:/bin" \
      HOME="$root/home" \
      FAKE_EVENT_LOG="$root/events.log" \
      FAKE_VERIFIER_LOG="$root/verifier.log" \
      FAKE_VERIFIER_MARKER="$root/verified.marker" \
      FAKE_XCRUN_LOG="$root/xcrun.log" \
      FAKE_EXPECTED_AAR="$root/build/assetpack/release/sam3d-body-pose.aar" \
      FAKE_EXPECTED_RECEIPT="$root/build/assetpack/release/sam3d-body-pose.aar.receipt.json" \
      FAKE_EXPECT_SNAPSHOT=1 \
      "${credential_env[@]}" \
      /bin/bash tools/assetpack/upload.sh --upload
  ) >"$root/output.txt" 2>&1
  status=$?
  set -e
  [[ "$status" -ne 0 ]] || fail "$case_name unexpectedly succeeded"
  assert_file_lines "$root/events.log" $'verifier:start\nverifier:pass' \
    "$case_name verification ordering"
  assert_no_file "$root/xcrun.log" "$case_name must not invoke altool"
  assert_verified_snapshot_removed "$root/verifier.log" "$case_name cleanup"
done
pass "malformed credentials and non-regular, missing, or symlink keys fail after verification"

# Source symlinks are rejected rather than silently dereferenced into a snapshot.
for case_name in symlink_aar symlink_receipt; do
  root="$(make_fixture "source_$case_name")"
  aar="$root/build/assetpack/release/sam3d-body-pose.aar"
  receipt="$aar.receipt.json"
  case "$case_name" in
    symlink_aar)
      /bin/rm "$aar"
      : > "$root/source-target.aar"
      /bin/ln -s "$root/source-target.aar" "$aar"
      ;;
    symlink_receipt)
      /bin/rm "$receipt"
      : > "$root/source-target.receipt.json"
      /bin/ln -s "$root/source-target.receipt.json" "$receipt"
      ;;
  esac
  run_gate "$root" \
    ASC_API_KEY_ID=TESTKEY123 \
    ASC_API_ISSUER=12345678-1234-1234-1234-123456789abc \
    -- --upload
  [[ "$RUN_STATUS" -ne 0 ]] || fail "$case_name source unexpectedly succeeded"
  assert_no_file "$root/verifier.log" "$case_name must fail before verifier"
  assert_no_file "$root/xcrun.log" "$case_name must not invoke altool"
  if /usr/bin/grep -Eq 'AuthKey_|API key|ASC_API' "$root/output.txt"; then
    fail "$case_name inspected credentials before source rejection"
  fi
done
pass "source symlinks fail before verifier without credential or altool access"

# Upload failure cannot fall through to the post-upload list operation.
root="$(make_fixture upload_failure)"
run_gate "$root" \
  FAKE_CREATE_KEY=1 \
  FAKE_EXPECT_SNAPSHOT=1 \
  FAKE_UPLOAD_EXIT=23 \
  ASC_API_KEY_ID=TESTKEY123 \
  ASC_API_ISSUER=12345678-1234-1234-1234-123456789abc \
  -- --upload
assert_status 23 "$RUN_STATUS" "upload failure propagation"
assert_file_lines "$root/events.log" \
  $'verifier:start\nverifier:pass\nxcrun:upload' \
  "upload failure ordering"
assert_verified_snapshot_removed "$root/verifier.log" "upload failure cleanup"
pass "upload failure propagates and suppresses version listing"

# A list failure propagates only after the one explicitly authorized upload.
root="$(make_fixture list_failure)"
run_gate "$root" \
  FAKE_CREATE_KEY=1 \
  FAKE_EXPECT_SNAPSHOT=1 \
  FAKE_LIST_EXIT=24 \
  ASC_API_KEY_ID=TESTKEY123 \
  ASC_API_ISSUER=12345678-1234-1234-1234-123456789abc \
  -- --upload
assert_status 24 "$RUN_STATUS" "list failure propagation"
assert_file_lines "$root/events.log" \
  $'verifier:start\nverifier:pass\nxcrun:upload\nxcrun:list' \
  "list failure ordering"
assert_verified_snapshot_removed "$root/verifier.log" "list failure cleanup"
pass "post-upload list failure propagates within the same explicit authorization"

# PATH is poisoned throughout; verifier and upload tool must be absolute choices.
if [[ "$(/usr/bin/sed -n '1p' "$REPO_ROOT/tools/assetpack/upload.sh")" != \
  '#!/bin/bash' ]]; then
  fail "upload gate does not pin /bin/bash"
fi
if ! /usr/bin/grep -Fq 'PATH="/usr/bin:/bin:/usr/sbin:/sbin"' \
  "$REPO_ROOT/tools/assetpack/upload.sh"; then
  fail "upload gate does not pin its trusted system PATH"
fi
if ! /usr/bin/grep -Fq \
  "/usr/bin/python3 -I \"\$VERIFIER\" receipt \"\$AAR\" \"\$RECEIPT\"" \
  "$REPO_ROOT/tools/assetpack/upload.sh"; then
  fail "upload gate does not pin the receipt verifier interpreter"
fi
if ! /usr/bin/grep -Fq 'XCRUN="/usr/bin/xcrun"' \
  "$REPO_ROOT/tools/assetpack/upload.sh"; then
  fail "upload gate does not default to absolute /usr/bin/xcrun"
fi
if /usr/bin/grep -Fq 'BIOMOTION_ASSETPACK_XCRUN' \
  "$REPO_ROOT/tools/assetpack/upload.sh"; then
  fail "upload gate contains a production xcrun override"
fi
if /usr/bin/grep -Fq 'BIOMOTION_ASC_APP_ID' \
  "$REPO_ROOT/tools/assetpack/upload.sh"; then
  fail "upload gate retains an ambient App Store target override"
fi
if ! /usr/bin/grep -Fq 'unset DEVELOPER_DIR TOOLCHAINS SDKROOT' \
  "$REPO_ROOT/tools/assetpack/upload.sh"; then
  fail "upload gate retains ambient Xcode tool-selection overrides"
fi
[[ -x "$REPO_ROOT/tools/assetpack/upload.sh" ]] || \
  fail "upload gate lost its executable mode"
pass "trusted tools are PATH-independent and absolute"

echo "ASSETPACK_UPLOAD_GATE_TEST_PASS count=$PASS_COUNT"
