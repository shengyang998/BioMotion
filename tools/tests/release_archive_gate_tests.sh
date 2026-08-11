#!/bin/bash -p
# Hermetic causal tests for the controlled signed-archive wrapper.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
TEST_ROOT="$(/usr/bin/mktemp -d \
  "${TMPDIR:-/tmp}/biomotion-archive-release-tests.XXXXXX")"
TEST_ROOT="$(cd "$TEST_ROOT" && pwd -P)"
trap '/bin/rm -rf "$TEST_ROOT"' EXIT

PASS_COUNT=0

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

assert_no_published_output() {
  local root="$1"
  local archive="$root/artifacts/BioMotion.xcarchive"
  local receipt="$archive.dependency-receipt.json"
  [[ ! -e "$archive" && ! -L "$archive" ]] || \
    fail "unexpected published archive: $archive"
  [[ ! -e "$receipt" && ! -L "$receipt" ]] || \
    fail "unexpected published receipt: $receipt"
}

assert_no_staging() {
  local root="$1"
  local found
  found="$(/usr/bin/find "$root/artifacts" -mindepth 1 -maxdepth 1 \
    -name '.biomotion-archive.*' -print -quit)"
  [[ -z "$found" ]] || fail "private staging path was not removed: $found"
}

make_fixture() {
  local name="$1"
  local root="$TEST_ROOT/$name"
  /bin/mkdir -p \
    "$root/tools/release" \
    "$root/tools/tests" \
    "$root/fake-tools" \
    "$root/artifacts"
  /bin/chmod 0700 "$root/artifacts"
  /bin/cp "$REPO_ROOT/tools/release/archive_release.sh" \
    "$root/tools/release/archive_release.sh"

  /usr/bin/python3 - "$root" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])

dependency_gate = r'''#!/bin/bash
set -euo pipefail
fixture_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
event_log="$fixture_root/events.log"
count_path="$fixture_root/dependency-count"
fail_stage=""
if [[ -f "$fixture_root/fail-stage" ]]; then
  fail_stage="$(/bin/cat "$fixture_root/fail-stage")"
fi
if [[ "$PATH" != /usr/bin:/bin:/usr/sbin:/sbin || \
  "$HOME" != /var/empty || \
  -n "${ASC_API_KEY_ID+x}" || -n "${ASC_API_ISSUER+x}" ]]; then
  printf '%s\n' 'credentials-or-path:leaked' >> "$event_log"
  exit 97
fi
if [[ -n "${DEVELOPER_DIR+x}" || -n "${TOOLCHAINS+x}" || \
  -n "${SDKROOT+x}" || -n "${PYTHONPATH+x}" || \
  -n "${PYTHONHOME+x}" || -n "${XCODE_XCCONFIG_FILE+x}" || \
  -n "${BASH_ENV+x}" || -n "${ENV+x}" || \
  -n "${BIOMOTION_BASH_ENV_WAS_SOURCED+x}" ]]; then
  printf '%s\n' 'environment:leaked' >> "$event_log"
  exit 96
fi
[[ "$#" -eq 1 && "$1" == --snapshot ]] || exit 94
lock_output="$(/usr/bin/shasum -a 256 \
  "$fixture_root/tools/dependencies.lock.json")"
lock_digest="${lock_output%% *}"
count=0
if [[ -f "$count_path" ]]; then
  count="$(/bin/cat "$count_path")"
fi
count=$((count + 1))
printf '%s\n' "$count" > "$count_path"
case "$count" in
  1)
    printf '%s\n' 'dependency:pre' >> "$event_log"
    [[ "$fail_stage" != dependency-pre ]] || exit 41
    if [[ "$fail_stage" == invalid-snapshot-pre ]]; then
      printf '%s\n' '{"fixture":"stable","schemaVersion":1}'
    else
      printf '{"dependencyLockSHA256":"%s","fixture":"stable","schemaVersion":1}\n' \
        "$lock_digest"
    fi
    ;;
  2)
    printf '%s\n' 'dependency:post' >> "$event_log"
    [[ "$fail_stage" != dependency-post ]] || exit 42
    if [[ "$fail_stage" == invalid-snapshot-post ]]; then
      printf '%s\n' '{"fixture":"stable","schemaVersion":1}'
    elif [[ "$fail_stage" == boundary-drift ]]; then
      printf '{"dependencyLockSHA256":"%s","fixture":"changed","schemaVersion":1}\n' \
        "$lock_digest"
    else
      printf '{"dependencyLockSHA256":"%s","fixture":"stable","schemaVersion":1}\n' \
        "$lock_digest"
      if [[ "$fail_stage" == seal-boundary-drift ]]; then
        printf '%s\n' changed > "$fixture_root/seal-observation"
      fi
    fi
    ;;
  *) exit 95 ;;
esac
'''

resource_gate = r'''#!/bin/bash
set -euo pipefail
fixture_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
event_log="$fixture_root/events.log"
fail_stage=""
if [[ -f "$fixture_root/fail-stage" ]]; then
  fail_stage="$(/bin/cat "$fixture_root/fail-stage")"
fi
if [[ "$PATH" != /usr/bin:/bin:/usr/sbin:/sbin || \
  "$HOME" != /var/empty || \
  -n "${ASC_API_KEY_ID+x}" || -n "${ASC_API_ISSUER+x}" ]]; then
  printf '%s\n' 'credentials-or-path:leaked' >> "$event_log"
  exit 97
fi
if [[ -n "${DEVELOPER_DIR+x}" || -n "${TOOLCHAINS+x}" || \
  -n "${SDKROOT+x}" || -n "${PYTHONPATH+x}" || \
  -n "${PYTHONHOME+x}" || -n "${XCODE_XCCONFIG_FILE+x}" || \
  -n "${BASH_ENV+x}" || -n "${ENV+x}" || \
  -n "${BIOMOTION_BASH_ENV_WAS_SOURCED+x}" ]]; then
  printf '%s\n' 'environment:leaked' >> "$event_log"
  exit 96
fi
case "$#:${1:-}" in
  0:)
    printf '%s\n' 'resource:source' >> "$event_log"
    [[ "$fail_stage" != resource-source ]] || exit 43
    ;;
  2:--release-archive)
    expected="$(/bin/cat "$fixture_root/staging-path")"
    [[ "$2" == "$expected" ]] || exit 94
    printf '%s\n' 'resource:archive' >> "$event_log"
    [[ "$fail_stage" != resource-archive ]] || exit 44
    if [[ "$fail_stage" == archive-identity-drift ]]; then
      /bin/mv "$2" "$2.replaced"
      /bin/mkdir "$2"
    fi
    ;;
  *) exit 93 ;;
esac
'''

privacy_gate = r'''#!/bin/bash
set -euo pipefail
fixture_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
fail_stage=""
if [[ -f "$fixture_root/fail-stage" ]]; then
  fail_stage="$(/bin/cat "$fixture_root/fail-stage")"
fi
[[ "$PATH" == /usr/bin:/bin:/usr/sbin:/sbin ]]
[[ "$HOME" == /var/empty ]]
[[ -z "${ASC_API_KEY_ID+x}" && -z "${ASC_API_ISSUER+x}" ]]
[[ -z "${DEVELOPER_DIR+x}" && -z "${TOOLCHAINS+x}" && \
  -z "${SDKROOT+x}" && -z "${PYTHONPATH+x}" && \
  -z "${PYTHONHOME+x}" && -z "${XCODE_XCCONFIG_FILE+x}" && \
  -z "${BASH_ENV+x}" && -z "${ENV+x}" && \
  -z "${BIOMOTION_BASH_ENV_WAS_SOURCED+x}" ]]
[[ "$#" -eq 1 ]]
expected="$(/bin/cat "$fixture_root/staging-path")"
[[ "$1" == "$expected/Products/Applications/BioMotion.app" ]]
printf '%s\n' 'privacy:archive' >> "$fixture_root/events.log"
[[ "$fail_stage" != privacy ]] || exit 45
'''

xcodebuild = r'''#!/bin/bash
set -euo pipefail
fixture_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
event_log="$fixture_root/events.log"
fail_stage=""
if [[ -f "$fixture_root/fail-stage" ]]; then
  fail_stage="$(/bin/cat "$fixture_root/fail-stage")"
fi
if [[ "$PATH" != /usr/bin:/bin:/usr/sbin:/sbin || \
  -n "${ASC_API_KEY_ID+x}" || -n "${ASC_API_ISSUER+x}" ]]; then
  printf '%s\n' 'credentials-or-path:leaked' >> "$event_log"
  exit 97
fi
trusted_home="$(/usr/bin/python3 -I -c \
  'import os, pwd; print(pwd.getpwuid(os.getuid()).pw_dir)')"
if [[ "$HOME" != "$trusted_home" || "$HOME" == /var/empty ]]; then
  printf '%s\n' 'xcode-home:wrong' >> "$event_log"
  exit 95
fi
if [[ -n "${DEVELOPER_DIR+x}" || -n "${TOOLCHAINS+x}" || \
  -n "${SDKROOT+x}" || -n "${PYTHONPATH+x}" || \
  -n "${PYTHONHOME+x}" || -n "${XCODE_XCCONFIG_FILE+x}" || \
  -n "${BASH_ENV+x}" || -n "${ENV+x}" || \
  -n "${BIOMOTION_BASH_ENV_WAS_SOURCED+x}" ]]; then
  printf '%s\n' 'environment:leaked' >> "$event_log"
  exit 96
fi
[[ "$#" -eq 13 ]]
[[ "$1" == -project && "$2" == "$fixture_root/BioMotion.xcodeproj" ]]
[[ "$3" == -scheme && "$4" == BioMotion ]]
[[ "$5" == -configuration && "$6" == Release ]]
[[ "$7" == -destination && "$8" == 'generic/platform=iOS' ]]
[[ "$9" == -archivePath ]]
[[ "$11" == -derivedDataPath ]]
archive="${10}"
derived_data="${12}"
[[ "$13" == archive ]]
case "$archive" in
  "$fixture_root/artifacts"/.biomotion-archive.*/BioMotion.xcarchive) ;;
  *) exit 94 ;;
esac
[[ "$(/usr/bin/stat -f '%Lp' "${archive%/*}")" == 700 ]]
[[ "$derived_data" == "${archive%/*}/DerivedData" ]]
[[ ! -L "$derived_data" && -d "$derived_data" ]]
[[ "$(/usr/bin/stat -f '%Lp' "$derived_data")" == 700 ]]
for argument in "$@"; do
  [[ "$argument" != -allowProvisioningUpdates ]] || exit 93
done
printf '%s\n' "$archive" > "$fixture_root/staging-path"
printf '%s\n' 'xcodebuild:archive' >> "$event_log"
[[ "$fail_stage" != xcodebuild ]] || exit 46
/bin/mkdir -p "$derived_data/Build/Products/Release-iphoneos/include"
printf '%s\n' 'private derived header' > \
  "$derived_data/Build/Products/Release-iphoneos/include/fixture.h"
if [[ "$fail_stage" == derived-data-identity-drift ]]; then
  /bin/mv "$derived_data" "$derived_data.replaced"
  /bin/mkdir "$derived_data"
  /bin/chmod 0700 "$derived_data"
fi
if [[ "$fail_stage" == staging-junk ]]; then
  printf '%s\n' 'unreviewed staging byte' > "${archive%/*}/unreviewed-cache"
fi
/bin/mkdir -p "$archive/Products/Applications/BioMotion.app"
printf '%s\n' 'signed app bytes' > \
  "$archive/Products/Applications/BioMotion.app/BioMotion"
'''

receipt_inspector = r'''#!/usr/bin/python3
import json
import os
from pathlib import Path
import sys

if any(name in os.environ for name in (
    "DEVELOPER_DIR", "TOOLCHAINS", "PYTHONPATH", "PYTHONHOME",
    "XCODE_XCCONFIG_FILE", "BASH_ENV", "ENV",
    "BIOMOTION_BASH_ENV_WAS_SOURCED", "PERL5LIB", "PERL5OPT",
)):
    raise SystemExit(96)
if any(name.startswith("BASH_FUNC_") for name in os.environ):
    raise SystemExit(96)
if os.environ.get("PATH") != "/usr/bin:/bin:/usr/sbin:/sbin":
    raise SystemExit(97)
if os.environ.get("HOME") != "/var/empty":
    raise SystemExit(97)
if "ASC_API_KEY_ID" in os.environ or "ASC_API_ISSUER" in os.environ:
    raise SystemExit(97)

mode = sys.argv[1] if len(sys.argv) > 1 else ""
if mode == "validate-snapshot" and len(sys.argv) == 4:
    repo, expected_digest = sys.argv[2:]
    archive = None
    receipt = None
elif mode == "seal" and len(sys.argv) == 6:
    repo, archive_text, receipt_text, expected_digest = sys.argv[2:]
    archive = Path(archive_text)
    receipt = Path(receipt_text)
elif mode == "verify" and len(sys.argv) == 5:
    repo, archive_text, receipt_text = sys.argv[2:]
    archive = Path(archive_text)
    receipt = Path(receipt_text)
    expected_digest = None
else:
    raise SystemExit(64)

fixture_root = Path(repo)
if not fixture_root.is_dir():
    raise SystemExit(95)

event_log = fixture_root / "events.log"
final_archive = fixture_root / "artifacts/BioMotion.xcarchive"
fail_path = fixture_root / "fail-stage"
fail_stage = fail_path.read_text(encoding="utf-8").strip() if fail_path.exists() else ""

def read_snapshot():
    data = sys.stdin.buffer.read()
    if not data.endswith(b"\n") or data.count(b"\n") != 1 or b"\r" in data:
        raise SystemExit(91)
    value = json.loads(data)
    canonical = (
        json.dumps(value, ensure_ascii=True, separators=(",", ":"), sort_keys=True)
        + "\n"
    ).encode("utf-8")
    if data != canonical:
        raise SystemExit(91)
    if set(value) != {"dependencyLockSHA256", "fixture", "schemaVersion"}:
        raise SystemExit(91)
    if value["dependencyLockSHA256"] != expected_digest:
        raise SystemExit(91)
    if value["schemaVersion"] != 1 or value["fixture"] not in {"stable", "changed"}:
        raise SystemExit(91)
    return value, data

if mode == "validate-snapshot":
    value, data = read_snapshot()
    count_path = fixture_root / "receipt-validator-count"
    count = int(count_path.read_text(encoding="utf-8")) if count_path.exists() else 0
    count += 1
    count_path.write_text(f"{count}\n", encoding="utf-8")
    stage = "pre" if count == 1 else "post" if count == 2 else "unexpected"
    with event_log.open("a", encoding="utf-8") as stream:
        stream.write(f"receipt:validate-snapshot:{stage}\n")
    if stage == "unexpected":
        raise SystemExit(90)
    sys.stdout.buffer.write(data)
elif mode == "seal":
    expected_snapshot, _ = read_snapshot()
    with event_log.open("a", encoding="utf-8") as stream:
        stream.write("receipt:seal\n")
    if fail_stage == "seal":
        raise SystemExit(47)
    if archive == final_archive or receipt != Path(f"{archive}.dependency-receipt.json"):
        raise SystemExit(94)
    actual_snapshot = dict(expected_snapshot)
    if (fixture_root / "seal-observation").exists():
        actual_snapshot["fixture"] = "changed"
    if actual_snapshot != expected_snapshot:
        raise SystemExit(50)
    descriptor = os.open(receipt, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(
            {
                "schemaVersion": 1,
                "fixture": "sealed",
                "dependencySnapshot": expected_snapshot,
            },
            stream,
            sort_keys=True,
        )
        stream.write("\n")
    print("DEPENDENCY_ARCHIVE_RECEIPT_SEALED")
else:
    stage = "final" if archive == final_archive else "staging"
    with event_log.open("a", encoding="utf-8") as stream:
        stream.write(f"receipt:verify:{stage}\n")
    if fail_stage == f"verify-{stage}":
        raise SystemExit(48 if stage == "staging" else 49)
    if not archive.is_dir() or receipt.is_symlink() or not receipt.is_file():
        raise SystemExit(93)
    stored = json.loads(receipt.read_text(encoding="utf-8"))
    if stored["fixture"] != "sealed" or stored["dependencySnapshot"]["fixture"] != "stable":
        raise SystemExit(92)
    print("DEPENDENCY_ARCHIVE_RECEIPT_PASS")
'''

(root / "tools/tests/dependency_boundary_probe.sh").write_text(
    dependency_gate, encoding="utf-8"
)
(root / "tools/tests/app_resource_boundary_probe.sh").write_text(
    resource_gate, encoding="utf-8"
)
(root / "tools/tests/privacy_manifest_probe.sh").write_text(
    privacy_gate, encoding="utf-8"
)
(root / "tools/release/dependency_archive_receipt.py").write_text(
    receipt_inspector, encoding="utf-8"
)
(root / "tools/dependencies.lock.json").write_text(
    '{"schemaVersion":1,"fixture":"stable"}\n', encoding="utf-8"
)
(root / "benign-bash-env").write_text(
    f"printf '%s\\n' 'unexpected:bash-env' > '{root / 'injection.log'}'\n"
    "exit 0\n",
    encoding="utf-8",
)
(root / "fake-tools/xcodebuild").write_text(xcodebuild, encoding="utf-8")

wrapper_path = root / "tools/release/archive_release.sh"
wrapper = wrapper_path.read_text(encoding="utf-8")
old = 'XCODEBUILD="/usr/bin/xcodebuild"'
new = f'XCODEBUILD="{root / "fake-tools/xcodebuild"}"'
if wrapper.count(old) != 1:
    raise SystemExit(f"fixture replacement count changed for {old!r}")
wrapper_path.write_text(wrapper.replace(old, new), encoding="utf-8")
PY

  /bin/chmod 0755 \
    "$root/tools/release/archive_release.sh" \
    "$root/tools/tests/dependency_boundary_probe.sh" \
    "$root/tools/tests/app_resource_boundary_probe.sh" \
    "$root/tools/tests/privacy_manifest_probe.sh" \
    "$root/tools/release/dependency_archive_receipt.py" \
    "$root/fake-tools/xcodebuild"
  printf '%s\n' "$root"
}

run_archive() {
  local root="$1"
  shift
  local fail_stage=""
  while [[ "$#" -gt 0 && "$1" != -- ]]; do
    case "$1" in
      FAKE_FAIL_STAGE=*) fail_stage="${1#FAKE_FAIL_STAGE=}" ;;
      *) fail "unsupported fixture setting: $1" ;;
    esac
    shift
  done
  [[ "$#" -gt 0 ]] || fail "run_archive requires an argument separator"
  shift

  if [[ -n "$fail_stage" ]]; then
    printf '%s\n' "$fail_stage" > "$root/fail-stage"
  fi

  local archive="$root/artifacts/BioMotion.xcarchive"
  set +e
  (
    cd "$root"
    /usr/bin/env \
      PATH="$root/untrusted-path" \
      DEVELOPER_DIR="$root/untrusted-developer" \
      TOOLCHAINS=untrusted-toolchain \
      SDKROOT="$root/untrusted-sdk" \
      PYTHONPATH="$root/untrusted-python" \
      PYTHONHOME="$root/untrusted-python-home" \
      XCODE_XCCONFIG_FILE="$root/untrusted.xcconfig" \
      PERL5LIB="$root/untrusted-perl" \
      PERL5OPT=-MThisModuleMustNotLoad \
      BASH_ENV="$root/benign-bash-env" \
      ENV="$root/benign-shell-env" \
      ASC_API_KEY_ID=untrusted-key \
      ASC_API_ISSUER=untrusted-issuer \
      'BASH_FUNC_untrusted%%=() { return 0; }' \
      tools/release/archive_release.sh "$@"
  ) >"$root/output.txt" 2>&1
  RUN_STATUS=$?
  set -e
}

standard_arguments() {
  STANDARD_ARGUMENTS=(
    --archive "$1/artifacts/BioMotion.xcarchive"
  )
}

# The happy path proves the exact causal order and publishes one verified pair.
root="$(make_fixture success)"
standard_arguments "$root"
run_archive "$root" -- "${STANDARD_ARGUMENTS[@]}"
if [[ "$RUN_STATUS" -ne 0 ]]; then
  /bin/cat "$root/output.txt" >&2
fi
assert_status 0 "$RUN_STATUS" "controlled archive success"
[[ ! -e "$root/injection.log" && ! -L "$root/injection.log" ]] || \
  fail "protected archive entry point executed inherited BASH_ENV"
assert_file_text "$root/events.log" \
  $'dependency:pre\nreceipt:validate-snapshot:pre\nresource:source\nxcodebuild:archive\ndependency:post\nreceipt:validate-snapshot:post\nreceipt:seal\nresource:archive\nprivacy:archive\nreceipt:verify:staging\nreceipt:verify:final' \
  "controlled archive causal order"
[[ -d "$root/artifacts/BioMotion.xcarchive" ]] || \
  fail "published archive is missing"
[[ -f "$root/artifacts/BioMotion.xcarchive.dependency-receipt.json" ]] || \
  fail "published receipt is missing"
/usr/bin/grep -Fq 'ARCHIVE_RELEASE_PASS' "$root/output.txt" || \
  fail "success marker is missing"
/usr/bin/grep -Fq \
  "$root/artifacts/BioMotion.xcarchive.dependency-receipt.json" \
  "$root/output.txt" || fail "receipt path is missing from success output"
assert_no_staging "$root"
pass "success publishes a post-build-verified archive and receipt in strict order"

# A pre-build dependency failure prevents the source gate and build.
root="$(make_fixture dependency_pre_failure)"
standard_arguments "$root"
run_archive "$root" FAKE_FAIL_STAGE=dependency-pre -- "${STANDARD_ARGUMENTS[@]}"
assert_status 41 "$RUN_STATUS" "pre-build dependency failure"
assert_file_text "$root/events.log" 'dependency:pre' \
  "pre-build dependency failure order"
assert_no_published_output "$root"
assert_no_staging "$root"
pass "a pre-build dependency failure blocks every later operation"

# Shape-looking JSON is not enough: both captures require the receipt schema.
for invalid_stage in invalid-snapshot-pre invalid-snapshot-post; do
  root="$(make_fixture "$invalid_stage")"
  standard_arguments "$root"
  run_archive "$root" FAKE_FAIL_STAGE="$invalid_stage" -- \
    "${STANDARD_ARGUMENTS[@]}"
  [[ "$RUN_STATUS" -ne 0 ]] || fail "$invalid_stage unexpectedly succeeded"
  assert_no_published_output "$root"
  assert_no_staging "$root"
done
pass "both dependency captures require canonical schema-valid snapshots"

# A post-build dependency failure proves the build cannot outrun input drift.
root="$(make_fixture dependency_post_failure)"
standard_arguments "$root"
run_archive "$root" FAKE_FAIL_STAGE=dependency-post -- "${STANDARD_ARGUMENTS[@]}"
assert_status 42 "$RUN_STATUS" "post-build dependency failure"
assert_file_text "$root/events.log" \
  $'dependency:pre\nreceipt:validate-snapshot:pre\nresource:source\nxcodebuild:archive\ndependency:post' \
  "post-build dependency failure order"
assert_no_published_output "$root"
assert_no_staging "$root"

root="$(make_fixture dependency_snapshot_drift)"
standard_arguments "$root"
run_archive "$root" FAKE_FAIL_STAGE=boundary-drift -- \
  "${STANDARD_ARGUMENTS[@]}"
assert_status 1 "$RUN_STATUS" "post-build dependency snapshot drift"
assert_file_text "$root/events.log" \
  $'dependency:pre\nreceipt:validate-snapshot:pre\nresource:source\nxcodebuild:archive\ndependency:post\nreceipt:validate-snapshot:post' \
  "post-build dependency snapshot drift order"
assert_no_published_output "$root"
assert_no_staging "$root"
pass "post-build dependency failure or snapshot drift discards staging"

# A drift after the post-build check is re-observed by the receipt sealer.
root="$(make_fixture seal_boundary_drift)"
standard_arguments "$root"
run_archive "$root" FAKE_FAIL_STAGE=seal-boundary-drift -- \
  "${STANDARD_ARGUMENTS[@]}"
assert_status 50 "$RUN_STATUS" "receipt-time dependency snapshot drift"
assert_file_text "$root/events.log" \
  $'dependency:pre\nreceipt:validate-snapshot:pre\nresource:source\nxcodebuild:archive\ndependency:post\nreceipt:validate-snapshot:post\nreceipt:seal' \
  "receipt-time dependency snapshot drift order"
assert_no_published_output "$root"
assert_no_staging "$root"
pass "receipt sealing binds the archive to the captured dependency snapshot"

# xcodebuild failure cannot leave a public archive-shaped directory.
root="$(make_fixture xcodebuild_failure)"
standard_arguments "$root"
run_archive "$root" FAKE_FAIL_STAGE=xcodebuild -- "${STANDARD_ARGUMENTS[@]}"
assert_status 46 "$RUN_STATUS" "xcodebuild failure"
assert_file_text "$root/events.log" \
  $'dependency:pre\nreceipt:validate-snapshot:pre\nresource:source\nxcodebuild:archive' \
  "xcodebuild failure order"
assert_no_published_output "$root"
assert_no_staging "$root"
pass "xcodebuild failure is contained entirely inside private staging"

# Replacing the fixed private DerivedData directory is caught immediately.
root="$(make_fixture derived_data_identity_drift)"
standard_arguments "$root"
run_archive "$root" FAKE_FAIL_STAGE=derived-data-identity-drift -- \
  "${STANDARD_ARGUMENTS[@]}"
[[ "$RUN_STATUS" -ne 0 ]] || fail "replaced DerivedData unexpectedly succeeded"
assert_file_text "$root/events.log" \
  $'dependency:pre\nreceipt:validate-snapshot:pre\nresource:source\nxcodebuild:archive' \
  "private DerivedData identity drift order"
assert_no_published_output "$root"
assert_no_staging "$root"
pass "xcodebuild uses a pinned private DerivedData directory that is cleaned"

# A build-created staging sibling cannot hitchhike into publication.
root="$(make_fixture unexpected_staging_entry)"
standard_arguments "$root"
run_archive "$root" FAKE_FAIL_STAGE=staging-junk -- \
  "${STANDARD_ARGUMENTS[@]}"
[[ "$RUN_STATUS" -ne 0 ]] || fail "unexpected staging entry was published"
assert_file_text "$root/events.log" \
  $'dependency:pre\nreceipt:validate-snapshot:pre\nresource:source\nxcodebuild:archive\ndependency:post\nreceipt:validate-snapshot:post\nreceipt:seal\nresource:archive\nprivacy:archive\nreceipt:verify:staging' \
  "unexpected staging entry rejection order"
assert_no_published_output "$root"
assert_no_staging "$root"
pass "publication requires staging to contain only the archive and receipt"

# Every source/archive/privacy gate dominates publication. Archive gates run
# after sealing so the final staged verify detects any gate-time byte change.
for failed_gate in resource-source resource-archive privacy; do
  root="$(make_fixture "gate_failure_${failed_gate}")"
  standard_arguments "$root"
  run_archive "$root" FAKE_FAIL_STAGE="$failed_gate" -- \
    "${STANDARD_ARGUMENTS[@]}"
  [[ "$RUN_STATUS" -ne 0 ]] || fail "$failed_gate unexpectedly succeeded"
  assert_no_published_output "$root"
  assert_no_staging "$root"
  case "$failed_gate" in
    resource-source)
      expected_events=$'dependency:pre\nreceipt:validate-snapshot:pre\nresource:source'
      ;;
    resource-archive)
      expected_events=$'dependency:pre\nreceipt:validate-snapshot:pre\nresource:source\nxcodebuild:archive\ndependency:post\nreceipt:validate-snapshot:post\nreceipt:seal\nresource:archive'
      ;;
    privacy)
      expected_events=$'dependency:pre\nreceipt:validate-snapshot:pre\nresource:source\nxcodebuild:archive\ndependency:post\nreceipt:validate-snapshot:post\nreceipt:seal\nresource:archive\nprivacy:archive'
      ;;
  esac
  assert_file_text "$root/events.log" "$expected_events" \
    "$failed_gate causal order"
done

root="$(make_fixture archive_identity_drift)"
standard_arguments "$root"
run_archive "$root" FAKE_FAIL_STAGE=archive-identity-drift -- \
  "${STANDARD_ARGUMENTS[@]}"
assert_status 1 "$RUN_STATUS" "staged archive identity drift"
assert_file_text "$root/events.log" \
  $'dependency:pre\nreceipt:validate-snapshot:pre\nresource:source\nxcodebuild:archive\ndependency:post\nreceipt:validate-snapshot:post\nreceipt:seal\nresource:archive' \
  "staged archive identity drift order"
assert_no_published_output "$root"
assert_no_staging "$root"
pass "all gates and a replaced staged archive block publication"

# Receipt creation failure keeps both public paths absent.
root="$(make_fixture seal_failure)"
standard_arguments "$root"
run_archive "$root" FAKE_FAIL_STAGE=seal -- "${STANDARD_ARGUMENTS[@]}"
assert_status 47 "$RUN_STATUS" "receipt seal failure"
assert_file_text "$root/events.log" \
  $'dependency:pre\nreceipt:validate-snapshot:pre\nresource:source\nxcodebuild:archive\ndependency:post\nreceipt:validate-snapshot:post\nreceipt:seal' \
  "receipt seal failure order"
assert_no_published_output "$root"
assert_no_staging "$root"
pass "receipt sealing is fail-closed"

# Verification failures before and after publication both roll back output.
for failed_verify in verify-staging verify-final; do
  root="$(make_fixture "${failed_verify}_failure")"
  standard_arguments "$root"
  run_archive "$root" FAKE_FAIL_STAGE="$failed_verify" -- \
    "${STANDARD_ARGUMENTS[@]}"
  [[ "$RUN_STATUS" -ne 0 ]] || fail "$failed_verify unexpectedly succeeded"
  assert_no_published_output "$root"
  assert_no_staging "$root"
done
pass "staging and final receipt verification failures publish nothing"

# Existing directories, files, and symlinks are never overwritten.
for existing_case in archive receipt archive-symlink receipt-symlink; do
  root="$(make_fixture "existing_${existing_case}")"
  archive="$root/artifacts/BioMotion.xcarchive"
  receipt="$archive.dependency-receipt.json"
  case "$existing_case" in
    archive) /bin/mkdir "$archive" ;;
    receipt) printf '%s\n' existing > "$receipt" ;;
    archive-symlink) /bin/ln -s "$root" "$archive" ;;
    receipt-symlink) /bin/ln -s "$root/missing" "$receipt" ;;
  esac
  standard_arguments "$root"
  run_archive "$root" -- "${STANDARD_ARGUMENTS[@]}"
  [[ "$RUN_STATUS" -ne 0 ]] || fail "$existing_case unexpectedly succeeded"
  [[ ! -f "$root/events.log" ]] || \
    fail "$existing_case reached a gate before rejection"
done
pass "existing archive or receipt paths, including symlinks, are rejected"

# A writable or symlinked parent is rejected before staging is created.
for unsafe_case in writable symlink; do
  root="$(make_fixture "unsafe_parent_${unsafe_case}")"
  case "$unsafe_case" in
    writable)
      /bin/chmod 0777 "$root/artifacts"
      unsafe_archive="$root/artifacts/BioMotion.xcarchive"
      ;;
    symlink)
      /bin/mkdir "$root/real-artifacts"
      /bin/chmod 0700 "$root/real-artifacts"
      /bin/rm -rf "$root/artifacts"
      /bin/ln -s "$root/real-artifacts" "$root/artifacts"
      unsafe_archive="$root/artifacts/BioMotion.xcarchive"
      ;;
  esac
  run_archive "$root" -- --archive "$unsafe_archive"
  [[ "$RUN_STATUS" -ne 0 ]] || fail "$unsafe_case parent unexpectedly succeeded"
  [[ ! -f "$root/events.log" ]] || \
    fail "$unsafe_case parent reached a gate before rejection"
done
pass "unsafe archive parents are rejected"

# Only one complete --archive option naming a new .xcarchive is accepted.
for argument_case in no_args missing_value duplicate unknown positional suffix help; do
  root="$(make_fixture "arguments_${argument_case}")"
  archive="$root/artifacts/BioMotion.xcarchive"
  case "$argument_case" in
    no_args) arguments=(unused) ;;
    missing_value) arguments=(--archive) ;;
    duplicate) arguments=(--archive "$archive" --archive "$archive") ;;
    unknown) arguments=(--unknown "$archive") ;;
    positional) arguments=("$archive") ;;
    suffix) arguments=(--archive "$root/artifacts/BioMotion.archive") ;;
    help) arguments=(--help) ;;
  esac
  if [[ "$argument_case" == no_args ]]; then
    run_archive "$root" --
  else
    run_archive "$root" -- "${arguments[@]}"
  fi
  assert_status 64 "$RUN_STATUS" "argument rejection: $argument_case"
  [[ ! -f "$root/events.log" ]] || \
    fail "$argument_case reached a gate before rejection"
done
pass "missing, malformed, duplicate, positional, and extra arguments fail 64"

printf 'RELEASE_ARCHIVE_GATE_TESTS_PASS %s/14\n' "$PASS_COUNT"
