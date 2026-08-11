#!/usr/bin/env bash
# Transaction-level tests for package.sh using a tiny locked model and fake Apple tools.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-assetpack-tests.XXXXXX")"
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

make_fixture() {
  local name="$1"
  local root="$TEST_ROOT/$name"
  mkdir -p \
    "$root/tools/assetpack" \
    "$root/tools/tests" \
    "$root/BioMotion/Resources" \
    "$root/fake-bin" \
    "$root/fixture"
  cp "$REPO_ROOT/tools/assetpack/package.sh" "$root/tools/assetpack/package.sh"
  cp "$REPO_ROOT/tools/assetpack/Manifest.json" "$root/tools/assetpack/Manifest.json"
  cp "$REPO_ROOT/tools/assetpack/verify_model_lock.py" "$root/tools/assetpack/verify_model_lock.py"
  cp "$REPO_ROOT/tools/tests/assetpack_fake_xcrun.py" "$root/fake-bin/xcrun"
  chmod 700 "$root/fake-bin/xcrun"

  python3 - "$root" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
fake_xcrun = root / "fake-bin/xcrun"
fake_xcodebuild = root / "fake-bin/xcodebuild"
fake_xcodebuild.write_text(
    "#!/bin/sh\nprintf 'Xcode 26.4\\nBuild version 17E192\\n'\n",
    encoding="utf-8",
)
fake_xcodebuild.chmod(0o755)
verifier_path = root / "tools/assetpack/verify_model_lock.py"
verifier_text = verifier_path.read_text(encoding="utf-8")
verifier_text = verifier_text.replace(
    'XCRUN = "/usr/bin/xcrun"', f'XCRUN = "{fake_xcrun}"'
).replace(
    'XCODEBUILD = "/usr/bin/xcodebuild"', f'XCODEBUILD = "{fake_xcodebuild}"'
)
rename_declaration = "def _rename_swap(source: Path, destination: Path) -> None:\n"
if rename_declaration not in verifier_text:
    raise SystemExit("cannot install explicit test-only rename-swap fault")
verifier_text = verifier_text.replace(
    rename_declaration,
    rename_declaration
    + '    test_calls = getattr(_rename_swap, "_test_calls", 0) + 1\n'
    + '    _rename_swap._test_calls = test_calls\n'
    + '    if os.environ.get("BIOMOTION_TEST_RENAME_SWAP_FAILURE") == "1":\n'
    + '        raise OSError("injected test-only swap failure")\n'
    + '    recovery_fault = os.environ.get("BIOMOTION_TEST_PUBLISH_RECOVERY_FAULT", "")\n'
    + '    if recovery_fault == "rollback-namespace" and test_calls == 2:\n'
    + '        raise OSError("injected test-only rollback swap failure")\n',
    1,
)
entrypoint = '\nif __name__ == "__main__":\n'
if entrypoint not in verifier_text:
    raise SystemExit("cannot install explicit test-only publication fsync fault")
test_fsync_hook = r'''
_test_original_fsync_directory = _fsync_directory

def _test_fsync_directory(path):
    fault = os.environ.get("BIOMOTION_TEST_PUBLISH_RECOVERY_FAULT", "")
    path = Path(path)
    if fault and path.name == "assetpack":
        calls = getattr(_test_fsync_directory, "_test_calls", 0) + 1
        _test_fsync_directory._test_calls = calls
        if calls == 1 and fault == "signal":
            os.kill(os.getpid(), 15)
        if calls == 1 and fault == "unexpected":
            raise RuntimeError("injected test-only unexpected publish crash")
        if calls == 1 or (fault == "rollback-fsync" and calls == 2):
            raise VerificationError(
                f"injected test-only publication fsync failure call {calls}"
            )
    return _test_original_fsync_directory(path)

_fsync_directory = _test_fsync_directory
'''
verifier_text = verifier_text.replace(
    entrypoint,
    "\n" + test_fsync_hook + entrypoint,
    1,
)
verifier_path.write_text(verifier_text, encoding="utf-8")
package_path = root / "tools/assetpack/package.sh"
package_text = package_path.read_text(encoding="utf-8").replace(
    'XCRUN="/usr/bin/xcrun"', f'XCRUN="{fake_xcrun}"'
)
package_path.write_text(package_text, encoding="utf-8")
license_bytes = b"fixture SAM license\n"
(root / "BioMotion/Resources/SAM-LICENSE.txt").write_bytes(license_bytes)

interface = {
    "specificationVersion": 8,
    "modelType": "mlProgram",
    "minimumIOS": "17.0",
    "generatedClassName": "SAM3DBodyPose",
    "allowAdditionalFeatures": False,
    "inputs": [
        {
            "name": "image",
            "featureType": "multiArray",
            "dataType": "float16",
            "shape": [1, 3, 4, 4],
            "optional": False,
            "shapeFlexible": False,
        }
    ],
    "outputs": [
        {
            "name": "joint_coords",
            "featureType": "multiArray",
            "dataType": "float32",
            "shape": [2, 3],
            "optional": False,
            "shapeFlexible": False,
        }
    ],
}

def feature(row):
    return {
        "name": row["name"],
        "type": "MultiArray",
        "dataType": {"float16": "Float16", "float32": "Float32"}[row["dataType"]],
        "shape": json.dumps(row["shape"]),
        "isOptional": "1" if row["optional"] else "0",
        "hasShapeFlexibility": "1" if row["shapeFlexible"] else "0",
    }

metadata = [
    {
        "specificationVersion": 8,
        "modelType": {"name": "MLModelType_mlProgram"},
        "generatedClassName": "SAM3DBodyPose",
        "availability": {"iOS": "17.0"},
        "inputSchema": [feature(row) for row in interface["inputs"]],
        "outputSchema": [feature(row) for row in interface["outputs"]],
    }
]

source_contents = {
    "Manifest.json": b"fixture source manifest\n",
    "Data/com.apple.CoreML/model.mlmodel": b"fixture model\n",
    "Data/com.apple.CoreML/weights/weight.bin": b"fixture source weights\n",
}
compiled_contents = {
    "coremldata.bin": b"fixture core data\n",
    "metadata.json": (json.dumps(metadata, separators=(",", ":")) + "\n").encode(),
    "model.mil": b"fixture MIL\n",
    "analytics/coremldata.bin": b"fixture analytics\n",
    "weights/weight.bin": b"fixture compiled weights\n",
}

def write_tree(directory, contents):
    directory.mkdir()
    for relative, content in contents.items():
        destination = directory / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)

def records(contents):
    return [
        {
            "path": path,
            "size": len(content),
            "sha256": hashlib.sha256(content).hexdigest(),
        }
        for path, content in sorted(contents.items())
    ]

source = root / "fixture/SAM3DBodyPose.mlpackage"
compiled = root / "fixture/SAM3DBodyPose.mlmodelc"
write_tree(source, source_contents)
write_tree(compiled, compiled_contents)
placeholder = hashlib.sha256(b"fixture provenance").hexdigest()
lock = {
    "schemaVersion": 1,
    "artifactRevision": 7,
    "assetPackID": "sam3d-body-pose",
    "modelBaseName": "SAM3DBodyPose",
    "license": {
        "file": "SAM-LICENSE.txt",
        "sha256": hashlib.sha256(license_bytes).hexdigest(),
        "upstreamPath": "LICENSE",
    },
    "provenance": {
        "repository": "https://example.invalid/sam-3d-body",
        "exportRecipeCommit": "a" * 40,
        "contract": {"path": "export/CONTRACT.md", "sha256": placeholder},
        "conversionLog": {"path": "export/convert.json", "sha256": placeholder},
        "coremlReport": {"path": "export/report.json", "sha256": placeholder},
        "sourceInputs": [
            {"path": "checkpoint.bin", "size": 1, "sha256": placeholder}
        ],
        "conversionToolchain": {
            "python": "3.11",
            "torch": "2.13",
            "coremltools": "9.0",
            "computePrecision": "mixed",
            "ioDType": "float16",
            "deploymentTarget": "iOS17",
        },
        "compileToolchain": {
            "xcode": "26.4",
            "xcodeBuild": "17E192",
            "coremlcompiler": "fixture",
            "baPackage": "fixture",
        },
    },
    "sourcePackage": {
        "directoryName": "SAM3DBodyPose.mlpackage",
        "files": records(source_contents),
    },
    "compiledModel": {
        "directoryName": "SAM3DBodyPose.mlmodelc",
        "files": records(compiled_contents),
    },
    "interface": interface,
}
(root / "BioMotion/Resources/SAM3DBodyPose.lock.json").write_text(
    json.dumps(lock, indent=2) + "\n", encoding="utf-8"
)
PY
  echo "$root"
}

run_package() {
  local root="$1"
  local fault="$2"
  local output="$3"
  local log="$4"
  local swap_failure=0
  local recovery_fault=""
  local fake_fault="$fault"
  if [[ "$fault" == "swap" ]]; then
    swap_failure=1
  fi
  case "$fault" in
    recovery-rollback-success)
      recovery_fault="rollback-success"
      fake_fault=""
      ;;
    recovery-rollback-namespace)
      recovery_fault="rollback-namespace"
      fake_fault=""
      ;;
    recovery-rollback-fsync)
      recovery_fault="rollback-fsync"
      fake_fault=""
      ;;
    recovery-unexpected)
      recovery_fault="unexpected"
      fake_fault=""
      ;;
    recovery-signal)
      recovery_fault="signal"
      fake_fault=""
      ;;
  esac
  set +e
  (
    cd "$root"
    BIOMOTION_FAKE_COMPILED_MODEL="$root/fixture/SAM3DBodyPose.mlmodelc" \
      BIOMOTION_FAKE_XCRUN_LOG="$log" \
      BIOMOTION_FAKE_XCRUN_FAULT="$fake_fault" \
      BIOMOTION_TEST_RENAME_SWAP_FAILURE="$swap_failure" \
      BIOMOTION_TEST_PUBLISH_RECOVERY_FAULT="$recovery_fault" \
      /bin/bash tools/assetpack/package.sh "$root/fixture/SAM3DBodyPose.mlpackage"
  ) >"$output" 2>&1
  RUN_STATUS=$?
  set -e
}

assert_order() {
  local file="$1"
  shift
  python3 - "$file" "$@" <<'PY'
from pathlib import Path
import sys

text = Path(sys.argv[1]).read_text(encoding="utf-8")
position = -1
for needle in sys.argv[2:]:
    next_position = text.find(needle, position + 1)
    if next_position < 0:
        raise SystemExit(f"missing ordered marker {needle!r} in:\n{text}")
    position = next_position
PY
}

assert_recovery_state() {
  local root="$1"
  local output="$2"
  PRESERVED_TRANSACTION="$(python3 - "$output" <<'PY'
from pathlib import Path
import sys

prefix = "PACKAGE_RECOVERY_REQUIRED transaction: "
for line in Path(sys.argv[1]).read_text(encoding="utf-8").splitlines():
    if line.startswith(prefix):
        print(line.removeprefix(prefix))
        break
PY
)"
  [[ -n "$PRESERVED_TRANSACTION" && -d "$PRESERVED_TRANSACTION" ]] || \
    fail "recovery transaction was not preserved"
  [[ -d "$root/build/assetpack/.package-sam3d-body-pose.lock" ]] || \
    fail "recovery package lock was not retained"
  [[ "$(<"$output")" == *"package lock retained"* ]] || \
    fail "package recovery lock diagnostic missing"
}

success_root="$(make_fixture success)"
success_output="$success_root/package.out"
success_log="$success_root/xcrun.log"
run_package "$success_root" "" "$success_output" "$success_log"
[[ "$RUN_STATUS" -eq 0 ]] || fail "happy package failed: $(tail -30 "$success_output")"
success_aar="$success_root/build/assetpack/release/sam3d-body-pose.aar"
success_receipt="$success_aar.receipt.json"
[[ -f "$success_aar" && -f "$success_receipt" ]] || fail "happy package did not publish pair"
assert_order "$success_output" \
  "snapshot authority" \
  "verify frozen repository" \
  "verify frozen toolchain" \
  "verify source" \
  "compile private" \
  "verify compiled" \
  "stage exact payload" \
  "package temporary AAR" \
  "seal AAR + receipt" \
  "publish validated pair"
python3 "$success_root/tools/assetpack/verify_model_lock.py" receipt \
  --lock "$success_root/BioMotion/Resources/SAM3DBodyPose.lock.json" \
  --license "$success_root/BioMotion/Resources/SAM-LICENSE.txt" \
  --manifest "$success_root/tools/assetpack/Manifest.json" \
  "$success_aar" "$success_receipt" >/dev/null
pass "happy path publishes a self-verifying AAR and receipt in required order"

for fault in compile package extra missing path symlink hash swap; do
  root="$(make_fixture "failure-$fault")"
  mkdir -p "$root/build/assetpack/release"
  aar="$root/build/assetpack/release/sam3d-body-pose.aar"
  receipt="$aar.receipt.json"
  printf 'known-good-aar\n' >"$aar"
  printf 'known-good-receipt\n' >"$receipt"
  output="$root/package.out"
  log="$root/xcrun.log"
  run_package "$root" "$fault" "$output" "$log"
  [[ "$RUN_STATUS" -ne 0 ]] || fail "fault $fault unexpectedly succeeded"
  [[ "$(<"$aar")" == "known-good-aar" ]] || fail "fault $fault overwrote old AAR"
  [[ "$(<"$receipt")" == "known-good-receipt" ]] || fail "fault $fault overwrote old receipt"
  if [[ "$fault" == "path" ]]; then
    ! rg -q '"aa","extract"' "$log" || fail "unsafe archive path reached extraction"
  fi
  if [[ "$fault" == "swap" ]]; then
    [[ "$(<"$output")" == *"atomic directory swap failed"* ]] || \
      fail "swap failure diagnostic missing"
  fi
  pass "$fault failure preserves the previous good pair"
done

rollback_root="$(make_fixture recovery-rollback-success)"
mkdir -p "$rollback_root/build/assetpack/release"
rollback_aar="$rollback_root/build/assetpack/release/sam3d-body-pose.aar"
rollback_receipt="$rollback_aar.receipt.json"
printf 'known-good-aar\n' >"$rollback_aar"
printf 'known-good-receipt\n' >"$rollback_receipt"
rollback_output="$rollback_root/package.out"
rollback_log="$rollback_root/xcrun.log"
run_package \
  "$rollback_root" \
  "recovery-rollback-success" \
  "$rollback_output" \
  "$rollback_log"
[[ "$RUN_STATUS" -eq 1 ]] || fail "durability rollback should return ordinary failure"
[[ "$(<"$rollback_aar")" == "known-good-aar" ]] || fail "durability rollback lost old AAR"
[[ "$(<"$rollback_receipt")" == "known-good-receipt" ]] || \
  fail "durability rollback lost old receipt"
[[ "$(<"$rollback_output")" == *"rolled back"* ]] || fail "rollback diagnostic missing"
[[ ! -e "$rollback_root/build/assetpack/.package-sam3d-body-pose.lock" ]] || \
  fail "successful rollback retained package lock"
rollback_transactions=("$rollback_root"/build/assetpack/.package-sam3d-body-pose.*)
[[ ! -e "${rollback_transactions[0]}" ]] || fail "successful rollback retained transaction"
pass "post-fsync failure rolls back old release before ordinary cleanup"

recovery_root="$(make_fixture recovery-rollback-namespace)"
mkdir -p "$recovery_root/build/assetpack/release"
recovery_aar="$recovery_root/build/assetpack/release/sam3d-body-pose.aar"
recovery_receipt="$recovery_aar.receipt.json"
printf 'known-good-aar\n' >"$recovery_aar"
printf 'known-good-receipt\n' >"$recovery_receipt"
recovery_output="$recovery_root/package.out"
recovery_log="$recovery_root/xcrun.log"
run_package \
  "$recovery_root" \
  "recovery-rollback-namespace" \
  "$recovery_output" \
  "$recovery_log"
[[ "$RUN_STATUS" -eq 2 ]] || fail "unproven rollback must return recovery status 2"
assert_recovery_state "$recovery_root" "$recovery_output"
preserved_aar="$PRESERVED_TRANSACTION/release-candidate/sam3d-body-pose.aar"
preserved_receipt="$preserved_aar.receipt.json"
[[ "$(<"$preserved_aar")" == "known-good-aar" ]] || fail "preserved old AAR missing"
[[ "$(<"$preserved_receipt")" == "known-good-receipt" ]] || \
  fail "preserved old receipt missing"
[[ "$(<"$recovery_output")" == *"MODEL_LOCK_RECOVERY_REQUIRED"* ]] || \
  fail "verifier recovery diagnostic missing"
pass "unproven rollback preserves complete transaction and package lock"

rollback_fsync_root="$(make_fixture recovery-rollback-fsync)"
mkdir -p "$rollback_fsync_root/build/assetpack/release"
rollback_fsync_aar="$rollback_fsync_root/build/assetpack/release/sam3d-body-pose.aar"
rollback_fsync_receipt="$rollback_fsync_aar.receipt.json"
printf 'known-good-aar\n' >"$rollback_fsync_aar"
printf 'known-good-receipt\n' >"$rollback_fsync_receipt"
rollback_fsync_output="$rollback_fsync_root/package.out"
rollback_fsync_log="$rollback_fsync_root/xcrun.log"
run_package \
  "$rollback_fsync_root" \
  "recovery-rollback-fsync" \
  "$rollback_fsync_output" \
  "$rollback_fsync_log"
[[ "$RUN_STATUS" -eq 2 ]] || fail "unproven rollback fsync must return recovery status 2"
assert_recovery_state "$rollback_fsync_root" "$rollback_fsync_output"
[[ "$(<"$rollback_fsync_aar")" == "known-good-aar" ]] || \
  fail "rollback fsync recovery lost old AAR"
[[ "$(<"$rollback_fsync_receipt")" == "known-good-receipt" ]] || \
  fail "rollback fsync recovery lost old receipt"
[[ -f "$PRESERVED_TRANSACTION/release-candidate/sam3d-body-pose.aar" && \
  -f "$PRESERVED_TRANSACTION/release-candidate/sam3d-body-pose.aar.receipt.json" ]] || \
  fail "rollback fsync recovery lost the complete new candidate pair"
pass "unproven rollback fsync preserves transaction and package lock"

for recovery_fault in recovery-unexpected recovery-signal; do
  abnormal_root="$(make_fixture "$recovery_fault")"
  mkdir -p "$abnormal_root/build/assetpack/release"
  abnormal_aar="$abnormal_root/build/assetpack/release/sam3d-body-pose.aar"
  abnormal_receipt="$abnormal_aar.receipt.json"
  printf 'known-good-aar\n' >"$abnormal_aar"
  printf 'known-good-receipt\n' >"$abnormal_receipt"
  abnormal_output="$abnormal_root/package.out"
  abnormal_log="$abnormal_root/xcrun.log"
  run_package \
    "$abnormal_root" \
    "$recovery_fault" \
    "$abnormal_output" \
    "$abnormal_log"
  if [[ "$recovery_fault" == "recovery-unexpected" ]]; then
    [[ "$RUN_STATUS" -eq 2 ]] || fail "unexpected publish crash must return recovery status 2"
  else
    [[ "$RUN_STATUS" -eq 143 ]] || fail "terminated publish should return signal status 143"
  fi
  assert_recovery_state "$abnormal_root" "$abnormal_output"
  preserved_old_aar="$PRESERVED_TRANSACTION/release-candidate/sam3d-body-pose.aar"
  preserved_old_receipt="$preserved_old_aar.receipt.json"
  [[ "$(<"$preserved_old_aar")" == "known-good-aar" ]] || \
    fail "$recovery_fault deleted the displaced old AAR"
  [[ "$(<"$preserved_old_receipt")" == "known-good-receipt" ]] || \
    fail "$recovery_fault deleted the displaced old receipt"
  [[ -f "$abnormal_aar" && -f "$abnormal_receipt" ]] || \
    fail "$recovery_fault lost the visible candidate pair"
  if [[ "$recovery_fault" == "recovery-unexpected" ]]; then
    [[ "$(<"$abnormal_output")" == *"MODEL_LOCK_RECOVERY_REQUIRED"* ]] || \
      fail "unexpected publish crash lacked verifier recovery diagnostic"
    pass "unexpected publish exception preserves displaced release and lock"
  else
    pass "publish subprocess signal preserves displaced release and lock"
  fi
done

text_root="$(make_fixture receipt-text-aar)"
text_aar="$text_root/sam3d-body-pose.aar"
text_receipt="$text_aar.receipt.json"
printf 'this is not an Apple Archive\n' >"$text_aar"
cp "$success_receipt" "$text_receipt"
set +e
text_output="$(PATH=/usr/bin:/bin:/usr/sbin:/sbin \
  /usr/bin/python3 "$text_root/tools/assetpack/verify_model_lock.py" receipt \
  --lock "$text_root/BioMotion/Resources/SAM3DBodyPose.lock.json" \
  --license "$text_root/BioMotion/Resources/SAM-LICENSE.txt" \
  --manifest "$text_root/tools/assetpack/Manifest.json" \
  "$text_aar" "$text_receipt" 2>&1)"
text_status=$?
set -e
[[ "$text_status" -ne 0 ]] || fail "public receipt accepted a text AAR"
[[ "$text_output" == *"aa list failed"* ]] || fail "text AAR diagnostic missing"
pass "public receipt rejects a text AAR before trusting receipt hashes"

python3 - "$success_receipt" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
receipt = json.loads(path.read_text(encoding="utf-8"))
receipt["aar"]["sha256"] = "0" * 64
path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
set +e
receipt_output="$(python3 "$success_root/tools/assetpack/verify_model_lock.py" receipt \
  --lock "$success_root/BioMotion/Resources/SAM3DBodyPose.lock.json" \
  --license "$success_root/BioMotion/Resources/SAM-LICENSE.txt" \
  --manifest "$success_root/tools/assetpack/Manifest.json" \
  "$success_aar" "$success_receipt" 2>&1)"
receipt_status=$?
set -e
[[ "$receipt_status" -ne 0 ]] || fail "drifted receipt unexpectedly verified"
[[ "$receipt_output" == *"AAR SHA-256 mismatch"* ]] || fail "receipt drift diagnostic missing"
pass "receipt drift fails closed"

[[ "$PASS_COUNT" -eq 16 ]] || fail "unexpected package receipt test count: $PASS_COUNT"
echo "ASSET_PACK_PACKAGE_RECEIPT_TESTS_PASS ${PASS_COUNT}/16"
