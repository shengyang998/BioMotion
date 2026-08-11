#!/bin/bash
# Isolated transaction tests for the receipt-gated developer model bundle.
set -euo pipefail
umask 077

PATH="/usr/bin:/bin:/usr/sbin:/sbin"
export PATH

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-dev-bundle-tests.XXXXXX")"
trap 'rm -rf -- "$TEST_ROOT"' EXIT

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
    "$root/BioMotion/Resources" \
    "$root/build/assetpack/release"
  cp "$REPO_ROOT/tools/assetpack/dev_bundle_model.sh" \
    "$root/tools/assetpack/dev_bundle_model.sh"
  cp "$REPO_ROOT/tools/tests/assetpack_dev_bundle_fake_verifier.py" \
    "$root/tools/assetpack/verify_model_lock.py"
  chmod 700 \
    "$root/tools/assetpack/dev_bundle_model.sh" \
    "$root/tools/assetpack/verify_model_lock.py"
  printf '{}\n' >"$root/tools/assetpack/Manifest.json"
  printf '{}\n' >"$root/BioMotion/Resources/SAM3DBodyPose.lock.json"
  printf 'fixture license\n' >"$root/BioMotion/Resources/SAM-LICENSE.txt"
  printf 'valid-aar\n' \
    >"$root/build/assetpack/release/sam3d-body-pose.aar"
  printf 'valid-receipt\n' \
    >"$root/build/assetpack/release/sam3d-body-pose.aar.receipt.json"
  echo "$root"
}

inject_namespace_failure() {
  local root="$1"
  local operation="$2"
  /usr/bin/python3 -I - \
    "$root/tools/assetpack/dev_bundle_model.sh" "$operation" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
operation = sys.argv[2]
source = path.read_text(encoding="utf-8")
if operation not in {"exclusive", "swap"}:
    raise SystemExit(f"unsupported namespace failure operation: {operation}")
function = "rename_exclusive" if operation == "exclusive" else "rename_swap"
declaration = f"def {function}(source: Path, destination: Path) -> None:\n"
if source.count(declaration) != 1:
    raise SystemExit(f"cannot install isolated rename-{operation} failure")
source = source.replace(
    declaration,
    declaration
    + f'    Path("rename-{operation}-hit").write_text("hit\\n", encoding="utf-8")\n'
    + f'    raise OSError(errno.EIO, "injected test-only rename-{operation} failure")\n',
    1,
)
path.write_text(source, encoding="utf-8")
PY
}

inject_snapshot_pair_swap() {
  local root="$1"
  /usr/bin/python3 -I - \
    "$root/tools/assetpack/dev_bundle_model.sh" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
source = path.read_text(encoding="utf-8")
original = """    for source, destination in pairs:
        freeze_regular_file(source, destination)
"""
injected = """    for pair_index, (source, destination) in enumerate(pairs):
        freeze_regular_file(source, destination)
        Path(f"freeze-input-{pair_index + 1}-hit").write_text(
            "hit\\n", encoding="utf-8"
        )
        if pair_index == 0:
            import ctypes

            live_release = source.parent
            next_release = live_release.parent / "release-next"
            library = ctypes.CDLL(None, use_errno=True)
            rename = library.renameatx_np
            rename.argtypes = [
                ctypes.c_int,
                ctypes.c_char_p,
                ctypes.c_int,
                ctypes.c_char_p,
                ctypes.c_uint,
            ]
            rename.restype = ctypes.c_int
            if rename(
                -2,
                os.fsencode(live_release),
                -2,
                os.fsencode(next_release),
                0x2,
            ) != 0:
                error_number = ctypes.get_errno()
                raise FreezeError(
                    f"test-only release swap failed: {os.strerror(error_number)}"
                )
            Path("freeze-between-pair-swap-hit").write_text(
                "hit\\n", encoding="utf-8"
            )
"""
if source.count(original) != 1:
    raise SystemExit("cannot install isolated between-input release swap")
path.write_text(source.replace(original, injected, 1), encoding="utf-8")
PY
}

inject_namespace_trace() {
  local root="$1"
  local operation="$2"
  /usr/bin/python3 -I - \
    "$root/tools/assetpack/dev_bundle_model.sh" "$operation" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
operation = sys.argv[2]
source = path.read_text(encoding="utf-8")
if operation not in {"exclusive", "swap"}:
    raise SystemExit(f"unsupported namespace trace operation: {operation}")
function = "rename_exclusive" if operation == "exclusive" else "rename_swap"
flag = "RENAME_EXCL" if operation == "exclusive" else "RENAME_SWAP"
original = (
    f"def {function}(source: Path, destination: Path) -> None:\n"
    f"    renameatx(source, destination, {flag})\n"
)
injected = original + (
    f'    with Path("rename-{operation}-success.log").open('
    '"a", encoding="utf-8") as marker:\n'
    '        marker.write("success\\n")\n'
)
if source.count(original) != 1:
    raise SystemExit(f"cannot install isolated rename-{operation} trace")
path.write_text(source.replace(original, injected, 1), encoding="utf-8")
PY
}

inject_post_namespace_sync_failure() {
  local root="$1"
  /usr/bin/python3 -I - \
    "$root/tools/assetpack/dev_bundle_model.sh" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
source = path.read_text(encoding="utf-8")
original = """        fsync_directory(candidate.parent)
        fsync_directory(destination.parent)
    except Exception as error:
        rollback_or_require_recovery(
"""
injected = """        Path("post-namespace-sync-hit").write_text("hit\\n", encoding="utf-8")
        raise OSError(errno.EIO, "injected test-only post-namespace sync failure")
    except Exception as error:
        rollback_or_require_recovery(
"""
if source.count(original) != 1:
    raise SystemExit("cannot install isolated post-namespace sync failure")
path.write_text(source.replace(original, injected, 1), encoding="utf-8")
PY
}

inject_post_namespace_identity_failure() {
  local root="$1"
  local identity="$2"
  /usr/bin/python3 -I - \
    "$root/tools/assetpack/dev_bundle_model.sh" "$identity" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
identity = sys.argv[2]
labels = {
    "candidate": "swapped previous destination",
    "destination": "published destination",
}
if identity not in labels:
    raise SystemExit(f"unsupported post-namespace identity: {identity}")
source = path.read_text(encoding="utf-8")
marker = "PY_ATOMIC_PUBLISH"
parts = source.split(marker)
if len(parts) != 3:
    raise SystemExit("cannot isolate atomic publisher for identity failure")
publisher = parts[1]
declaration = "def directory_identity(path: Path, label: str) -> tuple[int, int]:\n"
if publisher.count(declaration) != 1:
    raise SystemExit("cannot install isolated post-namespace identity failure")
injected = declaration + (
    f'    if label == {labels[identity]!r}:\n'
    f'        Path("post-namespace-{identity}-identity-hit").write_text(\n'
    '            "hit\\n", encoding="utf-8"\n'
    '        )\n'
    '        raise OSError(errno.EIO, "injected test-only identity failure")\n'
)
parts[1] = publisher.replace(declaration, injected, 1)
path.write_text(marker.join(parts), encoding="utf-8")
PY
}

inject_rollback_swap_failure() {
  local root="$1"
  /usr/bin/python3 -I - \
    "$root/tools/assetpack/dev_bundle_model.sh" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
source = path.read_text(encoding="utf-8")
original = """def rename_swap(source: Path, destination: Path) -> None:
    renameatx(source, destination, RENAME_SWAP)
"""
injected = """rename_swap_calls = 0


def rename_swap(source: Path, destination: Path) -> None:
    global rename_swap_calls
    rename_swap_calls += 1
    if rename_swap_calls == 2:
        Path("rename-swap-rollback-failure-hit").write_text(
            "hit\\n", encoding="utf-8"
        )
        raise OSError(errno.EIO, "injected test-only rollback swap failure")
    renameatx(source, destination, RENAME_SWAP)
    with Path("rename-swap-success.log").open("a", encoding="utf-8") as marker:
        marker.write("success\\n")
"""
if source.count(original) != 1:
    raise SystemExit("cannot install isolated rollback swap failure")
path.write_text(source.replace(original, injected, 1), encoding="utf-8")
PY
}

inject_post_swap_signal() {
  local root="$1"
  /usr/bin/python3 -I - \
    "$root/tools/assetpack/dev_bundle_model.sh" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
source = path.read_text(encoding="utf-8")
original = """def rename_swap(source: Path, destination: Path) -> None:
    renameatx(source, destination, RENAME_SWAP)
"""
injected = """def rename_swap(source: Path, destination: Path) -> None:
    renameatx(source, destination, RENAME_SWAP)
    Path("post-swap-signal-hit").write_text("hit\\n", encoding="utf-8")
    import signal

    os.kill(os.getpid(), signal.SIGTERM)
"""
if source.count(original) != 1:
    raise SystemExit("cannot install isolated post-swap signal")
path.write_text(source.replace(original, injected, 1), encoding="utf-8")
PY
}

inject_post_swap_unclassified_exit_one() {
  local root="$1"
  /usr/bin/python3 -I - \
    "$root/tools/assetpack/dev_bundle_model.sh" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
source = path.read_text(encoding="utf-8")
original = """def rename_swap(source: Path, destination: Path) -> None:
    renameatx(source, destination, RENAME_SWAP)
"""
injected = """def rename_swap(source: Path, destination: Path) -> None:
    renameatx(source, destination, RENAME_SWAP)
    Path("post-swap-unclassified-one-hit").write_text(
        "hit\\n", encoding="utf-8"
    )
    os._exit(1)
"""
if source.count(original) != 1:
    raise SystemExit("cannot install isolated post-swap unclassified exit")
path.write_text(source.replace(original, injected, 1), encoding="utf-8")
PY
}

seed_old_output() {
  local root="$1"
  mkdir -p "$root/build/DevBundledModel/SAM3DBodyPose.mlmodelc"
  printf 'known-good-old-model\n' \
    >"$root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/old.bin"
}

run_on() {
  local root="$1"
  shift
  set +e
  (
    cd "$root"
    /bin/bash tools/assetpack/dev_bundle_model.sh on "$@"
  ) >"$root/dev-bundle.out" 2>&1
  RUN_STATUS=$?
  set -e
}

run_on_with_release_swap() {
  local root="$1"
  set +e
  (
    export BIOMOTION_FAKE_LIVE_RELEASE="$root/build/assetpack/release"
    export BIOMOTION_FAKE_NEXT_RELEASE="$root/build/assetpack/release-next"
    cd "$root"
    /bin/bash tools/assetpack/dev_bundle_model.sh on
  ) >"$root/dev-bundle.out" 2>&1
  RUN_STATUS=$?
  set -e
}

run_off() {
  local root="$1"
  set +e
  (
    cd "$root"
    /bin/bash tools/assetpack/dev_bundle_model.sh off
  ) >"$root/dev-bundle.out" 2>&1
  RUN_STATUS=$?
  set -e
}

assert_old_output() {
  local root="$1"
  [[ "$(<"$root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/old.bin")" \
      == "known-good-old-model" ]] || fail "previous output was not preserved"
  [[ ! -e "$root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/coremldata.bin" ]] || \
    fail "failed transaction leaked a new compiled model"
}

assert_no_transactions() {
  local root="$1"
  if find "$root/build" -maxdepth 1 -name '.dev-bundle-model.*' -print -quit \
      | grep -q .; then
    fail "private developer-bundle transaction was not cleaned"
  fi
}

assert_namespace_success_count() {
  local root="$1"
  local operation="$2"
  local expected="$3"
  local log="$root/rename-$operation-success.log"
  [[ -f "$log" ]] || fail "rename-$operation success marker is missing"
  local actual
  actual="$(/usr/bin/awk 'END { print NR + 0 }' "$log")"
  [[ "$actual" -eq "$expected" ]] || \
    fail "rename-$operation ran $actual successful times, expected $expected"
}

/usr/bin/python3 -I - "$REPO_ROOT/tools/assetpack/dev_bundle_model.sh" <<'PY'
from pathlib import Path
import sys

source = Path(sys.argv[1]).read_text(encoding="utf-8")
if "PY_FREEZE_INPUTS" not in source:
    raise SystemExit("descriptor-based input snapshot helper is missing")
for required in (
    '"$snapshot_aar" "$snapshot_receipt"',
    'getattr(os, "O_NOFOLLOW", 0)',
    "os.fsync(destination_descriptor)",
    "source input parent changed while freezing pair",
):
    if required not in source:
        raise SystemExit(f"input snapshot lost required guard: {required}")
marker = "PY_ATOMIC_PUBLISH"
parts = source.split(marker)
if len(parts) != 3:
    raise SystemExit("atomic publisher heredoc is missing")
publisher = parts[1]
for required in (
    "renameatx_np",
    "RENAME_SWAP",
    "rename_swap(candidate, destination)",
    "fsync_tree(candidate)",
):
    if required not in publisher:
        raise SystemExit(f"atomic replacement lost {required}")
if publisher.count("fsync_directory(candidate.parent)") != 3:
    raise SystemExit("publisher must sync the candidate parent before and after publish, and after rollback")
if publisher.count("fsync_directory(destination.parent)") != 2:
    raise SystemExit("publisher must sync the destination parent after publish and rollback")
for forbidden in (
    "shutil.rmtree(destination",
    "os.remove(destination",
    "os.unlink(destination",
    "destination.unlink(",
    "os.rename(destination",
):
    if forbidden in publisher:
        raise SystemExit(
            f"replacement publisher reintroduced a remove-old window: {forbidden}"
        )
for forbidden in (
    "BIOMOTION_FAKE_",
    "freeze-input-",
    "injected test-only",
    "post-namespace-sync-hit",
    "post-namespace-candidate-identity-hit",
    "post-namespace-destination-identity-hit",
    "post-swap-signal-hit",
    "post-swap-unclassified-one-hit",
    "rename-swap-success.log",
):
    if forbidden in source:
        raise SystemExit(f"production script contains a test override or marker: {forbidden}")
PY

first_root="$(make_fixture first-publication)"
run_on "$first_root"
[[ "$RUN_STATUS" -eq 0 ]] || \
  fail "first publication failed: $(<"$first_root/dev-bundle.out")"
[[ "$(<"$first_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/coremldata.bin")" \
    == "verified compiled model" ]] || fail "first publication lost compiled model"
assert_no_transactions "$first_root"
pass "first publication atomically installs into an absent destination"

first_failure_root="$(make_fixture first-publication-failure)"
inject_namespace_failure "$first_failure_root" exclusive
run_on "$first_failure_root"
[[ "$RUN_STATUS" -ne 0 ]] || fail "injected first-publication failure published"
[[ -f "$first_failure_root/rename-exclusive-hit" ]] || \
  fail "first-publication failure did not dynamically reach rename_exclusive"
[[ ! -e "$first_failure_root/build/DevBundledModel" ]] || \
  fail "failed first publication created a destination"
assert_no_transactions "$first_failure_root"
pass "single namespace first-publication failure leaves destination absent"

first_sync_failure_root="$(make_fixture first-post-namespace-sync-failure)"
inject_namespace_trace "$first_sync_failure_root" exclusive
inject_post_namespace_sync_failure "$first_sync_failure_root"
run_on "$first_sync_failure_root"
[[ "$RUN_STATUS" -eq 1 ]] || \
  fail "first-publication post-namespace failure returned $RUN_STATUS, expected 1"
[[ -f "$first_sync_failure_root/fake-verifier.log" ]] || \
  fail "first-publication rollback test did not dynamically reach the verifier"
[[ -f "$first_sync_failure_root/post-namespace-sync-hit" ]] || \
  fail "first-publication rollback test did not reach post-namespace sync"
assert_namespace_success_count "$first_sync_failure_root" exclusive 2
[[ ! -e "$first_sync_failure_root/build/DevBundledModel" ]] || \
  fail "exclusive rollback did not restore an absent destination"
assert_no_transactions "$first_sync_failure_root"
pass "post-namespace first-publication failure rolls back to an absent destination"

first_identity_failure_root="$(make_fixture first-post-namespace-identity-failure)"
inject_namespace_trace "$first_identity_failure_root" exclusive
inject_post_namespace_identity_failure "$first_identity_failure_root" destination
run_on "$first_identity_failure_root"
[[ "$RUN_STATUS" -eq 1 ]] || \
  fail "first-publication identity failure returned $RUN_STATUS, expected 1"
[[ -f "$first_identity_failure_root/fake-verifier.log" ]] || \
  fail "first identity rollback test did not dynamically reach the verifier"
[[ -f "$first_identity_failure_root/post-namespace-destination-identity-hit" ]] || \
  fail "first identity rollback test did not reach the post-rename inspection"
assert_namespace_success_count "$first_identity_failure_root" exclusive 2
[[ ! -e "$first_identity_failure_root/build/DevBundledModel" ]] || \
  fail "identity-exception rollback did not restore an absent destination"
assert_no_transactions "$first_identity_failure_root"
pass "post-namespace first identity exception crosses the rollback boundary"

success_root="$(make_fixture success)"
seed_old_output "$success_root"
run_on "$success_root"
[[ "$RUN_STATUS" -eq 0 ]] || \
  fail "valid AAR/receipt pair did not enable the dev bundle: $(<"$success_root/dev-bundle.out")"
[[ "$(<"$success_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/coremldata.bin")" \
    == "verified compiled model" ]] || fail "verified compiled model was not installed"
[[ ! -e "$success_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/old.bin" ]] || \
  fail "atomic replacement retained the old model tree"
[[ ! -e "$success_root/build/DevBundledModel/SAM3DBodyPose.mlpackage" ]] || \
  fail "developer output contains a raw model package"
/usr/bin/python3 -I - "$success_root/fake-verifier.log" "$success_root" <<'PY'
import json
from pathlib import Path
import sys

record = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
root = Path(sys.argv[2]).resolve()
if record["isolated"] != 1 or record["mode"] != "receipt":
    raise SystemExit("verifier invocation was not isolated receipt verification")
extract = Path(record["extractDirectory"])
if extract.parent.parent != root / "build":
    raise SystemExit(f"extraction was not inside a build-local transaction: {extract}")
aar = Path(record["aar"])
receipt = Path(record["receipt"])
if aar.parent != extract.parent / "inputs" or receipt.parent != aar.parent:
    raise SystemExit("verifier did not consume the transaction-frozen input pair")
if aar.name != "sam3d-body-pose.aar":
    raise SystemExit("snapshot AAR lost its canonical filename")
if receipt.name != "sam3d-body-pose.aar.receipt.json":
    raise SystemExit("snapshot receipt lost its canonical filename")
if record["transactionMode"] != "700":
    raise SystemExit("developer-bundle transaction was not private")
PY
assert_no_transactions "$success_root"
pass "verified receipt extraction atomically installs only the compiled model"

replacement_sync_failure_root="$(make_fixture replacement-post-namespace-sync-failure)"
seed_old_output "$replacement_sync_failure_root"
replacement_old_digest="$(/usr/bin/shasum -a 256 \
  "$replacement_sync_failure_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/old.bin" \
  | /usr/bin/awk '{ print $1 }')"
inject_namespace_trace "$replacement_sync_failure_root" swap
inject_post_namespace_sync_failure "$replacement_sync_failure_root"
run_on "$replacement_sync_failure_root"
[[ "$RUN_STATUS" -eq 1 ]] || \
  fail "replacement post-namespace failure returned $RUN_STATUS, expected 1"
[[ -f "$replacement_sync_failure_root/fake-verifier.log" ]] || \
  fail "replacement rollback test did not dynamically reach the verifier"
[[ -f "$replacement_sync_failure_root/post-namespace-sync-hit" ]] || \
  fail "replacement rollback test did not reach post-namespace sync"
assert_namespace_success_count "$replacement_sync_failure_root" swap 2
[[ "$(/usr/bin/shasum -a 256 \
    "$replacement_sync_failure_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/old.bin" \
    | /usr/bin/awk '{ print $1 }')" == "$replacement_old_digest" ]] || \
  fail "swap rollback changed the previous output bytes"
assert_old_output "$replacement_sync_failure_root"
assert_no_transactions "$replacement_sync_failure_root"
pass "post-namespace replacement failure swaps the exact old output back"

replacement_identity_failure_root="$(make_fixture replacement-identity-failure)"
seed_old_output "$replacement_identity_failure_root"
replacement_identity_old_digest="$(/usr/bin/shasum -a 256 \
  "$replacement_identity_failure_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/old.bin" \
  | /usr/bin/awk '{ print $1 }')"
inject_namespace_trace "$replacement_identity_failure_root" swap
inject_post_namespace_identity_failure "$replacement_identity_failure_root" candidate
run_on "$replacement_identity_failure_root"
[[ "$RUN_STATUS" -eq 1 ]] || \
  fail "replacement identity failure returned $RUN_STATUS, expected 1"
[[ -f "$replacement_identity_failure_root/fake-verifier.log" ]] || \
  fail "replacement identity rollback test did not reach the verifier"
[[ -f "$replacement_identity_failure_root/post-namespace-candidate-identity-hit" ]] || \
  fail "replacement identity rollback test did not reach displaced-output inspection"
assert_namespace_success_count "$replacement_identity_failure_root" swap 2
[[ "$(/usr/bin/shasum -a 256 \
    "$replacement_identity_failure_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/old.bin" \
    | /usr/bin/awk '{ print $1 }')" == "$replacement_identity_old_digest" ]] || \
  fail "identity-exception rollback changed the previous output bytes"
assert_old_output "$replacement_identity_failure_root"
assert_no_transactions "$replacement_identity_failure_root"
pass "post-namespace replacement identity exception restores the exact old output"

rollback_failure_root="$(make_fixture rollback-swap-failure)"
seed_old_output "$rollback_failure_root"
inject_rollback_swap_failure "$rollback_failure_root"
inject_post_namespace_sync_failure "$rollback_failure_root"
run_on "$rollback_failure_root"
[[ "$RUN_STATUS" -eq 2 ]] || \
  fail "failed rollback returned $RUN_STATUS, expected recovery status 2"
[[ -f "$rollback_failure_root/fake-verifier.log" ]] || \
  fail "failed-rollback test did not dynamically reach the verifier"
[[ -f "$rollback_failure_root/post-namespace-sync-hit" ]] || \
  fail "failed-rollback test did not dynamically reach post-namespace sync"
[[ -f "$rollback_failure_root/rename-swap-rollback-failure-hit" ]] || \
  fail "failed-rollback test did not dynamically reach the second swap"
assert_namespace_success_count "$rollback_failure_root" swap 1
[[ "$(<"$rollback_failure_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/coremldata.bin")" \
    == "verified compiled model" ]] || \
  fail "failed rollback did not leave an identifiable verified model live"
[[ ! -e "$rollback_failure_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/old.bin" ]] || \
  fail "failed rollback mixed old and new destination trees"
recovery_transaction="$(find "$rollback_failure_root/build" -maxdepth 1 -type d \
  -name '.dev-bundle-model.*' -print -quit)"
[[ -n "$recovery_transaction" ]] || \
  fail "failed rollback did not preserve its recovery transaction"
recovery_transaction_count="$(find "$rollback_failure_root/build" -maxdepth 1 -type d \
  -name '.dev-bundle-model.*' -print | /usr/bin/awk 'END { print NR + 0 }')"
[[ "$recovery_transaction_count" -eq 1 ]] || \
  fail "failed rollback preserved $recovery_transaction_count transactions, expected one"
[[ "$(<"$recovery_transaction/publish-candidate/SAM3DBodyPose.mlmodelc/old.bin")" \
    == "known-good-old-model" ]] || \
  fail "failed rollback did not preserve the exact old model in its transaction"
[[ ! -e "$recovery_transaction/publish-candidate/SAM3DBodyPose.mlmodelc/coremldata.bin" ]] || \
  fail "failed rollback mixed new bytes into the preserved old model"
/usr/bin/python3 -I - \
  "$rollback_failure_root/dev-bundle.out" \
  "$recovery_transaction" \
  "$rollback_failure_root/build/DevBundledModel" <<'PY'
from pathlib import Path
import sys

output = Path(sys.argv[1]).read_text(encoding="utf-8")
expected = {
    "transaction": Path(sys.argv[2]).resolve(),
    "destination": Path(sys.argv[3]).resolve(),
}
reported: dict[str, Path] = {}
prefix = "DEV_BUNDLE_RECOVERY_REQUIRED "
for line in output.splitlines():
    if not line.startswith(prefix) or ": " not in line:
        continue
    label, value = line[len(prefix) :].split(": ", 1)
    if label in expected:
        reported[label] = Path(value).resolve()
if reported != expected:
    raise SystemExit(
        f"recovery paths do not identify both preserved models: "
        f"expected {expected}, found {reported}"
    )
PY
pass "rollback swap failure preserves both identifiable models for explicit recovery"

signal_root="$(make_fixture post-swap-signal)"
seed_old_output "$signal_root"
inject_post_swap_signal "$signal_root"
run_on "$signal_root"
[[ "$RUN_STATUS" -eq 143 ]] || \
  fail "post-swap SIGTERM returned $RUN_STATUS, expected 143"
[[ -f "$signal_root/fake-verifier.log" ]] || \
  fail "post-swap signal test did not dynamically reach the verifier"
[[ -f "$signal_root/post-swap-signal-hit" ]] || \
  fail "post-swap signal test did not reach the successful namespace operation"
[[ "$(<"$signal_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/coremldata.bin")" \
    == "verified compiled model" ]] || \
  fail "post-swap signal did not leave one identifiable verified model live"
signal_transaction="$(find "$signal_root/build" -maxdepth 1 -type d \
  -name '.dev-bundle-model.*' -print -quit)"
[[ -n "$signal_transaction" ]] || \
  fail "post-swap signal cleanup erased the displaced old model"
[[ "$(<"$signal_transaction/publish-candidate/SAM3DBodyPose.mlmodelc/old.bin")" \
    == "known-good-old-model" ]] || \
  fail "post-swap signal did not preserve the exact old model"
/usr/bin/python3 -I - \
  "$signal_root/dev-bundle.out" \
  "$signal_transaction" \
  "$signal_root/build/DevBundledModel" <<'PY'
from pathlib import Path
import sys

output = Path(sys.argv[1]).read_text(encoding="utf-8")
expected = {
    "transaction": Path(sys.argv[2]).resolve(),
    "destination": Path(sys.argv[3]).resolve(),
}
reported: dict[str, Path] = {}
prefix = "DEV_BUNDLE_RECOVERY_REQUIRED "
for line in output.splitlines():
    if line.startswith(prefix) and ": " in line:
        label, value = line[len(prefix) :].split(": ", 1)
        if label in expected:
            reported[label] = Path(value).resolve()
if reported != expected:
    raise SystemExit(
        f"signal recovery paths do not identify both models: "
        f"expected {expected}, found {reported}"
    )
PY
pass "post-swap signal preserves the live and displaced models for recovery"

unclassified_root="$(make_fixture post-swap-unclassified-exit-one)"
seed_old_output "$unclassified_root"
inject_post_swap_unclassified_exit_one "$unclassified_root"
run_on "$unclassified_root"
[[ "$RUN_STATUS" -eq 1 ]] || \
  fail "post-swap unclassified exit returned $RUN_STATUS, expected raw status 1"
[[ -f "$unclassified_root/fake-verifier.log" ]] || \
  fail "post-swap unclassified-exit test did not reach the verifier"
[[ -f "$unclassified_root/post-swap-unclassified-one-hit" ]] || \
  fail "post-swap unclassified-exit test did not reach the successful swap"
[[ "$(<"$unclassified_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/coremldata.bin")" \
    == "verified compiled model" ]] || \
  fail "unclassified exit did not leave one identifiable verified model live"
unclassified_transaction="$(find "$unclassified_root/build" -maxdepth 1 -type d \
  -name '.dev-bundle-model.*' -print -quit)"
[[ -n "$unclassified_transaction" ]] || \
  fail "raw status 1 incorrectly disarmed preservation and erased the old model"
[[ "$(<"$unclassified_transaction/publish-candidate/SAM3DBodyPose.mlmodelc/old.bin")" \
    == "known-good-old-model" ]] || \
  fail "unclassified exit did not preserve the exact old model"
/usr/bin/python3 -I - \
  "$unclassified_root/dev-bundle.out" \
  "$unclassified_transaction" \
  "$unclassified_root/build/DevBundledModel" <<'PY'
from pathlib import Path
import sys

output = Path(sys.argv[1]).read_text(encoding="utf-8")
expected = {
    "transaction": Path(sys.argv[2]).resolve(),
    "destination": Path(sys.argv[3]).resolve(),
}
reported: dict[str, Path] = {}
prefix = "DEV_BUNDLE_RECOVERY_REQUIRED "
for line in output.splitlines():
    if line.startswith(prefix) and ": " in line:
        label, value = line[len(prefix) :].split(": ", 1)
        if label in expected:
            reported[label] = Path(value).resolve()
if reported != expected:
    raise SystemExit(
        f"unclassified-exit recovery paths do not identify both models: "
        f"expected {expected}, found {reported}"
    )
PY
pass "post-swap unclassified status 1 cannot disarm recovery preservation"

explicit_root="$(make_fixture explicit-pair)"
seed_old_output "$explicit_root"
mkdir -p "$explicit_root/input"
printf 'valid-aar\n' >"$explicit_root/input/sam3d-body-pose.aar"
printf 'valid-receipt\n' \
  >"$explicit_root/input/sam3d-body-pose.aar.receipt.json"
run_on \
  "$explicit_root" \
  "$explicit_root/input/sam3d-body-pose.aar" \
  "$explicit_root/input/sam3d-body-pose.aar.receipt.json"
[[ "$RUN_STATUS" -eq 0 ]] || fail "explicit verified pair was rejected"
[[ -f "$explicit_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/coremldata.bin" ]] || \
  fail "explicit verified pair was not installed"
assert_no_transactions "$explicit_root"
pass "explicit AAR and receipt pair is supported"

mid_copy_swap_root="$(make_fixture between-input-generation-swap)"
seed_old_output "$mid_copy_swap_root"
printf 'valid-aar:generation-a\n' \
  >"$mid_copy_swap_root/build/assetpack/release/sam3d-body-pose.aar"
printf 'valid-receipt:generation-a\n' \
  >"$mid_copy_swap_root/build/assetpack/release/sam3d-body-pose.aar.receipt.json"
mkdir -p "$mid_copy_swap_root/build/assetpack/release-next"
printf 'valid-aar:generation-b\n' \
  >"$mid_copy_swap_root/build/assetpack/release-next/sam3d-body-pose.aar"
printf 'valid-receipt:generation-b\n' \
  >"$mid_copy_swap_root/build/assetpack/release-next/sam3d-body-pose.aar.receipt.json"
inject_snapshot_pair_swap "$mid_copy_swap_root"
run_on "$mid_copy_swap_root"
[[ "$RUN_STATUS" -ne 0 ]] || fail "between-input release swap unexpectedly published"
for marker in freeze-input-1-hit freeze-between-pair-swap-hit freeze-input-2-hit; do
  [[ -f "$mid_copy_swap_root/$marker" ]] || \
    fail "between-input release swap did not dynamically hit $marker"
done
[[ ! -e "$mid_copy_swap_root/fake-verifier.log" ]] || \
  fail "mixed snapshot reached the verifier instead of failing closed"
/usr/bin/grep -q "source input parent changed while freezing pair" \
  "$mid_copy_swap_root/dev-bundle.out" || \
  fail "between-input release swap did not fail on parent identity"
assert_old_output "$mid_copy_swap_root"
assert_no_transactions "$mid_copy_swap_root"
pass "release swap between AAR and receipt copies fails closed before verification"

generation_root="$(make_fixture generation-swap)"
seed_old_output "$generation_root"
printf 'valid-aar:generation-a\n' \
  >"$generation_root/build/assetpack/release/sam3d-body-pose.aar"
printf 'valid-receipt:generation-a\n' \
  >"$generation_root/build/assetpack/release/sam3d-body-pose.aar.receipt.json"
mkdir -p "$generation_root/build/assetpack/release-next"
printf 'valid-aar:generation-b\n' \
  >"$generation_root/build/assetpack/release-next/sam3d-body-pose.aar"
printf 'valid-receipt:generation-b\n' \
  >"$generation_root/build/assetpack/release-next/sam3d-body-pose.aar.receipt.json"
run_on_with_release_swap "$generation_root"
[[ "$RUN_STATUS" -eq 0 ]] || \
  fail "source generation swap escaped the frozen snapshot: $(<"$generation_root/dev-bundle.out")"
[[ "$(<"$generation_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/coremldata.bin")" \
    == "verified compiled model generation-a" ]] || \
  fail "installed model did not come from the frozen generation-a snapshot"
[[ "$(<"$generation_root/build/assetpack/release/sam3d-body-pose.aar")" \
    == "valid-aar:generation-b" ]] || fail "fake verifier did not swap the live release"
/usr/bin/python3 -I - "$generation_root/fake-verifier.log" <<'PY'
import json
from pathlib import Path
import sys

record = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
if not record["sourceSwapped"]:
    raise SystemExit("fake verifier did not dynamically swap the live release")
if record["generation"] != "generation-a":
    raise SystemExit("verification/extraction did not stay bound to frozen generation-a")
if Path(record["aar"]).parent.name != "inputs":
    raise SystemExit("generation test verifier received a live rather than snapshot AAR")
PY
assert_no_transactions "$generation_root"
pass "verification-time source swap stays bound to one frozen generation"

swap_failure_root="$(make_fixture swap-failure)"
seed_old_output "$swap_failure_root"
old_digest="$(/usr/bin/shasum -a 256 \
  "$swap_failure_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/old.bin" \
  | /usr/bin/awk '{ print $1 }')"
inject_namespace_failure "$swap_failure_root" swap
run_on "$swap_failure_root"
[[ "$RUN_STATUS" -ne 0 ]] || fail "injected rename-swap failure published"
[[ -f "$swap_failure_root/rename-swap-hit" ]] || \
  fail "replacement failure did not dynamically reach rename_swap"
[[ "$(/usr/bin/shasum -a 256 \
    "$swap_failure_root/build/DevBundledModel/SAM3DBodyPose.mlmodelc/old.bin" \
    | /usr/bin/awk '{ print $1 }')" == "$old_digest" ]] || \
  fail "rename-swap failure changed previous output bytes"
assert_old_output "$swap_failure_root"
assert_no_transactions "$swap_failure_root"
pass "single namespace replacement failure preserves old output byte-for-byte"

for fault in bad-aar bad-receipt; do
  root="$(make_fixture "$fault")"
  seed_old_output "$root"
  if [[ "$fault" == "bad-aar" ]]; then
    printf 'tampered-aar\n' >"$root/build/assetpack/release/sam3d-body-pose.aar"
  else
    printf 'tampered-receipt\n' \
      >"$root/build/assetpack/release/sam3d-body-pose.aar.receipt.json"
  fi
  run_on "$root"
  [[ "$RUN_STATUS" -ne 0 ]] || fail "$fault unexpectedly published"
  assert_old_output "$root"
  assert_no_transactions "$root"
  pass "$fault preserves the previous output"
done

raw_root="$(make_fixture raw-package)"
seed_old_output "$raw_root"
mkdir -p "$raw_root/input/SAM3DBodyPose.mlpackage"
printf 'raw model\n' >"$raw_root/input/SAM3DBodyPose.mlpackage/Manifest.json"
run_on "$raw_root" "$raw_root/input/SAM3DBodyPose.mlpackage"
[[ "$RUN_STATUS" -ne 0 ]] || fail "raw model package was accepted"
assert_old_output "$raw_root"
assert_no_transactions "$raw_root"
pass "raw model package input is rejected"

symlink_input_root="$(make_fixture symlink-input)"
seed_old_output "$symlink_input_root"
ln -s \
  "$symlink_input_root/build/assetpack/release/sam3d-body-pose.aar" \
  "$symlink_input_root/sam3d-body-pose.aar"
run_on "$symlink_input_root" "$symlink_input_root/sam3d-body-pose.aar"
[[ "$RUN_STATUS" -ne 0 ]] || fail "symlink AAR was accepted"
assert_old_output "$symlink_input_root"
assert_no_transactions "$symlink_input_root"
pass "symlink AAR is rejected without touching old output"

symlink_receipt_root="$(make_fixture symlink-receipt)"
seed_old_output "$symlink_receipt_root"
mv \
  "$symlink_receipt_root/build/assetpack/release/sam3d-body-pose.aar.receipt.json" \
  "$symlink_receipt_root/build/assetpack/release/receipt-target.json"
ln -s \
  "$symlink_receipt_root/build/assetpack/release/receipt-target.json" \
  "$symlink_receipt_root/build/assetpack/release/sam3d-body-pose.aar.receipt.json"
run_on "$symlink_receipt_root"
[[ "$RUN_STATUS" -ne 0 ]] || fail "symlink receipt was accepted"
assert_old_output "$symlink_receipt_root"
assert_no_transactions "$symlink_receipt_root"
pass "symlink receipt is rejected before verification"

special_input_root="$(make_fixture special-input)"
seed_old_output "$special_input_root"
mkfifo "$special_input_root/sam3d-body-pose.aar"
run_on "$special_input_root" "$special_input_root/sam3d-body-pose.aar"
[[ "$RUN_STATUS" -ne 0 ]] || fail "special-file AAR was accepted"
assert_old_output "$special_input_root"
assert_no_transactions "$special_input_root"
pass "special-file AAR is rejected without blocking or publishing"

special_receipt_root="$(make_fixture special-receipt)"
seed_old_output "$special_receipt_root"
rm "$special_receipt_root/build/assetpack/release/sam3d-body-pose.aar.receipt.json"
mkfifo \
  "$special_receipt_root/build/assetpack/release/sam3d-body-pose.aar.receipt.json"
run_on "$special_receipt_root"
[[ "$RUN_STATUS" -ne 0 ]] || fail "special-file receipt was accepted"
assert_old_output "$special_receipt_root"
assert_no_transactions "$special_receipt_root"
pass "special-file receipt is rejected before verification"

for tree_fault in symlink-tree special-tree; do
  root="$(make_fixture "$tree_fault")"
  seed_old_output "$root"
  printf '%s\n' "$tree_fault" \
    >"$root/build/assetpack/release/sam3d-body-pose.aar"
  run_on "$root"
  [[ "$RUN_STATUS" -ne 0 ]] || fail "$tree_fault unexpectedly published"
  assert_old_output "$root"
  assert_no_transactions "$root"
  pass "$tree_fault from a verifier is rejected before publication"
done

destination_link_root="$(make_fixture destination-link)"
outside="$TEST_ROOT/outside-destination"
mkdir -p "$outside/SAM3DBodyPose.mlmodelc"
printf 'outside-old\n' >"$outside/SAM3DBodyPose.mlmodelc/old.bin"
ln -s "$outside" "$destination_link_root/build/DevBundledModel"
run_on "$destination_link_root"
[[ "$RUN_STATUS" -ne 0 ]] || fail "symlink destination was accepted"
[[ "$(<"$outside/SAM3DBodyPose.mlmodelc/old.bin")" == "outside-old" ]] || \
  fail "symlink destination target was modified"
assert_no_transactions "$destination_link_root"
pass "symlink destination is rejected without following it"

build_link_root="$(make_fixture build-link)"
outside_build="$TEST_ROOT/outside-build"
mv "$build_link_root/build" "$outside_build"
ln -s "$outside_build" "$build_link_root/build"
run_on "$build_link_root"
[[ "$RUN_STATUS" -ne 0 ]] || fail "symlink build root was accepted"
[[ ! -e "$outside_build/DevBundledModel" ]] || \
  fail "symlink build root was followed"
pass "symlink build root is rejected"

off_root="$(make_fixture off-exact-destination)"
seed_old_output "$off_root"
printf 'adjacent-sentinel\n' >"$off_root/build/keep-adjacent.txt"
run_off "$off_root"
[[ "$RUN_STATUS" -eq 0 ]] || \
  fail "off rejected a safe developer bundle: $(<"$off_root/dev-bundle.out")"
[[ ! -e "$off_root/build/DevBundledModel" && ! -L "$off_root/build/DevBundledModel" ]] || \
  fail "off did not remove the exact developer bundle destination"
[[ "$(<"$off_root/build/keep-adjacent.txt")" == "adjacent-sentinel" ]] || \
  fail "off modified an adjacent build-root sentinel"
/usr/bin/grep -q "developer model bundle disabled" "$off_root/dev-bundle.out" || \
  fail "off did not dynamically reach its successful removal branch"
assert_no_transactions "$off_root"
pass "off removes only the exact developer bundle and preserves adjacent build data"

run_off "$off_root"
[[ "$RUN_STATUS" -eq 0 ]] || \
  fail "a second off invocation was not idempotent: $(<"$off_root/dev-bundle.out")"
[[ ! -e "$off_root/build/DevBundledModel" && ! -L "$off_root/build/DevBundledModel" ]] || \
  fail "idempotent off recreated the developer bundle destination"
[[ "$(<"$off_root/build/keep-adjacent.txt")" == "adjacent-sentinel" ]] || \
  fail "idempotent off modified adjacent build data"
/usr/bin/grep -q "developer model bundle already disabled" "$off_root/dev-bundle.out" || \
  fail "second off did not dynamically reach the already-disabled branch"
assert_no_transactions "$off_root"
pass "off is idempotent when the exact destination is already absent"

off_destination_link_root="$(make_fixture off-destination-link)"
off_outside_destination="$TEST_ROOT/off-outside-destination"
mkdir -p "$off_outside_destination/SAM3DBodyPose.mlmodelc"
printf 'outside-off-old\n' \
  >"$off_outside_destination/SAM3DBodyPose.mlmodelc/old.bin"
ln -s "$off_outside_destination" \
  "$off_destination_link_root/build/DevBundledModel"
run_off "$off_destination_link_root"
[[ "$RUN_STATUS" -ne 0 ]] || fail "off followed a symlink destination"
[[ -L "$off_destination_link_root/build/DevBundledModel" ]] || \
  fail "off removed the rejected destination symlink"
[[ "$(<"$off_outside_destination/SAM3DBodyPose.mlmodelc/old.bin")" \
    == "outside-off-old" ]] || \
  fail "off modified the symlink destination target"
/usr/bin/grep -q "destination must be a non-symlink directory" \
  "$off_destination_link_root/dev-bundle.out" || \
  fail "off destination-symlink test did not reach the lstat guard"
assert_no_transactions "$off_destination_link_root"
pass "off rejects a destination symlink without changing its target"

off_build_link_root="$(make_fixture off-build-link)"
seed_old_output "$off_build_link_root"
off_outside_build="$TEST_ROOT/off-outside-build"
mv "$off_build_link_root/build" "$off_outside_build"
ln -s "$off_outside_build" "$off_build_link_root/build"
run_off "$off_build_link_root"
[[ "$RUN_STATUS" -ne 0 ]] || fail "off followed a symlink build root"
[[ -L "$off_build_link_root/build" ]] || \
  fail "off removed the rejected build-root symlink"
[[ "$(<"$off_outside_build/DevBundledModel/SAM3DBodyPose.mlmodelc/old.bin")" \
    == "known-good-old-model" ]] || \
  fail "off modified the symlinked build-root target"
/usr/bin/grep -q "build root must be a non-symlink directory" \
  "$off_build_link_root/dev-bundle.out" || \
  fail "off build-root-symlink test did not reach the lstat guard"
pass "off rejects a symlink build root without changing its target"

echo "ASSETPACK_DEV_BUNDLE_RECEIPT_TESTS_PASS ($PASS_COUNT cases)"
