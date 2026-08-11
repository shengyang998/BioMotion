#!/bin/bash
# Install or remove the receipt-verified precompiled SAM model used by local builds.
#
#   /bin/bash tools/assetpack/dev_bundle_model.sh on \
#     [path/to/sam3d-body-pose.aar] \
#     [path/to/sam3d-body-pose.aar.receipt.json]
#   /bin/bash tools/assetpack/dev_bundle_model.sh off
#
# `on` never accepts or compiles a source model. It freezes the canonical AAR
# and receipt into a private build-local transaction, verifies and extracts
# only that immutable pair, and atomically publishes SAM3DBodyPose.mlmodelc.
set -euo pipefail
umask 077
PATH="/usr/bin:/bin:/usr/sbin:/sbin"
export PATH

SCRIPT_DIRECTORY="$(cd "$(/usr/bin/dirname "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd "$SCRIPT_DIRECTORY/../.." && pwd -P)"
PACK_ID="sam3d-body-pose"
MODEL_NAME="SAM3DBodyPose"
MODEL_FILE_NAME="${MODEL_NAME}.mlmodelc"
AAR_FILE_NAME="${PACK_ID}.aar"
RECEIPT_FILE_NAME="${AAR_FILE_NAME}.receipt.json"
BUILD_ROOT="$REPO_ROOT/build"
DESTINATION="$BUILD_ROOT/DevBundledModel"
DEFAULT_AAR="$BUILD_ROOT/assetpack/release/$AAR_FILE_NAME"
MANIFEST="$REPO_ROOT/tools/assetpack/Manifest.json"
LOCK_FILE="$REPO_ROOT/BioMotion/Resources/${MODEL_NAME}.lock.json"
LICENSE_FILE="$REPO_ROOT/BioMotion/Resources/SAM-LICENSE.txt"
VERIFIER="$REPO_ROOT/tools/assetpack/verify_model_lock.py"
PYTHON3="/usr/bin/python3"
TRANSACTION_DIRECTORY=""
PRESERVE_TRANSACTION=0

usage() {
  echo "usage: $0 {on|off} [path/to/$AAR_FILE_NAME] [path/to/$RECEIPT_FILE_NAME]" >&2
}

cleanup() {
  [[ -n "$TRANSACTION_DIRECTORY" ]] || return 0
  if [[ "$PRESERVE_TRANSACTION" -eq 1 ]]; then
    echo "DEV_BUNDLE_RECOVERY_REQUIRED transaction: $TRANSACTION_DIRECTORY" >&2
    echo "DEV_BUNDLE_RECOVERY_REQUIRED destination: $DESTINATION" >&2
    return
  fi
  if [[ "${TRANSACTION_DIRECTORY%/*}" != "$BUILD_ROOT" ]]; then
    echo "warning: refusing to clean transaction outside build root: $TRANSACTION_DIRECTORY" >&2
    return
  fi
  case "${TRANSACTION_DIRECTORY##*/}" in
    .dev-bundle-model.*) ;;
    *)
      echo "warning: refusing to clean unexpected transaction: $TRANSACTION_DIRECTORY" >&2
      return
      ;;
  esac
  if [[ -L "$TRANSACTION_DIRECTORY" || ! -d "$TRANSACTION_DIRECTORY" ]]; then
    echo "warning: refusing to clean non-directory transaction: $TRANSACTION_DIRECTORY" >&2
    return
  fi
  /bin/rm -rf -- "$TRANSACTION_DIRECTORY"
}
trap cleanup EXIT

require_regular_leaf() {
  local path="$1"
  local label="$2"
  "$PYTHON3" -I - "$path" "$label" <<'PY_LSTAT_REGULAR'
import os
from pathlib import Path
import stat
import sys

path = Path(sys.argv[1])
label = sys.argv[2]
try:
    entry = os.lstat(path)
except OSError as error:
    raise SystemExit(f"error: cannot inspect {label} {path}: {error}")
if stat.S_ISLNK(entry.st_mode):
    raise SystemExit(f"error: {label} must not be a symlink: {path}")
if not stat.S_ISREG(entry.st_mode):
    raise SystemExit(f"error: {label} must be a regular file: {path}")
PY_LSTAT_REGULAR
}

ensure_build_root() {
  if [[ -e "$BUILD_ROOT" || -L "$BUILD_ROOT" ]]; then
    "$PYTHON3" -I - "$BUILD_ROOT" <<'PY_LSTAT_DIRECTORY'
import os
from pathlib import Path
import stat
import sys

path = Path(sys.argv[1])
try:
    entry = os.lstat(path)
except OSError as error:
    raise SystemExit(f"error: cannot inspect build root {path}: {error}")
if stat.S_ISLNK(entry.st_mode) or not stat.S_ISDIR(entry.st_mode):
    raise SystemExit(f"error: build root must be a non-symlink directory: {path}")
PY_LSTAT_DIRECTORY
  else
    /bin/mkdir -m 700 "$BUILD_ROOT"
  fi
}

require_safe_destination() {
  [[ -e "$DESTINATION" || -L "$DESTINATION" ]] || return 0
  "$PYTHON3" -I - "$DESTINATION" <<'PY_LSTAT_DESTINATION'
import os
from pathlib import Path
import stat
import sys

path = Path(sys.argv[1])
try:
    entry = os.lstat(path)
except OSError as error:
    raise SystemExit(f"error: cannot inspect developer model destination {path}: {error}")
if stat.S_ISLNK(entry.st_mode) or not stat.S_ISDIR(entry.st_mode):
    raise SystemExit(
        f"error: developer model destination must be a non-symlink directory: {path}"
    )
PY_LSTAT_DESTINATION
}

validate_compiled_tree() {
  local model="$1"
  "$PYTHON3" -I - "$model" <<'PY_VALIDATE_COMPILED'
import os
from pathlib import Path
import stat
import sys

root = Path(sys.argv[1])
try:
    root_stat = os.lstat(root)
except OSError as error:
    raise SystemExit(f"error: verified compiled model is missing: {error}")
if stat.S_ISLNK(root_stat.st_mode) or not stat.S_ISDIR(root_stat.st_mode):
    raise SystemExit("error: verified compiled model root must be a non-symlink directory")

def walk_error(error: OSError) -> None:
    raise error

try:
    for current, directory_names, file_names in os.walk(
        root, topdown=True, onerror=walk_error, followlinks=False
    ):
        current_path = Path(current)
        for name in directory_names:
            entry_path = current_path / name
            entry = os.lstat(entry_path)
            if stat.S_ISLNK(entry.st_mode) or not stat.S_ISDIR(entry.st_mode):
                raise SystemExit(
                    f"error: compiled model contains non-directory entry: "
                    f"{entry_path.relative_to(root)}"
                )
        for name in file_names:
            entry_path = current_path / name
            entry = os.lstat(entry_path)
            if stat.S_ISLNK(entry.st_mode) or not stat.S_ISREG(entry.st_mode):
                raise SystemExit(
                    f"error: compiled model contains symlink or special file: "
                    f"{entry_path.relative_to(root)}"
                )
except OSError as error:
    raise SystemExit(f"error: cannot inspect verified compiled model: {error}")

probe = root / "coremldata.bin"
try:
    probe_stat = os.lstat(probe)
except OSError as error:
    raise SystemExit(f"error: compiled model coremldata.bin is missing: {error}")
if stat.S_ISLNK(probe_stat.st_mode) or not stat.S_ISREG(probe_stat.st_mode):
    raise SystemExit("error: compiled model coremldata.bin must be a regular file")
PY_VALIDATE_COMPILED
}

command="${1:-}"
case "$command" in
  on)
    if [[ "$#" -gt 3 ]]; then
      usage
      exit 1
    fi
    aar="${2:-$DEFAULT_AAR}"
    receipt="${3:-$aar.receipt.json}"
    if [[ "${aar##*/}" != "$AAR_FILE_NAME" ]]; then
      echo "error: AAR filename must be $AAR_FILE_NAME: $aar" >&2
      exit 1
    fi
    if [[ "${receipt##*/}" != "$RECEIPT_FILE_NAME" ]]; then
      echo "error: receipt filename must be $RECEIPT_FILE_NAME: $receipt" >&2
      exit 1
    fi

    # These lstat checks happen before the verifier can open either leaf. They
    # reject FIFOs immediately and reject symlinks rather than following them.
    require_regular_leaf "$aar" "asset-pack AAR"
    require_regular_leaf "$receipt" "asset-pack receipt"
    require_regular_leaf "$MANIFEST" "asset-pack manifest"
    require_regular_leaf "$LOCK_FILE" "model lock"
    require_regular_leaf "$LICENSE_FILE" "model license"
    require_regular_leaf "$VERIFIER" "model verifier"
    ensure_build_root
    require_safe_destination

    TRANSACTION_DIRECTORY="$(/usr/bin/mktemp -d "$BUILD_ROOT/.dev-bundle-model.XXXXXX")"
    /bin/chmod 700 "$TRANSACTION_DIRECTORY"
    input_snapshot="$TRANSACTION_DIRECTORY/inputs"
    extracted="$TRANSACTION_DIRECTORY/extracted"
    publish_candidate="$TRANSACTION_DIRECTORY/publish-candidate"
    snapshot_aar="$input_snapshot/$AAR_FILE_NAME"
    snapshot_receipt="$input_snapshot/$RECEIPT_FILE_NAME"
    /bin/mkdir -m 700 "$input_snapshot"

    echo "==> freeze canonical AAR + receipt into private transaction"
    "$PYTHON3" -I - \
      "$aar" "$snapshot_aar" \
      "$receipt" "$snapshot_receipt" <<'PY_FREEZE_INPUTS'
import hashlib
import os
from pathlib import Path
import stat
import sys


class FreezeError(RuntimeError):
    pass


def directory_identity(path: Path, label: str) -> tuple[int, int]:
    try:
        entry = os.lstat(path)
    except OSError as error:
        raise FreezeError(f"cannot inspect {label} {path}: {error}") from error
    if stat.S_ISLNK(entry.st_mode) or not stat.S_ISDIR(entry.st_mode):
        raise FreezeError(f"{label} must be a non-symlink directory: {path}")
    return entry.st_dev, entry.st_ino


def file_state(entry: os.stat_result) -> tuple[int, int, int, int, int, int]:
    return (
        entry.st_dev,
        entry.st_ino,
        stat.S_IFMT(entry.st_mode),
        entry.st_size,
        entry.st_mtime_ns,
        entry.st_ctime_ns,
    )


def open_regular_source(source: Path) -> tuple[int, os.stat_result]:
    try:
        expected = os.lstat(source)
    except OSError as error:
        raise FreezeError(f"cannot inspect source input {source}: {error}") from error
    if stat.S_ISLNK(expected.st_mode) or not stat.S_ISREG(expected.st_mode):
        raise FreezeError(f"source input must be a regular non-symlink file: {source}")

    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    # If a regular leaf is raced into a FIFO, opening must still fail closed
    # rather than blocking before fstat can reject it.
    flags |= getattr(os, "O_NONBLOCK", 0)
    try:
        descriptor = os.open(source, flags)
    except OSError as error:
        raise FreezeError(f"cannot open source input {source}: {error}") from error
    opened = os.fstat(descriptor)
    if not stat.S_ISREG(opened.st_mode):
        os.close(descriptor)
        raise FreezeError(f"opened source input is not a regular file: {source}")
    if (opened.st_dev, opened.st_ino) != (expected.st_dev, expected.st_ino):
        os.close(descriptor)
        raise FreezeError(f"source input changed while opening it: {source}")
    return descriptor, opened


def write_all(descriptor: int, data: bytes) -> None:
    view = memoryview(data)
    while view:
        written = os.write(descriptor, view)
        if written <= 0:
            raise FreezeError("snapshot write made no progress")
        view = view[written:]


def digest_regular_file(path: Path, expected_identity: tuple[int, int]) -> bytes:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    flags |= getattr(os, "O_NONBLOCK", 0)
    descriptor = os.open(path, flags)
    digest = hashlib.sha256()
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode):
            raise FreezeError(f"snapshot is not a regular file: {path}")
        if (opened.st_dev, opened.st_ino) != expected_identity:
            raise FreezeError(f"snapshot changed while reopening it: {path}")
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    finally:
        os.close(descriptor)
    return digest.digest()


def freeze_regular_file(source: Path, destination: Path) -> None:
    source_descriptor, source_before = open_regular_source(source)
    destination_flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    destination_flags |= getattr(os, "O_CLOEXEC", 0)
    destination_flags |= getattr(os, "O_NOFOLLOW", 0)
    source_digest = hashlib.sha256()
    try:
        destination_descriptor = os.open(destination, destination_flags, 0o600)
    except Exception:
        os.close(source_descriptor)
        raise

    try:
        while True:
            chunk = os.read(source_descriptor, 1024 * 1024)
            if not chunk:
                break
            source_digest.update(chunk)
            write_all(destination_descriptor, chunk)
        os.fsync(destination_descriptor)

        source_after = os.fstat(source_descriptor)
        if file_state(source_after) != file_state(source_before):
            raise FreezeError(f"source input changed while copying it: {source}")
        try:
            source_path_after = os.lstat(source)
        except OSError as error:
            raise FreezeError(f"source input disappeared while copying: {source}") from error
        if file_state(source_path_after) != file_state(source_before):
            raise FreezeError(f"source input path changed while copying: {source}")

        destination_opened = os.fstat(destination_descriptor)
        if not stat.S_ISREG(destination_opened.st_mode):
            raise FreezeError(f"snapshot output is not a regular file: {destination}")
        destination_identity = (
            destination_opened.st_dev,
            destination_opened.st_ino,
        )
    finally:
        os.close(destination_descriptor)
        os.close(source_descriptor)

    try:
        destination_path = os.lstat(destination)
    except OSError as error:
        raise FreezeError(f"cannot inspect completed snapshot {destination}: {error}") from error
    if stat.S_ISLNK(destination_path.st_mode) or not stat.S_ISREG(destination_path.st_mode):
        raise FreezeError(f"completed snapshot is not a regular file: {destination}")
    if (destination_path.st_dev, destination_path.st_ino) != destination_identity:
        raise FreezeError(f"snapshot path changed after copying: {destination}")
    if digest_regular_file(destination, destination_identity) != source_digest.digest():
        raise FreezeError(f"snapshot digest mismatch after copying: {destination}")


arguments = [Path(value) for value in sys.argv[1:]]
if len(arguments) != 4:
    raise SystemExit("error: freeze helper requires two source/destination pairs")
pairs = [(arguments[0], arguments[1]), (arguments[2], arguments[3])]
parent_states: dict[Path, tuple[int, int]] = {}
try:
    for source, _ in pairs:
        parent = Path(os.path.abspath(source.parent))
        parent_states.setdefault(
            parent, directory_identity(parent, "source input parent")
        )
    for source, destination in pairs:
        freeze_regular_file(source, destination)
    for parent, expected in parent_states.items():
        if directory_identity(parent, "source input parent") != expected:
            raise FreezeError(f"source input parent changed while freezing pair: {parent}")

    snapshot_parent = pairs[0][1].parent
    if pairs[1][1].parent != snapshot_parent:
        raise FreezeError("snapshot pair must share one private parent")
    parent_descriptor = os.open(
        snapshot_parent,
        os.O_RDONLY
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0),
    )
    try:
        os.fsync(parent_descriptor)
    finally:
        os.close(parent_descriptor)
except (OSError, FreezeError) as error:
    print(f"error: cannot freeze asset-pack input pair: {error}", file=sys.stderr)
    raise SystemExit(1)
PY_FREEZE_INPUTS

    echo "==> verify receipt + extract frozen private AAR snapshot"
    "$PYTHON3" -I "$VERIFIER" receipt \
      --lock "$LOCK_FILE" \
      --license "$LICENSE_FILE" \
      --manifest "$MANIFEST" \
      --extract-directory "$extracted" \
      "$snapshot_aar" "$snapshot_receipt"

    compiled_model="$extracted/Contents/$MODEL_FILE_NAME"
    validate_compiled_tree "$compiled_model"
    /bin/mkdir -m 700 "$publish_candidate"
    /bin/mv -- "$compiled_model" "$publish_candidate/$MODEL_FILE_NAME"

    echo "==> atomically publish verified compiled model"
    # Arm preservation before entering the publisher. A signal or other exit
    # that bypasses Python's classified status codes must never let the shell's
    # EXIT trap erase a displaced old model. Success, and the publisher's
    # private status 10 after either a pre-namespace failure or a proven
    # rollback, explicitly disarm it. Every unclassified status stays armed.
    PRESERVE_TRANSACTION=1
    if "$PYTHON3" -I - "$publish_candidate" "$DESTINATION" <<'PY_ATOMIC_PUBLISH'
import ctypes
import errno
import os
from pathlib import Path
import stat
import sys


AT_FDCWD = -2
RENAME_SWAP = 0x00000002
RENAME_EXCL = 0x00000004

class PublishError(RuntimeError):
    pass


class RecoveryRequired(RuntimeError):
    pass


def renameatx(source: Path, destination: Path, flags: int) -> None:
    library = ctypes.CDLL(None, use_errno=True)
    function = library.renameatx_np
    function.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    ]
    function.restype = ctypes.c_int
    result = function(
        AT_FDCWD,
        os.fsencode(source),
        AT_FDCWD,
        os.fsencode(destination),
        flags,
    )
    if result != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))


def rename_swap(source: Path, destination: Path) -> None:
    renameatx(source, destination, RENAME_SWAP)


def rename_exclusive(source: Path, destination: Path) -> None:
    renameatx(source, destination, RENAME_EXCL)


def directory_identity(path: Path, label: str) -> tuple[int, int]:
    try:
        entry = os.lstat(path)
    except OSError as error:
        raise PublishError(f"cannot inspect {label} {path}: {error}") from error
    if stat.S_ISLNK(entry.st_mode) or not stat.S_ISDIR(entry.st_mode):
        raise PublishError(f"{label} must be a non-symlink directory: {path}")
    return entry.st_dev, entry.st_ino


def fsync_directory(path: Path) -> None:
    expected = directory_identity(path, "directory to fsync")
    flags = os.O_RDONLY
    flags |= getattr(os, "O_DIRECTORY", 0)
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        opened = os.fstat(descriptor)
        if (opened.st_dev, opened.st_ino) != expected:
            raise PublishError(f"directory changed while opening it for fsync: {path}")
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def fsync_regular_file(path: Path) -> None:
    try:
        expected = os.lstat(path)
    except OSError as error:
        raise PublishError(f"cannot inspect candidate file {path}: {error}") from error
    if stat.S_ISLNK(expected.st_mode) or not stat.S_ISREG(expected.st_mode):
        raise PublishError(f"candidate contains a symlink or special file: {path}")
    flags = os.O_RDONLY
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    flags |= getattr(os, "O_NONBLOCK", 0)
    descriptor = os.open(path, flags)
    try:
        opened = os.fstat(descriptor)
        if (opened.st_dev, opened.st_ino) != (expected.st_dev, expected.st_ino):
            raise PublishError(f"candidate file changed while opening it: {path}")
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def fsync_tree(root: Path) -> None:
    directory_identity(root, "candidate tree")
    directories: list[Path] = []

    def walk_error(error: OSError) -> None:
        raise error

    try:
        for current, directory_names, file_names in os.walk(
            root, topdown=True, onerror=walk_error, followlinks=False
        ):
            current_path = Path(current)
            directories.append(current_path)
            for name in directory_names:
                directory_identity(current_path / name, "candidate subdirectory")
            for name in file_names:
                fsync_regular_file(current_path / name)
    except OSError as error:
        raise PublishError(f"cannot synchronize candidate tree {root}: {error}") from error

    for directory in reversed(directories):
        fsync_directory(directory)


def rollback_or_require_recovery(
    candidate: Path, destination: Path, had_destination: bool, cause: Exception
) -> None:
    try:
        if had_destination:
            rename_swap(candidate, destination)
        else:
            rename_exclusive(destination, candidate)
        fsync_directory(candidate.parent)
        fsync_directory(destination.parent)
    except Exception as rollback_error:
        raise RecoveryRequired(
            f"publish failed ({cause}); rollback could not be proven ({rollback_error})"
        ) from rollback_error
    raise PublishError(f"publish durability failed; previous output restored: {cause}")


def publish(candidate: Path, destination: Path) -> None:
    candidate_parent = directory_identity(candidate.parent, "candidate parent")
    destination_parent = directory_identity(destination.parent, "destination parent")
    if candidate_parent[0] != destination_parent[0]:
        raise PublishError("candidate and destination must be on the same filesystem")
    candidate_identity = directory_identity(candidate, "publish candidate")
    # Durability boundary: persist every candidate file and directory before
    # the one namespace operation that makes it live, then persist both parents
    # changed by the cross-directory exclusive rename or swap.
    fsync_tree(candidate)
    fsync_directory(candidate.parent)

    try:
        previous_identity = directory_identity(destination, "existing destination")
    except PublishError:
        try:
            os.lstat(destination)
        except FileNotFoundError:
            previous_identity = None
        except OSError as error:
            raise PublishError(f"cannot inspect destination {destination}: {error}") from error
        else:
            raise

    if previous_identity is None:
        try:
            rename_exclusive(candidate, destination)
        except OSError as error:
            if error.errno == errno.EEXIST:
                raise PublishError("destination appeared during first publication") from error
            raise PublishError(f"atomic first publication failed: {error}") from error
    else:
        try:
            rename_swap(candidate, destination)
        except OSError as error:
            raise PublishError(f"atomic directory swap failed: {error}") from error

    # From the first successful namespace operation onward, every validation
    # and persistence error must cross the same recovery boundary. In a
    # replacement, `candidate` now names the displaced previous output; letting
    # an identity-inspection exception escape would allow EXIT cleanup to erase
    # the only recoverable old model without first swapping it back.
    try:
        if (
            previous_identity is not None
            and directory_identity(candidate, "swapped previous destination")
            != previous_identity
        ):
            raise PublishError("destination changed during atomic swap")
        if directory_identity(destination, "published destination") != candidate_identity:
            raise PublishError("published destination identity mismatch")
        fsync_directory(candidate.parent)
        fsync_directory(destination.parent)
    except Exception as error:
        rollback_or_require_recovery(
            candidate, destination, previous_identity is not None, error
        )


try:
    publish(Path(sys.argv[1]), Path(sys.argv[2]))
except RecoveryRequired as error:
    print(f"error: developer model publication recovery required: {error}", file=sys.stderr)
    raise SystemExit(2)
except (OSError, PublishError) as error:
    print(f"error: developer model publication failed: {error}", file=sys.stderr)
    # Private shell protocol: this is the only nonzero status that proves no
    # displaced output needs preservation. The shell translates it to the
    # public failure status 1 after disarming cleanup protection.
    raise SystemExit(10)
except BaseException as error:
    print(
        f"error: unexpected developer model publisher exit; recovery required: {error}",
        file=sys.stderr,
    )
    raise SystemExit(2)
PY_ATOMIC_PUBLISH
    then
      PRESERVE_TRANSACTION=0
    else
      publish_status=$?
      if [[ "$publish_status" -eq 10 ]]; then
        PRESERVE_TRANSACTION=0
        exit 1
      fi
      exit "$publish_status"
    fi

    echo "DEV_BUNDLE_MODEL_PASS"
    echo "bundled verified $MODEL_FILE_NAME at $DESTINATION"
    ;;
  off)
    if [[ "$#" -ne 1 ]]; then
      usage
      exit 1
    fi
    if [[ ! -e "$BUILD_ROOT" && ! -L "$BUILD_ROOT" ]]; then
      echo "developer model bundle already disabled"
      exit 0
    fi
    ensure_build_root
    if [[ ! -e "$DESTINATION" && ! -L "$DESTINATION" ]]; then
      echo "developer model bundle already disabled"
      exit 0
    fi
    require_safe_destination
    /bin/rm -rf -- "$DESTINATION"
    echo "developer model bundle disabled"
    ;;
  *)
    usage
    exit 1
    ;;
esac
