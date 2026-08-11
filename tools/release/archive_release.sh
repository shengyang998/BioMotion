#!/bin/bash -p
# Create a signed Release archive and bind it to the reviewed native inputs.
case "$-" in
  *p*) ;;
  *)
    printf '%s\n' \
      'error: execute archive_release.sh directly or with /bin/bash -p' >&2
    exit 78
    ;;
esac
set -euo pipefail

PATH="/usr/bin:/bin:/usr/sbin:/sbin"
export PATH
unset \
  ASC_API_ISSUER ASC_API_KEY_ID BASH_ENV CDPATH DEVELOPER_DIR \
  DYLD_INSERT_LIBRARIES DYLD_LIBRARY_PATH ENV GIT_CONFIG_GLOBAL \
  GIT_CONFIG_SYSTEM GREP_OPTIONS PERL5LIB PERL5OPT PYTHONHOME \
  PYTHONINSPECT PYTHONPATH PYTHONSTARTUP PYTHONWARNINGS SDKROOT \
  TOOLCHAINS XCODE_XCCONFIG_FILE

BASH_TOOL="/bin/bash"
CAT="/bin/cat"
CHMOD="/bin/chmod"
ENV_TOOL="/usr/bin/env"
ID_TOOL="/usr/bin/id"
MKDIR="/bin/mkdir"
MKTEMP="/usr/bin/mktemp"
PYTHON="/usr/bin/python3"
RM="/bin/rm"
RMDIR="/bin/rmdir"
SHASUM="/usr/bin/shasum"
STAT="/usr/bin/stat"
XCODEBUILD="/usr/bin/xcodebuild"
HERMETIC_HOME="/var/empty"

usage() {
  "$CAT" <<'EOF'
Usage:
  /bin/bash -p tools/release/archive_release.sh \
    --archive NEW_PATH/BioMotion.xcarchive

Creates a new signed Release archive and its adjacent dependency receipt. The
archive and receipt paths must not already exist. Provisioning updates are
never authorized by this wrapper.
EOF
}

usage_error() {
  printf 'error: %s\n' "$*" >&2
  usage >&2
  exit 64
}

strip_trailing_slashes() {
  local path="$1"
  while [[ "$path" != "/" && "$path" == */ ]]; do
    path="${path%/}"
  done
  printf '%s\n' "$path"
}

path_directory() {
  local path="$1"
  case "$path" in
    */*)
      path="${path%/*}"
      [[ -n "$path" ]] || path="/"
      ;;
    *) path="." ;;
  esac
  printf '%s\n' "$path"
}

require_regular_file() {
  local path="$1"
  local label="$2"
  if [[ -L "$path" || ! -f "$path" ]]; then
    printf 'error: %s must be a non-symlink regular file: %s\n' \
      "$label" "$path" >&2
    exit 1
  fi
}

require_executable_file() {
  local path="$1"
  local label="$2"
  require_regular_file "$path" "$label"
  if [[ ! -x "$path" ]]; then
    printf 'error: %s must be executable: %s\n' "$label" "$path" >&2
    exit 1
  fi
}

object_identity() {
  "$STAT" -f '%d:%i' "$1"
}

sha256_file() {
  local path="$1"
  local label="$2"
  local output
  local digest
  require_regular_file "$path" "$label"
  output="$(run_hermetic "$SHASUM" -a 256 "$path")"
  digest="${output%% *}"
  if [[ ! "$digest" =~ ^[0-9a-f]{64}$ ]]; then
    printf 'error: could not parse %s SHA-256: %s\n' "$label" "$path" >&2
    exit 1
  fi
  printf '%s\n' "$digest"
}

run_hermetic() {
  "$ENV_TOOL" -i \
    PATH="$PATH" \
    HOME="$HERMETIC_HOME" \
    LANG=C \
    LC_ALL=C \
    "$@"
}

run_xcodebuild() {
  "$ENV_TOOL" -i \
    PATH="$PATH" \
    HOME="$TRUSTED_HOME" \
    USER="$TRUSTED_USER" \
    LOGNAME="$TRUSTED_USER" \
    LANG=C \
    LC_ALL=C \
    "$@"
}

# Darwin's exclusive rename publishes one object atomically without replacing
# a path that appeared after the wrapper's earlier absence check.
rename_exclusive() {
  local source="$1"
  local destination="$2"
  run_hermetic "$PYTHON" -I - "$source" "$destination" <<'PY'
import ctypes
import os
import sys

if len(sys.argv) != 3:
    raise SystemExit(64)

source = os.fsencode(sys.argv[1])
destination = os.fsencode(sys.argv[2])
libc = ctypes.CDLL(None, use_errno=True)
renameatx_np = libc.renameatx_np
renameatx_np.argtypes = [
    ctypes.c_int,
    ctypes.c_char_p,
    ctypes.c_int,
    ctypes.c_char_p,
    ctypes.c_uint,
]
renameatx_np.restype = ctypes.c_int

AT_FDCWD = -2
RENAME_EXCL = 0x00000004
if renameatx_np(
    AT_FDCWD, source, AT_FDCWD, destination, RENAME_EXCL
) != 0:
    error_number = ctypes.get_errno()
    raise OSError(
        error_number,
        os.strerror(error_number),
        os.fsdecode(destination),
    )
PY
}

ARCHIVE_SEEN=0
ARCHIVE_INPUT=""

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --archive)
      [[ "$ARCHIVE_SEEN" -eq 0 ]] || \
        usage_error "--archive may be provided only once"
      [[ "$#" -ge 2 && -n "$2" ]] || \
        usage_error "--archive requires a path"
      ARCHIVE_INPUT="$2"
      ARCHIVE_SEEN=1
      shift 2
      ;;
    *) usage_error "only --archive NEW_PATH.xcarchive is accepted" ;;
  esac
done

[[ "$ARCHIVE_SEEN" -eq 1 ]] || usage_error "--archive is required"

ARCHIVE_INPUT="$(strip_trailing_slashes "$ARCHIVE_INPUT")"
case "$ARCHIVE_INPUT" in
  ""|"/"|"."|".."|*/.|*/..)
    usage_error "unsafe --archive path"
    ;;
esac
ARCHIVE_LEAF="${ARCHIVE_INPUT##*/}"
case "$ARCHIVE_LEAF" in
  ?*.xcarchive) ;;
  *) usage_error "--archive must name a .xcarchive directory" ;;
esac

ARCHIVE_PARENT_INPUT="$(path_directory "$ARCHIVE_INPUT")"
if [[ -L "$ARCHIVE_PARENT_INPUT" || ! -d "$ARCHIVE_PARENT_INPUT" ]]; then
  printf 'error: archive parent must be an existing non-symlink directory: %s\n' \
    "$ARCHIVE_PARENT_INPUT" >&2
  exit 1
fi

if [[ -L "$HERMETIC_HOME" || ! -d "$HERMETIC_HOME" || \
  "$("$STAT" -f '%u' "$HERMETIC_HOME")" != "0" ]]; then
  printf 'error: hermetic HOME must be a root-owned physical directory: %s\n' \
    "$HERMETIC_HOME" >&2
  exit 1
fi
HERMETIC_HOME_MODE="$("$STAT" -f '%Lp' "$HERMETIC_HOME")"
if [[ ! "$HERMETIC_HOME_MODE" =~ ^[0-7]{3,4}$ ]] || \
  (( (8#$HERMETIC_HOME_MODE & 0022) != 0 )); then
  printf 'error: hermetic HOME must not be group- or world-writable: %s\n' \
    "$HERMETIC_HOME" >&2
  exit 1
fi
HERMETIC_HOME_PHYSICAL="$(cd -P -- "$HERMETIC_HOME" && pwd -P)"
if [[ "$HERMETIC_HOME_PHYSICAL" == "/" || \
  ! -d "$HERMETIC_HOME_PHYSICAL" ]]; then
  printf 'error: could not resolve hermetic HOME safely: %s\n' \
    "$HERMETIC_HOME" >&2
  exit 1
fi

# Comparing lexical and physical paths rejects a symlink in any parent
# component while still accepting an ordinary relative path.
ARCHIVE_PARENT_LEXICAL="$(
  run_hermetic "$PYTHON" -I -c \
    'import os, sys; print(os.path.abspath(sys.argv[1]))' \
    "$ARCHIVE_PARENT_INPUT"
)"
ARCHIVE_PARENT="$(cd -P -- "$ARCHIVE_PARENT_INPUT" && pwd -P)"
if [[ "$ARCHIVE_PARENT_LEXICAL" != "$ARCHIVE_PARENT" || \
  "$ARCHIVE_PARENT" == "/" ]]; then
  printf 'error: archive parent is not a safe physical directory: %s\n' \
    "$ARCHIVE_PARENT_INPUT" >&2
  exit 1
fi

CURRENT_UID="$("$ID_TOOL" -u)"
TRUSTED_USER="$("$ID_TOOL" -un)"
TRUSTED_HOME="$(
  run_hermetic "$PYTHON" -I -c \
    'import os, pwd; print(pwd.getpwuid(os.getuid()).pw_dir)'
)"
if [[ "$TRUSTED_HOME" != /* || ! -d "$TRUSTED_HOME" ]]; then
  printf 'error: could not resolve the current user home directory\n' >&2
  exit 1
fi
if [[ "$("$STAT" -f '%u' "$ARCHIVE_PARENT")" != "$CURRENT_UID" ]]; then
  printf 'error: archive parent must be owned by the current user: %s\n' \
    "$ARCHIVE_PARENT" >&2
  exit 1
fi
PARENT_MODE="$("$STAT" -f '%Lp' "$ARCHIVE_PARENT")"
if [[ ! "$PARENT_MODE" =~ ^[0-7]{3,4}$ ]] || \
  (( (8#$PARENT_MODE & 0022) != 0 )); then
  printf 'error: archive parent must not be group- or world-writable: %s\n' \
    "$ARCHIVE_PARENT" >&2
  exit 1
fi
ARCHIVE_PARENT_IDENTITY="$(object_identity "$ARCHIVE_PARENT")"

ARCHIVE="$ARCHIVE_PARENT/$ARCHIVE_LEAF"
RECEIPT="$ARCHIVE.dependency-receipt.json"
if [[ -e "$ARCHIVE" || -L "$ARCHIVE" ]]; then
  printf 'error: archive path already exists: %s\n' "$ARCHIVE" >&2
  exit 1
fi
if [[ -e "$RECEIPT" || -L "$RECEIPT" ]]; then
  printf 'error: dependency receipt path already exists: %s\n' \
    "$RECEIPT" >&2
  exit 1
fi

SOURCE_PATH="${BASH_SOURCE[0]}"
if [[ -L "$SOURCE_PATH" ]]; then
  printf 'error: archive wrapper must not be invoked through a symlink\n' >&2
  exit 1
fi
case "$SOURCE_PATH" in
  */*) SCRIPT_DIRECTORY="${SOURCE_PATH%/*}" ;;
  *) SCRIPT_DIRECTORY="." ;;
esac
REPO_ROOT="$(cd -P -- "$SCRIPT_DIRECTORY/../.." && pwd -P)"
DEPENDENCY_LOCK="$REPO_ROOT/tools/dependencies.lock.json"
DEPENDENCY_GATE="$REPO_ROOT/tools/tests/dependency_boundary_probe.sh"
RESOURCE_GATE="$REPO_ROOT/tools/tests/app_resource_boundary_probe.sh"
PRIVACY_GATE="$REPO_ROOT/tools/tests/privacy_manifest_probe.sh"
RECEIPT_INSPECTOR="$REPO_ROOT/tools/release/dependency_archive_receipt.py"

require_executable_file "$XCODEBUILD" "xcodebuild"
require_regular_file "$DEPENDENCY_LOCK" "dependency lock"
require_regular_file "$DEPENDENCY_GATE" "dependency-boundary gate"
require_regular_file "$RESOURCE_GATE" "resource-boundary gate"
require_regular_file "$PRIVACY_GATE" "privacy-manifest gate"
require_regular_file "$RECEIPT_INSPECTOR" "dependency-receipt inspector"

STAGING_ROOT=""
STAGING_ROOT_IDENTITY=""
STAGING_ARCHIVE_IDENTITY=""
STAGING_RECEIPT_IDENTITY=""
STAGING_VERIFIED=0
ARCHIVE_PUBLISHED=0
RECEIPT_PUBLISHED=0
PUBLISHED_ARCHIVE_IDENTITY=""
PUBLISHED_RECEIPT_IDENTITY=""
COMPLETED=0
DEPENDENCY_SNAPSHOT=""
DEPENDENCY_LOCK_SHA256=""
DERIVED_DATA=""
DERIVED_DATA_IDENTITY=""
DERIVED_DATA_ACTIVE=0

validate_archive_parent() {
  local mode
  if [[ -L "$ARCHIVE_PARENT" || ! -d "$ARCHIVE_PARENT" || \
    "$(object_identity "$ARCHIVE_PARENT" 2>/dev/null || true)" != \
      "$ARCHIVE_PARENT_IDENTITY" || \
    "$("$STAT" -f '%u' "$ARCHIVE_PARENT" 2>/dev/null || true)" != \
      "$CURRENT_UID" ]]; then
    printf 'error: archive parent identity changed during the release\n' >&2
    return 1
  fi
  mode="$("$STAT" -f '%Lp' "$ARCHIVE_PARENT" 2>/dev/null || true)"
  if [[ ! "$mode" =~ ^[0-7]{3,4}$ ]] || \
    (( (8#$mode & 0022) != 0 )); then
    printf 'error: archive parent permissions changed during the release\n' >&2
    return 1
  fi
}

validate_staging_context() {
  validate_archive_parent || return 1
  if [[ "$STAGING_ROOT" != "$ARCHIVE_PARENT"/.biomotion-archive.* || \
    -L "$STAGING_ROOT" || ! -d "$STAGING_ROOT" || \
    "$(object_identity "$STAGING_ROOT" 2>/dev/null || true)" != \
      "$STAGING_ROOT_IDENTITY" || \
    "$("$STAT" -f '%u' "$STAGING_ROOT" 2>/dev/null || true)" != \
      "$CURRENT_UID" || \
    "$("$STAT" -f '%Lp' "$STAGING_ROOT" 2>/dev/null || true)" != \
      "700" || \
    "$(cd -P -- "$(path_directory "$STAGING_ROOT")" 2>/dev/null && \
      pwd -P)" != "$ARCHIVE_PARENT" ]]; then
    printf 'error: staging directory identity changed during the release\n' >&2
    return 1
  fi
}

validate_staged_pair() {
  validate_staged_archive || return 1
  if [[ \
    "$("$STAT" -f '%u' "$STAGING_ARCHIVE" 2>/dev/null || true)" != \
      "$CURRENT_UID" || \
    -L "$STAGING_RECEIPT" || ! -f "$STAGING_RECEIPT" || \
    "$(object_identity "$STAGING_RECEIPT" 2>/dev/null || true)" != \
      "$STAGING_RECEIPT_IDENTITY" || \
    "$("$STAT" -f '%u' "$STAGING_RECEIPT" 2>/dev/null || true)" != \
      "$CURRENT_UID" || \
    "$("$STAT" -f '%Lp' "$STAGING_RECEIPT" 2>/dev/null || true)" != \
      "600" ]]; then
    printf 'error: staged archive or receipt identity changed before publication\n' \
      >&2
    return 1
  fi
}

validate_staged_archive() {
  validate_staging_context || return 1
  if [[ -L "$STAGING_ARCHIVE" || ! -d "$STAGING_ARCHIVE" || \
    "$(object_identity "$STAGING_ARCHIVE" 2>/dev/null || true)" != \
      "$STAGING_ARCHIVE_IDENTITY" || \
    "$("$STAT" -f '%u' "$STAGING_ARCHIVE" 2>/dev/null || true)" != \
      "$CURRENT_UID" ]]; then
    printf 'error: staged archive identity changed after xcodebuild\n' >&2
    return 1
  fi
}

validate_derived_data() {
  validate_staging_context || return 1
  if [[ "$DERIVED_DATA_ACTIVE" -ne 1 || \
    "$DERIVED_DATA" != "$STAGING_ROOT/DerivedData" || \
    -L "$DERIVED_DATA" || ! -d "$DERIVED_DATA" || \
    "$(object_identity "$DERIVED_DATA" 2>/dev/null || true)" != \
      "$DERIVED_DATA_IDENTITY" || \
    "$("$STAT" -f '%u' "$DERIVED_DATA" 2>/dev/null || true)" != \
      "$CURRENT_UID" || \
    "$("$STAT" -f '%Lp' "$DERIVED_DATA" 2>/dev/null || true)" != \
      "700" || \
    "$(cd -P -- "$(path_directory "$DERIVED_DATA")" 2>/dev/null && \
      pwd -P)" != "$STAGING_ROOT" ]]; then
    printf 'error: private DerivedData identity changed during the release\n' >&2
    return 1
  fi
}

validate_staging_contents() {
  validate_staged_pair || return 1
  run_hermetic "$PYTHON" -I - \
    "$STAGING_ROOT" "$ARCHIVE_LEAF" \
    "$ARCHIVE_LEAF.dependency-receipt.json" <<'PY'
import os
from pathlib import Path
import sys

if len(sys.argv) != 4:
    raise SystemExit(64)
root = Path(sys.argv[1])
expected = {sys.argv[2], sys.argv[3]}
if root.is_symlink() or not root.is_dir():
    raise SystemExit("unsafe staging root")
actual = {entry.name for entry in os.scandir(root)}
if actual != expected:
    raise SystemExit(
        f"unexpected staging entries: {','.join(sorted(actual - expected))}"
    )
PY
}

capture_dependency_snapshot() {
  local lock_before
  local lock_after
  local probe_output
  local validated_output
  lock_before="$(sha256_file "$DEPENDENCY_LOCK" "dependency lock")"
  probe_output="$(
    run_hermetic "$BASH_TOOL" -p "$DEPENDENCY_GATE" --snapshot
  )"
  lock_after="$(sha256_file "$DEPENDENCY_LOCK" "dependency lock")"
  if [[ "$lock_before" != "$lock_after" ]]; then
    printf 'error: dependency lock changed during its boundary probe\n' >&2
    return 1
  fi
  validated_output="$(
    printf '%s\n' "$probe_output" | \
      run_hermetic "$PYTHON" -I "$RECEIPT_INSPECTOR" \
        validate-snapshot "$REPO_ROOT" "$lock_before"
  )"
  printf '%s\n' 'DEPENDENCY_OBSERVED_SNAPSHOT_PASS'
  DEPENDENCY_LOCK_SHA256="$lock_before"
  DEPENDENCY_SNAPSHOT="$validated_output"
}

require_initial_dependency_lock() {
  local current
  current="$(sha256_file "$DEPENDENCY_LOCK" "dependency lock")"
  if [[ "$current" != "$INITIAL_DEPENDENCY_LOCK_SHA256" ]]; then
    printf 'error: dependency lock changed after the archive build\n' >&2
    return 1
  fi
}

rollback_published_file() {
  local published="$1"
  local staged="$2"
  local expected_identity="$3"
  local label="$4"
  if [[ -L "$published" || ! -f "$published" || \
    "$(object_identity "$published" 2>/dev/null || true)" != \
      "$expected_identity" || -e "$staged" || -L "$staged" ]]; then
    printf 'error: could not safely roll back published %s: %s\n' \
      "$label" "$published" >&2
    return 1
  fi
  rename_exclusive "$published" "$staged"
}

rollback_published_directory() {
  local published="$1"
  local staged="$2"
  local expected_identity="$3"
  local label="$4"
  if [[ -L "$published" || ! -d "$published" || \
    "$(object_identity "$published" 2>/dev/null || true)" != \
      "$expected_identity" || -e "$staged" || -L "$staged" ]]; then
    printf 'error: could not safely roll back published %s: %s\n' \
      "$label" "$published" >&2
    return 1
  fi
  rename_exclusive "$published" "$staged"
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP

  if [[ "$COMPLETED" -eq 0 && "$STAGING_VERIFIED" -eq 1 ]]; then
    if validate_staging_context; then
      if [[ "$RECEIPT_PUBLISHED" -eq 1 ]]; then
        rollback_published_file \
          "$RECEIPT" "$STAGING_RECEIPT" \
          "$PUBLISHED_RECEIPT_IDENTITY" "receipt" || status=1
      fi
      if [[ "$ARCHIVE_PUBLISHED" -eq 1 ]]; then
        rollback_published_directory \
          "$ARCHIVE" "$STAGING_ARCHIVE" \
          "$PUBLISHED_ARCHIVE_IDENTITY" "archive" || status=1
      fi
    else
      status=1
    fi
  fi

  if [[ "$STAGING_VERIFIED" -eq 1 ]]; then
    if validate_staging_context; then
      "$RM" -rf "$STAGING_ROOT"
    else
      printf 'error: refusing unsafe staging cleanup: %s\n' \
        "$STAGING_ROOT" >&2
      status=1
    fi
  fi
  exit "$status"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

umask 077
STAGING_ROOT="$(
  "$MKTEMP" -d "$ARCHIVE_PARENT/.biomotion-archive.XXXXXX"
)"
if [[ "$STAGING_ROOT" != "$ARCHIVE_PARENT"/.biomotion-archive.* || \
  -L "$STAGING_ROOT" || ! -d "$STAGING_ROOT" || \
  "$(cd -P -- "$(path_directory "$STAGING_ROOT")" && pwd -P)" != \
    "$ARCHIVE_PARENT" || \
  "$("$STAT" -f '%u' "$STAGING_ROOT")" != "$CURRENT_UID" ]]; then
  printf 'error: mktemp returned an unsafe staging directory: %s\n' \
    "$STAGING_ROOT" >&2
  exit 1
fi
"$CHMOD" 0700 "$STAGING_ROOT"
if [[ "$("$STAT" -f '%Lp' "$STAGING_ROOT")" != "700" ]]; then
  printf 'error: staging directory is not mode 0700: %s\n' \
    "$STAGING_ROOT" >&2
  exit 1
fi
STAGING_ROOT_IDENTITY="$(object_identity "$STAGING_ROOT")"
STAGING_VERIFIED=1
STAGING_ARCHIVE="$STAGING_ROOT/$ARCHIVE_LEAF"
STAGING_RECEIPT="$STAGING_ARCHIVE.dependency-receipt.json"
validate_staging_context

printf '%s\n' '==> checking locked native dependencies before archive'
capture_dependency_snapshot
INITIAL_DEPENDENCY_SNAPSHOT="$DEPENDENCY_SNAPSHOT"
INITIAL_DEPENDENCY_LOCK_SHA256="$DEPENDENCY_LOCK_SHA256"
validate_staging_context

printf '%s\n' '==> checking reviewed source and generated project'
run_hermetic "$BASH_TOOL" -p "$RESOURCE_GATE"
validate_staging_context

DERIVED_DATA="$STAGING_ROOT/DerivedData"
"$MKDIR" "$DERIVED_DATA"
"$CHMOD" 0700 "$DERIVED_DATA"
DERIVED_DATA_IDENTITY="$(object_identity "$DERIVED_DATA")"
DERIVED_DATA_ACTIVE=1
validate_derived_data

printf '%s\n' '==> creating signed Release archive in private staging'
(
  cd "$REPO_ROOT"
  run_xcodebuild \
    "$XCODEBUILD" \
    -project "$REPO_ROOT/BioMotion.xcodeproj" \
    -scheme BioMotion \
    -configuration Release \
    -destination 'generic/platform=iOS' \
    -archivePath "$STAGING_ARCHIVE" \
    -derivedDataPath "$DERIVED_DATA" \
    archive
)
validate_staging_context
validate_derived_data

if [[ -L "$STAGING_ARCHIVE" || ! -d "$STAGING_ARCHIVE" || \
  "$("$STAT" -f '%u' "$STAGING_ARCHIVE")" != "$CURRENT_UID" ]]; then
  printf 'error: xcodebuild did not create a safe archive directory: %s\n' \
    "$STAGING_ARCHIVE" >&2
  exit 1
fi
STAGING_ARCHIVE_IDENTITY="$(object_identity "$STAGING_ARCHIVE")"

printf '%s\n' '==> rechecking locked native dependencies after archive'
capture_dependency_snapshot
if [[ "$DEPENDENCY_LOCK_SHA256" != "$INITIAL_DEPENDENCY_LOCK_SHA256" || \
  "$DEPENDENCY_SNAPSHOT" != "$INITIAL_DEPENDENCY_SNAPSHOT" ]]; then
  printf 'error: dependency boundary changed while the archive was built\n' >&2
  exit 1
fi
validate_staged_archive
validate_derived_data

printf '%s\n' '==> sealing dependency receipt before archive inspection'
printf '%s\n' "$INITIAL_DEPENDENCY_SNAPSHOT" | \
  run_hermetic \
    "$PYTHON" -I "$RECEIPT_INSPECTOR" \
    seal "$REPO_ROOT" "$STAGING_ARCHIVE" "$STAGING_RECEIPT" \
    "$INITIAL_DEPENDENCY_LOCK_SHA256"
validate_staged_archive
validate_derived_data
require_initial_dependency_lock
if [[ -L "$STAGING_RECEIPT" || ! -f "$STAGING_RECEIPT" || \
  "$("$STAT" -f '%u' "$STAGING_RECEIPT")" != "$CURRENT_UID" || \
  "$("$STAT" -f '%Lp' "$STAGING_RECEIPT")" != "600" ]]; then
  printf 'error: receipt sealer did not create a private regular file: %s\n' \
    "$STAGING_RECEIPT" >&2
  exit 1
fi
STAGING_RECEIPT_IDENTITY="$(object_identity "$STAGING_RECEIPT")"
validate_staged_pair

printf '%s\n' '==> checking signed release archive resource boundary'
run_hermetic \
  "$BASH_TOOL" -p "$RESOURCE_GATE" --release-archive "$STAGING_ARCHIVE"
validate_staged_pair

printf '%s\n' '==> checking signed release archive privacy boundary'
run_hermetic \
  "$BASH_TOOL" -p "$PRIVACY_GATE" \
  "$STAGING_ARCHIVE/Products/Applications/BioMotion.app"
validate_staged_pair

printf '%s\n' '==> verifying staged archive dependency receipt'
run_hermetic \
  "$PYTHON" -I "$RECEIPT_INSPECTOR" \
  verify "$REPO_ROOT" "$STAGING_ARCHIVE" "$STAGING_RECEIPT"
validate_staged_pair
require_initial_dependency_lock

printf '%s\n' '==> removing private DerivedData before publication'
validate_derived_data
"$RM" -rf "$DERIVED_DATA"
if [[ -e "$DERIVED_DATA" || -L "$DERIVED_DATA" ]]; then
  printf 'error: private DerivedData remained after cleanup: %s\n' \
    "$DERIVED_DATA" >&2
  exit 1
fi
DERIVED_DATA_ACTIVE=0
validate_staging_contents

# The parent was safe at entry, but recheck both destination names immediately
# before exclusive publication so an unexpected concurrent file is never used.
validate_staged_pair
if [[ -e "$ARCHIVE" || -L "$ARCHIVE" || \
  -e "$RECEIPT" || -L "$RECEIPT" ]]; then
  printf 'error: archive or receipt destination appeared during the build\n' >&2
  exit 1
fi

PUBLISHED_ARCHIVE_IDENTITY="$STAGING_ARCHIVE_IDENTITY"
PUBLISHED_RECEIPT_IDENTITY="$STAGING_RECEIPT_IDENTITY"
ARCHIVE_PUBLISHED=1
rename_exclusive "$STAGING_ARCHIVE" "$ARCHIVE"
validate_staging_context
if [[ -L "$ARCHIVE" || ! -d "$ARCHIVE" || \
  "$(object_identity "$ARCHIVE")" != "$PUBLISHED_ARCHIVE_IDENTITY" ]]; then
  printf 'error: published archive identity changed unexpectedly\n' >&2
  exit 1
fi

RECEIPT_PUBLISHED=1
rename_exclusive "$STAGING_RECEIPT" "$RECEIPT"
validate_staging_context
if [[ -L "$RECEIPT" || ! -f "$RECEIPT" || \
  "$(object_identity "$RECEIPT")" != "$PUBLISHED_RECEIPT_IDENTITY" ]]; then
  printf 'error: published receipt identity changed unexpectedly\n' >&2
  exit 1
fi

printf '%s\n' '==> verifying published archive dependency receipt'
run_hermetic \
  "$PYTHON" -I "$RECEIPT_INSPECTOR" \
  verify "$REPO_ROOT" "$ARCHIVE" "$RECEIPT"
validate_staging_context
require_initial_dependency_lock

# Keep signal handlers from observing the intentional gap between removal of
# the now-empty staging directory and completion of both success markers.
trap '' INT TERM HUP
if ! "$RMDIR" "$STAGING_ROOT"; then
  trap 'exit 130' INT
  trap 'exit 143' TERM
  trap 'exit 129' HUP
  printf 'error: private staging directory was not empty at completion\n' >&2
  exit 1
fi
STAGING_VERIFIED=0
COMPLETED=1
printf 'ARCHIVE_RELEASE_PASS %s\n' "$ARCHIVE"
printf 'DEPENDENCY_RECEIPT %s\n' "$RECEIPT"
