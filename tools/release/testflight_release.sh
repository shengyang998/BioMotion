#!/bin/bash -p
# Export a previously reviewed archive, and only with explicit authorization
# validate or upload its byte-pinned IPA to App Store Connect.
case "$-" in
  *p*) ;;
  *)
    printf '%s\n' \
      'error: execute testflight_release.sh directly or with /bin/bash -p' >&2
    exit 78
    ;;
esac
set -euo pipefail

TRUSTED_PATH="/usr/bin:/bin:/usr/sbin:/sbin"
PATH="$TRUSTED_PATH"
export PATH
unset \
  BASH_ENV CDPATH DEVELOPER_DIR DYLD_INSERT_LIBRARIES DYLD_LIBRARY_PATH \
  ENV GIT_CONFIG_GLOBAL GIT_CONFIG_SYSTEM GREP_OPTIONS PERL5LIB PERL5OPT \
  PYTHONHOME PYTHONINSPECT PYTHONPATH PYTHONSTARTUP PYTHONWARNINGS SDKROOT \
  TOOLCHAINS XCODE_XCCONFIG_FILE

BASH_TOOL="/bin/bash"
CHMOD="/bin/chmod"
CP="/bin/cp"
ENV_TOOL="/usr/bin/env"
FIND="/usr/bin/find"
ID_TOOL="/usr/bin/id"
MKDIR="/bin/mkdir"
MKTEMP="/usr/bin/mktemp"
PLUTIL="/usr/bin/plutil"
PYTHON="/usr/bin/python3"
RM="/bin/rm"
SECURITY="/usr/bin/security"
SHASUM="/usr/bin/shasum"
STAT="/usr/bin/stat"
XCODEBUILD="/usr/bin/xcodebuild"
XCRUN="/usr/bin/xcrun"
HERMETIC_HOME="/var/empty"

# Bootstrap the account identity without consulting the caller's environment.
# In particular, HOME is an input to credential lookup and must come from the
# current uid's system account record rather than from an exported variable.
TRUSTED_UID="$(
  "$ENV_TOOL" -i PATH="$TRUSTED_PATH" LANG=C LC_ALL=C \
    "$ID_TOOL" -u
)"
TRUSTED_USER="$(
  "$ENV_TOOL" -i PATH="$TRUSTED_PATH" LANG=C LC_ALL=C \
    "$PYTHON" -I -c \
    'import os, pwd; print(pwd.getpwuid(os.getuid()).pw_name)'
)"
TRUSTED_HOME="$(
  "$ENV_TOOL" -i PATH="$TRUSTED_PATH" LANG=C LC_ALL=C \
    "$PYTHON" -I -c \
    'import os, pwd; print(pwd.getpwuid(os.getuid()).pw_dir)'
)"
if [[ ! "$TRUSTED_UID" =~ ^[0-9]+$ || -z "$TRUSTED_USER" || \
  "$TRUSTED_USER" == *$'\n'* || "$TRUSTED_USER" == *$'\r'* || \
  "$TRUSTED_USER" == *:* || "$TRUSTED_USER" == */* ]]; then
  printf '%s\n' 'error: could not establish a safe current account identity' >&2
  exit 1
fi
if [[ -z "$TRUSTED_HOME" || "$TRUSTED_HOME" != /* || \
  "$TRUSTED_HOME" == *$'\n'* || "$TRUSTED_HOME" == *$'\r'* || \
  -L "$TRUSTED_HOME" || ! -d "$TRUSTED_HOME" ]]; then
  printf '%s\n' 'error: system account HOME is not a safe directory' >&2
  exit 1
fi
CANONICAL_TRUSTED_HOME="$(cd -P -- "$TRUSTED_HOME" && pwd -P)"
if [[ "$CANONICAL_TRUSTED_HOME" != "$TRUSTED_HOME" || \
  "$("$ENV_TOOL" -i PATH="$TRUSTED_PATH" LANG=C LC_ALL=C \
    "$STAT" -f '%u' "$TRUSTED_HOME")" != "$TRUSTED_UID" || \
  "$("$ENV_TOOL" -i PATH="$TRUSTED_PATH" LANG=C LC_ALL=C \
    "$ID_TOOL" -un)" != "$TRUSTED_USER" ]]; then
  printf '%s\n' 'error: system account HOME or user identity failed validation' >&2
  exit 1
fi
if [[ -L "$HERMETIC_HOME" || ! -d "$HERMETIC_HOME" || \
  "$("$ENV_TOOL" -i PATH="$TRUSTED_PATH" LANG=C LC_ALL=C \
    "$STAT" -f '%u' "$HERMETIC_HOME")" != "0" ]]; then
  printf '%s\n' 'error: credential-blind gate HOME failed validation' >&2
  exit 1
fi

run_hermetic_gate() {
  "$ENV_TOOL" -i \
    PATH="$TRUSTED_PATH" \
    HOME="$HERMETIC_HOME" \
    USER="$TRUSTED_USER" \
    LOGNAME="$TRUSTED_USER" \
    LANG=C \
    LC_ALL=C \
    "$@"
}

run_trusted_user_tool() {
  "$ENV_TOOL" -i \
    PATH="$TRUSTED_PATH" \
    HOME="$TRUSTED_HOME" \
    USER="$TRUSTED_USER" \
    LOGNAME="$TRUSTED_USER" \
    LANG=C \
    LC_ALL=C \
    "$@"
}

usage() {
  run_hermetic_gate /bin/cat <<'EOF'
Usage:
  /bin/bash -p tools/release/testflight_release.sh \
    --archive PATH/BioMotion.xcarchive --export-dir NEW_OR_EMPTY_DIRECTORY
  /bin/bash -p tools/release/testflight_release.sh --validate \
    --archive PATH/BioMotion.xcarchive --export-dir NEW_OR_EMPTY_DIRECTORY
  /bin/bash -p tools/release/testflight_release.sh --upload \
    --archive PATH/BioMotion.xcarchive --export-dir NEW_OR_EMPTY_DIRECTORY

The archive must have the adjacent dependency receipt produced by
archive_release.sh. The default reverifies it, performs all local gates, and
exports exactly one IPA without an App Store Connect request. --validate
performs one altool validation after all local gates. --upload performs that
validation and then uploads the same byte-pinned private snapshot.
EOF
}

usage_error() {
  printf 'error: %s\n' "$*" >&2
  usage >&2
  exit 64
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

sha256_file() {
  local path="$1"
  local output
  local digest
  require_regular_file "$path" "hashed IPA"
  output="$(run_hermetic_gate "$SHASUM" -a 256 "$path")"
  digest="${output%% *}"
  if [[ ! "$digest" =~ ^[0-9a-f]{64}$ ]]; then
    printf 'error: could not parse SHA-256 for %s\n' "$path" >&2
    exit 1
  fi
  printf '%s\n' "$digest"
}

write_ipa_receipt_exclusive() {
  local path="$1"
  local digest="$2"
  local ipa_name="$3"
  run_hermetic_gate "$PYTHON" -I - "$path" "$digest" "$ipa_name" <<'PY'
import os
from pathlib import Path
import re
import sys

if len(sys.argv) != 4:
    raise SystemExit(64)
path = Path(sys.argv[1])
digest = sys.argv[2]
ipa_name = sys.argv[3]
if re.fullmatch(r"[0-9a-f]{64}", digest) is None:
    raise SystemExit("invalid IPA digest")
if not ipa_name or ipa_name in {".", ".."} or "/" in ipa_name or "\x00" in ipa_name:
    raise SystemExit("invalid IPA filename")
parent = path.parent
name = path.name
if not name or name in {".", ".."} or "/" in name or "\x00" in name:
    raise SystemExit("invalid receipt filename")

directory_flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
file_flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW
if hasattr(os, "O_CLOEXEC"):
    directory_flags |= os.O_CLOEXEC
    file_flags |= os.O_CLOEXEC
directory = os.open(parent, directory_flags)
descriptor = -1
created = False
try:
    descriptor = os.open(name, file_flags, 0o600, dir_fd=directory)
    created = True
    os.fchmod(descriptor, 0o600)
    payload = f"{digest}  {ipa_name}\n".encode("ascii")
    offset = 0
    while offset < len(payload):
        written = os.write(descriptor, payload[offset:])
        if written <= 0:
            raise OSError("short receipt write")
        offset += written
    os.fsync(descriptor)
    os.close(descriptor)
    descriptor = -1
    os.fsync(directory)
except Exception:
    if descriptor >= 0:
        os.close(descriptor)
    if created:
        try:
            os.unlink(name, dir_fd=directory)
            os.fsync(directory)
        except OSError:
            pass
    raise
finally:
    os.close(directory)
PY
}

require_private_key_directory() {
  local path="$1"
  local label="$2"
  local mode
  if [[ -L "$path" || ! -d "$path" || \
    "$(cd -P -- "$path" 2>/dev/null && pwd -P)" != "$path" || \
    "$(run_hermetic_gate "$STAT" -f '%u' "$path" 2>/dev/null || true)" != \
      "$TRUSTED_UID" ]]; then
    printf 'error: %s must be a physical current-user directory: %s\n' \
      "$label" "$path" >&2
    exit 1
  fi
  mode="$(run_hermetic_gate "$STAT" -f '%Lp' "$path")"
  if [[ ! "$mode" =~ ^[0-7]{3,4}$ ]] || (( (8#$mode & 0077) != 0 )); then
    printf 'error: %s must not grant group/world permissions: %s\n' \
      "$label" "$path" >&2
    exit 1
  fi
}

MODE="export-only"
MODE_SEEN=0
ARCHIVE_SEEN=0
EXPORT_SEEN=0
ARCHIVE_INPUT=""
EXPORT_INPUT=""

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --validate)
      [[ "$MODE_SEEN" -eq 0 ]] || usage_error "choose exactly one mode"
      MODE="validate"
      MODE_SEEN=1
      shift
      ;;
    --upload)
      [[ "$MODE_SEEN" -eq 0 ]] || usage_error "choose exactly one mode"
      MODE="upload"
      MODE_SEEN=1
      shift
      ;;
    --archive)
      [[ "$ARCHIVE_SEEN" -eq 0 ]] || \
        usage_error "--archive may be provided only once"
      [[ "$#" -ge 2 && -n "$2" ]] || \
        usage_error "--archive requires a path"
      ARCHIVE_INPUT="$2"
      ARCHIVE_SEEN=1
      shift 2
      ;;
    --export-dir)
      [[ "$EXPORT_SEEN" -eq 0 ]] || \
        usage_error "--export-dir may be provided only once"
      [[ "$#" -ge 2 && -n "$2" ]] || \
        usage_error "--export-dir requires a path"
      EXPORT_INPUT="$2"
      EXPORT_SEEN=1
      shift 2
      ;;
    --help|-h)
      [[ "$#" -eq 1 && "$MODE_SEEN" -eq 0 && \
        "$ARCHIVE_SEEN" -eq 0 && "$EXPORT_SEEN" -eq 0 ]] || \
        usage_error "--help cannot be combined with other arguments"
      usage
      exit 0
      ;;
    *) usage_error "unknown or positional arguments are not accepted" ;;
  esac
done

[[ "$ARCHIVE_SEEN" -eq 1 ]] || usage_error "--archive is required"
[[ "$EXPORT_SEEN" -eq 1 ]] || usage_error "--export-dir is required"

SOURCE_PATH="${BASH_SOURCE[0]}"
if [[ -L "$SOURCE_PATH" ]]; then
  printf 'error: release wrapper must not be invoked through a symlink\n' >&2
  exit 1
fi
case "$SOURCE_PATH" in
  */*) SCRIPT_DIRECTORY="${SOURCE_PATH%/*}" ;;
  *) SCRIPT_DIRECTORY="." ;;
esac
REPO_ROOT="$(cd -P -- "$SCRIPT_DIRECTORY/../.." && pwd -P)"
DEPENDENCY_GATE="$REPO_ROOT/tools/tests/dependency_boundary_probe.sh"
DEPENDENCY_RECEIPT_INSPECTOR="$REPO_ROOT/tools/release/dependency_archive_receipt.py"
RESOURCE_GATE="$REPO_ROOT/tools/tests/app_resource_boundary_probe.sh"
PRIVACY_GATE="$REPO_ROOT/tools/tests/privacy_manifest_probe.sh"
EXPORT_OPTIONS="$REPO_ROOT/tools/release/ExportOptions-TestFlight.plist"

require_regular_file "$DEPENDENCY_GATE" "dependency-boundary gate"
require_regular_file \
  "$DEPENDENCY_RECEIPT_INSPECTOR" "dependency-archive receipt inspector"
require_regular_file "$RESOURCE_GATE" "resource-boundary gate"
require_regular_file "$PRIVACY_GATE" "privacy-manifest gate"

ARCHIVE_INPUT="$(strip_trailing_slashes "$ARCHIVE_INPUT")"
if [[ -L "$ARCHIVE_INPUT" || ! -d "$ARCHIVE_INPUT" ]]; then
  printf 'error: archive must be a non-symlink directory: %s\n' \
    "$ARCHIVE_INPUT" >&2
  exit 1
fi
ARCHIVE="$(cd -P -- "$ARCHIVE_INPUT" && pwd -P)"
[[ "${ARCHIVE##*/}" == *.xcarchive ]] || \
  usage_error "--archive must name a .xcarchive directory"
DEPENDENCY_RECEIPT="$ARCHIVE.dependency-receipt.json"
require_regular_file "$DEPENDENCY_RECEIPT" "archive dependency receipt"

printf '%s\n' '==> checking current native dependency boundary'
run_hermetic_gate "$BASH_TOOL" -p "$DEPENDENCY_GATE"

printf '%s\n' '==> checking archive dependency receipt and app bytes'
run_hermetic_gate \
  "$PYTHON" -I "$DEPENDENCY_RECEIPT_INSPECTOR" verify \
  "$REPO_ROOT" "$ARCHIVE" "$DEPENDENCY_RECEIPT"

EXPORT_INPUT="$(strip_trailing_slashes "$EXPORT_INPUT")"
case "$EXPORT_INPUT" in
  ""|"/"|"."|".."|*/.|*/..) usage_error "unsafe --export-dir path" ;;
esac
EXPORT_LEAF="${EXPORT_INPUT##*/}"
EXPORT_PARENT_INPUT="$(path_directory "$EXPORT_INPUT")"
if [[ ! -d "$EXPORT_PARENT_INPUT" ]]; then
  usage_error "--export-dir parent does not exist: $EXPORT_PARENT_INPUT"
fi
EXPORT_PARENT="$(cd -P -- "$EXPORT_PARENT_INPUT" && pwd -P)"
EXPORT_DIR="$EXPORT_PARENT/$EXPORT_LEAF"
case "$EXPORT_DIR" in
  "$ARCHIVE"|"$ARCHIVE"/*)
    usage_error "--export-dir must not be inside the archive"
    ;;
esac

if [[ -e "$EXPORT_DIR" || -L "$EXPORT_DIR" ]]; then
  if [[ -L "$EXPORT_DIR" || ! -d "$EXPORT_DIR" ]]; then
    usage_error "--export-dir must be a non-symlink directory"
  fi
  if [[ "$(run_hermetic_gate "$STAT" -f '%u' "$EXPORT_DIR")" != \
    "$TRUSTED_UID" ]]; then
    usage_error "--export-dir must be owned by the current user"
  fi
  if [[ -n "$(run_hermetic_gate "$FIND" "$EXPORT_DIR" \
    -mindepth 1 -maxdepth 1 \
    -print -quit)" ]]; then
    usage_error "--export-dir must be empty"
  fi
else
  umask 077
  run_hermetic_gate "$MKDIR" "$EXPORT_DIR"
fi
run_hermetic_gate "$CHMOD" 0700 "$EXPORT_DIR"
if [[ "$(run_hermetic_gate "$STAT" -f '%Lp' "$EXPORT_DIR")" != "700" ]]; then
  printf 'error: export directory is not mode 0700: %s\n' \
    "$EXPORT_DIR" >&2
  exit 1
fi

APP_BUNDLE="$ARCHIVE/Products/Applications/BioMotion.app"

printf '%s\n' '==> checking reviewed source and generated project'
run_hermetic_gate "$BASH_TOOL" -p "$RESOURCE_GATE"

printf '%s\n' '==> checking signed release archive resource boundary'
run_hermetic_gate \
  "$BASH_TOOL" -p "$RESOURCE_GATE" --release-archive "$ARCHIVE"

printf '%s\n' '==> checking signed release archive privacy boundary'
run_hermetic_gate "$BASH_TOOL" -p "$PRIVACY_GATE" "$APP_BUNDLE"

require_regular_file "$EXPORT_OPTIONS" "TestFlight export options"
run_hermetic_gate "$PLUTIL" -lint "$EXPORT_OPTIONS" >/dev/null
if [[ "$(run_hermetic_gate "$PLUTIL" -extract destination raw -o - \
  "$EXPORT_OPTIONS")" \
  != "export" ]]; then
  printf '%s\n' \
    'error: TestFlight export options must use destination=export' >&2
  exit 1
fi
if [[ "$(run_hermetic_gate "$PLUTIL" -extract method raw -o - \
  "$EXPORT_OPTIONS")" \
  != "app-store-connect" ]]; then
  printf '%s\n' \
    'error: TestFlight export options must use method=app-store-connect' >&2
  exit 1
fi
if [[ "$(run_hermetic_gate "$PLUTIL" -extract signingStyle raw -o - \
  "$EXPORT_OPTIONS")" \
  != "manual" ]]; then
  printf '%s\n' 'error: TestFlight export must retain manual signing' >&2
  exit 1
fi
if [[ ! -f "$XCODEBUILD" || ! -x "$XCODEBUILD" ]]; then
  printf 'error: trusted xcodebuild is unavailable: %s\n' \
    "$XCODEBUILD" >&2
  exit 1
fi

printf '%s\n' '==> exporting locally (no upload authorization)'
run_trusted_user_tool "$XCODEBUILD" \
  -exportArchive \
  -archivePath "$ARCHIVE" \
  -exportPath "$EXPORT_DIR" \
  -exportOptionsPlist "$EXPORT_OPTIONS"

if [[ -L "$EXPORT_DIR" || ! -d "$EXPORT_DIR" ]]; then
  printf 'error: export directory changed type during export: %s\n' \
    "$EXPORT_DIR" >&2
  exit 1
fi
if [[ "$(run_hermetic_gate "$STAT" -f '%Lp' "$EXPORT_DIR")" != "700" ]]; then
  printf 'error: export directory permissions changed during export: %s\n' \
    "$EXPORT_DIR" >&2
  exit 1
fi

IPA_CANDIDATES=()
while IFS= read -r -d '' candidate; do
  IPA_CANDIDATES+=("$candidate")
done < <(run_hermetic_gate "$FIND" "$EXPORT_DIR" \
  \( -type f -o -type l \) \
  -name '*.ipa' -print0)
if [[ "${#IPA_CANDIDATES[@]}" -ne 1 ]]; then
  printf 'error: export must contain exactly one IPA; found %s\n' \
    "${#IPA_CANDIDATES[@]}" >&2
  exit 1
fi
IPA="${IPA_CANDIDATES[0]}"
require_regular_file "$IPA" "exported IPA"
IPA_PARENT="$(cd -P -- "${IPA%/*}" && pwd -P)"
if [[ "$IPA_PARENT" != "$EXPORT_DIR" ]]; then
  printf 'error: exported IPA must be at the export directory root: %s\n' \
    "$IPA" >&2
  exit 1
fi

printf '%s\n' '==> checking exported IPA against its signed archive'
run_hermetic_gate \
  "$BASH_TOOL" -p "$RESOURCE_GATE" --release-ipa "$IPA" "$ARCHIVE"

printf '%s\n' '==> rechecking archive dependency receipt after export'
run_hermetic_gate \
  "$PYTHON" -I "$DEPENDENCY_RECEIPT_INSPECTOR" verify \
  "$REPO_ROOT" "$ARCHIVE" "$DEPENDENCY_RECEIPT"

IPA_SHA256="$(sha256_file "$IPA")"
IPA_RECEIPT="$EXPORT_DIR/${IPA##*/}.sha256"
write_ipa_receipt_exclusive "$IPA_RECEIPT" "$IPA_SHA256" "${IPA##*/}"
require_regular_file "$IPA_RECEIPT" "IPA SHA-256 receipt"
if [[ "$(run_hermetic_gate "$STAT" -f '%Lp' "$IPA_RECEIPT")" != "600" || \
  "$(run_hermetic_gate "$STAT" -f '%u' "$IPA_RECEIPT")" != \
    "$TRUSTED_UID" ]]; then
  printf '%s\n' 'error: IPA SHA-256 receipt is not a private current-user file' >&2
  exit 1
fi
printf 'IPA_SHA256 %s  %s\n' "$IPA_SHA256" "$IPA"

if [[ "$MODE" == "export-only" ]]; then
  printf '%s\n' 'TESTFLIGHT_RELEASE_PASS export-only'
  exit 0
fi

TRUSTED_TEMP_ROOT="$(cd -P -- /tmp && pwd -P)"
SNAPSHOT_DIRECTORY=""
SNAPSHOT_IPA=""

is_safe_snapshot_directory() {
  local candidate="$1"
  local candidate_parent="${candidate%/*}"
  local candidate_name="${candidate##*/}"
  [[ "$candidate_parent" == "$TRUSTED_TEMP_ROOT" && \
    "$candidate_name" =~ ^biomotion-testflight\.[A-Za-z0-9]+$ ]]
}

cleanup_snapshot() {
  local original_exit="$?"
  local cleanup_exit=0
  trap - EXIT
  if [[ -n "$SNAPSHOT_DIRECTORY" ]]; then
    if is_safe_snapshot_directory "$SNAPSHOT_DIRECTORY"; then
      run_hermetic_gate "$RM" -rf -- "$SNAPSHOT_DIRECTORY" || \
        cleanup_exit=1
    else
      printf '%s\n' \
        'error: refusing to remove unsafe TestFlight snapshot path' >&2
      cleanup_exit=1
    fi
  fi
  if [[ "$original_exit" -eq 0 && "$cleanup_exit" -ne 0 ]]; then
    original_exit="$cleanup_exit"
  fi
  exit "$original_exit"
}
trap cleanup_snapshot EXIT

umask 077
SNAPSHOT_DIRECTORY="$(run_hermetic_gate "$MKTEMP" -d \
  "$TRUSTED_TEMP_ROOT/biomotion-testflight.XXXXXX")"
if ! is_safe_snapshot_directory "$SNAPSHOT_DIRECTORY"; then
  printf '%s\n' 'error: mktemp returned an unsafe snapshot path' >&2
  exit 1
fi
run_hermetic_gate "$CHMOD" 0700 "$SNAPSHOT_DIRECTORY"
if [[ "$(run_hermetic_gate "$STAT" -f '%Lp' \
  "$SNAPSHOT_DIRECTORY")" != "700" ]]; then
  printf '%s\n' 'error: TestFlight snapshot directory is not mode 0700' >&2
  exit 1
fi
SNAPSHOT_IPA="$SNAPSHOT_DIRECTORY/${IPA##*/}"
run_hermetic_gate "$CP" -P "$IPA" "$SNAPSHOT_IPA"
run_hermetic_gate "$CHMOD" 0600 "$SNAPSHOT_IPA"
require_regular_file "$SNAPSHOT_IPA" "private IPA snapshot"

verify_external_bytes() {
  local source_digest
  local snapshot_digest
  source_digest="$(sha256_file "$IPA")"
  snapshot_digest="$(sha256_file "$SNAPSHOT_IPA")"
  if [[ "$source_digest" != "$IPA_SHA256" || \
    "$snapshot_digest" != "$IPA_SHA256" ]]; then
    printf '%s\n' \
      'error: exported or private IPA bytes changed after local review' >&2
    exit 1
  fi
}

# Deliberately do not expand credential variables, consult Keychain, or derive
# the API-key path until every source, archive, privacy, export, and IPA gate
# above has passed. Explicit environment values remain the per-run override;
# otherwise use the workstation's owner-scoped unattended release references.
API_KEY_ID="${ASC_API_KEY_ID:-}"
API_ISSUER="${ASC_API_ISSUER:-}"
if [[ -z "$API_KEY_ID" ]]; then
  if [[ ! -f "$SECURITY" || ! -x "$SECURITY" ]]; then
    printf 'error: trusted security tool is unavailable: %s\n' \
      "$SECURITY" >&2
    exit 1
  fi
  API_KEY_ID="$(run_trusted_user_tool "$SECURITY" find-generic-password \
    -a biomotion \
    -s com.soleilyu.biomotion.appstoreconnect.key-id -w 2>/dev/null || true)"
fi
if [[ -z "$API_ISSUER" ]]; then
  if [[ ! -f "$SECURITY" || ! -x "$SECURITY" ]]; then
    printf 'error: trusted security tool is unavailable: %s\n' \
      "$SECURITY" >&2
    exit 1
  fi
  API_ISSUER="$(run_trusted_user_tool "$SECURITY" find-generic-password \
    -a biomotion \
    -s com.soleilyu.biomotion.appstoreconnect.issuer -w 2>/dev/null || true)"
fi
if [[ -z "$API_KEY_ID" ]]; then
  printf 'error: ASC API key id is absent from the environment and Keychain\n' >&2
  exit 1
fi
if [[ -z "$API_ISSUER" ]]; then
  printf 'error: ASC API issuer is absent from the environment and Keychain\n' >&2
  exit 1
fi
if [[ ! "$API_KEY_ID" =~ ^[A-Z0-9]{10}$ ]]; then
  printf '%s\n' \
    'error: ASC_API_KEY_ID must be exactly 10 uppercase letters/digits' >&2
  exit 1
fi
if [[ ! "$API_ISSUER" =~ ^[0-9A-Fa-f]{8}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{12}$ ]]; then
  printf '%s\n' 'error: ASC_API_ISSUER must be a UUID' >&2
  exit 1
fi
API_KEY_PATH="$TRUSTED_HOME/.appstoreconnect/private_keys/AuthKey_${API_KEY_ID}.p8"
require_private_key_directory \
  "$TRUSTED_HOME/.appstoreconnect" "App Store Connect configuration directory"
require_private_key_directory \
  "$TRUSTED_HOME/.appstoreconnect/private_keys" \
  "App Store Connect private-key directory"
require_regular_file "$API_KEY_PATH" "App Store Connect API key"
if [[ "$(run_hermetic_gate "$STAT" -f '%u' "$API_KEY_PATH")" != \
    "$TRUSTED_UID" || \
  "$(run_hermetic_gate "$STAT" -f '%Lp' "$API_KEY_PATH")" != "600" ]]; then
  printf '%s\n' \
    'error: App Store Connect API key must be current-user owned and mode 0600' >&2
  exit 1
fi
if [[ ! -f "$XCRUN" || ! -x "$XCRUN" ]]; then
  printf 'error: trusted xcrun is unavailable: %s\n' "$XCRUN" >&2
  exit 1
fi

verify_external_bytes
printf '%s\n' '==> validating byte-pinned IPA with App Store Connect'
run_trusted_user_tool "$XCRUN" altool --validate-app \
  -f "$SNAPSHOT_IPA" \
  -t ios \
  --apiKey "$API_KEY_ID" \
  --apiIssuer "$API_ISSUER" \
  --p8-file-path "$API_KEY_PATH"

if [[ "$MODE" == "validate" ]]; then
  printf '%s\n' 'TESTFLIGHT_RELEASE_PASS validate'
  exit 0
fi

verify_external_bytes
printf '%s\n' '==> uploading the same byte-pinned IPA to App Store Connect'
run_trusted_user_tool "$XCRUN" altool --upload-app \
  -f "$SNAPSHOT_IPA" \
  -t ios \
  --apiKey "$API_KEY_ID" \
  --apiIssuer "$API_ISSUER" \
  --p8-file-path "$API_KEY_PATH"
printf '%s\n' 'TESTFLIGHT_RELEASE_PASS upload'
