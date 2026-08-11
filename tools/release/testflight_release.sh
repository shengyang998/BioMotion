#!/bin/bash
# Export a previously reviewed archive, and only with explicit authorization
# validate or upload its byte-pinned IPA to App Store Connect.
set -euo pipefail

PATH="/usr/bin:/bin:/usr/sbin:/sbin"
export PATH
unset DEVELOPER_DIR TOOLCHAINS SDKROOT PYTHONPATH PYTHONHOME

BASH_TOOL="/bin/bash"
CHMOD="/bin/chmod"
CP="/bin/cp"
ENV_TOOL="/usr/bin/env"
FIND="/usr/bin/find"
ID_TOOL="/usr/bin/id"
MKDIR="/bin/mkdir"
MKTEMP="/usr/bin/mktemp"
PLUTIL="/usr/bin/plutil"
RM="/bin/rm"
SHASUM="/usr/bin/shasum"
STAT="/usr/bin/stat"
XCODEBUILD="/usr/bin/xcodebuild"
XCRUN="/usr/bin/xcrun"

usage() {
  /bin/cat <<'EOF'
Usage:
  /bin/bash tools/release/testflight_release.sh \
    --archive PATH/BioMotion.xcarchive --export-dir NEW_OR_EMPTY_DIRECTORY
  /bin/bash tools/release/testflight_release.sh --validate \
    --archive PATH/BioMotion.xcarchive --export-dir NEW_OR_EMPTY_DIRECTORY
  /bin/bash tools/release/testflight_release.sh --upload \
    --archive PATH/BioMotion.xcarchive --export-dir NEW_OR_EMPTY_DIRECTORY

The default performs all local gates and exports exactly one IPA without an
App Store Connect request. --validate performs one altool validation after all
local gates. --upload performs that validation and then uploads the same
byte-pinned private snapshot.
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
  output="$(LC_ALL=C "$SHASUM" -a 256 "$path")"
  digest="${output%% *}"
  if [[ ! "$digest" =~ ^[0-9a-f]{64}$ ]]; then
    printf 'error: could not parse SHA-256 for %s\n' "$path" >&2
    exit 1
  fi
  printf '%s\n' "$digest"
}

run_local_without_credentials() {
  "$ENV_TOOL" -u ASC_API_KEY_ID -u ASC_API_ISSUER "$@"
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
RESOURCE_GATE="$REPO_ROOT/tools/tests/app_resource_boundary_probe.sh"
PRIVACY_GATE="$REPO_ROOT/tools/tests/privacy_manifest_probe.sh"
EXPORT_OPTIONS="$REPO_ROOT/tools/release/ExportOptions-TestFlight.plist"

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
  if [[ "$("$STAT" -f '%u' "$EXPORT_DIR")" != \
    "$("$ID_TOOL" -u)" ]]; then
    usage_error "--export-dir must be owned by the current user"
  fi
  if [[ -n "$("$FIND" "$EXPORT_DIR" -mindepth 1 -maxdepth 1 \
    -print -quit)" ]]; then
    usage_error "--export-dir must be empty"
  fi
else
  umask 077
  "$MKDIR" "$EXPORT_DIR"
fi
"$CHMOD" 0700 "$EXPORT_DIR"
if [[ "$("$STAT" -f '%Lp' "$EXPORT_DIR")" != "700" ]]; then
  printf 'error: export directory is not mode 0700: %s\n' \
    "$EXPORT_DIR" >&2
  exit 1
fi

APP_BUNDLE="$ARCHIVE/Products/Applications/BioMotion.app"

printf '%s\n' '==> checking reviewed source and generated project'
run_local_without_credentials "$BASH_TOOL" "$RESOURCE_GATE"

printf '%s\n' '==> checking signed release archive resource boundary'
run_local_without_credentials \
  "$BASH_TOOL" "$RESOURCE_GATE" --release-archive "$ARCHIVE"

printf '%s\n' '==> checking signed release archive privacy boundary'
run_local_without_credentials "$BASH_TOOL" "$PRIVACY_GATE" "$APP_BUNDLE"

require_regular_file "$EXPORT_OPTIONS" "TestFlight export options"
"$PLUTIL" -lint "$EXPORT_OPTIONS" >/dev/null
if [[ "$("$PLUTIL" -extract destination raw -o - "$EXPORT_OPTIONS")" \
  != "export" ]]; then
  printf '%s\n' \
    'error: TestFlight export options must use destination=export' >&2
  exit 1
fi
if [[ "$("$PLUTIL" -extract method raw -o - "$EXPORT_OPTIONS")" \
  != "app-store-connect" ]]; then
  printf '%s\n' \
    'error: TestFlight export options must use method=app-store-connect' >&2
  exit 1
fi
if [[ "$("$PLUTIL" -extract signingStyle raw -o - "$EXPORT_OPTIONS")" \
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
run_local_without_credentials "$XCODEBUILD" \
  -exportArchive \
  -archivePath "$ARCHIVE" \
  -exportPath "$EXPORT_DIR" \
  -exportOptionsPlist "$EXPORT_OPTIONS"

if [[ -L "$EXPORT_DIR" || ! -d "$EXPORT_DIR" ]]; then
  printf 'error: export directory changed type during export: %s\n' \
    "$EXPORT_DIR" >&2
  exit 1
fi
if [[ "$("$STAT" -f '%Lp' "$EXPORT_DIR")" != "700" ]]; then
  printf 'error: export directory permissions changed during export: %s\n' \
    "$EXPORT_DIR" >&2
  exit 1
fi

IPA_CANDIDATES=()
while IFS= read -r -d '' candidate; do
  IPA_CANDIDATES+=("$candidate")
done < <("$FIND" "$EXPORT_DIR" \( -type f -o -type l \) \
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
run_local_without_credentials \
  "$BASH_TOOL" "$RESOURCE_GATE" --release-ipa "$IPA" "$ARCHIVE"

IPA_SHA256="$(sha256_file "$IPA")"
IPA_RECEIPT="$EXPORT_DIR/${IPA##*/}.sha256"
printf '%s  %s\n' "$IPA_SHA256" "${IPA##*/}" > "$IPA_RECEIPT"
"$CHMOD" 0600 "$IPA_RECEIPT"
require_regular_file "$IPA_RECEIPT" "IPA SHA-256 receipt"
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
      "$RM" -rf -- "$SNAPSHOT_DIRECTORY" || cleanup_exit=1
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
SNAPSHOT_DIRECTORY="$("$MKTEMP" -d \
  "$TRUSTED_TEMP_ROOT/biomotion-testflight.XXXXXX")"
if ! is_safe_snapshot_directory "$SNAPSHOT_DIRECTORY"; then
  printf '%s\n' 'error: mktemp returned an unsafe snapshot path' >&2
  exit 1
fi
"$CHMOD" 0700 "$SNAPSHOT_DIRECTORY"
if [[ "$("$STAT" -f '%Lp' "$SNAPSHOT_DIRECTORY")" != "700" ]]; then
  printf '%s\n' 'error: TestFlight snapshot directory is not mode 0700' >&2
  exit 1
fi
SNAPSHOT_IPA="$SNAPSHOT_DIRECTORY/${IPA##*/}"
"$CP" -P "$IPA" "$SNAPSHOT_IPA"
"$CHMOD" 0600 "$SNAPSHOT_IPA"
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

# Deliberately do not expand credential variables or derive the API-key path
# until every source, archive, privacy, export, and IPA gate above has passed.
if [[ -z "${ASC_API_KEY_ID:-}" ]]; then
  printf 'error: ASC_API_KEY_ID is required for --%s\n' "$MODE" >&2
  exit 1
fi
if [[ -z "${ASC_API_ISSUER:-}" ]]; then
  printf 'error: ASC_API_ISSUER is required for --%s\n' "$MODE" >&2
  exit 1
fi
if [[ -z "${HOME:-}" ]]; then
  printf '%s\n' \
    'error: HOME is required to locate the App Store Connect API key' >&2
  exit 1
fi
API_KEY_ID="$ASC_API_KEY_ID"
API_ISSUER="$ASC_API_ISSUER"
if [[ ! "$API_KEY_ID" =~ ^[A-Z0-9]{10}$ ]]; then
  printf '%s\n' \
    'error: ASC_API_KEY_ID must be exactly 10 uppercase letters/digits' >&2
  exit 1
fi
if [[ ! "$API_ISSUER" =~ ^[0-9A-Fa-f]{8}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{12}$ ]]; then
  printf '%s\n' 'error: ASC_API_ISSUER must be a UUID' >&2
  exit 1
fi
API_KEY_PATH="$HOME/.appstoreconnect/private_keys/AuthKey_${API_KEY_ID}.p8"
require_regular_file "$API_KEY_PATH" "App Store Connect API key"
if [[ ! -f "$XCRUN" || ! -x "$XCRUN" ]]; then
  printf 'error: trusted xcrun is unavailable: %s\n' "$XCRUN" >&2
  exit 1
fi

verify_external_bytes
printf '%s\n' '==> validating byte-pinned IPA with App Store Connect'
"$XCRUN" altool --validate-app \
  -f "$SNAPSHOT_IPA" \
  -t ios \
  --apiKey "$API_KEY_ID" \
  --apiIssuer "$API_ISSUER"

if [[ "$MODE" == "validate" ]]; then
  printf '%s\n' 'TESTFLIGHT_RELEASE_PASS validate'
  exit 0
fi

verify_external_bytes
printf '%s\n' '==> uploading the same byte-pinned IPA to App Store Connect'
"$XCRUN" altool --upload-app \
  -f "$SNAPSHOT_IPA" \
  -t ios \
  --apiKey "$API_KEY_ID" \
  --apiIssuer "$API_ISSUER"
printf '%s\n' 'TESTFLIGHT_RELEASE_PASS upload'
