#!/bin/bash
# Verify, and only with explicit authorization upload, the atomic asset-pack
# release pair. App Store Connect associates this separate upload with the app
# record by numeric Apple ID.
set -euo pipefail
PATH="/usr/bin:/bin:/usr/sbin:/sbin"
export PATH
unset DEVELOPER_DIR TOOLCHAINS SDKROOT

usage() {
  /bin/cat <<'EOF'
Usage:
  /bin/bash tools/assetpack/upload.sh [--verify-only]
  /bin/bash tools/assetpack/upload.sh --upload
  /bin/bash tools/assetpack/upload.sh [--verify-only|--upload] \
    --aar DIR/sam3d-body-pose.aar \
    --receipt DIR/sam3d-body-pose.aar.receipt.json

The default and --verify-only modes perform the complete local receipt/archive
gate and never contact App Store Connect. Only --upload authorizes both the
asset-pack upload and its subsequent version-list request.
EOF
}

usage_error() {
  echo "error: $*" >&2
  usage >&2
  exit 64
}

path_directory() {
  local path="$1"
  local directory
  case "$path" in
    */*)
      directory="${path%/*}"
      [[ -n "$directory" ]] || directory="/"
      ;;
    *) directory="." ;;
  esac
  printf '%s\n' "$directory"
}

MODE="verify-only"
MODE_SEEN=0
AAR_SEEN=0
RECEIPT_SEEN=0
AAR_INPUT=""
RECEIPT_INPUT=""

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --verify-only)
      [[ "$MODE_SEEN" -eq 0 ]] || usage_error "choose exactly one mode"
      MODE="verify-only"
      MODE_SEEN=1
      shift
      ;;
    --upload)
      [[ "$MODE_SEEN" -eq 0 ]] || usage_error "choose exactly one mode"
      MODE="upload"
      MODE_SEEN=1
      shift
      ;;
    --aar)
      [[ "$AAR_SEEN" -eq 0 ]] || usage_error "--aar may be provided only once"
      [[ "$#" -ge 2 && -n "$2" ]] || usage_error "--aar requires a path"
      AAR_INPUT="$2"
      AAR_SEEN=1
      shift 2
      ;;
    --receipt)
      [[ "$RECEIPT_SEEN" -eq 0 ]] || \
        usage_error "--receipt may be provided only once"
      [[ "$#" -ge 2 && -n "$2" ]] || usage_error "--receipt requires a path"
      RECEIPT_INPUT="$2"
      RECEIPT_SEEN=1
      shift 2
      ;;
    --help|-h)
      [[ "$#" -eq 1 && "$MODE_SEEN" -eq 0 && "$AAR_SEEN" -eq 0 && \
        "$RECEIPT_SEEN" -eq 0 ]] || \
        usage_error "--help cannot be combined with other arguments"
      usage
      exit 0
      ;;
    *) usage_error "unknown or positional arguments are not accepted" ;;
  esac
done

[[ "$AAR_SEEN" -eq "$RECEIPT_SEEN" ]] || \
  usage_error "--aar and --receipt must be supplied together"

SOURCE_PATH="${BASH_SOURCE[0]}"
case "$SOURCE_PATH" in
  */*) SCRIPT_DIRECTORY="${SOURCE_PATH%/*}" ;;
  *) SCRIPT_DIRECTORY="." ;;
esac
REPO_ROOT="$(cd -P -- "$SCRIPT_DIRECTORY/../.." && pwd -P)"
VERIFIER="$REPO_ROOT/tools/assetpack/verify_model_lock.py"

if [[ "$AAR_SEEN" -eq 0 ]]; then
  RELEASE_DIRECTORY="$REPO_ROOT/build/assetpack/release"
else
  [[ "${AAR_INPUT##*/}" == "sam3d-body-pose.aar" ]] || \
    usage_error "custom AAR must be named sam3d-body-pose.aar"
  [[ "${RECEIPT_INPUT##*/}" == "sam3d-body-pose.aar.receipt.json" ]] || \
    usage_error \
      "custom receipt must be named sam3d-body-pose.aar.receipt.json"

  AAR_DIRECTORY_INPUT="$(path_directory "$AAR_INPUT")"
  RECEIPT_DIRECTORY_INPUT="$(path_directory "$RECEIPT_INPUT")"
  if ! AAR_DIRECTORY="$(cd -P -- "$AAR_DIRECTORY_INPUT" && pwd -P)"; then
    usage_error "custom AAR directory does not exist: $AAR_DIRECTORY_INPUT"
  fi
  if ! RECEIPT_DIRECTORY="$(cd -P -- "$RECEIPT_DIRECTORY_INPUT" && pwd -P)"; then
    usage_error "custom receipt directory does not exist: $RECEIPT_DIRECTORY_INPUT"
  fi
  [[ "$AAR_DIRECTORY" == "$RECEIPT_DIRECTORY" ]] || \
    usage_error "custom AAR and receipt must be in the same directory"
  RELEASE_DIRECTORY="$AAR_DIRECTORY"
fi

SOURCE_AAR="$RELEASE_DIRECTORY/sam3d-body-pose.aar"
SOURCE_RECEIPT="$RELEASE_DIRECTORY/sam3d-body-pose.aar.receipt.json"
AAR="$SOURCE_AAR"
RECEIPT="$SOURCE_RECEIPT"

TRUSTED_TEMP_ROOT="$(cd -P -- /tmp && pwd -P)"
UPLOAD_SNAPSHOT_DIRECTORY=""

is_safe_snapshot_path() {
  local candidate="$1"
  local candidate_directory="${candidate%/*}"
  local candidate_name="${candidate##*/}"
  [[ "$candidate_directory" == "$TRUSTED_TEMP_ROOT" && \
    "$candidate_name" =~ ^biomotion-assetpack-upload\.[A-Za-z0-9]+$ ]]
}

cleanup_upload_snapshot() {
  local original_exit="$?"
  local cleanup_exit=0
  trap - EXIT
  if [[ -n "$UPLOAD_SNAPSHOT_DIRECTORY" ]]; then
    if is_safe_snapshot_path "$UPLOAD_SNAPSHOT_DIRECTORY"; then
      /bin/rm -rf -- "$UPLOAD_SNAPSHOT_DIRECTORY" || cleanup_exit=1
    else
      echo "error: refusing to remove unsafe snapshot path" >&2
      cleanup_exit=1
    fi
  fi
  if [[ "$original_exit" -eq 0 && "$cleanup_exit" -ne 0 ]]; then
    original_exit="$cleanup_exit"
  fi
  exit "$original_exit"
}
trap cleanup_upload_snapshot EXIT

create_upload_snapshot() {
  if [[ -L "$SOURCE_AAR" || ! -f "$SOURCE_AAR" ]]; then
    echo "error: source AAR must be a non-symlink regular file: $SOURCE_AAR" >&2
    exit 1
  fi
  if [[ -L "$SOURCE_RECEIPT" || ! -f "$SOURCE_RECEIPT" ]]; then
    echo "error: source receipt must be a non-symlink regular file: $SOURCE_RECEIPT" >&2
    exit 1
  fi

  umask 077
  UPLOAD_SNAPSHOT_DIRECTORY="$(/usr/bin/mktemp -d \
    "$TRUSTED_TEMP_ROOT/biomotion-assetpack-upload.XXXXXX")"
  if ! is_safe_snapshot_path "$UPLOAD_SNAPSHOT_DIRECTORY"; then
    echo "error: mktemp returned an unsafe upload snapshot path" >&2
    exit 1
  fi
  /bin/chmod 0700 "$UPLOAD_SNAPSHOT_DIRECTORY"
  if [[ "$(/usr/bin/stat -f '%Lp' "$UPLOAD_SNAPSHOT_DIRECTORY")" != "700" ]]; then
    echo "error: upload snapshot directory is not mode 0700" >&2
    exit 1
  fi

  AAR="$UPLOAD_SNAPSHOT_DIRECTORY/sam3d-body-pose.aar"
  RECEIPT="$UPLOAD_SNAPSHOT_DIRECTORY/sam3d-body-pose.aar.receipt.json"
  /bin/cp -P "$SOURCE_AAR" "$AAR"
  /bin/cp -P "$SOURCE_RECEIPT" "$RECEIPT"
  /bin/chmod 0600 "$AAR" "$RECEIPT"
  if [[ -L "$AAR" || ! -f "$AAR" || -L "$RECEIPT" || ! -f "$RECEIPT" ]]; then
    echo "error: upload snapshot pair must contain only regular files" >&2
    exit 1
  fi
}

[[ -f "$VERIFIER" && ! -L "$VERIFIER" ]] || {
  echo "error: trusted receipt verifier is unavailable: $VERIFIER" >&2
  exit 1
}

if [[ "$MODE" == "upload" ]]; then
  create_upload_snapshot
fi

echo "==> verifying atomic asset-pack release pair"
/usr/bin/python3 -I "$VERIFIER" receipt "$AAR" "$RECEIPT"
echo "==> verified $SOURCE_AAR"

if [[ "$MODE" == "verify-only" ]]; then
  exit 0
fi

# Deliberately do not expand any credential variable or key path until the
# complete receipt/archive verifier above has succeeded.
if [[ -z "${ASC_API_KEY_ID:-}" ]]; then
  echo "error: ASC_API_KEY_ID is required for --upload" >&2
  exit 1
fi
if [[ -z "${ASC_API_ISSUER:-}" ]]; then
  echo "error: ASC_API_ISSUER is required for --upload" >&2
  exit 1
fi
if [[ -z "${HOME:-}" ]]; then
  echo "error: HOME is required to locate the App Store Connect API key" >&2
  exit 1
fi

API_KEY_ID="$ASC_API_KEY_ID"
API_ISSUER="$ASC_API_ISSUER"
APPLE_ID="6761994383"

[[ "$API_KEY_ID" =~ ^[A-Z0-9]{10}$ ]] || {
  echo "error: ASC_API_KEY_ID must be exactly 10 uppercase letters/digits" >&2
  exit 1
}
[[ "$API_ISSUER" =~ ^[0-9A-Fa-f]{8}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{12}$ ]] || {
  echo "error: ASC_API_ISSUER must be a UUID" >&2
  exit 1
}
KEY_PATH="$HOME/.appstoreconnect/private_keys/AuthKey_${API_KEY_ID}.p8"
if [[ -L "$KEY_PATH" ]]; then
  echo "error: ASC API key must not be a symlink: $KEY_PATH" >&2
  exit 1
fi
if [[ ! -f "$KEY_PATH" ]]; then
  echo "error: ASC API key must be a regular file: $KEY_PATH" >&2
  exit 1
fi

XCRUN="/usr/bin/xcrun"
if [[ ! -f "$XCRUN" || ! -x "$XCRUN" ]]; then
  echo "error: trusted xcrun is unavailable: $XCRUN" >&2
  exit 1
fi

echo "==> uploading verified asset pack to app $APPLE_ID"
"$XCRUN" altool --upload-asset-pack "$AAR" \
  --apple-id "$APPLE_ID" \
  --platform ios \
  --apiKey "$API_KEY_ID" \
  --apiIssuer "$API_ISSUER"

echo "==> versions now on App Store Connect"
"$XCRUN" altool --list-asset-pack-versions \
  --apple-id "$APPLE_ID" \
  --asset-pack-identifier sam3d-body-pose \
  --apiKey "$API_KEY_ID" \
  --apiIssuer "$API_ISSUER"
