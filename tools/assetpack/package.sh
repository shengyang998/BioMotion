#!/bin/bash
# Build and transactionally publish the locked SAM 3D Body Background Assets pack.
#
#   bash tools/assetpack/package.sh [path/to/SAM3DBodyPose.mlpackage]
#
# Success outputs:
#   build/assetpack/release/sam3d-body-pose.aar
#   build/assetpack/release/sam3d-body-pose.aar.receipt.json
# Any final-publish outcome not explicitly proven safe retains its transaction
# and package lock, and prints exact paths instead of deleting recovery evidence.
set -euo pipefail
umask 077
PATH="/usr/bin:/bin:/usr/sbin:/sbin"
export PATH

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PACK_ID="sam3d-body-pose"
MODEL_NAME="SAM3DBodyPose"
MANIFEST="$REPO_ROOT/tools/assetpack/Manifest.json"
LOCK_FILE="$REPO_ROOT/BioMotion/Resources/${MODEL_NAME}.lock.json"
LICENSE_FILE="$REPO_ROOT/BioMotion/Resources/SAM-LICENSE.txt"
VERIFIER="$REPO_ROOT/tools/assetpack/verify_model_lock.py"
PYTHON3="/usr/bin/python3"
XCRUN="/usr/bin/xcrun"
OUTPUT_DIRECTORY="$REPO_ROOT/build/assetpack"
RELEASE_DIRECTORY="$OUTPUT_DIRECTORY/release"
OUT="$RELEASE_DIRECTORY/${PACK_ID}.aar"
RECEIPT_OUT="$OUT.receipt.json"
TRANSACTION_DIRECTORY=""
PACKAGE_LOCK="$OUTPUT_DIRECTORY/.package-$PACK_ID.lock"
LOCK_HELD=0
PRESERVE_TRANSACTION=0

DEFAULT_SOURCES=(
  "$REPO_ROOT/../sam-3d-body/export/coreml/${MODEL_NAME}.mlpackage"
  "$REPO_ROOT/BioMotion/CoreML/${MODEL_NAME}.mlpackage"
)

SOURCE="${1:-}"
if [[ -z "$SOURCE" ]]; then
  for candidate in "${DEFAULT_SOURCES[@]}"; do
    if [[ -d "$candidate" ]]; then
      SOURCE="$candidate"
      break
    fi
  done
fi
if [[ -z "$SOURCE" || ! -d "$SOURCE" ]]; then
  echo "error: no ${MODEL_NAME}.mlpackage found." >&2
  printf '       looked in:\n' >&2
  printf '         %s\n' "${DEFAULT_SOURCES[@]}" >&2
  echo "       pass one explicitly: bash tools/assetpack/package.sh <path.mlpackage>" >&2
  exit 1
fi

cleanup() {
  if [[ -n "$TRANSACTION_DIRECTORY" ]]; then
    if [[ "$PRESERVE_TRANSACTION" -eq 1 ]]; then
      echo "PACKAGE_RECOVERY_REQUIRED transaction: $TRANSACTION_DIRECTORY" >&2
      echo "PACKAGE_RECOVERY_REQUIRED release candidate: $RELEASE_CANDIDATE" >&2
      echo "PACKAGE_RECOVERY_REQUIRED release destination: $RELEASE_DIRECTORY" >&2
    else
      case "$TRANSACTION_DIRECTORY" in
        "$OUTPUT_DIRECTORY"/.package-"$PACK_ID".*)
          rm -rf -- "$TRANSACTION_DIRECTORY"
          ;;
        *)
          echo "warning: refusing to clean unexpected transaction path: $TRANSACTION_DIRECTORY" >&2
          ;;
      esac
    fi
  fi
  if [[ "$LOCK_HELD" -eq 1 ]]; then
    if [[ "$PRESERVE_TRANSACTION" -eq 1 ]]; then
      echo "PACKAGE_RECOVERY_REQUIRED package lock retained: $PACKAGE_LOCK" >&2
    else
      rmdir "$PACKAGE_LOCK" 2>/dev/null || true
    fi
  fi
}
trap cleanup EXIT

if [[ -L "$OUTPUT_DIRECTORY" ]]; then
  echo "error: output directory must not be a symlink: $OUTPUT_DIRECTORY" >&2
  exit 1
fi
mkdir -p "$OUTPUT_DIRECTORY"
if ! mkdir "$PACKAGE_LOCK" 2>/dev/null; then
  echo "error: another asset-pack package transaction is active: $PACKAGE_LOCK" >&2
  exit 1
fi
LOCK_HELD=1
TRANSACTION_DIRECTORY="$(mktemp -d "$OUTPUT_DIRECTORY/.package-$PACK_ID.XXXXXX")"

COMPILE_ROOT="$TRANSACTION_DIRECTORY/compiled"
AUTHORITY="$TRANSACTION_DIRECTORY/authority"
STAGE="$TRANSACTION_DIRECTORY/stage"
SEAL_EXTRACTED="$TRANSACTION_DIRECTORY/seal-extracted"
PUBLISH_EXTRACTED="$TRANSACTION_DIRECTORY/publish-extracted"
AAR_CANDIDATE="$TRANSACTION_DIRECTORY/${PACK_ID}.aar"
RECEIPT_CANDIDATE="$TRANSACTION_DIRECTORY/${PACK_ID}.aar.receipt.json"
COMPILED_MODEL="$COMPILE_ROOT/${MODEL_NAME}.mlmodelc"
RELEASE_CANDIDATE="$TRANSACTION_DIRECTORY/release-candidate"
SNAPSHOT_MANIFEST="$AUTHORITY/Manifest.json"
SNAPSHOT_LOCK="$AUTHORITY/${MODEL_NAME}.lock.json"
SNAPSHOT_LICENSE="$AUTHORITY/SAM-LICENSE.txt"

echo "==> snapshot authority"
mkdir -m 700 "$AUTHORITY"
cp -p "$MANIFEST" "$SNAPSHOT_MANIFEST"
cp -p "$LOCK_FILE" "$SNAPSHOT_LOCK"
cp -p "$LICENSE_FILE" "$SNAPSHOT_LICENSE"
cmp -s "$MANIFEST" "$SNAPSHOT_MANIFEST" || {
  echo "error: Manifest.json changed while snapshotting authority" >&2
  exit 1
}
cmp -s "$LOCK_FILE" "$SNAPSHOT_LOCK" || {
  echo "error: model lock changed while snapshotting authority" >&2
  exit 1
}
cmp -s "$LICENSE_FILE" "$SNAPSHOT_LICENSE" || {
  echo "error: SAM license changed while snapshotting authority" >&2
  exit 1
}

echo "==> verify frozen repository"
"$PYTHON3" "$VERIFIER" repository --lock "$SNAPSHOT_LOCK" --license "$SNAPSHOT_LICENSE"

echo "==> verify frozen toolchain"
"$PYTHON3" "$VERIFIER" toolchain --lock "$SNAPSHOT_LOCK" --license "$SNAPSHOT_LICENSE"

echo "==> verify source     $SOURCE"
"$PYTHON3" "$VERIFIER" source \
  --lock "$SNAPSHOT_LOCK" \
  --license "$SNAPSHOT_LICENSE" \
  "$SOURCE"

echo "==> compile private   ${MODEL_NAME}.mlpackage -> ${MODEL_NAME}.mlmodelc"
mkdir -m 700 "$COMPILE_ROOT"
"$XCRUN" coremlcompiler compile "$SOURCE" "$COMPILE_ROOT"
[[ -d "$COMPILED_MODEL" ]] || {
  echo "error: coremlcompiler produced no ${MODEL_NAME}.mlmodelc" >&2
  exit 1
}
"$PYTHON3" "$VERIFIER" normalize-compiled \
  --lock "$SNAPSHOT_LOCK" \
  --license "$SNAPSHOT_LICENSE" \
  "$COMPILED_MODEL"

echo "==> verify compiled"
"$PYTHON3" "$VERIFIER" compiled \
  --lock "$SNAPSHOT_LOCK" \
  --license "$SNAPSHOT_LICENSE" \
  "$COMPILED_MODEL"

echo "==> stage exact payload"
"$PYTHON3" "$VERIFIER" manifest \
  --lock "$SNAPSHOT_LOCK" \
  --license "$SNAPSHOT_LICENSE" \
  "$SNAPSHOT_MANIFEST"
mkdir -m 700 "$STAGE"
mv "$COMPILED_MODEL" "$STAGE/${MODEL_NAME}.mlmodelc"
cp -p "$SNAPSHOT_MANIFEST" "$STAGE/Manifest.json"
cp -p "$SNAPSHOT_LOCK" "$STAGE/${MODEL_NAME}.lock.json"
cp -p "$SNAPSHOT_LICENSE" "$STAGE/SAM-LICENSE.txt"
cmp -s "$SNAPSHOT_MANIFEST" "$STAGE/Manifest.json" || {
  echo "error: staged Manifest.json changed during copy" >&2
  exit 1
}
cmp -s "$SNAPSHOT_LOCK" "$STAGE/${MODEL_NAME}.lock.json" || {
  echo "error: staged model lock changed during copy" >&2
  exit 1
}
cmp -s "$SNAPSHOT_LICENSE" "$STAGE/SAM-LICENSE.txt" || {
  echo "error: staged SAM license changed during copy" >&2
  exit 1
}

echo "==> package temporary AAR"
(
  cd "$STAGE"
  "$XCRUN" ba-package Manifest.json -o "$AAR_CANDIDATE"
)
[[ -f "$AAR_CANDIDATE" && ! -L "$AAR_CANDIDATE" ]] || {
  echo "error: ba-package produced no regular ${PACK_ID}.aar" >&2
  exit 1
}

echo "==> seal AAR + receipt"
"$PYTHON3" "$VERIFIER" seal \
  --lock "$SNAPSHOT_LOCK" \
  --license "$SNAPSHOT_LICENSE" \
  --manifest "$SNAPSHOT_MANIFEST" \
  --extract-directory "$SEAL_EXTRACTED" \
  "$AAR_CANDIDATE" "$RECEIPT_CANDIDATE"

echo "==> publish validated pair"
mkdir -m 700 "$RELEASE_CANDIDATE"
mv "$AAR_CANDIDATE" "$RELEASE_CANDIDATE/${PACK_ID}.aar"
mv "$RECEIPT_CANDIDATE" "$RELEASE_CANDIDATE/${PACK_ID}.aar.receipt.json"
PRESERVE_TRANSACTION=1
if "$PYTHON3" "$VERIFIER" publish \
  --lock "$LOCK_FILE" \
  --license "$LICENSE_FILE" \
  --manifest "$MANIFEST" \
  --extract-directory "$PUBLISH_EXTRACTED" \
  "$RELEASE_CANDIDATE" "$RELEASE_DIRECTORY"; then
  PRESERVE_TRANSACTION=0
else
  publish_status=$?
  if [[ "$publish_status" -eq 1 ]]; then
    # The verifier reserves status 1 for a caught VerificationError, including
    # a rollback whose namespace and parent fsyncs were both proven complete.
    PRESERVE_TRANSACTION=0
  fi
  exit "$publish_status"
fi

echo
echo "PACKAGE_RECEIPT_PASS"
echo "asset pack: $OUT"
echo "receipt:    $RECEIPT_OUT"
echo "next: verify with tools/assetpack/upload.sh after that script's release gate lands"
