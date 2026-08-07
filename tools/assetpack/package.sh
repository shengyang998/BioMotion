#!/usr/bin/env bash
# Build the Apple-Hosted Background Assets pack that carries the SAM 3D Body
# Core ML weights.
#
#   bash tools/assetpack/package.sh [path/to/SAM3DBodyPose.mlpackage]
#
# Output: build/assetpack/sam3d-body-pose.aar
#
# The pack ships a PRE-COMPILED `.mlmodelc`, produced here by `coremlcompiler` —
# the same compiler Xcode invokes on a bundled `.mlpackage`. See the class doc on
# BioMotion/AssetPack/AssetPackModelStore.swift for why it is compiled here
# rather than on device.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PACK_ID="sam3d-body-pose"
MODEL_NAME="SAM3DBodyPose"

DEFAULT_SOURCES=(
  "$REPO_ROOT/../sam-3d-body/export/coreml/${MODEL_NAME}.mlpackage"
  "$REPO_ROOT/BioMotion/CoreML/${MODEL_NAME}.mlpackage"
)

SOURCE="${1:-}"
if [[ -z "$SOURCE" ]]; then
  for candidate in "${DEFAULT_SOURCES[@]}"; do
    if [[ -d "$candidate" ]]; then SOURCE="$candidate"; break; fi
  done
fi
if [[ -z "$SOURCE" || ! -d "$SOURCE" ]]; then
  echo "error: no ${MODEL_NAME}.mlpackage found." >&2
  printf '       looked in:\n' >&2
  printf '         %s\n' "${DEFAULT_SOURCES[@]}" >&2
  echo "       pass one explicitly: bash tools/assetpack/package.sh <path.mlpackage>" >&2
  exit 1
fi

STAGE="$REPO_ROOT/build/assetpack/stage"
OUT="$REPO_ROOT/build/assetpack/${PACK_ID}.aar"

echo "==> source          $SOURCE"
rm -rf "$STAGE"
mkdir -p "$STAGE"

echo "==> compiling       ${MODEL_NAME}.mlpackage -> ${MODEL_NAME}.mlmodelc"
xcrun coremlcompiler compile "$SOURCE" "$STAGE"
[[ -d "$STAGE/${MODEL_NAME}.mlmodelc" ]] || { echo "error: coremlcompiler produced no ${MODEL_NAME}.mlmodelc" >&2; exit 1; }
# Loading needs coremldata.bin; its absence means a broken/partial compile that
# would only surface as a load failure on a user's device.
[[ -f "$STAGE/${MODEL_NAME}.mlmodelc/coremldata.bin" ]] || { echo "error: compiled model has no coremldata.bin" >&2; exit 1; }

cp "$REPO_ROOT/tools/assetpack/Manifest.json" "$STAGE/Manifest.json"

echo "==> packaging       $OUT"
rm -f "$OUT"
# ba-package resolves the manifest's relative fileSelectors against the CWD.
( cd "$STAGE" && xcrun ba-package Manifest.json -o "$OUT" )

echo
echo "compiled model: $(du -sh "$STAGE/${MODEL_NAME}.mlmodelc" | cut -f1)"
echo "asset pack:     $(du -sh "$OUT" | cut -f1)  ->  $OUT"
echo
echo "next: bash tools/assetpack/upload.sh"
