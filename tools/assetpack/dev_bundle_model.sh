#!/usr/bin/env bash
# Toggle the developer-only bundled copy of the pose model.
#
#   bash tools/assetpack/dev_bundle_model.sh on   [path/to/SAM3DBodyPose.mlpackage]
#   bash tools/assetpack/dev_bundle_model.sh off
#
# then `xcodegen generate`.
#
# Why this exists: Background Assets serves no asset packs in the Simulator, and
# only serves them to App Store / TestFlight installs on a real device. A local
# build therefore has no model unless one is bundled. `on` puts the .mlpackage at
# build/DevBundledModel/, which project.yml picks up as an `optional` source
# path, so Xcode compiles it into the app as SAM3DBodyPose.mlmodelc and
# AssetPackModelStore prefers it over the pack.
#
# `on` adds ~1.31 GiB to the app. NEVER archive for TestFlight/App Store with it
# on — that is the exact bloat this whole change removed.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODEL_NAME="SAM3DBodyPose"
# Under build/ because .gitignore already covers it — a 1.3 GiB copy must
# never be committable by accident.
DEST_DIR="$REPO_ROOT/build/DevBundledModel"

MODE="${1:-}"
case "$MODE" in
  on)
    SOURCE="${2:-}"
    if [[ -z "$SOURCE" ]]; then
      for candidate in \
        "$REPO_ROOT/../sam-3d-body/export/coreml/${MODEL_NAME}.mlpackage" \
        "$REPO_ROOT/BioMotion/CoreML/${MODEL_NAME}.mlpackage"
      do
        if [[ -d "$candidate" ]]; then SOURCE="$candidate"; break; fi
      done
    fi
    [[ -n "$SOURCE" && -d "$SOURCE" ]] || { echo "error: no ${MODEL_NAME}.mlpackage found; pass one explicitly." >&2; exit 1; }
    mkdir -p "$DEST_DIR"
    rm -rf "${DEST_DIR:?}/${MODEL_NAME}.mlpackage"
    # A real copy, not a symlink: Xcode's Core ML build rule does not reliably
    # follow a symlinked .mlpackage.
    cp -R "$SOURCE" "$DEST_DIR/${MODEL_NAME}.mlpackage"
    echo "bundled $(du -sh "$DEST_DIR/${MODEL_NAME}.mlpackage" | cut -f1) at $DEST_DIR"
    ;;
  off)
    rm -rf "$DEST_DIR"
    echo "removed $DEST_DIR"
    ;;
  *)
    echo "usage: $0 {on|off} [path/to/${MODEL_NAME}.mlpackage]" >&2
    exit 2
    ;;
esac

echo "now run: xcodegen generate"
