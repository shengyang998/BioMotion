#!/usr/bin/env bash
# Upload the asset pack to App Store Connect. This is a SEPARATE channel from
# the app binary: the pack is uploaded on its own and App Store Connect
# associates it with the app record by Apple ID.
#
#   bash tools/assetpack/upload.sh [path/to/pack.aar]
#
# Requires an App Store Connect **API key**. An app-specific password does NOT
# work here — altool returns 401 for --upload-asset-pack.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# BioMotion's App Store Connect app id (numeric "Apple ID", not the bundle id).
APPLE_ID="${BIOMOTION_ASC_APP_ID:-6761994383}"
# ASC API key. The .p8 must live at ~/.appstoreconnect/private_keys/AuthKey_<id>.p8
API_KEY_ID="${ASC_API_KEY_ID:-4KH2G3HUYG}"
API_ISSUER="${ASC_API_ISSUER:-25194e91-5f40-43b4-b598-98a189994f54}"

AAR="${1:-$REPO_ROOT/build/assetpack/sam3d-body-pose.aar}"
if [[ ! -f "$AAR" ]]; then
  echo "error: $AAR not found — run tools/assetpack/package.sh first." >&2
  exit 1
fi

KEY_PATH="$HOME/.appstoreconnect/private_keys/AuthKey_${API_KEY_ID}.p8"
if [[ ! -f "$KEY_PATH" ]]; then
  echo "error: ASC API key not found at $KEY_PATH" >&2
  exit 1
fi

echo "==> uploading $(du -sh "$AAR" | cut -f1) to app $APPLE_ID"
xcrun altool --upload-asset-pack "$AAR" \
  --apple-id "$APPLE_ID" \
  --platform ios \
  --apiKey "$API_KEY_ID" \
  --apiIssuer "$API_ISSUER"

echo
echo "==> versions now on App Store Connect"
xcrun altool --list-asset-pack-versions \
  --apple-id "$APPLE_ID" \
  --asset-pack-identifier sam3d-body-pose \
  --apiKey "$API_KEY_ID" \
  --apiIssuer "$API_ISSUER"
