#!/bin/bash
set -euo pipefail
PATH=/usr/bin:/bin:/usr/sbin:/sbin
export PATH

if [ -z "${CONFIGURATION+x}" ] || [ -z "$CONFIGURATION" ]; then
  printf '%s\n' 'CONFIGURATION is required for the developer-model build guard' >&2
  exit 2
fi
if [ -z "${SRCROOT+x}" ] || [ -z "$SRCROOT" ]; then
  printf '%s\n' 'SRCROOT is required for the developer-model build guard' >&2
  exit 2
fi
if [ -z "${PLATFORM_NAME+x}" ] || [ -z "$PLATFORM_NAME" ]; then
  printf '%s\n' 'PLATFORM_NAME is required for the developer-model build guard' >&2
  exit 2
fi
if [ -z "${EFFECTIVE_PLATFORM_NAME+x}" ] || [ -z "$EFFECTIVE_PLATFORM_NAME" ]; then
  printf '%s\n' 'EFFECTIVE_PLATFORM_NAME is required for the developer-model build guard' >&2
  exit 2
fi
if [ -z "${SDK_NAME+x}" ] || [ -z "$SDK_NAME" ]; then
  printf '%s\n' 'SDK_NAME is required for the developer-model build guard' >&2
  exit 2
fi
if [ -z "${SDKROOT+x}" ] || [ -z "$SDKROOT" ]; then
  printf '%s\n' 'SDKROOT is required for the developer-model build guard' >&2
  exit 2
fi
if [ -z "${TARGET_BUILD_DIR+x}" ] || [ -z "$TARGET_BUILD_DIR" ]; then
  printf '%s\n' 'TARGET_BUILD_DIR is required for the developer-model build guard' >&2
  exit 2
fi
if [ -z "${WRAPPER_NAME+x}" ] || [ -z "$WRAPPER_NAME" ]; then
  printf '%s\n' 'WRAPPER_NAME is required for the developer-model build guard' >&2
  exit 2
fi

DEVELOPER_MODEL="$SRCROOT/build/DevBundledModel"
BUILT_PRODUCT="$TARGET_BUILD_DIR/$WRAPPER_NAME"
EXPECTED_SIMULATOR_SDK="$(/usr/bin/xcrun --sdk iphonesimulator --show-sdk-path)"
SDKROOT_CANONICAL=''
EXPECTED_SIMULATOR_SDK_CANONICAL=''
if [ -d "$SDKROOT" ]; then
  SDKROOT_CANONICAL="$(cd "$SDKROOT" && /bin/pwd -P)"
fi
if [ -d "$EXPECTED_SIMULATOR_SDK" ]; then
  EXPECTED_SIMULATOR_SDK_CANONICAL="$({
    cd "$EXPECTED_SIMULATOR_SDK" && /bin/pwd -P
  })"
fi

ALLOW_DEVELOPER_MODEL=0
if [ "$CONFIGURATION" = 'Debug' ] \
  && [ "$PLATFORM_NAME" = 'iphonesimulator' ] \
  && [ "$EFFECTIVE_PLATFORM_NAME" = '-iphonesimulator' ] \
  && [[ "$SDK_NAME" == iphonesimulator* ]] \
  && [ -n "$SDKROOT_CANONICAL" ] \
  && [ "$SDKROOT_CANONICAL" = "$EXPECTED_SIMULATOR_SDK_CANONICAL" ]; then
  ALLOW_DEVELOPER_MODEL=1
fi

if [ "$ALLOW_DEVELOPER_MODEL" -eq 1 ] \
  && { [ -e "$DEVELOPER_MODEL" ] || [ -L "$DEVELOPER_MODEL" ]; } \
  && { [ ! -d "$DEVELOPER_MODEL" ] || [ -L "$DEVELOPER_MODEL" ]; }; then
  printf 'developer model source is not a regular directory: %s\n' \
    "$DEVELOPER_MODEL" >&2
  exit 1
fi

BUNDLED_MODEL=''
if [ -e "$BUILT_PRODUCT" ] || [ -L "$BUILT_PRODUCT" ]; then
  if [ ! -d "$BUILT_PRODUCT" ] || [ -L "$BUILT_PRODUCT" ]; then
    printf 'built product is not a regular app directory: %s\n' \
      "$BUILT_PRODUCT" >&2
    exit 1
  fi
  BUNDLED_MODEL="$(
    /usr/bin/find "$BUILT_PRODUCT" \
      \( -iname '*.mlmodel' -o -iname '*.mlpackage' -o -iname '*.mlmodelc' \) \
      -print -quit
  )"
fi

# The optional source is available only to Debug Simulator iteration. `-L`
# also catches a dangling symlink, which `-e` alone misses. The same script is
# run before compilation and after resources: the latter scans an existing
# product and closes the source-check/copy interval. PLATFORM_NAME alone is not
# provenance: xcodebuild lets a caller contradict it while retaining an
# iphoneos SDK. The effective platform, SDK name, and canonical SDKROOT must all
# identify the active Simulator SDK.
if [ "$ALLOW_DEVELOPER_MODEL" -ne 1 ]; then
  if [ -e "$DEVELOPER_MODEL" ] || [ -L "$DEVELOPER_MODEL" ]; then
    printf '%s\n' \
      'developer model is enabled outside verified Debug Simulator; run tools/assetpack/dev_bundle_model.sh off and regenerate the project' >&2
    exit 1
  fi

  if [ -n "$BUNDLED_MODEL" ]; then
    printf 'developer model reached a non-Simulator product: %s\n' \
      "$BUNDLED_MODEL" >&2
    exit 1
  fi
elif [ -n "$BUNDLED_MODEL" ]; then
  PRODUCT_INFO="$BUILT_PRODUCT/Info.plist"
  if [ ! -f "$PRODUCT_INFO" ] || [ -L "$PRODUCT_INFO" ]; then
    printf 'developer-model product has no regular Info.plist: %s\n' \
      "$PRODUCT_INFO" >&2
    exit 1
  fi
  PRODUCT_PLATFORM="$(
    /usr/libexec/PlistBuddy -c 'Print :DTPlatformName' "$PRODUCT_INFO" 2>/dev/null \
      || true
  )"
  PRODUCT_EXECUTABLE_NAME="$(
    /usr/libexec/PlistBuddy -c 'Print :CFBundleExecutable' "$PRODUCT_INFO" 2>/dev/null \
      || true
  )"
  PRODUCT_EXECUTABLE="$BUILT_PRODUCT/$PRODUCT_EXECUTABLE_NAME"
  if [ "$PRODUCT_PLATFORM" != 'iphonesimulator' ] \
    || [ -z "$PRODUCT_EXECUTABLE_NAME" ] \
    || [ ! -f "$PRODUCT_EXECUTABLE" ] \
    || [ -L "$PRODUCT_EXECUTABLE" ]; then
    printf 'developer model reached a product not proven to be Simulator: %s\n' \
      "$BUILT_PRODUCT" >&2
    exit 1
  fi
  BUILD_RECEIPT="$(
    /usr/bin/xcrun vtool -show-build "$PRODUCT_EXECUTABLE" 2>/dev/null || true
  )"
  PLATFORM_LINES="$(
    printf '%s\n' "$BUILD_RECEIPT" \
      | /usr/bin/grep -Ec '^[[:space:]]*platform[[:space:]]+[A-Za-z0-9]+[[:space:]]*$' \
      || true
  )"
  SIMULATOR_LINES="$(
    printf '%s\n' "$BUILD_RECEIPT" \
      | /usr/bin/grep -Ec '^[[:space:]]*platform[[:space:]]+IOSSIMULATOR[[:space:]]*$' \
      || true
  )"
  if [ "$PLATFORM_LINES" -ne 1 ] || [ "$SIMULATOR_LINES" -ne 1 ]; then
    printf 'developer model executable is not an iOS Simulator image: %s\n' \
      "$PRODUCT_EXECUTABLE" >&2
    exit 1
  fi
fi

printf '%s\n' 'DEVELOPER_MODEL_BUILD_GUARD_PASS'
