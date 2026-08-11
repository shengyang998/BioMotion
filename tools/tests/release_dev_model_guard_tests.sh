#!/bin/bash
set -euo pipefail
PATH=/usr/bin:/bin:/usr/sbin:/sbin
export PATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
GUARD="$REPO_ROOT/tools/release/reject_dev_model.sh"
TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-model-guard.XXXXXX")"
trap 'rm -r "$TEST_ROOT" 2>/dev/null || true' EXIT

if [ ! -f "$GUARD" ] || [ -L "$GUARD" ]; then
  printf 'developer-model guard is missing or a symlink: %s\n' "$GUARD" >&2
  exit 2
fi

pass_count=0
total_count=0
SIMULATOR_SDK_ROOT="$(/usr/bin/xcrun --sdk iphonesimulator --show-sdk-path)"
SIMULATOR_SDK_NAME="iphonesimulator$(/usr/bin/xcrun --sdk iphonesimulator --show-sdk-version)"
DEVICE_SDK_ROOT="$(/usr/bin/xcrun --sdk iphoneos --show-sdk-path)"
DEVICE_SDK_NAME="iphoneos$(/usr/bin/xcrun --sdk iphoneos --show-sdk-version)"

expect_pass() {
  label=$1
  configuration=$2
  platform=$3
  effective_platform=$4
  sdk_name=$5
  sdk_root=$6
  total_count=$((total_count + 1))
  output="$(CONFIGURATION="$configuration" PLATFORM_NAME="$platform" \
    EFFECTIVE_PLATFORM_NAME="$effective_platform" SDK_NAME="$sdk_name" \
    SDKROOT="$sdk_root" \
    SRCROOT="$TEST_ROOT" TARGET_BUILD_DIR="$TEST_ROOT/products" \
    WRAPPER_NAME='BioMotion.app' \
    /bin/bash "$GUARD" 2>&1)"
  case "$output" in
    *DEVELOPER_MODEL_BUILD_GUARD_PASS*) pass_count=$((pass_count + 1)) ;;
    *)
      printf '%s did not produce the pass sentinel: %s\n' \
        "$label" "$output" >&2
      exit 1
      ;;
  esac
}

expect_failure() {
  label=$1
  configuration=$2
  platform=$3
  effective_platform=$4
  sdk_name=$5
  sdk_root=$6
  expected=$7
  total_count=$((total_count + 1))
  set +e
  output="$(CONFIGURATION="$configuration" PLATFORM_NAME="$platform" \
    EFFECTIVE_PLATFORM_NAME="$effective_platform" SDK_NAME="$sdk_name" \
    SDKROOT="$sdk_root" \
    SRCROOT="$TEST_ROOT" TARGET_BUILD_DIR="$TEST_ROOT/products" \
    WRAPPER_NAME='BioMotion.app' \
    /bin/bash "$GUARD" 2>&1)"
  status=$?
  set -e
  if [ "$status" -eq 0 ]; then
    printf '%s unexpectedly passed\n' "$label" >&2
    exit 1
  fi
  case "$output" in
    *"$expected"*) pass_count=$((pass_count + 1)) ;;
    *)
      printf '%s failed for the wrong reason: %s\n' \
        "$label" "$output" >&2
      exit 1
      ;;
  esac
}

expect_pass release_absent Release iphonesimulator -iphonesimulator \
  "$SIMULATOR_SDK_NAME" "$SIMULATOR_SDK_ROOT"
mkdir -p "$TEST_ROOT/build/DevBundledModel"
expect_pass debug_simulator_present Debug iphonesimulator -iphonesimulator \
  "$SIMULATOR_SDK_NAME" "$SIMULATOR_SDK_ROOT"
expect_failure debug_device_present Debug iphoneos -iphoneos \
  "$DEVICE_SDK_NAME" "$DEVICE_SDK_ROOT" \
  'developer model is enabled outside verified Debug Simulator'
expect_failure polluted_platform_name Debug iphonesimulator -iphoneos \
  "$DEVICE_SDK_NAME" "$DEVICE_SDK_ROOT" \
  'developer model is enabled outside verified Debug Simulator'
expect_failure release_present Release iphonesimulator -iphonesimulator \
  "$SIMULATOR_SDK_NAME" "$SIMULATOR_SDK_ROOT" \
  'developer model is enabled outside verified Debug Simulator'
expect_failure custom_distribution_present AppStore iphoneos -iphoneos \
  "$DEVICE_SDK_NAME" "$DEVICE_SDK_ROOT" \
  'developer model is enabled outside verified Debug Simulator'
rmdir "$TEST_ROOT/build/DevBundledModel"

ln -s missing-model "$TEST_ROOT/build/DevBundledModel"
expect_failure release_dangling_symlink Release iphonesimulator -iphonesimulator \
  "$SIMULATOR_SDK_NAME" "$SIMULATOR_SDK_ROOT" \
  'developer model is enabled outside verified Debug Simulator'
rm "$TEST_ROOT/build/DevBundledModel"

mkdir -p "$TEST_ROOT/products/BioMotion.app/RenamedCompiledModel.mlmodelc"
expect_failure post_copy_product_model Release iphoneos -iphoneos \
  "$DEVICE_SDK_NAME" "$DEVICE_SDK_ROOT" \
  'developer model reached a non-Simulator product'
rm -r "$TEST_ROOT/products"

mkdir -p \
  "$TEST_ROOT/build/DevBundledModel" \
  "$TEST_ROOT/products/BioMotion.app/SAM3DBodyPose.mlmodelc"
printf '%s\n' 'int main(void) { return 0; }' > "$TEST_ROOT/main.c"
/usr/bin/xcrun --sdk iphonesimulator clang \
  -target arm64-apple-ios26.0-simulator \
  -isysroot "$SIMULATOR_SDK_ROOT" \
  "$TEST_ROOT/main.c" -o "$TEST_ROOT/simulator-executable"
/usr/bin/xcrun --sdk iphoneos clang \
  -target arm64-apple-ios26.0 \
  -isysroot "$DEVICE_SDK_ROOT" \
  "$TEST_ROOT/main.c" -o "$TEST_ROOT/device-executable"
cp "$TEST_ROOT/simulator-executable" "$TEST_ROOT/products/BioMotion.app/BioMotion"
/usr/bin/python3 - "$TEST_ROOT/products/BioMotion.app/Info.plist" <<'PY'
from pathlib import Path
import plistlib
import sys

with Path(sys.argv[1]).open("wb") as stream:
    plistlib.dump(
        {
            "CFBundleExecutable": "BioMotion",
            "DTPlatformName": "iphonesimulator",
        },
        stream,
    )
PY
expect_pass debug_simulator_product Debug iphonesimulator -iphonesimulator \
  "$SIMULATOR_SDK_NAME" "$SIMULATOR_SDK_ROOT"
cp "$TEST_ROOT/device-executable" "$TEST_ROOT/products/BioMotion.app/BioMotion"
expect_failure debug_simulator_device_image Debug iphonesimulator -iphonesimulator \
  "$SIMULATOR_SDK_NAME" "$SIMULATOR_SDK_ROOT" \
  'developer model executable is not an iOS Simulator image'
rm -r "$TEST_ROOT/build/DevBundledModel" "$TEST_ROOT/products"

total_count=$((total_count + 1))
set +e
output="$(env -u CONFIGURATION SRCROOT="$TEST_ROOT" PLATFORM_NAME=iphoneos \
  EFFECTIVE_PLATFORM_NAME=-iphoneos SDK_NAME="$DEVICE_SDK_NAME" \
  SDKROOT="$DEVICE_SDK_ROOT" \
  TARGET_BUILD_DIR="$TEST_ROOT/products" WRAPPER_NAME=BioMotion.app \
  /bin/bash "$GUARD" 2>&1)"
status=$?
set -e
if [ "$status" -eq 2 ] && [[ "$output" == *'CONFIGURATION is required'* ]]; then
  pass_count=$((pass_count + 1))
else
  printf 'missing CONFIGURATION failed incorrectly: status=%s output=%s\n' \
    "$status" "$output" >&2
  exit 1
fi

total_count=$((total_count + 1))
set +e
output="$(env -u SRCROOT CONFIGURATION=Release PLATFORM_NAME=iphoneos \
  EFFECTIVE_PLATFORM_NAME=-iphoneos SDK_NAME="$DEVICE_SDK_NAME" \
  SDKROOT="$DEVICE_SDK_ROOT" \
  TARGET_BUILD_DIR="$TEST_ROOT/products" WRAPPER_NAME=BioMotion.app \
  /bin/bash "$GUARD" 2>&1)"
status=$?
set -e
if [ "$status" -eq 2 ] && [[ "$output" == *'SRCROOT is required'* ]]; then
  pass_count=$((pass_count + 1))
else
  printf 'missing SRCROOT failed incorrectly: status=%s output=%s\n' \
    "$status" "$output" >&2
  exit 1
fi

total_count=$((total_count + 1))
set +e
output="$(env -u PLATFORM_NAME CONFIGURATION=Release SRCROOT="$TEST_ROOT" \
  EFFECTIVE_PLATFORM_NAME=-iphoneos SDK_NAME="$DEVICE_SDK_NAME" \
  SDKROOT="$DEVICE_SDK_ROOT" \
  TARGET_BUILD_DIR="$TEST_ROOT/products" WRAPPER_NAME=BioMotion.app \
  /bin/bash "$GUARD" 2>&1)"
status=$?
set -e
if [ "$status" -eq 2 ] && [[ "$output" == *'PLATFORM_NAME is required'* ]]; then
  pass_count=$((pass_count + 1))
else
  printf 'missing PLATFORM_NAME failed incorrectly: status=%s output=%s\n' \
    "$status" "$output" >&2
  exit 1
fi

for missing in EFFECTIVE_PLATFORM_NAME SDK_NAME SDKROOT; do
  total_count=$((total_count + 1))
  set +e
  output="$(CONFIGURATION=Release SRCROOT="$TEST_ROOT" \
    PLATFORM_NAME=iphoneos EFFECTIVE_PLATFORM_NAME=-iphoneos \
    SDK_NAME="$DEVICE_SDK_NAME" SDKROOT="$DEVICE_SDK_ROOT" \
    TARGET_BUILD_DIR="$TEST_ROOT/products" WRAPPER_NAME=BioMotion.app \
    env -u "$missing" /bin/bash "$GUARD" 2>&1)"
  status=$?
  set -e
  if [ "$status" -eq 2 ] && [[ "$output" == *"$missing is required"* ]]; then
    pass_count=$((pass_count + 1))
  else
    printf 'missing %s failed incorrectly: status=%s output=%s\n' \
      "$missing" "$status" "$output" >&2
    exit 1
  fi
done

if [ "$pass_count" -ne "$total_count" ] || [ "$total_count" -ne 16 ]; then
  printf 'developer-model guard suite count mismatch: %s/%s\n' \
    "$pass_count" "$total_count" >&2
  exit 1
fi

printf 'RELEASE_DEV_MODEL_GUARD_TESTS_PASS %s/%s\n' \
  "$pass_count" "$total_count"
