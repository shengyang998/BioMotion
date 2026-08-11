#!/bin/bash
set -euo pipefail
PATH=/usr/bin:/bin:/usr/sbin:/sbin
export PATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-resource-tests.XXXXXX")"
FIXTURE_ROOT="$TEST_ROOT/repository"
APP_BUNDLE="$TEST_ROOT/BioMotion.app"
TEST_BUNDLE="$TEST_ROOT/BioMotionTests.xctest"
ARCHIVE="$TEST_ROOT/BioMotion.xcarchive"
SIMULATOR_SDK="$(/usr/bin/xcrun --sdk iphonesimulator --show-sdk-path)"
IPHONEOS_SDK="$(/usr/bin/xcrun --sdk iphoneos --show-sdk-path)"
MACOS_SDK="$(/usr/bin/xcrun --sdk macosx --show-sdk-path)"
trap 'rm -r "$TEST_ROOT" 2>/dev/null || true' EXIT

mkdir -p \
  "$FIXTURE_ROOT/tools/tests" \
  "$FIXTURE_ROOT/tools/release" \
  "$FIXTURE_ROOT/BioMotion.xcodeproj" \
  "$FIXTURE_ROOT/BioMotion/Resources" \
  "$FIXTURE_ROOT/BioMotionTests"
cp "$REPO_ROOT/tools/tests/app_resource_boundary_probe.sh" \
  "$FIXTURE_ROOT/tools/tests/app_resource_boundary_probe.sh"
cp "$REPO_ROOT/tools/release/resource_boundary.py" \
  "$FIXTURE_ROOT/tools/release/resource_boundary.py"
cp "$REPO_ROOT/tools/release/reject_dev_model.sh" \
  "$FIXTURE_ROOT/tools/release/reject_dev_model.sh"
cp "$REPO_ROOT/tools/release/ExportOptions-TestFlight.plist" \
  "$FIXTURE_ROOT/tools/release/ExportOptions-TestFlight.plist"
cp "$REPO_ROOT/project.yml" "$FIXTURE_ROOT/project.yml"
cp "$REPO_ROOT/BioMotion.xcodeproj/project.pbxproj" \
  "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj"
cp "$REPO_ROOT/LICENSE" "$FIXTURE_ROOT/LICENSE"
cp "$REPO_ROOT/NOTICE" "$FIXTURE_ROOT/NOTICE"
cp "$REPO_ROOT/BioMotion/PrivacyInfo.xcprivacy" \
  "$FIXTURE_ROOT/BioMotion/PrivacyInfo.xcprivacy"
cp -R "$REPO_ROOT/BioMotion/Assets.xcassets" \
  "$FIXTURE_ROOT/BioMotion/Assets.xcassets"
cp -R "$REPO_ROOT/BioMotionTests/Fixtures" \
  "$FIXTURE_ROOT/BioMotionTests/Fixtures"
for resource in \
  FullBody.osim \
  Rajagopal2016.osim \
  SAM-LICENSE.txt \
  SAM3DBodyPose.lock.json \
  THIRD-PARTY-NOTICES.txt; do
  cp "$REPO_ROOT/BioMotion/Resources/$resource" \
    "$FIXTURE_ROOT/BioMotion/Resources/$resource"
done

cp "$FIXTURE_ROOT/project.yml" "$TEST_ROOT/project.original.yml"
cp "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" \
  "$TEST_ROOT/project.original.pbxproj"
cp "$FIXTURE_ROOT/tools/release/reject_dev_model.sh" \
  "$TEST_ROOT/guard.original.sh"
cp "$FIXTURE_ROOT/tools/release/ExportOptions-TestFlight.plist" \
  "$TEST_ROOT/export-options.original.plist"
cp "$FIXTURE_ROOT/BioMotion/Resources/FullBody.osim" \
  "$TEST_ROOT/FullBody.original.osim"
cp "$FIXTURE_ROOT/BioMotion/Resources/THIRD-PARTY-NOTICES.txt" \
  "$TEST_ROOT/third-party.original.txt"
cp "$FIXTURE_ROOT/BioMotion/Assets.xcassets/AppIcon.appiconset/icon_60x60.png" \
  "$TEST_ROOT/icon.original.png"

pass_count=0
total_count=0

run_probe() {
  (cd "$FIXTURE_ROOT" \
    && /bin/bash tools/tests/app_resource_boundary_probe.sh "$@")
}

expect_pass() {
  label=$1
  shift
  total_count=$((total_count + 1))
  output="$(run_probe "$@" 2>&1)"
  case "$output" in
    *APP_RESOURCE_BOUNDARY_PROBE_PASS*) pass_count=$((pass_count + 1)) ;;
    *)
      printf '%s did not produce the pass sentinel: %s\n' \
        "$label" "$output" >&2
      exit 1
      ;;
  esac
}

expect_failure() {
  label=$1
  expected=$2
  shift 2
  total_count=$((total_count + 1))
  set +e
  output="$(run_probe "$@" 2>&1)"
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

run_extractor() {
  ipa=$1
  extraction_root="$(mktemp -d "$TEST_ROOT/extracted.XXXXXX")"
  /usr/bin/python3 - \
    "$FIXTURE_ROOT/tools/release/resource_boundary.py" \
    "$ipa" \
    "$extraction_root" <<'PY'
import importlib.util
from pathlib import Path
import stat
import sys

module_path = Path(sys.argv[1])
spec = importlib.util.spec_from_file_location("resource_boundary_fixture", module_path)
if spec is None or spec.loader is None:
    raise SystemExit("resource-boundary fixture module could not be loaded")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
app = module.extract_release_ipa(Path(sys.argv[2]), Path(sys.argv[3]))
app_executable = app / "BioMotion"
extension_executable = (
    app / "Extensions/AssetPackDownloader.appex/AssetPackDownloader"
)
info = app / "Info.plist"
if not app_executable.read_bytes() == b"app-code":
    raise SystemExit("normal IPA app executable changed")
if not extension_executable.read_bytes() == b"extension-code":
    raise SystemExit("normal IPA extension executable changed")
if not app_executable.stat().st_mode & 0o111:
    raise SystemExit("normal IPA app executable lost its execute bit")
if not extension_executable.stat().st_mode & 0o111:
    raise SystemExit("normal IPA extension executable lost its execute bit")
if info.stat().st_mode & 0o111:
    raise SystemExit("normal IPA resource became executable")
print("RAW_IPA_EXTRACTOR_PASS")
PY
  status=$?
  rm -r "$extraction_root"
  return "$status"
}

expect_extractor_pass() {
  label=$1
  ipa=$2
  total_count=$((total_count + 1))
  output="$(run_extractor "$ipa" 2>&1)"
  case "$output" in
    *RAW_IPA_EXTRACTOR_PASS*) pass_count=$((pass_count + 1)) ;;
    *)
      printf '%s did not produce the extractor pass sentinel: %s\n' \
        "$label" "$output" >&2
      exit 1
      ;;
  esac
}

expect_extractor_failure() {
  label=$1
  expected=$2
  ipa=$3
  total_count=$((total_count + 1))
  set +e
  output="$(run_extractor "$ipa" 2>&1)"
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

expect_pass source_baseline

/usr/bin/plutil -replace destination -string upload \
  "$FIXTURE_ROOT/tools/release/ExportOptions-TestFlight.plist"
expect_failure direct_upload_export_options \
  'TestFlight export options changed'
cp "$TEST_ROOT/export-options.original.plist" \
  "$FIXTURE_ROOT/tools/release/ExportOptions-TestFlight.plist"

/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
needle = "CURRENT_PROJECT_VERSION = 30;"
if text.count(needle) != 4:
    raise SystemExit("fixture generated build number inventory changed")
path.write_text(text.replace(needle, "CURRENT_PROJECT_VERSION = 29;", 1), encoding="utf-8")
PY
expect_failure stale_generated_build_number \
  'generated project version is stale'
cp "$TEST_ROOT/project.original.pbxproj" \
  "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj"

/usr/bin/python3 - "$FIXTURE_ROOT/project.yml" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
line = '          - "Resources/**"\n'
if text.count(line) != 1:
    raise SystemExit("fixture resource exclusion changed")
path.write_text(text.replace(line, "", 1), encoding="utf-8")
PY
expect_failure missing_resource_exclusion \
  'BioMotion broad source scan exclusion contract changed'
cp "$TEST_ROOT/project.original.yml" "$FIXTURE_ROOT/project.yml"

/usr/bin/python3 - "$FIXTURE_ROOT/project.yml" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
needle = (
    "      - path: BioMotionTests/Fixtures\n"
    "        type: folder\n"
    "        buildPhase: resources\n"
)
replacement = needle + (
    "      - path: BioMotion/Resources\n"
    "        type: folder\n"
    "        buildPhase: resources\n"
)
if text.count(needle) != 1:
    raise SystemExit("fixture test resource block changed")
path.write_text(text.replace(needle, replacement, 1), encoding="utf-8")
PY
expect_failure whole_test_resource_folder \
  'BioMotionTests must not copy the whole Resources folder'
cp "$TEST_ROOT/project.original.yml" "$FIXTURE_ROOT/project.yml"

/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
notice = re.search(
    r"^\s*([A-F0-9]{24}) /\* NOTICE in Resources \*/ = "
    r"\{isa = PBXBuildFile;",
    text,
    flags=re.MULTILINE,
)
if notice is None:
    raise SystemExit("fixture NOTICE build file changed")
notice_id = notice.group(1)
phases = list(re.finditer(
    r"^\t\t[A-F0-9]{24} /\* Resources \*/ = \{\n"
    r"\t\t\tisa = PBXResourcesBuildPhase;.*?^\t\t\};",
    text,
    flags=re.MULTILINE | re.DOTALL,
))
app_phase = next((match for match in phases if "Assets.xcassets" in match.group(0)), None)
test_phase = next((match for match in phases if "Fixtures in Resources" in match.group(0)), None)
if app_phase is None or test_phase is None:
    raise SystemExit("fixture resource phases changed")
notice_line = re.search(
    rf"^\s+{notice_id} /\* NOTICE in Resources \*/,\n",
    app_phase.group(0),
    flags=re.MULTILINE,
)
if notice_line is None:
    raise SystemExit("fixture app NOTICE membership changed")
app_replacement = app_phase.group(0).replace(notice_line.group(0), "", 1)
test_replacement = test_phase.group(0).replace(
    "\t\t\t);\n",
    f"\t\t\t\t{notice_id} /* NOTICE in Resources */,\n\t\t\t);\n",
    1,
)
for match, replacement in sorted(
    ((app_phase, app_replacement), (test_phase, test_replacement)),
    key=lambda item: item[0].start(),
    reverse=True,
):
    text = text[:match.start()] + replacement + text[match.end():]
path.write_text(text, encoding="utf-8")
PY
expect_failure notice_wrong_target \
  'generated BioMotion resource allowlist changed'
cp "$TEST_ROOT/project.original.pbxproj" \
  "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj"

printf '%s\n' '# stale after XcodeGen' \
  >> "$FIXTURE_ROOT/tools/release/reject_dev_model.sh"
expect_failure stale_generated_guard \
  'generated developer-model guard phase inventory changed'
cp "$TEST_ROOT/guard.original.sh" \
  "$FIXTURE_ROOT/tools/release/reject_dev_model.sh"

printf '%s' x >> "$FIXTURE_ROOT/BioMotion/Resources/FullBody.osim"
expect_failure changed_source_model 'source model FullBody.osim identity changed'
cp "$TEST_ROOT/FullBody.original.osim" \
  "$FIXTURE_ROOT/BioMotion/Resources/FullBody.osim"

mkdir "$FIXTURE_ROOT/BioMotion/Resources/Nested"
: > "$FIXTURE_ROOT/BioMotion/Resources/Nested/Unreviewed.OSIM"
expect_failure nested_uppercase_source_model 'source model allowlist changed'
rm "$FIXTURE_ROOT/BioMotion/Resources/Nested/Unreviewed.OSIM"
rmdir "$FIXTURE_ROOT/BioMotion/Resources/Nested"

mkdir "$FIXTURE_ROOT/BioMotion/Assets.xcassets/Weights.dataset"
: > "$FIXTURE_ROOT/BioMotion/Assets.xcassets/Weights.dataset/weights.bin"
expect_failure asset_catalog_dataset 'asset catalog directory allowlist changed'
rm "$FIXTURE_ROOT/BioMotion/Assets.xcassets/Weights.dataset/weights.bin"
rmdir "$FIXTURE_ROOT/BioMotion/Assets.xcassets/Weights.dataset"

printf '%s' x \
  >> "$FIXTURE_ROOT/BioMotion/Assets.xcassets/AppIcon.appiconset/icon_60x60.png"
expect_failure changed_asset_icon 'asset catalog entry AppIcon.appiconset/icon_60x60.png identity changed'
cp "$TEST_ROOT/icon.original.png" \
  "$FIXTURE_ROOT/BioMotion/Assets.xcassets/AppIcon.appiconset/icon_60x60.png"

printf '%s' x \
  >> "$FIXTURE_ROOT/BioMotion/Resources/THIRD-PARTY-NOTICES.txt"
expect_failure changed_legal_notice \
  'source text BioMotion/Resources/THIRD-PARTY-NOTICES.txt identity changed'
cp "$TEST_ROOT/third-party.original.txt" \
  "$FIXTURE_ROOT/BioMotion/Resources/THIRD-PARTY-NOTICES.txt"

mkdir -p "$APP_BUNDLE/Extensions/AssetPackDownloader.appex"
/usr/bin/xcrun actool \
  --compile "$APP_BUNDLE" \
  --platform iphonesimulator \
  --minimum-deployment-target 26.0 \
  --app-icon AppIcon \
  --output-partial-info-plist "$TEST_ROOT/asset-partial.plist" \
  "$FIXTURE_ROOT/BioMotion/Assets.xcassets" >/dev/null
printf '%s\n' 'int main(void) { return 0; }' > "$TEST_ROOT/main.c"
printf '%s\n' 'int biomotion_test_fixture(void) { return 0; }' \
  > "$TEST_ROOT/test-bundle.c"
/usr/bin/xcrun --sdk iphonesimulator clang \
  -target arm64-apple-ios26.0-simulator \
  -isysroot "$SIMULATOR_SDK" \
  "$TEST_ROOT/main.c" -o "$TEST_ROOT/simulator-executable"
/usr/bin/xcrun --sdk iphonesimulator clang -bundle \
  -target arm64-apple-ios26.0-simulator \
  -isysroot "$SIMULATOR_SDK" \
  "$TEST_ROOT/test-bundle.c" -o "$TEST_ROOT/simulator-test-bundle"
/usr/bin/xcrun --sdk macosx clang \
  -target arm64-apple-macos14.0 \
  -isysroot "$MACOS_SDK" \
  "$TEST_ROOT/main.c" -o "$TEST_ROOT/macos-executable"
/usr/bin/xcrun --sdk macosx clang -bundle \
  -target arm64-apple-macos14.0 \
  -isysroot "$MACOS_SDK" \
  "$TEST_ROOT/test-bundle.c" -o "$TEST_ROOT/macos-test-bundle"
cp "$TEST_ROOT/simulator-executable" "$APP_BUNDLE/BioMotion"
cp "$TEST_ROOT/simulator-executable" \
  "$APP_BUNDLE/Extensions/AssetPackDownloader.appex/AssetPackDownloader"
cp "$FIXTURE_ROOT/BioMotion/Resources/FullBody.osim" "$APP_BUNDLE/FullBody.osim"
cp "$FIXTURE_ROOT/BioMotion/Resources/Rajagopal2016.osim" \
  "$APP_BUNDLE/Rajagopal2016.osim"
cp "$FIXTURE_ROOT/BioMotion/Resources/THIRD-PARTY-NOTICES.txt" \
  "$APP_BUNDLE/THIRD-PARTY-NOTICES.txt"
cp "$FIXTURE_ROOT/BioMotion/PrivacyInfo.xcprivacy" \
  "$APP_BUNDLE/PrivacyInfo.xcprivacy"
cp "$FIXTURE_ROOT/NOTICE" "$APP_BUNDLE/NOTICE"
printf '%s' 'APPL????' > "$APP_BUNDLE/PkgInfo"

/usr/bin/python3 - "$APP_BUNDLE" iphonesimulator <<'PY'
from pathlib import Path
import plistlib
import sys

app = Path(sys.argv[1])
platform = sys.argv[2]
supported = "iPhoneSimulator" if platform == "iphonesimulator" else "iPhoneOS"
common = {
    "CFBundleSupportedPlatforms": [supported],
    "DTPlatformName": platform,
    "DTSDKName": f"{platform}26.4",
    "MinimumOSVersion": "26.0",
}
icons = {
    "CFBundlePrimaryIcon": {
        "CFBundleIconFiles": ["AppIcon60x60"],
        "CFBundleIconName": "AppIcon",
    }
}
ipad_icons = {
    "CFBundlePrimaryIcon": {
        "CFBundleIconFiles": ["AppIcon60x60", "AppIcon76x76"],
        "CFBundleIconName": "AppIcon",
    }
}
app_info = {
    **common,
    "CFBundleExecutable": "BioMotion",
    "CFBundleIdentifier": "com.soleil.BioMotion",
    "CFBundleName": "BioMotion",
    "CFBundlePackageType": "APPL",
    "CFBundleShortVersionString": "1.0.0",
    "CFBundleVersion": "30",
    "CFBundleIcons": icons,
    "CFBundleIcons~ipad": ipad_icons,
}
extension_info = {
    **common,
    "CFBundleExecutable": "AssetPackDownloader",
    "CFBundleIdentifier": "com.soleil.BioMotion.AssetPackDownloader",
    "CFBundleName": "AssetPackDownloader",
    "CFBundlePackageType": "XPC!",
    "CFBundleShortVersionString": "1.0.0",
    "CFBundleVersion": "30",
}
with (app / "Info.plist").open("wb") as stream:
    plistlib.dump(app_info, stream)
with (app / "Extensions/AssetPackDownloader.appex/Info.plist").open("wb") as stream:
    plistlib.dump(extension_info, stream)
PY

cp "$APP_BUNDLE/Assets.car" "$TEST_ROOT/Assets.original.car"
expect_pass simulator_app_baseline --simulator-smoke "$APP_BUNDLE"

cp "$APP_BUNDLE/BioMotion" "$TEST_ROOT/simulator-app-executable.original"
cp "$TEST_ROOT/macos-executable" "$APP_BUNDLE/BioMotion"
expect_failure simulator_app_macos_image 'app executable has the wrong Mach-O platform' \
  --simulator-smoke "$APP_BUNDLE"
cp "$TEST_ROOT/simulator-app-executable.original" "$APP_BUNDLE/BioMotion"

cp "$APP_BUNDLE/Rajagopal2016.osim" "$APP_BUNDLE/Unexpected.osim"
expect_failure unexpected_app_osim 'built app file allowlist changed' \
  --simulator-smoke "$APP_BUNDLE"
rm "$APP_BUNDLE/Unexpected.osim"

cp "$APP_BUNDLE/FullBody.osim" "$APP_BUNDLE/test_fixture_weights.bin"
expect_failure unexpected_app_binary 'built app file allowlist changed' \
  --simulator-smoke "$APP_BUNDLE"
rm "$APP_BUNDLE/test_fixture_weights.bin"

mkdir "$APP_BUNDLE/RenamedCompiledModel"
: > "$APP_BUNDLE/RenamedCompiledModel/coremldata.bin"
expect_failure renamed_model_directory 'built app file allowlist changed' \
  --simulator-smoke "$APP_BUNDLE"
rm "$APP_BUNDLE/RenamedCompiledModel/coremldata.bin"
rmdir "$APP_BUNDLE/RenamedCompiledModel"

mv "$APP_BUNDLE/NOTICE" "$TEST_ROOT/NOTICE.held"
expect_failure missing_bundled_notice 'built app file allowlist changed' \
  --simulator-smoke "$APP_BUNDLE"
mv "$TEST_ROOT/NOTICE.held" "$APP_BUNDLE/NOTICE"

printf '%s' x >> "$APP_BUNDLE/FullBody.osim"
expect_failure changed_bundled_model \
  'built app model differs from reviewed source: FullBody.osim' \
  --simulator-smoke "$APP_BUNDLE"
cp "$FIXTURE_ROOT/BioMotion/Resources/FullBody.osim" "$APP_BUNDLE/FullBody.osim"

cp "$FIXTURE_ROOT/BioMotion/Resources/SAM3DBodyPose.lock.json" \
  "$APP_BUNDLE/SAM3DBodyPose.lock.json"
expect_failure bundled_sam_authority 'built app file allowlist changed' \
  --simulator-smoke "$APP_BUNDLE"
rm "$APP_BUNDLE/SAM3DBodyPose.lock.json"

mkdir "$APP_BUNDLE/SAM3DBodyPose.mlmodelc"
: > "$APP_BUNDLE/SAM3DBodyPose.mlmodelc/coremldata.bin"
expect_failure bundled_core_ml 'built app file allowlist changed' \
  --simulator-smoke "$APP_BUNDLE"
rm "$APP_BUNDLE/SAM3DBodyPose.mlmodelc/coremldata.bin"
rmdir "$APP_BUNDLE/SAM3DBodyPose.mlmodelc"

rm "$APP_BUNDLE/Rajagopal2016.osim"
ln -s "$FIXTURE_ROOT/BioMotion/Resources/Rajagopal2016.osim" \
  "$APP_BUNDLE/Rajagopal2016.osim"
expect_failure symlinked_bundled_model 'built bundle contains a symlink or special file' \
  --simulator-smoke "$APP_BUNDLE"
rm "$APP_BUNDLE/Rajagopal2016.osim"
cp "$FIXTURE_ROOT/BioMotion/Resources/Rajagopal2016.osim" \
  "$APP_BUNDLE/Rajagopal2016.osim"

cp "$APP_BUNDLE/FullBody.osim" "$APP_BUNDLE/Assets.car"
expect_failure oversized_asset_car \
  'compiled asset catalog exceeds the reviewed size budget' \
  --simulator-smoke "$APP_BUNDLE"
cp "$TEST_ROOT/Assets.original.car" "$APP_BUNDLE/Assets.car"

cp "$APP_BUNDLE/AppIcon60x60@2x.png" "$APP_BUNDLE/AppIcon60x60@3x.png"
expect_failure extra_app_icon 'built app file allowlist changed' \
  --simulator-smoke "$APP_BUNDLE"
rm "$APP_BUNDLE/AppIcon60x60@3x.png"

mkdir -p "$TEST_BUNDLE/Fixtures"
cp "$TEST_ROOT/simulator-test-bundle" "$TEST_BUNDLE/BioMotionTests"
cp "$FIXTURE_ROOT/BioMotion/Resources/FullBody.osim" \
  "$TEST_BUNDLE/FullBody.osim"
cp "$FIXTURE_ROOT/BioMotion/Resources/Rajagopal2016.osim" \
  "$TEST_BUNDLE/Rajagopal2016.osim"
cp "$FIXTURE_ROOT/BioMotionTests/Fixtures/"* "$TEST_BUNDLE/Fixtures/"
/usr/bin/python3 - "$TEST_BUNDLE/Info.plist" <<'PY'
from pathlib import Path
import plistlib
import sys

info = {
    "CFBundleExecutable": "BioMotionTests",
    "CFBundleIdentifier": "com.soleil.BioMotionTests",
    "CFBundleName": "BioMotionTests",
    "CFBundlePackageType": "BNDL",
    "CFBundleSupportedPlatforms": ["iPhoneSimulator"],
    "DTPlatformName": "iphonesimulator",
    "DTSDKName": "iphonesimulator26.4",
    "MinimumOSVersion": "26.0",
}
with Path(sys.argv[1]).open("wb") as stream:
    plistlib.dump(info, stream)
PY
expect_pass tests_bundle_baseline --tests-bundle-smoke "$TEST_BUNDLE"

cp "$TEST_ROOT/macos-test-bundle" "$TEST_BUNDLE/BioMotionTests"
expect_failure tests_bundle_macos_image \
  'test executable has the wrong Mach-O platform' \
  --tests-bundle-smoke "$TEST_BUNDLE"
cp "$TEST_ROOT/simulator-test-bundle" "$TEST_BUNDLE/BioMotionTests"

cp "$FIXTURE_ROOT/BioMotion/Resources/SAM-LICENSE.txt" \
  "$TEST_BUNDLE/SAM-LICENSE.txt"
expect_failure extra_test_resource 'built test file allowlist changed' \
  --tests-bundle-smoke "$TEST_BUNDLE"
rm "$TEST_BUNDLE/SAM-LICENSE.txt"

printf '%s' x >> "$TEST_BUNDLE/Fixtures/mhr_root_coreml.txt"
expect_failure changed_test_fixture \
  'built test fixture differs from reviewed source: mhr_root_coreml.txt' \
  --tests-bundle-smoke "$TEST_BUNDLE"
cp "$FIXTURE_ROOT/BioMotionTests/Fixtures/mhr_root_coreml.txt" \
  "$TEST_BUNDLE/Fixtures/mhr_root_coreml.txt"

mkdir -p "$ARCHIVE/Products/Applications"
cp -R "$APP_BUNDLE" "$ARCHIVE/Products/Applications/BioMotion.app"
ARCHIVE_APP="$ARCHIVE/Products/Applications/BioMotion.app"
printf '%s\n' 'int main(void) { return 0; }' > "$TEST_ROOT/main.c"
/usr/bin/xcrun --sdk iphoneos clang \
  -target arm64-apple-ios26.0 \
  -isysroot "$IPHONEOS_SDK" \
  "$TEST_ROOT/main.c" \
  -o "$ARCHIVE_APP/BioMotion"
cp "$ARCHIVE_APP/BioMotion" \
  "$ARCHIVE_APP/Extensions/AssetPackDownloader.appex/AssetPackDownloader"
/usr/bin/python3 - "$ARCHIVE" <<'PY'
from pathlib import Path
import plistlib
import sys

archive = Path(sys.argv[1])
app = archive / "Products/Applications/BioMotion.app"
for relative in ("Info.plist", "Extensions/AssetPackDownloader.appex/Info.plist"):
    path = app / relative
    with path.open("rb") as stream:
        info = plistlib.load(stream)
    info["CFBundleSupportedPlatforms"] = ["iPhoneOS"]
    info["DTPlatformName"] = "iphoneos"
    info["DTSDKName"] = "iphoneos26.4"
    with path.open("wb") as stream:
        plistlib.dump(info, stream)
archive_info = {
    "ApplicationProperties": {
        "ApplicationPath": "Applications/BioMotion.app",
        "Architectures": ["arm64"],
        "CFBundleIdentifier": "com.soleil.BioMotion",
        "CFBundleShortVersionString": "1.0.0",
        "CFBundleVersion": "30",
        "SigningIdentity": "ad hoc fixture",
        "Team": "N7VVB6PWZS",
    },
    "ArchiveVersion": 2,
    "Name": "BioMotion",
    "SchemeName": "BioMotion",
}
with (archive / "Info.plist").open("wb") as stream:
    plistlib.dump(archive_info, stream)
PY
printf '%s' fixture > "$ARCHIVE_APP/embedded.mobileprovision"
printf '%s' fixture \
  > "$ARCHIVE_APP/Extensions/AssetPackDownloader.appex/embedded.mobileprovision"
/usr/bin/codesign --force --sign - --timestamp=none \
  "$ARCHIVE_APP/Extensions/AssetPackDownloader.appex" >/dev/null
/usr/bin/codesign --force --sign - --timestamp=none "$ARCHIVE_APP" >/dev/null
cp "$ARCHIVE_APP/Info.plist" "$TEST_ROOT/archive-app-info.original.plist"

expect_failure ad_hoc_release_archive_rejected \
  'asset-pack extension is not Apple Distribution signed' \
  --release-archive "$ARCHIVE"
expect_failure positional_app_rejected 'usage: app_resource_boundary_probe.sh' \
  "$APP_BUNDLE"

: > "$ARCHIVE_APP/unexpected-release-data.bin"
expect_failure release_archive_extra_file 'built app file allowlist changed' \
  --release-archive "$ARCHIVE"
rm "$ARCHIVE_APP/unexpected-release-data.bin"

/usr/bin/python3 - "$ARCHIVE_APP/Info.plist" <<'PY'
from pathlib import Path
import plistlib
import sys

path = Path(sys.argv[1])
with path.open("rb") as stream:
    info = plistlib.load(stream)
info["DTPlatformName"] = "iphonesimulator"
with path.open("wb") as stream:
    plistlib.dump(info, stream)
PY
expect_failure release_archive_wrong_platform 'app has the wrong platform' \
  --release-archive "$ARCHIVE"
cp "$TEST_ROOT/archive-app-info.original.plist" "$ARCHIVE_APP/Info.plist"

/usr/bin/python3 - "$TEST_ROOT/unsafe.ipa" <<'PY'
from pathlib import Path
import stat
import sys
import zipfile

with zipfile.ZipFile(Path(sys.argv[1]), "w") as archive:
    entry = zipfile.ZipInfo("Payload/../escape")
    entry.create_system = 3
    entry.external_attr = (stat.S_IFREG | 0o644) << 16
    archive.writestr(entry, b"unsafe")
PY
expect_failure release_ipa_path_traversal \
  'release IPA contains an unsafe ZIP path' \
  --release-ipa "$TEST_ROOT/unsafe.ipa" "$ARCHIVE"

/usr/bin/python3 - "$TEST_ROOT/nul-name.ipa" <<'PY'
from pathlib import Path
import stat
import sys
import zipfile

path = Path(sys.argv[1])
safe_name = b"Payload/BioMotion.app/Info.plistXevil"
unsafe_name = b"Payload/BioMotion.app/Info.plist\x00evil"
with zipfile.ZipFile(path, "w") as archive:
    entry = zipfile.ZipInfo(safe_name.decode("ascii"))
    entry.create_system = 3
    entry.external_attr = (stat.S_IFREG | 0o644) << 16
    archive.writestr(entry, b"fixture")
data = path.read_bytes()
if data.count(safe_name) != 2:
    raise SystemExit("fixture ZIP filename inventory changed")
path.write_bytes(data.replace(safe_name, unsafe_name))
PY
expect_failure release_ipa_nul_filename \
  'release IPA contains an unsafe ZIP entry' \
  --release-ipa "$TEST_ROOT/nul-name.ipa" "$ARCHIVE"

/usr/bin/python3 - \
  "$TEST_ROOT/normal-raw.ipa" \
  "$TEST_ROOT/forged-size.ipa" \
  "$TEST_ROOT/non-executable.ipa" \
  "$TEST_ROOT/semantic-extra.ipa" \
  "$TEST_ROOT/entry-comment.ipa" \
  "$TEST_ROOT/archive-comment.ipa" <<'PY'
from pathlib import Path
import stat
import struct
import sys
import zipfile


def add_entry(archive, name, payload, mode, compression=zipfile.ZIP_DEFLATED):
    entry = zipfile.ZipInfo(name)
    entry.create_system = 3
    entry.external_attr = mode << 16
    if stat.S_ISDIR(mode):
        entry.external_attr |= 0x10
    entry.compress_type = compression
    archive.writestr(entry, payload)


normal = Path(sys.argv[1])
with zipfile.ZipFile(normal, "w") as archive:
    for directory in (
        "Payload/",
        "Payload/BioMotion.app/",
        "Payload/BioMotion.app/Extensions/",
        "Payload/BioMotion.app/Extensions/AssetPackDownloader.appex/",
    ):
        add_entry(archive, directory, b"", stat.S_IFDIR | 0o755)
    add_entry(
        archive,
        "Payload/BioMotion.app/BioMotion",
        b"app-code",
        stat.S_IFREG | 0o755,
    )
    add_entry(
        archive,
        "Payload/BioMotion.app/Extensions/AssetPackDownloader.appex/"
        "AssetPackDownloader",
        b"extension-code",
        stat.S_IFREG | 0o755,
    )
    add_entry(
        archive,
        "Payload/BioMotion.app/Info.plist",
        b"resource",
        stat.S_IFREG | 0o644,
    )

forged = Path(sys.argv[2])
with zipfile.ZipFile(forged, "w") as archive:
    add_entry(
        archive,
        "Payload/BioMotion.app/Info.plist",
        b"A" * (8 * 1024 * 1024),
        stat.S_IFREG | 0o644,
    )
with zipfile.ZipFile(forged, "r") as archive:
    local_offset = archive.infolist()[0].header_offset
    central_offset = archive.start_dir
data = bytearray(forged.read_bytes())
if data[local_offset:local_offset + 4] != b"PK\x03\x04":
    raise SystemExit("forged fixture local header changed")
if data[central_offset:central_offset + 4] != b"PK\x01\x02":
    raise SystemExit("forged fixture central header changed")
struct.pack_into("<I", data, local_offset + 22, 1)
struct.pack_into("<I", data, central_offset + 24, 1)
forged.write_bytes(data)

non_executable = Path(sys.argv[3])
with zipfile.ZipFile(non_executable, "w") as archive:
    add_entry(
        archive,
        "Payload/BioMotion.app/BioMotion",
        b"app-code",
        stat.S_IFREG | 0o644,
    )

semantic_extra = Path(sys.argv[4])
with zipfile.ZipFile(semantic_extra, "w") as archive:
    entry = zipfile.ZipInfo("Payload/BioMotion.app/Info.plist")
    entry.create_system = 3
    entry.external_attr = (stat.S_IFREG | 0o644) << 16
    entry.extra = struct.pack("<HH", 0x756E, 0)
    archive.writestr(entry, b"resource")

entry_comment = Path(sys.argv[5])
with zipfile.ZipFile(entry_comment, "w") as archive:
    entry = zipfile.ZipInfo("Payload/BioMotion.app/Info.plist")
    entry.create_system = 3
    entry.external_attr = (stat.S_IFREG | 0o644) << 16
    entry.comment = b"alternate entry metadata"
    archive.writestr(entry, b"resource")

archive_comment = Path(sys.argv[6])
with zipfile.ZipFile(archive_comment, "w") as archive:
    entry = zipfile.ZipInfo("Payload/BioMotion.app/Info.plist")
    entry.create_system = 3
    entry.external_attr = (stat.S_IFREG | 0o644) << 16
    archive.writestr(entry, b"resource")
    archive.comment = b"alternate archive metadata"
PY

expect_extractor_pass release_ipa_raw_normal "$TEST_ROOT/normal-raw.ipa"
expect_extractor_failure release_ipa_forged_real_size \
  'real size or CRC disagrees with its ZIP declarations' \
  "$TEST_ROOT/forged-size.ipa"
expect_extractor_failure release_ipa_executable_bits_zero \
  'release IPA code image is not executable: BioMotion' \
  "$TEST_ROOT/non-executable.ipa"
expect_extractor_failure release_ipa_semantic_extra \
  'uses an unreviewed ZIP extra field: 0x756e' \
  "$TEST_ROOT/semantic-extra.ipa"
expect_extractor_failure release_ipa_entry_comment \
  'contains an unreviewed ZIP entry comment' \
  "$TEST_ROOT/entry-comment.ipa"
expect_extractor_failure release_ipa_archive_comment \
  'release IPA contains an unreviewed ZIP archive comment' \
  "$TEST_ROOT/archive-comment.ipa"

if [ "$pass_count" -ne "$total_count" ] || [ "$total_count" -ne 40 ]; then
  printf 'app-resource boundary suite count mismatch: %s/%s\n' \
    "$pass_count" "$total_count" >&2
  exit 1
fi

printf 'APP_RESOURCE_BOUNDARY_TESTS_PASS %s/%s\n' \
  "$pass_count" "$total_count"
