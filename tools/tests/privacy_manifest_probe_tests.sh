#!/bin/bash -p
set -euo pipefail
PATH=/usr/bin:/bin:/usr/sbin:/sbin
export PATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-privacy-tests.XXXXXX")"
trap 'rm -r "$TEST_ROOT" 2>/dev/null || true' EXIT

FIXTURE_ROOT="$TEST_ROOT/repository"
mkdir -p \
  "$FIXTURE_ROOT/tools/tests" \
  "$FIXTURE_ROOT/BioMotion.xcodeproj" \
  "$FIXTURE_ROOT/BioMotion/ARKit" \
  "$FIXTURE_ROOT/BioMotion/App" \
  "$FIXTURE_ROOT/BioMotion/Muscle" \
  "$FIXTURE_ROOT/BioMotion/Nimble" \
  "$FIXTURE_ROOT/BioMotion/Offline" \
  "$FIXTURE_ROOT/BioMotion/Recording"
cp "$REPO_ROOT/tools/tests/privacy_manifest_probe.sh" \
  "$FIXTURE_ROOT/tools/tests/privacy_manifest_probe.sh"
cp "$REPO_ROOT/BioMotion/PrivacyInfo.xcprivacy" \
  "$FIXTURE_ROOT/BioMotion/PrivacyInfo.xcprivacy"
cp "$REPO_ROOT/project.yml" "$FIXTURE_ROOT/project.yml"
cp "$REPO_ROOT/BioMotion.xcodeproj/project.pbxproj" \
  "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj"

write_elapsed_fixture() {
  destination=$1
  count=$2
  : > "$destination"
  index=0
  while [ "$index" -lt "$count" ]; do
    printf 'let reviewedElapsedClock%s = CACurrentMediaTime()\n' "$index" \
      >> "$destination"
    index=$((index + 1))
  done
}

write_elapsed_fixture \
  "$FIXTURE_ROOT/BioMotion/ARKit/BodyTrackingSession.swift" 3
write_elapsed_fixture \
  "$FIXTURE_ROOT/BioMotion/App/CalibrationView.swift" 2
write_elapsed_fixture \
  "$FIXTURE_ROOT/BioMotion/Muscle/MuscleSolver.mm" 2
write_elapsed_fixture \
  "$FIXTURE_ROOT/BioMotion/Nimble/NimbleEngine.swift" 4
write_elapsed_fixture \
  "$FIXTURE_ROOT/BioMotion/Offline/OfflineSessionRunner.swift" 4
write_elapsed_fixture \
  "$FIXTURE_ROOT/BioMotion/Recording/MotionRecorder.swift" 2

pass_count=0
total_count=0

expect_pass() {
  label=$1
  total_count=$((total_count + 1))
  output="$(cd "$FIXTURE_ROOT" \
    && /bin/bash -p tools/tests/privacy_manifest_probe.sh 2>&1)"
  case "$output" in
    *PRIVACY_MANIFEST_PROBE_PASS*)
      pass_count=$((pass_count + 1))
      ;;
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
  total_count=$((total_count + 1))
  set +e
  output="$(cd "$FIXTURE_ROOT" \
    && /bin/bash -p tools/tests/privacy_manifest_probe.sh 2>&1)"
  status=$?
  set -e
  if [ "$status" -eq 0 ]; then
    printf '%s unexpectedly passed\n' "$label" >&2
    exit 1
  fi
  case "$output" in
    *"$expected"*)
      pass_count=$((pass_count + 1))
      ;;
    *)
      printf '%s failed for the wrong reason: %s\n' "$label" "$output" >&2
      exit 1
      ;;
  esac
}

expect_environment_pass() {
  label=$1
  forbidden_path=$2
  shift 2
  total_count=$((total_count + 1))
  set +e
  output="$(cd "$FIXTURE_ROOT" \
    && /usr/bin/env "$@" \
      /bin/bash -p tools/tests/privacy_manifest_probe.sh 2>&1)"
  status=$?
  set -e
  if [ "$status" -ne 0 ]; then
    printf '%s failed under a hostile environment: %s\n' \
      "$label" "$output" >&2
    exit 1
  fi
  case "$output" in
    *PRIVACY_MANIFEST_PROBE_PASS*) ;;
    *)
      printf '%s omitted the pass sentinel: %s\n' "$label" "$output" >&2
      exit 1
      ;;
  esac
  if [ -n "$forbidden_path" ] && [ -e "$forbidden_path" ]; then
    printf '%s executed injected Python startup code\n' "$label" >&2
    exit 1
  fi
  pass_count=$((pass_count + 1))
}

expect_pass baseline

expect_environment_pass hostile_python_and_xcode_environment '' \
  PYTHONHOME="$TEST_ROOT/untrusted-python-home" \
  PYTHONINSPECT=1 \
  PYTHONSTARTUP="$TEST_ROOT/untrusted-python-startup" \
  PYTHONWARNINGS=error \
  DEVELOPER_DIR="$TEST_ROOT/untrusted-developer" \
  SDKROOT="$TEST_ROOT/untrusted-sdk" \
  TOOLCHAINS=untrusted-toolchain \
  XCODE_XCCONFIG_FILE="$TEST_ROOT/untrusted.xcconfig"

PYTHON_INJECTION_ROOT="$TEST_ROOT/python-injection"
PYTHON_INJECTION_MARKER="$TEST_ROOT/python-injection-ran"
mkdir -p "$PYTHON_INJECTION_ROOT"
cat > "$PYTHON_INJECTION_ROOT/sitecustomize.py" <<'PY'
import os
from pathlib import Path
Path(os.environ["BIOMOTION_PYTHON_INJECTION_MARKER"]).write_text(
    "injected\n", encoding="utf-8"
)
PY
expect_environment_pass hostile_pythonpath_sitecustomize \
  "$PYTHON_INJECTION_MARKER" \
  BIOMOTION_PYTHON_INJECTION_MARKER="$PYTHON_INJECTION_MARKER" \
  PYTHONPATH="$PYTHON_INJECTION_ROOT"

UNTRUSTED_TMPDIR="$TEST_ROOT/untrusted-tmpdir"
: > "$UNTRUSTED_TMPDIR"
expect_environment_pass hostile_tmpdir_is_ignored '' \
  TMPDIR="$UNTRUSTED_TMPDIR"

SOURCE_CASES=(
  'systemUptime|let value = ProcessInfo.processInfo.systemUptime'
  'mach_absolute_time|let value = mach_absolute_time('
  'creationDate|let value = attributes.creationDate'
  'modificationDate|let value = attributes.modificationDate'
  'fileModificationDate|let value = document.fileModificationDate'
  'contentModificationDateKey|let value = URLResourceKey.contentModificationDateKey'
  'creationDateKey|let value = URLResourceKey.creationDateKey'
  'getattrlist|let value = getattrlist('
  'getattrlistbulk|let value = getattrlistbulk('
  'fgetattrlist|let value = fgetattrlist('
  'stat|let value = stat('
  'fstat|let value = fstat('
  'fstatat|let value = fstatat('
  'lstat|let value = lstat('
  'getattrlistat|let value = getattrlistat('
  'volumeAvailableCapacityKey|let value = URLResourceKey.volumeAvailableCapacityKey'
  'volumeAvailableCapacityForImportantUsageKey|let value = URLResourceKey.volumeAvailableCapacityForImportantUsageKey'
  'volumeAvailableCapacityForOpportunisticUsageKey|let value = URLResourceKey.volumeAvailableCapacityForOpportunisticUsageKey'
  'volumeTotalCapacityKey|let value = URLResourceKey.volumeTotalCapacityKey'
  'systemFreeSize|let value = attributes.systemFreeSize'
  'systemSize|let value = attributes.systemSize'
  'statfs|let value = statfs('
  'statvfs|let value = statvfs('
  'fstatfs|let value = fstatfs('
  'fstatvfs|let value = fstatvfs('
  'activeInputModes|let value = UITextInputMode.activeInputModes'
  'UserDefaults|let value = UserDefaults.standard'
  'NSUserDefaults|let value = NSUserDefaults.standard'
  'CFPreferences|let value = CFPreferencesCopyAppValue('
  'AppStorage|@AppStorage("privacy") var value = false'
)

INJECTED_SOURCE="$FIXTURE_ROOT/BioMotion/PrivacyProbeInjected.swift"
for source_case in "${SOURCE_CASES[@]}"; do
  label=${source_case%%|*}
  fragment=${source_case#*|}
  printf '%s\n' "$fragment" > "$INJECTED_SOURCE"
  expect_failure "$label" 'required-reason source inventory changed'
  rm "$INJECTED_SOURCE"
done

printf '%s\n' 'let value = clock_gettime(' > "$INJECTED_SOURCE"
expect_failure clock_gettime 'reviewed non-required elapsed-clock inventory changed'
rm "$INJECTED_SOURCE"
printf '%s\n' 'let value = CACurrentMediaTime()' > "$INJECTED_SOURCE"
expect_failure CACurrentMediaTime \
  'reviewed non-required elapsed-clock inventory changed'
rm "$INJECTED_SOURCE"

MANIFEST="$FIXTURE_ROOT/BioMotion/PrivacyInfo.xcprivacy"
cp "$MANIFEST" "$TEST_ROOT/PrivacyInfo.original"
/usr/bin/python3 - "$MANIFEST" <<'PY'
import plistlib
import sys

path = sys.argv[1]
with open(path, "rb") as source:
    manifest = plistlib.load(source)
manifest["NSPrivacyAccessedAPITypes"] = []
with open(path, "wb") as destination:
    plistlib.dump(manifest, destination)
PY
expect_failure empty_accessed_api_array \
  'privacy manifest has an unreviewed top-level key'
cp "$TEST_ROOT/PrivacyInfo.original" "$MANIFEST"

PROJECT_SPEC="$FIXTURE_ROOT/project.yml"
cp "$PROJECT_SPEC" "$TEST_ROOT/project.original.yml"
/usr/bin/python3 - "$PROJECT_SPEC" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
pair = (
    "      - path: BioMotion/PrivacyInfo.xcprivacy\n"
    "        buildPhase: resources\n"
)
if text.count(pair) != 1 or "  AssetPackDownloader:\n" not in text:
    raise SystemExit("fixture project layout changed")
text = text.replace(pair, "", 1)
text = text.replace("  AssetPackDownloader:\n", "  AssetPackDownloader:\n" + pair, 1)
path.write_text(text, encoding="utf-8")
PY
expect_failure wrong_target_assignment \
  'project.yml must assign the manifest exactly once to BioMotion'
cp "$TEST_ROOT/project.original.yml" "$PROJECT_SPEC"

PROJECT_FILE="$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj"
cp "$PROJECT_FILE" "$TEST_ROOT/project.original.pbxproj"
/usr/bin/python3 - "$PROJECT_FILE" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
privacy_reference = re.search(
    r"^\s*([A-F0-9]{24}) /\* [^*]+ \*/ = "
    r"\{isa = PBXFileReference;[^}]*\bpath = \"?PrivacyInfo\.xcprivacy\"?;",
    text,
    flags=re.MULTILINE,
)
if privacy_reference is None:
    raise SystemExit("fixture privacy file reference changed")
privacy_reference_id = privacy_reference.group(1)
other_reference = re.search(
    rf"^\s*((?!{privacy_reference_id})[A-F0-9]{{24}}) /\* [^*]+ \*/ = "
    r"\{isa = PBXFileReference;",
    text,
    flags=re.MULTILINE,
)
build_file = re.search(
    rf"^\s*[A-F0-9]{{24}} /\* [^*]+ \*/ = "
    rf"\{{isa = PBXBuildFile;[^}}]*\bfileRef = {privacy_reference_id}\b[^}}]*\}};$",
    text,
    flags=re.MULTILINE,
)
if other_reference is None or build_file is None:
    raise SystemExit("fixture privacy build reference changed")
replacement = build_file.group(0).replace(
    f"fileRef = {privacy_reference_id}",
    f"fileRef = {other_reference.group(1)}",
    1,
)
text = text[:build_file.start()] + replacement + text[build_file.end():]
path.write_text(text, encoding="utf-8")
PY
expect_failure wrong_pbx_file_reference \
  'generated project must have one privacy build/file reference'
cp "$TEST_ROOT/project.original.pbxproj" "$PROJECT_FILE"

/usr/bin/python3 - "$PROJECT_FILE" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
privacy_reference = re.search(
    r"^\s*([A-F0-9]{24}) /\* [^*]+ \*/ = "
    r"\{isa = PBXFileReference;[^}]*\bpath = \"?PrivacyInfo\.xcprivacy\"?;",
    text,
    flags=re.MULTILINE,
)
if privacy_reference is None:
    raise SystemExit("fixture privacy file reference changed")
build_file = re.search(
    rf"^\s*([A-F0-9]{{24}}) /\* [^*]+ \*/ = "
    rf"\{{isa = PBXBuildFile;[^}}]*\bfileRef = {privacy_reference.group(1)}\b",
    text,
    flags=re.MULTILINE,
)
if build_file is None:
    raise SystemExit("fixture privacy build file changed")
membership = re.search(
    rf"^\s+{build_file.group(1)} /\* [^*]+ \*/,$",
    text,
    flags=re.MULTILINE,
)
if membership is None:
    raise SystemExit("fixture privacy phase membership changed")
line = membership.group(0)
text = text[:membership.start()] + line + "\n" + line + text[membership.end():]
path.write_text(text, encoding="utf-8")
PY
expect_failure duplicate_pbx_phase_membership \
  'privacy build-file identifier must occur exactly twice'
cp "$TEST_ROOT/project.original.pbxproj" "$PROJECT_FILE"

expect_pass restored_baseline
printf 'PRIVACY_MANIFEST_PROBE_TESTS_PASS %s/%s\n' \
  "$pass_count" "$total_count"
