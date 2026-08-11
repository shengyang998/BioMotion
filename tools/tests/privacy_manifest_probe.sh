#!/bin/bash
set -euo pipefail
PATH=/usr/bin:/bin:/usr/sbin:/sbin
export PATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
MANIFEST="$REPO_ROOT/BioMotion/PrivacyInfo.xcprivacy"
PROJECT_FILE="$REPO_ROOT/BioMotion.xcodeproj/project.pbxproj"
PROJECT_SPEC="$REPO_ROOT/project.yml"
APP_BUNDLE=${1:-}
PROBE_TMP="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-privacy.XXXXXX")"
trap 'rm -r "$PROBE_TMP" 2>/dev/null || true' EXIT

for required_file in "$MANIFEST" "$PROJECT_FILE" "$PROJECT_SPEC"; do
  if [ ! -f "$required_file" ]; then
    printf 'required privacy input is missing: %s\n' "$required_file" >&2
    exit 2
  fi
done

plutil -lint "$MANIFEST" >/dev/null
python3 - "$MANIFEST" <<'PY'
import plistlib
import sys

manifest_path = sys.argv[1]
with open(manifest_path, "rb") as manifest_file:
    manifest = plistlib.load(manifest_file)

if set(manifest) != {"NSPrivacyTracking"}:
    raise SystemExit("privacy manifest has an unreviewed top-level key")
if manifest["NSPrivacyTracking"] is not False:
    raise SystemExit("privacy tracking must remain false")
PY

python3 - "$REPO_ROOT" <<'PY'
from collections import Counter
from pathlib import Path
import re
import sys

repo = Path(sys.argv[1])
source_root = repo / "BioMotion"
source_suffixes = {".swift", ".m", ".mm", ".h", ".hpp", ".c", ".cc", ".cpp"}
sources = [
    path for path in source_root.rglob("*")
    if path.is_file() and path.suffix in source_suffixes
]

category_patterns = {
    "system_boot_time": [
        r"\bmach_absolute_time\s*\(",
        r"\bsystemUptime\b",
    ],
    "file_timestamp": [
        r"\.(?:creationDate|modificationDate|fileModificationDate|contentModificationDateKey|creationDateKey)\b",
        r"\b(?:getattrlist|getattrlistbulk|fgetattrlist|getattrlistat|stat|fstat|fstatat|lstat)\s*\(",
    ],
    "disk_space": [
        r"\b(?:statfs|statvfs|fstatfs|fstatvfs|getattrlist|fgetattrlist|getattrlistat)\s*\(",
        r"\.(?:volumeAvailableCapacityKey|volumeAvailableCapacityForImportantUsageKey|volumeAvailableCapacityForOpportunisticUsageKey|volumeTotalCapacityKey|systemFreeSize|systemSize)\b",
    ],
    "user_defaults": [
        r"\b(?:UserDefaults|NSUserDefaults|AppStorage)\b",
        r"\bCFPreferences[A-Za-z0-9_]*",
    ],
    "active_keyboards": [r"\bactiveInputModes\b"],
}

observed = {category: Counter() for category in category_patterns}
for source in sources:
    text = source.read_text(encoding="utf-8", errors="replace")
    relative = str(source.relative_to(repo))
    for category, patterns in category_patterns.items():
        for pattern in patterns:
            observed[category][relative] += len(re.findall(pattern, text))

observed = {
    category: Counter({path: count for path, count in counts.items() if count})
    for category, counts in observed.items()
}
expected = {
    "system_boot_time": Counter(),
    "file_timestamp": Counter(),
    "disk_space": Counter(),
    "user_defaults": Counter(),
    "active_keyboards": Counter(),
}
if observed != expected:
    raise SystemExit(
        "required-reason source inventory changed; review category and reason: "
        f"{observed}"
    )

# Apple does not list CACurrentMediaTime() or clock_gettime() under the System
# Boot Time required-reason category. Keep their reviewed elapsed-time use
# pinned separately so a new call site still triggers a privacy review instead
# of being silently confused with mach_absolute_time()/systemUptime.
elapsed_clock_patterns = {
    "CACurrentMediaTime": r"\bCACurrentMediaTime\s*\(",
    "clock_gettime": r"\bclock_gettime\s*\(",
}
elapsed_clocks = {name: Counter() for name in elapsed_clock_patterns}
for source in sources:
    text = source.read_text(encoding="utf-8", errors="replace")
    relative = str(source.relative_to(repo))
    for name, pattern in elapsed_clock_patterns.items():
        count = len(re.findall(pattern, text))
        if count:
            elapsed_clocks[name][relative] = count
expected_elapsed_clocks = {
    "CACurrentMediaTime": Counter({
        "BioMotion/ARKit/BodyTrackingSession.swift": 3,
        "BioMotion/Muscle/MuscleSolver.mm": 2,
        "BioMotion/Nimble/NimbleEngine.swift": 4,
        "BioMotion/Offline/OfflineSessionRunner.swift": 4,
    }),
    "clock_gettime": Counter(),
}
if elapsed_clocks != expected_elapsed_clocks:
    raise SystemExit(
        "reviewed non-required elapsed-clock inventory changed: "
        f"{elapsed_clocks}"
    )

network_or_sdk = re.compile(
    r"\b(?:URLSession|NSURLSession|NWConnection|NWPathMonitor|"
    r"ATTrackingManager|ASIdentifierManager|AdSupport|Firebase|"
    r"Sentry|Amplitude|Mixpanel|AppCenter)\b|\bWebSocket\b"
)
unexpected_network = []
for source in sources:
    text = source.read_text(encoding="utf-8", errors="replace")
    if network_or_sdk.search(text):
        unexpected_network.append(str(source.relative_to(repo)))
if unexpected_network:
    raise SystemExit(
        "network/tracking/analytics surface changed; privacy review required: "
        + ", ".join(unexpected_network)
    )
PY

python3 - "$PROJECT_SPEC" "$PROJECT_FILE" <<'PY'
from pathlib import Path
import re
import sys

spec_path = Path(sys.argv[1])
project_path = Path(sys.argv[2])
spec_lines = spec_path.read_text(encoding="utf-8").splitlines()
project = project_path.read_text(encoding="utf-8")

try:
    target_start = spec_lines.index("  BioMotion:")
except ValueError as error:
    raise SystemExit("project.yml has no BioMotion target") from error
target_end = len(spec_lines)
for index in range(target_start + 1, len(spec_lines)):
    if re.fullmatch(r"  [A-Za-z0-9_-]+:", spec_lines[index]):
        target_end = index
        break
target_lines = spec_lines[target_start:target_end]
resource_pair = [
    "      - path: BioMotion/PrivacyInfo.xcprivacy",
    "        buildPhase: resources",
]
pair_count = sum(
    target_lines[index:index + 2] == resource_pair
    for index in range(len(target_lines) - 1)
)
global_pair_count = sum(
    spec_lines[index:index + 2] == resource_pair
    for index in range(len(spec_lines) - 1)
)
if pair_count != 1 or global_pair_count != 1:
    raise SystemExit(
        "project.yml must assign the manifest exactly once to BioMotion"
    )

file_references = []
for match in re.finditer(
    r"^\s*([A-F0-9]{24}) /\* [^*]+ \*/ = "
    r"\{isa = PBXFileReference;(?P<body>[^}]*)\};$",
    project,
    flags=re.MULTILINE,
):
    if re.search(
        r'\bpath = "?PrivacyInfo\.xcprivacy"?;', match.group("body")
    ):
        file_references.append(match.group(1))
if len(file_references) != 1:
    raise SystemExit("generated project must have one privacy file reference")
privacy_file_reference = file_references[0]

build_files = []
for match in re.finditer(
    r"^\s*([A-F0-9]{24}) /\* [^*]+ \*/ = "
    r"\{isa = PBXBuildFile;(?P<body>[^}]*)\};$",
    project,
    flags=re.MULTILINE,
):
    reference_match = re.search(
        r"\bfileRef = ([A-F0-9]{24})\b", match.group("body")
    )
    if reference_match and reference_match.group(1) == privacy_file_reference:
        build_files.append(match.group(1))
if len(build_files) != 1:
    raise SystemExit("generated project must have one privacy build/file reference")
privacy_build_file = build_files[0]
if len(re.findall(rf"\b{privacy_build_file}\b", project)) != 2:
    raise SystemExit("privacy build-file identifier must occur exactly twice")

resource_phases = {}
for match in re.finditer(
    r"^\t\t([A-F0-9]{24}) /\* [^*]+ \*/ = \{\n"
    r"\t\t\tisa = PBXResourcesBuildPhase;(.*?)^\t\t\};",
    project,
    flags=re.MULTILINE | re.DOTALL,
):
    resource_phases[match.group(1)] = match.group(2)
membership_phases = []
membership_count = 0
for phase_id, body in resource_phases.items():
    occurrences = len(re.findall(rf"\b{privacy_build_file}\b", body))
    membership_count += occurrences
    if occurrences:
        membership_phases.append(phase_id)
if membership_count != 1 or len(membership_phases) != 1:
    raise SystemExit("privacy resource must occur once in one resources phase")

native_targets = {}
for match in re.finditer(
    r"^\t\t([A-F0-9]{24}) /\* [^*]+ \*/ = \{\n"
    r"\t\t\tisa = PBXNativeTarget;(.*?)^\t\t\};",
    project,
    flags=re.MULTILINE | re.DOTALL,
):
    body = match.group(2)
    name_match = re.search(r'^\t\t\tname = "?([^";]+)"?;$', body, re.MULTILINE)
    phase_match = re.search(
        r"\bbuildPhases = \((.*?)\);", body, flags=re.DOTALL
    )
    if name_match and phase_match:
        name = name_match.group(1)
        if name in native_targets:
            raise SystemExit(f"duplicate native target name: {name}")
        native_targets[name] = set(
            re.findall(r"\b[A-F0-9]{24}\b", phase_match.group(1))
        )
if "BioMotion" not in native_targets:
    raise SystemExit("generated project has no BioMotion native target")
privacy_phase = membership_phases[0]
if privacy_phase not in native_targets["BioMotion"]:
    raise SystemExit("privacy resource phase does not belong to BioMotion")
wrong_targets = [
    name
    for name, phases in native_targets.items()
    if name != "BioMotion" and privacy_phase in phases
]
if wrong_targets:
    raise SystemExit(
        "privacy resource phase is shared with the wrong target: "
        + ", ".join(wrong_targets)
    )
PY

if [ -n "$APP_BUNDLE" ]; then
  if [ ! -d "$APP_BUNDLE" ]; then
    printf 'app bundle does not exist: %s\n' "$APP_BUNDLE" >&2
    exit 6
  fi
  APP_INFO="$APP_BUNDLE/Info.plist"
  if [ ! -f "$APP_INFO" ]; then
    printf '%s\n' 'built app omitted Info.plist' >&2
    exit 6
  fi
  BUNDLE_IDENTIFIER="$(
    plutil -extract CFBundleIdentifier raw -o - "$APP_INFO"
  )"
  if [ "$BUNDLE_IDENTIFIER" != 'com.soleil.BioMotion' ]; then
    printf 'unexpected app bundle identifier: %s\n' "$BUNDLE_IDENTIFIER" >&2
    exit 6
  fi
  BUNDLED_MANIFEST="$APP_BUNDLE/PrivacyInfo.xcprivacy"
  if [ ! -f "$BUNDLED_MANIFEST" ] || [ -L "$BUNDLED_MANIFEST" ]; then
    printf '%s\n' 'built app omitted PrivacyInfo.xcprivacy' >&2
    exit 6
  fi
  ROOT_MANIFEST_COUNT="$(
    find "$APP_BUNDLE" -maxdepth 1 -type f \
      -name 'PrivacyInfo.xcprivacy' | awk 'END { print NR + 0 }'
  )"
  if [ "$ROOT_MANIFEST_COUNT" -ne 1 ]; then
    printf 'built app must have one root privacy manifest; found %s\n' \
      "$ROOT_MANIFEST_COUNT" >&2
    exit 6
  fi
  ALL_MANIFEST_COUNT="$(
    find "$APP_BUNDLE" -type f -name 'PrivacyInfo.xcprivacy' \
      | awk 'END { print NR + 0 }'
  )"
  if [ "$ALL_MANIFEST_COUNT" -ne 1 ]; then
    printf 'built app must have one total privacy manifest; found %s\n' \
      "$ALL_MANIFEST_COUNT" >&2
    exit 6
  fi
  if ! cmp -s "$MANIFEST" "$BUNDLED_MANIFEST"; then
    printf '%s\n' 'built privacy manifest differs from reviewed source' >&2
    exit 6
  fi

  while IFS= read -r embedded_manifest; do
    if [ -L "$embedded_manifest" ]; then
      printf 'privacy manifest must not be a symlink: %s\n' \
        "$embedded_manifest" >&2
      exit 6
    fi
    plutil -lint "$embedded_manifest" >/dev/null
  done < <(find "$APP_BUNDLE" -name 'PrivacyInfo.xcprivacy' -print | sort)

  APP_EXECUTABLE_NAME="$(
    plutil -extract CFBundleExecutable raw -o - "$APP_INFO"
  )"
  APP_EXECUTABLE="$APP_BUNDLE/$APP_EXECUTABLE_NAME"
  if [ ! -f "$APP_EXECUTABLE" ] || [ -L "$APP_EXECUTABLE" ]; then
    printf 'built app executable is missing: %s\n' "$APP_EXECUTABLE" >&2
    exit 6
  fi
  case "$(file -b "$APP_EXECUTABLE")" in
    Mach-O*) ;;
    *)
      printf 'built app executable is not Mach-O: %s\n' "$APP_EXECUTABLE" >&2
      exit 6
      ;;
  esac

  ALLOWED_INTERNAL_DEPENDENCIES="$PROBE_TMP/allowed-internal-dependencies"
  : > "$ALLOWED_INTERNAL_DEPENDENCIES"
  APP_DEBUG_DYLIB="$APP_BUNDLE/$APP_EXECUTABLE_NAME.debug.dylib"
  if [ -e "$APP_DEBUG_DYLIB" ] || [ -L "$APP_DEBUG_DYLIB" ]; then
    if [ ! -f "$APP_DEBUG_DYLIB" ] || [ -L "$APP_DEBUG_DYLIB" ]; then
      printf 'app debug dylib is not a regular in-bundle file: %s\n' \
        "$APP_DEBUG_DYLIB" >&2
      exit 6
    fi
    case "$(file -b "$APP_DEBUG_DYLIB")" in
      Mach-O*) ;;
      *)
        printf 'app debug dylib is not Mach-O: %s\n' "$APP_DEBUG_DYLIB" >&2
        exit 6
        ;;
    esac
    printf '@rpath/%s.debug.dylib\n' "$APP_EXECUTABLE_NAME" \
      >> "$ALLOWED_INTERNAL_DEPENDENCIES"
  fi

  while IFS= read -r extension_bundle; do
    EXTENSION_INFO="$extension_bundle/Info.plist"
    if [ ! -f "$EXTENSION_INFO" ] || [ -L "$EXTENSION_INFO" ]; then
      printf 'extension Info.plist is not a regular file: %s\n' \
        "$EXTENSION_INFO" >&2
      exit 6
    fi
    EXTENSION_EXECUTABLE_NAME="$(
      plutil -extract CFBundleExecutable raw -o - "$EXTENSION_INFO"
    )"
    EXTENSION_EXECUTABLE="$extension_bundle/$EXTENSION_EXECUTABLE_NAME"
    if [ ! -f "$EXTENSION_EXECUTABLE" ] \
        || [ -L "$EXTENSION_EXECUTABLE" ]; then
      printf 'extension executable is not a regular file: %s\n' \
        "$EXTENSION_EXECUTABLE" >&2
      exit 6
    fi
    case "$(file -b "$EXTENSION_EXECUTABLE")" in
      Mach-O*) ;;
      *)
        printf 'extension executable is not Mach-O: %s\n' \
          "$EXTENSION_EXECUTABLE" >&2
        exit 6
        ;;
    esac
    EXTENSION_DEBUG_DYLIB="$extension_bundle/$EXTENSION_EXECUTABLE_NAME.debug.dylib"
    if [ -e "$EXTENSION_DEBUG_DYLIB" ] || [ -L "$EXTENSION_DEBUG_DYLIB" ]; then
      if [ ! -f "$EXTENSION_DEBUG_DYLIB" ] \
          || [ -L "$EXTENSION_DEBUG_DYLIB" ]; then
        printf 'extension debug dylib is not a regular file: %s\n' \
          "$EXTENSION_DEBUG_DYLIB" >&2
        exit 6
      fi
      case "$(file -b "$EXTENSION_DEBUG_DYLIB")" in
        Mach-O*) ;;
        *)
          printf 'extension debug dylib is not Mach-O: %s\n' \
            "$EXTENSION_DEBUG_DYLIB" >&2
          exit 6
          ;;
      esac
      printf '@rpath/%s.debug.dylib\n' "$EXTENSION_EXECUTABLE_NAME" \
        >> "$ALLOWED_INTERNAL_DEPENDENCIES"
    fi
  done < <(find "$APP_BUNDLE" -type d -name '*.appex' -print | sort)

  while IFS= read -r -d '' symlink_candidate; do
    case "$(file -b -L "$symlink_candidate" 2>/dev/null || true)" in
      Mach-O*)
        printf 'code image must not be hidden behind a symlink: %s\n' \
          "$symlink_candidate" >&2
        exit 6
        ;;
    esac
  done < <(find "$APP_BUNDLE" -type l -print0)

  CODE_IMAGE_COUNT=0
  while IFS= read -r -d '' code_image; do
    case "$(file -b "$code_image")" in
      Mach-O*) ;;
      *) continue ;;
    esac
    CODE_IMAGE_COUNT=$((CODE_IMAGE_COUNT + 1))
    nm -u "$code_image" 2>/dev/null | c++filt \
      > "$PROBE_TMP/code-image.undefined"
    strings "$code_image" > "$PROBE_TMP/code-image.strings"
    if grep -Eiq \
        '(^|[[:space:]])_?(mach_absolute_time|getattrlist|getattrlistbulk|fgetattrlist|getattrlistat|stat|fstat|fstatat|lstat|statfs|statvfs|fstatfs|fstatvfs)$' \
        "$PROBE_TMP/code-image.undefined" \
        || grep -Eiq \
        'systemUptime|creationDate|modificationDate|fileModificationDate|contentModificationDateKey|creationDateKey|volumeAvailableCapacity|volumeTotalCapacityKey|systemFreeSize|systemSize|activeInputModes|UserDefaults|NSUserDefaults|CFPreferences|AppStorage|ATTrackingManager|ASIdentifierManager|AdSupport' \
        "$PROBE_TMP/code-image.strings"; then
      printf 'code image contains an undeclared privacy/tracking API: %s\n' \
        "$code_image" >&2
      exit 6
    fi

    while IFS= read -r dependency; do
      case "$dependency" in
        /System/Library/Frameworks/*|/usr/lib/*)
          ;;
        *)
          if ! grep -Fqx "$dependency" "$ALLOWED_INTERNAL_DEPENDENCIES"; then
            printf 'code image has an unreviewed dynamic dependency: %s: %s\n' \
              "$code_image" "$dependency" >&2
            exit 6
          fi
          ;;
      esac
    done < <(otool -L "$code_image" | tail -n +2 | awk '{ print $1 }')
  done < <(find "$APP_BUNDLE" -type f -print0)
  if [ "$CODE_IMAGE_COUNT" -eq 0 ]; then
    printf '%s\n' 'built app contains no Mach-O code image' >&2
    exit 6
  fi

  codesign --verify --deep --strict "$APP_BUNDLE"
fi

printf '%s\n' 'PRIVACY_MANIFEST_PROBE_PASS'
