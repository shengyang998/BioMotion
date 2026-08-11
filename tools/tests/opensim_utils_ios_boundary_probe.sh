#!/bin/bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NIMBLE_ROOT="$REPO_ROOT/nimblephysics"
HEADER="$NIMBLE_ROOT/dart/biomechanics/OpenSimParser.hpp"
IMPLEMENTATION="$NIMBLE_ROOT/dart/biomechanics/OpenSimParser.cpp"
SOURCE="$SCRIPT_DIR/opensim_utils_ios_boundary_probe.cpp"
SIM_ARCHIVE="$NIMBLE_ROOT/build_sim/libnimble_ios.a"
DEVICE_ARCHIVE="$NIMBLE_ROOT/build_ios/libnimble_ios.a"
OSQP_SIM_ARCHIVE="$REPO_ROOT/osqp/build_sim/out/libosqpstatic.a"
OSQP_DEVICE_ARCHIVE="$REPO_ROOT/osqp/build_ios/out/libosqpstatic.a"
PROBE_TMP="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-opensim-utils.XXXXXX")"
trap 'rm -r "$PROBE_TMP" 2>/dev/null || true' EXIT

SIM_SDK="$(xcrun --sdk iphonesimulator --show-sdk-path)"
SIM_CXX="$(xcrun --sdk iphonesimulator --find clang++)"
DEVICE_SDK="$(xcrun --sdk iphoneos --show-sdk-path)"
DEVICE_CXX="$(xcrun --sdk iphoneos --find clang++)"

SIM_COMMON=(
  -target arm64-apple-ios17.0-simulator
  -isysroot "$SIM_SDK"
  -std=c++17
  -DDART_IOS_BUILD=1
  -DDART_USE_IDENTITY_JACOBIAN=1
  -DEIGEN_DONT_PARALLELIZE
  -Wno-deprecated-literal-operator
  -Wno-sign-compare
  -I"$NIMBLE_ROOT/build_sim"
  -I"$NIMBLE_ROOT"
  -I"$NIMBLE_ROOT/third_party/eigen"
  -I"$NIMBLE_ROOT/third_party/tinyxml2"
)

DEVICE_COMMON=(
  -target arm64-apple-ios17.0
  -isysroot "$DEVICE_SDK"
  -std=c++17
  -DDART_IOS_BUILD=1
  -DDART_USE_IDENTITY_JACOBIAN=1
  -DEIGEN_DONT_PARALLELIZE
  -Wno-deprecated-literal-operator
  -Wno-sign-compare
  -I"$NIMBLE_ROOT/build_ios"
  -I"$NIMBLE_ROOT"
  -I"$NIMBLE_ROOT/third_party/eigen"
  -I"$NIMBLE_ROOT/third_party/tinyxml2"
)

failures=0
SUPPORTED_SYMBOL_TOKENS=(
  'OpenSimParser::parseOsim(dart::common::Uri const&'
  'OpenSimParser::parseOsim(tinyxml2::XMLDocument&'
  'OpenSimParser::loadTRC('
  'OpenSimParser::loadMot('
  'OpenSimParser::loadGRF('
  'OpenSimParser::loadMotAtLowestMarkerRMSERotation('
)

record_failure() {
  printf 'FAIL %s\n' "$1" >&2
  failures=$((failures + 1))
}

require_symbol_count() {
  label=$1
  symbols_file=$2
  symbol=$3
  expected=$4
  actual="$(grep -Fc "$symbol" "$symbols_file" || true)"
  if [ "$actual" -ne "$expected" ]; then
    record_failure \
      "$label contains $actual copies of $symbol, expected $expected"
  fi
}

require_supported_symbols() {
  label=$1
  symbols_file=$2
  for symbol in "${SUPPORTED_SYMBOL_TOKENS[@]}"; do
    require_symbol_count "$label" "$symbols_file" "$symbol" 1
  done
  require_symbol_count \
    "$label" "$symbols_file" 'OpenSimParser::parseOsim(' 2
}

archive_for() {
  if [ "$1" = simulator ]; then
    printf '%s\n' "$SIM_ARCHIVE"
  else
    printf '%s\n' "$DEVICE_ARCHIVE"
  fi
}

osqp_archive_for() {
  if [ "$1" = simulator ]; then
    printf '%s\n' "$OSQP_SIM_ARCHIVE"
  else
    printf '%s\n' "$OSQP_DEVICE_ARCHIVE"
  fi
}

compile_argv_for() {
  if [ "$1" = simulator ]; then
    COMPILE_ARGV=("$SIM_CXX" "${SIM_COMMON[@]}")
  else
    COMPILE_ARGV=("$DEVICE_CXX" "${DEVICE_COMMON[@]}")
  fi
}

expect_ios_api_hidden() {
  platform=$1
  label=$2
  macro=$3
  symbol=$4
  compile_argv_for "$platform"
  archive="$(archive_for "$platform")"
  osqp_archive="$(osqp_archive_for "$platform")"
  syntax_log="$PROBE_TMP/${platform}_${label}.syntax.log"
  object="$PROBE_TMP/${platform}_${label}.o"
  executable="$PROBE_TMP/${platform}_${label}"
  link_log="$PROBE_TMP/${platform}_${label}.link.log"

  if "${COMPILE_ARGV[@]}" -Wall -Wextra -Werror -Wno-sign-compare \
      "-D${macro}=1" \
      -fsyntax-only "$SOURCE" >"$syntax_log" 2>&1; then
    if ! "${COMPILE_ARGV[@]}" -Wall -Wextra -Werror -Wno-sign-compare \
        "-D${macro}=1" \
        -c "$SOURCE" -o "$object" >>"$syntax_log" 2>&1; then
      cat "$syntax_log" >&2
      record_failure \
        "$platform $symbol declaration compiled in syntax mode but not object mode"
      return
    fi

    nm -gu "$object" | c++filt >"$PROBE_TMP/${platform}_${label}.undefined"
    nm -gU "$archive" | c++filt >"$PROBE_TMP/${platform}_${label}.archive"
    if ! grep -Fq "OpenSimParser::$symbol" \
        "$PROBE_TMP/${platform}_${label}.undefined"; then
      record_failure "$platform causal probe did not reference $symbol"
    fi
    if grep -Fq "OpenSimParser::$symbol" \
        "$PROBE_TMP/${platform}_${label}.archive"; then
      record_failure "$platform archive unexpectedly defines $symbol"
    fi

    if "${COMPILE_ARGV[@]}" "$object" "$archive" "$osqp_archive" \
        -Wl,-dead_strip -o "$executable" >"$link_log" 2>&1; then
      record_failure \
        "$platform $symbol is advertised and unexpectedly linkable"
    else
      if ! grep -Eq 'Undefined symbols|symbol\(s\) not found' "$link_log"; then
        cat "$link_log" >&2
        record_failure "$platform $symbol link failed for an unrelated reason"
      fi
      printf 'CAUSAL_RED %s %s compile-success/link-fail\n' \
        "$platform" "$symbol" >&2
      record_failure \
        "$platform header advertises $symbol but the iOS archive cannot provide it"
    fi
    return
  fi

  if ! grep -F 'error:' "$syntax_log" \
      | grep -F 'no member named' \
      | grep -Fq "'$symbol'"; then
    cat "$syntax_log" >&2
    record_failure "$platform $symbol was rejected for an unrelated reason"
  fi
  if grep -Eqi \
      'file not found|no such file|could not build module|module map file' \
      "$syntax_log"; then
    cat "$syntax_log" >&2
    record_failure "$platform $symbol rejection came from a missing dependency"
  fi
}

for required_file in \
  "$HEADER" \
  "$IMPLEMENTATION" \
  "$SOURCE" \
  "$SIM_ARCHIVE" \
  "$DEVICE_ARCHIVE" \
  "$OSQP_SIM_ARCHIVE" \
  "$OSQP_DEVICE_ARCHIVE"; do
  if [ ! -f "$required_file" ]; then
    record_failure "required file is missing: $required_file"
  fi
done

if [ "$failures" -ne 0 ]; then
  exit 1
fi

for platform in simulator device; do
  expect_ios_api_hidden \
    "$platform" translate_markers BIOMOTION_OPEN_SIM_TRANSLATE_MARKERS \
    translateOsimMarkers
  expect_ios_api_hidden \
    "$platform" convert_sdf BIOMOTION_OPEN_SIM_CONVERT_SDF convertOsimToSDF
  expect_ios_api_hidden \
    "$platform" convert_mjcf BIOMOTION_OPEN_SIM_CONVERT_MJCF convertOsimToMJCF
done

if ! python3 - "$HEADER" "$IMPLEMENTATION" <<'PY'
from pathlib import Path
import re
import sys

header = Path(sys.argv[1]).read_text()
implementation = Path(sys.argv[2]).read_text()


class ContractError(RuntimeError):
    pass


def strip_comments(source):
    """Remove C++ comments without interpreting markers inside literals."""
    output = []
    index = 0
    state = "code"
    quote = ""
    while index < len(source):
        current = source[index]
        following = source[index + 1] if index + 1 < len(source) else ""
        if state == "code":
            raw = re.match(r'(?:u8|u|U|L)?R"([^ ()\\\t\r\n]{0,16})\(', source[index:])
            if raw:
                terminator = ")" + raw.group(1) + '"'
                end = source.find(terminator, index + raw.end())
                if end == -1:
                    raise ContractError("unterminated raw string literal")
                literal = source[index : end + len(terminator)]
                output.extend("\n" if char == "\n" else " " for char in literal)
                index = end + len(terminator)
                continue
            if current == "/" and following == "/":
                output.extend((" ", " "))
                index += 2
                state = "line_comment"
                continue
            if current == "/" and following == "*":
                output.extend((" ", " "))
                index += 2
                state = "block_comment"
                continue
            if current in ('"', "'"):
                quote = current
                output.append(current)
                index += 1
                state = "literal"
                continue
            output.append(current)
            index += 1
            continue
        if state == "line_comment":
            if current == "\n":
                output.append("\n")
                state = "code"
            else:
                output.append(" ")
            index += 1
            continue
        if state == "block_comment":
            if current == "*" and following == "/":
                output.extend((" ", " "))
                index += 2
                state = "code"
            else:
                output.append("\n" if current == "\n" else " ")
                index += 1
            continue
        output.append(current)
        index += 1
        if current == "\\" and index < len(source):
            output.append(source[index])
            index += 1
        elif current == quote:
            state = "code"

    if state == "block_comment":
        raise ContractError("unterminated block comment")
    if state == "literal":
        raise ContractError("unterminated string or character literal")
    return "".join(output)


def strip_literals(source):
    output = []
    index = 0
    quote = None
    while index < len(source):
        current = source[index]
        if quote is None and current in ('"', "'"):
            quote = current
            output.append(" ")
            index += 1
            continue
        if quote is None:
            output.append(current)
            index += 1
            continue
        output.append("\n" if current == "\n" else " ")
        index += 1
        if current == "\\" and index < len(source):
            output.append("\n" if source[index] == "\n" else " ")
            index += 1
        elif current == quote:
            quote = None
    if quote is not None:
        raise ContractError("unterminated literal after preprocessing")
    return "".join(output)


def evaluate_condition(kind, body, ios):
    body = body.strip()
    if kind in ("ifdef", "ifndef"):
        if not re.fullmatch(r"[A-Za-z_]\w*", body):
            raise ContractError(f"malformed #{kind}: {body}")
        if body == "DART_IOS_BUILD":
            return ios if kind == "ifdef" else not ios
        if "DART_IOS_BUILD" in body:
            raise ContractError(
                f"unsupported #{kind} involving DART_IOS_BUILD: {body}"
            )
        return None

    compact = re.sub(r"\s+", "", body)
    if not compact:
        raise ContractError(f"empty #{kind} condition")
    if compact == "0":
        return False
    if compact == "1":
        return True
    positive = {
        "DART_IOS_BUILD",
        "defined(DART_IOS_BUILD)",
        "definedDART_IOS_BUILD",
    }
    negative = {
        "!DART_IOS_BUILD",
        "!defined(DART_IOS_BUILD)",
        "!definedDART_IOS_BUILD",
    }
    if compact in positive:
        return ios
    if compact in negative:
        return not ios
    if "DART_IOS_BUILD" in compact:
        raise ContractError(
            f"unsupported condition involving DART_IOS_BUILD: {body}"
        )
    return None


def dart_ios_view(source, ios):
    source = strip_comments(source)
    directive_source = strip_literals(source)
    source_lines = source.splitlines()
    directive_lines = directive_source.splitlines()
    if len(source_lines) != len(directive_lines):
        raise ContractError("literal stripping changed the source line count")
    output = []
    active = True
    stack = []
    directive_pattern = re.compile(
        r"^\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b(.*)$"
    )
    for line_number, (line, directive_line) in enumerate(
        zip(source_lines, directive_lines), 1
    ):
        directive = directive_pattern.match(directive_line)
        if not directive:
            if active:
                output.append(line)
            continue
        kind = directive.group(1)
        body = directive.group(2).strip()
        if kind in ("if", "ifdef", "ifndef"):
            condition = evaluate_condition(kind, body, ios)
            stack.append(
                {
                    "parent": active,
                    "unknown": condition is None,
                    "taken": condition is True,
                    "else_seen": False,
                    "line": line_number,
                }
            )
            active = active and condition is not False
            continue
        if not stack:
            raise ContractError(f"unmatched #{kind} at line {line_number}")
        frame = stack[-1]
        if kind == "elif":
            if frame["else_seen"]:
                raise ContractError(f"#elif after #else at line {line_number}")
            condition = evaluate_condition("elif", body, ios)
            if frame["unknown"]:
                active = frame["parent"]
            elif frame["taken"]:
                active = False
            elif condition is None:
                frame["unknown"] = True
                active = frame["parent"]
            else:
                active = frame["parent"] and condition
                frame["taken"] = condition
            continue
        if kind == "else":
            if body:
                raise ContractError(f"unexpected tokens after #else: {body}")
            if frame["else_seen"]:
                raise ContractError(f"duplicate #else at line {line_number}")
            frame["else_seen"] = True
            active = frame["parent"] and (
                frame["unknown"] or not frame["taken"]
            )
            continue
        if body:
            raise ContractError(f"unexpected tokens after #endif: {body}")
        stack.pop()
        active = frame["parent"]

    if stack:
        raise ContractError(
            f"unterminated conditional opened at line {stack[-1]['line']}"
        )
    return "\n".join(output)


def require_once(source, pattern, label):
    count = len(re.findall(pattern, source, flags=re.DOTALL))
    if count != 1:
        raise ContractError(f"expected one {label}, found {count}")


def verify_contract(header_source, implementation_source):
    views = {}
    for platform, ios in (("ios", True), ("desktop", False)):
        views[f"{platform}_header"] = dart_ios_view(header_source, ios)
        views[f"{platform}_implementation"] = dart_ios_view(
            implementation_source, ios
        )

    unavailable = (
        "translateOsimMarkers",
        "convertOsimToSDF",
        "convertOsimToMJCF",
    )
    for symbol in unavailable:
        if symbol in strip_literals(views["ios_header"]):
            raise ContractError(f"iOS header still contains {symbol}")
        if symbol not in strip_literals(views["desktop_header"]):
            raise ContractError(f"non-iOS header lost {symbol}")
        definition = re.compile(rf"OpenSimParser::{re.escape(symbol)}\s*\(")
        if definition.search(strip_literals(views["ios_implementation"])):
            raise ContractError(f"iOS implementation still contains {symbol}")
        if not definition.search(strip_literals(views["desktop_implementation"])):
            raise ContractError(f"non-iOS implementation lost {symbol}")

    declarations = (
        (
            r"static\s+OpenSimFile\s+parseOsim\s*\(\s*const\s+common::Uri\s*&",
            "parseOsim(Uri) declaration",
        ),
        (
            r"static\s+OpenSimFile\s+parseOsim\s*\(\s*tinyxml2::XMLDocument\s*&",
            "parseOsim(XMLDocument) declaration",
        ),
        (r"static\s+OpenSimTRC\s+loadTRC\s*\(", "loadTRC declaration"),
        (r"static\s+OpenSimMot\s+loadMot\s*\(", "loadMot declaration"),
        (
            r"static\s+OpenSimMot\s+loadMotAtLowestMarkerRMSERotation\s*\(",
            "C3D rotation declaration",
        ),
        (r"static\s+std::vector\s*<\s*ForcePlate\s*>\s+loadGRF\s*\(", "loadGRF declaration"),
    )
    definitions = (
        (
            r"OpenSimFile\s+OpenSimParser::parseOsim\s*\(\s*const\s+common::Uri\s*&",
            "parseOsim(Uri) definition",
        ),
        (
            r"OpenSimFile\s+OpenSimParser::parseOsim\s*\(\s*tinyxml2::XMLDocument\s*&",
            "parseOsim(XMLDocument) definition",
        ),
        (r"OpenSimTRC\s+OpenSimParser::loadTRC\s*\(", "loadTRC definition"),
        (r"OpenSimMot\s+OpenSimParser::loadMot\s*\(", "loadMot definition"),
        (
            r"OpenSimMot\s+OpenSimParser::loadMotAtLowestMarkerRMSERotation\s*\(",
            "C3D rotation definition",
        ),
        (r"std::vector\s*<\s*ForcePlate\s*>\s+OpenSimParser::loadGRF\s*\(", "loadGRF definition"),
    )
    for platform in ("ios", "desktop"):
        header_view = strip_literals(views[f"{platform}_header"])
        implementation_view = strip_literals(
            views[f"{platform}_implementation"]
        )
        for pattern, label in declarations:
            require_once(header_view, pattern, f"{platform} {label}")
        for pattern, label in definitions:
            require_once(implementation_view, pattern, f"{platform} {label}")

    desktop_only_includes = (
        '#include "dart/biomechanics/MarkerFitter.hpp"',
        '#include "dart/utils/MJCFExporter.hpp"',
        '#include "dart/utils/sdf/SdfParser.hpp"',
    )
    for include in desktop_only_includes:
        if include in views["ios_implementation"]:
            raise ContractError(f"iOS implementation retains {include}")
        if include not in views["desktop_implementation"]:
            raise ContractError(f"non-iOS implementation lost {include}")

    active_implementation = strip_literals(strip_comments(implementation_source))
    if '#include "dart/server/GUIRecording.hpp"' in implementation_source:
        raise ContractError("unused GUIRecording.hpp include remains")
    if "GUIRecording" in active_implementation:
        raise ContractError("GUIRecording appears outside a comment")


def expect_contract_rejection(header_source, implementation_source, label):
    try:
        verify_contract(header_source, implementation_source)
    except ContractError:
        return
    raise ContractError(f"mutation self-test stayed green: {label}")


verify_contract(header, implementation)

declaration_pattern = re.compile(
    r"\s*static\s+OpenSimTRC\s+loadTRC\s*\(.*?\);", re.DOTALL
)
mutated_header, count = declaration_pattern.subn(
    lambda match: "\n/*" + match.group(0) + "*/", header, count=1
)
if count != 1:
    raise ContractError("could not construct declaration mutation")
expect_contract_rejection(
    mutated_header, implementation, "commented-out loadTRC declaration"
)

definition_pattern = re.compile(
    r"OpenSimTRC\s+OpenSimParser::loadTRC\s*\(.*?\)\s*(?=\{)", re.DOTALL
)
mutated_implementation, count = definition_pattern.subn(
    lambda match: "/*" + match.group(0) + "*/\n", implementation, count=1
)
if count != 1:
    raise ContractError("could not construct definition mutation")
expect_contract_rejection(
    header, mutated_implementation, "commented-out loadTRC definition"
)

for malformed in (
    "#if defined(DART_IOS_BUILD) && FEATURE\nvalue\n#endif\n",
    "#ifdef FEATURE_DART_IOS_BUILD\nvalue\n#endif\n",
    "#ifndef DART_IOS_BUILD\nvalue\n",
    "#else\nvalue\n#endif\n",
):
    try:
        dart_ios_view(malformed, True)
    except ContractError:
        continue
    raise ContractError(f"parser accepted malformed conditional: {malformed!r}")

elif_sample = """\
#if defined(DART_IOS_BUILD)
ios_branch
#elif 1
desktop_branch
#else
unreachable_branch
#endif
"""
if "ios_branch" not in dart_ios_view(elif_sample, True):
    raise ContractError("#if defined iOS branch self-test failed")
desktop_sample = dart_ios_view(elif_sample, False)
if "desktop_branch" not in desktop_sample or "ios_branch" in desktop_sample:
    raise ContractError("#elif desktop branch self-test failed")

print("OPENSIM_UTILS_SOURCE_CONTRACT_SELF_TEST_PASS")
PY
then
  record_failure \
    'OpenSim utility declarations/definitions are not paired across platforms'
fi

for platform in simulator device; do
  compile_argv_for "$platform"
  strict_log="$PROBE_TMP/OpenSimParser.${platform}.strict.log"
  if ! "${COMPILE_ARGV[@]}" -Wall -Wextra -Werror -Wno-sign-compare \
      -fsyntax-only "$IMPLEMENTATION" >"$strict_log" 2>&1; then
    cat "$strict_log" >&2
    record_failure \
      "OpenSimParser.cpp is not warning-clean for the iOS $platform target"
  fi
done

for archive in "$SIM_ARCHIVE" "$DEVICE_ARCHIVE"; do
  archive_label="$(basename "$(dirname "$archive")")"
  members="$PROBE_TMP/${archive_label}.members"
  defined="$PROBE_TMP/${archive_label}.defined"
  undefined="$PROBE_TMP/${archive_label}.undefined"
  ar -t "$archive" >"$members"
  nm -gU "$archive" | c++filt >"$defined"
  nm -gu "$archive" | c++filt >"$undefined"

  if [ "$(grep -Fxc 'OpenSimParser.cpp.o' "$members" || true)" -ne 1 ]; then
    record_failure \
      "$archive_label archive does not contain exactly one OpenSimParser.cpp.o"
  fi
  for symbol in \
    translateOsimMarkers convertOsimToSDF convertOsimToMJCF; do
    if grep -Fq "OpenSimParser::$symbol" "$defined" "$undefined"; then
      record_failure "$archive_label archive retains unavailable $symbol"
    fi
  done
  require_supported_symbols "$archive_label archive" "$defined"
done

for platform in simulator device; do
  compile_argv_for "$platform"
  archive="$(archive_for "$platform")"
  osqp_archive="$(osqp_archive_for "$platform")"
  object="$PROBE_TMP/runtime.${platform}.o"
  executable="$PROBE_TMP/runtime.${platform}"
  why_load="$PROBE_TMP/runtime.${platform}.why_load"
  map="$PROBE_TMP/runtime.${platform}.map"
  if ! "${COMPILE_ARGV[@]}" -Wall -Wextra -Werror -Wno-sign-compare \
      -c "$SOURCE" -o "$object" >"$PROBE_TMP/runtime.${platform}.compile" \
      2>&1; then
    cat "$PROBE_TMP/runtime.${platform}.compile" >&2
    record_failure "$platform supported-surface runtime probe did not compile"
    continue
  fi
  nm -gu "$object" | c++filt \
    >"$PROBE_TMP/runtime.${platform}.object.undefined"
  require_supported_symbols \
    "$platform positive-consumer object" \
    "$PROBE_TMP/runtime.${platform}.object.undefined"
  if ! "${COMPILE_ARGV[@]}" "$object" "$archive" "$osqp_archive" \
      -Wl,-dead_strip -Wl,-why_load -Wl,-map,"$map" \
      -o "$executable" 2>"$why_load"; then
    cat "$why_load" >&2
    record_failure "$platform ordinary dead-strip archive link failed"
    continue
  fi
  for receipt in "$why_load" "$map"; do
    if ! grep -Fq 'OpenSimParser.cpp.o' "$receipt"; then
      record_failure \
        "$platform dead-strip receipt omitted OpenSimParser.cpp.o"
    fi
  done
  awk '{ print $NF }' "$map" | c++filt \
    | grep -Ev ' \(\.cold\.[0-9]+\)$' | LC_ALL=C sort -u \
    >"$PROBE_TMP/runtime.${platform}.map.symbols"
  require_supported_symbols \
    "$platform dead-strip link map" \
    "$PROBE_TMP/runtime.${platform}.map.symbols"
  nm -gU "$executable" | c++filt \
    >"$PROBE_TMP/runtime.${platform}.defined"
  require_supported_symbols \
    "$platform linked executable" \
    "$PROBE_TMP/runtime.${platform}.defined"
  nm -gu "$executable" | c++filt \
    >"$PROBE_TMP/runtime.${platform}.undefined"
  if grep -Fq 'dart::' "$PROBE_TMP/runtime.${platform}.undefined"; then
    record_failure "$platform linked probe retains an unresolved DART symbol"
  fi
done

if [ -z "${SIMULATOR_UDID:-}" ]; then
  SIMULATOR_UDID="$({
    xcrun simctl list devices -j | python3 -c '
import json, sys
for runtime, devices in json.load(sys.stdin)["devices"].items():
    if "iOS" not in runtime:
        continue
    for device in devices:
        if device.get("isAvailable") and device.get("state") == "Booted":
            print(device["udid"])
            raise SystemExit(0)
raise SystemExit(1)
'
  } 2>/dev/null || true)"
fi

if [ -z "${SIMULATOR_UDID:-}" ]; then
  record_failure 'no booted, available iOS simulator was found'
elif [ -f "$PROBE_TMP/runtime.simulator" ]; then
  if output="$(
    xcrun simctl spawn "$SIMULATOR_UDID" \
      "$PROBE_TMP/runtime.simulator" 2>&1
  )"; then
    status=0
  else
    status=$?
  fi
  printf '%s\n' "$output"
  if [ "$status" -ne 0 ]; then
    record_failure "simulator runtime probe returned status $status"
  elif ! printf '%s\n' "$output" \
      | grep -Fxq 'OPENSIM_UTILS_IOS_BOUNDARY_PASS'; then
    record_failure 'simulator runtime probe omitted its pass sentinel'
  fi
fi

if [ "$failures" -ne 0 ]; then
  printf 'OpenSim utilities iOS boundary found %s contract failure(s)\n' \
    "$failures" >&2
  exit 1
fi

printf '%s\n' 'OPENSIM_UTILS_IOS_ARCHIVES_PASS'
