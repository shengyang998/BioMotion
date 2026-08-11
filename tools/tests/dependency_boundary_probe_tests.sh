#!/bin/bash -p
set -euo pipefail
PATH=/usr/bin:/bin:/usr/sbin:/sbin
export PATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
INSPECTOR="$REPO_ROOT/tools/release/dependency_boundary.py"
TEST_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/biomotion-dependency-tests.XXXXXX")"
TEST_ROOT="$(cd "$TEST_ROOT" && pwd -P)"
trap 'rm -r "$TEST_ROOT" 2>/dev/null || true' EXIT

if [ ! -f "$INSPECTOR" ] || [ -L "$INSPECTOR" ]; then
  printf 'dependency-boundary inspector is missing or a symlink: %s\n' \
    "$INSPECTOR" >&2
  exit 2
fi

FIXTURE_ROOT="$TEST_ROOT/repository"
ARCHIVE_TEMPLATES="$TEST_ROOT/archive-templates"
pass_count=0
total_count=0

mkdir -p "$ARCHIVE_TEMPLATES"
cat > "$ARCHIVE_TEMPLATES/osqp_symbols.c" <<'EOF'
void osqp_setup(void) {}
void osqp_solve(void) {}
void osqp_cleanup(void) {}
#ifndef OMIT_WARM_START
void osqp_warm_start(void) {}
#endif
EOF
cat > "$ARCHIVE_TEMPLATES/osqp_symbols_drift.c" <<'EOF'
void osqp_setup(void) {}
void osqp_solve(void) {}
void osqp_cleanup(void) {}
void osqp_warm_start(void) {}
int biomotion_unreviewed_implementation(void) { return 1; }
EOF
/usr/bin/xcrun --sdk iphoneos clang -target arm64-apple-ios17.0 \
  -c "$ARCHIVE_TEMPLATES/osqp_symbols.c" \
  -o "$ARCHIVE_TEMPLATES/device.o"
/usr/bin/xcrun --sdk iphonesimulator clang \
  -target arm64-apple-ios17.0-simulator \
  -c "$ARCHIVE_TEMPLATES/osqp_symbols.c" \
  -o "$ARCHIVE_TEMPLATES/simulator.o"
/usr/bin/xcrun --sdk macosx clang -target x86_64-apple-macos13.0 \
  -c "$ARCHIVE_TEMPLATES/osqp_symbols.c" \
  -o "$ARCHIVE_TEMPLATES/x86_64.o"
/usr/bin/xcrun --sdk macosx clang -target arm64-apple-macos13.0 \
  -c "$ARCHIVE_TEMPLATES/osqp_symbols.c" \
  -o "$ARCHIVE_TEMPLATES/arm64-macos.o"
/usr/bin/xcrun --sdk iphoneos clang -target arm64-apple-ios18.0 \
  -c "$ARCHIVE_TEMPLATES/osqp_symbols.c" \
  -o "$ARCHIVE_TEMPLATES/ios18.o"
/usr/bin/xcrun --sdk iphoneos clang -target arm64-apple-ios17.0 \
  -c "$ARCHIVE_TEMPLATES/osqp_symbols_drift.c" \
  -o "$ARCHIVE_TEMPLATES/drift.o"
/usr/bin/xcrun --sdk iphoneos clang -target arm64-apple-ios17.0 \
  -DOMIT_WARM_START -c "$ARCHIVE_TEMPLATES/osqp_symbols.c" \
  -o "$ARCHIVE_TEMPLATES/missing-symbol.o"
/usr/bin/xcrun ar rcs "$ARCHIVE_TEMPLATES/device.a" \
  "$ARCHIVE_TEMPLATES/device.o"
/usr/bin/xcrun ar rcs "$ARCHIVE_TEMPLATES/simulator.a" \
  "$ARCHIVE_TEMPLATES/simulator.o"
/usr/bin/xcrun ar rcs "$ARCHIVE_TEMPLATES/x86_64.a" \
  "$ARCHIVE_TEMPLATES/x86_64.o"
/usr/bin/xcrun ar rcs "$ARCHIVE_TEMPLATES/arm64-macos.a" \
  "$ARCHIVE_TEMPLATES/arm64-macos.o"
/usr/bin/xcrun ar rcs "$ARCHIVE_TEMPLATES/ios18.a" \
  "$ARCHIVE_TEMPLATES/ios18.o"
/usr/bin/xcrun ar rcs "$ARCHIVE_TEMPLATES/drift.a" \
  "$ARCHIVE_TEMPLATES/drift.o"
/usr/bin/xcrun ar rcs "$ARCHIVE_TEMPLATES/missing-symbol.a" \
  "$ARCHIVE_TEMPLATES/missing-symbol.o"

write_osqp_cache() {
  path=$1
  sysroot=$2
  build_root="$(dirname "$path")"
  mkdir -p "$(dirname "$path")"
  cat > "$path" <<EOF
CMAKE_BUILD_TYPE:STRING=Release
CMAKE_C_FLAGS:STRING=-ffile-prefix-map=$FIXTURE_ROOT/osqp=osqp -ffile-prefix-map=$build_root/_deps/qdldl-src=qdldl
CMAKE_C_FLAGS_RELEASE:STRING=-O3 -DNDEBUG
CMAKE_GENERATOR:INTERNAL=Ninja
CMAKE_HOME_DIRECTORY:INTERNAL=$FIXTURE_ROOT/osqp
CMAKE_SYSTEM_NAME:STRING=iOS
CMAKE_OSX_ARCHITECTURES:STRING=arm64
CMAKE_OSX_DEPLOYMENT_TARGET:STRING=17.0
CMAKE_OSX_SYSROOT:STRING=$sysroot
OSQP_BUILD_DEMO_EXE:BOOL=OFF
OSQP_BUILD_SHARED_LIB:BOOL=OFF
OSQP_BUILD_STATIC_LIB:BOOL=ON
OSQP_BUILD_UNITTESTS:BOOL=OFF
OSQP_ALGEBRA_BACKEND:STRING=builtin
OSQP_ASAN:BOOL=OFF
OSQP_CODEGEN:BOOL=ON
OSQP_ENABLE_DERIVATIVES:BOOL=ON
OSQP_ENABLE_INTERRUPT:BOOL=ON
OSQP_ENABLE_PRINTING:BOOL=ON
OSQP_ENABLE_PROFILING:BOOL=ON
OSQP_PACK_SETTINGS:BOOL=OFF
OSQP_PROFILER_ANNOTATIONS:STRING=OFF
OSQP_USE_FLOAT:BOOL=OFF
OSQP_USE_LONG:BOOL=OFF
QDLDL_BUILD_SHARED_LIB:BOOL=OFF
QDLDL_BUILD_STATIC_LIB:BOOL=OFF
QDLDL_DEV_ANALYSIS:BOOL=OFF
QDLDL_DEV_ASAN:BOOL=OFF
QDLDL_DEV_COVERAGE:BOOL=OFF
QDLDL_FLOAT:BOOL=OFF
QDLDL_LONG:BOOL=OFF
EOF
}

write_nimble_cache() {
  path=$1
  sysroot=$2
  mkdir -p "$(dirname "$path")"
  cat > "$path" <<EOF
CMAKE_BUILD_TYPE:STRING=Release
CMAKE_GENERATOR:INTERNAL=Ninja
CMAKE_HOME_DIRECTORY:INTERNAL=$FIXTURE_ROOT/nimblephysics/ios
CMAKE_SYSTEM_NAME:STRING=iOS
CMAKE_OSX_ARCHITECTURES:STRING=arm64
CMAKE_OSX_DEPLOYMENT_TARGET:STRING=17.0
CMAKE_OSX_SYSROOT:STRING=$sysroot
NIMBLE_IOS_HOST_PROBE:BOOL=OFF
EOF
}

write_lock() {
  commit=$1
  nimble_commit="$(git -C "$FIXTURE_ROOT/nimblephysics" rev-parse HEAD)"
  qdldl_commit="$(git -C "$FIXTURE_ROOT/osqp/build_ios/_deps/qdldl-src" rev-parse HEAD)"
  mkdir -p "$FIXTURE_ROOT/tools"
  /usr/bin/python3 - "$REPO_ROOT/tools/dependencies.lock.json" \
    "$FIXTURE_ROOT/tools/dependencies.lock.json" "$commit" \
    "$nimble_commit" \
    "$FIXTURE_ROOT/nimblephysics/build_ios/libnimble_ios.a" \
    "$FIXTURE_ROOT/nimblephysics/build_sim/libnimble_ios.a" \
    "$FIXTURE_ROOT" "$qdldl_commit" <<'PY'
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def archive_content(path):
    names = subprocess.run(
        ["/usr/bin/xcrun", "ar", "-t", str(path)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    digest = hashlib.sha256(b"BioMotion static archive members v1\0")
    count = 0
    for name in names:
        if name.startswith("__.SYMDEF"):
            continue
        data = subprocess.run(
            ["/usr/bin/xcrun", "ar", "-p", str(path), name],
            check=True,
            capture_output=True,
        ).stdout
        encoded = name.encode("utf-8")
        digest.update(len(encoded).to_bytes(8, "big"))
        digest.update(encoded)
        digest.update(len(data).to_bytes(8, "big"))
        digest.update(data)
        count += 1
    return count, digest.hexdigest()

source = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
source["dependencies"]["osqp"]["commit"] = sys.argv[3]
nimble = source["dependencies"]["nimblephysics"]
nimble["commit"] = sys.argv[4]
for build, archive_path in zip(nimble["builds"], sys.argv[5:7]):
    build["sha256"] = sha256(Path(archive_path))
root = Path(sys.argv[7])
for build in nimble["builds"]:
    build["generatedHeaderSHA256"] = sha256(root / build["generatedHeader"])
osqp = source["dependencies"]["osqp"]
osqp["qdldlCommit"] = sys.argv[8]
for build in osqp["builds"]:
    count, digest = archive_content(root / build["archive"])
    build["archiveMemberCount"] = count
    build["archiveContentSHA256"] = digest
    build["generatedHeaderSHA256"] = sha256(root / build["generatedHeader"])
Path(sys.argv[2]).write_text(
    json.dumps(source, indent=2, sort_keys=False) + "\n",
    encoding="utf-8",
)
PY
}

set_build_field() {
  build_name=$1
  field=$2
  json_value=$3
  /usr/bin/python3 - "$FIXTURE_ROOT/tools/dependencies.lock.json" \
    "$build_name" "$field" "$json_value" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
value = json.loads(path.read_text(encoding="utf-8"))
build = next(
    item
    for item in value["dependencies"]["osqp"]["builds"]
    if item["name"] == sys.argv[2]
)
build[sys.argv[3]] = json.loads(sys.argv[4])
path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
PY
}

make_fixture() {
  rm -r "$FIXTURE_ROOT" 2>/dev/null || true
  mkdir -p \
    "$FIXTURE_ROOT/osqp" \
    "$FIXTURE_ROOT/nimblephysics/ios" \
    "$FIXTURE_ROOT/tools/release"
  printf '%s\n' 'build_ios/' 'build_sim/' > "$FIXTURE_ROOT/osqp/.gitignore"
  printf '%s\n' 'fixture source' > "$FIXTURE_ROOT/osqp/source.txt"
  git -C "$FIXTURE_ROOT/osqp" init -q
  git -C "$FIXTURE_ROOT/osqp" config user.name 'BioMotion Test'
  git -C "$FIXTURE_ROOT/osqp" config user.email 'biomotion-test@example.invalid'
  git -C "$FIXTURE_ROOT/osqp" add .gitignore source.txt
  git -C "$FIXTURE_ROOT/osqp" commit -q -m fixture
  git -C "$FIXTURE_ROOT/osqp" remote add origin \
    https://github.com/osqp/osqp.git

  write_osqp_cache "$FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt" iphoneos
  write_osqp_cache "$FIXTURE_ROOT/osqp/build_sim/CMakeCache.txt" iphonesimulator
  mkdir -p \
    "$FIXTURE_ROOT/osqp/build_ios/out" \
    "$FIXTURE_ROOT/osqp/build_sim/out"
  cp "$ARCHIVE_TEMPLATES/device.a" \
    "$FIXTURE_ROOT/osqp/build_ios/out/libosqpstatic.a"
  cp "$ARCHIVE_TEMPLATES/simulator.a" \
    "$FIXTURE_ROOT/osqp/build_sim/out/libosqpstatic.a"
  mkdir -p \
    "$FIXTURE_ROOT/osqp/build_ios/include/public" \
    "$FIXTURE_ROOT/osqp/build_sim/include/public"
  printf '%s\n' 'fixture OSQP generated config' \
    > "$FIXTURE_ROOT/osqp/build_ios/include/public/osqp_configure.h"
  cp "$FIXTURE_ROOT/osqp/build_ios/include/public/osqp_configure.h" \
    "$FIXTURE_ROOT/osqp/build_sim/include/public/osqp_configure.h"

  mkdir -p "$FIXTURE_ROOT/osqp/build_ios/_deps/qdldl-src"
  printf '%s\n' 'fixture QDLDL source' \
    > "$FIXTURE_ROOT/osqp/build_ios/_deps/qdldl-src/source.txt"
  git -C "$FIXTURE_ROOT/osqp/build_ios/_deps/qdldl-src" init -q
  git -C "$FIXTURE_ROOT/osqp/build_ios/_deps/qdldl-src" config user.name \
    'BioMotion Test'
  git -C "$FIXTURE_ROOT/osqp/build_ios/_deps/qdldl-src" config user.email \
    'biomotion-test@example.invalid'
  git -C "$FIXTURE_ROOT/osqp/build_ios/_deps/qdldl-src" add source.txt
  git -C "$FIXTURE_ROOT/osqp/build_ios/_deps/qdldl-src" commit -q -m fixture
  git -C "$FIXTURE_ROOT/osqp/build_ios/_deps/qdldl-src" remote add origin \
    https://github.com/osqp/qdldl.git
  mkdir -p "$FIXTURE_ROOT/osqp/build_sim/_deps"
  git clone -q "$FIXTURE_ROOT/osqp/build_ios/_deps/qdldl-src" \
    "$FIXTURE_ROOT/osqp/build_sim/_deps/qdldl-src"
  git -C "$FIXTURE_ROOT/osqp/build_sim/_deps/qdldl-src" remote set-url origin \
    https://github.com/osqp/qdldl.git

  printf '%s\n' 'build_ios/' 'build_sim/' \
    > "$FIXTURE_ROOT/nimblephysics/.gitignore"
  printf '%s\n' 'fixture iOS source' \
    > "$FIXTURE_ROOT/nimblephysics/ios/source.txt"
  git -C "$FIXTURE_ROOT/nimblephysics" init -q
  git -C "$FIXTURE_ROOT/nimblephysics" config user.name 'BioMotion Test'
  git -C "$FIXTURE_ROOT/nimblephysics" config user.email \
    'biomotion-test@example.invalid'
  git -C "$FIXTURE_ROOT/nimblephysics" add .gitignore ios/source.txt
  git -C "$FIXTURE_ROOT/nimblephysics" commit -q -m fixture
  git -C "$FIXTURE_ROOT/nimblephysics" remote add origin \
    https://github.com/shengyang998/nimblephysics.git
  write_nimble_cache \
    "$FIXTURE_ROOT/nimblephysics/build_ios/CMakeCache.txt" iphoneos
  write_nimble_cache \
    "$FIXTURE_ROOT/nimblephysics/build_sim/CMakeCache.txt" iphonesimulator
  cp "$ARCHIVE_TEMPLATES/device.a" \
    "$FIXTURE_ROOT/nimblephysics/build_ios/libnimble_ios.a"
  cp "$ARCHIVE_TEMPLATES/simulator.a" \
    "$FIXTURE_ROOT/nimblephysics/build_sim/libnimble_ios.a"
  mkdir -p \
    "$FIXTURE_ROOT/nimblephysics/build_ios/dart" \
    "$FIXTURE_ROOT/nimblephysics/build_sim/dart"
  printf '%s\n' 'fixture Nimble generated config' \
    > "$FIXTURE_ROOT/nimblephysics/build_ios/dart/config.hpp"
  cp "$FIXTURE_ROOT/nimblephysics/build_ios/dart/config.hpp" \
    "$FIXTURE_ROOT/nimblephysics/build_sim/dart/config.hpp"
  cp "$REPO_ROOT/project.yml" "$FIXTURE_ROOT/project.yml"
  mkdir -p "$FIXTURE_ROOT/BioMotion.xcodeproj/project.xcworkspace"
  cp "$REPO_ROOT/BioMotion.xcodeproj/project.pbxproj" \
    "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj"
  cp "$REPO_ROOT/BioMotion.xcodeproj/project.xcworkspace/contents.xcworkspacedata" \
    "$FIXTURE_ROOT/BioMotion.xcodeproj/project.xcworkspace/contents.xcworkspacedata"
  cp "$REPO_ROOT/tools/release/reject_dev_model.sh" \
    "$FIXTURE_ROOT/tools/release/reject_dev_model.sh"
  commit="$(git -C "$FIXTURE_ROOT/osqp" rev-parse HEAD)"
  write_lock "$commit"
}

expect_pass() {
  label=$1
  total_count=$((total_count + 1))
  set +e
  output="$(/usr/bin/python3 -I "$INSPECTOR" "$FIXTURE_ROOT" 2>&1)"
  status=$?
  set -e
  if [ "$status" -ne 0 ]; then
    printf '%s unexpectedly failed: %s\n' "$label" "$output" >&2
    exit 1
  fi
  expected="DEPENDENCY_BOUNDARY_PASS nimble=$(git -C "$FIXTURE_ROOT/nimblephysics" rev-parse HEAD) osqp=$(git -C "$FIXTURE_ROOT/osqp" rev-parse HEAD)"
  if [ "$output" != "$expected" ]; then
    printf '%s changed the pass sentinel: %s\n' "$label" "$output" >&2
    exit 1
  fi
  pass_count=$((pass_count + 1))
}

expect_failure() {
  label=$1
  expected=$2
  total_count=$((total_count + 1))
  set +e
  output="$(/usr/bin/python3 -I "$INSPECTOR" "$FIXTURE_ROOT" 2>&1)"
  status=$?
  set -e
  if [ "$status" -eq 0 ]; then
    printf '%s unexpectedly passed\n' "$label" >&2
    exit 1
  fi
  case "$output" in
    *"$expected"*) pass_count=$((pass_count + 1)) ;;
    *)
      printf '%s failed for the wrong reason: %s\n' "$label" "$output" >&2
      exit 1
      ;;
  esac
}

make_fixture
expect_pass baseline

SNAPSHOT_ONE="$TEST_ROOT/dependency-snapshot-one.json"
SNAPSHOT_TWO="$TEST_ROOT/dependency-snapshot-two.json"
/usr/bin/python3 -I "$INSPECTOR" snapshot "$FIXTURE_ROOT" > "$SNAPSHOT_ONE"
/usr/bin/python3 -I "$INSPECTOR" snapshot "$FIXTURE_ROOT" > "$SNAPSHOT_TWO"

total_count=$((total_count + 1))
if ! cmp -s "$SNAPSHOT_ONE" "$SNAPSHOT_TWO"; then
  printf 'dependency snapshot is not deterministic\n' >&2
  exit 1
fi
if ! /usr/bin/python3 - "$SNAPSHOT_ONE" "$FIXTURE_ROOT" <<'PY'
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

snapshot_path = Path(sys.argv[1])
root = Path(sys.argv[2])
raw = snapshot_path.read_bytes()
if raw.count(b"\n") != 1 or not raw.endswith(b"\n"):
    raise SystemExit("snapshot must contain exactly one JSON line")
value = json.loads(raw)
canonical = (
    json.dumps(value, ensure_ascii=True, separators=(",", ":"), sort_keys=True)
    + "\n"
).encode("utf-8")
if raw != canonical:
    raise SystemExit("snapshot is not canonical JSON")
if type(value.get("schemaVersion")) is not int or value["schemaVersion"] != 1:
    raise SystemExit("snapshot schema is not integer 1")
lock_path = root / "tools" / "dependencies.lock.json"
lock = json.loads(lock_path.read_text(encoding="utf-8"))
if value["dependencyLockSHA256"] != hashlib.sha256(lock_path.read_bytes()).hexdigest():
    raise SystemExit("snapshot does not bind raw lock bytes")

dependencies = value["dependencies"]
nimble = dependencies["nimblephysics"]
osqp = dependencies["osqp"]
git = lambda repo: subprocess.run(
    ["/usr/bin/git", "-C", str(repo), "rev-parse", "HEAD"],
    check=True,
    capture_output=True,
    text=True,
).stdout.strip()
if nimble["head"] != git(root / "nimblephysics"):
    raise SystemExit("snapshot Nimble HEAD is not observed")
if osqp["head"] != git(root / "osqp"):
    raise SystemExit("snapshot OSQP HEAD is not observed")
if nimble["repository"] != "https://github.com/shengyang998/nimblephysics":
    raise SystemExit("snapshot Nimble repository is not canonical")
if osqp["repository"] != "https://github.com/osqp/osqp":
    raise SystemExit("snapshot OSQP repository is not canonical")

locked_nimble = {item["name"]: item for item in lock["dependencies"]["nimblephysics"]["builds"]}
locked_osqp = {item["name"]: item for item in lock["dependencies"]["osqp"]["builds"]}
common_cmake = {
    "CMAKE_BUILD_TYPE",
    "CMAKE_GENERATOR",
    "CMAKE_HOME_DIRECTORY",
    "CMAKE_OSX_ARCHITECTURES",
    "CMAKE_OSX_DEPLOYMENT_TARGET",
    "CMAKE_OSX_SYSROOT",
    "CMAKE_SYSTEM_NAME",
}
nimble_cmake = common_cmake | {"NIMBLE_IOS_HOST_PROBE"}
osqp_cmake = common_cmake | {
    "CMAKE_C_FLAGS",
    "CMAKE_C_FLAGS_RELEASE",
    "OSQP_ALGEBRA_BACKEND",
    "OSQP_ASAN",
    "OSQP_BUILD_DEMO_EXE",
    "OSQP_BUILD_SHARED_LIB",
    "OSQP_BUILD_STATIC_LIB",
    "OSQP_BUILD_UNITTESTS",
    "OSQP_CODEGEN",
    "OSQP_ENABLE_DERIVATIVES",
    "OSQP_ENABLE_INTERRUPT",
    "OSQP_ENABLE_PRINTING",
    "OSQP_ENABLE_PROFILING",
    "OSQP_PACK_SETTINGS",
    "OSQP_PROFILER_ANNOTATIONS",
    "OSQP_USE_FLOAT",
    "OSQP_USE_LONG",
    "QDLDL_BUILD_SHARED_LIB",
    "QDLDL_BUILD_STATIC_LIB",
    "QDLDL_DEV_ANALYSIS",
    "QDLDL_DEV_ASAN",
    "QDLDL_DEV_COVERAGE",
    "QDLDL_FLOAT",
    "QDLDL_LONG",
}
for sdk in ("device", "simulator"):
    observed = nimble["builds"][sdk]
    locked = locked_nimble[sdk]
    if observed["archiveSHA256"] != locked["sha256"]:
        raise SystemExit(f"snapshot Nimble {sdk} archive digest drifted")
    if observed["generatedHeaderSHA256"] != locked["generatedHeaderSHA256"]:
        raise SystemExit(f"snapshot Nimble {sdk} header digest drifted")
    if set(observed["cmake"]) != nimble_cmake:
        raise SystemExit(f"snapshot Nimble {sdk} CMake fields are incomplete")
    if observed["cmake"]["CMAKE_HOME_DIRECTORY"] != "nimblephysics/ios":
        raise SystemExit(f"snapshot Nimble {sdk} CMake home is not normalized")

    observed = osqp["builds"][sdk]
    locked = locked_osqp[sdk]
    if observed["archiveContentSHA256"] != locked["archiveContentSHA256"]:
        raise SystemExit(f"snapshot OSQP {sdk} content digest drifted")
    if observed["archiveMemberCount"] != locked["archiveMemberCount"]:
        raise SystemExit(f"snapshot OSQP {sdk} member count drifted")
    if observed["generatedHeaderSHA256"] != locked["generatedHeaderSHA256"]:
        raise SystemExit(f"snapshot OSQP {sdk} header digest drifted")
    if set(observed["cmake"]) != osqp_cmake:
        raise SystemExit(f"snapshot OSQP {sdk} CMake fields are incomplete")
    if observed["cmake"]["CMAKE_HOME_DIRECTORY"] != "osqp":
        raise SystemExit(f"snapshot OSQP {sdk} CMake home is not normalized")

qdldl_commit = lock["dependencies"]["osqp"]["qdldlCommit"]
for sdk in ("device", "simulator"):
    checkout = osqp["qdldl"][sdk]
    if checkout != {
        "head": qdldl_commit,
        "repository": "https://github.com/osqp/qdldl",
    }:
        raise SystemExit(f"snapshot QDLDL {sdk} identity drifted")

linkage = value["projectLinkage"]
expected_project = hashlib.sha256((root / "project.yml").read_bytes()).hexdigest()
expected_pbx = hashlib.sha256(
    (root / "BioMotion.xcodeproj" / "project.pbxproj").read_bytes()
).hexdigest()
if linkage["projectYMLSHA256"] != expected_project:
    raise SystemExit("snapshot project.yml identity drifted")
if linkage["projectPBXProjSHA256"] != expected_pbx:
    raise SystemExit("snapshot PBX identity drifted")
if re.fullmatch(r"[0-9a-f]{64}", linkage["normalizedLinkageSHA256"]) is None:
    raise SystemExit("snapshot normalized linkage digest is malformed")
PY
then
  printf 'dependency snapshot fields are incomplete or incorrect\n' >&2
  exit 1
fi
pass_count=$((pass_count + 1))

total_count=$((total_count + 1))
if ! /usr/bin/python3 - "$SNAPSHOT_ONE" "$FIXTURE_ROOT" <<'PY'
import json
from pathlib import Path
import sys

raw = Path(sys.argv[1]).read_bytes()
root = str(Path(sys.argv[2]).resolve())
if root.encode("utf-8") in raw:
    raise SystemExit("snapshot leaks its repository root")
value = json.loads(raw)
def visit(item):
    if isinstance(item, dict):
        for child in item.values():
            visit(child)
    elif isinstance(item, list):
        for child in item:
            visit(child)
    elif isinstance(item, str) and item.startswith(root):
        raise SystemExit("snapshot contains an absolute repository path")
visit(value)
PY
then
  printf 'dependency snapshot contains an absolute repository path\n' >&2
  exit 1
fi
pass_count=$((pass_count + 1))

total_count=$((total_count + 1))
INJECTED_SNAPSHOT="$TEST_ROOT/dependency-snapshot-injected.json"
if ! /usr/bin/python3 -I - "$INSPECTOR" "$FIXTURE_ROOT" > "$INJECTED_SNAPSHOT" <<'PY'
import os
import runpy
import sys

script, root = sys.argv[1:]
os.environ.update({
    "DEVELOPER_DIR": "/unreviewed/Xcode.app/Contents/Developer",
    "GIT_CONFIG_GLOBAL": "/unreviewed/gitconfig",
    "GIT_DIR": "/unreviewed/git-dir",
    "GIT_WORK_TREE": "/unreviewed/work-tree",
    "PYTHONHOME": "/unreviewed/python-home",
    "PYTHONINSPECT": "1",
    "PYTHONPATH": "/unreviewed/python-path",
    "SDKROOT": "/unreviewed/sdk",
    "TOOLCHAINS": "unreviewed-toolchain",
})
sys.argv = [script, "snapshot", root]
try:
    runpy.run_path(script, run_name="__main__")
finally:
    os.environ.pop("PYTHONINSPECT", None)
PY
then
  printf 'dependency snapshot rejected a sanitized injected environment\n' >&2
  exit 1
fi
if ! cmp -s "$SNAPSHOT_ONE" "$INJECTED_SNAPSHOT"; then
  printf 'dependency snapshot changed under injected environment\n' >&2
  exit 1
fi
pass_count=$((pass_count + 1))

write_lock 0000000000000000000000000000000000000000
expect_failure wrong_commit 'OSQP HEAD does not match lock'

make_fixture
printf '%s\n' dirty > "$FIXTURE_ROOT/osqp/untracked.txt"
expect_failure dirty_checkout 'OSQP checkout is dirty'

make_fixture
git -C "$FIXTURE_ROOT/osqp" remote set-url origin \
  https://example.invalid/not-osqp.git
expect_failure wrong_origin 'OSQP repository does not match lock'

make_fixture
sed -i '' 's/CMAKE_OSX_SYSROOT:STRING=iphoneos/CMAKE_OSX_SYSROOT:STRING=macosx/' \
  "$FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt"
expect_failure wrong_device_sysroot 'device CMAKE_OSX_SYSROOT expected iphoneos'

make_fixture
rm "$FIXTURE_ROOT/osqp/build_sim/out/libosqpstatic.a"
expect_failure missing_simulator_archive 'simulator archive is missing'

make_fixture
sed -i '' 's/"schemaVersion": 1,/"schemaVersion": 1,\
  "schemaVersion": 1,/' "$FIXTURE_ROOT/tools/dependencies.lock.json"
expect_failure duplicate_lock_key 'dependency lock repeats key: schemaVersion'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/tools/dependencies.lock.json" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
value = json.loads(path.read_text(encoding="utf-8"))
value["dependencies"]["osqp"]["unexpected"] = True
path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
PY
expect_failure extra_osqp_key 'dependency lock osqp entry has unexpected keys'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/tools/dependencies.lock.json" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
value = json.loads(path.read_text(encoding="utf-8"))
value["dependencies"]["osqp"]["builds"][0]["unexpected"] = True
path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
PY
expect_failure extra_build_key 'device build entry has unexpected keys'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/tools/dependencies.lock.json" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
value = json.loads(path.read_text(encoding="utf-8"))
value["schemaVersion"] = True
path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
PY
expect_failure boolean_schema_version \
  'dependency lock schemaVersion must be integer 1'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/tools/dependencies.lock.json" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
value = json.loads(path.read_text(encoding="utf-8"))
value["dependencies"]["osqp"]["qdldlRepository"] = True
path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
PY
expect_failure boolean_qdldl_repository \
  'dependency lock osqp qdldlRepository must be a non-empty string'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/tools/dependencies.lock.json" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
value = json.loads(path.read_text(encoding="utf-8"))
value["dependencies"]["osqp"]["builds"][0]["name"] = []
path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
PY
expect_failure unhashable_build_name \
  'dependency lock osqp build entry 0 name must be a string'

make_fixture
set_build_field device cache '"../outside/CMakeCache.txt"'
expect_failure traversal_cache_path \
  'device cache path must be repository-relative without traversal'

make_fixture
sed -i '' \
  's|osqp/build_ios/out/libosqpstatic.a|osqp/build_device/out/libosqpstatic.a|' \
  "$FIXTURE_ROOT/project.yml"
expect_failure project_device_path_drift \
  'project.yml BioMotion device linker inputs changed'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/project.yml" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
old = '          - "$(PROJECT_DIR)/osqp/build_ios/include/public"'
new = '          - "$(PROJECT_DIR)/osqp/build_wrong/include/public"'
if text.count(old) != 1:
    raise SystemExit("fixture project has an unexpected OSQP device header count")
path.write_text(text.replace(old, new), encoding="utf-8")
PY
expect_failure project_header_order_drift \
  'project.yml BioMotion device header search paths changed'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/project.yml" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
old = '"EIGEN_MPL2_ONLY=1"'
if old not in text:
    raise SystemExit("fixture project lacks the pinned Eigen definition")
path.write_text(text.replace(old, '"EIGEN_MPL2_ONLY=0"', 1), encoding="utf-8")
PY
expect_failure project_preprocessor_drift \
  'project.yml BioMotion base preprocessor definitions changed'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/project.yml" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
needle = (
    "        SWIFT_OBJC_BRIDGING_HEADER: "
    "BioMotion/Nimble/BioMotion-Bridging-Header.h"
)
replacement = needle + '\n        USER_HEADER_SEARCH_PATHS:\n          - "/tmp/unreviewed"'
if text.count(needle) != 1:
    raise SystemExit("fixture project app bridge setting is not unique")
path.write_text(text.replace(needle, replacement), encoding="utf-8")
PY
expect_failure project_competing_header_setting \
  'project.yml BioMotion contains competing dependency settings: USER_HEADER_SEARCH_PATHS'

make_fixture
sed -i '' 's/OSQP_BUILD_DEMO_EXE:BOOL=OFF/OSQP_BUILD_DEMO_EXE:BOOL=ON/' \
  "$FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt"
expect_failure demo_enabled 'device OSQP_BUILD_DEMO_EXE expected OFF'

make_fixture
rm "$FIXTURE_ROOT/osqp/build_sim/out/libosqpstatic.a"
ln -s "$FIXTURE_ROOT/osqp/build_ios/out/libosqpstatic.a" \
  "$FIXTURE_ROOT/osqp/build_sim/out/libosqpstatic.a"
expect_failure symlink_archive \
  'simulator archive path must not traverse symlinks'

make_fixture
cp "$ARCHIVE_TEMPLATES/x86_64.a" \
  "$FIXTURE_ROOT/osqp/build_ios/out/libosqpstatic.a"
expect_failure wrong_archive_architecture \
  'device archive architecture expected arm64'

make_fixture
cp "$ARCHIVE_TEMPLATES/missing-symbol.a" \
  "$FIXTURE_ROOT/osqp/build_ios/out/libosqpstatic.a"
expect_failure missing_archive_symbol \
  'device archive is missing required symbols: _osqp_warm_start'

make_fixture
cp "$ARCHIVE_TEMPLATES/drift.a" \
  "$FIXTURE_ROOT/osqp/build_ios/out/libosqpstatic.a"
expect_failure unreviewed_archive_content \
  'device archive content SHA-256 does not match lock'

make_fixture
printf '%s\n' 'not Mach-O' > "$FIXTURE_ROOT/osqp/build_ios/out/not_macho.txt"
/usr/bin/ar qS "$FIXTURE_ROOT/osqp/build_ios/out/libosqpstatic.a" \
  "$FIXTURE_ROOT/osqp/build_ios/out/not_macho.txt"
expect_failure non_macho_archive_member \
  'device archive member inventory does not match Mach-O inspection'

make_fixture
cp "$ARCHIVE_TEMPLATES/arm64-macos.a" \
  "$FIXTURE_ROOT/osqp/build_ios/out/libosqpstatic.a"
expect_failure wrong_archive_platform 'device archive platform expected 2'

make_fixture
cp "$ARCHIVE_TEMPLATES/ios18.a" \
  "$FIXTURE_ROOT/osqp/build_ios/out/libosqpstatic.a"
expect_failure wrong_archive_minos 'device archive minos expected 17.0'

make_fixture
sed -i '' 's/CMAKE_SYSTEM_NAME:STRING=iOS/CMAKE_SYSTEM_NAME:STRING=Darwin/' \
  "$FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt"
expect_failure wrong_system_name 'device CMAKE_SYSTEM_NAME expected iOS'

make_fixture
sed -i '' 's|CMAKE_C_FLAGS:STRING=.*|CMAKE_C_FLAGS:STRING=-ffast-math|' \
  "$FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt"
expect_failure compiler_flag_drift 'device CMAKE_C_FLAGS expected'

make_fixture
sed -i '' 's/QDLDL_LONG:BOOL=OFF/QDLDL_LONG:BOOL=ON/' \
  "$FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt"
expect_failure qdldl_integer_abi_drift 'device QDLDL_LONG expected OFF'

make_fixture
printf '%s\n' drift >> \
  "$FIXTURE_ROOT/osqp/build_ios/include/public/osqp_configure.h"
expect_failure osqp_generated_header_drift \
  'device generated header SHA-256 does not match lock'

make_fixture
printf '%s\n' drift >> \
  "$FIXTURE_ROOT/nimblephysics/build_ios/dart/config.hpp"
expect_failure nimble_generated_header_drift \
  'Nimble device generated header SHA-256 does not match lock'

make_fixture
printf '%s\n' dirty > \
  "$FIXTURE_ROOT/osqp/build_ios/_deps/qdldl-src/untracked.txt"
expect_failure dirty_qdldl_checkout 'QDLDL device checkout is dirty'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/tools/dependencies.lock.json" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
value = json.loads(path.read_text(encoding="utf-8"))
value["dependencies"]["osqp"]["qdldlCommit"] = "0" * 40
path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
PY
expect_failure wrong_qdldl_commit 'QDLDL device HEAD does not match lock'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/project.yml" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
old = '"$(PROJECT_DIR)/osqp/build_ios/out/libosqpstatic.a"'
if old not in text:
    raise SystemExit("fixture project lacks the OSQP device archive path")
text = text.replace(old, '"$(PROJECT_DIR)/wrong/libosqpstatic.a"', 1)
text += '\n# "$(PROJECT_DIR)/osqp/build_ios/out/libosqpstatic.a"\n'
path.write_text(text, encoding="utf-8")
PY
expect_failure misleading_project_token \
  'project.yml BioMotion device linker inputs changed'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
old = '"$(PROJECT_DIR)/osqp/build_ios/out/libosqpstatic.a"'
if old not in text:
    raise SystemExit("fixture generated project lacks OSQP device path")
path.write_text(
    text.replace(old, '"$(PROJECT_DIR)/wrong/libosqpstatic.a"', 1),
    encoding="utf-8",
)
PY
expect_failure stale_generated_project \
  'generated project BioMotion Debug device linker inputs changed'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
old = '"$(PROJECT_DIR)/osqp/build_ios/include/public"'
if old not in text:
    raise SystemExit("fixture generated project lacks OSQP device header path")
path.write_text(
    text.replace(old, '"$(PROJECT_DIR)/osqp/build_wrong/include/public"', 1),
    encoding="utf-8",
)
PY
expect_failure generated_header_order_drift \
  'generated project BioMotion Debug device header search paths changed'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
old = '"BIOMOTION_TEST_DIAGNOSTICS=1"'
if text.count(old) != 1:
    raise SystemExit("fixture generated project diagnostics definition is not unique")
path.write_text(
    text.replace(old, '"BIOMOTION_TEST_DIAGNOSTICS=0"'), encoding="utf-8"
)
PY
expect_failure generated_preprocessor_drift \
  'generated project BioMotion Debug preprocessor definitions changed'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
diagnostic = text.index('"BIOMOTION_TEST_DIAGNOSTICS=1"')
settings = text.rfind("\t\t\tbuildSettings = {\n", 0, diagnostic)
if settings < 0:
    raise SystemExit("fixture generated project app Debug settings are missing")
insertion = settings + len("\t\t\tbuildSettings = {\n")
text = (
    text[:insertion]
    + '\t\t\t\tUSER_HEADER_SEARCH_PATHS = "/tmp/unreviewed";\n'
    + text[insertion:]
)
path.write_text(text, encoding="utf-8")
PY
expect_failure generated_competing_header_setting \
  "generated project BioMotion Debug build setting surface changed: unexpected=['USER_HEADER_SEARCH_PATHS']"

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
objects = {
    match.group("id"): match.group(0)
    for match in re.finditer(
        r'^\t\t(?P<id>[0-9A-F]{24}) /\* [^\n]* \*/ = \{\n.*?^\t\t\};$',
        text,
        flags=re.MULTILINE | re.DOTALL,
    )
}
target = next(
    block
    for block in objects.values()
    if "\t\t\tisa = PBXNativeTarget;" in block
    and "\t\t\tname = BioMotion;" in block
)
list_match = re.search(r"buildConfigurationList = ([0-9A-F]{24}) ", target)
if list_match is None:
    raise SystemExit("fixture app configuration list is missing")
configuration_list = objects[list_match.group(1)]
configuration_ids = re.findall(
    r"^\t\t\t\t([0-9A-F]{24}) /\* (?:Debug|Release) \*/,$",
    configuration_list,
    flags=re.MULTILINE,
)
if len(configuration_ids) != 2:
    raise SystemExit("fixture app configuration graph is unexpected")

decoys = []
for index, identifier in enumerate(configuration_ids, 1):
    original = objects[identifier]
    decoy_id = "F" * 23 + str(index)
    decoys.append(original.replace(identifier, decoy_id, 1))
    changed = original.replace(
        "PRODUCT_BUNDLE_IDENTIFIER = com.soleil.BioMotion;",
        'PRODUCT_BUNDLE_IDENTIFIER = "$(BIOMOTION_BUNDLE_ID)";',
    ).replace(
        '"$(PROJECT_DIR)/nimblephysics/build_ios/libnimble_ios.a"',
        '"$(PROJECT_DIR)/wrong/libnimble_ios.a"',
    )
    if changed == original:
        raise SystemExit("fixture app configuration was not changed")
    text = text.replace(original, changed, 1)

marker = "/* End XCBuildConfiguration section */"
if text.count(marker) != 1:
    raise SystemExit("fixture XCBuildConfiguration section is unexpected")
text = text.replace(marker, "\n".join(decoys) + "\n" + marker)
path.write_text(text, encoding="utf-8")
PY
expect_failure unattached_pbx_decoy \
  'generated project contains unattached or repeated build configurations'

make_fixture
printf '%s\n' 'unreviewed same-name dynamic library' \
  > "$FIXTURE_ROOT/nimblephysics/build_ios/libnimble_ios.dylib"
printf '%s\n' 'unreviewed same-name dynamic library' \
  > "$FIXTURE_ROOT/osqp/build_ios/out/libosqpstatic.dylib"
expect_pass same_name_dylib_cannot_redirect_explicit_archive

make_fixture
git -C "$FIXTURE_ROOT/osqp" remote set-url origin \
  git@github.com:osqp/osqp.git
expect_pass ssh_origin_normalization

make_fixture
git -C "$FIXTURE_ROOT/osqp" remote remove origin
expect_failure missing_origin 'OSQP repository does not match lock'

make_fixture
sed -i '' 's/CMAKE_BUILD_TYPE:STRING=Release/CMAKE_BUILD_TYPE:STRING=Debug/' \
  "$FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt"
expect_failure wrong_configuration 'device CMAKE_BUILD_TYPE expected Release'

make_fixture
sed -i '' \
  's/CMAKE_OSX_ARCHITECTURES:STRING=arm64/CMAKE_OSX_ARCHITECTURES:STRING=x86_64/' \
  "$FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt"
expect_failure wrong_cache_architecture \
  'device CMAKE_OSX_ARCHITECTURES expected arm64'

make_fixture
sed -i '' \
  's/CMAKE_OSX_DEPLOYMENT_TARGET:STRING=17.0/CMAKE_OSX_DEPLOYMENT_TARGET:STRING=18.0/' \
  "$FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt"
expect_failure wrong_deployment_target \
  'device CMAKE_OSX_DEPLOYMENT_TARGET expected 17.0'

make_fixture
sed -i '' 's/CMAKE_GENERATOR:INTERNAL=Ninja/CMAKE_GENERATOR:INTERNAL=Xcode/' \
  "$FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt"
expect_failure wrong_generator 'device CMAKE_GENERATOR expected Ninja'

make_fixture
sed -i '' \
  "s|CMAKE_HOME_DIRECTORY:INTERNAL=.*|CMAKE_HOME_DIRECTORY:INTERNAL=$FIXTURE_ROOT|" \
  "$FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt"
expect_failure wrong_source_home 'device CMAKE_HOME_DIRECTORY expected'

make_fixture
sed -i '' 's/OSQP_USE_FLOAT:BOOL=OFF/OSQP_USE_FLOAT:BOOL=ON/' \
  "$FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt"
expect_failure float_abi_drift 'device OSQP_USE_FLOAT expected OFF'

make_fixture
sed -i '' 's/OSQP_USE_LONG:BOOL=OFF/OSQP_USE_LONG:BOOL=ON/' \
  "$FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt"
expect_failure integer_abi_drift 'device OSQP_USE_LONG expected OFF'

make_fixture
: > "$FIXTURE_ROOT/osqp/build_ios/out/libosqpstatic.a"
expect_failure empty_archive 'device archive is empty'

make_fixture
mv "$FIXTURE_ROOT/osqp" "$FIXTURE_ROOT/osqp-real"
ln -s osqp-real "$FIXTURE_ROOT/osqp"
expect_failure symlink_source 'OSQP source path must not traverse symlinks'

make_fixture
rm -r "$FIXTURE_ROOT/osqp/.git"
expect_failure missing_git_checkout 'OSQP source is not a Git checkout'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/tools/dependencies.lock.json" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
value = json.loads(path.read_text(encoding="utf-8"))
value["dependencies"]["nimblephysics"]["commit"] = "0" * 40
path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
PY
expect_failure wrong_nimble_commit 'Nimble HEAD does not match lock'

make_fixture
printf '%s\n' dirty > "$FIXTURE_ROOT/nimblephysics/untracked.txt"
expect_failure dirty_nimble_checkout 'Nimble checkout is dirty'

make_fixture
git -C "$FIXTURE_ROOT/nimblephysics" remote set-url origin \
  https://github.com/keenon/nimblephysics.git
expect_failure wrong_nimble_repository 'Nimble repository does not match lock'

make_fixture
printf '%s\n' drift >> \
  "$FIXTURE_ROOT/nimblephysics/build_ios/libnimble_ios.a"
expect_failure wrong_nimble_archive_hash 'Nimble device archive SHA-256 does not match lock'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
build_file = "F00000000000000000000001"
file_reference = "F00000000000000000000002"
framework_phase = "F00000000000000000000003"
for identifier in (build_file, file_reference, framework_phase):
    if identifier in text:
        raise SystemExit(f"fixture unexpectedly contains {identifier}")

text = text.replace(
    "/* End PBXBuildFile section */",
    f'''\t\t{build_file} /* libInjected.dylib in Frameworks */ = {{isa = PBXBuildFile; fileRef = {file_reference} /* libInjected.dylib */; }};\n'''
    "/* End PBXBuildFile section */",
    1,
)
text = text.replace(
    "/* End PBXFileReference section */",
    f'''\t\t{file_reference} /* libInjected.dylib */ = {{isa = PBXFileReference; explicitFileType = "compiled.mach-o.dylib"; path = libInjected.dylib; sourceTree = "<group>"; }};\n'''
    "/* End PBXFileReference section */",
    1,
)
phase_section = f'''/* Begin PBXFrameworksBuildPhase section */
\t\t{framework_phase} /* Frameworks */ = {{
\t\t\tisa = PBXFrameworksBuildPhase;
\t\t\tbuildActionMask = 2147483647;
\t\t\tfiles = (
\t\t\t\t{build_file} /* libInjected.dylib in Frameworks */,
\t\t\t);
\t\t\trunOnlyForDeploymentPostprocessing = 0;
\t\t}};
/* End PBXFrameworksBuildPhase section */

'''
group_marker = "/* Begin PBXGroup section */"
if text.count(group_marker) != 1:
    raise SystemExit("fixture PBXGroup marker is not unique")
text = text.replace(group_marker, phase_section + group_marker, 1)

target_marker = '''\t\tC13EF626D0B416A66FE2CD86 /* BioMotion */ = {
\t\t\tisa = PBXNativeTarget;
\t\t\tbuildConfigurationList = BBD8AA9B86F67B4ED7141590 /* Build configuration list for PBXNativeTarget "BioMotion" */;
\t\t\tbuildPhases = (
'''
if text.count(target_marker) != 1:
    raise SystemExit("fixture BioMotion target marker is not unique")
text = text.replace(
    target_marker,
    target_marker + f"\t\t\t\t{framework_phase} /* Frameworks */,\n",
    1,
)
path.write_text(text, encoding="utf-8")
PY
expect_failure attached_framework_dylib \
  'generated project contains forbidden target/linkage objects: PBXFrameworksBuildPhase'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
old = (
    "\t\t2054C2A9E28C0BC0833FCFC9 /* NimbleBridge.mm in Sources */ = "
    "{isa = PBXBuildFile; fileRef = CF5D35BA98B286FEF7D0F3A2 "
    "/* NimbleBridge.mm */; };"
)
new = old[:-3] + ' settings = {COMPILER_FLAGS = "-I/tmp/unreviewed"; }; };'
if text.count(old) != 1:
    raise SystemExit("fixture NimbleBridge build file is not unique")
path.write_text(text.replace(old, new, 1), encoding="utf-8")
PY
expect_failure source_compiler_flags \
  'generated project target BioMotion sources build file has unexpected keys (unexpected=settings)'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
objects = {
    match.group("id"): match.group(0)
    for match in re.finditer(
        r'^\t\t(?P<id>[0-9A-F]{24}) /\* [^\n]* \*/ = \{\n.*?^\t\t\};$',
        text,
        flags=re.MULTILINE | re.DOTALL,
    )
}
project = next(
    block for block in objects.values() if "\t\t\tisa = PBXProject;" in block
)
list_match = re.search(r"buildConfigurationList = ([0-9A-F]{24}) ", project)
if list_match is None:
    raise SystemExit("fixture PBXProject configuration list is missing")
configuration_list = objects[list_match.group(1)]
debug_match = re.search(r"([0-9A-F]{24}) /\* Debug \*/,", configuration_list)
if debug_match is None:
    raise SystemExit("fixture PBXProject Debug configuration is missing")
debug = objects[debug_match.group(1)]
marker = "\t\t\tbuildSettings = {\n"
if debug.count(marker) != 1:
    raise SystemExit("fixture PBXProject Debug build settings are unexpected")
changed = debug.replace(marker, marker + "\t\t\t\tOBJROOT = /tmp/unreviewed;\n", 1)
path.write_text(text.replace(debug, changed, 1), encoding="utf-8")
PY
expect_failure project_objroot_setting \
  "generated PBXProject Debug build setting surface changed: unexpected=['OBJROOT'], missing=[]"

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
objects = {
    match.group("id"): match.group(0)
    for match in re.finditer(
        r'^\t\t(?P<id>[0-9A-F]{24}) /\* [^\n]* \*/ = \{\n.*?^\t\t\};$',
        text,
        flags=re.MULTILINE | re.DOTALL,
    )
}
target = next(
    block
    for block in objects.values()
    if "\t\t\tisa = PBXNativeTarget;" in block
    and "\t\t\tname = BioMotion;" in block
)
list_match = re.search(r"buildConfigurationList = ([0-9A-F]{24}) ", target)
if list_match is None:
    raise SystemExit("fixture BioMotion configuration list is missing")
configuration_list = objects[list_match.group(1)]
debug_match = re.search(r"([0-9A-F]{24}) /\* Debug \*/,", configuration_list)
if debug_match is None:
    raise SystemExit("fixture BioMotion Debug configuration is missing")
debug = objects[debug_match.group(1)]
marker = "\t\t\tbuildSettings = {\n"
if debug.count(marker) != 1:
    raise SystemExit("fixture BioMotion Debug build settings are unexpected")
changed = debug.replace(
    marker,
    marker
    + '\t\t\t\tOTHER_SWIFT_FLAGS = "-Xfrontend -load-plugin-executable /tmp/unreviewed";\n',
    1,
)
path.write_text(text.replace(debug, changed, 1), encoding="utf-8")
PY
expect_failure target_other_swift_flags \
  "generated project BioMotion Debug build setting surface changed: unexpected=['OTHER_SWIFT_FLAGS'], missing=[]"

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
objects = {
    match.group("id"): match.group(0)
    for match in re.finditer(
        r'^\t\t(?P<id>[0-9A-F]{24}) /\* [^\n]* \*/ = \{\n.*?^\t\t\};$',
        text,
        flags=re.MULTILINE | re.DOTALL,
    )
}
project = next(
    block for block in objects.values() if "\t\t\tisa = PBXProject;" in block
)
list_match = re.search(r"buildConfigurationList = ([0-9A-F]{24}) ", project)
if list_match is None:
    raise SystemExit("fixture PBXProject configuration list is missing")
configuration_list = objects[list_match.group(1)]
debug_match = re.search(r"([0-9A-F]{24}) /\* Debug \*/,", configuration_list)
if debug_match is None:
    raise SystemExit("fixture PBXProject Debug configuration is missing")
debug = objects[debug_match.group(1)]
marker = "\t\t\tisa = XCBuildConfiguration;\n"
if debug.count(marker) != 1:
    raise SystemExit("fixture PBXProject Debug configuration is unexpected")
changed = debug.replace(
    marker,
    marker
    + "\t\t\tbaseConfigurationReference = F00000000000000000000004 /* Unreviewed.xcconfig */;\n",
    1,
)
file_reference = (
    '\t\tF00000000000000000000004 /* Unreviewed.xcconfig */ = '
    '{isa = PBXFileReference; lastKnownFileType = text.xcconfig; '
    'path = Unreviewed.xcconfig; sourceTree = "<group>"; };\n'
)
text = text.replace(debug, changed, 1)
text = text.replace(
    "/* End PBXFileReference section */",
    file_reference + "/* End PBXFileReference section */",
    1,
)
path.write_text(text, encoding="utf-8")
PY
expect_failure base_configuration_reference \
  'generated PBXProject configuration has unexpected keys (unexpected=baseConfigurationReference)'

make_fixture
mkdir -p \
  "$FIXTURE_ROOT/BioMotion.xcodeproj/xcshareddata/xcschemes"
printf '%s\n' '<Scheme version="1.7"></Scheme>' \
  > "$FIXTURE_ROOT/BioMotion.xcodeproj/xcshareddata/xcschemes/BioMotion.xcscheme"
expect_failure shared_scheme_sidecar \
  "generated project container contains scheme, user data, or extra files: unexpected=['xcshareddata/xcschemes/BioMotion.xcscheme']"

make_fixture
mkdir -p \
  "$FIXTURE_ROOT/BioMotion.xcodeproj/xcuserdata/attacker.xcuserdatad/xcschemes"
printf '%s\n' '<Scheme version="1.7"></Scheme>' \
  > "$FIXTURE_ROOT/BioMotion.xcodeproj/xcuserdata/attacker.xcuserdatad/xcschemes/Injected.xcscheme"
expect_failure user_scheme_sidecar \
  "generated project container contains scheme, user data, or extra files: unexpected=['xcuserdata/attacker.xcuserdatad/xcschemes/Injected.xcscheme']"

make_fixture
mkdir -p "$FIXTURE_ROOT/nimblephysics/build_ios/dart/math"
printf '%s\n' 'unreviewed ignored shadow header' \
  > "$FIXTURE_ROOT/nimblephysics/build_ios/dart/math/MathTypes.hpp"
expect_failure ignored_nimble_shadow_header \
  "Nimble device generated header surface changed: expected dart/config.hpp, got ['dart/config.hpp', 'dart/math/MathTypes.hpp']"

make_fixture
osqp_original="$(git -C "$FIXTURE_ROOT/osqp" rev-parse HEAD)"
osqp_tree="$(git -C "$FIXTURE_ROOT/osqp" rev-parse 'HEAD^{tree}')"
osqp_replacement="$(
  printf '%s\n' 'unreviewed replacement commit' \
    | git -C "$FIXTURE_ROOT/osqp" commit-tree "$osqp_tree"
)"
git -C "$FIXTURE_ROOT/osqp" replace "$osqp_original" "$osqp_replacement"
expect_failure git_replace_ref 'OSQP checkout contains replace refs'

make_fixture
git -C "$FIXTURE_ROOT/osqp" update-index --assume-unchanged source.txt
expect_failure assume_unchanged_index \
  'OSQP index contains assume-unchanged/skip-worktree state:'

make_fixture
git -C "$FIXTURE_ROOT/osqp" update-index --skip-worktree source.txt
expect_failure skip_worktree_index \
  'OSQP index contains assume-unchanged/skip-worktree state:'

make_fixture
OFFSET_WORKTREE="$TEST_ROOT/osqp-offset-worktree"
mkdir -p "$OFFSET_WORKTREE"
cp "$FIXTURE_ROOT/osqp/.gitignore" "$OFFSET_WORKTREE/.gitignore"
cp "$FIXTURE_ROOT/osqp/source.txt" "$OFFSET_WORKTREE/source.txt"
git -C "$FIXTURE_ROOT/osqp" config core.worktree "$OFFSET_WORKTREE"
expect_failure offset_core_worktree \
  'OSQP Git top-level does not match the expected checkout'

make_fixture
printf '%s\n' '# unreviewed guard drift' \
  >> "$FIXTURE_ROOT/tools/release/reject_dev_model.sh"
expect_failure reviewed_guard_drift \
  'developer-model build guard does not match its reviewed digest'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
identifier = "F00000000000000000000005"
package_object = f'''/* Begin XCRemoteSwiftPackageReference section */
\t\t{identifier} /* XCRemoteSwiftPackageReference "Injected" */ = {{
\t\t\tisa = XCRemoteSwiftPackageReference;
\t\t\trepositoryURL = "https://example.invalid/injected.git";
\t\t\trequirement = {{
\t\t\t\tkind = upToNextMajorVersion;
\t\t\t\tminimumVersion = 1.0.0;
\t\t\t}};
\t\t}};
/* End XCRemoteSwiftPackageReference section */

'''
configuration_marker = "/* Begin XCBuildConfiguration section */"
if text.count(configuration_marker) != 1:
    raise SystemExit("fixture configuration section marker is not unique")
text = text.replace(configuration_marker, package_object + configuration_marker, 1)
project_marker = "\t\t\tproductRefGroup = A5BA15E011D2144C8A6C2302 /* Products */;\n"
if text.count(project_marker) != 1:
    raise SystemExit("fixture PBXProject product group marker is not unique")
text = text.replace(
    project_marker,
    f"\t\t\tpackageReferences = (\n\t\t\t\t{identifier} /* XCRemoteSwiftPackageReference \"Injected\" */,\n\t\t\t);\n"
    + project_marker,
    1,
)
path.write_text(text, encoding="utf-8")
PY
expect_failure remote_swift_package_reference \
  'generated project contains forbidden target/linkage objects: XCRemoteSwiftPackageReference'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/BioMotion.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
identifier = "F00000000000000000000006"
package_object = f'''/* Begin XCSwiftPackageProductDependency section */
\t\t{identifier} /* InjectedProduct */ = {{
\t\t\tisa = XCSwiftPackageProductDependency;
\t\t\tproductName = InjectedProduct;
\t\t}};
/* End XCSwiftPackageProductDependency section */

'''
configuration_marker = "/* Begin XCBuildConfiguration section */"
if text.count(configuration_marker) != 1:
    raise SystemExit("fixture configuration section marker is not unique")
text = text.replace(configuration_marker, package_object + configuration_marker, 1)
objects = [
    match.group(0)
    for match in re.finditer(
        r'^\t\t[0-9A-F]{24} /\* [^\n]* \*/ = \{\n.*?^\t\t\};$',
        text,
        flags=re.MULTILINE | re.DOTALL,
    )
]
target = next(
    block
    for block in objects
    if "\t\t\tisa = PBXNativeTarget;" in block
    and "\t\t\tname = BioMotion;" in block
)
empty = "\t\t\tpackageProductDependencies = (\n\t\t\t);\n"
if target.count(empty) != 1:
    raise SystemExit("fixture BioMotion package dependency list is unexpected")
changed = target.replace(
    empty,
    f"\t\t\tpackageProductDependencies = (\n\t\t\t\t{identifier} /* InjectedProduct */,\n\t\t\t);\n",
    1,
)
path.write_text(text.replace(target, changed, 1), encoding="utf-8")
PY
expect_failure target_swift_package_dependency \
  'generated project contains forbidden target/linkage objects: XCSwiftPackageProductDependency'

make_fixture
/usr/bin/python3 - "$FIXTURE_ROOT/project.yml" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
needle = (
    "    dependencies:\n"
    "      - target: AssetPackDownloader   # embeds the Background Download extension\n"
)
if text.count(needle) != 1:
    raise SystemExit("fixture BioMotion dependency list is not unique")
replacement = needle + "      - package: InjectedPackage\n"
path.write_text(text.replace(needle, replacement, 1), encoding="utf-8")
PY
expect_failure project_non_target_dependency \
  'project.yml BioMotion contains a non-target or configured dependency: - package: InjectedPackage'

make_fixture
NIMBLE_ALTERNATE_ROOT="$FIXTURE_ROOT/nimblephysics/build_ios/alternate"
write_nimble_cache "$NIMBLE_ALTERNATE_ROOT/CMakeCache.txt" iphoneos
mkdir -p "$NIMBLE_ALTERNATE_ROOT/dart"
cp "$FIXTURE_ROOT/nimblephysics/build_ios/libnimble_ios.a" \
  "$NIMBLE_ALTERNATE_ROOT/libnimble_ios.a"
cp "$FIXTURE_ROOT/nimblephysics/build_ios/dart/config.hpp" \
  "$NIMBLE_ALTERNATE_ROOT/dart/config.hpp"
/usr/bin/python3 - "$FIXTURE_ROOT/tools/dependencies.lock.json" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
value = json.loads(path.read_text(encoding="utf-8"))
device = next(
    build
    for build in value["dependencies"]["nimblephysics"]["builds"]
    if build["name"] == "device"
)
device["cache"] = "nimblephysics/build_ios/alternate/CMakeCache.txt"
device["archive"] = "nimblephysics/build_ios/alternate/libnimble_ios.a"
device["generatedHeader"] = (
    "nimblephysics/build_ios/alternate/dart/config.hpp"
)
path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
PY
expect_failure lock_project_artifact_path_mismatch \
  'project.yml BioMotion device header search paths changed'

make_fixture
total_count=$((total_count + 1))
ORIGINAL_FIXTURE_ROOT="$FIXTURE_ROOT"
ORIGINAL_SNAPSHOT="$TEST_ROOT/dependency-snapshot-original-root.json"
MOVED_FIXTURE_ROOT="$TEST_ROOT/moved-repository"
MOVED_SNAPSHOT="$TEST_ROOT/dependency-snapshot-moved-root.json"
/usr/bin/python3 -I "$INSPECTOR" snapshot "$ORIGINAL_FIXTURE_ROOT" \
  > "$ORIGINAL_SNAPSHOT"
mv "$ORIGINAL_FIXTURE_ROOT" "$MOVED_FIXTURE_ROOT"
for cache in \
  "$MOVED_FIXTURE_ROOT/osqp/build_ios/CMakeCache.txt" \
  "$MOVED_FIXTURE_ROOT/osqp/build_sim/CMakeCache.txt" \
  "$MOVED_FIXTURE_ROOT/nimblephysics/build_ios/CMakeCache.txt" \
  "$MOVED_FIXTURE_ROOT/nimblephysics/build_sim/CMakeCache.txt"
do
  sed -i '' "s|$ORIGINAL_FIXTURE_ROOT|$MOVED_FIXTURE_ROOT|g" "$cache"
done
FIXTURE_ROOT="$MOVED_FIXTURE_ROOT"
/usr/bin/python3 -I "$INSPECTOR" snapshot "$FIXTURE_ROOT" > "$MOVED_SNAPSHOT"
if ! cmp -s "$ORIGINAL_SNAPSHOT" "$MOVED_SNAPSHOT"; then
  printf 'dependency snapshot changed after repository relocation\n' >&2
  diff -u "$ORIGINAL_SNAPSHOT" "$MOVED_SNAPSHOT" >&2 || true
  exit 1
fi
pass_count=$((pass_count + 1))

total_count=$((total_count + 1))
PROBE_WRAPPER="$REPO_ROOT/tools/tests/dependency_boundary_probe.sh"
if [ ! -f "$PROBE_WRAPPER" ] || [ -L "$PROBE_WRAPPER" ]; then
  printf 'dependency probe wrapper is missing or a symlink: %s\n' \
    "$PROBE_WRAPPER" >&2
  exit 1
fi
set +e
output="$("$PROBE_WRAPPER" 2>&1)"
status=$?
set -e
if [ "$status" -ne 0 ]; then
  printf 'real-tree dependency probe unexpectedly failed: %s\n' "$output" >&2
  exit 1
fi
case "$output" in
  DEPENDENCY_BOUNDARY_PASS\ nimble=*\ osqp=*) ;;
  *)
    printf 'real-tree dependency probe did not produce its sentinel: %s\n' \
      "$output" >&2
    exit 1
    ;;
esac
wrapped_snapshot="$("$PROBE_WRAPPER" --snapshot)"
direct_snapshot="$(
  /usr/bin/python3 -I "$INSPECTOR" snapshot "$REPO_ROOT"
)"
if [ "$wrapped_snapshot" != "$direct_snapshot" ]; then
  printf 'dependency probe wrapper changed the canonical snapshot\n' >&2
  exit 1
fi
case "$wrapped_snapshot" in
  '{"dependencies":'*'}') ;;
  *)
    printf 'dependency probe wrapper emitted malformed snapshot JSON\n' >&2
    exit 1
    ;;
esac
pass_count=$((pass_count + 1))

total_count=$((total_count + 1))
DEPENDENCY_ATTACK_ROOT="$TEST_ROOT/dependency-entry-attack"
mkdir -p "$DEPENDENCY_ATTACK_ROOT/fake-path"
cat > "$DEPENDENCY_ATTACK_ROOT/bash-env" <<EOF
printf '%s\n' sourced > "$DEPENDENCY_ATTACK_ROOT/bash-env-ran"
exit 0
EOF
cat > "$DEPENDENCY_ATTACK_ROOT/fake-path/python3" <<EOF
#!/bin/sh
printf '%s\n' fake > "$DEPENDENCY_ATTACK_ROOT/fake-python-ran"
exit 0
EOF
chmod 0755 "$DEPENDENCY_ATTACK_ROOT/fake-path/python3"
set +e
output="$(/usr/bin/env \
  PATH="$DEPENDENCY_ATTACK_ROOT/fake-path" \
  BASH_ENV="$DEPENDENCY_ATTACK_ROOT/bash-env" \
  DEVELOPER_DIR="$DEPENDENCY_ATTACK_ROOT/untrusted-developer" \
  PYTHONHOME="$DEPENDENCY_ATTACK_ROOT/untrusted-python-home" \
  PYTHONPATH="$DEPENDENCY_ATTACK_ROOT/untrusted-python-path" \
  SDKROOT="$DEPENDENCY_ATTACK_ROOT/untrusted-sdk" \
  TOOLCHAINS=untrusted-toolchain \
  "$PROBE_WRAPPER" 2>&1)"
status=$?
set -e
if [ "$status" -ne 0 ] || \
  [ -e "$DEPENDENCY_ATTACK_ROOT/bash-env-ran" ] || \
  [ -e "$DEPENDENCY_ATTACK_ROOT/fake-python-ran" ]; then
  printf 'hostile standalone dependency environment was not contained: %s\n' \
    "$output" >&2
  exit 1
fi
case "$output" in
  DEPENDENCY_BOUNDARY_PASS\ nimble=*\ osqp=*) ;;
  *)
    printf 'hostile dependency entry changed the pass sentinel: %s\n' \
      "$output" >&2
    exit 1
    ;;
esac
pass_count=$((pass_count + 1))

total_count=$((total_count + 1))
set +e
output="$(/usr/bin/python3 - "$REPO_ROOT/tools/run_tests.sh" 2>&1 <<'PY'
from pathlib import Path
import sys

text = Path(sys.argv[1]).read_text(encoding="utf-8")
cd = text.index('  cd "$REPO_ROOT" || return 1')
probe = text.index(
    '  run_tests_hermetic_tool \\\n'
    '    "$RUN_TESTS_BASH" -p \\\n'
    '    "$RUN_TESTS_SCRIPT_DIR/tests/dependency_boundary_probe.sh" || return 1'
)
lock = text.index('  run_tests_acquire_lock || return $?')
boot = text.index('  run_tests_resolve_device || return 1')
if not cd < probe < lock < boot:
    raise SystemExit("dependency probe must run after repo cd and before simulator lock/boot")
print("runner dependency preflight ordering is pinned")
PY
)"
status=$?
set -e
if [ "$status" -ne 0 ]; then
  printf 'run_tests dependency integration is missing: %s\n' "$output" >&2
  exit 1
fi
pass_count=$((pass_count + 1))

if [ "$pass_count" -ne "$total_count" ] || [ "$total_count" -ne 79 ]; then
  printf 'dependency-boundary suite count mismatch: %s/%s\n' \
    "$pass_count" "$total_count" >&2
  exit 1
fi

printf 'DEPENDENCY_BOUNDARY_TESTS_PASS %s/%s\n' "$pass_count" "$total_count"
