#!/usr/bin/env python3
"""Fail closed when a reviewed native dependency checkout drifts."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys


SUBPROCESS_ENVIRONMENT = {
    "GIT_ATTR_NOSYSTEM": "1",
    "GIT_CONFIG_GLOBAL": "/dev/null",
    "GIT_CONFIG_NOSYSTEM": "1",
    "GIT_CONFIG_SYSTEM": "/dev/null",
    "GIT_NO_REPLACE_OBJECTS": "1",
    "GIT_OPTIONAL_LOCKS": "0",
    "GIT_PAGER": "cat",
    "GIT_TERMINAL_PROMPT": "0",
    "LANG": "C",
    "LC_ALL": "C",
    "PAGER": "cat",
    "PATH": "/usr/bin:/bin:/usr/sbin:/sbin",
}


COMMON_CMAKE_KEYS = (
    "CMAKE_BUILD_TYPE",
    "CMAKE_GENERATOR",
    "CMAKE_HOME_DIRECTORY",
    "CMAKE_OSX_ARCHITECTURES",
    "CMAKE_OSX_DEPLOYMENT_TARGET",
    "CMAKE_OSX_SYSROOT",
    "CMAKE_SYSTEM_NAME",
)


OSQP_BOOL_KEYS = {
    "OSQP_ASAN": "asan",
    "OSQP_BUILD_DEMO_EXE": "buildDemo",
    "OSQP_BUILD_SHARED_LIB": "buildShared",
    "OSQP_BUILD_STATIC_LIB": "buildStatic",
    "OSQP_BUILD_UNITTESTS": "buildUnitTests",
    "OSQP_CODEGEN": "codegen",
    "OSQP_ENABLE_DERIVATIVES": "enableDerivatives",
    "OSQP_ENABLE_INTERRUPT": "enableInterrupt",
    "OSQP_ENABLE_PRINTING": "enablePrinting",
    "OSQP_ENABLE_PROFILING": "enableProfiling",
    "OSQP_PACK_SETTINGS": "packSettings",
    "OSQP_USE_FLOAT": "useFloat",
    "OSQP_USE_LONG": "useLong",
    "QDLDL_BUILD_SHARED_LIB": "qdldlBuildShared",
    "QDLDL_BUILD_STATIC_LIB": "qdldlBuildStatic",
    "QDLDL_DEV_ANALYSIS": "qdldlDevAnalysis",
    "QDLDL_DEV_ASAN": "qdldlDevASAN",
    "QDLDL_DEV_COVERAGE": "qdldlDevCoverage",
    "QDLDL_FLOAT": "qdldlFloat",
    "QDLDL_LONG": "qdldlLong",
}


OSQP_CMAKE_KEYS = COMMON_CMAKE_KEYS + (
    "CMAKE_C_FLAGS",
    "CMAKE_C_FLAGS_RELEASE",
    "OSQP_ALGEBRA_BACKEND",
    "OSQP_PROFILER_ANNOTATIONS",
) + tuple(OSQP_BOOL_KEYS)


NIMBLE_CMAKE_KEYS = COMMON_CMAKE_KEYS + ("NIMBLE_IOS_HOST_PROBE",)


HEADER_LIKE_SUFFIXES = {
    ".cuh",
    ".def",
    ".gch",
    ".h",
    ".hh",
    ".hpp",
    ".hxx",
    ".inc",
    ".inl",
    ".ipp",
    ".modulemap",
    ".pch",
    ".tcc",
}


REJECT_DEV_MODEL_SHA256 = (
    "a83bd4b5fbafb6442358ce6dd06627c574514a97c2fa7c28bf8750c8a29223d6"
)


EXPECTED_RELEASE_TOOLCHAIN = {
    "iPhoneOSSDKBuild": "23E237",
    "iPhoneOSSDKVersion": "26.4",
    "xcodeBuild": "17E192",
    "xcodeVersion": "26.4",
}


class BoundaryError(RuntimeError):
    pass


def fail(message: str) -> None:
    print(f"DEPENDENCY_BOUNDARY_FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def reject_duplicate_keys(pairs: list[tuple[str, object]]) -> dict:
    value: dict = {}
    for key, item in pairs:
        if key in value:
            raise BoundaryError(f"dependency lock repeats key: {key}")
        value[key] = item
    return value


def require_exact_keys(value: dict, expected: set[str], label: str) -> None:
    actual = set(value)
    if actual != expected:
        unexpected = sorted(actual - expected)
        missing = sorted(expected - actual)
        details: list[str] = []
        if unexpected:
            details.append(f"unexpected={','.join(unexpected)}")
        if missing:
            details.append(f"missing={','.join(missing)}")
        raise BoundaryError(f"{label} has unexpected keys ({'; '.join(details)})")


def load_lock(path: Path) -> tuple[dict, str]:
    if path.is_symlink() or not path.is_file():
        raise BoundaryError(f"dependency lock is missing or not a regular file: {path}")
    try:
        data = path.read_bytes()
        value = json.loads(
            data.decode("utf-8"),
            object_pairs_hook=reject_duplicate_keys,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise BoundaryError(f"cannot parse dependency lock: {error}") from error
    if not isinstance(value, dict):
        raise BoundaryError("dependency lock root must be an object")
    require_exact_keys(value, {"schemaVersion", "dependencies"}, "dependency lock root")
    if type(value.get("schemaVersion")) is not int or value["schemaVersion"] != 1:
        raise BoundaryError("dependency lock schemaVersion must be integer 1")
    dependencies = value.get("dependencies")
    if not isinstance(dependencies, dict) or set(dependencies) != {
        "nimblephysics",
        "osqp",
    }:
        raise BoundaryError(
            "dependency lock must contain exactly nimblephysics and osqp"
        )
    common_build_keys = {
        "name",
        "cache",
        "archive",
        "generatedHeader",
        "generatedHeaderSHA256",
        "configuration",
        "architecture",
        "deploymentTarget",
        "sysroot",
        "generator",
    }
    dependency_build_keys = {
        "nimblephysics": common_build_keys | {"sha256", "hostProbe"},
        "osqp": common_build_keys
        | {
            "archiveMemberCount",
            "archiveContentSHA256",
            "qdldlSourceDirectory",
            "buildDemo",
            "buildShared",
            "buildStatic",
            "buildUnitTests",
            "useFloat",
            "useLong",
            "algebraBackend",
            "asan",
            "codegen",
            "enableDerivatives",
            "enableInterrupt",
            "enablePrinting",
            "enableProfiling",
            "packSettings",
            "profilerAnnotations",
            "qdldlBuildShared",
            "qdldlBuildStatic",
            "qdldlDevAnalysis",
            "qdldlDevASAN",
            "qdldlDevCoverage",
            "qdldlFloat",
            "qdldlLong",
        },
    }
    for dependency_name in ("nimblephysics", "osqp"):
        dependency = dependencies[dependency_name]
        if not isinstance(dependency, dict):
            raise BoundaryError(
                f"dependency lock {dependency_name} entry must be an object"
            )
        dependency_keys = {"repository", "commit", "sourceDirectory", "builds"}
        if dependency_name == "osqp":
            dependency_keys |= {"qdldlRepository", "qdldlCommit"}
        require_exact_keys(
            dependency, dependency_keys, f"dependency lock {dependency_name} entry"
        )
        commit = dependency.get("commit")
        if not isinstance(commit, str) or re.fullmatch(r"[0-9a-f]{40}", commit) is None:
            raise BoundaryError(
                f"dependency lock {dependency_name} commit must be 40 lowercase hex characters"
            )
        builds = dependency.get("builds")
        if not isinstance(builds, list) or len(builds) != 2:
            raise BoundaryError(
                f"dependency lock {dependency_name} builds must contain device and simulator"
            )
        names: list[str] = []
        for index, build in enumerate(builds):
            if not isinstance(build, dict):
                raise BoundaryError(
                    f"dependency lock {dependency_name} build entry {index} must be an object"
                )
            name = build.get("name")
            if not isinstance(name, str):
                raise BoundaryError(
                    f"dependency lock {dependency_name} build entry {index} name "
                    "must be a string"
                )
            names.append(name)
        if set(names) != {"device", "simulator"} or len(set(names)) != 2:
            raise BoundaryError(
                f"dependency lock {dependency_name} builds must be named device and simulator"
            )
        for index, build in enumerate(builds):
            label = build["name"]
            entry_label = (
                f"{label} build entry"
                if dependency_name == "osqp"
                else f"Nimble {label} build entry"
            )
            require_exact_keys(
                build,
                dependency_build_keys[dependency_name],
                entry_label,
            )
            hash_fields = ["generatedHeaderSHA256"]
            if dependency_name == "nimblephysics":
                hash_fields.append("sha256")
            else:
                hash_fields.append("archiveContentSHA256")
            for hash_field in hash_fields:
                digest = build.get(hash_field)
                if not isinstance(digest, str) or re.fullmatch(
                    r"[0-9a-f]{64}", digest
                ) is None:
                    raise BoundaryError(
                        f"{dependency_name} {label} {hash_field} must be 64 "
                        "lowercase hex characters"
                    )
            if dependency_name == "osqp":
                count = build.get("archiveMemberCount")
                if type(count) is not int or count <= 0:
                    raise BoundaryError(
                        f"osqp {label} archiveMemberCount must be a positive integer"
                    )
        if dependency_name == "osqp":
            qdldl_commit = dependency.get("qdldlCommit")
            if not isinstance(qdldl_commit, str) or re.fullmatch(
                r"[0-9a-f]{40}", qdldl_commit
            ) is None:
                raise BoundaryError(
                    "dependency lock osqp qdldlCommit must be 40 lowercase hex characters"
                )
            qdldl_repository = dependency.get("qdldlRepository")
            if not isinstance(qdldl_repository, str) or not qdldl_repository:
                raise BoundaryError(
                    "dependency lock osqp qdldlRepository must be a non-empty string"
                )
    return value, hashlib.sha256(data).hexdigest()


def canonical_json(value: object) -> str:
    return json.dumps(value, ensure_ascii=True, separators=(",", ":"), sort_keys=True)


def canonical_repository(value: str) -> str:
    repository = value.strip()
    if repository.startswith("git@github.com:"):
        repository = "https://github.com/" + repository[len("git@github.com:"):]
    elif repository.startswith("ssh://git@github.com/"):
        repository = "https://github.com/" + repository[len("ssh://git@github.com/"):]
    repository = repository.rstrip("/")
    if repository.endswith(".git"):
        repository = repository[:-4]
    return repository


def parse_cmake_cache(path: Path, label: str) -> dict[str, str]:
    if path.is_symlink() or not path.is_file():
        raise BoundaryError(f"{label} CMake cache is missing or not a regular file: {path}")
    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise BoundaryError(f"cannot read {label} CMake cache: {error}") from error
    for line in lines:
        if not line or line.startswith(("#", "//")) or "=" not in line:
            continue
        typed_key, value = line.split("=", 1)
        key = typed_key.split(":", 1)[0]
        if key in values:
            raise BoundaryError(f"{label} CMake cache repeats {key}")
        values[key] = value
    return values


def require_cache_value(
    values: dict[str, str], label: str, key: str, expected: str
) -> None:
    actual = values.get(key)
    if actual != expected:
        raise BoundaryError(f"{label} {key} expected {expected}, got {actual!r}")


def require_cache_path(
    values: dict[str, str], label: str, key: str, expected: Path
) -> None:
    actual = values.get(key)
    if actual is None:
        raise BoundaryError(f"{label} {key} expected {expected}, got None")
    try:
        actual_path = Path(actual).resolve(strict=True)
    except OSError as error:
        raise BoundaryError(f"{label} {key} is not a real path: {actual!r}") from error
    if actual_path != expected.resolve(strict=True):
        raise BoundaryError(f"{label} {key} expected {expected}, got {actual!r}")


def require_regular_nonempty(path: Path, label: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise BoundaryError(f"{label} archive is missing or not a regular file: {path}")
    try:
        if path.stat().st_size <= 0:
            raise BoundaryError(f"{label} archive is empty: {path}")
    except OSError as error:
        raise BoundaryError(f"cannot inspect {label} archive: {error}") from error


def run_xcrun(label: str, *arguments: str) -> str:
    try:
        result = subprocess.run(
            ["/usr/bin/xcrun", *arguments],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            env=SUBPROCESS_ENVIRONMENT,
        )
    except OSError as error:
        raise BoundaryError(f"cannot execute xcrun for {label}: {error}") from error
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or f"rc={result.returncode}"
        raise BoundaryError(f"{label} inspection failed: {detail}")
    return result.stdout.strip()


def run_xcrun_bytes(label: str, *arguments: str) -> bytes:
    try:
        result = subprocess.run(
            ["/usr/bin/xcrun", *arguments],
            check=False,
            capture_output=True,
            env=SUBPROCESS_ENVIRONMENT,
        )
    except OSError as error:
        raise BoundaryError(f"cannot execute xcrun for {label}: {error}") from error
    if result.returncode != 0:
        detail = result.stderr.decode("utf-8", errors="replace").strip()
        if not detail:
            detail = result.stdout.decode("utf-8", errors="replace").strip()
        raise BoundaryError(f"{label} inspection failed: {detail or result.returncode}")
    return result.stdout


def inspect_release_toolchain() -> dict[str, str]:
    try:
        result = subprocess.run(
            ["/usr/bin/xcodebuild", "-version"],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            env=SUBPROCESS_ENVIRONMENT,
        )
    except OSError as error:
        raise BoundaryError(f"cannot inspect release Xcode: {error}") from error
    if result.returncode != 0 or result.stderr:
        detail = result.stderr.strip() or result.stdout.strip() or f"rc={result.returncode}"
        raise BoundaryError(f"release Xcode inspection failed: {detail}")
    match = re.fullmatch(
        r"Xcode ([^\n]+)\nBuild version ([^\n]+)\n?", result.stdout
    )
    if match is None:
        raise BoundaryError("release Xcode version output is malformed")
    observed = {
        "iPhoneOSSDKBuild": run_xcrun(
            "release iPhoneOS SDK build", "--sdk", "iphoneos", "--show-sdk-build-version"
        ),
        "iPhoneOSSDKVersion": run_xcrun(
            "release iPhoneOS SDK version", "--sdk", "iphoneos", "--show-sdk-version"
        ),
        "xcodeBuild": match.group(2),
        "xcodeVersion": match.group(1),
    }
    if observed != EXPECTED_RELEASE_TOOLCHAIN:
        raise BoundaryError(
            f"release Xcode/SDK changed: expected {EXPECTED_RELEASE_TOOLCHAIN}, "
            f"got {observed}"
        )
    return observed


def archive_members(path: Path, label: str) -> list[str]:
    names = run_xcrun(f"{label} archive inventory", "ar", "-t", str(path)).splitlines()
    members = [name for name in names if not name.startswith("__.SYMDEF")]
    if not members or len(members) != len(set(members)):
        raise BoundaryError(f"{label} archive member inventory is empty or duplicated")
    for name in members:
        if not name or "/" in name or name in {".", ".."}:
            raise BoundaryError(f"{label} archive has an unsafe member name: {name!r}")
    return members


def archive_content_sha256(path: Path, label: str, members: list[str]) -> str:
    digest = hashlib.sha256(b"BioMotion static archive members v1\0")
    for name in members:
        data = run_xcrun_bytes(
            f"{label} archive member {name}", "ar", "-p", str(path), name
        )
        encoded = name.encode("utf-8")
        digest.update(len(encoded).to_bytes(8, "big"))
        digest.update(encoded)
        digest.update(len(data).to_bytes(8, "big"))
        digest.update(data)
    return digest.hexdigest()


def inspect_archive(
    path: Path,
    label: str,
    expected_platform: str,
    expected_minos: str,
    required_symbols: set[str] | None = None,
    expected_member_count: int | None = None,
    expected_content_sha256: str | None = None,
) -> dict[str, object]:
    require_regular_nonempty(path, label)
    members = archive_members(path, label)
    architectures = run_xcrun(
        f"{label} archive architecture", "lipo", "-archs", str(path)
    ).split()
    if architectures != ["arm64"]:
        actual = " ".join(architectures) if architectures else "none"
        raise BoundaryError(
            f"{label} archive architecture expected arm64, got {actual}"
        )
    load_commands = run_xcrun(
        f"{label} archive platform", "otool", "-l", str(path)
    )
    header = re.compile(rf"^{re.escape(str(path))}\(([^)]+)\):$", re.MULTILINE)
    matches = list(header.finditer(load_commands))
    if not matches:
        raise BoundaryError(f"{label} archive has no inspectable Mach-O members")
    inspected_members = [match.group(1) for match in matches]
    if inspected_members != members:
        raise BoundaryError(
            f"{label} archive member inventory does not match Mach-O inspection"
        )
    for index, match in enumerate(matches):
        member = match.group(1)
        end = matches[index + 1].start() if index + 1 < len(matches) else len(load_commands)
        body = load_commands[match.end():end]
        versions = re.findall(
            r"^\s+cmd LC_BUILD_VERSION\n"
            r"^\s+cmdsize \d+\n"
            r"^\s+platform (\d+)\n"
            r"^\s+minos ([0-9.]+)$",
            body,
            flags=re.MULTILINE,
        )
        if len(versions) != 1:
            raise BoundaryError(
                f"{label} archive member {member} must have exactly one "
                "LC_BUILD_VERSION"
            )
        platform, minos = versions[0]
        if platform != expected_platform:
            raise BoundaryError(
                f"{label} archive platform expected {expected_platform}, got "
                f"{member}={platform}"
            )
        if minos != expected_minos:
            raise BoundaryError(
                f"{label} archive minos expected {expected_minos}, got "
                f"{member}={minos}"
            )
    if required_symbols:
        symbols = set(
            run_xcrun(
                f"{label} archive symbols", "nm", "-gj", str(path)
            ).splitlines()
        )
        missing = sorted(required_symbols - symbols)
        if missing:
            raise BoundaryError(
                f"{label} archive is missing required symbols: {', '.join(missing)}"
            )
    if expected_member_count is not None and len(members) != expected_member_count:
        raise BoundaryError(
            f"{label} archive member count expected {expected_member_count}, "
            f"got {len(members)}"
        )
    actual_content_sha256: str | None = None
    if expected_content_sha256 is not None:
        actual_content_sha256 = archive_content_sha256(path, label, members)
        if actual_content_sha256 != expected_content_sha256:
            raise BoundaryError(
                f"{label} archive content SHA-256 does not match lock: expected "
                f"{expected_content_sha256}, got {actual_content_sha256}"
            )
    return {
        "architecture": architectures[0],
        "contentSHA256": actual_content_sha256,
        "memberCount": len(members),
        "minimumOS": expected_minos,
        "platform": expected_platform,
    }


def sha256_file(path: Path, label: str) -> str:
    try:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()
    except OSError as error:
        raise BoundaryError(f"cannot hash {label}: {error}") from error


def inspect_generated_header(
    root: Path, path_value: object, expected_sha256: object, label: str
) -> str:
    path = locked_path(root, path_value, f"{label} generated header")
    if path.is_symlink() or not path.is_file():
        raise BoundaryError(
            f"{label} generated header is missing or not a regular file: {path}"
        )
    actual_sha256 = sha256_file(path, f"{label} generated header")
    if actual_sha256 != expected_sha256:
        raise BoundaryError(
            f"{label} generated header SHA-256 does not match lock: expected "
            f"{expected_sha256}, got {actual_sha256}"
        )
    return actual_sha256


def generated_header_surface(
    root: Path,
    build: dict,
    label: str,
    dependency: str,
) -> list[str]:
    """Reject ignored files that could shadow a pinned dependency header."""
    cache = locked_path(root, build.get("cache"), f"{label} cache")
    header = locked_path(
        root, build.get("generatedHeader"), f"{label} generated header"
    )
    build_root = cache.parent
    if dependency == "osqp":
        surface_root = build_root / "include"
        try:
            expected = header.relative_to(surface_root).as_posix()
        except ValueError as error:
            raise BoundaryError(
                f"{label} generated header must be inside its build include tree"
            ) from error
    elif dependency == "nimblephysics":
        surface_root = build_root
        try:
            expected = header.relative_to(surface_root).as_posix()
        except ValueError as error:
            raise BoundaryError(
                f"{label} generated header must be inside its build tree"
            ) from error
    else:
        raise BoundaryError(f"unsupported generated header surface: {dependency}")

    if surface_root.is_symlink() or not surface_root.is_dir():
        raise BoundaryError(
            f"{label} generated header surface is not a real directory: "
            f"{surface_root}"
        )

    observed: list[str] = []
    for directory, directory_names, file_names in os.walk(
        surface_root, topdown=True, followlinks=False
    ):
        directory_path = Path(directory)
        for name in list(directory_names):
            entry = directory_path / name
            if entry.is_symlink():
                raise BoundaryError(
                    f"{label} generated header surface contains a symlink: "
                    f"{entry.relative_to(surface_root).as_posix()}"
                )
        for name in file_names:
            entry = directory_path / name
            relative = entry.relative_to(surface_root).as_posix()
            try:
                metadata = entry.lstat()
            except OSError as error:
                raise BoundaryError(
                    f"cannot inspect {label} generated header surface: {error}"
                ) from error
            if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
                raise BoundaryError(
                    f"{label} generated header surface contains a non-regular "
                    f"entry: {relative}"
                )
            suffix = entry.suffix.lower()
            if suffix in HEADER_LIKE_SUFFIXES or (
                not suffix and name not in {".ninja_deps", ".ninja_log"}
            ):
                observed.append(relative)

    observed.sort()
    if observed != [expected]:
        raise BoundaryError(
            f"{label} generated header surface changed: expected {expected}, "
            f"got {observed}"
        )
    return observed


def locked_path(repo_root: Path, value: object, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise BoundaryError(
            f"{label} path must be repository-relative without traversal"
        )
    relative = Path(value)
    if relative.is_absolute() or any(part in {"", ".", ".."} for part in relative.parts):
        raise BoundaryError(
            f"{label} path must be repository-relative without traversal"
        )
    candidate = repo_root.joinpath(relative)
    current = repo_root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise BoundaryError(f"{label} path must not traverse symlinks: {value}")
    try:
        candidate.relative_to(repo_root)
    except ValueError as error:
        raise BoundaryError(
            f"{label} path must be repository-relative without traversal"
        ) from error
    return candidate


def yaml_target_lines(lines: list[str], target: str) -> list[str]:
    marker = f"  {target}:"
    indices = [index for index, line in enumerate(lines) if line == marker]
    if len(indices) != 1:
        raise BoundaryError(f"project.yml must define target {target} exactly once")
    start = indices[0]
    end = len(lines)
    for index in range(start + 1, len(lines)):
        if re.fullmatch(r"  [^ ].*:", lines[index]):
            end = index
            break
    return lines[start:end]


def yaml_list_setting(
    lines: list[str], key: str, label: str, indent: int = 8
) -> list[str]:
    marker = f"{' ' * indent}{key}:"
    indices = [index for index, line in enumerate(lines) if line == marker]
    if len(indices) != 1:
        raise BoundaryError(f"{label} must be defined exactly once")
    values: list[str] = []
    for line in lines[indices[0] + 1:]:
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        item_prefix = f"{' ' * (indent + 2)}- "
        if not line.startswith(item_prefix):
            break
        encoded = line[len(item_prefix):]
        try:
            value = json.loads(encoded)
        except json.JSONDecodeError as error:
            raise BoundaryError(f"{label} contains a non-JSON YAML scalar") from error
        if not isinstance(value, str):
            raise BoundaryError(f"{label} contains a non-string value")
        values.append(value)
    return values


def yaml_scalar_setting(
    lines: list[str], key: str, label: str, indent: int = 10
) -> str:
    prefix = f"{' ' * indent}{key}: "
    values = [line[len(prefix):] for line in lines if line.startswith(prefix)]
    if len(values) != 1:
        raise BoundaryError(f"{label} must be defined exactly once")
    try:
        value = json.loads(values[0])
    except json.JSONDecodeError as error:
        raise BoundaryError(f"{label} contains a non-JSON YAML scalar") from error
    if not isinstance(value, str):
        raise BoundaryError(f"{label} must be a string")
    return value


def yaml_config_lines(lines: list[str], config: str, label: str) -> list[str]:
    marker = f"        {config}:"
    indices = [index for index, line in enumerate(lines) if line == marker]
    if len(indices) != 1:
        raise BoundaryError(f"{label} must define {config} exactly once")
    start = indices[0]
    end = len(lines)
    for index in range(start + 1, len(lines)):
        line = lines[index]
        if line.strip() and not line.lstrip().startswith("#"):
            indentation = len(line) - len(line.lstrip(" "))
            if indentation <= 8:
                end = index
                break
    return lines[start:end]


def decode_setting_name(encoded: str, label: str) -> str:
    if encoded.startswith('"'):
        try:
            value = json.loads(encoded)
        except json.JSONDecodeError as error:
            raise BoundaryError(f"{label} contains a malformed quoted setting") from error
        if not isinstance(value, str):
            raise BoundaryError(f"{label} contains a non-string setting name")
        return value
    return encoded


def yaml_setting_names(lines: list[str], label: str) -> list[str]:
    names: list[str] = []
    pattern = re.compile(
        r'^\s+(?P<name>"(?:[^"\\]|\\.)+"|[A-Z][A-Z0-9_]*)\s*:'
    )
    for line in lines:
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        match = pattern.match(line)
        if match is not None:
            names.append(decode_setting_name(match.group("name"), label))
    return names


def yaml_target_dependencies(lines: list[str], label: str) -> list[str]:
    markers = [
        index
        for index, line in enumerate(lines)
        if line.rstrip() == "    dependencies:"
    ]
    if not markers:
        return []
    if len(markers) != 1:
        raise BoundaryError(f"{label} must define dependencies at most once")
    dependencies: list[str] = []
    for line in lines[markers[0] + 1:]:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        indentation = len(line) - len(line.lstrip(" "))
        if indentation <= 4:
            break
        match = re.fullmatch(
            r" {6}- target: ([A-Za-z0-9_.-]+)(?:\s+#.*)?", line.rstrip()
        )
        if match is None:
            raise BoundaryError(
                f"{label} contains a non-target or configured dependency: "
                f"{stripped}"
            )
        dependencies.append(match.group(1))
    return dependencies


def pbx_list_setting(body: str, key: str, label: str) -> list[str]:
    marker = f"{key} = ("
    lines = body.splitlines()
    indices = [index for index, line in enumerate(lines) if line.strip() == marker]
    if len(indices) != 1:
        raise BoundaryError(f"{label} must be defined exactly once")
    values: list[str] = []
    for line in lines[indices[0] + 1:]:
        item = line.strip()
        if item == ");":
            return values
        if not item.endswith(","):
            raise BoundaryError(f"{label} contains a malformed PBX value")
        try:
            value = json.loads(item[:-1])
        except json.JSONDecodeError as error:
            raise BoundaryError(f"{label} contains a non-JSON PBX scalar") from error
        if not isinstance(value, str):
            raise BoundaryError(f"{label} contains a non-string value")
        values.append(value)
    raise BoundaryError(f"{label} list is unterminated")


def pbx_objects(text: str) -> dict[str, str]:
    pattern = re.compile(
        r"^\t\t(?P<identifier>[0-9A-F]{24}) /\* [^\n]* \*/ = \{\n"
        r"(?P<body>.*?)"
        r"^\t\t\};$",
        flags=re.MULTILINE | re.DOTALL,
    )
    objects: dict[str, str] = {}
    for match in pattern.finditer(text):
        identifier = match.group("identifier")
        if identifier in objects:
            raise BoundaryError(f"generated project repeats object {identifier}")
        objects[identifier] = match.group("body")
    if not objects:
        raise BoundaryError("generated project contains no parseable PBX objects")
    return objects


def parse_generated_project(path: Path, text: str) -> tuple[dict[str, dict], str]:
    identifiers = re.findall(
        r"^\t\t([0-9A-F]{24})(?: /\* [^\n]* \*/)? = \{",
        text,
        flags=re.MULTILINE,
    )
    if not identifiers or len(identifiers) != len(set(identifiers)):
        raise BoundaryError(
            "generated project object definitions are empty or duplicated"
        )
    try:
        result = subprocess.run(
            ["/usr/bin/plutil", "-convert", "json", "-o", "-", str(path)],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            env=SUBPROCESS_ENVIRONMENT,
        )
    except OSError as error:
        raise BoundaryError(f"cannot parse generated project: {error}") from error
    if result.returncode != 0 or result.stderr:
        detail = result.stderr.strip() or result.stdout.strip() or f"rc={result.returncode}"
        raise BoundaryError(f"cannot parse generated project plist: {detail}")

    def reject_pbx_duplicates(pairs: list[tuple[str, object]]) -> dict:
        value: dict = {}
        for key, item in pairs:
            if key in value:
                raise BoundaryError(f"generated project plist repeats key: {key}")
            value[key] = item
        return value

    try:
        project = json.loads(
            result.stdout,
            object_pairs_hook=reject_pbx_duplicates,
        )
    except (json.JSONDecodeError, BoundaryError) as error:
        if isinstance(error, BoundaryError):
            raise
        raise BoundaryError(f"generated project plist is not JSON: {error}") from error
    if not isinstance(project, dict):
        raise BoundaryError("generated project plist root must be an object")
    objects = project.get("objects")
    root_object = project.get("rootObject")
    if not isinstance(objects, dict) or not isinstance(root_object, str):
        raise BoundaryError("generated project plist has no object graph")
    if set(objects) != set(identifiers):
        raise BoundaryError(
            "generated project plist object graph is not fully represented"
        )
    for identifier, value in objects.items():
        if re.fullmatch(r"[0-9A-F]{24}", identifier) is None or not isinstance(
            value, dict
        ):
            raise BoundaryError(
                f"generated project contains a malformed object: {identifier}"
            )
    if root_object not in objects:
        raise BoundaryError("generated project rootObject is unresolved")
    return objects, root_object


def pbx_scalar_setting(body: str, key: str, label: str) -> str:
    pattern = re.compile(
        rf"^\t\t\t{re.escape(key)} = (?P<value>[^;\n]+);$", re.MULTILINE
    )
    values = [match.group("value") for match in pattern.finditer(body)]
    if len(values) != 1:
        raise BoundaryError(f"{label} must define {key} exactly once")
    return values[0]


def pbx_string_setting(body: str, key: str, label: str) -> str:
    encoded = pbx_scalar_setting(body, key, label)
    return decode_setting_name(encoded, label)


def pbx_build_string_setting(body: str, key: str, label: str) -> str:
    pattern = re.compile(
        rf"^\t\t\t\t{re.escape(key)} = (?P<value>[^;\n]+);$",
        re.MULTILINE,
    )
    values = [match.group("value") for match in pattern.finditer(body)]
    if len(values) != 1:
        raise BoundaryError(f"{label} must define {key} exactly once")
    return decode_setting_name(values[0], label)


def pbx_identifier_setting(body: str, key: str, label: str) -> str:
    encoded = pbx_scalar_setting(body, key, label)
    match = re.fullmatch(r"(?P<identifier>[0-9A-F]{24})(?: /\* [^\n]* \*/)?", encoded)
    if match is None:
        raise BoundaryError(f"{label} {key} does not name one PBX object")
    return match.group("identifier")


def pbx_identifier_list(body: str, key: str, label: str) -> list[str]:
    lines = body.splitlines()
    marker = f"{key} = ("
    indices = [index for index, line in enumerate(lines) if line.strip() == marker]
    if len(indices) != 1:
        raise BoundaryError(f"{label} must define {key} exactly once")
    identifiers: list[str] = []
    for line in lines[indices[0] + 1:]:
        item = line.strip()
        if item == ");":
            return identifiers
        match = re.fullmatch(
            r"(?P<identifier>[0-9A-F]{24})(?: /\* [^\n]* \*/)?,", item
        )
        if match is None:
            raise BoundaryError(f"{label} {key} contains a malformed PBX object")
        identifiers.append(match.group("identifier"))
    raise BoundaryError(f"{label} {key} list is unterminated")


def pbx_build_settings(body: str, label: str) -> str:
    pattern = re.compile(
        r"^\t\t\tbuildSettings = \{\n(?P<settings>.*?)^\t\t\t\};$",
        flags=re.MULTILINE | re.DOTALL,
    )
    matches = list(pattern.finditer(body))
    if len(matches) != 1:
        raise BoundaryError(f"{label} must contain exactly one buildSettings block")
    return matches[0].group("settings")


def pbx_setting_names(body: str, label: str) -> list[str]:
    names: list[str] = []
    pattern = re.compile(
        r'^\t\t\t\t(?P<name>"(?:[^"\\]|\\.)+"|[A-Z][A-Z0-9_]*) = ',
        re.MULTILINE,
    )
    for match in pattern.finditer(body):
        names.append(decode_setting_name(match.group("name"), label))
    return names


UNSAFE_DEPENDENCY_SETTING_BASES = {
    "AR",
    "AS",
    "CC",
    "CLANG",
    "CLANGXX",
    "CLANG_MODULEMAP_FILE",
    "COMPILER_FLAGS",
    "CXX",
    "EXCLUDED_SOURCE_FILE_NAMES",
    "EXTERNAL_BUILD_TOOL",
    "FRAMEWORK_SEARCH_PATHS",
    "GCC_PREFIX_HEADER",
    "GCC_PRECOMPILE_PREFIX_HEADER",
    "GCC_VERSION",
    "INCLUDED_SOURCE_FILE_NAMES",
    "LD",
    "LDPLUSPLUS",
    "LD_INPUT_FILE_LIST",
    "LIBRARY_SEARCH_PATHS",
    "LIBTOOL",
    "LINK_FILE_LIST",
    "LIPO",
    "MODULEMAP_FILE",
    "NM",
    "OBJECT_FILE_DIR",
    "OBJECT_FILE_DIR_normal",
    "RANLIB",
    "STRIP",
    "SWIFT_DRIVER_EXEC",
    "SWIFT_EXEC",
    "SWIFT_INCLUDE_PATHS",
    "SYSTEM_HEADER_SEARCH_PATHS",
    "TOOLCHAINS",
    "USER_HEADER_SEARCH_PATHS",
    "WARNING_CFLAGS",
    "WARNING_LDFLAGS",
    "XCODE_XCCONFIG_FILE",
}


def reject_unsafe_dependency_settings(
    names: list[str], label: str, allowed_bases: set[str]
) -> None:
    injected: set[str] = set()
    for name in names:
        base = name.split("[", 1)[0]
        if base in allowed_bases:
            continue
        if (
            base in UNSAFE_DEPENDENCY_SETTING_BASES
            or base.startswith("OTHER_")
            or base.endswith("_SEARCH_PATHS")
        ):
            injected.add(base)
    if injected:
        raise BoundaryError(
            f"{label} contains competing dependency settings: "
            f"{', '.join(sorted(injected))}"
        )


def require_setting_name_surface(
    settings: dict, expected: set[str], label: str
) -> None:
    actual = set(settings)
    if actual != expected:
        raise BoundaryError(
            f"{label} build setting surface changed: "
            f"unexpected={sorted(actual - expected)}, "
            f"missing={sorted(expected - actual)}"
        )


def require_dependency_setting_surface(
    names: list[str], label: str, definition_count: int
) -> None:
    reject_unsafe_dependency_settings(
        names,
        label,
        {
            "GCC_PREPROCESSOR_DEFINITIONS",
            "HEADER_SEARCH_PATHS",
            "LD_RUNPATH_SEARCH_PATHS",
            "OTHER_LDFLAGS",
        },
    )
    by_base: dict[str, list[str]] = {}
    for name in names:
        base = name.split("[", 1)[0]
        by_base.setdefault(base, []).append(name)
    expected_conditional = {
        "HEADER_SEARCH_PATHS": {
            "HEADER_SEARCH_PATHS[sdk=iphoneos*]",
            "HEADER_SEARCH_PATHS[sdk=iphonesimulator*]",
        },
        "OTHER_LDFLAGS": {
            "OTHER_LDFLAGS[sdk=iphoneos*]",
            "OTHER_LDFLAGS[sdk=iphonesimulator*]",
        },
    }
    for base, expected in expected_conditional.items():
        actual = by_base.get(base, [])
        if len(actual) != len(expected) or set(actual) != expected:
            raise BoundaryError(f"{label} {base} setting surface changed: {actual}")
    definitions = by_base.get("GCC_PREPROCESSOR_DEFINITIONS", [])
    if definitions != ["GCC_PREPROCESSOR_DEFINITIONS"] * definition_count:
        raise BoundaryError(
            f"{label} GCC_PREPROCESSOR_DEFINITIONS setting surface changed"
        )


def require_link_settings(
    settings: dict[str, list[str]], expected: dict[str, list[str]], prefix: str
) -> None:
    for label, expected_values in expected.items():
        actual = settings[label]
        if actual != expected_values:
            raise BoundaryError(f"{prefix} {label} changed: {actual}")


def pbx_json_object(
    objects: dict[str, dict], identifier: object, expected_isa: str, label: str
) -> dict:
    if not isinstance(identifier, str) or re.fullmatch(
        r"[0-9A-F]{24}", identifier
    ) is None:
        raise BoundaryError(f"{label} does not name one PBX object")
    value = objects.get(identifier)
    if not isinstance(value, dict) or value.get("isa") != expected_isa:
        raise BoundaryError(f"{label} is not a {expected_isa}")
    return value


def pbx_json_configurations(
    objects: dict[str, dict], owner: dict, label: str
) -> dict[str, dict]:
    configuration_list = pbx_json_object(
        objects,
        owner.get("buildConfigurationList"),
        "XCConfigurationList",
        f"{label} buildConfigurationList",
    )
    require_exact_keys(
        configuration_list,
        {
            "isa",
            "buildConfigurations",
            "defaultConfigurationIsVisible",
            "defaultConfigurationName",
        },
        f"{label} buildConfigurationList",
    )
    if (
        configuration_list.get("defaultConfigurationIsVisible") != "0"
        or configuration_list.get("defaultConfigurationName") != "Debug"
    ):
        raise BoundaryError(f"{label} default configuration surface changed")
    identifiers = configuration_list.get("buildConfigurations")
    if (
        not isinstance(identifiers, list)
        or len(identifiers) != 2
        or len(set(identifiers)) != 2
    ):
        raise BoundaryError(f"{label} must reference two build configurations")
    configurations: dict[str, dict] = {}
    for identifier in identifiers:
        configuration = pbx_json_object(
            objects,
            identifier,
            "XCBuildConfiguration",
            f"{label} configuration",
        )
        require_exact_keys(
            configuration,
            {"isa", "buildSettings", "name"},
            f"{label} configuration",
        )
        name = configuration.get("name")
        settings = configuration.get("buildSettings")
        if name in configurations or name not in {"Debug", "Release"}:
            raise BoundaryError(f"{label} configuration names changed")
        if not isinstance(settings, dict) or not all(
            isinstance(key, str) for key in settings
        ):
            raise BoundaryError(f"{label} {name} buildSettings is malformed")
        configurations[name] = configuration
    if set(configurations) != {"Debug", "Release"}:
        raise BoundaryError(f"{label} must reference Debug and Release")
    return configurations


def inspect_project_container(project_directory: Path) -> dict[str, str]:
    workspace = project_directory / "project.xcworkspace" / "contents.xcworkspacedata"
    expected_workspace = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<Workspace\n'
        '   version = "1.0">\n'
        '   <FileRef\n'
        '      location = "self:">\n'
        '   </FileRef>\n'
        '</Workspace>\n'
    ).encode("utf-8")
    allowed_files = {
        "project.pbxproj",
        "project.xcworkspace/contents.xcworkspacedata",
    }
    observed_files: set[str] = set()
    for directory, directory_names, file_names in os.walk(
        project_directory, topdown=True, followlinks=False
    ):
        directory_path = Path(directory)
        for name in directory_names:
            entry = directory_path / name
            if entry.is_symlink():
                raise BoundaryError(
                    "generated project container contains a directory symlink: "
                    f"{entry.relative_to(project_directory).as_posix()}"
                )
        for name in file_names:
            entry = directory_path / name
            relative = entry.relative_to(project_directory).as_posix()
            try:
                metadata = entry.lstat()
            except OSError as error:
                raise BoundaryError(
                    f"cannot inspect generated project container: {error}"
                ) from error
            if not stat.S_ISREG(metadata.st_mode):
                raise BoundaryError(
                    "generated project container contains a non-regular file: "
                    f"{relative}"
                )
            observed_files.add(relative)
    if observed_files != allowed_files:
        raise BoundaryError(
            "generated project container contains scheme, user data, or extra files: "
            f"unexpected={sorted(observed_files - allowed_files)}, "
            f"missing={sorted(allowed_files - observed_files)}"
        )
    try:
        workspace_bytes = workspace.read_bytes()
    except OSError as error:
        raise BoundaryError(f"cannot read generated project workspace: {error}") from error
    if workspace_bytes != expected_workspace:
        raise BoundaryError("generated project workspace surface changed")
    return {
        "schemePolicy": "implicit-target-only",
        "workspaceSHA256": hashlib.sha256(workspace_bytes).hexdigest(),
    }


def is_linkable_file_reference(value: dict) -> bool:
    path = value.get("path", value.get("name", ""))
    lowered = path.lower() if isinstance(path, str) else ""
    if lowered.endswith((".a", ".o", ".dylib", ".tbd", ".framework", ".xcframework")):
        return True
    file_types = {
        value.get("explicitFileType"),
        value.get("lastKnownFileType"),
    }
    return bool(
        file_types
        & {
            "archive.ar",
            "compiled.mach-o.dylib",
            "compiled.mach-o.objfile",
            "sourcecode.text-based-dylib-definition",
            "wrapper.framework",
            "wrapper.xcframework",
        }
    )


def normalized_phase_build_file(
    objects: dict[str, dict], identifier: str, phase_kind: str, label: str
) -> dict:
    build_file = pbx_json_object(
        objects, identifier, "PBXBuildFile", f"{label} build file"
    )
    if phase_kind in {"sources", "resources"}:
        require_exact_keys(
            build_file,
            {"isa", "fileRef"},
            f"{label} build file",
        )
    reference = pbx_json_object(
        objects,
        build_file.get("fileRef"),
        "PBXFileReference",
        f"{label} file reference",
    )
    if is_linkable_file_reference(reference):
        raise BoundaryError(f"{label} contains a target-attached link input")
    normalized = {
        "fileType": reference.get(
            "explicitFileType", reference.get("lastKnownFileType", "")
        ),
        "path": reference.get("path", reference.get("name", "")),
    }
    if not all(isinstance(value, str) for value in normalized.values()):
        raise BoundaryError(f"{label} file reference is malformed")
    return normalized


def inspect_generated_project_graph(
    repo_root: Path,
    generated: Path,
    generated_text: str,
    expected_app: dict[str, list[str]],
    expected_tests: dict[str, list[str]],
    app_debug_definitions: list[str],
    app_release_definitions: list[str],
    test_definitions: list[str],
) -> dict:
    internal_ui_condition = "$(inherited) BIOMOTION_INTERNAL_UI"
    objects, root_identifier = parse_generated_project(generated, generated_text)
    raw_objects = pbx_objects(generated_text)
    strict_multiline_isas = {
        "PBXCopyFilesBuildPhase",
        "PBXNativeTarget",
        "PBXResourcesBuildPhase",
        "PBXShellScriptBuildPhase",
        "PBXSourcesBuildPhase",
        "XCBuildConfiguration",
        "XCConfigurationList",
    }
    for identifier, value in objects.items():
        if value.get("isa") not in strict_multiline_isas:
            continue
        raw_body = raw_objects.get(identifier)
        if raw_body is None:
            raise BoundaryError(
                f"generated project critical object is not canonical: {identifier}"
            )
        raw_keys = re.findall(
            r"^\t\t\t([A-Za-z][A-Za-z0-9_]*) = ",
            raw_body,
            flags=re.MULTILINE,
        )
        if len(raw_keys) != len(set(raw_keys)) or set(raw_keys) != set(value):
            raise BoundaryError(
                f"generated project critical object keys changed: {identifier}"
            )
        if value.get("isa") == "XCBuildConfiguration":
            raw_settings = pbx_build_settings(
                raw_body, f"generated project configuration {identifier}"
            )
            raw_setting_names = pbx_setting_names(
                raw_settings, f"generated project configuration {identifier}"
            )
            settings = value.get("buildSettings")
            if (
                not isinstance(settings, dict)
                or len(raw_setting_names) != len(set(raw_setting_names))
                or set(raw_setting_names) != set(settings)
            ):
                raise BoundaryError(
                    f"generated project configuration setting keys changed: {identifier}"
                )
    container = inspect_project_container(generated.parent)
    forbidden_isas = {
        "PBXAggregateTarget",
        "PBXFrameworksBuildPhase",
        "PBXLegacyTarget",
        "XCLocalSwiftPackageReference",
        "XCRemoteSwiftPackageReference",
        "XCSwiftPackageProductDependency",
    }
    present_forbidden = sorted(
        {
            value.get("isa")
            for value in objects.values()
            if value.get("isa") in forbidden_isas
        }
    )
    if present_forbidden:
        raise BoundaryError(
            "generated project contains forbidden target/linkage objects: "
            f"{', '.join(present_forbidden)}"
        )
    for value in objects.values():
        if value.get("isa") == "PBXFileReference" and is_linkable_file_reference(value):
            raise BoundaryError(
                "generated project contains a linkable file reference outside "
                "the pinned OTHER_LDFLAGS"
            )
        if value.get("isa") == "PBXBuildFile" and "productRef" in value:
            raise BoundaryError("generated project contains a package product build file")

    project = pbx_json_object(
        objects, root_identifier, "PBXProject", "generated project root"
    )
    package_references = project.get("packageReferences", [])
    if package_references != []:
        raise BoundaryError("generated project packageReferences must be empty")

    target_pairs = [
        (identifier, value)
        for identifier, value in objects.items()
        if value.get("isa") == "PBXNativeTarget"
    ]
    target_by_name: dict[str, tuple[str, dict]] = {}
    for identifier, value in target_pairs:
        name = value.get("name")
        if not isinstance(name, str) or name in target_by_name:
            raise BoundaryError("generated project native target names are malformed")
        target_by_name[name] = (identifier, value)
    expected_target_names = {"AssetPackDownloader", "BioMotion", "BioMotionTests"}
    if set(target_by_name) != expected_target_names:
        raise BoundaryError(
            "generated project native target surface changed: "
            f"{sorted(target_by_name)}"
        )
    project_targets = project.get("targets")
    if (
        not isinstance(project_targets, list)
        or len(project_targets) != 3
        or set(project_targets) != {pair[0] for pair in target_pairs}
    ):
        raise BoundaryError("generated project targets graph changed")

    configuration_owners = [project] + [
        target_by_name[name][1] for name in sorted(expected_target_names)
    ]
    configuration_list_identifiers = [
        owner.get("buildConfigurationList") for owner in configuration_owners
    ]
    configuration_list_objects = {
        identifier
        for identifier, value in objects.items()
        if value.get("isa") == "XCConfigurationList"
    }
    if (
        len(configuration_list_identifiers)
        != len(set(configuration_list_identifiers))
        or set(configuration_list_identifiers) != configuration_list_objects
    ):
        raise BoundaryError(
            "generated project contains unattached or repeated configuration lists"
        )
    configuration_identifiers: list[str] = []
    for identifier in configuration_list_identifiers:
        configuration_list = pbx_json_object(
            objects,
            identifier,
            "XCConfigurationList",
            "generated project configuration list",
        )
        identifiers = configuration_list.get("buildConfigurations")
        if not isinstance(identifiers, list):
            raise BoundaryError("generated project configuration list is malformed")
        configuration_identifiers.extend(identifiers)
    configuration_objects = {
        identifier
        for identifier, value in objects.items()
        if value.get("isa") == "XCBuildConfiguration"
    }
    if (
        len(configuration_identifiers) != len(set(configuration_identifiers))
        or set(configuration_identifiers) != configuration_objects
    ):
        raise BoundaryError(
            "generated project contains unattached or repeated build configurations"
        )

    project_configurations = pbx_json_configurations(
        objects, project, "generated PBXProject"
    )
    project_common_setting_names = {
        "ALWAYS_SEARCH_USER_PATHS",
        "ARCHS[sdk=iphonesimulator*]",
        "CLANG_ANALYZER_NONNULL",
        "CLANG_ANALYZER_NUMBER_OBJECT_CONVERSION",
        "CLANG_CXX_LANGUAGE_STANDARD",
        "CLANG_CXX_LIBRARY",
        "CLANG_ENABLE_MODULES",
        "CLANG_ENABLE_OBJC_ARC",
        "CLANG_ENABLE_OBJC_WEAK",
        "CLANG_WARN_BLOCK_CAPTURE_AUTORELEASING",
        "CLANG_WARN_BOOL_CONVERSION",
        "CLANG_WARN_COMMA",
        "CLANG_WARN_CONSTANT_CONVERSION",
        "CLANG_WARN_DEPRECATED_OBJC_IMPLEMENTATIONS",
        "CLANG_WARN_DIRECT_OBJC_ISA_USAGE",
        "CLANG_WARN_DOCUMENTATION_COMMENTS",
        "CLANG_WARN_EMPTY_BODY",
        "CLANG_WARN_ENUM_CONVERSION",
        "CLANG_WARN_INFINITE_RECURSION",
        "CLANG_WARN_INT_CONVERSION",
        "CLANG_WARN_NON_LITERAL_NULL_CONVERSION",
        "CLANG_WARN_OBJC_IMPLICIT_RETAIN_SELF",
        "CLANG_WARN_OBJC_LITERAL_CONVERSION",
        "CLANG_WARN_OBJC_ROOT_CLASS",
        "CLANG_WARN_QUOTED_INCLUDE_IN_FRAMEWORK_HEADER",
        "CLANG_WARN_RANGE_LOOP_ANALYSIS",
        "CLANG_WARN_STRICT_PROTOTYPES",
        "CLANG_WARN_SUSPICIOUS_MOVE",
        "CLANG_WARN_UNGUARDED_AVAILABILITY",
        "CLANG_WARN_UNREACHABLE_CODE",
        "CLANG_WARN__DUPLICATE_METHOD_MATCH",
        "COPY_PHASE_STRIP",
        "DEBUG_INFORMATION_FORMAT",
        "ENABLE_STRICT_OBJC_MSGSEND",
        "GCC_C_LANGUAGE_STANDARD",
        "GCC_NO_COMMON_BLOCKS",
        "GCC_WARN_64_TO_32_BIT_CONVERSION",
        "GCC_WARN_ABOUT_RETURN_TYPE",
        "GCC_WARN_UNDECLARED_SELECTOR",
        "GCC_WARN_UNINITIALIZED_AUTOS",
        "GCC_WARN_UNUSED_FUNCTION",
        "GCC_WARN_UNUSED_VARIABLE",
        "IPHONEOS_DEPLOYMENT_TARGET",
        "MTL_ENABLE_DEBUG_INFO",
        "MTL_FAST_MATH",
        "PRODUCT_NAME",
        "SDKROOT",
        "SWIFT_OPTIMIZATION_LEVEL",
        "SWIFT_VERSION",
    }
    project_setting_names = {
        "Debug": project_common_setting_names
        | {
            "ENABLE_TESTABILITY",
            "GCC_DYNAMIC_NO_PIC",
            "GCC_OPTIMIZATION_LEVEL",
            "GCC_PREPROCESSOR_DEFINITIONS",
            "ONLY_ACTIVE_ARCH",
            "SWIFT_ACTIVE_COMPILATION_CONDITIONS",
        },
        "Release": project_common_setting_names
        | {"ENABLE_NS_ASSERTIONS", "SWIFT_COMPILATION_MODE"},
    }
    normalized_project_settings: dict[str, dict] = {}
    for configuration in ("Debug", "Release"):
        settings = project_configurations[configuration]["buildSettings"]
        require_setting_name_surface(
            settings,
            project_setting_names[configuration],
            f"generated PBXProject {configuration}",
        )
        reject_unsafe_dependency_settings(
            list(settings), f"generated PBXProject {configuration}", set()
        )
        required = {
            "ALWAYS_SEARCH_USER_PATHS": "NO",
            "ARCHS[sdk=iphonesimulator*]": "arm64",
            "CLANG_CXX_LANGUAGE_STANDARD": "gnu++14",
            "CLANG_CXX_LIBRARY": "libc++",
            "SDKROOT": "iphoneos",
        }
        for key, expected in required.items():
            if settings.get(key) != expected:
                raise BoundaryError(
                    f"generated PBXProject {configuration} {key} changed"
                )
        expected_project_definitions = (
            ["$(inherited)", "DEBUG=1"] if configuration == "Debug" else None
        )
        if settings.get("GCC_PREPROCESSOR_DEFINITIONS") != expected_project_definitions:
            if not (
                configuration == "Release"
                and "GCC_PREPROCESSOR_DEFINITIONS" not in settings
            ):
                raise BoundaryError(
                    f"generated PBXProject {configuration} preprocessor definitions changed"
                )
        normalized_project_settings[configuration] = {
            **required,
            "GCC_PREPROCESSOR_DEFINITIONS": expected_project_definitions,
        }

    app_common_setting_names = {
        "ASSETCATALOG_COMPILER_APPICON_NAME",
        "CLANG_CXX_LANGUAGE_STANDARD",
        "CODE_SIGN_ENTITLEMENTS",
        "CODE_SIGN_IDENTITY",
        "CODE_SIGN_STYLE",
        "CURRENT_PROJECT_VERSION",
        "DEVELOPMENT_TEAM",
        "GCC_PREPROCESSOR_DEFINITIONS",
        "HEADER_SEARCH_PATHS[sdk=iphoneos*]",
        "HEADER_SEARCH_PATHS[sdk=iphonesimulator*]",
        "INFOPLIST_FILE",
        "LD_RUNPATH_SEARCH_PATHS",
        "MARKETING_VERSION",
        "OTHER_LDFLAGS[sdk=iphoneos*]",
        "OTHER_LDFLAGS[sdk=iphonesimulator*]",
        "PRODUCT_BUNDLE_IDENTIFIER",
        "SDKROOT",
        "SWIFT_OBJC_BRIDGING_HEADER",
        "SWIFT_VERSION",
        "TARGETED_DEVICE_FAMILY",
    }
    test_setting_names = {
        "BUNDLE_LOADER",
        "CLANG_CXX_LANGUAGE_STANDARD",
        "GCC_PREPROCESSOR_DEFINITIONS",
        "GENERATE_INFOPLIST_FILE",
        "HEADER_SEARCH_PATHS[sdk=iphoneos*]",
        "HEADER_SEARCH_PATHS[sdk=iphonesimulator*]",
        "LD_RUNPATH_SEARCH_PATHS",
        "OTHER_LDFLAGS[sdk=iphoneos*]",
        "OTHER_LDFLAGS[sdk=iphonesimulator*]",
        "PRODUCT_BUNDLE_IDENTIFIER",
        "SDKROOT",
        "SWIFT_OBJC_BRIDGING_HEADER",
        "SWIFT_VERSION",
        "TARGETED_DEVICE_FAMILY",
        "TEST_HOST",
    }
    target_setting_names = {
        "BioMotion": {
            "Debug": app_common_setting_names
            | {"SWIFT_ACTIVE_COMPILATION_CONDITIONS"},
            "Release": app_common_setting_names
            | {
                "CODE_SIGN_IDENTITY[sdk=iphoneos*]",
                "PROVISIONING_PROFILE_SPECIFIER[sdk=iphoneos*]",
            },
        },
        "BioMotionTests": {
            "Debug": test_setting_names,
            "Release": test_setting_names,
        },
    }
    expected_target_configuration = {
        "BioMotion": (
            expected_app,
            {"Debug": app_debug_definitions, "Release": app_release_definitions},
            {
                "CLANG_CXX_LANGUAGE_STANDARD": "c++17",
                "LD_RUNPATH_SEARCH_PATHS": [
                    "$(inherited)",
                    "@executable_path/Frameworks",
                ],
                "SDKROOT": "iphoneos",
                "SWIFT_OBJC_BRIDGING_HEADER": (
                    "BioMotion/Nimble/BioMotion-Bridging-Header.h"
                ),
            },
        ),
        "BioMotionTests": (
            expected_tests,
            {"Debug": test_definitions, "Release": test_definitions},
            {
                "BUNDLE_LOADER": "$(TEST_HOST)",
                "CLANG_CXX_LANGUAGE_STANDARD": "c++17",
                "LD_RUNPATH_SEARCH_PATHS": [
                    "$(inherited)",
                    "@executable_path/Frameworks",
                    "@loader_path/Frameworks",
                ],
                "SDKROOT": "iphoneos",
                "SWIFT_OBJC_BRIDGING_HEADER": (
                    "BioMotionTests/BioMotionTests-Bridging-Header.h"
                ),
                "TEST_HOST": "$(BUILT_PRODUCTS_DIR)/BioMotion.app/BioMotion",
            },
        ),
    }
    normalized_target_settings: dict[str, dict] = {}
    for target_name, (expected, definitions, pinned) in expected_target_configuration.items():
        target = target_by_name[target_name][1]
        configurations = pbx_json_configurations(
            objects, target, f"generated project target {target_name}"
        )
        normalized_target_settings[target_name] = {}
        for configuration in ("Debug", "Release"):
            settings = configurations[configuration]["buildSettings"]
            require_setting_name_surface(
                settings,
                target_setting_names[target_name][configuration],
                f"generated project {target_name} {configuration}",
            )
            require_dependency_setting_surface(
                list(settings),
                f"generated project {target_name} {configuration}",
                1,
            )
            json_settings = {
                "device header search paths": settings.get(
                    "HEADER_SEARCH_PATHS[sdk=iphoneos*]"
                ),
                "simulator header search paths": settings.get(
                    "HEADER_SEARCH_PATHS[sdk=iphonesimulator*]"
                ),
                "device linker inputs": settings.get(
                    "OTHER_LDFLAGS[sdk=iphoneos*]"
                ),
                "simulator linker inputs": settings.get(
                    "OTHER_LDFLAGS[sdk=iphonesimulator*]"
                ),
            }
            require_link_settings(
                json_settings,
                expected,
                f"generated project {target_name} {configuration}",
            )
            if settings.get("GCC_PREPROCESSOR_DEFINITIONS") != definitions[configuration]:
                raise BoundaryError(
                    f"generated project {target_name} {configuration} "
                    "preprocessor definitions changed"
                )
            expected_internal_ui = (
                internal_ui_condition
                if target_name == "BioMotion" and configuration == "Debug"
                else None
            )
            if settings.get("SWIFT_ACTIVE_COMPILATION_CONDITIONS") != expected_internal_ui:
                if not (
                    expected_internal_ui is None
                    and "SWIFT_ACTIVE_COMPILATION_CONDITIONS" not in settings
                ):
                    raise BoundaryError(
                        f"generated project {target_name} {configuration} "
                        "internal UI condition changed"
                    )
            for key, expected_value in pinned.items():
                if settings.get(key) != expected_value:
                    raise BoundaryError(
                        f"generated project {target_name} {configuration} {key} changed"
                    )
            normalized_target_settings[target_name][configuration] = {
                "definitions": definitions[configuration],
                "swiftConditions": expected_internal_ui,
                "pinned": pinned,
                "settings": json_settings,
            }

    extension_common_setting_names = {
        "CODE_SIGN_ENTITLEMENTS",
        "CODE_SIGN_STYLE",
        "CURRENT_PROJECT_VERSION",
        "DEVELOPMENT_TEAM",
        "INFOPLIST_FILE",
        "LD_RUNPATH_SEARCH_PATHS",
        "MARKETING_VERSION",
        "PRODUCT_BUNDLE_IDENTIFIER",
        "SDKROOT",
        "SWIFT_VERSION",
        "TARGETED_DEVICE_FAMILY",
    }
    extension_setting_names = {
        "Debug": extension_common_setting_names,
        "Release": extension_common_setting_names
        | {
            "CODE_SIGN_IDENTITY[sdk=iphoneos*]",
            "PROVISIONING_PROFILE_SPECIFIER[sdk=iphoneos*]",
        },
    }
    extension_configurations = pbx_json_configurations(
        objects,
        target_by_name["AssetPackDownloader"][1],
        "generated project target AssetPackDownloader",
    )
    normalized_target_settings["AssetPackDownloader"] = {}
    extension_pinned = {
        "CODE_SIGN_ENTITLEMENTS": (
            "BioMotion/AssetPack/Support/AssetPackDownloader.entitlements"
        ),
        "INFOPLIST_FILE": (
            "BioMotion/AssetPack/Support/AssetPackDownloader-Info.plist"
        ),
        "LD_RUNPATH_SEARCH_PATHS": [
            "$(inherited)",
            "@executable_path/Frameworks",
        ],
        "PRODUCT_BUNDLE_IDENTIFIER": "com.soleil.BioMotion.AssetPackDownloader",
        "SDKROOT": "iphoneos",
        "SWIFT_VERSION": "5.9",
    }
    for configuration in ("Debug", "Release"):
        settings = extension_configurations[configuration]["buildSettings"]
        require_setting_name_surface(
            settings,
            extension_setting_names[configuration],
            f"generated project AssetPackDownloader {configuration}",
        )
        reject_unsafe_dependency_settings(
            list(settings),
            f"generated project AssetPackDownloader {configuration}",
            {"LD_RUNPATH_SEARCH_PATHS"},
        )
        for key, expected_value in extension_pinned.items():
            if settings.get(key) != expected_value:
                raise BoundaryError(
                    "generated project AssetPackDownloader "
                    f"{configuration} {key} changed"
                )
        normalized_target_settings["AssetPackDownloader"][configuration] = {
            "pinned": extension_pinned
        }

    target_contracts = {
        "AssetPackDownloader": {
            "dependencies": [],
            "phases": [("PBXSourcesBuildPhase", None)],
            "productType": "com.apple.product-type.extensionkit-extension",
        },
        "BioMotion": {
            "dependencies": ["AssetPackDownloader"],
            "phases": [
                (
                    "PBXShellScriptBuildPhase",
                    "Reject developer model before non-Simulator build",
                ),
                ("PBXSourcesBuildPhase", None),
                ("PBXResourcesBuildPhase", None),
                ("PBXCopyFilesBuildPhase", "Embed ExtensionKit Extensions"),
                (
                    "PBXShellScriptBuildPhase",
                    "Reject bundled model after resources",
                ),
            ],
            "productType": "com.apple.product-type.application",
        },
        "BioMotionTests": {
            "dependencies": ["BioMotion"],
            "phases": [
                ("PBXSourcesBuildPhase", None),
                ("PBXResourcesBuildPhase", None),
            ],
            "productType": "com.apple.product-type.bundle.unit-test",
        },
    }
    guard_path = repo_root / "tools/release/reject_dev_model.sh"
    if guard_path.is_symlink() or not guard_path.is_file():
        raise BoundaryError("developer-model build guard is not a regular file")
    try:
        guard_bytes = guard_path.read_bytes()
        guard_script = guard_bytes.decode("utf-8")
    except (OSError, UnicodeError) as error:
        raise BoundaryError(f"cannot read developer-model build guard: {error}") from error
    guard_digest = hashlib.sha256(guard_bytes).hexdigest()
    if guard_digest != REJECT_DEV_MODEL_SHA256:
        raise BoundaryError(
            "developer-model build guard does not match its reviewed digest"
        )
    referenced_phases: list[str] = []
    referenced_build_files: list[str] = []
    referenced_dependencies: list[str] = []
    referenced_proxies: list[str] = []
    normalized_phases: dict[str, list[dict]] = {}
    normalized_dependencies: dict[str, list[str]] = {}
    for target_name, contract in target_contracts.items():
        target_identifier, target = target_by_name[target_name]
        require_exact_keys(
            target,
            {
                "isa",
                "buildConfigurationList",
                "buildPhases",
                "buildRules",
                "dependencies",
                "name",
                "packageProductDependencies",
                "productName",
                "productReference",
                "productType",
            },
            f"generated project target {target_name}",
        )
        if target.get("buildRules") != [] or target.get("packageProductDependencies") != []:
            raise BoundaryError(
                f"generated project target {target_name} build/package rules changed"
            )
        if target.get("productType") != contract["productType"]:
            raise BoundaryError(
                f"generated project target {target_name} productType changed"
            )
        phase_identifiers = target.get("buildPhases")
        expected_phases = contract["phases"]
        if (
            not isinstance(phase_identifiers, list)
            or len(phase_identifiers) != len(expected_phases)
            or len(set(phase_identifiers)) != len(phase_identifiers)
        ):
            raise BoundaryError(
                f"generated project target {target_name} build phase surface changed"
            )
        normalized_phases[target_name] = []
        for phase_identifier, (expected_isa, expected_name) in zip(
            phase_identifiers, expected_phases
        ):
            phase = pbx_json_object(
                objects,
                phase_identifier,
                expected_isa,
                f"generated project target {target_name} phase",
            )
            referenced_phases.append(phase_identifier)
            if expected_isa in {"PBXSourcesBuildPhase", "PBXResourcesBuildPhase"}:
                require_exact_keys(
                    phase,
                    {
                        "isa",
                        "buildActionMask",
                        "files",
                        "runOnlyForDeploymentPostprocessing",
                    },
                    f"generated project target {target_name} {expected_isa}",
                )
                if (
                    phase.get("buildActionMask") != "2147483647"
                    or phase.get("runOnlyForDeploymentPostprocessing") != "0"
                ):
                    raise BoundaryError(
                        f"generated project target {target_name} {expected_isa} settings changed"
                    )
                phase_kind = (
                    "sources" if expected_isa == "PBXSourcesBuildPhase" else "resources"
                )
                files = phase.get("files")
                if not isinstance(files, list) or len(files) != len(set(files)):
                    raise BoundaryError(
                        f"generated project target {target_name} {phase_kind} files changed"
                    )
                normalized_files = []
                for build_file_identifier in files:
                    referenced_build_files.append(build_file_identifier)
                    normalized_files.append(
                        normalized_phase_build_file(
                            objects,
                            build_file_identifier,
                            phase_kind,
                            f"generated project target {target_name} {phase_kind}",
                        )
                    )
                normalized_phases[target_name].append(
                    {"files": normalized_files, "isa": expected_isa}
                )
            elif expected_isa == "PBXCopyFilesBuildPhase":
                require_exact_keys(
                    phase,
                    {
                        "buildActionMask",
                        "dstPath",
                        "dstSubfolderSpec",
                        "files",
                        "isa",
                        "name",
                        "runOnlyForDeploymentPostprocessing",
                    },
                    f"generated project target {target_name} copy phase",
                )
                if phase != {
                    "buildActionMask": "2147483647",
                    "dstPath": "$(EXTENSIONS_FOLDER_PATH)",
                    "dstSubfolderSpec": "16",
                    "files": phase.get("files"),
                    "isa": "PBXCopyFilesBuildPhase",
                    "name": expected_name,
                    "runOnlyForDeploymentPostprocessing": "0",
                }:
                    raise BoundaryError(
                        f"generated project target {target_name} copy phase changed"
                    )
                files = phase.get("files")
                if not isinstance(files, list) or len(files) != 1:
                    raise BoundaryError(
                        f"generated project target {target_name} copy phase files changed"
                    )
                build_file_identifier = files[0]
                referenced_build_files.append(build_file_identifier)
                build_file = pbx_json_object(
                    objects,
                    build_file_identifier,
                    "PBXBuildFile",
                    f"generated project target {target_name} copy build file",
                )
                require_exact_keys(
                    build_file,
                    {"isa", "fileRef", "settings"},
                    f"generated project target {target_name} copy build file",
                )
                if build_file.get("settings") != {
                    "ATTRIBUTES": ["RemoveHeadersOnCopy"]
                }:
                    raise BoundaryError(
                        f"generated project target {target_name} copy attributes changed"
                    )
                reference = pbx_json_object(
                    objects,
                    build_file.get("fileRef"),
                    "PBXFileReference",
                    f"generated project target {target_name} embedded extension",
                )
                embedded = {
                    "explicitFileType": "wrapper.extensionkit-extension",
                    "includeInIndex": "0",
                    "isa": "PBXFileReference",
                    "path": "AssetPackDownloader.appex",
                    "sourceTree": "BUILT_PRODUCTS_DIR",
                }
                if reference != embedded:
                    raise BoundaryError(
                        f"generated project target {target_name} embedded extension changed"
                    )
                normalized_phases[target_name].append(
                    {"file": embedded, "isa": expected_isa, "name": expected_name}
                )
            else:
                expected_shell = {
                    "alwaysOutOfDate": "1",
                    "buildActionMask": "2147483647",
                    "files": [],
                    "inputFileListPaths": [],
                    "inputPaths": [],
                    "isa": "PBXShellScriptBuildPhase",
                    "name": expected_name,
                    "outputFileListPaths": [],
                    "outputPaths": [],
                    "runOnlyForDeploymentPostprocessing": "0",
                    "shellPath": "/bin/bash",
                    "shellScript": guard_script,
                    "showEnvVarsInLog": "0",
                }
                if phase != expected_shell:
                    raise BoundaryError(
                        f"generated project target {target_name} shell phase changed: "
                        f"{expected_name}"
                    )
                normalized_phases[target_name].append(
                    {
                        "isa": expected_isa,
                        "name": expected_name,
                        "scriptSHA256": guard_digest,
                    }
                )

        dependency_identifiers = target.get("dependencies")
        expected_dependencies = contract["dependencies"]
        if (
            not isinstance(dependency_identifiers, list)
            or len(dependency_identifiers) != len(expected_dependencies)
            or len(set(dependency_identifiers)) != len(dependency_identifiers)
        ):
            raise BoundaryError(
                f"generated project target {target_name} dependency graph changed"
            )
        observed_dependency_names: list[str] = []
        for dependency_identifier, expected_dependency_name in zip(
            dependency_identifiers, expected_dependencies
        ):
            dependency = pbx_json_object(
                objects,
                dependency_identifier,
                "PBXTargetDependency",
                f"generated project target {target_name} dependency",
            )
            require_exact_keys(
                dependency,
                {"isa", "target", "targetProxy"},
                f"generated project target {target_name} dependency",
            )
            expected_target_identifier = target_by_name[expected_dependency_name][0]
            if dependency.get("target") != expected_target_identifier:
                raise BoundaryError(
                    f"generated project target {target_name} dependency target changed"
                )
            proxy_identifier = dependency.get("targetProxy")
            proxy = pbx_json_object(
                objects,
                proxy_identifier,
                "PBXContainerItemProxy",
                f"generated project target {target_name} dependency proxy",
            )
            expected_proxy = {
                "containerPortal": root_identifier,
                "isa": "PBXContainerItemProxy",
                "proxyType": "1",
                "remoteGlobalIDString": expected_target_identifier,
                "remoteInfo": expected_dependency_name,
            }
            if proxy != expected_proxy:
                raise BoundaryError(
                    f"generated project target {target_name} dependency proxy changed"
                )
            referenced_dependencies.append(dependency_identifier)
            referenced_proxies.append(proxy_identifier)
            observed_dependency_names.append(expected_dependency_name)
        normalized_dependencies[target_name] = observed_dependency_names

    phase_objects = {
        identifier
        for identifier, value in objects.items()
        if isinstance(value.get("isa"), str) and value["isa"].endswith("BuildPhase")
    }
    build_file_objects = {
        identifier
        for identifier, value in objects.items()
        if value.get("isa") == "PBXBuildFile"
    }
    dependency_objects = {
        identifier
        for identifier, value in objects.items()
        if value.get("isa") == "PBXTargetDependency"
    }
    proxy_objects = {
        identifier
        for identifier, value in objects.items()
        if value.get("isa") == "PBXContainerItemProxy"
    }
    for observed, expected, label in (
        (referenced_phases, phase_objects, "build phases"),
        (referenced_build_files, build_file_objects, "build files"),
        (referenced_dependencies, dependency_objects, "target dependencies"),
        (referenced_proxies, proxy_objects, "target dependency proxies"),
    ):
        if len(observed) != len(set(observed)) or set(observed) != expected:
            raise BoundaryError(
                f"generated project contains unattached or repeated {label}"
            )

    return {
        "container": container,
        "dependencies": normalized_dependencies,
        "phases": normalized_phases,
        "projectSettings": normalized_project_settings,
        "targetSettings": normalized_target_settings,
    }


def inspect_project_linkage(repo_root: Path, dependency_lock: dict) -> dict[str, str]:
    project = repo_root / "project.yml"
    generated = repo_root / "BioMotion.xcodeproj" / "project.pbxproj"
    for path, label in ((project, "project.yml"), (generated, "generated project")):
        if path.is_symlink() or not path.is_file():
            raise BoundaryError(f"{label} is missing or not a regular file: {path}")
    try:
        project_lines = project.read_text(encoding="utf-8").splitlines()
        generated_text = generated.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise BoundaryError(f"cannot read project linkage: {error}") from error
    toolchain = inspect_release_toolchain()

    nimble_builds = builds_by_name(
        dependency_lock["dependencies"]["nimblephysics"], "Nimble"
    )
    osqp_builds = builds_by_name(
        dependency_lock["dependencies"]["osqp"], "OSQP"
    )

    def project_path(value: object, label: str) -> str:
        path = locked_path(repo_root, value, label)
        return f"$(PROJECT_DIR)/{path.relative_to(repo_root).as_posix()}"

    def nimble_generated_root(build: dict, label: str) -> str:
        header = locked_path(
            repo_root, build.get("generatedHeader"), f"{label} generated header"
        )
        return f"$(PROJECT_DIR)/{header.parents[1].relative_to(repo_root).as_posix()}"

    def osqp_generated_root(build: dict, label: str) -> str:
        header = locked_path(
            repo_root, build.get("generatedHeader"), f"{label} generated header"
        )
        return f"$(PROJECT_DIR)/{header.parent.relative_to(repo_root).as_posix()}"

    expected_app = {
        "device header search paths": [
            "$(PROJECT_DIR)/nimblephysics",
            "$(PROJECT_DIR)/nimblephysics/third_party/eigen",
            "$(PROJECT_DIR)/nimblephysics/third_party/tinyxml2",
            "$(PROJECT_DIR)/osqp/include/public",
            "$(PROJECT_DIR)/osqp/include/private",
            nimble_generated_root(nimble_builds["device"], "Nimble device"),
            osqp_generated_root(osqp_builds["device"], "OSQP device"),
        ],
        "simulator header search paths": [
            "$(PROJECT_DIR)/nimblephysics",
            "$(PROJECT_DIR)/nimblephysics/third_party/eigen",
            "$(PROJECT_DIR)/nimblephysics/third_party/tinyxml2",
            "$(PROJECT_DIR)/osqp/include/public",
            "$(PROJECT_DIR)/osqp/include/private",
            nimble_generated_root(nimble_builds["simulator"], "Nimble simulator"),
            osqp_generated_root(osqp_builds["simulator"], "OSQP simulator"),
        ],
        "device linker inputs": [
            project_path(
                nimble_builds["device"].get("archive"), "Nimble device archive"
            ),
            project_path(osqp_builds["device"].get("archive"), "OSQP device archive"),
            "-lc++",
        ],
        "simulator linker inputs": [
            project_path(
                nimble_builds["simulator"].get("archive"),
                "Nimble simulator archive",
            ),
            project_path(
                osqp_builds["simulator"].get("archive"), "OSQP simulator archive"
            ),
            "-lc++",
        ],
    }
    expected_tests = {
        "device header search paths": [
            "$(PROJECT_DIR)/BioMotion/Nimble",
            "$(PROJECT_DIR)/BioMotion/Muscle",
            "$(PROJECT_DIR)/nimblephysics",
            "$(PROJECT_DIR)/nimblephysics/third_party/eigen",
            "$(PROJECT_DIR)/nimblephysics/third_party/tinyxml2",
            nimble_generated_root(nimble_builds["device"], "Nimble device"),
        ],
        "simulator header search paths": [
            "$(PROJECT_DIR)/BioMotion/Nimble",
            "$(PROJECT_DIR)/BioMotion/Muscle",
            "$(PROJECT_DIR)/nimblephysics",
            "$(PROJECT_DIR)/nimblephysics/third_party/eigen",
            "$(PROJECT_DIR)/nimblephysics/third_party/tinyxml2",
            nimble_generated_root(nimble_builds["simulator"], "Nimble simulator"),
        ],
        "device linker inputs": [
            project_path(
                nimble_builds["device"].get("archive"), "Nimble device archive"
            ),
            "-lc++",
        ],
        "simulator linker inputs": [
            project_path(
                nimble_builds["simulator"].get("archive"),
                "Nimble simulator archive",
            ),
            "-lc++",
        ],
    }
    app_release_definitions = [
        "DART_IOS_BUILD=1",
        "DART_USE_IDENTITY_JACOBIAN=1",
        "EIGEN_DONT_PARALLELIZE=1",
        "EIGEN_MPL2_ONLY=1",
    ]
    app_debug_definitions = app_release_definitions + [
        "BIOMOTION_TEST_DIAGNOSTICS=1"
    ]
    internal_ui_condition = "$(inherited) BIOMOTION_INTERNAL_UI"
    test_definitions = ["$(inherited)"] + app_release_definitions
    yaml_keys = {
        "device header search paths": '"HEADER_SEARCH_PATHS[sdk=iphoneos*]"',
        "simulator header search paths": (
            '"HEADER_SEARCH_PATHS[sdk=iphonesimulator*]"'
        ),
        "device linker inputs": '"OTHER_LDFLAGS[sdk=iphoneos*]"',
        "simulator linker inputs": '"OTHER_LDFLAGS[sdk=iphonesimulator*]"',
    }
    normalized_yaml: dict[str, dict] = {}
    for target, expected in (("BioMotion", expected_app), ("BioMotionTests", expected_tests)):
        target_lines = yaml_target_lines(project_lines, target)
        expected_dependencies = (
            ["AssetPackDownloader"] if target == "BioMotion" else ["BioMotion"]
        )
        actual_dependencies = yaml_target_dependencies(
            target_lines, f"project.yml {target}"
        )
        if actual_dependencies != expected_dependencies:
            raise BoundaryError(
                f"project.yml {target} dependency graph changed: "
                f"{actual_dependencies}"
            )
        require_dependency_setting_surface(
            yaml_setting_names(target_lines, f"project.yml {target}"),
            f"project.yml {target}",
            2 if target == "BioMotion" else 1,
        )
        settings = {
            label: yaml_list_setting(
                target_lines,
                key,
                f"project.yml {target} {label}",
            )
            for label, key in yaml_keys.items()
        }
        require_link_settings(settings, expected, f"project.yml {target}")
        if target == "BioMotion":
            base_definitions = yaml_list_setting(
                target_lines,
                "GCC_PREPROCESSOR_DEFINITIONS",
                "project.yml BioMotion base preprocessor definitions",
            )
            debug_lines = yaml_config_lines(
                target_lines, "Debug", "project.yml BioMotion"
            )
            debug_definitions = yaml_list_setting(
                debug_lines,
                "GCC_PREPROCESSOR_DEFINITIONS",
                "project.yml BioMotion Debug preprocessor definitions",
                indent=10,
            )
            release_lines = yaml_config_lines(
                target_lines, "Release", "project.yml BioMotion"
            )
            condition_key = "SWIFT_ACTIVE_COMPILATION_CONDITIONS"
            condition_lines = [
                line for line in target_lines
                if line.lstrip().startswith(f"{condition_key}:")
            ]
            if len(condition_lines) != 1:
                raise BoundaryError(
                    "project.yml BioMotion must define the internal UI "
                    "condition exactly once in Debug"
                )
            debug_internal_ui = yaml_scalar_setting(
                debug_lines,
                condition_key,
                "project.yml BioMotion Debug internal UI condition",
            )
            if debug_internal_ui != internal_ui_condition:
                raise BoundaryError(
                    "project.yml BioMotion Debug internal UI condition changed: "
                    f"{debug_internal_ui!r}"
                )
            if any(
                line.lstrip().startswith(f"{condition_key}:")
                for line in release_lines
            ):
                raise BoundaryError(
                    "project.yml BioMotion Release must not enable internal UI"
                )
            if base_definitions != app_release_definitions:
                raise BoundaryError(
                    f"project.yml BioMotion base preprocessor definitions changed: "
                    f"{base_definitions}"
                )
            if debug_definitions != app_debug_definitions:
                raise BoundaryError(
                    f"project.yml BioMotion Debug preprocessor definitions changed: "
                    f"{debug_definitions}"
                )
            definitions = {
                "Debug": debug_definitions,
                "Release": base_definitions,
            }
            swift_conditions = {
                "Debug": debug_internal_ui,
                "Release": None,
            }
        else:
            base_definitions = yaml_list_setting(
                target_lines,
                "GCC_PREPROCESSOR_DEFINITIONS",
                "project.yml BioMotionTests preprocessor definitions",
            )
            if base_definitions != test_definitions:
                raise BoundaryError(
                    f"project.yml BioMotionTests preprocessor definitions changed: "
                    f"{base_definitions}"
                )
            definitions = {
                "Debug": base_definitions,
                "Release": base_definitions,
            }
            swift_conditions = {"Debug": None, "Release": None}
        normalized_yaml[target] = {
            "dependencies": actual_dependencies,
            "definitions": definitions,
            "swiftConditions": swift_conditions,
            "settings": settings,
        }

    extension_lines = yaml_target_lines(project_lines, "AssetPackDownloader")
    extension_dependencies = yaml_target_dependencies(
        extension_lines, "project.yml AssetPackDownloader"
    )
    if extension_dependencies:
        raise BoundaryError(
            "project.yml AssetPackDownloader dependency graph changed: "
            f"{extension_dependencies}"
        )
    normalized_yaml["AssetPackDownloader"] = {
        "dependencies": extension_dependencies
    }

    generated_graph = inspect_generated_project_graph(
        repo_root,
        generated,
        generated_text,
        expected_app,
        expected_tests,
        app_debug_definitions,
        app_release_definitions,
        test_definitions,
    )

    objects = pbx_objects(generated_text)
    normalized_pbx: dict[str, dict[str, dict[str, list[str]]]] = {}
    pbx_keys = {
        "device header search paths": '"HEADER_SEARCH_PATHS[sdk=iphoneos*]"',
        "simulator header search paths": (
            '"HEADER_SEARCH_PATHS[sdk=iphonesimulator*]"'
        ),
        "device linker inputs": '"OTHER_LDFLAGS[sdk=iphoneos*]"',
        "simulator linker inputs": '"OTHER_LDFLAGS[sdk=iphonesimulator*]"',
    }
    for target, expected, expected_definitions in (
        (
            "BioMotion",
            expected_app,
            {"Debug": app_debug_definitions, "Release": app_release_definitions},
        ),
        (
            "BioMotionTests",
            expected_tests,
            {"Debug": test_definitions, "Release": test_definitions},
        ),
    ):
        target_matches = [
            (identifier, body)
            for identifier, body in objects.items()
            if "\t\t\tisa = PBXNativeTarget;" in body
            and pbx_string_setting(
                body, "name", f"generated project native target {identifier}"
            )
            == target
        ]
        if len(target_matches) != 1:
            raise BoundaryError(
                f"generated project must contain one PBXNativeTarget named {target}"
            )
        target_identifier, target_body = target_matches[0]
        list_identifier = pbx_identifier_setting(
            target_body,
            "buildConfigurationList",
            f"generated project target {target} ({target_identifier})",
        )
        configuration_list = objects.get(list_identifier)
        if configuration_list is None or "\t\t\tisa = XCConfigurationList;" not in configuration_list:
            raise BoundaryError(
                f"generated project target {target} buildConfigurationList is invalid"
            )
        configuration_identifiers = pbx_identifier_list(
            configuration_list,
            "buildConfigurations",
            f"generated project target {target} configuration list",
        )
        if len(configuration_identifiers) != 2 or len(set(configuration_identifiers)) != 2:
            raise BoundaryError(
                f"generated project target {target} must reference two configurations"
            )
        by_name: dict[str, str] = {}
        for configuration_identifier in configuration_identifiers:
            configuration_body = objects.get(configuration_identifier)
            if configuration_body is None or "\t\t\tisa = XCBuildConfiguration;" not in configuration_body:
                raise BoundaryError(
                    f"generated project target {target} references an invalid configuration"
                )
            configuration = pbx_string_setting(
                configuration_body,
                "name",
                f"generated project target {target} configuration {configuration_identifier}",
            )
            if configuration in by_name:
                raise BoundaryError(
                    f"generated project target {target} repeats {configuration}"
                )
            by_name[configuration] = configuration_body
        if set(by_name) != {"Debug", "Release"}:
            raise BoundaryError(
                f"generated project target {target} must reference Debug and Release"
            )
        normalized_pbx[target] = {}
        for configuration in ("Debug", "Release"):
            settings_body = pbx_build_settings(
                by_name[configuration],
                f"generated project {target} {configuration}",
            )
            require_dependency_setting_surface(
                pbx_setting_names(
                    settings_body, f"generated project {target} {configuration}"
                ),
                f"generated project {target} {configuration}",
                1,
            )
            settings = {
                label: pbx_list_setting(
                    settings_body,
                    key,
                    f"generated project {target} {configuration} {label}",
                )
                for label, key in pbx_keys.items()
            }
            require_link_settings(
                settings, expected, f"generated project {target} {configuration}"
            )
            definitions = pbx_list_setting(
                settings_body,
                "GCC_PREPROCESSOR_DEFINITIONS",
                f"generated project {target} {configuration} preprocessor definitions",
            )
            if definitions != expected_definitions[configuration]:
                raise BoundaryError(
                    f"generated project {target} {configuration} preprocessor "
                    f"definitions changed: {definitions}"
                )
            condition_key = "SWIFT_ACTIVE_COMPILATION_CONDITIONS"
            setting_names = pbx_setting_names(
                settings_body,
                f"generated project {target} {configuration}",
            )
            expected_internal_ui = (
                internal_ui_condition
                if target == "BioMotion" and configuration == "Debug"
                else None
            )
            if expected_internal_ui is not None:
                internal_ui = pbx_build_string_setting(
                    settings_body,
                    condition_key,
                    f"generated project {target} {configuration}",
                )
                if internal_ui != expected_internal_ui:
                    raise BoundaryError(
                        f"generated project {target} {configuration} "
                        "internal UI condition changed"
                    )
            elif condition_key in setting_names:
                raise BoundaryError(
                    f"generated project {target} {configuration} must not "
                    "enable internal UI"
                )
            normalized_pbx[target][configuration] = {
                "definitions": definitions,
                "swiftConditions": expected_internal_ui,
                "settings": settings,
            }

    normalized = {
        "generatedProject": normalized_pbx,
        "generatedProjectGraph": generated_graph,
        "projectYML": normalized_yaml,
        "toolchain": toolchain,
    }
    normalized_digest = hashlib.sha256(b"BioMotion project linkage v3\0")
    normalized_digest.update(canonical_json(normalized).encode("utf-8"))
    return {
        "normalizedLinkageSHA256": normalized_digest.hexdigest(),
        "projectPBXProjSHA256": sha256_file(generated, "generated project"),
        "projectYMLSHA256": sha256_file(project, "project.yml"),
    }


def git(repo: Path, *arguments: str) -> str:
    try:
        result = subprocess.run(
            [
                "/usr/bin/git",
                "-c",
                "core.fsmonitor=false",
                "-c",
                "core.untrackedCache=false",
                "-C",
                str(repo),
                *arguments,
            ],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            env=SUBPROCESS_ENVIRONMENT,
        )
    except OSError as error:
        raise BoundaryError(f"cannot execute git: {error}") from error
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or f"rc={result.returncode}"
        raise BoundaryError(f"git {' '.join(arguments)} failed: {detail}")
    return result.stdout.strip()


def git_bytes(repo: Path, *arguments: str) -> bytes:
    try:
        result = subprocess.run(
            [
                "/usr/bin/git",
                "-c",
                "core.fsmonitor=false",
                "-c",
                "core.untrackedCache=false",
                "-C",
                str(repo),
                *arguments,
            ],
            check=False,
            capture_output=True,
            env=SUBPROCESS_ENVIRONMENT,
        )
    except OSError as error:
        raise BoundaryError(f"cannot execute git: {error}") from error
    if result.returncode != 0:
        detail = result.stderr.decode("utf-8", errors="replace").strip()
        if not detail:
            detail = result.stdout.decode("utf-8", errors="replace").strip()
        raise BoundaryError(
            f"git {' '.join(arguments)} failed: {detail or f'rc={result.returncode}'}"
        )
    return result.stdout


def git_blob_digest(path: Path, algorithm: str, label: str) -> str:
    try:
        size = path.stat().st_size
        digest = hashlib.new(algorithm)
        digest.update(f"blob {size}\0".encode("ascii"))
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()
    except (OSError, ValueError) as error:
        raise BoundaryError(f"cannot hash {label}: {error}") from error


def checkout_tree_entries(source: Path, label: str) -> dict[str, tuple[str, str]]:
    raw = git_bytes(source, "ls-tree", "-r", "-z", "--full-tree", "HEAD")
    entries: dict[str, tuple[str, str]] = {}
    for record in raw.split(b"\0"):
        if not record:
            continue
        try:
            metadata, encoded_path = record.split(b"\t", 1)
            mode, object_type, object_id = metadata.decode("ascii").split(" ")
            path = encoded_path.decode("utf-8")
        except (UnicodeDecodeError, ValueError) as error:
            raise BoundaryError(f"{label} HEAD tree is malformed") from error
        relative = Path(path)
        if (
            not path
            or relative.is_absolute()
            or any(part in {"", ".", ".."} for part in relative.parts)
            or path in entries
        ):
            raise BoundaryError(f"{label} HEAD tree has an unsafe path: {path!r}")
        if object_type != "blob" or mode not in {"100644", "100755"}:
            raise BoundaryError(
                f"{label} HEAD tree contains an unsupported entry: {path}"
            )
        entries[path] = (mode, object_id)
    if not entries:
        raise BoundaryError(f"{label} HEAD tree is empty")
    return entries


def require_checkout_bytes(
    source: Path,
    label: str,
    allowed_generated_roots: set[str],
) -> None:
    """Verify the physical checkout, bypassing replace refs and index hints."""
    try:
        physical_source = source.resolve(strict=True)
    except OSError as error:
        raise BoundaryError(f"cannot resolve {label} checkout: {error}") from error
    top_level = Path(git(source, "rev-parse", "--show-toplevel"))
    try:
        physical_top_level = top_level.resolve(strict=True)
    except OSError as error:
        raise BoundaryError(f"cannot resolve {label} Git top-level: {error}") from error
    if physical_top_level != physical_source:
        raise BoundaryError(
            f"{label} Git top-level does not match the expected checkout"
        )

    dot_git = source / ".git"
    if dot_git.is_symlink() or not dot_git.is_dir():
        raise BoundaryError(f"{label} .git must be a local real directory")
    absolute_git_dir = Path(git(source, "rev-parse", "--absolute-git-dir"))
    try:
        physical_git_dir = absolute_git_dir.resolve(strict=True)
        expected_git_dir = dot_git.resolve(strict=True)
    except OSError as error:
        raise BoundaryError(f"cannot resolve {label} Git directory: {error}") from error
    if physical_git_dir != expected_git_dir:
        raise BoundaryError(f"{label} Git directory is not local to the checkout")
    if git(source, "replace", "-l"):
        raise BoundaryError(f"{label} checkout contains replace refs")

    flags = git_bytes(source, "ls-files", "-v", "-z").split(b"\0")
    indexed_paths: set[str] = set()
    for record in flags:
        if not record:
            continue
        if not record.startswith(b"H "):
            display = record.decode("utf-8", errors="replace")
            raise BoundaryError(
                f"{label} index contains assume-unchanged/skip-worktree state: "
                f"{display}"
            )
        try:
            indexed_paths.add(record[2:].decode("utf-8"))
        except UnicodeDecodeError as error:
            raise BoundaryError(f"{label} index contains a non-UTF-8 path") from error

    entries = checkout_tree_entries(source, label)
    if indexed_paths != set(entries):
        raise BoundaryError(f"{label} index does not exactly match HEAD")
    object_format = git(source, "rev-parse", "--show-object-format")
    if object_format not in {"sha1", "sha256"}:
        raise BoundaryError(
            f"{label} uses an unsupported Git object format: {object_format}"
        )
    for relative, (mode, object_id) in entries.items():
        path = source / relative
        try:
            metadata = path.lstat()
        except OSError as error:
            raise BoundaryError(f"{label} tracked path is missing: {relative}") from error
        if not stat.S_ISREG(metadata.st_mode):
            raise BoundaryError(f"{label} tracked path is not regular: {relative}")
        executable = bool(metadata.st_mode & 0o111)
        if executable != (mode == "100755"):
            raise BoundaryError(f"{label} tracked mode changed: {relative}")
        actual = git_blob_digest(path, object_format, f"{label} {relative}")
        if actual != object_id:
            raise BoundaryError(f"{label} tracked bytes changed: {relative}")

    for generated_root in sorted(allowed_generated_roots):
        path = source / generated_root
        if path.is_symlink() or (path.exists() and not path.is_dir()):
            raise BoundaryError(
                f"{label} generated root is not a real directory: {generated_root}"
            )

    for directory, directory_names, file_names in os.walk(
        source, topdown=True, followlinks=False
    ):
        directory_path = Path(directory)
        relative_directory = directory_path.relative_to(source)
        if relative_directory == Path("."):
            excluded = {".git", *allowed_generated_roots}
            for name in list(directory_names):
                entry = directory_path / name
                if entry.is_symlink():
                    raise BoundaryError(
                        f"{label} checkout contains a directory symlink: {name}"
                    )
            directory_names[:] = [
                name for name in directory_names if name not in excluded
            ]
        else:
            for name in directory_names:
                entry = directory_path / name
                if entry.is_symlink():
                    relative = entry.relative_to(source).as_posix()
                    raise BoundaryError(
                        f"{label} checkout contains a directory symlink: {relative}"
                    )
        for name in file_names:
            entry = directory_path / name
            relative = entry.relative_to(source).as_posix()
            if relative not in entries:
                raise BoundaryError(
                    f"{label} checkout contains an unreviewed file: {relative}"
                )


def configured_remote_urls(source: Path) -> list[str]:
    remote_urls: list[str] = []
    for remote_name in git(source, "remote").splitlines():
        remote_urls.extend(
            git(source, "remote", "get-url", "--all", remote_name).splitlines()
        )
    return remote_urls


def inspect_checkout(
    root: Path,
    dependency: dict,
    expected_directory: str,
    display_name: str,
) -> tuple[Path, str, str]:
    source_directory = dependency.get("sourceDirectory")
    if source_directory != expected_directory:
        raise BoundaryError(
            f"dependency lock {expected_directory} sourceDirectory must be "
            f"{expected_directory}"
        )
    source = locked_path(root, source_directory, f"{display_name} source")
    if not source.is_dir():
        raise BoundaryError(
            f"{display_name} checkout is missing or not a real directory: {source}"
        )
    if not (source / ".git").exists():
        raise BoundaryError(f"{display_name} source is not a Git checkout: {source}")

    head = git(source, "rev-parse", "HEAD")
    expected = dependency["commit"]
    if head != expected:
        raise BoundaryError(
            f"{display_name} HEAD does not match lock: expected {expected}, got {head}"
        )
    status = git(source, "status", "--porcelain", "--untracked-files=all")
    if status:
        raise BoundaryError(
            f"{display_name} checkout is dirty: {status.splitlines()[0]}"
        )
    generated_roots = (
        {"build_ios", "build_sim", "build_xcframework"}
        if expected_directory == "nimblephysics"
        else {"build_ios", "build_sim"}
    )
    require_checkout_bytes(source, display_name, generated_roots)

    repository = dependency.get("repository")
    if not isinstance(repository, str) or not repository:
        raise BoundaryError(
            f"dependency lock {expected_directory} repository must be a non-empty string"
        )
    remote_urls = configured_remote_urls(source)
    expected_repository = canonical_repository(repository)
    matching_repositories = sorted(
        {
            canonical_repository(url)
            for url in remote_urls
            if canonical_repository(url) == expected_repository
        }
    )
    if not matching_repositories:
        actual = ", ".join(remote_urls) if remote_urls else "<none>"
        raise BoundaryError(
            f"{display_name} repository does not match lock: expected {repository}, "
            f"found {actual}"
        )
    return source, head, matching_repositories[0]


def inspect_qdldl_checkout(
    root: Path, dependency: dict, build: dict, sdk_label: str
) -> tuple[Path, str, str]:
    label = f"QDLDL {sdk_label}"
    source = locked_path(
        root, build.get("qdldlSourceDirectory"), f"{label} source"
    )
    if not source.is_dir() or not (source / ".git").exists():
        raise BoundaryError(f"{label} source is not a Git checkout: {source}")
    head = git(source, "rev-parse", "HEAD")
    expected = dependency["qdldlCommit"]
    if head != expected:
        raise BoundaryError(
            f"{label} HEAD does not match lock: expected {expected}, got {head}"
        )
    status = git(source, "status", "--porcelain", "--untracked-files=all")
    if status:
        raise BoundaryError(f"{label} checkout is dirty: {status.splitlines()[0]}")
    require_checkout_bytes(source, label, set())
    repository = dependency["qdldlRepository"]
    remote_urls = configured_remote_urls(source)
    expected_repository = canonical_repository(repository)
    matching_repositories = sorted(
        {
            canonical_repository(url)
            for url in remote_urls
            if canonical_repository(url) == expected_repository
        }
    )
    if not matching_repositories:
        actual = ", ".join(remote_urls) if remote_urls else "<none>"
        raise BoundaryError(
            f"{label} repository does not match lock: expected {repository}, "
            f"found {actual}"
        )
    return source, head, matching_repositories[0]


def builds_by_name(dependency: dict, label: str) -> dict[str, dict]:
    builds = dependency["builds"]
    by_name = {build["name"]: build for build in builds}
    if set(by_name) != {"device", "simulator"}:
        raise BoundaryError(f"{label} builds must be named device and simulator")
    return by_name


def inspect_common_build(
    root: Path,
    source: Path,
    build: dict,
    label: str,
    cmake_source: Path,
) -> tuple[dict[str, str], Path]:
    cache_path = locked_path(root, build.get("cache"), f"{label} cache")
    archive_path = locked_path(root, build.get("archive"), f"{label} archive")
    values = parse_cmake_cache(cache_path, label)
    require_cache_value(values, label, "CMAKE_BUILD_TYPE", build.get("configuration"))
    require_cache_value(values, label, "CMAKE_OSX_ARCHITECTURES", build.get("architecture"))
    require_cache_value(
        values, label, "CMAKE_OSX_DEPLOYMENT_TARGET", build.get("deploymentTarget")
    )
    require_cache_value(values, label, "CMAKE_OSX_SYSROOT", build.get("sysroot"))
    require_cache_value(values, label, "CMAKE_SYSTEM_NAME", "iOS")
    require_cache_value(values, label, "CMAKE_GENERATOR", build.get("generator"))
    require_cache_path(values, label, "CMAKE_HOME_DIRECTORY", cmake_source)
    require_regular_nonempty(archive_path, label)
    return values, archive_path


def normalized_cmake_settings(
    root: Path,
    values: dict[str, str],
    keys: tuple[str, ...],
    label: str,
) -> dict[str, str]:
    observed: dict[str, str] = {}
    root_prefix = str(root) + "/"
    for key in keys:
        value = values.get(key)
        if value is None:
            raise BoundaryError(f"{label} {key} is missing from observed settings")
        if key == "CMAKE_HOME_DIRECTORY":
            try:
                value = Path(value).resolve(strict=True).relative_to(root).as_posix()
            except (OSError, ValueError) as error:
                raise BoundaryError(
                    f"{label} {key} cannot be normalized repository-relatively"
                ) from error
        elif key == "CMAKE_C_FLAGS":
            value = value.replace(root_prefix, "")
        if str(root) in value:
            raise BoundaryError(f"{label} {key} snapshot contains an absolute root")
        observed[key] = value
    return observed


def inspect_osqp_builds(root: Path, source: Path, dependency: dict) -> dict:
    by_name = builds_by_name(dependency, "OSQP")
    required_symbols = {
        "_osqp_setup",
        "_osqp_solve",
        "_osqp_cleanup",
        "_osqp_warm_start",
    }
    observed_builds: dict[str, dict] = {}
    observed_qdldl: dict[str, dict[str, str]] = {}
    for label in ("device", "simulator"):
        build = by_name[label]
        qdldl_source, qdldl_head, qdldl_repository = inspect_qdldl_checkout(
            root, dependency, build, label
        )
        values, archive_path = inspect_common_build(
            root, source, build, label, source
        )
        expected_c_flags = (
            f"-ffile-prefix-map={source}=osqp "
            f"-ffile-prefix-map={qdldl_source}=qdldl"
        )
        require_cache_value(values, label, "CMAKE_C_FLAGS", expected_c_flags)
        require_cache_value(values, label, "CMAKE_C_FLAGS_RELEASE", "-O3 -DNDEBUG")
        require_cache_value(
            values, label, "OSQP_ALGEBRA_BACKEND", build.get("algebraBackend")
        )
        require_cache_value(
            values,
            label,
            "OSQP_PROFILER_ANNOTATIONS",
            build.get("profilerAnnotations"),
        )
        for cache_key, lock_key in OSQP_BOOL_KEYS.items():
            flag = build.get(lock_key)
            if not isinstance(flag, bool):
                raise BoundaryError(f"{label} {lock_key} must be boolean")
            require_cache_value(values, label, cache_key, "ON" if flag else "OFF")
        expected_platform = "2" if label == "device" else "7"
        archive_observation = inspect_archive(
            archive_path,
            label,
            expected_platform,
            build["deploymentTarget"],
            required_symbols,
            build["archiveMemberCount"],
            build["archiveContentSHA256"],
        )
        generated_header_sha256 = inspect_generated_header(
            root,
            build.get("generatedHeader"),
            build.get("generatedHeaderSHA256"),
            label,
        )
        generated_header_surface(root, build, label, "osqp")
        content_sha256 = archive_observation["contentSHA256"]
        if not isinstance(content_sha256, str):
            raise BoundaryError(f"{label} archive content digest was not observed")
        observed_builds[label] = {
            "archiveContentSHA256": content_sha256,
            "archiveMemberCount": archive_observation["memberCount"],
            "cmake": normalized_cmake_settings(
                root, values, OSQP_CMAKE_KEYS, label
            ),
            "generatedHeaderSHA256": generated_header_sha256,
        }
        observed_qdldl[label] = {
            "head": qdldl_head,
            "repository": qdldl_repository,
        }
    return {"builds": observed_builds, "qdldl": observed_qdldl}


def inspect_nimble_builds(root: Path, source: Path, dependency: dict) -> dict:
    by_name = builds_by_name(dependency, "Nimble")
    observed_builds: dict[str, dict] = {}
    for sdk_label in ("device", "simulator"):
        label = f"Nimble {sdk_label}"
        build = by_name[sdk_label]
        values, archive_path = inspect_common_build(
            root, source, build, label, source / "ios"
        )
        host_probe = build.get("hostProbe")
        if not isinstance(host_probe, bool):
            raise BoundaryError(f"{label} hostProbe must be boolean")
        require_cache_value(
            values,
            label,
            "NIMBLE_IOS_HOST_PROBE",
            "ON" if host_probe else "OFF",
        )
        actual_sha256 = sha256_file(archive_path, f"{label} archive")
        expected_sha256 = build["sha256"]
        if actual_sha256 != expected_sha256:
            raise BoundaryError(
                f"{label} archive SHA-256 does not match lock: expected "
                f"{expected_sha256}, got {actual_sha256}"
            )
        generated_header_sha256 = inspect_generated_header(
            root,
            build.get("generatedHeader"),
            build.get("generatedHeaderSHA256"),
            label,
        )
        generated_header_surface(root, build, label, "nimblephysics")
        expected_platform = "2" if sdk_label == "device" else "7"
        inspect_archive(
            archive_path,
            label,
            expected_platform,
            build["deploymentTarget"],
        )
        observed_builds[sdk_label] = {
            "archiveSHA256": actual_sha256,
            "cmake": normalized_cmake_settings(
                root, values, NIMBLE_CMAKE_KEYS, label
            ),
            "generatedHeaderSHA256": generated_header_sha256,
        }
    return observed_builds


def inspect(repo_root: Path) -> dict:
    root = repo_root.resolve(strict=True)
    lock, lock_sha256 = load_lock(root / "tools" / "dependencies.lock.json")
    dependencies = lock["dependencies"]

    osqp = dependencies["osqp"]
    osqp_source, osqp_head, osqp_repository = inspect_checkout(
        root, osqp, "osqp", "OSQP"
    )
    osqp_observation = inspect_osqp_builds(root, osqp_source, osqp)

    nimble = dependencies["nimblephysics"]
    nimble_source, nimble_head, nimble_repository = inspect_checkout(
        root, nimble, "nimblephysics", "Nimble"
    )
    nimble_builds = inspect_nimble_builds(root, nimble_source, nimble)

    project_linkage = inspect_project_linkage(root, lock)
    return {
        "dependencies": {
            "nimblephysics": {
                "builds": nimble_builds,
                "head": nimble_head,
                "repository": nimble_repository,
            },
            "osqp": {
                "builds": osqp_observation["builds"],
                "head": osqp_head,
                "qdldl": osqp_observation["qdldl"],
                "repository": osqp_repository,
            },
        },
        "dependencyLockSHA256": lock_sha256,
        "projectLinkage": project_linkage,
        "schemaVersion": 1,
    }


def main(arguments: list[str]) -> int:
    snapshot_mode = len(arguments) == 3 and arguments[1] == "snapshot"
    default_mode = len(arguments) == 2
    if not snapshot_mode and not default_mode:
        print(f"usage: {arguments[0]} [snapshot] REPO_ROOT", file=sys.stderr)
        return 2
    repo_root = arguments[2] if snapshot_mode else arguments[1]
    try:
        observation = inspect(Path(repo_root))
    except (BoundaryError, OSError) as error:
        fail(str(error))
    if snapshot_mode:
        print(canonical_json(observation))
        return 0
    dependencies = observation["dependencies"]
    print(
        "DEPENDENCY_BOUNDARY_PASS "
        f"nimble={dependencies['nimblephysics']['head']} "
        f"osqp={dependencies['osqp']['head']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
