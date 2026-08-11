#!/usr/bin/env python3
"""Seal and verify a Release xcarchive against the reviewed dependency lock."""

from __future__ import annotations

import errno
import hashlib
import json
import os
from pathlib import Path
import plistlib
import re
import stat
import subprocess
import sys


SCHEMA_VERSION = 1
INSPECTOR_RELATIVE_PATH = ("tools", "release", "dependency_boundary.py")
ARCHIVE_INFO_PATH = ("Info.plist",)
APPLICATION_PATH = ("Products", "Applications", "BioMotion.app")
APPLICATION_INFO_PATH = APPLICATION_PATH + ("Info.plist",)
APPLICATION_PATH_TEXT = "/".join(APPLICATION_PATH)
MAX_PLIST_BYTES = 16 * 1024 * 1024
MAX_RECEIPT_BYTES = 1024 * 1024
MAX_SNAPSHOT_BYTES = 1024 * 1024
HASH_PATTERN = re.compile(r"[0-9a-f]{64}")
COMMIT_PATTERN = re.compile(r"[0-9a-f]{40}")

INSPECTOR_ENVIRONMENT = {
    "GIT_ATTR_NOSYSTEM": "1",
    "GIT_CONFIG_GLOBAL": "/dev/null",
    "GIT_CONFIG_NOSYSTEM": "1",
    "GIT_OPTIONAL_LOCKS": "0",
    "GIT_PAGER": "cat",
    "GIT_TERMINAL_PROMPT": "0",
    "HOME": "/var/empty",
    "LANG": "C",
    "LC_ALL": "C",
    "PAGER": "cat",
    "PATH": "/usr/bin:/bin:/usr/sbin:/sbin",
}

COMMON_CMAKE_KEYS = {
    "CMAKE_BUILD_TYPE",
    "CMAKE_GENERATOR",
    "CMAKE_HOME_DIRECTORY",
    "CMAKE_OSX_ARCHITECTURES",
    "CMAKE_OSX_DEPLOYMENT_TARGET",
    "CMAKE_OSX_SYSROOT",
    "CMAKE_SYSTEM_NAME",
}
NIMBLE_CMAKE_KEYS = COMMON_CMAKE_KEYS | {"NIMBLE_IOS_HOST_PROBE"}
OSQP_CMAKE_KEYS = COMMON_CMAKE_KEYS | {
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


class ReceiptError(RuntimeError):
    """A fail-closed archive receipt validation error."""


def reject_duplicate_keys(pairs: list[tuple[str, object]]) -> dict:
    value: dict = {}
    for key, item in pairs:
        if key in value:
            raise ReceiptError(f"JSON repeats key: {key}")
        value[key] = item
    return value


def reject_nonstandard_number(value: str) -> None:
    raise ReceiptError(f"JSON contains non-standard number: {value}")


def parse_json(data: bytes, label: str) -> object:
    try:
        text = data.decode("utf-8")
        return json.loads(
            text,
            object_pairs_hook=reject_duplicate_keys,
            parse_constant=reject_nonstandard_number,
        )
    except ReceiptError:
        raise
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ReceiptError(f"cannot parse {label}: {error}") from error


def require_exact_keys(value: dict, expected: set[str], label: str) -> None:
    actual = set(value)
    if actual == expected:
        return
    details: list[str] = []
    unexpected = sorted(actual - expected)
    missing = sorted(expected - actual)
    if unexpected:
        details.append(f"unexpected={','.join(unexpected)}")
    if missing:
        details.append(f"missing={','.join(missing)}")
    raise ReceiptError(f"{label} keys are invalid ({'; '.join(details)})")


def require_object(value: object, label: str) -> dict:
    if not isinstance(value, dict):
        raise ReceiptError(f"{label} must be an object")
    return value


def require_string(value: object, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ReceiptError(f"{label} must be a non-empty string")
    if any(ord(character) < 0x20 for character in value):
        raise ReceiptError(f"{label} contains a control character")
    return value


def require_hash(value: object, label: str) -> str:
    if not isinstance(value, str) or HASH_PATTERN.fullmatch(value) is None:
        raise ReceiptError(f"{label} must be 64 lowercase hex characters")
    return value


def require_commit(value: object, label: str) -> str:
    if not isinstance(value, str) or COMMIT_PATTERN.fullmatch(value) is None:
        raise ReceiptError(f"{label} must be 40 lowercase hex characters")
    return value


def open_directory(path: Path, label: str) -> int:
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    try:
        descriptor = os.open(str(path), flags)
    except OSError as error:
        raise ReceiptError(f"{label} is missing, linked, or not a directory: {path}") from error
    try:
        if not stat.S_ISDIR(os.fstat(descriptor).st_mode):
            raise ReceiptError(f"{label} is not a directory: {path}")
    except Exception:
        os.close(descriptor)
        raise
    return descriptor


def open_relative_directory(root: Path, components: tuple[str, ...], label: str) -> int:
    descriptor = open_directory(root, label)
    try:
        for component in components:
            validate_component(component, label)
            flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
            if hasattr(os, "O_CLOEXEC"):
                flags |= os.O_CLOEXEC
            child = os.open(component, flags, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        return descriptor
    except OSError as error:
        os.close(descriptor)
        raise ReceiptError(f"{label} is missing, linked, or not a directory") from error
    except Exception:
        os.close(descriptor)
        raise


def validate_component(component: str, label: str) -> None:
    if (
        not component
        or component in {".", ".."}
        or "/" in component
        or "\x00" in component
    ):
        raise ReceiptError(f"{label} contains an unsafe path component")


def unchanged(before: os.stat_result, after: os.stat_result) -> bool:
    return (
        before.st_dev,
        before.st_ino,
        before.st_mode,
        before.st_size,
        before.st_mtime_ns,
        before.st_ctime_ns,
    ) == (
        after.st_dev,
        after.st_ino,
        after.st_mode,
        after.st_size,
        after.st_mtime_ns,
        after.st_ctime_ns,
    )


def read_descriptor(descriptor: int, label: str, maximum: int | None = None) -> bytes:
    before = os.fstat(descriptor)
    if not stat.S_ISREG(before.st_mode):
        raise ReceiptError(f"{label} is not a regular file")
    if maximum is not None and before.st_size > maximum:
        raise ReceiptError(f"{label} exceeds {maximum} bytes")
    chunks: list[bytes] = []
    total = 0
    while True:
        chunk = os.read(descriptor, 1024 * 1024)
        if not chunk:
            break
        total += len(chunk)
        if maximum is not None and total > maximum:
            raise ReceiptError(f"{label} exceeds {maximum} bytes")
        chunks.append(chunk)
    after = os.fstat(descriptor)
    if total != before.st_size or not unchanged(before, after):
        raise ReceiptError(f"{label} changed while it was being read")
    return b"".join(chunks)


def read_relative_regular(
    root: Path,
    components: tuple[str, ...],
    label: str,
    maximum: int | None = None,
) -> bytes:
    if not components:
        raise ReceiptError(f"{label} has no file component")
    parent = open_relative_directory(root, components[:-1], label)
    descriptor = -1
    try:
        validate_component(components[-1], label)
        metadata = os.stat(components[-1], dir_fd=parent, follow_symlinks=False)
        if not stat.S_ISREG(metadata.st_mode):
            raise ReceiptError(f"{label} is linked or not a regular file")
        flags = os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        descriptor = os.open(components[-1], flags, dir_fd=parent)
        if not unchanged(metadata, os.fstat(descriptor)):
            raise ReceiptError(f"{label} changed while it was being opened")
        return read_descriptor(descriptor, label, maximum)
    except OSError as error:
        raise ReceiptError(f"{label} is missing, linked, or unreadable") from error
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        os.close(parent)


def hash_relative_regular(
    root: Path, components: tuple[str, ...], label: str
) -> str:
    if not components:
        raise ReceiptError(f"{label} has no file component")
    parent = open_relative_directory(root, components[:-1], label)
    descriptor = -1
    try:
        validate_component(components[-1], label)
        metadata = os.stat(components[-1], dir_fd=parent, follow_symlinks=False)
        if not stat.S_ISREG(metadata.st_mode):
            raise ReceiptError(f"{label} is linked or not a regular file")
        flags = os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        descriptor = os.open(components[-1], flags, dir_fd=parent)
        before = os.fstat(descriptor)
        if not unchanged(metadata, before):
            raise ReceiptError(f"{label} changed while it was being opened")
        digest = hashlib.sha256()
        total = 0
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            total += len(chunk)
            digest.update(chunk)
        after = os.fstat(descriptor)
        if total != before.st_size or not unchanged(before, after):
            raise ReceiptError(f"{label} changed while it was being hashed")
        return digest.hexdigest()
    except OSError as error:
        raise ReceiptError(f"{label} is missing, linked, or unreadable") from error
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        os.close(parent)


def update_field(digest: "hashlib._Hash", value: bytes) -> None:
    digest.update(len(value).to_bytes(8, "big"))
    digest.update(value)


def safe_name_bytes(name: str, label: str) -> bytes:
    validate_component(name, label)
    encoded = os.fsencode(name)
    if b"/" in encoded or b"\x00" in encoded:
        raise ReceiptError(f"{label} contains an unsafe path component")
    return encoded


def hash_tree_directory(
    descriptor: int,
    relative: bytes,
    digest: "hashlib._Hash",
    label: str,
) -> None:
    before = os.fstat(descriptor)
    if not stat.S_ISDIR(before.st_mode):
        raise ReceiptError(f"{label} contains a non-directory traversal target")
    try:
        with os.scandir(descriptor) as iterator:
            entries = sorted(iterator, key=lambda entry: os.fsencode(entry.name))
    except OSError as error:
        raise ReceiptError(f"cannot enumerate {label}") from error

    for entry in entries:
        name = safe_name_bytes(entry.name, label)
        child_relative = name if not relative else relative + b"/" + name
        try:
            entry_stat = entry.stat(follow_symlinks=False)
        except OSError as error:
            raise ReceiptError(f"cannot inspect an entry in {label}") from error
        mode = stat.S_IMODE(entry_stat.st_mode).to_bytes(4, "big")

        if stat.S_ISLNK(entry_stat.st_mode):
            raise ReceiptError(f"{label} contains a symbolic link")
        if stat.S_ISDIR(entry_stat.st_mode):
            digest.update(b"D")
            update_field(digest, child_relative)
            digest.update(mode)
            flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
            if hasattr(os, "O_CLOEXEC"):
                flags |= os.O_CLOEXEC
            try:
                child = os.open(entry.name, flags, dir_fd=descriptor)
            except OSError as error:
                raise ReceiptError(f"cannot safely open a directory in {label}") from error
            try:
                opened = os.fstat(child)
                if not unchanged(entry_stat, opened):
                    raise ReceiptError(f"{label} changed while it was being hashed")
                hash_tree_directory(child, child_relative, digest, label)
            finally:
                os.close(child)
            continue
        if not stat.S_ISREG(entry_stat.st_mode):
            raise ReceiptError(f"{label} contains a special file")

        flags = os.O_RDONLY | os.O_NOFOLLOW
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        try:
            child = os.open(entry.name, flags, dir_fd=descriptor)
        except OSError as error:
            raise ReceiptError(f"cannot safely open a file in {label}") from error
        try:
            opened = os.fstat(child)
            if (
                not stat.S_ISREG(opened.st_mode)
                or not unchanged(entry_stat, opened)
            ):
                raise ReceiptError(f"{label} changed while it was being hashed")
            content = hashlib.sha256()
            total = 0
            while True:
                chunk = os.read(child, 1024 * 1024)
                if not chunk:
                    break
                total += len(chunk)
                content.update(chunk)
            after = os.fstat(child)
            if total != opened.st_size or not unchanged(opened, after):
                raise ReceiptError(f"{label} changed while it was being hashed")
        finally:
            os.close(child)
        digest.update(b"F")
        update_field(digest, child_relative)
        digest.update(mode)
        digest.update(total.to_bytes(8, "big"))
        digest.update(content.digest())

    after = os.fstat(descriptor)
    if not unchanged(before, after):
        raise ReceiptError(f"{label} changed while it was being hashed")


def hash_tree(path: Path, label: str) -> str:
    descriptor = open_directory(path, label)
    try:
        digest = hashlib.sha256(b"BioMotion archive tree v1\x00")
        hash_tree_directory(descriptor, b"", digest, label)
        return digest.hexdigest()
    finally:
        os.close(descriptor)


def require_repository(value: object, label: str) -> str:
    repository = require_string(value, label)
    if not repository.startswith(("https://", "ssh://", "git@")):
        raise ReceiptError(f"{label} must be a normalized remote repository")
    if repository.startswith("file://") or repository.startswith("/"):
        raise ReceiptError(f"{label} must not be an absolute filesystem path")
    return repository


def validate_cmake_snapshot(
    value: object,
    expected_keys: set[str],
    expected_home: str,
    label: str,
    repository_root: Path,
) -> None:
    cmake = require_object(value, label)
    require_exact_keys(cmake, expected_keys, label)
    root_text = str(repository_root.resolve(strict=True))
    for key, item in cmake.items():
        setting = require_string(item, f"{label} {key}")
        if root_text in setting or setting.startswith(("/", "file://")):
            raise ReceiptError(f"{label} {key} contains an absolute path")
    home = cmake.get("CMAKE_HOME_DIRECTORY")
    if home != expected_home:
        raise ReceiptError(f"{label} CMAKE_HOME_DIRECTORY is not normalized")
    if "CMAKE_C_FLAGS" in cmake:
        flags = cmake["CMAKE_C_FLAGS"]
        if re.search(r"(?:^|=)/", flags) is not None:
            raise ReceiptError(f"{label} CMAKE_C_FLAGS contains an absolute path")


def validate_nimble_build(
    value: object, label: str, repository_root: Path
) -> None:
    build = require_object(value, label)
    require_exact_keys(
        build,
        {"archiveSHA256", "cmake", "generatedHeaderSHA256"},
        label,
    )
    require_hash(build.get("archiveSHA256"), f"{label} archiveSHA256")
    require_hash(
        build.get("generatedHeaderSHA256"), f"{label} generatedHeaderSHA256"
    )
    validate_cmake_snapshot(
        build.get("cmake"),
        NIMBLE_CMAKE_KEYS,
        "nimblephysics/ios",
        f"{label} cmake",
        repository_root,
    )


def validate_osqp_build(
    value: object, label: str, repository_root: Path
) -> None:
    build = require_object(value, label)
    require_exact_keys(
        build,
        {
            "archiveContentSHA256",
            "archiveMemberCount",
            "cmake",
            "generatedHeaderSHA256",
        },
        label,
    )
    require_hash(
        build.get("archiveContentSHA256"), f"{label} archiveContentSHA256"
    )
    count = build.get("archiveMemberCount")
    if type(count) is not int or count <= 0:
        raise ReceiptError(f"{label} archiveMemberCount must be a positive integer")
    require_hash(
        build.get("generatedHeaderSHA256"), f"{label} generatedHeaderSHA256"
    )
    validate_cmake_snapshot(
        build.get("cmake"),
        OSQP_CMAKE_KEYS,
        "osqp",
        f"{label} cmake",
        repository_root,
    )


def validate_observed_dependency_snapshot(
    value: object, repository_root: Path
) -> dict:
    snapshot = require_object(value, "dependency snapshot")
    require_exact_keys(
        snapshot,
        {
            "dependencies",
            "dependencyLockSHA256",
            "projectLinkage",
            "schemaVersion",
        },
        "dependency snapshot",
    )
    if type(snapshot.get("schemaVersion")) is not int or snapshot["schemaVersion"] != 1:
        raise ReceiptError("dependency snapshot schemaVersion must be integer 1")
    require_hash(
        snapshot.get("dependencyLockSHA256"),
        "dependency snapshot dependencyLockSHA256",
    )

    dependencies = require_object(
        snapshot.get("dependencies"), "dependency snapshot dependencies"
    )
    require_exact_keys(
        dependencies,
        {"nimblephysics", "osqp"},
        "dependency snapshot dependencies",
    )

    nimble = require_object(
        dependencies.get("nimblephysics"), "dependency snapshot nimblephysics"
    )
    require_exact_keys(
        nimble,
        {"builds", "head", "repository"},
        "dependency snapshot nimblephysics",
    )
    require_commit(nimble.get("head"), "dependency snapshot nimblephysics HEAD")
    require_repository(
        nimble.get("repository"), "dependency snapshot nimblephysics repository"
    )
    nimble_builds = require_object(
        nimble.get("builds"), "dependency snapshot nimblephysics builds"
    )
    require_exact_keys(
        nimble_builds,
        {"device", "simulator"},
        "dependency snapshot nimblephysics builds",
    )
    for sdk in ("device", "simulator"):
        validate_nimble_build(
            nimble_builds.get(sdk),
            f"dependency snapshot nimblephysics {sdk}",
            repository_root,
        )

    osqp = require_object(dependencies.get("osqp"), "dependency snapshot osqp")
    require_exact_keys(
        osqp,
        {"builds", "head", "qdldl", "repository"},
        "dependency snapshot osqp",
    )
    require_commit(osqp.get("head"), "dependency snapshot osqp HEAD")
    require_repository(osqp.get("repository"), "dependency snapshot osqp repository")
    osqp_builds = require_object(osqp.get("builds"), "dependency snapshot osqp builds")
    require_exact_keys(
        osqp_builds,
        {"device", "simulator"},
        "dependency snapshot osqp builds",
    )
    for sdk in ("device", "simulator"):
        validate_osqp_build(
            osqp_builds.get(sdk),
            f"dependency snapshot osqp {sdk}",
            repository_root,
        )

    qdldl = require_object(osqp.get("qdldl"), "dependency snapshot qdldl")
    require_exact_keys(
        qdldl, {"device", "simulator"}, "dependency snapshot qdldl"
    )
    for sdk in ("device", "simulator"):
        checkout = require_object(
            qdldl.get(sdk), f"dependency snapshot qdldl {sdk}"
        )
        require_exact_keys(
            checkout,
            {"head", "repository"},
            f"dependency snapshot qdldl {sdk}",
        )
        require_commit(
            checkout.get("head"), f"dependency snapshot qdldl {sdk} HEAD"
        )
        require_repository(
            checkout.get("repository"),
            f"dependency snapshot qdldl {sdk} repository",
        )

    linkage = require_object(
        snapshot.get("projectLinkage"), "dependency snapshot projectLinkage"
    )
    require_exact_keys(
        linkage,
        {
            "normalizedLinkageSHA256",
            "projectPBXProjSHA256",
            "projectYMLSHA256",
        },
        "dependency snapshot projectLinkage",
    )
    for key in (
        "normalizedLinkageSHA256",
        "projectPBXProjSHA256",
        "projectYMLSHA256",
    ):
        require_hash(linkage.get(key), f"dependency snapshot projectLinkage {key}")
    return snapshot


def canonical_snapshot(value: dict) -> bytes:
    return (
        json.dumps(value, ensure_ascii=True, separators=(",", ":"), sort_keys=True)
        + "\n"
    ).encode("utf-8")


def read_expected_snapshot(
    repository: Path, expected_lock_digest: str
) -> dict:
    require_hash(expected_lock_digest, "expected dependency lock SHA-256")
    data = sys.stdin.buffer.read(MAX_SNAPSHOT_BYTES + 1)
    if len(data) > MAX_SNAPSHOT_BYTES:
        raise ReceiptError("expected dependency snapshot is too large")
    if not data.endswith(b"\n") or data.count(b"\n") != 1 or b"\r" in data:
        raise ReceiptError("expected dependency snapshot must be exactly one JSON line")
    snapshot = validate_observed_dependency_snapshot(
        parse_json(data[:-1], "expected dependency snapshot"), repository
    )
    if data != canonical_snapshot(snapshot):
        raise ReceiptError("expected dependency snapshot is not canonical JSON")
    if snapshot["dependencyLockSHA256"] != expected_lock_digest:
        raise ReceiptError("expected dependency snapshot does not match the lock digest")
    return snapshot


def dependency_snapshot(repository: Path) -> dict:
    inspector = repository.joinpath(*INSPECTOR_RELATIVE_PATH)
    current = repository
    for component in INSPECTOR_RELATIVE_PATH:
        current = current / component
        if current.is_symlink():
            raise ReceiptError("dependency inspector path must not traverse symlinks")
    if inspector.is_symlink() or not inspector.is_file():
        raise ReceiptError("dependency inspector is missing or not a regular file")
    try:
        result = subprocess.run(
            [
                "/usr/bin/python3",
                "-I",
                str(inspector),
                "snapshot",
                str(repository),
            ],
            check=False,
            capture_output=True,
            env=INSPECTOR_ENVIRONMENT,
        )
    except OSError as error:
        raise ReceiptError(f"cannot execute dependency inspector: {error}") from error
    if result.returncode != 0:
        detail = result.stderr.decode("utf-8", errors="replace").strip()
        raise ReceiptError(
            f"dependency inspector failed: {detail or f'rc={result.returncode}'}"
        )
    if result.stderr:
        raise ReceiptError("dependency inspector wrote unexpected stderr")
    output = result.stdout
    if len(output) > MAX_SNAPSHOT_BYTES:
        raise ReceiptError("dependency inspector snapshot is too large")
    if not output.endswith(b"\n") or output.count(b"\n") != 1 or b"\r" in output:
        raise ReceiptError("dependency inspector must emit exactly one JSON line")
    snapshot = validate_observed_dependency_snapshot(
        parse_json(output[:-1], "dependency inspector snapshot"), repository
    )
    if output != canonical_snapshot(snapshot):
        raise ReceiptError("dependency inspector snapshot is not canonical JSON")
    return snapshot


def parse_plist(data: bytes, label: str) -> dict:
    try:
        value = plistlib.loads(data)
    except (plistlib.InvalidFileException, ValueError, TypeError, OverflowError) as error:
        raise ReceiptError(f"cannot parse {label}: {error}") from error
    if not isinstance(value, dict):
        raise ReceiptError(f"{label} root must be a dictionary")
    return value


def validate_executable_name(value: object) -> str:
    name = require_string(value, "CFBundleExecutable")
    validate_component(name, "CFBundleExecutable")
    return name


def path_is_within(child: Path, parent: Path) -> bool:
    try:
        child_text = os.path.realpath(str(child))
        parent_text = os.path.realpath(str(parent))
        return os.path.commonpath((child_text, parent_text)) == parent_text
    except (OSError, ValueError):
        return False


def archive_snapshot(repository: Path, archive: Path, receipt: Path) -> dict:
    if path_is_within(receipt, archive):
        raise ReceiptError("receipt must be stored outside the xcarchive")

    dependency = dependency_snapshot(repository)
    archive_tree = hash_tree(archive, "xcarchive")
    archive_info_data = read_relative_regular(
        archive, ARCHIVE_INFO_PATH, "xcarchive Info.plist", MAX_PLIST_BYTES
    )
    parse_plist(archive_info_data, "xcarchive Info.plist")

    application = archive.joinpath(*APPLICATION_PATH)
    application_tree = hash_tree(application, "BioMotion.app")
    application_info_data = read_relative_regular(
        archive,
        APPLICATION_INFO_PATH,
        "BioMotion.app Info.plist",
        MAX_PLIST_BYTES,
    )
    application_info = parse_plist(application_info_data, "BioMotion.app Info.plist")
    executable_name = validate_executable_name(application_info.get("CFBundleExecutable"))
    executable_hash = hash_relative_regular(
        archive,
        APPLICATION_PATH + (executable_name,),
        "BioMotion.app executable",
    )

    return {
        "schemaVersion": SCHEMA_VERSION,
        "dependencySnapshot": dependency,
        "archive": {
            "application": {
                "executable": {
                    "name": executable_name,
                    "sha256": executable_hash,
                },
                "infoPlist": {
                    "path": "Info.plist",
                    "sha256": hashlib.sha256(application_info_data).hexdigest(),
                },
                "path": APPLICATION_PATH_TEXT,
                "treeSHA256": application_tree,
            },
            "infoPlist": {
                "path": "Info.plist",
                "sha256": hashlib.sha256(archive_info_data).hexdigest(),
            },
            "treeSHA256": archive_tree,
        },
    }


def validate_receipt(value: object, repository_root: Path) -> dict:
    receipt = require_object(value, "receipt")
    require_exact_keys(
        receipt,
        {"archive", "dependencySnapshot", "schemaVersion"},
        "receipt",
    )
    if type(receipt.get("schemaVersion")) is not int or receipt["schemaVersion"] != 1:
        raise ReceiptError("receipt schemaVersion must be integer 1")
    validate_observed_dependency_snapshot(
        receipt.get("dependencySnapshot"), repository_root
    )

    archive = require_object(receipt.get("archive"), "receipt archive")
    require_exact_keys(archive, {"application", "infoPlist", "treeSHA256"}, "receipt archive")
    require_hash(archive.get("treeSHA256"), "receipt archive treeSHA256")
    archive_info = require_object(archive.get("infoPlist"), "receipt archive infoPlist")
    require_exact_keys(archive_info, {"path", "sha256"}, "receipt archive infoPlist")
    if archive_info.get("path") != "Info.plist":
        raise ReceiptError("receipt archive Info.plist path is invalid")
    require_hash(archive_info.get("sha256"), "receipt archive Info.plist sha256")

    application = require_object(archive.get("application"), "receipt application")
    require_exact_keys(
        application,
        {"executable", "infoPlist", "path", "treeSHA256"},
        "receipt application",
    )
    if application.get("path") != APPLICATION_PATH_TEXT:
        raise ReceiptError("receipt application path is invalid")
    require_hash(application.get("treeSHA256"), "receipt application treeSHA256")
    app_info = require_object(application.get("infoPlist"), "receipt application infoPlist")
    require_exact_keys(app_info, {"path", "sha256"}, "receipt application infoPlist")
    if app_info.get("path") != "Info.plist":
        raise ReceiptError("receipt application Info.plist path is invalid")
    require_hash(app_info.get("sha256"), "receipt application Info.plist sha256")
    executable = require_object(application.get("executable"), "receipt executable")
    require_exact_keys(executable, {"name", "sha256"}, "receipt executable")
    validate_executable_name(executable.get("name"))
    require_hash(executable.get("sha256"), "receipt executable sha256")
    return receipt


def encode_receipt(value: dict) -> bytes:
    return (json.dumps(value, ensure_ascii=True, indent=2, sort_keys=True) + "\n").encode(
        "utf-8"
    )


def read_receipt(path: Path) -> bytes:
    try:
        initial = os.lstat(path)
    except OSError as error:
        raise ReceiptError("receipt is missing, linked, or unreadable") from error
    if not stat.S_ISREG(initial.st_mode):
        raise ReceiptError("receipt is linked or not a regular file")
    flags = os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    try:
        descriptor = os.open(str(path), flags)
    except OSError as error:
        raise ReceiptError("receipt is missing, linked, or unreadable") from error
    try:
        metadata = os.fstat(descriptor)
        if not unchanged(initial, metadata):
            raise ReceiptError("receipt changed while it was being opened")
        if stat.S_IMODE(metadata.st_mode) != 0o600:
            raise ReceiptError("receipt permissions must be 0600")
        return read_descriptor(descriptor, "receipt", MAX_RECEIPT_BYTES)
    finally:
        os.close(descriptor)


def write_receipt(path: Path, data: bytes) -> None:
    parent = path.parent if str(path.parent) else Path(".")
    parent_descriptor = open_directory(parent, "receipt parent")
    descriptor = -1
    created = False
    name = path.name
    try:
        validate_component(name, "receipt path")
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        try:
            descriptor = os.open(name, flags, 0o600, dir_fd=parent_descriptor)
        except OSError as error:
            if error.errno == errno.EEXIST:
                raise ReceiptError("receipt already exists") from error
            raise ReceiptError("cannot create receipt") from error
        created = True
        os.fchmod(descriptor, 0o600)
        offset = 0
        while offset < len(data):
            written = os.write(descriptor, data[offset:])
            if written <= 0:
                raise ReceiptError("cannot write complete receipt")
            offset += written
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = -1
        os.fsync(parent_descriptor)
    except Exception:
        if descriptor >= 0:
            os.close(descriptor)
        if created:
            try:
                os.unlink(name, dir_fd=parent_descriptor)
                os.fsync(parent_descriptor)
            except OSError:
                pass
        raise
    finally:
        os.close(parent_descriptor)


def seal(
    repository: Path,
    archive: Path,
    receipt: Path,
    expected_lock_digest: str,
) -> None:
    expected_dependency = read_expected_snapshot(
        repository, expected_lock_digest
    )
    snapshot = archive_snapshot(repository, archive, receipt)
    if snapshot["dependencySnapshot"] != expected_dependency:
        raise ReceiptError(
            "dependency boundary changed after the expected snapshot was captured"
        )
    write_receipt(receipt, encode_receipt(snapshot))


def verify(repository: Path, archive: Path, receipt_path: Path) -> None:
    data = read_receipt(receipt_path)
    receipt = validate_receipt(parse_json(data, "receipt"), repository)
    if data != encode_receipt(receipt):
        raise ReceiptError("receipt is not in the canonical stable JSON format")
    current = archive_snapshot(repository, archive, receipt_path)
    if receipt != current:
        raise ReceiptError("dependency lock or xcarchive no longer matches the receipt")


def usage() -> None:
    print(
        "usage:\n"
        "  dependency_archive_receipt.py validate-snapshot REPO_ROOT LOCK_SHA256 < SNAPSHOT\n"
        "  dependency_archive_receipt.py seal REPO_ROOT ARCHIVE RECEIPT LOCK_SHA256 < SNAPSHOT\n"
        "  dependency_archive_receipt.py verify REPO_ROOT ARCHIVE RECEIPT",
        file=sys.stderr,
    )


def main(arguments: list[str]) -> int:
    if not arguments:
        usage()
        return 2
    command = arguments[0]
    if command == "validate-snapshot" and len(arguments) == 3:
        repository = Path(arguments[1])
        expected_lock_digest = arguments[2]
        archive = None
        receipt = None
    elif command == "seal" and len(arguments) == 5:
        repository = Path(arguments[1])
        archive = Path(arguments[2])
        receipt = Path(arguments[3])
        expected_lock_digest = arguments[4]
    elif command == "verify" and len(arguments) == 4:
        repository = Path(arguments[1])
        archive = Path(arguments[2])
        receipt = Path(arguments[3])
        expected_lock_digest = None
    else:
        usage()
        return 2
    try:
        if command == "validate-snapshot":
            snapshot = read_expected_snapshot(
                repository, expected_lock_digest
            )
            sys.stdout.buffer.write(canonical_snapshot(snapshot))
        elif command == "seal":
            seal(
                repository,
                archive,
                receipt,
                expected_lock_digest,
            )
            print("DEPENDENCY_ARCHIVE_RECEIPT_SEALED")
        else:
            verify(repository, archive, receipt)
            print("DEPENDENCY_ARCHIVE_RECEIPT_PASS")
    except ReceiptError as error:
        print(f"DEPENDENCY_ARCHIVE_RECEIPT_FAIL: {error}", file=sys.stderr)
        return 1
    except OSError as error:
        print(f"DEPENDENCY_ARCHIVE_RECEIPT_FAIL: operating-system error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
