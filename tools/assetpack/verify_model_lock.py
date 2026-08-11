#!/usr/bin/env python3
"""Fail-closed verifier for the SAM3DBodyPose Core ML supply chain lock."""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import itertools
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import sys
import tempfile
from typing import Any, Iterable, Mapping, Sequence
from urllib.parse import urlsplit


LOCK_SCHEMA_VERSION = 1
RECEIPT_SCHEMA_VERSION = 1
EXPECTED_ASSET_PACK_ID = "sam3d-body-pose"
EXPECTED_MODEL_BASE_NAME = "SAM3DBodyPose"
EXPECTED_LOCK_FILE = "SAM3DBodyPose.lock.json"
EXPECTED_MANIFEST_FILE = "Manifest.json"
EXPECTED_PLATFORMS = ["iOS"]
EXPECTED_INSTALLATION_EVENT_TYPES = ["firstInstallation", "subsequentUpdate"]
XCRUN = "/usr/bin/xcrun"
XCODEBUILD = "/usr/bin/xcodebuild"
WHAT = "/usr/bin/what"
MAX_JSON_BYTES = 16 * 1024 * 1024
HASH_CHUNK_BYTES = 4 * 1024 * 1024
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}\Z")
COMMIT_PATTERN = re.compile(r"[0-9a-f]{40}\Z")
IDENTIFIER_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_]*\Z")
ASSET_PACK_PATTERN = re.compile(r"[a-z0-9]+(?:-[a-z0-9]+)*\Z")
IOS_VERSION_PATTERN = re.compile(r"[0-9]+(?:\.[0-9]+)+\Z")


class VerificationError(RuntimeError):
    """An expected supply-chain invariant was not satisfied."""


class RecoveryRequiredError(VerificationError):
    """Publication state could not be durably restored without human recovery."""


def _fail(message: str) -> None:
    raise VerificationError(message)


def _format_names(names: Iterable[str]) -> str:
    return ", ".join(sorted(names))


def _expect_mapping(value: Any, location: str) -> dict[str, Any]:
    if type(value) is not dict:
        _fail(f"{location} must be a JSON object")
    return value


def _expect_list(value: Any, location: str) -> list[Any]:
    if type(value) is not list:
        _fail(f"{location} must be a JSON array")
    return value


def _expect_exact_keys(
    value: Any, expected_keys: set[str], location: str
) -> dict[str, Any]:
    mapping = _expect_mapping(value, location)
    actual_keys = set(mapping)
    missing = expected_keys - actual_keys
    extra = actual_keys - expected_keys
    if missing:
        _fail(f"{location} is missing keys: {_format_names(missing)}")
    if extra:
        _fail(f"{location} has unknown keys: {_format_names(extra)}")
    return mapping


def _expect_string(value: Any, location: str) -> str:
    if type(value) is not str or not value:
        _fail(f"{location} must be a non-empty string")
    return value


def _expect_positive_int(value: Any, location: str) -> int:
    if type(value) is not int or value <= 0:
        _fail(f"{location} must be a positive integer")
    return value


def _expect_bool(value: Any, location: str) -> bool:
    if type(value) is not bool:
        _fail(f"{location} must be a boolean")
    return value


def _validate_sha256(value: Any, location: str) -> str:
    digest = _expect_string(value, location)
    if SHA256_PATTERN.fullmatch(digest) is None:
        _fail(f"{location} must be a lowercase SHA-256 digest")
    return digest


def _validate_relative_path(value: Any, location: str, *, single: bool = False) -> str:
    raw_path = _expect_string(value, location)
    if "\\" in raw_path or "\0" in raw_path:
        _fail(f"{location} must use a safe POSIX relative path")
    path = PurePosixPath(raw_path)
    if not path.parts or path.is_absolute() or path.as_posix() != raw_path:
        _fail(f"{location} must be a normalized POSIX relative path")
    if any(part in {"", ".", ".."} for part in path.parts):
        _fail(f"{location} must not contain empty, dot, or parent components")
    if single and len(path.parts) != 1:
        _fail(f"{location} must be a single path component")
    return raw_path


def _open_regular_fd(path: Path, label: str) -> tuple[int, os.stat_result]:
    try:
        before = os.lstat(path)
    except FileNotFoundError:
        _fail(f"{label} is missing: {path}")
    except OSError as error:
        _fail(f"cannot inspect {label} {path}: {error}")

    if stat.S_ISLNK(before.st_mode):
        _fail(f"{label} must not be a symlink: {path}")
    if not stat.S_ISREG(before.st_mode):
        _fail(f"{label} must be a regular file: {path}")

    flags = os.O_RDONLY
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        _fail(f"cannot open {label} {path}: {error}")

    try:
        try:
            opened = os.fstat(descriptor)
        except OSError as error:
            _fail(f"cannot inspect opened {label} {path}: {error}")
        if not stat.S_ISREG(opened.st_mode):
            _fail(f"{label} changed to a non-regular file: {path}")
        if (opened.st_dev, opened.st_ino) != (before.st_dev, before.st_ino):
            _fail(f"{label} changed while it was being opened: {path}")
        return descriptor, opened
    except BaseException:
        try:
            os.close(descriptor)
        except OSError:
            pass
        raise


def _read_regular_bytes(path: Path, label: str) -> bytes:
    descriptor, opened = _open_regular_fd(path, label)
    try:
        if opened.st_size > MAX_JSON_BYTES:
            _fail(f"{label} exceeds the {MAX_JSON_BYTES}-byte JSON safety limit: {path}")
        chunks: list[bytes] = []
        remaining = opened.st_size
        while remaining:
            chunk = os.read(descriptor, min(remaining, 1024 * 1024))
            if not chunk:
                _fail(f"{label} was truncated while being read: {path}")
            chunks.append(chunk)
            remaining -= len(chunk)
        if os.read(descriptor, 1):
            _fail(f"{label} grew while being read: {path}")
        after = os.fstat(descriptor)
        if (after.st_size, after.st_mtime_ns) != (opened.st_size, opened.st_mtime_ns):
            _fail(f"{label} changed while being read: {path}")
        return b"".join(chunks)
    finally:
        os.close(descriptor)


def _hash_regular_file(path: Path, label: str) -> tuple[int, str]:
    descriptor, opened = _open_regular_fd(path, label)
    digest = hashlib.sha256()
    bytes_read = 0
    try:
        while True:
            chunk = os.read(descriptor, HASH_CHUNK_BYTES)
            if not chunk:
                break
            digest.update(chunk)
            bytes_read += len(chunk)
        after = os.fstat(descriptor)
        if (
            bytes_read != opened.st_size
            or after.st_size != opened.st_size
            or after.st_mtime_ns != opened.st_mtime_ns
        ):
            _fail(f"{label} changed while being hashed: {path}")
        return opened.st_size, digest.hexdigest()
    finally:
        os.close(descriptor)


def _copy_regular_file_snapshot(
    source_path: Path, destination_path: Path, label: str
) -> tuple[int, str]:
    """Copy one opened regular-file generation into a new private snapshot."""

    source_path = Path(source_path)
    destination_path = Path(destination_path)
    source_descriptor, source_opened = _open_regular_fd(source_path, label)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        destination_descriptor = os.open(destination_path, flags, 0o600)
    except OSError as error:
        try:
            os.close(source_descriptor)
        except OSError:
            pass
        _fail(f"cannot create private snapshot for {label} {destination_path}: {error}")

    digest = hashlib.sha256()
    bytes_copied = 0
    completed = False
    try:
        while True:
            try:
                chunk = os.read(source_descriptor, HASH_CHUNK_BYTES)
            except OSError as error:
                _fail(f"cannot read {label} while snapshotting {source_path}: {error}")
            if not chunk:
                break
            digest.update(chunk)
            bytes_copied += len(chunk)
            view = memoryview(chunk)
            while view:
                try:
                    written = os.write(destination_descriptor, view)
                except OSError as error:
                    _fail(
                        f"cannot write private snapshot for {label} "
                        f"{destination_path}: {error}"
                    )
                if written <= 0:
                    _fail(
                        f"could not fully write private snapshot for {label} "
                        f"{destination_path}"
                    )
                view = view[written:]

        try:
            source_after = os.fstat(source_descriptor)
            destination_after = os.fstat(destination_descriptor)
        except OSError as error:
            _fail(f"cannot inspect private snapshot for {label}: {error}")
        if (
            bytes_copied != source_opened.st_size
            or source_after.st_size != source_opened.st_size
            or source_after.st_mtime_ns != source_opened.st_mtime_ns
        ):
            _fail(f"{label} changed while its private snapshot was being copied")
        if not stat.S_ISREG(destination_after.st_mode):
            _fail(f"private snapshot for {label} is not a regular file")
        if destination_after.st_size != bytes_copied:
            _fail(f"private snapshot for {label} has an unexpected size")
        try:
            os.fsync(destination_descriptor)
        except OSError as error:
            _fail(f"cannot fsync private snapshot for {label}: {error}")
        completed = True
        return bytes_copied, digest.hexdigest()
    finally:
        close_errors = []
        for descriptor, description in (
            (destination_descriptor, "destination"),
            (source_descriptor, "source"),
        ):
            try:
                os.close(descriptor)
            except OSError as error:
                close_errors.append(f"{description}: {error}")
        if completed and close_errors:
            _fail(
                f"cannot close private snapshot descriptors for {label}: "
                + "; ".join(close_errors)
            )


def _reject_duplicate_keys(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            _fail(f"JSON contains duplicate key: {key}")
        result[key] = value
    return result


def _parse_json_text(text: str, label: str) -> Any:
    try:
        return json.loads(text, object_pairs_hook=_reject_duplicate_keys)
    except VerificationError:
        raise
    except json.JSONDecodeError as error:
        _fail(f"{label} is not valid JSON: {error}")


def load_json_file(path: Path, label: str) -> Any:
    """Read strict UTF-8 JSON from a non-symlink regular file."""

    raw = _read_regular_bytes(Path(path), label)
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as error:
        _fail(f"{label} is not UTF-8: {error}")
    return _parse_json_text(text, label)


def _validate_reference_record(value: Any, location: str) -> None:
    record = _expect_exact_keys(value, {"path", "sha256"}, location)
    _validate_relative_path(record["path"], f"{location}.path")
    _validate_sha256(record["sha256"], f"{location}.sha256")


def _validate_file_records(value: Any, location: str) -> set[str]:
    records = _expect_list(value, location)
    if not records:
        _fail(f"{location} must contain at least one file record")

    paths: set[str] = set()
    for index, item in enumerate(records):
        item_location = f"{location}[{index}]"
        record = _expect_exact_keys(item, {"path", "size", "sha256"}, item_location)
        relative_path = _validate_relative_path(
            record["path"], f"{item_location}.path"
        )
        if relative_path in paths:
            _fail(f"{location} contains duplicate path: {relative_path}")
        paths.add(relative_path)
        _expect_positive_int(record["size"], f"{item_location}.size")
        _validate_sha256(record["sha256"], f"{item_location}.sha256")

    for relative_path in paths:
        for parent in PurePosixPath(relative_path).parents:
            if parent == PurePosixPath("."):
                break
            if parent.as_posix() in paths:
                _fail(
                    f"{location} uses one path as both a file and directory: "
                    f"{parent.as_posix()}"
                )
    return paths


def _validate_toolchain(
    value: Any, expected_keys: set[str], location: str
) -> None:
    toolchain = _expect_exact_keys(value, expected_keys, location)
    for key in expected_keys:
        _expect_string(toolchain[key], f"{location}.{key}")


def _validate_feature_records(value: Any, location: str) -> set[str]:
    features = _expect_list(value, location)
    if not features:
        _fail(f"{location} must contain at least one feature")

    names: set[str] = set()
    for index, item in enumerate(features):
        item_location = f"{location}[{index}]"
        feature = _expect_exact_keys(
            item,
            {
                "name",
                "featureType",
                "dataType",
                "shape",
                "optional",
                "shapeFlexible",
            },
            item_location,
        )
        name = _expect_string(feature["name"], f"{item_location}.name")
        if IDENTIFIER_PATTERN.fullmatch(name) is None:
            _fail(f"{item_location}.name must be a Swift-compatible identifier")
        if name in names:
            _fail(f"{location} contains duplicate feature: {name}")
        names.add(name)

        if feature["featureType"] != "multiArray":
            _fail(f"{item_location}.featureType must be multiArray")
        if feature["dataType"] not in {"float16", "float32"}:
            _fail(f"{item_location}.dataType must be float16 or float32")

        shape = _expect_list(feature["shape"], f"{item_location}.shape")
        if not shape:
            _fail(f"{item_location}.shape must contain at least one dimension")
        for dimension_index, dimension in enumerate(shape):
            _expect_positive_int(
                dimension, f"{item_location}.shape[{dimension_index}]"
            )

        if _expect_bool(feature["optional"], f"{item_location}.optional"):
            _fail(f"{item_location}.optional must be false")
        if _expect_bool(
            feature["shapeFlexible"], f"{item_location}.shapeFlexible"
        ):
            _fail(f"{item_location}.shapeFlexible must be false")
    return names


def validate_lock_document(value: Any) -> dict[str, Any]:
    """Validate the complete, versioned lock format and cross-field invariants."""

    lock = _expect_exact_keys(
        value,
        {
            "schemaVersion",
            "artifactRevision",
            "assetPackID",
            "modelBaseName",
            "license",
            "provenance",
            "sourcePackage",
            "compiledModel",
            "interface",
        },
        "lock",
    )

    if type(lock["schemaVersion"]) is not int:
        _fail("lock.schemaVersion must be an integer")
    if lock["schemaVersion"] != LOCK_SCHEMA_VERSION:
        _fail(
            f"lock.schemaVersion must be {LOCK_SCHEMA_VERSION}; "
            f"found {lock['schemaVersion']!r}"
        )
    _expect_positive_int(lock["artifactRevision"], "lock.artifactRevision")

    asset_pack_id = _expect_string(lock["assetPackID"], "lock.assetPackID")
    if ASSET_PACK_PATTERN.fullmatch(asset_pack_id) is None:
        _fail("lock.assetPackID must be a lowercase hyphenated identifier")
    if asset_pack_id != EXPECTED_ASSET_PACK_ID:
        _fail(
            f"lock.assetPackID must be {EXPECTED_ASSET_PACK_ID!r}; "
            f"found {asset_pack_id!r}"
        )
    model_base_name = _expect_string(lock["modelBaseName"], "lock.modelBaseName")
    if IDENTIFIER_PATTERN.fullmatch(model_base_name) is None:
        _fail("lock.modelBaseName must be a Swift-compatible identifier")
    if model_base_name != EXPECTED_MODEL_BASE_NAME:
        _fail(
            f"lock.modelBaseName must be {EXPECTED_MODEL_BASE_NAME!r}; "
            f"found {model_base_name!r}"
        )

    license_record = _expect_exact_keys(
        lock["license"], {"file", "sha256", "upstreamPath"}, "lock.license"
    )
    _validate_relative_path(license_record["file"], "lock.license.file", single=True)
    _validate_sha256(license_record["sha256"], "lock.license.sha256")
    _validate_relative_path(
        license_record["upstreamPath"], "lock.license.upstreamPath"
    )

    provenance = _expect_exact_keys(
        lock["provenance"],
        {
            "repository",
            "exportRecipeCommit",
            "contract",
            "conversionLog",
            "coremlReport",
            "sourceInputs",
            "conversionToolchain",
            "compileToolchain",
        },
        "lock.provenance",
    )
    repository = _expect_string(
        provenance["repository"], "lock.provenance.repository"
    )
    parsed_repository = urlsplit(repository)
    if (
        parsed_repository.scheme != "https"
        or not parsed_repository.netloc
        or not parsed_repository.path.strip("/")
        or parsed_repository.username is not None
        or parsed_repository.password is not None
        or parsed_repository.query
        or parsed_repository.fragment
    ):
        _fail("lock.provenance.repository must be a canonical HTTPS repository URL")
    export_commit = _expect_string(
        provenance["exportRecipeCommit"], "lock.provenance.exportRecipeCommit"
    )
    if COMMIT_PATTERN.fullmatch(export_commit) is None:
        _fail("lock.provenance.exportRecipeCommit must be a lowercase 40-hex commit")

    _validate_reference_record(provenance["contract"], "lock.provenance.contract")
    _validate_reference_record(
        provenance["conversionLog"], "lock.provenance.conversionLog"
    )
    _validate_reference_record(
        provenance["coremlReport"], "lock.provenance.coremlReport"
    )
    _validate_file_records(provenance["sourceInputs"], "lock.provenance.sourceInputs")
    _validate_toolchain(
        provenance["conversionToolchain"],
        {
            "python",
            "torch",
            "coremltools",
            "computePrecision",
            "ioDType",
            "deploymentTarget",
        },
        "lock.provenance.conversionToolchain",
    )
    _validate_toolchain(
        provenance["compileToolchain"],
        {"xcode", "xcodeBuild", "coremlcompiler", "baPackage"},
        "lock.provenance.compileToolchain",
    )

    source = _expect_exact_keys(
        lock["sourcePackage"], {"directoryName", "files"}, "lock.sourcePackage"
    )
    source_name = _validate_relative_path(
        source["directoryName"], "lock.sourcePackage.directoryName", single=True
    )
    if source_name != f"{model_base_name}.mlpackage":
        _fail(
            "lock.sourcePackage.directoryName must equal "
            "lock.modelBaseName plus .mlpackage"
        )
    source_paths = _validate_file_records(
        source["files"], "lock.sourcePackage.files"
    )
    if "Manifest.json" not in source_paths:
        _fail("lock.sourcePackage.files must contain Manifest.json")

    compiled = _expect_exact_keys(
        lock["compiledModel"], {"directoryName", "files"}, "lock.compiledModel"
    )
    compiled_name = _validate_relative_path(
        compiled["directoryName"], "lock.compiledModel.directoryName", single=True
    )
    if compiled_name != f"{model_base_name}.mlmodelc":
        _fail(
            "lock.compiledModel.directoryName must equal "
            "lock.modelBaseName plus .mlmodelc"
        )
    compiled_paths = _validate_file_records(
        compiled["files"], "lock.compiledModel.files"
    )
    if "metadata.json" not in compiled_paths:
        _fail("lock.compiledModel.files must contain metadata.json")

    interface = _expect_exact_keys(
        lock["interface"],
        {
            "specificationVersion",
            "modelType",
            "minimumIOS",
            "generatedClassName",
            "allowAdditionalFeatures",
            "inputs",
            "outputs",
        },
        "lock.interface",
    )
    if type(interface["specificationVersion"]) is not int:
        _fail("lock.interface.specificationVersion must be an integer")
    if interface["specificationVersion"] != 8:
        _fail("lock.interface.specificationVersion must be 8")
    if interface["modelType"] != "mlProgram":
        _fail("lock.interface.modelType must be mlProgram")
    minimum_ios = _expect_string(
        interface["minimumIOS"], "lock.interface.minimumIOS"
    )
    if IOS_VERSION_PATTERN.fullmatch(minimum_ios) is None:
        _fail("lock.interface.minimumIOS must be a dotted numeric version")
    if minimum_ios != "17.0":
        _fail("lock.interface.minimumIOS must be 17.0")
    generated_class = _expect_string(
        interface["generatedClassName"], "lock.interface.generatedClassName"
    )
    if generated_class != model_base_name:
        _fail("lock.interface.generatedClassName must equal lock.modelBaseName")
    if _expect_bool(
        interface["allowAdditionalFeatures"],
        "lock.interface.allowAdditionalFeatures",
    ):
        _fail("lock.interface.allowAdditionalFeatures must be false")

    input_names = _validate_feature_records(
        interface["inputs"], "lock.interface.inputs"
    )
    output_names = _validate_feature_records(
        interface["outputs"], "lock.interface.outputs"
    )
    overlap = input_names & output_names
    if overlap:
        _fail(
            "lock.interface input and output names overlap: "
            f"{_format_names(overlap)}"
        )
    return lock


def load_and_validate_lock(path: Path) -> dict[str, Any]:
    """Load a lock file, rejecting malformed JSON and every unknown field."""

    return validate_lock_document(load_json_file(Path(path), "model lock"))


def verify_license(lock: Mapping[str, Any], license_path: Path) -> None:
    """Verify the exact licensed bytes named by a validated lock."""

    record = lock["license"]
    path = Path(license_path)
    if path.name != record["file"]:
        _fail(
            f"license filename must be {record['file']!r}; found {path.name!r}"
        )
    _, actual_digest = _hash_regular_file(path, "license file")
    if actual_digest != record["sha256"]:
        _fail(
            f"license SHA-256 mismatch for {path}: expected "
            f"{record['sha256']}, found {actual_digest}"
        )


def _expected_manifest_selectors(lock: Mapping[str, Any]) -> list[dict[str, str]]:
    return [
        {"directory": lock["compiledModel"]["directoryName"]},
        {"file": EXPECTED_LOCK_FILE},
        {"file": lock["license"]["file"]},
    ]


def validate_manifest_document(
    value: Any, lock: Mapping[str, Any]
) -> dict[str, Any]:
    """Validate the exact Apple-hosted release payload selected by Manifest.json."""

    validate_lock_document(lock)
    manifest = _expect_exact_keys(
        value,
        {"assetPackID", "downloadPolicy", "fileSelectors", "platforms"},
        "manifest",
    )

    if manifest["assetPackID"] != lock["assetPackID"]:
        _fail(
            "manifest.assetPackID must match lock.assetPackID "
            f"{lock['assetPackID']!r}; found {manifest['assetPackID']!r}"
        )

    platforms = _expect_list(manifest["platforms"], "manifest.platforms")
    if platforms != EXPECTED_PLATFORMS:
        _fail(
            f"manifest.platforms must be exactly {EXPECTED_PLATFORMS!r}; "
            f"found {platforms!r}"
        )

    policy = _expect_exact_keys(
        manifest["downloadPolicy"], {"prefetch"}, "manifest.downloadPolicy"
    )
    prefetch = _expect_exact_keys(
        policy["prefetch"],
        {"installationEventTypes"},
        "manifest.downloadPolicy.prefetch",
    )
    event_types = _expect_list(
        prefetch["installationEventTypes"],
        "manifest.downloadPolicy.prefetch.installationEventTypes",
    )
    if event_types != EXPECTED_INSTALLATION_EVENT_TYPES:
        _fail(
            "manifest.downloadPolicy.prefetch.installationEventTypes must be "
            f"exactly {EXPECTED_INSTALLATION_EVENT_TYPES!r}; found "
            f"{event_types!r}"
        )

    selectors = _expect_list(manifest["fileSelectors"], "manifest.fileSelectors")
    for index, selector_value in enumerate(selectors):
        location = f"manifest.fileSelectors[{index}]"
        selector = _expect_mapping(selector_value, location)
        if len(selector) != 1:
            _expect_exact_keys(selector, {"file"}, location)
        selector_key = next(iter(selector), "")
        if selector_key not in {"directory", "file"}:
            _fail(f"{location} has unknown keys: {_format_names(selector)}")
        _validate_relative_path(
            selector[selector_key], f"{location}.{selector_key}", single=True
        )

    expected_selectors = _expected_manifest_selectors(lock)
    if selectors != expected_selectors:
        _fail(
            "manifest.fileSelectors must be exactly the locked compiled model, "
            f"lock, and license selectors in order: {expected_selectors!r}; "
            f"found {selectors!r}"
        )
    return manifest


def load_and_verify_manifest(
    manifest_path: Path, lock: Mapping[str, Any]
) -> dict[str, Any]:
    """Load a strict Manifest.json and verify its complete release contract."""

    path = Path(manifest_path)
    if path.name != EXPECTED_MANIFEST_FILE:
        _fail(
            f"asset-pack manifest filename must be {EXPECTED_MANIFEST_FILE!r}; "
            f"found {path.name!r}"
        )
    return validate_manifest_document(load_json_file(path, "asset-pack manifest"), lock)


def _expected_directories(file_paths: Iterable[str]) -> set[str]:
    directories: set[str] = set()
    for relative_path in file_paths:
        for parent in PurePosixPath(relative_path).parents:
            if parent == PurePosixPath("."):
                break
            directories.add(parent.as_posix())
    return directories


def _ensure_contained(path: Path, root: Path, label: str) -> None:
    try:
        resolved = path.resolve(strict=True)
        resolved.relative_to(root)
    except (OSError, ValueError) as error:
        _fail(f"{label} escapes or cannot be resolved within the artifact: {error}")


def verify_artifact_tree(
    root_path: Path, artifact: Mapping[str, Any], label: str
) -> None:
    """Verify an exact regular-file tree, rejecting extra nodes and symlinks."""

    root_path = Path(root_path)
    if root_path.name != artifact["directoryName"]:
        _fail(
            f"{label} directory must be named {artifact['directoryName']!r}; "
            f"found {root_path.name!r}"
        )
    try:
        root_stat = os.lstat(root_path)
    except FileNotFoundError:
        _fail(f"{label} directory is missing: {root_path}")
    except OSError as error:
        _fail(f"cannot inspect {label} directory {root_path}: {error}")
    if stat.S_ISLNK(root_stat.st_mode):
        _fail(f"{label} directory must not be a symlink: {root_path}")
    if not stat.S_ISDIR(root_stat.st_mode):
        _fail(f"{label} path must be a directory: {root_path}")

    try:
        resolved_root = root_path.resolve(strict=True)
    except OSError as error:
        _fail(f"cannot resolve {label} directory {root_path}: {error}")

    expected_records = {record["path"]: record for record in artifact["files"]}
    expected_files = set(expected_records)
    expected_directories = _expected_directories(expected_files)
    actual_files: set[str] = set()
    actual_directories: set[str] = set()

    def walk_error(error: OSError) -> None:
        _fail(f"cannot traverse {label} directory {root_path}: {error}")

    try:
        for current, directory_names, file_names in os.walk(
            root_path, topdown=True, onerror=walk_error, followlinks=False
        ):
            current_path = Path(current)
            directory_names.sort()
            file_names.sort()

            for directory_name in directory_names:
                directory_path = current_path / directory_name
                relative = directory_path.relative_to(root_path).as_posix()
                try:
                    entry_stat = os.lstat(directory_path)
                except OSError as error:
                    _fail(f"cannot inspect {label} directory entry {relative}: {error}")
                if stat.S_ISLNK(entry_stat.st_mode):
                    _fail(f"{label} contains symlink directory: {relative}")
                if not stat.S_ISDIR(entry_stat.st_mode):
                    _fail(f"{label} contains non-directory tree entry: {relative}")
                _ensure_contained(directory_path, resolved_root, f"{label} entry {relative}")
                actual_directories.add(relative)

            for file_name in file_names:
                file_path = current_path / file_name
                relative = file_path.relative_to(root_path).as_posix()
                try:
                    entry_stat = os.lstat(file_path)
                except OSError as error:
                    _fail(f"cannot inspect {label} file entry {relative}: {error}")
                if stat.S_ISLNK(entry_stat.st_mode):
                    _fail(f"{label} contains symlink file: {relative}")
                if not stat.S_ISREG(entry_stat.st_mode):
                    _fail(f"{label} contains non-regular file: {relative}")
                _ensure_contained(file_path, resolved_root, f"{label} entry {relative}")
                actual_files.add(relative)
    except VerificationError:
        raise
    except OSError as error:
        _fail(f"cannot traverse {label} directory {root_path}: {error}")

    missing_directories = expected_directories - actual_directories
    extra_directories = actual_directories - expected_directories
    missing_files = expected_files - actual_files
    extra_files = actual_files - expected_files
    if missing_directories:
        _fail(
            f"{label} is missing directories: {_format_names(missing_directories)}"
        )
    if extra_directories:
        _fail(f"{label} has extra directories: {_format_names(extra_directories)}")
    if missing_files:
        _fail(f"{label} is missing files: {_format_names(missing_files)}")
    if extra_files:
        _fail(f"{label} has extra files: {_format_names(extra_files)}")

    for relative_path in sorted(expected_files):
        record = expected_records[relative_path]
        file_path = root_path.joinpath(*PurePosixPath(relative_path).parts)
        actual_size, actual_digest = _hash_regular_file(
            file_path, f"{label} file {relative_path}"
        )
        if actual_size != record["size"]:
            _fail(
                f"{label} size mismatch for {relative_path}: expected "
                f"{record['size']}, found {actual_size}"
            )
        if actual_digest != record["sha256"]:
            _fail(
                f"{label} SHA-256 mismatch for {relative_path}: expected "
                f"{record['sha256']}, found {actual_digest}"
            )


def _decode_protobuf_varint(
    data: bytes, offset: int, end: int
) -> tuple[int, int]:
    value = 0
    shift = 0
    cursor = offset
    while cursor < end and shift < 70:
        byte = data[cursor]
        cursor += 1
        value |= (byte & 0x7F) << shift
        if byte < 0x80:
            return value, cursor
        shift += 7
    raise ValueError("truncated or oversized protobuf varint")


def _split_repeated_metadata_entries(payload: bytes) -> list[bytes] | None:
    """Split a message made solely of repeated field-100 length-delimited entries."""

    entries: list[bytes] = []
    cursor = 0
    while cursor < len(payload):
        entry_start = cursor
        try:
            tag, cursor = _decode_protobuf_varint(payload, cursor, len(payload))
            if tag != (100 << 3) | 2:
                return None
            length, cursor = _decode_protobuf_varint(payload, cursor, len(payload))
        except ValueError:
            return None
        entry_end = cursor + length
        if entry_end > len(payload):
            return None
        entries.append(payload[entry_start:entry_end])
        cursor = entry_end
    if not 2 <= len(entries) <= 8:
        return None
    return entries


def _canonicalize_compiled_metadata_map(
    data: bytes, expected_sha256: str
) -> bytes | None:
    """Find a map-entry permutation whose complete bytes match the locked digest."""

    field_tag = bytes((0xA2, 0x06))  # Protobuf field 100, length-delimited.
    candidates: set[bytes] = set()
    search_from = 0
    while True:
        field_start = data.find(field_tag, search_from)
        if field_start < 0:
            break
        search_from = field_start + 1
        try:
            tag, length_offset = _decode_protobuf_varint(
                data, field_start, len(data)
            )
            if tag != (100 << 3) | 2:
                continue
            payload_length, payload_start = _decode_protobuf_varint(
                data, length_offset, len(data)
            )
        except ValueError:
            continue
        payload_end = payload_start + payload_length
        if payload_end > len(data):
            continue
        entries = _split_repeated_metadata_entries(data[payload_start:payload_end])
        if entries is None:
            continue
        for permutation in itertools.permutations(entries):
            candidate = data[:payload_start] + b"".join(permutation) + data[payload_end:]
            if hashlib.sha256(candidate).hexdigest() == expected_sha256:
                candidates.add(candidate)
    if len(candidates) == 1:
        return candidates.pop()
    return None


def normalize_compiled_coremldata(
    compiled_path: Path, lock: Mapping[str, Any]
) -> None:
    """Canonicalize only protobuf map order when that reconstructs locked bytes."""

    validate_lock_document(lock)
    compiled_path = Path(compiled_path)
    expected_directory = lock["compiledModel"]["directoryName"]
    if compiled_path.name != expected_directory:
        _fail(
            f"compiled model directory must be named {expected_directory!r}; "
            f"found {compiled_path.name!r}"
        )
    try:
        compiled_parent_stat = os.lstat(compiled_path.parent)
    except OSError as error:
        _fail(f"cannot inspect compiled model parent {compiled_path.parent}: {error}")
    if stat.S_ISLNK(compiled_parent_stat.st_mode):
        _fail(
            f"compiled model parent directory must not be a symlink: "
            f"{compiled_path.parent}"
        )
    if not stat.S_ISDIR(compiled_parent_stat.st_mode):
        _fail(f"compiled model parent must be a directory: {compiled_path.parent}")
    try:
        compiled_stat = os.lstat(compiled_path)
    except OSError as error:
        _fail(f"cannot inspect compiled model directory {compiled_path}: {error}")
    if stat.S_ISLNK(compiled_stat.st_mode):
        _fail(f"compiled model directory must not be a symlink: {compiled_path}")
    if not stat.S_ISDIR(compiled_stat.st_mode):
        _fail(f"compiled model path must be a directory: {compiled_path}")
    records = {
        record["path"]: record for record in lock["compiledModel"]["files"]
    }
    if "coremldata.bin" not in records:
        _fail("lock.compiledModel.files must contain coremldata.bin")
    record = records["coremldata.bin"]
    target = compiled_path / "coremldata.bin"
    original = _read_regular_bytes(target, "compiled coremldata.bin")
    original_digest = hashlib.sha256(original).hexdigest()
    topology_artifact = {
        "directoryName": lock["compiledModel"]["directoryName"],
        "files": [dict(item) for item in lock["compiledModel"]["files"]],
    }
    for item in topology_artifact["files"]:
        if item["path"] == "coremldata.bin":
            item["size"] = len(original)
            item["sha256"] = original_digest
            break
    verify_artifact_tree(
        compiled_path,
        topology_artifact,
        "compiler output before coremldata normalization",
    )
    if len(original) == record["size"] and original_digest == record["sha256"]:
        return

    canonical = _canonicalize_compiled_metadata_map(original, record["sha256"])
    if canonical is None or len(canonical) != record["size"]:
        _fail(
            "compiled coremldata.bin cannot be normalized to the locked SHA-256 "
            "by reordering only its protobuf metadata map entries"
        )

    try:
        target_stat = os.lstat(target)
    except OSError as error:
        _fail(f"cannot re-inspect compiled coremldata.bin: {error}")
    if stat.S_ISLNK(target_stat.st_mode) or not stat.S_ISREG(target_stat.st_mode):
        _fail("compiled coremldata.bin changed to a symlink or non-regular file")

    descriptor = -1
    temporary_name = ""
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=".coremldata.bin.normalized.", dir=target.parent
        )
        os.fchmod(descriptor, stat.S_IMODE(target_stat.st_mode))
        view = memoryview(canonical)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                _fail("could not fully write normalized compiled coremldata.bin")
            view = view[written:]
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = -1
        if _read_regular_bytes(target, "compiled coremldata.bin") != original:
            _fail("compiled coremldata.bin changed during normalization")
        os.replace(temporary_name, target)
        temporary_name = ""
        _fsync_directory(target.parent)
    except VerificationError:
        raise
    except OSError as error:
        _fail(f"could not atomically normalize compiled coremldata.bin: {error}")
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary_name:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            except OSError:
                pass


def _metadata_feature_map(value: Any, location: str) -> dict[str, dict[str, Any]]:
    rows = _expect_list(value, location)
    result: dict[str, dict[str, Any]] = {}
    for index, item in enumerate(rows):
        item_location = f"{location}[{index}]"
        row = _expect_mapping(item, item_location)
        if "name" not in row:
            _fail(f"{item_location} is missing key: name")
        name = _expect_string(row["name"], f"{item_location}.name")
        if name in result:
            _fail(f"{location} contains duplicate feature: {name}")
        result[name] = row
    return result


def _metadata_field(row: Mapping[str, Any], key: str, location: str) -> Any:
    if key not in row:
        _fail(f"{location} is missing key: {key}")
    return row[key]


def _metadata_flag(value: Any, location: str) -> bool:
    if type(value) is bool:
        return value
    if type(value) is int and value in {0, 1}:
        return bool(value)
    if type(value) is str and value in {"0", "1"}:
        return value == "1"
    _fail(f"{location} must be a binary boolean flag")


def _metadata_shape(value: Any, location: str) -> list[int]:
    if type(value) is not str:
        _fail(f"{location} must be a JSON-array string")
    parsed = _parse_json_text(value, location)
    shape = _expect_list(parsed, location)
    if not shape:
        _fail(f"{location} must contain at least one dimension")
    for index, dimension in enumerate(shape):
        _expect_positive_int(dimension, f"{location}[{index}]")
    return shape


def _validate_metadata_features(
    actual_value: Any,
    expected_value: Any,
    location: str,
) -> None:
    actual = _metadata_feature_map(actual_value, location)
    expected_rows = _expect_list(expected_value, f"expected {location}")
    expected = {row["name"]: row for row in expected_rows}

    missing = set(expected) - set(actual)
    extra = set(actual) - set(expected)
    if missing:
        _fail(f"{location} is missing features: {_format_names(missing)}")
    if extra:
        _fail(f"{location} has extra features: {_format_names(extra)}")

    type_names = {"multiArray": "MultiArray"}
    data_type_names = {"float16": "Float16", "float32": "Float32"}
    for name, expected_row in expected.items():
        row = actual[name]
        row_location = f"{location}.{name}"
        actual_type = _metadata_field(row, "type", row_location)
        expected_type = type_names[expected_row["featureType"]]
        if actual_type != expected_type:
            _fail(
                f"{row_location}.type mismatch: expected {expected_type!r}, "
                f"found {actual_type!r}"
            )

        actual_data_type = _metadata_field(row, "dataType", row_location)
        expected_data_type = data_type_names[expected_row["dataType"]]
        if actual_data_type != expected_data_type:
            _fail(
                f"{row_location}.dataType mismatch: expected "
                f"{expected_data_type!r}, found {actual_data_type!r}"
            )

        actual_shape = _metadata_shape(
            _metadata_field(row, "shape", row_location), f"{row_location}.shape"
        )
        if actual_shape != expected_row["shape"]:
            _fail(
                f"{row_location}.shape mismatch: expected "
                f"{expected_row['shape']!r}, found {actual_shape!r}"
            )

        actual_optional = _metadata_flag(
            _metadata_field(row, "isOptional", row_location),
            f"{row_location}.isOptional",
        )
        if actual_optional != expected_row["optional"]:
            _fail(
                f"{row_location}.isOptional mismatch: expected "
                f"{expected_row['optional']!r}, found {actual_optional!r}"
            )

        actual_flexible = _metadata_flag(
            _metadata_field(row, "hasShapeFlexibility", row_location),
            f"{row_location}.hasShapeFlexibility",
        )
        if actual_flexible != expected_row["shapeFlexible"]:
            _fail(
                f"{row_location}.hasShapeFlexibility mismatch: expected "
                f"{expected_row['shapeFlexible']!r}, found {actual_flexible!r}"
            )


def validate_coreml_metadata(value: Any, lock: Mapping[str, Any]) -> None:
    """Compare coremlcompiler metadata with every locked interface property."""

    validate_lock_document(lock)
    documents = _expect_list(value, "Core ML metadata")
    if len(documents) != 1:
        _fail("Core ML metadata must contain exactly one model document")
    metadata = _expect_mapping(documents[0], "Core ML metadata[0]")
    interface = lock["interface"]

    if "specificationVersion" not in metadata:
        _fail("Core ML metadata[0] is missing key: specificationVersion")
    if type(metadata["specificationVersion"]) is not int:
        _fail("Core ML specificationVersion must be an integer")
    if metadata["specificationVersion"] != interface["specificationVersion"]:
        _fail(
            "Core ML specificationVersion mismatch: expected "
            f"{interface['specificationVersion']!r}, found "
            f"{metadata['specificationVersion']!r}"
        )

    if "modelType" not in metadata:
        _fail("Core ML metadata[0] is missing key: modelType")
    model_type = _expect_mapping(metadata["modelType"], "Core ML metadata[0].modelType")
    if "name" not in model_type:
        _fail("Core ML metadata[0].modelType is missing key: name")
    expected_model_type = f"MLModelType_{interface['modelType']}"
    if model_type["name"] != expected_model_type:
        _fail(
            f"Core ML modelType mismatch: expected {expected_model_type!r}, "
            f"found {model_type['name']!r}"
        )

    if "generatedClassName" not in metadata:
        _fail("Core ML metadata[0] is missing key: generatedClassName")
    if metadata["generatedClassName"] != interface["generatedClassName"]:
        _fail(
            "Core ML generatedClassName mismatch: expected "
            f"{interface['generatedClassName']!r}, found "
            f"{metadata['generatedClassName']!r}"
        )

    if "availability" not in metadata:
        _fail("Core ML metadata[0] is missing key: availability")
    availability = _expect_mapping(
        metadata["availability"], "Core ML metadata[0].availability"
    )
    if "iOS" not in availability:
        _fail("Core ML metadata[0].availability is missing key: iOS")
    if availability["iOS"] != interface["minimumIOS"]:
        _fail(
            f"Core ML minimum iOS mismatch: expected {interface['minimumIOS']!r}, "
            f"found {availability['iOS']!r}"
        )

    if "inputSchema" not in metadata:
        _fail("Core ML metadata[0] is missing key: inputSchema")
    if "outputSchema" not in metadata:
        _fail("Core ML metadata[0] is missing key: outputSchema")
    _validate_metadata_features(
        metadata["inputSchema"], interface["inputs"], "Core ML inputSchema"
    )
    _validate_metadata_features(
        metadata["outputSchema"], interface["outputs"], "Core ML outputSchema"
    )


def inspect_source_metadata(source_package: Path) -> Any:
    """Run Apple's compiler metadata command for an already verified package."""

    command = [XCRUN, "coremlcompiler", "metadata", str(source_package)]
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            timeout=120,
        )
    except FileNotFoundError:
        _fail("xcrun is unavailable; Core ML source schema cannot be verified")
    except subprocess.TimeoutExpired:
        _fail("coremlcompiler metadata timed out after 120 seconds")
    except OSError as error:
        _fail(f"could not run coremlcompiler metadata: {error}")
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or "no diagnostic"
        _fail(
            "coremlcompiler metadata failed with exit code "
            f"{result.returncode}: {detail[-2000:]}"
        )
    return _parse_json_text(result.stdout, "coremlcompiler metadata output")


def _run_version_command(command: Sequence[str], label: str) -> str:
    try:
        result = subprocess.run(
            list(command),
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            timeout=30,
        )
    except (FileNotFoundError, OSError, UnicodeError) as error:
        _fail(f"cannot inspect {label}: {error}")
    except subprocess.TimeoutExpired:
        _fail(f"{label} version inspection timed out")
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or "no diagnostic"
        _fail(
            f"{label} version inspection failed with exit code "
            f"{result.returncode}: {detail[-2000:]}"
        )
    return result.stdout.strip()


def verify_compile_toolchain(lock: Mapping[str, Any]) -> None:
    """Require the exact Xcode, Core ML compiler, and ba-package lock."""

    validate_lock_document(lock)
    expected = lock["provenance"]["compileToolchain"]
    xcode_lines = _run_version_command([XCODEBUILD, "-version"], "Xcode").splitlines()
    actual_xcode = xcode_lines[0].removeprefix("Xcode ") if xcode_lines else ""
    actual_build = (
        xcode_lines[1].removeprefix("Build version ")
        if len(xcode_lines) == 2
        else ""
    )
    if actual_xcode != expected["xcode"] or actual_build != expected["xcodeBuild"]:
        _fail(
            "Xcode toolchain mismatch: expected "
            f"Xcode {expected['xcode']} build {expected['xcodeBuild']}, found "
            f"Xcode {actual_xcode or '<unknown>'} build {actual_build or '<unknown>'}"
        )

    actual_ba = _run_version_command(
        [XCRUN, "ba-package", "--version"], "ba-package"
    )
    if actual_ba != expected["baPackage"]:
        _fail(
            f"ba-package mismatch: expected {expected['baPackage']!r}, "
            f"found {actual_ba!r}"
        )

    compiler_path_text = _run_version_command(
        [XCRUN, "--find", "coremlcompiler"], "coremlcompiler path"
    )
    compiler_path = Path(compiler_path_text)
    if not compiler_path.is_absolute():
        _fail(f"xcrun returned a non-absolute coremlcompiler path: {compiler_path}")
    what_output = _run_version_command(
        [WHAT, str(compiler_path)], "coremlcompiler build"
    )
    marker = f"PROJECT:CoreML-{expected['coremlcompiler']}"
    if marker not in what_output:
        _fail(
            f"coremlcompiler mismatch: expected marker {marker!r}, found "
            f"{what_output[-1000:]!r}"
        )


def expected_archive_entries(lock: Mapping[str, Any]) -> dict[str, str]:
    """Return the exact path/type allowlist for a compliant Background Assets AAR."""

    validate_lock_document(lock)
    compiled_name = lock["compiledModel"]["directoryName"]
    compiled_prefix = f"Contents/{compiled_name}"
    entries = {
        "": "D",
        "Contents": "D",
        compiled_prefix: "D",
        f"Contents/{EXPECTED_LOCK_FILE}": "F",
        f"Contents/{lock['license']['file']}": "F",
        EXPECTED_MANIFEST_FILE: "F",
    }
    for directory in _expected_directories(
        record["path"] for record in lock["compiledModel"]["files"]
    ):
        entries[f"{compiled_prefix}/{directory}"] = "D"
    for record in lock["compiledModel"]["files"]:
        entries[f"{compiled_prefix}/{record['path']}"] = "F"
    return entries


def _validate_archive_path(value: Any, location: str) -> str:
    if type(value) is not str:
        _fail(f"{location} must be a string")
    if value == "":
        return value
    try:
        return _validate_relative_path(value, location)
    except VerificationError as error:
        _fail(f"{location} must be a safe relative archive path: {error}")


def expected_archive_file_sizes(
    lock: Mapping[str, Any],
    manifest_path: Path,
    lock_path: Path,
    license_path: Path,
) -> dict[str, int]:
    compiled_prefix = f"Contents/{lock['compiledModel']['directoryName']}"
    sizes = {
        f"{compiled_prefix}/{record['path']}": record["size"]
        for record in lock["compiledModel"]["files"]
    }
    sizes[EXPECTED_MANIFEST_FILE] = _hash_regular_file(
        Path(manifest_path), "asset-pack manifest"
    )[0]
    sizes[f"Contents/{EXPECTED_LOCK_FILE}"] = _hash_regular_file(
        Path(lock_path), "model lock"
    )[0]
    sizes[f"Contents/{lock['license']['file']}"] = _hash_regular_file(
        Path(license_path), "license file"
    )[0]
    return sizes


def validate_archive_listing(
    value: Any,
    lock: Mapping[str, Any],
    expected_file_sizes: Mapping[str, int] | None = None,
) -> None:
    """Reject every AAR member not in the locked release payload."""

    rows = _expect_list(value, "Apple Archive listing")
    actual: dict[str, str] = {}
    for index, row_value in enumerate(rows):
        location = f"Apple Archive listing[{index}]"
        row = _expect_mapping(row_value, location)
        allowed_keys = {
            "TYP",
            "PAT",
            "UID",
            "GID",
            "MOD",
            "FLG",
            "CTM",
            "MTM",
            "BTM",
            "DAT",
        }
        unknown_keys = set(row) - allowed_keys
        if unknown_keys:
            _fail(
                f"{location} has unsupported archive metadata fields: "
                f"{_format_names(unknown_keys)}"
            )
        if "PAT" not in row:
            _fail(f"{location} is missing key: PAT")
        if "TYP" not in row:
            _fail(f"{location} is missing key: TYP")
        path = _validate_archive_path(row["PAT"], f"{location}.PAT")
        entry_type = row["TYP"]
        if type(entry_type) is not str or not entry_type:
            _fail(f"{location}.TYP must be a non-empty string")
        if path in actual:
            _fail(f"Apple Archive listing contains duplicate path: {path!r}")
        actual[path] = entry_type
        if entry_type == "F":
            if "DAT" not in row:
                _fail(f"{location} is missing regular-file size field DAT")
            if type(row["DAT"]) is not int or row["DAT"] < 0:
                _fail(f"{location}.DAT must be a nonnegative integer")
            if expected_file_sizes is not None and path in expected_file_sizes:
                if row["DAT"] != expected_file_sizes[path]:
                    _fail(
                        f"Apple Archive file size mismatch for {path!r}: expected "
                        f"{expected_file_sizes[path]}, found {row['DAT']}"
                    )
        elif "DAT" in row:
            _fail(f"{location} non-file entry must not carry DAT")

    expected = expected_archive_entries(lock)
    missing = set(expected) - set(actual)
    extra = set(actual) - set(expected)
    if missing:
        _fail(f"Apple Archive listing is missing paths: {_format_names(missing)}")
    if extra:
        _fail(f"Apple Archive listing has extra paths: {_format_names(extra)}")
    for path, expected_type in sorted(expected.items()):
        if actual[path] != expected_type:
            _fail(
                f"Apple Archive entry type mismatch for {path!r}: expected "
                f"{expected_type!r}, found {actual[path]!r}"
            )


def _run_aa(arguments: Sequence[str], operation: str, timeout: int) -> str:
    command = [XCRUN, "aa", *arguments]
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            timeout=timeout,
        )
    except FileNotFoundError:
        _fail(f"xcrun is unavailable; cannot {operation}")
    except subprocess.TimeoutExpired:
        _fail(f"aa {operation} timed out after {timeout} seconds")
    except (OSError, UnicodeError) as error:
        _fail(f"could not run aa {operation}: {error}")
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or "no diagnostic"
        _fail(
            f"aa {operation} failed with exit code {result.returncode}: "
            f"{detail[-2000:]}"
        )
    return result.stdout


def inspect_archive_listing(aar_path: Path) -> Any:
    output = _run_aa(
        ["list", "-i", str(aar_path), "-list-format", "json"],
        "list",
        300,
    )
    return _parse_json_text(output, "aa list JSON output")


def _inspect_extracted_tree(root_path: Path) -> dict[str, str]:
    root_path = Path(root_path)
    try:
        root_stat = os.lstat(root_path)
    except OSError as error:
        _fail(f"cannot inspect extracted AAR root {root_path}: {error}")
    if stat.S_ISLNK(root_stat.st_mode) or not stat.S_ISDIR(root_stat.st_mode):
        _fail(f"extracted AAR root must be a non-symlink directory: {root_path}")

    actual = {"": "D"}

    def walk_error(error: OSError) -> None:
        _fail(f"cannot traverse extracted AAR root {root_path}: {error}")

    try:
        for current, directory_names, file_names in os.walk(
            root_path, topdown=True, onerror=walk_error, followlinks=False
        ):
            current_path = Path(current)
            directory_names.sort()
            file_names.sort()
            for name in directory_names:
                path = current_path / name
                relative = path.relative_to(root_path).as_posix()
                entry_stat = os.lstat(path)
                if stat.S_ISLNK(entry_stat.st_mode):
                    _fail(f"extracted AAR contains symlink directory: {relative}")
                if not stat.S_ISDIR(entry_stat.st_mode):
                    _fail(f"extracted AAR contains non-directory entry: {relative}")
                actual[relative] = "D"
            for name in file_names:
                path = current_path / name
                relative = path.relative_to(root_path).as_posix()
                entry_stat = os.lstat(path)
                if stat.S_ISLNK(entry_stat.st_mode):
                    _fail(f"extracted AAR contains symlink file: {relative}")
                if not stat.S_ISREG(entry_stat.st_mode):
                    _fail(f"extracted AAR contains non-regular file: {relative}")
                actual[relative] = "F"
    except VerificationError:
        raise
    except OSError as error:
        _fail(f"cannot traverse extracted AAR root {root_path}: {error}")
    return actual


def verify_extracted_archive(
    extraction_root: Path,
    lock: Mapping[str, Any],
    manifest_path: Path,
    lock_path: Path,
    license_path: Path,
) -> None:
    """Verify the extracted AAR tree, sidecars, compiled bytes, and interface."""

    expected = expected_archive_entries(lock)
    actual = _inspect_extracted_tree(extraction_root)
    missing = set(expected) - set(actual)
    extra = set(actual) - set(expected)
    if missing:
        _fail(f"extracted AAR is missing paths: {_format_names(missing)}")
    if extra:
        _fail(f"extracted AAR has extra paths: {_format_names(extra)}")
    for path, expected_type in expected.items():
        if actual[path] != expected_type:
            _fail(
                f"extracted AAR entry type mismatch for {path!r}: expected "
                f"{expected_type!r}, found {actual[path]!r}"
            )

    root = Path(extraction_root)
    expected_sidecars = {
        root / EXPECTED_MANIFEST_FILE: Path(manifest_path),
        root / "Contents" / EXPECTED_LOCK_FILE: Path(lock_path),
        root / "Contents" / lock["license"]["file"]: Path(license_path),
    }
    for extracted, repository in expected_sidecars.items():
        if _read_regular_bytes(extracted, f"extracted sidecar {extracted.name}") != (
            _read_regular_bytes(repository, f"repository sidecar {repository.name}")
        ):
            _fail(
                f"extracted sidecar {extracted.name} is not byte-identical to "
                f"{repository}"
            )

    extracted_manifest = load_and_verify_manifest(
        root / EXPECTED_MANIFEST_FILE, lock
    )
    validate_manifest_document(extracted_manifest, lock)
    compiled_path = root / "Contents" / lock["compiledModel"]["directoryName"]
    verify_artifact_tree(compiled_path, lock["compiledModel"], "AAR compiled model")
    metadata = load_json_file(compiled_path / "metadata.json", "AAR compiled metadata")
    validate_coreml_metadata(metadata, lock)


def verify_archive(
    aar_path: Path,
    lock: Mapping[str, Any],
    manifest_path: Path,
    lock_path: Path,
    license_path: Path,
    extraction_directory: Path | None = None,
) -> None:
    """List, safely extract, and fully verify an AAR against repository authority."""

    aar_path = Path(aar_path)
    expected_name = f"{lock['assetPackID']}.aar"
    if aar_path.name != expected_name:
        _fail(f"AAR filename must be {expected_name!r}; found {aar_path.name!r}")
    initial_fingerprint = _hash_regular_file(aar_path, "AAR")
    expected_sizes = expected_archive_file_sizes(
        lock, manifest_path, lock_path, license_path
    )
    validate_archive_listing(
        inspect_archive_listing(aar_path), lock, expected_sizes
    )
    if _hash_regular_file(aar_path, "AAR after listing") != initial_fingerprint:
        _fail("AAR changed between hashing and aa list")

    temporary: tempfile.TemporaryDirectory[str] | None = None
    if extraction_directory is None:
        temporary = tempfile.TemporaryDirectory(prefix="biomotion-aar-verify.")
        extraction_root = Path(temporary.name) / "extracted"
    else:
        extraction_root = Path(extraction_directory)
    try:
        if extraction_root.exists() or extraction_root.is_symlink():
            _fail(f"AAR extraction directory must not already exist: {extraction_root}")
        try:
            extraction_parent_stat = os.lstat(extraction_root.parent)
        except OSError as error:
            _fail(
                f"cannot inspect AAR extraction parent {extraction_root.parent}: "
                f"{error}"
            )
        if stat.S_ISLNK(extraction_parent_stat.st_mode):
            _fail(
                f"AAR extraction parent must not be a symlink: "
                f"{extraction_root.parent}"
            )
        if not stat.S_ISDIR(extraction_parent_stat.st_mode):
            _fail(f"AAR extraction parent must be a directory: {extraction_root.parent}")
        extraction_root.mkdir(mode=0o700)
        _run_aa(
            ["extract", "-i", str(aar_path), "-d", str(extraction_root)],
            "extract",
            1800,
        )
        verify_extracted_archive(
            extraction_root, lock, manifest_path, lock_path, license_path
        )
        if _hash_regular_file(aar_path, "AAR after extraction") != initial_fingerprint:
            _fail("AAR changed during aa extraction")
    finally:
        if temporary is not None:
            temporary.cleanup()


def _validate_receipt_file_record(
    value: Any, location: str, expected_file: str
) -> dict[str, Any]:
    record = _expect_exact_keys(value, {"file", "sha256"}, location)
    filename = _validate_relative_path(record["file"], f"{location}.file", single=True)
    if filename != expected_file:
        _fail(
            f"{location}.file must be {expected_file!r}; found {filename!r}"
        )
    _validate_sha256(record["sha256"], f"{location}.sha256")
    return record


def validate_receipt_document(
    value: Any, lock: Mapping[str, Any]
) -> dict[str, Any]:
    """Validate the strict, versioned package receipt schema and identities."""

    validate_lock_document(lock)
    receipt = _expect_exact_keys(
        value,
        {
            "schemaVersion",
            "assetPackID",
            "artifactRevision",
            "modelBaseName",
            "aar",
            "sidecars",
        },
        "receipt",
    )
    if type(receipt["schemaVersion"]) is not int:
        _fail("receipt.schemaVersion must be an integer")
    if receipt["schemaVersion"] != RECEIPT_SCHEMA_VERSION:
        _fail(
            f"receipt.schemaVersion must be {RECEIPT_SCHEMA_VERSION}; "
            f"found {receipt['schemaVersion']!r}"
        )
    _expect_string(receipt["assetPackID"], "receipt.assetPackID")
    _expect_positive_int(receipt["artifactRevision"], "receipt.artifactRevision")
    _expect_string(receipt["modelBaseName"], "receipt.modelBaseName")
    for key in ("assetPackID", "artifactRevision", "modelBaseName"):
        if receipt[key] != lock[key]:
            _fail(
                f"receipt.{key} must match lock.{key} {lock[key]!r}; "
                f"found {receipt[key]!r}"
            )

    aar = _expect_exact_keys(receipt["aar"], {"file", "size", "sha256"}, "receipt.aar")
    aar_file = _validate_relative_path(
        aar["file"], "receipt.aar.file", single=True
    )
    expected_aar_file = f"{lock['assetPackID']}.aar"
    if aar_file != expected_aar_file:
        _fail(
            f"receipt archive filename must be {expected_aar_file!r}; "
            f"found {aar_file!r}"
        )
    _expect_positive_int(aar["size"], "receipt.aar.size")
    _validate_sha256(aar["sha256"], "receipt.aar.sha256")

    sidecars = _expect_exact_keys(
        receipt["sidecars"], {"manifest", "lock", "license"}, "receipt.sidecars"
    )
    _validate_receipt_file_record(
        sidecars["manifest"], "receipt.sidecars.manifest", EXPECTED_MANIFEST_FILE
    )
    _validate_receipt_file_record(
        sidecars["lock"], "receipt.sidecars.lock", EXPECTED_LOCK_FILE
    )
    _validate_receipt_file_record(
        sidecars["license"],
        "receipt.sidecars.license",
        lock["license"]["file"],
    )
    return receipt


def _verified_release_inputs(
    lock: Mapping[str, Any],
    manifest_path: Path,
    lock_path: Path,
    license_path: Path,
) -> None:
    if Path(lock_path).name != EXPECTED_LOCK_FILE:
        _fail(
            f"model lock filename must be {EXPECTED_LOCK_FILE!r}; "
            f"found {Path(lock_path).name!r}"
        )
    loaded_lock = load_and_validate_lock(Path(lock_path))
    if loaded_lock != lock:
        _fail("model lock bytes do not describe the supplied validated lock")
    verify_license(lock, Path(license_path))
    load_and_verify_manifest(Path(manifest_path), lock)


def build_receipt_document(
    lock: Mapping[str, Any],
    aar_path: Path,
    manifest_path: Path,
    lock_path: Path,
    license_path: Path,
) -> dict[str, Any]:
    """Hash a verified release input set into the canonical receipt document."""

    _verified_release_inputs(lock, manifest_path, lock_path, license_path)
    aar_path = Path(aar_path)
    expected_aar_file = f"{lock['assetPackID']}.aar"
    if aar_path.name != expected_aar_file:
        _fail(
            f"AAR filename must be {expected_aar_file!r}; found {aar_path.name!r}"
        )
    aar_size, aar_digest = _hash_regular_file(aar_path, "AAR")
    sidecar_paths = {
        "manifest": Path(manifest_path),
        "lock": Path(lock_path),
        "license": Path(license_path),
    }
    sidecars: dict[str, dict[str, str]] = {}
    for name, path in sidecar_paths.items():
        _, digest = _hash_regular_file(path, f"{name} sidecar")
        sidecars[name] = {"file": path.name, "sha256": digest}
    receipt = {
        "schemaVersion": RECEIPT_SCHEMA_VERSION,
        "assetPackID": lock["assetPackID"],
        "artifactRevision": lock["artifactRevision"],
        "modelBaseName": lock["modelBaseName"],
        "aar": {
            "file": aar_path.name,
            "size": aar_size,
            "sha256": aar_digest,
        },
        "sidecars": sidecars,
    }
    return validate_receipt_document(receipt, lock)


def write_receipt(
    output_path: Path,
    lock: Mapping[str, Any],
    aar_path: Path,
    manifest_path: Path,
    lock_path: Path,
    license_path: Path,
) -> tuple[int, int]:
    """Create a new canonical receipt without following or replacing any path."""

    expected_receipt_name = f"{lock['assetPackID']}.aar.receipt.json"
    if Path(output_path).name != expected_receipt_name:
        _fail(
            f"receipt filename must be {expected_receipt_name!r}; "
            f"found {Path(output_path).name!r}"
        )
    receipt = build_receipt_document(
        lock, aar_path, manifest_path, lock_path, license_path
    )
    encoded = (
        json.dumps(receipt, indent=2, sort_keys=True, ensure_ascii=True) + "\n"
    ).encode("utf-8")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(Path(output_path), flags, 0o600)
    except OSError as error:
        _fail(f"cannot create receipt {output_path}: {error}")
    try:
        view = memoryview(encoded)
        while view:
            try:
                written = os.write(descriptor, view)
            except OSError as error:
                _fail(f"cannot write receipt {output_path}: {error}")
            if written <= 0:
                _fail(f"could not fully write receipt {output_path}")
            view = view[written:]
        try:
            os.fsync(descriptor)
        except OSError as error:
            _fail(f"cannot fsync receipt {output_path}: {error}")
        try:
            written_stat = os.fstat(descriptor)
        except OSError as error:
            _fail(f"cannot inspect written receipt {output_path}: {error}")
        if not stat.S_ISREG(written_stat.st_mode):
            _fail(f"written receipt is not a regular file: {output_path}")
        return written_stat.st_dev, written_stat.st_ino
    finally:
        os.close(descriptor)


def _verify_receipt_bindings(
    receipt_path: Path,
    aar_path: Path,
    manifest_path: Path,
    lock_path: Path,
    license_path: Path,
) -> None:
    """Verify receipt hash bindings only; release gates must also verify the AAR."""

    lock = load_and_validate_lock(Path(lock_path))
    expected_receipt_name = f"{lock['assetPackID']}.aar.receipt.json"
    if Path(receipt_path).name != expected_receipt_name:
        _fail(
            f"receipt filename must be {expected_receipt_name!r}; "
            f"found {Path(receipt_path).name!r}"
        )
    _verified_release_inputs(lock, manifest_path, lock_path, license_path)
    receipt = validate_receipt_document(
        load_json_file(Path(receipt_path), "asset-pack receipt"), lock
    )
    if Path(aar_path).name != receipt["aar"]["file"]:
        _fail(
            f"AAR filename must be {receipt['aar']['file']!r}; "
            f"found {Path(aar_path).name!r}"
        )
    aar_size, aar_digest = _hash_regular_file(Path(aar_path), "AAR")
    if receipt["aar"]["size"] != aar_size:
        _fail(
            "receipt AAR size mismatch: expected "
            f"{receipt['aar']['size']}, found {aar_size}"
        )
    if receipt["aar"]["sha256"] != aar_digest:
        _fail(
            "receipt AAR SHA-256 mismatch: expected "
            f"{receipt['aar']['sha256']}, found {aar_digest}"
        )

    for name, path in {
        "manifest": Path(manifest_path),
        "lock": Path(lock_path),
        "license": Path(license_path),
    }.items():
        _, digest = _hash_regular_file(path, f"{name} sidecar")
        expected_digest = receipt["sidecars"][name]["sha256"]
        if expected_digest != digest:
            _fail(
                f"receipt {name} SHA-256 mismatch: expected "
                f"{expected_digest}, found {digest}"
            )


def verify_receipt(
    receipt_path: Path,
    aar_path: Path,
    manifest_path: Path,
    lock_path: Path,
    license_path: Path,
    extraction_directory: Path | None = None,
) -> None:
    """Fully verify the AAR payload first, then its strict receipt bindings."""

    lock = load_and_validate_lock(Path(lock_path))
    verify_archive(
        aar_path,
        lock,
        manifest_path,
        lock_path,
        license_path,
        extraction_directory,
    )
    _verify_receipt_bindings(
        receipt_path, aar_path, manifest_path, lock_path, license_path
    )


def seal_archive_receipt(
    receipt_path: Path,
    aar_path: Path,
    manifest_path: Path,
    lock_path: Path,
    license_path: Path,
    extraction_directory: Path | None = None,
) -> None:
    """Seal one verified private snapshot, then atomically install its receipt."""

    receipt_path = Path(receipt_path)
    aar_path = Path(aar_path)
    manifest_path = Path(manifest_path)
    lock_path = Path(lock_path)
    license_path = Path(license_path)
    if lock_path.name != EXPECTED_LOCK_FILE:
        _fail(
            f"model lock filename must be {EXPECTED_LOCK_FILE!r}; "
            f"found {lock_path.name!r}"
        )
    if manifest_path.name != EXPECTED_MANIFEST_FILE:
        _fail(
            f"Manifest filename must be {EXPECTED_MANIFEST_FILE!r}; "
            f"found {manifest_path.name!r}"
        )
    try:
        receipt_parent = os.lstat(receipt_path.parent)
    except OSError as error:
        _fail(f"cannot inspect receipt parent {receipt_path.parent}: {error}")
    if stat.S_ISLNK(receipt_parent.st_mode) or not stat.S_ISDIR(
        receipt_parent.st_mode
    ):
        _fail(f"receipt parent must be a non-symlink directory: {receipt_path.parent}")
    try:
        os.lstat(receipt_path)
    except FileNotFoundError:
        pass
    except OSError as error:
        _fail(f"cannot inspect receipt destination {receipt_path}: {error}")
    else:
        _fail(f"receipt destination already exists: {receipt_path}")

    try:
        temporary = tempfile.TemporaryDirectory(
            prefix=".biomotion-seal.", dir=str(receipt_path.parent)
        )
    except OSError as error:
        _fail(f"cannot create private seal snapshot: {error}")
    try:
        with temporary:
            snapshot_root = Path(temporary.name)
            authority_root = snapshot_root / "authority"
            authority_root.mkdir(mode=0o700)
            snapshot_lock = authority_root / EXPECTED_LOCK_FILE
            _copy_regular_file_snapshot(lock_path, snapshot_lock, "seal model lock")
            lock = load_and_validate_lock(snapshot_lock)
            if license_path.name != lock["license"]["file"]:
                _fail(
                    f"license filename must be {lock['license']['file']!r}; "
                    f"found {license_path.name!r}"
                )
            expected_aar_name = f"{lock['assetPackID']}.aar"
            expected_receipt_name = f"{expected_aar_name}.receipt.json"
            if aar_path.name != expected_aar_name:
                _fail(
                    f"AAR filename must be {expected_aar_name!r}; "
                    f"found {aar_path.name!r}"
                )
            if receipt_path.name != expected_receipt_name:
                _fail(
                    f"receipt filename must be {expected_receipt_name!r}; "
                    f"found {receipt_path.name!r}"
                )

            snapshot_manifest = authority_root / EXPECTED_MANIFEST_FILE
            snapshot_license = authority_root / lock["license"]["file"]
            snapshot_aar = snapshot_root / expected_aar_name
            snapshot_receipt = snapshot_root / expected_receipt_name
            _copy_regular_file_snapshot(
                manifest_path, snapshot_manifest, "seal Manifest"
            )
            _copy_regular_file_snapshot(
                license_path, snapshot_license, "seal license"
            )
            snapshot_aar_fingerprint = _copy_regular_file_snapshot(
                aar_path, snapshot_aar, "seal AAR"
            )

            verify_archive(
                snapshot_aar,
                lock,
                snapshot_manifest,
                snapshot_lock,
                snapshot_license,
                extraction_directory,
            )
            write_receipt(
                snapshot_receipt,
                lock,
                snapshot_aar,
                snapshot_manifest,
                snapshot_lock,
                snapshot_license,
            )
            _verify_receipt_bindings(
                snapshot_receipt,
                snapshot_aar,
                snapshot_manifest,
                snapshot_lock,
                snapshot_license,
            )

            if _hash_regular_file(aar_path, "live AAR before receipt install") != (
                snapshot_aar_fingerprint
            ):
                _fail("AAR changed while it was being sealed")
            for live_path, snapshot_path in (
                (manifest_path, snapshot_manifest),
                (lock_path, snapshot_lock),
                (license_path, snapshot_license),
            ):
                if _read_regular_bytes(
                    live_path, f"live seal authority {live_path.name}"
                ) != _read_regular_bytes(
                    snapshot_path, f"snapshot seal authority {snapshot_path.name}"
                ):
                    _fail(f"seal authority changed while it was being sealed: {live_path}")

            _verify_receipt_bindings(
                snapshot_receipt,
                aar_path,
                manifest_path,
                lock_path,
                license_path,
            )
            _fsync_directory(snapshot_root)
            try:
                os.link(snapshot_receipt, receipt_path)
            except FileExistsError:
                _fail(f"receipt destination already exists: {receipt_path}")
            except OSError as error:
                _fail(f"cannot atomically install verified receipt {receipt_path}: {error}")
            _fsync_directory(receipt_path.parent)
    except VerificationError:
        raise
    except OSError as error:
        _fail(f"private seal snapshot operation failed: {error}")


def _rename_swap(source: Path, destination: Path) -> None:
    if sys.platform != "darwin":
        raise OSError("renameatx_np(RENAME_SWAP) is required on this platform")
    library = ctypes.CDLL(None, use_errno=True)
    renameatx = library.renameatx_np
    renameatx.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    ]
    renameatx.restype = ctypes.c_int
    at_fdcwd = -2
    rename_swap = 0x00000002
    result = renameatx(
        at_fdcwd,
        os.fsencode(source),
        at_fdcwd,
        os.fsencode(destination),
        rename_swap,
    )
    if result != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))


def _fsync_regular_file(path: Path, label: str) -> None:
    descriptor, _ = _open_regular_fd(Path(path), label)
    completed = False
    try:
        try:
            os.fsync(descriptor)
        except OSError as error:
            _fail(f"cannot fsync {label} {path}: {error}")
        completed = True
    finally:
        try:
            os.close(descriptor)
        except OSError as error:
            if completed:
                _fail(f"cannot close {label} after fsync {path}: {error}")


def _fsync_directory(path: Path) -> None:
    path = Path(path)
    try:
        before = os.lstat(path)
    except OSError as error:
        _fail(f"cannot inspect directory before fsync {path}: {error}")
    if stat.S_ISLNK(before.st_mode):
        _fail(f"directory to fsync must not be a symlink: {path}")
    if not stat.S_ISDIR(before.st_mode):
        _fail(f"path to fsync must be a directory: {path}")

    flags = os.O_RDONLY
    flags |= getattr(os, "O_DIRECTORY", 0)
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        _fail(f"cannot open directory for fsync {path}: {error}")
    completed = False
    try:
        try:
            opened = os.fstat(descriptor)
        except OSError as error:
            _fail(f"cannot inspect opened directory for fsync {path}: {error}")
        if not stat.S_ISDIR(opened.st_mode):
            _fail(f"path changed to a non-directory before fsync: {path}")
        if (opened.st_dev, opened.st_ino) != (before.st_dev, before.st_ino):
            _fail(f"directory changed while it was being opened for fsync: {path}")
        try:
            os.fsync(descriptor)
        except OSError as error:
            _fail(f"cannot fsync directory {path}: {error}")
        completed = True
    finally:
        try:
            os.close(descriptor)
        except OSError as error:
            if completed:
                _fail(f"cannot close directory after fsync {path}: {error}")


def atomic_publish_directory(candidate: Path, destination: Path) -> None:
    """Publish or replace a complete pair with one atomic directory operation."""

    candidate = Path(candidate)
    destination = Path(destination)
    for label, parent in (
        ("release candidate parent", candidate.parent),
        ("release destination parent", destination.parent),
    ):
        try:
            parent_stat = os.lstat(parent)
        except OSError as error:
            _fail(f"cannot inspect {label} {parent}: {error}")
        if stat.S_ISLNK(parent_stat.st_mode):
            _fail(f"{label} must not be a symlink: {parent}")
        if not stat.S_ISDIR(parent_stat.st_mode):
            _fail(f"{label} must be a directory: {parent}")
    try:
        candidate_stat = os.lstat(candidate)
    except OSError as error:
        _fail(f"cannot inspect release candidate directory {candidate}: {error}")
    if stat.S_ISLNK(candidate_stat.st_mode):
        _fail(f"release candidate directory must not be a symlink: {candidate}")
    if not stat.S_ISDIR(candidate_stat.st_mode):
        _fail(f"release candidate must be a directory: {candidate}")
    if candidate.parent.stat().st_dev != destination.parent.stat().st_dev:
        _fail("release candidate and destination must be on the same filesystem")

    destination_exists = destination.exists() or destination.is_symlink()
    if destination_exists:
        destination_stat = os.lstat(destination)
        if stat.S_ISLNK(destination_stat.st_mode):
            _fail(f"release destination must not be a symlink: {destination}")
        if not stat.S_ISDIR(destination_stat.st_mode):
            _fail(f"release destination must be a directory: {destination}")
        try:
            _rename_swap(candidate, destination)
        except OSError as error:
            _fail(f"atomic directory swap failed: {error}")
    else:
        try:
            os.rename(candidate, destination)
        except OSError as error:
            _fail(f"atomic first publication failed: {error}")

    def fsync_namespace_parents() -> None:
        _fsync_directory(destination.parent)
        if candidate.parent != destination.parent:
            _fsync_directory(candidate.parent)

    publication_fsync_error: VerificationError | None = None
    try:
        fsync_namespace_parents()
        return
    except VerificationError as error:
        publication_fsync_error = error

    try:
        if destination_exists:
            _rename_swap(candidate, destination)
        else:
            os.rename(destination, candidate)
    except OSError as rollback_error:
        raise RecoveryRequiredError(
            "post-publication fsync failed after the new release became visible "
            f"({publication_fsync_error}); rollback namespace failed "
            f"({rollback_error}); manual recovery is required. Preserve both "
            f"the release candidate {candidate} and destination {destination}"
        ) from rollback_error

    try:
        fsync_namespace_parents()
    except VerificationError as rollback_fsync_error:
        raise RecoveryRequiredError(
            "post-publication fsync failed after the new release became visible "
            f"({publication_fsync_error}); rollback changed the namespace back "
            f"but its fsync failed ({rollback_fsync_error}); manual recovery is "
            f"required. Preserve both the release candidate {candidate} and "
            f"destination {destination}"
        ) from rollback_fsync_error

    _fail(
        "post-publication fsync failed, so the publication was rolled back to "
        f"its previous namespace state: {publication_fsync_error}"
    )


def _verify_release_pair_directory(
    candidate: Path,
    manifest_path: Path,
    lock_path: Path,
    license_path: Path,
    extraction_directory: Path | None = None,
) -> tuple[Path, Path]:
    candidate = Path(candidate)
    lock = load_and_validate_lock(Path(lock_path))
    aar_name = f"{lock['assetPackID']}.aar"
    receipt_name = f"{aar_name}.receipt.json"
    try:
        candidate_stat = os.lstat(candidate)
    except OSError as error:
        _fail(f"cannot inspect release candidate directory {candidate}: {error}")
    if stat.S_ISLNK(candidate_stat.st_mode) or not stat.S_ISDIR(candidate_stat.st_mode):
        _fail(f"release candidate must be a non-symlink directory: {candidate}")
    actual_names: set[str] = set()
    try:
        for entry in os.scandir(candidate):
            if entry.is_symlink() or not entry.is_file(follow_symlinks=False):
                _fail(f"release candidate contains non-regular entry: {entry.name}")
            actual_names.add(entry.name)
    except VerificationError:
        raise
    except OSError as error:
        _fail(f"cannot inspect release candidate entries: {error}")
    expected_names = {aar_name, receipt_name}
    if actual_names != expected_names:
        _fail(
            "release candidate must contain exactly the AAR and receipt; "
            f"expected {sorted(expected_names)!r}, found {sorted(actual_names)!r}"
        )
    aar_path = candidate / aar_name
    receipt_path = candidate / receipt_name
    verify_receipt(
        receipt_path,
        aar_path,
        manifest_path,
        lock_path,
        license_path,
        extraction_directory,
    )
    return aar_path, receipt_path


def publish_release_pair(
    candidate: Path,
    destination: Path,
    manifest_path: Path,
    lock_path: Path,
    license_path: Path,
    extraction_directory: Path | None = None,
) -> None:
    """Fully verify a candidate pair, then atomically publish its directory."""

    aar_path, receipt_path = _verify_release_pair_directory(
        candidate,
        manifest_path,
        lock_path,
        license_path,
        extraction_directory,
    )
    _fsync_regular_file(aar_path, "release candidate AAR")
    _fsync_regular_file(receipt_path, "release candidate receipt")
    _fsync_directory(candidate)
    atomic_publish_directory(candidate, destination)


def _default_lock_path() -> Path:
    repository_root = Path(__file__).resolve().parents[2]
    return repository_root / "BioMotion/Resources/SAM3DBodyPose.lock.json"


def _default_manifest_path() -> Path:
    repository_root = Path(__file__).resolve().parents[2]
    return repository_root / "tools/assetpack/Manifest.json"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Verify the SAM3DBodyPose lock, license, exact artifact tree, and "
            "Core ML interface."
        )
    )
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--lock",
        type=Path,
        default=_default_lock_path(),
        help="lock JSON (default: checked-in SAM3DBodyPose.lock.json)",
    )
    common.add_argument(
        "--license",
        type=Path,
        help="license path (default: lock directory plus the locked filename)",
    )

    subparsers = parser.add_subparsers(dest="mode", required=True)
    subparsers.add_parser(
        "repository",
        parents=[common],
        help="verify the lock format and exact license bytes",
    )
    subparsers.add_parser(
        "toolchain",
        parents=[common],
        help="verify the exact locked Xcode/Core ML/ba-package toolchain",
    )
    source = subparsers.add_parser(
        "source",
        parents=[common],
        help="also verify a source .mlpackage and its compiler-reported interface",
    )
    source.add_argument("artifact", type=Path, help="source .mlpackage directory")
    compiled = subparsers.add_parser(
        "compiled",
        parents=[common],
        help="also verify a compiled .mlmodelc and its metadata.json interface",
    )
    compiled.add_argument("artifact", type=Path, help="compiled .mlmodelc directory")
    normalize_compiled = subparsers.add_parser(
        "normalize-compiled",
        parents=[common],
        help=(
            "canonically reorder nondeterministic Core ML metadata map entries "
            "only when doing so reconstructs the locked bytes"
        ),
    )
    normalize_compiled.add_argument(
        "artifact", type=Path, help="private compiled .mlmodelc directory to normalize"
    )
    manifest = subparsers.add_parser(
        "manifest",
        parents=[common],
        help="verify the exact Apple-hosted manifest and three payload selectors",
    )
    manifest.add_argument(
        "artifact",
        type=Path,
        nargs="?",
        default=_default_manifest_path(),
        help="asset-pack Manifest.json",
    )
    archive = subparsers.add_parser(
        "archive",
        parents=[common],
        help="list, extract, and verify an AAR exact allowlist and payload",
    )
    archive.add_argument("artifact", type=Path, help="asset-pack AAR")
    archive.add_argument(
        "--manifest",
        type=Path,
        default=_default_manifest_path(),
        help="repository asset-pack Manifest.json",
    )
    archive.add_argument(
        "--extract-directory",
        type=Path,
        help="new private extraction directory (default: verifier temporary directory)",
    )
    seal = subparsers.add_parser(
        "seal",
        parents=[common],
        help="fully verify one private AAR snapshot, then write and verify its receipt",
    )
    seal.add_argument("artifact", type=Path, help="asset-pack AAR")
    seal.add_argument("receipt", type=Path, help="new receipt path")
    seal.add_argument(
        "--manifest",
        type=Path,
        default=_default_manifest_path(),
        help="repository asset-pack Manifest.json",
    )
    seal.add_argument(
        "--extract-directory",
        type=Path,
        help="new private extraction directory (default: verifier temporary directory)",
    )
    receipt = subparsers.add_parser(
        "receipt",
        parents=[common],
        help="verify a strict sidecar receipt against an AAR and repository inputs",
    )
    receipt.add_argument("artifact", type=Path, help="asset-pack AAR")
    receipt.add_argument("receipt", type=Path, help="receipt JSON")
    receipt.add_argument(
        "--manifest",
        type=Path,
        default=_default_manifest_path(),
        help="repository asset-pack Manifest.json",
    )
    receipt.add_argument(
        "--extract-directory",
        type=Path,
        help="new private extraction directory (default: verifier temporary directory)",
    )
    publish = subparsers.add_parser(
        "publish",
        parents=[common],
        help="fully verify a two-file release directory, then atomically publish it",
    )
    publish.add_argument("candidate", type=Path, help="private candidate directory")
    publish.add_argument("destination", type=Path, help="published release directory")
    publish.add_argument(
        "--manifest",
        type=Path,
        default=_default_manifest_path(),
        help="repository asset-pack Manifest.json",
    )
    publish.add_argument(
        "--extract-directory",
        type=Path,
        help="new private extraction directory (default: verifier temporary directory)",
    )
    return parser


def _verify_repository(lock_path: Path, license_path: Path | None) -> dict[str, Any]:
    lock = load_and_validate_lock(lock_path)
    resolved_license = (
        license_path
        if license_path is not None
        else lock_path.parent / lock["license"]["file"]
    )
    verify_license(lock, resolved_license)
    return lock


def run(arguments: Sequence[str] | None = None) -> None:
    args = _build_parser().parse_args(arguments)
    lock = _verify_repository(args.lock, args.license)
    license_path = (
        args.license
        if args.license is not None
        else args.lock.parent / lock["license"]["file"]
    )

    if args.mode == "toolchain":
        verify_compile_toolchain(lock)
    elif args.mode == "source":
        verify_artifact_tree(args.artifact, lock["sourcePackage"], "source package")
        validate_coreml_metadata(inspect_source_metadata(args.artifact), lock)
    elif args.mode == "compiled":
        verify_artifact_tree(args.artifact, lock["compiledModel"], "compiled model")
        metadata = load_json_file(args.artifact / "metadata.json", "compiled metadata")
        validate_coreml_metadata(metadata, lock)
    elif args.mode == "normalize-compiled":
        normalize_compiled_coremldata(args.artifact, lock)
    elif args.mode == "manifest":
        load_and_verify_manifest(args.artifact, lock)
    elif args.mode == "archive":
        load_and_verify_manifest(args.manifest, lock)
        verify_archive(
            args.artifact,
            lock,
            args.manifest,
            args.lock,
            license_path,
            args.extract_directory,
        )
    elif args.mode == "seal":
        seal_archive_receipt(
            args.receipt,
            args.artifact,
            args.manifest,
            args.lock,
            license_path,
            args.extract_directory,
        )
    elif args.mode == "receipt":
        verify_receipt(
            args.receipt,
            args.artifact,
            args.manifest,
            args.lock,
            license_path,
            args.extract_directory,
        )
    elif args.mode == "publish":
        try:
            publish_release_pair(
                args.candidate,
                args.destination,
                args.manifest,
                args.lock,
                license_path,
                args.extract_directory,
            )
        except VerificationError:
            raise
        except Exception as error:
            raise RecoveryRequiredError(
                "unexpected publish failure left publication state unproven "
                f"({type(error).__name__}: {error}); manual recovery is "
                f"required. Preserve both the release candidate "
                f"{args.candidate} and destination {args.destination}"
            ) from error

    print(f"MODEL_LOCK_VERIFY_PASS mode={args.mode}")


def main(arguments: Sequence[str] | None = None) -> int:
    command_arguments = list(sys.argv[1:] if arguments is None else arguments)
    requested_mode = command_arguments[0] if command_arguments else None
    try:
        run(command_arguments)
    except RecoveryRequiredError as error:
        print(f"MODEL_LOCK_RECOVERY_REQUIRED: {error}", file=sys.stderr)
        return 2
    except VerificationError as error:
        print(f"MODEL_LOCK_VERIFY_FAIL: {error}", file=sys.stderr)
        return 1
    except Exception as error:
        if requested_mode == "publish":
            print(
                "MODEL_LOCK_RECOVERY_REQUIRED: unexpected publish failure left "
                "publication state unproven "
                f"({type(error).__name__}: {error})",
                file=sys.stderr,
            )
            return 2
        raise
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
