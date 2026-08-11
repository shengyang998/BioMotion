#!/usr/bin/env python3
"""Fail-closed verifier for the SAM3DBodyPose Core ML supply chain lock."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import sys
from typing import Any, Iterable, Mapping, Sequence
from urllib.parse import urlsplit


LOCK_SCHEMA_VERSION = 1
EXPECTED_ASSET_PACK_ID = "sam3d-body-pose"
EXPECTED_MODEL_BASE_NAME = "SAM3DBodyPose"
MAX_JSON_BYTES = 16 * 1024 * 1024
HASH_CHUNK_BYTES = 4 * 1024 * 1024
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}\Z")
COMMIT_PATTERN = re.compile(r"[0-9a-f]{40}\Z")
IDENTIFIER_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_]*\Z")
ASSET_PACK_PATTERN = re.compile(r"[a-z0-9]+(?:-[a-z0-9]+)*\Z")
IOS_VERSION_PATTERN = re.compile(r"[0-9]+(?:\.[0-9]+)+\Z")


class VerificationError(RuntimeError):
    """An expected supply-chain invariant was not satisfied."""


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
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode):
            _fail(f"{label} changed to a non-regular file: {path}")
        if (opened.st_dev, opened.st_ino) != (before.st_dev, before.st_ino):
            _fail(f"{label} changed while it was being opened: {path}")
        return descriptor, opened
    except BaseException:
        os.close(descriptor)
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

    command = ["xcrun", "coremlcompiler", "metadata", str(source_package)]
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


def _default_lock_path() -> Path:
    repository_root = Path(__file__).resolve().parents[2]
    return repository_root / "BioMotion/Resources/SAM3DBodyPose.lock.json"


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

    if args.mode == "source":
        verify_artifact_tree(args.artifact, lock["sourcePackage"], "source package")
        validate_coreml_metadata(inspect_source_metadata(args.artifact), lock)
    elif args.mode == "compiled":
        verify_artifact_tree(args.artifact, lock["compiledModel"], "compiled model")
        metadata = load_json_file(args.artifact / "metadata.json", "compiled metadata")
        validate_coreml_metadata(metadata, lock)

    print(f"MODEL_LOCK_VERIFY_PASS mode={args.mode}")


def main(arguments: Sequence[str] | None = None) -> int:
    try:
        run(arguments)
    except VerificationError as error:
        print(f"MODEL_LOCK_VERIFY_FAIL: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
