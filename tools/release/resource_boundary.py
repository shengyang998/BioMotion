#!/usr/bin/python3
"""Source, generated-project, and built-product resource boundary for BioMotion."""

from __future__ import annotations

import base64
from collections import Counter
import datetime
import email.utils
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import plistlib
import re
import shutil
import stat
import struct
import subprocess
import sys
import tempfile
from typing import Optional
import zipfile
import zlib


APP_IDENTIFIER = "com.soleil.BioMotion"
APP_EXECUTABLE = "BioMotion"
EXTENSION_IDENTIFIER = "com.soleil.BioMotion.AssetPackDownloader"
EXTENSION_EXECUTABLE = "AssetPackDownloader"
EXTENSION_RELATIVE = "Extensions/AssetPackDownloader.appex"
TEST_IDENTIFIER = "com.soleil.BioMotionTests"
TEST_EXECUTABLE = "BioMotionTests"
TEAM_IDENTIFIER = "N7VVB6PWZS"
APP_GROUP = "group.com.soleilyu.biomotion"
APPLE_ROOT_CA_SHA256 = (
    "b0b1730ecbc7ff4505142c49f1295e6eda6bcaed7e2c68c5be91b5a11001f024"
)
MAX_APP_BYTES = 96 * 1024 * 1024
MAX_ASSET_CAR_BYTES = 1024 * 1024
MAX_IPA_BYTES = 128 * 1024 * 1024
MAX_IPA_UNCOMPRESSED_BYTES = 128 * 1024 * 1024
MAX_IPA_ENTRIES = 256
MIN_SIGNING_VALIDITY = datetime.timedelta(days=30)
TRUSTED_ENV = {"PATH": "/usr/bin:/bin:/usr/sbin:/sbin"}

REVIEWED_APP_CODE_IMAGES = {
    APP_EXECUTABLE,
    f"{APP_EXECUTABLE}.debug.dylib",
    "__preview.dylib",
    f"{EXTENSION_RELATIVE}/{EXTENSION_EXECUTABLE}",
    f"{EXTENSION_RELATIVE}/{EXTENSION_EXECUTABLE}.debug.dylib",
    f"{EXTENSION_RELATIVE}/__preview.dylib",
}
REQUIRED_APP_CODE_IMAGES = {
    APP_EXECUTABLE,
    f"{EXTENSION_RELATIVE}/{EXTENSION_EXECUTABLE}",
}

ZIP_CENTRAL_HEADER = struct.Struct("<4s6H3I5H2I")
ZIP_LOCAL_HEADER = struct.Struct("<4s5H3I2H")
ZIP_CENTRAL_SIGNATURE = b"PK\x01\x02"
ZIP_LOCAL_SIGNATURE = b"PK\x03\x04"
ZIP_DATA_DESCRIPTOR_SIGNATURE = b"PK\x07\x08"
ZIP_UTF8_FLAG = 0x800
ZIP_DATA_DESCRIPTOR_FLAG = 0x8
ZIP_ENCRYPTED_FLAG = 0x1
ZIP_DEFLATE_OPTION_FLAGS = 0x6
ZIP_ALLOWED_FLAGS = (
    ZIP_UTF8_FLAG | ZIP_DATA_DESCRIPTOR_FLAG | ZIP_DEFLATE_OPTION_FLAGS
)
ZIP64_EXTRA_ID = 0x0001
ZIP_ALTERNATE_UNICODE_EXTRA_IDS = {0x6375, 0x7075}
ZIP_REVIEWED_NON_SEMANTIC_EXTRA_IDS = {0x5455, 0x5855, 0x7875}

MODEL_IDENTITIES = {
    "FullBody.osim": (
        3_247_586,
        "0003473937af6883034df358194bd8f52853818e79e36fd23eb5ca2c8d741c09",
    ),
    "Rajagopal2016.osim": (
        874_820,
        "3f5c5f23e486073f2ad2aa4a4967ffe2fcdd582b1e355512bc54f70c36376bf4",
    ),
}

TEXT_IDENTITIES = {
    "LICENSE": (
        1_063,
        "78db6373fbcb9acc572db1b40562cd3bc20ad4241a640df5e20acdc7f928c287",
    ),
    "NOTICE": (
        3_030,
        "06dc022875d7f350ed1f64f72a34c4fc3c152cab96c356644235f55da0dd06fc",
    ),
    "BioMotion/PrivacyInfo.xcprivacy": (
        228,
        "a7081f1506c90e8e673100e5c161e765b8b7a624446e20bd4bd6efaca80808b8",
    ),
    "BioMotion/Resources/THIRD-PARTY-NOTICES.txt": (
        49_974,
        "8143c0931b05e12fb0451b57f262013599aceabf3a2cbe0102d2711fd655e7ab",
    ),
}

ASSET_IDENTITIES = {
    "AppIcon.appiconset/Contents.json": (
        2_271,
        "94ade82a855168514627f5aae77ab2ae1385cfaa4eb45e69b4e54a99b93eae66",
    ),
    "AppIcon.appiconset/icon_1024x1024.png": (
        41_155,
        "15c09add25ed7099dc9ee85a42ef8c8b3def8386c4b91bc72383d5060110a7e0",
    ),
    "AppIcon.appiconset/icon_120x120.png": (
        2_403,
        "4ee842e497fc80bf268fee2eb9213f98b1bca1770577bc6ec0a1e44ebb459ce8",
    ),
    "AppIcon.appiconset/icon_152x152.png": (
        3_680,
        "a44089c33b8046c736744932d824ae82c1f92f2668d7c3c2e96b01b558687836",
    ),
    "AppIcon.appiconset/icon_167x167.png": (
        4_162,
        "a19fa9c621bbbd7b7c2775c07c403d2625eaefbc39ae5e42571db33a6424d3b2",
    ),
    "AppIcon.appiconset/icon_180x180.png": (
        3_600,
        "4f0f0c5e4ed096751d370349a08026214118c9631efb103e89fe5a53307f722e",
    ),
    "AppIcon.appiconset/icon_20x20.png": (
        752,
        "7dd8d6f7feeb4eb48feb3d60aaa65ef291f18fd66c71504a503310ed1e7dda64",
    ),
    "AppIcon.appiconset/icon_29x29.png": (
        990,
        "f44401e63fe9ec1f611eb8b705de880246ec8dc801e6b4b97b671fd76d35dfcd",
    ),
    "AppIcon.appiconset/icon_40x40.png": (
        1_051,
        "2a698bf6660a7b53897730f29b35ac7f16b235432ee4b53024471e126253bbf3",
    ),
    "AppIcon.appiconset/icon_58x58.png": (
        1_442,
        "e010cbb6b631dfa69807bc2e058d11bef904d6446bacc97c2d0d48c8c4ee1630",
    ),
    "AppIcon.appiconset/icon_60x60.png": (
        1_166,
        "9788513ffb3df5f6394303caf516176138e64cf96f0b592443bf14d534482f2a",
    ),
    "AppIcon.appiconset/icon_76x76.png": (
        1_829,
        "9ef77494329aa2ceb543105035bfcb181a5d2831d5f5a0dfa044666fa584045b",
    ),
    "AppIcon.appiconset/icon_80x80.png": (
        1_899,
        "5a4d8fdfe0169375234f8c7b39b78eaf299508875d642072f5df0cda1c166694",
    ),
    "AppIcon.appiconset/icon_87x87.png": (
        1_901,
        "bdb8d678ab853bd898a103863ab3dcd98ed5eebae3b16ba568a6bcb1033b8b48",
    ),
    "Contents.json": (
        63,
        "0fd49ba3c3585c709678e0046a821c3c60685ec7063720d30d3a3448be3a208b",
    ),
}

FIXTURE_NAMES = {
    "gait_video_012.txt",
    "gait_video_013.txt",
    "gait_video_015.txt",
    "mhr_root_coreml.txt",
    "opensim_moment_arms.txt",
    "opensim_moment_arms_fd.txt",
    "opensim_multiwrap.txt",
}


def fail(message: str) -> "NoReturn":
    raise SystemExit(message)


def regular_file(path: Path, label: str) -> None:
    try:
        mode = path.lstat().st_mode
    except FileNotFoundError:
        fail(f"{label} is missing: {path}")
    if not stat.S_ISREG(mode):
        fail(f"{label} must be a regular non-symlink file: {path}")


def regular_directory(path: Path, label: str) -> None:
    try:
        mode = path.lstat().st_mode
    except FileNotFoundError:
        fail(f"{label} is missing: {path}")
    if not stat.S_ISDIR(mode):
        fail(f"{label} must be a regular non-symlink directory: {path}")


def file_identity(path: Path) -> tuple[int, str]:
    data = path.read_bytes()
    return len(data), hashlib.sha256(data).hexdigest()


def require_identity(
    path: Path, expected_size: int, expected_digest: str, label: str
) -> None:
    regular_file(path, label)
    size, digest = file_identity(path)
    if size != expected_size or digest != expected_digest:
        fail(f"{label} identity changed: size={size} sha256={digest}")


def target_lines(spec_lines: list[str], name: str) -> list[str]:
    marker = f"  {name}:"
    try:
        start = spec_lines.index(marker)
    except ValueError:
        fail(f"project.yml has no {name} target")
    end = len(spec_lines)
    for index in range(start + 1, len(spec_lines)):
        if re.fullmatch(r"  [A-Za-z0-9_-]+:", spec_lines[index]):
            end = index
            break
    return spec_lines[start:end]


def sequence_count(lines: list[str], sequence: tuple[str, ...]) -> int:
    width = len(sequence)
    return sum(
        tuple(lines[index : index + width]) == sequence
        for index in range(len(lines) - width + 1)
    )


def decode_pbx_value(value: str) -> str:
    value = value.strip()
    if value.startswith('"'):
        return json.loads(value)
    return value


def pbx_property(body: str, key: str) -> str | None:
    match = re.search(
        rf"(?:^|[;\n])\s*{re.escape(key)} = "
        rf"(?P<value>\"(?:\\.|[^\"])*\"|[^;]+);",
        body,
    )
    return decode_pbx_value(match.group("value")) if match else None


def parse_project_resources(project: str, guard_text: str) -> None:
    generated_signing_contract = {
        '"CODE_SIGN_IDENTITY[sdk=iphoneos*]" = "Apple Distribution";': 2,
        '"PROVISIONING_PROFILE_SPECIFIER[sdk=iphoneos*]" = "BioMotion AppStore AG";': 1,
        '"PROVISIONING_PROFILE_SPECIFIER[sdk=iphoneos*]" = "BioMotion Ext AppStore AG";': 1,
    }
    for fragment, expected_count in generated_signing_contract.items():
        if project.count(fragment) != expected_count:
            fail(f"generated Release signing contract changed: {fragment}")

    file_references: dict[str, tuple[str, str]] = {}
    for match in re.finditer(
        r"^\s*([A-F0-9]{24}) /\* [^*]+ \*/ = "
        r"\{isa = PBXFileReference;(?P<body>[^}]*)\};$",
        project,
        flags=re.MULTILINE,
    ):
        body = match.group("body")
        path = pbx_property(body, "path") or pbx_property(body, "name")
        source_tree = pbx_property(body, "sourceTree")
        if path and source_tree:
            file_references[match.group(1)] = (path, source_tree)

    groups: dict[str, tuple[str | None, str, list[str]]] = {}
    for match in re.finditer(
        r"^\t\t([A-F0-9]{24}) /\* [^*]+ \*/ = \{\n"
        r"\t\t\tisa = PBXGroup;(.*?)^\t\t\};",
        project,
        flags=re.MULTILINE | re.DOTALL,
    ):
        body = match.group(2)
        children_match = re.search(r"\bchildren = \((.*?)\);", body, re.DOTALL)
        children = (
            re.findall(r"\b[A-F0-9]{24}\b", children_match.group(1))
            if children_match
            else []
        )
        groups[match.group(1)] = (
            pbx_property(body, "path"),
            pbx_property(body, "sourceTree") or "<group>",
            children,
        )

    parent: dict[str, str] = {}
    for group_id, (_, _, children) in groups.items():
        for child in children:
            if child in parent:
                fail(f"generated project object has multiple PBXGroup parents: {child}")
            parent[child] = group_id

    def safe_relative(value: str, label: str) -> PurePosixPath:
        relative = PurePosixPath(value)
        if relative.is_absolute() or ".." in relative.parts or not relative.parts:
            fail(f"unsafe generated project path for {label}: {value}")
        return relative

    def resolved_reference(reference_id: str) -> str:
        if reference_id not in file_references:
            fail(f"unresolved PBXFileReference: {reference_id}")
        raw_path, source_tree = file_references[reference_id]
        reference_path = safe_relative(raw_path, reference_id)
        if source_tree == "SOURCE_ROOT":
            return reference_path.as_posix()
        if source_tree != "<group>":
            fail(
                f"resource reference has unsupported sourceTree {source_tree}: "
                f"{reference_id}"
            )
        pieces = [reference_path]
        current = parent.get(reference_id)
        seen = set()
        while current is not None:
            if current in seen:
                fail(f"PBXGroup parent cycle while resolving {reference_id}")
            seen.add(current)
            group_path, group_source_tree, _ = groups[current]
            if group_path:
                pieces.append(safe_relative(group_path, current))
            if group_source_tree == "SOURCE_ROOT":
                break
            if group_source_tree != "<group>":
                fail(
                    f"resource group has unsupported sourceTree "
                    f"{group_source_tree}: {current}"
                )
            current = parent.get(current)
        combined = PurePosixPath()
        for piece in reversed(pieces):
            combined /= piece
        return combined.as_posix()

    build_files: dict[str, str] = {}
    for match in re.finditer(
        r"^\s*([A-F0-9]{24}) /\* [^*]+ \*/ = "
        r"\{isa = PBXBuildFile;(?P<body>[^}]*)\};$",
        project,
        flags=re.MULTILINE,
    ):
        reference = re.search(
            r"\bfileRef = ([A-F0-9]{24})\b", match.group("body")
        )
        if reference:
            build_files[match.group(1)] = reference.group(1)

    resource_phases: dict[str, list[str]] = {}
    for match in re.finditer(
        r"^\t\t([A-F0-9]{24}) /\* [^*]+ \*/ = \{\n"
        r"\t\t\tisa = PBXResourcesBuildPhase;(.*?)^\t\t\};",
        project,
        flags=re.MULTILINE | re.DOTALL,
    ):
        files_match = re.search(
            r"\bfiles = \((.*?)\);", match.group(2), re.DOTALL
        )
        resource_phases[match.group(1)] = (
            re.findall(r"\b[A-F0-9]{24}\b", files_match.group(1))
            if files_match
            else []
        )

    shell_phases: dict[str, str] = {}
    for match in re.finditer(
        r"^\t\t([A-F0-9]{24}) /\* [^*]+ \*/ = \{\n"
        r"\t\t\tisa = PBXShellScriptBuildPhase;(.*?)^\t\t\};",
        project,
        flags=re.MULTILINE | re.DOTALL,
    ):
        shell_phases[match.group(1)] = match.group(2)

    target_phases: dict[str, list[str]] = {}
    for match in re.finditer(
        r"^\t\t([A-F0-9]{24}) /\* [^*]+ \*/ = \{\n"
        r"\t\t\tisa = PBXNativeTarget;(.*?)^\t\t\};",
        project,
        flags=re.MULTILINE | re.DOTALL,
    ):
        body = match.group(2)
        name = pbx_property(body, "name")
        phases_match = re.search(
            r"\bbuildPhases = \((.*?)\);", body, re.DOTALL
        )
        if name and phases_match:
            if name in target_phases:
                fail(f"duplicate native target name: {name}")
            target_phases[name] = re.findall(
                r"\b[A-F0-9]{24}\b", phases_match.group(1)
            )

    expected_target_resources = {
        "BioMotion": Counter(
            {
                "BioMotion/Assets.xcassets": 1,
                "BioMotion/PrivacyInfo.xcprivacy": 1,
                "BioMotion/Resources/FullBody.osim": 1,
                "BioMotion/Resources/Rajagopal2016.osim": 1,
                "BioMotion/Resources/THIRD-PARTY-NOTICES.txt": 1,
                "NOTICE": 1,
            }
        ),
        "BioMotionTests": Counter(
            {
                "BioMotionTests/Fixtures": 1,
                "BioMotion/Resources/FullBody.osim": 1,
                "BioMotion/Resources/Rajagopal2016.osim": 1,
            }
        ),
        "AssetPackDownloader": Counter(),
    }
    for phase_id in resource_phases:
        owners = [
            name for name, phases in target_phases.items() if phase_id in phases
        ]
        if len(owners) != 1:
            fail(
                "resource phase must belong to exactly one native target: "
                f"{phase_id} owners={owners}"
            )
    for target, expected in expected_target_resources.items():
        if target not in target_phases:
            fail(f"generated project has no {target} native target")
        phases = [
            phase for phase in target_phases[target] if phase in resource_phases
        ]
        if expected and len(phases) != 1:
            fail(f"{target} must have exactly one resource phase")
        if not expected and phases:
            fail(f"{target} unexpectedly has a resource phase")
        observed = Counter()
        for phase in phases:
            for build_file in resource_phases[phase]:
                reference = build_files.get(build_file)
                if reference is None:
                    fail(
                        f"unresolved resource build file in {target}: {build_file}"
                    )
                observed[resolved_reference(reference)] += 1
        if observed != expected:
            fail(
                f"generated {target} resource allowlist changed: "
                f"observed={dict(observed)} expected={dict(expected)}"
            )

    expected_guard_names = {
        "Reject developer model before non-Simulator build",
        "Reject bundled model after resources",
    }
    guard_phases: dict[str, str] = {}
    for phase_id, body in shell_phases.items():
        script_value = pbx_property(body, "shellScript")
        if script_value == guard_text:
            name = pbx_property(body, "name")
            if not name:
                fail("generated developer-model guard phase has no name")
            guard_phases[name] = phase_id
            for required_setting in (
                "alwaysOutOfDate = 1;",
                "shellPath = /bin/bash;",
                "showEnvVarsInLog = 0;",
            ):
                if required_setting not in body:
                    fail(
                        "generated developer-model guard setting changed: "
                        f"{required_setting}"
                    )
    if set(guard_phases) != expected_guard_names:
        fail(
            "generated developer-model guard phase inventory changed: "
            f"{sorted(guard_phases)}"
        )
    app_phases = target_phases["BioMotion"]
    pre_guard = guard_phases["Reject developer model before non-Simulator build"]
    post_guard = guard_phases["Reject bundled model after resources"]
    if app_phases[0] != pre_guard or app_phases[-1] != post_guard:
        fail("developer-model guards must be BioMotion's first and last phases")
    for guard_phase in guard_phases.values():
        wrong_owners = [
            name
            for name, phases in target_phases.items()
            if name != "BioMotion" and guard_phase in phases
        ]
        if wrong_owners:
            fail(
                "developer-model guard belongs to the wrong target: "
                + ", ".join(wrong_owners)
            )


def inspect_source_and_project(repo: Path) -> None:
    project_spec = repo / "project.yml"
    project_file = repo / "BioMotion.xcodeproj/project.pbxproj"
    export_options = repo / "tools/release/ExportOptions-TestFlight.plist"
    resource_root = repo / "BioMotion/Resources"
    asset_root = repo / "BioMotion/Assets.xcassets"
    fixtures_root = repo / "BioMotionTests/Fixtures"
    guard = repo / "tools/release/reject_dev_model.sh"
    for path, label in (
        (project_spec, "project spec"),
        (project_file, "generated project"),
        (export_options, "TestFlight export options"),
        (guard, "developer-model guard"),
        (resource_root / "SAM-LICENSE.txt", "SAM license authority"),
        (
            resource_root / "SAM3DBodyPose.lock.json",
            "SAM model lock authority",
        ),
    ):
        regular_file(path, label)

    for name, (size, digest) in MODEL_IDENTITIES.items():
        require_identity(resource_root / name, size, digest, f"source model {name}")
    observed_models = sorted(
        path.relative_to(resource_root).as_posix()
        for path in resource_root.rglob("*")
        if path.name.lower().endswith(".osim")
    )
    if observed_models != sorted(MODEL_IDENTITIES):
        fail(f"source model allowlist changed: {observed_models}")

    for relative, (size, digest) in TEXT_IDENTITIES.items():
        require_identity(repo / relative, size, digest, f"source text {relative}")
    consolidated = (resource_root / "THIRD-PARTY-NOTICES.txt").read_bytes()
    for authority in (repo / "LICENSE", repo / "NOTICE"):
        if authority.read_bytes() not in consolidated:
            fail(f"consolidated legal resource no longer embeds {authority.name}")

    regular_directory(asset_root, "asset catalog")
    observed_asset_files = set()
    observed_asset_directories = set()
    for path in asset_root.rglob("*"):
        mode = path.lstat().st_mode
        relative = path.relative_to(asset_root).as_posix()
        if stat.S_ISDIR(mode):
            observed_asset_directories.add(relative)
        elif stat.S_ISREG(mode):
            observed_asset_files.add(relative)
        else:
            fail(f"asset catalog contains a symlink or special entry: {relative}")
    if observed_asset_directories != {"AppIcon.appiconset"}:
        fail(
            "asset catalog directory allowlist changed: "
            f"{sorted(observed_asset_directories)}"
        )
    if observed_asset_files != set(ASSET_IDENTITIES):
        fail(
            "asset catalog file allowlist changed: "
            f"{sorted(observed_asset_files)}"
        )
    for relative, (size, digest) in ASSET_IDENTITIES.items():
        require_identity(
            asset_root / relative, size, digest, f"asset catalog entry {relative}"
        )

    regular_directory(fixtures_root, "test fixture directory")
    fixture_entries = set()
    for path in fixtures_root.iterdir():
        regular_file(path, f"test fixture {path.name}")
        fixture_entries.add(path.name)
    if fixture_entries != FIXTURE_NAMES:
        fail(f"test fixture allowlist changed: {sorted(fixture_entries)}")

    spec_lines = project_spec.read_text(encoding="utf-8").splitlines()
    app_spec = target_lines(spec_lines, "BioMotion")
    extension_spec = target_lines(spec_lines, "AssetPackDownloader")
    tests_spec = target_lines(spec_lines, "BioMotionTests")

    def target_version(lines: list[str], key: str, target: str) -> str:
        values = [
            match.group(1)
            for line in lines
            if (
                match := re.fullmatch(
                    rf"        {re.escape(key)}: \"([^\"]+)\"", line
                )
            )
        ]
        if len(values) != 1:
            fail(f"project.yml {target} must define exactly one {key}")
        return values[0]

    app_marketing = target_version(
        app_spec, "MARKETING_VERSION", "BioMotion"
    )
    app_build = target_version(
        app_spec, "CURRENT_PROJECT_VERSION", "BioMotion"
    )
    extension_marketing = target_version(
        extension_spec, "MARKETING_VERSION", "AssetPackDownloader"
    )
    extension_build = target_version(
        extension_spec, "CURRENT_PROJECT_VERSION", "AssetPackDownloader"
    )
    if app_marketing != extension_marketing or app_build != extension_build:
        fail("project.yml app and extension versions must match")
    if not re.fullmatch(r"[0-9]+(?:\.[0-9]+){2}", app_marketing):
        fail(f"project.yml marketing version is malformed: {app_marketing}")
    if not re.fullmatch(r"[1-9][0-9]*", app_build):
        fail(f"project.yml build number is malformed: {app_build}")

    generated_project = project_file.read_text(encoding="utf-8")
    for key, expected in (
        ("MARKETING_VERSION", app_marketing),
        ("CURRENT_PROJECT_VERSION", app_build),
    ):
        generated_values = re.findall(
            rf"^\s+{re.escape(key)} = ([^;]+);$",
            generated_project,
            flags=re.MULTILINE,
        )
        if generated_values != [expected] * 4:
            fail(
                "generated project version is stale; bump project.yml before "
                f"xcodegen generate: {key}={generated_values} expected={expected}"
            )

    expected_export_options = {
        "destination": "export",
        "distributionBundleIdentifier": APP_IDENTIFIER,
        "manageAppVersionAndBuildNumber": False,
        "method": "app-store-connect",
        "provisioningProfiles": {
            APP_IDENTIFIER: "BioMotion AppStore AG",
            EXTENSION_IDENTIFIER: "BioMotion Ext AppStore AG",
        },
        "signingCertificate": "Apple Distribution",
        "signingStyle": "manual",
        "teamID": TEAM_IDENTIFIER,
        "uploadSymbols": True,
    }
    observed_export_options = read_plist(
        export_options, "TestFlight export options"
    )
    if observed_export_options != expected_export_options:
        fail(
            "TestFlight export options changed: "
            f"observed={observed_export_options}"
        )
    try:
        source_start = app_spec.index("      - path: BioMotion")
        excludes_start = app_spec.index("        excludes:", source_start)
    except ValueError:
        fail("BioMotion source exclusion block is missing")
    excludes = set()
    for line in app_spec[excludes_start + 1 :]:
        if line.startswith("      - path:"):
            break
        if line.startswith("          - "):
            excludes.add(line.removeprefix("          - ").strip().strip('"'))
            continue
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if len(line) - len(line.lstrip()) <= 8:
            break
    required_excludes = {
        "Resources",
        "Resources/**",
        "**/*.md",
        "PrivacyInfo.xcprivacy",
    }
    if not required_excludes.issubset(excludes):
        fail(
            "BioMotion broad source scan exclusion contract changed: missing "
            f"{sorted(required_excludes - excludes)}"
        )

    app_resources = (
        "BioMotion/PrivacyInfo.xcprivacy",
        "BioMotion/Resources/FullBody.osim",
        "BioMotion/Resources/Rajagopal2016.osim",
        "BioMotion/Resources/THIRD-PARTY-NOTICES.txt",
        "NOTICE",
    )
    for relative in app_resources:
        pair = (
            f"      - path: {relative}",
            "        buildPhase: resources",
        )
        app_count = sequence_count(app_spec, pair)
        global_count = sequence_count(spec_lines, pair)
        expected_global = 2 if relative.endswith(".osim") else 1
        if app_count != 1 or global_count != expected_global:
            fail(
                f"project.yml app resource membership changed for {relative}: "
                f"app={app_count} global={global_count}"
            )
    for relative in (
        "BioMotion/Resources/FullBody.osim",
        "BioMotion/Resources/Rajagopal2016.osim",
    ):
        if sequence_count(
            tests_spec,
            (
                f"      - path: {relative}",
                "        buildPhase: resources",
            ),
        ) != 1:
            fail(f"project.yml test resource membership changed for {relative}")
    if any(
        line.strip() == "- path: BioMotion/Resources" for line in tests_spec
    ):
        fail("BioMotionTests must not copy the whole Resources folder")
    if sequence_count(
        app_spec,
        ("      - path: build/DevBundledModel", "        optional: true"),
    ) != 1:
        fail("developer model must remain one optional BioMotion source")

    guard_fields = (
        "        shell: /bin/bash",
        "        showEnvVars: false",
        "        basedOnDependencyAnalysis: false",
    )
    pre_guard = (
        "      - path: tools/release/reject_dev_model.sh",
        "        name: Reject developer model before non-Simulator build",
        *guard_fields,
    )
    post_guard = (
        "      - path: tools/release/reject_dev_model.sh",
        "        name: Reject bundled model after resources",
        *guard_fields,
    )
    if sequence_count(app_spec, pre_guard) != 1 or sequence_count(
        app_spec, post_guard
    ) != 1:
        fail("project.yml developer-model guard wiring changed")

    signing_profiles = (
        (app_spec, "BioMotion AppStore AG"),
        (extension_spec, "BioMotion Ext AppStore AG"),
    )
    for target_spec, profile in signing_profiles:
        required_lines = {
            "          CODE_SIGN_STYLE: Manual",
            '          "CODE_SIGN_IDENTITY[sdk=iphoneos*]": "Apple Distribution"',
            f'          "PROVISIONING_PROFILE_SPECIFIER[sdk=iphoneos*]": "{profile}"',
        }
        missing_signing = sorted(required_lines - set(target_spec))
        if missing_signing:
            fail(
                f"project.yml Release signing contract changed for {profile}: "
                f"missing={missing_signing}"
            )

    parse_project_resources(
        generated_project,
        guard.read_text(encoding="utf-8"),
    )


def read_plist(path: Path, label: str) -> dict:
    regular_file(path, label)
    try:
        with path.open("rb") as stream:
            value = plistlib.load(stream)
    except (OSError, plistlib.InvalidFileException) as error:
        fail(f"{label} is not a valid plist: {error}")
    if not isinstance(value, dict):
        fail(f"{label} root must be a dictionary")
    return value


def collect_inventory(root: Path) -> tuple[set[str], set[str], int]:
    files = set()
    directories = set()
    total_bytes = 0
    for current, dir_names, file_names in os.walk(root, followlinks=False):
        dir_names.sort()
        file_names.sort()
        current_path = Path(current)
        for name in dir_names:
            path = current_path / name
            mode = path.lstat().st_mode
            relative = path.relative_to(root).as_posix()
            if not stat.S_ISDIR(mode):
                fail(f"built bundle contains a symlink or special directory: {relative}")
            directories.add(relative)
        for name in file_names:
            path = current_path / name
            mode = path.lstat().st_mode
            relative = path.relative_to(root).as_posix()
            if not stat.S_ISREG(mode):
                fail(f"built bundle contains a symlink or special file: {relative}")
            files.add(relative)
            total_bytes += path.stat().st_size
    return files, directories, total_bytes


def require_macho(
    path: Path,
    label: str,
    maximum: int,
    platform: str,
    file_kind: str,
) -> None:
    regular_file(path, label)
    size = path.stat().st_size
    if size <= 0 or size > maximum:
        fail(f"{label} size is outside the reviewed budget: {size}")
    description = subprocess.run(
        ["/usr/bin/file", "-b", str(path)],
        check=False,
        capture_output=True,
        text=True,
        env=TRUSTED_ENV,
    )
    if (
        description.returncode != 0
        or "Mach-O" not in description.stdout
        or file_kind.lower() not in description.stdout.lower()
    ):
        fail(f"{label} is not Mach-O: {path}")
    architectures = subprocess.run(
        ["/usr/bin/lipo", "-archs", str(path)],
        check=False,
        capture_output=True,
        text=True,
        env=TRUSTED_ENV,
    )
    if architectures.returncode != 0 or architectures.stdout.strip() != "arm64":
        fail(
            f"{label} architecture inventory changed: "
            f"{architectures.stdout.strip()}"
        )
    build = subprocess.run(
        ["/usr/bin/xcrun", "vtool", "-show-build", str(path)],
        check=False,
        capture_output=True,
        text=True,
        env=TRUSTED_ENV,
    )
    if build.returncode != 0:
        fail(f"{label} has no readable Mach-O build receipt: {build.stderr.strip()}")
    expected_platform = "IOSSIMULATOR" if platform == "iphonesimulator" else "IOS"
    platforms = re.findall(r"^\s*platform\s+(\S+)\s*$", build.stdout, re.MULTILINE)
    minimum_versions = re.findall(r"^\s*minos\s+(\S+)\s*$", build.stdout, re.MULTILINE)
    if platforms != [expected_platform]:
        fail(f"{label} has the wrong Mach-O platform: {platforms}")
    if minimum_versions != ["26.0"]:
        fail(f"{label} has the wrong Mach-O minimum OS: {minimum_versions}")


def decode_signed_profile(path: Path, label: str) -> dict:
    regular_file(path, label)
    with tempfile.TemporaryDirectory(prefix="biomotion-profile-cms.") as temp:
        temp_root = Path(temp)
        preliminary_payload = temp_root / "preliminary-payload.plist"
        certificates = temp_root / "certificates.pem"
        preliminary_signer = temp_root / "preliminary-signer.pem"
        preliminary = subprocess.run(
            [
                "/usr/bin/openssl",
                "cms",
                "-verify",
                "-inform",
                "DER",
                "-in",
                str(path),
                "-noverify",
                "-out",
                str(preliminary_payload),
                "-signer",
                str(preliminary_signer),
                "-certsout",
                str(certificates),
            ],
            check=False,
            capture_output=True,
            text=True,
            env=TRUSTED_ENV,
        )
        if preliminary.returncode != 0:
            fail(f"{label} CMS content signature is invalid")
        for generated, generated_label in (
            (preliminary_payload, "verified CMS payload"),
            (preliminary_signer, "CMS signer"),
            (certificates, "CMS certificate inventory"),
        ):
            regular_file(generated, f"{label} {generated_label}")

        certificate_blocks = re.findall(
            rb"-----BEGIN CERTIFICATE-----\s+"
            rb"([A-Za-z0-9+/=\r\n]+?)"
            rb"-----END CERTIFICATE-----",
            certificates.read_bytes(),
        )
        if not (1 <= len(certificate_blocks) <= 6):
            fail(f"{label} CMS certificate inventory changed")
        for encoded in certificate_blocks:
            try:
                base64.b64decode(re.sub(rb"\s+", b"", encoded), validate=True)
            except ValueError:
                fail(f"{label} CMS contains a malformed certificate")

        system_roots = subprocess.run(
            [
                "/usr/bin/security",
                "find-certificate",
                "-a",
                "-p",
                "/System/Library/Keychains/SystemRootCertificates.keychain",
            ],
            check=False,
            capture_output=True,
            env=TRUSTED_ENV,
        )
        if system_roots.returncode != 0:
            fail("system Apple Root CA inventory is unavailable")
        root_blocks = []
        system_root_blocks = re.findall(
            rb"-----BEGIN CERTIFICATE-----\s+"
            rb"([A-Za-z0-9+/=\r\n]+?)"
            rb"-----END CERTIFICATE-----",
            system_roots.stdout,
        )
        for encoded in system_root_blocks:
            try:
                der = base64.b64decode(re.sub(rb"\s+", b"", encoded), validate=True)
            except ValueError:
                fail("system root certificate inventory is malformed")
            if hashlib.sha256(der).hexdigest() == APPLE_ROOT_CA_SHA256:
                root_blocks.append(encoded)
        if len(root_blocks) != 1:
            fail("the reviewed Apple Root CA is missing from the system trust store")
        apple_root = temp_root / "apple-root.pem"
        apple_root.write_bytes(
            b"-----BEGIN CERTIFICATE-----\n"
            + root_blocks[0].replace(b"\r", b"").strip()
            + b"\n-----END CERTIFICATE-----\n"
        )

        verified_payload = temp_root / "verified-payload.plist"
        verified_signer = temp_root / "verified-signer.pem"
        verified_certificates = temp_root / "verified-certificates.pem"
        verification = subprocess.run(
            [
                "/usr/bin/openssl",
                "cms",
                "-verify",
                "-inform",
                "DER",
                "-in",
                str(path),
                "-CAfile",
                str(apple_root),
                "-purpose",
                "any",
                "-out",
                str(verified_payload),
                "-signer",
                str(verified_signer),
                "-certsout",
                str(verified_certificates),
            ],
            check=False,
            capture_output=True,
            text=True,
            env=TRUSTED_ENV,
        )
        if verification.returncode != 0:
            fail(f"{label} CMS signer chain is not trusted")
        regular_file(verified_payload, f"{label} verified CMS payload")
        regular_file(verified_signer, f"{label} verified CMS signer")
        signer_blocks = re.findall(
            rb"-----BEGIN CERTIFICATE-----.*?-----END CERTIFICATE-----",
            verified_signer.read_bytes(),
            flags=re.DOTALL,
        )
        if len(signer_blocks) != 1:
            fail(f"{label} must have exactly one CMS signer")

        signer_identity = subprocess.run(
            [
                "/usr/bin/openssl",
                "x509",
                "-in",
                str(verified_signer),
                "-noout",
                "-subject",
                "-issuer",
                "-nameopt",
                "RFC2253",
            ],
            check=False,
            capture_output=True,
            text=True,
            env=TRUSTED_ENV,
        )
        expected_identity = {
            "subject= C=US,O=Apple Inc.,CN=Apple iPhone OS Provisioning Profile Signing",
            "issuer= C=US,O=Apple Inc.,OU=Certification Authority,CN=Apple iPhone Certification Authority",
        }
        if signer_identity.returncode != 0 or set(
            signer_identity.stdout.splitlines()
        ) != expected_identity:
            fail(f"{label} has an unexpected CMS signer identity")
        signer_text = subprocess.run(
            [
                "/usr/bin/openssl",
                "x509",
                "-in",
                str(verified_signer),
                "-noout",
                "-text",
            ],
            check=False,
            capture_output=True,
            text=True,
            env=TRUSTED_ENV,
        )
        if signer_text.returncode != 0:
            fail(f"{label} CMS signer certificate is unreadable")
        signer_contract = (
            r"X509v3 Basic Constraints: critical\s+CA:FALSE",
            r"X509v3 Key Usage: critical\s+Digital Signature",
            r"1\.2\.840\.113635\.100\.6\.58:",
        )
        if any(
            re.search(pattern, signer_text.stdout) is None
            for pattern in signer_contract
        ) or "X509v3 Extended Key Usage" in signer_text.stdout:
            fail(f"{label} CMS signer certificate contract changed")

        payload = verified_payload.read_bytes()
    try:
        profile = plistlib.loads(payload)
    except (ValueError, plistlib.InvalidFileException) as error:
        fail(f"{label} payload is not a plist: {error}")
    if not isinstance(profile, dict):
        fail(f"{label} payload root must be a dictionary")
    return profile


def signed_entitlements(path: Path, label: str) -> dict:
    result = subprocess.run(
        [
            "/usr/bin/codesign",
            "--display",
            "--entitlements",
            "-",
            "--xml",
            str(path),
        ],
        check=False,
        capture_output=True,
        env=TRUSTED_ENV,
    )
    if result.returncode != 0 or not result.stdout:
        fail(f"{label} has no readable signed entitlements")
    try:
        entitlements = plistlib.loads(result.stdout)
    except (ValueError, plistlib.InvalidFileException) as error:
        fail(f"{label} signed entitlements are not a plist: {error}")
    if not isinstance(entitlements, dict):
        fail(f"{label} signed entitlements root must be a dictionary")
    return entitlements


def validate_distribution_signature(
    bundle: Path,
    bundle_identifier: str,
    label: str,
    profile_path: Path,
    archive_signing_identity: Optional[str] = None,
) -> None:
    requirement = (
        f'identifier "{bundle_identifier}" and anchor apple generic and '
        f'certificate leaf[subject.OU] = "{TEAM_IDENTIFIER}" and '
        "certificate leaf[field.1.2.840.113635.100.6.1.4] exists"
    )
    verification = subprocess.run(
        [
            "/usr/bin/codesign",
            "--verify",
            "--strict",
            f"-R={requirement}",
            str(bundle),
        ],
        check=False,
        capture_output=True,
        text=True,
        env=TRUSTED_ENV,
    )
    if verification.returncode != 0:
        fail(f"{label} is not Apple Distribution signed: {verification.stderr.strip()}")

    details = subprocess.run(
        ["/usr/bin/codesign", "--display", "--verbose=4", str(bundle)],
        check=False,
        capture_output=True,
        text=True,
        env=TRUSTED_ENV,
    )
    if details.returncode != 0:
        fail(f"{label} signing details are unreadable")
    authorities = re.findall(r"^Authority=(.+)$", details.stderr, re.MULTILINE)
    teams = re.findall(r"^TeamIdentifier=(.+)$", details.stderr, re.MULTILINE)
    if not authorities or not authorities[0].startswith(
        ("Apple Distribution:", "iPhone Distribution:")
    ):
        fail(f"{label} has an unexpected leaf signing authority: {authorities[:1]}")
    if teams != [TEAM_IDENTIFIER]:
        fail(f"{label} signing team changed: {teams}")
    if archive_signing_identity is not None and authorities[0] != archive_signing_identity:
        fail(
            "archive SigningIdentity does not match the app leaf certificate: "
            f"{archive_signing_identity!r} != {authorities[0]!r}"
        )

    profile = decode_signed_profile(profile_path, f"{label} provisioning profile")
    if profile.get("TeamIdentifier") != [TEAM_IDENTIFIER]:
        fail(f"{label} provisioning-profile team changed")
    if profile.get("ApplicationIdentifierPrefix") != [TEAM_IDENTIFIER]:
        fail(f"{label} provisioning-profile application prefix changed")
    platforms = profile.get("Platform")
    if not isinstance(platforms, list) or "iOS" not in platforms:
        fail(f"{label} provisioning profile is not for iOS")
    if "ProvisionedDevices" in profile or profile.get("ProvisionsAllDevices") is True:
        fail(f"{label} provisioning profile is not an App Store profile")
    now = datetime.datetime.now(datetime.timezone.utc).replace(tzinfo=None)
    creation = profile.get("CreationDate")
    expiration = profile.get("ExpirationDate")
    if not isinstance(creation, datetime.datetime) or creation > now:
        fail(f"{label} provisioning profile has an invalid creation date")
    if not isinstance(expiration, datetime.datetime) or expiration <= now:
        fail(f"{label} provisioning profile is expired")
    if expiration < now + MIN_SIGNING_VALIDITY:
        fail(f"{label} provisioning profile has less than 30 days of validity")
    if not isinstance(profile.get("Name"), str) or not profile["Name"].strip():
        fail(f"{label} provisioning profile has no name")
    if not isinstance(profile.get("UUID"), str) or not re.fullmatch(
        r"[0-9A-Fa-f-]{36}", profile["UUID"]
    ):
        fail(f"{label} provisioning profile has no valid UUID")

    profile_entitlements = profile.get("Entitlements")
    if not isinstance(profile_entitlements, dict):
        fail(f"{label} provisioning profile has no entitlements")
    expected_application = f"{TEAM_IDENTIFIER}.{bundle_identifier}"
    critical = {
        "application-identifier": expected_application,
        "beta-reports-active": True,
        "com.apple.developer.team-identifier": TEAM_IDENTIFIER,
        "get-task-allow": False,
    }
    for key, value in critical.items():
        if profile_entitlements.get(key) != value:
            fail(f"{label} provisioning-profile entitlement changed: {key}")
    profile_groups = profile_entitlements.get("com.apple.security.application-groups")
    if not isinstance(profile_groups, list) or APP_GROUP not in profile_groups:
        fail(f"{label} provisioning profile does not authorize the reviewed app group")

    entitlements = signed_entitlements(bundle, label)
    allowed_entitlement_keys = {
        "application-identifier",
        "beta-reports-active",
        "com.apple.developer.team-identifier",
        "com.apple.security.application-groups",
        "get-task-allow",
    }
    unexpected_keys = sorted(set(entitlements) - allowed_entitlement_keys)
    if unexpected_keys:
        fail(f"{label} has unreviewed signed entitlements: {unexpected_keys}")
    for key, value in critical.items():
        if entitlements.get(key) != value:
            fail(f"{label} signed entitlement changed: {key}")
    if entitlements.get("com.apple.security.application-groups") != [APP_GROUP]:
        fail(f"{label} signed app-group inventory changed")

    developer_certificates = profile.get("DeveloperCertificates")
    if not isinstance(developer_certificates, list) or not all(
        isinstance(certificate, bytes) for certificate in developer_certificates
    ):
        fail(f"{label} provisioning profile has no developer certificates")
    with tempfile.TemporaryDirectory(prefix="biomotion-signing-cert.") as temp:
        prefix = Path(temp) / "certificate"
        extraction = subprocess.run(
            [
                "/usr/bin/codesign",
                "--display",
                f"--extract-certificates={prefix}",
                str(bundle),
            ],
            check=False,
            capture_output=True,
            text=True,
            env=TRUSTED_ENV,
        )
        leaf = Path(f"{prefix}0")
        if extraction.returncode != 0:
            fail(f"{label} signing certificate could not be extracted")
        regular_file(leaf, f"{label} leaf signing certificate")
        if leaf.read_bytes() not in developer_certificates:
            fail(f"{label} signing certificate is not authorized by its profile")
        certificate_dates = subprocess.run(
            [
                "/usr/bin/openssl",
                "x509",
                "-inform",
                "DER",
                "-in",
                str(leaf),
                "-noout",
                "-dates",
            ],
            check=False,
            capture_output=True,
            text=True,
            env=TRUSTED_ENV,
        )
        date_values = dict(
            line.split("=", 1)
            for line in certificate_dates.stdout.splitlines()
            if "=" in line
        )
        try:
            not_before = email.utils.parsedate_to_datetime(
                date_values["notBefore"]
            ).astimezone(datetime.timezone.utc)
            not_after = email.utils.parsedate_to_datetime(
                date_values["notAfter"]
            ).astimezone(datetime.timezone.utc)
        except (KeyError, TypeError, ValueError, OverflowError):
            fail(f"{label} signing certificate validity is unreadable")
        aware_now = datetime.datetime.now(datetime.timezone.utc)
        if not_before > aware_now or not_after <= aware_now:
            fail(f"{label} signing certificate is outside its validity period")
        if not_after < aware_now + MIN_SIGNING_VALIDITY:
            fail(f"{label} signing certificate has less than 30 days of validity")
        profile_expiration = expiration.replace(tzinfo=datetime.timezone.utc)
        if profile_expiration > not_after:
            fail(f"{label} provisioning profile outlives its signing certificate")


def icon_bases(info: dict) -> set[str]:
    bases = set()
    for key in ("CFBundleIcons", "CFBundleIcons~ipad"):
        icon_root = info.get(key)
        if not isinstance(icon_root, dict):
            continue
        primary = icon_root.get("CFBundlePrimaryIcon")
        if not isinstance(primary, dict):
            continue
        values = primary.get("CFBundleIconFiles")
        if isinstance(values, list):
            for value in values:
                if isinstance(value, str) and re.fullmatch(
                    r"[A-Za-z0-9._-]+", value
                ):
                    bases.add(value)
    return bases


def validate_png(path: Path, label: str) -> None:
    regular_file(path, label)
    size = path.stat().st_size
    if size <= 24 or size > 1024 * 1024:
        fail(f"{label} size is outside the reviewed budget: {size}")
    prefix = path.read_bytes()[:24]
    if prefix[:8] != b"\x89PNG\r\n\x1a\n" or prefix[12:16] != b"IHDR":
        fail(f"{label} is not a PNG with an IHDR")
    width = int.from_bytes(prefix[16:20], "big")
    height = int.from_bytes(prefix[20:24], "big")
    if not (1 <= width <= 2048 and 1 <= height <= 2048):
        fail(f"{label} dimensions are outside the reviewed budget: {width}x{height}")


def validate_asset_car(path: Path) -> None:
    regular_file(path, "compiled asset catalog")
    size = path.stat().st_size
    if size <= 0 or size > MAX_ASSET_CAR_BYTES:
        fail(f"compiled asset catalog exceeds the reviewed size budget: {size}")
    result = subprocess.run(
        ["/usr/bin/xcrun", "assetutil", "--info", str(path)],
        check=False,
        capture_output=True,
        text=True,
        env={"PATH": "/usr/bin:/bin:/usr/sbin:/sbin"},
    )
    if result.returncode != 0:
        fail(f"assetutil rejected the compiled asset catalog: {result.stderr.strip()}")
    try:
        records = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        fail(f"assetutil produced invalid JSON: {error}")
    if not isinstance(records, list) or not (2 <= len(records) <= 64):
        fail("compiled asset catalog record count is outside the reviewed boundary")
    allowed_types = {None, "Icon Image", "PackedImage", "MultiSized Image"}
    for record in records:
        if not isinstance(record, dict):
            fail("compiled asset catalog contains a non-dictionary record")
        asset_type = record.get("AssetType")
        if asset_type not in allowed_types:
            fail(f"compiled asset catalog contains an unreviewed type: {asset_type}")
        name = record.get("Name", "")
        if name and name != "AppIcon" and not re.fullmatch(
            r"ZZZZPackedAsset-[123]\.1\.0-gamut0", name
        ):
            fail(f"compiled asset catalog contains an unreviewed name: {name}")


def validate_platform_info(info: dict, platform: str, label: str) -> None:
    expected_supported = "iPhoneSimulator" if platform == "iphonesimulator" else "iPhoneOS"
    if info.get("DTPlatformName") != platform:
        fail(f"{label} has the wrong platform: {info.get('DTPlatformName')}")
    sdk_name = info.get("DTSDKName")
    if not isinstance(sdk_name, str) or not sdk_name.startswith(platform):
        fail(f"{label} has the wrong SDK: {sdk_name}")
    if info.get("CFBundleSupportedPlatforms") != [expected_supported]:
        fail(f"{label} has the wrong supported-platform inventory")
    if info.get("MinimumOSVersion") != "26.0":
        fail(f"{label} has the wrong minimum OS version")


def resolve_release_archive(archive: Path) -> tuple[Path, dict]:
    if archive.suffix != ".xcarchive":
        fail("release artifact must have the .xcarchive suffix")
    regular_directory(archive, "release archive")
    archive_info = read_plist(archive / "Info.plist", "archive Info.plist")
    properties = archive_info.get("ApplicationProperties")
    if not isinstance(properties, dict):
        fail("archive has no ApplicationProperties dictionary")
    expected = {
        "ArchiveVersion": 2,
        "Name": "BioMotion",
        "SchemeName": "BioMotion",
    }
    for key, value in expected.items():
        if archive_info.get(key) != value:
            fail(f"archive {key} changed: {archive_info.get(key)}")
    if properties.get("ApplicationPath") != "Applications/BioMotion.app":
        fail("archive ApplicationPath must be Applications/BioMotion.app")
    if properties.get("CFBundleIdentifier") != APP_IDENTIFIER:
        fail("archive application identifier changed")
    if properties.get("Architectures") != ["arm64"]:
        fail(f"archive architecture inventory changed: {properties.get('Architectures')}")
    if properties.get("Team") != TEAM_IDENTIFIER:
        fail(f"archive team changed: {properties.get('Team')}")
    signing_identity = properties.get("SigningIdentity")
    if not isinstance(signing_identity, str) or not signing_identity.strip():
        fail("archive has no signing identity")
    app = archive / "Products/Applications/BioMotion.app"
    current = archive / "Products"
    for component in ("Applications", "BioMotion.app"):
        current /= component
        regular_directory(current, f"archive path component {component}")
    return app, properties


def validate_zip_extra(extra: bytes, label: str) -> None:
    """Accept only metadata that cannot change an extracted path, type, or bytes."""
    offset = 0
    while offset < len(extra):
        if len(extra) - offset < 4:
            fail(f"{label} has a truncated ZIP extra field")
        field_id, field_size = struct.unpack_from("<HH", extra, offset)
        offset += 4
        if field_size > len(extra) - offset:
            fail(f"{label} has a truncated ZIP extra payload")
        if field_id == ZIP64_EXTRA_ID:
            fail(f"{label} uses unreviewed ZIP64 metadata")
        if field_id in ZIP_ALTERNATE_UNICODE_EXTRA_IDS:
            fail(f"{label} uses an alternate Unicode ZIP name or comment")
        if field_id not in ZIP_REVIEWED_NON_SEMANTIC_EXTRA_IDS:
            fail(f"{label} uses an unreviewed ZIP extra field: 0x{field_id:04x}")
        offset += field_size


def decode_zip_name(raw_name: bytes, flags: int, label: str) -> str:
    encoding = "utf-8" if flags & ZIP_UTF8_FLAG else "cp437"
    try:
        return raw_name.decode(encoding)
    except UnicodeDecodeError:
        fail(f"{label} has an invalid {encoding} filename")


def read_exact(stream, size: int, label: str) -> bytes:
    data = stream.read(size)
    if len(data) != size:
        fail(f"{label} is truncated")
    return data


def parse_zip_central_entries(
    ipa: Path,
    archive: zipfile.ZipFile,
    entries: list[zipfile.ZipInfo],
) -> list[dict]:
    """Read central records independently and bind each ZipInfo to raw bytes."""
    parsed = []
    try:
        with ipa.open("rb") as stream:
            stream.seek(archive.start_dir)
            for index, entry in enumerate(entries):
                label = f"release IPA central entry {index}"
                header = read_exact(stream, ZIP_CENTRAL_HEADER.size, label)
                (
                    signature,
                    version_made,
                    version_needed,
                    flags,
                    compression,
                    modified_time,
                    modified_date,
                    crc32,
                    compressed_size,
                    uncompressed_size,
                    name_size,
                    extra_size,
                    comment_size,
                    disk_start,
                    internal_attributes,
                    external_attributes,
                    local_offset,
                ) = ZIP_CENTRAL_HEADER.unpack(header)
                if signature != ZIP_CENTRAL_SIGNATURE:
                    fail(f"{label} has the wrong signature")
                raw_name = read_exact(stream, name_size, f"{label} filename")
                extra = read_exact(stream, extra_size, f"{label} extra field")
                comment = read_exact(stream, comment_size, f"{label} comment")
                validate_zip_extra(extra, label)
                if comment:
                    fail(f"{label} contains an unreviewed ZIP entry comment")
                decoded_name = decode_zip_name(raw_name, flags, label)
                if "\x00" in decoded_name:
                    fail(
                        "release IPA contains an unsafe ZIP entry: "
                        f"{decoded_name!r}"
                    )
                if disk_start != 0:
                    fail(f"{label} refers to an unreviewed split archive")
                if flags & ~ZIP_ALLOWED_FLAGS:
                    fail(f"{label} uses unreviewed general-purpose flags")
                if (
                    compression != zipfile.ZIP_DEFLATED
                    and flags & ZIP_DEFLATE_OPTION_FLAGS
                ):
                    fail(f"{label} applies DEFLATE flags to another method")
                if 0xFFFFFFFF in (
                    compressed_size,
                    uncompressed_size,
                    local_offset,
                ):
                    fail(f"{label} uses unreviewed ZIP64 sentinels")
                if (version_made >> 8) != 3:
                    fail(f"{label} does not carry Unix permission metadata")
                if (
                    entry.orig_filename != decoded_name
                    or entry.filename != decoded_name
                    or entry.flag_bits != flags
                    or entry.compress_type != compression
                    or entry.CRC != crc32
                    or entry.compress_size != compressed_size
                    or entry.file_size != uncompressed_size
                    or entry.header_offset != local_offset
                    or entry.external_attr != external_attributes
                    or entry.internal_attr != internal_attributes
                    or entry.extra != extra
                    or entry.comment != comment
                    or entry.create_system != version_made >> 8
                    or entry.create_version != version_made & 0xFF
                    or entry.extract_version != version_needed & 0xFF
                    or entry.reserved != version_needed >> 8
                ):
                    fail(f"{label} disagrees with the ZIP parser")
                parsed.append(
                    {
                        "entry": entry,
                        "name": decoded_name,
                        "raw_name": raw_name,
                        "version_needed": version_needed,
                        "flags": flags,
                        "compression": compression,
                        "modified_time": modified_time,
                        "modified_date": modified_date,
                        "crc32": crc32,
                        "compressed_size": compressed_size,
                        "uncompressed_size": uncompressed_size,
                        "external_attributes": external_attributes,
                        "local_offset": local_offset,
                    }
                )
    except OSError as error:
        fail(f"release IPA central directory is unreadable: {error}")
    offsets = [metadata["local_offset"] for metadata in parsed]
    if not offsets or len(set(offsets)) != len(offsets) or min(offsets) != 0:
        fail("release IPA has overlapping entries or an unreviewed preamble")
    return parsed


def parse_data_descriptor(data: bytes) -> tuple[int, int, int] | None:
    if len(data) == 16 and data[:4] == ZIP_DATA_DESCRIPTOR_SIGNATURE:
        return struct.unpack("<III", data[4:])
    if len(data) == 12:
        return struct.unpack("<III", data)
    return None


def require_data_descriptor(
    descriptor: bytes,
    crc32: int,
    compressed_size: int,
    uncompressed_size: int,
    label: str,
) -> None:
    values = parse_data_descriptor(descriptor)
    if values != (crc32, compressed_size, uncompressed_size):
        fail(f"{label} has an invalid or trailing ZIP data descriptor")


def inflate_raw_deflate(
    region: bytes,
    output_limit: int,
    label: str,
) -> tuple[bytes, int, bytes]:
    decoder = zlib.decompressobj(-zlib.MAX_WBITS)
    try:
        output = decoder.decompress(region, output_limit + 1)
    except zlib.error as error:
        fail(f"{label} has an invalid raw DEFLATE stream: {error}")
    if len(output) > output_limit or decoder.unconsumed_tail:
        fail(f"{label} real inflated size exceeds the reviewed budget")
    if not decoder.eof:
        fail(f"{label} has an incomplete raw DEFLATE stream")
    try:
        flushed = decoder.flush(output_limit - len(output) + 1)
    except zlib.error as error:
        fail(f"{label} raw DEFLATE stream could not be finalized: {error}")
    output += flushed
    if len(output) > output_limit:
        fail(f"{label} real inflated size exceeds the reviewed budget")
    trailing = decoder.unused_data
    compressed_size = len(region) - len(trailing)
    if compressed_size <= 0 and output:
        fail(f"{label} has an invalid raw DEFLATE span")
    return output, compressed_size, trailing


def decode_raw_zip_entry(
    stream,
    metadata: dict,
    region_end: int,
    output_limit: int,
) -> bytes:
    """Decode one local record without trusting either header's size fields."""
    name = metadata["name"]
    label = f"release IPA entry {name!r}"
    try:
        stream.seek(metadata["local_offset"])
        header = read_exact(stream, ZIP_LOCAL_HEADER.size, f"{label} local header")
        (
            signature,
            version_needed,
            flags,
            compression,
            modified_time,
            modified_date,
            local_crc32,
            local_compressed_size,
            local_uncompressed_size,
            name_size,
            extra_size,
        ) = ZIP_LOCAL_HEADER.unpack(header)
        if signature != ZIP_LOCAL_SIGNATURE:
            fail(f"{label} has the wrong local-header signature")
        raw_name = read_exact(stream, name_size, f"{label} local filename")
        local_extra = read_exact(stream, extra_size, f"{label} local extra field")
        validate_zip_extra(local_extra, f"{label} local header")
        data_start = stream.tell()
        if data_start > region_end:
            fail(f"{label} local header overlaps another ZIP record")
        region = read_exact(stream, region_end - data_start, f"{label} data span")
    except OSError as error:
        fail(f"{label} local record is unreadable: {error}")

    if (
        raw_name != metadata["raw_name"]
        or decode_zip_name(raw_name, flags, label) != name
        or version_needed != metadata["version_needed"]
        or flags != metadata["flags"]
        or compression != metadata["compression"]
        or modified_time != metadata["modified_time"]
        or modified_date != metadata["modified_date"]
    ):
        fail(f"{label} local and central headers disagree")
    if 0xFFFFFFFF in (local_compressed_size, local_uncompressed_size):
        fail(f"{label} local header uses unreviewed ZIP64 sentinels")

    uses_descriptor = bool(flags & ZIP_DATA_DESCRIPTOR_FLAG)
    descriptor = b""
    if compression == zipfile.ZIP_DEFLATED:
        output, actual_compressed_size, trailing = inflate_raw_deflate(
            region, output_limit, label
        )
        if uses_descriptor:
            descriptor = trailing
        elif trailing:
            fail(f"{label} has trailing bytes after its DEFLATE stream")
    elif compression == zipfile.ZIP_STORED:
        if uses_descriptor:
            candidates = []
            for descriptor_size in (16, 12):
                if len(region) < descriptor_size:
                    continue
                candidate_output = region[:-descriptor_size]
                candidate_descriptor = region[-descriptor_size:]
                values = parse_data_descriptor(candidate_descriptor)
                candidate_crc32 = zlib.crc32(candidate_output) & 0xFFFFFFFF
                candidate_size = len(candidate_output)
                if values == (candidate_crc32, candidate_size, candidate_size):
                    candidates.append(
                        (candidate_output, candidate_size, candidate_descriptor)
                    )
            if len(candidates) != 1:
                fail(f"{label} has an ambiguous or invalid STORED data descriptor")
            output, actual_compressed_size, descriptor = candidates[0]
        else:
            output = region
            actual_compressed_size = len(region)
        if len(output) > output_limit:
            fail(f"{label} real stored size exceeds the reviewed budget")
    else:
        fail(f"{label} uses an unreviewed compression method")

    actual_uncompressed_size = len(output)
    actual_crc32 = zlib.crc32(output) & 0xFFFFFFFF
    if uses_descriptor:
        require_data_descriptor(
            descriptor,
            actual_crc32,
            actual_compressed_size,
            actual_uncompressed_size,
            label,
        )
        if local_crc32 not in (0, actual_crc32) or local_compressed_size not in (
            0,
            actual_compressed_size,
        ) or local_uncompressed_size not in (0, actual_uncompressed_size):
            fail(f"{label} local data-descriptor placeholders are inconsistent")
    elif (
        local_crc32 != metadata["crc32"]
        or local_compressed_size != metadata["compressed_size"]
        or local_uncompressed_size != metadata["uncompressed_size"]
    ):
        fail(f"{label} local and central sizes or CRC disagree")
    if (
        actual_crc32 != metadata["crc32"]
        or actual_compressed_size != metadata["compressed_size"]
        or actual_uncompressed_size != metadata["uncompressed_size"]
    ):
        fail(f"{label} real size or CRC disagrees with its ZIP declarations")
    return output


def validate_app_file_modes(app: Path, files: set[str], label: str) -> None:
    for relative in sorted(files):
        executable = bool((app / relative).lstat().st_mode & 0o111)
        if relative in REVIEWED_APP_CODE_IMAGES:
            if not executable:
                fail(f"{label} code image is not executable: {relative}")
        elif executable:
            fail(f"{label} non-code resource is unexpectedly executable: {relative}")


def executable_inventory(root: Path, files: set[str]) -> dict[str, bool]:
    return {
        relative: bool((root / relative).lstat().st_mode & 0o111)
        for relative in files
    }


def extract_release_ipa(ipa: Path, destination: Path) -> Path:
    if ipa.suffix.lower() != ".ipa":
        fail("release export must have the .ipa suffix")
    regular_file(ipa, "release IPA")
    ipa_size = ipa.stat().st_size
    if ipa_size <= 0 or ipa_size > MAX_IPA_BYTES:
        fail(f"release IPA size is outside the reviewed budget: {ipa_size}")
    regular_directory(destination, "private IPA extraction directory")
    seen_paths = set()
    total_uncompressed = 0
    try:
        archive = zipfile.ZipFile(ipa, "r")
    except (OSError, zipfile.BadZipFile) as error:
        fail(f"release IPA is not a valid ZIP archive: {error}")
    with archive:
        if archive.comment:
            fail("release IPA contains an unreviewed ZIP archive comment")
        entries = archive.infolist()
        if not (1 <= len(entries) <= MAX_IPA_ENTRIES):
            fail(f"release IPA entry count is outside the reviewed budget: {len(entries)}")
        central_entries = parse_zip_central_entries(ipa, archive, entries)
        ordered_offsets = sorted(metadata["local_offset"] for metadata in central_entries)
        region_ends = {
            offset: (
                ordered_offsets[index + 1]
                if index + 1 < len(ordered_offsets)
                else archive.start_dir
            )
            for index, offset in enumerate(ordered_offsets)
        }
        try:
            raw_stream = ipa.open("rb")
        except OSError as error:
            fail(f"release IPA local records are unreadable: {error}")
        with raw_stream:
            for metadata in central_entries:
                entry = metadata["entry"]
                raw_name = metadata["name"]
                if (
                    entry.filename != raw_name
                    or not raw_name
                    or "\x00" in raw_name
                    or "\\" in raw_name
                    or raw_name.startswith("/")
                    or "//" in raw_name
                    or entry.flag_bits & ZIP_ENCRYPTED_FLAG
                ):
                    fail(f"release IPA contains an unsafe ZIP entry: {raw_name!r}")
                is_directory = entry.is_dir()
                normalized_name = raw_name[:-1] if is_directory else raw_name
                relative = PurePosixPath(normalized_name)
                if (
                    not relative.parts
                    or ".." in relative.parts
                    or relative.as_posix() != normalized_name
                ):
                    fail(f"release IPA contains an unsafe ZIP path: {raw_name!r}")
                parts = relative.parts
                if parts[0] != "Payload" or (
                    len(parts) >= 2 and parts[1] != "BioMotion.app"
                ):
                    fail(
                        "release IPA contains an unreviewed top-level entry: "
                        f"{raw_name}"
                    )
                if len(parts) == 1 and not is_directory:
                    fail("release IPA Payload entry must be a directory")
                folded = normalized_name.casefold()
                if folded in seen_paths:
                    fail(f"release IPA contains a duplicate path: {raw_name}")
                seen_paths.add(folded)
                unix_mode = entry.external_attr >> 16
                file_type = stat.S_IFMT(unix_mode)
                if is_directory:
                    if file_type != stat.S_IFDIR:
                        fail(
                            "release IPA directory lacks an explicit directory type: "
                            f"{raw_name}"
                        )
                elif file_type != stat.S_IFREG:
                    fail(
                        "release IPA file lacks an explicit regular-file type: "
                        f"{raw_name}"
                    )
                if entry.compress_type not in (
                    zipfile.ZIP_STORED,
                    zipfile.ZIP_DEFLATED,
                ):
                    fail(
                        "release IPA uses an unreviewed compression method: "
                        f"{raw_name}"
                    )
                if entry.file_size < 0 or entry.compress_size < 0:
                    fail(f"release IPA entry has an invalid size: {raw_name}")

                output = decode_raw_zip_entry(
                    raw_stream,
                    metadata,
                    region_ends[metadata["local_offset"]],
                    MAX_IPA_UNCOMPRESSED_BYTES - total_uncompressed,
                )
                total_uncompressed += len(output)
                if total_uncompressed > MAX_IPA_UNCOMPRESSED_BYTES:
                    fail(
                        "release IPA real uncompressed size exceeds the reviewed "
                        "budget"
                    )
                if is_directory and output:
                    fail(f"release IPA directory contains data: {raw_name}")

                target = destination.joinpath(*parts)
                if is_directory:
                    target.mkdir(parents=True, exist_ok=True)
                    target.chmod(0o700)
                    continue
                target.parent.mkdir(parents=True, exist_ok=True)
                target.parent.chmod(0o700)
                try:
                    with target.open("xb") as target_stream:
                        target_stream.write(output)
                except OSError as error:
                    fail(
                        "release IPA entry could not be extracted: "
                        f"{raw_name}: {error}"
                    )
                executable = bool(unix_mode & 0o111)
                app_relative = PurePosixPath(*parts[2:]).as_posix()
                if app_relative in REQUIRED_APP_CODE_IMAGES and not executable:
                    fail(
                        "release IPA code image is not executable: "
                        f"{app_relative}"
                    )
                if executable and app_relative not in REVIEWED_APP_CODE_IMAGES:
                    fail(
                        "release IPA non-code resource is unexpectedly executable: "
                        f"{app_relative}"
                    )
                target.chmod(0o700 if executable else 0o600)
    app = destination / "Payload/BioMotion.app"
    regular_directory(app, "exported app bundle")
    return app


def unsigned_macho_identity(path: Path, label: str) -> tuple[int, str]:
    regular_file(path, label)
    with tempfile.TemporaryDirectory(prefix="biomotion-unsigned-macho.") as temp:
        copy = Path(temp) / path.name
        shutil.copyfile(path, copy)
        removal = subprocess.run(
            ["/usr/bin/codesign", "--remove-signature", str(copy)],
            check=False,
            capture_output=True,
            text=True,
            env=TRUSTED_ENV,
        )
        if removal.returncode != 0:
            fail(f"{label} signature could not be normalized: {removal.stderr.strip()}")
        return file_identity(copy)


def compare_archive_and_export(archive_app: Path, exported_app: Path) -> None:
    archive_files, archive_directories, _ = collect_inventory(archive_app)
    exported_files, exported_directories, _ = collect_inventory(exported_app)
    if archive_files != exported_files or archive_directories != exported_directories:
        fail("exported IPA inventory differs from its reviewed archive")
    if executable_inventory(
        archive_app, archive_files
    ) != executable_inventory(exported_app, exported_files):
        fail("exported IPA executable-bit inventory differs from its reviewed archive")
    mutable_signing_files = {
        "_CodeSignature/CodeResources",
        "embedded.mobileprovision",
        f"{EXTENSION_RELATIVE}/_CodeSignature/CodeResources",
        f"{EXTENSION_RELATIVE}/embedded.mobileprovision",
    }
    macho_files = {
        APP_EXECUTABLE,
        f"{EXTENSION_RELATIVE}/{EXTENSION_EXECUTABLE}",
    }
    for relative in sorted(archive_files - mutable_signing_files):
        archive_path = archive_app / relative
        exported_path = exported_app / relative
        if relative in macho_files:
            if unsigned_macho_identity(
                archive_path, f"archive code image {relative}"
            ) != unsigned_macho_identity(
                exported_path, f"exported code image {relative}"
            ):
                fail(f"export changed code outside its signature: {relative}")
        elif archive_path.read_bytes() != exported_path.read_bytes():
            fail(f"export changed a non-signing app file: {relative}")


def inspect_release_ipa(repo: Path, ipa: Path, archive: Path) -> str:
    regular_file(ipa, "release IPA")
    initial_stat = ipa.stat()
    initial_size, initial_digest = file_identity(ipa)
    with tempfile.TemporaryDirectory(prefix="biomotion-release-ipa.") as temp:
        extraction_root = Path(temp)
        if stat.S_IMODE(extraction_root.stat().st_mode) != 0o700:
            fail("private IPA extraction directory is not mode 0700")
        exported_app = extract_release_ipa(ipa, extraction_root)
        archive_app, properties = resolve_release_archive(archive)
        inspect_app(
            repo,
            archive_app,
            "iphoneos",
            release=True,
            archive_properties=properties,
        )
        inspect_app(
            repo,
            exported_app,
            "iphoneos",
            release=True,
            archive_properties=properties,
            require_archive_identity=False,
        )
        compare_archive_and_export(archive_app, exported_app)
        privacy_probe = repo / "tools/tests/privacy_manifest_probe.sh"
        regular_file(privacy_probe, "privacy manifest probe")
        privacy = subprocess.run(
            ["/bin/bash", "-p", str(privacy_probe), str(exported_app)],
            check=False,
            capture_output=True,
            text=True,
            env=TRUSTED_ENV,
        )
        if privacy.returncode != 0 or "PRIVACY_MANIFEST_PROBE_PASS" not in privacy.stdout:
            detail = privacy.stderr.strip() or privacy.stdout.strip()
            fail(f"exported IPA privacy gate failed: {detail}")
        print("PRIVACY_MANIFEST_PROBE_PASS release-ipa")
    final_stat = ipa.stat()
    final_size, final_digest = file_identity(ipa)
    initial_receipt = (
        initial_stat.st_dev,
        initial_stat.st_ino,
        initial_stat.st_size,
        initial_stat.st_mtime_ns,
    )
    final_receipt = (
        final_stat.st_dev,
        final_stat.st_ino,
        final_stat.st_size,
        final_stat.st_mtime_ns,
    )
    if (
        initial_receipt != final_receipt
        or initial_size != final_size
        or initial_digest != final_digest
    ):
        fail("release IPA changed while it was being verified")
    return final_digest


def inspect_app(
    repo: Path,
    app: Path,
    platform: str,
    release: bool,
    archive_properties: Optional[dict] = None,
    require_archive_identity: bool = True,
) -> None:
    regular_directory(app, "app bundle")
    info = read_plist(app / "Info.plist", "app Info.plist")
    if info.get("CFBundleIdentifier") != APP_IDENTIFIER:
        fail(f"unexpected app bundle identifier: {info.get('CFBundleIdentifier')}")
    if info.get("CFBundleExecutable") != APP_EXECUTABLE:
        fail(f"unexpected app executable: {info.get('CFBundleExecutable')}")
    validate_platform_info(info, platform, "app")

    extension = app / EXTENSION_RELATIVE
    regular_directory(extension, "asset-pack extension")
    extension_info = read_plist(extension / "Info.plist", "extension Info.plist")
    if extension_info.get("CFBundleIdentifier") != EXTENSION_IDENTIFIER:
        fail("unexpected extension bundle identifier")
    if extension_info.get("CFBundleExecutable") != EXTENSION_EXECUTABLE:
        fail("unexpected extension executable")
    validate_platform_info(extension_info, platform, "extension")
    for version_key in ("CFBundleShortVersionString", "CFBundleVersion"):
        if not isinstance(info.get(version_key), str) or not info[version_key]:
            fail(f"app has no {version_key}")
        if extension_info.get(version_key) != info[version_key]:
            fail(f"extension {version_key} does not match the app")
        if release and archive_properties is not None:
            if archive_properties.get(version_key) != info[version_key]:
                fail(f"archive {version_key} does not match the app")

    files, directories, total_bytes = collect_inventory(app)
    if total_bytes > MAX_APP_BYTES:
        fail(f"built app exceeds the reviewed total-size budget: {total_bytes}")
    validate_app_file_modes(app, files, "app bundle")

    required_files = {
        "Assets.car",
        APP_EXECUTABLE,
        f"{EXTENSION_RELATIVE}/{EXTENSION_EXECUTABLE}",
        f"{EXTENSION_RELATIVE}/Info.plist",
        "FullBody.osim",
        "Info.plist",
        "NOTICE",
        "PkgInfo",
        "PrivacyInfo.xcprivacy",
        "Rajagopal2016.osim",
        "THIRD-PARTY-NOTICES.txt",
    }
    expected_directories = {"Extensions", EXTENSION_RELATIVE}
    if release:
        required_files.update(
            {
                "_CodeSignature/CodeResources",
                "embedded.mobileprovision",
                f"{EXTENSION_RELATIVE}/_CodeSignature/CodeResources",
                f"{EXTENSION_RELATIVE}/embedded.mobileprovision",
            }
        )
        expected_directories.update(
            {"_CodeSignature", f"{EXTENSION_RELATIVE}/_CodeSignature"}
        )
    else:
        for signature_directory in (
            "_CodeSignature",
            f"{EXTENSION_RELATIVE}/_CodeSignature",
        ):
            if signature_directory in directories:
                expected_directories.add(signature_directory)
                required_files.add(f"{signature_directory}/CodeResources")

    allowed_optional = set()
    if platform == "iphonesimulator":
        allowed_optional.update(
            {
                f"{APP_EXECUTABLE}.debug.dylib",
                "__preview.dylib",
                f"{EXTENSION_RELATIVE}/{EXTENSION_EXECUTABLE}.debug.dylib",
                f"{EXTENSION_RELATIVE}/__preview.dylib",
            }
        )
    observed_optional = files & allowed_optional

    bases = icon_bases(info)
    if bases != {"AppIcon60x60", "AppIcon76x76"}:
        fail(f"app Info.plist icon-base inventory changed: {sorted(bases)}")
    icon_files = {
        "AppIcon60x60@2x.png",
        "AppIcon76x76@2x~ipad.png",
    }
    required_files.update(icon_files)
    expected_files = required_files | observed_optional
    unexpected = sorted(files - expected_files)
    missing = sorted(required_files - files)
    if unexpected or missing:
        fail(
            "built app file allowlist changed: "
            f"unexpected={unexpected} missing={missing}"
        )
    if directories != expected_directories:
        fail(
            "built app directory allowlist changed: "
            f"observed={sorted(directories)} expected={sorted(expected_directories)}"
        )

    require_macho(
        app / APP_EXECUTABLE,
        "app executable",
        64 * 1024 * 1024,
        platform,
        "executable",
    )
    require_macho(
        extension / EXTENSION_EXECUTABLE,
        "extension executable",
        4 * 1024 * 1024,
        platform,
        "executable",
    )
    for relative in observed_optional:
        require_macho(
            app / relative,
            f"optional simulator image {relative}",
            64 * 1024 * 1024,
            platform,
            "dynamically linked shared library",
        )
    for icon in icon_files:
        validate_png(app / icon, f"app icon {icon}")
    validate_asset_car(app / "Assets.car")

    source_resource = repo / "BioMotion/Resources"
    for model in MODEL_IDENTITIES:
        if (app / model).read_bytes() != (source_resource / model).read_bytes():
            fail(f"built app model differs from reviewed source: {model}")
    for bundled, source in (
        ("NOTICE", repo / "NOTICE"),
        ("PrivacyInfo.xcprivacy", repo / "BioMotion/PrivacyInfo.xcprivacy"),
        (
            "THIRD-PARTY-NOTICES.txt",
            source_resource / "THIRD-PARTY-NOTICES.txt",
        ),
    ):
        if (app / bundled).read_bytes() != source.read_bytes():
            fail(f"built {bundled} differs from reviewed source")
    if (app / "PkgInfo").read_bytes() != b"APPL????":
        fail("built app PkgInfo changed")

    if release:
        validate_distribution_signature(
            extension,
            EXTENSION_IDENTIFIER,
            "asset-pack extension",
            extension / "embedded.mobileprovision",
        )
        validate_distribution_signature(
            app,
            APP_IDENTIFIER,
            "app",
            app / "embedded.mobileprovision",
            archive_signing_identity=(
                archive_properties["SigningIdentity"]
                if archive_properties is not None and require_archive_identity
                else None
            ),
        )
        result = subprocess.run(
            ["/usr/bin/codesign", "--verify", "--deep", "--strict", str(app)],
            check=False,
            capture_output=True,
            text=True,
            env=TRUSTED_ENV,
        )
        if result.returncode != 0:
            fail(f"release archive code signature is invalid: {result.stderr.strip()}")


def inspect_test_bundle(repo: Path, bundle: Path) -> None:
    regular_directory(bundle, "test bundle")
    info = read_plist(bundle / "Info.plist", "test Info.plist")
    if info.get("CFBundleIdentifier") != TEST_IDENTIFIER:
        fail("unexpected test bundle identifier")
    if info.get("CFBundleExecutable") != TEST_EXECUTABLE:
        fail("unexpected test executable")
    validate_platform_info(info, "iphonesimulator", "test bundle")
    files, directories, total_bytes = collect_inventory(bundle)
    if total_bytes > 96 * 1024 * 1024:
        fail(f"test bundle exceeds the reviewed total-size budget: {total_bytes}")
    required_files = {"Info.plist", TEST_EXECUTABLE, *MODEL_IDENTITIES}
    required_files.update(f"Fixtures/{name}" for name in FIXTURE_NAMES)
    expected_directories = {"Fixtures"}
    if "_CodeSignature" in directories:
        expected_directories.add("_CodeSignature")
        required_files.add("_CodeSignature/CodeResources")
    if files != required_files:
        fail(
            "built test file allowlist changed: "
            f"unexpected={sorted(files - required_files)} "
            f"missing={sorted(required_files - files)}"
        )
    if directories != expected_directories:
        fail(
            "built test directory allowlist changed: "
            f"observed={sorted(directories)} expected={sorted(expected_directories)}"
        )
    test_executable_mode = (bundle / TEST_EXECUTABLE).lstat().st_mode
    if not test_executable_mode & 0o111:
        fail("test code image is not executable")
    for relative in sorted(files - {TEST_EXECUTABLE}):
        if (bundle / relative).lstat().st_mode & 0o111:
            fail(f"test non-code resource is unexpectedly executable: {relative}")
    require_macho(
        bundle / TEST_EXECUTABLE,
        "test executable",
        80 * 1024 * 1024,
        "iphonesimulator",
        "bundle",
    )
    for model in MODEL_IDENTITIES:
        if (bundle / model).read_bytes() != (
            repo / "BioMotion/Resources" / model
        ).read_bytes():
            fail(f"built test model differs from reviewed source: {model}")
    for name in FIXTURE_NAMES:
        if (bundle / "Fixtures" / name).read_bytes() != (
            repo / "BioMotionTests/Fixtures" / name
        ).read_bytes():
            fail(f"built test fixture differs from reviewed source: {name}")


def usage() -> "NoReturn":
    fail(
        "usage: app_resource_boundary_probe.sh [--simulator-smoke APP | "
        "--tests-bundle-smoke XCTEST | --release-archive XCARCHIVE | "
        "--release-ipa IPA XCARCHIVE]"
    )


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        usage()
    repo = Path(argv[1])
    regular_directory(repo, "repository root")
    inspect_source_and_project(repo)
    arguments = argv[2:]
    if not arguments:
        print("APP_RESOURCE_BOUNDARY_PROBE_PASS source-project")
        return 0
    mode = arguments[0]
    if mode == "--release-ipa":
        if len(arguments) != 3:
            usage()
        digest = inspect_release_ipa(
            repo, Path(arguments[1]), Path(arguments[2])
        )
        print(
            "APP_RESOURCE_BOUNDARY_PROBE_PASS release-ipa "
            f"sha256={digest}"
        )
        return 0
    if len(arguments) != 2:
        usage()
    artifact = Path(arguments[1])
    if mode == "--simulator-smoke":
        inspect_app(repo, artifact, "iphonesimulator", release=False)
        print("APP_RESOURCE_BOUNDARY_PROBE_PASS simulator-smoke")
        return 0
    if mode == "--tests-bundle-smoke":
        inspect_test_bundle(repo, artifact)
        print("APP_RESOURCE_BOUNDARY_PROBE_PASS tests-bundle-smoke")
        return 0
    if mode == "--release-archive":
        app, properties = resolve_release_archive(artifact)
        inspect_app(
            repo,
            app,
            "iphoneos",
            release=True,
            archive_properties=properties,
        )
        print("APP_RESOURCE_BOUNDARY_PROBE_PASS release-archive")
        return 0
    usage()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
