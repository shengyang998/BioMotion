#!/usr/bin/env python3
"""Self-contained tests for the SAM3DBodyPose supply-chain verifier."""

from __future__ import annotations

import contextlib
import copy
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
VERIFIER_PATH = REPOSITORY_ROOT / "tools/assetpack/verify_model_lock.py"
MODULE_SPEC = importlib.util.spec_from_file_location("verify_model_lock", VERIFIER_PATH)
if MODULE_SPEC is None or MODULE_SPEC.loader is None:
    raise RuntimeError(f"cannot import verifier: {VERIFIER_PATH}")
verifier = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(verifier)


def sha256(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def file_records(contents: dict[str, bytes]) -> list[dict[str, object]]:
    return [
        {"path": path, "size": len(content), "sha256": sha256(content)}
        for path, content in sorted(contents.items())
    ]


def metadata_feature(feature: dict[str, object]) -> dict[str, str]:
    data_type_names = {"float16": "Float16", "float32": "Float32"}
    return {
        "name": str(feature["name"]),
        "type": "MultiArray",
        "dataType": data_type_names[str(feature["dataType"])],
        "shape": json.dumps(feature["shape"]),
        "isOptional": "1" if feature["optional"] else "0",
        "hasShapeFlexibility": "1" if feature["shapeFlexible"] else "0",
    }


def protobuf_varint(value: int) -> bytes:
    encoded = bytearray()
    while value >= 0x80:
        encoded.append((value & 0x7F) | 0x80)
        value >>= 7
    encoded.append(value)
    return bytes(encoded)


def protobuf_length_field(field_number: int, payload: bytes) -> bytes:
    return protobuf_varint((field_number << 3) | 2) + protobuf_varint(len(payload)) + payload


class SupplyChainFixture(unittest.TestCase):
    maxDiff = None

    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.license_content = b"fixture SAM license\n"
        self.license_path = self.root / "SAM-LICENSE.txt"
        self.license_path.write_bytes(self.license_content)

        self.interface = {
            "specificationVersion": 8,
            "modelType": "mlProgram",
            "minimumIOS": "17.0",
            "generatedClassName": "SAM3DBodyPose",
            "allowAdditionalFeatures": False,
            "inputs": [
                {
                    "name": "image",
                    "featureType": "multiArray",
                    "dataType": "float16",
                    "shape": [1, 3, 4, 4],
                    "optional": False,
                    "shapeFlexible": False,
                },
                {
                    "name": "ray_map",
                    "featureType": "multiArray",
                    "dataType": "float16",
                    "shape": [1, 2, 4, 4],
                    "optional": False,
                    "shapeFlexible": False,
                },
            ],
            "outputs": [
                {
                    "name": "joint_coords",
                    "featureType": "multiArray",
                    "dataType": "float32",
                    "shape": [2, 3],
                    "optional": False,
                    "shapeFlexible": False,
                }
            ],
        }
        self.metadata = [
            {
                "specificationVersion": 8,
                "modelType": {"name": "MLModelType_mlProgram"},
                "generatedClassName": "SAM3DBodyPose",
                "availability": {"iOS": "17.0", "macOS": "14.0"},
                "inputSchema": [
                    metadata_feature(feature) for feature in self.interface["inputs"]
                ],
                "outputSchema": [
                    metadata_feature(feature) for feature in self.interface["outputs"]
                ],
            }
        ]

        self.source_contents = {
            "Manifest.json": b"fixture manifest\n",
            "Data/com.apple.CoreML/model.mlmodel": b"fixture model\n",
            "Data/com.apple.CoreML/weights/weight.bin": b"fixture weights\n",
        }
        metadata_bytes = (
            json.dumps(self.metadata, sort_keys=True, separators=(",", ":")) + "\n"
        ).encode("utf-8")
        self.compiled_contents = {
            "coremldata.bin": b"fixture core data\n",
            "metadata.json": metadata_bytes,
            "model.mil": b"fixture MIL\n",
            "analytics/coremldata.bin": b"fixture analytics\n",
            "weights/weight.bin": b"fixture compiled weights\n",
        }

        placeholder_hash = sha256(b"fixture provenance")
        self.lock = {
            "schemaVersion": 1,
            "artifactRevision": 1,
            "assetPackID": "sam3d-body-pose",
            "modelBaseName": "SAM3DBodyPose",
            "license": {
                "file": "SAM-LICENSE.txt",
                "sha256": sha256(self.license_content),
                "upstreamPath": "LICENSE",
            },
            "provenance": {
                "repository": "https://example.invalid/sam-3d-body",
                "exportRecipeCommit": "a" * 40,
                "contract": {
                    "path": "export/CONTRACT.md",
                    "sha256": placeholder_hash,
                },
                "conversionLog": {
                    "path": "export/coreml/convert_log.json",
                    "sha256": placeholder_hash,
                },
                "coremlReport": {
                    "path": "export/coreml/coreml_report.json",
                    "sha256": placeholder_hash,
                },
                "sourceInputs": [
                    {
                        "path": "checkpoints/model.ckpt",
                        "size": 1,
                        "sha256": placeholder_hash,
                    }
                ],
                "conversionToolchain": {
                    "python": "3.11.15",
                    "torch": "2.13.0",
                    "coremltools": "9.0",
                    "computePrecision": "mixed",
                    "ioDType": "float16",
                    "deploymentTarget": "iOS17",
                },
                "compileToolchain": {
                    "xcode": "26.4",
                    "xcodeBuild": "17E192",
                    "coremlcompiler": "3520.5.1",
                    "baPackage": "1.2",
                },
            },
            "sourcePackage": {
                "directoryName": "SAM3DBodyPose.mlpackage",
                "files": file_records(self.source_contents),
            },
            "compiledModel": {
                "directoryName": "SAM3DBodyPose.mlmodelc",
                "files": file_records(self.compiled_contents),
            },
            "interface": copy.deepcopy(self.interface),
        }

        self.lock_path = self.root / "SAM3DBodyPose.lock.json"
        self.write_lock()
        self.manifest = {
            "assetPackID": "sam3d-body-pose",
            "downloadPolicy": {
                "prefetch": {
                    "installationEventTypes": [
                        "firstInstallation",
                        "subsequentUpdate",
                    ]
                }
            },
            "fileSelectors": [
                {"directory": "SAM3DBodyPose.mlmodelc"},
                {"file": "SAM3DBodyPose.lock.json"},
                {"file": "SAM-LICENSE.txt"},
            ],
            "platforms": ["iOS"],
        }
        self.manifest_path = self.root / "Manifest.json"
        self.write_manifest()
        self.source_path = self.write_artifact(
            "SAM3DBodyPose.mlpackage", self.source_contents
        )
        self.compiled_path = self.write_artifact(
            "SAM3DBodyPose.mlmodelc", self.compiled_contents
        )

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def write_lock(self, lock: dict[str, object] | None = None) -> None:
        document = self.lock if lock is None else lock
        self.lock_path.write_text(
            json.dumps(document, indent=2) + "\n", encoding="utf-8"
        )

    def write_manifest(self, manifest: dict[str, object] | None = None) -> None:
        document = self.manifest if manifest is None else manifest
        self.manifest_path.write_text(
            json.dumps(document, indent=2) + "\n", encoding="utf-8"
        )

    def write_artifact(
        self, directory_name: str, contents: dict[str, bytes]
    ) -> Path:
        root = self.root / directory_name
        root.mkdir()
        for relative_path, content in contents.items():
            destination = root.joinpath(*relative_path.split("/"))
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(content)
        return root

    def validated_lock(self) -> dict[str, object]:
        return verifier.load_and_validate_lock(self.lock_path)


class ProductionContractTests(unittest.TestCase):
    @unittest.skipUnless(sys.platform == "darwin", "Apple toolchain is Darwin-only")
    def test_local_apple_compile_toolchain_matches_the_lock(self) -> None:
        self.assertEqual(verifier.XCRUN, "/usr/bin/xcrun")
        self.assertEqual(verifier.XCODEBUILD, "/usr/bin/xcodebuild")
        lock = verifier.load_and_validate_lock(
            REPOSITORY_ROOT / "BioMotion/Resources/SAM3DBodyPose.lock.json"
        )
        verifier.verify_compile_toolchain(lock)

        drifted = copy.deepcopy(lock)
        drifted["provenance"]["compileToolchain"]["baPackage"] = "0.0"
        with self.assertRaisesRegex(verifier.VerificationError, "ba-package mismatch"):
            verifier.verify_compile_toolchain(drifted)

    def test_package_driver_freezes_authority_and_pins_trusted_tools(self) -> None:
        package_path = REPOSITORY_ROOT / "tools/assetpack/package.sh"
        package = package_path.read_text(encoding="utf-8")

        self.assertTrue(package.startswith("#!/bin/bash\n"))
        self.assertIn('PATH="/usr/bin:/bin:/usr/sbin:/sbin"', package)
        self.assertIn('PYTHON3="/usr/bin/python3"', package)
        self.assertNotRegex(package, r"(?m)^\s*python3\s")

        snapshot = package.index('echo "==> snapshot authority"')
        repository = package.index('echo "==> verify frozen repository"')
        toolchain = package.index('echo "==> verify frozen toolchain"')
        source = package.index('echo "==> verify source')
        self.assertLess(snapshot, repository)
        self.assertLess(repository, toolchain)
        self.assertLess(toolchain, source)
        self.assertIn(
            '"$PYTHON3" "$VERIFIER" repository '
            '--lock "$SNAPSHOT_LOCK" --license "$SNAPSHOT_LICENSE"',
            package,
        )
        self.assertIn(
            '"$PYTHON3" "$VERIFIER" toolchain '
            '--lock "$SNAPSHOT_LOCK" --license "$SNAPSHOT_LICENSE"',
            package,
        )

    def test_checked_in_lock_captures_the_audited_contract(self) -> None:
        lock_path = (
            REPOSITORY_ROOT / "BioMotion/Resources/SAM3DBodyPose.lock.json"
        )
        license_path = REPOSITORY_ROOT / "BioMotion/Resources/SAM-LICENSE.txt"
        lock = verifier.load_and_validate_lock(lock_path)
        verifier.verify_license(lock, license_path)

        self.assertEqual(lock["assetPackID"], "sam3d-body-pose")
        self.assertEqual(
            lock["provenance"]["exportRecipeCommit"],
            "faa96fc8f9e651131579849701e0fa682b4d4b9c",
        )
        self.assertEqual(
            lock["provenance"]["repository"],
            "https://github.com/facebookresearch/sam-3d-body",
        )
        self.assertEqual(lock["license"]["upstreamPath"], "LICENSE")
        self.assertEqual(
            lock["license"]["sha256"],
            "b3a5a0e2d973ab80e6610ccf1cffc40756050d0ace3cd4fec879b3ec290b2e9b",
        )
        self.assertEqual(
            {
                name: lock["provenance"][name]
                for name in ("contract", "conversionLog", "coremlReport")
            },
            {
                "contract": {
                    "path": "export/CONTRACT.md",
                    "sha256": "6aa70b392b750bcfb4c1695b88fca336a13d284721d1689383427a5654ca5f47",
                },
                "conversionLog": {
                    "path": "export/coreml/convert_log.json",
                    "sha256": "f1de5dd9a61e5d7677c952330dd5efe57abce29a64a15ba46ce81841dcb2370b",
                },
                "coremlReport": {
                    "path": "export/coreml/coreml_report.json",
                    "sha256": "76976adb3973cabc0e747c190dc11d6840730debbc9ce0c156b17d78e30a753a",
                },
            },
        )

        self.assertEqual(
            lock["provenance"]["sourceInputs"],
            [
                {
                    "path": "checkpoints/vith/model.ckpt",
                    "size": 1691205237,
                    "sha256": "3b1cb897f4bbd977bf81cbb0b30780a9582681ac642ee112865790ceb4d66056",
                },
                {
                    "path": "export/assets/mhr_export_assets.pt",
                    "size": 29750464,
                    "sha256": "4427ac984426313518ccb163edffda8c80c97b95557b47f80b6aa6b21fc2c673",
                },
            ],
        )
        self.assertEqual(
            lock["provenance"]["conversionToolchain"],
            {
                "python": "3.11.15",
                "torch": "2.13.0",
                "coremltools": "9.0",
                "computePrecision": "mixed",
                "ioDType": "float16",
                "deploymentTarget": "iOS17",
            },
        )

    def test_checked_in_manifest_selects_only_the_locked_release_payload(self) -> None:
        lock_path = REPOSITORY_ROOT / "BioMotion/Resources/SAM3DBodyPose.lock.json"
        manifest_path = REPOSITORY_ROOT / "tools/assetpack/Manifest.json"
        lock = verifier.load_and_validate_lock(lock_path)
        manifest = verifier.load_and_verify_manifest(manifest_path, lock)

        self.assertEqual(manifest["assetPackID"], lock["assetPackID"])
        self.assertEqual(manifest["platforms"], ["iOS"])
        self.assertEqual(
            manifest["downloadPolicy"],
            {
                "prefetch": {
                    "installationEventTypes": [
                        "firstInstallation",
                        "subsequentUpdate",
                    ]
                }
            },
        )
        self.assertEqual(
            manifest["fileSelectors"],
            [
                {"directory": lock["compiledModel"]["directoryName"]},
                {"file": lock_path.name},
                {"file": lock["license"]["file"]},
            ],
        )
        self.assertEqual(
            lock["provenance"]["compileToolchain"],
            {
                "xcode": "26.4",
                "xcodeBuild": "17E192",
                "coremlcompiler": "3520.5.1",
                "baPackage": "1.2",
            },
        )
        self.assertEqual(
            {
                record["path"]: (record["size"], record["sha256"])
                for record in lock["sourcePackage"]["files"]
            },
            {
                "Manifest.json": (
                    617,
                    "28fdc65d4b1ceca4e536e1a6d344280d68e1ce96ca0e8ca337513f0dac111efe",
                ),
                "Data/com.apple.CoreML/model.mlmodel": (
                    3201988,
                    "946e3457259a29f8fa4dae6028982f15d3f74c1f5dfd409971d532e519456284",
                ),
                "Data/com.apple.CoreML/weights/weight.bin": (
                    1401100744,
                    "57c7eacb381e6a898343a30e0303ddda780b7aea9b0e3193d2b01c7a79e46316",
                ),
            },
        )
        self.assertEqual(
            {
                record["path"]: (record["size"], record["sha256"])
                for record in lock["compiledModel"]["files"]
            },
            {
                "coremldata.bin": (
                    503,
                    "442b4588860a34c3e7695762a3ccd25cbe56c3897ff674536aea8c533fa43c38",
                ),
                "metadata.json": (
                    4300,
                    "2cf4a34b108adc679ac3c4f588e35f53f465d21ddb20bb0af1090aa22fddc2cd",
                ),
                "model.mil": (
                    3722685,
                    "5e6a81aa47306c54627be3d5df4dcfce7741d214e00bbff0fce4032c51b3017d",
                ),
                "analytics/coremldata.bin": (
                    243,
                    "0ca568f2ddf5852232bbf3ed335dfb30eaa07c460b8d225534173ec9cc477ccb",
                ),
                "weights/weight.bin": (
                    1401100744,
                    "57c7eacb381e6a898343a30e0303ddda780b7aea9b0e3193d2b01c7a79e46316",
                ),
            },
        )
        self.assertEqual(
            [
                (feature["name"], feature["dataType"], feature["shape"])
                for feature in lock["interface"]["inputs"]
            ],
            [
                ("image", "float16", [1, 3, 512, 384]),
                ("ray_map", "float16", [1, 2, 512, 384]),
                ("cliff", "float16", [1, 3]),
            ],
        )
        self.assertEqual(
            [
                (feature["name"], feature["dataType"], feature["shape"])
                for feature in lock["interface"]["outputs"]
            ],
            [
                ("joint_coords", "float32", [127, 3]),
                ("global_rots", "float32", [127, 3, 3]),
                ("cam_t", "float32", [3]),
                ("keypoints_2d", "float32", [70, 2]),
            ],
        )
        self.assertEqual(
            {
                key: lock["interface"][key]
                for key in (
                    "specificationVersion",
                    "modelType",
                    "minimumIOS",
                    "generatedClassName",
                    "allowAdditionalFeatures",
                )
            },
            {
                "specificationVersion": 8,
                "modelType": "mlProgram",
                "minimumIOS": "17.0",
                "generatedClassName": "SAM3DBodyPose",
                "allowAdditionalFeatures": False,
            },
        )


class HappyPathTests(SupplyChainFixture):
    def test_accepts_repository_source_compiled_and_metadata(self) -> None:
        lock = self.validated_lock()
        verifier.verify_license(lock, self.license_path)
        verifier.verify_artifact_tree(
            self.source_path, lock["sourcePackage"], "source package"
        )
        verifier.verify_artifact_tree(
            self.compiled_path, lock["compiledModel"], "compiled model"
        )
        verifier.validate_coreml_metadata(self.metadata, lock)

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            return_code = verifier.main(
                [
                    "compiled",
                    "--lock",
                    str(self.lock_path),
                    "--license",
                    str(self.license_path),
                    str(self.compiled_path),
                ]
            )
        self.assertEqual(return_code, 0)
        self.assertIn("MODEL_LOCK_VERIFY_PASS mode=compiled", output.getvalue())


class ManifestContractTests(SupplyChainFixture):
    def test_accepts_exact_manifest(self) -> None:
        lock = self.validated_lock()
        self.assertEqual(
            verifier.load_and_verify_manifest(self.manifest_path, lock),
            self.manifest,
        )

    def test_rejects_manifest_identity_platform_and_policy_drift(self) -> None:
        mutations = [
            ("assetPackID", "wrong-pack", "assetPackID"),
            ("platforms", ["macOS"], "platforms"),
            ("downloadPolicy", {}, "downloadPolicy"),
        ]
        for key, value, pattern in mutations:
            with self.subTest(key=key):
                manifest = copy.deepcopy(self.manifest)
                manifest[key] = value
                with self.assertRaisesRegex(verifier.VerificationError, pattern):
                    verifier.validate_manifest_document(
                        manifest, self.validated_lock()
                    )

    def test_rejects_extra_missing_reordered_and_unsafe_selectors(self) -> None:
        selectors = self.manifest["fileSelectors"]
        mutations = [
            (selectors + [{"file": "unexpected.txt"}], "fileSelectors"),
            (selectors[:-1], "fileSelectors"),
            ([selectors[1], selectors[0], selectors[2]], "fileSelectors"),
            (
                [selectors[0], {"file": "../SAM3DBodyPose.lock.json"}, selectors[2]],
                "parent components",
            ),
        ]
        for value, pattern in mutations:
            with self.subTest(value=value):
                manifest = copy.deepcopy(self.manifest)
                manifest["fileSelectors"] = value
                with self.assertRaisesRegex(verifier.VerificationError, pattern):
                    verifier.validate_manifest_document(
                        manifest, self.validated_lock()
                    )

    def test_rejects_unknown_manifest_and_selector_fields(self) -> None:
        manifest = copy.deepcopy(self.manifest)
        manifest["unexpected"] = True
        with self.assertRaisesRegex(verifier.VerificationError, "unknown keys"):
            verifier.validate_manifest_document(manifest, self.validated_lock())

        manifest = copy.deepcopy(self.manifest)
        manifest["fileSelectors"][0]["file"] = "also-a-file"
        with self.assertRaisesRegex(verifier.VerificationError, "unknown keys"):
            verifier.validate_manifest_document(manifest, self.validated_lock())


class ArchiveListingTests(SupplyChainFixture):
    def listing(self) -> list[dict[str, object]]:
        lock = self.validated_lock()
        sizes = verifier.expected_archive_file_sizes(
            lock,
            self.manifest_path,
            self.lock_path,
            self.license_path,
        )
        rows = []
        for path, entry_type in sorted(
            verifier.expected_archive_entries(lock).items()
        ):
            row = {"TYP": entry_type, "PAT": path}
            if entry_type == "F":
                row["DAT"] = sizes[path]
            rows.append(row)
        return rows

    def test_accepts_exact_archive_allowlist(self) -> None:
        lock = self.validated_lock()
        verifier.validate_archive_listing(
            self.listing(),
            lock,
            verifier.expected_archive_file_sizes(
                lock,
                self.manifest_path,
                self.lock_path,
                self.license_path,
            ),
        )

    def test_rejects_extra_missing_unsafe_and_symlink_archive_entries(self) -> None:
        listing = self.listing()
        mutations = [
            (
                listing + [{"TYP": "F", "PAT": "Contents/unlocked", "DAT": 1}],
                "extra",
            ),
            (listing[:-1], "missing"),
            (
                listing + [{"TYP": "F", "PAT": "../escape", "DAT": 1}],
                "safe relative",
            ),
        ]
        for value, pattern in mutations:
            with self.subTest(pattern=pattern):
                with self.assertRaisesRegex(verifier.VerificationError, pattern):
                    verifier.validate_archive_listing(value, self.validated_lock())

        symlink_listing = self.listing()
        for row in symlink_listing:
            if row["PAT"].endswith("SAM-LICENSE.txt"):
                row["TYP"] = "L"
                break
        with self.assertRaisesRegex(
            verifier.VerificationError, "type mismatch|must not carry DAT"
        ):
            verifier.validate_archive_listing(
                symlink_listing, self.validated_lock()
            )

    def test_rejects_file_size_and_unsupported_metadata_before_extraction(self) -> None:
        lock = self.validated_lock()
        sizes = verifier.expected_archive_file_sizes(
            lock,
            self.manifest_path,
            self.lock_path,
            self.license_path,
        )
        wrong_size = self.listing()
        next(row for row in wrong_size if row["TYP"] == "F")["DAT"] += 1
        with self.assertRaisesRegex(verifier.VerificationError, "size mismatch"):
            verifier.validate_archive_listing(wrong_size, lock, sizes)

        metadata = self.listing()
        metadata[0]["XAT"] = {"unexpected": "xattr"}
        with self.assertRaisesRegex(
            verifier.VerificationError, "unsupported archive metadata fields"
        ):
            verifier.validate_archive_listing(metadata, lock, sizes)


class CompiledMetadataNormalizationTests(SupplyChainFixture):
    def setUp(self) -> None:
        super().setUp()
        self.entries = [
            protobuf_length_field(1, key.encode("utf-8"))
            + protobuf_length_field(2, value.encode("utf-8"))
            for key, value in (
                ("source_dialect", "TorchScript"),
                ("source", "torch==2.13.0"),
                ("version", "9.0"),
                ("conversion_date", "2026-08-06"),
            )
        ]
        self.prefix = b"fixture compiled header\x00"
        self.suffix = b"\x00fixture compiled trailer"
        self.canonical = self.wrap(self.entries)
        self.permuted = self.wrap(
            [self.entries[3], self.entries[1], self.entries[0], self.entries[2]]
        )
        for record in self.lock["compiledModel"]["files"]:
            if record["path"] == "coremldata.bin":
                record["size"] = len(self.canonical)
                record["sha256"] = sha256(self.canonical)
                break
        self.write_lock()

    def wrap(self, entries: list[bytes]) -> bytes:
        map_payload = b"".join(
            protobuf_length_field(100, entry) for entry in entries
        )
        return self.prefix + protobuf_length_field(100, map_payload) + self.suffix

    def test_normalizes_only_a_permutation_that_reconstructs_locked_bytes(self) -> None:
        target = self.compiled_path / "coremldata.bin"
        target.write_bytes(self.permuted)
        verifier.normalize_compiled_coremldata(
            self.compiled_path, self.validated_lock()
        )
        self.assertEqual(target.read_bytes(), self.canonical)
        verifier.verify_artifact_tree(
            self.compiled_path,
            self.validated_lock()["compiledModel"],
            "compiled model",
        )

    def test_exact_locked_bytes_are_a_no_op(self) -> None:
        target = self.compiled_path / "coremldata.bin"
        target.write_bytes(self.canonical)
        before = target.stat()
        verifier.normalize_compiled_coremldata(
            self.compiled_path, self.validated_lock()
        )
        after = target.stat()
        self.assertEqual(target.read_bytes(), self.canonical)
        self.assertEqual((before.st_ino, before.st_mtime_ns), (after.st_ino, after.st_mtime_ns))

    def test_rejects_content_drift_without_modifying_the_compiler_output(self) -> None:
        target = self.compiled_path / "coremldata.bin"
        drifted = self.permuted.replace(b"TorchScript", b"TorchScripx")
        target.write_bytes(drifted)
        with self.assertRaisesRegex(
            verifier.VerificationError, "cannot be normalized to the locked SHA-256"
        ):
            verifier.normalize_compiled_coremldata(
                self.compiled_path, self.validated_lock()
            )
        self.assertEqual(target.read_bytes(), drifted)

    @unittest.skipUnless(hasattr(os, "symlink"), "symlinks are unavailable")
    def test_rejects_symlink_without_touching_its_target(self) -> None:
        target = self.compiled_path / "coremldata.bin"
        target.unlink()
        outside = self.root / "outside-coremldata.bin"
        outside.write_bytes(self.permuted)
        target.symlink_to(outside)
        with self.assertRaisesRegex(verifier.VerificationError, "symlink"):
            verifier.normalize_compiled_coremldata(
                self.compiled_path, self.validated_lock()
            )
        self.assertEqual(outside.read_bytes(), self.permuted)

    @unittest.skipUnless(hasattr(os, "symlink"), "symlinks are unavailable")
    def test_rejects_symlink_compiled_root_without_touching_external_tree(self) -> None:
        outside_parent = self.root / "outside"
        outside_parent.mkdir()
        outside_root = outside_parent / "SAM3DBodyPose.mlmodelc"
        self.compiled_path.rename(outside_root)
        target = outside_root / "coremldata.bin"
        target.write_bytes(self.permuted)
        self.compiled_path.symlink_to(outside_root, target_is_directory=True)
        with self.assertRaisesRegex(verifier.VerificationError, "must not be a symlink"):
            verifier.normalize_compiled_coremldata(
                self.compiled_path, self.validated_lock()
            )
        self.assertEqual(target.read_bytes(), self.permuted)

    @unittest.skipUnless(hasattr(os, "symlink"), "symlinks are unavailable")
    def test_rejects_symlink_compiled_parent_without_touching_external_tree(self) -> None:
        outside_parent = self.root / "outside-parent"
        outside_parent.mkdir()
        outside_root = outside_parent / "SAM3DBodyPose.mlmodelc"
        self.compiled_path.rename(outside_root)
        target = outside_root / "coremldata.bin"
        target.write_bytes(self.permuted)
        alias_parent = self.root / "alias-parent"
        alias_parent.symlink_to(outside_parent, target_is_directory=True)
        with self.assertRaisesRegex(
            verifier.VerificationError, "parent directory must not be a symlink"
        ):
            verifier.normalize_compiled_coremldata(
                alias_parent / "SAM3DBodyPose.mlmodelc", self.validated_lock()
            )
        self.assertEqual(target.read_bytes(), self.permuted)


class ReceiptContractTests(SupplyChainFixture):
    def setUp(self) -> None:
        super().setUp()
        self.aar_path = self.root / "sam3d-body-pose.aar"
        self.aar_path.write_bytes(b"fixture validated AAR\n")
        self.receipt_path = self.root / "sam3d-body-pose.aar.receipt.json"

    def build_receipt(self) -> dict[str, object]:
        return verifier.build_receipt_document(
            self.validated_lock(),
            self.aar_path,
            self.manifest_path,
            self.lock_path,
            self.license_path,
        )

    def write_receipt(self, receipt: dict[str, object] | None = None) -> None:
        document = self.build_receipt() if receipt is None else receipt
        self.receipt_path.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def test_accepts_receipt_bound_to_every_release_input(self) -> None:
        receipt = self.build_receipt()
        verifier.validate_receipt_document(receipt, self.validated_lock())
        self.write_receipt(receipt)
        verifier._verify_receipt_bindings(
            self.receipt_path,
            self.aar_path,
            self.manifest_path,
            self.lock_path,
            self.license_path,
        )

    def test_rejects_unknown_receipt_fields_and_identity_drift(self) -> None:
        receipt = self.build_receipt()
        receipt["unexpected"] = True
        with self.assertRaisesRegex(verifier.VerificationError, "unknown keys"):
            verifier.validate_receipt_document(receipt, self.validated_lock())

        for key, value in (
            ("schemaVersion", 2),
            ("assetPackID", "wrong-pack"),
            ("artifactRevision", 2),
            ("artifactRevision", True),
            ("modelBaseName", "WrongModel"),
        ):
            with self.subTest(key=key):
                receipt = self.build_receipt()
                receipt[key] = value
                with self.assertRaisesRegex(verifier.VerificationError, key):
                    verifier.validate_receipt_document(
                        receipt, self.validated_lock()
                    )

    def test_rejects_archive_filename_path_size_and_hash_drift(self) -> None:
        mutations = [
            ("file", "../sam3d-body-pose.aar", "parent components"),
            ("file", "wrong.aar", "archive filename"),
            ("size", 1, "size mismatch"),
            ("sha256", "0" * 64, "SHA-256 mismatch"),
        ]
        for key, value, pattern in mutations:
            with self.subTest(key=key, value=value):
                receipt = self.build_receipt()
                receipt["aar"][key] = value
                self.write_receipt(receipt)
                with self.assertRaisesRegex(verifier.VerificationError, pattern):
                    verifier._verify_receipt_bindings(
                        self.receipt_path,
                        self.aar_path,
                        self.manifest_path,
                        self.lock_path,
                        self.license_path,
                    )

    def test_rejects_manifest_lock_and_license_hash_drift(self) -> None:
        for sidecar in ("manifest", "lock", "license"):
            with self.subTest(sidecar=sidecar):
                receipt = self.build_receipt()
                receipt["sidecars"][sidecar]["sha256"] = "0" * 64
                self.write_receipt(receipt)
                with self.assertRaisesRegex(
                    verifier.VerificationError, f"{sidecar} SHA-256 mismatch"
                ):
                    verifier._verify_receipt_bindings(
                        self.receipt_path,
                        self.aar_path,
                        self.manifest_path,
                        self.lock_path,
                        self.license_path,
                    )

    def test_seal_rejects_aar_and_authority_changes_during_verification(self) -> None:
        cases = (
            (self.aar_path, b"changed AAR\n", "AAR changed"),
            (self.manifest_path, b"changed manifest\n", "seal authority changed"),
        )
        for changed_path, changed_bytes, diagnostic in cases:
            with self.subTest(changed_path=changed_path.name):
                self.aar_path.write_bytes(b"fixture validated AAR\n")
                self.write_manifest()
                if self.receipt_path.exists():
                    self.receipt_path.unlink()

                def mutate_during_archive_verification(*_args, **_kwargs) -> None:
                    changed_path.write_bytes(changed_bytes)

                with mock.patch.object(
                    verifier,
                    "verify_archive",
                    side_effect=mutate_during_archive_verification,
                ):
                    with self.assertRaisesRegex(
                        verifier.VerificationError, diagnostic
                    ):
                        verifier.seal_archive_receipt(
                            self.receipt_path,
                            self.aar_path,
                            self.manifest_path,
                            self.lock_path,
                            self.license_path,
                        )
                self.assertFalse(self.receipt_path.exists())

    def test_seal_rechecks_after_receipt_write_and_removes_failed_receipt(self) -> None:
        original_write_receipt = verifier.write_receipt
        original_verify_bindings = verifier._verify_receipt_bindings
        cases = (
            (
                "write AAR drift",
                "write_receipt",
                original_write_receipt,
                self.aar_path,
                "AAR changed while it was being sealed",
            ),
            (
                "write authority drift",
                "write_receipt",
                original_write_receipt,
                self.manifest_path,
                "seal authority changed",
            ),
            (
                "binding AAR drift",
                "_verify_receipt_bindings",
                original_verify_bindings,
                self.aar_path,
                "AAR changed while it was being sealed",
            ),
            (
                "binding authority drift",
                "_verify_receipt_bindings",
                original_verify_bindings,
                self.manifest_path,
                "seal authority changed",
            ),
        )
        for label, target, delegate, changed_path, diagnostic in cases:
            with self.subTest(label=label):
                self.aar_path.write_bytes(b"fixture validated AAR\n")
                self.write_manifest()
                if self.receipt_path.exists():
                    self.receipt_path.unlink()

                def mutate_then_continue(*args, **kwargs):
                    if changed_path == self.manifest_path:
                        changed_path.write_bytes(changed_path.read_bytes() + b" \n")
                    else:
                        changed_path.write_bytes(b"changed AAR during seal\n")
                    return delegate(*args, **kwargs)

                with mock.patch.object(
                    verifier, "verify_archive"
                ), mock.patch.object(
                    verifier, target, side_effect=mutate_then_continue
                ):
                    with self.assertRaisesRegex(
                        verifier.VerificationError, diagnostic
                    ):
                        verifier.seal_archive_receipt(
                            self.receipt_path,
                            self.aar_path,
                            self.manifest_path,
                            self.lock_path,
                            self.license_path,
                        )
                self.assertFalse(
                    self.receipt_path.exists(),
                    "failed seal must not leave a receipt behind",
                )

    def test_seal_receipt_stays_bound_to_one_snapshot_across_aba_drift(self) -> None:
        original_aar = self.aar_path.read_bytes()
        original_write_receipt = verifier.write_receipt
        original_verify_bindings = verifier._verify_receipt_bindings

        def switch_live_aar_then_write(*args, **kwargs):
            self.aar_path.write_bytes(b"temporary ABA AAR generation B\n")
            return original_write_receipt(*args, **kwargs)

        def verify_snapshot_then_restore_live_aar(*args, **kwargs):
            result = original_verify_bindings(*args, **kwargs)
            self.aar_path.write_bytes(original_aar)
            return result

        with mock.patch.object(verifier, "verify_archive"), mock.patch.object(
            verifier, "write_receipt", side_effect=switch_live_aar_then_write
        ), mock.patch.object(
            verifier,
            "_verify_receipt_bindings",
            side_effect=verify_snapshot_then_restore_live_aar,
        ):
            verifier.seal_archive_receipt(
                self.receipt_path,
                self.aar_path,
                self.manifest_path,
                self.lock_path,
                self.license_path,
            )

        verifier._verify_receipt_bindings(
            self.receipt_path,
            self.aar_path,
            self.manifest_path,
            self.lock_path,
            self.license_path,
        )
        receipt = verifier.load_json_file(self.receipt_path, "test receipt")
        self.assertEqual(receipt["aar"]["sha256"], sha256(original_aar))

    def test_seal_never_exposes_a_partially_written_receipt(self) -> None:
        def write_partial_then_fail(output_path, *_args, **_kwargs):
            Path(output_path).write_bytes(b"")
            raise verifier.VerificationError("injected receipt write failure")

        with mock.patch.object(verifier, "verify_archive"), mock.patch.object(
            verifier, "write_receipt", side_effect=write_partial_then_fail
        ):
            with self.assertRaisesRegex(
                verifier.VerificationError, "injected receipt write failure"
            ):
                verifier.seal_archive_receipt(
                    self.receipt_path,
                    self.aar_path,
                    self.manifest_path,
                    self.lock_path,
                    self.license_path,
                )
        self.assertFalse(self.receipt_path.exists())

    def test_rejects_sidecar_filename_and_receipt_symlink(self) -> None:
        receipt = self.build_receipt()
        receipt["sidecars"]["lock"]["file"] = "nested/lock.json"
        with self.assertRaisesRegex(
            verifier.VerificationError, "single path component"
        ):
            verifier.validate_receipt_document(receipt, self.validated_lock())

        if hasattr(os, "symlink"):
            target = self.root / "receipt-target.json"
            target.write_text("{}\n", encoding="utf-8")
            self.receipt_path.symlink_to(target)
            with self.assertRaisesRegex(verifier.VerificationError, "symlink"):
                verifier._verify_receipt_bindings(
                    self.receipt_path,
                    self.aar_path,
                    self.manifest_path,
                    self.lock_path,
                    self.license_path,
                )


class AtomicPublishTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.candidate = self.root / "transaction/release-candidate"
        self.destination = self.root / "published"
        self.candidate.mkdir(parents=True)
        (self.candidate / "pair.txt").write_text("new pair\n", encoding="utf-8")

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def simulated_swap(self, source: Path, destination: Path) -> None:
        temporary = self.root / ".simulated-swap"
        os.rename(source, temporary)
        os.rename(destination, source)
        os.rename(temporary, destination)

    def test_first_publication_is_one_directory_rename(self) -> None:
        verifier.atomic_publish_directory(self.candidate, self.destination)
        self.assertFalse(self.candidate.exists())
        self.assertEqual(
            (self.destination / "pair.txt").read_text(encoding="utf-8"),
            "new pair\n",
        )

    def test_first_publication_post_fsync_failure_rolls_back_namespace(self) -> None:
        fsync_calls = []

        def fail_first_fsync(path):
            fsync_calls.append(Path(path))
            if len(fsync_calls) == 1:
                raise verifier.VerificationError("injected post-publication fsync EIO")

        with mock.patch.object(
            verifier, "_fsync_directory", side_effect=fail_first_fsync
        ):
            with self.assertRaisesRegex(
                verifier.VerificationError, "rolled back"
            ):
                verifier.atomic_publish_directory(self.candidate, self.destination)

        self.assertFalse(self.destination.exists())
        self.assertEqual(
            (self.candidate / "pair.txt").read_text(encoding="utf-8"),
            "new pair\n",
        )
        self.assertGreaterEqual(len(fsync_calls), 3)

    def test_replacement_post_fsync_failure_swaps_old_release_back(self) -> None:
        self.destination.mkdir()
        (self.destination / "pair.txt").write_text("old pair\n", encoding="utf-8")
        fsync_calls = []

        def fail_second_parent_fsync(path):
            fsync_calls.append(Path(path))
            if len(fsync_calls) == 2:
                raise verifier.VerificationError("injected post-publication fsync EIO")

        with mock.patch.object(
            verifier, "_rename_swap", side_effect=self.simulated_swap
        ), mock.patch.object(
            verifier, "_fsync_directory", side_effect=fail_second_parent_fsync
        ):
            with self.assertRaisesRegex(
                verifier.VerificationError, "rolled back"
            ):
                verifier.atomic_publish_directory(self.candidate, self.destination)

        self.assertEqual(
            (self.destination / "pair.txt").read_text(encoding="utf-8"),
            "old pair\n",
        )
        self.assertEqual(
            (self.candidate / "pair.txt").read_text(encoding="utf-8"),
            "new pair\n",
        )

    def test_first_publication_rollback_rename_failure_requires_recovery(self) -> None:
        real_rename = os.rename
        rename_calls = 0

        def fail_rollback_rename(source, destination):
            nonlocal rename_calls
            rename_calls += 1
            if rename_calls == 2:
                raise OSError("injected rollback rename failure")
            return real_rename(source, destination)

        with mock.patch.object(os, "rename", side_effect=fail_rollback_rename), mock.patch.object(
            verifier,
            "_fsync_directory",
            side_effect=verifier.VerificationError(
                "injected post-publication fsync EIO"
            ),
        ):
            with self.assertRaisesRegex(
                verifier.RecoveryRequiredError, "manual recovery"
            ):
                verifier.atomic_publish_directory(self.candidate, self.destination)

        self.assertFalse(self.candidate.exists())
        self.assertEqual(
            (self.destination / "pair.txt").read_text(encoding="utf-8"),
            "new pair\n",
        )

    def test_replacement_rollback_swap_failure_requires_recovery(self) -> None:
        self.destination.mkdir()
        (self.destination / "pair.txt").write_text("old pair\n", encoding="utf-8")
        swap_calls = 0

        def fail_rollback_swap(source, destination):
            nonlocal swap_calls
            swap_calls += 1
            if swap_calls == 2:
                raise OSError("injected rollback swap failure")
            self.simulated_swap(source, destination)

        with mock.patch.object(
            verifier, "_rename_swap", side_effect=fail_rollback_swap
        ), mock.patch.object(
            verifier,
            "_fsync_directory",
            side_effect=verifier.VerificationError(
                "injected post-publication fsync EIO"
            ),
        ):
            with self.assertRaisesRegex(
                verifier.RecoveryRequiredError, "manual recovery"
            ):
                verifier.atomic_publish_directory(self.candidate, self.destination)

        self.assertEqual(
            (self.destination / "pair.txt").read_text(encoding="utf-8"),
            "new pair\n",
        )
        self.assertEqual(
            (self.candidate / "pair.txt").read_text(encoding="utf-8"),
            "old pair\n",
        )

    def test_replacement_rollback_fsync_failure_requires_recovery(self) -> None:
        self.destination.mkdir()
        (self.destination / "pair.txt").write_text("old pair\n", encoding="utf-8")
        fsync_calls = 0

        def fail_post_and_first_rollback_fsync(_path):
            nonlocal fsync_calls
            fsync_calls += 1
            if fsync_calls <= 2:
                raise verifier.VerificationError(
                    f"injected fsync EIO call {fsync_calls}"
                )

        with mock.patch.object(
            verifier, "_rename_swap", side_effect=self.simulated_swap
        ), mock.patch.object(
            verifier,
            "_fsync_directory",
            side_effect=fail_post_and_first_rollback_fsync,
        ):
            with self.assertRaisesRegex(
                verifier.RecoveryRequiredError, "manual recovery"
            ):
                verifier.atomic_publish_directory(self.candidate, self.destination)

        self.assertEqual(
            (self.destination / "pair.txt").read_text(encoding="utf-8"),
            "old pair\n",
        )
        self.assertEqual(
            (self.candidate / "pair.txt").read_text(encoding="utf-8"),
            "new pair\n",
        )

    def test_unexpected_publish_exception_returns_recovery_status(self) -> None:
        error_output = io.StringIO()
        with mock.patch.object(
            verifier,
            "run",
            side_effect=RuntimeError("injected unexpected publish crash"),
        ), contextlib.redirect_stderr(error_output):
            return_code = verifier.main(["publish"])

        self.assertEqual(return_code, 2)
        self.assertIn("MODEL_LOCK_RECOVERY_REQUIRED", error_output.getvalue())
        self.assertIn("unexpected publish failure", error_output.getvalue())

    def test_publish_fsyncs_pair_and_candidate_before_atomic_rename(self) -> None:
        aar = self.candidate / "sam3d-body-pose.aar"
        receipt = self.candidate / "sam3d-body-pose.aar.receipt.json"
        events = []

        def verified(*_args, **_kwargs):
            events.append(("verify", self.candidate))
            return aar, receipt

        def fsynced_file(path, label):
            events.append(("fsync-file", Path(path), label))

        def fsynced_directory(path):
            events.append(("fsync-directory", Path(path)))

        def published(candidate, destination):
            events.append(("publish", Path(candidate), Path(destination)))

        with mock.patch.object(
            verifier, "_verify_release_pair_directory", side_effect=verified
        ), mock.patch.object(
            verifier, "_fsync_regular_file", side_effect=fsynced_file
        ), mock.patch.object(
            verifier, "_fsync_directory", side_effect=fsynced_directory
        ), mock.patch.object(
            verifier, "atomic_publish_directory", side_effect=published
        ):
            verifier.publish_release_pair(
                self.candidate,
                self.destination,
                self.root / "Manifest.json",
                self.root / "SAM3DBodyPose.lock.json",
                self.root / "SAM-LICENSE.txt",
            )

        self.assertEqual(
            events,
            [
                ("verify", self.candidate),
                ("fsync-file", aar, "release candidate AAR"),
                ("fsync-file", receipt, "release candidate receipt"),
                ("fsync-directory", self.candidate),
                ("publish", self.candidate, self.destination),
            ],
        )

    def test_fsync_failures_are_verification_errors_before_publication(self) -> None:
        target = self.candidate / "pair.txt"
        real_close = os.close

        def close_then_fail(descriptor):
            real_close(descriptor)
            raise OSError("injected close EIO")

        cases = (
            (
                "file fstat",
                lambda: verifier._fsync_regular_file(
                    target, "release candidate AAR"
                ),
                "fstat",
                OSError("injected fstat EIO"),
                "cannot inspect opened release candidate AAR",
            ),
            (
                "file fsync",
                lambda: verifier._fsync_regular_file(
                    target, "release candidate AAR"
                ),
                "fsync",
                OSError("injected fsync EIO"),
                "cannot fsync release candidate AAR",
            ),
            (
                "file close",
                lambda: verifier._fsync_regular_file(
                    target, "release candidate AAR"
                ),
                "close",
                close_then_fail,
                "cannot close release candidate AAR after fsync",
            ),
            (
                "directory fstat",
                lambda: verifier._fsync_directory(self.candidate),
                "fstat",
                OSError("injected fstat EIO"),
                "cannot inspect opened directory for fsync",
            ),
            (
                "directory fsync",
                lambda: verifier._fsync_directory(self.candidate),
                "fsync",
                OSError("injected fsync EIO"),
                "cannot fsync directory",
            ),
            (
                "directory close",
                lambda: verifier._fsync_directory(self.candidate),
                "close",
                close_then_fail,
                "cannot close directory after fsync",
            ),
        )
        for label, operation, system_call, failure, diagnostic in cases:
            with self.subTest(label=label), mock.patch.object(
                os, system_call, side_effect=failure
            ):
                with self.assertRaisesRegex(
                    verifier.VerificationError,
                    diagnostic,
                ):
                    operation()

    @unittest.skipUnless(sys.platform == "darwin", "renameatx_np is Darwin-only")
    def test_existing_pair_is_replaced_by_one_atomic_directory_swap(self) -> None:
        self.destination.mkdir()
        (self.destination / "pair.txt").write_text("old pair\n", encoding="utf-8")
        verifier.atomic_publish_directory(self.candidate, self.destination)
        self.assertEqual(
            (self.destination / "pair.txt").read_text(encoding="utf-8"),
            "new pair\n",
        )
        self.assertEqual(
            (self.candidate / "pair.txt").read_text(encoding="utf-8"),
            "old pair\n",
        )

    def test_swap_failure_leaves_both_directories_unchanged(self) -> None:
        self.destination.mkdir()
        (self.destination / "pair.txt").write_text("old pair\n", encoding="utf-8")
        with mock.patch.object(
            verifier, "_rename_swap", side_effect=OSError("injected swap failure")
        ):
            with self.assertRaisesRegex(
                verifier.VerificationError, "atomic directory swap failed"
            ):
                verifier.atomic_publish_directory(self.candidate, self.destination)
        self.assertEqual(
            (self.destination / "pair.txt").read_text(encoding="utf-8"),
            "old pair\n",
        )
        self.assertEqual(
            (self.candidate / "pair.txt").read_text(encoding="utf-8"),
            "new pair\n",
        )

    @unittest.skipUnless(hasattr(os, "symlink"), "symlinks are unavailable")
    def test_rejects_symlink_destination_without_touching_it(self) -> None:
        outside = self.root / "outside"
        outside.mkdir()
        self.destination.symlink_to(outside, target_is_directory=True)
        with self.assertRaisesRegex(verifier.VerificationError, "symlink"):
            verifier.atomic_publish_directory(self.candidate, self.destination)
        self.assertTrue(self.candidate.is_dir())
        self.assertTrue(self.destination.is_symlink())


class LockAndLicenseFailureTests(SupplyChainFixture):
    def test_rejects_asset_pack_id_drift(self) -> None:
        drifted = copy.deepcopy(self.lock)
        drifted["assetPackID"] = "sam-3d-body-pose"
        self.write_lock(drifted)
        with self.assertRaisesRegex(
            verifier.VerificationError, "assetPackID must be 'sam3d-body-pose'"
        ):
            verifier.load_and_validate_lock(self.lock_path)

    def test_rejects_unknown_lock_fields(self) -> None:
        drifted = copy.deepcopy(self.lock)
        drifted["unexpected"] = True
        self.write_lock(drifted)
        with self.assertRaisesRegex(verifier.VerificationError, "unknown keys"):
            verifier.load_and_validate_lock(self.lock_path)

    def test_rejects_non_integer_version_fields(self) -> None:
        mutations = [
            ("schemaVersion", True, "schemaVersion must be an integer"),
            ("schemaVersion", 1.0, "schemaVersion must be an integer"),
        ]
        for key, value, pattern in mutations:
            with self.subTest(key=key, value=value):
                drifted = copy.deepcopy(self.lock)
                drifted[key] = value
                self.write_lock(drifted)
                with self.assertRaisesRegex(verifier.VerificationError, pattern):
                    verifier.load_and_validate_lock(self.lock_path)

        drifted = copy.deepcopy(self.lock)
        drifted["interface"]["specificationVersion"] = 8.0
        self.write_lock(drifted)
        with self.assertRaisesRegex(
            verifier.VerificationError,
            "interface.specificationVersion must be an integer",
        ):
            verifier.load_and_validate_lock(self.lock_path)

    def test_rejects_dot_as_a_provenance_path(self) -> None:
        drifted = copy.deepcopy(self.lock)
        drifted["provenance"]["contract"]["path"] = "."
        self.write_lock(drifted)
        with self.assertRaisesRegex(
            verifier.VerificationError, "normalized POSIX relative path"
        ):
            verifier.load_and_validate_lock(self.lock_path)

    def test_rejects_duplicate_json_keys(self) -> None:
        self.lock_path.write_text(
            '{"schemaVersion":1,"schemaVersion":1}\n', encoding="utf-8"
        )
        with self.assertRaisesRegex(verifier.VerificationError, "duplicate key"):
            verifier.load_and_validate_lock(self.lock_path)

    def test_rejects_license_hash_mismatch(self) -> None:
        lock = self.validated_lock()
        self.license_path.write_bytes(b"fixture SAM licensf\n")
        with self.assertRaisesRegex(verifier.VerificationError, "license SHA-256"):
            verifier.verify_license(lock, self.license_path)

    @unittest.skipUnless(hasattr(os, "symlink"), "symlinks are unavailable")
    def test_rejects_symlink_license(self) -> None:
        lock = self.validated_lock()
        target = self.root / "actual-license.txt"
        target.write_bytes(self.license_content)
        self.license_path.unlink()
        self.license_path.symlink_to(target)
        with self.assertRaisesRegex(verifier.VerificationError, "must not be a symlink"):
            verifier.verify_license(lock, self.license_path)


class ArtifactTreeFailureTests(SupplyChainFixture):
    def test_rejects_missing_source_file(self) -> None:
        lock = self.validated_lock()
        (self.source_path / "Manifest.json").unlink()
        with self.assertRaisesRegex(verifier.VerificationError, "missing files"):
            verifier.verify_artifact_tree(
                self.source_path, lock["sourcePackage"], "source package"
            )

    def test_rejects_extra_source_file(self) -> None:
        lock = self.validated_lock()
        (self.source_path / "unlocked.bin").write_bytes(b"extra")
        with self.assertRaisesRegex(verifier.VerificationError, "extra files"):
            verifier.verify_artifact_tree(
                self.source_path, lock["sourcePackage"], "source package"
            )

    def test_rejects_source_size_mismatch(self) -> None:
        lock = self.validated_lock()
        manifest = self.source_path / "Manifest.json"
        manifest.write_bytes(manifest.read_bytes() + b"x")
        with self.assertRaisesRegex(verifier.VerificationError, "size mismatch"):
            verifier.verify_artifact_tree(
                self.source_path, lock["sourcePackage"], "source package"
            )

    def test_rejects_source_hash_mismatch(self) -> None:
        lock = self.validated_lock()
        manifest = self.source_path / "Manifest.json"
        content = manifest.read_bytes()
        manifest.write_bytes(bytes([content[0] ^ 1]) + content[1:])
        with self.assertRaisesRegex(verifier.VerificationError, "SHA-256 mismatch"):
            verifier.verify_artifact_tree(
                self.source_path, lock["sourcePackage"], "source package"
            )

    @unittest.skipUnless(hasattr(os, "symlink"), "symlinks are unavailable")
    def test_rejects_symlink_source_file(self) -> None:
        lock = self.validated_lock()
        manifest = self.source_path / "Manifest.json"
        manifest.unlink()
        manifest.symlink_to(self.license_path)
        with self.assertRaisesRegex(verifier.VerificationError, "symlink file"):
            verifier.verify_artifact_tree(
                self.source_path, lock["sourcePackage"], "source package"
            )

    def test_rejects_missing_compiled_file(self) -> None:
        lock = self.validated_lock()
        (self.compiled_path / "model.mil").unlink()
        with self.assertRaisesRegex(verifier.VerificationError, "missing files"):
            verifier.verify_artifact_tree(
                self.compiled_path, lock["compiledModel"], "compiled model"
            )

    def test_rejects_extra_compiled_directory(self) -> None:
        lock = self.validated_lock()
        (self.compiled_path / "unlocked-directory").mkdir()
        with self.assertRaisesRegex(verifier.VerificationError, "extra directories"):
            verifier.verify_artifact_tree(
                self.compiled_path, lock["compiledModel"], "compiled model"
            )

    @unittest.skipUnless(hasattr(os, "symlink"), "symlinks are unavailable")
    def test_rejects_symlink_compiled_file(self) -> None:
        lock = self.validated_lock()
        model = self.compiled_path / "model.mil"
        model.unlink()
        model.symlink_to(self.license_path)
        with self.assertRaisesRegex(verifier.VerificationError, "symlink file"):
            verifier.verify_artifact_tree(
                self.compiled_path, lock["compiledModel"], "compiled model"
            )


class MetadataFailureTests(SupplyChainFixture):
    def assert_metadata_rejected(self, metadata: object, pattern: str) -> None:
        with self.assertRaisesRegex(verifier.VerificationError, pattern):
            verifier.validate_coreml_metadata(metadata, self.validated_lock())

    def test_rejects_missing_and_extra_features(self) -> None:
        missing = copy.deepcopy(self.metadata)
        missing[0]["inputSchema"].pop()
        self.assert_metadata_rejected(missing, "missing features")

        extra = copy.deepcopy(self.metadata)
        unexpected = copy.deepcopy(extra[0]["inputSchema"][0])
        unexpected["name"] = "unlocked_input"
        extra[0]["inputSchema"].append(unexpected)
        self.assert_metadata_rejected(extra, "extra features")

    def test_rejects_feature_type_and_dtype_mismatch(self) -> None:
        wrong_type = copy.deepcopy(self.metadata)
        wrong_type[0]["inputSchema"][0]["type"] = "Image"
        self.assert_metadata_rejected(wrong_type, "type mismatch")

        wrong_dtype = copy.deepcopy(self.metadata)
        wrong_dtype[0]["inputSchema"][0]["dataType"] = "Float32"
        self.assert_metadata_rejected(wrong_dtype, "dataType mismatch")

    def test_rejects_shape_mismatch(self) -> None:
        metadata = copy.deepcopy(self.metadata)
        metadata[0]["inputSchema"][0]["shape"] = "[1, 3, 4, 5]"
        self.assert_metadata_rejected(metadata, "shape mismatch")

    def test_rejects_optional_and_flexible_shape_mismatch(self) -> None:
        optional = copy.deepcopy(self.metadata)
        optional[0]["inputSchema"][0]["isOptional"] = "1"
        self.assert_metadata_rejected(optional, "isOptional mismatch")

        flexible = copy.deepcopy(self.metadata)
        flexible[0]["inputSchema"][0]["hasShapeFlexibility"] = "1"
        self.assert_metadata_rejected(flexible, "hasShapeFlexibility mismatch")

    def test_rejects_core_model_contract_mismatch(self) -> None:
        mutations = [
            ("specificationVersion", 9, "specificationVersion mismatch"),
            ("generatedClassName", "WrongModel", "generatedClassName mismatch"),
        ]
        for key, value, pattern in mutations:
            with self.subTest(key=key):
                metadata = copy.deepcopy(self.metadata)
                metadata[0][key] = value
                self.assert_metadata_rejected(metadata, pattern)

        wrong_type = copy.deepcopy(self.metadata)
        wrong_type[0]["modelType"]["name"] = "MLModelType_neuralNetwork"
        self.assert_metadata_rejected(wrong_type, "modelType mismatch")

        wrong_ios = copy.deepcopy(self.metadata)
        wrong_ios[0]["availability"]["iOS"] = "18.0"
        self.assert_metadata_rejected(wrong_ios, "minimum iOS mismatch")

    def test_rejects_non_integer_metadata_specification_version(self) -> None:
        for value in (8.0, True):
            with self.subTest(value=value):
                metadata = copy.deepcopy(self.metadata)
                metadata[0]["specificationVersion"] = value
                self.assert_metadata_rejected(
                    metadata, "specificationVersion must be an integer"
                )


if __name__ == "__main__":
    unittest.main()
