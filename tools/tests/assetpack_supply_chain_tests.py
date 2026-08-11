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
import tempfile
import unittest


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
