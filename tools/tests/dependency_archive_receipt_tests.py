#!/usr/bin/env python3
"""Hermetic adversarial tests for the Release dependency archive receipt."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import plistlib
import stat
import subprocess
import tempfile
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "release"
    / "dependency_archive_receipt.py"
)
PYTHON = "/usr/bin/python3"


class DependencyArchiveReceiptTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(
            prefix="biomotion-dependency-receipt-tests."
        )
        self.test_root = Path(self.temporary.name)
        self.repository = self.test_root / "repository"
        self.archive = self.test_root / "BioMotion.xcarchive"
        self.receipt = self.test_root / "BioMotion.xcarchive.dependency-receipt.json"
        self.lock = self.repository / "tools" / "dependencies.lock.json"
        self.inspector = (
            self.repository / "tools" / "release" / "dependency_boundary.py"
        )
        self.inspector_mode = self.inspector.parent / "inspector-mode.txt"
        self.observed_head = self.inspector.parent / "observed-head.txt"
        self.app = self.archive / "Products" / "Applications" / "BioMotion.app"
        self.executable = self.app / "BioMotion"
        self.make_fixture()
        self.expected_lock_digest = hashlib.sha256(self.lock.read_bytes()).hexdigest()
        self.expected_snapshot = self.capture_dependency_snapshot()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def make_fixture(self) -> None:
        self.lock.parent.mkdir(parents=True)
        lock = {
            "schemaVersion": 1,
            "dependencies": {
                "nimblephysics": {
                    "repository": "https://github.com/example/nimblephysics.git",
                    "commit": "1" * 40,
                    "sourceDirectory": "nimblephysics",
                    "builds": [],
                },
                "osqp": {
                    "repository": "https://github.com/osqp/osqp.git",
                    "commit": "2" * 40,
                    "sourceDirectory": "osqp",
                    "qdldlRepository": "https://github.com/osqp/qdldl.git",
                    "qdldlCommit": "3" * 40,
                    "builds": [],
                },
            },
        }
        self.lock.write_text(
            json.dumps(lock, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        self.write_fake_inspector()

        self.app.mkdir(parents=True)
        (self.archive / "Info.plist").write_bytes(
            plistlib.dumps(
                {
                    "ApplicationProperties": {
                        "ApplicationPath": "Applications/BioMotion.app",
                    },
                    "ArchiveVersion": 2,
                    "Name": "BioMotion",
                },
                fmt=plistlib.FMT_BINARY,
                sort_keys=True,
            )
        )
        (self.app / "Info.plist").write_bytes(
            plistlib.dumps(
                {
                    "CFBundleExecutable": "BioMotion",
                    "CFBundleIdentifier": "app.biomotion.fixture",
                    "CFBundleVersion": "1",
                },
                fmt=plistlib.FMT_BINARY,
                sort_keys=True,
            )
        )
        self.executable.write_bytes(b"fixture Mach-O bytes\x00\x01\x02")
        self.executable.chmod(0o755)
        resources = self.app / "AssetPack" / "Models"
        resources.mkdir(parents=True)
        (resources / "model.bin").write_bytes(b"reviewed model bytes")
        (self.app / "embedded.mobileprovision").write_bytes(b"profile bytes")
        dwarf = self.archive / "dSYMs" / "BioMotion.app.dSYM" / "Contents" / "Resources"
        dwarf.mkdir(parents=True)
        (dwarf / "symbols.bin").write_bytes(b"debug symbols")

    def write_fake_inspector(self) -> None:
        self.inspector.parent.mkdir(parents=True, exist_ok=True)
        self.inspector_mode.write_text("canonical\n", encoding="utf-8")
        self.observed_head.write_text("1" * 40 + "\n", encoding="utf-8")
        self.inspector.write_text(
            r'''#!/usr/bin/env python3
import hashlib
import json
import os
from pathlib import Path
import sys

for name, injected in {
    "DEVELOPER_DIR": "/unreviewed/Xcode.app/Contents/Developer",
    "GIT_DIR": "/unreviewed/git-dir",
    "GIT_WORK_TREE": "/unreviewed/work-tree",
    "PYTHONHOME": "/unreviewed/python-home",
    "PYTHONINSPECT": "1",
    "PYTHONPATH": "/unreviewed/python-path",
    "SDKROOT": "/unreviewed/sdk",
    "TOOLCHAINS": "unreviewed-toolchain",
}.items():
    if os.environ.get(name) == injected:
        print(f"injected environment reached inspector: {name}", file=sys.stderr)
        raise SystemExit(80)

if os.environ.get("HOME") != "/var/empty":
    print("dependency inspector did not receive hermetic HOME", file=sys.stderr)
    raise SystemExit(81)

if len(sys.argv) != 3 or sys.argv[1] != "snapshot":
    raise SystemExit(2)
root = Path(sys.argv[2])
script_root = Path(__file__).parent
mode = (script_root / "inspector-mode.txt").read_text(encoding="utf-8").strip()
if mode == "fail":
    print("fixture inspector failure", file=sys.stderr)
    raise SystemExit(9)

lock_path = root / "tools" / "dependencies.lock.json"
if lock_path.is_symlink() or not lock_path.is_file():
    print("fixture lock is linked or missing", file=sys.stderr)
    raise SystemExit(10)
lock_bytes = lock_path.read_bytes()

def reject_duplicates(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate key: {key}")
        value[key] = item
    return value

try:
    json.loads(lock_bytes.decode("utf-8"), object_pairs_hook=reject_duplicates)
except Exception as error:
    print(f"fixture lock invalid: {error}", file=sys.stderr)
    raise SystemExit(11)

common = {
    "CMAKE_BUILD_TYPE": "Release",
    "CMAKE_GENERATOR": "Ninja",
    "CMAKE_OSX_ARCHITECTURES": "arm64",
    "CMAKE_OSX_DEPLOYMENT_TARGET": "17.0",
    "CMAKE_SYSTEM_NAME": "iOS",
}

def nimble_cmake(sdk):
    return {
        **common,
        "CMAKE_HOME_DIRECTORY": "nimblephysics/ios",
        "CMAKE_OSX_SYSROOT": "iphoneos" if sdk == "device" else "iphonesimulator",
        "NIMBLE_IOS_HOST_PROBE": "OFF",
    }

def osqp_cmake(sdk):
    build = "build_ios" if sdk == "device" else "build_sim"
    return {
        **common,
        "CMAKE_C_FLAGS": (
            f"-ffile-prefix-map=osqp=osqp "
            f"-ffile-prefix-map=osqp/{build}/_deps/qdldl-src=qdldl"
        ),
        "CMAKE_C_FLAGS_RELEASE": "-O3 -DNDEBUG",
        "CMAKE_HOME_DIRECTORY": "osqp",
        "CMAKE_OSX_SYSROOT": "iphoneos" if sdk == "device" else "iphonesimulator",
        "OSQP_ALGEBRA_BACKEND": "builtin",
        "OSQP_ASAN": "OFF",
        "OSQP_BUILD_DEMO_EXE": "OFF",
        "OSQP_BUILD_SHARED_LIB": "OFF",
        "OSQP_BUILD_STATIC_LIB": "ON",
        "OSQP_BUILD_UNITTESTS": "OFF",
        "OSQP_CODEGEN": "ON",
        "OSQP_ENABLE_DERIVATIVES": "ON",
        "OSQP_ENABLE_INTERRUPT": "ON",
        "OSQP_ENABLE_PRINTING": "ON",
        "OSQP_ENABLE_PROFILING": "ON",
        "OSQP_PACK_SETTINGS": "OFF",
        "OSQP_PROFILER_ANNOTATIONS": "OFF",
        "OSQP_USE_FLOAT": "OFF",
        "OSQP_USE_LONG": "OFF",
        "QDLDL_BUILD_SHARED_LIB": "OFF",
        "QDLDL_BUILD_STATIC_LIB": "OFF",
        "QDLDL_DEV_ANALYSIS": "OFF",
        "QDLDL_DEV_ASAN": "OFF",
        "QDLDL_DEV_COVERAGE": "OFF",
        "QDLDL_FLOAT": "OFF",
        "QDLDL_LONG": "OFF",
    }

observed_head = (script_root / "observed-head.txt").read_text(encoding="utf-8").strip()
snapshot = {
    "dependencies": {
        "nimblephysics": {
            "builds": {
                sdk: {
                    "archiveSHA256": ("a" if sdk == "device" else "b") * 64,
                    "cmake": nimble_cmake(sdk),
                    "generatedHeaderSHA256": "c" * 64,
                }
                for sdk in ("device", "simulator")
            },
            "head": observed_head,
            "repository": "https://github.com/example/nimblephysics",
        },
        "osqp": {
            "builds": {
                sdk: {
                    "archiveContentSHA256": ("d" if sdk == "device" else "e") * 64,
                    "archiveMemberCount": 29,
                    "cmake": osqp_cmake(sdk),
                    "generatedHeaderSHA256": "f" * 64,
                }
                for sdk in ("device", "simulator")
            },
            "head": "2" * 40,
            "qdldl": {
                sdk: {
                    "head": "3" * 40,
                    "repository": "https://github.com/osqp/qdldl",
                }
                for sdk in ("device", "simulator")
            },
            "repository": "https://github.com/osqp/osqp",
        },
    },
    "dependencyLockSHA256": hashlib.sha256(lock_bytes).hexdigest(),
    "projectLinkage": {
        "normalizedLinkageSHA256": "4" * 64,
        "projectPBXProjSHA256": "5" * 64,
        "projectYMLSHA256": "6" * 64,
    },
    "schemaVersion": 1,
}
canonical = json.dumps(snapshot, ensure_ascii=True, separators=(",", ":"), sort_keys=True)
if mode == "noncanonical":
    print(json.dumps(snapshot, ensure_ascii=True, sort_keys=True))
elif mode == "multiline":
    print(canonical)
    print("unexpected second line")
elif mode == "stderr":
    print("unexpected stderr", file=sys.stderr)
    print(canonical)
else:
    print(canonical)
''',
            encoding="utf-8",
        )

    def capture_dependency_snapshot(self) -> str:
        result = subprocess.run(
            [PYTHON, "-I", str(self.inspector), "snapshot", str(self.repository)],
            check=False,
            capture_output=True,
            text=True,
            env={
                "HOME": "/var/empty",
                "PATH": "/usr/bin:/bin:/usr/sbin:/sbin",
                "LANG": "C",
                "LC_ALL": "C",
            },
            cwd=str(self.test_root),
        )
        if result.returncode != 0 or result.stderr or not result.stdout.endswith("\n"):
            raise RuntimeError(f"could not capture fixture snapshot: {result}")
        return result.stdout

    def run_cli(
        self,
        *arguments: str,
        extra_environment: dict[str, str] | None = None,
        stdin_text: str | None = None,
    ) -> subprocess.CompletedProcess:
        if arguments:
            command = [PYTHON, "-I", str(SCRIPT), *arguments]
        else:
            command = [PYTHON, "-I", str(SCRIPT)]
        if extra_environment:
            launcher = (
                "import json,os,runpy,sys\n"
                "script=sys.argv[1]\n"
                "os.environ.update(json.loads(sys.argv[2]))\n"
                "sys.argv=[script,*sys.argv[3:]]\n"
                "try:\n"
                "    runpy.run_path(script,run_name='__main__')\n"
                "finally:\n"
                "    os.environ.pop('PYTHONINSPECT',None)\n"
            )
            command = [
                PYTHON,
                "-I",
                "-c",
                launcher,
                str(SCRIPT),
                json.dumps(extra_environment, sort_keys=True),
                *arguments,
            ]
        environment = {
            "PATH": "/usr/bin:/bin:/usr/sbin:/sbin",
            "LANG": "C",
            "LC_ALL": "C",
            "TMPDIR": str(self.test_root),
        }
        return subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            input=stdin_text,
            env=environment,
            cwd=str(self.test_root),
        )

    def invoke(
        self,
        operation: str,
        repository: Path | None = None,
        archive: Path | None = None,
        receipt: Path | None = None,
        extra_environment: dict[str, str] | None = None,
        expected_snapshot: str | None = None,
        expected_lock_digest: str | None = None,
    ) -> subprocess.CompletedProcess:
        arguments = [
            operation,
            str(repository or self.repository),
            str(archive or self.archive),
            str(receipt or self.receipt),
        ]
        stdin_text = None
        if operation == "seal":
            arguments.append(expected_lock_digest or self.expected_lock_digest)
            stdin_text = (
                self.expected_snapshot
                if expected_snapshot is None
                else expected_snapshot
            )
        return self.run_cli(
            *arguments,
            extra_environment=extra_environment,
            stdin_text=stdin_text,
        )

    def assert_success(self, result: subprocess.CompletedProcess, marker: str) -> None:
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(marker, result.stdout)
        self.assertEqual(result.stderr, "")

    def assert_rejected(self, result: subprocess.CompletedProcess) -> None:
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("DEPENDENCY_ARCHIVE_RECEIPT_FAIL:", result.stderr)

    def seal(self) -> None:
        self.assert_success(
            self.invoke("seal"), "DEPENDENCY_ARCHIVE_RECEIPT_SEALED"
        )

    def verify(self) -> subprocess.CompletedProcess:
        return self.invoke("verify")

    def rewrite_receipt(self, value: dict) -> None:
        self.receipt.write_text(
            json.dumps(value, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        self.receipt.chmod(0o600)

    def test_validate_snapshot_accepts_only_the_expected_canonical_lock(self) -> None:
        result = self.run_cli(
            "validate-snapshot",
            str(self.repository),
            self.expected_lock_digest,
            stdin_text=self.expected_snapshot,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, self.expected_snapshot)
        self.assertEqual(result.stderr, "")

        mismatch = self.run_cli(
            "validate-snapshot",
            str(self.repository),
            "0" * 64,
            stdin_text=self.expected_snapshot,
        )
        self.assert_rejected(mismatch)

    def test_validate_snapshot_rejects_noncanonical_or_missing_input(self) -> None:
        value = json.loads(self.expected_snapshot)
        noncanonical = json.dumps(value, sort_keys=True) + "\n"
        self.assert_rejected(
            self.run_cli(
                "validate-snapshot",
                str(self.repository),
                self.expected_lock_digest,
                stdin_text=noncanonical,
            )
        )
        self.assert_rejected(
            self.run_cli(
                "validate-snapshot",
                str(self.repository),
                self.expected_lock_digest,
                stdin_text="",
            )
        )

    def test_seal_rejects_expected_snapshot_that_differs_from_observation(self) -> None:
        value = json.loads(self.expected_snapshot)
        value["dependencies"]["nimblephysics"]["head"] = "7" * 40
        changed = json.dumps(
            value, ensure_ascii=True, separators=(",", ":"), sort_keys=True
        ) + "\n"
        self.assert_rejected(
            self.invoke("seal", expected_snapshot=changed)
        )
        self.assertFalse(self.receipt.exists())

    def test_seal_requires_an_explicit_expected_snapshot(self) -> None:
        self.assert_rejected(self.invoke("seal", expected_snapshot=""))
        self.assertFalse(self.receipt.exists())

    def test_seal_and_verify_are_stable_private_and_path_independent(self) -> None:
        self.seal()
        self.assertEqual(stat.S_IMODE(self.receipt.stat().st_mode), 0o600)
        receipt_bytes = self.receipt.read_bytes()
        value = json.loads(receipt_bytes)
        self.assertEqual(
            receipt_bytes,
            (json.dumps(value, ensure_ascii=True, indent=2, sort_keys=True) + "\n").encode(),
        )
        self.assertNotIn(str(self.test_root), receipt_bytes.decode("utf-8"))
        self.assertEqual(value["schemaVersion"], 1)
        snapshot = value["dependencySnapshot"]
        self.assertEqual(snapshot["schemaVersion"], 1)
        self.assertEqual(
            snapshot["dependencies"]["nimblephysics"]["head"], "1" * 40
        )
        self.assertEqual(
            snapshot["dependencies"]["osqp"]["builds"]["device"]
            ["archiveMemberCount"],
            29,
        )
        self.assertEqual(
            value["archive"]["application"]["path"],
            "Products/Applications/BioMotion.app",
        )
        self.assert_success(self.verify(), "DEPENDENCY_ARCHIVE_RECEIPT_PASS")

        moved_archive = self.test_root / "Moved.xcarchive"
        moved_receipt = self.test_root / "Moved.dependency-receipt.json"
        self.archive.rename(moved_archive)
        self.receipt.rename(moved_receipt)
        result = self.invoke("verify", archive=moved_archive, receipt=moved_receipt)
        self.assert_success(result, "DEPENDENCY_ARCHIVE_RECEIPT_PASS")

    def test_app_resource_mutation_is_rejected(self) -> None:
        self.seal()
        (self.app / "AssetPack" / "Models" / "model.bin").write_bytes(b"changed")
        self.assert_rejected(self.verify())

    def test_executable_mutation_is_rejected(self) -> None:
        self.seal()
        self.executable.write_bytes(b"different executable")
        self.assert_rejected(self.verify())

    def test_dependency_lock_byte_mutation_is_rejected(self) -> None:
        self.seal()
        with self.lock.open("ab") as stream:
            stream.write(b"\n")
        self.assert_rejected(self.verify())

    def test_observed_dependency_snapshot_mutation_is_rejected(self) -> None:
        self.seal()
        self.observed_head.write_text("7" * 40 + "\n", encoding="utf-8")
        self.assert_rejected(self.verify())

    def test_stored_dependency_snapshot_mutation_is_rejected(self) -> None:
        self.seal()
        value = json.loads(self.receipt.read_text(encoding="utf-8"))
        value["dependencySnapshot"]["dependencies"]["nimblephysics"]["head"] = (
            "7" * 40
        )
        self.rewrite_receipt(value)
        self.assert_rejected(self.verify())

    def test_archive_info_plist_mutation_is_rejected(self) -> None:
        self.seal()
        info = self.archive / "Info.plist"
        value = plistlib.loads(info.read_bytes())
        value["Name"] = "Changed"
        info.write_bytes(plistlib.dumps(value, fmt=plistlib.FMT_BINARY))
        self.assert_rejected(self.verify())

    def test_app_info_plist_mutation_is_rejected(self) -> None:
        self.seal()
        info = self.app / "Info.plist"
        value = plistlib.loads(info.read_bytes())
        value["CFBundleVersion"] = "2"
        info.write_bytes(plistlib.dumps(value, fmt=plistlib.FMT_BINARY))
        self.assert_rejected(self.verify())

    def test_file_outside_application_mutation_is_rejected(self) -> None:
        self.seal()
        symbols = (
            self.archive
            / "dSYMs"
            / "BioMotion.app.dSYM"
            / "Contents"
            / "Resources"
            / "symbols.bin"
        )
        symbols.write_bytes(b"changed debug symbols")
        self.assert_rejected(self.verify())

    def test_receipt_whitespace_mutation_is_rejected(self) -> None:
        self.seal()
        with self.receipt.open("ab") as stream:
            stream.write(b" ")
        self.assert_rejected(self.verify())

    def test_receipt_value_mutation_is_rejected(self) -> None:
        self.seal()
        value = json.loads(self.receipt.read_text(encoding="utf-8"))
        value["archive"]["application"]["executable"]["sha256"] = "0" * 64
        self.rewrite_receipt(value)
        self.assert_rejected(self.verify())

    def test_duplicate_receipt_key_is_rejected(self) -> None:
        self.seal()
        text = self.receipt.read_text(encoding="utf-8")
        text = text.replace(
            '  "schemaVersion": 1\n',
            '  "schemaVersion": 1,\n  "schemaVersion": 1\n',
            1,
        )
        self.receipt.write_text(text, encoding="utf-8")
        self.receipt.chmod(0o600)
        self.assert_rejected(self.verify())

    def test_extra_receipt_key_is_rejected(self) -> None:
        self.seal()
        value = json.loads(self.receipt.read_text(encoding="utf-8"))
        value["unexpected"] = "not reviewed"
        self.rewrite_receipt(value)
        self.assert_rejected(self.verify())

    def test_boolean_receipt_schema_is_rejected(self) -> None:
        self.seal()
        value = json.loads(self.receipt.read_text(encoding="utf-8"))
        value["schemaVersion"] = True
        self.rewrite_receipt(value)
        self.assert_rejected(self.verify())

    def test_wrong_receipt_field_type_is_rejected(self) -> None:
        self.seal()
        value = json.loads(self.receipt.read_text(encoding="utf-8"))
        value["dependencySnapshot"]["dependencyLockSHA256"] = True
        self.rewrite_receipt(value)
        self.assert_rejected(self.verify())

    def test_failing_dependency_inspector_is_rejected(self) -> None:
        self.inspector_mode.write_text("fail\n", encoding="utf-8")
        self.assert_rejected(self.invoke("seal"))

    def test_missing_dependency_inspector_is_rejected(self) -> None:
        self.inspector.unlink()
        self.assert_rejected(self.invoke("seal"))

    def test_symlink_dependency_inspector_is_rejected(self) -> None:
        real_inspector = self.test_root / "real-inspector.py"
        self.inspector.rename(real_inspector)
        self.inspector.symlink_to(real_inspector)
        self.assert_rejected(self.invoke("seal"))

    def test_noncanonical_dependency_snapshot_is_rejected(self) -> None:
        self.inspector_mode.write_text("noncanonical\n", encoding="utf-8")
        self.assert_rejected(self.invoke("seal"))

    def test_multiline_dependency_snapshot_is_rejected(self) -> None:
        self.inspector_mode.write_text("multiline\n", encoding="utf-8")
        self.assert_rejected(self.invoke("seal"))

    def test_successful_inspector_stderr_is_rejected(self) -> None:
        self.inspector_mode.write_text("stderr\n", encoding="utf-8")
        self.assert_rejected(self.invoke("seal"))

    def test_dependency_inspector_environment_is_fixed(self) -> None:
        injected = {
            "DEVELOPER_DIR": "/unreviewed/Xcode.app/Contents/Developer",
            "GIT_DIR": "/unreviewed/git-dir",
            "GIT_WORK_TREE": "/unreviewed/work-tree",
            "PYTHONHOME": "/unreviewed/python-home",
            "PYTHONINSPECT": "1",
            "PYTHONPATH": "/unreviewed/python-path",
            "SDKROOT": "/unreviewed/sdk",
            "TOOLCHAINS": "unreviewed-toolchain",
        }
        result = self.invoke("seal", extra_environment=injected)
        self.assert_success(result, "DEPENDENCY_ARCHIVE_RECEIPT_SEALED")
        self.assert_success(
            self.invoke("verify", extra_environment=injected),
            "DEPENDENCY_ARCHIVE_RECEIPT_PASS",
        )

    def test_archive_symlink_is_rejected(self) -> None:
        real_archive = self.test_root / "Real.xcarchive"
        self.archive.rename(real_archive)
        self.archive.symlink_to(real_archive, target_is_directory=True)
        self.assert_rejected(self.invoke("seal"))

    def test_symlink_inside_archive_is_rejected_during_seal(self) -> None:
        model = self.app / "AssetPack" / "Models" / "model.bin"
        external = self.test_root / "external.bin"
        external.write_bytes(model.read_bytes())
        model.unlink()
        model.symlink_to(external)
        self.assert_rejected(self.invoke("seal"))

    def test_symlink_inside_archive_is_rejected_during_verify(self) -> None:
        self.seal()
        model = self.app / "AssetPack" / "Models" / "model.bin"
        external = self.test_root / "external.bin"
        external.write_bytes(model.read_bytes())
        model.unlink()
        model.symlink_to(external)
        self.assert_rejected(self.verify())

    def test_symlink_receipt_is_rejected(self) -> None:
        self.seal()
        real_receipt = self.test_root / "real-receipt.json"
        self.receipt.rename(real_receipt)
        self.receipt.symlink_to(real_receipt)
        self.assert_rejected(self.verify())

    def test_symlink_dependency_lock_is_rejected(self) -> None:
        real_lock = self.test_root / "real-lock.json"
        self.lock.rename(real_lock)
        self.lock.symlink_to(real_lock)
        self.assert_rejected(self.invoke("seal"))

    def test_special_file_inside_archive_is_rejected_during_seal(self) -> None:
        os.mkfifo(self.app / "unreviewed.fifo")
        self.assert_rejected(self.invoke("seal"))

    def test_special_file_inside_archive_is_rejected_during_verify(self) -> None:
        self.seal()
        os.mkfifo(self.archive / "unreviewed.fifo")
        self.assert_rejected(self.verify())

    def test_existing_receipt_is_never_overwritten(self) -> None:
        sentinel = b"do not overwrite\n"
        self.receipt.write_bytes(sentinel)
        result = self.invoke("seal")
        self.assert_rejected(result)
        self.assertEqual(self.receipt.read_bytes(), sentinel)

    def test_unsafe_bundle_executable_path_is_rejected(self) -> None:
        info = self.app / "Info.plist"
        value = plistlib.loads(info.read_bytes())
        value["CFBundleExecutable"] = "../BioMotion"
        info.write_bytes(plistlib.dumps(value, fmt=plistlib.FMT_BINARY))
        self.assert_rejected(self.invoke("seal"))

    def test_duplicate_dependency_lock_key_is_rejected(self) -> None:
        text = self.lock.read_text(encoding="utf-8")
        text = text.replace(
            '  "schemaVersion": 1\n',
            '  "schemaVersion": 1,\n  "schemaVersion": 1\n',
            1,
        )
        self.lock.write_text(text, encoding="utf-8")
        self.assert_rejected(self.invoke("seal"))

    def test_receipt_inside_archive_is_rejected(self) -> None:
        inside = self.archive / "receipt.json"
        self.assert_rejected(self.invoke("seal", receipt=inside))

    def test_bad_command_lines_return_usage_error(self) -> None:
        for arguments in (
            (),
            ("unknown", str(self.repository), str(self.archive), str(self.receipt)),
            ("seal", str(self.repository), str(self.archive)),
            (
                "verify",
                str(self.repository),
                str(self.archive),
                str(self.receipt),
                "extra",
            ),
        ):
            with self.subTest(arguments=arguments):
                result = self.run_cli(*arguments)
                self.assertEqual(result.returncode, 2, result)
                self.assertIn("usage:", result.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
