#!/usr/bin/env python3
"""Small deterministic xcrun double for the asset-pack package transaction tests."""
# @(#)PROGRAM:coremlcompiler  PROJECT:CoreML-fixture

from __future__ import annotations

import base64
import json
import os
from pathlib import Path
import shutil
import stat
import sys


def fail(message: str) -> None:
    print(f"fake-xcrun: {message}", file=sys.stderr)
    raise SystemExit(1)


def log(arguments: list[str]) -> None:
    path = os.environ.get("BIOMOTION_FAKE_XCRUN_LOG")
    if path:
        with Path(path).open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(arguments, separators=(",", ":")) + "\n")


def copy_selector(source: Path, destination: Path) -> None:
    if source.is_symlink():
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.symlink_to(os.readlink(source))
    elif source.is_dir():
        shutil.copytree(source, destination, symlinks=True)
    elif source.is_file():
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    else:
        fail(f"selector source is missing or unsupported: {source}")


def package(arguments: list[str]) -> None:
    if arguments == ["--version"]:
        print("fixture")
        return
    if os.environ.get("BIOMOTION_FAKE_XCRUN_FAULT") == "package":
        fail("injected ba-package failure")
    if arguments and arguments[0] == "package":
        arguments = arguments[1:]
    if len(arguments) != 3 or arguments[1] != "-o":
        fail(f"unexpected ba-package arguments: {arguments!r}")
    manifest_path = Path(arguments[0])
    output = Path(arguments[2])
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    snapshot = Path(f"{output}.fixture-contents")
    if snapshot.exists():
        shutil.rmtree(snapshot)
    (snapshot / "Contents").mkdir(parents=True)
    shutil.copy2(manifest_path, snapshot / "Manifest.json")
    for selector in manifest["fileSelectors"]:
        key, relative = next(iter(selector.items()))
        if key not in {"directory", "file"}:
            fail(f"unexpected selector: {selector!r}")
        copy_selector(Path.cwd() / relative, snapshot / "Contents" / relative)

    fault = os.environ.get("BIOMOTION_FAKE_XCRUN_FAULT", "")
    if fault == "extra":
        (snapshot / "Contents/unlocked.bin").write_bytes(b"extra\n")
    elif fault == "missing":
        (snapshot / "Contents/SAM-LICENSE.txt").unlink()
    elif fault == "symlink":
        license_path = snapshot / "Contents/SAM-LICENSE.txt"
        license_path.unlink()
        license_path.symlink_to("../Manifest.json")
    elif fault == "hash":
        model_path = snapshot / "Contents/SAM3DBodyPose.mlmodelc/model.mil"
        model_path.write_bytes(model_path.read_bytes() + b"drift")

    archive_entries = []
    for row in archive_rows(snapshot):
        entry = {"type": row["TYP"], "path": row["PAT"]}
        path = snapshot / row["PAT"] if row["PAT"] else snapshot
        if row["TYP"] == "F":
            entry["data"] = base64.b64encode(path.read_bytes()).decode("ascii")
        elif row["TYP"] == "L":
            entry["link"] = os.readlink(path)
        archive_entries.append(entry)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps({"entries": archive_entries}, separators=(",", ":")),
        encoding="utf-8",
    )
    shutil.rmtree(snapshot)


def archive_rows(root: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = [{"TYP": "D", "PAT": ""}]
    for current, directory_names, file_names in os.walk(
        root, topdown=True, followlinks=False
    ):
        current_path = Path(current)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            path = current_path / name
            relative = path.relative_to(root).as_posix()
            mode = os.lstat(path).st_mode
            if stat.S_ISLNK(mode):
                rows.append({"TYP": "L", "PAT": relative, "LNK": os.readlink(path)})
            else:
                rows.append({"TYP": "D", "PAT": relative})
        for name in file_names:
            path = current_path / name
            relative = path.relative_to(root).as_posix()
            mode = os.lstat(path).st_mode
            if stat.S_ISLNK(mode):
                rows.append({"TYP": "L", "PAT": relative, "LNK": os.readlink(path)})
            else:
                rows.append({"TYP": "F", "PAT": relative, "DAT": path.stat().st_size})
    return rows


def aa(arguments: list[str]) -> None:
    if not arguments:
        fail("missing aa operation")
    operation = arguments[0]
    if operation == "list":
        try:
            archive = Path(arguments[arguments.index("-i") + 1])
        except (ValueError, IndexError):
            fail(f"unexpected aa list arguments: {arguments!r}")
        document = json.loads(archive.read_text(encoding="utf-8"))
        rows = []
        for entry in document["entries"]:
            row = {"TYP": entry["type"], "PAT": entry["path"]}
            if entry["type"] == "F":
                row["DAT"] = len(base64.b64decode(entry["data"], validate=True))
            elif entry["type"] == "L":
                row["LNK"] = entry["link"]
            rows.append(row)
        if os.environ.get("BIOMOTION_FAKE_XCRUN_FAULT") == "path":
            rows.append({"TYP": "F", "PAT": "../escape", "DAT": 1})
        print(json.dumps(rows, separators=(",", ":")))
        return
    if operation == "extract":
        try:
            archive = Path(arguments[arguments.index("-i") + 1])
            destination = Path(arguments[arguments.index("-d") + 1])
        except (ValueError, IndexError):
            fail(f"unexpected aa extract arguments: {arguments!r}")
        document = json.loads(archive.read_text(encoding="utf-8"))
        for entry in document["entries"]:
            relative = entry["path"]
            path = destination / relative if relative else destination
            if entry["type"] == "D":
                path.mkdir(parents=True, exist_ok=True)
            elif entry["type"] == "F":
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(base64.b64decode(entry["data"], validate=True))
            elif entry["type"] == "L":
                path.parent.mkdir(parents=True, exist_ok=True)
                path.symlink_to(entry["link"])
            else:
                fail(f"unsupported fake archive entry: {entry!r}")
        return
    fail(f"unsupported aa operation: {operation}")


def coremlcompiler(arguments: list[str]) -> None:
    compiled_fixture = os.environ.get("BIOMOTION_FAKE_COMPILED_MODEL")
    if not compiled_fixture:
        fail("BIOMOTION_FAKE_COMPILED_MODEL is required")
    fixture = Path(compiled_fixture)
    if not arguments:
        fail("missing coremlcompiler operation")
    if arguments[0] == "metadata" and len(arguments) == 2:
        print((fixture / "metadata.json").read_text(encoding="utf-8"), end="")
        return
    if arguments[0] == "compile" and len(arguments) == 3:
        if os.environ.get("BIOMOTION_FAKE_XCRUN_FAULT") == "compile":
            fail("injected coremlcompiler failure")
        output_root = Path(arguments[2])
        shutil.copytree(fixture, output_root / fixture.name, symlinks=True)
        return
    fail(f"unsupported coremlcompiler arguments: {arguments!r}")


def main() -> None:
    arguments = sys.argv[1:]
    log(arguments)
    if not arguments:
        fail("missing tool")
    if arguments == ["--find", "coremlcompiler"]:
        print(Path(__file__).resolve())
        return
    tool, rest = arguments[0], arguments[1:]
    if tool == "coremlcompiler":
        coremlcompiler(rest)
    elif tool == "ba-package":
        package(rest)
    elif tool == "aa":
        aa(rest)
    else:
        fail(f"unsupported tool: {tool}")


if __name__ == "__main__":
    main()
