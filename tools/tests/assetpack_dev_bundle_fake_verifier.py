#!/usr/bin/python3
"""Tiny receipt verifier fixture for dev_bundle_model.sh transaction tests."""

from __future__ import annotations

import argparse
import ctypes
import json
import os
from pathlib import Path
import stat
import sys


def regular_file(path: Path, label: str) -> None:
    try:
        entry = os.lstat(path)
    except OSError as error:
        raise SystemExit(f"fake verifier cannot inspect {label}: {error}")
    if stat.S_ISLNK(entry.st_mode) or not stat.S_ISREG(entry.st_mode):
        raise SystemExit(f"fake verifier requires regular {label}")


def swap_live_release_if_requested() -> bool:
    live_value = os.environ.get("BIOMOTION_FAKE_LIVE_RELEASE")
    next_value = os.environ.get("BIOMOTION_FAKE_NEXT_RELEASE")
    if live_value is None and next_value is None:
        return False
    if live_value is None or next_value is None:
        raise SystemExit("fake release swap requires both test directories")

    live = Path(live_value)
    next_release = Path(next_value)
    library = ctypes.CDLL(None, use_errno=True)
    rename = library.renameatx_np
    rename.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    ]
    rename.restype = ctypes.c_int
    if rename(-2, os.fsencode(live), -2, os.fsencode(next_release), 0x2) != 0:
        error_number = ctypes.get_errno()
        raise SystemExit(
            f"fake verifier could not swap live release: "
            f"{os.strerror(error_number)}"
        )
    return True


def fixture_generation(value: str, prefix: str, label: str) -> str:
    if value == prefix:
        return "default"
    marker = f"{prefix}:"
    if value.startswith(marker) and value[len(marker) :]:
        return value[len(marker) :]
    raise SystemExit(f"fixture {label} verification failed")


parser = argparse.ArgumentParser()
parser.add_argument("mode")
parser.add_argument("--lock", required=True, type=Path)
parser.add_argument("--license", required=True, type=Path)
parser.add_argument("--manifest", required=True, type=Path)
parser.add_argument("--extract-directory", required=True, type=Path)
parser.add_argument("aar", type=Path)
parser.add_argument("receipt", type=Path)
args = parser.parse_args()

if args.mode != "receipt":
    raise SystemExit("fake verifier accepts only receipt mode")
if sys.flags.isolated != 1:
    raise SystemExit("dev bundle verifier was not launched with python -I")
for value, label in (
    (args.lock, "lock"),
    (args.license, "license"),
    (args.manifest, "manifest"),
    (args.aar, "AAR"),
    (args.receipt, "receipt"),
):
    regular_file(value, label)

source_swapped = swap_live_release_if_requested()
receipt_generation = fixture_generation(
    args.receipt.read_text(encoding="utf-8").strip(),
    "valid-receipt",
    "receipt",
)
aar_mode = args.aar.read_text(encoding="utf-8").strip()
if aar_mode in {"symlink-tree", "special-tree"}:
    aar_generation = "default"
else:
    aar_generation = fixture_generation(aar_mode, "valid-aar", "AAR")
if aar_generation != receipt_generation:
    raise SystemExit("fixture AAR and receipt belong to different generations")

if args.extract_directory.exists() or args.extract_directory.is_symlink():
    raise SystemExit("fake verifier extraction destination already exists")
parent_mode = stat.S_IMODE(os.lstat(args.extract_directory.parent).st_mode)
if parent_mode != 0o700:
    raise SystemExit(
        f"fake verifier extraction parent mode is {parent_mode:o}, expected 700"
    )

model = args.extract_directory / "Contents/SAM3DBodyPose.mlmodelc"
model.mkdir(parents=True, mode=0o700)
if aar_mode == "symlink-tree":
    target = args.extract_directory.parent / "outside-model-data"
    target.write_bytes(b"outside\n")
    (model / "coremldata.bin").symlink_to(target)
elif aar_mode == "special-tree":
    os.mkfifo(model / "coremldata.bin", 0o600)
else:
    payload = "verified compiled model"
    if aar_generation != "default":
        payload += f" {aar_generation}"
    (model / "coremldata.bin").write_text(payload + "\n", encoding="utf-8")
    (model / "metadata.json").write_text("[]\n", encoding="utf-8")

repository_root = Path(__file__).resolve().parents[2]
log_path = repository_root / "fake-verifier.log"
log_path.write_text(
    json.dumps(
        {
            "isolated": sys.flags.isolated,
            "mode": args.mode,
            "aar": str(args.aar),
            "receipt": str(args.receipt),
            "extractDirectory": str(args.extract_directory),
            "transactionMode": f"{parent_mode:o}",
            "generation": aar_generation,
            "sourceSwapped": source_swapped,
        },
        sort_keys=True,
    )
    + "\n",
    encoding="utf-8",
)
