#!/usr/bin/env python3
"""Hermetic receipt-verifier double for asset-pack upload gate tests."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import stat
import sys


def append_event(message: str) -> None:
    event_log = os.environ.get("FAKE_EVENT_LOG")
    if event_log:
        with Path(event_log).open("a", encoding="utf-8") as stream:
            stream.write(f"{message}\n")


def main() -> int:
    arguments = sys.argv[1:]
    append_event("verifier:start")

    poisoned_xcode_environment = [
        name
        for name in ("DEVELOPER_DIR", "TOOLCHAINS", "SDKROOT")
        if os.environ.get(f"FAKE_UNTRUSTED_{name}")
        and os.environ.get(name) == os.environ[f"FAKE_UNTRUSTED_{name}"]
    ]
    if poisoned_xcode_environment:
        append_event("verifier:poisoned-xcode-environment")
        print(
            f"fake verifier inherited Xcode selection: {poisoned_xcode_environment}",
            file=sys.stderr,
        )
        return 91

    log_path = Path(os.environ["FAKE_VERIFIER_LOG"])
    log_path.write_text(json.dumps(arguments) + "\n", encoding="utf-8")

    source_aar = Path(os.environ["FAKE_EXPECTED_AAR"])
    source_receipt = Path(os.environ["FAKE_EXPECTED_RECEIPT"])
    expected_snapshot = os.environ.get("FAKE_EXPECT_SNAPSHOT") == "1"
    valid_arguments = len(arguments) == 3 and arguments[0] == "receipt"
    if valid_arguments:
        verified_aar = Path(arguments[1])
        verified_receipt = Path(arguments[2])
        if expected_snapshot:
            directory_mode = stat.S_IMODE(verified_aar.parent.stat().st_mode)
            valid_arguments = (
                verified_aar != source_aar
                and verified_receipt != source_receipt
                and verified_aar.parent == verified_receipt.parent
                and verified_aar.name == "sam3d-body-pose.aar"
                and verified_receipt.name
                == "sam3d-body-pose.aar.receipt.json"
                and directory_mode == 0o700
                and stat.S_ISREG(verified_aar.lstat().st_mode)
                and not stat.S_ISLNK(verified_aar.lstat().st_mode)
                and stat.S_IMODE(verified_aar.lstat().st_mode) == 0o600
                and stat.S_ISREG(verified_receipt.lstat().st_mode)
                and not stat.S_ISLNK(verified_receipt.lstat().st_mode)
                and stat.S_IMODE(verified_receipt.lstat().st_mode) == 0o600
                and verified_aar.read_bytes() == source_aar.read_bytes()
                and verified_receipt.read_bytes() == source_receipt.read_bytes()
            )
        else:
            valid_arguments = (
                verified_aar == source_aar and verified_receipt == source_receipt
            )

    if not valid_arguments:
        append_event("verifier:bad-argv")
        print(
            f"fake verifier rejected receipt argv: {arguments!r}",
            file=sys.stderr,
        )
        return 97

    requested_exit = int(os.environ.get("FAKE_VERIFIER_EXIT", "0"))
    if requested_exit:
        append_event("verifier:fail")
        print("FAKE_RECEIPT_VERIFY_FAIL", file=sys.stderr)
        return requested_exit

    if os.environ.get("FAKE_CREATE_KEY") == "1":
        key_id = os.environ["ASC_API_KEY_ID"]
        key_directory = Path(os.environ["HOME"]) / ".appstoreconnect/private_keys"
        key_directory.mkdir(parents=True, exist_ok=True)
        key_path = key_directory / f"AuthKey_{key_id}.p8"
        key_path.write_text("fixture-private-key\n", encoding="utf-8")

    verified_digest = hashlib.sha256(verified_aar.read_bytes()).hexdigest()
    if os.environ.get("FAKE_MUTATE_SOURCE_AFTER_VERIFY") == "1":
        source_aar.write_bytes(b"replacement generation\n")

    marker_path = os.environ.get("FAKE_VERIFIER_MARKER")
    if marker_path:
        Path(marker_path).write_text(
            json.dumps({"aar": str(verified_aar), "sha256": verified_digest}) + "\n",
            encoding="utf-8",
        )
    append_event("verifier:pass")
    print("MODEL_LOCK_VERIFY_PASS mode=receipt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
