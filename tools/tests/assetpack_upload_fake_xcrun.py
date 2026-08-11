#!/usr/bin/env python3
"""Hermetic xcrun/altool double for asset-pack upload gate tests."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import stat
import sys


def append_event(message: str) -> None:
    with Path(os.environ["FAKE_EVENT_LOG"]).open("a", encoding="utf-8") as stream:
        stream.write(f"{message}\n")


def main() -> int:
    arguments = sys.argv[1:]
    if any(
        os.environ.get(f"FAKE_UNTRUSTED_{name}")
        and os.environ.get(name) == os.environ[f"FAKE_UNTRUSTED_{name}"]
        for name in ("DEVELOPER_DIR", "TOOLCHAINS", "SDKROOT")
    ):
        append_event("xcrun:poisoned-xcode-environment")
        return 91
    marker_path = Path(os.environ["FAKE_VERIFIER_MARKER"])
    if not marker_path.is_file():
        append_event("xcrun:before-verification")
        return 96
    marker = json.loads(marker_path.read_text(encoding="utf-8"))

    if len(arguments) < 2 or arguments[0] != "altool":
        append_event("xcrun:bad-argv")
        return 95

    operation = arguments[1]
    if operation == "--upload-asset-pack":
        event = "upload"
        requested_exit = int(os.environ.get("FAKE_UPLOAD_EXIT", "0"))
        uploaded_aar = Path(arguments[2]) if len(arguments) > 2 else None
        if (
            uploaded_aar is None
            or str(uploaded_aar) != marker["aar"]
            or hashlib.sha256(uploaded_aar.read_bytes()).hexdigest()
            != marker["sha256"]
        ):
            append_event("xcrun:upload:not-verified-snapshot")
            return 92
    elif operation == "--list-asset-pack-versions":
        event = "list"
        requested_exit = int(os.environ.get("FAKE_LIST_EXIT", "0"))
    else:
        append_event("xcrun:unexpected-operation")
        return 94

    key_id = os.environ["ASC_API_KEY_ID"]
    key_path = (
        Path(os.environ["HOME"])
        / ".appstoreconnect/private_keys"
        / f"AuthKey_{key_id}.p8"
    )
    key_lstat = key_path.lstat()
    if stat.S_ISLNK(key_lstat.st_mode) or not stat.S_ISREG(key_lstat.st_mode):
        append_event(f"xcrun:{event}:bad-key")
        return 93

    with Path(os.environ["FAKE_XCRUN_LOG"]).open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(arguments) + "\n")
    append_event(f"xcrun:{event}")
    return requested_exit


if __name__ == "__main__":
    raise SystemExit(main())
