#!/bin/bash
# Prove the iOS runtime accepts only a precompiled SAM Core ML directory.
set -euo pipefail

PATH="/usr/bin:/bin:/usr/sbin:/sbin"
export PATH

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
STORE="$REPO_ROOT/BioMotion/AssetPack/AssetPackModelStore.swift"
PYTHON3="/usr/bin/python3"

"$PYTHON3" -I - "$STORE" <<'PY'
from pathlib import Path
import re
import sys


class ContractError(RuntimeError):
    pass


def strip_swift_comments(source: str) -> str:
    """Remove nested Swift comments while preserving strings and line numbers."""

    if '"""' in source or re.search(r'#+"', source):
        raise ContractError("probe does not silently approximate Swift raw/multiline strings")
    output: list[str] = []
    index = 0
    state = "code"
    block_depth = 0
    while index < len(source):
        current = source[index]
        following = source[index + 1] if index + 1 < len(source) else ""
        if state == "code":
            if current == "/" and following == "/":
                output.extend((" ", " "))
                index += 2
                state = "line_comment"
                continue
            if current == "/" and following == "*":
                output.extend((" ", " "))
                index += 2
                state = "block_comment"
                block_depth = 1
                continue
            if current == '"':
                output.append(current)
                index += 1
                state = "string"
                continue
            output.append(current)
            index += 1
            continue
        if state == "line_comment":
            if current == "\n":
                output.append("\n")
                state = "code"
            else:
                output.append(" ")
            index += 1
            continue
        if state == "block_comment":
            if current == "/" and following == "*":
                output.extend((" ", " "))
                index += 2
                block_depth += 1
                continue
            if current == "*" and following == "/":
                output.extend((" ", " "))
                index += 2
                block_depth -= 1
                if block_depth == 0:
                    state = "code"
                continue
            output.append("\n" if current == "\n" else " ")
            index += 1
            continue
        output.append(current)
        index += 1
        if current == "\\" and index < len(source):
            output.append(source[index])
            index += 1
        elif current == '"':
            state = "code"

    if state == "block_comment":
        raise ContractError("unterminated Swift block comment")
    if state == "string":
        raise ContractError("unterminated Swift string")
    return "".join(output)


path = Path(sys.argv[1])
source = path.read_text(encoding="utf-8")
active = strip_swift_comments(source)

forbidden = {
    "import CoreML": "runtime no longer needs the CoreML compiler API",
    "MLModel.compileModel": "runtime must never compile a raw model package",
    "applicationSupportDirectory": "runtime must not own a compiled-model cache",
    "CompiledModels": "runtime must not persist a compiled-model cache",
    "sourceStamp": "runtime must not use source size/mtime cache stamps",
    ".modificationDate": "runtime must not inspect model modification time",
    "sourceModelFileName": "raw model packages must not be runtime candidates",
    "bundledPackage": "app-bundle raw packages must not be accepted",
    "assetPackPackage": "asset-pack raw packages must not be accepted",
    "compiledCopy(ofPackageAt": "raw packages must not enter a compiler helper",
    'withExtension: "mlpackage"': "raw app-bundle packages must not be probed",
    'hasSuffix(".mlpackage")': "raw asset-pack packages must not be probed",
    '"SAM3DBodyPose.mlpackage"': "the runtime must name only the compiled model",
    "completedUnitCount": "Background Assets Progress units are not documented as bytes",
    "totalUnitCount": "Background Assets Progress units are not documented as bytes",
}
violations = [
    f"{token}: {reason}" for token, reason in forbidden.items() if token in active
]
if violations:
    raise ContractError("active raw-model runtime paths remain:\n  " + "\n  ".join(violations))

compiled_constant = re.findall(
    r'compiledModelFileName\s*=\s*"SAM3DBodyPose\.mlmodelc"', active
)
if len(compiled_constant) != 1:
    raise ContractError(
        "expected exactly one locked SAM3DBodyPose.mlmodelc runtime identity"
    )

for required in (
    "bundledCompiledModelURL()",
    "assetPackURL()",
    'compiledModelInteriorFileName = "coremldata.bin"',
    'FilePath(leafPath)',
    "deletingLastPathComponent()",
    'withExtension: "mlmodelc"',
    "case bundledCompiled",
    "case assetPackCompiled",
):
    if required not in active:
        raise ContractError(f"compiled-only runtime surface lost {required!r}")

print("ASSETPACK_RUNTIME_PRECOMPILED_ONLY_PASS")
PY
