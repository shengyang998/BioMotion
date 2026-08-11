#!/usr/bin/env python3
"""Verify unreachable no-Assimp iOS methods share the tested rejection path."""

from __future__ import annotations

import re
import sys
from collections import Counter
from pathlib import Path


CONTRACTS = (
    (
        "throwMeshShapeUnavailable",
        {
            "SharedMeshWrapper::SharedMeshWrapper": 1,
            "MeshShape::MeshShape": 2,
            "MeshShape::getVertices": 1,
            "MeshShape::getMesh": 1,
            "MeshShape::getMeshUri": 1,
            "MeshShape::getMeshUri2": 1,
            "MeshShape::update": 1,
            "MeshShape::getMeshPath": 1,
            "MeshShape::getResourceRetriever": 1,
            "MeshShape::setMesh": 2,
            "MeshShape::setScale": 1,
            "MeshShape::getScale": 1,
            "MeshShape::setColorMode": 1,
            "MeshShape::getColorMode": 1,
            "MeshShape::setAlphaMode": 1,
            "MeshShape::getAlphaMode": 1,
            "MeshShape::setColorIndex": 1,
            "MeshShape::getColorIndex": 1,
            "MeshShape::getDisplayList": 1,
            "MeshShape::setDisplayList": 1,
            "MeshShape::loadMesh": 3,
            "MeshShape::computeInertia": 1,
            "MeshShape::clone": 1,
            "MeshShape::updateBoundingBox": 1,
            "MeshShape::updateVolume": 1,
        },
    ),
    (
        "throwSoftMeshShapeUnavailable",
        {
            "SoftMeshShape::SoftMeshShape": 1,
            "SoftMeshShape::getAssimpMesh": 1,
            "SoftMeshShape::getSoftBodyNode": 1,
            "SoftMeshShape::update": 1,
            "SoftMeshShape::computeInertia": 1,
            "SoftMeshShape::clone": 1,
            "SoftMeshShape::updateBoundingBox": 1,
            "SoftMeshShape::updateVolume": 1,
            "SoftMeshShape::_buildMesh": 1,
        },
    ),
)

SAFE_METADATA = (
    ("MeshShape", "MeshShape"),
    ("SoftMeshShape", "SoftMeshShape"),
)


def strip_comments(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", source)


def class_method_declarations(header: str, class_name: str) -> Counter[str]:
    """Return out-of-line method declaration names from one simple class."""
    source = strip_comments(header)
    declaration = re.search(
        rf"\b(?:class|struct)\s+{re.escape(class_name)}\b[^{{;]*\{{", source
    )
    if declaration is None:
        raise ValueError(f"class declaration not found for {class_name}")

    opening = source.find("{", declaration.start())
    depth = 0
    closing = -1
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                closing = index
                break
    if closing < 0:
        raise ValueError(f"unbalanced class declaration for {class_name}")

    body = source[opening + 1 : closing]
    declarations: Counter[str] = Counter()
    segment_start = 0
    nested_braces = 0
    parentheses = 0
    for index, character in enumerate(body):
        if character == "{":
            nested_braces += 1
        elif character == "}":
            nested_braces -= 1
        elif character == "(":
            parentheses += 1
        elif character == ")":
            parentheses -= 1
        elif character == ";" and nested_braces == 0 and parentheses == 0:
            segment = body[segment_start:index].strip()
            segment_start = index + 1
            segment = re.sub(
                r"^(?:(?:public|protected|private)\s*:\s*)+", "", segment
            )
            if not segment or "(" not in segment or re.search(r"\benum\b", segment):
                continue
            prefix = segment[: segment.find("(")].rstrip()
            name = re.search(r"(~?[A-Za-z_]\w*)\s*$", prefix)
            if name is not None:
                declarations[name.group(1)] += 1
    return declarations


def matching_bodies(source: str, qualified_name: str) -> list[str]:
    pattern = re.compile(rf"\b{re.escape(qualified_name)}\s*\(")
    bodies: list[str] = []
    for match in pattern.finditer(source):
        opening = source.find("{", match.end())
        semicolon = source.find(";", match.end())
        if opening < 0 or (semicolon >= 0 and semicolon < opening):
            continue

        depth = 0
        for index in range(opening, len(source)):
            character = source[index]
            if character == "{":
                depth += 1
            elif character == "}":
                depth -= 1
                if depth == 0:
                    bodies.append(source[opening + 1 : index])
                    break
        else:
            raise ValueError(f"unbalanced body for {qualified_name}")
    return bodies


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: mesh_shape_ios_source_contract.py IMPLEMENTATION", file=sys.stderr)
        return 2

    source_path = Path(sys.argv[1])
    source = source_path.read_text(encoding="utf-8")
    failures: list[str] = []

    for helper, methods in CONTRACTS:
        helper_definition = re.compile(
            rf"\[\[noreturn\]\]\s+void\s+{re.escape(helper)}\s*\(\s*\)"
        )
        if len(helper_definition.findall(source)) != 1:
            failures.append(
                f"{helper} must have exactly one [[noreturn]] void definition"
            )

        for qualified_name, expected_count in methods.items():
            try:
                bodies = matching_bodies(source, qualified_name)
            except ValueError as error:
                failures.append(str(error))
                continue
            if len(bodies) != expected_count:
                failures.append(
                    f"{qualified_name} has {len(bodies)} definitions; "
                    f"expected {expected_count}"
                )
                continue
            for index, body in enumerate(bodies, start=1):
                if re.fullmatch(
                    rf"\s*{re.escape(helper)}\s*\(\s*\)\s*;\s*", body
                ) is None:
                    failures.append(
                        f"{qualified_name} definition {index} is not an "
                        f"unconditional {helper}() rejection"
                    )

    # Rejected construction makes all instance behavior unreachable. Keep the
    # narrow metadata/destructor whitelist explicit so future ports cannot
    # silently turn some other API into a partial no-op.
    for class_name, expected_type in SAFE_METADATA:
        get_type_bodies = matching_bodies(source, f"{class_name}::getType")
        if len(get_type_bodies) != 1 or not re.fullmatch(
            r"\s*return\s+getStaticType\s*\(\s*\)\s*;\s*", get_type_bodies[0]
        ):
            failures.append(
                f"{class_name}::getType must only return getStaticType()"
            )

        static_type_bodies = matching_bodies(source, f"{class_name}::getStaticType")
        if len(static_type_bodies) != 1:
            failures.append(
                f"{class_name}::getStaticType must have exactly one definition"
            )
        else:
            body = static_type_bodies[0]
            expected_literal = re.escape(expected_type)
            if re.fullmatch(
                rf'\s*static\s+const\s+std::string\s+type\s*'
                rf'\(\s*"{expected_literal}"\s*\)\s*;\s*'
                rf'return\s+type\s*;\s*',
                body,
            ) is None:
                failures.append(
                    f"{class_name}::getStaticType must return the pinned type literal"
                )

        destructor = re.compile(
            rf"\b{re.escape(class_name)}::~{re.escape(class_name)}\s*\(\s*\)\s*=\s*default\s*;"
        )
        if len(destructor.findall(source)) != 1:
            failures.append(f"{class_name} destructor must be defaulted")

    wrapper_destructor = re.compile(
        r"\bSharedMeshWrapper::~SharedMeshWrapper\s*\(\s*\)\s*=\s*default\s*;"
    )
    if len(wrapper_destructor.findall(source)) != 1:
        failures.append("SharedMeshWrapper destructor must be defaulted")

    # Reconcile the explicit policy above with the current headers. This turns
    # a newly declared non-virtual method into a test failure until it is
    # deliberately classified as safe metadata or fail-closed behavior.
    implementation_dir = source_path.parent
    for class_name, header_name in (
        ("MeshShape", "MeshShape.hpp"),
        ("SoftMeshShape", "SoftMeshShape.hpp"),
    ):
        header_path = implementation_dir / header_name
        try:
            actual = class_method_declarations(
                header_path.read_text(encoding="utf-8"), class_name
            )
        except (OSError, ValueError) as error:
            failures.append(str(error))
            continue

        expected: Counter[str] = Counter()
        prefix = f"{class_name}::"
        for _, methods in CONTRACTS:
            for qualified_name, count in methods.items():
                if qualified_name.startswith(prefix):
                    expected[qualified_name[len(prefix) :]] += count
        expected.update({"getType": 1, "getStaticType": 1, f"~{class_name}": 1})
        if actual != expected:
            missing = expected - actual
            unclassified = actual - expected
            failures.append(
                f"{class_name} header policy mismatch: missing={dict(missing)}, "
                f"unclassified={dict(unclassified)}"
            )

    wrapper_header = (implementation_dir / "MeshShape.hpp").read_text(
        encoding="utf-8"
    )
    try:
        wrapper_actual = class_method_declarations(
            wrapper_header, "SharedMeshWrapper"
        )
    except ValueError as error:
        failures.append(str(error))
    else:
        wrapper_expected = Counter(
            {"SharedMeshWrapper": 1, "~SharedMeshWrapper": 1}
        )
        if wrapper_actual != wrapper_expected:
            failures.append(
                "SharedMeshWrapper header policy mismatch: "
                f"expected={dict(wrapper_expected)}, actual={dict(wrapper_actual)}"
            )

    if failures:
        for failure in failures:
            print(f"SOURCE_CONTRACT_FAIL {failure}", file=sys.stderr)
        return 1

    print("MESH_SHAPE_IOS_SOURCE_CONTRACT_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
