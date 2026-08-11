#!/usr/bin/env python3
"""Compare Holzbaur shoulder-muscle lineage against BioMotion FullBody.osim.

The audit accepts, but deliberately does not download, three explicit inputs:

* the Holzbaur SIMM ``.msl`` muscle file;
* an independently converted/published Holzbaur ``.osim`` file; and
* BioMotion's shipped ``FullBody.osim``.

Only Python's standard library is used.  A third-party SIMM mirror is useful
for a reproducible preflight, but its bytes must be replaced or verified with
the authenticated official SimTK ZIP before its hash is provenance evidence.
The published conversion cross-check catches a bad ``.msl`` parser; it does not
replace the official package or the separate ``.jnt`` pseudo-segment audit.
"""

from __future__ import annotations

import argparse
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path


NAMES = "DELT1 DELT2 DELT3 SUPSP INFSP SUBSC TMIN TMAJ PECM1 PECM2 PECM3 CORB".split()
EXPECTED_UNCHANGED_SCALARS = {
    "DELT1", "DELT2", "DELT3", "SUPSP", "INFSP", "SUBSC",
    "TMIN", "TMAJ", "CORB",
}
EXPECTED_UNCHANGED_FIXED_PATHS = {
    "DELT3", "SUPSP", "INFSP", "SUBSC", "TMIN", "TMAJ", "CORB",
}
RIGHT_BODY_MAP = {
    "humerus": "humerus_r",
    "scapula": "scapula_R",
    "clavicle": "clavicle_R",
}
LEFT_BODY_MAP = {
    "humerus": "humerus_l",
    "scapula": "scapula_L",
    "clavicle": "clavicle_L",
}


def parse_msl(path: Path) -> dict[str, dict]:
    text = path.read_text(errors="replace")
    muscles: dict[str, dict] = {}
    block_pattern = re.compile(
        r"(?ms)^beginmuscle\s+(\S+)\s*$\n(.*?)^endmuscle\s*$"
    )
    for match in block_pattern.finditer(text):
        name, block = match.groups()

        def scalar(key: str) -> float | None:
            found = re.search(
                rf"(?m)^\s*{re.escape(key)}\s+([-+.0-9Ee]+)", block
            )
            return float(found.group(1)) if found else None

        points: list[dict] = []
        point_block = re.search(
            r"(?ms)^\s*beginpoints\s*$\n(.*?)^\s*endpoints\s*$", block
        )
        if point_block:
            for line in point_block.group(1).splitlines():
                point = re.match(
                    r"^\s*([-+.0-9Ee]+)\s+([-+.0-9Ee]+)\s+"
                    r"([-+.0-9Ee]+)\s+segment\s+(\S+)\s*$",
                    line,
                )
                if point:
                    points.append(
                        {
                            "kind": "SIMMPoint",
                            "frame": point.group(4),
                            "xyz": tuple(map(float, point.groups()[:3])),
                        }
                    )
        muscles[name] = {
            "F": scalar("max_force"),
            "OFL": scalar("optimal_fiber_length"),
            "points": points,
        }
    return muscles


def parse_osim(path: Path) -> dict[str, dict]:
    root = ET.parse(path).getroot()
    muscles: dict[str, dict] = {}
    for element in root.iter():
        name = element.attrib.get("name")
        if not name:
            continue
        force = element.find("./max_isometric_force")
        length = element.find("./optimal_fiber_length")
        if force is None or length is None:
            continue
        points: list[dict] = []
        for point in element.iter():
            if not point.tag.endswith("PathPoint"):
                continue
            location = point.find("./location")
            frame = point.find("./socket_parent_frame")
            if frame is None:
                frame = point.find("./body")
            if location is None or frame is None:
                continue
            frame_name = frame.text.strip().split("/")[-1]
            points.append(
                {
                    "kind": point.tag,
                    "frame": frame_name,
                    "xyz": tuple(map(float, location.text.split())),
                }
            )
        muscles[name] = {
            "F": float(force.text),
            "OFL": float(length.text),
            "points": points,
        }
    return muscles


def close(a: float, b: float, tolerance: float) -> bool:
    return math.isclose(a, b, rel_tol=0.0, abs_tol=tolerance)


def xyz_close(a: tuple[float, ...], b: tuple[float, ...], tolerance: float) -> bool:
    return all(close(x, y, tolerance) for x, y in zip(a, b))


def print_scalar_table(msl: dict, published: dict, target: dict, tolerance: float) -> set[str]:
    print(
        "name,msl_F,published_F,target_R_F,target_L_F,R_delta_pct,"
        "msl_OFL,published_OFL,target_R_OFL,target_L_OFL,R_delta_pct_OFL"
    )
    unchanged: set[str] = set()
    for name in NAMES:
        source = msl[name]
        converted = published[name]
        right = target[name]
        left = target[name + "_l"]
        assert close(source["F"], converted["F"], tolerance), name
        assert close(source["OFL"], converted["OFL"], tolerance), name
        if all(
            close(source[key], side[key], tolerance)
            for key in ("F", "OFL")
            for side in (right, left)
        ):
            unchanged.add(name)
        values = (
            name,
            f'{source["F"]:.12g}',
            f'{converted["F"]:.12g}',
            f'{right["F"]:.15g}',
            f'{left["F"]:.15g}',
            f'{100 * (right["F"] / source["F"] - 1):+.9f}',
            f'{source["OFL"]:.12g}',
            f'{converted["OFL"]:.12g}',
            f'{right["OFL"]:.15g}',
            f'{left["OFL"]:.15g}',
            f'{100 * (right["OFL"] / source["OFL"] - 1):+.9f}',
        )
        print(",".join(values))
    return unchanged


def print_path_table(msl: dict, published: dict, target: dict, tolerance: float):
    print("\nRIGHT PATH POINTS")
    print(
        "muscle,index,source_frame,target_frame,source_xyz,target_xyz,"
        "mapped_frame_equal,xyz_equal"
    )
    unchanged_right: set[str] = set()
    unchanged_left: set[str] = set()
    for name in NAMES:
        source_points = msl[name]["points"]
        published_points = published[name]["points"]
        right_points = target[name]["points"]
        left_points = target[name + "_l"]["points"]
        assert len(source_points) == len(published_points), name
        assert all(
            a["frame"] == b["frame"] and xyz_close(a["xyz"], b["xyz"], tolerance)
            for a, b in zip(source_points, published_points)
        ), f"published conversion differs from MSL for {name}"

        right_ok = len(source_points) == len(right_points)
        left_ok = len(source_points) == len(left_points)
        for index, (source, right) in enumerate(zip(source_points, right_points), 1):
            expected_frame = RIGHT_BODY_MAP.get(source["frame"])
            frame_equal = expected_frame == right["frame"]
            coordinate_equal = xyz_close(source["xyz"], right["xyz"], tolerance)
            right_ok &= frame_equal and coordinate_equal
            source_xyz = " ".join(f"{x:.8g}" for x in source["xyz"])
            target_xyz = " ".join(f"{x:.8g}" for x in right["xyz"])
            print(
                f"{name},{index},{source['frame']},{right['frame']},"
                f'"{source_xyz}","{target_xyz}",{frame_equal},{coordinate_equal}'
            )
        for source, left in zip(source_points, left_points):
            expected_frame = LEFT_BODY_MAP.get(source["frame"])
            expected_xyz = (source["xyz"][0], source["xyz"][1], -source["xyz"][2])
            left_ok &= expected_frame == left["frame"] and xyz_close(
                expected_xyz, left["xyz"], tolerance
            )
        if right_ok:
            unchanged_right.add(name)
        if left_ok:
            unchanged_left.add(name)

    print("\nfully unchanged right (safe fixed-frame mapping):", " ".join(sorted(unchanged_right)))
    print("fully mirrored left (safe fixed-frame mapping):", " ".join(sorted(unchanged_left)))
    print(
        "Note: thorax->sternum and SIMM moving pseudo-segments are deliberately "
        "not declared equivalent by the raw-coordinate test."
    )
    return unchanged_right, unchanged_left


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--msl", type=Path, required=True)
    parser.add_argument("--published-osim", type=Path, required=True)
    parser.add_argument("--target-osim", type=Path, required=True)
    parser.add_argument("--tolerance", type=float, default=1e-12)
    args = parser.parse_args()

    msl = parse_msl(args.msl)
    published = parse_osim(args.published_osim)
    target = parse_osim(args.target_osim)
    for name in NAMES:
        assert name in msl and name in published and name in target
        assert name + "_l" in target

    unchanged_scalars = print_scalar_table(msl, published, target, args.tolerance)
    unchanged_right, unchanged_left = print_path_table(
        msl, published, target, args.tolerance
    )
    assert unchanged_scalars == EXPECTED_UNCHANGED_SCALARS
    assert unchanged_right == EXPECTED_UNCHANGED_FIXED_PATHS
    assert unchanged_left == EXPECTED_UNCHANGED_FIXED_PATHS
    print(
        f"summary: scalar lineage={2 * len(unchanged_scalars)}/24 bilateral; "
        f"fixed-path lineage={2 * len(unchanged_right)}/24 bilateral"
    )
    print("HOLZBAUR_SHOULDER_LINEAGE_AUDIT_PASS")


if __name__ == "__main__":
    main()
