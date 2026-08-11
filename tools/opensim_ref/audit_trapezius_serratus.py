"""Audit the shipped trapezius/serratus geometry without changing the model.

This is the reproducible receipt for STATUS.md's 2026-08-11 audit.  It asks
OpenSim which unlocked rotational coordinates each of the 48 target paths
structurally spans, measures their generalized lever arms at neutral and the
committed ``spine_flexed`` pose, and independently checks ``r = -dL/dq`` by
central difference.  It also proves the important negative boundary: the same
paths have exactly zero moment arm about all six glenohumeral coordinates.

Run from the repository root:

    tools/opensim_ref/.venv/bin/python \
      tools/opensim_ref/audit_trapezius_serratus.py

The helper removes OpenSim's file logger before loading the model, so this
script is read-only and does not leave ``opensim.log`` in app resources.
"""

from __future__ import annotations

import math
import statistics

import opensim as osim

import osim_model as model_helpers
import poses as pose_helpers


MIN_NONZERO_ARM_M = 1e-6
FINITE_DIFFERENCE_STEP_RAD = 1e-4
EXPECTED_TARGETS = 48
EXPECTED_ROTATIONAL_COORDINATES = 59
EXPECTED_STRUCTURAL_PAIRS = 852
EXPECTED_SHOULDER_SAMPLES = 288

SHOULDER_COORDINATES = (
    "shoulder_elv_r",
    "shoulder_rot_r",
    "elv_angle_r",
    "shoulder_elv_l",
    "shoulder_rot_l",
    "elv_angle_l",
)


def _targets(muscles):
    return [
        muscles.get(index).getName()
        for index in range(muscles.getSize())
        if muscles.get(index).getName().startswith(("trap_", "SerrAnt"))
    ]


def _coordinate_group(name: str) -> str:
    if name.startswith("T1_head_neck_"):
        return "head/neck"
    if name.startswith("SternumRot"):
        return "sternum"
    if "_r" in name and name.endswith("_X"):
        return "rib"
    return "thoracic spine"


def _set_named_pose(model, state, poses_by_name, name: str) -> None:
    model_helpers.set_pose(
        model,
        state,
        pose_helpers.to_radians(poses_by_name[name]),
    )


def _arms_at_pose(model, state, muscles, coordinates, pairs, poses_by_name, name):
    _set_named_pose(model, state, poses_by_name, name)
    return [
        muscles.get(muscle).getGeometryPath().computeMomentArm(
            state, coordinates.get(coordinate)
        )
        for muscle, coordinate in pairs
    ]


def _print_arm_summary(name: str, values) -> None:
    magnitudes = [abs(value) for value in values]
    print(
        f"{name}: nonzero={sum(value > MIN_NONZERO_ARM_M for value in magnitudes)}"
        f"/{len(values)} >=1mm={sum(value >= 1e-3 for value in magnitudes)}"
        f"/{len(values)} min/median/max-mm="
        f"{1000 * min(magnitudes):.6f}/"
        f"{1000 * statistics.median(magnitudes):.6f}/"
        f"{1000 * max(magnitudes):.6f}"
    )


def _finite_difference_check(
    model,
    state,
    muscles,
    coordinates,
    pairs,
    poses_by_name,
    pose_name: str,
):
    analytic = []
    finite = []

    for muscle_name, coordinate_name in pairs:
        _set_named_pose(model, state, poses_by_name, pose_name)
        coordinate = coordinates.get(coordinate_name)
        path = muscles.get(muscle_name).getGeometryPath()
        base = coordinate.getValue(state)
        analytic.append(path.computeMomentArm(state, coordinate))

        coordinate.setValue(state, base + FINITE_DIFFERENCE_STEP_RAD, False)
        model.realizePosition(state)
        length_plus = path.getLength(state)

        coordinate.setValue(state, base - FINITE_DIFFERENCE_STEP_RAD, False)
        model.realizePosition(state)
        length_minus = path.getLength(state)

        finite.append(
            -(length_plus - length_minus) / (2 * FINITE_DIFFERENCE_STEP_RAD)
        )

    errors = [abs(a - b) for a, b in zip(analytic, finite)]
    sign_disagreements = sum(
        (a > 0) != (b > 0) for a, b in zip(analytic, finite)
    )
    print(
        f"finite-difference {pose_name}: sign-disagreements="
        f"{sign_disagreements}/{len(pairs)} max/median-error-m="
        f"{max(errors):.12g}/{statistics.median(errors):.12g}"
    )
    assert sign_disagreements == 0
    assert max(errors) < 1e-6


def _path_point_counts(muscles, target_names):
    counts = {
        "trapezius": {"PathPoint": 0, "ConditionalPathPoint": 0},
        "serratus": {"PathPoint": 0, "ConditionalPathPoint": 0},
    }
    for name in target_names:
        group = "trapezius" if name.startswith("trap_") else "serratus"
        points = muscles.get(name).getGeometryPath().getPathPointSet()
        for index in range(points.getSize()):
            concrete = points.get(index).getConcreteClassName()
            if concrete in counts[group]:
                counts[group][concrete] += 1
    return counts


def main() -> None:
    osim.Logger.setLevelString("error")
    model, state, _ = model_helpers.load_model()
    coordinates = model.getCoordinateSet()
    muscles = model.getMuscles()
    targets = _targets(muscles)
    assert len(targets) == EXPECTED_TARGETS

    spanned = model_helpers.spanned_coordinates(model)
    pairs = [
        (muscle, coordinate)
        for muscle in targets
        for coordinate in spanned[muscle]
        if int(coordinates.get(coordinate).getMotionType()) == 1
    ]
    unique_coordinates = sorted({coordinate for _, coordinate in pairs})
    assert len(unique_coordinates) == EXPECTED_ROTATIONAL_COORDINATES
    assert len(pairs) == EXPECTED_STRUCTURAL_PAIRS

    print(
        f"targets={len(targets)} rotational-coordinates="
        f"{len(unique_coordinates)} structural-pairs={len(pairs)}"
    )
    print(
        "muscle-groups: trapezius="
        f"{sum(name.startswith('trap_') for name in targets)} serratus="
        f"{sum(name.startswith('SerrAnt') for name in targets)}"
    )

    poses_by_name = dict(
        pose_helpers.build_poses(model_helpers.limits_degrees(model))
    )
    arms_by_pose = {}
    for pose_name in ("neutral", "spine_flexed"):
        values = _arms_at_pose(
            model,
            state,
            muscles,
            coordinates,
            pairs,
            poses_by_name,
            pose_name,
        )
        arms_by_pose[pose_name] = values
        _print_arm_summary(pose_name, values)
        assert all(abs(value) > MIN_NONZERO_ARM_M for value in values)
        _finite_difference_check(
            model,
            state,
            muscles,
            coordinates,
            pairs,
            poses_by_name,
            pose_name,
        )

    neutral_by_pair = dict(zip(pairs, arms_by_pose["neutral"]))
    for group in ("trapezius", "serratus"):
        prefix = "trap_" if group == "trapezius" else "SerrAnt"
        values = [
            value
            for (muscle, _), value in neutral_by_pair.items()
            if muscle.startswith(prefix)
        ]
        _print_arm_summary(f"neutral {group}", values)

    for group in ("head/neck", "thoracic spine", "rib", "sternum"):
        values = [
            value
            for (_, coordinate), value in neutral_by_pair.items()
            if _coordinate_group(coordinate) == group
        ]
        print(
            f"neutral {group}: nonzero="
            f"{sum(abs(value) > MIN_NONZERO_ARM_M for value in values)}"
            f"/{len(values)}"
        )

    three_point_values = []
    for muscle_name, coordinate_name in pairs:
        values = []
        for degrees in (-4.0, 0.0, 4.0):
            model_helpers.set_pose(
                model,
                state,
                {coordinate_name: math.radians(degrees)},
            )
            values.append(
                muscles.get(muscle_name)
                .getGeometryPath()
                .computeMomentArm(state, coordinates.get(coordinate_name))
            )
        three_point_values.append(values)

    flat_values = [value for values in three_point_values for value in values]
    sign_reversals = sum(
        min(values) < 0 < max(values) for values in three_point_values
    )
    print(
        f"+/-4deg: nonzero-samples="
        f"{sum(abs(value) > MIN_NONZERO_ARM_M for value in flat_values)}"
        f"/{len(flat_values)} all-nonzero-pairs="
        f"{sum(all(abs(value) > MIN_NONZERO_ARM_M for value in values) for values in three_point_values)}"
        f"/{len(three_point_values)} sign-reversals={sign_reversals}"
        f"/{len(three_point_values)}"
    )
    assert all(abs(value) > MIN_NONZERO_ARM_M for value in flat_values)

    model_helpers.set_pose(model, state, {})
    shoulder_values = [
        muscles.get(muscle_name)
        .getGeometryPath()
        .computeMomentArm(state, coordinates.get(coordinate_name))
        for muscle_name in targets
        for coordinate_name in SHOULDER_COORDINATES
    ]
    assert len(shoulder_values) == EXPECTED_SHOULDER_SAMPLES
    print(
        f"shoulder: nonzero-above-1e-12="
        f"{sum(abs(value) > 1e-12 for value in shoulder_values)}"
        f"/{len(shoulder_values)} max-absolute={max(map(abs, shoulder_values))}"
    )
    assert max(map(abs, shoulder_values)) == 0.0

    print("path-points:", _path_point_counts(muscles, targets))
    print("TRAPEZIUS_SERRATUS_GEOMETRY_AUDIT_PASS")


if __name__ == "__main__":
    main()
