#!/usr/bin/env python3
"""Evaluate Holzbaur moving point 2 against BioMotion's fixed point 2.

Run with the BioMotion OpenSim 4.6 virtual environment.  Distances are
reported in each model's humerus frame, avoiding unrelated ground/body
placements.  The source ``.osim`` must be a checked conversion of the original
SIMM ``.jnt`` + ``.msl`` pair; this script cannot establish that provenance.
"""

from __future__ import annotations

import argparse
import math

import opensim as osim


MUSCLES = ("DELT2", "PECM1", "PECM2", "PECM3")
ELEVATIONS_DEG = (0, 30, 60, 90, 115)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-osim", required=True)
    parser.add_argument("--target-osim", required=True)
    args = parser.parse_args()

    osim.Logger.removeFileSink()
    osim.Logger.setLevelString("error")
    source_model = osim.Model(args.source_osim)
    source_state = source_model.initSystem()
    target_model = osim.Model(args.target_osim)
    target_state = target_model.initSystem()

    source_coord = source_model.getCoordinateSet().get("shoulder_elv")
    target_coord = target_model.getCoordinateSet().get("shoulder_elv_r")
    source_frame = source_model.getBodySet().get("humerus")
    target_frame = target_model.getBodySet().get("humerus_r")

    maximum_by_muscle = {name: 0.0 for name in MUSCLES}
    print("muscle,elevation_deg,source_actual_rad,target_actual_rad,distance_mm")
    for muscle_name in MUSCLES:
        source_point = (
            source_model.getMuscles()
            .get(muscle_name)
            .getGeometryPath()
            .getPathPointSet()
            .get(1)
        )
        target_point = (
            target_model.getMuscles()
            .get(muscle_name)
            .getGeometryPath()
            .getPathPointSet()
            .get(1)
        )
        for elevation_deg in ELEVATIONS_DEG:
            elevation_rad = math.radians(elevation_deg)
            source_coord.setValue(source_state, elevation_rad, True)
            target_coord.setValue(target_state, elevation_rad, True)
            source_model.realizePosition(source_state)
            target_model.realizePosition(target_state)

            source_xyz = source_point.getParentFrame().findStationLocationInAnotherFrame(
                source_state, source_point.getLocation(source_state), source_frame
            )
            target_xyz = target_point.getParentFrame().findStationLocationInAnotherFrame(
                target_state, target_point.getLocation(target_state), target_frame
            )
            distance_mm = 1000 * math.sqrt(
                sum((source_xyz[index] - target_xyz[index]) ** 2 for index in range(3))
            )
            maximum_by_muscle[muscle_name] = max(
                maximum_by_muscle[muscle_name], distance_mm
            )
            print(
                f"{muscle_name},{elevation_deg},"
                f"{source_coord.getValue(source_state):.15g},"
                f"{target_coord.getValue(target_state):.15g},"
                f"{distance_mm:.12g}"
            )

    assert maximum_by_muscle["DELT2"] > 70
    assert all(maximum_by_muscle[name] > 15 for name in MUSCLES[1:])
    print("maximum-distance-mm:", maximum_by_muscle)
    print("HOLZBAUR_MOVING_POINT_DIFFERENCE_PASS")


if __name__ == "__main__":
    main()
