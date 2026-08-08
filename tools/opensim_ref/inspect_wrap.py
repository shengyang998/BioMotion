"""Print OpenSim's wrapped path for one (pose, muscle), point by point.

The aggregate fixture says WHETHER the port agrees with OpenSim. This says
WHERE it stops agreeing: which wrap objects engaged, where the tangent points
landed, and how the total length decomposes into straight segments and spiral
arcs. Read-only against `FullBody.osim`.

    uv run --python .venv python inspect_wrap.py --pose neutral --muscle gasmed_r

Poses are read from the committed fixture
(`BioMotionTests/Fixtures/opensim_moment_arms.txt`) so the configuration is
byte-identical to the one the Swift suite drives.
"""

from __future__ import annotations

import argparse
import math
import os

import opensim as osim

import osim_model

FIXTURE = os.path.join(
    osim_model.REPO_ROOT, "BioMotionTests", "Fixtures", "opensim_moment_arms.txt"
)


def read_pose(pose_id):
    """coordinate name -> value, from the committed fixture."""
    coordinates = None
    with open(FIXTURE, "r", encoding="ascii") as handle:
        for line in handle:
            fields = line.rstrip("\n").split(" ")
            if fields[0] == "coordinates":
                coordinates = fields[1:]
            elif fields[0] == "pose" and fields[1] == pose_id:
                values = [float(v) for v in fields[2:]]
                return dict(zip(coordinates, values))
    raise SystemExit(f"pose {pose_id!r} is not in {FIXTURE}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose", default="neutral")
    parser.add_argument("--muscle", required=True)
    args = parser.parse_args()

    model, state, _ = osim_model.load_model()
    values = read_pose(args.pose)
    coordinates = model.getCoordinateSet()
    for i in range(coordinates.getSize()):
        coordinate = coordinates.get(i)
        if coordinate.getName() in values:
            coordinate.setValue(state, values[coordinate.getName()], False)
    model.assemble(state)
    model.realizePosition(state)

    muscle = model.getMuscles().get(args.muscle)
    path = muscle.getGeometryPath()

    print(f"pose={args.pose} muscle={args.muscle}")
    wraps = path.getWrapSet()
    for i in range(wraps.getSize()):
        wrap = wraps.get(i)
        print(f"  PathWrap[{i}] object={wrap.getWrapObjectName()} "
              f"range=({wrap.getStartPoint()}, {wrap.getEndPoint()})")

    current = path.getCurrentPath(state)
    total = 0.0
    previous = None
    previous_wrap = None
    for i in range(current.getSize()):
        point = current.get(i)
        raw = point.getLocationInGround(state)
        ground = (raw.get(0), raw.get(1), raw.get(2))
        wrap_point = osim.PathWrapPoint.safeDownCast(point)
        owner = None
        wrap_length = 0.0
        if wrap_point is not None:
            wrap_object = wrap_point.getWrapObject()
            owner = wrap_object.getName() if wrap_object is not None else "?"
            wrap_length = wrap_point.getWrapLength(state)
        if previous is not None:
            if owner is not None and previous_wrap == owner:
                segment = wrap_length
                kind = "spiral"
            else:
                segment = math.dist(ground, previous)
                kind = "straight"
            total += segment
            print(f"    +{segment:.6f}  ({kind})")
        label = f"WRAP[{owner}]" if owner else f"point[{point.getName()}]"
        print(f"  {i}: {label} ground=({ground[0]:.6f}, "
              f"{ground[1]:.6f}, {ground[2]:.6f})")
        previous = ground
        previous_wrap = owner

    print(f"  summed   = {total:.6f}")
    print(f"  getLength= {path.getLength(state):.6f}")

    # The solver's raw inputs, in the frame it works in. With these a standalone
    # harness can run the port on OpenSim's OWN geometry, which separates "the
    # wrap solve disagrees" from "the forward kinematics disagree".
    print("HARNESS")
    pps = path.getPathPointSet()
    for i in range(pps.getSize()):
        g = pps.get(i).getLocationInGround(state)
        print(f"HARNESS point {i} {g.get(0):.9f} {g.get(1):.9f} {g.get(2):.9f}")
    body_set = model.getBodySet()
    owner = {}
    for i in range(body_set.getSize()):
        b = body_set.get(i)
        wo = b.getWrapObjectSet()
        for j in range(wo.getSize()):
            owner[wo.get(j).getName()] = b
    for i in range(wraps.getSize()):
        name = wraps.get(i).getWrapObjectName()
        body = owner[name]
        obj = body.getWrapObjectSet().get(name)
        cyl = osim.WrapCylinder.safeDownCast(obj)
        # X_ground_wrapframe = X_ground_body * _pose
        x_gb = body.getTransformInGround(state)
        x_bp = obj.getTransform()
        x_gp = x_gb.compose(x_bp)
        R = x_gp.R()
        t = x_gp.p()
        rows = " ".join(f"{R.get(r, c):.12f}" for r in range(3) for c in range(3))
        radius = cyl.get_radius() if cyl is not None else -1.0
        length = cyl.get_length() if cyl is not None else -1.0
        print(f"HARNESS wrap {i} {name} radius {radius:.9f} length {length:.9f} "
              f"quadrant {obj.get_quadrant()} start {wraps.get(i).getStartPoint()} "
              f"end {wraps.get(i).getEndPoint()} t {t.get(0):.9f} {t.get(1):.9f} "
              f"{t.get(2):.9f} R {rows}")


if __name__ == "__main__":
    main()
