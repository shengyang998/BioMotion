"""OpenSim's own geometry for the MULTI-WRAP muscles, plus a length column that
is actually a path length.

# Why a third reference

`opensim_moment_arms.txt` and `opensim_moment_arms_fd.txt` both take
`GeometryPath::getLength(s)` at face value. For a muscle whose path wraps around
TWO cylinders that is not a path length. `calcLengthAfterPathComputation` sums

    straight segments measured between the wrap points OpenSim reports
  + the spiral length OpenSim stored beside them

and for the four gastrocnemius muscles those two halves do not describe the same
path. Measured on `gasmed_r` at knee 0 deg: the stored spiral length is
0.038054 m while the straight-line CHORD between the two tangent points that
spiral is supposed to connect is 0.045350 m. No curve joining two points can be
shorter than the straight line between them, so the reported total is 7.30 mm
below the length of any path through its own points. The gap is the cylinder
tangency adjustment (`WrapCylinder::_adjust_tangent_point`, which runs only when
a muscle carries more than one `PathWrap`): OpenSim moves the tangent points and
does not recompute the arc between them.

So this file carries, per sample:

  * `reported`   -- `getLength(s)`, i.e. what the other two fixtures use.
  * `reconciled` -- the same total with each CYLINDER spiral's stored arc
                    replaced by the shortest helix on that cylinder between the
                    two tangent points OpenSim itself reports. Every term then
                    belongs to one path.
  * `slack`      -- min over the reported spirals of (stored arc - chord), in
                    metres. Negative is the impossible-geometry statement above;
                    it is stored so the suite can assert the defect is real
                    rather than take this docstring's word for it.

It also carries the SOLVER'S RAW INPUTS -- the path points in ground and each
wrap object's ground frame -- so the port can be run on OpenSim's own forward
kinematics. That separates "the wrap solve disagrees" from "the forward
kinematics disagree", which is the whole reason `inspect_wrap.py` prints a
HARNESS block.

# The grid

Two sets, and the difference between them is the point.

  * BLIND: each muscle's driving coordinate swept in `--steps` uniform steps
    across its own clamped range, read from the model. Chosen by a rule, before
    any result, and it lands on no special value by construction.
  * ADVERSARIAL: the `--worst` rows with the largest |port - reported| found by
    a `--dense` sweep of the same coordinate. These are chosen BY the
    reference's misbehaviour, which is exactly what makes them a test: the port
    must still agree with the reconciled column there.

    uv run --python .venv python dump_multiwrap.py

Read-only against `FullBody.osim`.
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import tempfile

import numpy as np
import opensim as osim

import osim_model

FIXTURE = os.path.join(osim_model.REPO_ROOT, "BioMotionTests", "Fixtures",
                       "opensim_multiwrap.txt")
FORMAT_ID = "biomotion-osim-multiwrap-v1"
EPS = 1e-4

#: muscle -> (driving coordinate, extra coordinates held in DEGREES).
#: The extra hip flexion mirrors the committed `knee_sweep_*` family so the two
#: grids describe the same kind of configuration.
MUSCLES = [
    ("gasmed_r", "knee_angle_r", {"hip_flexion_r": 20.0}),
    ("gaslat140_r", "knee_angle_r", {"hip_flexion_r": 20.0}),
    ("gasmed_l", "knee_angle_l", {"hip_flexion_l": 20.0}),
    ("gaslat140_l", "knee_angle_l", {"hip_flexion_l": 20.0}),
    ("TRIlong_r", "elbow_flex_r", {"shoulder_elv_r": 25.0}),
    ("BIClong_r", "elbow_flex_r", {"shoulder_elv_r": 25.0}),
    ("TRIlong_l", "elbow_flex_l", {"shoulder_elv_l": -25.0}),
    ("BIClong_l", "elbow_flex_l", {"shoulder_elv_l": -25.0}),
]


def wrap_owner_table(model):
    owner = {}
    bodies = model.getBodySet()
    for i in range(bodies.getSize()):
        b = bodies.get(i)
        wo = b.getWrapObjectSet()
        for j in range(wo.getSize()):
            owner[wo.get(j).getName()] = b
    return owner


def wrap_specs(model, path, owner):
    """The muscle's `<PathWrapSet>` in file order, as flat records."""
    out = []
    wraps = path.getWrapSet()
    for i in range(wraps.getSize()):
        w = wraps.get(i)
        name = w.getWrapObjectName()
        obj = owner[name].getWrapObjectSet().get(name)
        cyl = osim.WrapCylinder.safeDownCast(obj)
        ell = osim.WrapEllipsoid.safeDownCast(obj)
        dims = (-1.0, -1.0, -1.0)
        if ell is not None:
            d = ell.get_dimensions()
            dims = (d.get(0), d.get(1), d.get(2))
        out.append(dict(
            name=name,
            kind=0 if cyl is not None else (1 if ell is not None else 2),
            radius=cyl.get_radius() if cyl is not None else -1.0,
            length=cyl.get_length() if cyl is not None else -1.0,
            dimensions=dims,
            quadrant=obj.get_quadrant().strip() or "all",
            method=0 if w.getMethodName() == "hybrid" else 1,
            start=w.getStartPoint(),
            end=w.getEndPoint(),
        ))
    return out


def sample(model, state, path, specs, owner, driver=None):
    """Everything one pose contributes: the solver's inputs and both columns."""
    points = []
    pps = path.getPathPointSet()
    for i in range(pps.getSize()):
        g = pps.get(i).getLocationInGround(state)
        points.append((g.get(0), g.get(1), g.get(2)))

    frames = []
    for spec in specs:
        body = owner[spec["name"]]
        obj = body.getWrapObjectSet().get(spec["name"])
        x_gp = body.getTransformInGround(state).compose(obj.getTransform())
        R = x_gp.R()
        p = x_gp.p()
        frames.append(([R.get(r, c) for r in range(3) for c in range(3)],
                       [p.get(0), p.get(1), p.get(2)]))

    by_name = {s["name"]: (i, s) for i, s in enumerate(specs)}
    current = path.getCurrentPath(state)
    reconciliation = 0.0
    slack = math.inf
    wrap_points = 0
    previous = None
    previous_owner = None
    for i in range(current.getSize()):
        point = current.get(i)
        raw = point.getLocationInGround(state)
        ground = np.array([raw.get(0), raw.get(1), raw.get(2)])
        wrap_point = osim.PathWrapPoint.safeDownCast(point)
        name = None
        stored = 0.0
        if wrap_point is not None:
            wrap_object = wrap_point.getWrapObject()
            name = wrap_object.getName() if wrap_object is not None else None
            stored = wrap_point.getWrapLength(state)
            wrap_points += 1
        if previous is not None and name is not None and previous_owner == name:
            index, spec = by_name[name]
            R = np.array(frames[index][0]).reshape(3, 3)
            t = np.array(frames[index][1])
            a = R.T @ (previous - t)
            b = R.T @ (ground - t)
            slack = min(slack, stored - float(np.linalg.norm(b - a)))
            if spec["kind"] == 0:
                u = np.array([a[0], a[1], 0.0])
                v = np.array([b[0], b[1], 0.0])
                theta = math.atan2(float(np.cross(u, v)[2]), float(u @ v))
                helix = math.hypot(spec["radius"] * theta, b[2] - a[2])
                reconciliation += helix - stored
        previous = ground
        previous_owner = name

    reported = path.getLength(state)
    # OpenSim's OTHER moment-arm column, and the third witness in this file.
    # `GeometryPath::computeMomentArm` asks `MomentArmSolver` for the generalized
    # force a unit tension along the CURRENT path produces with the wrap points
    # held fixed. It reads the reported wrap POINTS and never touches
    # `calcLengthAfterPathComputation`, so it is independent of the length defect
    # this file exists for -- and independent of the reconciliation, which is
    # this generator's arithmetic rather than OpenSim's.
    analytic = path.computeMomentArm(state, driver) if driver is not None else 0.0
    return dict(points=points, frames=frames, reported=reported,
                reconciled=reported + reconciliation,
                slack=(0.0 if slack is math.inf else slack),
                analytic=analytic,
                wrap_points=wrap_points)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=51,
                        help="blind grid: uniform steps across the clamped range")
    parser.add_argument("--dense", type=int, default=4001,
                        help="how finely the adversarial search sweeps")
    parser.add_argument("--worst", type=int, default=4,
                        help="adversarial rows kept per muscle")
    parser.add_argument("--out", default=FIXTURE)
    args = parser.parse_args()

    model, state, _ = osim_model.load_model()
    owner = wrap_owner_table(model)
    limits = osim_model.limits_degrees(model)
    coordinates = model.getCoordinateSet()
    by_name = {coordinates.get(i).getName(): coordinates.get(i)
               for i in range(coordinates.getSize())}

    port = build_port_probe()

    muscle_records = []
    groups = []          # (muscleIndex, kind, q_deg)
    for muscle_index, (muscle_name, driver, held) in enumerate(MUSCLES):
        muscle = model.getMuscles().get(muscle_name)
        path = muscle.getGeometryPath()
        specs = wrap_specs(model, path, owner)
        muscle_records.append(dict(name=muscle_name, driver=driver, specs=specs,
                                   points=path.getPathPointSet().getSize()))

        for name, value in held.items():
            by_name[name].setValue(state, math.radians(value), False)
        low, high = limits[driver]

        def take_offset(q_deg, offset=0.0):
            by_name[driver].setValue(state, math.radians(q_deg) + offset, False)
            model.realizePosition(state)
            return sample(model, state, path, specs, owner, by_name[driver])

        blind = [low + (high - low) * k / (args.steps - 1) for k in range(args.steps)]
        for q_deg in blind:
            groups.append((muscle_index, "blind", q_deg, held))

        # The adversarial rows: where the port's MOMENT ARM is furthest from a
        # central difference of the REPORTED column -- i.e. exactly the quantity
        # `CylinderWrapValidationTests` gates at 20 mm. Found by measurement over
        # the whole range, not by choosing a pose.
        # Stage 1, cheap: where does the REFERENCE's own length function jump?
        # A centred difference at eps only sees a jump when its stencil straddles
        # one, and the stencil is 2 * eps = 0.0115 deg wide, so a uniform grid
        # coarser than that finds these by luck -- which is the whole complaint
        # this fixture exists to answer. Scan for the jumps instead of hoping to
        # land on one.
        scan_n = max(args.dense, int((high - low) * math.pi / 180.0 / EPS) + 1)
        scan_q = [low + (high - low) * k / (scan_n - 1) for k in range(scan_n)]
        scan_l = []
        for q_deg in scan_q:
            by_name[driver].setValue(state, math.radians(q_deg), False)
            model.realizePosition(state)
            scan_l.append(path.getLength(state))
        curvature = np.abs(np.diff(np.array(scan_l), 2))
        jumps = []
        for k in np.argsort(curvature)[::-1]:
            q_deg = scan_q[int(k) + 1]
            if any(abs(q_deg - other) < (high - low) / 200.0 for other in jumps):
                continue
            jumps.append(q_deg)
            if len(jumps) >= 6 * args.worst:
                break

        # Stage 2: the arm gap at each candidate, at the eps the suite uses.
        dense_q = sorted(set(
            [low + (high - low) * k / (args.dense - 1) for k in range(args.dense)]
            + jumps))
        dense_rows = []
        for q_deg in dense_q:
            dense_rows.extend(take_offset(q_deg, offset) for offset in (-EPS, 0.0, EPS))
        dense_ours = port(dense_rows, specs, muscle_records[-1]["points"])
        scored = []
        worst_length = (0.0, dense_q[0])
        for k, q_deg in enumerate(dense_q):
            arm_port = -(dense_ours[3 * k + 2] - dense_ours[3 * k]) / (2 * EPS)
            arm_reported = -(dense_rows[3 * k + 2]["reported"]
                             - dense_rows[3 * k]["reported"]) / (2 * EPS)
            scored.append((abs(arm_port - arm_reported), q_deg))
            gap = abs(dense_ours[3 * k + 1] - dense_rows[3 * k + 1]["reported"])
            if gap > worst_length[0]:
                worst_length = (gap, q_deg)
        scored.sort(reverse=True)
        seen = []
        for gap, q_deg in scored:
            if any(abs(q_deg - other) < (high - low) / 50.0 for other in seen):
                continue
            seen.append(q_deg)
            groups.append((muscle_index, "adversarial", q_deg, held))
            if len(seen) >= args.worst:
                break
        print("  %-12s %-14s range %7.2f..%7.2f  over %d points: worst arm gap "
              "%8.3f mm at %9.5f deg, worst length gap %6.3f mm at %9.5f deg"
              % (muscle_name, driver, low, high, args.dense,
                 scored[0][0] * 1000, scored[0][1],
                 worst_length[0] * 1000, worst_length[1]))
        for name in held:
            by_name[name].setValue(state, 0.0, False)

    lines = [
        "# OpenSim 4.6 geometry for the 8 multi-wrap muscles of FullBody.osim,",
        "# with a length column that is a path length. Generated by",
        "# tools/opensim_ref/dump_multiwrap.py -- see its docstring for why",
        "# getLength() is not one for a two-cylinder path.",
        "format %s" % FORMAT_ID,
        "eps %.9f" % EPS,
        "muscles %d" % len(muscle_records),
    ]
    for index, record in enumerate(muscle_records):
        lines.append("muscle %d %s %s %d %d"
                     % (index, record["name"], record["driver"],
                        record["points"], len(record["specs"])))
        for slot, spec in enumerate(record["specs"]):
            lines.append("wrap %d %d %s %d %.12g %.12g %.12g %.12g %.12g %s %d %d %d"
                         % (index, slot, spec["name"], spec["kind"], spec["radius"],
                            spec["length"], spec["dimensions"][0], spec["dimensions"][1],
                            spec["dimensions"][2], spec["quadrant"], spec["method"],
                            spec["start"], spec["end"]))
    lines.append("groups %d" % len(groups))

    for group_index, (muscle_index, kind, q_deg, held) in enumerate(groups):
        record = muscle_records[muscle_index]
        path = model.getMuscles().get(record["name"]).getGeometryPath()
        for name, value in held.items():
            by_name[name].setValue(state, math.radians(value), False)
        for slot, offset in enumerate((-EPS, 0.0, EPS)):
            by_name[record["driver"]].setValue(state, math.radians(q_deg) + offset, False)
            model.realizePosition(state)
            row = sample(model, state, path, record["specs"], owner,
                         by_name[record["driver"]])
            lines.append("sample %d %d %d %s %.12g %.12g %.12g %.12g %.12g %d %d"
                         % (group_index, muscle_index, slot, kind, q_deg,
                            row["reported"], row["reconciled"], row["slack"],
                            row["analytic"], row["wrap_points"], len(row["points"])))
            for point in row["points"]:
                lines.append("point %.12g %.12g %.12g" % point)
            for wrap_slot, (rotation, translation) in enumerate(row["frames"]):
                lines.append("frame %d %s %.12g %.12g %.12g"
                             % (wrap_slot, " ".join("%.12g" % v for v in rotation),
                                translation[0], translation[1], translation[2]))
        for name in held:
            by_name[name].setValue(state, 0.0, False)

    with open(args.out, "w", encoding="ascii") as handle:
        handle.write("\n".join(lines) + "\n")
    print("wrote %s: %d muscles, %d groups, %d samples, %.1f KB"
          % (args.out, len(muscle_records), len(groups), len(groups) * 3,
             os.path.getsize(args.out) / 1024.0))


def build_port_probe():
    """Compile the SHIPPING wrap solver into a callable, so the adversarial rows
    are the ones where the port and the reference actually disagree rather than
    where a Python re-derivation would.

    Returns a function (sample-dict, specs, originalPointCount) -> length."""
    root = osim_model.REPO_ROOT
    source = os.path.join(tempfile.mkdtemp(prefix="biomotion-wrapprobe-"), "probe.cpp")
    binary = source[:-4]
    with open(source, "w", encoding="ascii") as handle:
        handle.write(PROBE_SOURCE)
    subprocess.run(
        ["clang++", "-std=c++17", "-O2",
         "-I" + os.path.join(root, "BioMotion", "Muscle"),
         "-I" + os.path.join(root, "nimblephysics", "third_party", "eigen"),
         source, os.path.join(root, "BioMotion", "Muscle", "MusclePathWrap.cpp"),
         "-o", binary],
        check=True)

    def run(rows, specs, original_points):
        """rows -> list of lengths, in one subprocess."""
        payload = ["%d %d" % (len(specs), original_points)]
        for spec in specs:
            payload.append("%d %.17g %.17g %.17g %.17g %.17g %s %d %d %d"
                           % (spec["kind"], spec["radius"], spec["length"],
                              spec["dimensions"][0], spec["dimensions"][1],
                              spec["dimensions"][2], spec["quadrant"],
                              spec["method"], spec["start"], spec["end"]))
        payload.append("%d" % len(rows))
        for row in rows:
            payload.append("%d" % len(row["points"]))
            for point in row["points"]:
                payload.append("%.17g %.17g %.17g" % point)
            for rotation, translation in row["frames"]:
                payload.append(" ".join("%.17g" % v for v in rotation)
                               + " " + " ".join("%.17g" % v for v in translation))
        result = subprocess.run([binary], input="\n".join(payload) + "\n",
                                capture_output=True, text=True, check=True)
        out = [float(line.split()[0]) for line in result.stdout.strip().split("\n")]
        assert len(out) == len(rows), (len(out), len(rows))
        return out

    return run


PROBE_SOURCE = r"""
#include <cstdio>
#include <string>
#include <vector>
#include "MusclePathWrap.h"
int main() {
    int nw = 0, originalCount = 0;
    if (scanf("%d %d", &nw, &originalCount) != 2) return 2;
    std::vector<biomotion::WrapObjectSpec> objects;
    std::vector<biomotion::PathWrapSpec> wraps;
    for (int i = 0; i < nw; i++) {
        int kind, method, start, end; double radius, length, d0, d1, d2; char quadrant[64];
        if (scanf("%d %lf %lf %lf %lf %lf %63s %d %d %d", &kind, &radius, &length,
                  &d0, &d1, &d2, quadrant, &method, &start, &end) != 10) return 3;
        biomotion::WrapObjectSpec o;
        o.kind = kind == 0 ? biomotion::WrapKind::Cylinder
               : kind == 1 ? biomotion::WrapKind::Ellipsoid
                           : biomotion::WrapKind::Unsupported;
        o.radius = radius; o.length = length;
        o.dimensions = Eigen::Vector3d(d0, d1, d2);
        o.pose = Eigen::Isometry3d::Identity();
        if (!biomotion::decodeWrapQuadrant(quadrant, o.wrapAxis, o.wrapSign)) return 4;
        objects.push_back(o);
        biomotion::PathWrapSpec w;
        w.wrapObject = i; w.startPoint = start; w.endPoint = end;
        w.method = method == 0 ? biomotion::PathWrapMethod::Hybrid
                               : biomotion::PathWrapMethod::Unsupported;
        wraps.push_back(w);
    }
    int nrows = 0;
    if (scanf("%d", &nrows) != 1) return 5;
    for (int row = 0; row < nrows; row++) {
        int npts = 0;
        if (scanf("%d", &npts) != 1) return 6;
        std::vector<Eigen::Vector3d> points; std::vector<int> index;
        for (int i = 0; i < npts; i++) {
            double x, y, z;
            if (scanf("%lf %lf %lf", &x, &y, &z) != 3) return 7;
            points.push_back(Eigen::Vector3d(x, y, z)); index.push_back(i);
        }
        std::vector<Eigen::Isometry3d> frames;
        for (int i = 0; i < nw; i++) {
            Eigen::Matrix3d R; double t[3];
            for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) if (scanf("%lf", &R(r, c)) != 1) return 8;
            for (int k = 0; k < 3; k++) if (scanf("%lf", &t[k]) != 1) return 9;
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.linear() = R; T.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
            frames.push_back(T);
        }
        biomotion::WrappedPathResult out = biomotion::solveWrappedPathLength(
            points.data(), index.data(), npts, originalCount, wraps.data(), nw,
            objects.data(), frames.data(), nw);
        printf("%.17g %d\n", out.length, out.wrapPointCount);
    }
    return 0;
}
"""


if __name__ == "__main__":
    main()
