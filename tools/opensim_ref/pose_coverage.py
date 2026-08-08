"""How much of the model does the pose grid cover, and what the clips say.

Two separate questions, deliberately not merged:

1. COORDINATE COVERAGE -- definition-free and checkable. What fraction of each
   sagittal lower-limb coordinate's clamped range does the grid sample, and at
   what step? This is the claim `poses.py` is allowed to make.

2. WHAT THE PINNED CLIPS SHOW -- reported, NOT used as a gate. The fixtures
   hold pelvis and both ankles, so pelvis-to-ankle distance over that clip's own
   maximum is a scale-free measure of how far the leg folds. It cannot be turned
   into a knee angle without a knee marker, and the denominator is only assumed
   to be a straight leg. So it is evidence about the clips, not a pass/fail on
   the grid -- and it turned up something worth writing down: see the verdict.

Run:  tools/opensim_ref/.venv/bin/python tools/opensim_ref/pose_coverage.py
"""

from __future__ import annotations

import math
import os
import sys

import osim_model as M
import poses as P

FIXTURE_DIR = os.path.join(M.REPO_ROOT, "BioMotionTests", "Fixtures")
CLIP_IDS = ["video_012", "video_013", "video_015"]
SAGITTAL = ["hip_flexion_r", "knee_angle_r", "ankle_angle_r",
            "shoulder_elv_r", "elbow_flex_r"]


def clip_ratios(clip_id):
    header_keys = {"format", "clip", "frames", "joints"}
    distances = []
    with open(os.path.join(FIXTURE_DIR, "gait_%s.txt" % clip_id)) as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split(" ")
            if fields[0] in header_keys:
                continue
            values = [float(v) for v in fields[2:]]
            pelvis = values[0:3]
            for ankle in (values[3:6], values[6:9]):
                distances.append(math.dist(pelvis, ankle))
    top = max(distances)
    return [d / top for d in distances]


def coordinate_coverage(model):
    cs = model.getCoordinateSet()
    ranges = {}
    for i in range(cs.getSize()):
        c = cs.get(i)
        ranges[c.getName()] = (math.degrees(c.getRangeMin()),
                               math.degrees(c.getRangeMax()))
    sampled = {name: set() for name in SAGITTAL}
    for _, degrees in P.build_poses(limits=M.limits_degrees(model)):
        for name in SAGITTAL:
            sampled[name].add(round(degrees.get(name, 0.0), 4))
    out = {}
    for name in SAGITTAL:
        values = sorted(sampled[name])
        low, high = ranges[name]
        span = high - low
        covered = (max(values) - min(values)) / span if span else 0.0
        step = max((b - a) for a, b in zip(values, values[1:])) if len(values) > 1 else span
        out[name] = (low, high, min(values), max(values), covered, step, len(values))
    return out


def fold_extremes(model, state):
    """Smallest pelvis-to-ankle distance, over that pose's straight-leg value,
    that FullBody.osim can reach ANYWHERE in its clamped sagittal range. Two
    endpoint definitions, because the clips' `hips_joint` is the MHR mid-hip and
    the model's `pelvis` body origin sits 9.7 cm away from the hip-centre
    midpoint (STATUS records that offset; it is re-measured here)."""
    bodies = model.getBodySet()

    def point(name):
        p = bodies.get(name).getPositionInGround(state)
        return [p.get(i) for i in range(3)]

    def hip_mid():
        return [(a + b) / 2 for a, b in zip(point("femur_r"), point("femur_l"))]

    M.set_pose(model, state, {})
    reference = {"pelvis_origin": math.dist(point("pelvis"), point("talus_r")),
                 "hip_mid": math.dist(hip_mid(), point("talus_r"))}
    offset = math.dist(point("pelvis"), hip_mid())

    best = {k: (9.0, None) for k in reference}
    for hip in range(-30, 121, 5):
        for knee in range(0, 141, 5):
            for ankle in (-40, -20, 0, 15, 30):
                M.set_pose(model, state, {
                    "hip_flexion_r": math.radians(hip),
                    "knee_angle_r": math.radians(knee),
                    "ankle_angle_r": math.radians(ankle)})
                values = {"pelvis_origin": math.dist(point("pelvis"), point("talus_r")),
                          "hip_mid": math.dist(hip_mid(), point("talus_r"))}
                for key, value in values.items():
                    ratio = value / reference[key]
                    if ratio < best[key][0]:
                        best[key] = (ratio, (hip, knee, ankle))
    return best, offset


def main():
    model, state, _ = M.load_model()

    sys.stdout.write("1. COORDINATE COVERAGE of the pose grid (%d poses)\n"
                     % len(P.build_poses(limits=M.limits_degrees(model))))
    sys.stdout.write("   %-16s %14s %16s %8s %7s %6s\n"
                     % ("coordinate", "clamped range", "grid range",
                        "covered", "step", "levels"))
    for name, (low, high, lo, hi, covered, step, levels) in \
            coordinate_coverage(model).items():
        sys.stdout.write("   %-16s %6.1f..%-6.1f %7.1f..%-7.1f %7.1f%% %6.1f %6d\n"
                         % (name, low, high, lo, hi, 100 * covered, step, levels))

    best, offset = fold_extremes(model, state)
    sys.stdout.write("\n2. LEG FOLD: pelvis-to-ankle over the straight-leg value\n")
    sys.stdout.write("   model pelvis origin is %.4f m from the hip-centre midpoint\n" % offset)
    for key, (ratio, at) in best.items():
        sys.stdout.write("   model minimum (%-13s) %.3f at hip/knee/ankle %s\n"
                         % (key, ratio, at))
    worst_clip = 1.0
    for clip_id in CLIP_IDS:
        values = clip_ratios(clip_id)
        worst_clip = min(worst_clip, min(values))
        sys.stdout.write("   clip %-10s frames %4d   min %.3f  median %.3f\n"
                         % (clip_id, len(values) // 2, min(values),
                            sorted(values)[len(values) // 2]))

    model_floor = min(v[0] for v in best.values())
    sys.stdout.write("\n   OBSERVATION, not a gate: the clips fold to %.3f and FullBody.osim\n"
                     % worst_clip)
    sys.stdout.write("   cannot fold below %.3f ANYWHERE in its clamped sagittal range,\n"
                     % model_floor)
    sys.stdout.write("   under either endpoint definition (%.3f / %.3f), so the 9.7 cm\n"
                     % (best["pelvis_origin"][0], best["hip_mid"][0]))
    sys.stdout.write("   pelvis-origin offset does not explain the gap. Three candidates\n")
    sys.stdout.write("   remain and this script cannot separate them: the clip's own\n")
    sys.stdout.write("   maximum may overstate a straight leg (which would make the gap\n")
    sys.stdout.write("   WIDER, not narrower); a single noisy frame may inflate that\n")
    sys.stdout.write("   maximum; or MHRRetarget emits leg configurations outside the\n")
    sys.stdout.write("   model's joint limits, in which case IK is clamping on those\n")
    sys.stdout.write("   frames. Recorded in STATUS.md, not acted on here.\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
