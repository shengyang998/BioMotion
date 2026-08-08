"""Dump OpenSim's own moment arms and path lengths for FullBody.osim.

Two models, same poses, same (muscle, coordinate) pairs:

  WRAP ON   -- the model as shipped, with all 76 PathWraps solved by OpenSim.
               This is the REFERENCE. Nothing in this repo has had one before.
  WRAP OFF  -- every WrapObject deactivated, so each path is the straight
               polyline through its path points. That is what
               `MomentArmComputer` computes today; the gap between the two
               columns is the defect that retired the per-muscle claim.

Outputs (see `--out`):
  moment_arms.csv   pose, muscle, coordinate, r_wrap_on, r_wrap_off   [metres]
  lengths.csv       pose, muscle, L_wrap_on, L_wrap_off, wrap_points  [metres]
  poses.csv         pose, <169 coordinate values as the model held them>

Run:  tools/opensim_ref/.venv/bin/python tools/opensim_ref/dump_reference.py
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import osim_model as M
import poses as P


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "out"))
    parser.add_argument("--limit-poses", type=int, default=0,
                        help="debug only: stop after N poses")
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    t0 = time.time()
    model_on, state_on, _ = M.load_model(disable_wrapping=False)
    model_off, state_off, deactivated = M.load_model(disable_wrapping=True)
    sys.stderr.write("loaded both models (%d wrap objects deactivated) in %.1fs\n"
                     % (deactivated, time.time() - t0))

    coord_names = M.coordinate_names(model_on)
    assert coord_names == M.coordinate_names(model_off)

    # The pair list comes from the WRAP-ON model: its structural span includes
    # bodies a muscle only touches through a wrap object, and the two models
    # must be evaluated on an identical pair list or the difference column is
    # comparing different sets.
    spanned = M.spanned_coordinates(model_on)

    muscles_on = model_on.getMuscles()
    muscles_off = model_off.getMuscles()
    n_muscles = muscles_on.getSize()
    muscle_names = [muscles_on.get(i).getName() for i in range(n_muscles)]
    assert muscle_names == [muscles_off.get(i).getName() for i in range(n_muscles)]

    paths_on = [muscles_on.get(i).getGeometryPath() for i in range(n_muscles)]
    paths_off = [muscles_off.get(i).getGeometryPath() for i in range(n_muscles)]

    coords_on = {}
    coords_off = {}
    cs_on, cs_off = model_on.getCoordinateSet(), model_off.getCoordinateSet()
    for i in range(cs_on.getSize()):
        coords_on[cs_on.get(i).getName()] = cs_on.get(i)
        coords_off[cs_off.get(i).getName()] = cs_off.get(i)

    pose_list = P.build_poses(limits=M.limits_degrees(model_on))
    if args.limit_poses:
        pose_list = pose_list[:args.limit_poses]
    n_pairs = sum(len(v) for v in spanned.values())
    sys.stderr.write("%d poses x %d muscles, %d spanned (muscle, coordinate) pairs\n"
                     % (len(pose_list), n_muscles, n_pairs))

    arms_path = os.path.join(args.out, "moment_arms.csv")
    lengths_path = os.path.join(args.out, "lengths.csv")
    poses_path = os.path.join(args.out, "poses.csv")

    with open(arms_path, "w") as f_arms, \
            open(lengths_path, "w") as f_len, \
            open(poses_path, "w") as f_pose:
        f_arms.write("pose,muscle,coordinate,r_wrap_on,r_wrap_off\n")
        f_len.write("pose,muscle,length_wrap_on,length_wrap_off,wrap_points\n")
        f_pose.write("pose," + ",".join(coord_names) + "\n")

        for index, (pose_id, degrees) in enumerate(pose_list):
            values = P.to_radians(degrees)
            # `Coordinate::setValue(state, v, False)` does not clamp, so a pose
            # outside the model's own declared range is held verbatim and
            # nothing downstream notices. Check it here: a moment arm at a
            # configuration the model is not defined at is not a reference.
            # 1e-6 rad = 0.2 arc-seconds of slack, because the ranges in the
            # .osim are truncated decimals: `hip_flexion_r` maxes at
            # 2.0943950999999998 while `math.radians(120)` is
            # 2.0943951023931953, i.e. the model's own limit is 119.99999986
            # deg. The slack cannot hide a real out-of-range pose; the two the
            # guard did catch were 5 deg and 25 deg out.
            for name, requested in values.items():
                c = coords_on[name]
                if not (c.getRangeMin() - 1e-6 <= requested <= c.getRangeMax() + 1e-6):
                    raise SystemExit(
                        "pose %s: %s = %.6f is outside its clamped range "
                        "[%.6f, %.6f]" % (pose_id, name, requested,
                                          c.getRangeMin(), c.getRangeMax()))
            M.set_pose(model_on, state_on, values)
            M.set_pose(model_off, state_off, values)
            held = M.actual_pose(model_on, state_on)
            held_off = M.actual_pose(model_off, state_off)
            worst = max(abs(a - b) for a, b in zip(held, held_off))
            if worst > 1e-12:
                raise SystemExit("pose %s: the two models hold different "
                                 "coordinates (max %g rad)" % (pose_id, worst))
            # A coordinate outside its clamped range comes back as something
            # else, and the pose label would then name a pose the model never
            # took. Refuse rather than record it.
            held_by_name = dict(zip(coord_names, held))
            for name, requested in values.items():
                if abs(held_by_name[name] - requested) > 1e-12:
                    raise SystemExit(
                        "pose %s: asked %s for %.6f, model held %.6f"
                        % (pose_id, name, requested, held_by_name[name]))
            f_pose.write(pose_id + "," + ",".join("%.12g" % v for v in held) + "\n")

            for m in range(n_muscles):
                name = muscle_names[m]
                f_len.write("%s,%s,%.12g,%.12g,%d\n"
                            % (pose_id, name,
                               paths_on[m].getLength(state_on),
                               paths_off[m].getLength(state_off),
                               M.wrap_point_count(paths_on[m], state_on)))
                for coordinate in spanned[name]:
                    r_on = paths_on[m].computeMomentArm(state_on, coords_on[coordinate])
                    r_off = paths_off[m].computeMomentArm(state_off, coords_off[coordinate])
                    f_arms.write("%s,%s,%s,%.12g,%.12g\n"
                                 % (pose_id, name, coordinate, r_on, r_off))

            if index % 10 == 0 or index == len(pose_list) - 1:
                sys.stderr.write("  pose %d/%d %s  (%.0fs)\n"
                                 % (index + 1, len(pose_list), pose_id, time.time() - t0))
                sys.stderr.flush()

    sys.stderr.write("wrote %s, %s, %s in %.0fs\n"
                     % (arms_path, lengths_path, poses_path, time.time() - t0))


if __name__ == "__main__":
    main()
