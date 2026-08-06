#!/usr/bin/env python3
"""Which of the 520 muscles actually changed, and did the hard constraints hold?

The iOS test suite cannot be run from this workstream, so this stands in for the
"did I break something else" question. It compares every muscle's path length
before vs after at four poses, and asserts that the only muscles that move are
the 8 quadriceps and (when the shoulder is actually posed) the muscles that cross
the glenohumeral joint.

    python regression.py        # prints a table; exits non-zero on any surprise
"""

from __future__ import annotations

import math
import pathlib
import sys

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
from osim_kinematics import Model  # noqa: E402

ORIG = HERE / "FullBody.osim.orig"
SHIPPED = HERE.parent.parent / "BioMotion" / "Resources" / "FullBody.osim"

QUADS = {"recfem_r", "vasint_r", "vaslat140_r", "vasmed_r",
         "recfem_l", "vasint_l", "vaslat140_l", "vasmed_l"}
ARM_BODIES = {"humerus_r", "humerus_l", "radius_r", "radius_l",
              "ulna_r", "ulna_l", "hand_r", "hand_l"}

POSES = {
    "neutral": {},
    "squat": {"hip_flexion_r": 1.05, "hip_flexion_l": 1.05,
              "knee_angle_r": 1.75, "knee_angle_l": 1.75,
              "ankle_angle_r": 0.35, "ankle_angle_l": 0.35, "L5_S1_FE": 0.3},
    "arms_up": {"shoulder_elv_r": 1.5, "shoulder_elv_l": 1.5},
    "trunk_twist": {"L5_S1_AR": 0.2, "hip_flexion_r": 0.4},
}


def main():
    old = Model(ORIG, nimble_rules=True)
    new = Model(SHIPPED, nimble_rules=True)
    gh = {n for n, m in new.muscles.items()
          if {p.body for p in m.points} & ARM_BODIES}
    expected = QUADS | gh
    print("muscles parsed: before %d, after %d" % (len(old.muscles), len(new.muscles)))
    print("glenohumeral-crossing muscles: %d   quadriceps: %d" % (len(gh), len(QUADS)))

    bad = 0
    print("\n%-13s %10s %12s %14s" % ("pose", "changed", "unexpected", "max |dL| (m)"))
    for pname, pose in POSES.items():
        for M in (old, new):
            M.set_positions({c: M.coord_defaults.get(c, 0.0) for c in M.coord_names})
            for k, v in pose.items():
                if k in M.q:
                    M.set_coord(k, v)
            M.latch_conditionals()
        diffs = {}
        for n in new.muscles:
            d = abs(new.muscle_length(n) - old.muscle_length(n))
            if d > 1e-9:
                diffs[n] = d
        unexpected = {k: v for k, v in diffs.items() if k not in expected}
        bad += len(unexpected)
        print("%-13s %10d %12d %14.5f" % (
            pname, len(diffs), len(unexpected),
            max(diffs.values()) if diffs else 0.0))
        for k, v in sorted(unexpected.items(), key=lambda x: -x[1])[:10]:
            print("     UNEXPECTED %s %.6f" % (k, v))

    # Hard constraints, checked structurally rather than by substring count
    # (a plain grep for "rib" also matches "prescribed" and "described").
    print()
    ob = set(old.body_names)
    nb = set(new.body_names)
    oj = {j.name for j in old.joints}
    nj = {j.name for j in new.joints}
    oc = set(old.coord_names)
    nc = set(new.coord_names)
    checks = [
        ("bodies added", nb - ob, {"kneecap_r", "kneecap_l"}),
        ("bodies removed", ob - nb, set()),
        ("joints added", nj - oj, {"kneecap_r_jnt", "kneecap_l_jnt"}),
        ("joints removed", oj - nj, set()),
        ("coordinates added", nc - oc, {"shoulder_elv_r", "shoulder_rot_r",
                                        "elv_angle_r", "shoulder_elv_l",
                                        "shoulder_rot_l", "elv_angle_l"}),
        ("coordinates removed", oc - nc, set()),
    ]
    for label, got, want in checks:
        ok = got == want
        print("  %-20s %-56s %s" % (label, sorted(got), "OK" if ok else "UNEXPECTED"))
        if not ok:
            bad += 1

    # the shoulder girdle's only remaining articulation must still be welded,
    # and every rib / thoracic / sternum body must still be present and unmoved
    girdle = {"sterR_clavR_jnt", "clavR_scapR_jnt", "sterL_clavL_jnt", "clavL_scapL_jnt"}
    for j in new.joints:
        if j.name in girdle and j.built_type != "Weld":
            print("  girdle joint %s is now %s" % (j.name, j.built_type))
            bad += 1
    print("  girdle welds intact:", sorted(girdle & nj))
    spine = sorted(b for b in nb if b.startswith(("rib", "thoracic", "lumbar", "sternum",
                                                  "sacrum", "Abd")))
    print("  spine/ribcage bodies present: %d (before %d)"
          % (len(spine), len([b for b in ob if b.startswith(
              ("rib", "thoracic", "lumbar", "sternum", "sacrum", "Abd"))])))
    for M in (old, new):
        M.set_positions({c: M.coord_defaults.get(c, 0.0) for c in M.coord_names})
    worst = max((abs(new.body_world(b) - old.body_world(b)).max() for b in spine),
                default=0.0)
    print("  max |dT| across spine/ribcage bodies at q=0: %.3e" % worst)
    if worst > 0:
        bad += 1

    trap = [k for k in new.muscles if k.lower().startswith("trap")]
    ser = [k for k in new.muscles if k.lower().startswith("ser")]
    for M in (old, new):
        M.set_positions({c: M.coord_defaults.get(c, 0.0) for c in M.coord_names})
        M.latch_conditionals()
    moved = [k for k in trap + ser
             if abs(new.muscle_length(k) - old.muscle_length(k)) > 0]
    print("  trapezius slips=%d serratus slips=%d  moved=%d %s"
          % (len(trap), len(ser), len(moved), moved))
    bad += len(moved)

    print("\n%s" % ("PASS" if bad == 0 else "FAIL: %d surprises" % bad))
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
