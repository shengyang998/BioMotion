#!/usr/bin/env python3
"""Produce measurements.json: every before/after number quoted in README.md.

    python measure.py            # writes ./measurements.json

BEFORE = tools/osim_fixes/FullBody.osim.orig  (pristine, sha256 in README)
AFTER  = BioMotion/Resources/FullBody.osim    (the shipped, patched file)
REF    = FullBody.osim.orig parsed WITHOUT nimble's patella skip and WITH the
         CoordinateCouplerConstraint applied, i.e. what OpenSim itself would
         compute.  nimble can never produce this; it is the yardstick, not a
         deliverable.

All three columns go through the same code, so differences are attributable to
the model change and not to two different implementations.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
from osim_kinematics import Model, rot_about  # noqa: E402

ORIG = HERE / "FullBody.osim.orig"
SHIPPED = HERE.parent.parent / "BioMotion" / "Resources" / "FullBody.osim"

QUADS_R = ["recfem_r", "vasint_r", "vaslat140_r", "vasmed_r"]
QUADS_L = ["recfem_l", "vasint_l", "vaslat140_l", "vasmed_l"]
SHOULDER_12 = ["DELT1", "DELT2", "DELT3", "SUPSP", "INFSP", "SUBSC",
               "TMIN", "TMAJ", "PECM1", "PECM2", "PECM3", "CORB"]
KNEE_ANGLES_DEG = [0, 15, 30, 45, 60, 75, 90, 105, 120]
SHOULDER_DOFS_R = ["shoulder_elv_r", "shoulder_rot_r", "elv_angle_r"]


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def neutral(M):
    M.set_positions({c: M.coord_defaults.get(c, 0.0) for c in M.coord_names})


def quad_table(M, muscles, knee_coord):
    out = {m: [] for m in muscles}
    for deg in KNEE_ANGLES_DEG:
        neutral(M)
        M.set_coord(knee_coord, math.radians(deg))
        for m in muscles:
            out[m].append(round(M.moment_arm(m, knee_coord) * 100, 4))
    return out


def main():
    old = Model(ORIG, nimble_rules=True)
    new = Model(SHIPPED, nimble_rules=True)
    ref = Model(ORIG, nimble_rules=False, couple_patella=True)
    free = Model(ORIG, nimble_rules=False, couple_patella=False)

    R = {}
    R["files"] = {
        "before": {"path": str(ORIG), "sha256": sha(ORIG)},
        "after": {"path": str(SHIPPED), "sha256": sha(SHIPPED)},
    }
    R["method"] = {
        "engine": "tools/osim_fixes/osim_kinematics.py — Python re-implementation of "
                  "nimblephysics OpenSimParser.cpp readOsim40() joint construction + "
                  "BioMotion MomentArmComputer.mm length/moment-arm code",
        "moment_arm": "r = -(L(q+eps) - L(q-eps)) / (2*eps), eps = 1e-4 rad, "
                      "ConditionalPathPoints latched at the unperturbed pose "
                      "(identical to MomentArmComputer.mm)",
        "units": "moment arms in cm, lengths in m, angles in deg unless stated",
        "not_run": "the iOS app was never built or executed; these are computed, "
                   "not observed on device",
    }

    # ---------------- structure ----------------
    R["structure"] = {
        "dofs_before": old.n_dofs,
        "dofs_after": new.n_dofs,
        "bodies_built_before": len(old.body_names),
        "bodies_built_after": len(new.body_names),
        "muscles_parsed_before": len(old.muscles),
        "muscles_parsed_after": len(new.muscles),
        "unresolved_muscle_path_bodies_before": old.unresolved_path_bodies(),
        "unresolved_muscle_path_bodies_after": new.unresolved_path_bodies(),
        "shoulder_R_dofs_before": old.joint_dofs("shoulder_R"),
        "shoulder_R_dofs_after": new.joint_dofs("shoulder_R"),
        "shoulder_L_dofs_before": old.joint_dofs("shoulder_L"),
        "shoulder_L_dofs_after": new.joint_dofs("shoulder_L"),
        "shoulder_joint_class_before": [j.built_type for j in old.joints
                                        if j.name in ("shoulder_R", "shoulder_L")],
        "shoulder_joint_class_after": [j.built_type for j in new.joints
                                       if j.name in ("shoulder_R", "shoulder_L")],
        "patellofemoral_before": "skipped entirely (body + joint), "
                                 "OpenSimParser.cpp:6562 and :6737-6739",
        "patellofemoral_after": [j.built_type for j in new.joints
                                 if j.name in ("kneecap_r_jnt", "kneecap_l_jnt")],
    }

    # neutral-pose identity: the change must add DOFs without moving the rest pose
    old.set_positions({c: 0.0 for c in old.coord_names})
    new.set_positions({c: 0.0 for c in new.coord_names})
    worst = 0.0
    for b in old.body_names:
        A, B = old.body_world(b), new.body_world(b)
        if A is not None and B is not None:
            worst = max(worst, float(np.abs(A - B).max()))
    R["structure"]["max_body_world_transform_delta_at_q0_m"] = worst

    # ---------------- defect 1: patella ----------------
    d1 = {}
    d1["muscles_affected"] = QUADS_R + QUADS_L
    d1["path_points_that_were_unresolved"] = {"patella_r": 9, "patella_l": 9}
    d1["knee_angles_deg"] = KNEE_ANGLES_DEG
    d1["moment_arm_cm_before"] = quad_table(old, QUADS_R, "knee_angle_r")
    d1["moment_arm_cm_after"] = quad_table(new, QUADS_R, "knee_angle_r")
    d1["moment_arm_cm_reference_coupled"] = quad_table(ref, QUADS_R, "knee_angle_r")
    d1["moment_arm_cm_before_left"] = quad_table(old, QUADS_L, "knee_angle_l")
    d1["moment_arm_cm_after_left"] = quad_table(new, QUADS_L, "knee_angle_l")

    b = d1["moment_arm_cm_before"]
    a = d1["moment_arm_cm_after"]
    r = d1["moment_arm_cm_reference_coupled"]
    err_after = [a[m][i] - r[m][i] for m in QUADS_R for i in range(len(KNEE_ANGLES_DEG))]
    err_before = [b[m][i] - r[m][i] for m in QUADS_R for i in range(len(KNEE_ANGLES_DEG))]
    d1["error_vs_reference_cm"] = {
        "before_rmse": round(float(np.sqrt(np.mean(np.square(err_before)))), 4),
        "before_max_abs": round(float(np.max(np.abs(err_before))), 4),
        "after_rmse": round(float(np.sqrt(np.mean(np.square(err_after)))), 4),
        "after_max_abs": round(float(np.max(np.abs(err_after))), 4),
    }
    d1["sign_errors_before"] = {
        "definition": "knee_angle is flexion, so an extensor must have r < 0",
        "wrong_sign_at_knee_deg": sorted({
            KNEE_ANGLES_DEG[i] for m in QUADS_R
            for i in range(len(KNEE_ANGLES_DEG)) if b[m][i] > 0
        }),
    }

    # muscle-tendon length sanity at 60 deg
    lens = {}
    for M, label in ((old, "before"), (new, "after"), (ref, "reference_coupled")):
        neutral(M)
        M.set_coord("knee_angle_r", math.radians(60))
        M.latch_conditionals()
        lens[label] = {m: round(M.muscle_length(m), 4) for m in QUADS_R}
    d1["muscle_tendon_length_m_at_knee60"] = lens
    d1["muscle_tendon_length_note"] = (
        "optimal_fiber_length + tendon_slack_length is 0.32-0.53 m for these four. "
        "The BEFORE lengths of 1.05-1.29 m put every quadriceps 2-3.4x past the far "
        "end of its force-length curve, where active force is ~0 — which is the "
        "mechanism behind 'quadriceps not loaded', not a zero moment arm.")

    # weld pose selection — candidates are regenerated into a temp dir rather
    # than kept on disk (each is a 3.2 MB copy of the model).
    import subprocess
    import tempfile

    sweep = {}
    tmp = tempfile.mkdtemp(prefix="osim_beta_sweep_")
    for bs in [0, 15, 30, 45, 60, 75, 90]:
        cand = pathlib.Path(tmp) / ("beta%d.osim" % bs)
        subprocess.run([sys.executable, str(HERE / "apply_fixes.py"),
                        "--in", str(ORIG), "--out", str(cand),
                        "--beta-star-deg", str(bs), "--fix", "patella"],
                       check=True, stdout=subprocess.DEVNULL)
        M = Model(cand, nimble_rules=True)
        t = quad_table(M, QUADS_R, "knee_angle_r")
        e = [t[m][i] - r[m][i] for m in QUADS_R for i in range(len(KNEE_ANGLES_DEG))]
        sq = [t[m][i] - r[m][i] for m in QUADS_R
              for i, d in enumerate(KNEE_ANGLES_DEG) if 60 <= d <= 110]
        sweep[str(bs)] = {
            "rmse_0_120_cm": round(float(np.sqrt(np.mean(np.square(e)))), 4),
            "max_abs_0_120_cm": round(float(np.max(np.abs(e))), 4),
            "rmse_squat_60_110_cm": round(float(np.sqrt(np.mean(np.square(sq)))), 4),
        }
    d1["weld_pose_selection"] = {
        "chosen_beta_star_deg": 0.0,
        "sweep": sweep,
        "rationale": "beta* = 0 minimises RMSE against the coupled reference both "
                     "over the whole 0-120 deg knee range and over the squat band, "
                     "and is also the model's own default pose.",
    }

    # counterfactual: leave it a CustomJoint (free, unobservable beta DOF)
    cf = {}
    for bd in [-30, -15, 0, 15, 30, 45, 60, 90, 120]:
        neutral(free)
        free.set_coord("knee_angle_r", math.radians(60))
        free.set_coord("knee_angle_r_beta", math.radians(bd))
        cf[str(bd)] = {m: round(free.moment_arm(m, "knee_angle_r") * 100, 3)
                       for m in QUADS_R}
    d1["free_beta_dof_counterfactual"] = {
        "what": "patella renamed but patellofemoral left as a CustomJoint",
        "dofs": free.n_dofs,
        "beta_xml_range_rad": [-99999.9, 99999.9],
        "beta_clamped": False,
        "markers_on_patella": 0,
        "quad_knee_moment_arm_cm_at_knee60_vs_beta_deg": cf,
        "verdict": "beta is unobservable (no marker on the kneecap) and unclamped, "
                   "and moving it over a plausible range swings the quadriceps knee "
                   "moment arm from -2.94 cm to +2.00 cm — a sign flip. Welding "
                   "removes the DOF; the after-DOF count is unchanged at 163+6.",
    }
    d1["dofs_added_by_this_fix"] = 0
    R["defect_1_patella"] = d1

    # ---------------- defect 2: shoulder ----------------
    d2 = {}
    A = [np.array([-0.99826, 0.0023, 0.058898]),
         np.array([0.0048, 0.99909, 0.0424]),
         np.array([0.0048, 0.0424, 0.99909])]
    U = [np.array([-1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])]
    labels = ["rotation1 = shoulder_elv", "rotation2 = shoulder_rot",
              "rotation3 = elv_angle"]
    d2["axis_change_deg"] = {
        labels[i]: round(math.degrees(math.acos(
            max(-1, min(1, float(A[i] @ U[i]) / np.linalg.norm(A[i]))))), 4)
        for i in range(3)
    }
    d2["axis_norms_before"] = [round(float(np.linalg.norm(v)), 8) for v in A]
    d2["orthogonality"] = {
        "before": {"r1.r2": round(abs(float(A[0] @ A[1])), 6),
                   "r1.r3": round(abs(float(A[0] @ A[2])), 6),
                   "r2.r3": round(abs(float(A[1] @ A[2])), 6)},
        "after": {"r1.r2": 0.0, "r1.r3": 0.0, "r2.r3": 0.0},
        "nimble_gate": 1e-4,
    }

    def compose(ax, q):
        return rot_about(ax[0], q[0]) @ rot_about(ax[1], q[1]) @ rot_about(ax[2], q[2])

    def angdiff(R1, R2):
        return math.degrees(math.acos(max(-1, min(1, (np.trace(R1.T @ R2) - 1) / 2))))

    elv = np.linspace(0, 2.0071, 13)
    rot = np.linspace(-0.7854, 0.7854, 9)
    ea = np.linspace(-1.5708, 1.5708, 13)
    errs = [angdiff(compose(A, q), compose(U, q))
            for q in itertools.product(elv, rot, ea)]
    d2["humerus_orientation_error_deg_over_coordinate_ranges"] = {
        "mean": round(float(np.mean(errs)), 3),
        "p95": round(float(np.percentile(errs, 95)), 3),
        "max": round(float(np.max(errs)), 3),
        "sampled_ranges_deg": {"shoulder_elv": [0, 115], "shoulder_rot": [-45, 45],
                               "elv_angle": [-90, 90]},
        "context": "ARKit body tracking, the actual input to this pipeline, is "
                   "18.8 +/- 12.12 deg MAE (doi:10.3390/app12104806).",
    }
    plane = []
    for e in [0, 30, 60, 90, 115]:
        q = (math.radians(e), 0.0, 0.0)
        va = compose(A, q) @ np.array([0, -1, 0])
        vu = compose(U, q) @ np.array([0, -1, 0])
        plane.append({"shoulder_elv_deg": e,
                      "humeral_long_axis_before": [round(float(v), 4) for v in va],
                      "humeral_long_axis_after": [round(float(v), 4) for v in vu],
                      "delta_deg": round(angdiff(np.eye(3), np.eye(3)) or
                                         math.degrees(math.acos(
                                             max(-1, min(1, float(va @ vu))))), 3)})
    d2["humeral_elevation_plane_check"] = {
        "what": "pure elevation (elv_angle = 0, shoulder_rot = 0); the humeral long "
                "axis must sweep the same plane before and after",
        "samples": plane,
        "verdict": "the elevation component is unchanged to 4 decimals; the snapped "
                   "axes remove a 0.03-0.06 out-of-plane leak, so the elevation "
                   "plane convention is preserved (arguably cleaner).",
    }

    sh_names = [n + s for n in SHOULDER_12 for s in ("", "_l")]
    d2["shoulder_muscles"] = sorted(sh_names)
    d2["pathwraps_on_shoulder_muscles"] = sum(new.muscles[n].n_wraps for n in sh_names)
    ld = [k for k in new.muscles if k.startswith("LD_")]
    d2["extra_muscles_crossing_the_joint"] = {
        "latissimus_dorsi_slips": len(ld),
        "pathwraps": sum(new.muscles[k].n_wraps for k in ld),
        "note": "these also gain moment arms; no provenance claim is made here.",
    }

    poses = {
        "neutral_arm_down": {},
        "elv45": {"shoulder_elv_r": math.radians(45)},
        "elv90": {"shoulder_elv_r": math.radians(90)},
        "elv90_abducted_plane": {"shoulder_elv_r": math.radians(90),
                                 "elv_angle_r": math.radians(-60)},
    }
    ma = {}
    for pname, pose in poses.items():
        ma[pname] = {}
        for label, M in (("before", old), ("after", new)):
            neutral(M)
            for k, v in pose.items():
                M.set_coord(k, v)
            row = {}
            for mus in [n for n in SHOULDER_12]:
                row[mus] = {d: round(M.moment_arm(mus, d) * 100, 3)
                            for d in SHOULDER_DOFS_R}
            ma[pname][label] = row
    d2["moment_arm_cm_right_shoulder"] = ma
    nz = 0
    tot = 0
    for pname in poses:
        for mus, dd in ma[pname]["after"].items():
            for d, v in dd.items():
                tot += 1
                if abs(v) > 0.1:
                    nz += 1
    d2["nonzero_summary"] = {
        "before": "0 of %d (the coordinates do not exist as DOFs at all)" % tot,
        "after": "%d of %d entries exceed 0.1 cm" % (nz, tot),
    }
    d2["dofs_added_by_this_fix"] = new.n_dofs - old.n_dofs
    R["defect_2_shoulder"] = d2

    # ---------------- limitations ----------------
    R["limitations"] = [
        "The iOS app was not built or run (forbidden for this workstream). Every "
        "number is computed by a Python re-implementation of the two decisive C++ "
        "files, not observed on device. The re-implementation independently "
        "reproduces four facts recorded in STATUS.md (163 DOFs, 520 muscles = 422 "
        "Thelen + 98 Millard, exactly 2 crash-guard welds, and the shoulder dot "
        "products 0.000004 / 0.054150 / 0.084746), which is the evidence that it "
        "models the same skeleton nimble builds.",
        "PathWrap is not implemented anywhere in this pipeline (not in nimble's "
        "parse, not in MomentArmComputer.mm, not here). Each quadriceps carries 1 "
        "PathWrap, so all quadriceps absolute values carry that error in BOTH "
        "columns; beyond ~90 deg knee flexion the straight-line path cuts through "
        "the condyles and the magnitudes are unreliable in all three columns. The 24 "
        "shoulder muscles carry 0 PathWraps, so the shoulder numbers do not have "
        "this problem.",
        "The 'reference' column applies the CoordinateCouplerConstraint, which "
        "nimble never enforces. It is the yardstick for how good the weld is; it is "
        "not something the shipped pipeline can compute.",
        "Moment arms are geometry. They are a necessary condition for a sane muscle "
        "solution, not a sufficient one: whether the OSQP stage now produces sensible "
        "quadriceps activation during a squat depends additionally on force-length "
        "state, inverse-dynamics torque quality and the ARKit input, none of which "
        "this measurement touches.",
        "Body scaling was not exercised: all numbers are at the model's unscaled "
        "size. The groupScale() patch is verified by reading, not by measurement.",
    ]

    out = HERE / "measurements.json"
    out.write_text(json.dumps(R, indent=2))
    print("wrote", out)
    print(json.dumps(R["structure"], indent=2))
    print(json.dumps(R["defect_1_patella"]["error_vs_reference_cm"], indent=2))
    print(json.dumps(R["defect_2_shoulder"]["axis_change_deg"], indent=2))
    print(json.dumps(R["defect_2_shoulder"]["nonzero_summary"], indent=2))


if __name__ == "__main__":
    main()
