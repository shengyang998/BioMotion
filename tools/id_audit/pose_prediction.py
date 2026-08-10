#!/usr/bin/env python3
"""
Predict the RIGHT-LEG joint moments for the ACTUAL test pose under both gravity
vectors, and compare with the measured torques.  READ-ONLY, no build.

Pose = OfflineMuscleChainFixture.markers (BioMotionTests/OfflineOrchestrationTests.swift:129),
i.e. the real SAM 3D Body output that OfflineMuscleChainTests feeds through IK.
Segment masses = FullBody.osim's own values, lumped onto marker-defined segments
(the model's own default-pose local CoM offsets are small relative to the effect
being measured, so segment CoMs are placed at segment midpoints).

max_ddq = 1.7e-16 and max_dq ~ 0, FullBody.osim declares no <stiffness>/<damping>,
and BioMotion never calls setExtWrench, so nimble's ID reduces exactly to
tau = G(q) - J^T x.  Only ONE foot is in contact (grf_N = 780 = full bodyweight,
NimbleBridge.mm:1060/1077 splits the guess by contactCount), and for a single
contact jacBlock is a square invertible 6x6 adjoint, so the 6-row equality
constraint at Skeleton.cpp:9976 determines the contact wrench UNIQUELY: the
wrench-guess and the 400-step CoP walk have no effect at all on this frame.
Whole-body equilibrium therefore gives the exact answer:

    f      = -M g
    m_free = -(p_contact - CoM) x f
    M_j    = sum_{k in subtree(j)} (c_k - o_j) x m_k g  -  [(p - o_j) x f + m_free]
"""

import numpy as np

W_MARKERS = {
    "MHR_ROOT": (0.000000, 0.923987, 0.000000),
    "LHJC": (0.049532, 0.940746, -0.059429),
    "RHJC": (-0.026827, 0.888276, 0.065355),
    "LKJC": (0.381692, 1.159680, 0.103828),
    "RKJC": (-0.016599, 0.460764, 0.110560),
    "LAJC": (0.271441, 0.770530, 0.026491),
    "RAJC": (-0.161081, 0.080435, 0.046929),
    "LTOE": (0.307562, 0.632544, 0.055199),
    "RTOE": (-0.106420, -0.043160, 0.104657),
    "SPINE_L": (-0.089250, 1.038710, -0.005759),
    "SPINE_M": (-0.240898, 1.231510, 0.003535),
    "C7": (-0.304259, 1.356519, 0.044218),
    "NECK": (-0.319732, 1.374271, 0.044922),
    "HEAD": (-0.446991, 1.441565, 0.027405),
    "LSJC": (-0.274981, 1.417246, -0.078846),
    "RSJC": (-0.372616, 1.261433, 0.128290),
    "LEJC": (-0.336363, 1.682226, -0.073540),
    "REJC": (-0.424209, 0.995907, 0.157344),
    "LWJC": (-0.577720, 1.737568, 0.012645),
    "RWJC": (-0.457117, 0.739004, 0.198106),
}
P = {k: np.array(v) for k, v in W_MARKERS.items()}

# (name, mass kg from FullBody.osim, world CoM, is-part-of-the-right-leg-subtree)
def mid(a, b, t=0.5):
    return P[a] + t * (P[b] - P[a])

SEGMENTS = [
    # --- right leg: the subtree distal to each stance-leg joint --------
    ("femur_r",   9.3014, mid("RHJC", "RKJC", 0.43), "thigh"),
    ("kneecap_r", 0.0862, mid("RHJC", "RKJC", 0.95), "thigh"),
    ("tibia_r",   3.7075, mid("RKJC", "RAJC", 0.43), "shank"),
    ("talus_r",   0.1000, P["RAJC"],                  "foot"),
    ("calcn_r",   1.2500, mid("RAJC", "RTOE", 0.45),  "foot"),
    ("toes_r",    0.2166, P["RTOE"],                  "foot"),
    # --- everything else ----------------------------------------------
    ("pelvis",    11.7770, mid("MHR_ROOT", "SPINE_L", 0.2), None),
    ("sacrum",     0.0010, P["MHR_ROOT"],  None),
    ("femur_l",    9.3014, mid("LHJC", "LKJC", 0.43), None),
    ("kneecap_l",  0.0862, mid("LHJC", "LKJC", 0.95), None),
    ("tibia_l",    3.7075, mid("LKJC", "LAJC", 0.43), None),
    ("talus_l",    0.1000, P["LAJC"], None),
    ("calcn_l",    1.2500, mid("LAJC", "LTOE", 0.45), None),
    ("toes_l",     0.2166, P["LTOE"], None),
    # lumbar 1-5 (10.30 kg) spread raw source root -> SPINE_M, thoracic 1-12 (15.10 kg)
    # spread SPINE_L -> C7 (matches the marker mapping in NimbleBridge.mm:362-365)
    ("lumbar",    10.2975, mid("SPINE_L", "SPINE_M", 0.35), None),
    ("thoracic",  15.0984, mid("SPINE_M", "C7", 0.45), None),
    ("head_neck",  5.4132, mid("NECK", "HEAD", 0.6), None),
    ("humerus_r",  2.0325, mid("RSJC", "REJC", 0.45), None),
    ("humerus_l",  2.0325, mid("LSJC", "LEJC", 0.45), None),
    ("forearm_r",  1.2150, mid("REJC", "RWJC", 0.43), None),
    ("forearm_l",  1.2150, mid("LEJC", "LWJC", 0.43), None),
    ("hand_r",     0.4575, P["RWJC"], None),
    ("hand_l",     0.4575, P["LWJC"], None),
    ("abd_ribs",   0.1050, mid("SPINE_L", "SPINE_M", 0.5), None),
]

SUBTREE = {"hip_r": {"thigh", "shank", "foot"},
           "knee_r": {"shank", "foot"},
           "ankle_r": {"foot"},
           "subtalar_r": {"foot"}}

MEASURED = {"hip_r": ("hip_flexion_r 106 / hip_adduction_r 137", 173.2),
            "knee_r": ("knee_angle_r 313", 313.0),
            "ankle_r": ("ankle_angle_r 472", 472.0),
            "subtalar_r": ("subtalar_angle_r 672", 672.0)}


def main():
    M = sum(s[1] for s in SEGMENTS)
    com = sum(s[1] * s[2] for s in SEGMENTS) / M
    print(f"lumped mass  = {M:.3f} kg   (FullBody.osim total 79.583; test 79.58)")
    print(f"pose CoM     = {np.round(com, 4)}")

    # stance foot: RAJC y = 0.080 vs LAJC y = 0.771 -> the RIGHT leg is the
    # stance leg, and grf_N = 780 = full bodyweight confirms single support.
    # nimble applies the wrench at the calcn_r BODY ORIGIN (MetaSkeleton.hpp:559).
    p = mid("RAJC", "RTOE", 0.45)
    joints = {"hip_r": P["RHJC"], "knee_r": P["RKJC"],
              "ankle_r": P["RAJC"], "subtalar_r": p}
    print(f"contact pt   = {np.round(p, 4)}  (calcn_r origin ~ heel)")
    print()

    for label, g in (("A   gravity (0,-9.81,0)  -- the .osim value; what "
                      "SubjectOnDisk.cpp:807 / DynamicsFitter.cpp:13706 set",
                      np.array([0.0, -9.81, 0.0])),
                     ("B   gravity (0,0,-9.81)  -- DART's default "
                      "(SkeletonAspect.hpp:82) when nobody calls setGravity",
                      np.array([0.0, 0.0, -9.81]))):
        f = -M * g
        m_free = -np.cross(p - com, f)
        print("=" * 80)
        print("HYPOTHESIS " + label)
        print(f"  contact force {np.round(f,1)}  |f| = {np.linalg.norm(f):.1f} N "
              f"(test reports grf_N = 780)")
        print(f"  free moment   {np.round(m_free,1)}  |m| = {np.linalg.norm(m_free):.1f} Nm")
        print(f"  {'joint':12s} {'|M_j| Nm':>10s}   measured (per-DOF, so <= |M_j|)")
        for jn, o in joints.items():
            Mj = -(np.cross(p - o, f) + m_free)
            for nm, mass, c, grp in SEGMENTS:
                if grp in SUBTREE[jn]:
                    Mj = Mj + np.cross(c - o, mass * g)
            lbl, mag = MEASURED[jn]
            print(f"  {jn:12s} {np.linalg.norm(Mj):10.1f}   {lbl}"
                  f"   (|.| = {mag:.0f}, ratio {mag/np.linalg.norm(Mj):.2f})")
        print()

    print("=" * 80)
    print("Ceiling argument (pose-independent): under correct gravity the contact")
    print("force is VERTICAL, so |M_j| = bodyweight * horizontal (CoP -> joint)")
    print("offset plus the distal segment weights. The horizontal offset cannot")
    print("exceed a foot length. 780.7 N * 0.30 m = 234 Nm is a hard ceiling for")
    print("EVERY leg joint. 672 Nm at the subtalar is 2.9x that ceiling, so no")
    print("choice of pose, CoP or contact split can produce it with gravity along -Y.")


if __name__ == "__main__":
    main()
