#!/usr/bin/env python3
"""
Historical quasi-static inverse-dynamics diagnostic for the RIGHT LEG of
FullBody.osim under the two candidate gravity vectors. READ-ONLY; no Xcode,
no build.

It checks gravity/frame algebra, not foot-support validity. FullBody's
ContactGeometrySet is empty and the near-CoP routine has no validated support
polygon, unilateral-contact, or friction constraint. Do not interpret the
reported torques/forces as product measurements.

The measured pose has max_ddq = 1.7e-16 and max_dq ~ 0, and FullBody.osim
declares no <stiffness> and no <damping>, and BioMotion never calls
setExtWrench.  So nimble's ID reduces exactly to

    tau = G(q)  -  J^T x

with G the gravity generalised force and x the contact wrench.  For a single
support foot, whole-body equilibrium (which is precisely the 6-row equality
constraint that getMultipleContactInverseDynamicsNearCoP enforces, see
Skeleton.cpp:9976 and :10361-10364) fixes the contact wrench completely:

    f       = -sum_k m_k g
    m_free  = -(p_contact - CoM) x f

so the moment carried at joint j is exactly

    M_j = sum_{k in subtree(j)} (c_k - o_j) x m_k g  -  [ (p - o_j) x f + m_free ]

and the reported per-DOF torque is M_j . axis_j.  No solver freedom is involved
in the *total* wrench; the 400-step CoP walk only moves within the null space,
which for a single contact is empty.  So this calculation is exact for the
single-support case the test hits (grf_N = 780 = full bodyweight => contactCount
== 1).

Measured, from BioMotionTests/OfflineMuscleChainTests on the real dancer pose:
    subtalar_angle_r 672   ankle_angle_r 472   knee_angle_r 313
    hip_adduction_r  137   hip_flexion_r 106
"""

import math
import xml.etree.ElementTree as ET

import numpy as np

OSIM = "/Users/soleilyu/claude_playground/labs/BioMotion/BioMotion/Resources/FullBody.osim"
JOINT_TAGS = ("CustomJoint", "PinJoint", "WeldJoint", "BallJoint",
              "FreeJoint", "SliderJoint", "UniversalJoint")


def rot(rx, ry, rz):
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return (np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
            @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
            @ np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]))


def iso(R, p):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def vec(t):
    return np.array([float(v) for v in t.split()])


def parse():
    root = ET.parse(OSIM).getroot()
    bodies = {}
    for b in root.iter("Body"):
        m = b.find("mass")
        if m is None:
            continue
        c = b.find("mass_center")
        bodies[b.get("name")] = {"mass": float(m.text),
                                 "com": vec(c.text) if c is not None else np.zeros(3)}
    joints = {}
    for tag in JOINT_TAGS:
        for j in root.iter(tag):
            pf, cf = j.find("socket_parent_frame"), j.find("socket_child_frame")
            if pf is None or cf is None:
                continue
            local = {}
            for f in j.iter("PhysicalOffsetFrame"):
                par = f.find("socket_parent")
                if par is None:
                    continue
                t, o = f.find("translation"), f.find("orientation")
                local.setdefault(f.get("name"), (
                    par.text.strip().split("/")[-1],
                    iso(rot(*(vec(o.text) if o is not None else np.zeros(3))),
                        vec(t.text) if t is not None else np.zeros(3))))
            pn, cn = pf.text.strip().split("/")[-1], cf.text.strip().split("/")[-1]
            if pn not in local or cn not in local:
                continue
            pb, Tp = local[pn]
            cb, Tc = local[cn]
            # rotation axes: PinJoint spins about the child-frame Z; a CustomJoint
            # lists its rotation axes explicitly in <SpatialTransform>.
            axes = []
            if tag == "PinJoint":
                axes = [("__pin__", np.array([0.0, 0.0, 1.0]))]
            else:
                for ta in j.iter("TransformAxis"):
                    ax = ta.find("axis")
                    co = ta.find("coordinates")
                    nm = ta.get("name") or ""
                    if ax is None or not nm.startswith("rotation"):
                        continue
                    axes.append(((co.text.strip() if co is not None and co.text else nm),
                                 vec(ax.text)))
            joints[j.get("name")] = {"type": tag, "parent_body": pb, "child_body": cb,
                                     "T_parent_off": Tp, "T_child_off": Tc, "axes": axes}
    return bodies, joints


def main():
    bodies, joints = parse()

    world = {"ground": np.eye(4)}
    frame_of_joint = {}          # joint -> world transform of its (child) frame
    changed = True
    while changed:
        changed = False
        for jn, j in joints.items():
            pb, cb = j["parent_body"], j["child_body"]
            if pb in world and cb not in world:
                Tframe = world[pb] @ j["T_parent_off"]
                world[cb] = Tframe @ np.linalg.inv(j["T_child_off"])
                frame_of_joint[jn] = Tframe
                changed = True

    children = {}
    for jn, j in joints.items():
        children.setdefault(j["parent_body"], []).append(j["child_body"])

    def subtree(body):
        out, stack = [], [body]
        while stack:
            b = stack.pop()
            out.append(b)
            stack.extend(children.get(b, []))
        return out

    M = sum(b["mass"] for b in bodies.values())
    com = sum(b["mass"] * (world[n][:3, :3] @ b["com"] + world[n][:3, 3])
              for n, b in bodies.items() if n in world) / M

    p_contact = world["calcn_r"][:3, 3]          # nimble applies the wrench at the
                                                 # BODY ORIGIN (MetaSkeleton.hpp:559)
    chain = [("hip_r", "femur_r"), ("walker_knee_r", "tibia_r"),
             ("ankle_r", "talus_r"), ("subtalar_r", "calcn_r")]
    measured = {"hip_r": {"hip_flexion_r": 106.0, "hip_adduction_r": 137.0},
                "walker_knee_r": {"knee_angle_r": 313.0},
                "ankle_r": {"ankle_angle_r": 472.0},
                "subtalar_r": {"subtalar_angle_r": 672.0}}

    print(f"total mass {M:.3f} kg,  CoM {np.round(com,4)},  contact at calcn_r origin "
          f"{np.round(p_contact,4)}\n")

    for label, g in (("A  gravity = (0,-9.81,0)   [the .osim value, and what every "
                      "nimble biomech entry point sets]", np.array([0., -9.81, 0.])),
                     ("B  gravity = (0,0,-9.81)   [DART's default when nobody calls "
                      "setGravity]", np.array([0., 0., -9.81]))):
        f = -M * g                                   # total contact force
        m_free = -np.cross(p_contact - com, f)       # whole-body moment balance
        print("=" * 78)
        print("HYPOTHESIS " + label)
        print(f"  contact force  = {np.round(f,1)}  (|f| = {np.linalg.norm(f):.1f} N)")
        print(f"  free moment    = {np.round(m_free,1)}  (|m| = {np.linalg.norm(m_free):.1f} Nm)")
        print(f"  {'joint':15s} {'|M_j| Nm':>10s}   per-DOF projections vs measured")
        for jn, child in chain:
            o = world[child][:3, 3]
            Mj = -(np.cross(p_contact - o, f) + m_free)
            for b in subtree(child):
                cw = world[b][:3, :3] @ bodies[b]["com"] + world[b][:3, 3]
                Mj = Mj + np.cross(cw - o, bodies[b]["mass"] * g)
            R = frame_of_joint[jn][:3, :3]
            parts = []
            for nm, ax in joints[jn]["axes"]:
                aw = R @ ax
                aw = aw / np.linalg.norm(aw)
                nm2 = nm if nm != "__pin__" else jn.replace("_r", "_angle_r")
                got = measured[jn].get(nm2)
                parts.append(f"{nm2}={Mj @ aw:+8.1f}" +
                             (f" (measured {got:.0f})" if got else ""))
            print(f"  {jn:15s} {np.linalg.norm(Mj):10.1f}   " +
                  ("\n" + " " * 30).join(parts))
        print()


if __name__ == "__main__":
    main()
