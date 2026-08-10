#!/usr/bin/env python3
"""
READ-ONLY numerical check for the historical BioMotion inverse-dynamics audit.

This checks gravity/frame algebra inside a raw solver. It does not validate foot
support: both bundled ContactGeometrySets are empty, and the near-CoP routine
has no support-polygon, unilateral-contact, or friction constraint. Values from
this script are engineering diagnostics, not product torque/GRF/CoP output.

Does NOT build or touch the Xcode project. Parses FullBody.osim, builds the
kinematic tree at the model's default pose (all coordinates = 0, i.e. upright
standing), and evaluates the two competing gravity hypotheses against the
measured torque profile from BioMotionTests/OfflineMuscleChainTests:

    subtalar_angle_r = 672 Nm
    ankle_angle_r    = 472 Nm
    knee_angle_r     = 313 Nm
    hip_adduction_r  = 137 Nm
    hip_flexion_r    = 106 Nm

Hypothesis A: gravity = (0, -9.81, 0)   (what OpenSim declares, what nimble's
              own biomechanics entry points set explicitly)
Hypothesis B: gravity = (0, 0, -9.81)   (DART's *default*, which is what you get
              when nobody calls Skeleton::setGravity after OpenSimParser::parseOsim)

For a quasi-static pose with contact only at the feet, whole-body equilibrium
fixes the total contact wrench: force = -m*g, and a free moment that zeroes the
moment about the body CoM.  The static moment carried at joint j is then

    M_j = m_distal(j) * g_moment_about_o_j  -  [ (p - o_j) x f + m_free ]

which, when the distal (leg) mass is small compared to bodyweight, reduces to

    |M_j| ~ W * |offset from joint j to the CoM, PERPENDICULAR to g|

i.e. gravity direction decides *which* offset sets the lever:
  - g along -Y (correct): lever = HORIZONTAL offset  -> small, no distal growth
  - g along -Z (default): lever = VERTICAL offset    -> huge, grows distally
"""

import math
import xml.etree.ElementTree as ET

import numpy as np

OSIM = "/Users/soleilyu/claude_playground/labs/BioMotion/BioMotion/Resources/FullBody.osim"

JOINT_TAGS = ("CustomJoint", "PinJoint", "WeldJoint", "BallJoint",
              "FreeJoint", "SliderJoint", "UniversalJoint")


def rot(rx, ry, rz):
    """OpenSim body-fixed X-Y-Z euler."""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rx @ Ry @ Rz


def iso(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def vec(text):
    return np.array([float(v) for v in text.split()])


def parse():
    root = ET.parse(OSIM).getroot()

    bodies = {}
    for b in root.iter("Body"):
        m = b.find("mass")
        if m is None:
            continue
        c = b.find("mass_center")
        bodies[b.get("name")] = {
            "mass": float(m.text),
            "com": vec(c.text) if c is not None else np.zeros(3),
        }

    joints = {}
    for tag in JOINT_TAGS:
        for j in root.iter(tag):
            pf = j.find("socket_parent_frame")
            cf = j.find("socket_child_frame")
            if pf is None or cf is None:
                continue
            # Offset frames are declared INSIDE the joint; names repeat across
            # joints (pelvis_offset appears in ground_pelvis, hip_r, Abdjnt...),
            # so the lookup MUST be scoped to this joint's own subtree.
            local = {}
            for f in j.iter("PhysicalOffsetFrame"):
                par = f.find("socket_parent")
                if par is None:
                    continue
                t = f.find("translation")
                o = f.find("orientation")
                local.setdefault(f.get("name"), (
                    par.text.strip().split("/")[-1],
                    iso(rot(*(vec(o.text) if o is not None else np.zeros(3))),
                        vec(t.text) if t is not None else np.zeros(3)),
                ))
            pname = pf.text.strip().split("/")[-1]
            cname = cf.text.strip().split("/")[-1]
            if pname not in local or cname not in local:
                continue
            pb, Tp = local[pname]
            cb, Tc = local[cname]
            joints[j.get("name")] = {
                "type": tag, "parent_body": pb, "child_body": cb,
                "T_parent_off": Tp, "T_child_off": Tc,
            }
    return bodies, joints


def main():
    bodies, joints = parse()

    # forward kinematics at q = 0 (joint transform = identity)
    world = {"ground": np.eye(4)}
    changed = True
    while changed:
        changed = False
        for j in joints.values():
            pb, cb = j["parent_body"], j["child_body"]
            if pb in world and cb not in world:
                world[cb] = (world[pb] @ j["T_parent_off"]
                             @ np.linalg.inv(j["T_child_off"]))
                changed = True

    total_mass = sum(b["mass"] for b in bodies.values())
    com = np.zeros(3)
    placed = [n for n in bodies if n in world]
    for name in placed:
        b, T = bodies[name], world[name]
        com += b["mass"] * (T[:3, :3] @ b["com"] + T[:3, 3])
    com /= total_mass

    W = total_mass * 9.81
    print(f"bodies parsed        = {len(bodies)}   placed by FK = {len(placed)}")
    print(f"total mass           = {total_mass:.3f} kg    (test reports totalMass 79.58)")
    print(f"bodyweight           = {W:.1f} N          (test reports grf_N = 780)")
    print(f"whole-body CoM       = {np.round(com, 4)}")
    print()

    chain = [("hip_r", "femur_r"), ("walker_knee_r", "tibia_r"),
             ("ankle_r", "talus_r"), ("subtalar_r", "calcn_r"),
             ("mtp_r", "toes_r")]
    print("right-leg joint centres at the model's default (upright) pose:")
    jpos = {}
    for jname, child in chain:
        if child not in world:
            print(f"  {jname:16s} MISSING")
            continue
        jpos[jname] = world[child][:3, 3]
        print(f"  {jname:16s} {np.round(jpos[jname], 4)}")
    print()

    calcn, talus = world["calcn_r"][:3, 3], world["talus_r"][:3, 3]
    horiz = math.hypot(calcn[0] - talus[0], calcn[2] - talus[2])
    print("copFromWrench cross-check")
    print(f"  |calcn_r origin - talus_r origin| in the horizontal plane = {horiz:.4f} m")
    print("  NimbleBridge.mm:1127 returns the calcn origin VERBATIM when |force.y| < 1e-6,")
    print("  and RAJC (the marker the test differences against) is the talus_r origin,")
    print("  so a contact force with no world-Y component reports exactly this number.")
    print(f"  test reports cop_to_ankle_horiz = 0.0897 m")
    print("  (model is unscaled here; NimbleBridge calls setBodyScales from the")
    print("   subject's limb lengths, so the shipped value is this times the foot scale)")
    print()

    measured = {"subtalar_r": 672.0, "ankle_r": 472.0,
                "walker_knee_r": 313.0, "hip_r": 106.0}

    print("=" * 74)
    print("HYPOTHESIS B - gravity = (0,0,-9.81), DART's default (nobody calls setGravity)")
    print("  contact force is HORIZONTAL (+Z), lever = VERTICAL offset to the CoM")
    print(f"  {'joint':16s} {'y_joint':>9s} {'lever_m':>9s} {'predicted':>10s} {'measured':>9s}")
    for jname in ("subtalar_r", "ankle_r", "walker_knee_r", "hip_r"):
        lev = abs(com[1] - jpos[jname][1])
        print(f"  {jname:16s} {jpos[jname][1]:+9.4f} {lev:9.4f} "
              f"{W*lev:10.1f} {measured[jname]:9.1f}")
    print()
    print("HYPOTHESIS A - gravity = (0,-9.81,0), the correct one")
    print("  contact force is VERTICAL (+Y), lever = HORIZONTAL offset to the CoM")
    print(f"  {'joint':16s} {'lever_m':>9s} {'predicted':>10s} {'measured':>9s}")
    for jname in ("subtalar_r", "ankle_r", "walker_knee_r", "hip_r"):
        lev = math.hypot(com[0] - jpos[jname][0], com[2] - jpos[jname][2])
        print(f"  {jname:16s} {lev:9.4f} {W*lev:10.1f} {measured[jname]:9.1f}")
    print("=" * 74)
    print()
    print("The default pose is a symmetric upright stance, so hypothesis A's levers")
    print("collapse to ~0 and hypothesis B's collapse to the CoM height above each")
    print("joint.  The SHAPE is what discriminates:  B is monotone-decreasing from")
    print("distal to proximal and hits ~0 at the hip (the hip sits near CoM height);")
    print("A is uniformly small and cannot grow distally at all.")
    print()
    print("Implied levers if you invert the measured torques through W = 780.7 N:")
    for jname in ("subtalar_r", "ankle_r", "walker_knee_r", "hip_r"):
        print(f"  {jname:16s} {measured[jname]/W:.4f} m")
    print("  -> a monotone ladder of ~0.86 / 0.60 / 0.40 / 0.14 m.  Those are LIMB-")
    print("     LENGTH-scale VERTICAL distances up a leg, not horizontal CoP offsets.")


if __name__ == "__main__":
    main()
