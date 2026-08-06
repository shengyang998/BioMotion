#!/usr/bin/env python3
"""Apply the two structural fixes to FullBody.osim as minimal, line-scoped
text edits.  Deliberately NOT an XML round-trip: ElementTree would reformat all
51,492 lines and make the change unreviewable.

Every edit is scoped to the line range of the specific joint it belongs to, so
frame names that repeat elsewhere in the file (`femur_r_offset` also exists
inside `walker_knee_r`) cannot be hit by accident.  The script asserts the exact
expected number of replacements for every rule and aborts otherwise.

  python apply_fixes.py --in FullBody.osim.orig --out FullBody.osim \
                        [--beta-star-deg 0] [--fix patella,shoulder]

Revert: copy tools/osim_fixes/FullBody.osim.orig back over
        BioMotion/Resources/FullBody.osim  (sha256 in README.md).
"""

from __future__ import annotations

import argparse
import math
import re
import sys

import numpy as np

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from osim_kinematics import SimmSpline, euler_xyz, iso, rot_z  # noqa: E402

# The orthogonalised shoulder axis triple.  Ground truth = the in-repo file
# nimblephysics/data/osim/Return11/unscaled_generic_ortho.osim, whose "before"
# values are byte-identical to FullBody.osim's.  That file is orphaned data (0
# references anywhere in nimblephysics), so it is used as a NUMBER source only.
SHOULDER_AXES_BEFORE = [
    "-0.99826000000000004 0.0023 0.058897999999999999",
    "0.0047999999999999996 0.99909000000000003 0.0424",
    "0.0047999999999999996 0.0424 0.99909000000000003",
]
SHOULDER_AXES_AFTER = ["-1.0 0.0 0.0", "0.0 1.0 0.0", "0.0 0.0 1.0"]


def find_element_range(lines, open_pat, start=0):
    """Return (i, j) line indices of an element and its matching close tag."""
    tag = None
    i = None
    for k in range(start, len(lines)):
        m = re.search(open_pat, lines[k])
        if m:
            i = k
            tag = re.match(r"\s*<([A-Za-z0-9_]+)", lines[k]).group(1)
            break
    if i is None:
        raise SystemExit("could not find element matching %r" % open_pat)
    depth = 0
    for k in range(i, len(lines)):
        depth += len(re.findall(r"<%s[ >]" % tag, lines[k]))
        depth -= len(re.findall(r"</%s>" % tag, lines[k]))
        if depth == 0:
            return i, k
    raise SystemExit("unbalanced element %r" % tag)


def sub_in_range(lines, lo, hi, old, new, expect):
    n = 0
    for k in range(lo, hi + 1):
        if old in lines[k]:
            lines[k] = lines[k].replace(old, new)
            n += 1
    if n != expect:
        raise SystemExit("expected %d replacements of %r in [%d,%d], got %d"
                         % (expect, old, lo, hi, n))
    return n


def patella_weld_transform(lines, lo, hi, beta_star):
    """Evaluate the patellofemoral SpatialTransform at beta = beta_star and
    return (R, p) — the pose that must be baked into the parent offset frame so
    the WeldJoint puts the kneecap where the coupled joint would have."""
    block = "\n".join(lines[lo:hi + 1])
    import xml.etree.ElementTree as ET

    el = ET.fromstring(block)
    st = el.find("SpatialTransform")
    R = np.eye(3)
    p = np.zeros(3)
    from osim_kinematics import parse_function, rot_about

    for idx, ta in enumerate(st.findall("TransformAxis")):
        axis = np.array([float(v) for v in ta.find("axis").text.split()])
        fn_el = ta.find("function")
        fn, _kind = parse_function(fn_el if fn_el is not None else ta)
        val = fn(beta_star)
        if idx < 3:
            R = R @ rot_about(axis, val)
        else:
            p = p + val * axis
    return R, p


def mat_to_xyz_euler(R):
    """Inverse of math::eulerXYZToMatrix (= Rx Ry Rz)."""
    sy = R[0, 2]
    sy = max(-1.0, min(1.0, sy))
    ry = math.asin(sy)
    if abs(math.cos(ry)) > 1e-9:
        rx = math.atan2(-R[1, 2], R[2, 2])
        rz = math.atan2(-R[0, 1], R[0, 0])
    else:
        rx = math.atan2(R[2, 1], R[1, 1])
        rz = 0.0
    return np.array([rx, ry, rz])


def fix_patella(lines, side, beta_star_deg):
    body = "patella_%s" % side
    new_body = "kneecap_%s" % side
    jname = "patellofemoral_%s" % side
    new_jname = "kneecap_%s_jnt" % side

    lo, hi = find_element_range(lines, r'<CustomJoint name="%s">' % jname)
    R, p = patella_weld_transform(lines, lo, hi, math.radians(beta_star_deg))

    # 1. parent offset frame: bake T_joint(beta*) into it.
    plo, phi = find_element_range(
        lines, r'<PhysicalOffsetFrame name="femur_%s_offset">' % side, lo)
    assert plo >= lo and phi <= hi
    t_line = next(k for k in range(plo, phi + 1) if "<translation>" in lines[k])
    o_line = next(k for k in range(plo, phi + 1) if "<orientation>" in lines[k])
    t_old = np.array([float(v) for v in re.search(
        r"<translation>(.*?)</translation>", lines[t_line]).group(1).split()])
    o_old = np.array([float(v) for v in re.search(
        r"<orientation>(.*?)</orientation>", lines[o_line]).group(1).split()])
    T_old = iso(euler_xyz(o_old), t_old)
    T_new = T_old @ iso(R, p)
    t_new = T_new[:3, 3]
    o_new = mat_to_xyz_euler(T_new[:3, :3])
    indent_t = lines[t_line][: len(lines[t_line]) - len(lines[t_line].lstrip())]
    indent_o = lines[o_line][: len(lines[o_line]) - len(lines[o_line].lstrip())]
    fmt = lambda v: "%.17g" % (0.0 if v == 0 else v)  # kill "-0"  # noqa: E731
    lines[t_line] = "%s<translation>%s</translation>\n" % (
        indent_t, " ".join(fmt(v) for v in t_new))
    lines[o_line] = "%s<orientation>%s</orientation>\n" % (
        indent_o, " ".join(fmt(v) for v in o_new))

    # 2. rename the child offset frame + its body socket
    sub_in_range(lines, lo, hi, "%s_offset" % body, "%s_offset" % new_body, 2)
    sub_in_range(lines, lo, hi, "/bodyset/%s<" % body, "/bodyset/%s<" % new_body, 1)

    # 3. drop <coordinates> and <SpatialTransform> (a WeldJoint has neither).
    #    Deleted lines are blanked, not spliced out, so every line index taken
    #    above stays valid; main() drops the blanks at the end.
    for pat in (r"<coordinates>", r"<SpatialTransform>"):
        a, b = find_element_range(lines, pat, lo)
        assert lo < a and b < hi, (a, b, lo, hi)
        for k in range(a, b + 1):
            lines[k] = ""

    # 4. CustomJoint -> WeldJoint, and rename the joint
    lines[lo] = lines[lo].replace('<CustomJoint name="%s">' % jname,
                                  '<WeldJoint name="%s">' % new_jname)
    lines[hi] = lines[hi].replace("</CustomJoint>", "</WeldJoint>")

    # 5. the Body itself
    bl = [k for k, s in enumerate(lines) if s and '<Body name="%s">' % body in s]
    if len(bl) != 1:
        raise SystemExit("expected 1 <Body name=%r>, got %d" % (body, len(bl)))
    lines[bl[0]] = lines[bl[0]].replace('<Body name="%s">' % body,
                                        '<Body name="%s">' % new_body)

    # 6. every muscle path point that attaches to it
    n = 0
    for k, s in enumerate(lines):
        if s and "<socket_parent_frame>/bodyset/%s<" % body in s:
            lines[k] = s.replace("/bodyset/%s<" % body, "/bodyset/%s<" % new_body)
            n += 1
    if n != 9:
        raise SystemExit("expected 9 muscle path points on %s, got %d" % (body, n))

    return {"beta_star_deg": beta_star_deg,
            "baked_translation": t_new.tolist(),
            "baked_orientation": o_new.tolist(),
            "translation_before": t_old.tolist(),
            "orientation_before": o_old.tolist(),
            "path_points_repointed": n}


def drop_coupler_constraints(lines):
    """The two CoordinateCouplerConstraints drive knee_angle_{r,l}_beta, which
    no longer exists once the joint is welded.  nimble ignores constraints
    entirely (OpenSimParser.cpp:423-436 only logs a warning), but leaving a
    constraint that names a deleted coordinate would make the file invalid for
    any real OpenSim consumer."""
    out = 0
    for side in ("r", "l"):
        a, b = find_element_range(
            lines, r'<CoordinateCouplerConstraint name="patellofemoral_knee_angle_%s_con">' % side)
        for k in range(a, b + 1):
            lines[k] = ""
        out += 1
    return out


def fix_shoulder(lines, side_tag):
    lo, hi = find_element_range(lines, r'<CustomJoint name="shoulder_%s">' % side_tag)
    changed = []
    for before, after in zip(SHOULDER_AXES_BEFORE, SHOULDER_AXES_AFTER):
        n = sub_in_range(lines, lo, hi, "<axis>%s</axis>" % before,
                         "<axis>%s</axis>" % after, 1)
        changed.append((before, after, n))
    return changed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="src", required=True)
    ap.add_argument("--out", dest="dst", required=True)
    ap.add_argument("--beta-star-deg", type=float, default=0.0)
    ap.add_argument("--fix", default="patella,shoulder")
    args = ap.parse_args()

    fixes = set(args.fix.split(","))
    with open(args.src) as f:
        lines = f.readlines()
    n0 = len(lines)
    report = {}

    if "patella" in fixes:
        report["patella_r"] = fix_patella(lines, "r", args.beta_star_deg)
        report["patella_l"] = fix_patella(lines, "l", args.beta_star_deg)
        report["couplers_removed"] = drop_coupler_constraints(lines)
    if "shoulder" in fixes:
        report["shoulder_R"] = fix_shoulder(lines, "R")
        report["shoulder_L"] = fix_shoulder(lines, "L")

    lines = [s for s in lines if s != ""]
    with open(args.dst, "w") as f:
        f.writelines(lines)
    report["lines_before"] = n0
    report["lines_after"] = len(lines)
    import json
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
