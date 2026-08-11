"""Shared helpers for the OpenSim reference dumps.

Everything here is READ-ONLY with respect to `BioMotion/Resources/FullBody.osim`.
The one mutation performed is `WrapObject.set_active(False)` on an in-memory
copy, which builds the historical straight-line baseline used before path
wrapping shipped on 2026-08-08 -- see `wrap_off_model()`.
"""

from __future__ import annotations

import math
import os

import opensim as osim

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OSIM_PATH = os.path.join(REPO_ROOT, "BioMotion", "Resources", "FullBody.osim")

# `Model(filename)` chdirs into the model's directory, and the default logger
# then writes `opensim.log` THERE -- i.e. inside `BioMotion/Resources`, which is
# a folder reference in `project.yml` and therefore ships in the app bundle and
# the test bundle. Kill the file sink at import, before any model is loaded.
osim.Logger.removeFileSink()


def load_model(disable_wrapping: bool = False):
    """Load FullBody.osim. With `disable_wrapping`, every WrapObject in the
    model is deactivated BEFORE `initSystem()`, so each GeometryPath degenerates
    to the straight polyline through its path points -- exactly what the
    pre-2026-08-08 `MomentArmComputer.computeMuscleLengthForIndex:` baseline
    walked before applying any wrap solver."""
    model = osim.Model(OSIM_PATH)
    deactivated = 0
    if disable_wrapping:
        bodies = model.getBodySet()
        for i in range(bodies.getSize()):
            wrap_set = bodies.get(i).upd_WrapObjectSet()
            for j in range(wrap_set.getSize()):
                wrap_set.get(j).set_active(False)
                deactivated += 1
    state = model.initSystem()
    model.realizePosition(state)
    return model, state, deactivated


def coordinate_names(model):
    cs = model.getCoordinateSet()
    return [cs.get(i).getName() for i in range(cs.getSize())]


def muscle_names(model):
    ms = model.getMuscles()
    return [ms.get(i).getName() for i in range(ms.getSize())]


def joint_chain_to_ground(model):
    """body name -> list of joint names from ground down to that body."""
    joints = model.getJointSet()
    parent_of = {}   # child body name -> (joint name, parent body name)
    for i in range(joints.getSize()):
        j = joints.get(i)
        child = j.getChildFrame().findBaseFrame().getName()
        parent = j.getParentFrame().findBaseFrame().getName()
        parent_of[child] = (j.getName(), parent)

    chains = {}

    def chain(body):
        if body in chains:
            return chains[body]
        if body not in parent_of:
            chains[body] = []
            return chains[body]
        joint_name, parent = parent_of[body]
        result = chain(parent) + [joint_name]
        chains[body] = result
        return result

    bodies = model.getBodySet()
    for i in range(bodies.getSize()):
        chain(bodies.get(i).getName())
    chain("ground")
    return chains


def joint_coordinates(model):
    """joint name -> list of coordinate names."""
    joints = model.getJointSet()
    out = {}
    for i in range(joints.getSize()):
        j = joints.get(i)
        out[j.getName()] = [j.get_coordinates(k).getName()
                            for k in range(j.numCoordinates())]
    return out


def muscle_bodies(model, muscle):
    """Bodies the muscle's geometry touches: every path-point parent frame plus
    every referenced wrap object's body. The wrap bodies matter -- a muscle can
    wrap around a segment it has no attachment on, and that segment's joint
    still moves its path length."""
    path = muscle.getGeometryPath()
    bodies = set()
    pps = path.getPathPointSet()
    for i in range(pps.getSize()):
        bodies.add(pps.get(i).getBody().getName())
    wraps = path.getWrapSet()
    body_set = model.getBodySet()
    wrap_owner = {}
    for i in range(body_set.getSize()):
        b = body_set.get(i)
        wo = b.getWrapObjectSet()
        for j in range(wo.getSize()):
            wrap_owner[wo.get(j).getName()] = b.getName()
    for i in range(wraps.getSize()):
        name = wraps.get(i).getWrapObjectName()
        if name in wrap_owner:
            bodies.add(wrap_owner[name])
    return bodies


def spanned_coordinates(model):
    """muscle name -> list of coordinate names the muscle structurally spans.

    A coordinate spans a muscle when its joint lies strictly between the deepest
    common ancestor of the muscle's bodies and one of those bodies. Joints ABOVE
    the common ancestor (the pelvis free joint, for a leg muscle) move the whole
    path rigidly and cannot change its length, so they are excluded by
    construction rather than by a numerical threshold.

    LOCKED coordinates are excluded. `GeometryPath::computeMomentArm` returns
    exactly 0.0 for a locked coordinate -- that is a refusal, not a measurement,
    and FullBody.osim locks 54 of its 169 (both mtp_angle, both wrists, 50 rib
    coordinates). nimble does NOT honour `<locked>` (that is why the app carries
    a runtime DOF mask), so `MomentArmComputer` produces a real number there.
    Differencing a real number against a convention would manufacture a 100%
    error and attribute it to the missing wrap solver."""
    chains = joint_chain_to_ground(model)
    jcoords = joint_coordinates(model)
    cs = model.getCoordinateSet()
    locked = {cs.get(i).getName() for i in range(cs.getSize()) if cs.get(i).get_locked()}
    ms = model.getMuscles()
    out = {}
    for i in range(ms.getSize()):
        mu = ms.get(i)
        bodies = muscle_bodies(model, mu)
        body_chains = [chains.get(b, []) for b in sorted(bodies)]
        if not body_chains:
            out[mu.getName()] = []
            continue
        common = 0
        shortest = min(len(c) for c in body_chains)
        while common < shortest and all(c[common] == body_chains[0][common]
                                        for c in body_chains):
            common += 1
        spanning_joints = []
        seen = set()
        for c in body_chains:
            for jn in c[common:]:
                if jn not in seen:
                    seen.add(jn)
                    spanning_joints.append(jn)
        coords = []
        for jn in spanning_joints:
            coords.extend(c for c in jcoords.get(jn, []) if c not in locked)
        out[mu.getName()] = coords
    return out


def set_pose(model, state, pose_values):
    """pose_values: dict coordinate name -> value in the coordinate's own unit
    (radians for rotational, metres for translational). Coordinates absent from
    the dict keep their model default.

    LOCKED coordinates are skipped rather than written. 54 of the 169 are locked
    in FullBody.osim (both `mtp_angle`, both wrists, and the 50 rib coordinates);
    writing them would either raise or be silently overridden, and either way the
    value in the dump would not be the value the model used."""
    cs = model.getCoordinateSet()
    for i in range(cs.getSize()):
        c = cs.get(i)
        if c.get_locked():
            continue
        value = pose_values.get(c.getName(), c.getDefaultValue())
        c.setValue(state, value, False)
    model.realizePosition(state)


def actual_pose(model, state):
    """The coordinate values the model is ACTUALLY holding, read back after
    `set_pose`. This is what the fixture records -- never the requested dict."""
    cs = model.getCoordinateSet()
    return [cs.get(i).getValue(state) for i in range(cs.getSize())]


def wrap_point_count(path, state):
    """Number of PathWrapPoints in the path's CURRENT path. Zero means the
    muscle is taking a straight line between its path points at this pose, i.e.
    the wrap is not engaged and the shipped straight-line code is momentarily
    right."""
    current = path.getCurrentPath(state)
    n = 0
    for i in range(current.getSize()):
        if current.get(i).getConcreteClassName() == "PathWrapPoint":
            n += 1
    return n


def limits_degrees(model):
    """coordinate name -> (min, max) in DEGREES, from the model itself.

    The .osim stores rounded decimals -- `shoulder_elv_r` maxes at 2.0071 rad,
    i.e. 114.99836 deg, and `elbow_flex_r` at 2.618 rad = 149.99964 deg -- so a
    sweep written as "0 to 115" is genuinely outside the model. The limits are
    read from the model rather than restated in `poses.py` so there is one
    source of truth for them."""
    cs = model.getCoordinateSet()
    return {cs.get(i).getName(): (math.degrees(cs.get(i).getRangeMin()),
                                  math.degrees(cs.get(i).getRangeMax()))
            for i in range(cs.getSize())}


def deg(x):
    return math.radians(x)
