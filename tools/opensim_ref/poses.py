"""The pose set the OpenSim reference is dumped over.

# Why these poses and not joint angles read off the gait fixtures

`BioMotionTests/Fixtures/gait_*.txt` hold FIVE marker positions per frame --
raw MHR root, both ankles, both toes -- in the frame `MHRRetarget.makeBodyFrame`
produces. Fifteen scalars with no knee, no thigh and no pelvis orientation do
not determine hip/knee/ankle flexion: the source-root-to-ankle distance constrains
the SUM of hip and knee flexion and says nothing about the split. Reading joint
angles off those files would mean inventing the split and then calling it
measured, which is the exact failure this repo keeps paying for.

So the poses below are a COVERAGE GRID over the model's own coordinate ranges,
not a claim about any particular runner. `pose_coverage.py` states the only
claim this grid is entitled to make -- 100% of the clamped range of
hip_flexion / knee_angle / ankle_angle / elbow_flex / shoulder_elv, sampled at
5 deg or finer -- so no configuration the model can take is outside its hull in
those coordinates.

The clips were tried as a falsifiable check on that and the check does not
work, which is worth knowing: their source-root-to-ankle distance folds to 0.280 of
each clip's own maximum, and FullBody.osim cannot fold below 0.337 anywhere in
its clamped sagittal range (0.349 measuring from the hip-centre midpoint
instead; raw MHR joint 1 is itself 15.1 mm from the source HJC midpoint, so
neither root convention explains the gap). Without a
knee marker that gap cannot be turned into a joint angle, so it is reported by
`pose_coverage.py` and NOT used as a gate here.

Angles are degrees here and converted once, at the bottom.
"""

from __future__ import annotations

import math

# (hip_flexion, knee_angle, ankle_angle) in degrees, one entry per running
# phase. Six phases spanning stance and swing; the contralateral leg is put
# half a cycle away so every running pose is ASYMMETRIC, which is the regime
# the retired left/right claim lived in.
RUN_PHASES = [
    ("initial_contact", 30.0, 20.0, 0.0),
    ("midstance", 10.0, 40.0, 10.0),
    ("toe_off", -10.0, 15.0, -25.0),
    ("early_swing", 20.0, 110.0, -5.0),
    ("mid_swing", 55.0, 130.0, 0.0),
    ("late_swing", 45.0, 30.0, 5.0),
]


def _leg(side, hip, knee, ankle):
    return {
        "hip_flexion_%s" % side: hip,
        "knee_angle_%s" % side: knee,
        "ankle_angle_%s" % side: ankle,
    }


def _frange(start, stop, step):
    """`start` to `stop` inclusive, never PAST `stop`.

    The obvious `round((stop - start) / step)` overshoots when the span is not a
    whole number of steps: (0, 115, 10) rounds 11.5 up to 12 and emits 120. That
    put `shoulder_elv_r` 5 deg outside its clamped 0..115 range, and
    `Coordinate::setValue(state, v, False)` does NOT clamp -- it silently held
    120, so the dump would have carried a pose the model is not defined at.
    Caught by `pose_coverage.py` reporting 104.3% coverage."""
    n = int(math.floor((stop - start) / step + 1e-9))
    values = [start + i * step for i in range(n + 1)]
    if abs(values[-1] - stop) > 1e-9:
        values.append(stop)
    return values


#: A sweep endpoint written as a round number can land a hair outside the
#: model's own limit, because the .osim stores rounded decimals (`elbow_flex_r`
#: maxes at 2.618 rad = 149.99964 deg). Overshoots up to this are snapped onto
#: the limit; anything larger RAISES, because the two real bugs this caught were
#: 5 deg and 25 deg out.
SNAP_TOLERANCE_DEG = 1.0


def build_poses(limits=None):
    """-> list of (pose_id, {coordinate name: value in DEGREES or metres}).

    Rotational coordinates are given in degrees and converted by the caller via
    `to_radians`; translational ones are already metres and are listed in
    `TRANSLATIONAL`.

    `limits`: optional {name: (min_deg, max_deg)} from `osim_model.limits_degrees`.
    When given, every value is checked against it -- snapped if it overshoots by
    less than `SNAP_TOLERANCE_DEG`, raised on otherwise."""
    poses = []

    poses.append(("neutral", {}))

    poses.append(("squat_deep", dict(
        **_leg("r", 95.0, 110.0, 20.0), **_leg("l", 95.0, 110.0, 20.0))))

    # A trunk-flexed pose so the spine muscles are not all sampled at zero.
    # 4 deg on each of the fifteen intervertebral FE joints ~= 60 deg of trunk
    # flexion, distributed the way the model's own chain distributes it.
    spine = {}
    for level in ["Abs", "L5_S1", "L4_L5", "L3_L4", "L2_L3", "L1_L2", "T12_L1",
                  "T11_T12", "T10_T11", "T9_T10", "T8_T9", "T7_T8", "T6_T7",
                  "T5_T6", "T4_T5", "T3_T4", "T2_T3"]:
        spine["%s_FE" % level] = 4.0
    poses.append(("spine_flexed", spine))

    for index, (name, hip, knee, ankle) in enumerate(RUN_PHASES):
        other = RUN_PHASES[(index + 3) % len(RUN_PHASES)]
        values = {}
        values.update(_leg("r", hip, knee, ankle))
        values.update(_leg("l", other[1], other[2], other[3]))
        # Arms swing opposite the legs; keep it modest and explicit.
        # `shoulder_elv_l` runs -115..0 deg while `shoulder_elv_r` runs 0..115:
        # the LEFT shoulder's elevation coordinate is signed the other way in
        # FullBody.osim. Mirroring an arm pose by copying the value puts the
        # left shoulder outside its range, and
        # `Coordinate::setValue(state, v, False)` accepts it silently.
        values["shoulder_elv_r"] = 25.0
        values["elbow_flex_r"] = 70.0
        values["shoulder_elv_l"] = -25.0
        values["elbow_flex_l"] = 70.0
        poses.append(("run_%d_%s" % (index, name), values))

    # One-dimensional sweeps. These are where a wrap solver engages and
    # disengages, so they are the poses that expose a dL/dq discontinuity.
    for knee in _frange(0.0, 140.0, 5.0):
        poses.append(("knee_sweep_%03d" % round(knee),
                      _leg("r", 20.0, knee, 0.0)))
    for hip in _frange(-20.0, 120.0, 5.0):
        poses.append(("hip_sweep_%+04d" % round(hip),
                      _leg("r", hip, 30.0, 0.0)))
    for ankle in _frange(-40.0, 30.0, 2.5):
        poses.append(("ankle_sweep_%+06.1f" % ankle,
                      _leg("r", 10.0, 20.0, ankle)))
    for elbow in _frange(0.0, 150.0, 10.0):
        poses.append(("elbow_sweep_%03d" % round(elbow),
                      {"shoulder_elv_r": 30.0, "elbow_flex_r": elbow}))
    for elv in _frange(0.0, 115.0, 10.0):
        poses.append(("shoulder_sweep_%03d" % round(elv),
                      {"shoulder_elv_r": elv, "elbow_flex_r": 30.0}))

    # Coarse three-dimensional lower-limb grid, so the sweeps are not the only
    # evidence and combinations of flexion are covered too.
    for hip in [0.0, 30.0, 60.0, 90.0]:
        for knee in [0.0, 40.0, 80.0, 120.0]:
            for ankle in [-20.0, 0.0, 20.0]:
                poses.append(("grid_h%03d_k%03d_a%+03d"
                              % (round(hip), round(knee), round(ankle)),
                              _leg("r", hip, knee, ankle)))

    ids = [p[0] for p in poses]
    assert len(ids) == len(set(ids)), "duplicate pose id"
    if limits is not None:
        poses = [(pose_id, _snap_to_limits(pose_id, values, limits))
                 for pose_id, values in poses]
    return poses


def _snap_to_limits(pose_id, values, limits):
    out = {}
    for name, value in values.items():
        if name not in limits or name in TRANSLATIONAL:
            out[name] = value
            continue
        low, high = limits[name]
        if value < low:
            overshoot = low - value
            target = low
        elif value > high:
            overshoot = value - high
            target = high
        else:
            out[name] = value
            continue
        if overshoot > SNAP_TOLERANCE_DEG:
            raise ValueError(
                "pose %s: %s = %.4f deg is %.4f deg outside its range "
                "[%.4f, %.4f] -- that is a mistake, not a rounding artefact"
                % (pose_id, name, value, overshoot, low, high))
        out[name] = target
    return out


# Coordinates whose value is metres, not degrees. Nothing in `build_poses`
# touches one today; the list exists so that adding a pelvis translation later
# cannot silently be read as degrees.
TRANSLATIONAL = {"pelvis_tx", "pelvis_ty", "pelvis_tz"}


def to_radians(values):
    return {name: (value if name in TRANSLATIONAL else math.radians(value))
            for name, value in values.items()}
