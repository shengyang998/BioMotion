"""Read the dump, measure the straight-line defect, and write the Swift fixture.

  python analyse.py                 report only
  python analyse.py --write-fixture also write BioMotionTests/Fixtures/...

The report answers the question that motivates the whole wrap-solver work:
for each muscle, how far is the straight-line moment arm the shipped
`MomentArmComputer` produces from OpenSim's own?
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys

import osim_model as M

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURE_PATH = os.path.join(M.REPO_ROOT, "BioMotionTests", "Fixtures",
                            "opensim_moment_arms.txt")
FORMAT_ID = "biomotion-osim-momentarm-v1"

# `GaitLoadSummary.displayNames` keys, verbatim. These are the muscles the
# product names on screen; kept in sync by `OpenSimReferenceFixtureTests`.
DISPLAY_BASES = [
    "glmax1", "glmax2", "glmax3", "glmed1", "glmed2", "glmed3",
    "glmin1", "glmin2", "glmin3", "recfem", "vasmed", "vaslat", "vaslat140",
    "vasint", "bflh", "bflh140", "bfsh", "semimem", "semiten",
    "gasmed", "gaslat", "gaslat140", "soleus", "tibant", "tibpost",
    "perlong", "perbrev", "psoas", "iliacus", "addlong", "addbrev",
    "addmagDist", "addmagIsch", "addmagMid", "addmagProx",
    "tfl", "sart", "grac", "edl", "ehl", "fdl", "fhl", "piri",
]


def base_name(muscle):
    if muscle.endswith("_r") or muscle.endswith("_l"):
        return muscle[:-2]
    return muscle


def read_csv(path):
    with open(path) as handle:
        return list(csv.DictReader(handle))


def percentiles(values, points=(50, 90, 95, 99, 100)):
    if not values:
        return {p: float("nan") for p in points}
    ordered = sorted(values)
    out = {}
    for p in points:
        if p >= 100:
            out[p] = ordered[-1]
            continue
        k = (len(ordered) - 1) * p / 100.0
        low = int(math.floor(k))
        high = min(low + 1, len(ordered) - 1)
        out[p] = ordered[low] + (ordered[high] - ordered[low]) * (k - low)
    return out


def report(arms, lengths, out=sys.stdout):
    wrapped = set()
    model, _, _ = M.load_model()
    muscles = model.getMuscles()
    for i in range(muscles.getSize()):
        mu = muscles.get(i)
        if mu.getGeometryPath().getWrapSet().getSize() > 0:
            wrapped.add(mu.getName())

    display_muscles = {r["muscle"] for r in arms
                       if base_name(r["muscle"]) in DISPLAY_BASES}

    def emit(title, rows):
        abs_err = [abs(float(r["r_wrap_on"]) - float(r["r_wrap_off"])) for r in rows]
        # Relative to the reference's own magnitude: a 1 cm error on a 1 cm
        # moment arm is a different fact from 1 cm on 5 cm. Pairs whose
        # reference moment arm is below 1 mm are reported separately rather
        # than divided by, because the ratio there is dominated by the
        # denominator.
        rel = []
        tiny = 0
        for r in rows:
            ref = abs(float(r["r_wrap_on"]))
            if ref < 1e-3:
                tiny += 1
                continue
            rel.append(abs(float(r["r_wrap_on"]) - float(r["r_wrap_off"])) / ref)
        sign_flips = sum(1 for r in rows
                         if float(r["r_wrap_on"]) * float(r["r_wrap_off"]) < 0
                         and abs(float(r["r_wrap_on"])) >= 1e-3)
        pa = percentiles(abs_err)
        pr = percentiles(rel)
        out.write("\n%s\n" % title)
        out.write("  pairs                 %d   (%d with |reference| < 1 mm, excluded from the ratio)\n"
                  % (len(rows), tiny))
        out.write("  |error| mm   median %7.3f  p90 %7.3f  p95 %7.3f  p99 %7.3f  max %8.3f\n"
                  % (pa[50] * 1e3, pa[90] * 1e3, pa[95] * 1e3, pa[99] * 1e3, pa[100] * 1e3))
        out.write("  |error|/|ref| median %6.1f%%  p90 %8.1f%%  p95 %8.1f%%  p99 %9.1f%%  max %10.1f%%\n"
                  % (pr[50] * 100, pr[90] * 100, pr[95] * 100, pr[99] * 100, pr[100] * 100))
        over = lambda t: sum(1 for v in rel if v > t)
        out.write("  share of pairs wrong by >10%%: %5.1f%%   >25%%: %5.1f%%   >50%%: %5.1f%%   >100%%: %5.1f%%\n"
                  % tuple(100.0 * over(t) / len(rel) if rel else float("nan")
                          for t in (0.10, 0.25, 0.50, 1.00)))
        out.write("  moment arms with the WRONG SIGN (|ref| >= 1 mm): %d  (%.2f%% of pairs)\n"
                  % (sign_flips, 100.0 * sign_flips / len(rows) if rows else float("nan")))

    emit("ALL muscles, all poses", arms)
    emit("Muscles the product NAMES (GaitLoadSummary.displayNames), all poses",
         [r for r in arms if r["muscle"] in display_muscles])
    emit("Muscles that carry a PathWrap (%d of 520)" % len(wrapped),
         [r for r in arms if r["muscle"] in wrapped])
    emit("Muscles with NO PathWrap -- the straight line is the right model here",
         [r for r in arms if r["muscle"] not in wrapped])

    running = [r for r in arms if r["pose"].startswith("run_")]
    emit("Running poses only, muscles the product names",
         [r for r in running if r["muscle"] in display_muscles])
    emit("Neutral pose only, muscles the product names",
         [r for r in arms if r["pose"] == "neutral" and r["muscle"] in display_muscles])

    # Worst named muscles, ranked by their own median relative error.
    out.write("\nWorst NAMED muscle/coordinate pairs by median relative error\n")
    out.write("  (median over all %d poses; only pairs whose reference is >= 1 mm)\n"
              % len({r["pose"] for r in arms}))
    per_pair = {}
    for r in arms:
        if r["muscle"] not in display_muscles:
            continue
        ref = abs(float(r["r_wrap_on"]))
        if ref < 1e-3:
            continue
        key = (r["muscle"], r["coordinate"])
        per_pair.setdefault(key, []).append(
            abs(float(r["r_wrap_on"]) - float(r["r_wrap_off"])) / ref)
    ranked = sorted(per_pair.items(), key=lambda kv: -statistics.median(kv[1]))
    out.write("  %-18s %-18s %8s %8s %7s\n"
              % ("muscle", "coordinate", "median%", "max%", "n"))
    for (muscle, coordinate), values in ranked[:25]:
        out.write("  %-18s %-18s %8.1f %8.1f %7d\n"
                  % (muscle, coordinate, 100 * statistics.median(values),
                     100 * max(values), len(values)))

    # Path length, which is what the wrap solver has to get right first.
    ldiff = [abs(float(r["length_wrap_on"]) - float(r["length_wrap_off"]))
             for r in lengths]
    lrel = [abs(float(r["length_wrap_on"]) - float(r["length_wrap_off"]))
            / float(r["length_wrap_on"])
            for r in lengths if float(r["length_wrap_on"]) > 1e-6]
    pl = percentiles(ldiff)
    prl = percentiles(lrel)
    out.write("\nPATH LENGTH, all muscles x all poses (%d rows)\n" % len(lengths))
    out.write("  |error| mm   median %7.3f  p90 %7.3f  p99 %7.3f  max %8.3f\n"
              % (pl[50] * 1e3, pl[90] * 1e3, pl[99] * 1e3, pl[100] * 1e3))
    out.write("  |error|/L    median %6.2f%%  p90 %6.2f%%  p99 %6.2f%%  max %7.2f%%\n"
              % (prl[50] * 100, prl[90] * 100, prl[99] * 100, prl[100] * 100))

    engaged = sum(1 for r in lengths if int(r["wrap_points"]) > 0)
    out.write("  wrap actually engaged on %d of %d (muscle, pose) rows = %.1f%%\n"
              % (engaged, len(lengths), 100.0 * engaged / len(lengths)))

    # The discontinuity risk named up front: where wrapping switches on or off
    # between two adjacent sweep poses, a centred difference straddling the
    # switch differentiates a kinked length function.
    out.write("\nWRAP ON/OFF TRANSITIONS along the 1-D sweeps\n")
    by_muscle = {}
    for r in lengths:
        by_muscle.setdefault(r["muscle"], []).append((r["pose"], int(r["wrap_points"])))
    all_movers = set()
    all_transitions = 0
    for prefix in ("knee_sweep_", "hip_sweep_", "ankle_sweep_",
                   "elbow_sweep_", "shoulder_sweep_"):
        transitions = 0
        movers = set()
        for muscle, rows in by_muscle.items():
            sweep = [(p, w) for p, w in rows if p.startswith(prefix)]
            for a, b in zip(sweep, sweep[1:]):
                if (a[1] == 0) != (b[1] == 0):
                    transitions += 1
                    movers.add(muscle)
        all_movers |= movers
        all_transitions += transitions
        out.write("  %-16s %3d transitions across %d muscles\n"
                  % (prefix, transitions, len(movers)))
    # The per-sweep muscle counts do NOT sum to the distinct total: a muscle
    # that switches in two different sweeps appears in both.
    out.write("  %-16s %3d transitions on %d DISTINCT muscles\n"
              % ("TOTAL", all_transitions, len(all_movers)))


def write_fixture(arms, lengths, poses, coord_names, opensim_version):
    wrapped = set()
    model, _, _ = M.load_model()
    muscles = model.getMuscles()
    for i in range(muscles.getSize()):
        mu = muscles.get(i)
        if mu.getGeometryPath().getWrapSet().getSize() > 0:
            wrapped.add(mu.getName())

    keep = sorted({r["muscle"] for r in arms
                   if r["muscle"] in wrapped or base_name(r["muscle"]) in DISPLAY_BASES})
    keep_index = {name: i for i, name in enumerate(keep)}

    pose_ids = [r["pose"] for r in poses]
    pose_index = {p: i for i, p in enumerate(pose_ids)}

    coords_of = {}
    for r in arms:
        if r["muscle"] in keep_index:
            coords_of.setdefault(r["muscle"], [])
            if r["coordinate"] not in coords_of[r["muscle"]]:
                coords_of[r["muscle"]].append(r["coordinate"])

    arm_lookup = {}
    for r in arms:
        if r["muscle"] in keep_index:
            arm_lookup[(r["pose"], r["muscle"], r["coordinate"])] = \
                (float(r["r_wrap_on"]), float(r["r_wrap_off"]))
    len_lookup = {(r["pose"], r["muscle"]): r for r in lengths
                  if r["muscle"] in keep_index}

    with open(FIXTURE_PATH, "w") as f:
        f.write("# GENERATED by tools/opensim_ref/analyse.py --write-fixture - do not hand-edit.\n")
        f.write("# OpenSim %s, reading BioMotion/Resources/FullBody.osim.\n" % opensim_version)
        f.write("# `on` columns are OpenSim's own values with all 76 PathWraps solved:\n")
        f.write("# the REFERENCE a wrap solver has to reproduce. `off` columns are the\n")
        f.write("# same model with every WrapObject deactivated - the straight-line\n")
        f.write("# shortcut MomentArmComputer takes today.\n")
        f.write("# Moment arms and lengths are metres, coordinate values radians\n")
        f.write("# (metres for pelvis_tx/ty/tz). Every number is a plain decimal:\n")
        f.write("# no exponent, no nan, no inf - see OpenSimReferenceFixture.swift.\n")
        f.write("format %s\n" % FORMAT_ID)
        f.write("coordinates %s\n" % " ".join(coord_names))
        f.write("poses %d\n" % len(pose_ids))
        f.write("muscles %d\n" % len(keep))
        for r in poses:
            f.write("pose %s %s\n" % (r["pose"],
                                      " ".join("%.9f" % float(r[c]) for c in coord_names)))
        for name in keep:
            cs = coords_of.get(name, [])
            # The wrap flag is DECLARED, not inferred. A muscle whose wrap
            # happens never to engage at any sampled pose is still a muscle
            # whose path is unmodelled, and inferring "wrapped" from
            # `lengthWrapOn != lengthWrapOff` would silently drop it.
            f.write("muscle %s %d %d %s\n"
                    % (name, 1 if name in wrapped else 0, len(cs), " ".join(cs)))
        rows = 0
        for pose_id in pose_ids:
            for name in keep:
                lrow = len_lookup.get((pose_id, name))
                if lrow is None:
                    raise SystemExit("no length row for %s / %s" % (pose_id, name))
                cs = coords_of.get(name, [])
                on = []
                off = []
                for coordinate in cs:
                    a, b = arm_lookup[(pose_id, name, coordinate)]
                    on.append(a)
                    off.append(b)
                f.write("row %d %d %s %.9f %.9f %s %s\n"
                        % (pose_index[pose_id], keep_index[name], lrow["wrap_points"],
                           float(lrow["length_wrap_on"]), float(lrow["length_wrap_off"]),
                           " ".join("%.9f" % v for v in on),
                           " ".join("%.9f" % v for v in off)))
                rows += 1
    size = os.path.getsize(FIXTURE_PATH)
    sys.stderr.write("wrote %s: %d poses x %d muscles = %d rows, %.1f MB\n"
                     % (FIXTURE_PATH, len(pose_ids), len(keep), rows, size / 1e6))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=os.path.join(HERE, "out"))
    parser.add_argument("--write-fixture", action="store_true")
    args = parser.parse_args()

    arms = read_csv(os.path.join(args.out, "moment_arms.csv"))
    lengths = read_csv(os.path.join(args.out, "lengths.csv"))
    with open(os.path.join(args.out, "poses.csv")) as handle:
        poses = list(csv.DictReader(handle))
    coord_names = [c for c in poses[0].keys() if c != "pose"]

    report(arms, lengths)
    if args.write_fixture:
        import opensim
        write_fixture(arms, lengths, poses, coord_names, opensim.GetVersion())


if __name__ == "__main__":
    main()
