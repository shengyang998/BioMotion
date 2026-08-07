# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=2"]
# ///
"""Write the pinned gait clip fixtures from the cached MHR trajectories.

    cd labs/BioMotion && uv run tools/gait_fixture/export_gait_fixture.py

Deterministic: the same npz in gives a byte-identical file out.

WHAT IT WRITES
==============
`BioMotionTests/Fixtures/gait_<clip>.txt`, one per clip — a line-oriented plain
ASCII file holding exactly the five joints a gait analysis needs (pelvis, both
ankles, both toes), in exactly the frame `MHRRetarget.makeBodyFrame(camT: nil)`
produces: metres, Y-up / X-right / Z-toward-camera, pelvis pinned at the model
constant 0.92398697, no smoothing, no floor alignment, no camera compensation.

For these five markers `MHRRetarget.table` is a straight index lookup
(`blendT == 0`, zero `localOffset`), so `jc[i]` IS the marker:

    hips_joint 1 · left_foot_joint 4 · right_foot_joint 20
               · left_toes_joint 8 · right_toes_joint 24

Only 5 of the 127 MHR joints are emitted. The other 122 are not needed to read
contact and flight timing off the clip, and a 127-joint fixture would be 25x
the size for no gate it could support.

WHY PLAIN DECIMAL ASCII, ASSERTED HERE
======================================
The previous fixture generator wrote `f"{ts[i]!r}"`. Under numpy 2 the repr of a
float64 scalar is `np.float64(3.0)`, so the fixture's FIRST data line was
`np.float64(3.0),0.0,...`. The Swift side parsed it with `Double(f[0])!`, and a
force-unwrap of nil does not fail a test — it traps the whole xctest process and
takes every other test in the target down with it. Nothing downstream of that
fixture was ever runnable.

So every emitted data line is validated HERE against a strict grammar
(`DATA_LINE_RE`) before anything is written: integers are `[0-9]+`, floats are
`-?[0-9]+.[0-9]+`, fields are single-space separated, and the whole file must
encode as ASCII. That grammar admits no exponent, no `nan`, no `inf`, no hex
float, no sign-prefixed `+`, and no parenthesis — so `np.float64(3.0)` cannot
survive it, and neither can the shapes Swift's `Double(_: String)` accepts but a
metre coordinate should never be (`Double("nan")` and `Double("0x1p3")` both
succeed). Every emitted token is then re-parsed with Python's own `float()` and
compared for EXACT equality with the source value, so the file is proven to
round-trip before it reaches disk.

WHAT ELSE THIS REFUSES TO WRITE
===============================
* Any two ADJACENT frames identical. `vision_box_probe.swift` wrote PNG names as
  `t%05.1f.png` — a filename quantised to 0.1 s — so at a 1/30 s step three
  consecutive records addressed ONE image: 41 distinct PNGs for 120 requested
  frames. Everything downstream read a 10 Hz staircase wearing a 30 fps label,
  and that staircase, not the runner, produced the "contact is exactly 6 frames
  on 13 of 13 contacts, unchanged across a 2.5x threshold span" result the whole
  gait route was resting on. Measured on the corrected caches: 0 duplicate
  adjacent pairs on all three clips, minimum adjacent displacement 0.10 m.
* Timestamps not strictly increasing, or a base sampling interval more than 1%
  off 1/30 s, or a gap that is not an integer multiple of that interval.
* Coordinates that are not exactly representable in Float32 (they all are — the
  cache came from an fp16 Core ML model widened to float64), which is what makes
  the pinned numbers reproduce bit-for-bit rather than approximately.

TIMESTAMPS ARE COPIED, NEVER REGENERATED
========================================
`t` is the decoder's presentation timestamp, verbatim; it is NOT `i * dt`.

`video_013` lost 3 frames to `VNDetectHumanRectanglesRequest` returning nothing
(119 records across a span that holds 122 frame slots), leaving one 2-frame and
one 3-frame hole. Writing `i * dt` would paper over both and silently move that
clip's measured stride from 613 ms to 593 ms. The holes are left in, and the
`frame` column carries the decoder slot each record fell in (verified against
the timestamp, not assumed), so a consumer that needs the video clock sees the
skipped numbers instead of counting records.

INPUT
=====
`/tmp/cache2_{012,013,015}.npz` <- `labs/sam-3d-body/export/gait_cache.py` over
frames from `frame_probe.swift`, a SEQUENTIAL `AVAssetReader` walk in
presentation order. Keys used: `ts` (N,), `jc` (N,127,3).
"""

import pathlib
import re
import sys

import numpy as np

# MHR joint_coords index -> ARKit joint id, in the fixture's column order.
# Cross-checked against `MHRRetarget.table` (BioMotion/Offline/MHRRetarget.swift)
# and `labs/sam-3d-body/export/gait_lib.MHR`. Load-bearing: the column order is
# the order the Swift loader assumes, and `GaitClipFixtureTests` re-checks these
# ids against `MHRRetarget.table` so a retarget change cannot silently desync.
COLUMNS = [
    ("hips_joint", 1),
    ("left_foot_joint", 4),
    ("right_foot_joint", 20),
    ("left_toes_joint", 8),
    ("right_toes_joint", 24),
]

CLIPS = [("video_012", "012"), ("video_013", "013"), ("video_015", "015")]

FORMAT_ID = "biomotion-gait-clip-v1"
NOMINAL_DT = 1.0 / 30.0

# A frame slot must be recoverable from the timestamp to well inside half a
# frame, or the "slot" label is a guess rather than a reading.
SLOT_TOLERANCE_FRAMES = 0.2

REPO = pathlib.Path(__file__).resolve().parents[2]
OUT_DIR = REPO / "BioMotionTests" / "Fixtures"

INT_RE = re.compile(r"^(?:0|[1-9][0-9]*)$")
DEC_RE = re.compile(r"^-?(?:0|[1-9][0-9]*)\.[0-9]+$")
# frame, t, then 3 coordinates per joint.
DATA_LINE_RE = re.compile(
    r"^(?:0|[1-9][0-9]*)(?: -?(?:0|[1-9][0-9]*)\.[0-9]+){%d}$" % (1 + 3 * len(COLUMNS))
)


class Refused(SystemExit):
    """Raised instead of writing a fixture that would encode a known defect."""


def dec64(v: float) -> str:
    """Shortest plain-decimal string that round-trips through float64."""
    return np.format_float_positional(np.float64(v), unique=True, trim="0")


def dec32(v: float) -> str:
    """Shortest plain-decimal string that round-trips through Float32.

    Swift parses these into `Float` bit-for-bit, so a pinned test number is
    reproducible rather than approximately reproducible.
    """
    return np.format_float_positional(np.float32(v), unique=True, trim="0")


def check_token(tok: str, pattern: re.Pattern, original, what: str, narrow=float) -> str:
    """Grammar-check one token AND prove it re-parses to exactly `original`.

    `narrow` is the width the CONSUMER parses at, because that is where equality
    has to hold: Swift reads `t` into a `Double` and coordinates into a `Float`,
    so a coordinate token only has to agree with the source after both are
    narrowed to Float32 (it does — the coordinates are exactly Float32, checked
    once per clip in `build`).
    """
    if not pattern.match(tok):
        raise Refused(f"{what}: {tok!r} is not plain decimal ASCII")
    if narrow(float(tok)) != narrow(original):
        raise Refused(f"{what}: {tok!r} does not round-trip to {original!r}")
    return tok


def build(clip_id: str, tag: str) -> tuple[str, dict]:
    d = np.load(f"/tmp/cache2_{tag}.npz", allow_pickle=True)
    ts = np.asarray(d["ts"], dtype=np.float64)
    jc = np.asarray(d["jc"], dtype=np.float64)
    n = len(ts)
    if n < 2:
        raise Refused(f"{clip_id}: {n} frames is not a clip")

    block = np.concatenate([jc[:, i, :] for _, i in COLUMNS], axis=1)

    # --- refusals, each pinned to a defect that actually shipped once ---------
    if not np.array_equal(block.astype(np.float32).astype(np.float64), block):
        raise Refused(f"{clip_id}: coordinates are not exactly Float32; the pinned "
                      "numbers would drift when Swift narrows them")

    diffs = np.diff(ts)
    if not np.all(diffs > 0):
        raise Refused(f"{clip_id}: timestamps are not strictly increasing")

    dt = float(np.median(diffs))
    if abs(dt - NOMINAL_DT) > NOMINAL_DT / 100.0:
        raise Refused(f"{clip_id}: base sampling interval {dt:.6f}s is more than 1% "
                      f"off {NOMINAL_DT:.6f}s")

    slots_f = (ts - ts[0]) / dt
    slot_err = float(np.abs(slots_f - np.round(slots_f)).max())
    if slot_err > SLOT_TOLERANCE_FRAMES:
        raise Refused(f"{clip_id}: a timestamp sits {slot_err:.3f} frames off the "
                      "sampling grid; its decoder slot is not readable")
    slots = np.round(slots_f).astype(np.int64)
    if not np.all(np.diff(slots) >= 1):
        raise Refused(f"{clip_id}: frame slots are not strictly increasing")

    dup = np.all(np.diff(block, axis=0) == 0.0, axis=1)
    if dup.any():
        raise Refused(
            f"{clip_id}: {int(dup.sum())} adjacent frame pairs are identical. This is "
            "the extraction defect that produced STATUS.md's original gait table "
            "(41 distinct PNGs for 120 requested frames). Fix the extractor; do not "
            "relax this check.")

    # --- emit -----------------------------------------------------------------
    joint_ids = " ".join(name for name, _ in COLUMNS)
    lines = [
        "# GENERATED by tools/gait_fixture/export_gait_fixture.py - do not hand-edit.",
        f"# Source /tmp/cache2_{tag}.npz <- labs/sam-3d-body/export/gait_cache.py.",
        "# Frame of reference: MHRRetarget.makeBodyFrame(camT: nil) - metres, Y-up,",
        "# pelvis pinned at the model constant 0.92398697.",
        "# Columns: frame t then x y z per joint, in the `joints` order below.",
        "# `frame` is the decoder slot (video_013 skips 3 - Vision found no person",
        "# there); `t` is the decoder's presentation timestamp, verbatim.",
        f"format {FORMAT_ID}",
        f"clip {clip_id}",
        f"frames {n}",
        f"joints {joint_ids}",
    ]

    for i in range(n):
        fields = [check_token(str(int(slots[i])), INT_RE, int(slots[i]),
                              f"{clip_id} row {i} frame", narrow=int)]
        fields.append(check_token(dec64(ts[i]), DEC_RE, ts[i], f"{clip_id} row {i} t"))
        for k, v in enumerate(block[i]):
            fields.append(check_token(dec32(v), DEC_RE, v, f"{clip_id} row {i} col {k}",
                                      narrow=np.float32))
        line = " ".join(fields)
        if not DATA_LINE_RE.match(line):
            raise Refused(f"{clip_id} row {i}: emitted line is not plain decimal ASCII:\n{line}")
        lines.append(line)

    text = "\n".join(lines) + "\n"
    try:
        text.encode("ascii")
    except UnicodeEncodeError as exc:
        raise Refused(f"{clip_id}: fixture is not ASCII ({exc})") from exc

    gaps = np.round(np.diff(slots)).astype(int)
    stats = {
        "frames": n,
        "dt_ms": dt * 1000.0,
        "span_s": float(ts[-1] - ts[0]),
        "holes": int((gaps > 1).sum()),
        "min_adjacent_move_m": float(np.abs(np.diff(block, axis=0)).max(axis=1).min()),
        "slot_err_frames": slot_err,
        "bytes": len(text),
    }
    return text, stats


def main() -> int:
    print("export_gait_fixture:")
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    written = []
    # Build every clip BEFORE writing any, so a refusal on the third clip cannot
    # leave two fresh fixtures beside one stale one.
    built = [(clip_id, *build(clip_id, tag)) for clip_id, tag in CLIPS]
    for clip_id, text, stats in built:
        path = OUT_DIR / f"gait_{clip_id}.txt"
        path.write_text(text, encoding="ascii")
        written.append(path)
        print(f"  {clip_id}: {stats['frames']} frames over {stats['span_s']:.3f}s, "
              f"dt {stats['dt_ms']:.4f} ms, {stats['holes']} hole(s), "
              f"min adjacent move {stats['min_adjacent_move_m']:.3f} m, "
              f"slot error {stats['slot_err_frames']:.2e} frames, "
              f"{stats['bytes']} bytes -> {path.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
