# osim_fixes — patella weld + shoulder orthogonalisation

Fixes the two structural defects that made BioMotion's muscle output unusable for
its two headline postures, and measures the fix. Everything here is re-runnable.

```
tools/osim_fixes/
  FullBody.osim.orig    pristine pre-change model  (sha256 a55c6434...a83913)
  osim_kinematics.py    OpenSim FK + moment-arm engine (see "How it was measured")
  apply_fixes.py        the actual edit, as a scripted, asserted transformation
  measure.py            writes measurements.json
  regression.py         "did anything else move?" — exits non-zero on a surprise
  measurements.json     every before/after number
```

Reproduce end to end (patched model sha256 `00034739...c8d741c09`):

```bash
PY=/Users/soleilyu/claude_playground/labs/sam-3d-body/.venv/bin/python
cd /Users/soleilyu/claude_playground/labs/BioMotion/tools/osim_fixes
$PY apply_fixes.py --in FullBody.osim.orig \
    --out ../../BioMotion/Resources/FullBody.osim --beta-star-deg 0
$PY measure.py       # -> measurements.json
$PY regression.py    # -> PASS
```

`apply_fixes.py` asserts the exact expected number of replacements for every rule
(9 muscle path points per side, 1 body, 1 joint, 2 frame renames, 3 axis lines per
shoulder) and aborts rather than write a partially-applied model.

---

## Files changed

| File | `git diff --numstat` | Change |
|---|---|---|
| `BioMotion/Resources/FullBody.osim` | `40 / 242` | 40 lines rewritten in place, 202 further lines removed (the patellofemoral `<coordinates>` + `<SpatialTransform>` blocks and the two `CoordinateCouplerConstraint`s), 0 other lines touched |
| `BioMotion/Nimble/NimbleBridge.mm` | `13 / 1` | `groupScale()` only — one new `find("kneecap")` branch + comment |

Nothing else was edited. `git diff --stat` on the submodule shows exactly these two files.

**Revert:** `cp tools/osim_fixes/FullBody.osim.orig BioMotion/Resources/FullBody.osim`
and drop the `kneecap` line from `groupScale()`. The `patella` line in `groupScale()`
was deliberately kept, so reverting the model alone leaves the bridge correct.

---

## Defect 1 — quadriceps read as unloaded during a squat

### What was actually wrong (this corrects STATUS.md)

STATUS.md says the ~8 quadriceps "have zero knee moment arm". That is true at one
knee angle and misleading everywhere else, and the difference matters.
`MomentArmComputer.mm:worldPositionForPathPoint` falls back to the **raw
body-local offset** when a path point's body is missing from the skeleton, so the
9 patella path points per leg became points **pinned in world space near the
origin**. The resulting curve is not zero, it is garbage that happens to pass
through zero near 30° flexion:

| knee flexion | before | after | reference (coupled patella) |
|---|---|---|---|
| 0° | **+3.63 cm (wrong sign — reads as a knee flexor)** | −5.02 cm | −4.42 cm |
| 30° | −0.11 cm (≈0, crosses zero here) | −4.64 cm | −4.61 cm |
| 60° | −3.97 cm | −3.72 cm | −4.03 cm |
| 90° | −6.54 cm (2.7× too large) | −2.42 cm | −2.39 cm |

(`recfem_r`; the other seven track it — full table in `measurements.json`.
Sign convention: `knee_angle` is flexion, so a knee extensor must be negative.)

RMSE against the coupled reference over 0–120°, all 4 right-leg quadriceps:
**4.79 cm before → 0.55 cm after** (max |error| 8.05 cm → 1.31 cm).

The mechanism behind "your quadriceps are not loaded" is the **path length**, not
the moment arm. Musculotendon length at 60° knee flexion:

| | recfem_r | vasint_r | vaslat140_r | vasmed_r |
|---|---|---|---|---|
| before | 1.285 m | 1.057 m | 1.068 m | 1.046 m |
| after | 0.560 m | 0.330 m | 0.344 m | 0.318 m |
| reference | 0.559 m | 0.325 m | 0.339 m | 0.312 m |
| `l_opt + l_ts` | 0.526 m | 0.322 m | 0.338 m | 0.318 m |

Before the fix every quadriceps sat 2–3.4× past the far end of its force–length
curve, where active force is ≈ 0 — so it *could not* generate force and the
optimiser left it at the activation lower bound. After the fix it is within 1.6%
of the reference.

### Muscles affected (8, named)

`recfem_r vasint_r vaslat140_r vasmed_r` and `recfem_l vasint_l vaslat140_l vasmed_l`
— exactly the muscles with a `patella_*` path point, verified by parsing the
`ForceSet`, not assumed.

### The edit

`nimblephysics/dart/biomechanics/OpenSimParser.cpp` skips the kneecap by three
**literal string comparisons** — `name == "patella_r" || name == "patella_l"`
(`:6147` readOsim30, `:6562` readOsim40) and
`name == "patellofemoral_r" || ... || childName == "/bodyset/patella_r" || ...`
(`:6737-6739`). FullBody.osim is `Version="40000"` so `:6562` and `:6737` are the
live ones. All three are cleared by renaming:

* body `patella_r` → `kneecap_r` (and `_l`)
* joint `patellofemoral_r` → `kneecap_r_jnt`, `CustomJoint` → `WeldJoint`
* offset frame `patella_r_offset` → `kneecap_r_offset`, `socket_parent`
  `/bodyset/patella_r` → `/bodyset/kneecap_r`
* the 9 muscle path points per side repointed to `/bodyset/kneecap_r`
* the two now-dangling `CoordinateCouplerConstraint`s removed (they name
  `knee_angle_{r,l}_beta`, which a WeldJoint no longer declares). nimble ignores
  constraints entirely, but leaving them would make the file invalid for any real
  OpenSim consumer.

`<Mesh name="patella_r_geom_1">` and `<mesh_file>r_patella.vtp</mesh_file>` were
**left alone**: nimble's checks read body/joint names only, and the mesh filename
is a real file reference.

### Weld, not a free DOF — verified, not assumed

A `WeldJoint` ignores the `SpatialTransform`, so the kneecap would land 5.24 cm
away from where the coupled joint puts it. The joint transform at β\* is therefore
**baked into the parent offset frame**: `translation` `-0.00809 -0.40796 -0.00275`
→ `0.04431 -0.41876 0`, `orientation` `0 0 0` → `0 0 0.00113686`.

β\* was chosen by sweep, not by taste (`measurements.json:weld_pose_selection`):

| β\* | RMSE 0–120° | RMSE squat band 60–110° |
|---|---|---|
| **0°** | **0.535 cm** | **0.336 cm** |
| 30° | 0.911 cm | 1.283 cm |
| 60° | 3.150 cm | 1.802 cm |
| 90° | 5.006 cm | 1.737 cm |

The alternative — rename but leave it a `CustomJoint` — was measured too. It adds
`knee_angle_r_beta` as a real DOF with `<range>±99999.9</range>`,
`<clamped>false</clamped>`, and **zero markers on the kneecap** (the `MarkerSet`
has 57 markers, none on the patella), so IK is free to park it anywhere. Sweeping
β at a fixed 60° knee swings the quadriceps moment arm from **−2.94 cm to +2.00 cm
— a sign flip**. There is also no way for nimble to enforce the coupler even in
principle: `CustomJoint::mFunctionDrivenByDof` indexes that joint's *own*
coordinate list (`OpenSimParser.cpp:4443-4470`), so a function driven by
`knee_angle_r` (a different joint's coordinate) is not expressible. Weld is right.

**The patella fix adds 0 DOFs** — the joint was skipped before and is welded now.

### Paired edit in `NimbleBridge.mm`

`groupScale()` classified the patella into the lower-limb scale group by
`bodyName.find("patella")`. Under the old name the body was never built so the
lambda never saw it; under the new name it does, and without a `kneecap` branch it
would fall through to `trunkScale` and scale the kneecap — and the four
quadriceps attachment points it carries — with the torso. Added
`bodyName.find("kneecap")` next to the existing `patella` test, which is kept as
an alias so a model revert doesn't silently break the bridge.

---

## Defect 2 — shoulders welded

### Before / after

| | before | after |
|---|---|---|
| `shoulder_R` joint class built by nimble | `WeldJoint` (crash-guard substitution) | `EulerJoint` XYZ, flips (−1, 1, 1) |
| `shoulder_R` DOFs | **0** | **3** |
| `shoulder_L` DOFs | **0** | **3** |
| skeleton DOFs | 163 | **169** |
| XML coordinates → DOFs | 171 → 163 (8 dropped) | 169 → 169 (none dropped) |

### The edit — 6 lines

The three rotation axes of `shoulder_R` / `shoulder_L` were not mutually
orthogonal, so nimble's `first3Linear` fast path hit
`assert(false && "3 rotation axis are not mutually orthogonal")`
(`OpenSimParser.cpp:5375`) — a no-op in Release — fell through with
`joint == nullptr`, and BioMotion's crash-guard at `:5791` substituted a
`WeldJoint`. Measured with an independent parse, exactly 2 of the 53 CustomJoints
do this, and the dot products reproduce STATUS.md exactly:

```
             before                    after
r1.r2   0.000004  (90.00°)  PASS   0.000000  (90.00°)  PASS
r1.r3   0.054150  (86.90°)  FAIL   0.000000  (90.00°)  PASS
r2.r3   0.084746  (85.14°)  FAIL   0.000000  (90.00°)  PASS      gate: |dot| < 1e-4
```

Values taken from `nimblephysics/data/osim/Return11/unscaled_generic_ortho.osim`,
whose "before" values are byte-identical to FullBody.osim's. That file is
**orphaned data** (`grep -r unscaled_generic_ortho nimblephysics/` → 0 hits); it is
used here as a *number* source only, and no upstream-endorsement argument is made.

### Angular change introduced, per axis

The three original axes are already exactly unit-norm, so the change is a pure
rotation of each:

| axis | coordinate | change |
|---|---|---|
| rotation1 | `shoulder_elv` | **3.379°** |
| rotation2 | `shoulder_rot` | **2.446°** |
| rotation3 | `elv_angle` | **2.446°** |

Whole-pose effect, sampled over the full declared coordinate ranges
(elv 0–115°, rot ±45°, elv_angle ±90°, 1521 poses): humerus orientation differs
from the original axes by **mean 3.96°, p95 7.32°, max 9.75°** (max at the corner
elv 115° + rot 45° + elv_angle −90°). For scale, the pipeline's input — ARKit body
tracking — is 18.8° ± 12.12° MAE.

### Does it still span the intended anatomical motion?

Yes. At pure elevation (`elv_angle = 0`, `shoulder_rot = 0`) the humeral long axis
sweeps the same arc; the elevation component is identical to 4 decimals
(−0.866 at 30°, −0.500 at 60°, 0.000 at 90°), and the snapped axes remove a
0.03–0.06 out-of-plane leak in x, i.e. elevation now happens *exactly* in the
plane the convention says it does. The three axes span R³ before and after
(rank 3), so no mobility is lost. Full-pose delta ≤ 3.5° along the pure-elevation
sweep.

A symmetric Gram-Schmidt orthogonalisation (the alternative) was measured too:
marginally better in the extreme corner (7.86° vs 9.75° max) but **worse per axis**
(3.11° / 4.87° / 0.00° vs 3.38° / 2.45° / 2.45°), and it would send nimble down the
`Euler(R-basis)` branch with its `errorOfCross` handedness heuristic instead of the
plain `getAxisOrder` → `EulerJoint` path. Unit-snap chosen.

Worth recording, because it contradicts the brief: the generic
`createCustomJoint<N>` path could **not** have represented this joint as-is. It
calls `getAxisOrder()` (`OpenSimParser.cpp:4424`) which requires the three
rotation axes to be exactly ±UnitX/Y/Z in one of four orderings and
`NIMBLE_THROW`s otherwise. Only the `R-basis` branch tolerates non-unit axes.

### Moment arms, right shoulder (cm)

Before: **identically 0 for all 144 measured entries** — not small, absent; the
coordinates are not DOFs of the built skeleton at all, so no upper-limb muscle can
carry any joint moment. After, 137 of 144 exceed 0.1 cm. Arm at side:

| muscle | `shoulder_elv_r` | `shoulder_rot_r` | `elv_angle_r` |
|---|---|---|---|
| DELT1 (ant.) | 0.23 | 0.14 | **+4.79** |
| DELT2 (mid.) | **1.48** (→ 3.73 at 90° elv) | 0.25 | 0.55 |
| DELT3 (post.) | 0.68 | −0.67 | **−4.71** |
| SUPSP | **2.08** (→ 0.83 at 90°) | −0.45 | 0.39 |
| INFSP | −0.26 | **−2.14** | 0.52 |
| SUBSC | 1.24 | **+1.47** | −0.38 |
| TMIN | −1.36 | −1.69 | −0.92 |
| TMAJ | −2.30 | 0.94 | −4.95 |
| PECM1/2/3 | −3.04 / −4.00 / −2.25 | 0.85 / 0.89 / −0.55 | 2.56 / 2.54 / 3.12 |
| CORB | −3.10 | 0.08 | 1.35 |

Four independent anatomical sanity checks pass: middle deltoid's elevation arm
grows 1.48 → 3.73 cm from 0° to 90° (the textbook abductor profile) while
supraspinatus falls 2.08 → 0.83 cm (the textbook initiator profile); subscapularis
is +1.47 on `shoulder_rot` while infraspinatus and teres minor are −2.14 / −1.69
(internal vs external rotators, opposite signs); anterior and posterior deltoid
are +4.79 / −4.71 on `elv_angle` (opposite signs); pectoralis and coracobrachialis
are negative on elevation (depressors).

**PathWrap is confirmed not to block this**: the 24 shoulder muscles carry **0**
PathWraps, re-counted here. So does a bonus 28 — the latissimus dorsi slips
(`LD_*`) also cross the glenohumeral joint with 0 PathWraps and also gain moment
arms. (No provenance/licence claim is made about them here.)

### Safety property

At `q = 0` the world transform of **every** body is bit-identical before and after
(max |ΔT| = 0.0 over all 78 shared bodies). The change adds DOFs without moving
the rest pose, so T-pose calibration and the virtual-marker offsets are unaffected.

---

## Regression check (in lieu of the test suite, which cannot be run here)

`python regression.py` → **PASS**. All 520 muscle path lengths compared before vs
after at 4 poses (neutral / squat / arms-up / trunk-twist):

| pose | muscles whose length changed | unexpected |
|---|---|---|
| neutral | 8 (the quadriceps) | **0** |
| squat | 8 | **0** |
| arms-up | 66 (8 quadriceps + 58 glenohumeral-crossing) | **0** |
| trunk-twist | 8 | **0** |

512 of 520 muscles are unchanged to 1e-9 m at every pose that does not move the
shoulder. Structural deltas are exactly the intended ones and nothing else:

* bodies added `{kneecap_r, kneecap_l}`, bodies removed `{}`
* joints added `{kneecap_r_jnt, kneecap_l_jnt}`, joints removed `{}`
* coordinates added `{shoulder_elv, shoulder_rot, elv_angle} × {r, l}`,
  coordinates removed `{}` (the two `*_beta` coordinates were never DOFs to begin
  with, which is why the DOF count goes 163 → 169 and not 163 → 167)

Neither hard constraint was violated: `sterR_clavR_jnt`, `clavR_scapR_jnt` and the
L pair are still `WeldJoint`s; all **54** spine/ribcage/sternum/abdomen bodies are
present with `max |ΔT| = 0` at `q = 0`; and all **28 trapezius + 20 serratus**
slips (48 total, matching STATUS.md's count) have length deltas of exactly 0.

---

## How it was measured, and what that does not prove

The iOS app was **not built or run** — that is forbidden for this workstream, and
no number here was observed on device. Instead `osim_kinematics.py` re-implements,
in Python, the two pieces of C++ that decide the answer:

1. `OpenSimParser.cpp::readOsim40()` — which bodies/joints get built and which
   joint class each becomes, including the patella skip, the `first3Linear`
   orthogonality gate and the BioMotion `WeldJoint` crash-guard. `SimmSpline` is a
   line-by-line port of BioMotion's patched `dart/math/SimmSpline.cpp`, including
   OpenSim's endpoint-tangent linear extrapolation; the Euler conventions were
   read out of `dart/math/Geometry.cpp` rather than assumed.
2. `MomentArmComputer.mm` — polyline length, the unresolved-body fallback to raw
   local offsets, ConditionalPathPoint latching, and `r = −ΔL/2ε` with ε = 1e-4 rad.

Evidence that it models the same skeleton nimble builds: without being told any of
them, it independently reproduces **163 DOFs**, **520 muscles = 422 Thelen + 98
Millard**, **exactly 2 crash-guard welds (shoulder_R, shoulder_L)**, the dot
products **0.000004 / 0.054150 / 0.084746**, **0 PathWraps on the shoulder**, and
**9 unresolved patella path points per leg** — every one of which STATUS.md records
as hand-verified. The reference column additionally lands the coupled-patella
quadriceps moment arm at 3.7–4.6 cm peaking near 15–30° flexion, which is the
published band for the quadriceps/patellar-tendon knee moment arm.

### Limitations, stated plainly

* **Computed, not observed.** A build-and-run on device is still required before
  any of this is claimed as shipped behaviour.
* **PathWrap is not implemented anywhere in this pipeline** — not in nimble's
  parse, not in `MomentArmComputer.mm`, not here. Each quadriceps carries exactly
  1 PathWrap, so the quadriceps absolute values carry that error in *all three*
  columns; past ~90° knee flexion the straight-line path cuts through the condyles
  and magnitudes are unreliable everywhere (visible as the reference itself turning
  back up between 105° and 120°). Differences between columns are meaningful;
  absolute deep-flexion values are not. The shoulder numbers are free of this —
  0 wraps.
* **The "reference" column is a yardstick, not a deliverable.** It applies the
  `CoordinateCouplerConstraint`, which nimble never enforces and now cannot,
  because the coordinate is gone.
* **Moment arms are geometry.** Non-zero, correctly-signed moment arms are a
  necessary condition for sane muscle output, not a sufficient one. Whether OSQP
  now reports loaded quadriceps in a real squat also depends on force-length state,
  inverse-dynamics torque quality and the ARKit input — none of which this touches.
* **Scaling was not exercised.** All numbers are at the model's unscaled size; the
  `groupScale()` patch is justified by code reading, not by measurement.
* **The 6 new shoulder DOFs are now unmasked and marker-observable only through
  one ARKit point per shoulder** (`RSJC`/`LSJC` at the humerus origin, plus the
  elbow). Elevation and elevation-plane are recoverable from the humerus direction;
  `shoulder_rot` (axial rotation of the humerus) is **not** observable from a
  point at each end of the segment. Expect `shoulder_rot_{r,l}` to be a null-space
  DOF. That is a solver/masking question, not a model question, and it is not
  addressed here.

---

## Tests that will change behaviour (could not be run — list, not verification)

| test | why |
|---|---|
| `DOFMaskTests.testDOFCountAccounting` | asserts `numDOFs == 163`; now 169. Asserts the 6 shoulder coordinates are absent as DOFs; they now exist. Asserts `171 − 8 == numDOFs`; the file now has 169 coordinates and drops 0. |
| `DOFMaskTests.testMaskComposition` | asserts `numDOFs == expectedNimbleDOFCount` and `numFreeDOFs == 106`; the 6 new shoulder DOFs are not in `runtimeMask`, so free DOFs become 112. |
| `DOFMaskTests.testMaskIsReversible` | asserts `numFreeDOFs == expectedNimbleDOFCount` after `clearDOFMask()`. |
| `E1MarkerSetComparisonTests.mm:1092` | `XCTAssertEqual((int)m.size(), 163)`. |
| `FullBodyDOFFixture.swift` | `expectedNimbleDOFCount = 163`, `expectedFreeDOFCountAfterMask = 106`, `coordinatesNimbleDrops` (8 entries), `xmlCoordinateCount = 171`, and the doc comment at line 238 all describe the pre-fix model. |
| `MomentArmTests`, `MuscleSolverTests` | no hard-coded counts found; `testSoleusAnkleMomentArm` and `MuscleSolverTests:30` (`recfem_r` present) should still pass. Listed because they touch the changed geometry. |
| `DOFMaskTests.testMaskedVsUnmaskedIK`, `IKDriftDiagnosticsTests` | print metrics rather than asserting them, but their numbers are now measured on 169 DOFs and are not comparable to previously recorded ones. |

**Not affected:** `NimbleBridgeTests` — including the deliberately-red
`testRepeatedIKOnIdenticalMarkersIsStable` — loads **Rajagopal2016**, not
FullBody (`NimbleBridgeTests.swift:16`, and it asserts `numDOFs <= 39`).
Rajagopal2016 was not touched. Leave that test red.

These test files were **not** edited — they are outside this change's ownership.
