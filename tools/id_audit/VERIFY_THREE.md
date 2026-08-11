# VERIFY_THREE — adversarial verification of the 2026-08-07 three-change wave

> **Historical raw-solver receipt — not a current muscle-output contract.** The static-hold and
> inverse-dynamics observations below predate the 2026-08-10 contact-support audit. Both bundled
> models have empty `ContactGeometrySet`s, and the active near-CoP routine has no validated support
> polygon, unilateral-contact, or friction constraint. Current production therefore keeps ID, GRF,
> CoP, muscle effort, and gait load nil even for a perfectly held pose or explicit floor. The old
> numbers remain useful only for unit, queue, and policy regression work.

> **Historical model-loader receipt.** The five-step raw-package/compile-cache
> findings in the Background Assets section also describe the 2026-08-07 code.
> The current runtime accepts only precompiled `.mlmodelc` directories, and the
> Simulator helper installs one only after receipt verification; see
> `tools/assetpack/README.md` for the active contract.

Scope: muscle-QP units, Background Assets delivery, static-hold detection.
Method: read `git diff` first; re-derive every load-bearing number by an independent
route; run the tests myself. No source was edited.

Environment: iPhone 17 Pro simulator `AE80D4E1-…` (test runs), iPhone 17 `F85FB2DA-…`
(app launch). Independent numerics: Python + `tools/osim_fixes/osim_kinematics.py`
(a re-implementation of nimble's parser + `MomentArmComputer`, not the shipped ObjC++).

---

## 0. Tests I ran

| Suite | Result |
|---|---|
| `StaticEquilibriumBenchmarkTests` | 7 / 7 pass |
| `OfflineMuscleChainTests` | pass |
| `OfflineOrchestrationTests` | pass |
| `MuscleSolverTests` | 11 / 11 pass |
| `MomentArmTests` | 11 / 11 pass |
| `DOFMaskTests` | 9 / 9 pass |
| `MuscleQPUnitsTests` (new) | 6 / 6 pass |
| `StaticHoldTests` (new) | 13 / 13 pass |
| **subtotal** | **59 executed, 0 failures** |
| `NimbleBridgeTests` | 22 executed, **1 failure** — `testRepeatedIKOnIdenticalMarkersIsStable`, the documented known-red test |
| `BodyJointTests` / `CalibrationTests` / `MotionRecorderTests` / `TRCExporterTests` | 44 / 44 pass |
| **total** | **125 executed, 1 failure, and it is the pre-existing red one** |

`git status` shows **no existing test file was modified**. The only touched sources are
`MuscleSolver.{h,mm}`, `NimbleEngine.swift`, three `Offline/` files,
`SAM3DPoseEstimator.swift`, `project.yml`, `Info.plist`, `project.pbxproj`, plus new
`AssetPack/`, `tools/assetpack/`, and two new test files. **No assertion was relaxed to
make anything pass.**

---

## 1. Muscle QP units — CONFIRMED, with two reporting gaps

### 1.1 I counted the muscled coordinates myself, on poses nobody used

Built the full 520 × 169 moment-arm matrix in Python at four poses — **neutral, arms
overhead, deep squat, trunk twist+bend** (the last three are mine; the agent used
neutral, a 4° lean and the dancer).

```
coordinates with max_m |R[m,j]| > threshold
 threshold     0     1e-11  1e-10  1e-8  1e-6  1e-4  1e-3
 neutral      165     159    159    159   159   159   159
 arms_overhead165     159    159    159   159   159   159
 deep_squat   165     159    159    159   159   159   159
 twist_bend   165     159    159    159   159   159   159
```

Bimodal gap at every pose: top of the low group `1.39e-12 … 1.67e-12`, bottom of the
high group `2.57e-3 … 4.13e-3`. **At least nine decades, on four independent poses.**
The excluded ten are exactly `pelvis_tilt / pelvis_list / pelvis_rotation /
pelvis_tx / pelvis_ty / pelvis_tz / wrist_flex_{r,l} / wrist_dev_{r,l}`.

**Verdict: the `kMomentArmFloor` 1e-10 → 1e-6 change is behaviourally inert and the
"structural, not tuned" claim is true.** I could not find a pose where it is not.

Also independently confirmed: 169 `<Coordinate>` elements; 54 `<locked>true</locked>`;
exactly six translational-only coordinates (`SternumX/Y/Z`, `pelvis_t{x,y,z}`);
`knee_angle_{r,l}` are named by *both* a `rotation1` axis and coupled `translation`
splines, so they are rotational; `T5_r5L_Z` really is `<TransformAxis name="rotation3">`
despite the `_Z`.

### 1.2 The SternumY claim is exactly right

Independent computation (gravity potential derivative, not nimble ID):
`τ(SternumY) = 72.6970 N` at neutral. Subtree below `r1R_sterR_jnt` = 13 bodies
(sternum, both clavicles, scapulae, humeri, ulnae, radii, hands) = **7.4105 kg ×
9.81 = 72.697 N**. The app's own ID prints `72.69697116205029`. Match to 5 decimals by
two unrelated routes. The claim that the arms' only load path is the sternocostal joint
is a fact of this model.

### 1.3 Did it lower the residual by REMOVING physics? — No, but the picture is more
### nuanced than the report says

The shipped default `excludesLockedCoordinates = YES` drops **50 coordinates that do
have muscles** (moment arms up to 0.189 m), taking the QP from 159 rows to **109**.
Measured effect (my run):

| pose | A: all muscled rows, unit-split | B: shipped (locked dropped) |
|---|---|---|
| upright | 0.2662 (10.238 Nm) | **0.2008 (7.717 Nm)** |
| lean 4° | 0.2052 (10.323 Nm) | **0.1526 (7.680 Nm)** |
| dancer | 0.6431 (112.31 Nm) | **0.6406 (111.81 Nm)** |

So yes, ~25 % of the standing improvement comes from removing rows. Five measurements
say it is nevertheless not a removal of physics:

1. **The locked set is anatomically principled, not convenient.** It is exactly the
   `rotation2` + `rotation3` axes of all 24 costovertebral joints (24 Y + 24 Z), plus
   `mtp_angle_{r,l}` and the 4 wrists. **All 24 `_X` axes — the rib pump-handle axis —
   stay free.** That is the standard one-DOF-per-rib convention, written by the model
   author, not chosen here.
2. **The dropped rows carry no demand.** `‖τ‖` on all 72 rib rows is 1.683 Nm against
   38.46 Nm for the rest; the specific locked rows read `τ ≈ 1e-23 … 1e-26 Nm` (rib
   bodies weigh 1e-4 kg). The 6.7 Nm of residual removed is pure `a_min` floor field.
3. **The cut does not take the worst rows.** Zeroing *all* 72 rib columns gives 0.0349;
   the shipped locked cut gives 0.2008. The 24 *unlocked* rib rows carry 7.60 of the
   remaining 7.72 Nm. If the goal had been to make the number small, this is not the
   cut you would make.
4. **No muscle loses its constraint.** Fraction of each muscle's `Σ|R|` lying on locked
   rows: 0 of 520 above 50 %. Trapezius mean 0.032 / max 0.057; serratus 0.075 / 0.133;
   scalenes 0.191 / 0.259. STATUS.md's warning that killing costovertebral articulation
   kills the scapular stabilisers applies to *welding the joint*, not to dropping the
   row — the moment-arm columns survive on the 109 remaining rows.
5. **Nothing is over capacity.** `rows_over_STATE_capacity = 0 of 159` at all three
   poses, using the state-dependent `forceScale` the QP actually uses.

### 1.4 The activation-floor mechanism reproduces independently

| quantity | agent (nimble ID + state forceScale) | me (Python gravity + F_max bound) |
|---|---|---|
| `‖τ‖` on the 72 rib rows, upright | 1.683 Nm | **1.687 Nm** |
| `a_min·A·1` field on those rows | 10.89 Nm | **12.11 Nm** (upper bound, uses F_max) |
| achieved residual there | 10.66 Nm (my run) | — |

New independent evidence for *why* it cannot be cancelled: one-sidedness
`|Σ_m R·F| / Σ_m |R·F|` has median **0.788 on rib rows** versus **0.074 on non-rib
rows** — rib musculature is ~10× more one-sided, so the floor field there is
structurally uncancellable. The worst upright rows are `T6_r6{R,L}_X` at ∓3.46 Nm
against `τ = 1e-23`, in an exact antisymmetric L/R pair. That is the floor field, not
unmet biomechanics.

### 1.5 Did the historical raw QP leave its activation box? — No

The diff does not touch the box constraint, the clamp, or `forceScale`. Activations are
still `clamp(x, aMin, aMax)` post-solve and `force = a · forceScale` with
`forceScale = F_max·f_AL·f_FV·cos α ≥ 0`. On a static hold `q̇ = 0` so `f_FV = 1` exactly.
No muscle is credited with force it cannot produce.

### 1.6 Two things the report understates

* **`coordsWithMuscles: 159` is not the shipped QP.** With the shipped default the QP
  solves **109** rows. The prose mentions the switch; the headline count does not.
* **The dancer's new `relativeForceResidual` is 0.467** — 33.98 N of a 72.70 N demand
  unmatched — and appears nowhere in the report. The narrative "the unit error was
  FLATTERING the number 2.1×" holds on upright (force rows match to 0.023 N and inflate
  the denominator). On the dancer the opposite is true: the split moves a 47 %-unmatched
  block *out* of the headline number.
* **The locked switch changes the rendered activations and nobody measured it.**
  `at_floor` goes 232 → 204 (upright) and 163 → 140 (dancer) when the switch flips —
  ~23–28 muscles change state. The overlay renders the top 24 by rank, so this can
  change what is drawn. Reported only as a residual effect.
* For calibration when reading 0.2008: on the 84 *non-rib rotational* rows the same
  solve reads **0.0568** (upright) / **0.0353** (lean) / **0.6242** (dancer).

### 1.7 Reproducibility

`repeat0/1/2` from three cold solvers are **bit-identical** at all three poses. The
λ 1→1e8 sweep moves the relative residual by 10.7 % (upright, upward) and 0.10 %
(dancer) — the reachability reading survives.

---

## 2. Background Assets — PARTIAL. Builds and fails visibly; **the load is not
## demonstrated on any path in this checkout.**

### What I verified

* `xcodebuild build-for-testing` exits 0.
* Built `BioMotion.app` = **53 MB** (Debug, simulator) and contains **no
  `SAM3DBodyPose.mlmodelc`**. The 1.3 GiB model is genuinely out of the bundle.
* `Extensions/AssetPackDownloader.appex` exists **and has an executable** — the earlier
  "appex with no executable" blocker is gone. Bundle id
  `com.soleil.BioMotion.AssetPackDownloader`, `EXExtensionPointIdentifier =
  com.apple.background-asset-downloader-extension`, `CFBundleVersion = 23`.
* `BackgroundAssets.framework` is linked; `BAUsesAppleHosting`, `BAHasManagedAssetPacks`,
  `BAAppGroupID = group.com.soleil.BioMotion` are all in the built `Info.plist`;
  the app-group entitlement is on both binaries.
* `build/assetpack/sam3d-body-pose.aar` = 1,096,262,256 B = **1.0210 GiB**, matching the
  claim. `Manifest.json` uses a directory selector on `SAM3DBodyPose.mlmodelc` and marks
  the pack `prefetch`.
* The app launches on the simulator, loads `FullBody.osim`, and logs
  `MuscleSolver: Loaded 520 muscles, 54 locked coordinates, 6 translational coordinates`
  — no crash from the new `SAM3DPoseEstimator.init()` prefetch task at launch.

### Failure is visible and does not hang — verified by code path

`resolveCompiledModelURL()` never awaits the transfer: it awaits only a detached
`AssetPackManager.url(for:)` probe, then `startDownloadAndDescribe()` which kicks a Task
and returns immediately. `OfflineSessionRunner` catches and sets
`phase = .failed("Couldn't load the pose model: …")`; `OfflineImportView` renders
`.failed` in a red `Section("Error")`. I could **not** drive the UI to see it — this
MCP configuration exposes `snapshot_ui` but no `tap`, so the two message strings the
agent quotes are unverified by me. Everything upstream of the render is verified.

### What is NOT verified, and it is the product-critical link

* **The model does not load in any configuration available here.** The simulator is
  served no pack, and `build/DevBundledModel/` is empty by default, so no path in this
  checkout reaches `MLModel(contentsOf:)`. The shipping path (`.mlmodelc` inside the
  pack) needs a TestFlight install on a device. `packUploaded: true` is not checkable
  from this machine.
* **Zero automated coverage.** No test references `AssetPackModelStore` or
  `SAM3DPoseEstimator`. The five-step resolution ladder, the interior-probe fallback and
  the stamp-based compile cache have no regression net at all.
* **Latent hang, out of the shipping config but reachable.** Paths 2 and 4 call
  `MLModel.compileModel(at:)` on a 1.3 GiB package *inside* `loadModelIfNeeded()`. That
  would block the first Run for minutes. Disclosed in the type doc; only reachable if a
  future pack ships uncompiled or a developer drops in the raw package.
* Minor report error: `CURRENT_PROJECT_VERSION` **was** changed in this diff (22 → 23).
  The report says "I left it at 23 (unchanged) deliberately."

---

## 3. Static-hold classification — historical detector receipt

### 3.1 I re-implemented the detector and reproduced the Swift bit-for-bit

Ported `StaticHoldDetector.ingest/classify` to Python, then replayed the Swift fixtures:

| | Swift (measured) | my port |
|---|---|---|
| 12-still / 8-moving / 12-still squat, dt 0.5 | holds 15, moving 17, peak 8.000 cm/s | **15 / 17 / 8.000** |
| single photo (9 replays) | hold, peak 0.0, span 0.26666666666666666, n 9 | **identical to 17 digits** |

So my adversarial sequences below are testing the shipped algorithm, not a paraphrase.

### 3.2 The historical raw path withheld a moving sequence

From the run I did:

```
HOLD-METRIC engine-moving isHold=false peak_cm_s=10.000  window_s=4.0
                          sawMuscle=false  lastMuscleResult=false
```
`lastSolve.muscle` and `lastSolve.id` are both nil across the whole clip. My own
sequences agree: constant 2.1 cm/s → not a hold; constant 5 cm/s → not a hold; every
constant-acceleration ramp from 0.005 to 0.08 m/s² → not a hold. The contaminated span
is exactly `motion + 8 frames`, i.e. a frame adjacent to real motion never claims
statics.

### 3.3 The historical raw path produced muscle output for one photo (now refused)

```
HOLD-METRIC photo        isHold=true peak=0.0 window_s=0.2667 samples=9 implied_accel=0.0
HOLD-METRIC engine-held  isHold=true static=true muscle=true peak_cm_s=0.0 center=0.0
```
At that historical engine state, gate ON, the runner's own 4+1+4 cadence, muscle and
ID were both non-nil and flagged as a static solve. This is no longer the product
contract: `.contactSupportUnavailable` now keeps the centred IK/hold verdict but
returns nil ID and muscle even with an explicit floor.

### 3.4 Threshold: defensible, but the "derived, not a knob" framing is circular

`0.02 m/s` and `0.5 s` **both** pre-exist in STATUS.md next-step 5 ("marker speeds
< ~2 cm/s for ≥ 0.5 s"), written before this work — so neither is reverse-engineered
from a fixture. `0.08 m/s²` is the only new constant, and it is *exactly* `2×0.02/0.5`.
The report's claim that "the 0.5 s duration is DERIVED from them, not a third knob"
inverts the dependency: 0.08 was chosen so that it reproduces the pre-existing 0.5 s.
"0.82 % of g" is a post-hoc description of that choice, not a budget set first. This is
fine engineering; the framing overstates it.

The cap itself is not fudged: my run of the 0.9×–1.1× sweep shows the verdict flip
exactly at `peak = 0.020` with no tolerance slack.

### 3.5 Gaps I measured that go beyond what is documented

| construction (mine) | verdict | note |
|---|---|---|
| constant **1.9 cm/s** drift, 2 fps | **HOLD** | markers travel 7.6 cm across the SG window; physically fine (zero acceleration) but the badge says "still: peak 1.9 cm/s" |
| **±1 cm every frame @ 2 fps** | **HOLD** (peak reads 2.00 cm/s) | true peak acceleration ≈ 0.16 m/s² = **2× the stated budget**. This is the documented sub-sampling blind spot; the magnitude is mine |
| **±2 mm every frame** | **HOLD @ 2 fps, MOVING @ 30 fps** | the verdict on identical physical motion inverts with frame rate |
| sustained accel a = 0.005 m/s² | **MOVING** | for *sustained* motion the detector is ~16× stricter than its own 0.08 budget, because speed accumulates over 4 s |
| still subject + SAM3D max 0.1 % bbox jitter (17.4 mm) | **MOVING** (3.48 cm/s) | independently reproduces the agent's unresolved item #4 |
| still subject + median jitter (2.4 mm / 6.0 mm) | HOLD | so the feature is plausible but genuinely unmeasured on real video |

### 3.6 Coverage gap and one latent inconsistency

* `OfflineOrchestrationTests` — the test whose job is to drive `NimbleEngine` *the way
  `OfflineSessionRunner` does* — **does not set `staticHoldGating`**, so the runner's
  actual shipping configuration is not exercised end to end. Engine-level coverage does
  exist in `StaticHoldTests`.
* `OfflineResultStore.updateBiomechanics` keeps `muscleResult ?? existing.muscleResult`.
  If a nil-muscle *moving* solve were ever routed onto a frame that already carried
  muscle, the UI would show "Pose + muscle" with a "moving … muscle loads need a still
  pose" caption underneath. I **could not construct a reachable path** — the match
  tolerance is 1 ms and solve centres are exact copies of distinct pushed timestamps —
  so this is latent, not live.

---

## 4. Bottom line

| change | verdict |
|---|---|
| Muscle QP units | **CONFIRMED inside the raw QP.** Row rule verified independently on 4 poses (159/10, ≥9-decade gap). The locked-row exclusion removes real rows but they are the model's own locked rib axes carrying ~1e-23 Nm of demand, and the biggest floor-field rows are *kept*. It did not remove rows from that raw objective; it says nothing about missing contact support. Two reporting gaps: shipped row count is 109 not 159, and the dancer's 0.467 force-row residual is unreported. |
| Background Assets | **PARTIAL.** Builds, ships without the model, extension has an executable, failure surfaces as a red error and never blocks on the transfer. But **no path in this checkout loads the model**, and the resolution ladder has zero test coverage. |
| Static hold | **Historical classifier confirmed; load output superseded.** Moving sequences were refused and one photo then reached raw muscle output. The 2026-08-10 capability gate now refuses load output for both. Thresholds were not fixture-fitted, though the "0.5 s is derived" framing is circular. Blind spots are real and mostly documented; I quantified two of them and found a third (frame-rate inversion). |

Regressions found: **none**.
