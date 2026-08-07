# BioMotion — STATUS

**Single source of truth for progress. Read this before touching anything.**
Last updated: 2026-08-07.

---

## TL;DR

The app's inaccuracy was diagnosed to root cause. It was never one bug — it is a chain, and the
biggest links were **not** where the effort had been going.

- Five implementation defects were found, fixed, and pinned with tests. The test target
  **did not even compile** before this work, so the project had no regression net at all.
- The dominant remaining error source is **not** the muscle solver: it is that IK solves
  **169 degrees of freedom from ~60 scalar observations** (20 markers × 3). ~127 of those DOFs are
  spine and rib coordinates no single-camera pose source can see.
- ~~The shipped model's **shoulders are welded** (zero shoulder DOFs), so upper-limb muscle output
  is currently meaningless regardless of anything else.~~ **Fixed 2026-08-06** — see
  [Muscle-output ship blockers](#muscle-output-ship-blockers-fixed-2026-08-06).
- A **commercial licence blocker** was found on the upper limb (MoBL-ARMS is non-commercial).
  A BSD-3 alternative exists.
- **IK is now a fixed point** (2026-08-07). Repeated solves on identical markers move exactly 0 rad,
  the answer no longer depends on how many solves preceded it, and the dancer fixture's true marker
  RMS went 5.4913 → 2.1224 cm. See
  [IK convergence](#ik-convergence-the-solver-is-now-a-fixed-point-2026-08-07).
- A **kinematics-only findings layer** ships (forward head, rounded shoulders, trunk lean, …). It
  carries **no clinical threshold and no verdict** and suppresses any finding whose measurement axis
  points into depth. See [Posture findings](#posture-findings-a-kinematics-only-layer-2026-08-07).
- **`cam_t` is the root translation the offline path was throwing away** — it is exported, stored and
  already used for the overlay, and the "needs SLAM" line in this file was wrong. Composing it back
  in is shipped and default-off; activating it is one argument at a call site another task owns.
  It is necessary and **not sufficient**: the depth channel carries 3.1 g of pure acceleration noise
  at 30 fps, and all three of the owner's clips are tracking shots with no inertial frame at all. See
  [cam_t recovers the root translation](#cam_t-recovers-the-root-translation-its-depth-cannot-be-differentiated-twice-2026-08-07).
- **Muscle force on running footage is achievable after all** (2026-08-07). The earlier
  "impossible on a tracking shot" verdict was wrong at the framing: root acceleration does not have
  to be measured, because the gait cycle supplies it (flight = free fall, stance closed by the
  stride's vertical impulse). Contact/flight timing IS robustly legible at 30 fps on the user's own
  clips — contact 200 ms with zero spread across a 2.5× threshold span — giving peak vertical GRF
  2.07–2.47 BW. Mac-validated, not yet implemented. See
  [Gait-cycle dynamics](#gait-cycle-dynamics-model-the-forces-instead-of-differentiating-the-measurement-2026-08-07).
- **"The skeleton doesn't match" is solved** (2026-08-07): `VNDetectHumanRectanglesRequest`
  defaults to `upperBodyOnly = true`, so the offline path was cropping the model's input to the
  torso and the legs were never in frame. Leg error 9.0% → 4.6% of subject height, torso unchanged.
  See [Device vs Mac](#device-vs-mac-solved--vision-was-returning-an-upper-body-box-2026-08-07).

There is **no known-red test any more**. `testRepeatedIKOnIdenticalMarkersIsStable` passes as
written; the tripwire that replaced its role is described under
[Known-red test](#known-red-test).

---

## Owner decisions already made

Do not re-litigate these; they are settled inputs.

| Question | Decision |
|---|---|
| Real-time required? | **No.** Offline / batch after capture is acceptable. |
| Where does processing run? | **On the phone.** Not a Mac companion, not cloud. |
| Muscle activation: qualitative or quantitative? | Owner wants **quantitative**. See the hard limits below — this is only partly achievable. |
| Target user | **Ordinary consumers**, not clinicians or researchers. |
| Product goal | Show muscle loading of the user's **current posture**; tell them **specifically** what is wrong. |
| Upper limb | **Required.** This rules out Rajagopal2016 (lower-extremity only). |

---

## The root-cause chain

Three symptoms that looked unrelated — IK drifting frame to frame, ~200 ms/frame solve time, and
jittery muscle activations — are one root cause.

1. **Observability.** 20 virtual markers = 60 scalar observations against **169 DOFs**
   (was "163 DOFs / ~12–14 markers" when this was written). The marker Jacobian is 60×169 with rank
   ≤ 60; **72 of its columns are identically zero**
   (`FullBodyDOFFixture.structurallyUnreachableCoordinates`), and the rest of the null space is
   near-singular rather than exact. Solutions there are artifacts of the warm start, not
   measurements.
2. **Amplification.** Those unconstrained DOFs wander; the wander is differentiated twice by the
   Savitzky–Golay filter (gain ≈ 1/dt² ≈ 3600 at 60 fps) to produce `ddq`.
3. **Propagation.** `τ = M·ddq + C − JᵀF_ext` is linear in `ddq`, and the muscle QP is
   τ-match-dominant (`softPenalty = 100`), so activations inherit essentially the full amplified
   torque noise. Pose error also enters a *second* time through `R(q)`, so the two compound
   bilinearly rather than additively.

**Implication:** fixing things downstream of IK has limited headroom. The leverage is at IK and above.

**2026-08-07 — link 1 is now bounded, and the chain is broken at link 2.** The rank deficiency has
not gone anywhere: 72 columns are still exactly zero. What changed is that the solver no longer
*moves* in that null space. Phase A damps toward the seed and phase B's steps lie in the row space
of `J`, so all 72 unreachable coordinates come back at exactly their seed value and repeated solves
on identical markers move exactly 0 rad. With no wander there is nothing for the filter to
differentiate — measured on the dancer fixture, per-solve drift 0.0 and the static-vs-dynamic peak
torque identical to 16 significant figures.

⚠️ **This is a stability result, not an accuracy result.** E1 already established that shrinking the
null space buys smoothness, not spinal truth, and that no marker set beats the null model on
per-intervertebral angles. Do not read "IK is a fixed point" as "the spine coordinates mean
something". See [the spine-claim constraint](#the-constraint-this-puts-on-product-claims).

---

## Phase 0 — done, verified, committed

All five defects were verified by direct code inspection *and* independently re-verified before
being reported. Every fix has a test that fails on the old behaviour.

| Defect | Location | Fix |
|---|---|---|
| Force–velocity curve returned **negative** force | `MuscleSolver.mm:63-76` | Old `1+v(1−0.25v)` crossed zero at `v ≈ −0.828` and returned `−0.25` at `v = −1`, contradicting its own doc comment. Replaced with normalized Hill hyperbola `(1+ṽ)/(1−ṽ/A_f)`, `A_f = 0.25`. Verified: exactly 0 at ṽ=−1, exactly 1 at ṽ=0, non-negative and strictly increasing on [−1,0]. |
| 418 `ConditionalPathPoint` + 4 `MovingPathPoint` silently dropped | `MomentArmComputer.mm:168` | tinyxml2 name-matched iteration skipped them. Now walks **all** children in document order (ordering matters — these are polyline vertices). Adds a load-time geometry-fidelity report. |
| Ground height was a **monotonic ratchet** | `NimbleBridge.mm:634-638` | One crouch/landing/drift permanently sank it → both feet read >6 cm above "ground" → `contactCount==0` → ID solved a free-floating body with **zero external force for the rest of the session**. Replaced with a bounded rolling robust percentile that can rise as well as fall, plus a reset hook. |
| `jointVelocities` accepted but never read | `MuscleSolver.mm:573,593` | Fiber velocity came from wall-clock finite differencing whose `dt` jittered with dropped frames and disagreed with the SG filter's own `dt`. Now uses the analytic identity `dL_MT/dt = −Rᵀ·dq`. Sign convention derived from `MomentArmComputer.mm:381` (`R = −∂L_MT/∂q`), not guessed. |
| IK did **5 random restarts every frame** | `NimbleBridge.mm:472,510` | `IKConfig.lossLowerBound` defaults to 0 (header) / 1e-10 (ctor) — both unreachable against a realistic 0.01–0.03 m ARKit marker residual, so the restart loop always ran to completion, each iteration calling `getRandomPose()` and discarding the previous solution at 171 DOF. Now 1 restart, warm-started from the previous pose, plus a static marker-reliability weighting (trunk 1.00 → toes 0.40). |

Also: torque residual now exposed on `MuscleActivationResult`; `aMin` comment no longer justifies an
optimizer bound by colormap appearance; dead `maxMuscleForceAtState` and the legacy
`solveWithJointTorques` hardcoded-moment-arm path removed (verified zero non-test callers).

### Test suite

| | 2026-08-06 start | after Phase 0 | 2026-08-07 |
|---|---|---|---|
| Test target | **did not compile** (`MomentArmTests.swift:124` used a stale signature) | builds | builds |
| Tests | 0 runnable | 88 total, 87 pass | **219 total, 219 pass, 0 crash-restarts** |

`E1MarkerSetComparisonTests` is EXCLUDED from that count. It costs over an hour and it currently
fails at `E1MarkerSetComparisonTests.mm:475` for a pre-existing reason — its fixture enumerates 163
coordinates against a 169-coordinate model, a leftover from the 2026-08-06 osim edit. See
next-step 14.

Run with:
```bash
# XcodeBuildMCP session defaults: project BioMotion.xcodeproj, scheme BioMotion,
# simulator "iPhone 17" (F85FB2DA-66AD-428D-A4EC-1390A42D9FCB), Debug
xcodebuild -project BioMotion.xcodeproj -scheme BioMotion \
  -destination 'platform=iOS Simulator,name=iPhone 17' \
  -skip-testing:BioMotionTests/E1MarkerSetComparisonTests test
```

Per class (2026-08-07): NimbleBridgeTests 22 · PostureFindingsTests 26 · IKConvergenceTests 13 ·
TRCExporterTests 14 · StaticHoldTests 13 · MomentArmTests / CalibrationTests / MuscleSolverTests 11 ·
BodyPlausibilityTests 11 · MotionRecorderTests 10 · BodyJointTests / DOFMaskTests 9 ·
StaticEquilibriumBenchmarkTests 7 · IKDriftDiagnosticsTests / MuscleQPUnitsTests 6 ·
ShoulderRotMaskTests 5 · ShoulderRotObservabilityTests 3 · OfflineMuscleChainTests /
OfflineOrchestrationTests 1.

### Known-red test

**RESOLVED 2026-08-07 — there is no known-red test.** `testRepeatedIKOnIdenticalMarkersIsStable`
passes against its ORIGINAL 1e-3 rad bound, unedited, at 0.0 measured drift. Do not re-add a
"leave it red" instruction. The entire section below is kept because its *diagnosis* is what the
fix was built from, and because two of its warnings are still live: this test must never be used as
a score (reasons 1–3), and its 31× run-to-run swing is what "not a stable measurement" looks like.

One assertion elsewhere flipped as a consequence, and it was a **pre-registered tripwire doing its
job**: `StaticHoldTests` asserted `maxConsecutive > 0` with the message *"if IK became reproducible
on identical markers, the artifact this feature removes is gone and the numbers below must be
re-derived"*. IK did become reproducible. The assertion is now inverted (`== 0`, exactly), the
numbers are re-derived in that method's header, and the consequence is recorded rather than hidden:
**static-hold gating is a measurable no-op on a hold** — `dynamicA = dynamicB = static =
84.10433817558118 Nm`, control delta 0, treatment delta 0. Its remaining justification is entirely
the other branch, refusing to publish muscle magnitudes for a MOVING frame.

---

The original diagnosis, kept for its reasoning:

`NimbleBridgeTests.testRepeatedIKOnIdenticalMarkersIsStable` — **leave it red.** It is not a stale
test; it surfaces a real defect: **IK has no null-space damping toward the seed pose.**

Evidence: warm-started IK on *identical* markers drifts 0.006 rad per solve. Making the fixture
*more* three-dimensional (soft-knee z-offset, toe markers) made drift **worse**, not better
(0.006 → 0.19 → 0.84 rad). That is the signature of near-singularity amplification in a damped
pseudo-inverse, not an exact null space. Compounding it, nimble's `refineIK`
(`IKSolver.cpp:321`) terminates only on error-*change* < 1e-7 or step count, and **never consults
`lossLowerBound`**, so every call keeps stepping along the flat manifold.

Do not fix this by loosening the tolerance or editing the fixture. Fix the solver.

Related: the drift value varies between identical full-suite runs, and `NimbleBridge.mm:296`
documents that the skeleton is **shared across instances** — so tests are order-dependent. Test
isolation is its own open item.

**2026-08-06 — the diagnosis above is confirmed, but this test must NOT be used as a score.**

Root cause, measured by a four-arm causal experiment: the marker Jacobian is rank-deficient, the IK
objective is flat along the unobservable directions, and `refineIK` terminates on an absolute
error-*change* test (< 1e-7 m²) rather than on stationarity — so it stops while `q` is still moving,
and every new call resets `lr` to 1.0 and resumes. With a full-column-rank Jacobian the solver
reaches a fixed point within ≤4 solves (drift 6.1e-16 rad); at rank 21/37 it still drifts
3.5e-3 rad/solve after 40 solves **even with an exactly reachable, zero-residual target**.

Three reasons it is a broken instrument:

1. **Confounded.** A solver-only change (gradient-based termination, or null-space damping toward the
   seed) turns it green with *zero* accuracy gain.
2. **Not a stable measurement.** Same binary, same minute, no rebuild: running the single test gives
   `0.00575`, running its enclosing class gives `0.00984`, running the 111-test suite gives `0.17676`
   — a **31× swing**, driven by the process-global `std::rand()` stream feeding the cold solve's five
   random restarts.
3. The marker residual cannot score it either, because the loss is flat exactly where `q` moves.

Score observability work on ground-truth coordinate error instead (see E1 below).

---

## The upper-limb licence blocker

`FullBody.osim` `<credits>` = *"Rajagopal et al., Lai et al., Allaire et al., Saul et al.,
McFarland et al."*. **Saul et al. = MoBL-ARMS**, whose SimTK licence states verbatim that it is
open sourced *"solely for non-commercial purposes"* and that *"commercial use requires a commercial
license"*. A repo-level MIT badge cannot sublicense an NC upstream.

**Verified licence findings** (each read verbatim from source, not inferred from a badge):

| Model | Licence | Commercial | Notes |
|---|---|---|---|
| **Stanford-VA Upper Limb (Holzbaur, Murray & Delp 2005)** — [group_id=324](https://simtk.org/frs/?group_id=324) | **BSD 3-Clause** | ✅ **yes** | No NC clause (confirmed by extracting the `data-content` popover from raw HTML — WebFetch's markdown conversion strips it, which is why earlier passes only saw the label "Custom Use Agreement"). Clause 3 forbids using "Stanford"/"VA Palo Alto"/author names in marketing. Distributed as **SIMM** format (2008) — conversion needed. |
| MoBL-ARMS (Saul 2015) — [group_id=657](https://simtk.org/frs/?group_id=657) | CC ANC | ❌ no | The *successor* to Holzbaur 2005. Same lab, same template, NC sentence added. |
| Thoracoscapular Shoulder (Seth 2019) | CC BY 4.0 | ⚠️ **do not use** | Package grant is real, but its scapulothoracic joint comes from **Seth et al. 2016, which is CC BY-NC 3.0** — same permissive-wrapper-over-NC structure as the current blocker. **And** `OpenSim::ScapulothoracicJoint` is a 4-DOF class shipped **only as a binary plugin** (.dylib/.dll); nimble cannot construct it at all. |
| `arm26.osim` | CC BY 3.0 (verbatim in-file `<credits>`) | ✅ yes | Only 2 DOF / 6 muscles. Useless alone, but proves the Holzbaur upstream can carry a permissive grant. |
| Wu shoulder model | CC ANC | ❌ no | |
| `LaiUhlrich2022_shoulder.osim` (OpenCap) | Apache 2.0 (repo badge — **not** an in-file grant) | ⚠️ unverified | Torque-driven arm *skeleton*, no arm muscles. Same Rajagopal/Lai naming as the existing lower body → near-zero remapping cost. Good fit for a kinematics-only arm. |

⚠️ **Method rule learned the hard way:** OpenSim models are **not** uniformly licensed. Official docs:
*"models, examples and plugins retain their own custom licenses. You should refer to each individual
file for license information."* `opensim-models` has **no repo-level LICENSE file at all**. An
earlier research pass got this exactly inverted — it flagged the two safe components as risky and
missed the blocking one. **Always quote verbatim licence text with a URL; never infer from a badge;
always trace the provenance chain of composite models.**

**Scope of the taint is smaller than first thought: 42 muscles, not 88.**
24 shoulder (DELT1-3, SUPSP, INFSP, SUBSC, TMIN, TMAJ, PECM1-3, CORB ×2) + 18 elbow.
The **48 trapezius/serratus muscles come from the Bruno/Allaire spine+ribcage model, which is
verified stock MIT** and is not affected.

---

## Model facts (verified — do not re-derive)

`BioMotion/Resources/FullBody.osim` (production, `cyclistFullBodyMuscleActuated`):

- **169 XML coordinates → 169 DOFs parsed**; ~127 are spine + rib. It was 171 → 163 before the
  2026-08-06 `tools/osim_fixes` edit: the patellofemoral weld removed the two `knee_angle_*_beta`
  coordinates from the XML, and the shoulder axis unit-snap stopped nimble dropping 8 (2
  patellofemoral + 6 shoulder). Nothing is dropped any more.
  ⚠️ `CLAUDE.md` said 171 until 2026-08-07; anything quoting 171 or 163 predates the osim edit.
- 520 muscles = 422 `Thelen2003Muscle` + 98 `Millard2012EquilibriumMuscle`.
- `PathPoint` 1444 · `ConditionalPathPoint` 418 · `MovingPathPoint` 4 · `PathWrap` 76 ·
  `WrapCylinder` 60 · `WrapEllipsoid` 9 · 15 `WeldJoint` · 53 `CustomJoint`.
- **PathWrap distribution** (settles a contradiction between two analyses): shoulder 24 muscles →
  **0 wraps**; trapezius/serratus 48 → **0 wraps**; elbow 18 → **16 have wraps**.
  ⇒ The unimplemented-wrap gap does **not** block the shoulder. Unwelding the shoulder really does
  deliver usable shoulder moment arms.

`BioMotion/Resources/Rajagopal2016.osim` (fallback, lower-extremity only): 80 muscles,
288 `PathPoint`, 0 conditional/moving, 46 `PathWrap`, 40 `WrapCylinder`. Parses cleanly.

### Why the shoulders are welded

`shoulder_R` / `shoulder_L` are `CustomJoint`s with 3 coordinates and 6 `TransformAxis` entries.
Their three rotation axes are **not mutually orthogonal**:

```
r1·r2 = 0.000004  → 90.00°  passes
r1·r3 = 0.054150  → 86.90°  FAILS  (540× over the 1e-4 gate)
r2·r3 = 0.084746  → 85.14°  FAILS  (847× over)
```

This is a deliberate anatomical convention (Holzbaur/Rajagopal humeral elevation plane), **not bad
data**. nimble's `first3Linear` fast path (which tries to recognise a disguised EulerJoint) demands
`|dot| < 1e-4`, hits `assert(false && "3 rotation axis are not mutually orthogonal")`
(`OpenSimParser.cpp:5375`) — **a no-op in Release** — falls through with `joint == nullptr`, and
BioMotion's own crash-guard patch at `OpenSimParser.cpp:5791` substitutes a `WeldJoint`.
Of all 53 CustomJoints, **only these two** trip it.

The generic `createCustomJoint<N>` path (`OpenSimParser.cpp:~5501`) does **not** require
orthogonality and could represent this joint fine — it is simply never reached.

**The fix is 6 lines of XML**, and the exact corrected values already exist in-repo:
`nimblephysics/data/osim/Return11/unscaled_generic_ortho.osim` differs from `unscaled_generic.osim`
by **exactly 6 lines** — the same shoulder axis triple, snapped to unit vectors, byte-identical
"before" values to `FullBody.osim`. ⚠️ Caveat: `grep -r unscaled_generic_ortho nimblephysics/`
returns **zero hits**, so that file is orphaned data, **not** an upstream-endorsed fixture. The
numbers are ground truth; the "upstream chose this approach" argument is not.

### Other model gotchas

- **The shoulder girdle has no DOF even in principle.** `sterR_clavR_jnt`, `clavR_scapR_jnt` and the
  L pair are literal `<WeldJoint>` in the XML. Scapular protraction / winging / scapulohumeral
  rhythm are unobtainable from this model — and unobservable from ARKit anyway (one point per
  shoulder).
- **Patella is skipped by literal string match**, not by joint type: `OpenSimParser.cpp:6147`,
  `:6562`, `:6737-6739`. Changing the joint to a WeldJoint does **not** help — the body must be
  **renamed**. Its coordinate is driven by a `CoordinateCouplerConstraint`, which nimble does not
  enforce. Consequence today: ~8 quadriceps muscles have zero knee moment arm and sit pinned at the
  activation lower bound — i.e. **the app would report "your quadriceps are not loaded" during a
  squat**. Ship blocker for the flagship posture.
  ⚠️ If the rename is done, `NimbleBridge.mm` `groupScale()` must be patched in the same commit — it
  matches `bodyName.find("patella")` to assign the lower-limb scale group, so a rename silently
  reassigns the patella to `trunkScale`.
- `CLAUDE.md`'s "OSQP ~0.5 ms / IK ~1 ms" figures are **stale** — measured on Rajagopal2016
  (81 muscles / 39 DOF), not the shipped FullBody (520 / 171). Real cost is ~200 ms/frame, which is
  why builds 15–16 needed frame-dropping backpressure.

---

## Accuracy ceilings (external evidence)

| Method | Joint-angle error | Source |
|---|---|---|
| Marker-based (Vicon, gold standard) | ~2–3° | conventional figure, no single citation |
| OpenCap (2 synchronised cameras) | **4.1° MAE** (RMSE 2.0–10.2°) | Uhlrich et al. 2023, PLoS Comput Biol |
| OpenCap Monocular (1 phone camera) | **4.8° MAE** | [arXiv:2603.24733](https://arxiv.org/abs/2603.24733) |
| SAM 3D Body / MHR (1 phone camera) | **4.66° raw**, 3.46° centred | [arXiv:2607.17639](https://arxiv.org/abs/2607.17639) — **v1 preprint, not peer reviewed**, n=9, needs a fitted per-model offset table |
| **ARKit body tracking (current input)** | **18.8° ± 12.12° MAE** (per-joint 3.75–47°) | [doi:10.3390/app12104806](https://doi.org/10.3390/app12104806) (peer reviewed) |

⚠️ **These rows are not mutually comparable, and no ratio derived from them is a measurement.**
The cohorts, movements, ground-truth pipelines and — decisively — the *DOF sets* differ. The ARKit
study covers shoulder/elbow/neck/knee/ankle; the SAM 3D Body study covers ankle/knee/hip/pelvis/torso.
They barely overlap, and ARKit's worst per-joint errors are likely in the upper limb, which the
SAM 3D Body study never measured. Offset correction is also asymmetric (raw vs centred).

Apple has **never published any accuracy figure** for ARKit body tracking or
`VNDetectHumanBodyPose3DRequest` (checked: API docs + WWDC23 "Explore 3D body pose and person
segmentation in Vision"). Any "N× better than ARKit" claim therefore has no measured denominator.

**Removed 2026-08-06:** a row reading `Monocular SMPL (HMR2.0 class) | 8.5° MAE`. It carried no
citation and none could be found. `18.8 ÷ 8.5 = 2.21` was the sole origin of the belief that
monocular mesh recovery is "about 2× more accurate than ARKit". Do not reintroduce an uncited
number here — see `labs/sam-3d-body/findings/DECISION.md` §6.

**Do not attempt real-time ViT-H HMR2.0 or WHAM+SLAM on iPhone.** ViT-H is ~630M params (~1.2–1.3 GB
fp16); the reference implementations are offline CUDA batch scripts. HMR2.0 is also per-frame and
camera-relative — it fixes single-frame pose plausibility while leaving acceleration and GRF, the
actual muscle-side bottlenecks, untouched. WHAM's expensive half (camera pose + gravity) is already
solved for free by ARKit; the useful part to borrow is its **architecture** — a small AMASS-trained
temporal denoiser + foot-contact head, ANE-sized — not its code.

**2026-08-06 — partially superseded by measurement.** The *compute* half of this paragraph was
pessimistic; the *value* half was right, and is now the whole argument.

Measured on M2 Pro for SAM 3D Body (`vith`, a 631.65M ViT-H): body-only path **0.70 s/image**
(the often-quoted 2.52 s is the full pipeline including a hand decoder this project does not use),
fp16 body-path weights **1.285 GiB**, and max activation magnitude sits **145× below the fp16
ceiling** — so the AutoLevel DINOv2 fp16-overflow experience does not transfer to this model.
"Not real-time" still holds; "cannot run on the phone" does not. Still-image tier is plausible.

**2026-08-06, second update — the model now ships.** It was converted to Core ML and integrated, but
as a **new video/photo input path**, not as a replacement for the ARKit marker set. See
[SAM 3D Body integration](#sam-3d-body-is-integrated-as-an-offline-videophoto-path-2026-08-06).
The paragraph below remains correct about what it is correct about: a better per-frame pose source
does not buy spinal observability.

The reason not to swap the marker set is **observability, not compute**. MHR carries 4 spine anchors
and **0 rib anchors** against this model's 54 spine and 72 rib coordinates, so ≥114 of the spine+rib
DOFs stay in the null space before and after the swap. Meanwhile 54 coordinates in `FullBody.osim`
already carry `<locked>true</locked>` and nimble appears not to honour it (171 − 54 = 117, yet
nimble reports 163 DOF). Runtime DOF masking is the cheaper lever by an order of magnitude.
Full accounting, measurements and pre-registered gates: `labs/sam-3d-body/findings/DECISION.md`.

### Hard limits on "quantitative muscle activation"

- The QP has ~520 unknowns against ~110 torque equations. The ~410-dimensional null space is filled
  **entirely by the cost function**. `epsA`/`lambda`/`aMin` were tuned — per the original code
  comments — so the visualization would not go *"permanently blue"*. That is a rendering parameter,
  not a measurement.
- Static optimization **structurally cannot see co-contraction**: it only needs to match net joint
  moment, so antagonists go to the activation floor. Literature confirms this
  ("Static optimization underestimates antagonist muscle activity"). Bracing, guarding and
  quad-dominance — the most common consumer "faulty force production" patterns — are exactly the
  subspace it cannot represent. Only EMG sees them.
- In quasi-static posture, activation is a **deterministic function of pose + bodyweight +
  anthropometry**. Same pose ⇒ same numbers, regardless of who is standing there. It renders the
  *posture*, not the *person*.
- Double-support L/R load split is **statically indeterminate**: `F_L·x_L^cop + F_R·x_R^cop =
  mg·x_com` is 4 unknowns / 2 equations with each foot's CoP free inside its polygon. With a
  *perfectly known* CoM and a 25 cm stance this alone gives **±18 pp**. Healthy adults sit ~2 pp
  from 50/50 and ~10 pp is the clinically meaningful threshold — **the instrument cannot resolve
  the effect it would be measuring**. `NimbleBridge.mm:652` additionally hardcodes a 50/50 wrench
  prior that the near-CoP objective pulls toward, so any L/R asymmetry reported today is an
  artifact.
  **Single-leg stance is statically determinate** and is the way out.

**What *is* defensibly quantitative:** joint angles in the sagittal plane, plumb-line deviation (cm),
hold duration and sway, CoM position within the base of support, and — once GRF is known
(single-leg) — **joint moments** (Nm/kg). Joint moments are uniquely determined by inverse dynamics;
the redundancy problem lives strictly *downstream* of them. That is the honest muscle-adjacent
number.

---

## Muscle-output ship blockers (fixed 2026-08-06)

Both defects below were long-diagnosed in this file and are now repaired. The edit, the
measurement harness and the revert instructions are in `tools/osim_fixes/`; the pristine model is
kept at `tools/osim_fixes/FullBody.osim.orig`.

**Measurement method matters here:** the app was NOT built or run to produce these numbers.
`tools/osim_fixes/osim_kinematics.py` is a Python re-implementation of the two pieces of code that
decide the answer — nimble's `OpenSimParser.cpp` joint construction (including the patella
literal-name skip, the `first3Linear` orthogonality gate and BioMotion's WeldJoint crash-guard) and
`MomentArmComputer.mm`'s polyline + central-difference moment arm. Before, after and a coupled
reference all run through the same code, so differences are attributable to the model change.
Its credibility rests on independently reproducing figures this file records as hand-verified:
163 DOFs, 520 muscles = 422 Thelen + 98 Millard, exactly 2 crash-guard welds, the dot products
`0.000004 / 0.054150 / 0.084746`, 0 PathWraps on the 24 shoulder muscles, and 9 unresolved patella
path points per leg.

### Quadriceps

`patella_r/l` → `kneecap_r/l`, and the patellofemoral `CustomJoint` → `WeldJoint` with the
`knee_angle_beta = 0` spline evaluation folded into the fixed offset (verified: at `q = 0` every
body's world transform is bit-identical before and after). `NimbleBridge.mm` `groupScale()` gained a
`kneecap` branch in the same change — without it the renamed body silently scales with the torso.

| Quadriceps knee moment arm | Before | After | Coupled reference |
|---|---|---|---|
| at 30° flexion (mean of 4 R-leg quads) | −0.0014 m | **−0.0463 m** | — |
| RMSE vs reference over 0–120° | 4.79 cm | **0.55 cm** | — |
| max abs error | 8.05 cm | **1.31 cm** | — |

⚠️ **This file was wrong about the mechanism.** The quadriceps did not have a *zero* moment arm.
They had a **sign-inverted, 2.7–5× mis-scaled** one (+3.63 cm at 0°, reading as a knee *flexor*;
−6.54 cm at 90°) that merely happens to cross zero near 30°. The "pinned at the activation lower
bound" conclusion still holds, but via force-length collapse from a 2–3.4× over-long path, not via a
zero moment arm.

### Shoulders

The two shoulder axis triples were snapped to unit vectors, so `first3Linear` no longer trips the
`|dot| < 1e-4` gate and the WeldJoint crash-guard no longer fires. Axis changes: 3.38°, 2.45°, 2.45°.
Shoulder DOFs **0 → 3 per side**; 137 of 144 measured moment arms now exceed 0.1 cm and pass four
anatomical sign checks (anterior vs posterior deltoid `+4.79 / −4.71` on `elv_angle`; subscapularis
`+1.47` vs infraspinatus `−2.14` and teres minor `−1.69` on `shoulder_rot`).

⚠️ **Correction to this file's earlier claim** that "the generic `createCustomJoint<N>` path does not
require orthogonality and could represent this joint fine": only half right. `createCustomJoint`
calls `getAxisOrder()` (`OpenSimParser.cpp:4424`), which requires the three rotation axes to be
exactly ±UnitX/Y/Z in one of four orderings and `NIMBLE_THROW`s otherwise. Only the Euler(R-basis)
branch tolerates non-unit axes. "Just reach the generic path" would not have worked — the unit-snap
was necessary, not merely convenient.

### What this does NOT establish

- **PathWrap is still unimplemented pipeline-wide.** Each quadriceps carries exactly 1 PathWrap, so
  quadriceps *absolute* values carry that error in every column, and past ~90° knee flexion the
  straight-line path cuts through the condyles. Differences between columns are meaningful; absolute
  deep-flexion values are not. The 24 shoulder muscles carry 0 PathWraps and are free of this.
- Moment arms are geometry — necessary but not sufficient. Whether OSQP now reports loaded
  quadriceps in a real squat also depends on force-length state and inverse-dynamics torque quality.
- `shoulder_rot_{r,l}` (humeral axial rotation) is **not observable** from one point per shoulder
  plus one at the elbow. Per this file's own E1 finding that unobservable DOFs get excited by the
  solver, those two should enter the runtime DOF mask before shoulder output is trusted.
- Coordinate counts changed: **171 → 169** XML coordinates (the 2 `knee_angle_*_beta` are gone with
  the weld) and **nothing is dropped any more** (was 8: 2 patellofemoral + 6 shoulder). Test fixtures
  in `FullBodyDOFFixture.swift` were updated to match.

---

## E1 — pose-source replacement was tested and REJECTED (2026-08-06)

**Verdict: STOP. Do not replace ARKit with SAM 3D Body / MHR markers.** Gates were frozen to disk
2h09m before any result existed (`E1_PREREGISTRATION.md` 10:49:00, `E1_results.json` 12:57:22), and
an independent adversarial audit recomputed every gate from the raw JSON and could not overturn it.
Full record: `labs/sam-3d-body/findings/{E1_PREREGISTRATION.md, E1_results.json, E1_gates.json}`;
harness `BioMotionTests/E1MarkerSetComparisonTests.mm`.

Design: three arms on the same synthetic scripted motion with known ground-truth `q(t)`
(120 frames @ 30 fps, 30° torso flexion distributed evenly across the 17 intervertebral joints,
markers = `FK(q_true)` + i.i.d. gaussian σ = 8 mm, fixed seeds).
**A** = the 20 shipped ARKit virtual markers · **A′/A″** = A plus runtime DOF masking ·
**A_damp** = A plus the null-space-damped solver · **B** = 25 MHR-derived markers.

| Gate | Result | Decisive number (5/5 seeds, both seed conditions) |
|---|---|---|
| G3 spine truth | **FAIL** | `E_B` 0.2124 rad vs `E_A` 0.1997 — B is *worse*, and 10.8× the null model |
| G5-CROSS | **FAIL — STOP** | B / A_damp = **4.20** (spine motion), 3.09 (`ddq`), 1.72 (spine error) |
| G5-DAMP | **FAIL — STOP** | solver held identical: B / A_damp = 1.51, 1.25, 0.96 — B loses 2 of 3 axes |
| G1, G2, G6 | FAIL | B produces **33% more** spurious intervertebral motion than the shipped set |
| G4 limb non-regression | PASS | limbs are fine; the failure is entirely spinal |

Robustness: re-run giving arm B a **2:1 noise advantage** (4 mm vs 8 mm — roughly what good temporal
smoothing would buy). Still STOP.

**Why, structurally:** MHR carries 4 spine anchors and **0 rib anchors** against this model's 54 spine
and 72 rib coordinates. Per-frame pose accuracy cannot buy spinal observability. This is a spatial
result, not a temporal one — hand-written smoothing does not address it, and the experiment fed arm B
an *idealised* marker stream with no model error at all, i.e. SAM 3D Body's ceiling. The ceiling lost.

**Audit caveats, recorded so they are not lost:** leakage exists but every asymmetry favours arm B
(it received 7 real vertebral-level markers MHR cannot produce, and a perfect retarget), so the STOP
is conservative. Arm B's one surviving advantage: with the solver held fixed it is consistently
~4% better on spine error (`E` ratio 0.963, 4/5 seeds < 1.0) — small, real, and far from the 0.7 gate.

### The constraint this puts on product claims

> **Spine coordinates are not measurements. They are priors.**
> **Make no quantitative spine-loading claim.**

This is not a consequence of choosing ARKit. **No arm beat the null model.** The best arm
(`B_damp`, 0.1113 rad) is 5.7× the null (0.0196 rad); `A_damp` is 5.9×. At 8 mm marker noise,
per-intervertebral joint angles are **not recoverable by any marker set**. Shrinking the null space
buys smoothness, not spinal truth.

### Reopening this line

Requires a new pre-registration. The only legitimate technical reason is implementing an
**orientation residual** in the nimble fork — E1 fed arm B marker *positions* only, because rotation
constraints need new C++ residuals and Jacobians that do not exist yet. MHR emits
`pred_global_rots` (127×3×3), so that is the untested upside.
⚠️ Known trap: `pred_global_rots` is **not** in the same frame as `pred_joint_coords`
(`sam3d_body.py:1615-1619` flips the latter but not the former). Correct transform:
`R_cam = diag(1,−1,−1) @ pred_global_rots @ diag(1,−1,−1)`. A naive retarget is silently off by 180°
about X.

---

## SAM 3D Body is integrated, as an offline video/photo path (2026-08-06)

**This is not a reversal of E1.** E1 asked "does swapping the pose source close the spine null
space?" and the answer is still no. This asks a different question — "can the app analyse a recorded
video or photo at all?" — and the answer is yes. The two are orthogonal: E1 gave both arms the same
8 mm marker noise, so it tested marker-set *geometry*, never model accuracy. E1's G4 (limb
non-regression) **passed**; the failure was entirely spinal, and the spine-claim constraint below
applies to both input paths equally.

### What ships

```
video/photo → Vision person box → affine warp 512×512 → crop 512×384
  → SAM3DBodyPose.mlpackage (Core ML, CPU+GPU) → 127 MHR joints
  → MHRRetarget → BodyFrame with the 20 ARKit joint ids
  → the existing NimbleEngine pipeline, unchanged
```

The whole integration is one seam: `NimbleEngine.processFrame` takes a plain `BodyFrame` and has
zero ARKit dependency, so producing that struct makes IK, ID, moment arms, the muscle QP and both
`MuscleOverlay` render passes work with no changes.

### Measured, on this Mac (M2 Pro) — phone numbers are not yet taken

| | |
|---|---|
| mlpackage on disk | 1.31 GiB |
| warm inference | **256.7 ms** (PyTorch MPS baseline was 700.4 ms) |
| cold load | 19.2 s |
| peak process memory | 3.43 GiB |
| CPU-fallback ops | **none** |
| parity vs the released pipeline | 1.72 mm max joint error |

Two engineering choices are worth not re-litigating. Compute precision is **not** blanket fp16: only
`linear`/`matmul`/`conv` are fp16 and every elementwise/reduction/trig/FK op stays fp32, which
measured **3.7× more accurate and 10% faster** than blanket fp16 for +14 MB. And the antialiased
resize in `camera_embed` was replaced by two extracted constant matrices (102 KB) rather than by
area pooling — area pooling measured 2.81 mm of joint error because it diverges at the border, and
the 6-stage decoder feedback amplifies that.

The MHR rig is reimplemented as pure forward kinematics: 8480 bytes, **zero learned parameters**.
The 166M-parameter mesh blob was reduced to the **468 vertices** that the 70 feedback keypoints
actually read (a 39.4× reduction, verified lossless to 0.0008 mm), because 21 of those 70 keypoints
are >50% vertex-driven and dropping the mesh outright would silently change the answer.

### Verified end-to-end

`labs/sam-3d-body/export/e2e_check.py` runs the shipping chain — Core ML → retarget → anatomy — and
is the only check that crosses that seam. 6/6 pass: vertical ordering, segment lengths in adult
range, bilateral symmetry to 1e-7 (this is what proves the L/R indices are not crossed), and the
three `scaleModelWithHeight` factors inside the `[0.7, 1.4]` clamp. Its scale factors
(0.957 / 0.989 / 1.014) independently match the PyTorch-side measurement (0.954 / 0.984 / 1.017).

Left/right assignment was confirmed three ways, all external to the model, because any
model-internal chirality test is circular: COCO-WholeBody keypoint naming, a photo with externally
known facing, and a mirror test (36× residual separation).

### The Savitzky-Golay window is CENTRED — this shapes the offline path (build 18)

`SavitzkyGolayFilter` is a 9-tap centred filter: it emits nothing until 9 samples are in, and what it
then emits is dated at `centerTimestamp`, **4 samples behind the newest push**
(`SavitzkyGolayFilter.swift:9`, `NimbleEngine.swift:318-320`). Two defects followed from that and are
fixed in build 18; both are easy to reintroduce.

1. **Muscle output was filed against the wrong frame.** `OfflineSessionRunner` attached
   `nimble.lastMuscleResult` to the frame it had just submitted, but that result describes a frame
   ~4 earlier. At the 2 fps default that is a **2-second offset** between the pose drawn and the
   muscle overlay drawn on top of it. Fixed by `routeBiomechanicsToOwningFrame()`, which matches on
   `MuscleOutput.timestamp` instead of assuming newest-result-belongs-to-newest-frame. A result with
   no close frame is discarded rather than misfiled.

2. **The first and last 4 real frames could never have muscle**, because they cannot sit at the
   centre of a full window. Fixed by edge-padding: `primeFilterHead` replays the first pose on
   backdated timestamps, `padFilterTail` replays the last pose forward. Head-pad results are centred
   on synthetic timestamps and are deliberately dropped; tail-pad results are centred on real frames
   and are kept. A single photo is the degenerate case — 4 + 1 + 4 = 9, centred exactly on the photo.

⚠️ **Pad on the clip's own cadence.** The filter derives `dt` from the window span
(`SavitzkyGolayFilter.swift:51`), so padding a 2 fps clip with 1/30 s synthetic frames corrupts
`dq`/`ddq` for every window straddling the boundary. `sampleInterval` is taken as the median decoded
frame gap for this reason. Held-pose padding (not reflection or extrapolation) is the deliberate
choice: it drives `dq`/`ddq` toward zero at the edges, which is the only reading this input supports
given the pelvis pinning described below.

### The Savitzky-Golay filters were starving each other (fixed 2026-08-06, build 20)

The per-DOF push loop in `NimbleEngine` broke out on the first filter whose window
was not yet full:

```swift
if let out = self.dofFilters[i].push(...) { ... } else { sgWarmedUp = false; break }
```

`break` meant every later filter got **no sample that frame**. `dofFilters[k]` only started
receiving samples once `0..<k` were already warm, so a full warm-up needed
`9 + 8 × (numDOFs − 1)` ≈ **1350 frames**, not 9.

The live ARKit path hid this completely: at 60 fps it grinds through in ~22 s of continuous
tracking, so muscle output did eventually appear and the delay read as ordinary warm-up
latency. The offline path pushes exactly 9 frames per clip, so it **never warmed up at all** —
an imported photo reported "Pose only (warming up)" and 0 frames with muscle data, permanently.

Fix: push every filter unconditionally, derive `sgWarmedUp` from whether all of them emitted,
and also gate on `smoothedQ.count == numDOFs` since a partial window leaves the arrays short.

**Two regression tests pin it**, and the diagnostic split between them is the point:

- `OfflineMuscleChainTests` walks IK → SG → ID → moment arms → QP stage by stage on real
  Core ML marker output, so a failure names the stage. It **passed** while the app was broken,
  which is what proved the solver chain was innocent and moved the search to orchestration.
- `OfflineOrchestrationTests` drives `NimbleEngine` the way `OfflineSessionRunner` does and
  asserts muscle output after 9 submissions. It **fails against the old loop, passes against
  the new one**.

⚠️ **Method note worth keeping.** Three rounds of this bug were spent reading code and
shipping plausible fixes; none of them was the cause. What found it was writing a test that
reproduced the failure. A visible skeleton says nothing about the solver — the skeleton is
drawn straight from `BodyFrame.joints`, while muscle output requires the whole IK → SG → ID →
moment-arm → QP chain. Those two share no stage.

### Muscle rendering: rank, never threshold (build 22)

A fixed activation cut is wrong on a 520-muscle model, and the measured distribution says why:

| | |
|---|---|
| muscles | 520 |
| clearing the old 0.08 cut | **139** |
| min activation | 0.02 (the `aMin` bound) |
| **median activation** | **0.0200027** |
| max activation | 1.0 (saturated) |

Almost every muscle is either pinned at the floor or railed at the ceiling; there is no spread in
between. Two consequences, both of which showed up on device as "muscles drawn everywhere, and
twitching":

1. **Drawing floor-valued muscles presents a rendering parameter as a measurement.** `aMin`/`epsA`
   were tuned so the visualisation would not go "permanently blue" (recorded above). That value
   carries no information about effort.
2. **A fixed threshold flickers.** With 139 near-identical values sitting at the floor, which ones
   happened to clear 0.08 changed frame to frame. That flicker was the "twitching", not muscle
   activity.

Fix: drop anything not meaningfully above the observed floor, then render only the strongest 24.
Ranking is inherently stable where thresholding a degenerate distribution is not.

Path endpoints were verified to be spatially co-located with the joint positions
(`path_bounds` vs `joint_bounds` in `OfflineOrchestrationTests`), so this was density and flicker,
never misplacement.

~~⚠️ **Still unfixed, and the deeper cause of the saturation:** peak joint torque measures 672 Nm…
This traces back to the pelvis pinning below.~~ **That attribution was WRONG and is fixed — see
[Gravity pointed sideways](#gravity-pointed-sideways-fixed-2026-08-07).** `max_ddq` on that fixture
is 1.7e-16, so there was no dynamic term to attribute anything to; the 672 Nm was pure static
gravity, pulling along the wrong axis.

### Gravity pointed sideways (fixed 2026-08-07)

**Every torque this project ever computed was wrong, because the skeleton's gravity vector ran along
the subject's medio-lateral axis.**

DART's `Skeleton` defaults gravity to `Eigen::Vector3s(0, 0, -9.81)` — a **Z-up** convention
(`nimblephysics/dart/dynamics/detail/SkeletonAspect.hpp:82`). OpenSim models are **Y-up**.
`FullBody.osim:38` even declares `<gravity>0 -9.8066 0</gravity>`, but `OpenSimParser` never reads
that element, and BioMotion never called `setGravity`. Nothing warns.

The measurement that settles it, at `q = 0` with `dq = ddq = 0`: the generalised force at the
coordinate antiparallel to gravity must be exactly `Σmᵢ·g`.

| | before | after |
|---|---|---|
| `τ(pelvis_ty)` | 3.6e-15 N | **780.714 N** |
| `τ(pelvis_tz)` | **780.714 N** | 3.6e-15 N |

With gravity sideways, body **height** became the moment arm instead of the few-centimetre
horizontal offsets that actually load a standing leg — a ~48× inflation at the hip from orientation
alone.

A second, independent defect was found in the same pass. `getMultipleContactInverseDynamicsNearCoP`
takes and returns wrenches in **each contact body's own frame at the body origin** — it builds its
Jacobians with `getJacobian(bodies[i])`, maps guesses to world with `dAdInvT` internally
(`Skeleton.cpp:10205`), and writes results out untransformed with the author's world conversion
commented out (`Skeleton.cpp:10352-10354`). BioMotion passed world-frame guesses and read the
results back as world. Component ordering (`[angular; linear]`) was already right; only the frame
was wrong. Fixed with `math::dAdT` / `math::dAdInvT`, and the hand-rolled CoP projection replaced by
nimble's own `math::projectWrenchToCoP`.

Attribution, measured by reverting the second fix while keeping the first: on a **single**-contact
pose the two are indistinguishable (the wrench is fully determined by the root torque, so the guess
cancels); on **double** support the frame bug moves the net CoP by 11 cm and the subtalar torque by
2.5×.

| | before | after |
|---|---|---|
| dancer ankle torque | 472.5 Nm | **22.5 Nm** |
| standing benchmark ankle | — | **18.55 Nm** (independently hand-derived 18.2) |
| GRF vector | 780 N sideways | **(0, 780.71, 0) N** |
| muscle QP torque residual | 785.3 Nm | **122.8 Nm** |
| muscles pinned at the activation floor | 277 | **193** |

⚠️ **Three things this file and I previously asserted were false. Do not reuse them.**

1. **`rootResidualNorm = 0.0` was not evidence of equilibrium — it was a hard-coded zero.**
   `Skeleton.cpp:10365` unconditionally does `result.jointTorques.head<6>().setZero()`, and the
   assert above it is compiled out (`build_sim/CMakeCache.txt`: Release, `-DNDEBUG`). The field is
   now redefined as a real linear-momentum residual in **newtons**, which is what caught the frame
   bug (2.31 N vs 6e-13 N). It is a frame-consistency check, never a balance check.
2. **The "CoP implies 70 Nm" cross-check was invalid.** The hand-rolled `copFromWrench` divided by
   `force.y()` of a *body-local* wrench while the real load was along z.
3. **`NimbleIKResult.error` is nimble's LOSS, not an RMS**, despite the header comment. The dancer's
   true per-marker RMS is `sqrt(0.013753/20)` = **2.6 cm**, and it never reached the 0.02 m
   convergence bound.

⚠️ **"Torques decrease distally" is NOT a law — do not assert it.** The adversarial pass disproved it
inside a single solve: in single-leg stance the *free* leg decreases distally (hip 37.2 → knee 5.7 →
ankle 0.90) while the *loaded* leg increases toward the contact (hip 29.8 → knee 44.5 → ankle 62.6).
Both are correct: with ground contact the dominant load enters at the foot. The real invariant, now
asserted in `StaticEquilibriumBenchmarkTests`, is the statics identity
`|τⱼ| ≤ |F_GRF|·|p_CoP − cⱼ| + W_distal·L_distal`, which the pre-fix numbers violated 8× at the ankle.

**Still open, and now proven NOT to be an ID-magnitude problem:** the muscle QP still saturates
(14 → 17 muscles at 1.0) with `relativeTorqueResidual` 0.612 against the 0.3 line `MuscleSolver.h`
itself documents. Only 4 of 169 coordinates exceed their musculature's torque capacity, and all four
are wrist DOFs that have *zero* muscles in the model and are asked for < 0.3 Nm. The live lead is a
**unit mix**: `SternumY` is a translational coordinate, so its generalised force is a **force in
newtons** (72.5 N, the largest single entry in both standing poses), and `FullBody.osim` has 6
sternum + 72 rib coordinates like it. `MuscleSolver`'s `‖A·a − τ‖` sums newtons and newton-metres in
one norm. Whether those coordinates belong in the muscle QP at all is the next question.

**E1 re-run and CONFIRMED (2026-08-07).** `E1MarkerSetComparisonTests` calls `getInverseDynamics` on
the shared skeleton, so its archived torque statistics were computed under the wrong gravity and the
STOP verdict was left unconfirmed. It has now been re-run end to end against the corrected gravity,
the 169-coordinate model and the rewritten IK: **`testE1RunAll` passed in 5706.9 s, 0 failures.**
Every pre-registered gate still holds, so the STOP verdict and the spine-claim constraint stand.

That run also needed a partition fix unrelated to E1's physics: the test asserts that eight
hard-coded DOF-name blocks cover every coordinate, and they summed to 163 — correct when the
experiment ran, wrong once the shoulder axis unit-snap turned six glenohumeral coordinates into real
DOFs and the patellofemoral weld removed the two `knee_angle_*_beta`. A `SHOULDER6` block was added
with its own size assertion so a misspelled name fails loudly instead of silently shrinking the cover.

### Muscle QP: what the residual actually is (2026-08-07)

Three premises that drove this investigation were measured and found **false, all in the flattering
direction**. Recorded so they are not rediscovered:

1. **The QP was never running on all 169 coordinates.** It already selected columns with a non-zero
   moment arm and was solving 159 rows. The 10 it drops are the 6 pelvis coordinates (nothing
   outside the body can pull on a floating base) and the 4 wrist coordinates (FullBody.osim has no
   wrist muscles), whose combined demand is under 0.2 Nm. That lever was already at its limit.
2. **The model has six translational coordinates, not 78.** `pelvis_tx/ty/tz` and `SternumX/Y/Z`.
   Every `T*_r*_{X,Y,Z}` rib coordinate is a rotation despite the name, and `knee_angle_*` drives
   coupled translation splines but is itself a rotation, so its generalised force is a moment.
3. **`SternumY`'s 72.7 N was flattering the residual, not inflating it.** It sat in the denominator:
   neutral standing read 0.1245 only because a force was being summed with a 38.46 Nm moment demand;
   on moment rows alone the same solve is 0.2662 — the unit error made the number look 2.1× better.
   It is also not junk. 72.697 N = 9.81 × 7.4104 kg is exactly the shoulder girdle plus both arms,
   whose only load path is the sternocostal joint because the clavicle and scapula are welded.

**What changed:** `torqueResidualNm` (moment rows) and `forceResidualN` (translational rows) are now
reported separately instead of summed into one norm with no nameable dimension. Coordinates the
model declares `<locked>true</locked>` are excluded — nimble does not implement that flag, so IK
moves them and ID hands their demand to muscle. The moment-arm cut went 1e-10 → 1e-6 with no
behavioural change: the per-coordinate maximum is bimodal with a **nine-decade gap**, so any cut in
`[1e-11, 1e-4]` selects the same 159 — structural, not a tuned threshold.

| pose | relative torque residual, 2026-08-07 | as of 2026-08-06 |
|---|---|---|
| neutral standing | **0.2008** | 0.2008 |
| 4° forward lean | **0.1526** | 0.1526 |
| dancer fixture | **0.3545** | 0.6406 |

Standing is under the 0.3 line `MuscleSolver.h` documents; the two standing figures did not move
because their IK was already fitting to 0.1 mm.

**The dancer's 0.6406 → 0.3545 is the IK fix, not a muscle-side change.** The 2026-08-06 text said
"no row cut moves it below ~0.60 — because that pose is wrong before the muscle solver sees it", and
that diagnosis was right: the same fixture now solves to 2.1224 cm true marker RMS instead of
5.4913, reproducibly, and the muscle residual almost halved with the muscle solver untouched. Nothing
in `MuscleSolver` or `MomentArmComputer` changed between those two columns.

⚠️ The dancer still does not clear 0.3, and the sentence **"the dancer fixture is not a usable
benchmark for the muscle stage until IK is fixed"** is only half retired. IK is fixed as a solver —
it is a fixed point, order-independent, and lands on the same answer every run — but the fixture's
remaining 2.12 cm is a marker DEFINITION error (PELVIS is registered at the `pelvis` body origin,
≈ 9.7 cm from the pose source's mid-hip point; per-marker PELVIS 5.76 cm, LSJC 3.73, RHJC 3.47).
Until that is resolved the pose handed to ID is still not the subject's pose, and 0.3545 is a
faithful report of that. The two standing poses remain the muscle stage's only clean benchmarks.

⚠️ **Saturated-muscle count is not a valid metric — stop using it.** Across a λ sweep at *fixed*
inputs it reads 19, 11, 22, 18, 20 with no trend, and at-floor reads 219, 189, 344, 170, 282. OSQP
runs `max_iter=200` at `eps 1e-3` on a badly conditioned P, so which vertex of a degenerate face it
lands on is not stable. Counting saturated muscles measures the solver's stopping point.

The residual that remains is **not** an objective-weighting artefact: sweeping λ from 1 to 1e8 moves
it by at most 11%, and *upward*. In quiet standing ~87% of it is the `aMin = 0.02` activation floor
projected onto costovertebral rows whose musculature is too one-sided to cancel it. `aMin` was
deliberately not lowered — STATUS records it as chosen so the visualisation would not go
"permanently blue", and moving a rendering parameter to improve a physics number is exactly the move
to avoid.

### Model delivery: the app binary is 8 MB (2026-08-07)

The 1.3 GiB Core ML model no longer ships inside the app. It is delivered as an **Apple-Hosted
Managed Background Asset** (iOS 26; ODR is deprecated as of iOS 27), carrying a **pre-compiled
`.mlmodelc`** — byte-diffed against the artifact Xcode was previously embedding, with only the
503-byte root `coremldata.bin` differing, and only in metadata key order.

App bundle **1.3151 GiB → 0.0069 GiB**; archived payload 8 MB. Device-side `compileModel` was
rejected because it would need the package and its output resident simultaneously (~2.6 GiB of user
disk), in the app container where iOS cannot evict it, and would stall the first import.

Resolution ladder, first hit wins: bundled `.mlmodelc` → bundled `.mlpackage` (compile + cache) →
pack `.mlmodelc` → pack `.mlpackage` (compile + cache) → start the download and **throw immediately**.
It never blocks on the transfer. The bundled branch exists so the Simulator and local iteration need
no download (`tools/assetpack/dev_bundle_model.sh on|off`).

⚠️ **The shipping load path has never been executed.** The Simulator is served no pack and the dev
bundle is empty by default, so nothing in this checkout reaches `MLModel(contentsOf:)`.
`AssetPackManager.url(for:)` against a *directory* entry is unproven. Packaging and upload are
verified (`.aar` = 1.0210 GiB); the download-and-load is not. It can only be confirmed by a
TestFlight install on a device. Build 23 remains installable with the model bundled, so a failure
here has a fallback.

Packaging and upload: `tools/assetpack/README.md`. The pack uploads **separately from the app** via
`xcrun altool --upload-asset-pack` and **requires an ASC API key** — an app-specific password
returns 401.

### Static-hold gating (2026-08-07)

The pose source zeroes global translation, so `M·q̈` and centre-of-mass acceleration would be
computed from motion that did not happen. Frames are now classified as hold or moving from
**measured marker speed**, and only holds are solved — as statics.

The criterion is deliberately *not* built on the SG-filtered `q̈`: that quantity is derived from the
very data whose global component is missing, so gating on it would be circular. Moving frames keep
pose, skeleton and `.mot` export and lose only the muscle magnitudes; the badge shows which, with the
measured number behind it.

⚠️ **Honest note on the framing:** the claim that the 0.5 s duration was *derived* from the two
constants rather than being a third knob was circular. 0.5 s pre-exists in next-step 5, and the one
genuinely new constant (0.08 m/s²) was exactly `2 × 0.02 / 0.5` — chosen to reproduce it. The gate
itself was not fitted to any fixture (a 0.9×–1.1× sweep flipped exactly at 0.020), but "0.82% of g"
was a post-hoc description, not a budget set first.

**Both constants were replaced 2026-08-07, and the noise floor is now measured rather than assumed.**
Thresholds are `0.20 m/s` (where `v²/(g·r)` reaches 1% on a 0.4 m segment) and `0.05·g = 0.4905 m/s²`
(an order of magnitude below the muscle QP's own 0.20-0.35 relative torque residual), implied minimum
window span `2v/a` = 0.8155 s. At the 2 fps offline cadence the admissible peak marker speed goes
2 cm/s → 20 cm/s. The measured pose noise floor is 4.69 cm/s at 30 fps — **above the old 2 cm/s cap**,
so the old gate could not have returned a hold on real 10-30 fps footage even for a motionless
subject. Full derivation, the measurement, and the new `MotionVerdict` reasons:
[cam_t recovers the root translation](#cam_t-recovers-the-root-translation-its-depth-cannot-be-differentiated-twice-2026-08-07).

### Device vs Mac: SOLVED — Vision was returning an upper-body box (2026-08-07)

**Root cause: `VNDetectHumanRectanglesRequest.upperBodyOnly` defaults to `true`.**
`SAM3DPoseEstimator.detectPersonBBox` constructed the request bare and never set it, so the offline
path cropped SAM 3D Body's 512×512 input to the **torso**. The legs were outside the square the
model could see, so it emitted a near-standing mean pose for them while the torso kept tracking.
Fixed by `makePersonRectangleRequest()`, which sets the flag and exists so a test can assert it
(`BioMotionTests/PersonBoxTests.swift`).

Measured on `video_015.mov` (576×768 running clip), same model, same frames, only the box differing.
Error is mean joint distance as a percentage of the subject's own pixel height, refereed by
**Vision's own 2-D body pose** — an estimator sharing no code with SAM 3D Body, so neither crop
judges itself.

| box | height as % of image | leg error | torso error |
|---|---|---|---|
| `upperBodyOnly = true` (was) | 20–26%, ends above the hips | **9.0%** | 2.0% |
| `upperBodyOnly = false` (fix) | 46–60%, reaches the feet | **4.6%** | 1.9% |
| whole-image fallback | 100% | 4.7% | 2.0% |

20 frames spanning the clip. A sparser 9-frame sample taken first read 9.2% → 3.0%; the denser
sample is the better-powered number and both are recorded rather than the flattering one. Every
frame improved in both samples, and the torso did not move in either. That "legs transform, torso
unchanged" signature is what separates the fix from a number that drifted. Harness:
`labs/sam-3d-body/export/{vision_box_probe.swift,box_ablation.py}`.

**The whole-image fallback needs no work** — it scores 4.7% against the real box's 4.6%, so the
path taken when Vision finds nobody is not a quality cliff and does not need a
carry-the-previous-frame's-box mechanism. Measured specifically to avoid building that.

**Asking for the whole body does not cost detection rate** — the obvious way this fix could have
backfired, since a whole-body box is the harder detection and a miss falls back to the whole image.
Over 60 frames from all three reference clips the two settings detect **identically**: 18/20, 20/20,
20/20. Box height as a fraction of image height, upper versus full: 19%/46%, 21%/47%, 24%/55%.

**Why this took so long, recorded so the same trap is cheaper next time.** The Mac reproduction was
fed a full-body box (`96,214,367,667` — 59% of image height) while the phone detected its own
torso box. Every comparison therefore differed in the one input nobody was printing. That produced
three confident wrong turns:

- **"The two Core ML backends diverge."** Wrong. It explained the facts — including why the Mac was
  invariant to twelve input perturbations — because the Mac never had the defect. Backend
  fingerprints (builds 27–28) were built to test a hypothesis that was false.
- **"The inputs differ, so the divergence is upstream."** Right conclusion, worthless evidence: the
  Mac reference pixels were extracted from a *screen recording* — re-encoded, rescaled, with the
  overlay burnt in. Those checksums could never have matched whatever the cause.
- **A visible, well-formed skeleton on the torso read as "mostly working."** It was the diagnostic
  signal: a monocular model that cannot see a limb does not fail loudly, it returns the mean pose.

The lesson that generalises: when two environments disagree, print **every** input each one derived
for itself before comparing anything downstream. The framework default nobody wrote down is a
better suspect than the numerics nobody can see.

**Ruled out earlier, and still true — this was all sound work on the wrong stage:**

| checked | result |
|---|---|
| video rotation / photo EXIF | both handled (`appliesPreferredTrackTransform`, `cgOrientation` to Vision) |
| projection, crop geometry, intrinsics, retarget indices | **1.0 px mean vs the model's own `keypoints_2d`; 0.0 px at the shoulders** |
| which frame the app displays | image correlation → the frame tested here, unambiguously |
| sampling timestamps | app and reproduction both `t = 0, 0.5, 1.0, …` |
| bone connectivity table | identical |
| vs an independent estimator (Vision body pose) | agree to ~13 px on a body ~450 px tall (≈3%) |

**The Mac's prediction was invariant to twelve perturbations**: six person-box variants including
the whole-image fallback, ±1 and ±2 LSB pixel noise, and an RGB/BGR channel swap. Read correctly,
this was the clue rather than the mystery — all six box variants were generous ones, so the model
could see the legs in every case and nothing moved. A torso box was never among them.

The FNV-1a fingerprints from builds 27–28 remain in the code and stay useful, now as a
backend-parity check rather than a bug hunt. Reference on this Mac (M2 Pro, `CPU_AND_GPU`):

```
self-test        in 0a6dd7e25b3013e8   out 24f80e92616dbaf7
video_015 t=26.5s in bd7f952e8e69ed34   out 79cba1631fb7ccd7   (Vision box 96,214,367,667)
```

⚠️ **ANE cannot help here, and is measured to be worse.** With `ComputeUnit.all`, **33 of 12,198
ops** go to the Neural Engine — 32 `linear`, nothing else — and warm inference goes **256.5 ms →
330.0 ms** (+29%) with cold load 17.0 s → 25.0 s. The model is dominated by elementwise and control
ops (2898 `mul`, 1809 `slice_by_index`, 1441 `cast`, 566 `logical_and`, 162 `select`, plus
`atan`/`sin`/`cos`), which is the MHR forward kinematics and the branchless rotation maths — not
work ANE is built for. The `grid_sample` feedback loop stays on GPU either way, so ANE would add a
*third* numeric implementation rather than remove a divergence.

**"19 frames produced no pose" was a misreading and is closed.** The 57 was a partial count read off
a recording while the batch was still running. Build 28 reports **76 pose-only of 76 sampled** on
the same clip.

### The limitation that shapes the product claim

**`joint_coords` pins the pelvis at a model constant `(0, 0.924, 0)` in every frame.** `global_trans`
is zeroed (`sam3d_body.py:1600`), so that number is not the subject's pelvis height and `y = 0` is
not the floor.

Joint *angles* are frame-invariant and therefore correct. But the body has no global vertical
motion, so **dynamic** inverse dynamics is not sound on this path — in a squat the pelvis does not
descend, the feet appear to rise instead.

⚠️ **The last sentence of this section used to read "recovering true global motion from monocular
video needs camera-pose/SLAM, which is a different project." That was wrong, and it is the reason
the offline path spent a build refusing every moving frame.** The model emits the root translation
separately, as `cam_t`, which this app already exports, already stores on `FrameResult` and already
uses to project the overlay. Measured and corrected 2026-08-07 — see
[cam_t recovers the root translation](#cam_t-recovers-the-root-translation-its-depth-cannot-be-differentiated-twice-2026-08-07).
Static-equilibrium ID over a detected hold remains the honest reading, but for a *narrower* reason
than "the translation is unavailable".

T-pose calibration **is** skippable, but only via `segmentScaleMarkers`, which rebuilds a synthetic
straight-limb marker set from pose-invariant chain sums. Handing the bridge raw posed markers fails
the `[0.7, 1.4]` clamp on 6 of 6 test predictions (a seated yoga pose gives lower 0.351).
The method cannot rescue a bad prediction: a small, heavily occluded subject produced a degenerate
0.070 m hip width. ~~A plausibility gate on hip width and stature is recommended and **not yet
built**.~~ **Built 2026-08-07** — see [Body-size gate](#body-size-gate-2026-08-07).

### cam_t recovers the root translation; its depth cannot be differentiated twice (2026-08-07)

Owner objection that started this: *"why muscle loads need a still pose? dynamic pose also needs the
muscle."* Correct, and the gate was standing on a false premise. Gates were pre-registered before any
result existed (`/tmp/camt/PREREGISTRATION.md`, reproduced below), harness
`labs/sam-3d-body/export/camt_probe.py` + a Swift decoder that mirrors `FrameSource.VideoDecoder`
and `SAM3DPoseEstimator.detectPersonBBox` so the person boxes are the ones the app would get.
309 frames of `video_012.mov` at native 30 fps through the shipping Core ML model.

**Finding 1 — `cam_t` IS the missing quantity, and the pinning is a bug in what we consume.**
It is the CLIFF/CameraHMR full-frame root translation
(`sam_3d_body/models/heads/camera_head.py:84-96`): `tz = 2f/(bbox_side·s)`,
`cx = 2(bbox_cx − w/2)/bs`, `cam_t = [tx+cx, ty+cy, tz]`. Measured on the 287 Vision-detected frames:
depth **−4.34 m**, **1.10 m** below the optical axis, ±0.37 m lateral — all physically right — and
`corr(1/bbox_side, depth) = +0.74`, exactly what that formula requires. In the Y-up frame the app
uses, `T = (cam_t.x, −cam_t.y, −cam_t.z)`.

**Finding 2 — necessary, not sufficient: the depth channel is noise.** `Tz`'s high-frequency residual
about its own 0.5 s mean is **12.7 cm std / 68 cm max**. Through this project's own 9-tap centred
Savitzky-Golay filter (`‖C₂‖ = 0.1140`, so `a_noise = σ·0.114/dt²`) that is:

| sampling | root accel noise, depth | as a fraction of g | 9-tap window span |
|---|---|---|---|
| 30 fps | 30.5 m/s² | **3.11 g** | 0.27 s |
| 15 fps | 5.46 | 0.56 g | 0.53 s |
| 10 fps | 2.27 | 0.23 g | 0.80 s |
| 6 fps | 1.38 | 0.14 g | 1.33 s |
| 2 fps | 0.21 | 0.02 g | 4.00 s |

The in-plane channels are 8-25× cleaner (`ax` 0.26 / `ay` 0.28 m/s² at 10 fps against `az` 2.27;
1% person-box jitter moves `cam_t` by x 2.2 mm / y 2.6 mm / **z 18.9 mm**).

**Finding 3 — the depth noise is intrinsic, not preprocessing.** Temporally smoothing the person box
cut the box's own high-frequency wobble **12×** (9.78 px → 0.82 px, 4.37% → 0.37% of the side) and
the depth residual only **28%** (6.75 → 4.83 cm). Re-running everything with full-body boxes after
`327ca89` (`upperBodyOnly = false`) changed it not at all. It is the model's monocular
depth-from-apparent-size, the classically ill-conditioned direction.

**Finding 4 — the sampling-rate trade-off is a scissors, and for fast motion it closes.** The 9-tap
window spans `8·dt`, so resolving a motion of period `T` wants `dt ≲ T/16`, while the noise grows as
`1/dt²`. A running stride (T ≈ 0.7 s) wants ~22 fps, where the depth noise is ~1.8 g against a true
peak CoM acceleration of 2-3 g: SNR ≈ 1. **Sampling faster makes this worse, not better**, as long as
the filter is a fixed 9 taps. Cost, for completeness: a 10 s clip at 30 fps is 300 model calls at
0.7-1 s each, and `FrameSource.maxFramesPerRun = 120` already caps a 10 s clip at 12 fps.

**Finding 5 — the camera moves, and that is measurable.** Background phase correlation outside a
dilated person box, chained at native rate, displacement over one 0.27 s filter window:

| clip | drift over the whole segment | rotation | spurious displacement at 1 m |
|---|---|---|---|
| `video_012` | **1.79 image diagonals / 10.2 s** | 13.5 °/s | **6.4 cm** per filter window |
| `video_015` | 0.08 diag / 6.6 s | 0.4-0.6 °/s | 0.2-0.3 cm |
| `video_013` | 0.03 diag / 6.6 s | 0.2 °/s | 0.1 cm |

A 20-60× separation, so a threshold fits between them. ⚠️ **It must be measured at the video's NATIVE
rate**: at a 10 fps analysis sampling the same estimator aliased `video_012`'s pan down to ~0, which
is the estimator's periodic ambiguity, not stillness. ⚠️ It detects camera ROTATION well and camera
TRANSLATION poorly (a distant background dominates the correlation and does not move under
translation). Physically that ordering is right — rotation tilts gravity and adds centripetal terms,
while constant-velocity translation is Galilean and harmless — but camera *acceleration* is not
measured and is the residual risk.

**Finding 6 — Vision loses the person on 7% of frames** (22/309 at 30 fps) and falls back to the
whole image, which makes `cam_t` wild (whole-clip depth range 4.47 m vs 2.57 m on detected frames)
and the body scale wrong (hip-width range 112 mm vs 26 mm). Those frames must never enter a
derivative.

**Finding 7 — the old gate's constants were below the instrument's own resolution.** Measured
per-frame drift of a rigid distance (hip width, 284 consecutive detected frames): median **3.13 mm**,
p90 6.83, max 12.07. As a noise floor that is **4.69 cm/s at 30 fps, 4.12 at 10 fps, 0.59 at 2 fps** —
so against the old **2 cm/s** cap the instrument's own noise exceeded the threshold at every rate
above ~3 fps and a perfectly still subject could not have been classified still. That is what made
the old gate self-defeating: getting under the noise floor required sampling slowly, and sampling
slowly stretched the 9-tap window's stillness requirement to **four seconds**.

#### Verdict against the pre-registered gates

| gate | pre-registered bound | result |
|---|---|---|
| A — is `cam_t` a real root translation | physically plausible depth/height/lateral, correct bbox coupling | **PASS** |
| B — root accel noise ≤ 0.98 m/s² (10% of g) | ≤ 0.98 pass, 0.98-2.94 marginal, > 2.94 fail | **FAIL at every rate that resolves motion** (30.5 at 30 fps, 2.27 at 10 fps) |
| C — camera motion separable by ≥ 3× | ≥ 3× | **PASS, 20-60×** |
| D — sampling rate | state the trade-off with numbers | see Finding 4: the scissors closes for stride-speed motion |

**So: dynamic muscle output is not achievable on the owner's actual footage, and the reason is
upstream of the sampling rate.** All three clips are tracking shots — the subject's range stays at
4.34 ± 0.66 m over 10 s while a runner would traverse ~50 m — so the camera translates with the
subject and `video_012` additionally rotates at 13.5 °/s. No filter, rate or budget recovers an
inertial frame from that. The largest honest subset is: **static camera + in-plane motion + a rate
that resolves it**, which is squat / lunge / sit-to-stand filmed side-on from a stand, not running
filmed from a moving camera.

#### What shipped, and what is blocked

Shipped (files this change owned):
* `MHRRetarget.makeBodyFrame(jointCoords:camT:…)` composes the root translation back in. `camT`
  defaults to nil = bit-identical to the previous behaviour. Proven a pure rigid translation
  (max pairwise distance change 2.4e-7 m) and tied to the already-validated projection: a composed
  marker projected through a zero `cam_t` lands on the **same pixel, 0.0 px gap**, as the pinned
  marker through the real `cam_t`.
* The gate's constants are now derived from a stated error budget instead of being knobs:
  `0.02 m/s → 0.20 m/s` (where `v²/(g·r)` reaches 1% on a 0.4 m segment) and
  `0.08 m/s² → 0.05·g = 0.4905 m/s²` (an order of magnitude below the muscle QP's own 0.20-0.35
  relative torque residual — a term held to 0.82% cannot improve an answer whose dominant error is
  20-35%, it can only refuse frames). Net effect at the 2 fps offline cadence: the admissible peak
  marker speed goes **2 cm/s → 20 cm/s**, exactly 10×.
* `MotionVerdict` replaces the boolean: `.hold`, `.movingBeyondStaticBudget`,
  `.indistinguishableFromNoise`, `.noMeasurement`, each with a sentence the user can act on. The
  third case is new and is Finding 7 made operational — the pose noise floor is now **measured per
  clip** from `rigidPairs`, distances that physically cannot change, as `median|Δd|/(2·dt)`
  (a rigorous lower bound, since a distance change of `d` needs ≥ `d/2` on one marker).
* `MotionClassification.rootTranslationObservable` reads the DATA, not a flag: a pinned stream
  repeats the model constant bit-for-bit, so the engine cannot disagree with the stream about
  whether it was handed a root translation.
* Tests: 197 → **219, all passing**, including a new `RootTranslationTests` (6) and 5 new
  noise-floor / budget tests.

⚠️ **Blocked, and deliberately not worked around.** Two seams live in files another task owns:

1. `OfflineSessionRunner.swift:242` calls `MHRRetarget.makeBodyFrame(jointCoords:timestamp:frameNumber:)`
   and has `estimate.camT` in hand one line above. Adding `camT: estimate.camT` is the entire
   activation. **Until that line changes, `rootTranslationObservable` is false on every real frame
   and the composition is exercised only by tests.**
2. The user-facing sentence comes from `OfflineResultStore.MotionState` (two cases: `.hold`,
   `.moving`) rendered by `OfflinePlaybackView.motionDetail`, which hard-codes *"muscle loads need a
   still pose"*. `MotionVerdict` carries the reason and the advice; surfacing it needs those two
   files.

⚠️ **No dynamic-ID branch was shipped**, and that is a decision, not an omission. It cannot be
reached by the app without seam 1, and its correctness turns on an unresolved design question — see
the owner decision below. Shipping an unreachable branch whose depth handling has never been
validated end-to-end is exactly the "silent wrong number" this file warns about.

#### SUPERSEDED 2026-08-07 — the question was wrongly framed

Both options below assume the root's acceleration is something we must MEASURE. For running it is
not: the gait cycle supplies it. See
[Gait-cycle dynamics](#gait-cycle-dynamics-model-the-forces-instead-of-differentiating-the-measurement-2026-08-07),
which measures the required signals on the user's own clips and passes. The depth-hold gate below
was not built. The (a)/(b) framing is kept because the reasoning inside it is still correct for
NON-periodic motion, where there is no gait cycle to close the system.

#### Owner decision this raised — DECIDED 2026-08-07: (b), as a checked precondition

Given Finding 2 — in-plane root motion is measurable, depth is not — there were two defensible
products:

* **(a) Refuse.** If the measured depth acceleration exceeds the budget, withhold. Honest, but on
  real footage it withholds at every usable rate, so dynamics never ships.
* **(b) Declare depth constant.** Treat the root's depth as unobservable above some bandwidth, run
  dynamics on the in-plane channels, and tell the user the assumption plus the measured slow depth
  drift so they can check it.

**Decision: (b), with the assumption implemented as a gate rather than a disclaimer.** Hold the
root's depth at its low-passed value, let in-plane root motion through, and refuse the frame when
the low-passed depth drift itself exceeds a pre-registered budget. The drift is already computed, so
this adds no mechanism — it converts a modelling assumption into a precondition the data has to
satisfy, and a subject walking toward the camera gets refused with the reason.

Three reasons (a) was rejected:

1. It is not the conservative option, it is abandoning the requirement. The product is posture **and
   muscle-force** analysis; an option under which dynamic muscle output never exists fails outright.
2. For the case (b) serves — in-place exercise filmed from a stand — the subject's depth is
   approximately constant **as physical fact**, not convenience. The assumption is nearly true for
   exactly the footage it applies to, which is a different thing from assuming it because the
   measurement is inconvenient.
3. Sagittal-plane 2-D analysis is a recognised biomechanics method. Naming it as such is honest.

**A gap found while reviewing, still open at the time of writing:** `poseNoiseFloorMetersPerSecond`
is derived from `rigidPairs`, i.e. inter-joint distances — which are **invariant to root
translation**, since both endpoints shift together. So the floor measures articulation noise and is
structurally blind to `cam_t` depth jitter. Once `camT` is composed in, ~12.7 cm of depth residual
at the 0.5 s offline cadence is ~0.36 m/s of root-speed noise, above the 0.20 m/s hold threshold,
while the rigid-pair floor stays put — a motionless subject would be told to hold still. The root
channel needs its own floor before the `camT` line in `OfflineSessionRunner` is wired.

---

### Gait-cycle dynamics: model the forces instead of differentiating the measurement (2026-08-07)

The "muscle force is unobtainable on a tracking shot" conclusion was WRONG, and it was wrong at the
framing, not at any of its measurements. Every number in the section above still holds. What does
not hold is the assumption underneath them: that the root's acceleration has to be **measured**.

**Where the chain broke.** We had: muscle needs ID → ID needs q̈ → root q̈ needs world root
translation → `cam_t` depth is too noisy to differentiate twice → no dynamics. Two facts break it:

1. **Joint angles are invariant to camera motion.** Camera translation shifts every reconstructed
   point together, so inter-joint vectors are unchanged; camera rotation preserves lengths and
   included angles. The 163 actuated DOFs are untouched by a tracking shot. Only the 6 root DOFs are
   affected — exactly the channel we were trying to differentiate.
2. **Root acceleration enters as a uniform pseudo-gravity.** Every segment picks up `m·a_root`,
   indistinguishable from changing g. So the question is not whether we can differentiate the root
   position, but whether we know `a_root` by another route.

For running we do, without differentiating anything. In flight `a_root = g` exactly — physics gives
it. In stance `m·a_CoM = F_GRF − m·g`, so GRF and CoM acceleration are ONE unknown; the gait cycle
closes it, because CoM velocity is periodic over a stride and the whole stride's vertical impulse
`m·g·T` must be delivered during stance alone. Assuming a half-sine stance force, that is exactly
`Fmax = m·g·(π/2)(1 + tf/tc)` (Morin et al. 2005) — impulse-momentum plus one shape assumption, not
a black box.

So the only thing that has to be read off the video is **contact and flight timing**.

**Measured on the user's own clips**, 4 s windows at native 30 fps (120 frames — already inside
`FrameSource.maxFramesPerRun`, so the compute budget is unchanged), using image-plane
pelvis-relative ankle height only, never `cam_t` depth. Harness:
`labs/sam-3d-body/export/{gait_events.py,gait_summary.py}`.

| clip | contact | stride | cadence | flight/step | peak vertical GRF |
|---|---|---|---|---|---|
| video_012 | 200 ms | 0.600 s | 200 spm | 100 ms | **2.36 BW** |
| video_013 | 167 ms | ~0.60 s | 229 spm | 95 ms | **2.47 BW** |
| video_015 | 244 ms | 0.64 s | 187 spm | 77 ms | **2.07 BW** |

- **The detection is not tuned.** On `video_012`, contact time was IDENTICAL (200 ms, zero spread,
  13/13 contacts exactly 6 frames) for every threshold from 0.08 to 0.20 of the ankle's vertical
  range — a 2.5× span — and only smeared at 0.25 where the threshold reaches into swing.
- **It is not a pipeline artefact.** The three clips give three DIFFERENT signatures, ordered as
  physics requires: `video_015` has the longest contact and lowest cadence and therefore the lowest
  force; `video_013` the shortest contact and the highest.
- Duty factor 0.33 (contact / stride) with a real flight phase — a genuine running gait.

**Honest precision.** One frame at 30 fps is 33 ms, i.e. 17% of a 200 ms contact, so peak GRF is
good to roughly **±17%** (`video_012`: 2.02–2.83 BW at ±1 frame). The muscle QP already carries a
0.20–0.35 relative torque residual, so this does not make the chain categorically worse.

**Why q̈ noise matters less than feared.** Knee angular acceleration splits sharply by phase
(fc = 6 Hz): stance mean 3,630–5,180 °/s² against swing mean 7,042–7,517 °/s². For an illustrative
70 kg runner the shank+foot inertial knee moment during stance is **15–21% of the GRF moment**
(peak 36–43%), so a 30% error in q̈ reaches the joint moment as roughly 6%. Stance-phase moments are
GRF-dominated, which is the regime this route is good at.

**What is genuinely unresolved.** The knee-angle spectrum has NO clean signal/noise separation — a
Winter residual analysis declines steadily from 2 to 14 Hz with no knee, and peak angular
acceleration reads 14,000 °/s² at fc = 6 and 25,000 at fc = 8. 4.5–4.7% of the power sits above
10 Hz, which the ω² weighting makes dominant in a raw second derivative (SNR 0.2–0.3 unfiltered).
The cutoff is therefore a judgement call, and this must not be quietly resolved in the flattering
direction. Also not yet addressed: horizontal GRF, CoP location, and non-steady running (the
periodicity argument needs a steady stride; walking has no flight phase, so the free-fall
calibration does not apply though the impulse argument survives).

**Nothing here is implemented in the app.** This is Mac-side validation of the route. What it needs:
a 4 s @ 30 fps sampling window in place of the whole clip at ~2 fps, contact detection, GRF from
timing feeding the EXISTING near-CoP ID path, and `a_root` taken from the gait model instead of from
differentiating `cam_t`.


## IK convergence: the solver is now a fixed point (2026-08-07)

App-side only. `NimbleBridge.mm` no longer calls `Skeleton::fitMarkersToWorldPositions` /
`math::solveIK` / `math::refineIK`; it runs its own bound-projected Levenberg-Marquardt at the call
site. **The vendored nimble tree is byte-identical** (`git status nimblephysics/ osqp/` = 0 lines).

Six distinct defects, each with its own mechanism:

| | Defect | Fix |
|---|---|---|
| 1 | `refineIK` stops on error-CHANGE, never on stationarity, then the next call resets `lr` to 1.0 and resumes | bound-projected-gradient test + step-norm test. The pose returned IS a stationary point, so the next call on the same markers passes its FIRST test having moved nothing — the fixed point is a property of the termination rule, not of a tolerance |
| 2 | `leastSquaresDamping` is a fixed 0.01 | trust-region λ adapted from the observed decrease, expressed as a multiple of `max(diag(JᵀJ))` so it is scale-free, plus a `1e-6·max(diag)` conditioning floor. `JᵀJ` is 169×169 with rank ≤ 60; below that floor, double-precision round-off in the gradient is amplified into ~1e-7 rad of null-space noise per step. At a 1e-9 floor the dancer burned all 120 iterations of BOTH phases and still reported converged=NO |
| 3 | `clampPositionsToLimits` inside the objective means a coordinate on its limit keeps generating steps the clamp undoes, so the unprojected gradient never reaches zero | active set — a coordinate at a bound whose gradient pushes it further out leaves both the step and the convergence test. This is what turned the dancer from converged=NO/240 iterations into converged=YES/177 |
| 4 | no null-space damping toward the seed | two phases. A: `‖W(f(q)−x*)‖²/2 + μ‖q−q_seed‖²/2`, μ = 1e-3 — chooses WHICH of the equally-good poses comes back. B: re-run with μ = 0 — drives `Jᵀr` to 0 so a fixed point exists at all. B's steps lie in the row space of `J`, so they cannot undo A where `J` has a zero column. Verified: all 72 coordinates in `FullBodyDOFFixture.structurallyUnreachableCoordinates` return at EXACTLY their seed value, across two poses |
| 5 | random restarts drew from process-global `std::rand()`, and `fitMarkersToWorldPositions` seeded from the skeleton's CURRENT positions (`Skeleton.cpp:8001`) — which the shared skeleton meant was whatever ID or `MomentArmComputer` last wrote | restarts removed; the cold seed is an explicit `neutralSeedPose` |
| 6 | nimble scaled the residual by the marker reliability weights but NOT the Jacobian (`Skeleton.cpp:7979-7986`), so its step minimised `‖J·d − W·r‖` — a descent direction for no objective it was measuring | both are scaled |

Measured before/after, same fixture, same machine:

| | before | after |
|---|---|---|
| dancer marker RMS (true, unweighted) | 5.4913 cm (max 19.46) | **2.1224 cm** (max 5.76), converged=YES in 177 iters |
| dancer, on the old weighted `sqrt(loss/N)` convention | 2.6224 cm | 2.3951 cm |
| dancer drift over 8 identical warm solves | 0.117–0.267 rad, no decay | **exactly 0.0** for 7 consecutive |
| order dependence (same markers, unequal preceding work) | 1.689 rad apart, RMS 5.49 vs 42.17 cm | **exactly 0.0**, identical `‖q‖` 5.746453 |
| continuity under a 2 mm rigid marker shift | 7.43e-2 rad | exactly 2.000e-3 — a pure root translation, the correct answer |
| standing benchmark poses | 0.10–0.14 mm | **0.032 mm** |
| Rajagopal2016 planar, 400 solves | ~6e-3 rad/solve, never stopping | 2.69e-9 rad cumulative, 0.0 per solve |
| dancer muscle relative torque residual | 0.6406 | **0.3545** |

The unweighted RMS improved 2.6× because the old solver parked its error on the down-weighted
markers.

⚠️ **Cost regression on MOVING input, measured and not resolved.** A warm solve on identical
markers went 409.6 → 49.0 ms (it exits on its first convergence test), but a moving subject
(~6 mm/frame) costs **1567 ms/frame at 77.8 iterations** versus the old solver's ~410 ms — the old
one ran a fixed budget and never terminated on convergence, which was the defect. Caveats: Debug
simulator build, where the 169×169 `JᵀJ` and its LDLT are compiled at `-O0` while nimble's
equivalent sits in a Release static library, so the ratio is pessimistic for the new code. The
obvious hypothesis was tested and is FALSE: relaxing `kIKStepTolerance` 100× moved iterations by 6%
and the RMS not at all, so the iterations are real convergence work. The untried lever: the normal
equations are 169×169 while the residual is only 60 rows, so a Woodbury/dual form would cut the
per-iteration cubic term by ~8×. **On-device Release timing has not been measured.**

⚠️ **Seed sensitivity is real and is not a bug.** The dancer solved from the neutral seed and from a
solved standing pose lands 2.32 rad away at essentially the same fit quality (loss 0.011473 vs
0.011638, RMS 2.1224 vs 2.1526 cm). The problem is non-convex and rank-deficient, so the warm start
is a genuine input. Determinism holds per session state; it is not basin-uniqueness.

⚠️ **The 2.12 cm that remains is a marker DEFINITION error, not solver error.** Per-marker: PELVIS
5.76, LSJC 3.73, RHJC 3.47, RSJC 3.27, LHJC 2.70 cm; everything else ≤ 2.02. The model's rigid
PELVIS-to-hip distance is 0.1237 m and the fixture's is 0.0792 m (4.46 cm on each side) while the
hip WIDTH agrees to 0.8 mm — that pattern is placement, not body scale. The pose source's
`hips_joint` is the MID-HIP point; `NimbleBridge.mm` registers PELVIS at the OpenSim `pelvis` body
ORIGIN, ≈ 9.7 cm away. Dropping PELVIS takes the fit to 1.5541 cm over the remaining 19. The marker
table was deliberately NOT changed: it would silently redefine the marker for the live ARKit path
too and invalidate the hand-built fixtures in `StaticEquilibriumBenchmarkTests`.

---

## Posture findings: a kinematics-only layer (2026-08-07)

`BioMotion/Findings/` computes nine measurements from `BodyFrame.joints` alone — forward head,
rounded shoulders, shoulder-height asymmetry, lateral head tilt, sagittal and lateral trunk lean,
a kyphosis proxy, transverse trunk rotation, lateral weight shift. It reads **no** `ikResult`,
`idResult` or `muscleResult`, so it is independent of every open defect downstream of the pose
model, and it is what the product can defend today with zero licence exposure.

**No clinical or normal range is applied anywhere.** No finding carries a verdict, a colour or a
red/green line; a permanently-visible line says so, and a unit test asserts both that the line
exists and that no row's text contains verdict vocabulary. Three non-clinical constants exist and
are disclosed in code and UI: a 0.5 depth-fraction crossover (45° of subject yaw — the point where a
measurement axis stops being mostly image-plane), a 0.5 cm / 1.0° DISPLAY floor that groups
near-zero findings instead of headlining them, and cos 45° for "is the subject standing over their
feet".

**View gating is the mechanism, not a heuristic.** Each finding declares the body axis it is
measured along and is gated on `depthFraction = |axis · camera optical axis|`, which is exactly
`∂(reported value)/∂(depth error on a contributing landmark)`. Above 0.5 the finding is listed under
"Not measurable from this view" **with the reason and the number**, never silently dropped. The
offline camera axis is known exactly (`MHRRetarget` documents and verifies X=image-right, Y=up,
Z=toward camera); the live ARKit path is a different frame and must supply its own — passing nil
suppresses everything.

⚠️ **The one real-data sample in the repo produces ZERO findings**, and that is correct: the dancer
fixture is a ~45°-oblique twisted pose (anteriorDepth 0.6217, lateralDepth 0.7989), so all nine are
withheld. The consequence is that the value path — "a real photo in a supported view yields a number
the user can check against their photo" — is exercised only by synthetic subjects. **That is the
biggest untested gap in this layer.** The panel has also never been seen in a running app; what was
verified is an off-screen `ImageRenderer` pass proving it lays out and draws.

⚠️ **A real defect in a file this layer does not own:** `MuscleOverlay.computeBodyFrame`'s `forward`
is `pelvisRight × up`, which by the right-hand rule is POSTERIOR. The muscle defs clearly intend
+z = anterior (rectus femoris "FRONT of thigh" at z = +0.04, semimembranosus "BACK" at z = −0.04,
erector spinae at z = −0.06), so every anterior/posterior capsule offset in `MuscleOverlay` is drawn
on the wrong side of the limb. The findings layer derives its own sign and pins it with two tests.
`MuscleOverlay.swift` is untouched — **this is still open.**

---

## Body-size gate (2026-08-07)

`MHRRetarget.plausibility(jointCoords:)` runs on **every** offline frame, ahead of
`nimble.scaleModel`. Bounds: inter-hip-joint-centre distance in `[0.10, 0.28] m`, chain-sum stature
in `[1.30, 2.10] m`.

The failure it exists for: `sample2`, a small heavily-occluded rider, produced a **0.070 m** hip
width, 0.116 m shoulder width and a 0.178 m humerus — a person at roughly half scale — and nothing
flagged it. `scaleModelWithHeight` clamps its per-segment factors into `[0.7, 1.4]`, so the collapse
did not FAIL there, it was **truncated into a model scaled to nobody** and every muscle number
computed on it looked ordinary.

- Both quantities are **pose-invariant** (a rigid inter-joint-centre distance and a chain sum), which
  is what licenses running the gate on every frame rather than only on the calibration frame. A bent
  knee or a raised arm cannot trip it — unlike the straight-line distances `scaleModelWithHeight`
  itself reads, where a raised knee collapses `|LHJC-LAJC|` from 0.816 m to 0.291 m. A test pins
  this by folding both knees fully and asserting the verdict is bit-identical.
- The frame is **kept** with its image, its retargeted skeleton and the measured numbers, and the
  playback badge reads e.g. *"Body size not measurable — hip width came out 7 cm (expected
  10–28 cm) … (hips 7 cm apart, height 0.84 m)"*. Dropping it silently is what made the original
  case invisible.
- Posture findings are suppressed for a rejected frame: every one of them is a distance or an angle
  on that skeleton, so a half-scale prediction would report half-scale centimetres.
- Margins: the four recorded PASSING statures are 1.602–1.715 m, so the 1.30 m floor sits ~19% below
  the smallest and the 2.10 m ceiling ~22% above the largest; `sample2`'s 0.070 m is 30% below the
  hip floor.

⚠️ These are a **gross-implausibility** gate, not an anthropometric norm and not a statement about
a body. A real person outside them would be rejected; the alternative is a muscle number computed on
a skeleton scaled to somebody who does not exist.

---

## Masking `shoulder_rot_{r,l}` was tested and REJECTED (2026-08-07)

Next-step 8 asked for the two axial-humeral-rotation coordinates to enter the runtime DOF mask,
because they are "structurally unobservable from one marker per shoulder plus one at the elbow".
**The premise is false and the change is a regression.** Nothing in the app installs a DOF mask, and
a test now asserts that.

The argument assumed the forearm markers lie on the humeral long axis when the elbow is straight. In
`FullBody.osim` they do not — the `ulna_*` and `hand_*` body origins are offset from that axis — so
axial rotation swings them even at zero elbow flexion. Marker-Jacobian columns
(`ShoulderRotObservabilityTests.mm`, at the model's neutral pose):

| column | ‖J[:,j]‖, m of marker motion per rad |
|---|---|
| `shoulder_elv_r` | 0.6077 |
| `elbow_flex_r` | 0.2507 |
| **`shoulder_rot_r`** | **0.0343** — small, 0.77% of the largest column, but **not null** |
| `shoulder_rot_r` at 90° elbow flexion | 0.266 (7.8× more observable) |

Per-marker at 0° elbow flexion: `d REJC/d shoulder_rot_r` = 16.3 mm/rad, `d RWJC` = 30.2 mm/rad. It
is **not** one of the 72 identically-zero columns in
`FullBodyDOFFixture.structurallyUnreachableCoordinates`.

Behaviour, A/B on the same bridge (`ShoulderRotMaskTests`):

| | unmasked | masked |
|---|---|---|
| dancer marker RMS | 2.1224 cm | **2.6874 cm** (+0.565) |
| dancer relative torque residual | 0.3545 | **0.3991** |
| dancer `shoulder_rot_r` | 0.6235 rad (35.7°) | pinned 0 |
| standing marker RMS | 0.0031813 cm | 0.0032228 cm (Δ 4.1e-5 — nothing to remove; the unmasked solver puts **0.04°** into the coordinate) |
| standing iterations / converged | **0 / YES** | **123 / NO** |
| standing per-solve drift | **0.0 rad** | **9.27e-5 rad** |

Root cause of the standing non-convergence, traced with `kIKTraceSolve`: unmasked, both LM phases
exit on the gradient test at `iters=0`, loss 2.0390e-8 — a genuine interior stationary point. Masked,
phase A still exits at 6 iterations but phase B (μ=0) hits the 120-iteration cap with the loss
creeping 2.07421885e-8 → 2.07421803e-8 → 2.07421721e-8. Removing the coordinate that was absorbing
the fixture's fourth-decimal rounding leaves a descent direction whose curvature is far below the
`1e-6·max(diag JᵀJ)` conditioning floor, so the damped Newton step degenerates into a gradient step
and the solver creeps. It is genuinely still moving, so neither tolerance should fire — **the
tolerances are not the problem, the extra pin is.**

The pre-registered gate ("adopt only if Δ RMS < 0.05 cm on both poses") failed at 0.565 cm. The test
now asserts the failure in both directions, so it fires if masking ever becomes free.

### A real order-dependence bug found on the way, and fixed

`applyDOFMaskWithNames:` took the pin from `_skeleton->getPositions()` — "wherever the coordinate
currently sits". The skeleton is shared process-wide (`sharedSkeleton`), so that meant *whatever
pose the last stage in the process wrote*. Measured: masking `shoulder_rot_{r,l}` straight after a
dancer solve pinned them at **0.6235 / 0.2877 rad** (the dancer's own answer), while the same call
on a freshly loaded model pinned them at 0. The pin is now the model's neutral pose (all-zero
clamped into the limits), which reproduces the old behaviour exactly for the 54 `<locked>true</locked>`
coordinates (nimble gives them a degenerate `[lo, lo]` range, so the clamp lands on `lo`) and turns
an inherited accident into a declared prior for the unlocked ones. The 57-name `runtimeMask` is
unaffected: drift 0.0, marker RMS identical to 1e-17, `Abs_*` were already 0.

⚠️ The test that pinned this was tautological in its first revision — it ran a dancer solve before
BOTH arms, so both inherited the same pose and it passed against the broken implementation. The two
arms must differ in the work that PRECEDES the mask.

---

## Next steps (ordered)

### Immediate — unblocked, no licence exposure

1. **Numerically diff the 24 shoulder muscle parameters** (F_max, optimal fibre length, attachment
   coordinates) in `FullBody.osim` against the **BSD-3 Holzbaur 2005** model. If they are the 2005
   values, the provenance is BSD-3 and the licence question changes character entirely. Hours of
   scripting, and it converts a legal inference into a fact. **Do this first.**
2. **Measure whether trapezius / serratus moment arms are non-zero** about the free thoracic DOFs.
   These 48 muscles are MIT-clear. Their scapular action is dead (spans a weld), but their thoracic
   action may be live. If it is, *"upper trapezius overworking from forward head"* is a
   commercially-clear, muscle-level, upper-body finding available today. Half a day.
3. ~~**Build the kinematics-only findings layer.**~~ **DONE 2026-08-07** — see
   [Posture findings](#posture-findings-a-kinematics-only-layer-2026-08-07). Nine findings, no
   clinical thresholds, view-gated. Note the basis was NOT lifted from
   `MuscleOverlay.computeBodyFrame`: that function's `forward` points posteriorly (see the same
   section), so the findings layer derives its own sign.
   **What remains on this line:** it has never run on a real photo in a supported view. The one
   real-data fixture in the repo is ~45° oblique, so every finding is correctly withheld and the
   value path is exercised only by synthetic subjects. The next step is three deliberate photos of
   the same person — straight-on, side-on, 45° — checking that the first two produce findings and
   the third produces none. That also tests whether the 0.5 depth gate is right in practice.

### High-value engineering — independent of the licence question

4. ~~**IK null-space damping / runtime DOF masking.**~~ **DONE 2026-08-07** — see
   [IK convergence](#ik-convergence-the-solver-is-now-a-fixed-point-2026-08-07). Damping toward the
   seed shipped as phase A (μ=1e-3) of a two-phase solve, together with five other fixes; the
   headline is that IK is now a fixed point and order-independent. **What remains on this line is
   the cost regression on moving input (1567 ms/frame in a Debug simulator build) and the fact that
   on-device Release timing has never been measured.**

   The DOF-mask half is built and reversible but **is not switched on anywhere** — see
   [Masking shoulder_rot](#masking-shoulder_rotrl-was-tested-and-rejected-2026-08-07) for why the
   one mask that was proposed measured as a regression. Historical corrections kept:
   `math::IKConfig` cannot express a DOF mask (11 fields, none selects DOFs; `refineIK` discards the
   bounds it is handed), so masking is a reparameterisation of the solve. The 57-name `runtimeMask`
   measures 169 → 112 free DOFs at identical marker RMS and 2.24× the speed.

   The rest of the 2026-08-06 rationale, kept because it is what motivated the work:
   E1 (below) measured a ~40-line solver change (gradient/step-based convergence in `refineIK`
   plus null-space damping toward the seed, μ=1e-3) as beating an entire pose-source replacement by
   **3.1–5.1×** on every axis: spurious spine motion 3.08×, SG-filtered `ddq` 3.55×, spine error
   1.70×, downstream torque 5.05× (measured against the healthy A′ arm, so this is not inflated by
   arm A's own ill-conditioning).
   Record it as a **stability** fix, not an accuracy fix — see the spine-claim constraint below.

   Rest of the original rationale still stands — reversible, no new shipped artifact, no rename, no
   patella bake, and it avoids nimble's documented penalty for intermediate WeldJoints
   (`OpenSimParser.cpp:5229`).
   This single change targets the red test, the ~200 ms/frame cost, and the `ddq` noise at once.
   ⚠️ **Do not weld the sternum or costovertebral joints.** Because the clavicle and scapula are
   already welded, those are the shoulder girdle's *only* articulation; welding them kills all 24
   trapezius and all 20 serratus slips — the scapular stabilisers, i.e. exactly the muscles behind
   the rounded-shoulder findings the product would sell.
5. ~~**Static-equilibrium inverse dynamics.**~~ **DONE 2026-08-07** — see
   [Static-hold gating](#static-hold-gating-2026-08-07). ⚠️ Re-scoped TWICE by measurement. First:
   now that IK is a fixed point there is no drift left for the filter to differentiate, so on a HOLD
   the gate is a measurable no-op (peak torque identical to 16 significant figures). Second: the
   claim that "the pelvis-pinned pose source cannot supply accelerations" is **half wrong** — it can,
   via `cam_t`, which the app already has. What it cannot supply is a usable *depth* acceleration,
   and no clip filmed from a moving camera has an inertial frame regardless. Both constants were
   re-derived from a stated error budget in the same pass.
6. ~~**Patella rename + weld** (with the `groupScale()` patch). Ship blocker for squat analysis.~~
   **DONE 2026-08-06** — see [Muscle-output ship blockers](#muscle-output-ship-blockers-fixed-2026-08-06).
7. ~~**Shoulder axis orthogonalisation** (6 lines).~~ **DONE 2026-08-06**, same section. Note it was
   *not* 6 lines of "reach the generic path" — that route is blocked by `getAxisOrder()`; the axes
   had to be unit-snapped.

### Newly opened by the 2026-08-06 work

8. ~~**Mask `shoulder_rot_{r,l}`.**~~ **TESTED AND REJECTED 2026-08-07** — see
   [Masking shoulder_rot](#masking-shoulder_rotrl-was-tested-and-rejected-2026-08-07). The premise
   was wrong: the marker-Jacobian column is 0.0343 m/rad, not zero, because the ulna and hand body
   origins sit off the humeral long axis. Masking costs the dancer 0.565 cm of marker RMS and breaks
   convergence at standing. **Do not re-open without re-measuring that column norm first.**
9. ~~**Plausibility gate on the offline path.**~~ **DONE 2026-08-07** — see
   [Body-size gate](#body-size-gate-2026-08-07).
10. ~~**Static-hold detection**~~ — same item as next-step 5, now done and re-scoped; see there.

### Newly opened by the 2026-08-07 work

11. **`MuscleOverlay.computeBodyFrame`'s `forward` points posteriorly.** `simd_cross(pelvisRight, up)`
    is −anterior, so every anterior/posterior capsule offset is drawn on the wrong side of the limb:
    quadriceps behind the thigh, hamstrings in front, erector spinae in front of the spine. Rendering
    only — it does not touch any solved number — but it is visibly wrong and it is cheap.
12. **Validate the findings layer on real photos** (see next-step 3's remainder). Currently the
    highest-value open item, because it is the only part of the product a user can check against
    their own photo.
13. **Measure IK cost in a Release build on device.** The 1567 ms/frame moving-input figure is a
    Debug simulator number and the app's live path depends on it. If it is real, the untried lever is
    the Woodbury/dual form of the normal equations (169×169 vs a 60-row residual, ~8× on the cubic
    term).
14. **`E1MarkerSetComparisonTests` no longer compiles its own premise.** It fails at
    `E1MarkerSetComparisonTests.mm:475` asserting its coordinate blocks cover the model: 163 != 169.
    The model gained 6 shoulder DOFs and lost 2 `knee_angle_*_beta` in the 2026-08-06 osim edit; E1's
    fixture still enumerates 163. Pre-existing, not caused by any 2026-08-07 change (the failing
    assertion is inside `buildCoordinateSets`, pure name bookkeeping, and never calls a solver).
    Separately, E1's V4 "does the harness reproduce production" probe is now structurally stale — it
    compares the bridge against a reimplementation of `refineIK` that production no longer uses. V4
    carries no assertion, so nothing fails, but its number is meaningless. **E1's STOP verdict is
    unaffected**: every arm uses E1's own internal solvers.

### Newly opened by the cam_t measurement (2026-08-07)

15. **Pass `camT` at `OfflineSessionRunner.swift:242`.** One argument. Until it lands,
    `MHRRetarget.makeBodyFrame(jointCoords:camT:…)` is exercised only by `RootTranslationTests` and
    `rootTranslationObservable` is false on every real frame. It also fixes things that have nothing
    to do with dynamics: the ground-height estimator, GRF contact detection and the CoP all currently
    run on a body whose pelvis sits at a model constant.
16. **Surface `MotionVerdict` in the UI.** `OfflineResultStore.MotionState` has two cases and
    `OfflinePlaybackView.motionDetail` hard-codes *"muscle loads need a still pose"*. The engine now
    reports four reasons with a sentence each (`MotionVerdict.advice`), including
    `.indistinguishableFromNoise`, which is *not* the user's fault and has a different remedy.
17. **Build the camera-static check, upstream where the frames are.** Measured to separate the
    owner's clips by 20-60× (§ cam_t). Two constraints from the measurement: it must run at the
    video's NATIVE rate (at 10 fps sampling the estimator aliased a 13.5 °/s pan down to ~0), and it
    detects rotation well but translation poorly. On iOS the natural tool is
    `VNTranslationalImageRegistrationRequest` on the region outside the person box. It is cheap —
    no pose model — so it can run densely even when the pose sampling is sparse.
18. **Decide (a) refuse vs (b) declare-depth-constant** — see the owner decision in the cam_t
    section. This is what gates whether a dynamic branch exists at all.
19. **Drop Vision-fallback frames out of any derivative.** 22/309 frames on `video_012`; they make
    `cam_t` and the body scale wild, and today they are pushed into the SG filter like any other.

### Owner decisions still open

- **Dynamic muscle output: refuse, or declare depth constant?** See
  [cam_t recovers the root translation](#cam_t-recovers-the-root-translation-its-depth-cannot-be-differentiated-twice-2026-08-07).
  Option (b) is the only one under which dynamics ships at all, and it puts a modelling assumption
  ("you are not moving toward or away from the camera") inside a number the product sells.
- **How to resolve the arm licence**: negotiate MoBL-ARMS commercially, adopt/convert the BSD-3
  Holzbaur model (SIMM → .osim conversion needed), or rely on step 1 showing the data is already
  BSD-3 lineage. **Needs actual legal counsel** — the verbatim quotes and URLs above are assembled
  so a lawyer can be briefed directly.
- **Does the upper limb need muscles, or only posture?** If the value is "rounded shoulders /
  forward head", that is pure geometry and the licence question does not block the product at all.
- **Whether to keep `FullBody.osim` at all.** Rajagopal2016 is ruled out (no upper limb). Every
  alternative surveyed trades the shoulder weld for a total *absence* of shoulder muscles.
  A proper survey of externally-sourced commercial-safe full-body models **has not been done** —
  one research agent silently returned placeholder output and that gap was never filled.

---

## Process notes

- **Verify agent claims before acting on them.** In this project two agents directly contradicted
  each other on whether shoulder muscles use `PathWrap` (they do not — 0 of 24). Licence analysis
  was inverted once. Muscle-count and wrap-count figures were misreported in both directions.
  Every load-bearing number in this document was re-derived by hand.
- **One agent destroyed `BioMotionTests/NimbleBridgeTests.swift` with an errant `git checkout`** and
  reconstructed it from memory. 11 of its 12 tests pass against an unmodified implementation, which
  corroborates fidelity, but **that file is a reconstruction, not the original bytes** — worth a
  careful read.
- Concurrent agents must own **disjoint files** and must **not** run builds (shared DerivedData
  thrashes). Build and verify serially afterwards.
- A **new** test file requires `xcodegen generate` to enter the target even when `project.yml` is
  unchanged — otherwise it sits on disk silently not running.
- **A test that would have passed against the broken implementation proves nothing.** Two examples
  from 2026-08-07, both caught only by deliberately re-reading the harness: the DOF-mask
  order-independence test ran the same prelude before BOTH arms, so both inherited the same shared
  skeleton pose and it passed against the very bug it was written for; and `StaticHoldTests`'
  attribution control exists precisely because the shared skeleton could have produced the measured
  gap on its own. Ask what the arms actually differ in.
- **Pre-register the gate, then let it fail.** The `shoulder_rot` mask was written, measured against
  a bound chosen before the numbers existed, and rejected. The durable artifact is the rejection
  plus its mechanism, not the code. Two of this file's own next-steps have now been closed by
  measurement showing the premise was wrong (next-step 8 here, the DINOv3 matrix in a sibling
  project) — a next-step is a hypothesis, not an instruction.
- **Instrument before theorising about a solver.** `kIKTraceSolve` (a compile-time flag in
  `NimbleBridge.mm`) turned "why does masking break standing?" from three plausible stories into one
  measured stop-reason trace in a single run.
