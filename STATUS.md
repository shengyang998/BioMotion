# BioMotion — STATUS

**Single source of truth for progress. Read this before touching anything.**
Last updated: 2026-08-06.

---

## TL;DR

The app's inaccuracy was diagnosed to root cause. It was never one bug — it is a chain, and the
biggest links were **not** where the effort had been going.

- Five implementation defects were found, fixed, and pinned with tests. The test target
  **did not even compile** before this work, so the project had no regression net at all.
- The dominant remaining error source is **not** the muscle solver: it is that IK solves
  **163 degrees of freedom from ~40 scalar observations**. ~127 of those DOFs are spine and rib
  coordinates that ARKit cannot see.
- ~~The shipped model's **shoulders are welded** (zero shoulder DOFs), so upper-limb muscle output
  is currently meaningless regardless of anything else.~~ **Fixed 2026-08-06** — see
  [Muscle-output ship blockers](#muscle-output-ship-blockers-fixed-2026-08-06).
- A **commercial licence blocker** was found on the upper limb (MoBL-ARMS is non-commercial).
  A BSD-3 alternative exists.

Current build: **87 / 88 tests pass** on the iOS simulator. One test is deliberately left red —
it surfaces a real defect (see [Known-red test](#known-red-test)).

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

1. **Observability.** ARKit gives ~19 joints; the app registers ~12–14 virtual markers ≈ 36–42
   scalar observations. `FullBody.osim` parses to **163 DOFs**. At least ~121 DOFs sit in the exact
   null space of the marker Jacobian. Solutions there are artifacts of the warm start, not
   measurements.
2. **Amplification.** Those unconstrained DOFs wander; the wander is differentiated twice by the
   Savitzky–Golay filter (gain ≈ 1/dt² ≈ 3600 at 60 fps) to produce `ddq`.
3. **Propagation.** `τ = M·ddq + C − JᵀF_ext` is linear in `ddq`, and the muscle QP is
   τ-match-dominant (`softPenalty = 100`), so activations inherit essentially the full amplified
   torque noise. Pose error also enters a *second* time through `R(q)`, so the two compound
   bilinearly rather than additively.

**Implication:** fixing things downstream of IK has limited headroom. The leverage is at IK and above.

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

| | Before | After |
|---|---|---|
| Test target | **did not compile** (`MomentArmTests.swift:124` used a stale signature) | builds |
| Tests | 0 runnable | 88 total, **87 pass** |

Run with:
```bash
# XcodeBuildMCP session defaults: project BioMotion.xcodeproj, scheme BioMotion,
# simulator "iPhone 17" (F85FB2DA-66AD-428D-A4EC-1390A42D9FCB), Debug
xcodebuild -project BioMotion.xcodeproj -scheme BioMotion \
  -destination 'platform=iOS Simulator,name=iPhone 17' test
```

### Known-red test

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

- 171 XML coordinates → **163 DOFs parsed**; ~127 are spine + rib.
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

### The limitation that shapes the product claim

**`joint_coords` pins the pelvis at a model constant `(0, 0.924, 0)` in every frame.** `global_trans`
is zeroed (`sam3d_body.py:1600`), so that number is not the subject's pelvis height and `y = 0` is
not the floor.

Joint *angles* are frame-invariant and therefore correct. But the body has no global vertical
motion, so **dynamic** inverse dynamics is not sound on this path — in a squat the pelvis does not
descend, the feet appear to rise instead. This points the same way as next-step 5 below:
static-equilibrium ID (`q̇ = q̈ = 0` over a detected hold) is both the honest reading of this input
and the largest accuracy lever available. Recovering true global motion from monocular video needs
camera-pose/SLAM, which is a different project.

T-pose calibration **is** skippable, but only via `segmentScaleMarkers`, which rebuilds a synthetic
straight-limb marker set from pose-invariant chain sums. Handing the bridge raw posed markers fails
the `[0.7, 1.4]` clamp on 6 of 6 test predictions (a seated yoga pose gives lower 0.351).
The method cannot rescue a bad prediction: a small, heavily occluded subject produced a degenerate
0.070 m hip width. A plausibility gate on hip width and stature is recommended and **not yet built**.

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
3. **Build the kinematics-only findings layer.** Zero model dependency, zero licence exposure, and
   it is what the product can actually defend: forward head, rounded shoulders, shoulder-height
   asymmetry, lateral head tilt, trunk plumb-line, thoracic-kyphosis proxy
   (`hips→spine_1→spine_4→spine_7` chain angle), transverse trunk rotation, humeral-elevation
   asymmetry, lateral weight shift. All from joints already in `BodyJoint.primary`.
   `MuscleOverlay.computeBodyFrame` already builds the trunk-stable basis needed — lift it into a
   shared helper.

### High-value engineering — independent of the licence question

4. **IK null-space damping / runtime DOF masking.** ~~Use the existing `math::IKConfig` at
   `NimbleBridge.mm:690` to lock unobservable coordinates at runtime~~ — **corrected 2026-08-06:
   `math::IKConfig` cannot express a DOF mask.** It has 11 fields and none of them selects DOFs, and
   `refineIK` explicitly discards the bounds it is given (`(void)upperBound; (void)lowerBound;`).
   Masking has to be done by reparameterising the solve instead; that was implemented and measured
   (163 → 106 DOFs, marker fit unchanged).

   **Damping is now the highest-value change in this file, on measured evidence — do it first.**
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
5. **Static-equilibrium inverse dynamics.** Add a hold detector (marker speeds < ~2 cm/s for ≥ 0.5 s),
   average the pose over the hold, and run ID with `q̇ = q̈ = 0`. This deletes the 1/dt²
   amplification chain entirely on the static path, and it matches the product's own framing
   ("current posture"). Largest single accuracy lever available.
6. ~~**Patella rename + weld** (with the `groupScale()` patch). Ship blocker for squat analysis.~~
   **DONE 2026-08-06** — see [Muscle-output ship blockers](#muscle-output-ship-blockers-fixed-2026-08-06).
7. ~~**Shoulder axis orthogonalisation** (6 lines).~~ **DONE 2026-08-06**, same section. Note it was
   *not* 6 lines of "reach the generic path" — that route is blocked by `getAxisOrder()`; the axes
   had to be unit-snapped.

### Newly opened by the 2026-08-06 work

8. **Mask `shoulder_rot_{r,l}`.** The shoulder unweld added 6 DOFs, of which the two axial-rotation
   coordinates are structurally unobservable from one point per shoulder plus one at the elbow.
   By this file's own E1 finding, unobservable DOFs get excited by the solver. Do this before
   trusting any shoulder muscle number.
9. **Plausibility gate on the offline path.** Reject predictions whose hip width falls outside
   ~0.10–0.28 m or whose chain-sum stature falls outside ~1.3–2.1 m, before they reach
   `scaleModelWithHeight`. One occluded test subject produced a 0.070 m hip width.
10. **Static-hold detection** (this is next-step 5 above, now load-bearing). The offline path's
    pelvis pinning means dynamic ID is not sound; static-equilibrium ID is the honest reading.

### Owner decisions still open

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
