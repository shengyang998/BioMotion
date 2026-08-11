# BioMotion — STATUS

**Single source of truth for progress. Read this before touching anything.**
Last updated: 2026-08-11.

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
- **IK is now a fixed point** (2026-08-07). Repeated converged solves on identical markers move
  exactly 0 rad and the answer no longer depends on how many solves preceded it. Under the former
  PELVIS-root mapping the dancer RMS went 5.4913 → 2.1224 cm; the source-specific MHR_ROOT repair on
  2026-08-10 lowers the current fixture to **1.5365 cm**, or **1.2758 cm** after source-aware scaling. See
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
- **The gait-cycle route still supports KINEMATIC CONTACT TIMING; its load route is CLOSED on the
  bundled models** (updated 2026-08-10). The historical reasoning treated root acceleration and
  stance impulse as a possible load model. The evidence did NOT validate foot support: the probe that produced
  it wrote 41 distinct frames for 120 requested, so the headline "contact 200 ms, zero spread across
  a 2.5× threshold span" measured a quantisation staircase. Corrected extraction gives contact
  167–247 ms and a raw, hypothetical peak of 2.08–2.85 BW at **±32%**, with no robustness result.
  More decisively, both bundled `ContactGeometrySet`s are empty and the active near-CoP routine has
  no validated support-domain, unilateral-contact, or friction constraint. The product preserves
  contact timing but publishes no GRF, CoP, torque, muscle, or gait-load value. See
  [Gait-cycle dynamics](#gait-cycle-dynamics-the-route-survives-its-evidence-did-not-2026-08-07).
- **The per-muscle LEFT/RIGHT claim is RETIRED** (2026-08-08). The measurement that certified it ran
  on a rig where it could not fail — the QP is linear in the joint torques, so a right leg scaled
  `0.8×` gives `a_R = 0.8·a_L` for ANY moment arms. Give the right leg a different torque SHAPE
  instead, and an unmodelled `PathWrap` moves a published figure by **9.92 pp**, turning a real
  −17.9 % into a displayed −8.0 %, on a muscle whose own path is modelled correctly. Where it does
  cancel, every muscle reads the same number. Separately, the eight rows shown were the top of 175
  screened pairs quoted at a per-comparison 95 %; family-wise, **zero claims survive on any pinned
  clip at any scatter level**. The running screen's surviving left/right finding is CONTACT TIME. See
  [Fifth round](#fifth-round-the-cancellation-was-an-identity-and-the-per-muscle-claim-is-retired-2026-08-08).
- **The PICTURE was making the retired claim too, on both screens** (2026-08-08). `MuscleOverlay`
  kept the strongest 24 activations and coloured every capsule from one shared blue→red ramp — the
  cross-muscle ordering retired in round four, drawn where it reads as most authoritative and with
  no number, caption or floor beside it — and the live screen carried a bar chart of twelve muscles'
  activations in per cent under it. Both are gone. `update(joints:)` now takes **no muscle solve at
  all**, so the renderer cannot re-acquire a magnitude; the capsules are the fixed 26-muscle
  anatomical set in one constant colour; both screens carry `MuscleOverlay.anatomyOnlyNote`. See
  [Sixth round](#sixth-round-the-same-claim-in-colour-2026-08-08).
- **The SURVIVING claim was gated on the wrong variance too, and its honest floor is roughly
  DOUBLE what shipped** (2026-08-08). `asymmetryClaim` published on
  `resolution.resolvableAsymmetryPercent` — quantisation plus STRIDE-PERIOD scatter — while the
  statistic is the difference of two means of ~5 CONTACT DURATIONS, whose own scatter was measured
  (`contactVariationPercent`) and consumed by nothing. Monte-Carlo, symmetric runner, 5 contacts a
  side at `video_015`'s measured 11.144 % contact scatter: the shipped gate published a left/right
  finding on **25.3 % of clips**; with the sampling half-width in the floor it publishes on 2.4 %
  against a 5 % nominal. **No pinned clip publishes a contact-time claim, and none did before
  either** — `video_012` 2.90 % against a 10.145 % floor, `video_015` −0.54 % against a floor that
  rises 8.086 → **16.464 %**, `video_013` refused. The claim can honestly find a 20-25 % left/right
  contact difference on a 4 s clip and not much finer. The LIVE screen's former undisclosed "L/R
  load" value was first narrowed to a raw `GRF sum` diagnostic and is now unreachable for both
  bundled models: the contact-support gate withholds the whole dynamics row. The muscle block states
  the model's permanent limit before any clip refusal, and solver bound counts are no longer facts about
  a body. See
  [Seventh round](#seventh-round-the-surviving-claim-was-gated-on-the-wrong-variance-too-2026-08-08).
- **The moment arms finally have an AUTHORITATIVE REFERENCE, and the defect that retired the muscle
  claim is measured** (2026-08-08). OpenSim 4.6 installs from PyPI on Apple Silicon
  (`uv pip install opensim`, no conda), loads the shipped `FullBody.osim` and reports its 69 wrap
  objects. Dumping its own moment arms over 173 poses: on the 66 muscles that carry a `PathWrap` the
  straight-line shortcut is out by a median of **13.7 %**, p90 124.4 %, worst **146.6 mm**, and
  **9.00 % of the pairs have the WRONG SIGN**. The control that makes it an attribution: the 454
  muscles with no wrap object are identical in the wrapped and unwrapped models to the last stored
  digit — exactly 0.0. The shipped `MomentArmComputer` reproduces OpenSim-with-wrapping-off to
  4.39 mm, so 97 % of its error is the missing solver.
  `BioMotionTests/Fixtures/opensim_moment_arms.txt` is now the gate a wrap solver has to pass. See
  [the reference](#the-moment-arms-now-have-a-reference-and-the-defect-is-measured-2026-08-08).
- **CYLINDER PATH WRAPPING SHIPS** (2026-08-08; historical cylinder-stage numbers).
  `MusclePathWrap.cpp`, ported from
  opensim-core (Apache 2.0, attribution in `./NOTICE`), solves **64 of FullBody's 76**
  `PathWrap` references and **all 46** of Rajagopal2016's. On the 56 muscles whose every
  wrap is a cylinder the moment-arm error against OpenSim falls from median 0.972 mm /
  max **146.6 mm** / **661 sign flips** to median **0.048 mm** / max **8.07 mm** /
  **4**; against OpenSim's own derivative of its own length the single-wrap muscles read
  max **3.569 mm** and **1** flip, at a 1.00 mm reference value below what that historical
  stage called the 3.758 mm no-wrap floor. The current gate no longer self-calibrates from that
  number. Wrap engagement matches on 2,016/2,016 rows.
  The discontinuity is handled by a signature-aware one-sided difference and proved on a
  constructed switch (raw centred **−19.62 m**, shipped **−0.0337 m**). Cost: 889 →
  6,049 ms per solve in Debug, ~36 ms extrapolated at −O2, NOT measured on device. Still
  missing: `WrapEllipsoid` (12 references, 10 elbow muscles). The per-muscle left/right
  claim is NOT reopened.
  [Cylinder wrapping](#cylinder-path-wrapping-ships-2026-08-08).
- **ELLIPSOID PATH WRAPPING SHIPS TOO, so every `PathWrap` in the model is solved**
  (2026-08-08; pre-exact-MovingPath snapshot). The remaining 12 references — 8
  `WrapEllipsoid`s on the humeri, 10 elbow
  muscles — are a port of opensim-core's `WrapEllipsoid.cpp` (`hybrid` method only; the
  other two are refused because they are not pure functions of the pose).
  `unmodelledPathWraps` is **0** and `GaitLoadSummary.musclesWithUnmodelledPaths` is
  **empty**. Against OpenSim's own derivative: single-wrap p90 **2.438 mm**, max
  **4.414 mm**; path LENGTH max **0.210 mm**; engagement **600/600**; sign flips
  **135 → 0**; numerical refusals **0** over 60 poses. It was implemented rather than
  disclosed because the paired A/B says **1.28×** (7,688 vs 6,019 ms, Debug) against a
  pre-registered 3× ceiling — an ellipsoid solve is 130–450× a cylinder solve at −O2 but
  only runs when the segment actually pierces the surface. The ablation is the finding:
  without it, `BRD_r`/`elbow_flex_r` read **−8.73 mm** against a true **+1.51 mm**, sign
  and all. In that pre-exact snapshot, the largest surviving residual was 4.414 mm on
  `BICshort_l`/`pro_sup_l` and was attributed to MovingPathPoint chord interpolation. Exact
  SimmSpline evaluation now lowers the measured maximum to 2.679 mm; see the 2026-08-10 entry.
  The per-muscle left/right claim is STILL NOT reopened.
  [Ellipsoid wrapping](#ellipsoid-path-wrapping-ships--every-pathwrap-in-the-model-is-solved-2026-08-08).
- **THE RE-MEASUREMENT IS DONE. The moment arms are fixed; the claim still cannot come back, and the
  reason is no longer the moment arms** (2026-08-09). Re-run on REAL geometry — 40 right-leg muscles
  of `FullBody.osim`, mirrored into a bilateral rig so the modelling error is bilateral by
  construction, 31 poses × 5 torque shapes × 3 effort levels × both OpenSim reference definitions,
  582 readable cells. The wrap solver did what it was for: the median moment-arm leak is **0.977 pp**
  against the straight line's **7.939 pp** on the identical rig, and the original three-muscle rig
  re-run with the perturbation resized from the guessed `×0.6` to the MEASURED p99 residual (1.114 %)
  reads **0.568 pp** where it read **9.92 pp**. But a second error was found that is larger than the
  first and is not about geometry at all: with the arms held FIXED, OSQP's answer differs from the
  exact minimiser of the SAME objective by a median of **14.88 pp** and a max of **100.98 pp**,
  against an 8.086 % publication floor. `MuscleSolver` runs at `eps_abs = eps_rel = 1e-3` with
  polishing off and accepts `OSQP_SOLVED_INACCURATE`, i.e. 0.02 of ABSOLUTE activation slack, and a
  left/right PERCENTAGE built from two such numbers at a median activation of 0.132 carries
  `100·2·0.02/0.132 = 30 pp`. `perMuscleLeftRightClaimIsSupported` stays `false`. See
  [the re-measurement](#the-re-measurement-the-moment-arms-are-fixed-and-the-claim-still-cannot-come-back-2026-08-09).
- **THE SCREENS SAID THE OLD REASON FOR ONE BUILD** (2026-08-09). The wrap commits and the
  re-measurement shipped with the user-facing text untouched, so `perMuscleRetirementSentence` told
  every user that "66 of its muscles are given a straight line where the real tendon wraps around
  bone" (runtime report: **76 solved / 0 unmodelled**) and that the error is worth "around 10
  percentage points on this app's own test rig" (that rig, at this build's residual: **0.568 pp**),
  and `MuscleOverlay.anatomyOnlyNote` said the same thing on BOTH the live and the offline screen.
  Two `XCTAssertTrue(... contains("wraps around bone"))` assertions held them there. The refusal is
  unchanged and its reason is now the measured one — the QP stops at "close enough", **14.88 pp**
  median against an **8.086 %** floor. The flag's own registered reopening condition, which was
  SATISFIED while the decision was correctly `false`, is replaced by the R1–R7 gate list that
  actually decides. See
  [the screens](#the-screens-now-state-the-reason-that-is-true-2026-08-09).
- **THE CALF MUSCLES ARE NOT WRONG — OpenSim's multi-wrap LENGTH is not the length of a path**
  (2026-08-09). The reported "systematic ~10-11 mm" offset on `gasmed`/`gaslat140` across the
  running knee range, and the W1 falsifier that "passes only because of where the 60-pose grid
  samples", are the same defect and it is the reference's. `calcLengthAfterPathComputation` adds
  straight segments measured between the wrap points OpenSim REPORTS to the spiral length OpenSim
  STORED beside them; for a two-cylinder path those halves describe different paths. On `gasmed_r`
  at knee 0°, the stored spiral is **0.038054 m** while the CHORD between its own two tangent points
  is **0.045350 m** — shorter than the straight line between the points it must connect, so the
  total is ≥ **7.30 mm** below any real path. Cause: `WrapCylinder::_adjust_tangent_point`, which
  runs only for multi-`PathWrap` muscles, moves the points and nothing recomputes the arc. Reconcile
  the total with OpenSim's OWN tangent points and `gasmed_r`'s moment-arm gap goes **10.484 mm →
  0.033 mm** median (length 4.203 → **0.0041 mm**); the reference's worst row, found by scanning for
  its own length jumps, reads **41.26 mm** raw and **0.54 mm** reconciled. OpenSim's OWN analytic
  moment arm — which reads the wrap points and never touches the length bookkeeping, so it owes this
  repair nothing — sides with the port to a median of **0.0005 mm** and a max of **1.05 mm** over
  every multi-wrap muscle's full range. The port iterates its wrap objects to a fixed point (2
  passes) and its path is self-consistent to the last stored digit at
  451/451 poses. Nothing in `BioMotion/Muscle/**` changed; the deliverable is
  `opensim_multiwrap.txt` + `MultiWrapReferenceTests`. See
  [the calf muscles](#the-calf-muscles-are-not-wrong-opensims-multi-wrap-length-is-not-a-path-length-2026-08-09).
- **THE LEAK EXPERIMENT WAS RE-RUN AND THE CLAIM DOES NOT COME BACK — the largest remaining term is
  the REFERENCE** (pre-SimmSpline snapshot, 2026-08-09). Same 582 cells, same R1–R7, same 1.617 pp
  bar; R1 **123.0971 pp**,
  R2 **108.5752 pp**, R7 **47.52**, control **66.8824** — bit-identical to the QP round, so the two
  initial diagnostics are provably inert. `perMuscleLeftRightClaimIsSupported` stays `false`. What is
  new is the attribution. R1's worst muscle was recorded as `piri`/`glmed3` because the print beside
  `leakExact` was the worst SHIPPED base; it is **`bflh140`**, which carries **no `PathWrap`, no
  `MovingPathPoint` and no finite-difference row**, so its moment arm is the SAME NUMBER in both
  reference matrices — and its figure still moves **126.44 pp** between them. Sweeping OpenSim's
  other column as a SUBJECT puts the reference's self-disagreement on R1's scale: median **5.28 pp**
  against our **0.977 pp**, worst **126.44 pp** (larger than R1's own worst, 78× the bar), and our
  leak is the smaller of the two in **466 of 582 cells**. So R1 as registered is maximised over a
  `truth` that is not one, and no work on `MomentArmComputer` can pass it that way — but against the
  better-founded analytic column alone our own worst is still **42.46 pp**, 26× the bar, so the claim
  stays retired on the measurement. The next diagnostic prints all 24 screened rows at that
  ANALYTIC worst cell and corrects the first attribution: `bflh140_r` itself has the largest arm
  error there, **16.059 vs 13.713 mm** about the knee (**+2.346 mm**), ahead of `gaslat140_r` at
  1.597 mm. The 42.46 pp tail is therefore not evidence for an unnamed neighbour; it is now
  localised to a wrap-free, fixed-point path whose pose-dependent derivative still disagrees.
  Also settled: R3 is **1.4022 pp** (0.568 was two noisy solves cancelling), and the two tests the QP
  round left failing. See
  [the leak re-run](#the-leak-experiment-re-run-the-claim-does-not-come-back-and-the-largest-remaining-term-is-the-reference-2026-08-09).
- **THE 42.46 pp ANALYTIC TAIL WAS A SIMMSPLINE ENDPOINT-EXTRAPOLATION BUG, AND IT IS FIXED**
  (2026-08-09). `walker_knee_r` permits 140°, but its five nonlinear transform splines end at 120°;
  Nimble continued the last cubic while OpenSim continues the endpoint tangent. A two-sided RED test
  pinned value/d1/d2 outside both ends; both device and simulator archives were rebuilt. At the 130°
  `run_4_mid_swing` pose, `bflh140_r` is now **13.713464915 mm** versus OpenSim
  **13.713465000 mm** (delta −0.000000085 mm), down from 16.059 mm. The analytic-only maximum falls
  **42.4623 → 3.6932 pp** (p99 9.94 → 3.332, median 0.412 → 0.312), a 91.3 % reduction. It is still
  2.28× the unchanged 1.617 pp bar, and registered R1 still includes the separate 123.083 pp
  central-difference/reference tail, so the per-muscle claim stays retired. See
  [the endpoint-extrapolation fix](#the-4246-pp-analytic-tail-was-endpoint-cubic-extrapolation-2026-08-09).
- **MOVINGPATHPOINT SIMMSPLINE IS EXACT END TO END** (2026-08-10). FullBody reports
  `Moving 4 parsed (0 approximated, 0 skipped)`: the app now evaluates the canonical Nimble
  SimmSpline rather than a chord through its knots, both in-domain and along the endpoint tangent.
  The affected ellipsoid sweep's central-difference maximum falls **4.414 → 2.679 mm** and its
  analytic maximum **4.385 → 2.301 mm**; the remaining residual is reported without
  pre-attribution. Malformed non-increasing knots are rejected. Wrap sign gates no longer derive a
  threshold from the current no-wrap maximum: cylinder and ellipsoid total-sign tripwires use the
  original fixed 1 mm resolution, while ellipsoid direction also has a fixed 1 mm causal A/B gate
  in which a zero actual effect fails. No claim is reopened. See
  [MovingPathPoint semantics](#movingpathpoint-uses-exact-simmspline-semantics-2026-08-10).
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
| 418 `ConditionalPathPoint` + 4 `MovingPathPoint` silently dropped | `MomentArmComputer.mm:168` | tinyxml2 name-matched iteration skipped them. Now walks **all** children in document order (ordering matters — these are polyline vertices). All four FullBody MovingPathPoints parse, their SimmSpline components use `dart::math::SimmSpline`, malformed knots fail closed, and the runtime report is `parsed 4 / approximated 0 / skipped 0`. |
| Ground height was a **monotonic ratchet** | `NimbleBridge.mm:634-638` | One crouch/landing/drift permanently sank it → both feet read >6 cm above "ground" → `contactCount==0` → ID solved a free-floating body with **zero external force for the rest of the session**. Replaced with a bounded rolling robust percentile that can rise as well as fall, plus a reset hook. |
| `jointVelocities` accepted but never read | `MuscleSolver.mm:573,593` | Fiber velocity came from wall-clock finite differencing whose `dt` jittered with dropped frames and disagreed with the SG filter's own `dt`. Now uses the analytic identity `dL_MT/dt = −Rᵀ·dq`. Sign convention derived from `MomentArmComputer.mm:381` (`R = −∂L_MT/∂q`), not guessed. |
| IK did **5 random restarts every frame** | `NimbleBridge.mm:472,510` | `IKConfig.lossLowerBound` defaults to 0 (header) / 1e-10 (ctor) — both unreachable against a realistic 0.01–0.03 m ARKit marker residual, so the restart loop always ran to completion, each iteration calling `getRandomPose()` and discarding the previous solution at 171 DOF. Now 1 restart, warm-started from the previous pose, plus a static marker-reliability weighting (trunk 1.00 → toes 0.40). |

Also: torque residual now exposed on `MuscleActivationResult`; `aMin` comment no longer justifies an
optimizer bound by colormap appearance; dead `maxMuscleForceAtState` and the legacy
`solveWithJointTorques` hardcoded-moment-arm path removed (verified zero non-test callers).

### Test suite

| | 2026-08-06 start | after Phase 0 | 2026-08-07 | 2026-08-08 |
|---|---|---|---|---|
| Test target | **did not compile** (`MomentArmTests.swift:124` used a stale signature) | builds | builds | builds |
| Tests | 0 runnable | 88 total, 87 pass | 219 total, 219 pass | **408 total, 408 pass, 0 crash-restarts** |

The last +10 are the OpenSim moment-arm reference (`OpenSimReferenceTests` 6,
`StraightLinePathErrorTests` 4) — see
[the moment arms now have a reference](#the-moment-arms-now-have-a-reference-and-the-defect-is-measured-2026-08-08).
The +30 after that are cylinder path wrapping (`MusclePathWrapTests` 17,
`CylinderWrapValidationTests` 11, `MomentArmWrapDiscontinuityTests` 2), taking the
`tools/run_tests.sh` floor to **438** — see
[cylinder wrapping](#cylinder-path-wrapping-ships-2026-08-08).
The +21 after THAT are ellipsoid path wrapping: `MusclePathWrapTests` 17 → 28 (the
sphere closed form, both quadrant tests, the engagement boundary, the chord-count
signature, purity, the closest-point routine against an exhaustive search and against
its own scale trap), `EllipsoidWrapValidationTests` 11 new, and
`CylinderWrapValidationTests` 11 → 10 (the "ellipsoid muscles are reported and not
claimed" test moved and became a claim). Floor **459** — see
[ellipsoid wrapping](#ellipsoid-path-wrapping-ships--every-pathwrap-in-the-model-is-solved-2026-08-08).
The +15 after that are the re-measurement: `WrappedMomentArmLeakTests` 10 (the real-geometry
mirrored rig, the exact-solver check, the straight-line control, the solver-slack finding
and the decision tripwire), `BoxQPTests` 4 (the exact QP against a closed form, a dense
direct solve, coordinate descent and a direct optimality test) and
`MomentArmErrorCancellationTests` 10 → 11 (the original rig re-run at the MEASURED
residual instead of the guessed `×0.6`). Floor **474** — see
[the re-measurement](#the-re-measurement-the-moment-arms-are-fixed-and-the-claim-still-cannot-come-back-2026-08-09).

The current runner separates the ordinary suite from the deliberately expensive E1 experiment:

| mode | selection | required receipt | meaning |
|---|---|---|---|
| `fast` | runner-owned non-E1 suite | exactly 532 passed; 0 failed/skipped/expected-failed/restarted | fast lane |
| `slow` | only `E1MarkerSetComparisonTests/testE1RunAll` | exactly 1 passed; 0 failed/skipped/expected-failed/restarted | slow lane |
| `subset` | caller-owned `-only-testing` selection | at least 1 passed; 0 failed/skipped/expected-failed/restarted | diagnostic, explicitly not a commit gate |
| `all` | `fast`, then `slow` | both lane receipts pass | **commit gate** |

Run `tools/run_tests.sh all` before committing. `fast`, `slow`, and `all` accept no caller
arguments; their fixed invocation is part of the reviewed receipt. `subset` is the only
selector-bearing mode. In particular, selecting E1 can no longer inherit the fast lane's E1
exclusion and report a zero-test run as green. Even `subset` rejects skips, retry/repetition
controls, and alternate test configurations: a later successful retry cannot erase a product
failure from the evidence.

**Do not hand-type an `xcodebuild test` line.** The script exists because typed lines named the
simulator by NAME, and that is what made this suite untrustworthy — see
[the commit gate](#the-commit-gate-what-green-means-and-what-it-does-not-2026-08-08). It provisions
a private device, refuses to start if another run holds it, and writes a unique xcresult receipt.
Missing evidence is failure: each lane requires `xcodebuild` rc 0, `TEST SUCCEEDED`, the exact
reviewed count, zero failures, zero skips, zero expected failures, zero restarts, and a readable
xcresult summary. A missing or unparsable result bundle fails the lane.

**Wall time roughly doubled on 2026-08-08** when cylinder path wrapping landed: the last
408-test run took 748 s, the first 438-test run took **1,627 s**. It is the Debug build, not
the algorithm — the same solver costs 58.0 us per one-wrap solve at -O0 and 0.268 us at -O2.
The suites that pay are the ones that drive `computeMomentArms` in a loop
(`GaitDynamicsTests`, `StraightLinePathErrorTests`, `CylinderWrapValidationTests`).

Historical per-class inventory for that 408-test receipt (2026-08-08; the later
30-test wrapping increment and all subsequent tests are intentionally not represented):
GaitLoadSummaryTests 39 · PostureFindingsTests 26 ·
NimbleBridgeTests 22 · GaitClipFixtureTests / GaitReportTests 20 · StaticHoldTests 19 ·
TRCExporterTests 14 · IKConvergenceTests / MuscleSolverTests 13 · MomentArmTests 12 ·
BodyPlausibilityTests / CalibrationTests / GaitDynamicsTests / NativeWindowSamplingTests 11 ·
MomentArmErrorCancellationTests / MotionRecorderTests 10 · BodyJointTests / DOFMaskTests /
DerivativeWindowTests / GaitStanceDetectionTests 9 · GaitContactClaimTests /
OfflineDisclosureTests / IKSolverInternalsTests 8 · StaticEquilibriumBenchmarkTests 7 ·
GaitLoadStatisticTests / IKDriftDiagnosticsTests / MuscleQPUnitsTests / OpenSimReferenceTests /
RootTranslationTests 6 · ClaimSurfaceTests / MuscleOverlayClaimTests / ShoulderRotMaskTests 5 ·
DecodedFrameMemoryTests / GaitConstantSensitivityTests / GaitContactAgreementTests /
PostureFindingsRealPoseTests / StraightLinePathErrorTests 4 ·
BodyFrameOrientationTests / GaitClaimSurvivalTests / PersonBoxTests /
ShoulderRotObservabilityTests 3 · OfflineMuscleChainTests / OfflineOrchestrationTests 1.

Six suites carry 92% of the 747 s wall clock: GaitDynamicsTests 369 s · IKConvergenceTests 91 s ·
ShoulderRotMaskTests 51 s · StaticHoldTests 48 s · MuscleQPUnitsTests 41 s ·
GaitContactClaimTests 39 s · IKSolverInternalsTests 33 s · StraightLinePathErrorTests 31 s. That
distribution matters for reading a crash report — see below.

### The commit gate: what green means, and what it does not (2026-08-08)

Three reviewers ran "the suite" on 2026-08-07 and got three different answers, so "the suite is
green" carried no information. Both causes were mechanical and neither was in the tests.

**Cause 1 — the shared simulator.** Every documented invocation named the device by NAME
(`name=iPhone 17` here, `name=iPhone 17 Pro` in `README.md`), so two `xcodebuild test` processes
resolved to one UDID and evicted each other's test host. Measured, same binary, same tests, same
device, one variable:

| | last `Executed` line | restarts | verdict |
|---|---|---|---|
| two processes sharing one UDID (proc A) | `Executed 2 tests, with 0 failures (0 unexpected)` | 5 | `** TEST FAILED **` |
| two processes sharing one UDID (proc B) | `Executed 2 tests, with 0 failures (0 unexpected)` | 5 | `** TEST FAILED **` |
| one process, private UDID | `Executed 19 tests, with 0 failures (0 unexpected)` | 0 | `** TEST SUCCEEDED **` |

**Cause 2 — `Executed N tests, with 0 failures` is not a verdict.** Look at the table: the killed
runs print exactly that line, with zero failures, having run 2 of 19 tests. A killed host reports
its lost tests as neither passed nor failed. The current gate therefore requires independent
process, log, and structured evidence: zero `xcodebuild` rc, trailing `** TEST SUCCEEDED **`, zero
restart count, and a readable xcresult summary with the lane's exact count and zero
failed/skipped/expected-failure tests. Any missing evidence fails closed.

**The `DecodedFrameMemoryTests` attribution did not survive.** The finding was that its ~250 MB
hold destabilises the rest of the suite. On a private device the full suite **including** it ran
green twice before any change — `Executed 353 tests, with 0 failures`, `** TEST SUCCEEDED **`,
0 restarts, 692 s and 678 s. Sampling the test host's RSS once a second across a whole run
(n = 626): median **463 MB**, peak **721 MB**, and the peak is indeed that test — but the engine
suites reach **600 MB** on their own, so it lifts the suite's high-water mark by 121 MB, to 2.1% of
this machine's 34.36 GB. The reported temporal adjacency ("the first kill is 1 s after it logs its
footprint") is what the schedule looks like anyway: `GaitDynamicsTests`, 369 s and 55% of the whole
suite, starts 0.35 s after `DecodedFrameMemoryTests` ends, and every named victim is one of the six
long suites above. Wall-clock exposure, not memory adjacency. The test is unchanged.

Residual, so it is not mistaken for closed: this is a 34 GB machine and no memory limit was in play.
On a much smaller host `DecodedFrameMemoryTests` is still the suite's peak, and the held-frame count
(`held = 60`, whose footprint check measured expected 248.7 MB vs observed 251.6 MB, ratio 1.012) is
the lever — the assertion is a ratio, so it survives a smaller hold.

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

### Trapezius / serratus geometry audit (2026-08-11)

The licence-clear 48 are not geometrically dead everywhere. At the fixed
`FullBody.osim` SHA-256 `0003473937af6883034df358194bd8f52853818e79e36fd23eb5ca2c8d741c09`,
OpenSim 4.6 found **59 unlocked rotational coordinates and 852 structural
muscle-coordinate pairs**. At both neutral and the committed `spine_flexed` pose, all **852/852**
had `|r| > 1e-6 m`; neutral min / median / max was **0.104 / 53.303 / 199.825 mm**, and
`spine_flexed` was **0.082 / 48.154 / 194.438 mm**. OpenSim's analytic result agreed with an
independent `-dL/dq` central difference with zero sign disagreements and maximum absolute error
`5.793e-8 m` / `4.717e-8 m` at the two poses.

That positive result has a hard boundary. The same 48 paths have **exactly zero** moment arm in all
**48 × 6 = 288** glenohumeral samples because the clavicle/scapula chain is welded. Only
`trap_cl`, `trap_acr_scap` and their left counterparts cross the three head/neck rotations, and
52 other muscles also cross that joint. Across a separate `-4° / 0° / +4°` sample all 2556 values
were non-zero, but 147/852 pairs changed sign. Therefore this is a sampled local geometry fact,
not evidence of activation, load, shoulder stabilisation, or an identifiable *"upper trapezius
overworking"* result. Spine coordinates remain priors rather than observations, and the static
optimisation remains underdetermined.

The permanent read-only receipt is
`tools/opensim_ref/audit_trapezius_serratus.py`; it also records the 28 trapezius / 20 serratus
partition, the head-neck / thoracic / rib / sternum groups, and the absence of shoulder action.
The existing `opensim_moment_arms.txt` regression fixture contains none of these 48 muscles, so this
manual OpenSim audit is not represented as an iOS runtime gate.

---

## Model facts (verified — do not re-derive)

`BioMotion/Resources/FullBody.osim` (production, `cyclistFullBodyMuscleActuated`):

- **169 XML coordinates → 169 DOFs parsed**; ~127 are spine + rib. It was 171 → 163 before the
  2026-08-06 `tools/osim_fixes` edit: the patellofemoral weld removed the two `knee_angle_*_beta`
  coordinates from the XML, and the shoulder axis unit-snap stopped nimble dropping 8 (2
  patellofemoral + 6 shoulder). Nothing is dropped any more.
  ⚠️ `CLAUDE.md` said 171 until 2026-08-07; anything quoting 171 or 163 predates the osim edit.
- 520 muscles = 422 `Thelen2003Muscle` + 98 `Millard2012EquilibriumMuscle`.
- `PathPoint` 1444 · `ConditionalPathPoint` 418 · `MovingPathPoint` 4
  (**runtime: 4 parsed, 0 approximated, 0 skipped**) · `PathWrap` 76 ·
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
data**. At the time of this diagnosis, nimble's `first3Linear` fast path (which tries to recognise a
disguised EulerJoint) demanded `|dot| < 1e-4`, hit a Release-disabled `assert()`, fell through with
`joint == nullptr`, and BioMotion's then-local crash workaround substituted a `WeldJoint`. Of all
53 CustomJoints, **only these two** triggered that historical path. The model's later unit-snap
removed the trigger; the 2026-08-11 parser patch also deleted the substitution and now rejects such
unsupported topology transactionally instead of returning a different joint.

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
- The earlier double-support analysis was already a counterexample under a *more favourable* model:
  if each foot had a valid support polygon, `F_L·x_L^cop + F_R·x_R^cop = mg·x_com` would still
  leave the split underdetermined (historical bound: **±18 pp** with perfect CoM). The shipping
  model/solver pair is less constrained than that hypothetical calculation: both contact-geometry
  sets are empty, and the near-CoP routine enforces no support polygon, unilateral force, or friction
  cone at all. Its 50/50 seed and returned wrenches are raw diagnostics, not an instrument. No L/R
  or total GRF is reported today. Single-leg stance does not repair the absent support model.

**What is currently defensibly quantitative:** joint angles in observable planes, plumb-line
deviation, hold duration/sway, kinematics-only posture findings, and resolution-qualified contact
timing. Joint moments could become a defensible muscle-adjacent number only after GRF/contact
support is independently validated; neither bundled model meets that prerequisite.

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
three legacy constant-relative `scaleModelWithHeight` factors inside the `[0.7, 1.4]` clamp. Its
recorded factors (0.957 / 0.989 / 1.014) independently matched the PyTorch-side measurement
(0.954 / 0.984 / 1.017), but those denominators were not the loaded FullBody model and are now a
historical export check, not the shipping ratios. The bridge now caches FullBody's actual
lower/trunk/upper references (0.8061 / 0.4820 / 0.5360 m); the shipping dancer regression records
1.044 / 1.094 / 0.997 and converges at 2.4586 cm marker RMS.

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
   muscle overlay drawn on top of it. Fixed by `routeSolveToOwningFrame()`, which matches on
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

### Muscle rendering: rank, never threshold (build 22) — SUPERSEDED 2026-08-08

⚠️ **The conclusion of this section shipped a cross-muscle claim, and the renderer it describes no
longer exists.** Everything below about the DISTRIBUTION is still true and still worth reading — it
is the measurement that shows the activations are degenerate. What is retired is the fix: "render
only the strongest 24" solved the flicker by ranking muscles against each other, which is a
comparison this model cannot support (see
[Sixth round](#sixth-round-the-same-claim-in-colour-2026-08-08)). The overlay draws a fixed
anatomical set in one constant colour and takes no activation input, so neither the flicker nor the
threshold question arises.

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

> **Historical raw-diagnostic scope (added 2026-08-10):** the values in this section verify gravity,
> coordinate frames, and statics algebra inside the old unconstrained contact solve. They do not
> validate the bundled models' empty foot contact geometry and are not publishable joint torque,
> GRF, or CoP measurements. The production path now refuses them.

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
| dancer fixture (legacy PELVIS mapping) | **0.3545** | 0.6406 |

Standing is under the 0.3 line `MuscleSolver.h` documents; the two standing figures did not move
because their IK was already fitting to 0.1 mm.

**The dancer's 0.6406 → 0.3545 is the IK fix, not a muscle-side change.** The 2026-08-06 text said
"no row cut moves it below ~0.60 — because that pose is wrong before the muscle solver sees it", and
that diagnosis was right: the same fixture now solves to 2.1224 cm true marker RMS instead of
5.4913, reproducibly, and the muscle residual almost halved with the muscle solver untouched. Nothing
in `MuscleSolver` or `MomentArmComputer` changed between those two columns.

⚠️ The paragraph above is the dated 2026-08-07 PELVIS-mapping result. The root-definition defect is
resolved on 2026-08-10 by the source-specific `MHR_ROOT` contract. Current unscaled dancer marker
RMS is **1.5365 cm**, but its shipped relative torque residual is **0.5939547**, not 0.3545. Better
marker fit did not make this oblique real pose a controlled muscle benchmark; the two standing poses
remain the muscle stage's clean benchmarks. See the MHR_ROOT section for the source/proxy distinction.

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

Resolution ladder, first hit wins: bundled `.mlmodelc` → pack `.mlmodelc` → start the download and
**throw immediately**. The runtime no longer accepts a raw `.mlpackage`, imports the Core ML compiler,
or owns an Application Support compile cache/source-mtime stamp. It never blocks on the transfer. The
bundled branch exists so the Simulator and local iteration need no download; the
`tools/assetpack/dev_bundle_model.sh on|off` helper installs only a receipt-verified precompiled
directory from the canonical AAR/receipt pair.

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

**The whole-image fallback is not a single-frame pose-quality cliff** — it scores 4.7% against the
real box's 4.6%, so it does not need a carry-the-previous-frame's-box mechanism. That result did
not license using fallback frames as if Vision had observed a continuous subject in a VIDEO:
22/309 slots on `video_012` take this path and their scale/root values are discontinuous. The pose
now stays visible for review, a PHOTO fallback remains analysable, and a VIDEO fallback is excluded
before scale, derivatives, gait, ID, or muscle. See the temporal-isolation section below.

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

**`joint_coords` pins raw MHR joint 1 at a model constant `(0, 0.924, 0)` in every frame.**
`global_trans` is zeroed (`sam3d_body.py:1600`), so that number is not the subject's global root
position and `y = 0` is
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
[Gait-cycle dynamics](#gait-cycle-dynamics-the-route-survives-its-evidence-did-not-2026-08-07),
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

### Gait-cycle dynamics: the route survives, ITS EVIDENCE DID NOT (2026-08-07)

**Read this before trusting any gait number anywhere in this file or in git history.**

#### The reasoning, which still stands

The earlier "muscle force is unobtainable on a tracking shot" conclusion was wrong at the framing.
It assumed the root's acceleration must be MEASURED. Two facts say otherwise:

1. **Joint angles are invariant to camera motion.** Translation shifts every reconstructed point
   together; rotation preserves lengths and included angles. Only the 6 root DOFs are affected by a
   tracking shot — exactly the channel we were trying to differentiate.
2. **Root acceleration enters as a uniform pseudo-gravity.** Every segment picks up `m·a_root`,
   indistinguishable from changing g. ⚠️ This is still an ARGUMENT, not a measurement: a comment
   claiming `GaitRootAccelerationTests` had verified it on the real 169-DOF model was found to cite
   a test that does not exist.

For running, `a_root` then comes from the gait cycle rather than from differentiation: flight is
free fall, and stance is closed by the stride's vertical impulse `m·g·T` being delivered during
contact alone. `Fmax = m·g·(π/2)(1 + tf/tc)` is that statement for a half-sine stance force.

#### The measurements were an artefact of a bug in the probe

`export/vision_box_probe.swift` wrote frames as `String(format: "t%05.1f.png", t)` — a filename
quantised to 0.1 s. At a 1/30 s sampling step three consecutive records therefore addressed ONE
file. **Verified: 41 distinct PNGs for 120 requested frames.** Everything downstream read a 10 Hz
staircase wearing a 30 fps label. The tool was written for a 6-timestamp `upperBodyOnly` probe,
where 0.1 s was ample, and reused at 30 fps without re-checking.

Both flattering results came from that staircase:

- **"contact 200 ms, zero spread, 13/13 contacts exactly 6 frames, identical across a 2.5× threshold
  span" was the quantisation itself.** Contact duration could only be a multiple of 3 frames. That
  was the single strongest piece of evidence for the route and it measured the bug.
- **The knee-angle spectrum's "no Winter knee, unexplained high-frequency content, an exact null at
  10 Hz"** was the staircase's spectral image — 10 Hz being 30/3.

#### What correct extraction gives (`export/frame_probe.swift` + `gait_cache.py`)

| clip | contact L / R | stride L / R | status |
|---|---|---|---|
| video_012 | 167 / 167 ms | 606 / 606 ms | measured |
| video_013 | 200 / 121 ms | 593 / 415 ms | **refused** by the input gate — Vision lost 3 frames |
| video_015 | 233 / 247 ms | 647 / 628 ms | measured |

Peak vertical GRF 2.85 / 2.08 BW for 012 / 015. Still physiological, and still different per clip —
so the route is **not refuted**. But:

- **Precision is much worse than reported.** The threshold sweep over 0.08–0.20 gives
  `video_012` [2.356, 4.560] BW and `video_015` [1.799, 3.421] BW: **±32%** about the arithmetic
  midpoint. The "±39%" quoted elsewhere is the geometric half-width; the "±17%" from the artefact
  era is simply wrong.
- **The robustness evidence is gone.** There is currently NO measurement showing the detector is
  insensitive to its threshold, because the only one we had was the staircase.

#### Two structural problems, independent of the artefact and of any implementation

1. **The Savitzky-Golay window is longer than a contact.** 9 taps centred spans `8·dt` = 267 ms at
   30 fps, against measured contacts of 167–247 ms. No stance frame has a window free of a
   touchdown/toe-off discontinuity — 0 of 114 interior frames on `video_012`. A withhold rule that
   drops only the first and last frame of each contact does not address this.
2. **That filter's second-derivative gain is 0.49 at the step fundamental and inverts sign above
   7 Hz.** For `accCoeffs = [28,7,-8,-17,-20,-17,-8,7,28]/462`, `H(ω)/(-ω²)` at `dt = 1/30` reads
   0.941 @ 1 Hz, 0.560 @ 3, 0.489 @ 3.30, 0.140 @ 5, 0.016 @ 6, −0.039 @ 7. The articulated inertial
   term the route depends on would be halved.

Neither is fatal, but neither has a proposed fix, and the offline path has never fed this filter at
30 fps before.

#### Status of the implementation: NOT SHIPPED

A workflow built it; two of its stages died on transport errors mid-edit and three independent
adversarial lenses returned BROKEN with 8 blockers. It is preserved on branch
`wip/gait-dynamics-broken` and is NOT on main. `main` is back at 219 passing tests.

What that review found, worth keeping whoever picks this up:

- The app target did not compile (`runGaitWindow` called, never defined), so **none** of the ten
  pre-registered gates was ever evaluated, and no gait test exists.
- The pinned clip fixture traps on its first line: the generator wrote numpy reprs
  (`np.float64(3.0)`) into a CSV parsed with `Double(f[0])!`.
- Horizontal root acceleration was **forced to zero** into the ID rather than left unmodelled,
  injecting an undisclosed 0.2–0.35 BW error into every joint moment.
- Three of eleven "gait condition receipts" had `passed` hard-coded to `true`.
- A stance run already in progress when the window opened was counted as a touchdown, which is the
  entire cause of one clip's registered failure.
- Nothing read the trust flags: `forceTrustworthy`, the threshold band and every force number were
  computed into a struct with no consumer, and `.nativeWindow` was unreachable from the UI.

#### The process failure that let a broken tree reach main

`main` was broken from commit `5e9b370` and it was not noticed for three commits. Cause: a `git
add -A` for a documentation commit swept in an in-flight edit from a live agent working in the SAME
working tree, so what was committed was not what had been tested. Restored by resetting
`NimbleEngine.swift` to `2dba6a7`, its last state that actually passed the suite.
**Do not edit or commit files while a resumed agent may still hold them.**

### Gait consistency re-measured for RELATIVE use — the limit is frames per contact (2026-08-07)

The owner set the bar: the product is posture and force CORRECTION, so relative values suffice and
absolute newtons are not the deliverable. That is a materially weaker requirement, because a
peak-GRF error is a COMMON SCALE over every muscle in a contact and the muscle QP is linear while
unsaturated — muscle-to-muscle ratios survive it exactly. (Ratio preservation degrades where
muscles hit `a <= 1`, which running peaks do reach.)

So the gates were re-registered around invariance rather than accuracy, and run on the CORRECTLY
extracted trajectories. Harness: `labs/sam-3d-body/export/gait_consistency.py`.

| clip | W: window shift | T: L/R ratio vs threshold | S: resolvable asymmetry |
|---|---|---|---|
| video_012 | left **FAIL** (1.40 frames), right pass | **FAIL** (0.307) | ~16% |
| video_013 | pass | **FAIL** (0.401) | ~44% |
| video_015 | pass | **FAIL** (0.261) | **~6%** |

**T failed on all three as registered, and that is recorded as a failure, not argued away.** The
post-hoc structure — every clip's outlier is `frac = 0.08`, and the ratio is stable to 0.05-0.10
over 0.12-0.25 — is a HYPOTHESIS formed on the same data, so it cannot also be its own
confirmation. What it does establish is that "a percentage of the ankle's vertical range" has no
physical meaning and is therefore bound to depend on an arbitrary constant: at 0.08 `video_012`
reads the left contact 24% SHORTER, at 0.25 it reads 7% LONGER. The sign of the asymmetry flips
with the constant.

A physically-defined replacement was tried — stance as the interval where the pelvis-relative
horizontal foot velocity sits at its most-negative plateau, which physically IS the running speed,
with the plateau level measured from the signal rather than assumed. It improved W (6/6 legs pass)
but not T (only `video_015` passes, 0.067) and made S worse on the two fast clips.

**Neither criterion is the problem. Frames per contact is.**

| | frames/contact | measured resolution | quantisation floor `0.5/N` |
|---|---|---|---|
| video_015 | 7 | 6% | 7% |
| video_012 | 5 | 16% | 10% |
| video_013 | 5 | 44% | 10% |

`corr = 0.73`, mean absolute error 15 pp — so quantisation is a FLOOR that explains the best case,
and the two fast clips are worse than the floor for reasons not yet isolated (`video_013` also has
dropped frames and is refused by the input gate anyway). `video_015` sits ON the floor: it has the
longest contact (233-247 ms) and the lowest cadence of the three.

**What this makes quantitative.** For a 200 ms contact, resolvable asymmetry is about
8.3% at 30 fps, 4.2% at 60, 2.1% at 120, 1.0% at 240. Clinically discussed running asymmetries are
often 5-15%, so 30 fps imported footage sits right at the edge of usefulness and a faster capture
path is the only lever that moves it — not a better detector, not a better model.

**Design consequence, decided:** ship the imported-footage path and have the app COMPUTE and DISPLAY
its own per-clip resolution from the measured frames-per-contact ("this clip resolves left/right to
about +/-8%"), refusing asymmetry claims finer than that. That turns the limit into a per-clip,
honest, actionable number instead of a global disclaimer, and it tells the user exactly when
filming at a higher frame rate would help. An in-app high-frame-rate capture path is deferred, not
rejected: it is the known lever if the displayed resolution proves too coarse in use.

### Adversarial review of the shipped gait layer — 3 blockers, 11 majors, all addressed (2026-08-08)

Three independent lenses read the gait implementation. Everything below is a defect they found and
what was done about it; the numbers are re-measured on the pinned fixtures, and every fix is a test.

**B1 — the declared falsifier could not disagree with the force model.** `measuredFlightSeconds` and
`modelledFlightSeconds` are two linear combinations of the SAME touchdown indices: for touchdowns
`L: nT`, `R: nT+s` with contacts `cL`, `cR` the observed gaps are `s−cL` and `T−s−cR`, whose mean is
exactly `(T−cL−cR)/2` — the modelled flight, identically, for every `s`. So it is 0 by algebra on
any periodic alternating schedule while `Fmax = (π/2)(1+tf/tc)` sweeps a factor of 6. The code
nevertheless named it as the check covering the frame residual's admitted blindness to shape and
peak, so **the peak — the common scale every muscle number rides on — had no falsifier at all**.
Renamed to `contactSequencePeriodicityErrorFrames`, documented as what it is, and the identity is
now asserted (`testThePeriodicityCheckIsAnIdentityAndCannotSeeTheForce`).

The real answer to the falsifiability requirement, and the (a)/(b) choice: **(a) for the mechanism,
because (b) is not available on this pose source.** `MHRRetarget` pins the raw source root, so measured
`a_root ≡ 0` and a plain-ID root residual would read `‖m·a_artic − m·g − F_gait‖ ≈ 3.9·m·g` on every
stance frame of every clip — the same failure on good and bad footage alike, i.e. a constant, not a
falsifier. At this historical stage, (b) was treated as available once `cam_t` was composed in AND
its depth became usable (3.11 g of noise at 30 fps). The 2026-08-10 contact audit supersedes that:
`cam_t` can repair kinematics but cannot create the missing support model, so dynamics remains
unavailable. Three quantities were then made to carry the burden, gated together through
`GaitLoadSummary.arePublishable`: **(1)** `‖a_artic‖/g` over frames both contact detectors agree on;
**(2)** the ID solver's geometric contact detector against the kinematic one; **(3)** per-muscle
saturation, which is exactly and only where a peak-force error stops cancelling out of a ratio. The
peak's magnitude is still untested and that is now said on screen.

**B2 — a dropped frame silently shortened a contact.** `StanceInterval.frames` was
`lastIndex − firstIndex + 1`, a SAMPLE count, converted to a duration by `× dt`. Vision loses ~7.1%
of frames (STATUS Finding 6). Measured on the fixture: `video_013`'s second left contact counts 4
samples (133.3 ms) and spans 200.0 ms on the clock. Monte Carlo at 7.1%, 400 trials: sample counting
fabricates up to **23.4%** (`video_012`, truth 2.9%) and **29.1%** (`video_015`, truth 0.5%) of
left/right asymmetry. Duration now comes off the clock (`lastStanceSample − touchdown + dt`), which
fixes interior holes exactly — `video_012` and `video_015` are byte-identical, `video_013` moves
150.0 → 161.1 ms. That is NOT enough on its own: an EDGE hole moves the retained edge inward by a
real sampling interval and the same Monte Carlo still reaches 19.4% / 17.0%. So a contact with a
hole inside it or against either edge is now a REFUSAL (`.droppedSamplesInContact`), not a flag.
`makePlan` also lays entries on the contact's own sample timestamps instead of `touchdown + k·dt`,
so a hole can no longer push later samples out of the ±dt/2 match window and have them solved as
flight with `a_root = −g` while the foot is planted.

**B3 — the loads rendered outside their own gate.** `loadBlock` drew every muscle's L/R peak and
bars unconditionally while `honestyBlock`, three lines below, printed "FAILED, loads withheld"; only
the per-muscle sentence was gated. `OfflineSceneView` likewise drew the 3-D muscle overlay from
`frame.muscleResult` with no gate. Both now ask `summary.arePublishable`, and the withheld case
states the measurement and a lever.

**Majors, each fixed:**

* `strideRepeatabilityPercent` was fed the CV of CONTACT DURATIONS, not of stride periods, while
  being named, documented and displayed as "this runner's own stride-to-stride variation" — and it
  gated the camera advice. `video_015`: 11.14% published against a real stride CoV of 2.08%/2.56%;
  most of that 11% was the quantisation counted twice, so the app told the user a faster camera
  could not help on the best of the three clips. Now fed the stride CoV. Published resolution:
  `video_015` 11.14 → **8.09%**, `video_013` 43.28 → **18.91%**, `video_012` unchanged at 10.15%.
* One clip-wide `Fmax` was applied to both legs, setting the timing model's own left/right peak
  asymmetry to zero by construction and running the wrong way round (a shorter contact must carry a
  HIGHER peak). Now per leg, `Fmax_side = (π/2)(1 + tf/tc_side)`, and the stride impulse still
  closes exactly (`Σ Fᵢ·2tcᵢ/π = T`, asserted). Cost of the old form on an asymmetric runner
  (200/160 ms, tf 130 ms): 2.7053 BW on both feet against 2.5918 / 2.8471 — a **9.4%** peak
  asymmetry discarded. On the owner's clips it is small (−1.31%, +0.20%), which is why nothing
  caught it.
* The residual gate conflated two failures. When the ID solver's geometric detector sees no foot
  down, `solveIDGRF` returns ZERO ground force, so the residual is the entire modelled force
  (~2-3 BW) and has nothing to do with limb inertia — one such frame withheld a whole clip under a
  sentence about inertia. The residual statistic is now computed over agreeing frames only, the
  disagreements are their own gate with their own number and lever, and those frames' activations no
  longer enter the peaks (they were solved without the load they are supposed to describe).
* The per-clip derivative window was sized from the SHORTEST contact, which picks 3 taps on
  `video_012`. Exact white-noise gain of the second-derivative coefficients: 9 taps 0.113961,
  7 taps 0.218218 (1.91×), 5 taps 0.534522 (4.69×), 3 taps 2.449490 (**21.49×**) — and at 3 taps the
  POSITION coefficients are `[0,1,0]`, so the "smoothed" pose the moment-arm stage consumes is raw
  IK. That noise is independent per DOF and therefore does NOT cancel out of a ratio, which is the
  one error the product is allowed to tolerate. Now sized from the MEDIAN contact (5 taps on both
  usable clips, 4.69×), with the frames whose window crosses a contact edge marked per frame and
  excluded from the load summary — measured clean frames: `video_012` 12 of 64, `video_015` 24 of
  68. A clip whose median contact is under 5 samples is refused rather than differentiated.
* The one action the product recommends made the analysis refuse. `maxFramesPerRun = 120` is a FRAME
  budget, so the native-rate window collapsed to 1.983 s at 60 fps, 0.992 s at 120 and 0.496 s at
  240 — below three complete contacts per side. `.nativeWindow` now has its own budget, DERIVED as
  `minimumAnalysisSeconds × 240 + 1 = 601`, holding ≥ 2.50 s at every rate up to 240 fps. 30 fps is
  unchanged at 120 frames. Cost at 240 fps is 7-10 min of pose model at the ~1 s/frame the brief
  quotes — estimated, not measured on device.
* The advice also promised a resolution the model said it could not deliver: it printed
  "filming at 61 fps would resolve ±5%" while `resolvableAsymmetryPercent = max(floor, repeatability)`
  would have been 7.2%. The target is now never finer than `bestAchievablePercentAtAnyFrameRate`,
  and the rate is never above `FrameSource.highestAnalysableFrameRate`.

**What this did NOT establish.** The whole chain has never been run end to end on the owner's real
clips: the 1.3 GiB pose model ships as a Background Asset and is not in the test bundle, and the
pinned fixtures carry only 5 joints — enough for the gait module, not for IK. So the new contact
gate's real pass rate on `video_012`/`video_015` is **unmeasured**, and it may withhold. The only
end-to-end number is synthetic: 9 stance frames, 5 contact-detector disagreements, 3 frames with a
clean derivative window, agreeing residual 0.008-0.183 BW against the 0.5 gate.

### Second review round: the two comparison-corrupting blockers, and the phone's memory (2026-08-08)

Three lenses re-read the shipped gait layer and returned 2 blockers and 10 majors. The three closed
here are the ones that corrupt a RATIO or terminate the app; the rest are the next stage's. Test
count 325 → **336**, all green.

**B1 — a double contact was invisible to the contact gate.** `contactDetectorsAgree` asked only
whether the ID solver also saw the CLAIMED foot down; it never asked about the other one. When
`solveIDGRF`'s geometric detector sees BOTH feet, it hands
`getMultipleContactInverseDynamicsNearCoP` two wrench guesses of `weightUp / 2`, and that solver is
a least-squares fit around the guess whose steps lie in the constraint null space — so the split
survives and the stance leg is solved with **half** its real ground force while the swing leg gets a
ground reaction that does not exist. Nothing downstream could see it: `residualInBodyWeights` is
built from `leftFootForce.y + rightFootForce.y`, the SUM, and the near-CoP constraint fixes the sum
exactly, so 50/50 and 100/0 produce the identical residual. The error lives entirely in the ratio.
Measured through the shipping `GaitLoadSummary`: six left-stance frames of a symmetric runner
(0.60/0.60) solved as double contact publish −66.7% (`differencePercent` normalises by the MEAN of
the two peaks, so halving one leg is worse than −50%) against a resolution the same screen states as
10.1%. The gate now requires the other foot to be UP, those frames count as disagreements, and
nothing publishes.

*Was the 6 cm threshold also wrong? Measured, and it is left alone.* The reviewer's hypothesis was
that `calcn_y − groundHeightY < 0.06` sits inside the pelvis's own bounce on a pelvis-pinned stream.
`GaitContactAgreementTests` drives the SHIPPED estimator (`observeLowestFootHeightY` →
`groundHeightY`) and the shipped threshold (now exposed as
`NimbleBridge.contactDetectionThresholdMeters`) over all three pinned clips:

| clip | ankle proxy (double/single/none) | min(ankle,toe) proxy | min swing clearance |
|---|---|---|---|
| video_012 | 0 / 58 / 64 | **1** / 59 / 62 | 0.106 m / 0.193 m |
| video_013 | 0 / 56 / 63 | 0 / 48 / 71 | 0.150 m / 0.094 m |
| video_015 | 0 / 67 / 55 | 0 / 86 / 36 | 0.326 m / 0.125 m |

So the gate is **load-bearing, not theoretical** — one frame of `video_012` does double up, which the
earlier ankle-only read had missed. But the swing foot's clearance on genuine single-stance frames is
0.094-0.326 m, 1.6-5.4× the threshold, so there is no margin problem: one frame in 122 is the gate's
job, not the constant's, and moving it on this evidence would be tuning it. ⚠️ The fixture carries 5
joints and cannot drive IK, so `calcn` is proxied by the ankle joint centre; both proxies are
reported and neither changes the conclusion.

**B2 — the published activations were not reproducible, and a test PINNED that.**
`MuscleSolver._prevActivations` warm-starts OSQP and was re-initialised only when the muscle COUNT
changed; `NimbleEngine.resetSessionState()` reset the bridge, the filters and the hold detector but
never the muscle solver. Two byte-identical static-hold runs measured **0.836** apart on the worst
muscle, **181.5 N** on the worst muscle's force and **1432 N** of total muscle force — and
`testAMotionlessSubject…` asserted `XCTAssertGreaterThan(controlActivation, 0.01)`, i.e. it required
the non-reproducibility to be there and documented it as QP null-space behaviour. It is not: the ID
torques were bit-identical (0.0 Nm) on the same runs, so the QP was starting from a different place,
not landing in a different place. Inside a gait pass it is worse than random — stance frames
alternate left, right, left, right, so every solve warm-started from the OPPOSITE leg's answer, which
is exactly the comparison being published.

`MuscleSolver.resetSessionState` now drops three things, and the third is the one that is easy to
miss: the primal warm start, **the OSQP workspace itself** (`warm_starting = true` and the caller
passes `OSQP_NULL` for `y`, so the DUAL iterate carried across clips and nothing else could clear
it — the workspace is torn down and rebuilt, one KKT factorization per clip), and `_prevMuscleLengths`
(the fiber-velocity fallback would otherwise difference clip B's first frame against clip A's last
pose). The assertion is inverted rather than deleted, and that inversion is the proof:

| | before | after |
|---|---|---|
| worst activation, two identical runs | 0.8365 | **0.0** |
| worst per-muscle force | 181.54 N | **0.0 N** |
| same clip, with/without another clip first | 0.9562 (DELT2) | **0.0** |
| ID torque | 0.0 Nm | 0.0 Nm |

The one quantity NOT held to bit-equality is the SUM of 520 forces (1432 N → 9.1e-12 N), because
`forces.values.reduce(0, +)` adds in Dictionary iteration order and can reassociate; 5e-15 relative
is the last bit of a double. The per-muscle assertions are the meaningful ones.

The engine-level proof needs the 520-muscle model and takes 3 minutes, so the same property is also
pinned on the single-muscle rig in `MuscleSolverTests` — solve, reset, solve again, and land where a
brand-new solver lands. Even on a one-variable QP the warm start matters: without the reset the
activation reads 0.20012 against 0.20000 and the torque residual 6.2e-3 Nm against 4.0e-7, a factor
of 15,500. It is not a null space, it is where OSQP stopped.

**M-mem — following the app's own advice OOMs the phone.** Every sampled frame is decoded to a
`UIImage` before any is processed and then retained by its `FrameResult` for the whole playback
session, so the peak is `frames × width × height × 4` paid in full alongside the 1.31 GiB model.
`resolutionSentence` says "film at 61 fps"; the user films at 60 and the native window grows 120 →
240 frames. Measured end to end on a synthetic 1080p 60 fps clip through the shipping decoder:
**1990.7 MB** before, **994.9 MB** after. `AVAssetImageGenerator.maximumSize` was never set, so there
was no downscale anywhere.

The budget is `maxFramesPerRun × 1920 × 1080 × 4` = 995,328,000 B — derived rather than chosen: it is
the decoded cost of the configuration that already ships, so the validated 30 fps path (and every
clip the product was built on: 576×1024 and 576×768, measured from the export caches) is decoded at
its natural size and **unchanged**, while 60/120/240 fps and 4K — which were 1.99/3.98/4.98/3.98 GB —
are all held to that same peak by scaling. 1080p decodes to 1356×762 at 60 fps, 960×540 at 120,
853×479 at 240. A phone jetsam limit is not measurable from this repo and is NOT what this number
came from.

Two things the measurement itself taught. The byte cost is **not** `pixels × 4`: a 1357×763 decode
reports `bytesPerRow = 5440` (1360 px), so rows pad to a 64-byte boundary and a "budget-sized" window
came out 0.08% over. The model now includes alignment (`FrameSource.decodedFrameBytes`), which also
costs portrait 1080p at 30 fps 8 px of long side (1080×4 = 4320 pads to 4352). And the accounting is
real, not notional: holding 60 frames moves the process's own `phys_footprint` by 253.7 MB against
248.7 MB predicted (ratio 1.02), and 503.0 vs 497.7 MB uncapped (1.01).

⚠️ **The cost, stated.** Above 30 fps the subject's pixel height falls, and the pose model warps the
padded person box to a fixed 512 px square. Measured on the owner's clips the box side is 360-711 px,
so the pipeline already both up- and downsamples there; at 120/240 fps it will always upsample. This
trades spatial resolution for temporal resolution, which is the trade this product's binding limit
calls for — but it is a trade, and `decodedWindowBudgetBytes` is one line if the owner wants it
elsewhere.


### Third round: the statistic that manufactured the finding, and what the screen claims (2026-08-08)

The seven remaining majors from the same review. Test count 336 → **353**, all green. Every fix below
was observed FAILING before it landed (each was temporarily reverted and re-run).

**M1 — the left/right comparison was a MAX over an unequal number of frames per side.** This is the
worst of the seven, because the inequality is produced by the very asymmetry being measured: with
`taps = 5` a contact of `n` samples yields exactly `n − 4` frames with a clean derivative window, so
a leg whose contacts run ONE frame longer contributes TWICE as many frames — and `E[max of n]` grows
with `n`. A symmetric runner therefore reads as asymmetric for a purely statistical reason.

MEASURED on the shipping path, 400 seeded synthetic clips, both legs drawn from the SAME
distribution (Normal(0.50, 0.12)), left contacts holding 6 usable frames and right 2:

| statistic | mean asymmetry | clips reading left-high |
|---|---|---|
| old: `max` over the side's frames | **+8.07 %** | 297 / 400 |
| new: mean over contacts of each contact's middle sample | **−0.19 %** (SE 0.47) | 192 / 400 |

+8.07 % is 80 % of `video_012`'s own publication floor (10.14 %). It does not have to clear the floor
on its own to do damage — it rides on the real effect, so a true 3 % difference publishes as 11 %
and a true 8 % right-high difference reads even and is refused.

The replacement is **one sample per contact — the middle of its usable frames — averaged over that
side's contacts**. Within a contact the count is fixed at one, so no extreme-value bias can enter;
across contacts the statistic is a mean, whose expectation equals the population mean for any
distribution and any sample count. The middle of the contact is where the modelled half-sine peaks,
so it is still "where the load concentrates" — the usable set is already the mid-stance core, because
`derivativeWindowInsideContact` keeps `k ∈ [taps/2, n−1−taps/2]`, symmetric about mid-stance.

⚠️ **The residual count-dependence, stated rather than hidden.** The middle sample sits at
`sin(π·φ)` of that contact's peak: 1.000 for an odd number of usable frames, 0.966 for an even one.
A leg with consistently even-length contacts and one with odd differ by **3.4 %** of force scale from
that parity alone. Bounded, deterministic, does not grow with the frame count, and asserted.
`leftPeak`/`rightPeak` are renamed `leftLoad`/`rightLoad` and `leftFrames`/`rightFrames` become
`leftContacts`/`rightContacts`, so no reader can mistake the new number for a maximum.

**M2 — per-leg `Fmax` re-injected the contact-time difference the same screen calls unresolvable.**
`Fmax_side = (π/2)(1 + tf/tc_side)` is a per-leg FORCE SCALE, and the QP is linear in the external
load while unsaturated, so it lands inside all 520 muscles' left/right comparisons. On `video_012`
the contact difference is 2.899 % against a 10.145 % floor: the panel says "left and right contact
times are even to within what this clip resolves" in one block and used to re-express that same
2.899 % as −1.31 % of muscle asymmetry in the next. Worst case is larger — 9 % of contact asymmetry
(just under a 10 % floor, correctly refused) injects ≈4 % of force scale, enough to publish a true
6.5 % muscle difference as "11 % harder on the left".

**Resolved in one direction: ONE gate governs both uses.** Above the clip's resolution the difference
is a finding, it is displayed, and each leg keeps its own peak (the 200/160 ms control still gives
2.5918 / 2.8471 and the shorter contact still carries the higher peak). Below it the difference is
noise, it is refused on screen, and both legs are closed on the MEAN contact — the minimum-variance
choice, not a claim that the legs are identical, and `peakVerticalForceIsSharedBetweenLegs` says
which regime is in force on the same screen as the bars. The stride impulse `Σ Fᵢ·2·tcᵢ/π = T` closes
exactly in both, asserted to 1e-12. **This deliberately replaces a registered expectation**: both
usable clips now read a single peak (`video_012` 2.8684, `video_015` 2.4578) where the pinned column
was 2.8499/2.8875 and 2.4602/2.4554.

**M3 — an empty residual set reported "gate passed" having measured nothing.** `sortedResiduals.last
?? 0` made max 0, median 0 and `residualGatePassed = true`, and the honesty block renders on every
`.analysed` clip — so a clip where no stance frame ever agreed printed "0.00 BW typical, 0.00 BW
worst (gate 0.50) — passed." in the calm secondary tint. `residualFrameCount` is published, the gate
is false when it is zero, and the line reads NOT MEASURED.

**M4 — the falsifier was labelled as the whole vector and measures one axis.**
`residualInBodyWeights` is `|ΣF_y − F_gait|/(m·g)`; `leftFootForce.x/.z` exist in the bridge's output
and are discarded. It was captioned "Limb inertia the timing model omits", i.e. `‖a_artic‖/g`, one
line below "braking and push-off are not modelled" — certifying a cancellation on the one axis it
never examined, where STATUS sizes the error at 0.2-0.35 BW against a measured 0.008-0.183. Renamed
`maxVerticalForceResidualInBodyWeights` throughout; the line now names the axis and says the fore-aft
term is not modelled and not seen. **No fix in `BioMotion/Nimble/**` — that path was not this
stage's to touch, and computing the horizontal residual remains open.**

**M5 — `strideRepeatabilityPercent` read exactly 0.000 and was published as a property of the
runner.** It is a CV over touchdown gaps quantised to whole frames; every gap on `video_012` is
exactly 18 samples. The panel printed "this runner's own stride-to-stride variation ±0%" on a clip
that cannot distinguish anything below `100/18 = 5.56 %` — a number the SAME report already carries
as `GaitSteadiness.boundPercent`. It is now floored at that bound, with the raw CV kept beside it.
Blast radius, measured: the published resolution moves on NONE of the three clips (10.145 / 18.909 /
8.086), because `50/framesPerContact` is 1.5-1.9× `100/stridePeriodFrames` on all three clips (18/(2·4.93), 19/(2·6.18), 18/(2·4.70)) and the ratio `stridePeriodFrames/(2·framesPerContact)` does not move with capture rate. What
changes is the sentence and the promise: "filming at 61 fps would resolve ±5%" becomes "filming at
55 fps would resolve ±6%", which is deliverable.

**M6 — per-frame exclusions reached the ranked list and not the overlay.** `GaitLoadSummary.make`
discards frames failing `isUsableForLoadComparison`; `OfflineSceneView` drew `frame.muscleResult`
gated only clip-wide. On the pinned fixtures that is 52 of 64 stance frames (`video_012`) and 44 of
68 (`video_015`) — the user scrubbed a clip that passed every gate and saw coloured capsules on
65-81 % of stance frames whose ddq was fitted across a touchdown, with no marker of any kind, while
the panel's list had silently dropped them. `FrameResult.gaitLoadsAreComparable` now gates the
overlay per frame, the caption reads "outside the load comparison", and
`gaitExclusionReason` names WHICH exclusion — the double contact, the ground-height disagreement and
the derivative window point at different levers. The still-pose path carries no gait outcome and is
untouched.

**M7 — the truncation banner was wrong in both halves for the case it most often fires on.** A 2 s
clip at 30 fps in the 4 s window: the sampler wants 120 frames, the clip has 60, `wasTruncated` is
true — and the banner said the clip was too LONG and that 120 frames were used, so the advice was
the opposite of the one that works. It also quoted `maxFramesPerRun` at 240 fps, where 601 frames
were used and 120 is not that mode's budget. `FrameBudgetNotice` decides the cause from the RESULT
rather than re-deriving the sampler's rules — if the run used every frame the clip has, the clip was
the limit — and states the real count and span either way.

**What this stage did NOT do.** No device run; everything is the iPhone 17 simulator. The horizontal
residual is still not computed (it needs `BioMotion/Nimble/**`). The double-contact and warm-start
blockers were closed in the previous round and re-verified here only by their existing tests.


### Fourth round: the muscle claim is scoped to LEFT/RIGHT, and the ranking is retired (2026-08-08)

The blocker and twelve majors from the fourth review. Test count 353 → **371**, all green via
`tools/run_tests.sh` on the private `BioMotion-CI` device, three consecutive whole-suite runs:
`Executed 371 tests, with 0 failures (0 unexpected)` × 3, **0 restarts** × 3,
`** TEST SUCCEEDED **` × 3, 677 / 683 / 683 s wall. The floor in `run_tests.sh` is raised to 371.

#### The blocker: a named per-muscle RANKING was put in front of the user on moment arms the engine
itself logs as wrong

`MomentArmComputer` prints at load that **76 `PathWrap` references on 66 muscles are not modelled** —
those paths cut straight through bone where the real tendon wraps, so their `L_MT` and moment arms
are wrong, worst at flexed poses, and running is a flexed-pose activity. Parsing the shipped
`FullBody.osim` says which muscles: glmax1/2/3, recfem, the vasti, gasmed, gaslat, the hamstrings,
psoas, iliacus, all four adductor-magnus heads, addlong, addbrev and grac — essentially every entry
in `GaitLoadSummary.displayNames`, i.e. the ones guaranteed to fill the top eight. The warning went
to the Xcode console; the user never saw it.

**The resolution, verified before it was relied on.** `MomentArmErrorCancellationTests` drives the
shipping OSQP solver on a bilateral rig whose analytic answer is known: every right torque is 0.8×
its left counterpart, so with no bound active the QP is linear in `τ` and EVERY muscle must read
`22.222 %` left-high. Measured:

| perturbation | worst shift in a left/right figure | cross-muscle key | list order |
|---|---|---|---|
| none (analytic 22.222 %) | **1.52 pp** — this is OSQP's own tolerance | — | — |
| `gamma` moment arm × 0.6, **both sides** | **1.04 pp** (below the solver's noise) | **+80.9 %** | reorders |
| `gamma` moment arm × 0.6, **left only** | **23.8 pp** | — | — |

So a per-muscle moment-arm error is a per-muscle SCALE: it cancels out of that muscle's left/right
ratio to below what the solver can resolve, and it moves the cross-muscle quantity by 78× as much
while changing the published order. The left/right comparison stays and is the product; **the
ranking is gone.**

Concretely: `GaitLoadSummary.ranked` → `muscles`, ordered by how far each left/right claim clears its
own floor (publishable claims first) — a key built only from a muscle's own two numbers, so it is
invariant to that muscle's scale for the same reason `differencePercent` is. `MuscleLoad.load` (the
clipped `max(left, right)` the old sort used) is deleted. Each row's two bars are now drawn to THAT
ROW's own scale, so no bar length is comparable to the row above, and `crossMuscleSentence` says both
things in words above the list, naming how many of the muscles shown are affected.

**What the sign-flip case actually does — this was not what the first draft assumed.** The loader's
warning says the sign can flip. A sign-flipped moment arm does not rescale the muscle: the QP refuses
to recruit it and pins it to the `a ≥ aMin = 0.02` bound on BOTH sides, where it reads **exactly
0.0 %** left/right against a true 22.7 %. That is a real finding destroyed and presented as "even".
So `MuscleLoad.isAtActivationFloor` now exists beside `isSaturated` and withholds the same way: the
cancellation argument holds in the interior of the box and nowhere else, and the box has two sides.

**The residual risk, with its number.** The cancellation needs the error to be the SAME factor on
both sides. The model is bilaterally symmetric and each side is sampled at its own mid-contact, so it
holds by construction — but a one-sided error costs 23.8 pp, and that is what the assumption is
worth. Not fixed here, and not fixable from this layer: the 3-D overlay still picks its strongest 24
muscles by the same uncalibrated cross-muscle number. It carries no names and no figures, and it is
shared with the live ARKit path, so it is disclosed on the panel rather than changed.

#### The majors

**M1 — a SOLVER-side hole split one contact into two, invisibly.** `GaitLoadSummary` recovered
contacts as maximal runs of consecutive stance frames it had received, so a non-converged IK, a
`submitAndWait` timeout or an unrouted solve turned one foot-strike into two, each contributing its
own off-mid-stance sample, and the pair was double-weighted in the mean. Nothing could see it: the
BODY frame was fine, so no frame number is missing and `GaitAnalysis` raises no
`.droppedSamplesInContact`, and the hole is absent from both sides of `usableStanceFraction`. The
authoritative boundary now travels with the frame — `GaitReport.stance` → `GaitPlan.Frame.contactIndex`
→ `GaitFrameOutcome.contactIndex` → the summary, which groups by it. MEASURED on a seven-frame
contact losing one frame: **9.09 %** of fabricated left-side asymmetry against `video_012`'s 10.145 %
publication floor.

**M2 — the claim gate had no term for the statistic's own noise.** `resolvableAsymmetryPercent` is
built entirely from timing quantisation (`max(50/framesPerContact, 100/stridePeriodFrames)`); the
2026-08-08 repair fixed the statistic's EXPECTATION (+8.07 % → −0.19 %) but its per-clip SPREAD is
what the gate compares against, and the gate cannot see activation scatter because scatter is not one
of its inputs. From this suite's own harness: SE 0.4737 over 400 trials → a per-clip standard
deviation of **9.47 %** against a 10.145 % floor, i.e. roughly one displayed muscle in four reading a
false finding on a symmetric runner with every clip-level gate green.

`MuscleLoad.samplingUncertaintyPercent` now computes, from the per-contact samples themselves, the
95 % half-width of `differencePercent`: `√(s_L²/n_L + s_R²/n_R)` scaled to the same denominator and
multiplied by the two-sided Student-t factor for `min(n_L, n_R) − 1` degrees of freedom. Student-t,
not 1.96: at the 4-6 contacts a real clip has, a normal multiplier is a third too narrow, and at
`df = 1` the factor is 12.7. Registered example, from the new test: a muscle reading 10.53 % — above
the 10.145 % timing floor, so the old gate published it — carries a **54.7 %** sampling floor and is
refused, while the same means with no contact-to-contact scatter still publish.

**M3 — `saturationThreshold = 0.999` was finer than the QP's own convergence band.** `MuscleSolver`
runs OSQP with `eps_abs = eps_rel = 1e-3`, `polishing = false`, and accepts `OSQP_SOLVED_INACCURATE`
(the same check at ten times the tolerance). With `A = I` and `z ∈ [0.02, 1]` a genuinely clipped
activation returns as low as **0.98**, `isSaturated` read false for it, and `%.2f` printed it as
"1.00" — so the one gate protecting the ratio argument was passing exactly the cases it exists to
catch. The two tolerances are now file-scope constants in `MuscleSolver.mm`, exposed as
`MuscleSolver.saturationActivationTolerance`, and the display threshold is derived from them
(`1 − 10·(1e-3 + 1e-3) = 0.98`) instead of being a second copy.

**M4 — the ranking key was clipped, and saturated rows drew full-length bars.** Both gone with the
ranking. A saturated or floor-pinned muscle now draws NO bars, because the bars encode the left/right
ratio and that ratio is precisely what is being withheld.

**M5 — the contact gate counted FRAMES, so a one-contact mean could be published against a
six-contact one.** `minimumContactsPerSide = 2` now, and it is entailed rather than chosen: with one
contact there is no second sample to estimate that side's scatter from, so M2's floor is infinite by
construction and the claim is uncertifiable rather than merely noisy. Registered example: one left
contact holding FIVE clean frames against four right contacts passes every frame count and is
refused, naming the reason.

**M6 — the "these are not diagnoses" note vanished on running clips.**
`PostureFindings.alwaysVisibleNote` is rendered unconditionally by `PostureFindingsPanel` under the
comment "Never behind a tap: this is the statement that keeps every number above it honest".
`GaitReportPanel` replaces that panel while making a strictly more clinical-sounding claim — named
anatomy, a 0-1 effort figure per side, a left/right verdict in warning orange — and carried no
equivalent. It now renders the SAME constant, on every branch including the refusals.

**M7 — a non-running side-on clip was routed to "Running, but withheld" and lost its posture
findings.** `GaitOutcome.isAboutRunning` is true for every case except `.notAttempted`, so `.refused`
counted as "this is a gait screen" — including `.notRunning`, whose entire meaning is "this is not
running". A user filming themselves side-on holding a squat, which is the app's stated purpose, got a
panel reading "only 0 complete contacts" and the measurements they came for — computed, and sitting
in the store — were not on screen. `replacesPostureFindings` is now true only for `.analysed`; a
refusal is a compact banner ABOVE the findings.

**M8 — per-leg `Fmax` puts the contact-time asymmetry inside every muscle bar, and the gate did not
widen by it.** `Fmax_side = (π/2)(1 + tf/tc_side)` and the QP is linear in the external load, so for
identical left/right activations the displayed difference IS that term. The screen said "part of the
left/right difference in these bars is that contact-time difference re-expressed as force" and never
how much — and it can be all of it. `contactTimeContributionPercent` is now computed from the
report's own per-leg peaks, printed with its size, and added to every claim floor. The panel's own
worked example (tcL 200 ms, tcR 160 ms, tf 130 ms) measures **−9.386 %**, so on a clip with an
8.086 % timing floor the claim floor becomes 17.47 % and a −9 % reading is refused.

**M9 — one hard-coded advice sentence under all nine refusals.** It named one cause and one lever and
was the wrong cause for five of them. The sharpest case: a clip refused for 2.8 frames per contact
was told to film a longer run at a steady pace, and re-filming that way produces byte-identical
output, because the only lever is frame rate — a number the app already computes.
`GaitReport.Refusal.advice(framesPerSecond:)` gives each refusal its own sentence, and the two rate
refusals quote the arithmetic: 2.8 frames per contact at 30 fps → **"Film at 33 fps or faster"**; a
4-frame median needing 5 → **"38 fps"**. `.notRunning` now says the clip is walking or holding a
position and points at the posture measurements, instead of telling the user to run more steadily.

**M10 — the fore-aft term was described as ABSENT when it is PRESENT and wrong.**
`getMultipleContactInverseDynamicsNearCoP` solves full 6-D wrenches per foot subject to Newton-Euler,
so it ASSIGNS a fore-aft ground force on every stance frame — whatever makes the horizontal CoM
acceleration match the raw source root `MHRRetarget` pins still. The engine's comment distinguishes "forced to
zero" from "left alone"; on this pose source the value ID consumes is ~0 either way, so the
distinction is numerically empty. STATUS sizes the error at 0.2-0.35 BW, LARGER than the worst
vertical residual the same screen reports as passing, and it is phase-dependent. The disclosure now
says it is fabricated, gives its size, and says which comparison it does and does not cancel out of.

**M11 — double contacts and missing contacts were collapsed into one counter, and the message
described the wrong one.** "The foot's height above the ground disagreed that it was planted" is
false for a double contact — the height agreed, and also flagged the other foot — and "film side-on,
with the ground in frame" is a lever that cannot work, because the ground was never the problem.
`solverSawDoubleContactCount` is now aggregated to the clip level and the refusal splits: the
double-contact branch names the 6 cm contact band and says outright that re-filming will not move it.

**M12 — the sparse `.fps` truncation banner was false in both halves.** It said "This clip is longer
than the analysis window, so N frames from the middle were used". `.fps` mode has no analysis window
(the cap is `maxFramesPerRun`) and its samples start at `t = 0` and step forward, so they come from
the BEGINNING. `OfflineDisclosureTests` asserted only `notice?.cause` for this mode while both
native-window tests asserted their message — the same defect class the notice was written to fix,
surviving on the other code path because the test did not look at the string. New cause
`.budgetStoppedTheSparseScan`, and the test now asserts the message.

#### What this stage did NOT do

* **No device run.** Everything is the `BioMotion-CI` simulator on one Mac.
* **The 3-D overlay's cross-muscle selection is unchanged** — see the blocker section. It is
  disclosed, not fixed.
* **M0 (17 killed hosts) and M13 (`DecodedFrameMemoryTests`) were the previous stage's**, and M13 was
  rejected there with two controlled experiments.
* The horizontal residual is still not computed, so M10 is a disclosure fix and not a measurement.
* The minors were not addressed.


### Fifth round: the cancellation was an identity, and the per-muscle claim is retired (2026-08-08)

Two blockers from the final adversarial pass. Both attacked the last surviving product claim — the
per-muscle LEFT/RIGHT percentage — and both stand up. **The claim is gone.** Test count 371 → **380**,
`Executed 380 tests, with 0 failures (0 unexpected)`, **0 restarts**, `** TEST SUCCEEDED **`, 680 s
wall, via `tools/run_tests.sh` on the private `BioMotion-CI` device. The floor in `run_tests.sh` is
raised to 380.

#### S1 — the test that certified the claim could not fail, and the test that can, fails

`MomentArmErrorCancellationTests` pinned `rightScale = 0.8` on EVERY right joint torque. The shipped
QP is `min ½aᵀ(εI + λAᵀA)a − λτᵀAa`, whose interior solution `a = (εI + λAᵀA)⁻¹λAᵀτ` is **linear in
τ** — so `a_R = 0.8·a_L` exactly, for ANY moment-arm matrix. The perturbation could not move the
answer. In exact arithmetic (active-set enumeration, `/tmp/qp_probe.py`) it moves it by **1e-6 pp**;
the **1.04 pp** recorded as evidence of cancellation is OSQP's own tolerance, and the assertion
`shift ≤ 2 × noise` was satisfied by construction.

Change ONE variable — the right leg differs in torque SHAPE rather than size (hip 0.80×, knee 1.00×
of the left, which is what a gait asymmetry is) — and the same bilateral `gamma × 0.6` perturbation
leaks. MEASURED on the shipping OSQP solver:

| case | worst shift in a left/right figure | note |
|---|---|---|
| solver noise floor | **1.52 pp** | gap between the analytic answer and what OSQP returns |
| proportional torques, `gamma × 0.6` both sides | **1.04 pp** | an identity — cannot exceed the noise |
| proportional torques: spread ACROSS the three muscles | **1.97 pp** | i.e. all three read the same figure |
| **shape asymmetry, `gamma × 0.6` both sides** | **9.92 pp** | on `beta`, whose own path is modelled correctly |
| shape asymmetry at `knee_r/knee_l = 0.6` | **17.72 pp** | the leak grows with the asymmetry |
| one-sided `gamma × 0.6` | **23.84 pp** | unchanged from the previous round |

`beta` reads `−17.89 %` with correct moment arms and `−7.97 %` with the wrong ones: a real 18 %
difference displayed as 8 %. Against publication floors of 10.145 % / 8.086 % on the two usable
pinned clips, 9.92 pp is larger than the finer of them and 98 % of the other. It is a BIAS, not
scatter — more contacts do not shrink it — and it lands on a correctly-modelled muscle, because the
QP redistributes load between synergists, so no per-row "this path is a straight line" flag contains
it.

**Why this retires the claim instead of widening a floor.** The leak is exactly zero when the two
legs' torques are proportional (measured: 1.04 pp at `knee_r/knee_l = hip_r/hip_l = 0.8`). But in
that regime the same linearity makes every muscle read the SAME figure — measured spread 1.97 pp
against a 1.52 pp noise floor — so the per-muscle breakdown is one number repeated, and that number
is the torque scale ratio the contact block already reports from timing. **All per-muscle
differentiation lives in the non-proportional part of the torque, which is precisely the part a wrong
moment arm distorts.** There is no regime that is both safe and informative, so
`perMuscleLeftRightClaimIsSupported` is a flat `false` and not a gate. The condition that would flip
it back is registered in its doc: model the 76 missing `PathWrap` references, or bound their error,
and re-run `testAShapeAsymmetryMakesABilateralMomentArmErrorLeak` against the 8.086 % floor.

#### S2 — eight order statistics quoted as eight independent tests

`make` builds a comparison for EVERY bilateral pair — **175 measured** on `FullBody.osim` — and
`ordered(_:)` sorts by `|difference| / claimFloor`, so the panel's `prefix(8)` took the eight largest
values of the very statistic the interval was about, under the caption "Each comparison is a 95 % one
and 8 are shown, so about one in twenty of them can read a difference that is not there."

`MuscleLoad.samplingUncertaintyPercent` now takes its Student-t at `α/N`, `N = screenedComparisonCount`
(pairs with ≥2 contacts a side and neither QP bound active — a rule that does not look at the
statistic). The multiplier is computed, not tabulated: `StudentT` evaluates the t CDF through the
regularized incomplete beta and inverts it by bisection, pinned against published quantiles
(`t₀.₉₇₅,₄ = 2.776`, `t₀.₉₉₉₅,₅ = 6.869`, `t₀.₉₉₉₉₅,₄ = 15.544`). Cost at real contact counts:
2.776 → **11.899** at df=4 (×4.29), 2.571 → 8.980 at df=5 (×3.49).

**MEASURED, per pinned clip — a symmetric runner, both legs drawn from one distribution, so every
survivor is false** (`GaitClaimSurvivalTests`; the clips supply the contact counts, timing floor and
contact-time term, the activations are synthetic because the fixtures carry five joints and no IK):

| clip | contacts/side | timing floor | σ | false claims, per-comparison rule | false claims, family-wise |
|---|---|---|---|---|---|
| video_012 | 6 | 10.145 % | 0.02 / 0.04 / 0.08 / 0.12 | 0 / **4** / **5** / **5** | 0 / 0 / 0 / 0 |
| video_015 | 5 | 8.086 % | 0.02 / 0.04 / 0.08 / 0.12 | 0 / **4** / **4** / **4** | 0 / 0 / 0 / 0 |
| video_013 | — | refused | — | 0 | 0 |

σ = 0.12 is the scatter every other measurement in this repo is taken at. The corrected claim floor
at that scatter is a MEDIAN of **121 %** (video_012) and **178 %** (video_015) — against a statistic
mathematically bounded at ±200 %. The smallest planted asymmetry that still survives: **40 %** at
σ = 0.04, **100 %** (video_012) and **120 %** (video_015) at σ = 0.12.

**So the honest answer to "what survives" is: nothing, on any clip.** Not because the correction was
tuned — it was not touched after the first run — but because 4-6 contacts a side cannot separate 175
muscles. Even with the moment-arm leak repaired, this feature would need a real asymmetry above 100 %
to report anything at the scatter we have.

#### What the running screen shows now

* **The surviving left/right finding is CONTACT TIME** — measured from stance timing, touching
  neither a moment arm nor the QP — and it is labelled as such ("Left vs right: time on the ground").
* The muscle block states, in one paragraph, that the comparison ran, how many pairs it covered, and
  why none of it is shown. No name, no `L 0.71 · R 0.55`, no bars, no orange verdict.
* **The 3-D muscle overlay is off on analysed running clips.** It selected the strongest 24
  activations and coloured them from one shared colormap — the cross-muscle ordering retired in round
  four, made in colour with no number and no caption. The still-pose path is untouched.
* The stance badge no longer reads "Pose + muscle (foot down — relative loads)", because no relative
  load is published.

#### Also fixed here, because the blockers' fixes touched them

* **The contact-time term was printed in the grammar of a share.** "so 9 % of every bar's left/right
  difference is that contact-time difference" — the value is an additive offset in percentage POINTS
  (−9.386 on the panel's own worked example), so a reader took 9 % of a 13 % reading, computed 1.2
  points of artefact where the truth is 9.4, and overstated their own asymmetry 3.3×. It now says
  "9 percentage points of right-high difference … subtract it, do not take a share of it".
* **`musclesWithUnmodelledPaths` was in the wrong namespace.** The table listed `vaslat`/`gaslat`
  (the `displayMuscleAliases` forms); the shipping path feeds it RAW solver names, so `vaslat140` and
  `gaslat140` — vastus lateralis and lateral gastrocnemius on the production model — were recorded as
  correctly modelled. The guard test passed because it applied the alias transform first, which the
  shipping path never does. Both names added; the test now asserts the RAW set separately.
* **`saturatedMuscleCount` counted muscle-SIDES** while `flooredMuscleCount` beside it counted
  muscles, and the panel printed them in one sentence — a clip clipping ten muscles on both legs told
  the user twenty had maxed out. Both are muscles now (`GaitLoadSummaryTests` asserts 1, not 2, for
  one muscle clipped on both sides).

#### What this stage did NOT do

* **No device run.** Everything is the `BioMotion-CI` simulator on one Mac.
* **The survival numbers use synthetic activation scatter.** No real per-muscle activation exists for
  the pinned clips — the fixtures carry five joints, which is what `GaitAnalysis` needs and not what
  IK needs. The scatter is swept over four levels rather than assumed at one, and the conclusion does
  not turn on the choice: above the level where the sampling term binds, a calibrated t-interval
  admits α of the family whatever the scatter is.
* **The LIVE ARKit path's muscle overlay is unchanged.** `MuscleOverlay` still picks its strongest 24
  by an uncalibrated cross-muscle number; only the offline running path is gated. The live path is a
  different surface with its own static-hold gating, and changing the shared renderer was not this
  stage's to do.
* **Minors deferred, recorded here rather than fixed:**
  1. ~~Pass-1 static-hold muscle output survives on frames the gait pass excluded.~~
     **CLOSED 2026-08-10:** one `BiomechanicsPayload` now replaces IK, ID, muscle, the static flag,
     and motion state atomically; a nil pass-2 field erases the pass-1 value.
  2. The multiplicity sentence's row count was hard-coded rather than taken from the rows drawn.
     Dead: the sentence and the rows are gone.
  3. ~~`GaitLoadSummary.framesPerSecond` used the video track's NOMINAL rate under sparse sampling.~~
     **CLOSED 2026-08-10:** analysed cadence now has one timestamp-derived source.
  4. `OfflinePlaybackView.statusText` consults `gaitLoadsAreComparable` and never
     `summary.arePublishable`, so a clip whose residual gate FAILED still gets a per-frame badge
     implying a successful solve. Half-closed: the badge no longer claims relative loads.
  5. One muscle-data refusal still states a fact and offers no lever ("No contact produced muscle
     output on both sides."). The separate nil-summary sentence was removed 2026-08-10 when timing
     was restored on that branch; it now says only which downstream section is unavailable.

### Sixth round: the same claim, in colour (2026-08-08)

Two adversarial lenses left 2 blockers and 5 majors. The blockers and four of the five majors were
closed in [round five](#fifth-round-the-cancellation-was-an-identity-and-the-per-muscle-claim-is-retired-2026-08-08);
this round closes the fifth, which is the one that had never been looked at, because it is not on the
running panel at all.

#### The finding: retiring a claim from a LIST is not retiring the claim

`MuscleOverlay` filtered `rawActivations`, dropped everything near the solver's floor, sorted the
rest descending and drew **the strongest 24**, colouring every capsule — both render passes — from
one shared blue→cyan→green→yellow→red ramp whose alpha rose 0.45 → 0.95 alongside it. Both halves of
that are the cross-muscle ordering retired in round four:

* **Selection.** Beyond the 26 hardcoded anatomical capsules, which muscles EXISTED on screen at all
  was a ranking by activation. A muscle whose unmodelled `PathWrap` under-states its moment arm has
  its activation inflated by `1/k` for its own unknown, pose-dependent `k`, so it sorts into the top
  24 ahead of a correctly-modelled muscle that is genuinely working harder.
* **Colour.** Every capsule of BOTH passes — including the fixed 26, which were always drawn — was
  then coloured by its activation, so the whole picture was ordered again, in hue and in opacity.

It shipped on **two** surfaces and the reviews had covered neither. The offline 3-D view drew it
beside the paragraph refusing the same comparison; the live ARKit screen drew it **and** carried
`MuscleActivationBar` under it — twelve named muscles, bar height ∝ activation, a blue/green/yellow/
red cut and `71 %` printed under each. That bar is the retired comparison in numbers *and* an
absolute effort claim on a scale the model does not have.

A picture makes this claim more loudly than a list, not less: no number to check, no floor quoted, no
caption. That is why the same defect on the more authoritative surface outranks the one on the panel.

#### What ships now

* **`MuscleOverlay.update(joints:)` takes no muscle solve.** Not "ignores one" — the parameter is
  gone, so the compiler refuses a magnitude at every call site. Deleted with it:
  `pathRenderActivationThreshold`, `floorMargin`, `maxRenderedPathMuscles`, the 64-bucket activation
  quantiser, `displayFloor`/`displaySaturation`, `displayValue`, `activationColor` and the local
  alias table the ranked pass needed.
* **The drawn set is the fixed 26-capsule anatomical list** (`muscleDefs`), minus any capsule whose
  two joints are not both tracked this frame. `capsulePlan(joints:)` is the pure function that
  produces it, split out so it is testable without RealityKit; `Capsule` has four fields — name,
  start, end, radius — and no magnitude channel.
* **One colour for the whole body**, `MuscleOverlay.capsuleColor`, deliberately off the retired ramp.
* **`MuscleActivationBar` is deleted.** The live screen's engineering diagnostics (marker residual in
  mm, `max |τ|/m` in Nm/kg, model mass, foot-load fractions, the N/kg frame check) are untouched:
  each is labelled with its unit and none is a per-muscle claim.
* **Both screens carry `MuscleOverlay.anatomyOnlyNote`** — one constant, so they cannot drift. It
  states the absence first ("Muscle effort is not shown."), then what the capsules are, then why.
  Live: under the diagnostics, whenever the layer is on. Offline: against the picture, at the bottom
  of the 3-D view, exactly when capsules are on screen.
* **The live toggle says `Anatomy ON/OFF`**, not `Muscles ON/OFF`, and the offline one is a labelled
  `Anatomy` control rather than a bare `figure.run` glyph.

**The offline clip-level gate did NOT change.** The overlay is still off on every analysed running
clip. Its *reason* changed and is restated in `muscleMagnitudesArePublishable`: the old one (the
capsules were a cross-muscle ordering) is gone with the ranking, and what survives is a coherence
rule — the panel beside that view is headed "Muscle by muscle: not shown, and why", and putting
muscle capsules next to it invites the reading that they are what the paragraph refused.

#### What is actually verified, and what is not

* **Structural, not numerical.** This round measured nothing about the body. The guarantee is a type
  signature: with no activation in scope, neither the selection nor the colour of a capsule can
  depend on one. Counted: ranked capsules 24 → 0, fixed anatomical capsules 26, activation bars
  12 → 0, and the activation→appearance map (hue across r/g/b plus alpha 0.45 → 0.95, four channels)
  → one constant.
* `MuscleOverlayClaimTests` (7 tests) guards the seams a regression would have to come through: the
  plan is the anatomical set and carries no `path_`-keyed entry; every capsule maps to ONE colour;
  `Capsule`'s stored properties are exactly `[end, name, radius, start]` (asserted by `Mirror`, so
  adding an `activation` field fails); a capsule with a missing joint is dropped without disturbing
  the other side; the note states the absence first and contains no digit and no `%`; all 16
  surface/tracking/frame/toggle combinations share one renderer/control/disclosure truth table; and
  a source-wiring contract pins the renderer, disclosure, control and calibration call sites to it.
* **No device run and no screenshot.** Simulator only, and no UI test — what is asserted is the
  string content and the plan, never the rendered appearance. Whether three lines of `caption2` sit
  well on the live screen under the diagnostics is unverified.
* **`BodyFrameOrientationTests` still passes unchanged**, so the capsule placement — the anatomy the
  layer now exists for — is still pinned to anterior/posterior correctness.

#### Known limitations, recorded rather than fixed

These are the MINORs from the two review lenses that survive, plus one found on the way. Each is a
statement the screen makes that is imprecise, not one that is false in a way a user can act on.

1. ~~**Pass-1 static-hold muscle output survives on frames the gait pass excluded.**~~
   **CLOSED 2026-08-10:** the result store no longer nil-coalesces fields from two solve generations.
   The complete pass-2 payload replaces pass 1, including erasing withheld ID/muscle values while
   preserving the frame envelope.
2. ~~**`GaitLoadSummary.framesPerSecond` used the video track's NOMINAL rate under sparse
   sampling.**~~ **CLOSED 2026-08-10:** the analysed outcome and summary factory no longer accept a
   second FPS beside `GaitReport.framesPerSecond`.
3. **`OfflinePlaybackView.statusText` never consults `summary.arePublishable`.** A clip whose
   vertical residual gate FAILED still gets per-frame badges implying a completed solve. Half-closed:
   the badge no longer says "relative loads".
4. **Two refusals state a fact and offer no lever** — `withheldReason`'s "No contact produced muscle
   output on both sides." and the panel's ".analysed with no summary" branch — while every other
   refusal on that screen ends in an action.
5. **`peakForceRegimeSentence` still ends "subtract it, do not take a share of it".** The unit is
   right (percentage POINTS, fixed in round five) and the fact is true of the internals, but there is
   no per-muscle figure on the screen any more to subtract it FROM. The imperative is stale.
6. **`NimbleEngine.displayMuscleResult` now has no consumer.** It is still `@Published` and still
   rebuilt every live frame through `normalizeActivations`/`normalizeForces`. Dead weight, not a
   false statement; removing it touches the solver publish path and was not worth the risk in a
   round whose subject is what the screen says.

#### What this stage did NOT do

* No device run, no TestFlight upload, `CURRENT_PROJECT_VERSION` not bumped.
* **It did not re-open the muscle claim anywhere.** Nothing in this round adds a reading; every
  change removes one or labels an absence.
* It did not touch `GaitLoadSummary`, the QP, the moment arms or any test that measures them.
* The parent repo's `labs/BioMotion` gitlink is still behind.

### Seventh round: the surviving claim was gated on the wrong variance too (2026-08-08)

Two reviewers left **1 blocker, 3 majors, 9 minors** on the state
[round six](#sixth-round-the-same-claim-in-colour-2026-08-08) shipped. The blocker and the three
majors are closed here; the nine minors are recorded below rather than fixed.

#### The blocker: the contact-time floor did not contain the contact-time scatter

`asymmetryClaim` published when `|contactAsymmetryPercent| >= resolution.resolvableAsymmetryPercent`,
and that floor is `max(50/framesPerContact, max(stride-period CV, 100/stridePeriodFrames))` —
quantisation plus STRIDE-PERIOD scatter. The statistic is the difference of two MEANS OF CONTACT
DURATIONS. Contact-duration scatter was measured all along (`contactVariationPercent`) and consumed
by **nothing**.

This is exactly the defect that killed the muscle claim one round earlier, on the claim that
survived it. `CLAUDE.md` already carried the principle — "An UNBIASED statistic is not a CERTAIN one,
and the gate only ever saw the bias" — and the same arithmetic had not been applied to the survivor.

**Measured, 20 000 seeded trials, a perfectly symmetric runner at `video_015`'s own configuration**
(5 contacts a side, contact durations scattering at its measured 11.144 %, timing floor 8.086 %):

| gate | publishes a left/right contact finding on |
|---|---|
| timing floor alone (what shipped) | **25.3 % of clips** |
| `max(timing floor, sampling half-width)` | **2.4 % of clips** |
| the same with Welch–Satterthwaite df | 4.0 % |

against a 5 % nominal. The shipped rule uses `min(n_L, n_R) − 1` degrees of freedom, matching the
muscle path, which is why it lands *below* nominal rather than at it — the conservatism is measured
and pinned rather than assumed
(`GaitContactClaimTests.testTheDegreesOfFreedomChoiceIsConservativeNotNominal`).

The in-code justification for the omission was falsified by this repo's own numbers. `GaitResolution`
said contact-duration scatter is "mostly detector edge jitter, which the quantisation floor already
counts". Two independent ±½-frame edges give a duration sd of `√(2/12) = 0.408` frames, i.e.
`0.408 / 6.1833 = 6.60 %` of CV on `video_015` against **11.144 %** measured — in quadrature, **65 %
of the variance is not edge jitter**, and no frame rate removes it.

**What ships**

* `GaitReport.contactSamplingUncertaintyPercent` — the Student-t half-width of the difference of two
  means, from the clip's own per-contact durations, at `α = 0.05` with ONE comparison (this is the
  screen's only left/right claim; if a second is ever added it becomes a family and the α splits).
* `GaitReport.contactClaimFloorPercent = max(timing floor, that half-width)`. `asymmetryClaim` and
  the `.asymmetryBelowResolution` flag are both built from it, so a clip cannot lose its claim
  without also getting the sentence that explains why.
* **One estimator, two consumers.** `MeanDifferenceUncertainty.halfWidthPercent` is new
  (`BioMotion/Gait/`), `GaitLoadSummary.samplingUncertaintyPercent` delegates to it, and `StudentT`
  moved there with it. The muscle path's numbers are unchanged — `GaitLoadSummaryTests`' t-multiplier
  table and `GaitClaimSurvivalTests` pass untouched.
* The panel prints each side's contact scatter beside its mean (`Left 206 ms ±11 %`), names both
  terms of the floor when it refuses, and quotes the floor when it publishes. `resolutionSentence`
  no longer says the timing figure is what the comparison has to clear, and **withholds the
  frame-rate promise entirely when the contact scatter is what binds** — a faster camera cannot
  deliver a floor it does not set.

#### ⚠️ THE DELIVERABLE IS EMPTY ON ALL THREE PINNED CLIPS, AND WAS BEFORE THIS CHANGE

| clip | measured | timing floor | sampling half-width | claim floor | claim |
|---|---|---|---|---|---|
| `video_012` | 2.90 % | 10.145 % | 7.451 % | 10.145 % | **none** |
| `video_015` | −0.54 % | 8.086 % | **16.464 %** | **16.464 %** | **none** |
| `video_013` | 5.57 % | 18.909 % | 62.031 % | 62.031 % | refused (unusable) |

The correction takes no claim away from these clips — their measured asymmetries were far under even
the old floor. What it removes is the one-in-four false finding on clips this fixture does not
contain, and what it reveals is that `video_015`'s honest floor is **double** what the screen was
printing: 16.5 %, not 8.1 %.

**The sensitivity, stated so the owner can judge the product.** At 5 contacts a side and this
runner's contact scatter, the corrected gate publishes a true 10 % asymmetry on 14 % of clips, a
true 25 % on 76 % and a true 40 % on 99 %. So the remaining claim can honestly find a **20-25 %**
left/right contact difference on a 4 s clip and not much finer. Longer clips are the only lever that
moves it and it moves as `1/√n`: the term is `t·√(s²_L/n_L + s²_R/n_R)`, so halving the floor needs
roughly four times the contacts — about 20 a side, i.e. a ~16 s steady run.

#### M1 — the LIVE screen's undisclosed left/right load claim (historical; now withheld entirely)

`ContentView` drew `AccuracyBadge(label: "L/R load", value: "0.62|0.38")` with no caption and no
floor, on the app's most-used screen, in the exact framing the offline path spent four rounds
retiring. Its `good` indicator was `abs(total − 1.0) < 0.3` — keyed to the SUM, which the near-CoP
solver constrains exactly — while the VALUE showed the split, which nothing checks.

**Decision at that stage: show the sum, do not show the split.** There is no discipline that could
have rescued it. `NimbleBridge.mm:1499` seeds the solve with a hardcoded 50/50 wrench guess whenever both feet are
down, and this file already records the double-support split as statically indeterminate at ±18 pp
with a perfectly known CoM against a ~10 pp clinically meaningful threshold. It is an artifact with a
plausible shape. The badge reads `GRF sum 1.00 BW`, its label and its indicator now measure the same
quantity, and `NimbleEngine.footLoadSplitIsNotMeasuredNote` sits under it on **the same `if`** as the
badges — the live path had already shipped one picture whose caption had a different gate.

**Superseded 2026-08-10:** the sum was still produced by a model/solver pair with empty contact
geometry and no support-domain, unilateral, or friction constraint. Both bundled models now fail
the capability gate before ID, so neither the sum nor the split is a current product diagnostic;
the availability explanation appears instead.

#### M2 — a data-gate failure sold a re-shoot that cannot help

`loadBlock` was an `if/else` on `withheldReason`, so a clip whose DATA gate failed saw only the data
refusal — "…film a steadier, straighter run" — under a header reading "Muscle by muscle: not shown,
and why". Every lever in `withheldReason` was written when passing that gate produced eight rows;
since `perMuscleLeftRightClaimIsSupported = false`, passing it produces a paragraph. The user
re-films, the gate passes, and the answer changes to "it was never possible".

`perMuscleRetirementSentence` now prints on **every** branch, the data refusal prints after it as a
separate statement about a separate subject, and `muscleRowsUnaffectedByRefilmingSentence` scopes the
lever — it is nil the moment a per-muscle claim is supported again, so it cannot outlive its truth.

#### M3 — solver stopping points printed as facts about the user's muscles

The honesty block printed "N muscle(s) reached full effort and M sat on the resting-tone floor, out
of S pairs the solver kept between the two". Three defects in one sentence, and **fixing it closes
minors 2 and 4 as well**:

1. N and M measure where OSQP stopped. This file's own λ sweep at FIXED inputs reads 19, 11, 22, 18,
   20 saturated and 219, 189, 344, 170, 282 floored, with no trend.
2. The trailing disclaimer covered the ACTIVATION at the bound, not the COUNT — and the count was the
   part rendered as a number about a body.
3. The denominator cannot contain its numerators: `screenedComparisonCount` is built with
   `guard !saturatedBases.contains(base), !flooredBases.contains(base)`, disjoint from both by
   construction. `ClaimSurfaceTests.testTheFlooredCountCanExceedTheScreenedCountSoTheyAreNotAFraction`
   constructs it: floored 3, screened 1.

What replaces it states the mechanism, which is stable and checkable: this QP answers with a bound
for most of 520 redundant muscles, and a bound is not a measurement. No count of the reader's muscles
appears. `saturatedMuscleCount` / `flooredMuscleCount` remain as data with their own tests.

#### Tests

`GaitContactClaimTests` (8) and `ClaimSurfaceTests` (5) are new; `tools/run_tests.sh` `MIN_TESTS`
385 → 398. The Monte-Carlo runs on contact DURATIONS rather than synthetic `BodyFrame`s on purpose —
the question is the gate's error rate at a stated scatter, and putting a stance detector in front of
it would make the answer a statement about the detector. `testThePublishedHalfWidthIsTheShared…` is
what ties that arithmetic back to what `GaitReport` publishes, on the real clips.

#### The nine minors, recorded rather than fixed

Numbered as the reviews numbered them. Minors 2 and 4 are the M3 sentence and are closed by that fix;
the rest stand.

1. ~~**`GaitLoadSummary.framesPerSecond` is the video track's NOMINAL rate even when the sparse
   `.fps` sampler ran.**~~ **CLOSED 2026-08-10.** The report's real timestamps are now the only FPS
   source accepted by the analysed outcome and summary factory; see the timestamp-cadence section.
2. **The honesty block's three counts in one sentence with a denominator excluding its numerators.**
   CLOSED by M3 — the sentence no longer prints any of the three.
3. **`peakForceRegimeSentence` still ends "subtract it, do not take a share of it".** The unit is
   right and the fact is true of the internals, but there is no per-muscle figure on screen to
   subtract it from. The imperative implies a number that is not there.
4. **The same sentence's "out of N pairs" could not contain its numerators.** CLOSED by M3.
5. ~~**`FrameBudgetNotice.budgetCappedTheWindow` names the wrong cause at 240 fps.**~~
   **CLOSED 2026-08-10** — this was exactly the rate
   `resolutionSentence` is allowed to recommend. A 4.0 s clip against a 4.0 s window wants 960
   samples and `maxNativeWindowFrames` caps at 601. Before the fix, the banner said the clip "is
   longer than the analysis window" when the window was 4.0 s; acting on it (trimming to 2 s) could
   trip `clipShorterThanTheWindow` and refuse the run for too few contacts. It now names the frame
   budget, and the selector discloses the 2.5-second high-rate span and rising processing cost.
6. **A doc comment cites a test suite that does not exist.** `GaitLoadSummary.tMultiplier`'s doc names
   `MuscleUncertaintyTests`; `grep` returns only that line. The guarantee holds under another name
   (`GaitLoadSummaryTests.testTheUncertaintyMultiplierIsStudentTAtSmallCounts`), so the citation is
   stale rather than empty — but a reader greping for the named guard finds nothing.
7. **Two code comments assert the identity `CLAUDE.md` exists to deny.** `NimbleEngine.swift:1195` and
   the derivation at `:263-268` say the residual's difference is `‖a_artic‖/g`. The shipping
   computation is `leftFootForce.y + rightFootForce.y` — one axis. The user-facing sentence gets this
   right (`verticalFalsifierSentence`); only the internals still say the retired thing.
8. ~~**On the LIVE screen the anatomy capsules and their caption have different gates.**~~
   **CLOSED 2026-08-10.** `LiveAnatomyPresentation` now owns one 16-row contract for the renderer,
   Anatomy control and disclosure. Calibration refuses all three even with a current frame and an
   enabled preference; tracking requires an active session plus a current frame; capsules and the
   sentence then share `anatomyIsPresented`. The anatomy path no longer depends on `.osim` loading,
   while the separate IK/ID control retains that model gate.
9. ~~**An analysed clip with no load summary loses the CONTACT-TIME finding too.**~~
   **CLOSED 2026-08-10.** Resolution, contact time and flags are now report-owned, non-optional
   sections; only the downstream muscle section follows `summary` availability.

#### What this round did NOT do

* No device run, no TestFlight upload, `CURRENT_PROJECT_VERSION` not bumped.
* It did not touch the QP, the moment arms, the solver or any test that measures them.
* It did not re-open any retired claim. The contact-time claim is narrower than it was, not wider.
* The parent repo's `labs/BioMotion` gitlink is still behind.
### What build 30 actually delivers, and what it refuses to (2026-08-08)

Five workflow rounds, twelve commits, 219 → 398 tests. The honest summary, because the headline is
that **two product claims were retired on evidence and the third is empty on all three reference
clips.**

**Retired: the cross-muscle ranking.** `MomentArmComputer` logs that 76 PathWrap references are not
modelled, so those muscles take a straight-line shortcut instead of wrapping around bone. The
affected set is essentially every muscle a running analysis would name. A wrong moment arm maps a
joint torque to a wrong force, and that does not cancel between two different muscles.

**Retired: the per-muscle left/right figure.** The cancellation argument — same wrong model on both
sides, so `F_left/F_right` survives — was MEASURED FALSE. The test that certified it pinned every
right-side torque at 0.8× its left counterpart, making the QP's response an algebraic identity
(1e-6 pp in exact arithmetic; the "1.04 pp" it reported was OSQP's own 1.52 pp noise floor). With a
shape-asymmetric torque instead of a proportional one, the same bilateral moment-arm error leaks
**9.92 pp** into `beta` — a muscle whose own path IS modelled correctly, because the QP redistributes
between synergists. Publication floors are 8.09–10.15%. `perMuscleLeftRightClaimIsSupported = false`.

**Retired: the 3-D muscle overlay on analysed running clips**, because colouring muscles against each
other is the cross-muscle ranking on a more authoritative surface. It is an anatomy layer now.

**Retired: the live screen's L/R load split.** `NimbleBridge` seeds the solve with a hardcoded 50/50
wrench when both feet are down, and double-support indeterminacy is ±18 pp against a ~10 pp
meaningful threshold. The badge shows the SUM.

**Multiplicity.** The eight muscle rows were the top-8 order statistics of ~175 bilateral pairs
quoted at a per-comparison error rate. On a symmetric runner — where every survivor is false — the
old rule published 4–5 claims at realistic scatter; family-wise correction over the 175 actually
screened publishes 0, and its floor (121–178%) exceeds what a statistic bounded at ±200% can express.

**The one surviving mechanical claim is left/right CONTACT TIME**, because it touches neither a
moment arm nor the QP. Its gate was built from the wrong variance — stride-period scatter, when the
statistic is the difference of two means of ~5 contact durations. Monte-Carlo on a symmetric runner
at `video_015`'s own configuration: the old gate published a false finding on **25.29%** of clips,
the corrected gate on **2.43%** (nominal 5%; Welch 4.01%).

**And it is empty on all three clips, which is the honest result, not a bug:**

| clip | measured | timing floor | contact half-width | claim floor | published |
|---|---|---|---|---|---|
| video_012 | 2.90% | 10.145% | 7.451% | 10.145% | none |
| video_015 | −0.54% | 8.086% | **16.464%** | **16.464%** | none |
| video_013 | — | — | 62.031% | — | refused |

The correction removed no existing finding. What it revealed is that `video_015`'s honest floor is
**double** what the screen had been printing. The surviving claim needs a 20–25% left/right contact
difference on a 4 s clip before it speaks (true 10% publishes on 14.5% of clips, 25% on 75.5%, 40%
on 99.2%).

**The lever is measured and it is not the camera.** The timing floor binds on only 0.49% of draws —
this claim is limited by the runner, not the sampling grid. The half-width falls as 1/√n, so halving
16.5% needs ~20 contacts a side ≈ a 16 s steady run against the current 4 s window. Cost: 480 model
calls ≈ 5.6 min at 0.7 s/frame, ~850 MB peak for 576×768 (it would NOT fit at 1080p). That is a
bounded engineering change and it is the obvious next step; it is out of scope for build 30.

**What build 30 delivered at that historical stage:** a skeleton whose legs finally track the stride (the `upperBodyOnly`
crop fix), gait timing per side with its scatter and step counts, a per-clip resolution figure the
app computes rather than disclaims, the kinematics posture findings, then-active raw static-hold
muscle diagnostics, and a commit gate (`tools/run_tests.sh`) whose green actually means green. The
2026-08-10 contact-support gate now withholds those diagnostics from the product.


## The moment arms now have a REFERENCE, and the defect is measured (2026-08-08)

Build 30 retired the per-muscle left/right claim because the moment arms feeding it are wrong: the
running muscles' paths WRAP around bone and `MomentArmComputer` cuts straight through. Until now
that was an argument with a count attached (`unmodelledPathWraps = 76`) and no measurement, because
this project had no authoritative moment arm to compare against. It has one now.

### The reference

**OpenSim 4.6 runs on this Mac.** `uv pip install opensim` — the PyPI wheel
`opensim-4.6-2-cp312-cp312-macosx_15_0_universal2.whl` — into
`tools/opensim_ref/.venv`; `osim.GetVersion()` = `4.6-2026-06-22-85aaf64`. No conda, no micromamba,
no build from source. It loads `BioMotion/Resources/FullBody.osim` and reports **169 coordinates,
520 muscles, 69 WrapObjects on bodies**, of which **64 are referenced by the 76 PathWraps on 66
muscles (56 WrapCylinder + 8 WrapEllipsoid)** — exactly the counts the workflow brief carried.

Two models, same poses, same (muscle, coordinate) pairs:

| | |
|---|---|
| **wrap ON** | the model as shipped, OpenSim solving all 76 PathWraps. **The reference.** |
| **wrap OFF** | every `WrapObject.set_active(False)`, so each path is the straight polyline through its path points — what `MomentArmComputer` computes today, inside OpenSim's own numerics |

`tools/opensim_ref/dump_reference.py` writes both over **173 poses** (neutral, deep squat,
trunk-flexed, six asymmetric running phases, five 1-D sweeps, a 48-point lower-limb grid) × **7,496
structurally-spanned pairs**, in 288 s. Output is ~98 MB of CSV, gitignored and regenerable; the
committed artefact is `BioMotionTests/Fixtures/opensim_moment_arms.txt` — 173 poses × 104 muscles
(all 66 wrapped + every muscle the product names) = 17,992 rows, 2.5 MB, in the plain-ASCII
no-force-unwrap grammar `GaitClipFixture` uses. The two loaders share `FixtureScanner` rather than
each carrying a copy of the number grammar.

### The defect, measured

Over all 173 poses. Ratios exclude pairs whose REFERENCE moment arm is under 1 mm — a ratio there is
a statement about its own denominator — and those are counted separately.

| set | pairs | median rel. | p90 | max | >10% | >100% | **wrong SIGN** |
|---|---|---|---|---|---|---|---|
| the 66 muscles that carry a PathWrap | 41,866 | **13.7%** | 124.4% | 694.7% | 53.1% | 20.4% | **3,769 = 9.00%** |
| the muscles the product NAMES | 37,714 | 0.0% | 22.0% | 376.9% | 16.2% | 0.7% | 232 = 0.62% |
| named muscles, running poses only | 1,308 | 0.0% | 13.9% | 116.7% | 12.1% | 0.3% | 2 |
| muscles with NO PathWrap | 1,254,942 | **0.0** | 0.0 | **0.0** | 0.0% | 0.0% | **0** |

The last row is the control: a muscle with no wrap object is IDENTICAL in the two models to the last
stored digit, at every pose. So the gap in the other rows is the missing wrap solver and nothing
else.

In absolute terms, on wrapped muscles: median 0.909 mm, p90 76.3 mm, max **146.6 mm**. Path length
itself is out by up to 129.6 mm = **51.8% of the muscle's own length**; wrapping is actually engaged
on 7.4% of (muscle, pose) rows, which is why the medians are 0 and the tails are enormous.

**9% of the wrapped-muscle pairs point the WRONG WAY.** STATUS already records what a sign-flipped
moment arm does — the QP refuses to recruit the muscle, pins it to `aMin` on BOTH sides, and the
per-muscle left/right figure reads exactly 0.0% against a true 22.7%. That is now a count, not a
hypothesis.

Worst named pairs, by median relative error over all 173 poses: `gasmed_l`/`knee_angle_l` **112.9%**,
`gaslat140_l`/`knee_angle_l` 94.5%, `psoas_r`/`hip_rotation_r` 87.8%, `addmagProx_l`/`hip_rotation_l`
71.3%, `gasmed_r`/`knee_angle_r` 63.5%, `iliacus_*`/`hip_adduction_*` 53.3%. The single largest
error anywhere is `TR2_l`/`L5_S1_LB`, where the shipped code returns **−0.00448 m** and OpenSim
returns **+0.14210 m**: wrong sign and 32× too small.

### Is "wrap off" really what this code computes? Yes, to 4.4 mm

`StraightLinePathErrorTests` drives the SHIPPED `MomentArmComputer` — nimble skeleton, its own FK,
its 1e-4 rad centred difference — over 35 of the fixture's poses and compares all three columns
(12,384 samples, 31 s):

| | median | p90 | p99 | max |
|---|---|---|---|---|
| ours vs OpenSim **wrap-OFF** | 0.000 | 0.000 | 3.79 mm | **4.39 mm** |
| ours vs OpenSim **reference** | 0.000 | 67.1 mm | 119.1 mm | **146.6 mm** |

The two independent implementations of the straight-line path agree to 4.4 mm worst case while the
gap to the truth is 33× larger, so essentially the whole error is the missing wrap solver. The 4.4 mm
residual is not wrapping at all: it is `BIClong_l`/`pro_sup_l` (ours +0.00957 vs +0.01396), i.e. the
linearly-interpolated `MovingPathPoint` splines the fidelity report already counts, plus the latched
`ConditionalPathPoint`s and nimble's FK. Worth fixing eventually; it is 3% of the wrap error.

`testOurStraightLineTracksOpenSimWithWrappingDisabled` is a **TRIPWIRE**: it asserts the shipped code
matches the wrap-OFF column, which is true exactly as long as wrapping is missing. When the solver
lands it must be repointed at the wrap-ON column, not deleted — the same comparison is then the gate
that says the solver works.

### The discontinuity risk is sampled, and it is small but real

`dL/dq` is discontinuous where a muscle starts or stops wrapping, and a centred difference straddling
that switch invents a moment arm. The fixture records `wrapPoints` per (pose, muscle) — the count of
`PathWrapPoint`s OpenSim inserted — and the flag is a clean separator: across all 89,960 rows, every
row with `wrapPoints == 0` has the two lengths equal to **exactly 0.0 m**, and the smallest
difference on an engaged row is 1.87e-8 m. The 1-D sweeps bracket **25 engage/disengage transitions
on 23 distinct muscles** (knee 8, hip 11, elbow 6; the ankle and shoulder sweeps produce none), so
the next stage has poses that straddle the switch rather than hoping it is rare.

### Three model traps this turned up, each caught by a guard rather than by luck

1. **`shoulder_elv_l` runs −115..0 deg while `shoulder_elv_r` runs 0..115.** Mirroring an arm pose by
   copying the value puts the left shoulder 25 deg outside its range, and
   `Coordinate::setValue(state, v, false)` accepts it **silently** — no clamp, no warning. Caught by
   an explicit range check in `dump_reference.py`; without it the reference would have carried a pose
   the model is not defined at.
2. **Ranges in the .osim are rounded decimals.** `shoulder_elv_r` maxes at 2.0071 rad = 114.99836
   deg, `hip_flexion_r` at 2.0943950999999998 = 119.99999986 deg. A sweep written as "0 to 115" is
   genuinely outside. Sweep endpoints are snapped onto the model's own limits, and an overshoot
   larger than 1 deg raises instead.
3. **`GeometryPath::computeMomentArm` returns exactly 0.0 for a LOCKED coordinate** — a refusal, not
   a measurement, and FullBody.osim locks 54 of its 169. nimble does not honour `<locked>` (that is
   why the app carries a runtime DOF mask), so the shipped code produces a real number there.
   Differencing a real number against a convention would have manufactured a 100% error and blamed
   the wrap solver for it. Those pairs are excluded from the reference by construction.

And one build trap: `osim.Model(filename)` **chdirs into the model's directory**, so the default
logger wrote `opensim.log` into `BioMotion/Resources/` — a `type: folder` resource in `project.yml`,
i.e. straight into the app bundle and the test bundle. `osim_model.py` calls
`osim.Logger.removeFileSink()` at import. The commit gate caught the stale project reference; nothing
else would have.

### What the pinned clips could NOT supply, and one thing they say anyway

The brief asked for joint angles from `BioMotionTests/Fixtures/gait_*.txt` if possible. **They cannot
supply them.** Those files hold five marker positions per frame — raw MHR root, both ankles, both toes.
Fifteen scalars with no knee, no thigh and no pelvis orientation do not determine hip/knee/ankle
flexion: source-root-to-ankle distance constrains the SUM of hip and knee flexion and says nothing about
the split. So the pose set is a coverage grid over the model's own coordinate ranges — 100% of the
clamped range of `knee_angle`, `ankle_angle`, `elbow_flex`, `shoulder_elv` and 93.3% of
`hip_flexion`, at 5 deg or finer — and it is labelled as that, not as a claim about a runner.

The clips do say one thing, recorded as an observation and **not** acted on: their source-root-to-ankle
distance folds to **0.280** of that clip's own maximum (0.315 / 0.280 / 0.316 for video_012/013/015),
and FullBody.osim cannot fold below **0.337** anywhere in its clamped sagittal range — 0.349
measuring from the model hip-centre midpoint instead. Raw MHR joint 1 is itself 15.1 mm from the
source HJC midpoint, so neither root convention explains it. Three candidates, and
`pose_coverage.py` cannot separate them: the clip's own maximum may
overstate a straight leg (which makes the gap WIDER), a single noisy frame may inflate that maximum,
or `MHRRetarget` emits leg configurations outside the model's joint limits and IK is clamping on
those frames. If it is the third, it is happening on exactly the deeply-flexed frames where wrapping
matters most.

### What this stage did NOT do

* It did not implement wrapping. No `WrapCylinder`, no `WrapEllipsoid`, no change to
  `MomentArmComputer`, no change to the QP, and **no retired claim reopened**.
* It did not measure the per-frame COST of a wrap solver, because there is no solver to time. The
  brief's second risk stands open: OpenSim's cylinder solve iterates (MAX_ITERATIONS 100) and the
  ellipsoid is a numerical geodesic, against a chain that already costs ~200 ms/frame.
* It did not decide anything about `quadrant`, which selects WHICH side the path wraps around. The
  reference will catch a backwards one; nothing here has looked at it.
* It did not fix the 4.4 mm `MovingPathPoint` spline residual.


## Cylinder path WRAPPING ships (2026-08-08)

The moment arms had a reference and a measured 146.6 mm defect. This closes most
of it: `MusclePathWrap.cpp` is a port of opensim-core's `WrapCylinder.cpp` +
`WrapObject.cpp` + `WrapMath.cpp` + `GeometryPath::applyWrapObjects`
(Apache 2.0, Stanford, Peter Loan / Frank C. Anderson — licence header in the
file, attribution in `./NOTICE`).

**64 of FullBody.osim's 76 `PathWrap` references are now SOLVED** (all 46 of
Rajagopal2016's are). `unmodelledPathWraps` counts 12 — the `WrapEllipsoid`
references on 10 elbow muscles — and nothing else.
`GaitLoadSummary.musclesWithUnmodelledPaths` went from 37 entries to 5.

### What it cost the error

Same 36 poses and the same structurally-spanned pairs
`StraightLinePathErrorTests` uses, over the 56 muscles whose every wrap is a
cylinder (7,488 pairs):

| vs the analytic reference | median | p90 | p99 | max | sign flips |
|---|---|---|---|---|---|
| straight line (what shipped) | 0.972 mm | 78.1 mm | 119.1 mm | **146.6 mm** | **661** |
| wrapped (this change) | **0.048 mm** | 1.24 mm | 2.04 mm | **8.07 mm** | **4** |

Path LENGTH, single-wrap muscles: median 0, p99 **0.047 mm**, max **0.428 mm**
against OpenSim's own. Wrap ENGAGEMENT — whether the wrap is on at all, which is
the thing a length can agree with by accident — matches OpenSim on **2,016 of
2,016** (pose, muscle) rows, including 1,189 where it is engaged.

The control still holds: the 454 muscles with NO wrap object are unchanged, max
3.758 mm from the reference, exactly where they were.

### Two things the reference turned out to be, that it was not thought to be

**1. `computeMomentArm` is not `dL/dq`.** `MomentArmSolver` computes the
generalized force a unit tension along the CURRENT path produces with the wrap
points held fixed on their bodies. That equals `−dL/dq` where the path varies
smoothly with q, and it does not where the wrap solution is marginal. OpenSim
differencing its OWN length at the same `eps` (`tools/opensim_ref/fd_check.py`):

    TR2_l   / L2_L3_FE    at spine_flexed: analytic +0.002252, central −0.005274
    gasmed_r/ knee_angle_r at neutral:     analytic +0.021761, central +0.004891
    gasmed_r/ knee_angle_r at squat_deep:  analytic +0.026091, central +0.026090

All four of the "sign flips" above are that gap. Against the definition-matched
column (`BioMotionTests/Fixtures/opensim_moment_arms_fd.txt`, generated in 7 s by
`dump_finite_difference.py`) the single-wrap muscles read median **0.000 mm**,
p99 **0.000 mm**, max **3.569 mm** over 7,056 pairs, and **1** sign flip — at a
reference value of 1.00 mm, i.e. below the 3.758 mm floor at which these two
implementations agree about anything at all.

**2. OpenSim's MULTI-wrap output is not self-consistent.** With one `PathWrap`
it solves the spiral in closed form; with two or more it re-solves the whole set
up to 8 times. On `gasmed_r` at `neutral` the path it reports has tangent points
that are the closed-form solution for the ORIGINAL `P1→P2` segment — verified by
deactivating the other wrap object, which reproduces them to 6 decimal places —
while the spiral length stored beside them, 0.038054, belongs to a later
`C2→P2` solve; the spiral implied by its own tangent points is 0.046516. This
port re-solves to a fixed point, so it cannot match that state. It differs by
**8.75 mm** of length on the 4 two-cylinder muscles (`gasmed`, `gaslat140`, left
and right) and by 15.8 mm of moment arm at `neutral` — where, note, this code is
1.05 mm from OpenSim's ANALYTIC value and OpenSim's own central difference is
16.9 mm away from it. Open; recorded, not fixed.

### The discontinuity, constructed and handled

`dL/dq` is discontinuous where a muscle starts or stops wrapping. The dangerous
switch is the cylinder-END rule (the surface is a finite segment, so when both
tangent points slide past `length/2` the wrap stops being applied from a length
that is NOT close to the straight line). Constructed on bare geometry
(`MusclePathWrapTests`): **L steps 36.1 mm**, and a centred difference at
`eps = 1e-4` returns **−180.7 metres** per unit while the one-sided difference on
the engaged branch reads 0.000.

`computeMomentArmsWithJointAngles:` therefore compares the wrap solver's
DISCRETE state — which objects engaged, on which segment, which branch, hashed
into `WrappedPathResult.signature` — at `q`, `q+eps` and `q−eps`, and drops to a
one-sided difference on the side that stays on the base pose's branch. Driven
through the shipped chain (`MomentArmWrapDiscontinuityTests` bisects to a real
switch: `grac_r` / `knee_angle_r` at q\* = −1.465481812):

| | value |
|---|---|
| raw centred difference across the switch | **−19.62 m** |
| shipped (branch-consistent one-sided) | **−0.033693 m** |
| the same muscle 20 steps inside the branch | −0.033602 m |

Three counters expose the decision: `lastCentredDifferenceSamples`,
`lastOneSidedDifferenceSamples`, `lastUnresolvedDiscontinuitySamples`. Over the
36 validation poses they read **3,163,680 / 0 / 0** — at a normal pose the
stencil never straddles a switch, which is exactly why the test has to construct
one rather than wait for it.

### Cost — the named risk, with numbers

Measured A/B on the same 36 poses, Debug, iOS Simulator, 169 coordinates x 520
muscles, by clearing every muscle's `pathWraps` after parsing and re-running:

| | mean per `computeMomentArms` call |
|---|---|
| straight line (before) | **889.0 ms** |
| wrapped (after) | **6048.8 ms** |

That is **6.8x**, and it is a Debug number. The solver benchmarked standalone
(`clang++`, same source, gasmed_r's own geometry) shows how much of that is the
build:

| | −O0 | −O2 | ratio |
|---|---|---|---|
| 2-wrap solve | 3096.8 us | **22.13 us** | 140x |
| 1-wrap solve | 58.0 us | **0.268 us** | 217x |
| straight polyline | 0.253 us | 0.004 us | 63x |

At the −O2 rates the wrap layer costs about **36 ms** per moment-arm solve
(1,356 two-wrap solves at 22.1 us + 20,340 one-wrap solves at 0.268 us) against
a chain STATUS sizes at ~200 ms/frame. **This is an extrapolation, not a
measurement: nothing here ran in a Release build or on the phone.** The
two-wrap path is 80x the one-wrap path and dominates, so that is where an
optimisation would go.

### One side effect, found by the commit gate and not swallowed

`MuscleQPUnitsTests.testResidualMechanismSweep` sweeps the QP's soft-penalty
weight to check that the leftover torque residual is a REACHABILITY distance
rather than an artefact of the objective weighting. With the corrected moment
arms, one point of that sweep — `lambda = 100` on the synthetic `dancer` pose —
now returns the **all-at-floor corner**: 520 of 520 muscles pinned to `aMin`,
`relativeForce = 1.026`, i.e. the solution explains none of the demand. Every
other lambda at the same pose solves normally (relative 0.3404-0.3505, 105-184
at the floor), and dropping the locked rows solves it at 0.334.

It happens ONLY with `excludesLockedCoordinates = false`, which the sweep forces
and the shipping path never uses. The test now refuses to read a solve that
returned the lower-bound corner — comparing that with a healthy residual is
comparing a failure with a measurement — and asserts the mechanism claim over
the solves that solved, where the spread is **0.029** (dancer) and **0.229**
(upright) against a 0.5 falsifier. **Open:** why lambda = 100 specifically
degenerates on that pose with locked rows included has not been chased.

### What this did NOT do

* `WrapEllipsoid` — 12 `PathWrap` references on 10 elbow muscles, still straight.
  **Closed on the same day — see the next section.**
* It did NOT reopen the per-muscle left/right claim.
  `perMuscleLeftRightClaimIsSupported` is still `false`: the 9.92 pp synergist
  leak in `MomentArmErrorCancellationTests` was measured with the OLD moment
  arms and has not been re-run against these.
* No Release build, no device, no re-run of the QP or `GaitLoadSummary`.
* The 4.4 mm `MovingPathPoint` spline residual is untouched; it is now the
  largest remaining implementation gap rather than the smallest.


## Ellipsoid path WRAPPING ships — every PathWrap in the model is solved (2026-08-08)

The cylinder left 12 `PathWrap` references unsolved: the 8 `WrapEllipsoid`s on
the two humeri, carried by 10 elbow muscles (`ANC`, `BIClong`, `BICshort`, `BRD`,
`TRIlong`, left and right). `wrapEllipsoidLine` in `MusclePathWrap.cpp` is a port
of opensim-core's `WrapEllipsoid.cpp` — `wrapLine` + `calcTangentPoint` +
`CalcDistanceOnEllipsoid` + `findClosestPoint` + `closestPointToEllipse`, the
last two being David Eberly's Graphics Gems IV routines — under the same Apache
2.0 header, with `WrapEllipsoid.cpp` added to `NOTICE`.

**`solvedPathWraps` is 76 and `unmodelledPathWraps` is 0** on FullBody.osim (46 /
0 on Rajagopal2016), and `GaitLoadSummary.musclesWithUnmodelledPaths` — 37
entries two commits ago, 5 one commit ago — **is now empty**. That table stays,
as the tripwire: a surface this build has no solver for, or a
`<PathWrap><method>` other than `hybrid`, puts entries back into it.

### Why it was implementable rather than disclosed

The brief allowed shipping the 56 cylinders and DISCLOSING the 8 ellipsoids if
the numerical solve turned out too slow — on a measurement. The measurement says
implement:

| paired A/B, one process, 6 poses, ellipsoids active then deactivated | mean per `computeMomentArms` |
|---|---|
| with `WrapEllipsoid` | **7687.6 ms** |
| without | **6018.9 ms** |
| ratio | **1.28×** (+1668.7 ms) |

That is Debug, iOS Simulator, 169 coordinates × 520 muscles, against a
pre-registered ceiling of 3×. It is cheap for a reason worth knowing: an
ellipsoid wrap engages ONLY when the straight segment actually pierces it, and
the expensive part — the 298-sample "fan" of point-to-ellipsoid Newton solves
that picks the wrapping plane — is downstream of that test. Standalone (an ad-hoc
`clang++` harness against the same `MusclePathWrap.cpp`, the model's own radii,
3,000 iterations per case; not committed, like the cylinder's):

| | −O0 | −O2 |
|---|---|---|
| ellipsoid, fan runs (segment 45° to the axes) | 690–762 µs | **12.3–40.8 µs** |
| ellipsoid, no fan (segment axis-aligned, `mu > 0.9`) | 174–179 µs | **0.71–1.98 µs** |
| cylinder, 1 wrap, same machine | 34.9 µs | 0.092 µs |

So one engaged ellipsoid solve is 130–450× a cylinder solve, and it is affordable
only because it is rare. The whole 60-pose sweep costs **8037.9 ms** mean per
solve now, against 6091.7 ms for cylinders alone.

### The pre-registered gates, and the two amendments

Gates E1–E8 / X1–X5 are in the `EllipsoidWrapValidationTests` docstring, written
before any number from this stage was read. The population is the 10 muscles the
parser itself reports as carrying an ellipsoid, stratified single-wrap
(`ANC`, `BICshort`, `BRD`) versus multi-wrap (`BIClong`, `TRIlong`) for the same
reason the cylinder gates are: with one `PathWrap` OpenSim solves the path once,
with more it re-solves the set up to 8 times.

| | median | p90 | p99 | max |
|---|---|---|---|---|
| single-wrap, ours vs OpenSim's own central difference (n=960) | 0.000 mm | 2.438 mm | 4.414 mm | **4.414 mm** |
| multi-wrap, same column (n=1,080) | 0.000 mm | 1.129 mm | 4.414 mm | 4.414 mm |
| path LENGTH, single-wrap (n=360) | 0.000 mm | 0.181 mm | 0.210 mm | **0.210 mm** |
| path LENGTH, multi-wrap (n=240) | 0.057 mm | 0.181 mm | 0.210 mm | 0.210 mm |

Wrap ENGAGEMENT — the discrete thing a length cannot agree with by accident —
matches OpenSim on **600 of 600** (pose, muscle) rows, 303 of them engaged. Sign
flips against the analytic column go **135 → 0**. Numerical refusals (the two
places this port returns `NoWrap` where OpenSim returns a NaN): **0 over 60
poses**.

**Amendment E-A1 — the p99 threshold was copied from the wrong population.** E2
was pre-registered at 4.0 mm because that was the cylinder muscles' measured p99.
This population is different in a way that was already documented: all four of
`FullBody.osim`'s `MovingPathPoint`s are on `BIClong_*` and `BICshort_*`,
`MomentArmComputer` interpolates them LINEARLY between cubic-spline control
points, and the section above recorded the resulting **4.39 mm** residual on
`BIClong_l`/`pro_sup_l` before this stage began. E2 is 5.0 mm, E1's bar.

The ablation settles the attribution rather than arguing it. The ten worst
surviving residuals are all `BIClong_l` and `BICshort_l` about `pro_sup_l` — the
two `MovingPathPoint` muscles, on the coordinate their moving point tracks — and
each reads **4.414 mm with the ellipsoid and 5.569 mm without it**. The ellipsoid
did not cause the survivor; it made it smaller.

**Amendment E-A2 — "beats the straight line" is measured by ABLATION, not
against OpenSim's wrap-off column.** X3 was pre-registered against the wrap-off
column, whose median error on these muscles is *exactly 0* over this pose set —
so "strictly better than the median" was unreachable by construction, and the
comparison also carries every other difference between the two implementations.
The honest A/B is this code with the ellipsoids ON and OFF at the same poses
(`MomentArmComputer.setEllipsoidWrapObjectsActive:`), against OpenSim's own
derivative:

| 204 ablation pairs, of which 100 the ellipsoid changes | median | p90 | max |
|---|---|---|---|
| ellipsoids OFF | 2.449 mm | 5.913 mm | **10.242 mm** |
| ellipsoids ON | **0.000 mm** | 3.851 mm | **4.414 mm** |

76 of the 100 changed pairs move towards OpenSim. The individual rows say it
better than the percentiles:

    BRD_r  / elbow_flex_r   on +0.00151   off -0.00873   OpenSim +0.00151
    BRD_l  / elbow_flex_l   on +0.00429   off -0.00423   OpenSim +0.00429
    BIClong_r/elbow_flex_r  on +0.00382   off -0.00208   OpenSim +0.00384

Elbow flexion is what brachioradialis and biceps DO. Without the ellipsoid their
moment arm about it had the wrong sign.

### Two things about `WrapEllipsoid` that were not known before

**1. `hybrid` is a pure function of the pose; `axial` is not.** OpenSim seeds
`r1`, `r2`, `c1` and `sv` from the PREVIOUS call's `WrapResult`. On the `hybrid`
branch all four are overwritten before they are read — `r1`/`r2` by the
line/ellipsoid intersection and then by `c1`; `c1`/`sv` by Frans, by the fan, or
by the blend of the two — so the answer depends only on `q`. On `axial`,
`use_c1_to_find_tangent_pts` can be false, which leaves the PREVIOUS call's
tangent points as the seed for the iteration. Differentiating a function of call
history is not differentiating a function of `q`. All 12 references in the model
say `hybrid`; anything else is refused at parse time and counted as unmodelled
(DEVIATION 8), and `MusclePathWrapTests` proves the refusal and the purity.

**2. `findClosestPoint` is scale-dependent, and at metre scale it silently
returns the query point.** Its Newton iteration stops on `|f| < 1e-9` where `f`
is a degree-6 polynomial in the radii. With the model's real radii (0.02 m) `f`
is already ~1e-19 at the first iterate, so the routine returns `t = 0`, i.e. the
point it was asked about, unchanged — a plausible answer, off by centimetres.
That is why OpenSim's `factor = 3/(a+b+c)` exists. Measured both ways in
`MusclePathWrapTests`: normalised, it agrees with an exhaustive search over the
surface to 1.5e-3 (the search's own grid); unnormalised, it hands the query point
straight back, and the test asserts that it does so nobody "fixes" the
normalisation away.

### What the geometry tests pin

* **A sphere has a closed form** and `ANC_l`/`ANC_r` are literally spheres
  (`0.02 0.02 0.02`). Great-circle arc 0.010218987 m against ours 0.010217239 m;
  the −1.748e-6 m gap is the chord sum's own deficit (`s·Δφ²/24` = 1.112e-6 for
  the 10 chords it used) and the test's tolerance IS that quantity. Tangent
  points land on the surface to 4.4e-7 m.
* **`<quadrant>` is tested twice**: mirrored on a sphere with the chord through
  the centre (contact at z = ±0.018856, identical length to 1e-9), and on
  asymmetric geometry where the two sides differ by 4.22 mm so a solver that
  ignored the quadrant would return the same number twice.
* **The engagement boundary is BENIGN.** Bisecting to the exact `h` where the
  segment starts piercing a 20 mm sphere: the jump in L is **0.000e+00** — the
  arc goes to zero there, so a centred difference across it is legitimate.
* **The chord count is not.** The surface distance is summed over
  `(int)(|r1−r2| / 1 mm)` chords, so L STEPS by 2.609e-6 m each time that integer
  ticks. Small, but divided by `2·eps` it is a 13 mm/rad moment arm, so the chord
  count is part of `WrappedPathResult::signature` and the stencil sees it. Over
  the 60 validation poses the counters read **5,272,800 centred / 0 one-sided /
  0 unresolved**, which is why the test has to construct the tick by sweeping.

### One side effect, found by the commit gate and not swallowed

`OfflineOrchestrationTests.testNineSubmissionsProduceMuscleOutput` failed on the
first full-gate run, reporting **"no muscle output after 9 submissions"**. It was
not an orchestration failure. That suite drives `NimbleEngine` one frame at a
time and waited a hard-coded 10 s per submission; the ninth push is the first
where the Savitzky-Golay window is full, so it is the first that runs ID + moment
arms + QP, and with the ellipsoids it costs **11.65 s** in a Debug simulator
build. It used to fit: the same test ran in 12.45–12.56 s end to end with every
submission inside the 10 s bound. Measured per push, now:

| push | cost | what runs |
|---|---|---|
| 0 | 4.07 s | first solve, cold |
| 1–7 | 0.09 s each | IK only — the window is not full |
| 8 | **11.65 s** | ID + moment arms + QP |

The bound is now a NAMED `submissionLivenessTimeout` of 45 s with the measurement
in its doc comment, the per-push seconds are printed, and `timedOut == 0` is
asserted explicitly — so the failure mode it used to report as "no muscle output"
now reports as the number that caused it. **This is a judgement call and it is
recorded as one:** a 10 s wall clock in an async Debug test was acting as an
unstated performance gate, and the honest instrument for per-frame cost is the
paired A/B in `EllipsoidWrapValidationTests`, which measures it directly against a
pre-registered ceiling. The trade is that a cost regression between 10 s and 45 s
on this path would no longer fail here; it would still fail the A/B's 3× ceiling.

### The pose set grew, and the cylinder numbers held

The harness now runs **60 poses**, not 36: `WrapValidationHarness` adds all 16
`elbow_sweep_*` and 13 `shoulder_sweep_*` poses, because all 8 ellipsoids are on
the humerus and without them the arm sat at ONE configuration in 31 of the 36
strided poses — a large sample count of near-copies of one arm pose. Both suites
read the same sweep (two harnesses would be two measurements). The cylinder gates
were re-run on the larger set and did not move: single-wrap max **3.569 mm**
(n=11,760), path length max **0.428 mm**, engagement **3,360 of 3,360**, 0 sign
flips above the 3.758 mm control floor, control unchanged at max 3.758 mm.

### What this did NOT do

* It did NOT reopen the per-muscle left/right claim.
  `perMuscleLeftRightClaimIsSupported` is still `false`. The 9.92 pp synergist
  leak in `MomentArmErrorCancellationTests` was measured with the OLD moment arms
  and still has not been re-run. Every `PathWrap` being solved is a NECESSARY
  condition that is now met; it is not the measurement.
* No Release build, no device. The 1.28× is Debug/simulator and the −O2 rates are
  a desktop benchmark of the solver in isolation. Extrapolating the same
  Debug→Release ratio the cylinder measured (≈170× on the solver itself) puts the
  ellipsoid layer at roughly **10 ms** per moment-arm solve on top of the
  cylinder's ~36 ms, against a chain this file sizes at ~200 ms/frame — but that
  is arithmetic, not a measurement, and the LIVE ARKit path runs this every frame.
* The `MovingPathPoint` linear interpolation is untouched and is now, by
  measurement, the largest implementation gap left: 4.414 mm on
  `BICshort_l`/`pro_sup_l`, against a 3.758 mm floor at which nimble's FK and
  OpenSim agree about anything at all.
* The 4 two-cylinder muscles still differ from OpenSim's non-self-consistent
  iterate by 8.749 mm of length and 15.824 mm of moment arm. Unchanged; open.
* Nothing was done about `MuscleOverlay`, `GaitLoadSummary`'s claim gate, or the
  QP. The only behaviour change downstream is that `pathIsModelled` is now true
  for every muscle in both shipped models.


## The re-measurement: the moment arms are fixed and the claim still cannot come back (2026-08-09)

This is the measurement the retirement registered as its own falsifier, and the answer is **no, with
a different reason than the one that closed it**. `perMuscleLeftRightClaimIsSupported` stays `false`.
Nothing the user sees changed. `WrappedMomentArmLeakTests` + `BoxQPTests` + one new test in
`MomentArmErrorCancellationTests`.

### The rig: real geometry, not three synthetic muscles

`MomentArmErrorCancellationTests` perturbs one invented muscle by `×0.6`, a number somebody chose to
stand for "this path is a straight line where the real one wraps". Since 2026-08-08 that is not what
the code does, so the 9.92 pp it measures is a measurement of a defect that no longer exists.

The new rig perturbs nothing. It takes the moment arms this build computes and the arms OpenSim 4.6
computes for the same model at the same pose, and solves the shipping QP twice.

* **Geometry** — the 40 right-leg muscles of `FullBody.osim` that structurally span at least one of
  the six unlocked right-leg coordinates (`mtp_angle_*` is locked, and a locked coordinate's
  `computeMomentArm` is a refusal, not a measurement). 39 of the 40 are in
  `GaitLoadSummary.displayNames`.
* **Bilateral by construction** — the left leg IS the right leg, mirrored onto the six left
  coordinates. That is what the cancellation argument was about, and it is asserted against the
  matrix the solver is handed rather than stated in a comment.
* **Shape-asymmetric torque** — the left leg's joint torques are the right's with a per-joint scale,
  five shapes around the `hip 0.80 / knee 1.00` case that retired the claim. Inverse dynamics never
  touches a moment arm, so `τ` is IDENTICAL in the two solves; the only thing that differs is `A`.
* **Isometric force scale** — every muscle sits at its own `l_opt + l_Ts` with zero pennation and
  zero velocity, so `f_AL = f_FV = cos α = 1` and `A = R·diag(F_max)` exactly. Checked against the
  solver's own returned forces: worst relative departure **1.5e-15**.
* 31 poses × 2 reference definitions × 5 shapes × 3 effort levels = 930 cells, 582 of them readable
  (≥20 muscles interior in the exact solution).

### The instrument had to be built first, and the first three attempts at it were wrong

Three instrument failures, each recorded because each produced a plausible number:

1. **A "solver noise floor" that was measuring the ACTIVE SET.** Solving the same arms under a
   PROPORTIONAL torque and comparing against the analytic `100(c−1)/(0.5(1+c))` is only valid while
   no activation sits on a bound. In an 80-muscle rig most sit on `aMin`, and a clamped muscle's
   contribution does not scale with `τ`, so the interior muscles compensate differently at different
   torque scales. It reported 18–45 pp of "noise" that was a genuine nonlinearity of the QP.
2. **A sign error inside that instrument** — it compared against `+100(1−c)/(0.5(1+c))` while the rig
   scales the LEFT leg. ~44 pp of pure sign.
3. **A KKT residual normalised by the gradient**, which reads exactly **1.0** at a perfect interior
   solution, where every gradient component is legitimately at rounding level. It sent a correct
   answer back as a total failure. The denominator has to be the largest TERM entering the gradient,
   not the gradient.

What replaced them is `BoxQP`: the same objective `MuscleSolver.mm` builds
(`min ½aᵀ(εI + λAᵀA)a − λτᵀAa`, `ε = 0.01`, `λ = 100`, `a ∈ [0.02, 1]`), solved to machine precision
by a primal active-set method. Two implementation notes, both measured rather than assumed:

* **Woodbury, not a dense factorisation.** `A` has only 12 rows, so the free-set solve collapses to a
  12×12 Cholesky through `(εI + λBᵀB)⁻¹b = (b − Bᵀ((ε/λ)I + BBᵀ)⁻¹Bb)/ε`. But `b = λAᵀτ` is of order
  1e7 while `εx` is of order 1e-2, so that subtraction cancels about NINE decimal digits; three steps
  of iterative refinement recover them. Without refinement the KKT residual was 1.0.
* **A step-length rule, not a clamp.** The obvious loop — solve on the free set, clamp whatever comes
  back out of bounds, release every wrong-signed multiplier — cycles, because a clamp is not a
  descent step. The loop now takes the longest feasible step toward the free-set solution and
  releases exactly ONE constraint per iteration, so each iteration strictly decreases a strictly
  convex objective and no active set can repeat.

`BoxQPTests` checks it against a closed form, a dense Gaussian-elimination solve, cyclic coordinate
descent, and a direct optimality test (300 feasible perturbations, largest objective improvement
**0.0**). Worst KKT residual over all 930 real cells: **9.7e-13**.

### What the measurement says

Three quantities, separated, because the first attempt confounded them:

| | median | p90/p99 | max |
|---|---|---|---|
| Moment arms alone, `\|d(ours,exact) − d(truth,exact)\|` | **0.977 pp** | p99 85.70 | 123.10 |
| … the same, straight-line arms (the CONTROL) | **7.939 pp** | — | 66.88 |
| The shipping solver alone, `\|d(ours,OSQP) − d(ours,exact)\|` | **14.88 pp** | p90 37.83 | **100.98** |
| Both together, `\|d(ours,OSQP) − d(truth,exact)\|` | 14.42 pp | — | 63.28 |

* **The wrap work bought a factor of 8.1 in the median** (0.977 against 7.939 pp on the identical
  rig), and the original three-muscle rig re-run with the perturbation resized from `×0.6` to the
  measured p99 residual reads **0.568 pp** where it read **9.92 pp**. That is the answer to "did
  fixing the moment arms fix the leak": for the typical muscle at the typical pose, yes.
* **The tail did not close.** Against OpenSim's ANALYTIC column the worst is 42.46 pp (`glmed3` at
  `run_4_mid_swing`, p99 9.94, median 0.41); against its own CENTRAL DIFFERENCE the worst is
  123.10 pp (`piri` at `grid_h060_k000_a+00`, p99 104.54, median 7.09). Those two columns disagree
  with each other by more than the gates allow, so **how much of that tail is this build's residual
  and how much is the reference's own inconsistency is NOT settled here.** Either way it is above the
  8.086 % floor, and R1 fails.
* **The binding constraint is now the solver, and it is not about geometry at all.** With the arms
  held fixed, OSQP's answer differs from the exact minimiser of the same objective by a median of
  14.88 pp. `MuscleSolver` runs OSQP at `eps_abs = eps_rel = 1e-3`, polishing off, and accepts
  `OSQP_SOLVED_INACCURATE` — `saturationActivationTolerance` is the project's own name for the
  resulting **0.02 of absolute activation slack**. At the rig's median activation of 0.132 the
  arithmetic is `100·2·0.02/0.132 = 30 pp`, which is the order of what was measured. An ABSOLUTE
  tolerance on `a` becomes a RELATIVE error in `100·(a_l − a_r)/mean`.
* **The retirement's SECOND argument is defeated, and it matters.** "The regime where the error
  cancels is the regime where every muscle reads the same number, so there is no regime that is both
  safe and informative" is a claim about a ratio. Measured: the spread of the true left/right figures
  across muscles is **48.5×** the moment-arm error in them, and only **2.90×** the error the product
  would actually print. The rows carry real per-muscle information; it is drowned by the solver, not
  by the geometry.

### Also answered, because the stage asked

**Zero muscles have unmodelled paths.** `MomentArmComputer`'s own fidelity report, read at runtime
rather than from the hand-written table: FullBody 76 solved / **0** unmodelled, Rajagopal2016 46
solved / **0** unmodelled, `musclesWithUnmodelledPathWraps` empty on both. **No muscle in
`GaitLoadSummary.displayNames` has an unmodelled path** — the intersection is empty because the
unmodelled set is.

### Pre-registered gates, and how they landed

Written before any number was read. `floor` = 8.086 %, the smallest `resolvableAsymmetryPercent` any
usable pinned clip achieves, read from the clips. `floor/5` is the reopening bar, on the argument
that the floor is a 95 % half-width on RANDOM error while a leak is a BIAS: in a normal approximation
a bias of `h/5` moves a nominal 5 % false-positive rate to 6.8 %, and `h/3` doubles it to 10.0 %.

| Gate | Requires | Measured | |
|---|---|---|---|
| R1 | moment-arm leak < 1.617 pp | 123.10 pp (max) | ❌ |
| R2 | printed-number error < 1.617 pp | 63.28 pp (max) | ❌ |
| R3 | three-muscle rig at the measured residual < 1.617 pp | 0.568 pp | ✅ |
| R4 | 0 unmodelled `PathWrap`s, none displayed | 0 / 0 | ✅ |
| R5 | straight-line CONTROL leaks > floor | 66.88 pp, 271/549 cells over it | ✅ |
| R6 | ≥20 muscles and ≥30 cells | 21 at the worst cell, 582 cells | ✅ |
| R7 | spread / printed error > 4 | 2.90 (48.5 against moment-arm error) | ❌ |

One amendment, disclosed with its mechanism: **R2 was registered as `leak + 2·noise < floor`** using
the instrument that turned out to be measuring the active set. With that withdrawn, R2 became a
DIRECT measurement of the solver's contribution against a machine-precision solve, at the same
`floor/5` bar — strictly stronger than what was registered, since the original admitted a leak up to
the full floor.

### What this did NOT do

* It did not touch the product. `perMuscleLeftRightClaimIsSupported` is still `false`, `permits(_:)`
  still returns false for every muscle on every clip, and no screen changed.
* It did not fix the solver tolerance. Tightening `eps_abs`/`eps_rel`, enabling polishing, or
  refusing `OSQP_SOLVED_INACCURATE` are all one-line changes with a per-frame cost that has not been
  measured, and any of them changes every activation the app has ever produced.
* It did not settle the moment-arm tail. `piri`/123 pp and `glmed3`/42 pp are attributed to "one of
  our residual or OpenSim's two mutually-inconsistent columns" and no further.
* It did not measure the real 520-muscle × 169-coordinate problem. The rig is 80 × 12. The MECHANISM
  is scale-free and the shipping activations are the same magnitude, but the shipping problem's own
  slack was not measured.
* Nothing ran in a Release build or on the phone.
* The stride case is untouched: this rig mirrors ONE pose, so both legs carry the same modelling
  error. A real clip samples each side at its own mid-contact, which is the one-sided bound (23.8 pp
  with the old arms) and is a different measurement.


## The screens now state the reason that is true (2026-08-09)

The section above changed the reason the per-muscle claim is withheld and changed nothing the user
reads. For one build the product therefore stated, on its most-read surfaces, two facts that its own
commits had measured **false**:

| Surface | What it said | What was measured beside it |
|---|---|---|
| `GaitLoadSummary.perMuscleRetirementSentence`, under "Muscle by muscle: not shown, and why" | "66 of its muscles are given a straight line where the real tendon wraps around bone" | `MomentArmComputer` fidelity report at runtime: **76 solved / 0 unmodelled**; `musclesWithUnmodelledPaths` empty |
| the same paragraph | "the error moves a muscle's figure by around 10 percentage points on this app's own test rig" | that rig at this build's residual: **0.568 pp**; the moment-arm leak on real geometry: **0.977 pp** median |
| `MuscleOverlay.anatomyOnlyNote`, on the LIVE ARKit screen and the offline 3-D view | "it gives many of its muscles a straight line where the real tendon wraps around bone" | same as above; the note is one constant rendered by both screens |

**The suite was holding them in place.** `ClaimSurfaceTests` asserted
`perMuscleRetirementSentence.contains("wraps around bone")` and `MuscleOverlayClaimTests` asserted
the same substring on the note — each written when the statement was true, each now a green test
whose only effect was to require a refuted sentence. Both assertions now name the mechanism that
actually withholds the rows AND refuse the one that was fixed (`XCTAssertFalse(contains("straight
line"))`), so the pair cannot go stale in the same direction twice.

### What the surfaces say now

* **The panel paragraph** states that every path that wraps around a bone IN THIS MODEL is solved as
  a wrapped path, that **for the typical muscle** the moment arms are therefore no longer the biggest
  error in the chain, and that what withholds the comparison is the SHARING step: it stops as soon as
  it is close enough, "close enough" is a fixed margin on each muscle's effort, and comparing two
  small efforts turns that margin into a large percentage — **about 15 percentage points**, roughly
  twice the smallest difference the clearest clip can resolve. Both hedges are load-bearing: "in this
  model" because whether a tendon that wraps in a body has a `WrapObject` declared for it is the
  model author's decision and nothing here checks it, and "for the typical muscle" because R1 still
  FAILS on the tail (123.10 pp worst against the solver's 100.98) and its attribution is open. One
  rounded number, because a paragraph a runner reads once is not a results table; the exact values
  live on the flag.
* **The anatomy note** carries no digits at all (asserted), so it names the sharing step and the
  reason that never depended on the paths: nothing puts two different muscles' efforts on one scale.
* Neither sentence changes what is shown. No row came back, `perMuscleLeftRightClaimIsSupported` is
  still `false`, and `permits(_:)` still returns false for every muscle on every clip.

### The reopening condition was satisfied and still meant "no"

`perMuscleLeftRightClaimIsSupported`'s own comment — the thing a future stage reads immediately
before flipping it — said: *model the 76 missing `PathWrap` references, then re-run
`testAShapeAsymmetryMakesABilateralMomentArmErrorLeak`; if the leak drops below 8.086 %, the claim
can come back.* **Both halves now hold** (76/76 solved; that rig reads 0.568 pp) while the decision is
correctly `false` on gates the comment never mentioned. A registered condition that is satisfied
while the answer is no reads as permission, so it is replaced rather than annotated: the comment now
carries R1–R7 with their measured values (R1 123.10 pp, R2 63.28 pp, R7 2.90 — all against
`floor/5 = 1.617 pp` and a required ratio of 4), names
`testTheShippedFlagMatchesWhatTheMeasurementSupports` as the tripwire that fails if the flag and the
gates disagree in either direction, and names the tolerance as the binding constraint.

Same class of staleness, fixed in the same pass because each was a sentence a reader would act on:
`claimFloorPercent`'s doc said the leak is not a floor term because "nothing in this pipeline
measures or bounds it" (it is measured — 0.977 pp median / 123.10 pp worst; the conclusion stands for
a different reason: a bias does not belong in a sum of half-widths); `GaitReportPanel`'s type doc and
`OfflinePlaybackView.muscleMagnitudesArePublishable` both cited the straight-line paths as current;
`MuscleOverlay`'s own type doc said the count was "down to 10 elbow muscles" after the ellipsoid
commit had taken it to 0; and `ContentView`'s comment beside the deleted bar chart still gave the
straight-line paths as the current reason the live screen shows no per-muscle number.

**Nothing here is a measurement.** This round measured nothing new; every number in the new text is
read from `WrappedMomentArmLeakTests` and `MomentArmComputer`'s runtime report. It is a
correction of what the product SAYS, gated on the same suite.

> ⚠️ **"About 15 percentage points" lasted one commit.** The section below took the solver's own
> slack from 14.88 pp to 4.4994e-05 pp, so the paragraph this round wrote went stale in exactly the
> way the paragraph it replaced had. Both wordings are now NEGATIVE assertions in
> `ClaimSurfaceTests` / `MuscleOverlayClaimTests`.


## The QP now returns its own answer: 14.88 pp → 4.4994e-05 pp, and the claim still does not come back (2026-08-09)

The re-measurement two sections up named `MuscleSolver`'s OSQP termination as the per-muscle claim's
binding constraint: with the geometry held FIXED, the shipping solver's answer differed from the
exact minimiser of the SAME objective by a median of **14.88 pp** of a published left/right figure.
This section closes that. **It is not a tolerance change, and tightening the tolerance was measured
and does not work.**

### The diagnosis, before anything was changed

A temporary probe on `osqp_solve` recorded `status_val`, `iter`, `prim_res`, `dual_res`, `‖q‖∞` and
`‖P‖max` for every solve in the leak sweep — **2,791 solves** — and for the real 520-muscle problem.

| reading | value | what it rules in or out |
|---|---|---|
| `status_val` | **2,770 SOLVED / 21 SOLVED_INACCURATE** | the solver is not reporting a failure; nothing downstream could have noticed |
| `iter` | **988 of 2,791 stopped at 25**, the FIRST termination check; 39 reached the 200 cap | ✗ NOT "the iteration cap is binding" on the rig |
| `prim_res` | median 3.8e-05, max 2.1e-03 | ✗ NOT the primal tolerance — the quantity `saturationActivationTolerance` describes was never what limited accuracy |
| `dual_res` | median **788**, p90 9,159, max 5.8e04 | ✓ this is the one |
| `‖q‖∞` | median **9.8e06**, max 4.1e07 | ✓ and this is why |

`compute_dual_tol` (`osqp/src/auxil.c`) is `eps_abs + eps_rel·max(‖q‖∞, ‖Px‖∞, ‖Aᵀy‖∞)`. With
`q = −λAᵀτ`, `λ = 100`, moment arms in metres and forces in newtons, that permits a stationarity
violation of **~8e03** and the solver delivers ~788 of it.

That is only fatal because the objective is FLAT in most directions. `A` has one row per coordinate,
so on the 80-muscle × 12-coordinate rig **68 of the 80 eigenvalues of `εI + λAᵀA` are exactly
ε = 0.01** and its condition number is **1.22e09** (measured, `numpy.linalg.eigvalsh` on a dumped
cell). Dividing the permitted violation by the curvature that resists it gives the activation error
the termination test tolerates:

* along the stiff directions: `788 / 1.22e07` = **6e-05** — tight;
* along the 68 flat ones: `788 / 0.01` = **8e04** — i.e. **no constraint at all**; only the box
  bounds them.

Projecting the measured error onto the two subspaces confirms it rather than inferring it: at
`scaling = 10`, `‖a_OSQP − a_exact‖ = 0.607` and **99.3 % of it lies in the flat complement**
(0.602 against 0.072 in the row space of `A`); at `scaling = 0` the norm is 3.6e-04 and still
98.4 % flat. The solver was never "roughly right everywhere" — it was right where its stopping rule
could see and unconstrained in the rest.

**The flat directions are "which synergist carries the load", which is the entire content of a
per-muscle comparison.** The claim was being refused by the one subspace the solver's own stopping
rule ignored.

### Tightening the tolerance was tried and REJECTED, with numbers

On a dumped cell (`ankle_sweep_+017.5`, the neighbourhood of the worst measured slack), against an
independent machine-precision active-set solve:

| setting | iterations | wall | `‖a − a*‖∞` |
|---|---|---|---|
| shipped: `eps = 1e-3`, 200 iters | 50 | 0.15 ms | **0.3076** |
| `eps = 1e-9`, 20,000 iters | 125 | 0.59 ms | **0.3025** |
| `eps_rel = 0`, `eps_abs = 1e-3`, 20,000 iters | 14,925 | 34.9 ms | 0.0718 |
| `eps_rel = 0`, `eps_abs = 1e-6`, 200,000 iters | 200,000 (cap) | 463 ms | 1.1e-03 |
| **`scaling = 0`** (everything else shipped) | 25 | **0.07 ms** | **5.4e-08** |

Six orders of magnitude of tolerance buys nothing, because the tolerance is relative to data six
orders of magnitude larger than the curvature. What works is turning OSQP's **Ruiz equilibration
off**: it rewrites the constraint matrix `A = I` into a diagonal the ADMM step no longer matches, and
the flat subspace stops contracting. With it off, the same solver at the same tolerance converges
there in 25 iterations. `polishing = 1` on top solves the equality-constrained KKT system on the
identified active set and takes what is left to ~1e-07.

### The change

Three constants in `MuscleSolver.mm`, each with its measurement in the comment above it:
`kOSQPScaling = 0`, `kOSQPPolishing = 1`, `kOSQPMaxIterations = 4000` (200 was not enough for the
shipped model — every 520-muscle solve in `StaticEquilibriumBenchmarkTests` terminated at exactly
`iter = 200` with `OSQP_SOLVED_INACCURATE`, i.e. the cap and not the tolerance decided when to stop,
and the result was accepted anyway). `eps_abs`/`eps_rel` are **unchanged at 1e-3**, deliberately:
they also define `saturationActivationTolerance`, which `GaitLoadSummary` uses to screen
bound-pinned muscles out of every comparison, and moving a screening threshold is a separate
decision from fixing a solver.

### What it measured — the same rig, the same gates, re-run

`WrappedMomentArmLeakTests`, 582 readable cells, real geometry:

| quantity | before | after |
|---|---|---|
| solver slack, median | 14.883 pp | **4.4994e-05 pp** |
| solver slack, p90 | 37.826 pp | **0.0471 pp** |
| solver slack, max | 100.977 pp | **21.981 pp** |
| median relative torque residual | 2.7995e-03 | **8.316e-09** |
| printed-number leak (R2), median | 14.424 pp | **1.045 pp** — inside the 1.617 pp bar |
| printed-number leak (R2), worst | 63.279 pp | 108.575 pp |
| R7, median spread ÷ printed error | 2.903 **FAIL** | **47.52 PASS** |
| R1, moment-arm leak worst | 123.097 pp | **123.097 pp — bit-identical** |

The last row is the control that makes the rest an attribution: every quantity that does not involve
OSQP is unchanged to the last stored digit, so the change touched the solve and nothing else.

**On the problem the app actually solves** — 520 muscles, 109 of `FullBody.osim`'s coordinates after
the locked and no-muscle exclusions, real wrapped moment arms at the neutral pose, `τ = A_eff · 0.1`
so a feasible interior answer exists — measured against `BoxQP` (KKT residual 8.5e-13, 138 active-set
iterations):

| | before | after |
|---|---|---|
| worst departure from the exact minimiser | **0.10269** (`ANC_l`, interior) | 5.882e-04 (`QL_post_I_3-L3_r`, AT a bound) |
| worst departure over INTERIOR muscles | 0.10269 | **1.017e-04** |
| the same as a left/right percentage at the median interior activation (0.0577) | **355.8 pp** | **0.352 pp** |
| OSQP wall, Debug simulator | 188.8 ms | 194.4 ms (**+3.0 %**) |

That closes next-step 25 (the shipping problem's slack was an inference; it is now a number) and
answers the cost question the previous round left open: **+3 % on the 520-muscle solve**, and
+2.9 % median / +17 % worst across `StaticEquilibriumBenchmarkTests`' three real poses
(236.7 → 243.6 ms median). No trade had to be made.

### Against the pre-registration: PARTIAL, not solved

Registered before any result was read (`/tmp/qp-prereg.md`, quoted here because a pre-registration
that lives only in a scratch file is not one):

* **S1 — median solver slack < 0.10 pp → PASS** at 4.4994e-05 pp (2,200× under).
* **S2 — max solver slack < 1.617 pp → FAIL** at 21.981 pp.
* **S3 — cost ≤ 5× → PASS** at 1.03×.

S2 fails on a tail that is **466× the p90**. The likely mechanism is the rig's own screen rather than
the solver: a cell is read when the EXACT solution is at least `interiorMargin = 1e-3` inside the
box, and a muscle 1.1e-3 inside is one an exact solver calls interior and any finite solver may put
on the bound, after which `100·(a_l − a_r)/mean` divides by a number near `aMin`. That is a
hypothesis, not a finding: the worst cell is now printed with its pose, shape, effort, reference and
screened count so the next stage can attribute it. **The bar was not moved.**

### The claim still does not come back, and the reason has changed

`perMuscleLeftRightClaimIsSupported` stays `false`.
`testTheShippedFlagMatchesWhatTheMeasurementSupports` still passes, on R1 and R2 alone — R7 flipped
to PASS with this change and R3/R4/R5/R6 already passed. What is left is the moment-arm TAIL: worst
123.10 pp against OpenSim's central-difference column, at cells where the median is 0.98 pp, and
where OpenSim's two columns disagree with each other by more than the bar. So the ordering of causes
is now: solver 4.5e-05 pp, typical moment arm 0.98 pp, and an unattributed geometry tail at 100+ pp.
Next-step 24 is no longer one of several open questions; it is the only thing between this product
and the deliverable.

**One test changed direction, on its own instruction.**
`testTheShippingSolversOwnSlackIsLargerThanThePublicationFloor` asserted the defect and carried the
note "if it ever fails, the solver has been tightened and the whole per-muscle decision has to be
re-read — do not delete it, re-run the decision". It failed at 4.4994e-05 against 8.086, the decision
was re-run, and it is now
`testTheShippingSolversOwnSlackIsBelowWhatAnyClipCouldResolve`, asserting the median under
`floor/500` and the p90 under `floor/5` — a strictly tighter statement about the same quantity.

### What this did NOT do

* It did not touch a moment arm, a wrap solver, a floor or a gate. R1's 123.10 pp is unchanged and
  unattributed.
* It did not measure anything in a Release build or on a phone. All costs are Debug simulator.
* It did not change `eps_abs`/`eps_rel`, so `saturationActivationTolerance` is still 0.02 — now a
  ~200× conservative screen rather than a tight bound. Over-screening drops muscles from comparisons,
  which is the safe direction; tightening it is a `GaitLoadSummary` decision with its own gates.
* It did not measure the STRIDE case (next-step 26) or the `MovingPathPoint` residual (next-step 21).


## The calf muscles are not wrong: OpenSim's multi-wrap length is not a path length (2026-08-09)

Two findings said the same class of muscle was broken — a **systematic ~10-11 mm** offset on
`gasmed`/`gaslat140` across the whole running knee range, and a pre-registered falsifier (W1) that
**passes only because of where the 60-pose grid lands**. Both are now measured to the same cause,
and it is not this port. `MusclePathWrap.cpp` is unchanged; the deliverable is a third reference,
a gate for the class, and this record.

### The hypothesis the stage was given, tested and rejected

"OpenSim iterates between the wrap objects until the path settles; check whether the port does that
or solves them independently — solving two cylinders independently gives a path that is
systematically SHORT."

The port **does** iterate. `solveWrappedPathLength` runs `maxIterations = pathWrapCount < 2 ? 1 : 8`
outer passes, breaks on `|L_k − L_{k−1}| < 0.0005` and carries OpenSim's `noWrap`/`insideRadius`
order-swap heuristic (`MusclePathWrap.cpp:1319-1456`). Instrumented on `gasmed_r` over a 451-point
knee sweep it converges in **2 passes** at every pose. And the sign is the opposite of the
hypothesis: the port is systematically **LONG**, by +8.749 mm at knee 0 deg falling smoothly to
+0.877 mm at 45 deg.

### What the difference actually is

`GeometryPath::calcLengthAfterPathComputation` sums *straight segments measured between the wrap
points OpenSim reports* + *the spiral length OpenSim stored beside them*. For a two-cylinder path
those two halves describe **different paths**. Measured on `gasmed_r` at knee 0 deg, from OpenSim's
own output:

| | stored spiral | chord between its own tangent points | shortest helix between them |
|---|---|---|---|
| `GasMed_at_shank_r` | 0.038054 | **0.045350** | 0.046517 |

A curve joining two points cannot be shorter than the straight line between them, so the reported
total is **at least 7.30 mm below the length of any path through its own points**. The mechanism is
`WrapCylinder::_adjust_tangent_point`, which runs *only* when a muscle carries more than one
`PathWrap`: OpenSim moves the tangent points and nothing recomputes the arc. That is why the offset
is systematic — the adjustment shrinks smoothly as the knee flexes, so its derivative is a roughly
constant ~10 mm across the running range rather than a tail case.

The port's own path is self-consistent: a replica of the outer loop, validated **bit-identical**
(`|shipped − replica| = 0` at 451/451 poses), reports `stored == implied helix` to the last stored
digit for every spiral at every pose.

### The reconciliation, and it closes the gap completely

Repair OpenSim's total using **only OpenSim's own reported tangent points** — replace each cylinder
spiral's stored arc with the shortest helix between the two points it says that spiral connects —
and every term belongs to one path. On a knee sweep of the running band (161 triples, eps = 1e-4):

| quantity | vs OpenSim as reported | vs OpenSim reconciled |
|---|---|---|
| `gasmed_r` length, median / max | 4.203 / 8.749 mm | **0.0041 / 0.0451 mm** |
| `gasmed_r` moment arm, median / max | 10.484 / 17.441 mm | **0.033 / 0.270 mm** |
| `gaslat140_r` length, median / max | 2.781 / 7.280 mm | **0.0054 / 0.0273 mm** |
| `gaslat140_r` moment arm, median / max | 11.026 / 19.549 mm | **0.035 / 0.724 mm** |

The finding's own numbers (10.43 mm and 11.08 mm medians) are reproduced exactly; **100 % of them
is the reference's bookkeeping.**

That agreement is not the reconciliation picking a convenient reading. The port and OpenSim end on
**different tangent points** — at knee 0 deg the port's shank chord is 0.037385 m and OpenSim's is
0.045350 m — and the two self-consistent totals still agree to **4 µm**, because total path length
is STATIONARY with respect to sliding a tangent point along the surface at tangency: what the straight
segment gains the arc gives back. So a self-consistent total is insensitive to tangent-point
placement to first order, and OpenSim's 8.7 mm comes entirely from mixing one path's straights with
another path's arc. The port's arc (0.038023 m) and OpenSim's STORED arc (0.038054 m) agree to 31 µm
— it is OpenSim's reported POINTS that are the odd term.

The control that makes this attribution rather than correlation:
`TRIlong_{r,l}` and `BIClong_{r,l}` are the other four multi-wrap muscles; over the full clamped
elbow range they never engage two CYLINDER spirals at once (`TRIlong` engages at most one wrap at a
time, `BIClong`'s two are both ellipsoids) and their reference slack is non-negative — and the port
matches their reported length to **0.0000 mm** and their reported arm to **0.0046 mm**. Same code,
same fixture, same reference: only the muscles whose two cylinders both engage disagree.

### The third witness, and it owes this repo's arithmetic nothing

`reconciled` is a repair *this project* computes from OpenSim's numbers, so on its own it is an
argument. OpenSim's **analytic** moment arm is not: `GeometryPath::computeMomentArm` asks
`MomentArmSolver` for the generalized force a unit tension along the current path produces with the
wrap points held fixed. It reads the reported wrap POINTS and never calls
`calcLengthAfterPathComputation`, so it is blind to this defect by construction. If the defect is
the reference's bookkeeping, that column must side with the port.

It does. Over all 8 multi-wrap muscles, at every group in the fixture (their full clamped ranges,
440 groups), `|port − OpenSim analytic|` is **median 0.000518 mm, p90 0.1082 mm, max 1.0467 mm**
(p99 clears C2's 4 mm bar) — against `|port − central difference of OpenSim's reported length|` of
p90 **11.15 mm** and max **41.26 mm**. At the worst adversarial row the analytic gap is 0.99 mm.
Sample values on `gasmed_r` across the running knee band:

| knee | port | OpenSim ANALYTIC | OpenSim −dL/dq of its own reported length |
|---|---|---|---|
| 0° | +20.72 mm | **+21.76 mm** | +4.89 mm |
| 20° | ≈ +21.0 mm | **+22.26 mm** | +11.62 mm |
| 40° | ≈ +23.8 mm | **+23.83 mm** | +17.86 mm |

Two OpenSim columns, one port, and the port sits with the one that cannot see the defect. This also
corrects a `CLAUDE.md` entry: `gasmed_r`/`knee_angle_r` "analytic +0.021761 against central
+0.004891" was filed under "the envelope theorem is not exact where the wrap solution is marginal".
For the multi-wrap class it is the other way round — the analytic column is right and the central
one differentiates a length that is not a path length.

### W1 re-sampled on a grid that was not chosen after seeing the answer

The excursion is a spike, not a plateau: it appears where the reference's own `L(q)` **steps**, and
a uniform grid finds those by luck at any density (0.25 deg → 17.4 mm; 0.025 deg → 17.6 mm). So the
generator stops guessing and *scans for the jumps*: it samples `getLength` at a step finer than the
centred-difference stencil (2·eps = 0.0115 deg) and takes the largest second differences.

That rule — stated before any result and applied to all 8 muscles — lands on
**`gaslat140_r` at knee 26.01866 deg, where W1's quantity reads 41.26 mm against a 20 mm bar**,
independently reproducing the review's 41.29 mm at 26.01 deg. At that same pose the reference's
`L(q)` steps **6.2 µm over 0.0005 deg with no change in wrap-point count** (ours steps 0.2 µm), and
the port is **0.54 mm** from the reconciled column. OpenSim's central difference there is
**−18.23 mm** against the port's **+23.07 mm** — a sign flip manufactured by a 6 µm step.

### What shipped

* `tools/opensim_ref/dump_multiwrap.py` → `BioMotionTests/Fixtures/opensim_multiwrap.txt`
  (975 KB, 24 s to regenerate). Per sample it carries OpenSim's **raw solver inputs** — path points
  in ground and each wrap object's ground frame — so the port runs on OpenSim's own forward
  kinematics and the wrap solve is isolated from FK; plus `reported`, `reconciled`, `analytic`
  (OpenSim's own `computeMomentArm`) and `slack` (`min(stored arc − chord)`, negative = impossible
  geometry). Two grids: **blind** (51 uniform steps across each coordinate's clamped range, 408
  groups) and **adversarial** (the reference's own length jumps, found by scanning its second
  difference at a step finer than the stencil, 32 groups). Regenerates byte-identically.
* `BioMotionTests/MultiWrapReferenceTests.mm` — 6 tests, the gate for this class. Bars are **carried
  over from C1/C2/C5's pre-registered single-wrap bars** (0.005 / 0.004 / 0.005 m); no new threshold
  was invented. Measured over 440 groups: arm vs reconciled median **0.000009 mm**, p90 0.0309 mm,
  max **0.5415 mm**; arm vs OpenSim's own ANALYTIC column median **0.000518 mm**, p90 0.1082, max
  **1.0467 mm**; length vs reconciled max **0.0451 mm**; engagement disagreements **0 of 440**; 88
  rows where the reference's stored arc is shorter than the chord it spans, worst **−7.2957 mm**.
  Beside them, the same rows against `reported`: arm p90 **11.1487 mm** / max **41.2603 mm**, length
  max **8.7495 mm**. Subset run on the gate script: 6 tests, 0 failures, 0 restarts, 4.2 s.
* `tools/run_tests.sh` `MIN_TESTS` 474 → **483** (474 baseline + 3 from the QP round + 6 here),
  with the arithmetic in a comment so the next stage can extend it. (Now **484**, +1 from the
  leak re-run below.)
* `CylinderWrapValidationTests` W1 and C5 keep their pre-registered bars unchanged — **nothing was
  weakened** — and now carry the warning that densifying `poses.py` can push the multi-wrap
  component through 20 mm for the reference's reason, with the number and the file to read.

### What this did NOT do

* It did **not** change `MusclePathWrap.cpp`, `MomentArmComputer.mm`, any moment arm, any floor, any
  claim or anything a user sees. `perMuscleLeftRightClaimIsSupported` is untouched.
* It did **not** establish that the port's tangent points are the ones OpenSim would produce if
  OpenSim recomputed. The checkable statement is narrower: the port returns the length of a path,
  it is the path OpenSim's own tangent points describe, and the residual is the reference's.
* It did **not** reconcile ELLIPSOID spirals — `CalcDistanceOnEllipsoid` is a chord sum, not a
  closed form. Their slack is non-negative and their agreement is 0.0000 mm, so nothing needed it;
  a future model where an ellipsoid multi-wrap muscle disagrees would need that work first.
* It did **not** add poses to `tools/opensim_ref/poses.py`, so `opensim_moment_arms.txt` and
  `opensim_moment_arms_fd.txt` are byte-identical and no other test's numbers moved.
* It did **not** measure anything in a Release build or on a phone.
* It did **not** touch the two tests the QP stage left failing (`MuscleQPUnitsTests`
  `testResidualMechanismSweep`, `MomentArmErrorCancellationTests`' cross-muscle ORDER assertion).
  Both are registered-claim restatements that stage returned as questions; they are unrelated to
  this one and are still open. **Settled in the section below.**


## The leak experiment, re-run: the claim does NOT come back, and the largest remaining term is the REFERENCE (2026-08-09)

**Historical snapshot at commit `40a24df`.** The immediately following SimmSpline section preserves
the same rig after fixing the endpoint-extrapolation mechanism this snapshot localised; use that
section for current analytic-column numbers.

The re-run the whole workflow was for. Same rig, same 582 readable cells, same pre-registered R1–R7,
same 1.617 pp bar and 8.086 % floor. **Nothing was redesigned; the first two DIAGNOSTICS and the
later 24-row analytic-cell dump leave every gated number bit-identical, which is the proof they are
inert.**

### The answer, with the number

**`perMuscleLeftRightClaimIsSupported` stays `false`.** No row came back, no floor moved, no gate was
weakened, and `testTheShippedFlagMatchesWhatTheMeasurementSupports` prints `supported=false
shipped_flag=false`.

| Gate | Requires | Measured | |
|---|---|---|---|
| R1 | moment-arm leak < 1.617 pp | **123.0971 pp** (median 0.977) | ❌ |
| R2 | printed-number leak < 1.617 pp | **108.5752 pp** (median 1.045) | ❌ |
| R3 | three-muscle rig at the measured residual < 1.617 pp | **1.4022 pp** | ✅ |
| R4 | 0 unmodelled `PathWrap`s, none displayed | 0 / 0 | ✅ |
| R5 | straight-line CONTROL leaks > floor | 66.8824 pp, 271/549 cells over it | ✅ |
| R6 | ≥20 muscles at the worst cell, ≥30 cells | 21, 582 | ✅ |
| R7 | spread / printed error > 4 | **47.52** | ✅ |

R1, R2, R7 and the control reproduce the QP round's values to the last stored digit
(123.09713008193307 / 108.57519173214942 / 47.52166243963286 / 66.8824128835621). **R3 moved, and it
is a correction rather than a regression**: it read 0.568 pp when both of its solves carried the old
solver's slack, and 0.568 was two noisy answers partly cancelling. At the exact minimiser the same
quantity is **1.4022 pp** — still inside the bar, but by 13 % and not by 65 %.

### The tail's muscle was mis-named, and the correct name is the whole finding

`Cell.worstBase` is the base of the worst SHIPPED leak. It was printed beside `leakExact` as if it
were R1's muscle; those are different maxima and they parted company the moment the solver stopped
contributing. That is how this file came to record R1's worst as **`piri`** and **`glmed3`** —
muscles that carry **no `PathWrap` at all**, so the attribution the names invited ("a wrapping
residual") was wrong twice over. With `Cell.worstExactBase` added, R1's worst is on **`bflh140`** at
`grid_h060_k000_a+00` (central difference) and at `run_4_mid_swing` (analytic).

**`bflh140_r` has no `PathWrap`, no `MovingPathPoint` and no row in
`opensim_moment_arms_fd.txt`** — three fixed path points. Because the finite-difference source falls
back to the analytic column for muscles the FD fixture does not carry, `bflh140`'s moment arm is the
**same number in both reference matrices by construction**. And its left/right figure still moves
**126.44 pp** between them.

**And it is not an inference — the commit gate printed the arms.** `LEAK-METRIC worst_cell_arms` at
that cell, in millimetres, across all four sources:

| coordinate | ours | analytic | centralDiff | straightLine |
|---|---|---|---|---|
| `hip_flexion_r` | −57.249 | −57.249 | −57.249 | −57.249 |
| `hip_adduction_r` | 16.044 | 16.044 | 16.044 | 16.044 |
| `hip_rotation_r` | −5.762 | −5.762 | −5.762 | −5.762 |
| `knee_angle_r` | 29.526 | 29.526 | 29.526 | 29.526 |

**Zero. The muscle carrying the worst moment-arm leak in the entire experiment has no moment-arm
error at all**, in any column, including the straight-line control this project shipped until
2026-08-08. R1's 123.10 pp is 100 % other muscles' arms arriving through the solve.

So the tail is not a path error on the muscle that shows it. It is the SHARING STEP: the QP couples
every muscle crossing a joint, so a neighbour's moment-arm error lands on a muscle whose own path is
exact. `bflh140` is a hamstring — hip extensor and knee flexor — sharing the knee with `gasmed` and
`gaslat140`, the two muscles the section above measured the central-difference column to be wrong
about by **10.5 and 11.0 mm**. This is the same mechanism the 2026-08-08 retirement named ("it lands
on a muscle whose OWN path is modelled correctly"), now with a named muscle and a construction proof
instead of an inference.

**That construction proves the 123.10 pp CENTRAL-DIFFERENCE cell; it did not attribute the separate
42.46 pp ANALYTIC cell.** The next diagnostic printed all 24 muscles admitted by R1's screen at that
cell (`run_4_mid_swing`, `hip0.90_ankle1.20`, effort 0.9), sorted by their largest arm discrepancy:

| muscle | largest ours − analytic arm | own figure movement |
|---|---:|---:|
| `bflh140_r` | knee **16.059 − 13.713 = +2.346 mm** | **−42.462 pp** |
| `gaslat140_r` | knee **20.503 − 22.100 = −1.597 mm** | −2.904 pp |
| `vasmed_r` | knee +0.716 mm | +0.630 pp |
| `semiten_r` | knee +0.625 mm | −0.298 pp |
| `grac_r` | knee +0.584 mm | −3.158 pp |

So the registered hypothesis — "`bflh140` is fixed-point geometry, therefore the analytic tail must
arrive through a synergist" — is **REJECTED**. `bflh140` has three fixed path points and no
`PathWrap`/`MovingPathPoint`, yet its own knee arm is the largest discrepancy in the cell. The full
24-row `LEAK-METRIC worst_analytic_cell_row` dump is the durable instrument; the remaining question
is why this wrap-free path's derivative agrees at `grid_h060_k000_a+00` and differs by 2.346 mm at
`run_4_mid_swing`. That points at the kinematic/path-derivative seam, not at path wrapping.

### The largest remaining term, named the way the solver was

The predecessor stage turned "the model is wrong" into one measurable quantity by putting the exact
minimiser of the app's own objective in the subject slot. The same move here: **OpenSim's OTHER
column swept as a SUBJECT**, same pose, same τ, same truth solve, same statistic — so the reference's
self-disagreement lands on R1's scale.

| quantity, same rig and same statistic | median | p90 | worst |
|---|---|---|---|
| **the reference against ITSELF** | **5.2831 pp** | 17.014 | **126.4356 pp** |
| ours against the reference | 0.9770 pp | 13.862 | 123.0971 pp |
| ours against the ANALYTIC column alone | 0.4124 pp | — | 42.4623 pp |
| the sharing step (OSQP vs exact) | 4.4994e-05 pp | 0.0471 | 21.9811 pp |

Our leak is the **smaller** of the top two in **466 of 582 cells**, and the worst reference-vs-itself
cell (126.44 pp, `bflh140`, `grid_h060_k000_a+00`) is larger than R1's own worst.

**R1, as registered, is therefore not a measurement of this codebase.** It is maximised over the
worse of two `truth` definitions that differ from each other by 78× the reopening bar. No work on
`MomentArmComputer` can pass it while the gate is taken that way — the next stage's job is a
reference this repo can defend, not another wrap solver, and `MultiWrapReferenceTests`' reconciled
column is the first instalment of exactly that.

**And that is not a way out.** Against OpenSim's better-founded analytic column alone — the one that
never touches `calcLengthAfterPathComputation` and that the port sits with to 1.05 mm over every
multi-wrap muscle — our own worst is **42.4623 pp**, 26× the bar. The largest term is the reference;
the largest term this repository OWNS is that 42.46 pp; neither is under 1.617, so the claim stays
retired on the measurement and not on an argument.

### The two registered claims the QP round left failing, settled with evidence

**`MuscleQPUnitsTests.testResidualMechanismSweep`.** The registered claim is "eight decades of
τ-match weight buy nothing, so the leftover residual is a reachability distance". The λ ≥ 1e6 points
were never measurements, and the proof is a theorem about the objective rather than a conditioning
argument: minimising `½ε‖a‖² + ½λ‖Aa − τ‖²` over a convex set, optimality of `a₁` at `λ₁` and `a₂` at
`λ₂` add to `½(λ₂ − λ₁)(r₂ − r₁) ≤ 0`, so **the τ-residual at the minimiser is non-increasing in λ**,
box constraints and all. Measured: upright 0.2420 / 0.2373 / 0.2336 through λ = 1e4 and then
**0.5833** at 1e6 — a **149.6 %** RISE — and 1.4760 at 1e8 (531.8 %); dancer 0.3392 / 0.3407 / 0.3390
and then **0.9535** at both (181.3 %). A point whose residual went up did not minimise.

The exclusion rule is now that theorem, applied with 1 % of slack (dancer's λ=100 rises 0.44 % and is
admitted; the violations are 150-530 %). It replaces an ad-hoc rule that admitted everything except
the all-at-floor corner and excused one known-bad point in a comment — and **that point is no longer
bad**: `scaling = 0` + `polishing = 1` solves `dancer` at λ=100 to relative 0.3407, where it used to
return the all-at-floor corner. The λ grid was densified to `[1, 10, 1e2, 1e3, 1e4, 1e5, 1e6, 1e8]`
so the "≥ 4 usable solves" bar is met inside the valid range rather than by lowering it.

Result: **six admitted λ spanning five decades on both poses**, spread **0.0356** (upright) and
**0.0051** (dancer) against the unchanged 0.5 bar — tighter than the 0.229 / 0.029 of 2026-08-08,
because the comparison stopped putting a non-minimiser next to a minimiser. **The claim is narrowed
from eight decades to five and it is stronger inside them.** Two assertions were ADDED, both of which
would have failed on the state this test used to excuse: the rejected λ must be a contiguous
high-end tail (or the exclusion is picking points), and **the shipping `softPenalty = 100` must
itself be an admitted minimiser**.

**`MomentArmErrorCancellationTests`' cross-muscle ORDER assertion.** `XCTAssertNotEqual(truthOrder,
wrongOrder)` fails, and that is a real result: with the exact solver the perturbed rig sorts
`[alpha, gamma, beta]` both ways, so part of what used to reorder it was OSQP's own slack reshuffling
two near-tied entries. It does not reopen anything — the cross-muscle ranking is retired
STRUCTURALLY (nothing puts two muscles' efforts on one scale, and `MuscleOverlay.update(joints:)`
takes no muscle solve) — but the evidence had to stop saying something false. Measured and asserted
instead: the perturbation moves a ranking key by **0.3036** of activation while the closest adjacent
pair in the TRUE ranking is separated by **0.0033** — a ratio of **92.5×**. A sort order that
survives that survives by which muscle happened to move, not by a margin. Binary assertion out,
quantitative one in the same units in, magnitude assertion (159.03 % > 10 %) unchanged.

### What shipped

* `WrappedMomentArmLeakTests` — `Cell.worstExactBase`; the other reference swept as a SUBJECT (+930
  solves, +33 % on this file, every gated number bit-identical); `testTheReferenceDisagreesWithItself
  ByMoreThanTheGateAllows` (new, 484th test); a `LEAK-METRIC worst_cell_arms` line printing the worst
  cell's muscle per coordinate in MILLIMETRES across all four arm sources, which is what next-step 24
  asked for; and `worst_analytic_cell_row`, which prints every one of the 24 screened muscles at the
  analytic maximum and rejects next-step 34's neighbour attribution.
* `MuscleQPUnitsTests` and `MomentArmErrorCancellationTests` as above. No product code changed.
* Every stale `0.568 pp` reading corrected to the measured **1.4022 pp** across `GaitLoadSummary`,
  `ClaimSurfaceTests`, `WrappedMomentArmLeakTests`, `CLAUDE.md`.
* `MIN_TESTS` 483 → **484**.

### What this did NOT do

* It did **not** move a floor, weaken a gate, restore a row, or change a user-facing sentence. The
  retirement paragraph already said "a few muscles at a few joint angles where that leverage does NOT
  agree … and the reference disagrees with itself at those same places" — written as an inference by
  the previous stage, and now a measurement.
* It did **not** settle whether the `analytic` column is RIGHT, only that it is better founded and
  that the two columns cannot both be. R1 against it is still 42.46 pp.
* It did **not** attribute the 21.98 pp solver-slack tail (next-step 27) or the 42.46 pp analytic-column
  tail to a mechanism inside `MomentArmComputer`; the latter is now localised to `bflh140_r`'s own
  wrap-free knee row, but the cause of its 2.346 mm discrepancy is still open.
* It did **not** measure the STRIDE case, run in Release, or run on a phone.


## The 42.46 pp analytic tail was endpoint-cubic extrapolation (2026-08-09)

The previous section deliberately remains as the pre-fix measurement. Its last diagnostic turned
an unattributed product tail into one row: at `run_4_mid_swing`, `bflh140_r`'s own knee moment arm
was **16.059165 mm** in BioMotion and **13.713465 mm** in OpenSim, despite the muscle having only
three fixed path points and no wrap. That was enough to stop inspecting the wrap solver and follow
the kinematic transform instead.

### RED: both ends, value and derivatives

`SimmSplineExtrapolationTests` links the same `libnimble_ios.a` the app uses. For the symmetric
three-knot spline `(0,0), (1,1), (2,0)`, OpenSim continues the endpoint tangents. Before touching
Nimble, the new test failed six assertions:

| query | expected `(value, d1, d2)` | linked Nimble before fix |
|---|---:|---:|
| `x = −1` | `(−2, +2, 0)` | `(−3, +4, −2)` |
| `x = 3` | `(−2, −2, 0)` | `(−3, −4, −2)` |

Testing both value and derivative is load-bearing. Fixing only `calcValue()` would make finite
differences and path length look right while leaving Nimble's analytic Jacobian inconsistent with
the pose it differentiates.

### Root cause and GREEN

FullBody allows `walker_knee_r` to move from 0° through 140°. Its five nonlinear `TransformAxis`
`SimmSpline`s carry knots only through 120°, and `run_4_mid_swing` asks for 130°. Nimble's
`SimmSpline.cpp` already contained OpenSim's intended endpoint-tangent branches, but both the value
and derivative branches were commented out; an out-of-domain value therefore continued the first
or last cubic.

The fix restores exactly those two branches, not the unrelated commented exact-endpoint special
case. Both `build_ios/libnimble_ios.a` and `build_sim/libnimble_ios.a` were rebuilt. The low-level
test then passed, and a separate FullBody regression measured:

```text
SIMMSPLINE-METRIC pose=run_4_mid_swing muscle=bflh140_r coordinate=knee_angle_r
ours_mm=13.713464915 opensim_mm=13.713465000 delta_mm=-0.000000085
```

The tracked `nimble-patches/simmspline-linear-extrapolation.patch` contains the production change
and an upstream GTest. It reverse-checks against the patched vendored tree and apply-checks against
a clean clone of pinned Nimble SHA `c405b056fc35068027e03e0c384e84e12870b475`. The standalone
Python kinematics diagnostic now uses the same endpoint-linear semantics.

### The 582-cell result after the fix

No gate, floor, torque shape or screen moved. Only the linked kinematic function changed:

| quantity | before | after |
|---|---:|---:|
| analytic-only maximum | 42.4623 pp (`bflh140`) | **3.6932 pp** (`glmax2`) |
| analytic-only p99 | 9.94 pp | **3.3322 pp** |
| analytic-only median | 0.4124 pp | **0.3121 pp** |
| registered R1 maximum, worse of both references | 123.0971 pp | **123.0833 pp** |
| registered R2 maximum | 108.5752 pp | **108.5576 pp** |
| registered R1 median | 0.9770 pp | **0.6571 pp** |

The repository-owned analytic maximum fell **91.3 %**. Its new cell is
`grid_h090_k000_a+00`, shape `hip0.80_knee1.00`, effort 0.9. `glmax2` carries the largest figure
movement there (**−3.6932 pp**) even though its own largest arm error is 0.234 mm; `gasmed` carries
the cell's largest arm error, **−1.047 mm**, while its own figure moves only −0.081 pp. That is the
same coupling warning as the central-difference maximum, now visible on the analytic column too.

**The claim still does not come back.** The analytic-only 3.6932 pp maximum is 2.28× the unchanged
1.6173 pp reopening bar. More importantly, registered R1 is still maximised over OpenSim's analytic
and central-difference columns, whose multi-wrap bookkeeping disagreement produces the separate
123.083 pp tail. The fix closes a real product bug; it does not redefine the reference or weaken the
claim gate.


## MovingPathPoint uses exact SimmSpline semantics (2026-08-10)

The endpoint fix above corrected the joint transform, but `MomentArmComputer` still evaluated a
`MovingPathPoint` by drawing straight chords through every function's knots and clamping outside
the domain. That contradicted both OpenSim and the newly corrected Nimble evaluator. FullBody's
four moving points are the BIC long/short paths, so this was also the remaining named mechanism
behind the 4.414 mm `pro_sup` snapshot.

The RED was product-facing. In a synthetic path, the three-knot SimmSpline
`(-1,0), (0,0.1), (1,0)` must read `0.075 m` at `q=0.5`; the old chord returned `0.05 m` and counted
the point as approximated. At `q=2`, the old path clamped while OpenSim continues the endpoint
tangent. A FullBody assertion also pins `BICshort_l`'s moving-point z coordinate at
`−0.0135667107416569 m` for `pro_sup_l = 0`. A separate RED showed that the previous parser accepted
the invalid knot sequence `−1, 1, 0`; it now rejects non-finite or non-increasing knots and counts
the whole moving point as skipped.

The GREEN stores a read-only `dart::math::SimmSpline` per Simm axis and calls it directly on every
FK evaluation. Unsupported `NaturalCubicSpline` / `GCVSpline` functions remain visible in
`movingPathPointsApproximated`; the separate PiecewiseLinear endpoint-clamp gap remains documented
and is not falsely counted as cubic approximation. FullBody now reports:

```text
Moving 4 parsed (0 approximated, 0 skipped)
```

Re-running the affected ellipsoid validation after the implementation — not guessing from the old
attribution — gives:

| comparison | pre-exact snapshot | exact MovingPath |
|---|---:|---:|
| max vs OpenSim central difference | 4.414 mm | **2.679 mm** |
| p99 vs OpenSim central difference | 4.414 mm | **2.414 mm** |
| max vs OpenSim analytic column | 4.385 mm | **2.301 mm** |

Length and engagement remain exact (600/600). The six-pose causal sign gate sees 28 resolved
single-wrap effects and zero reversals; the straight line still has 135 analytic sign reversals,
the solver has zero. The remaining 2.679 mm is reported beside the ellipsoid-off result without
pre-attribution.

This rerun also exposed a test-design defect: deriving a sign threshold from the current no-wrap
maximum let an unrelated SimmSpline fix redefine the population. That runtime floor is retired.
Cylinder and ellipsoid total-sign tests use the original fixed 1 mm tripwire; the ellipsoid also
compares `(ours on − ours off)` with `(OpenSim on − OpenSim off)` at a fixed 1 mm reference effect,
and treats a zero actual effect as failure. The 1–5 mm band remains independently gated rather than
being hidden by E1/C1's 5 mm magnitude contract.


## The native-rate frame-budget notice names the budget, not the clip (2026-08-10)

At 240 fps, a four-second native-rate window asks for 960 samples and the bounded run takes 601.
That is true even when the clip is exactly 4.0 seconds long, but the import screen said “This clip
is longer than the analysis window”. Following that diagnosis and trimming the clip could make it
too short for the minimum contact count.

`OfflineDisclosureTests` now drives both a 30-second clip and the exact 4.0-second boundary through
the real sampler. The RED was the same false sentence for both inputs (three failed assertions:
the missing frame-budget cause twice and the invented clip length at the boundary). The notice now
says the four-second window exceeds one run's 601-frame budget at that video's rate and reports the
actual middle 601 frames / 2.5 seconds. The related disclosure and sampling gate is green with 19
tests, zero failures and zero restarts.

The selector immediately above the notice carried a second stale promise: it said native-rate mode
always covered four seconds for the “same number of model calls” as the 120-frame sparse mode.
`FrameSource.nativeWindowDisclosure` is now the single tested source of that copy. It says “up to
4 seconds”, names the 601-frame / 2.5-second-at-240-fps cap, and says processing time rises with the
video's frame rate. This changed disclosure only; the sampler arithmetic stayed unchanged and the
test floor was 486 at that commit.

A post-commit boundary review found the same cause classifier was also unsafe for sparse sampling:
at 10 fps, a 12.01 s clip has a 121st sample but the 120-frame cap stops first. The notice floored
`duration / step` to 120, mislabelled the run clip-limited, and emitted the native mode's four-second
window advice. The RED failed cause, location and no-window assertions. Sparse `wasTruncated` now
maps directly to `.budgetStoppedTheSparseScan` (the sampler only raises it when a next sample exists);
only native mode compares used frames with clip capacity. The 3 s / 240 fps competing-limit case is
also pinned: although the clip is shorter than four seconds, its 720 available frames exceed the
601-frame budget, so the budget remains the cause. Both boundary tests pass with zero restarts.


## Analysed cadence has one timestamp-derived source (2026-08-10)

The refusal path already used `GaitReport.framesPerSecond`, derived as `1 / median(timestamp
interval)`. The successful path carried a second value: AVAsset's nominal track rate was published
as `OfflineSessionRunner.sampledFrameRate`, copied into `.analysed`, unpacked by playback, and passed
to `GaitLoadSummary.make`. That is numerically hidden in native mode. In sparse mode it made a 10 fps
analysis of a nominal 30 fps track print “at 30 fps”, and scaled `frameRateNeeded` advice from the
same wrong base; timing, contact detection and dynamics themselves continued to use timestamps.

The RED deliberately supplied conflicting sources: a fixture report at 30 fps and a metadata value
at 90 fps. The screen sentence printed 90 fps and advised 164 fps; three assertions failed. The
minimal GREEN made the summary use the report and printed the report cadence. The subsequent green
refactor removed the duplicate channel entirely:

- `GaitLoadSummary.make` no longer accepts `framesPerSecond`;
- `GaitOutcome.analysed` carries only the report and gait plan;
- `OfflineSessionRunner.sampledFrameRate` is gone; the nominal rate remains local to decoding and
  frame-budget calculations, where it is the correct quantity.

The permanent regression asserts that the summary carries and prints the report's timestamp rate;
the type signatures prevent a caller from injecting metadata alongside it. Seven related suites
run **67 tests, 0 failures, 0 restarts**. One new test moved the fast lane's exact expected count
from 486 to **487** at that commit; no sampling, gait, dynamics or claim arithmetic changed.


## Contact timing stays visible without a muscle summary (2026-08-10)

> **Superseded later on 2026-08-10 by the contact-support publication boundary below.** The
> intermediate design still published a full `GaitReport` and accepted an optional
> `GaitLoadSummary`; the final product contract publishes detached `GaitTimingReport` only and the
> panel has no load-summary initializer.

An `.analysed` report replaces the posture panel because a run has no single representative pose.
When `GaitLoadSummary.make` returned nil, `GaitReportPanel` then skipped both the resolution and
contact-time blocks and showed only “The strides were measured but no contact produced muscle
output.” The product's one surviving left/right finding was already computed in `GaitReport` and
uses neither inverse dynamics nor the muscle solver, but this branch discarded it.

The RED required a nil-summary analysed presentation to retain resolution, contact time and report
flags, while selecting an explicit unavailable muscle section. It did not compile because no such
policy or report-owned timing type existed. The GREEN introduces `GaitTimingSummary(report:)` and a
tested `AnalysedPresentation`: timing and flags are non-optional; only muscle/load/honesty content
follows the optional `GaitLoadSummary`. Existing summary-based resolution APIs delegate to the same
timing type, so there is one copy of the advice arithmetic.

The nil branch now says the contact-time result is complete, no stance frame reached the downstream
muscle-analysis summary, and muscle-model checks are unavailable. It does not claim that a contact
failed to produce output, and it explicitly says this build would not publish per-muscle left/right
rows even after a successful downstream solve. `flags(report)` was already outside the old branch
and remains visible; the regression locks that rather than misreporting it as part of the defect.

The related summary/presentation suites run **54 tests, 0 failures, 0 restarts**. One new regression
moves the fast lane's exact expected count from 487 to **488**. No gait report, claim floor or
muscle computation changed; this is a visibility and ownership repair.


## The test gate fails closed, and E1 has its own receipt (2026-08-10)

The old runner treated an `-only-testing` invocation as exempt from its count floor. That made a
zero-test selection green. It also ignored XCTSkip totals and the captured `xcodebuild` return code,
accepted caller exclusions, and always appended the E1 skip — so asking it to run E1 selected and
skipped the same class. The log could say success while no required test supplied evidence.

The runner now has four explicit modes. `fast` owns the E1 exclusion and currently requires exactly
**514** ordinary tests (488 when this gate itself landed, plus three temporal-isolation, two
atomic-payload regressions, two live-anatomy contracts, three model-scaling contracts below, and
eight MHR-root/source-provenance and reload-lifecycle contracts, plus eight ground-trust,
availability-provenance, queue-ownership and pass-reset contracts).
`slow` owns the one exact E1
selector and requires exactly **1** test. `all` runs both lanes and is the commit gate. `subset`
requires at least one caller-selected
test, rejects all
skips, and prints `SUBSET PASS` rather than a gate verdict. Gating lanes accept no caller arguments,
so selection, configuration, repetition, destination, and result-path semantics cannot be changed
under the same receipt name.

The decision is made from both the log and a unique result bundle. A zero `xcodebuild` return code,
`TEST SUCCEEDED`, zero host restarts, case-sensitive xcresult `Passed`, exact total/passed counts,
and zero failed/skipped/expected-failure tests are all mandatory. Missing, malformed, or
contradictory evidence is a failure. The pure policy harness exercises the fail-closed branches
without a simulator; required FullBody/reference setup in XCTest now throws a failure rather than
`XCTSkip`, so the same rule also holds for an IDE run.

E1's 163-coordinate partition is not an open blocker: SHOULDER6 restored 169-coordinate coverage
and `testE1RunAll` passed in **5706.9 s** on 2026-08-07. The separate non-asserting V4 diagnostic
still reimplements an obsolete production solver and remains structurally stale; the slow lane does
not promote that diagnostic into evidence.


## Video whole-frame fallbacks split temporal analysis (2026-08-10)

`usedFallbackBBox` used to be a warning attached after the fact. The runner still calibrated from
that frame, primed/submitted it to Nimble, left it in the centred derivative window, and appended it
to gait input. On `video_012` that is 22 of 309 decoder slots. A fallback pose can look plausible in
isolation while its camera/root/scale jump is exactly the kind of discontinuity a derivative must
not cross.

The policy is now source-specific rather than treating fallback as failure:

- a **photo** whole-frame fallback remains the one available still pose and follows the existing
  static analysis path;
- a **video** whole-frame fallback remains `.success` and its projected pose stays visible for
  review, but `TemporalAnalysisExclusion.videoVisionWholeFrameFallback` branches before body-size
  plausibility, calibration, SG padding, Nimble, `usableBodyFrames`, gait, ID, and muscle;
- the result store refuses any later biomechanics route to an excluded frame, and every UI/load
  consumer asks the same eligibility-derived gate. The exact orange disclosure says the pose was
  not used for scale, motion, gait, or muscle calculations.

Continuity is keyed to `DecodedFrame.index`/`BodyFrame.frameNumber`, never the compact result-store
id. `DecodedBatch` retains the first and last REQUESTED slot even when decode failed, so leading or
trailing failures cannot silently move an endpoint inward. The static pass clears SG/hold/display
state before the next waiter; the gait pass groups trusted frames into explicit contiguous ranges.
Only a trusted frame at the real requested head/tail receives held-pose padding. Internal gaps get
no padding on either side: inserting a held pose there would assert stillness in an interval known
to be missing.

Three regressions cover the four source/fallback combinations, exact disclosure and fail-closed
store update, decoder-slot segmentation, and leading/middle/trailing endpoint policy. The existing
orchestration test now resets the real engine and proves the first `T−1` pushes produce no solve,
the `T`th does, and the reset's own `objectWillChange` cannot wake a waiter. The fast lane's reviewed
count therefore moves from 488 to **491**; E1 remains the one slow test.

Verification on this exact tree: the related offline/gait subset passed **66/66** with zero
failures, skips, expected failures, or restarts; the fail-closed shell harness passed **49/49**;
and the full commit gate passed fast **491/491** in **2416 s** plus slow E1 **1/1** in **6160 s**.
Both `xcodebuild` and `xcresulttool` exited 0, both xcresults were `Passed`, and both lanes recorded
zero failures, skips, expected failures, and restarts (`ALL GATE PASS`).


## Offline solve generations replace atomically (2026-08-10)

`OfflineResultStore.updateBiomechanics` independently nil-coalesced IK, ID, and muscle against the
existing frame. That made the gait second pass able to publish impossible mixed generations:
pose-only became `IK₂ + ID₁ + muscle₁ + motion₂`, and an ID-only pass kept `muscle₁` while replacing
its static-hold provenance. The method also changed any frame carrying a new muscle result to
`.success`, although solve routing does not own decoder/pose/timeout status.

The RED exercised pose-only, ID-only, and full pass-2 payloads plus an envelope whose status was
`.nimbleTimeout`: **2/2 tests failed**, showing the stale generation-1 fields and the status rewrite.
The GREEN introduces one `BiomechanicsPayload` built from one `SolveRecord`. Its IK is non-optional;
ID and muscle are optional and assigned directly, so nil erases. Static-hold provenance and
`MotionState` travel in the same value, while image, timestamp, decoder/model provenance,
`BodyFrame`, fallback admission, and `FrameStatus` are preserved. The video-fallback admission guard
still rejects the whole replacement. `temporalAnalysisExclusion` is now an immutable `let`; an
explicit `FrameResult` initializer retains the source-compatible nil default.

The two permanent regressions plus the video-exclusion seam pass **3/3** with zero failures, skips,
expected failures, or restarts; the related offline/gait suites pass **68/68**; and the fail-closed
shell harness passes **49/49**. They move the fast lane's reviewed count from 491 to **493**. The
full commit gate passed fast **493/493** in **2416 s** plus slow E1 **1/1** in **6140 s**. Both
`xcodebuild` and `xcresulttool` exited 0, both xcresults were `Passed`, and both lanes recorded zero
failures, skips, expected failures, and restarts (`ALL GATE PASS`).


## Live anatomy has one presentation gate (2026-08-10)

The live anatomy layer had three independent policies. `SkeletonARView` defaulted both tracking and
capsules to true, so the calibration surface drew muscle-shaped capsules without a disclosure.
On the tracking surface the renderer consumed the raw user toggle, while the Anatomy control and
`MuscleOverlay.anatomyOnlyNote` additionally required a loaded Nimble model. A missing `.osim`
therefore removed the control and sentence without removing the joint-only capsules; conversely,
tracking with no current frame could show both controls while the renderer necessarily hid them.

The RED added one explicit 16-row truth table before its production seam existed. The selected build
failed with xcodebuild 65 and zero executed tests because `LiveAnatomyPresentation` was not in scope;
that is recorded as a compile RED, not a behavioural assertion. A later review found that a pure
policy could remain unused while old wiring returned, so a second structural regression pins all
three production call sites. A temporary reverse mutation restored the old model/raw-toggle caption
gate; that selected test failed **1/1** on both the missing shared gate and the reintroduced Nimble
dependency, then passed after the mutation was removed. The GREEN introduces a pure policy
over `surface × tracking × current-frame × enabled`. Calibration always refuses the control and
layer. Tracking exposes the control only for a usable live pose, and presents anatomy only when that
pose exists and the user preference is enabled. `SkeletonARView` now requires both tracking state and
the final presentation value—there are no fail-open defaults. The renderer and disclosure consume
the same `anatomyIsPresented`; the Anatomy control consumes `showsControl`; and none of those gates
depends on Nimble model loading because the fixed capsule plan consumes only joints. The separate
IK/ID control retains its model-loaded gate.

The new truth-table regression passes **1/1**, the complete `MuscleOverlayClaimTests` suite passes
**7/7**, and the related calibration/orientation suites pass **14/14**, with zero failures, skips,
expected failures, or restarts; the fail-closed shell harness passes **49/49**. It moves the reviewed
fast count from 493 to **495**. The full commit gate passed fast **495/495** in **2364 s** plus slow E1
**1/1** in **6026 s**. Both `xcodebuild` and `xcresulttool` exited 0, both xcresults were `Passed`,
and both lanes recorded zero failures, skips, expected failures, and restarts
(`ALL GATE PASS`).


## Model scaling is relative to the loaded model (2026-08-10)

`scaleModelWithHeight` used three Rajagopal-era denominators (lower 0.88 m, trunk 0.52 m,
upper 0.54 m), then wrote each resulting subject ratio as an absolute uniform body scale. Those
numbers were not exact even for Rajagopal and were not the shipped FullBody model's references;
the write also discarded any non-unit or anisotropic default declared by a future model. There was
no load-time baseline, so an implementation that tried to repair the multiplication by reading the
current skeleton would compound on a repeated call, while a model reload had no state whose refresh
could be tested.

The RED added a dedicated ObjC++ `ModelScalingTests` target seam and generated its project entry.
All **3/3 failed** under the old implementation (xcodebuild 65, zero skips/restarts): passing
Rajagopal's own joint-centre geometry produced lower/trunk/upper scales
0.914191 / 0.891122 / 0.992541 instead of identity; a 1.12× subject produced
1.023894 / 0.998056 / 1.111646 instead of 1.12× the loaded defaults; and loading FullBody on the
same bridge then passing FullBody's own geometry produced 0.916012 / 0.926834 / 0.992541. The
failure is therefore the old reference/assignment semantics, not a missing fixture or test target.

Every successful model load now replaces one coherent baseline: the exact body-scale vector and
lower/upper references plus separate PELVIS→shoulder-midpoint and HJC-midpoint→shoulder-midpoint
trunk references. Subject ratios are measured against the cache matching their source alias, clamped in
`[0.7, 1.4]`, and multiplied component-wise into the cached defaults. Neither references nor scales
are read from a previously scaled state. Missing references fall back to the existing height ratio,
and a default-vector size mismatch refuses the scale. Rajagopal caches lower 0.8045, trunk
PELVIS/MHR_ROOT 0.4634/0.5331, upper 0.5360 m; FullBody caches
0.8061, 0.4820/0.5517, 0.5360 m. A successful reload replaces
all five cached values, including replacing an unavailable reference with invalid state rather than
leaking the earlier model's value.

The original three permanent contracts passed **3/3**: loaded-model identity, repeated 1.12×
idempotence, and Rajagopal→FullBody reload. The source-specific fourth contract and strengthened
reload arm are recorded in the MHR_ROOT section below. At this commit's historical receipt, the
scaled dancer plus those contracts passed **4/4**; the broader bridge,
IK, calibration, body-plausibility and offline-orchestration set passes **61/61**, all with zero
failures, skips, expected failures, or restarts. The dancer now uses FullBody-relative ratios
1.044 / 1.094 / 0.997 and converges at 2.4586 cm marker RMS. These tests move the reviewed fast
count from 495 to **498**; the fail-closed shell harness passed **49/49**. The full commit gate for
this exact change passed fast **498/498 in 2365s** plus slow E1 **1/1 in 6025s**. Both lanes report
`xcodebuild` 0, `xcresulttool` 0, and zero failures, skips, expected failures, or restarts; the
runner's final verdict is **ALL GATE PASS**.


## MHR root anatomy and marker provenance are source-specific (2026-08-10)

The offline root used one stable joint id for three different anatomical points. Raw MHR joint 1 is
**not** the bilateral hip-joint-centre midpoint: the pinned shipping fixture measures
**15.081552 mm** between them (the source rest skeleton measures 19.2 mm). OpenSim `PELVIS` is a
third point, the pelvis body origin, about 96.6 mm from the model's bilateral HJC midpoint. Mapping
raw joint 1 straight to `PELVIS` therefore imposed a triangle the model could not satisfy.

The fix keeps identity and anatomy separate:

- `hips_joint` remains the stable body-joint id. Live/default frames resolve it to `PELVIS`; MHR
  frames carry an explicit `MHR_ROOT` override. The input coordinate remains raw MHR joint 1 — it is
  not rewritten to pretend that the source root equals its HJC midpoint.
- `NimbleBridge` reserves `MHR_ROOT` as a model-side bilateral-HJC-midpoint proxy attached to the
  pelvis. Its local offset is computed through the full inverse pelvis transform and divided by the
  loaded pelvis scale, so native rotation and anisotropic/non-unit scale are respected. `PELVIS`
  remains registered at the pelvis body origin for live and legacy callers.
- `OneEuroFilter`, offline/gait fixtures, test transformations and `NimbleEngine` preserve the
  source marker override. Resolution first whitelists the stable joint id; an empty override or an
  override on an unknown id fails closed.
- TRC uses the effective source marker name. A joint that changes alias across frames, an empty
  alias, or two ids collapsing to one marker refuses export. If TRC fails while MOT/STO succeeds,
  the share bundle carries `BioMotion_export_warnings.txt`; failure to create that disclosure stops
  the partial share and surfaces an alert.

Scaling now observes the same distinction. MHR's synthetic straight-limb input emits `MHR_ROOT`
and measures trunk length from source HJC midpoint to shoulder midpoint; it no longer measures from
raw joint 1 and labels that point `PELVIS`. Each successful load caches both trunk denominators:
FullBody is **0.4820 m PELVIS / 0.5517 m MHR_ROOT**, Rajagopal is
**0.4634 / 0.5331 m**. The input alias chooses the matching denominator; lower/upper references and
the loaded default body-scale vector remain the same baseline. The current dancer reaches
**1.2758 cm** RMS after this source-aware scale, versus 1.5365 cm unscaled.

Model loading is transactional across layers. Native code constructs the candidate skeleton,
marker map and all five scaling caches before swapping them. A failed reload retains the old model
and its DOF mask; a successful reload clears the now-invalid index mask and session state. The Swift
engine likewise keeps reporting the retained model after failure, while success clears SG, hold,
muscle-timestamp, activation-filter and published-result history before a new frame can run.

The geometry receipt is deliberately two rulers. On the fixture's root/HJC/shoulder subset, the
legacy `PELVIS` triangle has **38.912142 mm RMS** and the `MHR_ROOT` proxy has
**7.209843 mm RMS**. On all 20 dancer markers, the current solve is **1.5365 cm RMS**, max
**3.7268 cm**; shoulders now dominate and MHR_ROOT itself is 1.33 cm. The root fix does **not**
certify downstream muscles: the unscaled dancer's shipped relative torque residual is
**0.5939547**, so standing remains the controlled muscle benchmark.

TDD receipts before the commit gate: the first four-contract run passed 1/4 and failed exactly on
successful-reload mask cleanup, MHR trunk scaling and gait provenance; failed reload preservation
already passed. The export-disclosure RED was a compile failure because its seam did not exist.
After implementation the six focused contracts pass **6/6**, the non-heavy related set passes
**71/71**, the three IK measurements pass **3/3**, and the two updated shoulder/QP measurements pass
individually, with zero failures, skips or restarts. Two neighbouring test-fixture changes are
explicit rather than hidden gate relaxation: the gait end-to-end contact has 7 frames instead of 5
so a 5-tap window has more than one interior candidate; production contact rules are unchanged.
StaticHold keeps exact 0 warm→warm drift and allows only **<1e-8 rad** cold→warm, against a measured
1.5725e-9 numerical settle. Eight new methods move the reviewed fast count from 498 to **506**.
The commit gate then passes in full: fast is **506/506** in **2225 s** and slow E1 is **1/1** in
**6052 s**. Both lanes have `xcodebuild rc=0`, `xcresulttool rc=0`, `xcresult=Passed`, and zero
failures, skips, expected failures or restarts; the combined runner reports **ALL GATE PASS**.


## Ground-plane trust is necessary but not sufficient for dynamics (2026-08-10)

This section records the earlier floor-provenance repair. It remains a real necessary gate for any
future dynamics path, but it is **not** sufficient: the contact-support audit immediately below
found that neither bundled model can pass the earlier model/solver capability gate.

The rolling ground estimator used to expose its provisional floor as though it were calibrated.
On its very first call, `solveIDGRF` placed the floor 1 cm below the lowest observed foot. That
construction guarantees an apparent contact even for an airborne or arbitrary first pose. Although
the native bridge already distinguished provisional from trusted ground, Swift never read
`groundHeightTrusted`: provisional torques, GRFs, CoPs and muscle output therefore escaped into the
live UI, offline payload, gait residuals and export history.

The boundary is now explicit and same-generation:

- `DynamicsAvailability` travels with `SolveRecord` and `BiomechanicsPayload`. The current product
  store additionally requires a successful, temporally eligible tracked body plus owner-matched
  body/IK timestamps; `.available` requires a same-generation ID, and muscle has its own timestamp
  gate. Waiting, policy withholding, missing root-y, untrusted ground and native ID failure each
  keep dynamics nil and carry their own explanation. Missing is never rendered or aggregated as a
  measured zero.
- For a capability-valid model, the native call can observe the current feet and trust is tested
  after the observation. Samples 1–29 remain pose-only; sample 30 can satisfy the floor requirement
  on that same call. It is only *eligible for later gates*, not automatically publishable. The
  bundled models stop earlier at `.contactSupportUnavailable` and never run this load solve.
- Savitzky–Golay endpoint replay supplies derivative context only. Repeating one photo does not
  manufacture 30 independent floor observations. `setExplicitGroundHeightY` remains the ordered
  seam for a real external floor, but with either floor source both bundled models remain pose-only
  because contact support fails first.
- A gait second pass over the same continuous clip clears SG/hold state, IK warm start and QP warm
  start while preserving that clip's ground window. It invalidates all pass-one dynamics before the
  first pass-two submission; an incomplete/timeout row stays explicitly pose-only rather than
  retaining static physics. A new clip, tracking loss, or ARKit
  `.resetTracking` boundary performs the full reset and discards both the floor and stale physics.
- Live and offline presentation now show the availability reason instead of green `0.0 Nm/kg`,
  `0.00 BW`, `0.00 N/kg`, an old muscle overlay, or a misleading “warming up” label. Offline payload
  replacement clears old ID/muscle/gait fields atomically and keeps pose-only availability explicit.

The RED contracts proved three concrete leaks: sample 29 still produced ID/muscle/history, an
untrusted gait frame still produced numeric outcome/summary state, and the first warm solve after a
session reset still published physics. All three failed before the implementation. The same
selection then passed **3/3 in 58 s**; the bridge/orchestration/static related set passed **46/46 in
149 s**. The gait falsifier was corrected to pin the estimator's own provisional floor and compare
only contact-backed residuals: calm **0.030604 BW**, violent **1.782345 BW**, against the unchanged
**0.5 BW** gate, and its focused run passed **1/1 in 208 s**. Five initial contracts pin atomic
availability replacement and same-clip ground preservation. Review then found three fail-closed
edges: the display-filter dictionary could race reset across queues; recording history was written
before the generation guard; and an unavailable/timeout gait row could retain or aggregate old
physics. Their RED ran **2/2 failed** with the exact residual and queue-ownership leaks, plus one
expected compile RED for the missing replacement-pass seam. Moving filter cleanup onto the solver
queue, recording only inside the guarded main publication, pre-clearing pass-one dynamics, and
checking availability again in `GaitLoadSummary` makes the focused selection pass **3/3 in 10 s**.
The complete related selection (`GaitDynamics`, `GaitLoadSummary`, bridge, disclosure,
orchestration and static-hold suites) then passes **113/113 in 826 s**, with zero failures, skips,
expected failures, or restarts. The eight new contracts move the reviewed fast count from 506 to
**514**. The full `tools/run_tests.sh all` commit gate then passed: fast **514/514 in 2245 s** and
slow E1 **1/1 in 6066 s**. Both lanes returned `xcodebuild = 0`, `xcresulttool = 0`, `Passed`, and
zero failures, skips, expected failures, or test-host restarts; the runner ended with
`ALL GATE PASS`.


## Bundled models fail closed without validated foot contact support (2026-08-10)

The model/solver pair had no load-bearing contact representation. Static inspection finds an empty
`ContactGeometrySet` in both production resources (`FullBody.osim:51285-51288` and
`Rajagopal2016.osim:14282-14285`). The near-CoP call takes foot bodies and wrench guesses, then
closes Newton–Euler near those guesses; it does **not** impose a foot support polygon, a unilateral
normal-force constraint, or a friction cone. A low heel and a trusted ground height can say where a
foot appears to be. They cannot say where or in which directions that foot may transmit force.

That makes every prior floor-only release condition insufficient. An explicit floor, observation
30, a static hold, a gait second pass, a longer clip, or a cleaner re-shoot still cannot create the
missing mechanics. The production policy is now fail-closed:

- `NimbleBridge.hasValidatedFootContactSupport` is false for both bundled models, and production
  `solveIDGRF` returns nil before mutating the rolling floor or exposing an unconstrained wrench.
- `NimbleEngine` publishes `.contactSupportUnavailable` before both static and gait dynamics. IK,
  fixed-colour anatomy, kinematics-only posture findings, bilateral mean contact duration,
  variation/count, timing resolution, and the resolution-qualified left/right contact-time
  comparison remain available. Specific contact intervals and cadence/stride period stay internal.
  Joint torque, GRF, CoP, muscle effort, and gait-load fields stay nil; no history or old coloured
  overlay survives.
- The availability explanation states the missing capability and explicitly says re-filming cannot
  enable it. Ground trust remains a second gate for a future model/solver pair, never an alternative
  to contact support.
- The research `GaitForceModel`, `GaitLoadSummary`, earlier static-equilibrium probes, and all
  figures such as 780.71 N, 18.55 Nm, or 0.030604 BW are retained as **historical raw diagnostics**.
  They checked algebra, units, frames, or falsifiers inside an unconstrained solver; they are not
  validated biological measurements and are not product output.
- The publication seam itself is timing-only, rather than relying on the UI to hide optional data.
  `GaitOutcome.refused` and `.analysed` carry a detached `GaitTimingReport`; it contains copied
  resolution, contact duration/count/uncertainty, timing refusals and flags, and cannot retain the
  research report's force/peak fields, a `GaitPlan`, residuals or `GaitLoadSummary`.
  `GaitReportPanel` has no load-summary initializer, and every stored frame strips the native gait
  force/residual outcome while preserving stance/flight/outside verdicts.
- Append, replacement, and late capability downgrade share one product frame projector. It requires
  success, temporal eligibility, a tracked body, and owner-matched body/IK timestamps before pose
  state survives. Same-generation ID and muscle timestamps are gated independently: stale ID
  becomes `.inverseDynamicsFailed` without erasing valid IK/verdict, while stale muscle removes only
  muscle. Capability false still clears all ID/muscle output from an otherwise valid generation.
- `OfflineResultStore.hasValidatedFootContactSupport` records the model/session capability instead
  of inferring it from a count of second-pass frames. Its permanent notice remains beside immediate
  moving/flight/outside advice. For both bundled models the runner publishes analysed timing, then
  returns before `makePlan`, pass-two invalidation or frame replay. A
  `.contactTooShortForACleanDerivative` refusal blocks only that future private plan; it neither
  refuses timing nor suppresses a contact asymmetry that clears the timing floor.

Reopening dynamics requires a model/solver pair with an explicit, reviewed foot-support
representation and adversarial tests for at least the support domain, unilateral loading, friction,
contact selection, and wrench-frame conversion. It then still has to pass ground provenance and all
downstream claim gates. Merely setting `hasValidatedFootContactSupport` true is not evidence.

Test receipt: after the implementation and documentation were frozen, the broad focused selection
(`GaitLoadSummary`, disclosure/orchestration, gait dynamics, claim surface, TRC export, bridge and
static-hold suites) passed **139/139 in 240 s** with `xcodebuild = 0`, `xcresulttool = 0`, `Passed`,
and zero failures, skips, expected failures, or test-host restarts. The unsigned arm64-simulator
Release build passed in **30 s** (`xcodebuild = 0`); its 13,006,072-byte Mach-O retained only the
public fail-closed `solveIDGRF` selector and contained neither the test-diagnostic selectors nor the
removed raw-ID selector/string. The shell gate passed **49/49**. Finally,
`tools/run_tests.sh all` passed the exact commit gate: fast **519/519 in 1676 s** and slow E1
**1/1 in 6150 s**. Both lanes returned `xcodebuild = 0`, `xcresulttool = 0`, `Passed`, and zero
failures, skips, expected failures, or test-host restarts; the runner ended with `ALL GATE PASS`.


## Vendored Nimble OpenSim parsing is fail-closed and semantics-preserving (2026-08-11)

The vendored parser carried two different risks. Its upstream Release paths used disabled
`assert()` checks and unchecked XML/topology dereferences. Their outcomes were path-dependent: a
genuinely null joint could crash, the older local null-joint workaround could silently substitute a
`WeldJoint`, and other malformed input could still select a wrong specialized joint and return
success. Separately, successful specialized-joint fast paths
could discard linear slope/intercept, ignore explicit coordinate mappings, accept an unrepresentable
negative translation axis, or turn a remapped six-axis root into a three-DOF Euler joint. Those are
scientific-data corruptions even when they do not crash.

The parser slice is now isolated as two reviewed commits on the nested branch
`biomotion/ios-static-c405b05`:

- `7ecf61c` replaces Release-disabled/fallthrough rejection points for missing
  `SpatialTransform`, unsupported transform functions, non-orthogonal specialized paths and
  unsupported CustomJoint DOF counts, then refuses any null joint or child body before use.
- `6b082fd` extends validation across exact six-axis, coordinate, parent and root topology; root
  locked CustomJoints and root WeldJoints use the skeleton root API. It also preserves valid
  CustomJoint semantics: specialized joints require canonical functions, exact dimensionality and
  matching `drivenByDofs`; otherwise the parser retains a CustomJoint. Non-identity slope/intercept
  survive, the `-q+b` axis-flip intercept sign is corrected, and six linear axes cannot fall into
  the three-rotation Euler branch.

The tracked `nimble-patches/opensimparser-fail-closed.patch` is the combined baseline-to-head diff
from pinned Nimble `c405b056fc35068027e03e0c384e84e12870b475`. It reverse-checks against the current
tree, is 562 lines, and has SHA-256
`50701bb5ae848f9192c1c0e5ffcfdef4a94314f98a95c00e6f7390b751482b3b`. The earlier
`opensimparser-null-joint-fallback.patch` was deleted because applying it would restore the silent
Weld substitution this work proves unsafe.

The regression was deliberately causal. Against the pre-fix archive, an unsupported function
returned success without exercising the null/Weld path. Rajagopal remained at 37 total DOFs, but
the bridge treated the malformed load as a successful replacement, cleared its one-DOF active mask
and changed the free count from 36 to 37. Non-Cartesian/negative axes, unknown parents and unknown
coordinates leaked similarly. A `2*q`
translation moved only `0.1 m` at `q=0.1`, and a valid `pelvis_tx -> pelvis_ty` remapping still made
`pelvis_tx` move X while `pelvis_ty` did not. The final linked-archive checks make all malformed
cases fail transactionally, produce `0.2 m` for the scaled function, preserve `-q+0.5`, assert the
reflected fixture's root as `CustomJoint<6>`, and make the remapped `pelvis_ty` drive X/Y exactly as
the XML states. The same focused run logged the unmodified Rajagopal fixture at 37 total DOFs; that
root/count description is a fixture receipt, not a general parser invariant.

Final focused receipt: the transaction test, direct representation test and bundled-model test
passed **3/3**, with zero failures, skips, expected failures or test-host restarts. Both simulator
and device archives rebuilt successfully. The final source hash passed simulator Release, device
Release and simulator non-`NDEBUG` `-fsyntax-only` compilation with zero warnings/errors; an
independent staged-diff review reported no blocker/high issue.

Full outer commit-gate receipt (`tools/run_tests.sh all`): fast passed **519/519 in 1690 s** and
slow E1 passed **1/1 in 6170 s**. Both lanes returned `xcodebuild = 0`, `xcresulttool = 0`,
`Passed`, and zero failures, skips, expected failures or test-host restarts; the runner ended with
`ALL GATE PASS`.

Fork publication receipt (`git ls-remote --symref`): remote `fork` is
`https://github.com/shengyang998/nimblephysics.git`; public branch
[`biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05)
resolved to `6b082fd0feec9cac7bc2d21b15bc63bd6225c58f`. Its remote symbolic `HEAD`
resolved to `refs/heads/master` at
`c405b056fc35068027e03e0c384e84e12870b475`.

This closes the OpenSim parser slice, not the complete iOS port. At that receipt, the nested working
tree still held older, deliberately unstaged CMake/GUI/mesh/collision/dependency changes; none was
included in `7ecf61c` or `6b082fd`. The collision subset has since been isolated below. CMake, GUI,
mesh and dependency work remains uncommitted until separately licensed and validated.


## The unavailable iOS collision backend now fails closed (2026-08-11)

The old local iOS collision stub was not a harmless no-op. It redeclared a shortened version of
`DARTCollisionDetector`, defined only `create()`, and returned `nullptr`. Its object file also had no
ordinary static-archive reference from `CollisionDetector.cpp`; a factory-only link extracted
`CollisionDetector.cpp.o` but not `DARTCollisionDetector_ios.cpp.o`, so the registry did not even
contain the `"dart"` key. Direct consumers could receive null, while `ConstraintSolver`,
`BoxedLcpConstraintSolver`, and `World` dereferenced the result during construction.

The regression was causal and crash-safe. Against the old archive, the five-method focused suite
failed **0/5**: direct creation returned null, factory lookup could not create `"dart"`, and the
consumer cases deliberately stopped after that exact preflight rather than dereferencing the known
bad pointer. The independent archive probe failed at the `-why_load` receipt because the stub member
was not extracted. Those are two different REDs for the runtime and static-link defects.

The reviewed implementation now has one explicit contract:

- `DARTCollisionDetector_ios.cpp` includes the real class declaration and defines the full virtual
  surface. `getType()` / `getStaticType()` preserve `"dart"`; construction and every group,
  collision, distance, object, refresh, or clone operation that would require Assimp/libccd throw
  `std::runtime_error("DARTCollisionDetector is unavailable in this iOS build because Assimp/libccd collision support is not linked.")`.
- `CollisionDetector::getFactory()` initializes the singleton pointer and registration through
  function-local statics. Its creator directly references `DARTCollisionDetector::create()` and
  `getStaticType()`, so ordinary archive extraction does not depend on an unreferenced global
  registrar.
- `World` now initializes `mRecording` to null, constructs and installs its solver, and allocates the
  raw `Recording` only after that throwing step succeeds. A partially constructed `World` therefore
  owns no recording that its never-run destructor would have needed to delete.
- The earlier fake `aiScene`/libccd header types and shortened Assimp license/whole-header guard were
  restored to their pinned upstream bytes; they are not part of this commit or a substitute backend.

Verification is layered instead of treating one green XCTest line as sufficient:

- Both arm64 simulator and device `nimble_ios` archives rebuilt; each has exactly one
  `CollisionDetector.cpp.o` and one `DARTCollisionDetector_ios.cpp.o`. The factory member has the two
  intended undefined references, while the stub defines every method, vtable and typeinfo and has no
  Assimp, `ai*`, libccd, real DART collision-object/group, cache, or `mRegistrar` dependency.
- The final focused receipt passed **5/5 in 7 s**, with `xcodebuild = 0`, `xcresulttool = 0`,
  `Passed`, and zero failures, skips, expected failures, or test-host restarts.
- `collision_static_link_probe.sh` compiles a consumer that includes only `CollisionDetector.hpp`,
  links the simulator archive with dead-stripping and without `-all_load` / `-force_load`, requires
  both members in `-why_load` and the link map, rejects accidental solver/World extraction, and runs
  to `ARCHIVE_FACTORY_PROBE_PASS` on `BioMotion-CI`.
- `collision_world_leak_probe.sh` makes a fresh current-port macOS host-native build and runs two
  separate `leaks --atExit` processes. The positive control reports one deliberate
  `leakForPositiveControl()` allocation (160 bytes after allocator rounding); 32 `World()` plus 32
  `World::create()` rejection attempts report **0 leaks for 0 total leaked bytes** and end in
  `WORLD_COLLISION_REJECTION_LEAK_PROBE_PASS`. This proves the exception-order repair with the host
  allocator, not the iOS allocator. That historical receipt predates the standalone CMake export;
  the later migration section records the current revalidation boundary separately.
- The shell gate-policy harness passes **49/49**. The full outer commit gate passed fast
  **524/524 in 1685s** and slow **1/1 in 6172s**. Both `xcodebuild` and `xcresulttool` returned 0,
  both result bundles reported `Passed`, and the receipts contained zero failures, skips, expected
  failures, or runner restarts before `ALL GATE PASS`.

The nested source change is isolated in
`23e359d516e3d6da38cda0207ab057c37c9c7779` (`fix(ios): fail closed without collision backend`),
containing exactly `CollisionDetector.cpp`, `DARTCollisionDetector_ios.cpp`, and `World.cpp`. It is
published at
[`shengyang998/nimblephysics:biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05),
and `git ls-remote` resolved that branch to the same SHA. The tracked
`nimble-patches/ios-collision-fail-closed.patch` is byte-identical to that commit's three-file diff,
reverse-checks against the branch head, is 210 lines, and has SHA-256
`d8c50a8f58e4e4a79c43d4e1be173e9d4f5d539d3be44b717c561ac97b304399`.

This closes an unsupported-API boundary; it does **not** add collision/contact simulation, validate
foot support, or reopen torque, GRF, CoP, muscle-effort, or gait-load product output. This isolated
patch did not export the older CMake/source-manifest port; the later standalone-CMake section closes
that separate fresh-clone boundary.


## The SAM3DBodyPose artifact and license contract is now pinned (2026-08-11)

The 1.3 GiB model must stay outside Git, but accepting a directory merely because it is named
`SAM3DBodyPose.mlpackage` or `.mlmodelc` is not a supply-chain contract. The repository now carries
the small, reviewable authority files `BioMotion/Resources/SAM3DBodyPose.lock.json` and
`SAM-LICENSE.txt`, plus `tools/assetpack/verify_model_lock.py`.

The lock fixes the asset-pack ID to exactly `sam3d-body-pose`, the upstream repository and export
recipe commit `faa96fc8f9e651131579849701e0fa682b4d4b9c`, both source-input hashes, conversion and compile
toolchains, the exact three-file source package, exact five-file compiled model, and every Core ML
input/output name, type, dtype, shape, optionality, and shape-flexibility flag. The 8,204-byte SAM
license is checked byte-for-byte at SHA-256
`b3a5a0e2d973ab80e6610ccf1cffc40756050d0ace3cd4fec879b3ec290b2e9b`.

The verifier reads strict UTF-8 JSON from non-symlink regular files, rejects duplicate/unknown keys
and unsafe paths, and exact-matches file sets, directories, sizes, hashes, and Core ML metadata.
Its self-contained suite passes **23/23**, covering ID/version drift, duplicate JSON, missing/extra
or symlinked artifacts, license mismatch, and interface/schema drift. Repository mode passes, and
the real local source `.mlpackage` and staged compiled `.mlmodelc` both pass their respective
full-tree/interface modes. The external dirty SAM export worktree was read-only throughout.

This was deliberately the first supply-chain slice, not a claim that delivery was finished. At
that receipt, audit of the existing paths found these release blockers:

- `package.sh` did not invoke the verifier or re-extract and verify its AAR (closed by the next
  slice below);
- `upload.sh` reads credentials before artifact validation and lacks a receipt and `--verify-only`;
- the historical App Store Connect version 1 AAR omits both the lock and full license and is not a
  compliant shipping artifact;
- developer bundling and runtime loading still accept unverified artifacts; and
- package/upload/runtime enforcement, NOTICE, app resources, and a replacement asset-pack version
  remain separate tested commits.

The asset-pack README no longer recommends raw packaging or upload bypasses while those gates are
open. No upload or other external mutation was performed by this contract slice.


## The SAM package and receipt are now fail-closed and atomically published (2026-08-11)

`tools/assetpack/package.sh` is now the only approved local package entry point. It fixes `/bin/bash`,
`/usr/bin/python3`, `/usr/bin/xcrun`, and a trusted system PATH; snapshots Manifest/lock/license into
a private mode-0700 transaction first; and then validates the frozen repository contract and exact
Xcode 26.4 / build 17E192 / Core ML 3520.5.1 / `ba-package` 1.2 toolchain back-to-back. It verifies
the exact source package, compiles privately, verifies the exact compiled tree and complete interface,
and stages only these three Manifest selectors:

- `SAM3DBodyPose.mlmodelc`;
- `SAM3DBodyPose.lock.json`; and
- `SAM-LICENSE.txt`.

The temporary AAR is inspected with `/usr/bin/xcrun aa list` before extraction. Its allowlist fixes
all 13 directory/file entries, exact entry types, and every regular-file `DAT` size; unsafe paths,
links, unsupported archive metadata, missing/extra entries, and expansion-size drift fail before
`aa extract`. The privately extracted Manifest, lock, and license must be byte-identical to the
authority snapshot, and the extracted compiled model must again pass its exact tree and interface
contract. Hashing before/after list and extraction rejects an AAR that changes during the gate.

The real compiler exposed a reproducibility edge that the gate correctly caught: consecutive
compiles produced different SHA-256 values for only the 503-byte root `coremldata.bin`, while the
other four compiled files were byte-identical. The difference was solely the order of four protobuf
map entries containing coremltools metadata. Normalization is therefore deliberately narrower than
semantic reserialization: it enumerates only those map-entry permutations and writes one only when
the resulting *complete raw file* reconstructs the SHA-256 already pinned by the lock. Any changed
metadata value, topology, other file, or other byte remains a hard failure. Root/file/parent symlink
and containment checks run before the private compiler output is changed.

The strict 722-byte JSON receipt binds schema version, asset-pack ID, artifact revision, model base
name, AAR filename/size/SHA-256, and Manifest/lock/license filenames and SHA-256 values. `seal` copies
the AAR and all three authority files through opened regular-file descriptors into one random
mode-0700 sibling snapshot. Archive verification, receipt construction, and bindings read only that
snapshot; live inputs must then equal it before the complete receipt is atomically installed with a
same-filesystem no-clobber hard link. This closes pathname ABA and guarantees that a failed/partial
receipt write never touches the public final pathname. Both the standalone `receipt` gate and final
`publish` gate repeat the full archive extraction rather than trusting hashes alone. The AAR and
receipt become visible only as one `build/assetpack/release/` directory operation: first publication
uses one rename and replacement uses Darwin
`renameatx_np(RENAME_SWAP)`, eliminating the former two-rename crash window. The verified candidate
AAR, receipt, and candidate directory are fsynced before that namespace operation, and the affected
parent directories are fsynced afterward. A post-namespace fsync failure triggers the exact inverse
rename/swap and a second fsync of both parents. If that rollback is durably proven, the command returns
ordinary failure with the previous namespace restored and normal cleanup is safe. If rollback namespace
or rollback fsync cannot be proven, the verifier returns dedicated status 2
(`MODEL_LOCK_RECOVERY_REQUIRED`); `package.sh` preserves the complete transaction and package lock,
prints candidate/destination recovery paths, and deliberately blocks another package run until manual
reconciliation. Unexpected exceptions inside the final publisher are also mapped to recovery status 2.
The package driver enters preservation mode before launching that publisher and clears it only for
success or the verifier's explicitly safe status 1; signal exits and every other unclassified status
therefore retain the displaced release, transaction, and lock instead of deleting recovery evidence.

TDD receipts are **60/60** self-contained verifier tests and **16/16** transaction tests with explicit
fake Apple tools in copied fixture repositories. They cover toolchain/identity/policy drift,
duplicate JSON, extra/missing/unsafe/symlink/size/hash archive entries, authority and receipt drift,
authority/AAR mutation during every seal phase, ABA generation swaps, partial receipt writes, public
rejection of a text AAR, compiler/package failure, root and parent symlink write attempts,
fstat/fsync/close errors, an end-to-end replacement/swap failure, first-publication and replacement
post-fsync rollback, rollback namespace/fsync failure, proven restoration of the old release, and
retention of both recovery material and package lock when restoration cannot be proven. End-to-end
fixtures also inject rollback-fsync failure, an unexpected post-swap Python exception, and SIGTERM,
proving that abnormal publisher exits retain the displaced release and lock. Shell syntax,
ShellCheck, JSON parsing, and `git diff --check` pass under the pinned system interpreter. An independent
adversarial review found and drove closure of the initial root-symlink write, non-atomic two-file publish,
receipt TOCTOU, PATH-tool substitution, unchecked archive-size, seal ABA, unsafe final-path cleanup, and
post-publication fsync cleanup issues.

The seal threat boundary assumes host/account integrity. Its random mode-0700 snapshot prevents other
users and ordinary concurrent live-input drift; it is not intended to contain a malicious process
that already has arbitrary code execution as the same macOS UID. The final adversarial review found
no blocker under the normal local-release threat model and records same-UID compromise as conditional
future hardening rather than a claim this tool can provide privilege isolation.

The real local source was then packaged four times without editing the external dirty SAM worktree.
All full chains passed; the latest exercised the private seal snapshot, atomic no-clobber receipt
install, frozen-authority order, trusted system tools, pre-publication fsync, and replacement of an
existing release with the real atomic swap. The final AAR is 1,096,258,817 bytes at SHA-256
`910ba2f3c1578810d0202de782412ac8f52e5f3f13529f70acd7747a7f29d7db`; its 722-byte receipt passes a
fresh standalone list/extract verification and binds Manifest `8dd36bea…`, lock `20970430…`, and
license `b3a5a0e2…`. Apple Archive filesystem metadata makes the AAR hash generation-specific, which is
why the receipt records the exact published instance rather than claiming reproducible AAR container
bytes.

At this receipt, local package/receipt enforcement was closed but delivery was not. `upload.sh` still
used the obsolete top-level `build/assetpack/sam3d-body-pose.aar`; the immediately following slice
closes that local upload gate. Developer bundling and runtime model loading, NOTICE/app-resource
wiring, a replacement App Store Connect pack version, and real-device TestFlight download/load
remained open. No credential was read and no upload or other external mutation was performed in the
package/receipt slice.


## The SAM upload entry point is receipt-first and explicit (2026-08-11)

`tools/assetpack/upload.sh` no longer accepts the obsolete top-level AAR or treats invocation as
implicit permission to contact App Store Connect. Its default behavior and explicit `--verify-only`
mode run `/usr/bin/python3 -I tools/assetpack/verify_model_lock.py receipt` over the canonical atomic
pair in `build/assetpack/release/`. These modes do not expand or validate `ASC_API_KEY_ID`,
`ASC_API_ISSUER`, or the `$HOME` key path; they never invoke `altool` or make a network request. The
verifier's local Apple Archive list/extract operations remain part of the complete receipt gate.

Only a literal `--upload` authorizes both `altool --upload-asset-pack` and the subsequent
`--list-asset-pack-versions`. After validation, that mode requires an exact ten-character uppercase
API key ID, an issuer UUID, and a non-symlink regular `.p8` at the standard App Store Connect key
location. The numeric target is fixed to BioMotion's Apple ID `6761994383`; ambient app-target
overrides are not accepted. If upload succeeds and the list operation fails, the process returns
nonzero but the remote mutation may already exist, so a retry requires checking App Store Connect
state first.

The initial receipt-first implementation still had an artifact-generation gap: it validated the
published pathname and later let `altool` reopen that pathname, so a normal concurrent package
directory swap could change the uploaded bytes. Upload mode now copies both canonical files with
symlinks preserved into one random mode-0700 private directory, sets both snapshot files to mode
0600, and rejects non-regular source or snapshot leaves. The full receipt/archive gate runs on this
snapshot, and the upload receives that exact same snapshot AAR path. The snapshot remains present
through upload and version listing and is removed through a prefix-checked EXIT cleanup. Replacing
the public release after verification cannot change the bytes consumed by `altool`; a replacement
between the two source copies produces a mismatched AAR/receipt generation and fails the verifier.

The entry point pins `/bin/bash`, resets PATH to `/usr/bin:/bin:/usr/sbin:/sbin`, invokes the verifier
with isolated Python mode (`/usr/bin/python3 -I`), and fixes `/usr/bin/xcrun`. It removes ambient
`DEVELOPER_DIR`, `TOOLCHAINS`, and `SDKROOT` before either local archive verification or `altool`
selection. Custom candidates are deliberately narrow: `--aar` and `--receipt` must appear together,
must retain the exact canonical filenames, and must resolve to the same physical directory.
Positional paths, partial pairs, duplicate/conflicting modes, unknown arguments, and a combined help
request fail before verification or external action; parameter errors return status 64.

The strict TDD suite passes **11/11 hermetic causal groups covering 27 scenarios**. It uses only
copied fixture repositories, an exact fake verifier, and a fixture-only mechanical replacement of
the otherwise fixed `/usr/bin/xcrun`; no test can reach real `altool`. The tests prove exact receipt
argv, default and explicit read-only behavior, zero `altool` calls on verification/credential/key
failure, credential creation only after verification, upload-then-list ordering, propagation of both
external failures, canonical custom-pair enforcement, source and key symlink rejection, safe snapshot
cleanup on every tested exit, and fixed trusted tools. An adversarial source-generation test hashes
the private AAR, replaces the public source after verifier success, and requires the fake upload to
receive the same path and digest recorded by the verifier. A malicious `PYTHONPATH/sitecustomize` and
poisoned Xcode-selection variables are also injected and proven unable to replace the gate or tool
selection. Shell syntax, ShellCheck, Python byte-compilation, executable modes, and tracked/untracked
whitespace checks pass. Independent review reports blocker 0, high 0, and medium 0 under the stated
host/account-integrity model.

The real published 1,096,258,817-byte AAR and its 722-byte receipt pass the new default gate. A real
upload-mode local preflight also created and fully verified the private snapshot, then stopped on an
intentionally empty `ASC_API_KEY_ID`, cleaned the snapshot, and never entered `altool`. No real API
key path was inspected, no real `xcrun altool` command ran, and no App Store Connect request, listing,
upload, or other external mutation occurred. Historical hosted version 1 therefore remains obsolete;
developer bundling, runtime model verification/loading, NOTICE/resource wiring, an explicitly
authorized replacement upload, and real-device TestFlight download/load remain open.

This snapshot is a transaction-stability boundary, not a same-UID sandbox. A malicious process with
arbitrary code execution as the same macOS user can still race paths or the credential store and is
already inside the trusted account boundary. A hard kill can also leave a mode-0700 temporary
snapshot for manual removal from the system temporary directory. Those are conditional hardening
items, not claims of privilege isolation.


## The SAM runtime and developer bundle accept only verified precompiled models (2026-08-11)

The shipping runtime previously advertised a precompiled asset pack while still
accepting raw `.mlpackage` candidates from both the app bundle and Background
Assets. Those branches called `MLModel.compileModel(at:)`, copied the result into
Application Support, and keyed that cache with source size and modification
time. The local Simulator helper reinforced the split contract by accepting a
raw package and relying on Xcode to compile it. The causal runtime probe found
all of those active paths, and the first developer-bundle receipt test failed
because the helper rejected a valid canonical AAR/receipt pair while looking for
a `.mlpackage`.

`AssetPackModelStore` now has exactly two model candidates: a bundled
`SAM3DBodyPose.mlmodelc`, then the same compiled directory in the managed asset
pack. It no longer imports Core ML's compiler API, names or probes a raw package,
owns an Application Support compile cache, or reads model modification time.
The missing-pack path is unchanged: it starts/joins the Background Assets
download and throws immediately rather than waiting on the transfer.

`tools/assetpack/dev_bundle_model.sh on` now defaults to the canonical atomic
pair under `build/assetpack/release/`. It rejects symlink/FIFO/special-file
inputs, freezes the AAR and receipt through no-follow descriptors into one
mode-0700 build-local transaction, and runs the isolated receipt verifier and
extraction only against that frozen pair. The extracted compiled tree must
contain only ordinary directories/files and a regular `coremldata.bin`; a raw
package is never accepted or compiled. Publication fsyncs the candidate tree,
uses one Darwin exclusive rename for the first install or `RENAME_SWAP` for a
replacement, and synchronizes both parents changed by the cross-directory
namespace operation. A classified post-publication failure swaps/renames the
old state back and synchronizes both parents; if rollback cannot be proven, the
helper returns status 2 and preserves the private transaction with both recovery
paths.
Preservation is armed before the publisher starts, so a signal or unclassified
exit cannot make the shell cleanup erase a displaced model. Only private status
10 proves a pre-namespace failure or completed rollback and is translated to
public status 1 after disarming preservation; a raw unclassified status 1 keeps
both recovery paths. They remain reported with that nonzero status.
`off` removes only the fixed non-symlink `build/DevBundledModel` destination.

The developer transaction suite passes **29/29** dynamic scenarios. Its copied
fixtures mechanically inject failures without adding a production test
override. Coverage includes a release-directory swap between copying the AAR
and receipt, verifier-time live replacement after the snapshot, first and
replacement namespace failures, first/replacement post-namespace sync failures
and identity-inspection failures that execute real rollback, rollback-swap
failure and post-swap SIGTERM with preserved recovery material, bad
receipts/artifacts, a post-swap unclassified raw status 1, raw packages,
symlink/FIFO/special-file input and extracted-tree entries, build/destination
symlinks, exact `off` deletion, adjacent sentinel preservation, and a second
idempotent disablement. Dynamic markers prove the intended verifier, namespace,
rollback, signal/status, and refusal branches were actually reached.

The compiled-only runtime probe, `/bin/bash -n`, ShellCheck, `git diff --check`,
the existing **60/60** supply-chain tests, and the **16/16** package/receipt
suite pass. The real 1,096,258,817-byte AAR/receipt was also reverified and
extracted into a private transaction; the compiled probe was a regular file and
the extracted tree contained no symlink. That was a local read-only dry run: it
did not publish `DevBundledModel`, inspect credentials, invoke upload mode, or
contact App Store Connect. A fresh arm64 Simulator `build-for-testing` passed.

The remaining limits are explicit. Background Assets directory resolution and
the final `MLModel(contentsOf:)` load still need a TestFlight device. The local
install temporarily needs space for the roughly 1 GiB frozen AAR plus extracted
model. Existing Nimble/OSQP simulator archives are arm64-only, so a universal
x86_64 simulator link is not claimed. The mode-0700 transaction is a normal
local-account boundary, not a sandbox against malicious code already running as
the same UID. Removing the source-mtime cache also removes BioMotion's only File
Timestamp required-reason API use; the following privacy-manifest gate records
that resulting contract.


## Privacy manifest is source-evidenced and bundled exactly once (2026-08-11)

The draft privacy manifest previously declared File Timestamp reason `C617.1`
for a model-cache mtime read that no longer exists, and System Boot Time reason
`35F9.1` for `CACurrentMediaTime()`. Apple's Required Reason API list names
`systemUptime` and `mach_absolute_time()` for System Boot Time; it does not name
`CACurrentMediaTime()` or `clock_gettime()`. Declaring those stale/speculative
categories would therefore make the manifest less accurate, not safer.

`PrivacyInfo.xcprivacy` now sets only `NSPrivacyTracking = false`. It omits
tracking domains, collected-data types, and required-reason API types because
those reviewed inventories are empty. That omission is required rather than
cosmetic: Apple TN3181 identifies an empty `NSPrivacyAccessedAPITypes` array as
invalid and instructs developers to remove the key. A source gate requires the
actual Apple-listed System Boot Time, File Timestamp, disk-space, UserDefaults,
and active-keyboard inventories to remain empty. The patterns cover every API
spelling in Apple's current five categories, including `getattrlistat`, all file
date keys, and all four disk-capacity keys, plus the NSUserDefaults,
CFPreferences, and AppStorage language-layer aliases. The gate separately pins
all 13 reviewed `CACurrentMediaTime()` calls by file and requires
`clock_gettime()` to remain absent, so elapsed-clock changes cannot bypass
review or be mistaken for a required-reason category. Network, analytics,
advertising, and tracking SDK tokens remain prohibited by the same source gate.

XcodeGen assigns the manifest explicitly to the app resource phase and excludes
it from broad source inference. The generated project must contain exactly one
file reference and one build-file membership in BioMotion's own Resources
phase—not the extension or test target. The built-app gate additionally
requires one byte-identical root manifest, recursively scans every embedded
Mach-O for undeclared required-reason APIs, rejects unreviewed dynamic
dependencies, and verifies the complete code signature. The main TestFlight
flow runs this gate on the archive before export. The archive Privacy Report and
current Apple documentation remain human-reviewed release artifacts; the
static gate does not claim to replace either one.

The source/shape/target suite passes **38/38** cases. An arm64 Simulator
Debug build also passed after project regeneration. Its app root contains one
byte-identical manifest, the app and extension signatures verify, and all six
Mach-O images—the two thin launch executables, two exact Debug dylibs, and two
preview dylibs—pass the symbol/string and dependency audit. Separate copied-app
negatives added an unreferenced dylib carrying `_getattrlistat`, one carrying
`volumeTotalCapacityKey`, and one with an unknown `@rpath`; recursive discovery
rejected each for the intended reason before the deliberately invalidated
signature. An earlier `mach_absolute_time()` negative also exposed and fixed an
`nm -u` format assumption: on this host undefined symbols have no leading
whitespace. Existing Swift concurrency/deprecation and Nimble/Eigen
documentation warnings remain, but the build has no new compile or link error.


## App resources and release provenance are now an exact allowlist (2026-08-11)

The generated app had inherited files from the broad `BioMotion/` source root: source-only
README/lock/license material could enter the product while the repository's root `NOTICE` was
missing. Conversely, the first product probe required a few known files but did not reject an
extra OSIM, a renamed compiled model, arbitrary large data, or payload hidden in an asset catalog.
It also accepted a bare Debug/Simulator `.app` as if it were release evidence.

`project.yml` now excludes the whole source resource directory from broad inference and adds an
explicit shipping allowlist: the privacy manifest, exactly two reviewed OSIM files, root `NOTICE`,
and `BioMotion/Resources/THIRD-PARTY-NOTICES.txt`. The test target separately owns only its Fixtures
folder and the two model files. The consolidated legal resource carries the complete selected
binary-distribution terms and attributions for BioMotion, Nimble/DART, ODE, OSQP/QDLDL/AMD, Eigen,
tinyxml2, OpenSim, and Rajagopal. `EIGEN_MPL2_ONLY=1` is now part of the app and test consumer
contract. This closes notice delivery; it does **not** grant commercial rights to FullBody's 42
MoBL-ARMS-derived muscles, so the owner/legal blocker below remains unchanged.

`tools/release/resource_boundary.py` is the shared source/project/product inspector. It resolves
actual PBX group ancestry, pins every model/legal/asset source identity, and requires exact
target-scoped Resources phases. Its built-product modes are deliberately distinct:

- `--simulator-smoke` checks an exact Simulator app and extension inventory;
- `--tests-bundle-smoke` checks the exact model and seven-fixture `.xctest` inventory;
- `--release-archive` accepts only the signed arm64 iPhoneOS `.xcarchive`, validates its archive
  receipt/team/path/platform, exact app and extension contents, compiled icon catalog, size budgets,
  model/legal bytes, Mach-O architecture, and strict deep signature;
- `--release-ipa` safely extracts the locally re-signed IPA, repeats the distribution/privacy
  checks, and binds it back to the archive while allowing only profiles, CodeResources, and
  normalized Mach-O signatures to change.

The optional 1.3 GiB developer model is now legal only for a Debug iOS Simulator build. The app
target runs `tools/release/reject_dev_model.sh` as its first and last phases: non-Simulator or
non-Debug builds reject the source path (including a dangling symlink), and the final phase scans
the copied product so a model toggled during compilation cannot cross the release boundary.

The first adversarial rerun found two provenance holes before commit. `PLATFORM_NAME` could be
overridden to say Simulator while Xcode retained the iphoneos SDK, and the original archive fixture
was merely ad-hoc signed with self-reported Team/profile text. The guard now cross-checks
`EFFECTIVE_PLATFORM_NAME`, `SDK_NAME`, canonical `SDKROOT`, and—after resources—the product
Info.plist plus its executable's Mach-O platform receipt. The product inspector checks every
accepted Mach-O's real `LC_BUILD_VERSION`; Release further binds an Apple Distribution certificate,
actual signing Team, exact signed entitlements, App Store provisioning profile, profile-authorized
leaf certificate, bundle id, architecture, and archive receipt. A macOS image renamed as iOS and
the former ad-hoc archive are both causal negatives.

A second release-engineering review found three later-stage gaps. The documented build-number bump
occurred after XcodeGen, the export plist lived only under ignored `build/`, and Xcode could re-sign
and directly upload an IPA after only the pre-export archive had been checked. The order is now
`project.yml` bump, XcodeGen, source gate, archive. The exact manual TestFlight export plist is
tracked with `destination=export` and the generic `Apple Distribution` selector. A fail-fast
`tools/release/testflight_release.sh` validates source/archive/privacy, exports locally, validates
the final IPA and its SHA-256, and remains network-free by default; only `--validate` or `--upload`
can contact App Store Connect, and upload uses the same private byte-pinned snapshot it validates.

The same review invalidated an apparent provisioning-profile proof: `security cms -D` decodes CMS
but accepted an equal-length mutation of the signed XML. Profiles now pass OpenSSL content-signature
verification and an offline signer-chain verification to the pinned classic Apple Root CA, followed
by exact Apple provisioning-profile signer subject/issuer, CA/key-usage, and private-OID checks.
Both current named App Store profiles pass that path; the mutated parseable CMS negative fails at
the content signature. The embedded profile and actual leaf distribution certificate must each
retain at least 30 days of validity. Because this project does not enable Keychain Sharing, any signed
`keychain-access-groups` entitlement is now unreviewed and rejected instead of receiving wildcard
subset treatment.

The strengthened resource suite passes **40/40** source, PBX, arm64 Simulator-app, test-bundle,
archive-provenance, stale-version, export-option, and unsafe-IPA cases. The IPA negatives include
a raw ZIP name whose NUL suffix Python would otherwise truncate, an 8 MiB DEFLATE stream whose
local and central metadata falsely claim one byte, and an app executable with its execute bits
removed. The gate independently checks raw central/local headers, actual inflate EOF/CRC/size and
data descriptors, rejects semantic ASi Unix metadata and ZIP comments, then binds Unix execute
permissions back to the archive. The developer-model guard
suite passes **16/16**, the CMS mutation suite **2/2**, and the fail-fast TestFlight wrapper suite
**9/9**. A fresh arm64 Debug Simulator `build-for-testing` succeeded and its actual `.xctest` passed
the exact bundle smoke gate. A separate clean app-only product—so no embedded test plugin could be
mistaken for shipping content—passed both the exact Simulator app gate and the recursive privacy
gate after an ad-hoc local re-seal. A separate clean arm64 Release Simulator build also succeeded,
ran both developer-model guards, and passed the same resource and privacy gates; the manual
iphoneos signing settings do not poison the Simulator configuration. The checked-in
source/generated-project probe passes. The Release configuration records the two manual App Store
profile names instead of relying on external memory. A controlled real Xcode negative temporarily
enabled a symlinked `build/DevBundledModel`; the Release Simulator build failed in the first guard
before the app target compiled, and the exact symlink was removed. A real signed archive attempt
compiled to the signing phase but remains blocked on the interactive macOS private-key access
prompt; therefore neither a successful signed archive nor its exported IPA is yet claimed.
Simulator evidence cannot close that device/archive/signing gate.

These pathname checks assume a quiescent artifact and a trusted macOS account. They reject ordinary
drift, symlinks, and invalid signatures, but they are not a sandbox against malicious code already
running as the same UID and racing inspection or mutating the archive after the gate returns. Run
the integrated gate immediately before export/upload; arbitrary same-account code execution means
the release host is compromised.


## XML conversion no longer depends on a machine-specific Boost install (2026-08-11)

`dart/utils/XmlHelpers.cpp` previously pulled header-only Boost string and lexical-conversion code
into the iOS archive. The Xcode app and test targets consequently named the absolute local path
`/opt/homebrew/Cellar/boost/1.90.0_1/include`, so a clean machine needed the same Homebrew formula
revision even though no Boost library was linked.

The behavior was characterized before changing the implementation. The old archive passed exact
formatting and parsing cases for bool, signed/unsigned integers, char, float/double round trips,
NaN/Infinity, hexadecimal floats, overflow/underflow/subnormals, vectors, transforms, and tinyxml2
value/attribute wrappers. Every scalar failure is required to derive from `std::bad_cast`; successful
conversions preserve caller `errno`, while the range failures leave `ERANGE` on the reviewed Apple
libc++ targets. The surprising unsigned `"-1"` wrap and untrimmed boolean behavior remain pinned.

Nested commit `78b292e19af13ad77501c9b22f49c1fa06146501`
(`refactor(utils): remove Boost from XML helpers`) replaces Boost with classic-locale standard
streams and is published on
[`shengyang998/nimblephysics:biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05).
`git ls-remote` resolved the branch to that exact SHA. The exported
`nimble-patches/xml-helpers-no-boost.patch` is byte-identical to the commit diff, reverse-checks at
the branch head, is 492 lines, and has SHA-256
`86bf0987efa9961a06679115e926408fc451a5ca4ee165fee29c5b808ec58aa9`.

Both arm64 simulator and device archives rebuilt. The refactor gate:

- fresh-compiled the source for both SDKs with `-Wall -Wextra -Werror` and found no Boost symbol;
- required exactly one `XmlHelpers.cpp.o` in each rebuilt archive and rejected `boost::` in those
  exact members;
- ordinary-linked both archives with dead stripping and no force-load;
- ran under a hostile global comma/grouping locale to `XML_HELPERS_CLASSIC_LOCALE_PASS`; and
- reran the independent characterization to `XML_HELPERS_CHARACTERIZATION_PASS`,
  `XML_HELPERS_ARCHIVE_PROBE_PASS`, and `XML_HELPERS_NO_BOOST_ARCHIVES_PASS`.

`project.yml` and the generated Xcode project no longer carry the absolute Boost include path.
Whole-app no-signing builds passed for the dedicated arm64 simulator and generic arm64 device
destinations. This closes the XML helper dependency and its locale ambiguity. This isolated commit
did not itself make the older iOS CMake/source-manifest port reproducible; the later migration
section closes that separate boundary.


## OpenSim geometry now fails closed on the no-Assimp iOS build (2026-08-11)

The iOS archive cannot load Assimp meshes, but both `OpenSimParser::parseOsim` overloads still
defaulted `ignoreGeometry` to false. The URI path could touch the retriever and collapse an
exception into a null skeleton; the document path inspected XML first; and the older MeshShape
stub could make a requested geometry load look successful while returning no mesh shapes.

Nested commit `3829f1682b772cee7812e59767dae1cb067f3541`
(`fix(ios): reject unavailable OpenSim geometry`) puts the same exact `std::runtime_error` at the
start of both overloads, before retriever creation, URI I/O, or XML inspection. It is published on
[`shengyang998/nimblephysics:biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05),
and `git ls-remote` resolved the branch to that SHA. BioMotion and the existing parser tests now
pass `ignoreGeometry=true` explicitly. Desktop behavior is unchanged.

The tracked `nimble-patches/ios-opensim-geometry-boundary.patch` is byte-identical to the nested
commit diff, reverse-checks at the branch head, is 76 lines, and has SHA-256
`38b76a5d4a7464ca494bd0574c8d97dd89bd53e4cc7d8da44b8a77f1a91ebf7e`.
Both simulator and device archives rebuilt. A normal dead-stripped link extracted
`OpenSimParser.cpp.o` for each SDK with no unresolved DART symbols; the simulator executable called
both overloads and reached `OPENSIM_GEOMETRY_IOS_BOUNDARY_PASS` and
`OPENSIM_GEOMETRY_IOS_ARCHIVES_PASS`, with both instrumented retrievers still at zero accesses.
An unsigned arm64 simulator `build-for-testing` compiled and linked the whole app and all six new
XCTest methods. The test-gate harness passed **49/49**, and the reviewed fast count is now **532**.

The standard post-change XCTest receipt is deliberately not claimed. Xcode 26.4's local
testmanager stopped launching even a one-method `MotionRecorder` smoke test across reset/fresh
simulators, while the standalone iOS executable still ran normally. Missing xcresult evidence is a
gate failure, not a pass. The focused six methods and then `tools/run_tests.sh all` remain required
on a fresh runner before release.


## Unavailable OpenSim conversion utilities are no longer advertised on iOS (2026-08-11)

The iOS archive deliberately omits MarkerFitter and the SDF/MJCF exporters, but
`OpenSimParser.hpp` still declared `translateOsimMarkers`, `convertOsimToSDF`,
and `convertOsimToMJCF`. Before this fix, simulator and device consumers for
each API compiled successfully, produced the intended undefined reference, and
then failed an ordinary link because neither archive defined it: six exact
`CAUSAL_RED` receipts.

Nested commit `24712fc826374c887ffb6eceac48a30f8cb6f2b8`
(`fix(ios): hide unavailable OpenSim utilities`) pairs those three declarations,
implementations, and their MarkerFitter/SDF/MJCF includes behind
`DART_IOS_BUILD`. It also removes `GUIRecording.hpp`; the only apparent use was
inside a block comment. Desktop keeps all three utilities. The commit is
published on
[`shengyang998/nimblephysics:biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05),
and `git ls-remote` resolved the branch to the same full SHA.

The supported surface is independently pinned rather than inferred from the
three removals. Both `parseOsim` overloads, `loadTRC`, `loadMot`, `loadGRF`, and
`loadMotAtLowestMarkerRMSERotation` remain public and defined. The final probe:

- negative-compiles all three unavailable names for simulator and device with
  exact `no member named` diagnostics;
- uses a comment- and literal-aware active-view parser for iOS and desktop,
  fails closed on unknown `DART_IOS_BUILD` expressions or malformed/unbalanced
  conditional stacks, and mutation-tests declarations hidden in block comments;
- takes exact function pointers to all six supported signatures and requires
  their exact counts in both consumer objects, both archives, both dead-strip
  link maps, and both linked executables; and
- executes both `parseOsim` geometry-refusal overloads on the simulator before
  emitting `OPENSIM_UTILS_SOURCE_CONTRACT_SELF_TEST_PASS`,
  `OPENSIM_UTILS_IOS_BOUNDARY_PASS`, and
  `OPENSIM_UTILS_IOS_ARCHIVES_PASS`.

Both arm64 archives rebuilt after the production change, and a fresh unsigned
arm64 simulator `build-for-testing` succeeded. A true desktop binary build is
not claimed: this maintained checkout currently carries an iOS-only generated
config and lacks the desktop Asio dependency. Desktop preservation is covered
by the strict source contract until the reproducible CMake/dependency slice
restores a clean desktop configuration. Final independent review reported
zero blocker, high, medium, or low issue.

The tracked `nimble-patches/ios-opensim-utilities-boundary.patch` is
byte-identical to the nested commit diff, applies to the pinned `c405b05`
baseline, reverse-checks at the public branch head, is 100 lines, and has
SHA-256
`68c34b8e38752b64e4a7a6ed8ade15b3bffee6ff143e61304786911c1f42570a`.
This closes an API/header-to-archive boundary; it does not add iOS marker
translation or SDF/MJCF export.


## Mesh-backed anthropometric scoring now fails closed on iOS (2026-08-11)

The iOS archive omits the mesh/GUI stack needed by Nimble's anthropometric
measurement code, but `Anthropometrics.hpp` still included those dependencies
and advertised the complete desktop surface. `IKErrorReport` also accepted a
non-null anthropometric prior. The causal probe found 31 failures: all ten
unsupported methods compiled for an iOS consumer, both archives retained the
mesh-backed non-debug symbols, and a runtime prior could return a plausible
score instead of refusing the unavailable capability.

Nested commit `b7068024286a72f66b7d9c841527656d7631b192`
(`fix(ios): reject unavailable anthropometric scoring`) is published on
[`shengyang998/nimblephysics:biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05),
and `git ls-remote` resolved the branch to that exact SHA. Its boundary is:

- iOS hides GUI debug output, mesh marker resolution/measurement, PDF/log-PDF
  scoring, and all four body/group gradient methods. Their declarations and
  definitions are paired behind `DART_IOS_BUILD`.
- Construction, metric-file loading, metric insertion/names, distribution
  set/get, conditioning, and metric-pose application remain public and linked.
- `IKErrorReport` keeps its prior-free path. A non-null anthropometric prior
  throws the exact descriptive `std::runtime_error` before any skeleton read or
  mutation; desktop still calls `getLogPDF`.
- The public headers no longer expose `LilypadSolver`, `MeshShape`, or
  `GUIWebsocketServer` transitively on iOS.

Both arm64 archives rebuilt successfully. The final probe then strictly
compiled the current `Anthropometrics.cpp` and `IKErrorReport.cpp` for simulator
and device, checked those fresh objects for ten absent and eight retained
symbols, and linked the fresh objects ahead of each archive. Link maps require
the current objects and reject extraction of the archive's same-name members,
so an old static library cannot make the test falsely green. The simulator ran
those current-source objects, preserved seeded non-empty positions, body
scales, and group scales across both the prior-free and rejected-prior paths,
and emitted `ANTHROPOMETRICS_IOS_BOUNDARY_PASS` plus
`ANTHROPOMETRICS_IOS_ARCHIVES_PASS`. Header include tracing, all ten exact
negative consumers, `bash -n`, ShellCheck, dual-SDK strict compilation, and a
strict macOS SDK source compile also passed. A complete desktop binary build is
not claimed while the checkout carries an iOS-only generated config.

The tracked `nimble-patches/ios-anthropometrics-boundary.patch` is 204 lines,
has SHA-256
`89c8292a8fa6b86b89b57d4bf98e17bef7b063b67d6970ff7835b93a5796ffdd`,
is byte-identical to the nested commit diff and the cumulative four-file diff
from the pinned `c405b05` baseline, and reverse-checks at the public branch
head. This closes a false capability boundary; it does not add mesh-backed
anthropometric scoring to iOS.


## Mesh shapes now fail closed on the no-Assimp iOS build (2026-08-11)

The iOS source manifest replaces desktop `MeshShape.cpp` and omits
`SoftMeshShape.cpp`, but the public headers still included Assimp and did not
have one complete reviewed definition for their current surface. That was not
usable mesh support: clients could fail at include/link time, and the normal
`SoftBodyNode` constructor still tried to create `SoftMeshShape` after attaching
itself to a parent BodyNode and Jacobian tree.

Nested commit `ffad0db626ba90ad9d7f73e813202c0a7a176381`
(`fix(ios): reject unavailable mesh shapes`) is published on
[`shengyang998/nimblephysics:biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05),
and `git ls-remote` resolved the branch to that exact SHA. It contains exactly
four files: `MeshShape.hpp`, new `MeshShape_ios.cpp`, `SoftMeshShape.hpp`, and
`SoftBodyNode.cpp`.

The boundary has one explicit policy:

- iOS keeps Assimp types incomplete. It does not invent layout-compatible fake
  `aiScene`/`aiNode`/`aiMesh` structs. Desktop retains the real Assimp include
  and `SoftMeshShape`'s `unique_ptr<aiMesh>` ownership.
- `getStaticType()`, `getType()`, and destructors are the only safe metadata
  whitelist. Both constructors and every backend-dependent Mesh/SoftMesh API
  unconditionally throw one of two exact `std::runtime_error` messages.
- `SoftBodyNode` preserves the upstream notifier-before-mesh successful order.
  On an exception it now deletes any derived point masses and notifier, then
  detaches while the dynamic type is intact so both the parent's public child
  list and protected Jacobian-child set are cleaned before base unwinding. This
  is a general constructor exception-safety improvement used to make the iOS
  refusal transactional; desktop exception behavior is improved rather than
  claimed unchanged.

The verification closes the earlier false-green gaps:

- The source contract requires each unavailable method body to be only its
  shared throw helper, full-matches the safe metadata bodies/destructors, and
  parses both headers to reject any unclassified current or future method.
- Simulator and device archives were rebuilt after the final source bytes.
  Each contains exactly one `MeshShape_ios.cpp.o`, defines the complete surface,
  carries both pinned messages, and has no unresolved Assimp dependency.
- Ordinary dead-stripped fresh-object and archive-object links passed for both
  SDKs without force-loading. All four simulator executions reached the two
  exact fail-closed sentinels, followed by
  `MESH_SHAPE_IOS_BOUNDARY_PROBE_PASS`.
- The host fault-injection transaction removes the production mesh object and
  injects the exact SoftMesh rejection. Root and child construction each
  rejected **32/32** times; Skeleton counts, public child membership, and the
  directly inspected hidden Jacobian-child set returned to zero every time.
  A pointer-level global allocation tracker proved every notifier-sized
  allocation was released, while its positive control reported
  `allocated=1 freed=0 live=1`. The normal path was AddressSanitizer-clean and
  ended in `SOFT_BODY_MESH_REJECTION_TRANSACTION_PASS`.
- A fresh unsigned arm64 simulator `build-for-testing` compiled and linked the
  whole app and test target. The final independent review reported no blocker,
  high, or medium issue.

The exported `nimble-patches/ios-mesh-shape-boundary.patch` is byte-identical
to the nested commit diff, is 435 lines at SHA-256
`da9f70d1b908f0936a1f0323fb6fa805f10de2890ee8c749f8d244759929d160`,
applies to the pinned `c405b05` baseline in a temporary clean worktree, and
reverse-checks at the branch head.

This closes a refusal and constructor-transaction boundary. It does not add
Assimp, mesh rendering/loading, collision/contact, or soft-mesh simulation. At
this slice's receipt the CMake/source manifest still needed a separate export
and consistent `DART_IOS_BUILD` publication; the later standalone-CMake section
closes that build boundary. This slice adds no XCTest methods or count change,
and no post-change XCTest execution is claimed while the local Xcode 26.4
testmanager launch channel remains broken.


## The iOS C3D header surface now matches the absent ezc3d backend (2026-08-11)

The iOS source manifest has never compiled `C3DLoader.cpp` or `C3DForcePlatforms.cpp`, but the public
headers still exposed their full API. `C3DLoader` and
`C3D::getWeightedDistFromCoPToNearestMarker()` could therefore compile and fail only at link time;
including `C3DForcePlatforms.hpp` instead failed immediately because ezc3d is not present. That was
a split contract rather than usable C3D support.

The reviewed boundary keeps exactly the portion that the app and archive can support:

- The pure `C3D` value struct retains all fields, including its `std::vector<ForcePlate>`; its layout
  is unchanged. `ForcePlate.cpp.o` remains in both archives.
- `OpenSimParser::loadMotAtLowestMarkerRMSERotation(..., C3D&, ...)` remains public and defined. It
  consumes only `dataRotation` and `markerTimesteps`, not ezc3d or `C3DLoader`.
- iOS hides the loader-only weighted convention method, the whole `C3DLoader` class, and
  `FORCE_PLATFORM_NUM_CONVENTIONS`, `ForcePlatform`, and `ForcePlatforms`.
- Both production `.cpp` files include their own header before the platform guard. An accidental
  future iOS source-manifest entry therefore produces a valid empty translation unit rather than
  reintroducing ezc3d through a transitive include.
- Non-iOS declarations and implementation bodies are unchanged. `C3DLoader.hpp` now explicitly
  includes `<string>` and no longer depends on the unused `Skeleton.hpp` include.

The causal RED reported **8 contract failures**: the adapter header and both production sources
leaked missing ezc3d includes, two unavailable loader APIs still compiled, and three adapter
negative cases failed for that missing include instead of because the names were absent. After the
four-file fix:

- simulator and device `nimble_ios` archives rebuilt successfully;
- the two retained-contract XCTest methods passed separate **1/1** receipts in 12 s and 7 s, with
  zero failures, skips, expected failures, or restarts;
- `c3d_ios_boundary_probe.sh` passed all supported and forbidden compile surfaces, verified both
  archives member by member as simulator platform 7 and device platform 2, required the retained
  OpenSimParser/ForcePlate members and symbols exactly once in each, and rejected the complete
  forbidden surface from defined and undefined symbols. Ordinary dead-stripped links for both
  platforms extracted `OpenSimParser.cpp.o` and `ForcePlate.cpp.o` without force-loading or
  unresolved ezc3d/DART symbols; the simulator binary ran on `BioMotion-CI` to
  `C3D_IOS_ARCHIVE_PROBE_PASS` and `C3D_IOS_BOUNDARY_PROBE_PASS`;
- an independent nested review reported no blocker, high, medium, or low issue; and
- the enclosing `tools/run_tests.sh all` gate passed. The fast lane completed **526/526** tests in
  1696 s and the slow lane completed **1/1** in 6274 s. Both `xcodebuild` and `xcresulttool`
  returned 0 for both lanes; both xcresults were `Passed`, with zero failures, skips, expected
  failures, or test-host restarts. The runner emitted `ALL GATE PASS`.

The nested source change is isolated in
`03fa30ca524376747f7e0e884c8c8c14c4d5526f` (`fix(ios): hide unavailable C3D loading APIs`),
contains exactly four C3D files, and is published at
[`shengyang998/nimblephysics:biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05).
`git ls-remote` resolved the fork branch to the same SHA. The tracked
`nimble-patches/ios-c3d-boundary.patch` is byte-identical to that commit diff, reverse-checks against
the branch head, is 116 lines, and has SHA-256
`63b5bc8ad9206738eedabf89100a1fee84ce856f3cba6895dc63bb5fc50ea6a7`.

Every iOS consumer target must define `DART_IOS_BUILD=1`; the current CMake target, app, tests, and
probe do. This closes a false public API. It does **not** add C3D file loading. This isolated slice
did not export the older CMake/source-manifest port; the following section records that separate
fresh-clone result.


## The Nimble iOS port is pinned and fresh-clone reproducible (2026-08-11)

The complete BioMotion port now lives on the maintained
[`shengyang998/nimblephysics:biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05)
branch. The final integration receipt is
`0ecf26a1557ee738146511cd81fbe99f2bc94d38`, and `git ls-remote` resolved the
remote branch to that same SHA. Fresh setup therefore clones this fork/branch
and detaches that exact commit; it no longer clones upstream and reconstructs
the port with hand edits or `git apply`. The outer `nimble-patches/` files remain
historical single-diff audit, pinned-baseline replay, and reverse-check records.
They are not a second source of the complete tree.

Three nested commits close the dependency/build/header chain:

- `f039af4c19ec12a3af3e6fa29ae926a7f0449981` pins the vendored Eigen and
  tinyxml2 sources and restores their license files. The iOS build has no
  Homebrew dependency include and no Boost dependency.
- `bf6519c91b74b387b2a999ce59afd35f26abd6ba` adds the standalone
  `ios/CMakeLists.txt`, `ios/config.hpp.in`, and build instructions. The reviewed
  workflow requires CMake 3.24+ and Ninja and configures cleanly with
  `cmake --fresh -S ios -B build_ios` or `build_sim`. Each build tree generates
  its own DART 6.9.0 `dart/config.hpp`; a source-tree `dart/config.hpp` is a hard
  error. The target accepts only arm64 iphoneos/iphonesimulator product builds,
  exports all four public definitions, and exposes macOS test compilation only
  through the explicit `NIMBLE_IOS_HOST_PROBE=ON` option.
- `0ecf26a1557ee738146511cd81fbe99f2bc94d38` removes unavailable GUI/asio and
  Ipopt transitive includes from the six supported biomechanics headers while
  preserving the desktop surface behind the platform guard.

Both archives were recreated from fresh caches. The device SHA-256 is
`77b483af8f555ca5d2a2ab6344d183b0945245a8fc6b62084d7e648d739105e3`; the
Simulator SHA-256 is
`483f77cff99518b7832290690c6086994c9c78e0c9196a39d549e51731348b34`.
Each archive has exactly **162** `ar` members: one symbol table and **161**
reviewed object members. The compile-command audit covered all 161 translation
units in each build and required the four target definitions plus the matching
generated-config directory ahead of the source root. The dual-SDK public-header
probe passed **12/12** positive compiles and **8/8** poisoned-dependency
negatives; it also proved the selected DART 6.9.0 config, vendored Eigen, and
vendored tinyxml2 came from the intended roots.

An independent clean clone from the published HTTPS branch then detached the
same exact receipt and rebuilt both SDK variants. It reproduced both archive
hashes and both 162-member inventories, ending in
`FRESH_REMOTE_NIMBLE_REPRODUCIBILITY_PASS`; the checkout remained clean.

Outer integration then passed all eight archive probes: anthropometrics, C3D,
collision static-link, mesh shape, OpenSim geometry, OpenSim utilities, XML
characterization, and XML refactor. The separate `SoftBodyNode` rejection
transaction completed its root/child rollback checks and AddressSanitizer-clean
normal path to `SOFT_BODY_MESH_REJECTION_TRANSACTION_PASS`.

The first fresh generic-Simulator product build correctly exposed one remaining
integration defect: Xcode selected x86_64 even though the reviewed Nimble and
OSQP Simulator archives are intentionally arm64-only. The project now declares
that arm64 boundary explicitly instead of failing later at link time. A fresh
generic Simulator build and a fresh unsigned Release generic-device build both
pass after the fix. The generated app-and-test target also passes
`build-for-testing` and emits `BIOMOTION_NIMBLE_TEST_TARGET_BUILD_PASS`.

No new `leaks` result is claimed for this migration. The current host has
`DevToolsSecurity -status` disabled, so non-interactive `leaks` attachment to
the target process is not authorized or reliable under the current host policy.
The hardened probe now fails immediately with exit **25** under that condition;
when permission is available it inspects a live child PID rather than a dead
process. That fail-fast policy and the older
historical leak receipt do not substitute for a new permitted run.

This closes build reproducibility and header/dependency integration only. It
does not add collision/contact, Assimp/mesh, ezc3d, anthropometric scoring, or
dynamics support. It also does not grant commercial rights to the
**42 MoBL-ARMS-derived upper-limb muscles** in `FullBody.osim`: commercial/App
Store release remains blocked until the owner obtains written commercial
permission or replaces/removes that material.


## IK convergence: the solver is now a fixed point (2026-08-07)

App-side only. `NimbleBridge.mm` no longer calls `Skeleton::fitMarkersToWorldPositions` /
`math::solveIK` / `math::refineIK`; it runs its own bound-projected Levenberg-Marquardt at the call
site. **At this historical 2026-08-07 receipt, the vendored nimble tree was byte-identical**
(`git status nimblephysics/ osqp/` = 0 lines); the maintained iOS fork described above was published
later.

Six distinct defects, each with its own mechanism:

| | Defect | Fix |
|---|---|---|
| 1 | `refineIK` stops on error-CHANGE, never on stationarity, then the next call resets `lr` to 1.0 and resumes | bound-projected-gradient test + step-norm test. The pose returned IS a stationary point, so the next call on the same markers passes its FIRST test having moved nothing — the fixed point is a property of the termination rule, not of a tolerance |
| 2 | `leastSquaresDamping` is a fixed 0.01 | trust-region λ adapted from the observed decrease, expressed as a multiple of `max(diag(JᵀJ))` so it is scale-free, plus a `1e-6·max(diag)` conditioning floor. `JᵀJ` is 169×169 with rank ≤ 60; below that floor, double-precision round-off in the gradient is amplified into ~1e-7 rad of null-space noise per step. At a 1e-9 floor the dancer burned all 120 iterations of BOTH phases and still reported converged=NO |
| 3 | `clampPositionsToLimits` inside the objective means a coordinate on its limit keeps generating steps the clamp undoes, so the unprojected gradient never reaches zero | active set — a coordinate at a bound whose gradient pushes it further out leaves both the step and the convergence test. This is what turned the dancer from converged=NO/240 iterations into converged=YES/177 |
| 4 | no null-space damping toward the seed | two phases. A: `‖W(f(q)−x*)‖²/2 + μ‖q−q_seed‖²/2`, μ = 1e-3 — chooses WHICH of the equally-good poses comes back. B: re-run with μ = 0 — drives `Jᵀr` to 0 so a fixed point exists at all. B's steps lie in the row space of `J`, so they cannot undo A where `J` has a zero column. Verified: all 72 coordinates in `FullBodyDOFFixture.structurallyUnreachableCoordinates` return at EXACTLY their seed value, across two poses |
| 5 | random restarts drew from process-global `std::rand()`, and `fitMarkersToWorldPositions` seeded from the skeleton's CURRENT positions (`Skeleton.cpp:8001`) — which the shared skeleton meant was whatever ID or `MomentArmComputer` last wrote | restarts removed; the cold seed is an explicit `neutralSeedPose` |
| 6 | nimble scaled the residual by the marker reliability weights but NOT the Jacobian (`Skeleton.cpp:7979-7986`), so its step minimised `‖J·d − W·r‖` — a descent direction for no objective it was measuring | both are scaled |

Measured before/after on the **2026-08-07 legacy PELVIS fixture**, same machine. These rows are the
solver-change receipt, not the current MHR marker receipt:

| | before | after |
|---|---|---|
| dancer marker RMS (true, unweighted) | 5.4913 cm (max 19.46) | **2.1224 cm** (max 5.76), converged=YES in 177 iters |
| dancer, on the old weighted `sqrt(loss/N)` convention | 2.6224 cm | 2.3951 cm |
| dancer drift over 8 identical warm solves | 0.117–0.267 rad, no decay | **exactly 0.0** for 7 consecutive |
| order dependence (same markers, unequal preceding work) | 1.689 rad apart, RMS 5.49 vs 42.17 cm | **exactly 0.0**, identical `‖q‖` 5.746453 |
| continuity under a 2 mm rigid marker shift | 7.43e-2 rad | exactly 2.000e-3 — a pure root translation, the correct answer |
| standing benchmark poses | 0.10–0.14 mm | **0.032 mm** |
| Rajagopal2016 planar, 400 solves | ~6e-3 rad/solve, never stopping | 2.69e-9 rad cumulative, 0.0 per solve |
| dancer muscle relative torque residual (legacy root mapping) | 0.6406 | **0.3545** |

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

⚠️ **Seed sensitivity is real and is not a bug.** On that legacy fixture, the dancer solved from the neutral seed and from a
solved standing pose lands 2.32 rad away at essentially the same fit quality (loss 0.011473 vs
0.011638, RMS 2.1224 vs 2.1526 cm). The problem is non-convex and rank-deficient, so the warm start
is a genuine input. Determinism holds per session state; it is not basin-uniqueness.

✅ **The root-definition part of that 2.12 cm was resolved 2026-08-10.** The old per-marker PELVIS
error was 5.76 cm and dropping it took the remaining 19 markers to 1.5541 cm. The original diagnosis
was directionally right but called raw MHR joint 1 an exact mid-hip point; measurement shows it is
15.081552 mm from the source HJC midpoint. The repair therefore does not globally redefine PELVIS.
It adds source-specific `MHR_ROOT`, keeps live PELVIS unchanged, and preserves raw joint 1 as the
target with the approximation disclosed. Current 20-marker RMS is 1.5365 cm unscaled and 1.2758 cm
after source-aware scaling; shoulder geometry, not the root alias, is now the largest mismatch.

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
| dancer marker RMS (current MHR_ROOT) | 1.5365 cm | **2.2535 cm** (+0.7170) |
| dancer relative torque residual | 0.5940 | **0.5064** (lower, but on a marker-worse pose) |
| dancer `shoulder_rot_r` | 0.5876 rad (33.7°) | pinned 0 |
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

The pre-registered gate ("adopt only if Δ RMS < 0.05 cm on both poses") now fails at 0.7170 cm
(the legacy PELVIS fixture failed at 0.565). The test asserts the failure in both directions, so it
fires if masking ever becomes free. The lower QP residual does not rescue a mask that discards
observed marker information and breaks the standing fixed point.

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
2. ~~**Measure whether trapezius / serratus moment arms are non-zero** about the free thoracic
   DOFs.~~ **GEOMETRY AUDIT DONE 2026-08-11; the proposed product inference is rejected.** All
   852/852 structural rotational pairs were non-zero at neutral and `spine_flexed`, and analytic
   OpenSim arms agreed with independent length differentiation. However all 288 shoulder samples
   are exactly zero, only four paths cross head/neck rotation, the spine coordinates are priors,
   and a non-zero lever arm cannot identify activation or *"overworking"*. See
   [Trapezius / serratus geometry audit](#trapezius--serratus-geometry-audit-2026-08-11) and its
   reproducible script.
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
   claim that "the root-pinned pose source cannot supply accelerations" is **half wrong** — it can,
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
   origins sit off the humeral long axis. On the current MHR_ROOT fixture masking costs the dancer
   0.7170 cm of marker RMS (legacy PELVIS: 0.565 cm) and breaks
   convergence at standing. **Do not re-open without re-measuring that column norm first.**
9. ~~**Plausibility gate on the offline path.**~~ **DONE 2026-08-07** — see
   [Body-size gate](#body-size-gate-2026-08-07).
10. ~~**Static-hold detection**~~ — same item as next-step 5, now done and re-scoped; see there.

### Newly opened by the 2026-08-07 work

11. ~~**`MuscleOverlay.computeBodyFrame`'s `forward` points posteriorly.**~~ **DONE 2026-08-07.**
    Commit `bd1ce43` changed the third axis from `right × up` to the anatomical
    `up × right`, then re-orthogonalised the subject-right axis. The permanent
    `BodyFrameOrientationTests` assert anterior direction, orthonormal/left-handed
    anatomy, and quadriceps in front of hamstrings; all **3/3** passed unchanged
    in the later **526/526** fast-gate receipt. Rendering only — no solved number
    changed. This item remained open here after its implementation was already
    committed and tested.
12. **Validate the findings layer on real photos** (see next-step 3's remainder). Currently the
    highest-value open item, because it is the only part of the product a user can check against
    their own photo.
13. **Measure IK cost in a Release build on device.** The 1567 ms/frame moving-input figure is a
    Debug simulator number and the app's live path depends on it. If it is real, the untried lever is
    the Woodbury/dual form of the normal equations (169×169 vs a 60-row residual, ~8× on the cubic
    term).
14. ~~**E1's 163-coordinate partition no longer covers the 169-coordinate model.**~~ **CLOSED
    2026-08-07.** The SHOULDER6 partition restored full coverage and `testE1RunAll` passed in
    **5706.9 s**. It is now the slow lane's exact one test. The separate V4 “does the harness
    reproduce production” probe remains structurally stale: it compares the bridge against a
    reimplementation of `refineIK` that production no longer uses, and it carries no assertion.
    Its number is diagnostic, not slow-lane evidence.

### Newly opened by the 2026-08-08 muscle-claim scoping

15. ~~**The 3-D muscle overlay still picks its strongest 24 muscles by the same uncalibrated
    cross-muscle number.**~~ ~~Closed for the offline running path on 2026-08-08; still open for the
    LIVE ARKit path, which shares `MuscleOverlay` verbatim.~~ **CLOSED on both paths, 2026-08-08.**
    The renderer takes no muscle solve at all: fixed 26-capsule anatomical set, one constant colour,
    a note on each screen. The live `MuscleActivationBar` — the same claim in numbers — is deleted.
    See [Sixth round](#sixth-round-the-same-claim-in-colour-2026-08-08).
16. ~~**The left/right claim depends on the two legs' moment-arm errors being IDENTICAL.**~~
    **Overtaken on 2026-08-08.** The claim needed more than that: it needed the two legs' TORQUES to
    be proportional, which is both unmeasured and — where it holds — fatal to the claim's content
    (every muscle then reads the same figure). The claim is retired. The one-sided figure stands at
    **23.8 pp** and the shape-asymmetry leak at **9.92 pp**, both in
    `MomentArmErrorCancellationTests`.

### Newly opened by the 2026-08-08 retirement

17. **The route that is not exposed to this defect: compare LEFT/RIGHT at the JOINT, not at the
    muscle.** `τ = M·q̈ + C − JᵀF` never touches a moment arm — the moment arms enter only in the
    muscle QP — so a left/right comparison of the hip, knee and ankle moments at mid-contact carries
    none of the leak that retired the per-muscle claim. It is NOT free: it still inherits the
    fabricated fore-aft term (0.2-0.35 BW), the timing-derived vertical force, and the per-leg `Fmax`
    contact-time contamination, and there are still only 4-6 contacts a side — so the multiplicity
    arithmetic in `GaitClaimSurvivalTests` applies to it too, over a family of 3-6 joints instead of
    175 muscles, which is a very different number (`t` at `α/6`, df=4, is 4.77 rather than 11.90).
    Preregister the gates before building it.
18. ~~**Repair the 76 unmodelled `PathWrap` references, or bound their moment-arm error.**~~
    **ALL 76 DONE 2026-08-08** — cylinders first
    ([cylinder wrapping](#cylinder-path-wrapping-ships-2026-08-08), 64 references, error on the 56
    cylinder-only muscles from median 0.972 mm / max 146.6 mm / 661 sign flips to median 0.048 mm /
    max 8.07 mm / 4), then the ellipsoids
    ([ellipsoid wrapping](#ellipsoid-path-wrapping-ships--every-pathwrap-in-the-model-is-solved-2026-08-08),
    the remaining 12, single-wrap max 4.414 mm against OpenSim's own derivative, engagement 600/600,
    sign flips 135 → 0). `unmodelledPathWraps` is 0 and
    `GaitLoadSummary.musclesWithUnmodelledPaths` is empty. **What remains on this line, in the order
    it matters:** (a) ~~**THE re-measurement**~~ **DONE 2026-08-09** — see
    [the re-measurement](#the-re-measurement-the-moment-arms-are-fixed-and-the-claim-still-cannot-come-back-2026-08-09).
    Median moment-arm leak **0.977 pp** against the straight line's **7.939 pp**, and the
    three-muscle rig at the measured p99 residual reads **0.568 pp** where it read 9.92 pp — but the
    claim STAYS retired, because the tail is still 42–123 pp and because a second error was found
    that is larger and is not geometric: the shipping OSQP tolerance alone moves a published
    left/right figure by a median of **14.88 pp**. `perMuscleLeftRightClaimIsSupported` is still
    `false`; (b) the cost in a RELEASE build on the phone, which was
    extrapolated (~36 ms for the cylinders, +1.28× for the ellipsoids in Debug) and never measured,
    and where the two-wrap cylinder solve is 80× the one-wrap solve and one engaged ellipsoid solve
    is 130–450× it; (c) ~~the `MovingPathPoint` linear interpolation~~ **DONE 2026-08-10** — all
    four now use exact SimmSpline evaluation (`4 parsed / 0 approximated / 0 skipped`), lowering the
    affected central-difference maximum 4.414 → 2.679 mm; the remainder is not pre-attributed;
    (d) the 8.75 mm gap on the 4 two-cylinder muscles, which is OpenSim's own
    non-self-consistent iterate and needs a decision about which answer is wanted rather than a bug
    fix.

    The original item, kept because the falsifier it registers is unchanged:
    **Repair the 76 unmodelled `PathWrap` references, or bound their moment-arm error.** This is the
    only thing that reopens the muscle claim. The falsifier is registered on
    `GaitLoadSummary.perMuscleLeftRightClaimIsSupported`: with the wraps modelled, re-run
    `testAShapeAsymmetryMakesABilateralMomentArmErrorLeak` and require the leak below 8.086 %.
    **The "or bound it" half is now closed: the error is measured, not bounded — median 13.7 %,
    9.00 % sign-flipped, worst 146.6 mm** (see
    [the reference](#the-moment-arms-now-have-a-reference-and-the-defect-is-measured-2026-08-08)).
    What remains is the solver: 56 WrapCylinder + 8 WrapEllipsoid, and only the path's scalar LENGTH
    has to be right because `r = dL/dq`. `BioMotionTests/Fixtures/opensim_moment_arms.txt` is the
    gate, `StraightLinePathErrorTests` is the harness already wired to it, and its
    `wrapPoints` column marks the 25 engage/disengage transitions a centred difference must not
    straddle. Three things this stage did NOT do and the next one must: measure the per-frame COST
    (the chain is already ~200 ms/frame and OpenSim's cylinder solve iterates), check `quadrant`
    (getting the wrap side backwards produces a plausible wrong path), and decide the licence
    paperwork for opensim-core's Apache-2.0 `WrapCylinder`/`WrapEllipsoid`/`WrapMath` if that code
    is used (header + NOTICE).
18b. ~~**Close the 4.4 mm implementation residual that is NOT wrapping.**~~ **DONE 2026-08-10 for
    the identified mechanism.** FullBody's four MovingPathPoints now use the canonical SimmSpline
    in-domain and at both endpoint tails; the fidelity report is `4 parsed / 0 approximated / 0
    skipped`. Re-running the affected sweep lowers the central-difference maximum from 4.414 to
    **2.679 mm** and the analytic maximum from 4.385 to **2.301 mm**. The residual is deliberately
    reported without assigning it to ConditionalPathPoint, FK or finite differencing until one of
    those mechanisms is isolated.
19. **The scatter the sampling interval is built from has never been measured on real footage.**
    Every survival number in `GaitClaimSurvivalTests` sweeps it because of that. Real per-muscle
    contact-to-contact scatter needs a clip that reaches the muscle solver — i.e. the 20-marker
    offline path on real video, not the five-joint gait fixtures.

### Newly opened by the 2026-08-08 contact-claim repair (seventh round)

20. ~~**Restore the contact-time finding on `.analysed` with no summary.**~~
    **DONE, then strengthened 2026-08-10.** `GaitTimingReport` now detaches resolution, contact
    timing and flags from the research report. The final panel has no optional muscle summary at
    all, and the unsupported bundled path does not build or run a dynamics plan. (Minor 9.)
21. **Longer clips are the only lever on the contact-claim floor, and it is `1/√n`.** The sampling
    half-width is `t·√(s²_L/n_L + s²_R/n_R)`, so halving `video_015`'s 16.5 % needs ~4× the
    contacts — about 20 a side, a ~16 s steady run against the current 4 s window and the 601-frame
    `maxNativeWindowFrames` budget. Whether the window can be extended without breaking the frame
    budget, and what that costs in memory, is unmeasured. This is the only route to a claim finer
    than ~20 % that does not need a new statistic.
22. **The remaining and closed minors** are tracked in
    [Seventh round](#seventh-round-the-surviving-claim-was-gated-on-the-wrong-variance-too-2026-08-08)
    (minors 1, 3, 5, 6, 7, 8, 9 plus the round-six list). Minor 1 — `framesPerSecond` using the
    NOMINAL rate under the sparse sampler — was closed 2026-08-10 by removing the duplicate source;
    minor 8 — live anatomy renderer/caption gate drift — was closed the same day by the unified
    presentation contract.

### Newly opened by the 2026-08-09 re-measurement (eighth round)

23. ~~**THE MUSCLE CLAIM'S NEW BLOCKER IS THE OSQP TOLERANCE.**~~ **DONE 2026-08-09, and the
    diagnosis in this item was wrong.** It was not the tolerance and not
    `OSQP_SOLVED_INACCURATE`: `eps` at 1e-9 with 20,000 iterations still lands 0.30 from the answer,
    because the DUAL tolerance is relative to `‖q‖∞ ≈ 1e7` while the curvature holding the redundant
    directions is `ε = 0.01`. `scaling = 0` + `polishing = 1` + `max_iter = 4000` took the median
    from **14.88 pp to 4.4994e-05 pp** at **+3 %** cost, `eps` unchanged. Full account:
    [the section above](#the-qp-now-returns-its-own-answer-1488-pp--44994e-05-pp-and-the-claim-still-does-not-come-back-2026-08-09).
    What is left of this item: the max solver slack is **21.98 pp** against a p90 of 0.047, and that
    tail is unattributed — the hypothesis on record is the rig's `interiorMargin = 1e-3` screen
    admitting cells where a muscle sits a thousandth inside the bound, not a solver failure. The
    worst cell is printed with pose/shape/effort/base.
24. ~~**Settle the moment-arm tail, or stop quoting it as one number.**~~ **DONE 2026-08-09, and the
    muscle names in this item were both wrong.** `piri` and `glmed3` were the worst SHIPPED base of
    the cell, printed beside `leakExact` as if they were its muscle; R1's worst is `bflh140`, at the
    same two cells. All three carry no `PathWrap`. The tail is the SHARING STEP carrying a
    neighbour's moment-arm error onto a muscle whose own path is exact — proven, not inferred, by
    `bflh140` having no finite-difference row, so its arm is the same number in both reference
    matrices while its figure moves 126.44 pp between them. The reference's own two columns disagree
    by **126.44 pp** worst / **5.28 pp** median, larger than ours in 466 of 582 cells. The
    "dump it in METRES" discriminator this item asked for now prints as
    `LEAK-METRIC worst_cell_arms`. Full account:
    [the leak re-run](#the-leak-experiment-re-run-the-claim-does-not-come-back-and-the-largest-remaining-term-is-the-reference-2026-08-09).
    **What is left of this item, and it is now the top of the list: R1 cannot be passed as
    registered.** It is maximised over the worse of two `truth` definitions that differ by 78× its
    own bar. Either the gate names ONE reference this repo can defend — the reconciled column of
    `MultiWrapReferenceTests`, extended past the 8 multi-wrap muscles (next-step 30) — or it is a
    measurement of OpenSim's bookkeeping. The later SimmSpline endpoint fix reduced the analytic
    column alone from **42.46 to 3.693 pp** without touching this registered-reference problem; see
    next-step 34 and the endpoint-extrapolation section.
25. ~~**The real problem is 520 × 169 and the rig is 80 × 12.**~~ **DONE 2026-08-09.**
    `MuscleSolverExactnessTests.testTheShippedSolverIsExactOnTheRealFiveHundredMuscleProblem` solves
    the real 520-muscle × 109-coordinate problem with both solvers. `BoxQP` reaches a KKT residual of
    8.5e-13 in 138 active-set iterations (26 s, Debug). The shipping solver's departure from it went
    from **0.10269** (355.8 pp of a left/right figure at the median interior activation) to
    **1.017e-04 over interior muscles** (0.352 pp), 5.88e-04 counting a muscle the exact solution
    puts on a bound. The inference was directionally right and quantitatively low: the real problem
    was WORSE than the rig, not the same.
26. **The STRIDE case is still unmeasured.** This rig mirrors ONE pose, so both legs carry the same
    modelling error by construction. A real clip samples each side at its own mid-contact and nothing
    checks that those two poses are comparable; with the OLD arms that one-sided case cost 23.8 pp.
    It needs the two mirrored run poses (`run_1_midstance` / `run_4_mid_swing` are an exact pair in
    the fixture) and a statistic taken across two solves rather than within one.

### Newly opened by the 2026-08-09 solver fix (ninth round)

27. **The 21.98 pp solver-slack tail is unattributed, and it is the one pre-registered gate this
    round failed.** Median 4.4994e-05 pp, p90 0.0471, max **21.981** — a tail 466× its own p90 on
    582 cells. The hypothesis on record is the rig's own screen, not the solver:
    `WrappedMomentArmLeakTests.interiorMargin = 1e-3` admits a cell where a muscle sits a thousandth
    inside `aMin` in the EXACT solution, which any finite solver may place on the bound, after which
    `100·(a_l − a_r)/mean` divides by a number near `aMin`. `testTheShippingSolversOwnSlackIsBelow
    WhatAnyClipCouldResolve` now prints the worst cell's pose, shape, effort, reference and screened
    count — start there. If it IS the screen, the fix is to state the margin in units of the statistic rather
    than of the activation; if it is not, `polishing` is failing on those cells and
    `info->status_polish` is the reading (it is −1 on some real poses and +1 on others).
28. **`saturationActivationTolerance` is now a ~200× conservative screen.** It is
    `10·(eps_abs + eps_rel)` = 0.02, and a clipped activation now comes back within ~1e-07 of its
    bound (measured: worst departure over the whole 520-muscle problem 5.88e-04). So
    `GaitLoadSummary.saturationThreshold = 0.98` can call a genuinely interior muscle at 0.985
    saturated and drop it from a comparison. That is the SAFE direction and it was left alone
    deliberately — tightening it changes which muscles are screened, which is a `GaitLoadSummary`
    decision with its own gates and its own fixtures (`GaitLoadSummaryTests` pins 0.98 and builds
    frames at 0.982). If it is tightened, the falsifier is: no muscle that the exact solve puts on a
    bound may read as interior on any pinned clip.
29. **The QP costs 194 ms of a ~200 ms/frame chain in Debug, and nobody has measured it in Release
    or on the phone.** The fix added 3 %, so it did not create this, but the measurement now exists
    at 520 muscles × 109 coordinates and it is the largest single term in the chain. `P` is built
    DENSE upper-triangular (520×520 = 135k nonzeros) and re-factorised every frame; the standard
    OSQP formulation for a least-squares objective introduces `t = A·a` and keeps `P` DIAGONAL, which
    would make the KKT sparse. That is a rewrite of the OSQP wiring, not a constant, and it should be
    gated on `MuscleSolverExactnessTests` before and after.

### Newly opened by the 2026-08-09 multi-wrap measurement (tenth round)

30. **The two-cylinder muscles now have a valid reference; the other 62 wrapped muscles do not have
    one that is checked for this defect.** `opensim_multiwrap.txt` reconciles only the 8 multi-wrap
    muscles, because `_adjust_tangent_point` is the only mechanism found that separates OpenSim's
    reported points from its stored arc, and it runs only when `PathWrapSet.size() > 1`. That is an
    argument from the code path, not a measurement over the single-wrap class. The cheap check is to
    add `min(stored arc − chord)` to `dump_reference.py` for ALL 66 wrapped muscles and assert it is
    non-negative; if any single-wrap muscle comes back negative, C1/C2/C5's bars are measuring the
    same artefact at a smaller size.
31. **The ellipsoid spirals are unreconciled by construction.** `CalcDistanceOnEllipsoid` sums
    chords rather than evaluating a closed form, so "the shortest surface path between the reported
    tangent points" has no cheap expression. It did not matter here — the ellipsoid slack is
    non-negative and `TRIlong`/`BIClong` match the reported column to 0.0000 mm — but a future model
    where an ellipsoid multi-wrap muscle disagrees needs that work before it can be attributed.
32. **`poses.py` should gain the reference's jump poses, once someone owns the fallout.** The
    generator can now find them by rule (`dump_multiwrap.py` stage 1). Adding them to the shared
    grid would regenerate `opensim_moment_arms.txt` and `opensim_moment_arms_fd.txt` and move every
    population in `CylinderWrapValidationTests` and `EllipsoidWrapValidationTests`. The fixed 1 mm
    sign thresholds themselves do not move; there is no runtime control floor. W1's multi-wrap
    component would then fail at ~41 mm for the reference's reason.
    Deliberately not done in the same round as the measurement.

### Newly opened by the 2026-08-09 leak re-run (eleventh round)

33. **R1 needs ONE reference, and the repo now knows how to build it.** The gate is maximised over
    two OpenSim columns that disagree by 126.44 pp on this rig, so it cannot certify anything about
    `MomentArmComputer`. `MultiWrapReferenceTests`' reconciled column is the model: recompute the
    reference's total from its OWN reported tangent points. Extending it past the 8 multi-wrap
    muscles is next-step 30's cheap check; a `WrapValidationHarness` sample carrying a
    `reconciled` field beside `wrapOn`/`centralDifference` would let R1 be re-registered against a
    single defensible truth. **Do not re-register it quietly** — R1's current value would change,
    and a gate whose reference is chosen after a number is read is not pre-registered.
34. ~~**Localise and explain the 42.46 pp ANALYTIC tail.**~~ **DONE 2026-08-09.** The 24-row dump
    first rejected the neighbour hypothesis: at `run_4_mid_swing`, `bflh140_r` itself carried knee
    **16.059 vs 13.713 mm (+2.346 mm)** and a −42.462 pp figure movement. The kinematic seam then
    closed it: `walker_knee_r` permits 140° while five `SimmSpline` axes end at 120°, and Nimble was
    continuing their last cubics instead of OpenSim's endpoint tangents. Two-sided value/d1/d2 RED,
    both archives rebuilt, product regression **13.713464915 vs 13.713465000 mm** at 130°. The
    analytic maximum is now **3.6932 pp** (p99 3.3322, median 0.3121), still 2.28× the gate; the
    separate registered-reference tail remains. Tracked patch and full account are in the
    endpoint-extrapolation section.
35. **The sharing step's amplification is unbounded per muscle and nothing measures it.** At the
    separate central-difference cell, a muscle with an exact row took 126 pp from its neighbours.
    That is a property of the QP's coupling and
    it applies to every per-muscle statement this product could ever make, not just to this gate —
    so "validate a muscle's path, then trust its row" is not a valid inference and no future
    per-muscle claim may rest on it. The measurable form: the sensitivity of muscle `i`'s left/right
    figure to a perturbation of muscle `j`'s moment arm, which is one Jacobian of the QP solution
    map and is cheap on the exact solver.

### Newly opened by the cam_t measurement (2026-08-07)

15. **Pass `camT` at `OfflineSessionRunner.swift:242`.** One argument. Until it lands,
    `MHRRetarget.makeBodyFrame(jointCoords:camT:…)` is exercised only by `RootTranslationTests` and
    `rootTranslationObservable` is false on every real frame. It also fixes things that have nothing
    to do with dynamics: the ground-height estimator, GRF contact detection and the CoP all currently
    run on a body whose pelvis sits at a model constant.
16. ~~**Surface `MotionVerdict` in the UI.**~~ **DONE 2026-08-07.** Commit
    `5e9b370` removed the parallel `hold`/`moving` taxonomy:
    `OfflineResultStore.MotionState` now carries the engine's exact
    `MotionVerdict`, and `OfflinePlaybackView.motionDetail` renders its advice.
    `.indistinguishableFromNoise` shows the measured pose-noise floor beside
    the speed instead of blaming the user; `.movingBeyondStaticBudget` retains
    the distinct hold-still remedy. The later **526/526** fast gate exercised
    the current verdict/store/UI target with zero failures or skips.
17. **Build the camera-static check, upstream where the frames are.** Measured to separate the
    owner's clips by 20-60× (§ cam_t). Two constraints from the measurement: it must run at the
    video's NATIVE rate (at 10 fps sampling the estimator aliased a 13.5 °/s pan down to ~0), and it
    detects rotation well but translation poorly. On iOS the natural tool is
    `VNTranslationalImageRegistrationRequest` on the region outside the person box. It is cheap —
    no pose model — so it can run densely even when the pose sampling is sparse.
18. **Decide (a) refuse vs (b) declare-depth-constant** — see the owner decision in the cam_t
    section. This is what gates whether a dynamic branch exists at all.
19. ~~**Drop Vision-fallback frames out of any derivative.**~~ **DONE 2026-08-10.** The 22/309
    frames remain visible as reviewable poses, but video fallback now branches before scale/Nimble/
    gait and splits both solve passes. Photo fallback remains analysable; raw decoder slots and
    requested endpoints prevent gap compression or false edge padding.

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
