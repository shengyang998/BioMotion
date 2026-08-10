# Adversarial verification of the inverse-dynamics fix

> **Historical raw-solver receipt — not product validation.** This 2026-08-07 audit correctly checks
> gravity direction, wrench-frame transforms, and statics algebra inside the then-active solver. It
> predates the finding that both bundled `ContactGeometrySet`s are empty and the near-CoP routine
> enforces no support polygon, unilateral-contact, or friction constraint. Its torques, GRFs, and
> CoPs are unvalidated engineering diagnostics; production now returns pose-only and does not expose
> them. "CONFIRMED" below applies only to the two frame/convention fixes it names.

Auditor: independent agent, 2026-08-07. Source was read but never edited. Everything below was
either measured on this machine or derived from `FullBody.osim` from scratch.

**Historical verdict: CONFIRMED within that raw diagnostic.** I could not break the two named
defects. They are frame/convention errors rather than tuned numbers, and the corrected pipeline reproduces from-scratch free-body
statics to 4 decimal places on six poses I built myself. Three secondary claims are wrong or
overstated and are listed at the end.

---

## Tooling used

Two independent instruments, neither of which reuses the app's code path:

1. `/tmp/idverify/fk.py` — a from-scratch parser + forward-kinematics + mass-table computation over
   `BioMotion/Resources/FullBody.osim`. Used to re-derive the benchmark's "hand-computed" constants.
2. `/tmp/idverify/idcheck2.cpp`, `idcheck3.cpp` — standalone iOS-simulator binaries linked against
   the same vendored `nimblephysics/build_sim/libnimble_ios.a` the app links, run with
   `xcrun simctl spawn`. They re-implement `solveIDGRF` under three selectable conventions
   (`FIXED`, `PREFIX` = gravity-Z + world frames, `D1only` = gravity-Y + world frames), on poses I
   construct, and check the reported torques against a free-body statics computation whose only
   inputs are FK world transforms, the `.osim` mass table and the solved contact wrench.
   These do not touch DerivedData.

---

## 1. Tuned constant? NO

`git diff` contains no multiplication, clamp, rescale or threshold on any torque. The three
substantive edits are:

| Edit | What it is |
|---|---|
| `setGravity(0, -9.81, 0)` | the value the model itself declares (`FullBody.osim:38` = `0 -9.8066 0`) and the value every nimble biomechanics entry point sets (`SubjectOnDisk.cpp:807/844`, `DynamicsFitter.cpp:13706`) |
| `math::dAdT` on the guess / `math::dAdInvT` on the readback | exact SE(3) dual-adjoint transforms from `dart/math/Geometry.cpp:1504/1530`; identical to the reference caller `DynamicsFitter.cpp:16538-16556` |
| `math::projectWrenchToCoP` | nimble's own function, replacing a hand-rolled projection |

Both defects verified against the vendored source, not just asserted:

* **D1.** `SkeletonAspect.hpp:82` really does default gravity to `Vector3s(0,0,-9.81)`, and
  `OpenSimParser.cpp` contains **zero** `setGravity` calls. Measured directly with a standalone
  probe: after `parseOsim`, `gravity = 0 0 -9.81`.
* **D2.** `Skeleton.cpp:9865-9868` and `10294-10299` build the contact Jacobians with
  `getJacobian(bodies[i])` under the in-source comment *"This is the Jacobian in local body space.
  We're going to end up applying our contact force in local body space, so this works out."*
  `Skeleton.cpp:10205` maps the guesses with `dAdInvT` before projecting to a CoP, and
  `Skeleton.cpp:10352-10354` returns `x` with the author's world conversion commented out.
  The wrenches in and out are body-local. Confirmed.

`-9.81` vs the file's `-9.8066` is a 0.035 % (0.27 N) discrepancy, and it is self-consistent with
the pre-existing `weightUp = mass * 9.81` two lines away. Not material; noted, not a defect.

---

## 2. Does it explain the reversed gradient? YES — and criterion 2 as worded is bad physics

I reproduced the pathological signature on demand by reverting the fix inside my own probe.

Pose A (quiet stance, feet flat, 4° lean, both feet down), same solve, three conventions:

| convention | hip_r | knee_r | ankle_r | subtalar_r | GRF sum |
|---|---|---|---|---|---|
| **FIXED** | 3.56 | 4.15 | **−14.26** | 3.78 | (0, 780.71, 0) |
| PREFIX (gZ + world) | −644.5 | 394.1 | −192.6 | −256.9 | (3.7, −2.4, **764.5**) |
| D1only (gY + world) | 3.65 | 25.8 | −26.3 | **47.8** | (8.6, 780.7, 0) |

Pose E (single-leg stance on the right — the same single-contact topology as the dancer fixture):

| convention | hip_r | knee_r | ankle_r | subtalar_r |
|---|---|---|---|---|
| **FIXED** | −29.83 | 44.55 | −62.64 | −11.88 |
| PREFIX (gZ + world) | 0.00 | −5.24 | 70.61 | **−529.9** |

The pre-fix convention reproduces the exact reported symptom shape on a pose it was never tuned
on: hundreds of Nm, subtalar dominant on a single-contact stance (−530 here vs the reported 672),
and — the giveaway — gross left/right asymmetry on a mirror-symmetric pose (pose A PREFIX:
`hip_r = −644.5`, `hip_l = +644.5`), because gravity along z breaks the sagittal mirror plane.
That is the mechanism, not a magnitude coincidence.

**But the brief's criterion 2 is itself wrong**, and pose E proves it inside a single solve:

* right leg (loaded through the ground): hip 29.8 → knee 44.5 → **ankle 62.6** — grows distally
* left leg (lifted, free-hanging): hip 37.2 → knee 5.7 → ankle 0.90 → subtalar 0.44 — falls distally

Same frame, same solver, same gravity. "Less mass is distal so distal torques are smaller" is a
statement about a free limb and it is visibly true on the free limb. On a limb loaded through a
ground contact the dominant load enters at the foot, so the ankle is legitimately the largest
moment — which is the textbook picture of quiet standing. The previous agent's refusal to assert a
distal ordering is correct, and its replacement (the per-joint lever bound
`|tau_j| ≤ |F|·|p_CoP − c_j| + W_distal·L_distal`) is a genuine statics identity.

---

## 3. Does it generalise? YES — exact on six poses I built myself

Poses constructed in the standalone probe (joint coordinates set directly; `pelvis_tilt` bisected
so the stance foot is flat). Reported torque vs my from-scratch free-body statics
`tau_j = −omega_j · [ Σ_distal (com_i − c) × m_i g + contact wrench moved to c ]`, with `omega_j`
and `c` obtained by finite-differencing forward kinematics (no reuse of the ID code):

| pose | CoM vs ankles | contacts | max &#124;reported − independent&#124; over 5 pin DOFs | GRF sum error | net CoP − CoM |
|---|---|---|---|---|---|
| A quiet stance 4° lean | +3.9 cm | L+R | **0.0000 Nm** | 0.0000 N | 0.0000 m |
| B forward lean 12° | +15.9 cm | L+R | **0.0000 Nm** | 0.0000 N | 0.0000 m |
| C backward lean 6° | −10.8 cm | L+R | **0.0000 Nm** | 0.0000 N | 0.0000 m |
| D half squat (see note) | — | L+R | **0.0000 Nm** | 0.0000 N | 0.0000 m |
| E single-leg, right | — | R only | **0.0000 Nm** | 0.0000 N | 0.0000 m |
| F deep squat (see note) | +0.4 cm | L+R | **0.0000 Nm** | 0.0000 N | 0.0000 m |

Note: my flat-foot bisection picked an inverted root for poses D and F, so those two are not
anatomically plausible configurations. The identity held there anyway, which is the point — it is
an identity, not a fit. A/B/C/E are anatomically sensible (toe ahead of heel, knee below hip,
CoP between heel and toe on A and E).

Within the historical free-body assumptions, these magnitudes are internally
consistent. This is **not** physical validation of the absent contact-support
model:

* A — CoP−ankle lever 3.90 cm × 390.4 N × 0.979 (talocrural axis z-component) − 0.98 Nm of foot
  weight = **14.0 Nm**; reported 14.26. Ankle is the largest raw leg moment in this construction.
* B — 12° lean, lever 15.9 cm → **60.2 Nm** ankle, CoP right at the front of the foot. That is the
  edge assumed by this free-body construction.
* C — CoM behind the ankles → raw ankle torque **flips sign** (+42.0, dorsiflexor), as the assumed
  moment equation requires.
* E — one foot carries |F| = 780.71 N (full bodyweight), CoP between heel and toe.

Non-quasi-static check (`idcheck3`): with `ddq(pelvis_ty) = +3 m/s²` the solved GRF rises to
1019.5 N = m(g+a) exactly, and the new `rootResidualNorm` formula reports **0.0000 N** while the
quasi-static form would report 238.75 N. With `ddq(pelvis_tx) = +2 m/s²` the CoP moves *behind* the
CoM (−0.2632 vs −0.0744) — the correct direction for forward acceleration. The `m·a_com` term is
right and `getCOMLinearAcceleration()` correctly excludes gravity.

---

## 4. Is the benchmark honest? YES — but its headline hand-derivation has a real arithmetic slip

I re-derived every model constant the test file claims, from the `.osim`, independently:

| quantity | test file claims | my FK/parse | |
|---|---|---|---|
| total mass | 79.5835 kg | 79.58347 | ✓ |
| whole-body CoM rel. pelvis | (−0.0746, −0.0006, 0) | (−0.07456, −0.00063, 0.00000) | ✓ |
| talus (ankle JC) origin | (−0.0627, −0.8846, ±0.0770) | (−0.0627, −0.8846, ±0.0770) | ✓ |
| talus+calcn+toes mass | 1.567 kg | 1.5666 | ✓ |
| talocrural axis | (−0.1050, −0.1740, 0.9791) | (−0.10501, −0.17402, 0.97913) | ✓ |
| subtalar axis | (0.7872, 0.6047, −0.1209) | (0.78718, 0.60475, −0.12095) | ✓ |

The pose is built from rigid geometry the markers cannot change, plus explicit free choices
(lean angle, straight legs, flat feet, 50/50 stance). Nothing is back-fitted.

**The slip.** The docstring says *"the foot's own weight (1.567 kg) acts 0.0009 m ahead of the
ankle, worth 0.02 Nm, and is dropped"*. 0.0009 m is the foot CoM's x relative to the **pelvis**,
not the ankle. The true lever is **0.0636 m**, worth **0.98 Nm**. Redoing it:

```
M_z(GRF)  = 390.357 N × 0.05005 m            = +19.538 Nm
M_z(foot) = 0.0636 m × (−15.368 N)           = − 0.977 Nm
M_x(GRF)  = −(CoP_z − ankle_z) × 390.357     = − 3.244 Nm
tau = (−0.10501)(−3.244+0.078) + (0.97913)(18.561) = 18.51 Nm
```

**My independent number is 18.2 Nm** using only the mass table and geometry (per-foot lateral CoP
unknown), or **18.51 Nm** if I use the solver's own per-foot CoP_z. Measured: **18.5468 Nm**
(agreement 0.2 %). The claimed 19.1 Nm is 3 % high and in the *conservative* direction — which is
itself evidence the number was derived, not reverse-engineered from the output. The benchmark is
honest; `benchmarkAnkleExpectedNm: 19.1` should read ~18.2.

Sharpest assertion in the file (`net CoP under CoM`, tolerance 1 cm) checks out independently:
hand-derived −0.01264 m, measured −0.012637 m. My probe reproduces `net CoP == CoM` to 4 decimals
in all six of my own poses, and `D1only` misses it by 6–21 cm — so that assertion really does
isolate D2. The rest of the bounds are loose (ankle < 45 Nm against a 19 Nm derivation): they
would catch the 472 Nm bug by 10× but would not catch a 30 % error.

---

## 5. Regressions? NONE FOUND

Everything below was run by me, on this tree, not taken on report.

| suite | result |
|---|---|
| DOFMaskTests | 9/9 pass |
| MomentArmTests | 11/11 pass |
| MuscleSolverTests | 11/11 pass |
| OfflineMuscleChainTests | 1/1 pass |
| OfflineOrchestrationTests | 1/1 pass |
| StaticEquilibriumBenchmarkTests | 7/7 pass |
| BodyJointTests / CalibrationTests / IKDriftDiagnosticsTests / MotionRecorderTests / TRCExporterTests | 50/50 pass |
| NimbleBridgeTests | 21/22 — `testRepeatedIKOnIdenticalMarkersIsStable` fails (0.005748 vs 1e-3) |

**112 tests, 1 failure.** That failure is pre-existing and provably unrelated:
`BioMotionTests/NimbleBridgeTests.swift` is byte-identical to HEAD (`git diff --quiet` passes), and
its own HEAD comment at line 350 already says *"it is what makes
testRepeatedIKOnIdenticalMarkersIsStable fail … (0.006 → …)"*. `solveIK` contains no reference to
gravity or to any dynamics call, so `setGravity` cannot reach it.

`scaleModelWithHeight:` does **not** re-parse the model, so the gravity setting survives scaling.
`parseOsim` appears exactly once in shipping code, with `setGravity` on the next statement.

---

## 6. Where the claim is wrong or overstated

1. **`benchmarkAnkleExpectedNm: 19.1` is not the hand-derived value.** The foot-weight lever was
   taken relative to the pelvis instead of the ankle. Correct value ≈ 18.2 Nm (18.51 with the
   solver's own lateral CoP). Cosmetic; the conclusion is unchanged and the error is conservative.
2. **`explainsReversedGradient` needs a qualifier.** The magnitude *and* the pathological ordering
   are both explained, and I reproduced them by reverting. But the fixed pipeline still shows
   torque growing distally on a *loaded* limb, and that is correct physics, demonstrated on the
   free vs loaded limb inside a single solve (pose E above). Anyone reading criterion 2 literally
   will think the fix failed; it did not, the criterion is wrong.
3. **E1 was not re-run and its torque statistics are now stale.**
   `E1MarkerSetComparisonTests.mm:376` takes `_skel = [_bridge sharedSkeleton]`, i.e. the skeleton
   `loadModelFromPath` now gives Y-gravity, and lines 791 and 1184 call
   `_skel->getInverseDynamics(...)` on it. `tauHat − tauTrue` therefore contains
   `G(q̂) − G(q_true)`, which changes with the gravity direction. The archived
   `E1_results.json` torque numbers were produced under Z-gravity and no longer describe this
   model. The E1 STOP verdict itself rests on kinematic gates (spine error, spurious
   intervertebral motion, `ddq`), which are gravity-independent, so the verdict is almost certainly
   safe — but nobody has confirmed that, and the previous agent did not claim to. Cost to close:
   one E1 run (>1 h).

   **Superseded later the same day:** the SHOULDER6 partition fix restored all 169 coordinates and
   the 5706.9 s E1 rerun passed; see `STATUS.md`. This paragraph remains the audit's historical
   state at the time it was written.

---

## 7. Remaining concerns (none of them regressions; all pre-existing or out of scope)

* **The double-support load split is imposed, not solved.** With both feet down the six root
  equations leave the vertical split indeterminate, and the solver inherits it from the
  `weight/contactCount` guess. In every two-foot pose I tried the split came back exactly
  50.00/50.00 (390.3569 N each) regardless of lateral lean. Per-leg torques in an asymmetric
  double-support pose are therefore an assumption, not a measurement. Unchanged by this fix.
* **Nothing constrains the CoP to lie inside the foot.** In my backward-lean pose C the solved CoP
  landed 6 cm behind the heel, and in the 12° lean it sat past the toes. ID is faithfully reporting
  an unbalanceable pose, but it means the "support-polygon bound" the benchmark asserts is a
  property of *that* pose, not a guarantee. A badly-estimated photo pose can still produce a large,
  physically unrealisable ankle torque without anything flagging it.
* **`projectWrenchToCoP` has no zero-load guard.** The deleted `copFromWrench` had
  `if (abs(force.y()) < 1e-6) return ctr;`. The new path relies on
  `completeOrthogonalDecomposition()` degrading to a minimum-norm solution. It will not produce
  NaN, but a near-unloaded foot in contact now reports a CoP near the world origin rather than the
  foot centre. Low severity, untested.
* **`rootResidualNorm` is now ~0 by construction whenever the frames are right** (measured 5.8e-13
  and 7.2e-13). It is a good frame check and a useless balance check — which the new header comment
  says. But `ContentView.swift:136-137` still renders it as `"%.2f Nm/kg"` (wrong unit, disclosed)
  with a `< 0.5 = good` badge that is now always green. The old value was a hard-coded zero, so no
  information was lost; the UI is just showing a tautology.
* **The muscle QP still mixes units.** `SternumY` is `translation2` on axis `(0,1,0)` of CustomJoint
  `r1R_sterR_jnt` (`FullBody.osim:14421,14533`), so its generalised force is a **force in newtons**;
  it is the 2nd largest entry in the dancer solve (54.1) and the largest in both standing poses
  (72.5). `MuscleSolver`'s `‖A·a − tau‖` sums newtons and newton-metres in one norm. Real,
  pre-existing, correctly flagged by the previous agent, not addressed here.
* **Muscle saturation is not fixed** (14 → 17), `relativeTorqueResidual` 0.612 still exceeds the
  0.3 line `MuscleSolver.h` documents. Verified from the run: only 4 of 169 coordinates now exceed
  their musculature's capacity and all four are wrist DOFs with capacity exactly 0.0 and demand
  < 0.3 Nm. Top demand is `knee_angle_r` at 78.1 Nm against a capacity of 760.9. So this is no
  longer an ID-magnitude problem — correctly reported as unresolved.
* **`STATUS.md:596-599` attributes the 672 Nm to pelvis pinning.** That is wrong and I confirm it
  independently: `max_ddq = 1.67e-16` on the dancer fixture, so no dynamic term exists to attribute
  anything to. Should be corrected when STATUS is next updated.
* **Y-up is hardcoded.** `setGravity(0,-9.81,0)` and `verticalAxis = 1` assume the model is Y-up.
  True for `FullBody.osim`; brittle if another model is ever loaded. Reading `<gravity>` from the
  file would be strictly better.
