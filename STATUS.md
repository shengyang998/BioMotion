# BioMotion — STATUS

**Single source of truth for progress. Read this before touching anything.**
Last updated: 2026-07-22.

---

## TL;DR

The app's inaccuracy was diagnosed to root cause. It was never one bug — it is a chain, and the
biggest links were **not** where the effort had been going.

- Five implementation defects were found, fixed, and pinned with tests. The test target
  **did not even compile** before this work, so the project had no regression net at all.
- The dominant remaining error source is **not** the muscle solver: it is that IK solves
  **163 degrees of freedom from ~40 scalar observations**. ~127 of those DOFs are spine and rib
  coordinates that ARKit cannot see.
- The shipped model's **shoulders are welded** (zero shoulder DOFs), so upper-limb muscle output
  is currently meaningless regardless of anything else.
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

| Method | Joint-angle error |
|---|---|
| Marker-based (Vicon, gold standard) | ~2–3° |
| OpenCap (2 synchronised cameras) | **4.1° MAE** (RMSE 2.0–10.2°) |
| Monocular SMPL (HMR2.0 class) | **8.5° MAE** |
| **ARKit body tracking (current input)** | **18.8° ± 12.12° MAE** (per-joint 3.75–47°) |

**Do not attempt real-time ViT-H HMR2.0 or WHAM+SLAM on iPhone.** ViT-H is ~630M params (~1.2–1.3 GB
fp16); the reference implementations are offline CUDA batch scripts. HMR2.0 is also per-frame and
camera-relative — it fixes single-frame pose plausibility while leaving acceleration and GRF, the
actual muscle-side bottlenecks, untouched. WHAM's expensive half (camera pose + gravity) is already
solved for free by ARKit; the useful part to borrow is its **architecture** — a small AMASS-trained
temporal denoiser + foot-contact head, ANE-sized — not its code.

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

4. **IK null-space damping / runtime DOF masking.** Use the existing `math::IKConfig` at
   `NimbleBridge.mm:690` to lock unobservable coordinates at runtime rather than welding them in the
   model file — reversible, no new shipped artifact, no rename, no patella bake, and it avoids
   nimble's own documented penalty for intermediate WeldJoints (`OpenSimParser.cpp:5229`).
   This single change targets the red test, the ~200 ms/frame cost, and the `ddq` noise at once.
   ⚠️ **Do not weld the sternum or costovertebral joints.** Because the clavicle and scapula are
   already welded, those are the shoulder girdle's *only* articulation; welding them kills all 24
   trapezius and all 20 serratus slips — the scapular stabilisers, i.e. exactly the muscles behind
   the rounded-shoulder findings the product would sell.
5. **Static-equilibrium inverse dynamics.** Add a hold detector (marker speeds < ~2 cm/s for ≥ 0.5 s),
   average the pose over the hold, and run ID with `q̇ = q̈ = 0`. This deletes the 1/dt²
   amplification chain entirely on the static path, and it matches the product's own framing
   ("current posture"). Largest single accuracy lever available.
6. **Patella rename + weld** (with the `groupScale()` patch). Ship blocker for squat analysis.
7. **Shoulder axis orthogonalisation** (6 lines). Now known to actually pay off, since shoulder
   muscles carry no wraps. Sequence after step 1 so it is not applied to data that may be replaced.

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
