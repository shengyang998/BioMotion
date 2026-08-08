# BioMotion

Musculoskeletal analysis iOS app. **Two input paths**, one shared solver chain:
a live ARKit body-tracking path, and an offline path that analyses an imported
photo or video through a Core ML SAM 3D Body model.

> **Setup, dependency cloning, patching, and full build/TestFlight commands live in [`README.md`](./README.md).** This file is LLM-facing context for architecture and gotchas only.
>
> **Single source of truth for progress, diagnosis, and next steps: [`STATUS.md`](./STATUS.md) — read it before touching anything.** It records the root-cause analysis of the accuracy problem, what has been fixed, the verified model/licence facts, and the ordered next steps.

## Architecture

```
              ┌─ LIVE ────────────────────────────────────────────────┐
              │ ARKit 91 joints → 1-euro filter                       │
              │                                                       │
              └─ OFFLINE (photo / video import) ──────────────────────┘
                video/photo → Vision person box → 512x512 warp
                  → SAM3DBodyPose.mlmodelc (Core ML, CPU+GPU)
                  → 127 MHR joints → MHRRetarget → BODY-SIZE GATE

                              ↓
                    BodyFrame (20 ARKit joint ids)   ← the ONLY seam
                              ↓
   NimbleEngine ─→ Nimble IK  (app-side Levenberg-Marquardt, NOT nimble's
                 │             refineIK — see NimbleBridge.mm)
                 ├→ Savitzky-Golay, 9-tap CENTRED (dates results 4 samples back)
                 ├→ static-hold gate (offline only: marker speed <= 0.02 m/s)
                 ├→ Nimble ID with GRF decomposition (near-CoP, multi-contact)
                 ├→ Computed moment arms (FK + numerical diff)
                 └→ OSQP muscle optimization
                              ↓
        ┌─────────────────────┴─────────────────────┐
   3-D muscle ANATOMY overlay            posture findings layer
   (RealityKit; fixed 26-capsule         (kinematics only — reads
   set, ONE constant colour, takes       BodyFrame.joints, NOT ik/id/muscle)
   no activation input at all)            + 2-D overlay on the source photo

# COST. The old "~1ms IK / ~0.5ms OSQP" figures were measured on Rajagopal2016
# (81 muscles / 39 DOF). The shipped model is FullBody.osim — 169 coordinates
# (was 171 before the 2026-08-06 patellofemoral weld removed the two
# knee_angle_*_beta) and 520 muscles — and costs ~200ms/frame, hence the
# frame-dropping backpressure added in build 15. See STATUS.md.
```

### Swift ↔ C++ Bridge

ObjC++ wrappers in `BioMotion/Nimble/` and `BioMotion/Muscle/`:
- `NimbleBridge.h/.mm` — loads .osim, runs IK/ID, owns the runtime DOF mask. Registers virtual markers at joint centers for ARKit compatibility. **The IK solve is the app's own Levenberg-Marquardt**, not `Skeleton::fitMarkersToWorldPositions` / `math::refineIK`; the vendored nimble tree is untouched.
- `MuscleSolver.h/.mm` — parses the model's muscles (520 in FullBody.osim, 80 in Rajagopal2016), runs OSQP static optimization
- `MomentArmComputer.h/.mm` — parses muscle paths, computes moment arms via FK + numerical differentiation. Shares the bridge's skeleton (`NimbleBridge+Internal.h`) rather than parsing a second copy.
- Bridging header: `BioMotion/Nimble/BioMotion-Bridging-Header.h`

### Key files

| File | Purpose |
|------|---------|
| `project.yml` | XcodeGen project definition (team, signing, lib paths, build settings) |
| `BioMotion/Resources/FullBody.osim` | **Production** model — 169 coordinates, 520 muscles, full spine + ribcage + upper limb |
| `BioMotion/Resources/Rajagopal2016.osim` | Fallback only (lower extremity: 80 muscles, 39 DOFs, 66 markers). Loaded when FullBody.osim is missing from the bundle. |
| `BioMotion/ARKit/BodyTrackingSession.swift` | ARKit body tracking + 1-euro filter |
| `BioMotion/ARKit/MuscleOverlay.swift` | 3-D muscle **anatomy** capsules — fixed set, one colour, no activation input. Read its type doc before adding anything magnitude-shaped |
| `BioMotion/Nimble/NimbleEngine.swift` | Orchestrates IK → SG → ID → moment arms → muscle on a background queue; owns `staticHoldGating` |
| `BioMotion/Offline/` | The photo/video path: `FrameSource` (decode), `SAM3DPoseEstimator` (Core ML), `MHRRetarget` (127 MHR joints → 20 markers + the body-size gate), `OfflineSessionRunner` (batch + SG edge padding), `OfflineResultStore`, `OfflinePlaybackView` / `PhotoOverlayView` |
| `BioMotion/Findings/` | `PostureFindings` + `PostureFindingsPanel` — kinematics-only posture measurements with view gating. **No clinical thresholds, no verdicts.** |
| `BioMotion/Gait/GaitAnalysis.swift` | Pure frames-in/report-out gait pass. Owns the product's ONE surviving left/right claim: `contactClaimFloorPercent` = `max(timing resolution, contact-duration sampling half-width)`. Never gate a claim on `GaitResolution` alone |
| `BioMotion/Gait/MeanDifferenceUncertainty.swift` | The single Student-t half-width of a difference of two means, plus `StudentT`. Both the contact-time claim and the muscle path call it — a third claim must not reimplement it |
| `BioMotion/CoreML/`, `BioMotion/AssetPack/` | Core ML model loading and the Apple-Hosted Background Asset that delivers it (the 1.3 GiB model is NOT in the app bundle; archived payload is 8 MB) |
| `BioMotion/Recording/TRCExporter.swift` | OpenSim .trc export |
| `BioMotion/App/CalibrationView.swift` | T-pose calibration with live camera (live path only — the offline path scales from one frame's chain sums, see `MHRRetarget.segmentScaleMarkers`) |
| `BioMotion/Muscle/osqp_interrupt_stub.c` | OSQP interrupt handler stub for iOS |
| `nimblephysics/CMakeLists.txt` | iOS-specific CMake (NOT the original — upstream is preserved as `CMakeLists_original.txt`) |
| `tools/osim_fixes/` | The FullBody.osim edit (patella weld + shoulder axis unit-snap), its measurement harness and revert instructions |
| `tools/assetpack/` | Pack build + upload; `dev_bundle_model.sh on\|off` bundles the model locally so the Simulator needs no download |

### Nimble iOS patches

The vendored `nimblephysics/` tree carries iOS-specific patches. Grep for `DART_IOS_BUILD` to find them. Touched areas:

- `config.hpp` — manual config with `HAVE_IPOPT=0`, `DART_IOS_BUILD=1`
- `MeshShape.hpp` / `MeshShape_ios.cpp` — Assimp stubs
- `OpenSimParser.cpp` — guarded MarkerFitter, GUIRecording, SdfParser, MJCFExporter includes
- `MarkerAspect.hpp` / `Marker.hpp` — enum `NO` → `CONSTRAINT_NONE` (ObjC macro conflict)
- `AssimpInputResourceAdaptor.hpp`, `SoftMeshShape.hpp` — Assimp guards
- `C3DLoader.hpp`, `LilypadSolver.hpp`, `Anthropometrics.hpp`, `IKErrorReport.hpp` etc — GUIWebsocketServer guards
- `DARTCollisionDetector_ios.cpp` — stub for collision detector factory
- Vendored: Eigen 3.4.0 (`third_party/eigen`), tinyxml2 (`third_party/tinyxml2`)

## Gotchas

- **Eigen version**: Nimble requires Eigen 3.x. Eigen 5.x (Homebrew default) has breaking API changes. Use vendored `third_party/eigen` (3.4.0).
- **Marker names**: ARKit joints map to virtual markers at body node origins, NOT to the model's surface markers (RASI, LASI etc). See `NimbleBridge.mm` virtual marker registration.
- **C++ exceptions**: Always use C++ `try/catch`, never ObjC `@try/@catch` — ObjC exceptions don't catch `std::exception` or SIGSEGV.
- **Build number**: Must increment `CURRENT_PROJECT_VERSION` in `project.yml` before each TestFlight upload.
- **Library search paths**: Conditional on SDK — `[sdk=iphoneos*]` for device, `[sdk=iphonesimulator*]` for simulator.
- **XcodeGen**: Always run `xcodegen generate` after editing `project.yml` — never edit `BioMotion.xcodeproj/` by hand. A **new test file** needs it too, even when `project.yml` is unchanged, or it sits on disk silently not running.
- **The skeleton is shared process-wide.** `NimbleBridge -sharedSkeleton` hands the same `shared_ptr` to `MomentArmComputer` and the ID path, and it survives across `NimbleBridge` instances. Anything that reads "wherever the skeleton currently sits" is therefore reading process history, not the model — that was a real defect in `applyDOFMaskWithNames:` (fixed 2026-08-07) and it is why the IK cold seed is an explicit `neutralSeedPose`.

## Readings that lie — each has already cost a wrong conclusion

- **`Executed N tests, with 0 failures (0 unexpected)` is not a pass, and this one governs every
  other reading in this file.** A killed test host still prints that line for every suite that
  finished before the kill, and reports the lost tests as neither passed nor failed. Measured:
  two `xcodebuild test` processes sharing one simulator UDID each ended `Executed 2 tests, with 0
  failures (0 unexpected)` / 5 `Restarting after unexpected exit` / `** TEST FAILED **`, on a
  19-test selection that reads `Executed 19 tests` / 0 restarts / `** TEST SUCCEEDED **` when run
  alone on a private device. Naming a simulator (`name=iPhone 17`) instead of a UDID you own is the
  whole mechanism, and it is why three reviewers got three answers on 2026-08-07. Run
  `tools/run_tests.sh` — it takes a private device plus a lock, and gates on all three numbers.
  Corollary for reading a crash report: six suites carry 95% of the wall clock (GaitDynamics 369 s,
  IKConvergence 91 s, ShoulderRotMask 51 s, StaticHold 48 s, MuscleQPUnits 41 s,
  IKSolverInternals 33 s), so "the kill landed just after X" is almost always a statement about the
  schedule, not about X.
- **"A wrong moment arm cancels out of a LEFT/RIGHT ratio" was measured on a rig where it could not
  have failed — and it is FALSE.** This entry said the opposite for two days and cost the product its
  last claim. The rig made every right joint torque `0.8×` its left counterpart, and the shipped QP
  (`min ½aᵀ(εI + λAᵀA)a − λτᵀAa`) is LINEAR in `τ`, so `a_R = 0.8·a_L` exactly **for any moment-arm
  matrix**: the perturbation moved the answer by 1e-6 pp in exact arithmetic, and the **1.04 pp**
  reported as evidence was OSQP's own `eps = 1e-3` tolerance (measured noise floor **1.52 pp**).
  Change one variable — give the right leg a different torque SHAPE (hip 0.80×, knee 1.00×) instead
  of a different size, which is what a gait asymmetry is — and the same bilateral `×0.6` perturbation
  moves a published figure by **9.92 pp** on the shipping solver (13.11 pp in exact arithmetic,
  17.72 pp at a bigger shape difference), turning a real `−17.9 %` into a displayed `−8.0 %`. It
  lands on a muscle whose OWN path is modelled correctly, because the QP redistributes load between
  synergists. And the regime where it does cancel is the regime where **every muscle reads the same
  figure**, i.e. where the per-muscle breakdown carries no per-muscle information. Both halves in
  `MomentArmErrorCancellationTests`. Consequences: the per-muscle left/right claim is retired
  (`GaitLoadSummary.perMuscleLeftRightClaimIsSupported = false`), the 3-D muscle overlay is off on
  analysed running clips, and the surviving left/right finding on that screen is CONTACT TIME, which
  touches neither a moment arm nor the QP. A one-sided error still costs **23.8 pp**; a sign-flipped
  one still pins both sides to `aMin` and reads exactly 0.0 % against a true 22.7 %.
- **A PICTURE makes the ranking claim more loudly than a list, and it outlived the round that killed
  the list.** `MuscleOverlay` filtered `rawActivations`, kept the strongest 24 and coloured every
  capsule from one shared blue→red ramp with alpha rising 0.45 → 0.95 — so BOTH which muscles
  appeared and how they looked were ordered by a number whose per-muscle scale is unknown (`1/k` per
  muscle, `k` pose-dependent). It shipped on the LIVE screen, where `MuscleActivationBar` printed
  the same twelve muscles' activations as bars and per cent underneath it, and on the offline 3-D
  view beside the paragraph refusing that exact comparison. A picture has no number to check, no
  floor and no caption, so it is the MORE authoritative surface, not the lesser one. Retiring a claim
  from one view is not retiring the claim: grep for every consumer of the same numbers. Since
  2026-08-08 `update(joints:)` takes no muscle solve at all, the capsules are a fixed 26-muscle set
  in one constant colour, the bar chart is deleted, and `MuscleOverlay.anatomyOnlyNote` states the
  absence on both screens (`MuscleOverlayClaimTests`).
- **A list sorted BY a statistic is not a sample of that statistic, and the top of it is an order
  statistic.** `GaitLoadSummary.make` builds a comparison for **all 175** bilateral pairs in
  `FullBody.osim`, `ordered(_:)` sorts by `|difference| / claimFloor`, and the panel drew the top 8
  under "Each comparison is a 95 % one and 8 are shown, so about one in twenty…". A calibrated
  t-interval admits α of the family BY CONSTRUCTION, so on a symmetric runner the per-comparison rule
  publishes **4-5 false findings per clip** at the scatter this repo measures everything else at —
  and they sort to positions 1-5. `samplingUncertaintyPercent` now takes its Student-t at `α/N` for
  `N = screenedComparisonCount`; the multiplier goes 2.776 → 11.899 at df=4 (×4.3), and the measured
  survivor count is **0 on every pinned clip at every scatter level** (`GaitClaimSurvivalTests`).
- **`isSaturated` is not "the QP is in its linear regime" — the box has TWO bounds.** The
  cancellation above holds only in the interior. A muscle whose modelled path has the wrong sign is
  not rescaled: the QP refuses to recruit it and pins it to `a ≥ aMin = 0.02` on BOTH sides, where it
  reads **exactly 0.0 %** left/right against a true 22.7 % — a finding destroyed and presented as
  "even". This cost a wrong test assumption on 2026-08-08. Read `isAtActivationFloor` beside
  `isSaturated`, and derive both thresholds from `MuscleSolver.saturationActivationTolerance`
  (0.02, i.e. `10·(eps_abs + eps_rel)` because `OSQP_SOLVED_INACCURATE` is accepted) — a 0.999 test
  for the upper bound missed every clipped muscle, since a clipped activation returns as low as 0.98.
- **A FLOOR BUILT FROM TIMING DOES NOT CONTAIN A DIFFERENCE OF TWO MEANS — and this cost the
  product's LAST claim its gate, one round after the same defect cost it the muscle claim.**
  `asymmetryClaim` published whenever `|contactAsymmetryPercent| >=
  resolution.resolvableAsymmetryPercent`, i.e. `max(50/framesPerContact, max(stride-period CV,
  100/stridePeriodFrames))`. The statistic is the difference of two MEANS OF ~5 CONTACT DURATIONS.
  Contact-duration scatter was measured all along as `contactVariationPercent` and had **zero
  consumers**. Measured, 20 000 seeded trials, symmetric runner at `video_015`'s own configuration
  (5 contacts a side, 11.144 % contact scatter, 8.086 % timing floor): the shipped gate printed
  "Contact time is 9 % longer on the left", unhedged and in orange, on **25.3 % of clips**. With
  `GaitReport.contactSamplingUncertaintyPercent` in the floor it publishes on **2.4 %** against a
  5 % nominal (Welch df would give 4.0 %; `min(n)−1` is the conservative choice, pinned). The
  in-code justification — "contact-duration scatter is mostly detector edge jitter, which the
  quantisation floor already counts" — was false by this repo's own arithmetic: two ±½-frame edges
  give `√(2/12)/6.1833 = 6.60 %` of CV against 11.144 % measured, so **65 % of the variance is not
  edge jitter**. Consequences: `video_015`'s floor doubles to 16.5 %, no pinned clip publishes a
  contact-time claim (none did before either), and the claim's honest sensitivity is a 20-25 %
  left/right difference on a 4 s clip. Read `contactClaimFloorPercent`, never
  `resolution.resolvableAsymmetryPercent`, and use `MeanDifferenceUncertainty.halfWidthPercent` —
  one estimator, so a third claim cannot get this wrong in a different file
  (`GaitContactClaimTests`).
- **A COUNT of muscles at a solver bound is not a fact about a body, and its denominator excluded
  its own numerators.** The honesty block printed "N muscle(s) reached full effort and M sat on the
  resting-tone floor, out of S pairs the solver kept between the two". `screenedComparisonCount` is
  built with `guard !saturatedBases.contains(base), !flooredBases.contains(base)` — disjoint from
  both by construction, so "140 out of 30" is what it produced (constructed: floored 3, screened 1,
  `ClaimSurfaceTests`). And the trailing "at either bound the answer is the bound" disclaimed the
  ACTIVATION, not the COUNT, which was the part rendered as a number about the reader. The sentence
  states the mechanism now and prints no count. Same list, entry below: the counts read 19, 11, 22,
  18, 20 across a λ sweep at fixed inputs.
- **A GATE THAT CANNOT DELIVER IS NOT A LEVER.** `GaitReportPanel.loadBlock` was an `if/else` on
  `withheldReason`, so a clip failing the DATA gate saw only "…film a steadier, straighter run"
  under a header reading "Muscle by muscle: not shown, and why" — while
  `perMuscleLeftRightClaimIsSupported = false` means no clip, however clean, produces a muscle row.
  Every lever in `withheldReason` was written when passing the gate produced eight rows. The
  permanent reason prints on every branch now, and `muscleRowsUnaffectedByRefilmingSentence` scopes
  the lever and disappears on its own if a per-muscle claim ever comes back.
- **An UNBIASED statistic is not a CERTAIN one, and the gate only ever saw the bias.** The
  2026-08-08 repair moved the load statistic's mean asymmetry on a symmetric runner from +8.07 % to
  −0.19 %, and the publication gate (`resolvableAsymmetryPercent`) is built from frames per contact
  and the stride period — timing only. The per-clip STANDARD DEVIATION is **9.47 %** against
  `video_012`'s 10.145 % floor, so roughly one displayed muscle in four still read a false finding
  with every clip-level gate green. Any claim must clear `MuscleLoad.samplingUncertaintyPercent`
  (Student-t, not 1.96 — a clip has 4-6 contacts a side) as well as the timing floor.
- **"A contact is a run of consecutive stance frames" is false downstream of the solver.** A
  non-converged IK, a `submitAndWait` timeout or an unrouted solve leaves NO gap in the frame
  numbering and raises no `.droppedSamplesInContact`, because the body frame was fine — so
  adjacency-based grouping split one foot-strike into two and double-weighted the pair. Measured
  **9.09 %** of fabricated left-side asymmetry from one missing frame in a seven-frame contact.
  Contact identity is `GaitFrameOutcome.contactIndex`, carried from `GaitReport.stance`.
- **`GaitOutcome.isAboutRunning` is true for `.notRunning`.** It is true for everything except
  `.notAttempted`, so a refusal whose entire meaning is "this is not running" counted as "this is a
  gait screen" and took the posture findings off the screen with it. Ask
  `replacesPostureFindings`, which is true only for `.analysed`.
- **`NimbleIDResult.jointTorques.head<6>()` is a hard-coded zero.** `Skeleton.cpp:10365` does `setZero()` unconditionally and its assert is compiled out of the Release static libs. `rootResidualNorm` is now a real linear-momentum residual in **newtons** — a frame-consistency check, never a balance check.
- **`NimbleIKResult.error` is nimble's LOSS** (`Σ wᵢ²‖Δpᵢ‖²`, in m²), not an RMS and not in metres. Read `markerRMSMeters` for accuracy. `NimbleEngine.IKOutput` exposes both as `ikLossSquaredMeters` and `markerRMSMeters` for the same reason.
- **"Torque decreases distally" is not a law.** In single-leg stance the free leg decreases distally while the loaded leg increases toward the contact. Both are correct.
- **Saturated-muscle count is not a metric.** Across a λ sweep at fixed inputs it reads 19, 11, 22, 18, 20 with no trend — it measures where OSQP stopped.
- **`leftFootLoadFraction`/`rightFootLoadFraction` are a SUM you may read and a SPLIT you may not,
  and the live screen printed the split for the life of the project.** `AccuracyBadge(label:
  "L/R load", value: "0.62|0.38")` had no caption, no floor and nothing validating it, on the app's
  most-used surface, in the exact framing the offline path spent four rounds retiring — and its
  green indicator was `abs(total − 1.0) < 0.3`, keyed to the sum alone. The split is not merely
  unchecked: `NimbleBridge.mm:1499` seeds the solve with a hardcoded 50/50 wrench guess whenever
  both feet are down, and double support is statically indeterminate at ±18 pp with a perfectly
  known CoM against a ~10 pp meaningful threshold. The badge reads `GRF sum … BW` now, with
  `NimbleEngine.footLoadSplitIsNotMeasuredNote` under it on the SAME `if` (the live path had
  already shipped one picture whose caption had a different gate — that one is still open, minor 8).
- **The gait residual is blind to WHICH foot carries the load.** `GaitFrameOutcome.residualInBodyWeights` is built from `leftFootForce.y + rightFootForce.y`, and the near-CoP solver's constraint fixes that SUM exactly — so a 50/50 split between the feet and a 100/0 split give the identical residual while halving one leg's torques. Only `contactDetectorsAgree` can see the split. A residual that passed says nothing about the left/right claim.
- **That same residual is VERTICAL ONLY, and it is not `‖a_artic‖/g`.** `leftFootForce.x/.z` exist in the bridge's output (`NimbleBridge.h`: `[fx, fy, fz] N`) and are discarded. The fore-aft braking/push-off term STATUS sizes at 0.2-0.35 BW is 10-17× the measured vertical residual, is phase-dependent, and does NOT cancel out of a muscle-to-muscle ratio — and no check in this pipeline examines it. Read `GaitLoadSummary.maxVerticalForceResidualInBodyWeights`, whose name says so.
- **A "peak" over a side's frames is not a left/right statistic.** The two legs contribute different numbers of usable frames (a contact one sample longer yields twice as many at `taps = 5`), and `E[max of n]` grows with `n` — measured at **+8.07 %** of fabricated left-high asymmetry on a symmetric runner, 80 % of `video_012`'s own 10.14 % publication floor. Any statistic over per-side frames must have an expectation independent of the count; `GaitLoadSummary.MuscleLoad` uses one sample per contact, averaged over contacts.
- **A gate that measured nothing is not a gate that passed.** `sortedResiduals.last ?? 0` reported max 0, median 0 and "passed" for a clip where no stance frame was ever usable. Check `residualFrameCount`/`residualWasMeasured` before reading any aggregate here.
- **`strideRepeatabilityPercent` can read exactly 0.000 and mean nothing.** It is a CV over touchdown gaps quantised to whole frames; `video_012`'s are all exactly 18 samples. The clip cannot distinguish anything below `GaitSteadiness.boundPercent = 100/stridePeriodFrames`, so the published figure is floored there and `measuredStrideRepeatabilityPercent` holds the raw CV.
- **A visible skeleton says nothing about the solver.** The skeleton is drawn straight from `BodyFrame.joints`; muscle output needs the whole IK → SG → ID → moment-arm → QP chain. They share no stage.
- **A well-drawn torso says nothing about the legs.** A monocular pose model that cannot see a limb does not fail loudly — it returns its mean pose for that limb, which looks like a plausible standing leg. `VNDetectHumanRectanglesRequest.upperBodyOnly` defaulting to `true` hid behind this for a day (fixed 2026-08-07, `PersonBoxTests`). Score limbs separately against an independent estimator; an aggregate that mixes torso and legs dilutes a 3× leg error into noise.
