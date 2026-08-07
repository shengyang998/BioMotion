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
   3D muscle overlay (RealityKit,        posture findings layer
   strongest 24 by rank, never           (kinematics only — reads
   by a fixed threshold)                 BodyFrame.joints, NOT ik/id/muscle)
                                          + 2-D overlay on the source photo

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
| `BioMotion/ARKit/MuscleOverlay.swift` | 3D muscle capsule visualization (strongest 24 by rank) |
| `BioMotion/Nimble/NimbleEngine.swift` | Orchestrates IK → SG → ID → moment arms → muscle on a background queue; owns `staticHoldGating` |
| `BioMotion/Offline/` | The photo/video path: `FrameSource` (decode), `SAM3DPoseEstimator` (Core ML), `MHRRetarget` (127 MHR joints → 20 markers + the body-size gate), `OfflineSessionRunner` (batch + SG edge padding), `OfflineResultStore`, `OfflinePlaybackView` / `PhotoOverlayView` |
| `BioMotion/Findings/` | `PostureFindings` + `PostureFindingsPanel` — kinematics-only posture measurements with view gating. **No clinical thresholds, no verdicts.** |
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

- **`NimbleIDResult.jointTorques.head<6>()` is a hard-coded zero.** `Skeleton.cpp:10365` does `setZero()` unconditionally and its assert is compiled out of the Release static libs. `rootResidualNorm` is now a real linear-momentum residual in **newtons** — a frame-consistency check, never a balance check.
- **`NimbleIKResult.error` is nimble's LOSS** (`Σ wᵢ²‖Δpᵢ‖²`, in m²), not an RMS and not in metres. Read `markerRMSMeters` for accuracy. `NimbleEngine.IKOutput` exposes both as `ikLossSquaredMeters` and `markerRMSMeters` for the same reason.
- **"Torque decreases distally" is not a law.** In single-leg stance the free leg decreases distally while the loaded leg increases toward the contact. Both are correct.
- **Saturated-muscle count is not a metric.** Across a λ sweep at fixed inputs it reads 19, 11, 22, 18, 20 with no trend — it measures where OSQP stopped.
- **The gait residual is blind to WHICH foot carries the load.** `GaitFrameOutcome.residualInBodyWeights` is built from `leftFootForce.y + rightFootForce.y`, and the near-CoP solver's constraint fixes that SUM exactly — so a 50/50 split between the feet and a 100/0 split give the identical residual while halving one leg's torques. Only `contactDetectorsAgree` can see the split. A residual that passed says nothing about the left/right claim.
- **That same residual is VERTICAL ONLY, and it is not `‖a_artic‖/g`.** `leftFootForce.x/.z` exist in the bridge's output (`NimbleBridge.h`: `[fx, fy, fz] N`) and are discarded. The fore-aft braking/push-off term STATUS sizes at 0.2-0.35 BW is 10-17× the measured vertical residual, is phase-dependent, and does NOT cancel out of a muscle-to-muscle ratio — and no check in this pipeline examines it. Read `GaitLoadSummary.maxVerticalForceResidualInBodyWeights`, whose name says so.
- **A "peak" over a side's frames is not a left/right statistic.** The two legs contribute different numbers of usable frames (a contact one sample longer yields twice as many at `taps = 5`), and `E[max of n]` grows with `n` — measured at **+8.07 %** of fabricated left-high asymmetry on a symmetric runner, 80 % of `video_012`'s own 10.14 % publication floor. Any statistic over per-side frames must have an expectation independent of the count; `GaitLoadSummary.MuscleLoad` uses one sample per contact, averaged over contacts.
- **A gate that measured nothing is not a gate that passed.** `sortedResiduals.last ?? 0` reported max 0, median 0 and "passed" for a clip where no stance frame was ever usable. Check `residualFrameCount`/`residualWasMeasured` before reading any aggregate here.
- **`strideRepeatabilityPercent` can read exactly 0.000 and mean nothing.** It is a CV over touchdown gaps quantised to whole frames; `video_012`'s are all exactly 18 samples. The clip cannot distinguish anything below `GaitSteadiness.boundPercent = 100/stridePeriodFrames`, so the published figure is floored there and `measuredStrideRepeatabilityPercent` holds the raw CV.
- **A visible skeleton says nothing about the solver.** The skeleton is drawn straight from `BodyFrame.joints`; muscle output needs the whole IK → SG → ID → moment-arm → QP chain. They share no stage.
- **A well-drawn torso says nothing about the legs.** A monocular pose model that cannot see a limb does not fail loudly — it returns its mean pose for that limb, which looks like a plausible standing leg. `VNDetectHumanRectanglesRequest.upperBodyOnly` defaulting to `true` hid behind this for a day (fixed 2026-08-07, `PersonBoxTests`). Score limbs separately against an independent estimator; an aggregate that mixes torso and legs dilutes a 3× leg error into noise.
