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
- `MomentArmComputer.h/.mm` — parses muscle paths, computes moment arms via FK + numerical differentiation. Shares the bridge's skeleton (`NimbleBridge+Internal.h`) rather than parsing a second copy. Applies path WRAPPING, and picks a one-sided difference where the wrap state changes inside the stencil. FullBody's four `MovingPathPoint`s all survive; their `SimmSpline`s use Nimble's OpenSim-compatible evaluator (`parsed 4 / approximated 0`).
- `MusclePathWrap.h/.cpp` — the cylinder AND ellipsoid wrap solvers, ported from opensim-core (Apache 2.0; licence header in the file, attribution in `./NOTICE`). Length only: no `wrap_pts` leave the solver, and only the ellipsoid's `hybrid` `<method>` is implemented — the other two are refused, not approximated. All 76 `PathWrap` references in `FullBody.osim` and all 46 in `Rajagopal2016.osim` are solved. Every intentional difference is listed under DEVIATIONS at the top of the .cpp (12 of them).
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
| `tools/opensim_ref/` | The OpenSim 4.6 reference generators (`uv` venv, PyPI `opensim` wheel), all read-only against the shipped `.osim`. `dump_reference.py` → CSV, `analyse.py --write-fixture` → `BioMotionTests/Fixtures/opensim_moment_arms.txt`; `dump_finite_difference.py` → `opensim_moment_arms_fd.txt`, OpenSim's own central difference of its own length (the column a `-dL/dq` implementation is comparable with); `fd_check.py` → analytic vs central for one pose/muscle; `inspect_wrap.py` → the wrapped path point by point, with the solver's raw inputs; `pose_coverage.py` → what the pose grid covers |
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
- **Nimble source is not the linked artefact.** The app links
  `nimblephysics/build_ios/libnimble_ios.a` and `build_sim/libnimble_ios.a`, not
  the stale XCFramework. After changing vendored C++, rebuild BOTH archives and
  add a reviewed patch under `nimble-patches/`; a source-only fix can otherwise
  look correct in `git diff` while every test still runs the old object code.
- **XcodeGen**: Always run `xcodegen generate` after editing `project.yml` — never edit `BioMotion.xcodeproj/` by hand. A **new test file** needs it too, even when `project.yml` is unchanged, or it sits on disk silently not running.
- **Native-rate sampling is span-bounded and frame-bounded.** It targets up to 4 s, but the
  601-frame cap covers about 2.5 s at 240 fps; it is not the same 120-call budget as sparse mode.
  Keep selector copy in `FrameSource.nativeWindowDisclosure` and truncation causes in
  `FrameBudgetNotice`, where `OfflineDisclosureTests` pins the exact-window boundary.
- **Analysis FPS has one source:** `GaitReport.framesPerSecond`, derived from the median interval of
  surviving frame timestamps. AVAsset's nominal rate is valid for native decoding and frame-budget
  notices only. Do not add it to `GaitOutcome.analysed` or `GaitLoadSummary.make`; sparse sampling
  is precisely where nominal rate and analysed cadence differ.
- **An analysed report does not depend on a muscle summary.** Resolution, contact-time findings and
  report flags come from `GaitReport` through `GaitTimingSummary` and stay visible when
  `GaitLoadSummary.make` returns nil. Only muscle/load/honesty sections may follow that optional.
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
- **The reference's MOMENT ARM is not the derivative of the reference's own LENGTH, and gating a
  finite-difference implementation on it manufactures four sign flips.**
  `GeometryPath::computeMomentArm` asks `MomentArmSolver` for the generalized force a unit tension
  along the CURRENT path produces with the wrap points held fixed on their bodies — the envelope
  theorem, exact where the path varies smoothly with q and NOT where the wrap solution is marginal.
  OpenSim differencing its OWN length at the same 1e-4 rad: `TR2_l`/`L2_L3_FE` at `spine_flexed`
  reads analytic **+0.002252** against central **−0.005274**; `gasmed_r`/`knee_angle_r` at `neutral`
  reads **+0.021761** against **+0.004891** (and at `squat_deep` the two agree to 1e-6). Every
  "sign flip" the first cylinder-wrapping run reported was that gap, not a backwards `quadrant`.
  The definition-matched column is `BioMotionTests/Fixtures/opensim_moment_arms_fd.txt`
  (`tools/opensim_ref/dump_finite_difference.py`, 7 s). ⚠️ **The `gasmed_r` half of that example was
  mis-attributed and it is the opposite way round.** Measured 2026-08-09: the analytic +21.76 mm is
  the good column there and the central −dL/dq is the broken one, because for a MULTI-wrap muscle the
  length it differentiates is not a path length (entry below). Over all 8 multi-wrap muscles' full
  clamped ranges the port sits **1.05 mm** worst case from OpenSim's analytic column and up to
  **41.26 mm** from a central difference of OpenSim's own reported length. "Definition-matched" is
  the right rule for the single-wrap class; for the multi-wrap class the two columns differ because
  one of them is wrong, and it is not the analytic one.
- **OpenSim's MULTI-WRAP PATH LENGTH IS NOT THE LENGTH OF A PATH, and it is a BIAS across the whole
  running range rather than a tail case.** This entry used to say the reported tangent points "belong
  to a later `C2→P2` solve"; measured on 2026-08-09, that mechanism is wrong and the size was
  understated. `calcLengthAfterPathComputation` adds straight segments measured between the wrap
  points OpenSim REPORTS to the spiral length OpenSim STORED beside them, and for a two-cylinder path
  those halves describe different paths. On `gasmed_r` at knee 0°: stored spiral **0.038054 m**
  against a **chord** between its own two tangent points of **0.045350 m** — shorter than the
  straight line between the points it is supposed to connect, i.e. impossible for any curve, so the
  total is ≥ 7.30 mm below the length of any path through its own points (88 such rows in
  `opensim_multiwrap.txt`, worst −7.2957 mm). The cause is `WrapCylinder::_adjust_tangent_point`,
  which runs ONLY when a muscle carries more than one `PathWrap`: OpenSim moves the tangent points
  and nothing recomputes the arc — its stored arc still equals the PRE-adjustment one, which is why
  it matches this port's arc to 31 µm while its POINTS sit 8.7 mm away. Consequences, all measured:
  the `gasmed`/`gaslat140` "systematic 10-11 mm moment-arm error" is **100 %** this
  (`gasmed_r` median 10.484 mm against the reported column, **0.033 mm** against the same column
  reconciled with its own tangent points; length 4.203 → **0.0041 mm**); and the port's own path is
  self-consistent to the last stored digit at 451/451 poses, verified with a replica of the outer
  loop that reproduces `solveWrappedPathLength` bit-for-bit. `TRIlong`/`BIClong` never engage two
  CYLINDER spirals at once and agree with the reported column to **0.0000 mm**, which is the control.
  **The third witness owes this repo nothing**: `GeometryPath::computeMomentArm` reads the reported
  wrap POINTS and never calls `calcLengthAfterPathComputation`, and over all 8 multi-wrap muscles'
  full clamped ranges it agrees with the port to a median of **0.0005 mm** and a max of **1.05 mm**
  — including the 26.019° row where the length-derived column is out by 41.26 mm. Two OpenSim
  columns, one port, and the port sits with the one that is blind to the defect.
  Gate for the class: `MultiWrapReferenceTests`. Never gate a two-cylinder muscle on `getLength`.
- **A UNIFORM GRID CANNOT FIND A REFERENCE'S JUMPS, so "the falsifier passes" can be a statement
  about the grid.** `CylinderWrapValidationTests`' W1 bounds |ours − OpenSim's central difference| at
  20 mm and the multi-wrap class reads 15.8 mm on the committed 5° grid. The excursions are SPIKES
  at the reference's own `L(q)` steps, and a stencil of half-width `eps = 1e-4 rad` only sees one
  when it straddles it — so refining the grid does not reliably find them either (0.25° → 17.4 mm,
  0.025° → 17.6 mm). Scan for the jumps instead: sample the reference's length at a step finer than
  `2·eps = 0.0115°` and take the largest second differences. That rule, stated before any result,
  lands on `gaslat140_r` at knee **26.01866°**, where the same quantity reads **41.26 mm** — the
  reference's `L` steps **6.2 µm over 0.0005° with no change in wrap-point count** (ours steps
  0.2 µm), flipping its own central difference to **−18.23 mm** against our **+23.07 mm**. Against
  the reconciled column at that same pose we are **0.54 mm** out. A max-based assertion against a
  jittery column cannot tell "the port is wrong" from "the reference jumped", at any threshold.
- **`dL/dq` IS DISCONTINUOUS where a muscle starts or stops wrapping, and the centred difference
  across it reads in METRES.** Not at the tangency boundary — L is continuous there — but at the
  cylinder-END rule: the surface is a finite segment, so when both tangent points slide past
  `length/2` the wrap stops being applied from a length that is nowhere near the straight line.
  Measured on constructed geometry: **L steps 36.1 mm**, a centred difference at 1e-4 returns
  **−180.7 m** per unit, the one-sided difference on the engaged branch reads 0.000. Driven
  through the shipped chain at a real switch (`grac_r`/`knee_angle_r`), the raw centred difference
  is **−19.62 m** against a true **−0.0337 m**. `MomentArmComputer` compares
  `WrappedPathResult.signature` — the wrap solver's DISCRETE state — at `q`, `q±eps` and drops to
  a one-sided difference; `lastOneSidedDifferenceSamples` is how anybody downstream can tell. It
  fires **0 times in 5,272,800 samples** at the 60 validation poses, which is why the test that
  proves it has to CONSTRUCT the switch by bisection rather than wait for one. The ellipsoid adds a
  SMALL sibling to the same hazard: its surface distance is a sum of `(int)(|r1−r2| / 1 mm)`
  chords, so L steps by **2.6e-6 m** every time that integer ticks — 13 mm/rad once divided by
  `2·eps`, large enough to contaminate a finite-difference moment arm. The chord count is therefore
  part of the signature too.
- **A NUMERICAL ROUTINE CAN BE VALID ONLY AT ONE SCALE, and OpenSim's point-to-ellipsoid solver
  is.** `WrapEllipsoid::findClosestPoint` (Graphics Gems IV) stops its Newton iteration on
  `|f| < 1e-9`, where `f` is a DEGREE-6 polynomial in the radii. Called with the model's real
  radii — `TRIlonghh_*` is `0.035 0.02 0.02` m — `f` is already ~1e-19 at the first iterate, so it
  returns `t = 0`, i.e. **the point it was asked about, unchanged**. Measured: probe
  `(0.004, 0.003, 0.002)` against radii `(0.035, 0.02, 0.02)` returns `(0.004, 0.003, 0.002)`;
  the true closest surface point is 16.7 mm away. That is what `factor = 3/(a+b+c)` is for, and it
  is why `wrapEllipsoidLine` scales EVERYTHING — points, radii, the returned length — before doing
  any geometry. Normalised, the same routine agrees with an exhaustive search over the surface to
  1.5e-3. Both halves are pinned in `MusclePathWrapTests` so nobody "fixes" the normalisation away.
- **`WrapEllipsoid` SEEDS ITSELF FROM THE PREVIOUS CALL, and only `hybrid` overwrites all of it.**
  `wrapLine` copies `r1`, `r2`, `c1` and `sv` out of `aPathWrap.getPreviousWrap()` before it starts.
  On the `hybrid` branch every one of the four is overwritten before it is read (`r1`/`r2` by the
  line/ellipsoid intersection and then by `c1`; `c1`/`sv` by Frans, by the fan, or by the blend), so
  hybrid is a pure function of `q` and differentiating it is legitimate. On `axial`,
  `use_c1_to_find_tangent_pts` can be false, which leaves the PREVIOUS call's tangent points as the
  seed for the iteration — a function of call history, not of `q`. All 12 references in
  `FullBody.osim` say `hybrid`; `MomentArmComputer` parses `<method>` and counts anything else as
  UNMODELLED rather than solving it as hybrid.
- **`EQUAL_WITHIN_ERROR(x, -Infinity)` IS ALWAYS FALSE, and OpenSim uses it as a sentinel test.**
  It expands to `fabs(-inf − −inf) <= 2e-13`, i.e. `NaN <= 2e-13`. So the `fanWeight` sentinel
  branch in `WrapEllipsoid::wrapLine` never fires and the quadrant-flip bisection is skipped
  whenever the fan did not run. This port reproduces that verbatim (DEVIATION 12) — repairing it
  would fork the answer away from the reference every gate here is measured against.
- **The straight-line moment arm was not "a bit off" — 9.00 % of the pairs on wrapped muscles
  pointed the WRONG WAY, and the worst was 32× too small with the sign reversed. Cylinder and then
  ellipsoid wrapping shipped on 2026-08-08 and this entry is now the BEFORE.** After the cylinder:
  median 0.048 mm, max 8.07 mm, 4 sign flips (all of them the definition gap above), and against
  OpenSim's own derivative the single-wrap muscles read max 3.569 mm. After the ellipsoid,
  `unmodelledPathWraps` counts **0**, not 12 and not 76, and
  `GaitLoadSummary.musclesWithUnmodelledPaths` is EMPTY — which does not mean the paths are exact,
  it means every wrap OBJECT is solved. Non-wrap fidelity is reported separately: FullBody now
  reports four MovingPathPoints parsed and zero approximated, using the canonical Nimble
  SimmSpline. The 4.414 mm BIC result below is the pre-exact-MovingPath snapshot; the current
  central-difference maximum is 2.679 mm. The elbow muscles' own before/after is an ABLATION rather
  than an argument: `BRD_r`/`elbow_flex_r` reads **−8.73 mm** with the ellipsoids off against a true
  **+1.51 mm**, and **+1.51 mm** with them on. The numbers below describe the code as it
  was, and they are what the port has to keep beating. This entry used to be a
  count of unmodelled `PathWrap`s and an argument; since 2026-08-08 it is a measurement against
  OpenSim 4.6 reading the same `FullBody.osim` (`BioMotionTests/Fixtures/opensim_moment_arms.txt`,
  173 poses × 104 muscles, generated by `tools/opensim_ref/`). On the 66 muscles that carry a
  PathWrap: median relative error **13.7 %**, p90 124.4 %, max 694.7 %, and **3,769 of 41,866 pairs
  sign-flipped**. Path length itself is out by up to **51.8 % of the muscle's own length**. The
  control that makes this an attribution rather than a correlation: the 454 muscles with NO wrap
  object are identical in the wrapped and unwrapped models to the last stored digit, at every pose —
  **exactly 0.0**. And the shipped `MomentArmComputer` reproduces OpenSim-with-wrapping-disabled to
  **4.39 mm** worst case over 12,384 samples while the gap to the truth is **146.6 mm**, so 97 % of
  the error is the missing solver, not the other implementation differences. Worst named pairs:
  `gasmed_l`/`knee_angle_l` 112.9 %, `gaslat140_l`/`knee_angle_l` 94.5 %, `psoas_r`/`hip_rotation_r`
  87.8 %. Do not reason about this from `unmodelledPathWraps` any more; read the fixture.
- **Three ways FullBody.osim's coordinates lie to you, all caught in one afternoon.** (a)
  `shoulder_elv_l` runs **−115..0 deg** while `shoulder_elv_r` runs **0..115** — mirroring an arm
  pose by copying the value is 25 deg out of range, and `Coordinate::setValue(state, v, false)`
  accepts it **silently**: OpenSim does not clamp. (b) The ranges are ROUNDED DECIMALS —
  `shoulder_elv_r` maxes at 2.0071 rad = 114.99836 deg, `hip_flexion_r` at 119.99999986 — so a sweep
  written as "0 to 115" is genuinely outside the model. (c) `GeometryPath::computeMomentArm` returns
  **exactly 0.0** for a LOCKED coordinate, which is a refusal and not a measurement; 54 of the 169
  are locked, nimble does not honour `<locked>` at all, and differencing the two would manufacture a
  100 % error and blame it on wrapping. Separately, `osim.Model(filename)` **chdirs into the model's
  directory**, so OpenSim's default logger writes `opensim.log` into `BioMotion/Resources/` — a
  `type: folder` resource, i.e. straight into the shipped bundle. `tools/opensim_ref/osim_model.py`
  calls `osim.Logger.removeFileSink()` at import.
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
  **Re-measured 2026-08-09 with the wraps solved, and the 9.92 pp is now a number about a defect that
  no longer exists.** `×0.6` was a stand-in for the straight-line path; the MEASURED p99 residual on
  the muscles the product names, at reference arms ≥20 mm, is **1.114 %**, and the same rig at that
  perturbation reads **1.4022 pp**. On real geometry (`WrappedMomentArmLeakTests`, 40 right-leg
  muscles mirrored into a bilateral rig, 582 readable cells) the median moment-arm leak is
  **0.977 pp** against the straight line's **7.939 pp**. The claim still does not come back, and as
  of the 2026-08-09 re-run the binding term is neither the solver (fixed, 4.5e-05 pp) nor the typical
  moment arm: it is a TAIL on muscles whose own paths are exact, against a reference that disagrees
  with itself by more than the whole gate budget. Read the two entries below on R1 and on the sharing
  step, not the solver entry — that one is now history.
- **A SOLVER'S RELATIVE TOLERANCE IS SCALED BY ITS DATA AND ITS ERROR IS DIVIDED BY ITS CURVATURE,
  and where those two differ by nine decades the stopping rule constrains nothing.** This entry said
  "an OSQP tolerance is ABSOLUTE" for one day and named `saturationActivationTolerance = 0.02` as
  the mechanism. That was the PRIMAL tolerance, and the primal residual was never the problem —
  measured over 2,791 solves, median 3.8e-05 against a 2e-03 allowance. The DUAL check is
  `eps_abs + eps_rel·max(‖q‖∞, ‖Px‖∞, ‖Aᵀy‖∞)` (`osqp/src/auxil.c:compute_dual_tol`), and with
  `q = −λAᵀτ`, `λ = 100`, arms in metres and forces in newtons, `‖q‖∞` MEASURED at a median of
  **9.8e06** — so a stationarity violation of ~8e03 passes as `OSQP_SOLVED`, and 788 of it was
  delivered. What turns that into an activation error is the CURVATURE that resists it, and
  `εI + λAᵀA` has one row of `A` per coordinate: **68 of the rig's 80 eigenvalues are exactly
  ε = 0.01**, condition number 1.22e09. `788/1.22e07 = 6e-05` along the stiff directions and
  `788/0.01 = 8e04` along the flat 68 — which are precisely "which synergist carries the load", i.e.
  the entire content of a per-muscle claim. **Tightening the tolerance does not fix it**: `eps = 1e-9`
  with 20,000 iterations still lands 0.3025 from the exact answer, and `eps_rel = 0, eps_abs = 1e-6`
  needs 200,000 iterations and 463 ms to reach 1.1e-03. What fixes it is `scaling = 0` — Ruiz
  equilibration rewrites `A = I` into a diagonal the ADMM step no longer matches and the flat
  subspace stops contracting — plus `polishing = 1`. Measured on the leak rig: median **14.88 pp →
  4.4994e-05 pp**, p90 37.83 → 0.047, torque residual 2.80e-03 → 8.32e-09. On the real 520-muscle
  problem: **0.10269 → 1.017e-04** over interior muscles (355.8 pp → 0.352 pp of a left/right
  figure), at **+3 %** wall. The 21.98 pp max that survives on the rig is 466× its own p90 and is
  unattributed. Two corollaries: `iter` sitting at the cap is worth checking (every 520-muscle solve
  stopped at exactly 200 and was accepted as `SOLVED_INACCURATE`), and a small rig cannot certify a
  large solve — `MomentArmErrorCancellationTests`' three-muscle rig read 1.52 pp for this quantity
  and was right, because OSQP solves a three-muscle problem accurately.
- **THE τ-RESIDUAL AT THE MINIMISER IS NON-INCREASING IN λ, so a residual that ROSE proves the solve
  did not solve — and a sweep that admits such a point compares a failure with a measurement.**
  `MuscleQPUnitsTests.testResidualMechanismSweep` claims "eight decades of τ-match weight buy
  nothing". Two of those decades were never measurements. Minimising `½ε‖a‖² + ½λ‖Aa − τ‖²` over a
  convex set, optimality of `a₁` at `λ₁` and of `a₂` at `λ₂` add to `½(λ₂ − λ₁)(r₂ − r₁) ≤ 0`, i.e.
  **r₂ ≤ r₁**; the argument uses only convexity of the feasible set, so the BOX does not weaken it.
  Measured 2026-08-09: `upright` reads 0.2420 / 0.2373 / 0.2336 through λ = 1e4 and then **0.5833**
  at 1e6 — a **149.6 % RISE** — and 1.4760 at 1e8; `dancer` 0.3392 / 0.3407 / 0.3390 and then
  **0.9535** at both (181.3 %), all 520 muscles on the floor. The old test passed by comparing two
  points that had not solved and happened to agree, and excused the one it could not ignore in a
  comment ("it happens once, at λ = 100 on the `dancer` pose") — a point `scaling = 0` +
  `polishing = 1` now solves to 0.3407. Excluding by this theorem, with 1 % of slack (dancer's λ=100
  legitimately rises 0.44 %), leaves **six admitted λ over five decades** at a spread of **0.0356**
  and **0.0051** against a 0.5 bar: the claim is narrower in range and stronger inside it. No
  conditioning number is needed. Corollary now asserted: the SHIPPING `softPenalty = 100` must itself
  be an admitted minimiser, or the sweep says nothing about the app.
- **THE NAME PRINTED BESIDE A MAXIMUM CAN BE A DIFFERENT MAXIMUM'S NAME, and this one put two
  wrap-free muscles at the centre of a wrapping investigation.** `WrappedMomentArmLeakTests.Cell`
  recorded `worstBase` for the worst SHIPPED leak and had no field for the worst EXACT leak, so R1's
  maximum was printed with R2's muscle. They are different maxima and they parted company the instant
  the solver stopped contributing — which is why STATUS.md recorded R1's worst as **`piri`** and
  **`glmed3`**, and why the next step it opened asked for the wrapping residual on `piri`. `piri_r`,
  `glmed3_r` AND the actual worst muscle `bflh140_r` all carry **no `PathWrap` at all**. Fixed by
  `Cell.worstExactBase`. Before quoting "the worst X is on muscle M", check M is the argmax of X and
  not of a neighbouring statistic.
- **A GATE MAXIMISED OVER TWO REFERENCES MEASURES THEIR DISAGREEMENT, and R1 is one.** R1 is
  `|d(ours, exact) − d(truth, exact)|` maximised over BOTH of OpenSim's definitions of `truth`, so it
  is a statement about this codebase only while the two agree. Measured 2026-08-09 by sweeping the
  other column as a SUBJECT — same pose, same τ, same truth solve, same statistic, so it lands on
  R1's own scale: the two columns disagree by **126.44 pp** worst and **5.28 pp** median against
  the post-SimmSpline R1's **123.083 / 0.657**. The reopening bar is 1.617 pp, so the reference
  disagrees with itself by 78× the whole gate budget and no work on `MomentArmComputer` can pass R1
  while it is taken that way. NOT a way out of the retirement: after fixing the repository-owned
  endpoint-extrapolation bug, the better-founded analytic column alone is still **3.693 pp** worst
  (median 0.312), 2.28× the bar. Register a gate against ONE reference you can defend, or the gate
  measures the reference.
- **ONE MAXIMUM PROVED SHARING; A DIFFERENT MAXIMUM FOUND A SIMMSPLINE BUG.** At R1's
  central-difference worst cell, `bflh140` — three fixed points, no `PathWrap` or `MovingPathPoint` —
  has exactly the same four arms in every source (`−57.249 / 16.044 / −5.762 / 29.526 mm`) and its
  figure still moves **126.44 pp** when neighbours change. That proves the QP sharing step can move
  an exact row. The separate 42.46 pp ANALYTIC maximum was `bflh140_r`'s own knee arm: ours
  **16.059 mm** vs analytic **13.713 mm**. `walker_knee_r` permits 140° while its five nonlinear
  transform splines stop at 120°; Nimble continued the last cubic where OpenSim continues the
  endpoint tangent. Restore BOTH value and derivative branches, rebuild BOTH static archives, and
  pin both ends: at 130° BioMotion is now **13.713464915 mm** vs OpenSim **13.713465000 mm**. The
  analytic maximum falls to **3.693 pp** on `glmax2`; its cell's largest arm discrepancy is actually
  `gasmed` at 1.047 mm, another direct example of sharing. The central cell still means a future
  per-muscle claim needs the QP coupling sensitivity measured; validating one row never bounds its
  output error.
- **A KKT RESIDUAL NORMALISED BY THE GRADIENT READS 1.0 AT A PERFECT ANSWER.** At an interior optimum
  every gradient component is at rounding level, so dividing the worst violation by `max|∇f|` divides
  noise by noise. `BoxQP` normalises by the largest TERM entering the gradient
  (`ε|a| + λ|Aᵀ(Aa)| + |g|`) instead. Two sibling traps in the same file, both measured: solving
  `(εI + λBᵀB)x = b` by Woodbury cancels ~9 decimal digits when `b ~ 1e7` and `εx ~ 1e-2` (three
  steps of iterative refinement recover them; without it the residual was 1.0), and a "solve on the
  free set, then CLAMP" active-set loop cycles forever because a clamp is not a descent step — take
  the longest feasible step and release ONE constraint per iteration. Also: an instrument that solves
  the same arms under a PROPORTIONAL torque and compares against `100(c−1)/(0.5(1+c))` is only valid
  while NO activation is on a bound; in an 80-muscle rig it measures the ACTIVE SET and reported
  18–45 pp of "solver noise" that was a real nonlinearity of the QP.
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
- **A GREEN ASSERTION ON A STRING IS A LOCK ON A CLAIM, and it kept a refuted sentence on the most-
  read screen in the product.** `GaitLoadSummary.perMuscleRetirementSentence` told every user that
  "66 of its muscles are given a straight line where the real tendon wraps around bone" and that the
  error is "around 10 percentage points on this app's own test rig", and
  `MuscleOverlay.anatomyOnlyNote` said the first half on BOTH the live and offline screens — while
  `MomentArmComputer`'s runtime fidelity report read **76 solved / 0 unmodelled** and the rig read
  **1.4022 pp**. Two assertions of the form `XCTAssertTrue(text.contains("wraps around bone"))` had
  been written when the statement was true, so the suite REQUIRED the false version and went green
  on it for a build. A fix's own commit is where its user-facing text goes stale, because that is the
  commit whose diff nobody greps for prose. Since 2026-08-09 both assertions name the mechanism that
  is true now AND carry the negative (`XCTAssertFalse(contains("straight line"))`), which is what
  makes the pair asymmetric to staleness. Corollary, same round: a REGISTERED REOPENING CONDITION can
  be satisfied while the decision it guards is correctly no — the flag's comment said "model the 76
  missing `PathWrap` references and re-run the rig", both halves now hold, and R1/R2/R7 keep the flag
  `false` regardless. State a reopening condition as the gates that decide, or it reads as
  permission. **Second corollary, earned the very next commit: naming a MECHANISM instead of a number
  is not protection.** The replacement paragraph blamed the sharing step — "it stops as soon as it is
  close enough … about 15 percentage points" — and the solver fix took that quantity from 14.88 pp to
  4.4994e-05 pp, so the same paragraph went stale twice in two commits, each time with a green
  `XCTAssertTrue` on its substring. The wording that finally holds is the one that CANNOT be
  repaired: nothing puts two muscles' efforts on a common scale, because the sharing step divides by
  each muscle's own leverage and its own maximum force. When a user-facing reason describes something
  the roadmap intends to fix, it will go stale on the commit that fixes it — so assert it with a
  negative for every refuted version, and prefer a structural reason to a numerical one.
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
