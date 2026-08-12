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
                load SAM + Nimble models → CONTACT CAPABILITY [FIRST]
                  → bundled models absent: skip camera pass, pose/timing only
                  → future contact-valid model:
                    typed camera calibration admission
                      → absent profile: skip reader, calibration unavailable
                      → admitted: ≤4 s / ≤1000 actual SOURCE-PTS samples
                        → cadence/coverage + background registration
                        → translation/rotation/scale clip state
                video/photo → Vision person box → 512x512 warp
                  → SAM3DBodyPose.mlmodelc (Core ML, CPU+GPU)
                  → 127 MHR joints → MHRRetarget → BODY-SIZE GATE

                              ↓
                    BodyFrame (20 ARKit joint ids)   ← the ONLY seam
                              ↓
   NimbleEngine ─→ Nimble IK  (app-side Levenberg-Marquardt, NOT nimble's
                 │             refineIK — see NimbleBridge.mm)
                 ├→ kinematics-only posture findings and gait contact timing
                 ├→ Savitzky-Golay, 9-tap CENTRED (dates results 4 samples back)
                 └→ validated foot-support capability gate             [FIRST]
                       ├→ bundled models: ABSENT → pose/anatomy/timing only
                       └→ future validated model → camera solve authorization
                              ├→ denied state: stop before solveIDGRF, pose-only
                              └→ permitted state → gravity-aligned axes
                                   ├→ static equilibrium: no root derivative needed
                                   └→ temporal: whole window has qualified root trajectory
                                        └→ floor trust → ID → moment arms → QP
                              ↓
        ┌─────────────────────┴─────────────────────┐
   3-D muscle ANATOMY overlay            posture findings layer
   (RealityKit; fixed 26-capsule         (kinematics only — reads
   set, ONE constant colour, takes       BodyFrame.joints, NOT ik/id/muscle)
   no activation input at all)            + 2-D overlay on the source photo

# PERFORMANCE RECEIPT. Rajagopal2016 has 80 muscles and 39 XML coordinates,
# of which Nimble exposes 37 runtime DOFs. FullBody.osim has 169 coordinates /
# runtime DOFs (171 XML coordinates before the 2026-08-06 patellofemoral weld
# removed knee_angle_*_beta) and 520 muscles. Current timings are separate Debug
# iOS Simulator measurements and MUST NOT be added: moving-input warm-start IK
# (~6 mm/frame) is 1567 ms/frame at 77.8 iterations, an identical-marker warm
# solve is 49 ms/frame,
# and the 520-muscle × 109-coordinate QP is 194.4 ms/frame. Release performance
# on a physical device has not been measured; do not claim a one-second budget or
# real-time throughput from these receipts. See STATUS.md.
```

### Swift ↔ C++ Bridge

ObjC++ wrappers in `BioMotion/Nimble/` and `BioMotion/Muscle/`:
- `NimbleBridge.h/.mm` — loads .osim, runs IK, and owns the runtime DOF mask. Raw zero-external-force and near-CoP ID live in a Debug-host diagnostics category declared only to XCTest; Release contains neither selector, and Release-configuration tests do not support the diagnostics. The only product ID entry checks `hasValidatedFootContactSupport` first, and both bundled models return false. Registers virtual markers at joint centers for ARKit compatibility. **The IK solve is the app's own Levenberg-Marquardt**, not `Skeleton::fitMarkersToWorldPositions` / `math::refineIK`; the vendored nimble tree is untouched.
- `MuscleSolver.h/.mm` — parses the model's muscles (520 in FullBody.osim, 80 in Rajagopal2016), runs OSQP static optimization
- `MomentArmComputer.h/.mm` — parses muscle paths, computes moment arms via FK + numerical differentiation. Shares the bridge's skeleton (`NimbleBridge+Internal.h`) rather than parsing a second copy. Applies path WRAPPING, and picks a one-sided difference where the wrap state changes inside the stencil. FullBody's four `MovingPathPoint`s all survive; their `SimmSpline`s use Nimble's OpenSim-compatible evaluator (`parsed 4 / approximated 0`).
- `MusclePathWrap.h/.cpp` — the cylinder AND ellipsoid wrap solvers, ported from opensim-core (Apache 2.0; licence header in the file, attribution in `./NOTICE`). Length only: no `wrap_pts` leave the solver, and only the ellipsoid's `hybrid` `<method>` is implemented — the other two are refused, not approximated. FullBody is 76/76 solved with 0 unmodelled references; Rajagopal2016 is 46/46 solved with 0 unmodelled references. Every intentional difference is listed under DEVIATIONS at the top of the .cpp (12 of them).
- Bridging header: `BioMotion/Nimble/BioMotion-Bridging-Header.h`

### Key files

| File | Purpose |
|------|---------|
| `project.yml` | XcodeGen project definition (team, signing, lib paths, build settings) |
| `tools/dependencies.lock.json` | Exact Nimble/OSQP repositories, commits, dual-SDK CMake settings and reviewed archive receipts; enforced before any test simulator starts |
| `BioMotion/Resources/FullBody.osim` | **Production** model — 169 coordinates / runtime DOFs, 520 muscles, full spine + ribcage + upper limb |
| `BioMotion/Resources/Rajagopal2016.osim` | Fallback only (lower extremity: 80 muscles, 39 XML coordinates / 37 Nimble runtime DOFs, 66 markers). Loaded when FullBody.osim is missing from the bundle. |
| `BioMotion/ARKit/BodyTrackingSession.swift` | ARKit body tracking + 1-euro filter |
| `BioMotion/ARKit/MuscleOverlay.swift` | 3-D muscle **anatomy** capsules — fixed set, one colour, no activation input. Read its type doc before adding anything magnitude-shaped |
| `BioMotion/Nimble/NimbleEngine.swift` | Orchestrates IK and fail-closed gates on a serial queue. `processFrame` returns an exact generation/submission receipt; publication authority, physical solver occupancy, and engine-global offline policy ownership are separate leases. Contact capability is checked before camera authorization and any unauthorized solve stops before `solveIDGRF` |
| `BioMotion/Offline/CameraMotionAnalyzer.swift`, `CameraMotionVideoAnalyzer.swift` | Clip-level camera policy/adapter: contact-first admission, typed versioned calibration fingerprint, ≤4 s/≤1000 native samples, actual-PTS cadence/coverage, upright bounded raster, person-excluded background registration, and translation/rotation/scale reduction. Production has no profile and skips the pass fail-closed |
| `BioMotion/Offline/` | Photo/video path: `FrameSource`, `SAM3DPoseEstimator`, `MHRRetarget`, `OfflineSessionRunner` (engine lease, exact receipt wait/routing, `.waitingForModel`, camera-state finalization, batch + SG padding), `OfflineResultStore` (second dynamics projection), and playback/overlay views |
| `BioMotion/Findings/` | `PostureFindings` + `PostureFindingsPanel` — kinematics-only posture measurements with view gating. **No clinical thresholds, no verdicts.** |
| `BioMotion/Gait/GaitAnalysis.swift` | Pure frames-in/report-out gait pass. Owns the product's ONE surviving left/right claim: `contactClaimFloorPercent` = `max(timing resolution, contact-duration sampling half-width)`. Never gate a claim on `GaitResolution` alone |
| `BioMotion/Gait/MeanDifferenceUncertainty.swift` | The single Student-t half-width of a difference of two means, plus `StudentT`. Both the contact-time claim and the muscle path call it — a third claim must not reimplement it |
| `BioMotion/CoreML/`, `BioMotion/AssetPack/` | Core ML loading and observable, single-flight, generation-fenced Apple-Hosted Background Asset delivery (the 1.3 GiB model is NOT in the app bundle; archived payload measured 0.0069 GiB / 7.0 MiB) |
| `BioMotion/Recording/TRCExporter.swift` | OpenSim .trc export |
| `BioMotion/App/CalibrationView.swift` | T-pose calibration with live camera (live path only — the offline path scales from one frame's chain sums, see `MHRRetarget.segmentScaleMarkers`) |
| `BioMotion/Muscle/osqp_interrupt_stub.c` | OSQP interrupt handler stub for iOS |
| `nimblephysics/ios/CMakeLists.txt` | Standalone, reproducible `nimble_ios` CMake target; generates the matching build-tree `dart/config.hpp` |
| `tools/osim_fixes/` | The FullBody.osim edit (patella weld + shoulder axis unit-snap), its measurement harness and revert instructions |
| `tools/opensim_ref/` | The OpenSim 4.6 reference generators (`uv` venv, PyPI `opensim` wheel), all read-only against the shipped `.osim`. `dump_reference.py` → CSV, `analyse.py --write-fixture` → `BioMotionTests/Fixtures/opensim_moment_arms.txt`; `dump_finite_difference.py` → `opensim_moment_arms_fd.txt`, OpenSim's own central difference of its own length (the column a `-dL/dq` implementation is comparable with); `fd_check.py` → analytic vs central for one pose/muscle; `inspect_wrap.py` → the wrapped path point by point, with the solver's raw inputs; `pose_coverage.py` → what the pose grid covers |
| `tools/assetpack/` | Pack build + upload; `dev_bundle_model.sh on\|off` receipt-verifies and bundles the precompiled model locally so the Simulator needs no download |
| `tools/release/archive_release.sh`, `testflight_release.sh` | Controlled signed-archive creation plus local export/explicit validate/upload; the adjacent receipt binds the complete xcarchive bytes to the dependency state actually observed at archive time and is mandatory at export |

### Nimble iOS fork boundary

Fresh setup clones the maintained
[`biomotion/ios-static-c405b05`](https://github.com/shengyang998/nimblephysics/tree/biomotion/ios-static-c405b05)
fork and detaches exact receipt
`0ecf26a1557ee738146511cd81fbe99f2bc94d38`. That fork is the complete source
of truth for the port; do not rebuild it from upstream with hand edits or apply
the historical `nimble-patches/*.patch` files on top. Those files remain for
audit, pinned-baseline replay, and reverse-checks. Grep the fork for
`DART_IOS_BUILD` to find platform boundaries. Key areas:

- `ios/CMakeLists.txt` / `ios/config.hpp.in` — standalone arm64 iOS target and generated DART 6.9.0 config; a source-tree `dart/config.hpp` is forbidden
- `MeshShape.hpp` / `MeshShape_ios.cpp` — Assimp stubs
- `OpenSimParser.cpp` — guarded MarkerFitter, GUIRecording, SdfParser, MJCFExporter includes
- `MarkerAspect.hpp` / `Marker.hpp` — enum `NO` → `CONSTRAINT_NONE` (ObjC macro conflict)
- `AssimpInputResourceAdaptor.hpp`, `SoftMeshShape.hpp` — Assimp guards
- `C3DLoader.*` / `C3DForcePlatforms.*` — iOS keeps the pure C3D/ForcePlate data surface but hides
  loader-only ezc3d and GUI adapters; every iOS consumer target must define `DART_IOS_BUILD=1`
- `XmlHelpers.cpp` — classic-locale standard-library conversion with no Boost dependency
- `LilypadSolver.hpp`, `Anthropometrics.hpp`, `IKErrorReport.hpp` etc — GUIWebsocketServer guards and explicit iOS header boundaries
- `DARTCollisionDetector_ios.cpp` — stub for collision detector factory
- Vendored and pinned: Eigen 3.4.0 (`third_party/eigen`) and tinyxml2 (`third_party/tinyxml2`), including their licenses; no Homebrew or Boost dependency path

## Gotchas

- **Managed Background Assets progress is system truth.** Display only the
  finite/clamped `Progress.fractionCompleted`; its unit counts are not documented
  as bytes and must never be labelled MB. A missing model is `.waitingForModel`,
  not runner `.failed`: the shared store owns checking/downloading/paused,
  explicit Retry and automatic ready-after-leaf-probe. Each attempt constructs
  its observer before ensure, gets one generation, and prevents A from
  publishing into B. After every awaited pack probe, recheck the cached URL:
  the main actor is re-entrant and an older nil probe must not reopen a ready
  store. Relaunch
  probes and rejoins through iOS; never persist a percentage, resume data,
  `AssetPack` or pack URL. Resolve the required
  `SAM3DBodyPose.mlmodelc/coremldata.bin` leaf and take its parent, even though
  the SDK also documents directory/package URLs. Closing the import sheet or
  cancelling analysis must not cancel the OS transfer. Injected tests and the
  Simulator prove local state/UI behavior only; TestFlight CDN events, relaunch
  recovery and `MLModel(contentsOf:)` still need a device receipt.
- **Nimble build boundary**: use CMake 3.24+ with Ninja and configure with
  root-relative `cmake --fresh -S nimblephysics/ios -B
  nimblephysics/build_ios` / `build_sim`. Each build tree owns its generated
  `dart/config.hpp`, but the tracked Nimble, vendored Eigen/tinyxml2 and OSQP
  source roots must precede ignored build roots in Xcode's header search paths.
  The generated-header inventory permits only the locked `dart/config.hpp` and
  `osqp_configure.h` for the active SDK. The target exports `DART_IOS_BUILD=1`,
  `DART_USE_IDENTITY_JACOBIAN=1`, `EIGEN_DONT_PARALLELIZE=1`, and
  `EIGEN_MPL2_ONLY=1`. Both device and Simulator archives are arm64-only.
  Host-native fault/leak probes require the explicit
  `-DNIMBLE_IOS_HOST_PROBE=ON` option.
- **Native dependency receipt**: `tools/dependencies.lock.json` pins the
  maintained Nimble fork at `0ecf26a1557ee738146511cd81fbe99f2bc94d38`
  and OSQP at `1572ae068e9ce9ca723cf8223548ade1ff7acc29`.
  `/bin/bash -p tools/tests/dependency_boundary_probe.sh` requires Nimble, OSQP
  and both fetched QDLDL checkouts to have physical roots and local real `.git`
  directories, no replace refs or index assume-unchanged/skip-worktree flags,
  and every tracked working-tree blob byte-identical to `HEAD`. It also rejects
  ignored/untracked material outside the reviewed build roots, requires a
  configured remote matching each locked repository,
  verifies the exact device/Simulator CMake ABI settings, generated headers and
  per-member iOS platform receipts, byte-pins both reproducible Nimble archives,
  binds OSQP's normalized object-member content and fetched QDLDL checkout, and
  pins Xcode 26.4 build `17E192` plus iPhoneOS SDK 26.4 build `23E237`.
  Generated-project inspection owns the PBXProject and exactly the three native
  targets, every Debug/Release configuration list and setting surface, target
  dependency, ordered build phase and referenced build file. Extra targets,
  framework/package linkage, per-file flags, base xcconfigs, tool overrides,
  unattached graph objects, shared/user schemes, `xcuserdata` and project
  sidecars fail closed. The two developer-model phases must also embed the
  reviewed guard at SHA-256
  `a83bd4b5fbafb6442358ce6dd06627c574514a97c2fa7c28bf8750c8a29223d6`.
  It derives the SDK-conditioned direct `.a` inputs from the lock and pins the
  tracked-first header order and consumer ABI macros; name-based Nimble/OSQP
  lookup, competing header/flag settings, same-name dylib redirection, and
  unattached decoy configurations fail closed. The main test runner executes
  this probe before taking a simulator lock or booting
  a device. `dependency_boundary_probe.sh --snapshot` emits the same inspection
  as one canonical JSON line for release tooling; it includes the raw lock hash,
  actual checkout identities, dual-SDK artifact/header/CMake observations and
  project-linkage hashes without embedding the checkout root. Generate the
  pbxproj before running the probe. Its snapshot output is one canonical JSON
  payload plus its terminating newline; do not bind documentation or callers to
  a historical byte count. Never delete tracked files from either
  pinned checkout to reduce its transfer size. Run every protected shell gate
  directly through its `#!/bin/bash -p` shebang or explicitly with
  `/bin/bash -p`. `bash script.sh` is unsupported and is not evidence because
  hostile `BASH_ENV` content can execute before the script reaches its own
  guard or sanitizer. The adversarial dependency suite passes **83/83**. On the
  reviewed Asset Pack tree, the protected gate passes fast **652/652** and slow
  **1/1**, with zero failures, skips, expected failures, or test-host restarts;
  its structured receipts are under `/tmp/biomotion-tests.SbMAmG`. Hosted pack
  delivery remains separate TestFlight/device evidence.
- **Release archive provenance**: never hand-run `xcodebuild archive`. After
  XcodeGen, execute `tools/release/archive_release.sh` directly or with
  `/bin/bash -p`; it compares the full observed dependency snapshot before and
  after the signed build, gives Xcode a fresh mode-0700 DerivedData, runs
  resource/privacy gates in private staging, and publishes the `.xcarchive` only
  with an adjacent mode-0600 receipt. Its local gates use `HOME=/var/empty`; only
  `xcodebuild` receives the passwd-derived account HOME. After the snapshots
  match, the initial observation is passed explicitly on stdin to the receipt
  sealer. `testflight_release.sh` reverifies the current dependency
  tree, receipt, complete archive tree and executable before and after local
  export. Both wrappers launch their child gates with fixed environments;
  TestFlight credentials reach only the final trusted-user `xcrun` calls after
  every local gate and a byte-pinned private IPA snapshot. The test runner also
  owns a fresh private DerivedData for every invocation. These gates assume a
  trusted, quiescent same-user build machine. The receipt proves all tracked
  blobs in the four nested checkouts matched their recorded `HEAD` at both
  observations, but it is not a signature or a link-map/dependency-closure
  proof that every inspected source, header or archive member reached the
  executable.
- **Release UI is a compile-time boundary.** The BioMotion target's Debug
  configuration has exactly `$(inherited) BIOMOTION_INTERNAL_UI`; Release must
  not define it, and `#if BIOMOTION_INTERNAL_UI && !DEBUG` is a compile error.
  Internal panels, self-tests, backend/tool names, raw metrics and checksums must
  live under the exact guard, never behind `UserDefaults`, `AppStorage`, a
  server flag, or a bare `DEBUG` condition. Product views consume typed failures
  with stable actionable copy; preserve raw framework/path detail only as an
  internal diagnostic and never in shared export warnings. The source/project
  gate verifies guard authority, the Release executable is scanned for reviewed
  literals, and the adversarial resource harness passes **45/45**.
- **Eigen version**: Nimble requires Eigen 3.x. Eigen 5.x (Homebrew default) has breaking API changes. Use vendored `third_party/eigen` (3.4.0).
- **Marker names**: ARKit joints map to virtual markers at body node origins, NOT to the model's surface markers (RASI, LASI etc). See `NimbleBridge.mm` virtual marker registration.
- **Stable joint id is not marker anatomy.** Live `hips_joint` resolves to `PELVIS`; MHR keeps the
  same stable id but resolves to `MHR_ROOT`. Its coordinate remains raw MHR joint 1 (15.1 mm from
  the source HJC midpoint); the model marker is an explicit HJC-midpoint proxy, not a claim that
  those points are identical. The current 20-marker MHR receipt reports **1.5365 cm before
  source-aware scaling** and **1.2758 cm after it**; these are different measurement conditions,
  not two interchangeable RMS values. Preserve `opensimMarkerNameOverride` through every filter/copy. TRC
  must fail if one id changes marker alias across frames or two ids collapse to one marker.
- **C++ exceptions**: Always use C++ `try/catch`, never ObjC `@try/@catch` — ObjC exceptions don't catch `std::exception` or SIGSEGV.
- **Build number**: Must increment `CURRENT_PROJECT_VERSION` in `project.yml` before each TestFlight upload.
- **Library search paths**: Conditional on SDK — `[sdk=iphoneos*]` for device, `[sdk=iphonesimulator*]` for simulator.
- **Nimble source is not the linked artefact.** The app links
  `nimblephysics/build_ios/libnimble_ios.a` and `build_sim/libnimble_ios.a`, not
  the stale XCFramework. After changing vendored C++, commit the change on the
  maintained fork, rebuild BOTH archives from `nimblephysics/ios/CMakeLists.txt`,
  and update the pinned receipt/docs. A source-only fix can otherwise look
  correct in `git diff` while every test still runs the old object code.
- **XcodeGen**: Always run `xcodegen generate` after editing `project.yml` — never edit `BioMotion.xcodeproj/` by hand. A **new test file** needs it too, even when `project.yml` is unchanged, or it sits on disk silently not running.
- **Native-rate sampling is span-bounded and frame-bounded.** It targets up to 4 s, but the
  601-frame cap covers about 2.5 s at 240 fps; it is not the same 120-call budget as sparse mode.
  Keep selector copy in `FrameSource.nativeWindowDisclosure` and truncation causes in
  `FrameBudgetNotice`, where `OfflineDisclosureTests` pins the exact-window boundary.
- **Camera-reference sampling is a different native reader, not the pose sample list.** Require
  exactly one video track and pass only it to the composition output; multi-track input is
  indeterminate until the product owns a selection policy. Reject empty edits in-range, cropped
  clean apertures, non-square pixel aspect ratio, and inconsistent encoded rasters: calibration is
  defined over one full render-pixel domain. Contact capability and typed calibration
  readiness are resolved first; current bundled models and production's absent profile skip this
  reader. An admitted request is at most 4 s and 1000 actual native samples. Preserve the selected
  track with `sourceTrackIDForFrameTiming`; robust returned PTS/duration cadence sets the gap and
  one-way endpoint tolerance, while nominal FPS never widens permission. Accept `preferredTransform`
  only when its finite 2x2 part is unit-orthogonal within 1e-4 (mirrors allowed; scale/shear/singular
  metadata fail closed), zero the transformed origin, and scale the layer into
  the profile-owned `renderSize`/maximum pixel count, and record the actual dimensions in evidence.
  The actual even BGRA output must satisfy the 4096 px dimension, 4:1 aspect and 64 MiB retained
  `bytesPerRow * height * 5` limits; estimated tight rows are not a memory receipt.
  Merely shrinking `renderSize` crops, non-1
  `renderScale` is for `AVPlayerItem`, and retaining full-resolution BGRA snapshots makes 4K memory
  unbounded. At EOF require the exact `.completed` reader state plus first/last-sample coverage of
  the requested range; one valid interior derivative window cannot represent an unread head or
  tail. Pin `VNDetectHumanRectanglesRequest` to revision 2 and translational registration to revision
  1; runtime support, both revision values and the typed revision-3 fingerprint must match. The
  fingerprint also binds dimensions, classification knobs, cadence and derivative domains; Runner
  and reducer both validate it, so a direct call cannot bypass admission.
- **Camera registration is background evidence with a quality gate, not optical truth.** Exclude the
  inflated union of both frames' person boxes, screen each tile for two-dimensional structure,
  appearance/uniqueness and a finite translation-only Vision transform. Every planned tile must
  register; every quality-valid tile enters one similarity fit, with no iterative deletion that can
  manufacture stillness. Recompute coverage from the actual sampled-box union of the final usable,
  spatially dispersed tiles. Box-average both frames on a fixed 8 px render lattice, sample
  reference-target correlation every 32 px over the full +/-48 px two-dimensional residual surface,
  using one common inward domain for every fine candidate. Then enumerate every 8 px offset with at
  least 64 overlapping boxes: fixed 8x8 matched domains in the four sign quadrants and exact
  48-sample matched domains in the narrow tails. Compare each remote candidate only with zero lag on
  the identical samples. Count the `width*height` zero-product prefix and every shifted product;
  local+global cost must be <=500,000 (the 48x48 reference grid is exactly 496,889). Carve larger
  regular regions into deterministic, globally phase-aligned 48x45-box leaves; use bounded long-axis
  recursion for smaller irregular regions. Limit either path to 16 leaves and never reduce candidate
  stride or sample count to fit. Require the broad alias rectangle to contain the complete fine square,
  not merely have a larger candidate count. Pre-screen feasibility before Vision, discard narrow strips, and recompute
  sampled coverage. Require Vision's candidate to be the local global peak with provisional
  normalized correlation >= 0.5, and penalize every distinct local or remote matched-domain alias.
  Bind all box/grid/search/global-domain values, the cost cap, correlation floor, Vision revisions
  and adapter revision into calibration. Jumps, slow drift, zoom,
  periodic aliasing, parallax, weak coverage, timestamp gaps, and inconsistent fits must become
  explicit moving/ambiguous/indeterminate states, never zero motion.
- **Dynamics spatial admission is contact -> camera -> gravity -> root.** A photo or explicit
  single frame is `.notRequiredForSingleFrame`: it can authorize only future static-equilibrium
  dynamics, never temporal/gait dynamics. A multi-frame clip authorizes temporal dynamics only as
  `.staticWithinBudget`; unmeasured, moving, between-band, calibration-required,
  calibration-unavailable, and indeterminate
  states authorize neither. Camera authorization still does **not** establish gravity or an
  inertial root trajectory. `BodyFrame.DynamicsReference` is fail-closed by default: live ARKit
  construction explicitly uses `.liveARKit`, which records gravity-aligned global position but is
  still position-only for temporal dynamics; only the separately explicit
  `.dynamicsQualifiedWorld` authority admits a calibrated temporal root. Current MHR production
  frames use `.mhrRootRelative`; a raw-composed MHR frame is only
  `.mhrCameraRelativePosition`.
  `OfflineSessionRunner` starts camera-denied, maps the finalized state to
  `CameraDynamicsAuthorization`, and `processFrame` snapshots it plus the frame reference before
  crossing to `solverQueue`. A temporal derivative window must carry one unanimous qualified
  reference; a newer frame cannot authorize older differently-labelled samples. Contact support is
  checked first, then camera, gravity and (for a temporal solve) dynamics-qualified root provenance
  refuse upstream of `solveIDGRF`;
  `OfflineResultStore` repeats the same projection order to
  strip stale ID/muscle while preserving pose. Store authorization/results are ordinary stored
  state committed before one explicit notification; static-ID provenance does not depend on an
  optional muscle solve. Local run ownership, a pre-fence lifecycle invocation epoch and an
  engine-global conditional lease prevent a cancelled/second Runner from weakening its successor or
  admitting live work into an offline result. Segment resets and offline scaling must present the
  captured lease and recheck it after any synchronous reset notification. That lease also owns the
  imported subject's temporary geometry: offline scaling never updates the solver-queue live recipe,
  and exact release enqueues recipe/default restoration plus bridge/QP/filter/ground cleanup before
  notifying observers, so a reentrant successor's solver work is FIFO after the whole block. A stale
  release restores nothing. Production's typed
  profile/fingerprint stays nil until the exact algorithm,
  raster, cadence/window and memory domain passes versioned tripod/static calibration and disjoint
  moving-camera fixtures; production therefore returns calibration-unavailable without opening the
  reader. Device Vision behaviour, peak memory/runtime, and cancellation latency remain external.
- **Never wire raw `estimate.camT` into the production MHR BodyFrame as a one-line change.**
  `(x, -y, -z)` is a valid metric camera-relative position, not a gravity-aligned world trajectory.
  The current photo overlay already projects pinned markers with raw `camT`; composing it in the
  runner without changing that convention applies translation twice. Activation must atomically
  provide synchronized camera-to-gravity evidence, calibrated root/depth and whole-window continuity
  policy, switch projection semantics, and reset solver temporal/ground state. Non-finite or
  structurally out-of-domain `joint_coords`, wrong-rank/non-finite/non-positive/out-of-domain
  `cam_t`, non-finite projection results, and marker arithmetic that overflows after retargeting
  are rejected before IK/display; those are numeric safety checks, not physical validation.
- **Offline frame completion is an exact receipt, never `objectWillChange`.** An accepted receipt is
  `(generation, submissionID)` and completion is `.published/.failed/.superseded` for that identity.
  Publication authorization and physical solver occupancy are separate: timeout/cancel revokes the
  former before resuming, while the non-cancellable solver retains the latter so B/C cannot queue
  behind a stuck A. `lastSolveReceipt` must match before store routing. The offline lease also owns
  resets, so a queued live tracking-loss callback cannot supersede the batch. Padding propagates
  failure and never increments a false push. Timeout, native IK failure, admission refusal,
  exhausted busy retries and external supersession remain distinct frame statuses/user messages.
  Result fields and full reset (including ground) commit as one ordinary-storage main-thread batch,
  then send one explicit `objectWillChange`; no owner-sensitive write may follow that synchronous
  notification. Runner acquire/release/defer must likewise recheck or retire the exact token,
  invocation and lease before a callback can start a successor. Do not reintroduce per-field
  `@Published` engine/store result writes, settle delays, ownerless resets, or global `lastSolve`
  reads.
- **A picked video is an owned resource, not a naked provider URL.** `PhotosPicker`'s provider URL
  is borrowed. `PickedMovie` must synchronously copy it inside the import-only transfer closure into
  a unique mode-0700 system-temporary directory, then pass `AppOwnedTemporaryVideo` through the
  selection, `RunSource`, decoder and every child task that reads the file. The final owner removes
  only that directory; a crash/force-kill remains system-temporary cleanup territory. Selection
  generations advance before predecessor cancellation: stale A success/failure/cancellation cannot
  publish into B, while a failed/cancelled B retains the last usable selection. Active analysis
  cancellation clears partial playback but keeps that selection for retry; idle cancellation keeps
  completed playback. Preserve the pre-reset and post-reset invocation/lease fences because both
  engine release and result-store reset notify synchronously and may start a successor.
- **Analysis FPS has one source:** the median interval of surviving frame timestamps, copied from
  the internal `GaitReport` into `GaitTimingReport.timing.framesPerSecond`. AVAsset's nominal rate
  is valid for native decoding and frame-budget notices only. Do not add a second rate to
  `GaitOutcome.analysed`; sparse sampling is precisely where nominal rate and analysed cadence
  differ.
- **A video whole-frame fallback is a temporal gap, not a failed pose.** Keep its projected skeleton
  visible, but exclude it before body plausibility, scale, SG priming, Nimble submission, gait, ID,
  and muscle. A photo fallback remains analysable for pose, but without an explicit/external floor
  it is not evidence for ID/GRF/muscle. Segment on `DecodedFrame.index`/
  `BodyFrame.frameNumber`; reset realtime state before the next waiter, and pad held poses only at
  real requested clip endpoints — never across an internal or leading/trailing known gap.
- **A stored biomechanics result is one solve generation.** Route a complete
  `OfflineResultStore.BiomechanicsPayload` from one `SolveRecord`; never nil-coalesce IK, ID, or
  muscle fields with the previous frame payload. The store's one projector validates success,
  temporal eligibility, a tracked `BodyFrame`, and body/IK timestamps within 1 ms of the owner
  frame. `.available` additionally requires a same-generation ID; muscle has its own same-generation
  timestamp gate. A stale ID clears ID/muscle but preserves valid IK and the report-neutral motion
  verdict; a stale muscle clears only muscle. Missing pose provenance clears the solve envelope.
  Session gates can still downgrade an otherwise valid ID in the engine's exact order: contact
  capability, clip camera permission, gravity alignment, then solve-class root provenance (required
  for temporal dynamics). This repeats the engine's pre-`solveIDGRF` authorization as a stale-payload
  backstop. Image/decoder/model provenance and `FrameStatus` stay fixed. Absence is never a measured
  zero.
  Before a gait replacement pass, invalidate all pass-one dynamics to
  `.analysisPassIncomplete`; only same-generation pass-two solves may repopulate them.
- **Validated foot support is the first hard dynamics boundary; camera authorization and ground
  trust are later, separately necessary boundaries.** Both bundled `.osim` files have an empty
  `ContactGeometrySet`, and the
  near-CoP routine supplies no support-polygon, unilateral-contact, or friction constraint. Thus
  `hasValidatedFootContactSupport == false`, `.contactSupportUnavailable` wins even with an explicit
  floor, and no bundled frame may publish ID/GRF/CoP/muscle/gait-load output. Refilming cannot change
  a model capability. If a future model/solver pair passes that gate, first require the clip camera
  state to authorize this static or temporal solve before calling `solveIDGRF`. That call observes
  the feet
  before solving, so inspect `groundHeightTrusted` after the call: observations 1–29 remain pose-only
  and observation 30 is the first same-call result eligible for the later gates. SG endpoint replay
  supplies filter context, not independent floor evidence. A second pass over the same continuous
  clip resets IK/QP/filter state but preserves that clip's ground estimate; a new clip or AR
  world-origin reset discards it. State touched before the main-thread generation guard (including
  display filters) is solver-queue-owned; recording history is written only after that guard.
- **The published gait result is a detached timing-only value.** `GaitAnalysis` may retain its
  research `GaitReport`, force hypothesis and private plan builder, but `GaitOutcome` accepts only
  `GaitTimingReport`: copied resolution, contact-time findings, timing refusals and flags, with no
  reference back to force, residual, `GaitPlan` or `GaitLoadSummary`. `GaitReportPanel` has no load
  summary initializer. The runner snapshots contact capability; for either bundled model the
  conditional pass-one invalidation is skipped, timing is published, and the runner returns before
  constructing a plan or running a second pass. A future capability-valid branch clears pass-one
  dynamics before publishing `.analysed`, so observers cannot pair it with stale loads.
- **The skeleton is shared process-wide.** `NimbleBridge -sharedSkeleton` hands the same `shared_ptr` to `MomentArmComputer` and the ID path, and it survives across `NimbleBridge` instances. Anything that reads "wherever the skeleton currently sits" is therefore reading process history, not the model — that was a real defect in `applyDOFMaskWithNames:` (fixed 2026-08-07) and it is why the IK cold seed is an explicit `neutralSeedPose`.
- **Live calibration is a fail-closed observation and mutation boundary.**
  `BodyTrackingSession` publishes typed permission/searching/tracking/interruption/failure state and
  clears stale frames whenever the camera session becomes inactive; a late permission,
  interruption or failure callback cannot override an explicitly paused session. Calibration
  starts only with tracking plus a loaded native model, counts 60 strictly monotonic
  `(frameNumber, timestamp)` observations, and reports
  tracking loss or a six-second sparse/frozen-stream timeout instead of polling one frame repeatedly.
  Manual height uses the user's decimal locale. Leaving for offline import invalidates the timer and
  pending UI attempt. Success is published only after `scaleLiveModel` receives the real native FIFO
  result; queue admission, a failed native scale, or an active offline lease is never shown as done.
- **Subject scaling starts from the loaded model, never the current skeleton or another model's
  constants.** `loadModelFromPath:` caches the exact default body-scale vector plus lower/upper and
  two source-specific trunk references: live `PELVIS` uses pelvis-origin→shoulder-mid; MHR_ROOT uses
  bilateral-HJC-mid→shoulder-mid. `scaleModelWithHeight:` applies
  `cachedDefault × clamp(measured/cachedReference)`. Recomputing references after a prior scale
  compounds; writing a uniform ratio discards native anisotropy; failing to replace all caches on
  reload leaks Rajagopal proportions into FullBody. The live calibration recipe is value-only and
  solver-queue-owned; only a successful live native scale replaces it, offline scaling cannot, and a
  successful reload invalidates it. `restoreLoadedModelBodyScales` restores only the loaded body-scale
  baseline and must not be folded into ordinary tracking/session resets, which preserve the current
  live subject. `ModelScalingTests` pins identity, idempotence, source-specific MHR scaling,
  cross-model reload, the full scale vector, and neutral pelvis/femur/talus/humerus/hand transforms
  across live → offline → live/default restoration.

## Readings that lie — each has already cost a wrong conclusion

- **`Executed N tests, with 0 failures (0 unexpected)` is not a pass, and this one governs every
  other reading in this file.** A killed test host still prints that line for every suite that
  finished before the kill, and reports the lost tests as neither passed nor failed. Measured:
  two `xcodebuild test` processes sharing one simulator UDID each ended `Executed 2 tests, with 0
  failures (0 unexpected)` / 5 `Restarting after unexpected exit` / `** TEST FAILED **`, on a
  19-test selection that reads `Executed 19 tests` / 0 restarts / `** TEST SUCCEEDED **` when run
  alone on a private device. Naming a simulator (`name=iPhone 17`) instead of a UDID you own is the
  whole mechanism, and it is why three reviewers got three answers on 2026-08-07. Run the named
  lanes in `tools/run_tests.sh`: `fast` is exactly 698 non-E1 tests, `slow` is exactly the one E1
  test, and `all` runs both and is the commit gate. A lane passes only when `xcodebuild` exits 0,
  the final log verdict is `TEST SUCCEEDED`, the xcresult summary is readable, the executed count
  is exact, and failures, skips, expected failures, and crash restarts are all zero. `subset`
  requires at least one selected test, labels itself non-gating, and is the only lane that accepts
  caller arguments. The three gating lanes accept none: their fixed invocation is part of the
  receipt. Even `subset` rejects skips, retry/repetition controls, and alternate test
  configurations; a later successful retry is not evidence that the first execution passed.
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
  **exactly 0.0**. And the then-shipped, pre-wrap `MomentArmComputer` reproduced
  OpenSim-with-wrapping-disabled to **4.39 mm** worst case over 12,384 samples
  while the gap to the truth was **146.6 mm**, so 97 % of that historical error
  was the then-missing solver, not the other implementation differences. Worst named pairs:
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
  absence on both screens (`MuscleOverlayClaimTests`). `LiveAnatomyPresentation` is the live
  renderer/control/disclosure gate: calibration always refuses the layer, and tracking requires an
  active session plus a current frame before either the control or capsules can appear. Do not gate
  this anatomical layer on Nimble model loading; it consumes joints, not a solve.
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
- **`leftFootLoadFraction`/`rightFootLoadFraction` are historical raw-solver fields; neither the SUM
  nor the SPLIT is publishable for a bundled model.** `AccuracyBadge(label:
  "L/R load", value: "0.62|0.38")` had no caption, no floor and nothing validating it, on the app's
  most-used surface, in the exact framing the offline path spent four rounds retiring — and its
  green indicator was `abs(total − 1.0) < 0.3`, keyed to the sum alone. The split is not merely
  unchecked: `NimbleBridge.mm:1499` seeds the solve with a hardcoded 50/50 wrench guess whenever
  both feet are down, and double support is statically indeterminate at ±18 pp with a perfectly
  known CoM against a ~10 pp meaningful threshold. The later `GRF sum` badge was still generated by
  a pair with empty contact geometry and no support-domain/unilateral/friction constraints. The
  `.contactSupportUnavailable` gate now keeps that entire row unreachable; its availability detail,
  not a number, is the bundled-model surface.
- **The historical gait residual is blind to WHICH foot carries the load and does not validate
  contact support.** `GaitFrameOutcome.residualInBodyWeights` is built from
  `leftFootForce.y + rightFootForce.y`, and the raw near-CoP constraint fixes that SUM exactly — so
  a 50/50 split and a 100/0 split give the identical residual while halving one leg's torques. The
  older residuals remain engineering falsifiers only; no bundled product frame produces one.
- **That same historical residual is VERTICAL ONLY, and it is not `‖a_artic‖/g`.** The fore-aft
  wrench components exist but are discarded. No check examines the missing support domain or the
  phase-dependent 0.2-0.35 BW fore-aft term. Treat every earlier `GaitLoadSummary` residual as an
  unvalidated raw-solver receipt, never as permission to publish load.
- **A "peak" over a side's frames is not a left/right statistic.** The two legs contribute different numbers of usable frames (a contact one sample longer yields twice as many at `taps = 5`), and `E[max of n]` grows with `n` — measured at **+8.07 %** of fabricated left-high asymmetry on a symmetric runner, 80 % of `video_012`'s own 10.14 % publication floor. Any statistic over per-side frames must have an expectation independent of the count; `GaitLoadSummary.MuscleLoad` uses one sample per contact, averaged over contacts.
- **A gate that measured nothing is not a gate that passed.** `sortedResiduals.last ?? 0` reported max 0, median 0 and "passed" for a clip where no stance frame was ever usable. Check `residualFrameCount`/`residualWasMeasured` before reading any aggregate here.
- **`strideRepeatabilityPercent` can read exactly 0.000 and mean nothing.** It is a CV over touchdown gaps quantised to whole frames; `video_012`'s are all exactly 18 samples. The clip cannot distinguish anything below `GaitSteadiness.boundPercent = 100/stridePeriodFrames`, so the published figure is floored there and `measuredStrideRepeatabilityPercent` holds the raw CV.
- **A visible skeleton says nothing about dynamics.** The skeleton and fixed-colour anatomy are
  drawn from `BodyFrame.joints`. Muscle output would require validated foot support plus the whole
  IK → SG → ID → moment-arm → QP chain; the bundled path stops before ID.
- **A well-drawn torso says nothing about the legs.** A monocular pose model that cannot see a limb does not fail loudly — it returns its mean pose for that limb, which looks like a plausible standing leg. `VNDetectHumanRectanglesRequest.upperBodyOnly` defaulting to `true` hid behind this for a day (fixed 2026-08-07, `PersonBoxTests`). Score limbs separately against an independent estimator; an aggregate that mixes torso and legs dilutes a 3× leg error into noise.
