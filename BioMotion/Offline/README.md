# Offline import (photo/video → pose, anatomy, and gait timing)

Adds a second input path alongside live ARKit tracking: pick a photo or video
from the library, run the frozen `SAM3DBodyPose` Core ML model over sampled
frames, feed the shared IK path, and scrub through the result in a non-AR 3D
view. The shipped contract is pose, fixed-colour anatomy, kinematics-only
posture findings, and gait contact timing. Both bundled OpenSim files have an
empty `ContactGeometrySet`, while the active near-CoP routine supplies no
validated support polygon, unilateral-contact, or friction constraint.
Production therefore stops at `.contactSupportUnavailable`: ID,
ground-reaction force, centre of pressure, muscle effort, and gait-load output
remain nil even with an explicit or statistically trusted floor. Refilming
cannot change that model/solver capability. The path is covered by the ordinary
test gate; the remaining external boundary is real-device/UI validation called
out below.

## Flow

```
OfflineImportView (PhotosPicker: photo or video)
  -> OfflineSessionRunner.run(source:samplingMode:)
       -> poseEstimator.loadModelIfNeeded()          [SAM3DPoseEstimator]
       -> wait for nimble.isModelLoaded               [poll, ≤10s]
       -> nimble.resetSessionState()                  [new clip: clear IK/QP and ground provenance]
       -> FrameSource.decodePhoto / VideoDecoder       [decode]
       -> for each frame:
            SAM3DPoseEstimator.estimate(uiImage:)      [Vision bbox -> warp -> CoreML predict]
            MHRRetarget.makeBodyFrame(jointCoords:)    [-> BodyFrame + source marker names]
            (first successful frame only) nimble.scaleModel(...)  [MHRRetarget.segmentScaleMarkers/estimatedStatureMeters]
            nimble.processFrame(bodyFrame) + wait for NimbleEngine's next publish, or timeout
            -> OfflineResultStore.append(...)
       -> edge-pad trusted requested endpoints so the centred Savitzky-Golay
          window can produce results for the first/last real frames
          [filter support only; replay is not independent ground evidence]
       -> if the clip has usable product timing:
            snapshot hasValidatedFootContactSupport
            future validated-contact branch only:
              resultStore.beginGaitReplacementPass() before analysed publication
            publish detached GaitTimingReport
            guard the capability snapshot
            [both bundled models return here; no plan and no second pass]
            future validated-contact branch continues:
              apply load-only refusal gates and build the private gait plan
              nimble.resetAnalysisPassStatePreservingGround()
              re-submit the same clip
  -> OfflinePlaybackView (RealityKit .nonAR ARView + MuscleOverlay + scrubber)
```

`BodyFrame` keeps a stable joint id separately from its source-specific OpenSim
marker. Live `hips_joint` defaults to `PELVIS`; MHR `hips_joint` carries
`MHR_ROOT`. Filters and test-fixture transformations preserve that override,
and the engine resolves it only after the stable-id whitelist succeeds.

## Design decisions and why

### Backpressure (constraint: never submit frame N+1 while N is in flight)

`NimbleEngine.processFrame` drops a frame outright if a solve is in flight, and
has no per-frame completion callback — every result lands via
`DispatchQueue.main.async` inside `publishResults`, itself only called when
`solveIK` succeeds. `OfflineSessionRunner`'s `NimbleFrameWaiter` calls
`nimble.processFrame(_:)` then awaits the engine's next `objectWillChange`
(Combine), with a fixed 6s timeout treated as a failed frame. See the doc
comments on `NimbleFrameWaiter` and `OfflineSessionRunner` for the full
reasoning, including why a 30ms settle delay after the first
`objectWillChange` is added (it fires from inside a `@Published` property's
`willSet`, before the engine finishes writing the ~12 fields `publishResults`
sets). `OfflineOrchestrationTests` exercises the same waiter against the real
engine, including a reset between two windows; device scheduling remains part
of the external device verification boundary.

### 9-frame Savitzky-Golay warm-up is not ground calibration

`SavitzkyGolayFilter` (`BioMotion/Nimble/SavitzkyGolayFilter.swift`) needs 9
pushes before it emits a centred derivative window. That is a filtering
requirement, not evidence of foot support and not permission to treat replayed
poses as new observations of the floor. This means:

- **A single photo can produce a centred pose/IK result, but it remains
  pose-only with either floor source.** Edge replay can fill the derivative
  window; it cannot manufacture missing contact-support mechanics. It also
  cannot manufacture the 30 independent observations a future validated
  dynamics path would require for a rolling floor.
- `OfflineSessionRunner` replays a real endpoint pose over the leading and
  trailing half-window, at the decoded clip's median cadence. A photo is the
  degenerate 4 head + 1 real + 4 tail sequence. Because every padded push is
  an IDENTICAL pose, the SG filter's velocity/acceleration
  coefficients (which sum to zero for a constant input by construction —
  verified: `[86,-142,-193,-126,0,126,193,142,-86]` sums to 0) come out at
  ~0 regardless of the exact spacing. This licenses a static-hold derivative
  assumption only. It is not permission to publish static-equilibrium ID or
  muscle values. Both bundled models fail the earlier contact-support gate;
  supplying an explicit floor does not change that.
- Padding creates no independent ground observation. Head-pad publications are
  centred on synthetic timestamps and discarded; tail padding advances the
  centred window onto the remaining distinct real frames. Thus one photo still
  contributes one ground observation, not nine, and a video contributes at
  most one observation per real frame whose centred solve reaches ID.
- Padding is legal only when the trusted pose is the real first/last REQUESTED
  decoder slot. An undecodable, pose-rejected, or review-only slot splits the
  stream; no held sample is inserted beside an internal or leading/trailing
  known gap. Each later segment resets SG/hold/display state before the next
  waiter.
- This is surfaced honestly, not silently: `OfflineResultStore.FrameResult`
  carries both `isStaticHoldEstimate` and `DynamicsAvailability`.
  `OfflinePlaybackView` labels the current bundled-model result "Pose only —
  foot contact is not supported" instead of displaying zero-valued dynamics.
  Pose, the anatomy overlay, and kinematic gait timing remain available.
- The padding rewrites the ORIGINAL frame's stored result in place
  (`OfflineResultStore.replaceBiomechanics`) rather than appending a phantom
  extra scrubber row.
- Every routed `SolveRecord` becomes one `BiomechanicsPayload`: IK, optional ID,
  optional muscle, `DynamicsAvailability`, the static-hold flag, and motion
  state replace the prior generation together. The result-store projector first
  requires success, temporal eligibility, a tracked body, and body/IK timestamps
  within 1 ms of the owner frame. `.available` then requires a same-generation
  ID; muscle has its own same-generation timestamp gate. A stale ID clears ID
  and muscle while keeping valid IK/motion; a stale muscle clears only muscle.
  Missing pose provenance clears the solve envelope. A nil/stale value is never
  converted into a measured zero. Image/frame/model provenance and `FrameStatus`
  are not owned by the solve and remain unchanged.
  Starting the gait replacement pass first clears every eligible pass-one ID,
  muscle, and static flag to `.analysisPassIncomplete`; each successful
  same-generation gait solve then replaces that marker. A timeout or missing
  centred publication therefore cannot leave static physics under a running
  result.

### Contact support gates dynamics before ground-plane trust

The two requirements answer different questions. Ground provenance says where
the floor is. Contact support says how a foot may transmit force through it.
Both bundled models have empty `ContactGeometrySet`s, and the active solver does
not impose a validated support domain itself, so knowing the floor is not
enough. `hasValidatedFootContactSupport` is checked first and both bundled
models remain pose-only. An explicit floor, 30 observations, a session reset,
or a second pass cannot unlock a missing capability.

The rolling estimator remains as a necessary second gate for a future
model/solver pair that does define validated support. An external caller may
pin an explicit ground height; that source is trusted immediately and observed
foot heights cannot overwrite it during the session. Without one,
`NimbleBridge` maintains a bounded rolling low-percentile estimate from the
solved model's lowest heel height: the 10th percentile of the most recent 180
observations, shifted down by the 1 cm contact offset. The estimate is
provisional for the first 29 independent observations and becomes trusted at
observation 30.

For a future capability-valid model, ordering at that second boundary remains
deliberate: the current observation updates the floor before trust is checked.
Therefore:

- observations 1–29 remain `.groundPlaneUntrusted` and publish no dynamics;
- observation 30 can satisfy the floor requirement on that **same call**, but
  only a separately validated contact-support model could then proceed to ID;
- a ground observation is evidence from a distinct real centred frame, not a
  `processFrame` push. Savitzky-Golay head/tail replay supplies filter context
  and does not turn one endpoint pose into several observations.

State reset follows coordinate provenance. Starting a new imported clip calls
`resetSessionState()`, clearing the rolling floor, IK warm start, and muscle-QP
warm start. For a future capability-valid model, the private gait second pass
would be over the **same clip**, so
`resetAnalysisPassStatePreservingGround()` clears the SG/hold/display state and
resets IK/QP warm starts while retaining that clip's ground samples and trusted
source. Clearing the floor there would force the second pass to rediscover the
same ground and would withhold its first 29 observations again. Pass-one
dynamics are invalidated before that re-solve begins, so an incomplete second
pass remains pose-only rather than mixing policies. Bundled models return
before this reset/invalidation/replay branch. On the live AR
path, tracking loss or an AR world-origin reset uses the full session reset,
because a floor expressed in the old world frame is no longer valid.

### Whole-frame fallback admission

`SAM3DPoseEstimator` still runs when Vision finds no person box. The result is
not uniformly a failure:

- a photo fallback remains an analysable still pose;
- a video fallback is `.success` and remains projected over the source image
  for review, but carries `videoVisionWholeFrameFallback` and branches before
  body-size plausibility, model scaling, SG priming, Nimble submission, gait,
  ID, or muscle.

The decoder batch retains the bounds of the timestamp request, while surviving
`BodyFrame`s keep the original decoder slot as `frameNumber`. That makes both a
failed decode and an excluded fallback a visible temporal gap. The result store
also refuses later biomechanics routing to an excluded frame, so a future
caller cannot bypass admission by writing fields directly after the fact.

### Calibration without a T-pose

Live tracking calibrates from a 60-frame T-pose hold (`CalibrationView`).
There's no equivalent live capture for an imported clip, so
`OfflineSessionRunner` scales the model from the FIRST temporally eligible,
plausible, successfully-estimated
frame via `MHRRetarget.segmentScaleMarkers`/`estimatedStatureMeters` (both
pose-invariant chain-sum derivations per that file's own extensive doc
comments), then feeds that same frame through `processFrame` normally. This
makes offline import self-sufficient — it doesn't depend on the app having
gone through live calibration first, and works even if the imported subject is
a different person.

MHR scaling emits `MHR_ROOT` and measures trunk length from the bilateral HJC
midpoint to the shoulder midpoint. The bridge caches a matching model
HJC-midpoint reference. Live/legacy `PELVIS` scaling remains separate and uses
the OpenSim pelvis-body-origin reference; the two anatomical points are never
made aliases merely because they share the stable `hips_joint` id.

`nimble.scaleModel` has no completion signal either. It's called immediately
before `processFrame` with no `await` between them; both dispatch onto
`NimbleEngine`'s private SERIAL `solverQueue`, which preserves FIFO submission
order, so the scale operation completes before that frame's IK solve begins.
This is a documented, reasoned argument about GCD queue ordering — **not
something a build could fully re-verify either, but it rests on public,
stable GCD serial-queue semantics rather than private timing behavior.**

### Marker provenance and export disclosure

The raw MHR root is not silently renamed to the OpenSim pelvis origin. The
shipping dancer fixture places raw joint 1 15.081552 mm from its source HJC
midpoint; model-side `MHR_ROOT` is an explicit bilateral-HJC proxy, while
`PELVIS` remains the live/legacy pelvis-body-origin marker. A TRC export fails
closed if one stable joint changes source marker between frames, if an override
is empty, or if two joint ids collapse onto one marker name. If TRC fails while
MOT or STO succeeds, the share bundle includes
`BioMotion_export_warnings.txt`; if that warning cannot be written, the app
shows an error instead of sharing an unexplained partial bundle.

### Rendering — non-AR surface

`SkeletonARView`/`SkeletonOverlayView` is hard-wired to a live `ARSession` and
only shows anything once ARKit's delegate sets `isTracking`. `OfflinePlaybackView`
instead builds its own `ARView(cameraMode: .nonAR, automaticallyConfigureSession:
false)` with a manual `PerspectiveCamera` entity, reusing `MuscleOverlay`
verbatim (not reimplemented). A minimal joint/bone renderer (spheres + boxes,
`UnlitMaterial` so no scene lighting setup is needed) is duplicated in
`OfflineSceneView.Coordinator` — it mirrors `SkeletonOverlayView.Coordinator`'s
approach but is a fresh implementation, since that Coordinator is a private
nested type inside the ARSession-bound view and isn't reusable.

The camera auto-frames once (first frame with tracked joints; fixed for the
rest of the scrub session) from the joint bounding box, rather than a
hardcoded world position — `MHRRetarget.swift`'s own doc comments confirm
`joint_coords` pins the raw MHR source root at a MODEL-CONSTANT `(0, 0.924, 0)` in every
prediction, not a real-world camera distance, so a fixed camera position could
easily show a blank screen. No orbit/manual camera control is implemented
(out of scope for this pass — noted as a natural follow-up, not added to avoid
scope creep).

### Sampling and progress

Default 2fps, user-adjustable 0.5–10fps, plus an explicit single-frame mode
(auto-forced for photos). `FrameSource.maxFramesPerRun = 120` caps a
pathological long-clip × high-fps selection; `OfflineSessionRunner` surfaces
`frameBudgetNotice` rather than truncating silently — a `FrameBudgetNotice`
that names WHICH cause fired (the native clip is shorter than the window, the
native-rate window exceeded the 601-frame run budget, or the sparse scan hit its
120-frame cap) and how many frames were really used. At 240 fps the native cap spans
about 2.5 s even when the clip itself
is exactly as long as the 4 s configured window; neither the notice nor the mode
selector describes that as a long clip. The selector's copy lives in
`FrameSource.nativeWindowDisclosure`, beside the arithmetic it discloses. It replaced a
single boolean whose one sentence stated both causes wrongly on a short clip. Progress ETA
(`OfflineSessionRunner.eta`) is derived from the running average of measured
per-frame wall time (`perFrameDurations`), shown as "estimating time…" (not a
fabricated number) until at least one frame has completed.

The video's nominal track rate is used only where it is the relevant fact: native-rate sampling,
decode-memory sizing, and the budget notice. Analysis cadence comes from the median interval of the
surviving `BodyFrame` timestamps and is stored in `GaitReport.framesPerSecond`. `GaitOutcome.analysed`
publishes a detached `GaitTimingReport` whose `GaitTimingSummary` copies that rate; it carries no
second FPS value, full research report, dynamics plan, residual or load summary. A sparse 10 fps
analysis of a nominal 30 fps track therefore cannot print 30 fps or scale camera advice from it.

An `.analysed` `GaitTimingReport` owns the whole product gait UI: resolution, left/right contact
time and timing flags. `GaitReportPanel` has no `GaitLoadSummary` initializer, so hiding a load
block is not the safety mechanism — load data cannot cross the type boundary. For the bundled
models `hasValidatedFootContactSupport` is false; the runner publishes timing, then returns before
`makePlan` and before the second dynamics pass. A derivative-window refusal belongs only to that
unreached plan and does not remove timestamp-derived timing.

### Model loading

`MLModelConfiguration.computeUnits = .cpuAndGPU` (not `.all`) per this task's
explicit constraint. Loading happens on a dedicated background queue via a
checked continuation, resolving `SAM3DBodyPose.mlmodelc` first (what Xcode
actually compiles a bundled `.mlpackage` target member into) with `.mlpackage`
as a fallback name. A missing model throws `EstimatorError.modelNotBundled`
with a specific message — the UI shows `runner.phase = .failed(...)`, no crash,
no silent no-op. Loading state is shown as an indeterminate spinner + text
("Loading pose model…"), not a fake percentage — Core ML's `MLModel(contentsOf:)`
gives no progress callback, so a determinate bar would have to be fabricated,
which the task's own honesty requirement rules out.

## Preprocessing — derivation and verification

`CONTRACT.md` (`labs/sam-3d-body/export/CONTRACT.md`) did not exist when this
was written (checked at the start of this task and again before finalizing).
Every geometric formula in `SAM3DPoseEstimator` was derived from the released
Python source and then **empirically verified** by running the actual PyTorch
functions in `labs/sam-3d-body/.venv` against this file's closed-form Swift
formulas on concrete numeric examples:

- `GetBBoxCenterScale` + `TopdownAffine(input_size=(512,512))` — bbox padding
  (1.25), the two-stage `fix_aspect_ratio` square-forcing, and the resulting
  affine warp matrix — verified to match `get_warp_matrix`'s actual
  `cv2.getAffineTransform`-based output exactly (`scale=0.8192, tx=-71.68,
  ty=-14.336` for a synthetic 800×600 image / [250,80,550,580] bbox).
- `prepare_batch.py`'s default `cam_int` (`f=sqrt(h²+w²)`, principal point at
  image center) — this is the path taken with no external calibration and no
  FOV estimator (`fov_estimator=None` prints "Using the default FOV!" and
  `prepare_batch` falls through to this exact formula). **This is a real
  assumption, not just an unverified guess: the alternative in the Python
  source is a full MoGe2 monocular-geometry model
  (`tools/build_fov_estimator.py`), which is obviously out of scope for an
  on-device pipeline and is not hinted anywhere in this task's brief — so the
  no-calibration default is very likely what the model-export agent's
  CONTRACT.md will also specify, but that file should still be checked once it
  exists.**
- `get_ray_condition` (`sam3d_body.py:1027`) — its own code comments claim
  shapes ("B x N x H x W x 2", "B x num_person x 2 x H x W") that are actually
  WRONG for `H != W` inputs (verified: `torch.meshgrid(..., indexing="xy")`
  produces a transposed shape the comment doesn't account for) — resolved by
  tracing the REAL call site, which always passes the FULL SQUARE 512×512
  image (the width crop to 384 happens separately, afterward, via the same
  `[..., 64:-64]` slice applied to both the image tensor and `ray_cond`). At
  that real (square) call shape the derived closed form was verified against
  the library's actual output and matched exactly (< 1e-9) at both corners,
  the center, and two off-center points.
- `_get_decoder_condition`'s `cliff` formula, `[(cx-cx_int)/f, (cy-cy_int)/f,
  b/f]` — verified to equal the frozen contract's stated formula exactly, and
  verified that the `USE_INTRIN_CENTER` true/false branches are numerically
  identical given the default `cam_int` (its principal point equals
  `img_size/2` exactly).

All of this is written up with inline citations in
`SAM3DPoseEstimator.PreprocessingConstants`'s doc comment and each formula's
own comment — cross-check against `CONTRACT.md` once it exists regardless;
this is the highest-value single thing to re-verify before shipping.

## External verification boundaries

- **Pixel-level resampling fidelity.** The affine warp uses
  `CGContext.interpolationQuality = .high`, which is CoreGraphics' best
  available resampling but is not documented to be exactly bilinear; the
  Python reference uses `cv2.warpAffine(..., INTER_LINEAR)`, strictly
  bilinear. **This repo has a confirmed precedent for exactly this class of
  mismatch**: AutoLevel's wiki notes record that a "float→8-bit
  Pillow→Lanczos composite" path was NOT equivalent to iOS CoreGraphics
  `.high` and caused a measurable accuracy regression. This is flagged
  in-code (`renderWarpedRGBA`'s doc comment) as the top candidate for a
  device-side numeric check once frames can actually be run through the real
  model — not something fixable without hardware to measure against.
- The exact Core ML input/output feature names ("image", "ray_map", "cliff",
  "joint_coords", "global_rots", "cam_t", "keypoints_2d") match the frozen
  contract as given to this task; whether the actual exported `.mlpackage`
  declares them with those exact strings is the export agent's responsibility
  and unverified here.
- `MLMultiArray.dataPointer.bindMemory(to: Float16.self, ...)` for filling
  freshly-created input arrays, and `array[[NSNumber...]]` for reading
  (presumably Float32) output arrays — standard, long-stable Core ML Swift
  APIs, but never compiled in this pass.
- `VNDetectHumanRectanglesRequest` behavior on real photos (confidence
  ranking, bbox tightness) — Vision framework usage is standard but untested
  here.
- The `RealityKit` `ARView(cameraMode: .nonAR, ...)` / `PerspectiveCamera` /
  camera auto-framing math — geometrically reasoned, not rendered.
- The GCD serial-queue ordering argument for `scaleModel` → `processFrame` and
  the `objectWillChange` + 30ms-settle waiter are exercised on the simulator
  against the real engine. Their device scheduling/performance still needs the
  device verification recorded in STATUS.
- Whether a single Core ML model instance loaded with `.cpuAndGPU` safely
  serves sequential (never concurrent) `prediction(from:)` calls from a
  background serial queue the way this file assumes — standard usage, not
  device-tested.
