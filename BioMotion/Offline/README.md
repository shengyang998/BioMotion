# Offline import (photo/video → muscle result)

Adds a second input path alongside live ARKit tracking: pick a photo or video
from the library, run the frozen `SAM3DBodyPose` Core ML model over sampled
frames, feed the existing IK → ID → muscle pipeline, and scrub through the
result in a non-AR 3D view. Written entirely against
`labs/BioMotion/BioMotion/Offline/*.swift` — none of it has been built or run
(see "What could not be verified" below).

## Flow

```
OfflineImportView (PhotosPicker: photo or video)
  -> OfflineSessionRunner.run(source:samplingMode:)
       -> poseEstimator.loadModelIfNeeded()          [SAM3DPoseEstimator]
       -> wait for nimble.isModelLoaded               [poll, ≤10s]
       -> nimble.resetSessionState()                  [clip boundary — NEW method, see integration diff]
       -> FrameSource.decodePhoto / VideoDecoder       [decode]
       -> for each frame:
            SAM3DPoseEstimator.estimate(uiImage:)      [Vision bbox -> warp -> CoreML predict]
            MHRRetarget.makeBodyFrame(jointCoords:)    [-> BodyFrame, another agent's file]
            (first successful frame only) nimble.scaleModel(...)  [MHRRetarget.segmentScaleMarkers/estimatedStatureMeters]
            nimble.processFrame(bodyFrame) + wait for NimbleEngine's next publish, or timeout
            -> OfflineResultStore.append(...)
       -> if total pushes < 9: replay the last pose to warm up the
          Savitzky-Golay filter (see "9-frame warm-up" below)
  -> OfflinePlaybackView (RealityKit .nonAR ARView + MuscleOverlay + scrubber)
```

`SAM3DPoseEstimator`, `FrameSource`, `OfflineResultStore`, `OfflineSessionRunner`,
`OfflineImportView`, `OfflinePlaybackView` are all new files under this
directory. `MHRRetarget.swift` (same directory) is owned by another agent —
read, not edited; its actual on-disk signatures matched what this file set
assumed exactly (`makeBodyFrame(jointCoords: [SIMD3<Float>], timestamp:,
frameNumber:) -> BodyFrame`, `segmentScaleMarkers(jointCoords:) ->
(positions: [Float], names: [String])`, `estimatedStatureMeters(jointCoords:) ->
Float`).

## Design decisions and why

### Backpressure (constraint: never submit frame N+1 while N is in flight)

`NimbleEngine.processFrame` drops a frame outright if a solve is in flight, and
has no per-frame completion callback — every result lands via
`DispatchQueue.main.async` inside `publishResults`, itself only called when
`solveIK` succeeds. `OfflineSessionRunner`'s private `NimbleFrameWaiter` calls
`nimble.processFrame(_:)` then awaits the engine's next `objectWillChange`
(Combine), with a fixed 6s timeout treated as a failed frame. See the doc
comments on `NimbleFrameWaiter` and `OfflineSessionRunner` for the full
reasoning, including why a 30ms settle delay after the first
`objectWillChange` is added (it fires from inside a `@Published` property's
`willSet`, before the engine finishes writing the ~12 fields `publishResults`
sets) — this reasoning is logical/documented but **not verified on a device**.

### 9-frame Savitzky-Golay warm-up

`SavitzkyGolayFilter` (`BioMotion/Nimble/SavitzkyGolayFilter.swift`) needs 9
pushes before it emits ANYTHING, and ID/muscle output only exists once it's
warmed up. This means:

- **A single photo can never produce a muscle result through 1 push.** The
  task brief requires "a still photo is one frame and must work end to end"
  and requires the pipeline to "show the muscle result" — the only way to
  reconcile those is to replay the same pose multiple times.
- `OfflineSessionRunner.padToWarmUp` replays the LAST successfully estimated
  pose with synthetic, evenly-spaced (1/30s) timestamps until 9 total pushes
  have been made, whenever a run ends with fewer than 9. Because every padded
  push is an IDENTICAL pose, the SG filter's velocity/acceleration
  coefficients (which sum to zero for a constant input by construction —
  verified: `[86,-142,-193,-126,0,126,193,142,-86]` sums to 0) come out at
  ~0 regardless of the exact spacing, giving a physically meaningful
  **static-hold** muscle-activation estimate (the effort needed to hold that
  exact pose against gravity) rather than nothing.
- This is surfaced honestly, not silently: `OfflineResultStore.FrameResult`
  carries `isStaticHoldEstimate`, and `OfflinePlaybackView` labels it "Pose +
  static-hold muscle estimate" instead of implying continuous dynamics were
  measured.
- The padding rewrites the ORIGINAL frame's stored result in place
  (`OfflineResultStore.updateBiomechanics`) rather than appending a phantom
  extra scrubber row.

### Calibration without a T-pose

Live tracking calibrates from a 60-frame T-pose hold (`CalibrationView`).
There's no equivalent live capture for an imported clip, so
`OfflineSessionRunner` scales the model from the FIRST successfully-estimated
frame via `MHRRetarget.segmentScaleMarkers`/`estimatedStatureMeters` (both
pose-invariant chain-sum derivations per that file's own extensive doc
comments), then feeds that same frame through `processFrame` normally. This
makes offline import self-sufficient — it doesn't depend on the app having
gone through live calibration first, and works even if the imported subject is
a different person.

`nimble.scaleModel` has no completion signal either. It's called immediately
before `processFrame` with no `await` between them; both dispatch onto
`NimbleEngine`'s private SERIAL `solverQueue`, which preserves FIFO submission
order, so the scale operation completes before that frame's IK solve begins.
This is a documented, reasoned argument about GCD queue ordering — **not
something a build could fully re-verify either, but it rests on public,
stable GCD serial-queue semantics rather than private timing behavior.**

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
`joint_coords` pins the pelvis at a MODEL-CONSTANT `(0, 0.924, 0)` in every
prediction, not a real-world camera distance, so a fixed camera position could
easily show a blank screen. No orbit/manual camera control is implemented
(out of scope for this pass — noted as a natural follow-up, not added to avoid
scope creep).

### Sampling and progress

Default 2fps, user-adjustable 0.5–10fps, plus an explicit single-frame mode
(auto-forced for photos). `FrameSource.maxFramesPerRun = 120` caps a
pathological long-clip × high-fps selection; `OfflineSessionRunner` surfaces
`frameBudgetNotice` rather than truncating silently — a `FrameBudgetNotice`
that names WHICH of the two causes fired (the clip is shorter than the window,
or the native-rate window exceeded the 601-frame run budget) and how many frames
were really used. At 240 fps the cap spans about 2.5 s even when the clip itself
is exactly as long as the 4 s configured window; neither the notice nor the mode
selector describes that as a long clip. The selector's copy lives in
`FrameSource.nativeWindowDisclosure`, beside the arithmetic it discloses. It replaced a
single boolean whose one sentence stated both causes wrongly on a short clip. Progress ETA
(`OfflineSessionRunner.eta`) is derived from the running average of measured
per-frame wall time (`perFrameDurations`), shown as "estimating time…" (not a
fabricated number) until at least one frame has completed.

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

## What could not be verified (no build, no device)

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
  the `objectWillChange` + 30ms-settle argument for `NimbleFrameWaiter` (both
  documented above) rest on public, stable Dispatch/Combine semantics, but
  neither was exercised against the real `NimbleEngine`/`NimbleBridge` binary.
- Whether a single Core ML model instance loaded with `.cpuAndGPU` safely
  serves sequential (never concurrent) `prediction(from:)` calls from a
  background serial queue the way this file assumes — standard usage, not
  device-tested.

## Integration diffs needed (not applied here — see this task's structured output)

1. `NimbleEngine.swift` — add a `resetSessionState()` method (calls the
   existing `resetRealtimeState()` plus `bridge.resetSessionState()` on
   `solverQueue`). `NimbleEngine` currently exposes no wrapper for
   `NimbleBridge.resetSessionState`, only its own Swift-side filter reset.
2. `ContentView.swift` — a `showOfflineImport` state var, a button in
   `trackingView`'s top bar, and a `.sheet` presenting
   `OfflineImportView(nimble: nimble, onDismiss: ...)`.

**No `project.yml` / `Info.plist` change is believed necessary.** `PhotosPicker`
(PhotosUI) does not require `NSPhotoLibraryUsageDescription` for either images
or videos — it runs out-of-process like a share sheet and only grants access to
the specific selected item(s), which Apple documents explicitly. The new
`BioMotion/Offline/*.swift` files should be picked up automatically by the
existing `sources: [BioMotion]` folder-source entry in `project.yml` (XcodeGen
recursively includes a whole-folder source) once `xcodegen generate` re-runs;
the same mechanism is already how `BioMotion/Resources/FullBody.osim` reaches
the bundle today with no special-casing, which is why the coming
`SAM3DBodyPose.mlpackage` drop into `BioMotion/Resources/` is expected to need
no project.yml change either — flagged here for the orchestrator to confirm
once the file actually lands and a real `xcodegen generate` can be run.
