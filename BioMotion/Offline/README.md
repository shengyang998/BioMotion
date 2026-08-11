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
cannot change that model/solver capability. A separate clip-level camera
reference gate is already wired for a future capability-valid model, but its
production calibration profile is intentionally nil. The runner resolves
contact capability first: the bundled models skip the camera pass as
`.unmeasured`, while a future contact-valid model without that exact profile
would stop at `.calibrationUnavailable` without opening the native reader.
Provisional image thresholds are never treated as evidence.
The path is covered by the ordinary test gate; real camera fixtures, real-video
adapter coverage, and device UI/performance/cancellation remain external
boundaries called out below.

## Flow

```
OfflineImportView (PhotosPicker: photo or video)
  -> picker provider video is copied synchronously into one private 0700
     UUID directory; an AppOwnedTemporaryVideo owns that copy
  -> OfflineSessionRunner.run(source:samplingMode:)
       -> poseEstimator.loadModelIfNeeded()          [SAM3DPoseEstimator]
       -> wait for nimble.isModelLoaded               [poll, ≤10s]
       -> snapshot contact capability
       -> nimble.resetSessionState()                  [new clip: clear IK/QP and ground provenance]
       -> CameraReferenceAnalysisAdmission            [contact FIRST]
          photo/single-frame -> .notRequiredForSingleFrame
          contact unavailable -> .unmeasured          [skip native reader]
          calibration unavailable -> .calibrationUnavailable [skip native reader]
          only contact-valid + exact versioned profile:
            CameraMotionVideoAnalyzer
              require exactly one video track (multi-track is indeterminate)
              require requested span <= 4s and <= 1000 actual native samples
              decode upright bounded frames at actual SOURCE PTS
              derive continuity/endpoint tolerance from actual cadence
              exclude the person; register quality-screened background tiles
              reduce translation/rotation/scale to ONE clip-level state
       -> finalize OfflineResultStore.cameraReferenceState before any frame
          and map it to NimbleEngine.cameraDynamicsAuthorization
       -> FrameSource.decodePhoto / VideoDecoder       [decode]
       -> for each frame:
            SAM3DPoseEstimator.estimate(uiImage:)      [Vision bbox -> warp -> CoreML predict]
            MHRRetarget.makeBodyFrame(jointCoords:)    [-> pelvis-pinned BodyFrame
                                                         + .mhrRootRelative provenance]
            retain raw camT on FrameResult             [finite/positive-depth structure check;
                                                         camera-relative overlay position only]
            (first successful frame only) nimble.scaleModel(...)  [MHRRetarget.segmentScaleMarkers/estimatedStatureMeters]
            nimble.processFrame(bodyFrame, exact offline lease)
              + await that exact receipt, or fail closed
            [IK remains eligible; unauthorized dynamics stop before solveIDGRF]
            -> OfflineResultStore.append(...)
       -> edge-pad trusted requested endpoints so the centred Savitzky-Golay
          window can produce results for the first/last real frames
          [filter support only; replay is not independent ground evidence]
       -> if the clip has usable product timing:
            snapshot hasValidatedFootContactSupport
            future validated-contact branch only:
              also require clip camera state to permit TEMPORAL dynamics
              require each solver derivative window to carry one
                gravity-aligned, dynamics-qualified root reference
              resultStore.beginGaitReplacementPass() before analysed publication
            publish detached GaitTimingReport
            guard the capability snapshot
            [both bundled models return here; no plan and no second pass]
            future validated-contact branch continues:
              apply load-only refusal gates and build the private gait plan
              nimble.resetAnalysisPassStatePreservingGround()
              re-submit the same clip
       -> release exact OfflinePolicyLease on every terminal path
            -> enqueue before notification: replay LIVE recipe/defaults
            -> then clear bridge/QP/filter/ground state in that FIFO block
  -> OfflinePlaybackView (RealityKit .nonAR ARView + MuscleOverlay + scrubber)
```

### Picked media has explicit ownership and latest-selection-wins semantics

A `PhotosPicker` file URL is borrowed from the transfer provider; it is not a
stable application document. The import-only `FileRepresentation` therefore
finishes a synchronous copy before its closure returns. Each video gets a new
mode-0700 `biomotion-import-<UUID>` directory under the system temporary
directory. `AppOwnedTemporaryVideo` retains that directory through the view,
`RunSource`, both video decoders and their asynchronous size-cap work. Its final
reference removes only that private directory, never the provider's file.
Replacement, failed copy, and normal view/run teardown are covered. As with any
`deinit` cleanup, force-kill or process crash cannot run it; the remaining
system-temporary directory is an operating-system/manual-cleanup boundary, not
durable user data.

Picker loads use a monotonic generation. Starting B advances the generation
before cancelling A, so late success, failure, or cancellation from A cannot
replace B or clear B's loading state. A failed or cancelled B retains the last
usable photo/video for retry; only a successful B atomically replaces it. The
picker remains available while a selection is loading, is disabled during an
analysis, and Run is disabled during either loading or analysis.

Cancelling an active analysis snapshots its attempted counts, fences its exact
runner/engine ownership, and clears partial playback. It deliberately retains
the imported selection so the user can retry. Cancelling while idle leaves a
completed playback store intact. Both the engine-release notification and the
store reset can synchronously start a successor, so the cancelling invocation
must still be latest before and after either notification; an old Cancel never
clears or overwrites that successor.

`BodyFrame` keeps a stable joint id separately from its source-specific OpenSim
marker. Live `hips_joint` defaults to `PELVIS`; MHR `hips_joint` carries
`MHR_ROOT`. Filters and test-fixture transformations preserve that override,
and the engine resolves it only after the stable-id whitelist succeeds.

## Design decisions and why

### Camera reference is clip-level evidence, not subject motion

`CameraReferenceState` answers whether an imported image sequence supplies a
calibrated stationary image reference for the requested solve class. It does
not describe the subject; per-frame `MotionState` / `MotionVerdict` still does
that. The runner resolves the camera state before it appends the first
`FrameResult`, publishes the state once for the whole clip, and carries the
numeric evidence separately from the pose results. No later frame can silently
upgrade an earlier unknown camera into dynamics.

For multi-frame video, `CameraMotionVideoAnalyzer` has a deliberately separate
reader from the sparsely sampled pose decoder. It requires exactly one video
track; a multi-track asset is indeterminate rather than composited at a
fabricated cadence. Empty edits inside the requested range, a cropped clean
aperture, non-square pixels, or format descriptions with inconsistent encoded
rasters are also refused: the calibration domain is one full encoded raster,
not a best-effort presentation crop. The video
composition names that track as `sourceTrackIDForFrameTiming`, so variable and
native presentation timestamps survive. Its upright render transform also
scales into a fixed, profile-owned maximum pixel budget. The track's
`preferredTransform` must have a finite unit-orthogonal 2x2 linear part within
1e-4 (a mirror is allowed); scale, shear and singular metadata are refused
rather than normalized into the calibrated raster. Changing `renderSize` alone
would crop rather than scale, while analysing full-resolution 4K BGRA buffers
would make the rolling snapshot set an unbounded memory cost. The actual output
must be an even BGRA raster no larger than 4096 px on either axis or 4:1 in
aspect, and its real `bytesPerRow * height * 5` retained-buffer cost must fit the
64 MiB adapter budget. Human rectangles are pinned to Vision revision 2 and
translational registration to revision 1; a runtime that lacks either exact
revision fails readiness before media is opened. The revisions, render size and
all resource limits are recorded in the revision-3 calibration fingerprint.

Each native frame first gets a human bounding box. The union of the reference
and target boxes is inflated and excluded, leaving disjoint background tiles;
the moving person is never deliberately used as a camera landmark. Tiles are
admitted only after texture/structure, alignment appearance, and uniqueness
checks. Coverage is recomputed from those FINAL usable tiles, not from the
regions that were merely requested, and the accepted set must contain enough
spatially dispersed tiles to fit a similarity field. Vision results must be a
finite translation-only transform, and every planned tile must return a valid
registration. Every quality-valid tile then participates in one fit; no
iterative outlier deletion may manufacture a static answer from inconsistent
motion. That fit exposes image translation, in-plane rotation, and scale
(zoom/push-in); adjacent-frame fits catch jumps and short-lived anchors
accumulate slow sub-pixel drift.

Texture and ambiguity have fixed render-pixel semantics. Both frames are
box-averaged on an 8 px lattice; normalized reference-to-target correlation is
sampled every 32 px over the complete +/-48 px two-dimensional residual surface
around Vision's candidate. Every candidate uses the same inward-cropped sample
domain; a tile is screened for complete support before Vision runs, and sampled
coverage is recomputed after narrow strips are removed or feasible side regions
are split. A second exhaustive screen visits every 8 px lattice offset whose
overlap contains at least 64 box samples, including periods outside the local
window and offsets with arbitrary scale remainders. Four sign quadrants use
fixed matched 8x8 domains; narrow tails use an exact 48-sample matched domain.
Each remote correlation is compared only with zero lag on those identical
samples. One zero-lag product integral costs `width * height` pairs, every
shifted product is counted explicitly, and local plus global work must stay at
or below 500,000 pairs. The 48x48-box reference case costs exactly 496,889.
Larger regular regions are carved into deterministic, globally phase-aligned
48x45-box leaves; smaller irregular regions use bounded long-axis recursion.
Either path is limited to 16 leaves and never silently samples more coarsely.
Planning also rejects any narrow grid whose broad alias rectangle does not
wholly contain the fine +/-48 px candidate square; candidate counts are set
differences, so matching totals alone are not sufficient.
The candidate must be the
local global peak, its absolute normalized correlation must meet the provisional
0.5 floor, and either a local or remote matched-domain alias lowers uniqueness.
Thus a texture with an 80 px period is refused even though its equal peak lies
outside +/-48 px. The box/grid/search/separation, global overlap/domain values,
cost cap, correlation floor, pinned Vision revisions and adapter revision are
fingerprinted calibration inputs, not tile-area-dependent performance
shortcuts. Weak, inconsistent, foreground-dominated, aliased, repetitive or
over-budget evidence becomes `.indeterminate`, never zero motion.

The reducer consumes the actual sample PTS. It requires every interval to be
valid and continuous, positive bounded sample durations, the first and last
native samples to cover the requested range endpoints, and at least one
complete derivative window. Its gap/endpoint allowance comes from the robust
actual PTS cadence and is capped by the versioned profile; nominal frame rate
cannot widen it. EOF is accepted only when the reader itself reports
`.completed`; failed, cancelled, still-reading, and unknown terminal states
stay distinct and fail closed.
Checking merely one good interior window is insufficient: an unread head or
tail, decoder failure, timestamp gap, or cancellation cannot stand in for the
whole requested clip. The resulting state is one of measured static, measured
moving, between calibrated bands, calibration required, or an explicit
indeterminate reason.

Admission also bounds cost independently of pose sampling: the requested
native span is at most 4 seconds and the adapter stops at 1000 actual samples.
The calibration fingerprint binds the adapter revision, analysis dimensions,
motion bands, cadence/window domains, person/tile policy and fit thresholds;
changing any of them invalidates readiness. Both the runner and analyzer /
reducer boundary check the typed profile, so a direct caller cannot turn a
non-empty string into calibration authority. At the reducer boundary, a
structurally valid fingerprint that differs from the running adapter is
reported as `.calibrationRequired`; non-positive resource and alias-count
fields are structurally invalid measurements. Cadence-domain comparisons
allow only a `1e-12 s` representation tolerance, enough for native-PTS
subtraction at 120/240 fps without materially widening the calibrated time
domain; a larger mismatch still requires calibration.

Permissions differ by solve class:

- A photo or explicit single-frame import is
  `.notRequiredForSingleFrame`. With a future contact-capable model it may
  authorize an explicitly static-equilibrium solve, because there is no
  temporal camera path to measure. It never authorizes gait/temporal dynamics.
- Only `.staticWithinBudget` authorizes temporal dynamics. It may also
  authorize static equilibrium.
- `.unmeasured`, `.moving`, `.betweenCalibrationBands`,
  `.calibrationRequired`, `.calibrationUnavailable`, and every
  `.indeterminate` reason authorize neither.

The order is intentional. Validated foot-contact support is tested first,
because a camera cannot repair an absent support model; both bundled models
therefore continue to report `.contactSupportUnavailable`. For a future model
that passes contact capability, the runner maps the state to
`CameraDynamicsAuthorization`. `NimbleEngine.processFrame` captures that value
and the frame's `DynamicsReference` before crossing to `solverQueue`, and
`dynamicsPreflightAvailability` refuses before `solveIDGRF`. The complete order
is contact capability, camera authorization, gravity alignment, then (for a
temporal solve) dynamics-qualified root trajectory. `OfflineResultStore`
independently applies the same order when projecting a complete solve generation and
purges ID/muscle on a downgrade. That second check is a stale-payload safety
boundary, not the mechanism that makes an unauthorized solve acceptable. Its
frames, capability, camera state, gait and selection are ordinary stored state:
each method commits a complete snapshot and then sends one notification, so a
synchronous observer cannot see denied authorization beside old loads or let an
older setter overwrite a reentrant reset. Static-equilibrium provenance follows
the retained ID solve even when the optional muscle solve is absent.

`Configuration.production` has no typed calibration profile until the exact
implementation, dimensions, cadence/window domain and fingerprint pass
versioned tripod/static controls plus held-out moving-camera fixtures. The
runner therefore does not run this expensive pass in production today: a
future contact-valid multi-frame path would finalize
`.calibrationUnavailable` with no numeric evidence. This is the intended safe
state. Visible-background registration does not prove absolute physical camera
translation, and uncalibrated thresholds do not become truth because a clip
looks steady.

### Camera-relative position is not an inertial root trajectory

SAM 3D Body emits `cam_t` in metres in an OpenCV camera frame. The exact
Y-up translation used by `MHRRetarget` is `(x, -y, -z)`, and the pure
composition remains useful and tested. It establishes only where the body sits
relative to the image camera. It does not establish physical gravity, a floor,
camera acceleration, or a depth channel safe to differentiate twice.

`BodyFrame.DynamicsReference` keeps those claims separate:

- `.liveARKit` records gravity alignment and global position, but intentionally
  remains position-only for temporal dynamics until source-specific root
  continuity/noise evidence is calibrated.
- `.dynamicsQualifiedWorld` is the explicit future/test authority for a
  gravity-aligned root trajectory whose derivatives have been qualified.
- `.mhrRootRelative` is the current production offline solver frame: metric
  relative pose, pelvis pinned, no synchronized gravity reference.
- `.mhrCameraRelativePosition` means raw `cam_t` was composed for position/IK;
  it still permits neither static nor temporal dynamics without a gravity
  transform, and its root remains position-only for temporal work.
- the initializer default is `.unmeasured`, so legacy/adversarial callers do
  not inherit live authority accidentally.

The Photos runner intentionally continues to use pinned markers plus raw
`cam_t` for projection. Passing `cam_t` into `makeBodyFrame` without changing
the overlay would apply the translation twice. Production activation therefore
waits for one atomic path that supplies synchronized camera-to-gravity evidence,
a calibrated camera/reference decision, a pre-registered depth-drift and root
noise policy, continuity resets, and physical-device ground truth. A derivative
window must carry one unanimous qualified reference; one newer trusted frame
cannot authorize older samples. Non-finite or structurally out-of-domain
`joint_coords`, wrong-rank/non-finite/out-of-domain `cam_t`, non-positive
`cam_t.z`, non-finite retarget arithmetic, and non-finite projection results
are rejected before native IK/display. The broad 10 m source-joint and 1,000 m
camera-translation ceilings are numeric safety domains, not physical
validation; the separate stature/hip gate remains the human plausibility test.

### Backpressure (constraint: never submit frame N+1 while N is in flight)

`NimbleEngine.processFrame` synchronously returns `.accepted(FrameReceipt)`,
`.dropped`, or `.rejected`. The receipt combines reset generation with an
accepted-only monotonic submission id. A dedicated completion publisher sends
`.published`, `.failed`, or `.superseded` for that exact receipt; ordinary
`objectWillChange` is never interpreted as frame completion. A successful
event is sent only after every published field is coherent, the physical
solver occupancy is released, and `lastSolveReceipt` names the same snapshot.
Those result fields are ordinary stored state committed as one main-thread
transaction, followed by one explicit UI notification; a Combine subscriber
cannot reset midway through fifteen independent `@Published` setters and then
let the superseded closure write its remaining fields or recording history.
Full-session ground clearing is inside the same reset transaction.

Publication authority and physical solver occupancy are separate. Timeout or
cancellation synchronously supersedes the exact publication and bumps its
generation before resuming the waiter, but a non-cancellable GCD solve keeps
the physical occupancy until it really returns. Later frames are dropped and
retried rather than queued behind a stuck solve, so cancellation cannot leave a
backlog delaying the live path. Engine-global `OfflinePolicyLease` also prevents
a second Runner or live producer from inserting work, resetting the batch, or
restoring policy over the current owner; queued AR tracking-loss callbacks are
also ignored while the offline sheet is active. Runner acquisition, phase
publication, release and task-defer cleanup recheck or retire their exact local
token/lease before any synchronous engine notification. A lifecycle invocation
epoch is registered before predecessor release, so a synchronously reentrant
newer Run or Cancel wins instead of being overwritten by the older call as its
stack unwinds. Segment reset and model scaling also present the captured engine
lease and stop if reset notification transfers ownership. An observer-started
successor therefore cannot lose its lease, phase, cancellation handle or local
segment state. Offline subject geometry is owned by that same lease. Only a
successful live native scale updates the value-only live recipe; exact release
enqueues recipe/default restoration followed by bridge, QP, filter and ground
cleanup before it notifies observers. A stale release is a complete no-op, and
any reentrant successor's scale/solve is therefore FIFO after the entire block.
Head/tail padding returns
failure to its caller and never increments the push count for an unpublished
receipt. A real timeout, native IK failure, admission refusal, exhausted busy
retry, and an externally superseded session remain different frame statuses and
different user messages.

`OfflineOrchestrationTests` exercises the same waiter against the real engine;
the exact cancellation/timeout scheduling still needs a healthy Simulator and
device receipt as called out below.

### 9-frame Savitzky-Golay warm-up is not ground calibration

`SavitzkyGolayFilter` (`BioMotion/Nimble/SavitzkyGolayFilter.swift`) needs 9
pushes before it emits a centred derivative window. That is a filtering
requirement, not evidence of foot support and not permission to treat replayed
poses as new observations of the floor. This means:

- A `DynamicsReference` change clears IK/filter/ground history. Until a full
  all-new window exists, the explicit `referenceTransitionWarmup` state clears
  any old coloured dynamics display without falsely claiming that the new
  reference itself lacks gravity or root qualification.
- Before any production source may emit `.dynamicsQualifiedWorld`, its
  reference generation must change on world-origin/relocalization events, or
  those events must force the same explicit session reset. Equal category
  labels alone do not prove two world frames share one origin.

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
  are not owned by the solve and remain unchanged. If a same-generation ID is
  otherwise valid, the projector applies the session gates in the same order as
  the engine: contact capability, clip camera permission, gravity alignment,
  then solve-class root provenance (required for temporal dynamics). Any
  downgrade strips ID and muscle while preserving pose and report-neutral
  motion evidence.
  Starting the gait replacement pass first clears every eligible pass-one ID,
  muscle, and static flag to `.analysisPassIncomplete`; each successful
  same-generation gait solve then replaces that marker. A timeout or missing
  centred publication therefore cannot leave static physics under a running
  result.

### Contact, camera, spatial reference, then ground trust

The boundaries answer different questions. Contact support says how a foot may
transmit force. Camera authorization says whether image derivatives share a
calibrated stationary reference. `DynamicsReference` separately says whether
marker axes follow physical gravity and whether the requested solve class has a
qualified root trajectory. Ground provenance says where the floor is. Both
bundled models have empty `ContactGeometrySet`s, and the active solver does not
impose a validated support domain itself, so the permanent contact capability
is checked first and both bundled models remain pose-only. An explicit floor, a
calibrated camera, a qualified coordinate label, 30 observations, a session
reset, or a second pass cannot unlock a missing capability.

For a future model/solver pair that does define validated support, the requested
solve class must next be authorized by the clip camera state, then by gravity
provenance and—only for temporal dynamics—a dynamics-qualified root trajectory
across the whole derivative window. Those checks all refuse upstream of
`solveIDGRF`. The rolling ground estimator remains a necessary later gate. An
external caller may pin an explicit ground height; that source is trusted
immediately and observed
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
a different person. That imported scale lives only for the exact offline lease:
normal completion, Cancel and Close restore the last successful live recipe, or
the loaded defaults when live calibration was skipped. It never becomes the
next live subject's geometry.

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

The playback camera derives its orbit centre and default radius once from the
first displayed frame with tracked joints after the 3-D view is created; that
baseline is not recomputed while scrubbing. One-finger drag orbits, pinch zooms
over a bounded radius, and double-tap restores the auto-framed starting view.
That initial view comes from the joint bounding box rather than a hardcoded
playback position —
`MHRRetarget.swift` documents that `joint_coords` pins the raw MHR source root
at a model-constant `(0, 0.924, 0)` in every prediction, not at a real-world
camera distance, so a hardcoded position could easily show a blank screen.
These gestures move only the non-AR review viewpoint; they neither measure nor
change the source recording camera and grant no dynamics authorization.

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

Camera-reference cadence is intentionally independent of that pose budget.
After model/contact and calibration admission, an eligible multi-frame video is
read across the same requested time range at the selected source track's actual
sample PTS. A 2 fps pose run therefore does not sample a pan at 2 fps, and a
120/240 fps or variable-rate source is not relabelled with `nominalFrameRate`.
Nominal rate helps choose the request/window only; robust actual PTS cadence,
positive sample duration, continuity and endpoint coverage decide admission.

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

Every run begins in `.loadingModel`, because the model's validated contact
capability is the higher-priority admission boundary. Only a contact-valid,
versioned-calibration multi-frame source later enters
`.checkingCameraReference`; bundled models and an absent calibration skip that
phase entirely. Neither phase claims a percentage it cannot measure. Photos and
explicit single-frame imports resolve their non-temporal policy without opening
a video reader.

`MLModelConfiguration.computeUnits = .cpuAndGPU` (not `.all`) per this task's
explicit constraint. Loading happens on a dedicated background queue via a
checked continuation. `AssetPackModelStore` resolves only a precompiled
`SAM3DBodyPose.mlmodelc`: first the optional developer-bundled copy, then the
Managed Background Assets pack. It never accepts or compiles a raw `.mlpackage`
at runtime. If neither copy is local, it starts or joins the system-managed
download and throws promptly with the observed progress or real failure reason;
the UI publishes that failure state without crashing or silently skipping the
model. Core ML loading itself remains indeterminate because
`MLModel(contentsOf:)` exposes no progress callback.

## Preprocessing — derivation and verification

The exact `CONTRACT.md` revision is pinned at SHA-256
`6aa70b392b750bcfb4c1695b88fca336a13d284721d1689383427a5654ca5f47`
by `BioMotion/Resources/SAM3DBodyPose.lock.json`. The Swift interface, shapes,
dtypes, bbox/camera formulae, axes, `cam_t`, and crop-space keypoints have now
been checked field by field against that frozen revision. Independently, every
geometric formula in `SAM3DPoseEstimator` was derived from the released Python
source and **empirically verified** by running the actual PyTorch functions in
`labs/sam-3d-body/.venv` against this file's closed-form Swift formulas:

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
  on-device pipeline. The frozen contract explicitly requires this same
  no-calibration default.**
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
own comment. The locked contract cross-check and the independent Python
derivation now agree; pixel-level resampling remains the separate boundary below.

## External verification boundaries

- **Camera-reference calibration is not installed.** Keep
  the production typed profile/fingerprint nil until the exact reader, upright
  analysis resolution, cadence/window domain, tile-quality policy, and reducer pass tripod
  static controls plus held-out pan, tilt, roll, lateral translation,
  push-in/pull-out, zoom, and slow sub-pixel drift fixtures. Calibration and
  held-out clips must be disjoint, the profile must identify its allowed
  lens/resolution/frame-rate domain, and the accepted false-static risk must be
  recorded rather than inferred from three convenient videos.
- **The real adapter still needs adversarial video coverage.** Run the complete
  `AVAssetReader` + Vision path on the owner's clips and on 30/60/120/240 fps
  plus VFR sources, different orientations/resolutions/lenses/lighting, head and
  tail decode loss, low-texture walls, one-directional or repetitive patterns,
  exposure/rolling shadows/reflections, multiple people, and independently
  moving background objects. Pure reducer/geometry tests cannot characterize
  Vision registration or prove source endpoint coverage on those media.
- **Camera-pass device cost and cancellation remain unmeasured.** Verify peak
  decoder/pixel-buffer memory stays within the profile-owned pixel/byte/shape
  budget, that the 4-second/1000-native-sample caps hold on device, and that cancelling
  during reader/Vision work promptly leaves no late clip-state or solve
  authorization behind. Simulator pixel formats and scheduling are not a
  substitute for this receipt.
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
- The lock records and the verifier checks the real source/compiled Core ML
  interface: `image`, `ray_map`, and `cliff` are fixed-shape Float16 inputs;
  `joint_coords`, `global_rots`, `cam_t`, and `keypoints_2d` are fixed-shape
  Float32 outputs. The Swift output parser now rejects any dtype drift before
  reading values, and rejects non-finite joints, rotations, camera translation,
  and keypoints. Finite crop-external keypoints remain unclamped as required.
- Frozen `CONTRACT.md` §2.2 labels `global_rots` “world→joint” but immediately
  defines `R @ v_local = v_world`, which is joint-local→world. The Swift matrix
  layout follows the explicit equation and does not re-orthonormalise; no current
  product path consumes these rotations. The locked document cannot be edited
  alone without invalidating its lock and artifact receipts, so the export owner
  must correct and republish that prose before rotations become product input.
- The Core ML array APIs and the real names/shapes/dtypes compile in the app and
  are pinned by the artifact lock. Loading the shipping Managed Background
  Assets copy still needs the real-device/TestFlight receipt described in STATUS.
- `VNDetectHumanRectanglesRequest` behavior on real photos (confidence
  ranking, bbox tightness) — Vision framework usage is standard but untested
  here.
- The `RealityKit` `ARView(cameraMode: .nonAR, ...)` / `PerspectiveCamera`
  auto-framing plus orbit, pinch and double-tap interaction is implemented and
  geometrically reasoned, but has not been rendered or interaction-tested on a
  Simulator or device.
- The GCD serial-queue ordering argument for `scaleModel` → `processFrame`,
  exact receipt completion, timeout/cancellation supersession, physical
  occupancy, and engine-global offline lease have compile/link and pure/source
  contracts. Their real scheduling still needs the focused Simulator and
  device verification recorded in STATUS; no `objectWillChange` or settle-delay
  inference remains in the product runner.
- Whether a single Core ML model instance loaded with `.cpuAndGPU` safely
  serves sequential (never concurrent) `prediction(from:)` calls from a
  background serial queue the way this file assumes — standard usage, not
  device-tested.
