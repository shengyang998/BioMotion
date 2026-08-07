import Foundation
import Combine
import QuartzCore
import UIKit

/// Drives the offline (photo/video import) batch: decode frames -> Core ML pose
/// estimate -> MHR retarget -> NimbleEngine IK/ID/muscle -> OfflineResultStore.
///
/// # Backpressure (design constraint — see this file set's task brief)
/// `NimbleEngine.processFrame` DROPS a frame outright if a solve is already in
/// flight (`NimbleEngine.swift`: `isFrameInFlight -> droppedFrameCount += 1 ->
/// return`). That is correct for live camera input and would silently discard
/// most of a batch run. `NimbleEngine` exposes no per-frame completion callback
/// — every result lands via `DispatchQueue.main.async` inside `publishResults`,
/// which is itself only called when `solveIK` succeeds. So this runner NEVER
/// calls `processFrame` again until it has observed the engine's next publish
/// (via `objectWillChange`, since `@Published` gives no per-property completion
/// signal either) OR a generous timeout has elapsed — see `NimbleFrameWaiter`.
/// A timeout is treated as a failed frame, not a hang.
///
/// # Clip boundaries
/// `resetSessionState()` (added to `NimbleEngine` — see this task's
/// `integration_diffs_needed`, NOT editable directly here) is called once before
/// each run, dropping the bridge's IK warm-start pose and rolling ground-height
/// window so a new clip never blends with a previous one or the live session.
///
/// # Calibration without a T-pose
/// There is no live T-pose capture for an imported clip. Instead, the FIRST
/// frame that produces a usable pose is used to scale the model via
/// `MHRRetarget.segmentScaleMarkers`/`estimatedStatureMeters` (chain-sum-derived,
/// pose-invariant — see MHRRetarget.swift), then immediately fed through
/// `processFrame` like any other frame. `nimble.scaleModel` has no completion
/// signal either, but it is safe to call immediately before `processFrame` with
/// no `await` between them: both dispatch onto NimbleEngine's own private
/// SERIAL `solverQueue`, which preserves submission order, so
/// `bridge.scaleModelWithHeight` finishes before `bridge.solveIK` for that same
/// frame begins.
///
/// # Static holds only
/// This runner turns on `NimbleEngine.staticHoldGating` for the duration of a
/// run. On this path the pose source zeroes `global_trans`, so the pelvis sits
/// at a model constant in every frame and the body has no global translation:
/// `M·q̈` and the centre-of-mass acceleration would be computed from motion
/// that did not happen (in a squat the pelvis never descends — the feet appear
/// to rise). So frames where the subject was measurably moving get pose and no
/// muscle magnitudes, and frames where they were still are solved as statics.
/// The engine is SHARED with the live ARKit view, whose q̈ IS observable, hence
/// the flag is scoped to the run rather than made unconditional.
@MainActor
final class OfflineSessionRunner: ObservableObject {

    enum RunSource {
        case photo(UIImage)
        case video(URL)
    }

    enum RunPhase: Equatable {
        case idle
        case loadingModel
        case decodingFrames
        case running(current: Int, total: Int, etaSeconds: Double?)
        case finished(processed: Int, failed: Int, cancelled: Bool)
        case failed(String)
    }

    @Published private(set) var phase: RunPhase = .idle
    /// True if the video's requested sampling would have exceeded
    /// `FrameSource.maxFramesPerRun` and was capped — surfaced so the UI can say
    /// so rather than silently processing fewer frames than the user asked for.
    @Published private(set) var wasFrameCountCapped = false

    let resultStore = OfflineResultStore()

    private let nimble: NimbleEngine
    private let poseEstimator: SAM3DPoseEstimator
    private var runTask: Task<Void, Never>?
    private var perFrameDurations: [TimeInterval] = []
    private var calibrated = false
    private var lastSuccessfulFrame: (id: Int, bodyFrame: BodyFrame)?
    /// Identifies the current run so a cancelled predecessor cannot clean up
    /// shared engine state that now belongs to its successor. See `runInternal`.
    private var runToken: UInt64 = 0

    /// Generous upper bound on one `nimble.processFrame` solve. NimbleEngine's
    /// module docs record ~200ms/frame for the shipped FullBody.osim (520
    /// muscles / 171 coordinates) on comparable hardware; this leaves wide
    /// margin for thermal throttling or a cold (non-warm-started) first solve
    /// without letting one stuck frame stall the whole batch indefinitely.
    private static let solveTimeout: TimeInterval = 6.0
    /// How many times to resubmit a frame the engine dropped because an earlier
    /// solve was still in flight. See `submitAndWait`.
    private static let maxSubmitAttempts = 8
    private static let dropRetryDelay: TimeInterval = 0.25
    /// Slack when matching a Savitzky-Golay centre timestamp back to its source
    /// frame. The value is a copy of a real frame timestamp, so this only has to
    /// absorb float representation, not sampling jitter.
    private static let biomechanicsMatchTolerance: TimeInterval = 0.001
    /// Lag of the centred Savitzky-Golay window, in samples
    /// (`SavitzkyGolayFilter.windowSize / 2` = 4). Also the number of synthetic
    /// frames padded onto each end of the clip.
    private static let sgHalfWindow = SavitzkyGolayFilter.halfWindow

    /// Spacing used for the synthetic edge-padding frames. Set from the decoded
    /// clip so the padded samples sit on the SAME cadence as the real ones —
    /// the filter derives `dt` from the window span, so mixing a 1/30 s pad into
    /// a 2 fps clip would corrupt `dq`/`ddq` for every window that straddles the
    /// boundary.
    private var sampleInterval: TimeInterval = 1.0 / 30.0

    init(nimble: NimbleEngine) {
        self.nimble = nimble
        self.poseEstimator = SAM3DPoseEstimator()
    }

    func cancel() {
        runTask?.cancel()
    }

    func run(source: RunSource, samplingMode: FrameSource.SamplingMode) {
        runTask?.cancel()
        phase = .idle
        wasFrameCountCapped = false
        runTask = Task { [weak self] in
            await self?.runInternal(source: source, samplingMode: samplingMode)
        }
    }

    // MARK: - Run

    private func runInternal(source: RunSource, samplingMode: FrameSource.SamplingMode) async {
        resultStore.reset()
        perFrameDurations.removeAll()
        calibrated = false
        lastSuccessfulFrame = nil

        // Scoped to this run — see the class header. `defer` covers every exit
        // path including cancellation, so the shared engine goes back to
        // dynamic ID for the live camera view.
        //
        // Guarded by a token rather than flipped unconditionally: `run()`
        // cancels the previous task and immediately starts a new one, but
        // cancellation is cooperative — the old task keeps going until its next
        // check. Without the guard its `defer` could fire AFTER the new run set
        // the flag, silently leaving the new clip on dynamic ID, which is
        // exactly the failure this gate exists to prevent.
        runToken &+= 1
        let myToken = runToken
        nimble.staticHoldGating = true
        defer { if runToken == myToken { nimble.staticHoldGating = false } }

        phase = .loadingModel
        do {
            try await poseEstimator.loadModelIfNeeded()
        } catch {
            phase = .failed("Couldn't load the pose model: \(error.localizedDescription)")
            return
        }

        guard await waitForNimbleModelLoaded() else {
            phase = .failed("Musculoskeletal model isn't loaded — FullBody.osim may be missing from the bundle, or is still loading. Try again in a moment.")
            return
        }

        // Clip boundary — see this class's header comment.
        nimble.resetSessionState()

        let decoded: [FrameSource.DecodedFrame]
        do {
            decoded = try await decodeFrames(source: source, samplingMode: samplingMode)
        } catch {
            phase = .failed(error.localizedDescription)
            return
        }
        guard !decoded.isEmpty else {
            phase = .failed("No frames could be decoded from the selection.")
            return
        }

        var failureCount = 0
        var totalPushes = 0  // real pushes to nimble.processFrame, including edge padding

        // Median gap between decoded frames, so edge padding lands on the same
        // cadence. A single photo has no gap to measure and keeps the default.
        if decoded.count > 1 {
            let gaps = zip(decoded.dropFirst(), decoded).map { $0.timestamp - $1.timestamp }.filter { $0 > 0 }
            if !gaps.isEmpty {
                sampleInterval = gaps.sorted()[gaps.count / 2]
            }
        }

        for (i, frame) in decoded.enumerated() {
            if Task.isCancelled {
                phase = .finished(processed: i, failed: failureCount, cancelled: true)
                return
            }
            let frameStart = CACurrentMediaTime()
            phase = .running(current: i, total: decoded.count, etaSeconds: eta(remainingFrames: decoded.count - i))

            let succeeded = await processOneFrame(frame, frameIndex: i, totalPushes: &totalPushes)
            if !succeeded { failureCount += 1 }

            perFrameDurations.append(CACurrentMediaTime() - frameStart)
        }

        if Task.isCancelled {
            phase = .finished(processed: decoded.count, failed: failureCount, cancelled: true)
            return
        }

        // Edge-pad the tail. Mirrors the head priming in `processOneFrame`: the
        // centred SG window lags 4 samples, so without this the LAST 4 real
        // frames never reach the middle of a full window. Each of these pushes
        // yields a result centred on a real frame, which
        // `routeBiomechanicsToOwningFrame` files correctly. A single photo
        // (1 real push) is just the degenerate case — 4 head + 1 + 4 tail = 9,
        // centred exactly on the photo.
        if totalPushes > 0, let last = lastSuccessfulFrame {
            await padFilterTail(with: last.bodyFrame, totalPushes: &totalPushes)
        }

        phase = .finished(processed: decoded.count, failed: failureCount, cancelled: false)
    }

    /// Returns true iff the frame produced a `.success` result (pose estimated,
    /// nimble published something for it — muscle/ID may still be nil during SG
    /// warm-up, which is still `.success`).
    @discardableResult
    private func processOneFrame(_ frame: FrameSource.DecodedFrame, frameIndex: Int, totalPushes: inout Int) async -> Bool {
        let estimate: SAM3DPoseEstimator.Output
        do {
            estimate = try await poseEstimator.estimate(uiImage: frame.image)
        } catch {
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
                status: .poseEstimationFailed(error.localizedDescription), usedFallbackBBox: false,
                camT: nil,
                bodyFrame: nil, ikResult: nil, idResult: nil, muscleResult: nil, isStaticHoldEstimate: false, motionState: .undetermined))
            return false
        }

        let bodyFrame = MHRRetarget.makeBodyFrame(jointCoords: estimate.jointCoords,
                                                   timestamp: frame.timestamp,
                                                   frameNumber: frameIndex)

        guard bodyFrame.joints.contains(where: \.isTracked) else {
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
                status: .poseEstimationFailed("retarget produced no usable joints"), usedFallbackBBox: estimate.usedFallbackBBox,
                camT: estimate.camT,
                bodyFrame: bodyFrame, ikResult: nil, idResult: nil, muscleResult: nil, isStaticHoldEstimate: false, motionState: .undetermined))
            return false
        }

        if !calibrated {
            let stature = Double(MHRRetarget.estimatedStatureMeters(jointCoords: estimate.jointCoords))
            let (positions, names) = MHRRetarget.segmentScaleMarkers(jointCoords: estimate.jointCoords)
            // See this class's header comment for why no completion signal is
            // needed here: solverQueue is serial and FIFO, so this happens-before
            // the processFrame submission immediately following it.
            nimble.scaleModel(height: stature, markerPositions: positions, markerNames: names)
            calibrated = true
        }

        // Edge-pad the head of the sequence. The SG filter is CENTRED, so
        // without this the first 4 real frames can never sit at the middle of a
        // full window and would show pose with no muscle. Replaying this frame
        // backdated fills the leading half-window; the results those pushes
        // produce are centred on synthetic timestamps and get discarded by
        // `routeBiomechanicsToOwningFrame`, which is what we want.
        if totalPushes == 0 {
            await primeFilterHead(with: bodyFrame, totalPushes: &totalPushes)
        }

        let published = await submitAndWait(bodyFrame, timeout: Self.solveTimeout)
        totalPushes += 1
        lastSuccessfulFrame = (frameIndex, bodyFrame)

        guard published else {
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
                status: .nimbleTimeout, usedFallbackBBox: estimate.usedFallbackBBox,
                camT: estimate.camT,
                bodyFrame: bodyFrame, ikResult: nil, idResult: nil, muscleResult: nil, isStaticHoldEstimate: false, motionState: .undetermined))
            return false
        }

        // Biomechanics are deliberately NOT attached here — see
        // `routeBiomechanicsToOwningFrame`. The newest solve does not describe
        // the newest frame.
        resultStore.append(OfflineResultStore.FrameResult(
            id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
            status: .success, usedFallbackBBox: estimate.usedFallbackBBox,
            camT: estimate.camT,
                bodyFrame: bodyFrame, ikResult: nil, idResult: nil,
            muscleResult: nil, isStaticHoldEstimate: false, motionState: .undetermined))
        routeSolveToOwningFrame()
        return true
    }

    /// Files the newest complete solve against the frame it actually describes.
    ///
    /// `NimbleEngine` dates ID, muscle and the motion verdict at
    /// `centerTimestamp` — the centre of the 9-tap Savitzky-Golay window —
    /// because that is the sample its `dq`/`ddq` are valid for. So the newest
    /// published result is roughly half a window OLDER than the frame just
    /// pushed. Attaching it to the frame we just submitted would label every
    /// overlay with a pose it does not belong to: about 4 frames of lag, which
    /// at the 2 fps default is two seconds of visibly wrong muscle.
    ///
    /// `SolveRecord` carries that timestamp, so match on it. The centre value
    /// is a copy of a previously pushed frame timestamp, so the match is exact
    /// up to float representation; a result with no close frame belongs to a
    /// synthetic head-pad push or a previous clip and is discarded rather than
    /// misfiled.
    ///
    /// Reading `SolveRecord` rather than the individual `@Published` fields is
    /// load-bearing now that a moving frame publishes IK with muscle = nil: the
    /// engine leaves `lastMuscleResult` pointing at the previous HOLD, so
    /// pairing it with a fresh `lastIKResult` would file a stale muscle result
    /// under a fresh pose. Everything in the record shares one timestamp.
    private func routeSolveToOwningFrame() {
        guard let solve = nimble.lastSolve else { return }
        let t = solve.centerTimestamp
        guard let owner = resultStore.frames.min(by: {
            abs($0.timestamp - t) < abs($1.timestamp - t)
        }), abs(owner.timestamp - t) <= Self.biomechanicsMatchTolerance else { return }

        let motion = solve.motion
        let state: OfflineResultStore.MotionState = motion.isHold
            ? .hold(peakSpeedMetersPerSecond: motion.peakMarkerSpeedMetersPerSecond,
                    windowSeconds: motion.windowSeconds)
            : .moving(peakSpeedMetersPerSecond: motion.peakMarkerSpeedMetersPerSecond,
                      windowSeconds: motion.windowSeconds)

        resultStore.updateBiomechanics(forFrameID: owner.id,
                                       muscleResult: solve.muscle,
                                       idResult: solve.id,
                                       ikResult: solve.ik,
                                       isStaticHoldEstimate: solve.isStaticHoldEstimate,
                                       motionState: state)
    }

    /// Replays `bodyFrame` on backdated timestamps to fill the LEADING half of
    /// the Savitzky-Golay window before the first real frame is pushed.
    ///
    /// Held-pose padding is the right choice here rather than reflection or
    /// extrapolation: it makes `dq`/`ddq` tend to zero at the clip edges, which
    /// matches what this input can actually support. `joint_coords` pins the
    /// pelvis every frame, so global motion is unavailable and the defensible
    /// reading of the muscle numbers is a static-equilibrium one anyway.
    ///
    /// ⚠️ These pads have ZERO marker displacement by construction, so the hold
    /// detector sees them as still. The verdict for the first real frame is
    /// therefore built from 4 real transitions after it and 4 synthetic zeros
    /// before it: whatever the subject was doing in the moment before the clip
    /// started is unobservable and is scored as stillness. The tail padding has
    /// the mirror-image bias. A single photo is the fully degenerate case —
    /// all 8 transitions are synthetic — which is why "one frame is a hold" is
    /// an ASSUMPTION this path inherits from the padding, not a measurement.
    private func primeFilterHead(with bodyFrame: BodyFrame, totalPushes: inout Int) async {
        let dt = sampleInterval
        for step in stride(from: Self.sgHalfWindow, through: 1, by: -1) {
            if Task.isCancelled { return }
            let padded = BodyFrame(timestamp: bodyFrame.timestamp - Double(step) * dt,
                                   frameNumber: bodyFrame.frameNumber,
                                   joints: bodyFrame.joints)
            _ = await submitAndWait(padded, timeout: Self.solveTimeout)
            totalPushes += 1
            // Deliberately not routed: these are centred on synthetic
            // timestamps that match no real frame.
        }
    }

    /// Mirror of `primeFilterHead` for the TRAILING half-window. Each push here
    /// advances the window centre onto one of the last real frames, so unlike
    /// the head padding these results are routed and kept.
    private func padFilterTail(with bodyFrame: BodyFrame, totalPushes: inout Int) async {
        let dt = sampleInterval
        for step in 1...Self.sgHalfWindow {
            if Task.isCancelled { return }
            let padded = BodyFrame(timestamp: bodyFrame.timestamp + Double(step) * dt,
                                   frameNumber: bodyFrame.frameNumber,
                                   joints: bodyFrame.joints)
            let published = await submitAndWait(padded, timeout: Self.solveTimeout)
            totalPushes += 1
            guard published else { continue }
            routeSolveToOwningFrame()
        }
    }

    private func eta(remainingFrames: Int) -> Double? {
        guard !perFrameDurations.isEmpty else { return nil }
        let avg = perFrameDurations.reduce(0, +) / Double(perFrameDurations.count)
        return avg * Double(remainingFrames)
    }

    // MARK: - NimbleEngine readiness

    /// `NimbleEngine.loadBundledModel()` (called from `ContentView.onAppear`)
    /// loads the .osim asynchronously with no completion callback beyond the
    /// `isModelLoaded` @Published flag. Poll briefly instead of failing
    /// immediately, in case offline import opens before that background load
    /// finishes.
    private func waitForNimbleModelLoaded(timeout: TimeInterval = 10.0) async -> Bool {
        if nimble.isModelLoaded { return true }
        let deadline = CACurrentMediaTime() + timeout
        while CACurrentMediaTime() < deadline {
            if nimble.isModelLoaded { return true }
            try? await Task.sleep(nanoseconds: 150_000_000)
        }
        return nimble.isModelLoaded
    }

    // MARK: - Frame decoding

    private func decodeFrames(source: RunSource, samplingMode: FrameSource.SamplingMode) async throws -> [FrameSource.DecodedFrame] {
        switch source {
        case .photo(let image):
            return FrameSource.decodePhoto(image)

        case .video(let url):
            phase = .decodingFrames
            let decoder = FrameSource.VideoDecoder(url: url)
            let duration = try await decoder.duration()
            let (timestamps, truncated) = FrameSource.sampleTimestamps(duration: duration, mode: samplingMode)
            wasFrameCountCapped = truncated

            var frames: [FrameSource.DecodedFrame] = []
            frames.reserveCapacity(timestamps.count)
            for (i, t) in timestamps.enumerated() {
                if Task.isCancelled { break }
                // One undecodable timestamp shouldn't abort the whole clip.
                if let image = try? await decoder.decodeFrame(at: t) {
                    frames.append(FrameSource.DecodedFrame(image: image, timestamp: t, index: i))
                }
            }
            return frames
        }
    }

    // MARK: - Backpressure-safe submission

    /// Submits one frame and waits for its result.
    ///
    /// Retries across drops. `NimbleEngine.processFrame` discards a submission
    /// outright while a previous solve is still in flight, publishing nothing
    /// (`NimbleEngine.swift:226-229`). That matters here in a way it never does
    /// live: if one frame times out, the solve behind it is still running, so
    /// the next frame is dropped, times out too, and the whole rest of the clip
    /// degrades into 6 s waits producing nothing. Worse, the late publish from
    /// the timed-out frame would be picked up by the *next* frame's waiter and
    /// stored against the wrong frame index.
    ///
    /// A drop is detected synchronously — `processFrame` increments
    /// `droppedFrameCount` inline before returning — so it is never confused
    /// with a slow solve.
    private func submitAndWait(_ bodyFrame: BodyFrame, timeout: TimeInterval) async -> Bool {
        for _ in 0..<Self.maxSubmitAttempts {
            switch await NimbleFrameWaiter.submit(on: nimble, timeout: timeout, { [nimble] in
                nimble.processFrame(bodyFrame)
            }) {
            case .published:
                return true
            case .timedOut:
                return false
            case .dropped:
                // The engine is still busy with an earlier solve. Back off and
                // resubmit rather than burning a timeout on a frame it never
                // accepted.
                try? await Task.sleep(nanoseconds: UInt64(Self.dropRetryDelay * 1_000_000_000))
            }
        }
        return false
    }
}

/// Waits for NimbleEngine's next publish after a `processFrame` submission, or
/// times out. See `OfflineSessionRunner`'s header comment for why this exists
/// (no per-frame completion callback) and why a timeout is a legitimate,
/// expected outcome (a totally failed `solveIK` never publishes at all).
@MainActor
private enum NimbleFrameWaiter {
    /// Extra delay after the first `objectWillChange` signal before reporting
    /// "done". `objectWillChange` fires from inside a `@Published` property's
    /// `willSet` — i.e. before NimbleEngine finishes writing the ~12 fields
    /// `publishResults` sets in sequence, and before its `defer` block (which
    /// clears `isFrameInFlight`) even runs. Both of those are enqueued onto the
    /// SAME serial main queue, strictly after `publishResults`'s own field
    /// writes are enqueued, from the same background thread with no
    /// intervening work — so in practice `isFrameInFlight` clears before this
    /// fires. This fixed grace period (far larger than any realistic
    /// same-queue scheduling jitter) makes that non-load-bearing instead of a
    /// requirement: by the time it elapses, every property publishResults sets,
    /// AND NimbleEngine's own `isFrameInFlight = false`, are guaranteed to have
    /// run, regardless of that ordering argument.
    private static let publishSettleDelay: TimeInterval = 0.03

    enum Outcome {
        /// The engine accepted the frame and published a result.
        case published
        /// The engine accepted the frame but published nothing in time. A
        /// fully-failed `solveIK` never calls `publishResults`, so this is an
        /// expected outcome, not necessarily a stall.
        case timedOut
        /// The engine refused the frame because a previous solve was still in
        /// flight. Nothing was computed and nothing will be published for it.
        case dropped
    }

    /// Runs `submit` with the publish subscription ALREADY live, then waits.
    ///
    /// The subscription must be established before the submission: `processFrame`
    /// hands the solve to a background queue, and a fast solve can publish before
    /// a subscription created afterwards exists — the signal would be missed and
    /// the caller would burn the full timeout on a frame that actually succeeded.
    static func submit(
        on engine: NimbleEngine,
        timeout: TimeInterval,
        _ submit: () -> Void
    ) async -> Outcome {
        await withCheckedContinuation { (continuation: CheckedContinuation<Outcome, Never>) in
            var cancellable: AnyCancellable?
            var resumed = false
            let finish: (Outcome) -> Void = { result in
                guard !resumed else { return }
                resumed = true
                cancellable?.cancel()
                continuation.resume(returning: result)
            }

            cancellable = engine.objectWillChange
                .receive(on: DispatchQueue.main)
                .sink { _ in
                    DispatchQueue.main.asyncAfter(deadline: .now() + publishSettleDelay) {
                        finish(.published)
                    }
                }

            let droppedBefore = engine.droppedFrameCount
            submit()
            // `processFrame` increments `droppedFrameCount` inline before
            // returning, so this read is exact — no race with the solve.
            if engine.droppedFrameCount != droppedBefore {
                finish(.dropped)
                return
            }

            DispatchQueue.main.asyncAfter(deadline: .now() + timeout) {
                finish(.timedOut)
            }
        }
    }
}
