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
    /// SavitzkyGolayFilter.swift: 9 pushes fill the filter's window before ANY
    /// muscle/ID output exists.
    private static let sgWarmupFrameCount = 9
    /// Synthetic frame spacing used only for the end-of-clip warm-up pad — an
    /// arbitrary but plausible live cadence. Because every padded push replays
    /// the IDENTICAL pose, the Savitzky-Golay filter's velocity/acceleration
    /// coefficients (which sum to zero for a constant input, by construction)
    /// come out at ~0 regardless of the exact spacing chosen.
    private static let padSyntheticFrameInterval: TimeInterval = 1.0 / 30.0

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
        var totalPushes = 0  // real pushes to nimble.processFrame, including warm-up padding

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

        // Short clip / single photo: pad so the Savitzky-Golay filter still
        // warms up and the user gets a (static-hold) muscle estimate instead of
        // pose-only results forever. See this class's header comment.
        if totalPushes > 0, totalPushes < Self.sgWarmupFrameCount, let last = lastSuccessfulFrame {
            await padToWarmUp(last, totalPushes: &totalPushes)
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
                bodyFrame: nil, ikResult: nil, idResult: nil, muscleResult: nil, isStaticHoldEstimate: false))
            return false
        }

        let bodyFrame = MHRRetarget.makeBodyFrame(jointCoords: estimate.jointCoords,
                                                   timestamp: frame.timestamp,
                                                   frameNumber: frameIndex)

        guard bodyFrame.joints.contains(where: \.isTracked) else {
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
                status: .poseEstimationFailed("retarget produced no usable joints"), usedFallbackBBox: estimate.usedFallbackBBox,
                bodyFrame: bodyFrame, ikResult: nil, idResult: nil, muscleResult: nil, isStaticHoldEstimate: false))
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

        let published = await submitAndWait(bodyFrame, timeout: Self.solveTimeout)
        totalPushes += 1
        lastSuccessfulFrame = (frameIndex, bodyFrame)

        guard published else {
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
                status: .nimbleTimeout, usedFallbackBBox: estimate.usedFallbackBBox,
                bodyFrame: bodyFrame, ikResult: nil, idResult: nil, muscleResult: nil, isStaticHoldEstimate: false))
            return false
        }

        resultStore.append(OfflineResultStore.FrameResult(
            id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
            status: .success, usedFallbackBBox: estimate.usedFallbackBBox,
            bodyFrame: bodyFrame, ikResult: nimble.lastIKResult, idResult: nimble.lastIDResult,
            muscleResult: nimble.lastMuscleResult, isStaticHoldEstimate: false))
        return true
    }

    /// Replays the last real frame's pose with synthetic, evenly-spaced
    /// timestamps until the SG filter has seen `sgWarmupFrameCount` total
    /// pushes, then writes whatever muscle/ID/IK result that produced back onto
    /// the ORIGINAL frame's row (not a new one) via
    /// `OfflineResultStore.updateBiomechanics`.
    private func padToWarmUp(_ last: (id: Int, bodyFrame: BodyFrame), totalPushes: inout Int) async {
        let baseTimestamp = last.bodyFrame.timestamp
        while totalPushes < Self.sgWarmupFrameCount {
            if Task.isCancelled { return }
            let syntheticTimestamp = baseTimestamp + Double(totalPushes) * Self.padSyntheticFrameInterval
            let padded = BodyFrame(timestamp: syntheticTimestamp, frameNumber: last.bodyFrame.frameNumber,
                                    joints: last.bodyFrame.joints)
            let published = await submitAndWait(padded, timeout: Self.solveTimeout)
            totalPushes += 1
            guard published else { continue }
            if let muscle = nimble.lastMuscleResult {
                resultStore.updateBiomechanics(forFrameID: last.id, muscleResult: muscle,
                                                idResult: nimble.lastIDResult, ikResult: nimble.lastIKResult,
                                                isStaticHoldEstimate: true)
            }
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
