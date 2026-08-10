import Foundation
import Combine
import QuartzCore
import UIKit

/// Pure admission and segmentation rules for the offline temporal pipeline.
/// Kept outside the `@MainActor` runner so every edge decision can be tested
/// without loading Core ML, Nimble, or a simulator scene.
enum OfflineTemporalPolicy {
    enum SourceKind: Equatable, Sendable {
        case photo
        case video
    }

    struct SegmentPlan: Equatable {
        /// Indices into the compact array of trusted `BodyFrame`s. The frames
        /// themselves keep their decoder slot numbers, so a hole still splits
        /// two adjacent ranges here.
        let frameIndices: Range<Int>
        /// The first segment follows a full clip reset. Every later segment
        /// needs only the derivative/hold/display reset that preserves the
        /// clip's IK and ground warm starts.
        let resetsRealtimeStateBefore: Bool
        /// Held-pose padding is legal only at a real requested clip endpoint,
        /// never beside a known missing interval.
        let padsHead: Bool
        let padsTail: Bool
    }

    static func exclusion(
        source: SourceKind,
        usedFallbackBBox: Bool
    ) -> OfflineResultStore.TemporalAnalysisExclusion? {
        guard source == .video, usedFallbackBBox else { return nil }
        return .videoVisionWholeFrameFallback
    }

    static func areContiguous(previousFrameNumber: Int, currentFrameNumber: Int) -> Bool {
        currentFrameNumber == previousFrameNumber + 1
    }

    static func segmentPlans(
        frameNumbers: [Int],
        firstRequestedFrameNumber: Int?,
        lastRequestedFrameNumber: Int?
    ) -> [SegmentPlan] {
        guard !frameNumbers.isEmpty else { return [] }

        var ranges: [Range<Int>] = []
        var lowerBound = 0
        for index in 1..<frameNumbers.count where !areContiguous(
            previousFrameNumber: frameNumbers[index - 1],
            currentFrameNumber: frameNumbers[index]
        ) {
            ranges.append(lowerBound..<index)
            lowerBound = index
        }
        ranges.append(lowerBound..<frameNumbers.count)

        return ranges.enumerated().map { segmentIndex, range in
            SegmentPlan(
                frameIndices: range,
                resetsRealtimeStateBefore: segmentIndex > 0,
                padsHead: range.lowerBound == 0
                    && frameNumbers[range.lowerBound] == firstRequestedFrameNumber,
                padsTail: range.upperBound == frameNumbers.count
                    && frameNumbers[range.upperBound - 1] == lastRequestedFrameNumber
            )
        }
    }
}

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
    /// Non-nil when the run analysed fewer frames than the sampling mode asked
    /// for — carrying WHICH of the two causes it was and how many frames were
    /// really used, because the single boolean this replaced let the UI state
    /// both wrongly. See `FrameBudgetNotice`.
    @Published private(set) var frameBudgetNotice: FrameBudgetNotice?
    let resultStore = OfflineResultStore()

    private let nimble: NimbleEngine
    private let poseEstimator: SAM3DPoseEstimator
    private var runTask: Task<Void, Never>?
    private var perFrameDurations: [TimeInterval] = []
    private var calibrated = false
    private var lastSuccessfulFrame: (id: Int, bodyFrame: BodyFrame)?
    /// Last frame admitted to the current SG/hold segment, using the decoder's
    /// original slot number rather than the compact result-store id.
    private var lastTemporalFrameNumber: Int?
    /// Every trusted frame that produced a usable skeleton, in decode order.
    /// Decoder slot numbers stay on the frames, so exclusions and decode holes
    /// remain visible even though this is a compact array. Gait analysis needs
    /// the whole clip before it can say anything — a stride is not visible one
    /// frame at a time — so it runs after the batch and splits these raw gaps.
    private var usableBodyFrames: [BodyFrame] = []
    /// Identifies the current run so a cancelled predecessor cannot clean up
    /// shared engine state that now belongs to its successor. See `runInternal`.
    private var runToken: UInt64 = 0

    /// Generous upper bound on one `nimble.processFrame` solve. NimbleEngine's
    /// module docs record ~200ms/frame for the shipped FullBody.osim (520
    /// muscles / 169 coordinates) on comparable hardware; this leaves wide
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
    /// frames padded onto each successfully observed clip endpoint. Known
    /// leading/trailing failures and internal gaps are never padded.
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
        frameBudgetNotice = nil
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
        lastTemporalFrameNumber = nil
        usableBodyFrames.removeAll()
        nimble.gaitPlan = nil

        let sourceKind: OfflineTemporalPolicy.SourceKind
        switch source {
        case .photo: sourceKind = .photo
        case .video: sourceKind = .video
        }

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

        let batch: DecodedBatch
        do {
            batch = try await decodeFrames(source: source, samplingMode: samplingMode)
        } catch {
            phase = .failed(error.localizedDescription)
            return
        }
        let decoded = batch.frames
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

            let succeeded = await processOneFrame(
                frame,
                frameIndex: i,
                sourceKind: sourceKind,
                firstRequestedFrameNumber: batch.firstRequestedFrameNumber,
                totalPushes: &totalPushes
            )
            if !succeeded { failureCount += 1 }

            perFrameDurations.append(CACurrentMediaTime() - frameStart)
        }

        if Task.isCancelled {
            phase = .finished(processed: decoded.count, failed: failureCount, cancelled: true)
            return
        }

        // Edge-pad the tail only when the final trusted pose belongs to the
        // final requested decoder slot. This mirrors head priming: the centred
        // SG window lags 4 samples, so the LAST 4 real frames otherwise never
        // reach its middle. Each push yields a result centred on a real frame,
        // which `routeSolveToOwningFrame` files correctly. A known trailing
        // decode/pose/fallback gap therefore gets no padding. A single photo
        // (1 real push) is the degenerate case — 4 head + 1 + 4 tail = 9,
        // centred exactly on the photo.
        if totalPushes > 0,
           let last = lastSuccessfulFrame,
           last.bodyFrame.frameNumber == batch.lastRequestedFrameNumber {
            await padFilterTail(with: last.bodyFrame, totalPushes: &totalPushes,
                                halfWindow: Self.sgHalfWindow)
        }

        // --- Is this a RUN? ---------------------------------------------------
        //
        // Only now, because a stride is not visible one frame at a time. The
        // static-hold pass above has already produced everything it can; if the
        // clip turns out to be running, the second pass re-solves the STANCE
        // frames with the root acceleration the gait cycle supplies, which is
        // the only way a moving subject gets muscle numbers at all.
        await runGaitPassIfThisIsARun(
            firstRequestedFrameNumber: batch.firstRequestedFrameNumber,
            lastRequestedFrameNumber: batch.lastRequestedFrameNumber
        )

        phase = .finished(processed: decoded.count, failed: failureCount, cancelled: false)
    }

    // MARK: - Gait pass

    /// Runs `GaitAnalysis` over the whole window and, if the clip is a usable
    /// run, re-solves its stance frames with gait dynamics.
    ///
    /// Everything here is guarded so a non-running clip — a photo, a squat, a
    /// subject standing still — costs one cheap analysis and changes nothing.
    private func runGaitPassIfThisIsARun(
        firstRequestedFrameNumber: Int?,
        lastRequestedFrameNumber: Int?
    ) async {
        guard !Task.isCancelled, usableBodyFrames.count >= GaitSignal.minimumFrames else {
            resultStore.setGait(.notAttempted(reason: usableBodyFrames.isEmpty
                                              ? "no usable frames"
                                              : "\(usableBodyFrames.count) usable frames; a stride needs at least \(GaitSignal.minimumFrames)"))
            return
        }

        let report: GaitReport
        do {
            report = try GaitAnalysis.analyse(frames: usableBodyFrames)
        } catch {
            resultStore.setGait(.notAttempted(reason: "\(error)"))
            return
        }

        guard report.isUsable else {
            // Refused BY THE CLIP'S OWN MODEL. The reasons are published so the
            // user is told what to film differently, not just that it failed.
            resultStore.setGait(.refused(report: report))
            return
        }

        guard let plan = Self.makePlan(from: report) else {
            resultStore.setGait(.notAttempted(reason: "the report has no complete contact to build a plan from"))
            return
        }

        // Clip boundary between passes: the derivative window changes length
        // here (9 taps -> `plan.filterTaps`), and the IK warm start belongs to
        // the last frame of pass 1, not the first of pass 2.
        nimble.resetAnalysisPassStatePreservingGround()
        // Pass two owns the dynamics generation. Clear pass-one statics up
        // front so a timeout or missing centred publication cannot leave old
        // ID/muscle values under an analysed-running result.
        resultStore.beginGaitReplacementPass()
        nimble.staticHoldGating = false
        nimble.gaitPlan = plan
        defer { nimble.gaitPlan = nil; nimble.staticHoldGating = true }

        let half = plan.filterTaps / 2
        var pushes = 0
        var processed = 0
        let segments = OfflineTemporalPolicy.segmentPlans(
            frameNumbers: usableBodyFrames.map(\.frameNumber),
            firstRequestedFrameNumber: firstRequestedFrameNumber,
            lastRequestedFrameNumber: lastRequestedFrameNumber
        )

        for segment in segments {
            if segment.resetsRealtimeStateBefore {
                // Must precede the next waiter's subscription: the reset emits
                // `objectWillChange` synchronously, then queues filter clearing
                // ahead of the next solve on the same FIFO solver queue.
                nimble.resetRealtimeState()
            }

            let frames = usableBodyFrames[segment.frameIndices]
            if segment.padsHead, let first = frames.first {
                await primeFilterHead(with: first, totalPushes: &pushes, halfWindow: half)
            }

            for body in frames {
                if Task.isCancelled { return }
                phase = .running(current: processed,
                                 total: usableBodyFrames.count,
                                 etaSeconds: nil)
                if await submitAndWait(body, timeout: Self.solveTimeout) {
                    routeSolveToOwningFrame()
                }
                pushes += 1
                processed += 1
            }

            if segment.padsTail, let last = frames.last {
                await padFilterTail(with: last, totalPushes: &pushes, halfWindow: half)
            }
        }

        resultStore.setGait(.analysed(report: report, plan: plan))
    }

    /// Turns a `GaitReport` into the per-frame root acceleration and ground
    /// force the engine consumes.
    ///
    /// The stance force is the half sine the impulse derivation implies:
    /// `F(t)/(m·g) = Fmax_bw · sin(π·φ)`, with `φ` the fraction through the
    /// contact. Samples are placed at the MIDPOINTS of their sampling
    /// intervals, not at the edges — a sample is an average over the interval it
    /// represents, and putting the first one at `φ = 0` would claim the foot
    /// carries exactly zero force at the instant it is already measurably
    /// planted.
    ///
    /// # Two things this does NOT do, each because of a defect it caused
    ///
    /// 1. It does not lay entries at `touchdown + k·dt`. It lays them at the
    ///    contact's OWN sample timestamps. With a dropped frame inside a
    ///    contact the two differ, and every sample after the hole then fell
    ///    outside `GaitPlan.entry(at:)`'s ±dt/2 match window and was solved as
    ///    FLIGHT — force 0 and `a_root = −g` — while the foot was still planted.
    /// 2. It does not apply one clip-wide peak to both feet. Each contact
    ///    carries its OWN leg's peak, closed on its own contact time. See
    ///    `GaitReport.peakVerticalForceInBodyWeights`.
    ///
    /// Frames between contacts get force 0 and side 0: flight. Frames outside
    /// the first touchdown or the last toe-off get NO entry at all, so the
    /// engine reports them as outside the analysis rather than guessing.
    nonisolated static func makePlan(from report: GaitReport) -> NimbleEngine.GaitPlan? {
        let dt = report.sampleInterval
        guard dt > 0 else { return nil }
        let intervals = report.stance.left + report.stance.right
        guard !intervals.isEmpty else { return nil }

        let taps = WindowedDerivativeFilter.admissibleTaps(report.derivativeFilterTaps)
        let half = taps / 2

        var entries: [TimeInterval: NimbleEngine.GaitPlan.Frame] = [:]
        var earliest = Double.infinity
        var latest = -Double.infinity

        // The contact INDEX is stamped here, at the only place that knows where
        // the detector put the boundaries, and travels with every frame from
        // here on. Ordered by touchdown so the numbering is the clip's own
        // chronology and does not depend on `stance.left + stance.right`.
        for (contactIndex, interval) in intervals
            .sorted(by: { $0.touchdown < $1.touchdown })
            .enumerated() {
            let stamps = interval.sampleTimestamps
            let n = stamps.count
            guard n > 0 else { continue }
            let fmax = report.peakVerticalForceInBodyWeights[interval.side]
            guard fmax.isFinite, fmax > 0 else { continue }
            let side = interval.side == .left ? -1 : 1
            for (k, t) in stamps.enumerated() {
                let phase = (Double(k) + 0.5) / Double(n)
                let force = fmax * sin(Double.pi * phase)
                entries[t] = .init(timestamp: t,
                                   verticalForceInBodyWeights: force,
                                   contactSide: side,
                                   contactIndex: contactIndex,
                                   derivativeWindowInsideContact: k >= half && k <= n - 1 - half)
            }
            earliest = Swift.min(earliest, interval.touchdown)
            latest = Swift.max(latest, interval.lastStanceSample)
        }
        guard earliest.isFinite, latest.isFinite, latest > earliest else { return nil }

        // Fill the gaps between contacts with flight.
        var t = earliest
        while t <= latest + dt / 2 {
            if entries[t] == nil, !entries.keys.contains(where: { abs($0 - t) < dt / 2 }) {
                entries[t] = .init(timestamp: t, verticalForceInBodyWeights: 0, contactSide: 0,
                                   contactIndex: -1,
                                   derivativeWindowInsideContact: false)
            }
            t += dt
        }

        return NimbleEngine.GaitPlan(
            frames: entries.values.sorted { $0.timestamp < $1.timestamp },
            filterTaps: taps,
            sampleInterval: dt)
    }

    /// Returns true iff the frame produced a `.success` result (pose estimated,
    /// nimble published something for it — muscle/ID may still be nil during SG
    /// warm-up, which is still `.success`).
    @discardableResult
    private func processOneFrame(
        _ frame: FrameSource.DecodedFrame,
        frameIndex: Int,
        sourceKind: OfflineTemporalPolicy.SourceKind,
        firstRequestedFrameNumber: Int?,
        totalPushes: inout Int
    ) async -> Bool {
        // The tail candidate must describe the final requested slot itself.
        // Clearing it before any fallible work prevents a trailing pose failure
        // or review-only fallback from padding the previous trusted pose across
        // a known missing interval.
        lastSuccessfulFrame = nil

        let estimate: SAM3DPoseEstimator.Output
        do {
            estimate = try await poseEstimator.estimate(uiImage: frame.image)
        } catch {
            endTemporalSegment()
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
                status: .poseEstimationFailed(error.localizedDescription), usedFallbackBBox: false,
                camT: nil,
                modelChecksums: nil,
                bodyFrame: nil, ikResult: nil, idResult: nil, muscleResult: nil, isStaticHoldEstimate: false, motionState: .undetermined))
            return false
        }

        // `frame.index` is the DECODER SLOT, not this frame's position in the
        // surviving array. They differ exactly when a timestamp failed to
        // decode, and the difference is what makes a dropped frame visible:
        // `GaitSignal` reads `frameNumber` gaps to report `droppedFrameCount`,
        // and `video_013` lost 3 frames to Vision finding no person. Passing the
        // array position instead would present a clip with holes as a
        // continuous one sampled slightly slower.
        let bodyFrame = MHRRetarget.makeBodyFrame(jointCoords: estimate.jointCoords,
                                                   timestamp: frame.timestamp,
                                                   frameNumber: frame.index)

        // A whole-image fallback is still a pose result. For a PHOTO it remains
        // the only pose and follows the existing static analysis path. For a
        // VIDEO it is a measured discontinuity: keep the projected skeleton for
        // review, but branch before plausibility, scale, SG priming, Nimble, and
        // the gait-frame collection so no downstream calculation can consume it.
        if let exclusion = OfflineTemporalPolicy.exclusion(
            source: sourceKind,
            usedFallbackBBox: estimate.usedFallbackBBox
        ) {
            endTemporalSegment()
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex,
                sourceImage: frame.image,
                timestamp: frame.timestamp,
                status: .success,
                usedFallbackBBox: true,
                temporalAnalysisExclusion: exclusion,
                camT: estimate.camT,
                modelChecksums: (estimate.inputChecksum, estimate.outputChecksum,
                                 estimate.sourceHash, estimate.bboxHash, estimate.warpHash),
                bodyFrame: bodyFrame,
                ikResult: nil,
                idResult: nil,
                muscleResult: nil,
                isStaticHoldEstimate: false,
                motionState: .undetermined
            ))
            return true
        }

        // BODY-SIZE GATE — must sit ahead of `nimble.scaleModel`.
        //
        // `scaleModelWithHeight` clamps its per-segment factors into
        // [0.7, 1.4], so a collapsed prediction does not fail there, it is
        // silently truncated into a model scaled to nobody (STATUS.md: one
        // occluded subject produced a 0.070 m hip width and nothing flagged it).
        // The frame is kept in the store with its image, its retargeted
        // skeleton and the measured numbers, so the user sees WHY — dropping it
        // silently is what made the original case invisible.
        let plausibility = MHRRetarget.plausibility(jointCoords: estimate.jointCoords)
        if case .implausible(let reason, let hip, let stature) = plausibility {
            endTemporalSegment()
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
                status: .implausibleBody(reason: reason, hipWidthMeters: hip, statureMeters: stature),
                usedFallbackBBox: estimate.usedFallbackBBox,
                camT: estimate.camT,
                modelChecksums: (estimate.inputChecksum, estimate.outputChecksum,
                                 estimate.sourceHash, estimate.bboxHash, estimate.warpHash),
                bodyFrame: bodyFrame, ikResult: nil, idResult: nil, muscleResult: nil,
                isStaticHoldEstimate: false, motionState: .undetermined))
            return false
        }

        guard bodyFrame.joints.contains(where: \.isTracked) else {
            endTemporalSegment()
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
                status: .poseEstimationFailed("retarget produced no usable joints"), usedFallbackBBox: estimate.usedFallbackBBox,
                camT: estimate.camT,
                modelChecksums: (estimate.inputChecksum, estimate.outputChecksum,
                                 estimate.sourceHash, estimate.bboxHash, estimate.warpHash),
                bodyFrame: bodyFrame, ikResult: nil, idResult: nil, muscleResult: nil, isStaticHoldEstimate: false, motionState: .undetermined))
            return false
        }

        if let previous = lastTemporalFrameNumber,
           !OfflineTemporalPolicy.areContiguous(
               previousFrameNumber: previous,
               currentFrameNumber: bodyFrame.frameNumber
           ) {
            // A decoder hole has no `processOneFrame` call of its own. Detect it
            // from the original slot numbers and clear the derivative state
            // before the next waiter exists.
            endTemporalSegment()
        }

        let shouldPrimeHead = lastTemporalFrameNumber == nil
            && bodyFrame.frameNumber == firstRequestedFrameNumber
        lastTemporalFrameNumber = bodyFrame.frameNumber

        if !calibrated {
            let stature = Double(MHRRetarget.estimatedStatureMeters(jointCoords: estimate.jointCoords))
            let (positions, names) = MHRRetarget.segmentScaleMarkers(jointCoords: estimate.jointCoords)
            // See this class's header comment for why no completion signal is
            // needed here: solverQueue is serial and FIFO, so this happens-before
            // the processFrame submission immediately following it.
            nimble.scaleModel(height: stature, markerPositions: positions, markerNames: names)
            calibrated = true
        }

        // Edge-pad only a trusted pose at the first requested decoder slot.
        // The SG filter is CENTRED, so without this the first 4 real frames can
        // never sit at the middle of a full window and would show pose with no
        // muscle. Replaying this frame backdated fills the leading half-window;
        // results centred on synthetic timestamps are discarded by
        // `routeSolveToOwningFrame`, which is what we want. A known leading gap
        // and every segment after an internal gap get no synthetic history.
        if shouldPrimeHead {
            await primeFilterHead(with: bodyFrame, totalPushes: &totalPushes,
                                  halfWindow: Self.sgHalfWindow)
        }

        let published = await submitAndWait(bodyFrame, timeout: Self.solveTimeout)
        totalPushes += 1
        usableBodyFrames.append(bodyFrame)

        guard published else {
            endTemporalSegment()
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
                status: .nimbleTimeout, usedFallbackBBox: estimate.usedFallbackBBox,
                camT: estimate.camT,
                modelChecksums: (estimate.inputChecksum, estimate.outputChecksum,
                                 estimate.sourceHash, estimate.bboxHash, estimate.warpHash),
                bodyFrame: bodyFrame, ikResult: nil, idResult: nil, muscleResult: nil, isStaticHoldEstimate: false, motionState: .undetermined))
            return false
        }

        lastSuccessfulFrame = (frameIndex, bodyFrame)

        // Biomechanics are deliberately NOT attached here — see
        // `routeSolveToOwningFrame`. The newest solve does not describe
        // the newest frame.
        resultStore.append(OfflineResultStore.FrameResult(
            id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
            status: .success, usedFallbackBBox: estimate.usedFallbackBBox,
            camT: estimate.camT,
                modelChecksums: (estimate.inputChecksum, estimate.outputChecksum,
                                 estimate.sourceHash, estimate.bboxHash, estimate.warpHash),
                bodyFrame: bodyFrame, ikResult: nil, idResult: nil,
            muscleResult: nil, isStaticHoldEstimate: false, motionState: .undetermined))
        routeSolveToOwningFrame()
        return true
    }

    /// Ends the current trusted temporal segment without discarding clip-wide
    /// IK, ground, or QP warm starts.
    ///
    /// `resetRealtimeState()` synchronously clears published values and queues
    /// SG/hold clearing on NimbleEngine's serial solver queue. Every call site
    /// invokes this before the next `NimbleFrameWaiter` subscribes, so the
    /// reset's own `objectWillChange` cannot masquerade as a completed solve;
    /// FIFO then guarantees the filter clear runs before the next solve.
    private func endTemporalSegment() {
        guard lastTemporalFrameNumber != nil else {
            lastSuccessfulFrame = nil
            return
        }
        nimble.resetRealtimeState()
        lastTemporalFrameNumber = nil
        lastSuccessfulFrame = nil
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
        // The gait cases carry different evidence — a modelled force and the
        // residual that can contradict it — so they get their own case in the
        // SAME enum rather than being flattened into the stillness numbers,
        // which mean nothing about a runner.
        let state: OfflineResultStore.MotionState
        if motion.verdict.isGait {
            state = .gait(verdict: motion.verdict, outcome: solve.gait)
        } else {
            state = .measured(
                verdict: motion.verdict,
                peakSpeedMetersPerSecond: motion.peakMarkerSpeedMetersPerSecond,
                windowSeconds: motion.windowSeconds,
                noiseFloorMetersPerSecond: motion.poseNoiseFloorMetersPerSecond)
        }

        resultStore.replaceBiomechanics(
            forFrameID: owner.id,
            with: OfflineResultStore.BiomechanicsPayload(
                ikResult: solve.ik,
                idResult: solve.id,
                muscleResult: solve.muscle,
                dynamicsAvailability: solve.dynamicsAvailability,
                isStaticHoldEstimate: solve.isStaticHoldEstimate,
                motionState: state))
    }

    /// Replays `bodyFrame` on backdated timestamps to fill the LEADING half of
    /// the Savitzky-Golay window before a real, observed clip head is pushed.
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
    private func primeFilterHead(with bodyFrame: BodyFrame, totalPushes: inout Int,
                                 halfWindow: Int) async {
        let dt = sampleInterval
        guard halfWindow >= 1 else { return }
        for step in stride(from: halfWindow, through: 1, by: -1) {
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

    /// Mirror of `primeFilterHead` for a real, observed clip tail. Each push
    /// advances the window centre onto one of the last real frames, so unlike
    /// the head padding these results are routed and kept.
    private func padFilterTail(with bodyFrame: BodyFrame, totalPushes: inout Int,
                               halfWindow: Int) async {
        let dt = sampleInterval
        guard halfWindow >= 1 else { return }
        for step in 1...halfWindow {
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

    /// Surviving images plus the bounds of the timestamp request that produced
    /// them. Keeping the latter is what makes leading/trailing decode failures
    /// visible; `frames.last?.index` alone silently moves the clip endpoint
    /// inward and would license false held-pose padding there.
    private struct DecodedBatch {
        let frames: [FrameSource.DecodedFrame]
        let firstRequestedFrameNumber: Int?
        let lastRequestedFrameNumber: Int?
    }

    private func decodeFrames(
        source: RunSource,
        samplingMode: FrameSource.SamplingMode
    ) async throws -> DecodedBatch {
        switch source {
        case .photo(let image):
            return DecodedBatch(frames: FrameSource.decodePhoto(image),
                                firstRequestedFrameNumber: 0,
                                lastRequestedFrameNumber: 0)

        case .video(let url):
            phase = .decodingFrames
            let decoder = FrameSource.VideoDecoder(url: url)
            let duration = try await decoder.duration()
            let rate = await decoder.nominalFrameRate()
            let (timestamps, truncated) = FrameSource.sampleTimestamps(duration: duration,
                                                                      mode: samplingMode,
                                                                      nominalFrameRate: rate)
            frameBudgetNotice = FrameBudgetNotice.make(mode: samplingMode,
                                                       duration: duration,
                                                       nominalFrameRate: rate,
                                                       timestamps: timestamps,
                                                       wasTruncated: truncated)

            var frames: [FrameSource.DecodedFrame] = []
            frames.reserveCapacity(timestamps.count)
            for (i, t) in timestamps.enumerated() {
                if Task.isCancelled { break }
                // One undecodable timestamp shouldn't abort the whole clip.
                if let image = try? await decoder.decodeFrame(at: t) {
                    frames.append(FrameSource.DecodedFrame(image: image, timestamp: t, index: i))
                }
            }
            return DecodedBatch(frames: frames,
                                firstRequestedFrameNumber: timestamps.indices.first,
                                lastRequestedFrameNumber: timestamps.indices.last)
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
enum NimbleFrameWaiter {
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
