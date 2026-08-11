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

/// Monotonic ownership for an actor-reentrant offline run. Acquiring a new
/// token synchronously makes every older task stale before either task can
/// cross another suspension point.
struct OfflineRunOwnership: Equatable, Sendable {
    private var nextInvocation: UInt64 = 0
    private(set) var latestInvocation: UInt64 = 0
    private var nextToken: UInt64 = 0
    private(set) var activeToken: UInt64?

    /// Registers the caller before it performs any synchronously publishing
    /// cleanup. If that cleanup re-enters `run()`, the nested (later) caller
    /// advances this epoch and the outer caller must not acquire afterward.
    mutating func beginInvocation() -> UInt64 {
        nextInvocation &+= 1
        latestInvocation = nextInvocation
        return nextInvocation
    }

    func isLatestInvocation(_ invocation: UInt64) -> Bool {
        invocation == latestInvocation
    }

    mutating func beginRun() -> UInt64 {
        nextToken &+= 1
        activeToken = nextToken
        return nextToken
    }

    /// Invalidates the active lease exactly once. Bumping the counter at the
    /// fence ensures a cancelled token can never become current again even if
    /// integer identity is inspected outside `activeToken`.
    @discardableResult
    mutating func invalidateCurrent() -> UInt64? {
        guard let token = activeToken else { return nil }
        nextToken &+= 1
        activeToken = nil
        return token
    }

    /// Normal task-defer completion. A stale or already-completed task has no
    /// cleanup authority, making repeated backstops harmless.
    @discardableResult
    mutating func complete(_ token: UInt64) -> Bool {
        guard activeToken == token else { return false }
        activeToken = nil
        return true
    }

    func isCurrent(_ token: UInt64) -> Bool {
        token == activeToken
    }
}

/// Drives the offline (photo/video import) batch: decode frames -> Core ML pose
/// estimate -> MHR retarget -> NimbleEngine IK -> pose/timing publication.
/// ID/muscle and a second gait pass remain behind a future validated-contact
/// branch; both bundled models stop before that branch.
///
/// # Backpressure (design constraint — see this file set's task brief)
/// `NimbleEngine.processFrame` synchronously returns accepted(receipt), dropped
/// or rejected. A dedicated completion stream terminates only that exact
/// publication: `.published`/`.failed` follow physical completion, while a
/// reset can send `.superseded` immediately and deliberately retains physical
/// solver occupancy until the non-cancellable block really ends.
/// This runner therefore never advances an accepted batch frame until its
/// `.published`/`.failed`/`.superseded` event or a generous timeout. Timeout
/// immediately supersedes the exact publication before resuming; later frames
/// are dropped rather than queued until the old physical solve releases.
///
/// # Clip boundaries
/// `resetSessionState()` is called once before each run, dropping the bridge's
/// IK warm-start pose and rolling ground-height window so a new clip never
/// blends with a previous one or the live session.
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
/// # Static-hold policy (future capability-valid dynamics only)
/// This runner turns on `NimbleEngine.staticHoldGating` for the duration of a
/// run. On this path the pose source zeroes `global_trans`, so the pelvis sits
/// at a model constant in every frame and the body has no global translation:
/// `M·q̈` and the centre-of-mass acceleration would be computed from motion
/// that did not happen (in a squat the pelvis never descends — the feet appear
/// to rise). So frames where the subject was measurably moving get pose only.
/// A still frame would be eligible for static dynamics only after the separate
/// foot-support capability gate; current bundled models publish pose only.
/// The engine is SHARED with the live ARKit view, whose q̈ IS observable, hence
/// the flag is scoped to the run rather than made unconditional.
@MainActor
final class OfflineSessionRunner: ObservableObject {

    nonisolated let objectWillChange = ObservableObjectPublisher()

    enum RunSource {
        case photo(UIImage)
        case video(URL)
    }

    enum RunPhase: Equatable {
        case idle
        case checkingCameraReference
        case loadingModel
        case decodingFrames
        case running(current: Int, total: Int, etaSeconds: Double?)
        case finished(processed: Int, failed: Int, cancelled: Bool)
        case failed(String)
    }

    // These two UI fields commit before their one explicit notification.
    // `@Published` would notify from willSet, allowing a synchronous Cancel or
    // replacement Run to publish newer state that the old setter then overwrote.
    private(set) var phase: RunPhase = .idle
    /// Non-nil when the run analysed fewer frames than the sampling mode asked
    /// for — carrying WHICH of the two causes it was and how many frames were
    /// really used, because the single boolean this replaced let the UI state
    /// both wrongly. See `FrameBudgetNotice`.
    private(set) var frameBudgetNotice: FrameBudgetNotice?
    let resultStore = OfflineResultStore()

    private let nimble: NimbleEngine
    private let poseEstimator: SAM3DPoseEstimator
    private let cameraMotionAnalyzer: any CameraMotionAnalyzing
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
    /// Identifies the current run so a cancelled predecessor cannot publish or
    /// clean up shared state that now belongs to its successor.
    private var runOwnership = OfflineRunOwnership()
    /// Matching engine-global lease. This closes the seam that a per-Runner
    /// token cannot see when two Runner instances share one NimbleEngine.
    private var enginePolicyLease: NimbleEngine.OfflinePolicyLease?

    /// Liveness fence on one `nimble.processFrame` solve, not a performance
    /// claim. The available receipts are isolated Debug iOS Simulator stages:
    /// moving-input warm-start IK at ~6 mm/frame measured 1567 ms/frame at
    /// 77.8 iterations, while unchanged-marker warm IK measured
    /// 49 ms, and the 520×109 QP 194.4 ms. They are not additive end-to-end
    /// measurements, and Release-device timing has never been measured. Six
    /// seconds therefore remains a provisional stuck-work bound to revisit
    /// with device evidence, rather than a promised per-frame duration.
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

    init(
        nimble: NimbleEngine,
        cameraMotionAnalyzer: any CameraMotionAnalyzing = CameraMotionVideoAnalyzer()
    ) {
        self.nimble = nimble
        self.poseEstimator = SAM3DPoseEstimator()
        self.cameraMotionAnalyzer = cameraMotionAnalyzer
    }

    deinit {
        // The task strongly captures this runner until `runInternal` installs
        // and executes its conditional lease cleanup. UI close/onDisappear is
        // still the prompt synchronous path; this is only a cancellation
        // backstop after no active task/lease remains.
        runTask?.cancel()
    }

    func cancel() {
        // Cancel is a newer lifecycle command than any `run()` currently
        // unwinding a synchronous release/reset notification. Advance the same
        // epoch so that outer run cannot resume and acquire after cancellation.
        _ = runOwnership.beginInvocation()
        fenceCurrentRun(markCancelled: true)
    }

    func run(source: RunSource, samplingMode: FrameSource.SamplingMode) {
        // Register before fencing: releasing the predecessor publishes
        // synchronously and may invoke a newer `run()` on this same object.
        // That nested invocation must win; the outer call may not resume and
        // acquire over it merely because it started first.
        let invocation = runOwnership.beginInvocation()
        // Fence/cancel/restore the predecessor before acquiring the successor
        // lease. This also covers a Task cancelled before its closure starts.
        fenceCurrentRun(markCancelled: false)
        guard runOwnership.isLatestInvocation(invocation) else { return }
        let token = runOwnership.beginRun()
        let engineLease = nimble.acquireOfflinePolicyLease()
        // Acquisition performs a reset and its final notification may
        // synchronously start a replacement run. Never install the stale
        // returned lease over that successor.
        guard runOwnership.isCurrent(token),
              nimble.ownsOfflinePolicyLease(engineLease) else {
            retireUnlaunchedRun(token: token, engineLease: engineLease)
            return
        }
        enginePolicyLease = engineLease
        // Model/contact capability is the first admission boundary. A camera
        // pass cannot change an unsupported contact model, so never advertise
        // or pay for that secondary phase before this one is known.
        phase = .loadingModel
        frameBudgetNotice = nil
        objectWillChange.send()
        guard runOwnership.isCurrent(token),
              enginePolicyLease == engineLease,
              nimble.ownsOfflinePolicyLease(engineLease) else {
            retireUnlaunchedRun(token: token, engineLease: engineLease)
            return
        }
        runTask = Task { [self] in
            await runInternal(
                source: source,
                samplingMode: samplingMode,
                token: token,
                engineLease: engineLease
            )
        }
    }

    /// Retires a token that never reached its Task body. Local ownership and
    /// UI state commit before either notification below; a subscriber may
    /// synchronously start a successor without this failed launch clearing or
    /// overwriting it when the call unwinds.
    private func retireUnlaunchedRun(
        token: UInt64,
        engineLease: NimbleEngine.OfflinePolicyLease
    ) {
        let retired = runOwnership.complete(token)
        if enginePolicyLease == engineLease {
            enginePolicyLease = nil
        }
        if retired {
            phase = .finished(
                processed: resultStore.frames.count,
                failed: resultStore.frames.filter { $0.status != .success }.count,
                cancelled: true
            )
            objectWillChange.send()
        }
        _ = nimble.releaseOfflinePolicyLease(engineLease)
    }

    /// Synchronous lifecycle fence shared by Cancel, Close and run-to-run
    /// replacement. Invalidating ownership first removes the old task's defer
    /// authority before cancellation can resume it at a suspension point.
    private func fenceCurrentRun(markCancelled: Bool) {
        let invalidatedToken = runOwnership.invalidateCurrent()
        let lease = enginePolicyLease
        guard invalidatedToken != nil || lease != nil else { return }
        runTask?.cancel()
        runTask = nil
        if let lease {
            _ = nimble.releaseOfflinePolicyLease(lease)
            if enginePolicyLease == lease {
                enginePolicyLease = nil
            }
        }
        if markCancelled,
           runOwnership.activeToken == nil,
           enginePolicyLease == nil {
            phase = .finished(
                processed: resultStore.frames.count,
                failed: resultStore.frames.filter { $0.status != .success }.count,
                cancelled: true
            )
            objectWillChange.send()
        }
    }

    private func isRunActive(_ token: UInt64) -> Bool {
        guard runOwnership.isCurrent(token),
              !Task.isCancelled,
              let enginePolicyLease else { return false }
        return nimble.ownsOfflinePolicyLease(enginePolicyLease)
    }

    private func requireRunActive(_ token: UInt64) throws {
        guard runOwnership.isCurrent(token),
              let enginePolicyLease,
              nimble.ownsOfflinePolicyLease(enginePolicyLease) else {
            throw CancellationError()
        }
        try Task.checkCancellation()
    }

    private func setPhase(_ newPhase: RunPhase, for token: UInt64) {
        guard isRunActive(token) else { return }
        phase = newPhase
        objectWillChange.send()
    }

    private func setFrameBudgetNotice(
        _ newNotice: FrameBudgetNotice?,
        for token: UInt64
    ) {
        guard isRunActive(token) else { return }
        frameBudgetNotice = newNotice
        objectWillChange.send()
    }

    private func finishCancelledIfCurrent(
        _ token: UInt64,
        processed: Int,
        failed: Int
    ) {
        guard runOwnership.isCurrent(token) else { return }
        phase = .finished(
            processed: processed,
            failed: failed,
            cancelled: true
        )
        objectWillChange.send()
    }

    /// Converts offline camera evidence into the engine's source-independent
    /// authorization. Kept pure so every state is exhaustively testable without
    /// loading a model or starting a simulator solve.
    nonisolated static func cameraDynamicsAuthorization(
        for state: CameraReferenceState
    ) -> NimbleEngine.CameraDynamicsAuthorization {
        switch state {
        case .notRequiredForSingleFrame:
            return .init(
                permitsStaticEquilibrium: true,
                permitsTemporalDynamics: false
            )
        case .staticWithinBudget:
            return .unrestricted
        case .unmeasured, .moving, .betweenCalibrationBands,
             .calibrationRequired, .calibrationUnavailable, .indeterminate:
            return .denied
        }
    }

    // MARK: - Run

    private func runInternal(
        source: RunSource,
        samplingMode: FrameSource.SamplingMode,
        token: UInt64,
        engineLease: NimbleEngine.OfflinePolicyLease
    ) async {
        // Install cleanup before the first guard. Another Runner can acquire
        // the engine lease after `run()` launches this Task but before its body
        // starts; that stale task must still retire its local token/task state.
        defer {
            if runOwnership.isCurrent(token),
               enginePolicyLease == engineLease {
                _ = runOwnership.complete(token)
                enginePolicyLease = nil
                runTask = nil
                // Release publishes synchronously. All local ownership and
                // handles are already retired, so a successor started by that
                // notification cannot be cleared when this defer unwinds.
                _ = nimble.releaseOfflinePolicyLease(engineLease)
            }
        }
        guard isRunActive(token) else {
            finishCancelledIfCurrent(token, processed: 0, failed: 0)
            return
        }
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

        setPhase(.loadingModel, for: token)
        do {
            try await poseEstimator.loadModelIfNeeded()
        } catch {
            guard isRunActive(token) else {
                finishCancelledIfCurrent(token, processed: 0, failed: 0)
                return
            }
            setPhase(
                .failed("Couldn't load the pose model: \(error.localizedDescription)"),
                for: token
            )
            return
        }
        guard isRunActive(token) else {
            finishCancelledIfCurrent(token, processed: 0, failed: 0)
            return
        }

        let nimbleLoaded: Bool
        do {
            nimbleLoaded = try await waitForNimbleModelLoaded(token: token)
        } catch is CancellationError {
            finishCancelledIfCurrent(token, processed: 0, failed: 0)
            return
        } catch {
            guard isRunActive(token) else {
                finishCancelledIfCurrent(token, processed: 0, failed: 0)
                return
            }
            nimbleLoaded = false
        }
        guard isRunActive(token) else {
            finishCancelledIfCurrent(token, processed: 0, failed: 0)
            return
        }
        guard nimbleLoaded else {
            setPhase(.failed("Musculoskeletal model isn't loaded — FullBody.osim may be missing from the bundle, or is still loading. Try again in a moment."),
                     for: token)
            return
        }
        let hasValidatedFootContactSupport =
            nimble.hasValidatedFootContactSupport
        resultStore.setValidatedFootContactSupport(
            hasValidatedFootContactSupport)

        // Clip boundary — see this class's header comment.
        guard nimble.resetSessionState(offlinePolicyLease: engineLease) else {
            finishCancelledIfCurrent(token, processed: 0, failed: 0)
            return
        }
        guard isRunActive(token),
              enginePolicyLease == engineLease,
              nimble.ownsOfflinePolicyLease(engineLease) else { return }

        // Resolve the higher-priority capability before opening the native
        // video reader. The current bundled models stop at `.unmeasured`, and
        // a build without an exact versioned calibration stops at
        // `.calibrationUnavailable`; neither case performs Vision work that
        // cannot affect product output. Only a contact-valid, calibrated,
        // multi-frame source enters the bounded adapter.
        let isSingleFrame: Bool
        switch source {
        case .photo:
            isSingleFrame = true
        case .video:
            isSingleFrame = samplingMode == .singleFrame
        }
        let cameraAdmission = CameraReferenceAnalysisAdmission.decide(
            isSingleFrame: isSingleFrame,
            hasValidatedFootContactSupport: hasValidatedFootContactSupport,
            isVersionedCalibrationReady:
                cameraMotionAnalyzer.isVersionedCalibrationReady
        )
        let cameraState: CameraReferenceState
        if let resolvedState = cameraAdmission.resolvedState {
            cameraState = resolvedState
        } else {
            do {
                cameraState = try await resolveCameraReference(
                    source: source,
                    samplingMode: samplingMode,
                    token: token
                )
            } catch is CancellationError {
                finishCancelledIfCurrent(token, processed: 0, failed: 0)
                return
            } catch {
                guard isRunActive(token) else {
                    finishCancelledIfCurrent(token, processed: 0, failed: 0)
                    return
                }
                cameraState = .indeterminate(.videoReadFailed)
            }
        }
        guard isRunActive(token) else {
            finishCancelledIfCurrent(token, processed: 0, failed: 0)
            return
        }
        // Finalize the clip-level state before any FrameResult can enter the
        // store. A later upgrade cannot restore payloads stripped while the
        // state was unknown.
        resultStore.setCameraReferenceState(cameraState)
        // The store's post-commit `objectWillChange` delivery is synchronous. A
        // subscriber may start a replacement run while the call above is on
        // the stack; never let this now-stale task overwrite the successor's
        // denied default.
        guard isRunActive(token) else { return }
        nimble.cameraDynamicsAuthorization = Self.cameraDynamicsAuthorization(
            for: cameraState
        )

        let batch: DecodedBatch
        do {
            batch = try await decodeFrames(
                source: source,
                samplingMode: samplingMode,
                token: token
            )
        } catch is CancellationError {
            finishCancelledIfCurrent(token, processed: 0, failed: 0)
            return
        } catch {
            guard isRunActive(token) else {
                finishCancelledIfCurrent(token, processed: 0, failed: 0)
                return
            }
            setPhase(.failed(error.localizedDescription), for: token)
            return
        }
        guard isRunActive(token) else {
            finishCancelledIfCurrent(token, processed: 0, failed: 0)
            return
        }
        let decoded = batch.frames
        guard !decoded.isEmpty else {
            setPhase(.failed("No frames could be decoded from the selection."),
                     for: token)
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
            guard isRunActive(token) else {
                finishCancelledIfCurrent(
                    token,
                    processed: i,
                    failed: failureCount
                )
                return
            }
            let frameStart = CACurrentMediaTime()
            setPhase(
                .running(
                    current: i,
                    total: decoded.count,
                    etaSeconds: eta(remainingFrames: decoded.count - i)
                ),
                for: token
            )

            let succeeded = await processOneFrame(
                frame,
                frameIndex: i,
                sourceKind: sourceKind,
                firstRequestedFrameNumber: batch.firstRequestedFrameNumber,
                totalPushes: &totalPushes,
                token: token,
                engineLease: engineLease
            )
            guard isRunActive(token) else {
                finishCancelledIfCurrent(
                    token,
                    processed: i,
                    failed: failureCount
                )
                return
            }
            if !succeeded { failureCount += 1 }

            perFrameDurations.append(CACurrentMediaTime() - frameStart)
        }

        guard isRunActive(token) else {
            finishCancelledIfCurrent(
                token,
                processed: decoded.count,
                failed: failureCount
            )
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
            let padded = await padFilterTail(
                with: last.bodyFrame,
                totalPushes: &totalPushes,
                halfWindow: Self.sgHalfWindow,
                token: token
            )
            guard isRunActive(token) else {
                finishCancelledIfCurrent(
                    token,
                    processed: decoded.count,
                    failed: failureCount
                )
                return
            }
            if !padded,
               !endTemporalSegment(token: token, engineLease: engineLease) {
                finishCancelledIfCurrent(
                    token,
                    processed: decoded.count,
                    failed: failureCount
                )
                return
            }
        }

        // --- Is this a RUN? ---------------------------------------------------
        //
        // Only now, because a stride is not visible one frame at a time. The
        // first pass above has already produced every supported pose. A usable
        // run publishes detached contact timing. Only a future model/solver
        // with validated foot support may continue into the private second
        // pass; bundled models do not construct its plan or replay any frame.
        await runGaitPassIfThisIsARun(
            firstRequestedFrameNumber: batch.firstRequestedFrameNumber,
            lastRequestedFrameNumber: batch.lastRequestedFrameNumber,
            token: token
        )
        guard isRunActive(token) else {
            finishCancelledIfCurrent(
                token,
                processed: decoded.count,
                failed: failureCount
            )
            return
        }

        setPhase(
            .finished(
                processed: decoded.count,
                failed: failureCount,
                cancelled: false
            ),
            for: token
        )
    }

    // MARK: - Gait pass

    /// Runs `GaitAnalysis` over the whole window and publishes detached timing.
    /// A future capability-valid model may then re-solve stance frames inside
    /// the private dynamics branch.
    ///
    /// Everything here is guarded so a non-running clip — a photo, a squat, a
    /// subject standing still — costs one cheap analysis and changes nothing.
    private func runGaitPassIfThisIsARun(
        firstRequestedFrameNumber: Int?,
        lastRequestedFrameNumber: Int?,
        token: UInt64
    ) async {
        guard isRunActive(token) else { return }
        guard usableBodyFrames.count >= GaitSignal.minimumFrames else {
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

        let timingReport = GaitTimingReport(report: report)
        guard timingReport.isUsable else {
            // Refused BY THE CLIP'S OWN MODEL. The reasons are published so the
            // user is told what to film differently, not just that it failed.
            resultStore.setGait(.refused(report: timingReport))
            return
        }

        // Publish the supported result before considering any internal
        // dynamics work. This value owns only timestamp-derived timing. Both
        // bundled models stop here: neither has validated foot-support
        // mechanics, so making a force plan or replaying a second pass would
        // create unavailable values merely to discard them.
        let supportsFootContact = nimble.hasValidatedFootContactSupport
        let cameraPermitsTemporalDynamics =
            resultStore.cameraReferenceState.permitsTemporalDynamics
        if supportsFootContact && cameraPermitsTemporalDynamics {
            // Clear pass-one static physics before publishing `.analysed` so
            // no Combine observer can see an analysed gait result paired with
            // load values from the old policy. This also precedes every
            // load-plan refusal/return.
            resultStore.beginGaitReplacementPass()
        }
        resultStore.setGait(.analysed(report: timingReport))
        // Store publication can re-enter `run()`. Recheck before reading the
        // mutable lease property or changing any shared engine policy.
        guard isRunActive(token) else { return }
        guard supportsFootContact else { return }
        guard cameraPermitsTemporalDynamics else { return }

        // A derivative-fit refusal is specifically a dynamics-plan boundary;
        // it never takes an already measured timing report away.
        guard report.isUsable else { return }

        guard let plan = Self.makePlan(from: report) else {
            return
        }

        // Clip boundary between passes: the derivative window changes length
        // here (9 taps -> `plan.filterTaps`), and the IK warm start belongs to
        // the last frame of pass 1, not the first of pass 2.
        guard isRunActive(token), let enginePolicyLease,
              nimble.resetAnalysisPassStatePreservingGround(
                  offlinePolicyLease: enginePolicyLease
              ) else { return }
        // The reset publishes synchronously. A subscriber may replace this
        // run and acquire a new engine lease before the call unwinds; the old
        // task must not then overwrite its successor's policy.
        guard isRunActive(token),
              self.enginePolicyLease == enginePolicyLease,
              nimble.ownsOfflinePolicyLease(enginePolicyLease) else { return }
        nimble.staticHoldGating = false
        nimble.gaitPlan = plan
        defer {
            if isRunActive(token),
               self.enginePolicyLease == enginePolicyLease,
               nimble.ownsOfflinePolicyLease(enginePolicyLease) {
                nimble.gaitPlan = nil
                nimble.staticHoldGating = true
            }
        }

        let half = plan.filterTaps / 2
        var pushes = 0
        var processed = 0
        let segments = OfflineTemporalPolicy.segmentPlans(
            frameNumbers: usableBodyFrames.map(\.frameNumber),
            firstRequestedFrameNumber: firstRequestedFrameNumber,
            lastRequestedFrameNumber: lastRequestedFrameNumber
        )

        for segment in segments {
            guard isRunActive(token) else { return }
            if segment.resetsRealtimeStateBefore {
                // Queues filter clearing ahead of the next accepted solve on
                // the same FIFO solver queue and supersedes any old receipt.
                guard nimble.resetRealtimeState(
                          offlinePolicyLease: enginePolicyLease
                      ) else { return }
                guard isRunActive(token),
                      self.enginePolicyLease == enginePolicyLease,
                      nimble.ownsOfflinePolicyLease(enginePolicyLease) else { return }
            }

            let frames = usableBodyFrames[segment.frameIndices]
            if segment.padsHead, let first = frames.first {
                let primed = await primeFilterHead(
                    with: first,
                    totalPushes: &pushes,
                    halfWindow: half,
                    token: token
                )
                guard isRunActive(token) else { return }
                guard primed else { return }
            }

            for body in frames {
                guard isRunActive(token) else { return }
                setPhase(
                    .running(
                        current: processed,
                        total: usableBodyFrames.count,
                        etaSeconds: nil
                    ),
                    for: token
                )
                guard case .success(let receipt) = await submitAndWait(
                    body,
                    timeout: Self.solveTimeout,
                    token: token
                ) else { return }
                guard isRunActive(token) else { return }
                routeSolveToOwningFrame(token: token, receipt: receipt)
                pushes += 1
                processed += 1
            }

            if segment.padsTail, let last = frames.last {
                let padded = await padFilterTail(
                    with: last,
                    totalPushes: &pushes,
                    halfWindow: half,
                    token: token
                )
                guard isRunActive(token) else { return }
                guard padded else { return }
            }
        }

        guard isRunActive(token) else { return }
        resultStore.setGait(.analysed(report: timingReport))
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
        totalPushes: inout Int,
        token: UInt64,
        engineLease: NimbleEngine.OfflinePolicyLease
    ) async -> Bool {
        guard isRunActive(token) else { return false }
        // The tail candidate must describe the final requested slot itself.
        // Clearing it before any fallible work prevents a trailing pose failure
        // or review-only fallback from padding the previous trusted pose across
        // a known missing interval.
        lastSuccessfulFrame = nil

        let estimate: SAM3DPoseEstimator.Output
        do {
            estimate = try await poseEstimator.estimate(uiImage: frame.image)
        } catch {
            guard isRunActive(token) else { return false }
            guard endTemporalSegment(token: token, engineLease: engineLease) else {
                return false
            }
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
                status: .poseEstimationFailed(error.localizedDescription), usedFallbackBBox: false,
                camT: nil,
                modelChecksums: nil,
                bodyFrame: nil, ikResult: nil, idResult: nil, muscleResult: nil,
                dynamicsAvailability: .waitingForMotionWindow,
                isStaticHoldEstimate: false, motionState: .undetermined))
            return false
        }
        guard isRunActive(token) else { return false }

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
            guard endTemporalSegment(token: token, engineLease: engineLease) else {
                return false
            }
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
                dynamicsAvailability: .waitingForMotionWindow,
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
            guard endTemporalSegment(token: token, engineLease: engineLease) else {
                return false
            }
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
                status: .implausibleBody(reason: reason, hipWidthMeters: hip, statureMeters: stature),
                usedFallbackBBox: estimate.usedFallbackBBox,
                camT: estimate.camT,
                modelChecksums: (estimate.inputChecksum, estimate.outputChecksum,
                                 estimate.sourceHash, estimate.bboxHash, estimate.warpHash),
                bodyFrame: bodyFrame, ikResult: nil, idResult: nil, muscleResult: nil,
                dynamicsAvailability: .waitingForMotionWindow,
                isStaticHoldEstimate: false, motionState: .undetermined))
            return false
        }

        guard bodyFrame.joints.contains(where: \.isTracked) else {
            guard endTemporalSegment(token: token, engineLease: engineLease) else {
                return false
            }
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
                status: .poseEstimationFailed("retarget produced no usable joints"), usedFallbackBBox: estimate.usedFallbackBBox,
                camT: estimate.camT,
                modelChecksums: (estimate.inputChecksum, estimate.outputChecksum,
                                 estimate.sourceHash, estimate.bboxHash, estimate.warpHash),
                bodyFrame: bodyFrame, ikResult: nil, idResult: nil, muscleResult: nil,
                dynamicsAvailability: .waitingForMotionWindow,
                isStaticHoldEstimate: false, motionState: .undetermined))
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
            guard endTemporalSegment(token: token, engineLease: engineLease) else {
                return false
            }
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
            guard nimble.scaleModel(
                height: stature,
                markerPositions: positions,
                markerNames: names,
                offlinePolicyLease: engineLease
            ) else { return false }
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
            let primed = await primeFilterHead(
                with: bodyFrame,
                totalPushes: &totalPushes,
                halfWindow: Self.sgHalfWindow,
                token: token
            )
            guard isRunActive(token) else { return false }
            if !primed {
                // Keep the pose path usable, but start a fresh unpadded
                // temporal segment. Failed synthetic history must never be
                // treated as context for this real frame.
                guard endTemporalSegment(token: token, engineLease: engineLease) else {
                    return false
                }
                lastTemporalFrameNumber = bodyFrame.frameNumber
            }
        }

        let submission = await submitAndWait(
            bodyFrame,
            timeout: Self.solveTimeout,
            token: token
        )
        guard isRunActive(token) else { return false }
        usableBodyFrames.append(bodyFrame)

        guard case .success(let receipt) = submission else {
            let failure: OfflineResultStore.FrameStatus.SolverFailure
            if case .failure(let reason) = submission {
                failure = reason
            } else {
                preconditionFailure("Result must be success or failure")
            }
            guard endTemporalSegment(token: token, engineLease: engineLease) else {
                return false
            }
            resultStore.append(OfflineResultStore.FrameResult(
                id: frameIndex, sourceImage: frame.image, timestamp: frame.timestamp,
                status: .nimbleFailure(failure),
                usedFallbackBBox: estimate.usedFallbackBBox,
                camT: estimate.camT,
                modelChecksums: (estimate.inputChecksum, estimate.outputChecksum,
                                 estimate.sourceHash, estimate.bboxHash, estimate.warpHash),
                bodyFrame: bodyFrame, ikResult: nil, idResult: nil, muscleResult: nil,
                dynamicsAvailability: .waitingForMotionWindow,
                isStaticHoldEstimate: false, motionState: .undetermined))
            return false
        }
        totalPushes += 1

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
                muscleResult: nil, dynamicsAvailability: .waitingForMotionWindow,
                isStaticHoldEstimate: false, motionState: .undetermined))
        routeSolveToOwningFrame(token: token, receipt: receipt)
        return true
    }

    /// Ends the current trusted temporal segment without discarding clip-wide
    /// IK, ground, or QP warm starts.
    ///
    /// `resetRealtimeState()` synchronously clears published values, supersedes
    /// an exact in-flight receipt, and queues SG/hold clearing on NimbleEngine's
    /// serial solver queue. FIFO guarantees the clear precedes the next solve.
    private func endTemporalSegment(
        token: UInt64,
        engineLease: NimbleEngine.OfflinePolicyLease
    ) -> Bool {
        guard runOwnership.isCurrent(token),
              enginePolicyLease == engineLease,
              nimble.ownsOfflinePolicyLease(engineLease) else { return false }
        guard lastTemporalFrameNumber != nil else {
            lastSuccessfulFrame = nil
            return true
        }
        guard nimble.resetRealtimeState(
            offlinePolicyLease: engineLease
        ) else { return false }
        // Reset publishes synchronously. A Cancel, Close, another Runner, or a
        // replacement run may now own both the local token and engine policy.
        // The predecessor must not clear successor-local segment state.
        guard runOwnership.isCurrent(token),
              enginePolicyLease == engineLease,
              nimble.ownsOfflinePolicyLease(engineLease) else { return false }
        lastTemporalFrameNumber = nil
        lastSuccessfulFrame = nil
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
    /// Reading the atomic `SolveRecord` rather than assembling individual
    /// result fields is load-bearing now that a moving frame publishes IK with
    /// muscle = nil: a held display payload may still describe the previous
    /// HOLD, so pairing it with a fresh IK result would file stale muscle under
    /// a fresh pose. Everything in the record shares one timestamp.
    private func routeSolveToOwningFrame(
        token: UInt64,
        receipt: NimbleEngine.FrameReceipt
    ) {
        guard isRunActive(token) else { return }
        guard nimble.lastSolveReceipt == receipt else { return }
        guard let solve = nimble.lastSolve else { return }
        let t = solve.centerTimestamp
        guard let owner = resultStore.frames.min(by: {
            abs($0.timestamp - t) < abs($1.timestamp - t)
        }), abs(owner.timestamp - t) <= Self.biomechanicsMatchTolerance else { return }

        let motion = solve.motion
        // Gait gets its own case so stance/flight timing is not flattened into
        // stillness numbers, which mean nothing about a runner. The raw solve
        // may carry a private force/residual diagnostic; the product store
        // unconditionally strips it while retaining this verdict.
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
    /// matches what this input can actually support. It affects pose/motion
    /// classification only for the bundled models. A static-dynamics reading
    /// belongs exclusively to a future capability-valid branch; padding itself
    /// cannot turn an apparent photo hold into a measured load.
    ///
    /// ⚠️ These pads have ZERO marker displacement by construction, so the hold
    /// detector sees them as still. The verdict for the first real frame is
    /// therefore built from 4 real transitions after it and 4 synthetic zeros
    /// before it: whatever the subject was doing in the moment before the clip
    /// started is unobservable and is scored as stillness. The tail padding has
    /// the mirror-image bias. A single photo is the fully degenerate case —
    /// all 8 transitions are synthetic — which is why "one frame is a hold" is
    /// an ASSUMPTION this path inherits from the padding, not a measurement.
    private func primeFilterHead(
        with bodyFrame: BodyFrame,
        totalPushes: inout Int,
        halfWindow: Int,
        token: UInt64
    ) async -> Bool {
        let dt = sampleInterval
        guard halfWindow >= 1 else { return true }
        for step in stride(from: halfWindow, through: 1, by: -1) {
            guard isRunActive(token) else { return false }
            let padded = BodyFrame(timestamp: bodyFrame.timestamp - Double(step) * dt,
                                   frameNumber: bodyFrame.frameNumber,
                                   joints: bodyFrame.joints,
                                   dynamicsReference: bodyFrame.dynamicsReference)
            let submission = await submitAndWait(
                padded,
                timeout: Self.solveTimeout,
                token: token
            )
            guard isRunActive(token) else { return false }
            guard case .success = submission else { return false }
            totalPushes += 1
            // Deliberately not routed: these are centred on synthetic
            // timestamps that match no real frame.
        }
        return true
    }

    /// Mirror of `primeFilterHead` for a real, observed clip tail. Each push
    /// advances the window centre onto one of the last real frames, so unlike
    /// the head padding these results are routed and kept.
    private func padFilterTail(
        with bodyFrame: BodyFrame,
        totalPushes: inout Int,
        halfWindow: Int,
        token: UInt64
    ) async -> Bool {
        let dt = sampleInterval
        guard halfWindow >= 1 else { return true }
        for step in 1...halfWindow {
            guard isRunActive(token) else { return false }
            let padded = BodyFrame(timestamp: bodyFrame.timestamp + Double(step) * dt,
                                   frameNumber: bodyFrame.frameNumber,
                                   joints: bodyFrame.joints,
                                   dynamicsReference: bodyFrame.dynamicsReference)
            let submission = await submitAndWait(
                padded,
                timeout: Self.solveTimeout,
                token: token
            )
            guard isRunActive(token) else { return false }
            guard case .success(let receipt) = submission else { return false }
            totalPushes += 1
            routeSolveToOwningFrame(token: token, receipt: receipt)
        }
        return true
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
    private func waitForNimbleModelLoaded(
        timeout: TimeInterval = 10.0,
        token: UInt64
    ) async throws -> Bool {
        try requireRunActive(token)
        if nimble.isModelLoaded { return true }
        let deadline = CACurrentMediaTime() + timeout
        while CACurrentMediaTime() < deadline {
            try requireRunActive(token)
            if nimble.isModelLoaded { return true }
            try await Task.sleep(nanoseconds: 150_000_000)
            try requireRunActive(token)
        }
        try requireRunActive(token)
        return nimble.isModelLoaded
    }

    // MARK: - Frame decoding

    private func resolveCameraReference(
        source: RunSource,
        samplingMode: FrameSource.SamplingMode,
        token: UInt64
    ) async throws -> CameraReferenceState {
        try requireRunActive(token)
        guard case .video(let url) = source else {
            return .notRequiredForSingleFrame
        }
        guard samplingMode != .singleFrame else {
            return .notRequiredForSingleFrame
        }

        setPhase(.checkingCameraReference, for: token)
        let decoder = FrameSource.VideoDecoder(url: url)
        let duration = try await decoder.duration()
        try requireRunActive(token)
        let nominalRate = await decoder.nominalFrameRate()
        try requireRunActive(token)
        let (timestamps, _) = FrameSource.sampleTimestamps(
            duration: duration,
            mode: samplingMode,
            nominalFrameRate: nominalRate
        )
        guard let range = CameraAnalysisPolicy.analysisRange(
            requestedTimestamps: timestamps,
            assetDuration: duration,
            nominalFrameRate: nominalRate
        ), let derivativeWindow = CameraAnalysisPolicy.derivativeWindowSeconds(
            samplingMode: samplingMode,
            nominalFrameRate: nominalRate
        ) else {
            return .indeterminate(.insufficientCoverage)
        }
        let state = try await cameraMotionAnalyzer.analyzeVideo(
            at: url,
            range: range,
            derivativeWindowSeconds: derivativeWindow
        )
        try requireRunActive(token)
        return state
    }

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
        samplingMode: FrameSource.SamplingMode,
        token: UInt64
    ) async throws -> DecodedBatch {
        try requireRunActive(token)
        switch source {
        case .photo(let image):
            return DecodedBatch(frames: FrameSource.decodePhoto(image),
                                firstRequestedFrameNumber: 0,
                                lastRequestedFrameNumber: 0)

        case .video(let url):
            setPhase(.decodingFrames, for: token)
            let decoder = FrameSource.VideoDecoder(url: url)
            let duration = try await decoder.duration()
            try requireRunActive(token)
            let rate = await decoder.nominalFrameRate()
            try requireRunActive(token)
            let (timestamps, truncated) = FrameSource.sampleTimestamps(duration: duration,
                                                                      mode: samplingMode,
                                                                      nominalFrameRate: rate)
            setFrameBudgetNotice(
                FrameBudgetNotice.make(
                    mode: samplingMode,
                    duration: duration,
                    nominalFrameRate: rate,
                    timestamps: timestamps,
                    wasTruncated: truncated
                ),
                for: token
            )

            var frames: [FrameSource.DecodedFrame] = []
            frames.reserveCapacity(timestamps.count)
            for (i, t) in timestamps.enumerated() {
                try requireRunActive(token)
                // One undecodable timestamp shouldn't abort the whole clip.
                let image = try? await decoder.decodeFrame(at: t)
                try requireRunActive(token)
                if let image {
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
    /// Retries synchronous `.dropped` admissions. Accepted frames wait on their
    /// exact receipt; a timeout fences its generation immediately, so neither
    /// its late publish nor its in-flight cleanup can be attributed to a later
    /// frame.
    private func submitAndWait(
        _ bodyFrame: BodyFrame,
        timeout: TimeInterval,
        token: UInt64
    ) async -> Result<
        NimbleEngine.FrameReceipt,
        OfflineResultStore.FrameStatus.SolverFailure
    > {
        for _ in 0..<Self.maxSubmitAttempts {
            guard isRunActive(token) else { return .failure(.superseded) }
            guard let enginePolicyLease else { return .failure(.superseded) }
            let outcome = await NimbleFrameWaiter.submit(
                on: nimble,
                timeout: timeout,
                { [nimble] in
                    nimble.processFrame(
                        bodyFrame,
                        offlinePolicyLease: enginePolicyLease
                    )
                }
            )
            guard isRunActive(token) else { return .failure(.superseded) }
            switch outcome {
            case .published(let receipt):
                return .success(receipt)
            case .failed:
                return .failure(.solveFailed)
            case .timedOut:
                return .failure(.timedOut)
            case .superseded:
                return .failure(.superseded)
            case .rejected:
                return .failure(.admissionRejected)
            case .dropped:
                // The engine is still busy with an earlier solve. Back off and
                // resubmit rather than burning a timeout on a frame it never
                // accepted.
                do {
                    try await Task.sleep(
                        nanoseconds: UInt64(
                            Self.dropRetryDelay * 1_000_000_000
                        )
                    )
                } catch {
                    return .failure(.superseded)
                }
                guard isRunActive(token) else { return .failure(.superseded) }
            }
        }
        return .failure(.busy)
    }
}

/// Waits for the terminal event belonging to one exact accepted frame receipt.
/// Unrelated `@Published` changes, resets and older solves cannot satisfy it.
@MainActor
enum NimbleFrameWaiter {
    enum Outcome {
        /// The engine accepted the frame and published a result.
        case published(NimbleEngine.FrameReceipt)
        /// IK failed. The engine resets the affected segment before delivering
        /// this exact terminal event.
        case failed
        /// The local liveness deadline revoked this exact receipt.
        case timedOut
        /// An external reset or cancellation revoked this exact receipt (or
        /// cancellation happened before it was admitted).
        case superseded
        /// The engine refused the frame because a previous solve was still in
        /// flight. Nothing was computed and no receipt/completion exists.
        case dropped
        /// Admission rejected the frame before creating a receipt.
        case rejected
    }

    /// One-shot continuation state. MainActor confinement plus the `finished`
    /// guard makes timeout, completion and cancellation race safely: exactly
    /// one path resumes the continuation and tears down the others.
    private final class WaitState {
        private var continuation: CheckedContinuation<Outcome, Never>?
        private var expectedReceipt: NimbleEngine.FrameReceipt?
        private weak var engine: NimbleEngine?
        private var cancellable: AnyCancellable?
        private var timeoutWorkItem: DispatchWorkItem?
        private var finished = false

        func install(_ continuation: CheckedContinuation<Outcome, Never>) {
            precondition(self.continuation == nil)
            self.continuation = continuation
        }

        init(engine: NimbleEngine) {
            self.engine = engine
        }

        func observe(_ publisher: AnyPublisher<NimbleEngine.FrameCompletion, Never>) {
            cancellable = publisher.sink { [weak self] completion in
                guard let self,
                      completion.receipt == self.expectedReceipt else { return }
                switch completion.status {
                case .published:
                    self.finish(.published(completion.receipt))
                case .failed:
                    self.finish(.failed)
                case .superseded:
                    self.finish(.superseded)
                }
            }
        }

        func accepted(
            _ receipt: NimbleEngine.FrameReceipt,
            timeout: TimeInterval
        ) {
            expectedReceipt = receipt
            let workItem = DispatchWorkItem { [weak self] in
                self?.supersedeAndFinish(.timedOut)
            }
            timeoutWorkItem = workItem
            DispatchQueue.main.asyncAfter(deadline: .now() + timeout,
                                          execute: workItem)
        }

        func cancel() {
            supersedeAndFinish(.superseded)
        }

        private func supersedeAndFinish(_ outcome: Outcome) {
            // Revoke the exact publication on this same MainActor turn before
            // resuming. A solver callback queued behind timeout/cancellation
            // therefore cannot transiently publish after the terminal verdict.
            // Stop observing first: `supersedeFrame` synchronously emits its
            // engine-level `.superseded` completion. Without this ordering that
            // reentrant event would steal a local timeout's more precise
            // `.timedOut` reason before `finish(outcome)` runs.
            cancellable?.cancel()
            cancellable = nil
            if let expectedReceipt {
                _ = engine?.supersedeFrame(expectedReceipt)
            }
            finish(outcome)
        }

        func finish(_ outcome: Outcome) {
            guard !finished else { return }
            finished = true
            timeoutWorkItem?.cancel()
            timeoutWorkItem = nil
            cancellable?.cancel()
            cancellable = nil
            engine = nil
            let continuation = continuation
            self.continuation = nil
            continuation?.resume(returning: outcome)
        }
    }

    /// Subscribe before admission, then bind the stream to the returned exact
    /// receipt. Completion is dispatched on main, so it cannot run between the
    /// synchronous `submit()` return and `accepted(_:)` in this MainActor turn.
    static func submit(
        on engine: NimbleEngine,
        timeout: TimeInterval,
        _ submit: () -> NimbleEngine.FrameSubmission
    ) async -> Outcome {
        let state = WaitState(engine: engine)
        return await withTaskCancellationHandler {
            await withCheckedContinuation {
                (continuation: CheckedContinuation<Outcome, Never>) in
                state.install(continuation)
                guard !Task.isCancelled else {
                    state.cancel()
                    return
                }
                state.observe(engine.frameCompletionPublisher)
                switch submit() {
                case .accepted(let receipt):
                    state.accepted(receipt, timeout: timeout)
                case .dropped:
                    state.finish(.dropped)
                case .rejected:
                    state.finish(.rejected)
                }
            }
        } onCancel: {
            // Covers cancellation before continuation installation as well as
            // an in-flight wait. `finish` is idempotent, so the Task-isCancelled
            // check above and this hop may both arrive safely.
            Task { @MainActor in
                state.cancel()
            }
        }
    }
}
