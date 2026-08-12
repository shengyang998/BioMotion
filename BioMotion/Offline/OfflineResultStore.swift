import Foundation
import Combine
import UIKit

/// Accumulates per-frame results from one offline (photo/video import) run so
/// `OfflinePlaybackView` can scrub through them — during the run (frames are
/// appended incrementally, so playback can start before the batch finishes) and
/// after.
///
/// Ownership: one instance per run, created and reset by `OfflineSessionRunner`;
/// `OfflinePlaybackView` only reads it.
@MainActor
final class OfflineResultStore: ObservableObject {

    /// Store fields form one product-facing authorization snapshot. Individual
    /// `@Published` wrappers notify from `willSet`, which exposed stale loads
    /// beside a new camera/contact denial and let a synchronous subscriber's
    /// newer mutation be overwritten when the older setter resumed. Every
    /// mutation below commits all fields first, then sends exactly once.
    nonisolated let objectWillChange = ObservableObjectPublisher()

    enum FrameStatus: Equatable {
        enum PoseFailure: Equatable {
            case modelProcessing
            case noUsableJoints

            var publicDescription: String {
                switch self {
                case .modelProcessing:
                    return "The pose could not be estimated for this frame. Try a clearer, full-body view."
                case .noUsableJoints:
                    return "No usable body joints were found in this frame. Keep the full body visible and try again."
                }
            }
        }

        enum SolverFailure: Error, Equatable {
            /// An accepted solve exceeded the explicit per-frame liveness
            /// budget and its exact receipt was superseded.
            case timedOut
            /// The native IK bridge returned no solution for this frame.
            case solveFailed
            /// The engine rejected admission (for example because the caller
            /// no longer owned the active offline policy lease).
            case admissionRejected
            /// Every bounded retry found the physical solver still occupied.
            case busy
            /// A reset superseded the exact receipt while the run itself was
            /// still current. This is distinct from a timeout and from IK
            /// failure so diagnostics never prescribe the wrong remedy.
            case superseded

            var userFacingDescription: String {
                switch self {
                case .timedOut:
                    return "Solver timed out on this frame"
                case .solveFailed:
                    return "The skeleton solver could not fit this frame"
                case .admissionRejected:
                    return "The solver could not accept this frame"
                case .busy:
                    return "The solver was still busy with an earlier frame"
                case .superseded:
                    return "The solver session changed before this frame completed"
                }
            }
        }

        case success
        case poseEstimationFailed(PoseFailure)
        /// The pose model returned a full skeleton, but one whose BODY SIZE is
        /// not a person's — see `MHRRetarget.plausibility`. Rejected before it
        /// could scale the musculoskeletal model, and reported with the measured
        /// number rather than dropped, because a frame that vanishes without a
        /// reason is indistinguishable from a crash.
        case implausibleBody(failure: MHRRetarget.PlausibilityFailure,
                             hipWidthMeters: Double,
                             statureMeters: Double)
        /// The pose existed but the musculoskeletal solve did not publish.
        /// Preserve the exact terminal cause so a deterministic IK failure,
        /// admission refusal, busy engine, reset and real timeout never collapse
        /// into the same user-facing diagnosis.
        case nimbleFailure(SolverFailure)

        /// The sentence shown for `.implausibleBody`, nil for every other case.
        ///
        /// It lives on the model rather than inside `OfflinePlaybackView`'s
        /// private `statusText` so it can be tested: this string is the ONLY
        /// thing the user gets when a frame is rejected, and a frame that
        /// disappears without a number is indistinguishable from a crash.
        var implausibleBodyDescription: String? {
            guard case .implausibleBody(let failure, let hip, let stature) = self else { return nil }
            // BOTH measurements are shown whichever bound tripped, so the user
            // can see the whole prediction rather than the one number that
            // happened to fail first.
            return failure.publicDescription(
                hipWidthMeters: hip,
                statureMeters: stature
            )
        }
    }

    /// What the static-hold detector concluded about this frame — the reason a
    /// frame does or does not carry muscle magnitudes.
    ///
    /// This exists because "no muscle data" had causes that looked identical in
    /// the UI: the Savitzky-Golay window had not filled yet (a startup
    /// artifact, harmless), versus the detector declining the instant (a
    /// statement about what this input can support). See
    /// `NimbleEngine.staticHoldGating`.
    ///
    /// Carries `NimbleEngine.MotionVerdict` rather than restating it. The two
    /// used to be parallel taxonomies — `hold`/`moving` here against the
    /// engine's four cases — which meant the UI could only ever say "moving",
    /// and the engine's distinction between *the subject moved* and *the pose
    /// estimate is too noisy to tell* was discarded at this boundary. Those are
    /// different problems with different fixes, and only one of them is
    /// something the user did.
    enum MotionState: Equatable {
        case measured(verdict: NimbleEngine.MotionVerdict,
                      peakSpeedMetersPerSecond: Double,
                      windowSeconds: Double,
                      noiseFloorMetersPerSecond: Double)
        /// RUNNING. The gait cycle, not the stillness test, decided this frame.
        /// The optional native outcome is accepted at the runner seam so raw
        /// diagnostics remain testable, but every value entering this
        /// published store is projected through `withoutGaitLoadEvidence`.
        ///
        /// A separate CASE rather than a separate TYPE: the vocabulary the UI
        /// renders stays one enum wide, which is the whole reason
        /// `NimbleEngine.MotionVerdict` is carried here instead of restated.
        case gait(verdict: NimbleEngine.MotionVerdict,
                  outcome: NimbleEngine.GaitFrameOutcome?)
        /// Nothing reached the detector for this frame: the filter was still
        /// warming up, the solve failed, or nothing was ever routed to it.
        case undetermined

        var peakSpeedMetersPerSecond: Double? {
            switch self {
            case .measured(_, let v, _, _): return v
            case .gait, .undetermined: return nil
            }
        }

        var isHold: Bool {
            switch self {
            case .measured(let verdict, _, _, _): return verdict == .hold
            case .gait, .undetermined: return false
            }
        }

        var verdict: NimbleEngine.MotionVerdict? {
            switch self {
            case .measured(let v, _, _, _): return v
            case .gait(let v, _): return v
            case .undetermined: return nil
            }
        }

        var gaitOutcome: NimbleEngine.GaitFrameOutcome? {
            if case .gait(_, let outcome) = self { return outcome }
            return nil
        }

        /// Removes force/residual evidence while retaining the report-neutral
        /// contact classification. Published mean contact timing/count belongs
        /// to the detached `GaitTimingReport`; the optional outcome here is
        /// downstream dynamics and must disappear at the product boundary.
        var withoutGaitLoadEvidence: MotionState {
            guard case .gait(let verdict, _) = self else { return self }
            return .gait(verdict: verdict, outcome: nil)
        }
    }

    /// Why a successfully estimated pose must not enter any calculation that
    /// depends on neighbouring frames or on a musculoskeletal solve.
    ///
    /// This is deliberately separate from `usedFallbackBBox`: a photo has no
    /// temporal neighbours and the whole-image fallback remains an analysable
    /// still pose. In a video, however, Vision missing the person is a real
    /// discontinuity. The fallback pose stays visible for review but may not
    /// calibrate the model or enter IK, derivatives, gait, ID, or muscle work.
    enum TemporalAnalysisExclusion: Equatable {
        case videoVisionWholeFrameFallback

        var badgeTitle: String {
            "Pose only — excluded from motion analysis"
        }

        var badgeDetail: String {
            "Vision found no person box; the full-frame fallback pose is shown for review "
                + "and was not used for scale, motion, gait, or muscle calculations."
        }
    }

    struct FrameResult: Identifiable {
        let id: Int  // frame index — stable, matches the scrubber position
        let sourceImage: UIImage
        let timestamp: TimeInterval
        let status: FrameStatus
        /// True if person detection found nobody and preprocessing fell back to
        /// the whole image. The source-specific policy is carried separately:
        /// photos remain analysable; video fallback frames are review-only.
        let usedFallbackBBox: Bool
        /// Immutable admission provenance. The explicit initializer below gives
        /// this a source-compatible nil default without making it mutable after
        /// the frame enters the store.
        let temporalAnalysisExclusion: TemporalAnalysisExclusion?

        // Present only on `.success`. `muscleResult` (and therefore
        // `hasFullBiomechanics`) can still be nil on a `.success` frame: the
        // Savitzky-Golay filter in NimbleEngine needs 9 pushes before ANY
        // muscle/ID output exists (see OfflineSessionRunner), so early frames in
        // a clip legitimately show pose-only results.
        /// Camera translation the model predicted for this frame. Needed to
        /// project the 3-D joints back onto `sourceImage` through the model's
        /// own camera — see `MHRRetarget.projectToImage`.
        let camT: SIMD3<Float>?
        /// Model input/output checksums for the on-device vs Mac comparison.
        /// See `SAM3DPoseEstimator.Output.inputChecksum`.
        let modelChecksums: (input: UInt64, output: UInt64,
                             source: UInt64, bbox: UInt64, warp: UInt64)?
        let bodyFrame: BodyFrame?
        let ikResult: NimbleEngine.IKOutput?
        let idResult: NimbleEngine.IDOutput?
        let muscleResult: NimbleEngine.MuscleOutput?
        /// Same-generation explanation for `idResult`. Numeric dynamics UI is
        /// permitted only for `.available`; every other case is pose-only with
        /// a named reason rather than a zero sentinel.
        let dynamicsAvailability: NimbleEngine.DynamicsAvailability
        /// True iff this frame's ID solve (and its muscle solve, when present)
        /// used a STATIC EQUILIBRIUM problem (q̇ = q̈ = 0) on a detected
        /// hold — i.e. any muscle numbers are a posture estimate, not a
        /// measurement of dynamics.
        ///
        /// It used to mean "the end-of-clip padding replayed this pose", which
        /// was true of the last four frames of every clip regardless of whether
        /// the subject had moved, so it distinguished nothing.
        let isStaticHoldEstimate: Bool
        /// Why this frame does or does not carry muscle data.
        let motionState: MotionState

        init(
            id: Int,
            sourceImage: UIImage,
            timestamp: TimeInterval,
            status: FrameStatus,
            usedFallbackBBox: Bool,
            temporalAnalysisExclusion: TemporalAnalysisExclusion? = nil,
            camT: SIMD3<Float>?,
            modelChecksums: (input: UInt64, output: UInt64,
                             source: UInt64, bbox: UInt64, warp: UInt64)?,
            bodyFrame: BodyFrame?,
            ikResult: NimbleEngine.IKOutput?,
            idResult: NimbleEngine.IDOutput?,
            muscleResult: NimbleEngine.MuscleOutput?,
            dynamicsAvailability: NimbleEngine.DynamicsAvailability,
            isStaticHoldEstimate: Bool,
            motionState: MotionState
        ) {
            self.id = id
            self.sourceImage = sourceImage
            self.timestamp = timestamp
            self.status = status
            self.usedFallbackBBox = usedFallbackBBox
            self.temporalAnalysisExclusion = temporalAnalysisExclusion
            self.camT = camT
            self.modelChecksums = modelChecksums
            self.bodyFrame = bodyFrame
            self.ikResult = ikResult
            self.idResult = idResult
            self.muscleResult = muscleResult
            self.dynamicsAvailability = dynamicsAvailability
            self.isStaticHoldEstimate = isStaticHoldEstimate
            self.motionState = motionState
        }

        var isEligibleForTemporalAnalysis: Bool { temporalAnalysisExclusion == nil }

        /// Whether a successful pose contains anything the non-AR scene can use
        /// to draw the anatomy layer. Muscle capsules are fixed anatomy, not a
        /// rendering of ID/muscle magnitudes, so dynamics availability does not
        /// enter this gate.
        var hasDrawableAnatomy: Bool {
            guard case .success = status, let bodyFrame else { return false }
            return bodyFrame.joints.contains(where: \.isTracked)
        }

        var hasFullBiomechanics: Bool {
            guard case .success = status else { return false }
            return isEligibleForTemporalAnalysis
                && hasValidatedDynamicsPayload
                && muscleResult != nil
        }

        /// Raw/research consistency predicate: `.available` is backed by an ID
        /// payload. This is not product authorization. Product frames must also
        /// pass the store projector's session-capability, envelope, tracked-body,
        /// IK, and same-generation timestamp checks before they can retain ID.
        var hasValidatedDynamicsPayload: Bool {
            dynamicsAvailability.hasInverseDynamics && idResult != nil
        }
        /// Pose was solved fine, but the detector could not certify a still
        /// instant, so no muscle magnitudes are claimed. Distinct from the
        /// warm-up case — and deliberately NOT named "because moving": one of
        /// the two reasons is that the pose estimate is too noisy to tell,
        /// which is the app's limitation and not the subject's.
        var isPoseOnlyBecauseNotStill: Bool {
            guard case .success = status else { return false }
            guard isEligibleForTemporalAnalysis else { return false }
            guard case .measured(let verdict, _, _, _) = motionState else { return false }
            return verdict != .hold && muscleResult == nil
        }

        /// True on a running clip's stance frames — the ones whose muscle
        /// numbers came from the gait cycle rather than from a static hold.
        var isGaitStance: Bool {
            guard case .success = status else { return false }
            return isEligibleForTemporalAnalysis && motionState.verdict == .gaitStance
        }

        /// **Whether THIS frame's muscle numbers may be drawn.**
        ///
        /// `GaitLoadSummary.make` already discards individual frames that fail
        /// `isUsableForLoadComparison` — a derivative window fitted across a
        /// touchdown, or two contact detectors that disagree about which foot is
        /// down — but the 3-D overlay and the per-frame caption were gated only
        /// CLIP-wide, by `summary.arePublishable`. Measured on the pinned
        /// fixtures: 52 of 64 stance frames on `video_012` and 44 of 68 on
        /// `video_015` have a derivative window crossing a contact edge, so a
        /// user scrubbing a clip that passed every clip-level gate saw coloured
        /// muscle capsules on 65-81 % of stance frames whose ddq was fitted
        /// across a discontinuity — the exact defect `WindowedDerivativeFilter`
        /// exists to prevent — with no marker of any kind, while the panel's
        /// ranked list had silently discarded them. The overlay reads as more
        /// authoritative than the list.
        ///
        /// Off the running path this is always true: a still-pose clip carries
        /// no gait outcome and is governed by the static-hold gate as before.
        var gaitLoadsAreComparable: Bool {
            if isGaitStance && !hasValidatedDynamicsPayload { return false }
            guard let outcome = motionState.gaitOutcome else { return true }
            return outcome.isUsableForLoadComparison
        }

        /// Why this frame's loads are not comparable, in the user's terms. Nil
        /// when they are. Each case names a different lever, which is why they
        /// are not collapsed into one sentence.
        var gaitExclusionReason: String? {
            if isGaitStance && !hasValidatedDynamicsPayload {
                return dynamicsAvailability.hasInverseDynamics
                    ? NimbleEngine.DynamicsAvailability.inverseDynamicsFailed.detail
                    : dynamicsAvailability.detail
            }
            guard let outcome = motionState.gaitOutcome, !outcome.isUsableForLoadComparison else {
                return nil
            }
            // Defensive, and deliberately so. `NimbleEngine` routes a flight
            // frame through `publishPoseOnly`, so today a flight frame carries
            // NO outcome and never reaches here — but that is a policy in a file
            // this type does not own, and if it ever publishes one, "too close
            // to a touchdown" would be the wrong sentence for it.
            if outcome.contactSide == 0 {
                return "both feet off the ground — no contact load here"
            }
            if outcome.solverSawDoubleContact {
                return "the solver put ground force under BOTH feet, so this frame's load is "
                     + "split between them"
            }
            if !outcome.contactDetectorsAgree {
                return "the foot's height above the ground disagrees that it was planted — "
                     + "solved with no ground force"
            }
            return "too close to a touchdown or toe-off to differentiate"
        }
    }

    /// One generation of output from one `NimbleEngine.SolveRecord`.
    ///
    /// IK is non-optional because a published solve always contains it. ID and
    /// muscle are optional by policy; nil means this generation withheld that
    /// output and must erase any older generation stored on the same frame.
    struct BiomechanicsPayload {
        let ikResult: NimbleEngine.IKOutput
        let idResult: NimbleEngine.IDOutput?
        let muscleResult: NimbleEngine.MuscleOutput?
        let dynamicsAvailability: NimbleEngine.DynamicsAvailability
        let isStaticHoldEstimate: Bool
        let motionState: MotionState

        init(ikResult: NimbleEngine.IKOutput,
             idResult: NimbleEngine.IDOutput?,
             muscleResult: NimbleEngine.MuscleOutput?,
             dynamicsAvailability: NimbleEngine.DynamicsAvailability,
             isStaticHoldEstimate: Bool,
             motionState: MotionState) {
            // Deliberately accept contradictory inputs here. This is the
            // orchestration seam, and the store's one projection function must
            // prove it fails closed for `.available + nil ID`, stale muscle,
            // or a session whose contact capability is false/unknown.
            self.ikResult = ikResult
            self.idResult = idResult
            self.muscleResult = muscleResult
            self.dynamicsAvailability = dynamicsAvailability
            self.isStaticHoldEstimate = isStaticHoldEstimate
            self.motionState = motionState
        }
    }

    private(set) var frames: [FrameResult] = []
    private(set) var selectedIndex: Int = 0
    /// What the gait pass concluded about the clip as a whole, or why it never
    /// ran. Nil until the batch finishes.
    private(set) var gait: GaitOutcome?
    /// Model/session capability, independent of any individual frame verdict.
    /// `nil` means the model has not finished loading for this session. `false`
    /// is permanent for both bundled models and must remain visible beside
    /// moving/flight/outside verdicts, none of which can unlock load mechanics.
    private(set) var hasValidatedFootContactSupport: Bool?
    /// Clip/session-level camera evidence. It is separate from `MotionState`,
    /// which describes the subject. The default is explicit and fail-closed;
    /// the runner finalizes it before appending the first frame.
    private(set) var cameraReferenceState: CameraReferenceState = .unmeasured

    /// What `GaitAnalysis` concluded about this clip.
    enum GaitAttemptFailure: Equatable {
        case insufficientFrames(usable: Int, required: Int)
        case analysisFailed

        var publicMessage: String {
            switch self {
            case .insufficientFrames(let usable, let required):
                return "This clip has \(usable) usable frames; running analysis needs at least \(required). Record a longer, clear running clip."
            case .analysisFailed:
                return "Running could not be analysed from this clip. Try a clear running clip with the full body visible."
            }
        }
    }

    enum GaitOutcome {
        /// Not a run, or not enough of one to try. The typed failure exposes
        /// stable actionable copy without carrying an analysis error string.
        case notAttempted(failure: GaitAttemptFailure)
        /// A run, but the clip's own model refused it. Every refusal carries the
        /// number that produced it.
        case refused(report: GaitTimingReport)
        /// A usable kinematic run. The published value is deliberately unable
        /// to carry a force hypothesis, dynamics plan, residual, or load.
        case analysed(report: GaitTimingReport)

        var report: GaitTimingReport? {
            switch self {
            case .notAttempted: return nil
            case .refused(let r): return r
            case .analysed(let r): return r
            }
        }

        /// True only when the clip really was a run. The gait pass runs on
        /// EVERY clip and declines most of them, so a UI that keyed off "a gait
        /// outcome exists" would put a sentence about strides in front of every
        /// imported photo.
        var isAboutRunning: Bool {
            if case .notAttempted = self { return false }
            return true
        }

        /// **Whether the gait screen may take the posture findings' place.**
        ///
        /// Only an ANALYSED run may. A refusal means the running analysis
        /// produced nothing, and it used to take the posture findings down with
        /// it: `isAboutRunning` is true for every refusal, including
        /// `.notRunning`, whose entire meaning is "this is not running". A user
        /// filming themselves side-on holding a squat — the app's stated purpose
        /// — got a panel headed "Running, but withheld" reading "only 0 complete
        /// contacts", and the measurements they came for, which had been
        /// computed and were sitting in this store, were simply not on screen.
        /// A walking clip took the same path.
        ///
        /// So a refusal is now shown BESIDE the findings, not instead of them.
        var replacesPostureFindings: Bool {
            if case .analysed = self { return true }
            return false
        }
    }

    func setGait(_ outcome: GaitOutcome) {
        gait = outcome
        objectWillChange.send()
    }

    func setValidatedFootContactSupport(_ isValidated: Bool) {
        let projectedFrames = frames.map {
            productProjectedFrame($0, contactSupportCapability: isValidated)
        }
        frames = projectedFrames
        hasValidatedFootContactSupport = isValidated
        objectWillChange.send()
    }

    func setCameraReferenceState(_ state: CameraReferenceState) {
        let projectedFrames = frames.map {
            productProjectedFrame($0, cameraReference: state)
        }
        frames = projectedFrames
        cameraReferenceState = state
        objectWillChange.send()
    }

    func reset() {
        frames = []
        selectedIndex = 0
        gait = nil
        hasValidatedFootContactSupport = nil
        cameraReferenceState = .unmeasured
        objectWillChange.send()
    }

    /// Appends a new frame result and pins the scrubber to it — while a run is
    /// in progress this keeps playback following the newest processed frame;
    /// the user can still drag the scrubber back at any time.
    func append(_ result: FrameResult) {
        frames.append(productProjectedFrame(result))
        selectedIndex = frames.count - 1
        objectWillChange.send()
    }

    /// User-driven scrubber selection follows the same post-commit publication
    /// rule as batch mutations. Ignore stale indices from a view update that
    /// raced a reset instead of manufacturing an out-of-range selection.
    func selectFrame(at index: Int) {
        guard frames.indices.contains(index), selectedIndex != index else { return }
        selectedIndex = index
        objectWillChange.send()
    }

    /// The single product publication seam used by append, replacement, and a
    /// late session-capability downgrade.
    ///
    /// Immediate per-frame absence reasons survive: moving, flight, outside
    /// analysis, warm-up, and pose failure remain actionable. Only a claimed
    /// `.available` payload is remapped, because it is contradictory unless
    /// the frame succeeded, is temporally eligible, and carries a solved IK pose
    /// backed by a tracked body frame at the owner timestamp. ID and muscle each
    /// have an additional same-generation timestamp gate; stale load payloads
    /// cannot erase a valid pose or its report-neutral motion verdict.
    private func productProjectedFrame(
        _ input: FrameResult,
        contactSupportCapability: Bool? = nil,
        cameraReference: CameraReferenceState? = nil
    ) -> FrameResult {
        let capability: Bool? = contactSupportCapability != nil
            ? contactSupportCapability
            : hasValidatedFootContactSupport
        let camera = cameraReference ?? cameraReferenceState
        // A caller-provided static flag is not enough to obtain the single-image
        // exception. It must agree with the same-generation motion verdict that
        // actually classified the solve as a hold. Contradictory orchestration
        // inputs fail closed instead of being reinterpreted as temporal output.
        let isConsistentStaticHold = input.isStaticHoldEstimate
            && input.motionState.isHold
        let solveClass: NimbleEngine.DynamicsSolveClass = isConsistentStaticHold
            ? .staticEquilibrium
            : .temporal
        let cameraPermitsThisSolve = input.isStaticHoldEstimate
            ? isConsistentStaticHold && camera.permitsStaticEquilibrium
            : camera.permitsTemporalDynamics
        let dynamicsReference = input.bodyFrame?.dynamicsReference ?? .unmeasured
        let hasGravityReference = dynamicsReference.gravity == .gravityAligned
        let referencePermitsThisSolve = dynamicsReference.permits(solveClass)
        let timestampTolerance: TimeInterval = 0.001
        func timestampsMatch(_ lhs: TimeInterval, _ rhs: TimeInterval) -> Bool {
            lhs.isFinite && rhs.isFinite && abs(lhs - rhs) <= timestampTolerance
        }
        let hasPoseProvenance: Bool = {
            guard case .success = input.status else { return false }
            guard input.isEligibleForTemporalAnalysis,
                  let ikResult = input.ikResult,
                  let bodyFrame = input.bodyFrame,
                  bodyFrame.joints.contains(where: \.isTracked),
                  timestampsMatch(bodyFrame.timestamp, input.timestamp) else {
                return false
            }
            return timestampsMatch(ikResult.timestamp, input.timestamp)
        }()
        let hasSameGenerationID: Bool = {
            guard hasPoseProvenance,
                  let ikResult = input.ikResult,
                  let idResult = input.idResult else { return false }
            return timestampsMatch(idResult.timestamp, ikResult.timestamp)
        }()
        let hasSameGenerationMuscle: Bool = {
            guard hasSameGenerationID,
                  let ikResult = input.ikResult,
                  let muscleResult = input.muscleResult else { return false }
            return timestampsMatch(muscleResult.timestamp, ikResult.timestamp)
        }()

        let mappedAvailability: NimbleEngine.DynamicsAvailability
        if input.dynamicsAvailability.hasInverseDynamics {
            if !hasPoseProvenance {
                mappedAvailability = .withheld(.noMeasurement)
            } else if let capability {
                if !capability {
                    mappedAvailability = .contactSupportUnavailable
                } else if !cameraPermitsThisSolve {
                    mappedAvailability = .cameraReferenceUnavailable
                } else if !hasGravityReference {
                    mappedAvailability = .gravityReferenceUnavailable
                } else if !referencePermitsThisSolve {
                    mappedAvailability = .rootTrajectoryUnavailable
                } else {
                    mappedAvailability = .productFacing(
                        current: input.dynamicsAvailability,
                        hasValidatedFootContactSupport: true,
                        hasInverseDynamicsPayload: hasSameGenerationID)
                }
            } else {
                // Frames cannot precede model/capability establishment in the
                // normal runner. If an adversarial caller does so, use the
                // existing explicit startup absence rather than trusting ID.
                mappedAvailability = .waitingForMotionWindow
            }
        } else {
            mappedAvailability = input.dynamicsAvailability
        }

        let retainsID = hasPoseProvenance
            && capability == true
            && cameraPermitsThisSolve
            && hasGravityReference
            && referencePermitsThisSolve
            && mappedAvailability.hasInverseDynamics
            && hasSameGenerationID
        let retainsMuscle = retainsID && hasSameGenerationMuscle

        return FrameResult(
            id: input.id,
            sourceImage: input.sourceImage,
            timestamp: input.timestamp,
            status: input.status,
            usedFallbackBBox: input.usedFallbackBBox,
            temporalAnalysisExclusion: input.temporalAnalysisExclusion,
            camT: input.camT,
            modelChecksums: input.modelChecksums,
            bodyFrame: input.bodyFrame,
            ikResult: hasPoseProvenance ? input.ikResult : nil,
            idResult: retainsID ? input.idResult : nil,
            muscleResult: retainsMuscle ? input.muscleResult : nil,
            dynamicsAvailability: mappedAvailability,
            isStaticHoldEstimate: retainsID && isConsistentStaticHold,
            motionState: hasPoseProvenance
                ? input.motionState.withoutGaitLoadEvidence
                : .undetermined)
    }

    /// Atomically replaces frame `id`'s biomechanics generation.
    ///
    /// The Savitzky-Golay window is centred, so a solve never describes the
    /// frame that was just pushed — `OfflineSessionRunner` matches on the
    /// solve's own timestamp and calls this to file it against the frame it
    /// actually belongs to. A gait pass may replace a static-pass solve, so all
    /// solve fields travel together: optional nil values erase the old ID,
    /// muscle, and gait-load evidence rather than mixing two generations.
    func replaceBiomechanics(
        forFrameID id: Int,
        with payload: BiomechanicsPayload
    ) {
        guard let index = frames.firstIndex(where: { $0.id == id }),
              frames[index].isEligibleForTemporalAnalysis else { return }
        let existing = frames[index]
        let candidate = FrameResult(
            id: existing.id,
            sourceImage: existing.sourceImage,
            timestamp: existing.timestamp,
            status: existing.status,
            usedFallbackBBox: existing.usedFallbackBBox,
            temporalAnalysisExclusion: existing.temporalAnalysisExclusion,
            camT: existing.camT,
            modelChecksums: existing.modelChecksums,
            bodyFrame: existing.bodyFrame,
            ikResult: payload.ikResult,
            idResult: payload.idResult,
            muscleResult: payload.muscleResult,
            dynamicsAvailability: payload.dynamicsAvailability,
            isStaticHoldEstimate: payload.isStaticHoldEstimate,
            motionState: payload.motionState
        )
        frames[index] = productProjectedFrame(candidate)
        objectWillChange.send()
    }

    /// Begins a gait re-solve by invalidating every eligible pass-one dynamics
    /// payload before pass two can publish anything.
    ///
    /// Successful centred solves replace this marker atomically. A timeout,
    /// cancellation, missing window centre, or unrouted publication leaves the
    /// explicit `.analysisPassIncomplete` reason instead of silently retaining
    /// static torques/muscle output from a different analysis policy. Pose and
    /// source/model provenance remain available for review.
    func beginGaitReplacementPass() {
        for index in frames.indices where frames[index].isEligibleForTemporalAnalysis {
            let existing = frames[index]
            frames[index] = FrameResult(
                id: existing.id,
                sourceImage: existing.sourceImage,
                timestamp: existing.timestamp,
                status: existing.status,
                usedFallbackBBox: existing.usedFallbackBBox,
                temporalAnalysisExclusion: existing.temporalAnalysisExclusion,
                camT: existing.camT,
                modelChecksums: existing.modelChecksums,
                bodyFrame: existing.bodyFrame,
                ikResult: existing.ikResult,
                idResult: nil,
                muscleResult: nil,
                dynamicsAvailability: .analysisPassIncomplete,
                isStaticHoldEstimate: false,
                motionState: existing.motionState.withoutGaitLoadEvidence
            )
        }
        objectWillChange.send()
    }

    var selectedFrame: FrameResult? {
        frames.indices.contains(selectedIndex) ? frames[selectedIndex] : nil
    }

    var successCount: Int {
        frames.filter { if case .success = $0.status { return true } else { return false } }.count
    }

    /// Frames rejected by the body-size gate. Surfaced separately from
    /// `poseEstimationFailed` because the fix is different: the model DID find a
    /// person, they were just too small or too occluded in frame to measure.
    var implausibleBodyCount: Int {
        frames.filter { if case .implausibleBody = $0.status { return true } else { return false } }.count
    }

    var biomechanicsCount: Int { frames.filter(\.hasFullBiomechanics).count }

    /// Successful poses whose ID/GRF/muscle payload was withheld specifically
    /// because the rolling floor had not reached its trust threshold.
    var groundUntrustedCount: Int {
        frames.filter { $0.dynamicsAvailability == .groundPlaneUntrusted }.count
    }

    /// Successful poses whose loads are unavailable because the loaded model
    /// and native solver have no validated foot-support representation. This is
    /// a permanent capability boundary for the current model, not warm-up or a
    /// filming problem.
    var contactSupportUnavailableCount: Int {
        frames.filter { $0.dynamicsAvailability == .contactSupportUnavailable }.count
    }

    /// Frames whose pose was solved but whose muscle numbers were withheld
    /// because the subject was moving. Surfaced so "few frames have muscle
    /// data" reads as a property of the clip rather than as a solver failure.
    var poseOnlyNotStillCount: Int { frames.filter(\.isPoseOnlyBecauseNotStill).count }
}
