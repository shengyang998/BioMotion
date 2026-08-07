import Foundation
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

    enum FrameStatus: Equatable {
        case success
        case poseEstimationFailed(String)
        /// The pose model returned a full skeleton, but one whose BODY SIZE is
        /// not a person's — see `MHRRetarget.plausibility`. Rejected before it
        /// could scale the musculoskeletal model, and reported with the measured
        /// number rather than dropped, because a frame that vanishes without a
        /// reason is indistinguishable from a crash.
        case implausibleBody(reason: String, hipWidthMeters: Double, statureMeters: Double)
        /// `NimbleEngine` never published a result within the wait budget. This
        /// covers two indistinguishable-from-the-outside cases: the solve is
        /// still slow/running, and the solve silently failed (`solveIK` returned
        /// nil, which never calls `publishResults` at all — see
        /// `OfflineSessionRunner`'s waiter doc comment). Either way, the frame
        /// has no usable result.
        case nimbleTimeout

        /// The sentence shown for `.implausibleBody`, nil for every other case.
        ///
        /// It lives on the model rather than inside `OfflinePlaybackView`'s
        /// private `statusText` so it can be tested: this string is the ONLY
        /// thing the user gets when a frame is rejected, and a frame that
        /// disappears without a number is indistinguishable from a crash.
        var implausibleBodyDescription: String? {
            guard case .implausibleBody(let reason, let hip, let stature) = self else { return nil }
            // BOTH measurements are shown whichever bound tripped, so the user
            // can see the whole prediction rather than the one number that
            // happened to fail first.
            return String(format: "Body size not measurable — %@. Measured: hips %.0f cm apart, height %.2f m.",
                          reason, hip * 100, stature)
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
        /// `outcome` is non-nil only where a contact was claimed and dynamics
        /// actually ran, and it carries the falsifier.
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
    }

    struct FrameResult: Identifiable {
        let id: Int  // frame index — stable, matches the scrubber position
        let sourceImage: UIImage
        let timestamp: TimeInterval
        let status: FrameStatus
        /// True if person detection found nobody and preprocessing fell back to
        /// the whole image. Informational, not a failure — pose estimation still
        /// ran; the result may just be lower quality.
        let usedFallbackBBox: Bool

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
        /// True iff this frame's ID and muscle results were solved as a STATIC
        /// EQUILIBRIUM problem (q̇ = q̈ = 0) on a detected hold — i.e. the
        /// muscle numbers are a posture estimate, not a measurement of dynamics.
        ///
        /// It used to mean "the end-of-clip padding replayed this pose", which
        /// was true of the last four frames of every clip regardless of whether
        /// the subject had moved, so it distinguished nothing.
        let isStaticHoldEstimate: Bool
        /// Why this frame does or does not carry muscle data.
        let motionState: MotionState

        var hasFullBiomechanics: Bool { muscleResult != nil }
        /// Pose was solved fine, but the detector could not certify a still
        /// instant, so no muscle magnitudes are claimed. Distinct from the
        /// warm-up case — and deliberately NOT named "because moving": one of
        /// the two reasons is that the pose estimate is too noisy to tell,
        /// which is the app's limitation and not the subject's.
        var isPoseOnlyBecauseNotStill: Bool {
            guard case .measured(let verdict, _, _, _) = motionState else { return false }
            return verdict != .hold && muscleResult == nil
        }

        /// True on a running clip's stance frames — the ones whose muscle
        /// numbers came from the gait cycle rather than from a static hold.
        var isGaitStance: Bool { motionState.verdict == .gaitStance }
    }

    @Published private(set) var frames: [FrameResult] = []
    @Published var selectedIndex: Int = 0
    /// What the gait pass concluded about the clip as a whole, or why it never
    /// ran. Nil until the batch finishes.
    @Published private(set) var gait: GaitOutcome?

    /// What `GaitAnalysis` concluded about this clip.
    enum GaitOutcome {
        /// Not a run, or not enough of one to try. The reason is the analysis's
        /// own error text.
        case notAttempted(reason: String)
        /// A run, but the clip's own model refused it. Every refusal carries the
        /// number that produced it.
        case refused(report: GaitReport)
        /// A usable run: dynamics were solved on its stance frames.
        case analysed(report: GaitReport,
                      plan: NimbleEngine.GaitPlan,
                      framesPerSecond: Double)

        var report: GaitReport? {
            switch self {
            case .notAttempted: return nil
            case .refused(let r): return r
            case .analysed(let r, _, _): return r
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
    }

    func setGait(_ outcome: GaitOutcome) { gait = outcome }

    func reset() {
        frames.removeAll()
        selectedIndex = 0
        gait = nil
    }

    /// Appends a new frame result and pins the scrubber to it — while a run is
    /// in progress this keeps playback following the newest processed frame;
    /// the user can still drag the scrubber back at any time.
    func append(_ result: FrameResult) {
        frames.append(result)
        selectedIndex = frames.count - 1
    }

    /// Rewrites frame `id`'s biomechanics fields in place.
    ///
    /// The Savitzky-Golay window is centred, so a solve never describes the
    /// frame that was just pushed — `OfflineSessionRunner` matches on the
    /// solve's own timestamp and calls this to file it against the frame it
    /// actually belongs to. That includes solves that produced NO muscle
    /// output because the subject was moving: `motionState` is the payload
    /// there, and passing `muscleResult: nil` must not erase the reason.
    func updateBiomechanics(forFrameID id: Int,
                             muscleResult: NimbleEngine.MuscleOutput?,
                             idResult: NimbleEngine.IDOutput?,
                             ikResult: NimbleEngine.IKOutput?,
                             isStaticHoldEstimate: Bool,
                             motionState: MotionState) {
        guard let index = frames.firstIndex(where: { $0.id == id }) else { return }
        let existing = frames[index]
        frames[index] = FrameResult(
            id: existing.id,
            sourceImage: existing.sourceImage,
            timestamp: existing.timestamp,
            status: muscleResult != nil ? .success : existing.status,
            usedFallbackBBox: existing.usedFallbackBBox,
            camT: existing.camT,
            modelChecksums: existing.modelChecksums,
            bodyFrame: existing.bodyFrame,
            ikResult: ikResult ?? existing.ikResult,
            idResult: idResult ?? existing.idResult,
            muscleResult: muscleResult ?? existing.muscleResult,
            isStaticHoldEstimate: isStaticHoldEstimate,
            motionState: motionState
        )
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

    /// Frames whose pose was solved but whose muscle numbers were withheld
    /// because the subject was moving. Surfaced so "few frames have muscle
    /// data" reads as a property of the clip rather than as a solver failure.
    var poseOnlyNotStillCount: Int { frames.filter(\.isPoseOnlyBecauseNotStill).count }
}
