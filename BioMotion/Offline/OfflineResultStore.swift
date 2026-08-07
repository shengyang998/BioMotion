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
        /// `NimbleEngine` never published a result within the wait budget. This
        /// covers two indistinguishable-from-the-outside cases: the solve is
        /// still slow/running, and the solve silently failed (`solveIK` returned
        /// nil, which never calls `publishResults` at all — see
        /// `OfflineSessionRunner`'s waiter doc comment). Either way, the frame
        /// has no usable result.
        case nimbleTimeout
    }

    /// What the static-hold detector concluded about this frame — the reason a
    /// frame does or does not carry muscle magnitudes.
    ///
    /// This exists because "no muscle data" had two very different causes that
    /// looked identical in the UI: the Savitzky-Golay window had not filled
    /// yet (a startup artifact, harmless), versus the subject was moving (a
    /// statement about what this input can support). See
    /// `NimbleEngine.staticHoldGating`.
    enum MotionState: Equatable {
        /// Marker motion around this frame stayed inside the static-hold
        /// bound, so ID and the muscle QP were solved with q̇ = q̈ = 0.
        case hold(peakSpeedMetersPerSecond: Double, windowSeconds: Double)
        /// The subject was measurably moving. Pose is still shown; muscle
        /// magnitudes are deliberately withheld, because the pose source pins
        /// the pelvis every frame and so cannot supply the accelerations those
        /// magnitudes would be derived from.
        case moving(peakSpeedMetersPerSecond: Double, windowSeconds: Double)
        /// No verdict reached this frame: the filter was still warming up, the
        /// solve failed, or nothing was ever routed to it.
        case undetermined

        var peakSpeedMetersPerSecond: Double? {
            switch self {
            case .hold(let v, _), .moving(let v, _): return v
            case .undetermined: return nil
            }
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
        /// Pose was solved fine, but the subject was moving, so no muscle
        /// magnitudes are claimed. Distinct from the warm-up case.
        var isPoseOnlyBecauseMoving: Bool {
            if case .moving = motionState { return muscleResult == nil }
            return false
        }
    }

    @Published private(set) var frames: [FrameResult] = []
    @Published var selectedIndex: Int = 0

    func reset() {
        frames.removeAll()
        selectedIndex = 0
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

    var biomechanicsCount: Int { frames.filter(\.hasFullBiomechanics).count }

    /// Frames whose pose was solved but whose muscle numbers were withheld
    /// because the subject was moving. Surfaced so "few frames have muscle
    /// data" reads as a property of the clip rather than as a solver failure.
    var poseOnlyMovingCount: Int { frames.filter(\.isPoseOnlyBecauseMoving).count }
}
