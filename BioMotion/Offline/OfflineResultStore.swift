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
        let bodyFrame: BodyFrame?
        let ikResult: NimbleEngine.IKOutput?
        let idResult: NimbleEngine.IDOutput?
        let muscleResult: NimbleEngine.MuscleOutput?
        /// True once `OfflineSessionRunner`'s end-of-clip padding has replayed
        /// this exact frame's pose to warm up the SG filter and produced a
        /// static-hold muscle estimate — lets the UI say "static estimate"
        /// instead of implying continuous dynamics were measured.
        let isStaticHoldEstimate: Bool

        var hasFullBiomechanics: Bool { muscleResult != nil }
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

    /// Rewrites frame `id`'s biomechanics fields in place. Used only by
    /// `OfflineSessionRunner`'s end-of-clip Savitzky-Golay warm-up padding, so
    /// the padding shows up as an update to the last real frame rather than as
    /// phantom extra rows in the scrubber.
    func updateBiomechanics(forFrameID id: Int,
                             muscleResult: NimbleEngine.MuscleOutput?,
                             idResult: NimbleEngine.IDOutput?,
                             ikResult: NimbleEngine.IKOutput?,
                             isStaticHoldEstimate: Bool) {
        guard let index = frames.firstIndex(where: { $0.id == id }) else { return }
        let existing = frames[index]
        frames[index] = FrameResult(
            id: existing.id,
            sourceImage: existing.sourceImage,
            timestamp: existing.timestamp,
            status: muscleResult != nil ? .success : existing.status,
            usedFallbackBBox: existing.usedFallbackBBox,
            bodyFrame: existing.bodyFrame,
            ikResult: ikResult ?? existing.ikResult,
            idResult: idResult ?? existing.idResult,
            muscleResult: muscleResult ?? existing.muscleResult,
            isStaticHoldEstimate: isStaticHoldEstimate
        )
    }

    var selectedFrame: FrameResult? {
        frames.indices.contains(selectedIndex) ? frames[selectedIndex] : nil
    }

    var successCount: Int {
        frames.filter { if case .success = $0.status { return true } else { return false } }.count
    }

    var biomechanicsCount: Int { frames.filter(\.hasFullBiomechanics).count }
}
