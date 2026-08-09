import Foundation

/// Why a run analysed fewer frames than the sampling mode asked for, and how
/// many it actually used.
///
/// # The banner this replaces was wrong in both halves
///
/// `FrameSource.sampleTimestamps` returns one boolean, `wasTruncated`, and its
/// own doc comment says the two causes "both matter to the user and both are
/// actionable". The UI collapsed them into a single sentence and named the wrong
/// one:
///
///     "The clip is longer than the analysis window, so only 120 frames from
///      the middle were used."
///
/// On a 2 s clip at 30 fps in `.nativeWindow(4.0)` the sampler wants 120 frames
/// and the clip has 60, so `wasTruncated` is true — and the sentence tells the
/// user their clip is too LONG and that 120 frames were used, when the clip is
/// too SHORT and 60 were used. The advice ("shorten it") is the opposite of the
/// action that would fix it. The same string also fires at 240 fps, where 601
/// frames were used and it still says 120, because `maxFramesPerRun` is not the
/// native-window mode's budget at all. At 240 fps the same sentence is false
/// even for a clip that is EXACTLY as long as the four-second window: the
/// window asks for 960 frames, the bounded run takes 601, and only the frame
/// budget — not the clip length — explains the shorter analysed span.
///
/// # How the cause is decided, without duplicating the sampler's arithmetic
///
/// Re-deriving "which budget bound this" would be a second copy of
/// `FrameSource`'s rules, free to drift from them. In native-window mode the
/// question is asked of the RESULT: if the run used every frame the clip has,
/// the clip was the limit; otherwise the native budget was. In sparse mode the
/// sampler only sets `wasTruncated` when another sample exists beyond its cap,
/// so a notice is necessarily the sparse budget case — there is no window.
///
/// # …and the same defect then survived on the other sampling mode
///
/// The budget sentence above is written for `.nativeWindow`, which CENTRES its
/// window. `.fps` has no analysis window at all — its cap is `maxFramesPerRun` —
/// and it starts at `t = 0` and steps forward, so its frames come from the
/// BEGINNING. A ten-minute clip sampled at 2 fps was told 120 frames "from the
/// middle" were used when they cover the first minute, so a user whose held pose
/// is at minute four trims the wrong end. It survived because
/// `OfflineDisclosureTests` asserted only `notice?.cause` for that mode while
/// both native-window tests asserted the message. Hence
/// `.budgetStoppedTheSparseScan`, and a test that reads the string.
struct FrameBudgetNotice: Equatable {

    enum Cause: Equatable {
        /// The clip is SHORTER than the analysis window: every frame it holds
        /// was used, and the lever is a longer clip.
        case clipShorterThanTheWindow
        /// The native-rate window asks for more frames than one bounded run can
        /// analyse at this rate, so its MIDDLE was analysed. The clip can be
        /// longer than, equal to, or shorter than the configured window; the
        /// frame budget is the cause. `.nativeWindow` only — that mode centres
        /// its samples (`FrameSource.sampleTimestamps`).
        case budgetCappedTheWindow
        /// **The sparse `.fps` scan ran out of frame budget.** A separate case
        /// because both halves of the sentence above are false for it: there is
        /// no analysis window in this mode (the cap is `maxFramesPerRun`), and
        /// the samples start at `t = 0` and step forward, so they come from the
        /// BEGINNING of the clip and not the middle.
        ///
        /// The one-sentence version of why this is its own case: a user whose
        /// held pose is at minute four of a ten-minute clip was told the middle
        /// had been analysed, and trimmed the wrong end.
        case budgetStoppedTheSparseScan
    }

    let cause: Cause
    /// How many frames were actually sampled. Always the real number.
    let framesUsed: Int
    /// The span those frames cover, seconds.
    let analysedSeconds: Double
    /// The whole clip's duration, seconds.
    let clipSeconds: Double

    /// The sentence the import screen shows. Names the real cause, the real
    /// count and the lever.
    var message: String {
        switch cause {
        case .clipShorterThanTheWindow:
            return String(format: "This clip is shorter than the %.0f s analysis window: all %d "
                          + "of its frames (%.1f s) were analysed. A left/right comparison needs "
                          + "at least %.1f s of running — film for longer.",
                          FrameSource.analysisWindowSeconds, framesUsed, analysedSeconds,
                          FrameSource.minimumAnalysisSeconds)
        case .budgetCappedTheWindow:
            return String(format: "At this video's frame rate, the %.0f s analysis window "
                          + "exceeds one run's %d-frame budget, so the middle %d frames "
                          + "(%.1f s) were analysed.",
                          FrameSource.analysisWindowSeconds,
                          FrameSource.maxNativeWindowFrames, framesUsed, analysedSeconds)
        case .budgetStoppedTheSparseScan:
            return String(format: "This clip is longer than one run's %d-frame budget at this "
                          + "sampling rate, so only the FIRST %d frames (%.1f s of %.1f s) were "
                          + "analysed. Trim the clip to the part you want measured.",
                          FrameSource.maxFramesPerRun, framesUsed, analysedSeconds, clipSeconds)
        }
    }

    /// - Parameters:
    ///   - timestamps: exactly what `FrameSource.sampleTimestamps` returned.
    ///   - wasTruncated: its second return value. `nil` comes back when it is
    ///     false — there is nothing to tell the user.
    /// - Returns: the notice, or nil when the run got everything it asked for.
    static func make(mode: FrameSource.SamplingMode,
                     duration: TimeInterval,
                     nominalFrameRate: Double,
                     timestamps: [TimeInterval],
                     wasTruncated: Bool) -> FrameBudgetNotice? {
        guard wasTruncated, !timestamps.isEmpty, duration > 0 else { return nil }
        let step: Double
        /// Which "a budget bound this" sentence applies. The two sampling modes
        /// take their frames from different parts of the clip, so one sentence
        /// cannot serve both.
        let budgetCause: Cause
        /// Only native-window truncation can mean either "short clip" or
        /// "budget". Sparse truncation already means its 120-frame cap fired.
        let clipCapacity: Int?
        switch mode {
        case .singleFrame:
            return nil
        case .fps(let fps):
            guard fps > 0 else { return nil }
            step = 1.0 / fps
            budgetCause = .budgetStoppedTheSparseScan
            clipCapacity = nil
        case .nativeWindow:
            step = 1.0 / FrameSource.sanitisedFrameRate(nominalFrameRate)
            budgetCause = .budgetCappedTheWindow
            clipCapacity = Swift.max(1, Int(duration / step))
        }
        let span = (timestamps.last ?? 0) - (timestamps.first ?? 0) + step
        let cause = clipCapacity.map {
            timestamps.count >= $0 ? Cause.clipShorterThanTheWindow : budgetCause
        } ?? budgetCause
        return FrameBudgetNotice(
            cause: cause,
            framesUsed: timestamps.count,
            analysedSeconds: span,
            clipSeconds: duration)
    }
}
