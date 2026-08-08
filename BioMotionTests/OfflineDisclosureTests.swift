import XCTest
import UIKit
@testable import BioMotion

/// **What the screen claims, per frame and per clip.**
///
/// Two defects live here, and they are the same defect twice: a number the
/// pipeline had already refused reaching the user under a caption that says it
/// was measured.
///
/// 1. The 3-D muscle overlay and the frame caption were gated CLIP-wide only, so
///    a clip that passed every gate still drew coloured muscle capsules on the
///    individual frames `GaitLoadSummary.make` had discarded.
/// 2. The truncation banner named the wrong cause and the wrong frame count for
///    the case it most often fires on.
final class OfflineDisclosureTests: XCTestCase {

    // MARK: - The per-frame gate on the overlay and the caption

    /// A frame whose derivative window crossed a contact edge does not enter the
    /// ranked list, and now does not draw either. MEASURED share of stance
    /// frames this affects on the pinned clips: 52 of 64 (`video_012`) and 44 of
    /// 68 (`video_015`).
    func testAFrameOutsideTheLoadComparisonDrawsNoMuscleOverlay() {
        let clean = Self.stanceFrame(side: -1, cleanWindow: true)
        XCTAssertTrue(clean.gaitLoadsAreComparable)
        XCTAssertNil(clean.gaitExclusionReason)

        let dirty = Self.stanceFrame(side: -1, cleanWindow: false)
        XCTAssertFalse(dirty.gaitLoadsAreComparable,
                       "the ranked list drops this frame; so must the overlay")
        XCTAssertEqual(dirty.gaitExclusionReason, "too close to a touchdown or toe-off "
                       + "to differentiate")
        // It still carries muscle numbers — the ONLY thing between them and the
        // screen is this flag, which is exactly why it has to be asked.
        XCTAssertNotNil(dirty.muscleResult)
    }

    /// Each exclusion names its own lever. Collapsing them would tell a user
    /// with a ground-height problem to film at a higher frame rate.
    func testEachExclusionNamesItsOwnCause() {
        let noFootDown = Self.stanceFrame(side: -1, solverLeft: false, solverRight: false)
        XCTAssertFalse(noFootDown.gaitLoadsAreComparable)
        XCTAssertEqual(noFootDown.gaitExclusionReason,
                       "the foot's height above the ground disagrees that it was planted — "
                       + "solved with no ground force")

        let doubleContact = Self.stanceFrame(side: -1, solverLeft: true, solverRight: true)
        XCTAssertFalse(doubleContact.gaitLoadsAreComparable)
        XCTAssertEqual(doubleContact.gaitExclusionReason,
                       "the solver put ground force under BOTH feet, so this frame's load is "
                       + "split between them")

        // A flight frame carrying an outcome does not occur today —
        // `NimbleEngine` routes flight through `publishPoseOnly`, so it has no
        // outcome and no muscle result. This pins the defensive branch so that
        // if that ever changes the sentence is the right one rather than
        // "too close to a touchdown".
        let flight = Self.gaitFrame(contactSide: 0, solverLeft: false, solverRight: false,
                                    cleanWindow: false)
        XCTAssertFalse(flight.gaitLoadsAreComparable)
        XCTAssertEqual(flight.gaitExclusionReason, "both feet off the ground — no contact load here")
    }

    /// **The non-regression that licenses the change.** A still-pose clip has no
    /// gait outcome at all, so nothing about the static-hold path moves.
    func testTheStillPosePathIsUnchanged() {
        let hold = OfflineResultStore.FrameResult(
            id: 0, sourceImage: UIImage(), timestamp: 0, status: .success,
            usedFallbackBBox: false, camT: nil, modelChecksums: nil, bodyFrame: nil,
            ikResult: nil, idResult: nil,
            muscleResult: NimbleEngine.MuscleOutput(activations: ["soleus_l": 0.4],
                                                    forces: ["soleus_l": 400],
                                                    converged: true, timestamp: 0),
            isStaticHoldEstimate: true,
            motionState: .measured(verdict: .hold, peakSpeedMetersPerSecond: 0.001,
                                   windowSeconds: 1.0, noiseFloorMetersPerSecond: 0.004))
        XCTAssertTrue(hold.gaitLoadsAreComparable)
        XCTAssertNil(hold.gaitExclusionReason)

        let undetermined = OfflineResultStore.FrameResult(
            id: 1, sourceImage: UIImage(), timestamp: 0, status: .success,
            usedFallbackBBox: false, camT: nil, modelChecksums: nil, bodyFrame: nil,
            ikResult: nil, idResult: nil, muscleResult: nil, isStaticHoldEstimate: false,
            motionState: .undetermined)
        XCTAssertTrue(undetermined.gaitLoadsAreComparable)
    }

    /// The per-frame flag and the clip-level statistic have to agree about which
    /// frames they are talking about, or the panel's count and the overlay's
    /// behaviour drift apart.
    func testThePerFrameFlagAgreesWithWhatTheSummaryCounted() throws {
        let bundle = Bundle(for: type(of: self))
        let report = try GaitAnalysis.analyse(
            frames: try GaitClipFixture.load("video_012", bundle: bundle).frames)
        var frames: [OfflineResultStore.FrameResult] = []
        for i in 0..<8 {
            frames.append(Self.stanceFrame(id: i, side: i < 4 ? -1 : 1,
                                           cleanWindow: i % 2 == 0))
        }
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   framesPerSecond: 30, filterTaps: 5))
        XCTAssertEqual(s.stanceFrameCount, frames.filter(\.gaitLoadsAreComparable).count)
        XCTAssertEqual(s.claimedStanceFrameCount - s.stanceFrameCount,
                       frames.filter { !$0.gaitLoadsAreComparable }.count)
    }

    // MARK: - The truncation banner

    /// **The case the banner most often fires on, and used to describe
    /// backwards.** A 2 s clip at 30 fps in the 4 s native window: the sampler
    /// wants 120 frames, the clip has 60, so `wasTruncated` is true — and the
    /// old sentence said the clip was too LONG and that 120 frames were used.
    func testAClipShorterThanTheWindowSaysSoAndCountsItsOwnFrames() {
        let duration = 2.0
        let (timestamps, truncated) = FrameSource.sampleTimestamps(
            duration: duration, mode: .nativeWindow(seconds: FrameSource.analysisWindowSeconds),
            nominalFrameRate: 30)
        XCTAssertTrue(truncated, "the sampler still reports it, which is correct")
        XCTAssertEqual(timestamps.count, 60)

        let notice = try? XCTUnwrap(FrameBudgetNotice.make(
            mode: .nativeWindow(seconds: FrameSource.analysisWindowSeconds),
            duration: duration, nominalFrameRate: 30,
            timestamps: timestamps, wasTruncated: truncated))
        XCTAssertEqual(notice?.cause, .clipShorterThanTheWindow)
        XCTAssertEqual(notice?.framesUsed, 60)
        XCTAssertEqual(notice?.analysedSeconds ?? 0, 2.0, accuracy: 0.05)
        let message = notice?.message ?? ""
        print("UI-METRIC short_clip_banner=\(message)")
        XCTAssertTrue(message.contains("shorter"), message)
        XCTAssertFalse(message.contains("longer than the analysis window"), message)
        XCTAssertTrue(message.contains("60"), "the real count, not the budget: \(message)")
        XCTAssertFalse(message.contains("120"), message)
        XCTAssertTrue(message.contains("film for longer") || message.contains("film for longer."),
                      "the lever must be the one that works: \(message)")
    }

    /// And the case the old sentence was written for still reads correctly — at
    /// a rate where the budget is 601 frames and not the 120 it used to quote.
    func testAClipLongerThanTheWindowNamesTheFramesItActuallyUsed() {
        let mode = FrameSource.SamplingMode.nativeWindow(seconds: FrameSource.analysisWindowSeconds)
        let (timestamps, truncated) = FrameSource.sampleTimestamps(
            duration: 30.0, mode: mode, nominalFrameRate: 240)
        XCTAssertTrue(truncated)
        XCTAssertEqual(timestamps.count, FrameSource.maxNativeWindowFrames)
        let notice = try? XCTUnwrap(FrameBudgetNotice.make(
            mode: mode, duration: 30.0, nominalFrameRate: 240,
            timestamps: timestamps, wasTruncated: truncated))
        XCTAssertEqual(notice?.cause, .budgetCappedTheWindow)
        XCTAssertEqual(notice?.framesUsed, FrameSource.maxNativeWindowFrames)
        let message = notice?.message ?? ""
        print("UI-METRIC long_clip_banner=\(message)")
        XCTAssertTrue(message.contains("longer"), message)
        XCTAssertTrue(message.contains("\(FrameSource.maxNativeWindowFrames)"),
                      "601 frames were used, and 120 is not this mode's budget: \(message)")
        XCTAssertFalse(message.contains("\(FrameSource.maxFramesPerRun) frames"), message)
    }

    /// A clip that got everything it asked for says nothing at all.
    func testAnUntruncatedRunShowsNoBanner() {
        let mode = FrameSource.SamplingMode.nativeWindow(seconds: FrameSource.analysisWindowSeconds)
        let (timestamps, truncated) = FrameSource.sampleTimestamps(
            duration: 30.0, mode: mode, nominalFrameRate: 30)
        XCTAssertFalse(truncated)
        XCTAssertEqual(timestamps.count, 120)
        XCTAssertNil(FrameBudgetNotice.make(mode: mode, duration: 30.0, nominalFrameRate: 30,
                                            timestamps: timestamps, wasTruncated: truncated))
        // And a single frame is never a budget story.
        XCTAssertNil(FrameBudgetNotice.make(mode: .singleFrame, duration: 30.0,
                                            nominalFrameRate: 30, timestamps: [15.0],
                                            wasTruncated: true))
    }

    /// The sparse-sampling mode has its own budget and its own arithmetic, and
    /// the same rule decides the cause there.
    /// **The sparse mode's banner, checked on the STRING and not only on the
    /// cause.** This test asserted `notice?.cause` alone while both
    /// native-window tests asserted their message, and the message it was not
    /// asserting was false in both halves: `.fps` has no analysis window (the
    /// cap is `maxFramesPerRun`) and its samples start at `t = 0` and step
    /// forward, so they come from the BEGINNING and not the middle. A user with
    /// a ten-minute clip and a held pose at minute four was told the middle had
    /// been analysed, and trimmed the wrong end.
    func testTheSparseModeIsCappedByItsOwnBudget() throws {
        let mode = FrameSource.SamplingMode.fps(10)
        let (timestamps, truncated) = FrameSource.sampleTimestamps(duration: 600, mode: mode)
        XCTAssertTrue(truncated)
        XCTAssertEqual(timestamps.count, FrameSource.maxFramesPerRun)
        XCTAssertEqual(timestamps.first ?? -1, 0.0, accuracy: 1e-12,
                       "the sparse scan starts at the beginning of the clip")
        let notice = try XCTUnwrap(FrameBudgetNotice.make(
            mode: mode, duration: 600, nominalFrameRate: 30,
            timestamps: timestamps, wasTruncated: truncated))
        XCTAssertEqual(notice.cause, .budgetStoppedTheSparseScan)
        XCTAssertEqual(notice.framesUsed, FrameSource.maxFramesPerRun)
        XCTAssertEqual(notice.analysedSeconds, 12.0, accuracy: 1e-9)

        let message = notice.message
        print("UI-METRIC sparse_budget_notice=\(message)")
        XCTAssertTrue(message.contains("FIRST"), "it names WHERE the frames came from: \(message)")
        XCTAssertFalse(message.lowercased().contains("middle"),
                       "and they did not come from the middle: \(message)")
        XCTAssertFalse(message.lowercased().contains("analysis window"),
                       "there is no analysis window in this mode: \(message)")
        XCTAssertTrue(message.contains("12.0 s"), message)
        XCTAssertTrue(message.contains("600.0 s"), "and the clip's real length: \(message)")
    }

    // MARK: - Fixtures

    static func stanceFrame(id: Int = 0, side: Int,
                            solverLeft: Bool? = nil, solverRight: Bool? = nil,
                            cleanWindow: Bool = true) -> OfflineResultStore.FrameResult {
        gaitFrame(id: id, contactSide: side,
                  solverLeft: solverLeft ?? (side < 0),
                  solverRight: solverRight ?? (side > 0),
                  cleanWindow: cleanWindow)
    }

    static func gaitFrame(id: Int = 0, contactSide: Int,
                          solverLeft: Bool, solverRight: Bool,
                          cleanWindow: Bool) -> OfflineResultStore.FrameResult {
        let outcome = NimbleEngine.GaitFrameOutcome(
            modelledVerticalForceInBodyWeights: 2.0,
            solvedVerticalForceInBodyWeights: 2.1,
            residualInBodyWeights: 0.1,
            contactSide: contactSide,
            contactIndex: contactSide < 0 ? 0 : 1,
            solverSawLeftContact: solverLeft,
            solverSawRightContact: solverRight,
            rootVerticalAccelerationMetersPerSecondSquared: 9.81,
            horizontalRootAccelerationModelled: false,
            derivativeWindowInsideContact: cleanWindow)
        let muscle = NimbleEngine.MuscleOutput(
            activations: ["soleus_l": 0.5, "soleus_r": 0.5],
            forces: ["soleus_l": 500, "soleus_r": 500],
            converged: true, timestamp: Double(id) / 30.0)
        return OfflineResultStore.FrameResult(
            id: id, sourceImage: UIImage(), timestamp: Double(id) / 30.0,
            status: .success, usedFallbackBBox: false, camT: nil, modelChecksums: nil,
            bodyFrame: nil, ikResult: nil, idResult: nil, muscleResult: muscle,
            isStaticHoldEstimate: false,
            motionState: .gait(verdict: contactSide == 0 ? .gaitFlight : .gaitStance,
                               outcome: outcome))
    }
}
