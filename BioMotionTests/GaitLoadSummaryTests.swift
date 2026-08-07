import XCTest
import UIKit
@testable import BioMotion

/// What the user is shown for a running clip: RELATIVE loads, a left/right
/// comparison the clip is actually allowed to make, and a refusal that always
/// names the lever.
final class GaitLoadSummaryTests: XCTestCase {

    // MARK: - No headline newton figure

    /// The product rule, enforced on the strings themselves: nothing the UI
    /// renders may quote a force in newtons. Body weights appear only in the
    /// "what this does not measure" block, as the residual.
    func testNothingUserFacingQuotesNewtons() {
        let s = Self.summary(resolvable: 10)
        let strings = [s.resolutionSentence,
                       s.resolutionBreakdownSentence,
                       s.unmodelledTermSentence,
                       s.claim(for: s.ranked[0]),
                       s.claim(for: s.ranked[1])]
        for text in strings {
            XCTAssertFalse(text.contains(" N "), "\(text)")
            XCTAssertFalse(text.lowercased().contains("newton"), "\(text)")
        }
        // And every verdict's advice is a sentence a person can act on.
        for verdict in Self.allVerdicts where verdict != .hold && verdict != .gaitStance {
            XCTAssertFalse(verdict.advice.isEmpty, "\(verdict) has no advice")
            XCTAssertFalse(verdict.advice.lowercased().contains("newton"))
        }
    }

    /// The vocabulary is ONE enum, extended — not a second taxonomy beside the
    /// first. A previous round shipped exactly such a duplication and the UI
    /// could then only ever say "moving".
    func testTheVerdictVocabularyIsExtendedNotDuplicated() {
        // Every verdict the store can carry is a `NimbleEngine.MotionVerdict`.
        for verdict in Self.allVerdicts {
            let measured = OfflineResultStore.MotionState.measured(
                verdict: verdict, peakSpeedMetersPerSecond: 0,
                windowSeconds: 1, noiseFloorMetersPerSecond: 0)
            XCTAssertEqual(measured.verdict, verdict)
            let gait = OfflineResultStore.MotionState.gait(verdict: verdict, outcome: nil)
            XCTAssertEqual(gait.verdict, verdict)
            XCTAssertEqual(gait.isHold, false, "the gait case never claims a hold")
        }
        XCTAssertNil(OfflineResultStore.MotionState.undetermined.verdict)

        // Only the four gait cases answer to `isGait`, so a static-path
        // consumer cannot be handed one by accident.
        let gaitCases: Set<String> = ["gaitStance", "gaitFlight", "gaitOutsideAnalysis", "gaitRefused"]
        for verdict in Self.allVerdicts {
            XCTAssertEqual(verdict.isGait, gaitCases.contains("\(verdict)"), "\(verdict)")
        }
    }

    /// The gait pass runs on every clip and declines most of them. A photo must
    /// keep its posture findings, not be handed a screen about strides.
    func testDecliningToAnalyseAClipDoesNotTurnItIntoAGaitScreen() throws {
        XCTAssertFalse(OfflineResultStore.GaitOutcome
            .notAttempted(reason: "1 usable frame").isAboutRunning)
        let report = try Self.usableReport()
        XCTAssertTrue(OfflineResultStore.GaitOutcome.refused(report: report).isAboutRunning)
        let plan = try XCTUnwrap(OfflineSessionRunner.makePlan(from: report))
        XCTAssertTrue(OfflineResultStore.GaitOutcome
            .analysed(report: report, plan: plan, framesPerSecond: 30).isAboutRunning)
    }

    /// A solve that stopped on its iteration cap is now reported as itself. It
    /// used to inherit the stillness verdict, so a solver failure was shown to
    /// the user as "the subject moved" — advice they cannot act on about a
    /// thing that did not happen.
    func testANonConvergedSolveIsNotReportedAsSubjectMotion() {
        let verdict = NimbleEngine.MotionVerdict.poseDidNotConverge
        XCTAssertFalse(verdict.isGait)
        XCTAssertNotEqual(verdict, .movingBeyondStaticBudget)
        XCTAssertFalse(verdict.advice.lowercased().contains("hold the position"))
        XCTAssertTrue(verdict.advice.lowercased().contains("frame"))
    }

    /// "Film at a higher frame rate" has to be a real, arithmetically correct
    /// thing to say — resolution is `0.5/N`, so halving it needs twice the
    /// frames per contact and therefore twice the frame rate.
    func testTheHigherFrameRateAdviceIsArithmeticallyRight() {
        // Repeatability well under 5 %, so the ±5 % target is reachable and the
        // sentence is pure arithmetic. The guard against an unreachable target
        // is tested separately.
        let s = Self.summary(resolvable: 10, repeatability: 1.0, framesPerContact: 5, fps: 30)
        XCTAssertEqual(s.frameRateNeeded(forPercent: 10), 30, accuracy: 1e-9)
        XCTAssertEqual(s.frameRateNeeded(forPercent: 5), 60, accuracy: 1e-9)
        XCTAssertEqual(s.frameRateNeeded(forPercent: 2.5), 120, accuracy: 1e-9)
        XCTAssertEqual(s.frameRateNeeded(forPercent: 1.25), 240, accuracy: 1e-9)
        XCTAssertTrue(s.resolutionSentence.contains("60 fps"),
                      "the sentence must name the rate: \(s.resolutionSentence)")
        print("UI-METRIC resolution_sentence=\(s.resolutionSentence)")
    }

    /// When the runner's own stride scatter is the binding limit, a faster
    /// camera does NOT help, so the sentence must not promise it would.
    func testWhenTheRunnerIsTheLimitTheCameraAdviceIsWithheld() {
        let s = Self.summary(resolvable: 20, floor: 8, repeatability: 20,
                             framesPerContact: 6.25, fps: 30)
        XCTAssertFalse(s.resolutionSentence.contains("fps would resolve"),
                       "a faster camera cannot fix stride-to-stride variation: \(s.resolutionSentence)")
        XCTAssertTrue(s.resolutionBreakdownSentence.contains("8%"))
        XCTAssertTrue(s.resolutionBreakdownSentence.contains("20%"))
        print("UI-METRIC runner_limited=\(s.resolutionSentence) | \(s.resolutionBreakdownSentence)")
    }

    // MARK: - The refusal

    /// A difference finer than the clip's resolution is refused, not shown with
    /// a caveat, and the refusal says so in the user's terms.
    func testAClaimFinerThanTheResolutionIsRefused() {
        let s = Self.summary(resolvable: 10)
        let even = GaitLoadSummary.MuscleLoad(id: "x", displayName: "X",
                                              leftPeak: 0.51, rightPeak: 0.49,
                                              leftFrames: 5, rightFrames: 5,
                                              isSaturated: false)
        XCTAssertEqual(even.differencePercent, 4.0, accuracy: 1e-9)
        XCTAssertFalse(s.permits(differencePercent: even.differencePercent))
        XCTAssertTrue(s.claim(for: even).contains("Even to within"))
        XCTAssertTrue(s.claim(for: even).contains("10%"))

        let real = GaitLoadSummary.MuscleLoad(id: "y", displayName: "Y",
                                              leftPeak: 0.70, rightPeak: 0.50,
                                              leftFrames: 5, rightFrames: 5,
                                              isSaturated: false)
        XCTAssertEqual(real.differencePercent, 100.0 * 0.2 / 0.6, accuracy: 1e-9)
        XCTAssertTrue(s.permits(differencePercent: real.differencePercent))
        XCTAssertTrue(s.claim(for: real).contains("left"))
    }

    /// Exactly at the boundary, and either side of it.
    func testTheRefusalBoundaryIsInclusive() {
        let s = Self.summary(resolvable: 10)
        XCTAssertTrue(s.permits(differencePercent: 10.0))
        XCTAssertTrue(s.permits(differencePercent: -10.0))
        XCTAssertTrue(s.permits(differencePercent: 10.001))
        XCTAssertFalse(s.permits(differencePercent: 9.999))
        XCTAssertFalse(s.permits(differencePercent: .nan))
        XCTAssertFalse(s.permits(differencePercent: 0))
    }

    /// A failed falsifier withholds everything, however large the difference.
    func testAFailedResidualGateWithholdsEveryClaim() {
        let s = Self.summary(resolvable: 5, residual: 3.0)
        XCTAssertFalse(s.residualGatePassed)
        XCTAssertFalse(s.permits(differencePercent: 200))
        XCTAssertTrue(s.claim(for: s.ranked[0]).contains("Withheld"))
    }

    // MARK: - Building from frames

    /// Each leg is credited only during ITS OWN stance, so the comparison is of
    /// sides and not of gait phases.
    func testEachLegIsCreditedOnlyDuringItsOwnContact() throws {
        let report = try Self.usableReport()
        let frames = [
            Self.gaitFrame(id: 0, side: -1, activations: ["glmax1_l": 0.80, "glmax1_r": 0.99]),
            Self.gaitFrame(id: 1, side: 1, activations: ["glmax1_l": 0.99, "glmax1_r": 0.40]),
        ]
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   framesPerSecond: 30, filterTaps: 5))
        let load = try XCTUnwrap(s.ranked.first { $0.id == "glmax1" })
        XCTAssertEqual(load.leftPeak, 0.80, accuracy: 1e-9,
                       "the left value from the RIGHT contact must be ignored")
        XCTAssertEqual(load.rightPeak, 0.40, accuracy: 1e-9)
        XCTAssertEqual(load.displayName, "Glute max (upper)")
        XCTAssertEqual(s.stanceFrameCount, 2)
    }

    /// Trunk muscles have no side and cannot enter a left/right table.
    func testUnsidedMusclesAreExcludedFromTheComparison() throws {
        let report = try Self.usableReport()
        let frames = [
            Self.gaitFrame(id: 0, side: -1, activations: ["soleus_l": 0.6, "multifidus_T9_T7": 0.9]),
            Self.gaitFrame(id: 1, side: 1, activations: ["soleus_r": 0.5, "multifidus_T9_T7": 0.9]),
        ]
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   framesPerSecond: 30, filterTaps: 5))
        XCTAssertEqual(s.ranked.map(\.id), ["soleus"])
        XCTAssertNil(GaitLoadSummary.split("multifidus_T9_T7"))
        XCTAssertEqual(GaitLoadSummary.split("bflh140_r")?.base, "bflh140")
        XCTAssertEqual(GaitLoadSummary.split("bflh140_r")?.side, "r")
    }

    /// The ranking IS the muscle-to-muscle comparison, so it has to be by load.
    func testRankingIsByPeakLoadDescending() throws {
        let report = try Self.usableReport()
        let frames = [
            Self.gaitFrame(id: 0, side: -1,
                           activations: ["soleus_l": 0.30, "glmax1_l": 0.70, "recfem_l": 0.50]),
            Self.gaitFrame(id: 1, side: 1,
                           activations: ["soleus_r": 0.35, "glmax1_r": 0.60, "recfem_r": 0.45]),
        ]
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   framesPerSecond: 30, filterTaps: 5))
        XCTAssertEqual(s.ranked.map(\.id), ["glmax1", "recfem", "soleus"])
        for i in 1..<s.ranked.count {
            XCTAssertGreaterThanOrEqual(s.ranked[i - 1].peak, s.ranked[i].peak)
        }
    }

    /// Saturation is where "a force error is a common scale" stops holding, so
    /// it is counted and shown rather than quietly absorbed.
    func testSaturatedMusclesAreCounted() throws {
        let report = try Self.usableReport()
        let frames = [
            Self.gaitFrame(id: 0, side: -1, activations: ["soleus_l": 1.0, "glmax1_l": 0.5]),
            Self.gaitFrame(id: 1, side: 1, activations: ["soleus_r": 0.999, "glmax1_r": 0.5]),
        ]
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   framesPerSecond: 30, filterTaps: 5))
        XCTAssertEqual(s.saturatedMuscleCount, 2)
    }

    /// A clip with no stance frame returns nil rather than an empty screen that
    /// looks like a measurement of zero.
    func testNoStanceFramesYieldsNoSummary() throws {
        let report = try Self.usableReport()
        XCTAssertNil(GaitLoadSummary.make(frames: [], report: report,
                                          framesPerSecond: 30, filterTaps: 5))
    }

    /// The residual and the contact-detector disagreement are aggregated from
    /// the frames, so they cannot be quietly dropped on the way to the screen —
    /// and they are aggregated SEPARATELY, because they are different failures.
    ///
    /// A frame where the ID solver's geometric detector saw a different foot
    /// down was solved with NO ground force at all (`solveIDGRF` returns zero),
    /// so its "residual" is the entire modelled force and says nothing about
    /// limb inertia. Feeding it into the inertia statistic made one such frame
    /// withhold a whole clip under a sentence about the body's own inertia that
    /// was not what happened.
    func testTheTwoFailureModesAreAggregatedSeparately() throws {
        let report = try Self.usableReport()
        let frames = [
            Self.gaitFrame(id: 0, side: -1, activations: ["soleus_l": 0.5], residual: 0.10),
            Self.gaitFrame(id: 1, side: 1, activations: ["soleus_r": 0.9], residual: 2.90,
                           solverLeft: true, solverRight: false),
            Self.gaitFrame(id: 2, side: -1, activations: ["soleus_l": 0.4], residual: 0.20),
            Self.gaitFrame(id: 3, side: 1, activations: ["soleus_r": 0.5], residual: 0.20),
        ]
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   framesPerSecond: 30, filterTaps: 5))
        XCTAssertEqual(s.maxForceResidualInBodyWeights, 0.20, accuracy: 1e-9,
                       "the 2.90 BW frame is a detector disagreement, not limb inertia")
        XCTAssertTrue(s.residualGatePassed)
        XCTAssertEqual(s.contactDetectorDisagreements, 1,
                       "the right-foot frame where the solver saw the LEFT foot down")
        XCTAssertEqual(s.claimedStanceFrameCount, 4)
        XCTAssertEqual(s.stanceFrameCount, 3, "and its activations never entered a peak")
        let load = try XCTUnwrap(s.ranked.first { $0.id == "soleus" })
        XCTAssertEqual(load.rightPeak, 0.5, accuracy: 1e-9,
                       "0.9 came from a frame solved without a ground reaction")
        XCTAssertFalse(s.horizontalRootAccelerationModelled)
        XCTAssertTrue(s.unmodelledTermSentence.lowercased().contains("braking"))
    }

    /// A frame whose derivative window crossed a contact edge carries an
    /// acceleration fitted across a discontinuity, so it does not enter a peak
    /// either — and it is counted, not silently dropped.
    func testFramesWithoutACleanDerivativeWindowAreExcludedAndCounted() throws {
        let report = try Self.usableReport()
        let frames = [
            Self.gaitFrame(id: 0, side: -1, activations: ["soleus_l": 0.95], cleanWindow: false),
            Self.gaitFrame(id: 1, side: -1, activations: ["soleus_l": 0.40]),
            Self.gaitFrame(id: 2, side: 1, activations: ["soleus_r": 0.30]),
            Self.gaitFrame(id: 3, side: 1, activations: ["soleus_r": 0.99], cleanWindow: false),
        ]
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   framesPerSecond: 30, filterTaps: 5))
        XCTAssertEqual(s.framesWithoutACleanDerivativeWindow, 2)
        XCTAssertEqual(s.stanceFrameCount, 2)
        XCTAssertEqual(s.claimedStanceFrameCount, 4)
        let load = try XCTUnwrap(s.ranked.first { $0.id == "soleus" })
        XCTAssertEqual(load.leftPeak, 0.40, accuracy: 1e-9)
        XCTAssertEqual(load.rightPeak, 0.30, accuracy: 1e-9)
        XCTAssertEqual(s.usableStanceFraction, 0.5, accuracy: 1e-9)
    }

    // MARK: - The gates that stand between the numbers and the screen

    /// **`arePublishable` is the one question, and everything answers to it.**
    ///
    /// The bars, the numbers, the ranking and the 3-D overlay all rest on the
    /// same assumptions, so they are withheld together. Before this, only the
    /// per-muscle sentence went through a gate: the screen showed a full
    /// left/right comparison under a caption reading "loads withheld".
    func testAFailedGateWithholdsTheWholeLoadBlockAndSaysWhy() {
        let failed = Self.summary(resolvable: 5, residual: 3.0)
        XCTAssertFalse(failed.residualGatePassed)
        XCTAssertFalse(failed.arePublishable)
        XCTAssertFalse(failed.permits(differencePercent: 200),
                       "a failed gate withholds even a huge left/right difference")
        // The data is still there — the ONLY thing between it and the screen is
        // the flag, which is exactly why the flag has to be asked.
        XCTAssertFalse(failed.ranked.isEmpty)
        let reason = try? XCTUnwrap(failed.withheldReason)
        XCTAssertTrue(reason!.contains("3.00"), "the refusal names the measurement: \(reason!)")
        XCTAssertTrue(reason!.lowercased().contains("film"), "and a lever: \(reason!)")
        XCTAssertTrue(failed.claim(for: failed.ranked[0]).contains("Withheld"))

        let passing = Self.summary(resolvable: 5, residual: 0.1)
        XCTAssertTrue(passing.arePublishable)
        XCTAssertNil(passing.withheldReason)
    }

    /// The invariant the panel relies on: withheld iff there is a reason.
    func testWithheldReasonExistsExactlyWhenTheLoadsAreNotPublishable() {
        for residual in [0.0, 0.1, 0.49, 0.5, 0.51, 3.0] {
            let s = Self.summary(resolvable: 5, residual: residual)
            XCTAssertEqual(s.withheldReason == nil, s.arePublishable, "residual \(residual)")
        }
    }

    /// A clip where the two contact detectors mostly disagree cannot say which
    /// foot carried the load, so it publishes nothing — and the refusal names
    /// the geometry rather than the inertia.
    func testTheContactGateWithholdsWhenTheDetectorsMostlyDisagree() throws {
        let report = try Self.usableReport()
        var frames: [OfflineResultStore.FrameResult] = []
        for i in 0..<10 {
            let side = i % 2 == 0 ? -1 : 1
            // 7 of 10 frames: the solver saw neither foot down.
            let disagree = i >= 3
            frames.append(Self.gaitFrame(id: i, side: side,
                                         activations: ["soleus_l": 0.5, "soleus_r": 0.5],
                                         solverLeft: disagree ? false : side < 0,
                                         solverRight: disagree ? false : side > 0))
        }
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   framesPerSecond: 30, filterTaps: 5))
        XCTAssertEqual(s.contactDetectorDisagreements, 7)
        XCTAssertEqual(s.agreementFraction, 0.3, accuracy: 1e-9)
        XCTAssertFalse(s.contactGatePassed)
        XCTAssertFalse(s.arePublishable)
        let reason = try XCTUnwrap(s.withheldReason)
        XCTAssertTrue(reason.lowercased().contains("height"), reason)
        XCTAssertTrue(reason.lowercased().contains("film"), reason)
    }

    /// Two usable frames per side, minimum, or there is nothing to compare.
    func testTooFewUsableFramesOnOneSideWithholds() throws {
        let report = try Self.usableReport()
        let frames = [
            Self.gaitFrame(id: 0, side: -1, activations: ["soleus_l": 0.5]),
            Self.gaitFrame(id: 1, side: -1, activations: ["soleus_l": 0.6]),
            Self.gaitFrame(id: 2, side: 1, activations: ["soleus_r": 0.5]),
        ]
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   framesPerSecond: 30, filterTaps: 5))
        XCTAssertEqual(s.leftStanceFrameCount, 2)
        XCTAssertEqual(s.rightStanceFrameCount, 1)
        XCTAssertFalse(s.contactGatePassed)
        XCTAssertNotNil(s.withheldReason)
    }

    /// **The only gate on the untested quantity.** Nothing in this pipeline
    /// tests the peak ground force; the product survives that because a peak
    /// error is a common scale that cancels out of a ratio — and it stops
    /// cancelling exactly where a muscle saturates. So a saturated muscle's
    /// left/right claim is withheld while its neighbours' are not.
    func testASaturatedMuscleGetsNoLeftRightClaimButItsNeighboursStillDo() throws {
        let report = try Self.usableReport()
        let frames = [
            Self.gaitFrame(id: 0, side: -1, activations: ["soleus_l": 1.0, "glmax1_l": 0.90]),
            Self.gaitFrame(id: 1, side: -1, activations: ["soleus_l": 1.0, "glmax1_l": 0.90]),
            Self.gaitFrame(id: 2, side: 1, activations: ["soleus_r": 0.40, "glmax1_r": 0.30]),
            Self.gaitFrame(id: 3, side: 1, activations: ["soleus_r": 0.40, "glmax1_r": 0.30]),
        ]
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   framesPerSecond: 30, filterTaps: 5))
        XCTAssertTrue(s.arePublishable, "the clip as a whole is fine")
        let soleus = try XCTUnwrap(s.ranked.first { $0.id == "soleus" })
        let glmax = try XCTUnwrap(s.ranked.first { $0.id == "glmax1" })
        XCTAssertTrue(soleus.isSaturated)
        XCTAssertFalse(glmax.isSaturated)
        // Both differences are far above the resolution, so only saturation
        // separates them.
        XCTAssertGreaterThan(abs(soleus.differencePercent), s.resolvableAsymmetryPercent)
        XCTAssertGreaterThan(abs(glmax.differencePercent), s.resolvableAsymmetryPercent)
        XCTAssertTrue(s.claim(for: soleus).contains("Withheld"), s.claim(for: soleus))
        XCTAssertTrue(s.claim(for: soleus).contains("full effort"))
        XCTAssertTrue(s.claim(for: glmax).contains("harder on the left"), s.claim(for: glmax))
    }

    // MARK: - The advice has to be deliverable

    /// The promise must not be finer than any camera could buy on this runner.
    func testTheAdviceNeverPromisesFinerThanTheRunnerAllows() {
        // Repeatability 7.2 % with a 10.1 % floor: the old rule promised ±5 %,
        // which `max(4.99, 7.204)` never reaches.
        let s = Self.summary(resolvable: 10.145, floor: 10.145, repeatability: 7.204,
                             framesPerContact: 4.9286)
        let sentence = s.resolutionSentence
        XCTAssertFalse(sentence.contains("±5%"), "an unreachable promise: \(sentence)")
        XCTAssertTrue(sentence.contains("±7%"), sentence)
        // And the rate quoted is the rate that reaches the target it names.
        let needed = s.frameRateNeeded(forPercent: 7.204)
        XCTAssertTrue(sentence.contains(String(format: "%.0f fps", needed)), sentence)

        // With a repeatability below 5 % the promise is the usual ±5 %.
        let clean = Self.summary(resolvable: 8.086, floor: 8.086, repeatability: 2.564,
                                 framesPerContact: 6.1833)
        XCTAssertTrue(clean.resolutionSentence.contains("±5%"), clean.resolutionSentence)
    }

    /// And it must not recommend a rate the analysis window cannot cover.
    func testTheAdviceNeverRecommendsARateThePipelineCannotAnalyse() {
        // 1 frame per contact needs 300 fps to reach 5 %, past the 240 fps the
        // analysis window can still cover.
        let s = Self.summary(resolvable: 50, floor: 50, repeatability: 1.0,
                             framesPerContact: 1.0)
        XCTAssertGreaterThan(s.frameRateNeeded(forPercent: 5),
                             FrameSource.highestAnalysableFrameRate)
        XCTAssertFalse(s.resolutionSentence.contains("Filming at"), s.resolutionSentence)
        XCTAssertTrue(s.resolutionSentence.contains("analysis window"), s.resolutionSentence)
    }

    // MARK: - Fixtures

    private static let allVerdicts: [NimbleEngine.MotionVerdict] = [
        .hold, .movingBeyondStaticBudget, .indistinguishableFromNoise, .noMeasurement,
        .poseDidNotConverge, .gaitStance, .gaitFlight, .gaitOutsideAnalysis, .gaitRefused,
    ]

    private static func usableReport() throws -> GaitReport {
        let bundle = Bundle(for: GaitLoadSummaryTests.self)
        let frames = try GaitClipFixture.load("video_012", bundle: bundle).frames
        let report = try GaitAnalysis.analyse(frames: frames)
        XCTAssertTrue(report.isUsable)
        return report
    }

    private static func gaitFrame(id: Int, side: Int,
                                  activations: [String: Double],
                                  residual: Double = 0.1,
                                  solverLeft: Bool? = nil,
                                  solverRight: Bool? = nil,
                                  cleanWindow: Bool = true) -> OfflineResultStore.FrameResult {
        let outcome = NimbleEngine.GaitFrameOutcome(
            modelledVerticalForceInBodyWeights: 2.0,
            solvedVerticalForceInBodyWeights: 2.0 + residual,
            residualInBodyWeights: residual,
            contactSide: side,
            solverSawLeftContact: solverLeft ?? (side < 0),
            solverSawRightContact: solverRight ?? (side > 0),
            rootVerticalAccelerationMetersPerSecondSquared: 9.81,
            horizontalRootAccelerationModelled: false,
            derivativeWindowInsideContact: cleanWindow)
        let muscle = NimbleEngine.MuscleOutput(activations: activations,
                                               forces: activations.mapValues { $0 * 1000 },
                                               converged: true,
                                               timestamp: Double(id) / 30.0)
        return OfflineResultStore.FrameResult(
            id: id, sourceImage: UIImage(), timestamp: Double(id) / 30.0,
            status: .success, usedFallbackBBox: false, camT: nil, modelChecksums: nil,
            bodyFrame: nil, ikResult: nil, idResult: nil, muscleResult: muscle,
            isStaticHoldEstimate: false,
            motionState: .gait(verdict: .gaitStance, outcome: outcome))
    }

    private static func summary(resolvable: Double,
                                floor: Double? = nil,
                                repeatability: Double = 7,
                                framesPerContact: Double = 5,
                                fps: Double = 30,
                                residual: Double = 0.1) -> GaitLoadSummary {
        GaitLoadSummary(
            ranked: [
                .init(id: "glmax1", displayName: "Glute max (upper)",
                      leftPeak: 0.9, rightPeak: 0.3, leftFrames: 5, rightFrames: 5,
                      isSaturated: false),
                .init(id: "soleus", displayName: "Soleus",
                      leftPeak: 0.5, rightPeak: 0.49, leftFrames: 5, rightFrames: 5,
                      isSaturated: false),
            ],
            resolvableAsymmetryPercent: resolvable,
            quantisationFloorPercent: floor ?? resolvable,
            strideRepeatabilityPercent: repeatability,
            framesPerContact: framesPerContact,
            framesPerSecond: fps,
            stanceFrameCount: 10,
            claimedStanceFrameCount: 10,
            saturatedMuscleCount: 0,
            maxForceResidualInBodyWeights: residual,
            medianForceResidualInBodyWeights: residual,
            residualGatePassed: residual <= NimbleEngine.maxGaitForceResidualInBodyWeights,
            contactDetectorDisagreements: 0,
            framesWithoutACleanDerivativeWindow: 0,
            leftStanceFrameCount: 5,
            rightStanceFrameCount: 5,
            horizontalRootAccelerationModelled: false,
            derivativeFilterTaps: 5,
            derivativeFilterSpanMilliseconds: 133,
            shortestContactMilliseconds: 167,
            derivativeNoiseAmplification: 4.69)
    }
}
