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
                       s.verticalFalsifierSentence,
                       s.peakForceRegimeSentence,
                       s.perMuscleRetirementSentence]
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
            .analysed(report: report, plan: plan).isAboutRunning)
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

    /// A difference finer than the clip's resolution does not clear the
    /// statistical floor. Asserted on `clearsStatisticalFloor` rather than on
    /// `permits`, which is false for every muscle since the per-muscle claim was
    /// retired — the discrimination this test exists for is the floor, and it
    /// still has to work, because the floor is what a future de-retirement would
    /// rest on.
    func testAClaimFinerThanTheResolutionIsRefused() {
        let s = Self.summary(resolvable: 10)
        let even = Self.load(differencePercent: 4.0)
        XCTAssertEqual(even.differencePercent, 4.0, accuracy: 1e-9)
        XCTAssertFalse(s.clearsStatisticalFloor(even))
        XCTAssertEqual(s.claimFloorPercent(for: even), 10, accuracy: 1e-9)

        let real = Self.load(differencePercent: 100.0 * 0.2 / 0.6)
        XCTAssertTrue(s.clearsStatisticalFloor(real))
        XCTAssertEqual(real.heavierSide, "left")
        // And nothing reaches the user either way.
        XCTAssertFalse(s.permits(real))
    }

    /// Exactly at the boundary, and either side of it.
    func testTheRefusalBoundaryIsInclusive() {
        let s = Self.summary(resolvable: 10)
        XCTAssertTrue(s.clearsStatisticalFloor(Self.load(differencePercent: 10.0)))
        XCTAssertTrue(s.clearsStatisticalFloor(Self.load(differencePercent: -10.0)))
        XCTAssertTrue(s.clearsStatisticalFloor(Self.load(differencePercent: 10.001)))
        XCTAssertFalse(s.clearsStatisticalFloor(Self.load(differencePercent: 9.999)))
        XCTAssertFalse(s.clearsStatisticalFloor(Self.load(differencePercent: 0, mean: 0)))
        XCTAssertFalse(s.clearsStatisticalFloor(Self.load(differencePercent: 0)))
    }

    /// A failed falsifier withholds everything, however large the difference.
    func testAFailedResidualGateWithholdsEveryClaim() {
        let s = Self.summary(resolvable: 5, residual: 3.0)
        XCTAssertFalse(s.residualGatePassed)
        XCTAssertFalse(s.clearsStatisticalFloor(Self.load(differencePercent: 200)))
        XCTAssertFalse(s.arePublishable)
        XCTAssertTrue(try XCTUnwrap(s.withheldReason).contains("Withheld"))
    }

    /// **The retirement, at the level of the type.** No clip, no muscle, no
    /// difference size gets a per-muscle left/right statement through — and the
    /// statistical predicate underneath still discriminates, so the two are not
    /// the same switch wired twice.
    func testNoMuscleIsEverPermittedWhileTheClaimIsRetired() {
        XCTAssertFalse(GaitLoadSummary.perMuscleLeftRightClaimIsSupported)
        let s = Self.summary(resolvable: 5)
        for difference in [0.0, 4.0, 10.0, 50.0, 199.0] {
            let load = Self.load(differencePercent: difference)
            XCTAssertFalse(s.permits(load), "\(difference) % was permitted")
        }
        XCTAssertTrue(s.clearsStatisticalFloor(Self.load(differencePercent: 199.0)))
        XCTAssertFalse(s.clearsStatisticalFloor(Self.load(differencePercent: 1.0)))
    }

    // MARK: - Building from frames

    /// The report's cadence comes from the surviving frame timestamps. Track
    /// metadata is a different fact: sparse 10 fps sampling on a nominal 30 fps
    /// video used to hand both to this factory, and the UI trusted the latter.
    /// The factory and analysed outcome no longer accept that second source.
    func testTheSummaryCarriesTheReportsTimestampCadence() throws {
        let report = try Self.usableReport()
        let frames = [
            Self.gaitFrame(id: 0, side: -1, activations: ["soleus_l": 0.6]),
            Self.gaitFrame(id: 1, side: 1, activations: ["soleus_r": 0.5]),
        ]
        let summary = try XCTUnwrap(GaitLoadSummary.make(
            frames: frames, report: report, filterTaps: 5))
        let reportLabel = String(format: "at %.0f fps", report.framesPerSecond)
        let inventedMetadataLabel = String(format: "at %.0f fps", report.framesPerSecond * 3)
        XCTAssertEqual(summary.framesPerSecond, report.framesPerSecond, accuracy: 1e-12)
        XCTAssertTrue(summary.resolutionSentence.contains(reportLabel), summary.resolutionSentence)
        XCTAssertFalse(summary.resolutionSentence.contains(inventedMetadataLabel),
                       summary.resolutionSentence)
    }

    /// Each leg is credited only during ITS OWN stance, so the comparison is of
    /// sides and not of gait phases.
    func testEachLegIsCreditedOnlyDuringItsOwnContact() throws {
        let report = try Self.usableReport()
        let frames = [
            Self.gaitFrame(id: 0, side: -1, activations: ["glmax1_l": 0.80, "glmax1_r": 0.99]),
            Self.gaitFrame(id: 1, side: 1, activations: ["glmax1_l": 0.99, "glmax1_r": 0.40]),
        ]
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   filterTaps: 5))
        let load = try XCTUnwrap(s.muscles.first { $0.id == "glmax1" })
        XCTAssertEqual(load.leftLoad, 0.80, accuracy: 1e-9,
                       "the left value from the RIGHT contact must be ignored")
        XCTAssertEqual(load.rightLoad, 0.40, accuracy: 1e-9)
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
                                                   filterTaps: 5))
        XCTAssertEqual(s.muscles.map(\.id), ["soleus"])
        XCTAssertNil(GaitLoadSummary.split("multifidus_T9_T7"))
        XCTAssertEqual(GaitLoadSummary.split("bflh140_r")?.base, "bflh140")
        XCTAssertEqual(GaitLoadSummary.split("bflh140_r")?.side, "r")
    }

    /// **The list is NOT a strength ranking, and this pins that it is not.**
    ///
    /// It was sorted by `max(leftLoad, rightLoad)` under a doc comment reading
    /// "position 1 is the muscle carrying the most of this runner's stance
    /// load". That is a muscle-to-muscle comparison, and this build's moment
    /// arms cannot support one: `MomentArmErrorCancellationTests` measures that
    /// a per-muscle moment-arm error reorders exactly that key while leaving
    /// every left/right figure alone. So the order is now by how far each
    /// left/right claim clears its own floor.
    func testTheListIsOrderedByResolvedClaimAndNotByLoad() throws {
        let report = try Self.usableReport()
        // soleus is the WEAKEST muscle here and carries the LARGEST left/right
        // difference; glmax1 is the strongest and is nearly even. Sorting by
        // load puts glmax1 first, sorting by resolved claim puts soleus first,
        // so the two orderings are opposite and the test cannot pass by luck.
        let frames = (0..<4).map { i -> OfflineResultStore.FrameResult in
            let onLeft = i < 2
            return Self.gaitFrame(
                id: i, side: onLeft ? -1 : 1,
                activations: onLeft
                    ? ["soleus_l": 0.30, "glmax1_l": 0.90, "recfem_l": 0.50]
                    : ["soleus_r": 0.10, "glmax1_r": 0.88, "recfem_r": 0.40],
                contact: i)
        }
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   filterTaps: 5))
        XCTAssertTrue(s.arePublishable, s.withheldReason ?? "-")
        let byLoad = s.muscles
            .sorted { Swift.max($0.leftLoad, $0.rightLoad) > Swift.max($1.leftLoad, $1.rightLoad) }
            .map(\.id)
        XCTAssertEqual(byLoad, ["glmax1", "recfem", "soleus"], "the load order, for contrast")
        XCTAssertEqual(s.muscles.map(\.id), ["soleus", "recfem", "glmax1"],
                       "the published order is by resolved left/right claim")
        // And the ordering key is monotone in what it claims to be.
        let headroom = s.muscles.map { abs($0.differencePercent) / s.claimFloorPercent(for: $0) }
        for i in 1..<headroom.count {
            XCTAssertGreaterThanOrEqual(headroom[i - 1], headroom[i])
        }
        // ⚠️ And the head of that order is an ORDER STATISTIC. Nothing may show
        // the top k of it with a per-comparison error rate; the panel did, and
        // the interval that makes the head defensible is the family-wise one in
        // `samplingUncertaintyPercent`. This ordering has no screen consumer at
        // all now — `perMuscleLeftRightClaimIsSupported` is false — which is
        // asserted here so a future reader cannot re-attach one silently.
        XCTAssertFalse(GaitLoadSummary.perMuscleLeftRightClaimIsSupported)
        for load in s.muscles { XCTAssertFalse(s.permits(load), load.id) }
        print("UI-METRIC retirement_sentence=\(s.perMuscleRetirementSentence)")
    }

    /// Which muscles carry a path this build does not model — surfaced per row,
    /// because the row above is on a different scale and the user cannot know
    /// that from the numbers.
    func testAMuscleWithUnmodelledWrapGeometryIsFlaggedAndStillGetsItsLeftRightClaim() throws {
        let report = try Self.usableReport()
        let frames = (0..<4).map { i -> OfflineResultStore.FrameResult in
            let onLeft = i < 2
            return Self.gaitFrame(
                id: i, side: onLeft ? -1 : 1,
                activations: onLeft ? ["soleus_l": 0.70, "glmax1_l": 0.70]
                                    : ["soleus_r": 0.40, "glmax1_r": 0.40],
                contact: i)
        }
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   filterTaps: 5))
        let soleus = try XCTUnwrap(s.muscles.first { $0.id == "soleus" })
        let glmax = try XCTUnwrap(s.muscles.first { $0.id == "glmax1" })
        XCTAssertTrue(soleus.pathIsModelled, "soleus carries no PathWrap in either model")
        // glmax1 wraps around the pelvis on a WrapCylinder, and cylinder
        // wrapping ships since 2026-08-08, so its path IS modelled now. The
        // elbow ellipsoids followed on the same day, so the table is empty.
        XCTAssertTrue(glmax.pathIsModelled,
                      "glmax1's pelvis cylinder is solved since cylinder wrapping shipped")
        XCTAssertFalse(GaitLoadSummary.musclesWithUnmodelledPaths.contains("soleus"))
        XCTAssertFalse(GaitLoadSummary.musclesWithUnmodelledPaths.contains("BIClong"),
                       "BIClong's two WrapEllipsoids are solved since ellipsoid "
                       + "wrapping shipped")
        // The flag was a per-ROW warning, and the reason it is not enough is
        // measured in `MomentArmErrorCancellationTests`: the QP redistributes
        // load between synergists, so an unmodelled wrap on glmax1 moves
        // SOLEUS's left/right figure too, and soleus's row would have carried no
        // warning at all. The whole comparison is retired; the flag survives as
        // a property of the model, not as a caveat on a claim.
        XCTAssertTrue(s.clearsStatisticalFloor(glmax), "the statistics were never the problem")
        XCTAssertFalse(s.permits(glmax))
        XCTAssertFalse(s.permits(soleus))
    }

    /// Saturation is where "a force error is a common scale" stops holding, so
    /// it is counted and shown rather than quietly absorbed.
    ///
    /// **The count is of MUSCLES.** It counted muscle-SIDES — `soleus_l` and
    /// `soleus_r` as two — and the panel printed it in one sentence beside
    /// `flooredMuscleCount`, which counts muscles, so a clip that clipped ten
    /// muscles on both legs told the user twenty of theirs had maxed out.
    func testSaturatedMusclesAreCounted() throws {
        let report = try Self.usableReport()
        let frames = [
            Self.gaitFrame(id: 0, side: -1, activations: ["soleus_l": 1.0, "glmax1_l": 0.5]),
            Self.gaitFrame(id: 1, side: 1, activations: ["soleus_r": 0.999, "glmax1_r": 0.5]),
        ]
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   filterTaps: 5))
        XCTAssertEqual(s.saturatedMuscleCount, 1,
                       "one muscle clipped, on both of its sides — and one comparison lost")
    }

    /// A clip with no stance frame returns nil rather than an empty screen that
    /// looks like a measurement of zero.
    func testNoStanceFramesYieldsNoSummary() throws {
        let report = try Self.usableReport()
        XCTAssertNil(GaitLoadSummary.make(frames: [], report: report,
                                          filterTaps: 5))
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
                                                   filterTaps: 5))
        XCTAssertEqual(s.maxVerticalForceResidualInBodyWeights, 0.20, accuracy: 1e-9,
                       "the 2.90 BW frame is a detector disagreement, not limb inertia")
        XCTAssertTrue(s.residualGatePassed)
        XCTAssertEqual(s.contactDetectorDisagreements, 1,
                       "the right-foot frame where the solver saw the LEFT foot down")
        XCTAssertEqual(s.claimedStanceFrameCount, 4)
        XCTAssertEqual(s.stanceFrameCount, 3, "and its activations never entered a peak")
        let load = try XCTUnwrap(s.muscles.first { $0.id == "soleus" })
        XCTAssertEqual(load.rightLoad, 0.5, accuracy: 1e-9,
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
                                                   filterTaps: 5))
        XCTAssertEqual(s.framesWithoutACleanDerivativeWindow, 2)
        XCTAssertEqual(s.stanceFrameCount, 2)
        XCTAssertEqual(s.claimedStanceFrameCount, 4)
        let load = try XCTUnwrap(s.muscles.first { $0.id == "soleus" })
        XCTAssertEqual(load.leftLoad, 0.40, accuracy: 1e-9)
        XCTAssertEqual(load.rightLoad, 0.30, accuracy: 1e-9)
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
        XCTAssertFalse(failed.clearsStatisticalFloor(Self.load(differencePercent: 200)),
                       "a failed gate withholds even a huge left/right difference")
        // The data is still there — the ONLY thing between it and the screen is
        // the flag, which is exactly why the flag has to be asked.
        XCTAssertFalse(failed.muscles.isEmpty)
        let reason = try? XCTUnwrap(failed.withheldReason)
        XCTAssertTrue(reason!.contains("3.00"), "the refusal names the measurement: \(reason!)")
        XCTAssertTrue(reason!.lowercased().contains("film"), "and a lever: \(reason!)")
        XCTAssertNil(failed.perMuscleRetirementSentence.range(of: "harder on"),
                     "the retirement paragraph states no difference of its own")

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
        // Twelve alternating contacts. The first five agree — enough frames and
        // enough contacts on each side to clear those two gates — so the ONLY
        // thing left to fail is detector agreement, at 5/12.
        var frames: [OfflineResultStore.FrameResult] = []
        for i in 0..<12 {
            let side = i % 2 == 0 ? -1 : 1
            let disagree = i >= 5      // the solver saw neither foot down
            frames.append(Self.gaitFrame(id: i, side: side,
                                         activations: ["soleus_l": 0.5, "soleus_r": 0.5],
                                         solverLeft: disagree ? false : side < 0,
                                         solverRight: disagree ? false : side > 0,
                                         contact: i))
        }
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   filterTaps: 5))
        XCTAssertEqual(s.contactDetectorDisagreements, 7)
        XCTAssertEqual(s.agreementFraction, 5.0 / 12.0, accuracy: 1e-9)
        XCTAssertGreaterThanOrEqual(s.leftContactCount,
                                    GaitLoadSummary.minimumContactsPerSide)
        XCTAssertGreaterThanOrEqual(s.rightContactCount,
                                    GaitLoadSummary.minimumContactsPerSide)
        XCTAssertFalse(s.contactGatePassed)
        XCTAssertFalse(s.arePublishable)
        let reason = try XCTUnwrap(s.withheldReason)
        XCTAssertTrue(reason.contains("NO foot down"), reason)
        XCTAssertTrue(reason.lowercased().contains("ground"), reason)
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
                                                   filterTaps: 5))
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
        // Two contacts per side: one is not enough to estimate a side's own
        // step-to-step scatter, and the contact gate now says so.
        let frames = [
            Self.gaitFrame(id: 0, side: -1, activations: ["soleus_l": 1.0, "glmax1_l": 0.90],
                           contact: 0),
            Self.gaitFrame(id: 1, side: -1, activations: ["soleus_l": 1.0, "glmax1_l": 0.90],
                           contact: 1),
            Self.gaitFrame(id: 2, side: 1, activations: ["soleus_r": 0.40, "glmax1_r": 0.30],
                           contact: 2),
            Self.gaitFrame(id: 3, side: 1, activations: ["soleus_r": 0.40, "glmax1_r": 0.30],
                           contact: 3),
        ]
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   filterTaps: 5))
        XCTAssertTrue(s.arePublishable, "the clip as a whole is fine")
        let soleus = try XCTUnwrap(s.muscles.first { $0.id == "soleus" })
        let glmax = try XCTUnwrap(s.muscles.first { $0.id == "glmax1" })
        XCTAssertTrue(soleus.isSaturated)
        XCTAssertFalse(glmax.isSaturated)
        // Both differences are far above the resolution, so only saturation
        // separates them.
        XCTAssertGreaterThan(abs(soleus.differencePercent), s.resolvableAsymmetryPercent)
        XCTAssertGreaterThan(abs(glmax.differencePercent), s.resolvableAsymmetryPercent)
        XCTAssertFalse(s.clearsStatisticalFloor(soleus), "saturated: the bound, not a measurement")
        XCTAssertTrue(s.clearsStatisticalFloor(glmax), "its neighbour is unaffected by that bound")
        // And neither reaches the user, for the separate reason that the whole
        // per-muscle comparison is retired.
        XCTAssertFalse(s.permits(glmax))
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

    // MARK: - What the screen claims about the falsifier

    /// **A check that measured nothing is not a check that passed.**
    ///
    /// `sortedResiduals.last ?? 0` made an empty residual set report max 0,
    /// median 0 and `residualGatePassed = true`, and `honestyBlock` renders on
    /// every `.analysed` clip — so a ground-height estimate bad enough that no
    /// stance frame ever agreed printed "0.00 BW typical, 0.00 BW worst (gate
    /// 0.50) — passed." in the calm secondary tint. This is not a hypothetical
    /// margin: the suite's own end-to-end engine run leaves 4 agreeing frames of
    /// 9, four frames from the zero case.
    func testAFalsifierThatMeasuredNothingIsNotReportedAsPassed() throws {
        let report = try Self.usableReport()
        // Every stance frame disagrees, so no residual is ever recorded.
        let frames = (0..<6).map { i in
            Self.gaitFrame(id: i, side: i.isMultiple(of: 2) ? -1 : 1,
                           activations: ["soleus_l": 0.5, "soleus_r": 0.5],
                           residual: 0.0,
                           solverLeft: false, solverRight: false)
        }
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   filterTaps: 5))
        XCTAssertEqual(s.residualFrameCount, 0)
        XCTAssertFalse(s.residualWasMeasured)
        XCTAssertFalse(s.residualGatePassed, "a gate no frame reached has not been passed")
        XCTAssertFalse(s.arePublishable)
        XCTAssertTrue(s.verticalFalsifierSentence.contains("NOT MEASURED"),
                      s.verticalFalsifierSentence)
        XCTAssertFalse(s.verticalFalsifierSentence.lowercased().contains("passed"),
                       s.verticalFalsifierSentence)

        // The control: one agreeing frame per side and the check runs again, so
        // this is not "the sentence now always says NOT MEASURED".
        let measured = [
            Self.gaitFrame(id: 0, side: -1, activations: ["soleus_l": 0.5], residual: 0.10),
            Self.gaitFrame(id: 1, side: 1, activations: ["soleus_r": 0.5], residual: 0.12),
            Self.gaitFrame(id: 2, side: -1, activations: ["soleus_l": 0.5], residual: 0.11),
            Self.gaitFrame(id: 3, side: 1, activations: ["soleus_r": 0.5], residual: 0.13),
        ]
        let ok = try XCTUnwrap(GaitLoadSummary.make(frames: measured, report: report,
                                                    filterTaps: 5))
        XCTAssertEqual(ok.residualFrameCount, 4)
        XCTAssertTrue(ok.residualWasMeasured)
        XCTAssertTrue(ok.residualGatePassed)
        XCTAssertTrue(ok.verticalFalsifierSentence.contains("passed"), ok.verticalFalsifierSentence)
    }

    /// **The falsifier line is named for the axis it measures.**
    ///
    /// The residual is `|ΣF_y − F_gait|/(m·g)` — the two feet's VERTICAL force
    /// only; `leftFootForce.x/.z` exist in the bridge's output and are discarded.
    /// It was labelled "Limb inertia the timing model omits", i.e. `‖a_artic‖/g`,
    /// one line below the disclosure that braking and push-off are not modelled
    /// — certifying a cancellation on the one axis it never examined, where
    /// STATUS sizes the error at 0.2-0.35 BW against a measured 0.02.
    func testTheFalsifierLineNamesTheAxisItActuallyMeasures() {
        let s = Self.summary(resolvable: 10)
        let sentence = s.verticalFalsifierSentence
        XCTAssertTrue(sentence.lowercased().contains("vertical"), sentence)
        XCTAssertTrue(sentence.lowercased().contains("fore-aft"),
                      "and it names what it does NOT see: \(sentence)")
        XCTAssertFalse(sentence.lowercased().contains("limb inertia"),
                       "the old label claimed the whole vector: \(sentence)")
        // The unmodelled-term disclosure and the falsifier now agree about which
        // axis is which, instead of contradicting each other three lines apart.
        XCTAssertTrue(s.unmodelledTermSentence.lowercased().contains("braking"))
        XCTAssertTrue(s.unmodelledTermSentence.lowercased().contains("fore-aft"),
                      "the two lines name the two axes between them: "
                      + s.unmodelledTermSentence)
    }

    // MARK: - What the screen claims about the runner and the force scale

    /// **"This runner's own stride-to-stride variation ±0%" is not a finding.**
    ///
    /// It is a coefficient of variation over touchdown gaps quantised to whole
    /// frames: on `video_012` every gap is exactly 18 samples, so the CV is
    /// exactly 0 while the clip cannot distinguish anything below 100/18 =
    /// 5.56 %. The floored number is what the sentence and the frame-rate
    /// promise must both use.
    func testAZeroStrideScatterIsShownAsTheFloorItActuallyIs() {
        let s = Self.summary(resolvable: 10.145, floor: 10.145,
                             repeatability: 100.0 / 18.0, measuredRepeatability: 0.0,
                             repeatabilityBound: 100.0 / 18.0, framesPerContact: 4.9286)
        let breakdown = s.resolutionBreakdownSentence
        XCTAssertFalse(breakdown.contains("variation ±0%"), breakdown)
        XCTAssertTrue(breakdown.contains("5.6%"), "it names the bound: \(breakdown)")
        XCTAssertTrue(breakdown.contains("0.0%"), "and the raw measurement: \(breakdown)")

        // The promise the raw 0.000 used to license: "filming at 61 fps would
        // resolve ±5%", on a clip whose own stride scatter could be 5.6 %.
        XCTAssertEqual(s.bestAchievablePercentAtAnyFrameRate, 100.0 / 18.0, accuracy: 1e-9)
        let sentence = s.resolutionSentence
        XCTAssertFalse(sentence.contains("±5%"), "an undeliverable promise: \(sentence)")
        XCTAssertTrue(sentence.contains("±6%"), sentence)
        XCTAssertTrue(sentence.contains("55 fps"), sentence)
        print("UI-METRIC floored_repeatability=\(sentence) | \(breakdown)")

        // A runner whose measured scatter genuinely exceeds the bound keeps it,
        // so the floor is a floor and not a replacement.
        let variable = Self.summary(resolvable: 20, floor: 8, repeatability: 20,
                                    measuredRepeatability: 20, repeatabilityBound: 5.56,
                                    framesPerContact: 6.25)
        XCTAssertTrue(variable.resolutionBreakdownSentence.contains("variation ±20%"),
                      variable.resolutionBreakdownSentence)
    }

    /// **The circularity, closed and said out loud.** `Fmax_side = (π/2)(1 +
    /// tf/tc_side)` turns the left/right contact difference into a per-leg force
    /// scale that lands inside every muscle bar. Where the clip cannot resolve
    /// that difference it is no longer used — and the screen says which of the
    /// two regimes it is in, because "the legs measured the same" and "this clip
    /// refused to distinguish them" are different statements.
    func testThePeakForceRegimeIsStatedOnTheSameScreenAsTheBars() {
        let shared = Self.summary(resolvable: 10, sharedPeak: true)
        XCTAssertTrue(shared.peakForceIsSharedBetweenLegs)
        XCTAssertTrue(shared.peakForceRegimeSentence.contains("MEAN contact time"),
                      shared.peakForceRegimeSentence)
        XCTAssertTrue(shared.peakForceRegimeSentence.contains("not inside the muscle numbers"),
                      shared.peakForceRegimeSentence)

        // The per-leg branch has to state the SIZE of what it injects, in
        // percentage POINTS, on a summary that actually carries a non-zero term
        // — quoting "0 percentage points" for the default fixture would have let
        // the wording pass without the number ever being exercised.
        let perLeg = Self.summary(resolvable: 10, sharedPeak: false,
                                  contactTimeContribution: -9.3863)
        XCTAssertTrue(perLeg.peakForceRegimeSentence.contains("OWN contact time"),
                      perLeg.peakForceRegimeSentence)
        XCTAssertTrue(perLeg.peakForceRegimeSentence.contains("9 percentage points"),
                      "the size is stated in its own unit: \(perLeg.peakForceRegimeSentence)")
        XCTAssertTrue(perLeg.peakForceRegimeSentence.contains("right-high"),
                      perLeg.peakForceRegimeSentence)
    }

    // MARK: - The floors a claim has to clear

    /// **The saturation test is the SOLVER's tolerance, not a hand-picked
    /// constant.**
    ///
    /// It was 0.999. OSQP stops when the primal residual is under
    /// `eps_abs + eps_rel·max(‖Ax‖∞, ‖z‖∞)`; with `A = I`, `z ∈ [0.02, 1]` and
    /// `eps_abs = eps_rel = 1e-3` that is 2e-3, and `MuscleSolver` accepts
    /// `OSQP_SOLVED_INACCURATE` — the same check at ten times the tolerance —
    /// with polishing off. So a muscle genuinely pinned at `a ≤ 1` comes back as
    /// low as 0.98, `isSaturated` read false for it, `%.2f` printed it as
    /// "1.00", and the one gate protecting the ratio argument passed exactly the
    /// case it exists to catch.
    func testTheSaturationThresholdIsTheSolversOwnTolerance() throws {
        XCTAssertEqual(GaitLoadSummary.saturationThreshold,
                       MuscleSolver.maxActivation - MuscleSolver.saturationActivationTolerance,
                       accuracy: 1e-12)
        XCTAssertEqual(GaitLoadSummary.saturationThreshold, 0.98, accuracy: 1e-12,
                       "1 - 10*(1e-3 + 1e-3) — change this only when the solver's settings change")
        print("QP-METRIC saturation_threshold=\(GaitLoadSummary.saturationThreshold) "
              + "tolerance=\(MuscleSolver.saturationActivationTolerance)")

        let report = try Self.usableReport()
        // 0.982 is what a clipped muscle can return; 0.97 is genuinely below the
        // bound. The old 0.999 test called both of them unsaturated.
        let frames = (0..<4).map { i -> OfflineResultStore.FrameResult in
            let onLeft = i < 2
            return Self.gaitFrame(
                id: i, side: onLeft ? -1 : 1,
                activations: onLeft ? ["soleus_l": 0.982, "glmax1_l": 0.97]
                                    : ["soleus_r": 0.50, "glmax1_r": 0.50],
                contact: i)
        }
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   filterTaps: 5))
        let soleus = try XCTUnwrap(s.muscles.first { $0.id == "soleus" })
        let glmax = try XCTUnwrap(s.muscles.first { $0.id == "glmax1" })
        XCTAssertTrue(soleus.isSaturated, "0.982 is on the bound within OSQP's own tolerance")
        XCTAssertFalse(glmax.isSaturated, "0.97 is not, so the threshold is not simply lower")
        XCTAssertFalse(s.clearsStatisticalFloor(soleus), "a saturated muscle publishes nothing")
        XCTAssertEqual(s.saturatedMuscleCount, 1, "and it is counted once, as one muscle")
    }

    /// **A claim has to clear the statistic's OWN noise, not only the clip's
    /// timing quantisation.**
    ///
    /// The published floor was `max(50/framesPerContact, 100/stridePeriodFrames)`
    /// — both terms about WHEN samples were taken, neither about how much the
    /// samples scattered. Measured consequence, from this suite's own bias
    /// harness: at σ = 0.12 of activation the per-clip standard deviation of
    /// `differencePercent` is 9.47 % against `video_012`'s 10.145 % floor, so a
    /// perfectly symmetric runner publishes a false finding on roughly one
    /// muscle in four, with every clip-level gate green.
    func testAClaimMustAlsoClearItsOwnStepToStepScatter() throws {
        let report = try Self.usableReport()
        // Four contacts a side. Left and right differ by 10.5 % of the mean —
        // just above the clip's 10.145 % timing floor — but each side swings by
        // ±0.10 from contact to contact, so the difference is inside the noise.
        let leftValues = [0.60, 0.40, 0.60, 0.40]
        let rightValues = [0.55, 0.35, 0.55, 0.35]
        var frames: [OfflineResultStore.FrameResult] = []
        for (i, v) in leftValues.enumerated() {
            frames.append(Self.gaitFrame(id: i, side: -1, activations: ["soleus_l": v],
                                         contact: i))
        }
        for (i, v) in rightValues.enumerated() {
            frames.append(Self.gaitFrame(id: 10 + i, side: 1, activations: ["soleus_r": v],
                                         contact: 10 + i))
        }
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   filterTaps: 5))
        let load = try XCTUnwrap(s.muscles.first { $0.id == "soleus" })
        XCTAssertTrue(s.arePublishable, "the CLIP is fine — this is a per-muscle refusal")
        XCTAssertGreaterThan(abs(load.differencePercent), s.resolvableAsymmetryPercent,
                             "the old timing-only gate would have published this")
        print("GAIT-METRIC per_muscle_noise difference=\(load.differencePercent) "
              + "timing_floor=\(s.resolvableAsymmetryPercent) "
              + "sampling_floor=\(load.samplingUncertaintyPercent) "
              + "claim_floor=\(s.claimFloorPercent(for: load))")
        XCTAssertGreaterThan(load.samplingUncertaintyPercent, abs(load.differencePercent),
                             "this muscle's own scatter is larger than the difference")
        XCTAssertFalse(s.clearsStatisticalFloor(load))

        // The control: the SAME means with no contact-to-contact scatter do
        // publish, so this is not a gate that now refuses everything.
        var steady: [OfflineResultStore.FrameResult] = []
        for i in 0..<4 {
            steady.append(Self.gaitFrame(id: i, side: -1, activations: ["soleus_l": 0.50],
                                         contact: i))
            steady.append(Self.gaitFrame(id: 10 + i, side: 1, activations: ["soleus_r": 0.45],
                                         contact: 10 + i))
        }
        let calm = try XCTUnwrap(GaitLoadSummary.make(frames: steady, report: report,
                                                      filterTaps: 5))
        let calmLoad = try XCTUnwrap(calm.muscles.first { $0.id == "soleus" })
        XCTAssertEqual(calmLoad.samplingUncertaintyPercent, 0, accuracy: 1e-12)
        XCTAssertTrue(calm.clearsStatisticalFloor(calmLoad),
                      "zero scatter, so the timing floor is the only one left and it clears it")
    }

    /// The multiplier is Student-t and not 1.96, because a clip has a handful of
    /// contacts and at four degrees of freedom a normal interval is a third too
    /// narrow.
    func testTheUncertaintyMultiplierIsStudentTAtSmallCounts() {
        XCTAssertEqual(GaitLoadSummary.tMultiplier(degreesOfFreedom: 1), 12.706, accuracy: 1e-3)
        XCTAssertEqual(GaitLoadSummary.tMultiplier(degreesOfFreedom: 4), 2.776, accuracy: 1e-3)
        XCTAssertEqual(GaitLoadSummary.tMultiplier(degreesOfFreedom: 40), 2.131, accuracy: 1e-3,
                       "held at the df=15 value rather than relaxed to 1.96")
        XCTAssertFalse(GaitLoadSummary.tMultiplier(degreesOfFreedom: 0).isFinite)
        // The multiplier is now COMPUTED, not looked up, because the corrected
        // level is a property of the clip. These are the published two-sided
        // 95 % values it has to reproduce, and three deeper tails the old table
        // never carried — checked against R's `qt`.
        for (df, expected) in [(2, 4.303), (3, 3.182), (5, 2.571), (10, 2.228), (15, 2.131)] {
            XCTAssertEqual(GaitLoadSummary.tMultiplier(degreesOfFreedom: df), expected,
                           accuracy: 1e-3, "df=\(df)")
        }
        XCTAssertEqual(StudentT.twoSidedQuantile(alpha: 0.01, degreesOfFreedom: 10),
                       3.169, accuracy: 1e-3)
        XCTAssertEqual(StudentT.twoSidedQuantile(alpha: 0.001, degreesOfFreedom: 5),
                       6.869, accuracy: 1e-3)
        XCTAssertEqual(StudentT.twoSidedQuantile(alpha: 0.0001, degreesOfFreedom: 4),
                       15.544, accuracy: 1e-2)
        // And the correction only ever widens.
        XCTAssertGreaterThan(GaitLoadSummary.tMultiplier(degreesOfFreedom: 4, comparisons: 175),
                             GaitLoadSummary.tMultiplier(degreesOfFreedom: 4, comparisons: 1))
        // One contact on a side means no scatter estimate at all.
        XCTAssertFalse(GaitLoadSummary
            .samplingUncertaintyPercent(left: [0.5], right: [0.4, 0.4]).isFinite)
    }

    /// **Frames are not contacts.** The gate counted usable FRAMES, so a clip
    /// whose left leg lost five of its six contacts but kept three clean frames
    /// in the sixth passed with one contact against six — one side's mean
    /// carrying σ and the other σ/√6, with nothing on screen saying so.
    func testOneContactOnASideCannotPublishHoweverManyFramesItHas() throws {
        let report = try Self.usableReport()
        // Left: ONE contact holding five frames. Right: four contacts.
        var frames = (0..<5).map { i in
            Self.gaitFrame(id: i, side: -1, activations: ["soleus_l": 0.70], contact: 0)
        }
        frames += (0..<4).map { i in
            Self.gaitFrame(id: 10 + i, side: 1, activations: ["soleus_r": 0.40], contact: 10 + i)
        }
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   filterTaps: 5))
        XCTAssertEqual(s.leftStanceFrameCount, 5, "the frame count alone would pass")
        XCTAssertEqual(s.leftContactCount, 1)
        XCTAssertEqual(s.rightContactCount, 4)
        XCTAssertFalse(s.contactGatePassed)
        XCTAssertFalse(s.arePublishable)
        let reason = try XCTUnwrap(s.withheldReason)
        XCTAssertTrue(reason.contains("contacts"), reason)
        XCTAssertTrue(reason.lowercased().contains("step to step"), reason)
        print("UI-METRIC one_contact_refusal=\(reason)")
    }

    /// **The contact-time term is named with its size, and the floor widens by
    /// it.** The screen said "part of the left/right difference in these bars is
    /// that contact-time difference re-expressed as force" and never how much —
    /// and it can be all of it.
    func testTheContactTimeTermIsSizedAndWidensEveryClaimFloor() {
        // The panel's own worked example: tcL = 200 ms, tcR = 160 ms, tf = 130 ms.
        let peaks = Bilateral(
            left: GaitForceModel.peakInBodyWeights(contactSeconds: 0.200, flightSeconds: 0.130),
            right: GaitForceModel.peakInBodyWeights(contactSeconds: 0.160, flightSeconds: 0.130))
        let injected = GaitLoadSummary.contactTimePeakContributionPercent(peaks: peaks)
        XCTAssertEqual(peaks.left, 2.5918, accuracy: 1e-4)
        XCTAssertEqual(peaks.right, 2.8471, accuracy: 1e-4)
        XCTAssertEqual(injected, -9.3863, accuracy: 1e-3)
        print("GAIT-METRIC contact_time_injection_percent=\(injected)")

        let s = Self.summary(resolvable: 8.086, sharedPeak: false,
                             contactTimeContribution: injected)
        let load = Self.load(differencePercent: -9.0)
        XCTAssertEqual(s.claimFloorPercent(for: load), 8.086 + 9.3863, accuracy: 1e-3)
        XCTAssertFalse(s.clearsStatisticalFloor(load),
                       "a 9 % reading that the contact-time term alone can produce is not a "
                       + "muscle finding, however far above the timing floor it sits")
        XCTAssertTrue(s.clearsStatisticalFloor(Self.load(differencePercent: -25.0)),
                      "and a difference well past the injected term still clears the floor")
        // The size is quoted in the unit it is in. It used to be phrased as a
        // SHARE — "9 % of every bar's left/right difference is contact time" —
        // from which a reader takes 9 % of 13 % = 1.2 points of artefact where
        // the truth is 9.4, and concludes their imbalance is 3.3x what it is.
        XCTAssertTrue(s.peakForceRegimeSentence.contains("9 percentage points"),
                      s.peakForceRegimeSentence)
        XCTAssertFalse(s.peakForceRegimeSentence.contains("9% of"), s.peakForceRegimeSentence)
        XCTAssertTrue(s.peakForceRegimeSentence.contains("right"), s.peakForceRegimeSentence)
        print("UI-METRIC peak_force_regime=\(s.peakForceRegimeSentence)")

        // Shared peaks inject nothing, so the floor is the timing floor again.
        let shared = Self.summary(resolvable: 8.086, sharedPeak: true)
        XCTAssertEqual(shared.claimFloorPercent(for: load), 8.086, accuracy: 1e-9)
    }

    // MARK: - What the screen says about what it did not measure

    /// **The fore-aft term is PRESENT and fabricated, not absent.**
    ///
    /// The disclosure said braking and push-off were "not modelled, so fore-aft
    /// joint loads are missing a term", which is false in the flattering
    /// direction: `getMultipleContactInverseDynamicsNearCoP` solves full 6-D
    /// wrenches subject to Newton-Euler, so it ASSIGNS a fore-aft ground force —
    /// whatever makes the horizontal CoM acceleration match a pelvis
    /// `MHRRetarget` holds still. STATUS sizes the error at 0.2-0.35 BW, larger
    /// than the worst vertical residual the same screen reports as passing.
    func testTheForeAftDisclosureSaysTheTermIsFabricatedNotMissing() {
        let s = Self.summary(resolvable: 10)
        let sentence = s.unmodelledTermSentence
        print("UI-METRIC unmodelled_term=\(sentence)")
        XCTAssertTrue(sentence.lowercased().contains("fabricated"), sentence)
        XCTAssertTrue(sentence.contains("0.2-0.35 body weights"),
                      "the size, so it can be compared with the residual above it: \(sentence)")
        XCTAssertFalse(sentence.lowercased().contains("missing a term"),
                       "the old wording said the opposite of what happens: \(sentence)")
        XCTAssertFalse(sentence.lowercased().contains("not modelled"), sentence)
        // And the falsifier line agrees with it instead of contradicting it.
        XCTAssertTrue(s.verticalFalsifierSentence.lowercased().contains("present and fabricated"),
                      s.verticalFalsifierSentence)
    }

    /// **A double contact and a missing contact are different failures with
    /// different levers, and the refusal has to name the right one.**
    ///
    /// Collapsed into one counter, the message read "the foot's height above the
    /// ground disagreed that it was planted … Film side-on, with the whole body
    /// and the ground in frame" — false for a double contact (the height agreed
    /// it was planted; it also flagged the other foot) and advice that cannot
    /// work, because the ground was never the problem.
    func testADoubleContactRefusalNamesTheThresholdAndNotTheGround() throws {
        let report = try Self.usableReport()
        // Ten stance frames, six of them double contacts: 40 % agreement. Two
        // agreeing frames and two contacts survive on each side, so the contact
        // COUNT and FRAME gates both clear and the only failure is agreement.
        let doubled = (0..<10).map { i -> OfflineResultStore.FrameResult in
            let onLeft = i < 5
            let agrees = (i % 5) < 2
            return Self.gaitFrame(id: i, side: onLeft ? -1 : 1,
                                  activations: ["soleus_l": 0.5, "soleus_r": 0.5],
                                  solverLeft: agrees ? onLeft : true,
                                  solverRight: agrees ? !onLeft : true,
                                  contact: i)
        }
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: doubled, report: report,
                                                   filterTaps: 5))
        XCTAssertEqual(s.solverSawDoubleContactCount, 6)
        XCTAssertEqual(s.contactDetectorDisagreements, 6)
        XCTAssertFalse(s.arePublishable)
        let reason = try XCTUnwrap(s.withheldReason)
        print("UI-METRIC double_contact_refusal=\(reason)")
        XCTAssertTrue(reason.contains("BOTH feet"), reason)
        XCTAssertTrue(reason.contains("6 cm"), "it names the threshold that caused it: \(reason)")
        XCTAssertTrue(reason.lowercased().contains("re-filming will not move it"),
                      "and refuses to offer a lever that cannot help: \(reason)")
        XCTAssertFalse(reason.contains("ground in frame"), reason)

        // The control: the same shape of failure with NO foot seen keeps the
        // ground-height sentence, so the split is real and not a rename.
        let missing = (0..<10).map { i -> OfflineResultStore.FrameResult in
            let onLeft = i < 5
            let agrees = (i % 5) < 2
            return Self.gaitFrame(id: i, side: onLeft ? -1 : 1,
                                  activations: ["soleus_l": 0.5, "soleus_r": 0.5],
                                  solverLeft: agrees ? onLeft : false,
                                  solverRight: agrees ? !onLeft : false,
                                  contact: i)
        }
        let m = try XCTUnwrap(GaitLoadSummary.make(frames: missing, report: report,
                                                   filterTaps: 5))
        XCTAssertEqual(m.solverSawDoubleContactCount, 0)
        let groundReason = try XCTUnwrap(m.withheldReason)
        XCTAssertTrue(groundReason.contains("NO foot down"), groundReason)
        XCTAssertTrue(groundReason.contains("ground in frame"), groundReason)
    }

    /// **A refusal does not take the posture findings down with it.**
    ///
    /// `isAboutRunning` is true for every refusal including `.notRunning`, whose
    /// entire meaning is "this is not running", so a side-on squat or a walking
    /// clip was routed to a panel headed "Running, but withheld" and lost the
    /// measurements it could actually support — measurements that had been
    /// computed and were sitting in the store.
    func testOnlyAnAnalysedRunReplacesThePostureFindings() throws {
        let report = try Self.usableReport()
        let plan = try XCTUnwrap(OfflineSessionRunner.makePlan(from: report))
        XCTAssertFalse(OfflineResultStore.GaitOutcome
            .notAttempted(reason: "1 usable frame").replacesPostureFindings)
        XCTAssertFalse(OfflineResultStore.GaitOutcome.refused(report: report)
            .replacesPostureFindings,
            "a refused run has nothing to put in the findings' place")
        XCTAssertTrue(OfflineResultStore.GaitOutcome
            .analysed(report: report, plan: plan).replacesPostureFindings)
    }

    /// The gait screen carries the SAME not-a-diagnosis note as the panel it
    /// replaces. It is the more clinical-sounding of the two — named anatomy, a
    /// 0-1 effort figure per side, a left/right verdict in warning orange — and
    /// it was the one without the note.
    func testTheGaitScreenCarriesTheSameNotADiagnosisNote() {
        XCTAssertEqual(GaitReportPanel.alwaysVisibleNote, PostureFindings.alwaysVisibleNote)
        XCTAssertTrue(GaitReportPanel.alwaysVisibleNote.contains("not diagnoses"),
                      GaitReportPanel.alwaysVisibleNote)
    }

    /// **Every refusal names its own cause and its own lever**, and the two that
    /// a faster camera fixes quote the rate the app already knows.
    func testEveryRefusalCarriesItsOwnAdviceAndTheRateOnesNameARate() {
        let all: [GaitReport.Refusal] = [
            .tooFewContacts(side: .left, count: 0),
            .stridePeriodDisagreesBetweenLegs(frames: 2.0),
            .contactSequenceNotPeriodic(frames: 0.9),
            .strideNotSteady(side: .right, percent: 18.9, boundPercent: 5.56),
            .notRunning(dutyFactor: 0.62, flightToContactRatio: 0),
            .stanceBudgetInconsistent(assumed: 5, measured: 8),
            .contactTooShortToResolve(framesPerContact: 2.8),
            .droppedSamplesInContact(side: .left, inside: 1, atEdges: 0),
            .contactTooShortForACleanDerivative(medianSamples: 4, neededSamples: 5),
        ]
        var seen: Set<String> = []
        for refusal in all {
            let advice = refusal.advice(framesPerSecond: 30)
            XCTAssertFalse(advice.isEmpty, "\(refusal) has no advice")
            seen.insert(advice)
            print("UI-METRIC refusal_advice \(refusal) -> \(advice)")
        }
        XCTAssertEqual(seen.count, all.count - 1,
                       "only the two stride-steadiness refusals may share a sentence")

        // The rate refusals quote the arithmetic: 30 fps at 2.8 frames per
        // contact needs 33 to reach 3; a 4-frame median needs 38 to reach 5.
        XCTAssertTrue(GaitReport.Refusal.contactTooShortToResolve(framesPerContact: 2.8)
            .advice(framesPerSecond: 30).contains("33 fps"))
        XCTAssertTrue(GaitReport.Refusal
            .contactTooShortForACleanDerivative(medianSamples: 4, neededSamples: 5)
            .advice(framesPerSecond: 30).contains("38 fps"))
        // And the one sentence that used to be printed under all nine is not the
        // sentence any of these gets.
        let old = NimbleEngine.MotionVerdict.gaitRefused.advice
        for refusal in all {
            XCTAssertNotEqual(refusal.advice(framesPerSecond: 30), old, "\(refusal)")
        }
        // "Not running" must not tell the user to run more steadily.
        let notRunning = GaitReport.Refusal
            .notRunning(dutyFactor: 0.62, flightToContactRatio: 0).advice(framesPerSecond: 30)
        XCTAssertTrue(notRunning.lowercased().contains("posture"), notRunning)
        XCTAssertFalse(notRunning.lowercased().contains("steady pace"), notRunning)
    }

    // MARK: - Fixtures

    private static let allVerdicts: [NimbleEngine.MotionVerdict] = [
        .hold, .movingBeyondStaticBudget, .indistinguishableFromNoise, .noMeasurement,
        .poseDidNotConverge, .gaitStance, .gaitFlight, .gaitOutsideAnalysis, .gaitRefused,
    ]

    /// A muscle load with an exact `differencePercent`. `mean = 0` makes the
    /// statistic NaN, which is the "no load on one side" case.
    static func load(differencePercent d: Double,
                     mean: Double = 0.5,
                     uncertainty: Double = 0,
                     saturated: Bool = false,
                     contacts: Int = 5) -> GaitLoadSummary.MuscleLoad {
        .init(id: "x", displayName: "X",
              leftLoad: mean + mean * d / 200, rightLoad: mean - mean * d / 200,
              leftContacts: contacts, rightContacts: contacts,
              isSaturated: saturated, isAtActivationFloor: false,
              samplingUncertaintyPercent: uncertainty, pathIsModelled: true)
    }

    private static func usableReport() throws -> GaitReport {
        let bundle = Bundle(for: GaitLoadSummaryTests.self)
        let frames = try GaitClipFixture.load("video_012", bundle: bundle).frames
        let report = try GaitAnalysis.analyse(frames: frames)
        XCTAssertTrue(report.isUsable)
        return report
    }

    /// - Parameter contact: the plan's contact index. Frames sharing one are
    ///   the same foot-strike. Defaults to one contact per side, which is what
    ///   most fixtures here mean.
    private static func gaitFrame(id: Int, side: Int,
                                  activations: [String: Double],
                                  residual: Double = 0.1,
                                  solverLeft: Bool? = nil,
                                  solverRight: Bool? = nil,
                                  cleanWindow: Bool = true,
                                  contact: Int? = nil) -> OfflineResultStore.FrameResult {
        let outcome = NimbleEngine.GaitFrameOutcome(
            modelledVerticalForceInBodyWeights: 2.0,
            solvedVerticalForceInBodyWeights: 2.0 + residual,
            residualInBodyWeights: residual,
            contactSide: side,
            contactIndex: contact ?? (side < 0 ? 0 : 1),
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

    static func summary(resolvable: Double,
                        floor: Double? = nil,
                        repeatability: Double = 7,
                        measuredRepeatability: Double? = nil,
                        repeatabilityBound: Double = 0,
                        sharedPeak: Bool = true,
                        framesPerContact: Double = 5,
                        fps: Double = 30,
                        residual: Double = 0.1,
                        residualFrameCount: Int = 10,
                        uncertainty: Double = 0,
                        contactTimeContribution: Double = 0,
                        // Defaults to "the contact durations added nothing", so
                        // every string assertion written before the contact
                        // sampling term existed still exercises the branch it
                        // was written for. Tests that care pass them.
                        contactClaimFloor: Double? = nil,
                        contactUncertainty: Double = 0) -> GaitLoadSummary {
        GaitLoadSummary(
            muscles: [
                .init(id: "glmax1", displayName: "Glute max (upper)",
                      leftLoad: 0.9, rightLoad: 0.3, leftContacts: 5, rightContacts: 5,
                      isSaturated: false, isAtActivationFloor: false,
                      samplingUncertaintyPercent: uncertainty, pathIsModelled: false),
                .init(id: "soleus", displayName: "Soleus",
                      leftLoad: 0.5, rightLoad: 0.49, leftContacts: 5, rightContacts: 5,
                      isSaturated: false, isAtActivationFloor: false,
                      samplingUncertaintyPercent: uncertainty, pathIsModelled: true),
            ],
            resolvableAsymmetryPercent: resolvable,
            quantisationFloorPercent: floor ?? resolvable,
            strideRepeatabilityPercent: repeatability,
            measuredStrideRepeatabilityPercent: measuredRepeatability ?? repeatability,
            strideRepeatabilityBoundPercent: repeatabilityBound,
            contactClaimFloorPercent: contactClaimFloor ?? resolvable,
            contactSamplingUncertaintyPercent: contactUncertainty,
            peakForceIsSharedBetweenLegs: sharedPeak,
            contactTimeContributionPercent: contactTimeContribution,
            framesPerContact: framesPerContact,
            framesPerSecond: fps,
            stanceFrameCount: 10,
            claimedStanceFrameCount: 10,
            screenedComparisonCount: 2,
            saturatedMuscleCount: 0,
            flooredMuscleCount: 0,
            maxVerticalForceResidualInBodyWeights: residual,
            medianVerticalForceResidualInBodyWeights: residual,
            residualFrameCount: residualFrameCount,
            residualGatePassed: residualFrameCount > 0
                && residual <= NimbleEngine.maxGaitForceResidualInBodyWeights,
            contactDetectorDisagreements: 0,
            solverSawDoubleContactCount: 0,
            framesWithoutACleanDerivativeWindow: 0,
            leftStanceFrameCount: 5,
            rightStanceFrameCount: 5,
            leftContactCount: 5,
            rightContactCount: 5,
            horizontalRootAccelerationModelled: false,
            derivativeFilterTaps: 5,
            derivativeFilterSpanMilliseconds: 133,
            shortestContactMilliseconds: 167,
            derivativeNoiseAmplification: 4.69)
    }
}
