import XCTest
import UIKit
@testable import BioMotion

/// **Every surface that states something about the user's body, and what each
/// one is allowed to say.**
///
/// The recurring failure in this project is not a wrong number — it is a right
/// number on the wrong surface: a retired claim surviving in a picture, in a
/// badge, or in the branch of an `if/else` nobody re-read. Three of them are
/// pinned here.
final class ClaimSurfaceTests: XCTestCase {

    // MARK: - The product-wide contact-support refusal

    /// The old live repair narrowed an unsupported L/R split to a `GRF sum`.
    /// The contact audit found that the SUM is unvalidated too: neither bundled
    /// model defines contact geometry, and the active solver supplies no valid
    /// support mechanics. The surface must refuse the whole dynamics family,
    /// not merely explain one split.
    func testContactSupportRefusalNamesEveryWithheldDynamicsFamily() {
        let availability = NimbleEngine.DynamicsAvailability.contactSupportUnavailable
        let detail = availability.detail
        print("UI-METRIC contact_support_refusal=\(detail)")

        XCTAssertFalse(availability.hasInverseDynamics)
        XCTAssertTrue(availability.title.lowercased().contains("pose only"), availability.title)
        XCTAssertTrue(availability.title.lowercased().contains("foot contact"), availability.title)
        for term in ["joint torque", "ground force", "centre of pressure",
                     "muscle effort", "gait-load"] {
            XCTAssertTrue(detail.lowercased().contains(term),
                          "the refusal omitted \(term): \(detail)")
        }
    }

    /// This is a model capability, not a capture-quality gate. The same text
    /// must preserve the outputs that genuinely remain and close the tempting
    /// "try a better video" workaround. Ground trust has its own text, but it
    /// must also say that trust is not sufficient by itself.
    func testContactSupportRefusalPreservesKinematicsAndCannotSellAReshoot() throws {
        let detail = NimbleEngine.DynamicsAvailability.contactSupportUnavailable.detail
        for term in ["Pose", "anatomy", "contact timing"] {
            XCTAssertTrue(detail.contains(term), "the refusal hid surviving \(term): \(detail)")
        }
        XCTAssertTrue(detail.lowercased().contains("refilming cannot"), detail)

        let ground = NimbleEngine.DynamicsAvailability.groundPlaneUntrusted.detail.lowercased()
        XCTAssertTrue(ground.contains("trusted floor"), ground)
        XCTAssertTrue(ground.contains("validated foot-support mechanics"),
                      "ground trust must not read as sufficient: \(ground)")

        // A per-frame reason remains useful, but it cannot be the only thing
        // shown for a bundled model: holding still or filming longer will not
        // cure the independent capability boundary.
        let alongsideMotion = NimbleEngine.DynamicsAvailability
            .permanentContactSupportNotice(
                isModelLoaded: true,
                hasValidatedFootContactSupport: false,
                current: .withheld(.movingBeyondStaticBudget))
        XCTAssertEqual(alongsideMotion, .contactSupportUnavailable)
        XCTAssertNil(NimbleEngine.DynamicsAvailability.permanentContactSupportNotice(
            isModelLoaded: true,
            hasValidatedFootContactSupport: false,
            current: .contactSupportUnavailable),
            "the permanent refusal should not be printed twice")
        XCTAssertNil(NimbleEngine.DynamicsAvailability.permanentContactSupportNotice(
            isModelLoaded: true,
            hasValidatedFootContactSupport: true,
            current: .withheld(.movingBeyondStaticBudget)),
            "a future capability-valid model must not inherit this warning")

        // The detailed live IK panel must obey the same two-part publication
        // gate as the main badges. A stale ID cannot override an unavailable
        // generation, and `.available` without its ID must fail closed rather
        // than presenting an empty-but-successful dynamics state.
        let staleID = NimbleEngine.IDOutput(
            jointTorques: ["knee_angle_r": 42], timestamp: 0)
        let stalePanel = IKReadoutPanel(
            ikResult: NimbleEngine.IKOutput(
                jointAngles: ["knee_angle_r": 0], markerRMSMeters: 0,
                ikLossSquaredMeters: 0, timestamp: 0),
            idResult: staleID,
            dynamicsAvailability: .contactSupportUnavailable,
            hasValidatedFootContactSupport: false)
        XCTAssertFalse(stalePanel.hasValidatedDynamicsPayload)
        XCTAssertEqual(stalePanel.displayedDynamicsAvailability,
                       .contactSupportUnavailable)

        let missingPanel = IKReadoutPanel(
            ikResult: stalePanel.ikResult,
            idResult: nil,
            dynamicsAvailability: .available,
            hasValidatedFootContactSupport: true)
        XCTAssertFalse(missingPanel.hasValidatedDynamicsPayload)
        XCTAssertEqual(missingPanel.displayedDynamicsAvailability,
                       .inverseDynamicsFailed)

        let validPanel = IKReadoutPanel(
            ikResult: stalePanel.ikResult,
            idResult: staleID,
            dynamicsAvailability: .available,
            hasValidatedFootContactSupport: true)
        XCTAssertTrue(validPanel.hasValidatedDynamicsPayload)
        XCTAssertEqual(validPanel.displayedDynamicsAvailability, .available)

        let misdatedID = NimbleEngine.IDOutput(
            jointTorques: ["knee_angle_r": 42], timestamp: 0.002)
        let misdatedPanel = IKReadoutPanel(
            ikResult: stalePanel.ikResult,
            idResult: misdatedID,
            dynamicsAvailability: .available,
            hasValidatedFootContactSupport: true)
        XCTAssertFalse(misdatedPanel.hasValidatedDynamicsPayload,
                       "a non-nil ID from another centred solve must not accompany this IK")
        XCTAssertEqual(misdatedPanel.displayedDynamicsAvailability,
                       .inverseDynamicsFailed)
        XCTAssertTrue(NimbleEngine.inverseDynamicsPayloadIsSameGeneration(
            ikResult: stalePanel.ikResult,
            idResult: staleID))
        XCTAssertFalse(NimbleEngine.inverseDynamicsPayloadIsSameGeneration(
            ikResult: stalePanel.ikResult,
            idResult: misdatedID))
        XCTAssertFalse(NimbleEngine.inverseDynamicsPayloadIsSameGeneration(
            ikResult: stalePanel.ikResult,
            idResult: nil))
        XCTAssertFalse(NimbleEngine.inverseDynamicsPayloadIsSameGeneration(
            ikResult: stalePanel.ikResult,
            idResult: NimbleEngine.IDOutput(jointTorques: [:], timestamp: .infinity)))

        let unsupportedPanel = IKReadoutPanel(
            ikResult: stalePanel.ikResult,
            idResult: staleID,
            dynamicsAvailability: .available,
            hasValidatedFootContactSupport: false)
        XCTAssertFalse(unsupportedPanel.hasValidatedDynamicsPayload)
        XCTAssertEqual(unsupportedPanel.displayedDynamicsAvailability,
                       .contactSupportUnavailable)

        XCTAssertFalse(NimbleEngine.recordedInverseDynamicsIsPublishable(
            hasValidatedFootContactSupport: false, rowCount: 10),
            "stale recorded torque rows cannot outrank the current model capability")
        XCTAssertFalse(NimbleEngine.recordedInverseDynamicsIsPublishable(
            hasValidatedFootContactSupport: true, rowCount: 0))
        XCTAssertTrue(NimbleEngine.recordedInverseDynamicsIsPublishable(
            hasValidatedFootContactSupport: true, rowCount: 10))

        let repositoryRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()
        let engineSource = try String(
            contentsOf: repositoryRoot.appendingPathComponent(
                "BioMotion/Nimble/NimbleEngine.swift"),
            encoding: .utf8)
        let contentSource = try String(
            contentsOf: repositoryRoot.appendingPathComponent(
                "BioMotion/App/ContentView.swift"),
            encoding: .utf8)
        let liveGateStart = try XCTUnwrap(
            contentSource.range(of: "    private var liveDynamicsAvailability"))
        let liveGateEnd = try XCTUnwrap(contentSource.range(
            of: "    private var liveHasValidatedDynamicsPayload",
            range: liveGateStart.upperBound..<contentSource.endIndex))
        let liveGate = String(contentSource[liveGateStart.lowerBound..<liveGateEnd.lowerBound])
        XCTAssertTrue(liveGate.contains("inverseDynamicsPayloadIsSameGeneration"),
                      "the main live badges must compare IK/ID provenance, not only ID presence")
        XCTAssertTrue(liveGate.contains("ikResult: nimble.lastIKResult"))
        XCTAssertTrue(liveGate.contains("idResult: nimble.lastIDResult"))

        let loadStart = try XCTUnwrap(engineSource.range(of: "    func loadBundledModel()"))
        let loadEnd = try XCTUnwrap(engineSource.range(
            of: "    func scaleModel(", range: loadStart.upperBound..<engineSource.endIndex))
        let loadBody = String(engineSource[loadStart.lowerBound..<loadEnd.lowerBound])
        XCTAssertTrue(loadBody.contains("self.ikHistory.removeAll(keepingCapacity: false)"))
        XCTAssertTrue(loadBody.contains("self.idHistory.removeAll(keepingCapacity: false)"),
                      "a successful model replacement must erase older torque rows")

        let exportStart = try XCTUnwrap(engineSource.range(of: "    func exportSTO("))
        let exportBody = String(engineSource[exportStart.lowerBound...])
        XCTAssertTrue(exportBody.contains("guard hasPublishableIDHistory else"),
                      "STO export must re-check the current model capability")
    }

    // MARK: - The muscle block's two causes

    /// **A data-gate failure must not answer "why are there no muscle rows?".**
    ///
    /// `loadBlock` was an `if/else` on `withheldReason`, so on a clip whose data
    /// gate failed the user saw only "…film a steadier, straighter run" under a
    /// header reading "Muscle by muscle: not shown, and why". Re-filming clears
    /// that gate and produces no rows, because `perMuscleLeftRightClaimIsSupported`
    /// is false for every clip.
    ///
    /// The panel now prints the permanent reason on both branches. What is
    /// asserted here is the material the panel needs for that: the retirement
    /// sentence exists and does NOT depend on the data gate.
    func testThePermanentRetirementSentenceIsAvailableOnAWithheldClipToo() throws {
        let withheld = Self.summary(residual: 3.0)
        let clean = Self.summary(residual: 0.1)
        XCTAssertFalse(withheld.arePublishable)
        XCTAssertTrue(clean.arePublishable)
        XCTAssertNotNil(withheld.withheldReason)
        XCTAssertNil(clean.withheldReason)

        // The sentence is the same on both, because the reason is the same on
        // both: it is a property of the model, not of the clip.
        XCTAssertEqual(withheld.perMuscleRetirementSentence,
                       clean.perMuscleRetirementSentence)
        XCTAssertFalse(withheld.perMuscleRetirementSentence.isEmpty)

        // ⚠️ **This paragraph has gone stale TWICE, and each time the assertion
        // under it was the thing holding the false version green.**
        //
        // Round 1: it read `contains("wraps around bone")` while the wrapping
        // commits had taken `MomentArmComputer` to 76 solved / 0 unmodelled and
        // the 9.92 pp rig to 1.4022 pp.
        // Round 2: the replacement asserted `contains("15 percentage points")`,
        // naming the solver's termination slack — and the very next commit took
        // that quantity from 14.88 pp to 4.4994e-05 pp by turning OSQP's Ruiz
        // scaling off and its polish step on.
        //
        // So the POSITIVE assertions name what is true now (the sharing step is
        // exact; the remaining cause is a few muscles at a few joint angles where
        // the leverage disagrees with the reference, and the reference disagrees
        // with itself there), and the NEGATIVES carry every refuted version, so a
        // future repair cannot leave any of them on screen.
        let sentence = withheld.perMuscleRetirementSentence
        print("UI-METRIC retirement_sentence=\(sentence)")
        XCTAssertTrue(sentence.contains("exact answer to its own calculation"),
                      "the sharing step is no longer a cause and must not be blamed: "
                      + "\(sentence)")
        XCTAssertTrue(sentence.contains("a few muscles at a few joint angles"),
                      "it names what IS the cause — a tail, not the typical muscle: "
                      + "\(sentence)")
        XCTAssertTrue(sentence.contains("disagrees with itself"),
                      "and says the reference is in dispute there, which is why the app "
                      + "cannot pick the good rows out: \(sentence)")
        XCTAssertFalse(sentence.contains("straight line"),
                       "the straight-line path defect was fixed on 2026-08-08; a sentence "
                       + "citing it as current is false on the most-read surface: \(sentence)")
        XCTAssertFalse(sentence.contains("66"),
                       "no muscle has an unmodelled path: \(sentence)")
        XCTAssertFalse(sentence.contains("10 percentage points"),
                       "the 9.92 pp rig reads 1.4022 pp at this build's residual: \(sentence)")
        XCTAssertFalse(sentence.contains("close enough"),
                       "the solver's own slack is 4.4994e-05 pp since `scaling = 0` and "
                       + "`polishing = 1`; blaming it is the SECOND stale version of this "
                       + "paragraph: \(sentence)")
        XCTAssertFalse(sentence.contains("15 percentage points"),
                       "same defect, same round: \(sentence)")
    }

    /// The sentence that stops the refusal selling a re-shoot. Present exactly
    /// while no per-muscle claim is supported, absent the moment one is — so it
    /// cannot outlive its own truth.
    func testTheRefilmingScopeSentenceTracksWhetherAMuscleClaimIsPossible() {
        let s = Self.summary(residual: 3.0)
        let scope = s.muscleRowsUnaffectedByRefilmingSentence
        if GaitLoadSummary.perMuscleLeftRightClaimIsSupported {
            XCTAssertNil(scope, "with a supported claim the levers deliver and this must vanish")
            return
        }
        let text = try? XCTUnwrap(scope)
        print("UI-METRIC refilming_scope=\(text ?? "-")")
        XCTAssertNotNil(text)
        XCTAssertTrue(text!.lowercased().contains("filming again"), text!)
        XCTAssertTrue(text!.contains("would not produce the muscle comparison"), text!)
        XCTAssertTrue(text!.contains("the model's, not this clip's"),
                      "it says WHOSE limit it is: \(text!)")
    }

    // MARK: - The honesty block's arithmetic

    /// **The denominator could not contain its numerators, and this is the
    /// construction that proves it.**
    ///
    /// The honesty block printed "N muscle(s) reached full effort and M sat on
    /// the resting-tone floor, out of S pairs the solver kept between the two",
    /// where `screenedComparisonCount` is built with
    /// `guard !saturatedBases.contains(base), !flooredBases.contains(base)` —
    /// disjoint from both by construction. On a clip that floors most muscles,
    /// M exceeds S several-fold and the sentence reads as broken arithmetic.
    ///
    /// Both counts are also solver stopping points rather than facts about a
    /// body (STATUS: 19, 11, 22, 18, 20 across a λ sweep at FIXED inputs), so
    /// the sentence is gone. The counts remain as data; this pins why nothing
    /// may print them as a fraction of the screened family again.
    func testTheFlooredCountCanExceedTheScreenedCountSoTheyAreNotAFraction() throws {
        let report = try Self.usableReport()
        // Three muscles pinned at the activation floor on both sides, one live.
        let floored = ["soleus", "glmax1", "recfem"]
        var left: [String: Double] = ["gasmed_l": 0.55]
        var right: [String: Double] = ["gasmed_r": 0.50]
        for base in floored {
            left["\(base)_l"] = MuscleSolver().minActivation
            right["\(base)_r"] = MuscleSolver().minActivation
        }
        var frames: [OfflineResultStore.FrameResult] = []
        for i in 0..<4 {
            frames.append(Self.gaitFrame(id: i, side: -1, activations: left, contact: i))
        }
        for i in 0..<4 {
            frames.append(Self.gaitFrame(id: 10 + i, side: 1, activations: right, contact: 10 + i))
        }
        let s = try XCTUnwrap(GaitLoadSummary.make(frames: frames, report: report,
                                                   filterTaps: 5))
        print("GAIT-METRIC honesty_block_arithmetic floored=\(s.flooredMuscleCount) "
              + "saturated=\(s.saturatedMuscleCount) screened=\(s.screenedComparisonCount) "
              + "pairs=\(s.muscles.count)")

        XCTAssertEqual(s.flooredMuscleCount, floored.count)
        XCTAssertGreaterThan(s.flooredMuscleCount, s.screenedComparisonCount,
                             "\"\(s.flooredMuscleCount) sat on the floor, out of "
                             + "\(s.screenedComparisonCount) pairs\" is not arithmetic")
        // And the screened family really is disjoint from the floored set, which
        // is the mechanism rather than a coincidence of this fixture.
        for base in floored {
            let load = try XCTUnwrap(s.muscles.first { $0.id == base })
            XCTAssertTrue(load.isAtActivationFloor, base)
            XCTAssertFalse(s.clearsStatisticalFloor(load), "\(base) is not in the screened family")
        }
    }

    // MARK: - Fixtures

    private static func usableReport() throws -> GaitReport {
        try GaitAnalysis.analyse(
            frames: GaitClipFixture.load("video_012",
                                         bundle: Bundle(for: ClaimSurfaceTests.self)).frames)
    }

    private static func summary(residual: Double) -> GaitLoadSummary {
        GaitLoadSummary(
            muscles: [
                .init(id: "glmax1", displayName: "Glute max (upper)",
                      leftLoad: 0.9, rightLoad: 0.3, leftContacts: 5, rightContacts: 5,
                      isSaturated: false, isAtActivationFloor: false,
                      samplingUncertaintyPercent: 0, pathIsModelled: false),
            ],
            resolvableAsymmetryPercent: 10,
            quantisationFloorPercent: 10,
            strideRepeatabilityPercent: 7,
            measuredStrideRepeatabilityPercent: 7,
            strideRepeatabilityBoundPercent: 5.56,
            contactClaimFloorPercent: 10,
            contactSamplingUncertaintyPercent: 0,
            peakForceIsSharedBetweenLegs: true,
            contactTimeContributionPercent: 0,
            framesPerContact: 5,
            framesPerSecond: 30,
            stanceFrameCount: 10,
            claimedStanceFrameCount: 10,
            screenedComparisonCount: 1,
            saturatedMuscleCount: 0,
            flooredMuscleCount: 0,
            maxVerticalForceResidualInBodyWeights: residual,
            medianVerticalForceResidualInBodyWeights: residual,
            residualFrameCount: 10,
            residualGatePassed: residual <= NimbleEngine.maxGaitForceResidualInBodyWeights,
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

    private static func gaitFrame(id: Int, side: Int,
                                  activations: [String: Double],
                                  contact: Int) -> OfflineResultStore.FrameResult {
        let outcome = NimbleEngine.GaitFrameOutcome(
            modelledVerticalForceInBodyWeights: 2.0,
            solvedVerticalForceInBodyWeights: 2.1,
            residualInBodyWeights: 0.1,
            contactSide: side,
            contactIndex: contact,
            solverSawLeftContact: side < 0,
            solverSawRightContact: side > 0,
            rootVerticalAccelerationMetersPerSecondSquared: 9.81,
            horizontalRootAccelerationModelled: false,
            derivativeWindowInsideContact: true)
        let muscle = NimbleEngine.MuscleOutput(activations: activations,
                                               forces: activations.mapValues { $0 * 1000 },
                                               converged: true,
                                               timestamp: Double(id) / 30.0)
        return OfflineResultStore.FrameResult(
            id: id, sourceImage: UIImage(), timestamp: Double(id) / 30.0,
            status: .success, usedFallbackBBox: false, camT: nil, modelChecksums: nil,
            bodyFrame: nil, ikResult: nil,
            idResult: NimbleEngine.IDOutput(jointTorques: [:], timestamp: Double(id) / 30.0),
            muscleResult: muscle,
            dynamicsAvailability: .available,
            isStaticHoldEstimate: false,
            motionState: .gait(verdict: .gaitStance, outcome: outcome))
    }
}
