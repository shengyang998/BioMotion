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

    // MARK: - The LIVE screen's foot-load badge

    /// **The live screen showed `L/R load 0.62|0.38` with no caption, no floor
    /// and nothing validating it** — the product's own deliverable framing, on
    /// its most-used surface, while the offline path spent four rounds learning
    /// it cannot support that comparison.
    ///
    /// Its green/amber indicator was `abs(total - 1.0) < 0.3`: keyed to the SUM,
    /// which the near-CoP solver constrains exactly, while the VALUE printed the
    /// split, which nothing checks and which starts from a hardcoded 50/50
    /// wrench guess (`NimbleBridge.mm:1499`). The badge shows the sum now and
    /// this note says what is missing.
    func testTheFootLoadNoteStatesThatTheSplitIsNotMeasured() {
        let note = NimbleEngine.footLoadSplitIsNotMeasuredNote
        print("UI-METRIC foot_load_note=\(note)")

        // The absence, in the words a reader would look for.
        XCTAssertTrue(note.contains("NOT measured"),
                      "the absence has to be stated, not implied: \(note)")
        XCTAssertTrue(note.lowercased().contains("splits between your two feet")
                        || note.lowercased().contains("split"),
                      "and it has to name the SPLIT specifically: \(note)")
        // The mechanism, so it does not read as caution.
        XCTAssertTrue(note.contains("50/50"), "it names the prior: \(note)")
        XCTAssertTrue(note.lowercased().contains("not determined by the pose"),
                      "and the indeterminacy: \(note)")
        // And it must not itself become a claim: no per-side figure in it.
        XCTAssertFalse(note.contains("|"), note)
        XCTAssertFalse(note.contains("%"), note)
    }

    /// The sum is a consistency check, and the note says so rather than letting
    /// "GRF sum 1.00" read as "you are balanced". `rootResidualPerKg`'s own
    /// comment already makes this distinction for the frame check; this is the
    /// same distinction one badge to its left.
    func testTheFootLoadNoteRefusesTheBalanceReading() {
        let note = NimbleEngine.footLoadSplitIsNotMeasuredNote
        XCTAssertTrue(note.lowercased().contains("consistency check"), note)
        XCTAssertTrue(note.lowercased().contains("not a balance score"), note)
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
        XCTAssertTrue(withheld.perMuscleRetirementSentence.contains("wraps around bone"),
                      "it names the mechanism the user cannot change")
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
                                                   framesPerSecond: 30, filterTaps: 5))
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
            bodyFrame: nil, ikResult: nil, idResult: nil, muscleResult: muscle,
            isStaticHoldEstimate: false,
            motionState: .gait(verdict: .gaitStance, outcome: outcome))
    }
}
