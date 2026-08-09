import XCTest
import UIKit
@testable import BioMotion

/// The contact gate, and the one failure it could not see: a DOUBLE contact.
///
/// # Why this is its own file and its own gate
///
/// `NimbleBridge.solveIDGRF` decides contact per foot independently, and when it
/// decides BOTH feet are down it hands
/// `Skeleton::getMultipleContactInverseDynamicsNearCoP` two wrench guesses of
/// `weightUp / 2`. That solver is a least-squares fit around the guess whose
/// steps live in the constraint null space, so the 50/50 split survives: the
/// stance leg is solved with roughly HALF its real ground force, and the swing
/// leg with a ground reaction that does not exist.
///
/// Nothing else in the pipeline can see that. The residual falsifier is built
/// from `leftFootForce.y + rightFootForce.y` — the SUM — and the near-CoP
/// constraint fixes the sum exactly, so a 50/50 split and a 100/0 split give the
/// identical residual. The error is confined entirely to the RATIO between the
/// legs, which is the product's whole deliverable: a symmetric runner would be
/// published as "about 50% harder on one side", five times the ~10% resolution
/// the same screen says it can resolve.
///
/// So the gate has to ask about both feet, and this file is the proof that it
/// does — plus the measurement of how often the geometric detector would
/// actually produce a double contact on the owner's own footage.
final class GaitContactAgreementTests: XCTestCase {

    private var bundle: Bundle { Bundle(for: GaitContactAgreementTests.self) }

    // MARK: - The gate

    private func outcome(claimed side: Int, left: Bool, right: Bool,
                         cleanWindow: Bool = true) -> NimbleEngine.GaitFrameOutcome {
        NimbleEngine.GaitFrameOutcome(
            modelledVerticalForceInBodyWeights: 2.5,
            solvedVerticalForceInBodyWeights: 2.5,
            residualInBodyWeights: 0.0,
            contactSide: side,
            contactIndex: 0,
            solverSawLeftContact: left,
            solverSawRightContact: right,
            rootVerticalAccelerationMetersPerSecondSquared: 14.7,
            horizontalRootAccelerationModelled: false,
            derivativeWindowInsideContact: cleanWindow)
    }

    /// **The blocker, as one assertion.** A frame the kinematic detector calls
    /// left stance and the ID solver reads as double contact is NOT an agreeing
    /// frame, and its muscle numbers must not enter a comparison.
    func testADoubleContactIsADisagreementOnEitherClaimedSide() {
        let left = outcome(claimed: -1, left: true, right: true)
        XCTAssertFalse(left.contactDetectorsAgree,
                       "the solver put force under the swing foot too; that is not agreement")
        XCTAssertFalse(left.isUsableForLoadComparison)
        XCTAssertTrue(left.solverSawDoubleContact)

        let right = outcome(claimed: 1, left: true, right: true)
        XCTAssertFalse(right.contactDetectorsAgree)
        XCTAssertFalse(right.isUsableForLoadComparison)
        XCTAssertTrue(right.solverSawDoubleContact)
    }

    /// And the cases that were already right stay right — the fix must not
    /// widen into "refuse everything", which would pass the blocker's test and
    /// delete the feature.
    func testTheAgreeingAndAlreadyDisagreeingCasesAreUnchanged() {
        // Agreement: exactly the claimed foot, and only it.
        XCTAssertTrue(outcome(claimed: -1, left: true, right: false).contactDetectorsAgree)
        XCTAssertTrue(outcome(claimed: 1, left: false, right: true).contactDetectorsAgree)
        XCTAssertTrue(outcome(claimed: 0, left: false, right: false).contactDetectorsAgree,
                      "flight: neither detector sees a foot down")

        // Already-caught disagreements.
        XCTAssertFalse(outcome(claimed: -1, left: false, right: false).contactDetectorsAgree,
                       "solver saw no foot down at all — ID ran with zero ground force")
        XCTAssertFalse(outcome(claimed: -1, left: false, right: true).contactDetectorsAgree,
                       "solver picked the other foot")
        XCTAssertFalse(outcome(claimed: 0, left: true, right: false).contactDetectorsAgree,
                       "flight claimed, but the solver loaded a foot")
        XCTAssertFalse(outcome(claimed: 0, left: true, right: true).contactDetectorsAgree)

        // `solverSawDoubleContact` is diagnostic only and says nothing on its
        // own about usability — a genuine double contact in FLIGHT is already
        // refused by the flight branch.
        XCTAssertFalse(outcome(claimed: -1, left: true, right: false).solverSawDoubleContact)

        // The two conditions stay independent: agreement does not rescue a
        // derivative window that crossed a contact edge.
        XCTAssertFalse(outcome(claimed: -1, left: true, right: false,
                               cleanWindow: false).isUsableForLoadComparison)
    }

    // MARK: - Through the summary the panel actually reads

    /// **The failure scenario end to end.** Six left-stance frames, all of them
    /// double contact, each carrying activations halved on the left exactly as
    /// the 50/50 split would produce. Before the gate saw double contacts these
    /// were "agreeing" frames and the summary published a fabricated 50%
    /// asymmetry; now they are counted as disagreements and no load survives.
    func testFabricatedAsymmetryFromDoubleContactNeverReachesTheSummary() throws {
        let report = try Self.usableReport(bundle: bundle)

        // Truth: this runner is symmetric — both legs would solve to 0.60.
        // A 50/50 wrench split halves whichever leg is claimed as stance.
        let halved = ["glmax1_l": 0.30, "glmax1_r": 0.60]
        let honest = ["glmax1_l": 0.60, "glmax1_r": 0.60]

        let corrupted = (0..<6).map {
            Self.frame(id: $0, side: -1, activations: halved,
                       solverLeft: true, solverRight: true)
        }
        let corruptedSummary = try XCTUnwrap(
            GaitLoadSummary.make(frames: corrupted, report: report,
                                 filterTaps: 5))
        XCTAssertEqual(corruptedSummary.claimedStanceFrameCount, 6)
        XCTAssertEqual(corruptedSummary.contactDetectorDisagreements, 6,
                       "every double contact must be counted as a disagreement")
        XCTAssertEqual(corruptedSummary.stanceFrameCount, 0,
                       "and none of them may contribute a peak")
        XCTAssertTrue(corruptedSummary.muscles.isEmpty,
                      "with no usable frame there is no load to publish")
        XCTAssertFalse(corruptedSummary.contactGatePassed)
        XCTAssertFalse(corruptedSummary.arePublishable)

        // The control: the same frames with a clean single contact do publish,
        // and publish the symmetric truth. Without this the test above would
        // also pass if the gate refused everything.
        // Six alternating contacts — three per side. Two per side is the
        // minimum the contact gate accepts, because a side's step-to-step
        // scatter cannot be estimated from one sample.
        let clean = (0..<6).map { i in
            Self.frame(id: i, side: i.isMultiple(of: 2) ? -1 : 1, activations: honest,
                       solverLeft: i.isMultiple(of: 2), solverRight: !i.isMultiple(of: 2),
                       contact: i)
        }
        let cleanSummary = try XCTUnwrap(
            GaitLoadSummary.make(frames: clean, report: report,
                                 filterTaps: 5))
        XCTAssertEqual(cleanSummary.contactDetectorDisagreements, 0)
        XCTAssertEqual(cleanSummary.stanceFrameCount, 6)
        XCTAssertTrue(cleanSummary.arePublishable,
                      "the control has to publish, or the assertion above passes for the "
                      + "wrong reason: \(cleanSummary.withheldReason ?? "-")")
        let load = try XCTUnwrap(cleanSummary.muscles.first)
        XCTAssertEqual(load.leftLoad, 0.60, accuracy: 1e-12)
        XCTAssertEqual(load.rightLoad, 0.60, accuracy: 1e-12)
        XCTAssertEqual(load.differencePercent, 0, accuracy: 1e-9,
                       "a symmetric runner reads symmetric once the double contacts are gone")

        // And the size of what was suppressed, stated rather than implied.
        // `differencePercent` is signed and normalised by the MEAN of the two
        // peaks, so halving one leg of a symmetric runner reads −66.7%, not
        // −50%: the artefact is larger than the naive halving suggests.
        let fabricated = GaitLoadSummary.MuscleLoad(
            id: "glmax1", displayName: "Glute max (upper)",
            leftLoad: 0.30, rightLoad: 0.60, leftContacts: 6, rightContacts: 6,
            isSaturated: false, isAtActivationFloor: false,
            samplingUncertaintyPercent: 0, pathIsModelled: false)
        print("GAIT-METRIC double_contact_fabricated_asymmetry_percent="
              + "\(fabricated.differencePercent) "
              + "resolution_percent=\(cleanSummary.resolvableAsymmetryPercent)")
        XCTAssertGreaterThan(abs(fabricated.differencePercent),
                             cleanSummary.resolvableAsymmetryPercent,
                             "the artefact is far above what this clip claims to resolve, "
                             + "which is why it had to be gated rather than tolerated")
    }

    // MARK: - Does it actually happen? The measurement

    /// **The threshold question, measured rather than assumed.**
    ///
    /// The reviewer's hypothesis was that `calcn_y − groundHeightY < 0.06 m` is
    /// too loose on a pelvis-pinned stream, because the pelvis's own bounce is
    /// of the same order as the threshold, so the swing foot would routinely
    /// fall inside it. This drives the SHIPPED estimator
    /// (`NimbleBridge.observeLowestFootHeightY:` → `groundHeightY`) and the
    /// SHIPPED threshold (`NimbleBridge.contactDetectionThresholdMeters`) over
    /// all three pinned clips and counts what comes out.
    ///
    /// ⚠️ **Proxy, stated.** The fixture carries five joints, which cannot drive
    /// IK, so `calcn` — the heel body origin — is not available. The ankle joint
    /// centre stands in for it. The substitution adds roughly the same constant
    /// offset to both feet AND to the ground estimate (which is a percentile of
    /// the same signal), so the clearance being thresholded is preserved to
    /// first order; it is not exact, and the toe-inclusive variant is reported
    /// alongside so the conclusion can be seen not to depend on the choice.
    ///
    /// # Result, measured 2026-08-08 and pinned below
    ///
    /// | clip | ankle proxy | min(ankle, toe) proxy | min swing clearance |
    /// |---|---|---|---|
    /// | video_012 | 0 double / 58 single / 64 none | **1** / 59 / 62 | 0.106 m / 0.193 m |
    /// | video_013 | 0 / 56 / 63 | 0 / 48 / 71 | 0.150 m / 0.094 m |
    /// | video_015 | 0 / 67 / 55 | 0 / 86 / 36 | 0.326 m / 0.125 m |
    ///
    /// Two conclusions, and they point in different directions.
    ///
    /// **The gate is load-bearing, not theoretical.** One frame of `video_012`
    /// does produce a double contact — under the more permissive of the two
    /// proxies, but it produces one. The earlier read that this was a logic gap
    /// with no measured occurrence was taken under the ankle proxy alone.
    ///
    /// **The 6 cm threshold is left where it is.** The swing foot's clearance on
    /// the frames that ARE single stance is 0.094-0.326 m, i.e. 1.6-5.4× the
    /// threshold, so there is no margin problem to fix; and tightening it would
    /// cost real contacts (the `single > 0` assertion is what that would break
    /// first). One frame in 122 is the gate's job, not the threshold's. Moving a
    /// constant on this evidence would be tuning it.
    func testHowOftenTheGeometricDetectorWouldSeeBothFeetDown() throws {
        let threshold = NimbleBridge.contactDetectionThresholdMeters
        XCTAssertEqual(threshold, 0.06, accuracy: 1e-12,
                       "the measurement below describes THIS threshold")

        // Pinned per clip and proxy, so a change in the fixture, the estimator
        // or the threshold shows up as a number rather than as silence.
        let expectedDoubles: [String: Int] = [
            "video_012/ankle": 0, "video_012/min(ankle,toe)": 1,
            "video_013/ankle": 0, "video_013/min(ankle,toe)": 0,
            "video_015/ankle": 0, "video_015/min(ankle,toe)": 0,
        ]
        var measuredDoubles: [String: Int] = [:]
        for id in GaitClipFixture.allIds {
            let clip = try GaitClipFixture.load(id, bundle: bundle)

            for useToes in [false, true] {
                // A fresh bridge per pass: the estimator is a rolling window and
                // must see this clip's samples only.
                let bridge = NimbleBridge()
                var double = 0, single = 0, none = 0
                var minSwingClearance = Double.infinity
                for frame in clip.frames {
                    guard let l = Self.footHeight(frame, side: "left", includingToes: useToes),
                          let r = Self.footHeight(frame, side: "right", includingToes: useToes)
                    else { continue }
                    bridge.observeLowestFootHeightY(min(l, r))
                    let ground = bridge.groundHeightY
                    let lDown = (l - ground) < threshold
                    let rDown = (r - ground) < threshold
                    switch (lDown, rDown) {
                    case (true, true): double += 1
                    case (false, false): none += 1
                    default:
                        single += 1
                        // How much margin the SWING foot had. This is the
                        // quantity that would have to shrink for a double
                        // contact to appear.
                        minSwingClearance = Swift.min(minSwingClearance,
                                                      (lDown ? r : l) - ground)
                    }
                }
                let key = "\(id)/\(useToes ? "min(ankle,toe)" : "ankle")"
                measuredDoubles[key] = double
                print("GAIT-METRIC geometric_contact clip=\(id) "
                      + "proxy=\(useToes ? "min(ankle,toe)" : "ankle") "
                      + "double=\(double) single=\(single) none=\(none) "
                      + "min_swing_clearance_m=\(minSwingClearance) "
                      + "threshold_m=\(threshold)")
                XCTAssertGreaterThan(single, 0,
                                     "\(key): the detector must find SOME single stance, "
                                     + "or the proxy is broken rather than the threshold")
                // The clearance the swing foot actually has on the frames the
                // detector calls single stance. This is the number that would
                // have to collapse toward 0.06 m for the threshold — rather than
                // the gate — to be the thing at fault.
                XCTAssertGreaterThan(minSwingClearance, threshold,
                                     "\(key): a single-stance frame whose swing foot is inside "
                                     + "the threshold is a contradiction in the measurement")
            }
        }

        XCTAssertEqual(measuredDoubles, expectedDoubles,
                       "pinned measurement: the geometric detector doubles up on exactly one "
                       + "frame of video_012 under the toe-inclusive proxy and nowhere else")
    }

    // MARK: - Fixtures

    private static func footHeight(_ frame: BodyFrame, side: String,
                                   includingToes: Bool) -> Double? {
        guard let ankle = frame.joints.first(where: { $0.id == "\(side)_foot_joint" })
        else { return nil }
        let a = Double(ankle.worldPosition.y)
        guard includingToes,
              let toe = frame.joints.first(where: { $0.id == "\(side)_toes_joint" })
        else { return a }
        return Swift.min(a, Double(toe.worldPosition.y))
    }

    private static func usableReport(bundle: Bundle) throws -> GaitReport {
        let frames = try GaitClipFixture.load("video_012", bundle: bundle).frames
        let report = try GaitAnalysis.analyse(frames: frames)
        XCTAssertTrue(report.isUsable)
        return report
    }

    /// - Parameter contact: which foot-strike this frame belongs to. Frames
    ///   sharing a value are ONE contact, whatever order they arrive in — the
    ///   plan's own index, not adjacency. Defaults to one contact per side.
    private static func frame(id: Int, side: Int,
                              activations: [String: Double],
                              solverLeft: Bool,
                              solverRight: Bool,
                              contact: Int? = nil) -> OfflineResultStore.FrameResult {
        let outcome = NimbleEngine.GaitFrameOutcome(
            modelledVerticalForceInBodyWeights: 2.5,
            solvedVerticalForceInBodyWeights: 2.5,
            residualInBodyWeights: 0.0,
            contactSide: side,
            contactIndex: contact ?? (side < 0 ? 0 : 1),
            solverSawLeftContact: solverLeft,
            solverSawRightContact: solverRight,
            rootVerticalAccelerationMetersPerSecondSquared: 14.7,
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
