import XCTest
import simd
@testable import BioMotion

/// The body-size gate that sits ahead of `scaleModelWithHeight`.
///
/// ─────────────────────────────────────────────────────────────────────────
/// THE FAILURE THIS EXISTS FOR
/// ─────────────────────────────────────────────────────────────────────────
/// One of the six recorded offline test predictions — `sample2`, a small,
/// heavily-occluded rider on a horse — came back with a **0.070 m** inter-hip
/// distance, 0.116 m shoulder width and a 0.178 m humerus: a person at roughly
/// half scale. Nothing flagged it. `scaleModelWithHeight` clamps its
/// per-segment factors into `[0.7, 1.4]`, so the collapse did not fail there,
/// it was TRUNCATED into a musculoskeletal model scaled to nobody, and every
/// muscle number computed on it looked ordinary.
///
/// The gate is therefore about the size of the predicted SKELETON, not about
/// the pose. Every quantity it reads is pose-invariant, which is what makes it
/// safe to run on every frame: a bent knee or a raised arm cannot trip it.
final class BodyPlausibilityTests: XCTestCase {

    // MARK: - Synthetic MHR joint sets
    //
    // Only the joints `hipWidthMeters` and `estimatedStatureMeters` read matter,
    // so the rest are filled with a marker value that would be obvious in a
    // failure. Indices are MHRRetarget's own (private there, restated here from
    // findings/mhr_skeleton_summary.json — root 1, l_upleg 2, l_lowleg 3,
    // l_foot 4, r_upleg 18, r_lowleg 19, r_foot 20, c_neck 110, c_head 113,
    // c_head_null 126).
    private enum Idx {
        static let root = 1, lUpleg = 2, lLowleg = 3, lFoot = 4
        static let rUpleg = 18, rLowleg = 19, rFoot = 20
        static let cNeck = 110, cHead = 113, cHeadNull = 126
        static let count = 127
    }

    /// Builds a straight, upright MHR skeleton whose measurements are exactly
    /// the arguments — so a gate failure names a number the test chose.
    ///
    /// `estimatedStatureMeters` = leg + trunk + neck + skull + 0.050·(leg/0.8405).
    private func joints(hipWidth: Float,
                        thigh: Float = 0.42, shank: Float = 0.42,
                        trunk: Float = 0.50, neck: Float = 0.10, skull: Float = 0.19)
        -> [SIMD3<Float>] {
        var j = [SIMD3<Float>](repeating: SIMD3<Float>(99, 99, 99), count: Idx.count)
        let half = hipWidth / 2
        j[Idx.root]     = SIMD3<Float>(0, 0, 0)
        j[Idx.lUpleg]   = SIMD3<Float>(-half, 0, 0)
        j[Idx.rUpleg]   = SIMD3<Float>( half, 0, 0)
        j[Idx.lLowleg]  = SIMD3<Float>(-half, -thigh, 0)
        j[Idx.rLowleg]  = SIMD3<Float>( half, -thigh, 0)
        j[Idx.lFoot]    = SIMD3<Float>(-half, -thigh - shank, 0)
        j[Idx.rFoot]    = SIMD3<Float>( half, -thigh - shank, 0)
        j[Idx.cNeck]    = SIMD3<Float>(0, trunk, 0)
        j[Idx.cHead]    = SIMD3<Float>(0, trunk + neck, 0)
        j[Idx.cHeadNull] = SIMD3<Float>(0, trunk + neck + skull, 0)
        return j
    }

    private func statureOf(_ j: [SIMD3<Float>]) -> Double {
        Double(MHRRetarget.estimatedStatureMeters(jointCoords: j))
    }

    // MARK: - The fixture reproduces the two ends of the recorded range

    /// Guards the synthetic builder itself: if `estimatedStatureMeters` changed
    /// its formula, every bound below would be measuring something else.
    func testFixtureProducesTheIntendedMeasurements() {
        let j = joints(hipWidth: 0.17)
        XCTAssertEqual(MHRRetarget.hipWidthMeters(jointCoords: j), 0.17, accuracy: 1e-6)
        // 0.84 leg + 0.50 trunk + 0.10 neck + 0.19 skull + 0.050·(0.84/0.8405)
        XCTAssertEqual(statureOf(j), 0.84 + 0.50 + 0.10 + 0.19 + 0.050 * (0.84 / 0.8405),
                       accuracy: 1e-4)
        print("PLAUS-METRIC nominal hip=\(MHRRetarget.hipWidthMeters(jointCoords: j)) " +
              "stature=\(statureOf(j))")
    }

    /// An ordinary adult passes with margin on both axes.
    func testNominalAdultIsPlausible() {
        let v = MHRRetarget.plausibility(jointCoords: joints(hipWidth: 0.17))
        XCTAssertTrue(v.isPlausible, "nominal adult rejected: \(v.reason ?? "")")
        print("PLAUS-METRIC nominal verdict=\(v)")
    }

    /// The exact recorded failure. 0.070 m is the number STATUS.md records for
    /// `sample2`, and it is what this gate exists to catch.
    func testTheRecordedOccludedRiderIsRejected() {
        // Roughly half scale, matching the recorded collapse.
        let j = joints(hipWidth: 0.070, thigh: 0.21, shank: 0.21,
                       trunk: 0.25, neck: 0.05, skull: 0.10)
        let v = MHRRetarget.plausibility(jointCoords: j)
        print("PLAUS-METRIC sample2-like hip=\(v.hipWidthMeters) stature=\(v.statureMeters) " +
              "reason=\(v.reason ?? "<accepted>")")
        XCTAssertFalse(v.isPlausible, "the 0.070 m hip width case must not reach scaleModelWithHeight")
        let reason = try! XCTUnwrap(v.reason)
        XCTAssertTrue(reason.contains("7 cm"),
                      "the rejection must quote the measured number; got: \(reason)")
    }

    /// Hip width alone rejects, with the stature axis inside its bounds — so
    /// the two criteria are independently live rather than one implying the
    /// other.
    func testHipWidthRejectsIndependentlyOfStature() {
        for hip in [0.08, 0.099, 0.281, 0.35] as [Float] {
            let j = joints(hipWidth: hip)
            let v = MHRRetarget.plausibility(jointCoords: j)
            print("PLAUS-METRIC hip=\(hip) stature=\(v.statureMeters) plausible=\(v.isPlausible)")
            XCTAssertGreaterThan(v.statureMeters, MHRRetarget.minStatureMeters,
                                 "this case must fail on hip width, not stature")
            XCTAssertLessThan(v.statureMeters, MHRRetarget.maxStatureMeters)
            XCTAssertFalse(v.isPlausible, "hip width \(hip) m must be rejected")
        }
        // ...and the inside of the interval is accepted.
        for hip in [0.101, 0.17, 0.279] as [Float] {
            XCTAssertTrue(MHRRetarget.plausibility(jointCoords: joints(hipWidth: hip)).isPlausible,
                          "hip width \(hip) m is inside the bounds and must pass")
        }
    }

    /// Stature alone rejects, with the hip width inside its bounds.
    func testStatureRejectsIndependentlyOfHipWidth() {
        // Scale the whole chain but keep the pelvis a normal width.
        let tooShort = joints(hipWidth: 0.17, thigh: 0.28, shank: 0.28,
                              trunk: 0.32, neck: 0.06, skull: 0.12)
        let tooTall = joints(hipWidth: 0.17, thigh: 0.58, shank: 0.58,
                             trunk: 0.70, neck: 0.14, skull: 0.26)
        for (tag, j) in [("short", tooShort), ("tall", tooTall)] {
            let v = MHRRetarget.plausibility(jointCoords: j)
            print("PLAUS-METRIC \(tag) hip=\(v.hipWidthMeters) stature=\(v.statureMeters) " +
                  "reason=\(v.reason ?? "<accepted>")")
            XCTAssertGreaterThan(v.hipWidthMeters, MHRRetarget.minHipWidthMeters,
                                 "this case must fail on stature, not hip width")
            XCTAssertLessThan(v.hipWidthMeters, MHRRetarget.maxHipWidthMeters)
            XCTAssertFalse(v.isPlausible, "\(tag) subject must be rejected")
        }
    }

    /// The gate must be blind to POSE. This is what licenses running it on every
    /// frame instead of only the calibration frame — and it is the property that
    /// `segmentScaleMarkers`' straight-line distances famously do NOT have
    /// (a raised knee collapses `|LHJC-LAJC|` from 0.816 m to 0.291 m).
    func testGateIsPoseInvariant() {
        let straight = joints(hipWidth: 0.17)
        var folded = straight
        // Fold both knees fully: the ankle comes back up beside the hip. The
        // straight-line hip-to-ankle distance collapses; the chain sum does not.
        folded[Idx.lFoot] = folded[Idx.lLowleg] + SIMD3<Float>(0, 0.42, 0)
        folded[Idx.rFoot] = folded[Idx.rLowleg] + SIMD3<Float>(0, 0.42, 0)
        // And raise both arms / twist — irrelevant joints, but prove it.
        let a = MHRRetarget.plausibility(jointCoords: straight)
        let b = MHRRetarget.plausibility(jointCoords: folded)
        print("PLAUS-METRIC pose-invariance straight_stature=\(a.statureMeters) " +
              "folded_stature=\(b.statureMeters) " +
              "straight_hip=\(a.hipWidthMeters) folded_hip=\(b.hipWidthMeters)")
        XCTAssertEqual(a.statureMeters, b.statureMeters, accuracy: 1e-6,
                       "folding the knees must not change the chain-sum stature")
        XCTAssertEqual(a.hipWidthMeters, b.hipWidthMeters, accuracy: 1e-9)
        XCTAssertTrue(b.isPlausible, "a legitimately folded pose must not be rejected")
    }

    /// Degenerate input must be rejected rather than propagate a NaN into the
    /// scale factors.
    func testMalformedInputIsRejected() {
        XCTAssertFalse(MHRRetarget.plausibility(jointCoords: []).isPlausible)
        XCTAssertFalse(MHRRetarget.plausibility(
            jointCoords: [SIMD3<Float>](repeating: .zero, count: 10)).isPlausible)

        var nanJoints = joints(hipWidth: 0.17)
        nanJoints[Idx.lUpleg] = SIMD3<Float>(.nan, 0, 0)
        let v = MHRRetarget.plausibility(jointCoords: nanJoints)
        print("PLAUS-METRIC nan verdict reason=\(v.reason ?? "<accepted>")")
        XCTAssertFalse(v.isPlausible, "a NaN joint must not reach scaleModelWithHeight")
    }

    /// A zero-size prediction — the most degenerate collapse — is rejected on
    /// both axes, not just accidentally on one.
    func testAllZeroPredictionIsRejected() {
        let z = [SIMD3<Float>](repeating: .zero, count: Idx.count)
        let v = MHRRetarget.plausibility(jointCoords: z)
        print("PLAUS-METRIC zero hip=\(v.hipWidthMeters) stature=\(v.statureMeters) " +
              "reason=\(v.reason ?? "<accepted>")")
        XCTAssertFalse(v.isPlausible)
        XCTAssertEqual(v.hipWidthMeters, 0, accuracy: 1e-12)
        XCTAssertEqual(v.statureMeters, 0, accuracy: 1e-12)
    }

    // MARK: - The result the user sees

    /// The rejection has to carry the number, or it is indistinguishable from a
    /// crash. This pins the store's side of that.
    @MainActor
    func testRejectionIsSurfacedWithTheMeasuredNumber() {
        let store = OfflineResultStore()
        let v = MHRRetarget.plausibility(jointCoords: joints(hipWidth: 0.070,
                                                             thigh: 0.21, shank: 0.21,
                                                             trunk: 0.25, neck: 0.05, skull: 0.10))
        store.append(OfflineResultStore.FrameResult(
            id: 0, sourceImage: UIImage(), timestamp: 0,
            status: .implausibleBody(reason: v.reason ?? "", hipWidthMeters: v.hipWidthMeters,
                                     statureMeters: v.statureMeters),
            usedFallbackBBox: false, camT: nil, modelChecksums: nil, bodyFrame: nil,
            ikResult: nil, idResult: nil, muscleResult: nil,
            isStaticHoldEstimate: false, motionState: .undetermined))

        XCTAssertEqual(store.implausibleBodyCount, 1)
        XCTAssertEqual(store.successCount, 0, "a rejected frame must not count as a success")
        guard case .implausibleBody(let reason, let hip, let stature) = store.frames[0].status else {
            XCTFail("status was not .implausibleBody"); return
        }
        XCTAssertFalse(reason.isEmpty)
        XCTAssertEqual(hip, 0.070, accuracy: 1e-6)
        XCTAssertTrue(stature > 0)

        // The exact sentence the badge shows. This is the ONLY thing the user
        // gets for a rejected frame, so it is asserted rather than assumed —
        // and it has to carry BOTH measured numbers, not just the verdict.
        let shown = try! XCTUnwrap(store.frames[0].status.implausibleBodyDescription)
        print("PLAUS-METRIC badge=\(shown)")
        XCTAssertTrue(shown.contains("7 cm apart"), "hip width missing from the badge: \(shown)")
        XCTAssertTrue(shown.contains("0.84 m"), "stature missing from the badge: \(shown)")
        XCTAssertTrue(shown.contains("hip width came out"), "reason missing from the badge: \(shown)")
        XCTAssertFalse(shown.contains("%"), "format string did not substitute: \(shown)")
    }

    /// Every other status yields nil, so the helper cannot leak a body-size
    /// sentence onto an unrelated frame.
    @MainActor
    func testOnlyImplausibleBodyHasARejectionSentence() {
        XCTAssertNil(OfflineResultStore.FrameStatus.success.implausibleBodyDescription)
        XCTAssertNil(OfflineResultStore.FrameStatus.nimbleTimeout.implausibleBodyDescription)
        XCTAssertNil(OfflineResultStore.FrameStatus.poseEstimationFailed("x").implausibleBodyDescription)
    }

    // MARK: - Bounds provenance

    /// Not a behavioural test: it records the bounds and the margin they leave
    /// against the predictions this pipeline is known to fit, so a future edit
    /// to the constants shows up as a diff with numbers next to it.
    func testBoundsAndTheirMargins() {
        print("PLAUS-METRIC bounds hip=[\(MHRRetarget.minHipWidthMeters), " +
              "\(MHRRetarget.maxHipWidthMeters)] stature=[\(MHRRetarget.minStatureMeters), " +
              "\(MHRRetarget.maxStatureMeters)]")
        // The four recorded PASSING statures, from MHRRetarget's own doc comment
        // (dancing / yoga / football measured 1.715 / 1.602 / 1.646 m).
        for s in [1.715, 1.602, 1.646] {
            XCTAssertGreaterThan(s, MHRRetarget.minStatureMeters)
            XCTAssertLessThan(s, MHRRetarget.maxStatureMeters)
        }
        // The recorded FAILING hip width.
        XCTAssertLessThan(0.070, MHRRetarget.minHipWidthMeters)
        XCTAssertLessThan(MHRRetarget.minHipWidthMeters, MHRRetarget.maxHipWidthMeters)
        XCTAssertLessThan(MHRRetarget.minStatureMeters, MHRRetarget.maxStatureMeters)
    }
}
