import XCTest
@testable import BioMotion

/// Settles two questions and measures the third:
///
/// 1. Does nimble honour `<locked>true</locked>`?  (`testNimbleKeepsLockedCoordinatesAsDOFs`)
/// 2. What is the exact 171 -> 163 accounting?     (`testDOFCountAccounting`)
/// 3. What does runtime DOF masking cost/buy?      (`testMaskedVsUnmaskedIK`)
///
/// Every number this file prints is written to the test log with an
/// `DOFMASK-METRIC` prefix so it can be harvested without re-running.
final class DOFMaskTests: XCTestCase {

    private var bridge: NimbleBridge!

    override func setUp() {
        super.setUp()
        bridge = NimbleBridge()
    }

    // MARK: - Part 1: does nimble honour <locked>true</locked>?

    /// If nimble honoured the flag by removing the coordinate, none of the 54
    /// locked names could appear in `dofNames`. They all do.
    func testNimbleKeepsLockedCoordinatesAsDOFs() throws {
        try loadFullBody()
        let names = Set(bridge.dofNames)

        var present: [String] = []
        var absent: [String] = []
        for n in FullBodyDOFFixture.xmlLockedCoordinates {
            if names.contains(n) { present.append(n) } else { absent.append(n) }
        }

        print("DOFMASK-METRIC locked_coordinates_total=\(FullBodyDOFFixture.xmlLockedCoordinates.count)")
        print("DOFMASK-METRIC locked_coordinates_present_as_dofs=\(present.count)")
        print("DOFMASK-METRIC locked_coordinates_absent=\(absent.count) \(absent)")

        XCTAssertEqual(present.count, 54,
                       "nimble does NOT drop <locked>true</locked> coordinates; " +
                       "all 54 must still be DOFs of the built skeleton")
        XCTAssertTrue(absent.isEmpty)
    }

    /// XML coordinates vs parsed DOFs. The gap used to be 8 (2 patellofemoral +
    /// 6 shoulder); after the `tools/osim_fixes` change it is 0 and the counts
    /// are 169 == 169.
    func testDOFCountAccounting() throws {
        try loadFullBody()
        let names = Set(bridge.dofNames)

        print("DOFMASK-METRIC xml_coordinates=\(FullBodyDOFFixture.xmlCoordinateCount)")
        print("DOFMASK-METRIC nimble_dofs=\(bridge.numDOFs)")

        XCTAssertEqual(bridge.numDOFs, FullBodyDOFFixture.expectedNimbleDOFCount)

        // This list is now empty: the patellofemoral coordinates were removed
        // from the XML with the weld, and the shoulder axes were orthogonalised
        // so the WeldJoint crash-guard no longer fires. The loop is kept so the
        // invariant is re-checked if a future model reintroduces a dropped
        // coordinate.
        for n in FullBodyDOFFixture.coordinatesNimbleDrops {
            XCTAssertFalse(names.contains(n),
                           "\(n) should not exist as a DOF in the built skeleton")
        }
        XCTAssertEqual(
            FullBodyDOFFixture.xmlCoordinateCount
                - FullBodyDOFFixture.coordinatesNimbleDrops.count,
            bridge.numDOFs,
            "xmlCoordinateCount - coordinatesNimbleDrops must equal the parsed DOF count")
    }

    /// Whether a locked coordinate is actually *held* at its pinned value
    /// through a real IK solve, or merely clamped on some iterations.
    func testLockedCoordinateValuesAfterUnmaskedIK() throws {
        try loadFullBody()
        _ = solveStandingPose()
        guard let r = solveStandingPose() else {
            XCTFail("IK should solve"); return
        }
        let names = bridge.dofNames
        let locked = Set(FullBodyDOFFixture.xmlLockedCoordinates)
        var worst = 0.0
        var worstName = ""
        for (i, n) in names.enumerated() where locked.contains(n) {
            let v = abs(r.jointAngles[i].doubleValue)
            if v > worst { worst = v; worstName = n }
        }
        print("DOFMASK-METRIC unmasked_max_abs_locked_dof_value=\(worst) at=\(worstName)")
        // Not an assertion about what SHOULD happen — this records what does.
        XCTAssertTrue(worst.isFinite)
    }

    // MARK: - Part 2/3: the mask

    func testMaskComposition() throws {
        try loadFullBody()

        // The hard constraint: the sternum and the free costovertebral
        // coordinates must never enter the mask.
        let mask = Set(FullBodyDOFFixture.runtimeMask)
        for n in FullBodyDOFFixture.protectedShoulderGirdleCoordinates {
            XCTAssertFalse(mask.contains(n),
                           "\(n) is shoulder-girdle mobility and must not be masked")
        }

        let matched = bridge.applyDOFMask(withNames: FullBodyDOFFixture.runtimeMask)
        print("DOFMASK-METRIC mask_requested=\(FullBodyDOFFixture.runtimeMask.count)")
        print("DOFMASK-METRIC mask_matched=\(matched)")
        print("DOFMASK-METRIC free_dofs=\(bridge.numFreeDOFs)")

        XCTAssertEqual(matched, FullBodyDOFFixture.runtimeMask.count,
                       "every masked name must resolve to a real DOF")
        XCTAssertEqual(bridge.numFreeDOFs,
                       FullBodyDOFFixture.expectedFreeDOFCountAfterMask)
        XCTAssertEqual(bridge.numDOFs, FullBodyDOFFixture.expectedNimbleDOFCount,
                       "masking must NOT change the skeleton's DOF count")
    }

    /// Masking is reversible; welding is not. This pins that property.
    func testMaskIsReversible() throws {
        try loadFullBody()
        _ = solveStandingPose()
        guard let before = solveStandingPose() else { XCTFail(); return }

        _ = bridge.applyDOFMask(withNames: FullBodyDOFFixture.runtimeMask)
        XCTAssertTrue(bridge.isDOFMaskActive)
        _ = solveStandingPose()

        bridge.clearDOFMask()
        XCTAssertFalse(bridge.isDOFMaskActive)
        XCTAssertEqual(bridge.numFreeDOFs, FullBodyDOFFixture.expectedNimbleDOFCount)

        bridge.resetSessionState()
        _ = solveStandingPose()
        guard let after = solveStandingPose() else { XCTFail(); return }
        XCTAssertEqual(before.jointAngles.count, after.jointAngles.count)
    }

    func testMaskedDOFsStayExactlyPinned() throws {
        try loadFullBody()
        _ = bridge.applyDOFMask(withNames: FullBodyDOFFixture.runtimeMask)
        _ = solveStandingPose()
        guard let a = solveStandingPose(), let b = solveStandingPose(shiftX: 0.01) else {
            XCTFail(); return
        }
        let names = bridge.dofNames
        let masked = Set(FullBodyDOFFixture.runtimeMask)
        var worstDrift = 0.0
        for (i, n) in names.enumerated() where masked.contains(n) {
            worstDrift = max(worstDrift,
                             abs(a.jointAngles[i].doubleValue - b.jointAngles[i].doubleValue))
        }
        print("DOFMASK-METRIC masked_dof_max_drift=\(worstDrift)")
        XCTAssertLessThan(worstDrift, 1e-12,
                          "a masked DOF must not move at all between solves")
    }

    /// The headline measurement: wall clock and repeat-solve drift, masked vs
    /// unmasked, on the SAME fixture and the SAME process.
    func testMaskedVsUnmaskedIK() throws {
        try loadFullBody()

        let unmasked = measureArm(label: "unmasked", mask: nil)
        let masked = measureArm(label: "masked", mask: FullBodyDOFFixture.runtimeMask)

        print("DOFMASK-METRIC unmasked_dofs=\(unmasked.dofs)")
        print("DOFMASK-METRIC unmasked_ms_per_solve=\(unmasked.msPerSolve)")
        print("DOFMASK-METRIC unmasked_drift_max_rad=\(unmasked.driftMax)")
        print("DOFMASK-METRIC unmasked_drift_l2_rad=\(unmasked.driftL2)")
        print("DOFMASK-METRIC unmasked_final_marker_rms_m=\(unmasked.markerRMS)")
        print("DOFMASK-METRIC masked_dofs=\(masked.dofs)")
        print("DOFMASK-METRIC masked_ms_per_solve=\(masked.msPerSolve)")
        print("DOFMASK-METRIC masked_drift_max_rad=\(masked.driftMax)")
        print("DOFMASK-METRIC masked_drift_l2_rad=\(masked.driftL2)")
        print("DOFMASK-METRIC masked_final_marker_rms_m=\(masked.markerRMS)")
        print("DOFMASK-METRIC speedup=\(unmasked.msPerSolve / max(masked.msPerSolve, 1e-9))")

        XCTAssertGreaterThan(unmasked.msPerSolve, 0)
        XCTAssertGreaterThan(masked.msPerSolve, 0)
    }

    /// 54 of the 57 masked coordinates already carry `<locked>true</locked>`, so
    /// nimble's degenerate [lo, lo] position limits already pin them and masking
    /// cannot move them. The 3 `Abdjnt` coordinates are the exception: they are
    /// unlocked, and 26 abdominal muscles (external/internal oblique,
    /// transversus) span that joint. If unmasked IK parks them away from the
    /// mask's pin value, masking changes the pose the muscle stage sees.
    func testAbdjntIsTheOnlyPoseChangingMask() throws {
        try loadFullBody()
        let names = bridge.dofNames
        let masked = Set(FullBodyDOFFixture.runtimeMask)

        bridge.resetSessionState()
        _ = solveStandingPose()
        guard let unmaskedPose = solveStandingPose() else { XCTFail(); return }

        bridge.clearDOFMask()
        _ = bridge.applyDOFMask(withNames: FullBodyDOFFixture.runtimeMask)
        bridge.resetSessionState()
        _ = solveStandingPose()
        guard let maskedPose = solveStandingPose() else { XCTFail(); return }

        var worstMaskedDelta = 0.0
        var worstMaskedName = ""
        var report: [String] = []
        for (i, n) in names.enumerated() where masked.contains(n) {
            let u = unmaskedPose.jointAngles[i].doubleValue
            let m = maskedPose.jointAngles[i].doubleValue
            let d = abs(u - m)
            if d > worstMaskedDelta { worstMaskedDelta = d; worstMaskedName = n }
            if d > 1e-9 {
                report.append("\(n) unmasked=\(String(format: "%.6f", u)) " +
                              "masked=\(String(format: "%.6f", m))")
            }
        }
        print("DOFMASK-METRIC masked_coords_whose_value_changed=\(report.count)")
        for line in report { print("DOFMASK-METRIC changed \(line)") }
        print("DOFMASK-METRIC worst_masked_pose_delta=\(worstMaskedDelta) at=\(worstMaskedName)")

        // Also record how far the FREE coordinates moved — that is the part of
        // the pose the muscle stage consumes for every non-abdominal muscle.
        var worstFreeDelta = 0.0
        var worstFreeName = ""
        for (i, n) in names.enumerated() where !masked.contains(n) {
            let d = abs(unmaskedPose.jointAngles[i].doubleValue
                        - maskedPose.jointAngles[i].doubleValue)
            if d > worstFreeDelta { worstFreeDelta = d; worstFreeName = n }
        }
        print("DOFMASK-METRIC worst_free_pose_delta=\(worstFreeDelta) at=\(worstFreeName)")
        XCTAssertTrue(worstMaskedDelta.isFinite)
    }

    /// The above test pins Abdjnt at whatever pose the skeleton held when the
    /// mask was applied, so a zero delta there could be tautological. This one
    /// is not: it loads a fresh model, never masks, and simply reads what
    /// unmasked IK leaves in the three Abdjnt coordinates. If those values are
    /// the post-load rest values, then pinning them costs the 26 abdominal
    /// muscles nothing.
    func testAbdjntValuesAfterUnmaskedIKFromFreshLoad() throws {
        try loadFullBody()
        let names = bridge.dofNames
        // The Abdjnt coordinates are spelled Abs_FE / Abs_LB / Abs_AR.
        let abd = names.enumerated().filter { $0.element.hasPrefix("Abs_") }
        print("DOFMASK-METRIC abdjnt_dof_names=\(abd.map { $0.element })")

        // Pose straight after parse, before any solve.
        _ = solveStandingPose()   // cold
        guard let first = solveStandingPose(),
              let second = solveStandingPose(shiftX: 0.01) else { XCTFail(); return }

        for (i, n) in abd {
            print("DOFMASK-METRIC abdjnt \(n) afterIK=\(first.jointAngles[i].doubleValue) " +
                  "afterShiftedIK=\(second.jointAngles[i].doubleValue)")
        }
        XCTAssertFalse(abd.isEmpty, "FullBody.osim must expose Abdjnt coordinates")
    }

    // MARK: - Helpers

    private struct ArmResult {
        var dofs: Int
        var msPerSolve: Double
        var driftMax: Double
        var driftL2: Double
        var markerRMS: Double
    }

    private func measureArm(label: String, mask: [String]?) -> ArmResult {
        bridge.clearDOFMask()
        if let mask { _ = bridge.applyDOFMask(withNames: mask) }
        bridge.resetSessionState()

        // Cold solve, discarded: it is the only one that pays the random-restart
        // path, and it is not what runs per frame.
        _ = solveStandingPose()

        let reps = 12
        var results: [NimbleIKResult] = []
        let t0 = Date()
        for _ in 0..<reps {
            if let r = solveStandingPose() { results.append(r) }
        }
        let elapsed = Date().timeIntervalSince(t0)

        var driftMax = 0.0
        var driftL2 = 0.0
        if results.count >= 2 {
            let a = results[results.count - 2].jointAngles
            let b = results[results.count - 1].jointAngles
            let names = bridge.dofNames
            var sq = 0.0
            var perDOF: [(String, Double)] = []
            for i in 0..<min(a.count, b.count) {
                let d = abs(a[i].doubleValue - b[i].doubleValue)
                driftMax = max(driftMax, d)
                sq += d * d
                if d > 0 { perDOF.append((i < names.count ? names[i] : "#\(i)", d)) }
            }
            driftL2 = sq.squareRoot()

            // Where does the drift actually live? If the masked set is not in
            // this list, masking is not a drift fix — it is only a speed fix.
            let top = perDOF.sorted { $0.1 > $1.1 }.prefix(10)
            let rendered = top.map { "\($0.0)=\(String(format: "%.5f", $0.1))" }
                .joined(separator: " ")
            print("DOFMASK-METRIC \(label)_n_dofs_that_moved=\(perDOF.count)")
            print("DOFMASK-METRIC \(label)_top_drifting_dofs \(rendered)")

            let maskSet = Set(FullBodyDOFFixture.runtimeMask)
            let inMask = perDOF.filter { maskSet.contains($0.0) }
            let inMaskL2 = inMask.reduce(0.0) { $0 + $1.1 * $1.1 }.squareRoot()
            print("DOFMASK-METRIC \(label)_drift_l2_inside_maskset=\(inMaskL2)")
        }

        // `error` from nimble is the squared norm of the weighted residual
        // stack, RMS-1 normalised => per-marker RMS = sqrt(error / numMarkers).
        let numMarkers = Double(arkitStandingMarkers().names.count)
        let rms = results.last.map { ($0.error / numMarkers).squareRoot() } ?? .nan

        print("DOFMASK-METRIC \(label)_reps=\(reps) total_s=\(elapsed)")
        return ArmResult(dofs: bridge.numFreeDOFs,
                         msPerSolve: elapsed / Double(reps) * 1000.0,
                         driftMax: driftMax,
                         driftL2: driftL2,
                         markerRMS: rms)
    }

    /// Same 12-marker standing fixture as `NimbleBridgeTests`, so the drift
    /// numbers here are comparable to the ones recorded in STATUS.md — with the
    /// caveat that STATUS.md's 0.006 rad figure was measured on Rajagopal2016
    /// (39 DOF), while this file loads FullBody.osim (163 DOF).
    private func arkitStandingMarkers(shiftX: Double = 0)
        -> (positions: [NSNumber], names: [String]) {
        let layout: [(String, Double, Double, Double)] = [
            ("PELVIS",  0.00, 0.95, 0.00),
            ("LHJC",   -0.09, 0.92, 0.00),
            ("RHJC",    0.09, 0.92, 0.00),
            ("LKJC",   -0.09, 0.52, 0.00),
            ("RKJC",    0.09, 0.52, 0.00),
            ("LAJC",   -0.09, 0.10, 0.00),
            ("RAJC",    0.09, 0.10, 0.00),
            ("C7",      0.00, 1.40, 0.00),
            ("LSJC",   -0.18, 1.35, 0.00),
            ("RSJC",    0.18, 1.35, 0.00),
            ("LEJC",   -0.20, 1.07, 0.00),
            ("REJC",    0.20, 1.07, 0.00),
        ]
        var positions: [NSNumber] = []
        var names: [String] = []
        for (name, x, y, z) in layout {
            names.append(name)
            positions.append(NSNumber(value: x + shiftX))
            positions.append(NSNumber(value: y))
            positions.append(NSNumber(value: z))
        }
        return (positions, names)
    }

    private func solveStandingPose(shiftX: Double = 0) -> NimbleIKResult? {
        let m = arkitStandingMarkers(shiftX: shiftX)
        return bridge.solveIK(withMarkerPositions: m.positions, markerNames: m.names)
    }

    private func loadFullBody() throws {
        let path = Bundle(for: type(of: self)).path(forResource: "FullBody", ofType: "osim")
            ?? Bundle.main.path(forResource: "FullBody", ofType: "osim")
        guard let path else {
            throw XCTSkip("FullBody.osim is not reachable from the test bundle")
        }
        XCTAssertTrue(bridge.loadModel(fromPath: path), "FullBody.osim must load")
    }
}
