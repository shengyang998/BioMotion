import XCTest
@testable import BioMotion

final class NimbleBridgeTests: XCTestCase {

    private var bridge: NimbleBridge!

    override func setUp() {
        super.setUp()
        bridge = NimbleBridge()
    }

    // MARK: - Model Loading

    func testLoadRajagopalModel() {
        let path = Bundle(for: type(of: self)).path(forResource: "Rajagopal2016", ofType: "osim")
            ?? Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim")
        XCTAssertNotNil(path, "Rajagopal2016.osim must be in the test bundle or app bundle")

        guard let path else { return }
        let success = bridge.loadModel(fromPath: path)
        XCTAssertTrue(success, "Model should load successfully")
        XCTAssertTrue(bridge.isModelLoaded)
    }

    func testModelDOFCount() {
        loadModel()
        // Rajagopal2016 has 39 coordinates (some locked)
        XCTAssertGreaterThan(bridge.numDOFs, 20, "Should have >20 DOFs")
        XCTAssertLessThanOrEqual(bridge.numDOFs, 39, "Should have <=39 DOFs")
    }

    func testModelDOFNames() {
        loadModel()
        let names = bridge.dofNames
        XCTAssertFalse(names.isEmpty)

        // Check for known DOF names
        XCTAssertTrue(names.contains(where: { $0.contains("hip_flexion") }),
                      "Should contain hip_flexion DOF")
        XCTAssertTrue(names.contains(where: { $0.contains("knee_angle") }),
                      "Should contain knee_angle DOF")
    }

    func testModelMarkerNames() {
        loadModel()
        let markers = bridge.markerNames
        XCTAssertFalse(markers.isEmpty, "Should have markers")
        // Rajagopal2016 has 66 markers
        XCTAssertGreaterThan(markers.count, 30, "Should have >30 markers")
    }

    // MARK: - Inverse Kinematics

    func testIKWithSyntheticData() {
        loadModel()

        // Create synthetic marker positions (standing pose)
        // Use a subset of markers that exist in the model
        let modelMarkers = bridge.markerNames
        guard modelMarkers.count >= 3 else {
            XCTFail("Need at least 3 markers")
            return
        }

        // Use first 5 markers with plausible 3D positions
        let numMarkers = min(5, modelMarkers.count)
        var positions: [NSNumber] = []
        var names: [String] = []

        for i in 0..<numMarkers {
            names.append(modelMarkers[i])
            // Generic standing position (spread around origin at ~1m height)
            positions.append(NSNumber(value: Double(i) * 0.1 - 0.2))  // x
            positions.append(NSNumber(value: 1.0))                      // y (up)
            positions.append(NSNumber(value: 0.0))                      // z
        }

        let result = bridge.solveIK(withMarkerPositions: positions, markerNames: names)
        XCTAssertNotNil(result, "IK should return a result")

        if let result {
            XCTAssertEqual(result.jointAngles.count, Int(bridge.numDOFs),
                           "Should return angles for all DOFs")
            XCTAssertGreaterThanOrEqual(result.error, 0, "Error should be non-negative")
            XCTAssertEqual(result.dofNames.count, Int(bridge.numDOFs))
        }
    }

    func testIKResultHasReasonableAngles() {
        loadModel()
        let result = runIKWithStandingPose()
        guard let result else { return }

        // Joint angles should be within a reasonable range (< 180 degrees = pi radians)
        for angle in result.jointAngles {
            let value = angle.doubleValue
            XCTAssertLessThan(abs(value), .pi * 2,
                              "Joint angle should be within ±2π radians")
        }
    }

    // MARK: - Inverse Dynamics

    func testIDWithSyntheticData() {
        loadModel()
        let numDOFs = Int(bridge.numDOFs)
        guard numDOFs > 0 else { return }

        // Set up synthetic joint state (small angles, zero velocities/accelerations)
        let angles = Array(repeating: NSNumber(value: 0.0), count: numDOFs)
        let velocities = Array(repeating: NSNumber(value: 0.0), count: numDOFs)
        let accelerations = Array(repeating: NSNumber(value: 0.0), count: numDOFs)

        let result = bridge.solveID(withJointAngles: angles,
                                    jointVelocities: velocities,
                                    jointAccelerations: accelerations)
        XCTAssertNotNil(result, "ID should return a result")

        if let result {
            XCTAssertEqual(result.jointTorques.count, numDOFs,
                           "Should return torques for all DOFs")
            // At zero position/velocity/acceleration, torques should reflect gravity compensation
            let hasNonZeroTorque = result.jointTorques.contains(where: { $0.doubleValue != 0 })
            XCTAssertTrue(hasNonZeroTorque, "Gravity should produce non-zero torques")
        }
    }

    // MARK: - Edge Cases

    func testIKWithEmptyMarkers() {
        loadModel()
        let result = bridge.solveIK(withMarkerPositions: [], markerNames: [])
        XCTAssertNil(result, "Empty markers should return nil")
    }

    func testIKWithMismatchedArrays() {
        loadModel()
        // 2 names but only 3 positions (should be 6 = 2*3)
        let result = bridge.solveIK(
            withMarkerPositions: [1, 2, 3].map { NSNumber(value: $0) },
            markerNames: ["A", "B"]
        )
        XCTAssertNil(result, "Mismatched arrays should return nil")
    }

    func testIDWithWrongDOFCount() {
        loadModel()
        // Pass wrong number of angles
        let result = bridge.solveID(withJointAngles: [NSNumber(value: 0)],
                                    jointVelocities: [NSNumber(value: 0)],
                                    jointAccelerations: [NSNumber(value: 0)])
        XCTAssertNil(result, "Wrong DOF count should return nil")
    }

    func testLoadModelFromInvalidPath() {
        let success = bridge.loadModel(fromPath: "/nonexistent/model.osim")
        XCTAssertFalse(success)
        XCTAssertFalse(bridge.isModelLoaded)
    }

    // MARK: - Ground height estimation
    //
    // Ground height used to be a monotonic ratchet (`if lowest - 0.01 <
    // groundHeightY { groundHeightY = lowest - 0.01 }`), so a single crouch,
    // landing spike or bout of ARKit vertical drift permanently sank the floor
    // and ID stayed in its zero-external-force flight branch for the rest of
    // the session. These tests pin the rolling percentile estimator that
    // replaced it.

    func testGroundHeightIgnoresTransientDipInsteadOfRatchetingDown() {
        loadModel()
        for _ in 0..<120 { bridge.observeLowestFootHeightY(0.05) }
        XCTAssertEqual(bridge.groundHeightY, 0.04, accuracy: 1e-9)

        // Six frames of the heel reported 40 cm through the floor.
        for _ in 0..<6 { bridge.observeLowestFootHeightY(-0.40) }
        XCTAssertEqual(bridge.groundHeightY, 0.04, accuracy: 1e-9,
                       "A short burst of outliers must not move the floor")

        for _ in 0..<60 { bridge.observeLowestFootHeightY(0.05) }
        XCTAssertEqual(bridge.groundHeightY, 0.04, accuracy: 1e-9,
                       "Ground estimate must stay at the true floor as the session continues")
    }

    func testGroundHeightRisesWhenTheObservedFloorRises() {
        loadModel()
        for _ in 0..<180 { bridge.observeLowestFootHeightY(0.0) }
        XCTAssertEqual(bridge.groundHeightY, -0.01, accuracy: 1e-9)

        // The subject steps onto a 30 cm platform (or the world origin drifts).
        for _ in 0..<180 { bridge.observeLowestFootHeightY(0.30) }
        XCTAssertEqual(bridge.groundHeightY, 0.29, accuracy: 1e-9,
                       "Ground estimate must be able to rise, not just fall")
    }

    func testGroundHeightTrustProgression() {
        loadModel()
        XCTAssertEqual(bridge.groundHeightSource, .uncalibrated)
        XCTAssertFalse(bridge.groundHeightCalibrated)
        XCTAssertFalse(bridge.groundHeightTrusted)

        bridge.observeLowestFootHeightY(0.05)
        XCTAssertTrue(bridge.groundHeightCalibrated,
                      "A single sample already yields a usable estimate")
        XCTAssertEqual(bridge.groundHeightSource, .provisional)
        XCTAssertFalse(bridge.groundHeightTrusted,
                       "One sample is not enough to reject outliers")

        for _ in 0..<29 { bridge.observeLowestFootHeightY(0.05) }
        XCTAssertEqual(bridge.groundHeightSource, .estimated)
        XCTAssertTrue(bridge.groundHeightTrusted)
    }

    func testExplicitGroundHeightSurvivesObservedFootHeights() {
        loadModel()
        bridge.setGroundHeightY(0.12)
        XCTAssertEqual(bridge.groundHeightSource, .explicit)
        XCTAssertTrue(bridge.groundHeightTrusted)

        for _ in 0..<200 { bridge.observeLowestFootHeightY(-0.50) }
        XCTAssertEqual(bridge.groundHeightY, 0.12, accuracy: 1e-9,
                       "Explicit calibration must not be overwritten by observations")
        XCTAssertEqual(bridge.groundHeightSource, .explicit)
    }

    func testResetSessionStateClearsGroundEstimate() {
        loadModel()
        for _ in 0..<180 { bridge.observeLowestFootHeightY(-0.50) }
        XCTAssertEqual(bridge.groundHeightY, -0.51, accuracy: 1e-9)

        bridge.resetSessionState()
        XCTAssertEqual(bridge.groundHeightSource, .uncalibrated)
        XCTAssertFalse(bridge.groundHeightCalibrated)

        // The first post-reset sample defines the floor, instead of being
        // outvoted by samples taken in a stale world frame.
        bridge.observeLowestFootHeightY(0.20)
        XCTAssertEqual(bridge.groundHeightY, 0.19, accuracy: 1e-9)
    }

    func testResetSessionStateReleasesExplicitGroundCalibration() {
        loadModel()
        bridge.setGroundHeightY(0.12)
        XCTAssertEqual(bridge.groundHeightSource, .explicit)

        bridge.resetSessionState()
        XCTAssertEqual(bridge.groundHeightSource, .uncalibrated)

        bridge.observeLowestFootHeightY(0.30)
        XCTAssertEqual(bridge.groundHeightSource, .provisional)
        XCTAssertEqual(bridge.groundHeightY, 0.29, accuracy: 1e-9)
    }

    func testNonFiniteFootHeightSamplesAreIgnored() {
        loadModel()
        bridge.observeLowestFootHeightY(Double.nan)
        bridge.observeLowestFootHeightY(-Double.infinity)
        XCTAssertEqual(bridge.groundHeightSource, .uncalibrated)
        XCTAssertFalse(bridge.groundHeightCalibrated)

        bridge.observeLowestFootHeightY(0.05)
        XCTAssertEqual(bridge.groundHeightY, 0.04, accuracy: 1e-9)
    }

    func testGRFSolveFeedsTheGroundHeightEstimator() {
        loadModel()
        XCTAssertFalse(bridge.groundHeightCalibrated)

        let n = bridge.numDOFs
        let zeros = (0..<n).map { _ in NSNumber(value: 0.0) }
        _ = bridge.solveIDGRF(withJointAngles: zeros,
                              jointVelocities: zeros,
                              jointAccelerations: zeros)

        // Pins the wiring: the ID+GRF path must drive the rolling estimator
        // rather than maintaining its own ground height.
        XCTAssertTrue(bridge.groundHeightCalibrated)
        XCTAssertEqual(bridge.groundHeightSource, .provisional,
                       "One ID frame is one sample — provisional, not trusted")
    }

    // MARK: - IK warm start
    //
    // IK used to run five random restarts every frame (Nimble's default
    // maxRestarts with an unreachable lossLowerBound), each one calling
    // getRandomPose() and discarding the previous frame's solution. These
    // tests pin the warm-start path that replaced it.

    func testIKWarmStartIsUnavailableUntilFirstSolveAndClearedByReset() {
        loadModel()
        XCTAssertFalse(bridge.ikWarmStartAvailable,
                       "Freshly loaded model has no previous pose to warm-start from")

        XCTAssertNotNil(solveStandingPose())
        XCTAssertTrue(bridge.ikWarmStartAvailable)

        bridge.resetSessionState()
        XCTAssertFalse(bridge.ikWarmStartAvailable,
                       "Tracking loss must invalidate the warm-start pose")
    }

    func testIKOnlyResetPreservesTrustedGroundForASecondPass() {
        loadModel()
        for _ in 0..<30 { bridge.observeLowestFootHeightY(0.05) }
        XCTAssertTrue(bridge.groundHeightTrusted)
        let ground = bridge.groundHeightY

        XCTAssertNotNil(solveStandingPose())
        XCTAssertTrue(bridge.ikWarmStartAvailable)

        bridge.resetIKWarmStart()
        XCTAssertFalse(bridge.ikWarmStartAvailable)
        XCTAssertEqual(bridge.groundHeightSource, .estimated)
        XCTAssertTrue(bridge.groundHeightTrusted)
        XCTAssertEqual(bridge.groundHeightY, ground, accuracy: 1e-12,
                       "A second analysis pass belongs to the same floor")
    }

    func testRepeatedIKOnIdenticalMarkersIsStable() {
        loadModel()
        _ = solveStandingPose()  // first solve is cold; warms the solver
        guard let a = solveStandingPose(), let b = solveStandingPose() else {
            XCTFail("IK should solve the standing pose")
            return
        }

        let maxDelta = zip(a.jointAngles, b.jointAngles)
            .map { abs($0.0.doubleValue - $0.1.doubleValue) }
            .max() ?? 0
        XCTAssertLessThan(maxDelta, 1e-3,
                          "Warm-started IK on identical markers must not wander; " +
                          "random restarts can land in a different basin")
    }

    func testIKPoseIsContinuousUnderSmallMarkerPerturbation() {
        loadModel()
        _ = solveStandingPose()
        guard let a = solveStandingPose(),
              let b = solveStandingPose(shiftX: 0.002) else {
            XCTFail("IK should solve the standing pose")
            return
        }

        // 2 mm of marker motion is far below the ARKit noise floor; the pose
        // must move correspondingly little. A random restart winning the
        // search produces joint-angle jumps of order 0.5-1 rad, which the
        // Savitzky-Golay stage would differentiate twice into the ID input.
        let maxDelta = zip(a.jointAngles, b.jointAngles)
            .map { abs($0.0.doubleValue - $0.1.doubleValue) }
            .max() ?? 0
        XCTAssertLessThan(maxDelta, 0.15,
                          "A 2 mm marker shift must not jump the pose to a different basin")
    }

    // MARK: - Helpers

    /// ARKit-style joint-center markers for a neutral standing pose, in meters.
    /// `shiftX` offsets every marker laterally, to probe frame-to-frame
    /// continuity with a perturbation well below the ARKit noise floor.
    private func arkitStandingMarkers(shiftX: Double = 0)
        -> (positions: [NSNumber], names: [String]) {
        // NOTE: this fixture is a near-singular marker set — every point lies in
        // the z = 0 plane and each leg's HJC/KJC/AJC are colinear, so rotations
        // about the femur long axis barely move any marker. That is deliberate
        // here only in the sense that it is the ORIGINAL fixture; it is what
        // makes testRepeatedIKOnIdenticalMarkersIsStable fail. Perturbing it
        // toward a more realistic 3D pose (soft-knee z-offset, toe markers) was
        // tried and made the drift an order of magnitude WORSE (0.006 -> 0.19 ->
        // 0.84 rad), which is the signature of near-singularity amplification in
        // the damped pseudo-inverse rather than an exact null space. The real
        // defect is upstream: the bridge's IK has no null-space damping toward
        // the seed pose, and nimble's refineIK terminates on error-CHANGE, never
        // on lossLowerBound, so it keeps stepping along the flat manifold on
        // every call. Fix the solver, not this table.
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
        let markers = arkitStandingMarkers(shiftX: shiftX)
        return bridge.solveIK(withMarkerPositions: markers.positions,
                              markerNames: markers.names)
    }

    private func loadModel() {
        let path = Bundle(for: type(of: self)).path(forResource: "Rajagopal2016", ofType: "osim")
            ?? Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim")
        guard let path else {
            XCTFail("Cannot find Rajagopal2016.osim")
            return
        }
        let success = bridge.loadModel(fromPath: path)
        XCTAssertTrue(success)
    }

    private func runIKWithStandingPose() -> NimbleIKResult? {
        let modelMarkers = bridge.markerNames
        let numMarkers = min(5, modelMarkers.count)
        var positions: [NSNumber] = []
        var names: [String] = []

        for i in 0..<numMarkers {
            names.append(modelMarkers[i])
            positions.append(NSNumber(value: Double(i) * 0.1 - 0.2))
            positions.append(NSNumber(value: 1.0))
            positions.append(NSNumber(value: 0.0))
        }

        return bridge.solveIK(withMarkerPositions: positions, markerNames: names)
    }
}
