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
        // Rajagopal2016 declares 39 XML coordinates. Nimble omits the two
        // patellofemoral `knee_angle_*_beta` coordinates with their skipped
        // patella bodies, leaving 37 runtime DOFs.
        XCTAssertEqual(bridge.numDOFs, 37,
                       "Rajagopal2016's 39 XML coordinates must parse to 37 runtime DOFs")
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

    func testUnvalidatedIDDiagnosticWithSyntheticData() {
        loadModel()
        let numDOFs = Int(bridge.numDOFs)
        guard numDOFs > 0 else { return }

        // Set up synthetic joint state (small angles, zero velocities/accelerations)
        let angles = Array(repeating: NSNumber(value: 0.0), count: numDOFs)
        let velocities = Array(repeating: NSNumber(value: 0.0), count: numDOFs)
        let accelerations = Array(repeating: NSNumber(value: 0.0), count: numDOFs)

        let result = bridge.solveUnvalidatedIDForDiagnostics(
            withJointAngles: angles,
            jointVelocities: velocities,
            jointAccelerations: accelerations)
        XCTAssertNotNil(result, "the zero-external-force ID diagnostic should return a result")

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

    func testUnvalidatedIDDiagnosticWithWrongDOFCount() {
        loadModel()
        // Pass wrong number of angles
        let result = bridge.solveUnvalidatedIDForDiagnostics(
            withJointAngles: [NSNumber(value: 0)],
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

    func testBundledModelsFailClosedWithoutValidatedFootContactSupport() {
        for resource in ["FullBody", "Rajagopal2016"] {
            let candidate = NimbleBridge()
            let path = Bundle(for: type(of: self)).path(forResource: resource, ofType: "osim")
                ?? Bundle.main.path(forResource: resource, ofType: "osim")
            guard let path else {
                XCTFail("\(resource).osim must be in the test or app bundle")
                continue
            }

            XCTAssertTrue(candidate.loadModel(fromPath: path), resource)
            XCTAssertFalse(candidate.hasValidatedFootContactSupport,
                           "bundled models and the active solver do not define validated foot support")

            let zeros = (0..<candidate.numDOFs).map { _ in NSNumber(value: 0.0) }
            XCTAssertNil(candidate.solveIDGRF(withJointAngles: zeros,
                                               jointVelocities: zeros,
                                               jointAccelerations: zeros),
                         "production GRF must fail closed instead of publishing an unconstrained wrench")
            XCTAssertEqual(candidate.groundHeightSource, .uncalibrated,
                           "a rejected solve must not mutate the floor estimator")

            candidate.setGroundHeightY(0)
            XCTAssertNil(candidate.solveIDGRF(withJointAngles: zeros,
                                               jointVelocities: zeros,
                                               jointAccelerations: zeros),
                         "an explicit floor cannot manufacture a missing contact-support model")
            XCTAssertEqual(candidate.groundHeightSource, .explicit)

            candidate.resetSessionState()
            XCTAssertFalse(candidate.hasValidatedFootContactSupport,
                           "session reset must not invent or erase model capability")
            XCTAssertFalse(candidate.loadModel(fromPath: "/nonexistent/model.osim"))
            XCTAssertTrue(candidate.isModelLoaded,
                          "a failed reload must retain the last successful model")
            XCTAssertFalse(candidate.hasValidatedFootContactSupport,
                           "a failed reload must retain the old capability value")
        }
    }

    func testUnvalidatedDynamicsDiagnosticSelectorsRemainTestOnly() throws {
        let repositoryRoot = URL(fileURLWithPath: #filePath)
            .deletingLastPathComponent()
            .deletingLastPathComponent()

        let testHeader = try String(
            contentsOf: repositoryRoot.appendingPathComponent(
                "BioMotionTests/BioMotionTests-Bridging-Header.h"),
            encoding: .utf8)
        for selector in [
            "solveUnvalidatedIDForDiagnosticsWithJointAngles",
            "solveUnvalidatedIDGRFForDiagnosticsWithJointAngles",
        ] {
            for relativePath in [
                "BioMotion/Nimble/NimbleBridge.h",
                "BioMotion/Nimble/BioMotion-Bridging-Header.h",
                "BioMotion/Nimble/NimbleEngine.swift",
            ] {
                let source = try String(
                    contentsOf: repositoryRoot.appendingPathComponent(relativePath),
                    encoding: .utf8)
                XCTAssertFalse(source.contains(selector),
                               "\(selector) leaked into production surface \(relativePath)")
            }
            XCTAssertTrue(testHeader.contains(selector),
                          "the legacy numerical diagnostics need their test-only category")
        }

        let publicZeroForceSelector = "solveIDWithJointAngles"
        let publicHeader = try String(
            contentsOf: repositoryRoot.appendingPathComponent(
                "BioMotion/Nimble/NimbleBridge.h"),
            encoding: .utf8)
        XCTAssertFalse(publicHeader.contains(publicZeroForceSelector),
                       "zero-external-force ID must not regain a production selector")

        let implementation = try String(
            contentsOf: repositoryRoot.appendingPathComponent(
                "BioMotion/Nimble/NimbleBridge.mm"),
            encoding: .utf8)
        let diagnosticsGuard =
            "#if defined(BIOMOTION_TEST_DIAGNOSTICS) && BIOMOTION_TEST_DIAGNOSTICS"
        var inDiagnosticsBranch: Bool?
        var implementationCounts = [String: Int]()
        let selectors = [
            "solveUnvalidatedIDForDiagnosticsWithJointAngles",
            "solveUnvalidatedIDGRFForDiagnosticsWithJointAngles",
        ]
        for rawLine in implementation.split(separator: "\n", omittingEmptySubsequences: false) {
            let line = rawLine.trimmingCharacters(in: .whitespaces)
            if line == diagnosticsGuard {
                XCTAssertNil(inDiagnosticsBranch, "diagnostics guards must not nest")
                inDiagnosticsBranch = true
                continue
            }
            if inDiagnosticsBranch != nil && line == "#else" {
                inDiagnosticsBranch = false
                continue
            }
            if inDiagnosticsBranch != nil && line.hasPrefix("#endif") {
                inDiagnosticsBranch = nil
                continue
            }
            for selector in selectors where line.contains(selector) {
                implementationCounts[selector, default: 0] += 1
                XCTAssertEqual(inDiagnosticsBranch, true,
                               "\(selector) must compile only in the Debug diagnostics branch")
            }
        }
        XCTAssertNil(inDiagnosticsBranch, "unterminated diagnostics guard")
        XCTAssertEqual(implementationCounts[selectors[0]], 2,
                       "plain ID should have one private declaration and one implementation")
        XCTAssertEqual(implementationCounts[selectors[1]], 3,
                       "GRF should have a declaration, guarded delegate, and implementation")

        let projectSpec = try String(
            contentsOf: repositoryRoot.appendingPathComponent("project.yml"),
            encoding: .utf8)
        let macroDefinition = "- \"BIOMOTION_TEST_DIAGNOSTICS=1\""
        XCTAssertEqual(
            projectSpec.components(separatedBy: macroDefinition).count - 1,
            1,
            "the diagnostics macro must be defined exactly once, by the app Debug config")
        let appStart = try XCTUnwrap(projectSpec.range(of: "  BioMotion:\n"))
        let appEnd = try XCTUnwrap(projectSpec.range(
            of: "\n  AssetPackDownloader:",
            range: appStart.upperBound..<projectSpec.endIndex))
        let appTarget = String(projectSpec[appStart.lowerBound..<appEnd.lowerBound])
        XCTAssertTrue(appTarget.contains("      configs:\n        Debug:"))
        let macroRange = try XCTUnwrap(
            appTarget.range(of: macroDefinition))
        let beforeMacro = appTarget[..<macroRange.lowerBound]
        let nearestDebug = try XCTUnwrap(
            beforeMacro.range(of: "\n        Debug:", options: .backwards))
        if let nearestRelease = beforeMacro.range(
            of: "\n        Release:", options: .backwards) {
            XCTAssertGreaterThan(nearestDebug.lowerBound, nearestRelease.lowerBound,
                                 "the macro's nearest configuration must be Debug")
        }

        let generatedProject = try String(
            contentsOf: repositoryRoot.appendingPathComponent(
                "BioMotion.xcodeproj/project.pbxproj"),
            encoding: .utf8)
        let generatedMacro = "\"BIOMOTION_TEST_DIAGNOSTICS=1\","
        XCTAssertEqual(
            generatedProject.components(separatedBy: generatedMacro).count - 1,
            1,
            "xcodegen output must carry the Debug-only macro exactly once")
        let generatedMacroRange = try XCTUnwrap(
            generatedProject.range(of: generatedMacro))
        let generatedConfigStart = try XCTUnwrap(generatedProject.range(
            of: "isa = XCBuildConfiguration;",
            options: .backwards,
            range: generatedProject.startIndex..<generatedMacroRange.lowerBound))
        let generatedConfigEnd = try XCTUnwrap(generatedProject.range(
            of: "\n\t\t};",
            range: generatedMacroRange.upperBound..<generatedProject.endIndex))
        let generatedConfig = String(
            generatedProject[generatedConfigStart.lowerBound..<generatedConfigEnd.upperBound])
        XCTAssertTrue(generatedConfig.contains("name = Debug;"))
        XCTAssertFalse(generatedConfig.contains("name = Release;"))
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
        assertRepeatedIKIsStable(
            markers: arkitStandingMarkers(),
            fixture: "planar"
        )
        assertRepeatedIKIsStable(
            markers: arkitThreeDimensionalStandingMarkers(),
            fixture: "three-dimensional"
        )
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
        // This near-singular planar fixture originally exposed repeated-solve
        // drift. Keep it beside the 3D amplification fixture below as a
        // permanent fixed-point regression for the shipped bridge solver.
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

    /// The historical discriminator that amplified the old repeated-solve
    /// drift to 0.19 / 0.84 rad: soft-knee depth plus toe markers.
    private func arkitThreeDimensionalStandingMarkers()
        -> (positions: [NSNumber], names: [String]) {
        let layout: [(String, Double, Double, Double)] = [
            ("PELVIS",  0.00, 0.95,  0.00),
            ("LHJC",   -0.09, 0.92,  0.00),
            ("RHJC",    0.09, 0.92,  0.00),
            ("LKJC",   -0.09, 0.52,  0.04),
            ("RKJC",    0.09, 0.52,  0.04),
            ("LAJC",   -0.09, 0.10,  0.00),
            ("RAJC",    0.09, 0.10,  0.00),
            ("LTOE",   -0.09, 0.03,  0.16),
            ("RTOE",    0.09, 0.03,  0.16),
            ("C7",      0.00, 1.40, -0.02),
            ("LSJC",   -0.18, 1.35, -0.02),
            ("RSJC",    0.18, 1.35, -0.02),
            ("LEJC",   -0.20, 1.07,  0.02),
            ("REJC",    0.20, 1.07,  0.02),
        ]
        var positions: [NSNumber] = []
        var names: [String] = []
        for (name, x, y, z) in layout {
            names.append(name)
            positions.append(NSNumber(value: x))
            positions.append(NSNumber(value: y))
            positions.append(NSNumber(value: z))
        }
        return (positions, names)
    }

    private func assertRepeatedIKIsStable(
        markers: (positions: [NSNumber], names: [String]),
        fixture: String,
        file: StaticString = #filePath,
        line: UInt = #line
    ) {
        let isolatedBridge = NimbleBridge()
        guard loadModel(into: isolatedBridge, file: file, line: line) else { return }

        var previousWarmPose: [Double]?
        for solveIndex in 0..<8 {
            guard let result = isolatedBridge.solveIK(
                withMarkerPositions: markers.positions,
                markerNames: markers.names
            ) else {
                XCTFail("IK should solve the \(fixture) fixture", file: file, line: line)
                return
            }

            let pose = result.jointAngles.map(\.doubleValue)
            let expectedDOFs = Int(isolatedBridge.numDOFs)
            XCTAssertGreaterThan(expectedDOFs, 0, file: file, line: line)
            XCTAssertEqual(pose.count, expectedDOFs, file: file, line: line)
            guard expectedDOFs > 0, pose.count == expectedDOFs else { return }
            // Solve 0 is cold and solve 1 establishes the warm fixed point.
            // Every later identical input must remain at that point.
            if solveIndex >= 2, let previousWarmPose {
                XCTAssertEqual(pose.count, previousWarmPose.count, file: file, line: line)
                let maxDelta = zip(previousWarmPose, pose)
                    .map { abs($0.0 - $0.1) }
                    .max() ?? .infinity
                XCTAssertLessThan(
                    maxDelta,
                    1e-6,
                    "Warm-started IK wandered on the \(fixture) fixture at solve \(solveIndex)",
                    file: file,
                    line: line
                )
            }
            previousWarmPose = pose
        }
    }

    private func solveStandingPose(shiftX: Double = 0) -> NimbleIKResult? {
        let markers = arkitStandingMarkers(shiftX: shiftX)
        return bridge.solveIK(withMarkerPositions: markers.positions,
                              markerNames: markers.names)
    }

    private func loadModel() {
        _ = loadModel(into: bridge)
    }

    @discardableResult
    private func loadModel(
        into target: NimbleBridge,
        file: StaticString = #filePath,
        line: UInt = #line
    ) -> Bool {
        let path = Bundle(for: type(of: self)).path(forResource: "Rajagopal2016", ofType: "osim")
            ?? Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim")
        guard let path else {
            XCTFail("Cannot find Rajagopal2016.osim", file: file, line: line)
            return false
        }
        let success = target.loadModel(fromPath: path)
        XCTAssertTrue(success, file: file, line: line)
        return success
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
