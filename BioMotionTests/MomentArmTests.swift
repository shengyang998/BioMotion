import XCTest
@testable import BioMotion

final class MomentArmTests: XCTestCase {

    private var computer: MomentArmComputer!
    private var bridge: NimbleBridge!

    override func setUp() {
        super.setUp()
        bridge = NimbleBridge()
        computer = MomentArmComputer()
    }

    func testParseMusclePaths() {
        loadModel()
        // Should parse all 80 muscles
        XCTAssertEqual(computer.numMuscles, 80, "Should parse 80 muscle paths")
    }

    func testMusclePathData() {
        loadModel()
        let soleus = computer.musclePathData(forName: "soleus_r")
        XCTAssertNotNil(soleus)
        if let s = soleus {
            XCTAssertEqual(s.name, "soleus_r")
            XCTAssertGreaterThan(s.pathPoints.count, 1, "Soleus should have ≥2 path points")
            XCTAssertGreaterThan(s.maxIsometricForce, 1000, "Soleus F0 should be > 1000N")
            XCTAssertGreaterThan(s.optimalFiberLength, 0.01)
        }
    }

    func testComputeMomentArms() {
        loadModel()
        let dofNames = ["hip_flexion_r", "knee_angle_r", "ankle_angle_r"]
        let angles = dofNames.map { _ in NSNumber(value: 0.0) }

        let result = computer.computeMomentArms(withJointAngles: angles, dofNames: dofNames)
        XCTAssertNotNil(result)

        if let result {
            XCTAssertEqual(result.count, 80 * 3, "Should be nMuscles × nDOFs")

            // Check that SOME moment arms are non-zero
            let nonZero = result.filter { abs($0.doubleValue) > 1e-6 }
            XCTAssertGreaterThan(nonZero.count, 0, "Some moment arms should be non-zero")
        }
    }

    func testSoleusAnkleMomentArm() {
        loadModel()
        let dofNames = ["ankle_angle_r"]
        let angles = [NSNumber(value: 0.0)]

        guard let result = computer.computeMomentArms(withJointAngles: angles, dofNames: dofNames) else {
            XCTFail("Should compute moment arms")
            return
        }

        // Find soleus_r index
        let names = computer.muscleNames
        guard let soleusIdx = (names as [String]).firstIndex(of: "soleus_r") else {
            XCTFail("soleus_r should exist")
            return
        }

        let soleusAnkleMomentArm = result[soleusIdx].doubleValue
        // Soleus should have a substantial moment arm at the ankle (~0.04-0.06m)
        // The sign depends on convention (plantarflexor = negative in OpenSim)
        XCTAssertGreaterThan(abs(soleusAnkleMomentArm), 0.01,
                             "Soleus should have a significant ankle moment arm, got \(soleusAnkleMomentArm)")
    }

    func testMomentArmPerformance() {
        loadModel()
        let dofNames = ["hip_flexion_r", "hip_flexion_l",
                         "knee_angle_r", "knee_angle_l",
                         "ankle_angle_r", "ankle_angle_l"]
        let angles = dofNames.map { _ in NSNumber(value: 0.0) }

        measure {
            _ = computer.computeMomentArms(withJointAngles: angles, dofNames: dofNames)
        }
    }

    func testOneEuroFilter() {
        let filter = OneEuroFilter(minCutoff: 1.0, beta: 0.01)

        // Feed a noisy signal (constant value + noise)
        var outputs: [Double] = []
        for i in 0..<100 {
            let t = Double(i) / 30.0  // 30 fps
            let noise = Double.random(in: -0.01...0.01)
            let value = 1.0 + noise
            outputs.append(filter.filter(value, timestamp: t))
        }

        // After warmup, filtered values should be closer to 1.0 than raw
        let lastOutputs = Array(outputs.suffix(20))
        let avgDeviation = lastOutputs.map { abs($0 - 1.0) }.reduce(0, +) / Double(lastOutputs.count)
        XCTAssertLessThan(avgDeviation, 0.005, "Filter should reduce noise significantly")
    }

    func testOneEuroFilter3D() {
        let filter = OneEuroFilter3D(minCutoff: 1.0, beta: 0.01)

        let smoothed = filter.filter(SIMD3<Float>(1.0, 2.0, 3.0), timestamp: 0.0)
        XCTAssertEqual(smoothed.x, 1.0, accuracy: 0.001)

        // Second sample with noise
        let smoothed2 = filter.filter(SIMD3<Float>(1.05, 2.05, 3.05), timestamp: 0.033)
        // Should be somewhere between the two values (partially filtered)
        XCTAssertGreaterThan(smoothed2.x, 0.99)
        XCTAssertLessThan(smoothed2.x, 1.06)
    }

    // MARK: - Geometry fidelity report

    /// The fallback model has no ConditionalPathPoint / MovingPathPoint at all,
    /// so document-order parsing must reproduce it byte-for-byte. This is the
    /// guard that the conditional/moving support cannot silently perturb the
    /// model the app falls back to.
    func testRajagopalFidelityReportIsUnchanged() {
        loadModel()
        let report = computer.fidelityReport

        XCTAssertEqual(report.musclesParsed, 80)
        XCTAssertEqual(computer.numMuscles, 80)
        XCTAssertEqual(report.pathPointsParsed, 288, "Rajagopal2016 has 288 PathPoints")
        XCTAssertEqual(report.conditionalPathPointsParsed, 0)
        XCTAssertEqual(report.conditionalPathPointsSkipped, 0)
        XCTAssertEqual(report.movingPathPointsParsed, 0)
        XCTAssertEqual(report.movingPathPointsSkipped, 0)
        XCTAssertEqual(report.unknownPathPointElementsSkipped, 0)
        XCTAssertEqual(report.musclesWithDefaultedTendonSlackLength, [String](),
                       "Every Rajagopal muscle declares tendon_slack_length")
        // Wrap geometry is deliberately out of scope, but must not be silent.
        XCTAssertEqual(report.unmodelledPathWraps, 46,
                       "46 PathWrap references exist and none of them are modelled")
    }

    /// FullBody.osim is what ships as production. Before document-order parsing
    /// every one of its 418 ConditionalPathPoints was dropped, so its lumbar and
    /// abdominal muscles cut straight through the spine.
    func testFullBodyParsesConditionalPathPoints() throws {
        guard let path = osimPath(named: "FullBody") else {
            throw XCTSkip("FullBody.osim is not reachable from the test bundle")
        }
        XCTAssertTrue(bridge.loadModel(fromPath: path), "NimbleBridge failed to load FullBody")
        XCTAssertTrue(computer.parseMusclePaths(fromOsimPath: path, from: bridge))

        let report = computer.fidelityReport
        XCTAssertGreaterThan(report.conditionalPathPointsParsed, 400,
                             "FullBody has 418 ConditionalPathPoints; \(report.summary)")
        XCTAssertEqual(report.conditionalPathPointsSkipped, 0,
                       "No ConditionalPathPoint may be dropped; \(report.summary)")
        XCTAssertEqual(report.pathPointsParsed, 1444)
        // A MovingPathPoint whose driving coordinate does not resolve to a
        // skeleton DOF is dropped rather than evaluated at an invented value,
        // so assert the total is accounted for rather than that all 4 survive.
        XCTAssertEqual(report.movingPathPointsParsed + report.movingPathPointsSkipped, 4)
        XCTAssertEqual(report.unknownPathPointElementsSkipped, 0)
        XCTAssertEqual(report.unmodelledPathWraps, 76,
                       "76 PathWrap references exist and none of them are modelled")
    }

    /// Path points are polyline vertices: a via point at the wrong index is
    /// worse than a missing one. Pins both ordering and the per-kind counts.
    func testSyntheticPathPointOrderingAndCounts() throws {
        try loadSyntheticModel()

        let report = computer.fidelityReport
        XCTAssertEqual(report.musclesParsed, 3)
        XCTAssertEqual(report.pathPointsParsed, 7)
        XCTAssertEqual(report.conditionalPathPointsParsed, 2)
        XCTAssertEqual(report.conditionalPathPointsSkipped, 0)
        XCTAssertEqual(report.conditionalPathPointsUnresolvedCoordinate, 0,
                       "ankle_angle_r must resolve to a skeleton DOF")
        XCTAssertEqual(report.movingPathPointsParsed, 1)
        XCTAssertEqual(report.movingPathPointsApproximated, 1,
                       "the 3-knot SimmSpline axis is linearly interpolated")
        XCTAssertEqual(report.movingPathPointsSkipped, 0)
        XCTAssertEqual(report.unknownPathPointElementsSkipped, 1)
        XCTAssertEqual(report.musclesWithDefaultedTendonSlackLength, ["notendon_r"])

        // Pin the pose so the MovingPathPoint is evaluated at a known coordinate.
        _ = computer.computeMomentArms(withJointAngles: [NSNumber(value: 0.0)],
                                       dofNames: ["ankle_angle_r"])

        guard let ordered = computer.musclePathData(forName: "ordered_r") else {
            XCTFail("ordered_r should be parsed")
            return
        }
        XCTAssertEqual(ordered.pathPoints.count, 5,
                       "3 PathPoint + 1 ConditionalPathPoint + 1 MovingPathPoint")
        // Document order is 0.0 (plain), 0.1 (conditional), 0.2 (plain),
        // 0.6 (moving, spline midpoint at q=0), 0.3 (plain).
        let xs = ordered.pathPoints.map { $0.x }
        XCTAssertEqual(xs[0], 0.0, accuracy: 1e-9)
        XCTAssertEqual(xs[1], 0.1, accuracy: 1e-9)
        XCTAssertEqual(xs[2], 0.2, accuracy: 1e-9)
        XCTAssertEqual(xs[3], 0.6, accuracy: 1e-9,
                       "MovingPathPoint x = linear interp of SimmSpline at q=0")
        XCTAssertEqual(xs[4], 0.3, accuracy: 1e-9)
    }

    /// A ConditionalPathPoint outside its `<range>` must leave the polyline.
    /// All three points of `gated_r` sit on the same body, so the length is
    /// pose-invariant and any change is purely the range test firing.
    func testConditionalPathPointRangeGating() throws {
        try loadSyntheticModel()

        guard let gatedIdx = (computer.muscleNames as [String]).firstIndex(of: "gated_r") else {
            XCTFail("gated_r should be parsed")
            return
        }

        // ankle_angle_r = 0 is OUTSIDE the via point's [0.5, 1.0] range:
        // the path collapses to the straight line between its endpoints.
        _ = computer.computeMomentArms(withJointAngles: [NSNumber(value: 0.0)],
                                       dofNames: ["ankle_angle_r"])
        let inactiveLength = computer.currentMuscleLengths[gatedIdx].doubleValue
        XCTAssertEqual(inactiveLength, 0.2, accuracy: 1e-6)

        // ankle_angle_r = 0.7 is inside the range: the via point re-enters and
        // the path detours through (0, 0.1, 0).
        _ = computer.computeMomentArms(withJointAngles: [NSNumber(value: 0.7)],
                                       dofNames: ["ankle_angle_r"])
        let activeLength = computer.currentMuscleLengths[gatedIdx].doubleValue
        XCTAssertEqual(activeLength, 0.1 + (0.05 as Double).squareRoot(), accuracy: 1e-6)
        XCTAssertGreaterThan(activeLength, inactiveLength)
    }

    // MARK: - Helpers

    private func osimPath(named name: String) -> String? {
        for bundle in [Bundle(for: type(of: self)), Bundle.main] {
            if let p = bundle.path(forResource: name, ofType: "osim") { return p }
            // XcodeGen adds BioMotion/Resources as a folder reference, which can
            // land the models in a "Resources" subdirectory of the bundle.
            if let p = bundle.path(forResource: name, ofType: "osim", inDirectory: "Resources") {
                return p
            }
        }
        return nil
    }

    /// Parses a hand-written .osim covering path-point kinds the shipped
    /// fallback model does not contain. The skeleton still comes from
    /// Rajagopal2016 (MomentArmComputer adopts the bridge's skeleton and only
    /// reads GeometryPath data from the file it is handed), so the body and
    /// coordinate names below must exist in that model.
    private func loadSyntheticModel() throws {
        let path = Bundle(for: type(of: self)).path(forResource: "Rajagopal2016", ofType: "osim")
            ?? Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim")
        guard let path else {
            XCTFail("Cannot find Rajagopal2016.osim")
            return
        }
        XCTAssertTrue(bridge.loadModel(fromPath: path), "NimbleBridge failed to load the model")

        let fixtureURL = FileManager.default.temporaryDirectory
            .appendingPathComponent("BioMotionSyntheticPath-\(UUID().uuidString).osim")
        try Self.syntheticOsimXML.write(to: fixtureURL, atomically: true, encoding: .utf8)
        addTeardownBlock { try? FileManager.default.removeItem(at: fixtureURL) }

        XCTAssertTrue(computer.parseMusclePaths(fromOsimPath: fixtureURL.path, from: bridge))
    }

    private static let syntheticOsimXML = """
    <?xml version="1.0" encoding="UTF-8" ?>
    <OpenSimDocument Version="40000">
      <Model name="synthetic">
        <ForceSet>
          <objects>
            <Millard2012EquilibriumMuscle name="ordered_r">
              <max_isometric_force>1000</max_isometric_force>
              <optimal_fiber_length>0.1</optimal_fiber_length>
              <tendon_slack_length>0.2</tendon_slack_length>
              <pennation_angle_at_optimal>0</pennation_angle_at_optimal>
              <GeometryPath>
                <PathPointSet>
                  <objects>
                    <PathPoint name="p0">
                      <socket_parent_frame>/bodyset/femur_r</socket_parent_frame>
                      <location>0 0 0</location>
                    </PathPoint>
                    <ConditionalPathPoint name="c1">
                      <socket_parent_frame>/bodyset/femur_r</socket_parent_frame>
                      <location>0.1 0 0</location>
                      <socket_coordinate>/jointset/ankle_r/ankle_angle_r</socket_coordinate>
                      <range>-10 10</range>
                    </ConditionalPathPoint>
                    <PathPoint name="p2">
                      <socket_parent_frame>/bodyset/femur_r</socket_parent_frame>
                      <location>0.2 0 0</location>
                    </PathPoint>
                    <MovingPathPoint name="m3">
                      <socket_parent_frame>/bodyset/femur_r</socket_parent_frame>
                      <socket_x_coordinate>/jointset/ankle_r/ankle_angle_r</socket_x_coordinate>
                      <socket_y_coordinate>/jointset/ankle_r/ankle_angle_r</socket_y_coordinate>
                      <socket_z_coordinate>/jointset/ankle_r/ankle_angle_r</socket_z_coordinate>
                      <x_location>
                        <SimmSpline><x> -1 1</x><y> 0.5 0.7</y></SimmSpline>
                      </x_location>
                      <y_location>
                        <Constant><value>0</value></Constant>
                      </y_location>
                      <z_location>
                        <SimmSpline><x> -1 0 1</x><y> 0 0 0</y></SimmSpline>
                      </z_location>
                    </MovingPathPoint>
                    <PathPoint name="p4">
                      <socket_parent_frame>/bodyset/femur_r</socket_parent_frame>
                      <location>0.3 0 0</location>
                    </PathPoint>
                    <UnsupportedFuturePathPoint name="weird">
                      <socket_parent_frame>/bodyset/femur_r</socket_parent_frame>
                      <location>9 9 9</location>
                    </UnsupportedFuturePathPoint>
                  </objects>
                </PathPointSet>
              </GeometryPath>
            </Millard2012EquilibriumMuscle>
            <Millard2012EquilibriumMuscle name="gated_r">
              <max_isometric_force>1000</max_isometric_force>
              <optimal_fiber_length>0.1</optimal_fiber_length>
              <tendon_slack_length>0.2</tendon_slack_length>
              <pennation_angle_at_optimal>0</pennation_angle_at_optimal>
              <GeometryPath>
                <PathPointSet>
                  <objects>
                    <PathPoint name="g0">
                      <socket_parent_frame>/bodyset/femur_r</socket_parent_frame>
                      <location>0 0 0</location>
                    </PathPoint>
                    <ConditionalPathPoint name="g1">
                      <socket_parent_frame>/bodyset/femur_r</socket_parent_frame>
                      <location>0 0.1 0</location>
                      <socket_coordinate>/jointset/ankle_r/ankle_angle_r</socket_coordinate>
                      <range>0.5 1.0</range>
                    </ConditionalPathPoint>
                    <PathPoint name="g2">
                      <socket_parent_frame>/bodyset/femur_r</socket_parent_frame>
                      <location>0.2 0 0</location>
                    </PathPoint>
                  </objects>
                </PathPointSet>
              </GeometryPath>
            </Millard2012EquilibriumMuscle>
            <Millard2012EquilibriumMuscle name="notendon_r">
              <max_isometric_force>1000</max_isometric_force>
              <optimal_fiber_length>0.1</optimal_fiber_length>
              <pennation_angle_at_optimal>0</pennation_angle_at_optimal>
              <GeometryPath>
                <PathPointSet>
                  <objects>
                    <PathPoint name="n0">
                      <socket_parent_frame>/bodyset/femur_r</socket_parent_frame>
                      <location>0 0 0</location>
                    </PathPoint>
                    <PathPoint name="n1">
                      <socket_parent_frame>/bodyset/femur_r</socket_parent_frame>
                      <location>0.2 0 0</location>
                    </PathPoint>
                  </objects>
                </PathPointSet>
              </GeometryPath>
            </Millard2012EquilibriumMuscle>
          </objects>
        </ForceSet>
      </Model>
    </OpenSimDocument>
    """

    private func loadModel() {
        let path = Bundle(for: type(of: self)).path(forResource: "Rajagopal2016", ofType: "osim")
            ?? Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim")
        guard let path else {
            XCTFail("Cannot find Rajagopal2016.osim")
            return
        }
        // MomentArmComputer adopts the bridge's already-loaded skeleton rather
        // than parsing its own, so the bridge must load the model first.
        XCTAssertTrue(bridge.loadModel(fromPath: path), "NimbleBridge failed to load the model")
        let success = computer.parseMusclePaths(fromOsimPath: path, from: bridge)
        XCTAssertTrue(success)
    }
}
