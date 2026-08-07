import XCTest
@testable import BioMotion

/// Pins the three properties the IK stage has to have before anything
/// downstream of it means anything:
///
///   1. a repeated solve on IDENTICAL markers is a fixed point,
///   2. the fit it reaches on a hard, real pose is as good as that pose allows,
///   3. the answer does not depend on how many solves preceded it in the
///      process (no RNG, no leftover skeleton state).
///
/// Every number printed here is prefixed `IKCONV-METRIC` so a run can be
/// diffed against an earlier one from the xcodebuild log alone.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHY `error` IS NOT USED AS THE ACCURACY NUMBER
/// ─────────────────────────────────────────────────────────────────────────
/// `NimbleIKResult.error` is nimble's LOSS — the squared norm of the
/// *reliability-weighted* residual stack, in m². It was documented as "RMS
/// marker error in meters" for a long time, and that is how the dancer
/// fixture's 0.0138 loss got read as 1.4 cm when the real per-marker error was
/// 2.6 cm. These tests read `markerRMSMeters`, which is computed from the
/// solved skeleton's own marker world positions with the weights removed.
final class IKConvergenceTests: XCTestCase {

    private var bridge: NimbleBridge!

    override func setUp() {
        super.setUp()
        continueAfterFailure = true   // every metric must print, even after a failure
        bridge = NimbleBridge()
    }

    // MARK: - Fixtures

    /// The real Core ML dancer pose. Shared with `OfflineMuscleChainTests` and
    /// `StaticEquilibriumBenchmarkTests` so they cannot drift apart.
    private var dancerMarkers: (positions: [NSNumber], names: [String]) {
        var positions: [NSNumber] = []
        var names: [String] = []
        for (_, opensim, p) in OfflineMuscleChainFixture.markers {
            names.append(opensim)
            positions.append(NSNumber(value: Double(p.x)))
            positions.append(NSNumber(value: Double(p.y)))
            positions.append(NSNumber(value: Double(p.z)))
        }
        return (positions, names)
    }

    /// A neutral standing pose built from FullBody.osim's own rigid geometry —
    /// the same construction `StaticEquilibriumBenchmarkTests` uses, so it is
    /// exactly reachable and its residual is pure solver error.
    private var standingMarkers: (positions: [NSNumber], names: [String]) {
        let ankleY = 0.0750
        let kneeY = ankleY + 0.4000
        let hipY = kneeY + 0.4060
        let layout: [(String, Double, Double, Double)] = [
            ("PELVIS",  0.000, hipY + 0.0785, 0.0000),
            ("LHJC",   -0.0563, hipY, -0.0773),
            ("RHJC",   -0.0563, hipY,  0.0773),
            ("LKJC",   -0.0527, kneeY, -0.0770),
            ("RKJC",   -0.0527, kneeY,  0.0770),
            ("LAJC",   -0.0627, ankleY, -0.0770),
            ("RAJC",   -0.0627, ankleY,  0.0770),
        ]
        var positions: [NSNumber] = []
        var names: [String] = []
        for (n, x, y, z) in layout {
            names.append(n)
            positions.append(NSNumber(value: x))
            positions.append(NSNumber(value: y))
            positions.append(NSNumber(value: z))
        }
        return (positions, names)
    }

    private func loadFullBody() throws {
        let path = try XCTUnwrap(
            Bundle(for: type(of: self)).path(forResource: "FullBody", ofType: "osim")
                ?? Bundle.main.path(forResource: "FullBody", ofType: "osim"),
            "Cannot find FullBody.osim")
        XCTAssertTrue(bridge.loadModel(fromPath: path), "bridge failed to load FullBody.osim")
    }

    @discardableResult
    private func solve(_ m: (positions: [NSNumber], names: [String])) -> NimbleIKResult? {
        bridge.solveIK(withMarkerPositions: m.positions, markerNames: m.names)
    }

    private func q(_ r: NimbleIKResult) -> [Double] { r.jointAngles.map { $0.doubleValue } }

    private func maxAbsDelta(_ a: [Double], _ b: [Double]) -> Double {
        zip(a, b).map { abs($0 - $1) }.max() ?? .infinity
    }

    private func norm(_ a: [Double]) -> Double { sqrt(a.reduce(0) { $0 + $1 * $1 }) }

    // MARK: - 1. Repeated solves on identical markers are a fixed point

    /// The defect this replaces: nimble's `refineIK` (IKSolver.cpp:321)
    /// terminates on error-CHANGE < 1e-7 or on step count, never on
    /// stationarity, so it stops while `q` is still moving along the flat
    /// manifold; every fresh call then resets `lr` to 1.0 and resumes. Measured
    /// drift was ~6e-3 rad per solve on the planar fixture and did not stop
    /// after 400 solves.
    ///
    /// The bound here is not a tolerance chosen to pass. 1e-6 rad is 5.7e-5
    /// degrees: three orders of magnitude below the 1e-3 rad the old test asked
    /// for and about eight below the ~1e-2 rad of real pose error. A solver
    /// that has genuinely stopped moving sits many orders below it; a solver
    /// that is still creeping does not.
    func testRepeatedDancerSolvesReachAFixedPoint() throws {
        try loadFullBody()
        let m = dancerMarkers

        var qs: [[Double]] = []
        for _ in 0..<8 {
            guard let r = solve(m) else { XCTFail("IK returned nil"); return }
            qs.append(q(r))
        }
        for i in 1..<qs.count {
            print(String(format: "IKCONV-METRIC dancer_drift solve=%d maxAbsDelta=%.6e",
                         i, maxAbsDelta(qs[i - 1], qs[i])))
        }
        // Solve 1 is cold, 2..7 are warm refinements of an identical input.
        let lateDrift = (3..<qs.count).map { maxAbsDelta(qs[$0 - 1], qs[$0]) }.max() ?? .infinity
        print(String(format: "IKCONV-METRIC dancer_late_drift_rad=%.6e", lateDrift))
        XCTAssertLessThan(lateDrift, 1e-6,
                          "warm-started IK on identical markers is still moving")
    }

    func testRepeatedStandingSolvesReachAFixedPoint() throws {
        try loadFullBody()
        let m = standingMarkers

        var qs: [[Double]] = []
        for _ in 0..<8 {
            guard let r = solve(m) else { XCTFail("IK returned nil"); return }
            qs.append(q(r))
        }
        for i in 1..<qs.count {
            print(String(format: "IKCONV-METRIC standing_drift solve=%d maxAbsDelta=%.6e",
                         i, maxAbsDelta(qs[i - 1], qs[i])))
        }
        let lateDrift = (3..<qs.count).map { maxAbsDelta(qs[$0 - 1], qs[$0]) }.max() ?? .infinity
        print(String(format: "IKCONV-METRIC standing_late_drift_rad=%.6e", lateDrift))
        XCTAssertLessThan(lateDrift, 1e-6)
    }

    // MARK: - 2. Accuracy on the hard pose

    /// The dancer fixture is a real Core ML output at a stature that is NOT the
    /// unscaled model's, so part of its residual is unreachable geometry rather
    /// than solver error. The per-marker breakdown is printed so the two can be
    /// told apart, and the rigid-pair floor below quantifies the geometry half.
    func testDancerMarkerFit() throws {
        try loadFullBody()
        let m = dancerMarkers

        guard let cold = solve(m) else { XCTFail("IK returned nil"); return }
        print(String(format: "IKCONV-METRIC dancer_cold rms_cm=%.4f max_cm=%.4f loss=%.9f weighted_rms_cm=%.4f markers=%d iters=%ld converged=%@",
                     cold.markerRMSMeters * 100, cold.markerMaxErrorMeters * 100,
                     cold.error, (cold.error / Double(cold.markerCount)).squareRoot() * 100,
                     cold.markerCount, cold.iterations,
                     cold.converged ? "YES" : "NO"))

        var last = cold
        for _ in 0..<12 { if let r = solve(m) { last = r } }
        print(String(format: "IKCONV-METRIC dancer_settled rms_cm=%.4f max_cm=%.4f loss=%.9f",
                     last.markerRMSMeters * 100, last.markerMaxErrorMeters * 100, last.error))
        for (name, e) in last.markerErrorsMeters.sorted(by: { $0.value.doubleValue > $1.value.doubleValue }) {
            print(String(format: "IKCONV-METRIC dancer_marker %@=%.2f cm", name, e.doubleValue * 100))
        }

        // Hitting the per-phase iteration ceiling instead of a convergence test
        // is the failure mode that made the old solver drift, so it fails the
        // test rather than quietly degrading the fit.
        XCTAssertTrue(cold.converged,
                      "the cold dancer solve ran out of iterations (\(cold.iterations)) " +
                      "instead of reaching a stationary point")
        XCTAssertLessThan(last.markerRMSMeters, 0.024,
                          "dancer per-marker RMS regressed past the geometry floor this " +
                          "unscaled fixture allows")
    }

    /// How much of the answer is the seed's choice rather than the data's.
    ///
    /// The problem is non-convex and rank-deficient, so a warm start is a real
    /// input, not just an accelerator. Printed rather than asserted: a
    /// difference here is a property of the problem, and pinning it with a
    /// bound would only pin today's arithmetic.
    func testSeedSensitivityIsMeasuredNotAssumedAway() throws {
        try loadFullBody()
        let m = dancerMarkers

        bridge.resetSessionState()
        guard let fromNeutral = solve(m) else { XCTFail("IK returned nil"); return }

        // Same markers, but seeded from a solved standing pose instead.
        bridge.resetSessionState()
        _ = solve(standingMarkers)
        guard let fromStanding = solve(m) else { XCTFail("IK returned nil"); return }

        let delta = maxAbsDelta(q(fromNeutral), q(fromStanding))
        print(String(format: "IKCONV-METRIC seed_sensitivity maxAbsDelta=%.6e neutral_rms_cm=%.4f neutral_loss=%.9f standing_rms_cm=%.4f standing_loss=%.9f",
                     delta,
                     fromNeutral.markerRMSMeters * 100, fromNeutral.error,
                     fromStanding.markerRMSMeters * 100, fromStanding.error))
    }

    /// How much of the dancer residual is geometry the model cannot reach.
    ///
    /// For any two markers rigidly separated by `D_model` in the skeleton and
    /// `D_target` in the input, the two errors must satisfy
    /// `‖e_a‖ + ‖e_b‖ ≥ |D_model − D_target|` — no pose can shorten a bone.
    /// This prints the mismatch for each rigid pair the fixture constrains, so
    /// the residual can be attributed instead of argued about.
    func testDancerResidualIsDominatedByUnreachableSegmentLengths() throws {
        try loadFullBody()
        let m = dancerMarkers
        guard let r = solve(m) else { XCTFail("IK returned nil"); return }

        // Model-side rigid distances, read off the neutral FK geometry recorded
        // in StaticEquilibriumBenchmarkTests' header.
        let modelPairs: [(String, String, Double)] = [
            ("PELVIS", "RHJC", 0.12372),   // pelvis origin -> femur origin
            ("PELVIS", "LHJC", 0.12372),
            ("RHJC",   "RKJC", 0.40612),   // femur
            ("LHJC",   "LKJC", 0.40612),
            ("RKJC",   "RAJC", 0.40013),   // tibia
            ("LKJC",   "LAJC", 0.40013),
        ]
        func target(_ n: String) -> SIMD3<Double> {
            let p = OfflineMuscleChainFixture.markers.first { $0.1 == n }!.2
            return SIMD3<Double>(Double(p.x), Double(p.y), Double(p.z))
        }
        var floorSumSq = 0.0
        for (a, b, dModel) in modelPairs {
            let d = target(a) - target(b)
            let dTarget = (d.x * d.x + d.y * d.y + d.z * d.z).squareRoot()
            let gap = abs(dModel - dTarget)
            // Best case the gap is split evenly, so each marker carries gap/2.
            floorSumSq += 2 * (gap / 2) * (gap / 2)
            print(String(format: "IKCONV-METRIC rigid_pair %@-%@ model=%.4f target=%.4f gap_cm=%.2f",
                         a, b, dModel, dTarget, gap * 100))
        }
        let floorRMS = (floorSumSq / Double(r.markerCount)).squareRoot()
        print(String(format: "IKCONV-METRIC geometry_floor_rms_cm=%.4f achieved_rms_cm=%.4f ratio=%.2f",
                     floorRMS * 100, r.markerRMSMeters * 100, r.markerRMSMeters / floorRMS))

        // Hip WIDTH is the control. If the subject were simply a different size
        // from the model, |LHJC−RHJC| would be off by a similar fraction to
        // |PELVIS−HJC|. If instead the width agrees and only the pelvis-to-hip
        // distance is wrong, the PELVIS virtual marker is registered at the
        // wrong point on the pelvis body — a marker-table error, which no
        // amount of solving or scaling can absorb.
        let dHip = target("LHJC") - target("RHJC")
        let hipWidthTarget = (dHip.x * dHip.x + dHip.y * dHip.y + dHip.z * dHip.z).squareRoot()
        print(String(format: "IKCONV-METRIC hip_width model=%.4f target=%.4f gap_cm=%.2f",
                     0.1546, hipWidthTarget, abs(0.1546 - hipWidthTarget) * 100))

        // Assumption-free version of the same question: drop PELVIS from the
        // solve and see what the other 19 markers can then reach.
        var pos: [NSNumber] = []
        var names: [String] = []
        for (_, opensim, p) in OfflineMuscleChainFixture.markers where opensim != "PELVIS" {
            names.append(opensim)
            pos.append(NSNumber(value: Double(p.x)))
            pos.append(NSNumber(value: Double(p.y)))
            pos.append(NSNumber(value: Double(p.z)))
        }
        bridge.resetSessionState()
        var noPelvis: NimbleIKResult?
        for _ in 0..<4 {
            noPelvis = bridge.solveIK(withMarkerPositions: pos, markerNames: names)
        }
        if let np = noPelvis {
            print(String(format: "IKCONV-METRIC dancer_without_pelvis_marker rms_cm=%.4f max_cm=%.4f markers=%d",
                         np.markerRMSMeters * 100, np.markerMaxErrorMeters * 100, np.markerCount))
        }
    }

    /// Same pose through the scaling the shipping offline path applies.
    ///
    /// `OfflineSessionRunner` does NOT hand the posed markers to
    /// `scaleModelWithHeight` — that fails the [0.7, 1.4] clamp on 6 of 6
    /// measured frames because a bent limb's straight-line length is not its
    /// segment length. It builds a synthetic straight-limb marker set from the
    /// pose-invariant chain sums first (`MHRRetarget.segmentScaleMarkers`).
    /// Reproduced here, otherwise this test would measure a path that does not
    /// ship.
    func testDancerMarkerFitAfterSegmentScaling() throws {
        try loadFullBody()
        let m = dancerMarkers

        func p(_ n: String) -> SIMD3<Double> {
            let v = OfflineMuscleChainFixture.markers.first { $0.1 == n }!.2
            return SIMD3<Double>(Double(v.x), Double(v.y), Double(v.z))
        }
        func dist(_ a: String, _ b: String) -> Double {
            let d = p(a) - p(b)
            return (d.x * d.x + d.y * d.y + d.z * d.z).squareRoot()
        }
        let lLower = dist("LHJC", "LKJC") + dist("LKJC", "LAJC")
        let rLower = dist("RHJC", "RKJC") + dist("RKJC", "RAJC")
        let lUpper = dist("LSJC", "LEJC") + dist("LEJC", "LWJC")
        let rUpper = dist("RSJC", "REJC") + dist("REJC", "RWJC")
        let halfHip = 0.5 * dist("LHJC", "RHJC")
        let halfShoulder = 0.5 * dist("LSJC", "RSJC")
        let shoulderMid = 0.5 * (p("LSJC") + p("RSJC"))
        let trunkV = shoulderMid - p("PELVIS")
        let trunk = (trunkV.x * trunkV.x + trunkV.y * trunkV.y + trunkV.z * trunkV.z).squareRoot()

        let layout: [(String, SIMD3<Double>)] = [
            ("PELVIS", SIMD3(0, 0, 0)),
            ("LHJC", SIMD3(halfHip, 0, 0)), ("RHJC", SIMD3(-halfHip, 0, 0)),
            ("LAJC", SIMD3(halfHip, -lLower, 0)), ("RAJC", SIMD3(-halfHip, -rLower, 0)),
            ("LSJC", SIMD3(halfShoulder, trunk, 0)), ("RSJC", SIMD3(-halfShoulder, trunk, 0)),
            ("LWJC", SIMD3(halfShoulder + lUpper, trunk, 0)),
            ("RWJC", SIMD3(-halfShoulder - rUpper, trunk, 0)),
        ]
        var scalePos: [NSNumber] = []
        var scaleNames: [String] = []
        for (n, v) in layout {
            scaleNames.append(n)
            scalePos.append(NSNumber(value: v.x))
            scalePos.append(NSNumber(value: v.y))
            scalePos.append(NSNumber(value: v.z))
        }
        XCTAssertTrue(bridge.scaleModel(withHeight: 1.75,
                                        markerPositions: scalePos,
                                        markerNames: scaleNames))

        var last: NimbleIKResult?
        for _ in 0..<8 { if let r = solve(m) { last = r } }
        let r = try XCTUnwrap(last)
        print(String(format: "IKCONV-METRIC dancer_scaled rms_cm=%.4f max_cm=%.4f loss=%.9f converged=%@",
                     r.markerRMSMeters * 100, r.markerMaxErrorMeters * 100, r.error,
                     r.converged ? "YES" : "NO"))
        for (name, e) in r.markerErrorsMeters.sorted(by: { $0.value.doubleValue > $1.value.doubleValue }).prefix(6) {
            print(String(format: "IKCONV-METRIC dancer_scaled_marker %@=%.2f cm", name, e.doubleValue * 100))
        }
    }

    // MARK: - The null-space damping is doing what it claims

    /// 72 of this model's coordinates have an identically-zero column in the
    /// 20-marker Jacobian (`FullBodyDOFFixture.structurallyUnreachableCoordinates`,
    /// generated by a direct parse). Nothing in the data can decide their
    /// values, so a solver with no null-space regularisation puts them wherever
    /// its numerics happen to leave them — and the Savitzky-Golay stage then
    /// differentiates that twice, at gain ~1/dt², straight into the
    /// accelerations that drive inverse dynamics.
    ///
    /// Phase A pins them at the seed. Phase B cannot move them because its
    /// steps lie in the row space of `J`, which by definition excludes a
    /// zero column. So they must come back at exactly their seed value.
    func testUnobservableCoordinatesStayAtTheSeed() throws {
        try loadFullBody()
        let names = bridge.dofNames
        let unreachable = Set(FullBodyDOFFixture.structurallyUnreachableCoordinates)
        let idx = names.enumerated().filter { unreachable.contains($0.element) }.map { $0.offset }
        XCTAssertFalse(idx.isEmpty, "fixture names must resolve against the loaded model")

        // Cold solve from the neutral seed: every unobservable coordinate must
        // still hold its neutral value.
        guard let cold = solve(dancerMarkers) else { XCTFail("IK returned nil"); return }
        let qc = q(cold)
        var worst = 0.0
        var worstName = ""
        for i in idx where abs(qc[i]) > worst { worst = abs(qc[i]); worstName = names[i] }
        print(String(format: "IKCONV-METRIC unobservable_after_cold worst=%.6e (%@) count=%d",
                     worst, worstName, idx.count))
        XCTAssertLessThan(worst, 1e-9,
                          "\(worstName) is invisible to every marker yet the solve moved it")

        // And a second pose must not move them either.
        guard let second = solve(standingMarkers) else { XCTFail("IK returned nil"); return }
        let qs = q(second)
        var worst2 = 0.0
        for i in idx { worst2 = max(worst2, abs(qs[i] - qc[i])) }
        print(String(format: "IKCONV-METRIC unobservable_after_second_pose worst_move=%.6e", worst2))
        XCTAssertLessThan(worst2, 1e-9)
    }

    /// The exactly-reachable standing pose. This one has no geometry excuse:
    /// residual here is pure solver error, and the hand-built benchmark poses
    /// already fit to ~0.1 mm, so this is the non-regression floor.
    func testStandingPoseStillFitsToATenthOfAMillimetre() throws {
        try loadFullBody()
        let m = standingMarkers
        guard let r = solve(m) else { XCTFail("IK returned nil"); return }
        print(String(format: "IKCONV-METRIC standing rms_mm=%.6f max_mm=%.6f iters=%ld converged=%@",
                     r.markerRMSMeters * 1000, r.markerMaxErrorMeters * 1000,
                     r.iterations, r.converged ? "YES" : "NO"))
        XCTAssertLessThan(r.markerRMSMeters, 2e-4,
                          "an exactly reachable pose must fit to well under a millimetre")
    }

    // MARK: - 3. Order independence

    /// The reported symptom: the dancer landed on two different solutions
    /// (‖q‖ 4.52 vs 5.06) depending only on how many poses were solved earlier
    /// in the same test method, *even after* `resetSessionState()`.
    ///
    /// Two mechanisms produced that, and both are checked here:
    ///  * `Skeleton::getRandomPose()` draws from the process-global `rand()`,
    ///    so the cold path's random restarts inherit whatever RNG state earlier
    ///    tests left behind;
    ///  * `fitMarkersToWorldPositions` seeds `math::solveIK` with the
    ///    skeleton's CURRENT positions, and `resetSessionState` clears the
    ///    bridge's warm-start pose without touching the shared skeleton — so a
    ///    "cold" solve still started from the previous pose.
    ///
    /// The RNG stream is perturbed by running an UNEQUAL number of unrelated
    /// solves between the two arms, which is exactly the reported symptom and
    /// needs no `srand` (unavailable in Swift). Each cold solve that still used
    /// random restarts consumed `5 × numDOFs` draws from `rand()`, so three
    /// extra solves put the two arms in completely different RNG states.
    func testDancerSolutionIsIndependentOfPrecedingSolvesAndRNG() throws {
        try loadFullBody()
        let dancer = dancerMarkers
        let standing = standingMarkers

        bridge.resetSessionState()
        guard let a = solve(dancer) else { XCTFail("IK returned nil"); return }

        // A different amount of unrelated work — which is also a different RNG
        // stream position — then the identical request.
        for _ in 0..<3 {
            _ = solve(standing)
            bridge.resetSessionState()   // forces the cold path, which is where
                                         // the random restarts lived
        }
        bridge.resetSessionState()
        guard let b = solve(dancer) else { XCTFail("IK returned nil"); return }

        let qa = q(a), qb = q(b)
        let delta = maxAbsDelta(qa, qb)
        print(String(format: "IKCONV-METRIC order_independence maxAbsDelta=%.6e normA=%.6f normB=%.6f rmsA_cm=%.4f rmsB_cm=%.4f",
                     delta, norm(qa), norm(qb),
                     a.markerRMSMeters * 100, b.markerRMSMeters * 100))
        XCTAssertLessThan(delta, 1e-9,
                          "same markers after resetSessionState must give the same pose " +
                          "regardless of preceding solves or RNG state")
    }

    /// Two independently constructed bridges, same model, same markers, with
    /// unrelated solves run in between so the process-global RNG and the shared
    /// C++ allocator state differ. Catches any residual dependence on
    /// process-global state that a single-bridge test would miss.
    func testTwoBridgesAgreeBitForBit() throws {
        try loadFullBody()
        let m = dancerMarkers
        guard let a = solve(m) else { XCTFail("IK returned nil"); return }

        let other = NimbleBridge()
        let path = try XCTUnwrap(
            Bundle(for: type(of: self)).path(forResource: "FullBody", ofType: "osim")
                ?? Bundle.main.path(forResource: "FullBody", ofType: "osim"))
        XCTAssertTrue(other.loadModel(fromPath: path))
        for _ in 0..<2 {
            _ = other.solveIK(withMarkerPositions: standingMarkers.positions,
                              markerNames: standingMarkers.names)
        }
        other.resetSessionState()
        guard let b = other.solveIK(withMarkerPositions: m.positions,
                                    markerNames: m.names) else {
            XCTFail("IK returned nil"); return
        }
        let delta = maxAbsDelta(q(a), q(b))
        print(String(format: "IKCONV-METRIC cross_instance maxAbsDelta=%.6e", delta))
        XCTAssertLessThan(delta, 1e-9, "two fresh bridges must solve identically")
    }

    // MARK: - 4. Continuity

    /// A marker perturbation far below the pose-source noise floor must move
    /// the pose correspondingly little. This is the property the SG stage
    /// depends on: it differentiates `q` twice, with gain ~1/dt².
    func testSmallMarkerPerturbationMovesThePoseSmoothly() throws {
        try loadFullBody()
        let m = dancerMarkers
        _ = solve(m)
        guard let a = solve(m) else { XCTFail("IK returned nil"); return }

        var shifted = m.positions
        for i in stride(from: 0, to: shifted.count, by: 3) {
            shifted[i] = NSNumber(value: shifted[i].doubleValue + 0.002)
        }
        guard let b = bridge.solveIK(withMarkerPositions: shifted,
                                     markerNames: m.names) else {
            XCTFail("IK returned nil"); return
        }
        let delta = maxAbsDelta(q(a), q(b))
        print(String(format: "IKCONV-METRIC continuity maxAbsDelta=%.6e", delta))
        XCTAssertLessThan(delta, 0.15,
                          "a 2 mm rigid marker shift must not jump the pose to another basin")
    }

    // MARK: - 5. Cost

    /// The solve has to stay affordable — the shipped model is 520 muscles /
    /// 169 coordinates and already costs ~200 ms/frame end to end. A converged
    /// warm solve should be CHEAPER than the old one, not dearer, because it
    /// starts at its own fixed point.
    func testWarmSolveCostDoesNotRegress() throws {
        try loadFullBody()
        let m = dancerMarkers
        _ = solve(m)   // cold
        let t0 = CFAbsoluteTimeGetCurrent()
        for _ in 0..<10 { _ = solve(m) }
        let msPerWarmSolve = (CFAbsoluteTimeGetCurrent() - t0) * 100.0
        print(String(format: "IKCONV-METRIC warm_solve_ms=%.2f", msPerWarmSolve))
    }

    /// Identical markers are the best case — the solve exits on its first
    /// convergence test. The live path never sees that, so cost is also
    /// measured on a MOVING subject, where every frame is a real re-solve.
    ///
    /// This is a Debug simulator build; nimble's own solver ran inside a
    /// prebuilt Release static library while this loop's linear algebra is
    /// compiled at -O0, so the ratio here is pessimistic for the new code, not
    /// optimistic. The number worth watching across changes is frames/second,
    /// not the absolute milliseconds.
    func testMovingSubjectSolveCost() throws {
        try loadFullBody()
        let m = dancerMarkers
        _ = solve(m)   // cold

        // A 1 cm/frame sinusoidal sway of the whole marker cloud plus a 2 cm
        // swing of the free leg: motion well above the pose source's noise
        // floor, so the solver cannot coast.
        let legMarkers: Set<String> = ["LHJC", "LKJC", "LAJC", "LTOE"]
        var totalIterations = 0
        let frames = 20
        let t0 = CFAbsoluteTimeGetCurrent()
        for k in 0..<frames {
            let phase = Double(k) * 0.31
            var pos = m.positions
            for (i, name) in m.names.enumerated() {
                pos[i * 3 + 0] = NSNumber(value: pos[i * 3 + 0].doubleValue + 0.01 * sin(phase))
                if legMarkers.contains(name) {
                    pos[i * 3 + 1] = NSNumber(value: pos[i * 3 + 1].doubleValue + 0.02 * cos(phase))
                }
            }
            if let r = bridge.solveIK(withMarkerPositions: pos, markerNames: m.names) {
                totalIterations += r.iterations
            }
        }
        let msPerFrame = (CFAbsoluteTimeGetCurrent() - t0) * 1000.0 / Double(frames)
        print(String(format: "IKCONV-METRIC moving_solve_ms_per_frame=%.2f mean_iterations=%.1f",
                     msPerFrame, Double(totalIterations) / Double(frames)))
    }
}
