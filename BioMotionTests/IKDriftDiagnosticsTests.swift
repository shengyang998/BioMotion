import XCTest
@testable import BioMotion

/// TEMPORARY DIAGNOSTIC — root-causing `testRepeatedIKOnIdenticalMarkersIsStable`.
///
/// These tests assert almost nothing; they print machine-parseable lines prefixed
/// with `IKDIAG|` so the trajectory of repeated warm-started IK solves on
/// *identical* marker input can be reconstructed from the xcodebuild log.
///
/// Delete once the drift is fixed (or converted into a real regression test).
final class IKDriftDiagnosticsTests: XCTestCase {

    private var bridge: NimbleBridge!

    override func setUp() {
        super.setUp()
        bridge = NimbleBridge()
    }

    // MARK: - Fixtures

    /// The ORIGINAL fixture from NimbleBridgeTests (planar, z = 0 everywhere).
    private static let planarLayout: [(String, Double, Double, Double)] = [
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

    /// The "more three-dimensional" fixture described in the STATUS.md note:
    /// soft-knee z-offset plus toe markers.
    private static let threeDLayout: [(String, Double, Double, Double)] = [
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

    private func flatten(_ layout: [(String, Double, Double, Double)])
        -> (positions: [NSNumber], names: [String]) {
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

    private func loadModel(_ resource: String = "Rajagopal2016") {
        let path = Bundle(for: type(of: self)).path(forResource: resource, ofType: "osim")
            ?? Bundle.main.path(forResource: resource, ofType: "osim")
        guard let path else {
            XCTFail("Cannot find \(resource).osim")
            return
        }
        XCTAssertTrue(bridge.loadModel(fromPath: path))
    }

    // MARK: - Trajectory recorder

    private struct Solve {
        let q: [Double]
        let error: Double
    }

    private func runTrajectory(label: String,
                               layout: [(String, Double, Double, Double)],
                               count: Int,
                               on b: NimbleBridge) -> [Solve] {
        let markers = flatten(layout)
        var out: [Solve] = []
        for _ in 0..<count {
            guard let r = b.solveIK(withMarkerPositions: markers.positions,
                                    markerNames: markers.names) else {
                XCTFail("[\(label)] IK returned nil")
                return out
            }
            out.append(Solve(q: r.jointAngles.map { $0.doubleValue }, error: r.error))
        }
        return out
    }

    private func report(_ label: String, _ solves: [Solve], dofNames: [String]) {
        guard solves.count > 1 else { return }
        var prevDelta: [Double] = []
        for n in 1..<solves.count {
            let a = solves[n - 1].q, b = solves[n].q
            let delta = zip(a, b).map { $1 - $0 }
            let maxDelta = delta.map { abs($0) }.max() ?? 0
            let argmax = delta.map { abs($0) }.firstIndex(of: maxDelta) ?? 0
            let l2 = sqrt(delta.reduce(0) { $0 + $1 * $1 })
            var cosPrev = Double.nan
            if !prevDelta.isEmpty {
                let dot = zip(prevDelta, delta).reduce(0) { $0 + $1.0 * $1.1 }
                let n1 = sqrt(prevDelta.reduce(0) { $0 + $1 * $1 })
                let n2 = l2
                if n1 > 0 && n2 > 0 { cosPrev = dot / (n1 * n2) }
            }
            prevDelta = delta
            let name = argmax < dofNames.count ? dofNames[argmax] : "dof\(argmax)"
            print(String(format: "IKDIAG|%@|n=%d|err=%.12e|dErr=%.6e|maxDelta=%.6e|l2=%.6e|cosPrev=%.6f|argmax=%@",
                         label, n, solves[n].error,
                         solves[n].error - solves[n - 1].error,
                         maxDelta, l2, cosPrev, name))
        }
        // Cumulative displacement from the first solve.
        let first = solves[0].q, last = solves[solves.count - 1].q
        let cum = zip(first, last).map { abs($1 - $0) }.max() ?? 0
        print(String(format: "IKDIAG|%@|SUMMARY|n=%d|err0=%.12e|errN=%.12e|cumMaxDrift=%.6e",
                     label, solves.count, solves[0].error, last.isEmpty ? 0 : solves[solves.count - 1].error, cum))
    }

    // MARK: - Experiments

    /// E0. Environment: how many DOFs / markers are actually in play, and does
    /// the cold-restart fallback (`NimbleBridge.mm:710`) fire on this fixture?
    func testDiag00Environment() {
        loadModel()
        let names = bridge.markerNames
        let placed = Self.planarLayout.filter { names.contains($0.0) }
        print("IKDIAG|ENV|numDOFs=\(bridge.numDOFs)|modelMarkers=\(names.count)|fixtureMarkersPlaced=\(placed.count)/\(Self.planarLayout.count)")
        let placed3D = Self.threeDLayout.filter { names.contains($0.0) }
        print("IKDIAG|ENV|fixture3DPlaced=\(placed3D.count)/\(Self.threeDLayout.count)")

        // Cold solve (fresh bridge, no warm start).
        XCTAssertFalse(bridge.ikWarmStartAvailable)
        let markers = flatten(Self.planarLayout)
        let cold = bridge.solveIK(withMarkerPositions: markers.positions, markerNames: markers.names)
        let rejectBound = Double(placed.count) * 0.15 * 0.15
        print(String(format: "IKDIAG|ENV|coldError=%.12e|warmRejectBound=%.6e|wouldRejectWarm=%@",
                     cold?.error ?? -1, rejectBound,
                     (cold?.error ?? 0) > rejectBound ? "YES" : "NO"))
        print("IKDIAG|ENV|dofNames=\(bridge.dofNames.joined(separator: ","))")
    }

    /// E1. 80 warm solves on the ORIGINAL planar fixture.
    func testDiag01PlanarTrajectory() {
        loadModel()
        let s = runTrajectory(label: "PLANAR", layout: Self.planarLayout, count: 80, on: bridge)
        report("PLANAR", s, dofNames: bridge.dofNames)
    }

    /// E2. 80 warm solves on the "more 3D" fixture (the 0.19 / 0.84 rad case).
    func testDiag02ThreeDTrajectory() {
        loadModel()
        let s = runTrajectory(label: "THREED", layout: Self.threeDLayout, count: 80, on: bridge)
        report("THREED", s, dofNames: bridge.dofNames)
    }

    /// E3. Determinism + cross-instance state leakage.
    /// Two independently constructed bridges run the identical sequence.
    func testDiag03DeterminismAcrossInstances() {
        loadModel()
        let a = runTrajectory(label: "DETA", layout: Self.planarLayout, count: 6, on: bridge)

        let b2 = NimbleBridge()
        let path = Bundle(for: type(of: self)).path(forResource: "Rajagopal2016", ofType: "osim")
            ?? Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim")
        XCTAssertTrue(b2.loadModel(fromPath: path!))
        let b = runTrajectory(label: "DETB", layout: Self.planarLayout, count: 6, on: b2)

        guard a.count == b.count, a.count > 0 else { XCTFail("no solves"); return }
        var worst = 0.0
        for i in 0..<a.count {
            let d = zip(a[i].q, b[i].q).map { abs($0 - $1) }.max() ?? 0
            worst = max(worst, d)
            print(String(format: "IKDIAG|DETERMINISM|i=%d|maxAbsDiff=%.6e|errA=%.12e|errB=%.12e",
                         i, d, a[i].error, b[i].error))
        }
        print(String(format: "IKDIAG|DETERMINISM|SUMMARY|worstAbsDiff=%.6e|bitIdentical=%@",
                     worst, worst == 0 ? "YES" : "NO"))
    }

    /// E4. Does it EVER converge? 400 warm solves; drift sampled on a log grid.
    func testDiag04LongRunConvergence() {
        loadModel()
        let s = runTrajectory(label: "LONG", layout: Self.planarLayout, count: 400, on: bridge)
        guard s.count == 400 else { XCTFail("short run"); return }
        for n in [1, 2, 5, 10, 20, 50, 100, 200, 399] {
            let d = zip(s[n - 1].q, s[n].q).map { abs($0 - $1) }.max() ?? 0
            print(String(format: "IKDIAG|LONG|probe|n=%d|maxDelta=%.6e|err=%.12e", n, d, s[n].error))
        }
        let cum = zip(s[0].q, s[399].q).map { abs($1 - $0) }.max() ?? 0
        print(String(format: "IKDIAG|LONG|SUMMARY|cumMaxDrift=%.6e|err0=%.12e|err399=%.12e|errDrop=%.6e",
                     cum, s[0].error, s[399].error, s[0].error - s[399].error))
    }

    /// E5. Warm-start seeding check: after the pose has drifted, does an
    /// explicit `resetSessionState` (which forces a COLD 5-restart solve)
    /// return to the same place, or somewhere else entirely?
    func testDiag05ColdRestartAfterDrift() {
        loadModel()
        let s = runTrajectory(label: "PRE", layout: Self.planarLayout, count: 40, on: bridge)
        guard let drifted = s.last else { XCTFail(); return }
        bridge.resetSessionState()
        XCTAssertFalse(bridge.ikWarmStartAvailable)
        let markers = flatten(Self.planarLayout)
        guard let cold = bridge.solveIK(withMarkerPositions: markers.positions,
                                        markerNames: markers.names) else { XCTFail(); return }
        let coldQ = cold.jointAngles.map { $0.doubleValue }
        let d = zip(drifted.q, coldQ).map { abs($0 - $1) }.max() ?? 0
        print(String(format: "IKDIAG|COLDAFTERDRIFT|maxAbsDiff=%.6e|warmErr=%.12e|coldErr=%.12e",
                     d, drifted.error, cold.error))
    }
}
