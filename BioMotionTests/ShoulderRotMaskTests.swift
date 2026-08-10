import XCTest
@testable import BioMotion

/// STATUS.md next-step 8 asked for `shoulder_rot_{r,l}` (axial humeral rotation)
/// to be added to the runtime DOF mask, because they are "structurally
/// unobservable from one marker per shoulder plus one at the elbow" and this
/// project has recorded that unobservable coordinates get excited by the solver.
///
/// ─────────────────────────────────────────────────────────────────────────
/// VERDICT: MEASURED, AND NOT ADOPTED. The premise is false.
/// ─────────────────────────────────────────────────────────────────────────
/// The marker set has one point per shoulder (LSJC / RSJC, at the `humerus_*`
/// body ORIGIN — i.e. on the rotation axis), one at the elbow (LEJC / REJC, at
/// the `ulna_*` origin) and one at the wrist (LWJC / RWJC, at `hand_*`). The
/// argument for unobservability assumes the forearm markers lie on the humeral
/// long axis when the elbow is straight. In FullBody.osim they do not: the ulna
/// and hand origins are offset from that axis, so axial rotation swings them
/// even at zero elbow flexion.
///
/// Measured (2026-08-07, `ShoulderRotObservabilityTests.mm` for the Jacobian,
/// this file for the behaviour):
///
///   ‖J[:, shoulder_rot_r]‖   0.0343 m/rad at neutral   (NOT null)
///   ‖J[:, shoulder_elv_r]‖   0.6077 m/rad              (17.7x more sensitive)
///   ‖J[:, elbow_flex_r]‖     0.2507 m/rad
///   d REJC / d shoulder_rot_r   16.3 mm/rad  at 0° elbow flexion
///   d RWJC / d shoulder_rot_r   30.2 mm/rad  at 0° elbow flexion
///   ‖J[:, shoulder_rot_r]‖   0.266 m/rad at 90° elbow flexion (7.8x)
///
///   dancer   marker RMS   1.53645 -> 2.25348 cm  (masking COSTS 0.71703 cm)
///   dancer   rel. torque residual 0.59395 -> 0.50643 (still unusable; lower
///                                                        does not restore lost markers)
///   standing marker RMS   unchanged to 4.1e-5 cm (nothing to remove: the
///                         unmasked solver puts 0.04° into the coordinate)
///   standing iterations   0 -> 123, converged YES -> NO,
///                         per-solve drift 0 -> 9.3e-5 rad
///
/// So the coordinate is WEAKLY OBSERVED, not unobservable, and pinning it
/// trades accuracy on the pose where it matters while breaking the fixed-point
/// property on the pose where it does not. The tests below assert that verdict:
/// they fail if someone turns the mask on, and they also fail if the underlying
/// measurements change enough to make it worth revisiting.
///
/// ─────────────────────────────────────────────────────────────────────────
/// WHY THE STANDING SOLVE STOPS CONVERGING UNDER THE MASK (root cause)
/// ─────────────────────────────────────────────────────────────────────────
/// Traced with `kIKTraceSolve`. Unmasked, both LM phases exit on the gradient
/// test with `iters=0` at loss 2.0390e-8 — a genuine interior stationary point.
/// Masked, phase A still exits on the gradient at 6 iterations, but phase B
/// (mu = 0) hits the 120-iteration cap with the loss creeping 2.07421885e-8 ->
/// 2.07421803e-8 -> 2.07421721e-8, i.e. ~8e-16 per full solve. Removing the
/// coordinate that was absorbing the fixture's fourth-decimal rounding leaves a
/// descent direction whose curvature is far below `kIKConditionFloorRel`
/// (1e-6 x max diag JᵀJ), so the damped Newton step degenerates into a
/// gradient step of size ~1/floor and the solver creeps. It is genuinely still
/// moving, so neither the 1e-12 gradient test nor the 1e-9 step test should
/// fire — the tolerances are not the problem, the extra pin is.
///
/// Every number is printed with a `SHROT-METRIC` prefix so a run can be diffed
/// against an earlier one from the xcodebuild log alone.
final class ShoulderRotMaskTests: XCTestCase {

    /// The two coordinates under test. Deliberately NOT exported from app code:
    /// nothing in the app applies this mask, and an unused constant there would
    /// read as "someone forgot to wire it up".
    static let shoulderRotationMask = ["shoulder_rot_r", "shoulder_rot_l"]

    private var bridge: NimbleBridge!

    override func setUp() {
        super.setUp()
        continueAfterFailure = true   // every metric must print, even after a failure
        bridge = NimbleBridge()
    }

    // MARK: - Fixtures

    /// The real Core ML dancer pose — same 20 markers `OfflineMuscleChainTests`,
    /// `IKConvergenceTests` and `MuscleQPUnitsTests` use.
    private var dancerMarkers: (positions: [NSNumber], names: [String]) {
        flatten(OfflineMuscleChainFixture.markers.map {
            ($0.1, SIMD3<Double>(Double($0.2.x), Double($0.2.y), Double($0.2.z)))
        })
    }

    /// The 20-marker upright standing pose, built from FullBody.osim's own
    /// neutral forward kinematics so it is exactly reachable. Copied from
    /// `StaticEquilibriumBenchmarkTests` (private there) — the arm rows are the
    /// ones that matter here: arms hanging at the sides, elbows EXTENDED, which
    /// is the configuration in which axial humeral rotation should be invisible.
    private static let standingLayout: [(String, Double, Double, Double)] = {
        let ankleY = 0.0750, shank = 0.4000, thigh = 0.4060, rise = 0.0785
        let kneeY = ankleY + shank
        let hipY = kneeY + thigh
        let pelvisY = hipY + rise
        var rows: [(String, Double, Double, Double)] = [
            ("RTOE",   0.0673, 0.0310,  0.0860), ("LTOE",   0.0673, 0.0310, -0.0860),
            ("RAJC",  -0.0627, ankleY,  0.0770), ("LAJC",  -0.0627, ankleY, -0.0770),
            ("RKJC",  -0.0527, kneeY,   0.0770), ("LKJC",  -0.0527, kneeY,  -0.0770),
            ("RHJC",  -0.0563, hipY,    0.0773), ("LHJC",  -0.0563, hipY,   -0.0773),
            ("PELVIS", 0.0000, pelvisY, 0.0000),
        ]
        // Trunk / head / arm offsets from the pelvis origin, from the model's
        // neutral FK.
        let trunk: [(String, Double, Double, Double)] = [
            ("SPINE_L", -0.1019,  0.1447,  0.0000),
            ("SPINE_M", -0.1595,  0.3849,  0.0000),
            ("C7",      -0.1173,  0.5217,  0.0000),
            ("NECK",    -0.1130,  0.5426,  0.0000),
            ("HEAD",    -0.1130,  0.6926,  0.0000),
            ("RSJC",    -0.0997,  0.4715,  0.1706),
            ("LSJC",    -0.0997,  0.4715, -0.1706),
            ("REJC",    -0.0865,  0.1853,  0.1610),
            ("LEJC",    -0.0865,  0.1853, -0.1610),
            ("RWJC",    -0.1020, -0.0636,  0.2007),
            ("LWJC",    -0.1020, -0.0636, -0.2007),
        ]
        for (n, dx, dy, dz) in trunk { rows.append((n, dx, pelvisY + dy, dz)) }
        return rows
    }()

    private var standingMarkers: (positions: [NSNumber], names: [String]) {
        flatten(Self.standingLayout.map { ($0.0, SIMD3<Double>($0.1, $0.2, $0.3)) })
    }

    private func flatten(_ rows: [(String, SIMD3<Double>)]) -> (positions: [NSNumber], names: [String]) {
        var positions: [NSNumber] = []
        var names: [String] = []
        for (n, p) in rows {
            names.append(n)
            positions.append(NSNumber(value: p.x))
            positions.append(NSNumber(value: p.y))
            positions.append(NSNumber(value: p.z))
        }
        return (positions, names)
    }

    private func loadFullBody() throws -> String {
        let path = try XCTUnwrap(
            Bundle(for: type(of: self)).path(forResource: "FullBody", ofType: "osim")
                ?? Bundle.main.path(forResource: "FullBody", ofType: "osim"),
            "Cannot find FullBody.osim")
        XCTAssertTrue(bridge.loadModel(fromPath: path), "bridge failed to load FullBody.osim")
        return path
    }

    // MARK: - One measured arm

    private struct Arm {
        var rmsCm: Double
        var maxCm: Double
        var iterations: Int
        var converged: Bool
        /// |q| of each of the six shoulder coordinates, radians.
        var shoulder: [String: Double]
        /// Per-marker error, cm, for the six arm markers.
        var armMarkerCm: [String: Double]
        /// max |Δq| over two consecutive solves on IDENTICAL markers.
        var repeatDriftRad: Double
        var freeDOFs: Int
    }

    private static let shoulderCoordinates = [
        "elv_angle_r", "shoulder_elv_r", "shoulder_rot_r",
        "elv_angle_l", "shoulder_elv_l", "shoulder_rot_l",
    ]
    private static let armMarkers = ["LSJC", "RSJC", "LEJC", "REJC", "LWJC", "RWJC"]

    /// Solves `m` from a clean session with `mask` applied (nil = unmasked) and
    /// reports everything that could plausibly move.
    ///
    /// The mask is applied straight after `resetSessionState`, i.e. before any
    /// solve in this arm, so the pinned value is the coordinate's model default
    /// rather than whatever a previous pose left behind. That is what the app
    /// would do (mask once at model load).
    private func measure(_ m: (positions: [NSNumber], names: [String]),
                         mask: [String]?,
                         label: String) -> Arm {
        bridge.clearDOFMask()
        bridge.resetSessionState()
        if let mask { _ = bridge.applyDOFMask(withNames: mask) }

        guard let cold = bridge.solveIK(withMarkerPositions: m.positions, markerNames: m.names),
              let warm = bridge.solveIK(withMarkerPositions: m.positions, markerNames: m.names),
              let warm2 = bridge.solveIK(withMarkerPositions: m.positions, markerNames: m.names)
        else {
            XCTFail("[\(label)] solveIK returned nil")
            return Arm(rmsCm: .nan, maxCm: .nan, iterations: -1, converged: false,
                       shoulder: [:], armMarkerCm: [:], repeatDriftRad: .nan, freeDOFs: 0)
        }
        _ = cold

        let names = bridge.dofNames
        var shoulder: [String: Double] = [:]
        for (i, n) in names.enumerated() where Self.shoulderCoordinates.contains(n) {
            shoulder[n] = warm2.jointAngles[i].doubleValue
        }
        var armMarkerCm: [String: Double] = [:]
        for n in Self.armMarkers {
            if let e = warm2.markerErrorsMeters[n]?.doubleValue { armMarkerCm[n] = e * 100 }
        }
        let a = warm.jointAngles.map { $0.doubleValue }
        let b = warm2.jointAngles.map { $0.doubleValue }
        let drift = zip(a, b).map { abs($0 - $1) }.max() ?? .infinity

        return Arm(rmsCm: warm2.markerRMSMeters * 100,
                   maxCm: warm2.markerMaxErrorMeters * 100,
                   iterations: warm2.iterations,
                   converged: warm2.converged,
                   shoulder: shoulder,
                   armMarkerCm: armMarkerCm,
                   repeatDriftRad: drift,
                   freeDOFs: bridge.numFreeDOFs)
    }

    private func report(_ a: Arm, _ tag: String) {
        print("SHROT-METRIC [\(tag)] rms_cm=\(a.rmsCm) max_cm=\(a.maxCm) " +
              "iters=\(a.iterations) converged=\(a.converged) " +
              "free_dofs=\(a.freeDOFs) repeat_drift_rad=\(a.repeatDriftRad)")
        for n in Self.shoulderCoordinates {
            print("SHROT-METRIC [\(tag)] q_\(n)=\(a.shoulder[n] ?? .nan)")
        }
        for n in Self.armMarkers {
            print("SHROT-METRIC [\(tag)] marker_\(n)_cm=\(a.armMarkerCm[n] ?? .nan)")
        }
    }

    // MARK: - 1. What the unmasked solver puts into shoulder_rot

    /// Records the excursion first, with no mask anywhere, because "the solver
    /// excites unobservable DOFs" is the premise the mask rests on and it has
    /// never been measured on these two poses.
    func testUnmaskedShoulderRotExcursion() throws {
        _ = try loadFullBody()
        let dancer = measure(dancerMarkers, mask: nil, label: "dancer/unmasked")
        report(dancer, "dancer/unmasked")
        let standing = measure(standingMarkers, mask: nil, label: "standing/unmasked")
        report(standing, "standing/unmasked")

        for (tag, arm) in [("dancer", dancer), ("standing", standing)] {
            let rot = max(abs(arm.shoulder["shoulder_rot_r"] ?? 0),
                          abs(arm.shoulder["shoulder_rot_l"] ?? 0))
            let elv = max(abs(arm.shoulder["shoulder_elv_r"] ?? 0),
                          abs(arm.shoulder["shoulder_elv_l"] ?? 0))
            print("SHROT-METRIC [\(tag)] unmasked_max_abs_shoulder_rot_rad=\(rot) " +
                  "unmasked_max_abs_shoulder_elv_rad=\(elv) " +
                  "unmasked_max_abs_shoulder_rot_deg=\(rot * 180 / .pi)")
        }
        XCTAssertTrue(dancer.rmsCm.isFinite)
        XCTAssertTrue(standing.rmsCm.isFinite)
    }

    // MARK: - 2. A/B: what the mask actually costs

    /// The decisive measurement, and the pre-registered gate it failed.
    ///
    /// The gate was: adopt the mask only if pinning `shoulder_rot` leaves the
    /// marker fit essentially unchanged on BOTH poses (`Δ RMS < 0.05 cm`). That
    /// bound was not chosen to pass — 0.5 mm is an order of magnitude below the
    /// ~2 cm the dancer's own model mismatch already costs and two orders above
    /// the 0.03 mm the standing pose fits to, so it separates "no information
    /// lost" from "some information lost" on both. The current MHR_ROOT
    /// dancer fails it at 0.717 cm (the legacy PELVIS mapping failed at 0.565).
    ///
    /// This test now asserts the FAILURE, so it is a live tripwire in both
    /// directions: it fires if masking suddenly becomes free (re-open the
    /// question) and it fires if masking becomes even more expensive (the
    /// recorded numbers are stale).
    func testMaskingShoulderRotCostsMarkerFitOnTheDancerAndBuysNothingStanding() throws {
        _ = try loadFullBody()
        let mask = Self.shoulderRotationMask

        var delta: [String: Double] = [:]
        var maskedIterations: [String: Int] = [:]
        var maskedDrift: [String: Double] = [:]
        for (tag, m) in [("dancer", dancerMarkers), ("standing", standingMarkers)] {
            let off = measure(m, mask: nil, label: "\(tag)/unmasked")
            let on = measure(m, mask: mask, label: "\(tag)/masked")
            report(off, "\(tag)/unmasked")
            report(on, "\(tag)/masked")
            delta[tag] = on.rmsCm - off.rmsCm
            maskedIterations[tag] = on.iterations
            maskedDrift[tag] = on.repeatDriftRad
            print("SHROT-METRIC [\(tag)] delta_rms_cm=\(delta[tag]!) " +
                  "delta_max_cm=\(on.maxCm - off.maxCm) " +
                  "delta_iters=\(on.iterations - off.iterations) " +
                  "unmasked_drift=\(off.repeatDriftRad) masked_drift=\(on.repeatDriftRad)")

            XCTAssertEqual(on.freeDOFs, off.freeDOFs - 2,
                           "[\(tag)] the mask must remove exactly the two shoulder_rot coordinates")
        }

        // The dancer pays for the pin. If this ever drops under the gate, the
        // premise behind STATUS.md next-step 8 has become true and the mask is
        // worth re-testing.
        XCTAssertGreaterThan(delta["dancer"]!, 0.05,
                             "masking shoulder_rot no longer costs the dancer marker fit " +
                             "(Δ = \(delta["dancer"]!) cm). The current MHR_ROOT verdict was " +
                             "0.717 cm — re-run the whole A/B and revisit the mask.")

        // Standing gains nothing: the coordinate is already at 0.04° there.
        XCTAssertLessThan(abs(delta["standing"]!), 0.001,
                          "standing marker fit moved by \(delta["standing"]!) cm under the mask")

        // ...and it loses the fixed-point property that the 2026-08-07 IK work
        // established. Printed AND asserted, because a solver change that fixed
        // this would change the trade-off.
        print("SHROT-METRIC standing_masked_iterations=\(maskedIterations["standing"]!) " +
              "standing_masked_drift_rad=\(maskedDrift["standing"]!)")
        XCTAssertGreaterThan(maskedDrift["standing"]!, 1e-9,
                             "the masked standing solve now reaches a fixed point " +
                             "(drift \(maskedDrift["standing"]!) rad). That removes one of the " +
                             "three reasons the mask was rejected — re-derive the verdict.")
    }

    /// Nothing in the app installs a DOF mask. Asserted directly rather than
    /// left to code review, because the whole verdict above is void if some
    /// later change turns one on at load time.
    func testLoadingTheModelInstallsNoDOFMask() throws {
        _ = try loadFullBody()
        XCTAssertFalse(bridge.isDOFMaskActive,
                       "loadModel must not install a DOF mask — see this file's header verdict")
        XCTAssertEqual(bridge.numFreeDOFs, bridge.numDOFs)
        XCTAssertEqual(bridge.maskedDOFNames.count, 0)
    }

    /// A masked coordinate must not move at all, and must sit at the MODEL's
    /// neutral value rather than at whatever a previous pose left in the shared
    /// skeleton.
    ///
    /// The defect this pins, measured 2026-08-07: `applyDOFMaskWithNames:` read
    /// the pin from `_skeleton->getPositions()`, and the skeleton is shared
    /// process-wide, so masking straight after a dancer solve pinned
    /// `shoulder_rot_r` at 0.6235 rad (the dancer's own answer) while the same
    /// call on a freshly loaded model pinned it at 0.
    ///
    /// ⚠️ The two arms below must differ in the work that PRECEDES the mask,
    /// otherwise this test is tautological — an earlier revision of it ran a
    /// dancer solve before both arms and therefore passed against the broken
    /// implementation.
    func testMaskedShoulderRotIsExactlyPinnedAndOrderIndependent() throws {
        _ = try loadFullBody()
        let mask = Self.shoulderRotationMask

        // Arm 1: mask on a model that has solved NOTHING.
        bridge.clearDOFMask()
        bridge.resetSessionState()
        let clean = measure(standingMarkers, mask: mask, label: "standing/masked-clean")

        // Arm 2: park the shared skeleton in an extreme pose first, unmasked,
        // then mask and solve the same standing pose.
        bridge.clearDOFMask()
        bridge.resetSessionState()
        _ = bridge.solveIK(withMarkerPositions: dancerMarkers.positions,
                           markerNames: dancerMarkers.names)
        _ = bridge.solveIK(withMarkerPositions: dancerMarkers.positions,
                           markerNames: dancerMarkers.names)
        let dirty = measure(standingMarkers, mask: mask, label: "standing/masked-after-dancer")

        for n in ["shoulder_rot_r", "shoulder_rot_l"] {
            let d = dirty.shoulder[n] ?? .nan
            let c = clean.shoulder[n] ?? .nan
            print("SHROT-METRIC pin_\(n) after_dancer=\(d) clean=\(c) delta=\(abs(d - c))")
            XCTAssertEqual(d, c, accuracy: 1e-15,
                           "\(n)'s pinned value depends on solve order — the mask is " +
                           "inheriting a previous pose from the shared skeleton")
            XCTAssertEqual(c, 0.0, accuracy: 1e-15,
                           "\(n) must pin at the model's neutral value, not at a solved pose")
        }
        print("SHROT-METRIC standing_masked_rms_cm_after_dancer=\(dirty.rmsCm) clean=\(clean.rmsCm)")
        XCTAssertEqual(dirty.rmsCm, clean.rmsCm, accuracy: 1e-9)
    }

    // MARK: - 3. Downstream: does the muscle stage move?

    /// Masking changes the pose ID and the muscle QP see. This measures by how
    /// much, on both poses, through the same call sequence the photo path uses.
    func testMuscleResidualMaskedVsUnmasked() throws {
        let path = try loadFullBody()
        let computer = MomentArmComputer()
        XCTAssertTrue(computer.parseMusclePaths(fromOsimPath: path, from: bridge))

        for (tag, m) in [("dancer", dancerMarkers), ("standing", standingMarkers)] {
            for (armTag, mask) in [("unmasked", nil), ("masked", Self.shoulderRotationMask)] as [(String, [String]?)] {
                bridge.clearDOFMask()
                bridge.resetSessionState()
                if let mask { _ = bridge.applyDOFMask(withNames: mask) }

                guard let ik = bridge.solveIK(withMarkerPositions: m.positions, markerNames: m.names) else {
                    XCTFail("[\(tag)/\(armTag)] solveIK nil"); continue
                }
                // Savitzky-Golay warm-up on a held pose: q̇ = q̈ = 0, which is
                // how the static-hold path solves.
                let n = ik.jointAngles.count
                var filters = (0..<n).map { _ in SavitzkyGolayFilter() }
                var q: [Double] = [], dq: [Double] = [], ddq: [Double] = []
                for push in 0..<SavitzkyGolayFilter.windowSize {
                    q.removeAll(); dq.removeAll(); ddq.removeAll()
                    for i in 0..<n {
                        if let out = filters[i].push(ik.jointAngles[i].doubleValue,
                                                     timestamp: Double(push) * 0.5) {
                            q.append(out.pos); dq.append(out.vel); ddq.append(out.acc)
                        }
                    }
                }
                guard q.count == n else { XCTFail("[\(tag)/\(armTag)] SG never warmed"); continue }

                guard let id = bridge.solveIDGRF(withJointAngles: q.map { NSNumber(value: $0) },
                                                 jointVelocities: dq.map { NSNumber(value: $0) },
                                                 jointAccelerations: ddq.map { NSNumber(value: $0) }) else {
                    XCTFail("[\(tag)/\(armTag)] solveIDGRF nil"); continue
                }
                guard let arms = computer.computeMomentArms(withJointAngles: q.map { NSNumber(value: $0) },
                                                           dofNames: ik.dofNames) else {
                    XCTFail("[\(tag)/\(armTag)] moment arms nil"); continue
                }

                // Fresh solver per arm: MuscleSolver warm-starts OSQP from the
                // previous activations, so a shared one makes the answer
                // order-dependent (documented in MuscleQPUnitsTests).
                let solver = MuscleSolver()
                XCTAssertTrue(solver.loadMuscles(fromOsimPath: path))
                guard let r = solver.solveReal(withJointTorques: id.jointTorques,
                                               momentArms: arms,
                                               muscleNames: computer.muscleNames,
                                               muscleLengths: computer.currentMuscleLengths,
                                               maxForces: computer.maxIsometricForces,
                                               optimalFiberLengths: computer.optimalFiberLengths,
                                               tendonSlackLengths: computer.tendonSlackLengths,
                                               pennationAngles: computer.pennationAngles,
                                               jointVelocities: dq.map { NSNumber(value: $0) },
                                               dofNames: ik.dofNames,
                                               dt: 0.5,
                                               softPenalty: 100) else {
                    XCTFail("[\(tag)/\(armTag)] muscle solve nil"); continue
                }
                // The two shoulder rows are the ones the mask can plausibly move.
                var shoulderTorque: [String: Double] = [:]
                for (i, dof) in ik.dofNames.enumerated() where Self.shoulderCoordinates.contains(dof) {
                    shoulderTorque[dof] = id.jointTorques[i].doubleValue
                }
                print("SHROT-QP [\(tag)/\(armTag)] relative_torque_residual=\(r.relativeTorqueResidual) " +
                      "torque_residual_Nm=\(r.torqueResidualNm) " +
                      "relative_force_residual=\(r.relativeForceResidual) " +
                      "ik_rms_cm=\(ik.markerRMSMeters * 100)")
                for dof in Self.shoulderCoordinates {
                    print("SHROT-QP [\(tag)/\(armTag)] tau_\(dof)=\(shoulderTorque[dof] ?? .nan)")
                }
            }
        }
    }
}
