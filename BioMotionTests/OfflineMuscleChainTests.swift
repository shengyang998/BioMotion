import XCTest
@testable import BioMotion

/// Reproduces the offline photo path's biomechanics chain stage by stage, to
/// find where muscle output stops being produced.
///
/// On device, a single imported photo reports "Pose only (warming up)" and
/// "0 with muscle data" — the skeleton renders but no muscle ever appears. The
/// skeleton and the muscle solve do NOT share an input: the skeleton is drawn
/// straight from `BodyFrame.joints`, whereas muscle output requires
/// IK -> Savitzky-Golay warm-up -> ID -> moment arms -> QP. A visible skeleton
/// therefore says nothing about any of those stages.
///
/// The marker positions below are the real Core ML output for the upstream demo
/// image, produced by `labs/sam-3d-body/export/e2e_check.py --swift` and passed
/// through `MHRRetarget`. They are a genuinely extreme pose (a dancer with a
/// raised leg), which is representative of what the model emits, not a
/// hand-made T-pose that would hide pose-dependent failures.
final class OfflineMuscleChainTests: XCTestCase {

    private var bridge: NimbleBridge!
    private var computer: MomentArmComputer!
    private var solver: MuscleSolver!

    override func setUp() {
        super.setUp()
        bridge = NimbleBridge()
        computer = MomentArmComputer()
        solver = MuscleSolver()
    }

    private func loadFullBody() throws {
        let path = try XCTUnwrap(
            Bundle(for: type(of: self)).path(forResource: "FullBody", ofType: "osim")
                ?? Bundle.main.path(forResource: "FullBody", ofType: "osim"),
            "Cannot find FullBody.osim")
        XCTAssertTrue(bridge.loadModel(fromPath: path), "bridge failed to load FullBody.osim")
        XCTAssertTrue(computer.parseMusclePaths(fromOsimPath: path, from: bridge),
                      "MomentArmComputer failed to parse muscle paths")
        XCTAssertTrue(solver.loadMuscles(fromOsimPath: path),
                      "MuscleSolver failed to load muscles")
    }

    /// Walks the whole chain and reports the first stage that fails, so the
    /// failure message names the culprit instead of just "no muscle".
    func testPhotoPathProducesMuscleOutput() throws {
        try loadFullBody()

        var positions: [NSNumber] = []
        var names: [String] = []
        for (arkitId, opensim, p) in OfflineMuscleChainFixture.markers {
            XCTAssertNotNil(JointMapping.primary.first { $0.arkitName == arkitId },
                            "\(arkitId) missing from JointMapping.primary")
            names.append(opensim)
            positions.append(NSNumber(value: Double(p.x)))
            positions.append(NSNumber(value: Double(p.y)))
            positions.append(NSNumber(value: Double(p.z)))
        }
        print("CHAIN-METRIC markers_mapped=\(names.count)")

        // --- Stage 1: IK -------------------------------------------------
        let ik = try XCTUnwrap(bridge.solveIK(withMarkerPositions: positions, markerNames: names),
                               "STAGE 1 FAILED: solveIK returned nil")
        let numDOFs = ik.jointAngles.count
        print("CHAIN-METRIC ik_dofs=\(numDOFs) ik_error_m=\(ik.error)")
        XCTAssertGreaterThan(numDOFs, 0)

        // --- Stage 2: Savitzky-Golay warm-up -----------------------------
        // The offline runner pushes 4 head-pad + 1 real + 4 tail-pad = 9.
        // Replay exactly that and confirm the filter actually emits.
        var filters = (0..<numDOFs).map { _ in SavitzkyGolayFilter() }
        var smoothedQ: [Double] = []
        var smoothedDQ: [Double] = []
        var smoothedDDQ: [Double] = []
        let dt = 0.5  // the 2 fps default
        for push in 0..<SavitzkyGolayFilter.windowSize {
            smoothedQ.removeAll(); smoothedDQ.removeAll(); smoothedDDQ.removeAll()
            for i in 0..<numDOFs {
                let q = ik.jointAngles[i].doubleValue
                if let out = filters[i].push(q, timestamp: Double(push) * dt) {
                    smoothedQ.append(out.pos)
                    smoothedDQ.append(out.vel)
                    smoothedDDQ.append(out.acc)
                }
            }
        }
        print("CHAIN-METRIC sg_emitted_dofs=\(smoothedQ.count) of \(numDOFs) after \(SavitzkyGolayFilter.windowSize) pushes")
        XCTAssertEqual(smoothedQ.count, numDOFs,
                       "STAGE 2 FAILED: Savitzky-Golay did not warm up after \(SavitzkyGolayFilter.windowSize) pushes")

        // --- Stage 3: inverse dynamics -----------------------------------
        let id = try XCTUnwrap(
            bridge.solveIDGRF(withJointAngles: smoothedQ.map { NSNumber(value: $0) },
                              jointVelocities: smoothedDQ.map { NSNumber(value: $0) },
                              jointAccelerations: smoothedDDQ.map { NSNumber(value: $0) }),
            "STAGE 3 FAILED: solveIDGRF returned nil (DOF count mismatch is the only non-fallback nil path)")
        let torqueMags = id.jointTorques.map { abs($0.doubleValue) }
        let maxTorque = torqueMags.max() ?? 0
        print("CHAIN-METRIC id_torques=\(id.jointTorques.count) max_torque_Nm=\(maxTorque) leftContact=\(id.leftFootInContact) rightContact=\(id.rightFootInContact)")
        // With no foot in contact, solveIDGRF falls back to plain inverse
        // dynamics with ZERO external force, so bodyweight has to be carried
        // entirely by joint torques. That inflates the torques the muscle QP is
        // asked to match, and muscles slam into their bounds.
        print("CHAIN-METRIC groundHeightY=\(bridge.groundHeightY)")

        // Where the torque actually lands, and whether the pose is even in
        // static equilibrium. The edge padding replays one pose, so dq/ddq are
        // ~0 and tau reduces to gravity minus GRF. A large root residual means
        // no set of joint torques can hold this pose against gravity with the
        // GRF we inferred — i.e. the pose is not statically balanced, and every
        // magnitude downstream of it is meaningless regardless of the solver.
        let ranked = zip(ik.dofNames, torqueMags).sorted { $0.1 > $1.1 }.prefix(6)
        for (name, t) in ranked { print("CHAIN-METRIC torque \(name)=\(Int(t)) Nm") }
        print("CHAIN-METRIC rootResidualNorm=\(id.rootResidualNorm) totalMassKg=\(bridge.totalMass)")
        let dqMax = smoothedDQ.map { abs($0) }.max() ?? 0
        let ddqMax = smoothedDDQ.map { abs($0) }.max() ?? 0
        print("CHAIN-METRIC max_dq=\(dqMax) max_ddq=\(ddqMax)")

        // Distal torques that grow down the support leg point at the GRF being
        // applied through too long a lever. Compare the centre of pressure with
        // the ankle it is supposed to act near: bodyweight times the horizontal
        // offset between them is the ankle moment the GRF alone imposes.
        let f = id.rightFootForce.map { $0.doubleValue }
        let cop = id.rightFootCoP.map { $0.doubleValue }
        let ankle = OfflineMuscleChainFixture.markers.first { $0.0 == "right_foot_joint" }!.2
        let lever = ((cop[0] - Double(ankle.x)) * (cop[0] - Double(ankle.x))
                   + (cop[2] - Double(ankle.z)) * (cop[2] - Double(ankle.z))).squareRoot()
        let fMag = (f[0]*f[0] + f[1]*f[1] + f[2]*f[2]).squareRoot()
        print("CHAIN-METRIC grf_N=\(Int(fMag)) bodyweight_N=\(Int(bridge.totalMass * 9.81))")
        print("CHAIN-METRIC cop=(\(cop[0]),\(cop[1]),\(cop[2])) ankle=(\(ankle.x),\(ankle.y),\(ankle.z))")
        print("CHAIN-METRIC cop_to_ankle_horizontal_m=\(lever) implied_ankle_moment_Nm=\(fMag * lever)")

        // --- Stage 4: moment arms ----------------------------------------
        let dofNames = ik.dofNames
        let momentArms = computer.computeMomentArms(
            withJointAngles: smoothedQ.map { NSNumber(value: $0) },
            dofNames: dofNames) ?? []
        print("CHAIN-METRIC moment_arms=\(momentArms.count) muscles=\(computer.muscleNames.count)")
        XCTAssertFalse(momentArms.isEmpty,
                       "STAGE 4 FAILED: computeMomentArms returned empty — NimbleEngine gates the muscle solve on this")
        XCTAssertGreaterThan(computer.muscleNames.count, 0, "STAGE 4 FAILED: no muscle names")

        // --- Stage 5: muscle QP ------------------------------------------
        // `NimbleIDResult.jointTorques` is positional (one entry per DOF, in
        // skeleton order), not keyed by name.
        let torqueVals = id.jointTorques
        let muscle = try XCTUnwrap(
            solver.solveReal(withJointTorques: torqueVals,
                             momentArms: momentArms,
                             muscleNames: computer.muscleNames,
                             muscleLengths: computer.currentMuscleLengths,
                             maxForces: computer.maxIsometricForces,
                             optimalFiberLengths: computer.optimalFiberLengths,
                             tendonSlackLengths: computer.tendonSlackLengths,
                             pennationAngles: computer.pennationAngles,
                             jointVelocities: smoothedDQ.map { NSNumber(value: $0) },
                             dofNames: dofNames,
                             dt: dt,
                             softPenalty: 100.0),
            "STAGE 5 FAILED: solveReal returned nil")
        print("CHAIN-METRIC activations=\(muscle.activations.count) converged=\(muscle.converged)")
        XCTAssertGreaterThan(muscle.activations.count, 0, "STAGE 5 FAILED: no activations")
    }
}
