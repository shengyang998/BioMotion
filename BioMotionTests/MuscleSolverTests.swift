import XCTest
@testable import BioMotion

final class MuscleSolverTests: XCTestCase {

    private var solver: MuscleSolver!

    override func setUp() {
        super.setUp()
        solver = MuscleSolver()
    }

    // MARK: - Model Loading

    func testLoadMusclesFromOsim() {
        loadMuscles()
        XCTAssertGreaterThan(solver.numMuscles, 0, "Should load muscles")
        // Rajagopal2016 has 80 muscles
        XCTAssertEqual(solver.numMuscles, 80, "Rajagopal2016 should have 80 muscles")
    }

    func testMuscleNames() {
        loadMuscles()
        let names = solver.muscleNames
        XCTAssertEqual(names.count, 80)

        // Check for known muscles
        XCTAssertTrue(names.contains("soleus_r"), "Should contain soleus_r")
        XCTAssertTrue(names.contains("soleus_l"), "Should contain soleus_l")
        XCTAssertTrue(names.contains("recfem_r"), "Should contain recfem_r")
        XCTAssertTrue(names.contains("tibant_r"), "Should contain tibant_r")
        XCTAssertTrue(names.contains("gasmed_r"), "Should contain gasmed_r")
    }

    func testBilateralSymmetry() {
        loadMuscles()
        let names = solver.muscleNames

        // Every _r muscle should have a _l counterpart
        let rightMuscles = names.filter { ($0 as String).hasSuffix("_r") }
        for rMuscle in rightMuscles {
            let lMuscle = (rMuscle as String).replacingOccurrences(of: "_r", with: "_l")
            XCTAssertTrue(names.contains(where: { ($0 as String) == lMuscle }),
                          "Missing left counterpart for \(rMuscle)")
        }
    }

    func testLoadFromInvalidPath() {
        let success = solver.loadMuscles(fromOsimPath: "/nonexistent.osim")
        XCTAssertFalse(success)
        XCTAssertEqual(solver.numMuscles, 0)
    }

    // MARK: - Force-velocity curve

    /// Drives the muscle to maximum shortening velocity through the
    /// wall-clock fallback (no joint velocities supplied) and checks that the
    /// force-velocity multiplier honours its documented contract: zero at max
    /// shortening, never negative.
    ///
    /// The previous curve `1 + v(1 - 0.25v)` crossed zero at v ≈ -0.828 and
    /// returned -0.25 at v = -1, which made forceScale — and therefore the
    /// reported force — negative, telling the QP that activation produces
    /// torque opposite to the muscle's moment arm.
    func testMaxShorteningProducesZeroForceNotNegativeForce() {
        // Frame 1 establishes the length history; no previous frame, so the
        // fallback reports zero velocity here.
        let first = solveSingleMuscle(torque: 10.0, jointVelocity: nil,
                                      muscleLength: 0.35, dt: 0.05)
        XCTAssertNotNil(first, "Empty jointVelocities must select the fallback, not fail")

        // Frame 2: L_MT drops 0.05 m in 0.05 s → dL/dt = -1.0 m/s, and
        // V_max = 10·l_opt = 1.0 m/s, so the normalized velocity is exactly -1.
        guard let second = solveSingleMuscle(torque: 10.0, jointVelocity: nil,
                                             muscleLength: 0.30, dt: 0.05) else {
            XCTFail("Solver returned nil on the fallback velocity path")
            return
        }

        XCTAssertTrue(second.converged)
        let force = second.forces[0].doubleValue
        XCTAssertGreaterThanOrEqual(force, 0.0,
                                    "f_FV must never be negative — a negative force here means "
                                    + "the solver believes activation acts against the moment arm")
        XCTAssertEqual(force, 0.0, accuracy: 1e-9,
                       "f_FV(-1) must be exactly 0 (Hill hyperbola at max shortening)")
        XCTAssertGreaterThanOrEqual(second.activations[0].doubleValue, solver.minActivation)
    }

    /// Force capacity must increase monotonically as shortening slows.
    /// Each speed gets its own solver so no length/warm-start history leaks
    /// between cases.
    func testForceCapacityIncreasesMonotonicallyAsShorteningSlows() {
        // dL/dt = -R·dq, R = 0.05 m, V_max = 1.0 m/s
        //   dq = 20 → ṽ = -1.0 (max shortening)
        //   dq = 10 → ṽ = -0.5
        //   dq =  0 → ṽ =  0.0 (isometric)
        let fastest = forceForJointVelocity(20.0)
        let medium = forceForJointVelocity(10.0)
        let isometric = forceForJointVelocity(0.0)

        for f in [fastest, medium, isometric] {
            XCTAssertGreaterThanOrEqual(f, 0.0, "Muscle force must never be negative")
        }
        XCTAssertLessThan(fastest, medium,
                          "Faster shortening must not deliver more force than slower shortening")
        XCTAssertLessThan(medium, isometric,
                          "Shortening must not deliver more force than isometric")
    }

    // MARK: - Joint velocities drive fiber velocity

    /// The analytic identity is dL_MT/dt = -Rᵀ·dq. With a positive moment
    /// arm, a positive joint velocity SHORTENS the muscle, which lowers its
    /// force capacity and therefore requires MORE activation for the same
    /// torque; a negative joint velocity lengthens it (eccentric, capacity
    /// above isometric) and requires LESS.
    ///
    /// Before the fix `jointVelocities` was ignored entirely, so all three
    /// cases returned the same activation.
    func testJointVelocitySignConventionMatchesMomentArmDefinition() {
        let shortening = activationForJointVelocity(8.0)
        let isometric = activationForJointVelocity(0.0)
        let lengthening = activationForJointVelocity(-20.0)

        XCTAssertGreaterThan(shortening, isometric,
                             "Shortening reduces capacity → needs more activation")
        XCTAssertGreaterThan(isometric, lengthening,
                             "Lengthening raises capacity → needs less activation")
    }

    func testEmptyJointVelocitiesIsAcceptedButWrongLengthIsRejected() {
        XCTAssertNotNil(solveSingleMuscle(torque: 10.0, jointVelocity: nil),
                        "Empty jointVelocities selects the wall-clock fallback")

        // One DOF but two velocities — a caller bug, must be rejected.
        let mismatched = solver.solveReal(
            withJointTorques: [NSNumber(value: 10.0)],
            momentArms: [NSNumber(value: Rig.momentArm)],
            muscleNames: [Rig.muscleName],
            muscleLengths: [NSNumber(value: Rig.isometricLength)],
            maxForces: [NSNumber(value: Rig.maxForce)],
            optimalFiberLengths: [NSNumber(value: Rig.optimalFiberLength)],
            tendonSlackLengths: [NSNumber(value: Rig.tendonSlackLength)],
            pennationAngles: [NSNumber(value: 0.0)],
            jointVelocities: [NSNumber(value: 0.0), NSNumber(value: 0.0)],
            dofNames: [Rig.dofName],
            dt: 1.0 / 60.0,
            softPenalty: Rig.softPenalty
        )
        XCTAssertNil(mismatched, "jointVelocities of the wrong non-zero length must be rejected")
    }

    // MARK: - Torque residual

    /// 10 Nm through a 0.05 m moment arm needs 200 N from a 1000 N muscle —
    /// comfortably achievable, so the soft-equality term should be satisfied.
    func testResidualIsSmallWhenTorqueIsAchievable() {
        guard let result = solveSingleMuscle(torque: 10.0, jointVelocity: 0.0) else {
            XCTFail("Solver returned nil")
            return
        }
        XCTAssertTrue(result.converged)
        XCTAssertLessThan(result.torqueResidualNm, 1.0)
        XCTAssertLessThan(result.relativeTorqueResidual, 0.05,
                          "Achievable torque must be reproduced by the activations")
    }

    /// 500 Nm needs 10 kN from a 1000 N muscle. OSQP still converges — on the
    /// regularized objective — but the physics is nowhere near satisfied.
    /// This is exactly the case `converged` cannot report and the residual can.
    func testResidualIsLargeWhenTorqueIsUnachievable() {
        guard let result = solveSingleMuscle(torque: 500.0, jointVelocity: 0.0) else {
            XCTFail("Solver returned nil")
            return
        }
        XCTAssertTrue(result.converged,
                      "The QP converges even though the torque cannot be produced")
        XCTAssertEqual(result.activations[0].doubleValue, 1.0, accuracy: 1e-3,
                       "Activation should saturate at its upper bound")
        XCTAssertGreaterThan(result.torqueResidualNm, 100.0)
        XCTAssertGreaterThan(result.relativeTorqueResidual, 0.5,
                             "An unsatisfiable torque must be visible in the relative residual")
    }

    // MARK: - Activation bound

    func testMinActivationIsExposedAndEnforced() {
        XCTAssertEqual(solver.minActivation, 0.02, accuracy: 1e-12)

        guard let result = solveSingleMuscle(torque: 0.0, jointVelocity: 0.0) else {
            XCTFail("Solver returned nil")
            return
        }
        XCTAssertEqual(result.activations[0].doubleValue, solver.minActivation, accuracy: 1e-3,
                       "With no torque to produce, activation rests on the optimizer's floor")
    }

    // MARK: - Helpers

    /// Single-muscle / single-DOF rig. With these numbers the muscle sits at
    /// normalized fiber length 1.0 (peak of the force-length curve) and
    /// V_max = 10·l_opt = 1.0 m/s, so a joint velocity of 20 rad/s through a
    /// 0.05 m moment arm is exactly max shortening.
    private enum Rig {
        static let muscleName = "testmuscle_r"
        static let dofName = "knee_angle_r"
        static let momentArm = 0.05          // m
        static let maxForce = 1000.0         // N
        static let optimalFiberLength = 0.1  // m
        static let tendonSlackLength = 0.2   // m
        static let isometricLength = 0.3     // L_MT with normalized fiber length 1.0
        static let softPenalty = 100.0       // matches NimbleEngine's call site
    }

    /// - Parameter jointVelocity: `nil` passes an EMPTY array, which selects
    ///   the wall-clock finite-difference fallback.
    private func solveSingleMuscle(torque: Double,
                                   jointVelocity: Double?,
                                   muscleLength: Double = Rig.isometricLength,
                                   dt: Double = 1.0 / 60.0,
                                   using target: MuscleSolver? = nil) -> MuscleActivationResult? {
        let velocities: [NSNumber]
        if let jointVelocity {
            velocities = [NSNumber(value: jointVelocity)]
        } else {
            velocities = []
        }
        return (target ?? solver).solveReal(
            withJointTorques: [NSNumber(value: torque)],
            momentArms: [NSNumber(value: Rig.momentArm)],
            muscleNames: [Rig.muscleName],
            muscleLengths: [NSNumber(value: muscleLength)],
            maxForces: [NSNumber(value: Rig.maxForce)],
            optimalFiberLengths: [NSNumber(value: Rig.optimalFiberLength)],
            tendonSlackLengths: [NSNumber(value: Rig.tendonSlackLength)],
            pennationAngles: [NSNumber(value: 0.0)],
            jointVelocities: velocities,
            dofNames: [Rig.dofName],
            dt: dt,
            softPenalty: Rig.softPenalty
        )
    }

    /// Solves on a FRESH solver so length history and warm starts cannot
    /// couple the cases together.
    private func solveOnFreshSolver(jointVelocity: Double,
                                    torque: Double = 10.0) -> MuscleActivationResult? {
        return solveSingleMuscle(torque: torque,
                                 jointVelocity: jointVelocity,
                                 using: MuscleSolver())
    }

    private func forceForJointVelocity(_ dq: Double) -> Double {
        guard let result = solveOnFreshSolver(jointVelocity: dq) else {
            XCTFail("Solver returned nil for dq=\(dq)")
            return .nan
        }
        XCTAssertTrue(result.converged, "Solve did not converge for dq=\(dq)")
        return result.forces[0].doubleValue
    }

    private func activationForJointVelocity(_ dq: Double) -> Double {
        guard let result = solveOnFreshSolver(jointVelocity: dq) else {
            XCTFail("Solver returned nil for dq=\(dq)")
            return .nan
        }
        XCTAssertTrue(result.converged, "Solve did not converge for dq=\(dq)")
        return result.activations[0].doubleValue
    }

    private func loadMuscles() {
        let path = Bundle(for: type(of: self)).path(forResource: "Rajagopal2016", ofType: "osim")
            ?? Bundle.main.path(forResource: "Rajagopal2016", ofType: "osim")
        guard let path else {
            XCTFail("Cannot find Rajagopal2016.osim")
            return
        }
        let success = solver.loadMuscles(fromOsimPath: path)
        XCTAssertTrue(success)
    }
}
