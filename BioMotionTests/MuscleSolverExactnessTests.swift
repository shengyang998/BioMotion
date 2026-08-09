import XCTest
@testable import BioMotion

/// **Does the shipping solver return the answer to its own question?**
///
/// `WrappedMomentArmLeakTests` measures the product consequence of the gap, on
/// real geometry, and costs ten minutes of `WrapValidationHarness` setup. This
/// file is the cheap tripwire underneath it: no model, no fixture, no wrap
/// solver — one ill-conditioned muscle-sized QP, solved by `MuscleSolver` and by
/// `BoxQP`, which is the same machine-precision instrument that measurement
/// uses (`BoxQPTests` checks IT against a closed form, a dense Gaussian
/// elimination, coordinate descent and a direct optimality test).
///
/// # What was wrong, in one paragraph, because the numbers are the argument
///
/// OSQP terminates on `dual_res <= eps_abs + eps_rel·max(‖q‖∞, ‖Px‖∞, ‖Aᵀy‖∞)`.
/// This QP is `min ½aᵀ(εI + λAᵀA)a − λτᵀAa` with `ε = 0.01`, `λ = 100`, moment
/// arms in metres and forces in newtons, so MEASURED on the real leak rig
/// `‖q‖∞ = 8.3e6` — and the test therefore accepted a stationarity violation of
/// 8.3e3, with a median of 788 actually delivered over 2,791 solves. `A` has one
/// row per coordinate, so 68 of the rig's 80 Hessian eigenvalues are exactly
/// `ε = 0.01`: dividing the permitted violation by the curvature that resists it
/// gives `788/1.2e7 = 7e-5` of activation along the stiff directions and
/// `788/0.01 = 8e4` along the flat ones. The flat ones are "which synergist
/// carries the load", which is the entire content of a per-muscle comparison.
///
/// It was not a tolerance that could be tightened: at `eps = 1e-9` and 20,000
/// iterations the answer still sat 0.30 from the exact one. `scaling = 0` (Ruiz
/// equilibration rewrites `A = I` into a diagonal the ADMM step no longer
/// matches, and the flat subspace stops contracting) plus `polishing = 1` fixed
/// it. See the constants in `MuscleSolver.mm`.
///
/// # The bound, registered before it was measured
///
/// The product statistic is `100·(a_l − a_r)/mean`, so an absolute activation
/// error `δ` shows up as `≈ 200·δ/ā` percentage points, and the rig's median
/// activation is 0.132. The reopening bar every gate in this project uses is
/// `floor/5 = 1.617 pp`. Setting `200·δ/0.132 = 1.617` gives `δ = 1.07e-3`, so
/// the assertion below is `1e-3`: at that departure the solver cannot move a
/// published left/right figure by as much as the bar, whatever the geometry
/// does.
///
/// Scale of what was fixed, all against the same instrument: on the real
/// 520-muscle problem below, `0.10269 → 1.017e-04` over interior muscles; on
/// dumped leak-rig cells replayed offline, 0.21-0.34 → ~1e-07; on the seeded
/// problems here, worst 1.9e-07 (the "before" was not run on these seeds).
final class MuscleSolverExactnessTests: XCTestCase {

    /// See the doc above: `floor/5` in percentage points, converted back into
    /// absolute activation at the rig's own median activation.
    static let maximumActivationDeparture = 1e-3

    /// The problem generator is `BoxQPTests.problem` — deliberately shared, so
    /// the two solvers cannot be handed subtly different matrices, and so a
    /// future change to what "muscle-sized" means moves both at once.
    /// `scale = 90` is `R · F_max` in newton-metres: a 3 cm arm on a 3 kN muscle.
    func testTheShippedSolverReturnsTheExactMinimiserOfItsOwnObjective() throws {
        var worstDeparture = 0.0
        var rows: [String] = []
        for seed in [3, 5, 9, 17, 23] as [UInt64] {
            let (arms, torques, n, d) = BoxQPTests.problem(seed: seed, nMuscles: 80, nDOFs: 12,
                                                           scale: 90)
            let solver = MuscleSolver()
            // `MuscleSolver` multiplies every moment arm by
            // `F_max · f_AL(L̃) · f_FV(Ṽ) · cos α`. Held at `l_opt + l_Ts` with zero
            // pennation and zero velocity all three multipliers are exactly 1, so
            // passing `F_max = 1` makes the QP's `A` exactly `arms` and the two
            // solvers are given the same objective. Asserted below, not assumed.
            let names = (0..<n).map { "m\($0)" }
            let result = try XCTUnwrap(solver.solveReal(
                withJointTorques: torques.map(NSNumber.init(value:)),
                momentArms: arms.map(NSNumber.init(value:)),
                muscleNames: names,
                muscleLengths: names.map { _ in NSNumber(value: 0.2) },
                maxForces: names.map { _ in NSNumber(value: 1.0) },
                optimalFiberLengths: names.map { _ in NSNumber(value: 0.1) },
                tendonSlackLengths: names.map { _ in NSNumber(value: 0.1) },
                pennationAngles: names.map { _ in NSNumber(value: 0.0) },
                jointVelocities: (0..<d).map { _ in NSNumber(value: 0.0) },
                dofNames: (0..<d).map { "q\($0)" },
                dt: 1.0 / 30.0,
                softPenalty: 100.0))
            XCTAssertTrue(result.converged, "seed \(seed)")
            for index in 0..<n {
                let a = result.activations[index].doubleValue
                guard a > 0 else { continue }
                XCTAssertEqual(result.forces[index].doubleValue / a, 1.0, accuracy: 1e-12,
                               "the force scale must be exactly F_max here, or BoxQP is being "
                               + "compared against a different objective")
            }

            let exact = BoxQP.solve(arms: arms, nMuscles: n, nDOFs: d, torques: torques,
                                    lower: solver.minActivation, upper: MuscleSolver.maxActivation)
            XCTAssertLessThan(exact.kktResidual, 1e-9,
                              "the instrument has to reach a KKT point before it can judge "
                              + "anything")
            var departure = 0.0
            for index in 0..<n {
                departure = Swift.max(departure,
                                      abs(result.activations[index].doubleValue
                                          - exact.activations[index]))
            }
            worstDeparture = Swift.max(worstDeparture, departure)
            rows.append(String(format: "seed=%d departure=%.3e solve_ms=%.3f exact_kkt=%.2e "
                               + "at_lower=%d at_upper=%d", Int(seed), departure,
                               result.solveTimeMs, exact.kktResidual,
                               exact.activeAtLower, exact.activeAtUpper))
        }
        print("QP-EXACTNESS-METRIC \(rows.joined(separator: " | ")) "
              + "worst_departure=\(worstDeparture) bound=\(Self.maximumActivationDeparture)")
        XCTAssertLessThan(worstDeparture, Self.maximumActivationDeparture,
                          "the shipping solver is \(worstDeparture) from the exact minimiser of "
                          + "the objective it was given, which is \(200 * worstDeparture / 0.132) "
                          + "pp of a published left/right figure at the rig's median activation")
    }

    /// **THE SHIPPING PROBLEM, not a rig.** `WrappedMomentArmLeakTests` measures
    /// 80 muscles over 12 coordinates because that is what a bilateral leg rig
    /// is; the app solves 520 muscles over `FullBody.osim`'s coordinates, and
    /// STATUS.md's next-step 25 records that the shipping problem's own slack was
    /// an INFERENCE from the small one. This makes it a number.
    ///
    /// The moment arms are the real ones — `MomentArmComputer` at the neutral
    /// pose, wraps solved — and only `τ` is constructed: `τ = A_eff · 0.1`, so a
    /// feasible interior answer is known to exist. Inverse dynamics never touches
    /// a moment arm, so a constructed `τ` tests the same QP the app solves.
    ///
    /// **Why the coordinate list is filtered here.** `MuscleSolver` drops
    /// coordinates that are `<locked>` or that no muscle crosses
    /// (`kMomentArmFloor`), so handing `BoxQP` all of them would compare two
    /// different objectives. The rows are filtered by the same two documented
    /// rules BEFORE the solve, which makes the solver's own filter a no-op and
    /// both solvers see the identical matrix.
    func testTheShippedSolverIsExactOnTheRealFiveHundredMuscleProblem() throws {
        let bundle = Bundle(for: type(of: self))
        let path = try XCTUnwrap(bundle.path(forResource: "FullBody", ofType: "osim"))
        let bridge = NimbleBridge()
        XCTAssertTrue(bridge.loadModel(fromPath: path))
        let computer = MomentArmComputer()
        XCTAssertTrue(computer.parseMusclePaths(fromOsimPath: path, from: bridge))
        let solver = MuscleSolver()
        XCTAssertTrue(solver.loadMuscles(fromOsimPath: path))

        let allDOFs = bridge.dofNames
        let angles = allDOFs.map { _ in NSNumber(value: 0.0) }
        let flat = try XCTUnwrap(computer.computeMomentArms(withJointAngles: angles,
                                                            dofNames: allDOFs))
        let nMuscles = computer.numMuscles
        XCTAssertEqual(flat.count, nMuscles * allDOFs.count)
        XCTAssertGreaterThan(nMuscles, 500, "this is meant to be the shipped model")

        // The two documented exclusions, applied here so both solvers agree.
        let locked = Set(solver.lockedCoordinateNames)
        var keptColumns: [Int] = []
        for (column, name) in allDOFs.enumerated() {
            guard !locked.contains(name) else { continue }
            var peak = 0.0
            for m in 0..<nMuscles {
                peak = Swift.max(peak, abs(flat[m * allDOFs.count + column].doubleValue))
            }
            if peak > 1e-6 { keptColumns.append(column) }
        }
        let dofNames = keptColumns.map { allDOFs[$0] }
        let d = dofNames.count
        var arms = [Double](repeating: 0, count: nMuscles * d)
        for m in 0..<nMuscles {
            for (slot, column) in keptColumns.enumerated() {
                arms[m * d + slot] = flat[m * allDOFs.count + column].doubleValue
            }
        }

        // Isometric, zero pennation, zero velocity ⇒ every Hill multiplier is 1,
        // so `A_eff = R · diag(F_max)` — asserted below against the solver's own
        // returned forces rather than assumed.
        let maxForces = computer.maxIsometricForces.map { $0.doubleValue }
        let optimal = computer.optimalFiberLengths.map { $0.doubleValue }
        let slack = computer.tendonSlackLengths.map { $0.doubleValue }
        var scaled = [Double](repeating: 0, count: nMuscles * d)
        for m in 0..<nMuscles {
            for j in 0..<d { scaled[m * d + j] = arms[m * d + j] * maxForces[m] }
        }
        var torques = [Double](repeating: 0, count: d)
        for m in 0..<nMuscles {
            for j in 0..<d { torques[j] += scaled[m * d + j] * 0.1 }
        }

        let result = try XCTUnwrap(solver.solveReal(
            withJointTorques: torques.map(NSNumber.init(value:)),
            momentArms: arms.map(NSNumber.init(value:)),
            muscleNames: computer.muscleNames,
            muscleLengths: (0..<nMuscles).map { NSNumber(value: optimal[$0] + slack[$0]) },
            maxForces: maxForces.map(NSNumber.init(value:)),
            optimalFiberLengths: optimal.map(NSNumber.init(value:)),
            tendonSlackLengths: slack.map(NSNumber.init(value:)),
            pennationAngles: (0..<nMuscles).map { _ in NSNumber(value: 0.0) },
            jointVelocities: (0..<d).map { _ in NSNumber(value: 0.0) },
            dofNames: dofNames, dt: 1.0 / 30.0, softPenalty: 100.0))
        XCTAssertTrue(result.converged)
        var worstScale = 0.0
        for m in 0..<nMuscles where result.activations[m].doubleValue > 0 {
            let implied = result.forces[m].doubleValue / result.activations[m].doubleValue
            worstScale = Swift.max(worstScale, abs(implied - maxForces[m]) / maxForces[m])
        }
        XCTAssertLessThan(worstScale, 1e-9,
                          "the force scale must be exactly F_max or the two solvers are being "
                          + "given different objectives")

        let started = Date()
        let exact = BoxQP.solve(arms: scaled, nMuscles: nMuscles, nDOFs: d, torques: torques,
                                lower: solver.minActivation, upper: MuscleSolver.maxActivation,
                                maxIterations: 4000)
        let exactSeconds = Date().timeIntervalSince(started)
        // TWO departures, because they mean different things. A muscle the exact
        // solution puts ON a bound is one the panel never compares — `isSaturated`
        // and `isAtActivationFloor` screen it out — so its error cannot reach a
        // published number. The INTERIOR departure is the one that can.
        var departure = 0.0
        var interiorDeparture = 0.0
        var worstMuscle = ""
        var worstIsInterior = false
        var interior: [Double] = []
        for m in 0..<nMuscles {
            let gap = abs(result.activations[m].doubleValue - exact.activations[m])
            let a = exact.activations[m]
            let isInterior = a > solver.minActivation + 1e-3
                && a < MuscleSolver.maxActivation - 1e-3
            if gap > departure {
                departure = gap
                worstMuscle = computer.muscleNames[m]
                worstIsInterior = isInterior
            }
            if isInterior {
                interior.append(a)
                interiorDeparture = Swift.max(interiorDeparture, gap)
            }
        }
        let median = interior.sorted()[max(0, interior.count / 2)]
        print("QP-EXACTNESS-METRIC full_body muscles=\(nMuscles) coordinates_kept=\(d)"
              + "/\(allDOFs.count) departure=\(departure) on=\(worstMuscle) "
              + "worst_is_interior=\(worstIsInterior) interior_departure=\(interiorDeparture) "
              + "pp_at_median_activation=\(200 * departure / median) "
              + "interior_pp=\(200 * interiorDeparture / median) "
              + "median_interior_activation=\(median) interior=\(interior.count) "
              + "exact_kkt=\(exact.kktResidual) exact_iterations=\(exact.iterations) "
              + "exact_seconds=\(exactSeconds) osqp_ms=\(result.solveTimeMs) "
              + "at_lower=\(exact.activeAtLower) at_upper=\(exact.activeAtUpper)")
        XCTAssertLessThan(exact.kktResidual, 1e-6,
                          "the instrument must reach a KKT point on this problem too")
        // Both quantities, at the SAME registered bound. The interior one is the
        // one that can reach a published number and it has ~10× headroom
        // (1.02e-04 measured); the overall one includes a muscle the exact solve
        // pins to a bound and has ~1.7× (5.88e-04). If the overall assertion ever
        // fails while the interior one holds, that is a fact about bound-pinned
        // muscles — which `isSaturated`/`isAtActivationFloor` already screen out
        // of every comparison — and it must be reported, not relaxed away.
        XCTAssertLessThan(interiorDeparture, Self.maximumActivationDeparture,
                          "over the muscles a comparison could ever read, the shipping solver is "
                          + "\(interiorDeparture) from the exact minimiser of its own objective, "
                          + "i.e. \(200 * interiorDeparture / median) pp of a left/right figure")
        XCTAssertLessThan(departure, Self.maximumActivationDeparture,
                          "on the problem the app actually solves, the shipping solver is "
                          + "\(departure) from the exact minimiser of its own objective")
    }

    /// **The mechanism, isolated.** The defect was never the size of the residual
    /// — it was that the residual is compared against a tolerance scaled by data
    /// that is six orders of magnitude larger than the curvature holding the
    /// answer in place. This checks the two halves that make that true of THIS
    /// problem, so a future change of `ε`, `λ` or the force units cannot quietly
    /// remove the reason the settings are what they are.
    func testTheObjectiveIsFlatInMostDirectionsAndHugeInItsData() {
        let (arms, torques, n, d) = BoxQPTests.problem(seed: 3, nMuscles: 80, nDOFs: 12, scale: 90)
        // ‖q‖∞ with q = −λAᵀτ.
        var qInf = 0.0
        for m in 0..<n {
            var dot = 0.0
            for j in 0..<d { dot += arms[m * d + j] * torques[j] }
            qInf = Swift.max(qInf, abs(100 * dot))
        }
        // The flat subspace: `AᵀA` has rank at most `d`, so at least `n − d`
        // eigenvalues of `εI + λAᵀA` are exactly ε.
        let flatDirections = n - d
        let permitted = 1e-3 + 1e-3 * qInf
        print("QP-EXACTNESS-METRIC conditioning q_inf=\(qInf) flat_directions=\(flatDirections)"
              + "/\(n) osqp_permitted_dual_residual=\(permitted) "
              + "activation_error_that_permits_along_a_flat_direction=\(permitted / 0.01)")
        XCTAssertGreaterThan(qInf, 1e5,
                             "if the data ever stops being enormous, the relative termination "
                             + "test stops being the hazard this file documents")
        XCTAssertGreaterThan(flatDirections, n / 2,
                             "and if the muscle set ever stops being redundant, there is no flat "
                             + "subspace for the error to hide in")
    }
}
